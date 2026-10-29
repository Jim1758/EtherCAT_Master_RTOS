#pragma once
#include "NCSpindleCodes.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// EDM287 architecture only. This numerical controller has NO RT, PDO, PLC,
// NC-dispatch or AxisContext connection. Its outputs are proposals, never
// proof that a motor stopped or that isHomed was changed. The future adapter
// must bind exact AUTO program authority and final-send receipts to Input.
namespace NCRotarySpindle
{
constexpr unsigned Count=3U;
constexpr unsigned RequiredSamples=200U;
constexpr double CycleSeconds=0.00025;
constexpr bool RuntimeConnected=false;

struct Scope
{
    std::uint64_t runToken=0ULL,epoch=0ULL,ownerGeneration=0ULL;
    bool Valid() const noexcept {return runToken&&epoch&&ownerGeneration;}
    bool Same(const Scope& other) const noexcept
    {return runToken==other.runToken&&epoch==other.epoch&&ownerGeneration==other.ownerGeneration;}
};
enum class Lifecycle : std::uint8_t {Run,Hold,End,Alarm,Reset,AuthorityLost};
enum class Phase : std::uint8_t {Stopped,Ramping,Running,Stopping,Held,Fault};
enum class Code : std::uint8_t
{
    None,Accepted,NotInitialized,ScopeMismatch,NotRunning,InvalidCommand,
    AxisUnavailable,AxisPositioning,InvalidConfig,Overspeed,StaleRequest,
    StopNotProven,InvalidSource,DiscontinuousInput,Alarm,Reset,AuthorityLost
};
struct Config
{
    unsigned axisSlot=0U;
    bool enabled=false,rotary=false;
    double maximumRpm=0.0,accelerationRpmPerSecond=0.0,decelerationRpmPerSecond=0.0,stopDecelerationRpmPerSecond=0.0;
    // S is OUTPUT-shaft rev/min, not motor RPM. For native degrees the
    // adapter derives this as 360 * resolution_PPR / finalLead (gear included).
    double pulsesPerOutputRevolution=0.0;
    double stoppedToleranceRpm=0.0,readyToleranceRpm=0.0;
};
struct AxisSample
{
    double actualRpm=0.0;
    bool sourceValid=false,positioningActive=false,finalZeroSent=false;
    std::uint64_t zeroSendTick=0ULL,zeroSendRequestSequence=0ULL;
};
struct Input
{
    Scope scope{};
    std::uint64_t tick=0ULL,physicalSequence=0ULL,startSequence=0ULL;
    Lifecycle lifecycle=Lifecycle::AuthorityLost;
    // Run is a formally committed MEMORY program, not MDI/MANUAL/EDIT.
    bool pdoValid=false,configurationCurrent=false,memoryProgram=false,programCommitted=false;
    std::array<AxisSample,Count> axis{};
};
struct Request
{
    Scope scope{};
    std::uint64_t sequence=0ULL; // monotonic independently for each spindle
    int mCode=-1;
    bool hasSpeed=false;
    double rpm=0.0;             // unsigned, explicit S required on each start
};
struct AxisOutput
{
    Phase phase=Phase::Stopped;
    double desiredRpm=0.0,commandRpm=0.0,pulsesPerSecond=0.0,lastSpeedRpm=0.0;
    std::uint64_t requestSequence=0ULL,acknowledgedSequence=0ULL;
    bool zeroDemand=true,stopped=false,ready=false,ownsAxis=false;
    bool invalidateHome=false,referenceInvalidated=false;
};
struct Snapshot
{
    Scope scope{};
    std::uint64_t tick=0ULL;
    std::array<AxisOutput,Count> axis{};
    Code admission=Code::None,fault=Code::None;
    int admissionAxis=-1;
    bool activeProgram=false,holdComplete=false,programComplete=false,forceZero=true;
};
inline bool Positive(double value) noexcept {return std::isfinite(value)&&value>0.0;}
inline bool ValidConfig(const Config& c,unsigned) noexcept
{
    if(c.axisSlot>=8U)return false;
    if(!c.enabled)return true;
    if(!c.rotary||!Positive(c.maximumRpm)||!Positive(c.accelerationRpmPerSecond)||
        !Positive(c.decelerationRpmPerSecond)||!Positive(c.stopDecelerationRpmPerSecond)||!Positive(c.pulsesPerOutputRevolution)||
        !Positive(c.stoppedToleranceRpm)||!Positive(c.readyToleranceRpm)||
        c.stoppedToleranceRpm>=c.maximumRpm||c.readyToleranceRpm>=c.maximumRpm)return false;
    const double maximumPps=c.maximumRpm/60.0*c.pulsesPerOutputRevolution;
    return Positive(maximumPps)&&maximumPps<=static_cast<double>((std::numeric_limits<std::int32_t>::max)())&&
        Positive(c.accelerationRpmPerSecond*CycleSeconds)&&Positive(c.decelerationRpmPerSecond*CycleSeconds)&&
        Positive(c.stopDecelerationRpmPerSecond*CycleSeconds);
}

class Controller
{
public:
    bool Begin(const Scope& scope,const std::array<Config,Count>& config,const Input& input) noexcept
    {
        if(m_active||!scope.Valid()||scope.runToken<=m_highWaterRun||!scope.Same(input.scope)||
            input.lifecycle!=Lifecycle::Run||!input.tick||!input.physicalSequence||
            !input.pdoValid||!input.configurationCurrent||!input.memoryProgram||!input.programCommitted)return false;
        for(unsigned i=0U;i<Count;++i){
            if(!ValidConfig(config[i],i))return false;
            for(unsigned j=0U;j<i;++j)if(config[i].enabled&&config[j].enabled&&config[i].axisSlot==config[j].axisSlot)return false;
            if(config[i].enabled&&(!input.axis[i].sourceValid||input.axis[i].positioningActive||
                !std::isfinite(input.axis[i].actualRpm)||
                std::fabs(input.axis[i].actualRpm)>config[i].stoppedToleranceRpm))return false;
        }
        std::array<bool,Count> invalidated{};
        for(unsigned i=0U;i<Count;++i)invalidated[i]=m_axis[i].out.referenceInvalidated;
        m_scope=scope;m_config=config;m_highWaterRun=scope.runToken;
        m_axis={};m_snapshot={};m_snapshot.scope=scope;m_snapshot.activeProgram=true;m_snapshot.forceZero=false;
        m_tick=input.tick;m_physicalSequence=input.physicalSequence;
        m_hold=m_end=m_resumePending=false;m_active=true;m_initialized=true;
        m_startFloor=input.startSequence;
        for(unsigned i=0U;i<Count;++i){
            m_axis[i].lastZeroDemandTick=input.tick;
            m_axis[i].out.referenceInvalidated=invalidated[i];
            if(!config[i].enabled){m_axis[i].out.stopped=true;m_axis[i].zeroSamples=RequiredSamples;}
        }
        Publish();return true;
    }

    // A premature START is consumed as a floor. Reusing its sequence after
    // standstill cannot restart a spindle. This only stages a restart intent;
    // the next fresh Run sample must revalidate every source and scope.
    bool TryResume(const Scope& scope,std::uint64_t startSequence) noexcept
    {
        if(!m_active||!scope.Same(m_scope)||!m_hold||m_end||!startSequence||startSequence<=m_startFloor)return false;
        m_startFloor=startSequence;
        if(!m_snapshot.holdComplete||m_resumePending)return false;
        m_resumePending=true;return true;
    }

    const Snapshot& Step(const Input& input,const Request* request=nullptr) noexcept
    {
        std::array<Request,Count> requests{};
        std::array<bool,Count> present{};
        if(request){
            const auto code=NCSpindleCodes::Decode(request->mCode);
            const unsigned spindle=code.valid?code.spindle:0U;
            requests[spindle]=*request;present[spindle]=true;
        }
        return StepBatch(input,requests,present);
    }

    // EDM289: the RT adapter consumes at most one immutable request per
    // spindle during a single physical input tick. In particular all mapped
    // HOLD/End stop packets must share that tick; separate Step calls would
    // counterfeit continuity or delay one spindle's stop behind another.
    const Snapshot& StepBatch(const Input& input,const std::array<Request,Count>& requests,
        const std::array<bool,Count>& present) noexcept
    {
        for(auto& axis:m_axis)axis.out.invalidateHome=false;
        m_snapshot.admission=Code::None;m_snapshot.admissionAxis=-1;
        if(!m_initialized){m_snapshot.admission=Code::NotInitialized;return m_snapshot;}
        if(!m_active){
            for(bool pending:present)if(pending)m_snapshot.admission=Code::NotRunning;
            Publish();return m_snapshot;
        }
        Code failure=Code::None;
        if(!m_scope.Same(input.scope))failure=Code::ScopeMismatch;
        else if(input.lifecycle==Lifecycle::Alarm)failure=Code::Alarm;
        else if(input.lifecycle==Lifecycle::Reset)failure=Code::Reset;
        else if((input.lifecycle!=Lifecycle::Run&&input.lifecycle!=Lifecycle::Hold&&input.lifecycle!=Lifecycle::End)||
            !input.memoryProgram||!input.programCommitted)failure=Code::AuthorityLost;
        else if(m_tick==(std::numeric_limits<std::uint64_t>::max)()||
            m_physicalSequence==(std::numeric_limits<std::uint64_t>::max)()||
            input.tick!=m_tick+1ULL||input.physicalSequence!=m_physicalSequence+1ULL)failure=Code::DiscontinuousInput;
        else if(!input.pdoValid||!input.configurationCurrent)failure=Code::InvalidSource;
        for(unsigned i=0U;failure==Code::None&&i<Count;++i)if(m_config[i].enabled&&
            (!input.axis[i].sourceValid||!std::isfinite(input.axis[i].actualRpm)||
                (input.axis[i].positioningActive&&m_axis[i].out.ownsAxis)))
            failure=input.axis[i].positioningActive?Code::AxisPositioning:Code::InvalidSource;
        for(unsigned i=0U;failure==Code::None&&i<Count;++i)if(m_config[i].enabled&&m_axis[i].out.ownsAxis&&
            std::fabs(input.axis[i].actualRpm)-m_config[i].maximumRpm>m_config[i].readyToleranceRpm)failure=Code::Overspeed;
        if(failure!=Code::None){Contain(failure);Publish();return m_snapshot;}
        m_tick=input.tick;m_physicalSequence=input.physicalSequence;
        if((!m_hold||!m_snapshot.holdComplete)&&input.startSequence>m_startFloor)m_startFloor=input.startSequence;

        for(unsigned i=0U;i<Count;++i)ObserveZero(i,input);
        if(input.lifecycle==Lifecycle::End&&!m_end){
            m_end=true;m_hold=false;m_resumePending=false;
            for(auto& axis:m_axis){axis.savedRpm=axis.pendingRpm=0.0;axis.out.lastSpeedRpm=0.0;Stop(axis);}
        }else if(input.lifecycle==Lifecycle::Hold&&!m_end&&!m_hold){
            m_hold=true;m_resumePending=false;
            for(auto& axis:m_axis){axis.savedRpm=axis.pendingRpm!=0.0?axis.pendingRpm:axis.out.desiredRpm;
                axis.pendingRpm=0.0;Stop(axis);}
        }
        if(m_resumePending&&input.lifecycle==Lifecycle::Run&&m_hold&&!m_end){
            m_resumePending=false;
            bool stillStopped=true;
            for(unsigned i=0U;i<Count;++i)if(m_config[i].enabled&&m_axis[i].zeroSamples<RequiredSamples)stillStopped=false;
            if(stillStopped){m_hold=false;
                for(unsigned i=0U;i<Count;++i){auto& axis=m_axis[i];
                    if(axis.savedRpm!=0.0)Start(axis,axis.savedRpm);
                    else {axis.out.phase=Phase::Stopped;axis.out.ownsAxis=false;}
                    axis.savedRpm=0.0;
                }
            }
        }
        for(unsigned i=0U;i<Count;++i)if(present[i]){
            const auto code=NCSpindleCodes::Decode(requests[i].mCode);
            if(code.valid&&code.spindle!=i){
                m_snapshot.admission=Code::InvalidCommand;
                m_snapshot.admissionAxis=static_cast<int>(m_config[i].axisSlot);
            }else Accept(requests[i],input);
        }
        for(unsigned i=0U;i<Count;++i)Advance(i,input.axis[i]);
        bool allStopped=true;
        for(unsigned i=0U;i<Count;++i)if(m_config[i].enabled)allStopped=allStopped&&m_axis[i].out.stopped;
        if(m_end&&allStopped){m_active=false;m_snapshot.programComplete=true;m_snapshot.forceZero=true;}
        m_snapshot.holdComplete=m_hold&&allStopped;
        for(auto& axis:m_axis){
            axis.out.zeroDemand=axis.out.commandRpm==0.0;
            axis.lastZeroDemandTick=axis.out.zeroDemand?m_tick:0ULL;
            axis.lastZeroSequence=axis.out.requestSequence;
        }
        Publish();return m_snapshot;
    }

    const Snapshot& Current() const noexcept {return m_snapshot;}
    // EDM289: the physical adapter has already committed a stopped reference
    // handback. Retire this exact run without resetting lifetime watermarks or
    // retaining a pending/saved speed that could restart under a later owner.
    bool RetireStopped(const Scope& scope) noexcept
    {
        if(!m_active||!scope.Same(m_scope)||m_snapshot.fault!=Code::None||m_resumePending||m_hold||m_end)return false;
        for(unsigned i=0U;i<Count;++i)if(m_config[i].enabled){
            const auto& axis=m_axis[i];
            if(!axis.out.stopped||axis.out.phase!=Phase::Stopped||axis.out.commandRpm!=0.0||
                axis.out.desiredRpm!=0.0||axis.out.ownsAxis||axis.pendingRpm!=0.0||axis.savedRpm!=0.0||
                axis.zeroSamples<RequiredSamples)return false;
        }
        m_active=false;m_snapshot.forceZero=true;Publish();return true;
    }
    // Homing/reference establishment is an independent RT-owned transaction.
    // A latched fault does not grant positioning: the future safety adapter
    // must first establish a new stopped scope before this controller is reused.
    bool CanPosition(unsigned axisSlot,bool referenceEstablished) const noexcept
    {
        if(!m_initialized||axisSlot>=8U||!referenceEstablished||m_snapshot.fault!=Code::None||m_resumePending)return false;
        for(unsigned i=0U;i<Count;++i)if(m_config[i].enabled&&m_config[i].axisSlot==axisSlot){
            const auto& axis=m_axis[i];
            return axis.out.stopped&&!axis.out.ownsAxis&&axis.out.phase==Phase::Stopped&&
                axis.savedRpm==0.0&&axis.pendingRpm==0.0;
        }
        return true; // Not mapped to this controller; normal positioning guards still apply.
    }

private:
    struct Runtime
    {
        AxisOutput out{};
        double pendingRpm=0.0,savedRpm=0.0;
        unsigned zeroSamples=0U,readySamples=0U;
        std::uint64_t lastZeroDemandTick=0ULL,lastZeroSequence=0ULL;
    };
    Scope m_scope{};
    std::array<Config,Count> m_config{};
    std::array<Runtime,Count> m_axis{};
    Snapshot m_snapshot{};
    std::uint64_t m_highWaterRun=0ULL,m_tick=0ULL,m_physicalSequence=0ULL,m_startFloor=0ULL;
    bool m_initialized=false,m_active=false,m_hold=false,m_end=false,m_resumePending=false;

    static void Stop(Runtime& axis) noexcept
    {
        axis.out.desiredRpm=0.0;axis.out.phase=Phase::Stopping;
        axis.out.stopped=false;axis.out.ready=false;axis.zeroSamples=axis.readySamples=0U;
    }
    static void Start(Runtime& axis,double signedRpm) noexcept
    {
        axis.out.desiredRpm=signedRpm;axis.out.phase=Phase::Ramping;
        axis.out.stopped=axis.out.ready=false;axis.zeroSamples=axis.readySamples=0U;
        axis.out.invalidateHome=true;axis.out.referenceInvalidated=true;
        axis.out.ownsAxis=true;
    }
    void ObserveZero(unsigned i,const Input& input) noexcept
    {
        auto& axis=m_axis[i];if(!m_config[i].enabled)return;
        const auto& sample=input.axis[i];
        const bool zero=axis.out.commandRpm==0.0&&axis.lastZeroDemandTick&&
            axis.lastZeroDemandTick+1ULL==input.tick&&sample.finalZeroSent&&
            sample.zeroSendTick==axis.lastZeroDemandTick&&sample.zeroSendRequestSequence==axis.lastZeroSequence&&
            std::fabs(sample.actualRpm)<=m_config[i].stoppedToleranceRpm;
        axis.zeroSamples=zero?(axis.zeroSamples<RequiredSamples?axis.zeroSamples+1U:RequiredSamples):0U;
    }
    void Accept(const Request& request,const Input& input) noexcept
    {
        auto fail=[&](Code code) noexcept {m_snapshot.admission=code;};
        const auto code=NCSpindleCodes::Decode(request.mCode);
        if(!code.valid){fail(Code::InvalidCommand);return;}
        m_snapshot.admissionAxis=static_cast<int>(m_config[code.spindle].axisSlot);
        if(!request.scope.Same(m_scope)){fail(Code::ScopeMismatch);return;}
        if(input.lifecycle!=Lifecycle::Run||m_hold||m_end){fail(Code::NotRunning);return;}
        auto& axis=m_axis[code.spindle];const auto& config=m_config[code.spindle];
        if(!config.enabled||!config.rotary){fail(Code::AxisUnavailable);return;}
        if(input.axis[code.spindle].positioningActive){fail(Code::AxisPositioning);return;}
        if(!request.sequence||request.sequence<=axis.out.requestSequence){fail(Code::StaleRequest);return;}
        if((code.stop&&request.hasSpeed)||(!code.stop&&(!request.hasSpeed||!Positive(request.rpm)))){
            fail(Code::InvalidCommand);return;
        }
        if(!code.stop&&request.rpm>config.maximumRpm){fail(Code::Overspeed);return;}
        if(!code.stop&&axis.out.phase==Phase::Stopped&&axis.zeroSamples<RequiredSamples){fail(Code::StopNotProven);return;}
        axis.out.requestSequence=request.sequence;m_snapshot.admission=Code::Accepted;
        axis.out.ready=false;axis.readySamples=0U;
        if(code.stop){axis.pendingRpm=axis.savedRpm=0.0;axis.out.ownsAxis=true;Stop(axis);return;}
        const double desired=code.direction>0?request.rpm:-request.rpm;
        axis.out.lastSpeedRpm=request.rpm;
        if(axis.out.phase==Phase::Stopping){axis.pendingRpm=desired;axis.zeroSamples=0U;return;}
        if((axis.out.commandRpm>0.0&&desired<0.0)||(axis.out.commandRpm<0.0&&desired>0.0)||
            (axis.out.desiredRpm>0.0&&desired<0.0)||(axis.out.desiredRpm<0.0&&desired>0.0)){
            axis.pendingRpm=desired;Stop(axis);return;
        }
        if(axis.out.phase==Phase::Stopped)Start(axis,desired);
        else {axis.out.desiredRpm=desired;axis.out.phase=Phase::Ramping;}
    }
    void Advance(unsigned i,const AxisSample& sample) noexcept
    {
        auto& axis=m_axis[i];const auto& config=m_config[i];
        if(!config.enabled){axis.out.stopped=true;axis.out.phase=Phase::Stopped;return;}
        if(axis.out.phase==Phase::Stopping&&axis.out.commandRpm==0.0&&axis.zeroSamples>=RequiredSamples){
            if(axis.pendingRpm!=0.0&&!m_hold&&!m_end){const double pending=axis.pendingRpm;axis.pendingRpm=0.0;Start(axis,pending);}
            else {axis.out.phase=m_hold?Phase::Held:Phase::Stopped;axis.out.stopped=true;
                if(!m_hold)axis.out.ownsAxis=false;
                axis.out.acknowledgedSequence=axis.out.requestSequence;}
        }
        if(axis.out.phase==Phase::Stopped||axis.out.phase==Phase::Held){
            axis.out.stopped=axis.zeroSamples>=RequiredSamples;return;
        }
        const double target=axis.out.desiredRpm,current=axis.out.commandRpm;
        const bool speedingUp=std::fabs(target)>std::fabs(current);
        // Full stops (M-stop, HOLD, End and reversal's zero barrier) use the
        // dedicated stop ramp; same-direction speed reductions use Decel.
        const double slope=axis.out.phase==Phase::Stopping?config.stopDecelerationRpmPerSecond:
            speedingUp?config.accelerationRpmPerSecond:config.decelerationRpmPerSecond;
        const double step=slope*CycleSeconds;
        if(current<target)axis.out.commandRpm=(target-current<=step)?target:current+step;
        else if(current>target)axis.out.commandRpm=(current-target<=step)?target:current-step;
        axis.out.pulsesPerSecond=axis.out.commandRpm/60.0*config.pulsesPerOutputRevolution;
        if(axis.out.phase!=Phase::Stopping&&axis.out.commandRpm==target&&
            std::fabs(sample.actualRpm-target)<=config.readyToleranceRpm){
            if(axis.readySamples<RequiredSamples)++axis.readySamples;
            if(axis.readySamples==RequiredSamples){axis.out.phase=Phase::Running;axis.out.ready=true;
                axis.out.acknowledgedSequence=axis.out.requestSequence;}
        }else {axis.readySamples=0U;axis.out.ready=false;}
    }
    void Contain(Code fault) noexcept
    {
        m_snapshot.fault=fault;m_snapshot.forceZero=true;m_snapshot.holdComplete=false;
        m_active=false;m_hold=m_end=m_resumePending=false;
        for(auto& axis:m_axis){axis.pendingRpm=axis.savedRpm=0.0;axis.out.lastSpeedRpm=0.0;axis.out.desiredRpm=axis.out.commandRpm=axis.out.pulsesPerSecond=0.0;
            axis.out.phase=Phase::Fault;axis.out.ready=axis.out.stopped=false;axis.out.zeroDemand=true;
            axis.zeroSamples=axis.readySamples=0U;}
    }
    void Publish() noexcept
    {
        m_snapshot.tick=m_tick;m_snapshot.activeProgram=m_active;
        for(unsigned i=0U;i<Count;++i)m_snapshot.axis[i]=m_axis[i].out;
    }
};
}
