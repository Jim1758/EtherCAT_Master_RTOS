#pragma once
#include "NCSpindleSource.h"
#include "NCRotarySpindle.h"
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// EDM289: this NC-owner-thread broker stages immutable data; it is NOT a
// mailbox, servo adapter, motion permit, or physical stop detector. NCManager
// connects its packets to the separate Motion RT adapter. RuntimeConnected
// stays false here because this pure contract itself has no actuator. Only the
// RT adapter may publish coherent final-send/feedback receipts; NC cannot turn
// numeric RPM zero, a Boolean, or an ordinary group-drained flag into that proof.
namespace NCSpindleDispatch
{
constexpr bool RuntimeConnected=false;
using Config=std::array<NCRotarySpindle::Config,NCRotarySpindle::Count>;
enum class State : std::uint8_t {Inactive,Run,Hold,Resuming,End,AuthorityLost,Alarm,Reset};
enum class Kind : std::uint8_t {Command,HoldStop,Resume,EndStop,AuthorityLostStop,AlarmStop,ResetStop};
enum class Status : std::uint8_t
{
    Ok,Inactive,Busy,InvalidAuthority,StaleRun,InvalidConfig,InvalidRow,
    InvalidSource,AxisUnavailable,Overspeed,NotRunning,NotPrepared,
    CommitMismatch,InvalidCommit,StaleCommit,SequenceExhausted,
    NoPendingRequest,ReceiptMismatch,StaleReceipt,NotReady,StopNotProven,
    InvalidLifecycle,StaleLifecycle,LowerPriority,StaleStart,NotHeld
};
struct Authority
{
    NCRotarySpindle::Scope scope{};
    std::uint64_t cacheIdentity=0ULL,cacheGeneration=0ULL;
    std::uint64_t configGeneration=0ULL,configReceipt=0ULL;
    bool memoryProgram=false,autoOwner=false;
    bool Valid() const noexcept
    {return scope.Valid()&&cacheIdentity&&cacheGeneration&&configGeneration&&configReceipt&&memoryProgram&&autoOwner;}
    bool Same(const Authority& v) const noexcept
    {return scope.Same(v.scope)&&cacheIdentity==v.cacheIdentity&&cacheGeneration==v.cacheGeneration&&
        configGeneration==v.configGeneration&&configReceipt==v.configReceipt&&memoryProgram==v.memoryProgram&&autoOwner==v.autoOwner;}
};
struct Row
{
    // MEMORY uses frameIdentity 0; exact equality still binds the commit.
    std::uint64_t frameIdentity=0ULL,programCounter=0ULL,sourceLine=0ULL,dispatchId=0ULL;
    bool Valid() const noexcept
    {return sourceLine&&dispatchId&&programCounter!=(std::numeric_limits<std::uint64_t>::max)();}
    bool Same(const Row& v) const noexcept
    {return frameIdentity==v.frameIdentity&&programCounter==v.programCounter&&sourceLine==v.sourceLine&&dispatchId==v.dispatchId;}
};
// These facts must be copied from the successful formal CommitProgramBlock
// ledger and the matching Begin/EndProgramBlockMotionCapture, never inferred
// from ExecuteBlock returning true or from the row having only an M word.
struct FormalCommit
{
    Authority authority{};
    Row row{};
    std::uint64_t commitSequence=0ULL,targetProgramCounter=0ULL,committedProgramCounter=0ULL;
    unsigned ledgerMotionSegments=0U,capturedMotionSegments=0U;
    bool committed=false,ledgerFound=false,captureValid=false,captureOverflow=false;
    bool dispatchFailed=false,macroBypass=false;
};
// A nonzero Command/Resume is only an intent. Before accepting it, the RT
// consumer must independently revalidate current source/config/AUTO authority
// and its fresh all-axis stop/reversal barrier. A later ready receipt cannot
// retroactively authorize a stale restart. This broker cannot supply that gate.
struct Packet
{
    Authority authority{};
    Row row{}; // Only Command has an authored row / formal commit sequence.
    Kind kind=Kind::Command;
    unsigned spindle=0U,axisSlot=0U;
    std::uint64_t sequence=0ULL,commitSequence=0ULL,lifecycleSequence=0ULL,startSequence=0ULL;
    int mCode=-1;
    double signedRpm=0.0;
    bool Same(const Packet& v) const noexcept
    {return authority.Same(v.authority)&&row.Same(v.row)&&kind==v.kind&&spindle==v.spindle&&axisSlot==v.axisSlot&&
        sequence==v.sequence&&commitSequence==v.commitSequence&&lifecycleSequence==v.lifecycleSequence&&
        startSequence==v.startSequence&&mCode==v.mCode&&signedRpm==v.signedRpm;}
};
// RT writer contract: every sample in this window has the same request/scope,
// installed axis, coherent valid feedback, and validated final output. The
// adjacent tick/physical sequence counts are checked here; obtaining those
// samples and proving continuity remain duties of the future RT adapter.
struct Window
{
    std::uint64_t firstTick=0ULL,lastTick=0ULL,firstPhysicalSequence=0ULL,lastPhysicalSequence=0ULL;
    unsigned samples=0U;
    // RT-computed maxima over EVERY sample, not just the last feedback value.
    double maximumAbsActualRpm=0.0,maximumAbsTrackingErrorRpm=0.0;
    bool continuous=false,sourceValid=false,configurationCurrent=false,finalSendValidated=false;
    bool commandMatchedEverySample=false;
    bool Valid() const noexcept
    {return samples>=NCRotarySpindle::RequiredSamples&&continuous&&sourceValid&&configurationCurrent&&finalSendValidated&&commandMatchedEverySample&&
        std::isfinite(maximumAbsActualRpm)&&maximumAbsActualRpm>=0.0&&
        std::isfinite(maximumAbsTrackingErrorRpm)&&maximumAbsTrackingErrorRpm>=0.0&&
        firstTick&&lastTick>=firstTick&&firstPhysicalSequence&&lastPhysicalSequence>=firstPhysicalSequence&&
        lastTick-firstTick==static_cast<std::uint64_t>(samples-1U)&&
        lastPhysicalSequence-firstPhysicalSequence==static_cast<std::uint64_t>(samples-1U);}
};
// When AUTO authority has been revoked, only the RT safety writer can bind a
// cancellation's original scope to its new containment authority and final-zero
// ticket. These are asserted writer facts, not NC-generated stop evidence.
struct Containment
{
    std::uint64_t causalLifecycleSequence=0ULL,safetyEpoch=0ULL,safetyOwnerGeneration=0ULL,safetyStopTicket=0ULL;
    bool finalSafetyZeroValidated=false;
    bool ValidFor(const Packet& packet) const noexcept
    {return packet.lifecycleSequence&&causalLifecycleSequence==packet.lifecycleSequence&&safetyEpoch&&
        safetyOwnerGeneration&&safetyStopTicket&&finalSafetyZeroValidated;}
};
struct Receipt
{
    Packet packet{}; // Echo the complete accepted immutable packet.
    Window window{};
    Containment containment{};
    std::uint64_t publicationSequence=0ULL,appliedTick=0ULL,appliedRequestSequence=0ULL;
    std::uint64_t firstFinalSendTick=0ULL,lastFinalSendTick=0ULL,finalSendRequestSequence=0ULL;
    double actualRpm=0.0,commandRpm=0.0;
    bool accepted=false,authorityCurrent=false,ready=false,stopped=false;
    bool zeroSentEverySample=false,zeroDemand=false;
};

class Broker
{
public:
    // Run, request, lifecycle, receipt, RT tick and physical sequences remain
    // monotonic for this broker's lifetime; callers must not reset them at Begin.
    // Only authored dispatch/commit counters and the START floor are run-local.
    Status Begin(const Authority& authority,const Config& config,std::uint64_t startSequence=0ULL) noexcept
    {
        if(m_state!=State::Inactive||m_prepared||AnyPending())return Status::Busy;
        if(!authority.Valid())return Status::InvalidAuthority;
        if(authority.scope.runToken<=m_highWaterRun)return Status::StaleRun;
        bool enabled=false;
        for(unsigned i=0U;i<NCRotarySpindle::Count;++i){
            if(!NCRotarySpindle::ValidConfig(config[i],i))return Status::InvalidConfig;
            enabled=enabled||config[i].enabled;
            for(unsigned j=0U;j<i;++j)if(config[i].enabled&&config[j].enabled&&config[i].axisSlot==config[j].axisSlot)
                return Status::InvalidConfig;
        }
        if(!enabled)return Status::InvalidConfig;
        m_authority=authority;m_config=config;m_highWaterRun=authority.scope.runToken;
        m_state=State::Run;m_lastCommit=m_lastDispatch=0ULL;m_startFloor=startSequence;
        m_intent={};m_saved={};m_acknowledged={};m_packet={};m_prepared=false;return Status::Ok;
    }

    // Preparation is side-effect free outside this broker. There is no readable
    // request until the exact row's formal commit succeeds. No modal S exists.
    Status Prepare(const Authority& authority,const Row& row,const NCParsedBlock& parsed,const NCBlock& resolved) noexcept
    {
        if(m_state==State::Inactive)return Status::Inactive;
        if(!authority.Same(m_authority))return Status::InvalidAuthority;
        if(m_state!=State::Run)return Status::NotRunning;
        if(m_prepared||AnyPending())return Status::Busy;
        if(!row.Valid()||row.dispatchId<=m_lastDispatch)return Status::InvalidRow;
        NCSpindleSource::Words words{};
        if(NCSpindleSource::ValidateResolved(parsed,resolved,words)!=NCSpindleSource::Status::Ok)return Status::InvalidSource;
        const auto& config=m_config[words.spindle];
        if(!config.enabled||!config.rotary)return Status::AxisUnavailable;
        if(!words.stop&&words.rpm>config.maximumRpm)return Status::Overspeed;
        m_staged={};m_staged.authority=authority;m_staged.row=row;m_staged.kind=Kind::Command;
        m_staged.spindle=words.spindle;m_staged.axisSlot=config.axisSlot;m_staged.mCode=words.mCode;
        m_staged.signedRpm=words.stop?0.0:static_cast<double>(words.direction)*words.rpm;
        m_prepared=true;return Status::Ok;
    }
    void CancelPrepared() noexcept {m_prepared=false;m_staged={};}
    // NC may abandon a staged session before it submitted any committed packet.
    // This is not a physical retire shortcut: published requests cannot use it.
    Status AbortUnpublished() noexcept
    {
        if(m_lastCommit||AnyPending())return Status::Busy;
        CancelPrepared();m_state=State::Inactive;m_intent={};m_saved={};m_acknowledged={};return Status::Ok;
    }


    Status Commit(const FormalCommit& facts) noexcept
    {
        if(!m_prepared)return Status::NotPrepared;
        // Any failed attempt consumes the prepared row. A caller must re-validate
        // source and identity rather than repair a receipt for a stale request.
        const Packet staged=m_staged;CancelPrepared();
        if(m_state!=State::Run)return Status::NotRunning;
        if(!facts.authority.Same(m_authority)||!facts.authority.Same(staged.authority)||!facts.row.Same(staged.row))
            return Status::CommitMismatch;
        if(!facts.committed||!facts.ledgerFound||!facts.captureValid||facts.captureOverflow||facts.dispatchFailed||facts.macroBypass||
            facts.ledgerMotionSegments||facts.capturedMotionSegments||
            facts.targetProgramCounter!=staged.row.programCounter||facts.committedProgramCounter!=facts.targetProgramCounter)
            return Status::InvalidCommit;
        if(!facts.commitSequence||facts.commitSequence<=m_lastCommit||facts.row.dispatchId<=m_lastDispatch)return Status::StaleCommit;
        const unsigned i=staged.spindle;
        if(!CanAdvance(i))return Status::SequenceExhausted;
        Packet packet=staged;packet.sequence=++m_requestHighWater[i];packet.commitSequence=facts.commitSequence;
        m_packet[i]=packet;m_pending[i]=true;m_acknowledged[i]=false;m_intent[i]=packet.signedRpm;
        m_lastCommit=facts.commitSequence;m_lastDispatch=facts.row.dispatchId;return Status::Ok;
    }

    // Copy only an outstanding immutable packet. This is deliberately not an
    // atomic mailbox; future NC/RT transport must preserve whole-packet coherence.
    bool ReadRequest(unsigned spindle,Packet& output) const noexcept
    {
        output={};if(spindle>=NCRotarySpindle::Count||!m_pending[spindle])return false;
        output=m_packet[spindle];return true;
    }
    Status Poll(const Receipt& receipt) noexcept
    {
        const unsigned i=receipt.packet.spindle;
        if(i>=NCRotarySpindle::Count||!m_pending[i])return Status::NoPendingRequest;
        const auto& packet=m_packet[i];
        if(!receipt.packet.Same(packet))return Status::ReceiptMismatch;
        if(!receipt.publicationSequence||receipt.publicationSequence<=m_receiptHighWater[i]||
            receipt.appliedTick<m_tickHighWater[i]||receipt.window.firstTick<=m_tickHighWater[i]||
            receipt.window.firstPhysicalSequence<=m_physicalHighWater[i])return Status::StaleReceipt;
        if(!receipt.accepted||receipt.appliedRequestSequence!=packet.sequence||!receipt.appliedTick||
            !receipt.window.Valid()||receipt.window.firstTick<=receipt.appliedTick||
            receipt.firstFinalSendTick!=receipt.window.firstTick-1ULL||
            receipt.lastFinalSendTick!=receipt.window.lastTick-1ULL||receipt.finalSendRequestSequence!=packet.sequence||
            !std::isfinite(receipt.actualRpm)||!std::isfinite(receipt.commandRpm))return Status::ReceiptMismatch;
        if(packet.signedRpm==0.0){
            const bool safetyCancellation=packet.kind==Kind::AlarmStop||packet.kind==Kind::ResetStop||packet.kind==Kind::AuthorityLostStop;
            // Only explicit safety cancellation may use a stopped receipt after
            // AUTO authority is lost. The adapter must bind that proof to this
            // original packet and its final safety-zero output, never a new run.
            if((!receipt.authorityCurrent&&(!safetyCancellation||!receipt.containment.ValidFor(packet)))||
                !receipt.stopped||!receipt.zeroDemand||!receipt.zeroSentEverySample||receipt.commandRpm!=0.0||
                receipt.window.maximumAbsActualRpm>m_config[i].stoppedToleranceRpm||
                receipt.window.maximumAbsActualRpm<std::fabs(receipt.actualRpm))return Status::StopNotProven;
        }else if(!receipt.authorityCurrent||!receipt.ready||receipt.commandRpm!=packet.signedRpm||
            receipt.window.maximumAbsTrackingErrorRpm>m_config[i].readyToleranceRpm||
            receipt.window.maximumAbsTrackingErrorRpm<std::fabs(receipt.actualRpm-packet.signedRpm))return Status::NotReady;
        m_receiptHighWater[i]=receipt.publicationSequence;m_tickHighWater[i]=receipt.window.lastTick;
        m_physicalHighWater[i]=receipt.window.lastPhysicalSequence;m_pending[i]=false;m_acknowledged[i]=true;
        if(m_state==State::Resuming&&AllAcknowledged())m_state=State::Run;
        return Status::Ok;
    }

    // Control packets never counterfeit an NC authored M or formal commit.
    // Priorities only escalate: Reset > Alarm > AuthorityLost > End > Hold.
    // Call AuthorityLost when the current AUTO owner/epoch/config stops matching;
    // these zero-demand cancellation packets retain the original exact scope.
    Status ApplyLifecycle(State state,std::uint64_t eventSequence,std::uint64_t observedStartSequence=0ULL) noexcept
    {
        if(m_state==State::Inactive)return Status::Inactive;
        if(state!=State::Hold&&state!=State::End&&state!=State::AuthorityLost&&state!=State::Alarm&&state!=State::Reset)
            return Status::InvalidLifecycle;
        if(observedStartSequence>m_startFloor)m_startFloor=observedStartSequence;
        if(!eventSequence||eventSequence<=m_lifecycleHighWater)return Status::StaleLifecycle;
        if(Priority(state)<Priority(m_state))return Status::LowerPriority;
        m_lifecycleHighWater=eventSequence;
        if(state==m_state)return Status::Ok; // Repeated HOLD does not destroy its saved intent.
        if(!CanAdvanceAll())return Exhausted();
        CancelPrepared();
        if(state==State::Hold)m_saved=m_intent;
        else {m_saved={};m_intent={};}
        m_state=state;
        PublishControl(StopKind(state),eventSequence,0ULL,false);return Status::Ok;
    }

    // Every new START is consumed even while stop proof is incomplete. The
    // same button sequence cannot later trigger automatic restart. Restoring
    // Run also requires each new Resume packet's exact ready/stop receipt.
    Status TryResume(const Authority& authority,std::uint64_t startSequence) noexcept
    {
        if(!authority.Same(m_authority))return Status::InvalidAuthority;
        if(!startSequence||startSequence<=m_startFloor)return Status::StaleStart;
        m_startFloor=startSequence;
        if(m_state!=State::Hold)return Status::NotHeld;
        if(!AllAcknowledged())return Status::StopNotProven;
        if(!CanAdvanceAll())return Exhausted();
        m_state=State::Resuming;
        PublishControl(Kind::Resume,m_lifecycleHighWater,startSequence,true);m_saved={};return Status::Ok;
    }
    // Terminal cancellation is not a reusable run. Retiring it requires all
    // mapped axes' exact fresh physical stop receipts, including after faults.
    Status Retire() noexcept
    {
        if(m_state!=State::End&&m_state!=State::AuthorityLost&&m_state!=State::Alarm&&m_state!=State::Reset)
            return Status::InvalidLifecycle;
        if(!AllAcknowledged())return Status::StopNotProven;
        m_state=State::Inactive;m_pending={};m_acknowledged={};m_intent={};m_saved={};CancelPrepared();return Status::Ok;
    }
    State CurrentState() const noexcept {return m_state;}
    bool HoldComplete() const noexcept {return m_state==State::Hold&&AllAcknowledged();}
    bool HasPrepared() const noexcept {return m_prepared;}
    bool HasPending() const noexcept {return AnyPending();}

private:
    Authority m_authority{};
    Config m_config{};
    State m_state=State::Inactive;
    Packet m_staged{};
    std::array<Packet,NCRotarySpindle::Count> m_packet{};
    std::array<std::uint64_t,NCRotarySpindle::Count> m_requestHighWater{},m_receiptHighWater{},m_tickHighWater{},m_physicalHighWater{};
    std::array<bool,NCRotarySpindle::Count> m_pending{},m_acknowledged{};
    std::array<double,NCRotarySpindle::Count> m_intent{},m_saved{};
    std::uint64_t m_highWaterRun=0ULL,m_lastCommit=0ULL,m_lastDispatch=0ULL,m_lifecycleHighWater=0ULL,m_startFloor=0ULL;
    bool m_prepared=false;
    bool AnyPending() const noexcept {for(bool pending:m_pending)if(pending)return true;return false;}
    bool AllAcknowledged() const noexcept
    {for(unsigned i=0U;i<NCRotarySpindle::Count;++i)if(m_config[i].enabled&&(!m_acknowledged[i]||m_pending[i]))return false;return true;}
    bool CanAdvance(unsigned i) const noexcept {return m_requestHighWater[i]!=(std::numeric_limits<std::uint64_t>::max)();}
    bool CanAdvanceAll() const noexcept
    {for(unsigned i=0U;i<NCRotarySpindle::Count;++i)if(m_config[i].enabled&&!CanAdvance(i))return false;return true;}
    static unsigned Priority(State state) noexcept
    {
        switch(state){case State::Hold:return 1U;case State::End:return 2U;case State::AuthorityLost:return 3U;
        case State::Alarm:return 4U;case State::Reset:return 5U;default:return 0U;}
    }
    static Kind StopKind(State state) noexcept
    {
        switch(state){case State::Hold:return Kind::HoldStop;case State::End:return Kind::EndStop;
        case State::Alarm:return Kind::AlarmStop;case State::Reset:return Kind::ResetStop;default:return Kind::AuthorityLostStop;}
    }
    Status Exhausted() noexcept
    {CancelPrepared();m_pending={};m_acknowledged={};m_intent={};m_saved={};m_state=State::AuthorityLost;return Status::SequenceExhausted;}
    void PublishControl(Kind kind,std::uint64_t lifecycleSequence,std::uint64_t startSequence,bool resume) noexcept
    {
        for(unsigned i=0U;i<NCRotarySpindle::Count;++i){
            m_pending[i]=false;m_acknowledged[i]=!m_config[i].enabled;
            if(!m_config[i].enabled)continue;
            Packet packet{};packet.authority=m_authority;packet.kind=kind;packet.spindle=i;packet.axisSlot=m_config[i].axisSlot;
            packet.sequence=++m_requestHighWater[i];packet.lifecycleSequence=lifecycleSequence;packet.startSequence=startSequence;
            packet.signedRpm=resume?m_saved[i]:0.0;m_packet[i]=packet;m_pending[i]=true;
            if(resume)m_intent[i]=packet.signedRpm;
        }
    }
};
static_assert(std::is_trivially_copyable<Packet>::value,"Spindle dispatch packets must not own memory.");
static_assert(std::is_trivially_copyable<Receipt>::value,"Spindle dispatch receipts must not own memory.");
}
