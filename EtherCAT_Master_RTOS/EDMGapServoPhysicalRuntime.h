#pragma once
#include "EDMGapServoRuntime.h"
#include "EDMGapRTSource.h"
#include "EDMFlushPhysicalRuntime.h"
#include "EDMGapServoRecipeBinding.h"
#include "EDMGapServoFlushContract.h"
#include <cmath>
#include <cstdint>
#include <limits>

// EDM56: one RT owner produces bounded single-Z command references. This
// component has no axis setters, PDO access, discharge output or stop receipt.
// The caller must commit each exact candidate and retain the physical policy's
// final-output guards and genuine Stop/Disarm/Released evidence.
namespace EDM56
{
constexpr double CycleSeconds = .00025;
constexpr double MaximumMmS = .5;
constexpr double AccelerationMmS2 = MaximumMmS / .3;
constexpr std::uint64_t MaximumDurationUs = 30000000ULL;
enum class Reason : std::uint8_t
{ None, InvalidStart, Clock, Source, Identity, Bounds, Following, Kernel,
  Application, Timeout, Cancelled, Complete, PersistentShort, ExpectedStale };
enum class Phase : std::uint32_t
{ Advance, ShortRetreat, ClearReturn, ReturnToOrigin, OriginDwell, Terminal, OriginSettle };
// EDM59-FIX1-DIAG1: observation only. This bounded snapshot cannot grant a
// cycle, change a reference or replace the separate physical Stop receipt.
// sampledTick/sampledUs identify the last valid predicate evaluation, even
// when a later invalid observation ends the controller before this gate.
struct OriginSettleDiagnostic
{
    std::uint64_t enteredUs=0, sampledTick=0, sampledUs=0;
    double errorMm=0, pdoMmS=0, errorMinMm=0, errorMaxMm=0;
    double maxAbsPdoMmS=0, windowRangeMm=0;
    std::uint32_t blockerMask=0, observations=0, stableUs=0, stableTicks=0;
    std::uint32_t referenceRejectTicks=0, positionRejectTicks=0, pdoHighTicks=0, rangeResetCount=0;
    std::uint32_t bestStableUs=0, bestStableTicks=0;
};
static_assert(sizeof(OriginSettleDiagnostic)<=112U,"EDM59 origin diagnostics must remain bounded.");
struct Feedback
{
    std::uint64_t session=0, generation=0, candidateSequence=0, appliedSequence=0;
    std::uint64_t tick=0, nowUs=0, sourceSequence=0, sourceCapturedAtUs=0;
    std::uint64_t startedTick=0, startedUs=0, steps=0;
    std::uint32_t phase=0, cycles=0;
    std::uint8_t profile=0;
    Reason reason=Reason::None;
    EDM55::Fault kernelFault=EDM55::Fault::None;
    bool active=false, needFormalStop=false, terminal=false, completed=false;
    bool persistentShort=false, expectedStale=false, shortActive=false, feedInhibited=true;
    double originPulse=0, commandPulse=0, nativeVelocityPps=0;
    double rawVoltageV=0, filteredVoltageV=0, targetMmMin=0, servoMmMin=0, appliedMmMin=0;
    EDM57::Feedback flush{};
    OriginSettleDiagnostic originSettle{};
    constexpr bool PhysicalStopProven() const noexcept { return false; }
    constexpr bool DriveAcknowledged() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
};
struct Candidate
{
    std::uint64_t session=0, generation=0, sequence=0, tick=0, nowUs=0;
    double nativeTargetPulse=0, nativeVelocityPps=0;
    bool valid=false, needFormalStop=false;
    Reason reason=Reason::None;
};
class Controller
{
public:
    bool Active() const noexcept { return f_.active; }
    bool HasPendingCandidate() const noexcept { return pending_; }
    const Feedback& Snapshot() const noexcept { return f_; }

    // Templates accept the production EDM28 scope/config/observation without
    // including its header (which embeds this small telemetry structure).
    // Compatibility entry point owns a separate bounded SIM provider. Native
    // Motion uses the explicit binding/provider overloads exclusively.
    template<class S,class C,class O>
    bool Begin(const S& scope,const C& config,double firstOriginPulse,
        std::uint64_t generation,const O& observation) noexcept
    {
        const auto binding=EDM61::MakeBinding(scope,static_cast<unsigned>(config.profile),
            generation,observation.tick,observation.monotonicUs);
        if(!Begin(scope,config,firstOriginPulse,generation,observation,binding)) return false;
        if(!compatibilitySource_.Begin(binding)) { Fail(Reason::InvalidStart); return false; }
        compatibilityMode_=true; return true;
    }
    template<class S,class C,class O>
    bool Begin(const S& scope,const C& config,double firstOriginPulse,
        std::uint64_t generation,const O& observation,const EDM61::Binding& binding) noexcept
    {
        compatibilityMode_=false;
        if(!generation || generation<=sourceGenerationHighWater_)
        { Fail(Reason::InvalidStart); return false; }
        sourceGenerationHighWater_=generation;
        if(!EDM61::Valid(binding) || !EDM61::Same(binding,EDM61::MakeBinding(scope,
            static_cast<unsigned>(config.profile),generation,observation.tick,observation.monotonicUs)))
        { Fail(Reason::InvalidStart); return false; }
        const bool sameSession=f_.session!=0 && scope.session==f_.session;
        if(f_.active || pending_ || !generation || !scope.session ||
            (f_.session && scope.session<f_.session) ||
            (sameSession && (generation<=f_.generation || firstOriginPulse!=f_.originPulse)) ||
            (sameSession && f_.reason!=Reason::None && f_.reason!=Reason::Cancelled) ||
            !scope.runGeneration || !scope.cacheGeneration || !scope.dispatchGeneration ||
            !scope.axisMapGeneration || !scope.configGeneration || !scope.executionEpoch ||
            !scope.ownerGeneration || scope.owner!=1U ||
            (static_cast<unsigned>(config.profile)<35U || static_cast<unsigned>(config.profile)>52U) ||
            (EDM57::IsServoFlushProfile(static_cast<unsigned>(config.profile)) && !EDM60::ValidateConfig(config)) ||
            (!IsFlush(static_cast<unsigned>(config.profile)) && !EDM57::SameRecipe(config.flush,EDM57::Recipe{})) ||
            (IsFlush(static_cast<unsigned>(config.profile)) && (!EDM57::ValidateRecipeForProfile(config.flush,static_cast<unsigned>(config.profile)) ||
                (sameSession && !EDM57::SameRecipe(config.flush,flush_.FrozenRecipe())))) ||
            (IsBoundServo(static_cast<unsigned>(config.profile)) ?
                (!EDM59::ValidateRecipe(config.servoRecipe) ||
                    (sameSession && !EDM59::SameRecipe(config.servoRecipe,servoRecipe_))) :
                !EDM59::IsEmptyRecipe(config.servoRecipe)) ||
            config.axisIndex!=2U || config.feedMmMin!=30.0 || config.pdoCapMmS!=.5 ||
            config.targetHalfMm!=.25 || config.outerHalfMm!=.4 ||
            config.accelerationTimeSec!=.3 || config.decelerationTimeSec!=.3 ||
            !Finite(config.pulsePerMm) || config.pulsePerMm<1000.0 || config.pulsePerMm>1000000000.0 ||
            !Finite(firstOriginPulse) || !Finite(config.referenceOffsetPulse) ||
            !observation.tick || !observation.monotonicUs ||
            std::fabs(firstOriginPulse)+.4*config.pulsePerMm>2147483647.0 ||
            (config.hardwareSign!=1 && config.hardwareSign!=-1) ||
            (sameSession && (config.pulsePerMm!=scale_ || config.referenceOffsetPulse!=offset_ ||
                config.hardwareSign!=sign_ || static_cast<unsigned>(config.profile)!=f_.profile)))
        { Fail(Reason::InvalidStart); return false; }
        const bool retainedShort=sameSession && persistentIntent_;
        scope_={scope.session,scope.runGeneration,scope.cacheGeneration,scope.dispatchGeneration,
            scope.axisMapGeneration,scope.configGeneration,scope.executionEpoch,scope.ownerGeneration,scope.owner};
        servoRecipe_=config.servoRecipe;
        scale_=config.pulsePerMm; offset_=config.referenceOffsetPulse; sign_=config.hardwareSign;
        core_.Cancel(); core_=EDM55::Runtime{};
        f_=Feedback{}; f_.session=scope.session; f_.generation=generation;
        f_.profile=static_cast<std::uint8_t>(config.profile); f_.originPulse=firstOriginPulse;
        f_.commandPulse=observation.commandPulse; f_.startedTick=observation.tick;
        f_.startedUs=observation.monotonicUs; f_.tick=observation.tick; f_.nowUs=observation.monotonicUs;
        persistentIntent_=retainedShort; staleInjected_=false; pending_=false;
        lastTick_=lastUs_=zeroSinceUs_=originSinceUs_=0; lastDirection_=0;
        originStableSinceUs_=0; originStableTicks_=0; originSettleWasReturn_=false;
        originStableLow_=originStableHigh_=0.0;
        position_=(firstOriginPulse-observation.commandPulse)/scale_; velocity_=0;
        shortMinimum_=observation.actualPulse; sample_=EDM55::Sample{};
        if(!Facts(observation) || !observation.axisIdle ||
            std::fabs(observation.cmdVelocityPps)>1.0 || std::fabs(observation.logicalVelocityPps)>1.0 ||
            std::fabs(observation.targetVelocityPps)>1.0 || position_<-.001 || position_>.25)
        { Fail(Reason::InvalidStart); return false; }
        EDM55::Config model{}; model.referenceV=60.0; model.curve.deadbandV=.5;
        model.curve.maxFeedMmPerMin=model.curve.maxRetreatMmPerMin=model.curve.shortRetreatMmPerMin=30.0;
        if(IsBoundServo(f_.profile))
        {
            // Only the static installed curve and effective E8/E9 cross this
            // boundary. The SIM fixture retains its independent short guard.
            model.curve=servoRecipe_.curve; model.referenceV=servoRecipe_.referenceV;
            model.overridePercent=servoRecipe_.overridePercent;
        }
        model.filterTauUs=4000; model.maxAgeUs=5000; model.maxTickGapUs=2000;
        model.accelerationMmPerMinPerSecond=model.decelerationMmPerMinPerSecond=100.0;
        model.reversalDwellUs=5000;
        sourceBinding_=binding;
        if(!core_.Configure(model) || !core_.Start(binding.identity)) { Fail(Reason::InvalidStart); return false; }
        f_.active=true;
        if(IsFlush(f_.profile))
        {
            if(!flush_.Begin(config.flush,f_.profile,FlushReadback(observation),retainedShort))
            { Fail(Reason::InvalidStart); return false; }
            f_.flush=flush_.Snapshot();
        }
        SetPhase((position_!=0.0 || std::fabs(observation.actualPulse-firstOriginPulse)>.001*scale_ || retainedShort)?
            Phase::ReturnToOrigin:Phase::Advance);
        return true;
    }

    template<class O> Candidate Step(const O& o) noexcept
    {
        if(!f_.active) return View();
        if(!compatibilityMode_) return Fail(Reason::Source);
        return Step(o,compatibilitySource_);
    }
    template<class O,class Provider> Candidate Step(const O& o,Provider& source) noexcept
    {
        if(!f_.active) return View();
        if(pending_) return Fail(Reason::Application);
        f_.tick=o.tick; f_.nowUs=o.monotonicUs;
        // ServoStart is applied before trajectory dispatch in the same RT
        // cycle, so its first zero candidate may share Begin's clock. A first
        // candidate deferred to the next cycle is also allowed, but no skipped
        // cycle or stale admission window can earn catch-up motion.
        if(!lastTick_ && (o.tick<f_.startedTick || o.tick-f_.startedTick>1 ||
            o.monotonicUs<f_.startedUs || o.monotonicUs-f_.startedUs>2000)) return Fail(Reason::Clock);
        if(!o.tick || !o.monotonicUs || (lastTick_ &&
            (lastTick_==(std::numeric_limits<std::uint64_t>::max)() || o.tick!=lastTick_+1 ||
             o.monotonicUs<=lastUs_ || o.monotonicUs-lastUs_>2000))) return Fail(Reason::Clock);
        if(!Facts(o)) return Fail(factsReason_);
        if(f_.appliedSequence && (o.commandPulse!=f_.commandPulse || o.planningPulse!=f_.commandPulse ||
            o.cmdVelocityPps!=f_.nativeVelocityPps || o.logicalVelocityPps!=f_.nativeVelocityPps ||
            o.targetVelocityPps!=f_.nativeVelocityPps)) return Fail(Reason::Application);
        if(o.monotonicUs<f_.startedUs || o.monotonicUs-f_.startedUs>MaximumDurationUs)
            return Fail(Reason::Timeout);
        lastTick_=o.tick; lastUs_=o.monotonicUs;
        if(f_.steps==(std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::Clock);
        ++f_.steps;
        if(IsFlush(f_.profile)) return FlushStep(o,source);
        const double actual=(f_.originPulse-o.actualPulse)/scale_;
        Phase phase=static_cast<Phase>(f_.phase);
        if(phase==Phase::Advance && actual>=(IsBoundServo(f_.profile)?.03:.13))
        {
            SetPhase(Phase::ShortRetreat); phase=Phase::ShortRetreat;
            shortMinimum_=o.actualPulse; if(IsPersistent(f_.profile)) persistentIntent_=true;
        }
        if(phase==Phase::ShortRetreat)
        {
            if(o.actualPulse<shortMinimum_) shortMinimum_=o.actualPulse;
            if(IsNormal(f_.profile) && f_.shortActive &&
                o.actualPulse-shortMinimum_>=(IsBoundServo(f_.profile)?.005:.045)*scale_)
            { SetPhase(Phase::ClearReturn); phase=Phase::ClearReturn; }
        }
        if(IsStale(f_.profile) && phase==Phase::Advance &&
            actual>=(IsBoundServo(f_.profile)?.01:.03)) staleInjected_=true;
        const bool originCommand=position_==0.0 && velocity_==0.0;
        const bool originActual=std::fabs(actual)<=.001;
        // EDM59-FIX1: reaching the commanded endpoint can precede mechanical
        // settling. Keep the scoped VELOCITY reference active at FIRST_ARM's
        // exact origin so the normal servo PI can still correct late coast.
        // This stage earns no physical Stop/Disarm receipt.
        if(IsBoundServo(f_.profile) &&
            (phase==Phase::ShortRetreat || phase==Phase::ClearReturn || phase==Phase::ReturnToOrigin) &&
            originCommand)
        {
            originSettleWasReturn_=phase==Phase::ReturnToOrigin;
            originStableSinceUs_=0; originStableTicks_=0;
            f_.originSettle=OriginSettleDiagnostic{}; f_.originSettle.enteredUs=o.monotonicUs;
            SetPhase(Phase::OriginSettle); phase=Phase::OriginSettle;
        }
        if((phase==Phase::ShortRetreat || phase==Phase::ClearReturn || phase==Phase::ReturnToOrigin) &&
            originCommand && originActual)
        {
            if(IsPersistent(f_.profile) && persistentIntent_)
            {
                // A HOLD at the origin may restart a previously latched low
                // intent. Obtain real new-generation low samples and detector
                // dwell before exposing its terminal source evidence.
                if(f_.shortActive && sample_.valid && sample_.voltageV==20.0 &&
                    o.monotonicUs>=sample_.capturedAtUs && o.monotonicUs-sample_.capturedAtUs<=5000)
                    return TerminalWithSource(Reason::PersistentShort,o,source,20.0);
            }
            else if(phase!=Phase::ReturnToOrigin)
            {
                ++f_.cycles;
                if(f_.cycles>=6) return TerminalWithSource(Reason::Complete,o,source,
                    phase==Phase::ShortRetreat?20.0:50.0);
            }
            if(!(IsPersistent(f_.profile) && persistentIntent_))
            { SetPhase(Phase::OriginDwell); phase=Phase::OriginDwell; originSinceUs_=o.monotonicUs; }
        }
        if(phase==Phase::OriginDwell && o.monotonicUs-originSinceUs_>=50000)
        { SetPhase(Phase::Advance); phase=Phase::Advance; }
        const double voltage=phase==Phase::Advance?80.0:
            (phase==Phase::ShortRetreat ||
                ((phase==Phase::ReturnToOrigin || phase==Phase::OriginSettle) && persistentIntent_))?20.0:50.0;
        if(!ReadSource(o,source,voltage)) return Fail(Reason::Source);
        const EDM55::Output out=core_.Step(sample_,o.monotonicUs,o.tick);
        f_.sourceSequence=out.sequence; f_.sourceCapturedAtUs=out.capturedAtUs;
        f_.rawVoltageV=out.rawVoltageV; f_.filteredVoltageV=out.filteredVoltageV;
        f_.targetMmMin=out.targetMmPerMin; f_.servoMmMin=out.speedMmPerMin;
        f_.shortActive=out.shortActive; f_.feedInhibited=out.feedInhibited; f_.kernelFault=out.fault;
        if(!out.valid || out.fault!=EDM55::Fault::None)
            return IsStale(f_.profile) && staleInjected_ && out.fault==EDM55::Fault::Stale ?
                Terminal(Reason::ExpectedStale):Fail(Reason::Kernel);
        if(phase==Phase::OriginSettle && BoundOriginSettled(o))
        {
            if(IsPersistent(f_.profile) && persistentIntent_)
            {
                if(out.shortActive && out.feedInhibited && sample_.valid &&
                    sample_.voltageV==20.0 && out.rawVoltageV==20.0 &&
                    o.monotonicUs>=out.capturedAtUs && o.monotonicUs-out.capturedAtUs<=5000)
                    return Terminal(Reason::PersistentShort);
            }
            else
            {
                if(!originSettleWasReturn_ && ++f_.cycles>=6) return Terminal(Reason::Complete);
                SetPhase(Phase::OriginDwell); phase=Phase::OriginDwell; originSinceUs_=o.monotonicUs;
            }
        }
        double demand=out.speedMmPerMin/60.0;
        if(phase==Phase::OriginDwell || phase==Phase::OriginSettle) demand=0.0;
        // A tolerated sub-micron initial command above the origin is returned
        // toward it, not snapped. All regular profile directions follow GAP.
        if(phase==Phase::ReturnToOrigin && position_<0.0) demand=std::fabs(demand);
        return Plan(demand,o);
    }

    template<class O> bool Commit(const Candidate& value,bool applied,const O& o) noexcept
    {
        if(!pending_ || !Same(value,candidate_) || !applied ||
            o.tick!=value.tick || o.monotonicUs!=value.nowUs || !Facts(o) ||
            o.commandPulse!=value.nativeTargetPulse || o.planningPulse!=value.nativeTargetPulse ||
            o.cmdVelocityPps!=value.nativeVelocityPps || o.logicalVelocityPps!=value.nativeVelocityPps ||
            o.targetVelocityPps!=value.nativeVelocityPps)
        { Fail(Reason::Application); return false; }
        pending_=false; position_=candidatePosition_; velocity_=candidateVelocity_;
        f_.appliedSequence=value.sequence; f_.commandPulse=value.nativeTargetPulse;
        f_.nativeVelocityPps=value.nativeVelocityPps; f_.appliedMmMin=velocity_*60.0;
        if(velocity_==0.0) { if(!zeroSinceUs_) zeroSinceUs_=o.monotonicUs; }
        else { lastDirection_=velocity_>0?1:-1; zeroSinceUs_=0; }
        return true;
    }
    void Cancel() noexcept
    {
        core_.Cancel(); compatibilitySource_.Cancel(); pending_=false; f_.active=false; f_.needFormalStop=true;
        if(!f_.terminal && f_.reason==Reason::None) f_.reason=Reason::Cancelled;
        // Preserve committed velocity and terminal telemetry. Neither cancelling
        // nor requesting a Stop establishes a physical zero or stop receipt.
    }
private:
    struct Scope
    {
        std::uint64_t session,run,cache,dispatch,map,config;
        std::uint32_t epoch,ownerGeneration;
        std::uint8_t owner;
    } scope_{};
    Feedback f_{}; EDM55::Runtime core_{}; EDM55::Sample sample_{}; EDM57::Runtime flush_{};
    EDM61::Binding sourceBinding_{}; EDM61::SimSource compatibilitySource_{};
    bool compatibilityMode_=false;
    std::uint64_t sourceGenerationHighWater_=0;
    EDM59::Recipe servoRecipe_{};
    Candidate candidate_{}; bool pending_=false,persistentIntent_=false,staleInjected_=false;
    double scale_=0,offset_=0,position_=0,velocity_=0,candidatePosition_=0,candidateVelocity_=0,shortMinimum_=0;
    int sign_=1,lastDirection_=0;
    std::uint64_t lastTick_=0,lastUs_=0,zeroSinceUs_=0,originSinceUs_=0;
    std::uint64_t originStableSinceUs_=0;
    std::uint32_t originStableTicks_=0;
    double originStableLow_=0,originStableHigh_=0;
    bool originSettleWasReturn_=false;
    Reason factsReason_=Reason::None;
    static bool IsBoundServo(unsigned p) noexcept { return p>=46U && p<=52U; }
    static bool IsFlush(unsigned p) noexcept { return EDM57::IsFlushProfile(p); }
    static bool IsNormal(unsigned p) noexcept { return p==35U || p==46U; }
    static bool IsPersistent(unsigned p) noexcept { return p==36U || p==47U; }
    static bool IsStale(unsigned p) noexcept { return p==37U || p==48U; }
    static bool Finite(double value) noexcept { return std::isfinite(value); }
    void SetPhase(Phase phase) noexcept { f_.phase=static_cast<std::uint32_t>(phase); }
    static void IncrementDiagnostic(std::uint32_t& value) noexcept
    { if(value<(std::numeric_limits<std::uint32_t>::max)()) ++value; }
    template<class O> void ObserveOriginPredicate(const O& o,double actual) noexcept
    {
        auto& d=f_.originSettle;
        d.sampledTick=o.tick; d.sampledUs=o.monotonicUs;
        d.errorMm=actual/scale_; d.pdoMmS=o.pdoVelocityPps/scale_;
        if(!d.observations) d.errorMinMm=d.errorMaxMm=d.errorMm;
        if(d.errorMm<d.errorMinMm) d.errorMinMm=d.errorMm;
        if(d.errorMm>d.errorMaxMm) d.errorMaxMm=d.errorMm;
        if(std::fabs(d.pdoMmS)>d.maxAbsPdoMmS) d.maxAbsPdoMmS=std::fabs(d.pdoMmS);
        IncrementDiagnostic(d.observations); d.blockerMask=0U; d.windowRangeMm=0.0;
        // These repeated comparisons only label the unchanged gate below.
        if(!(position_==0.0 && velocity_==0.0 &&
            o.commandPulse==f_.originPulse && o.planningPulse==f_.originPulse &&
            o.cmdVelocityPps==0.0 && o.logicalVelocityPps==0.0 && o.targetVelocityPps==0.0))
        { d.blockerMask|=1U; IncrementDiagnostic(d.referenceRejectTicks); }
        if(!(std::fabs(actual)<=.00075*scale_))
        { d.blockerMask|=2U; IncrementDiagnostic(d.positionRejectTicks); }
        // FIX2: PDO TargetVelocity is a correction request, not measured
        // motion. Retain its magnitude as information, never as a gate bit.
        if(!(std::fabs(o.pdoVelocityPps)<=.002*scale_))
        { IncrementDiagnostic(d.pdoHighTicks); }
    }
    template<class O> void ObserveOriginProgress(const O& o) noexcept
    {
        auto& d=f_.originSettle;
        const std::uint64_t stable=originStableSinceUs_ && o.monotonicUs>=originStableSinceUs_ ?
            o.monotonicUs-originStableSinceUs_:0ULL;
        d.stableUs=static_cast<std::uint32_t>((std::min)(stable,
            static_cast<std::uint64_t>((std::numeric_limits<std::uint32_t>::max)())));
        d.stableTicks=originStableTicks_;
        if(d.stableUs>d.bestStableUs) d.bestStableUs=d.stableUs;
        if(d.stableTicks>d.bestStableTicks) d.bestStableTicks=d.stableTicks;
        if(d.stableUs<50000U) d.blockerMask|=16U;
        if(d.stableTicks<200U) d.blockerMask|=32U;
    }
    template<class O> bool BoundOriginSettled(const O& o) noexcept
    {
        // Position range tolerates quantized encoder velocity while refusing
        // a transient crossing or continuing physical excursion. A finite
        // nonzero PDO correction request can coexist with a stationary axis.
        // These unchanged inner bounds reserve margin for the separate 1 um
        // formal Stop proof; PDO/source validity is still required by Facts.
        const double actual=o.actualPulse-f_.originPulse;
        ObserveOriginPredicate(o,actual);
        const bool quiet=position_==0.0 && velocity_==0.0 &&
            o.commandPulse==f_.originPulse && o.planningPulse==f_.originPulse &&
            o.cmdVelocityPps==0.0 && o.logicalVelocityPps==0.0 && o.targetVelocityPps==0.0 &&
            std::fabs(actual)<=.00075*scale_;
        if(!quiet)
        { originStableSinceUs_=0; originStableTicks_=0; ObserveOriginProgress(o); return false; }
        if(!originStableSinceUs_)
        { originStableSinceUs_=o.monotonicUs; originStableLow_=originStableHigh_=actual; originStableTicks_=0; }
        if(actual<originStableLow_) originStableLow_=actual;
        if(actual>originStableHigh_) originStableHigh_=actual;
        f_.originSettle.windowRangeMm=(originStableHigh_-originStableLow_)/scale_;
        if(originStableHigh_-originStableLow_>.00025*scale_)
        {
            f_.originSettle.blockerMask|=8U; IncrementDiagnostic(f_.originSettle.rangeResetCount);
            originStableSinceUs_=o.monotonicUs; originStableTicks_=0;
            originStableLow_=originStableHigh_=actual;
        }
        if(originStableTicks_<200U) ++originStableTicks_;
        ObserveOriginProgress(o);
        return originStableTicks_>=200U && o.monotonicUs-originStableSinceUs_>=50000ULL;
    }
    template<class O> bool Facts(const O& o) noexcept
    {
        factsReason_=Reason::Source;
        if(!o.pdoValid || !o.contiguous || !o.clockValid || !o.axisExists || !o.linear ||
            !o.servoReady || !o.modeReady || o.fault || !o.allOtherAxesIdle || !o.groupQuiet ||
            !o.axisMailboxQuiet || !o.noCompensation || !o.motorEncoderSource || o.hardPositive || o.hardNegative ||
            !Finite(o.actualPulse) || !Finite(o.commandPulse) || !Finite(o.planningPulse) ||
            !Finite(o.cmdVelocityPps) || !Finite(o.logicalVelocityPps) || !Finite(o.targetVelocityPps) ||
            !Finite(o.maximumVelocityPps) || o.maximumVelocityPps<MaximumMmS*scale_ || o.feedrateOverride!=1.0 ||
            (IsBoundServo(f_.profile) && !Finite(o.pdoVelocityPps))) return false;
        factsReason_=Reason::Identity;
        if(o.axisMapGeneration!=scope_.map || o.configGeneration!=scope_.config ||
            o.currentEpoch!=scope_.epoch || o.currentOwnerGeneration!=scope_.ownerGeneration ||
            o.currentOwner!=scope_.owner || o.pulsePerMm!=scale_ || o.referenceOffsetPulse!=offset_ ||
            o.hardwareSign!=sign_) return false;
        factsReason_=Reason::Bounds;
        if(std::fabs(o.actualPulse-f_.originPulse)>.4*scale_ ||
            std::fabs(o.commandPulse-f_.originPulse)>.4*scale_ ||
            std::fabs(o.planningPulse-f_.originPulse)>.4*scale_) return false;
        factsReason_=Reason::Following;
        if(std::fabs(o.actualPulse-o.commandPulse)>.02*scale_) return false;
        factsReason_=Reason::None; return true;
    }
    Candidate View() const noexcept
    {
        Candidate out{}; out.session=f_.session; out.generation=f_.generation;
        out.tick=f_.tick; out.nowUs=f_.nowUs; out.needFormalStop=f_.needFormalStop; out.reason=f_.reason;
        return out;
    }
    Candidate Fail(Reason reason) noexcept
    {
        if(f_.reason==Reason::None || f_.reason==Reason::Cancelled) f_.reason=reason;
        f_.needFormalStop=true; f_.active=false; pending_=false; core_.Cancel(); compatibilitySource_.Cancel(); return View();
    }
    Candidate Terminal(Reason reason) noexcept
    {
        f_.terminal=true; f_.completed=reason==Reason::Complete;
        f_.persistentShort=reason==Reason::PersistentShort; f_.expectedStale=reason==Reason::ExpectedStale;
        f_.reason=reason; SetPhase(Phase::Terminal); return Fail(reason);
    }
    static bool Same(const Candidate&a,const Candidate&b) noexcept
    {
        return a.session==b.session && a.generation==b.generation && a.sequence==b.sequence &&
            a.tick==b.tick && a.nowUs==b.nowUs && a.nativeTargetPulse==b.nativeTargetPulse &&
            a.nativeVelocityPps==b.nativeVelocityPps && a.valid==b.valid &&
            a.needFormalStop==b.needFormalStop && a.reason==b.reason;
    }
    template<class O> EDM57::Readback FlushReadback(const O&o) const noexcept
    {
        EDM57::Readback r{}; r.nowUs=o.monotonicUs;
        r.actualMm=(f_.originPulse-o.actualPulse)/scale_;
        // Keep exact endpoint arithmetic only when all observed native command
        // fields are the exact last committed candidate. Step enforces this
        // identity before entry; Begin/first Step retain measured fields.
        const bool exact=f_.appliedSequence && o.commandPulse==f_.commandPulse &&
            o.planningPulse==f_.commandPulse && o.cmdVelocityPps==f_.nativeVelocityPps &&
            o.logicalVelocityPps==f_.nativeVelocityPps && o.targetVelocityPps==f_.nativeVelocityPps;
        r.commandMm=exact?position_:(f_.originPulse-o.commandPulse)/scale_;
        r.planningMm=exact?position_:(f_.originPulse-o.planningPulse)/scale_;
        r.velocityMmS=-o.cmdVelocityPps/scale_;
        r.logicalVelocityMmS=-o.logicalVelocityPps/scale_;
        r.targetVelocityMmS=-o.targetVelocityPps/scale_;
        return r;
    }
    template<class O,class Provider> bool ReadSource(const O&o,Provider&source,double voltage) noexcept
    {
        EDM61::Request request{}; request.tick=o.tick; request.nowUs=o.monotonicUs;
        request.voltageV=voltage; request.freezeCapture=staleInjected_;
        const EDM61::Frame frame=source.Capture(request);
        if(!frame.live || !EDM61::Valid(frame.binding) || !EDM61::Same(frame.binding,sourceBinding_) ||
            !EDM55::SameIdentity(frame.sample.identity,sourceBinding_.identity) ||
            (frame.sample.valid && frame.sample.capturedAtUs<f_.startedUs)) return false;
        sample_=frame.sample; return true;
    }
    template<class O,class Provider> Candidate TerminalWithSource(Reason reason,const O&o,
        Provider&source,double voltage) noexcept
    {
        // Legacy non-flush completion precedes the regular kernel call. Check
        // this final frame using a bounded probe while retaining its existing
        // last-applied telemetry; a revoked source cannot earn completion.
        if(!ReadSource(o,source,voltage)) return Fail(Reason::Source);
        EDM55::Runtime probe=core_;
        const auto output=probe.Step(sample_,o.monotonicUs,o.tick);
        if(!output.valid || output.fault!=EDM55::Fault::None)
        { f_.kernelFault=output.fault; return Fail(Reason::Kernel); }
        if(reason==Reason::PersistentShort &&
            !(output.shortActive && output.feedInhibited && output.rawVoltageV==20.0))
            return Fail(Reason::Source);
        return Terminal(reason);
    }
    template<class O,class Provider> Candidate FlushStep(const O&o,Provider&source) noexcept
    {
        const EDM57::Readback r=FlushReadback(o);
        const double voltage=flush_.SourceVoltage(r);
        persistentIntent_=flush_.Snapshot().persistentLatched;
        if(flush_.InjectStale(r)) staleInjected_=true;
        if(!ReadSource(o,source,voltage)) return Fail(Reason::Source);
        const bool combined=EDM57::IsServoFlushProfile(f_.profile);
        const EDM55::Output out=core_.Step(sample_,o.monotonicUs,o.tick,
            !combined || static_cast<EDM57::Phase>(flush_.Snapshot().phase)==EDM57::Phase::Work);
        f_.sourceSequence=out.sequence; f_.sourceCapturedAtUs=out.capturedAtUs;
        f_.rawVoltageV=out.rawVoltageV; f_.filteredVoltageV=out.filteredVoltageV;
        f_.targetMmMin=out.targetMmPerMin; f_.servoMmMin=out.speedMmPerMin;
        f_.shortActive=out.shortActive; f_.feedInhibited=out.feedInhibited; f_.kernelFault=out.fault;
        if(!out.valid || out.fault!=EDM55::Fault::None)
            return EDM57::IsStaleProfile(f_.profile) && staleInjected_ && out.fault==EDM55::Fault::Stale ?
                Terminal(Reason::ExpectedStale):Fail(Reason::Kernel);
        EDM57::Request request=flush_.Step(r,out);
        if(combined)
        {
            const bool settling=static_cast<EDM57::Phase>(flush_.Snapshot().phase)==EDM57::Phase::OriginSettle;
            if(settling)
            {
                if(static_cast<Phase>(f_.phase)!=Phase::OriginSettle)
                {
                    originStableSinceUs_=0; originStableTicks_=0;
                    f_.originSettle=OriginSettleDiagnostic{}; f_.originSettle.enteredUs=o.monotonicUs;
                    SetPhase(Phase::OriginSettle);
                }
                if(BoundOriginSettled(o)) request=flush_.CompleteOrigin(r,out);
            }
            else SetPhase(Phase::Advance);
        }
        f_.flush=flush_.Snapshot(); f_.cycles=f_.flush.cycles;
        if(request.complete) return Terminal(Reason::Complete);
        if(request.persistentShort) return Terminal(Reason::PersistentShort);
        if(!Finite(request.destinationMm) || request.destinationMm<0.0 || request.destinationMm>.25)
            return Fail(Reason::Bounds);
        // Report process phase in the dedicated flush feedback. Legacy EDM56
        // phase values remain meaningful for its unchanged P35..P37 fixtures.
        return Plan(f_.appliedSequence?request.speedMmMin/60.0:0.0,o,true,request.destinationMm);
    }
    template<class O> Candidate Plan(double demand,const O& o,bool process=false,double processDestination=0.0) noexcept
    {
        if(!Finite(demand) || std::fabs(demand)>MaximumMmS+1e-12) return Fail(Reason::Kernel);
        const int wanted=demand>0?1:demand<0?-1:0;
        const int moving=velocity_>0?1:velocity_<0?-1:0;
        if((moving && wanted && wanted!=moving) ||
            (!moving && wanted && lastDirection_ && wanted!=lastDirection_ &&
                (!zeroSinceUs_ || o.monotonicUs-zeroSinceUs_<5000))) demand=0.0;
        const bool aboveOriginReturn=static_cast<Phase>(f_.phase)==Phase::ReturnToOrigin && position_<0.0;
        const double destination=process?processDestination:(demand>0.0 && !aboveOriginReturn?.25:0.0);
        const double distance=demand>0.0?destination-position_:position_-destination;
        if(demand!=0.0)
        {
            // Include one whole RT cycle of reserve. Trapezoidal integration
            // below is also checked against remaining braking distance.
            const double reserve=AccelerationMmS2*CycleSeconds;
            const double limit=distance>0.0?std::sqrt(reserve*reserve+2.0*AccelerationMmS2*distance)-reserve:0.0;
            if(std::fabs(demand)>limit) demand=(demand>0?1.0:-1.0)*limit;
        }
        const double step=AccelerationMmS2*CycleSeconds;
        double next=velocity_;
        if(next<demand) next=(demand-next<step)?demand:next+step;
        else if(next>demand) next=(next-demand<step)?demand:next-step;
        double nextPosition=position_+(velocity_+next)*(.5*CycleSeconds);
        // Exact final microscopic integration: at most one acceleration step
        // from rest, with no skipped nonzero command or artificial stop proof.
        if(std::fabs(velocity_)<=step && std::fabs(next)<=step &&
            ((position_>=destination && position_-destination<=step*CycleSeconds && demand<=0.0) ||
             (position_<destination && destination-position_<=step*CycleSeconds && demand>=0.0)))
        { nextPosition=destination; next=0.0; }
        if(!Finite(nextPosition) || !Finite(next) || nextPosition<-.001 || nextPosition>.25+1e-12 ||
            std::fabs(next)>MaximumMmS+1e-12) return Fail(Reason::Bounds);
        if(f_.candidateSequence==(std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::Application);
        Candidate out{}; out.session=f_.session; out.generation=f_.generation;
        out.sequence=++f_.candidateSequence; out.tick=o.tick; out.nowUs=o.monotonicUs;
        out.nativeTargetPulse=f_.originPulse-nextPosition*scale_;
        out.nativeVelocityPps=-next*scale_; out.valid=true;
        if(!Finite(out.nativeTargetPulse) || !Finite(out.nativeVelocityPps) ||
            std::fabs(out.nativeTargetPulse)>2147483647.0 || std::fabs(out.nativeVelocityPps)>2147483647.0)
            return Fail(Reason::Bounds);
        candidatePosition_=nextPosition; candidateVelocity_=next; candidate_=out; pending_=true;
        return out;
    }
};
static_assert(sizeof(Controller)<=2304U,"EDM61 controller and compatibility source must remain bounded.");
static_assert(sizeof(Feedback)<=512U,"EDM56 telemetry must remain bounded.");
}
