#pragma once
#include "EDMPhysicalServoRTSession.h"
#include "EDMGapServoPhysicalRuntime.h"
#include "EDMPhysicalServoFlushContract.h"
#include "EDMPhysicalRetreatContract.h"
#include "EDMPhysicalBidirectionalFlushContract.h"
#include <algorithm>

// EDM70: explicit unloaded Z-only PHYSICAL-AD fixture. Live high-gap voltage
// drives bounded forward references; depth causes a FIXTURE return, never a
// fabricated low/short ADC sample. Low/short, lost source or authority requests
// the existing formal Stop. No PDO setters, discharge or physical stop claims.
// EDM71 adds P57: frozen E-code timed jumps with live physical AD throughout.
// P56 retains its original empty flush recipe and depth-return behavior.
// EDM72 adds P58: real negative-error retreat to a fixed endpoint, then hold.
// It never returns toward the work origin and never treats a short as recovery.
// EDM73 adds P59: repeat the real N-curve retreat with a bounded FIXTURE return.
// Every full cycle settles at both ends; expiry returns without partial credit.
// EDM74 adds P60: signed live AD work inside +/- .03 mm, then a FIXTURE return.
// Work direction evidence never includes expiry braking or the fixture return.
// EDM75 adds P61: signed work with timed, settled fixture jumps. Combined
// normal/deep lift is limited to .03 mm, keeping targets inside [-.06,+.03].
// EDM76 adds isolated P62: the same signed flush geometry with a 60 s source
// window and at least ten full cycles. P56..P61 retain their 10 s windows.
namespace EDM70
{
constexpr double DepthMm=.03, OuterMm=.1, FeedMmMin=5.0, PdoCapMmS=.1;
constexpr double MaximumMmS=FeedMmMin/60.0, AccelerationMmS2=PdoCapMmS/.3;
constexpr double CycleSeconds=.00025;
constexpr bool IsSignedFlushProfile(unsigned profile) noexcept
{ return profile==61U || profile==62U; }
constexpr std::uint32_t MinimumSignedFlushCyclesFor(unsigned profile) noexcept
{ return profile==62U?10U:profile==61U?2U:0U; }
// EDM74 FIX1 observations plus the FIX2 local integral-handoff receipt.
// Neither the observations nor the receipt can qualify physical completion.
struct OriginControlDiagnostic
{
    std::uint64_t sampleTick=0;
    double kp=0,ki=0,kvff=0,proportionalMmS=0,integralMmS=0,guardedHardwareMmS=0;
    std::uint32_t samples=0,idleGainTicks=0,motionGainTicks=0,otherGainTicks=0,clampedTicks=0,invalidTicks=0;
    std::uint64_t handoffTick=0;
    double handoffIntegralMmS=0;
    std::uint64_t handoffEntryUs=0;
    std::uint32_t handoffCount=0,handoffPhase=0;
};
struct Proof
{
    std::uint64_t session=0,generation=0,firstSequence=0,lastSequence=0;
    std::uint64_t windowUs=0,runtimeSteps=0,freshSamples=0,maximumSourceAgeUs=0,maximumStepUs=0;
    double maximumTargetError=0,maximumActualDepthMm=0,minimumRawV=0,maximumRawV=0;
    double maximumActualRetreatMm=0;
    std::uint64_t negativeTargetTicks=0,negativeAppliedTicks=0;
    std::uint64_t fixtureReturnTicks=0;
    std::uint64_t positiveTargetTicks=0,positiveAppliedTicks=0,workTicks=0;
    double minimumFilteredV=0,maximumFilteredV=0;
    double maximumWorkCommandDepthMm=0,maximumWorkCommandRetreatMm=0;
    std::uint32_t retreatEndpointCount=0;
    std::uint32_t workReversals=0;
    EDM69::Reason inputReason=EDM69::Reason::None;
    bool completedWindow=false,lowInput=false,partialRetreatAborted=false,bidirectionalObserved=false;
    OriginControlDiagnostic originControl{};
};
template<class S> bool MatchesScope(const EDM69::Request& r,const S& s) noexcept
{
    const auto& a=r.capture.scope;
    return a.session==s.session && a.run==s.runGeneration && a.cache==s.cacheGeneration &&
        a.dispatch==s.dispatchGeneration && a.epoch==s.executionEpoch &&
        a.owner==s.owner && a.ownerGeneration==s.ownerGeneration;
}
template<class C> bool ValidConfig(const C& c) noexcept
{
    const unsigned profile=static_cast<unsigned>(c.profile);
    const bool flush=profile==57U,retreat=profile==58U || profile==59U,bidirectional=profile==60U;
    const bool signedFlush=IsSignedFlushProfile(profile);
    return (profile==56U || flush || retreat || bidirectional || signedFlush) && EDM69::ValidRequest(c.physicalInput) &&
        c.physicalInput.recipeFamily==(profile==62U?EDM69::RecipeFamily::PhysicalBidirectionalEndurance:
            (bidirectional || signedFlush)?EDM69::RecipeFamily::PhysicalBidirectional:
            retreat?EDM69::RecipeFamily::PhysicalRetreat:EDM69::RecipeFamily::Legacy) &&
        EDM59::SameRecipe(c.servoRecipe,c.physicalInput.capture.recipe) &&
        (signedFlush?(EDM75::ValidateRecipe(c.flush) && EDM75::SameAdmission(c.flush,c.servoRecipe)):
            flush?(EDM71::ValidateRecipe(c.flush) && EDM71::SameAdmission(c.flush,c.servoRecipe)):
            EDM57::SameRecipe(c.flush,EDM57::Recipe{})) && c.axisIndex==2U &&
        c.feedMmMin==FeedMmMin && c.pdoCapMmS==PdoCapMmS &&
        c.targetHalfMm==DepthMm && c.outerHalfMm==OuterMm &&
        c.accelerationTimeSec==.3 && c.decelerationTimeSec==.3 &&
        c.followingLimitMm==.02 && c.positionToleranceMm==.001 && c.excursionToleranceMm==.001 &&
        c.heartbeatTicks==200U && c.maximumIssueAgeTicks==80U && c.stopTimeoutTicks==4000U && c.settleCycles==200U;
}
class Controller
{
public:
    bool Active() const noexcept { return f_.active; }
    bool HasPendingCandidate() const noexcept { return pending_; }
    bool NeedsRuntimeService() const noexcept { return input_.NeedsRuntimeService(); }
    const EDM56::Feedback& Snapshot() const noexcept { return f_; }
    const Proof& SourceProof() const noexcept { return proof_; }
    // Called only after MotionCore's current held-target, owner, raw servo
    // and command checks. P60 hands off once at the final origin; P61/P62 once
    // per exact capture/top/approach/return/origin settle entry. Let the
    // unchanged PI loop accumulate new origin-hold error. The caller takes
    // its anti-windup snapshot AFTER this handoff, so a cap cannot restore I.
    bool HandoffOriginIntegral(std::uint64_t tick,std::uint64_t nowUs,
        double ki,double& integral) noexcept
    {
        auto& d=proof_.originControl;
        const bool signedFlush=IsSignedFlushProfile(f_.profile);
        double destination=0.0;
        std::uint64_t entry=f_.originSettle.enteredUs;
        std::uint32_t phase=f_.phase;
        if(signedFlush)
        {
            const auto held=static_cast<EDM57::Phase>(f_.flush.phase);
            if(held==EDM57::Phase::TopDwell || !EDM75::HoldDestination(f_.flush,flush_,destination) ||
                f_.phase!=static_cast<std::uint32_t>(held==EDM57::Phase::OriginSettle?
                    EDM56::Phase::OriginSettle:EDM56::Phase::Advance)) return false;
            entry=f_.flush.phaseSinceUs; phase=f_.flush.phase;
            if((d.handoffEntryUs==entry && d.handoffPhase==phase) || entry<d.handoffEntryUs) return false;
        }
        else if(f_.profile!=60U || f_.phase!=static_cast<std::uint32_t>(EDM56::Phase::OriginSettle) ||
            !cycleFinishing_ || !proof_.completedWindow || d.handoffTick) return false;
        if(!f_.active || f_.terminal || f_.needFormalStop ||
            pending_ || !f_.appliedSequence || f_.candidateSequence!=f_.appliedSequence ||
            !tick || tick!=f_.tick || nowUs!=f_.nowUs || !entry || entry>nowUs ||
            proof_.session!=f_.session || proof_.generation!=f_.generation ||
            d.handoffCount==(std::numeric_limits<std::uint32_t>::max)() ||
            position_!=destination || velocity_!=0.0 || f_.commandPulse!=f_.originPulse-destination*scale_ ||
            f_.nativeVelocityPps!=0.0 || !std::isfinite(ki) || ki<0.0 || !std::isfinite(integral))
            return false;
        const double previousMmS=integral*ki/scale_;
        if(!std::isfinite(previousMmS)) return false;
        d.handoffTick=tick; d.handoffIntegralMmS=previousMmS;
        d.handoffEntryUs=entry; d.handoffPhase=phase; ++d.handoffCount;
        integral=0.0;
        return true;
    }
    void ObserveOriginControl(std::uint64_t tick,double kp,double ki,double kvff,
        double proportional,double integral,double hardwareVelocity,
        bool idleGains,bool motionGains,bool clamped) noexcept
    {
        double held=0.0;
        const bool origin=f_.profile==60U && f_.phase==static_cast<std::uint32_t>(EDM56::Phase::OriginSettle);
        const bool flushHold=IsSignedFlushProfile(f_.profile) && EDM75::HoldDestination(f_.flush,flush_,held) &&
            f_.phase==static_cast<std::uint32_t>(f_.flush.phase==static_cast<std::uint32_t>(EDM57::Phase::OriginSettle)?
                EDM56::Phase::OriginSettle:EDM56::Phase::Advance) &&
            f_.nativeVelocityPps==0.0 && f_.commandPulse==f_.originPulse-held*scale_;
        if(!f_.active || (!origin && !flushHold) ||
            pending_ || !f_.appliedSequence || f_.candidateSequence!=f_.appliedSequence ||
            tick!=f_.tick || tick<=proof_.originControl.sampleTick) return;
        auto& d=proof_.originControl; d.sampleTick=tick;
        if(!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kvff) ||
            !std::isfinite(proportional) || !std::isfinite(integral) || !std::isfinite(hardwareVelocity))
        { ++d.invalidTicks; return; }
        d.kp=kp; d.ki=ki; d.kvff=kvff;
        d.proportionalMmS=proportional/scale_; d.integralMmS=integral/scale_;
        d.guardedHardwareMmS=hardwareVelocity/scale_; ++d.samples;
        if(idleGains) ++d.idleGainTicks;
        if(motionGains) ++d.motionGainTicks;
        if(!idleGains && !motionGains) ++d.otherGainTicks;
        if(clamped) ++d.clampedTicks;
    }
    // Same PDO owner's captured source BEFORE its one existing QPC read.
    // Continue after cancellation to earn the source consumer's later zero.
    void ObserveRuntime(const EDM69::Facts& facts,const EDM64::Snapshot& raw,
        const EDM53::Stamp& stamp,std::uint64_t frequency) noexcept
    {
        facts_=facts; frequency_=frequency;
        if(stamp.Ready()) { readable_=raw; readableStamp_=stamp; }
        if(input_.NeedsRuntimeService())
        {
            input_.RuntimeCycle(facts,raw,stamp,frequency);
            inputReady_=input_.ControlRead(inputFeedback_);
        }
    }
    bool Touch(std::uint64_t issueUs) noexcept
    {
        if(!f_.active) return true;
        if(input_.ControlTouch(request_.capture.scope.session,issueUs)) return true;
        Fail(EDM56::Reason::Identity); return false;
    }
    template<class S,class C,class O> bool Begin(const S& scope,const C& config,
        double origin,std::uint64_t generation,const O& o) noexcept
    {
        if(f_.active || pending_ || !generation || generation<=generationHighWater_ ||
            !scope.session || scope.session<=sessionHighWater_) return false;
        generationHighWater_=generation; sessionHighWater_=scope.session;
        if(!ValidConfig(config) || !MatchesScope(config.physicalInput,scope) ||
            !scope.axisMapGeneration || !scope.configGeneration ||
            !std::isfinite(origin) || !std::isfinite(config.pulsePerMm) ||
            config.pulsePerMm<1000.0 || config.pulsePerMm>1000000000.0 ||
            std::fabs(origin)+OuterMm*config.pulsePerMm>2147483647.0 ||
            !std::isfinite(config.referenceOffsetPulse) ||
            (config.hardwareSign!=1 && config.hardwareSign!=-1) ||
            facts_.tick!=o.tick || facts_.nowUs!=o.monotonicUs ||
            frequency_!=config.physicalInput.capture.qpcFrequency ||
            o.monotonicUs<config.physicalInput.capture.capturedUs ||
            o.monotonicUs-config.physicalInput.capture.capturedUs>=EDM69::StartDeadlineUs) return false;
        EDM64::Snapshot admission=readable_;
        EDM64::PhysicalRTShadow::ValidateRead(admission,readableStamp_,o.monotonicUs,o.clockValid,frequency_);
        const auto& frozen=config.physicalInput;
        if(!readableStamp_.Ready() || admission.readerState!=EDM64::ReaderState::Fresh ||
            admission.state!=EDM64::State::Valid || !admission.output.valid || !admission.input.Usable() ||
            admission.physicalPermit || admission.discharge ||
            !EDM55::SameIdentity(admission.identity,frozen.physicalIdentity) ||
            !EDM63::SameRoute(EDM63::ProducerRoute(admission.input.sample),frozen.route) ||
            admission.maxAgeUs!=frozen.maxAgeUs || admission.maxTickGapUs!=frozen.maxTickGapUs ||
            admission.input.sample.sequence<frozen.baselineSequence ||
            admission.input.sample.capturedAtUs<frozen.baselineCapturedUs) return false;
        const double admissionVoltage=static_cast<double>(admission.input.conversion.voltageMv)/1000.0;
        const unsigned profile=static_cast<unsigned>(config.profile);
        if((profile==60U || IsSignedFlushProfile(profile))?!EDM74::InputAllowed(frozen.capture.recipe,admissionVoltage):
            (profile==58U || profile==59U)?!EDM72::NegativeInputAllowed(frozen.capture.recipe,admissionVoltage):
            admissionVoltage<=frozen.capture.recipe.referenceV+frozen.capture.recipe.curve.deadbandV) return false;
        request_=frozen;
        // This private consumer starts only after the physical Arm/ServoStart
        // handshake. Keep the NC config, recipe, scope, route and source pins
        // immutable; give only this private RT admission copy a current source
        // baseline and the existing observation clock. A restarted consumer's
        // zero receipt can be newer than the earlier NC capture. The bounded
        // capture-age gate above still applies, and EDM69 requires a distinct
        // real PDO sample AFTER this new RT admission before any advance.
        request_.capture.capturedUs=o.monotonicUs;
        request_.baselineSequence=admission.input.sample.sequence;
        request_.baselineCapturedUs=admission.input.sample.capturedAtUs;
        map_=scope.axisMapGeneration; config_=scope.configGeneration;
        scale_=config.pulsePerMm; offset_=config.referenceOffsetPulse; sign_=config.hardwareSign;
        f_=EDM56::Feedback{}; proof_=Proof{};
        f_.session=scope.session; f_.generation=generation; f_.profile=static_cast<std::uint8_t>(config.profile);
        flush_=config.flush; flushDeep_=false; finishDestination_=0.0;
        f_.originPulse=origin; f_.commandPulse=o.commandPulse;
        f_.startedTick=f_.tick=o.tick; f_.startedUs=f_.nowUs=o.monotonicUs;
        proof_.session=scope.session; proof_.generation=generation;
        position_=(origin-o.commandPulse)/scale_; velocity_=0;
        lastTick_=lastUs_=zeroSinceUs_=stableSinceUs_=0; stableTicks_=0; direction_=workDirection_=0;
        pending_=atDepth_=cycleQualified_=cycleFinishing_=false; inputReady_=false;
        if(!Facts(o) || !o.axisIdle || std::fabs(position_)>.001 ||
            o.cmdVelocityPps!=0.0 || o.logicalVelocityPps!=0.0 || o.targetVelocityPps!=0.0 ||
            !input_.ControlStart(request_))
        { Fail(EDM56::Reason::InvalidStart); return false; }
        // Begin is after ObserveRuntime. Defer adoption to the next callback;
        // never call the input runtime twice at this tick, including restart.
        // The EDM69 gate subsequently requires its own later real PDO sample.
        f_.active=true;
        SetPhase(position_==0.0?EDM56::Phase::Advance:EDM56::Phase::ReturnToOrigin);
        if(f_.profile==57U)
            SetFlushPhase(position_==0.0?EDM57::Phase::Work:EDM57::Phase::ReturnToOrigin,0ULL);
        else if(f_.profile==58U || f_.profile==59U) f_.phase=EDM72::RetreatPhase;
        else if(f_.profile==60U) f_.phase=EDM74::WorkPhase;
        else if(IsSignedFlushProfile(f_.profile)) SetFlushPhase(EDM57::Phase::Work,0ULL);
        return true;
    }
    template<class O> EDM56::Candidate Step(const O& o) noexcept
    {
        if(!f_.active) return View();
        f_.tick=o.tick; f_.nowUs=o.monotonicUs;
        if(pending_) return Fail(EDM56::Reason::Application);
        if(!o.tick || !o.monotonicUs ||
            (!lastTick_ && (o.tick<f_.startedTick || o.tick-f_.startedTick>1ULL ||
                o.monotonicUs<f_.startedUs || o.monotonicUs-f_.startedUs>2000ULL)) ||
            (lastTick_ && (o.tick!=lastTick_+1ULL || o.monotonicUs<=lastUs_ || o.monotonicUs-lastUs_>2000ULL)))
            return Fail(EDM56::Reason::Clock);
        if(!Facts(o)) return Fail(EDM56::Reason::Source);
        if(f_.appliedSequence && (o.commandPulse!=f_.commandPulse || o.planningPulse!=f_.commandPulse ||
            o.cmdVelocityPps!=f_.nativeVelocityPps || o.logicalVelocityPps!=f_.nativeVelocityPps ||
            o.targetVelocityPps!=f_.nativeVelocityPps)) return Fail(EDM56::Reason::Application);
        if(o.monotonicUs<f_.startedUs || o.monotonicUs-f_.startedUs>=EDM69::TotalDeadlineUsFor(request_.recipeFamily))
            return Fail(EDM56::Reason::Timeout);
        lastTick_=o.tick; lastUs_=o.monotonicUs; ++f_.steps;
        if(!inputReady_ && o.tick==f_.startedTick) return Plan(0.0,position_,o);
        if(!inputReady_ || !EDM69::SameRequest(inputFeedback_.request,request_) ||
            inputFeedback_.rtTick!=o.tick || inputFeedback_.nowUs!=o.monotonicUs ||
            inputFeedback_.qpcFrequency!=frequency_) return Fail(EDM56::Reason::Source);
        const auto& in=inputFeedback_;
        proof_.inputReason=in.reason;
        if(in.state==EDM69::State::Fault || in.state==EDM69::State::Cancelled)
            return Fail(EDM56::Reason::Source);
        if(in.state==EDM69::State::AwaitingStart) return Plan(0.0,position_,o);
        if(!in.startAcknowledged || (in.state!=EDM69::State::Running &&
            in.state!=EDM69::State::Draining && in.state!=EDM69::State::Completed) ||
            !readableStamp_.Ready() || readable_.input.sample.sequence!=in.sourceSequence ||
            readable_.input.sample.capturedAtUs!=in.sourceCapturedUs)
            return Fail(EDM56::Reason::Source);
        const double voltage=static_cast<double>(readable_.input.conversion.voltageMv)/1000.0;
        f_.sourceSequence=in.sourceSequence; f_.sourceCapturedAtUs=in.sourceCapturedUs;
        f_.rawVoltageV=voltage; f_.filteredVoltageV=in.output.filteredVoltageV;
        f_.targetMmMin=in.output.targetMmPerMin; f_.servoMmMin=in.output.speedMmPerMin;
        f_.shortActive=in.output.shortActive; f_.feedInhibited=in.output.feedInhibited;
        f_.kernelFault=in.output.fault;
        // P56/P57 retain their high-gap-only gate. P58/P59 admit a real
        // negative error above the fixed short-clear ceiling; it never accepts
        // a short or a transition back to the forward/deadband region.
        // Check RAW input even after EDM69 Completed neutralizes its kernel.
        const bool retreat=f_.profile==58U || f_.profile==59U,bidirectional=f_.profile==60U,signedFlush=IsSignedFlushProfile(f_.profile);
        const bool signedInput=bidirectional || signedFlush;
        const bool rejectedVoltage=signedInput?!EDM74::InputAllowed(request_.capture.recipe,voltage):
            retreat?!EDM72::NegativeInputAllowed(request_.capture.recipe,voltage):
            (!std::isfinite(voltage) || voltage<=request_.capture.recipe.referenceV+request_.capture.recipe.curve.deadbandV);
        if(rejectedVoltage || in.output.shortActive || in.output.fault!=EDM55::Fault::None ||
            (in.state==EDM69::State::Running && (in.output.feedInhibited ||
                (!signedInput && (retreat?in.output.targetMmPerMin>=0.0:in.output.targetMmPerMin<0.0)))))
        { proof_.lowInput=true; return Fail(EDM56::Reason::Source); }
        if(retreat && in.state==EDM69::State::Running && in.output.targetMmPerMin<0.0)
            ++proof_.negativeTargetTicks;
        if(signedInput && in.state==EDM69::State::Running &&
            (bidirectional || (!cycleFinishing_ && f_.flush.phase==static_cast<std::uint32_t>(EDM57::Phase::Work))))
        {
            if((bidirectional && f_.phase!=EDM74::WorkPhase) || cycleFinishing_ || !std::isfinite(in.output.filteredVoltageV) ||
                !std::isfinite(in.output.targetMmPerMin) || !std::isfinite(in.output.speedMmPerMin))
                return Fail(EDM56::Reason::Source);
            if(!proof_.workTicks) proof_.minimumFilteredV=proof_.maximumFilteredV=in.output.filteredVoltageV;
            proof_.minimumFilteredV=(std::min)(proof_.minimumFilteredV,in.output.filteredVoltageV);
            proof_.maximumFilteredV=(std::max)(proof_.maximumFilteredV,in.output.filteredVoltageV);
            ++proof_.workTicks;
            if(in.output.targetMmPerMin>0.0) ++proof_.positiveTargetTicks;
            else if(in.output.targetMmPerMin<0.0) ++proof_.negativeTargetTicks;
        }
        proof_.firstSequence=in.coverage.firstSourceSequence;
        proof_.lastSequence=in.coverage.lastSourceSequence;
        proof_.runtimeSteps=in.runtimeSteps; proof_.freshSamples=in.coverage.freshSamples;
        proof_.maximumSourceAgeUs=in.maximumSourceAgeUs; proof_.maximumStepUs=in.maximumStepUs;
        proof_.maximumTargetError=in.maximumTargetError;
        proof_.minimumRawV=in.coverage.minRawV; proof_.maximumRawV=in.coverage.maxRawV;
        if(in.output.capturedAtUs>=in.firstSourceCapturedUs)
            proof_.windowUs=in.output.capturedAtUs-in.firstSourceCapturedUs;
        const double actual=(f_.originPulse-o.actualPulse)/scale_;
        if(actual>proof_.maximumActualDepthMm) proof_.maximumActualDepthMm=actual;
        if((retreat || signedInput) && -actual>proof_.maximumActualRetreatMm) proof_.maximumActualRetreatMm=-actual;
        if(in.state==EDM69::State::Completed)
        {
            if(!in.zeroReceipt || in.zeroReceiptTick<=in.terminalTick || proof_.windowUs<EDM69::DurationUsFor(request_.recipeFamily) ||
                proof_.freshSamples<2ULL || in.coverage.saturated) return Fail(EDM56::Reason::Source);
            proof_.completedWindow=true;
        }
        if(signedFlush) return BidirectionalFlushStep(o,in);
        if(bidirectional) return BidirectionalStep(o,in);
        if(f_.profile==59U) return RetreatCycleStep(o,in);
        if(retreat) return RetreatStep(o,in);
        if(f_.profile==57U) return FlushStep(o,in);
        auto phase=static_cast<EDM56::Phase>(f_.phase);
        if(phase==EDM56::Phase::Advance && position_==DepthMm && velocity_==0.0 && actual>=DepthMm-.001)
        { atDepth_=true; SetPhase(EDM56::Phase::ReturnToOrigin); phase=EDM56::Phase::ReturnToOrigin; }
        if(proof_.completedWindow && phase==EDM56::Phase::Advance)
        { SetPhase(EDM56::Phase::ReturnToOrigin); phase=EDM56::Phase::ReturnToOrigin; }
        if(phase==EDM56::Phase::ReturnToOrigin && position_==0.0 && velocity_==0.0)
        {
            stableSinceUs_=0; stableTicks_=0;
            f_.originSettle=EDM56::OriginSettleDiagnostic{}; f_.originSettle.enteredUs=o.monotonicUs;
            SetPhase(EDM56::Phase::OriginSettle); phase=EDM56::Phase::OriginSettle;
        }
        if(phase==EDM56::Phase::OriginSettle && OriginSettled(o))
        {
            if(atDepth_) { ++f_.cycles; atDepth_=false; }
            if(proof_.completedWindow)
            {
                if(!f_.cycles || proof_.maximumActualDepthMm<DepthMm-.001) return Fail(EDM56::Reason::Bounds);
                f_.active=false; f_.terminal=f_.completed=f_.needFormalStop=true;
                f_.reason=EDM56::Reason::Complete; SetPhase(EDM56::Phase::Terminal);
                input_.ControlCancel(request_.capture.scope.session); return View();
            }
            SetPhase(EDM56::Phase::Advance); phase=EDM56::Phase::Advance;
        }
        double demand=0.0,destination=0.0;
        if(phase==EDM56::Phase::Advance)
        { demand=(std::min)(MaximumMmS,(std::max)(0.0,in.output.speedMmPerMin/60.0)); destination=DepthMm; }
        else if(phase==EDM56::Phase::ReturnToOrigin) demand=position_<0.0?MaximumMmS:-MaximumMmS;
        return Plan(f_.appliedSequence?demand:0.0,destination,o);
    }
    template<class O> bool Commit(const EDM56::Candidate& c,bool applied,const O& o) noexcept
    {
        if(!pending_ || !f_.active || !SameCandidate(c,candidate_) || !applied || !Facts(o) ||
            o.tick!=c.tick || o.monotonicUs!=c.nowUs || o.commandPulse!=c.nativeTargetPulse ||
            o.planningPulse!=c.nativeTargetPulse || o.cmdVelocityPps!=c.nativeVelocityPps ||
            o.logicalVelocityPps!=c.nativeVelocityPps || o.targetVelocityPps!=c.nativeVelocityPps)
        { Fail(EDM56::Reason::Application); return false; }
        pending_=false; position_=candidatePosition_; velocity_=candidateVelocity_;
        f_.commandPulse=c.nativeTargetPulse; f_.nativeVelocityPps=c.nativeVelocityPps;
        f_.appliedMmMin=-c.nativeVelocityPps/scale_*60.0; f_.appliedSequence=c.sequence;
        if((f_.profile==58U || f_.profile==59U) && velocity_<0.0) ++proof_.negativeAppliedTicks;
        if(f_.profile==59U && f_.phase==static_cast<std::uint32_t>(EDM56::Phase::ReturnToOrigin) && velocity_>0.0)
            ++proof_.fixtureReturnTicks;
        if(f_.profile==60U || IsSignedFlushProfile(f_.profile))
        {
            const bool work=f_.profile==60U?f_.phase==EDM74::WorkPhase:
                f_.phase==static_cast<std::uint32_t>(EDM56::Phase::Advance) &&
                f_.flush.phase==static_cast<std::uint32_t>(EDM57::Phase::Work);
            if(work && inputFeedback_.state==EDM69::State::Running && !cycleFinishing_)
            {
                const int appliedDirection=velocity_>0.0?1:velocity_<0.0?-1:0;
                if(appliedDirection>0) ++proof_.positiveAppliedTicks;
                else if(appliedDirection<0) ++proof_.negativeAppliedTicks;
                if(appliedDirection)
                {
                    if(workDirection_ && appliedDirection!=workDirection_) ++proof_.workReversals;
                    workDirection_=appliedDirection;
                }
                proof_.bidirectionalObserved=proof_.positiveAppliedTicks && proof_.negativeAppliedTicks && proof_.workReversals;
                proof_.maximumWorkCommandDepthMm=(std::max)(proof_.maximumWorkCommandDepthMm,position_);
                proof_.maximumWorkCommandRetreatMm=(std::max)(proof_.maximumWorkCommandRetreatMm,-position_);
            }
            else if(f_.phase==static_cast<std::uint32_t>(EDM56::Phase::ReturnToOrigin) && velocity_!=0.0)
                ++proof_.fixtureReturnTicks;
        }
        if(velocity_!=0.0) { direction_=velocity_>0?1:-1; zeroSinceUs_=0; }
        else if(!zeroSinceUs_) zeroSinceUs_=o.monotonicUs;
        return true;
    }
    void Cancel() noexcept
    {
        if(request_.capture.scope.session) input_.ControlCancel(request_.capture.scope.session);
        f_.active=false; pending_=false; f_.needFormalStop=true;
        if(!f_.terminal && f_.reason==EDM56::Reason::None) f_.reason=EDM56::Reason::Cancelled;
    }
private:
    EDM69::Session input_{};
    EDM57::Recipe flush_{};
    bool flushDeep_=false;
    EDM69::Request request_{}; EDM69::Feedback inputFeedback_{}; EDM69::Facts facts_{};
    EDM64::Snapshot readable_{}; EDM53::Stamp readableStamp_{};
    EDM56::Feedback f_{}; Proof proof_{}; EDM56::Candidate candidate_{};
    std::uint64_t frequency_=0,generationHighWater_=0,sessionHighWater_=0,map_=0,config_=0;
    std::uint64_t lastTick_=0,lastUs_=0,zeroSinceUs_=0,stableSinceUs_=0;
    std::uint32_t stableTicks_=0;
    double scale_=0,offset_=0,position_=0,velocity_=0,candidatePosition_=0,candidateVelocity_=0,stableLow_=0,stableHigh_=0;
    double finishDestination_=0;
    int sign_=1,direction_=0,workDirection_=0;
    bool pending_=false,inputReady_=false,atDepth_=false,cycleQualified_=false,cycleFinishing_=false;
    void SetPhase(EDM56::Phase p) noexcept { f_.phase=static_cast<std::uint32_t>(p); }
    EDM56::Candidate View() const noexcept
    { EDM56::Candidate c{}; c.session=f_.session; c.generation=f_.generation; c.tick=f_.tick;
      c.nowUs=f_.nowUs; c.needFormalStop=f_.needFormalStop; c.reason=f_.reason; return c; }
    EDM56::Candidate Fail(EDM56::Reason r) noexcept
    { if(f_.reason==EDM56::Reason::None || f_.reason==EDM56::Reason::Cancelled) f_.reason=r;
      Cancel(); return View(); }
    static bool SameCandidate(const EDM56::Candidate& a,const EDM56::Candidate& b) noexcept
    { return a.session==b.session && a.generation==b.generation && a.sequence==b.sequence &&
        a.tick==b.tick && a.nowUs==b.nowUs && a.nativeTargetPulse==b.nativeTargetPulse &&
        a.nativeVelocityPps==b.nativeVelocityPps && a.valid==b.valid &&
        a.needFormalStop==b.needFormalStop && a.reason==b.reason; }
    template<class O> bool Facts(const O& o) const noexcept
    {
        return o.pdoValid && o.contiguous && o.clockValid && o.axisExists && o.linear &&
            o.servoReady && o.modeReady && !o.fault && o.allOtherAxesIdle && o.groupQuiet &&
            o.axisMailboxQuiet && o.noCompensation && o.motorEncoderSource && !o.hardPositive && !o.hardNegative &&
            std::isfinite(o.actualPulse) && std::isfinite(o.commandPulse) && std::isfinite(o.planningPulse) &&
            std::isfinite(o.cmdVelocityPps) && std::isfinite(o.logicalVelocityPps) && std::isfinite(o.targetVelocityPps) &&
            std::isfinite(o.pdoVelocityPps) && std::isfinite(o.maximumVelocityPps) &&
            o.maximumVelocityPps>=PdoCapMmS*scale_ && o.feedrateOverride==1.0 &&
            o.axisMapGeneration==map_ && o.configGeneration==config_ &&
            o.currentEpoch==request_.capture.scope.epoch && o.currentOwner==request_.capture.scope.owner &&
            o.currentOwnerGeneration==request_.capture.scope.ownerGeneration &&
            o.pulsePerMm==scale_ && o.referenceOffsetPulse==offset_ && o.hardwareSign==sign_ &&
            std::fabs(o.actualPulse-f_.originPulse)<=OuterMm*scale_ &&
            std::fabs(o.commandPulse-f_.originPulse)<=OuterMm*scale_ &&
            std::fabs(o.planningPulse-f_.originPulse)<=OuterMm*scale_ &&
            std::fabs(o.actualPulse-o.commandPulse)<=.02*scale_;
    }
    template<class O> bool OriginSettled(const O& o) noexcept
    { return PositionSettled(o,0.0); }
    template<class O> bool PositionSettled(const O& o,double destination) noexcept
    {
        auto& d=f_.originSettle;
        const double target=f_.originPulse-destination*scale_;
        const double actual=o.actualPulse-target;
        const bool diagnose=f_.profile==60U || IsSignedFlushProfile(f_.profile);
        if(diagnose)
        {
            const double error=actual/scale_,pdo=std::fabs(o.pdoVelocityPps/scale_);
            if(!d.observations) d.errorMinMm=d.errorMaxMm=error;
            d.errorMinMm=(std::min)(d.errorMinMm,error); d.errorMaxMm=(std::max)(d.errorMaxMm,error);
            d.maxAbsPdoMmS=(std::max)(d.maxAbsPdoMmS,pdo);
            if(pdo>.001) ++d.pdoHighTicks; // Diagnostic only, never a settle gate.
        }
        d.sampledTick=o.tick; d.sampledUs=o.monotonicUs; d.errorMm=actual/scale_;
        d.pdoMmS=o.pdoVelocityPps/scale_; ++d.observations; d.blockerMask=0;
        const bool references=position_==destination && velocity_==0.0 && o.commandPulse==target &&
            o.planningPulse==target && o.cmdVelocityPps==0.0 && o.logicalVelocityPps==0.0 && o.targetVelocityPps==0.0;
        if(!references || std::fabs(actual)>.00075*scale_)
        {
            if(diagnose)
            {
                if(!references) ++d.referenceRejectTicks;
                if(std::fabs(actual)>.00075*scale_) ++d.positionRejectTicks;
                d.stableUs=d.stableTicks=0;
            }
            d.blockerMask=references?2U:1U; stableSinceUs_=0; stableTicks_=0; return false;
        }
        if(!stableSinceUs_) { stableSinceUs_=o.monotonicUs; stableLow_=stableHigh_=actual; stableTicks_=0; }
        stableLow_=(std::min)(stableLow_,actual); stableHigh_=(std::max)(stableHigh_,actual);
        d.windowRangeMm=(stableHigh_-stableLow_)/scale_;
        if(d.windowRangeMm>.00025)
        {
            if(diagnose) ++d.rangeResetCount;
            d.blockerMask|=8U; stableSinceUs_=o.monotonicUs; stableLow_=stableHigh_=actual; stableTicks_=0;
        }
        if(stableTicks_<200U) ++stableTicks_;
        d.stableTicks=stableTicks_; d.stableUs=static_cast<std::uint32_t>(o.monotonicUs-stableSinceUs_);
        if(diagnose)
        {
            d.bestStableUs=(std::max)(d.bestStableUs,d.stableUs);
            d.bestStableTicks=(std::max)(d.bestStableTicks,d.stableTicks);
        }
        if(d.stableUs<50000U) d.blockerMask|=16U;
        if(stableTicks_<200U) d.blockerMask|=32U;
        return stableTicks_>=200U && o.monotonicUs-stableSinceUs_>=50000ULL;
    }
    double WorkDestination(double demand) const noexcept
    {
        // While braking/reversing keep the current direction's finite bound.
        // Snapping to the present position with nonzero velocity would erase
        // a real ramp increment and falsely look like a stopped reference.
        return velocity_>0.0?DepthMm:velocity_<0.0?-DepthMm:
            demand>0.0?DepthMm:demand<0.0?-DepthMm:position_;
    }
    template<class O> EDM56::Candidate BidirectionalStep(const O& o,const EDM69::Feedback& in) noexcept
    {
        const auto returning=static_cast<std::uint32_t>(EDM56::Phase::ReturnToOrigin);
        const auto settling=static_cast<std::uint32_t>(EDM56::Phase::OriginSettle);
        // The first neutralized source callback irreversibly ends live work.
        // Its deceleration and later fixture return cannot earn P/N coverage.
        if(in.state==EDM69::State::Draining || proof_.completedWindow)
        {
            cycleFinishing_=true;
            if(f_.phase==EDM74::WorkPhase) SetRetreatCyclePhase(EDM74::BrakePhase,o.monotonicUs);
        }
        double demand=0.0,destination=position_;
        if(f_.phase==EDM74::WorkPhase)
        {
            if(cycleFinishing_ || in.state!=EDM69::State::Running)
                return Fail(EDM56::Reason::Source);
            demand=(std::max)(-MaximumMmS,(std::min)(MaximumMmS,in.output.speedMmPerMin/60.0));
            destination=WorkDestination(demand);
        }
        else
        {
            if(!cycleFinishing_) return Fail(EDM56::Reason::Bounds);
            if(f_.phase==EDM74::BrakePhase)
            {
                destination=WorkDestination(0.0);
                if(velocity_==0.0) SetRetreatCyclePhase(returning,o.monotonicUs);
            }
            if(f_.phase==returning)
            {
                destination=0.0;
                if(position_==0.0 && velocity_==0.0) SetRetreatCyclePhase(settling,o.monotonicUs);
                else demand=position_<0.0?MaximumMmS:-MaximumMmS;
            }
            else if(f_.phase==settling)
            {
                destination=0.0;
                if(position_!=0.0 || velocity_!=0.0) return Fail(EDM56::Reason::Bounds);
                if(OriginSettled(o) && proof_.completedWindow)
                {
                    if(!proof_.workTicks || proof_.positiveTargetTicks+proof_.negativeTargetTicks>proof_.workTicks ||
                        proof_.positiveAppliedTicks+proof_.negativeAppliedTicks>proof_.workTicks ||
                        proof_.bidirectionalObserved!=(proof_.positiveAppliedTicks && proof_.negativeAppliedTicks && proof_.workReversals))
                        return Fail(EDM56::Reason::Source);
                    f_.active=false; f_.terminal=f_.completed=f_.needFormalStop=true;
                    f_.reason=EDM56::Reason::Complete; SetPhase(EDM56::Phase::Terminal);
                    input_.ControlCancel(request_.capture.scope.session); return View();
                }
            }
            else if(f_.phase!=EDM74::BrakePhase) return Fail(EDM56::Reason::Bounds);
        }
        return Plan(f_.appliedSequence?demand:0.0,destination,o);
    }
    template<class O> EDM56::Candidate RetreatStep(const O& o,const EDM69::Feedback& in) noexcept
    {
        const double endpoint=-EDM72::RetreatDepthMm;
        if(f_.phase==EDM72::RetreatPhase && position_==endpoint && velocity_==0.0)
        {
            f_.phase=EDM72::HoldPhase; stableSinceUs_=0; stableTicks_=0;
            f_.originSettle=EDM56::OriginSettleDiagnostic{}; f_.originSettle.enteredUs=o.monotonicUs;
        }
        if(f_.phase==EDM72::HoldPhase)
        {
            if(position_!=endpoint || velocity_!=0.0) return Fail(EDM56::Reason::Bounds);
            const bool settled=PositionSettled(o,endpoint);
            if(proof_.completedWindow && settled)
            {
                if(!proof_.negativeTargetTicks || !proof_.negativeAppliedTicks ||
                    proof_.maximumActualRetreatMm<EDM72::RetreatDepthMm-.001)
                    return Fail(EDM56::Reason::Bounds);
                f_.cycles=1U; f_.active=false; f_.terminal=f_.completed=f_.needFormalStop=true;
                f_.reason=EDM56::Reason::Complete; SetPhase(EDM56::Phase::Terminal);
                input_.ControlCancel(request_.capture.scope.session); return View();
            }
            return Plan(0.0,endpoint,o);
        }
        if(f_.phase!=EDM72::RetreatPhase || velocity_>0.0)
            return Fail(EDM56::Reason::Bounds);
        // Source-window expiry is not permission to invent more retreat speed.
        // A run that never reached its endpoint must stop without PASS.
        if(proof_.completedWindow) return Fail(EDM56::Reason::Timeout);
        const double demand=(std::max)(-MaximumMmS,(std::min)(0.0,in.output.speedMmPerMin/60.0));
        return Plan(f_.appliedSequence?demand:0.0,endpoint,o);
    }
    void SetRetreatCyclePhase(std::uint32_t phase,std::uint64_t nowUs) noexcept
    {
        f_.phase=phase; stableSinceUs_=0; stableTicks_=0;
        f_.originSettle=EDM56::OriginSettleDiagnostic{};
        f_.originSettle.enteredUs=nowUs;
    }
    template<class O> EDM56::Candidate RetreatCycleStep(const O& o,const EDM69::Feedback& in) noexcept
    {
        const double endpoint=-EDM72::RetreatDepthMm;
        const auto returning=static_cast<std::uint32_t>(EDM56::Phase::ReturnToOrigin);
        const auto settling=static_cast<std::uint32_t>(EDM56::Phase::OriginSettle);
        // Draining has already neutralized the N-curve. It cannot start or
        // extend another outward stroke. Keep actual raw/source gates active
        // during this bounded fixture return and wait for the later zero receipt.
        if(in.state==EDM69::State::Draining || proof_.completedWindow)
        {
            cycleFinishing_=true;
            if(f_.phase==EDM72::RetreatPhase || f_.phase==EDM72::HoldPhase)
            {
                proof_.partialRetreatAborted=true; cycleQualified_=false;
                SetRetreatCyclePhase(returning,o.monotonicUs);
            }
        }
        double demand=0.0,destination=position_;
        if(f_.phase==EDM72::RetreatPhase)
        {
            if(cycleFinishing_ || cycleQualified_ || velocity_>0.0)
                return Fail(EDM56::Reason::Bounds);
            destination=endpoint;
            if(position_==endpoint && velocity_==0.0)
                SetRetreatCyclePhase(EDM72::HoldPhase,o.monotonicUs);
            else demand=(std::max)(-MaximumMmS,(std::min)(0.0,in.output.speedMmPerMin/60.0));
        }
        else if(f_.phase==EDM72::HoldPhase)
        {
            destination=endpoint;
            if(position_!=endpoint || velocity_!=0.0 || cycleQualified_)
                return Fail(EDM56::Reason::Bounds);
            if(PositionSettled(o,endpoint))
            {
                if(proof_.retreatEndpointCount==(std::numeric_limits<std::uint32_t>::max)())
                    return Fail(EDM56::Reason::Bounds);
                ++proof_.retreatEndpointCount; cycleQualified_=true;
                SetRetreatCyclePhase(returning,o.monotonicUs);
            }
        }
        else if(f_.phase==returning)
        {
            destination=0.0;
            if(position_==0.0 && velocity_==0.0)
                SetRetreatCyclePhase(settling,o.monotonicUs);
            else demand=position_<0.0?MaximumMmS:-MaximumMmS;
        }
        else if(f_.phase==settling)
        {
            destination=0.0;
            if(position_!=0.0 || velocity_!=0.0) return Fail(EDM56::Reason::Bounds);
            if(OriginSettled(o))
            {
                if(cycleQualified_)
                {
                    if(f_.cycles==(std::numeric_limits<std::uint32_t>::max)())
                        return Fail(EDM56::Reason::Bounds);
                    ++f_.cycles; cycleQualified_=false;
                }
                if(cycleFinishing_)
                {
                    if(proof_.completedWindow)
                    {
                        if(f_.cycles<2U || proof_.retreatEndpointCount!=f_.cycles ||
                            !proof_.negativeTargetTicks || !proof_.negativeAppliedTicks || !proof_.fixtureReturnTicks ||
                            proof_.maximumActualRetreatMm<EDM72::RetreatDepthMm-.001)
                            return Fail(EDM56::Reason::Bounds);
                        f_.active=false; f_.terminal=f_.completed=f_.needFormalStop=true;
                        f_.reason=EDM56::Reason::Complete; SetPhase(EDM56::Phase::Terminal);
                        input_.ControlCancel(request_.capture.scope.session); return View();
                    }
                }
                else SetRetreatCyclePhase(EDM72::RetreatPhase,o.monotonicUs);
            }
        }
        else return Fail(EDM56::Reason::Bounds);
        return Plan(f_.appliedSequence?demand:0.0,destination,o);
    }
    void SetFlushPhase(EDM57::Phase phase,std::uint64_t nowUs) noexcept
    {
        if(IsSignedFlushProfile(f_.profile) && phase==EDM57::Phase::Work) workDirection_=0;
        f_.flush.phase=static_cast<std::uint32_t>(phase); f_.flush.phaseSinceUs=nowUs;
        if(phase!=EDM57::Phase::Terminal)
        {
            f_.flush.stableSinceUs=0; stableSinceUs_=0; stableTicks_=0;
            f_.originSettle=EDM56::OriginSettleDiagnostic{}; f_.originSettle.enteredUs=nowUs;
        }
        SetPhase(phase==EDM57::Phase::ReturnToOrigin?EDM56::Phase::ReturnToOrigin:
            phase==EDM57::Phase::OriginSettle?EDM56::Phase::OriginSettle:
            phase==EDM57::Phase::Terminal?EDM56::Phase::Terminal:EDM56::Phase::Advance);
    }
    template<class O> bool FlushSettled(const O& o,double destination) noexcept
    {
        const bool settled=PositionSettled(o,destination);
        f_.flush.stableSinceUs=stableSinceUs_; return settled;
    }
    template<class O> EDM56::Candidate BidirectionalFlushStep(const O& o,const EDM69::Feedback& in) noexcept
    {
        // End live work at the first source Draining callback. Retain the
        // current stroke's finite target while braking: a jump may already
        // be outside the +/- .03 work range. No partial jump earns a cycle.
        if((in.state==EDM69::State::Draining || proof_.completedWindow) && !cycleFinishing_)
        {
            cycleFinishing_=true; finishDestination_=f_.flush.destinationMm;
            if(!std::isfinite(finishDestination_) || finishDestination_<-.06 || finishDestination_>DepthMm)
                return Fail(EDM56::Reason::Bounds);
            f_.phase=EDM74::BrakePhase;
        }
        if(cycleFinishing_ && f_.phase==EDM74::BrakePhase)
        {
            if(velocity_!=0.0) return Plan(0.0,finishDestination_,o);
            SetFlushPhase(EDM57::Phase::ReturnToOrigin,o.monotonicUs);
        }
        const auto phase=static_cast<EDM57::Phase>(f_.flush.phase);
        if(cycleFinishing_ && phase!=EDM57::Phase::ReturnToOrigin && phase!=EDM57::Phase::OriginSettle)
            return Fail(EDM56::Reason::Bounds);
        double demand=0.0,destination=position_;
        switch(phase)
        {
        case EDM57::Phase::Work:
            if(in.state!=EDM69::State::Running || position_<-DepthMm || position_>DepthMm)
                return Fail(EDM56::Reason::Bounds);
            if(!f_.flush.phaseSinceUs) f_.flush.phaseSinceUs=o.monotonicUs;
            if(o.monotonicUs-f_.flush.phaseSinceUs>=flush_.workTimeUs)
            {
                destination=WorkDestination(0.0);
                SetFlushPhase(EDM57::Phase::CaptureWorkPoint,o.monotonicUs);
            }
            else
            {
                demand=(std::max)(-MaximumMmS,(std::min)(MaximumMmS,in.output.speedMmPerMin/60.0));
                destination=WorkDestination(demand);
            }
            break;
        case EDM57::Phase::CaptureWorkPoint:
            destination=WorkDestination(0.0);
            if(velocity_==0.0 && FlushSettled(o,position_))
            {
                if(position_<-DepthMm || position_>DepthMm ||
                    f_.flush.cycles==(std::numeric_limits<std::uint32_t>::max)())
                    return Fail(EDM56::Reason::Bounds);
                flushDeep_=flush_.deepInterval && (f_.flush.cycles+1U)%flush_.deepInterval==0U;
                f_.flush.workPointMm=position_;
                f_.flush.liftHeightMm=EDM75::LiftHeight(flush_,flushDeep_);
                f_.flush.topMm=position_-f_.flush.liftHeightMm;
                if(!EDM75::CapturedGeometryValid(f_.flush,flush_)) return Fail(EDM56::Reason::Bounds);
                SetFlushPhase(EDM57::Phase::Lift,o.monotonicUs);
            }
            break;
        case EDM57::Phase::Lift:
            destination=f_.flush.topMm; demand=-flush_.liftMmMin/60.0;
            if(position_==destination && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::TopSettle,o.monotonicUs); }
            break;
        case EDM57::Phase::TopSettle:
            destination=f_.flush.topMm;
            if(FlushSettled(o,destination)) SetFlushPhase(EDM57::Phase::TopDwell,o.monotonicUs);
            break;
        case EDM57::Phase::TopDwell:
            destination=f_.flush.topMm;
            if(position_!=destination || velocity_!=0.0 ||
                std::fabs(o.actualPulse-(f_.originPulse-destination*scale_))>.00075*scale_)
                SetFlushPhase(EDM57::Phase::TopSettle,o.monotonicUs);
            else if(o.monotonicUs-f_.flush.phaseSinceUs>=flush_.topDwellUs)
            { ++f_.flush.topDwellCount; SetFlushPhase(EDM57::Phase::Return,o.monotonicUs); }
            break;
        case EDM57::Phase::Return:
            destination=f_.flush.workPointMm-flush_.finalApproachMm; demand=flush_.returnMmMin/60.0;
            if(position_==destination && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::ApproachBoundarySettle,o.monotonicUs); }
            break;
        case EDM57::Phase::ApproachBoundarySettle:
            destination=f_.flush.workPointMm-flush_.finalApproachMm;
            if(FlushSettled(o,destination))
            { ++f_.flush.slowApproachCount; SetFlushPhase(EDM57::Phase::SlowApproach,o.monotonicUs); }
            break;
        case EDM57::Phase::SlowApproach:
            destination=f_.flush.workPointMm; demand=flush_.finalApproachMmMin/60.0;
            if(position_==destination && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::ReturnSettle,o.monotonicUs); }
            break;
        case EDM57::Phase::ReturnSettle:
            destination=f_.flush.workPointMm;
            if(FlushSettled(o,destination))
            {
                ++f_.flush.cycles; if(flushDeep_) ++f_.flush.deepCycles;
                f_.cycles=f_.flush.cycles; SetFlushPhase(EDM57::Phase::Work,o.monotonicUs);
            }
            break;
        case EDM57::Phase::ReturnToOrigin:
            if(!cycleFinishing_) return Fail(EDM56::Reason::Bounds);
            destination=0.0; demand=position_<0.0?MaximumMmS:-MaximumMmS;
            if(position_==0.0 && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::OriginSettle,o.monotonicUs); }
            break;
        case EDM57::Phase::OriginSettle:
            if(!cycleFinishing_ || position_!=0.0 || velocity_!=0.0) return Fail(EDM56::Reason::Bounds);
            destination=0.0;
            if(FlushSettled(o,0.0) && proof_.completedWindow)
            {
                SetFlushPhase(EDM57::Phase::Terminal,o.monotonicUs);
                f_.flush.destinationMm=0.0; f_.flush.complete=true;
                if(!EDM75::CompletionValid(f_.flush,flush_) || f_.cycles!=f_.flush.cycles ||
                    f_.flush.cycles<MinimumSignedFlushCyclesFor(f_.profile) ||
                    !proof_.workTicks || proof_.positiveTargetTicks+proof_.negativeTargetTicks>proof_.workTicks ||
                    proof_.positiveAppliedTicks+proof_.negativeAppliedTicks>proof_.workTicks ||
                    proof_.bidirectionalObserved!=(proof_.positiveAppliedTicks && proof_.negativeAppliedTicks && proof_.workReversals))
                    return Fail(EDM56::Reason::Bounds);
                f_.active=false; f_.terminal=f_.completed=f_.needFormalStop=true;
                f_.reason=EDM56::Reason::Complete;
                input_.ControlCancel(request_.capture.scope.session); return View();
            }
            break;
        default: return Fail(EDM56::Reason::Bounds);
        }
        f_.flush.destinationMm=destination;
        return Plan(f_.appliedSequence?demand:0.0,destination,o);
    }
    template<class O> EDM56::Candidate FlushStep(const O& o,const EDM69::Feedback& in) noexcept
    {
        auto phase=static_cast<EDM57::Phase>(f_.flush.phase);
        // The 10 s source window owns the finish deadline. Abandon any partial
        // jump immediately, without claiming another cycle; the final origin
        // return uses the original fixed cap, independently of E7/slow approach.
        if(proof_.completedWindow && phase!=EDM57::Phase::ReturnToOrigin &&
            phase!=EDM57::Phase::OriginSettle)
        { SetFlushPhase(EDM57::Phase::ReturnToOrigin,o.monotonicUs); phase=EDM57::Phase::ReturnToOrigin; }
        double demand=0.0,destination=position_;
        switch(phase)
        {
        case EDM57::Phase::Work:
            if(!f_.flush.phaseSinceUs) f_.flush.phaseSinceUs=o.monotonicUs;
            destination=DepthMm;
            if(o.monotonicUs-f_.flush.phaseSinceUs>=flush_.workTimeUs)
                SetFlushPhase(EDM57::Phase::CaptureWorkPoint,o.monotonicUs);
            else demand=(std::min)(MaximumMmS,(std::max)(0.0,in.output.speedMmPerMin/60.0));
            break;
        case EDM57::Phase::CaptureWorkPoint:
            // First decelerate the committed reference. Capture that bounded
            // reference only after actual position is independently settled;
            // copying noisy actual feedback here could exceed the .03 mm cap.
            destination=velocity_==0.0?position_:DepthMm;
            if(velocity_==0.0 && FlushSettled(o,position_))
            {
                if(position_<0.0 || position_>DepthMm ||
                    f_.flush.cycles==(std::numeric_limits<std::uint32_t>::max)())
                    return Fail(EDM56::Reason::Bounds);
                flushDeep_=flush_.deepInterval && (f_.flush.cycles+1U)%flush_.deepInterval==0U;
                f_.flush.workPointMm=position_;
                f_.flush.liftHeightMm=EDM71::LiftHeight(flush_,flushDeep_);
                f_.flush.topMm=position_-f_.flush.liftHeightMm;
                if(!std::isfinite(f_.flush.topMm) || f_.flush.topMm<-.06 ||
                    f_.flush.topMm>=position_) return Fail(EDM56::Reason::Bounds);
                SetFlushPhase(EDM57::Phase::Lift,o.monotonicUs);
            }
            break;
        case EDM57::Phase::Lift:
            destination=f_.flush.topMm; demand=-flush_.liftMmMin/60.0;
            if(position_==destination && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::TopSettle,o.monotonicUs); }
            break;
        case EDM57::Phase::TopSettle:
            destination=f_.flush.topMm;
            if(FlushSettled(o,destination))
                SetFlushPhase(EDM57::Phase::TopDwell,o.monotonicUs);
            break;
        case EDM57::Phase::TopDwell:
            destination=f_.flush.topMm;
            if(position_!=destination || velocity_!=0.0 ||
                std::fabs(o.actualPulse-(f_.originPulse-destination*scale_))>.00075*scale_)
                SetFlushPhase(EDM57::Phase::TopSettle,o.monotonicUs);
            else if(o.monotonicUs-f_.flush.phaseSinceUs>=flush_.topDwellUs)
            { ++f_.flush.topDwellCount; SetFlushPhase(EDM57::Phase::Return,o.monotonicUs); }
            break;
        case EDM57::Phase::Return:
            destination=f_.flush.workPointMm-flush_.finalApproachMm;
            demand=flush_.returnMmMin/60.0;
            if(position_==destination && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::ApproachBoundarySettle,o.monotonicUs); }
            break;
        case EDM57::Phase::ApproachBoundarySettle:
            destination=f_.flush.workPointMm-flush_.finalApproachMm;
            if(FlushSettled(o,destination))
            { ++f_.flush.slowApproachCount; SetFlushPhase(EDM57::Phase::SlowApproach,o.monotonicUs); }
            break;
        case EDM57::Phase::SlowApproach:
            destination=f_.flush.workPointMm; demand=flush_.finalApproachMmMin/60.0;
            if(position_==destination && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::ReturnSettle,o.monotonicUs); }
            break;
        case EDM57::Phase::ReturnSettle:
            destination=f_.flush.workPointMm;
            if(FlushSettled(o,destination))
            {
                ++f_.flush.cycles; if(flushDeep_) ++f_.flush.deepCycles;
                f_.cycles=f_.flush.cycles;
                SetFlushPhase(EDM57::Phase::Work,o.monotonicUs);
            }
            break;
        case EDM57::Phase::ReturnToOrigin:
            destination=0.0; demand=position_<0.0?MaximumMmS:-MaximumMmS;
            if(position_==0.0 && velocity_==0.0)
            { demand=0.0; SetFlushPhase(EDM57::Phase::OriginSettle,o.monotonicUs); }
            break;
        case EDM57::Phase::OriginSettle:
            destination=0.0;
            if(FlushSettled(o,0.0))
            {
                if(!proof_.completedWindow) SetFlushPhase(EDM57::Phase::Work,o.monotonicUs);
                else
                {
                    SetFlushPhase(EDM57::Phase::Terminal,o.monotonicUs);
                    f_.flush.destinationMm=0.0; f_.flush.complete=true;
                    if(!EDM71::CompletionValid(f_.flush,flush_) || proof_.maximumActualDepthMm<=0.0)
                        return Fail(EDM56::Reason::Bounds);
                    f_.active=false; f_.terminal=f_.completed=f_.needFormalStop=true;
                    f_.reason=EDM56::Reason::Complete;
                    input_.ControlCancel(request_.capture.scope.session); return View();
                }
            }
            break;
        default: return Fail(EDM56::Reason::Bounds);
        }
        f_.flush.destinationMm=destination;
        return Plan(f_.appliedSequence?demand:0.0,destination,o);
    }

    template<class O> EDM56::Candidate Plan(double demand,double destination,const O& o) noexcept
    {
        const bool retreat=f_.profile==58U || f_.profile==59U;
        const double lower=(f_.profile==57U || IsSignedFlushProfile(f_.profile))?-.061:f_.profile==60U?-DepthMm:retreat?-EDM72::RetreatDepthMm:-.001;
        const double upper=retreat?.001:DepthMm;
        if(!std::isfinite(demand) || !std::isfinite(destination) || std::fabs(demand)>MaximumMmS+1e-12 ||
            destination<lower || destination>upper) return Fail(EDM56::Reason::Bounds);
        const int wanted=demand>0?1:demand<0?-1:0,moving=velocity_>0?1:velocity_<0?-1:0;
        if((moving && wanted && wanted!=moving) || (!moving && wanted && direction_ && wanted!=direction_ &&
            (!zeroSinceUs_ || o.monotonicUs-zeroSinceUs_<5000ULL))) demand=0.0;
        const double step=AccelerationMmS2*CycleSeconds;
        if(demand!=0.0)
        {
            const double distance=demand>0?destination-position_:position_-destination;
            // Reserve two whole integration ticks before the finite endpoint.
            // This also keeps the final microscopic snap below one ramp step.
            const double reserve=2.0*step;
            const double limit=distance>0?std::sqrt(reserve*reserve+2.0*AccelerationMmS2*distance)-reserve:0.0;
            if(std::fabs(demand)>limit) demand=(demand>0?1.0:-1.0)*limit;
        }
        double next=velocity_;
        if(next<demand) next=(std::min)(demand,next+step);
        else if(next>demand) next=(std::max)(demand,next-step);
        double position=position_+(velocity_+next)*(.5*CycleSeconds);
        if(std::fabs(velocity_)<=step && std::fabs(next)<=step &&
            ((position_>=destination && position_-destination<=step*CycleSeconds && demand<=0.0) ||
             (position_<destination && destination-position_<=step*CycleSeconds && demand>=0.0)))
        { position=destination; next=0.0; }
        if(!std::isfinite(position) || !std::isfinite(next) || position<lower || position>upper+1e-12 ||
            std::fabs(next)>MaximumMmS+1e-12 || f_.candidateSequence==(std::numeric_limits<std::uint64_t>::max)())
            return Fail(EDM56::Reason::Bounds);
        EDM56::Candidate c{}; c.session=f_.session; c.generation=f_.generation; c.sequence=++f_.candidateSequence;
        c.tick=o.tick; c.nowUs=o.monotonicUs; c.nativeTargetPulse=f_.originPulse-position*scale_;
        c.nativeVelocityPps=-next*scale_; c.valid=true;
        if(!std::isfinite(c.nativeTargetPulse) || std::fabs(c.nativeTargetPulse)>2147483647.0 ||
            !std::isfinite(c.nativeVelocityPps) || std::fabs(c.nativeVelocityPps)>2147483647.0)
            return Fail(EDM56::Reason::Bounds);
        candidatePosition_=position; candidateVelocity_=next; candidate_=c; pending_=true; return c;
    }
};
static_assert(sizeof(OriginControlDiagnostic)<=112U,"EDM74/75 held-target observations and handoff remain bounded.");
static_assert(sizeof(Proof)<=320U,"EDM70/72/73/74/75 proof is bounded and cannot replace physical Stop evidence.");
static_assert(sizeof(Controller)<=16384U,"EDM70 RT storage must remain bounded.");
}
