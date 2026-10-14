#pragma once

#include "EDMSynchronizedPath.h"
#include "EDMToolDirectionMapping.h"
#include "EDMCoordinatedGapSource.h"
#include "EDMCoordinatedGapDecision.h"

// EDM84 numerical, single-owner path/GAP composition. No axis command, output
// permit, allocation, clock read or device I/O is exposed by this value type.
namespace EDM84
{
enum class ExcursionMode : std::uint8_t { RetainedPath, DirectionOffset };
struct Config
{
    // Explicit contract for VS2019 in-place reconstruction; preserve all member defaults.
    Config() noexcept {}
    EDM81::Config motion{};
    ExcursionMode excursion=ExcursionMode::RetainedPath;
    DirectionMapping direction{};
    ToolMapping mapping{};
    EDM88::SourceConfig gapSource{}; // Disabled preserves the EDM84--87 input contract.
};
struct Snapshot
{
    EDM81::State state=EDM81::State::Idle;
    EDM81::Reason reason=EDM81::Reason::None;
    EDM81::Scope scope{};
    EDM81::Pose pose{},nominal{},anchorPose{};
    std::array<double,3> frozenDirection{{0.0,0.0,1.0}};
    double cursorMm=0.0,anchorMm=0.0,pathLengthMm=0.0,offsetMm=0.0;
    unsigned ordinal=0U;
    double u=0.0;
    std::uint64_t steps=0U;
    bool committed=false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
    constexpr bool DischargeEnabled() const noexcept { return false; }
};

class Kernel
{
public:
    // Fixed numeric members only. Do not rely on inferred constructor noexcept
    // in the RT reset trait: VS2019 rejected the implicit FIX1 constructor.
    Kernel() noexcept {}

    const Snapshot& Current() const noexcept { return current_; }
    const SynchronizedPath& Path() const noexcept { return path_; }
    const EDM88::Evidence& GapSourceEvidence() const noexcept { return gapSource_.Current(); }
    const EDM89::Evidence& GapDecisionEvidence() const noexcept { return gapDecision_.Current(); }

    bool Begin(const Config& config,const EDM81::Scope& scope,
        const SynchronizedPath& path) noexcept
    {
        if(current_.state!=EDM81::State::Idle && !EDM81::Terminal(current_.state))
            return FailBool(EDM81::Reason::Lifecycle);
        if(!EDM81::ValidScope(scope)) return FailBool(EDM81::Reason::Scope);
        if(scope.run<=retiredRun_) return FailBool(EDM81::Reason::RetiredRun);
        retiredRun_=scope.run; current_=Snapshot{}; current_.scope=scope;
        gapDecision_=EDM89::Decision{};
        if(!path.Valid()) return FailBool(EDM81::Reason::Path);
        if(!EDM81::SameScope(scope,path.FrozenScope())) return FailBool(EDM81::Reason::Scope);
        if(config.motion.mode!=EDM81::Mode::RetainedPath ||
            config.motion.frame!=EDM81::Frame::MachineCartesian ||
            config.motion.kinematics!=EDM81::Kinematics::DirectJoint ||
            !ValidDirection(config.direction) || !ValidMapping(config.mapping))
            return FailBool(EDM81::Reason::UnsupportedMapping);
        if(config.direction.revision!=scope.frameRevision ||
            config.mapping.revision!=scope.kinematicsRevision)
            return FailBool(EDM81::Reason::Scope);
        if(!ValidConfig(config)) return FailBool(EDM81::Reason::Configuration);
        const double envelope=config.excursion==ExcursionMode::DirectionOffset?
            config.motion.maxRetreatMm:0.0;
        const auto bounds=ValidateMappedBounds(path,config.motion,config.mapping,envelope);
        if(bounds!=EDM81::Reason::None) return FailBool(bounds);
        // A tool-frame direction must be resolvable at every later anchor,
        // including when the output mapping itself is Identity.
        EDM81::Pose minimum{},maximum{}; std::array<double,3U> unit{};
        if(path.NativeBounds(minimum,maximum)!=EDM81::Reason::None ||
            !ResolveDirection(config.direction,config.mapping,minimum,unit) ||
            !ResolveDirection(config.direction,config.mapping,maximum,unit))
            return FailBool(EDM81::Reason::UnsupportedMapping);
        if(config.gapSource.enabled && config.gapSource.identity.producerGeneration!=scope.sourceGeneration)
            return FailBool(EDM81::Reason::Scope);
        if(!gapSource_.Begin(config.gapSource)) return FailBool(EDM81::Reason::Configuration);
        if(config.gapSource.enabled && !gapDecision_.Begin(config.motion.servo))
            return FailBool(EDM81::Reason::Configuration);
        path_=path; config_=config;
        if(!path_.Evaluate(0.0,current_.nominal,current_.ordinal,current_.u) ||
            !ResolveDirection(config.direction,config.mapping,current_.nominal,current_.frozenDirection) ||
            !ComposePose(config.mapping,current_.nominal,current_.frozenDirection,0.0,current_.pose))
            return FailBool(EDM81::Reason::UnsupportedMapping);
        origin_=current_.pose; anchorJoint_=current_.pose; current_.anchorPose=current_.nominal;
        current_.pathLengthMm=path_.LengthMm(); current_.state=EDM81::State::Running;
        seeded_=false; lastSequence_=lastSourceUs_=lastNowUs_=0U;
        highWaterMm_=retreatTargetMm_=0.0; shortDetector_.Reset();
        return true;
    }

    const Snapshot& Step(const EDM81::Input& input) noexcept
    {
        if(!config_.gapSource.enabled) return StepImpl(input);
        if(EDM81::Terminal(current_.state)) return current_;
        if(current_.state==EDM81::State::Idle) return Fail(EDM81::Reason::Lifecycle);
        if(input.event==EDM81::Event::Hold || input.event==EDM81::Event::Reset ||
            input.event==EDM81::Event::Alarm) return Cancel(input.event);
        return Fail(EDM81::Reason::Configuration);
    }

    // control.sequence/nowUs/capturedAtUs describe the 250 us MODEL control
    // clock, explicitly not acquisition evidence. The immutable frame carries
    // its own conversion clock/sequence; only the source qualifier uses those.
    // The qualified held voltage is used on each control step. Neither the
    // original frame nor its capture time is rewritten into a fresh conversion.
    const Snapshot& StepSampled(const EDM81::Input& control,const EDM88::Frame& frame,
        std::uint64_t sourceNowUs) noexcept
    {
        if(EDM81::Terminal(current_.state)) return current_;
        if(current_.state==EDM81::State::Idle) return Fail(EDM81::Reason::Lifecycle);
        if(control.event==EDM81::Event::Hold || control.event==EDM81::Event::Reset ||
            control.event==EDM81::Event::Alarm) return Cancel(control.event);
        if(!config_.gapSource.enabled) return Fail(EDM81::Reason::Configuration);
        if(control.capturedAtUs!=control.nowUs) return Fail(EDM81::Reason::Cadence);
        if(!gapSource_.Observe(frame,sourceNowUs)) return Fail(EDM81::Reason::Sample);
        if(!gapDecision_.Observe(gapSource_.Current().lastFrame)) return Fail(EDM81::Reason::Sample);
        EDM81::Input qualified=control;
        qualified.voltageV=gapSource_.Current().lastFrame.voltageV;
        return StepImpl(qualified);
    }
private:
    const Snapshot& StepImpl(const EDM81::Input& input) noexcept
    {
        using EDM81::State; using EDM81::Reason; using EDM81::Event;
        current_.committed=false;
        if(EDM81::Terminal(current_.state)) return current_;
        if(current_.state==State::Idle) return Fail(Reason::Lifecycle);
        // Cancellation does not require a healthy source or current authority.
        if(input.event==Event::Hold || input.event==Event::Reset || input.event==Event::Alarm)
            return Cancel(input.event);
        if(input.event!=Event::None && input.event!=Event::Jump && input.event!=Event::M30)
            return Fail(Reason::Lifecycle);
        if(!EDM81::SameScope(input.scope,current_.scope)) return Fail(Reason::Scope);
        if(!input.authorityCurrent) return Fail(Reason::Authority);
        if(input.limitMask) return Fail(Reason::AxisLimit);
        if(!input.valid || !EDM81::Finite(input.voltageV) || input.voltageV<0.0)
            return Fail(Reason::Sample);
        if(!input.sequence || !input.capturedAtUs || !input.nowUs || input.capturedAtUs>input.nowUs)
            return Fail(Reason::Sample);
        if(input.nowUs-input.capturedAtUs>config_.motion.maxAgeUs) return Fail(Reason::Stale);
        const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
        if(current_.steps==maximum) return Fail(Reason::CounterOverflow);
        if(seeded_)
        {
            if(lastSequence_==maximum || input.sequence!=lastSequence_+1U)
                return Fail(Reason::Sequence);
            if(input.capturedAtUs<=lastSourceUs_ || input.nowUs<=lastNowUs_ ||
                input.capturedAtUs-lastSourceUs_!=config_.motion.periodUs ||
                input.nowUs-lastNowUs_!=config_.motion.periodUs)
                return Fail(Reason::Cadence);
        }
        const auto& c=config_.motion;
        const auto curve=EDMGapServo::Evaluate(c.servo,input.voltageV,c.referenceV,c.overridePercent);
        if(!curve.valid) return Fail(Reason::NonFinite);
        EDMGapServo::ShortResult shortResult{};
        if(config_.gapSource.enabled) shortResult=gapDecision_.Result();
        else
        {
            // Preserve the original fresh-per-control contract for unsampled profiles.
            EDMGapServo::Sample sample{}; sample.valid=true; sample.voltage=input.voltageV;
            sample.sequence=input.sequence; sample.nowMs=input.capturedAtUs/1000U;
            shortResult=shortDetector_.Step(c.servo,sample,true);
        }
        if(!shortResult.valid) return Fail(Reason::Sample);
        if(input.event==Event::M30)
        {
            if(!seeded_ || current_.state!=State::Running || shortResult.feedInhibited ||
                current_.cursorMm!=current_.pathLengthMm || current_.offsetMm!=0.0)
                return Fail(Reason::PrematureEnd);
            current_.state=State::Completed; current_.reason=Reason::None;
            return current_;
        }
        if(!seeded_)
        {
            if(input.event!=Event::None) return Fail(Reason::Lifecycle);
            AcceptSample(input); seeded_=true; return current_;
        }

        const bool directional=config_.excursion==ExcursionMode::DirectionOffset;
        State nextState=current_.state;
        double nextAnchor=current_.anchorMm,nextTarget=retreatTargetMm_;
        auto nextAnchorPose=current_.anchorPose;
        auto nextAnchorJoint=anchorJoint_;
        auto nextDirection=current_.frozenDirection;
        bool newExcursion=false;
        if(shortResult.shortActive)
        {
            if(nextState==State::Running)
            {
                nextAnchor=current_.cursorMm; newExcursion=true;
            }
            nextTarget=directional?-c.maxRetreatMm:EDM81::Maximum(0.0,nextAnchor-c.maxRetreatMm);
            nextState=State::ShortRetreat;
        }
        else if(nextState==State::ShortRetreat && !shortResult.feedInhibited)
            nextState=State::Returning;
        if(input.event==Event::Jump)
        {
            if(nextState!=State::Running || shortResult.feedInhibited) return Fail(Reason::Lifecycle);
            nextAnchor=current_.cursorMm; newExcursion=true;
            nextTarget=directional?-c.jumpMm:EDM81::Maximum(0.0,nextAnchor-c.jumpMm);
            nextState=State::JumpRetreat;
        }
        if(newExcursion)
        {
            nextAnchorPose=current_.nominal; nextAnchorJoint=current_.pose;
            if(!ResolveDirection(config_.direction,config_.mapping,nextAnchorPose,nextDirection))
                return Fail(Reason::UnsupportedMapping);
        }
        const double dtMinutes=static_cast<double>(c.periodUs)/60000000.0;
        double proposedCursor=current_.cursorMm,proposedOffset=current_.offsetMm;
        if(nextState==State::Running && !shortResult.feedInhibited)
        {
            const double speed=EDM81::Clamp(curve.limitedSpeedMmPerMin,-c.maxScalarMmMin,c.maxScalarMmMin);
            const double floor=EDM81::Maximum(0.0,highWaterMm_-c.maxRetreatMm);
            proposedCursor=EDM81::Clamp(proposedCursor+speed*dtMinutes,floor,current_.pathLengthMm);
        }
        else if(nextState==State::JumpRetreat || nextState==State::ShortRetreat)
        {
            double speed=c.excursionSpeedMmMin;
            if(nextState==State::ShortRetreat)
            {
                const auto retreat=EDMGapServo::EvaluateShortRetreat(c.servo,curve);
                if(!retreat.valid) return Fail(Reason::NonFinite);
                // Fixed demand gets E8 once; curve demand is already scaled.
                const double factor=retreat.usingCurve?1.0:c.overridePercent/100.0;
                speed=EDM81::Minimum(speed,-retreat.signedSpeedMmPerMin*factor);
            }
            double& value=directional?proposedOffset:proposedCursor;
            value=EDM81::Maximum(nextTarget,value-speed*dtMinutes);
            if(value==nextTarget && nextState==State::JumpRetreat) nextState=State::Returning;
        }
        else if(nextState==State::Returning && !shortResult.feedInhibited)
        {
            double& value=directional?proposedOffset:proposedCursor;
            const double target=directional?0.0:nextAnchor;
            value=EDM81::Minimum(target,value+c.excursionSpeedMmMin*dtMinutes);
            if(value==target) nextState=State::Running;
        }
        if(!EDM81::Finite(proposedCursor) || !EDM81::Finite(proposedOffset)) return Fail(Reason::NonFinite);
        EDM81::Pose nominal{},pose{}; unsigned ordinal=0U; double u=0.0;
        if(!path_.Evaluate(proposedCursor,nominal,ordinal,u)) return Fail(Reason::Path);
        const bool inFrozenExcursion=directional &&
            (newExcursion || current_.state!=State::Running);
        if(inFrozenExcursion)
        {
            // The frozen six-axis anchor has one exact restore point. Cutting
            // may resume only on a later Step, after that restore commits.
            proposedCursor=nextAnchor; nominal=nextAnchorPose;
        }
        if(!ComposePose(config_.mapping,nominal,nextDirection,proposedOffset,pose))
            return Fail(Reason::UnsupportedMapping);
        if(inFrozenExcursion && proposedOffset==0.0) pose=nextAnchorJoint;
        const auto traversal=ValidateMappedTraversal(path_,c,config_.mapping,
            current_.cursorMm,proposedCursor,current_.offsetMm,proposedOffset,nextDirection);
        if(traversal!=Reason::None) return Fail(traversal);
        const auto position=ValidatePosition(pose);
        if(position!=Reason::None) return Fail(position);
        current_.pose=pose; current_.nominal=nominal; current_.ordinal=ordinal; current_.u=u;
        current_.cursorMm=proposedCursor; current_.offsetMm=proposedOffset;
        current_.anchorMm=nextAnchor; current_.anchorPose=nextAnchorPose;
        current_.frozenDirection=nextDirection; current_.state=nextState;
        anchorJoint_=nextAnchorJoint; retreatTargetMm_=nextTarget;
        if(nextState==State::Running) highWaterMm_=EDM81::Maximum(highWaterMm_,proposedCursor);
        AcceptSample(input); return current_;
    }

public:
    const Snapshot& Cancel(EDM81::Event event) noexcept
    {
        current_.committed=false;
        if(EDM81::Terminal(current_.state)) return current_;
        if(event!=EDM81::Event::Hold && event!=EDM81::Event::Reset && event!=EDM81::Event::Alarm)
            return Fail(EDM81::Reason::Lifecycle);
        current_.state=EDM81::State::Cancelled;
        current_.reason=event==EDM81::Event::Hold?EDM81::Reason::Hold:
            (event==EDM81::Event::Reset?EDM81::Reason::Reset:EDM81::Reason::Alarm);
        return current_;
    }
private:
    static bool ValidConfig(const Config& config) noexcept
    {
        const auto& c=config.motion;
        if(!EDM88::Valid(config.gapSource)) return false;
        if(config.excursion!=ExcursionMode::RetainedPath && config.excursion!=ExcursionMode::DirectionOffset)
            return false;
        if(!EDMGapServo::ValidateProfile(c.servo) || !EDM81::Finite(c.referenceV) || c.referenceV<0.0 ||
            !EDM81::Finite(c.overridePercent) || c.overridePercent<0.0 || c.overridePercent>100.0 ||
            !EDM81::Finite(c.maxScalarMmMin) || c.maxScalarMmMin<=0.0 ||
            !EDM81::Finite(c.maxRetreatMm) || c.maxRetreatMm<=0.0 ||
            !EDM81::Finite(c.jumpMm) || c.jumpMm<=0.0 || c.jumpMm>c.maxRetreatMm ||
            !EDM81::Finite(c.excursionSpeedMmMin) || c.excursionSpeedMmMin<=0.0 ||
            c.excursionSpeedMmMin>c.maxScalarMmMin || c.periodUs!=250U ||
            c.maxAgeUs<c.periodUs || c.maxAgeUs>5000U) return false;
        for(const auto& b:c.axes)
            if(!EDM81::Finite(b.minimum) || !EDM81::Finite(b.maximum) || b.minimum>=b.maximum ||
                !EDM81::Finite(b.maxTravel) || b.maxTravel<=0.0 ||
                !EDM81::Finite(b.maxVelocityPerMin) || b.maxVelocityPerMin<=0.0) return false;
        return true;
    }
    EDM81::Reason ValidatePosition(const EDM81::Pose& pose) const noexcept
    {
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
        {
            const auto& b=config_.motion.axes[axis];
            if(!EDM81::Finite(pose[axis])) return EDM81::Reason::NonFinite;
            if(pose[axis]<b.minimum || pose[axis]>b.maximum) return EDM81::Reason::AxisLimit;
            const double travel=std::fabs(pose[axis]-origin_[axis]);
            if(!EDM81::Finite(travel) || travel>b.maxTravel) return EDM81::Reason::TravelLimit;
            const double speed=std::fabs(pose[axis]-current_.pose[axis])*
                (60000000.0/static_cast<double>(config_.motion.periodUs));
            if(!EDM81::Finite(speed) || speed>b.maxVelocityPerMin*(1.0+1e-12))
                return EDM81::Reason::VelocityLimit;
        }
        return EDM81::Reason::None;
    }
    void AcceptSample(const EDM81::Input& input) noexcept
    {
        lastSequence_=input.sequence; lastSourceUs_=input.capturedAtUs; lastNowUs_=input.nowUs;
        ++current_.steps; current_.committed=true;
    }
    const Snapshot& Fail(EDM81::Reason reason) noexcept
    {
        // Never replace accepted pose/progress with an invalid trial.
        current_.state=EDM81::State::Fault; current_.reason=reason; current_.committed=false;
        return current_;
    }
    bool FailBool(EDM81::Reason reason) noexcept { Fail(reason); return false; }
    Config config_{};
    SynchronizedPath path_{};
    Snapshot current_{};
    EDM81::Pose origin_{},anchorJoint_{};
    EDMGapServo::ShortDetector shortDetector_{};
    EDM88::Qualifier gapSource_{};
    EDM89::Decision gapDecision_{};
    double highWaterMm_=0.0,retreatTargetMm_=0.0;
    std::uint64_t retiredRun_=0U,lastSequence_=0U,lastSourceUs_=0U,lastNowUs_=0U;
    bool seeded_=false;
};
static_assert(sizeof(Kernel)<12288U,"EDM84 fixed-size numerical kernel budget changed");
}
