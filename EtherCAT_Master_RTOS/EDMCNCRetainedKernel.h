#pragma once

#include "EDMCNCRetainedPath.h"

namespace EDM82
{
struct Snapshot
{
    EDM81::State state=EDM81::State::Idle;
    EDM81::Reason reason=EDM81::Reason::None;
    EDM81::Scope scope{};
    EDM81::Pose pose{};
    double cursorMm=0.0,anchorMm=0.0,pathLengthMm=0.0;
    unsigned ordinal=0U;
    double u=0.0;
    std::uint64_t steps=0U;
    bool committed=false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
    constexpr bool DischargeEnabled() const noexcept { return false; }
};

// One numerical owner, one canonical path cursor. EDM81 retains sole ownership
// of GAP/short/jump lifecycle. Its synthetic X coordinate is distance in mm;
// it is never a machine-axis command. A candidate cursor is mapped through the
// original CNC evaluator, checked, then committed with all six native values.
// This value has no RT transport, output permit, acceleration/jerk or stopping
// envelope. It cannot authorize a CNC queue, EtherCAT or drive operation.
class Kernel
{
public:
    const Snapshot& Current() const noexcept { return current_; }
    const RetainedPath& Path() const noexcept { return path_; }
    bool Begin(const EDM81::Config& config,const EDM81::Scope& scope,
        const RetainedPath& path) noexcept
    {
        if(current_.state!=EDM81::State::Idle && !EDM81::Terminal(current_.state))
            return FailBool(EDM81::Reason::Lifecycle);
        if(!EDM81::ValidScope(scope)) return FailBool(EDM81::Reason::Scope);
        if(scope.run<=retiredRun_) return FailBool(EDM81::Reason::RetiredRun);
        retiredRun_=scope.run; current_=Snapshot{}; current_.scope=scope;
        if(!path.Valid()) return FailBool(EDM81::Reason::Path);
        if(!EDM81::SameScope(scope,path.FrozenScope())) return FailBool(EDM81::Reason::Scope);
        if(config.mode!=EDM81::Mode::RetainedPath ||
            config.frame!=EDM81::Frame::MachineCartesian ||
            config.kinematics!=EDM81::Kinematics::DirectJoint)
            return FailBool(EDM81::Reason::UnsupportedMapping);
        for(const auto& b:config.axes)
            if(!EDM81::Finite(b.minimum) || !EDM81::Finite(b.maximum) || b.minimum>=b.maximum ||
                !EDM81::Finite(b.maxTravel) || b.maxTravel<=0.0 ||
                !EDM81::Finite(b.maxVelocityPerMin) || b.maxVelocityPerMin<=0.0)
                return FailBool(EDM81::Reason::Configuration);
        const auto bounds=path.ValidateBounds(config);
        if(bounds!=EDM81::Reason::None) return FailBool(bounds);
        EDM81::Config scalarConfig=config;
        for(auto& b:scalarConfig.axes)
        { b.minimum=-1.0; b.maximum=1.0; b.maxTravel=1.0;
          // EDM83: accumulated cursor subtraction can round one nominal step
          // slightly upward. This relative allowance applies ONLY to the
          // private scalar cursor's redundant numerical velocity check.
          // maxScalarMmMin still clamps demand unchanged; the six native
          // axis bounds and ValidateTraversal below retain exact limits.
          b.maxVelocityPerMin=config.maxScalarMmMin*(1.0+1e-9); }
        scalarConfig.axes[0].minimum=0.0;
        scalarConfig.axes[0].maximum=path.LengthMm();
        scalarConfig.axes[0].maxTravel=path.LengthMm();
        EDM81::Plan scalarPlan{}; scalarPlan.count=1U;
        scalarPlan.point[1][0]=path.LengthMm();
        if(!scalar_.Begin(scalarConfig,scope,scalarPlan))
            return FailBool(scalar_.Current().reason);
        path_=path; config_=config;
        if(!path_.Evaluate(0.0,current_.pose,current_.ordinal,current_.u))
            return FailBool(EDM81::Reason::Path);
        current_.pathLengthMm=path_.LengthMm(); current_.state=EDM81::State::Running;
        return true;
    }

    const Snapshot& Step(const EDM81::Input& input) noexcept
    {
        current_.committed=false;
        if(EDM81::Terminal(current_.state)) return current_;
        if(current_.state==EDM81::State::Idle) return Fail(EDM81::Reason::Lifecycle);
        const auto& candidate=scalar_.Step(input);
        if(candidate.state==EDM81::State::Fault) return Fail(candidate.reason);
        if(candidate.state==EDM81::State::Cancelled)
        { current_.state=candidate.state; current_.reason=candidate.reason; return current_; }
        EDM81::Pose pose{}; unsigned ordinal=0U; double u=0.0;
        if(!path_.Evaluate(candidate.cursorMm,pose,ordinal,u)) return Fail(EDM81::Reason::Path);
        const auto traversal=path_.ValidateTraversal(current_.cursorMm,candidate.cursorMm,config_);
        if(traversal!=EDM81::Reason::None) return Fail(traversal);
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
        {
            const auto& b=config_.axes[axis];
            if(!EDM81::Finite(pose[axis])) return Fail(EDM81::Reason::NonFinite);
            if(pose[axis]<b.minimum || pose[axis]>b.maximum) return Fail(EDM81::Reason::AxisLimit);
            if(std::fabs(pose[axis]-path_.Row(0U)->startMCS[axis])>b.maxTravel)
                return Fail(EDM81::Reason::TravelLimit);
            const double speed=std::fabs(pose[axis]-current_.pose[axis])*
                (60000000.0/static_cast<double>(config_.periodUs));
            if(!EDM81::Finite(speed) || speed>b.maxVelocityPerMin*(1.0+1e-12))
                return Fail(EDM81::Reason::VelocityLimit);
        }
        current_.pose=pose; current_.ordinal=ordinal; current_.u=u;
        current_.cursorMm=candidate.cursorMm; current_.anchorMm=candidate.anchorMm;
        current_.steps=candidate.steps; current_.state=candidate.state;
        current_.reason=candidate.reason; current_.committed=candidate.committed;
        return current_;
    }

    const Snapshot& Cancel(EDM81::Event event) noexcept
    {
        current_.committed=false;
        if(EDM81::Terminal(current_.state)) return current_;
        const auto& result=scalar_.Cancel(event);
        if(result.state==EDM81::State::Fault) return Fail(result.reason);
        current_.state=result.state; current_.reason=result.reason;
        return current_;
    }
private:
    const Snapshot& Fail(EDM81::Reason reason) noexcept
    {
        // Keep the last accepted pose/cursor/anchor/ordinal/steps coherent.
        // The scalar trial may have advanced, but a fault is terminal here.
        current_.state=EDM81::State::Fault; current_.reason=reason;
        current_.committed=false; scalar_.Cancel(EDM81::Event::Alarm);
        return current_;
    }
    bool FailBool(EDM81::Reason reason) noexcept { Fail(reason); return false; }
    RetainedPath path_{};
    EDM81::Kernel scalar_{};
    EDM81::Config config_{};
    Snapshot current_{};
    std::uint64_t retiredRun_=0U;
};
static_assert(sizeof(Kernel)<9216U,"EDM82 retained kernel budget changed");
}
