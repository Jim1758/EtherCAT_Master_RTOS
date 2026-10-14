#pragma once
#include "EDMMultiAxisKernel.h"

// EDM81 built-in numerical verification. Advance performs at most 128 virtual
// 250 us samples, independent of the NC service clock. No hardware is sampled,
// armed or commanded. The installed curve, reference and E8 are copied once.
namespace EDM81
{
struct SimulationReport
{
    bool complete=false, passed=false;
    std::uint32_t casesPassed=0, checks=0, virtualTicks=0;
    const char* failedCase=nullptr;
};

class SimulationSuite
{
public:
    static constexpr unsigned CaseCount=33U;
    static constexpr unsigned MaximumTicksPerAdvance=128U;
    const SimulationReport& Report() const noexcept { return report_; }
    static const char* CaseName(unsigned n) noexcept
    {
        static const char* const names[CaseCount]={
            "SINGLE_Z_PATH", "XYZ_COMMON_CURSOR", "XYZC_COMMON_CURSOR",
            "XYZCUV_COMMON_CURSOR", "ARBITRARY_XYZ_DIRECTION", "UNWRAPPED_ROTARY",
            "GAP_REVERSE_REPLAYS_PATH", "SYNCHRONOUS_JUMP_RETURN", "SHORT_RETURN_ANCHOR",
            "PERSISTENT_SHORT_NO_ADVANCE", "HOLD_TERMINAL", "RESET_TERMINAL",
            "ALARM_TERMINAL", "PREMATURE_M30", "ANY_AXIS_LIMIT_ATOMIC",
            "TRAVEL_BOUND_ADMISSION", "VELOCITY_BOUND_ATOMIC", "NONFINITE_GAP",
            "STALE_SOURCE", "DUPLICATE_SEQUENCE", "SKIPPED_SEQUENCE",
            "CADENCE_MISMATCH", "CLOCK_ROLLBACK", "OWNER_LEASE_CHANGED",
            "RECIPE_GENERATION_ABA", "FRAME_GENERATION_CHANGED", "AUTHORITY_REVOKED",
            "TCP_MAPPING_NOT_INSTALLED", "INVALID_DIRECTION", "PURE_ROTARY_METRIC",
            "RETIRED_RUN_REJECTED", "NONFINITE_PATH", "ALL_SCOPE_FIELDS_FENCED"
        };
        return n<CaseCount?names[n]:"SUITE_COMPLETE";
    }
    bool Start(const EDMGapServo::Profile& profile,double referenceV,double overridePercent) noexcept
    {
        *this=SimulationSuite{};
        servo_=profile; referenceV_=referenceV; overridePercent_=overridePercent;
        const auto forward=EDMGapServo::Evaluate(profile,referenceV+10.0,referenceV,overridePercent);
        const auto reverse=EDMGapServo::Evaluate(profile,referenceV-10.0,referenceV,overridePercent);
        // This diagnostic fixture needs both signs and a configured short
        // detector. Reject unsuitable test conditions instead of reporting a
        // vacuous motion PASS; these restrictions do not change legacy EDM.
        if(!EDMGapServo::ValidateProfile(profile) || !Finite(referenceV) || referenceV<10.0 ||
            !forward.valid || !reverse.valid || forward.limitedSpeedMmPerMin<=0.01 ||
            reverse.limitedSpeedMmPerMin>=-0.01 || !profile.shortMachiningEnabled ||
            profile.shortEnterMs>100U || profile.shortExitMs>100U ||
            profile.shortMachiningV+profile.shortHysteresisV>=referenceV-10.0 ||
            profile.shortRetreatMmPerMin<=0.01)
        { Fail("SIM_FIXTURE_RECIPE_UNSUPPORTED"); return false; }
        started_=true;
        return true;
    }
    void Cancel() noexcept
    {
        kernel_.Cancel(Event::Reset);
        started_=false; report_.complete=false; report_.passed=false;
        report_.failedCase="CANCELLED";
    }
    const SimulationReport& Advance() noexcept
    {
        if(!started_ || report_.complete) return report_;
        for(unsigned i=0;i<MaximumTicksPerAdvance && !report_.complete;++i)
        {
            if(!caseStarted_) { BeginCase(); if(report_.complete) break; }
            if(!caseStarted_) continue;
            TickCase();
        }
        return report_;
    }
private:
    bool Check(bool condition) noexcept
    {
        ++report_.checks;
        if(!condition) Fail(CaseName(case_));
        return condition;
    }
    void Fail(const char* name) noexcept
    { report_.complete=true; report_.passed=false; report_.failedCase=name; }
    void FinishCase() noexcept
    {
        if(report_.complete) return;
        ++report_.casesPassed; ++case_; caseStarted_=false;
        if(case_==CaseCount) { report_.complete=true; report_.passed=true; }
    }
    void BaseFixture() noexcept
    {
        config_=Config{}; config_.servo=servo_; config_.referenceV=referenceV_;
        config_.overridePercent=overridePercent_; config_.jumpMm=0.0001;
        config_.maxRetreatMm=0.0003; config_.excursionSpeedMmMin=3.0;
        for(unsigned a=0;a<AxisCount;++a)
        {
            config_.axes[a].minimum=a<3U?-1.0:-1000.0;
            config_.axes[a].maximum=a<3U?1.0:1000.0;
            config_.axes[a].maxTravel=1.0;
            config_.axes[a].maxVelocityPerMin=20.0;
        }
        plan_=Plan{}; plan_.count=2;
        plan_.point[0]=Pose{{0.0,0.0,0.0,359.998,10.0,-5.0}};
        plan_.point[1]=Pose{{0.001,0.002,0.002,359.999,10.002,-5.001}};
        plan_.point[2]=Pose{{0.003,0.003,0.004,360.002,10.001,-5.002}};
        scope_=Scope{};
        scope_.run=case_+1U; scope_.cache=2; scope_.dispatch=3; scope_.executionEpoch=4;
        scope_.ownerLease=5; scope_.configRevision=6; scope_.recipeGeneration=7;
        scope_.frameRevision=8; scope_.kinematicsRevision=9; scope_.sourceGeneration=10;
        sequence_=0; tick_=0; phase_=0; dwell_=0; anchor_=0.0;
        kernel_=Kernel{};
    }
    void BeginCase() noexcept
    {
        BaseFixture();
        if(case_==0U)
        {
            for(unsigned p=0;p<=plan_.count;++p)
            {
                plan_.point[p][0]=plan_.point[p][1]=0.0;
                for(unsigned a=3;a<AxisCount;++a) plan_.point[p][a]=plan_.point[0][a];
            }
        }
        if(case_==1U || case_==2U)
            for(unsigned p=0;p<=plan_.count;++p)
                for(unsigned a=case_==1U?3U:4U;a<AxisCount;++a)
                    plan_.point[p][a]=plan_.point[0][a];
        if(case_==4U)
        {
            config_.mode=Mode::DirectionOffset; config_.directionOrigin=plan_.point[0];
            config_.machiningUnit={{1.0/3.0,2.0/3.0,-2.0/3.0}};
            config_.directionExtentMm=0.004;
        }
        if(case_==15U) config_.axes[5].maxTravel=0.0001;
        if(case_==16U) config_.axes[5].maxVelocityPerMin=1e-9;
        if(case_==27U) config_.kinematics=Kinematics::TCP;
        if(case_==28U)
        { config_.mode=Mode::DirectionOffset; config_.machiningUnit={{1.0,1.0,1.0}}; }
        if(case_==29U)
            for(unsigned a=0;a<3U;++a) plan_.point[1][a]=plan_.point[0][a];
        if(case_==31U) plan_.point[1][5]=(std::numeric_limits<double>::quiet_NaN)();
        const bool admitted=kernel_.Begin(config_,scope_,plan_);
        if(case_==15U || case_==27U || case_==28U || case_==29U || case_==31U)
        {
            if(Check(!admitted && kernel_.Current().state==State::Fault &&
                !kernel_.Current().committed && !kernel_.Current().PhysicalPermit() &&
                !kernel_.Current().DischargeEnabled())) FinishCase();
            return;
        }
        if(!Check(admitted)) return;
        caseStarted_=true;
    }
    Input NextInput(double voltage,Event event=Event::None) noexcept
    {
        Input input{}; input.scope=scope_; input.sequence=++sequence_;
        input.capturedAtUs=1000000U+sequence_*250U; input.nowUs=input.capturedAtUs;
        input.voltageV=voltage; input.event=event; return input;
    }
    bool GeometrySame(const Snapshot& s) noexcept
    {
        if(config_.mode==Mode::DirectionOffset)
        {
            for(unsigned a=0;a<AxisCount;++a)
            {
                const double expected=config_.directionOrigin[a]+(a<3U?config_.machiningUnit[a]*s.cursorMm:0.0);
                if(std::fabs(s.pose[a]-expected)>1e-10) return false;
            }
            return true;
        }
        double distance=0.0;
        for(unsigned p=0;p<plan_.count;++p)
        {
            double lengthSquared=0.0;
            for(unsigned a=0;a<3U;++a)
            { const double d=plan_.point[p+1U][a]-plan_.point[p][a]; lengthSquared+=d*d; }
            const double length=std::sqrt(lengthSquared);
            if(s.cursorMm>distance+length && p+1U<plan_.count) { distance+=length; continue; }
            const double u=(s.cursorMm-distance)/length;
            for(unsigned a=0;a<AxisCount;++a)
                if(std::fabs(s.pose[a]-(plan_.point[p][a]+u*(plan_.point[p+1U][a]-plan_.point[p][a])))>1e-10)
                    return false;
            return true;
        }
        return false;
    }
    const Snapshot& Step(Input input) noexcept
    {
        ++report_.virtualTicks; ++tick_;
        const auto& result=kernel_.Step(input);
        Check(!result.PhysicalPermit() && !result.DischargeEnabled());
        if(tick_>20000U) Fail("SIM_CASE_TICK_BUDGET");
        return result;
    }
    void TickCase() noexcept
    {
        const Snapshot before=kernel_.Current();
        if(case_<=9U) { MotionCase(before); return; }
        Input input=NextInput(referenceV_+10.0);
        if(tick_==0U)
        { const auto& s=Step(input); Check(s.state==State::Running && s.cursorMm==0.0); return; }
        Reason expected=Reason::None;
        if(case_>=10U && case_<=12U)
        {
            input.event=case_==10U?Event::Hold:(case_==11U?Event::Reset:Event::Alarm);
            const Snapshot terminal=Step(input);
            if(!Check(terminal.state==State::Cancelled && !terminal.committed && terminal.pose==before.pose)) return;
            const auto& after=Step(NextInput(referenceV_+10.0));
            if(Check(after.state==State::Cancelled && !after.committed && after.pose==before.pose)) FinishCase();
            return;
        }
        switch(case_)
        {
        case 13: input.event=Event::M30; expected=Reason::PrematureEnd; break;
        case 14: input.limitMask=1U<<5U; expected=Reason::AxisLimit; break;
        case 16: expected=Reason::VelocityLimit; break;
        case 17: input.voltageV=(std::numeric_limits<double>::quiet_NaN)(); expected=Reason::Sample; break;
        case 18: input.nowUs+=config_.maxAgeUs+1U; expected=Reason::Stale; break;
        case 19: --input.sequence; expected=Reason::Sequence; break;
        case 20: ++input.sequence; expected=Reason::Sequence; break;
        case 21: ++input.capturedAtUs; ++input.nowUs; expected=Reason::Cadence; break;
        case 22: input.capturedAtUs-=500U; input.nowUs-=500U; expected=Reason::Cadence; break;
        case 23: ++input.scope.ownerLease; expected=Reason::Scope; break;
        case 24: input.scope.recipeGeneration+=2U; expected=Reason::Scope; break;
        case 25: ++input.scope.frameRevision; expected=Reason::Scope; break;
        case 26: input.authorityCurrent=false; expected=Reason::Authority; break;
        case 30:
            kernel_.Cancel(Event::Reset);
            if(!Check(!kernel_.Begin(config_,scope_,plan_) && kernel_.Current().reason==Reason::RetiredRun)) return;
            ++scope_.run;
            if(Check(kernel_.Begin(config_,scope_,plan_))) FinishCase();
            return;
        case 32:
            switch(phase_)
            {
            case 0: ++input.scope.run; break; case 1: ++input.scope.cache; break;
            case 2: ++input.scope.dispatch; break; case 3: ++input.scope.executionEpoch; break;
            case 4: ++input.scope.ownerLease; break; case 5: ++input.scope.configRevision; break;
            case 6: ++input.scope.recipeGeneration; break; case 7: ++input.scope.frameRevision; break;
            case 8: ++input.scope.kinematicsRevision; break; default: ++input.scope.sourceGeneration; break;
            }
            expected=Reason::Scope; break;
        default: Fail("SIM_CASE_UNIMPLEMENTED"); return;
        }
        const Snapshot failed=Step(input);
        if(!Check(failed.state==State::Fault && failed.reason==expected && !failed.committed &&
            failed.pose==before.pose && failed.cursorMm==before.cursorMm)) return;
        if(case_==32U && ++phase_<10U)
        {
            ++scope_.run;
            if(!Check(kernel_.Begin(config_,scope_,plan_))) return;
            tick_=0; sequence_=0; return;
        }
        const auto& unchanged=Step(NextInput(referenceV_+10.0));
        if(Check(unchanged.state==State::Fault && !unchanged.committed && unchanged.pose==failed.pose)) FinishCase();
    }
    void MotionCase(const Snapshot& before) noexcept
    {
        double voltage=referenceV_+10.0; Event event=Event::None;
        if(case_==6U)
        {
            if(phase_==0U && before.cursorMm>=0.001) { anchor_=before.cursorMm; phase_=1U; }
            if(phase_==1U)
            {
                voltage=referenceV_-10.0;
                if(before.cursorMm<=anchor_-0.0002) { phase_=2U; voltage=referenceV_+10.0; }
            }
        }
        if(case_>=7U && case_<=9U)
        {
            if(phase_==0U && before.cursorMm>=0.001)
            { anchor_=before.cursorMm; phase_=1U; if(case_==7U) event=Event::Jump; }
            if(phase_==1U && case_!=7U) voltage=0.0;
            if(phase_==1U && before.state==State::ShortRetreat && before.cursorMm<=anchor_-config_.maxRetreatMm+1e-14)
            {
                if(++dwell_>=64U)
                {
                    if(case_==9U)
                    { if(Check(before.state==State::ShortRetreat && before.cursorMm<anchor_)) FinishCase(); return; }
                    phase_=2U; voltage=referenceV_+10.0;
                }
            }
        }
        if(case_<=6U && before.cursorMm==before.pathLengthMm && tick_>0U) event=Event::M30;
        const Snapshot result=Step(NextInput(voltage,event));
        if(report_.complete) return;
        if(!Check(result.state!=State::Fault && result.state!=State::Cancelled && GeometrySame(result))) return;
        if(tick_==1U && !Check(result.cursorMm==0.0)) return;
        if(case_>=7U && case_<=9U && phase_>0U)
        {
            if(!Check(result.cursorMm<=anchor_+1e-14)) return;
            if(result.state!=State::Running && !Check(result.anchorMm==anchor_)) return;
            if(result.state==State::Running && (before.state==State::Returning || before.state==State::JumpRetreat))
            {
                if(Check(result.cursorMm==anchor_ && GeometrySame(result))) FinishCase();
                return;
            }
        }
        if(result.state==State::Completed)
        {
            if(case_==6U && !Check(phase_==2U)) return;
            if(case_==5U && !Check(result.pose[3]>360.0 && result.pose[3]==plan_.point[2][3])) return;
            if(Check(result.cursorMm==result.pathLengthMm && !result.committed)) FinishCase();
        }
    }
    SimulationReport report_{};
    EDMGapServo::Profile servo_{};
    double referenceV_=75.0,overridePercent_=75.0,anchor_=0.0;
    Kernel kernel_{}; Config config_{}; Scope scope_{}; Plan plan_{};
    std::uint64_t sequence_=0;
    unsigned case_=0,tick_=0,phase_=0,dwell_=0;
    bool started_=false,caseStarted_=false;
};
static_assert(sizeof(SimulationSuite)<8192U,"EDM81 simulation storage must stay bounded");
}
