#pragma once
#include "EDMCNCRetainedKernel.h"

// EDM82: private diagnostic geometry built by the ORIGINAL CNC builders.
// It never borrows the live NC retained history and cannot publish axis output.
// Advance is bounded by 128 virtual samples, not a real-time scheduling claim.
namespace EDM82
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
    static constexpr unsigned CaseCount=32U;
    static constexpr unsigned MaximumTicksPerAdvance=128U;
    const SimulationReport& Report() const noexcept { return report_; }
    static const char* CaseName(unsigned n) noexcept
    {
        static const char* const names[CaseCount]={
            "CNC_XYZ_LINE", "CNC_XY_CCW_ARC", "CNC_XY_CW_ARC", "CNC_FULL_CIRCLE",
            "CNC_LINE_ARC_LINE", "CROSS_SEGMENT_GAP_REVERSE", "CROSS_SEGMENT_JUMP_RETURN",
            "CROSS_SEGMENT_SHORT_RETURN", "PERSISTENT_SHORT_STAYS_RETRACTED",
            "ARC_INTERIOR_SOFT_LIMIT", "ARC_INTERIOR_TRAVEL_LIMIT", "ARC_VELOCITY_LIMIT",
            "IMMUTABLE_SOURCE_COPY", "PENDING_SOURCE_REJECTED", "DISCONNECTED_SOURCE_REJECTED",
            "CAPACITY_NO_TRUNCATION", "INVALID_CAPTURE_SCOPE", "CAPTURE_BEGIN_SCOPE_MISMATCH",
            "HOLD_CANCELS_ATOMICALLY", "RESET_CANCELS_ATOMICALLY", "ALARM_CANCELS_ATOMICALLY",
            "PREMATURE_M30", "HELD_V_AXIS_LIMIT", "STALE_SAMPLE", "NONFINITE_SAMPLE",
            "TCP_MAPPING_REJECTED", "ROTARY_CNC_LINE_REJECTED", "CURSOR_DOMAIN_REJECTED",
            "RETIRED_RUN_REJECTED", "SOURCE_GENERATION_ABA", "INVALID_SCALE_REJECTED",
            "CANONICAL_ENDPOINTS_EXACT"
        };
        return n<CaseCount?names[n]:"SUITE_COMPLETE";
    }
    bool Start(const EDMGapServo::Profile& profile,double referenceV,double overridePercent) noexcept
    {
        Cancel();
        report_=SimulationReport{}; case_=0; caseStarted_=false;
        servo_=profile; referenceV_=referenceV; overridePercent_=overridePercent;
        const auto f=EDMGapServo::Evaluate(profile,referenceV+10.0,referenceV,overridePercent);
        const auto r=EDMGapServo::Evaluate(profile,referenceV-10.0,referenceV,overridePercent);
        if(!EDMGapServo::ValidateProfile(profile) || !EDM81::Finite(referenceV) || referenceV<10.0 ||
            !f.valid || !r.valid ||
            !profile.shortMachiningEnabled || profile.shortEnterMs>100U || profile.shortExitMs>100U ||
            profile.shortMachiningV+profile.shortHysteresisV>=referenceV-10.0 || profile.shortRetreatMmPerMin<=0.01)
        { Fail("EDM82_FIXTURE_RECIPE_UNSUPPORTED"); return false; }
        const auto shortCurve=EDMGapServo::Evaluate(profile,0.0,referenceV,overridePercent);
        const auto shortDemand=EDMGapServo::EvaluateShortRetreat(profile,shortCurve);
        const double shortSpeed=-shortDemand.signedSpeedMmPerMin*(shortDemand.usingCurve?1.0:overridePercent/100.0);
        // The longest fixture cut/reversal is below 0.007 mm; these explicit
        // diagnostic prerequisites keep every case below 20000 virtual ticks.
        // A slow but valid machining curve is not a canonical-path fault.
        // This restriction belongs only to the diagnostic, not EDM operation.
        if(f.limitedSpeedMmPerMin<0.1 || r.limitedSpeedMmPerMin>-0.1 ||
            !shortCurve.valid || !shortDemand.valid || !EDM81::Finite(shortSpeed) || shortSpeed<0.05)
        { Fail("EDM82_FIXTURE_SPEED_BELOW_BUDGET"); return false; }
        started_=true; return true;
    }
    void Cancel() noexcept
    {
        kernel_.Cancel(EDM81::Event::Reset);
        started_=false; caseStarted_=false;
        report_.complete=report_.passed=false; report_.failedCase="CANCELLED";
    }
    const SimulationReport& Advance() noexcept
    {
        if(!started_ || report_.complete) return report_;
        const std::uint32_t initialTicks=report_.virtualTicks;
        for(unsigned i=0;i<MaximumTicksPerAdvance && !report_.complete;++i)
        {
            // A terminal-fault assertion may issue a second sample to prove
            // no resume. Reserve both slots before starting that case tick.
            if(report_.virtualTicks-initialTicks>=MaximumTicksPerAdvance-1U) break;
            if(!caseStarted_) { BeginCase(); if(report_.complete) break; }
            if(caseStarted_) TickCase();
        }
        return report_;
    }
private:
    using NativePose=std::array<double,8>;
    bool Check(bool value) noexcept
    { ++report_.checks; if(!value) Fail(CaseName(case_)); return value; }
    void Fail(const char* reason) noexcept
    { report_.complete=true; report_.passed=false; report_.failedCase=reason; }
    void FinishCase() noexcept
    {
        if(report_.complete) return;
        kernel_.Cancel(EDM81::Event::Reset);
        ++report_.casesPassed; ++case_; caseStarted_=false;
        if(case_==CaseCount) { report_.complete=true; report_.passed=true; }
    }
    NativePose Origin() const noexcept
    { return NativePose{{0.0,0.0,0.0002,359.998,10.0,-5.0,0.0,0.0}}; }
    bool AppendLine(const NativePose& a,const NativePose& b) noexcept
    {
        NCPathCoreFeedLineInput input{}; input.startMCS=a; input.endMCS=b;
        input.axisMask=7U; input.feedMMMin=5.0; input.maxVelocityPPS.fill(100000.0);
        for(unsigned i=0;i<8U;++i)
        { input.startPulse[i]=a[i]*scales_[i]; input.endPulse[i]=b[i]*scales_[i]; }
        NCPathCoreFeedLineV2 line{}; NCPathCoreRetainedGeometry row{};
        return BuildNCPathCoreFeedLine(input,line)==NCPathCoreFeedLineCode::BUILT_LINE &&
            BuildNCPathCoreRetainedLine(line,row) && source_.Append(row,0x3FU,scales_);
    }
    bool AppendArc(const NativePose& a,const NativePose& b,int direction,bool fullCircle=false) noexcept
    {
        NCPathCoreFeedArcInput input{}; input.startMCS=a; input.endMCS=b;
        input.centerOffsetMM={{-a[0],-a[1]}}; input.pulsePerMM={{scales_[0],scales_[1]}};
        input.maxVelocityPPS={{100000.0,100000.0}}; input.feedMMMin=5.0;
        input.direction=direction; input.fullCircle=fullCircle;
        for(unsigned i=0;i<8U;++i)
        { input.startPulse[i]=a[i]*scales_[i]; input.endPulse[i]=b[i]*scales_[i]; }
        NCPathCoreFeedArcV2 arc{}; NCPathCoreRetainedGeometry row{};
        return BuildNCPathCoreFeedArc(input,arc)==NCPathCoreFeedArcCode::BUILT_ARC &&
            BuildNCPathCoreRetainedArc(arc,row) && source_.Append(row,0x3FU,scales_);
    }
    bool BuildFixture() noexcept
    {
        source_.Clear(); scales_.fill(1000.0);
        NativePose a=Origin(), b=a, c=a, d=a;
        b[0]=0.001; c[0]=-0.001; d[0]=-0.002; d[1]=0.001; d[2]+=0.0005;
        if(case_==0U) { b[1]=0.002; b[2]+=0.003; return AppendLine(a,b); }
        if(case_==1U || case_==2U || case_==3U || case_==9U || case_==10U || case_==11U)
            return AppendArc(b,case_==3U?b:c,case_==2U?-1:1,case_==3U);
        return AppendLine(a,b) && AppendArc(b,c,1) && AppendLine(c,d);
    }
    void BeginCase() noexcept
    {
        kernel_.Cancel(EDM81::Event::Reset);
        if(runSerial_==(std::numeric_limits<std::uint64_t>::max)()) { Fail("EDM82_RUN_COUNTER_OVERFLOW"); return; }
        scope_=EDM81::Scope{}; scope_.run=++runSerial_; scope_.cache=2; scope_.dispatch=3;
        scope_.executionEpoch=4; scope_.ownerLease=5; scope_.configRevision=6;
        scope_.recipeGeneration=7; scope_.frameRevision=8; scope_.kinematicsRevision=9; scope_.sourceGeneration=10;
        config_=EDM81::Config{}; config_.servo=servo_; config_.referenceV=referenceV_;
        config_.overridePercent=overridePercent_; config_.jumpMm=0.0005; config_.maxRetreatMm=0.0005;
        for(unsigned a=0;a<6U;++a)
        {
            config_.axes[a].minimum=a<3U?-1.0:-1000.0; config_.axes[a].maximum=a<3U?1.0:1000.0;
            config_.axes[a].maxTravel=1.0; config_.axes[a].maxVelocityPerMin=20.0;
        }
        sequence_=0; tick_=phase_=dwell_=0; anchor_=0.0; crossed_=false;
        if(!Check(BuildFixture())) return;
        if(case_==13U)
        {
            const NCPathCoreRetainedGeometry* g=nullptr; std::uint32_t ordinal=0;
            if(!Check(source_.BeginRetreat(source_.Count()) && source_.SelectStep(false,g,ordinal) && source_.MarkSubmitted())) return;
            if(Check(!path_.Capture(source_,scope_,scales_))) FinishCase();
            return;
        }
        if(case_==14U)
        {
            NativePose a=Origin(),b=a; a[0]=0.25; b[0]=0.251;
            if(Check(!AppendLine(a,b) && !path_.Capture(source_,scope_,scales_))) FinishCase();
            return;
        }
        if(case_==15U)
        {
            source_.Clear(); NativePose a=Origin(),b=a;
            for(unsigned i=0;i<16U;++i) { b[0]=double(i+1U)*0.001; if(!Check(AppendLine(a,b))) return; a=b; }
            b[0]=0.017;
            if(Check(!AppendLine(a,b) && source_.Fault()==NCPathCoreRetainedFault::CAPACITY && source_.Get(0U)==nullptr && !path_.Capture(source_,scope_,scales_) && path_.Count()==0U)) FinishCase();
            return;
        }
        if(case_==16U)
        { scope_.frameRevision=0; if(Check(!path_.Capture(source_,scope_,scales_))) FinishCase();
            return; }
        if(case_==26U)
        { source_.Clear(); NativePose a=Origin(),b=a; b[0]=0.001; b[3]+=0.01;
            if(Check(!AppendLine(a,b))) FinishCase();
            return; }
        if(case_==30U)
        { scales_[3]=0.0; if(Check(!path_.Capture(source_,scope_,scales_))) FinishCase();
            return; }
        if(!Check(path_.Capture(source_,scope_,scales_))) return;
        if(case_==27U)
        {
            EDM81::Pose pose{}; std::uint32_t ordinal=0; double u=0;
            if(Check(!path_.Evaluate(-0.001,pose,ordinal,u) &&
                !path_.Evaluate(path_.LengthMm()+0.001,pose,ordinal,u) &&
                !path_.Evaluate((std::numeric_limits<double>::quiet_NaN)(),pose,ordinal,u))) FinishCase();
            return;
        }
        if(case_==31U)
        {
            EDM81::Pose first{},last{}; std::uint32_t ordinal=0; double u=0;
            if(!Check(path_.Evaluate(0.0,first,ordinal,u) && ordinal==1U && u==0.0 &&
                path_.Evaluate(path_.LengthMm(),last,ordinal,u) && ordinal==source_.Count() && u==1.0)) return;
            for(unsigned a=0;a<6U;++a)
                if(!Check(first[a]==source_.Get(0U)->startMCS[a] && last[a]==source_.Get(source_.Count()-1U)->endMCS[a])) return;
            FinishCase();
            return;
        }
        if(case_==9U) config_.axes[1].maximum=0.0005;
        if(case_==10U) config_.axes[1].maxTravel=0.0005;
        if(case_==11U) config_.axes[1].maxVelocityPerMin=1e-9;
        if(case_==17U) ++scope_.sourceGeneration;
        if(case_==25U) config_.kinematics=EDM81::Kinematics::TCP;
        const bool admitted=kernel_.Begin(config_,scope_,path_);
        if(case_==9U || case_==10U || case_==17U || case_==25U)
        { if(Check(!admitted && kernel_.Current().state==EDM81::State::Fault && !kernel_.Current().committed)) FinishCase();
            return; }
        if(!Check(admitted)) return;
        if(case_==12U) source_.Clear(); // kernel and capture own copies, not pointers.
        caseStarted_=true;
    }
    EDM81::Input NextInput(double voltage,EDM81::Event event=EDM81::Event::None) noexcept
    {
        EDM81::Input input{}; input.scope=scope_; input.sequence=++sequence_;
        input.capturedAtUs=1000000ULL+sequence_*250ULL; input.nowUs=input.capturedAtUs;
        input.voltageV=voltage; input.event=event; return input;
    }
    bool GeometrySame(const Snapshot& s) const noexcept
    {
        EDM81::Pose expected{}; std::uint32_t ordinal=0; double u=0;
        if(!path_.Evaluate(s.cursorMm,expected,ordinal,u)) return false;
        if(s.ordinal!=ordinal || s.u!=u) return false;
        for(unsigned a=0;a<6U;++a) if(s.pose[a]!=expected[a]) return false;
        return true;
    }
    const Snapshot& Step(const EDM81::Input& input) noexcept
    {
        ++tick_; ++report_.virtualTicks;
        const auto& result=kernel_.Step(input);
        Check(!result.PhysicalPermit() && !result.DischargeEnabled());
        if(tick_>20000U) Fail("EDM82_CASE_TICK_BUDGET");
        return result;
    }
    void TickCase() noexcept
    {
        const Snapshot before=kernel_.Current();
        if(case_<=8U || case_==12U) { MotionCase(before); return; }
        EDM81::Input input=NextInput(referenceV_+10.0);
        if(tick_==0U)
        { const auto& r=Step(input); Check(r.state==EDM81::State::Running && r.cursorMm==0.0); return; }
        if(case_>=18U && case_<=20U)
        {
            input.event=case_==18U?EDM81::Event::Hold:(case_==19U?EDM81::Event::Reset:EDM81::Event::Alarm);
            const Snapshot r=Step(input);
            if(!Check(r.state==EDM81::State::Cancelled && !r.committed && r.pose==before.pose && r.cursorMm==before.cursorMm)) return;
            if(Check(Step(NextInput(referenceV_+10.0)).state==EDM81::State::Cancelled)) FinishCase();
            return;
        }
        if(case_==28U)
        {
            kernel_.Cancel(EDM81::Event::Reset);
            if(Check(!kernel_.Begin(config_,scope_,path_) && kernel_.Current().reason==EDM81::Reason::RetiredRun)) FinishCase();
            return;
        }
        EDM81::Reason expected=EDM81::Reason::None;
        switch(case_)
        {
        case 11: expected=EDM81::Reason::VelocityLimit; break;
        case 21: input.event=EDM81::Event::M30; expected=EDM81::Reason::PrematureEnd; break;
        case 22: input.limitMask=1U<<5U; expected=EDM81::Reason::AxisLimit; break;
        case 23: input.nowUs+=config_.maxAgeUs+1U; expected=EDM81::Reason::Stale; break;
        case 24: input.voltageV=(std::numeric_limits<double>::quiet_NaN)(); expected=EDM81::Reason::Sample; break;
        case 29: input.scope.sourceGeneration+=2U; expected=EDM81::Reason::Scope; break;
        default: Fail("EDM82_UNIMPLEMENTED_CASE"); return;
        }
        const Snapshot r=Step(input);
        if(!Check(r.state==EDM81::State::Fault && r.reason==expected && !r.committed && r.pose==before.pose && r.cursorMm==before.cursorMm)) return;
        const auto& terminal=Step(NextInput(referenceV_+10.0));
        if(Check(terminal.state==EDM81::State::Fault && !terminal.committed && terminal.pose==r.pose)) FinishCase();
    }
    void MotionCase(const Snapshot& before) noexcept
    {
        double voltage=referenceV_+10.0; EDM81::Event event=EDM81::Event::None;
        if(case_>=5U && case_<=8U)
        {
            if(phase_==0U && before.cursorMm>=0.0012)
            { anchor_=before.cursorMm; phase_=1U; if(case_==6U) event=EDM81::Event::Jump; }
            if(phase_==1U && case_==5U)
            { voltage=referenceV_-10.0; if(before.cursorMm<0.00085) { phase_=2U; voltage=referenceV_+10.0; } }
            if(phase_==1U && (case_==7U || case_==8U))
            {
                voltage=0.0;
                if(before.state==EDM81::State::ShortRetreat && before.cursorMm<=anchor_-config_.maxRetreatMm+1e-14)
                {
                    if(++dwell_>=64U)
                    {
                        if(case_==8U) { if(Check(crossed_ && before.state==EDM81::State::ShortRetreat)) FinishCase();
            return; }
                        phase_=2U; voltage=referenceV_+10.0;
                    }
                }
            }
        }
        if(before.cursorMm==before.pathLengthMm && tick_>0U) event=EDM81::Event::M30;
        const Snapshot result=Step(NextInput(voltage,event));
        if(report_.complete) return;
        if(!Check(result.state!=EDM81::State::Fault && result.state!=EDM81::State::Cancelled && GeometrySame(result))) return;
        if(tick_==1U && !Check(result.cursorMm==0.0)) return;
        if(case_>=5U && case_<=8U && phase_>0U && result.cursorMm<0.001) crossed_=true;
        if(case_>=6U && case_<=8U && phase_>0U)
        {
            if(!Check(result.cursorMm<=anchor_+1e-14)) return;
            if(result.state!=EDM81::State::Running && !Check(result.anchorMm==anchor_)) return;
            if(result.state==EDM81::State::Running && before.state==EDM81::State::Returning)
            { if(Check(crossed_ && result.cursorMm==anchor_ && GeometrySame(result))) FinishCase();
            return; }
        }
        if(result.state==EDM81::State::Completed)
        {
            if(case_==5U && !Check(phase_==2U && crossed_)) return;
            if(Check(result.cursorMm==result.pathLengthMm && !result.committed)) FinishCase();
        }
    }
    SimulationReport report_{};
    EDMGapServo::Profile servo_{};
    NCPathCoreRetainedPath source_{};
    RetainedPath path_{};
    Kernel kernel_{};
    EDM81::Config config_{};
    EDM81::Scope scope_{};
    std::array<double,8> scales_{};
    double referenceV_=75.0,overridePercent_=75.0,anchor_=0.0;
    std::uint64_t runSerial_=0,sequence_=0;
    unsigned case_=0,tick_=0,phase_=0,dwell_=0;
    bool started_=false,caseStarted_=false,crossed_=false;
};
static_assert(sizeof(SimulationSuite)<24576U,"EDM82 private suite must remain bounded");
}
