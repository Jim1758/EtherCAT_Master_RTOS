#pragma once

#include "EDMPhysicalRecipeCapture.h"
#include <cmath>
#include <cstdint>

// EDM68 is an NC-owned diagnostic receipt, never an axis or discharge permit.
// The PDO owner continues its independent observer during HOLD/RESET. ReadRaw
// must precede the consumer clock passed to Observe, including after Ready.
namespace EDM68
{
enum class Status : std::uint8_t { Wait, Ready, Invalid };
constexpr std::uint64_t WindowUs=10000000ULL;
constexpr std::uint64_t TotalDeadlineUs=12000000ULL;
constexpr std::uint64_t MaximumConsumerGapUs=100000ULL;
constexpr std::uint64_t MinimumFreshObservations=100ULL;
constexpr std::uint32_t MaximumStalledCalls=4096U;
constexpr double TargetToleranceMmPerMin=0.00000001;

struct WindowReport
{
    std::uint64_t startObservedUs=0,lastObservedUs=0,durationUs=0,freshObservations=0;
    std::uint64_t sourcePdoDelta=0,firstSourceSequence=0,lastSourceSequence=0;
    std::uint64_t maxSourceAgeUs=0,maxConsumerGapUs=0,shortObservations=0,feedInhibitObservations=0;
    double minRawV=0.0,maxRawV=0.0,minTargetMmPerMin=0.0,maxTargetMmPerMin=0.0;
    double minSpeedMmPerMin=0.0,maxSpeedMmPerMin=0.0,maxTargetErrorMmPerMin=0.0;
};

class WindowReceipt
{
public:
    bool Begin(const EDM65::Request& request,std::uint64_t publicationFloor,
        std::uint64_t sourceSequenceFloor) noexcept
    {
        *this=WindowReceipt{};
        if(!ack_.Begin(request,publicationFloor,sourceSequenceFloor))
        { Fail(ack_.ReasonText()); return false; }
        active_=true; reason_="AWAIT_ADOPTION"; return true;
    }

    Status Observe(EDM65::Snapshot& raw,const EDM53::Stamp& stamp,
        std::uint64_t nowUs,bool clockValid,std::uint64_t frequency) noexcept
    {
        if(!acknowledged_)
        {
            if(!active_) { current_=raw; return Status::Invalid; }
            const auto result=ack_.Observe(raw,stamp,nowUs,clockValid,frequency);
            current_=raw;
            if(result==EDM67::Status::Invalid) return Fail(ack_.ReasonText());
            // Verify the first eligible evidence as well as the second receipt.
            if(stamp.Ready() && EDM65::SameRequest(raw.request,Request()) &&
                raw.readerState==EDM64::ReaderState::Fresh && raw.state==EDM65::State::Valid)
            {
                const char* invalid=CheckSnapshot(raw);
                if(invalid) return Fail(invalid);
            }
            if(result!=EDM67::Status::Ready) return Wait(ack_.ReasonText());
            acknowledged_=true; lastNowUs_=nowUs; last_=raw;
            if(!Accept(raw,nowUs)) return Fail("COUNTER_EXHAUSTED");
            return Wait("WINDOW_COLLECTING");
        }
        EDM65::PhysicalRecipeRTShadow::ValidateRead(raw,stamp,nowUs,clockValid,frequency);
        current_=raw;
        if(!active_) return Status::Invalid;
        if(!clockValid || !nowUs || !frequency || frequency!=Request().qpcFrequency ||
            nowUs<Request().capturedUs || nowUs<lastNowUs_) return Fail("CLOCK");
        if(nowUs-Request().capturedUs>=TotalDeadlineUs) return Fail("TOTAL_DEADLINE");
        if(nowUs==lastNowUs_)
        { if(++stalledCalls_>=MaximumStalledCalls) return Fail("CLOCK_STALLED"); }
        else stalledCalls_=0;
        lastNowUs_=nowUs;
        // Neither Busy nor a recovered fresh copy can conceal a consumer gap.
        if(nowUs-lastFreshConsumerUs_>MaximumConsumerGapUs) return Fail("CONSUMER_GAP");
        if(stamp.status==EDM53::Status::Closed) return Fail("CHANNEL_CLOSED");
        if(stamp.status==EDM53::Status::Busy) return Wait("CHANNEL_BUSY");
        if(stamp.status==EDM53::Status::Empty) return Wait("CHANNEL_EMPTY");
        if(!stamp.Ready()) return Fail("STAMP_INVALID");
        if(raw.readerState==EDM64::ReaderState::Clock) return Fail("READER_CLOCK");
        if(raw.readerState==EDM64::ReaderState::Stale) return Fail("READER_STALE");
        if(!EDM65::SameRequest(raw.request,Request())) return Fail("REQUEST_CHANGED");
        const char* invalid=CheckSnapshot(raw);
        if(invalid) return Fail(invalid);
        if(!EDM55::SameIdentity(raw.identity,last_.identity) ||
            raw.sourceGeneration!=last_.sourceGeneration || raw.observerGeneration!=last_.observerGeneration ||
            raw.recipePublication!=last_.recipePublication || raw.maxAgeUs!=last_.maxAgeUs ||
            raw.maxTickGapUs!=last_.maxTickGapUs) return Fail("SOURCE_IDENTITY_CHANGED");
        if(raw.publication<last_.publication || raw.cycle<last_.cycle ||
            raw.sourceSequence<last_.sourceSequence || raw.observedUs<last_.observedUs ||
            raw.sourceCapturedUs<last_.sourceCapturedUs) return Fail("SEQUENCE_REGRESSION");
        if(!CoverageContinues(raw.coverage,last_.coverage)) return Fail("COVERAGE_REGRESSION");
        if(raw.publication==last_.publication)
        {
            if(!SameObservation(raw,last_)) return Fail("PUBLICATION_CHANGED");
            return Wait("AWAIT_FRESH_SOURCE");
        }
        if(raw.cycle<=last_.cycle || raw.observedUs<=last_.observedUs ||
            raw.coverage.validTicks-last_.coverage.validTicks!=raw.cycle-last_.cycle)
            return Fail("TICK_PROGRESS");
        if(raw.sourceSequence==last_.sourceSequence)
        {
            if(raw.sourceCapturedUs!=last_.sourceCapturedUs ||
                raw.output.rawVoltageV!=last_.output.rawVoltageV ||
                raw.output.filteredVoltageV!=last_.output.filteredVoltageV || raw.output.fresh ||
                raw.coverage.freshSamples!=last_.coverage.freshSamples)
                return Fail("SOURCE_CHANGED");
        }
        else if(raw.sourceCapturedUs<=last_.sourceCapturedUs ||
            raw.coverage.freshSamples<=last_.coverage.freshSamples ||
            raw.coverage.freshSamples-last_.coverage.freshSamples>raw.sourceSequence-last_.sourceSequence)
            return Fail("SOURCE_PROGRESS");
        last_=raw;
        if(!raw.output.fresh || raw.sourceSequence<=report_.lastSourceSequence ||
            raw.observedUs<=report_.lastObservedUs) return Wait("AWAIT_FRESH_SOURCE");
        if(!Accept(raw,nowUs)) return Fail("COUNTER_EXHAUSTED");
        if(report_.durationUs<WindowUs || report_.freshObservations<MinimumFreshObservations)
            return Wait("WINDOW_COLLECTING");
        reason_="CONTINUOUS_PHYSICAL_RECIPE_WINDOW"; return Status::Ready;
    }

    void Cancel() noexcept { active_=false; ack_.Cancel(); reason_="CANCELLED"; }
    bool Active() const noexcept { return active_; }
    bool Acknowledged() const noexcept { return acknowledged_; }
    const char* ReasonText() const noexcept { return reason_; }
    const EDM65::Request& Request() const noexcept { return ack_.Request(); }
    const EDM65::Snapshot& Current() const noexcept { return current_; }
    const WindowReport& Report() const noexcept { return report_; }

private:
    static bool CoverageValid(const EDM65::Snapshot& s) noexcept
    {
        const auto& c=s.coverage;
        if(!c.hasSamples || c.saturated || !c.validTicks || !c.freshSamples ||
            c.freshSamples>c.validTicks || c.repeatSamples!=c.validTicks-c.freshSamples ||
            c.advanceTicks>c.validTicks || c.retreatTicks>c.validTicks-c.advanceTicks ||
            c.zeroTicks!=c.validTicks-c.advanceTicks-c.retreatTicks ||
            c.shortTicks>c.feedInhibitTicks || c.feedInhibitTicks>c.validTicks ||
            c.shortEntries>c.validTicks || c.shortExits>c.validTicks ||
            !c.firstObservedUs || c.firstObservedUs>c.lastObservedUs || c.lastObservedUs!=s.observedUs ||
            !c.firstSourceSequence || c.firstSourceSequence>c.lastSourceSequence ||
            c.lastSourceSequence!=s.sourceSequence || c.lastShort!=s.output.shortActive ||
            c.freshSamples-1ULL>c.lastSourceSequence-c.firstSourceSequence ||
            (s.output.shortActive && !c.shortTicks) || (s.output.feedInhibited && !c.feedInhibitTicks) ||
            (s.output.targetMmPerMin>0.0 && !c.advanceTicks) ||
            (s.output.targetMmPerMin<0.0 && !c.retreatTicks) ||
            (s.output.targetMmPerMin==0.0 && !c.zeroTicks) ||
            !std::isfinite(c.minRawV) || !std::isfinite(c.maxRawV) || c.minRawV<0.0 || c.maxRawV>300.0 ||
            c.minRawV>c.maxRawV || s.output.rawVoltageV<c.minRawV || s.output.rawVoltageV>c.maxRawV)
            return false;
        if(c.initialShort==c.lastShort) return c.shortEntries==c.shortExits;
        if(c.lastShort) return c.shortEntries>c.shortExits && c.shortEntries-c.shortExits==1ULL;
        return c.shortExits>c.shortEntries && c.shortExits-c.shortEntries==1ULL;
    }
    static bool CoverageContinues(const EDM65::Coverage& a,const EDM65::Coverage& b) noexcept
    {
        return a.firstObservedUs==b.firstObservedUs && a.firstSourceSequence==b.firstSourceSequence &&
            a.initialShort==b.initialShort && a.validTicks>=b.validTicks && a.freshSamples>=b.freshSamples &&
            a.repeatSamples>=b.repeatSamples && a.advanceTicks>=b.advanceTicks &&
            a.retreatTicks>=b.retreatTicks && a.zeroTicks>=b.zeroTicks && a.shortTicks>=b.shortTicks &&
            a.shortEntries>=b.shortEntries && a.shortExits>=b.shortExits &&
            a.feedInhibitTicks>=b.feedInhibitTicks && a.minRawV<=b.minRawV && a.maxRawV>=b.maxRawV;
    }
    static bool SameObservation(const EDM65::Snapshot& a,const EDM65::Snapshot& b) noexcept
    {
        const auto& x=a.output; const auto& y=b.output;
        const auto& c=a.coverage; const auto& d=b.coverage;
        return a.cycle==b.cycle && a.observedUs==b.observedUs && a.sourceSequence==b.sourceSequence &&
            a.sourceCapturedUs==b.sourceCapturedUs && x.fresh==y.fresh && x.rawVoltageV==y.rawVoltageV &&
            x.filteredVoltageV==y.filteredVoltageV && x.targetMmPerMin==y.targetMmPerMin &&
            x.speedMmPerMin==y.speedMmPerMin && x.shortActive==y.shortActive && x.feedInhibited==y.feedInhibited &&
            c.validTicks==d.validTicks && c.freshSamples==d.freshSamples && c.repeatSamples==d.repeatSamples &&
            c.advanceTicks==d.advanceTicks && c.retreatTicks==d.retreatTicks && c.zeroTicks==d.zeroTicks &&
            c.shortTicks==d.shortTicks && c.shortEntries==d.shortEntries && c.shortExits==d.shortExits &&
            c.feedInhibitTicks==d.feedInhibitTicks && c.minRawV==d.minRawV && c.maxRawV==d.maxRawV;
    }
    const char* CheckSnapshot(const EDM65::Snapshot& s) const noexcept
    {
        if(s.physicalPermit || s.discharge) return "OUTPUT_PERMISSION";
        if(s.state!=EDM65::State::Valid || s.reason!=EDM65::Reason::None || !s.recipeAdopted ||
            !s.output.valid || !s.output.running || s.output.stopRequired || s.output.fault!=EDM55::Fault::None)
            return "DERIVED_INVALID";
        if(s.readerState!=EDM64::ReaderState::Fresh || !s.recipePublication ||
            s.qpcFrequency!=Request().qpcFrequency || !EDM55::ValidIdentity(s.identity) ||
            s.identity.source!=EDM55::SourceKind::Physical || !s.observerGeneration ||
            s.identity.session!=s.observerGeneration || !s.sourceGeneration) return "PHYSICAL_IDENTITY";
        if(!s.sourceAgeKnown || !s.maxAgeUs || s.maxAgeUs>5000ULL || s.sourceAgeUs>s.maxAgeUs ||
            !s.maxTickGapUs || s.maxTickGapUs>2000ULL || s.maxTickGapUs>s.maxAgeUs ||
            !s.sourceSequence || !s.sourceCapturedUs || s.sourceCapturedUs>s.observedUs ||
            !s.cycle || s.output.tick!=s.cycle || s.output.nowUs!=s.observedUs ||
            s.output.sequence!=s.sourceSequence || s.output.capturedAtUs!=s.sourceCapturedUs ||
            !std::isfinite(s.output.rawVoltageV) || !std::isfinite(s.output.filteredVoltageV) ||
            s.output.rawVoltageV<0.0 || s.output.rawVoltageV>300.0 ||
            s.output.filteredVoltageV<0.0 || s.output.filteredVoltageV>300.0 ||
            !std::isfinite(s.output.targetMmPerMin) || !std::isfinite(s.output.speedMmPerMin))
            return "SNAPSHOT_INVALID";
        if(!CoverageValid(s) || s.coverage.firstObservedUs<=Request().capturedUs) return "COVERAGE_INVALID";
        const double expected=ExpectedTarget(s);
        if(!std::isfinite(expected) || std::fabs(expected-s.output.targetMmPerMin)>TargetToleranceMmPerMin)
            return "TARGET_MISMATCH";
        const auto& curve=Request().recipe.curve;
        if(s.output.speedMmPerMin>curve.maxFeedMmPerMin+TargetToleranceMmPerMin ||
            s.output.speedMmPerMin< -curve.maxRetreatMmPerMin-TargetToleranceMmPerMin ||
            (s.output.shortActive && !s.output.feedInhibited) ||
            (s.output.feedInhibited && s.output.speedMmPerMin>0.0)) return "ADVISORY_INVALID";
        return nullptr;
    }
    double ExpectedTarget(const EDM65::Snapshot& s) const noexcept
    {
        const auto& recipe=Request().recipe;
        const auto result=EDMGapServo::Evaluate(recipe.curve,s.output.filteredVoltageV,
            recipe.referenceV,recipe.overridePercent);
        if(!result.valid) return (std::numeric_limits<double>::quiet_NaN)();
        if(s.output.shortActive) return -recipe.curve.shortRetreatMmPerMin*(recipe.overridePercent/100.0);
        if(s.output.feedInhibited && result.limitedSpeedMmPerMin>0.0) return 0.0;
        return result.limitedSpeedMmPerMin;
    }
    bool Accept(const EDM65::Snapshot& s,std::uint64_t nowUs) noexcept
    {
        auto& r=report_;
        if(!r.freshObservations)
        {
            r.startObservedUs=s.observedUs; r.firstSourceSequence=s.sourceSequence;
            r.minRawV=r.maxRawV=s.output.rawVoltageV;
            r.minTargetMmPerMin=r.maxTargetMmPerMin=s.output.targetMmPerMin;
            r.minSpeedMmPerMin=r.maxSpeedMmPerMin=s.output.speedMmPerMin;
        }
        else
        {
            const auto gap=nowUs-lastFreshConsumerUs_;
            if(gap>r.maxConsumerGapUs) r.maxConsumerGapUs=gap;
            if(s.output.rawVoltageV<r.minRawV) r.minRawV=s.output.rawVoltageV;
            if(s.output.rawVoltageV>r.maxRawV) r.maxRawV=s.output.rawVoltageV;
            if(s.output.targetMmPerMin<r.minTargetMmPerMin) r.minTargetMmPerMin=s.output.targetMmPerMin;
            if(s.output.targetMmPerMin>r.maxTargetMmPerMin) r.maxTargetMmPerMin=s.output.targetMmPerMin;
            if(s.output.speedMmPerMin<r.minSpeedMmPerMin) r.minSpeedMmPerMin=s.output.speedMmPerMin;
            if(s.output.speedMmPerMin>r.maxSpeedMmPerMin) r.maxSpeedMmPerMin=s.output.speedMmPerMin;
        }
        if(!EDM65::AdvanceCounter(r.freshObservations) ||
            (s.output.shortActive && !EDM65::AdvanceCounter(r.shortObservations)) ||
            (s.output.feedInhibited && !EDM65::AdvanceCounter(r.feedInhibitObservations))) return false;
        r.lastObservedUs=s.observedUs; r.durationUs=s.observedUs-r.startObservedUs;
        r.lastSourceSequence=s.sourceSequence; r.sourcePdoDelta=s.sourceSequence-r.firstSourceSequence;
        if(s.sourceAgeUs>r.maxSourceAgeUs) r.maxSourceAgeUs=s.sourceAgeUs;
        const double error=std::fabs(ExpectedTarget(s)-s.output.targetMmPerMin);
        if(error>r.maxTargetErrorMmPerMin) r.maxTargetErrorMmPerMin=error;
        lastFreshConsumerUs_=nowUs; return true;
    }
    Status Fail(const char* reason) noexcept { active_=false; reason_=reason; return Status::Invalid; }
    Status Wait(const char* reason) noexcept { reason_=reason; return Status::Wait; }
    EDM67::CaptureReceipt ack_{};
    EDM65::Snapshot current_{},last_{};
    WindowReport report_{};
    std::uint64_t lastNowUs_=0,lastFreshConsumerUs_=0;
    std::uint32_t stalledCalls_=0;
    const char* reason_="INACTIVE";
    bool active_=false,acknowledged_=false;
};
static_assert(sizeof(WindowReceipt)<=3072U,"EDM68 NC receipt must remain bounded.");
}
