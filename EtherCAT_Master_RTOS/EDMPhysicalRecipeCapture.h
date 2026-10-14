#pragma once

#include "EDMGapPhysicalRecipeRTShadow.h"
#include <cstdint>

// EDM67 NC-owned receipt only. It never publishes a recipe, resets PDO state,
// commands an axis, or grants physical/discharge permission. ReadRaw must run
// BEFORE sampling the NC clock passed to Observe, even after an earlier Ready.
namespace EDM67
{
enum class Status : std::uint8_t { Wait, Ready, Invalid };
constexpr std::uint64_t ReceiptDeadlineUs=500000ULL;
constexpr std::uint32_t MaximumStalledCalls=4096U;

class CaptureReceipt
{
public:
    bool Begin(const EDM65::Request& request,std::uint64_t publicationFloor,
        std::uint64_t sourceSequenceFloor) noexcept
    {
        *this=CaptureReceipt{};
        request_=request;
        publicationFloor_=lastPublication_=publicationFloor;
        // EDM64 and EDM65 are separate publications. The last EDM65 copy may
        // legitimately trail the independently sampled EDM64 source floor.
        // Use the floor for eligibility, never as an already seen EDM65 value.
        sequenceFloor_=sourceSequenceFloor;
        if(!EDM65::ValidScope(request.scope) || !EDM59::ValidateRecipe(request.recipe) ||
            !request.capturedUs || !request.qpcFrequency)
        { Fail("REQUEST_INVALID"); return false; }
        active_=true; reason_="AWAIT_ADOPTION"; return true;
    }

    Status Observe(EDM65::Snapshot& raw,const EDM53::Stamp& stamp,
        std::uint64_t nowUs,bool clockValid,std::uint64_t frequency) noexcept
    {
        EDM65::PhysicalRecipeRTShadow::ValidateRead(raw,stamp,nowUs,clockValid,frequency);
        current_=raw;
        if(!active_) return Status::Invalid;
        if(!clockValid || !nowUs || !frequency || frequency!=request_.qpcFrequency ||
            nowUs<request_.capturedUs || (lastNowUs_ && nowUs<lastNowUs_))
            return Fail("CLOCK");
        if(nowUs-request_.capturedUs>=ReceiptDeadlineUs) return Fail("DEADLINE");
        if(nowUs==lastNowUs_)
        {
            if(++stalledCalls_>=MaximumStalledCalls) return Fail("CLOCK_STALLED");
        }
        else stalledCalls_=0;
        lastNowUs_=nowUs;
        if(stamp.status==EDM53::Status::Closed) return Fail("CHANNEL_CLOSED");
        if(stamp.status==EDM53::Status::Busy) return Wait("CHANNEL_BUSY");
        if(stamp.status==EDM53::Status::Empty) return Wait("CHANNEL_EMPTY");
        if(!stamp.Ready()) return Fail("STAMP_INVALID");
        if(raw.readerState==EDM64::ReaderState::Clock) return Fail("READER_CLOCK");
        if(raw.readerState==EDM64::ReaderState::Stale) return Fail("READER_STALE");
        if(raw.state==EDM65::State::Revoked) return Fail("OBSERVER_REVOKED");
        if(raw.physicalPermit || raw.discharge) return Fail("OUTPUT_PERMISSION");
        if(raw.output.fault!=EDM55::Fault::None) return Fail("KERNEL_FAULT");
        if(raw.publication<lastPublication_ || (raw.sourceSequence && raw.sourceSequence<lastSequence_) ||
            (lastObservedUs_ && raw.observedUs<lastObservedUs_) ||
            (lastCapturedUs_ && raw.sourceCapturedUs<lastCapturedUs_))
            return Fail("SEQUENCE_REGRESSION");
        if(raw.sourceSequence && raw.sourceSequence==lastSequence_ && lastCapturedUs_ &&
            raw.sourceCapturedUs!=lastCapturedUs_) return Fail("SOURCE_CLOCK_CHANGED");
        lastPublication_=raw.publication;
        if(raw.sourceSequence) lastSequence_=raw.sourceSequence;
        if(raw.observedUs) lastObservedUs_=raw.observedUs;
        if(raw.sourceCapturedUs) lastCapturedUs_=raw.sourceCapturedUs;
        if(raw.request.scope.session>request_.scope.session) return Fail("REQUEST_SUPERSEDED");
        if(raw.request.scope.session==request_.scope.session && !EDM65::SameRequest(raw.request,request_))
            return Fail("REQUEST_MISMATCH");
        if(raw.request.scope.session<request_.scope.session) return Wait("OLD_SCOPE");
        if(raw.readerState!=EDM64::ReaderState::Fresh) return Wait("AWAIT_OBSERVATION");
        if(!raw.recipePublication) return Fail("RECIPE_PUBLICATION");
        if(raw.state!=EDM65::State::Valid) return Wait("AWAIT_VALID");
        if(raw.reason!=EDM65::Reason::None || !raw.recipeAdopted || !raw.output.valid ||
            !raw.output.running || raw.output.stopRequired)
            return Fail("DERIVED_INVALID");
        if(!EDM55::ValidIdentity(raw.identity) || raw.identity.source!=EDM55::SourceKind::Physical ||
            !raw.observerGeneration || raw.identity.session!=raw.observerGeneration || !raw.sourceGeneration)
            return Fail("PHYSICAL_IDENTITY");
        if(!raw.sourceAgeKnown || !raw.maxAgeUs || raw.maxAgeUs>5000ULL ||
            !raw.maxTickGapUs || raw.maxTickGapUs>2000ULL || raw.maxTickGapUs>raw.maxAgeUs ||
            !raw.sourceCapturedUs || raw.sourceCapturedUs>raw.observedUs ||
            !raw.cycle || raw.output.tick!=raw.cycle || raw.output.nowUs!=raw.observedUs ||
            raw.output.sequence!=raw.sourceSequence || raw.output.capturedAtUs!=raw.sourceCapturedUs ||
            !std::isfinite(raw.output.rawVoltageV) || !std::isfinite(raw.output.filteredVoltageV) ||
            !std::isfinite(raw.output.targetMmPerMin) || !std::isfinite(raw.output.speedMmPerMin))
            return Fail("SNAPSHOT_INVALID");
        if(!raw.coverage.hasSamples || !raw.coverage.validTicks || !raw.coverage.freshSamples ||
            raw.coverage.freshSamples>raw.coverage.validTicks ||
            !raw.coverage.firstObservedUs || raw.coverage.firstObservedUs>raw.coverage.lastObservedUs ||
            !raw.coverage.firstSourceSequence || raw.coverage.firstSourceSequence>raw.coverage.lastSourceSequence ||
            raw.coverage.saturated || raw.coverage.lastObservedUs!=raw.observedUs ||
            raw.coverage.lastSourceSequence!=raw.sourceSequence)
            return Fail("COVERAGE_INVALID");
        if(raw.publication<=publicationFloor_ || raw.sourceSequence<=sequenceFloor_ ||
            raw.sourceCapturedUs<=request_.capturedUs) return Wait("AWAIT_NEW_SOURCE");
        if(haveEvidence_)
        {
            if(!EDM55::SameIdentity(identity_,raw.identity) || sourceGeneration_!=raw.sourceGeneration ||
                recipePublication_!=raw.recipePublication || maxAgeUs_!=raw.maxAgeUs ||
                maxTickGapUs_!=raw.maxTickGapUs) return Fail("SOURCE_IDENTITY_CHANGED");
            if(raw.coverage.freshSamples<evidenceFresh_ || raw.cycle<evidenceCycle_)
                return Fail("EVIDENCE_REGRESSION");
            if(!raw.output.fresh || raw.publication<=evidencePublication_ ||
                raw.sourceSequence<=evidenceSequence_ || raw.sourceCapturedUs<=evidenceCapturedUs_ ||
                raw.observedUs<=evidenceObservedUs_ || raw.coverage.freshSamples<=evidenceFresh_)
                return Wait("AWAIT_SECOND_FRESH");
            RememberEvidence(raw);
            reason_="FRESH_PHYSICAL_RECIPE_ACK"; return Status::Ready;
        }
        if(!raw.output.fresh) return Wait("AWAIT_FRESH_SOURCE");
        identity_=raw.identity; sourceGeneration_=raw.sourceGeneration;
        recipePublication_=raw.recipePublication; maxAgeUs_=raw.maxAgeUs; maxTickGapUs_=raw.maxTickGapUs;
        haveEvidence_=true; RememberEvidence(raw); return Wait("AWAIT_SECOND_FRESH");
    }

    const char* ReasonText() const noexcept { return reason_; }
    void Cancel() noexcept { active_=false; reason_="CANCELLED"; }
    const EDM65::Request& Request() const noexcept { return request_; }
    bool Active() const noexcept { return active_; }
    const EDM65::Snapshot& Current() const noexcept { return current_; }

private:
    Status Fail(const char* reason) noexcept { active_=false; reason_=reason; return Status::Invalid; }
    Status Wait(const char* reason) noexcept { reason_=reason; return Status::Wait; }
    void RememberEvidence(const EDM65::Snapshot& value) noexcept
    {
        evidencePublication_=value.publication; evidenceSequence_=value.sourceSequence;
        evidenceCapturedUs_=value.sourceCapturedUs; evidenceObservedUs_=value.observedUs;
        evidenceFresh_=value.coverage.freshSamples; evidenceCycle_=value.cycle;
    }
    EDM65::Request request_{};
    EDM65::Snapshot current_{};
    EDM55::Identity identity_{};
    std::uint64_t publicationFloor_=0,sequenceFloor_=0,lastNowUs_=0;
    std::uint64_t lastPublication_=0,lastSequence_=0,lastObservedUs_=0,lastCapturedUs_=0;
    std::uint64_t sourceGeneration_=0,recipePublication_=0,maxAgeUs_=0,maxTickGapUs_=0;
    std::uint64_t evidencePublication_=0,evidenceSequence_=0,evidenceCapturedUs_=0;
    std::uint64_t evidenceObservedUs_=0,evidenceFresh_=0,evidenceCycle_=0;
    const char* reason_="INACTIVE";
    std::uint32_t stalledCalls_=0;
    bool active_=false,haveEvidence_=false;
};
static_assert(sizeof(CaptureReceipt)<=1536U,"EDM67 receipt must remain bounded.");
}
