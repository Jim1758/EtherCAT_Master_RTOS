#pragma once
// PBC-3C: immutable X-only evidence from the servo-command -> send transaction.
// SPSC producer is the 250 us callback; HMI is the only consumer. No control
// authority, AxisContext mutation, dynamic storage, waits or RT formatting.
#include "MechanicalCompensationSendContract.h"
#include "CompensationEngine.h"
#include "MotionCommandRing.h"

struct MotionPbcXSource
{
    pbc::CoordinateFrame frame{};
    double rawFeedbackPulse = 0.0;
    std::uint64_t tick = 0ULL;
    bool valid = false;
    bool homed = false;
    CompensationEngine::XCycleTransaction cycle{};
    std::int32_t controlVelocity = 0;
    pbc::Error cycleError = pbc::Error::None;
    bool controlUsed = false;
};
struct MotionPbcXSendSample
{
    pbc::SendIdentity identity{};
    MotionPbcXSource source{};
    std::uint64_t sequence = 0ULL, ticket = 0ULL;
    std::uint32_t epoch = 0U, owner = 0U, ownerGeneration = 0U;
    std::int32_t proposedVelocity = 0, finalVelocity = 0;
    std::uint16_t sourceStatusWord = 0U;
    std::int8_t sourceMode = 0;
    pbc::SendResult result = pbc::SendResult::Bypassed;
    bool sourceReady = false, eligible = false, prepared = false, sealed = false;
    bool scrubbed = false, finalChecked = false, imagePresent = false;
    bool attempted = false, apiAccepted = false, stickyUncertain = false;
    bool modelSealed = false, modelFinished = false;
    pbc::SendResult modelResult = pbc::SendResult::Bypassed;
    pbc::Error modelError = pbc::Error::None;
};
struct MotionPbcXSendEvent
{
    MotionPbcXSendSample sample{};
    std::uint64_t total = 0ULL, prepared = 0ULL, sealed = 0ULL, shadowOK = 0ULL;
    std::uint64_t fence = 0ULL, discarded = 0ULL, uncertain = 0ULL, bypassed = 0ULL;
    std::uint64_t normal = 0ULL, stop = 0ULL, idle = 0ULL, zeroOnly = 0ULL, invalid = 0ULL;
    std::uint64_t apiFail = 0ULL, noSend = 0ULL, scrub = 0ULL, changed = 0ULL;
    std::uint64_t missingImage = 0ULL, sourceNotReady = 0ULL, prepareRejected = 0ULL;
    std::uint64_t tickGaps = 0ULL, identityChanges = 0ULL;
    std::uint64_t cyclePrepared = 0ULL, controlUsed = 0ULL, cycleSealed = 0ULL;
    std::uint64_t modelApplied = 0ULL, zeroApplied = 0ULL, modelDiscarded = 0ULL;
    std::uint64_t modelFence = 0ULL, modelUncertain = 0ULL, modelRejected = 0ULL;
    std::uint64_t unsentCancelled = 0ULL;
    std::uint64_t lastIssueSequence = 0ULL, lastIssueFrame = 0ULL, lastIssueTick = 0ULL;
    std::uint32_t lastIssue = 0U;
    std::uint32_t kind = 0U; // 1=periodic 500 ms; 2=issue, rate limited to 100 ms
};
static_assert(std::is_trivially_copyable<MotionPbcXSource>::value &&
    std::is_trivially_copyable<MotionPbcXSendSample>::value &&
    std::is_trivially_copyable<MotionPbcXSendEvent>::value,
    "X send diagnostics must be fixed POD values.");

class MotionPbcXSendAudit
{
public:
    static constexpr std::size_t Capacity = 32U, DrainBudget = 4U;
    // Counts are cumulative for this process. Rolling console snapshots are
    // overlapping observations; never add these totals across printed lines.
    void Observe(const MotionPbcXSendSample& s) noexcept
    {
        if (event_.total != 0ULL)
        {
            const auto& previous = event_.sample;
            if (previous.identity.tick != 0ULL && s.identity.tick != previous.identity.tick + 1ULL)
                Increment(event_.tickGaps);
            if (previous.identity.ownerState != s.identity.ownerState ||
                previous.identity.execution != s.identity.execution) Increment(event_.identityChanges);
        }
        event_.sample = s; Increment(event_.total);
        switch (s.identity.mode)
        {
        case 1U: Increment(event_.zeroOnly); break;
        case 2U: Increment(event_.normal); break;
        case 3U: Increment(event_.stop); break;
        case 4U: Increment(event_.idle); break;
        default: Increment(event_.invalid); break;
        }
        CountCycle(s);
        if (s.prepared) Increment(event_.prepared);
        if (s.sealed) Increment(event_.sealed);
        switch (s.result)
        {
        case pbc::SendResult::HandoffAccepted: Increment(event_.shadowOK); break;
        case pbc::SendResult::FenceAccepted: Increment(event_.fence); break;
        case pbc::SendResult::Discarded: Increment(event_.discarded); break;
        case pbc::SendResult::OutputUncertain: Increment(event_.uncertain); break;
        default: Increment(event_.bypassed); break;
        }
        if (s.attempted && !s.apiAccepted) Increment(event_.apiFail);
        if (!s.attempted) Increment(event_.noSend);
        if (s.scrubbed) Increment(event_.scrub);
        if (s.finalChecked && s.imagePresent && s.finalVelocity != s.proposedVelocity) Increment(event_.changed);
        if (s.finalChecked && !s.imagePresent) Increment(event_.missingImage);
        if (!s.sourceReady) Increment(event_.sourceNotReady);
        if (s.eligible && !s.prepared) Increment(event_.prepareRejected);
        // Issue bits keep causes distinct: API_FAIL, UNCERTAIN, PREPARE_REJECT,
        // MISSING_IMAGE, UNEXPECTED_IMAGE_CHANGE. Scrubs are expected fences.
        const std::uint32_t issue = (s.attempted && !s.apiAccepted ? 1U : 0U) |
            (s.result == pbc::SendResult::OutputUncertain ? 2U : 0U) |
            (s.eligible && !s.prepared ? 4U : 0U) |
            (s.finalChecked && !s.imagePresent ? 8U : 0U) |
            (s.finalChecked && s.imagePresent && !s.scrubbed && s.finalVelocity != s.proposedVelocity ? 16U : 0U) |
            (s.modelError != pbc::Error::None || s.source.cycleError != pbc::Error::None ? 32U : 0U);
        if (issue != 0U)
        {
            event_.lastIssue = issue;
            event_.lastIssueSequence = s.sequence;
            event_.lastIssueFrame = s.identity.frame;
            event_.lastIssueTick = s.identity.tick;
        }
        // Count-based scheduling still emits if an input/source tick stalls.
        const bool issueDue = issue != 0U && (!issueReported_ || event_.total - lastIssueEmit_ >= 400ULL);
        if (issueDue || event_.total == 1ULL || event_.total - lastEmit_ >= 2000ULL)
        {
            event_.kind = issueDue ? 2U : 1U;
            if (!events_.ProducerTryPush(event_))
            {
                std::uint32_t n = dropped_.load(std::memory_order_relaxed);
                if (n != (std::numeric_limits<std::uint32_t>::max)()) ++n;
                dropped_.store(n, std::memory_order_release);
            }
            lastEmit_ = event_.total;
            if (issueDue) { lastIssueEmit_ = event_.total; issueReported_ = true; }
        }
    }
    // A candidate abandoned before a transport entry has no NIC observation.
    // Account for it once; the next bounded snapshot carries these totals.
    void ObserveUnsentCycle(const MotionPbcXSource& source, bool cancelled) noexcept
    {
        MotionPbcXSendSample sample{}; sample.source = source;
        sample.modelResult = pbc::SendResult::Discarded; sample.modelFinished = cancelled;
        if (!cancelled) sample.modelError = pbc::Error::LifecycleRejected;
        CountCycle(sample); Increment(event_.unsentCancelled);
    }
    bool TryPop(MotionPbcXSendEvent& event) noexcept { return events_.ConsumerTryPop(event); }
    std::uint32_t Dropped() const noexcept { return dropped_.load(std::memory_order_acquire); }
private:
    MotionPbcXSendEvent event_{};
    std::uint64_t lastEmit_ = 0ULL, lastIssueEmit_ = 0ULL;
    bool issueReported_ = false;
    FixedCapacitySpscRing<MotionPbcXSendEvent, Capacity> events_{};
    std::atomic<std::uint32_t> dropped_{0U};
    void CountCycle(const MotionPbcXSendSample& s) noexcept
    {
        if (s.source.cycle.prepared) Increment(event_.cyclePrepared);
        if (s.source.controlUsed) Increment(event_.controlUsed);
        if (s.modelSealed) Increment(event_.cycleSealed);
        if (s.modelError != pbc::Error::None || s.source.cycleError != pbc::Error::None)
            Increment(event_.modelRejected);
        if (!s.source.cycle.prepared) return;
        switch (s.modelResult)
        {
        case pbc::SendResult::HandoffAccepted:
            if (s.modelFinished) { Increment(event_.modelApplied); if (s.finalVelocity == 0) Increment(event_.zeroApplied); }
            break;
        case pbc::SendResult::FenceAccepted: Increment(event_.modelFence); break;
        case pbc::SendResult::Discarded: Increment(event_.modelDiscarded); break;
        case pbc::SendResult::OutputUncertain: Increment(event_.modelUncertain); break;
        default: break;
        }
    }
    static void Increment(std::uint64_t& n) noexcept
    { if (n != (std::numeric_limits<std::uint64_t>::max)()) ++n; }
};
