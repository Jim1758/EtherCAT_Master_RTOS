#pragma once

#include "EDMSixAxisModeFlush.h"

// EDM118 retains diagnostic values only. No value in this header grants
// output, recovery, RESET or START authority. A stopped executor is not a
// physical-stop proof; NC must also validate the existing EDM94 zero window.
namespace EDM118
{
using Point = EDM114::Point;
using Source = EDM109::Source;
constexpr unsigned ArmP = 134U, FinishP = 135U;
constexpr unsigned TargetStep = 2U, TargetSpan = 3U;
constexpr EDM116::Fixture TargetFixture = EDM116::Fixture::Complement117;
constexpr EDM116::Mode TargetMode = EDM116::Mode::B2;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

struct Receipt
{
    EDM116::Policy policy{};
    Source trigger{}, previous{};
    EDM115::Feedback initial{};
    Point commandPulse{}, commandVelocity{};
    std::uint64_t armEpoch = 0ULL, mappedTick = 0ULL, captureTick = 0ULL, captureUs = 0ULL;
    std::uint64_t safetyEpoch = 0ULL, safetyOwnerGeneration = 0ULL, safetyTicket = 0ULL;
    unsigned authorizedRows = 0U, completedRows = 0U, eventsStarted = 0U, eventsCompleted = 0U;
    bool captured = false;
};

struct Feedback
{
    Receipt first{};
    EDM115::Feedback final{};
    Point stoppedPulse{}, stoppedVelocity{};
    std::uint64_t session = 0ULL, armEpoch = 0ULL, ownerGeneration = 0ULL;
    std::uint64_t lastTick = 0ULL, stoppedTick = 0ULL;
    std::uint64_t safetyEpoch = 0ULL, safetyOwnerGeneration = 0ULL, safetyTicket = 0ULL;
    bool stopAccepted = false;
    bool stopped = false; // Successfully mapped executor stop, not encoder/CSV proof.
    bool publishedFailure = false;
};

static_assert(std::is_trivially_copyable<Receipt>::value && sizeof(Receipt) <= 768U,
    "EDM118 first committed stop observation remains a bounded value");
static_assert(std::is_trivially_copyable<Feedback>::value && sizeof(Feedback) <= 1024U,
    "EDM118 independent diagnostic publication remains bounded");

inline bool FinitePoint(const Point& point) noexcept
{
    for (double value : point) if (!std::isfinite(value)) return false;
    return true;
}

inline bool ValidInitial(const EDM115::Feedback& initial) noexcept
{
    return initial.geometryValid && initial.active && !initial.faulted && !initial.complete &&
        !initial.stopRequested && !initial.stopped && initial.reason == EDM115::Reason::None &&
        (initial.phase == EDM115::Phase::InitialSettle || initial.phase == EDM115::Phase::Moving ||
            initial.phase == EDM115::Phase::EndpointSettle) &&
        initial.tick && initial.nowUs && initial.startTick && initial.startUs &&
        initial.startTick <= initial.tick && initial.startUs <= initial.nowUs &&
        initial.started == 1ULL && initial.completed == 0ULL && initial.anchorReturned == 0ULL &&
        std::isfinite(initial.q) && initial.q >= 0.0 && std::isfinite(initial.v) &&
        std::isfinite(initial.a) && std::isfinite(initial.progress) && initial.progress >= 0.0 &&
        EDM114::Positive(initial.length) && initial.progress <= initial.length &&
        EDM114::Positive(initial.deceleration) &&
        (initial.rowKind == EDM114::Kind::Line || initial.rowKind == EDM114::Kind::Arc);
}

inline bool TargetSample(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture &&
        policy.mode == TargetMode && source.step == TargetStep &&
        ValidInitial(initial) && initial.span == TargetSpan && initial.phase == EDM115::Phase::Moving &&
        initial.moving && !initial.endpointReady && initial.v > 0.0 && initial.direction == 1 &&
        initial.rowKind == EDM114::Kind::Arc && initial.backwardCrossings == 1ULL &&
        initial.forwardCrossings == 1ULL && initial.completedDwells == 4U;
}

inline bool RequestWindow(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return TargetSample(policy, source, initial) &&
        initial.progress >= initial.length * RequestProgressBegin &&
        initial.progress <= initial.length * RequestProgressEnd;
}

inline bool ValidReceipt(const Receipt& receipt) noexcept
{
    const auto& source = receipt.trigger;
    const auto maximum = (std::numeric_limits<std::uint32_t>::max)();
    if (!receipt.captured || !EDM116::PolicyMatchesSource(receipt.policy, source) ||
        !ValidInitial(receipt.initial) || !receipt.armEpoch || receipt.armEpoch > maximum ||
        !receipt.mappedTick || receipt.initial.tick != receipt.mappedTick ||
        receipt.captureTick < receipt.mappedTick || receipt.captureUs < receipt.initial.nowUs ||
        !receipt.safetyEpoch || receipt.safetyEpoch > maximum || !receipt.safetyOwnerGeneration ||
        receipt.safetyOwnerGeneration > maximum || !receipt.safetyTicket || receipt.safetyTicket > maximum ||
        !FinitePoint(receipt.commandPulse) || !FinitePoint(receipt.commandVelocity) ||
        receipt.authorizedRows != source.step + 1U || receipt.completedRows != source.step ||
        receipt.eventsStarted != EDM116::EventOrdinal(source.step) ||
        receipt.eventsStarted != receipt.eventsCompleted + 1U) return false;
    if (receipt.policy.mode == EDM116::Mode::B2)
        return EDM114::ValidSource(receipt.previous) && source.step != 0U &&
            receipt.previous.step + 1U == source.step &&
            EDM109::SameScope(receipt.previous.scope, source.scope) &&
            receipt.previous.dispatch < source.dispatch && receipt.previous.epoch < source.epoch &&
            receipt.previous.segment < source.segment && receipt.previous.sourcePC < source.sourcePC;
    return EDM114::SameSource(receipt.previous, Source{});
}

inline bool TargetReceipt(const Receipt& receipt) noexcept
{
    // The consumer may have progressed beyond the NC request window; the
    // actual first committed sample must still be moving in the same span.
    return ValidReceipt(receipt) && TargetSample(receipt.policy, receipt.trigger, receipt.initial);
}

inline bool RetainFirst(Receipt& retained, const Receipt& candidate) noexcept
{
    if (retained.captured) return false;
    Receipt first = candidate;
    first.captured = true;
    if (!ValidReceipt(first)) return false;
    retained = first;
    return true;
}
}

// EDM119 selects the other B2 interruption direction while sharing the
// unchanged committed RT receipt. The trigger remains the G02 source;
// span 1 maps the retained previous G01 backwards after the first crossing.
namespace EDM119
{
using Point = EDM118::Point;
using Source = EDM118::Source;
using Receipt = EDM118::Receipt;
using Feedback = EDM118::Feedback;
constexpr unsigned ArmP = 136U, FinishP = 137U;
constexpr unsigned TargetStep = 2U, TargetSpan = 1U, TargetDwells = 2U;
constexpr int TargetDirection = -1;
constexpr std::uint64_t TargetBackwardCrossings = 1ULL, TargetForwardCrossings = 0ULL;
constexpr EDM114::Kind TargetRowKind = EDM114::Kind::Line;
constexpr EDM116::Fixture TargetFixture = EDM116::Fixture::Canonical116;
constexpr EDM116::Mode TargetMode = EDM116::Mode::B2;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

inline bool TargetSample(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture &&
        policy.mode == TargetMode && source.step == TargetStep &&
        EDM118::ValidInitial(initial) && initial.span == TargetSpan && initial.phase == EDM115::Phase::Moving &&
        initial.moving && !initial.endpointReady && initial.v < 0.0 && initial.direction == TargetDirection &&
        initial.rowKind == TargetRowKind && initial.backwardCrossings == TargetBackwardCrossings &&
        initial.forwardCrossings == TargetForwardCrossings && initial.completedDwells == TargetDwells;
}

inline bool RequestWindow(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return TargetSample(policy, source, initial) &&
        initial.progress >= initial.length * RequestProgressBegin &&
        initial.progress <= initial.length * RequestProgressEnd;
}

inline bool TargetReceipt(const Receipt& receipt) noexcept
{
    // The committed RT capture may follow the NC request window, but must
    // still be moving backwards on the exact same retained-line span.
    return EDM118::ValidReceipt(receipt) && TargetSample(receipt.policy, receipt.trigger, receipt.initial);
}

inline bool ProgressContinues(const EDM115::Feedback& before, const EDM115::Feedback& after) noexcept
{
    // Positive local progress increases while the retained row's scalar q
    // decreases. Their sum is the frozen start of span 1. Addition may round,
    // so compare that derived sum with a small relative arithmetic tolerance;
    // source identity, timestamps and frozen dynamics remain caller checks.
    if (before.span != TargetSpan || after.span != TargetSpan ||
        before.direction != TargetDirection || after.direction != TargetDirection ||
        before.rowKind != TargetRowKind || after.rowKind != TargetRowKind ||
        !std::isfinite(before.q) || !std::isfinite(after.q) || before.q < 0.0 || after.q < 0.0 ||
        !std::isfinite(before.progress) || !std::isfinite(after.progress) ||
        before.progress < 0.0 || after.progress < before.progress ||
        !EDM114::Positive(before.length) || !EDM109::SameDouble(before.length, after.length) ||
        before.progress > before.length || after.progress > after.length || after.q > before.q) return false;
    const double beforeOrigin = before.q + before.progress;
    const double afterOrigin = after.q + after.progress;
    if (!EDM114::Positive(beforeOrigin) || !EDM114::Positive(afterOrigin)) return false;
    const double scale = (std::max)(before.length, (std::max)(beforeOrigin, afterOrigin));
    return std::fabs(afterOrigin - beforeOrigin) <=
        64.0 * std::numeric_limits<double>::epsilon() * scale;
}
}

// EDM120 observes B2 while retracing the trigger G02 itself, before the
// first row crossing. Span 0 starts at anchorQ and ends at zero, so its
// frozen length equals anchorQ. Evidence remains independent of authority.
namespace EDM120
{
using Point = EDM118::Point;
using Source = EDM118::Source;
using Receipt = EDM118::Receipt;
using Feedback = EDM118::Feedback;
constexpr unsigned ArmP = 138U, FinishP = 139U;
constexpr unsigned TargetStep = 2U, TargetSpan = 0U, TargetDwells = 1U;
constexpr int TargetDirection = -1;
constexpr std::uint64_t TargetBackwardCrossings = 0ULL, TargetForwardCrossings = 0ULL;
constexpr EDM114::Kind TargetRowKind = EDM114::Kind::Arc;
constexpr EDM116::Fixture TargetFixture = EDM116::Fixture::Canonical116;
constexpr EDM116::Mode TargetMode = EDM116::Mode::B2;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

inline bool ValidReverseCoordinate(const EDM115::Feedback& sample) noexcept
{
    if (sample.span != TargetSpan || sample.direction != TargetDirection || sample.rowKind != TargetRowKind ||
        !EDM114::Positive(sample.length) || !std::isfinite(sample.q) || !std::isfinite(sample.progress) ||
        sample.q < 0.0 || sample.q > sample.length || sample.progress < 0.0 || sample.progress > sample.length) return false;
    // Computing anchorQ-progress and then adding progress can round. The
    // small arithmetic tolerance does not relax source, clock or motion caps.
    const double origin = sample.q + sample.progress;
    return std::isfinite(origin) && std::fabs(origin - sample.length) <=
        64.0 * std::numeric_limits<double>::epsilon() * sample.length;
}

inline bool TargetSample(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture &&
        policy.mode == TargetMode && source.step == TargetStep &&
        EDM118::ValidInitial(initial) && ValidReverseCoordinate(initial) && initial.phase == EDM115::Phase::Moving &&
        initial.moving && !initial.endpointReady && initial.v < 0.0 &&
        initial.backwardCrossings == TargetBackwardCrossings &&
        initial.forwardCrossings == TargetForwardCrossings && initial.completedDwells == TargetDwells;
}

inline bool RequestWindow(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return TargetSample(policy, source, initial) &&
        initial.progress >= initial.length * RequestProgressBegin &&
        initial.progress <= initial.length * RequestProgressEnd;
}

inline bool TargetReceipt(const Receipt& receipt) noexcept
{
    // The committed RT sample may follow the request window, but must still
    // be moving backwards on this trigger arc with no row-crossing credit.
    return EDM118::ValidReceipt(receipt) && TargetSample(receipt.policy, receipt.trigger, receipt.initial);
}

inline bool ProgressContinues(const EDM115::Feedback& before, const EDM115::Feedback& after) noexcept
{
    // Local progress increases as original-arc q decreases; both samples
    // retain the same frozen anchorQ, length, span and authored arc mapping.
    return ValidReverseCoordinate(before) && ValidReverseCoordinate(after) &&
        EDM109::SameDouble(before.length, after.length) &&
        after.progress >= before.progress && after.q <= before.q;
}
}

// EDM121 observes the retained previous G01 on its forward B2 return,
// before the second row crossing. The trigger remains the original G02;
// this diagnostic contract does not grant stop, RESET or START authority.
namespace EDM121
{
using Point = EDM118::Point;
using Source = EDM118::Source;
using Receipt = EDM118::Receipt;
using Feedback = EDM118::Feedback;
constexpr unsigned ArmP = 140U, FinishP = 141U;
constexpr unsigned TargetStep = 2U, TargetSpan = 2U, TargetDwells = 3U;
constexpr int TargetDirection = 1;
constexpr std::uint64_t TargetBackwardCrossings = 1ULL, TargetForwardCrossings = 0ULL;
constexpr EDM114::Kind TargetRowKind = EDM114::Kind::Line;
constexpr EDM116::Fixture TargetFixture = EDM116::Fixture::Canonical116;
constexpr EDM116::Mode TargetMode = EDM116::Mode::B2;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

inline bool ValidForwardCoordinate(const EDM115::Feedback& sample) noexcept
{
    if (sample.span != TargetSpan || sample.direction != TargetDirection || sample.rowKind != TargetRowKind ||
        !EDM114::Positive(sample.length) || !std::isfinite(sample.q) || !std::isfinite(sample.progress) ||
        sample.q < 0.0 || sample.progress < 0.0 || sample.progress > sample.length) return false;
    // Begin freezes retreat=previousLength*(1-fraction), and span length is
    // previousLength-retreat. Therefore q-progress retains the retreat point
    // at nine span lengths for the existing ten-percent previous-row retreat.
    // Subtracting that span length and reconstructing q may round; use only a
    // small arithmetic tolerance, with source, clocks and dynamics checked by NC.
    const double origin = sample.q - sample.progress;
    const double expectedOrigin = sample.length *
        ((1.0 - EDM115::PreviousRetreatFraction) / EDM115::PreviousRetreatFraction);
    if (!EDM114::Positive(origin) || !EDM114::Positive(expectedOrigin)) return false;
    const double scale = (std::max)(sample.length, (std::max)(origin, expectedOrigin));
    return std::fabs(origin - expectedOrigin) <=
        64.0 * std::numeric_limits<double>::epsilon() * scale;
}

inline bool TargetSample(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture &&
        policy.mode == TargetMode && source.step == TargetStep &&
        EDM118::ValidInitial(initial) && ValidForwardCoordinate(initial) && initial.phase == EDM115::Phase::Moving &&
        initial.moving && !initial.endpointReady && initial.v > 0.0 &&
        initial.backwardCrossings == TargetBackwardCrossings &&
        initial.forwardCrossings == TargetForwardCrossings && initial.completedDwells == TargetDwells;
}

inline bool RequestWindow(const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return TargetSample(policy, source, initial) &&
        initial.progress >= initial.length * RequestProgressBegin &&
        initial.progress <= initial.length * RequestProgressEnd;
}

inline bool TargetReceipt(const Receipt& receipt) noexcept
{
    // Capture can follow the request window, but must remain moving on this
    // retained forward-return line, without recrossing into the trigger arc.
    return EDM118::ValidReceipt(receipt) && TargetSample(receipt.policy, receipt.trigger, receipt.initial);
}

inline bool ProgressContinues(const EDM115::Feedback& before, const EDM115::Feedback& after) noexcept
{
    // Both local progress and original previous-row q increase. Each sample
    // independently retains the frozen ten-percent span origin and length.
    return ValidForwardCoordinate(before) && ValidForwardCoordinate(after) &&
        EDM109::SameDouble(before.length, after.length) &&
        after.progress >= before.progress && after.q >= before.q;
}
}

// EDM122 observes local B0/B1 retreat legs sourced by the authored arcs.
// Their generated line advances from anchor to retreat with positive scalar
// velocity; it does not consume a B2 previous-row completion token.
namespace EDM122
{
using Point = EDM118::Point;
using Source = EDM118::Source;
using Receipt = EDM118::Receipt;
using Feedback = EDM118::Feedback;
enum class Profile : std::uint8_t { B0TangentRetreat = 0U, B1ZRetreat = 1U, Invalid = 255U };
constexpr unsigned B0ArmP = 142U, B0FinishP = 143U, B1ArmP = 144U, B1FinishP = 145U;
constexpr unsigned TargetSpan = 0U, TargetDwells = 1U;
constexpr int TargetDirection = 1;
constexpr std::uint64_t TargetBackwardCrossings = 0ULL, TargetForwardCrossings = 0ULL;
constexpr EDM114::Kind TargetRowKind = EDM114::Kind::Line;
constexpr EDM116::Fixture TargetFixture = EDM116::Fixture::Canonical116;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

inline constexpr Profile ProfileForArm(unsigned arm) noexcept
{
    return arm == B0ArmP ? Profile::B0TangentRetreat : arm == B1ArmP ? Profile::B1ZRetreat : Profile::Invalid;
}

inline constexpr bool ValidProfile(Profile profile) noexcept
{ return profile == Profile::B0TangentRetreat || profile == Profile::B1ZRetreat; }

inline constexpr unsigned TargetStep(Profile profile) noexcept
{
    return profile == Profile::B0TangentRetreat ? 6U : profile == Profile::B1ZRetreat ? 10U : EDM114::RequiredRows;
}

inline constexpr EDM116::Mode TargetMode(Profile profile) noexcept
{
    return profile == Profile::B0TangentRetreat ? EDM116::Mode::B0 :
        profile == Profile::B1ZRetreat ? EDM116::Mode::B1 : EDM116::Mode::Invalid;
}

inline bool ValidLocalCoordinate(const EDM115::Feedback& sample) noexcept
{
    // Local SetSample copies positive progress directly to q in span 0.
    // Require that exact relation, rather than the original arc's anchorQ.
    return sample.span == TargetSpan && sample.direction == TargetDirection && sample.rowKind == TargetRowKind &&
        EDM114::Positive(sample.length) && std::isfinite(sample.q) && std::isfinite(sample.progress) &&
        sample.q >= 0.0 && sample.q <= sample.length && sample.progress >= 0.0 && sample.progress <= sample.length &&
        EDM109::SameDouble(sample.q, sample.progress);
}

inline bool TargetSample(Profile profile, const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return ValidProfile(profile) && EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture &&
        policy.mode == TargetMode(profile) && source.step == TargetStep(profile) &&
        EDM114::IsArcStep(source.step) && policy.mode == EDM116::ExpectedMode(source.step) &&
        EDM118::ValidInitial(initial) && ValidLocalCoordinate(initial) && initial.phase == EDM115::Phase::Moving &&
        initial.moving && !initial.endpointReady && initial.v > 0.0 &&
        initial.backwardCrossings == TargetBackwardCrossings && initial.forwardCrossings == TargetForwardCrossings &&
        initial.completedDwells == TargetDwells;
}

inline bool RequestWindow(Profile profile, const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return TargetSample(profile, policy, source, initial) &&
        initial.progress >= initial.length * RequestProgressBegin &&
        initial.progress <= initial.length * RequestProgressEnd;
}

inline bool TargetReceipt(Profile profile, const Receipt& receipt) noexcept
{
    // ValidReceipt requires an exactly empty previous Source for B0/B1.
    // The first committed capture may follow the NC request window, but must
    // still be moving on this generated retreat line with the same source.
    return EDM118::ValidReceipt(receipt) && TargetSample(profile, receipt.policy, receipt.trigger, receipt.initial);
}

inline bool ProgressContinues(const EDM115::Feedback& before, const EDM115::Feedback& after) noexcept
{
    return ValidLocalCoordinate(before) && ValidLocalCoordinate(after) &&
        EDM109::SameDouble(before.length, after.length) && after.progress >= before.progress && after.q >= before.q;
}
}

// EDM123 observes the generated local B0/B1 return line, before it reaches
// the frozen authored-arc anchor. It shares the existing receipt ABI and
// grants neither machining resume nor manual RESET authority.
namespace EDM123
{
using Point = EDM118::Point;
using Source = EDM118::Source;
using Receipt = EDM118::Receipt;
using Feedback = EDM118::Feedback;
enum class Profile : std::uint8_t { B0TangentReturn = 0U, B1ZReturn = 1U, Invalid = 255U };
constexpr unsigned B0ArmP = 146U, B0FinishP = 147U, B1ArmP = 148U, B1FinishP = 149U;
constexpr unsigned TargetSpan = 1U, TargetDwells = 2U;
constexpr int TargetDirection = -1;
constexpr std::uint64_t TargetBackwardCrossings = 0ULL, TargetForwardCrossings = 0ULL;
constexpr EDM114::Kind TargetRowKind = EDM114::Kind::Line;
constexpr EDM116::Fixture TargetFixture = EDM116::Fixture::Canonical116;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

inline constexpr Profile ProfileForArm(unsigned arm) noexcept
{
    return arm == B0ArmP ? Profile::B0TangentReturn : arm == B1ArmP ? Profile::B1ZReturn : Profile::Invalid;
}

inline constexpr bool ValidProfile(Profile profile) noexcept
{ return profile == Profile::B0TangentReturn || profile == Profile::B1ZReturn; }

inline constexpr unsigned TargetStep(Profile profile) noexcept
{
    return profile == Profile::B0TangentReturn ? 6U : profile == Profile::B1ZReturn ? 10U : EDM114::RequiredRows;
}

inline constexpr EDM116::Mode TargetMode(Profile profile) noexcept
{
    return profile == Profile::B0TangentReturn ? EDM116::Mode::B0 :
        profile == Profile::B1ZReturn ? EDM116::Mode::B1 : EDM116::Mode::Invalid;
}

inline bool ValidLocalCoordinate(const EDM115::Feedback& sample) noexcept
{
    // The generated mapping still points from anchor to retreat. Local
    // SetSample advances return progress and copies length-progress to q;
    // repeat that producer arithmetic exactly instead of adding q+progress,
    // whose sum can round differently. Endpoints match its explicit branches.
    return sample.span == TargetSpan && sample.direction == TargetDirection && sample.rowKind == TargetRowKind &&
        EDM114::Positive(sample.length) && std::isfinite(sample.q) && std::isfinite(sample.progress) &&
        sample.q >= 0.0 && sample.q <= sample.length && sample.progress >= 0.0 && sample.progress <= sample.length &&
        EDM109::SameDouble(sample.q, sample.length - sample.progress);
}

inline bool TargetSample(Profile profile, const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return ValidProfile(profile) && EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture &&
        policy.mode == TargetMode(profile) && source.step == TargetStep(profile) &&
        EDM114::IsArcStep(source.step) && policy.mode == EDM116::ExpectedMode(source.step) &&
        EDM118::ValidInitial(initial) && ValidLocalCoordinate(initial) && initial.phase == EDM115::Phase::Moving &&
        initial.moving && !initial.endpointReady && initial.v < 0.0 &&
        initial.progress > 0.0 && initial.progress < initial.length && initial.q > 0.0 && initial.q < initial.length &&
        initial.backwardCrossings == TargetBackwardCrossings && initial.forwardCrossings == TargetForwardCrossings &&
        initial.completedDwells == TargetDwells;
}

inline bool RequestWindow(Profile profile, const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& initial) noexcept
{
    return TargetSample(profile, policy, source, initial) &&
        initial.progress >= initial.length * RequestProgressBegin &&
        initial.progress <= initial.length * RequestProgressEnd;
}

inline bool TargetReceipt(Profile profile, const Receipt& receipt) noexcept
{
    // B0/B1 require an exactly empty previous Source. A delayed first capture
    // may follow the request window, but remains on the same moving return
    // line; endpoint settle and normal completion are independently rejected.
    return EDM118::ValidReceipt(receipt) && TargetSample(profile, receipt.policy, receipt.trigger, receipt.initial);
}

inline bool ProgressContinues(const EDM115::Feedback& before, const EDM115::Feedback& after) noexcept
{
    return ValidLocalCoordinate(before) && ValidLocalCoordinate(after) &&
        EDM109::SameDouble(before.length, after.length) && after.progress >= before.progress && after.q <= before.q;
}
}

// EDM125 observes the retained previous authored arc at the second B2 event.
// The existing committed receipt and physical-zero proof remain unchanged.
namespace EDM125
{
using Point = EDM118::Point;
using Source = EDM118::Source;
using Receipt = EDM118::Receipt;
using Feedback = EDM118::Feedback;
enum class Profile : std::uint8_t { PreviousArcRetreat = 0U, PreviousArcReturn = 1U, Invalid = 255U };
constexpr unsigned RetreatArmP = 154U, RetreatFinishP = 155U, ReturnArmP = 156U, ReturnFinishP = 157U;
constexpr unsigned TargetStep = 4U;
constexpr EDM116::Mode TargetMode = EDM116::Mode::B2;
constexpr EDM114::Kind TargetRowKind = EDM114::Kind::Arc;
constexpr std::uint64_t TargetBackwardCrossings = 1ULL, TargetForwardCrossings = 0ULL;
constexpr double RequestProgressBegin = 0.25, RequestProgressEnd = 0.50;

inline constexpr Profile ProfileForArm(unsigned arm) noexcept
{ return arm == RetreatArmP ? Profile::PreviousArcRetreat : arm == ReturnArmP ? Profile::PreviousArcReturn : Profile::Invalid; }
inline constexpr bool ValidProfile(Profile profile) noexcept
{ return profile == Profile::PreviousArcRetreat || profile == Profile::PreviousArcReturn; }
inline constexpr EDM116::Fixture TargetFixture(Profile profile) noexcept
{ return profile == Profile::PreviousArcRetreat ? EDM116::Fixture::ArcPair124 :
    profile == Profile::PreviousArcReturn ? EDM116::Fixture::ArcPairComplement124 : static_cast<EDM116::Fixture>(255U); }
inline constexpr unsigned TargetSpan(Profile profile) noexcept
{ return profile == Profile::PreviousArcRetreat ? 1U : profile == Profile::PreviousArcReturn ? 2U : 4U; }
inline constexpr int TargetDirection(Profile profile) noexcept
{ return profile == Profile::PreviousArcRetreat ? -1 : profile == Profile::PreviousArcReturn ? 1 : 0; }
inline constexpr unsigned TargetDwells(Profile profile) noexcept
{ return profile == Profile::PreviousArcRetreat ? 2U : profile == Profile::PreviousArcReturn ? 3U : 0U; }

inline bool ValidCoordinate(Profile profile, const EDM115::Feedback& sample) noexcept
{
    if (!ValidProfile(profile) || sample.span != TargetSpan(profile) || sample.direction != TargetDirection(profile) ||
        sample.rowKind != TargetRowKind || !EDM114::Positive(sample.length) ||
        !std::isfinite(sample.q) || !std::isfinite(sample.progress) || sample.q <= 0.0 ||
        sample.progress < 0.0 || sample.progress > sample.length) return false;
    // Both legs retain the previous arc's ten-percent span. Reverse q+p is
    // its full length; forward q-p is its ninety-percent retreat point.
    const double origin = sample.q - static_cast<double>(TargetDirection(profile)) * sample.progress;
    const double expectedOrigin = sample.length * (profile == Profile::PreviousArcRetreat ?
        1.0 / EDM115::PreviousRetreatFraction : (1.0 - EDM115::PreviousRetreatFraction) / EDM115::PreviousRetreatFraction);
    if (!EDM114::Positive(origin) || !EDM114::Positive(expectedOrigin)) return false;
    const double scale = (std::max)(sample.length, (std::max)(origin, expectedOrigin));
    return std::fabs(origin - expectedOrigin) <= 64.0 * std::numeric_limits<double>::epsilon() * scale;
}
inline bool TargetSample(Profile profile, const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& sample) noexcept
{
    return ValidProfile(profile) && EDM116::PolicyMatchesSource(policy, source) && policy.fixture == TargetFixture(profile) &&
        policy.mode == TargetMode && source.step == TargetStep && EDM118::ValidInitial(sample) && ValidCoordinate(profile, sample) &&
        sample.phase == EDM115::Phase::Moving && sample.moving && !sample.endpointReady &&
        sample.v * static_cast<double>(TargetDirection(profile)) > 0.0 && sample.progress > 0.0 && sample.progress < sample.length &&
        sample.backwardCrossings == TargetBackwardCrossings && sample.forwardCrossings == TargetForwardCrossings &&
        sample.completedDwells == TargetDwells(profile);
}
inline bool RequestWindow(Profile profile, const EDM116::Policy& policy, const Source& source,
    const EDM115::Feedback& sample) noexcept
{
    return TargetSample(profile, policy, source, sample) && sample.progress >= sample.length * RequestProgressBegin &&
        sample.progress <= sample.length * RequestProgressEnd;
}
inline bool TargetReceipt(Profile profile, const Receipt& receipt) noexcept
{
    return EDM118::ValidReceipt(receipt) && TargetSample(profile, receipt.policy, receipt.trigger, receipt.initial) &&
        receipt.previous.step + 1U == TargetStep;
}
inline bool ProgressContinues(Profile profile, const EDM115::Feedback& before, const EDM115::Feedback& after) noexcept
{
    return ValidCoordinate(profile, before) && ValidCoordinate(profile, after) &&
        EDM109::SameDouble(before.length, after.length) && after.progress >= before.progress &&
        (profile == Profile::PreviousArcRetreat ? after.q <= before.q : after.q >= before.q);
}
}
