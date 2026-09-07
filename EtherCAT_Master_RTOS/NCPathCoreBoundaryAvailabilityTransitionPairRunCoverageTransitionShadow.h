#pragma once

#include "NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2X / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Pair Continuity Run Pattern Coverage Transition Shadow.
//
// Describes only one directly adjacent, proven W summary pair in the same
// observed W/V local run. Both W records pass their complete scalar proof;
// five publication advances, count, run identities, shared T relation and
// exact mask union must agree. The masks describe presence of eight U pair
// patterns, not qualification, full closure, stable availability or paths.
// Newly observed bits are current & ~previous; retained bits are current &
// previous. One extension can add at most one bit. A proven loss or full
// coverage exit is impossible under W's monotone union recurrence. Such a
// mask mismatch is invalid, never an inferred loss or an event.
//
// X may attach or recover using any valid W extension pair: this certificate
// makes no claim about earlier X observations. W still requires its actual
// observed V head. A W head, a different W run, unavailable/nonproven input,
// malformed proof, stale/gapped publication or binding failure never becomes
// a proven transition. Rejections clear every proof field and keep at most
// current W publication as a diagnostic anchor; null clears that anchor.
// When retained X is proven, the supplied previous W must match its complete
// current-W semantic projection, including mask, relations and latest reason.
// X's own publication counter also binds its retained newest record.
//
// Previous W's current U relation is retained to check previous-mask latest
// membership and the two-pair seed relation. Earlier reason and individual
// pattern order/count/duration cannot be reconstructed. The T unavailable
// reason omitted by U remains unavailable here. Trusted same-thread sources
// are required. A structurally coherent accumulated-mask alteration, even
// in one supplied record at fresh attachment, may pass these pair fences;
// X derives differences from trusted summaries, not independent event time.
// Coherent forgery, full-wrap identity reuse, wall-clock freshness and
// authentication of discarded history are excluded.
//
// Exactly two 112-byte scalar records; 240-byte NCManager heap-owned observer.
// No Observe-time allocation, W/stack snapshot, coordinates, displacement,
// endpoint arrays, nodes, segment/event list, full order or reverse traversal.
// Not a Path/Motion Queue, planner, STARTED/DONE proof, actual-position
// history, B2 breadcrumb or B2 execution. Shadow-only; no control consumer.
// No Motion, G00, Gate/Registry, Alarm, PC, HMI/SHM/API or PDO/DC changes;
// no log, thread, timer, mutex, wait or sleep. M00 FIX1 and filenames preserved.

constexpr std::size_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_SCHEMA_V1 = 1U;

enum class NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_COVERAGE_UNAVAILABLE = 2U,
    NOT_APPLICABLE_CURRENT_COVERAGE_NOT_PROVEN = 3U,
    NOT_APPLICABLE_PREVIOUS_COVERAGE_NOT_PROVEN = 4U,
    NOT_APPLICABLE_CURRENT_COVERAGE_RUN_HEAD = 5U,
    NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN = 6U,
    SOURCE_REPORTED_CURRENT_COVERAGE_INVALID = 7U,
    SOURCE_REPORTED_PREVIOUS_COVERAGE_INVALID = 8U,
    INVALID_CURRENT_COVERAGE_RECORD = 9U,
    INVALID_PREVIOUS_COVERAGE_RECORD = 10U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 11U,
    INVALID_PREVIOUS_TRANSITION_RECORD = 12U,
    INVALID_PREVIOUS_SOURCE_BINDING = 13U,
    INVALID_COVERAGE_ADVANCE = 14U,
    INVALID_SHARED_TRANSITION_BINDING = 15U,
    INVALID_COVERAGE_MASK_RECURRENCE = 16U,
    INVALID_COVERAGE_TRANSITION = 17U,
    PROVEN_DIRECT_RUN_PATTERN_COVERAGE_TRANSITION = 18U
};

enum class NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionFullCoverageTransition : std::uint8_t
{
    NONE = 0U,
    NOT_FULL = 1U,
    BECAME_FULL = 2U,
    RETAINED_FULL = 3U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
#endif

namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail
{
    using CoverageRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1;
    using PairRelation = CoverageRecord::PairRelation;
    using TransitionRelation = CoverageRecord::TransitionRelation;
    using Reason = CoverageRecord::UnavailableReason;
    using FullCoverageTransition = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionFullCoverageTransition;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::NextNonZeroSequence;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::ForwardNonZeroSequenceDistance;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::PairRelationBit;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::PopulationCount;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::PairCurrentTransition;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        inline TransitionRelation PairPreviousTransition(PairRelation relation) noexcept
    {
        if (relation == PairRelation::GAIN_THEN_RETAIN || relation == PairRelation::GAIN_THEN_LOSS)
            return TransitionRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD;
        if (relation == PairRelation::RETAIN_THEN_RETAIN || relation == PairRelation::RETAIN_THEN_LOSS)
            return TransitionRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION;
        if (relation == PairRelation::LOSS_THEN_GAIN || relation == PairRelation::LOSS_THEN_UNAVAILABLE)
            return TransitionRelation::BECAME_UNAVAILABLE;
        if (relation == PairRelation::UNAVAILABLE_THEN_GAIN || relation == PairRelation::UNAVAILABLE_THEN_UNAVAILABLE)
            return TransitionRelation::REMAINED_UNAVAILABLE;
        return TransitionRelation::NONE;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        inline FullCoverageTransition ClassifyFullCoverage(std::uint8_t previous, std::uint8_t current) noexcept
    {
        return current != 0xFFU ? FullCoverageTransition::NOT_FULL :
            previous == 0xFFU ? FullCoverageTransition::RETAINED_FULL : FullCoverageTransition::BECAME_FULL;
    }
}

struct NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1
{
    using Disposition = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDisposition;
    using PairRelation = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::PairRelation;
    using TransitionRelation = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::TransitionRelation;
    using UnavailableReason = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::Reason;
    using FullCoverageTransition = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::FullCoverageTransition;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t currentCoveragePublicationSequence = 0ULL;
    std::uint64_t coverageGeneration = 0ULL;
    std::uint64_t sourceRunPublicationSequence = 0ULL;
    std::uint64_t continuityRunGeneration = 0ULL;
    std::uint64_t firstPairPublicationSequence = 0ULL;
    std::uint64_t sourcePairPublicationSequence = 0ULL;
    std::uint64_t firstTransitionPublicationSequence = 0ULL;
    std::uint64_t currentTransitionPublicationSequence = 0ULL;
    std::uint64_t firstBoundaryPublicationSequence = 0ULL;
    std::uint64_t currentBoundaryPublicationSequence = 0ULL;
    std::uint32_t consecutivePairCount = 0U;
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    std::uint8_t previousObservedPairRelationMask = 0U;
    std::uint8_t currentObservedPairRelationMask = 0U;
    std::uint8_t newlyObservedPairRelationMask = 0U;
    std::uint8_t retainedPairRelationMask = 0U;
    FullCoverageTransition fullCoverageTransition = FullCoverageTransition::NONE;
    PairRelation firstPairRelation = PairRelation::NONE;
    PairRelation sourcePairRelation = PairRelation::NONE;
    PairRelation previousSourcePairRelation = PairRelation::NONE;
    TransitionRelation previousTransitionRelation = TransitionRelation::NONE;
    TransitionRelation currentTransitionRelation = TransitionRelation::NONE;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::array<std::uint8_t, 6U> reserved{};

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        bool IsProvenBoundaryAvailabilityTransitionPairRunCoverageTransition() const noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail;
        if (publicationSequence == 0ULL || currentCoveragePublicationSequence == 0ULL || coverageGeneration == 0ULL ||
            sourceRunPublicationSequence == 0ULL || continuityRunGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL || sourcePairPublicationSequence == 0ULL ||
            firstTransitionPublicationSequence == 0ULL || currentTransitionPublicationSequence == 0ULL ||
            firstBoundaryPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            consecutivePairCount < 3U ||
            schemaVersion != NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_SCHEMA_V1 ||
            disposition != Disposition::PROVEN_DIRECT_RUN_PATTERN_COVERAGE_TRANSITION ||
            sourcePairRelation == PairRelation::NONE ||
            sourcePairRelation != NCPathCoreBoundaryAvailabilityTransitionPairDetail::Classify(
                previousTransitionRelation, currentTransitionRelation) ||
            !NCPathCoreBoundaryAvailabilityTransitionPairDetail::CurrentReasonMatches(
                currentTransitionRelation, currentUnavailableReason)) return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        const std::uint8_t firstBit = PairRelationBit(firstPairRelation);
        const std::uint8_t previousBit = PairRelationBit(previousSourcePairRelation);
        const std::uint8_t currentBit = PairRelationBit(sourcePairRelation);
        const std::uint8_t previousRequired = static_cast<std::uint8_t>(firstBit | previousBit);
        if (firstBit == 0U || previousBit == 0U || currentBit == 0U ||
            (previousObservedPairRelationMask & previousRequired) != previousRequired ||
            PopulationCount(previousObservedPairRelationMask) > consecutivePairCount - 1U ||
            PairCurrentTransition(previousSourcePairRelation) != previousTransitionRelation ||
            currentObservedPairRelationMask != static_cast<std::uint8_t>(previousObservedPairRelationMask | currentBit) ||
            PopulationCount(currentObservedPairRelationMask) > consecutivePairCount ||
            newlyObservedPairRelationMask != static_cast<std::uint8_t>(currentObservedPairRelationMask &
                static_cast<std::uint8_t>(~previousObservedPairRelationMask)) ||
            retainedPairRelationMask != static_cast<std::uint8_t>(
                currentObservedPairRelationMask & previousObservedPairRelationMask) ||
            fullCoverageTransition != ClassifyFullCoverage(previousObservedPairRelationMask, currentObservedPairRelationMask))
            return false;
        // At count three the predecessor was W's actual two-U seed.
        if (consecutivePairCount == 3U && (previousObservedPairRelationMask != previousRequired ||
            PairCurrentTransition(firstPairRelation) != PairPreviousTransition(previousSourcePairRelation))) return false;
        const std::uint64_t sourceDistance = static_cast<std::uint64_t>(consecutivePairCount) - 1ULL;
        return ForwardNonZeroSequenceDistance(coverageGeneration, currentCoveragePublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(continuityRunGeneration, sourceRunPublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, sourcePairPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstTransitionPublicationSequence, currentTransitionPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstBoundaryPublicationSequence, currentBoundaryPublicationSequence) == sourceDistance;
    }
};

class NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow final
{
public:
    using Record = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1;
    using Disposition = Record::Disposition;
    using PairRelation = Record::PairRelation;
    using TransitionRelation = Record::TransitionRelation;
    using UnavailableReason = Record::UnavailableReason;
    using FullCoverageTransition = Record::FullCoverageTransition;
    using CoverageRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::CoverageRecord;
    using CoverageDisposition = CoverageRecord::Disposition;

    NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow() noexcept = default;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        void ObserveLatestBoundaryAvailabilityTransitionPairRunCoverageSameThread(
            const CoverageRecord* previousCoverage, const CoverageRecord* currentCoverage) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenBoundaryAvailabilityTransitionPairRunCoverageTransition();
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            (previousProven || IsCanonicalNonProvenTransition(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentCoveragePublicationSequence;
        m_publicationSequence = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentCoverage == nullptr ? 0ULL : currentCoverage->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        else if (currentCoverage == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE;
        else if (anchor != 0ULL && currentCoverage->publicationSequence !=
            NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluateTransition(previousCoverage, *currentCoverage, previous, previousProven, target);
        m_latestIndex = static_cast<std::uint8_t>(index);
        if (m_recordCount < HISTORY_CAPACITY) ++m_recordCount;
    }

    const Record* GetNewestObservationSameThread(std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount) return nullptr;
        const std::size_t index = (static_cast<std::size_t>(m_latestIndex) + HISTORY_CAPACITY - historyOffset) % HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept { return m_recordCount; }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_HISTORY_CAPACITY;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static bool IsCanonicalNonProvenCoverage(const CoverageRecord& source) noexcept
    {
        const bool known = source.disposition >= CoverageDisposition::NOT_APPLICABLE_RUN_UNAVAILABLE &&
            source.disposition <= CoverageDisposition::INVALID_COVERAGE_SUMMARY;
        const bool identity = source.disposition == CoverageDisposition::NOT_APPLICABLE_RUN_UNAVAILABLE
            ? source.sourceRunPublicationSequence == 0ULL
            : source.sourceRunPublicationSequence != 0ULL ||
            source.disposition == CoverageDisposition::INVALID_CURRENT_RUN_RECORD ||
            source.disposition == CoverageDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == CoverageDisposition::INVALID_PREVIOUS_COVERAGE_RECORD;
        if (!known || !identity || source.publicationSequence == 0ULL ||
            source.schemaVersion != NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_SCHEMA_V1 ||
            source.coverageGeneration != 0ULL || source.continuityRunGeneration != 0ULL ||
            source.firstPairPublicationSequence != 0ULL || source.sourcePairPublicationSequence != 0ULL ||
            source.firstTransitionPublicationSequence != 0ULL || source.currentTransitionPublicationSequence != 0ULL ||
            source.firstBoundaryPublicationSequence != 0ULL || source.currentBoundaryPublicationSequence != 0ULL ||
            source.consecutivePairCount != 0U || source.observedPairRelationMask != 0U ||
            source.firstPairRelation != PairRelation::NONE || source.sourcePairRelation != PairRelation::NONE ||
            source.previousTransitionRelation != TransitionRelation::NONE || source.currentTransitionRelation != TransitionRelation::NONE ||
            source.currentUnavailableReason != UnavailableReason::NONE) return false;
        for (std::uint8_t value : source.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static bool IsNeutralCoverageDisposition(CoverageDisposition disposition) noexcept
    {
        return disposition == CoverageDisposition::NOT_APPLICABLE_RUN_UNAVAILABLE ||
            disposition == CoverageDisposition::NOT_APPLICABLE_RUN_NOT_PROVEN ||
            disposition == CoverageDisposition::NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED ||
            disposition == CoverageDisposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE ||
            disposition == CoverageDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static bool IsCanonicalNonProvenTransition(const Record& source) noexcept
    {
        const bool known = source.disposition >= Disposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE &&
            source.disposition <= Disposition::INVALID_COVERAGE_TRANSITION;
        const bool identity = source.disposition == Disposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE
            ? source.currentCoveragePublicationSequence == 0ULL
            : source.currentCoveragePublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_COVERAGE_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        if (!known || !identity || source.publicationSequence == 0ULL ||
            source.schemaVersion != NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_SCHEMA_V1 ||
            source.coverageGeneration != 0ULL || source.sourceRunPublicationSequence != 0ULL ||
            source.continuityRunGeneration != 0ULL || source.firstPairPublicationSequence != 0ULL ||
            source.sourcePairPublicationSequence != 0ULL || source.firstTransitionPublicationSequence != 0ULL ||
            source.currentTransitionPublicationSequence != 0ULL || source.firstBoundaryPublicationSequence != 0ULL ||
            source.currentBoundaryPublicationSequence != 0ULL || source.consecutivePairCount != 0U ||
            source.previousObservedPairRelationMask != 0U || source.currentObservedPairRelationMask != 0U ||
            source.newlyObservedPairRelationMask != 0U || source.retainedPairRelationMask != 0U ||
            source.fullCoverageTransition != FullCoverageTransition::NONE || source.firstPairRelation != PairRelation::NONE ||
            source.sourcePairRelation != PairRelation::NONE || source.previousSourcePairRelation != PairRelation::NONE ||
            source.previousTransitionRelation != TransitionRelation::NONE || source.currentTransitionRelation != TransitionRelation::NONE ||
            source.currentUnavailableReason != UnavailableReason::NONE) return false;
        for (std::uint8_t value : source.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static bool PreviousCoverageMatchesTransition(const CoverageRecord& source, const Record& previous) noexcept
    {
        return source.publicationSequence == previous.currentCoveragePublicationSequence &&
            source.coverageGeneration == previous.coverageGeneration &&
            source.sourceRunPublicationSequence == previous.sourceRunPublicationSequence &&
            source.continuityRunGeneration == previous.continuityRunGeneration &&
            source.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            source.sourcePairPublicationSequence == previous.sourcePairPublicationSequence &&
            source.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            source.currentTransitionPublicationSequence == previous.currentTransitionPublicationSequence &&
            source.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            source.currentBoundaryPublicationSequence == previous.currentBoundaryPublicationSequence &&
            source.consecutivePairCount == previous.consecutivePairCount &&
            source.observedPairRelationMask == previous.currentObservedPairRelationMask &&
            source.firstPairRelation == previous.firstPairRelation && source.sourcePairRelation == previous.sourcePairRelation &&
            source.previousTransitionRelation == previous.previousTransitionRelation &&
            source.currentTransitionRelation == previous.currentTransitionRelation &&
            source.currentUnavailableReason == previous.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static bool SameCoverageRun(const CoverageRecord& previous, const CoverageRecord& current) noexcept
    {
        return current.coverageGeneration == previous.coverageGeneration &&
            current.continuityRunGeneration == previous.continuityRunGeneration &&
            current.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            current.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            current.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            current.firstPairRelation == previous.firstPairRelation;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static bool DirectCoverageAdvance(const CoverageRecord& previous, const CoverageRecord& current) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail;
        return previous.consecutivePairCount != (std::numeric_limits<std::uint32_t>::max)() &&
            current.consecutivePairCount == previous.consecutivePairCount + 1U &&
            current.publicationSequence == NextNonZeroSequence(previous.publicationSequence) &&
            current.sourceRunPublicationSequence == NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence == NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            current.currentTransitionPublicationSequence == NextNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            current.currentBoundaryPublicationSequence == NextNonZeroSequence(previous.currentBoundaryPublicationSequence);
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static void EvaluateTransition(const CoverageRecord* previousCoverage, const CoverageRecord& currentCoverage,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (!currentCoverage.IsProvenBoundaryAvailabilityTransitionPairRunCoverage())
        {
            target.disposition = !IsCanonicalNonProvenCoverage(currentCoverage) ? Disposition::INVALID_CURRENT_COVERAGE_RECORD :
                IsNeutralCoverageDisposition(currentCoverage.disposition) ? Disposition::NOT_APPLICABLE_CURRENT_COVERAGE_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_CURRENT_COVERAGE_INVALID;
            return;
        }
        if (currentCoverage.disposition == CoverageDisposition::PROVEN_RUN_PATTERN_COVERAGE_START)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CURRENT_COVERAGE_RUN_HEAD;
            return;
        }
        if (previousCoverage == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_COVERAGE_UNAVAILABLE;
            return;
        }
        if (!previousCoverage->IsProvenBoundaryAvailabilityTransitionPairRunCoverage())
        {
            target.disposition = !IsCanonicalNonProvenCoverage(*previousCoverage) ? Disposition::INVALID_PREVIOUS_COVERAGE_RECORD :
                IsNeutralCoverageDisposition(previousCoverage->disposition) ? Disposition::NOT_APPLICABLE_PREVIOUS_COVERAGE_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_PREVIOUS_COVERAGE_INVALID;
            return;
        }
        if (previousProven && previous != nullptr && !PreviousCoverageMatchesTransition(*previousCoverage, *previous))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (!SameCoverageRun(*previousCoverage, currentCoverage))
        {
            target.disposition = Disposition::NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN;
            return;
        }
        if (!DirectCoverageAdvance(*previousCoverage, currentCoverage))
        {
            target.disposition = Disposition::INVALID_COVERAGE_ADVANCE;
            return;
        }
        if (currentCoverage.previousTransitionRelation != previousCoverage->currentTransitionRelation)
        {
            target.disposition = Disposition::INVALID_SHARED_TRANSITION_BINDING;
            return;
        }
        if (currentCoverage.observedPairRelationMask != static_cast<std::uint8_t>(previousCoverage->observedPairRelationMask |
            NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::PairRelationBit(currentCoverage.sourcePairRelation)))
        {
            target.disposition = Disposition::INVALID_COVERAGE_MASK_RECURRENCE;
            return;
        }
        BuildTransition(*previousCoverage, currentCoverage, target);
        if (!target.IsProvenBoundaryAvailabilityTransitionPairRunCoverageTransition())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_COVERAGE_TRANSITION;
        }
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static void BuildTransition(const CoverageRecord& previous, const CoverageRecord& current, Record& target) noexcept
    {
        target.coverageGeneration = current.coverageGeneration;
        target.sourceRunPublicationSequence = current.sourceRunPublicationSequence;
        target.continuityRunGeneration = current.continuityRunGeneration;
        target.firstPairPublicationSequence = current.firstPairPublicationSequence;
        target.sourcePairPublicationSequence = current.sourcePairPublicationSequence;
        target.firstTransitionPublicationSequence = current.firstTransitionPublicationSequence;
        target.currentTransitionPublicationSequence = current.currentTransitionPublicationSequence;
        target.firstBoundaryPublicationSequence = current.firstBoundaryPublicationSequence;
        target.currentBoundaryPublicationSequence = current.currentBoundaryPublicationSequence;
        target.consecutivePairCount = current.consecutivePairCount;
        target.previousObservedPairRelationMask = previous.observedPairRelationMask;
        target.currentObservedPairRelationMask = current.observedPairRelationMask;
        target.newlyObservedPairRelationMask = static_cast<std::uint8_t>(current.observedPairRelationMask &
            static_cast<std::uint8_t>(~previous.observedPairRelationMask));
        target.retainedPairRelationMask = static_cast<std::uint8_t>(current.observedPairRelationMask & previous.observedPairRelationMask);
        target.fullCoverageTransition = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::ClassifyFullCoverage(
            previous.observedPairRelationMask, current.observedPairRelationMask);
        target.firstPairRelation = current.firstPairRelation;
        target.sourcePairRelation = current.sourcePairRelation;
        target.previousSourcePairRelation = previous.sourcePairRelation;
        target.previousTransitionRelation = current.previousTransitionRelation;
        target.currentTransitionRelation = current.currentTransitionRelation;
        target.currentUnavailableReason = current.currentUnavailableReason;
        target.disposition = Disposition::PROVEN_DIRECT_RUN_PATTERN_COVERAGE_TRANSITION;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        target.coverageGeneration = 0ULL;
        target.sourceRunPublicationSequence = 0ULL;
        target.continuityRunGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.sourcePairPublicationSequence = 0ULL;
        target.firstTransitionPublicationSequence = 0ULL;
        target.currentTransitionPublicationSequence = 0ULL;
        target.firstBoundaryPublicationSequence = 0ULL;
        target.currentBoundaryPublicationSequence = 0ULL;
        target.consecutivePairCount = 0U;
        target.previousObservedPairRelationMask = 0U;
        target.currentObservedPairRelationMask = 0U;
        target.newlyObservedPairRelationMask = 0U;
        target.retainedPairRelationMask = 0U;
        target.fullCoverageTransition = FullCoverageTransition::NONE;
        target.firstPairRelation = PairRelation::NONE;
        target.sourcePairRelation = PairRelation::NONE;
        target.previousSourcePairRelation = PairRelation::NONE;
        target.previousTransitionRelation = TransitionRelation::NONE;
        target.currentTransitionRelation = TransitionRelation::NONE;
        target.currentUnavailableReason = UnavailableReason::NONE;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentCoveragePublicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_TRANSITION_NOINLINE

static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDisposition) == 1U, "X disposition one byte.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionFullCoverageTransition) == 1U, "X full coverage transition one byte.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1>::value, "X record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1>::value, "X record scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1) == 112U, "X record exactly 112 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1) == 8U, "X record alignment.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, consecutivePairCount) == 88U, "X count offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, schemaVersion) == 92U, "X schema offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, disposition) == 94U, "X disposition offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, previousObservedPairRelationMask) == 95U, "X previous mask offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, currentObservedPairRelationMask) == 96U, "X current mask offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, newlyObservedPairRelationMask) == 97U, "X newly mask offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, retainedPairRelationMask) == 98U, "X retained mask offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, fullCoverageTransition) == 99U, "X full coverage offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, firstPairRelation) == 100U, "X first relation offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, sourcePairRelation) == 101U, "X current relation offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, previousSourcePairRelation) == 102U, "X previous relation offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, previousTransitionRelation) == 103U, "X previous transition offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, currentTransitionRelation) == 104U, "X current transition offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, currentUnavailableReason) == 105U, "X reason offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1, reserved) == 106U, "X reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow>::value, "X observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow>::value, "X observer scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow) == 240U, "X observer exactly 240 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow) == 8U, "X observer alignment.");
