#pragma once

#include "NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2Y / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Pair Continuity Run Pattern Coverage Transition
// Pair Continuity Shadow.
//
// Describes only two directly adjacent, fully proven X transitions in the
// same observed W/V run. Six publications (X/W/V/U/T/S), count, run heads,
// the shared W mask, U relation and T relation must bind. Discovery pairs
// describe whether each X transition adds a pattern bit or retains its mask;
// full-coverage pairs describe only the eight-pattern mask state. No mask
// loss, coverage exit, interval interruption or invalid proof is converted
// into a normal transition. W's observed-head requirement remains intact.
//
// Y may attach or recover on any valid adjacent X pair; this is a pair
// certificate, not a Y run-head or earlier-observation claim. Rejections
// clear every proof field and retain at most current X publication as a
// diagnostic anchor; null clears it. The newest retained Y record must be
// canonical and bound to Y's own publication counter. When it is proven,
// supplied previous X must match its complete current-X semantic projection.
//
// Three mask scalars and three latest U relation scalars validate the two
// local union steps, including the earliest two-U seed at count four. They
// are not a transition/event list and do not reconstruct full event order,
// discarded history, duration, positions, committed segments or geometry.
// The earlier X's current unavailable reason is not retained in Y, so Y's
// standalone proof cannot reconstruct or authenticate that omitted reason.
// X itself omits the preceding W reason: Y cannot cross-bind that shared-W
// reason, and does not invent one. Both actual supplied X records must pass
// their complete proofs, including each available current reason.
//
// Trusted same-thread sources are required. Structurally coherent altered
// accumulated masks may pass these local fences; discovery refers to the
// trusted W summaries, not independently verified first event occurrence.
// Coherent forgery, full-wrap identity reuse, wall-clock freshness and
// authentication of discarded history are outside this scalar certificate.
//
// Exactly two 120-byte records; 256-byte NCManager heap-owned observer.
// No Observe-time allocation, X/W stack snapshot, coordinates, displacement,
// endpoint arrays, nodes, segment/event list or reverse traversal. Not a
// Path/Motion Queue, planner, STARTED/DONE proof, actual-position history,
// B2 breadcrumb or B2 execution. Shadow-only; no control consumer. No Motion,
// G00, Gate/Registry, Alarm, PC, HMI/SHM/API or PDO/DC changes; no log, thread,
// timer, mutex, wait or sleep. Existing M00 FIX1 and filenames are preserved.

constexpr std::size_t NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_SCHEMA_V1 = 1U;

enum class NCPathCoreRunCoverageTransitionPairDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE = 2U,
    NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN = 3U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN = 4U,
    NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN = 5U,
    SOURCE_REPORTED_CURRENT_TRANSITION_INVALID = 6U,
    SOURCE_REPORTED_PREVIOUS_TRANSITION_INVALID = 7U,
    INVALID_CURRENT_TRANSITION_RECORD = 8U,
    INVALID_PREVIOUS_TRANSITION_RECORD = 9U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 10U,
    INVALID_PREVIOUS_PAIR_RECORD = 11U,
    INVALID_PREVIOUS_SOURCE_BINDING = 12U,
    INVALID_TRANSITION_ADVANCE = 13U,
    INVALID_SHARED_COVERAGE_BINDING = 14U,
    INVALID_TRANSITION_PAIR = 15U,
    PROVEN_DIRECT_RUN_COVERAGE_TRANSITION_PAIR_CONTINUITY = 16U
};

enum class NCPathCoreRunCoverageTransitionPairDiscoveryPairRelation : std::uint8_t
{
    NONE = 0U,
    GAIN_THEN_GAIN = 1U,
    GAIN_THEN_RETAIN = 2U,
    RETAIN_THEN_GAIN = 3U,
    RETAIN_THEN_RETAIN = 4U
};

enum class NCPathCoreRunCoverageTransitionPairFullCoveragePairRelation : std::uint8_t
{
    NONE = 0U,
    PARTIAL_THEN_PARTIAL = 1U,
    PARTIAL_THEN_ENTRY = 2U,
    ENTRY_THEN_RETAINED = 3U,
    RETAINED_THEN_RETAINED = 4U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
#endif

namespace NCPathCoreRunCoverageTransitionPairDetail
{
    using TransitionRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionRecordV1;
    using PairRelation = TransitionRecord::PairRelation;
    using TransitionRelation = TransitionRecord::TransitionRelation;
    using Reason = TransitionRecord::UnavailableReason;
    using FullCoverageTransition = TransitionRecord::FullCoverageTransition;
    using DiscoveryPairRelation = NCPathCoreRunCoverageTransitionPairDiscoveryPairRelation;
    using FullCoveragePairRelation = NCPathCoreRunCoverageTransitionPairFullCoveragePairRelation;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::NextNonZeroSequence;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::ForwardNonZeroSequenceDistance;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::PairRelationBit;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::PopulationCount;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::PairCurrentTransition;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::PairPreviousTransition;
    using NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionDetail::ClassifyFullCoverage;

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        inline DiscoveryPairRelation ClassifyDiscoveryPair(std::uint8_t previousNew, std::uint8_t currentNew) noexcept
    {
        if (previousNew != 0U)
            return currentNew != 0U ? DiscoveryPairRelation::GAIN_THEN_GAIN : DiscoveryPairRelation::GAIN_THEN_RETAIN;
        return currentNew != 0U ? DiscoveryPairRelation::RETAIN_THEN_GAIN : DiscoveryPairRelation::RETAIN_THEN_RETAIN;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        inline FullCoveragePairRelation ClassifyFullCoveragePair(
            FullCoverageTransition previous, FullCoverageTransition current) noexcept
    {
        if (previous == FullCoverageTransition::NOT_FULL)
        {
            if (current == FullCoverageTransition::NOT_FULL) return FullCoveragePairRelation::PARTIAL_THEN_PARTIAL;
            if (current == FullCoverageTransition::BECAME_FULL) return FullCoveragePairRelation::PARTIAL_THEN_ENTRY;
        }
        else if (current == FullCoverageTransition::RETAINED_FULL)
        {
            if (previous == FullCoverageTransition::BECAME_FULL) return FullCoveragePairRelation::ENTRY_THEN_RETAINED;
            if (previous == FullCoverageTransition::RETAINED_FULL) return FullCoveragePairRelation::RETAINED_THEN_RETAINED;
        }
        return FullCoveragePairRelation::NONE;
    }
}

struct NCPathCoreRunCoverageTransitionPairRecordV1
{
    using Disposition = NCPathCoreRunCoverageTransitionPairDisposition;
    using PairRelation = NCPathCoreRunCoverageTransitionPairDetail::PairRelation;
    using TransitionRelation = NCPathCoreRunCoverageTransitionPairDetail::TransitionRelation;
    using UnavailableReason = NCPathCoreRunCoverageTransitionPairDetail::Reason;
    using FullCoverageTransition = NCPathCoreRunCoverageTransitionPairDetail::FullCoverageTransition;
    using DiscoveryPairRelation = NCPathCoreRunCoverageTransitionPairDetail::DiscoveryPairRelation;
    using FullCoveragePairRelation = NCPathCoreRunCoverageTransitionPairDetail::FullCoveragePairRelation;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t currentCoverageTransitionPublicationSequence = 0ULL;
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
    std::uint8_t earlierObservedPairRelationMask = 0U;
    PairRelation earlierSourcePairRelation = PairRelation::NONE;
    DiscoveryPairRelation discoveryPairRelation = DiscoveryPairRelation::NONE;
    FullCoveragePairRelation fullCoveragePairRelation = FullCoveragePairRelation::NONE;
    std::array<std::uint8_t, 2U> reserved{};

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        bool IsProvenRunCoverageTransitionPair() const noexcept
    {
        using namespace NCPathCoreRunCoverageTransitionPairDetail;
        if (publicationSequence == 0ULL || currentCoverageTransitionPublicationSequence == 0ULL ||
            currentCoveragePublicationSequence == 0ULL || coverageGeneration == 0ULL ||
            sourceRunPublicationSequence == 0ULL || continuityRunGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL || sourcePairPublicationSequence == 0ULL ||
            firstTransitionPublicationSequence == 0ULL || currentTransitionPublicationSequence == 0ULL ||
            firstBoundaryPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            consecutivePairCount < 4U || schemaVersion != NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_SCHEMA_V1 ||
            disposition != Disposition::PROVEN_DIRECT_RUN_COVERAGE_TRANSITION_PAIR_CONTINUITY ||
            sourcePairRelation == PairRelation::NONE ||
            sourcePairRelation != NCPathCoreBoundaryAvailabilityTransitionPairDetail::Classify(
                previousTransitionRelation, currentTransitionRelation) ||
            !NCPathCoreBoundaryAvailabilityTransitionPairDetail::CurrentReasonMatches(
                currentTransitionRelation, currentUnavailableReason)) return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;

        // Validate the complete retained current-X scalar projection in place.
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

        // The predecessor X's earlier mask/U are needed for its local proof.
        const std::uint8_t earlierBit = PairRelationBit(earlierSourcePairRelation);
        const std::uint8_t earlierRequired = static_cast<std::uint8_t>(firstBit | earlierBit);
        if (earlierBit == 0U || (earlierObservedPairRelationMask & earlierRequired) != earlierRequired ||
            PopulationCount(earlierObservedPairRelationMask) > consecutivePairCount - 2U ||
            previousObservedPairRelationMask != static_cast<std::uint8_t>(earlierObservedPairRelationMask | previousBit) ||
            PairCurrentTransition(earlierSourcePairRelation) != PairPreviousTransition(previousSourcePairRelation)) return false;
        // At count four, the earlier W was the actual two-U seed.
        if (consecutivePairCount == 4U && (earlierObservedPairRelationMask != earlierRequired ||
            PairCurrentTransition(firstPairRelation) != PairPreviousTransition(earlierSourcePairRelation))) return false;
        const std::uint8_t previousNew = static_cast<std::uint8_t>(previousObservedPairRelationMask &
            static_cast<std::uint8_t>(~earlierObservedPairRelationMask));
        const FullCoveragePairRelation expectedFull = ClassifyFullCoveragePair(
            ClassifyFullCoverage(earlierObservedPairRelationMask, previousObservedPairRelationMask), fullCoverageTransition);
        if (discoveryPairRelation != ClassifyDiscoveryPair(previousNew, newlyObservedPairRelationMask) ||
            expectedFull == FullCoveragePairRelation::NONE || fullCoveragePairRelation != expectedFull) return false;

        // X/Y may attach late, so neither publication is a run-head span.
        const std::uint64_t sourceDistance = static_cast<std::uint64_t>(consecutivePairCount) - 1ULL;
        return ForwardNonZeroSequenceDistance(coverageGeneration, currentCoveragePublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(continuityRunGeneration, sourceRunPublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, sourcePairPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstTransitionPublicationSequence, currentTransitionPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstBoundaryPublicationSequence, currentBoundaryPublicationSequence) == sourceDistance;
    }
};

class NCPathCoreRunCoverageTransitionPairShadow final
{
public:
    using Record = NCPathCoreRunCoverageTransitionPairRecordV1;
    using Disposition = Record::Disposition;
    using PairRelation = Record::PairRelation;
    using TransitionRelation = Record::TransitionRelation;
    using UnavailableReason = Record::UnavailableReason;
    using FullCoverageTransition = Record::FullCoverageTransition;
    using DiscoveryPairRelation = Record::DiscoveryPairRelation;
    using FullCoveragePairRelation = Record::FullCoveragePairRelation;
    using TransitionRecord = NCPathCoreRunCoverageTransitionPairDetail::TransitionRecord;
    using TransitionDisposition = TransitionRecord::Disposition;

    NCPathCoreRunCoverageTransitionPairShadow() noexcept = default;

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        void ObserveLatestRunCoverageTransitionPairSameThread(
            const TransitionRecord* previousTransition, const TransitionRecord* currentTransition) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenRunCoverageTransitionPair();
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            (previousProven || IsCanonicalNonProvenPair(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentCoverageTransitionPublicationSequence;
        m_publicationSequence = NCPathCoreRunCoverageTransitionPairDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentTransition == nullptr ? 0ULL : currentTransition->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        else if (currentTransition == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE;
        else if (anchor != 0ULL && currentTransition->publicationSequence !=
            NCPathCoreRunCoverageTransitionPairDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluatePair(previousTransition, *currentTransition, previous, previousProven, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_HISTORY_CAPACITY;

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static bool IsCanonicalNonProvenTransition(const TransitionRecord& source) noexcept
    {
        const bool known = source.disposition >= TransitionDisposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE &&
            source.disposition <= TransitionDisposition::INVALID_COVERAGE_TRANSITION;
        const bool identity = source.disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE
            ? source.currentCoveragePublicationSequence == 0ULL
            : source.currentCoveragePublicationSequence != 0ULL ||
            source.disposition == TransitionDisposition::INVALID_CURRENT_COVERAGE_RECORD ||
            source.disposition == TransitionDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == TransitionDisposition::INVALID_PREVIOUS_TRANSITION_RECORD;
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

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static bool IsNeutralTransitionDisposition(TransitionDisposition disposition) noexcept
    {
        return disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_COVERAGE_UNAVAILABLE ||
            disposition == TransitionDisposition::NOT_APPLICABLE_PREVIOUS_COVERAGE_UNAVAILABLE ||
            disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_COVERAGE_NOT_PROVEN ||
            disposition == TransitionDisposition::NOT_APPLICABLE_PREVIOUS_COVERAGE_NOT_PROVEN ||
            disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_COVERAGE_RUN_HEAD ||
            disposition == TransitionDisposition::NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static bool IsCanonicalNonProvenPair(const Record& source) noexcept
    {
        const bool known = source.disposition >= Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            source.disposition <= Disposition::INVALID_TRANSITION_PAIR;
        const bool identity = source.disposition == Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE
            ? source.currentCoverageTransitionPublicationSequence == 0ULL
            : source.currentCoverageTransitionPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_TRANSITION_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        if (!known || !identity || source.publicationSequence == 0ULL ||
            source.schemaVersion != NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_SCHEMA_V1 ||
            source.currentCoveragePublicationSequence != 0ULL || source.coverageGeneration != 0ULL ||
            source.sourceRunPublicationSequence != 0ULL || source.continuityRunGeneration != 0ULL ||
            source.firstPairPublicationSequence != 0ULL || source.sourcePairPublicationSequence != 0ULL ||
            source.firstTransitionPublicationSequence != 0ULL || source.currentTransitionPublicationSequence != 0ULL ||
            source.firstBoundaryPublicationSequence != 0ULL || source.currentBoundaryPublicationSequence != 0ULL ||
            source.consecutivePairCount != 0U || source.previousObservedPairRelationMask != 0U ||
            source.currentObservedPairRelationMask != 0U || source.newlyObservedPairRelationMask != 0U ||
            source.retainedPairRelationMask != 0U || source.fullCoverageTransition != FullCoverageTransition::NONE ||
            source.firstPairRelation != PairRelation::NONE || source.sourcePairRelation != PairRelation::NONE ||
            source.previousSourcePairRelation != PairRelation::NONE || source.previousTransitionRelation != TransitionRelation::NONE ||
            source.currentTransitionRelation != TransitionRelation::NONE || source.currentUnavailableReason != UnavailableReason::NONE ||
            source.earlierObservedPairRelationMask != 0U || source.earlierSourcePairRelation != PairRelation::NONE ||
            source.discoveryPairRelation != DiscoveryPairRelation::NONE ||
            source.fullCoveragePairRelation != FullCoveragePairRelation::NONE) return false;
        for (std::uint8_t value : source.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static bool PreviousTransitionMatchesPair(const TransitionRecord& source, const Record& previous) noexcept
    {
        return source.publicationSequence == previous.currentCoverageTransitionPublicationSequence &&
            source.currentCoveragePublicationSequence == previous.currentCoveragePublicationSequence &&
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
            source.previousObservedPairRelationMask == previous.previousObservedPairRelationMask &&
            source.currentObservedPairRelationMask == previous.currentObservedPairRelationMask &&
            source.newlyObservedPairRelationMask == previous.newlyObservedPairRelationMask &&
            source.retainedPairRelationMask == previous.retainedPairRelationMask &&
            source.fullCoverageTransition == previous.fullCoverageTransition &&
            source.firstPairRelation == previous.firstPairRelation && source.sourcePairRelation == previous.sourcePairRelation &&
            source.previousSourcePairRelation == previous.previousSourcePairRelation &&
            source.previousTransitionRelation == previous.previousTransitionRelation &&
            source.currentTransitionRelation == previous.currentTransitionRelation &&
            source.currentUnavailableReason == previous.currentUnavailableReason;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static bool SameCoverageRun(const TransitionRecord& previous, const TransitionRecord& current) noexcept
    {
        return current.coverageGeneration == previous.coverageGeneration &&
            current.continuityRunGeneration == previous.continuityRunGeneration &&
            current.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            current.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            current.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            current.firstPairRelation == previous.firstPairRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static bool DirectTransitionAdvance(const TransitionRecord& previous, const TransitionRecord& current) noexcept
    {
        using namespace NCPathCoreRunCoverageTransitionPairDetail;
        return previous.consecutivePairCount != (std::numeric_limits<std::uint32_t>::max)() &&
            current.consecutivePairCount == previous.consecutivePairCount + 1U &&
            current.publicationSequence == NextNonZeroSequence(previous.publicationSequence) &&
            current.currentCoveragePublicationSequence == NextNonZeroSequence(previous.currentCoveragePublicationSequence) &&
            current.sourceRunPublicationSequence == NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence == NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            current.currentTransitionPublicationSequence == NextNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            current.currentBoundaryPublicationSequence == NextNonZeroSequence(previous.currentBoundaryPublicationSequence);
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static void EvaluatePair(const TransitionRecord* previousTransition, const TransitionRecord& currentTransition,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (!currentTransition.IsProvenBoundaryAvailabilityTransitionPairRunCoverageTransition())
        {
            target.disposition = !IsCanonicalNonProvenTransition(currentTransition) ? Disposition::INVALID_CURRENT_TRANSITION_RECORD :
                IsNeutralTransitionDisposition(currentTransition.disposition) ? Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_CURRENT_TRANSITION_INVALID;
            return;
        }
        if (previousTransition == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE;
            return;
        }
        if (!previousTransition->IsProvenBoundaryAvailabilityTransitionPairRunCoverageTransition())
        {
            target.disposition = !IsCanonicalNonProvenTransition(*previousTransition) ? Disposition::INVALID_PREVIOUS_TRANSITION_RECORD :
                IsNeutralTransitionDisposition(previousTransition->disposition) ? Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_PREVIOUS_TRANSITION_INVALID;
            return;
        }
        if (previousProven && previous != nullptr && !PreviousTransitionMatchesPair(*previousTransition, *previous))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (!SameCoverageRun(*previousTransition, currentTransition))
        {
            target.disposition = Disposition::NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN;
            return;
        }
        if (!DirectTransitionAdvance(*previousTransition, currentTransition))
        {
            target.disposition = Disposition::INVALID_TRANSITION_ADVANCE;
            return;
        }
        if (currentTransition.previousObservedPairRelationMask != previousTransition->currentObservedPairRelationMask ||
            currentTransition.previousSourcePairRelation != previousTransition->sourcePairRelation ||
            currentTransition.previousTransitionRelation != previousTransition->currentTransitionRelation)
        {
            target.disposition = Disposition::INVALID_SHARED_COVERAGE_BINDING;
            return;
        }
        BuildPair(*previousTransition, currentTransition, target);
        if (!target.IsProvenRunCoverageTransitionPair())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_TRANSITION_PAIR;
        }
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static void BuildPair(const TransitionRecord& previous, const TransitionRecord& current, Record& target) noexcept
    {
        target.currentCoveragePublicationSequence = current.currentCoveragePublicationSequence;
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
        target.previousObservedPairRelationMask = current.previousObservedPairRelationMask;
        target.currentObservedPairRelationMask = current.currentObservedPairRelationMask;
        target.newlyObservedPairRelationMask = current.newlyObservedPairRelationMask;
        target.retainedPairRelationMask = current.retainedPairRelationMask;
        target.fullCoverageTransition = current.fullCoverageTransition;
        target.firstPairRelation = current.firstPairRelation;
        target.sourcePairRelation = current.sourcePairRelation;
        target.previousSourcePairRelation = current.previousSourcePairRelation;
        target.previousTransitionRelation = current.previousTransitionRelation;
        target.currentTransitionRelation = current.currentTransitionRelation;
        target.currentUnavailableReason = current.currentUnavailableReason;
        target.earlierObservedPairRelationMask = previous.previousObservedPairRelationMask;
        target.earlierSourcePairRelation = previous.previousSourcePairRelation;
        target.discoveryPairRelation = NCPathCoreRunCoverageTransitionPairDetail::ClassifyDiscoveryPair(
            previous.newlyObservedPairRelationMask, current.newlyObservedPairRelationMask);
        target.fullCoveragePairRelation = NCPathCoreRunCoverageTransitionPairDetail::ClassifyFullCoveragePair(
            previous.fullCoverageTransition, current.fullCoverageTransition);
        target.disposition = Disposition::PROVEN_DIRECT_RUN_COVERAGE_TRANSITION_PAIR_CONTINUITY;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        target.currentCoveragePublicationSequence = 0ULL;
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
        target.earlierObservedPairRelationMask = 0U;
        target.earlierSourcePairRelation = PairRelation::NONE;
        target.discoveryPairRelation = DiscoveryPairRelation::NONE;
        target.fullCoveragePairRelation = FullCoveragePairRelation::NONE;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentCoverageTransitionPublicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_NOINLINE

static_assert(sizeof(NCPathCoreRunCoverageTransitionPairDisposition) == 1U, "Y disposition one byte.");
static_assert(sizeof(NCPathCoreRunCoverageTransitionPairDiscoveryPairRelation) == 1U, "Y discovery pair one byte.");
static_assert(sizeof(NCPathCoreRunCoverageTransitionPairFullCoveragePairRelation) == 1U, "Y full coverage pair one byte.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageTransitionPairRecordV1>::value, "Y record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageTransitionPairRecordV1>::value, "Y record scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageTransitionPairRecordV1) == 120U, "Y record exactly 120 bytes.");
static_assert(alignof(NCPathCoreRunCoverageTransitionPairRecordV1) == 8U, "Y record alignment.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, consecutivePairCount) == 96U, "Y count offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, schemaVersion) == 100U, "Y schema offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, disposition) == 102U, "Y disposition offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, previousObservedPairRelationMask) == 103U, "Y previous mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, currentObservedPairRelationMask) == 104U, "Y current mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, newlyObservedPairRelationMask) == 105U, "Y newly mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, retainedPairRelationMask) == 106U, "Y retained mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, fullCoverageTransition) == 107U, "Y full coverage offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, firstPairRelation) == 108U, "Y first relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, sourcePairRelation) == 109U, "Y current relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, previousSourcePairRelation) == 110U, "Y previous relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, previousTransitionRelation) == 111U, "Y previous transition offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, currentTransitionRelation) == 112U, "Y current transition offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, currentUnavailableReason) == 113U, "Y reason offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, earlierObservedPairRelationMask) == 114U, "Y earlier mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, earlierSourcePairRelation) == 115U, "Y earlier relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, discoveryPairRelation) == 116U, "Y discovery pair offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, fullCoveragePairRelation) == 117U, "Y full coverage pair offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRecordV1, reserved) == 118U, "Y reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageTransitionPairShadow>::value, "Y observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageTransitionPairShadow>::value, "Y observer scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageTransitionPairShadow) == 256U, "Y observer exactly 256 bytes.");
static_assert(alignof(NCPathCoreRunCoverageTransitionPairShadow) == 8U, "Y observer alignment.");
