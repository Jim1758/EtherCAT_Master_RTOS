#pragma once

#include "NCPathCoreBoundaryAvailabilityTransitionPairRunShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2W / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Pair Continuity Run Pattern Coverage Shadow.
//
// Summarizes the presence of the eight U pair relations only inside a V
// local continuity run whose actual two-pair head was observed here. Each
// bit is (1 << (U relation - 1)); no count or order of individual relations
// is retained. A V extension cannot seed coverage of an unobserved head.
// Null, neutral, invalid, malformed, stale or mismatched input clears the
// proven payload. Only a later observed V head can re-establish coverage;
// a suffix never pretends to summarize the original head. Rejections keep
// at most the supplied V publication as a diagnostic freshness anchor.
//
// Both U records pass their complete proof, directly overlap, and bind the
// latest U to every semantic field projected by V. Extension also binds
// the previous U to the retained latest-U projection. Five zero-skipping
// spans bind W, V, U, current T and current S identities to the pair count.
// The first T/S identities are the current references of the first U;
// they are not geometry endpoints. Before overwriting either slot, the
// latest extended W summary must obey the exact mask-union recurrence
// against its retained predecessor. This detects a changed latest mask
// even when its standalone membership/count checks still pass.
//
// Availability gains, losses and source-reported coverage boundaries may
// occur inside this certificate run. A mask of 0xff means only that all
// eight local U relation patterns were observed; it is not path, closure,
// all-axes qualification, stable availability or Motion execution proof.
// U omits its previous T unavailable reason; that shared reason cannot be
// independently compared. Trusted same-thread producers are required.
// Coherent multi-record forgery, full-wrap identity reuse, wall-clock
// freshness and authentication of discarded history are not claimed.
//
// Exactly two 96-byte scalar records, 208-byte NCManager heap-owned observer.
// No Observe-time allocation, source/stack snapshot, coordinates, displacement,
// endpoint arrays, nodes, segment/event list, full order, reverse traversal
// or unbounded history. Not a Path/Motion Queue, planner, STARTED/DONE proof,
// actual-position history, B2 breadcrumb or B2 execution. Shadow-only; no
// control consumer. No Motion, G00, Gate/Registry, Alarm, PC, HMI/SHM/API or
// PDO/DC changes; no log, thread, timer, mutex, wait or sleep. Existing M00
// FIX1 and existing filenames remain unchanged.

constexpr std::size_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_SCHEMA_V1 = 1U;

enum class NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_RUN_UNAVAILABLE = 1U,
    NOT_APPLICABLE_RUN_NOT_PROVEN = 2U,
    SOURCE_REPORTED_RUN_INVALID = 3U,
    NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED = 4U,
    NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE = 5U,
    NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE = 6U,
    INVALID_CURRENT_RUN_RECORD = 7U,
    INVALID_PREVIOUS_PAIR_RECORD = 8U,
    INVALID_CURRENT_PAIR_RECORD = 9U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 10U,
    INVALID_PREVIOUS_COVERAGE_RECORD = 11U,
    INVALID_RUN_CONTINUITY_BINDING = 12U,
    INVALID_PAIR_RUN_BINDING = 13U,
    INVALID_PAIR_OVERLAP = 14U,
    INVALID_PREVIOUS_PAIR_BINDING = 15U,
    INVALID_COVERAGE_SUMMARY = 16U,
    PROVEN_RUN_PATTERN_COVERAGE_START = 17U,
    PROVEN_RUN_PATTERN_COVERAGE_EXTENSION = 18U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
#endif

namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail
{
    using RunRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1;
    using PairRecord = NCPathCoreBoundaryAvailabilityTransitionPairRecordV1;
    using PairRelation = PairRecord::Relation;
    using TransitionRelation = PairRecord::TransitionRelation;
    using Reason = PairRecord::UnavailableReason;

    constexpr std::uint64_t NextNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)() ? 1ULL : value + 1ULL;
    }

    constexpr std::uint64_t PreviousNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == 1ULL ? (std::numeric_limits<std::uint64_t>::max)() : value - 1ULL;
    }

    constexpr std::uint64_t ForwardNonZeroSequenceDistance(std::uint64_t first, std::uint64_t current) noexcept
    {
        return current >= first ? current - first :
            (std::numeric_limits<std::uint64_t>::max)() - first + current;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        inline std::uint8_t PairRelationBit(PairRelation relation) noexcept
    {
        const unsigned int value = static_cast<unsigned int>(relation);
        return value >= 1U && value <= 8U ? static_cast<std::uint8_t>(1U << (value - 1U)) :
            static_cast<std::uint8_t>(0U);
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        inline std::uint32_t PopulationCount(std::uint8_t mask) noexcept
    {
        std::uint32_t count = 0U;
        unsigned int value = static_cast<unsigned int>(mask);
        while (value != 0U)
        {
            count += value & 1U;
            value >>= 1U;
        }
        return count;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        inline TransitionRelation PairCurrentTransition(PairRelation relation) noexcept
    {
        if (relation == PairRelation::GAIN_THEN_RETAIN || relation == PairRelation::RETAIN_THEN_RETAIN)
            return TransitionRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION;
        if (relation == PairRelation::GAIN_THEN_LOSS || relation == PairRelation::RETAIN_THEN_LOSS)
            return TransitionRelation::BECAME_UNAVAILABLE;
        if (relation == PairRelation::LOSS_THEN_GAIN || relation == PairRelation::UNAVAILABLE_THEN_GAIN)
            return TransitionRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD;
        if (relation == PairRelation::LOSS_THEN_UNAVAILABLE || relation == PairRelation::UNAVAILABLE_THEN_UNAVAILABLE)
            return TransitionRelation::REMAINED_UNAVAILABLE;
        return TransitionRelation::NONE;
    }
}

struct NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1
{
    using Disposition = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDisposition;
    using PairRelation = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::PairRelation;
    using TransitionRelation = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::TransitionRelation;
    using UnavailableReason = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::Reason;

    std::uint64_t publicationSequence = 0ULL;
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
    std::uint8_t observedPairRelationMask = 0U;
    PairRelation firstPairRelation = PairRelation::NONE;
    PairRelation sourcePairRelation = PairRelation::NONE;
    TransitionRelation previousTransitionRelation = TransitionRelation::NONE;
    TransitionRelation currentTransitionRelation = TransitionRelation::NONE;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::array<std::uint8_t, 3U> reserved{};

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        bool IsProvenBoundaryAvailabilityTransitionPairRunCoverage() const noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail;
        if (publicationSequence == 0ULL || coverageGeneration == 0ULL || sourceRunPublicationSequence == 0ULL ||
            continuityRunGeneration == 0ULL || firstPairPublicationSequence == 0ULL || sourcePairPublicationSequence == 0ULL ||
            firstTransitionPublicationSequence == 0ULL || currentTransitionPublicationSequence == 0ULL ||
            firstBoundaryPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            schemaVersion != NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_SCHEMA_V1 ||
            consecutivePairCount < 2U ||
            !((disposition == Disposition::PROVEN_RUN_PATTERN_COVERAGE_START && consecutivePairCount == 2U) ||
                (disposition == Disposition::PROVEN_RUN_PATTERN_COVERAGE_EXTENSION && consecutivePairCount > 2U)) ||
            sourcePairRelation == PairRelation::NONE ||
            sourcePairRelation != NCPathCoreBoundaryAvailabilityTransitionPairDetail::Classify(
                previousTransitionRelation, currentTransitionRelation) ||
            !NCPathCoreBoundaryAvailabilityTransitionPairDetail::CurrentReasonMatches(
                currentTransitionRelation, currentUnavailableReason)) return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        const std::uint8_t firstBit = PairRelationBit(firstPairRelation);
        const std::uint8_t latestBit = PairRelationBit(sourcePairRelation);
        const std::uint8_t required = static_cast<std::uint8_t>(firstBit | latestBit);
        if (firstBit == 0U || latestBit == 0U || (observedPairRelationMask & required) != required ||
            PopulationCount(observedPairRelationMask) > consecutivePairCount) return false;
        if (consecutivePairCount == 2U && (observedPairRelationMask != required ||
            PairCurrentTransition(firstPairRelation) != previousTransitionRelation)) return false;
        const std::uint64_t sourceDistance = static_cast<std::uint64_t>(consecutivePairCount) - 1ULL;
        return ForwardNonZeroSequenceDistance(coverageGeneration, publicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(continuityRunGeneration, sourceRunPublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, sourcePairPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstTransitionPublicationSequence, currentTransitionPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstBoundaryPublicationSequence, currentBoundaryPublicationSequence) == sourceDistance;
    }
};

class NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow final
{
public:
    using Record = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1;
    using Disposition = Record::Disposition;
    using PairRelation = Record::PairRelation;
    using TransitionRelation = Record::TransitionRelation;
    using UnavailableReason = Record::UnavailableReason;
    using RunRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::RunRecord;
    using RunDisposition = RunRecord::Disposition;
    using PairRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::PairRecord;

    NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow() noexcept = default;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        void ObserveLatestBoundaryAvailabilityTransitionPairRunSameThread(
            const RunRecord* run, const PairRecord* previousPair, const PairRecord* currentPair) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenBoundaryAvailabilityTransitionPairRunCoverage();
        // This must precede ResetTarget: the older retained record occupies
        // the slot about to be overwritten and validates local recurrence.
        const bool previousValid = IsRetainedHistoryValid(previous, previousProven,
            GetNewestObservationSameThread(1U), m_publicationSequence);
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->sourceRunPublicationSequence;
        m_publicationSequence = NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, run == nullptr ? 0ULL : run->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_COVERAGE_RECORD;
        else if (run == nullptr) target.disposition = Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE;
        else if (anchor != 0ULL && run->publicationSequence !=
            NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluateCoverage(*run, previousPair, currentPair, previous, previousProven, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_HISTORY_CAPACITY;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool IsCanonicalNonProvenRun(const RunRecord& source) noexcept
    {
        const bool known = source.disposition >= RunDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            source.disposition <= RunDisposition::INVALID_CONTINUITY_RUN;
        const bool identity = source.disposition == RunDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE
            ? source.sourcePairPublicationSequence == 0ULL
            : source.sourcePairPublicationSequence != 0ULL ||
            source.disposition == RunDisposition::INVALID_CURRENT_PAIR_RECORD ||
            source.disposition == RunDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == RunDisposition::INVALID_PREVIOUS_RUN_RECORD;
        if (!known || !identity || source.publicationSequence == 0ULL ||
            source.schemaVersion != NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_SCHEMA_V1 ||
            source.continuityRunGeneration != 0ULL || source.firstPairPublicationSequence != 0ULL ||
            source.firstTransitionPublicationSequence != 0ULL || source.currentTransitionPublicationSequence != 0ULL ||
            source.firstBoundaryPublicationSequence != 0ULL || source.currentBoundaryPublicationSequence != 0ULL ||
            source.consecutivePairCount != 0U || source.sourcePairRelation != PairRelation::NONE ||
            source.previousTransitionRelation != TransitionRelation::NONE || source.currentTransitionRelation != TransitionRelation::NONE ||
            source.currentUnavailableReason != UnavailableReason::NONE) return false;
        for (std::uint8_t value : source.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool IsCanonicalNonProvenCoverage(const Record& source) noexcept
    {
        const bool known = source.disposition >= Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE &&
            source.disposition <= Disposition::INVALID_COVERAGE_SUMMARY;
        const bool identity = source.disposition == Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE
            ? source.sourceRunPublicationSequence == 0ULL
            : source.sourceRunPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_RUN_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_COVERAGE_RECORD;
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

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool IsRetainedHistoryValid(const Record* newest, bool newestProven,
            const Record* older, std::uint64_t publication) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail;
        if (newest == nullptr) return true;
        if (newest->publicationSequence != publication) return false;
        if (!newestProven) return IsCanonicalNonProvenCoverage(*newest);
        if (newest->disposition == Disposition::PROVEN_RUN_PATTERN_COVERAGE_START) return true;
        if (older == nullptr || !older->IsProvenBoundaryAvailabilityTransitionPairRunCoverage() ||
            older->consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)()) return false;
        return newest->publicationSequence == NextNonZeroSequence(older->publicationSequence) &&
            newest->sourceRunPublicationSequence == NextNonZeroSequence(older->sourceRunPublicationSequence) &&
            newest->sourcePairPublicationSequence == NextNonZeroSequence(older->sourcePairPublicationSequence) &&
            newest->currentTransitionPublicationSequence == NextNonZeroSequence(older->currentTransitionPublicationSequence) &&
            newest->currentBoundaryPublicationSequence == NextNonZeroSequence(older->currentBoundaryPublicationSequence) &&
            newest->consecutivePairCount == older->consecutivePairCount + 1U &&
            newest->coverageGeneration == older->coverageGeneration &&
            newest->continuityRunGeneration == older->continuityRunGeneration &&
            newest->firstPairPublicationSequence == older->firstPairPublicationSequence &&
            newest->firstTransitionPublicationSequence == older->firstTransitionPublicationSequence &&
            newest->firstBoundaryPublicationSequence == older->firstBoundaryPublicationSequence &&
            newest->firstPairRelation == older->firstPairRelation &&
            newest->previousTransitionRelation == older->currentTransitionRelation &&
            newest->observedPairRelationMask == static_cast<std::uint8_t>(
                older->observedPairRelationMask | PairRelationBit(newest->sourcePairRelation));
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool PairOverlapMatches(const PairRecord& previous, const PairRecord& current) noexcept
    {
        return current.publicationSequence ==
            NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail::NextNonZeroSequence(previous.publicationSequence) &&
            previous.currentTransitionPublicationSequence == current.previousTransitionPublicationSequence &&
            previous.currentBoundaryPublicationSequence == current.sharedBoundaryPublicationSequence &&
            previous.currentTransitionRelation == current.previousTransitionRelation;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool CurrentPairMatchesRun(const PairRecord& source, const RunRecord& run) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail;
        return source.publicationSequence == run.sourcePairPublicationSequence &&
            source.previousTransitionPublicationSequence == PreviousNonZeroSequence(run.currentTransitionPublicationSequence) &&
            source.currentTransitionPublicationSequence == run.currentTransitionPublicationSequence &&
            source.sharedBoundaryPublicationSequence == PreviousNonZeroSequence(run.currentBoundaryPublicationSequence) &&
            source.currentBoundaryPublicationSequence == run.currentBoundaryPublicationSequence &&
            source.relation == run.sourcePairRelation && source.previousTransitionRelation == run.previousTransitionRelation &&
            source.currentTransitionRelation == run.currentTransitionRelation && source.currentUnavailableReason == run.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool PreviousPairMatchesCoverage(const PairRecord& source, const Record& previous) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail;
        return source.publicationSequence == previous.sourcePairPublicationSequence &&
            source.previousTransitionPublicationSequence == PreviousNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            source.currentTransitionPublicationSequence == previous.currentTransitionPublicationSequence &&
            source.sharedBoundaryPublicationSequence == PreviousNonZeroSequence(previous.currentBoundaryPublicationSequence) &&
            source.currentBoundaryPublicationSequence == previous.currentBoundaryPublicationSequence &&
            source.relation == previous.sourcePairRelation && source.previousTransitionRelation == previous.previousTransitionRelation &&
            source.currentTransitionRelation == previous.currentTransitionRelation &&
            source.currentUnavailableReason == previous.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static bool RunExtensionMatchesCoverage(const RunRecord& run, const Record& previous) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail;
        return previous.consecutivePairCount != (std::numeric_limits<std::uint32_t>::max)() &&
            run.consecutivePairCount == previous.consecutivePairCount + 1U &&
            run.publicationSequence == NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            run.sourcePairPublicationSequence == NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            run.currentTransitionPublicationSequence == NextNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            run.currentBoundaryPublicationSequence == NextNonZeroSequence(previous.currentBoundaryPublicationSequence) &&
            run.continuityRunGeneration == previous.continuityRunGeneration &&
            run.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            run.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            run.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static void EvaluateCoverage(const RunRecord& run, const PairRecord* previousPair,
            const PairRecord* currentPair, const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (!run.IsProvenBoundaryAvailabilityTransitionPairRun())
        {
            target.disposition = !IsCanonicalNonProvenRun(run) ? Disposition::INVALID_CURRENT_RUN_RECORD :
                run.disposition <= RunDisposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN ?
                Disposition::NOT_APPLICABLE_RUN_NOT_PROVEN : Disposition::SOURCE_REPORTED_RUN_INVALID;
            return;
        }
        if (currentPair == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
            return;
        }
        if (!currentPair->IsProvenBoundaryAvailabilityTransitionPairContinuity())
        {
            target.disposition = Disposition::INVALID_CURRENT_PAIR_RECORD;
            return;
        }
        if (previousPair == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE;
            return;
        }
        if (!previousPair->IsProvenBoundaryAvailabilityTransitionPairContinuity())
        {
            target.disposition = Disposition::INVALID_PREVIOUS_PAIR_RECORD;
            return;
        }
        if (!PairOverlapMatches(*previousPair, *currentPair))
        {
            target.disposition = Disposition::INVALID_PAIR_OVERLAP;
            return;
        }
        if (!CurrentPairMatchesRun(*currentPair, run))
        {
            target.disposition = Disposition::INVALID_PAIR_RUN_BINDING;
            return;
        }
        if (run.disposition == RunDisposition::PROVEN_LOCAL_PAIR_RUN_START)
        {
            if (previousProven)
            {
                target.disposition = Disposition::INVALID_RUN_CONTINUITY_BINDING;
                return;
            }
            if (previousPair->publicationSequence != run.firstPairPublicationSequence ||
                previousPair->currentTransitionPublicationSequence != run.firstTransitionPublicationSequence ||
                previousPair->currentBoundaryPublicationSequence != run.firstBoundaryPublicationSequence)
            {
                target.disposition = Disposition::INVALID_PAIR_RUN_BINDING;
                return;
            }
        }
        else
        {
            if (!previousProven || previous == nullptr)
            {
                target.disposition = Disposition::NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED;
                return;
            }
            if (!RunExtensionMatchesCoverage(run, *previous))
            {
                target.disposition = Disposition::INVALID_RUN_CONTINUITY_BINDING;
                return;
            }
            if (!PreviousPairMatchesCoverage(*previousPair, *previous))
            {
                target.disposition = Disposition::INVALID_PREVIOUS_PAIR_BINDING;
                return;
            }
        }
        BuildCoverage(run, *previousPair, previous, previousProven, target);
        if (!target.IsProvenBoundaryAvailabilityTransitionPairRunCoverage())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_COVERAGE_SUMMARY;
        }
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static void BuildCoverage(const RunRecord& run, const PairRecord& previousPair,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDetail;
        if (previousProven && previous != nullptr)
        {
            target.coverageGeneration = previous->coverageGeneration;
            target.firstPairRelation = previous->firstPairRelation;
            target.observedPairRelationMask = static_cast<std::uint8_t>(
                previous->observedPairRelationMask | PairRelationBit(run.sourcePairRelation));
            target.disposition = Disposition::PROVEN_RUN_PATTERN_COVERAGE_EXTENSION;
        }
        else
        {
            target.coverageGeneration = target.publicationSequence;
            target.firstPairRelation = previousPair.relation;
            target.observedPairRelationMask = static_cast<std::uint8_t>(
                PairRelationBit(previousPair.relation) | PairRelationBit(run.sourcePairRelation));
            target.disposition = Disposition::PROVEN_RUN_PATTERN_COVERAGE_START;
        }
        target.continuityRunGeneration = run.continuityRunGeneration;
        target.firstPairPublicationSequence = run.firstPairPublicationSequence;
        target.sourcePairPublicationSequence = run.sourcePairPublicationSequence;
        target.firstTransitionPublicationSequence = run.firstTransitionPublicationSequence;
        target.currentTransitionPublicationSequence = run.currentTransitionPublicationSequence;
        target.firstBoundaryPublicationSequence = run.firstBoundaryPublicationSequence;
        target.currentBoundaryPublicationSequence = run.currentBoundaryPublicationSequence;
        target.consecutivePairCount = run.consecutivePairCount;
        target.sourcePairRelation = run.sourcePairRelation;
        target.previousTransitionRelation = run.previousTransitionRelation;
        target.currentTransitionRelation = run.currentTransitionRelation;
        target.currentUnavailableReason = run.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        target.coverageGeneration = 0ULL;
        target.continuityRunGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.sourcePairPublicationSequence = 0ULL;
        target.firstTransitionPublicationSequence = 0ULL;
        target.currentTransitionPublicationSequence = 0ULL;
        target.firstBoundaryPublicationSequence = 0ULL;
        target.currentBoundaryPublicationSequence = 0ULL;
        target.consecutivePairCount = 0U;
        target.observedPairRelationMask = 0U;
        target.firstPairRelation = PairRelation::NONE;
        target.sourcePairRelation = PairRelation::NONE;
        target.previousTransitionRelation = TransitionRelation::NONE;
        target.currentTransitionRelation = TransitionRelation::NONE;
        target.currentUnavailableReason = UnavailableReason::NONE;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.sourceRunPublicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_COVERAGE_NOINLINE

static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageDisposition) == 1U, "W disposition must be one byte.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1>::value, "W record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1>::value, "W record scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1) == 96U, "W record exactly 96 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1) == 8U, "W record alignment.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, consecutivePairCount) == 80U, "W count offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, schemaVersion) == 84U, "W schema offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, disposition) == 86U, "W disposition offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, observedPairRelationMask) == 87U, "W mask offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, firstPairRelation) == 88U, "W first relation offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, sourcePairRelation) == 89U, "W latest relation offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, previousTransitionRelation) == 90U, "W previous transition offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, currentTransitionRelation) == 91U, "W current transition offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, currentUnavailableReason) == 92U, "W reason offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageRecordV1, reserved) == 93U, "W reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow>::value, "W observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow>::value, "W observer scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow) == 208U, "W observer exactly 208 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow) == 8U, "W observer alignment.");
