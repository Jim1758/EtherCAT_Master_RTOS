#pragma once

#include "NCPathCoreRunCoverageTransitionPairShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2Z / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Pair Continuity Run Pattern Coverage Transition
// Pair Continuity Run-Length Shadow.
//
// Counts consecutive, directly overlapping Y pair certificates observed
// here. Two fully proven adjacent Y records start a local count of two;
// recovery may start a new proven suffix within the same W/V coverage run.
// It does not claim observation of the original W/V run head. An extension
// binds the supplied previous Y to every retained latest-Y semantic field.
// Seven direct source advances (Y/X/W/V/U/T/S), source count, common heads,
// two shared masks, two shared U relations and the shared T relation bind.
//
// The local Z publication and Y certificate head spans bind the local count.
// Five inherited W/V/U/T/S spans validate the latest Y source proof; they
// are not seven separately retained local head spans. The pinned first Y's
// lower U count binds the lower count delta to the local count. Before slot
// overwrite, a retained extension must satisfy the two-record recurrence.
// These checks use the fixed two slots, without a Y/X/W stack snapshot.
//
// Y omits its earlier X's current unavailable reason; X also omits its
// preceding W reason. The shared omitted reasons cannot independently be
// compared across supplied Y records, and are not reconstructed. The
// available latest reason is validated and bound on each local extension.
// Trusted same-thread producers are required. Structurally coherent altered
// accumulated masks may pass local fences; discarded history, coherent
// forgery, full-wrap identity reuse and wall-clock freshness are not proven.
//
// Null, neutral, invalid, malformed, stale, gap, changed run, overflow or
// binding failure clears every proof field and count. At most current Y
// publication remains as a diagnostic anchor; null clears it. A subsequent
// complete direct pair starts a new local suffix, never bridges rejection.
// Count does not describe segments, duration, pattern frequency, stable
// availability, one underlying coverage interval, closure, all-axis
// qualification, complete path history, or Motion STARTED/DONE.
//
// Exactly two 152-byte scalar records; 320-byte NCManager heap-owned
// observer. No Observe-time allocation, coordinates, displacement, endpoint
// arrays, nodes, segment/event list, complete order or reverse traversal.
// Not a Path/Motion Queue, planner, actual-position history, B2 breadcrumb
// or B2 execution. Shadow-only; no control consumer. No Motion, G00,
// Gate/Registry, Alarm, PC, HMI/SHM/API or PDO/DC changes; no log, thread,
// timer, mutex, wait or sleep. Existing M00 FIX1 and filenames are preserved.

constexpr std::size_t NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_SCHEMA_V1 = 1U;

enum class NCPathCoreRunCoverageTransitionPairRunDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE = 2U,
    NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN = 3U,
    NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN = 4U,
    NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN = 5U,
    SOURCE_REPORTED_CURRENT_PAIR_INVALID = 6U,
    SOURCE_REPORTED_PREVIOUS_PAIR_INVALID = 7U,
    INVALID_CURRENT_PAIR_RECORD = 8U,
    INVALID_PREVIOUS_PAIR_RECORD = 9U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 10U,
    INVALID_PREVIOUS_RUN_RECORD = 11U,
    INVALID_PREVIOUS_SOURCE_BINDING = 12U,
    INVALID_PAIR_ADVANCE = 13U,
    INVALID_SHARED_PAIR_BINDING = 14U,
    INVALID_PAIR_COUNT_OVERFLOW = 15U,
    INVALID_CONTINUITY_RUN = 16U,
    PROVEN_LOCAL_PAIR_RUN_START = 17U,
    PROVEN_LOCAL_PAIR_RUN_EXTENSION = 18U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
#endif

namespace NCPathCoreRunCoverageTransitionPairRunDetail
{
    using PairRecord = NCPathCoreRunCoverageTransitionPairRecordV1;
    using NCPathCoreRunCoverageTransitionPairDetail::NextNonZeroSequence;
    using NCPathCoreRunCoverageTransitionPairDetail::ForwardNonZeroSequenceDistance;
    using NCPathCoreRunCoverageTransitionPairDetail::PairRelationBit;
    using NCPathCoreRunCoverageTransitionPairDetail::PopulationCount;
    using NCPathCoreRunCoverageTransitionPairDetail::PairCurrentTransition;
    using NCPathCoreRunCoverageTransitionPairDetail::PairPreviousTransition;
    using NCPathCoreRunCoverageTransitionPairDetail::ClassifyFullCoverage;
    using NCPathCoreRunCoverageTransitionPairDetail::ClassifyDiscoveryPair;
    using NCPathCoreRunCoverageTransitionPairDetail::ClassifyFullCoveragePair;
}

struct NCPathCoreRunCoverageTransitionPairRunRecordV1
{
    using Disposition = NCPathCoreRunCoverageTransitionPairRunDisposition;
    using SourceRecord = NCPathCoreRunCoverageTransitionPairRunDetail::PairRecord;
    using PairRelation = SourceRecord::PairRelation;
    using TransitionRelation = SourceRecord::TransitionRelation;
    using UnavailableReason = SourceRecord::UnavailableReason;
    using FullCoverageTransition = SourceRecord::FullCoverageTransition;
    using DiscoveryPairRelation = SourceRecord::DiscoveryPairRelation;
    using FullCoveragePairRelation = SourceRecord::FullCoveragePairRelation;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t firstCertificatePublicationSequence = 0ULL;
    std::uint64_t currentCertificatePublicationSequence = 0ULL;
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
    std::uint32_t consecutiveCertificateCount = 0U;
    std::uint32_t firstSourcePairCount = 0U;
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

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        bool IsProvenRunCoverageTransitionPairRun() const noexcept
    {
        using namespace NCPathCoreRunCoverageTransitionPairRunDetail;
        if (publicationSequence == 0ULL || runGeneration == 0ULL ||
            firstCertificatePublicationSequence == 0ULL || currentCertificatePublicationSequence == 0ULL ||
            currentCoverageTransitionPublicationSequence == 0ULL ||
            currentCoveragePublicationSequence == 0ULL || coverageGeneration == 0ULL ||
            sourceRunPublicationSequence == 0ULL || continuityRunGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL || sourcePairPublicationSequence == 0ULL ||
            firstTransitionPublicationSequence == 0ULL || currentTransitionPublicationSequence == 0ULL ||
            firstBoundaryPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            firstSourcePairCount < 4U || consecutivePairCount < firstSourcePairCount ||
            consecutiveCertificateCount < 2U ||
            consecutiveCertificateCount >(std::numeric_limits<std::uint32_t>::max)() - 3U ||
            consecutivePairCount - firstSourcePairCount != consecutiveCertificateCount - 1U || schemaVersion != NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_SCHEMA_V1 ||
            !((disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_START && consecutiveCertificateCount == 2U) ||
                (disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION && consecutiveCertificateCount > 2U)) ||
            sourcePairRelation == PairRelation::NONE ||
            sourcePairRelation != NCPathCoreBoundaryAvailabilityTransitionPairDetail::Classify(
                previousTransitionRelation, currentTransitionRelation) ||
            !NCPathCoreBoundaryAvailabilityTransitionPairDetail::CurrentReasonMatches(
                currentTransitionRelation, currentUnavailableReason)) return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;

        // Validate the complete retained latest-Y semantic projection in place.
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
        // Latest Y here has count at least five. A count-four first Y's
        // earlier W seed equation is checked on the actual supplied Y at START.
        const std::uint8_t previousNew = static_cast<std::uint8_t>(previousObservedPairRelationMask &
            static_cast<std::uint8_t>(~earlierObservedPairRelationMask));
        const FullCoveragePairRelation expectedFull = ClassifyFullCoveragePair(
            ClassifyFullCoverage(earlierObservedPairRelationMask, previousObservedPairRelationMask), fullCoverageTransition);
        if (discoveryPairRelation != ClassifyDiscoveryPair(previousNew, newlyObservedPairRelationMask) ||
            expectedFull == FullCoveragePairRelation::NONE || fullCoveragePairRelation != expectedFull) return false;

        // The Z/Y spans describe only this observed local certificate run.
        const std::uint64_t localDistance = static_cast<std::uint64_t>(consecutiveCertificateCount) - 1ULL;
        if (ForwardNonZeroSequenceDistance(runGeneration, publicationSequence) != localDistance - 1ULL ||
            ForwardNonZeroSequenceDistance(firstCertificatePublicationSequence, currentCertificatePublicationSequence) !=
            localDistance) return false;
        // These five spans retain the complete latest-Y lower-level proof.
        const std::uint64_t sourceDistance = static_cast<std::uint64_t>(consecutivePairCount) - 1ULL;
        return ForwardNonZeroSequenceDistance(coverageGeneration, currentCoveragePublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(continuityRunGeneration, sourceRunPublicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, sourcePairPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstTransitionPublicationSequence, currentTransitionPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstBoundaryPublicationSequence, currentBoundaryPublicationSequence) == sourceDistance;
    }
};

class NCPathCoreRunCoverageTransitionPairRunShadow final
{
public:
    using Record = NCPathCoreRunCoverageTransitionPairRunRecordV1;
    using Disposition = Record::Disposition;
    using PairRelation = Record::PairRelation;
    using TransitionRelation = Record::TransitionRelation;
    using UnavailableReason = Record::UnavailableReason;
    using FullCoverageTransition = Record::FullCoverageTransition;
    using DiscoveryPairRelation = Record::DiscoveryPairRelation;
    using FullCoveragePairRelation = Record::FullCoveragePairRelation;
    using PairRecord = NCPathCoreRunCoverageTransitionPairRunDetail::PairRecord;
    using PairDisposition = PairRecord::Disposition;

    NCPathCoreRunCoverageTransitionPairRunShadow() noexcept = default;

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        void ObserveLatestRunCoverageTransitionPairRunSameThread(
            const PairRecord* previousPair, const PairRecord* currentPair) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenRunCoverageTransitionPairRun();
        // Validate both retained slots before resetting the older target slot.
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            ((previousProven && (previous->disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_START ||
                IsRetainedRunRecurrence(GetNewestObservationSameThread(1U), *previous))) ||
                IsCanonicalNonProvenRun(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentCertificatePublicationSequence;
        m_publicationSequence = NCPathCoreRunCoverageTransitionPairRunDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentPair == nullptr ? 0ULL : currentPair->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_RUN_RECORD;
        else if (currentPair == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
        else if (anchor != 0ULL && currentPair->publicationSequence !=
            NCPathCoreRunCoverageTransitionPairRunDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluateRun(previousPair, *currentPair, previous, previousProven, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_HISTORY_CAPACITY;

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsCanonicalNonProvenPair(const PairRecord& source) noexcept
    {
        const bool known = source.disposition >= PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            source.disposition <= PairDisposition::INVALID_TRANSITION_PAIR;
        const bool identity = source.disposition == PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE
            ? source.currentCoverageTransitionPublicationSequence == 0ULL
            : source.currentCoverageTransitionPublicationSequence != 0ULL ||
            source.disposition == PairDisposition::INVALID_CURRENT_TRANSITION_RECORD ||
            source.disposition == PairDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == PairDisposition::INVALID_PREVIOUS_PAIR_RECORD;
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


    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsNeutralPairDisposition(PairDisposition disposition) noexcept
    {
        return disposition >= PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            disposition <= PairDisposition::NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsCanonicalNonProvenRun(const Record& source) noexcept
    {
        const bool known = source.disposition >= Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            source.disposition <= Disposition::INVALID_CONTINUITY_RUN;
        const bool identity = source.disposition == Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE
            ? source.currentCertificatePublicationSequence == 0ULL
            : source.currentCertificatePublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_PAIR_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_RUN_RECORD;
        if (!known || !identity || source.publicationSequence == 0ULL ||
            source.schemaVersion != NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_SCHEMA_V1 ||
            source.runGeneration != 0ULL || source.firstCertificatePublicationSequence != 0ULL ||
            source.currentCoverageTransitionPublicationSequence != 0ULL || source.currentCoveragePublicationSequence != 0ULL ||
            source.coverageGeneration != 0ULL || source.sourceRunPublicationSequence != 0ULL ||
            source.continuityRunGeneration != 0ULL || source.firstPairPublicationSequence != 0ULL ||
            source.sourcePairPublicationSequence != 0ULL || source.firstTransitionPublicationSequence != 0ULL ||
            source.currentTransitionPublicationSequence != 0ULL || source.firstBoundaryPublicationSequence != 0ULL ||
            source.currentBoundaryPublicationSequence != 0ULL || source.consecutivePairCount != 0U ||
            source.consecutiveCertificateCount != 0U || source.firstSourcePairCount != 0U ||
            source.previousObservedPairRelationMask != 0U || source.currentObservedPairRelationMask != 0U ||
            source.newlyObservedPairRelationMask != 0U || source.retainedPairRelationMask != 0U ||
            source.fullCoverageTransition != FullCoverageTransition::NONE || source.firstPairRelation != PairRelation::NONE ||
            source.sourcePairRelation != PairRelation::NONE || source.previousSourcePairRelation != PairRelation::NONE ||
            source.previousTransitionRelation != TransitionRelation::NONE || source.currentTransitionRelation != TransitionRelation::NONE ||
            source.currentUnavailableReason != UnavailableReason::NONE || source.earlierObservedPairRelationMask != 0U ||
            source.earlierSourcePairRelation != PairRelation::NONE || source.discoveryPairRelation != DiscoveryPairRelation::NONE ||
            source.fullCoveragePairRelation != FullCoveragePairRelation::NONE) return false;
        for (std::uint8_t value : source.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool PreviousSourceMatches(const PairRecord& source, const Record& previous) noexcept
    {
        return source.publicationSequence == previous.currentCertificatePublicationSequence &&
            source.currentCoverageTransitionPublicationSequence == previous.currentCoverageTransitionPublicationSequence && source.currentCoveragePublicationSequence == previous.currentCoveragePublicationSequence &&
            source.coverageGeneration == previous.coverageGeneration && source.sourceRunPublicationSequence == previous.sourceRunPublicationSequence &&
            source.continuityRunGeneration == previous.continuityRunGeneration && source.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            source.sourcePairPublicationSequence == previous.sourcePairPublicationSequence && source.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            source.currentTransitionPublicationSequence == previous.currentTransitionPublicationSequence && source.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            source.currentBoundaryPublicationSequence == previous.currentBoundaryPublicationSequence && source.consecutivePairCount == previous.consecutivePairCount &&
            source.previousObservedPairRelationMask == previous.previousObservedPairRelationMask && source.currentObservedPairRelationMask == previous.currentObservedPairRelationMask &&
            source.newlyObservedPairRelationMask == previous.newlyObservedPairRelationMask && source.retainedPairRelationMask == previous.retainedPairRelationMask &&
            source.fullCoverageTransition == previous.fullCoverageTransition && source.firstPairRelation == previous.firstPairRelation &&
            source.sourcePairRelation == previous.sourcePairRelation && source.previousSourcePairRelation == previous.previousSourcePairRelation &&
            source.previousTransitionRelation == previous.previousTransitionRelation && source.currentTransitionRelation == previous.currentTransitionRelation &&
            source.currentUnavailableReason == previous.currentUnavailableReason && source.earlierObservedPairRelationMask == previous.earlierObservedPairRelationMask &&
            source.earlierSourcePairRelation == previous.earlierSourcePairRelation && source.discoveryPairRelation == previous.discoveryPairRelation &&
            source.fullCoveragePairRelation == previous.fullCoveragePairRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool SameCoverageRun(const PairRecord& previous, const PairRecord& current) noexcept
    {
        return current.coverageGeneration == previous.coverageGeneration &&
            current.continuityRunGeneration == previous.continuityRunGeneration &&
            current.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            current.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            current.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            current.firstPairRelation == previous.firstPairRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool DirectPairAdvance(const PairRecord& previous, const PairRecord& current) noexcept
    {
        using namespace NCPathCoreRunCoverageTransitionPairRunDetail;
        return previous.consecutivePairCount != (std::numeric_limits<std::uint32_t>::max)() &&
            current.consecutivePairCount == previous.consecutivePairCount + 1U &&
            current.publicationSequence == NextNonZeroSequence(previous.publicationSequence) &&
            current.currentCoverageTransitionPublicationSequence == NextNonZeroSequence(previous.currentCoverageTransitionPublicationSequence) &&
            current.currentCoveragePublicationSequence == NextNonZeroSequence(previous.currentCoveragePublicationSequence) &&
            current.sourceRunPublicationSequence == NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence == NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            current.currentTransitionPublicationSequence == NextNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            current.currentBoundaryPublicationSequence == NextNonZeroSequence(previous.currentBoundaryPublicationSequence);
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool SharedPairBinding(const PairRecord& previous, const PairRecord& current) noexcept
    {
        return current.earlierObservedPairRelationMask == previous.previousObservedPairRelationMask &&
            current.previousObservedPairRelationMask == previous.currentObservedPairRelationMask &&
            current.earlierSourcePairRelation == previous.previousSourcePairRelation &&
            current.previousSourcePairRelation == previous.sourcePairRelation &&
            current.previousTransitionRelation == previous.currentTransitionRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsRetainedRunRecurrence(const Record* older, const Record& newest) noexcept
    {
        using namespace NCPathCoreRunCoverageTransitionPairRunDetail;
        if (older == nullptr || !older->IsProvenRunCoverageTransitionPairRun() ||
            newest.publicationSequence != NextNonZeroSequence(older->publicationSequence) ||
            older->consecutiveCertificateCount >= (std::numeric_limits<std::uint32_t>::max)() - 3U ||
            newest.consecutiveCertificateCount != older->consecutiveCertificateCount + 1U ||
            newest.runGeneration != older->runGeneration ||
            newest.firstCertificatePublicationSequence != older->firstCertificatePublicationSequence ||
            newest.firstSourcePairCount != older->firstSourcePairCount ||
            older->consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)() ||
            newest.consecutivePairCount != older->consecutivePairCount + 1U ||
            newest.coverageGeneration != older->coverageGeneration ||
            newest.continuityRunGeneration != older->continuityRunGeneration ||
            newest.firstPairPublicationSequence != older->firstPairPublicationSequence ||
            newest.firstTransitionPublicationSequence != older->firstTransitionPublicationSequence ||
            newest.firstBoundaryPublicationSequence != older->firstBoundaryPublicationSequence ||
            newest.firstPairRelation != older->firstPairRelation ||
            newest.currentCertificatePublicationSequence != NextNonZeroSequence(older->currentCertificatePublicationSequence) ||
            newest.currentCoverageTransitionPublicationSequence != NextNonZeroSequence(older->currentCoverageTransitionPublicationSequence) ||
            newest.currentCoveragePublicationSequence != NextNonZeroSequence(older->currentCoveragePublicationSequence) ||
            newest.sourceRunPublicationSequence != NextNonZeroSequence(older->sourceRunPublicationSequence) ||
            newest.sourcePairPublicationSequence != NextNonZeroSequence(older->sourcePairPublicationSequence) ||
            newest.currentTransitionPublicationSequence != NextNonZeroSequence(older->currentTransitionPublicationSequence) ||
            newest.currentBoundaryPublicationSequence != NextNonZeroSequence(older->currentBoundaryPublicationSequence) ||
            newest.earlierObservedPairRelationMask != older->previousObservedPairRelationMask ||
            newest.previousObservedPairRelationMask != older->currentObservedPairRelationMask ||
            newest.earlierSourcePairRelation != older->previousSourcePairRelation ||
            newest.previousSourcePairRelation != older->sourcePairRelation ||
            newest.previousTransitionRelation != older->currentTransitionRelation) return false;
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static void EvaluateRun(const PairRecord* previousPair, const PairRecord& currentPair,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (!currentPair.IsProvenRunCoverageTransitionPair())
        {
            target.disposition = !IsCanonicalNonProvenPair(currentPair) ? Disposition::INVALID_CURRENT_PAIR_RECORD :
                IsNeutralPairDisposition(currentPair.disposition) ? Disposition::NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_CURRENT_PAIR_INVALID;
            return;
        }
        if (previousPair == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE;
            return;
        }
        if (!previousPair->IsProvenRunCoverageTransitionPair())
        {
            target.disposition = !IsCanonicalNonProvenPair(*previousPair) ? Disposition::INVALID_PREVIOUS_PAIR_RECORD :
                IsNeutralPairDisposition(previousPair->disposition) ? Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_PREVIOUS_PAIR_INVALID;
            return;
        }
        if (previousProven && previous != nullptr && !PreviousSourceMatches(*previousPair, *previous))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (!SameCoverageRun(*previousPair, currentPair))
        {
            target.disposition = Disposition::NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN;
            return;
        }
        if (previousPair->consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)() ||
            (previousProven && previous != nullptr &&
                previous->consecutiveCertificateCount >= (std::numeric_limits<std::uint32_t>::max)() - 3U))
        {
            target.disposition = Disposition::INVALID_PAIR_COUNT_OVERFLOW;
            return;
        }
        if (!DirectPairAdvance(*previousPair, currentPair))
        {
            target.disposition = Disposition::INVALID_PAIR_ADVANCE;
            return;
        }
        if (!SharedPairBinding(*previousPair, currentPair))
        {
            target.disposition = Disposition::INVALID_SHARED_PAIR_BINDING;
            return;
        }
        BuildRun(*previousPair, currentPair, previous, previousProven, target);
        if (!target.IsProvenRunCoverageTransitionPairRun())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_CONTINUITY_RUN;
        }
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static void BuildRun(const PairRecord& previousPair, const PairRecord& currentPair,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (previousProven && previous != nullptr)
        {
            target.runGeneration = previous->runGeneration;
            target.firstCertificatePublicationSequence = previous->firstCertificatePublicationSequence;
            target.consecutiveCertificateCount = previous->consecutiveCertificateCount + 1U;
            target.firstSourcePairCount = previous->firstSourcePairCount;
            target.disposition = Disposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION;
        }
        else
        {
            target.runGeneration = target.publicationSequence;
            target.firstCertificatePublicationSequence = previousPair.publicationSequence;
            target.consecutiveCertificateCount = 2U;
            target.firstSourcePairCount = previousPair.consecutivePairCount;
            target.disposition = Disposition::PROVEN_LOCAL_PAIR_RUN_START;
        }
        target.currentCoverageTransitionPublicationSequence = currentPair.currentCoverageTransitionPublicationSequence;
        target.currentCoveragePublicationSequence = currentPair.currentCoveragePublicationSequence;
        target.coverageGeneration = currentPair.coverageGeneration;
        target.sourceRunPublicationSequence = currentPair.sourceRunPublicationSequence;
        target.continuityRunGeneration = currentPair.continuityRunGeneration;
        target.firstPairPublicationSequence = currentPair.firstPairPublicationSequence;
        target.sourcePairPublicationSequence = currentPair.sourcePairPublicationSequence;
        target.firstTransitionPublicationSequence = currentPair.firstTransitionPublicationSequence;
        target.currentTransitionPublicationSequence = currentPair.currentTransitionPublicationSequence;
        target.firstBoundaryPublicationSequence = currentPair.firstBoundaryPublicationSequence;
        target.currentBoundaryPublicationSequence = currentPair.currentBoundaryPublicationSequence;
        target.consecutivePairCount = currentPair.consecutivePairCount;
        target.previousObservedPairRelationMask = currentPair.previousObservedPairRelationMask;
        target.currentObservedPairRelationMask = currentPair.currentObservedPairRelationMask;
        target.newlyObservedPairRelationMask = currentPair.newlyObservedPairRelationMask;
        target.retainedPairRelationMask = currentPair.retainedPairRelationMask;
        target.fullCoverageTransition = currentPair.fullCoverageTransition;
        target.firstPairRelation = currentPair.firstPairRelation;
        target.sourcePairRelation = currentPair.sourcePairRelation;
        target.previousSourcePairRelation = currentPair.previousSourcePairRelation;
        target.previousTransitionRelation = currentPair.previousTransitionRelation;
        target.currentTransitionRelation = currentPair.currentTransitionRelation;
        target.currentUnavailableReason = currentPair.currentUnavailableReason;
        target.earlierObservedPairRelationMask = currentPair.earlierObservedPairRelationMask;
        target.earlierSourcePairRelation = currentPair.earlierSourcePairRelation;
        target.discoveryPairRelation = currentPair.discoveryPairRelation;
        target.fullCoveragePairRelation = currentPair.fullCoveragePairRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        target.runGeneration = 0ULL;
        target.firstCertificatePublicationSequence = 0ULL;
        target.currentCoverageTransitionPublicationSequence = 0ULL;
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
        target.consecutiveCertificateCount = 0U;
        target.firstSourcePairCount = 0U;
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

    NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentCertificatePublicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_NOINLINE

static_assert(sizeof(NCPathCoreRunCoverageTransitionPairRunDisposition) == 1U, "Z disposition one byte.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageTransitionPairRunRecordV1>::value, "Z record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageTransitionPairRunRecordV1>::value, "Z record scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageTransitionPairRunRecordV1) == 152U, "Z record exactly 152 bytes.");
static_assert(alignof(NCPathCoreRunCoverageTransitionPairRunRecordV1) == 8U, "Z record alignment.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, runGeneration) == 8U, "Z runGeneration offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, firstCertificatePublicationSequence) == 16U, "Z firstCertificatePublicationSequence offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, currentCertificatePublicationSequence) == 24U, "Z currentCertificatePublicationSequence offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, consecutivePairCount) == 120U, "Z consecutivePairCount offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, consecutiveCertificateCount) == 124U, "Z consecutiveCertificateCount offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, firstSourcePairCount) == 128U, "Z firstSourcePairCount offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, schemaVersion) == 132U, "Z schemaVersion offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, disposition) == 134U, "Z disposition offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, previousObservedPairRelationMask) == 135U, "Z previousObservedPairRelationMask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, currentObservedPairRelationMask) == 136U, "Z currentObservedPairRelationMask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, newlyObservedPairRelationMask) == 137U, "Z newlyObservedPairRelationMask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, retainedPairRelationMask) == 138U, "Z retainedPairRelationMask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, fullCoverageTransition) == 139U, "Z fullCoverageTransition offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, firstPairRelation) == 140U, "Z firstPairRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, sourcePairRelation) == 141U, "Z sourcePairRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, previousSourcePairRelation) == 142U, "Z previousSourcePairRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, previousTransitionRelation) == 143U, "Z previousTransitionRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, currentTransitionRelation) == 144U, "Z currentTransitionRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, currentUnavailableReason) == 145U, "Z currentUnavailableReason offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, earlierObservedPairRelationMask) == 146U, "Z earlierObservedPairRelationMask offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, earlierSourcePairRelation) == 147U, "Z earlierSourcePairRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, discoveryPairRelation) == 148U, "Z discoveryPairRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, fullCoveragePairRelation) == 149U, "Z fullCoveragePairRelation offset.");
static_assert(offsetof(NCPathCoreRunCoverageTransitionPairRunRecordV1, reserved) == 150U, "Z reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageTransitionPairRunShadow>::value, "Z observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageTransitionPairRunShadow>::value, "Z observer scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageTransitionPairRunShadow) == 320U, "Z observer exactly 320 bytes.");
static_assert(alignof(NCPathCoreRunCoverageTransitionPairRunShadow) == 8U, "Z observer alignment.");
