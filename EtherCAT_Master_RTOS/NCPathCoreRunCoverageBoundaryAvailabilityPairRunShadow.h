#pragma once

#include "NCPathCoreRunCoverageBoundaryAvailabilityPairShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2AD / Run Pattern Coverage Transition Pair Continuity Run
// Observed-Head Coverage Boundary Availability Transition Pair Continuity
// Run-Length Shadow.
//
// Counts consecutive, directly overlapping, individually proven AC pair
// certificates observed here. A complete adjacent pair starts at count 2;
// a later complete pair extends only after exhaustive retained AC binding.
// This local observer run can continue across compatible available/lost/
// unavailable/gained relations and a different lower Z run. It is not one
// available AA interval, one committed run, physical duration, event count,
// segment count, path availability, geometric loss or Motion completion.
// firstPairPublicationSequence identifies the first counted AC certificate,
// not the oldest underlying AB/AA certificate. No earlier AB/AA/Z run heads
// are retained or inferred. uint32 overflow rejects; it never saturates or
// silently starts another run in the same observation.
//
// Admission checks both complete AC proofs, direct AC publication, shared
// AB identity/relation, the supplied AB edge and both cached current AA
// records. Retention rechecks the first-Y mask, Z/source heads, both counts,
// eight direct Z/Y/X/W/V/U/T/S publications and shared masks/relations.
// A missing lower Z anchor establishes no common lower interval. Invalid,
// stale, gapped or source-not-proven AC is not an ordinary unavailable AA.
//
// Current AC has discarded its previous full AB (and AB its previous full
// AA). The supplied overlap can prove compatibility, not byte identity
// with that omitted payload. In particular an omitted shared neutral AA
// reason cannot be authenticated on attachment. All retained semantic
// AC/AB/AA/Z fields are bound on extension; padding is not compared.
// Trusted same-thread producers are required. Coherent forged history,
// complete-wrap identity reuse and wall-clock freshness are not proven.
// Record self-proof cannot repeat the omitted previous-AC admission fence.
//
// Newest proof/canonical diagnostic is checked before older-slot overwrite.
// Every rejection terminates the proven local run and clears proof payload,
// retaining only current AC publication as an anchor; null clears it.
// A later complete adjacent pair may start a fresh observed suffix at 2,
// without filling in missing certificates. Older slot is diagnostic only.
// Two 248-byte scalar records in a 512-byte heap-owned NCManager observer;
// direct source lvalue-to-slot copy, no source snapshot on the stack or
// Observe allocation. No coordinates, displacement, endpoint arrays, nodes,
// segment/event list, total event ordering or reverse traversal data.
// Shadow-only: no Path/Motion Queue, planner, STARTED/DONE proof, actual
// position history, B2 breadcrumb/execution, control consumer, log, thread,
// timer, mutex, wait or sleep. Predecessors, M00 FIX1, L.2I FIX1, Motion and
// Priority-64 PDO remain unchanged.

constexpr std::size_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_SCHEMA_V1 = 1U;

enum class NCPathCoreRunCoverageBoundaryAvailabilityPairRunDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE = 2U,
    NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN = 3U,
    NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN = 4U,
    SOURCE_REPORTED_CURRENT_PAIR_INVALID = 5U,
    SOURCE_REPORTED_PREVIOUS_PAIR_INVALID = 6U,
    INVALID_CURRENT_PAIR_RECORD = 7U,
    INVALID_PREVIOUS_PAIR_RECORD = 8U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 9U,
    INVALID_PREVIOUS_RUN_RECORD = 10U,
    INVALID_PREVIOUS_SOURCE_BINDING = 11U,
    INVALID_PAIR_ADVANCE = 12U,
    INVALID_SHARED_TRANSITION_BINDING = 13U,
    INVALID_TRANSITION_ADVANCE = 14U,
    INVALID_SHARED_BOUNDARY_BINDING = 15U,
    INVALID_AVAILABILITY_STATE_BINDING = 16U,
    INVALID_BOUNDARY_ADVANCE = 17U,
    INVALID_SOURCE_RUN_ADVANCE = 18U,
    INVALID_SOURCE_RUN_BINDING = 19U,
    INVALID_SHARED_RUN_BINDING = 20U,
    INVALID_SOURCE_RUN_COUNT_OVERFLOW = 21U,
    INVALID_PAIR_COUNT_OVERFLOW = 22U,
    INVALID_CONTINUITY_RUN = 23U,
    PROVEN_LOCAL_PAIR_RUN_START = 24U,
    PROVEN_LOCAL_PAIR_RUN_EXTENSION = 25U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
#endif

namespace NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail
{
    using PairRecord = NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1;
    using PairDisposition = PairRecord::Disposition;
    using PairRelation = PairRecord::Relation;
    using TransitionRecord = PairRecord::TransitionRecord;
    using TransitionDisposition = TransitionRecord::Disposition;
    using TransitionRelation = TransitionRecord::Relation;
    using Availability = TransitionRecord::Availability;
    using UnavailableReason = TransitionRecord::UnavailableReason;
    using BoundaryRecord = TransitionRecord::BoundaryRecord;
    using BoundaryDisposition = BoundaryRecord::Disposition;
    using RunRecord = BoundaryRecord::SourceRecord;
    using NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::NextNonZeroSequence;
    using NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::IsZeroBoundaryPayload;

    constexpr std::uint64_t ForwardNonZeroSequenceDistance(std::uint64_t first, std::uint64_t current) noexcept
    {
        return current >= first ? current - first : ((std::numeric_limits<std::uint64_t>::max)() - first) + current;
    }

    // Excludes only the AC publication diagnostic anchor.
    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        inline bool IsZeroPairPayload(const PairRecord& pair) noexcept
    {
        const TransitionRecord& transition = pair.currentTransition;
        return pair.previousTransitionPublicationSequence == 0ULL &&
            pair.schemaVersion == 0U && pair.disposition == PairDisposition::EMPTY &&
            pair.previousRelation == TransitionRelation::NONE && pair.relation == PairRelation::NONE &&
            pair.reserved[0U] == 0U && pair.reserved[1U] == 0U && pair.reserved[2U] == 0U &&
            transition.publicationSequence == 0ULL && transition.previousBoundaryPublicationSequence == 0ULL &&
            transition.currentBoundary.publicationSequence == 0ULL && transition.schemaVersion == 0U &&
            transition.disposition == TransitionDisposition::EMPTY && transition.relation == TransitionRelation::NONE &&
            transition.previousAvailability == Availability::UNKNOWN && transition.currentAvailability == Availability::UNKNOWN &&
            transition.currentUnavailableReason == UnavailableReason::NONE && transition.reserved == 0U &&
            IsZeroBoundaryPayload(transition.currentBoundary);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        inline bool IsCanonicalNonProvenPair(const PairRecord& record) noexcept
    {

        const bool known = record.disposition >= PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            record.disposition <= PairDisposition::INVALID_AVAILABILITY_TRANSITION;
        // A missing pointer has zero identity; canonical unproven AB carries
        // a nonzero diagnostic identity under the same unavailable disposition.
        const bool identity = record.currentTransition.publicationSequence != 0ULL ||
            record.disposition == PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE ||
            record.disposition == PairDisposition::INVALID_CURRENT_TRANSITION_RECORD ||
            record.disposition == PairDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            record.disposition == PairDisposition::INVALID_PREVIOUS_PAIR_RECORD;
        const TransitionRecord& transition = record.currentTransition;
        return known && identity && record.publicationSequence != 0ULL &&
            record.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_SCHEMA_V1 &&
            record.previousTransitionPublicationSequence == 0ULL && record.previousRelation == TransitionRelation::NONE &&
            record.relation == PairRelation::NONE && record.reserved[0U] == 0U &&
            record.reserved[1U] == 0U && record.reserved[2U] == 0U &&
            transition.previousBoundaryPublicationSequence == 0ULL && transition.currentBoundary.publicationSequence == 0ULL &&
            transition.schemaVersion == 0U && transition.disposition == TransitionDisposition::EMPTY &&
            transition.relation == TransitionRelation::NONE && transition.previousAvailability == Availability::UNKNOWN &&
            transition.currentAvailability == Availability::UNKNOWN && transition.currentUnavailableReason == UnavailableReason::NONE &&
            transition.reserved == 0U && IsZeroBoundaryPayload(transition.currentBoundary);
    }

    inline bool IsNeutralPairDisposition(PairDisposition disposition) noexcept
    {
        return disposition == PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE ||
            disposition == PairDisposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE;
    }
}

struct NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1
{
    using Disposition = NCPathCoreRunCoverageBoundaryAvailabilityPairRunDisposition;
    using PairRecord = NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::PairRecord;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t continuityRunGeneration = 0ULL;
    std::uint64_t firstPairPublicationSequence = 0ULL;
    PairRecord currentPair{};
    std::uint32_t consecutivePairCount = 0U;
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    std::uint8_t reserved = 0U;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        bool IsProvenRunCoverageBoundaryAvailabilityPairRun() const noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::ForwardNonZeroSequenceDistance;
        if (publicationSequence == 0ULL || continuityRunGeneration == 0ULL || firstPairPublicationSequence == 0ULL ||
            schemaVersion != NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_SCHEMA_V1 || reserved != 0U ||
            !currentPair.IsProvenRunCoverageBoundaryAvailabilityPair()) return false;
        const bool start = disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_START && consecutivePairCount == 2U;
        const bool extension = disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION && consecutivePairCount > 2U;
        if (!start && !extension) return false;
        const std::uint64_t count = static_cast<std::uint64_t>(consecutivePairCount);
        return ForwardNonZeroSequenceDistance(continuityRunGeneration, publicationSequence) == count - 2ULL &&
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, currentPair.publicationSequence) == count - 1ULL;
    }
};

class NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow final
{
public:
    using Record = NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1;
    using Disposition = Record::Disposition;
    using PairRecord = Record::PairRecord;
    using PairDisposition = PairRecord::Disposition;
    using PairRelation = PairRecord::Relation;
    using TransitionRecord = PairRecord::TransitionRecord;
    using TransitionDisposition = TransitionRecord::Disposition;
    using TransitionRelation = TransitionRecord::Relation;
    using Availability = TransitionRecord::Availability;
    using UnavailableReason = TransitionRecord::UnavailableReason;
    using BoundaryRecord = TransitionRecord::BoundaryRecord;
    using BoundaryDisposition = BoundaryRecord::Disposition;
    using RunRecord = BoundaryRecord::SourceRecord;

    NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow() noexcept = default;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        void ObserveImmediateRunCoverageBoundaryAvailabilityPairRunSameThread(const PairRecord* previousPair,
            const PairRecord* currentPair) noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::NextNonZeroSequence;
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenRunCoverageBoundaryAvailabilityPairRun();
        // Validate newest before clearing the independent older diagnostic slot.
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            (previousProven || IsCanonicalNonProvenRun(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentPair.publicationSequence;
        m_publicationSequence = NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentPair == nullptr ? 0ULL : currentPair->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_RUN_RECORD;
        else if (currentPair == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
        else if (anchor != 0ULL && currentPair->publicationSequence != NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluatePair(previousPair, *currentPair, previous, previousProven, anchor, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_HISTORY_CAPACITY;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool IsCanonicalNonProvenRun(const Record& record) noexcept
    {
        const bool known = record.disposition >= Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            record.disposition <= Disposition::INVALID_CONTINUITY_RUN;
        const bool identity = record.disposition == Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE ?
            record.currentPair.publicationSequence == 0ULL : record.currentPair.publicationSequence != 0ULL ||
            record.disposition == Disposition::INVALID_CURRENT_PAIR_RECORD ||
            record.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            record.disposition == Disposition::INVALID_PREVIOUS_RUN_RECORD;
        return known && identity && record.publicationSequence != 0ULL &&
            record.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_SCHEMA_V1 &&
            record.continuityRunGeneration == 0ULL && record.firstPairPublicationSequence == 0ULL &&
            record.consecutivePairCount == 0U && record.reserved == 0U &&
            NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::IsZeroPairPayload(record.currentPair);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool PreviousPairMatches(const PairRecord& source, const PairRecord& retained) noexcept
    {
        return source.publicationSequence == retained.publicationSequence &&
            source.previousTransitionPublicationSequence == retained.previousTransitionPublicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.previousRelation == retained.previousRelation && source.relation == retained.relation &&
            source.reserved[0U] == retained.reserved[0U] && source.reserved[1U] == retained.reserved[1U] &&
            source.reserved[2U] == retained.reserved[2U] &&
            PreviousSourceMatches(source.currentTransition, retained.currentTransition);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool PreviousSourceMatches(const TransitionRecord& source, const TransitionRecord& retained) noexcept
    {
        return source.publicationSequence == retained.publicationSequence &&
            source.previousBoundaryPublicationSequence == retained.previousBoundaryPublicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.relation == retained.relation && source.previousAvailability == retained.previousAvailability &&
            source.currentAvailability == retained.currentAvailability &&
            source.currentUnavailableReason == retained.currentUnavailableReason && source.reserved == retained.reserved &&
            PreviousBoundaryMatches(source.currentBoundary, retained.currentBoundary);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool PreviousBoundaryMatches(const BoundaryRecord& source, const BoundaryRecord& retained) noexcept
    {
        // Semantic equality is exhaustive; padding and object representations are not read.
        return source.publicationSequence == retained.publicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.firstYCoverageMask == retained.firstYCoverageMask &&
            source.gainedSinceFirstYMask == retained.gainedSinceFirstYMask &&
            source.fullCoverageRelation == retained.fullCoverageRelation &&
            source.reserved[0U] == retained.reserved[0U] && source.reserved[1U] == retained.reserved[1U] &&
            source.currentRun.publicationSequence == retained.currentRun.publicationSequence &&
            source.currentRun.runGeneration == retained.currentRun.runGeneration &&
            source.currentRun.firstCertificatePublicationSequence == retained.currentRun.firstCertificatePublicationSequence &&
            source.currentRun.currentCertificatePublicationSequence == retained.currentRun.currentCertificatePublicationSequence &&
            source.currentRun.currentCoverageTransitionPublicationSequence == retained.currentRun.currentCoverageTransitionPublicationSequence &&
            source.currentRun.currentCoveragePublicationSequence == retained.currentRun.currentCoveragePublicationSequence &&
            source.currentRun.coverageGeneration == retained.currentRun.coverageGeneration &&
            source.currentRun.sourceRunPublicationSequence == retained.currentRun.sourceRunPublicationSequence &&
            source.currentRun.continuityRunGeneration == retained.currentRun.continuityRunGeneration &&
            source.currentRun.firstPairPublicationSequence == retained.currentRun.firstPairPublicationSequence &&
            source.currentRun.sourcePairPublicationSequence == retained.currentRun.sourcePairPublicationSequence &&
            source.currentRun.firstTransitionPublicationSequence == retained.currentRun.firstTransitionPublicationSequence &&
            source.currentRun.currentTransitionPublicationSequence == retained.currentRun.currentTransitionPublicationSequence &&
            source.currentRun.firstBoundaryPublicationSequence == retained.currentRun.firstBoundaryPublicationSequence &&
            source.currentRun.currentBoundaryPublicationSequence == retained.currentRun.currentBoundaryPublicationSequence &&
            source.currentRun.consecutivePairCount == retained.currentRun.consecutivePairCount &&
            source.currentRun.consecutiveCertificateCount == retained.currentRun.consecutiveCertificateCount &&
            source.currentRun.firstSourcePairCount == retained.currentRun.firstSourcePairCount &&
            source.currentRun.schemaVersion == retained.currentRun.schemaVersion &&
            source.currentRun.disposition == retained.currentRun.disposition &&
            source.currentRun.previousObservedPairRelationMask == retained.currentRun.previousObservedPairRelationMask &&
            source.currentRun.currentObservedPairRelationMask == retained.currentRun.currentObservedPairRelationMask &&
            source.currentRun.newlyObservedPairRelationMask == retained.currentRun.newlyObservedPairRelationMask &&
            source.currentRun.retainedPairRelationMask == retained.currentRun.retainedPairRelationMask &&
            source.currentRun.fullCoverageTransition == retained.currentRun.fullCoverageTransition &&
            source.currentRun.firstPairRelation == retained.currentRun.firstPairRelation &&
            source.currentRun.sourcePairRelation == retained.currentRun.sourcePairRelation &&
            source.currentRun.previousSourcePairRelation == retained.currentRun.previousSourcePairRelation &&
            source.currentRun.previousTransitionRelation == retained.currentRun.previousTransitionRelation &&
            source.currentRun.currentTransitionRelation == retained.currentRun.currentTransitionRelation &&
            source.currentRun.currentUnavailableReason == retained.currentRun.currentUnavailableReason &&
            source.currentRun.earlierObservedPairRelationMask == retained.currentRun.earlierObservedPairRelationMask &&
            source.currentRun.earlierSourcePairRelation == retained.currentRun.earlierSourcePairRelation &&
            source.currentRun.discoveryPairRelation == retained.currentRun.discoveryPairRelation &&
            source.currentRun.fullCoveragePairRelation == retained.currentRun.fullCoveragePairRelation &&
            source.currentRun.reserved[0U] == retained.currentRun.reserved[0U] &&
            source.currentRun.reserved[1U] == retained.currentRun.reserved[1U];
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool SameSourceRun(const RunRecord& previous, const RunRecord& current) noexcept
    {
        return current.runGeneration == previous.runGeneration &&
            current.firstCertificatePublicationSequence == previous.firstCertificatePublicationSequence &&
            current.firstSourcePairCount == previous.firstSourcePairCount &&
            current.coverageGeneration == previous.coverageGeneration &&
            current.continuityRunGeneration == previous.continuityRunGeneration &&
            current.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            current.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            current.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            current.firstPairRelation == previous.firstPairRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool DirectRunAdvance(const RunRecord& previous, const RunRecord& current) noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityDetail::NextNonZeroSequence;
        return previous.consecutivePairCount != (std::numeric_limits<std::uint32_t>::max)() &&
            previous.consecutiveCertificateCount < (std::numeric_limits<std::uint32_t>::max)() - 3U &&
            current.consecutivePairCount == previous.consecutivePairCount + 1U &&
            current.consecutiveCertificateCount == previous.consecutiveCertificateCount + 1U &&
            current.publicationSequence == NextNonZeroSequence(previous.publicationSequence) &&
            current.currentCertificatePublicationSequence == NextNonZeroSequence(previous.currentCertificatePublicationSequence) &&
            current.currentCoverageTransitionPublicationSequence == NextNonZeroSequence(previous.currentCoverageTransitionPublicationSequence) &&
            current.currentCoveragePublicationSequence == NextNonZeroSequence(previous.currentCoveragePublicationSequence) &&
            current.sourceRunPublicationSequence == NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence == NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            current.currentTransitionPublicationSequence == NextNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            current.currentBoundaryPublicationSequence == NextNonZeroSequence(previous.currentBoundaryPublicationSequence);
    }

    static bool SharedRunBinding(const RunRecord& previous, const RunRecord& current) noexcept
    {
        return current.earlierObservedPairRelationMask == previous.previousObservedPairRelationMask &&
            current.previousObservedPairRelationMask == previous.currentObservedPairRelationMask &&
            current.earlierSourcePairRelation == previous.previousSourcePairRelation &&
            current.previousSourcePairRelation == previous.sourcePairRelation &&
            current.previousTransitionRelation == previous.currentTransitionRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static void EvaluatePair(const PairRecord* previousPair, const PairRecord& currentPair,
            const Record* previous, bool previousProven, std::uint64_t anchor, Record& target) noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail;
        if (!currentPair.IsProvenRunCoverageBoundaryAvailabilityPair())
        {
            target.disposition = !IsCanonicalNonProvenPair(currentPair) ? Disposition::INVALID_CURRENT_PAIR_RECORD :
                (IsNeutralPairDisposition(currentPair.disposition) ? Disposition::NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN :
                    Disposition::SOURCE_REPORTED_CURRENT_PAIR_INVALID);
            return;
        }
        if (previousPair == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE;
            return;
        }
        if (!previousPair->IsProvenRunCoverageBoundaryAvailabilityPair())
        {
            target.disposition = !IsCanonicalNonProvenPair(*previousPair) ? Disposition::INVALID_PREVIOUS_PAIR_RECORD :
                (IsNeutralPairDisposition(previousPair->disposition) ? Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN :
                    Disposition::SOURCE_REPORTED_PREVIOUS_PAIR_INVALID);
            return;
        }
        if (currentPair.publicationSequence != NextNonZeroSequence(previousPair->publicationSequence))
        {
            target.disposition = Disposition::INVALID_PAIR_ADVANCE;
            return;
        }
        if ((anchor != 0ULL && previousPair->publicationSequence != anchor) ||
            (previousProven && previous != nullptr && !PreviousPairMatches(*previousPair, previous->currentPair)))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (currentPair.previousTransitionPublicationSequence != previousPair->currentTransition.publicationSequence ||
            currentPair.previousRelation != previousPair->currentTransition.relation)
        {
            target.disposition = Disposition::INVALID_SHARED_TRANSITION_BINDING;
            return;
        }
        if (!CheckTransitionEdge(previousPair->currentTransition, currentPair.currentTransition, target)) return;
        if (previousProven && previous != nullptr && previous->consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)())
        {
            target.disposition = Disposition::INVALID_PAIR_COUNT_OVERFLOW;
            return;
        }
        BuildRun(*previousPair, currentPair, previous, previousProven, target);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool CheckTransitionEdge(const TransitionRecord& previousTransition,
            const TransitionRecord& currentTransition, Record& target) noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::NextNonZeroSequence;
        if (currentTransition.publicationSequence != NextNonZeroSequence(previousTransition.publicationSequence))
        {
            target.disposition = Disposition::INVALID_TRANSITION_ADVANCE;
            return false;
        }
        if (currentTransition.previousBoundaryPublicationSequence != previousTransition.currentBoundary.publicationSequence)
        {
            target.disposition = Disposition::INVALID_SHARED_BOUNDARY_BINDING;
            return false;
        }
        if (currentTransition.previousAvailability != previousTransition.currentAvailability)
        {
            target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
            return false;
        }
        return CheckBoundaryEdge(previousTransition, currentTransition, target);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static bool CheckBoundaryEdge(const TransitionRecord& previousTransition,
            const TransitionRecord& currentTransition, Record& target) noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::NextNonZeroSequence;
        const BoundaryRecord& previousBoundary = previousTransition.currentBoundary;
        const BoundaryRecord& currentBoundary = currentTransition.currentBoundary;
        if (currentBoundary.publicationSequence != NextNonZeroSequence(previousBoundary.publicationSequence))
        {
            target.disposition = Disposition::INVALID_BOUNDARY_ADVANCE;
            return false;
        }
        const RunRecord& previousRun = previousBoundary.currentRun;
        const RunRecord& currentRun = currentBoundary.currentRun;
        if (previousRun.publicationSequence != 0ULL && currentRun.publicationSequence != 0ULL &&
            currentRun.publicationSequence != NextNonZeroSequence(previousRun.publicationSequence))
        {
            target.disposition = Disposition::INVALID_SOURCE_RUN_ADVANCE;
            return false;
        }
        // Both AB self-proofs have already validated these complete AA states,
        // including the exact current unavailable reason. Only the supplied
        // overlap is available; current AB's discarded previous AA is unknown.
        const bool previousAvailable = previousTransition.currentAvailability == Availability::AVAILABLE;
        const bool currentAvailable = currentTransition.currentAvailability == Availability::AVAILABLE;
        if (currentAvailable)
        {
            if ((previousAvailable && currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION) ||
                (!previousAvailable && currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START))
            {
                target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
                return false;
            }
            if (previousAvailable)
            {
                if (currentBoundary.firstYCoverageMask != previousBoundary.firstYCoverageMask ||
                    !SameSourceRun(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_BINDING;
                    return false;
                }
                if (previousRun.consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)() ||
                    previousRun.consecutiveCertificateCount >= (std::numeric_limits<std::uint32_t>::max)() - 3U)
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_COUNT_OVERFLOW;
                    return false;
                }
                if (!DirectRunAdvance(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_ADVANCE;
                    return false;
                }
                if (!SharedRunBinding(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SHARED_RUN_BINDING;
                    return false;
                }
            }
        }
        else if (previousAvailable && currentBoundary.disposition == BoundaryDisposition::NOT_APPLICABLE_HEAD_NOT_OBSERVED)
        {
            target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
            return false;
        }
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static void BuildRun(const PairRecord& previousPair, const PairRecord& currentPair,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (previousProven && previous != nullptr)
        {
            target.continuityRunGeneration = previous->continuityRunGeneration;
            target.firstPairPublicationSequence = previous->firstPairPublicationSequence;
            target.consecutivePairCount = previous->consecutivePairCount + 1U;
            target.disposition = Disposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION;
        }
        else
        {
            target.continuityRunGeneration = target.publicationSequence;
            target.firstPairPublicationSequence = previousPair.publicationSequence;
            target.consecutivePairCount = 2U;
            target.disposition = Disposition::PROVEN_LOCAL_PAIR_RUN_START;
        }
        target.currentPair = currentPair; // Direct AC lvalue-to-slot copy; no AC/AB/AA/Z temporary.
        if (!target.IsProvenRunCoverageBoundaryAvailabilityPairRun())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_CONTINUITY_RUN;
        }
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        // Preserve only currentPair.publicationSequence as diagnostic identity.
        RunRecord& source = target.currentPair.currentTransition.currentBoundary.currentRun;
        source.publicationSequence = 0ULL;
        source.runGeneration = 0ULL;
        source.firstCertificatePublicationSequence = 0ULL;
        source.currentCertificatePublicationSequence = 0ULL;
        source.currentCoverageTransitionPublicationSequence = 0ULL;
        source.currentCoveragePublicationSequence = 0ULL;
        source.coverageGeneration = 0ULL;
        source.sourceRunPublicationSequence = 0ULL;
        source.continuityRunGeneration = 0ULL;
        source.firstPairPublicationSequence = 0ULL;
        source.sourcePairPublicationSequence = 0ULL;
        source.firstTransitionPublicationSequence = 0ULL;
        source.currentTransitionPublicationSequence = 0ULL;
        source.firstBoundaryPublicationSequence = 0ULL;
        source.currentBoundaryPublicationSequence = 0ULL;
        source.consecutivePairCount = 0U;
        source.consecutiveCertificateCount = 0U;
        source.firstSourcePairCount = 0U;
        source.schemaVersion = 0U;
        source.disposition = RunRecord::Disposition::EMPTY;
        source.previousObservedPairRelationMask = 0U;
        source.currentObservedPairRelationMask = 0U;
        source.newlyObservedPairRelationMask = 0U;
        source.retainedPairRelationMask = 0U;
        source.fullCoverageTransition = RunRecord::FullCoverageTransition::NONE;
        source.firstPairRelation = RunRecord::PairRelation::NONE;
        source.sourcePairRelation = RunRecord::PairRelation::NONE;
        source.previousSourcePairRelation = RunRecord::PairRelation::NONE;
        source.previousTransitionRelation = RunRecord::TransitionRelation::NONE;
        source.currentTransitionRelation = RunRecord::TransitionRelation::NONE;
        source.currentUnavailableReason = RunRecord::UnavailableReason::NONE;
        source.earlierObservedPairRelationMask = 0U;
        source.earlierSourcePairRelation = RunRecord::PairRelation::NONE;
        source.discoveryPairRelation = RunRecord::DiscoveryPairRelation::NONE;
        source.fullCoveragePairRelation = RunRecord::FullCoveragePairRelation::NONE;
        for (std::uint8_t& value : source.reserved) value = 0U;
        target.currentPair.currentTransition.currentBoundary.schemaVersion = 0U;
        target.currentPair.currentTransition.currentBoundary.disposition = BoundaryDisposition::EMPTY;
        target.currentPair.currentTransition.currentBoundary.firstYCoverageMask = 0U;
        target.currentPair.currentTransition.currentBoundary.gainedSinceFirstYMask = 0U;
        target.currentPair.currentTransition.currentBoundary.fullCoverageRelation = BoundaryRecord::FullCoverageRelation::NONE;
        for (std::uint8_t& value : target.currentPair.currentTransition.currentBoundary.reserved) value = 0U;
        target.currentPair.currentTransition.currentBoundary.publicationSequence = 0ULL;
        target.currentPair.currentTransition.previousBoundaryPublicationSequence = 0ULL;
        target.currentPair.currentTransition.schemaVersion = 0U;
        target.currentPair.currentTransition.disposition = TransitionDisposition::EMPTY;
        target.currentPair.currentTransition.relation = TransitionRelation::NONE;
        target.currentPair.currentTransition.previousAvailability = Availability::UNKNOWN;
        target.currentPair.currentTransition.currentAvailability = Availability::UNKNOWN;
        target.currentPair.currentTransition.currentUnavailableReason = UnavailableReason::NONE;
        target.currentPair.currentTransition.reserved = 0U;
        target.currentPair.previousTransitionPublicationSequence = 0ULL;
        target.currentPair.previousRelation = TransitionRelation::NONE;
        target.currentPair.relation = PairRelation::NONE;
        for (std::uint8_t& value : target.currentPair.reserved) value = 0U;
        target.currentPair.currentTransition.publicationSequence = 0ULL;
        target.currentPair.schemaVersion = 0U;
        target.currentPair.disposition = PairDisposition::EMPTY;
        target.continuityRunGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.consecutivePairCount = 0U;
        target.reserved = 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentPair.publicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_NOINLINE

static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunDisposition) == 1U, "AD disposition one byte.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1>::value, "AD record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1>::value, "AD record scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1) == 248U, "AD record exactly 248 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1) == 8U, "AD record alignment.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, continuityRunGeneration) == 8U, "AD generation offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, firstPairPublicationSequence) == 16U, "AD first AC identity offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, currentPair) == 24U, "AD retained AC offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, consecutivePairCount) == 240U, "AD count offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, schemaVersion) == 244U, "AD schema offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, disposition) == 246U, "AD disposition offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1, reserved) == 247U, "AD reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow>::value, "AD observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow>::value, "AD observer scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow) == 512U, "AD observer exactly 512 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow) == 8U, "AD observer alignment.");
