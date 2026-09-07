#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2R / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run-Length Shadow.
//
// Counts consecutively observed proven L.2Q continuity certificates.  Each
// extension binds directly advancing Q publications to the retained P/O/N/M
// identities, overlapping transition counts and coverage/run/chain context
// of the preceding local observation.  Q proved only its retained scalar
// projection; this observer does not reconstruct discarded source records.
// The count is NOT a segment count, qualification duration, qualification
// stability, complete event order or proof of an entire committed run.
// The underlying coverage interval may itself be a proven run suffix.
//
// Initial attachment or the first valid direct publication after an
// interruption starts at ONE certificate.  Invalid/nonproven/unavailable
// input, stale/gapped source publications or binding failures clear the
// proven payload.  A coverage change while a local run is proven publishes
// a boundary, discards that certificate and clears the run; a following
// valid direct certificate may start a new local run.  No gap is bridged.
// Source-reported coverage boundaries remain distinct from a direct
// observed coverage change and from a neutral cause that is not proven.
//
// continuityRunGeneration is the R publication at local run start, not a
// count of runs.  It and publication identities wrap, skipping zero.  Both
// the R publication span and Q publication span must equal count minus one;
// the nonwrapping transition-count span must agree as well.  Counters never
// saturate.  Freshness is local publication adjacency, not elapsed time or
// durable replay detection.  Null input clears the source freshness anchor;
// a rejected source publication is only a diagnostic resynchronization
// anchor.  Trusted same-thread producer records are assumed, not protection
// against coherent forged state or source identity reuse after a full wrap.
//
// Fixed two-record scalar history, NCManager heap-owned.  No Observe-time
// allocation, large stack object, coordinates, displacement, endpoints,
// nodes, segment/event list, retained masks or reverse-traversal data.
// Not Path/Motion Queue, planner, STARTED/DONE proof, actual-position
// history, B2 breadcrumb or execution.  Shadow-only, no control consumer:
// no Motion, G00, Gate/Registry, Alarm, PC, HMI/SHM/API, PDO, NIC, EtherCAT
// or DC changes; no logging, thread, timer, mutex, wait or sleep.

constexpr std::size_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_SCHEMA_V1 = 1U;

namespace NCPathCoreReturnQualificationTransitionPairContinuityRunDetail
{
    constexpr std::uint64_t NextNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL : value + 1ULL;
    }

    constexpr std::uint64_t ForwardNonZeroSequenceDistance(
        std::uint64_t first, std::uint64_t latest) noexcept
    {
        return latest >= first ? latest - first :
            ((std::numeric_limits<std::uint64_t>::max)() - first) + latest;
    }

    template<std::size_t Count>
    bool ReservedIsZero(const std::array<std::uint8_t, Count>& reserved) noexcept
    {
        for (std::size_t index = 0U; index < reserved.size(); ++index)
        {
            if (reserved[index] != 0U)
            {
                return false;
            }
        }
        return true;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CONTINUITY_UNAVAILABLE = 1U,
    NOT_APPLICABLE_CONTINUITY_NOT_PROVEN_CAUSE_UNKNOWN = 2U,
    NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY = 3U,
    NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY = 4U,
    INVALID_CURRENT_CONTINUITY_RECORD = 5U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 6U,
    INVALID_PREVIOUS_RUN_RECORD = 7U,
    INVALID_CONTINUITY_BINDING = 8U,
    INVALID_CONTINUITY_COUNT_OVERFLOW = 9U,
    INVALID_CONTINUITY_RUN = 10U,
    PROVEN_LOCAL_CONTINUITY_RUN_START = 11U,
    PROVEN_LOCAL_CONTINUITY_RUN_EXTENSION = 12U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRelation : std::uint8_t
{
    NONE = 0U,
    LOCAL_RUN_STARTED_FROM_ONE_PROVEN_CONTINUITY = 1U,
    DIRECT_SAME_COVERAGE_INTERVAL_CONTINUITY_RUN_EXTENDED = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunExtent : std::uint8_t
{
    NONE = 0U,
    SCALAR_LOCAL_CONTINUITY_CERTIFICATE_RUN_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1
{
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunExtent;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t continuityRunGeneration = 0ULL;
    std::uint64_t firstContinuityPublicationSequence = 0ULL;
    std::uint64_t sourceContinuityPublicationSequence = 0ULL;
    std::uint64_t latestPairPublicationSequence = 0ULL;
    std::uint64_t sharedTransitionPublicationSequence = 0ULL;
    std::uint64_t latestSharedQualificationPublicationSequence = 0ULL;
    std::uint64_t latestSharedSummaryPublicationSequence = 0ULL;
    std::uint64_t coverageGeneration = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;

    std::uint32_t firstSharedFirstProvenTransitionCount = 0U;
    std::uint32_t latestSharedFirstProvenTransitionCount = 0U;
    std::uint32_t latestSharedLastProvenTransitionCount = 0U;
    std::uint32_t consecutiveContinuityCount = 0U;

    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    Extent extent = Extent::NONE;
    std::array<std::uint8_t, 3U> reserved{};

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRun() const noexcept
    {
        if (publicationSequence == 0ULL || continuityRunGeneration == 0ULL ||
            firstContinuityPublicationSequence == 0ULL ||
            sourceContinuityPublicationSequence == 0ULL ||
            latestPairPublicationSequence == 0ULL ||
            sharedTransitionPublicationSequence == 0ULL ||
            latestSharedQualificationPublicationSequence == 0ULL ||
            latestSharedSummaryPublicationSequence == 0ULL ||
            coverageGeneration == 0ULL || runGeneration == 0ULL ||
            acceptedInputChainGeneration == 0ULL ||
            firstSharedFirstProvenTransitionCount <= 1U ||
            latestSharedFirstProvenTransitionCount < firstSharedFirstProvenTransitionCount ||
            latestSharedFirstProvenTransitionCount == (std::numeric_limits<std::uint32_t>::max)() ||
            latestSharedLastProvenTransitionCount != latestSharedFirstProvenTransitionCount + 1U ||
            latestSharedLastProvenTransitionCount == (std::numeric_limits<std::uint32_t>::max)() ||
            consecutiveContinuityCount == 0U ||
            schemaVersion != NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_SCHEMA_V1 ||
            extent != Extent::SCALAR_LOCAL_CONTINUITY_CERTIFICATE_RUN_ONLY ||
            !NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::ReservedIsZero(reserved))
        {
            return false;
        }
        const std::uint64_t expectedDistance =
            static_cast<std::uint64_t>(consecutiveContinuityCount) - 1ULL;
        return NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::ForwardNonZeroSequenceDistance(
            continuityRunGeneration, publicationSequence) == expectedDistance &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::ForwardNonZeroSequenceDistance(
                firstContinuityPublicationSequence, sourceContinuityPublicationSequence) == expectedDistance &&
            static_cast<std::uint64_t>(latestSharedFirstProvenTransitionCount) -
            static_cast<std::uint64_t>(firstSharedFirstProvenTransitionCount) == expectedDistance &&
            ((consecutiveContinuityCount == 1U &&
                disposition == Disposition::PROVEN_LOCAL_CONTINUITY_RUN_START &&
                relation == Relation::LOCAL_RUN_STARTED_FROM_ONE_PROVEN_CONTINUITY) ||
                (consecutiveContinuityCount > 1U &&
                    disposition == Disposition::PROVEN_LOCAL_CONTINUITY_RUN_EXTENSION &&
                    relation == Relation::DIRECT_SAME_COVERAGE_INTERVAL_CONTINUITY_RUN_EXTENDED));
    }
};

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow final
{
public:
    using Record = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1;
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunExtent;
    using ContinuityRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1;
    using ContinuityDisposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityDisposition;
    using ContinuityRelation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRelation;
    using ContinuityExtent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityExtent;

    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow() noexcept = default;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE
        void ObserveLatestQualificationTransitionPairContinuitySameThread(
            const ContinuityRecord* currentContinuity) noexcept
    {
        const Record* const previousObservation = GetNewestObservationSameThread();
        const bool previousIsProven = previousObservation != nullptr &&
            previousObservation->IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRun();
        const bool previousIsValid = previousObservation == nullptr ||
            (previousObservation->publicationSequence == m_publicationSequence &&
                (previousIsProven || IsCanonicalNonProvenRun(*previousObservation)));
        const std::uint64_t previouslyObservedSourceSequence = previousObservation == nullptr
            ? 0ULL : previousObservation->sourceContinuityPublicationSequence;
        m_publicationSequence =
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::NextNonZeroSequence(
                m_publicationSequence);
        const std::size_t targetIndex = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[targetIndex];
        ResetTarget(target, m_publicationSequence,
            currentContinuity == nullptr ? 0ULL : currentContinuity->publicationSequence);

        if (!previousIsValid)
        {
            target.disposition = Disposition::INVALID_PREVIOUS_RUN_RECORD;
        }
        else if (currentContinuity == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CONTINUITY_UNAVAILABLE;
        }
        else if (previouslyObservedSourceSequence != 0ULL &&
            currentContinuity->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::NextNonZeroSequence(
                previouslyObservedSourceSequence))
        {
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        }
        else if (IsCanonicalNonProvenContinuity(*currentContinuity))
        {
            ClassifyNonProvenContinuity(*currentContinuity, target);
        }
        else if (!currentContinuity->IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuity())
        {
            target.disposition = Disposition::INVALID_CURRENT_CONTINUITY_RECORD;
        }
        else if (!previousIsProven)
        {
            PopulateRun(*currentContinuity, nullptr, target);
        }
        else if (currentContinuity->coverageGeneration != previousObservation->coverageGeneration)
        {
            target.disposition = Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY;
        }
        else if (!DirectContinuityBindingMatches(*previousObservation, *currentContinuity))
        {
            target.disposition = Disposition::INVALID_CONTINUITY_BINDING;
        }
        else if (previousObservation->consecutiveContinuityCount ==
            (std::numeric_limits<std::uint32_t>::max)())
        {
            target.disposition = Disposition::INVALID_CONTINUITY_COUNT_OVERFLOW;
        }
        else
        {
            PopulateRun(*currentContinuity, previousObservation, target);
        }

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount < HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const Record* GetNewestObservationSameThread(std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount)
        {
            return nullptr;
        }
        const std::size_t index = (static_cast<std::size_t>(m_latestIndex) +
            HISTORY_CAPACITY - historyOffset) % HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_HISTORY_CAPACITY;

    static bool IsNeutralContinuityDisposition(ContinuityDisposition disposition) noexcept
    {
        return disposition == ContinuityDisposition::NOT_APPLICABLE_PAIR_UNAVAILABLE ||
            disposition == ContinuityDisposition::NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN_CAUSE_UNKNOWN ||
            disposition == ContinuityDisposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE ||
            disposition == ContinuityDisposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN_CAUSE_UNKNOWN ||
            disposition == ContinuityDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            disposition == ContinuityDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
    }

    static bool IsInvalidContinuityDisposition(ContinuityDisposition disposition) noexcept
    {
        return disposition == ContinuityDisposition::INVALID_CURRENT_PAIR_RECORD ||
            disposition == ContinuityDisposition::INVALID_PREVIOUS_PAIR_RECORD ||
            disposition == ContinuityDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            disposition == ContinuityDisposition::INVALID_PAIR_ADVANCE_FENCE ||
            disposition == ContinuityDisposition::INVALID_SHARED_TRANSITION_BINDING ||
            disposition == ContinuityDisposition::INVALID_PAIR_CONTINUITY;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE
        static bool IsCanonicalNonProvenContinuity(const ContinuityRecord& source) noexcept
    {
        const bool diagnosticIdentityIsValid = source.disposition ==
            ContinuityDisposition::NOT_APPLICABLE_PAIR_UNAVAILABLE
            ? source.currentPairPublicationSequence == 0ULL
            : source.currentPairPublicationSequence != 0ULL ||
            source.disposition == ContinuityDisposition::INVALID_CURRENT_PAIR_RECORD ||
            source.disposition == ContinuityDisposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        return (IsNeutralContinuityDisposition(source.disposition) ||
            IsInvalidContinuityDisposition(source.disposition)) &&
            source.publicationSequence != 0ULL && diagnosticIdentityIsValid &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_SCHEMA_V1 &&
            source.previousPairPublicationSequence == 0ULL &&
            source.sharedTransitionPublicationSequence == 0ULL &&
            source.latestSharedQualificationPublicationSequence == 0ULL &&
            source.latestSharedSummaryPublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL && source.runGeneration == 0ULL &&
            source.acceptedInputChainGeneration == 0ULL &&
            source.sharedFirstProvenTransitionCount == 0U &&
            source.sharedLastProvenTransitionCount == 0U &&
            source.relation == ContinuityRelation::NONE &&
            source.extent == ContinuityExtent::NONE &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::ReservedIsZero(source.reserved);
    }

    static void ClassifyNonProvenContinuity(const ContinuityRecord& source, Record& target) noexcept
    {
        if (!IsNeutralContinuityDisposition(source.disposition))
        {
            target.disposition = Disposition::INVALID_CURRENT_CONTINUITY_RECORD;
        }
        else if (source.disposition == ContinuityDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            source.disposition == ContinuityDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY)
        {
            target.disposition = Disposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
        }
        else
        {
            target.disposition = Disposition::NOT_APPLICABLE_CONTINUITY_NOT_PROVEN_CAUSE_UNKNOWN;
        }
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE
        static bool IsCanonicalNonProvenRun(const Record& source) noexcept
    {
        const bool dispositionIsValid =
            source.disposition == Disposition::NOT_APPLICABLE_CONTINUITY_UNAVAILABLE ||
            source.disposition == Disposition::NOT_APPLICABLE_CONTINUITY_NOT_PROVEN_CAUSE_UNKNOWN ||
            source.disposition == Disposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY ||
            source.disposition == Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            source.disposition == Disposition::INVALID_CURRENT_CONTINUITY_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_RUN_RECORD ||
            source.disposition == Disposition::INVALID_CONTINUITY_BINDING ||
            source.disposition == Disposition::INVALID_CONTINUITY_COUNT_OVERFLOW ||
            source.disposition == Disposition::INVALID_CONTINUITY_RUN;
        const bool diagnosticIdentityIsValid = source.disposition ==
            Disposition::NOT_APPLICABLE_CONTINUITY_UNAVAILABLE
            ? source.sourceContinuityPublicationSequence == 0ULL
            : source.sourceContinuityPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_CONTINUITY_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_RUN_RECORD;
        // EMPTY is valid only as untouched unpublished storage.  Once an
        // observation is exposed, EMPTY (even all zero) cannot seed a run.
        return dispositionIsValid && diagnosticIdentityIsValid &&
            source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_SCHEMA_V1 &&
            source.continuityRunGeneration == 0ULL &&
            source.firstContinuityPublicationSequence == 0ULL &&
            source.latestPairPublicationSequence == 0ULL &&
            source.sharedTransitionPublicationSequence == 0ULL &&
            source.latestSharedQualificationPublicationSequence == 0ULL &&
            source.latestSharedSummaryPublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL && source.runGeneration == 0ULL &&
            source.acceptedInputChainGeneration == 0ULL &&
            source.firstSharedFirstProvenTransitionCount == 0U &&
            source.latestSharedFirstProvenTransitionCount == 0U &&
            source.latestSharedLastProvenTransitionCount == 0U &&
            source.consecutiveContinuityCount == 0U &&
            source.relation == Relation::NONE && source.extent == Extent::NONE &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::ReservedIsZero(source.reserved);
    }

    static bool DirectContinuityBindingMatches(const Record& previous,
        const ContinuityRecord& current) noexcept
    {
        return current.runGeneration == previous.runGeneration &&
            current.acceptedInputChainGeneration == previous.acceptedInputChainGeneration &&
            current.previousPairPublicationSequence == previous.latestPairPublicationSequence &&
            current.sharedTransitionPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::NextNonZeroSequence(
                previous.sharedTransitionPublicationSequence) &&
            current.latestSharedQualificationPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::NextNonZeroSequence(
                previous.latestSharedQualificationPublicationSequence) &&
            current.latestSharedSummaryPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunDetail::NextNonZeroSequence(
                previous.latestSharedSummaryPublicationSequence) &&
            current.sharedFirstProvenTransitionCount == previous.latestSharedLastProvenTransitionCount;
    }

    static void ClearProvenPayload(Record& target) noexcept
    {
        target.continuityRunGeneration = 0ULL;
        target.firstContinuityPublicationSequence = 0ULL;
        target.latestPairPublicationSequence = 0ULL;
        target.sharedTransitionPublicationSequence = 0ULL;
        target.latestSharedQualificationPublicationSequence = 0ULL;
        target.latestSharedSummaryPublicationSequence = 0ULL;
        target.coverageGeneration = 0ULL;
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.firstSharedFirstProvenTransitionCount = 0U;
        target.latestSharedFirstProvenTransitionCount = 0U;
        target.latestSharedLastProvenTransitionCount = 0U;
        target.consecutiveContinuityCount = 0U;
        target.relation = Relation::NONE;
        target.extent = Extent::NONE;
        target.reserved.fill(0U);
    }

    static void ResetTarget(Record& target, std::uint64_t publicationSequence,
        std::uint64_t sourceContinuityPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourceContinuityPublicationSequence = sourceContinuityPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CONTINUITY_UNAVAILABLE;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE
        static void PopulateRun(const ContinuityRecord& current,
            const Record* previous, Record& target) noexcept
    {
        target.continuityRunGeneration = previous == nullptr
            ? target.publicationSequence : previous->continuityRunGeneration;
        target.firstContinuityPublicationSequence = previous == nullptr
            ? current.publicationSequence : previous->firstContinuityPublicationSequence;
        target.latestPairPublicationSequence = current.currentPairPublicationSequence;
        target.sharedTransitionPublicationSequence = current.sharedTransitionPublicationSequence;
        target.latestSharedQualificationPublicationSequence = current.latestSharedQualificationPublicationSequence;
        target.latestSharedSummaryPublicationSequence = current.latestSharedSummaryPublicationSequence;
        target.coverageGeneration = current.coverageGeneration;
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration = current.acceptedInputChainGeneration;
        target.firstSharedFirstProvenTransitionCount = previous == nullptr
            ? current.sharedFirstProvenTransitionCount : previous->firstSharedFirstProvenTransitionCount;
        target.latestSharedFirstProvenTransitionCount = current.sharedFirstProvenTransitionCount;
        target.latestSharedLastProvenTransitionCount = current.sharedLastProvenTransitionCount;
        target.consecutiveContinuityCount = previous == nullptr
            ? 1U : previous->consecutiveContinuityCount + 1U;
        target.disposition = previous == nullptr
            ? Disposition::PROVEN_LOCAL_CONTINUITY_RUN_START
            : Disposition::PROVEN_LOCAL_CONTINUITY_RUN_EXTENSION;
        target.relation = previous == nullptr
            ? Relation::LOCAL_RUN_STARTED_FROM_ONE_PROVEN_CONTINUITY
            : Relation::DIRECT_SAME_COVERAGE_INTERVAL_CONTINUITY_RUN_EXTENDED;
        target.extent = Extent::SCALAR_LOCAL_CONTINUITY_CERTIFICATE_RUN_ONLY;
        if (!target.IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRun())
        {
            ClearProvenPayload(target);
            target.disposition = Disposition::INVALID_CONTINUITY_RUN;
        }
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_NOINLINE

static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunDisposition) == 1U,
    "Qualification-transition pair continuity run disposition must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRelation) == 1U,
    "Qualification-transition pair continuity run relation must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunExtent) == 1U,
    "Qualification-transition pair continuity run extent must remain one byte.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1>::value,
    "Qualification-transition pair continuity run record must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1>::value,
    "Qualification-transition pair continuity run record must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1) == 8U,
    "Qualification-transition pair continuity run record alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1) == 112U,
    "Qualification-transition pair continuity run record must remain exactly 112 bytes.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1, firstSharedFirstProvenTransitionCount) == 88U,
    "Qualification-transition pair continuity run count block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1, schemaVersion) == 104U,
    "Qualification-transition pair continuity run schema offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1, reserved) == 109U,
    "Qualification-transition pair continuity run reserved block offset changed.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow>::value,
    "Qualification-transition pair continuity run observer must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow>::value,
    "Qualification-transition pair continuity run observer must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow) == 8U,
    "Qualification-transition pair continuity run observer alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow) == 240U,
    "Qualification-transition pair continuity run observer must remain exactly 240 bytes.");
