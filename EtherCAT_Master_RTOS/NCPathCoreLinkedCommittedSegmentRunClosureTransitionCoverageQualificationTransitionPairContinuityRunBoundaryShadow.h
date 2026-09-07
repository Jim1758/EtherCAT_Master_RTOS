#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2S / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary Shadow.
//
// Captures first/latest retained identity references only when this observer
// actually observes an L.2R local run START (count one), then every direct
// extension.  R/Q/P/O/N/M references identify the scalar projections retained
// by R; they are not full source records, geometry endpoints or snapshots.
// First references are copied from the observed head, never reconstructed
// by subtracting counts from later references.  The evolving head-to-latest
// boundary does not prove that the local run has ended or is complete.
// R's local run, and its underlying coverage interval, may themselves be
// suffixes of a committed run.  This observer does not erase that limitation.
//
// Initial attachment to an R extension, or resumption after an interruption,
// publishes HEAD_NOT_OBSERVED with no proven payload until an actual R START
// is observed.  Null, neutral, invalid, stale, gapped or misbound source
// records discard the captured head.  No missing observations are bridged.
// A direct R START immediately after a proven boundary is invalid binding:
// the trusted R producer must first publish an interruption before restarting.
// Source-reported boundaries and directly observed coverage differences are
// kept distinct from unknown causes.  Neither reconstructs missing events.
//
// Publication identities wrap, skipping zero.  All six retained R/Q/P/O/N/M
// first/latest spans must equal count minus one, as must the nonwrapping
// transition-count span.  S publication is only its own diagnostic identity;
// no S publication-span claim is made.  Freshness is local source adjacency,
// not elapsed time, permanent replay detection, authentication or protection
// against coherent forged records/source identity reuse after a full wrap.
// Null clears the source anchor; rejected source identities are diagnostic
// resynchronization anchors only.  Trusted same-thread producers are assumed.
//
// Fixed two-record scalar history, NCManager heap-owned.  No Observe-time
// allocation, large stack object, coordinates, displacement, endpoint arrays,
// nodes, segment/event list, masks, complete order or reverse-traversal data.
// Not Path/Motion Queue, planner, STARTED/DONE proof, actual-position history,
// B2 breadcrumb or execution.  Shadow-only, no control consumer; no Motion,
// G00, Gate/Registry, Alarm, PC, HMI/SHM/API, PDO, NIC, EtherCAT or DC changes;
// no logging, thread, timer, mutex, wait or sleep.

constexpr std::size_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_SCHEMA_V1 = 1U;

namespace NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail
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

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_RUN_UNAVAILABLE = 1U,
    NOT_APPLICABLE_RUN_NOT_PROVEN_CAUSE_UNKNOWN = 2U,
    NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY = 3U,
    NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED = 4U,
    NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY = 5U,
    INVALID_CURRENT_RUN_RECORD = 6U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 7U,
    INVALID_PREVIOUS_BOUNDARY_RECORD = 8U,
    INVALID_RUN_BINDING = 9U,
    INVALID_RUN_BOUNDARY = 10U,
    PROVEN_OBSERVED_RUN_HEAD = 11U,
    PROVEN_DIRECT_RUN_BOUNDARY_EXTENSION = 12U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRelation : std::uint8_t
{
    NONE = 0U,
    OBSERVED_LOCAL_RUN_HEAD = 1U,
    DIRECT_SAME_LOCAL_RUN_BOUNDARY_EXTENDED = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryExtent : std::uint8_t
{
    NONE = 0U,
    OBSERVED_LOCAL_RUN_HEAD_TO_LATEST_RETAINED_IDENTITY_REFERENCES_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1
{
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryExtent;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t firstRunPublicationSequence = 0ULL;
    std::uint64_t sourceRunPublicationSequence = 0ULL;
    std::uint64_t firstContinuityPublicationSequence = 0ULL;
    std::uint64_t latestContinuityPublicationSequence = 0ULL;
    std::uint64_t firstPairPublicationSequence = 0ULL;
    std::uint64_t latestPairPublicationSequence = 0ULL;
    std::uint64_t firstTransitionPublicationSequence = 0ULL;
    std::uint64_t latestTransitionPublicationSequence = 0ULL;
    std::uint64_t firstQualificationPublicationSequence = 0ULL;
    std::uint64_t latestQualificationPublicationSequence = 0ULL;
    std::uint64_t firstSummaryPublicationSequence = 0ULL;
    std::uint64_t latestSummaryPublicationSequence = 0ULL;
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

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundary() const noexcept
    {
        if (publicationSequence == 0ULL || firstRunPublicationSequence == 0ULL ||
            sourceRunPublicationSequence == 0ULL ||
            firstContinuityPublicationSequence == 0ULL || latestContinuityPublicationSequence == 0ULL ||
            firstPairPublicationSequence == 0ULL || latestPairPublicationSequence == 0ULL ||
            firstTransitionPublicationSequence == 0ULL || latestTransitionPublicationSequence == 0ULL ||
            firstQualificationPublicationSequence == 0ULL || latestQualificationPublicationSequence == 0ULL ||
            firstSummaryPublicationSequence == 0ULL || latestSummaryPublicationSequence == 0ULL ||
            coverageGeneration == 0ULL || runGeneration == 0ULL ||
            acceptedInputChainGeneration == 0ULL ||
            firstSharedFirstProvenTransitionCount <= 1U ||
            latestSharedFirstProvenTransitionCount < firstSharedFirstProvenTransitionCount ||
            latestSharedFirstProvenTransitionCount == (std::numeric_limits<std::uint32_t>::max)() ||
            latestSharedLastProvenTransitionCount != latestSharedFirstProvenTransitionCount + 1U ||
            latestSharedLastProvenTransitionCount == (std::numeric_limits<std::uint32_t>::max)() ||
            consecutiveContinuityCount == 0U ||
            schemaVersion != NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_SCHEMA_V1 ||
            extent != Extent::OBSERVED_LOCAL_RUN_HEAD_TO_LATEST_RETAINED_IDENTITY_REFERENCES_ONLY ||
            !NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ReservedIsZero(reserved))
        {
            return false;
        }
        const std::uint64_t expectedDistance =
            static_cast<std::uint64_t>(consecutiveContinuityCount) - 1ULL;
        return NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ForwardNonZeroSequenceDistance(
            firstRunPublicationSequence, sourceRunPublicationSequence) == expectedDistance &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ForwardNonZeroSequenceDistance(
                firstContinuityPublicationSequence, latestContinuityPublicationSequence) == expectedDistance &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ForwardNonZeroSequenceDistance(
                firstPairPublicationSequence, latestPairPublicationSequence) == expectedDistance &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ForwardNonZeroSequenceDistance(
                firstTransitionPublicationSequence, latestTransitionPublicationSequence) == expectedDistance &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ForwardNonZeroSequenceDistance(
                firstQualificationPublicationSequence, latestQualificationPublicationSequence) == expectedDistance &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ForwardNonZeroSequenceDistance(
                firstSummaryPublicationSequence, latestSummaryPublicationSequence) == expectedDistance &&
            static_cast<std::uint64_t>(latestSharedFirstProvenTransitionCount) -
            static_cast<std::uint64_t>(firstSharedFirstProvenTransitionCount) == expectedDistance &&
            ((consecutiveContinuityCount == 1U &&
                disposition == Disposition::PROVEN_OBSERVED_RUN_HEAD &&
                relation == Relation::OBSERVED_LOCAL_RUN_HEAD) ||
                (consecutiveContinuityCount > 1U &&
                    disposition == Disposition::PROVEN_DIRECT_RUN_BOUNDARY_EXTENSION &&
                    relation == Relation::DIRECT_SAME_LOCAL_RUN_BOUNDARY_EXTENDED));
    }
};

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow final
{
public:
    using Record = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1;
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryExtent;
    using RunRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRecordV1;
    using RunDisposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunDisposition;
    using RunRelation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunRelation;
    using RunExtent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunExtent;

    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow() noexcept = default;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE
        void ObserveLatestQualificationTransitionPairContinuityRunSameThread(
            const RunRecord* currentRun) noexcept
    {
        const Record* const previousObservation = GetNewestObservationSameThread();
        const bool previousIsProven = previousObservation != nullptr &&
            previousObservation->IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundary();
        const bool previousIsValid = previousObservation == nullptr ||
            (previousObservation->publicationSequence == m_publicationSequence &&
                (previousIsProven || IsCanonicalNonProvenBoundary(*previousObservation)));
        const std::uint64_t previouslyObservedSourceSequence = previousObservation == nullptr
            ? 0ULL : previousObservation->sourceRunPublicationSequence;
        m_publicationSequence =
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                m_publicationSequence);
        const std::size_t targetIndex = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[targetIndex];
        ResetTarget(target, m_publicationSequence,
            currentRun == nullptr ? 0ULL : currentRun->publicationSequence);

        if (!previousIsValid)
        {
            target.disposition = Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
        }
        else if (currentRun == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE;
        }
        else if (previouslyObservedSourceSequence != 0ULL &&
            currentRun->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                previouslyObservedSourceSequence))
        {
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        }
        else if (IsCanonicalNonProvenRun(*currentRun))
        {
            ClassifyNonProvenRun(*currentRun, target);
        }
        else if (!currentRun->IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRun())
        {
            target.disposition = Disposition::INVALID_CURRENT_RUN_RECORD;
        }
        else if (!previousIsProven)
        {
            if (currentRun->consecutiveContinuityCount == 1U)
            {
                PopulateBoundary(*currentRun, nullptr, target);
            }
            else
            {
                target.disposition = Disposition::NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED;
            }
        }
        else if (currentRun->consecutiveContinuityCount == 1U)
        {
            target.disposition = Disposition::INVALID_RUN_BINDING;
        }
        else if (currentRun->coverageGeneration != previousObservation->coverageGeneration)
        {
            target.disposition = Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY;
        }
        else if (!DirectRunBindingMatches(*previousObservation, *currentRun))
        {
            target.disposition = Disposition::INVALID_RUN_BINDING;
        }
        else
        {
            PopulateBoundary(*currentRun, previousObservation, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_HISTORY_CAPACITY;

    static bool IsNeutralRunDisposition(RunDisposition disposition) noexcept
    {
        return disposition == RunDisposition::NOT_APPLICABLE_CONTINUITY_UNAVAILABLE ||
            disposition == RunDisposition::NOT_APPLICABLE_CONTINUITY_NOT_PROVEN_CAUSE_UNKNOWN ||
            disposition == RunDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY ||
            disposition == RunDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY;
    }

    static bool IsInvalidRunDisposition(RunDisposition disposition) noexcept
    {
        return disposition == RunDisposition::INVALID_CURRENT_CONTINUITY_RECORD ||
            disposition == RunDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            disposition == RunDisposition::INVALID_PREVIOUS_RUN_RECORD ||
            disposition == RunDisposition::INVALID_CONTINUITY_BINDING ||
            disposition == RunDisposition::INVALID_CONTINUITY_COUNT_OVERFLOW ||
            disposition == RunDisposition::INVALID_CONTINUITY_RUN;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE
        static bool IsCanonicalNonProvenRun(const RunRecord& source) noexcept
    {
        const bool diagnosticIdentityIsValid = source.disposition ==
            RunDisposition::NOT_APPLICABLE_CONTINUITY_UNAVAILABLE
            ? source.sourceContinuityPublicationSequence == 0ULL
            : source.sourceContinuityPublicationSequence != 0ULL ||
            source.disposition == RunDisposition::INVALID_CURRENT_CONTINUITY_RECORD ||
            source.disposition == RunDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == RunDisposition::INVALID_PREVIOUS_RUN_RECORD;
        return (IsNeutralRunDisposition(source.disposition) ||
            IsInvalidRunDisposition(source.disposition)) &&
            diagnosticIdentityIsValid && source.publicationSequence != 0ULL &&
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
            source.relation == RunRelation::NONE && source.extent == RunExtent::NONE &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ReservedIsZero(source.reserved);
    }

    static void ClassifyNonProvenRun(const RunRecord& source, Record& target) noexcept
    {
        if (!IsNeutralRunDisposition(source.disposition))
        {
            target.disposition = Disposition::INVALID_CURRENT_RUN_RECORD;
        }
        else if (source.disposition == RunDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            source.disposition == RunDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY)
        {
            target.disposition = Disposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
        }
        else
        {
            target.disposition = Disposition::NOT_APPLICABLE_RUN_NOT_PROVEN_CAUSE_UNKNOWN;
        }
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE
        static bool IsCanonicalNonProvenBoundary(const Record& source) noexcept
    {
        const bool dispositionIsValid =
            source.disposition == Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE ||
            source.disposition == Disposition::NOT_APPLICABLE_RUN_NOT_PROVEN_CAUSE_UNKNOWN ||
            source.disposition == Disposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY ||
            source.disposition == Disposition::NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED ||
            source.disposition == Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            source.disposition == Disposition::INVALID_CURRENT_RUN_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD ||
            source.disposition == Disposition::INVALID_RUN_BINDING ||
            source.disposition == Disposition::INVALID_RUN_BOUNDARY;
        const bool diagnosticIdentityIsValid = source.disposition ==
            Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE
            ? source.sourceRunPublicationSequence == 0ULL
            : source.sourceRunPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_RUN_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
        return dispositionIsValid && diagnosticIdentityIsValid &&
            source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_SCHEMA_V1 &&
            source.firstRunPublicationSequence == 0ULL &&
            source.firstContinuityPublicationSequence == 0ULL && source.latestContinuityPublicationSequence == 0ULL &&
            source.firstPairPublicationSequence == 0ULL && source.latestPairPublicationSequence == 0ULL &&
            source.firstTransitionPublicationSequence == 0ULL && source.latestTransitionPublicationSequence == 0ULL &&
            source.firstQualificationPublicationSequence == 0ULL && source.latestQualificationPublicationSequence == 0ULL &&
            source.firstSummaryPublicationSequence == 0ULL && source.latestSummaryPublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL && source.runGeneration == 0ULL &&
            source.acceptedInputChainGeneration == 0ULL &&
            source.firstSharedFirstProvenTransitionCount == 0U &&
            source.latestSharedFirstProvenTransitionCount == 0U &&
            source.latestSharedLastProvenTransitionCount == 0U &&
            source.consecutiveContinuityCount == 0U &&
            source.relation == Relation::NONE && source.extent == Extent::NONE &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::ReservedIsZero(source.reserved);
    }

    static bool DirectRunBindingMatches(const Record& previous, const RunRecord& current) noexcept
    {
        return current.continuityRunGeneration == previous.firstRunPublicationSequence &&
            current.firstContinuityPublicationSequence == previous.firstContinuityPublicationSequence &&
            current.firstSharedFirstProvenTransitionCount == previous.firstSharedFirstProvenTransitionCount &&
            current.runGeneration == previous.runGeneration &&
            current.acceptedInputChainGeneration == previous.acceptedInputChainGeneration &&
            previous.consecutiveContinuityCount != (std::numeric_limits<std::uint32_t>::max)() &&
            current.consecutiveContinuityCount == previous.consecutiveContinuityCount + 1U &&
            current.latestSharedFirstProvenTransitionCount == previous.latestSharedLastProvenTransitionCount &&
            current.sourceContinuityPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                previous.latestContinuityPublicationSequence) &&
            current.latestPairPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                previous.latestPairPublicationSequence) &&
            current.sharedTransitionPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                previous.latestTransitionPublicationSequence) &&
            current.latestSharedQualificationPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                previous.latestQualificationPublicationSequence) &&
            current.latestSharedSummaryPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryDetail::NextNonZeroSequence(
                previous.latestSummaryPublicationSequence);
    }

    static void ClearProvenPayload(Record& target) noexcept
    {
        target.firstRunPublicationSequence = 0ULL;
        target.firstContinuityPublicationSequence = 0ULL;
        target.latestContinuityPublicationSequence = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.latestPairPublicationSequence = 0ULL;
        target.firstTransitionPublicationSequence = 0ULL;
        target.latestTransitionPublicationSequence = 0ULL;
        target.firstQualificationPublicationSequence = 0ULL;
        target.latestQualificationPublicationSequence = 0ULL;
        target.firstSummaryPublicationSequence = 0ULL;
        target.latestSummaryPublicationSequence = 0ULL;
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
        std::uint64_t sourceRunPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourceRunPublicationSequence = sourceRunPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_RUN_UNAVAILABLE;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE
        static void PopulateBoundary(const RunRecord& current, const Record* previous, Record& target) noexcept
    {
        target.firstRunPublicationSequence = previous == nullptr
            ? current.publicationSequence : previous->firstRunPublicationSequence;
        target.firstContinuityPublicationSequence = previous == nullptr
            ? current.sourceContinuityPublicationSequence : previous->firstContinuityPublicationSequence;
        target.latestContinuityPublicationSequence = current.sourceContinuityPublicationSequence;
        target.firstPairPublicationSequence = previous == nullptr
            ? current.latestPairPublicationSequence : previous->firstPairPublicationSequence;
        target.latestPairPublicationSequence = current.latestPairPublicationSequence;
        target.firstTransitionPublicationSequence = previous == nullptr
            ? current.sharedTransitionPublicationSequence : previous->firstTransitionPublicationSequence;
        target.latestTransitionPublicationSequence = current.sharedTransitionPublicationSequence;
        target.firstQualificationPublicationSequence = previous == nullptr
            ? current.latestSharedQualificationPublicationSequence : previous->firstQualificationPublicationSequence;
        target.latestQualificationPublicationSequence = current.latestSharedQualificationPublicationSequence;
        target.firstSummaryPublicationSequence = previous == nullptr
            ? current.latestSharedSummaryPublicationSequence : previous->firstSummaryPublicationSequence;
        target.latestSummaryPublicationSequence = current.latestSharedSummaryPublicationSequence;
        target.coverageGeneration = current.coverageGeneration;
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration = current.acceptedInputChainGeneration;
        target.firstSharedFirstProvenTransitionCount = previous == nullptr
            ? current.latestSharedFirstProvenTransitionCount : previous->firstSharedFirstProvenTransitionCount;
        target.latestSharedFirstProvenTransitionCount = current.latestSharedFirstProvenTransitionCount;
        target.latestSharedLastProvenTransitionCount = current.latestSharedLastProvenTransitionCount;
        target.consecutiveContinuityCount = current.consecutiveContinuityCount;
        target.disposition = previous == nullptr
            ? Disposition::PROVEN_OBSERVED_RUN_HEAD : Disposition::PROVEN_DIRECT_RUN_BOUNDARY_EXTENSION;
        target.relation = previous == nullptr
            ? Relation::OBSERVED_LOCAL_RUN_HEAD : Relation::DIRECT_SAME_LOCAL_RUN_BOUNDARY_EXTENDED;
        target.extent = Extent::OBSERVED_LOCAL_RUN_HEAD_TO_LATEST_RETAINED_IDENTITY_REFERENCES_ONLY;
        if (!target.IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundary())
        {
            ClearProvenPayload(target);
            target.disposition = Disposition::INVALID_RUN_BOUNDARY;
        }
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_NOINLINE

static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryDisposition) == 1U,
    "Qualification-transition pair continuity run boundary disposition must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRelation) == 1U,
    "Qualification-transition pair continuity run boundary relation must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryExtent) == 1U,
    "Qualification-transition pair continuity run boundary extent must remain one byte.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1>::value,
    "Qualification-transition pair continuity run boundary record must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1>::value,
    "Qualification-transition pair continuity run boundary record must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1) == 8U,
    "Qualification-transition pair continuity run boundary record alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1) == 152U,
    "Qualification-transition pair continuity run boundary record must remain exactly 152 bytes.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1, firstSharedFirstProvenTransitionCount) == 128U,
    "Qualification-transition pair continuity run boundary count block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1, schemaVersion) == 144U,
    "Qualification-transition pair continuity run boundary schema offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1, reserved) == 149U,
    "Qualification-transition pair continuity run boundary reserved block offset changed.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow>::value,
    "Qualification-transition pair continuity run boundary observer must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow>::value,
    "Qualification-transition pair continuity run boundary observer must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow) == 8U,
    "Qualification-transition pair continuity run boundary observer alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow) == 320U,
    "Qualification-transition pair continuity run boundary observer must remain exactly 320 bytes.");
