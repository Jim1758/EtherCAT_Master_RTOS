#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2U / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Pair Continuity Shadow.
//
// Joins two directly adjacent proven T certificates on the same shared S
// publication and availability state. The eight relations describe only
// this local pair. LOSS_THEN_GAIN does not prove restoration of the same
// run, coverage interval or boundary certificate. T retains no such context.
// T also omits its previous S unavailable reason: equality of that shared
// reason across the two T records cannot be independently proved here.
// Only the latest S unavailable reason is retained, exactly as reported by T.
//
// Both supplied T records must pass their complete self-proof. U checks T
// adjacency, shared S identity/state, its own publication counter and its
// retained latest T projection. That projection reconstructs every semantic
// field of the previous T record (canonical schema/disposition/reserved;
// availability derived from relation); no source snapshot is retained.
// These checks assume trusted same-thread producers. Coherent forgery,
// full-wrap identity reuse, complete source reconstruction and wall-clock
// freshness are outside this contract. No authentication is claimed.
//
// Null, canonical neutral, source-reported invalid, malformed and stale input
// never become a loss/gain pair. A complete direct pair may establish an
// initial local proof. A rejection is only a diagnostic resynchronization
// anchor; a later pair cannot claim continuity through the rejected input.
// Null clears that source anchor. No duration or cumulative count is stored.
//
// Exactly two 48-byte scalar records, 112-byte NCManager heap-owned observer.
// No Observe-time allocation, large stack copy, coordinates, displacement,
// endpoint arrays, node/segment/event list, full order or reverse traversal.
// Not a Path/Motion Queue, planner, STARTED/DONE proof, actual-position
// history, B2 breadcrumb or B2 execution. Shadow-only; no control consumer.
// No Motion, G00, Gate/Registry, Alarm, PC, HMI/SHM/API or PDO/DC changes;
// no log, thread, timer, mutex, wait or sleep. Existing M00 FIX1 is preserved.

constexpr std::size_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_SCHEMA_V1 = 1U;

enum class NCPathCoreBoundaryAvailabilityTransitionPairDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE = 2U,
    NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN = 3U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN = 4U,
    SOURCE_REPORTED_CURRENT_TRANSITION_INVALID = 5U,
    SOURCE_REPORTED_PREVIOUS_TRANSITION_INVALID = 6U,
    INVALID_CURRENT_TRANSITION_RECORD = 7U,
    INVALID_PREVIOUS_TRANSITION_RECORD = 8U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 9U,
    INVALID_PREVIOUS_PAIR_RECORD = 10U,
    INVALID_PREVIOUS_SOURCE_BINDING = 11U,
    INVALID_TRANSITION_PAIR_ADVANCE = 12U,
    INVALID_SHARED_BOUNDARY_BINDING = 13U,
    INVALID_SHARED_AVAILABILITY_BINDING = 14U,
    INVALID_PAIR_CONTINUITY = 15U,
    PROVEN_DIRECT_TRANSITION_PAIR_CONTINUITY = 16U
};

enum class NCPathCoreBoundaryAvailabilityTransitionPairRelation : std::uint8_t
{
    NONE = 0U,
    GAIN_THEN_RETAIN = 1U,
    GAIN_THEN_LOSS = 2U,
    RETAIN_THEN_RETAIN = 3U,
    RETAIN_THEN_LOSS = 4U,
    LOSS_THEN_GAIN = 5U,
    LOSS_THEN_UNAVAILABLE = 6U,
    UNAVAILABLE_THEN_GAIN = 7U,
    UNAVAILABLE_THEN_UNAVAILABLE = 8U
};

namespace NCPathCoreBoundaryAvailabilityTransitionPairDetail
{
    using SourceRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1;
    using SourceRelation = SourceRecord::Relation;
    using Availability = SourceRecord::Availability;
    using Reason = SourceRecord::UnavailableReason;
    using PairRelation = NCPathCoreBoundaryAvailabilityTransitionPairRelation;

    constexpr std::uint64_t NextNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)() ? 1ULL : value + 1ULL;
    }

    inline Availability PreviousAvailability(SourceRelation relation) noexcept
    {
        if (relation == SourceRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD ||
            relation == SourceRelation::REMAINED_UNAVAILABLE) return Availability::UNAVAILABLE;
        if (relation == SourceRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION ||
            relation == SourceRelation::BECAME_UNAVAILABLE) return Availability::AVAILABLE;
        return Availability::UNKNOWN;
    }

    inline Availability CurrentAvailability(SourceRelation relation) noexcept
    {
        if (relation == SourceRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD ||
            relation == SourceRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION) return Availability::AVAILABLE;
        if (relation == SourceRelation::BECAME_UNAVAILABLE ||
            relation == SourceRelation::REMAINED_UNAVAILABLE) return Availability::UNAVAILABLE;
        return Availability::UNKNOWN;
    }

    inline PairRelation Classify(SourceRelation previous, SourceRelation current) noexcept
    {
        if (previous == SourceRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD)
        {
            if (current == SourceRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION) return PairRelation::GAIN_THEN_RETAIN;
            if (current == SourceRelation::BECAME_UNAVAILABLE) return PairRelation::GAIN_THEN_LOSS;
        }
        else if (previous == SourceRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION)
        {
            if (current == SourceRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION) return PairRelation::RETAIN_THEN_RETAIN;
            if (current == SourceRelation::BECAME_UNAVAILABLE) return PairRelation::RETAIN_THEN_LOSS;
        }
        else if (previous == SourceRelation::BECAME_UNAVAILABLE)
        {
            if (current == SourceRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD) return PairRelation::LOSS_THEN_GAIN;
            if (current == SourceRelation::REMAINED_UNAVAILABLE) return PairRelation::LOSS_THEN_UNAVAILABLE;
        }
        else if (previous == SourceRelation::REMAINED_UNAVAILABLE)
        {
            if (current == SourceRelation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD) return PairRelation::UNAVAILABLE_THEN_GAIN;
            if (current == SourceRelation::REMAINED_UNAVAILABLE) return PairRelation::UNAVAILABLE_THEN_UNAVAILABLE;
        }
        return PairRelation::NONE;
    }

    inline bool CurrentReasonMatches(SourceRelation relation, Reason reason) noexcept
    {
        if (CurrentAvailability(relation) == Availability::AVAILABLE) return reason == Reason::NONE;
        const bool unavailable = reason == Reason::SOURCE_RUN_UNAVAILABLE ||
            reason == Reason::SOURCE_RUN_NOT_PROVEN_CAUSE_UNKNOWN ||
            reason == Reason::SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY ||
            reason == Reason::RUN_HEAD_NOT_OBSERVED || reason == Reason::DIRECT_COVERAGE_INTERVAL_BOUNDARY;
        if (!unavailable) return false;
        if (relation == SourceRelation::BECAME_UNAVAILABLE) return reason != Reason::RUN_HEAD_NOT_OBSERVED;
        return relation == SourceRelation::REMAINED_UNAVAILABLE && reason != Reason::DIRECT_COVERAGE_INTERVAL_BOUNDARY;
    }
}

#if defined(_MSC_VER)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
#endif

struct NCPathCoreBoundaryAvailabilityTransitionPairRecordV1
{
    using Disposition = NCPathCoreBoundaryAvailabilityTransitionPairDisposition;
    using Relation = NCPathCoreBoundaryAvailabilityTransitionPairRelation;
    using TransitionRelation = NCPathCoreBoundaryAvailabilityTransitionPairDetail::SourceRelation;
    using UnavailableReason = NCPathCoreBoundaryAvailabilityTransitionPairDetail::Reason;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousTransitionPublicationSequence = 0ULL;
    std::uint64_t currentTransitionPublicationSequence = 0ULL;
    std::uint64_t sharedBoundaryPublicationSequence = 0ULL;
    std::uint64_t currentBoundaryPublicationSequence = 0ULL;
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    TransitionRelation previousTransitionRelation = TransitionRelation::NONE;
    TransitionRelation currentTransitionRelation = TransitionRelation::NONE;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::uint8_t reserved = 0U;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
        bool IsProvenBoundaryAvailabilityTransitionPairContinuity() const noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairDetail;
        return publicationSequence != 0ULL && previousTransitionPublicationSequence != 0ULL &&
            currentTransitionPublicationSequence == NextNonZeroSequence(previousTransitionPublicationSequence) &&
            sharedBoundaryPublicationSequence != 0ULL &&
            currentBoundaryPublicationSequence == NextNonZeroSequence(sharedBoundaryPublicationSequence) &&
            schemaVersion == NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_SCHEMA_V1 &&
            disposition == Disposition::PROVEN_DIRECT_TRANSITION_PAIR_CONTINUITY && reserved == 0U &&
            relation != Relation::NONE && relation == Classify(previousTransitionRelation, currentTransitionRelation) &&
            CurrentReasonMatches(currentTransitionRelation, currentUnavailableReason);
    }
};

class NCPathCoreBoundaryAvailabilityTransitionPairShadow final
{
public:
    using Record = NCPathCoreBoundaryAvailabilityTransitionPairRecordV1;
    using Disposition = Record::Disposition;
    using Relation = Record::Relation;
    using TransitionRelation = Record::TransitionRelation;
    using UnavailableReason = Record::UnavailableReason;
    using TransitionRecord = NCPathCoreBoundaryAvailabilityTransitionPairDetail::SourceRecord;
    using TransitionDisposition = TransitionRecord::Disposition;

    NCPathCoreBoundaryAvailabilityTransitionPairShadow() noexcept = default;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
        void ObserveImmediateBoundaryAvailabilityTransitionPairSameThread(
            const TransitionRecord* previousTransition, const TransitionRecord* currentTransition) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenBoundaryAvailabilityTransitionPairContinuity();
        const bool previousValid = previous == nullptr ||
            (previous->publicationSequence == m_publicationSequence &&
                (previousProven || IsCanonicalNonProvenPair(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentTransitionPublicationSequence;
        m_publicationSequence = NCPathCoreBoundaryAvailabilityTransitionPairDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentTransition == nullptr ? 0ULL : currentTransition->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        else if (currentTransition == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE;
        else if (anchor != 0ULL && currentTransition->publicationSequence !=
            NCPathCoreBoundaryAvailabilityTransitionPairDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluatePair(previousTransition, *currentTransition, previous, previousProven, anchor, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_HISTORY_CAPACITY;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
        static bool IsCanonicalNonProvenTransition(const TransitionRecord& source) noexcept
    {
        using D = TransitionDisposition;
        const bool known = source.disposition == D::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE ||
            source.disposition == D::NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE ||
            source.disposition == D::INVALID_CURRENT_BOUNDARY_RECORD || source.disposition == D::INVALID_PREVIOUS_BOUNDARY_RECORD ||
            source.disposition == D::INVALID_OBSERVER_SOURCE_ADVANCE || source.disposition == D::INVALID_PREVIOUS_TRANSITION_RECORD ||
            source.disposition == D::INVALID_PREVIOUS_SOURCE_BINDING || source.disposition == D::INVALID_BOUNDARY_PAIR_ADVANCE ||
            source.disposition == D::INVALID_BOUNDARY_STATE_BINDING || source.disposition == D::INVALID_AVAILABILITY_TRANSITION;
        const bool identity = source.disposition == D::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE
            ? source.currentBoundaryPublicationSequence == 0ULL
            : source.currentBoundaryPublicationSequence != 0ULL || source.disposition == D::INVALID_CURRENT_BOUNDARY_RECORD ||
            source.disposition == D::INVALID_OBSERVER_SOURCE_ADVANCE || source.disposition == D::INVALID_PREVIOUS_TRANSITION_RECORD;
        return known && identity && source.publicationSequence != 0ULL && source.previousBoundaryPublicationSequence == 0ULL &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_SCHEMA_V1 &&
            source.relation == TransitionRelation::NONE && source.previousAvailability == TransitionRecord::Availability::UNKNOWN &&
            source.currentAvailability == TransitionRecord::Availability::UNKNOWN &&
            source.currentUnavailableReason == UnavailableReason::NONE && source.reserved == 0U;
    }

    static bool IsNeutralTransition(const TransitionRecord& source) noexcept
    {
        return source.disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE ||
            source.disposition == TransitionDisposition::NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
        static bool IsCanonicalNonProvenPair(const Record& source) noexcept
    {
        const bool known = source.disposition >= Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            source.disposition <= Disposition::INVALID_PAIR_CONTINUITY;
        const bool identity = source.disposition == Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE
            ? source.currentTransitionPublicationSequence == 0ULL
            : source.currentTransitionPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_TRANSITION_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        return known && identity && source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_SCHEMA_V1 &&
            source.previousTransitionPublicationSequence == 0ULL && source.sharedBoundaryPublicationSequence == 0ULL &&
            source.currentBoundaryPublicationSequence == 0ULL && source.relation == Relation::NONE &&
            source.previousTransitionRelation == TransitionRelation::NONE && source.currentTransitionRelation == TransitionRelation::NONE &&
            source.currentUnavailableReason == UnavailableReason::NONE && source.reserved == 0U;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
        static bool PreviousSourceMatches(const TransitionRecord& source, const Record& previous) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairDetail;
        return source.publicationSequence == previous.currentTransitionPublicationSequence &&
            source.previousBoundaryPublicationSequence == previous.sharedBoundaryPublicationSequence &&
            source.currentBoundaryPublicationSequence == previous.currentBoundaryPublicationSequence &&
            source.relation == previous.currentTransitionRelation &&
            source.previousAvailability == PreviousAvailability(previous.currentTransitionRelation) &&
            source.currentAvailability == CurrentAvailability(previous.currentTransitionRelation) &&
            source.currentUnavailableReason == previous.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE
        static void EvaluatePair(const TransitionRecord* previousSource, const TransitionRecord& current,
            const Record* previousObservation, bool previousProven, std::uint64_t anchor, Record& target) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairDetail;
        if (!current.IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransition())
        {
            target.disposition = !IsCanonicalNonProvenTransition(current) ? Disposition::INVALID_CURRENT_TRANSITION_RECORD :
                IsNeutralTransition(current) ? Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_CURRENT_TRANSITION_INVALID;
            return;
        }
        if (previousSource == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE;
            return;
        }
        if (!previousSource->IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransition())
        {
            target.disposition = !IsCanonicalNonProvenTransition(*previousSource) ? Disposition::INVALID_PREVIOUS_TRANSITION_RECORD :
                IsNeutralTransition(*previousSource) ? Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_PREVIOUS_TRANSITION_INVALID;
            return;
        }
        if (current.publicationSequence != NextNonZeroSequence(previousSource->publicationSequence))
        {
            target.disposition = Disposition::INVALID_TRANSITION_PAIR_ADVANCE;
            return;
        }
        if ((anchor != 0ULL && previousSource->publicationSequence != anchor) ||
            (previousProven && previousObservation != nullptr && !PreviousSourceMatches(*previousSource, *previousObservation)))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (previousSource->currentBoundaryPublicationSequence != current.previousBoundaryPublicationSequence)
        {
            target.disposition = Disposition::INVALID_SHARED_BOUNDARY_BINDING;
            return;
        }
        if (previousSource->currentAvailability != current.previousAvailability)
        {
            target.disposition = Disposition::INVALID_SHARED_AVAILABILITY_BINDING;
            return;
        }
        target.previousTransitionPublicationSequence = previousSource->publicationSequence;
        target.sharedBoundaryPublicationSequence = current.previousBoundaryPublicationSequence;
        target.currentBoundaryPublicationSequence = current.currentBoundaryPublicationSequence;
        target.previousTransitionRelation = previousSource->relation;
        target.currentTransitionRelation = current.relation;
        target.currentUnavailableReason = current.currentUnavailableReason;
        target.relation = Classify(previousSource->relation, current.relation);
        target.disposition = Disposition::PROVEN_DIRECT_TRANSITION_PAIR_CONTINUITY;
        if (!target.IsProvenBoundaryAvailabilityTransitionPairContinuity())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_PAIR_CONTINUITY;
        }
    }

    static void ClearPayload(Record& target) noexcept
    {
        target.previousTransitionPublicationSequence = 0ULL;
        target.sharedBoundaryPublicationSequence = 0ULL;
        target.currentBoundaryPublicationSequence = 0ULL;
        target.relation = Relation::NONE;
        target.previousTransitionRelation = TransitionRelation::NONE;
        target.currentTransitionRelation = TransitionRelation::NONE;
        target.currentUnavailableReason = UnavailableReason::NONE;
        target.reserved = 0U;
    }

    static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentTransitionPublicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_NOINLINE

static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairDisposition) == 1U, "U disposition must be one byte.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRelation) == 1U, "U relation must be one byte.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRecordV1>::value, "U record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRecordV1>::value, "U record scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRecordV1) == 48U, "U record exactly 48 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRecordV1) == 8U, "U record alignment.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRecordV1, schemaVersion) == 40U, "U schema offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRecordV1, currentUnavailableReason) == 46U, "U reason offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRecordV1, reserved) == 47U, "U reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairShadow>::value, "U observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairShadow>::value, "U observer scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairShadow) == 112U, "U observer exactly 112 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairShadow) == 8U, "U observer alignment.");
