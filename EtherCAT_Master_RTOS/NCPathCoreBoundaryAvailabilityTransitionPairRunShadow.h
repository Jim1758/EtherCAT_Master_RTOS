#pragma once

#include "NCPathCoreBoundaryAvailabilityTransitionPairShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2V / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Pair Continuity Run-Length Shadow.
//
// Counts only consecutive, directly overlapping, self-proven U pair
// certificates observed here. A complete first pair of U records starts a
// local run of two U certificates. Extension binds every semantic field of
// the supplied previous U to the exact latest-U projection retained here.
// Four zero-skipping publication spans independently bind the V run, U,
// current T and current S heads to their current identities and count.
// The first T/S identities are the current T/S references carried by the
// first counted U, not its earliest underlying T/S endpoint or geometry.
//
// This is not a count of segments, time, a stable qualification interval,
// or one committed/coverage run. U may prove pairs containing availability
// losses and gains; this count does not restore discarded run context.
// Proven U pairs may also cross a source-reported coverage boundary without
// interrupting certificate continuity. No stable AVAILABLE state is implied.
// U omits the unavailable reason of its previous T certificate. Therefore
// the shared T unavailable reason cannot independently be compared across
// supplied U records. The shared T/S identities and relation are checked;
// the complete latest U projection is bound on each extension. Trusted
// same-thread producers are required. Coherent forgery, full-wrap identity
// reuse and wall-clock freshness are outside this contract.
//
// Null, neutral, invalid, malformed, stale and mismatched input do not extend
// a run. Every rejection clears the proven payload and retains at most the
// current U publication as a diagnostic resynchronization anchor. Null
// clears that anchor. The next complete direct pair may start only a new
// local suffix, never bridge the interruption. Count overflow is rejected;
// it is neither saturated nor silently wrapped into a new proven run.
//
// Exactly two 80-byte scalar records, 176-byte NCManager heap-owned observer.
// No Observe-time allocation, large source/stack snapshot, coordinates,
// displacement, endpoint arrays, intermediate nodes, segment/event list,
// complete order, reverse traversal or unbounded history. Not a Path/Motion
// Queue, planner, STARTED/DONE proof, actual-position history, B2 breadcrumb
// or B2 execution. Shadow-only; no control consumer. No Motion, G00,
// Gate/Registry, Alarm, PC, HMI/SHM/API or PDO/DC changes; no log, thread,
// timer, mutex, wait or sleep. Existing M00 FIX1 and filenames are preserved.

constexpr std::size_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_SCHEMA_V1 = 1U;

enum class NCPathCoreBoundaryAvailabilityTransitionPairRunDisposition : std::uint8_t
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
    INVALID_PAIR_COUNT_OVERFLOW = 14U,
    INVALID_CONTINUITY_RUN = 15U,
    PROVEN_LOCAL_PAIR_RUN_START = 16U,
    PROVEN_LOCAL_PAIR_RUN_EXTENSION = 17U
};

namespace NCPathCoreBoundaryAvailabilityTransitionPairRunDetail
{
    using SourceRecord = NCPathCoreBoundaryAvailabilityTransitionPairRecordV1;
    using PairRelation = SourceRecord::Relation;
    using TransitionRelation = SourceRecord::TransitionRelation;
    using Reason = SourceRecord::UnavailableReason;

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
}

#if defined(_MSC_VER)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
#endif

struct NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1
{
    using Disposition = NCPathCoreBoundaryAvailabilityTransitionPairRunDisposition;
    using PairRelation = NCPathCoreBoundaryAvailabilityTransitionPairRunDetail::PairRelation;
    using TransitionRelation = NCPathCoreBoundaryAvailabilityTransitionPairRunDetail::TransitionRelation;
    using UnavailableReason = NCPathCoreBoundaryAvailabilityTransitionPairRunDetail::Reason;

    std::uint64_t publicationSequence = 0ULL;
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
    PairRelation sourcePairRelation = PairRelation::NONE;
    TransitionRelation previousTransitionRelation = TransitionRelation::NONE;
    TransitionRelation currentTransitionRelation = TransitionRelation::NONE;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::array<std::uint8_t, 5U> reserved{};

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        bool IsProvenBoundaryAvailabilityTransitionPairRun() const noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunDetail;
        if (publicationSequence == 0ULL || continuityRunGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL || sourcePairPublicationSequence == 0ULL ||
            firstTransitionPublicationSequence == 0ULL || currentTransitionPublicationSequence == 0ULL ||
            firstBoundaryPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            schemaVersion != NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_SCHEMA_V1 ||
            consecutivePairCount < 2U ||
            !((disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_START && consecutivePairCount == 2U) ||
                (disposition == Disposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION && consecutivePairCount > 2U)) ||
            sourcePairRelation == PairRelation::NONE ||
            sourcePairRelation != NCPathCoreBoundaryAvailabilityTransitionPairDetail::Classify(
                previousTransitionRelation, currentTransitionRelation) ||
            !NCPathCoreBoundaryAvailabilityTransitionPairDetail::CurrentReasonMatches(
                currentTransitionRelation, currentUnavailableReason)) return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        const std::uint64_t sourceDistance = static_cast<std::uint64_t>(consecutivePairCount) - 1ULL;
        return ForwardNonZeroSequenceDistance(continuityRunGeneration, publicationSequence) == sourceDistance - 1ULL &&
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, sourcePairPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstTransitionPublicationSequence, currentTransitionPublicationSequence) == sourceDistance &&
            ForwardNonZeroSequenceDistance(firstBoundaryPublicationSequence, currentBoundaryPublicationSequence) == sourceDistance;
    }
};

class NCPathCoreBoundaryAvailabilityTransitionPairRunShadow final
{
public:
    using Record = NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1;
    using Disposition = Record::Disposition;
    using PairRelation = Record::PairRelation;
    using TransitionRelation = Record::TransitionRelation;
    using UnavailableReason = Record::UnavailableReason;
    using PairRecord = NCPathCoreBoundaryAvailabilityTransitionPairRunDetail::SourceRecord;
    using PairDisposition = PairRecord::Disposition;

    NCPathCoreBoundaryAvailabilityTransitionPairRunShadow() noexcept = default;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        void ObserveImmediateBoundaryAvailabilityTransitionPairRunSameThread(
            const PairRecord* previousPair, const PairRecord* currentPair) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenBoundaryAvailabilityTransitionPairRun();
        const bool previousValid = previous == nullptr ||
            (previous->publicationSequence == m_publicationSequence &&
                (previousProven || IsCanonicalNonProvenRun(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->sourcePairPublicationSequence;
        m_publicationSequence = NCPathCoreBoundaryAvailabilityTransitionPairRunDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentPair == nullptr ? 0ULL : currentPair->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_RUN_RECORD;
        else if (currentPair == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
        else if (anchor != 0ULL && currentPair->publicationSequence !=
            NCPathCoreBoundaryAvailabilityTransitionPairRunDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluateRun(previousPair, *currentPair, previous, previousProven, anchor, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_HISTORY_CAPACITY;

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsCanonicalNonProvenPair(const PairRecord& source) noexcept
    {
        using D = PairDisposition;
        const bool known = source.disposition >= D::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            source.disposition <= D::INVALID_PAIR_CONTINUITY;
        const bool identity = source.disposition == D::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE
            ? source.currentTransitionPublicationSequence == 0ULL
            : source.currentTransitionPublicationSequence != 0ULL ||
            source.disposition == D::INVALID_CURRENT_TRANSITION_RECORD ||
            source.disposition == D::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == D::INVALID_PREVIOUS_PAIR_RECORD;
        return known && identity && source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_SCHEMA_V1 &&
            source.previousTransitionPublicationSequence == 0ULL && source.sharedBoundaryPublicationSequence == 0ULL &&
            source.currentBoundaryPublicationSequence == 0ULL && source.relation == PairRelation::NONE &&
            source.previousTransitionRelation == TransitionRelation::NONE && source.currentTransitionRelation == TransitionRelation::NONE &&
            source.currentUnavailableReason == UnavailableReason::NONE && source.reserved == 0U;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsNeutralPair(const PairRecord& source) noexcept
    {
        return source.disposition >= PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            source.disposition <= PairDisposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static bool IsCanonicalNonProvenRun(const Record& source) noexcept
    {
        const bool known = source.disposition >= Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            source.disposition <= Disposition::INVALID_CONTINUITY_RUN;
        const bool identity = source.disposition == Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE
            ? source.sourcePairPublicationSequence == 0ULL
            : source.sourcePairPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_PAIR_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_RUN_RECORD;
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

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static bool PreviousSourceMatches(const PairRecord& source, const Record& previous) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunDetail;
        return source.publicationSequence == previous.sourcePairPublicationSequence &&
            source.previousTransitionPublicationSequence == PreviousNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            source.currentTransitionPublicationSequence == previous.currentTransitionPublicationSequence &&
            source.sharedBoundaryPublicationSequence == PreviousNonZeroSequence(previous.currentBoundaryPublicationSequence) &&
            source.currentBoundaryPublicationSequence == previous.currentBoundaryPublicationSequence &&
            source.relation == previous.sourcePairRelation &&
            source.previousTransitionRelation == previous.previousTransitionRelation &&
            source.currentTransitionRelation == previous.currentTransitionRelation &&
            source.currentUnavailableReason == previous.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static void EvaluateRun(const PairRecord* previousSource, const PairRecord& current,
            const Record* previousObservation, bool previousProven, std::uint64_t anchor, Record& target) noexcept
    {
        using namespace NCPathCoreBoundaryAvailabilityTransitionPairRunDetail;
        if (!current.IsProvenBoundaryAvailabilityTransitionPairContinuity())
        {
            target.disposition = !IsCanonicalNonProvenPair(current) ? Disposition::INVALID_CURRENT_PAIR_RECORD :
                IsNeutralPair(current) ? Disposition::NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_CURRENT_PAIR_INVALID;
            return;
        }
        if (previousSource == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE;
            return;
        }
        if (!previousSource->IsProvenBoundaryAvailabilityTransitionPairContinuity())
        {
            target.disposition = !IsCanonicalNonProvenPair(*previousSource) ? Disposition::INVALID_PREVIOUS_PAIR_RECORD :
                IsNeutralPair(*previousSource) ? Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_PREVIOUS_PAIR_INVALID;
            return;
        }
        if (current.publicationSequence != NextNonZeroSequence(previousSource->publicationSequence))
        {
            target.disposition = Disposition::INVALID_PAIR_ADVANCE;
            return;
        }
        if ((anchor != 0ULL && previousSource->publicationSequence != anchor) ||
            (previousProven && previousObservation != nullptr && !PreviousSourceMatches(*previousSource, *previousObservation)))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (previousSource->currentTransitionPublicationSequence != current.previousTransitionPublicationSequence ||
            previousSource->currentBoundaryPublicationSequence != current.sharedBoundaryPublicationSequence ||
            previousSource->currentTransitionRelation != current.previousTransitionRelation)
        {
            target.disposition = Disposition::INVALID_SHARED_TRANSITION_BINDING;
            return;
        }
        if (previousProven && previousObservation != nullptr &&
            previousObservation->consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)())
        {
            target.disposition = Disposition::INVALID_PAIR_COUNT_OVERFLOW;
            return;
        }
        BuildRun(*previousSource, current, previousObservation, previousProven, target);
        if (!target.IsProvenBoundaryAvailabilityTransitionPairRun())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_CONTINUITY_RUN;
        }
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static void BuildRun(const PairRecord& previousSource, const PairRecord& current,
            const Record* previousObservation, bool previousProven, Record& target) noexcept
    {
        if (previousProven && previousObservation != nullptr)
        {
            target.continuityRunGeneration = previousObservation->continuityRunGeneration;
            target.firstPairPublicationSequence = previousObservation->firstPairPublicationSequence;
            target.firstTransitionPublicationSequence = previousObservation->firstTransitionPublicationSequence;
            target.firstBoundaryPublicationSequence = previousObservation->firstBoundaryPublicationSequence;
            target.consecutivePairCount = previousObservation->consecutivePairCount + 1U;
            target.disposition = Disposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION;
        }
        else
        {
            target.continuityRunGeneration = target.publicationSequence;
            target.firstPairPublicationSequence = previousSource.publicationSequence;
            target.firstTransitionPublicationSequence = previousSource.currentTransitionPublicationSequence;
            target.firstBoundaryPublicationSequence = previousSource.currentBoundaryPublicationSequence;
            target.consecutivePairCount = 2U;
            target.disposition = Disposition::PROVEN_LOCAL_PAIR_RUN_START;
        }
        target.currentTransitionPublicationSequence = current.currentTransitionPublicationSequence;
        target.currentBoundaryPublicationSequence = current.currentBoundaryPublicationSequence;
        target.sourcePairRelation = current.relation;
        target.previousTransitionRelation = current.previousTransitionRelation;
        target.currentTransitionRelation = current.currentTransitionRelation;
        target.currentUnavailableReason = current.currentUnavailableReason;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        target.continuityRunGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.firstTransitionPublicationSequence = 0ULL;
        target.currentTransitionPublicationSequence = 0ULL;
        target.firstBoundaryPublicationSequence = 0ULL;
        target.currentBoundaryPublicationSequence = 0ULL;
        target.consecutivePairCount = 0U;
        target.sourcePairRelation = PairRelation::NONE;
        target.previousTransitionRelation = TransitionRelation::NONE;
        target.currentTransitionRelation = TransitionRelation::NONE;
        target.currentUnavailableReason = UnavailableReason::NONE;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.sourcePairPublicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_BOUNDARY_AVAILABILITY_TRANSITION_PAIR_RUN_NOINLINE

static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunDisposition) == 1U, "V disposition must be one byte.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1>::value, "V record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1>::value, "V record scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1) == 80U, "V record exactly 80 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1) == 8U, "V record alignment.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1, consecutivePairCount) == 64U, "V count offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1, schemaVersion) == 68U, "V schema offset.");
static_assert(offsetof(NCPathCoreBoundaryAvailabilityTransitionPairRunRecordV1, reserved) == 75U, "V reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreBoundaryAvailabilityTransitionPairRunShadow>::value, "V observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreBoundaryAvailabilityTransitionPairRunShadow>::value, "V observer scalar copyability.");
static_assert(sizeof(NCPathCoreBoundaryAvailabilityTransitionPairRunShadow) == 176U, "V observer exactly 176 bytes.");
static_assert(alignof(NCPathCoreBoundaryAvailabilityTransitionPairRunShadow) == 8U, "V observer alignment.");
