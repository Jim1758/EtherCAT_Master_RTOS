#pragma once

#include "NCPathCoreRunCoverageBoundaryShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2AB / Run Pattern Coverage Transition Pair Continuity Run
// Observed-Head Coverage Boundary Availability Transition Shadow.
//
// Describes AA boundary-certificate availability across one complete direct
// AA pair. Neutral -> observed START gains availability; available -> direct
// EXT retains it; available -> canonical neutral loses it; two canonical
// neutral records remain unavailable. All six neutral reasons are exact AA
// diagnostics, not an inferred cause or a claim about the lower run. Invalid,
// malformed, stale or gapped proof yields no transition, never ordinary loss.
// HEAD_NOT_OBSERVED cannot follow an available AA certificate. A neutral
// predecessor cannot gain availability through EXT, nor can available AA
// be replaced directly by START. A first complete available -> EXT pair may
// prove retention; a lone EXT record proves nothing about availability change.
//
// Both complete source self-proofs are checked in place during admission.
// Retention requires the same first-Y mask and all common Z/source heads,
// both counts advancing without overflow, eight direct Z/Y/X/W/V/U/T/S
// publications and shared two masks/two U relations/T relation. When both
// supplied AA records retain nonzero Z publications, those publications must
// also be adjacent. Only canonical AA CURRENT_RUN_UNAVAILABLE has a zero Z
// anchor; that observation resets this lower anchor without claiming the
// same Z interval. No additional reason or interval boundary is inferred.
//
// The latest complete AA scalar certificate is retained for exact semantic
// binding to the next supplied previous AA. This includes every nested Z
// field, all metadata, masks, relations, counts and reserved bytes. Padding
// is never compared. The other previous AA record is checked at admission
// then discarded except its identity and availability. Self-proof cannot
// reconstruct that discarded certificate or independently repeat its pair
// fences. The older AB slot is deliberately not a cumulative proof source.
// The newest AB proof or canonical diagnostic is checked before overwrite.
// Rejections clear all proof fields and retain only the current AA publication
// as a diagnostic anchor. A subsequent complete adjacent pair may recertify;
// no unavailable transition or missing event is filled in across a gap.
//
// Trusted same-thread producers are required. Coherent forged certificates,
// omitted earlier reasons/history, full-wrap identity reuse and wall-clock
// freshness are not authenticated. Availability is a boundary-certificate
// property, not geometric coverage loss, elapsed stability or a motion event.
// Exactly two 192-byte scalar records in a 400-byte heap-owned NCManager
// observer. Direct source lvalue-to-slot copy; no source stack snapshot or
// Observe-time allocation. No coordinates, displacement, endpoint arrays,
// nodes, segment/event list, complete order or reverse traversal data.
// Shadow-only: no Path/Motion Queue, planner, STARTED/DONE proof, actual
// position history, B2 breadcrumb or execution. No control consumer, log,
// thread, timer, mutex, wait or sleep. Predecessor files and control paths,
// including M00 FIX1, L.2I FIX1 and the Priority-64 PDO path, stay unchanged.

constexpr std::size_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_SCHEMA_V1 = 1U;

enum class NCPathCoreRunCoverageBoundaryAvailabilityDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE = 2U,
    SOURCE_REPORTED_CURRENT_BOUNDARY_INVALID = 3U,
    SOURCE_REPORTED_PREVIOUS_BOUNDARY_INVALID = 4U,
    INVALID_CURRENT_BOUNDARY_RECORD = 5U,
    INVALID_PREVIOUS_BOUNDARY_RECORD = 6U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 7U,
    INVALID_PREVIOUS_AVAILABILITY_RECORD = 8U,
    INVALID_PREVIOUS_SOURCE_BINDING = 9U,
    INVALID_BOUNDARY_ADVANCE = 10U,
    INVALID_SOURCE_RUN_ADVANCE = 11U,
    INVALID_SOURCE_RUN_BINDING = 12U,
    INVALID_SHARED_RUN_BINDING = 13U,
    INVALID_RUN_COUNT_OVERFLOW = 14U,
    INVALID_AVAILABILITY_STATE_BINDING = 15U,
    INVALID_AVAILABILITY_TRANSITION = 16U,
    PROVEN_AVAILABILITY_TRANSITION = 17U
};

enum class NCPathCoreRunCoverageBoundaryAvailabilityRelation : std::uint8_t
{
    NONE = 0U,
    BECAME_AVAILABLE_AT_OBSERVED_HEAD = 1U,
    RETAINED_BY_DIRECT_BOUNDARY_EXTENSION = 2U,
    BECAME_UNAVAILABLE = 3U,
    REMAINED_UNAVAILABLE = 4U
};

enum class NCPathCoreRunCoverageBoundaryAvailabilityAvailability : std::uint8_t
{
    UNKNOWN = 0U,
    UNAVAILABLE = 1U,
    AVAILABLE = 2U
};

enum class NCPathCoreRunCoverageBoundaryAvailabilityUnavailableReason : std::uint8_t
{
    NONE = 0U,
    CURRENT_RUN_UNAVAILABLE = 1U,
    PREVIOUS_RUN_UNAVAILABLE = 2U,
    CURRENT_RUN_NOT_PROVEN = 3U,
    PREVIOUS_RUN_NOT_PROVEN = 4U,
    HEAD_NOT_OBSERVED = 5U,
    DIFFERENT_RUN = 6U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
#endif

namespace NCPathCoreRunCoverageBoundaryAvailabilityDetail
{
    using BoundaryRecord = NCPathCoreRunCoverageBoundaryRecordV1;
    using BoundaryDisposition = BoundaryRecord::Disposition;
    using RunRecord = BoundaryRecord::SourceRecord;
    using UnavailableReason = NCPathCoreRunCoverageBoundaryAvailabilityUnavailableReason;
    using NCPathCoreRunCoverageBoundaryDetail::NextNonZeroSequence;

    // The nested Z publication is the sole excluded diagnostic anchor.
    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        inline bool IsZeroRunPayload(const RunRecord& source) noexcept
    {
        return source.runGeneration == 0ULL &&
            source.firstCertificatePublicationSequence == 0ULL &&
            source.currentCertificatePublicationSequence == 0ULL &&
            source.currentCoverageTransitionPublicationSequence == 0ULL &&
            source.currentCoveragePublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL &&
            source.sourceRunPublicationSequence == 0ULL &&
            source.continuityRunGeneration == 0ULL &&
            source.firstPairPublicationSequence == 0ULL &&
            source.sourcePairPublicationSequence == 0ULL &&
            source.firstTransitionPublicationSequence == 0ULL &&
            source.currentTransitionPublicationSequence == 0ULL &&
            source.firstBoundaryPublicationSequence == 0ULL &&
            source.currentBoundaryPublicationSequence == 0ULL &&
            source.consecutivePairCount == 0U &&
            source.consecutiveCertificateCount == 0U &&
            source.firstSourcePairCount == 0U &&
            source.schemaVersion == 0U &&
            source.disposition == RunRecord::Disposition::EMPTY &&
            source.previousObservedPairRelationMask == 0U &&
            source.currentObservedPairRelationMask == 0U &&
            source.newlyObservedPairRelationMask == 0U &&
            source.retainedPairRelationMask == 0U &&
            source.fullCoverageTransition == RunRecord::FullCoverageTransition::NONE &&
            source.firstPairRelation == RunRecord::PairRelation::NONE &&
            source.sourcePairRelation == RunRecord::PairRelation::NONE &&
            source.previousSourcePairRelation == RunRecord::PairRelation::NONE &&
            source.previousTransitionRelation == RunRecord::TransitionRelation::NONE &&
            source.currentTransitionRelation == RunRecord::TransitionRelation::NONE &&
            source.currentUnavailableReason == RunRecord::UnavailableReason::NONE &&
            source.earlierObservedPairRelationMask == 0U &&
            source.earlierSourcePairRelation == RunRecord::PairRelation::NONE &&
            source.discoveryPairRelation == RunRecord::DiscoveryPairRelation::NONE &&
            source.fullCoveragePairRelation == RunRecord::FullCoveragePairRelation::NONE &&
            source.reserved[0U] == 0U && source.reserved[1U] == 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        inline bool IsCanonicalNonProvenBoundary(const BoundaryRecord& source) noexcept
    {
        const bool known = source.disposition >= BoundaryDisposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE &&
            source.disposition <= BoundaryDisposition::INVALID_COVERAGE_BOUNDARY;
        const bool identity = source.disposition == BoundaryDisposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE
            ? source.currentRun.publicationSequence == 0ULL
            : source.currentRun.publicationSequence != 0ULL ||
            source.disposition == BoundaryDisposition::INVALID_CURRENT_RUN_RECORD ||
            source.disposition == BoundaryDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == BoundaryDisposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
        return known && identity && source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_SCHEMA_V1 &&
            IsZeroRunPayload(source.currentRun) && source.firstYCoverageMask == 0U &&
            source.gainedSinceFirstYMask == 0U &&
            source.fullCoverageRelation == BoundaryRecord::FullCoverageRelation::NONE &&
            source.reserved[0U] == 0U && source.reserved[1U] == 0U;
    }

    inline bool IsNeutralBoundaryDisposition(BoundaryDisposition disposition) noexcept
    {
        return disposition >= BoundaryDisposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE &&
            disposition <= BoundaryDisposition::NOT_APPLICABLE_DIFFERENT_RUN;
    }

    inline UnavailableReason MapUnavailableReason(BoundaryDisposition disposition) noexcept
    {
        switch (disposition)
        {
        case BoundaryDisposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE: return UnavailableReason::CURRENT_RUN_UNAVAILABLE;
        case BoundaryDisposition::NOT_APPLICABLE_PREVIOUS_RUN_UNAVAILABLE: return UnavailableReason::PREVIOUS_RUN_UNAVAILABLE;
        case BoundaryDisposition::NOT_APPLICABLE_CURRENT_RUN_NOT_PROVEN: return UnavailableReason::CURRENT_RUN_NOT_PROVEN;
        case BoundaryDisposition::NOT_APPLICABLE_PREVIOUS_RUN_NOT_PROVEN: return UnavailableReason::PREVIOUS_RUN_NOT_PROVEN;
        case BoundaryDisposition::NOT_APPLICABLE_HEAD_NOT_OBSERVED: return UnavailableReason::HEAD_NOT_OBSERVED;
        case BoundaryDisposition::NOT_APPLICABLE_DIFFERENT_RUN: return UnavailableReason::DIFFERENT_RUN;
        default: return UnavailableReason::NONE;
        }
    }
}

struct NCPathCoreRunCoverageBoundaryAvailabilityRecordV1
{
    using Disposition = NCPathCoreRunCoverageBoundaryAvailabilityDisposition;
    using Relation = NCPathCoreRunCoverageBoundaryAvailabilityRelation;
    using Availability = NCPathCoreRunCoverageBoundaryAvailabilityAvailability;
    using UnavailableReason = NCPathCoreRunCoverageBoundaryAvailabilityUnavailableReason;
    using BoundaryRecord = NCPathCoreRunCoverageBoundaryAvailabilityDetail::BoundaryRecord;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousBoundaryPublicationSequence = 0ULL;
    BoundaryRecord currentBoundary{};
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    Availability previousAvailability = Availability::UNKNOWN;
    Availability currentAvailability = Availability::UNKNOWN;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::uint8_t reserved = 0U;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        bool IsProvenRunCoverageBoundaryAvailabilityTransition() const noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryAvailabilityDetail;
        if (publicationSequence == 0ULL || previousBoundaryPublicationSequence == 0ULL ||
            currentBoundary.publicationSequence != NextNonZeroSequence(previousBoundaryPublicationSequence) ||
            schemaVersion != NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_SCHEMA_V1 ||
            disposition != Disposition::PROVEN_AVAILABILITY_TRANSITION || reserved != 0U) return false;
        if (currentBoundary.IsProvenRunCoverageBoundary())
        {
            if (currentAvailability != Availability::AVAILABLE || currentUnavailableReason != UnavailableReason::NONE)
                return false;
            return (relation == Relation::BECAME_AVAILABLE_AT_OBSERVED_HEAD &&
                previousAvailability == Availability::UNAVAILABLE &&
                currentBoundary.disposition == BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START) ||
                (relation == Relation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION &&
                    previousAvailability == Availability::AVAILABLE &&
                    currentBoundary.disposition == BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION);
        }
        if (!IsCanonicalNonProvenBoundary(currentBoundary) ||
            !IsNeutralBoundaryDisposition(currentBoundary.disposition) ||
            currentAvailability != Availability::UNAVAILABLE ||
            currentUnavailableReason != MapUnavailableReason(currentBoundary.disposition)) return false;
        return (relation == Relation::BECAME_UNAVAILABLE && previousAvailability == Availability::AVAILABLE &&
            currentUnavailableReason != UnavailableReason::HEAD_NOT_OBSERVED) ||
            (relation == Relation::REMAINED_UNAVAILABLE && previousAvailability == Availability::UNAVAILABLE);
    }
};

class NCPathCoreRunCoverageBoundaryAvailabilityShadow final
{
public:
    using Record = NCPathCoreRunCoverageBoundaryAvailabilityRecordV1;
    using Disposition = Record::Disposition;
    using Relation = Record::Relation;
    using Availability = Record::Availability;
    using UnavailableReason = Record::UnavailableReason;
    using BoundaryRecord = Record::BoundaryRecord;
    using BoundaryDisposition = BoundaryRecord::Disposition;
    using RunRecord = BoundaryRecord::SourceRecord;

    NCPathCoreRunCoverageBoundaryAvailabilityShadow() noexcept = default;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        void ObserveLatestRunCoverageBoundaryAvailabilitySameThread(const BoundaryRecord* previousBoundary,
            const BoundaryRecord* currentBoundary) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenRunCoverageBoundaryAvailabilityTransition();
        // This local transition has no cumulative recurrence in the older slot.
        // Validate the newest record before clearing the independent older slot.
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            (previousProven || IsCanonicalNonProvenAvailability(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentBoundary.publicationSequence;
        m_publicationSequence = NCPathCoreRunCoverageBoundaryAvailabilityDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentBoundary == nullptr ? 0ULL : currentBoundary->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_AVAILABILITY_RECORD;
        else if (currentBoundary == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE;
        else if (anchor != 0ULL && currentBoundary->publicationSequence !=
            NCPathCoreRunCoverageBoundaryAvailabilityDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluatePair(previousBoundary, *currentBoundary, previous, previousProven, anchor, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_HISTORY_CAPACITY;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        static bool IsCanonicalNonProvenAvailability(const Record& record) noexcept
    {
        const bool known = record.disposition >= Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE &&
            record.disposition <= Disposition::INVALID_AVAILABILITY_TRANSITION;
        const bool identity = record.disposition == Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE
            ? record.currentBoundary.publicationSequence == 0ULL
            : record.currentBoundary.publicationSequence != 0ULL ||
            record.disposition == Disposition::INVALID_CURRENT_BOUNDARY_RECORD ||
            record.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            record.disposition == Disposition::INVALID_PREVIOUS_AVAILABILITY_RECORD;
        const BoundaryRecord& boundary = record.currentBoundary;
        return known && identity && record.publicationSequence != 0ULL &&
            record.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_SCHEMA_V1 &&
            record.previousBoundaryPublicationSequence == 0ULL && record.relation == Relation::NONE &&
            record.previousAvailability == Availability::UNKNOWN && record.currentAvailability == Availability::UNKNOWN &&
            record.currentUnavailableReason == UnavailableReason::NONE && record.reserved == 0U &&
            boundary.schemaVersion == 0U && boundary.disposition == BoundaryDisposition::EMPTY &&
            boundary.currentRun.publicationSequence == 0ULL &&
            NCPathCoreRunCoverageBoundaryAvailabilityDetail::IsZeroRunPayload(boundary.currentRun) &&
            boundary.firstYCoverageMask == 0U && boundary.gainedSinceFirstYMask == 0U &&
            boundary.fullCoverageRelation == BoundaryRecord::FullCoverageRelation::NONE &&
            boundary.reserved[0U] == 0U && boundary.reserved[1U] == 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        static bool PreviousSourceMatches(const BoundaryRecord& source, const BoundaryRecord& retained) noexcept
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

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
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

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
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

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        static void EvaluatePair(const BoundaryRecord* previousBoundary, const BoundaryRecord& currentBoundary,
            const Record* previous, bool previousProven, std::uint64_t anchor, Record& target) noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryAvailabilityDetail;
        const bool currentAvailable = currentBoundary.IsProvenRunCoverageBoundary();
        if (!currentAvailable)
        {
            if (!IsCanonicalNonProvenBoundary(currentBoundary))
            {
                target.disposition = Disposition::INVALID_CURRENT_BOUNDARY_RECORD;
                return;
            }
            if (!IsNeutralBoundaryDisposition(currentBoundary.disposition))
            {
                target.disposition = Disposition::SOURCE_REPORTED_CURRENT_BOUNDARY_INVALID;
                return;
            }
        }
        if (previousBoundary == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE;
            return;
        }
        const bool previousAvailable = previousBoundary->IsProvenRunCoverageBoundary();
        if (!previousAvailable)
        {
            if (!IsCanonicalNonProvenBoundary(*previousBoundary))
            {
                target.disposition = Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
                return;
            }
            if (!IsNeutralBoundaryDisposition(previousBoundary->disposition))
            {
                target.disposition = Disposition::SOURCE_REPORTED_PREVIOUS_BOUNDARY_INVALID;
                return;
            }
        }
        if (currentBoundary.publicationSequence != NextNonZeroSequence(previousBoundary->publicationSequence))
        {
            target.disposition = Disposition::INVALID_BOUNDARY_ADVANCE;
            return;
        }
        if ((anchor != 0ULL && previousBoundary->publicationSequence != anchor) ||
            (previousProven && previous != nullptr && !PreviousSourceMatches(*previousBoundary, previous->currentBoundary)))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        const RunRecord& previousRun = previousBoundary->currentRun;
        const RunRecord& currentRun = currentBoundary.currentRun;
        if (previousRun.publicationSequence != 0ULL && currentRun.publicationSequence != 0ULL &&
            currentRun.publicationSequence != NextNonZeroSequence(previousRun.publicationSequence))
        {
            target.disposition = Disposition::INVALID_SOURCE_RUN_ADVANCE;
            return;
        }
        if (currentAvailable)
        {
            if ((previousAvailable && currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION) ||
                (!previousAvailable && currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START))
            {
                target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
                return;
            }
            if (previousAvailable)
            {
                if (currentBoundary.firstYCoverageMask != previousBoundary->firstYCoverageMask ||
                    !SameSourceRun(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_BINDING;
                    return;
                }
                if (previousRun.consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)() ||
                    previousRun.consecutiveCertificateCount >= (std::numeric_limits<std::uint32_t>::max)() - 3U)
                {
                    target.disposition = Disposition::INVALID_RUN_COUNT_OVERFLOW;
                    return;
                }
                if (!DirectRunAdvance(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_ADVANCE;
                    return;
                }
                if (!SharedRunBinding(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SHARED_RUN_BINDING;
                    return;
                }
            }
        }
        else if (previousAvailable && currentBoundary.disposition == BoundaryDisposition::NOT_APPLICABLE_HEAD_NOT_OBSERVED)
        {
            target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
            return;
        }
        BuildTransition(*previousBoundary, currentBoundary, previousAvailable, currentAvailable, target);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        static void BuildTransition(const BoundaryRecord& previousBoundary, const BoundaryRecord& currentBoundary,
            bool previousAvailable, bool currentAvailable, Record& target) noexcept
    {
        target.previousBoundaryPublicationSequence = previousBoundary.publicationSequence;
        target.currentBoundary = currentBoundary; // Direct lvalue-to-slot copy; no AA/Z temporary.
        target.previousAvailability = previousAvailable ? Availability::AVAILABLE : Availability::UNAVAILABLE;
        target.currentAvailability = currentAvailable ? Availability::AVAILABLE : Availability::UNAVAILABLE;
        target.currentUnavailableReason = currentAvailable ? UnavailableReason::NONE :
            NCPathCoreRunCoverageBoundaryAvailabilityDetail::MapUnavailableReason(currentBoundary.disposition);
        target.relation = currentAvailable ? (previousAvailable ? Relation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION :
            Relation::BECAME_AVAILABLE_AT_OBSERVED_HEAD) :
            (previousAvailable ? Relation::BECAME_UNAVAILABLE : Relation::REMAINED_UNAVAILABLE);
        target.disposition = Disposition::PROVEN_AVAILABILITY_TRANSITION;
        if (!target.IsProvenRunCoverageBoundaryAvailabilityTransition())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_AVAILABILITY_TRANSITION;
        }
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        // Preserve only currentBoundary.publicationSequence as diagnostic identity.
        RunRecord& source = target.currentBoundary.currentRun;
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
        target.currentBoundary.schemaVersion = 0U;
        target.currentBoundary.disposition = BoundaryDisposition::EMPTY;
        target.currentBoundary.firstYCoverageMask = 0U;
        target.currentBoundary.gainedSinceFirstYMask = 0U;
        target.currentBoundary.fullCoverageRelation = BoundaryRecord::FullCoverageRelation::NONE;
        for (std::uint8_t& value : target.currentBoundary.reserved) value = 0U;
        target.previousBoundaryPublicationSequence = 0ULL;
        target.relation = Relation::NONE;
        target.previousAvailability = Availability::UNKNOWN;
        target.currentAvailability = Availability::UNKNOWN;
        target.currentUnavailableReason = UnavailableReason::NONE;
        target.reserved = 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentBoundary.publicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_NOINLINE

static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityDisposition) == 1U, "AB disposition one byte.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityRelation) == 1U, "AB relation one byte.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityAvailability) == 1U, "AB availability one byte.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityUnavailableReason) == 1U, "AB reason one byte.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryAvailabilityRecordV1>::value, "AB record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryAvailabilityRecordV1>::value, "AB record scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1) == 192U, "AB record exactly 192 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1) == 8U, "AB record alignment.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, previousBoundaryPublicationSequence) == 8U, "AB previous AA identity offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, currentBoundary) == 16U, "AB retained AA offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, schemaVersion) == 184U, "AB schema offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, disposition) == 186U, "AB disposition offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, relation) == 187U, "AB relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, previousAvailability) == 188U, "AB previous state offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, currentAvailability) == 189U, "AB current state offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, currentUnavailableReason) == 190U, "AB reason offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityRecordV1, reserved) == 191U, "AB reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryAvailabilityShadow>::value, "AB observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryAvailabilityShadow>::value, "AB observer scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityShadow) == 400U, "AB observer exactly 400 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryAvailabilityShadow) == 8U, "AB observer alignment.");
