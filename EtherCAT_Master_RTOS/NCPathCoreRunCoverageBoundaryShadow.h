#pragma once

#include "NCPathCoreRunCoverageTransitionPairRunShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2AA / Run Pattern Coverage Transition Pair Continuity Run
// Observed-Head Coverage Boundary Shadow.
//
// Attaches only when the current Z record is an actually observed local
// START. Its previous coverage mask is the first counted Y certificate's
// current coverage mask, established by Z's two-Y overlap proof. This mask
// may already contain patterns observed before the local Z suffix began.
// No omitted first-Y classification, reason, coordinate or event is inferred.
//
// Extensions retain that first-Y mask and the latest complete scalar Z
// certificate. Eight direct Z/Y/X/W/V/U/T/S publications, both counts,
// common source heads, full previous-Z field binding, and the shared two
// masks/two U relations/T relation are required. Before overwriting a slot,
// a retained AA extension must also pass the two-slot source recurrence.
// Z self-proof runs in place; no source record is materialized on the stack.
//
// Full coverage compares the first-Y/current-Y boundaries for all eight U
// availability-transition pair relation categories, not axes, endpoint
// closure, qualification, path completion or current/stable availability.
// FULL_TO_FULL does not claim all eight patterns occurred within this Z run.
// Gained bits mean categories added to cumulative coverage since the first
// counted Y mask; they do not identify their occurrence or complete order.
// A gap/rejection clears proof. An arbitrary Z extension cannot reattach;
// the observer waits for a new current Z START, even if previous Z is START.
// AA may remain unavailable while Z continues; no source restart is forced.
// A rejected record retains only current Z publication as a diagnostic
// anchor, cleared on null. Rejection never describes normal coverage loss.
//
// Trusted same-thread sources are required. Omitted shared source reasons
// remain unknown. Structurally coherent forged masks/certificates, discarded
// history, full-wrap identity reuse and wall-clock freshness are not proven.
// Exactly two 168-byte scalar records in a 352-byte heap-owned NCManager
// observer. No Observe-time allocation, coordinates, displacement, endpoint
// arrays, nodes, segment/event list, reverse traversal or complete history.
// Shadow-only; no Path/Motion Queue, planner, Motion STARTED/DONE proof,
// actual-position history, B2 breadcrumb or execution. No control consumer,
// logging, thread, timer, mutex, wait or sleep; existing control paths remain
// unchanged. No predecessor header, M00 FIX1 or L.2I FIX1 replacement.

constexpr std::size_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_SCHEMA_V1 = 1U;

enum class NCPathCoreRunCoverageBoundaryDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_RUN_UNAVAILABLE = 2U,
    NOT_APPLICABLE_CURRENT_RUN_NOT_PROVEN = 3U,
    NOT_APPLICABLE_PREVIOUS_RUN_NOT_PROVEN = 4U,
    NOT_APPLICABLE_HEAD_NOT_OBSERVED = 5U,
    NOT_APPLICABLE_DIFFERENT_RUN = 6U,
    SOURCE_REPORTED_CURRENT_RUN_INVALID = 7U,
    SOURCE_REPORTED_PREVIOUS_RUN_INVALID = 8U,
    INVALID_CURRENT_RUN_RECORD = 9U,
    INVALID_PREVIOUS_RUN_RECORD = 10U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 11U,
    INVALID_PREVIOUS_BOUNDARY_RECORD = 12U,
    INVALID_PREVIOUS_SOURCE_BINDING = 13U,
    INVALID_RUN_ADVANCE = 14U,
    INVALID_SHARED_RUN_BINDING = 15U,
    INVALID_RUN_COUNT_OVERFLOW = 16U,
    INVALID_COVERAGE_BOUNDARY = 17U,
    PROVEN_OBSERVED_HEAD_BOUNDARY_START = 18U,
    PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION = 19U
};

enum class NCPathCoreRunCoverageBoundaryFullCoverageRelation : std::uint8_t
{
    NONE = 0U,
    PARTIAL_TO_PARTIAL = 1U,
    PARTIAL_TO_FULL = 2U,
    FULL_TO_FULL = 3U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
#endif

namespace NCPathCoreRunCoverageBoundaryDetail
{
    using SourceRecord = NCPathCoreRunCoverageTransitionPairRunRecordV1;
    using SourceDisposition = SourceRecord::Disposition;
    using FullCoverageRelation = NCPathCoreRunCoverageBoundaryFullCoverageRelation;
    using NCPathCoreRunCoverageTransitionPairRunDetail::NextNonZeroSequence;
    using NCPathCoreRunCoverageTransitionPairRunDetail::PairRelationBit;
    using NCPathCoreRunCoverageTransitionPairRunDetail::PopulationCount;

    inline FullCoverageRelation ClassifyFullCoverage(std::uint8_t firstMask,
        std::uint8_t currentMask) noexcept
    {
        if (firstMask == 0xFFU)
            return currentMask == 0xFFU ? FullCoverageRelation::FULL_TO_FULL : FullCoverageRelation::NONE;
        return currentMask == 0xFFU ? FullCoverageRelation::PARTIAL_TO_FULL : FullCoverageRelation::PARTIAL_TO_PARTIAL;
    }
}

struct NCPathCoreRunCoverageBoundaryRecordV1
{
    using Disposition = NCPathCoreRunCoverageBoundaryDisposition;
    using SourceRecord = NCPathCoreRunCoverageBoundaryDetail::SourceRecord;
    using FullCoverageRelation = NCPathCoreRunCoverageBoundaryFullCoverageRelation;

    std::uint64_t publicationSequence = 0ULL;
    SourceRecord currentRun{};
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    std::uint8_t firstYCoverageMask = 0U;
    std::uint8_t gainedSinceFirstYMask = 0U;
    FullCoverageRelation fullCoverageRelation = FullCoverageRelation::NONE;
    std::array<std::uint8_t, 2U> reserved{};

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        bool IsProvenRunCoverageBoundary() const noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryDetail;
        if (publicationSequence == 0ULL || schemaVersion != NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_SCHEMA_V1 ||
            !currentRun.IsProvenRunCoverageTransitionPairRun() ||
            !((disposition == Disposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START &&
                currentRun.disposition == SourceDisposition::PROVEN_LOCAL_PAIR_RUN_START) ||
                (disposition == Disposition::PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION &&
                    currentRun.disposition == SourceDisposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION))) return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        const std::uint8_t firstBit = PairRelationBit(currentRun.firstPairRelation);
        const std::uint8_t outsideFirst = static_cast<std::uint8_t>(~firstYCoverageMask);
        const std::uint32_t count = currentRun.consecutiveCertificateCount;
        if (firstBit == 0U || (firstYCoverageMask & firstBit) != firstBit ||
            (firstYCoverageMask & currentRun.previousObservedPairRelationMask) != firstYCoverageMask ||
            PopulationCount(firstYCoverageMask) > currentRun.firstSourcePairCount ||
            gainedSinceFirstYMask != static_cast<std::uint8_t>(currentRun.currentObservedPairRelationMask & outsideFirst) ||
            PopulationCount(gainedSinceFirstYMask) > count - 1U ||
            PopulationCount(static_cast<std::uint8_t>(currentRun.previousObservedPairRelationMask & outsideFirst)) > count - 2U ||
            (count == 2U && firstYCoverageMask != currentRun.previousObservedPairRelationMask) ||
            fullCoverageRelation != ClassifyFullCoverage(firstYCoverageMask, currentRun.currentObservedPairRelationMask))
            return false;
        return count < 3U ||
            ((firstYCoverageMask & currentRun.earlierObservedPairRelationMask) == firstYCoverageMask &&
                PopulationCount(static_cast<std::uint8_t>(currentRun.earlierObservedPairRelationMask & outsideFirst)) <= count - 3U &&
                (count != 3U || firstYCoverageMask == currentRun.earlierObservedPairRelationMask));
    }
};

class NCPathCoreRunCoverageBoundaryShadow final
{
public:
    using Record = NCPathCoreRunCoverageBoundaryRecordV1;
    using Disposition = Record::Disposition;
    using SourceRecord = Record::SourceRecord;
    using SourceDisposition = SourceRecord::Disposition;
    using FullCoverageRelation = Record::FullCoverageRelation;

    NCPathCoreRunCoverageBoundaryShadow() noexcept = default;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        void ObserveLatestRunCoverageBoundarySameThread(const SourceRecord* previousRun,
            const SourceRecord* currentRun) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenRunCoverageBoundary();
        // Both retained slots must be checked before clearing the overwrite slot.
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            ((previousProven && (previous->disposition == Disposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START ||
                IsRetainedBoundaryRecurrence(GetNewestObservationSameThread(1U), *previous))) ||
                IsCanonicalNonProvenBoundary(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentRun.publicationSequence;
        m_publicationSequence = NCPathCoreRunCoverageBoundaryDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentRun == nullptr ? 0ULL : currentRun->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
        else if (currentRun == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE;
        else if (anchor != 0ULL && currentRun->publicationSequence !=
            NCPathCoreRunCoverageBoundaryDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluateBoundary(previousRun, *currentRun, previous, previousProven, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_HISTORY_CAPACITY;

    // This predicate excludes the source's own publication, diagnostic Y
    // publication, schema and disposition. Every other source field is zero.
    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool IsZeroSourcePayload(const SourceRecord& source) noexcept
    {
        if (source.runGeneration != 0ULL || source.firstCertificatePublicationSequence != 0ULL ||
            source.currentCoverageTransitionPublicationSequence != 0ULL || source.currentCoveragePublicationSequence != 0ULL ||
            source.coverageGeneration != 0ULL || source.sourceRunPublicationSequence != 0ULL ||
            source.continuityRunGeneration != 0ULL || source.firstPairPublicationSequence != 0ULL ||
            source.sourcePairPublicationSequence != 0ULL || source.firstTransitionPublicationSequence != 0ULL ||
            source.currentTransitionPublicationSequence != 0ULL || source.firstBoundaryPublicationSequence != 0ULL ||
            source.currentBoundaryPublicationSequence != 0ULL || source.consecutivePairCount != 0U ||
            source.consecutiveCertificateCount != 0U || source.firstSourcePairCount != 0U ||
            source.previousObservedPairRelationMask != 0U || source.currentObservedPairRelationMask != 0U ||
            source.newlyObservedPairRelationMask != 0U || source.retainedPairRelationMask != 0U ||
            source.fullCoverageTransition != SourceRecord::FullCoverageTransition::NONE ||
            source.firstPairRelation != SourceRecord::PairRelation::NONE || source.sourcePairRelation != SourceRecord::PairRelation::NONE ||
            source.previousSourcePairRelation != SourceRecord::PairRelation::NONE ||
            source.previousTransitionRelation != SourceRecord::TransitionRelation::NONE ||
            source.currentTransitionRelation != SourceRecord::TransitionRelation::NONE ||
            source.currentUnavailableReason != SourceRecord::UnavailableReason::NONE ||
            source.earlierObservedPairRelationMask != 0U || source.earlierSourcePairRelation != SourceRecord::PairRelation::NONE ||
            source.discoveryPairRelation != SourceRecord::DiscoveryPairRelation::NONE ||
            source.fullCoveragePairRelation != SourceRecord::FullCoveragePairRelation::NONE) return false;
        for (std::uint8_t value : source.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool IsCanonicalNonProvenRun(const SourceRecord& source) noexcept
    {
        const bool known = source.disposition >= SourceDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            source.disposition <= SourceDisposition::INVALID_CONTINUITY_RUN;
        const bool identity = source.disposition == SourceDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE
            ? source.currentCertificatePublicationSequence == 0ULL
            : source.currentCertificatePublicationSequence != 0ULL ||
            source.disposition == SourceDisposition::INVALID_CURRENT_PAIR_RECORD ||
            source.disposition == SourceDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == SourceDisposition::INVALID_PREVIOUS_RUN_RECORD;
        return known && identity && source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_TRANSITION_PAIR_RUN_SCHEMA_V1 && IsZeroSourcePayload(source);
    }

    static bool IsNeutralRunDisposition(SourceDisposition disposition) noexcept
    {
        return disposition >= SourceDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            disposition <= SourceDisposition::NOT_APPLICABLE_DIFFERENT_COVERAGE_RUN;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool IsCanonicalNonProvenBoundary(const Record& record) noexcept
    {
        const bool known = record.disposition >= Disposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE &&
            record.disposition <= Disposition::INVALID_COVERAGE_BOUNDARY;
        const bool identity = record.disposition == Disposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE
            ? record.currentRun.publicationSequence == 0ULL
            : record.currentRun.publicationSequence != 0ULL ||
            record.disposition == Disposition::INVALID_CURRENT_RUN_RECORD ||
            record.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            record.disposition == Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
        if (!known || !identity || record.publicationSequence == 0ULL ||
            record.schemaVersion != NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_SCHEMA_V1 ||
            record.currentRun.currentCertificatePublicationSequence != 0ULL || record.currentRun.schemaVersion != 0U ||
            record.currentRun.disposition != SourceDisposition::EMPTY || !IsZeroSourcePayload(record.currentRun) ||
            record.firstYCoverageMask != 0U || record.gainedSinceFirstYMask != 0U ||
            record.fullCoverageRelation != FullCoverageRelation::NONE) return false;
        for (std::uint8_t value : record.reserved) if (value != 0U) return false;
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool PreviousSourceMatches(const SourceRecord& source, const SourceRecord& retained) noexcept
    {
        // Field equality includes canonical metadata; padding is never read.
        return source.publicationSequence == retained.publicationSequence &&
            source.runGeneration == retained.runGeneration &&
            source.firstCertificatePublicationSequence == retained.firstCertificatePublicationSequence &&
            source.currentCertificatePublicationSequence == retained.currentCertificatePublicationSequence &&
            source.currentCoverageTransitionPublicationSequence == retained.currentCoverageTransitionPublicationSequence &&
            source.currentCoveragePublicationSequence == retained.currentCoveragePublicationSequence &&
            source.coverageGeneration == retained.coverageGeneration &&
            source.sourceRunPublicationSequence == retained.sourceRunPublicationSequence &&
            source.continuityRunGeneration == retained.continuityRunGeneration &&
            source.firstPairPublicationSequence == retained.firstPairPublicationSequence &&
            source.sourcePairPublicationSequence == retained.sourcePairPublicationSequence &&
            source.firstTransitionPublicationSequence == retained.firstTransitionPublicationSequence &&
            source.currentTransitionPublicationSequence == retained.currentTransitionPublicationSequence &&
            source.firstBoundaryPublicationSequence == retained.firstBoundaryPublicationSequence &&
            source.currentBoundaryPublicationSequence == retained.currentBoundaryPublicationSequence &&
            source.consecutivePairCount == retained.consecutivePairCount &&
            source.consecutiveCertificateCount == retained.consecutiveCertificateCount &&
            source.firstSourcePairCount == retained.firstSourcePairCount &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.previousObservedPairRelationMask == retained.previousObservedPairRelationMask &&
            source.currentObservedPairRelationMask == retained.currentObservedPairRelationMask &&
            source.newlyObservedPairRelationMask == retained.newlyObservedPairRelationMask &&
            source.retainedPairRelationMask == retained.retainedPairRelationMask &&
            source.fullCoverageTransition == retained.fullCoverageTransition &&
            source.firstPairRelation == retained.firstPairRelation && source.sourcePairRelation == retained.sourcePairRelation &&
            source.previousSourcePairRelation == retained.previousSourcePairRelation &&
            source.previousTransitionRelation == retained.previousTransitionRelation &&
            source.currentTransitionRelation == retained.currentTransitionRelation &&
            source.currentUnavailableReason == retained.currentUnavailableReason &&
            source.earlierObservedPairRelationMask == retained.earlierObservedPairRelationMask &&
            source.earlierSourcePairRelation == retained.earlierSourcePairRelation &&
            source.discoveryPairRelation == retained.discoveryPairRelation &&
            source.fullCoveragePairRelation == retained.fullCoveragePairRelation &&
            source.reserved[0U] == retained.reserved[0U] && source.reserved[1U] == retained.reserved[1U];
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool SameSourceRun(const SourceRecord& previous, const SourceRecord& current) noexcept
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

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool DirectRunAdvance(const SourceRecord& previous, const SourceRecord& current) noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryDetail;
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

    static bool SharedRunBinding(const SourceRecord& previous, const SourceRecord& current) noexcept
    {
        return current.earlierObservedPairRelationMask == previous.previousObservedPairRelationMask &&
            current.previousObservedPairRelationMask == previous.currentObservedPairRelationMask &&
            current.earlierSourcePairRelation == previous.previousSourcePairRelation &&
            current.previousSourcePairRelation == previous.sourcePairRelation &&
            current.previousTransitionRelation == previous.currentTransitionRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static bool IsRetainedBoundaryRecurrence(const Record* older, const Record& newest) noexcept
    {
        return older != nullptr && older->IsProvenRunCoverageBoundary() &&
            newest.publicationSequence == NCPathCoreRunCoverageBoundaryDetail::NextNonZeroSequence(older->publicationSequence) &&
            newest.firstYCoverageMask == older->firstYCoverageMask &&
            SameSourceRun(older->currentRun, newest.currentRun) &&
            DirectRunAdvance(older->currentRun, newest.currentRun) &&
            SharedRunBinding(older->currentRun, newest.currentRun);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static void EvaluateBoundary(const SourceRecord* previousRun, const SourceRecord& currentRun,
            const Record* previous, bool previousProven, Record& target) noexcept
    {
        if (!currentRun.IsProvenRunCoverageTransitionPairRun())
        {
            target.disposition = !IsCanonicalNonProvenRun(currentRun) ? Disposition::INVALID_CURRENT_RUN_RECORD :
                IsNeutralRunDisposition(currentRun.disposition) ? Disposition::NOT_APPLICABLE_CURRENT_RUN_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_CURRENT_RUN_INVALID;
            return;
        }
        if (currentRun.disposition == SourceDisposition::PROVEN_LOCAL_PAIR_RUN_START)
        {
            // A trusted Z producer reports a nonproven record before a new
            // local START. A START cannot replace an uninterrupted AA proof.
            if (previousProven)
            {
                target.disposition = Disposition::INVALID_RUN_ADVANCE;
                return;
            }
            // Only an actual current START establishes the observed head.
            BuildBoundary(currentRun, currentRun.previousObservedPairRelationMask,
                Disposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START, target);
            return;
        }
        if (previousRun == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_RUN_UNAVAILABLE;
            return;
        }
        if (!previousRun->IsProvenRunCoverageTransitionPairRun())
        {
            target.disposition = !IsCanonicalNonProvenRun(*previousRun) ? Disposition::INVALID_PREVIOUS_RUN_RECORD :
                IsNeutralRunDisposition(previousRun->disposition) ? Disposition::NOT_APPLICABLE_PREVIOUS_RUN_NOT_PROVEN :
                Disposition::SOURCE_REPORTED_PREVIOUS_RUN_INVALID;
            return;
        }
        if (previousProven && previous != nullptr && !PreviousSourceMatches(*previousRun, previous->currentRun))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (!SameSourceRun(*previousRun, currentRun))
        {
            target.disposition = Disposition::NOT_APPLICABLE_DIFFERENT_RUN;
            return;
        }
        if (previousRun->consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)() ||
            previousRun->consecutiveCertificateCount >= (std::numeric_limits<std::uint32_t>::max)() - 3U)
        {
            target.disposition = Disposition::INVALID_RUN_COUNT_OVERFLOW;
            return;
        }
        if (!DirectRunAdvance(*previousRun, currentRun))
        {
            target.disposition = Disposition::INVALID_RUN_ADVANCE;
            return;
        }
        if (!SharedRunBinding(*previousRun, currentRun))
        {
            target.disposition = Disposition::INVALID_SHARED_RUN_BINDING;
            return;
        }
        if (!previousProven || previous == nullptr)
        {
            // Missing-head status requires an otherwise fully linked EXT pair.
            target.disposition = Disposition::NOT_APPLICABLE_HEAD_NOT_OBSERVED;
            return;
        }
        BuildBoundary(currentRun, previous->firstYCoverageMask,
            Disposition::PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION, target);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static void BuildBoundary(const SourceRecord& source, std::uint8_t firstMask,
            Disposition disposition, Record& target) noexcept
    {
        target.currentRun = source; // Direct lvalue-to-slot copy; no source stack snapshot.
        target.firstYCoverageMask = firstMask;
        target.gainedSinceFirstYMask = static_cast<std::uint8_t>(source.currentObservedPairRelationMask &
            static_cast<std::uint8_t>(~firstMask));
        target.fullCoverageRelation = NCPathCoreRunCoverageBoundaryDetail::ClassifyFullCoverage(
            firstMask, source.currentObservedPairRelationMask);
        target.disposition = disposition;
        if (!target.IsProvenRunCoverageBoundary())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_COVERAGE_BOUNDARY;
        }
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        SourceRecord& source = target.currentRun;
        // Preserve only source.publicationSequence as diagnostic identity.
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
        source.disposition = SourceDisposition::EMPTY;
        source.previousObservedPairRelationMask = 0U;
        source.currentObservedPairRelationMask = 0U;
        source.newlyObservedPairRelationMask = 0U;
        source.retainedPairRelationMask = 0U;
        source.fullCoverageTransition = SourceRecord::FullCoverageTransition::NONE;
        source.firstPairRelation = SourceRecord::PairRelation::NONE;
        source.sourcePairRelation = SourceRecord::PairRelation::NONE;
        source.previousSourcePairRelation = SourceRecord::PairRelation::NONE;
        source.previousTransitionRelation = SourceRecord::TransitionRelation::NONE;
        source.currentTransitionRelation = SourceRecord::TransitionRelation::NONE;
        source.currentUnavailableReason = SourceRecord::UnavailableReason::NONE;
        source.earlierObservedPairRelationMask = 0U;
        source.earlierSourcePairRelation = SourceRecord::PairRelation::NONE;
        source.discoveryPairRelation = SourceRecord::DiscoveryPairRelation::NONE;
        source.fullCoveragePairRelation = SourceRecord::FullCoveragePairRelation::NONE;
        for (std::uint8_t& value : source.reserved) value = 0U;
        target.firstYCoverageMask = 0U;
        target.gainedSinceFirstYMask = 0U;
        target.fullCoverageRelation = FullCoverageRelation::NONE;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentRun.publicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_RUN_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_NOINLINE

static_assert(sizeof(NCPathCoreRunCoverageBoundaryDisposition) == 1U, "AA disposition one byte.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryFullCoverageRelation) == 1U, "AA full coverage relation one byte.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryRecordV1>::value, "AA record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryRecordV1>::value, "AA record scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryRecordV1) == 168U, "AA record exactly 168 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryRecordV1) == 8U, "AA record alignment.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, currentRun) == 8U, "AA current Z offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, schemaVersion) == 160U, "AA schema offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, disposition) == 162U, "AA disposition offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, firstYCoverageMask) == 163U, "AA first Y coverage mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, gainedSinceFirstYMask) == 164U, "AA gained mask offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, fullCoverageRelation) == 165U, "AA coverage relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryRecordV1, reserved) == 166U, "AA reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryShadow>::value, "AA observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryShadow>::value, "AA observer scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryShadow) == 352U, "AA observer exactly 352 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryShadow) == 8U, "AA observer alignment.");
