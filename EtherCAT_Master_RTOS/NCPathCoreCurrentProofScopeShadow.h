#pragma once

#include "NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AE / Current Local Proof Scope Coherence Shadow.
//
// A terminal diagnostic projection, NOT another transition/run accumulator.
// Checks the CURRENT AD proof against the CURRENT AC, AB and AA slots by
// exhaustive semantic equality, including the nested Z certificate. A valid
// record in isolation is not sufficient: independently valid but mismatched
// current sources fail closed. No raw representation or padding is compared.
//
// Scope is exclusively the local AC-certificate run certified by source AD.
// sourceConsecutivePairCount is copied from AD, NOT counted by this observer.
// Attachment to AD EXT is allowed; it does not reconstruct an AE-observed
// head or any omitted history. AD can span availability loss/gain and different
// lower intervals. currentBoundaryRunGeneration/currentBoundaryCoverageGeneration
// identify ONLY the current available AA's nested Z/coverage, never the whole
// AD run. Unavailable AA retains at most its nested Z diagnostic publication;
// unavailable reason is copied exactly from the CURRENT AB, not reconstructed
// for any discarded previous AB/AA. Coverage masks still mean eight U relation
// categories, not eight axes, path coverage, interpolation or completion.
//
// IsStructurallyCoherent() checks the projection's scalar invariants only. It
// is NOT a standalone replay of the source proofs or source binding. Use
// IsBoundToCurrentSourcesSameThread() while all four supplied source slots are
// valid in the SAME producer-thread observation. It rechecks full current
// AD/AC/AB/AA proofs and bindings. No independent current Z slot or pre-AD
// history is authenticated. Trusted same-thread producers are required;
// coherently forged histories, full-wrap reuse and wall-clock freshness are
// not authenticated. A matching old bundle is not a live readiness claim.
//
// A repeated AD publication is STALE; any other nonadjacent nonzero anchor is
// a SEQUENCE_GAP (also includes backwards/out-of-order; no direction is inferred).
// Null clears the source anchor. Every rejection clears ALL projected scope,
// count, availability and coverage; only AD identity/raw diagnostic code stay.
// The next adjacent valid bundle may be checked independently, without claiming
// continuity through rejection. Normal unavailable AA is distinct from unproven
// AD, upstream-reported invalid, malformed records, stale and gap diagnostics.
// Newest canonical record is validated before the older fixed slot is cleared.
//
// Two 96-byte scalar records; 208-byte heap-owned NCManager member. No source
// snapshot, endpoint array, allocation in Observe, event list or retrace data.
// No return value to control, HMI/SHM/API publication, log, thread, timer, mutex,
// wait, sleep, queue, planner, Motion STARTED/DONE, B2 or PDO-path work.
// Existing Path Core headers, M00 FIX1, L.2I FIX1 and all control remain intact.

constexpr std::size_t NC_PATH_CORE_CURRENT_PROOF_SCOPE_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_CURRENT_PROOF_SCOPE_SCHEMA_V1 = 1U;
constexpr std::uint8_t NC_PATH_CORE_CURRENT_PROOF_SCOPE_BINDINGS = 0x0FU;

enum class NCPathCoreCurrentProofScopeDisposition : std::uint8_t
{
    EMPTY = 0U,
    SOURCE_UNAVAILABLE = 1U,
    SOURCE_NOT_PROVEN = 2U,
    SOURCE_REPORTED_INVALID = 3U,
    INVALID_SOURCE_RECORD = 4U,
    INVALID_PREVIOUS_AUDIT_RECORD = 5U,
    INVALID_STALE_SOURCE = 6U,
    INVALID_SOURCE_SEQUENCE_GAP = 7U,
    INVALID_CURRENT_PAIR_BINDING = 8U,
    INVALID_CURRENT_TRANSITION_BINDING = 9U,
    INVALID_CURRENT_BOUNDARY_BINDING = 10U,
    INVALID_PROJECTION = 11U,
    COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE = 12U,
    COHERENT_LOCAL_RUN_BOUNDARY_UNAVAILABLE = 13U
};

enum class NCPathCoreCurrentProofScope : std::uint8_t
{
    NONE = 0U,
    SOURCE_AD_LOCAL_AC_CERTIFICATE_RUN = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
#endif

namespace NCPathCoreCurrentProofScopeDetail
{
    using SourceRecord = NCPathCoreRunCoverageBoundaryAvailabilityPairRunRecordV1;
    using SourceDisposition = SourceRecord::Disposition;
    using PairRecord = SourceRecord::PairRecord;
    using TransitionRecord = PairRecord::TransitionRecord;
    using BoundaryRecord = TransitionRecord::BoundaryRecord;
    using Availability = TransitionRecord::Availability;
    using UnavailableReason = TransitionRecord::UnavailableReason;
    using Disposition = NCPathCoreCurrentProofScopeDisposition;
    using NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::NextNonZeroSequence;
    using NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::ForwardNonZeroSequenceDistance;

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        inline bool IsCanonicalNonProvenSource(const SourceRecord& record) noexcept
    {
        const bool known = record.disposition >= SourceDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE &&
            record.disposition <= SourceDisposition::INVALID_CONTINUITY_RUN;
        const bool identity = record.disposition == SourceDisposition::NOT_APPLICABLE_CURRENT_PAIR_UNAVAILABLE ?
            record.currentPair.publicationSequence == 0ULL : record.currentPair.publicationSequence != 0ULL ||
            record.disposition == SourceDisposition::INVALID_CURRENT_PAIR_RECORD ||
            record.disposition == SourceDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            record.disposition == SourceDisposition::INVALID_PREVIOUS_RUN_RECORD;
        return known && identity && record.publicationSequence != 0ULL &&
            record.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_RUN_SCHEMA_V1 &&
            record.continuityRunGeneration == 0ULL && record.firstPairPublicationSequence == 0ULL &&
            record.consecutivePairCount == 0U && record.reserved == 0U &&
            NCPathCoreRunCoverageBoundaryAvailabilityPairRunDetail::IsZeroPairPayload(record.currentPair);
    }

    // Field-by-field AD semantic comparisons. No private predecessor access.
    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        inline bool MatchesBoundary(const BoundaryRecord& source, const BoundaryRecord& retained) noexcept
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

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        inline bool MatchesTransition(const TransitionRecord& source, const TransitionRecord& retained) noexcept
    {
        return source.publicationSequence == retained.publicationSequence &&
            source.previousBoundaryPublicationSequence == retained.previousBoundaryPublicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.relation == retained.relation && source.previousAvailability == retained.previousAvailability &&
            source.currentAvailability == retained.currentAvailability &&
            source.currentUnavailableReason == retained.currentUnavailableReason && source.reserved == retained.reserved &&
            MatchesBoundary(source.currentBoundary, retained.currentBoundary);
    }

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        inline bool MatchesPair(const PairRecord& source, const PairRecord& retained) noexcept
    {
        return source.publicationSequence == retained.publicationSequence &&
            source.previousTransitionPublicationSequence == retained.previousTransitionPublicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.previousRelation == retained.previousRelation && source.relation == retained.relation &&
            source.reserved[0U] == retained.reserved[0U] && source.reserved[1U] == retained.reserved[1U] &&
            source.reserved[2U] == retained.reserved[2U] &&
            MatchesTransition(source.currentTransition, retained.currentTransition);
    }

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        inline Disposition CheckCurrentSources(const SourceRecord* source, const PairRecord* pair,
            const TransitionRecord* transition, const BoundaryRecord* boundary) noexcept
    {
        if (source == nullptr) return Disposition::SOURCE_UNAVAILABLE;
        if (!source->IsProvenRunCoverageBoundaryAvailabilityPairRun())
        {
            if (!IsCanonicalNonProvenSource(*source)) return Disposition::INVALID_SOURCE_RECORD;
            return source->disposition <= SourceDisposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN ?
                Disposition::SOURCE_NOT_PROVEN : Disposition::SOURCE_REPORTED_INVALID;
        }
        if (pair == nullptr || !MatchesPair(source->currentPair, *pair))
            return Disposition::INVALID_CURRENT_PAIR_BINDING;
        if (transition == nullptr || !MatchesTransition(pair->currentTransition, *transition))
            return Disposition::INVALID_CURRENT_TRANSITION_BINDING;
        if (boundary == nullptr || !MatchesBoundary(transition->currentBoundary, *boundary))
            return Disposition::INVALID_CURRENT_BOUNDARY_BINDING;
        // AD self-proof recursively validates these exact semantic payloads.
        return transition->currentAvailability == Availability::AVAILABLE ?
            Disposition::COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE :
            Disposition::COHERENT_LOCAL_RUN_BOUNDARY_UNAVAILABLE;
    }
}

struct NCPathCoreCurrentProofScopeRecordV1
{
    using Disposition = NCPathCoreCurrentProofScopeDisposition;
    using Scope = NCPathCoreCurrentProofScope;
    using SourceRecord = NCPathCoreCurrentProofScopeDetail::SourceRecord;
    using PairRecord = NCPathCoreCurrentProofScopeDetail::PairRecord;
    using TransitionRecord = NCPathCoreCurrentProofScopeDetail::TransitionRecord;
    using BoundaryRecord = NCPathCoreCurrentProofScopeDetail::BoundaryRecord;
    using Availability = NCPathCoreCurrentProofScopeDetail::Availability;
    using UnavailableReason = NCPathCoreCurrentProofScopeDetail::UnavailableReason;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourcePublicationSequence = 0ULL; // AD; sole rejection identity.
    std::uint64_t sourceRunGeneration = 0ULL;
    std::uint64_t firstPairPublicationSequence = 0ULL;
    std::uint64_t currentPairPublicationSequence = 0ULL;
    std::uint64_t currentTransitionPublicationSequence = 0ULL;
    std::uint64_t currentBoundaryPublicationSequence = 0ULL;
    std::uint64_t currentBoundarySourcePublicationSequence = 0ULL; // Nested Z anchor only.
    std::uint64_t currentBoundaryRunGeneration = 0ULL;
    std::uint64_t currentBoundaryCoverageGeneration = 0ULL;
    std::uint32_t sourceConsecutivePairCount = 0U;
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Scope scope = Scope::NONE;
    Availability currentBoundaryAvailability = Availability::UNKNOWN;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::uint8_t currentCoverageMask = 0U;
    std::uint8_t firstYCoverageMask = 0U;
    std::uint8_t gainedSinceFirstYMask = 0U;
    std::uint8_t bindingMask = 0U; // Four current source checks, NOT coverage.
    std::uint8_t sourceDisposition = 0U; // Raw AD diagnostic, not AA reason.
    std::uint8_t reserved = 0U;

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        bool IsStructurallyCoherent() const noexcept
    {
        using namespace NCPathCoreCurrentProofScopeDetail;
        if (publicationSequence == 0ULL || sourcePublicationSequence == 0ULL || sourceRunGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL || currentPairPublicationSequence == 0ULL ||
            currentTransitionPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            schemaVersion != NC_PATH_CORE_CURRENT_PROOF_SCOPE_SCHEMA_V1 || reserved != 0U ||
            scope != Scope::SOURCE_AD_LOCAL_AC_CERTIFICATE_RUN || bindingMask != NC_PATH_CORE_CURRENT_PROOF_SCOPE_BINDINGS ||
            sourceConsecutivePairCount < 2U) return false;
        const bool start = sourceDisposition == static_cast<std::uint8_t>(SourceDisposition::PROVEN_LOCAL_PAIR_RUN_START) &&
            sourceConsecutivePairCount == 2U;
        const bool extension = sourceDisposition == static_cast<std::uint8_t>(SourceDisposition::PROVEN_LOCAL_PAIR_RUN_EXTENSION) &&
            sourceConsecutivePairCount > 2U;
        if ((!start && !extension) ||
            ForwardNonZeroSequenceDistance(sourceRunGeneration, sourcePublicationSequence) !=
            static_cast<std::uint64_t>(sourceConsecutivePairCount) - 2ULL ||
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, currentPairPublicationSequence) !=
            static_cast<std::uint64_t>(sourceConsecutivePairCount) - 1ULL) return false;
        if (disposition == Disposition::COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE)
        {
            return currentBoundaryAvailability == Availability::AVAILABLE &&
                currentUnavailableReason == UnavailableReason::NONE && currentBoundarySourcePublicationSequence != 0ULL &&
                currentBoundaryRunGeneration != 0ULL && currentBoundaryCoverageGeneration != 0ULL &&
                firstYCoverageMask != 0U && (firstYCoverageMask & currentCoverageMask) == firstYCoverageMask &&
                gainedSinceFirstYMask == static_cast<std::uint8_t>(currentCoverageMask &
                    static_cast<std::uint8_t>(~firstYCoverageMask));
        }
        return disposition == Disposition::COHERENT_LOCAL_RUN_BOUNDARY_UNAVAILABLE &&
            currentBoundaryAvailability == Availability::UNAVAILABLE &&
            currentUnavailableReason >= UnavailableReason::CURRENT_RUN_UNAVAILABLE &&
            currentUnavailableReason <= UnavailableReason::DIFFERENT_RUN &&
            (currentUnavailableReason == UnavailableReason::CURRENT_RUN_UNAVAILABLE ?
                currentBoundarySourcePublicationSequence == 0ULL : currentBoundarySourcePublicationSequence != 0ULL) &&
            currentBoundaryRunGeneration == 0ULL && currentBoundaryCoverageGeneration == 0ULL &&
            currentCoverageMask == 0U && firstYCoverageMask == 0U && gainedSinceFirstYMask == 0U;
    }

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        bool IsCanonicalDiagnostic() const noexcept
    {
        return publicationSequence != 0ULL && schemaVersion == NC_PATH_CORE_CURRENT_PROOF_SCOPE_SCHEMA_V1 &&
            disposition >= Disposition::SOURCE_UNAVAILABLE && disposition <= Disposition::INVALID_PROJECTION &&
            (disposition != Disposition::SOURCE_UNAVAILABLE || (sourcePublicationSequence == 0ULL && sourceDisposition == 0U)) &&
            sourceRunGeneration == 0ULL && firstPairPublicationSequence == 0ULL && currentPairPublicationSequence == 0ULL &&
            currentTransitionPublicationSequence == 0ULL && currentBoundaryPublicationSequence == 0ULL &&
            currentBoundarySourcePublicationSequence == 0ULL && currentBoundaryRunGeneration == 0ULL &&
            currentBoundaryCoverageGeneration == 0ULL && sourceConsecutivePairCount == 0U &&
            scope == Scope::NONE && currentBoundaryAvailability == Availability::UNKNOWN &&
            currentUnavailableReason == UnavailableReason::NONE && currentCoverageMask == 0U &&
            firstYCoverageMask == 0U && gainedSinceFirstYMask == 0U && bindingMask == 0U && reserved == 0U;
    }

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        bool IsBoundToCurrentSourcesSameThread(const SourceRecord* source, const PairRecord* pair,
            const TransitionRecord* transition, const BoundaryRecord* boundary) const noexcept
    {
        using NCPathCoreCurrentProofScopeDetail::CheckCurrentSources;
        if (!IsStructurallyCoherent() || source == nullptr || pair == nullptr || transition == nullptr || boundary == nullptr ||
            CheckCurrentSources(source, pair, transition, boundary) != disposition) return false;
        return sourcePublicationSequence == source->publicationSequence && sourceRunGeneration == source->continuityRunGeneration &&
            firstPairPublicationSequence == source->firstPairPublicationSequence &&
            currentPairPublicationSequence == pair->publicationSequence &&
            currentTransitionPublicationSequence == transition->publicationSequence &&
            currentBoundaryPublicationSequence == boundary->publicationSequence &&
            currentBoundarySourcePublicationSequence == boundary->currentRun.publicationSequence &&
            currentBoundaryRunGeneration == boundary->currentRun.runGeneration &&
            currentBoundaryCoverageGeneration == boundary->currentRun.coverageGeneration &&
            sourceConsecutivePairCount == source->consecutivePairCount &&
            sourceDisposition == static_cast<std::uint8_t>(source->disposition) &&
            currentBoundaryAvailability == transition->currentAvailability &&
            currentUnavailableReason == transition->currentUnavailableReason &&
            currentCoverageMask == boundary->currentRun.currentObservedPairRelationMask &&
            firstYCoverageMask == boundary->firstYCoverageMask && gainedSinceFirstYMask == boundary->gainedSinceFirstYMask;
    }
};

class NCPathCoreCurrentProofScopeShadow final
{
public:
    using Record = NCPathCoreCurrentProofScopeRecordV1;
    using Disposition = Record::Disposition;
    using SourceRecord = Record::SourceRecord;
    using PairRecord = Record::PairRecord;
    using TransitionRecord = Record::TransitionRecord;
    using BoundaryRecord = Record::BoundaryRecord;

    NCPathCoreCurrentProofScopeShadow() noexcept = default;

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        void ObserveCurrentProofScopeSameThread(const SourceRecord* source, const PairRecord* pair,
            const TransitionRecord* transition, const BoundaryRecord* boundary) noexcept
    {
        using NCPathCoreCurrentProofScopeDetail::NextNonZeroSequence;
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousValid = StorageValid() && (previous == nullptr ||
            (previous->publicationSequence == m_publicationSequence &&
                (previous->IsStructurallyCoherent() || previous->IsCanonicalDiagnostic())));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->sourcePublicationSequence;
        m_publicationSequence = NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex < HISTORY_CAPACITY ?
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY : 0U;
        Record& target = m_records[index];
        ClearProjection(target); // No record temporary; newest checked before older overwrite.
        target.publicationSequence = m_publicationSequence;
        target.sourcePublicationSequence = source == nullptr ? 0ULL : source->publicationSequence;
        target.sourceDisposition = source == nullptr ? 0U : static_cast<std::uint8_t>(source->disposition);
        target.schemaVersion = NC_PATH_CORE_CURRENT_PROOF_SCOPE_SCHEMA_V1;
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_AUDIT_RECORD;
        else if (source != nullptr && anchor != 0ULL && source->publicationSequence == anchor)
            target.disposition = Disposition::INVALID_STALE_SOURCE;
        else if (source != nullptr && anchor != 0ULL && source->publicationSequence != NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_SOURCE_SEQUENCE_GAP;
        else
        {
            target.disposition = NCPathCoreCurrentProofScopeDetail::CheckCurrentSources(source, pair, transition, boundary);
            if (source != nullptr && (target.disposition == Disposition::COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE ||
                target.disposition == Disposition::COHERENT_LOCAL_RUN_BOUNDARY_UNAVAILABLE))
            {
                Project(*source, target);
                // Full bindings were checked above; projection structural check adds no source copy.
                if (!target.IsStructurallyCoherent())
                {
                    ClearProjection(target);
                    target.disposition = Disposition::INVALID_PROJECTION;
                }
            }
        }
        m_latestIndex = static_cast<std::uint8_t>(index);
        m_recordCount = m_recordCount < HISTORY_CAPACITY ? static_cast<std::uint8_t>(m_recordCount + 1U) :
            static_cast<std::uint8_t>(HISTORY_CAPACITY);
    }

    const Record* GetNewestObservationSameThread(std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex >= HISTORY_CAPACITY || m_recordCount == 0U || m_recordCount > HISTORY_CAPACITY ||
            historyOffset >= m_recordCount) return nullptr;
        return &m_records[(static_cast<std::size_t>(m_latestIndex) + HISTORY_CAPACITY - historyOffset) % HISTORY_CAPACITY];
    }

    std::size_t GetRecordCountSameThread() const noexcept { return m_recordCount; }

private:
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_CURRENT_PROOF_SCOPE_HISTORY_CAPACITY;
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    bool StorageValid() const noexcept
    {
        for (std::uint8_t value : m_reserved) if (value != 0U) return false;
        return (m_recordCount == 0U && m_latestIndex == INVALID_INDEX && m_publicationSequence == 0ULL) ||
            (m_recordCount > 0U && m_recordCount <= HISTORY_CAPACITY && m_latestIndex < HISTORY_CAPACITY&&
                m_publicationSequence != 0ULL);
    }

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        static void Project(const SourceRecord& source, Record& target) noexcept
    {
        const PairRecord& pair = source.currentPair;
        const TransitionRecord& transition = pair.currentTransition;
        const BoundaryRecord& boundary = transition.currentBoundary;
        target.sourceRunGeneration = source.continuityRunGeneration;
        target.firstPairPublicationSequence = source.firstPairPublicationSequence;
        target.currentPairPublicationSequence = pair.publicationSequence;
        target.currentTransitionPublicationSequence = transition.publicationSequence;
        target.currentBoundaryPublicationSequence = boundary.publicationSequence;
        target.currentBoundarySourcePublicationSequence = boundary.currentRun.publicationSequence;
        target.currentBoundaryRunGeneration = boundary.currentRun.runGeneration;
        target.currentBoundaryCoverageGeneration = boundary.currentRun.coverageGeneration;
        target.sourceConsecutivePairCount = source.consecutivePairCount;
        target.scope = Record::Scope::SOURCE_AD_LOCAL_AC_CERTIFICATE_RUN;
        target.currentBoundaryAvailability = transition.currentAvailability;
        target.currentUnavailableReason = transition.currentUnavailableReason;
        target.currentCoverageMask = boundary.currentRun.currentObservedPairRelationMask;
        target.firstYCoverageMask = boundary.firstYCoverageMask;
        target.gainedSinceFirstYMask = boundary.gainedSinceFirstYMask;
        target.bindingMask = NC_PATH_CORE_CURRENT_PROOF_SCOPE_BINDINGS;
    }

    NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE
        static void ClearProjection(Record& target) noexcept
    {
        // The publication/AD diagnostic anchors, schema and outcome are set by caller.
        target.sourceRunGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.currentPairPublicationSequence = 0ULL;
        target.currentTransitionPublicationSequence = 0ULL;
        target.currentBoundaryPublicationSequence = 0ULL;
        target.currentBoundarySourcePublicationSequence = 0ULL;
        target.currentBoundaryRunGeneration = 0ULL;
        target.currentBoundaryCoverageGeneration = 0ULL;
        target.sourceConsecutivePairCount = 0U;
        target.scope = Record::Scope::NONE;
        target.currentBoundaryAvailability = Record::Availability::UNKNOWN;
        target.currentUnavailableReason = Record::UnavailableReason::NONE;
        target.currentCoverageMask = 0U;
        target.firstYCoverageMask = 0U;
        target.gainedSinceFirstYMask = 0U;
        target.bindingMask = 0U;
        target.reserved = 0U;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_CURRENT_PROOF_SCOPE_NOINLINE

static_assert(sizeof(NCPathCoreCurrentProofScopeDisposition) == 1U, "AE disposition one byte.");
static_assert(sizeof(NCPathCoreCurrentProofScope) == 1U, "AE scope one byte.");
static_assert(std::is_standard_layout<NCPathCoreCurrentProofScopeRecordV1>::value, "AE record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreCurrentProofScopeRecordV1>::value, "AE record scalar copyability.");
static_assert(sizeof(NCPathCoreCurrentProofScopeRecordV1) == 96U, "AE record exactly 96 bytes.");
static_assert(alignof(NCPathCoreCurrentProofScopeRecordV1) == 8U, "AE record alignment.");
static_assert(offsetof(NCPathCoreCurrentProofScopeRecordV1, sourceConsecutivePairCount) == 80U, "AE count offset.");
static_assert(offsetof(NCPathCoreCurrentProofScopeRecordV1, schemaVersion) == 84U, "AE schema offset.");
static_assert(offsetof(NCPathCoreCurrentProofScopeRecordV1, reserved) == 95U, "AE reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreCurrentProofScopeShadow>::value, "AE observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreCurrentProofScopeShadow>::value, "AE observer scalar copyability.");
static_assert(sizeof(NCPathCoreCurrentProofScopeShadow) == 208U, "AE observer exactly 208 bytes.");
static_assert(alignof(NCPathCoreCurrentProofScopeShadow) == 8U, "AE observer alignment.");
