#pragma once

#include "NCPathCoreCurrentProofScopeShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AF / Coherent Current Proof Frontier Certificate Shadow.
//
// AE proves that the CURRENT AD/AC/AB/AA bundle is mutually coherent, but a
// later Path Core component should not have to reinterpret AE's complete
// diagnostic projection or retain pointers into the observer chain. AF makes
// one compact scalar identity for that already-proven current proof frontier.
// It is a handoff certificate only, not another relation/run accumulator.
//
// A proven AF record binds the exact AE publication, AD publication/local run,
// first/current AC publications, current AB and AA publications, AD count,
// current AA availability and current eight-U-relation coverage mask. It then
// rechecks AE against the supplied CURRENT AD/AC/AB/AA sources in the same
// producer-thread observation before publication. The two AD zero-skipping
// spans are retained in scalar form so a detached AF record can reject an
// internally inconsistent local-run identity; it still cannot replay omitted
// previous AC admission, discarded AB/AA payloads or any pre-AD history.
//
// AVAILABLE and UNAVAILABLE are AA boundary-certificate availability only.
// They are not path availability, stable readiness, one Z/committed/coverage
// interval, all-axis return, geometric completion, Motion STARTED/DONE or B2
// eligibility. The coverage mask still represents the eight U relation modes,
// not axes or path segments. A coherent unavailable frontier is a proven
// diagnostic state, not permission to infer why an older AA was discarded.
//
// Null, malformed AE, AE diagnostic state, stale/gapped AE publication or
// failed current-source rebinding publishes only a canonical diagnostic and
// clears every frontier field. No rejection is converted into normal boundary
// unavailability. A subsequent valid bundle may establish a new current
// frontier independently; AF claims no continuity across its observations.
//
// Trusted same-thread producers are required. Coherent forgery, full-wrap
// identity reuse and wall-clock freshness are not authenticated. Exactly two
// 80-byte scalar records in a 176-byte heap-owned NCManager member. No source
// snapshot, coordinate, displacement, endpoint array, node, segment/event
// list, reverse traversal, Observe allocation, log, thread, timer, mutex, wait
// or sleep. Shadow-only; no HMI/SHM/API/PDO publication and no control consumer.
// MotionCore, G00, Gate/Registry, M00 FIX1, L.2I FIX1, DC/PDO/NIC/EtherCAT and
// Priority-64 PDO paths remain unchanged.

constexpr std::size_t NC_PATH_CORE_PROOF_FRONTIER_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_PROOF_FRONTIER_SCHEMA_V1 = 1U;
constexpr std::uint8_t NC_PATH_CORE_PROOF_FRONTIER_BINDINGS = 0x1FU; // AE + AD + AC + AB + AA.

enum class NCPathCoreProofFrontierDisposition : std::uint8_t
{
    EMPTY = 0U,
    SOURCE_UNAVAILABLE = 1U,
    SOURCE_AUDIT_NOT_COHERENT = 2U,
    INVALID_SOURCE_AUDIT_RECORD = 3U,
    INVALID_PREVIOUS_FRONTIER_RECORD = 4U,
    INVALID_STALE_SOURCE_AUDIT = 5U,
    INVALID_SOURCE_AUDIT_SEQUENCE_GAP = 6U,
    INVALID_CURRENT_SOURCE_BINDING = 7U,
    INVALID_PROJECTION = 8U,
    PROVEN_CURRENT_FRONTIER_BOUNDARY_AVAILABLE = 9U,
    PROVEN_CURRENT_FRONTIER_BOUNDARY_UNAVAILABLE = 10U
};

enum class NCPathCoreProofFrontierScope : std::uint8_t
{
    NONE = 0U,
    CURRENT_AE_BOUND_LOCAL_AD_RUN = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_PROOF_FRONTIER_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_PROOF_FRONTIER_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
#endif

namespace NCPathCoreProofFrontierDetail
{
    using AuditRecord = NCPathCoreCurrentProofScopeRecordV1;
    using AuditDisposition = AuditRecord::Disposition;
    using SourceRecord = AuditRecord::SourceRecord;
    using PairRecord = AuditRecord::PairRecord;
    using TransitionRecord = AuditRecord::TransitionRecord;
    using BoundaryRecord = AuditRecord::BoundaryRecord;
    using Availability = AuditRecord::Availability;
    using NCPathCoreCurrentProofScopeDetail::NextNonZeroSequence;
    using NCPathCoreCurrentProofScopeDetail::ForwardNonZeroSequenceDistance;

    inline bool IsCoherentAudit(const AuditRecord& audit) noexcept
    {
        return audit.disposition == AuditDisposition::COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE ||
            audit.disposition == AuditDisposition::COHERENT_LOCAL_RUN_BOUNDARY_UNAVAILABLE;
    }
}

struct NCPathCoreProofFrontierRecordV1
{
    using Disposition = NCPathCoreProofFrontierDisposition;
    using Scope = NCPathCoreProofFrontierScope;
    using AuditRecord = NCPathCoreProofFrontierDetail::AuditRecord;
    using AuditDisposition = NCPathCoreProofFrontierDetail::AuditDisposition;
    using SourceRecord = NCPathCoreProofFrontierDetail::SourceRecord;
    using PairRecord = NCPathCoreProofFrontierDetail::PairRecord;
    using TransitionRecord = NCPathCoreProofFrontierDetail::TransitionRecord;
    using BoundaryRecord = NCPathCoreProofFrontierDetail::BoundaryRecord;
    using Availability = NCPathCoreProofFrontierDetail::Availability;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourceAuditPublicationSequence = 0ULL; // Exact AE observation.
    std::uint64_t sourcePublicationSequence = 0ULL; // AD publication.
    std::uint64_t sourceRunGeneration = 0ULL;
    std::uint64_t firstPairPublicationSequence = 0ULL;
    std::uint64_t currentPairPublicationSequence = 0ULL;
    std::uint64_t currentTransitionPublicationSequence = 0ULL;
    std::uint64_t currentBoundaryPublicationSequence = 0ULL;
    std::uint32_t sourceConsecutivePairCount = 0U;
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Scope scope = Scope::NONE;
    Availability currentBoundaryAvailability = Availability::UNKNOWN;
    std::uint8_t currentCoverageMask = 0U;
    std::uint8_t bindingMask = 0U;
    std::uint8_t sourceAuditDisposition = 0U; // Raw AE diagnostic/proven outcome.
    std::array<std::uint8_t, 4U> reserved{};

    NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
        bool IsProvenCurrentProofFrontier() const noexcept
    {
        using namespace NCPathCoreProofFrontierDetail;
        if (publicationSequence == 0ULL || sourceAuditPublicationSequence == 0ULL ||
            sourcePublicationSequence == 0ULL || sourceRunGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL || currentPairPublicationSequence == 0ULL ||
            currentTransitionPublicationSequence == 0ULL || currentBoundaryPublicationSequence == 0ULL ||
            sourceConsecutivePairCount < 2U || schemaVersion != NC_PATH_CORE_PROOF_FRONTIER_SCHEMA_V1 ||
            scope != Scope::CURRENT_AE_BOUND_LOCAL_AD_RUN || bindingMask != NC_PATH_CORE_PROOF_FRONTIER_BINDINGS)
            return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        if (ForwardNonZeroSequenceDistance(sourceRunGeneration, sourcePublicationSequence) !=
            static_cast<std::uint64_t>(sourceConsecutivePairCount) - 2ULL ||
            ForwardNonZeroSequenceDistance(firstPairPublicationSequence, currentPairPublicationSequence) !=
            static_cast<std::uint64_t>(sourceConsecutivePairCount) - 1ULL) return false;
        const auto auditDisposition = static_cast<AuditDisposition>(sourceAuditDisposition);
        if (disposition == Disposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_AVAILABLE)
        {
            return auditDisposition == AuditDisposition::COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE &&
                currentBoundaryAvailability == Availability::AVAILABLE && currentCoverageMask != 0U;
        }
        return disposition == Disposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_UNAVAILABLE &&
            auditDisposition == AuditDisposition::COHERENT_LOCAL_RUN_BOUNDARY_UNAVAILABLE &&
            currentBoundaryAvailability == Availability::UNAVAILABLE && currentCoverageMask == 0U;
    }

    NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
        bool IsCanonicalDiagnostic() const noexcept
    {
        if (publicationSequence == 0ULL || schemaVersion != NC_PATH_CORE_PROOF_FRONTIER_SCHEMA_V1 ||
            disposition < Disposition::SOURCE_UNAVAILABLE || disposition > Disposition::INVALID_PROJECTION)
            return false;
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        return (disposition != Disposition::SOURCE_UNAVAILABLE ||
            (sourceAuditPublicationSequence == 0ULL && sourceAuditDisposition == 0U)) &&
            sourcePublicationSequence == 0ULL && sourceRunGeneration == 0ULL &&
            firstPairPublicationSequence == 0ULL && currentPairPublicationSequence == 0ULL &&
            currentTransitionPublicationSequence == 0ULL && currentBoundaryPublicationSequence == 0ULL &&
            sourceConsecutivePairCount == 0U && scope == Scope::NONE &&
            currentBoundaryAvailability == Availability::UNKNOWN && currentCoverageMask == 0U && bindingMask == 0U;
    }

    NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
        bool IsBoundToCurrentSourcesSameThread(const AuditRecord* audit, const SourceRecord* source,
            const PairRecord* pair, const TransitionRecord* transition, const BoundaryRecord* boundary) const noexcept
    {
        if (!IsProvenCurrentProofFrontier() || audit == nullptr || source == nullptr || pair == nullptr ||
            transition == nullptr || boundary == nullptr || !NCPathCoreProofFrontierDetail::IsCoherentAudit(*audit) ||
            !audit->IsBoundToCurrentSourcesSameThread(source, pair, transition, boundary)) return false;
        return sourceAuditPublicationSequence == audit->publicationSequence &&
            sourceAuditDisposition == static_cast<std::uint8_t>(audit->disposition) &&
            sourcePublicationSequence == audit->sourcePublicationSequence &&
            sourceRunGeneration == audit->sourceRunGeneration &&
            firstPairPublicationSequence == audit->firstPairPublicationSequence &&
            currentPairPublicationSequence == audit->currentPairPublicationSequence &&
            currentTransitionPublicationSequence == audit->currentTransitionPublicationSequence &&
            currentBoundaryPublicationSequence == audit->currentBoundaryPublicationSequence &&
            sourceConsecutivePairCount == audit->sourceConsecutivePairCount &&
            currentBoundaryAvailability == audit->currentBoundaryAvailability &&
            currentCoverageMask == audit->currentCoverageMask;
    }
};

class NCPathCoreProofFrontierShadow final
{
public:
    using Record = NCPathCoreProofFrontierRecordV1;
    using Disposition = Record::Disposition;
    using AuditRecord = Record::AuditRecord;
    using SourceRecord = Record::SourceRecord;
    using PairRecord = Record::PairRecord;
    using TransitionRecord = Record::TransitionRecord;
    using BoundaryRecord = Record::BoundaryRecord;

    NCPathCoreProofFrontierShadow() noexcept = default;

    NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
        void ObserveProofFrontierSameThread(const AuditRecord* audit, const SourceRecord* source,
            const PairRecord* pair, const TransitionRecord* transition, const BoundaryRecord* boundary) noexcept
    {
        using namespace NCPathCoreProofFrontierDetail;
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousValid = StorageValid() && (previous == nullptr ||
            (previous->publicationSequence == m_publicationSequence &&
                (previous->IsProvenCurrentProofFrontier() || previous->IsCanonicalDiagnostic())));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->sourceAuditPublicationSequence;
        m_publicationSequence = NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex < HISTORY_CAPACITY ?
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY : 0U;
        Record& target = m_records[index];
        ClearProjection(target);
        target.publicationSequence = m_publicationSequence;
        target.sourceAuditPublicationSequence = audit == nullptr ? 0ULL : audit->publicationSequence;
        target.sourceAuditDisposition = audit == nullptr ? 0U : static_cast<std::uint8_t>(audit->disposition);
        target.schemaVersion = NC_PATH_CORE_PROOF_FRONTIER_SCHEMA_V1;

        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_FRONTIER_RECORD;
        else if (audit == nullptr) target.disposition = Disposition::SOURCE_UNAVAILABLE;
        else if (!audit->IsStructurallyCoherent() && !audit->IsCanonicalDiagnostic())
            target.disposition = Disposition::INVALID_SOURCE_AUDIT_RECORD;
        else if (anchor != 0ULL && audit->publicationSequence == anchor)
            target.disposition = Disposition::INVALID_STALE_SOURCE_AUDIT;
        else if (anchor != 0ULL && audit->publicationSequence != NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_SOURCE_AUDIT_SEQUENCE_GAP;
        else if (!audit->IsStructurallyCoherent())
            target.disposition = Disposition::SOURCE_AUDIT_NOT_COHERENT;
        else if (!IsCoherentAudit(*audit) ||
            !audit->IsBoundToCurrentSourcesSameThread(source, pair, transition, boundary))
            target.disposition = Disposition::INVALID_CURRENT_SOURCE_BINDING;
        else
        {
            Project(*audit, target);
            target.disposition = audit->disposition == AuditDisposition::COHERENT_LOCAL_RUN_BOUNDARY_AVAILABLE ?
                Disposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_AVAILABLE :
                Disposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_UNAVAILABLE;
            if (!target.IsProvenCurrentProofFrontier())
            {
                ClearProjection(target);
                target.disposition = Disposition::INVALID_PROJECTION;
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_PROOF_FRONTIER_HISTORY_CAPACITY;
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    bool StorageValid() const noexcept
    {
        for (std::uint8_t value : m_reserved) if (value != 0U) return false;
        return (m_recordCount == 0U && m_latestIndex == INVALID_INDEX && m_publicationSequence == 0ULL) ||
            (m_recordCount > 0U && m_recordCount <= HISTORY_CAPACITY && m_latestIndex < HISTORY_CAPACITY&&
                m_publicationSequence != 0ULL);
    }

    NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
        static void Project(const AuditRecord& audit, Record& target) noexcept
    {
        target.sourcePublicationSequence = audit.sourcePublicationSequence;
        target.sourceRunGeneration = audit.sourceRunGeneration;
        target.firstPairPublicationSequence = audit.firstPairPublicationSequence;
        target.currentPairPublicationSequence = audit.currentPairPublicationSequence;
        target.currentTransitionPublicationSequence = audit.currentTransitionPublicationSequence;
        target.currentBoundaryPublicationSequence = audit.currentBoundaryPublicationSequence;
        target.sourceConsecutivePairCount = audit.sourceConsecutivePairCount;
        target.scope = Record::Scope::CURRENT_AE_BOUND_LOCAL_AD_RUN;
        target.currentBoundaryAvailability = audit.currentBoundaryAvailability;
        target.currentCoverageMask = audit.currentCoverageMask;
        target.bindingMask = NC_PATH_CORE_PROOF_FRONTIER_BINDINGS;
    }

    NC_PATH_CORE_PROOF_FRONTIER_NOINLINE
        static void ClearProjection(Record& target) noexcept
    {
        target.sourcePublicationSequence = 0ULL;
        target.sourceRunGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.currentPairPublicationSequence = 0ULL;
        target.currentTransitionPublicationSequence = 0ULL;
        target.currentBoundaryPublicationSequence = 0ULL;
        target.sourceConsecutivePairCount = 0U;
        target.scope = Record::Scope::NONE;
        target.currentBoundaryAvailability = Record::Availability::UNKNOWN;
        target.currentCoverageMask = 0U;
        target.bindingMask = 0U;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_PROOF_FRONTIER_NOINLINE

static_assert(sizeof(NCPathCoreProofFrontierDisposition) == 1U, "AF disposition one byte.");
static_assert(sizeof(NCPathCoreProofFrontierScope) == 1U, "AF scope one byte.");
static_assert(std::is_standard_layout<NCPathCoreProofFrontierRecordV1>::value, "AF record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreProofFrontierRecordV1>::value, "AF record scalar copyability.");
static_assert(sizeof(NCPathCoreProofFrontierRecordV1) == 80U, "AF record exactly 80 bytes.");
static_assert(alignof(NCPathCoreProofFrontierRecordV1) == 8U, "AF record alignment.");
static_assert(offsetof(NCPathCoreProofFrontierRecordV1, sourceConsecutivePairCount) == 64U, "AF count offset.");
static_assert(offsetof(NCPathCoreProofFrontierRecordV1, schemaVersion) == 68U, "AF schema offset.");
static_assert(offsetof(NCPathCoreProofFrontierRecordV1, reserved) == 76U, "AF reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreProofFrontierShadow>::value, "AF observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreProofFrontierShadow>::value, "AF observer scalar copyability.");
static_assert(sizeof(NCPathCoreProofFrontierShadow) == 176U, "AF observer exactly 176 bytes.");
static_assert(alignof(NCPathCoreProofFrontierShadow) == 8U, "AF observer alignment.");
