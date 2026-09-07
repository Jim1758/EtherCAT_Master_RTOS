#pragma once

#include "NCPathCoreLinkedCommittedSegmentPairShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2H / Proven Linked Committed Segment Run-Length Shadow
//
// This fixed two-record observer consumes only the newest L.2G relationship.
// A proven L.2G pair establishes a two-segment scalar run.  Each later proven
// pair may extend that run only when it overlaps the exact previously proven
// tail segment and all publication, accepted-input, link and source-PC fences
// remain direct.  A neutral, malformed or non-overlapping observation breaks
// the run fail-closed and is itself published as the newest observation.
//
// L.2H stores only scalar provenance, a saturating segment count and two axis
// union masks.  It stores no endpoint arrays and no per-segment list.  Its run
// length is therefore not Path Queue depth, executable look-ahead, Motion
// progress, retrace capacity or a B2 breadcrumb count.  The summary cannot be
// traversed to reconstruct geometry and has no Gate, PC, Alarm, Motion,
// HMI/SHM/API, PDO, EtherCAT or DC consumer.
//
// The shadow is same-thread, fixed-capacity and heap-resident through
// NCManager.  It performs no allocation, logging, waiting or synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_SCHEMA_V1 = 1U;

namespace NCPathCoreLinkedCommittedSegmentRunDetail
{
    constexpr std::uint32_t LINKED_SEGMENT_RUN_LENGTH_MAX =
        (std::numeric_limits<std::uint32_t>::max)();

    constexpr std::uint32_t SaturatingIncrementRunLength(
        std::uint32_t current) noexcept
    {
        return current == LINKED_SEGMENT_RUN_LENGTH_MAX
            ? LINKED_SEGMENT_RUN_LENGTH_MAX
            : current + 1U;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_PAIR_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PAIR_NOT_PROVEN = 2U,
    INVALID_PAIR_RECORD = 3U,
    INVALID_PREVIOUS_RUN_RECORD = 4U,
    INVALID_RUN_EXTENSION_FENCE = 5U,
    PROVEN_LINKED_SEGMENT_RUN_STARTED = 6U,
    PROVEN_LINKED_SEGMENT_RUN_EXTENDED = 7U
};

enum class NCPathCoreLinkedCommittedSegmentRunRelation : std::uint8_t
{
    NONE = 0U,
    FIRST_PROVEN_PAIR_ESTABLISHES_TWO_SEGMENT_RUN = 1U,
    OVERLAPPING_PROVEN_PAIR_EXTENDS_EXISTING_RUN = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunExtent : std::uint8_t
{
    NONE = 0U,
    SCALAR_LINKED_SEGMENT_RUN_SUMMARY_ONLY = 1U
};

struct NCPathCoreLinkedCommittedSegmentRunRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourcePairPublicationSequence = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t firstPairPublicationSequence = 0ULL;
    std::uint64_t latestPairPublicationSequence = 0ULL;
    std::uint64_t firstSegmentPublicationSequence = 0ULL;
    std::uint64_t latestSegmentPublicationSequence = 0ULL;
    std::uint64_t firstLinkPublicationSequence = 0ULL;
    std::uint64_t latestLinkPublicationSequence = 0ULL;

    std::uint32_t linkedSegmentRunLength = 0U;
    std::uint32_t firstAcceptedInputRunLength = 0U;
    std::uint32_t latestAcceptedInputRunLength = 0U;
    std::uint32_t participatingAxisUnionMask = 0U;
    std::uint32_t coordinateChangeAxisUnionMask = 0U;
    std::uint32_t latestSegmentAxisMask = 0U;
    std::uint32_t latestCoordinateChangeAxisMask = 0U;
    std::int32_t firstSourcePC = -1;
    std::int32_t latestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentRunDisposition disposition =
        NCPathCoreLinkedCommittedSegmentRunDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentRunRelation runRelation =
        NCPathCoreLinkedCommittedSegmentRunRelation::NONE;
    NCPathCoreLinkedCommittedSegmentRunExtent extent =
        NCPathCoreLinkedCommittedSegmentRunExtent::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 4U> reserved{};

    bool IsProvenLinkedCommittedSegmentRun() const noexcept
    {
        const bool started =
            disposition == NCPathCoreLinkedCommittedSegmentRunDisposition::
            PROVEN_LINKED_SEGMENT_RUN_STARTED &&
            runRelation == NCPathCoreLinkedCommittedSegmentRunRelation::
            FIRST_PROVEN_PAIR_ESTABLISHES_TWO_SEGMENT_RUN;
        const bool extended =
            disposition == NCPathCoreLinkedCommittedSegmentRunDisposition::
            PROVEN_LINKED_SEGMENT_RUN_EXTENDED &&
            runRelation == NCPathCoreLinkedCommittedSegmentRunRelation::
            OVERLAPPING_PROVEN_PAIR_EXTENDS_EXISTING_RUN;

        if (publicationSequence == 0ULL ||
            sourcePairPublicationSequence == 0ULL ||
            sourcePairPublicationSequence != latestPairPublicationSequence ||
            runGeneration == 0ULL ||
            acceptedInputChainGeneration == 0ULL ||
            firstPairPublicationSequence == 0ULL ||
            latestPairPublicationSequence == 0ULL ||
            firstSegmentPublicationSequence == 0ULL ||
            latestSegmentPublicationSequence == 0ULL ||
            firstLinkPublicationSequence == 0ULL ||
            latestLinkPublicationSequence == 0ULL ||
            linkedSegmentRunLength < 2U ||
            firstAcceptedInputRunLength < 2U ||
            latestAcceptedInputRunLength < firstAcceptedInputRunLength ||
            participatingAxisUnionMask == 0U ||
            (participatingAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_AXIS_MASK) != 0U ||
            (coordinateChangeAxisUnionMask &
                ~participatingAxisUnionMask) != 0U ||
            latestSegmentAxisMask == 0U ||
            (latestSegmentAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_AXIS_MASK) != 0U ||
            (latestSegmentAxisMask & ~participatingAxisUnionMask) != 0U ||
            (latestCoordinateChangeAxisMask &
                ~latestSegmentAxisMask) != 0U ||
            (latestCoordinateChangeAxisMask &
                ~coordinateChangeAxisUnionMask) != 0U ||
            firstSourcePC < 0 ||
            latestSourcePC < firstSourcePC ||
            schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_SCHEMA_V1 ||
            (!started && !extended) ||
            extent != NCPathCoreLinkedCommittedSegmentRunExtent::
            SCALAR_LINKED_SEGMENT_RUN_SUMMARY_ONLY ||
            frame != NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE ||
            kind != NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR ||
            commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP ||
            !ReservedIsZero())
        {
            return false;
        }

        const std::int64_t sourcePcSpan =
            static_cast<std::int64_t>(latestSourcePC) -
            static_cast<std::int64_t>(firstSourcePC) + 1LL;
        if (sourcePcSpan <= 0LL ||
            static_cast<std::uint64_t>(sourcePcSpan) !=
            static_cast<std::uint64_t>(linkedSegmentRunLength))
        {
            return false;
        }

        if (latestAcceptedInputRunLength !=
            NCPathCoreLinkedCommittedSegmentRunDetail::
            LINKED_SEGMENT_RUN_LENGTH_MAX)
        {
            const std::uint64_t acceptedRunSpan =
                static_cast<std::uint64_t>(latestAcceptedInputRunLength) -
                static_cast<std::uint64_t>(firstAcceptedInputRunLength) +
                1ULL;
            if (acceptedRunSpan !=
                static_cast<std::uint64_t>(linkedSegmentRunLength))
            {
                return false;
            }
        }

        if (started)
        {
            return
                linkedSegmentRunLength == 2U &&
                firstPairPublicationSequence ==
                latestPairPublicationSequence &&
                latestSegmentPublicationSequence ==
                NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                    firstSegmentPublicationSequence) &&
                latestLinkPublicationSequence ==
                NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                    firstLinkPublicationSequence) &&
                latestAcceptedInputRunLength ==
                NCPathCoreDetail::
                SaturatingIncrementAcceptedHandoffRunLength(
                    firstAcceptedInputRunLength) &&
                static_cast<std::int64_t>(latestSourcePC) ==
                static_cast<std::int64_t>(firstSourcePC) + 1LL;
        }

        return
            linkedSegmentRunLength >= 3U &&
            firstPairPublicationSequence != latestPairPublicationSequence &&
            firstSegmentPublicationSequence !=
            latestSegmentPublicationSequence &&
            firstLinkPublicationSequence != latestLinkPublicationSequence;
    }

private:
    bool ReservedIsZero() const noexcept
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
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentRunShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunShadow() noexcept = default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_NOINLINE
        void ObserveLatestPairSameThread(
            const NCPathCoreLinkedCommittedSegmentPairRecordV1* pair) noexcept
    {
        const NCPathCoreLinkedCommittedSegmentRunRecordV1* const previousRun =
            GetNewestObservationSameThread();

        m_publicationSequence =
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_HISTORY_CAPACITY;

        // Write directly into the NCManager-owned fixed slot.  No L.2G record,
        // run record or endpoint array is copied onto the NC thread stack.
        NCPathCoreLinkedCommittedSegmentRunRecordV1& target =
            m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            pair == nullptr ? 0ULL : pair->publicationSequence);

        if (pair == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisposition::
                NOT_APPLICABLE_PAIR_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenPair(*pair))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisposition::
                NOT_APPLICABLE_PAIR_NOT_PROVEN;
        }
        else if (!pair->IsProvenImmediateLinkedSegmentContinuation() ||
            !ReservedIsZero(pair->reserved))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisposition::
                INVALID_PAIR_RECORD;
        }
        else if (previousRun == nullptr)
        {
            StartRun(*pair, target);
        }
        else if (previousRun->IsProvenLinkedCommittedSegmentRun())
        {
            if (IsDirectRunExtensionFenceValid(*previousRun, *pair))
            {
                ExtendRun(*previousRun, *pair, target);
            }
            else
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunDisposition::
                    INVALID_RUN_EXTENSION_FENCE;
            }
        }
        else if (IsStructurallyValidNonProvenRun(*previousRun))
        {
            StartRun(*pair, target);
        }
        else
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisposition::
                INVALID_PREVIOUS_RUN_RECORD;
        }

        // Every neutral or invalid result becomes newest, so an older proven
        // scalar run can never masquerade as the current Path Core state.
        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRunRecordV1*
        GetNewestObservationSameThread(
            std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX ||
            historyOffset >= m_recordCount)
        {
            return nullptr;
        }

        const std::size_t index =
            (static_cast<std::size_t>(m_latestIndex) +
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunRecordV1& target,
        std::uint64_t publicationSequence,
        std::uint64_t sourcePairPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourcePairPublicationSequence = sourcePairPublicationSequence;
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.firstPairPublicationSequence = 0ULL;
        target.latestPairPublicationSequence = 0ULL;
        target.firstSegmentPublicationSequence = 0ULL;
        target.latestSegmentPublicationSequence = 0ULL;
        target.firstLinkPublicationSequence = 0ULL;
        target.latestLinkPublicationSequence = 0ULL;
        target.linkedSegmentRunLength = 0U;
        target.firstAcceptedInputRunLength = 0U;
        target.latestAcceptedInputRunLength = 0U;
        target.participatingAxisUnionMask = 0U;
        target.coordinateChangeAxisUnionMask = 0U;
        target.latestSegmentAxisMask = 0U;
        target.latestCoordinateChangeAxisMask = 0U;
        target.firstSourcePC = -1;
        target.latestSourcePC = -1;
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_UNAVAILABLE;
        target.runRelation =
            NCPathCoreLinkedCommittedSegmentRunRelation::NONE;
        target.extent = NCPathCoreLinkedCommittedSegmentRunExtent::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);
    }

    static bool ReservedIsZero(
        const std::array<std::uint8_t, 3U>& reserved) noexcept
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

    static bool IsStructurallyValidNonProvenPair(
        const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair) noexcept
    {
        if (pair.publicationSequence == 0ULL ||
            pair.schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_SCHEMA_V1 ||
            pair.pairRelation !=
            NCPathCoreLinkedCommittedSegmentPairRelation::NONE ||
            pair.extent !=
            NCPathCoreLinkedCommittedSegmentPairExtent::NONE ||
            pair.frame != NCPathCoreCommittedGeometryFrame::NONE ||
            pair.kind != NCPathCoreCommittedGeometryKind::NONE ||
            pair.commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE ||
            (pair.junctionNumericMismatchAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_AXIS_MASK) != 0U ||
            (pair.junctionRepresentationDifferenceAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_AXIS_MASK) != 0U ||
            !ReservedIsZero(pair.reserved))
        {
            return false;
        }

        switch (pair.disposition)
        {
            case NCPathCoreLinkedCommittedSegmentPairDisposition::
            NOT_PROVEN_JUNCTION_GEOMETRY_MISMATCH:
                return
                    pair.numericRelation ==
                    NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
                    NUMERIC_MISMATCH &&
                    pair.junctionNumericMismatchAxisMask != 0U;

                case NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_APPLICABLE_INSUFFICIENT_HISTORY:
                    case NCPathCoreLinkedCommittedSegmentPairDisposition::
                    NOT_APPLICABLE_PREVIOUS_SEGMENT_NOT_FORMED:
                        case NCPathCoreLinkedCommittedSegmentPairDisposition::
                        NOT_APPLICABLE_CURRENT_SEGMENT_NOT_FORMED:
                            case NCPathCoreLinkedCommittedSegmentPairDisposition::
                            INVALID_PREVIOUS_SEGMENT_RECORD:
                                case NCPathCoreLinkedCommittedSegmentPairDisposition::
                                INVALID_CURRENT_SEGMENT_RECORD:
                                    case NCPathCoreLinkedCommittedSegmentPairDisposition::
                                    INVALID_DIRECT_PAIR_FENCE:
                                        return
                                            pair.numericRelation ==
                                            NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
                                            NOT_EVALUATED &&
                                            pair.junctionNumericMismatchAxisMask == 0U &&
                                            pair.junctionRepresentationDifferenceAxisMask == 0U;

                                    case NCPathCoreLinkedCommittedSegmentPairDisposition::EMPTY:
                                        case NCPathCoreLinkedCommittedSegmentPairDisposition::
                                        PROVEN_IMMEDIATE_LINKED_SEGMENT_CONTINUATION:
                                        default:
                                            return false;
        }
    }

    static bool IsStructurallyValidNonProvenRun(
        const NCPathCoreLinkedCommittedSegmentRunRecordV1& run) noexcept
    {
        const bool sourcePairExpected =
            run.disposition !=
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_UNAVAILABLE;
        const bool knownDisposition =
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_UNAVAILABLE ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_NOT_PROVEN ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            INVALID_PAIR_RECORD ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            INVALID_PREVIOUS_RUN_RECORD ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            INVALID_RUN_EXTENSION_FENCE;

        if (!knownDisposition ||
            run.publicationSequence == 0ULL ||
            run.schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_SCHEMA_V1 ||
            (sourcePairExpected &&
                run.sourcePairPublicationSequence == 0ULL) ||
            (!sourcePairExpected &&
                run.sourcePairPublicationSequence != 0ULL) ||
            run.runGeneration != 0ULL ||
            run.acceptedInputChainGeneration != 0ULL ||
            run.firstPairPublicationSequence != 0ULL ||
            run.latestPairPublicationSequence != 0ULL ||
            run.firstSegmentPublicationSequence != 0ULL ||
            run.latestSegmentPublicationSequence != 0ULL ||
            run.firstLinkPublicationSequence != 0ULL ||
            run.latestLinkPublicationSequence != 0ULL ||
            run.linkedSegmentRunLength != 0U ||
            run.firstAcceptedInputRunLength != 0U ||
            run.latestAcceptedInputRunLength != 0U ||
            run.participatingAxisUnionMask != 0U ||
            run.coordinateChangeAxisUnionMask != 0U ||
            run.latestSegmentAxisMask != 0U ||
            run.latestCoordinateChangeAxisMask != 0U ||
            run.firstSourcePC != -1 ||
            run.latestSourcePC != -1 ||
            run.runRelation !=
            NCPathCoreLinkedCommittedSegmentRunRelation::NONE ||
            run.extent != NCPathCoreLinkedCommittedSegmentRunExtent::NONE ||
            run.frame != NCPathCoreCommittedGeometryFrame::NONE ||
            run.kind != NCPathCoreCommittedGeometryKind::NONE ||
            run.commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE)
        {
            return false;
        }

        for (std::size_t index = 0U; index < run.reserved.size(); ++index)
        {
            if (run.reserved[index] != 0U)
            {
                return false;
            }
        }
        return true;
    }

    static bool IsDirectRunExtensionFenceValid(
        const NCPathCoreLinkedCommittedSegmentRunRecordV1& previous,
        const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair) noexcept
    {
        return
            previous.sourcePairPublicationSequence ==
            previous.latestPairPublicationSequence &&
            pair.publicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previous.latestPairPublicationSequence) &&
            pair.acceptedInputChainGeneration ==
            previous.acceptedInputChainGeneration &&
            pair.previousSegmentPublicationSequence ==
            previous.latestSegmentPublicationSequence &&
            pair.previousLinkPublicationSequence ==
            previous.latestLinkPublicationSequence &&
            pair.previousTerminalAxisMask ==
            previous.latestSegmentAxisMask &&
            pair.previousCoordinateChangeAxisMask ==
            previous.latestCoordinateChangeAxisMask &&
            pair.previousAcceptedInputRunLength ==
            previous.latestAcceptedInputRunLength &&
            pair.previousSourcePC == previous.latestSourcePC;
    }

    void StartRun(
        const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair,
        NCPathCoreLinkedCommittedSegmentRunRecordV1& target) noexcept
    {
        m_runGenerationSequence =
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                m_runGenerationSequence);
        target.runGeneration = m_runGenerationSequence;
        target.acceptedInputChainGeneration =
            pair.acceptedInputChainGeneration;
        target.firstPairPublicationSequence = pair.publicationSequence;
        target.latestPairPublicationSequence = pair.publicationSequence;
        target.firstSegmentPublicationSequence =
            pair.previousSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            pair.currentSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            pair.previousLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            pair.currentLinkPublicationSequence;
        target.linkedSegmentRunLength = 2U;
        target.firstAcceptedInputRunLength =
            pair.previousAcceptedInputRunLength;
        target.latestAcceptedInputRunLength =
            pair.currentAcceptedInputRunLength;
        target.participatingAxisUnionMask =
            pair.previousTerminalAxisMask | pair.currentSegmentAxisMask;
        target.coordinateChangeAxisUnionMask =
            pair.previousCoordinateChangeAxisMask |
            pair.currentCoordinateChangeAxisMask;
        target.latestSegmentAxisMask = pair.currentSegmentAxisMask;
        target.latestCoordinateChangeAxisMask =
            pair.currentCoordinateChangeAxisMask;
        target.firstSourcePC = pair.previousSourcePC;
        target.latestSourcePC = pair.currentSourcePC;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            PROVEN_LINKED_SEGMENT_RUN_STARTED;
        target.runRelation =
            NCPathCoreLinkedCommittedSegmentRunRelation::
            FIRST_PROVEN_PAIR_ESTABLISHES_TWO_SEGMENT_RUN;
        ApplyProvenSemanticMetadata(target);
    }

    static void ExtendRun(
        const NCPathCoreLinkedCommittedSegmentRunRecordV1& previous,
        const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair,
        NCPathCoreLinkedCommittedSegmentRunRecordV1& target) noexcept
    {
        target.runGeneration = previous.runGeneration;
        target.acceptedInputChainGeneration =
            previous.acceptedInputChainGeneration;
        target.firstPairPublicationSequence =
            previous.firstPairPublicationSequence;
        target.latestPairPublicationSequence = pair.publicationSequence;
        target.firstSegmentPublicationSequence =
            previous.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            pair.currentSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            previous.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            pair.currentLinkPublicationSequence;
        target.linkedSegmentRunLength =
            NCPathCoreLinkedCommittedSegmentRunDetail::
            SaturatingIncrementRunLength(
                previous.linkedSegmentRunLength);
        target.firstAcceptedInputRunLength =
            previous.firstAcceptedInputRunLength;
        target.latestAcceptedInputRunLength =
            pair.currentAcceptedInputRunLength;
        target.participatingAxisUnionMask =
            previous.participatingAxisUnionMask |
            pair.previousTerminalAxisMask |
            pair.currentSegmentAxisMask;
        target.coordinateChangeAxisUnionMask =
            previous.coordinateChangeAxisUnionMask |
            pair.previousCoordinateChangeAxisMask |
            pair.currentCoordinateChangeAxisMask;
        target.latestSegmentAxisMask = pair.currentSegmentAxisMask;
        target.latestCoordinateChangeAxisMask =
            pair.currentCoordinateChangeAxisMask;
        target.firstSourcePC = previous.firstSourcePC;
        target.latestSourcePC = pair.currentSourcePC;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            PROVEN_LINKED_SEGMENT_RUN_EXTENDED;
        target.runRelation =
            NCPathCoreLinkedCommittedSegmentRunRelation::
            OVERLAPPING_PROVEN_PAIR_EXTENDS_EXISTING_RUN;
        ApplyProvenSemanticMetadata(target);
    }

    static void ApplyProvenSemanticMetadata(
        NCPathCoreLinkedCommittedSegmentRunRecordV1& target) noexcept
    {
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunExtent::
            SCALAR_LINKED_SEGMENT_RUN_SUMMARY_ONLY;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint64_t m_runGenerationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_NOINLINE

static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunDisposition) == 1U,
    "Linked committed segment-run disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunRelation) == 1U,
    "Linked committed segment-run relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunExtent) == 1U,
    "Linked committed segment-run extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunRecordV1>::value,
    "Linked committed segment-run record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunRecordV1>::value,
    "Linked committed segment-run record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreLinkedCommittedSegmentRunRecordV1) == 8U,
    "Linked committed segment-run record alignment changed.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunRecordV1) == 128U,
    "Linked committed segment-run record must remain exactly 128 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunRecordV1,
        linkedSegmentRunLength) == 80U,
    "Linked committed segment-run length offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunRecordV1,
        schemaVersion) == 116U,
    "Linked committed segment-run schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunRecordV1,
        reserved) == 124U,
    "Linked committed segment-run reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunShadow>::value,
    "Linked committed segment-run shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunShadow>::value,
    "Linked committed segment-run shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunShadow) == 280U,
    "Linked committed segment-run shadow must remain exactly 280 bytes.");
