#pragma once

#include "NCPathCoreInputHandoffCompactShadow.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================
// NC-0.2L.2D / Committed Geometry Descriptor Contract Shadow
//
// This fixed two-record history observes the commanded endpoint pair of an
// ordinary no-P G00 only after the existing K.7 registry and commit proofs
// have accepted it.  It is diagnostic history on the existing NC producer
// thread, not a queue, path planner, Runtime permit or Motion command source.
//
// startMCS and endMCS use runtime axis slots 0..7 in each axis's native MCS
// unit (linear axes: mm; rotary axes: degrees).  The pair does not establish
// travel direction, rotary unwrapping, path length, interpolation, timing,
// STARTED/EXECUTED/COMPLETED state, actual position or a B2 breadcrumb.
// acceptedInputPairRelation, run length and chain generation describe only
// accepted-input provenance continuity.  They do not prove endpoint equality,
// C0 geometry continuity, execution adjacency or executed-path history.
// axisMask identifies the runtime axis slots submitted by this G00.  An
// unmasked slot is the carried-forward commanded queue tail, not another
// moving axis; it must therefore remain identical across this endpoint pair.
// =============================================================

constexpr std::size_t NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY = 8U;
constexpr std::size_t NC_PATH_CORE_COMMITTED_GEOMETRY_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_MASK = 0xFFU;
constexpr std::uint16_t NC_PATH_CORE_COMMITTED_GEOMETRY_SCHEMA_V1 = 1U;

namespace NCPathCoreCommittedGeometryDetail
{
    constexpr std::uint64_t NextNonZero(std::uint64_t current) noexcept
    {
        return current == UINT64_MAX ? 1ULL : current + 1ULL;
    }
}

enum class NCPathCoreCommittedGeometryKind : std::uint8_t
{
    NONE = 0U,
    ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR = 1U
};

enum class NCPathCoreCommittedGeometryFrame : std::uint8_t
{
    NONE = 0U,
    MCS_AXIS_NATIVE = 1U
};

enum class NCPathCoreCommittedGeometryCommandPathPolicy : std::uint8_t
{
    NONE = 0U,
    EXACT_STOP = 1U
};

enum class NCPathCoreCommittedGeometryExtent : std::uint8_t
{
    NONE = 0U,
    COMMANDED_ENDPOINT_PAIR_ONLY = 1U
};

enum class NCPathCoreCommittedGeometryValidity : std::uint8_t
{
    EMPTY = 0U,
    ACCEPTED_COMMAND_COMMITTED = 1U,
    INVALID_FENCE = 2U,
    INVALID_COORDINATES = 3U
};

struct NCPathCoreOrdinaryG00CommittedEndpointPairV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t preparedSession = 0ULL;
    std::uint64_t preparedEntrySequence = 0ULL;
    std::uint64_t dispatchId = 0ULL;
    std::uint64_t commitSequence = 0ULL;
    std::uint64_t motionExecutionEpoch = 0ULL;
    std::uint64_t motionSegmentId = 0ULL;
    std::uint64_t queueTailTransactionSequence = 0ULL;
    std::uint64_t queueTailBeforeFingerprint = 0ULL;
    std::uint64_t queueTailCommittedFingerprint = 0ULL;

    std::array<
        double,
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY> startMCS{};
    std::array<
        double,
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY> endMCS{};

    std::uint32_t axisMask = 0U;
    std::uint32_t acceptedInputRunLength = 0U;
    std::int32_t sourcePC = -1;
    std::int32_t sourceLineNumber = 0;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    NCPathCoreCommittedGeometryExtent extent =
        NCPathCoreCommittedGeometryExtent::NONE;
    NCPathCoreCommittedGeometryValidity validity =
        NCPathCoreCommittedGeometryValidity::EMPTY;
    NCPathCoreAcceptedInputPairRelation acceptedInputPairRelation =
        NCPathCoreAcceptedInputPairRelation::NONE;

    // "Committed" here means Motion producer acceptance, transactional queue
    // tail commit and NC Program Commit proof.  It is not Motion consumer
    // acceptance, STARTED, terminal feedback or physical execution.
    bool IsAcceptedCommandCommitted() const noexcept
    {
        return
            validity == NCPathCoreCommittedGeometryValidity::
            ACCEPTED_COMMAND_COMMITTED &&
            schemaVersion == NC_PATH_CORE_COMMITTED_GEOMETRY_SCHEMA_V1 &&
            kind == NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR &&
            frame == NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE &&
            commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP &&
            extent == NCPathCoreCommittedGeometryExtent::
            COMMANDED_ENDPOINT_PAIR_ONLY;
    }
};

class NCPathCoreCommittedGeometryShadow final
{
public:
    NCPathCoreCommittedGeometryShadow() noexcept = default;

    void ObserveAcceptedOrdinaryG00CommittedEndpointPair(
        const NCPathCoreAcceptedInputRecord* acceptedInput,
        NCPathCoreAcceptedInputPairRelation pairRelation,
        std::uint32_t acceptedInputRunLength,
        std::uint64_t queueTailTransactionSequence,
        std::uint32_t axisMask,
        std::uint64_t queueTailBeforeFingerprint,
        std::uint64_t queueTailCommittedFingerprint,
        const double(&startMCS)
        [NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY],
        const double(&endMCS)
        [NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY],
        bool upstreamGeometryFenceValid) noexcept
    {
        const bool pairRunValid = IsPairRunValid(
            pairRelation,
            acceptedInputRunLength);
        if (pairRunValid &&
            (pairRelation ==
                NCPathCoreAcceptedInputPairRelation::FIRST_INPUT ||
                pairRelation ==
                NCPathCoreAcceptedInputPairRelation::CHAIN_BOUNDARY))
        {
            m_acceptedInputChainGeneration =
                NCPathCoreCommittedGeometryDetail::NextNonZero(
                    m_acceptedInputChainGeneration);
            m_acceptedInputChainOpen = 1U;
        }
        else if (!pairRunValid)
        {
            // Do not allow a later CONTIGUOUS observation to bridge across a
            // malformed accepted-input relation/run fence.  A subsequent
            // valid FIRST or BOUNDARY observation opens a new generation.
            m_acceptedInputChainOpen = 0U;
        }

        m_publicationSequence =
            NCPathCoreCommittedGeometryDetail::NextNonZero(
                m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_COMMITTED_GEOMETRY_HISTORY_CAPACITY;

        // Write directly into the NCManager-owned fixed workspace.  Do not
        // create a 240-byte descriptor temporary on the NC thread stack.
        NCPathCoreOrdinaryG00CommittedEndpointPairV1& target =
            m_records[targetIndex];
        target.publicationSequence = m_publicationSequence;
        target.acceptedInputChainGeneration =
            m_acceptedInputChainGeneration;
        target.preparedSession =
            acceptedInput == nullptr
            ? 0ULL
            : acceptedInput->preparedSession;
        target.preparedEntrySequence =
            acceptedInput == nullptr
            ? 0ULL
            : acceptedInput->preparedEntrySequence;
        target.dispatchId =
            acceptedInput == nullptr ? 0ULL : acceptedInput->dispatchId;
        target.commitSequence =
            acceptedInput == nullptr ? 0ULL : acceptedInput->commitSequence;
        target.motionExecutionEpoch =
            acceptedInput == nullptr
            ? 0ULL
            : acceptedInput->motionExecutionEpoch;
        target.motionSegmentId =
            acceptedInput == nullptr
            ? 0ULL
            : acceptedInput->motionSegmentId;
        target.queueTailTransactionSequence =
            queueTailTransactionSequence;
        target.queueTailBeforeFingerprint = queueTailBeforeFingerprint;
        target.queueTailCommittedFingerprint =
            queueTailCommittedFingerprint;

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            target.startMCS[axis] = startMCS[axis];
            target.endMCS[axis] = endMCS[axis];
        }

        target.axisMask = axisMask;
        target.acceptedInputRunLength = acceptedInputRunLength;
        target.sourcePC =
            acceptedInput == nullptr ? -1 : acceptedInput->sourcePC;
        target.sourceLineNumber =
            acceptedInput == nullptr
            ? 0
            : acceptedInput->sourceLineNumber;
        target.schemaVersion = NC_PATH_CORE_COMMITTED_GEOMETRY_SCHEMA_V1;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
        target.extent = NCPathCoreCommittedGeometryExtent::
            COMMANDED_ENDPOINT_PAIR_ONLY;
        target.acceptedInputPairRelation = pairRelation;

        const bool fenceValid =
            upstreamGeometryFenceValid &&
            acceptedInput != nullptr &&
            acceptedInput->preparedSession != 0ULL &&
            acceptedInput->preparedEntrySequence != 0ULL &&
            acceptedInput->dispatchId != 0ULL &&
            acceptedInput->commitSequence != 0ULL &&
            acceptedInput->motionExecutionEpoch != 0ULL &&
            acceptedInput->motionSegmentId != 0ULL &&
            acceptedInput->sourcePC >= 0 &&
            queueTailTransactionSequence != 0ULL &&
            axisMask != 0U &&
            (axisMask & ~NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_MASK) == 0U &&
            pairRunValid &&
            m_acceptedInputChainGeneration != 0ULL &&
            m_acceptedInputChainOpen != 0U;
        const bool coordinatesValid =
            AreCoordinatesValid(startMCS, endMCS, axisMask);

        target.validity =
            !fenceValid
            ? NCPathCoreCommittedGeometryValidity::INVALID_FENCE
            : !coordinatesValid
            ? NCPathCoreCommittedGeometryValidity::INVALID_COORDINATES
            : NCPathCoreCommittedGeometryValidity::
            ACCEPTED_COMMAND_COMMITTED;

        // Invalid observations are retained as the newest record.  This
        // prevents an older valid descriptor from masquerading as the latest
        // accepted handoff and preserves accepted-chain boundaries even when
        // the coordinate image itself is invalid.
        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_COMMITTED_GEOMETRY_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreOrdinaryG00CommittedEndpointPairV1*
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
                NC_PATH_CORE_COMMITTED_GEOMETRY_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_COMMITTED_GEOMETRY_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static bool IsPairRunValid(
        NCPathCoreAcceptedInputPairRelation pairRelation,
        std::uint32_t acceptedInputRunLength) noexcept
    {
        return
            ((pairRelation ==
                NCPathCoreAcceptedInputPairRelation::FIRST_INPUT ||
                pairRelation ==
                NCPathCoreAcceptedInputPairRelation::CHAIN_BOUNDARY) &&
                acceptedInputRunLength == 1U) ||
            (pairRelation ==
                NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR &&
                acceptedInputRunLength >= 2U);
    }

    static bool AreCoordinatesValid(
        const double(&startMCS)
        [NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY],
        const double(&endMCS)
        [NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY],
        std::uint32_t axisMask) noexcept
    {
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (!std::isfinite(startMCS[axis]) ||
                !std::isfinite(endMCS[axis]))
            {
                return false;
            }

            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if ((axisMask & axisBit) == 0U &&
                startMCS[axis] != endMCS[axis])
            {
                return false;
            }
        }
        return true;
    }

    std::array<
        NCPathCoreOrdinaryG00CommittedEndpointPairV1,
        NC_PATH_CORE_COMMITTED_GEOMETRY_HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint64_t m_acceptedInputChainGeneration = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::uint8_t m_acceptedInputChainOpen = 0U;
};

static_assert(
    sizeof(NCPathCoreCommittedGeometryKind) == 1U,
    "Path Core committed geometry kind must remain one byte.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryFrame) == 1U,
    "Path Core committed geometry frame must remain one byte.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryCommandPathPolicy) == 1U,
    "Path Core command path policy must remain one byte.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryExtent) == 1U,
    "Path Core geometry extent must remain one byte.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryValidity) == 1U,
    "Path Core geometry validity must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreOrdinaryG00CommittedEndpointPairV1>::value,
    "Path Core committed geometry record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreOrdinaryG00CommittedEndpointPairV1>::value,
    "Path Core committed geometry record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreOrdinaryG00CommittedEndpointPairV1) == 8U,
    "Path Core committed geometry record alignment changed.");
static_assert(
    sizeof(NCPathCoreOrdinaryG00CommittedEndpointPairV1) == 240U,
    "Path Core committed geometry record must remain exactly 240 bytes.");
static_assert(
    offsetof(NCPathCoreOrdinaryG00CommittedEndpointPairV1, startMCS) == 88U,
    "Path Core committed geometry start offset changed.");
static_assert(
    offsetof(NCPathCoreOrdinaryG00CommittedEndpointPairV1, endMCS) == 152U,
    "Path Core committed geometry end offset changed.");
static_assert(
    offsetof(NCPathCoreOrdinaryG00CommittedEndpointPairV1, axisMask) == 216U,
    "Path Core committed geometry mask offset changed.");
static_assert(
    offsetof(
        NCPathCoreOrdinaryG00CommittedEndpointPairV1,
        acceptedInputRunLength) == 220U,
    "Path Core committed geometry run offset changed.");
static_assert(
    offsetof(NCPathCoreOrdinaryG00CommittedEndpointPairV1, sourcePC) == 224U,
    "Path Core committed geometry source PC offset changed.");
static_assert(
    offsetof(
        NCPathCoreOrdinaryG00CommittedEndpointPairV1,
        schemaVersion) == 232U,
    "Path Core committed geometry schema offset changed.");
static_assert(
    std::is_standard_layout<NCPathCoreCommittedGeometryShadow>::value,
    "Path Core committed geometry shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<NCPathCoreCommittedGeometryShadow>::value,
    "Path Core committed geometry shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryShadow) == 504U,
    "Path Core committed geometry shadow must remain exactly 504 bytes.");

