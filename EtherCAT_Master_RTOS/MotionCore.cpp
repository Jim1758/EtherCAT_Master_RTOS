#include "MotionCore.h"
#include <algorithm> // for std::abs, std::sqrt, std::max, std::min
#include <cstring>
#include <iostream>  // for debug prints if needed
#include <limits>
#include "EtherCatMaster.h"
#include "CoordinateManager.h"
#include "SHM_Types.h"
#include "AlarmManager.h"

namespace
{
    constexpr std::uint64_t EXECUTION_EPOCH_PUBLICATION_EPOCH_MASK =
        0x00000000FFFFFFFFULL;
    constexpr std::uint64_t EXECUTION_EPOCH_PUBLICATION_ABORT_ACTIVE =
        0x0000000100000000ULL;
    constexpr unsigned EXECUTION_EPOCH_PUBLICATION_SOURCE_SHIFT = 33U;
    constexpr std::uint64_t EXECUTION_EPOCH_PUBLICATION_SOURCE_MASK =
        0x000001FE00000000ULL;
    constexpr std::uint64_t EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED =
        0x4000000000000000ULL;
    constexpr std::uint64_t EXECUTION_EPOCH_PUBLICATION_PENDING =
        0x8000000000000000ULL;

    std::uint64_t PackExecutionEpochPublication(
        MotionExecutionEpoch executionEpoch,
        MotionCommandSource source,
        bool abortActiveCommand,
        bool pending) noexcept
    {
        return
            static_cast<std::uint64_t>(executionEpoch) |
            (abortActiveCommand
                ? EXECUTION_EPOCH_PUBLICATION_ABORT_ACTIVE
                : 0ULL) |
            ((static_cast<std::uint64_t>(source) <<
                EXECUTION_EPOCH_PUBLICATION_SOURCE_SHIFT) &
                EXECUTION_EPOCH_PUBLICATION_SOURCE_MASK) |
            (pending
                ? EXECUTION_EPOCH_PUBLICATION_PENDING
                : 0ULL);
    }

    MotionExecutionEpoch UnpackExecutionEpochPublication(
        std::uint64_t publication) noexcept
    {
        return static_cast<MotionExecutionEpoch>(
            publication & EXECUTION_EPOCH_PUBLICATION_EPOCH_MASK);
    }

    MotionCommandSource UnpackExecutionEpochPublicationSource(
        std::uint64_t publication) noexcept
    {
        return static_cast<MotionCommandSource>(
            (publication & EXECUTION_EPOCH_PUBLICATION_SOURCE_MASK) >>
            EXECUTION_EPOCH_PUBLICATION_SOURCE_SHIFT);
    }

    constexpr std::uint64_t RESET_SAFETY_BATCH_EPOCH_MASK =
        0x00000000FFFFFFFFULL;
    constexpr std::uint64_t RESET_SAFETY_BATCH_RESET_FAULTS =
        0x0000000100000000ULL;
    constexpr std::uint64_t RESET_SAFETY_BATCH_PRESENT =
        0x8000000000000000ULL;

    constexpr std::uint64_t P1_MAPPING_ALARM_EPOCH_MASK =
        0x00000000FFFFFFFFULL;
    constexpr std::uint64_t P1_MAPPING_ALARM_SEQUENCE_MASK =
        0x7FFFFFFF00000000ULL;
    constexpr unsigned P1_MAPPING_ALARM_SEQUENCE_SHIFT = 32U;
    constexpr std::uint64_t P1_MAPPING_ALARM_PENDING =
        0x8000000000000000ULL;
    constexpr std::uint64_t P1_MAPPING_ALARM_SEQUENCE_MAX =
        0x7FFFFFFFULL;

    std::uint64_t PackResetSafetyBatch(
        MotionExecutionEpoch publishedEpoch,
        bool requestResetAllFaults) noexcept
    {
        return
            RESET_SAFETY_BATCH_PRESENT |
            static_cast<std::uint64_t>(publishedEpoch) |
            (requestResetAllFaults
                ? RESET_SAFETY_BATCH_RESET_FAULTS
                : 0ULL);
    }

    MotionExecutionEpoch UnpackResetSafetyBatchEpoch(
        std::uint64_t packed) noexcept
    {
        return static_cast<MotionExecutionEpoch>(
            packed & RESET_SAFETY_BATCH_EPOCH_MASK);
    }

    bool MotionExecutionIdentityExactlyMatches(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
    }

    bool IsFiniteNearlyEqual(
        double lhs,
        double rhs,
        double tolerance = 1.0e-12) noexcept
    {
        return
            std::isfinite(lhs) &&
            std::isfinite(rhs) &&
            std::abs(lhs - rhs) <= tolerance;
    }

    int ClampMotionAxisCount(int axisCount) noexcept
    {
        return (std::max)(0, (std::min)(axisCount, MAX_AXES));
    }

    bool MotionCommandHasAxis(
        const MotionCommand& command,
        int axisIndex) noexcept
    {
        const int axisCount = ClampMotionAxisCount(command.axisCount);
        for (int slot = 0; slot < axisCount; ++slot)
        {
            if (command.axisIndices[slot] == axisIndex)
            {
                return true;
            }
        }
        return false;
    }

    bool MotionCommandsHaveIdenticalAxisMapping(
        const MotionCommand& lhs,
        const MotionCommand& rhs) noexcept
    {
        const int lhsAxisCount = ClampMotionAxisCount(lhs.axisCount);
        const int rhsAxisCount = ClampMotionAxisCount(rhs.axisCount);
        if (lhsAxisCount == 0 ||
            lhsAxisCount != lhs.axisCount ||
            rhsAxisCount != rhs.axisCount ||
            lhsAxisCount != rhsAxisCount)
        {
            return false;
        }

        for (int slot = 0; slot < lhsAxisCount; ++slot)
        {
            if (lhs.axisIndices[slot] != rhs.axisIndices[slot])
            {
                return false;
            }
        }
        return true;
    }

    std::uint32_t BuildMotionCommandAxisMask(
        const MotionCommand& command) noexcept
    {
        std::uint32_t mask = 0U;
        const int axisCount = ClampMotionAxisCount(command.axisCount);
        for (int slot = 0; slot < axisCount; ++slot)
        {
            const int axisIndex = command.axisIndices[slot];
            if (axisIndex >= 0 && axisIndex < 32)
            {
                mask |= 1U << static_cast<unsigned>(axisIndex);
            }
        }
        return mask;
    }

    bool IsMotionCommandConsumerGeometryValid(
        const MotionCommand& command,
        const std::vector<AxisContext>* contexts) noexcept
    {
        if (contexts == nullptr ||
            command.axisCount <= 0 ||
            command.axisCount > MAX_AXES ||
            (command.mode != InterpolationMode::LINEAR &&
                command.axisCount < 2) ||
            !std::isfinite(command.targetVel) ||
            !std::isfinite(command.accTime) ||
            !std::isfinite(command.decTime))
        {
            return false;
        }

        std::array<bool, MAX_AXES> seen{};
        for (int slot = 0; slot < command.axisCount; ++slot)
        {
            const int axisIndex = command.axisIndices[slot];
            if (axisIndex < 0 ||
                axisIndex >= MAX_AXES ||
                axisIndex >= static_cast<int>(contexts->size()) ||
                seen[static_cast<std::size_t>(axisIndex)] ||
                !(*contexts)[axisIndex].isExist ||
                !std::isfinite(command.targetPos[slot]))
            {
                return false;
            }
            seen[static_cast<std::size_t>(axisIndex)] = true;
        }

        if (command.mode == InterpolationMode::CIRCULAR_CW ||
            command.mode == InterpolationMode::CIRCULAR_CCW)
        {
            return
                (command.axisCount == 2 || command.axisCount == 3) &&
                std::isfinite(command.centerPos[0]) &&
                std::isfinite(command.centerPos[1]);
        }
        return command.mode == InterpolationMode::LINEAR;
    }

    bool IsIncomingPhysicalAxisReadyForGroup(
        const AxisContext& axis) noexcept
    {
        const double followingError =
            std::abs(axis.currentCmdPos - axis.currentActPos);
        return
            axis.isExist &&
            axis.isServoOn &&
            axis.startupLagMonitorArmed &&
            !axis.startupLagPrematureMotionBlocked &&
            !axis.isFault &&
            !axis.isLagAlarm &&
            axis.state == MotionState::MotionState_IDLE &&
            axis.inPosition &&
            std::isfinite(axis.currentCmdPos) &&
            std::isfinite(axis.currentActPos) &&
            std::isfinite(axis.logicalCmdPos) &&
            std::isfinite(axis.planningPos) &&
            std::isfinite(axis.finalTargetPos) &&
            std::isfinite(axis.currentCmdVel) &&
            std::isfinite(axis.logicalCmdVel) &&
            std::isfinite(axis.targetVelocity) &&
            std::isfinite(axis.targetEndVel) &&
            std::isfinite(axis.inPositionWindow_Pulse) &&
            axis.inPositionWindow_Pulse > 0.0 &&
            std::abs(axis.currentCmdVel) <= 1.0 &&
            std::abs(axis.logicalCmdVel) <= 1.0 &&
            std::abs(axis.targetVelocity) <= 1.0 &&
            std::abs(axis.targetEndVel) <= 1.0 &&
            std::isfinite(followingError) &&
            followingError <= axis.inPositionWindow_Pulse;
    }

    bool IsMotionCommandHistorySnapshotValid(
        const MotionCommand& command) noexcept
    {
        const int axisCount = ClampMotionAxisCount(command.axisCount);
        if (axisCount <= 0 ||
            axisCount != command.axisCount ||
            !std::isfinite(command.mem_totalDist) ||
            command.mem_totalDist <= 0.0)
        {
            return false;
        }

        for (int slot = 0; slot < axisCount; ++slot)
        {
            if (!std::isfinite(command.mem_startPos[slot]) ||
                !std::isfinite(command.mem_ratio[slot]))
            {
                return false;
            }
        }

        if (command.mem_enableTransform)
        {
            for (int row = 0; row < 3; ++row)
            {
                if (!std::isfinite(command.mem_transformOrigin[row]))
                {
                    return false;
                }
                for (int column = 0; column < 3; ++column)
                {
                    if (!std::isfinite(
                        command.mem_transformMatrix[row][column]))
                    {
                        return false;
                    }
                }
            }
        }

        if (command.mode == InterpolationMode::CIRCULAR_CW ||
            command.mode == InterpolationMode::CIRCULAR_CCW)
        {
            return
                std::isfinite(command.startRadius) &&
                std::isfinite(command.endRadius) &&
                std::isfinite(command.mem_radius) &&
                std::isfinite(command.mem_startAngle) &&
                std::isfinite(command.mem_centerX) &&
                std::isfinite(command.mem_centerY) &&
                std::isfinite(command.mem_totalAngle);
        }
        return command.mode == InterpolationMode::LINEAR;
    }

    // IDLE is a command-state invariant, not merely an enum value.  Only the
    // 250 us Runtime calls this helper.  Non-finite state is deliberately left
    // untouched so the NC-0.2J.5 proof remains fail-closed.
    bool TryCanonicalizeIdleAxisCommandState(
        AxisContext& axis) noexcept
    {
        if (axis.state != MotionState::MotionState_IDLE ||
            axis.isFault ||
            axis.isLagAlarm ||
            !std::isfinite(axis.currentCmdPos) ||
            !std::isfinite(axis.planningPos) ||
            !std::isfinite(axis.finalTargetPos) ||
            !std::isfinite(axis.currentCmdVel) ||
            !std::isfinite(axis.logicalCmdVel) ||
            !std::isfinite(axis.targetVelocity) ||
            !std::isfinite(axis.targetEndVel))
        {
            return false;
        }

        axis.planningPos = axis.currentCmdPos;
        axis.finalTargetPos = axis.currentCmdPos;
        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        std::fill(axis.velBuffer.begin(), axis.velBuffer.end(), 0.0);
        axis.bufferSum = 0.0;
        axis.bufferIndex = 0;
        axis.inPosition = true;
        return true;
    }

    bool CanCanonicalizeInactivePhysicalAxisCommandState(
        const AxisContext& axis) noexcept
    {
        const bool groupOwnedState =
            axis.state == MotionState::MotionState_INTERPOLATING ||
            axis.state == MotionState::MotionState_STOPPING ||
            axis.state == MotionState::MotionState_IDLE;

        if (!groupOwnedState ||
            axis.isFault ||
            axis.isLagAlarm ||
            axis.state == MotionState::MotionState_ERROR ||
            axis.state == MotionState::MotionState_ESTOP ||
            !std::isfinite(axis.currentCmdPos) ||
            !std::isfinite(axis.currentActPos) ||
            !std::isfinite(axis.currentCmdVel) ||
            !std::isfinite(axis.logicalCmdVel) ||
            !std::isfinite(axis.targetVelocity) ||
            !std::isfinite(axis.targetEndVel))
        {
            return false;
        }

        return true;
    }

    bool TryCanonicalizeInactivePhysicalAxisCommandState(
        AxisContext& axis) noexcept
    {
        if (!CanCanonicalizeInactivePhysicalAxisCommandState(axis))
        {
            return false;
        }

        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        std::fill(axis.velBuffer.begin(), axis.velBuffer.end(), 0.0);
        axis.bufferSum = 0.0;
        axis.bufferIndex = 0;
        axis.state = MotionState::MotionState_IDLE;
        axis.inPosition = true;
        axis.planningPos = axis.currentCmdPos;
        axis.finalTargetPos = axis.currentCmdPos;
        return true;
    }
}

MotionCore::MotionCore()
{
    // Publish a valid default POD image before the first 250 us Motion pass.
    // In particular, diagnostic axis indices retain their -1 sentinel rather
    // than inheriting an all-zero atomic bank image.
    MotionStopSettlePublicationPayload initialPayload{};
    for (std::size_t profileIndex = 0U;
        profileIndex < MOTION_NC_SETTLE_PROFILE_COUNT;
        ++profileIndex)
    {
        initialPayload.ncSettleSnapshots[profileIndex].profile =
            static_cast<MotionNCSettleProfile>(profileIndex);
        initialPayload.ncSettleSnapshots[profileIndex].blocker =
            MotionNCSettleBlocker::RUNTIME_NOT_OBSERVED;
        initialPayload.ncSettleSnapshots[profileIndex].requiredCycles =
            MOTION_NC_SETTLE_REQUIRED_CYCLES;
    }
    std::array<
        std::uint64_t,
        MOTION_STOP_SETTLE_PUBLICATION_WORD_COUNT> words{};

    std::memcpy(
        words.data(),
        &initialPayload,
        sizeof(initialPayload));

    for (std::size_t bankIndex = 0U;
        bankIndex < m_stopSettlePublicationBanks.size();
        ++bankIndex)
    {
        for (std::size_t wordIndex = 0U;
            wordIndex < words.size();
            ++wordIndex)
        {
            m_stopSettlePublicationBanks[bankIndex].words[wordIndex].store(
                words[wordIndex],
                std::memory_order_relaxed);
        }
    }
}


// =============================================================================
// Stage NC-0.2J.5 - Control producer / PDO validity seams
// =============================================================================
MotionNCSettleRequestSequence
MotionCore::AllocateNCSettleRequestSequence() noexcept
{
    MotionNCSettleRequestSequence sequence =
        m_nextNCSettleRequestSequence.fetch_add(
            1ULL,
            std::memory_order_relaxed);

    if (sequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID)
    {
        sequence = m_nextNCSettleRequestSequence.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    return sequence;
}


bool MotionCore::SubmitNCSettleRequest(
    const MotionNCSettleRequest& request) noexcept
{
    return m_ncSettleRequestRing.ProducerTryPush(request);
}


MotionNCSettleRequestSequence MotionCore::RequestFeedHoldNCSettle(
    MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease) noexcept
{
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !ownerLease.IsValid())
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    MotionNCSettleRequest request{};
    request.requestSequence = AllocateNCSettleRequestSequence();
    request.profile = MotionNCSettleProfile::FEED_HOLD_GROUP;
    request.executionEpoch = executionEpoch;
    request.ownerLease = ownerLease;

    if (!SubmitNCSettleRequest(request))
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    return request.requestSequence;
}


MotionNCSettleRequestSequence
MotionCore::RequestResetNCSettleAndRebase(
    MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease,
    const MotionNCResetExecutionState& executionState,
    bool unsupportedFaultOrEstop) noexcept
{
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !ownerLease.IsValid())
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    MotionNCSettleRequest request{};
    request.requestSequence = AllocateNCSettleRequestSequence();
    request.profile = MotionNCSettleProfile::RESET_ALL;
    request.executionEpoch = executionEpoch;
    request.ownerLease = ownerLease;
    request.resetExecutionState = executionState;
    request.unsupportedFaultOrEstop = unsupportedFaultOrEstop;

    if (!SubmitNCSettleRequest(request))
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    return request.requestSequence;
}


void MotionCore::ObserveNCSettleRuntimeCycle(
    std::uint64_t runtimeCycleTick,
    bool pdoCycleValid) noexcept
{
    const bool hadPreviousObservation = m_ncSettleRuntimeObserved;
    const bool adjacentToPrevious =
        !hadPreviousObservation ||
        (m_ncSettlePreviousRuntimeCycleValid &&
            runtimeCycleTick == m_ncSettlePreviousRuntimeCycleTick + 1ULL);

    m_ncSettleRuntimeObserved = true;
    m_ncSettleRuntimeCycleTick = runtimeCycleTick;
    m_ncSettleRuntimeCycleValid = pdoCycleValid;
    m_ncSettleRuntimeCycleContiguous = adjacentToPrevious;
    m_ncSettleMotionPassCompleted = false;

    m_ncSettlePreviousRuntimeCycleTick = runtimeCycleTick;
    m_ncSettlePreviousRuntimeCycleValid = pdoCycleValid;

    // The invalid-first-cycle Runtime path intentionally skips
    // UpdateAllMotion().  Publish the invalidation here so no old proof can
    // survive until a later valid Motion pass.
    if (!pdoCycleValid)
    {
        PublishStopSettleEvidence();
    }
}


void MotionCore::SetPendingResetExecutionState(
    const MotionNCResetExecutionState& executionState) noexcept
{
    m_pendingSourcePC = executionState.physicalExecutionPC;
    m_pendingSourceWCS = executionState.physicalExecutionWCS;
    m_pendingToolMode = executionState.physicalToolMode;
    m_pendingHCode = executionState.physicalHCode;
    m_pendingToolRadMode = executionState.physicalToolRadiusMode;
    m_pendingDCode = executionState.physicalDCode;
    m_pendingIsAbsoluteMode = executionState.physicalIsAbsoluteMode;
    m_pendingG68Active = executionState.physicalG68Active;
    m_pendingG68Angle = executionState.physicalG68Angle;
    m_pendingG168Active = executionState.physicalG168Active;
    m_pendingWCode = executionState.physicalWCode;
    m_pendingG51Active = executionState.physicalG51Active;
    m_pendingScaleRatio = executionState.physicalScaleRatio;
    m_pendingMirrorMask = executionState.physicalMirrorMask;
    m_pendingG16Active = executionState.physicalG16Active;
    m_pendingG162Active = executionState.physicalG162Active;
    m_pendingPlaneMode = executionState.physicalPlaneMode;
}


std::uint32_t MotionCore::BuildExistingNCAxisMask() const noexcept
{
    if (m_pContexts == nullptr)
    {
        return 0U;
    }

    const std::size_t axisCount = (std::min)(
        m_pContexts->size(),
        static_cast<std::size_t>(MAX_AXES));
    std::uint32_t mask = 0U;

    for (std::size_t axisSlot = 0U; axisSlot < axisCount; ++axisSlot)
    {
        if ((*m_pContexts)[axisSlot].isExist)
        {
            mask |= (1U << static_cast<unsigned>(axisSlot));
        }
    }

    return mask;
}


std::uint32_t MotionCore::BuildCurrentNCGroupAxisMask() const noexcept
{
    if (m_pContexts == nullptr)
    {
        return 0U;
    }

    int groupAxisCount = m_Group.axisCount;
    if (groupAxisCount < 0)
    {
        groupAxisCount = 0;
    }
    if (groupAxisCount > MAX_AXES)
    {
        groupAxisCount = MAX_AXES;
    }

    std::uint32_t mask = 0U;
    for (int groupAxis = 0; groupAxis < groupAxisCount; ++groupAxis)
    {
        const int axisSlot = m_Group.axisIndices[groupAxis];
        if (axisSlot < 0 ||
            axisSlot >= MAX_AXES ||
            static_cast<std::size_t>(axisSlot) >= m_pContexts->size() ||
            !(*m_pContexts)[static_cast<std::size_t>(axisSlot)].isExist)
        {
            continue;
        }

        mask |= (1U << static_cast<unsigned>(axisSlot));
    }

    return mask;
}


bool MotionCore::HasNCResetUnsupportedMotion() const noexcept
{
    if (m_Group.jumpManager.state != JumpState::IDLE ||
        (m_Group.isActive &&
            m_Group.mode != InterpolationMode::LINEAR) ||
        m_Group.pathMode == PathMode::PATH_SERVO ||
        m_Group.pathMode == PathMode::JUMP_TRACKING ||
        m_Group.virtualAxis.isFault ||
        m_Group.virtualAxis.state == MotionState::MotionState_ERROR ||
        m_Group.virtualAxis.state == MotionState::MotionState_ESTOP)
    {
        return true;
    }

    if (m_pContexts == nullptr)
    {
        return true;
    }

    const std::size_t axisCount = (std::min)(
        m_pContexts->size(),
        static_cast<std::size_t>(MAX_AXES));
    const std::uint32_t groupMask = BuildCurrentNCGroupAxisMask();
    for (std::size_t axisSlot = 0U; axisSlot < axisCount; ++axisSlot)
    {
        const AxisContext& axis = (*m_pContexts)[axisSlot];
        if (!axis.isExist)
        {
            continue;
        }

        if (axis.isFault ||
            axis.state == MotionState::MotionState_ERROR ||
            axis.state == MotionState::MotionState_ESTOP)
        {
            return true;
        }

        const bool groupControlledState =
            axis.state == MotionState::MotionState_INTERPOLATING ||
            axis.state == MotionState::MotionState_STOPPING;
        if ((groupControlledState &&
            (groupMask &
                (1U << static_cast<unsigned>(axisSlot))) == 0U) ||
            (!groupControlledState &&
                axis.state != MotionState::MotionState_IDLE))
        {
            return true;
        }
    }

    return false;
}


bool MotionCore::HasNCResetActiveCompensation() const noexcept
{
    if (m_pContexts == nullptr)
    {
        return false;
    }

    const std::size_t axisCount = (std::min)(
        m_pContexts->size(),
        static_cast<std::size_t>(MAX_AXES));
    for (std::size_t axisSlot = 0U; axisSlot < axisCount; ++axisSlot)
    {
        const AxisContext& axis = (*m_pContexts)[axisSlot];
        if (!axis.isExist)
        {
            continue;
        }

        // J.5 rebase is intentionally not compensation-aware.  Any enabled
        // backlash/pitch path is therefore unsupported, even when its
        // currently injected offset happens to be zero.
        if (axis.enableBacklash || axis.enablePitch)
        {
            return true;
        }
    }

    return false;
}


void MotionCore::ResetNCSettleCandidate(
    MotionNCSettleProfile profile,
    MotionNCSettleBlocker blocker,
    bool identityReset,
    bool scopeReset) noexcept
{
    const std::size_t profileIndex = static_cast<std::size_t>(profile);
    if (profileIndex >= MOTION_NC_SETTLE_PROFILE_COUNT)
    {
        return;
    }

    MotionNCSettleTracker& tracker = m_ncSettleTrackers[profileIndex];
    MotionNCSettleCounters& counters =
        m_ncSettleProducerCounters[profileIndex];

    if (tracker.candidate || tracker.dwellCycles != 0U)
    {
        ++counters.candidateResetCount;
    }
    if (tracker.settled)
    {
        ++counters.proofRevokeCount;
    }
    if (identityReset)
    {
        ++counters.identityResetCount;
    }
    if (scopeReset)
    {
        ++counters.scopeResetCount;
    }

    tracker.candidate = false;
    tracker.settled = false;
    tracker.dwellCycles = 0U;
    m_ncSettlePublishedSnapshots[profileIndex].blocker = blocker;
}


void MotionCore::ApplyNCResetScalarRebase() noexcept
{
    const std::uint32_t axisMask =
        m_ncResetRebaseAckProducer.requestedAxisMask;
    const std::size_t axisCount =
        (m_pContexts != nullptr)
        ? (std::min)(m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES))
        : 0U;

    for (std::size_t axisSlot = 0U; axisSlot < axisCount; ++axisSlot)
    {
        if ((axisMask & (1U << static_cast<unsigned>(axisSlot))) == 0U)
        {
            continue;
        }

        AxisContext& axis = (*m_pContexts)[axisSlot];
        const double actualPulse = axis.currentActPos;
        m_ncResetRebaseAckProducer.actualPulse[axisSlot] = actualPulse;

        double actualUnit = 0.0;
        if (std::isfinite(axis.resolution_PPR) &&
            std::isfinite(axis.finalLead) &&
            axis.resolution_PPR != 0.0)
        {
            actualUnit = actualPulse * axis.finalLead / axis.resolution_PPR;
            if (axis.axisType == AxisType::ROTARY &&
                std::isfinite(axis.rotaryModulo) &&
                axis.rotaryModulo > 0.0)
            {
                actualUnit = std::fmod(actualUnit, axis.rotaryModulo);
                if (actualUnit < 0.0)
                {
                    actualUnit += axis.rotaryModulo;
                }
            }
        }
        m_ncResetRebaseAckProducer.actualMcsUnit[axisSlot] = actualUnit;

        axis.currentCmdPos = actualPulse;
        axis.logicalCmdPos = actualPulse;
        axis.planningPos = actualPulse;
        axis.finalTargetPos = actualPulse;
        axis.startCmdPos = actualPulse;
        axis.lastQueuedPulse = actualPulse;
        axis.lastActPos = actualPulse;

        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        axis.programmedVel_PPS = 0.0;
        axis.cruiseVel_PPS = 0.0;
        axis.acc_PPS2 = 0.0;
        axis.dec_PPS2 = 0.0;
        axis.currentActVel = 0.0;
        axis.motionTime = 0.0;
        axis.feedrateOverride = 1.0;

        axis.pid.prevError = 0.0;
        axis.pid.integralAcc = 0.0;
        axis.Pid_IDLE.prevError = 0.0;
        axis.Pid_IDLE.integralAcc = 0.0;
        axis.Pid_G00.prevError = 0.0;
        axis.Pid_G00.integralAcc = 0.0;

        axis.bufferSum = 0.0;
        axis.bufferIndex = 0;
        axis.state = MotionState::MotionState_IDLE;
        axis.inPosition = true;
        axis.resetRequest = false;
    }

    AxisContext& virtualAxis = m_Group.virtualAxis;
    virtualAxis.currentActPos = 0.0;
    virtualAxis.lastActPos = 0.0;
    virtualAxis.currentActVel = 0.0;
    virtualAxis.currentCmdPos = 0.0;
    virtualAxis.logicalCmdPos = 0.0;
    virtualAxis.planningPos = 0.0;
    virtualAxis.finalTargetPos = 0.0;
    virtualAxis.startCmdPos = 0.0;
    virtualAxis.lastQueuedPulse = 0.0;
    virtualAxis.currentCmdVel = 0.0;
    virtualAxis.logicalCmdVel = 0.0;
    virtualAxis.targetVelocity = 0.0;
    virtualAxis.targetEndVel = 0.0;
    virtualAxis.programmedVel_PPS = 0.0;
    virtualAxis.cruiseVel_PPS = 0.0;
    virtualAxis.acc_PPS2 = 0.0;
    virtualAxis.dec_PPS2 = 0.0;
    virtualAxis.motionTime = 0.0;
    virtualAxis.pid.prevError = 0.0;
    virtualAxis.pid.integralAcc = 0.0;
    virtualAxis.Pid_IDLE.prevError = 0.0;
    virtualAxis.Pid_IDLE.integralAcc = 0.0;
    virtualAxis.Pid_G00.prevError = 0.0;
    virtualAxis.Pid_G00.integralAcc = 0.0;
    virtualAxis.bufferSum = 0.0;
    virtualAxis.bufferIndex = 0;
    virtualAxis.state = MotionState::MotionState_IDLE;
    virtualAxis.inPosition = true;
    virtualAxis.feedrateOverride = 1.0;

    const MotionNCResetExecutionState& tags =
        m_activeResetNCSettleRequest.resetExecutionState;
    m_Group.currentExecutionPC = tags.physicalExecutionPC;
    m_Group.currentExecutionWCS = tags.physicalExecutionWCS;
    m_Group.currentExecutionToolMode = tags.physicalToolMode;
    m_Group.currentExecutionHCode = tags.physicalHCode;
    m_Group.currentExecutionToolRadiusMode = tags.physicalToolRadiusMode;
    m_Group.currentExecutionDCode = tags.physicalDCode;
    m_Group.currentExecutionIsAbsoluteMode = tags.physicalIsAbsoluteMode;
    m_Group.currentExecutionG68Active = tags.physicalG68Active;
    m_Group.currentExecutionG68Angle = tags.physicalG68Angle;
    m_Group.currentExecutionG168Active = tags.physicalG168Active;
    m_Group.currentExecutionWCode = tags.physicalWCode;
    m_Group.currentExecutionG51Active = tags.physicalG51Active;
    m_Group.currentExecutionScaleRatio = tags.physicalScaleRatio;
    m_Group.currentExecutionMirrorMask = tags.physicalMirrorMask;
    m_Group.currentExecutionG16Active = tags.physicalG16Active;
    m_Group.currentExecutionG162Active = tags.physicalG162Active;
    m_Group.currentExecutionPlaneMode = tags.physicalPlaneMode;

    m_Group.isActive = false;
    m_Group.axisCount = 0;
    m_Group.mode = InterpolationMode::LINEAR;
    m_Group.feedrateOverride = 1.0;
    m_Group.pathMode = PathMode::EXACT_STOP;
    m_Group.pathServoVel = 0.0;
    m_Group.centerX = 0.0;
    m_Group.centerY = 0.0;
    m_Group.radius = 0.0;
    m_Group.startAngle = 0.0;
    m_Group.totalAngle = 0.0;
    m_Group.totalDist3D = 0.0;
    for (int groupAxis = 0; groupAxis < MAX_AXES; ++groupAxis)
    {
        m_Group.axisIndices[groupAxis] = 0;
        m_Group.startPos[groupAxis] = 0.0;
        m_Group.ratio[groupAxis] = 0.0;
    }
    m_Group.enableTransform = false;
    for (int transformAxis = 0; transformAxis < 3; ++transformAxis)
    {
        m_Group.transformOrigin[transformAxis] = 0.0;
        for (int transformComponent = 0;
            transformComponent < 3;
            ++transformComponent)
        {
            m_Group.transformMatrix[transformAxis][transformComponent] =
                (transformAxis == transformComponent) ? 1.0 : 0.0;
        }
    }
    m_Group.currentCmd = MotionCommand{};
    m_Group.jumpManager.state = JumpState::IDLE;
    m_Group.jumpManager.triggerPos = 0.0;
    m_Group.jumpManager.jumpVel = 0.0;
    m_Group.jumpManager.currentOffset = 0.0;
    m_Group.jumpManager.targetOffset = 0.0;
    m_Group.jumpManager.currentStepIdx = 0;
    m_Group.jumpManager.dwellTimer = 0.0;
    m_Group.jumpManager.dwellTimeTarget = 0.0;
    m_Group.jumpManager.isRecovering = false;
    m_Group.jumpManager.isPauseMode = false;
    m_Group.jumpManager.resumeAlignMode = 0;
    m_Group.jumpManager.firstStageMask = 0;
    m_Group.jumpManager.alignVel = 0.0;
    m_Group.jumpManager.alignOffset = 0.0;
    m_Group.jumpManager.alignDist = 0.0;

    m_ncResetRebaseAckProducer.appliedAxisMask = axisMask;
    m_ncResetScalarRebaseApplied = true;
    m_ncResetBufferClearAxisSlot = 0U;
    m_ncResetBufferClearElement = 0U;
}


bool MotionCore::ClearNCResetBuffersWithBudget() noexcept
{
    const std::size_t physicalAxisCount =
        (m_pContexts != nullptr)
        ? (std::min)(m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES))
        : 0U;
    const std::size_t totalSlots = physicalAxisCount + 1U;
    std::size_t remainingBudget = MOTION_NC_RESET_BUFFER_CLEAR_BUDGET;

    while (remainingBudget != 0U &&
        m_ncResetBufferClearAxisSlot < totalSlots)
    {
        AxisContext* axis = nullptr;
        if (m_ncResetBufferClearAxisSlot < physicalAxisCount)
        {
            const std::uint32_t bit =
                1U << static_cast<unsigned>(m_ncResetBufferClearAxisSlot);
            if ((m_ncResetRebaseAckProducer.requestedAxisMask & bit) != 0U)
            {
                axis = &(*m_pContexts)[m_ncResetBufferClearAxisSlot];
            }
        }
        else
        {
            axis = &m_Group.virtualAxis;
        }

        if (axis == nullptr ||
            m_ncResetBufferClearElement >= axis->velBuffer.size())
        {
            if (axis != nullptr)
            {
                axis->bufferSum = 0.0;
                axis->bufferIndex = 0;
            }
            ++m_ncResetBufferClearAxisSlot;
            m_ncResetBufferClearElement = 0U;
            continue;
        }

        axis->velBuffer[m_ncResetBufferClearElement] = 0.0;
        ++m_ncResetBufferClearElement;
        --remainingBudget;
    }

    return m_ncResetBufferClearAxisSlot >= totalSlots;
}


bool MotionCore::VerifyNCResetRebaseState() const noexcept
{
    const MotionNCResetRebaseAck& ack = m_ncResetRebaseAckProducer;
    const MotionNCResetExecutionState& tags = ack.executionState;
    const AxisContext& virtualAxis = m_Group.virtualAxis;
    const PathJumpManager& jump = m_Group.jumpManager;

    if (!m_ncResetScalarRebaseApplied ||
        !ack.rebaseApplied ||
        ack.requestedAxisMask == 0U ||
        ack.appliedAxisMask != ack.requestedAxisMask ||
        m_pContexts == nullptr ||
        ack.requestedAxisMask != BuildExistingNCAxisMask() ||
        m_Group.isActive ||
        m_Group.axisCount != 0 ||
        m_Group.mode != InterpolationMode::LINEAR ||
        m_Group.pathMode != PathMode::EXACT_STOP ||
        !IsFiniteNearlyEqual(m_Group.feedrateOverride, 1.0) ||
        !IsFiniteNearlyEqual(m_Group.pathServoVel, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.centerX, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.centerY, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.radius, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.startAngle, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.totalAngle, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.totalDist3D, 0.0) ||
        m_Group.enableTransform ||
        m_Group.currentCmd.execution.IsAssigned() ||
        m_Group.currentCmd.ownerLease.IsValid() ||
        m_Group.currentCmd.axisCount != 0 ||
        m_Group.currentCmd.sourceLinePC != 0 ||
        !IsFiniteNearlyEqual(m_Group.currentCmd.targetVel, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.currentCmd.accTime, 0.0) ||
        !IsFiniteNearlyEqual(m_Group.currentCmd.decTime, 0.0) ||
        jump.state != JumpState::IDLE ||
        jump.currentStepIdx != 0 ||
        jump.isRecovering ||
        jump.isPauseMode ||
        jump.resumeAlignMode != 0 ||
        jump.firstStageMask != 0 ||
        !IsFiniteNearlyEqual(jump.triggerPos, 0.0) ||
        !IsFiniteNearlyEqual(jump.currentOffset, 0.0) ||
        !IsFiniteNearlyEqual(jump.targetOffset, 0.0) ||
        !IsFiniteNearlyEqual(jump.dwellTimer, 0.0) ||
        !IsFiniteNearlyEqual(jump.dwellTimeTarget, 0.0) ||
        !IsFiniteNearlyEqual(jump.jumpVel, 0.0) ||
        !IsFiniteNearlyEqual(jump.alignVel, 0.0) ||
        !IsFiniteNearlyEqual(jump.alignOffset, 0.0) ||
        !IsFiniteNearlyEqual(jump.alignDist, 0.0) ||
        m_Group.currentExecutionPC != tags.physicalExecutionPC ||
        m_Group.currentExecutionWCS != tags.physicalExecutionWCS ||
        m_Group.currentExecutionToolMode != tags.physicalToolMode ||
        m_Group.currentExecutionHCode != tags.physicalHCode ||
        m_Group.currentExecutionToolRadiusMode !=
        tags.physicalToolRadiusMode ||
        m_Group.currentExecutionDCode != tags.physicalDCode ||
        m_Group.currentExecutionWCode != tags.physicalWCode ||
        m_Group.currentExecutionPlaneMode != tags.physicalPlaneMode ||
        m_Group.currentExecutionMirrorMask != tags.physicalMirrorMask ||
        m_Group.currentExecutionIsAbsoluteMode !=
        tags.physicalIsAbsoluteMode ||
        m_Group.currentExecutionG68Active != tags.physicalG68Active ||
        m_Group.currentExecutionG168Active != tags.physicalG168Active ||
        m_Group.currentExecutionG51Active != tags.physicalG51Active ||
        m_Group.currentExecutionG16Active != tags.physicalG16Active ||
        m_Group.currentExecutionG162Active != tags.physicalG162Active ||
        !IsFiniteNearlyEqual(
            m_Group.currentExecutionG68Angle,
            tags.physicalG68Angle) ||
        !IsFiniteNearlyEqual(
            m_Group.currentExecutionScaleRatio,
            tags.physicalScaleRatio) ||
        m_ncResetBufferClearAxisSlot <
        (std::min)(m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES)) + 1U ||
        m_ncResetBufferClearElement != 0U ||
        !IsFiniteNearlyEqual(virtualAxis.currentCmdPos, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.logicalCmdPos, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.planningPos, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.finalTargetPos, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.startCmdPos, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.lastQueuedPulse, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.currentCmdVel, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.logicalCmdVel, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.targetVelocity, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.targetEndVel, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.programmedVel_PPS, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.cruiseVel_PPS, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.acc_PPS2, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.dec_PPS2, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.motionTime, 0.0) ||
        !IsFiniteNearlyEqual(virtualAxis.feedrateOverride, 1.0) ||
        !IsFiniteNearlyEqual(virtualAxis.bufferSum, 0.0) ||
        virtualAxis.bufferIndex != 0 ||
        virtualAxis.state != MotionState::MotionState_IDLE)
    {
        return false;
    }

    for (int groupAxis = 0; groupAxis < MAX_AXES; ++groupAxis)
    {
        if (m_Group.axisIndices[groupAxis] != 0 ||
            !IsFiniteNearlyEqual(m_Group.startPos[groupAxis], 0.0) ||
            !IsFiniteNearlyEqual(m_Group.ratio[groupAxis], 0.0) ||
            m_Group.currentCmd.axisIndices[groupAxis] != 0 ||
            !IsFiniteNearlyEqual(
                m_Group.currentCmd.targetPos[groupAxis],
                0.0))
        {
            return false;
        }
    }

    for (int transformAxis = 0; transformAxis < 3; ++transformAxis)
    {
        if (!IsFiniteNearlyEqual(
            m_Group.transformOrigin[transformAxis],
            0.0))
        {
            return false;
        }
        for (int transformComponent = 0;
            transformComponent < 3;
            ++transformComponent)
        {
            const double expected =
                (transformAxis == transformComponent) ? 1.0 : 0.0;
            if (!IsFiniteNearlyEqual(
                m_Group.transformMatrix[transformAxis][transformComponent],
                expected))
            {
                return false;
            }
        }
    }

    const std::size_t axisCount = (std::min)(
        m_pContexts->size(),
        static_cast<std::size_t>(MAX_AXES));
    for (std::size_t axisSlot = 0U; axisSlot < axisCount; ++axisSlot)
    {
        const std::uint32_t bit =
            1U << static_cast<unsigned>(axisSlot);
        if ((ack.requestedAxisMask & bit) == 0U)
        {
            continue;
        }

        const AxisContext& axis = (*m_pContexts)[axisSlot];
        const double reference = ack.actualPulse[axisSlot];
        if (!axis.isExist ||
            !IsFiniteNearlyEqual(axis.currentCmdPos, reference, 0.5) ||
            !IsFiniteNearlyEqual(axis.logicalCmdPos, reference, 0.5) ||
            !IsFiniteNearlyEqual(axis.planningPos, reference, 0.5) ||
            !IsFiniteNearlyEqual(axis.finalTargetPos, reference, 0.5) ||
            !IsFiniteNearlyEqual(axis.startCmdPos, reference, 0.5) ||
            !IsFiniteNearlyEqual(axis.lastQueuedPulse, reference, 0.5) ||
            !IsFiniteNearlyEqual(axis.currentCmdVel, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.logicalCmdVel, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.targetVelocity, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.targetEndVel, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.programmedVel_PPS, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.cruiseVel_PPS, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.acc_PPS2, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.dec_PPS2, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.motionTime, 0.0, 1.0e-9) ||
            !IsFiniteNearlyEqual(axis.feedrateOverride, 1.0) ||
            !IsFiniteNearlyEqual(axis.bufferSum, 0.0, 1.0e-9) ||
            axis.bufferIndex != 0 ||
            axis.state != MotionState::MotionState_IDLE)
        {
            return false;
        }

    }

    return true;
}


MotionNCSettleBlocker
MotionCore::ValidateNCResetCommitSeam() const noexcept
{
    const MotionNCSettleRequest& request =
        m_activeResetNCSettleRequest;
    const MotionNCResetRebaseAck& ack =
        m_ncResetRebaseAckProducer;
    const MotionNCSettleTracker& tracker = m_ncSettleTrackers[
        static_cast<std::size_t>(MotionNCSettleProfile::RESET_ALL)];

    if (m_ncResetRebasePhase !=
        MotionNCResetRebasePhase::CLEARING_BUFFERS ||
        request.profile != MotionNCSettleProfile::RESET_ALL ||
        request.requestSequence ==
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        request.requestSequence != ack.requestSequence ||
        request.requestSequence != tracker.requestSequence ||
        !ack.requestAccepted ||
        !tracker.requestAccepted)
    {
        return MotionNCSettleBlocker::REQUEST_MISSING;
    }

    if (request.executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        request.executionEpoch != GetCurrentExecutionEpoch() ||
        request.executionEpoch != ack.executionEpoch ||
        request.executionEpoch != tracker.executionEpoch)
    {
        return MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH;
    }

    if (!request.ownerLease.IsValid() ||
        request.ownerLease.owner != MotionOwner::SAFETY ||
        ack.owner != MotionOwner::SAFETY ||
        ack.ownerGeneration != request.ownerLease.generation ||
        !tracker.ownerLease.Matches(request.ownerLease) ||
        !IsMotionOwnerLeaseCurrent(request.ownerLease))
    {
        return MotionNCSettleBlocker::OWNER_LEASE_MISMATCH;
    }

    const std::uint32_t currentAxisMask = BuildExistingNCAxisMask();
    if (ack.requestedAxisMask == 0U ||
        currentAxisMask == 0U)
    {
        return MotionNCSettleBlocker::SCOPE_EMPTY;
    }
    if (ack.requestedAxisMask != currentAxisMask ||
        tracker.scopeMask != currentAxisMask)
    {
        return MotionNCSettleBlocker::SCOPE_CHANGED;
    }

    if (HasPendingSafetyOrRecoveryRequests())
    {
        return MotionNCSettleBlocker::SAFETY_OR_RECOVERY_PENDING;
    }

    if (request.unsupportedFaultOrEstop ||
        ack.unsupportedFaultOrEstop)
    {
        return MotionNCSettleBlocker::RESET_UNSUPPORTED;
    }

    if (m_Group.virtualAxis.isFault ||
        m_Group.virtualAxis.state == MotionState::MotionState_ERROR ||
        m_Group.virtualAxis.state == MotionState::MotionState_ESTOP)
    {
        return MotionNCSettleBlocker::AXIS_FAULT_OR_ESTOP;
    }
    if (m_pContexts == nullptr)
    {
        return MotionNCSettleBlocker::SCOPE_EMPTY;
    }

    const std::size_t axisCount = (std::min)(
        m_pContexts->size(),
        static_cast<std::size_t>(MAX_AXES));
    for (std::size_t axisSlot = 0U; axisSlot < axisCount; ++axisSlot)
    {
        const AxisContext& axis = (*m_pContexts)[axisSlot];
        if (!axis.isExist)
        {
            continue;
        }
        if (axis.isFault ||
            axis.state == MotionState::MotionState_ERROR ||
            axis.state == MotionState::MotionState_ESTOP)
        {
            return MotionNCSettleBlocker::AXIS_FAULT_OR_ESTOP;
        }
    }

    if (HasNCResetUnsupportedMotion())
    {
        return MotionNCSettleBlocker::PATH_RUNTIME_UNSUPPORTED;
    }
    if (HasNCResetActiveCompensation())
    {
        return MotionNCSettleBlocker::COMPENSATION_ACTIVE;
    }
    if (!m_Group.cmdQueue.empty() ||
        m_axisCommandChannel.command_size() != 0U)
    {
        return MotionNCSettleBlocker::COMMAND_QUEUE;
    }

    return MotionNCSettleBlocker::NONE;
}


void MotionCore::BlockNCResetCommit(
    MotionNCSettleBlocker blocker) noexcept
{
    m_ncResetRebasePhase = MotionNCResetRebasePhase::BLOCKED;
    m_ncResetRebaseAckProducer.phase =
        MotionNCResetRebasePhase::BLOCKED;
    m_ncResetRebaseAckProducer.failureBlocker = blocker;
    m_ncResetRebaseAckProducer.blocked = true;
    m_ncResetRebaseAckProducer.appliedAxisMask = 0U;
    m_ncResetRebaseAckProducer.rebaseApplied = false;
    m_ncResetRebaseAckProducer.postVerifyPassed = false;
    m_ncResetRebaseAckProducer.acknowledged = false;
    m_ncResetRebaseAckProducer.acked = false;
    m_ncResetScalarRebaseApplied = false;
    ResetNCSettleCandidate(
        MotionNCSettleProfile::RESET_ALL,
        blocker);
    ++m_ncSettleProducerCounters[
        static_cast<std::size_t>(MotionNCSettleProfile::RESET_ALL)].
        requestRejectCount;
}


bool MotionCore::ProcessNCSettleRequestsAndResetRebase() noexcept
{
    const std::size_t resetProfileIndex = static_cast<std::size_t>(
        MotionNCSettleProfile::RESET_ALL);
    MotionNCSettleTracker& resetTracker =
        m_ncSettleTrackers[resetProfileIndex];
    const MotionExecutionEpoch entryObservedExecutionEpoch =
        GetCurrentExecutionEpoch();
    const MotionOwnerLease entryObservedOwnerLease =
        GetMotionOwnerLease();

    // NC-0.2J.5.1: a Reset transaction is active only while its complete
    // authoritative request still owns the current RT Epoch and SAFETY lease.
    // Internal tracker/ACK coherence is checked separately so corruption can
    // only fail closed; it can never make a current transaction look stale.
    const auto ResetAuthoritativeTupleMatches =
        [this](
            MotionExecutionEpoch observedExecutionEpoch,
            const MotionOwnerLease& observedOwnerLease) noexcept -> bool
    {
        const MotionNCSettleRequest& activeRequest =
            m_activeResetNCSettleRequest;

        return
            activeRequest.profile == MotionNCSettleProfile::RESET_ALL &&
            activeRequest.requestSequence !=
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
            activeRequest.executionEpoch !=
            MOTION_EXECUTION_EPOCH_INVALID &&
            activeRequest.executionEpoch ==
            observedExecutionEpoch &&
            activeRequest.ownerLease.IsValid() &&
            activeRequest.ownerLease.owner == MotionOwner::SAFETY &&
            activeRequest.ownerLease.Matches(
                observedOwnerLease);
    };

    const bool postRebaseTransaction =
        m_ncResetRebasePhase ==
        MotionNCResetRebasePhase::WAIT_POSTPROOF ||
        m_ncResetRebasePhase ==
        MotionNCResetRebasePhase::ACKNOWLEDGED;

    if (postRebaseTransaction &&
        !ResetAuthoritativeTupleMatches(
            entryObservedExecutionEpoch,
            entryObservedOwnerLease))
    {
        const MotionNCSettleRequest& activeRequest =
            m_activeResetNCSettleRequest;
        const bool sequenceMatches =
            activeRequest.requestSequence !=
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
            activeRequest.requestSequence == resetTracker.requestSequence &&
            activeRequest.requestSequence ==
            m_ncResetRebaseAckProducer.requestSequence;
        const bool epochMatches =
            activeRequest.executionEpoch !=
            MOTION_EXECUTION_EPOCH_INVALID &&
            activeRequest.executionEpoch == resetTracker.executionEpoch &&
            activeRequest.executionEpoch ==
            m_ncResetRebaseAckProducer.executionEpoch &&
            activeRequest.executionEpoch ==
            entryObservedExecutionEpoch;
        const bool ownerMatches =
            activeRequest.ownerLease.IsValid() &&
            activeRequest.ownerLease.owner == MotionOwner::SAFETY &&
            resetTracker.ownerLease.Matches(activeRequest.ownerLease) &&
            m_ncResetRebaseAckProducer.owner ==
            activeRequest.ownerLease.owner &&
            m_ncResetRebaseAckProducer.ownerGeneration ==
            activeRequest.ownerLease.generation &&
            activeRequest.ownerLease.Matches(
                entryObservedOwnerLease);

        MotionNCSettleBlocker retirementBlocker =
            MotionNCSettleBlocker::REQUEST_MISSING;
        if (sequenceMatches && !epochMatches)
        {
            retirementBlocker =
                MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH;
        }
        else if (sequenceMatches && epochMatches && !ownerMatches)
        {
            retirementBlocker =
                MotionNCSettleBlocker::OWNER_LEASE_MISMATCH;
        }

        resetTracker.candidate = false;
        resetTracker.settled = false;
        resetTracker.dwellCycles = 0U;
        resetTracker.requestAccepted = false;
        m_activeResetNCSettleRequest = MotionNCSettleRequest{};
        m_ncResetRebasePhase = MotionNCResetRebasePhase::IDLE;
        m_ncResetScalarRebaseApplied = false;
        m_ncResetBufferClearAxisSlot = 0U;
        m_ncResetBufferClearElement = 0U;

        // Preserve applied masks/Actual snapshots as history, but revoke all
        // authority.  SUPERSEDED is terminal evidence, not a Reset failure;
        // a still-active release gate will nevertheless fail closed on it.
        m_ncResetRebaseAckProducer.phase =
            MotionNCResetRebasePhase::SUPERSEDED;
        m_ncResetRebaseAckProducer.failureBlocker = retirementBlocker;
        m_ncResetRebaseAckProducer.postVerifyPassed = false;
        m_ncResetRebaseAckProducer.acknowledged = false;
        m_ncResetRebaseAckProducer.acked = false;
        m_ncResetRebaseAckProducer.blocked = false;
        m_ncResetRebaseAckProducer.superseded = true;
    }

    MotionNCSettleRequest request{};
    if (m_ncSettleRequestRing.ConsumerTryPop(request))
    {
        // The producer publishes Epoch/owner before its release-push.  Acquire
        // them only after dequeue so a request arriving across this RT seam is
        // never judged against the older entry snapshot used for retirement.
        const MotionExecutionEpoch requestObservedExecutionEpoch =
            GetCurrentExecutionEpoch();
        const MotionOwnerLease requestObservedOwnerLease =
            GetMotionOwnerLease();
        const bool tupleCurrent =
            request.executionEpoch == requestObservedExecutionEpoch &&
            request.ownerLease.IsValid() &&
            request.ownerLease.Matches(requestObservedOwnerLease);

        if (request.profile == MotionNCSettleProfile::FEED_HOLD_GROUP)
        {
            m_activeFeedHoldNCSettleRequest = request;
            MotionNCSettleTracker& tracker = m_ncSettleTrackers[
                static_cast<std::size_t>(
                    MotionNCSettleProfile::FEED_HOLD_GROUP)];
            ResetNCSettleCandidate(
                MotionNCSettleProfile::FEED_HOLD_GROUP,
                MotionNCSettleBlocker::REQUEST_MISSING,
                true,
                true);
            tracker.requestSequence = request.requestSequence;
            tracker.executionEpoch = request.executionEpoch;
            tracker.ownerLease = request.ownerLease;
            tracker.executionIdentity = m_Group.currentCmd.execution;
            const bool requestIdentityCurrent =
                tracker.executionIdentity.IsAssigned() &&
                tracker.executionIdentity.epoch == request.executionEpoch;
            const std::uint32_t currentScopeMask =
                requestIdentityCurrent
                ? BuildCurrentNCGroupAxisMask()
                : 0U;

            tracker.scopeMask = currentScopeMask;
            if (tracker.scopeMask != 0U)
            {
                m_ncLastGroupScopeMask = tracker.scopeMask;
                m_ncLastGroupScopeExecutionEpoch = request.executionEpoch;
                m_ncLastGroupScopeExecutionIdentity =
                    tracker.executionIdentity;
            }
            else if (m_ncLastGroupScopeMask != 0U &&
                m_ncLastGroupScopeExecutionEpoch == request.executionEpoch &&
                MotionExecutionIdentityExactlyMatches(
                    m_ncLastGroupScopeExecutionIdentity,
                    tracker.executionIdentity))
            {
                tracker.scopeMask = m_ncLastGroupScopeMask;
            }
            tracker.requestAccepted =
                tupleCurrent &&
                requestIdentityCurrent &&
                tracker.scopeMask != 0U;
            if (!tracker.requestAccepted)
            {
                ++m_ncSettleProducerCounters[
                    static_cast<std::size_t>(
                        MotionNCSettleProfile::FEED_HOLD_GROUP)].
                    requestRejectCount;
            }
        }
        else if (request.profile == MotionNCSettleProfile::RESET_ALL)
        {
            const bool previousResetActive =
                (m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::WAIT_PREPROOF ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::CLEARING_BUFFERS ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::WAIT_POSTPROOF ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::ACKNOWLEDGED ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::BLOCKED) &&
                ResetAuthoritativeTupleMatches(
                    requestObservedExecutionEpoch,
                    requestObservedOwnerLease);

            if (previousResetActive)
            {
                // A true overlapping producer request is terminal.  Publish
                // the incoming sequence explicitly so a gate already armed
                // for it cannot wait forever on the older ACK.  Both old and
                // new observers fail closed, then an operator Reset may start
                // one fresh Epoch / request transaction.
                m_ncResetRebaseAckProducer = MotionNCResetRebaseAck{};
                m_ncResetRebaseAckProducer.requestSequence =
                    request.requestSequence;
                m_ncResetRebaseAckProducer.executionEpoch =
                    request.executionEpoch;
                m_ncResetRebaseAckProducer.owner = request.ownerLease.owner;
                m_ncResetRebaseAckProducer.ownerGeneration =
                    request.ownerLease.generation;
                m_ncResetRebaseAckProducer.phase =
                    MotionNCResetRebasePhase::SUPERSEDED;
                m_ncResetRebaseAckProducer.failureBlocker =
                    MotionNCSettleBlocker::RESET_UNSUPPORTED;
                m_ncResetRebaseAckProducer.blocked = true;
                m_ncResetRebaseAckProducer.superseded = true;
                m_ncResetRebasePhase =
                    MotionNCResetRebasePhase::SUPERSEDED;
                ResetNCSettleCandidate(
                    MotionNCSettleProfile::RESET_ALL,
                    MotionNCSettleBlocker::RESET_UNSUPPORTED);
                ++m_ncSettleProducerCounters[resetProfileIndex].
                    requestRejectCount;
            }
            else
            {
                m_activeResetNCSettleRequest = request;
                m_ncResetRebaseAckProducer = MotionNCResetRebaseAck{};
                m_ncResetRebaseAckProducer.requestSequence =
                    request.requestSequence;
                m_ncResetRebaseAckProducer.executionEpoch =
                    request.executionEpoch;
                m_ncResetRebaseAckProducer.owner = request.ownerLease.owner;
                m_ncResetRebaseAckProducer.ownerGeneration =
                    request.ownerLease.generation;
                m_ncResetRebaseAckProducer.requestedAxisMask =
                    BuildExistingNCAxisMask();
                m_ncResetRebaseAckProducer.executionState =
                    request.resetExecutionState;
                m_ncResetRebaseAckProducer.unsupportedFaultOrEstop =
                    request.unsupportedFaultOrEstop;

                const bool pathUnsupported =
                    HasNCResetUnsupportedMotion();
                const bool compensationActive =
                    HasNCResetActiveCompensation();
                const bool accepted =
                    tupleCurrent &&
                    request.ownerLease.owner == MotionOwner::SAFETY &&
                    !request.unsupportedFaultOrEstop &&
                    !pathUnsupported &&
                    !compensationActive &&
                    m_ncResetRebaseAckProducer.requestedAxisMask != 0U;

                MotionNCSettleTracker& tracker = m_ncSettleTrackers[
                    static_cast<std::size_t>(
                        MotionNCSettleProfile::RESET_ALL)];
                ResetNCSettleCandidate(
                    MotionNCSettleProfile::RESET_ALL,
                    MotionNCSettleBlocker::REQUEST_MISSING,
                    true,
                    true);
                tracker.requestSequence = request.requestSequence;
                tracker.executionEpoch = request.executionEpoch;
                tracker.ownerLease = request.ownerLease;
                tracker.executionIdentity = m_Group.currentCmd.execution;
                tracker.scopeMask =
                    m_ncResetRebaseAckProducer.requestedAxisMask;
                tracker.requestAccepted = accepted;
                m_ncResetRebaseAckProducer.requestAccepted = accepted;

                m_ncResetScalarRebaseApplied = false;
                m_ncResetBufferClearAxisSlot = 0U;
                m_ncResetBufferClearElement = 0U;

                if (accepted)
                {
                    m_ncResetRebasePhase =
                        MotionNCResetRebasePhase::WAIT_PREPROOF;
                    m_ncResetRebaseAckProducer.phase =
                        MotionNCResetRebasePhase::WAIT_PREPROOF;
                }
                else
                {
                    MotionNCSettleBlocker blocker =
                        MotionNCSettleBlocker::RESET_UNSUPPORTED;
                    if (compensationActive)
                    {
                        blocker = MotionNCSettleBlocker::COMPENSATION_ACTIVE;
                    }
                    else if (pathUnsupported &&
                        !request.unsupportedFaultOrEstop)
                    {
                        blocker =
                            MotionNCSettleBlocker::PATH_RUNTIME_UNSUPPORTED;
                    }
                    else if (!tupleCurrent)
                    {
                        blocker =
                            MotionNCSettleBlocker::OWNER_LEASE_MISMATCH;
                    }

                    m_ncResetRebasePhase =
                        MotionNCResetRebasePhase::BLOCKED;
                    m_ncResetRebaseAckProducer.phase =
                        MotionNCResetRebasePhase::BLOCKED;
                    m_ncResetRebaseAckProducer.failureBlocker = blocker;
                    m_ncResetRebaseAckProducer.blocked = true;
                    m_ncResetRebaseAckProducer.compensationBlocked =
                        compensationActive;
                    ++m_ncSettleProducerCounters[
                        static_cast<std::size_t>(
                            MotionNCSettleProfile::RESET_ALL)].
                        requestRejectCount;
                }
            }
        }
    }

    if (m_ncResetRebasePhase ==
        MotionNCResetRebasePhase::CLEARING_BUFFERS)
    {
        if (!m_ncResetScalarRebaseApplied)
        {
            // Commit-time validation is intentionally repeated after this
            // pass has consumed all pending Safety/Emergency work.  A Reset
            // pre-proof from the preceding PDO cycle is never authority to
            // overwrite a same-pass ESTOP/fault or a superseded tuple.
            const MotionNCSettleBlocker commitBlocker =
                ValidateNCResetCommitSeam();
            if (commitBlocker != MotionNCSettleBlocker::NONE)
            {
                BlockNCResetCommit(commitBlocker);
                return true;
            }

            ApplyNCResetScalarRebase();
        }

        if (ClearNCResetBuffersWithBudget())
        {
            m_ncResetRebaseAckProducer.rebaseApplied = true;
            m_ncResetRebasePhase =
                MotionNCResetRebasePhase::WAIT_POSTPROOF;
            m_ncResetRebaseAckProducer.phase =
                MotionNCResetRebasePhase::WAIT_POSTPROOF;
            ResetNCSettleCandidate(
                MotionNCSettleProfile::RESET_ALL,
                MotionNCSettleBlocker::REBASE_IN_PROGRESS);
        }

        // Freeze the interpolator while command references and all smoothing
        // buffers are being rebased.  UpdateAllMotion still runs and keeps
        // PDO/feedback service deterministic.
        return true;
    }

    return false;
}


// ============================================================================
// Stage NC-0.1E - Motion Owner + Generation Lease Arbitration
// ============================================================================

std::uint64_t MotionCore::PackMotionOwnerState(
    MotionOwner owner,
    MotionOwnerGeneration generation) noexcept
{
    return
        (static_cast<std::uint64_t>(generation) << 32U) |
        static_cast<std::uint64_t>(
            static_cast<std::uint8_t>(owner));
}


MotionOwnerLease MotionCore::UnpackMotionOwnerState(
    std::uint64_t packed) noexcept
{
    MotionOwnerLease lease{};

    lease.owner =
        static_cast<MotionOwner>(
            static_cast<std::uint8_t>(
                packed & 0xFFULL));

    lease.generation =
        static_cast<MotionOwnerGeneration>(
            packed >> 32U);

    return lease;
}


MotionOwnerGeneration MotionCore::NextMotionOwnerGeneration(
    MotionOwnerGeneration current) noexcept
{
    if (current ==
        (std::numeric_limits<MotionOwnerGeneration>::max)())
    {
        return 1U;
    }

    const MotionOwnerGeneration next =
        static_cast<MotionOwnerGeneration>(
            current + 1U);

    return
        next == MOTION_OWNER_GENERATION_INVALID
        ? 1U
        : next;
}


MotionOwnerLease MotionCore::GetMotionOwnerLease() const noexcept
{
    return
        UnpackMotionOwnerState(
            m_motionOwnerState.load(
                std::memory_order_acquire));
}


bool MotionCore::IsMotionOwnerLeaseCurrent(
    const MotionOwnerLease& lease) const noexcept
{
    if (!lease.IsValid())
    {
        return false;
    }

    return
        m_motionOwnerState.load(
            std::memory_order_acquire) ==
        PackMotionOwnerState(
            lease.owner,
            lease.generation);
}


bool MotionCore::TryAcquireMotionOwner(
    MotionOwner requestedOwner,
    MotionOwnerLease& outLease) noexcept
{
    outLease = MotionOwnerLease{};

    const std::uint8_t requestedValue =
        static_cast<std::uint8_t>(requestedOwner);

    if (requestedOwner == MotionOwner::NONE ||
        requestedValue >
        static_cast<std::uint8_t>(MotionOwner::SAFETY))
    {
        return false;
    }

    std::uint64_t currentPacked =
        m_motionOwnerState.load(
            std::memory_order_acquire);

    for (;;)
    {
        const MotionOwnerLease currentLease =
            UnpackMotionOwnerState(currentPacked);

        // 同一個邏輯 Owner 重複 Acquire 視為冪等操作，不產生新世代。
        if (currentLease.owner == requestedOwner &&
            currentLease.generation != MOTION_OWNER_GENERATION_INVALID)
        {
            outLease = currentLease;
            return true;
        }

        if (currentLease.owner != MotionOwner::NONE)
        {
            return false;
        }

        MotionOwnerLease requestedLease{};
        requestedLease.owner = requestedOwner;
        requestedLease.generation =
            NextMotionOwnerGeneration(currentLease.generation);

        const std::uint64_t requestedPacked =
            PackMotionOwnerState(
                requestedLease.owner,
                requestedLease.generation);

        if (m_motionOwnerState.compare_exchange_weak(
            currentPacked,
            requestedPacked,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            outLease = requestedLease;
            return true;
        }
    }
}


bool MotionCore::TryTransferMotionOwner(
    const MotionOwnerLease& currentLease,
    MotionOwner requestedOwner,
    MotionOwnerLease& outLease) noexcept
{
    outLease = MotionOwnerLease{};

    const std::uint8_t requestedValue =
        static_cast<std::uint8_t>(requestedOwner);

    if (!currentLease.IsValid() ||
        requestedOwner == MotionOwner::NONE ||
        requestedValue >
        static_cast<std::uint8_t>(MotionOwner::SAFETY))
    {
        return false;
    }

    if (requestedOwner == currentLease.owner)
    {
        if (!IsMotionOwnerLeaseCurrent(currentLease))
        {
            return false;
        }

        outLease = currentLease;
        return true;
    }

    std::uint64_t expectedPacked =
        PackMotionOwnerState(
            currentLease.owner,
            currentLease.generation);

    MotionOwnerLease requestedLease{};
    requestedLease.owner = requestedOwner;
    requestedLease.generation =
        NextMotionOwnerGeneration(currentLease.generation);

    const std::uint64_t requestedPacked =
        PackMotionOwnerState(
            requestedLease.owner,
            requestedLease.generation);

    if (!m_motionOwnerState.compare_exchange_strong(
        expectedPacked,
        requestedPacked,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    outLease = requestedLease;
    return true;
}


bool MotionCore::ReleaseMotionOwner(
    const MotionOwnerLease& lease) noexcept
{
    if (!lease.IsValid())
    {
        return false;
    }

    std::uint64_t expectedPacked =
        PackMotionOwnerState(
            lease.owner,
            lease.generation);

    const std::uint64_t releasedPacked =
        PackMotionOwnerState(
            MotionOwner::NONE,
            NextMotionOwnerGeneration(lease.generation));

    return
        m_motionOwnerState.compare_exchange_strong(
            expectedPacked,
            releasedPacked,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
}


MotionOwnerLease MotionCore::TakeSafetyMotionOwner() noexcept
{
    std::uint64_t currentPacked =
        m_motionOwnerState.load(
            std::memory_order_acquire);

    for (;;)
    {
        const MotionOwnerLease currentLease =
            UnpackMotionOwnerState(currentPacked);

        // Alarm 狀態可能每個 Scan 都重複呼叫；Safety 已持有時不遞增。
        if (currentLease.owner == MotionOwner::SAFETY &&
            currentLease.generation != MOTION_OWNER_GENERATION_INVALID)
        {
            return currentLease;
        }

        MotionOwnerLease safetyLease{};
        safetyLease.owner = MotionOwner::SAFETY;
        safetyLease.generation =
            NextMotionOwnerGeneration(currentLease.generation);

        const std::uint64_t safetyPacked =
            PackMotionOwnerState(
                safetyLease.owner,
                safetyLease.generation);

        if (m_motionOwnerState.compare_exchange_weak(
            currentPacked,
            safetyPacked,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return safetyLease;
        }
    }
}


// ============================================================================
// Stage NC-0.1F - Axis Command Mailbox / RT-only state mutation
// ============================================================================

MotionAxisCommandSequence MotionCore::AllocateAxisCommandSequence() noexcept
{
    MotionAxisCommandSequence sequence =
        m_nextAxisCommandSequence.fetch_add(1ULL, std::memory_order_relaxed);

    if (sequence == MOTION_AXIS_COMMAND_SEQUENCE_INVALID)
    {
        sequence =
            m_nextAxisCommandSequence.fetch_add(1ULL, std::memory_order_relaxed);
    }

    return sequence;
}

bool MotionCore::SubmitAxisCommand(
    MotionAxisCommand command,
    MotionAxisCommandSequence* outSequence) noexcept
{
    if (outSequence != nullptr)
    {
        *outSequence = MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
    }

    const MotionOwner expectedOwner =
        ResolveMotionOwnerForSource(command.source);

    if (command.type == MotionAxisCommandType::NONE ||
        command.axisIndex < 0 ||
        expectedOwner == MotionOwner::NONE ||
        command.ownerLease.owner != expectedOwner ||
        !IsMotionOwnerLeaseCurrent(command.ownerLease))
    {
        return false;
    }

    command.sequence = AllocateAxisCommandSequence();

    if (!m_axisCommandChannel.ControlTrySubmit(command))
    {
        m_axisCommandQueueFullCount.fetch_add(1ULL, std::memory_order_relaxed);
        return false;
    }

    if (outSequence != nullptr)
    {
        *outSequence = command.sequence;
    }

    return true;
}

bool MotionCore::SubmitAxisMoveToPosition(
    int axisIndex,
    double targetPosition,
    double targetVelocity,
    double accelerationTime,
    double decelerationTime,
    bool useShortestPath,
    MotionCommandSource source,
    const MotionOwnerLease& ownerLease,
    MotionAxisCommandSequence* outSequence) noexcept
{
    MotionAxisCommand command{};
    command.type = MotionAxisCommandType::MOVE_TO_POSITION;
    command.axisIndex = axisIndex;
    command.source = source;
    command.ownerLease = ownerLease;
    command.value0 = targetPosition;
    command.value1 = targetVelocity;
    command.value2 = accelerationTime;
    command.value3 = decelerationTime;
    command.flags = useShortestPath
        ? MOTION_AXIS_COMMAND_FLAG_USE_SHORTEST_PATH
        : MOTION_AXIS_COMMAND_FLAG_NONE;
    return SubmitAxisCommand(command, outSequence);
}

bool MotionCore::SubmitAxisVelocityMove(
    int axisIndex,
    double targetVelocity,
    double accelerationTime,
    MotionCommandSource source,
    const MotionOwnerLease& ownerLease,
    MotionAxisCommandSequence* outSequence) noexcept
{
    MotionAxisCommand command{};
    command.type = MotionAxisCommandType::VELOCITY_MOVE;
    command.axisIndex = axisIndex;
    command.source = source;
    command.ownerLease = ownerLease;
    command.value0 = targetVelocity;
    command.value1 = accelerationTime;
    return SubmitAxisCommand(command, outSequence);
}

bool MotionCore::SubmitAxisMPGMove(
    int axisIndex,
    double targetPosition,
    double maximumVelocity,
    double accelerationTime,
    double decelerationTime,
    MotionCommandSource source,
    const MotionOwnerLease& ownerLease,
    MotionAxisCommandSequence* outSequence) noexcept
{
    MotionAxisCommand command{};
    command.type = MotionAxisCommandType::MPG_MOVE;
    command.axisIndex = axisIndex;
    command.source = source;
    command.ownerLease = ownerLease;
    command.value0 = targetPosition;
    command.value1 = maximumVelocity;
    command.value2 = accelerationTime;
    command.value3 = decelerationTime;
    return SubmitAxisCommand(command, outSequence);
}

bool MotionCore::SubmitAxisStopMove(
    int axisIndex,
    double decelerationTime,
    MotionCommandSource source,
    const MotionOwnerLease& ownerLease,
    MotionAxisCommandSequence* outSequence) noexcept
{
    MotionAxisCommand command{};
    command.type = MotionAxisCommandType::STOP_MOVE;
    command.axisIndex = axisIndex;
    command.source = source;
    command.ownerLease = ownerLease;
    command.value0 = decelerationTime;
    return SubmitAxisCommand(command, outSequence);
}

bool MotionCore::SubmitApplyMachineHome(
    int axisIndex,
    double capturedReferencePulse,
    double homeOffsetUnit,
    const MotionOwnerLease& ownerLease,
    MotionAxisCommandSequence& outSequence) noexcept
{
    MotionAxisCommand command{};
    command.type = MotionAxisCommandType::APPLY_MACHINE_HOME;
    command.axisIndex = axisIndex;
    command.source = MotionCommandSource::HOME;
    command.ownerLease = ownerLease;
    command.value0 = capturedReferencePulse;
    command.value1 = homeOffsetUnit;
    return SubmitAxisCommand(command, &outSequence);
}

bool MotionCore::SubmitDriveTouchProbeFunction(
    int axisIndex,
    std::uint16_t value,
    const MotionOwnerLease& ownerLease,
    MotionAxisCommandSequence* outSequence) noexcept
{
    MotionAxisCommand command{};
    command.type = MotionAxisCommandType::SET_TOUCH_PROBE_FUNCTION;
    command.axisIndex = axisIndex;
    command.source = ResolveMotionCommandSourceForOwner(ownerLease.owner);
    command.ownerLease = ownerLease;
    command.wordValue = value;
    return SubmitAxisCommand(command, outSequence);
}

void MotionCore::ProcessAxisCommandResults() noexcept
{
    for (std::size_t i = 0U;
        i < MOTION_AXIS_RESULT_DRAIN_LIMIT_PER_CONTROL_PASS;
        ++i)
    {
        MotionAxisCommandResult result{};
        if (!m_axisCommandChannel.ControlTryConsumeResult(result))
        {
            break;
        }

        if (result.sequence == MOTION_AXIS_COMMAND_SEQUENCE_INVALID)
        {
            continue;
        }

        const std::size_t index = static_cast<std::size_t>(
            result.sequence % static_cast<MotionAxisCommandSequence>(
                MOTION_AXIS_RESULT_CAPACITY));
        m_axisCommandResultLedger[index] = result;
    }
}

bool MotionCore::TryGetAxisCommandResult(
    MotionAxisCommandSequence sequence,
    MotionAxisCommandResult& outResult) const noexcept
{
    outResult = MotionAxisCommandResult{};
    if (sequence == MOTION_AXIS_COMMAND_SEQUENCE_INVALID)
    {
        return false;
    }

    const std::size_t index = static_cast<std::size_t>(
        sequence % static_cast<MotionAxisCommandSequence>(
            MOTION_AXIS_RESULT_CAPACITY));
    const MotionAxisCommandResult& candidate =
        m_axisCommandResultLedger[index];

    if (candidate.sequence != sequence)
    {
        return false;
    }

    outResult = candidate;
    return true;
}

void MotionCore::RequestEmergencyStopAllAxes() noexcept
{
    m_emergencyStopRequestAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    TakeSafetyMotionOwner();

    const bool wasPending =
        m_emergencyStopAllPending.exchange(
            true,
            std::memory_order_acq_rel);

    if (wasPending)
    {
        m_emergencyStopRequestCoalescedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_emergencyStopRequestPublishedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
}

bool MotionCore::TryPublishGroupMappingIntegrityAlarmRequest(
    MotionExecutionEpoch executionEpoch) noexcept
{
    // AlarmManager remains NC single-writer.  Motion publishes only this
    // coherent request; the 10 ms NC task materializes Alarm 3021 before it
    // drains any causal/retirement terminal feedback.
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        AlarmManager::GetInstance().HasAlarm())
    {
        return false;
    }

    std::uint64_t observed =
        m_p1MappingIntegrityAlarmRequestPublication.load(
            std::memory_order_acquire);
    if ((observed & P1_MAPPING_ALARM_PENDING) != 0ULL)
    {
        return false;
    }

    const std::uint64_t previousSequence =
        (observed & P1_MAPPING_ALARM_SEQUENCE_MASK) >>
        P1_MAPPING_ALARM_SEQUENCE_SHIFT;
    const std::uint64_t nextSequence =
        previousSequence >= P1_MAPPING_ALARM_SEQUENCE_MAX
        ? 1ULL
        : previousSequence + 1ULL;
    const std::uint64_t desired =
        P1_MAPPING_ALARM_PENDING |
        (nextSequence << P1_MAPPING_ALARM_SEQUENCE_SHIFT) |
        static_cast<std::uint64_t>(executionEpoch);

    return m_p1MappingIntegrityAlarmRequestPublication.compare_exchange_strong(
        observed,
        desired,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

void MotionCore::TriggerGroupMappingIntegrityEmergencyStop(
    int axisIndex,
    bool forceExecutionInvalidation) noexcept
{
    // The 250 us containment may run before the 10 ms NC observer. Publish a
    // formal Alarm first so J.6.3.2 can correlate the pre-latched E-stop and
    // SAFETY ownership is released only through the normal Reset lifecycle.
    TryPublishGroupMappingIntegrityAlarmRequest(
        GetCurrentExecutionEpoch());

    // This is already the RT consumer. Publish one immediate request/apply
    // accounting edge without leaving the deferred pending bit set; otherwise
    // a deep stale queue could make the next pass apply a second invalidation.
    m_emergencyStopRequestAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    m_emergencyStopRequestPublishedCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    EmergencyStopAllAxesImpl(forceExecutionInvalidation);
}

void MotionCore::RequestAxisFaultReset(int axisIndex) noexcept
{
    if (axisIndex < 0 || axisIndex >= 32)
    {
        return;
    }

    const std::uint32_t mask =
        static_cast<std::uint32_t>(1U << static_cast<unsigned>(axisIndex));
    m_axisFaultResetPendingMask.fetch_or(mask, std::memory_order_release);
}

void MotionCore::RequestResetAllFaults() noexcept
{
    TakeSafetyMotionOwner();
    m_resetAllFaultsPending.store(true, std::memory_order_release);
}

void MotionCore::RequestStopGroup() noexcept
{
    TakeSafetyMotionOwner();
    m_stopGroupPending.store(true, std::memory_order_release);
}

void MotionCore::RequestResetSafetyBatch(
    MotionExecutionEpoch publishedEpoch,
    bool requestResetAllFaults) noexcept
{
    TakeSafetyMotionOwner();

    std::uint64_t current =
        m_resetSafetyBatchPending.load(std::memory_order_relaxed);

    for (;;)
    {
        // Repeated Reset requests before the RT consumer runs use the latest
        // Epoch, while retaining a fault-reset request already latched by an
        // earlier request in the same pending batch.
        const bool mergedResetAllFaults =
            requestResetAllFaults ||
            ((current & RESET_SAFETY_BATCH_PRESENT) != 0ULL &&
                (current & RESET_SAFETY_BATCH_RESET_FAULTS) != 0ULL);

        const std::uint64_t desired =
            PackResetSafetyBatch(
                publishedEpoch,
                mergedResetAllFaults);

        if (m_resetSafetyBatchPending.compare_exchange_weak(
            current,
            desired,
            std::memory_order_release,
            std::memory_order_relaxed))
        {
            break;
        }
    }
}

bool MotionCore::HasPendingSafetyOrRecoveryRequests() const noexcept
{
    return
        HasPendingExecutionEpochChange() ||
        m_safetyRecoveryRequestInProgress.load(std::memory_order_acquire) ||
        m_emergencyStopAllPending.load(std::memory_order_acquire) ||
        m_resetAllFaultsPending.load(std::memory_order_acquire) ||
        m_stopGroupPending.load(std::memory_order_acquire) ||
        m_resetSafetyBatchPending.load(std::memory_order_acquire) != 0ULL ||
        m_axisFaultResetPendingMask.load(std::memory_order_acquire) != 0U;
}

void MotionCore::PublishAxisCommandResult(
    const MotionAxisCommand& command,
    MotionAxisCommandResultType resultType,
    MotionRejectReason rejectReason) noexcept
{
    MotionAxisCommandResult result{};
    result.sequence = command.sequence;
    result.commandType = command.type;
    result.resultType = resultType;
    result.axisIndex = command.axisIndex;
    result.rejectReason = rejectReason;
    result.owner = command.ownerLease.owner;
    result.ownerGeneration = command.ownerLease.generation;

    if (!m_axisCommandChannel.RuntimeTryPublishResult(result))
    {
        m_axisCommandResultOverflowCount.fetch_add(1ULL, std::memory_order_relaxed);
    }
}

void MotionCore::ApplyPendingSafetyAndRecoveryRequests() noexcept
{
    const bool hasPendingRequest =
        m_emergencyStopAllPending.load(std::memory_order_acquire) ||
        m_resetAllFaultsPending.load(std::memory_order_acquire) ||
        m_stopGroupPending.load(std::memory_order_acquire) ||
        m_resetSafetyBatchPending.load(std::memory_order_acquire) != 0ULL ||
        m_axisFaultResetPendingMask.load(std::memory_order_acquire) != 0U;

    if (!hasPendingRequest)
    {
        return;
    }

    // Keep the Safety lease held until the 250 us owner has actually
    // consumed and applied every request. A pending bit may be cleared by
    // exchange before the underlying AxisContext mutation has completed.
    m_safetyRecoveryRequestInProgress.store(true, std::memory_order_release);

    if (m_emergencyStopAllPending.exchange(false, std::memory_order_acq_rel))
    {
        m_resetAllFaultsPending.store(false, std::memory_order_release);
        m_stopGroupPending.store(false, std::memory_order_release);
        m_resetSafetyBatchPending.store(0ULL, std::memory_order_release);
        m_axisFaultResetPendingMask.store(0U, std::memory_order_release);
        EmergencyStopAllAxes();
        m_safetyRecoveryRequestInProgress.store(false, std::memory_order_release);
        return;
    }

    const std::uint64_t resetSafetyBatch =
        m_resetSafetyBatchPending.exchange(
            0ULL,
            std::memory_order_acq_rel);

    if ((resetSafetyBatch & RESET_SAFETY_BATCH_PRESENT) != 0ULL)
    {
        const MotionExecutionEpoch coveredEpoch =
            UnpackResetSafetyBatchEpoch(resetSafetyBatch);

        const bool epochCoverageValid =
            coveredEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            coveredEpoch == GetCurrentExecutionEpoch();

        if (!epochCoverageValid)
        {
            // Another lifecycle event genuinely intervened. Fail closed, but
            // publish at most one replacement Epoch for the entire safety
            // batch rather than one per leaf operation.
            BeginNewExecutionEpoch(
                MotionCommandSource::SAFETY);
        }

        // UpdateInterpolation normally applies the Epoch before entering this
        // function. Repeat the consumer seam to close the narrow race where
        // NC publishes the covered Epoch between the first apply and this
        // request exchange.
        ApplyPendingExecutionEpochChange();

        if ((resetSafetyBatch &
            RESET_SAFETY_BATCH_RESET_FAULTS) != 0ULL)
        {
            ResetAllFaultsImpl(false);
        }

        StopGroupImpl(false);
    }

    if (m_resetAllFaultsPending.exchange(false, std::memory_order_acq_rel))
    {
        ResetAllFaults();
    }

    const std::uint32_t resetMask =
        m_axisFaultResetPendingMask.exchange(0U, std::memory_order_acq_rel);

    if (resetMask != 0U && m_pContexts != nullptr)
    {
        const std::size_t axisCount = (std::min)(
            m_pContexts->size(), static_cast<std::size_t>(32U));
        for (std::size_t i = 0U; i < axisCount; ++i)
        {
            if ((resetMask & (1U << static_cast<unsigned>(i))) != 0U)
            {
                ResetFault((*m_pContexts)[i]);
            }
        }
    }

    if (m_stopGroupPending.exchange(false, std::memory_order_acq_rel))
    {
        StopGroup();
    }

    m_safetyRecoveryRequestInProgress.store(false, std::memory_order_release);
}

void MotionCore::DrainAxisCommandMailbox() noexcept
{
    for (std::size_t i = 0U;
        i < MOTION_AXIS_COMMAND_DRAIN_LIMIT_PER_RUNTIME_PASS;
        ++i)
    {
        MotionAxisCommand command{};
        if (!m_axisCommandChannel.RuntimeTryConsume(command))
        {
            break;
        }

        const MotionOwner expectedOwner =
            ResolveMotionOwnerForSource(command.source);

        if (expectedOwner == MotionOwner::NONE ||
            command.ownerLease.owner != expectedOwner ||
            !IsMotionOwnerLeaseCurrent(command.ownerLease))
        {
            PublishAxisCommandResult(
                command,
                MotionAxisCommandResultType::REJECTED,
                MotionRejectReason::OWNER_CONFLICT);
            continue;
        }

        if (m_pContexts == nullptr ||
            command.axisIndex < 0 ||
            command.axisIndex >= static_cast<int>(m_pContexts->size()))
        {
            PublishAxisCommandResult(
                command,
                MotionAxisCommandResultType::REJECTED,
                MotionRejectReason::INVALID_AXIS);
            continue;
        }

        AxisContext& axis = (*m_pContexts)[
            static_cast<std::size_t>(command.axisIndex)];

        if (!axis.isExist)
        {
            PublishAxisCommandResult(
                command,
                MotionAxisCommandResultType::REJECTED,
                MotionRejectReason::INVALID_AXIS);
            continue;
        }

        bool applied = true;
        MotionRejectReason rejectReason = MotionRejectReason::NONE;

        switch (command.type)
        {
        case MotionAxisCommandType::MOVE_TO_POSITION:
        {
            const bool originalShortestPath = axis.useShortestPath;
            axis.useShortestPath =
                (command.flags & MOTION_AXIS_COMMAND_FLAG_USE_SHORTEST_PATH) != 0U;
            MoveToPosition(
                axis, command.value0, command.value1, command.value2, command.value3);
            axis.useShortestPath = originalShortestPath;
            break;
        }

        case MotionAxisCommandType::VELOCITY_MOVE:
            VelocityMove(axis, command.value0, command.value1);
            break;

        case MotionAxisCommandType::MPG_MOVE:
            MPGMove(
                axis, command.value0, command.value1, command.value2, command.value3);
            break;

        case MotionAxisCommandType::STOP_MOVE:
            StopMove(axis, command.value0);
            break;

        case MotionAxisCommandType::APPLY_MACHINE_HOME:
            applied = ApplyMachineHome(axis, command.value0, command.value1);
            if (!applied) rejectReason = MotionRejectReason::NOT_READY;
            break;

        case MotionAxisCommandType::SET_TOUCH_PROBE_FUNCTION:
            applied = SetDriveTouchProbeFunction(command.axisIndex, command.wordValue);
            if (!applied) rejectReason = MotionRejectReason::NOT_READY;
            break;

        case MotionAxisCommandType::NONE:
        default:
            applied = false;
            rejectReason = MotionRejectReason::INVALID_GEOMETRY;
            break;
        }

        PublishAxisCommandResult(
            command,
            applied
            ? MotionAxisCommandResultType::APPLIED
            : MotionAxisCommandResultType::REJECTED,
            rejectReason);
    }
}

// ============================================================================
// Stage NC-0.1B - Active Execution Epoch / Segment Identity
// ============================================================================

MotionExecutionEpoch MotionCore::BeginNewExecutionEpoch(
    MotionCommandSource source) noexcept
{
    return PublishNewExecutionEpoch(
        source,
        false);
}


MotionExecutionEpoch MotionCore::GetCurrentExecutionEpoch() const noexcept
{
    return UnpackExecutionEpochPublication(
        m_executionEpochPublication.load(
            std::memory_order_acquire));
}


MotionExecutionEpoch MotionCore::PublishNewExecutionEpoch(
    MotionCommandSource source,
    bool abortActiveCommand) noexcept
{
    std::uint64_t currentPublication =
        m_executionEpochPublication.load(
            std::memory_order_relaxed);

    MotionExecutionEpoch nextEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    bool publisherWaitRecorded = false;

    for (;;)
    {
        // The RT owner holds this bit only across a bounded terminal / handoff
        // commit.  A publisher that arrives second waits until the complete
        // mutation is visible, then publishes the next Epoch.  It must never
        // build PackExecutionEpochPublication() from a reserved word because
        // that would silently clear the RT reservation.
        if ((currentPublication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
        {
            if (!publisherWaitRecorded)
            {
                m_lifecycleCommitReservationPublisherWaitCount.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
                publisherWaitRecorded = true;
            }
            currentPublication = m_executionEpochPublication.load(
                std::memory_order_acquire);
            continue;
        }

        // Epoch 0 永遠保留為 INVALID。
        const MotionExecutionEpoch currentEpoch =
            UnpackExecutionEpochPublication(
                currentPublication);

        nextEpoch =
            (currentEpoch ==
                (std::numeric_limits<MotionExecutionEpoch>::max)())
            ? 1U
            : static_cast<MotionExecutionEpoch>(
                currentEpoch + 1U);

        const std::uint64_t desiredPublication =
            PackExecutionEpochPublication(
                nextEpoch,
                source,
                abortActiveCommand,
                true);

        // Epoch allocation, its exact Abort Policy and the RT Pending bit are
        // one CAS publication.  Therefore a later Reset Epoch cannot inherit
        // an earlier G00 ABORTING boolean, even with multiple producers.
        if (m_executionEpochPublication.compare_exchange_weak(
            currentPublication,
            desiredPublication,
            std::memory_order_acq_rel,
            std::memory_order_relaxed))
        {
            break;
        }
    }

    return
        nextEpoch;
}


bool MotionCore::TryAcquireLifecycleCommitReservation(
    const MotionExecutionIdentity& execution,
    std::uint64_t& reservationToken) noexcept
{
    reservationToken = 0ULL;
    m_lifecycleCommitReservationAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    std::uint64_t observed = m_executionEpochPublication.load(
        std::memory_order_acquire);
    if (!execution.IsAssigned() ||
        (observed & EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL ||
        (observed &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL ||
        execution.epoch != UnpackExecutionEpochPublication(observed) ||
        execution.source !=
        UnpackExecutionEpochPublicationSource(observed))
    {
        m_lifecycleCommitReservationBlockedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        return false;
    }

    const std::uint64_t reserved =
        observed | EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED;
    if (!m_executionEpochPublication.compare_exchange_strong(
        observed,
        reserved,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_lifecycleCommitReservationCASLostCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        return false;
    }

    reservationToken = reserved;
    m_lifecycleCommitReservationAcquiredCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    return true;
}


void MotionCore::ReleaseLifecycleCommitReservation(
    std::uint64_t reservationToken) noexcept
{
    if ((reservationToken &
        EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) == 0ULL)
    {
        return;
    }

    std::uint64_t expected = reservationToken;
    const std::uint64_t released =
        reservationToken &
        ~EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED;
    if (m_executionEpochPublication.compare_exchange_strong(
        expected,
        released,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_lifecycleCommitReservationReleasedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        return;
    }

    // Defensive fail-open: a stuck reservation would indefinitely block every
    // RESET / Alarm publisher.  This path is an invariant failure and is made
    // visible in diagnostics; clearing only our private bit preserves the
    // latest lifecycle tuple if another writer violated the protocol.
    m_lifecycleCommitReservationReleaseFailureCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    m_executionEpochPublication.fetch_and(
        ~EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED,
        std::memory_order_release);
}


MotionCore::LifecycleCommitReservationGuard::
LifecycleCommitReservationGuard(
    MotionCore& owner,
    const MotionExecutionIdentity& execution) noexcept
    : m_owner(&owner)
{
    if (!m_owner->TryAcquireLifecycleCommitReservation(
        execution,
        m_reservationToken))
    {
        m_owner = nullptr;
    }
}


MotionCore::LifecycleCommitReservationGuard::
~LifecycleCommitReservationGuard() noexcept
{
    Release();
}


bool MotionCore::LifecycleCommitReservationGuard::
IsAcquired() const noexcept
{
    return m_owner != nullptr;
}


void MotionCore::LifecycleCommitReservationGuard::Release() noexcept
{
    if (m_owner == nullptr)
    {
        return;
    }

    m_owner->ReleaseLifecycleCommitReservation(m_reservationToken);
    m_owner = nullptr;
    m_reservationToken = 0ULL;
}


void MotionCore::BeginProgramBlockMotionCapture() noexcept
{
    m_programBlockMotionCapture = MotionProgramBlockCapture{};
    m_programBlockMotionCaptureActive = true;
}


MotionProgramBlockCapture MotionCore::EndProgramBlockMotionCapture() noexcept
{
    m_programBlockMotionCaptureActive = false;
    return m_programBlockMotionCapture;
}


void MotionCore::RecordProgramBlockMotionSubmission(
    const MotionExecutionIdentity& identity,
    bool producerAccepted,
    MotionRejectReason immediateRejectReason) noexcept
{
    if (!m_programBlockMotionCaptureActive)
    {
        return;
    }

    if (m_programBlockMotionCapture.count >=
        MOTION_PROGRAM_BLOCK_CAPTURE_CAPACITY)
    {
        m_programBlockMotionCapture.overflow = true;
        return;
    }

    MotionProgramBlockSubmission& submission =
        m_programBlockMotionCapture.submissions[
            m_programBlockMotionCapture.count++];
    submission.identity = identity;
    submission.producerAccepted = producerAccepted;
    submission.immediateRejectReason = immediateRejectReason;
}

void MotionCore::RejectInvalidProducerMotionCommand(
    MotionCommand command,
    MotionExecutionEpoch executionEpoch,
    MotionCommandSource commandSource,
    const MotionOwnerLease& ownerLease) noexcept
{
    m_p1InvalidProducerRejectCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    AssignExecutionIdentity(
        command,
        executionEpoch,
        commandSource,
        ownerLease);

    m_lastRejectedSegmentId.store(
        command.execution.segmentId,
        std::memory_order_relaxed);

    // Publish one coherent Motion -> NC request before both the RT stop and
    // producer terminal notice. NC owns the AlarmManager write and opens the
    // old-Epoch lifecycle boundary before draining that notice.
    m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
    TryPublishGroupMappingIntegrityAlarmRequest(executionEpoch);

    // This API is called from the NC producer. Publish an RT-owned request;
    // never mutate AxisContext directly from the 10 ms side.
    RequestEmergencyStopAllAxes();

    TryQueueProducerFeedbackNotice(
        command,
        MotionFeedbackType::REJECTED,
        MotionRejectReason::INVALID_GEOMETRY,
        static_cast<std::uint32_t>(
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY),
        0.0);

    RecordProgramBlockMotionSubmission(
        command.execution,
        false,
        MotionRejectReason::INVALID_GEOMETRY);
}


MotionSegmentId MotionCore::AllocateMotionSegmentId() noexcept
{
    MotionSegmentId segmentId =
        m_nextSegmentId.fetch_add(
            1ULL,
            std::memory_order_relaxed);

    // SegmentId 0 保留為 INVALID。
    if (segmentId == MOTION_SEGMENT_ID_INVALID)
    {
        segmentId =
            m_nextSegmentId.fetch_add(
                1ULL,
                std::memory_order_relaxed);
    }

    return
        segmentId;
}


void MotionCore::AssignExecutionIdentity(
    MotionCommand& command,
    MotionExecutionEpoch exactEpoch,
    MotionCommandSource exactSource,
    const MotionOwnerLease& exactOwnerLease) noexcept
{
    command.execution.epoch =
        exactEpoch;

    command.execution.segmentId =
        AllocateMotionSegmentId();

    command.execution.sourceBlockId =
        static_cast<MotionSourceBlockId>(
            command.sourceLinePC);

    command.execution.source =
        exactSource;

    // Owner + Generation 必須和 Identity 同時封存在命令中。
    command.ownerLease =
        exactOwnerLease;
}


bool MotionCore::IsCommandFromCurrentEpoch(
    const MotionCommand& command) const noexcept
{
    const std::uint64_t publication =
        m_executionEpochPublication.load(
            std::memory_order_acquire);

    return
        command.execution.IsAssigned() &&
        command.execution.epoch ==
        UnpackExecutionEpochPublication(publication) &&
        command.execution.source ==
        UnpackExecutionEpochPublicationSource(publication);
}


bool MotionCore::HasPendingExecutionEpochChange() const noexcept
{
    return
        (m_executionEpochPublication.load(
            std::memory_order_acquire) &
            EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL;
}


bool MotionCore::IsCommandOwnerLeaseCurrent(
    const MotionCommand& command) const noexcept
{
    const MotionOwner expectedOwner =
        ResolveMotionOwnerForSource(
            command.execution.source);

    return
        expectedOwner != MotionOwner::NONE &&
        command.ownerLease.owner == expectedOwner &&
        IsMotionOwnerLeaseCurrent(command.ownerLease);
}


MotionRejectReason MotionCore::GetCommandAuthorizationFailure(
    const MotionCommand& command) const noexcept
{
    if (!IsCommandFromCurrentEpoch(command))
    {
        return MotionRejectReason::STALE_EPOCH;
    }

    if (!IsCommandOwnerLeaseCurrent(command))
    {
        return MotionRejectReason::OWNER_CONFLICT;
    }

    return MotionRejectReason::NONE;
}


namespace
{
    double ClampMotionProgress(
        double progress) noexcept
    {
        if (!std::isfinite(progress))
        {
            return 0.0;
        }

        if (progress < 0.0)
        {
            return 0.0;
        }

        if (progress > 1.0)
        {
            return 1.0;
        }

        return progress;
    }
}


MotionFeedbackSequence MotionCore::AllocateMotionFeedbackSequence() noexcept
{
    MotionFeedbackSequence sequence =
        m_nextMotionFeedbackSequence;

    if (sequence == MOTION_FEEDBACK_SEQUENCE_INVALID)
    {
        sequence = 1ULL;
    }

    m_nextMotionFeedbackSequence =
        (sequence ==
            (std::numeric_limits<MotionFeedbackSequence>::max)())
        ? 1ULL
        : static_cast<MotionFeedbackSequence>(
            sequence + 1ULL);

    return sequence;
}


bool MotionCore::TryQueueProducerFeedbackNotice(
    const MotionCommand& command,
    MotionFeedbackType type,
    MotionRejectReason rejectReason,
    std::uint32_t errorCode,
    double progress) noexcept
{
    MotionFeedbackEvent event{};
    event.identity = command.execution;
    event.type = type;
    event.rejectReason = rejectReason;
    event.owner = command.ownerLease.owner;
    event.ownerGeneration = command.ownerLease.generation;
    event.errorCode = errorCode;
    event.progress = ClampMotionProgress(progress);

    if (m_motionFeedbackChannel.
        CommandProducerTryPublishNotice(event))
    {
        return true;
    }

    m_motionFeedbackProducerNoticeOverflowCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_lastDroppedMotionFeedbackSegmentId.store(
        command.execution.segmentId,
        std::memory_order_relaxed);

    m_lastDroppedMotionFeedbackType.store(
        type,
        std::memory_order_relaxed);

    return false;
}


void MotionCore::DrainProducerFeedbackNotices() noexcept
{
    MotionFeedbackEvent event{};

    // 固定上限，避免 Producer Notice Burst 壟斷單一 250 us Cycle。
    // 未讀 Notice 留在 Ring，下一個 Runtime Cycle 繼續轉送。
    for (std::size_t i = 0U;
        i <
        MOTION_FEEDBACK_PRODUCER_NOTICE_DRAIN_LIMIT_PER_RUNTIME_CYCLE;
        ++i)
    {
        if (!m_motionFeedbackChannel.
            RuntimeTryConsumeProducerNotice(event))
        {
            break;
        }

        PublishMotionFeedback(event);
    }
}


bool MotionCore::PublishMotionFeedback(
    MotionFeedbackEvent event) noexcept
{
    event.sequence =
        AllocateMotionFeedbackSequence();

    if (!m_motionFeedbackChannel.
        RuntimeTryPublishFeedback(event))
    {
        m_motionFeedbackOverflowCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_lastDroppedMotionFeedbackSegmentId.store(
            event.identity.segmentId,
            std::memory_order_relaxed);

        m_lastDroppedMotionFeedbackType.store(
            event.type,
            std::memory_order_relaxed);

        return false;
    }

    m_lastPublishedMotionFeedbackSequence.store(
        event.sequence,
        std::memory_order_release);

    return true;
}


bool MotionCore::PublishMotionFeedbackForCommand(
    const MotionCommand& command,
    MotionFeedbackType type,
    MotionRejectReason rejectReason,
    std::uint32_t errorCode,
    double progress) noexcept
{
    MotionFeedbackEvent event{};
    event.identity = command.execution;
    event.type = type;
    event.rejectReason = rejectReason;
    event.owner = command.ownerLease.owner;
    event.ownerGeneration = command.ownerLease.generation;
    event.errorCode = errorCode;
    event.progress = ClampMotionProgress(progress);

    return
        PublishMotionFeedback(event);
}


double MotionCore::GetTrackedMotionProgress() const noexcept
{
    const double totalDistance =
        m_Group.virtualAxis.finalTargetPos;

    if (!std::isfinite(totalDistance) ||
        std::abs(totalDistance) <= 1e-12)
    {
        return 0.0;
    }

    const double progress =
        m_Group.virtualAxis.currentCmdPos /
        totalDistance;

    return
        ClampMotionProgress(progress);
}


void MotionCore::TrackMotionCommandAccepted(
    const MotionCommand& command) noexcept
{
    // 正常情況下，上一段會在 LoadNextCommand 入口先 COMPLETED。
    // 若未來其他路徑直接換段，這裡仍保證上一段不會無聲消失。
    if (m_feedbackTrackedAccepted &&
        !m_feedbackTrackedTerminal &&
        !m_feedbackTrackedIdentity.Matches(
            command.execution))
    {
        AbortTrackedMotionCommand();
    }

    m_feedbackTrackedIdentity =
        command.execution;

    m_feedbackTrackedOwnerLease =
        command.ownerLease;

    m_feedbackTrackedAccepted = true;
    m_feedbackTrackedStarted = false;
    m_feedbackTrackedTerminal = false;

    PublishMotionFeedbackForCommand(
        command,
        MotionFeedbackType::ACCEPTED,
        MotionRejectReason::NONE,
        0U,
        0.0);
}


void MotionCore::TrackMotionCommandStarted(
    const MotionCommand& command) noexcept
{
    if (!m_feedbackTrackedAccepted ||
        m_feedbackTrackedTerminal ||
        m_feedbackTrackedStarted ||
        !m_feedbackTrackedIdentity.Matches(
            command.execution))
    {
        return;
    }

    m_feedbackTrackedStarted = true;

    PublishMotionFeedbackForCommand(
        command,
        MotionFeedbackType::STARTED,
        MotionRejectReason::NONE,
        0U,
        0.0);
}


void MotionCore::CompleteTrackedMotionCommand(
    const MotionCommand& command) noexcept
{
    if (!m_feedbackTrackedAccepted ||
        m_feedbackTrackedTerminal ||
        !m_feedbackTrackedIdentity.Matches(
            command.execution))
    {
        return;
    }

    PublishMotionFeedbackForCommand(
        command,
        MotionFeedbackType::COMPLETED,
        MotionRejectReason::NONE,
        0U,
        1.0);

    m_feedbackTrackedTerminal = true;
}


void MotionCore::AbortTrackedMotionCommand() noexcept
{
    if (!m_feedbackTrackedAccepted ||
        m_feedbackTrackedTerminal ||
        !m_feedbackTrackedIdentity.IsAssigned())
    {
        return;
    }

    MotionFeedbackEvent event{};
    event.identity = m_feedbackTrackedIdentity;
    event.type = MotionFeedbackType::ABORTED;
    event.rejectReason = MotionRejectReason::NONE;
    event.owner = m_feedbackTrackedOwnerLease.owner;
    event.ownerGeneration = m_feedbackTrackedOwnerLease.generation;
    event.errorCode = 0U;
    event.progress = GetTrackedMotionProgress();

    PublishMotionFeedback(event);

    m_feedbackTrackedTerminal = true;
}


void MotionCore::FaultTrackedMotionCommand(
    std::uint32_t errorCode,
    MotionRejectReason reason) noexcept
{
    if (!m_feedbackTrackedAccepted ||
        m_feedbackTrackedTerminal ||
        !m_feedbackTrackedIdentity.IsAssigned())
    {
        return;
    }

    MotionFeedbackEvent event{};
    event.identity = m_feedbackTrackedIdentity;
    event.type = MotionFeedbackType::FAULTED;
    event.rejectReason = reason;
    event.owner = m_feedbackTrackedOwnerLease.owner;
    event.ownerGeneration = m_feedbackTrackedOwnerLease.generation;
    event.errorCode = errorCode;
    event.progress = GetTrackedMotionProgress();

    PublishMotionFeedback(event);

    m_feedbackTrackedTerminal = true;
}


bool MotionCore::IsTrackedMotionCommand(
    const MotionCommand& command) const noexcept
{
    return
        m_feedbackTrackedAccepted &&
        MotionExecutionIdentityExactlyMatches(
            m_feedbackTrackedIdentity,
            command.execution);
}


bool MotionCore::IsTerminalizedReplayOrTrackedCommand(
    const MotionCommand& command) const noexcept
{
    return
        command.replayTerminalAlreadyPublished ||
        IsTrackedMotionCommand(command);
}


void MotionCore::RejectMotionCommand(
    const MotionCommand& command,
    MotionRejectReason reason,
    std::uint32_t errorCode) noexcept
{
    if (command.replayTerminalAlreadyPublished)
    {
        return;
    }

    if (m_feedbackTrackedAccepted &&
        MotionExecutionIdentityExactlyMatches(
            m_feedbackTrackedIdentity,
            command.execution))
    {
        // B2 may keep a replay copy of the currently tracked segment.  Once
        // an Epoch stop has terminated that identity, retiring the replay
        // transport copy must not publish a conflicting second terminal.
        if (m_feedbackTrackedTerminal)
        {
            return;
        }

        if (reason == MotionRejectReason::STALE_EPOCH ||
            reason == MotionRejectReason::OWNER_CONFLICT)
        {
            AbortTrackedMotionCommand();
            return;
        }

        if (m_feedbackTrackedStarted)
        {
            FaultTrackedMotionCommand(errorCode, reason);
            return;
        }

        PublishMotionFeedbackForCommand(
            command,
            MotionFeedbackType::REJECTED,
            reason,
            errorCode,
            0.0);
        m_feedbackTrackedTerminal = true;
        return;
    }

    PublishMotionFeedbackForCommand(
        command,
        MotionFeedbackType::REJECTED,
        reason,
        errorCode,
        0.0);
}


bool MotionCore::TryEnqueueMotionCommand(
    const MotionCommand& command) noexcept
{
    const MotionRejectReason authorizationFailure =
        GetCommandAuthorizationFailure(command);

    if (authorizationFailure != MotionRejectReason::NONE)
    {
        if (authorizationFailure == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        m_lastRejectedSegmentId.store(
            command.execution.segmentId,
            std::memory_order_relaxed);

        TryQueueProducerFeedbackNotice(
            command,
            MotionFeedbackType::REJECTED,
            authorizationFailure,
            0U,
            0.0);

        RecordProgramBlockMotionSubmission(
            command.execution,
            false,
            authorizationFailure);
        return false;
    }

    if (m_Group.cmdQueue.ProducerTryPush(command))
    {
        RecordProgramBlockMotionSubmission(
            command.execution,
            true,
            MotionRejectReason::NONE);
        return true;
    }

    // 固定容量 Queue 不配置記憶體，也不覆寫尚未執行的命令。
    // 正常 NC 預讀在 50 筆就會節流，因此這個計數應維持 0。
    m_commandQueueFullRejectCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_lastRejectedSegmentId.store(
        command.execution.segmentId,
        std::memory_order_relaxed);

    // Queue Full 發生在 NC Producer 執行緒，不能直接寫入由 Runtime
    // 單一 Producer 擁有的 Final Feedback Ring。先送入 Producer Notice Ring。
    TryQueueProducerFeedbackNotice(
        command,
        MotionFeedbackType::REJECTED,
        MotionRejectReason::QUEUE_FULL,
        static_cast<std::uint32_t>(
            AlarmManager::MOTION_COMMAND_QUEUE_FULL),
        0.0);

    RecordProgramBlockMotionSubmission(
        command.execution,
        false,
        MotionRejectReason::QUEUE_FULL);

    AlarmManager::GetInstance().Trigger(
        AlarmManager::MOTION_COMMAND_QUEUE_FULL,
        command.sourceLinePC);

    return false;
}


bool MotionCore::TryPeekNextMotionCommand(
    MotionCommand& command) const noexcept
{
    return
        m_Group.cmdQueue.ConsumerTryPeek(
            command);
}


bool MotionCore::TryPeekQueuedMotionCommandAt(
    std::size_t offset,
    MotionCommand& command) const noexcept
{
    return
        m_Group.cmdQueue.ConsumerTryPeekAt(
            offset,
            command);
}


bool MotionCore::TryDequeueNextMotionCommand(
    MotionCommand& command) noexcept
{
    // A command belonging to a newly published Epoch may already be visible
    // in the SPSC ring while the 250 us owner has not yet applied that Epoch's
    // exact Abort policy.  Never pop/start it early: otherwise the following
    // pass would consume the publication and abort the command against its own
    // Epoch.  Peek is an acquire observation of the producer release, so the
    // preceding packed publication is visible to the pending check below.
    //
    // Peek 與 Pop 之間仍可能發生 RESET / GOTO / Owner Transfer；每一筆真正
    // 移除的命令都必須在 Pop 後再次驗證 Epoch + Owner Lease。
    while (m_staleCommandDiscardBudgetRemaining > 0U)
    {
        MotionCommand frontCommand{};
        if (!TryPeekNextMotionCommand(frontCommand))
        {
            return false;
        }

        if (HasPendingExecutionEpochChange())
        {
            // Leave the exact-current command in the ring.  The normal Epoch
            // seam applies the publication first; a later pass can then start
            // the command without a self-abort window.
            return false;
        }

        if (!m_Group.cmdQueue.ConsumerTryPop(
            command))
        {
            return false;
        }

        const MotionRejectReason authorizationFailure =
            GetCommandAuthorizationFailure(command);

        if (authorizationFailure == MotionRejectReason::NONE)
        {
            return true;
        }

        --m_staleCommandDiscardBudgetRemaining;

        const bool trackedTransportCopy =
            IsTerminalizedReplayOrTrackedCommand(command);
        if (!trackedTransportCopy &&
            authorizationFailure == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else if (!trackedTransportCopy)
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        if (!trackedTransportCopy)
        {
            m_lastRejectedSegmentId.store(
                command.execution.segmentId,
                std::memory_order_relaxed);
        }

        RejectMotionCommand(
            command,
            authorizationFailure,
            0U);
    }

    // Front 若仍未通過授權，留給下一個 250 us Cycle 繼續淘汰。
    return false;
}

bool MotionCore::TryRequeueMotionCommandFront(
    const MotionCommand& command) noexcept
{
    MotionCommand replayCommand = command;
    replayCommand.replayTerminalAlreadyPublished =
        command.replayTerminalAlreadyPublished ||
        !IsTrackedMotionCommand(command) ||
        m_feedbackTrackedTerminal;

    if (m_Group.cmdQueue.ConsumerTryPushFront(
        replayCommand))
    {
        return true;
    }

    m_commandReplayOverflowCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_lastRejectedSegmentId.store(
        command.execution.segmentId,
        std::memory_order_relaxed);

    // Replay Overflow 代表目前整體路徑執行已無法安全繼續。
    // B2 倒退跨節時 command 可能是歷史段，但 NC 正在等待的 Active
    // Segment 仍是原觸發段；優先以 Active Identity 結案，避免把 Fault
    // 錯記到暫時回放的歷史 Segment。
    if (m_feedbackTrackedAccepted &&
        !m_feedbackTrackedTerminal)
    {
        FaultTrackedMotionCommand(
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_REPLAY_QUEUE_FULL),
            MotionRejectReason::TRANSPORT_OVERFLOW);
    }
    else if (!command.replayTerminalAlreadyPublished)
    {
        PublishMotionFeedbackForCommand(
            command,
            MotionFeedbackType::FAULTED,
            MotionRejectReason::TRANSPORT_OVERFLOW,
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_REPLAY_QUEUE_FULL),
            0.0);
    }

    AlarmManager::GetInstance().Trigger(
        AlarmManager::MOTION_REPLAY_QUEUE_FULL,
        command.sourceLinePC);

    return false;
}


MotionExecutionEpoch MotionCore::RequestAbortingExecutionEpoch(
    MotionCommandSource source) noexcept
{
    return PublishNewExecutionEpoch(
        source,
        true);
}


void MotionCore::DiscardStaleQueuedCommands()
{
    MotionCommand queuedCommand{};

    while (m_staleCommandDiscardBudgetRemaining > 0U)
    {
        if (!TryPeekNextMotionCommand(
            queuedCommand))
        {
            return;
        }

        const MotionRejectReason authorizationFailure =
            GetCommandAuthorizationFailure(queuedCommand);

        if (authorizationFailure == MotionRejectReason::NONE)
        {
            return;
        }

        MotionCommand discardedCommand{};

        // Peek 已證明 Front 無效，因此使用 Raw Pop；不可呼叫會持續過濾的
        // TryDequeueNextMotionCommand()，以免多取第一筆有效命令。
        if (!m_Group.cmdQueue.ConsumerTryPop(
            discardedCommand))
        {
            return;
        }

        --m_staleCommandDiscardBudgetRemaining;

        const bool trackedTransportCopy =
            IsTerminalizedReplayOrTrackedCommand(discardedCommand);
        if (!trackedTransportCopy &&
            authorizationFailure == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else if (!trackedTransportCopy)
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        if (!trackedTransportCopy)
        {
            m_lastRejectedSegmentId.store(
                discardedCommand.execution.segmentId,
                std::memory_order_relaxed);
        }

        RejectMotionCommand(
            discardedCommand,
            authorizationFailure,
            0U);
    }

    // 本 Runtime Pass 的固定額度已用完；下一個 250 us Cycle 再處理。
}

void MotionCore::ApplyPendingExecutionEpochChange()
{
    std::uint64_t publication =
        m_executionEpochPublication.load(
            std::memory_order_acquire);

    if ((publication &
        EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL ||
        (publication &
            EXECUTION_EPOCH_PUBLICATION_PENDING) == 0ULL)
    {
        return;
    }

    // Consume exactly the Epoch/Source/Policy tuple that was observed.  The
    // RT seam makes one bounded strong-CAS attempt per call; a concurrent
    // publisher wins cleanly and the newer tuple is retried by the safety seam
    // below or on the next 250 us pass.
    const std::uint64_t consumedPublication = publication;
    const MotionExecutionEpoch consumedEpoch =
        UnpackExecutionEpochPublication(
            consumedPublication);
    const MotionCommandSource consumedSource =
        UnpackExecutionEpochPublicationSource(
            consumedPublication);
    const std::uint64_t appliedPublication =
        PackExecutionEpochPublication(
            consumedEpoch,
            consumedSource,
            false,
            false);

    if (!m_executionEpochPublication.compare_exchange_strong(
        publication,
        appliedPublication,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return;
    }

    publication = consumedPublication;

    const bool abortActiveCommand =
        (publication &
            EXECUTION_EPOCH_PUBLICATION_ABORT_ACTIVE) != 0ULL;

    // 目前已 ACCEPTED / STARTED、但尚未終止的 Active Segment，
    // 只要跨 Epoch 就必須以 ABORTED 結案，不能被誤報為 COMPLETED。
    AbortTrackedMotionCommand();

    // Replay 與 Ingress 共用同一個 Consumer View。先依 Epoch 逐筆淘汰，
    // 不會誤刪已經由 Producer 發布的新 Epoch 第一筆命令。
    DiscardStaleQueuedCommands();

    // 不允許 B2 / History 跨越 RESET、GOTO 或不同程式執行世代。
    m_Group.historyQueue.clear();

    // ABORTING 以前由 Producer 直接改 isActive。NC-0.1C 改由 250 us
    // Consumer 套用，避免 NC 執行緒寫入 Motion Runtime 狀態。
    if (abortActiveCommand)
    {
        m_Group.isActive = false;

        // Exact Abort is an immediate position hold.  It must not leave the
        // previous segment's virtual command speed alive after Group inactive.
        // Never snap command position to Actual here; Reset rebase still owns
        // that transition after its formal pre-proof.  ERROR/ESTOP/fault state
        // is never downgraded to IDLE; the formal proof must stay fail-closed.
        if (!m_Group.virtualAxis.isFault &&
            !m_Group.virtualAxis.isLagAlarm &&
            m_Group.virtualAxis.state !=
            MotionState::MotionState_ERROR &&
            m_Group.virtualAxis.state !=
            MotionState::MotionState_ESTOP)
        {
            m_Group.virtualAxis.state =
                MotionState::MotionState_IDLE;
            TryCanonicalizeIdleAxisCommandState(
                m_Group.virtualAxis);
        }

        if (m_pContexts != nullptr)
        {
            const int safeGroupAxisCount =
                (std::max)(0,
                    (std::min)(m_Group.axisCount, MAX_AXES));
            for (int groupAxis = 0;
                groupAxis < safeGroupAxisCount;
                ++groupAxis)
            {
                const int axisIndex =
                    m_Group.axisIndices[groupAxis];
                if (axisIndex < 0 ||
                    axisIndex >= static_cast<int>(m_pContexts->size()))
                {
                    continue;
                }

                TryCanonicalizeInactivePhysicalAxisCommandState(
                    (*m_pContexts)[axisIndex]);
            }
        }
    }

    if (!m_Group.isActive)
    {
        m_Group.currentCmd.execution =
            MotionExecutionIdentity{};

        m_Group.currentCmd.ownerLease =
            MotionOwnerLease{};
    }
}


// [修正] 改用 ServoDrive
void MotionCore::Link(std::vector<ENI_ServoDrive>* pAxisList)
{
    m_pAxes = pAxisList;
}
void MotionCore::Link(std::vector<ENI_ServoDrive>* pDriveList, std::vector<AxisContext>* pContextList)
{
    m_pDrives = pDriveList;
    m_pContexts = pContextList;
}


void MotionCore::BindStructuredServoReadShadowMaster(
    EtherCatMaster* pMaster)
{
    m_pStructuredServoReadShadowMaster =
        pMaster;
}


// ============================================================================
// Stage 11E.5 - Centralized Servo OUTPUT Command Seam
//
// Before E4 qualification:
//     legacy pOutput producer.
//
// After E4 qualification:
//     structured AxisIndex producer writes the SAME m_IoMap field.
//
// If structured write fails:
//     this same call immediately falls back to legacy pOutput.
// ============================================================================

void MotionCore::WriteServoControlWordCommand(
    ServoOutput* output,
    int axisIndex,
    uint16_t value)
{
    if (output == nullptr)
    {
        return;
    }

    bool structuredWritten = false;

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        structuredWritten =
            m_pStructuredServoReadShadowMaster->
            TryWriteMotionServoOutputCommandStructured(
                axisIndex,
                MotionServoOutputCommandField::ControlWord,
                static_cast<int64_t>(value),
                output);
    }

    if (!structuredWritten)
    {
        output->ControlWord = value;
    }

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        m_pStructuredServoReadShadowMaster->
            ObserveMotionServoOutputCommandSeamWriteShadow(
                axisIndex,
                MotionServoOutputCommandField::ControlWord,
                static_cast<int64_t>(value));
    }
}


void MotionCore::WriteServoTargetVelocityCommand(
    ServoOutput* output,
    int axisIndex,
    int32_t value)
{
    if (output == nullptr)
    {
        return;
    }

    bool structuredWritten = false;

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        structuredWritten =
            m_pStructuredServoReadShadowMaster->
            TryWriteMotionServoOutputCommandStructured(
                axisIndex,
                MotionServoOutputCommandField::TargetVelocity,
                static_cast<int64_t>(value),
                output);
    }

    if (!structuredWritten)
    {
        output->TargetVelocity = value;
    }

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        m_pStructuredServoReadShadowMaster->
            ObserveMotionServoOutputCommandSeamWriteShadow(
                axisIndex,
                MotionServoOutputCommandField::TargetVelocity,
                static_cast<int64_t>(value));
    }
}


void MotionCore::WriteServoTouchProbeFunctionCommand(
    ServoOutput* output,
    int axisIndex,
    uint16_t value)
{
    if (output == nullptr)
    {
        return;
    }

    bool structuredWritten = false;

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        structuredWritten =
            m_pStructuredServoReadShadowMaster->
            TryWriteMotionServoOutputCommandStructured(
                axisIndex,
                MotionServoOutputCommandField::TouchProbeFunction,
                static_cast<int64_t>(value),
                output);
    }

    if (!structuredWritten)
    {
        output->TouchProbeFunc = value;
    }

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        m_pStructuredServoReadShadowMaster->
            ObserveMotionServoOutputCommandSeamWriteShadow(
                axisIndex,
                MotionServoOutputCommandField::TouchProbeFunction,
                static_cast<int64_t>(value));
    }
}


void MotionCore::WriteServoModesOfOperationCommand(
    ServoOutput* output,
    int axisIndex,
    int8_t value)
{
    if (output == nullptr)
    {
        return;
    }

    bool structuredWritten = false;

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        structuredWritten =
            m_pStructuredServoReadShadowMaster->
            TryWriteMotionServoOutputCommandStructured(
                axisIndex,
                MotionServoOutputCommandField::ModesOfOperation,
                static_cast<int64_t>(value),
                output);
    }

    if (!structuredWritten)
    {
        output->ModesOfOperation = value;
    }

    if (m_pStructuredServoReadShadowMaster != nullptr)
    {
        m_pStructuredServoReadShadowMaster->
            ObserveMotionServoOutputCommandSeamWriteShadow(
                axisIndex,
                MotionServoOutputCommandField::ModesOfOperation,
                static_cast<int64_t>(value));
    }
}


// ============================================================================
// Stage 11D.6 - centralized active LEGACY Servo input snapshot.
// ============================================================================

bool MotionCore::ReadLegacyMotionServoInputSnapshot(
    const ENI_ServoDrive& servo,
    MotionServoInputSnapshot& snapshot) const
{
    snapshot =
        MotionServoInputSnapshot{};

    if (servo.pInput ==
        nullptr)
    {
        return false;
    }

    snapshot.StatusWord =
        servo.pInput->StatusWord;

    snapshot.ActualPosition =
        servo.pInput->ActualPosition;

    snapshot.ModesOfOperationDisplay =
        servo.pInput->ModesOfOperationDisplay;

    snapshot.TouchProbeStatus =
        servo.pInput->TouchProbeStatus;

    snapshot.TouchProbePosition =
        servo.pInput->TouchProbePos1;

    return true;
}


bool MotionCore::ReadLegacyMotionServoInputSnapshotBySlot(
    int motionSlot,
    MotionServoInputSnapshot& snapshot) const
{
    snapshot =
        MotionServoInputSnapshot{};

    if (m_pDrives ==
        nullptr ||
        motionSlot <
        0 ||
        motionSlot >=
        static_cast<int>(
            m_pDrives->size()))
    {
        return false;
    }

    return
        ReadLegacyMotionServoInputSnapshot(
            (*m_pDrives)[
                static_cast<size_t>(
                    motionSlot)],
            snapshot);
}


// ============================================================================
// Stage 11D.8 - Compatibility input seam
//
// Normal compatibility path after D7 qualification:
//
//     motionSlot
//         -> AxisContext.axisIndex
//         -> published Motion input snapshot
//
// Legacy pInput remains only as warmup / emergency compatibility fallback.
// ============================================================================

bool MotionCore::ReadMotionServoInputCompatibilitySnapshotBySlot(
    int motionSlot,
    MotionServoInputSnapshot& snapshot) const
{
    snapshot =
        MotionServoInputSnapshot{};


    if (motionSlot <
        0 ||
        m_pContexts ==
        nullptr ||
        motionSlot >=
        static_cast<int>(
            m_pContexts->size()))
    {
        return
            false;
    }


    if (m_pStructuredServoReadShadowMaster !=
        nullptr)
    {
        const int semanticAxisIndex =
            (*m_pContexts)[
                static_cast<size_t>(
                    motionSlot)]
            .axisIndex;


                if (m_pStructuredServoReadShadowMaster->
                    TryReadMotionServoPublishedInputCompatibility(
                        semanticAxisIndex,
                        snapshot))
                {
                    return
                        true;
                }
    }


    // D7 warmup or D8 compatibility fallback.
    return
        ReadLegacyMotionServoInputSnapshotBySlot(
            motionSlot,
            snapshot);
}


// 輔助函式: 符號判斷
int MotionCore::Sgn(double val) {
    return (0.0 < val) - (val < 0.0);
}

// 輔助函式: 單位轉換
double MotionCore::RpmToPps(double rpm, double resolution) {
    return (rpm / 60.0) * resolution;
}

double MotionCore::PpsToRpm(double pps, double resolution) {
    if (resolution == 0) return 0.0;
    return (pps * 60.0) / resolution;
}

double MotionCore::UnitPerMinToPps(double unitPerMin, double resolution, double finalLead)
{
    // 防呆：避免導程設定為 0 導致除以零崩潰
    if (std::abs(finalLead) < 0.000001) return 0.0;

    // 1. 每分鐘多少單位 -> 每秒多少單位 (mm/sec 或 deg/sec)
    double unitPerSec = unitPerMin / 60.0;

    // 2. 移動 1 單位需要多少 Pulse
    double pulsePerUnit = resolution / finalLead;

    // 3. 兩者相乘，得到每秒需要發出多少 Pulse (PPS)
    return unitPerSec * pulsePerUnit;
}

// ============================================================
// G81 HOME Helpers
// ============================================================

double MotionCore::GetRawLogicalPositionPulse(const AxisContext& axis) const
{
    return axis.currentActPos + axis.machineCoordinateOffsetPulse;
}

bool MotionCore::GetDriveTouchProbeFunction(int axisIndex, uint16_t& functionValue) const
{
    functionValue = 0;

    if (m_pDrives == nullptr ||
        axisIndex < 0 ||
        axisIndex >= static_cast<int>(m_pDrives->size()))
    {
        return false;
    }

    const ENI_ServoDrive& drive =
        (*m_pDrives)[axisIndex];

    if (drive.pOutput == nullptr)
    {
        return false;
    }

    functionValue =
        drive.pOutput->TouchProbeFunc;

    return true;
}

bool MotionCore::GetDriveTouchProbeData(int axisIndex, uint16_t& status, int32_t& capturedPosition) const
{
    status =
        0U;

    capturedPosition =
        0;

    MotionServoInputSnapshot
        input;

    if (!ReadMotionServoInputCompatibilitySnapshotBySlot(
        axisIndex,
        input))
    {
        return false;
    }

    status =
        input.TouchProbeStatus;

    capturedPosition =
        input.TouchProbePosition;

    return true;
}

bool MotionCore::SetDriveTouchProbeFunction(int axisIndex, uint16_t value)
{
    if (m_pDrives == nullptr ||
        axisIndex < 0 ||
        axisIndex >= static_cast<int>(
            m_pDrives->size()))
    {
        return
            false;
    }


    ENI_ServoDrive& drive =
        (*m_pDrives)[
            static_cast<size_t>(
                axisIndex)];


    if (drive.pOutput ==
        nullptr)
    {
        return
            false;
    }


    WriteServoTouchProbeFunctionCommand(
        drive.pOutput,
        axisIndex,
        value);


    return
        true;
}

double MotionCore::ConvertDriveCaptureToRawLogicalPulse(int axisIndex, int32_t capturedPosition, HomeReferenceSource source) const
{
    if (m_pContexts == nullptr || axisIndex < 0 || axisIndex >= static_cast<int>(m_pContexts->size())) return 0.0;
    const AxisContext& axis = (*m_pContexts)[axisIndex];

    if (source == HomeReferenceSource::LINEAR_SCALE_INDEX_DRIVE && axis.fbMode == FeedbackSource::LINEAR_SCALE)
        return static_cast<double>(capturedPosition) * axis.scaleToMotorRatio;

    const uint32_t currentRaw = static_cast<uint32_t>(axis.lastRawActPos);
    const uint32_t capturedRaw = static_cast<uint32_t>(capturedPosition);
    const int32_t delta = static_cast<int32_t>(capturedRaw - currentRaw);
    double logical = axis.unwrappedActPos + static_cast<double>(delta);
    if (axis.isReverse) logical = -logical;
    if (axis.Axis_Reverse) logical = -logical;
    return logical;
}

bool MotionCore::ApplyMachineHome(AxisContext& axis, double capturedReferencePulse, double homeOffsetUnit)
{
    if (!axis.isExist || axis.state != MotionState::MotionState_IDLE || !std::isfinite(capturedReferencePulse) || !std::isfinite(homeOffsetUnit) || axis.resolution_PPR <= 0.0 || std::abs(axis.finalLead) < 1.0e-12) return false;

    const double pulsePerUnit = axis.resolution_PPR / std::abs(axis.finalLead);
    const double homeOffsetPulse = homeOffsetUnit * pulsePerUnit;
    const double oldOffset = axis.machineCoordinateOffsetPulse;
    const double newOffset = capturedReferencePulse - homeOffsetPulse;
    const double shift = oldOffset - newOffset;

    axis.machineCoordinateOffsetPulse = newOffset;
    axis.currentActPos += shift;
    axis.currentCmdPos += shift;
    axis.logicalCmdPos += shift;
    axis.planningPos += shift;
    axis.finalTargetPos += shift;
    axis.startCmdPos += shift;
    axis.lastQueuedPulse += shift;
    axis.lastActPos += shift;

    axis.currentCmdVel = 0.0;
    axis.logicalCmdVel = 0.0;
    axis.currentActVel = 0.0;
    axis.targetVelocity = 0.0;
    axis.targetEndVel = 0.0;
    axis.motionTime = 0.0;
    axis.pid.prevError = 0.0;
    axis.pid.integralAcc = 0.0;
    for (size_t i = 0; i < axis.velBuffer.size(); ++i) axis.velBuffer[i] = 0.0;
    axis.bufferSum = 0.0;
    axis.bufferIndex = 0;
    axis.inPosition = true;
    return true;
}

// ==========================================
// [API] 初始化與設定
// =============================================================================
// Stage NC-0.2J.6.4 - Startup Feedback Alignment / Permanent Lag Arming
// =============================================================================
void MotionCore::AlignStartupAxisCommandToActual(
    AxisContext& axis) noexcept
{
    const double actualPosition = axis.currentActPos;

    axis.startCmdPos = actualPosition;
    axis.planningPos = actualPosition;
    axis.currentCmdPos = actualPosition;
    axis.logicalCmdPos = actualPosition;
    axis.finalTargetPos = actualPosition;
    axis.lastQueuedPulse = actualPosition;

    axis.currentCmdVel = 0.0;
    axis.logicalCmdVel = 0.0;
    axis.currentActVel = 0.0;
    axis.lastActPos = actualPosition;
    axis.targetVelocity = 0.0;
    axis.targetEndVel = 0.0;
    axis.cruiseVel_PPS = 0.0;
    axis.programmedVel_PPS = 0.0;
    axis.motionTime = 0.0;

    axis.pid.prevError = 0.0;
    axis.pid.integralAcc = 0.0;

    std::fill(axis.velBuffer.begin(), axis.velBuffer.end(), 0.0);
    axis.bufferSum = 0.0;
    axis.bufferIndex = 0;
    axis.inPosition = true;
}


bool MotionCore::ObserveStartupLagMonitorArming(
    AxisContext& axis,
    bool feedbackReady) noexcept
{
    // Arming is permanent for this boot.  After this point the exact legacy
    // Servo / compensation / PID / Lag path below remains authoritative.
    if (axis.startupLagMonitorArmed)
    {
        return true;
    }

    if (!axis.isExist)
    {
        return false;
    }

    // A sequencing violation is boot-latched.  An operator Reset must not
    // turn the forbidden pre-arm command into an implicit coordinate rebase.
    if (axis.startupLagPrematureMotionBlocked)
    {
        axis.startupLagFeedbackReady = false;
        axis.startupLagPositionAligned = false;
        axis.startupLagStableSampleCount = 0U;
        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        axis.isFault = true;
        axis.inPosition = false;
        axis.state = MotionState::MotionState_ERROR;
        return false;
    }

    // Existing ERROR / ESTOP evidence must never be hidden by coordinate
    // alignment.  The normal fault path will keep the final PDO command zero.
    if (axis.isFault ||
        axis.state == MotionState::MotionState_ERROR ||
        axis.state == MotionState::MotionState_ESTOP)
    {
        if (axis.startupLagStableSampleCount != 0U ||
            axis.startupLagPositionAligned ||
            axis.startupLagFeedbackReady)
        {
            ++m_startupLagReadinessResetProducer;
        }

        axis.startupLagFeedbackReady = false;
        axis.startupLagPositionAligned = false;
        axis.startupLagStableSampleCount = 0U;
        return false;
    }

    // Any motion before the one-shot startup contract is complete is a real
    // sequencing violation.  Preserve Cmd/Act evidence and fail closed.
    if (m_Group.isActive || axis.state != MotionState::MotionState_IDLE)
    {
        if (!axis.startupLagPrematureMotionBlocked)
        {
            axis.startupLagPrematureMotionBlocked = true;
            ++m_startupLagPrematureMotionBlockProducer;
        }

        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        axis.startupLagFeedbackReady = false;
        axis.startupLagPositionAligned = false;
        axis.startupLagStableSampleCount = 0U;
        axis.isFault = true;
        axis.inPosition = false;
        axis.state = MotionState::MotionState_ERROR;
        return false;
    }

    const bool trustworthyFeedback =
        feedbackReady && std::isfinite(axis.currentActPos);

    // Before arming, no command may retain the constructor's zero coordinate.
    // Even an unready sample is safe to mirror while the final PDO is forced
    // to zero; later trustworthy samples repeat this alignment before arming.
    if (std::isfinite(axis.currentActPos))
    {
        AlignStartupAxisCommandToActual(axis);
    }

    if (!trustworthyFeedback)
    {
        if (axis.startupLagStableSampleCount != 0U ||
            axis.startupLagPositionAligned ||
            axis.startupLagFeedbackReady)
        {
            ++m_startupLagReadinessResetProducer;
        }

        axis.startupLagFeedbackReady = false;
        axis.startupLagPositionAligned = false;
        axis.startupLagStableSampleCount = 0U;
        return false;
    }

    axis.startupLagFeedbackReady = true;
    if (!axis.startupLagPositionAligned)
    {
        axis.startupLagPositionAligned = true;
        ++m_startupLagAlignmentEventProducer;
    }

    if (axis.startupLagStableSampleCount <
        MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES)
    {
        ++axis.startupLagStableSampleCount;
    }

    if (axis.startupLagStableSampleCount >=
        MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES)
    {
        axis.startupLagStableSampleCount =
            MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES;
        axis.startupLagMonitorArmed = true;
        ++m_startupLagArmingTransitionProducer;

        // Keep this transition cycle at zero output.  The next PDO cycle is
        // the first cycle allowed to enter the unchanged runtime Lag check.
        return false;
    }

    return false;
}


void MotionCore::PublishStartupLagArmingEvidence() noexcept
{
    std::uint32_t existingMask = 0U;
    std::uint32_t readyMask = 0U;
    std::uint32_t alignedMask = 0U;
    std::uint32_t armedMask = 0U;
    std::uint32_t blockedMask = 0U;
    std::uint32_t minimumStableSamples = 0U;

    if (m_pContexts != nullptr)
    {
        const std::size_t axisCount = (std::min)(
            m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES));
        bool hasExistingAxis = false;
        minimumStableSamples = MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES;

        for (std::size_t axisSlot = 0U;
            axisSlot < axisCount;
            ++axisSlot)
        {
            const AxisContext& axis = (*m_pContexts)[axisSlot];
            if (!axis.isExist)
            {
                continue;
            }

            hasExistingAxis = true;
            const std::uint32_t bit =
                1U << static_cast<unsigned int>(axisSlot);
            existingMask |= bit;
            if (axis.startupLagFeedbackReady)
            {
                readyMask |= bit;
            }
            if (axis.startupLagPositionAligned)
            {
                alignedMask |= bit;
            }
            if (axis.startupLagMonitorArmed)
            {
                armedMask |= bit;
            }
            if (axis.startupLagPrematureMotionBlocked)
            {
                blockedMask |= bit;
            }

            minimumStableSamples = (std::min)(
                minimumStableSamples,
                axis.startupLagStableSampleCount);
        }

        if (!hasExistingAxis)
        {
            minimumStableSamples = 0U;
        }
    }

    const std::uint64_t packedMasks =
        static_cast<std::uint64_t>(existingMask & 0xFFU) |
        (static_cast<std::uint64_t>(readyMask & 0xFFU) << 8U) |
        (static_cast<std::uint64_t>(alignedMask & 0xFFU) << 16U) |
        (static_cast<std::uint64_t>(armedMask & 0xFFU) << 24U) |
        (static_cast<std::uint64_t>(blockedMask & 0xFFU) << 32U);

    m_startupLagAlignmentEvents.store(
        m_startupLagAlignmentEventProducer,
        std::memory_order_release);
    m_startupLagArmingTransitions.store(
        m_startupLagArmingTransitionProducer,
        std::memory_order_release);
    m_startupLagReadinessResets.store(
        m_startupLagReadinessResetProducer,
        std::memory_order_release);
    m_startupLagPrematureMotionBlocks.store(
        m_startupLagPrematureMotionBlockProducer,
        std::memory_order_release);
    m_startupLagMinimumStableSampleCount.store(
        minimumStableSamples,
        std::memory_order_release);
    m_startupLagArmingMasks.store(
        packedMasks,
        std::memory_order_release);
}


// ==========================================
void MotionCore::InitAxis(AxisContext& axis, double resolution)
{
    axis.isExist = false; // 🌟 預設為不存在，直到掃描到 EtherCAT 實體站點才開啟
    axis.machineCoordinateOffsetPulse = 0.0;
    // 1. 物理參數
    axis.resolution_PPR = resolution;

    // 2. 運動參數 (預設值)
    // 設定最大速度 3000 RPM (轉成 PPS)
    axis.maxVel_PPS = RpmToPps(3000.0, axis.resolution_PPR);

    // 設定加減速 (預設 0.5 秒達到滿速)
    axis.acc_PPS2 = axis.maxVel_PPS * 2.0;//2.0
    axis.dec_PPS2 = axis.maxVel_PPS * 2.0;//2.0

    // 3. 雙回授預設
    axis.fbMode = FeedbackSource::MOTOR_ENCODER;
    axis.pScaleActualPos = nullptr; // 預設沒有光學尺
    axis.scaleToMotorRatio = 1.0;
    axis.maxDeviation = axis.resolution_PPR * 0.1; // 允許 0.1 圈誤差

    // 4. 狀態初始化
    axis.currentCmdPos = 0.0;
    axis.currentCmdVel = 0.0;
    axis.currentActPos = 0.0;
    axis.logicalCmdPos = 0.0; // 🟢 確保同步
    axis.logicalCmdVel = 0.0;
    axis.targetVelocity = 0.0;
    axis.targetEndVel = 0.0;
    axis.planningPos = 0.0;
    axis.finalTargetPos = 0.0;


    axis.state = MotionState::MotionState_IDLE;
    axis.inPosition = true;
    axis.isFault = false;
    axis.isLagAlarm = false;

    axis.startupLagFeedbackReady = false;
    axis.startupLagPositionAligned = false;
    axis.startupLagMonitorArmed = false;
    axis.startupLagPrematureMotionBlocked = false;
    axis.startupLagStableSampleCount = 0U;

    axis.isServoOn = false;   // 開機必須強制為 false，直到 CiA 402 狀態機建立激磁
    axis.targetMode = 9;      // 預設 CSV

    // 5. PID 參數 (自動依解析度縮放)
    // 基準: 16777216 解析度下, Kp = 20
    double ratio = axis.resolution_PPR / 16777216.0;

    axis.pid.Kp = 20.0 * ratio;       // 剛性
    axis.pid.Ki = 0.0;                // 積分 (預設關閉)
    axis.pid.Kd = 0.0;                // 微分
    axis.pid.MaxIntegral = RpmToPps(100.0, axis.resolution_PPR);
    axis.pid.MaxLag = 100000.0 * ratio; // 允許誤差 (約2度)

    axis.pid.prevError = 0.0;
    axis.pid.integralAcc = 0.0;

    // 在 InitAxis 初始化時，把它轉成 Pulse 存起來
    double pulsePerUnit = axis.resolution_PPR / axis.finalLead;
    axis.inPositionWindow_Pulse = axis.inPositionWindow_mm * pulsePerUnit;
    // 把算好的 Pulse 寫入 PID 的 MaxLag 供伺服迴圈檢查
    axis.pid.MaxLag = axis.maxLag_mm * pulsePerUnit;
}

void MotionCore::InitSmoothBuffer(AxisContext& axis, double smoothTime_ms)
{
    // 計算需要幾個 Cycle 的 Buffer
    // 例如：平滑時間 100ms，週期 1ms -> 需要 100 個格子
    int steps = (int)(smoothTime_ms / (CYCLE_TIME_SEC * 1000.0));

    if (steps < 2) steps = 2; // 最少要有 2 格

    axis.velBuffer.resize(steps, 0.0); // 初始化為 0
    axis.bufferIndex = 0;
    axis.bufferSum = 0.0;
    axis.smoothTime_ms = smoothTime_ms;
}
// 🌟 實作綁定函式
void MotionCore::LinkCoordinateManager(CoordinateManager* pCoord)
{
    m_pCoordMgr = pCoord;
}
// 檔案：MotionCore.cpp
void MotionCore::UpdateAllMotion()//更新全部軸狀態 逐步激磁
{
    m_ncSettleMotionPassCompleted = false;

    // 1. 防呆：確保指標沒丟失
    if (m_pDrives == nullptr || m_pContexts == nullptr) {
        PublishStartupLagArmingEvidence();
        PublishStopSettleEvidence();
        return;
    }

    // 2. 防呆：確保兩個清單長度一致
    if (m_pDrives->size() != m_pContexts->size()) {
        // 這裡可以丟個錯誤 log
        PublishStartupLagArmingEvidence();
        PublishStopSettleEvidence();
        return;
    }


    // =========================================================
    // Stage 11D.3 - Motion Servo Input Consumer Bridge SHADOW
    //
    // This runs inside the existing Motion phase after the PDO
    // Process Image has been refreshed.
    //
    // No allocation.
    // No logging.
    // No write.
    // Actual UpdateMotion / UpdateServoState below remain legacy.
    // =========================================================

    bool legacyNormalPathRetired =
        false;
    bool motionInputComplete = true;


    if (m_pStructuredServoReadShadowMaster !=
        nullptr)
    {
        // =====================================================
        // Historical D3/D5 comparisons are required only until
        // Stage11D.9 retirement.
        // =====================================================

        if (!m_pStructuredServoReadShadowMaster->
            IsLegacyMotionServoInputNormalPathRetired())
        {
            m_pStructuredServoReadShadowMaster->
                SampleMotionServoInputConsumerBridgeShadow();
        }


        // Stage 11D.7 source gate.
        m_pStructuredServoReadShadowMaster->
            UpdateControlledMotionServoInputCutoverGate();


        // =====================================================
        // Stage 11D.9
        //
        // This gate may transition to retired state only after
        // Stage11D.8 is fully qualified.
        // =====================================================

        m_pStructuredServoReadShadowMaster->
            UpdateLegacyMotionServoInputNormalPathRetirementGate();


        // =====================================================
        // Stage 11D.10 - final Servo INPUT release gate.
        //
        // Once D9 and D2 are qualified this retires the final
        // D2 1ms legacy comparison sampler.
        // =====================================================

        m_pStructuredServoReadShadowMaster->
            UpdateServoInputReleaseGate();


        legacyNormalPathRetired =
            m_pStructuredServoReadShadowMaster->
            IsLegacyMotionServoInputNormalPathRetired();
    }


    for (size_t i = 0; i < m_pDrives->size(); ++i)
    {
        MotionServoInputSnapshot
            input;


        if (legacyNormalPathRetired &&
            m_pStructuredServoReadShadowMaster !=
            nullptr)
        {
            // =================================================
            // Stage 11D.9 NORMAL PATH
            //
            // No legacy pInput snapshot is read here.
            //
            // Semantic input is now the first and normal source.
            // =================================================

            const bool semanticPass =
                m_pStructuredServoReadShadowMaster->
                TryReadRetiredMotionServoInputByAxisIndex(
                    (*m_pContexts)[i].axisIndex,
                    input);


            if (!semanticPass)
            {
                // =============================================
                // Emergency only:
                //
                // Read legacy pInput ON DEMAND after a semantic
                // failure.  This is no longer the normal path.
                // =============================================

                if (!ReadLegacyMotionServoInputSnapshot(
                    (*m_pDrives)[i],
                    input))
                {
                    motionInputComplete = false;
                    continue;
                }


                m_pStructuredServoReadShadowMaster->
                    ReportLegacyMotionServoInputEmergencyFallback();


                // One semantic route failure ends retirement for
                // the remainder of this boot.
                legacyNormalPathRetired =
                    false;
            }
        }
        else
        {
            // =================================================
            // Pre-retirement compatibility path.
            //
            // Required for D3/D5/D6/D7/D8 current-boot
            // qualification.
            // =================================================

            MotionServoInputSnapshot
                legacyInput;


            if (!ReadLegacyMotionServoInputSnapshot(
                (*m_pDrives)[i],
                legacyInput))
            {
                motionInputComplete = false;
                continue;
            }


            if (m_pStructuredServoReadShadowMaster !=
                nullptr)
            {
                m_pStructuredServoReadShadowMaster->
                    ObserveMotionServoInputConsumerSeamShadow(
                        (*m_pContexts)[i].axisIndex,
                        legacyInput.StatusWord,
                        legacyInput.ActualPosition,
                        legacyInput.ModesOfOperationDisplay,
                        legacyInput.TouchProbeStatus,
                        legacyInput.TouchProbePosition);
            }


            input =
                legacyInput;


            if (m_pStructuredServoReadShadowMaster !=
                nullptr)
            {
                m_pStructuredServoReadShadowMaster->
                    TryReadControlledMotionServoInputByAxisIndex(
                        (*m_pContexts)[i].axisIndex,
                        input);
            }
        }


        if (m_pStructuredServoReadShadowMaster !=
            nullptr)
        {
            // Stage 11D.8 publication remains active in both
            // warmup and retired modes.
            m_pStructuredServoReadShadowMaster->
                PublishMotionServoInputConsumerSnapshot(
                    (*m_pContexts)[i].axisIndex,
                    input);
        }


        UpdateMotion(
            (*m_pDrives)[i],
            (*m_pContexts)[i],
            input);


        UpdateServoState(
            (*m_pDrives)[i],
            (*m_pContexts)[i],
            input);


        // =====================================================
        // Stage 11E.6 - Servo OUTPUT final-command dispatcher.
        //
        // Before E5 release:
        //   runs the historical E1->E5 qualification chain.
        //
        // After E5 release:
        //   freezes historical output qualification and directly
        //   validates the structured output mainline.
        // =====================================================

        if (m_pStructuredServoReadShadowMaster !=
            nullptr &&
            (*m_pDrives)[i].pOutput !=
            nullptr)
        {
            const ENI_ServoDrive&
                commandServo =
                (*m_pDrives)[i];


            m_pStructuredServoReadShadowMaster->
                ObserveMotionServoOutputFinalCommandRuntime(
                    (*m_pContexts)[i].axisIndex,
                    commandServo.pOutput->ControlWord,
                    commandServo.pOutput->TargetVelocity,
                    commandServo.pOutput->TouchProbeFunc,
                    commandServo.pOutput->ModesOfOperation);
        }
    }


    //座標轉換：直接使用內部的 m_pCoordMgr 指標

    if (m_pCoordMgr != nullptr)
    {
        double tempMCS[8] = { 0.0 };

        for (size_t i = 0; i < 8; ++i)
        {
            if (i < m_pContexts->size())
            {
                AxisContext& axis = (*m_pContexts)[i];
                double rawPulse = axis.currentActPos;
                double unitsPerPulse = (axis.resolution_PPR > 0) ? (axis.finalLead / axis.resolution_PPR) : 0.0;

                double physicalPos = rawPulse * unitsPerPulse;

                // 🌟 [新增顯示過濾] 如果是標準旋轉軸，顯示時強制 Modulo 360
                if (axis.axisType == AxisType::ROTARY)
                {
                    // 使用 fmod 取餘數
                    physicalPos = std::fmod(physicalPos, axis.rotaryModulo);

                    // 處理負數 (fmod 在 C++ 對負數取餘仍為負，需轉為正數)
                    if (physicalPos < 0.0) physicalPos += axis.rotaryModulo;
                }

                // ========================================================
         // 🌟 [神級修復]：直接把 physicalPos 給 tempMCS！
         // 因為大腦 (currentActPos) 早就已經是完美的邏輯方向了，絕對不要再反轉它！
         // ========================================================
                tempMCS[i] = physicalPos;
            }
            else
            {
                tempMCS[i] = 0.0;
            }
        }

        m_pCoordMgr->UpdateActualMCS(tempMCS);
    }





    // Stage NC-0.2J.4: the 250 us Motion owner is the only code allowed to
    // inspect AxisContext / Group while constructing this diagnostic image.
    // Keep publication at the end of the completed Motion pass so final PDO
    // TargetVelocity and encoder-derived Actual Velocity describe this cycle.
    m_ncSettleMotionPassCompleted = motionInputComplete;
    PublishStartupLagArmingEvidence();
    PublishStopSettleEvidence();
}

void MotionCore::ExportDebugInfo(SHM_AxisDebugInfo* outDebugArray, bool outputInMM)
{
    if (m_pContexts == nullptr || outDebugArray == nullptr) return;

    for (size_t i = 0; i < m_pContexts->size() && i < 8; ++i)
    {
        AxisContext& axis = (*m_pContexts)[i];

        // 1. 取得解析度與導程 (防止除零)
        double resolution = (axis.resolution_PPR > 0.0) ? axis.resolution_PPR : 1.0;
        double lead = (axis.finalLead > 0.0) ? axis.finalLead : 1.0;
        double unitsPerPulse = lead / resolution; // 1 Pulse = 多少 mm

        // 2. 基礎數值匯出
        outDebugArray[i].CmdPos = axis.currentCmdPos * (outputInMM ? unitsPerPulse : 1.0);
        outDebugArray[i].ActPos = axis.currentActPos * (outputInMM ? unitsPerPulse : 1.0);
        outDebugArray[i].LagError = (axis.currentCmdPos - axis.currentActPos) * (outputInMM ? unitsPerPulse : 1.0);
        outDebugArray[i].CmdVel = axis.currentCmdVel * (outputInMM ? unitsPerPulse : 1.0);
        outDebugArray[i].ActVel = axis.currentActVel * (outputInMM ? unitsPerPulse : 1.0);
        outDebugArray[i].MaxLagLimit = axis.pid.MaxLag * (outputInMM ? unitsPerPulse : 1.0);

        // 🌟 3. 計算 RPM (實際轉速)
        // 公式：(Pulse/sec * 60秒) / 每轉 Pulse 數
        outDebugArray[i].ActualRPM = (axis.currentActVel * 60.0) / resolution;

        // 🌟 4. 計算 m/min (米/分鐘)
        // 計算邏輯： (PPS * (mm/Pulse) * 60秒) / 1000 (轉為米)
        // 只有當單位是 mm 時這個才有意義
        double mm_per_min = (axis.currentActVel * unitsPerPulse) * 60.0;
        outDebugArray[i].ActualMpm = mm_per_min / 1000.0;

        // 狀態
        outDebugArray[i].State = (int)axis.state;
        outDebugArray[i].IsServoOn = axis.isServoOn ? 1 : 0;
        outDebugArray[i].IsFault = axis.isFault ? 1 : 0;
        outDebugArray[i].IsLagAlarm = axis.isLagAlarm ? 1 : 0;
        outDebugArray[i].IsHomed = axis.isHomed ? 1 : 0;

        // =====================================================
        // Drive Touch Probe Raw Monitor
        // =====================================================

        outDebugArray[i].TouchProbeFunction = 0;
        outDebugArray[i].TouchProbeStatus = 0;
        outDebugArray[i].TouchProbePosition = 0;

        if (m_pDrives != nullptr &&
            i < m_pDrives->size())
        {
            const ENI_ServoDrive& drive =
                (*m_pDrives)[i];

            if (drive.pOutput != nullptr)
            {
                outDebugArray[i].TouchProbeFunction =
                    drive.pOutput->TouchProbeFunc;
            }

            MotionServoInputSnapshot
                input;

            if (ReadMotionServoInputCompatibilitySnapshotBySlot(
                static_cast<int>(
                    i),
                input))
            {
                outDebugArray[i].TouchProbeStatus =
                    input.TouchProbeStatus;

                outDebugArray[i].TouchProbePosition =
                    input.TouchProbePosition;
            }
        }
    }
}
void MotionCore::UpdateServoState(
    ENI_ServoDrive& servo,
    AxisContext& axis,
    const MotionServoInputSnapshot& input)//更新單軸狀態 逐步激磁
{
    if (servo.pOutput ==
        nullptr)
    {
        return;
    }

    const uint16_t statusWord =
        input.StatusWord;

    // 1. 強制設定 CSV 模式 (Mode 9)
    WriteServoModesOfOperationCommand(
        servo.pOutput,
        axis.axisIndex,
        9);

    // 2. [CiA 402 狀態機]
    // 遮罩與值定義 (為了可讀性)
    const uint16_t MASK_STATE = 0x006F;
    const uint16_t MASK_FAULT = 0x0008;

    // =========================================================
      // A. 檢查故障 (Fault)
      // =========================================================
    if ((statusWord & MASK_FAULT) != 0)
    {
        axis.isServoOn = false;

        if (axis.resetRequest)
        {
            // 🌟 情況 1：系統正在嘗試復歸 (開機初始化，或是手動按了 Reset)
            WriteServoControlWordCommand(
                servo.pOutput,
                axis.axisIndex,
                0x0080); // 送出 Fault Reset
            // 💡 故意不把 axis.isFault 設為 true，保護 NC 不跳機
        }
        else
        {
            // 🌟 情況 2：非預期的真實警報 (例如加工中過載、撞到極限)
            axis.isFault = true;
            axis.state = MotionState::MotionState_ERROR;
            WriteServoControlWordCommand(
                servo.pOutput,
                axis.axisIndex,
                0x0000); // 停止送出指令
        }
    }
    else
    {
        // =========================================================
        // 只要脫離了 FAULT 狀態，立刻清除復歸旗標
        // =========================================================
        axis.resetRequest = false;

        // B. Switch On Disabled (驅動器剛上電，未準備好)
        if ((statusWord & 0x004F) == 0x0040)
        {
            WriteServoControlWordCommand(
                servo.pOutput,
                axis.axisIndex,
                0x0006); // Shutdown
        }
        // C. Ready to Switch On (準備就緒)
        else if ((statusWord & MASK_STATE) == 0x0021)
        {
            WriteServoControlWordCommand(
                servo.pOutput,
                axis.axisIndex,
                0x0007); // Switch On
        }
        // D. Switched On (電路已接通，等待最後一指令)
        else if ((statusWord & MASK_STATE) == 0x0023)
        {
            WriteServoControlWordCommand(
                servo.pOutput,
                axis.axisIndex,
                0x000F); // Enable Operation (激磁！)
        }
        // E. Operation Enabled (已成功激磁)
        else if ((statusWord & MASK_STATE) == 0x0027)
        {
            WriteServoControlWordCommand(
                servo.pOutput,
                axis.axisIndex,
                0x000F); // 維持激磁狀態
            axis.isServoOn = true;
        }
        else
        {
            // 狀態未知或正在切換中
            axis.isServoOn = false;
        }
    }
}

double MotionCore::CalculateShortestTarget(double currentPos, double targetPos, double modulo)
{
    // 防呆
    if (modulo <= 0.0) return targetPos;

    // 1. 將「目前絕對位置」轉換到 0 ~ 360 的真實角度
    double curMod = std::fmod(currentPos, modulo);
    if (curMod < 0.0) curMod += modulo;

    // 2. 將「指令目標位置」轉換到 0 ~ 360 的真實角度
    double tgtMod = std::fmod(targetPos, modulo);
    if (tgtMod < 0.0) tgtMod += modulo;

    // 3. 計算兩者的「最小角度差」
    double diff = tgtMod - curMod;

    // 4. 判斷最短路徑：如果差距超過半圈 (180度)，就反過來走！
    double halfModulo = modulo / 2.0;
    if (diff > halfModulo) {
        diff -= modulo; // 原本要正轉超過180度 -> 改成逆轉
    }
    else if (diff < -halfModulo) {
        diff += modulo; // 原本要逆轉超過180度 -> 改成正轉
    }

    // 5. 🌟 關鍵：將算出來的「真實角度差」，加上原本龐大的絕對累積座標！
    // 範例： 7696.0 + diff
    return currentPos + diff;
}
void MotionCore::MoveToPosition(AxisContext& axis, double targetPos, double targetVel, double acc_time, double dec_time)
{

    // =========================================================
    // 🌟 [新增] 旋轉軸最短路徑展開
    // =========================================================
    if (axis.axisType == AxisType::ROTARY && axis.useShortestPath)
    {
        // 將指令的目標度數 (例如 10度)，依據現在的絕對度數 (例如 350度)
        // 轉換為真實要走的物理連續度數 (變成 370度，往前走 20度)
        targetPos = CalculateShortestTarget(axis.currentCmdPos, targetPos, axis.rotaryModulo);
    }


    // =========================================================
    //  1. 【起跑線對齊 (Bumpless Transfer)】
    // 防止 PID 瞬間爆衝，消除高達上億的 Lag Error！
    // =========================================================
    if (axis.state == MotionState::MotionState_IDLE ||
        axis.state == MotionState::MotionState_ERROR)
    {
        axis.currentCmdPos = axis.currentActPos;
        axis.logicalCmdPos = axis.currentActPos; // 🟢 同步起跑線
        axis.currentCmdVel = 0.0;
    }

    // =========================================================
    //  2. 【安全限速檢查】(必須在計算加減速之前！)
    // =========================================================
    if (targetVel > axis.maxVel_PPS) {
        targetVel = axis.maxVel_PPS;
    }
    // 防呆：防止使用者輸入 0 造成除以零或不動
    if (targetVel <= 1.0) {
        targetVel = axis.maxVel_PPS * 0.1;
    }

    // =========================================================
    //  3. 【儲存原始指令速度】
    // 把它鎖進保險箱，供 UpdateMotion 乘上進給倍率 (Feedrate Override)
    // =========================================================
    axis.programmedVel_PPS = targetVel;

    // =========================================================
 //  4. 【計算真實加減速度 (PPS^2)】
 // =========================================================

 // ---------------------------------------------------------
 // 防呆：加速時間不能為 0 或負數
 //
 // 若上層真的傳入 0，使用 0.2 秒作為安全預設值。
 // 不能先做 targetVel / 0。
 // ---------------------------------------------------------
    double safeAccTime = acc_time;

    if (safeAccTime < 0.001)
    {
        safeAccTime = 0.2;
    }


    // ---------------------------------------------------------
    // 防呆：減速時間不能為 0 或負數
    //
    // dec_time 未指定或異常時，直接沿用加速時間。
    // ---------------------------------------------------------
    double safeDecTime = dec_time;

    if (safeDecTime < 0.001)
    {
        safeDecTime = safeAccTime;
    }


    // ---------------------------------------------------------
    // 現在才真正計算加減速度
    // ---------------------------------------------------------
    double calc_acc =
        targetVel / safeAccTime;

    double calc_dec =
        targetVel / safeDecTime;


    // ---------------------------------------------------------
    // 最低加減速度防呆
    // 避免異常參數導致幾乎不會移動
    // ---------------------------------------------------------
    if (calc_acc <= 10.0)
    {
        calc_acc = 10000.0;
    }

    if (calc_dec <= 10.0)
    {
        calc_dec = 10000.0;
    }

    // =========================================================
    //  5. 【設定運動參數給軌跡規劃器】
    // =========================================================
    axis.cruiseVel_PPS = targetVel;  // 給定當前巡航速度
    axis.acc_PPS2 = calc_acc;
    axis.dec_PPS2 = calc_dec;
    axis.finalTargetPos = targetPos;
    axis.targetEndVel = 0.0;         // 單軸定位，終點速度強制為 0

    // =========================================================
    //  6. 【單軸軌跡初始化】
    // =========================================================
    axis.planningPos = axis.currentCmdPos; // 梯形規劃從這裡開始算
    axis.startCmdPos = axis.currentCmdPos; // 紀錄起點 (備用)
    axis.motionTime = 0.0;                 // 碼表歸零

    // [極度重要] 清空 S-Curve 濾波緩衝區！
    // 避免上一次測試殘留的「負速度」影響到這次剛起步的波形
    for (size_t i = 0; i < axis.velBuffer.size(); ++i) {
        axis.velBuffer[i] = 0.0;
    }
    axis.bufferSum = 0.0;
    axis.bufferIndex = 0;

    // =========================================================
    //  7. 【扣下扳機，開始移動！】
    // =========================================================
    axis.state = MotionState::MotionState_MOVING;
    axis.inPosition = false;
    axis.isFault = false;
}

void MotionCore::VelocityMove(AxisContext& axis, double velocity, double acc_time)
{



    // 1. 無論如何，都要更新目標速度 (這是每一圈都要做的)
    axis.targetVelocity = velocity;



    double calculated_acc = 0.0;

    // 1. 如果時間設為 0 (或極小)，代表要瞬間到位
    if (acc_time < 0.001)
    {
        calculated_acc = 0.0; // 傳入 0 給底層，代表無限大加速度
    }
    // 2. 正常計算：加速度 = 速度變化量 / 時間
    else
    {
        // 為了計算斜率，我們取目標速度的絕對值
        double target_mag = std::abs(velocity);

        // [特殊處理] 如果目標速度是 0 (要停車)
        // 我們不能用 0 除以時間 (會得到 0 加速度，導致卡死)
        // 所以這裡我們假設是要從 "最大速度" 煞車到 0 的斜率，或是用當前速度
        // 簡單做法：如果目標是 0，就用 MaxVel 來算斜率，保證煞車力道足夠
        if (target_mag < 1.0) {
            target_mag = axis.maxVel_PPS;
        }

        // 公式：A = V / t
        calculated_acc = target_mag / acc_time;
    }



    axis.VelocityMove_Acc = calculated_acc;






    // =========================================================
    // 3. [關鍵修正] 狀態切換保護
    // 只有當 "原本不是速度模式" 時，才執行初始化！
    // =========================================================
    if (axis.state != MotionState::MotionState_VELOCITY)
    {
        // 這些程式碼 "絕對不能" 在每一圈都執行，否則積分會被清零
        axis.state = MotionState::MotionState_VELOCITY;

        // 讓虛擬規劃點同步當前位置 (防止跳動)
        axis.planningPos = axis.currentCmdPos;

        // 注意：千萬不要在這裡把 axis.currentCmdVel 歸零！
        // 如果你要從運動中無縫切換，保留原本的速度是正確的。
    }
}
// =========================================================
// MPG Move
//
// Handwheel Position Following
//
// 第一次進入 MPG：
//     初始化 MPG 狀態
//
// 已經在 MPG：
//     只更新 finalTargetPos
//     不重新初始化
//     不重新清 S-Curve
//
// targetPos:
//     Absolute Pulse Target
//
// maxVel:
//     MPG Maximum Following Velocity
// =========================================================

void MotionCore::MPGMove(
    AxisContext& axis,
    double targetPos,
    double maxVel,
    double acc_time,
    double dec_time)
{
    // =====================================================
    // Basic Parameter Validation
    // =====================================================

    if (axis.maxVel_PPS <= 0.0)
    {
        return;
    }


    double safeMaxVel =
        std::abs(maxVel);


    if (safeMaxVel <= 0.0)
    {
        return;
    }


    if (safeMaxVel >
        axis.maxVel_PPS)
    {
        safeMaxVel =
            axis.maxVel_PPS;
    }


    // =====================================================
    // Safe Acc / Dec Time
    // =====================================================

    double safeAccTime =
        acc_time;


    if (safeAccTime < 0.001)
    {
        safeAccTime =
            0.2;
    }


    double safeDecTime =
        dec_time;


    if (safeDecTime < 0.001)
    {
        safeDecTime =
            safeAccTime;
    }


    const double acc =
        safeMaxVel /
        safeAccTime;


    const double dec =
        safeMaxVel /
        safeDecTime;


    // =====================================================
    // Already MPG
    //
    // Handwheel 每增加一格，只更新 Target。
    //
    // 絕對不能：
    //
    // - Reset currentCmdPos
    // - Reset currentCmdVel
    // - Clear S-Curve
    // - Reset motionTime
    //
    // 否則手輪快速旋轉會一直重新起步。
    // =====================================================

    if (axis.state ==
        MotionState::MotionState_MPG)
    {
        axis.finalTargetPos =
            targetPos;

        axis.cruiseVel_PPS =
            safeMaxVel;

        axis.acc_PPS2 =
            acc;

        axis.dec_PPS2 =
            dec;


        if (std::abs(
            axis.finalTargetPos -
            axis.currentCmdPos) > 0.01)
        {
            axis.inPosition =
                false;
        }


        return;
    }


    // =====================================================
    // MPG 只能從 IDLE 接管
    //
    // 不允許搶：
    //
    // P2P
    // Continuous JOG
    // Fine JOG
    // Interpolation
    // STOPPING
    // =====================================================

    if (axis.state !=
        MotionState::MotionState_IDLE)
    {
        return;
    }


    // =====================================================
    // First MPG Entry
    // =====================================================

    axis.finalTargetPos =
        targetPos;

    axis.cruiseVel_PPS =
        safeMaxVel;

    axis.acc_PPS2 =
        acc;

    axis.dec_PPS2 =
        dec;

    axis.targetEndVel =
        0.0;

    axis.targetVelocity =
        0.0;


    // IDLE 起點保持 Command Position。
    axis.planningPos =
        axis.currentCmdPos;

    axis.currentCmdVel =
        0.0;


    // =====================================================
    // S-Curve
    //
    // 只在「第一次進入 MPG」清一次。
    //
    // 後續 Handwheel Count 更新不可再清。
    // =====================================================

    for (size_t i = 0;
        i < axis.velBuffer.size();
        ++i)
    {
        axis.velBuffer[i] =
            0.0;
    }


    axis.bufferSum =
        0.0;

    axis.bufferIndex =
        0;


    axis.inPosition =
        std::abs(
            axis.finalTargetPos -
            axis.currentCmdPos) <= 0.01;


    axis.state =
        MotionState::MotionState_MPG;
}
void MotionCore::StopMove(AxisContext& axis, double dec_time)
{
    // =========================================================
    // 1. 已經停止就不需要再次處理
    // =========================================================
    if (axis.state == MotionState::MotionState_IDLE)
    {
        // Repair legacy/in-flight IDLE snapshots that still carry a finite
        // planner velocity.  Non-finite state remains untouched and therefore
        // continues to fail the formal Reset proof closed.
        TryCanonicalizeIdleAxisCommandState(axis);
        return;
    }


    // =========================================================
    // 2. 減速時間安全防呆
    //
    // Header 允許 StopMove(axis) 不帶 dec_time，
    // 因此 dec_time 可能是 0。
    //
    // 絕對不能直接：
    //
    //     max_v / dec_time
    //
    // 否則會發生除以 0。
    //
    // 未設定時預設使用 0.2 秒。
    // =========================================================
    double safeDecTime = dec_time;

    if (safeDecTime < 0.001)
    {
        safeDecTime = 0.2;
    }


    // =========================================================
    // 3. 以「目前真正命令速度」計算減速度
    //
    // HOME DOG / INDEX 通常使用低速搜尋。
    // 若用 axis.maxVel_PPS 計算 dec_PPS2，低速 HOME 也可能
    // 接近瞬間停車，造成馬達或機構震動。
    //
    // 所以 StopMove(dec_time) 的語意固定為：
    //
    //     從目前命令速度，在 dec_time 秒內平順降到 0。
    // =========================================================

    double currentSpeed =
        std::abs(axis.currentCmdVel);

    if (currentSpeed < 0.1)
    {
        currentSpeed =
            std::abs(axis.logicalCmdVel);
    }

    if (currentSpeed < 0.1)
    {
        currentSpeed =
            std::abs(axis.targetVelocity);
    }

    if (currentSpeed < 0.1)
    {
        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        axis.finalTargetPos = axis.currentCmdPos;
        axis.state = MotionState::MotionState_IDLE;
        TryCanonicalizeIdleAxisCommandState(axis);
        return;
    }


    // =========================================================
    // 4. 計算減速度
    // =========================================================

    axis.dec_PPS2 =
        currentSpeed / safeDecTime;


    // =========================================================
    // 5. 依目前運動模式進行停止
    // =========================================================

    // ---------------------------------------------------------
    // A. P2P 定位移動
    //
    // 不直接切換 STOPPING，
    // 而是修改最後停止位置，
    // 讓原本梯形軌跡規劃器自己完成減速。
    // ---------------------------------------------------------
    if (axis.state == MotionState::MotionState_MOVING)
    {
        double currentSpeed =
            std::abs(axis.currentCmdVel);

        double stopDist =
            (currentSpeed * currentSpeed) /
            (2.0 * axis.dec_PPS2);

        if (axis.currentCmdVel > 0.0)
        {
            axis.finalTargetPos =
                axis.currentCmdPos + stopDist;
        }
        else if (axis.currentCmdVel < 0.0)
        {
            axis.finalTargetPos =
                axis.currentCmdPos - stopDist;
        }
        else
        {
            axis.finalTargetPos =
                axis.currentCmdPos;
        }
    }

    // ---------------------------------------------------------
    // B. Velocity Mode
    //
    // 未來 JOG 就會走這裡。
    //
    // C120 X+ 放開：
    //
    // VelocityMove
    //      ↓
    // StopMove
    //      ↓
    // STOPPING
    //      ↓
    // Calc_Trajectory_Velocity()
    //      ↓
    // 速度逐漸降至 0
    //      ↓
    // IDLE
    // ---------------------------------------------------------
    else
    {
        axis.state =
            MotionState::MotionState_STOPPING;
    }
}
void MotionCore::EmergencyStop(AxisContext& axis)
{
    // 如果已經在 ERROR 或 ESTOP 狀態，就不需要重複觸發
    if (axis.state == MotionState::MotionState_ERROR ||
        axis.state == MotionState::MotionState_ESTOP) return;



    // 1. 狀態強制切換為 ESTOP
    axis.state = MotionState::MotionState_ESTOP;

    // 2. 瞬間掐斷大腦所有的速度
    axis.currentCmdVel = 0.0;
    axis.targetVelocity = 0.0;
    axis.logicalCmdVel = 0.0; // 🌟 [新增] 同步清空邏輯速度，避免空間轉換矩陣產生突波

    // 3. 強制截斷所有「未來的目標」
    // 🌟 [新增] 把終點和規劃點都死鎖在「發生急停的這一瞬間」
    // 這樣就算急停解除，大腦也不會企圖去追趕原本沒跑完的路線
    axis.planningPos = axis.currentCmdPos;
    axis.finalTargetPos = axis.currentCmdPos;

    // 4. 暴力清空 S-Curve 緩衝區！
    // 絕對不能讓煞車前的餘速留在 Buffer 裡，否則解除急停時機台會抖一下
    if (axis.velBuffer.size() > 0) {
        std::fill(axis.velBuffer.begin(), axis.velBuffer.end(), 0.0);
        axis.bufferSum = 0.0;
        axis.bufferIndex = 0; // 🌟 [新增] 緩衝區指針也要歸零，確保下次從頭開始填寫
    }

    // RtPrintf(">>> [ALARM] Axis E-STOP Triggered! Velocity Killed Instantly.\n");
}
void MotionCore::EmergencyStopAllAxes()
{
    EmergencyStopAllAxesImpl(false);
}

void MotionCore::EmergencyStopAllAxesImpl(
    bool forceExecutionInvalidation) noexcept
{
    // =========================================================
    // 1. 使目前執行世代失效
    //
    // Emergency 訊號可能每個 Scan 都持續呼叫本函式。只有仍有
    // Active / Queued Motion 時才切換 Epoch，避免空機時無限遞增。
    // =========================================================
    const MotionExecutionEpoch executionEpochBeforeStop =
        GetCurrentExecutionEpoch();
    MotionCommand queuedCommand{};
    // Alarm blocks NC dispatch and the queue is FIFO. After the first
    // invalidation, a stale front therefore proves the remaining visible
    // backlog belongs to the retired Epoch; repeated E-stop requests drain it
    // with the bounded stale budget but must not publish another Epoch.
    const bool queuedExecutionCurrent =
        TryPeekNextMotionCommand(queuedCommand) &&
        queuedCommand.execution.IsAssigned() &&
        queuedCommand.execution.epoch == executionEpochBeforeStop;
    const bool hadExecutionToInvalidate =
        forceExecutionInvalidation ||
        m_Group.isActive ||
        queuedExecutionCurrent;

    // Safety 以新 Generation 搶占控制權；重複急停時保持同一份 Lease。
    // Capture the current-Epoch work first: after this takeover an AUTO
    // command is intentionally owner-conflicted and cannot prove whether the
    // stop still needs one (and only one) Epoch invalidation.
    TakeSafetyMotionOwner();

    MotionExecutionEpoch executionEpochAfterStop =
        executionEpochBeforeStop;

    if (hadExecutionToInvalidate)
    {
        executionEpochAfterStop = BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
        ++m_emergencyStopEpochInvalidationCount;
    }

    // 防止 UpdateInterpolation 繼續對實體軸寫入新的命令。
    m_Group.isActive = false;


    // =========================================================
    // 2. 急停虛擬主軸
    //
    // 即使真正需要停的是所有實體軸，
    // interpolation virtual axis 也必須一起停止。
    // =========================================================
    EmergencyStop(m_Group.virtualAxis);

    ++m_emergencyStopRTApplicationCount;
    m_emergencyStopLastAppliedExecutionEpoch =
        GetCurrentExecutionEpoch();
    m_emergencyStopLastAppliedOwnerLease =
        GetMotionOwnerLease();
    m_emergencyStopLastApplyHadExecutionToInvalidate =
        hadExecutionToInvalidate;

    // Stage NC-0.2J.6.3.2: preserve the last application that *really*
    // invalidated execution.  Level-sensitive C5/axis-protection inputs can
    // apply E-stop again after the group is already stopped; those no-op
    // applications must not overwrite the only exact old-Epoch -> new-Epoch
    // correlation available to the later 10 ms J.6 observer.
    if (hadExecutionToInvalidate)
    {
        const std::uint64_t packedEpochInvalidation =
            static_cast<std::uint64_t>(executionEpochBeforeStop) |
            (static_cast<std::uint64_t>(executionEpochAfterStop) << 32U);

        // Odd = writer active, even = stable.  The separate seqlock preserves
        // an exact pair across two 64-bit atomics while keeping the main RT
        // stop/settle payload at its original 192-word ceiling.
        m_emergencyStopEpochInvalidationWriteSequence.fetch_add(
            1ULL,
            std::memory_order_acq_rel);
        m_emergencyStopLastEpochInvalidationPacked.store(
            packedEpochInvalidation,
            std::memory_order_relaxed);
        m_emergencyStopLastEpochInvalidationCount.store(
            m_emergencyStopEpochInvalidationCount,
            std::memory_order_relaxed);
        m_emergencyStopEpochInvalidationWriteSequence.fetch_add(
            1ULL,
            std::memory_order_release);
    }


    // =========================================================
    // 3. 尚未執行的命令已由新 Epoch 取消。
    //
    // 固定 SPSC Ring 只允許 250 us Consumer 移除命令；
    // ApplyPendingExecutionEpochChange() 會淘汰舊世代。
    // =========================================================


    // =========================================================
    // 4. Context 尚未 Link 時直接離開
    // =========================================================
    if (m_pContexts == nullptr)
    {
        return;
    }


    // =========================================================
    // 5. 掃描所有實體 AxisContext
    //
    // 不使用：
    //
    //     m_Group.axisCount
    //
    // 因為 C5 是「全機 Emergency」，
    // 不管該軸目前是否參與 G-code interpolation，
    // 只要 axis.isExist == true 就必須停止。
    // =========================================================
    for (size_t i = 0;
        i < m_pContexts->size();
        ++i)
    {
        AxisContext& axis =
            (*m_pContexts)[i];

        if (!axis.isExist)
        {
            continue;
        }

        EmergencyStop(axis);
    }
}
void MotionCore::SetAxisFeedrateOverride(int axisIndex, double overrideRatio)
{
    (*m_pContexts)[axisIndex].feedrateOverride = overrideRatio;
}
void MotionCore::Stop(AxisContext& axis)
{
    // 簡易急停：將目標設為當前規劃位置，讓速度歸零
    axis.finalTargetPos = axis.currentCmdPos;
    axis.state = MotionState::MotionState_STOPPING;
}

void MotionCore::ResetFault(AxisContext& axis)
{
    // NC-0.2J.6.4: a command that started before the permanent Lag monitor
    // armed is a boot sequencing violation, not an operator-resettable Alarm.
    // Keep the original Cmd/Act evidence and require a controlled restart.
    if (axis.startupLagPrematureMotionBlocked &&
        !axis.startupLagMonitorArmed)
    {
        axis.resetRequest = false;
        axis.isFault = true;
        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.targetVelocity = 0.0;
        axis.targetEndVel = 0.0;
        axis.inPosition = false;
        axis.state = MotionState::MotionState_ERROR;
        return;
    }

    // 🌟 1. 先把「有沒有發生過嚴重脫節」的狀態記下來
    // 只有這些情況，大腦才需要放棄尊嚴，去跟實體座標對齊
    bool needPhysicalSnap = axis.isFault ||
        axis.isLagAlarm ||
        (axis.state == MotionState::MotionState_ESTOP);

    // 2. 清除所有異常旗標與 PID 歷史
    axis.resetRequest = true;
    axis.isFault = false;
    axis.isLagAlarm = false;
    axis.pid.integralAcc = 0.0;
    axis.pid.prevError = 0.0;
    axis.state = MotionState::MotionState_IDLE;

    // =========================================================
    // 🌟 3. [神級修復] 條件式座標對齊 (消滅無謂的座標飄移)
    // =========================================================
    if (axis.isVirtualAxis)
    {
        // 【虛擬軸】：它是大腦的幽靈，沒有實體。直接清空進度就好。
        axis.currentCmdPos = 0.0;
        axis.logicalCmdPos = 0.0;
        axis.planningPos = 0.0;
        axis.currentCmdVel = 0.0;
    }
    else if (needPhysicalSnap)
    {
        // 【嚴重脫節的實體軸】：大腦必須向實體馬達低頭，重新對齊起跑線
        // 否則下次一 Servo On，就會瞬間衝向原本的 CmdPos 造成撞機！
        axis.currentCmdPos = axis.currentActPos;
        axis.logicalCmdPos = axis.currentActPos;
        axis.planningPos = axis.currentActPos;
        axis.currentCmdVel = 0.0;
    }
    // else 
    // {
    //      【健康的實體軸】：例如只是普通 NC Reset，沒有報警。
    //      🌟 絕對不要動大腦座標！保留最完美的數學理論精度！
    // }
}
void MotionCore::ResetAllFaults()//全軸 清除異常狀態
{
    ResetAllFaultsImpl(true);
}

void MotionCore::ResetAllFaultsImpl(
    bool publishExecutionEpoch)
{
    if (m_pContexts == nullptr) return;

    for (size_t i = 0; i < m_pContexts->size(); ++i) {
        ResetFault((*m_pContexts)[i]);
    }

    const bool hadExecutionToInvalidate =
        m_Group.isActive ||
        !m_Group.cmdQueue.empty();

    if (publishExecutionEpoch && hadExecutionToInvalidate)
    {
        BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
    }

    m_Group.isActive = false;
    // Future Command 由新 Epoch 在 250 us Consumer 端淘汰。
    ResetFault(m_Group.virtualAxis); // 👈 現在有了 isVirtualAxis 保護，這裡也安全了！

    if (m_Group.jumpManager.state != JumpState::IDLE) {
        m_Group.jumpManager.state = JumpState::IDLE;
        m_Group.jumpManager.currentOffset = 0.0;
        m_Group.jumpManager.jumpVel = 0.0;
    }
}
void MotionCore::Calc_Trajectory_Trapezoidal(
    AxisContext& axis,
    AxisCommand& outCmd)
{
    // =========================================================
    // 0. 不需要規劃的狀態
    // =========================================================
    if (axis.state == MotionState::MotionState_IDLE ||
        axis.state == MotionState::MotionState_ERROR)
    {
        axis.currentCmdVel = 0.0;

        outCmd.instantCmdPos =
            axis.currentCmdPos;

        outCmd.instantCmdVel =
            0.0;

        return;
    }


    // =========================================================
    // 1. 基本路徑資訊
    // =========================================================
    double dt = CYCLE_TIME_SEC;

    double planDistErr = axis.finalTargetPos - axis.planningPos;

    double dir = (planDistErr >= 0.0) ? 1.0 : -1.0;

    double planDist = std::abs(planDistErr);


    // 本 Cycle 依目前速度理論上能走多少距離
    double stepDist = std::abs(axis.currentCmdVel * dt);


    // 規劃器是否已經到最後一個 Cycle
    bool isPlanDone = (planDist <= stepDist) || (planDist < 0.001);


    // =========================================================
    // 2. 規劃終點處理
    // =========================================================
    if (isPlanDone)
    {
        axis.planningPos = axis.finalTargetPos;


        // -----------------------------------------------------
        // P1 / Continuous：
        // 有指定終點速度，維持交接速度
        // -----------------------------------------------------
        if (std::abs(axis.targetEndVel) > 0.1)
        {
            axis.currentCmdVel = dir * std::abs(axis.targetEndVel);
        }

        // -----------------------------------------------------
        // P0 / Exact Stop：
        // 最後一小步精準走完
        // -----------------------------------------------------
        else
        {
            axis.currentCmdVel = dir * (planDist / dt);
        }
    }


    // =========================================================
    // 3. 正常梯形速度規劃
    // =========================================================
    else
    {
        // -----------------------------------------------------
        // 最大巡航速度
        // -----------------------------------------------------
        double max_v = axis.cruiseVel_PPS;

        if (max_v < 0.0)
        {
            max_v = 0.0;
        }


        // -----------------------------------------------------
        // 減速度
        // -----------------------------------------------------
        double dec = axis.dec_PPS2;


        // -----------------------------------------------------
        // 終點速度
        // -----------------------------------------------------
        double v_end = std::abs(axis.targetEndVel);

        if (v_end > max_v)
        {
            v_end = max_v;
        }


        // -----------------------------------------------------
        // 根據剩餘距離算現在允許的最大速度
        //
        // v² = u² + 2as
        // -----------------------------------------------------
        double max_allowable_vel = std::sqrt(v_end * v_end + 2.0 * dec * planDist);


        if (max_allowable_vel > max_v)
        {
            max_allowable_vel = max_v;
        }


        // 加入運動方向
        double target_v = dir * max_allowable_vel;


        // =====================================================
        // 🌟 本次真正修改的地方
        //
        // currentCmdVel 每次加 / 減速後，
        // 絕對不允許越過 target_v。
        // =====================================================


        // =====================================================
        // 正方向
        // =====================================================
        if (dir > 0.0)
        {
            // -------------------------------------------------
            // 加速
            // -------------------------------------------------
            if (axis.currentCmdVel < target_v)
            {
                axis.currentCmdVel += axis.acc_PPS2 * dt;


                // 🌟 新增 Clamp
                // 防止 90 -> 140，而 target_v 只有 100
                if (axis.currentCmdVel > target_v)
                {
                    axis.currentCmdVel = target_v;
                }
            }


            // -------------------------------------------------
            // 減速
            // -------------------------------------------------
            else if (axis.currentCmdVel > target_v)
            {
                axis.currentCmdVel -= axis.dec_PPS2 * dt;


                // 🌟 新增 Clamp
                // 防止 140 -> 80，而 target_v 是 100
                if (axis.currentCmdVel < target_v)
                {
                    axis.currentCmdVel = target_v;
                }
            }


            // 正方向規劃不允許跑出負速度
            if (axis.currentCmdVel < 0.0)
            {
                axis.currentCmdVel = 0.0;
            }
        }


        // =====================================================
        // 負方向
        // =====================================================
        else
        {
            // target_v 此時是負數
            //
            // 例如：
            // current = -50
            // target  = -100
            //
            // 要往負方向加速


            // -------------------------------------------------
            // 負方向加速
            // -------------------------------------------------
            if (axis.currentCmdVel > target_v)
            {
                axis.currentCmdVel -= axis.acc_PPS2 * dt;


                // 🌟 新增 Clamp
                //
                // 例如：
                // target = -100
                // 算完變 -140
                //
                // 必須拉回 -100
                if (axis.currentCmdVel < target_v)
                {
                    axis.currentCmdVel = target_v;
                }
            }


            // -------------------------------------------------
            // 負方向減速
            // -------------------------------------------------
            else if (axis.currentCmdVel < target_v)
            {
                axis.currentCmdVel += axis.dec_PPS2 * dt;


                // 🌟 新增 Clamp
                //
                // 例如：
                // current = -140
                // target  = -100
                //
                // 算完變 -80
                // 必須拉回 -100
                if (axis.currentCmdVel > target_v)
                {
                    axis.currentCmdVel = target_v;
                }
            }


            // 負方向規劃不允許跑出正速度
            if (axis.currentCmdVel > 0.0)
            {
                axis.currentCmdVel = 0.0;
            }
        }


        // =====================================================
        // 更新規劃位置
        // =====================================================
        axis.planningPos += axis.currentCmdVel * dt;
    }


    // =========================================================
    // 4. S-Curve 濾波
    // =========================================================
    double finalOutputVel = axis.currentCmdVel;


    if (axis.velBuffer.size() > 1)
    {
        axis.bufferSum -= axis.velBuffer[axis.bufferIndex];
        axis.velBuffer[axis.bufferIndex] = axis.currentCmdVel;
        axis.bufferSum += axis.currentCmdVel;
        axis.bufferIndex = (axis.bufferIndex + 1) % (int)axis.velBuffer.size();
        finalOutputVel = axis.bufferSum / (double)axis.velBuffer.size();
    }


    // =========================================================
    // 5. S-Curve Output Position 積分
    // =========================================================
    double stepPos = finalOutputVel * dt;


    // ---------------------------------------------------------
    // P0 / 最後一行：
    // 才允許精準夾到終點
    // ---------------------------------------------------------
    bool isStopping = (std::abs(axis.targetEndVel) <= 0.1) || (axis.isVirtualAxis && m_Group.cmdQueue.empty());


    // =========================================================
    // 6. 正方向終點 Clamp
    // =========================================================
    if (isStopping && dir > 0.0 && (axis.currentCmdPos + stepPos) >= axis.finalTargetPos)
    {
        stepPos = axis.finalTargetPos - axis.currentCmdPos;

        finalOutputVel = stepPos / dt;


        axis.currentCmdPos = axis.finalTargetPos;


        // 清空 S-Curve Buffer
        for (size_t i = 0; i < axis.velBuffer.size(); ++i)
        {
            axis.velBuffer[i] = 0.0;
        }
        axis.bufferSum = 0.0;
    }


    // =========================================================
    // 7. 負方向終點 Clamp
    // =========================================================
    else if (isStopping && dir < 0.0 && (axis.currentCmdPos + stepPos) <= axis.finalTargetPos)
    {
        stepPos = axis.finalTargetPos - axis.currentCmdPos;


        finalOutputVel = stepPos / dt;


        axis.currentCmdPos = axis.finalTargetPos;


        // 清空 S-Curve Buffer
        for (size_t i = 0; i < axis.velBuffer.size(); ++i)
        {
            axis.velBuffer[i] = 0.0;
        }

        axis.bufferSum =
            0.0;
    }


    // =========================================================
    // 8. 正常積分
    // =========================================================
    else
    {
        axis.currentCmdPos += stepPos;
    }


    // =========================================================
    // 9. 輸出 Command
    // =========================================================
    outCmd.instantCmdVel = finalOutputVel;

    outCmd.instantCmdPos = axis.currentCmdPos;


    // =========================================================
    // 10. 到位 / P1 交接判斷
    // =========================================================
    bool isBufferDry = (std::abs(finalOutputVel) < 1.0);
    bool isHandoverReady = false;


    // ---------------------------------------------------------
    // P0
    // ---------------------------------------------------------
    if (isStopping)
    {
        isHandoverReady = (isPlanDone && isBufferDry);
    }


    // ---------------------------------------------------------
    // P1
    // ---------------------------------------------------------
    else
    {
        if (dir > 0.0)
        {
            isHandoverReady = (axis.currentCmdPos >= axis.finalTargetPos);
        }
        else
        {
            isHandoverReady = (axis.currentCmdPos <= axis.finalTargetPos);
        }
    }


    // =========================================================
    // 11. 實體軸到位確認
    // Virtual Axis 不檢查 Physical Lag
    // =========================================================
    bool isPhysicalInPos = true;


    if (!axis.isVirtualAxis)
    {
        double currentLag = std::abs(axis.currentCmdPos - axis.currentActPos);
        isPhysicalInPos = (currentLag <= axis.inPositionWindow_Pulse);
    }


    // =========================================================
    // 12. 正式完成 / 交接
    // =========================================================
    if (isHandoverReady && isPhysicalInPos && axis.state == MotionState::MotionState_MOVING)
    {
        if (isStopping)
        {
            axis.state = MotionState::MotionState_IDLE;

            // Exact Stop is complete only when the whole virtual command state
            // is canonical IDLE.  finalOutputVel can already be zero after the
            // endpoint clamp while the raw planner scalar still contains the
            // former cruise speed; leaving that value alive permanently blocks
            // the NC-0.2J.5 Reset pre-proof once Group becomes inactive.
            if (TryCanonicalizeIdleAxisCommandState(axis))
            {
                outCmd.instantCmdPos = axis.currentCmdPos;
                outCmd.instantCmdVel = 0.0;
            }
        }

        // P0 = 完成
        // P1 = 可以交下一段
        axis.inPosition = true;
    }
}

// =========================================================
// MPG Trajectory Planner
//
// Dynamic Position Following
//
// 特性：
//
// 1. finalTargetPos 可隨時更新
// 2. 不重新初始化軌跡
// 3. 根據剩餘距離自動算煞車速度
// 4. 手輪反轉時先減速到 0，再反向
// 5. 到達 Target 後仍保持 MPG State
//
// 因為 C21 還可能保持 ON，下一個 Handwheel Count
// 隨時可能再更新 Target。
// =========================================================

void MotionCore::Calc_Trajectory_MPG(
    AxisContext& axis,
    AxisCommand& outCmd)
{
    // =====================================================
    // Guard
    // =====================================================

    if (axis.state !=
        MotionState::MotionState_MPG)
    {
        outCmd.instantCmdPos =
            axis.currentCmdPos;

        outCmd.instantCmdVel =
            0.0;

        return;
    }


    const double dt =
        CYCLE_TIME_SEC;


    // =====================================================
    // Parameter
    // =====================================================

    double maxVel =
        std::abs(
            axis.cruiseVel_PPS);


    if (axis.maxVel_PPS > 0.0 &&
        maxVel > axis.maxVel_PPS)
    {
        maxVel =
            axis.maxVel_PPS;
    }


    double acc =
        axis.acc_PPS2;


    double dec =
        axis.dec_PPS2;


    if (acc <= 0.0)
    {
        acc =
            1.0;
    }


    if (dec <= 0.0)
    {
        dec =
            acc;
    }


    // =====================================================
    // Position Error
    // =====================================================

    const double error =
        axis.finalTargetPos -
        axis.currentCmdPos;


    const double distance =
        std::abs(error);


    constexpr double POSITION_TOLERANCE =
        0.01;


    // =====================================================
    // Dynamic Target Velocity
    //
    // v = sqrt(2as)
    //
    // 距離越靠近 Target，
    // 允許速度自然下降。
    // =====================================================

    double desiredVelocity =
        0.0;


    if (distance >
        POSITION_TOLERANCE &&
        maxVel > 0.0)
    {
        double brakingVelocity =
            std::sqrt(
                2.0 *
                dec *
                distance);


        if (brakingVelocity >
            maxVel)
        {
            brakingVelocity =
                maxVel;
        }


        if (error > 0.0)
        {
            desiredVelocity =
                brakingVelocity;
        }
        else
        {
            desiredVelocity =
                -brakingVelocity;
        }


        axis.inPosition =
            false;
    }


    axis.targetVelocity =
        desiredVelocity;


    // =====================================================
    // Velocity Ramp Helper
    // =====================================================

    auto MoveToward =
        [](
            double current,
            double target,
            double step)
    {
        if (step <= 0.0)
        {
            return target;
        }


        if (current < target)
        {
            current += step;

            if (current > target)
            {
                current =
                    target;
            }
        }
        else if (current > target)
        {
            current -= step;

            if (current < target)
            {
                current =
                    target;
            }
        }


        return current;
    };


    // =====================================================
    // Direction Reversal
    //
    // 正在往 +，但 Handwheel 已經要求 -
    //
    // 或
    //
    // 正在往 -，但 Handwheel 已經要求 +
    //
    // 必須先減速至 0。
    // 不允許瞬間反向。
    // =====================================================

    const bool reversing =
        (
            axis.currentCmdVel > 0.0 &&
            desiredVelocity < 0.0
            )
        ||
        (
            axis.currentCmdVel < 0.0 &&
            desiredVelocity > 0.0
            );


    if (reversing)
    {
        axis.currentCmdVel =
            MoveToward(
                axis.currentCmdVel,
                0.0,
                dec * dt);
    }
    else
    {
        // =================================================
        // Speed Increasing
        // =================================================

        if (std::abs(desiredVelocity) >
            std::abs(axis.currentCmdVel))
        {
            axis.currentCmdVel =
                MoveToward(
                    axis.currentCmdVel,
                    desiredVelocity,
                    acc * dt);
        }

        // =================================================
        // Speed Decreasing
        // =================================================

        else
        {
            axis.currentCmdVel =
                MoveToward(
                    axis.currentCmdVel,
                    desiredVelocity,
                    dec * dt);
        }
    }


    // =====================================================
    // S-Curve Moving Average
    // =====================================================

    double finalOutputVel =
        axis.currentCmdVel;


    if (axis.velBuffer.size() > 1)
    {
        axis.bufferSum -=
            axis.velBuffer[
                axis.bufferIndex];

        axis.velBuffer[
            axis.bufferIndex] =
            axis.currentCmdVel;

            axis.bufferSum +=
                axis.currentCmdVel;

            axis.bufferIndex =
                (
                    axis.bufferIndex +
                    1
                    )
                %
                static_cast<int>(
                    axis.velBuffer.size());


            finalOutputVel =
                axis.bufferSum /
                static_cast<double>(
                    axis.velBuffer.size());
    }


    // =====================================================
    // Position Integration
    // =====================================================

    const double nextPosition =
        axis.currentCmdPos +
        finalOutputVel *
        dt;


    // =====================================================
    // Target Crossing Clamp
    //
    // 避免因最後一點 S-Curve 餘速穿過目標。
    // =====================================================

    const bool crossPositive =
        error > 0.0 &&
        finalOutputVel > 0.0 &&
        nextPosition >=
        axis.finalTargetPos;


    const bool crossNegative =
        error < 0.0 &&
        finalOutputVel < 0.0 &&
        nextPosition <=
        axis.finalTargetPos;


    if (crossPositive ||
        crossNegative)
    {
        axis.currentCmdPos =
            axis.finalTargetPos;

        axis.planningPos =
            axis.finalTargetPos;

        axis.currentCmdVel =
            0.0;

        axis.targetVelocity =
            0.0;


        // ---------------------------------------------
        // 到 Target 時 S-Curve 已沒有繼續追趕的意義。
        // 清除剩餘速度，避免越過後又反拉。
        // ---------------------------------------------

        for (size_t i = 0;
            i < axis.velBuffer.size();
            ++i)
        {
            axis.velBuffer[i] =
                0.0;
        }


        axis.bufferSum =
            0.0;

        axis.bufferIndex =
            0;


        finalOutputVel =
            0.0;

        axis.inPosition =
            true;
    }
    else
    {
        axis.currentCmdPos =
            nextPosition;

        axis.planningPos =
            axis.currentCmdPos;


        // =================================================
        // Completely Arrived
        // =================================================

        const double remaining =
            std::abs(
                axis.finalTargetPos -
                axis.currentCmdPos);


        if (remaining <=
            POSITION_TOLERANCE &&
            std::abs(
                axis.currentCmdVel) <
            0.1 &&
            std::abs(
                finalOutputVel) <
            0.1)
        {
            axis.currentCmdPos =
                axis.finalTargetPos;

            axis.planningPos =
                axis.finalTargetPos;

            axis.currentCmdVel =
                0.0;

            axis.targetVelocity =
                0.0;

            finalOutputVel =
                0.0;

            axis.inPosition =
                true;
        }
        else
        {
            axis.inPosition =
                false;
        }
    }


    // =====================================================
    // Output
    //
    // 注意：
    // 到位後仍保持 MotionState_MPG。
    //
    // 下一個 DR200 Count 一來，
    // MPGMove() 只更新 finalTargetPos，
    // 馬上可以繼續追。
    // =====================================================

    outCmd.instantCmdPos =
        axis.currentCmdPos;

    outCmd.instantCmdVel =
        finalOutputVel;
}
void MotionCore::Calc_Trajectory_Velocity(AxisContext& axis, AxisCommand& outCmd)
{
    if (axis.state == MotionState::MotionState_IDLE || axis.state == MotionState::MotionState_ERROR) {
        axis.currentCmdVel = 0.0;
        outCmd.instantCmdPos = axis.currentCmdPos;
        outCmd.instantCmdVel = 0.0;
        return;
    }



    double dt = CYCLE_TIME_SEC;

    // =========================================================
    // 1. [大腦規劃層] 變速與煞車邏輯
    // =========================================================
    if (axis.state == MotionState::MotionState_STOPPING)
    {
        // [修正 1] 移除魔法數字 0.1，利用精準的數學運算降至 0
        if (axis.currentCmdVel > 0.0) {
            axis.currentCmdVel -= axis.dec_PPS2 * dt;
            if (axis.currentCmdVel < 0.0) axis.currentCmdVel = 0.0; // 完美截斷
        }
        else if (axis.currentCmdVel < 0.0) {
            axis.currentCmdVel += axis.dec_PPS2 * dt;
            if (axis.currentCmdVel > 0.0) axis.currentCmdVel = 0.0; // 完美截斷
        }


    }
    else // MotionState::MotionState_MOVING
    {
        double targetVel = axis.targetVelocity;

        // 限制最大速度
        if (targetVel > axis.maxVel_PPS) targetVel = axis.maxVel_PPS;
        if (targetVel < -axis.maxVel_PPS) targetVel = -axis.maxVel_PPS;

        // [修正 3] 動態判斷現在是「變快」還是「變慢」，決定要用 Acc 還是 Dec
        double activeSlope;
        if (std::abs(targetVel) > std::abs(axis.currentCmdVel)) {
            // 速度變快 -> 使用加速度
            activeSlope = (axis.VelocityMove_Acc > 0) ? axis.VelocityMove_Acc : axis.acc_PPS2;
        }
        else {
            // 速度變慢 -> 使用減速度 (如果沒設定，才退回使用加速度)
            activeSlope = (axis.dec_PPS2 > 0) ? axis.dec_PPS2 : axis.acc_PPS2;
        }

        // 斜坡追隨
        if (axis.currentCmdVel < targetVel) {
            axis.currentCmdVel += activeSlope * dt;
            if (axis.currentCmdVel > targetVel) axis.currentCmdVel = targetVel;
        }
        else if (axis.currentCmdVel > targetVel) {
            axis.currentCmdVel -= activeSlope * dt;
            if (axis.currentCmdVel < targetVel) axis.currentCmdVel = targetVel;
        }
    }

    // =========================================================
    // 2. [Layer B] S-Curve 濾波
    // =========================================================
    double finalOutputVel = axis.currentCmdVel;

    if (axis.velBuffer.size() > 1) {
        axis.bufferSum -= axis.velBuffer[axis.bufferIndex];
        axis.velBuffer[axis.bufferIndex] = axis.currentCmdVel;
        axis.bufferSum += axis.currentCmdVel;
        axis.bufferIndex = (axis.bufferIndex + 1) % (int)axis.velBuffer.size();

        finalOutputVel = axis.bufferSum / (double)axis.velBuffer.size();
    }

    // =========================================================
    // 3. [Layer C] 積分與同步
    // =========================================================
    axis.planningPos += finalOutputVel * dt;
    axis.currentCmdPos += finalOutputVel * dt;

    outCmd.instantCmdVel = finalOutputVel;
    outCmd.instantCmdPos = axis.currentCmdPos;

    // =========================================================
    // 4. [完全停止判斷] 
    // =========================================================
    // [修正 2] 解決卡死 BUG：如果是 STOPPING，或「正在移動但目標速度被設為 0」
    bool isStoppingState = (axis.state == MotionState::MotionState_STOPPING);
    bool isTargetZero = (axis.state == MotionState::MotionState_MOVING && std::abs(axis.targetVelocity) < 0.001);

    if (isStoppingState || isTargetZero) {
        // 條件：大腦速度歸零，且 Buffer 內的餘速也流乾
        if (std::abs(axis.currentCmdVel) < 0.001 && std::abs(finalOutputVel) < 0.1) {

            axis.state = MotionState::MotionState_IDLE;
            axis.inPosition = true;
            axis.targetVelocity = 0.0; // 確保目標也清空

            // RtPrintf(">>> [IDLE] VelocityMove / Stop Complete.\n");
        }
    }
}







void MotionCore::DetermineActiveGainSet(AxisContext& axis)// PID 依照狀態切換
{
    // =========================================================
    // G81 HOME Dedicated Gain
    //
    // MotionState 只表示「軸現在怎麼動」：
    //
    // VELOCITY
    // STOPPING
    // MOVING
    //
    // HomeState 才表示「這次運動屬於 HOME 的哪一階段」。
    //
    // 因此 HOME 專用 Gain 必須比一般 MotionState Gain
    // 更高優先權，否則 SEARCH_SWITCH 使用 VelocityMove() 時，
    // 會被下面的一般規則套成 Pid_G00。
    // =========================================================

    if (axis.homeRuntime.active)
    {
        switch (axis.homeRuntime.state)
        {
            // -----------------------------------------------------
            // HOME Search Gain
            //
            // 尋找 DOG / LIMIT、碰到後滑行停止，以及 Backoff，
            // 都使用 HOME Search 專用增益。
            // -----------------------------------------------------

        case HomeState::SEARCH_SWITCH:
        case HomeState::SWITCH_DECEL_STOP:
        case HomeState::BACK_OFF:

            axis.pid.Kp =
                axis.home.searchGain.Kp;

            axis.pid.Ki =
                axis.home.searchGain.Ki;

            axis.pid.Kd =
                axis.home.searchGain.Kd;

            axis.pid.Kvff =
                axis.home.searchGain.Kvff;

            return;


            // -----------------------------------------------------
            // HOME INDEX Gain
            //
            // 搜尋 INDEX 與 INDEX 捕捉後的滑行停止，
            // 使用 HOME INDEX 專用增益。
            // -----------------------------------------------------

        case HomeState::SEARCH_INDEX:
        case HomeState::INDEX_CAPTURED:
        case HomeState::INDEX_DECEL_STOP:

            axis.pid.Kp =
                axis.home.indexGain.Kp;

            axis.pid.Ki =
                axis.home.indexGain.Ki;

            axis.pid.Kd =
                axis.home.indexGain.Kd;

            axis.pid.Kvff =
                axis.home.indexGain.Kvff;

            return;


            // -----------------------------------------------------
            // 其他 HOME State
            //
            // PREPARE / WAIT / APPLY_HOME：
            //     軸通常是 IDLE，沿用 Pid_IDLE。
            //
            // MOVE_TO_ZERO：
            //     後續使用一般 P2P Move，沿用 Pid_G00。
            // -----------------------------------------------------

        default:
            break;
        }
    }


    // =========================================================
    // Existing General Motion Gain
    // =========================================================

    switch (axis.state)
    {
    case MotionState::MotionState_IDLE:
    case MotionState::MotionState_ERROR:
    case MotionState::MotionState_ESTOP:
    default:

        axis.pid.Kp =
            axis.Pid_IDLE.Kp;

        axis.pid.Ki =
            axis.Pid_IDLE.Ki;

        axis.pid.Kd =
            axis.Pid_IDLE.Kd;

        axis.pid.Kvff =
            axis.Pid_IDLE.Kvff;

        break;


    case MotionState::MotionState_MOVING:
    case MotionState::MotionState_INTERPOLATING:
    case MotionState::MotionState_STOPPING:
    case MotionState::MotionState_VELOCITY:
    case MotionState::MotionState_MPG:

        axis.pid.Kp =
            axis.Pid_G00.Kp;

        axis.pid.Ki =
            axis.Pid_G00.Ki;

        axis.pid.Kd =
            axis.Pid_G00.Kd;

        axis.pid.Kvff =
            axis.Pid_G00.Kvff;

        break;
    }
}

// ==========================================
// [Layer 2] 伺服迴路 (Servo Loop)
// ==========================================
template <typename DriveType>
void MotionCore::Run_Servo_Loop(DriveType& servo, AxisContext& axis, const AxisCommand& cmd)
{

    DetermineActiveGainSet(axis); // PID 依照狀態切換

        // =========================================================
        // 🌟 1. [Feedback Selection] 雙回授處理
        // ⚠️ 絕對不要在這裡重新讀取 servo.pInput->ActualPosition！
        // 因為 UpdateMotion 已經算好「不會溢位、完美展開」的 axis.currentActPos 了！
        // =========================================================

    if (axis.fbMode == FeedbackSource::LINEAR_SCALE && axis.pScaleActualPos != nullptr)
    {
        // 全閉迴路：讀取光學尺並換算
        double rawScalePos = (double)(*axis.pScaleActualPos);
        double convertedScalePos = rawScalePos * axis.scaleToMotorRatio;

        // [Safety] 斷帶保護 (Slip Detection)
        // currentActPos 已扣除 Machine Offset，比對 Raw Scale 前先加回。
        const double motorRawLogicalPos = axis.currentActPos + axis.machineCoordinateOffsetPulse;
        if (std::abs(motorRawLogicalPos - convertedScalePos) > axis.maxDeviation) {
            axis.isFault = true;
            WriteServoTargetVelocityCommand(
                servo.pOutput,
                axis.axisIndex,
                0);
            RtPrintf("ALARM: Dual Loop Deviation Error!\n");
            return;
        }

        // 檢查完畢後，套用 HOME Machine Coordinate Offset。
        axis.currentActPos = convertedScalePos - axis.machineCoordinateOffsetPulse;
    }
    // =========================================================
    // ⚠️ 原本的 else { axis.currentActPos = rawMotorPos; } 已經整塊刪除！
    // 如果是半閉迴路 (馬達編碼器)，直接沿用 UpdateMotion 傳進來的完美 axis.currentActPos！
    // =========================================================


      // =========================================================
    // Software Travel Limit - Runtime State Update
    //
    // 必須放在 Feedback Selection 完成之後，
    // 確保使用的是本 Cycle 最終有效的 Actual Position。
    //
    // !isHomed 時 CoordinateManager 會自動 bypass，
    // 所以尚未尋原點時 Software Limit 不生效。
    //
    // Physical +OT / -OT 不在這裡處理。
    // =========================================================

    if (m_pCoordMgr != nullptr)
    {
        m_pCoordMgr->UpdateSoftwareTravelLimitState(
            axis);
    }


    // =========================================================
    // Automatic Motion - Runtime Software Overtravel
    // =========================================================

    if (m_pCoordMgr != nullptr && m_Group.isActive && axis.state == MotionState::MotionState_INTERPOLATING && axis.resolution_PPR > 0.0)
    {
        const double actualMCS = axis.currentActPos * (axis.finalLead / axis.resolution_PPR);

        const bool actualWithinSoftwareLimit = m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, actualMCS);
        if (!actualWithinSoftwareLimit)
        {
            if (!AlarmManager::GetInstance().HasAlarm())
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::OVER_TRAVEL, 0, axis.axisIndex);
                EmergencyStopGroup();
            }

            return;
        }
    }


    // ==========================================
    // 🌟 [新增] 實體速度差分計算 (ActVel)
    // 速度 = (現在位置 - 上次位置) / 週期時間
    // ==========================================
    axis.currentActVel = (axis.currentActPos - axis.lastActPos) / CYCLE_TIME_SEC;
    axis.lastActPos = axis.currentActPos; // 記錄本次位置，供下 1ms 使用

        // =========================================================
    // Software Travel Limit Runtime Update
    //
    // 這裡已經完成 Feedback Selection：
    //
    // Motor Encoder
    // 或
    // Linear Scale
    //
    // 所以此時的 currentActPos 才是本 Cycle
    // 真正採用的 Machine Position。
    //
    // Software Limit 1 / 2 / 3
    // 全部由 CoordinateManager 統一判斷。
    // =========================================================

    if (m_pCoordMgr != nullptr)
    {
        m_pCoordMgr->UpdateSoftwareTravelLimitState(axis);
    }


    // 2. [Lag Monitor] 跟隨誤差檢查 (此時的 ActPos 絕對不會溢位)
    double error = cmd.instantCmdPos - axis.currentActPos;

    if (axis.pid.EnableLagCheck == true)
    {

        if (std::abs(error) > axis.pid.MaxLag)
        {
            axis.isFault = true;
            axis.isLagAlarm = true;
            WriteServoTargetVelocityCommand(
                servo.pOutput,
                axis.axisIndex,
                0);
            axis.state = MotionState::MotionState_ERROR;

            // 🌟 修正：不要再轉 (int) 了，直接印 double
            RtPrintf("ALARM! Lag:%d | Cmd:%d | Act:%d\n", (int)error, (int)cmd.instantCmdPos, (int)axis.currentActPos);
            return;
        }



    }


    // 3. [PID Calculation]
    // P term
    double p_term = error * axis.pid.Kp;

    // I term
    axis.pid.integralAcc += (error * CYCLE_TIME_SEC);
    // Anti-windup
    if (axis.pid.integralAcc > axis.pid.MaxIntegral) axis.pid.integralAcc = axis.pid.MaxIntegral;
    if (axis.pid.integralAcc < -axis.pid.MaxIntegral) axis.pid.integralAcc = -axis.pid.MaxIntegral;
    double i_term = axis.pid.integralAcc * axis.pid.Ki;

    // D term (CSV Mode usually 0)
    double d_term = 0.0;

    // 4. [Feedforward] 前饋控制 (關鍵！)
    // 最終輸出 = 理論速度(VFF) + PID修正量
  // 🌟 [修改這行] 將理論速度乘上 Kvff 增益
    double finalVel = (cmd.instantCmdVel * axis.pid.Kvff) + (p_term + i_term + d_term);









    // =======================================================
    // 🌟 [修正] 狀態切換瞬間快照 (解決 %f 與洗頻問題)
    // =======================================================
    static int last_state_log[8] = { -1 };
    int idx = axis.axisIndex;

    if (idx >= 0 && idx < 8)
    {
        int currentState = (int)axis.state;

        // 條件：如果「上一次不是 IDLE」，但「這一次變成 IDLE」了 -> 抓捕切換的這 1 毫秒！
        if (last_state_log[idx] != (int)MotionState::MotionState_IDLE &&
            currentState == (int)MotionState::MotionState_IDLE)
        {
            // 🌟 RTX64 解法：數值全部強制轉型為 (int)，改用 %d 輸出
            /*
            RtPrintf(">>> [IDLE_SNAP] Ax:%d Kp:%d | Err:%d | P_Term:%d | Vff:%d | FinalV:%d\n",
                idx,
                (int)axis.pid.Kp,
                (int)error,
                (int)p_term,
                (int)(cmd.instantCmdVel * axis.pid.Kvff),
                (int)finalVel);*/
        }

        last_state_log[idx] = currentState;
    }









    // 5. [Output Clamp] 輸出限速保護
    // 限制不超過最大速度的 1.2 倍
    double limit = axis.maxVel_PPS * 1.2;
    if (finalVel > limit) finalVel = limit;
    if (finalVel < -limit) finalVel = -limit;


    // =========================================================
// Software Travel Limit - 250us Fast Guard
//
// 最後一道 Runtime Protection。
//
// 注意：
//
// 這裡的 finalVel 還沒有經過：
//
//     axis.isReverse
//     axis.Axis_Reverse
//
// 所以 finalVel 正負號仍然代表「邏輯 Machine Direction」：
//
//     finalVel > 0  = Machine +
//     finalVel < 0  = Machine -
//
// 正好可以直接與 Software Travel Limit 的
// Positive / Negative Direction Permission 比較。
// =========================================================

    if (m_pCoordMgr != nullptr)
    {
        const bool softwarePositiveAllowed =
            m_pCoordMgr->CanMoveSoftwarePositive(axis);

        const bool softwareNegativeAllowed =
            m_pCoordMgr->CanMoveSoftwareNegative(axis);


        const bool blockedPositiveMotion =
            finalVel > 0.0 &&
            !softwarePositiveAllowed;

        const bool blockedNegativeMotion =
            finalVel < 0.0 &&
            !softwareNegativeAllowed;


        if (blockedPositiveMotion ||
            blockedNegativeMotion)
        {
            // =================================================
            // A. Interpolation Group
            //
            // 多軸插補不能只停其中一軸。
            //
            // 若 Runtime Guard 已經真的抓到越界方向，
            // 代表前面的 Target Pre-Check 沒有攔住，
            // 此時屬於最後一道 Backstop。
            //
            // 必須整組停止。
            // =================================================

            if (m_Group.isActive &&
                axis.state ==
                MotionState::MotionState_INTERPOLATING)
            {
                EmergencyStopGroup();

                WriteServoTargetVelocityCommand(
                    servo.pOutput,
                    axis.axisIndex,
                    0);

                return;
            }


            // =================================================
            // B. Single Axis / Manual Motion
            //
            // Runtime Guard 已經到達 Software Boundary，
            // 這時不能再做一般減速，否則煞車距離本身
            // 還會繼續穿過 Limit。
            //
            // 所以這是「立即停止」的最後一道保護。
            // =================================================

            finalVel = 0.0;

            axis.currentCmdVel = 0.0;
            axis.logicalCmdVel = 0.0;
            axis.targetVelocity = 0.0;


            // =================================================
            // Command Position Snap
            //
            // 不可以只把 PDO Velocity 歸零，
            // 否則 Command Position 還留在 Limit 外，
            // PID Error 會持續變大，最後可能造成 Lag Alarm。
            //
            // 因此 Runtime Guard 觸發時，
            // Command / Planning / Target 全部同步到
            // 此刻實際 Machine Position。
            // =================================================

            axis.currentCmdPos =
                axis.currentActPos;

            axis.logicalCmdPos =
                axis.currentActPos;

            axis.planningPos =
                axis.currentActPos;

            axis.finalTargetPos =
                axis.currentActPos;


            // =================================================
            // Clear PID / S-Curve History
            //
            // 防止下一次反方向 Recovery 時，
            // 還帶著之前往 Limit 方向的殘留速度。
            // =================================================

            axis.pid.integralAcc = 0.0;

            for (size_t i = 0;
                i < axis.velBuffer.size();
                ++i)
            {
                axis.velBuffer[i] = 0.0;
            }

            axis.bufferSum = 0.0;
            axis.bufferIndex = 0;


            // =================================================
            // 回到 IDLE
            //
            // 不設 ERROR / ESTOP。
            //
            // Software Limit 的 Manual Recovery
            // 必須仍然允許往反方向離開。
            // =================================================

            axis.state =
                MotionState::MotionState_IDLE;

            axis.inPosition = true;


            // 本 Cycle 直接輸出 0。
            WriteServoTargetVelocityCommand(
                servo.pOutput,
                axis.axisIndex,
                0);

            return;
        }
    }

    // 🌟 輸出給硬體前，根據硬體方向翻轉速度
    if (axis.isReverse)
    {
        finalVel = -finalVel;
    }

    if (axis.Axis_Reverse)
    {
        finalVel = -finalVel;
    }

    // 6. [Write PDO] 寫入 EtherCAT
    WriteServoTargetVelocityCommand(
        servo.pOutput,
        axis.axisIndex,
        static_cast<int32_t>(
            finalVel));
}



// ==========================================
// [Core] 主更新迴圈
// ==========================================
template <typename DriveType>
void MotionCore::UpdateMotion(
    DriveType& servo,
    AxisContext& axis,
    const MotionServoInputSnapshot& input)
{

    if (!axis.isExist)
    {
        WriteServoTargetVelocityCommand(
            servo.pOutput,
            axis.axisIndex,
            0);
        return;
    }

    // =========================================================
      // 🌟 [神級修復] 絕對安全的 32-bit 展開寫法 (過濾編譯器 UB)
      // =========================================================
      // 1. 強制轉為無號整數 (uint32_t)，不管怎麼翻轉，相減絕對是正確的微小正負差
    uint32_t currentRawAct = (uint32_t)input.ActualPosition;

    if (axis.isFirstCycle) {
        axis.lastRawActPos = currentRawAct;
        axis.unwrappedActPos = (double)(int32_t)currentRawAct;
        axis.isFirstCycle = false;
    }

    // 2. 無號相減後，再強制轉回有號整數 (int32_t)，取得真實移動的 Pulse 數量
    int32_t pulseDelta = (int32_t)(currentRawAct - (uint32_t)axis.lastRawActPos);

    // 3. 記錄歷史與累加到無限大的 double 容器中
    axis.lastRawActPos = currentRawAct;
    axis.unwrappedActPos += (double)pulseDelta;


    // =========================================================
      // 🌟 核心邏輯：只針對硬體方向做翻轉，這才是 PID 和規劃器需要的「純淨邏輯座標」
      // =========================================================
    double rawLogicalActPos = axis.isReverse ? -axis.unwrappedActPos : axis.unwrappedActPos;
    if (axis.Axis_Reverse == true)
    {
        rawLogicalActPos = -rawLogicalActPos;
    }

    axis.currentActPos = rawLogicalActPos - axis.machineCoordinateOffsetPulse;

    // =========================================================

    const int opMode = input.ModesOfOperationDisplay;
    const bool rawOperationEnabled =
        (input.StatusWord & 0x006FU) == 0x0027U;

    // NC-0.2J.6.4: do not expose the constructor's Cmd=0 coordinate to the
    // runtime Lag monitor.  Eight adjacent trustworthy samples align the
    // command history first; arming is then permanent for the rest of boot.
    if (!ObserveStartupLagMonitorArming(
        axis,
        rawOperationEnabled && opMode == 9))
    {
        WriteServoTargetVelocityCommand(
            servo.pOutput,
            axis.axisIndex,
            0);
        return;
    }


    // 🟢 修改為：防抖動濾波寫法 (Debounce)
    bool rawServoOn = (input.StatusWord & 0x0027) == 0x0027;

    if (!rawServoOn) {
        axis.servoOffCounter++;
    }
    else {
        axis.servoOffCounter = 0; // 只要有讀到正常狀態就清零
    }


    // 連續 5 個 Cycle (5 毫秒) 確實沒有激磁，才認定真的斷電了
    bool isServoOn = (axis.servoOffCounter < 5);

    // 🌟 [優先權最高] 大腦監視實體馬達
    if (!isServoOn || opMode != 9) {
        if (axis.state != MotionState::MotionState_ERROR && axis.state != MotionState::MotionState_IDLE) {
            RtPrintf("[ALARM] Axis %d LOST SERVO POWER DURING MOTION!\\n", axis.axisIndex);
            axis.isFault = true; // 🚨 這裡必須觸發嚴重錯誤！
        }

        axis.currentCmdPos = axis.currentActPos;
        axis.logicalCmdPos = axis.currentActPos;
        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0;
        axis.pid.integralAcc = 0.0;
        // axis.state = MotionState::MotionState_IDLE; // ❌ 不要直接設為 IDLE
        WriteServoTargetVelocityCommand(
            servo.pOutput,
            axis.axisIndex,
            0);
        return;
    }

    // 🌟 2. 已經 Servo On 的情況下，才檢查軟體警報
    if (axis.isFault) {
        WriteServoTargetVelocityCommand(
            servo.pOutput,
            axis.axisIndex,
            0);
        return;
    }




    // ==========================================
     // 1. [Layer 3] 軌跡規劃分流 (核心修改)
     // ==========================================
    AxisCommand cmd;

    const int contextAxisSlot =
        (m_pContexts != nullptr && !m_pContexts->empty())
        ? static_cast<int>(&axis - m_pContexts->data())
        : axis.axisIndex;

    // 初始化 cmd (防呆)
    cmd.instantCmdPos = axis.currentCmdPos;
    cmd.instantCmdVel = 0.0;

    if (axis.state == MotionState::MotionState_MOVING ||
        axis.state == MotionState::MotionState_VELOCITY)
    {
        // 🟢 用「使用者下單的速度」去乘上「倍率旋鈕」
        axis.cruiseVel_PPS = axis.programmedVel_PPS * axis.feedrateOverride;

        // 安全底線：不管倍率轉多大，絕對不准超過物理機台極速
        if (axis.cruiseVel_PPS > axis.maxVel_PPS) {
            axis.cruiseVel_PPS = axis.maxVel_PPS;
        }
    }


    switch (axis.state)
    {
    case MotionState::MotionState_MOVING:
        // [模式 A] 定位模式 (Jump) -> 執行梯形 S-Curve
        Calc_Trajectory_Trapezoidal(axis, cmd);
        break;

    case MotionState::MotionState_VELOCITY:
        // [模式 B] 速度模式 (放電) -> 執行純速度規劃
        // 這裡會呼叫你剛剛新增的函式
        Calc_Trajectory_Velocity(axis, cmd);
        break;


    case MotionState::MotionState_MPG:

        Calc_Trajectory_MPG(
            axis,
            cmd);

        break;

        // =========================================================
    // [新增] 模式 C: 多軸插補
    // =========================================================
    case MotionState::MotionState_INTERPOLATING:
        // 既然 UpdateInterpolation() 已經在迴圈外面把
        // axis.currentCmdPos 和 axis.currentCmdVel 都算好並填進去了
        // 這裡我們只要負責 "傳遞" 給後面的 PID 就好

    {
        bool exactCurrentMember = false;
        if (m_Group.isActive)
        {
            const int safeGroupAxisCount =
                ClampMotionAxisCount(m_Group.axisCount);
            for (int slot = 0;
                slot < safeGroupAxisCount;
                ++slot)
            {
                if (m_Group.axisIndices[slot] == contextAxisSlot)
                {
                    exactCurrentMember = true;
                    break;
                }
            }
        }

        if (exactCurrentMember)
        {
            // Only an exact member of the current ordered mapping may
            // forward the interpolation command to the servo loop.
            cmd.instantCmdPos = axis.currentCmdPos;
            cmd.instantCmdVel = axis.currentCmdVel;
        }
        else
        {
            // Independent backstop for any future path that bypasses the
            // UpdateInterpolation pre-scan.  Stop every existing axis in
            // this same RT pass; never integrate the stale orphan speed.
            m_p1OrphanAxisContainmentCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            m_p1LastOrphanAxisIndex.store(
                axis.axisIndex,
                std::memory_order_release);
            TriggerGroupMappingIntegrityEmergencyStop(
                axis.axisIndex);
            cmd.instantCmdPos = axis.currentCmdPos;
            cmd.instantCmdVel = 0.0;
        }
    }
    break;

    case MotionState::MotionState_STOPPING:
        // STOPPING is also used by independent single-axis velocity motion.
        // Preserve that controlled-deceleration contract unless this axis is
        // an exact member of the active interpolation group.
    {
        bool exactCurrentMember = false;
        if (m_Group.isActive)
        {
            const int safeGroupAxisCount =
                ClampMotionAxisCount(m_Group.axisCount);
            for (int slot = 0;
                slot < safeGroupAxisCount;
                ++slot)
            {
                if (m_Group.axisIndices[slot] == contextAxisSlot)
                {
                    exactCurrentMember = true;
                    break;
                }
            }
        }

        if (exactCurrentMember)
        {
            cmd.instantCmdPos = axis.currentCmdPos;
            cmd.instantCmdVel = axis.currentCmdVel;
        }
        else
        {
            Calc_Trajectory_Velocity(axis, cmd);
        }
    }
    break;

    case MotionState::MotionState_ESTOP:

        axis.currentCmdVel = 0.0;
        cmd.instantCmdVel = 0.0;
        cmd.instantCmdPos = axis.currentCmdPos;
        axis.planningPos = axis.currentCmdPos;
        break;

    default:
    case MotionState::MotionState_ERROR:



        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0; // 🟢
        cmd.instantCmdVel = 0.0;
        cmd.instantCmdPos = axis.currentCmdPos;
        axis.planningPos = axis.currentCmdPos;
        axis.logicalCmdPos = axis.currentCmdPos; // 🟢 閒置時邏輯跟隨物理
        break;
    }


    // ==========================================
    // 🌟 2. [Layer 2.5] 進入補償層 (動態加入螺距與背隙誤差)
    // ==========================================
    // 將大腦算出來的純淨理論座標 (cmd.instantCmdPos) 丟進去查表
    // 引擎會自動將背隙與螺距誤差疊加上去，保護 PID 與機構
    m_CompEngine.ApplyCompensation(axis.axisIndex, axis, cmd, CYCLE_TIME_SEC);

    // ==========================================
    // 2. [Layer 2] 執行伺服控制
    // ==========================================
    // 無論是 "定位" 還是 "速度" 模式，算出來的 cmd 
    // 最後都統一進 PID 迴圈算出 TargetVelocity




    Run_Servo_Loop(servo, axis, cmd);
}



//多軸插補------------------------------------------------------------------------------------------------

// MotionCore.cpp
void MotionCore::InitVirtualAxisSmooth(int windowSize)
{
    // ==========================================
    // 1. 初始化插補群組 (m_Group) 的大腦狀態
    // ==========================================
    m_Group.isActive = false;
    m_Group.mode = InterpolationMode::LINEAR; // 預設為直線模式
    m_Group.feedrateOverride = 1.0;           // 倍率預設 100%
    m_Group.axisCount = 0;

    // ==========================================
    // 2. 初始化虛擬主軸 (Virtual Axis) 的位置與狀態
    // ==========================================
    AxisContext& vAxis = m_Group.virtualAxis;
    vAxis.isVirtualAxis = true; // 🌟 [新增] 發放免死金牌！我是幽靈，不要檢查我的物理誤差！
    vAxis.state = MotionState::MotionState_IDLE; // 確保一開機是閒置狀態
    vAxis.currentCmdPos = 0.0;
    vAxis.currentCmdVel = 0.0;
    vAxis.logicalCmdVel = 0.0;
    vAxis.targetVelocity = 0.0;
    vAxis.targetEndVel = 0.0;
    vAxis.planningPos = 0.0;
    vAxis.finalTargetPos = 0.0;
    vAxis.inPosition = true;
    vAxis.feedrateOverride = 1.0;

    // ==========================================
    // 3. 配置 S-Curve 平滑緩衝區 (保留你原本的完美邏輯)
    // ==========================================
    if (windowSize <= 1) windowSize = 1; // 至少要為 1，防止除以零

    vAxis.velBuffer.resize(windowSize);
    vAxis.bufferSum = 0.0;
    vAxis.bufferIndex = 0;
    std::fill(vAxis.velBuffer.begin(), vAxis.velBuffer.end(), 0.0);

    // RtPrintf("[DEBUG] Virtual Axis & Group Initialized. Smooth Buffer: %d\n", windowSize);
}

void MotionCore::LineMove(const std::vector<int>& axes, const std::vector<double>& targetPos, double targetVel, double acc_time, double dec_time, BufferMode mode)
{
    // Capture one producer tuple before validation. Even malformed producer
    // input must receive an identity, a terminal REJECTED notice and a formal
    // Alarm/E-stop transaction; it must never disappear before M30 accounting.
    const MotionCommandSource commandSource =
        m_pendingCommandSource.load(
            std::memory_order_acquire);
    const MotionOwnerLease commandOwnerLease =
        GetMotionOwnerLease();
    MotionExecutionEpoch commandEpoch =
        GetCurrentExecutionEpoch();

    MotionCommand invalidCommand{};
    invalidCommand.mode = InterpolationMode::LINEAR;
    invalidCommand.sourceLinePC = m_pendingSourcePC;

    if (m_pContexts == nullptr ||
        axes.empty() ||
        axes.size() > static_cast<std::size_t>(MAX_AXES) ||
        targetPos.size() != axes.size() ||
        !std::isfinite(targetVel) ||
        !std::isfinite(acc_time) ||
        !std::isfinite(dec_time))
    {
        RejectInvalidProducerMotionCommand(
            invalidCommand,
            commandEpoch,
            commandSource,
            commandOwnerLease);
        return;
    }

    std::array<bool, MAX_AXES> seenAxis{};
    for (std::size_t slot = 0U; slot < axes.size(); ++slot)
    {
        const int axisIndex = axes[slot];
        if (axisIndex < 0 ||
            axisIndex >= MAX_AXES ||
            axisIndex >= static_cast<int>(m_pContexts->size()) ||
            seenAxis[static_cast<std::size_t>(axisIndex)] ||
            !(*m_pContexts)[axisIndex].isExist ||
            !std::isfinite(targetPos[slot]))
        {
            RejectInvalidProducerMotionCommand(
                invalidCommand,
                commandEpoch,
                commandSource,
                commandOwnerLease);
            return;
        }
        seenAxis[static_cast<std::size_t>(axisIndex)] = true;
    }

    // 1. 建立一個全新的任務包裹
    MotionCommand cmd{};
    cmd.mode = InterpolationMode::LINEAR;
    cmd.axisCount = (int)axes.size();

    // 將座標與參數抄寫到包裹裡
    for (int i = 0; i < cmd.axisCount; ++i) {
        cmd.axisIndices[i] = axes[i];
        cmd.targetPos[i] = targetPos[i];
    }

    cmd.targetVel = std::abs(targetVel);
    cmd.accTime = acc_time;
    cmd.decTime = dec_time;

    // 🌟 貼上標籤！記錄這條路徑是來自哪一行 G-Code
    cmd.sourceLinePC = m_pendingSourcePC;
    cmd.sourceWCS = m_pendingSourceWCS;

    // 🌟 貼上刀具標籤！
    cmd.sourceToolLengthMode = m_pendingToolMode;
    cmd.sourceHCode = m_pendingHCode;

    cmd.sourceToolRadiusMode = m_pendingToolRadMode; // 刀徑 G 碼
    cmd.sourceDCode = m_pendingDCode;                // D 碼

    cmd.sourceIsAbsoluteMode = m_pendingIsAbsoluteMode;

    cmd.sourceG68Active = m_pendingG68Active; // 🌟 印上 G68 標籤
    cmd.sourceG68Angle = m_pendingG68Angle; // 🌟 印上角度標籤

    cmd.sourceG168Active = m_pendingG168Active; // 🌟 把 G168 狀態印在包裹上
    cmd.sourceWCode = m_pendingWCode; // 🌟 印上 W 碼標籤


    cmd.sourceG51Active = m_pendingG51Active;
    cmd.sourceScaleRatio = m_pendingScaleRatio; // 🌟 印上縮放標籤

    cmd.sourceMirrorMask = m_pendingMirrorMask; // 🌟 印上鏡像標籤

    cmd.sourceG16Active = m_pendingG16Active; // 🌟 印上極座標標籤

    cmd.sourceG162Active = m_pendingG162Active;
    cmd.sourcePlaneMode = m_pendingPlaneMode;

    // 🔍 [加入這行] 確認收到指令
    //RtPrintf("[DBG-1] LineMove Queueing! AxisCnt:%d | FirstAxis:%d | TargetPulse:%d\n",cmd.axisCount, cmd.axisIndices[0], (int)cmd.targetPos[0]);

    // 2. 判斷是「乖乖排隊」還是「緊急覆寫」？
    if (mode == BufferMode::ABORTING)
    {
        commandEpoch = RequestAbortingExecutionEpoch(
            commandSource);

        // Future Queue / History 與 Active Abort 都由 250 us Consumer
        // 在 Epoch 邊界套用；Producer 不再修改 Motion Runtime 容器。
    }

    // 3. 在最終 Epoch 確定後配置 Identity，再把包裹推入倉庫。
    AssignExecutionIdentity(
        cmd,
        commandEpoch,
        commandSource,
        commandOwnerLease);
    TryEnqueueMotionCommand(cmd);
}
// =========================================================
// Spiral / Variable Radius Arc Helpers
//
// 參數化：
//
// r(p)     = r0 + (r1 - r0) * p
// theta(p) = theta0 + totalAngle * p
// z(p)     = z0 + deltaZ * p
//
// p = 0 ~ 1
//
// 這裡計算的是真正 3D 路徑長度，包含：
// 1. 圓周方向
// 2. 半徑方向
// 3. Z 軸方向
// =========================================================
static double CalcSpiralArcLengthAtProgress(double progress, double startRadius, double endRadius, double totalAngle, double deltaZ)
{
    // -----------------------------------------------------
    // Clamp Progress
    // -----------------------------------------------------
    if (progress <= 0.0)
    {
        return 0.0;
    }

    if (progress > 1.0)
    {
        progress = 1.0;
    }


    const double deltaRadius = endRadius - startRadius;

    const double absAngle = std::abs(totalAngle);


    // =====================================================
    // Case 1：
    // 沒有角度變化
    //
    // 只剩 Radius / Z 的直線變化
    // =====================================================
    if (absAngle < 1e-12)
    {
        const double totalLinearLength = std::sqrt(deltaRadius * deltaRadius + deltaZ * deltaZ);

        return totalLinearLength * progress;
    }


    // =====================================================
    // Case 2：
    // 固定半徑
    //
    // 這就是目前標準 G02 / G03。
    //
    // 保留原本完全相同的幾何長度：
    //
    // sqrt(
    //     (R * Angle)^2 +
    //     DeltaZ^2
    // )
    // =====================================================
    if (std::abs(deltaRadius) < 1e-12)
    {
        const double planarLength = startRadius * absAngle;

        const double totalLength = std::sqrt(planarLength * planarLength + deltaZ * deltaZ);

        return totalLength * progress;
    }


    // =====================================================
    // Case 3：
    // 真正 Variable Radius Spiral
    //
    // ds/dp =
    //
    // sqrt(
    //      DeltaRadius^2
    //    + DeltaZ^2
    //    + (r * DeltaTheta)^2
    // )
    //
    // 對 p 積分得到真正路徑長度。
    // =====================================================

    const double A2 = deltaRadius * deltaRadius + deltaZ * deltaZ;

    const double A = std::sqrt(A2);

    const double B = absAngle;


    auto Primitive = [A, A2, B](double radius) -> double
    {
        const double root = std::sqrt(A2 + B * B * radius * radius);

        return    0.5 * (radius * root + (A2 / B) * std::asinh((B * radius) / A));
    };


    const double currentRadius = startRadius + deltaRadius * progress;
    const double f0 = Primitive(startRadius);
    const double f1 = Primitive(currentRadius);
    double length = (f1 - f0) / deltaRadius;


    // Numerical safety
    if (length < 0.0)
    {
        length = -length;
    }


    return length;
}


// =========================================================
// 根據「已走路徑長度」反求 Spiral Progress
//
// Virtual Axis 的 currentCmdPos 是 Path Distance，
// 但 Spiral 的 p 不能單純使用：
//
// currentCmdPos / totalDist
//
// 因為變半徑時，每一段 p 所代表的實際距離不同。
//
// 使用 Newton iteration 反求 p。
// =========================================================
static double CalcSpiralProgressFromPathLength(double pathLength, double totalLength, double startRadius, double endRadius, double totalAngle, double deltaZ)
{
    if (totalLength <= 1e-12)
    {
        return 0.0;
    }


    if (pathLength <= 0.0)
    {
        return 0.0;
    }

    if (pathLength >= totalLength)
    {
        return 1.0;
    }


    const double deltaRadius = endRadius - startRadius;


    // =====================================================
    // 固定半徑時，不需要 Newton
    //
    // 標準 G02 / G03 直接走原本線性 Progress。
    // =====================================================
    if (std::abs(deltaRadius) < 1e-12)
    {
        return pathLength / totalLength;
    }


    // 初始猜測
    double progress = pathLength / totalLength;


    // =====================================================
    // Newton Iteration
    //
    // 通常 3~4 次就會非常接近。
    // 固定使用 4 次，避免 RT Thread 不確定迴圈。
    // =====================================================
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        const double currentLength = CalcSpiralArcLengthAtProgress(progress, startRadius, endRadius, totalAngle, deltaZ);
        const double currentRadius = startRadius + deltaRadius * progress;


        // dS / dp
        const double metric = std::sqrt(deltaRadius * deltaRadius + deltaZ * deltaZ + currentRadius * currentRadius * totalAngle * totalAngle);

        if (metric < 1e-12)
        {
            break;
        }


        progress -= (currentLength - pathLength) / metric;


        // Clamp
        if (progress < 0.0)
        {
            progress = 0.0;
        }
        else if (progress > 1.0)
        {
            progress = 1.0;
        }
    }


    return progress;
}
void MotionCore::LoadNextCommand()
{
    // ======================================================
    // 1. 派單保護
    // ======================================================
    // This acquire observation is the handoff/completion linearization seam.
    // If a newer lifecycle tuple is already pending, the active segment must
    // be terminated by ApplyPendingExecutionEpochChange(), not incorrectly
    // reported COMPLETED here before the dequeue gate gets a chance to defer.
    if (HasPendingExecutionEpochChange())
    {
        return;
    }

    if (m_Group.cmdQueue.empty())
        return;

    AxisContext& vAxis = m_Group.virtualAxis;

    // 群組還在跑，而且虛擬軸尚未允許交接
    if (m_Group.isActive && !vAxis.inPosition)
    {
        return;
    }


    // ======================================================
    // 2. 先取得已授權的 Next，並在任何 Complete / Pop / Mapping
    //    overwrite 前驗證零速交接。
    //
    // Changed axis count/order is never a tangent-continuous junction.  The
    // outgoing virtual and physical commands must already be zero and inside
    // the following window before the old mapping can be retired.  Encoder
    // velocity remains advisory per the established J.5 quantization contract.
    // This closes the X->Y->X->Y P1 runaway seam.
    // ======================================================
    DiscardStaleQueuedCommands();

    if (m_Group.cmdQueue.empty())
    {
        return;
    }

    MotionCommand frontCommand{};
    if (!TryPeekNextMotionCommand(frontCommand) ||
        GetCommandAuthorizationFailure(frontCommand) !=
        MotionRejectReason::NONE)
    {
        return;
    }

    if (!IsMotionCommandConsumerGeometryValid(
        frontCommand,
        m_pContexts))
    {
        MotionCommand invalidCommand{};
        // The lifecycle tuple may win after the peek. Never overwrite a
        // pending RESET/GOTO Epoch with a forced 3021 Epoch. Only the exact
        // peeked front, raw-popped and still authorized, may become the
        // causal invalid-geometry terminal.
        if (HasPendingExecutionEpochChange() ||
            !m_Group.cmdQueue.ConsumerTryPop(invalidCommand))
        {
            return;
        }

        const bool exactPeekedIdentity =
            MotionExecutionIdentityExactlyMatches(
                frontCommand.execution,
                invalidCommand.execution) &&
            frontCommand.ownerLease.owner ==
            invalidCommand.ownerLease.owner &&
            frontCommand.ownerLease.generation ==
            invalidCommand.ownerLease.generation;
        const MotionRejectReason authorizationFailure =
            GetCommandAuthorizationFailure(invalidCommand);
        if (!exactPeekedIdentity ||
            authorizationFailure != MotionRejectReason::NONE)
        {
            if (authorizationFailure != MotionRejectReason::NONE)
            {
                RejectMotionCommand(
                    invalidCommand,
                    authorizationFailure,
                    0U);
            }
            else
            {
                if (HasPendingExecutionEpochChange())
                {
                    RejectMotionCommand(
                        invalidCommand,
                        MotionRejectReason::STALE_EPOCH,
                        0U);
                    return;
                }
                m_p1DroppedAxisRetirementFailureCount.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
                m_p1LastOrphanAxisIndex.store(
                    -1,
                    std::memory_order_release);
                TriggerGroupMappingIntegrityEmergencyStop(-1, true);
                RejectMotionCommand(
                    invalidCommand,
                    MotionRejectReason::INVALID_GEOMETRY,
                    static_cast<std::uint32_t>(
                        AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
            }
            return;
        }

        if (HasPendingExecutionEpochChange())
        {
            RejectMotionCommand(
                invalidCommand,
                MotionRejectReason::STALE_EPOCH,
                0U);
            return;
        }
        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        RejectMotionCommand(
            invalidCommand,
            MotionRejectReason::INVALID_GEOMETRY,
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
        return;
    }

    const double prospectiveCruiseVelocity =
        std::abs(frontCommand.targetVel) * m_Group.feedrateOverride;
    if (!std::isfinite(m_Group.feedrateOverride) ||
        m_Group.feedrateOverride < 0.0 ||
        !std::isfinite(prospectiveCruiseVelocity) ||
        prospectiveCruiseVelocity < 0.0)
    {
        if (HasPendingExecutionEpochChange())
        {
            return;
        }
        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        TriggerGroupMappingIntegrityEmergencyStop(-1);
        return;
    }

    const int previousAxisCount =
        ClampMotionAxisCount(m_Group.axisCount);
    std::array<int, MAX_AXES> previousAxisIndices{};
    for (int slot = 0; slot < previousAxisCount; ++slot)
    {
        previousAxisIndices[slot] = m_Group.axisIndices[slot];
    }

    const bool hasOutgoingCommand =
        m_Group.isActive &&
        m_Group.currentCmd.execution.IsAssigned() &&
        previousAxisCount > 0;

    if (hasOutgoingCommand && !std::isfinite(vAxis.targetEndVel))
    {
        if (HasPendingExecutionEpochChange())
        {
            return;
        }
        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        TriggerGroupMappingIntegrityEmergencyStop(-1);
        return;
    }

    const bool mappingChanged =
        hasOutgoingCommand &&
        !MotionCommandsHaveIdenticalAxisMapping(
            m_Group.currentCmd,
            frontCommand);
    const bool zeroSpeedJunction =
        hasOutgoingCommand &&
        (mappingChanged ||
            (m_Group.pathMode != PathMode::PATH_SERVO &&
                std::abs(vAxis.targetEndVel) <= 0.1));

    if (mappingChanged)
    {
        m_p1LastPreviousAxisMask.store(
            BuildMotionCommandAxisMask(m_Group.currentCmd),
            std::memory_order_release);
        m_p1LastNextAxisMask.store(
            BuildMotionCommandAxisMask(frontCommand),
            std::memory_order_release);

        // PATH_SERVO and B2/JUMP endpoints do not establish the canonical
        // IDLE proof required to retire an axis mapping. Mixed-map crossing
        // is explicitly unsupported in those modes and must alarm instead
        // of waiting forever or switching for one unsafe Runtime pass.
        if (m_Group.pathMode != PathMode::EXACT_STOP &&
            m_Group.pathMode != PathMode::CONTINUOUS)
        {
            if (HasPendingExecutionEpochChange())
            {
                return;
            }
            m_p1DroppedAxisRetirementFailureCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
            TriggerGroupMappingIntegrityEmergencyStop(-1);
            return;
        }

        // A changed mapping must have been planned as an exact-stop boundary
        // when the outgoing command was loaded.  A nonzero end speed here is
        // an invariant breach; never pop the next command or overwrite the
        // only mapping that can still stop the outgoing axes.
        if (!std::isfinite(vAxis.targetEndVel) ||
            std::abs(vAxis.targetEndVel) > 0.1)
        {
            if (HasPendingExecutionEpochChange())
            {
                return;
            }
            m_p1DroppedAxisRetirementFailureCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
            TriggerGroupMappingIntegrityEmergencyStop(-1);
            return;
        }
    }

    if (zeroSpeedJunction)
    {
        bool outgoingEvidenceInvalid =
            !std::isfinite(vAxis.currentCmdVel) ||
            !std::isfinite(vAxis.logicalCmdVel) ||
            vAxis.isFault ||
            vAxis.isLagAlarm ||
            vAxis.state == MotionState::MotionState_ERROR ||
            vAxis.state == MotionState::MotionState_ESTOP;
        bool outgoingStopped =
            vAxis.inPosition &&
            vAxis.state == MotionState::MotionState_IDLE &&
            std::abs(vAxis.currentCmdVel) <= 1.0 &&
            std::abs(vAxis.logicalCmdVel) <= 1.0;

        if (m_pContexts == nullptr)
        {
            outgoingEvidenceInvalid = true;
        }
        else
        {
            for (int slot = 0;
                !outgoingEvidenceInvalid && slot < previousAxisCount;
                ++slot)
            {
                const int axisIndex = previousAxisIndices[slot];
                if (axisIndex < 0 ||
                    axisIndex >= static_cast<int>(m_pContexts->size()))
                {
                    outgoingEvidenceInvalid = true;
                    m_p1LastOrphanAxisIndex.store(
                        axisIndex,
                        std::memory_order_release);
                    break;
                }

                const AxisContext& axis = (*m_pContexts)[axisIndex];
                const double followingError =
                    std::abs(axis.currentCmdPos - axis.currentActPos);
                const bool stateAllowed =
                    axis.state == MotionState::MotionState_INTERPOLATING ||
                    axis.state == MotionState::MotionState_STOPPING ||
                    axis.state == MotionState::MotionState_IDLE;
                const bool evidenceValid =
                    axis.isExist &&
                    axis.isServoOn &&
                    !axis.isFault &&
                    !axis.isLagAlarm &&
                    stateAllowed &&
                    std::isfinite(axis.currentCmdVel) &&
                    std::isfinite(axis.logicalCmdVel) &&
                    std::isfinite(axis.currentCmdPos) &&
                    std::isfinite(axis.currentActPos) &&
                    std::isfinite(axis.inPositionWindow_Pulse) &&
                    axis.inPositionWindow_Pulse > 0.0;
                if (!evidenceValid)
                {
                    outgoingEvidenceInvalid = true;
                    m_p1LastOrphanAxisIndex.store(
                        axisIndex,
                        std::memory_order_release);
                    break;
                }

                outgoingStopped =
                    outgoingStopped &&
                    std::abs(axis.currentCmdVel) <= 1.0 &&
                    std::abs(axis.logicalCmdVel) <= 1.0 &&
                    std::isfinite(followingError) &&
                    followingError <= axis.inPositionWindow_Pulse;
            }
        }

        if (outgoingEvidenceInvalid)
        {
            if (HasPendingExecutionEpochChange())
            {
                return;
            }
            m_p1DroppedAxisRetirementFailureCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            TriggerGroupMappingIntegrityEmergencyStop(
                m_p1LastOrphanAxisIndex.load(
                    std::memory_order_acquire));
            return;
        }

        // Hold the stopped endpoint and retry on the next 250 us pass.  This
        // is not a fault: the encoder may need several cycles to settle after
        // the command velocity reaches zero.
        if (!outgoingStopped)
        {
            return;
        }
    }


    // ======================================================
    // 3. 由 250 us Consumer 取得下一條 Motion Command
    //
    // The outgoing segment is deliberately not terminal yet. Pop,
    // re-authorization, consumer validation and dropped-axis retirement form
    // one handoff transaction; only its successful commit may complete and
    // archive the outgoing segment.
    // ======================================================
    MotionCommand cmd{};

    if (HasPendingExecutionEpochChange() ||
        !m_Group.cmdQueue.ConsumerTryPop(cmd))
    {
        return;
    }

    const bool exactPeekedIdentity =
        MotionExecutionIdentityExactlyMatches(
            frontCommand.execution,
            cmd.execution) &&
        frontCommand.ownerLease.owner == cmd.ownerLease.owner &&
        frontCommand.ownerLease.generation ==
        cmd.ownerLease.generation;

    // Epoch 或 Owner Lease 可能在 Pop 後、切入 Current Command 前改變。
    const MotionRejectReason postPopAuthorizationFailure =
        GetCommandAuthorizationFailure(cmd);

    if (postPopAuthorizationFailure != MotionRejectReason::NONE)
    {
        const bool trackedTransportCopy =
            IsTerminalizedReplayOrTrackedCommand(cmd);
        if (!trackedTransportCopy &&
            postPopAuthorizationFailure == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else if (!trackedTransportCopy)
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        if (!trackedTransportCopy)
        {
            m_lastRejectedSegmentId.store(
                cmd.execution.segmentId,
                std::memory_order_relaxed);
        }

        RejectMotionCommand(
            cmd,
            postPopAuthorizationFailure,
            0U);

        return;
    }

    const auto RejectPoppedCommandIfLifecycleSuperseded =
        [this, &cmd]() noexcept -> bool
    {
        const MotionRejectReason authorizationFailure =
            GetCommandAuthorizationFailure(cmd);
        if (!HasPendingExecutionEpochChange() &&
            authorizationFailure == MotionRejectReason::NONE)
        {
            return false;
        }

        const MotionRejectReason terminalReason =
            authorizationFailure == MotionRejectReason::NONE
            ? MotionRejectReason::STALE_EPOCH
            : authorizationFailure;
        const bool trackedTransportCopy =
            IsTerminalizedReplayOrTrackedCommand(cmd);
        if (!trackedTransportCopy &&
            terminalReason == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else if (!trackedTransportCopy)
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        if (!trackedTransportCopy)
        {
            m_lastRejectedSegmentId.store(
                cmd.execution.segmentId,
                std::memory_order_relaxed);
        }
        RejectMotionCommand(cmd, terminalReason, 0U);
        return true;
    };

    if (!exactPeekedIdentity ||
        !IsMotionCommandConsumerGeometryValid(cmd, m_pContexts) ||
        (hasOutgoingCommand &&
            mappingChanged !=
            (!MotionCommandsHaveIdenticalAxisMapping(
                m_Group.currentCmd,
                cmd))))
    {
        if (RejectPoppedCommandIfLifecycleSuperseded())
        {
            return;
        }
        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        RejectMotionCommand(
            cmd,
            MotionRejectReason::INVALID_GEOMETRY,
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
        return;
    }

    // An axis introduced by the incoming mapping is not owned by the
    // outgoing group. Prove it is servo-ready, canonical IDLE, stopped and
    // inside its following window before this transaction can claim it.
    for (int slot = 0; slot < cmd.axisCount; ++slot)
    {
        const int axisIndex = cmd.axisIndices[slot];
        const bool incomingOnly =
            !hasOutgoingCommand ||
            !MotionCommandHasAxis(m_Group.currentCmd, axisIndex);
        if (!incomingOnly)
        {
            continue;
        }

        const AxisContext& incomingAxis = (*m_pContexts)[axisIndex];
        if (!IsIncomingPhysicalAxisReadyForGroup(incomingAxis))
        {
            if (RejectPoppedCommandIfLifecycleSuperseded())
            {
                return;
            }
            m_p1DroppedAxisRetirementFailureCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            m_p1LastOrphanAxisIndex.store(
                axisIndex,
                std::memory_order_release);
            TriggerGroupMappingIntegrityEmergencyStop(axisIndex, true);
            RejectMotionCommand(
                cmd,
                MotionRejectReason::NOT_READY,
                static_cast<std::uint32_t>(
                    AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
            return;
        }
    }

    // Prove every dropped axis is canonicalizable without mutating it. The
    // final lifecycle seam below must run after all pure validation and before
    // any AxisContext state changes.
    if (hasOutgoingCommand && mappingChanged && m_pContexts != nullptr)
    {
        bool retirementFailed = false;
        for (int slot = 0; slot < previousAxisCount; ++slot)
        {
            const int axisIndex = previousAxisIndices[slot];
            if (MotionCommandHasAxis(cmd, axisIndex))
            {
                continue;
            }

            if (axisIndex < 0 ||
                axisIndex >= static_cast<int>(m_pContexts->size()) ||
                !CanCanonicalizeInactivePhysicalAxisCommandState(
                    (*m_pContexts)[axisIndex]))
            {
                retirementFailed = true;
                m_p1LastOrphanAxisIndex.store(
                    axisIndex,
                    std::memory_order_release);
                break;
            }

        }

        if (retirementFailed)
        {
            if (RejectPoppedCommandIfLifecycleSuperseded())
            {
                return;
            }
            m_p1DroppedAxisRetirementFailureCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            TriggerGroupMappingIntegrityEmergencyStop(
                m_p1LastOrphanAxisIndex.load(
                    std::memory_order_acquire));
            RejectMotionCommand(
                cmd,
                MotionRejectReason::NOT_READY,
                static_cast<std::uint32_t>(
                    AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
            return;
        }
    }

    // Final lifecycle seam after all bounded readiness/retirement work and
    // immediately before the only commit that completes the outgoing
    // identity and publishes the incoming one. A newly published Reset or
    // Alarm Epoch wins; this pass must not dispatch after that boundary.
    if (RejectPoppedCommandIfLifecycleSuperseded())
    {
        return;
    }

    const MotionExecutionIdentity& commitExecution =
        hasOutgoingCommand
        ? m_Group.currentCmd.execution
        : cmd.execution;
    LifecycleCommitReservationGuard lifecycleCommit(
        *this,
        commitExecution);
    if (!lifecycleCommit.IsAcquired())
    {
        // A strong CAS loss means a lifecycle publisher changed the packed
        // word first.  The popped command can no longer be dispatched.
        if (!RejectPoppedCommandIfLifecycleSuperseded())
        {
            RejectMotionCommand(
                cmd,
                MotionRejectReason::STALE_EPOCH,
                0U);
        }
        return;
    }

    // Apply the already-proven zero-state changes. AxisContext has one RT
    // writer, so these helpers cannot fail unless an internal invariant was
    // corrupted inside this same pass.
    for (int slot = 0; slot < cmd.axisCount; ++slot)
    {
        const int axisIndex = cmd.axisIndices[slot];
        const bool incomingOnly =
            !hasOutgoingCommand ||
            !MotionCommandHasAxis(m_Group.currentCmd, axisIndex);
        if (incomingOnly &&
            !TryCanonicalizeIdleAxisCommandState(
                (*m_pContexts)[axisIndex]))
        {
            if (RejectPoppedCommandIfLifecycleSuperseded())
            {
                return;
            }
            m_p1DroppedAxisRetirementFailureCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
            m_p1LastOrphanAxisIndex.store(
                axisIndex,
                std::memory_order_release);
            lifecycleCommit.Release();
            TriggerGroupMappingIntegrityEmergencyStop(axisIndex, true);
            RejectMotionCommand(
                cmd,
                MotionRejectReason::NOT_READY,
                static_cast<std::uint32_t>(
                    AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
            return;
        }
    }

    std::uint64_t retiredAxisCount = 0ULL;
    if (hasOutgoingCommand && mappingChanged && m_pContexts != nullptr)
    {
        for (int slot = 0; slot < previousAxisCount; ++slot)
        {
            const int axisIndex = previousAxisIndices[slot];
            if (MotionCommandHasAxis(cmd, axisIndex))
            {
                continue;
            }

            if (!TryCanonicalizeInactivePhysicalAxisCommandState(
                (*m_pContexts)[axisIndex]))
            {
                if (RejectPoppedCommandIfLifecycleSuperseded())
                {
                    return;
                }
                m_p1DroppedAxisRetirementFailureCount.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
                m_p1LastOrphanAxisIndex.store(
                    axisIndex,
                    std::memory_order_release);
                lifecycleCommit.Release();
                TriggerGroupMappingIntegrityEmergencyStop(axisIndex, true);
                RejectMotionCommand(
                    cmd,
                    MotionRejectReason::NOT_READY,
                    static_cast<std::uint32_t>(
                        AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
                return;
            }
            ++retiredAxisCount;
        }
    }

    // Close the small validation-to-mutation interval as well. If lifecycle
    // wins here, zeroed axes remain safe but no successful handoff diagnostic
    // or incoming dispatch is committed.
    if (RejectPoppedCommandIfLifecycleSuperseded())
    {
        return;
    }

    if (mappingChanged)
    {
        m_p1MappingBoundaryStopCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        if (retiredAxisCount != 0ULL)
        {
            m_p1DroppedAxisRetirementCount.fetch_add(
                retiredAxisCount,
                std::memory_order_relaxed);
        }
    }

    // ======================================================
    // 4. Commit the successful handoff transaction.
    // ======================================================
    if (m_Group.isActive && vAxis.inPosition)
    {
        CompleteTrackedMotionCommand(
            m_Group.currentCmd);
    }

    if (m_Group.isActive &&
        m_Group.enableHistory &&
        GetCommandAuthorizationFailure(m_Group.currentCmd) ==
        MotionRejectReason::NONE)
    {
        MotionCommand completedHistory = m_Group.currentCmd;
        completedHistory.replayTerminalAlreadyPublished = true;
        m_Group.historyQueue.push_back(completedHistory);

        if (m_Group.historyQueue.size() >
            MOTION_COMMAND_HISTORY_LIMIT)
        {
            m_Group.historyQueue.pop_front();
        }
    }

    // 保存目前執行指令
    m_Group.currentCmd = cmd;


    // ======================================================
    // 5. 更新 NC 執行資訊
    // ======================================================
    m_Group.currentExecutionPC = cmd.sourceLinePC;
    m_Group.currentExecutionWCS = cmd.sourceWCS;
    m_Group.currentExecutionToolMode = cmd.sourceToolLengthMode;
    m_Group.currentExecutionHCode = cmd.sourceHCode;
    m_Group.currentExecutionToolRadiusMode = cmd.sourceToolRadiusMode;
    m_Group.currentExecutionDCode = cmd.sourceDCode;
    m_Group.currentExecutionIsAbsoluteMode = cmd.sourceIsAbsoluteMode;
    m_Group.currentExecutionG68Active = cmd.sourceG68Active;
    m_Group.currentExecutionG68Angle = cmd.sourceG68Angle;
    m_Group.currentExecutionG168Active = cmd.sourceG168Active;
    m_Group.currentExecutionWCode = cmd.sourceWCode;
    m_Group.currentExecutionG51Active = cmd.sourceG51Active;
    m_Group.currentExecutionScaleRatio = cmd.sourceScaleRatio;
    m_Group.currentExecutionMirrorMask = cmd.sourceMirrorMask;
    m_Group.currentExecutionG16Active = cmd.sourceG16Active;
    m_Group.currentExecutionG162Active = cmd.sourceG162Active;
    m_Group.currentExecutionPlaneMode = cmd.sourcePlaneMode;


    // ======================================================
    // 5. 更新群組模式
    // ======================================================
    m_Group.mode = cmd.mode;
    m_Group.axisCount = cmd.axisCount;


    // ======================================================
    // 6. S-Curve 殘留距離
    // ======================================================
    double trappedDist = vAxis.planningPos - vAxis.currentCmdPos;

    if (trappedDist < 0.0)
    {
        trappedDist = 0.0;
    }


    // ======================================================
    // 注意：
    // 這裡絕對不要先做：
    //
    // vAxis.inPosition = false;
    //
    // 因為此時還不知道這條指令是否真的需要移動。
    // ======================================================


    // 預設終點速度先歸零
    vAxis.targetEndVel = 0.0;


    // ======================================================
    // Helper：
    // 將「已經在到位視窗內 / 極小路徑」安全視為完成
    // ======================================================
    auto CompleteWithoutMotion =
        [&]()
    {
        // ----------------------------------------------
        // 先更新所有邏輯座標
        // ----------------------------------------------
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = m_Group.axisIndices[i];

            AxisContext& realAxis = (*m_pContexts)[idx];

            realAxis.logicalCmdPos = m_Group.currentCmd.targetPos[i];

            realAxis.logicalCmdVel = 0.0;
            realAxis.currentCmdVel = 0.0;
        }


        // ----------------------------------------------
        // G68 / 空間 Transform 啟用時
        // Logical 不能直接等於 Physical
        // 必須套矩陣
        // ----------------------------------------------
        if (m_Group.enableTransform && m_Group.axisCount >= 2)
        {
            int idxX = m_Group.axisIndices[0];

            int idxY = m_Group.axisIndices[1];

            int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

            AxisContext& realX = (*m_pContexts)[idxX];
            AxisContext& realY = (*m_pContexts)[idxY];

            double logP[3] = { realX.logicalCmdPos,realY.logicalCmdPos, 0.0 };

            if (idxZ != -1)
            {
                logP[2] = (*m_pContexts)[idxZ].logicalCmdPos;
            }

            double physP[3] = { 0.0,0.0, 0.0 };

            for (int r = 0; r < 3; ++r)
            {
                physP[r] = m_Group.transformOrigin[r];

                for (int c = 0; c < 3; ++c)
                {
                    physP[r] += m_Group.transformMatrix[r][c] * (logP[c] - m_Group.transformOrigin[c]);
                }
            }

            realX.currentCmdPos = physP[0];

            realY.currentCmdPos = physP[1];

            if (idxZ != -1)
            {
                (*m_pContexts)[idxZ].currentCmdPos = physP[2];
            }
        }
        else
        {
            // ------------------------------------------
            // 沒有 Transform：
            // Physical Command = Logical Command
            // ------------------------------------------
            for (int i = 0; i < m_Group.axisCount; ++i)
            {
                int idx = m_Group.axisIndices[i];

                AxisContext& realAxis = (*m_pContexts)[idx];

                realAxis.currentCmdPos = realAxis.logicalCmdPos;
            }
        }


        // ----------------------------------------------
        // 實體軸安全完成
        // ----------------------------------------------
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = m_Group.axisIndices[i];

            AxisContext& realAxis = (*m_pContexts)[idx];

            realAxis.logicalCmdVel = 0.0;
            realAxis.currentCmdVel = 0.0;

            realAxis.inPosition = true;

            realAxis.state = MotionState::MotionState_IDLE;
        }


        // ----------------------------------------------
        // Virtual Axis 完成
        // ----------------------------------------------
        vAxis.currentCmdVel = 0.0;
        vAxis.targetEndVel = 0.0;
        vAxis.inPosition = true;
        vAxis.state = MotionState::MotionState_IDLE;
        m_Group.isActive = false;

        // 合法但不需要實際位移的 Segment，仍有完整生命週期：
        // ACCEPTED -> COMPLETED；不發布 STARTED。
        TrackMotionCommandAccepted(
            m_Group.currentCmd);

        CompleteTrackedMotionCommand(
            m_Group.currentCmd);
    };


    // ======================================================
    // Helper：
    // 非 PATH_SERVO 模式不允許「有距離但速度為 0」
    // ======================================================
    auto HasInvalidZeroVelocity =
        [&]() -> bool
    {
        // PATH_SERVO 的速度可能由放電 Servo 外部控制
        // 所以不能在這裡擋
        if (m_Group.pathMode == PathMode::PATH_SERVO)
        {
            return false;
        }

        if (std::abs(cmd.targetVel) >= 1.0)
        {
            return false;
        }

        const MotionRejectReason lifecycleFailure =
            GetCommandAuthorizationFailure(m_Group.currentCmd);
        if (HasPendingExecutionEpochChange() ||
            lifecycleFailure != MotionRejectReason::NONE)
        {
            RejectMotionCommand(
                m_Group.currentCmd,
                lifecycleFailure == MotionRejectReason::NONE
                ? MotionRejectReason::STALE_EPOCH
                : lifecycleFailure,
                0U);
            return true;
        }

        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        lifecycleCommit.Release();
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        RejectMotionCommand(
            m_Group.currentCmd,
            MotionRejectReason::INVALID_GEOMETRY,
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));

        return true;
    };

    const auto FailDerivedConsumerGeometry =
        [this, &cmd, &lifecycleCommit]() noexcept
    {
        const MotionRejectReason lifecycleFailure =
            GetCommandAuthorizationFailure(m_Group.currentCmd);
        if (HasPendingExecutionEpochChange() ||
            lifecycleFailure != MotionRejectReason::NONE)
        {
            RejectMotionCommand(
                m_Group.currentCmd,
                lifecycleFailure == MotionRejectReason::NONE
                ? MotionRejectReason::STALE_EPOCH
                : lifecycleFailure,
                0U);
            return;
        }

        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        lifecycleCommit.Release();
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        RejectMotionCommand(
            cmd,
            MotionRejectReason::INVALID_GEOMETRY,
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
    };


    // ======================================================
    // 7. 幾何運算
    // ======================================================

    // ======================================================
    // LINEAR
    // ======================================================
    if (m_Group.mode == InterpolationMode::LINEAR)
    {
        double totalDist = 0.0;
        bool alreadyAtTarget = true;
        bool derivedGeometryValid = true;


        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = cmd.axisIndices[i];

            m_Group.axisIndices[i] = idx;

            AxisContext& realAxis = (*m_pContexts)[idx];


            // ----------------------------------------------
            // 起點使用 Logical Command Position
            // ----------------------------------------------
            m_Group.startPos[i] = realAxis.logicalCmdPos;


            // ----------------------------------------------
            // 目標
            // ----------------------------------------------
            double actualTarget = cmd.targetPos[i];


            // ----------------------------------------------
            // Rotary shortest path
            // ----------------------------------------------
            if (realAxis.axisType == AxisType::ROTARY && realAxis.useShortestPath)
            {
                actualTarget = CalculateShortestTarget(m_Group.startPos[i], actualTarget, realAxis.rotaryModulo);


                // local cmd
                cmd.targetPos[i] = actualTarget;

                // current command
                m_Group.currentCmd.targetPos[i] = actualTarget;
            }


            // ----------------------------------------------
            // 距離
            // ----------------------------------------------
            double delta = actualTarget - m_Group.startPos[i];

            if (!std::isfinite(m_Group.startPos[i]) ||
                !std::isfinite(actualTarget) ||
                !std::isfinite(delta) ||
                !std::isfinite(realAxis.currentCmdPos) ||
                !std::isfinite(realAxis.currentActPos) ||
                !std::isfinite(realAxis.inPositionWindow_Pulse) ||
                realAxis.inPositionWindow_Pulse <= 0.0)
            {
                derivedGeometryValid = false;
                break;
            }

            totalDist = std::hypot(totalDist, delta);
            if (!std::isfinite(totalDist))
            {
                derivedGeometryValid = false;
                break;
            }

            m_Group.ratio[i] = delta;


            // ----------------------------------------------
            // 只要命令差距超過到位視窗
            // 就真的需要運動
            // ----------------------------------------------
            if (std::abs(delta) > realAxis.inPositionWindow_Pulse)
            {
                alreadyAtTarget = false;
            }


            // ----------------------------------------------
            // 額外安全：
            // 如果實體馬達自己還沒追進視窗
            // 也不能直接宣告完成
            // ----------------------------------------------
            double physicalLag = std::abs(realAxis.currentCmdPos - realAxis.currentActPos);

            if (physicalLag > realAxis.inPositionWindow_Pulse)
            {
                alreadyAtTarget = false;
            }
        }


        if (!derivedGeometryValid || !std::isfinite(totalDist))
        {
            FailDerivedConsumerGeometry();
            return;
        }


        // ==================================================
        // 已經在目標視窗
        // ==================================================
        if (alreadyAtTarget)
        {
            CompleteWithoutMotion();
            return;
        }


        // ==================================================
        // 數學極小值保護
        // ==================================================
        if (totalDist < 1e-5)
        {
            CompleteWithoutMotion();
            return;
        }


        // ==================================================
        // 有距離要跑，速度卻是 0
        // ==================================================
        if (HasInvalidZeroVelocity())
        {
            return;
        }


        // ==================================================
        // 到這裡才確定「真的需要移動」
        // ==================================================
        vAxis.inPosition = false;


        // ==================================================
        // 實體軸正式切 INTERPOLATING
        // ==================================================
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = m_Group.axisIndices[i];

            AxisContext& realAxis = (*m_pContexts)[idx];

            realAxis.state = MotionState::MotionState_INTERPOLATING;

            realAxis.inPosition = false;
        }


        // ==================================================
        // 正規化方向向量
        // ==================================================
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            m_Group.ratio[i] /=
                totalDist;
            if (!std::isfinite(m_Group.ratio[i]))
            {
                FailDerivedConsumerGeometry();
                return;
            }
        }


        // Virtual Path Distance
        vAxis.finalTargetPos = totalDist;
    }


    // ======================================================
    // CIRCULAR
    // ======================================================
    else if (m_Group.mode == InterpolationMode::CIRCULAR_CW || m_Group.mode == InterpolationMode::CIRCULAR_CCW)
    {
        int axisX = cmd.axisIndices[0];

        int axisY = cmd.axisIndices[1];


        m_Group.axisIndices[0] = axisX;

        m_Group.axisIndices[1] = axisY;


        // ----------------------------------------------
        // 起點
        // ----------------------------------------------
        m_Group.startPos[0] = (*m_pContexts)[axisX].logicalCmdPos;

        m_Group.startPos[1] = (*m_pContexts)[axisY].logicalCmdPos;


        // ----------------------------------------------
        // Z 軸
        // ----------------------------------------------
        double deltaZ = 0.0;

        int axisZ = -1;


        if (m_Group.axisCount >= 3)
        {
            axisZ = cmd.axisIndices[2];
            m_Group.axisIndices[2] = axisZ;

            m_Group.startPos[2] = (*m_pContexts)[axisZ].logicalCmdPos;

            deltaZ = cmd.targetPos[2] - m_Group.startPos[2];
        }


        // ----------------------------------------------
        // 圓弧幾何
        // ----------------------------------------------
        double sx = m_Group.startPos[0];
        double sy = m_Group.startPos[1];
        double cx = cmd.centerPos[0];
        double cy = cmd.centerPos[1];
        double ex = cmd.targetPos[0];
        double ey = cmd.targetPos[1];

        if (!std::isfinite(sx) ||
            !std::isfinite(sy) ||
            !std::isfinite(cx) ||
            !std::isfinite(cy) ||
            !std::isfinite(ex) ||
            !std::isfinite(ey) ||
            !std::isfinite(deltaZ))
        {
            FailDerivedConsumerGeometry();
            return;
        }


        m_Group.centerX = cx;
        m_Group.centerY = cy;


        // ----------------------------------------------
        // 起始 / 結束角
        // ----------------------------------------------
        m_Group.startAngle = std::atan2(sy - cy, sx - cx);

        double endAngle = std::atan2(ey - cy, ex - cx);


        double totalAngle = endAngle - m_Group.startAngle;


        if (m_Group.mode == InterpolationMode::CIRCULAR_CCW)
        {
            if (totalAngle <= 0.0)
            {
                totalAngle += 2.0 * 3.14159265359;
            }
        }
        else
        {
            if (totalAngle >= 0.0)
            {
                totalAngle -= 2.0 * 3.14159265359;
            }
        }


        m_Group.totalAngle = totalAngle;


        // ----------------------------------------------
        // Radius
        // ----------------------------------------------
        const double startDeltaX = sx - cx;
        const double startDeltaY = sy - cy;
        const double endDeltaX = ex - cx;
        const double endDeltaY = ey - cy;
        double startRadius = std::hypot(startDeltaX, startDeltaY);
        double endRadius = std::hypot(endDeltaX, endDeltaY);
        if (!std::isfinite(startDeltaX) ||
            !std::isfinite(startDeltaY) ||
            !std::isfinite(endDeltaX) ||
            !std::isfinite(endDeltaY) ||
            !std::isfinite(startRadius) ||
            !std::isfinite(endRadius) ||
            !std::isfinite(totalAngle))
        {
            FailDerivedConsumerGeometry();
            return;
        }
        m_Group.currentCmd.startRadius = startRadius;
        m_Group.currentCmd.endRadius = endRadius;
        m_Group.radius = startRadius;


        // ----------------------------------------------
        // 3D Arc Distance
        // ----------------------------------------------
        double avgRadius = (startRadius + endRadius) / 2.0;

        double arcLength = avgRadius * std::abs(totalAngle);


        // ----------------------------------------------
        // True 3D Spiral / Arc Distance
        //
        // 支援：
        // 1. 標準等半徑 Arc
        // 2. Helix
        // 3. Variable Radius Spiral
        // 4. Variable Radius + Z
        // ----------------------------------------------
        double totalDist3D = CalcSpiralArcLengthAtProgress(1.0, startRadius, endRadius, totalAngle, deltaZ);

        if (!std::isfinite(totalDist3D))
        {
            FailDerivedConsumerGeometry();
            return;
        }


        // ==================================================
        // 極小圓弧
        //
        // 舊版：
        // LoadNextCommand();
        //
        // 不再這樣做，避免 recursive / 派單鎖互卡
        // ==================================================
        if (totalDist3D < 1e-5)
        {
            CompleteWithoutMotion();
            return;
        }


        // ==================================================
        // 有距離卻沒有速度
        // ==================================================
        if (HasInvalidZeroVelocity())
        {
            return;
        }


        // ==================================================
        // 到這裡才真的進入插補
        // ==================================================
        vAxis.inPosition = false;


        AxisContext& realX = (*m_pContexts)[axisX];

        AxisContext& realY = (*m_pContexts)[axisY];


        realX.state = MotionState::MotionState_INTERPOLATING;

        realX.inPosition = false;


        realY.state = MotionState::MotionState_INTERPOLATING;

        realY.inPosition = false;


        if (axisZ != -1)
        {
            AxisContext& realZ = (*m_pContexts)[axisZ];

            realZ.state = MotionState::MotionState_INTERPOLATING;

            realZ.inPosition = false;
        }


        m_Group.totalDist3D = totalDist3D;

        vAxis.finalTargetPos = totalDist3D;
    }


    // ======================================================
    // 不支援的 Mode
    // ======================================================
    else
    {
        //RtPrintf("[MOTION ERROR] " "Unknown interpolation mode!\n");

        vAxis.currentCmdVel = 0.0;
        vAxis.inPosition = true;

        vAxis.state = MotionState::MotionState_ERROR;

        vAxis.isFault = true;

        m_Group.isActive = false;

        RejectMotionCommand(
            m_Group.currentCmd,
            MotionRejectReason::INVALID_GEOMETRY,
            0U);

        return;
    }


    // ======================================================
    // 8. Virtual Axis 軌跡初始化
    // ======================================================

    // S-Curve 前一段殘留距離
    vAxis.planningPos = trappedDist;

    // 新路徑 Virtual Position 從 0 開始
    vAxis.currentCmdPos = 0.0;


    // EXACT_STOP 必須從 0 速度起跑
    if (m_Group.pathMode == PathMode::EXACT_STOP)
    {
        vAxis.currentCmdVel = 0.0;
    }


    // ======================================================
    // 9. 速度 / 加減速參數
    // ======================================================
    vAxis.maxVel_PPS = std::abs(cmd.targetVel);


    vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;



    vAxis.acc_PPS2 = (cmd.accTime < 0.0001) ? 1e10 : (vAxis.maxVel_PPS / cmd.accTime);


    double f_dec = (cmd.decTime < 0.0) ? cmd.accTime : cmd.decTime;


    vAxis.dec_PPS2 = (f_dec < 0.0001) ? 1e10 : (vAxis.maxVel_PPS / f_dec);

    if (!std::isfinite(vAxis.maxVel_PPS) ||
        !std::isfinite(vAxis.cruiseVel_PPS) ||
        !std::isfinite(vAxis.acc_PPS2) ||
        !std::isfinite(vAxis.dec_PPS2) ||
        vAxis.acc_PPS2 < 0.0 ||
        vAxis.dec_PPS2 < 0.0)
    {
        FailDerivedConsumerGeometry();
        return;
    }

    /*
    RtPrintf(
        "[P1 LOAD] PC:%d | Queue:%d | CurTarget:%d | CurVel:%d\n",
        cmd.sourceLinePC,
        (int)m_Group.cmdQueue.size(),
        (int)cmd.targetPos[0],
        (int)cmd.targetVel);
        */

        // ======================================================
        // 10. CONTINUOUS 智能轉角速度
        // ======================================================
    DiscardStaleQueuedCommands();

    MotionCommand nextCmd{};

    if (m_Group.pathMode == PathMode::CONTINUOUS &&
        TryPeekNextMotionCommand(nextCmd) &&
        GetCommandAuthorizationFailure(nextCmd) ==
        MotionRejectReason::NONE &&
        IsMotionCommandConsumerGeometryValid(nextCmd, m_pContexts))
    {
        const bool identicalAxisMapping =
            MotionCommandsHaveIdenticalAxisMapping(cmd, nextCmd);

        if (!identicalAxisMapping)
        {
            // Axis count/order changes are topological boundaries, not path
            // tangency.  They must reach a true zero-speed junction before
            // LoadNextCommand may retire the outgoing mapping.
            vAxis.targetEndVel = 0.0;
            m_p1LastPreviousAxisMask.store(
                BuildMotionCommandAxisMask(cmd),
                std::memory_order_release);
            m_p1LastNextAxisMask.store(
                BuildMotionCommandAxisMask(nextCmd),
                std::memory_order_release);
        }
        else
        {
            double dot = 0.0;
            bool tangentValid = false;

            if (cmd.mode == InterpolationMode::LINEAR &&
                nextCmd.mode == InterpolationMode::LINEAR)
            {
                // Same ordered mapping: calculate the tangent in the actual
                // N-dimensional command slots.  Single-axis P1 is therefore
                // valid and no stale slot can impersonate X or Y.
                double currentLengthSquared = 0.0;
                double nextLengthSquared = 0.0;
                double dotNumerator = 0.0;
                const int axisCount = ClampMotionAxisCount(cmd.axisCount);
                for (int slot = 0; slot < axisCount; ++slot)
                {
                    const double currentComponent =
                        cmd.targetPos[slot] - m_Group.startPos[slot];
                    const double nextComponent =
                        nextCmd.targetPos[slot] - cmd.targetPos[slot];
                    currentLengthSquared +=
                        currentComponent * currentComponent;
                    nextLengthSquared += nextComponent * nextComponent;
                    dotNumerator += currentComponent * nextComponent;
                }

                if (currentLengthSquared > 1.0e-12 &&
                    nextLengthSquared > 1.0e-12)
                {
                    dot = dotNumerator /
                        std::sqrt(
                            currentLengthSquared * nextLengthSquared);
                    tangentValid = std::isfinite(dot);
                }
            }
            else if (cmd.axisCount == 2 && nextCmd.axisCount == 2)
            {
                // Preserve 2-D line/arc tangent behavior only after exact
                // ordered-axis equivalence. Helical (3-axis) arc junctions
                // remain exact-stop until their full 3-D tangent is proven.
                const auto GetPlanarTangent =
                    [](
                        const MotionCommand& command,
                        double start0,
                        double start1,
                        bool exitTangent,
                        double& tangent0,
                        double& tangent1) noexcept -> bool
                {
                    if (command.mode == InterpolationMode::LINEAR)
                    {
                        tangent0 = command.targetPos[0] - start0;
                        tangent1 = command.targetPos[1] - start1;
                    }
                    else if (command.mode ==
                        InterpolationMode::CIRCULAR_CW ||
                        command.mode ==
                        InterpolationMode::CIRCULAR_CCW)
                    {
                        const double point0 = exitTangent
                            ? command.targetPos[0]
                            : start0;
                        const double point1 = exitTangent
                            ? command.targetPos[1]
                            : start1;
                        const double radius0 =
                            point0 - command.centerPos[0];
                        const double radius1 =
                            point1 - command.centerPos[1];
                        if (command.mode ==
                            InterpolationMode::CIRCULAR_CCW)
                        {
                            tangent0 = -radius1;
                            tangent1 = radius0;
                        }
                        else
                        {
                            tangent0 = radius1;
                            tangent1 = -radius0;
                        }
                    }
                    else
                    {
                        return false;
                    }

                    const double length = std::sqrt(
                        tangent0 * tangent0 + tangent1 * tangent1);
                    if (!std::isfinite(length) || length <= 1.0e-6)
                    {
                        return false;
                    }
                    tangent0 /= length;
                    tangent1 /= length;
                    return true;
                };

                double currentTangent0 = 0.0;
                double currentTangent1 = 0.0;
                double nextTangent0 = 0.0;
                double nextTangent1 = 0.0;
                tangentValid =
                    GetPlanarTangent(
                        cmd,
                        m_Group.startPos[0],
                        m_Group.startPos[1],
                        true,
                        currentTangent0,
                        currentTangent1) &&
                    GetPlanarTangent(
                        nextCmd,
                        cmd.targetPos[0],
                        cmd.targetPos[1],
                        false,
                        nextTangent0,
                        nextTangent1);
                if (tangentValid)
                {
                    dot =
                        currentTangent0 * nextTangent0 +
                        currentTangent1 * nextTangent1;
                }
            }

            if (tangentValid)
            {
                dot = (std::max)(-1.0, (std::min)(1.0, dot));
                const double angleFactor = (1.0 + dot) / 2.0;
                const double nextVelocity = std::abs(
                    nextCmd.targetVel * m_Group.feedrateOverride);
                vAxis.targetEndVel =
                    (std::min)(vAxis.cruiseVel_PPS, nextVelocity) *
                    angleFactor;
            }
            else
            {
                vAxis.targetEndVel = 0.0;
            }
        }
    }
    else
    {
        vAxis.targetEndVel = 0.0;

        /*
        RtPrintf(
            "[P1 NO NEXT] PC:%d | CurTarget:%d | Queue EMPTY\n",
            cmd.sourceLinePC,
            (int)cmd.targetPos[0]);*/
    }


    if (!std::isfinite(vAxis.targetEndVel) ||
        vAxis.targetEndVel < 0.0)
    {
        FailDerivedConsumerGeometry();
        return;
    }

    // ======================================================
    // 11. EXACT_STOP S-Curve Buffer Reset
    // ======================================================
    if (m_Group.pathMode ==
        PathMode::EXACT_STOP)
    {
        if (vAxis.velBuffer.empty())
        {
            vAxis.velBuffer.resize(400);
        }

        vAxis.bufferSum = 0.0;
        vAxis.bufferIndex = 0;

        std::fill(vAxis.velBuffer.begin(), vAxis.velBuffer.end(), 0.0);
    }


    // ======================================================
    // 12. 正式啟動 Virtual Axis
    // ======================================================
    TrackMotionCommandAccepted(
        m_Group.currentCmd);

    vAxis.state = MotionState::MotionState_MOVING;
    vAxis.inPosition = false;
    m_Group.isActive = true;

    TrackMotionCommandStarted(
        m_Group.currentCmd);


    // ======================================================
    // 13. History 幾何快照
    // ======================================================
    if (m_Group.enableHistory)
    {
        for (int i = 0; i < 8; ++i)
        {
            m_Group.currentCmd.mem_startPos[i] = m_Group.startPos[i];
            m_Group.currentCmd.mem_ratio[i] = m_Group.ratio[i];

            m_Group.currentCmd.axisIndices[i] = m_Group.axisIndices[i];
        }


        m_Group.currentCmd.mem_radius = m_Group.radius;
        m_Group.currentCmd.mem_startAngle = m_Group.startAngle;
        m_Group.currentCmd.mem_centerX = m_Group.centerX;
        m_Group.currentCmd.mem_centerY = m_Group.centerY;


        // 3D Geometry
        m_Group.currentCmd.mem_totalDist = vAxis.finalTargetPos;


        m_Group.currentCmd.mem_totalAngle = m_Group.totalAngle;


        // Transform Snapshot
        m_Group.currentCmd.mem_enableTransform = m_Group.enableTransform;


        for (int i = 0; i < 3; ++i)
        {
            m_Group.currentCmd.mem_transformOrigin[i] = m_Group.transformOrigin[i];

            for (int j = 0; j < 3; ++j)
            {
                m_Group.currentCmd.mem_transformMatrix[i][j] = m_Group.transformMatrix[i][j];
            }
        }
    }

    // ======================================================
    // Debug
    // ======================================================
    /*
    RtPrintf(
        "[LOAD] PC:%d "
        "Dist:%d "
        "TgtV:%d "
        "Cruise:%d "
        "EndV:%d "
        "Queue:%d\n",
        cmd.sourceLinePC,
        (int)vAxis.finalTargetPos,
        (int)cmd.targetVel,
        (int)vAxis.cruiseVel_PPS,
        (int)vAxis.targetEndVel,
        (int)m_Group.cmdQueue.size());
    */
}
// 計算指令的方向向量 (Normalized)
void MotionCore::GetDirectionVector(const MotionCommand& cmd, double startX, double startY, double& vx, double& vy) {
    if (cmd.mode == InterpolationMode::LINEAR)
    {
        double dx = cmd.targetPos[0] - startX;
        double dy = cmd.targetPos[1] - startY;
        double len = std::sqrt(dx * dx + dy * dy);
        vx = (len < 1.0) ? 0 : dx / len;
        vy = (len < 1.0) ? 0 : dy / len;
    }
    else
    {
        // 圓弧的出口方向是終點的切線方向 (假設是在平面 0,1)
        double rx = cmd.targetPos[0] - cmd.centerPos[0];
        double ry = cmd.targetPos[1] - cmd.centerPos[1];
        double len = std::sqrt(rx * rx + ry * ry);
        // 切線向量 (CCW: [-y, x], CW: [y, -x])
        if (cmd.mode == InterpolationMode::CIRCULAR_CCW)
        {
            vx = -ry / len; vy = rx / len;
        }
        else
        {
            vx = ry / len; vy = -rx / len;
        }
    }
}

// dir: 1 代表 CCW (逆時針 G03), -1 代表 CW (順時針 G02)
void MotionCore::ArcMove(const std::vector<int>& axes, const std::vector<double>& targetPos, const std::vector<double>& centerPos, int dir, double targetVel, double acc_time, double dec_time, BufferMode mode)
{
    const MotionCommandSource commandSource =
        m_pendingCommandSource.load(
            std::memory_order_acquire);
    const MotionOwnerLease commandOwnerLease =
        GetMotionOwnerLease();
    MotionExecutionEpoch commandEpoch =
        GetCurrentExecutionEpoch();

    MotionCommand invalidCommand{};
    invalidCommand.mode = (dir == 1)
        ? InterpolationMode::CIRCULAR_CCW
        : InterpolationMode::CIRCULAR_CW;
    invalidCommand.sourceLinePC = m_pendingSourcePC;

    if (m_pContexts == nullptr ||
        axes.size() < 2U ||
        axes.size() > 3U ||
        targetPos.size() != axes.size() ||
        centerPos.size() < 2U ||
        !std::isfinite(centerPos[0]) ||
        !std::isfinite(centerPos[1]) ||
        (dir != 1 && dir != -1) ||
        !std::isfinite(targetVel) ||
        !std::isfinite(acc_time) ||
        !std::isfinite(dec_time))
    {
        RejectInvalidProducerMotionCommand(
            invalidCommand,
            commandEpoch,
            commandSource,
            commandOwnerLease);
        return;
    }

    std::array<bool, MAX_AXES> seenAxis{};
    for (std::size_t slot = 0U; slot < axes.size(); ++slot)
    {
        const int axisIndex = axes[slot];
        if (axisIndex < 0 ||
            axisIndex >= MAX_AXES ||
            axisIndex >= static_cast<int>(m_pContexts->size()) ||
            seenAxis[static_cast<std::size_t>(axisIndex)] ||
            !(*m_pContexts)[axisIndex].isExist ||
            !std::isfinite(targetPos[slot]))
        {
            RejectInvalidProducerMotionCommand(
                invalidCommand,
                commandEpoch,
                commandSource,
                commandOwnerLease);
            return;
        }
        seenAxis[static_cast<std::size_t>(axisIndex)] = true;
    }

    // 1. 打包包裹
    MotionCommand cmd{};
    cmd.mode = (dir == 1) ? InterpolationMode::CIRCULAR_CCW : InterpolationMode::CIRCULAR_CW;

    // 🟢 動態打包軸數
    cmd.axisCount = (int)axes.size();
    for (int i = 0; i < cmd.axisCount; ++i)
    {
        cmd.axisIndices[i] = axes[i];
        cmd.targetPos[i] = targetPos[i];
    }

    cmd.centerPos[0] = centerPos[0];
    cmd.centerPos[1] = centerPos[1];
    cmd.dir = dir;
    cmd.targetVel = std::abs(targetVel);
    cmd.accTime = acc_time;
    cmd.decTime = dec_time;

    // 🌟 貼上標籤！記錄這條路徑是來自哪一行 G-Code
    cmd.sourceLinePC = m_pendingSourcePC;
    cmd.sourceWCS = m_pendingSourceWCS; // 🌟 貼上 WCS 標籤！
    // 🌟 貼上刀具標籤！
    cmd.sourceToolLengthMode = m_pendingToolMode;
    cmd.sourceHCode = m_pendingHCode;

    cmd.sourceToolRadiusMode = m_pendingToolRadMode; // 刀徑 G 碼
    cmd.sourceDCode = m_pendingDCode;                // D 碼

    cmd.sourceIsAbsoluteMode = m_pendingIsAbsoluteMode;

    cmd.sourceG68Active = m_pendingG68Active; // 🌟 印上 G68 標籤
    cmd.sourceG68Angle = m_pendingG68Angle; // 🌟 印上角度標籤
    cmd.sourceG168Active = m_pendingG168Active; // 🌟 把 G168 狀態印在包裹上
    cmd.sourceWCode = m_pendingWCode; // 🌟 印上 W 碼標籤

    cmd.sourceG51Active = m_pendingG51Active;
    cmd.sourceScaleRatio = m_pendingScaleRatio; // 🌟 印上縮放標籤

    cmd.sourceMirrorMask = m_pendingMirrorMask; // 🌟 印上鏡像標籤

    cmd.sourceG16Active = m_pendingG16Active; // 🌟 印上極座標標籤

    cmd.sourceG162Active = m_pendingG162Active;
    cmd.sourcePlaneMode = m_pendingPlaneMode;

    // 2. 判斷插隊或排隊
    if (mode == BufferMode::ABORTING)
    {
        commandEpoch = RequestAbortingExecutionEpoch(
            commandSource);

        // Future Queue / History 與 Active Abort 都由 250 us Consumer
        // 在 Epoch 邊界套用。
    }

    // 3. 在最終 Epoch 確定後配置 Identity，再推入佇列。
    AssignExecutionIdentity(
        cmd,
        commandEpoch,
        commandSource,
        commandOwnerLease);
    TryEnqueueMotionCommand(cmd);
}

void MotionCore::StopGroup()
{
    StopGroupImpl(true);
}

void MotionCore::StopGroupImpl(
    bool publishExecutionEpoch)
{
    const bool hadExecutionToInvalidate =
        m_Group.isActive ||
        !m_Group.cmdQueue.empty();

    if (publishExecutionEpoch && hadExecutionToInvalidate)
    {
        BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
    }

    // 尚未開始的 Future Queue 已由新 Epoch 取消；
    // 實際淘汰由 250 us Consumer 執行。

    if (!m_Group.isActive)
    {
        // Reset can arrive after an exact Abort or terminal Group transition.
        // Inactive + IDLE must still be a fully quiescent command state; repair
        // finite legacy residue here before J.5 samples the Reset pre-proof.
        if (m_Group.virtualAxis.state ==
            MotionState::MotionState_IDLE)
        {
            TryCanonicalizeIdleAxisCommandState(
                m_Group.virtualAxis);
        }

        return;
    }

    // =========================================================
    // 🌟 自動計算群組的「最小減速度」 (即尋找最長的減速時間)
    // =========================================================
    double safeDecTime = 0.0;

    if (m_pContexts != nullptr && m_Group.axisCount > 0)
    {
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = m_Group.axisIndices[i];
            AxisContext& axis = (*m_pContexts)[idx];

            // 取最大時間 (牽就煞車最慢、最需要保護的那個軸)
            safeDecTime = (std::max)(safeDecTime, axis.Stop_dec_time);
        }
    }

    // 防呆保護：避免參數沒設導致除以零崩潰，給予最低安全值 0.2 秒
    if (safeDecTime < 0.001) {
        safeDecTime = 0.2;
    }

    // 將算出的最安全時間，交給虛擬主軸執行平滑煞車
    StopMove(m_Group.virtualAxis, safeDecTime);
}

void MotionCore::EmergencyStopGroup()
{
    // Group E-Stop 也必須使先前 Owner Lease 失效。
    TakeSafetyMotionOwner();

    const bool hadExecutionToInvalidate =
        m_Group.isActive ||
        !m_Group.cmdQueue.empty();

    if (hadExecutionToInvalidate)
    {
        BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
    }

    // Future Queue 已由新 Epoch 取消，等待 250 us Consumer 淘汰。

    // 1. 關閉群組插補引擎，防止 UpdateInterpolation 繼續寫入位置
    m_Group.isActive = false;

    // 2. 徹底殺掉虛擬主軸 (清空 Buffer 是關鍵)
    EmergencyStop(m_Group.virtualAxis);

    // 3. 遍歷所有實體軸執行急停
    for (int i = 0; i < m_Group.axisCount; ++i)
    {
        int idx = m_Group.axisIndices[i];
        AxisContext& realAxis = (*m_pContexts)[idx];

        // 呼叫我們先前寫好的單軸 EmergencyStop
        EmergencyStop(realAxis);
    }

    // RtPrintf(">>> [ALARM] EmergencyStopGroup Executed. All motions killed.\n");
}
void MotionCore::SetGroupFeedrateOverride(double overrideRatio)
{
    // 1. 防呆檢查 (限制在 0.0 到 1.2 之間，最高允許 120% 超頻)
    // 如果你要做 EDM 退刀，這裡可以允許負數 (如 -0.5)，否則先鎖在 >= 0

    // 2. 把倍率寫進群組與虛擬主軸
    m_Group.feedrateOverride = overrideRatio;
    m_Group.virtualAxis.feedrateOverride = overrideRatio;
}
void MotionCore::SetGroupPathMode(PathMode mode)
{
    m_Group.pathMode = mode;

    // 如果切換回 EXACT_STOP (G61)，為了安全，我們確保虛擬主軸的目標終點速度立刻歸零
    if (mode == PathMode::EXACT_STOP) {
        m_Group.virtualAxis.targetEndVel = 0.0;
    }

    // RtPrintf("[INFO] Path Mode Switched to: %s\n", 

}

void MotionCore::UpdatePathServoVelocity(double velocity_pps)
{
    // 這裡的速度由外部感測器計算後傳入
    m_Group.pathServoVel = velocity_pps;
    //RtPrintf("UpdatePathServoVelocity >>> %d\n", (int)velocity_pps);
}

void MotionCore::EnableHistoryBuffer(bool enable)
{
    // 1. 切換群組內的時光機開關
    m_Group.enableHistory = enable;

    // 2. 如果使用者決定「關閉」時光機，順手把歷史垃圾清掉，釋放記憶體
    if (!enable)
    {
        m_Group.historyQueue.clear();
    }

    // RtPrintf("[INFO] History Buffer (Time Machine) is now: %s\n", enable ? "ON" : "OFF");
}

// =================================================================
// 🟢 [新增] 空間座標旋轉設定 (G68)
// =================================================================
void MotionCore::SetCoordinateTransform(bool enable, double ox, double oy, double oz, double yaw_deg, double pitch_deg, double roll_deg)
{
    m_Group.enableTransform = enable;
    if (!enable) return;

    m_Group.transformOrigin[0] = ox;
    m_Group.transformOrigin[1] = oy;
    m_Group.transformOrigin[2] = oz;

    // 將角度轉為弧度 (Radian)
    double a = yaw_deg * (3.14159265359 / 180.0);   // 繞 Z 軸 (Yaw)
    double b = pitch_deg * (3.14159265359 / 180.0); // 繞 Y 軸 (Pitch)
    double c = roll_deg * (3.14159265359 / 180.0);  // 繞 X 軸 (Roll)

    // 計算預先準備的 sin 與 cos
    double ca = std::cos(a), sa = std::sin(a);
    double cb = std::cos(b), sb = std::sin(b);
    double cc = std::cos(c), sc = std::sin(c);

    // 產生 3D 空間的合成旋轉矩陣 R = Rz(a) * Ry(b) * Rx(c)
    m_Group.transformMatrix[0][0] = ca * cb;
    m_Group.transformMatrix[0][1] = ca * sb * sc - sa * cc;
    m_Group.transformMatrix[0][2] = ca * sb * cc + sa * sc;

    m_Group.transformMatrix[1][0] = sa * cb;
    m_Group.transformMatrix[1][1] = sa * sb * sc + ca * cc;
    m_Group.transformMatrix[1][2] = sa * sb * cc - ca * sc;

    m_Group.transformMatrix[2][0] = -sb;
    m_Group.transformMatrix[2][1] = cb * sc;
    m_Group.transformMatrix[2][2] = cb * cc;
}

PathMode MotionCore::GetGroupPathMode() const
{
    return m_Group.pathMode;
}





void MotionCore::TriggerPathJump(JumpMode mode, const std::vector<JumpSegment>& retract, const std::vector<JumpSegment>& approach, double dwellTime_ms, int b1_axis)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

    // 🟢 儲存模式與專用參數
    m_Group.jumpManager.mode = mode;
    m_Group.jumpManager.b1_AxisIndex = b1_axis;




    // 🟢 [修復 1] 宣告 jm 參照，解決 E0020 "jm 未定義"
    PathJumpManager& jm = m_Group.jumpManager;


    // 🌟 [修正 1]：先清空舊任務，確保沒有殘留
    jm.retractSteps.clear();
    jm.approachSteps.clear();

    // 🌟 [修正 2]：手動逐一拷貝，不要直接用 = 賦值 (防止 RTX64 vector 記憶體坑)
    for (const auto& s : retract) jm.retractSteps.push_back(s);
    for (const auto& s : approach) jm.approachSteps.push_back(s);

    // =========================================================
    // 🌟 [神級修復]：在做任何事之前，先拍下現在所有實體軸的絕對位置！
    // 沒有這一步，後面的數學全部都會算錯導致瞬移！
    // =========================================================
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[i].logicalCmdPos;
    }
    // =========================================================

    // 儲存模式與專用參數
    jm.mode = mode;





    // =========================================================
    // 🟢 [新增] B0 模式：拍下瞬間車頭方向的「快照」！
    // =========================================================
    if (mode == JumpMode::B0_REVERSE && m_pContexts != nullptr && m_Group.axisCount >= 2)
    {
        int idxX = m_Group.axisIndices[0];
        int idxY = m_Group.axisIndices[1];
        int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

        double vx = (*m_pContexts)[idxX].logicalCmdVel;
        double vy = (*m_pContexts)[idxY].logicalCmdVel;
        double vz = (idxZ != -1) ? (*m_pContexts)[idxZ].logicalCmdVel : 0.0;

        // 計算向量長度
        double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);

        // 防呆：如果剛好靜止不動，預設往 Z 軸正向退
        if (v_norm > 1e-6)
        {
            m_Group.jumpManager.b0_Vector[0] = -vx / v_norm; // 掛負號！反向！
            m_Group.jumpManager.b0_Vector[1] = -vy / v_norm;
            m_Group.jumpManager.b0_Vector[2] = -vz / v_norm;
        }
        else {
            m_Group.jumpManager.b0_Vector[0] = 0.0;
            m_Group.jumpManager.b0_Vector[1] = 0.0;
            m_Group.jumpManager.b0_Vector[2] = 1.0;
        }
    }



    // 裝填任務
    m_Group.jumpManager.retractSteps = retract;
    m_Group.jumpManager.approachSteps = approach;
    m_Group.jumpManager.dwellTimeTarget = dwellTime_ms;

    // 拍下快照
    m_Group.jumpManager.triggerPos = m_Group.virtualAxis.currentCmdPos;
    m_Group.jumpManager.currentOffset = 0.0;
    m_Group.jumpManager.jumpVel = 0.0;
    m_Group.jumpManager.currentStepIdx = 0;





    // =========================================================
    // 🟢 [新增] 徹底洗除大腦的 S-Curve 時光記憶！
    // 這樣跳刀結束接回 PATH_SERVO 時，才不會把剛剛的高速噴出來(紫色的尖刺)
    // =========================================================
    m_Group.virtualAxis.currentCmdVel = 0.0;
    m_Group.virtualAxis.planningPos = m_Group.virtualAxis.currentCmdPos;
    if (!m_Group.virtualAxis.velBuffer.empty()) {
        std::fill(m_Group.virtualAxis.velBuffer.begin(), m_Group.virtualAxis.velBuffer.end(), 0.0);
    }
    m_Group.virtualAxis.bufferSum = 0.0;
    // =========================================================


    if (!retract.empty()) {
        m_Group.jumpManager.targetOffset = -retract[0].distance;
        m_Group.jumpManager.state = JumpState::RETRACTING;
        m_Group.pathMode = PathMode::JUMP_TRACKING;
    }
}

void MotionCore::TriggerCenterJump_B3(
    double cx, double cy, double cz, double vx, double vy, double vz,
    const std::vector<JumpSegment>& toCenter, const std::vector<JumpSegment>& toApex,
    const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece,
    double dwellTime_ms)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    jm.mode = JumpMode::B3_CENTER;

    // 1. 拍下放電點快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // ==========================================================
  // 🌟 核心關鍵：拍下路徑進度快照 (這就是它的家！)
  // ==========================================================
    AxisContext& vAxis = m_Group.virtualAxis;
    jm.triggerPos = vAxis.currentCmdPos;
    // ==========================================================

    // 2. 存下安全中心點與 3D 拔高向量
    jm.b3_centerPos[0] = cx; jm.b3_centerPos[1] = cy; jm.b3_centerPos[2] = cz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b3_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b3_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b3_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 3. 算出【真實的橫移距離】
    double dx = cx - jm.frozenPos[0];
    double dy = cy - jm.frozenPos[1];
    double dz = cz - jm.frozenPos[2];
    jm.b3_distToCenter = std::sqrt(dx * dx + dy * dy + dz * dz);

    // ========================================================
    // 🌟 [神級等比例平帳]：把使用者的腳本，無縫縮放成真實距離！
    // ========================================================
    jm.b3_toCenterSteps = toCenter;
    jm.b3_toWorkpieceSteps = toWorkpiece;

    // 處理去程：把腳本距離縮放成 b3_distToCenter
    double sum1 = 0; for (auto& s : jm.b3_toCenterSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b3_toCenterSteps) s.distance = s.distance * (jm.b3_distToCenter / sum1);
    }
    else if (!jm.b3_toCenterSteps.empty()) {
        jm.b3_toCenterSteps[0].distance = jm.b3_distToCenter; // 防呆
    }

    // 處理回程：同樣必須等比例縮放成 b3_distToCenter
    double sum4 = 0; for (auto& s : jm.b3_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b3_toWorkpieceSteps) s.distance = s.distance * (jm.b3_distToCenter / sum4);
    }
    else if (!jm.b3_toWorkpieceSteps.empty()) {
        jm.b3_toWorkpieceSteps[0].distance = jm.b3_distToCenter;
    }

    // =====================================================================
    // 🟢 [數學驗證 LOG 1：觸發瞬間的幾何計算]
    // =====================================================================
    /*
    RtPrintf("[MATH_TRIGGER] cx:%d, cy:%d, frozenX:%d, frozenY:%d\n",
        (int)(cx * 1000.0), (int)(cy * 1000.0),
        (int)(jm.frozenPos[0] * 1000.0), (int)(jm.frozenPos[1] * 1000.0));

    RtPrintf("[MATH_TRIGGER] dx:%d, dy:%d, dz:%d => distToCenter:%d\n",
        (int)((cx - jm.frozenPos[0]) * 1000.0),
        (int)((cy - jm.frozenPos[1]) * 1000.0),
        (int)((cz - jm.frozenPos[2]) * 1000.0),
        (int)(jm.b3_distToCenter * 1000.0));

    if (!jm.b3_toWorkpieceSteps.empty()) {
        RtPrintf("[MATH_TRIGGER] sum4:%d, Step4_Dist1:%d, Step4_Dist2:%d\n",
            (int)(sum4 * 1000.0),
            (int)(jm.b3_toWorkpieceSteps[0].distance * 1000.0),
            jm.b3_toWorkpieceSteps.size() > 1 ? (int)(jm.b3_toWorkpieceSteps[1].distance * 1000.0) : 0);
    }*/
    // =====================================================================

    // 4. 裝填拔高/降落腳本
    jm.b3_toApexSteps = toApex;
    jm.b3_fromApexSteps = fromApex;
    jm.b3_distToApex = 0;
    for (auto& s : jm.b3_toApexSteps) jm.b3_distToApex += s.distance; // 算出拔高總長

    // 5. 啟動 B3 狀態機 (進入第一段)
    jm.dwellTimeTarget = dwellTime_ms;
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B3_TO_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 清空原本軌跡速度
    m_Group.virtualAxis.currentCmdVel = 0.0;
}


void MotionCore::TriggerOrbitalJump_B4(
    double upperCx, double upperCy, double upperCz,
    double vx, double vy, double vz,
    const std::vector<JumpSegment>& toUpper,
    const std::vector<JumpSegment>& toApex,
    const std::vector<JumpSegment>& fromApex,
    const std::vector<JumpSegment>& toWorkpiece,
    double dwellTime_ms)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;
    jm.mode = JumpMode::B4_ORBITAL_DIAGONAL;

    // 1. 拍下實體軸「放電點」快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 🌟 2. 拍下路徑進度快照 (這就是之前查很久的家！)
    jm.triggerPos = vAxis.currentCmdPos;

    // 3. 儲存上中心點與拔高向量
    jm.b4_upperCenterPos[0] = upperCx;
    jm.b4_upperCenterPos[1] = upperCy;
    jm.b4_upperCenterPos[2] = upperCz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b4_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b4_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b4_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 4. 算出【斜向真實距離】(放電點 -> 上中心)
    double dx = upperCx - jm.frozenPos[0];
    double dy = upperCy - jm.frozenPos[1];
    double dz = upperCz - jm.frozenPos[2];
    jm.b4_distToUpper = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 5. 絕對安全深拷貝腳本 (防止記憶體坑)
    jm.b4_toUpperSteps.clear();
    for (const auto& s : toUpper) jm.b4_toUpperSteps.push_back(s);
    jm.b4_toApexSteps.clear();
    for (const auto& s : toApex) jm.b4_toApexSteps.push_back(s);
    jm.b4_fromApexSteps.clear();
    for (const auto& s : fromApex) jm.b4_fromApexSteps.push_back(s);
    jm.b4_toWorkpieceSteps.clear();
    for (const auto& s : toWorkpiece) jm.b4_toWorkpieceSteps.push_back(s);

    // 6. 等比例平帳 (縮放去程與回程腳本距離)
    double sum1 = 0; for (auto& s : jm.b4_toUpperSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b4_toUpperSteps) s.distance = s.distance * (jm.b4_distToUpper / sum1);
    }
    else if (!jm.b4_toUpperSteps.empty()) {
        jm.b4_toUpperSteps[0].distance = jm.b4_distToUpper;
    }

    double sum4 = 0; for (auto& s : jm.b4_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b4_toWorkpieceSteps) s.distance = s.distance * (jm.b4_distToUpper / sum4);
    }
    else if (!jm.b4_toWorkpieceSteps.empty()) {
        jm.b4_toWorkpieceSteps[0].distance = jm.b4_distToUpper;
    }

    // 7. 算出拔高總長
    jm.b4_distToApex = 0;
    for (auto& s : jm.b4_toApexSteps) jm.b4_distToApex += s.distance;

    // 8. 啟動狀態機
    jm.dwellTimeTarget = dwellTime_ms;
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B4_TO_UPPER_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 清空原本軌跡速度
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::FinalizeSafePath(const std::vector<JumpSegment>& retract, std::vector<JumpSegment>& approach)//排渣跳躍腳本安全保護
{
    if (retract.empty() || approach.empty()) return;

    // 1. 算出這趟去程「總負債」(Total Debt)
    double totalOutbound = 0;
    for (const auto& s : retract) totalOutbound += s.distance;

    // 2. 遍歷進刀段落進行動態平帳
    double currentReturnSum = 0;
    size_t count = approach.size();

    if (count == 1)
    {
        approach[0].distance = totalOutbound;
    }
    else
    {
        double total_Back_dir = 0;

        for (size_t i = 0; i < count; ++i)
        {
            total_Back_dir += approach[i].distance;
        }

        //暫時不寫 顯下斷點防止暴衝
        double Sum = totalOutbound - total_Back_dir;
        if (Sum == 0)
        {

        }
        else if (Sum > 0)
        {
            approach[0].distance += Sum;
        }
        else
        {
            if (approach[0].distance < std::abs(Sum))
            {
                //通常不會
            }
            else
            {
                approach[0].distance -= Sum;
            }

        }
    }



    // 3. 再次檢查：如果債務已經還完但還有多餘段落，將多餘段落距離設為 0
    // (這對應你說的「刪除或修改中間段落導致變長」的情況)
}


// 🌟 1. 觸發 B0 暫停 (一次給齊兩個腳本)
void MotionCore::TriggerPause_B0(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B0_REVERSE;

    // 🌟 這是讓機台停在空中的「絕對關鍵」！沒有這行，它就會直接降落！
    jm.isPauseMode = true;

    jm.triggerPos = vAxis.currentCmdPos;

    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 拍下反向向量
    int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
    int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;
    double vx = (*m_pContexts)[idxX].logicalCmdVel;
    double vy = (*m_pContexts)[idxY].logicalCmdVel;
    double vz = (idxZ != -1) ? (*m_pContexts)[idxZ].logicalCmdVel : 0.0;
    double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (v_norm > 1e-6) {
        jm.b0_Vector[0] = -vx / v_norm; jm.b0_Vector[1] = -vy / v_norm; jm.b0_Vector[2] = -vz / v_norm;
    }
    else {
        jm.b0_Vector[0] = 0.0; jm.b0_Vector[1] = 0.0; jm.b0_Vector[2] = 1.0;
    }

    // 🌟 一次把退刀與進刀腳本都載入大腦
    jm.retractSteps = retractScript;
    jm.approachSteps = approachScript;

    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::RETRACTING;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
    vAxis.currentCmdVel = 0.0;
}
void MotionCore::TriggerPause_B1(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript, int axisIndex, double dir)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B1_SPECIFIC_AXIS;
    jm.isPauseMode = true; // 🌟 這是讓它停在空中的護身符！

    jm.triggerPos = vAxis.currentCmdPos;

    // 拍下放電點快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 🌟 B1 專屬設定：指定要跳哪一軸，以及方向 (+1.0 或 -1.0)
    jm.b1_AxisIndex = axisIndex;
    jm.b1_dir = (dir >= 0.0) ? 1.0 : -1.0;

    // 載入退刀與降落腳本
    jm.retractSteps = retractScript;
    jm.approachSteps = approachScript;

    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::RETRACTING;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::TriggerPause_B2(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;



    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B2_PATH_REVERSE;
    jm.isPauseMode = true; // 🌟 空中懸停護身符

    jm.triggerPos = vAxis.currentCmdPos;

    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    jm.retractSteps = retractScript;


    jm.approachSteps = approachScript;

    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = vAxis.currentCmdVel;
    jm.jumpVel = m_Group.virtualAxis.currentCmdVel;
    jm.state = JumpState::RETRACTING;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
}
void MotionCore::Process_B2_Approach_Planner(double dt)
{
    PathJumpManager& jm = m_Group.jumpManager;
    JumpSegment& seg = jm.approachSteps[jm.currentStepIdx];
    double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
    double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

    // =========================================================
    // 🌟 1. 無限前瞻 (Look-Ahead)：尋找未來的「真正轉角」
    // =========================================================
    double accumulatedDist = m_Group.currentCmd.mem_totalDist;
    bool needToStop = false;

    DiscardStaleQueuedCommands();

    if (!m_Group.cmdQueue.empty())
    {
        double curRatio[8];
        for (int i = 0; i < m_Group.axisCount; i++) curRatio[i] = m_Group.ratio[i];
        InterpolationMode curMode = m_Group.mode;

        // Stage NC-0.1C：以 Consumer Snapshot 逐筆查看 Replay + Ingress。
        // 不暴露 Ring Slot，也不讓 RT Runtime 使用 deque iterator。
        const std::size_t queuedCommandCount =
            m_Group.cmdQueue.size();

        for (std::size_t queueOffset = 0U;
            queueOffset < queuedCommandCount;
            ++queueOffset)
        {
            MotionCommand futureCommand{};

            if (!TryPeekQueuedMotionCommandAt(
                queueOffset,
                futureCommand))
            {
                break;
            }

            if (GetCommandAuthorizationFailure(
                futureCommand) != MotionRejectReason::NONE)
            {
                break;
            }

            bool isTurn = false;

            // 條件 A：模式改變 (直線變圓弧，或圓弧變直線)，絕對要煞車
            if (curMode != futureCommand.mode) {
                isTurn = true;
            }
            // 條件 B：都是直線，用內積檢查折角
            else if (curMode == InterpolationMode::LINEAR)
            {
                double dotProduct = 0.0, len1 = 0.0, len2 = 0.0;
                for (int a = 0; a < m_Group.axisCount; a++) {
                    double cR = curRatio[a];
                    double nR = futureCommand.mem_ratio[a];
                    dotProduct += cR * nR;
                    len1 += cR * cR;
                    len2 += nR * nR;
                }
                if (len1 > 0.01 && len2 > 0.01) {
                    double cosTheta = dotProduct / (std::sqrt(len1) * std::sqrt(len2));
                    if (cosTheta < 0.999) isTurn = true; // 夾角大於 2.5 度就算轉彎
                }
            }
            // 條件 C：連續圓弧，安全起見當作轉折煞車
            else {
                isTurn = true;
            }

            if (isTurn) {
                needToStop = true;
                break; // 找到轉角了！紅燈線就畫在這！
            }

            // 如果是直走，累加距離
            accumulatedDist += futureCommand.mem_totalDist;

            // 更新比較基準，繼續看下一段
            curMode = futureCommand.mode;
            for (int a = 0; a < m_Group.axisCount; a++) curRatio[a] = futureCommand.mem_ratio[a];
        }
    }

    // 如果掃到底都沒轉彎，代表一路直通放電原點！
    if (!needToStop) {
        needToStop = true;
        accumulatedDist = jm.triggerPos; // 這樣 targetPhysicalOffset 才會是 0.0 (原點)
    }

    // =========================================================
    // 🌟 2. 計算出絕對精準的物理紅燈線
    // =========================================================
    double targetPhysicalOffset = accumulatedDist - jm.triggerPos;

    // =========================================================
    // 🌟 3. 呼叫規劃器 
    // =========================================================
    double plannerTarget = targetPhysicalOffset + 0.1;
    jm.currentOffset = PlanTrapezoidal_B2(jm.currentOffset, plannerTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

    // =========================================================
    // 🌟 4. 終極護城河：卡死時光機
    // =========================================================
    if (needToStop && jm.currentOffset >= targetPhysicalOffset - 1.0)
    {
        if (std::abs(jm.jumpVel) > 50000.0) {
            jm.currentOffset = targetPhysicalOffset - 1.0;
        }
    }

    // =========================================================
    // 🌟 5. 速度劇本切換 (這才是真正的速度換檔！)
    // =========================================================
    // 算出「當前腳本段落」的換檔邊界。
    // 因為進刀的最終終點是 0.0，所以這一段的邊界，就是「後面所有段落距離總和的負數」
    double speedStepBoundary = 0.0;
    for (int i = jm.currentStepIdx + 1; i < (int)jm.approachSteps.size(); i++) {
        speedStepBoundary -= jm.approachSteps[i].distance;
    }

    // 如果當前進度 (currentOffset) 越過了邊界，就切換下一段速度！
    if (jm.currentOffset >= speedStepBoundary && jm.currentStepIdx < (int)jm.approachSteps.size() - 1) {
        jm.currentStepIdx++;
        //RtPrintf("[B2_SPEED_SHIFT] Switched to Step %d! Boundary: %d\n", jm.currentStepIdx, (int)speedStepBoundary);
    }
}


void MotionCore::TriggerPause_B3(double cx, double cy, double cz, double vx, double vy, double vz, const std::vector<JumpSegment>& toCenter, const std::vector<JumpSegment>& toApex, const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B3_CENTER;
    jm.isPauseMode = true; // 🌟 關鍵標記：這是一次暫停，不是排渣

    // 1. 拍下放電點與路徑進度快照 (暫停的起點)
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }
    jm.triggerPos = vAxis.currentCmdPos;

    // 2. 存下安全中心點與 3D 拔高向量
    jm.b3_centerPos[0] = cx; jm.b3_centerPos[1] = cy; jm.b3_centerPos[2] = cz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b3_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b3_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b3_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 3. 算出【真實的橫移距離】與【拔高總距離】
    double dx = cx - jm.frozenPos[0];
    double dy = cy - jm.frozenPos[1];
    double dz = cz - jm.frozenPos[2];
    jm.b3_distToCenter = std::sqrt(dx * dx + dy * dy + dz * dz);

    jm.b3_toApexSteps = toApex;
    jm.b3_fromApexSteps = fromApex;
    jm.b3_distToApex = 0;
    for (const auto& s : jm.b3_toApexSteps) jm.b3_distToApex += s.distance;

    // ========================================================
    // 🌟 [神級等比例平帳]：針對「暫停」重新縮放腳本
    // ========================================================
    jm.b3_toCenterSteps = toCenter;
    jm.b3_toWorkpieceSteps = toWorkpiece;

    // 處理第一階段 (To Center) 平帳
    double sum1 = 0; for (auto& s : jm.b3_toCenterSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b3_toCenterSteps) s.distance *= (jm.b3_distToCenter / sum1);
    }
    else if (!jm.b3_toCenterSteps.empty()) {
        jm.b3_toCenterSteps[0].distance = jm.b3_distToCenter;
    }

    // 處理第五階段 (To Workpiece) 平帳
    double sum4 = 0; for (auto& s : jm.b3_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b3_toWorkpieceSteps) s.distance *= (jm.b3_distToCenter / sum4);
    }
    else if (!jm.b3_toWorkpieceSteps.empty()) {
        jm.b3_toWorkpieceSteps[0].distance = jm.b3_distToCenter;
    }

    // 🌟 數學驗證 LOG：確保暫停幾何正確
    /*
    RtPrintf("[PAUSE_B3_MATH] DistCenter:%d, DistApex:%d, TriggerPos:%d\n",
        (int)(jm.b3_distToCenter * 1000.0), (int)(jm.b3_distToApex * 1000.0), (int)jm.triggerPos);*/

        // 4. 啟動 B3 狀態機
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B3_TO_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 凍結加工速度
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::TriggerPause_B4(double upperCx, double upperCy, double upperCz, double vx, double vy, double vz, const std::vector<JumpSegment>& toUpper, const std::vector<JumpSegment>& toApex, const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B4_ORBITAL_DIAGONAL;
    jm.isPauseMode = true; // 🌟 關鍵標記：這是一次暫停

    // 1. 拍下實體軸「放電點」快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 2. 拍下路徑進度快照 (鎖定生命線)
    jm.triggerPos = vAxis.currentCmdPos;

    // 3. 儲存上中心點與 3D 拔高向量
    jm.b4_upperCenterPos[0] = upperCx;
    jm.b4_upperCenterPos[1] = upperCy;
    jm.b4_upperCenterPos[2] = upperCz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b4_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b4_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b4_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 4. 算出【斜向真實距離】(放電點 -> 上中心)
    double dx = upperCx - jm.frozenPos[0];
    double dy = upperCy - jm.frozenPos[1];
    double dz = upperCz - jm.frozenPos[2];
    jm.b4_distToUpper = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 5. 載入腳本
    jm.b4_toApexSteps = toApex;
    jm.b4_fromApexSteps = fromApex;
    jm.b4_distToApex = 0;
    for (const auto& s : jm.b4_toApexSteps) jm.b4_distToApex += s.distance;

    // 🌟 6. [神級等比例平帳] 處理去程與回程
    jm.b4_toUpperSteps = toUpper;
    jm.b4_toWorkpieceSteps = toWorkpiece;

    double sum1 = 0; for (auto& s : jm.b4_toUpperSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b4_toUpperSteps) s.distance *= (jm.b4_distToUpper / sum1);
    }
    else if (!jm.b4_toUpperSteps.empty()) {
        jm.b4_toUpperSteps[0].distance = jm.b4_distToUpper;
    }

    double sum4 = 0; for (auto& s : jm.b4_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b4_toWorkpieceSteps) s.distance *= (jm.b4_distToUpper / sum4);
    }
    else if (!jm.b4_toWorkpieceSteps.empty()) {
        jm.b4_toWorkpieceSteps[0].distance = jm.b4_distToUpper;
    }
    /*
    RtPrintf("[PAUSE_B4] Triggered! DistToUpper:%d, DistToApex:%d\n",
        (int)(jm.b4_distToUpper * 1000.0), (int)(jm.b4_distToApex * 1000.0));
        */

        // 7. 啟動狀態機
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B4_TO_UPPER_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 清空原本軌跡速度
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::Process_Forward_Crossing()
{
    AxisContext& vAxis = m_Group.virtualAxis;
    PathJumpManager& jm = m_Group.jumpManager;

    // 檢查有沒有同一 Epoch 的下一段，沒有就不需要換檔。
    DiscardStaleQueuedCommands();

    if (m_Group.cmdQueue.empty()) return;

    MotionCommand frontCommand{};
    if (!TryPeekNextMotionCommand(frontCommand) ||
        GetCommandAuthorizationFailure(frontCommand) !=
        MotionRejectReason::NONE)
    {
        return;
    }

    const bool forwardMappingSafe =
        IsMotionCommandConsumerGeometryValid(
            m_Group.currentCmd,
            m_pContexts) &&
        IsMotionCommandConsumerGeometryValid(
            frontCommand,
            m_pContexts) &&
        IsMotionCommandHistorySnapshotValid(m_Group.currentCmd) &&
        IsMotionCommandHistorySnapshotValid(frontCommand) &&
        MotionCommandsHaveIdenticalAxisMapping(
            m_Group.currentCmd,
            frontCommand);
    if (!forwardMappingSafe)
    {
        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastPreviousAxisMask.store(
            BuildMotionCommandAxisMask(m_Group.currentCmd),
            std::memory_order_release);
        m_p1LastNextAxisMask.store(
            BuildMotionCommandAxisMask(frontCommand),
            std::memory_order_release);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        if (!HasPendingExecutionEpochChange())
        {
            TriggerGroupMappingIntegrityEmergencyStop(-1);
        }
        return;
    }

    double lastDist = m_Group.currentCmd.mem_totalDist;

    // ==========================================
    // 🌟 核心 A：先取得下一個包裹
    //
    // Queue 理論上不會在 Consumer 端突然消失；仍先完成 Pop，
    // 確保失敗時不會提前改動 startPos 或 History。
    // ==========================================
    MotionCommand nextCommand{};

    if (HasPendingExecutionEpochChange() ||
        !m_Group.cmdQueue.ConsumerTryPop(nextCommand))
    {
        return;
    }

    const bool exactPeekedIdentity =
        MotionExecutionIdentityExactlyMatches(
            frontCommand.execution,
            nextCommand.execution) &&
        frontCommand.ownerLease.owner == nextCommand.ownerLease.owner &&
        frontCommand.ownerLease.generation ==
        nextCommand.ownerLease.generation;

    // Epoch 或 Owner Lease 可能在 Pop 後、切換 Current Command 前改變。
    const MotionRejectReason postPopAuthorizationFailure =
        GetCommandAuthorizationFailure(nextCommand);

    if (postPopAuthorizationFailure != MotionRejectReason::NONE)
    {
        const bool trackedTransportCopy =
            IsTerminalizedReplayOrTrackedCommand(nextCommand);
        if (!trackedTransportCopy &&
            postPopAuthorizationFailure == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else if (!trackedTransportCopy)
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        if (!trackedTransportCopy)
        {
            m_lastRejectedSegmentId.store(
                nextCommand.execution.segmentId,
                std::memory_order_relaxed);
        }

        RejectMotionCommand(
            nextCommand,
            postPopAuthorizationFailure,
            0U);

        return;
    }

    if (!exactPeekedIdentity ||
        !IsMotionCommandConsumerGeometryValid(nextCommand, m_pContexts) ||
        !IsMotionCommandHistorySnapshotValid(nextCommand) ||
        !MotionCommandsHaveIdenticalAxisMapping(
            m_Group.currentCmd,
            nextCommand))
    {
        m_p1DroppedAxisRetirementFailureCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastPreviousAxisMask.store(
            BuildMotionCommandAxisMask(m_Group.currentCmd),
            std::memory_order_release);
        m_p1LastNextAxisMask.store(
            BuildMotionCommandAxisMask(nextCommand),
            std::memory_order_release);
        m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
        if (!HasPendingExecutionEpochChange())
        {
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        }
        RejectMotionCommand(
            nextCommand,
            MotionRejectReason::INVALID_GEOMETRY,
            static_cast<std::uint32_t>(
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
        return;
    }

    const MotionRejectReason preCrossAuthorizationFailure =
        GetCommandAuthorizationFailure(nextCommand);
    if (HasPendingExecutionEpochChange() ||
        preCrossAuthorizationFailure != MotionRejectReason::NONE)
    {
        const MotionRejectReason terminalReason =
            preCrossAuthorizationFailure == MotionRejectReason::NONE
            ? MotionRejectReason::STALE_EPOCH
            : preCrossAuthorizationFailure;
        const bool trackedTransportCopy =
            IsTerminalizedReplayOrTrackedCommand(nextCommand);
        if (!trackedTransportCopy &&
            terminalReason == MotionRejectReason::STALE_EPOCH)
        {
            m_staleCommandDiscardCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else if (!trackedTransportCopy)
        {
            m_motionOwnerConflictRejectCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        RejectMotionCommand(nextCommand, terminalReason, 0U);
        return;
    }

    LifecycleCommitReservationGuard lifecycleCommit(
        *this,
        m_Group.currentCmd.execution);
    if (!lifecycleCommit.IsAcquired())
    {
        const MotionRejectReason lifecycleFailure =
            GetCommandAuthorizationFailure(nextCommand);
        RejectMotionCommand(
            nextCommand,
            lifecycleFailure == MotionRejectReason::NONE
            ? MotionRejectReason::STALE_EPOCH
            : lifecycleFailure,
            0U);
        return;
    }

    // ==========================================
    // 🌟 核心 B：物理座標補償 (身體跨步)
    // ==========================================
    for (int i = 0; i < m_Group.axisCount; i++) {
        m_Group.startPos[i] += (lastDist * m_Group.ratio[i]);
    }

    // ==========================================
    // 🌟 核心 C：保存舊包裹並切換
    // ==========================================
    m_Group.historyQueue.push_back(m_Group.currentCmd);

    if (m_Group.historyQueue.size() >
        MOTION_COMMAND_HISTORY_LIMIT)
    {
        m_Group.historyQueue.pop_front();
    }

    m_Group.currentCmd = nextCommand;

    // ==========================================
    // 🌟 核心 D：虛擬進度同步縮放 (維持連續性)
    // ==========================================
    vAxis.currentCmdPos -= lastDist;
    if (jm.state != JumpState::IDLE) {
        jm.triggerPos -= lastDist; // B2 生命線平移
    }

    // ==========================================
    // 🌟 核心 E：大腦索引同步 (通知 Planner 換下一段)
    // ==========================================
    /*
    if (jm.state == JumpState::APPROACHING && jm.mode == JumpMode::B2_PATH_REVERSE) {
        if (jm.currentStepIdx < (int)jm.approachSteps.size() - 1) {
            jm.currentStepIdx++;
        }
    }*/

    // ==========================================
    // 🌟 核心 F：更新方向盤 (Ratio)
    // ==========================================
    m_Group.mode = m_Group.currentCmd.mode;
    for (int i = 0; i < 8; i++) {
        m_Group.ratio[i] = m_Group.currentCmd.mem_ratio[i];
    }

    // 換檔成功 LOG
    //RtPrintf("[NORMAL_CROSS] Sync Idx to:%d | vVel:%d\n", jm.currentStepIdx, (int)jm.jumpVel);
}


// 🌟 2. 觸發復歸 (只需給對齊模式和速度)
void MotionCore::TriggerPauseResume(int alignMode, double alignVel, int firstStageMask)
{
    if (m_Group.jumpManager.state != JumpState::PAUSED_HOLD) return;

    PathJumpManager& jm = m_Group.jumpManager;




    // =========================================================
    // 🌟 [神級自動化]：B2 專屬 - 鏡像翻轉並裁剪劇本
    // =========================================================
    if (jm.mode == JumpMode::B2_PATH_REVERSE)
    {
        double actualRetracted = std::abs(jm.currentOffset); // 剛才實際上退了多遠
        jm.approachSteps.clear();
        double accumulated = 0.0;

        // 從退刀劇本的第一段開始，吃到滿為止
        for (const auto& s : jm.retractSteps) {
            double remaining = actualRetracted - accumulated;
            if (remaining <= 1e-6) break;

            JumpSegment newSeg = s;
            if (s.distance > remaining) newSeg.distance = remaining; // 裁剪多出的距離

            jm.approachSteps.push_back(newSeg);
            accumulated += newSeg.distance;
        }
        // 翻轉順序，讓最後退的變成最先回來的
        std::reverse(jm.approachSteps.begin(), jm.approachSteps.end());
    }








    jm.resumeAlignMode = alignMode;
    jm.alignVel = alignVel;
    // 🌟 存入遮罩
    jm.firstStageMask = firstStageMask;

    // 拍下操作員 Jog 完的 8 軸現有座標，並算出要回頂點的總距離！
    double sum_sq = 0.0;
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.joggedStartPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
        double delta = jm.apexPos[i] - jm.joggedStartPos[i];
        sum_sq += (delta * delta);
    }

    jm.alignDist = std::sqrt(sum_sq); // 要走回頂點的總距離
    jm.alignOffset = 0.0;             // 對齊進度歸零
    jm.jumpVel = 0.0;

    // 啟動對齊引擎
    if (alignMode == 0) jm.state = JumpState::RESUME_ALIGN_PRIMARY;
    else if (alignMode == 1) jm.state = JumpState::RESUME_ALIGN_OTHERS;
    else jm.state = JumpState::RESUME_ALIGN_ALL;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
}





// =================================================================
// 🟢 [新增] 單軸獨立梯形規劃器 (跳刀專用)
// 負責計算每一毫秒的加減速與位移，並自動處理方向
// =================================================================
double MotionCore::PlanTrapezoidal(double currentPos, double targetPos, double maxVel, double acc, double dec, double& currentVel, double dt)
{
    double error = targetPos - currentPos;
    double dir = (error >= 0.0) ? 1.0 : -1.0;
    double distLeft = std::abs(error);
    double currentSpeed = std::abs(currentVel);

    // 🟢 [改良 1] 完美降落：必須「距離極短」且「速度已經極慢」才能切斷動力，徹底消滅紫色尖刺！
    if (distLeft < 0.001 || (currentSpeed < 0.01 && distLeft < 0.5)) {
        currentVel = 0.0;
        return targetPos;
    }

    // 2. 計算煞車距離 (公式: v^2 / 2a)
    // 預判目前的車速，需要多少距離才能煞停
    double stopDist = (currentSpeed * currentSpeed) / (2.0 * dec);

    double targetSpeed = 0.0;

    // 3. 判斷要加速還是減速
    if (distLeft <= stopDist) {
        // 已經進入煞車區！強迫把目標速度設為 0
        targetSpeed = 0.0;
    }
    else {
        // 還在安全區，可以往最高速衝刺
        targetSpeed = maxVel;
    }

    // 4. 執行速度斜坡 (Acc / Dec)
    double activeAcc = (targetSpeed >= currentSpeed) ? acc : dec;

    if (currentSpeed < targetSpeed) {
        currentSpeed += activeAcc * dt;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    }
    else if (currentSpeed > targetSpeed) {
        currentSpeed -= activeAcc * dt;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }
    // 🟢 [改良 2] 防卡死蠕動：如果速度變 0 但還沒碰到終點，給予微小推力，保證 100% 抵達
    if (currentSpeed < 0.0001 && distLeft > 0.001) {
        currentSpeed = 0.0001;
    }

    currentVel = dir * currentSpeed;
    double nextPos = currentPos + (currentVel * dt);

    // 🟢 [改良 3] 防過沖保護：確保這 1ms 跨出去絕對不會超過目標點，避免抖動
    if ((dir > 0.0 && nextPos > targetPos) || (dir < 0.0 && nextPos < targetPos)) {
        currentVel = 0.0;
        return targetPos;
    }

    return nextPos;
}
double MotionCore::PlanTrapezoidal_B2(double currentPos, double targetPos, double maxVel, double acc, double dec, double& currentVel, double dt)
{
    double error = targetPos - currentPos;
    double dir = (error >= 0.0) ? 1.0 : -1.0;
    double distLeft = std::abs(error);

    // 🌟 1. 方向反轉偵測 (把門檻降到極低：只要有 0.1 的微小速度，就必須保護)
    bool isReversing = (currentVel > 0.1 && dir < 0) || (currentVel < -0.1 && dir > 0);

    // 🌟 2. 目標速度決策
    double targetSpeed = 0.0;
    if (isReversing) {
        targetSpeed = 0.0; // 只要方向不對，唯一目標就是停下來
    }
    else {
        // 計算煞車距離
        double stopDist = (currentVel * currentVel) / (2.0 * dec);
        targetSpeed = (distLeft <= stopDist) ? 0.0 : maxVel;
    }

    // 🌟 3. 核心：強制斜坡爬升與下降 (消滅垂直線)
    double targetVelWithDir = dir * targetSpeed;
    if (isReversing) targetVelWithDir = 0.0; // 煞車時維持原本的符號，朝 0 逼近

    if (currentVel < targetVelWithDir) {
        currentVel += acc * dt;
        if (currentVel > targetVelWithDir && !isReversing) currentVel = targetVelWithDir;
    }
    else if (currentVel > targetVelWithDir) {
        currentVel -= dec * dt;
        if (currentVel < targetVelWithDir && !isReversing) currentVel = targetVelWithDir;
    }

    // 容許誤差：萬分之一圈 (1/10000 Rev)
    double posTolerance = 16777216.0 / 10000.0; // 大約 1677 Pulse
    double velTolerance = 16777216.0 / 1000.0;  // 大約 16777 Pulse/sec (約 0.06 RPM)

    if (distLeft < posTolerance && std::abs(currentVel) < velTolerance) {
        currentVel = 0.0;
        return targetPos;
    }

    return currentPos + (currentVel * dt);
}
void MotionCore::UpdateInterpolation()
{
    // 所有 Stale 清理 Helper 共用這一份 250 us Pass 額度。
    m_staleCommandDiscardBudgetRemaining =
        MOTION_COMMAND_STALE_DISCARD_LIMIT_PER_RUNTIME_PASS;

    // Stage NC-0.1D：Final Feedback Ring 只有 250 us Runtime 能發布。
    // 先轉送 NC Producer 產生的 Queue-Full Notice，再套用 Epoch 變更。
    DrainProducerFeedbackNotices();

    ApplyPendingExecutionEpochChange();

    // Continue bounded stale ingress/replay retirement on every pass.  A
    // Reset with more than one discard budget of queued work must still
    // reach an empty transport before its pre-rebase proof can complete.
    DiscardStaleQueuedCommands();

    // Safety requests have priority over manual/home mailbox commands.
    ApplyPendingSafetyAndRecoveryRequests();

    // A producer can publish a newer Epoch while the bounded CAS seam above
    // is running.  Safety/recovery requests have already had priority; all
    // ordinary settle/mailbox/planner work now waits until that exact packed
    // lifecycle tuple has been applied.  TryDequeueNextMotionCommand() repeats
    // the gate at the final command-acquisition seam to close publication that
    // occurs later in this pass.
    if (HasPendingExecutionEpochChange())
    {
        return;
    }

    // J.5 consumes at most one fixed-ring request per pass.  Reset rebase
    // buffer clearing is budgeted and temporarily owns this interpolation
    // pass; controlled stop and ordinary motion continue while pre-proof is
    // still being collected.
    if (ProcessNCSettleRequestsAndResetRebase())
    {
        return;
    }

    DrainAxisCommandMailbox();

    // 1. 基本防呆
    if (m_pContexts == nullptr) return;

    // =====================================================================
    // NC-0.2K.2.1 - RT group-membership invariant
    //
    // INTERPOLATING is owned exclusively by the active interpolation group.
    // Detect an invalid/duplicate group layout or any physical axis left in
    // INTERPOLATING outside that exact layout before this pass can write new
    // geometry.  The containment action is the existing all-axis E-stop,
    // never a best-effort per-axis deceleration.
    // =====================================================================
    std::array<bool, MAX_AXES> currentGroupMembership{};
    bool currentGroupMappingValid = true;
    if (m_Group.isActive)
    {
        const int safeGroupAxisCount =
            ClampMotionAxisCount(m_Group.axisCount);
        currentGroupMappingValid =
            safeGroupAxisCount > 0 &&
            safeGroupAxisCount == m_Group.axisCount &&
            m_Group.currentCmd.axisCount == m_Group.axisCount;

        for (int slot = 0;
            currentGroupMappingValid && slot < safeGroupAxisCount;
            ++slot)
        {
            const int axisIndex = m_Group.axisIndices[slot];
            if (axisIndex < 0 ||
                axisIndex >= static_cast<int>(m_pContexts->size()) ||
                axisIndex >= MAX_AXES ||
                m_Group.currentCmd.axisIndices[slot] != axisIndex ||
                currentGroupMembership[
                    static_cast<std::size_t>(axisIndex)] ||
                !(*m_pContexts)[axisIndex].isExist)
            {
                currentGroupMappingValid = false;
                break;
            }
                    currentGroupMembership[
                        static_cast<std::size_t>(axisIndex)] = true;
        }
    }

    int orphanAxisIndex = -1;
    if (currentGroupMappingValid)
    {
        for (std::size_t axisSlot = 0U;
            axisSlot < m_pContexts->size();
            ++axisSlot)
        {
            const AxisContext& axis = (*m_pContexts)[axisSlot];
            if (!axis.isExist ||
                axis.state != MotionState::MotionState_INTERPOLATING)
            {
                continue;
            }

            const bool exactCurrentMember =
                m_Group.isActive &&
                axisSlot < currentGroupMembership.size() &&
                currentGroupMembership[axisSlot];
            if (!exactCurrentMember)
            {
                orphanAxisIndex = static_cast<int>(axisSlot);
                break;
            }
        }
    }

    if (!currentGroupMappingValid || orphanAxisIndex >= 0)
    {
        m_p1OrphanAxisContainmentCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_p1LastOrphanAxisIndex.store(
            orphanAxisIndex,
            std::memory_order_release);
        TriggerGroupMappingIntegrityEmergencyStop(
            orphanAxisIndex);
        return;
    }

    // [安全門] 檢查參與群組的所有實體軸是否全部激磁
    for (int i = 0; i < m_Group.axisCount; i++)
    {
        int axisIdx = m_Group.axisIndices[i];
        if (!(*m_pContexts)[axisIdx].isServoOn)
        {
            static int logCnt5 = 0;
            if (logCnt5++ % 500 == 0) {
                //RtPrintf("[DBG-3] BLOCKED! Axis %d is NOT ServoOn. Group aborted.\\n", axisIdx);
            }

            FaultTrackedMotionCommand(
                static_cast<std::uint32_t>(
                    AlarmManager::SERVO_ERROR),
                MotionRejectReason::NOT_READY);

            m_Group.isActive = false;

            // 🚨 新增：宣告群組與虛擬主軸進入錯誤狀態，防止 G-code 繼續下單！
            m_Group.virtualAxis.state = MotionState::MotionState_ERROR;
            m_Group.virtualAxis.isFault = true;

            return;
        }
    }




    AxisContext& vAxis = m_Group.virtualAxis; // 先取得 vAxis 的引用
    double dt = CYCLE_TIME_SEC;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisCommand vCmd{}; // 準備統一收集速度與位置
    bool virtualCommandResynchronized = false;




    // ======================================================
    // 🌟 [司機 A] 跳刀狀態機接管 (優先權最高)
    // ======================================================
    if (jm.state != JumpState::IDLE)
    {
        if (jm.state == JumpState::RETRACTING)
        {
            JumpSegment& seg = jm.retractSteps[jm.currentStepIdx];
            double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
            double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

            // 🌟 1. 找出「整個退刀」的最終目標 (不到最後一刻，規劃器不會煞車！)
            double finalTarget = 0.0;
            for (const auto& s : jm.retractSteps) finalTarget -= s.distance;

            // 🌟 2. 找出「當前這一段」的換檔邊界
            double stepBoundary = 0.0;
            for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary -= jm.retractSteps[i].distance;




            // =========================================================
            // 🌟 [只有 B2 才需要] 無腦防呆機制：自動偵測歷史極限
            // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE && m_Group.historyQueue.empty()) {
                double absoluteOrigin = -jm.triggerPos;
                // 強制把大腦的目標截斷在原點，絕對不准多退！
                if (finalTarget < absoluteOrigin) finalTarget = absoluteOrigin;
            }


            // =========================================================
            // 🌟 3. [微創修改] 獨立隔離 B2 與加工暫停的規劃器
            // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE || jm.isPauseMode)
            {
                // 👉 關鍵 A：時光機的觸發點是 vAxis.currentCmdPos < 0
                double trueBoundary = -jm.triggerPos - 1.0;

                // 確保退刀不要退超過最終的總目標
                if (trueBoundary < finalTarget) {
                    trueBoundary = finalTarget;
                }

                // 呼叫規劃器
                jm.currentOffset = PlanTrapezoidal_B2(jm.currentOffset, trueBoundary, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 👉👉👉 【關鍵修正】：比對對象改為 trueBoundary 👈👈👈
                // 這樣不管是退到腳本極限，還是退到歷史盡頭，大腦都會認可「到站了」
                if (std::abs(jm.currentOffset - trueBoundary) < 5.0 && std::abs(jm.jumpVel) < 10.0) {

                    // 補回換檔邏輯 (如果還有下一段腳本)
                    if (jm.currentStepIdx < (int)jm.retractSteps.size() - 1 && std::abs(jm.currentOffset - finalTarget) > 10.0) {
                        jm.currentStepIdx++;
                    }
                    // 抵達最終目標 (不管是物理的還是腳本的)
                    else {
                        if (jm.isPauseMode) {
                            jm.state = JumpState::PAUSED_HOLD;
                            jm.jumpVel = 0.0;
                            for (int i = 0; i < m_Group.axisCount; ++i) {
                                jm.apexPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
                            }
                            // RtPrintf("[B2] 成功抵達物理頂點，切換至 PAUSED_HOLD\n");
                        }
                        else {
                            jm.state = JumpState::DWELL;
                            jm.dwellTimer = 0.0;
                        }
                    }
                }
            }
            else
            {
                // B0, B1 等其他模式，走原本的規劃器 (完全不影響舊功能)
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);
            }
            // =========================================================

            // 🌟 4. 動態換檔：只要跨越了邊界，立刻切換下一段的速度 (不要求速度降到 0)

            if (jm.currentOffset <= stepBoundary && jm.currentStepIdx < (int)jm.retractSteps.size() - 1) {
                jm.currentStepIdx++;
            }

            // 🌟 5. 真正抵達最高點，才進入 DWELL
            if (std::abs(jm.currentOffset - finalTarget) < 0.0001 && std::abs(jm.jumpVel) < 0.1) {
                // 🌟 修改 1：在這裡攔截暫停！如果是暫停，就停在空中並拍下快照。
                if (jm.isPauseMode) {
                    jm.state = JumpState::PAUSED_HOLD;
                    jm.jumpVel = 0.0;
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        jm.apexPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
                    }
                }
                else {
                    jm.state = JumpState::DWELL;
                    jm.dwellTimer = 0.0;
                }
            }
        }
        else if (jm.state == JumpState::DWELL)
        {
            jm.dwellTimer += dt * 1000.0;
            if (jm.dwellTimer >= jm.dwellTimeTarget) {
                jm.state = JumpState::APPROACHING;
                jm.currentStepIdx = 0;
            }

            // =========================================================
                 // 🌟 [神級邏輯]：自動將退刀劇本翻轉為進刀劇本
                 // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE)
            {
                // 1. 直接複製退刀劇本 (確保段數一模一樣)
                jm.approachSteps = jm.retractSteps;

                // 2. 翻轉順序：原本最後退的段落，變成最先跑的進刀段落
                std::reverse(jm.approachSteps.begin(), jm.approachSteps.end());

                // 3. 處理「殘留距離」：扣除多出來的部分
                // 實際退刀總距離就是 std::abs(jm.currentOffset)
                double totalActualMoved = std::abs(jm.currentOffset);
                double accumulatedDist = 0.0;

                // 從最後一段進刀往回看 (也就是最初的退刀段落)
                // 我們要確保進刀的總距離剛好等於實際退刀的距離
                for (int i = (int)jm.approachSteps.size() - 1; i >= 0; --i) {
                    double stepDist = jm.approachSteps[i].distance;
                    if (accumulatedDist + stepDist > totalActualMoved) {
                        // 這一小段就是被「截斷」的地方
                        jm.approachSteps[i].distance = totalActualMoved - accumulatedDist;
                        // 前面那些還沒算到的段落通通歸零 (因為退刀根本沒走到那邊)
                        for (int k = 0; k < i; ++k) jm.approachSteps[k].distance = 0.0;
                        break;
                    }
                    accumulatedDist += stepDist;
                }

                // 4. (選配) 如果你有特殊的尋邊速度要求，可以在這裡覆寫最後一段的速度
                // jm.approachSteps.back().velocity = ONE_REV * 1.0; 
            }
        }
        else if (jm.state == JumpState::APPROACHING)
        {
            // 🌟 [防爆衝護欄] 防止 Idx 越界崩潰 (Access Violation 殺手)
            if (jm.approachSteps.empty() || jm.currentStepIdx >= (int)jm.approachSteps.size() || jm.currentStepIdx < 0)
            {
                RtPrintf("!!! ERROR !!! Approach Index Out of Range!\n");
                jm.state = JumpState::IDLE;
                m_Group.pathMode = PathMode::PATH_SERVO;
                return;
            }

            // =========================================================
            // 🌟 [完全分流]：只有 B2 模式才執行的邏輯
            // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE)
            {
                // 👉 沒錯！就只要這一行！把大腦完全交給副程式去算！
                Process_B2_Approach_Planner(dt);
            }
            else
            {
                // ✅ [原本程式]：B0, B1, B3, B4 維持原封不動的邏輯
                JumpSegment& seg = jm.approachSteps[jm.currentStepIdx];
                double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
                double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

                // 🌟 1. 進刀的最終目標：回到出發點 (0.0)
                double finalTarget = 0.0;

                // 🌟 2. 算出總起點 (腳本極限)
                double startOffset = 0.0;
                for (const auto& s : jm.retractSteps) startOffset -= s.distance;

                double stepBoundary = startOffset;
                for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary += jm.approachSteps[i].distance;

                // 原本的規劃器
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 原本的動態換檔 (不要求速降為 0)
                if (jm.currentOffset >= stepBoundary && jm.currentStepIdx < (int)jm.approachSteps.size() - 1) {
                    jm.currentStepIdx++;
                }
            }




            // 🌟 5. 真正降落完畢 (這段保留)
            double finalTarget = 0.0;
            // 🌟 5. 真正降落完畢 (這段是共用的，但判斷門檻放寬到 5.0 Pulse)
            if (std::abs(jm.currentOffset - finalTarget) < 5.0 && std::abs(jm.jumpVel) < 10.0) {

                int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                (*m_pContexts)[idxX].logicalCmdPos = jm.frozenPos[0];
                (*m_pContexts)[idxY].logicalCmdPos = jm.frozenPos[1];

                //RtPrintf("[JUMP END] Back to Spark Point! Offset:%d\n", (int)jm.currentOffset);

                jm.state = JumpState::IDLE;
                jm.currentOffset = 0.0;
                m_Group.pathMode = PathMode::PATH_SERVO;
            }
        }







        // 🔴 把跳刀算出來的 Offset 套用到虛擬主軸上
        // ======================================================
        // 🔴 核心分流：B2 是倒退嚕，B1/B3/B4 是凍結放電進度！
        // ======================================================
        if (jm.mode == JumpMode::B2_PATH_REVERSE)
        {
            // B2：把跳刀 Offset 套用到虛擬主軸上 (原路徑倒退)
            vAxis.currentCmdPos = jm.triggerPos + jm.currentOffset;
            vAxis.currentCmdVel = jm.jumpVel;
        }
        else
        {
            // B1, B0, B3, B4：【凍結】虛擬主軸！讓放電軌跡停在半空中
            vAxis.currentCmdPos = jm.triggerPos;
            vAxis.currentCmdVel = 0.0;
        }

        vCmd.instantCmdPos = vAxis.currentCmdPos;
        vCmd.instantCmdVel = vAxis.currentCmdVel;
    }
    // ======================================================
    // 🌟 [司機 B] PATH_SERVO 放電上帝模式 (優先權次高)
    // ======================================================
    else if (m_Group.pathMode == PathMode::PATH_SERVO)
    {


        // ✅ 換成這段 [工業級 G00/G61 準停檢查邏輯]：
        if (!m_Group.isActive || vAxis.inPosition)
        {
            bool allAxesInPos = true;

            // 如果是 G00 快速定位 或 G61 準停模式，必須確認實體馬達有跟上
            if (m_Group.isActive && m_Group.pathMode == PathMode::EXACT_STOP)
            {
                for (int i = 0; i < m_Group.axisCount; ++i) {
                    int idx = m_Group.axisIndices[i];
                    AxisContext& realAxis = (*m_pContexts)[idx];

                    double lag = std::abs(realAxis.currentCmdPos - realAxis.currentActPos);

                    // 只要有一軸還沒擠進視窗 (例如 0.005mm)，就不准換下一行！
                    if (lag > realAxis.inPositionWindow_Pulse) {
                        allAxesInPos = false;
                        // 🟢 [加入 LOG 2]：觀察實體馬達的誤差是不是卡住了
                        if (m_Group.cmdQueue.empty() && vAxis.inPosition) {
                            static int logCnt2 = 0;
                            if (logCnt2++ % 500 == 0) {
                                //RtPrintf("[LOG 2] Axis %d STUCK! Lag: %d > Window: %d\n",idx, (int)lag, (int)realAxis.inPositionWindow_Pulse);
                            }
                        }
                        break;
                    }
                }
            }

            // 大腦算完了，且實體馬達也都擠進視窗了，才准拆下一個包裹！
            if (allAxesInPos && !m_Group.cmdQueue.empty()) {
                LoadNextCommand();
            }
        }
        if (!m_Group.isActive) return;

        vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;



        // 🟢 [修復]：把外部速度當作「目標」，交給速度規劃器去產生平滑的斜坡與 S-Curve！
         //vAxis.targetVelocity = m_Group.pathServoVel;
         //Calc_Trajectory_Velocity(vAxis, vCmd);




         // =========================================================
         // 🟢 智慧分流：排渣回來的那一次 (S-Curve) vs 正常放電 (即時速度)
         // =========================================================
        if (jm.isRecovering)
        {
            // 【狀態 1：軟著陸中】交給規劃器，畫出平滑起步的斜坡，避免突刺！
            vAxis.targetVelocity = m_Group.pathServoVel;
            Calc_Trajectory_Velocity(vAxis, vCmd);


            //jm.isRecovering = false;//
        }
        else
        {
            // 【Path Servo 模式】：由外部速度積分
            vAxis.currentCmdVel = m_Group.pathServoVel;
            vAxis.currentCmdPos += vAxis.currentCmdVel * dt;
        }
        // =========================================================


        // 🟢 關鍵修復：防止鬼畜卡死！
        if (vAxis.currentCmdPos < vAxis.finalTargetPos) { vAxis.inPosition = false; }

        // =========================================================
        // PATH_SERVO 正方向抵達目前路徑終點
        //
        // 注意：
        // 「向後跨節」不要在 PATH_SERVO 裡處理。
        // 統一交給下面共用的 History Crossing 區塊。
        // =========================================================
        if (vAxis.currentCmdPos >= vAxis.finalTargetPos)
        {
            vAxis.currentCmdPos = vAxis.finalTargetPos;
            vAxis.inPosition = true;
        }

        vCmd.instantCmdPos = vAxis.currentCmdPos;
        vCmd.instantCmdVel = vAxis.currentCmdVel;
        vCmd.instantCmdPos = vAxis.currentCmdPos;
        vCmd.instantCmdVel = vAxis.currentCmdVel;
    }
    // ======================================================
    // 🌟 [司機 C] 正常加工模式 (優先權最低)
    // ======================================================
    else
    {
        // 拆包裹邏輯
        // ✅ 換成這段 [工業級 G00/G61 準停檢查邏輯]：
        if (!m_Group.isActive || vAxis.inPosition)
        {
            bool allAxesInPos = true;

            // 如果是 G00 快速定位 或 G61 準停模式，必須確認實體馬達有跟上
            if (m_Group.isActive && m_Group.pathMode == PathMode::EXACT_STOP)
            {
                for (int i = 0; i < m_Group.axisCount; ++i) {
                    int idx = m_Group.axisIndices[i];
                    AxisContext& realAxis = (*m_pContexts)[idx];

                    double lag = std::abs(realAxis.currentCmdPos - realAxis.currentActPos);

                    // 只要有一軸還沒擠進視窗 (例如 0.005mm)，就不准換下一行！
                    if (lag > realAxis.inPositionWindow_Pulse)
                    {
                        allAxesInPos = false;

                        // 🟢 [加入 LOG 2]：觀察實體馬達的誤差是不是卡住了
                        if (m_Group.cmdQueue.empty() && vAxis.inPosition)
                        {
                            static int logCnt2 = 0;
                            if (logCnt2++ % 500 == 0)
                            {
                                //RtPrintf("[LOG 2] Axis %d STUCK! Lag: %d > Window: %d\n",idx, (int)lag, (int)realAxis.inPositionWindow_Pulse);
                            }
                        }
                        break;
                    }
                }
            }

            // 大腦算完了，且實體馬達也都擠進視窗了，才准拆下一個包裹！
            if (allAxesInPos && !m_Group.cmdQueue.empty())
            {
                LoadNextCommand();
            }
        }

        if (!m_Group.isActive) return;
        vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;

        if (vAxis.state == MotionState::MotionState_STOPPING)
        {
            Calc_Trajectory_Velocity(vAxis, vCmd);
        }
        else
        {
            Calc_Trajectory_Trapezoidal(vAxis, vCmd);
        }
    }



    // =========================================================
    // 🔴 獨立的時光機 (向後跨節)：必須放在司機分流的外面！
    // =========================================================
    if (vAxis.currentCmdPos < 0.0)
    {
        virtualCommandResynchronized = true;

        if (m_Group.enableHistory && !m_Group.historyQueue.empty())
        {
            const MotionCommand& historyCandidate =
                m_Group.historyQueue.back();
            if (HasPendingExecutionEpochChange() ||
                GetCommandAuthorizationFailure(historyCandidate) !=
                MotionRejectReason::NONE)
            {
                return;
            }

            if (!IsMotionCommandConsumerGeometryValid(
                m_Group.currentCmd,
                m_pContexts) ||
                !IsMotionCommandConsumerGeometryValid(
                    historyCandidate,
                    m_pContexts) ||
                !IsMotionCommandHistorySnapshotValid(
                    m_Group.currentCmd) ||
                !IsMotionCommandHistorySnapshotValid(
                    historyCandidate) ||
                !MotionCommandsHaveIdenticalAxisMapping(
                    m_Group.currentCmd,
                    historyCandidate))
            {
                if (HasPendingExecutionEpochChange() ||
                    GetCommandAuthorizationFailure(m_Group.currentCmd) !=
                    MotionRejectReason::NONE ||
                    GetCommandAuthorizationFailure(historyCandidate) !=
                    MotionRejectReason::NONE)
                {
                    return;
                }
                m_p1DroppedAxisRetirementFailureCount.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
                m_p1LastPreviousAxisMask.store(
                    BuildMotionCommandAxisMask(m_Group.currentCmd),
                    std::memory_order_release);
                m_p1LastNextAxisMask.store(
                    BuildMotionCommandAxisMask(historyCandidate),
                    std::memory_order_release);
                m_p1LastOrphanAxisIndex.store(
                    -1,
                    std::memory_order_release);
                TriggerGroupMappingIntegrityEmergencyStop(-1);
                return;
            }

            // SPSC Ingress 不允許 RT Consumer push_front。
            // 離開的 Current Segment 放入 RT-only Replay Front，正向返回時
            // 會優先於 NC 新發布的 Ingress Command 被取出。
            LifecycleCommitReservationGuard lifecycleCommit(
                *this,
                m_Group.currentCmd.execution);
            if (!lifecycleCommit.IsAcquired())
            {
                return;
            }

            if (TryRequeueMotionCommandFront(
                m_Group.currentCmd))
            {
                if (HasPendingExecutionEpochChange() ||
                    GetCommandAuthorizationFailure(m_Group.currentCmd) !=
                    MotionRejectReason::NONE ||
                    GetCommandAuthorizationFailure(historyCandidate) !=
                    MotionRejectReason::NONE)
                {
                    return;
                }

                m_Group.currentCmd = m_Group.historyQueue.back();
                m_Group.historyQueue.pop_back();

                // 恢復大腦的幾何狀態
                m_Group.mode = m_Group.currentCmd.mode;
                m_Group.axisCount = m_Group.currentCmd.axisCount;
                for (int i = 0; i < 8; i++)
                {
                    m_Group.startPos[i] = m_Group.currentCmd.mem_startPos[i];
                    m_Group.ratio[i] = m_Group.currentCmd.mem_ratio[i];
                    m_Group.axisIndices[i] = m_Group.currentCmd.axisIndices[i];
                }
                m_Group.radius = m_Group.currentCmd.mem_radius; m_Group.startAngle = m_Group.currentCmd.mem_startAngle;
                m_Group.centerX = m_Group.currentCmd.mem_centerX; m_Group.centerY = m_Group.currentCmd.mem_centerY;
                m_Group.totalDist3D = m_Group.currentCmd.mem_totalDist; m_Group.totalAngle = m_Group.currentCmd.mem_totalAngle;

                m_Group.enableTransform = m_Group.currentCmd.mem_enableTransform;
                for (int i = 0; i < 3; ++i)
                {
                    m_Group.transformOrigin[i] = m_Group.currentCmd.mem_transformOrigin[i];
                    for (int j = 0; j < 3; ++j) m_Group.transformMatrix[i][j] = m_Group.currentCmd.mem_transformMatrix[i][j];
                }

                // 🌟 終極修復：跨節後，必須把剩餘的負數距離，灌給新的線條長度！
                vAxis.currentCmdPos += m_Group.currentCmd.mem_totalDist;

                // 🌟 如果是 B2 模式，必須同步平移起點，才不會無限閃退！
                if (jm.state != JumpState::IDLE) jm.triggerPos += m_Group.currentCmd.mem_totalDist;
            }
            else
            {
                // Replay 固定容量理論上大於 History 上限；若仍失敗，
                // 不可丟失 Current Segment，只能鎖在本段起點等待 Alarm。
                vAxis.currentCmdPos = 0.0;
                vAxis.currentCmdVel = 0.0;
            }
        }
        else
        {
            // 🌟 已經退到最最最起點 (歷史空了)，強制鎖死在 0.0！
            vAxis.currentCmdPos = 0.0;
            vAxis.currentCmdVel = 0.0;
        }
    }

    // =========================================================
    // 🔴 Jump / B2 專用：向前跨節
    //
    // 向後跨節已經由上面的共用區統一處理，
    // 這裡絕對不要再 pop history。
    // =========================================================
    if (jm.state != JumpState::IDLE)
    {
        if (vAxis.currentCmdPos >
            m_Group.currentCmd.mem_totalDist)
        {
            Process_Forward_Crossing();
            if (!m_Group.isActive)
            {
                return;
            }
            virtualCommandResynchronized = true;
        }
    }

    vCmd.instantCmdPos = vAxis.currentCmdPos;
    if (virtualCommandResynchronized)
    {
        // History/B2 crossing mutates the raw virtual command after the
        // planner returned, so only that path needs an explicit velocity
        // resynchronization.  Normal motion must retain the planner's
        // filtered/clamped instantCmdVel instead of restoring raw cruise speed.
        vCmd.instantCmdVel = vAxis.currentCmdVel;
    }
    vAxis.logicalCmdVel = vCmd.instantCmdVel;





    // ======================================================
    // 🌟 以下完全保留你原本的幾何分配與空間旋轉 (原封不動！)
    // ======================================================
    if (m_Group.mode == InterpolationMode::LINEAR)
    {
        for (int i = 0; i < m_Group.axisCount; ++i) {
            int idx = m_Group.axisIndices[i];
            AxisContext& realAxis = (*m_pContexts)[idx];
            realAxis.logicalCmdPos = m_Group.startPos[i] + (vCmd.instantCmdPos * m_Group.ratio[i]);
            realAxis.logicalCmdVel = vCmd.instantCmdVel * m_Group.ratio[i];
        }
    }
    else if (m_Group.mode == InterpolationMode::CIRCULAR_CW || m_Group.mode == InterpolationMode::CIRCULAR_CCW)
    {
        // =====================================================
        // 1. Axis
        // =====================================================
        int idxX = m_Group.axisIndices[0];
        int idxY = m_Group.axisIndices[1];

        AxisContext& realX = (*m_pContexts)[idxX];
        AxisContext& realY = (*m_pContexts)[idxY];


        // =====================================================
        // 2. Spiral Geometry
        // =====================================================
        const double startRadius = m_Group.currentCmd.startRadius;
        const double endRadius = m_Group.currentCmd.endRadius;
        const double deltaRadius = endRadius - startRadius;
        const double totalAngle = m_Group.totalAngle;
        double deltaZ = 0.0;


        if (m_Group.axisCount >= 3)
        {
            deltaZ = m_Group.currentCmd.targetPos[2] - m_Group.startPos[2];
        }


        // =====================================================
        // 3. Path Distance -> Spiral Progress
        //
        // 不能再直接：
        //
        // progress =
        //     currentCmdPos / totalDist
        //
        // 因為 Spiral 每個 progress 的實際距離不同。
        // =====================================================
        double progressRatio = CalcSpiralProgressFromPathLength(vCmd.instantCmdPos, m_Group.totalDist3D, startRadius, endRadius, totalAngle, deltaZ);


        // =====================================================
        // 4. Current Radius / Angle
        // =====================================================
        const double currentRadius = startRadius + deltaRadius * progressRatio;
        const double currentAngle = m_Group.startAngle + totalAngle * progressRatio;
        const double cosAngle = std::cos(currentAngle);
        const double sinAngle = std::sin(currentAngle);


        // =====================================================
        // 5. Position
        //
        // 這裡才是真正 Variable Radius。
        // =====================================================
        realX.logicalCmdPos = m_Group.centerX + currentRadius * cosAngle;
        realY.logicalCmdPos = m_Group.centerY + currentRadius * sinAngle;


        // =====================================================
        // 6. Z Position
        // =====================================================
        AxisContext* realZ = nullptr;

        if (m_Group.axisCount >= 3)
        {
            int idxZ = m_Group.axisIndices[2];
            realZ = &(*m_pContexts)[idxZ];
            realZ->logicalCmdPos = m_Group.startPos[2] + deltaZ * progressRatio;
        }


        // =====================================================
        // 7. 計算 ds/dp
        //
        // ds/dp =
        //
        // sqrt(
        //      dr^2
        //    + (r*dTheta)^2
        //    + dz^2
        // )
        //
        // 因為：
        //
        // vPath = ds/dt
        //
        // 所以：
        //
        // dp/dt = vPath / (ds/dp)
        // =====================================================
        const double pathMetric = std::sqrt(deltaRadius * deltaRadius + currentRadius * currentRadius * totalAngle * totalAngle + deltaZ * deltaZ);


        double progressVelocity = 0.0;


        if (pathMetric > 1e-12)
        {
            progressVelocity = vCmd.instantCmdVel / pathMetric;
        }


        // =====================================================
        // 8. Position Derivative
        //
        // X =
        // Cx + r cos(theta)
        //
        // Y =
        // Cy + r sin(theta)
        //
        // dx/dp =
        // dr*cos(theta)
        // - r*sin(theta)*dTheta
        //
        // dy/dp =
        // dr*sin(theta)
        // + r*cos(theta)*dTheta
        //
        // =====================================================
        const double dx_dp = deltaRadius * cosAngle - currentRadius * sinAngle * totalAngle;
        const double dy_dp = deltaRadius * sinAngle + currentRadius * cosAngle * totalAngle;


        // =====================================================
        // 9. Exact Velocity Vector
        //
        // 同時包含：
        //
        // Tangential Velocity
        // +
        // Radial Velocity
        //
        // CW / CCW 不用另外反轉，
        // 因為 totalAngle 本身已經帶正負號。
        // =====================================================
        realX.logicalCmdVel = dx_dp * progressVelocity;
        realY.logicalCmdVel = dy_dp * progressVelocity;


        // =====================================================
        // 10. Z Velocity
        // =====================================================
        if (realZ != nullptr)
        {
            realZ->logicalCmdVel = deltaZ * progressVelocity;
        }
    }


    // =====================================================================
    // 🌟 🟢 [全新架構：跳刀 3D 向量疊加層] 🟢 🌟
    // =====================================================================
    if (jm.state != JumpState::IDLE)
    {
        double offsetDist = std::abs(jm.currentOffset);
        double offsetVel = std::abs(jm.jumpVel);
        int sign = (jm.state == JumpState::RETRACTING) ? 1 : -1; // 退刀加，進刀減


        // 🌟 修改 2：加入復歸對齊引擎 (獨立運作，不干擾原本降落邏輯)
        if (jm.state == JumpState::RESUME_ALIGN_PRIMARY ||
            jm.state == JumpState::RESUME_ALIGN_OTHERS ||
            jm.state == JumpState::RESUME_ALIGN_ALL)
        {
            for (int i = 0; i < m_Group.axisCount; ++i) {

                bool isFirstStageAxis = (jm.firstStageMask & (1 << i)) != 0;
                bool shouldMove = false;
                if (jm.state == JumpState::RESUME_ALIGN_ALL) shouldMove = true;
                else if (jm.state == JumpState::RESUME_ALIGN_PRIMARY) shouldMove = isFirstStageAxis;
                else if (jm.state == JumpState::RESUME_ALIGN_OTHERS) shouldMove = !isFirstStageAxis;

                int axisIdx = m_Group.axisIndices[i];
                if (shouldMove && jm.alignDist > 1e-5) {
                    double delta = jm.apexPos[i] - jm.joggedStartPos[i];
                    (*m_pContexts)[axisIdx].logicalCmdPos = jm.joggedStartPos[i] + (delta * (jm.alignOffset / jm.alignDist));
                }
                else {
                    // 🌟 終極護盾：如果這軸這回合不動，或者根本沒有距離 (alignDist=0)，
                    // 必須強制把它鎖死在原位，防止被上方的 LINEAR 幾何破壞！
                    (*m_pContexts)[axisIdx].logicalCmdPos = jm.joggedStartPos[i];
                }
                // 🌟 對齊期間，速度交給引擎算，非移動軸強制為 0
                (*m_pContexts)[axisIdx].logicalCmdVel = 0.0;
            }

            if (jm.alignDist < 1e-5 || jm.alignOffset >= jm.alignDist - 1e-5)
            {
                if (jm.state == JumpState::RESUME_ALIGN_PRIMARY && jm.resumeAlignMode == 0)
                {
                    // 🟢 第一階段 (Primary) 結束：只把「有參與第一階段」的軸，起點更新為頂點
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        if ((jm.firstStageMask & (1 << i)) != 0) jm.joggedStartPos[i] = jm.apexPos[i];
                    }
                    jm.state = JumpState::RESUME_ALIGN_OTHERS;
                    jm.alignOffset = 0.0; jm.jumpVel = 0.0;

                    // 🌟 重新計算第二階段的總距離！
                    double sum_sq = 0.0;
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        double delta = jm.apexPos[i] - jm.joggedStartPos[i];
                        sum_sq += delta * delta;
                    }
                    jm.alignDist = std::sqrt(sum_sq);
                }
                else if (jm.state == JumpState::RESUME_ALIGN_OTHERS && jm.resumeAlignMode == 1)
                {
                    // 🟢 第一階段 (Others) 結束：只把「有參與 Others」的軸，起點更新為頂點
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        if ((jm.firstStageMask & (1 << i)) == 0) jm.joggedStartPos[i] = jm.apexPos[i];
                    }
                    jm.state = JumpState::RESUME_ALIGN_PRIMARY;
                    jm.alignOffset = 0.0; jm.jumpVel = 0.0;

                    // 🌟 重新計算第二階段的總距離！
                    double sum_sq = 0.0;
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        double delta = jm.apexPos[i] - jm.joggedStartPos[i];
                        sum_sq += delta * delta;
                    }
                    jm.alignDist = std::sqrt(sum_sq);
                }
                else
                {
                    // 🟢 所有對齊階段都結束了！完美收尾。
                    for (int i = 0; i < m_Group.axisCount; ++i) jm.joggedStartPos[i] = jm.apexPos[i];
                    jm.alignOffset = 0.0; jm.jumpVel = 0.0;

                    // =========================================================
                    // 🌟 完美銜接：根據模式把機台推回軌道
                    // =========================================================
                    if (jm.mode == JumpMode::B3_CENTER) jm.state = JumpState::B3_FROM_APEX;
                    else if (jm.mode == JumpMode::B4_ORBITAL_DIAGONAL) jm.state = JumpState::B4_FROM_APEX;
                    else jm.state = JumpState::APPROACHING;

                    jm.isPauseMode = false; jm.currentStepIdx = 0;

                    // ⚠️ 還原降落起點 (B2 絕對不能動！)
                    if (jm.mode == JumpMode::B0_REVERSE || jm.mode == JumpMode::B1_SPECIFIC_AXIS) {
                        double startOffset = 0.0;
                        for (const auto& s : jm.retractSteps) startOffset -= s.distance;
                        jm.currentOffset = startOffset;
                    }
                }
            }
            else {
                double acc = jm.alignVel * 5.0;
                jm.alignOffset = PlanTrapezoidal(jm.alignOffset, jm.alignDist, jm.alignVel, acc, acc, jm.jumpVel, dt);
            }
        }
        else if (jm.mode == JumpMode::B1_SPECIFIC_AXIS)
        {
            int targetIdx = jm.b1_AxisIndex;
            // 絕對鎖死：起點 + (現在的距離 * 向量)
            (*m_pContexts)[targetIdx].logicalCmdPos = jm.frozenPos[targetIdx] + (offsetDist * jm.b1_dir);
            (*m_pContexts)[targetIdx].logicalCmdVel = offsetVel * jm.b1_dir * sign;


        }
        else if (jm.mode == JumpMode::B0_REVERSE)
        {
            for (int i = 0; i < m_Group.axisCount; ++i) {
                int axisIdx = m_Group.axisIndices[i];
                double vec_component = jm.b0_Vector[i];
                // 絕對鎖死：起點 + (現在的距離 * 向量)
                (*m_pContexts)[axisIdx].logicalCmdPos = jm.frozenPos[axisIdx] + (offsetDist * vec_component);
                (*m_pContexts)[axisIdx].logicalCmdVel = (offsetVel * sign) * vec_component;
            }
        }// ======================================================
        // 🌟 B3 專屬的 5 階段狀態機 (完美轉角煞停 + 內部融合)
        // ======================================================
        else if (jm.mode == JumpMode::B3_CENTER && (jm.state >= JumpState::B3_TO_CENTER && jm.state <= JumpState::B3_TO_WORKPIECE || jm.state == JumpState::PAUSED_HOLD))
        {
            std::vector<JumpSegment>* currentScript = nullptr;
            double finalTarget = 0.0;
            JumpState nextState = JumpState::IDLE;

            // 1. 根據當前狀態，選擇對應的腳本與目標
            if (jm.state == JumpState::B3_TO_CENTER) {
                currentScript = &jm.b3_toCenterSteps;
                finalTarget = jm.b3_distToCenter;
                nextState = JumpState::B3_TO_APEX;
            }
            else if (jm.state == JumpState::B3_TO_APEX) {
                currentScript = &jm.b3_toApexSteps;
                finalTarget = jm.b3_distToApex;
                nextState = JumpState::B3_DWELL;
            }
            else if (jm.state == JumpState::B3_FROM_APEX) {
                currentScript = &jm.b3_fromApexSteps;
                finalTarget = jm.b3_distToApex;
                nextState = JumpState::B3_TO_WORKPIECE;
            }
            else if (jm.state == JumpState::B3_TO_WORKPIECE) {
                currentScript = &jm.b3_toWorkpieceSteps;
                finalTarget = jm.b3_distToCenter;
                nextState = JumpState::IDLE;
            }

            // 2. 處理 DWELL 狀態
            if (jm.state == JumpState::B3_DWELL) {
                jm.dwellTimer += dt * 1000.0;
                if (jm.dwellTimer >= jm.dwellTimeTarget) {
                    jm.state = JumpState::B3_FROM_APEX;
                    jm.currentStepIdx = 0; jm.currentOffset = 0.0; jm.jumpVel = 0.0;
                }
            }
            // 3. 處理移動狀態 (執行規劃器與速度融合)
            else if (currentScript != nullptr && !currentScript->empty())
            {
                JumpSegment& seg = (*currentScript)[jm.currentStepIdx];
                double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
                double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

                // 🌟 內部融合邊界判斷
                double stepBoundary = 0.0;
                for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary += (*currentScript)[i].distance;

                // 呼叫規劃器 (往 finalTarget 前進)
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 跨越內部腳本，無縫換檔
                if (jm.currentOffset >= stepBoundary && jm.currentStepIdx < (int)currentScript->size() - 1) {
                    jm.currentStepIdx++;
                }
                // 🌟 【真・防爆衝換檔】：只要距離大於等於目標(容許微小誤差)，立刻強制斬斷！絕不讓規劃器發瘋反轉！
                if (jm.currentOffset >= finalTarget - 1e-5) {

                    jm.currentStepIdx = 0;
                    jm.currentOffset = 0.0; // 進入下一階段，里程碑強制歸零！
                    jm.jumpVel = 0.0;       // 速度強制歸零煞停！
                    jm.dwellTimer = 0.0;

                    // ==========================================================
                    // 🌟 [絕對關鍵：暫停空中攔截網]
                    // 如果剛走完 TO_APEX 抵達最高點，且是暫停模式，直接鎖死在 PAUSED_HOLD！
                    // ==========================================================
                    if (jm.state == JumpState::B3_TO_APEX && jm.isPauseMode) {
                        jm.state = JumpState::PAUSED_HOLD;

                        // 💥 破案核心：絕對不能用 logicalCmdPos 拍快照！那會有 1ms 的落後誤差！
                         // 必須直接用幾何數學，算出 100% 完美的實體頂點座標！
                        jm.apexPos[0] = jm.b3_centerPos[0] + jm.b3_distToApex * jm.b3_retractVector[0];
                        jm.apexPos[1] = jm.b3_centerPos[1] + jm.b3_distToApex * jm.b3_retractVector[1];
                        if (m_Group.axisCount >= 3) {
                            jm.apexPos[2] = jm.b3_centerPos[2] + jm.b3_distToApex * jm.b3_retractVector[2];
                        }
                        // RtPrintf("[PAUSE_B3] Safely Holding at Math Apex.\n");
                    }
                    else {
                        // 正常的排渣模式，或者是 B3 的其他階段，就順順切換到下一個狀態
                        jm.state = nextState;
                    }

                    // ==========================================================
                    // 🌟 【神級修復：歸還放電控制權】
                    // 如果第四段走完，狀態變成 IDLE，必須立刻把鑰匙還給 PATH_SERVO！
                    // ==========================================================
                    if (jm.state == JumpState::IDLE) {
                        m_Group.pathMode = PathMode::PATH_SERVO; // 交還給上帝模式
                        jm.isRecovering = true; // 啟動軟著陸 (無縫接軌放電速度)

                        // 🛡️ 防呆保險：在最後一微秒，強制將座標鎖死在最初拍下的快照點！
                        int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                        int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

                        (*m_pContexts)[idxX].logicalCmdPos = jm.frozenPos[0];
                        (*m_pContexts)[idxY].logicalCmdPos = jm.frozenPos[1];
                        if (idxZ != -1) (*m_pContexts)[idxZ].logicalCmdPos = jm.frozenPos[2];
                    }
                }
            }

            // ======================================================
            // 4. 根據狀態，套用實體軸座標 (完美 3D 座標映射)
            // ======================================================
            if (jm.state != JumpState::IDLE)
            {
                double curX = jm.frozenPos[0], curY = jm.frozenPos[1], curZ = jm.frozenPos[2];
                double velX = 0, velY = 0, velZ = 0;

                int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

                double offset = jm.currentOffset;
                double jVel = jm.jumpVel;  // 保留規劃器的正負號

                if (jm.state == JumpState::B3_TO_CENTER)
                {
                    if (offset > jm.b3_distToCenter) offset = jm.b3_distToCenter; // 🛡️ 絕對夾死
                    if (jm.b3_distToCenter > 1e-6) {
                        double dirX = (jm.b3_centerPos[0] - jm.frozenPos[0]) / jm.b3_distToCenter;
                        double dirY = (jm.b3_centerPos[1] - jm.frozenPos[1]) / jm.b3_distToCenter;
                        double dirZ = (jm.b3_centerPos[2] - jm.frozenPos[2]) / jm.b3_distToCenter;

                        curX = jm.frozenPos[0] + offset * dirX;
                        curY = jm.frozenPos[1] + offset * dirY;
                        curZ = jm.frozenPos[2] + offset * dirZ;
                        velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                    }
                }
                else if (jm.state == JumpState::B3_TO_APEX || jm.state == JumpState::B3_DWELL || jm.state == JumpState::PAUSED_HOLD)
                {

                    if (offset > jm.b3_distToApex) offset = jm.b3_distToApex; // 🛡️ 絕對夾死

                    // 🛡️ DWELL 或 PAUSED_HOLD 期間強制鎖死在頂點，絕不跟著 offset 歸零亂跑！
                    double tempOffset = (jm.state == JumpState::B3_DWELL || jm.state == JumpState::PAUSED_HOLD) ? jm.b3_distToApex : offset;

                    curX = jm.b3_centerPos[0] + tempOffset * jm.b3_retractVector[0];
                    curY = jm.b3_centerPos[1] + tempOffset * jm.b3_retractVector[1];
                    curZ = jm.b3_centerPos[2] + tempOffset * jm.b3_retractVector[2];

                    // 速度強制為 0
                    if (jm.state == JumpState::B3_DWELL || jm.state == JumpState::PAUSED_HOLD) {
                        velX = 0; velY = 0; velZ = 0;
                    }
                    else {
                        velX = jVel * jm.b3_retractVector[0];
                        velY = jVel * jm.b3_retractVector[1];
                        velZ = jVel * jm.b3_retractVector[2];
                    }
                }
                else if (jm.state == JumpState::B3_FROM_APEX)
                {
                    if (offset > jm.b3_distToApex) offset = jm.b3_distToApex; // 🛡️ 絕對夾死
                    double dropOffset = jm.b3_distToApex - offset;

                    curX = jm.b3_centerPos[0] + dropOffset * jm.b3_retractVector[0];
                    curY = jm.b3_centerPos[1] + dropOffset * jm.b3_retractVector[1];
                    curZ = jm.b3_centerPos[2] + dropOffset * jm.b3_retractVector[2];

                    velX = -jVel * jm.b3_retractVector[0];
                    velY = -jVel * jm.b3_retractVector[1];
                    velZ = -jVel * jm.b3_retractVector[2];
                }
                else if (jm.state == JumpState::B3_TO_WORKPIECE)
                {
                    if (offset > jm.b3_distToCenter) offset = jm.b3_distToCenter; // 🛡️ 絕對夾死
                    if (jm.b3_distToCenter > 1e-6) {
                        double dirX = (jm.frozenPos[0] - jm.b3_centerPos[0]) / jm.b3_distToCenter;
                        double dirY = (jm.frozenPos[1] - jm.b3_centerPos[1]) / jm.b3_distToCenter;
                        double dirZ = (jm.frozenPos[2] - jm.b3_centerPos[2]) / jm.b3_distToCenter;




                        curX = jm.b3_centerPos[0] + offset * dirX;
                        curY = jm.b3_centerPos[1] + offset * dirY;
                        curZ = jm.b3_centerPos[2] + offset * dirZ;
                        velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                    }
                }

                // 🌟 寫入實體軸 (完全不變)
                (*m_pContexts)[idxX].logicalCmdPos = curX;
                (*m_pContexts)[idxX].logicalCmdVel = velX;
                (*m_pContexts)[idxY].logicalCmdPos = curY;
                (*m_pContexts)[idxY].logicalCmdVel = velY;
                if (idxZ != -1) {
                    (*m_pContexts)[idxZ].logicalCmdPos = curZ;
                    (*m_pContexts)[idxZ].logicalCmdVel = velZ;
                }

                vAxis.currentCmdPos = jm.triggerPos;
                vAxis.currentCmdVel = 0.0;
            }
            else
            {
                // B3 結束！完美交接回放電點
                m_Group.pathMode = PathMode::PATH_SERVO;
            }



        }
        // ======================================================
        // 🌟 B4 搖動/行星加工：專屬 5 階段狀態機與座標映射
        // ======================================================
        else if (jm.mode == JumpMode::B4_ORBITAL_DIAGONAL && ((jm.state >= JumpState::B4_TO_UPPER_CENTER && jm.state <= JumpState::B4_TO_WORKPIECE) || jm.state == JumpState::PAUSED_HOLD))
        {
            // --------------------------------------------------
            // 【第一部分：大腦 (狀態機與速度規劃)】
            // --------------------------------------------------
            std::vector<JumpSegment>* currentScript = nullptr;
            double finalTarget = 0.0;
            JumpState nextState = JumpState::IDLE;

            // 1. 選擇腳本與目標
            if (jm.state == JumpState::B4_TO_UPPER_CENTER)
            {
                currentScript = &jm.b4_toUpperSteps;
                finalTarget = jm.b4_distToUpper;
                nextState = JumpState::B4_TO_APEX;
            }
            else if (jm.state == JumpState::B4_TO_APEX)
            {
                currentScript = &jm.b4_toApexSteps;
                finalTarget = jm.b4_distToApex;
                nextState = JumpState::B4_DWELL;
            }
            else if (jm.state == JumpState::B4_FROM_APEX)
            {
                currentScript = &jm.b4_fromApexSteps;
                finalTarget = jm.b4_distToApex;
                nextState = JumpState::B4_TO_WORKPIECE;
            }
            else if (jm.state == JumpState::B4_TO_WORKPIECE)
            {
                currentScript = &jm.b4_toWorkpieceSteps;
                finalTarget = jm.b4_distToUpper;
                nextState = JumpState::IDLE;
            }

            // 2. 執行 DWELL 或 移動
            if (jm.state == JumpState::B4_DWELL)
            {
                jm.dwellTimer += dt * 1000.0;
                if (jm.dwellTimer >= jm.dwellTimeTarget)
                {
                    jm.state = JumpState::B4_FROM_APEX;
                    jm.currentStepIdx = 0; jm.currentOffset = 0.0; jm.jumpVel = 0.0;
                }
            }
            else if (currentScript != nullptr && !currentScript->empty())
            {
                JumpSegment& seg = (*currentScript)[jm.currentStepIdx];
                double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
                double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

                double stepBoundary = 0.0;
                for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary += (*currentScript)[i].distance;

                // 呼叫梯形規劃器
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 內部換檔
                if (jm.currentOffset >= stepBoundary && jm.currentStepIdx < (int)currentScript->size() - 1)
                {
                    jm.currentStepIdx++;
                }

                if (jm.currentOffset >= finalTarget - 1e-5)
                {

                    jm.currentStepIdx = 0;
                    jm.currentOffset = 0.0;
                    jm.jumpVel = 0.0;
                    jm.dwellTimer = 0.0;

                    // ==========================================================
                    // 🌟 [絕對關鍵：B4 暫停空中攔截網 (純數學真頂點)]
                    // ==========================================================
                    if (jm.state == JumpState::B4_TO_APEX && jm.isPauseMode)
                    {
                        jm.state = JumpState::PAUSED_HOLD;

                        jm.apexPos[0] = jm.b4_upperCenterPos[0] + jm.b4_distToApex * jm.b4_retractVector[0];
                        jm.apexPos[1] = jm.b4_upperCenterPos[1] + jm.b4_distToApex * jm.b4_retractVector[1];
                        if (m_Group.axisCount >= 3)
                        {
                            jm.apexPos[2] = jm.b4_upperCenterPos[2] + jm.b4_distToApex * jm.b4_retractVector[2];
                        }
                        //RtPrintf("[PAUSE_B4] Safely Holding at Math Apex.\n");
                    }
                    else
                    {
                        jm.state = nextState;
                    }

                    // 🚨 PATH_SERVO 交接！
                    if (jm.state == JumpState::IDLE)
                    {
                        m_Group.pathMode = PathMode::PATH_SERVO;
                        jm.isRecovering = true; // 啟動軟著陸

                        int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                        int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;
                        (*m_pContexts)[idxX].logicalCmdPos = jm.frozenPos[0];
                        (*m_pContexts)[idxY].logicalCmdPos = jm.frozenPos[1];
                        if (idxZ != -1) (*m_pContexts)[idxZ].logicalCmdPos = jm.frozenPos[2];
                    }
                }
            }

            // --------------------------------------------------
            // 【第二部分：手腳 (3D 座標映射與無敵夾鉗)】
            // --------------------------------------------------
            if (jm.state != JumpState::IDLE)
            {
                double curX = jm.frozenPos[0], curY = jm.frozenPos[1], curZ = jm.frozenPos[2];
                double velX = 0, velY = 0, velZ = 0;
                double offset = jm.currentOffset;
                double jVel = jm.jumpVel;

                int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

                if (jm.state == JumpState::B4_TO_UPPER_CENTER)
                {
                    if (offset > jm.b4_distToUpper) offset = jm.b4_distToUpper; // 🛡️ 絕對夾死
                    if (jm.b4_distToUpper > 1e-6) {
                        double dirX = (jm.b4_upperCenterPos[0] - jm.frozenPos[0]) / jm.b4_distToUpper;
                        double dirY = (jm.b4_upperCenterPos[1] - jm.frozenPos[1]) / jm.b4_distToUpper;
                        double dirZ = (jm.b4_upperCenterPos[2] - jm.frozenPos[2]) / jm.b4_distToUpper;

                        curX = jm.frozenPos[0] + offset * dirX;
                        curY = jm.frozenPos[1] + offset * dirY;
                        curZ = jm.frozenPos[2] + offset * dirZ;
                        velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                    }
                }
                else if (jm.state == JumpState::B4_TO_APEX || jm.state == JumpState::B4_DWELL || jm.state == JumpState::PAUSED_HOLD)
                {
                    if (offset > jm.b4_distToApex) offset = jm.b4_distToApex; // 🛡️ 絕對夾死
                    double tempOffset = (jm.state == JumpState::B4_DWELL || jm.state == JumpState::PAUSED_HOLD) ? jm.b4_distToApex : offset;

                    curX = jm.b4_upperCenterPos[0] + tempOffset * jm.b4_retractVector[0];
                    curY = jm.b4_upperCenterPos[1] + tempOffset * jm.b4_retractVector[1];
                    curZ = jm.b4_upperCenterPos[2] + tempOffset * jm.b4_retractVector[2];

                    if (jm.state == JumpState::B4_DWELL || jm.state == JumpState::PAUSED_HOLD) {
                        velX = 0; velY = 0; velZ = 0;
                    }
                    else {
                        velX = jVel * jm.b4_retractVector[0];
                        velY = jVel * jm.b4_retractVector[1];
                        velZ = jVel * jm.b4_retractVector[2];
                    }
                }
                else if (jm.state == JumpState::B4_FROM_APEX)
                {
                    if (offset > jm.b4_distToApex) offset = jm.b4_distToApex; // 🛡️ 絕對夾死
                    double dropOffset = jm.b4_distToApex - offset;

                    curX = jm.b4_upperCenterPos[0] + dropOffset * jm.b4_retractVector[0];
                    curY = jm.b4_upperCenterPos[1] + dropOffset * jm.b4_retractVector[1];
                    curZ = jm.b4_upperCenterPos[2] + dropOffset * jm.b4_retractVector[2];

                    velX = -jVel * jm.b4_retractVector[0];
                    velY = -jVel * jm.b4_retractVector[1];
                    velZ = -jVel * jm.b4_retractVector[2];
                }
                else if (jm.state == JumpState::B4_TO_WORKPIECE)
                {
                    if (offset > jm.b4_distToUpper) offset = jm.b4_distToUpper; // 🛡️ 絕對夾死
                    if (jm.b4_distToUpper > 1e-6) {
                        // 方向：從「上中心」指回「放電點」
                        double dirX = (jm.frozenPos[0] - jm.b4_upperCenterPos[0]) / jm.b4_distToUpper;
                        double dirY = (jm.frozenPos[1] - jm.b4_upperCenterPos[1]) / jm.b4_distToUpper;
                        double dirZ = (jm.frozenPos[2] - jm.b4_upperCenterPos[2]) / jm.b4_distToUpper;

                        curX = jm.b4_upperCenterPos[0] + offset * dirX;
                        curY = jm.b4_upperCenterPos[1] + offset * dirY;
                        curZ = jm.b4_upperCenterPos[2] + offset * dirZ;
                        velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                    }
                }

                // 🌟 寫入實體軸
                (*m_pContexts)[idxX].logicalCmdPos = curX;
                (*m_pContexts)[idxX].logicalCmdVel = velX;
                (*m_pContexts)[idxY].logicalCmdPos = curY;
                (*m_pContexts)[idxY].logicalCmdVel = velY;
                if (idxZ != -1) {
                    (*m_pContexts)[idxZ].logicalCmdPos = curZ;
                    (*m_pContexts)[idxZ].logicalCmdVel = velZ;
                }

                // 凍結虛擬主軸 (保持放電進度)
                vAxis.currentCmdPos = jm.triggerPos;
                vAxis.currentCmdVel = 0.0;
            }
        }
    }


    // =====================================================================
    // 🌟 🟢 [空間座標轉換過濾器] 🟢 🌟
    // =====================================================================
    // First perform an exact slot-limited pass-through.  In particular, a
    // one-axis Y command must never read stale axisIndices[1] and write an old
    // X logical velocity back into the physical command.
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
    {
        const int axisIndex = m_Group.axisIndices[slot];
        AxisContext& axis = (*m_pContexts)[axisIndex];
        axis.currentCmdPos = axis.logicalCmdPos;
        axis.currentCmdVel = axis.logicalCmdVel;
    }

    if (m_Group.enableTransform && m_Group.axisCount >= 2)
    {
        const int idxX = m_Group.axisIndices[0];
        const int idxY = m_Group.axisIndices[1];
        const int idxZ =
            (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;
        AxisContext& realX = (*m_pContexts)[idxX];
        AxisContext& realY = (*m_pContexts)[idxY];

        double logP[3] = { realX.logicalCmdPos, realY.logicalCmdPos, 0.0 };
        double logV[3] = { realX.logicalCmdVel, realY.logicalCmdVel, 0.0 };
        if (idxZ != -1)
        {
            logP[2] = (*m_pContexts)[idxZ].logicalCmdPos;
            logV[2] = (*m_pContexts)[idxZ].logicalCmdVel;
        }

        double physP[3] = { 0,0,0 };
        double physV[3] = { 0,0,0 };

        for (int r = 0; r < 3; ++r)
        {
            physP[r] = m_Group.transformOrigin[r];
            physV[r] = 0.0;
            for (int c = 0; c < 3; ++c)
            {
                physP[r] += m_Group.transformMatrix[r][c] * (logP[c] - m_Group.transformOrigin[c]);
                physV[r] += m_Group.transformMatrix[r][c] * logV[c];
            }
        }

        realX.currentCmdPos = physP[0];
        realX.currentCmdVel = physV[0];
        realY.currentCmdPos = physP[1];
        realY.currentCmdVel = physV[1];
        if (idxZ != -1)
        {
            (*m_pContexts)[idxZ].currentCmdPos = physP[2];
            (*m_pContexts)[idxZ].currentCmdVel = physV[2];
        }
    }


    // 3. 結束檢查 (G00 / G01 實體馬達準停確認)
    // =========================================================
    if (vAxis.state == MotionState::MotionState_IDLE)
    {
        bool allPhysicalInPos = true;

        // 遍歷檢查群組內的所有實體軸，是不是真的都走到終點了？
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = m_Group.axisIndices[i];
            AxisContext& realAxis = (*m_pContexts)[idx];

            // 計算大腦完美位置與實體馬達位置的落差
            double currentLag = std::abs(realAxis.currentCmdPos - realAxis.currentActPos);

            // 只要有一軸還沒擠進到位視窗，就判定尚未結束！
            if (!std::isfinite(realAxis.currentCmdPos) ||
                !std::isfinite(realAxis.currentActPos) ||
                !std::isfinite(realAxis.inPositionWindow_Pulse) ||
                realAxis.inPositionWindow_Pulse <= 0.0 ||
                !std::isfinite(realAxis.currentCmdVel) ||
                !std::isfinite(realAxis.logicalCmdVel) ||
                !std::isfinite(realAxis.targetVelocity) ||
                !std::isfinite(realAxis.targetEndVel) ||
                realAxis.isFault ||
                realAxis.isLagAlarm ||
                (realAxis.state != MotionState::MotionState_INTERPOLATING &&
                    realAxis.state != MotionState::MotionState_STOPPING &&
                    realAxis.state != MotionState::MotionState_IDLE) ||
                std::abs(realAxis.currentCmdVel) > 1.0 ||
                std::abs(realAxis.logicalCmdVel) > 1.0 ||
                !std::isfinite(currentLag) ||
                currentLag > realAxis.inPositionWindow_Pulse)
            {
                allPhysicalInPos = false;
                break; // 退堂！繼續等！
            }
        }

        // 🌟 神級收尾：大腦算完，且實體馬達"全部"都擠進視窗後，才准切換為 IDLE！
        if (allPhysicalInPos)
        {
            if (HasPendingExecutionEpochChange() ||
                GetCommandAuthorizationFailure(m_Group.currentCmd) !=
                MotionRejectReason::NONE)
            {
                return;
            }

            MotionCommand nextCommand{};
            const bool nextCommandValid =
                TryPeekNextMotionCommand(nextCommand) &&
                GetCommandAuthorizationFailure(nextCommand) ==
                MotionRejectReason::NONE &&
                IsMotionCommandConsumerGeometryValid(
                    nextCommand,
                    m_pContexts);
            const bool terminalMappingChanged =
                nextCommandValid &&
                !MotionCommandsHaveIdenticalAxisMapping(
                    m_Group.currentCmd,
                    nextCommand);

            LifecycleCommitReservationGuard lifecycleCommit(
                *this,
                m_Group.currentCmd.execution);
            if (!lifecycleCommit.IsAcquired() ||
                !TryCanonicalizeIdleAxisCommandState(vAxis))
            {
                return;
            }

            CompleteTrackedMotionCommand(
                m_Group.currentCmd);

            if (m_Group.enableHistory &&
                GetCommandAuthorizationFailure(m_Group.currentCmd) ==
                MotionRejectReason::NONE)
            {
                MotionCommand completedHistory = m_Group.currentCmd;
                completedHistory.replayTerminalAlreadyPublished = true;
                m_Group.historyQueue.push_back(completedHistory);
                if (m_Group.historyQueue.size() >
                    MOTION_COMMAND_HISTORY_LIMIT)
                {
                    m_Group.historyQueue.pop_front();
                }
            }

            if (terminalMappingChanged)
            {
                m_p1MappingBoundaryStopCount.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
                m_p1LastPreviousAxisMask.store(
                    BuildMotionCommandAxisMask(m_Group.currentCmd),
                    std::memory_order_release);
                m_p1LastNextAxisMask.store(
                    BuildMotionCommandAxisMask(nextCommand),
                    std::memory_order_release);
            }

            m_Group.isActive = false;
            std::uint64_t retiredAxisCount = 0ULL;
            for (int i = 0; i < m_Group.axisCount; ++i)
            {
                int idx = m_Group.axisIndices[i];
                const bool droppedFromNext =
                    terminalMappingChanged &&
                    !MotionCommandHasAxis(nextCommand, idx);
                if (!TryCanonicalizeInactivePhysicalAxisCommandState(
                    (*m_pContexts)[idx]))
                {
                    m_p1DroppedAxisRetirementFailureCount.fetch_add(
                        1ULL,
                        std::memory_order_relaxed);
                    m_p1LastOrphanAxisIndex.store(
                        idx,
                        std::memory_order_release);
                    lifecycleCommit.Release();
                    TriggerGroupMappingIntegrityEmergencyStop(idx, true);
                    return;
                }
                if (droppedFromNext)
                {
                    ++retiredAxisCount;
                }
            }
            if (retiredAxisCount != 0ULL)
            {
                m_p1DroppedAxisRetirementCount.fetch_add(
                    retiredAxisCount,
                    std::memory_order_relaxed);
            }
        }
        // 若實體軸還沒追上，就會維持在 MotionState_INTERPOLATING，
        // 繼續用柔軟的 Kp_G00 參數，讓馬達滑順地滑進終點。
    }
    // 🟢 監視器 3：物理突波偵測 (放在最尾巴)
    static double lastRealVelX = 0;
    double currentRealVelX = (*m_pContexts)[m_Group.axisIndices[0]].logicalCmdVel;


    lastRealVelX = currentRealVelX;





    static double last_log_VelX = 0;
    double current_log_VelX = (*m_pContexts)[m_Group.axisIndices[0]].logicalCmdVel;

    // 🌟 偵測門檻：一毫秒內速度變化超過 500,000 PPS (你可以視情況調整)
    if (std::abs(current_log_VelX - last_log_VelX) > 500000.0)
    {
        // 在這行下斷點 (F9)，程式就會停在速度驟降的那一瞬間
        //RtPrintf(">>> [HIT] Velocity Spike Detected! Before: %d | After: %d | Gap: %d\n",(int)last_log_VelX, (int)current_log_VelX, (int)(current_log_VelX - last_log_VelX));
    }
    last_log_VelX = current_log_VelX;
}

void MotionCore::SyncVirtualEndPosition()
{
    // 防呆：確認指標不是空的，且 vector 裡面真的有東西
    if (m_pContexts == nullptr || m_pContexts->empty()) return;

    // 🌟 修正：動態取得 vector 實際的大小，絕對不寫死 8！
    for (size_t i = 0; i < m_pContexts->size(); i++)
    {
        if ((*m_pContexts)[i].isExist)
        {
            // 將每一軸的預讀追蹤點 (lastQueuedPulse) 拉回馬達當下的真實邏輯位置
            (*m_pContexts)[i].lastQueuedPulse = (*m_pContexts)[i].logicalCmdPos;
        }
    }

    // DEBUG_PRINT("[Motion] Virtual End Position Synced! Axes checked: %zu\n", m_pContexts->size());
}
bool MotionCore::IsGroupStandstill() const
{
    // =========================================================
    // 1. 插補群組本身必須已經完全結束
    // =========================================================
    if (!IsGroupDone())
    {
        return false;
    }


    // =========================================================
    // 2. Context 必須有效
    // =========================================================
    if (m_pContexts == nullptr ||
        m_pContexts->empty())
    {
        return true;
    }


    // =========================================================
    // 3. 實際速度 Deadband
    //
    // currentActVel 是：
    //
    //   ΔEncoder / CYCLE_TIME_SEC
    //
    // 目前 Cycle = 250 us
    //
    // Encoder 只跳 1 Pulse：
    //
    //   1 / 0.00025 = 4000 PPS
    //
    // 所以不能用 1 PPS 判斷 Actual Velocity。
    //
    // 這裡允許約 1 Pulse / Servo Cycle 的量化抖動。
    // =========================================================
    const double actualVelDeadband_PPS = 1.0 / CYCLE_TIME_SEC;

    // 命令速度本身可以用很小的門檻
    const double commandVelDeadband_PPS = 1.0;


    // =========================================================
    // 4. 檢查所有存在的實體軸
    // =========================================================
    for (size_t i = 0; i < m_pContexts->size(); ++i)
    {
        const AxisContext& axis = (*m_pContexts)[i];
        // 未啟用軸不參與 Standstill 判斷
        if (!axis.isExist)
        {
            continue;
        }


        // -----------------------------------------------------
        // A. State 必須已經回 IDLE
        // -----------------------------------------------------
        if (axis.state != MotionState::MotionState_IDLE)
        {
            return false;
        }


        // -----------------------------------------------------
        // B. Command Velocity 必須停止
        // -----------------------------------------------------
        if (std::abs(axis.currentCmdVel) > commandVelDeadband_PPS)
        {
            return false;
        }


        // -----------------------------------------------------
        // C. Actual Velocity 也必須接近停止
        //
        // 注意：
        // 不使用 1 PPS，
        // 因為 Encoder 250us 差分本身就有量化。
        // -----------------------------------------------------
        if (std::abs(axis.currentActVel) > actualVelDeadband_PPS)
        {
            return false;
        }
    }


    // =========================================================
    // 所有條件都通過：
    //
    // Group Done
    // + Axis IDLE
    // + CmdVel ≈ 0
    // + ActVel ≈ 0
    //
    // 才是真正 Standstill
    // =========================================================
    return true;
}


// =============================================================================
// Stage NC-0.2J.5 - Formal NC settle producer (250 us owner only)
// =============================================================================
void MotionCore::UpdateNCSettleProducer(
    MotionStopSettlePublicationPayload& payload) noexcept
{
    const std::uint32_t currentGroupMask =
        BuildCurrentNCGroupAxisMask();

    const std::size_t commandIngressDepth =
        m_Group.cmdQueue.ingress_size();
    const std::size_t commandReplayDepth =
        m_Group.cmdQueue.replay_size();
    const std::size_t commandQueueDepth =
        commandIngressDepth + commandReplayDepth;
    const bool groupDrained =
        !m_Group.isActive && commandQueueDepth == 0U;

    const bool isNewRuntimeCycle =
        !m_ncSettleHasEvaluatedRuntimeCycle ||
        m_ncSettleRuntimeCycleTick !=
        m_ncSettleLastEvaluatedRuntimeCycleTick;

    if (isNewRuntimeCycle)
    {
        const bool runtimeUsable =
            m_ncSettleRuntimeObserved &&
            m_ncSettleRuntimeCycleValid &&
            m_ncSettleMotionPassCompleted;

        const MotionOwnerLease currentOwnerLease =
            GetMotionOwnerLease();
        const MotionExecutionEpoch currentExecutionEpoch =
            GetCurrentExecutionEpoch();
        const std::uint32_t existingAxisMask =
            BuildExistingNCAxisMask();
        const MotionExecutionIdentity currentGroupIdentity =
            m_Group.currentCmd.execution;
        const bool currentGroupIdentityCorrelated =
            currentGroupIdentity.IsAssigned() &&
            currentGroupIdentity.epoch == currentExecutionEpoch;
        const std::uint32_t correlatedCurrentGroupMask =
            currentGroupIdentityCorrelated
            ? currentGroupMask
            : 0U;

        if (correlatedCurrentGroupMask != 0U)
        {
            m_ncLastGroupScopeMask = correlatedCurrentGroupMask;
            m_ncLastGroupScopeExecutionEpoch = currentExecutionEpoch;
            m_ncLastGroupScopeExecutionIdentity = currentGroupIdentity;
        }

        std::uint32_t advisoryMaxPdoVelocity = 0U;
        if (m_pDrives != nullptr)
        {
            const std::size_t driveCount = (std::min)(
                m_pDrives->size(),
                static_cast<std::size_t>(MAX_AXES));
            for (std::size_t driveSlot = 0U;
                driveSlot < driveCount;
                ++driveSlot)
            {
                if ((*m_pDrives)[driveSlot].pOutput == nullptr)
                {
                    continue;
                }
                const std::int64_t value = static_cast<std::int64_t>(
                    (*m_pDrives)[driveSlot].pOutput->TargetVelocity);
                const std::uint32_t absoluteValue =
                    static_cast<std::uint32_t>(
                        value < 0 ? -value : value);
                advisoryMaxPdoVelocity = (std::max)(
                    advisoryMaxPdoVelocity,
                    absoluteValue);
            }
        }

        for (std::size_t profileIndex = 0U;
            profileIndex < MOTION_NC_SETTLE_PROFILE_COUNT;
            ++profileIndex)
        {
            const MotionNCSettleProfile profile =
                static_cast<MotionNCSettleProfile>(profileIndex);
            MotionNCSettleTracker& tracker =
                m_ncSettleTrackers[profileIndex];
            MotionNCSettleCounters& counters =
                m_ncSettleProducerCounters[profileIndex];
            MotionNCSettleSnapshot snapshot{};

            snapshot.profile = profile;
            snapshot.runtimeCycleTick = m_ncSettleRuntimeCycleTick;
            snapshot.runtimeObserved = m_ncSettleRuntimeObserved;
            snapshot.runtimeCycleValid = runtimeUsable;
            snapshot.runtimeCycleContiguous =
                m_ncSettleRuntimeCycleContiguous;
            snapshot.requiredCycles = MOTION_NC_SETTLE_REQUIRED_CYCLES;
            snapshot.groupActive = m_Group.isActive;
            snapshot.groupDrained = groupDrained;
            snapshot.feedrateOverride = m_Group.feedrateOverride;
            const bool virtualCommandVelocityFinite =
                std::isfinite(m_Group.virtualAxis.currentCmdVel) &&
                std::isfinite(m_Group.virtualAxis.logicalCmdVel);
            snapshot.virtualCommandVelocityAbsPps =
                virtualCommandVelocityFinite
                ? (std::max)(
                    std::abs(m_Group.virtualAxis.currentCmdVel),
                    std::abs(m_Group.virtualAxis.logicalCmdVel))
                : (std::numeric_limits<double>::infinity)();
            snapshot.maxCommandVelocityAbsPps =
                snapshot.virtualCommandVelocityAbsPps;
            snapshot.overrideZero =
                std::abs(snapshot.feedrateOverride) <= 1.0e-9;
            snapshot.virtualCommandStopped =
                std::isfinite(snapshot.virtualCommandVelocityAbsPps) &&
                snapshot.virtualCommandVelocityAbsPps <= 1.0;
            snapshot.allAxisCommandStopped = true;
            snapshot.safetyOrRecoveryPending =
                HasPendingSafetyOrRecoveryRequests();
            snapshot.commandQueueDepth = static_cast<std::uint32_t>(
                (std::min)(commandQueueDepth,
                    static_cast<std::size_t>(
                        (std::numeric_limits<std::uint32_t>::max)())));
            snapshot.commandIngressDepth = static_cast<std::uint32_t>(
                (std::min)(commandIngressDepth,
                    static_cast<std::size_t>(
                        (std::numeric_limits<std::uint32_t>::max)())));
            snapshot.commandReplayDepth = static_cast<std::uint32_t>(
                (std::min)(commandReplayDepth,
                    static_cast<std::size_t>(
                        (std::numeric_limits<std::uint32_t>::max)())));
            snapshot.advisoryMaxPdoTargetVelocityAbs =
                advisoryMaxPdoVelocity;

            ++counters.sampleCount;
            snapshot.sampleSequence = counters.sampleCount;

            MotionNCSettleBlocker blocker = MotionNCSettleBlocker::NONE;
            std::int32_t blockerAxisIndex = -1;

            if (!m_ncSettleRuntimeObserved)
            {
                blocker = MotionNCSettleBlocker::RUNTIME_NOT_OBSERVED;
            }
            else if (!runtimeUsable)
            {
                blocker = MotionNCSettleBlocker::RUNTIME_INVALID;
                ++counters.invalidRuntimeCycleCount;
            }
            else if (!m_ncSettleRuntimeCycleContiguous)
            {
                blocker = MotionNCSettleBlocker::RUNTIME_GAP;
                ++counters.runtimeGapCount;
            }

            if (profile == MotionNCSettleProfile::GROUP_COMPLETION)
            {
                // NC-0.2K.2.1: Program End is a machine-wide standstill
                // boundary.  The previous latest-segment mask could be Y-only
                // while an orphaned X still carried a command, allowing M30 to
                // finalize.  Keep the exact current Segment/Epoch/Owner
                // identity, but formally scope the 200-cycle proof to every
                // existing physical axis.  No payload field or ABI changes.
                const std::uint32_t scopeMask =
                    currentGroupIdentityCorrelated
                    ? existingAxisMask
                    : 0U;

                const bool identityChanged =
                    tracker.executionEpoch != currentExecutionEpoch ||
                    !tracker.ownerLease.Matches(currentOwnerLease) ||
                    !MotionExecutionIdentityExactlyMatches(
                        tracker.executionIdentity,
                        currentGroupIdentity);
                const bool scopeChanged = tracker.scopeMask != scopeMask;

                if (identityChanged || scopeChanged)
                {
                    ResetNCSettleCandidate(
                        profile,
                        identityChanged
                        ? MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH
                        : MotionNCSettleBlocker::SCOPE_CHANGED,
                        identityChanged,
                        scopeChanged);
                    tracker.executionEpoch = currentExecutionEpoch;
                    tracker.ownerLease = currentOwnerLease;
                    tracker.executionIdentity = currentGroupIdentity;
                    tracker.scopeMask = scopeMask;
                    tracker.requestAccepted = scopeMask != 0U;
                    if (blocker == MotionNCSettleBlocker::NONE)
                    {
                        blocker = identityChanged
                            ? MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH
                            : MotionNCSettleBlocker::SCOPE_CHANGED;
                    }
                }

                if (scopeMask == 0U &&
                    blocker == MotionNCSettleBlocker::NONE)
                {
                    blocker = MotionNCSettleBlocker::SCOPE_EMPTY;
                }
            }

            snapshot.requestSequence = tracker.requestSequence;
            snapshot.executionEpoch = tracker.executionEpoch;
            snapshot.owner = tracker.ownerLease.owner;
            snapshot.ownerGeneration = tracker.ownerLease.generation;
            snapshot.scopeMask = tracker.scopeMask;
            snapshot.requestAccepted = tracker.requestAccepted;

            const bool resetAuthoritativeCurrent =
                profile == MotionNCSettleProfile::RESET_ALL &&
                m_activeResetNCSettleRequest.profile ==
                MotionNCSettleProfile::RESET_ALL &&
                m_activeResetNCSettleRequest.requestSequence !=
                MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
                m_activeResetNCSettleRequest.executionEpoch ==
                currentExecutionEpoch &&
                m_activeResetNCSettleRequest.ownerLease.IsValid() &&
                m_activeResetNCSettleRequest.ownerLease.owner ==
                MotionOwner::SAFETY &&
                m_activeResetNCSettleRequest.ownerLease.Matches(
                    currentOwnerLease);
            const bool resetInternalCoherent =
                profile == MotionNCSettleProfile::RESET_ALL &&
                m_activeResetNCSettleRequest.requestSequence ==
                tracker.requestSequence &&
                m_activeResetNCSettleRequest.requestSequence ==
                m_ncResetRebaseAckProducer.requestSequence &&
                tracker.executionEpoch == currentExecutionEpoch &&
                m_ncResetRebaseAckProducer.executionEpoch ==
                currentExecutionEpoch &&
                tracker.ownerLease.Matches(
                    m_activeResetNCSettleRequest.ownerLease) &&
                m_ncResetRebaseAckProducer.owner ==
                currentOwnerLease.owner &&
                m_ncResetRebaseAckProducer.ownerGeneration ==
                currentOwnerLease.generation &&
                tracker.requestAccepted &&
                m_ncResetRebaseAckProducer.requestAccepted;
            const bool resetTupleCurrent =
                resetAuthoritativeCurrent && resetInternalCoherent;
            const bool resetPhaseRequiresCoherence =
                profile == MotionNCSettleProfile::RESET_ALL &&
                (m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::WAIT_PREPROOF ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::CLEARING_BUFFERS ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::WAIT_POSTPROOF ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::ACKNOWLEDGED);
            const bool resetInvariantFailure =
                resetPhaseRequiresCoherence &&
                resetAuthoritativeCurrent &&
                !resetInternalCoherent;

            if (profile != MotionNCSettleProfile::GROUP_COMPLETION)
            {
                if (tracker.requestSequence ==
                    MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
                    !tracker.requestAccepted)
                {
                    if (blocker == MotionNCSettleBlocker::NONE)
                    {
                        blocker = MotionNCSettleBlocker::REQUEST_MISSING;
                    }
                }
                else if (tracker.executionEpoch != currentExecutionEpoch)
                {
                    if (blocker == MotionNCSettleBlocker::NONE)
                    {
                        blocker =
                            MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH;
                    }
                }
                else if (!tracker.ownerLease.Matches(currentOwnerLease))
                {
                    if (blocker == MotionNCSettleBlocker::NONE)
                    {
                        blocker = MotionNCSettleBlocker::OWNER_LEASE_MISMATCH;
                    }
                }
                else if (profile ==
                    MotionNCSettleProfile::FEED_HOLD_GROUP)
                {
                    if (correlatedCurrentGroupMask != 0U &&
                        correlatedCurrentGroupMask != tracker.scopeMask)
                    {
                        if (blocker == MotionNCSettleBlocker::NONE)
                        {
                            blocker = MotionNCSettleBlocker::SCOPE_CHANGED;
                        }
                    }
                    if (!tracker.executionIdentity.IsAssigned() ||
                        !currentGroupIdentityCorrelated ||
                        !MotionExecutionIdentityExactlyMatches(
                            tracker.executionIdentity,
                            currentGroupIdentity))
                    {
                        if (blocker == MotionNCSettleBlocker::NONE)
                        {
                            blocker = MotionNCSettleBlocker::
                                EXECUTION_EPOCH_MISMATCH;
                        }
                    }
                }
                else if (profile == MotionNCSettleProfile::RESET_ALL &&
                    existingAxisMask != tracker.scopeMask)
                {
                    if (blocker == MotionNCSettleBlocker::NONE)
                    {
                        blocker = MotionNCSettleBlocker::SCOPE_CHANGED;
                    }
                }
            }

            if (resetInvariantFailure)
            {
                blocker = MotionNCSettleBlocker::REQUEST_MISSING;
            }

            if (profile == MotionNCSettleProfile::RESET_ALL)
            {
                snapshot.rebasePreProofPassed =
                    m_ncResetScalarRebaseApplied ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::CLEARING_BUFFERS ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::WAIT_POSTPROOF ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::ACKNOWLEDGED;
                snapshot.rebasePostProofPassed =
                    m_ncResetRebaseAckProducer.postVerifyPassed;

                if (blocker == MotionNCSettleBlocker::NONE &&
                    HasNCResetActiveCompensation())
                {
                    blocker = MotionNCSettleBlocker::COMPENSATION_ACTIVE;
                }
                if (blocker == MotionNCSettleBlocker::NONE &&
                    HasNCResetUnsupportedMotion())
                {
                    blocker =
                        MotionNCSettleBlocker::PATH_RUNTIME_UNSUPPORTED;
                }

                if (m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::BLOCKED ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::SUPERSEDED)
                {
                    blocker = m_ncResetRebaseAckProducer.failureBlocker;
                    if (blocker == MotionNCSettleBlocker::NONE)
                    {
                        blocker = MotionNCSettleBlocker::RESET_UNSUPPORTED;
                    }
                }
                else if (m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::CLEARING_BUFFERS)
                {
                    blocker = MotionNCSettleBlocker::REBASE_IN_PROGRESS;
                }
                else if (m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::ACKNOWLEDGED)
                {
                    // Keep evaluating all health, command, following-error,
                    // freeze and excursion invalidators below.  ACK is a
                    // current proof, not a permanently sticky stale token.
                }
            }

            if (blocker == MotionNCSettleBlocker::NONE &&
                profile == MotionNCSettleProfile::GROUP_COMPLETION &&
                !groupDrained)
            {
                blocker = m_Group.isActive
                    ? MotionNCSettleBlocker::GROUP_ACTIVE
                    : MotionNCSettleBlocker::COMMAND_QUEUE;
            }
            if (blocker == MotionNCSettleBlocker::NONE &&
                profile == MotionNCSettleProfile::RESET_ALL &&
                (commandQueueDepth != 0U ||
                    m_axisCommandChannel.command_size() != 0U))
            {
                blocker = MotionNCSettleBlocker::COMMAND_QUEUE;
            }
            if (blocker == MotionNCSettleBlocker::NONE &&
                profile == MotionNCSettleProfile::FEED_HOLD_GROUP &&
                (!std::isfinite(m_Group.feedrateOverride) ||
                    std::abs(m_Group.feedrateOverride) > 1.0e-9))
            {
                blocker = MotionNCSettleBlocker::FEED_OVERRIDE_NONZERO;
            }
            if (blocker == MotionNCSettleBlocker::NONE &&
                snapshot.safetyOrRecoveryPending)
            {
                blocker =
                    MotionNCSettleBlocker::SAFETY_OR_RECOVERY_PENDING;
            }
            if (m_Group.virtualAxis.isFault ||
                m_Group.virtualAxis.state == MotionState::MotionState_ERROR ||
                m_Group.virtualAxis.state == MotionState::MotionState_ESTOP)
            {
                snapshot.anyAxisFaultOrEstop = true;
                if (blocker == MotionNCSettleBlocker::NONE)
                {
                    blocker = MotionNCSettleBlocker::AXIS_FAULT_OR_ESTOP;
                }
            }
            if (blocker == MotionNCSettleBlocker::NONE &&
                (!std::isfinite(m_Group.virtualAxis.currentCmdVel) ||
                    !std::isfinite(m_Group.virtualAxis.logicalCmdVel) ||
                    std::abs(m_Group.virtualAxis.currentCmdVel) > 1.0 ||
                    std::abs(m_Group.virtualAxis.logicalCmdVel) > 1.0))
            {
                blocker = MotionNCSettleBlocker::COMMAND_VELOCITY;
            }

            const bool resetPostProof =
                profile == MotionNCSettleProfile::RESET_ALL &&
                (m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::WAIT_POSTPROOF ||
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::ACKNOWLEDGED);
            const bool requireFollowingError =
                profile != MotionNCSettleProfile::RESET_ALL ||
                resetPostProof;

            if (m_pContexts == nullptr || existingAxisMask == 0U)
            {
                if (blocker == MotionNCSettleBlocker::NONE)
                {
                    blocker = MotionNCSettleBlocker::RESET_UNSUPPORTED;
                }
            }
            else
            {
                const std::size_t axisCount = (std::min)(
                    m_pContexts->size(),
                    static_cast<std::size_t>(MAX_AXES));
                for (std::size_t axisSlot = 0U;
                    axisSlot < axisCount;
                    ++axisSlot)
                {
                    const AxisContext& axis = (*m_pContexts)[axisSlot];
                    if (!axis.isExist)
                    {
                        continue;
                    }

                    const std::uint32_t axisBit =
                        1U << static_cast<unsigned>(axisSlot);
                    const bool scoped =
                        (tracker.scopeMask & axisBit) != 0U;
                    const std::int32_t publishedAxisIndex =
                        static_cast<std::int32_t>(axis.axisIndex);
                    const bool commandVelocityFinite =
                        std::isfinite(axis.currentCmdVel) &&
                        std::isfinite(axis.logicalCmdVel);
                    const double commandVelocity =
                        commandVelocityFinite
                        ? (std::max)(
                            std::abs(axis.currentCmdVel),
                            std::abs(axis.logicalCmdVel))
                        : (std::numeric_limits<double>::infinity)();
                    const double actualVelocity =
                        std::abs(axis.currentActVel);

                    if (commandVelocity >
                        snapshot.maxCommandVelocityAbsPps ||
                        (snapshot.worstCommandVelocityAxisIndex < 0 &&
                            snapshot.maxCommandVelocityAbsPps == 0.0))
                    {
                        snapshot.worstCommandVelocityAxisIndex =
                            publishedAxisIndex;
                        snapshot.maxCommandVelocityAbsPps = commandVelocity;
                    }
                    snapshot.advisoryMaxActualVelocityAbsPps = (std::max)(
                        snapshot.advisoryMaxActualVelocityAbsPps,
                        actualVelocity);

                    if (!scoped)
                    {
                        continue;
                    }

                    if (blocker == MotionNCSettleBlocker::NONE &&
                        (axis.isFault ||
                            axis.state == MotionState::MotionState_ERROR ||
                            axis.state == MotionState::MotionState_ESTOP))
                    {
                        blocker = MotionNCSettleBlocker::AXIS_FAULT_OR_ESTOP;
                        blockerAxisIndex = publishedAxisIndex;
                    }
                    if (axis.isFault ||
                        axis.state == MotionState::MotionState_ERROR ||
                        axis.state == MotionState::MotionState_ESTOP)
                    {
                        snapshot.anyAxisFaultOrEstop = true;
                    }
                    if (blocker == MotionNCSettleBlocker::NONE &&
                        !axis.isServoOn)
                    {
                        blocker = MotionNCSettleBlocker::AXIS_SERVO_OFF;
                        blockerAxisIndex = publishedAxisIndex;
                    }
                    if (blocker == MotionNCSettleBlocker::NONE &&
                        (!std::isfinite(commandVelocity) ||
                            commandVelocity > 1.0))
                    {
                        blocker = MotionNCSettleBlocker::COMMAND_VELOCITY;
                        blockerAxisIndex = publishedAxisIndex;
                    }
                    if (!std::isfinite(commandVelocity) ||
                        commandVelocity > 1.0)
                    {
                        snapshot.allAxisCommandStopped = false;
                    }

                    const bool stateAllowed =
                        axis.state == MotionState::MotionState_IDLE ||
                        ((profile ==
                            MotionNCSettleProfile::FEED_HOLD_GROUP ||
                            profile == MotionNCSettleProfile::RESET_ALL) &&
                            (axis.state ==
                                MotionState::MotionState_INTERPOLATING ||
                                axis.state ==
                                MotionState::MotionState_STOPPING));
                    if (blocker == MotionNCSettleBlocker::NONE &&
                        !stateAllowed)
                    {
                        blocker = MotionNCSettleBlocker::AXIS_STATE;
                        blockerAxisIndex = publishedAxisIndex;
                    }

                    const double window = axis.inPositionWindow_Pulse;
                    const double followingError =
                        std::abs(axis.currentCmdPos - axis.currentActPos);
                    const double followingRatio =
                        (std::isfinite(window) && window > 0.0)
                        ? followingError / window
                        : (std::numeric_limits<double>::max)();
                    const double publishedWorstRatio =
                        (snapshot.worstFollowingWindowPulse > 0.0)
                        ? snapshot.worstFollowingErrorAbsPulse /
                        snapshot.worstFollowingWindowPulse
                        : -1.0;
                    if (snapshot.worstFollowingErrorAxisIndex < 0 ||
                        followingRatio > publishedWorstRatio)
                    {
                        snapshot.worstFollowingErrorAxisIndex =
                            publishedAxisIndex;
                        snapshot.worstFollowingErrorAbsPulse =
                            followingError;
                        snapshot.worstFollowingWindowPulse = window;
                    }

                    if (blocker == MotionNCSettleBlocker::NONE &&
                        (!std::isfinite(window) || window <= 0.0))
                    {
                        blocker =
                            MotionNCSettleBlocker::IN_POSITION_WINDOW_INVALID;
                        blockerAxisIndex = publishedAxisIndex;
                    }
                    if (blocker == MotionNCSettleBlocker::NONE &&
                        requireFollowingError &&
                        (!std::isfinite(followingError) ||
                            followingError > window))
                    {
                        blocker = MotionNCSettleBlocker::FOLLOWING_ERROR;
                        blockerAxisIndex = publishedAxisIndex;
                    }
                }
            }

            if (blocker == MotionNCSettleBlocker::NONE &&
                !tracker.candidate)
            {
                tracker.candidate = true;
                tracker.dwellCycles = 0U;
                ++counters.candidateStartCount;

                if (m_pContexts != nullptr)
                {
                    const std::size_t axisCount = (std::min)(
                        m_pContexts->size(),
                        static_cast<std::size_t>(MAX_AXES));
                    for (std::size_t axisSlot = 0U;
                        axisSlot < axisCount;
                        ++axisSlot)
                    {
                        if ((tracker.scopeMask &
                            (1U << static_cast<unsigned>(axisSlot))) == 0U)
                        {
                            continue;
                        }
                        const AxisContext& axis =
                            (*m_pContexts)[axisSlot];
                        tracker.commandAnchorPulse[axisSlot] =
                            axis.currentCmdPos;
                        tracker.actualMinimumPulse[axisSlot] =
                            axis.currentActPos;
                        tracker.actualMaximumPulse[axisSlot] =
                            axis.currentActPos;
                    }
                }
            }

            if (blocker == MotionNCSettleBlocker::NONE &&
                m_pContexts != nullptr)
            {
                const std::size_t axisCount = (std::min)(
                    m_pContexts->size(),
                    static_cast<std::size_t>(MAX_AXES));
                for (std::size_t axisSlot = 0U;
                    axisSlot < axisCount;
                    ++axisSlot)
                {
                    if ((tracker.scopeMask &
                        (1U << static_cast<unsigned>(axisSlot))) == 0U)
                    {
                        continue;
                    }

                    const AxisContext& axis = (*m_pContexts)[axisSlot];
                    const std::int32_t publishedAxisIndex =
                        static_cast<std::int32_t>(axis.axisIndex);
                    if (!std::isfinite(axis.currentCmdPos) ||
                        std::abs(axis.currentCmdPos -
                            tracker.commandAnchorPulse[axisSlot]) > 0.5)
                    {
                        blocker =
                            MotionNCSettleBlocker::COMMAND_POSITION_CHANGED;
                        blockerAxisIndex = publishedAxisIndex;
                        break;
                    }

                    tracker.actualMinimumPulse[axisSlot] = (std::min)(
                        tracker.actualMinimumPulse[axisSlot],
                        axis.currentActPos);
                    tracker.actualMaximumPulse[axisSlot] = (std::max)(
                        tracker.actualMaximumPulse[axisSlot],
                        axis.currentActPos);
                    const double excursion =
                        tracker.actualMaximumPulse[axisSlot] -
                        tracker.actualMinimumPulse[axisSlot];
                    const double window = axis.inPositionWindow_Pulse;
                    const double excursionLimit = (std::min)(
                        0.5 * window,
                        (std::max)(4.0, 0.25 * window));

                    if (snapshot.worstActualExcursionAxisIndex < 0 ||
                        excursion > snapshot.worstActualExcursionPulse)
                    {
                        snapshot.worstActualExcursionAxisIndex =
                            publishedAxisIndex;
                        snapshot.worstActualExcursionPulse = excursion;
                        snapshot.worstActualExcursionLimitPulse =
                            excursionLimit;
                    }
                    if (!std::isfinite(axis.currentActPos) ||
                        !std::isfinite(excursionLimit) ||
                        excursion > excursionLimit)
                    {
                        blocker = MotionNCSettleBlocker::ACTUAL_EXCURSION;
                        blockerAxisIndex = publishedAxisIndex;
                        break;
                    }
                }
            }

            if (blocker != MotionNCSettleBlocker::NONE)
            {
                if (profile == MotionNCSettleProfile::RESET_ALL &&
                    resetTupleCurrent &&
                    m_ncResetRebasePhase ==
                    MotionNCResetRebasePhase::ACKNOWLEDGED)
                {
                    m_ncResetRebasePhase =
                        MotionNCResetRebasePhase::WAIT_POSTPROOF;
                    m_ncResetRebaseAckProducer.phase =
                        MotionNCResetRebasePhase::WAIT_POSTPROOF;
                    m_ncResetRebaseAckProducer.postVerifyPassed = false;
                    m_ncResetRebaseAckProducer.acknowledged = false;
                    m_ncResetRebaseAckProducer.acked = false;
                }
                if (profile == MotionNCSettleProfile::RESET_ALL &&
                    (resetInvariantFailure ||
                        (resetTupleCurrent &&
                            (blocker ==
                                MotionNCSettleBlocker::COMPENSATION_ACTIVE ||
                                blocker ==
                                MotionNCSettleBlocker::PATH_RUNTIME_UNSUPPORTED ||
                                blocker ==
                                MotionNCSettleBlocker::AXIS_FAULT_OR_ESTOP))))
                {
                    m_ncResetRebasePhase =
                        MotionNCResetRebasePhase::BLOCKED;
                    m_ncResetRebaseAckProducer.phase =
                        MotionNCResetRebasePhase::BLOCKED;
                    m_ncResetRebaseAckProducer.failureBlocker = blocker;
                    m_ncResetRebaseAckProducer.blocked = true;
                    m_ncResetRebaseAckProducer.compensationBlocked =
                        blocker ==
                        MotionNCSettleBlocker::COMPENSATION_ACTIVE;
                    m_ncResetRebaseAckProducer.postVerifyPassed = false;
                    m_ncResetRebaseAckProducer.acknowledged = false;
                    m_ncResetRebaseAckProducer.acked = false;
                }
                ResetNCSettleCandidate(
                    profile,
                    blocker,
                    blocker == MotionNCSettleBlocker::
                    EXECUTION_EPOCH_MISMATCH ||
                    blocker == MotionNCSettleBlocker::OWNER_LEASE_MISMATCH,
                    blocker == MotionNCSettleBlocker::SCOPE_CHANGED);
            }
            else
            {
                if (tracker.dwellCycles <
                    MOTION_NC_SETTLE_REQUIRED_CYCLES)
                {
                    ++tracker.dwellCycles;
                }

                if (tracker.dwellCycles >=
                    MOTION_NC_SETTLE_REQUIRED_CYCLES &&
                    !tracker.settled)
                {
                    if (profile == MotionNCSettleProfile::RESET_ALL &&
                        m_ncResetRebasePhase ==
                        MotionNCResetRebasePhase::WAIT_PREPROOF)
                    {
                        m_ncResetRebasePhase =
                            MotionNCResetRebasePhase::CLEARING_BUFFERS;
                        m_ncResetRebaseAckProducer.phase =
                            MotionNCResetRebasePhase::CLEARING_BUFFERS;
                        ResetNCSettleCandidate(
                            profile,
                            MotionNCSettleBlocker::REBASE_IN_PROGRESS);
                        blocker =
                            MotionNCSettleBlocker::REBASE_IN_PROGRESS;
                    }
                    else if (profile ==
                        MotionNCSettleProfile::RESET_ALL &&
                        m_ncResetRebasePhase ==
                        MotionNCResetRebasePhase::WAIT_POSTPROOF)
                    {
                        if (VerifyNCResetRebaseState())
                        {
                            tracker.settled = true;
                            tracker.proofSequence = ++m_ncNextProofSequence;
                            ++counters.proofRiseCount;
                            m_ncResetRebasePhase =
                                MotionNCResetRebasePhase::ACKNOWLEDGED;
                            m_ncResetRebaseAckProducer.phase =
                                MotionNCResetRebasePhase::ACKNOWLEDGED;
                            m_ncResetRebaseAckProducer.postVerifyPassed = true;
                            m_ncResetRebaseAckProducer.acknowledged = true;
                            m_ncResetRebaseAckProducer.acked = true;
                        }
                        else
                        {
                            ResetNCSettleCandidate(
                                profile,
                                MotionNCSettleBlocker::REBASE_VERIFY_FAILED);
                            blocker = MotionNCSettleBlocker::
                                REBASE_VERIFY_FAILED;
                            m_ncResetRebasePhase =
                                MotionNCResetRebasePhase::BLOCKED;
                            m_ncResetRebaseAckProducer.phase =
                                MotionNCResetRebasePhase::BLOCKED;
                            m_ncResetRebaseAckProducer.failureBlocker =
                                blocker;
                            m_ncResetRebaseAckProducer.blocked = true;
                            m_ncResetRebaseAckProducer.postVerifyPassed =
                                false;
                            m_ncResetRebaseAckProducer.acknowledged = false;
                            m_ncResetRebaseAckProducer.acked = false;
                        }
                    }
                    else
                    {
                        tracker.settled = true;
                        tracker.proofSequence = ++m_ncNextProofSequence;
                        ++counters.proofRiseCount;
                    }
                }
            }

            snapshot.blocker = blocker;
            snapshot.blockerAxisIndex = blockerAxisIndex;
            snapshot.candidate = tracker.candidate;
            snapshot.settled = tracker.settled;
            snapshot.dwellCycles = tracker.dwellCycles;
            snapshot.proofSequence = tracker.proofSequence;
            snapshot.rebasePreProofPassed =
                profile == MotionNCSettleProfile::RESET_ALL &&
                m_ncResetScalarRebaseApplied;
            snapshot.rebasePostProofPassed =
                profile == MotionNCSettleProfile::RESET_ALL &&
                m_ncResetRebaseAckProducer.postVerifyPassed;

            m_ncSettlePublishedSnapshots[profileIndex] = snapshot;
        }

        m_ncSettleLastEvaluatedRuntimeCycleTick =
            m_ncSettleRuntimeCycleTick;
        m_ncSettleHasEvaluatedRuntimeCycle = true;
    }

    payload.ncSettleSnapshots = m_ncSettlePublishedSnapshots;
    payload.ncSettleCounters = m_ncSettleProducerCounters;
    payload.resetRebaseAck = m_ncResetRebaseAckProducer;
}


// =============================================================================
// Stage NC-0.2J.4/J.5 - Atomic Motion Stop / Settle Publication
// =============================================================================
void MotionCore::PublishStopSettleEvidence() noexcept
{
    MotionStopSettlePublicationPayload payload{};
    MotionStopSettleSnapshot& snapshot = payload.snapshot;
    MotionEmergencyStopEvidence& emergency =
        payload.emergencyStopEvidence;
    MotionEmergencyStopCounters& emergencyCounters =
        payload.emergencyStopCounters;

    const double commandVelocityDeadbandPps = 1.0;
    const double actualVelocityDeadbandPps = 1.0 / CYCLE_TIME_SEC;

    snapshot.commandVelocityDeadbandPps = commandVelocityDeadbandPps;
    snapshot.actualVelocityDeadbandPps = actualVelocityDeadbandPps;
    snapshot.groupActive = m_Group.isActive;

    emergencyCounters.requestAttempts =
        m_emergencyStopRequestAttemptCount.load(
            std::memory_order_acquire);
    emergencyCounters.requestsPublished =
        m_emergencyStopRequestPublishedCount.load(
            std::memory_order_acquire);
    emergencyCounters.requestsCoalesced =
        m_emergencyStopRequestCoalescedCount.load(
            std::memory_order_acquire);
    emergencyCounters.rtApplications =
        m_emergencyStopRTApplicationCount;
    emergencyCounters.epochInvalidations =
        m_emergencyStopEpochInvalidationCount;

    const MotionOwnerLease emergencyCurrentOwner =
        GetMotionOwnerLease();
    emergency.currentExecutionEpoch =
        GetCurrentExecutionEpoch();
    emergency.lastAppliedExecutionEpoch =
        m_emergencyStopLastAppliedExecutionEpoch;
    emergency.currentOwner = emergencyCurrentOwner.owner;
    emergency.currentOwnerGeneration =
        emergencyCurrentOwner.generation;
    emergency.lastAppliedOwner =
        m_emergencyStopLastAppliedOwnerLease.owner;
    emergency.lastAppliedOwnerGeneration =
        m_emergencyStopLastAppliedOwnerLease.generation;
    emergency.requestPending =
        m_emergencyStopAllPending.load(
            std::memory_order_acquire);
    emergency.requestInProgress =
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire);
    emergency.lastApplyHadExecutionToInvalidate =
        m_emergencyStopLastApplyHadExecutionToInvalidate;
    emergency.groupActive = m_Group.isActive;
    emergency.groupEmergencyStopped =
        m_Group.virtualAxis.state ==
        MotionState::MotionState_ESTOP;
    emergency.groupError =
        m_Group.virtualAxis.state ==
        MotionState::MotionState_ERROR;
    emergency.virtualCommandZero =
        std::abs(m_Group.virtualAxis.currentCmdVel) <=
        commandVelocityDeadbandPps &&
        std::abs(m_Group.virtualAxis.logicalCmdVel) <=
        commandVelocityDeadbandPps &&
        std::abs(m_Group.virtualAxis.targetVelocity) <=
        commandVelocityDeadbandPps;
    emergency.virtualTargetSealed =
        std::abs(
            m_Group.virtualAxis.planningPos -
            m_Group.virtualAxis.currentCmdPos) <= 1.0 &&
        std::abs(
            m_Group.virtualAxis.finalTargetPos -
            m_Group.virtualAxis.currentCmdPos) <= 1.0;

    const std::size_t commandIngressDepth =
        m_Group.cmdQueue.ingress_size();
    const std::size_t commandReplayDepth =
        m_Group.cmdQueue.replay_size();
    const std::size_t commandQueueDepth =
        commandIngressDepth + commandReplayDepth;
    const std::size_t uint32Maximum =
        static_cast<std::size_t>(
            (std::numeric_limits<std::uint32_t>::max)());

    const auto ToPublishedCount =
        [uint32Maximum](std::size_t value) noexcept -> std::uint32_t
    {
        return static_cast<std::uint32_t>(
            (value > uint32Maximum) ? uint32Maximum : value);
    };

    snapshot.commandQueueDepth = ToPublishedCount(commandQueueDepth);
    snapshot.commandIngressDepth = ToPublishedCount(commandIngressDepth);
    snapshot.commandReplayDepth = ToPublishedCount(commandReplayDepth);

    int groupAxisLimit = m_Group.axisCount;
    if (groupAxisLimit < 0)
    {
        groupAxisLimit = 0;
    }
    if (groupAxisLimit > MAX_AXES)
    {
        groupAxisLimit = MAX_AXES;
    }
    snapshot.groupAxisCount =
        static_cast<std::uint32_t>(groupAxisLimit);

    MotionStopSettlePrimaryBlocker primaryBlocker =
        MotionStopSettlePrimaryBlocker::NONE;
    std::int32_t primaryAxisIndex = -1;
    MotionState primaryAxisState = MotionState::MotionState_IDLE;

    // This order mirrors IsGroupDone(): active segment first, then any queued
    // ingress/replay command.  It is the exact first group-level reason that
    // IsGroupStandstill cannot pass.
    if (snapshot.groupActive)
    {
        primaryBlocker =
            MotionStopSettlePrimaryBlocker::GROUP_ACTIVE;
    }
    else if (commandQueueDepth != 0U)
    {
        primaryBlocker =
            MotionStopSettlePrimaryBlocker::COMMAND_QUEUE;
    }

    if (m_pContexts != nullptr)
    {
        for (std::size_t i = 0U; i < m_pContexts->size(); ++i)
        {
            const AxisContext& axis = (*m_pContexts)[i];
            if (!axis.isExist)
            {
                continue;
            }

            ++snapshot.existingAxisCount;

            std::uint32_t axisBit = 0U;
            if (i < static_cast<std::size_t>(32U))
            {
                axisBit =
                    static_cast<std::uint32_t>(
                        1U << static_cast<unsigned>(i));
                emergency.existingAxisMask |= axisBit;

                if (axis.state == MotionState::MotionState_ESTOP)
                {
                    emergency.estopAxisMask |= axisBit;
                }
                if (axis.state == MotionState::MotionState_ERROR)
                {
                    emergency.errorAxisMask |= axisBit;
                }
                if (axis.isFault)
                {
                    emergency.faultAxisMask |= axisBit;
                }
                if (axis.isLagAlarm)
                {
                    emergency.lagAlarmAxisMask |= axisBit;
                }

                const bool axisCommandZero =
                    std::abs(axis.currentCmdVel) <=
                    commandVelocityDeadbandPps &&
                    std::abs(axis.logicalCmdVel) <=
                    commandVelocityDeadbandPps &&
                    std::abs(axis.targetVelocity) <=
                    commandVelocityDeadbandPps;
                if (axisCommandZero)
                {
                    emergency.commandZeroAxisMask |= axisBit;
                }

                const bool axisTargetSealed =
                    std::abs(
                        axis.planningPos -
                        axis.currentCmdPos) <= 1.0 &&
                    std::abs(
                        axis.finalTargetPos -
                        axis.currentCmdPos) <= 1.0;
                if (axisTargetSealed)
                {
                    emergency.targetSealedAxisMask |= axisBit;
                }
            }

            const std::int32_t publishedAxisIndex =
                static_cast<std::int32_t>(axis.axisIndex);
            const double commandVelocityAbsPps =
                std::abs(axis.currentCmdVel);
            const double actualVelocityAbsPps =
                std::abs(axis.currentActVel);

            if (snapshot.worstCommandVelocityAxisIndex < 0 ||
                commandVelocityAbsPps >
                snapshot.maxAxisCommandVelocityAbsPps)
            {
                snapshot.worstCommandVelocityAxisIndex =
                    publishedAxisIndex;
                snapshot.maxAxisCommandVelocityAbsPps =
                    commandVelocityAbsPps;
            }

            if (snapshot.worstActualVelocityAxisIndex < 0 ||
                actualVelocityAbsPps >
                snapshot.maxAxisActualVelocityAbsPps)
            {
                snapshot.worstActualVelocityAxisIndex =
                    publishedAxisIndex;
                snapshot.maxAxisActualVelocityAbsPps =
                    actualVelocityAbsPps;
            }

            if (axis.state != MotionState::MotionState_IDLE)
            {
                ++snapshot.nonIdleAxisCount;
            }
            if (commandVelocityAbsPps > commandVelocityDeadbandPps)
            {
                ++snapshot.commandMovingAxisCount;
            }
            if (actualVelocityAbsPps > actualVelocityDeadbandPps)
            {
                ++snapshot.actualMovingAxisCount;
            }

            // Preserve the existing per-axis short-circuit order exactly:
            // State -> Command Velocity -> raw 250 us Actual Velocity.
            if (primaryBlocker == MotionStopSettlePrimaryBlocker::NONE)
            {
                if (axis.state != MotionState::MotionState_IDLE)
                {
                    primaryBlocker =
                        MotionStopSettlePrimaryBlocker::AXIS_NOT_IDLE;
                    primaryAxisIndex = publishedAxisIndex;
                    primaryAxisState = axis.state;
                }
                else if (commandVelocityAbsPps >
                    commandVelocityDeadbandPps)
                {
                    primaryBlocker =
                        MotionStopSettlePrimaryBlocker::AXIS_COMMAND_VELOCITY;
                    primaryAxisIndex = publishedAxisIndex;
                    primaryAxisState = axis.state;
                }
                else if (actualVelocityAbsPps >
                    actualVelocityDeadbandPps)
                {
                    primaryBlocker =
                        MotionStopSettlePrimaryBlocker::AXIS_ACTUAL_VELOCITY;
                    primaryAxisIndex = publishedAxisIndex;
                    primaryAxisState = axis.state;
                }
            }

            const double followingErrorAbsPulse =
                std::abs(axis.currentCmdPos - axis.currentActPos);
            const double inPositionWindowAbsPulse =
                std::abs(axis.inPositionWindow_Pulse);
            const double unitsPerPulse =
                (axis.resolution_PPR != 0.0)
                ? std::abs(axis.finalLead / axis.resolution_PPR)
                : 0.0;
            const double followingErrorAbsMm =
                followingErrorAbsPulse * unitsPerPulse;
            const double inPositionWindowAbsMm =
                inPositionWindowAbsPulse * unitsPerPulse;

            double followingErrorWindowRatio = 0.0;
            if (inPositionWindowAbsPulse > 0.0)
            {
                followingErrorWindowRatio =
                    followingErrorAbsPulse / inPositionWindowAbsPulse;
            }
            else if (followingErrorAbsPulse > 0.0)
            {
                followingErrorWindowRatio =
                    (std::numeric_limits<double>::max)();
            }

            if (followingErrorAbsPulse > inPositionWindowAbsPulse)
            {
                ++snapshot.outsideInPositionWindowAxisCount;
            }

            if (snapshot.worstFollowingErrorAxisIndex < 0 ||
                followingErrorWindowRatio >
                snapshot.worstFollowingErrorWindowRatio)
            {
                snapshot.worstFollowingErrorAxisIndex = publishedAxisIndex;
                snapshot.worstFollowingErrorAbsMm = followingErrorAbsMm;
                snapshot.worstFollowingErrorWindowMm =
                    inPositionWindowAbsMm;
                snapshot.worstFollowingErrorWindowRatio =
                    followingErrorWindowRatio;
            }

            bool axisBelongsToGroup = false;
            for (int groupAxis = 0;
                groupAxis < groupAxisLimit;
                ++groupAxis)
            {
                if (m_Group.axisIndices[groupAxis] == static_cast<int>(i))
                {
                    axisBelongsToGroup = true;
                    break;
                }
            }

            if (axisBelongsToGroup)
            {
                if (followingErrorAbsPulse > inPositionWindowAbsPulse)
                {
                    ++snapshot.groupOutsideInPositionWindowAxisCount;
                }

                if (snapshot.worstGroupFollowingErrorAxisIndex < 0 ||
                    followingErrorWindowRatio >
                    snapshot.worstGroupFollowingErrorWindowRatio)
                {
                    snapshot.worstGroupFollowingErrorAxisIndex =
                        publishedAxisIndex;
                    snapshot.worstGroupFollowingErrorAbsMm =
                        followingErrorAbsMm;
                    snapshot.worstGroupFollowingErrorWindowMm =
                        inPositionWindowAbsMm;
                    snapshot.worstGroupFollowingErrorWindowRatio =
                        followingErrorWindowRatio;
                }
            }

            if (snapshot.maxStopDecTimeAxisIndex < 0 ||
                axis.Stop_dec_time > snapshot.maxConfiguredStopDecTimeSec)
            {
                snapshot.maxStopDecTimeAxisIndex = publishedAxisIndex;
                snapshot.maxConfiguredStopDecTimeSec = axis.Stop_dec_time;
            }

            if (m_pDrives != nullptr &&
                i < m_pDrives->size() &&
                (*m_pDrives)[i].pOutput != nullptr)
            {
                const std::int32_t targetVelocity =
                    (*m_pDrives)[i].pOutput->TargetVelocity;
                const std::int64_t targetVelocityWide =
                    static_cast<std::int64_t>(targetVelocity);
                const std::uint32_t targetVelocityAbs =
                    static_cast<std::uint32_t>(
                        (targetVelocityWide < 0)
                        ? -targetVelocityWide
                        : targetVelocityWide);

                ++snapshot.pdoTargetVelocitySampledAxisCount;
                if (axisBit != 0U)
                {
                    emergency.pdoTargetVelocitySampledAxisMask |=
                        axisBit;
                    if (targetVelocity == 0)
                    {
                        emergency.pdoTargetVelocityZeroAxisMask |=
                            axisBit;
                    }
                }
                if (targetVelocity != 0)
                {
                    ++snapshot.pdoTargetVelocityNonzeroAxisCount;
                }

                if (snapshot.worstFinalPdoTargetVelocityAxisIndex < 0 ||
                    targetVelocityAbs >
                    snapshot.maxFinalPdoTargetVelocityAbs)
                {
                    snapshot.worstFinalPdoTargetVelocityAxisIndex =
                        publishedAxisIndex;
                    snapshot.worstFinalPdoTargetVelocity = targetVelocity;
                    snapshot.maxFinalPdoTargetVelocityAbs = targetVelocityAbs;
                }
            }
        }
    }

    const std::uint32_t emergencySafeStateAxisMask =
        emergency.estopAxisMask |
        emergency.errorAxisMask;
    emergency.allExistingAxesSafe =
        (emergencySafeStateAxisMask &
            emergency.existingAxisMask) ==
        emergency.existingAxisMask;
    emergency.allExistingAxisCommandsZero =
        (emergency.commandZeroAxisMask &
            emergency.existingAxisMask) ==
        emergency.existingAxisMask;
    emergency.allExistingAxisTargetsSealed =
        (emergency.targetSealedAxisMask &
            emergency.existingAxisMask) ==
        emergency.existingAxisMask;
    emergency.allSampledPdoTargetVelocitiesZero =
        (emergency.pdoTargetVelocityZeroAxisMask &
            emergency.pdoTargetVelocitySampledAxisMask) ==
        emergency.pdoTargetVelocitySampledAxisMask;
    emergency.safetyLeaseMatchesLastApply =
        emergency.currentOwner == MotionOwner::SAFETY &&
        emergency.currentOwner == emergency.lastAppliedOwner &&
        emergency.currentOwnerGeneration !=
        MOTION_OWNER_GENERATION_INVALID &&
        emergency.currentOwnerGeneration ==
        emergency.lastAppliedOwnerGeneration;
    emergency.rtStopStateApplied =
        emergencyCounters.rtApplications != 0ULL &&
        !emergency.groupActive &&
        (emergency.groupEmergencyStopped || emergency.groupError) &&
        emergency.virtualCommandZero &&
        emergency.virtualTargetSealed &&
        emergency.allExistingAxesSafe &&
        emergency.allExistingAxisCommandsZero &&
        emergency.allExistingAxisTargetsSealed &&
        emergency.safetyLeaseMatchesLastApply;

    snapshot.primaryBlocker = primaryBlocker;
    snapshot.primaryAxisIndex = primaryAxisIndex;
    snapshot.primaryAxisState = primaryAxisState;
    snapshot.standstill =
        (primaryBlocker == MotionStopSettlePrimaryBlocker::NONE);

    MotionStopSettleCounters& counters = m_stopSettleProducerCounters;
    ++counters.sampleCount;
    if (snapshot.standstill)
    {
        ++counters.standstillSampleCount;
    }
    else
    {
        ++counters.blockedSampleCount;
    }

    switch (primaryBlocker)
    {
    case MotionStopSettlePrimaryBlocker::GROUP_ACTIVE:
        ++counters.groupActiveBlockerCount;
        break;
    case MotionStopSettlePrimaryBlocker::COMMAND_QUEUE:
        ++counters.commandQueueBlockerCount;
        break;
    case MotionStopSettlePrimaryBlocker::AXIS_NOT_IDLE:
        ++counters.axisNotIdleBlockerCount;
        break;
    case MotionStopSettlePrimaryBlocker::AXIS_COMMAND_VELOCITY:
        ++counters.axisCommandVelocityBlockerCount;
        break;
    case MotionStopSettlePrimaryBlocker::AXIS_ACTUAL_VELOCITY:
        ++counters.axisActualVelocityBlockerCount;
        break;
    case MotionStopSettlePrimaryBlocker::NONE:
        ++counters.noneBlockerCount;
        break;
    default:
        break;
    }

    if (m_stopSettleHasPreviousSample)
    {
        if (primaryBlocker != m_stopSettlePreviousPrimaryBlocker)
        {
            ++counters.primaryBlockerTransitionCount;
        }
        if (snapshot.standstill != m_stopSettlePreviousStandstill)
        {
            ++counters.standstillTransitionCount;
            if (snapshot.standstill)
            {
                ++counters.transitionIntoStandstillCount;
            }
            else
            {
                ++counters.transitionOutOfStandstillCount;
            }
        }
    }

    m_stopSettlePreviousPrimaryBlocker = primaryBlocker;
    m_stopSettlePreviousStandstill = snapshot.standstill;
    m_stopSettleHasPreviousSample = true;

    payload.counters = counters;
    snapshot.sampleSequence = counters.sampleCount;

    // Formal J.5 profiles and Reset ACK share this exact coherent bank with
    // the legacy J.4 diagnostics.
    UpdateNCSettleProducer(payload);

    ++m_stopSettleNextPublicationGeneration;
    snapshot.publicationGeneration =
        m_stopSettleNextPublicationGeneration;
    emergency.publicationGeneration =
        snapshot.publicationGeneration;
    for (std::size_t profileIndex = 0U;
        profileIndex < MOTION_NC_SETTLE_PROFILE_COUNT;
        ++profileIndex)
    {
        payload.ncSettleSnapshots[profileIndex].publicationGeneration =
            snapshot.publicationGeneration;
        m_ncSettlePublishedSnapshots[profileIndex].publicationGeneration =
            snapshot.publicationGeneration;
    }

    std::array<
        std::uint64_t,
        MOTION_STOP_SETTLE_PUBLICATION_WORD_COUNT> words{};
    std::memcpy(words.data(), &payload, sizeof(payload));

    MotionStopSettleAtomicBank& publicationBank =
        m_stopSettlePublicationBanks[
            static_cast<std::size_t>(
                snapshot.publicationGeneration & 1ULL)];

    publicationBank.writeSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);

    for (std::size_t i = 0U; i < words.size(); ++i)
    {
        publicationBank.words[i].store(
            words[i],
            std::memory_order_relaxed);
    }

    // Complete this bank before the single active-Generation commit.  A
    // reader overtaken by bank reuse or by a new Motion pass retries.
    std::atomic_thread_fence(std::memory_order_release);
    publicationBank.writeSequence.fetch_add(
        1ULL,
        std::memory_order_release);
    m_stopSettlePublicationGeneration.store(
        snapshot.publicationGeneration,
        std::memory_order_release);
}


bool MotionCore::TryReadStopSettlePublication(
    MotionStopSettlePublicationPayload& payload) const noexcept
{
    std::array<
        std::uint64_t,
        MOTION_STOP_SETTLE_PUBLICATION_WORD_COUNT> words{};

    // Gate consumers are never allowed to spin without a bound in P50/HMI.
    // A busy bank fails closed and the caller may try again next scan.
    for (std::uint32_t attempt = 0U; attempt < 16U; ++attempt)
    {
        const std::uint64_t generationBefore =
            m_stopSettlePublicationGeneration.load(
                std::memory_order_acquire);
        const MotionStopSettleAtomicBank& publicationBank =
            m_stopSettlePublicationBanks[
                static_cast<std::size_t>(generationBefore & 1ULL)];

        const std::uint64_t bankSequenceBefore =
            publicationBank.writeSequence.load(
                std::memory_order_acquire);
        if ((bankSequenceBefore & 1ULL) != 0ULL)
        {
            continue;
        }

        for (std::size_t i = 0U; i < words.size(); ++i)
        {
            words[i] = publicationBank.words[i].load(
                std::memory_order_relaxed);
        }

        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint64_t bankSequenceAfter =
            publicationBank.writeSequence.load(
                std::memory_order_acquire);
        const std::uint64_t generationAfter =
            m_stopSettlePublicationGeneration.load(
                std::memory_order_acquire);
        if (bankSequenceBefore == bankSequenceAfter &&
            (bankSequenceAfter & 1ULL) == 0ULL &&
            generationBefore == generationAfter)
        {
            MotionStopSettlePublicationPayload candidate{};
            std::memcpy(&candidate, words.data(), sizeof(candidate));

            // Also reject a stale active-generation read that happened to
            // select a bank already rewritten two publications later.
            if (candidate.snapshot.publicationGeneration ==
                generationBefore)
            {
                payload = candidate;
                return true;
            }
        }
    }

    payload = MotionStopSettlePublicationPayload{};
    return false;
}


MotionStopSettleSnapshot MotionCore::GetStopSettleSnapshot() const noexcept
{
    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        return MotionStopSettleSnapshot{};
    }
    return payload.snapshot;
}


MotionStopSettleCounters MotionCore::GetStopSettleCounters() const noexcept
{
    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        return MotionStopSettleCounters{};
    }
    return payload.counters;
}


void MotionCore::GetStopSettleEvidence(
    MotionStopSettleSnapshot& snapshot,
    MotionStopSettleCounters& counters) const noexcept
{
    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        snapshot = MotionStopSettleSnapshot{};
        counters = MotionStopSettleCounters{};
        return;
    }

    snapshot = payload.snapshot;
    counters = payload.counters;
}


bool MotionCore::TryGetEmergencyStopEvidence(
    MotionEmergencyStopEvidence& evidence,
    MotionEmergencyStopCounters& counters) const noexcept
{
    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        evidence = MotionEmergencyStopEvidence{};
        counters = MotionEmergencyStopCounters{};
        return false;
    }

    evidence = payload.emergencyStopEvidence;
    counters = payload.emergencyStopCounters;
    return true;
}


MotionEmergencyStopEpochInvalidationEvidence
MotionCore::GetEmergencyStopEpochInvalidationEvidence() const noexcept
{
    // A bounded seqlock read keeps the packed Epoch pair and its monotonic
    // invalidation count coherent without adding bytes to the stop/settle
    // publication.  E-stop invalidations are rare; four retries are ample and
    // an unstable read deliberately returns INVALID evidence.
    for (unsigned int attempt = 0U; attempt < 4U; ++attempt)
    {
        const std::uint64_t sequenceBefore =
            m_emergencyStopEpochInvalidationWriteSequence.load(
                std::memory_order_acquire);
        if ((sequenceBefore & 1ULL) != 0ULL)
        {
            continue;
        }

        const std::uint64_t packed =
            m_emergencyStopLastEpochInvalidationPacked.load(
                std::memory_order_acquire);
        const std::uint64_t invalidationCount =
            m_emergencyStopLastEpochInvalidationCount.load(
                std::memory_order_acquire);
        const std::uint64_t sequenceAfter =
            m_emergencyStopEpochInvalidationWriteSequence.load(
                std::memory_order_acquire);

        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1ULL) == 0ULL)
        {
            MotionEmergencyStopEpochInvalidationEvidence evidence{};
            evidence.fromExecutionEpoch =
                static_cast<MotionExecutionEpoch>(
                    packed & 0xFFFFFFFFULL);
            evidence.toExecutionEpoch =
                static_cast<MotionExecutionEpoch>(
                    (packed >> 32U) & 0xFFFFFFFFULL);
            evidence.invalidationCount = invalidationCount;
            return evidence;
        }
    }

    return MotionEmergencyStopEpochInvalidationEvidence{};
}


MotionStartupLagArmingEvidence
MotionCore::GetStartupLagArmingEvidence() const noexcept
{
    MotionStartupLagArmingEvidence evidence{};

    const std::uint64_t packedMasks =
        m_startupLagArmingMasks.load(std::memory_order_acquire);
    evidence.existingAxisMask = static_cast<std::uint32_t>(
        packedMasks & 0xFFULL);
    evidence.feedbackReadyAxisMask = static_cast<std::uint32_t>(
        (packedMasks >> 8U) & 0xFFULL);
    evidence.positionAlignedAxisMask = static_cast<std::uint32_t>(
        (packedMasks >> 16U) & 0xFFULL);
    evidence.lagArmedAxisMask = static_cast<std::uint32_t>(
        (packedMasks >> 24U) & 0xFFULL);
    evidence.prematureMotionBlockedAxisMask =
        static_cast<std::uint32_t>(
            (packedMasks >> 32U) & 0xFFULL);
    evidence.pendingAxisMask =
        evidence.existingAxisMask & ~evidence.lagArmedAxisMask;

    evidence.minimumStableSampleCount =
        m_startupLagMinimumStableSampleCount.load(
            std::memory_order_acquire);
    evidence.stableSamplesRequired =
        MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES;
    evidence.alignmentEvents =
        m_startupLagAlignmentEvents.load(std::memory_order_acquire);
    evidence.armingTransitions =
        m_startupLagArmingTransitions.load(std::memory_order_acquire);
    evidence.readinessResets =
        m_startupLagReadinessResets.load(std::memory_order_acquire);
    evidence.prematureMotionBlocks =
        m_startupLagPrematureMotionBlocks.load(std::memory_order_acquire);

    evidence.allExistingAxesArmed =
        evidence.existingAxisMask != 0U &&
        evidence.pendingAxisMask == 0U;
    evidence.blocked =
        evidence.prematureMotionBlockedAxisMask != 0U;
    return evidence;
}


MotionP1HandoverSafetySnapshot
MotionCore::GetP1HandoverSafetySnapshot() const noexcept
{
    MotionP1HandoverSafetySnapshot snapshot{};
    const std::uint64_t alarmRequestPublication =
        m_p1MappingIntegrityAlarmRequestPublication.load(
            std::memory_order_acquire);
    snapshot.mappingBoundaryStops =
        m_p1MappingBoundaryStopCount.load(std::memory_order_acquire);
    snapshot.droppedAxisRetirements =
        m_p1DroppedAxisRetirementCount.load(std::memory_order_acquire);
    snapshot.droppedAxisRetirementFailures =
        m_p1DroppedAxisRetirementFailureCount.load(
            std::memory_order_acquire);
    snapshot.orphanAxisContainments =
        m_p1OrphanAxisContainmentCount.load(std::memory_order_acquire);
    snapshot.invalidProducerRejects =
        m_p1InvalidProducerRejectCount.load(std::memory_order_acquire);
    snapshot.mappingIntegrityAlarmRequests =
        (alarmRequestPublication & P1_MAPPING_ALARM_SEQUENCE_MASK) >>
        P1_MAPPING_ALARM_SEQUENCE_SHIFT;
    snapshot.mappingIntegrityAlarmPending =
        (alarmRequestPublication & P1_MAPPING_ALARM_PENDING) != 0ULL;
    snapshot.lastPreviousAxisMask =
        m_p1LastPreviousAxisMask.load(std::memory_order_acquire);
    snapshot.lastNextAxisMask =
        m_p1LastNextAxisMask.load(std::memory_order_acquire);
    snapshot.lastMappingIntegrityAlarmExecutionEpoch =
        static_cast<MotionExecutionEpoch>(
            alarmRequestPublication & P1_MAPPING_ALARM_EPOCH_MASK);
    snapshot.lastOrphanAxisIndex =
        m_p1LastOrphanAxisIndex.load(std::memory_order_acquire);
    return snapshot;
}


MotionLifecycleCommitReservationSnapshot
MotionCore::GetLifecycleCommitReservationSnapshot() const noexcept
{
    MotionLifecycleCommitReservationSnapshot snapshot{};
    snapshot.attempts =
        m_lifecycleCommitReservationAttemptCount.load(
            std::memory_order_acquire);
    snapshot.acquired =
        m_lifecycleCommitReservationAcquiredCount.load(
            std::memory_order_acquire);
    snapshot.blockedByLifecycle =
        m_lifecycleCommitReservationBlockedCount.load(
            std::memory_order_acquire);
    snapshot.compareExchangeLost =
        m_lifecycleCommitReservationCASLostCount.load(
            std::memory_order_acquire);
    snapshot.released =
        m_lifecycleCommitReservationReleasedCount.load(
            std::memory_order_acquire);
    snapshot.releaseFailures =
        m_lifecycleCommitReservationReleaseFailureCount.load(
            std::memory_order_acquire);
    snapshot.publisherWaits =
        m_lifecycleCommitReservationPublisherWaitCount.load(
            std::memory_order_acquire);

    const std::uint64_t publication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    snapshot.currentExecutionEpoch =
        UnpackExecutionEpochPublication(publication);
    snapshot.reservationActive =
        (publication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL;
    snapshot.executionEpochPending =
        (publication & EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL;
    return snapshot;
}


bool MotionCore::AcknowledgeP1MappingIntegrityAlarmRequest(
    std::uint64_t requestSequence) noexcept
{
    if (requestSequence == 0ULL ||
        requestSequence > P1_MAPPING_ALARM_SEQUENCE_MAX)
    {
        return false;
    }

    std::uint64_t observed =
        m_p1MappingIntegrityAlarmRequestPublication.load(
            std::memory_order_acquire);
    const std::uint64_t observedSequence =
        (observed & P1_MAPPING_ALARM_SEQUENCE_MASK) >>
        P1_MAPPING_ALARM_SEQUENCE_SHIFT;
    if ((observed & P1_MAPPING_ALARM_PENDING) == 0ULL ||
        observedSequence != requestSequence)
    {
        return false;
    }

    const std::uint64_t acknowledged =
        observed & ~P1_MAPPING_ALARM_PENDING;
    return m_p1MappingIntegrityAlarmRequestPublication.compare_exchange_strong(
        observed,
        acknowledged,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}


bool MotionCore::TryGetNCSettleEvidence(
    MotionNCSettleProfile profile,
    MotionNCSettleSnapshot& snapshot,
    MotionNCSettleCounters& counters) const noexcept
{
    const std::size_t profileIndex = static_cast<std::size_t>(profile);
    if (profileIndex >= MOTION_NC_SETTLE_PROFILE_COUNT)
    {
        snapshot = MotionNCSettleSnapshot{};
        counters = MotionNCSettleCounters{};
        return false;
    }

    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        snapshot = MotionNCSettleSnapshot{};
        snapshot.profile = profile;
        snapshot.blocker = MotionNCSettleBlocker::PUBLICATION_BUSY;
        counters = MotionNCSettleCounters{};
        return false;
    }

    snapshot = payload.ncSettleSnapshots[profileIndex];
    counters = payload.ncSettleCounters[profileIndex];
    return true;
}


bool MotionCore::IsGroupNCSettled() const noexcept
{
    MotionNCSettleSnapshot snapshot{};
    MotionNCSettleCounters counters{};
    return
        TryGetNCSettleEvidence(
            MotionNCSettleProfile::GROUP_COMPLETION,
            snapshot,
            counters) &&
        snapshot.runtimeCycleValid &&
        snapshot.runtimeCycleContiguous &&
        snapshot.settled;
}


bool MotionCore::IsGroupNCDrained() const noexcept
{
    MotionNCSettleSnapshot snapshot{};
    MotionNCSettleCounters counters{};
    return
        TryGetNCSettleEvidence(
            MotionNCSettleProfile::GROUP_COMPLETION,
            snapshot,
            counters) &&
        snapshot.runtimeCycleValid &&
        snapshot.groupDrained;
}


MotionNCResetRebaseAck MotionCore::GetNCResetRebaseAck() const noexcept
{
    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        return MotionNCResetRebaseAck{};
    }
    return payload.resetRebaseAck;
}


MotionFeedHoldStopSnapshot MotionCore::GetFeedHoldStopSnapshot() const noexcept
{
    MotionFeedHoldStopSnapshot snapshot{};

    // Fail closed before the bounded coherent-bank read.
    snapshot.virtualCommandStopped = false;
    snapshot.axisCommandStopped = false;
    snapshot.axisActualStopped = false;
    snapshot.commandStopped = false;
    snapshot.actualStopped = false;
    snapshot.motionStopped = false;

    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        return snapshot;
    }

    const MotionNCSettleSnapshot& formal =
        payload.ncSettleSnapshots[static_cast<std::size_t>(
            MotionNCSettleProfile::FEED_HOLD_GROUP)];
    const MotionStopSettleSnapshot& advisory = payload.snapshot;

    snapshot.feedrateOverride = formal.feedrateOverride;
    snapshot.virtualCommandVelocityPps =
        formal.virtualCommandVelocityAbsPps;
    snapshot.maxAxisCommandVelocityPps =
        formal.maxCommandVelocityAbsPps;
    snapshot.maxAxisActualVelocityPps =
        formal.advisoryMaxActualVelocityAbsPps;
    snapshot.commandQueueDepth = formal.commandQueueDepth;
    snapshot.commandIngressDepth = formal.commandIngressDepth;
    snapshot.commandReplayDepth = formal.commandReplayDepth;
    snapshot.groupActive = formal.groupActive;
    snapshot.groupDone = formal.groupDrained;
    snapshot.groupFaulted = formal.anyAxisFaultOrEstop;
    snapshot.groupEmergencyStopped = formal.anyAxisFaultOrEstop;
    snapshot.safetyOrRecoveryPending =
        formal.safetyOrRecoveryPending;
    snapshot.overrideZero = formal.overrideZero;
    snapshot.virtualCommandStopped = formal.virtualCommandStopped;
    snapshot.axisCommandStopped = formal.allAxisCommandStopped;
    snapshot.commandStopped =
        snapshot.virtualCommandStopped && snapshot.axisCommandStopped;

    for (std::uint32_t bit = 0U; bit < MAX_AXES; ++bit)
    {
        if ((formal.scopeMask & (1U << bit)) != 0U)
        {
            ++snapshot.groupAxisCount;
        }
    }
    snapshot.commandMovingAxes =
        snapshot.axisCommandStopped ? 0U : 1U;
    snapshot.actualMovingAxes = advisory.actualMovingAxisCount;
    snapshot.faultedAxes = formal.anyAxisFaultOrEstop ? 1U : 0U;

    snapshot.settleRequestSequence = formal.requestSequence;
    snapshot.settleProofSequence = formal.proofSequence;
    snapshot.settleScopeMask = formal.scopeMask;
    snapshot.settleDwellCycles = formal.dwellCycles;
    snapshot.settleRequiredCycles = formal.requiredCycles;
    snapshot.settleRequestAccepted = formal.requestAccepted;
    snapshot.settleProofValid =
        formal.runtimeObserved &&
        formal.runtimeCycleValid &&
        formal.runtimeCycleContiguous;
    snapshot.ncSettled = formal.settled;

    // The old ActualStopped name now deliberately maps to the continuous
    // formal proof.  Raw encoder velocity remains available above only for
    // diagnostics and cannot hold Feed Hold for seconds.
    snapshot.axisActualStopped = formal.settled;
    snapshot.actualStopped = formal.settled;
    snapshot.motionStopped =
        formal.settled &&
        formal.requestAccepted &&
        !formal.anyAxisFaultOrEstop &&
        !formal.safetyOrRecoveryPending;

    return snapshot;
}

// 🌟 檢查全系統所有存在的實體軸是否有警報
bool MotionCore::IsAnyAxisFaulted() const
{
    // 防呆：指標尚未初始化則回傳無錯誤
    if (m_pContexts == nullptr) {
        return false;
    }

    // 掃描所有軸
    for (size_t i = 0; i < m_pContexts->size(); i++)
    {
        const AxisContext& axis = (*m_pContexts)[i];

        // 條件：軸實體存在，且狀態為 isFault (硬體或軟體跳機)
        if (axis.isExist && axis.isFault)
        {
            return true; // 只要有一軸異常，就回傳 true
        }
    }

    return false;
}

// 🌟 檢查當前插補群組內參與的軸是否有警報
bool MotionCore::IsGroupFaulted() const
{
    if (m_pContexts == nullptr || !m_Group.isActive)
    {
        return false;
    }

    const std::size_t contextCount =
        (std::min)(
            m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES));

    const int commandAxisCount =
        (std::max)(
            0,
            (std::min)(
                m_Group.currentCmd.axisCount,
                MAX_AXES));

    for (int j = 0; j < commandAxisCount; ++j)
    {
        const int axisIndex =
            m_Group.currentCmd.axisIndices[j];

        if (axisIndex < 0 ||
            static_cast<std::size_t>(axisIndex) >= contextCount)
        {
            continue;
        }

        const AxisContext& axis =
            (*m_pContexts)[static_cast<std::size_t>(axisIndex)];

        if (axis.isExist &&
            (axis.isFault ||
                axis.state == MotionState::MotionState_ERROR ||
                axis.state == MotionState::MotionState_ESTOP))
        {
            return true;
        }
    }

    return false;
}

bool MotionCore::IsGroupEmergencyStopped() const
{
    if (m_Group.virtualAxis.state ==
        MotionState::MotionState_ESTOP)
    {
        return true;
    }

    if (m_pContexts == nullptr)
    {
        return false;
    }

    const std::size_t contextCount =
        (std::min)(
            m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES));

    const int groupAxisCount =
        (std::max)(
            0,
            (std::min)(
                m_Group.axisCount,
                MAX_AXES));

    for (int i = 0; i < groupAxisCount; ++i)
    {
        const int axisIndex =
            m_Group.axisIndices[i];

        if (axisIndex < 0 ||
            static_cast<std::size_t>(axisIndex) >= contextCount)
        {
            continue;
        }

        const AxisContext& axis =
            (*m_pContexts)[static_cast<std::size_t>(axisIndex)];

        if (axis.state == MotionState::MotionState_ESTOP)
        {
            return true;
        }
    }

    return false;
}

// ==========================================
// [樣板實例化] (Explicit Instantiation)
// ==========================================
// 這是為了讓編譯器知道要為你的 ENI_ServoDrive 產生代碼
// 請確保這裡 include 了你的驅動器定義檔，例如: 
// #include "EtherCatDefinitions.h" 

// 假設你的結構叫 ENI_ServoDrive (請根據你的專案修改)

template void MotionCore::UpdateMotion<ENI_ServoDrive>(
    ENI_ServoDrive&,
    AxisContext&,
    const MotionServoInputSnapshot&);
template void MotionCore::Run_Servo_Loop<ENI_ServoDrive>(ENI_ServoDrive&, AxisContext&, const AxisCommand&);
