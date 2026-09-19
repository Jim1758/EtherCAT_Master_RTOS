#include "MotionCore.h"

#include "AlarmManager.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{
    std::uint64_t FoldQueueTailFingerprint(
        std::uint64_t fingerprint,
        std::uint64_t value) noexcept
    {
        return (fingerprint ^ value) * 1099511628211ULL;
    }

    std::uint64_t QueueTailDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        static_assert(sizeof(bits) == sizeof(value),
            "Queue-tail fingerprint requires an IEEE-754-sized double.");
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }

    std::uint64_t BuildQueueTailFingerprint(
        const std::array<double, MAX_AXES>& producerQueueTailPulse,
        const double* commandedMCS,
        double rapidOverride) noexcept
    {
        std::uint64_t fingerprint = MOTION_QUEUE_TAIL_FINGERPRINT_SEED;
        for (int axisIndex = 0; axisIndex < MAX_AXES; ++axisIndex)
        {
            const double mcs = commandedMCS != nullptr
                ? commandedMCS[axisIndex]
                : 0.0;
            const double pulse =
                producerQueueTailPulse[
                    static_cast<std::size_t>(axisIndex)];
            fingerprint = FoldQueueTailFingerprint(
                fingerprint,
                QueueTailDoubleBits(mcs));
            fingerprint = FoldQueueTailFingerprint(
                fingerprint,
                QueueTailDoubleBits(pulse));
        }
        return FoldQueueTailFingerprint(
            fingerprint,
            QueueTailDoubleBits(rapidOverride));
    }
} // namespace


void MotionCore::G00_Move(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode)
{
    // Backward-compatible API: preserve the accepted G00 mapping for any
    // out-of-tree caller that has not yet supplied the independent K.6 field.
    const MotionCommandPathMode commandPathMode =
        mode == BufferMode::BUFFERED
        ? MotionCommandPathMode::CONTINUOUS
        : MotionCommandPathMode::EXACT_STOP;

    G00_Move(
        axes,
        targetPos_mm,
        mode,
        commandPathMode);
}


void MotionCore::G00_Move(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode,
    MotionCommandPathMode commandPathMode)
{
    // Compatibility callers do not own CoordinateManager::commandedMCS, but
    // their Motion pulse tail still follows the same success-only rule.
    (void)TryG00MoveInternal(
        axes,
        targetPos_mm,
        mode,
        commandPathMode,
        G00_overrideRatio,
        nullptr,
        false);
}


bool MotionCore::TryG00MoveTransactionalTail(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode,
    MotionCommandPathMode commandPathMode,
    double rapidOverrideCandidate,
    double(&commandedMCSTail)[MAX_AXES])
{
    return TryG00MoveInternal(
        axes,
        targetPos_mm,
        mode,
        commandPathMode,
        rapidOverrideCandidate,
        commandedMCSTail,
        true);
}


// Sparse/incremental endpoints and R-arc centers depend on the accepted native
// tail, including unrotated G90 R arcs. Prove that reference against the SAME
// start sample used by this producer before enqueue; callers own feature scope.
// Both exact arithmetic representations are valid: accepted forward conversion,
// or the division-built baseline produced by START/RESET. No tolerance hides drift.
bool MotionCore::IsPlanarEndpointBasisCurrent(const double* referenceMCS,
    const std::array<double, 8U>& startPulse, std::uint32_t validAxisMask,
    std::uint32_t requiredAxisMask) const noexcept
{
    if (referenceMCS == nullptr || m_pContexts == nullptr || requiredAxisMask == 0U ||
        (requiredAxisMask & ~7U) != 0U || (validAxisMask & requiredAxisMask) != requiredAxisMask ||
        !IsNCTranslationSnapshotValid(m_pendingTranslation) ||
        !IsPendingFixedTranslationSourceAllowed()) return false;
    for (std::size_t slot = 0U; slot < 3U; ++slot)
    {
        if ((requiredAxisMask & (1U << static_cast<unsigned>(slot))) == 0U) continue;
        if (slot >= m_pContexts->size()) return false;
        const AxisContext& axis = (*m_pContexts)[slot];
        if (!axis.isExist || axis.axisType != AxisType::LINEAR ||
            !std::isfinite(referenceMCS[slot]) || !std::isfinite(startPulse[slot]) ||
            !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0) return false;
        const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
        if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0) return false;
        const double forwardPulse = referenceMCS[slot] * pulsePerMM;
        const double reverseMCS = startPulse[slot] * axis.finalLead / axis.resolution_PPR;
        if (!(std::isfinite(forwardPulse) && forwardPulse == startPulse[slot]) &&
            !(std::isfinite(reverseMCS) && reverseMCS == referenceMCS[slot]))
        {
            // Producer-thread diagnostic only; preserve the exact rejection.
            RtPrintf("[ROTATION][BASIS_REJECT] axis=%u referenceMMBits=%llu sampledPulseBits=%llu expectedPulseBits=%llu\n",
                static_cast<unsigned>(slot),
                static_cast<unsigned long long>(QueueTailDoubleBits(referenceMCS[slot])),
                static_cast<unsigned long long>(QueueTailDoubleBits(startPulse[slot])),
                static_cast<unsigned long long>(QueueTailDoubleBits(forwardPulse)));
            return false;
        }
    }
    return true;
}


// BQ: the caller owns the output workspace; no Motion capture ABI changes.
bool MotionCore::TryG00MoveTransactionalTail(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode,
    MotionCommandPathMode commandPathMode,
    double rapidOverrideCandidate,
    double(&commandedMCSTail)[MAX_AXES],
    MotionCommandedEndpointReceiptV1* commandedEndpointReceipt,
    bool requirePlanarBaselineMatch)
{
    return TryG00MoveInternal(
        axes,
        targetPos_mm,
        mode,
        commandPathMode,
        rapidOverrideCandidate,
        commandedMCSTail,
        true,
        commandedEndpointReceipt,
        requirePlanarBaselineMatch);
}


bool MotionCore::TryG00MoveInternal(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode,
    MotionCommandPathMode commandPathMode,
    double rapidOverrideCandidate,
    double* commandedMCSTail,
    bool transactionalTail,
    MotionCommandedEndpointReceiptV1* commandedEndpointReceipt,
    bool requirePlanarBaselineMatch,
    bool useG53Profile)
{
    // BQ: every invocation starts unpublished, including every rejection path.
    if (commandedEndpointReceipt != nullptr)
    {
        commandedEndpointReceipt->Clear();
    }

    // Capture one Producer tuple. TryLineMove revalidates it without blocking:
    // BUFFERED keeps this exact tuple, while ABORTING may publish only its exact
    // successor.  The sidecar tag is the final commit seal after enqueue.
    const MotionExecutionEpoch plannedTailEpoch =
        GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedTailOwnerLease =
        GetMotionOwnerLease();
    const MotionCommandSource commandSource =
        m_pendingCommandSource.load(std::memory_order_acquire);

    MotionQueueTailCommitReceipt receipt{};
    if (transactionalTail)
    {
        receipt.transactionSequence =
            AllocateQueueTailTransactionSequence();
        receipt.attempted = true;
        if (commandedMCSTail != nullptr)
        {
            receipt.beforeFingerprint = BuildQueueTailFingerprint(
                m_g00ProducerQueueTailPulse,
                commandedMCSTail,
                G00_overrideRatio);
            receipt.committedFingerprint = receipt.beforeFingerprint;
        }
    }

    const auto rejectWithoutTailMutation =
        [&](bool invalidInput,
            bool publishFormalMotionReject,
            MotionRejectReason formalRejectReason) -> bool
    {
        // BQ: a rejected attempt cannot leave a preceding endpoint readable.
        if (commandedEndpointReceipt != nullptr)
        {
            commandedEndpointReceipt->Clear();
        }

        if (publishFormalMotionReject)
        {
            MotionCommand rejectedCommand{};
            rejectedCommand.mode = InterpolationMode::LINEAR;
            rejectedCommand.sourceLinePC = m_pendingSourcePC;
            rejectedCommand.commandPathMode = commandPathMode;

            if (formalRejectReason ==
                MotionRejectReason::INVALID_GEOMETRY)
            {
                RejectInvalidProducerMotionCommand(
                    rejectedCommand,
                    GetCurrentExecutionEpoch(),
                    commandSource,
                    GetMotionOwnerLease(),
                    &receipt.identity,
                    &receipt.ownerLease);
            }
            else
            {
                RejectNonGeometryProducerMotionCommand(
                    rejectedCommand,
                    GetCurrentExecutionEpoch(),
                    commandSource,
                    GetMotionOwnerLease(),
                    formalRejectReason,
                    &receipt.identity,
                    &receipt.ownerLease);
            }
        }

        if (!transactionalTail)
        {
            return false;
        }

        if (commandedMCSTail != nullptr)
        {
            receipt.committedFingerprint =
                BuildQueueTailFingerprint(
                    m_g00ProducerQueueTailPulse,
                    commandedMCSTail,
                    G00_overrideRatio);
        }
        else
        {
            receipt.committedFingerprint =
                receipt.beforeFingerprint;
        }
        receipt.commandAccepted = false;
        receipt.commandedMCSCommitted = false;
        receipt.lastQueuedPulseCommitted = false;
        receipt.rapidOverrideCommitted = false;
        receipt.preservedOnReject =
            receipt.beforeFingerprint ==
            receipt.committedFingerprint;
        receipt.endpointExact = false;
        receipt.accountingValid = receipt.preservedOnReject;
        PublishQueueTailTransactionReceipt(receipt, invalidInput);
        return false;
    };

    if (m_pContexts == nullptr ||
        (commandSource == MotionCommandSource::NC_MEMORY && m_pendingToolRadMode != 40) ||
        (transactionalTail && commandedMCSTail == nullptr))
    {
        return rejectWithoutTailMutation(
            true,
            true,
            MotionRejectReason::INVALID_GEOMETRY);
    }

    // BASE-PLANE-3: a mismatched G17 tag must not downgrade a non-XY
    // frozen source into the legacy rapid lane before source rejection.
    const bool basePlaneLinear = m_pendingPlaneMode != 17 ||
        (!IsNCTranslationSnapshotEmpty(m_pendingTranslation) && m_pendingTranslation.rotationPlane != 17);
    if (basePlaneLinear && (commandSource != MotionCommandSource::NC_MEMORY ||
        !transactionalTail || useG53Profile || !IsNCTranslationSnapshotValid(m_pendingTranslation) ||
        !IsPendingFixedTranslationSourceAllowed() || !IsNCTranslationBaseArcPlaneFrame(m_pendingTranslation) ||
        m_pendingPlaneMode != m_pendingTranslation.rotationPlane ||
        !IsNCNativeXYZLinearMapping(static_cast<int>(axes.size()), axes.data()) ||
        plannedTailOwnerLease.owner != MotionOwner::AUTO || mode != BufferMode::ABORTING ||
        commandPathMode != MotionCommandPathMode::EXACT_STOP ||
        HasPendingSafetyOrRecoveryRequests() || !IsGroupDone() ||
        GetCommandIngressSize() != 0U || GetCommandReplaySize() != 0U))
    {
        return rejectWithoutTailMutation(false, true, MotionRejectReason::NOT_READY);
    }

    // G53 owns native endpoints and its own dynamics. A frozen descriptor
    // proves source only; no transform is applied to the machine targets.
    if (useG53Profile && (mode != BufferMode::ABORTING ||
        commandPathMode != MotionCommandPathMode::EXACT_STOP ||
        (commandSource == MotionCommandSource::NC_MEMORY &&
            (!m_pendingIsAbsoluteMode || m_pendingPlaneMode != 17 ||
                (!IsNCTranslationSnapshotEmpty(m_pendingTranslation) &&
                    (!IsPendingFixedTranslationSourceAllowed() ||
                        CheckG53NativeHandoff(m_pendingTranslation, plannedTailEpoch, plannedTailOwnerLease) !=
                            MotionNCTranslationTransitionResult::ACCEPTED))))))
    {
        return rejectWithoutTailMutation(false, true, MotionRejectReason::NOT_READY);
    }

    if (mode != BufferMode::BUFFERED &&
        mode != BufferMode::ABORTING)
    {
        return rejectWithoutTailMutation(
            true,
            true,
            MotionRejectReason::INVALID_GEOMETRY);
    }

    if (axes.empty() ||
        axes.size() > static_cast<std::size_t>(MAX_AXES) ||
        axes.size() != targetPos_mm.size() ||
        !IsValidMotionCommandPathMode(commandPathMode) ||
        !std::isfinite(rapidOverrideCandidate) ||
        rapidOverrideCandidate < 0.0)
    {
        return rejectWithoutTailMutation(
            true,
            true,
            MotionRejectReason::INVALID_GEOMETRY);
    }

    if (plannedTailEpoch == MOTION_EXECUTION_EPOCH_INVALID)
    {
        return rejectWithoutTailMutation(
            false,
            true,
            MotionRejectReason::STALE_EPOCH);
    }
    if (!plannedTailOwnerLease.IsValid())
    {
        return rejectWithoutTailMutation(
            false,
            true,
            MotionRejectReason::OWNER_CONFLICT);
    }

    // A BUFFERED plan is meaningful only relative to the exact tagged
    // Producer tail.  Lifecycle tuple drift is a normal formal rejection, not
    // malformed geometry and therefore does not request an E-stop here.
    if (mode == BufferMode::BUFFERED &&
        (plannedTailEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
            m_g00ProducerQueueTailEpoch != plannedTailEpoch))
    {
        return rejectWithoutTailMutation(
            false,
            true,
            MotionRejectReason::STALE_EPOCH);
    }
    if (mode == BufferMode::BUFFERED &&
        (!plannedTailOwnerLease.IsValid() ||
            !m_g00ProducerQueueTailOwnerLease.IsValid() ||
            !m_g00ProducerQueueTailOwnerLease.Matches(
                plannedTailOwnerLease)))
    {
        return rejectWithoutTailMutation(
            false,
            true,
            MotionRejectReason::OWNER_CONFLICT);
    }

    std::array<bool, MAX_AXES> seenAxis{};
    std::array<double, MAX_AXES> stagedQueueTailPulse =
        m_g00ProducerQueueTailPulse;
    std::array<double, MAX_AXES> stagedCommandedMCS{};
    if (transactionalTail)
    {
        for (int axisIndex = 0; axisIndex < MAX_AXES; ++axisIndex)
        {
            stagedCommandedMCS[static_cast<std::size_t>(axisIndex)] =
                commandedMCSTail[axisIndex];
        }
    }
    std::uint32_t stagedValidMask =
        m_g00ProducerQueueTailValidMask;

    // ABORTING cancels every future endpoint in the old Epoch. Snapshot every
    // existing axis from the RT-owned logical position before planning, then
    // rebuild both the successor pulse tail and its MCS baseline. This keeps an
    // immediately following P1 command on an unprogrammed axis coherent.
    if (mode == BufferMode::ABORTING)
    {
        stagedQueueTailPulse.fill(0.0);
        stagedValidMask = 0U;

        const std::size_t contextCount = (std::min)(
            m_pContexts->size(),
            static_cast<std::size_t>(MAX_AXES));
        for (std::size_t axisSlot = 0U;
            axisSlot < contextCount;
            ++axisSlot)
        {
            const AxisContext& axis = (*m_pContexts)[axisSlot];
            if (!axis.isExist)
            {
                continue;
            }

            const double logicalPulse = axis.logicalCmdPos.Load();
            if (!std::isfinite(logicalPulse))
            {
                return rejectWithoutTailMutation(
                    true,
                    true,
                    MotionRejectReason::INVALID_GEOMETRY);
            }

            stagedQueueTailPulse[axisSlot] = logicalPulse;
            stagedValidMask |=
                (1U << static_cast<unsigned>(axisSlot));

            if (transactionalTail)
            {
                if (!std::isfinite(axis.finalLead) ||
                    !std::isfinite(axis.resolution_PPR) ||
                    axis.resolution_PPR <= 0.0)
                {
                    return rejectWithoutTailMutation(
                        true,
                        true,
                        MotionRejectReason::INVALID_GEOMETRY);
                }

                double baselineMCS =
                    logicalPulse * axis.finalLead /
                    axis.resolution_PPR;
                // Preserve the accepted dependent-endpoint MCS representation only when
                // it converts exactly to this sampled native pulse. Otherwise
                // retain the reconstructed baseline (and the baseline proof
                // rejects stale axes). Consecutive receipt seams stay bit-exact.
                if (commandSource == MotionCommandSource::NC_MEMORY &&
                    (NCTranslationHasPlanarRotation(m_pendingTranslation) ||
                        m_pendingTranslation.distanceMode == 91 || basePlaneLinear) &&
                    axis.axisType == AxisType::LINEAR)
                {
                    const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
                    const double reference = commandedMCSTail[axisSlot];
                    const double referencePulse = reference * pulsePerMM;
                    if (std::isfinite(pulsePerMM) && pulsePerMM > 0.0 &&
                        std::isfinite(reference) && std::isfinite(referencePulse) &&
                        referencePulse == logicalPulse) baselineMCS = reference;
                }
                if (axis.axisType == AxisType::ROTARY &&
                    std::isfinite(axis.rotaryModulo) &&
                    axis.rotaryModulo > 0.0)
                {
                    baselineMCS = std::fmod(
                        baselineMCS,
                        axis.rotaryModulo);
                    if (baselineMCS < 0.0)
                    {
                        baselineMCS += axis.rotaryModulo;
                    }
                }
                if (!std::isfinite(baselineMCS))
                {
                    return rejectWithoutTailMutation(
                        true,
                        true,
                        MotionRejectReason::INVALID_GEOMETRY);
                }
                stagedCommandedMCS[axisSlot] = baselineMCS;
            }
        }
    }

    // Every incremental endpoint depends on its sampled native start, even
    // full XY and Z-only blocks. Preserve the old XY proof for sparse G90.
    const bool incrementalEndpoint = commandSource == MotionCommandSource::NC_MEMORY &&
        m_pendingTranslation.distanceMode == 91;
    std::uint32_t requiredBaselineMask = requirePlanarBaselineMatch ? 3U : 0U;
    if (incrementalEndpoint || basePlaneLinear)
    {
        std::uint32_t selectedMask = 0U;
        for (int axis : axes)
        {
            if (axis < 0 || axis > 2 || (selectedMask & (1U << axis)) != 0U)
                return rejectWithoutTailMutation(true, true, MotionRejectReason::INVALID_GEOMETRY);
            selectedMask |= 1U << axis;
        }
        requiredBaselineMask |= selectedMask;
    }
    if ((requirePlanarBaselineMatch || incrementalEndpoint || basePlaneLinear) &&
        (!transactionalTail || mode != BufferMode::ABORTING ||
            commandPathMode != MotionCommandPathMode::EXACT_STOP ||
            !IsPlanarEndpointBasisCurrent(commandedMCSTail, stagedQueueTailPulse,
                stagedValidMask, requiredBaselineMask)))
    {
        return rejectWithoutTailMutation(false, true, MotionRejectReason::NOT_READY);
    }

    // BQ: retain the exact staged baseline before programmed targets replace
    // it. ABORTING has already sampled/rebuilt its successor native MCS here.
    // The mask describes existing baseline axes, separate from programmed axes.
    if (transactionalTail && commandedEndpointReceipt != nullptr)
    {
        commandedEndpointReceipt->startMCS = stagedCommandedMCS;
        commandedEndpointReceipt->validAxisMask = stagedValidMask;
        commandedEndpointReceipt->origin = mode == BufferMode::ABORTING
            ? MotionCommandedBaselineOrigin::ABORTING_BASELINE
            : MotionCommandedBaselineOrigin::BUFFERED_TAIL;
    }

    std::vector<double> targetPosPulse(axes.size(), 0.0);
    std::uint32_t stagedAxisMask = 0U;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0;
    double sumSquaredPulse = 0.0;
    double sumSquaredUnit = 0.0;

    for (std::size_t slot = 0U; slot < axes.size(); ++slot)
    {
        const int axisIndex = axes[slot];
        if (axisIndex < 0 ||
            axisIndex >= MAX_AXES ||
            axisIndex >= static_cast<int>(m_pContexts->size()) ||
            seenAxis[static_cast<std::size_t>(axisIndex)] ||
            !std::isfinite(targetPos_mm[slot]))
        {
            return rejectWithoutTailMutation(
                true,
                true,
                MotionRejectReason::INVALID_GEOMETRY);
        }

        const AxisContext& axis =
            (*m_pContexts)[static_cast<std::size_t>(axisIndex)];
        if (!axis.isExist)
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::axis_is_not_enabledr);
            return rejectWithoutTailMutation(
                true,
                true,
                MotionRejectReason::INVALID_GEOMETRY);
        }

        if (useG53Profile)
        {
            if (axis.axisIndex != axisIndex ||
                (axis.axisType != AxisType::LINEAR && axis.axisType != AxisType::ROTARY &&
                    axis.axisType != AxisType::ROTARY_CONTINUOUS))
                return rejectWithoutTailMutation(true, true, MotionRejectReason::INVALID_GEOMETRY);
            if (!axis.isHomed)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_Homed,
                    m_pendingSourcePC, axisIndex);
                return rejectWithoutTailMutation(false, true, MotionRejectReason::NOT_READY);
            }
            if (m_pCoordMgr != nullptr &&
                !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, targetPos_mm[slot]))
            {
                AlarmManager::GetInstance().Trigger(m_pCoordMgr->GetSoftwareTravelLimitAlarmCode(
                    axis, AlarmManager::PROGRAMMED_OVER_TRAVEL), m_pendingSourcePC, axisIndex);
                return rejectWithoutTailMutation(true, true, MotionRejectReason::INVALID_GEOMETRY);
            }
        }
        const double axisAccTime = useG53Profile ? axis.G53_acc_time : axis.G00_acc_time;
        const double axisDecTime = useG53Profile ? axis.G53_dec_time : axis.G00_dec_time;
        const double axisMaxPPS = useG53Profile ? axis.G53_PPS : axis.G00_PPS;
        if (useG53Profile && (axisAccTime < 0.0 || axisDecTime < 0.0 || axisMaxPPS <= 0.0))
            return rejectWithoutTailMutation(true, true, MotionRejectReason::INVALID_GEOMETRY);

        const std::uint32_t axisBit =
            static_cast<std::uint32_t>(1U << axisIndex);

        if (!std::isfinite(axisAccTime) ||
            !std::isfinite(axisDecTime) ||
            !std::isfinite(axisMaxPPS) ||
            !std::isfinite(axis.finalLead) ||
            !std::isfinite(axis.resolution_PPR) ||
            !std::isfinite(
                stagedQueueTailPulse[
                    static_cast<std::size_t>(axisIndex)]) ||
            axis.resolution_PPR <= 0.0)
        {
            return rejectWithoutTailMutation(
                true,
                true,
                MotionRejectReason::INVALID_GEOMETRY);
        }

                    if ((stagedValidMask & axisBit) == 0U)
                    {
                        return rejectWithoutTailMutation(
                            false,
                            true,
                            MotionRejectReason::NOT_READY);
                    }

                    const bool rotaryShortestPath =
                        axis.axisType == AxisType::ROTARY && axis.useShortestPath;
                    double pulsePerUnit = 0.0;
                    if (!TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead,
                        rotaryShortestPath, pulsePerUnit))
                    {
                        return rejectWithoutTailMutation(
                            true,
                            true,
                            MotionRejectReason::INVALID_GEOMETRY);
                    }

                    const double startPulse =
                        stagedQueueTailPulse[
                            static_cast<std::size_t>(axisIndex)];
                    // BASE-PLANE-3: a coupled selected axis can be exactly
                    // stationary after G68. Keep its proven native start pulse
                    // instead of introducing a forward/reverse-conversion ULP.
                    // G17 and legacy/nontransactional rapid arithmetic is unchanged.
                    const bool stationaryNative = basePlaneLinear && transactionalTail &&
                        targetPos_mm[slot] == stagedCommandedMCS[static_cast<std::size_t>(axisIndex)];
                    double targetPulse = stationaryNative ? startPulse :
                        targetPos_mm[slot] * pulsePerUnit;
                    if (!std::isfinite(targetPulse))
                    {
                        return rejectWithoutTailMutation(
                            true,
                            true,
                            MotionRejectReason::INVALID_GEOMETRY);
                    }

                    if (!TryResolveMotionTargetPulse(startPulse, targetPulse,
                        pulsePerUnit, rotaryShortestPath, axis.rotaryModulo, targetPulse))
                    {
                        return rejectWithoutTailMutation(
                            true,
                            true,
                            MotionRejectReason::INVALID_GEOMETRY);
                    }
                    if (rotaryShortestPath)
                    {
                        const double resolvedTargetUnits = targetPulse / pulsePerUnit;
                        if (!std::isfinite(resolvedTargetUnits))
                        {
                            return rejectWithoutTailMutation(
                                true, true, MotionRejectReason::INVALID_GEOMETRY);
                        }
                        // Check the actual unwrapped endpoint before queue/tail commit.
                        if (m_pCoordMgr != nullptr &&
                            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, resolvedTargetUnits))
                        {
                            AlarmManager::GetInstance().Trigger(
                                m_pCoordMgr->GetSoftwareTravelLimitAlarmCode(axis, AlarmManager::PROGRAMMED_OVER_TRAVEL), m_pendingSourcePC, axisIndex);
                            return rejectWithoutTailMutation(
                                true, true, MotionRejectReason::INVALID_GEOMETRY);
                        }
                    }

                    seenAxis[static_cast<std::size_t>(axisIndex)] = true;
                    stagedQueueTailPulse[static_cast<std::size_t>(axisIndex)] =
                        targetPulse;
                    if (transactionalTail)
                    {
                        stagedCommandedMCS[
                            static_cast<std::size_t>(axisIndex)] =
                            targetPos_mm[slot];
                    }
                    targetPosPulse[slot] = targetPulse;
                    stagedAxisMask |= axisBit;
                    stagedValidMask |= axisBit;
                    if (transactionalTail)
                    {
                        receipt.axisMask = stagedAxisMask;
                    }

                    groupAccTime = std::max<double>(
                        groupAccTime,
                        axisAccTime);
                    groupDecTime = std::max<double>(
                        groupDecTime,
                        axisDecTime);

                    const double distancePulse =
                        std::abs(targetPulse - startPulse);
                    const double distanceUnit = distancePulse / pulsePerUnit;
                    sumSquaredPulse += distancePulse * distancePulse;
                    sumSquaredUnit += distanceUnit * distanceUnit;

                    const double currentAxisMaxPPS =
                        useG53Profile ? axisMaxPPS : axisMaxPPS * rapidOverrideCandidate;
                    if (std::isfinite(currentAxisMaxPPS) &&
                        currentAxisMaxPPS > (useG53Profile ? 0.0 : 1.0))
                    {
                        maxTimeNeeded = std::max<double>(
                            maxTimeNeeded,
                            distancePulse / currentAxisMaxPPS);
                    }
    }

    if (groupAccTime < 0.001)
    {
        groupAccTime = 0.2;
    }
    if (groupDecTime < 0.001)
    {
        groupDecTime = 0.2;
    }

    const double totalDistancePulse = std::sqrt(sumSquaredPulse);
    const double totalDistanceUnit = std::sqrt(sumSquaredUnit);
    double groupG00VelocityPPS = 0.0;

    const double targetFeedrateUnitPerMinute = 5000.0;
    const double targetFeedrateUnitPerSecond =
        targetFeedrateUnitPerMinute / 60.0;
    if (useG53Profile)
    {
        if (maxTimeNeeded > 0.0) groupG00VelocityPPS = totalDistancePulse / maxTimeNeeded;
    }
    else if (totalDistanceUnit > 0.0001 &&
        targetFeedrateUnitPerSecond > 0.0)
    {
        const double exactTimeNeeded =
            totalDistanceUnit / targetFeedrateUnitPerSecond;
        const double finalMotionTime =
            std::max<double>(exactTimeNeeded, maxTimeNeeded);
        if (finalMotionTime > 0.0 && std::isfinite(finalMotionTime))
        {
            groupG00VelocityPPS =
                totalDistancePulse / finalMotionTime;
        }
    }

    if (!std::isfinite(groupG00VelocityPPS))
    {
        return rejectWithoutTailMutation(
            true,
            true,
            MotionRejectReason::INVALID_GEOMETRY);
    }

    const std::uint64_t stagedCommittedFingerprint =
        transactionalTail
        ? BuildQueueTailFingerprint(
            stagedQueueTailPulse,
            stagedCommandedMCS.data(),
            rapidOverrideCandidate)
        : MOTION_QUEUE_TAIL_FINGERPRINT_SEED;

    MotionExecutionIdentity producedIdentity{};
    MotionOwnerLease producedOwnerLease{};
    const bool accepted = TryLineMove(
        axes,
        targetPosPulse,
        groupG00VelocityPPS,
        groupAccTime,
        groupDecTime,
        mode,
        commandPathMode,
        &producedIdentity,
        &producedOwnerLease,
        plannedTailEpoch,
        &plannedTailOwnerLease);

    if (!accepted)
    {
        receipt.identity = producedIdentity;
        receipt.ownerLease = producedOwnerLease;
        return rejectWithoutTailMutation(
            false,
            false,
            MotionRejectReason::NOT_READY);
    }

    receipt.identity = producedIdentity;
    receipt.ownerLease = producedOwnerLease;

    // Non-blocking E/O/E/O validation closes both sides of the assignment
    // window.  The sidecar tag remains unchanged until the second stable check,
    // so any lifecycle drift leaves future G00 planning fail-closed.
    const auto executionOwnerTupleIsStable =
        [&](const MotionExecutionIdentity& identity,
            const MotionOwnerLease& ownerLease) -> bool
    {
        const std::uint64_t ownerState1 =
            m_motionOwnerState.load(std::memory_order_acquire);
        const MotionExecutionEpoch epoch1 =
            GetCurrentExecutionEpoch();
        const MotionOwnerLease owner1 =
            UnpackMotionOwnerState(ownerState1);
        const MotionExecutionEpoch epoch2 =
            GetCurrentExecutionEpoch();
        const std::uint64_t ownerState2 =
            m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease owner2 =
            UnpackMotionOwnerState(ownerState2);

        return
            identity.IsAssigned() &&
            ownerLease.IsValid() &&
            identity.source == commandSource &&
            epoch1 == identity.epoch &&
            epoch2 == identity.epoch &&
            owner1.IsValid() &&
            owner2.IsValid() &&
            owner1.Matches(ownerLease) &&
            owner2.Matches(ownerLease) &&
            owner1.Matches(owner2) &&
            ownerState1 == ownerState2 &&
            !UnpackMotionOwnerSafetyHandshake(ownerState2) &&
            UnpackMotionOwnerSafetyRequestTicket(ownerState2) ==
            m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire);
    };

    const auto failAcceptedCommit =
        [&](bool assignmentCommitted) -> bool
    {
        // BQ: post-enqueue tuple failures keep the data export unpublished.
        if (commandedEndpointReceipt != nullptr)
        {
            commandedEndpointReceipt->Clear();
        }

        if (transactionalTail)
        {
            receipt.commandAccepted = true;
            receipt.commandedMCSCommitted = assignmentCommitted;
            receipt.lastQueuedPulseCommitted = assignmentCommitted;
            receipt.rapidOverrideCommitted = assignmentCommitted;
            receipt.preservedOnReject = false;
            receipt.endpointExact = false;
            receipt.accountingValid = false;
            receipt.committedFingerprint =
                assignmentCommitted
                ? stagedCommittedFingerprint
                : receipt.beforeFingerprint;
            PublishQueueTailTransactionReceipt(receipt, false);
        }

        (void)TryPublishGroupMappingIntegrityAlarmRequest(
            GetCurrentExecutionEpoch());
        RequestEmergencyStopAllAxes();
        return false;
    };

    if (!executionOwnerTupleIsStable(
        producedIdentity,
        producedOwnerLease))
    {
        return failAcceptedCommit(false);
    }

    if (useG53Profile && commandSource == MotionCommandSource::NC_MEMORY)
    {
        bool captureMatches = false;
        if (m_programBlockMotionCaptureActive && !m_programBlockMotionCapture.overflow &&
            m_programBlockMotionCapture.count == 1U)
        {
            const MotionProgramBlockSubmission& submission = m_programBlockMotionCapture.submissions[0U];
            captureMatches = submission.producerAccepted &&
                submission.immediateRejectReason == MotionRejectReason::NONE &&
                submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
                submission.translationGeneration == m_pendingTranslation.generation &&
                submission.identity.epoch == producedIdentity.epoch &&
                submission.identity.segmentId == producedIdentity.segmentId &&
                submission.identity.sourceBlockId == producedIdentity.sourceBlockId &&
                submission.identity.source == producedIdentity.source;
        }
        if (!IsNCTranslationSnapshotEmpty(m_pendingTranslation) &&
            (!IsPendingFixedTranslationSourceAllowed() || !MatchesNCTranslation(m_pendingTranslation)))
            captureMatches = false;
        if (!captureMatches)
        {
            m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
            m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
            m_g00ProducerQueueTailValidMask = 0U;
            return failAcceptedCommit(false);
        }
    }

    // ProducerTryPush() is the enqueue linearization point. Everything below
    // is an assignment-only commit from precomputed candidates. ABORTING
    // replaces every existing-axis baseline; BUFFERED extends the tagged tail.
    m_g00ProducerQueueTailPulse = stagedQueueTailPulse;
    if (mode == BufferMode::ABORTING)
    {
        for (std::size_t axisSlot = 0U;
            axisSlot < static_cast<std::size_t>(MAX_AXES);
            ++axisSlot)
        {
            const std::uint32_t axisBit =
                (1U << static_cast<unsigned>(axisSlot));
            if ((stagedValidMask & axisBit) != 0U)
            {
                (*m_pContexts)[axisSlot].lastQueuedPulse.Store(
                    stagedQueueTailPulse[axisSlot]);
            }
        }
    }
    else
    {
        for (std::size_t slot = 0U; slot < axes.size(); ++slot)
        {
            const std::size_t axisSlot =
                static_cast<std::size_t>(axes[slot]);
            (*m_pContexts)[axisSlot].lastQueuedPulse.Store(
                stagedQueueTailPulse[axisSlot]);
        }
    }

    if (transactionalTail)
    {
        for (int axisIndex = 0; axisIndex < MAX_AXES; ++axisIndex)
        {
            commandedMCSTail[axisIndex] =
                stagedCommandedMCS[
                    static_cast<std::size_t>(axisIndex)];
        }
    }
    m_g00ProducerQueueTailValidMask = stagedValidMask;
    G00_overrideRatio = rapidOverrideCandidate;

    if (!executionOwnerTupleIsStable(
        producedIdentity,
        producedOwnerLease))
    {
        return failAcceptedCommit(true);
    }

    // Publish the tuple last. No fallible endpoint readback follows this seal.
    m_g00ProducerQueueTailEpoch = producedIdentity.epoch;
    m_g00ProducerQueueTailOwnerLease = producedOwnerLease;

    // Close request-intent publication that races the tag-last seal. The
    // sidecar is producer-private, so invalidating its identity before return
    // prevents any later G00 from consuming this superseded baseline.
    if (!executionOwnerTupleIsStable(
        producedIdentity,
        producedOwnerLease))
    {
        m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
        m_g00ProducerQueueTailValidMask = 0U;
        return failAcceptedCommit(true);
    }

    if (!transactionalTail)
    {
        return true;
    }

    receipt.commandAccepted = true;
    receipt.commandedMCSCommitted = true;
    receipt.lastQueuedPulseCommitted = true;
    receipt.rapidOverrideCommitted = true;
    receipt.preservedOnReject = false;
    receipt.endpointExact = true;
    receipt.accountingValid = true;
    receipt.committedFingerprint = stagedCommittedFingerprint;
    PublishQueueTailTransactionReceipt(receipt, false);

    // BQ: PublishQueueTailTransactionReceipt fills captureBound; copy its final
    // value only after publication. Data validation never changes G00 outcome.
    if (commandedEndpointReceipt != nullptr)
    {
        commandedEndpointReceipt->transaction = receipt;
        commandedEndpointReceipt->endMCS = stagedCommandedMCS;
        const std::uint32_t validAxisMask =
            commandedEndpointReceipt->validAxisMask;
        bool endpointValid = receipt.IsCommitted() && receipt.captureBound &&
            validAxisMask != 0U &&
            (validAxisMask >> MAX_AXES) == 0U &&
            (receipt.axisMask & validAxisMask) == receipt.axisMask;
        for (std::size_t axisSlot = 0U;
            axisSlot < MOTION_COMMANDED_ENDPOINT_AXIS_COUNT;
            ++axisSlot)
        {
            endpointValid = endpointValid &&
                std::isfinite(commandedEndpointReceipt->startMCS[axisSlot]) &&
                std::isfinite(commandedEndpointReceipt->endMCS[axisSlot]);
        }
        if (endpointValid)
        {
            commandedEndpointReceipt->schemaVersion =
                MOTION_COMMANDED_ENDPOINT_SCHEMA_VERSION;
            commandedEndpointReceipt->valid = true;
        }
        else
        {
            commandedEndpointReceipt->Clear();
        }
    }
    return true;
}
