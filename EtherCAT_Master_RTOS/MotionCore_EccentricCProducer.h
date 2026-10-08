// Included once by MotionCore_G01.cpp after FeedLineNormalOverride.
#include "MotionEccentricCProducer.h"
#include "NCEccentricCFeedScope.h"
#include <limits>
#include <cstring>
#include <windows.h>
#include <rtapi.h>

#if defined(_MSC_VER)
#define BASE79G_PRODUCER_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79G_PRODUCER_NOINLINE __attribute__((noinline))
#else
#define BASE79G_PRODUCER_NOINLINE
#endif

BASE79G_PRODUCER_NOINLINE bool MotionCore::TryG01EccentricCMoveTransactionalTail(
    double programmedC, double programmedFeed, double(&commandedMCSTail)[MAX_AXES],
    MotionEccentricCProducerWorkspace& workspace, MotionFeedLineReceipt& result,
    double programmedZ, bool hasProgrammedZ)
{
    const std::array<double, 3> programmedXYZ{{ 0.0, 0.0, programmedZ }};
    return TryG01EccentricXYZCMoveTransactionalTail(programmedC, programmedFeed,
        commandedMCSTail, workspace, result, programmedXYZ, hasProgrammedZ ? 4U : 0U);
}

BASE79G_PRODUCER_NOINLINE bool MotionCore::TryG01EccentricXYZCMoveTransactionalTail(
    double programmedC, double programmedFeed, double(&commandedMCSTail)[MAX_AXES],
    MotionEccentricCProducerWorkspace& workspace, MotionFeedLineReceipt& result,
    const std::array<double, 3>& programmedXYZ, std::uint32_t linearMask)
{
    using namespace NCEccentricCDetail;
    result.Clear();
    result.eccentricFeed = true;
    MotionEccentricCTransportDetail::ResetInPlace(workspace.input);
    MotionEccentricCTransportDetail::ResetInPlace(workspace.runtime);
    MotionEccentricCTransportDetail::ResetInPlace(workspace.command);
    workspace.originalTail.fill(0.0);
    workspace.physicalEndMCS.fill(0.0);
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source = m_pendingCommandSource.load(std::memory_order_acquire);
    const std::uint32_t alarmRevision = AlarmManager::GetInstance().GetUpdateCount();
    const std::uint64_t alarmIntent = AlarmManager::MotionAdmissionBaseState(
        AlarmManager::GetInstance().GetMotionSafetyIntentState());
    auto& input = workspace.input;
    input.geometry.source = m_pendingTranslation;
    const auto& frozen = input.geometry.source;
    unsigned role = 8U;
    const bool authoredZ = (linearMask & 4U) != 0U;
    const bool authoredXY = (linearMask & 3U) != 0U;
    const bool mixedLinear = authoredXY || authoredZ;
    const double programmedZ = programmedXYZ[2];
    bool linearShapeValid = (linearMask & ~7U) == 0U;
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        const bool authored = (linearMask & (1U << axis)) != 0U;
        if (!std::isfinite(programmedXYZ[axis]) ||
            (!authored && programmedXYZ[axis] != 0.0) ||
            (authored && frozen.distanceMode == 91 &&
                programmedXYZ[axis] == 0.0)) linearShapeValid = false;
    }
    // This entry is deliberately narrower than the pure geometry kernels and
    // marked consumer: C/index 3, signed <=10 degrees, <=2 mm XY eccentricity.
    if (m_pContexts == nullptr || m_pContexts->size() < 4U || m_pContexts->size() > 8U ||
        m_pCoordMgr == nullptr || !IsEccentricCConsumerReady() ||
        !Scope(frozen) || !Role(frozen, role) || role != 3U ||
        frozen.axisIdentity.address[0] != 'X' || frozen.axisIdentity.address[1] != 'Y' ||
        frozen.axisIdentity.address[2] != 'Z' || frozen.axisIdentity.address[3] != 'C' ||
        (frozen.distanceMode != 90 && frozen.distanceMode != 91) ||
        !std::isfinite(programmedC) || !Positive(programmedFeed) ||
        !linearShapeValid ||
        (!mixedLinear && programmedFeed > 100.0) ||
        !std::isfinite(std::hypot(frozen.toolOffsetMM[0], frozen.toolOffsetMM[1])) ||
        std::hypot(frozen.toolOffsetMM[0], frozen.toolOffsetMM[1]) > 2.0)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    const bool plannedShortestPath = (*m_pContexts)[role].useShortestPath;
    const double plannedRotaryModulo = (*m_pContexts)[role].rotaryModulo;
    // BASE79M-DIAG1 observes the first rejected predicate in its original
    // short-circuit order. It grants no new authority and changes no endpoint.
    unsigned sourceRejectGate = 0U;
    unsigned entryRejectGate = 0U;
    MotionFeedHoldStopSnapshot overrideDiagnostic{};
    const auto sourceGate = [&](bool ok, unsigned gate) noexcept -> bool
    { if (!ok) sourceRejectGate = gate; return ok; };
    const auto entryGate = [&](bool ok, unsigned gate) noexcept -> bool
    { if (!ok) entryRejectGate = gate; return ok; };
    const auto sourceCurrent = [&]() noexcept -> bool
    {
        sourceRejectGate = 0U;
        return sourceGate(frozen.distanceMode != 90 ||
            ((*m_pContexts)[role].useShortestPath == plannedShortestPath &&
                Same((*m_pContexts)[role].rotaryModulo, plannedRotaryModulo)), 1U) &&
            sourceGate(m_pCoordMgr->IsTranslationRunFrozen(), 2U) &&
            sourceGate(m_pCoordMgr->IsTranslationRunCurrent(), 3U) &&
            sourceGate(SameNCTranslationSnapshot(frozen, m_pCoordMgr->GetTranslationSnapshot()), 4U) &&
            sourceGate(SameNCTranslationSnapshot(frozen, m_pendingTranslation), 5U) &&
            sourceGate(MatchesNCTranslation(frozen), 6U) &&
            sourceGate(m_pendingSourcePC >= 0, 7U) &&
            sourceGate(m_pendingCommandSource.load(std::memory_order_acquire) == source, 8U) &&
            sourceGate(m_pendingSourceWCS == frozen.wcsCode, 9U) &&
            sourceGate(m_pendingToolMode == frozen.toolLengthMode, 10U) &&
            sourceGate(m_pendingHCode == frozen.toolHCode, 11U) &&
            sourceGate(m_pendingToolRadMode == 40, 12U) &&
            sourceGate(m_pendingDCode == 0, 13U) &&
            sourceGate(m_pendingIsAbsoluteMode == (frozen.distanceMode == 90), 14U) &&
            sourceGate(m_pendingPlaneMode == 17, 15U) &&
            sourceGate(m_pendingG162Active, 16U) &&
            sourceGate(!m_pendingG68Active, 17U) &&
            sourceGate(Same(m_pendingG68Angle, 0.0), 18U) &&
            sourceGate(!m_pendingG168Active, 19U) &&
            sourceGate(m_pendingWCode == 0, 20U) &&
            sourceGate(!m_pendingG51Active, 21U) &&
            sourceGate(Same(m_pendingScaleRatio, 1.0), 22U) &&
            sourceGate(m_pendingMirrorMask == 0U, 23U) &&
            sourceGate(!m_pendingG16Active, 24U);
    };
    const auto diagnosticBits = [](double value) noexcept -> unsigned long long
    {
        std::uint64_t bits = 0ULL;
        static_assert(sizeof(bits) == sizeof(value), "binary64 diagnostic");
        std::memcpy(&bits, &value, sizeof(bits));
        return static_cast<unsigned long long>(bits);
    };
    const auto traceNotReady = [&](const char* stage, int axisIndex) noexcept
    {
        // NC producer thread only, once on refusal; no RT per-tick output.
        RtPrintf("[BASE79M-DIAG1][ECC-NOT-READY] stage=%s pc=%d axis=%d entryGate=%u sourceGate=%u epoch=%llu owner=%u ownerGen=%llu mode=%d linearMask=%u accepted=%u\n",
            stage, m_pendingSourcePC, axisIndex, entryRejectGate, sourceRejectGate,
            static_cast<unsigned long long>(plannedEpoch), static_cast<unsigned>(plannedOwner.owner),
            static_cast<unsigned long long>(plannedOwner.generation), frozen.distanceMode,
            static_cast<unsigned>(linearMask), result.commandAccepted ? 1U : 0U);
        if (entryRejectGate == 13U || entryRejectGate == 22U)
        {
            const auto& s = overrideDiagnostic;
            RtPrintf("[BASE79M-DIAG1][ECC-STOP-SNAPSHOT] exactEvaluated=1 overrideBits=%llu zero=%u done=%u active=%u queue=%u ingress=%u replay=%u fault=%u emergency=%u safety=%u faultAxes=%u commandStopped=%u virtualStopped=%u axisStopped=%u virtualVelBits=%llu axisVelBits=%llu\n",
                diagnosticBits(s.feedrateOverride), s.overrideZero ? 1U : 0U,
                s.groupDone ? 1U : 0U, s.groupActive ? 1U : 0U,
                static_cast<unsigned>(s.commandQueueDepth), static_cast<unsigned>(s.commandIngressDepth),
                static_cast<unsigned>(s.commandReplayDepth), s.groupFaulted ? 1U : 0U,
                s.groupEmergencyStopped ? 1U : 0U, s.safetyOrRecoveryPending ? 1U : 0U,
                static_cast<unsigned>(s.faultedAxes), s.commandStopped ? 1U : 0U,
                s.virtualCommandStopped ? 1U : 0U, s.axisCommandStopped ? 1U : 0U,
                diagnosticBits(s.virtualCommandVelocityPps), diagnosticBits(s.maxAxisCommandVelocityPps));
        }
        if (axisIndex >= 0 && static_cast<std::size_t>(axisIndex) < m_pContexts->size())
        {
            const unsigned a = static_cast<unsigned>(axisIndex);
            const AxisContext& axis = (*m_pContexts)[a];
            const double pulse = input.startPulse[a], ppu = input.pulsePerNative[a];
            const double tail = workspace.originalTail[a];
            const double direct = pulse / ppu;
            const double legacy = pulse * axis.finalLead / axis.resolution_PPR;
            double modulo = std::numeric_limits<double>::quiet_NaN();
            if (axis.axisType == AxisType::ROTARY && Positive(axis.rotaryModulo))
            { modulo = std::fmod(legacy, axis.rotaryModulo); if (modulo < 0.0) modulo += axis.rotaryModulo; }
            const unsigned equalityMask = (tail * ppu == pulse ? 1U : 0U) |
                (tail == direct ? 2U : 0U) | (tail == legacy ? 4U : 0U) |
                (std::isfinite(modulo) && tail == modulo ? 8U : 0U);
            RtPrintf("[BASE79M-DIAG1][ECC-BASIS] axis=%u type=%u sampledTailBits=%llu sampledPulseBits=%llu ppuBits=%llu forwardBits=%llu directBits=%llu legacyBits=%llu moduloBits=%llu equalityMask=%u leadBits=%llu pprBits=%llu shortest=%u rotaryModuloBits=%llu currentTailBits=%llu currentPulseBits=%llu\n",
                a, static_cast<unsigned>(axis.axisType), diagnosticBits(tail), diagnosticBits(pulse),
                diagnosticBits(ppu), diagnosticBits(tail * ppu), diagnosticBits(direct),
                diagnosticBits(legacy), diagnosticBits(modulo), equalityMask,
                diagnosticBits(axis.finalLead), diagnosticBits(axis.resolution_PPR),
                axis.useShortestPath ? 1U : 0U, diagnosticBits(axis.rotaryModulo),
                diagnosticBits(commandedMCSTail[a]), diagnosticBits(axis.logicalCmdPos.Load()));
            const auto& previous = m_zcFeedProducerTail;
            RtPrintf("[BASE79M-DIAG1][ECC-BASIS-PROOF] observedAfterReject=1 axis=%u queuePulseBits=%llu queueMask=%u queueEpoch=%llu queueOwner=%u queueOwnerGen=%llu prevValid=%u prevMask=%u prevValidMask=%u prevPulseBits=%llu prevMCSBits=%llu prevEpoch=%llu prevSegment=%llu prevSource=%u prevOwner=%u prevOwnerGen=%llu\n",
                a, diagnosticBits(m_g00ProducerQueueTailPulse[a]),
                static_cast<unsigned>(m_g00ProducerQueueTailValidMask),
                static_cast<unsigned long long>(m_g00ProducerQueueTailEpoch),
                static_cast<unsigned>(m_g00ProducerQueueTailOwnerLease.owner),
                static_cast<unsigned long long>(m_g00ProducerQueueTailOwnerLease.generation),
                previous.valid ? 1U : 0U, static_cast<unsigned>(previous.axisMask),
                static_cast<unsigned>(previous.validAxisMask), diagnosticBits(previous.endPulse[a]),
                diagnosticBits(previous.endMCS[a]), static_cast<unsigned long long>(previous.identity.epoch),
                static_cast<unsigned long long>(previous.identity.segmentId),
                static_cast<unsigned>(previous.identity.source), static_cast<unsigned>(previous.ownerLease.owner),
                static_cast<unsigned long long>(previous.ownerLease.generation));
        }
    };
    if (!entryGate(plannedEpoch != MOTION_EXECUTION_EPOCH_INVALID, 1U) ||
        !entryGate(plannedOwner.IsValid(), 2U) ||
        !entryGate(plannedOwner.owner == MotionOwner::AUTO, 3U) ||
        !entryGate(source == MotionCommandSource::NC_MEMORY, 4U) ||
        !entryGate(m_programBlockMotionCaptureActive, 5U) ||
        !entryGate(!m_programBlockMotionCapture.overflow, 6U) ||
        !entryGate(m_programBlockMotionCapture.count == 0U, 7U) ||
        !entryGate(!HasPendingExecutionEpochChange(), 8U) ||
        !entryGate(!HasPendingSafetyOrRecoveryRequests(), 9U) ||
        !entryGate(static_cast<std::uint32_t>(alarmIntent) == 0U, 10U) ||
        !entryGate(!AlarmManager::GetInstance().HasAlarm(), 11U) ||
        !entryGate(FeedLineNormalOverride(*this, false, &overrideDiagnostic), 13U) ||
        !entryGate(sourceCurrent(), 14U))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        traceNotReady("ENTRY", -1);
        return false;
    }
    for (unsigned index = 3U; index < 8U; ++index)
        if (frozen.toolOffsetMM[index] != 0.0)
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
    // Retain each physical axis ceiling before canonical packet compression,
    // so a late configuration change cannot publish a stale mixed-axis proof.
    std::array<std::array<double, 4>, 4> plannedAxisRates{};
    input.firSamples = 1U;
    input.travelMask = 3U | (mixedLinear ? 4U : 0U) | (1U << role);
    const bool generatedXY = frozen.toolOffsetMM[0] != 0.0 || frozen.toolOffsetMM[1] != 0.0;
    const std::uint32_t groupMask = (generatedXY ? 3U : 0U) |
        linearMask | (1U << role);
    const auto baselineBound = [&](unsigned index, const AxisContext& axis,
        double pulse, double ppu) noexcept -> bool
    {
        const double tail = commandedMCSTail[index];
        const auto& previous = m_zcFeedProducerTail;
        const std::uint32_t bit = 1U << index;
        if (previous.valid && previous.identity.IsAssigned() &&
            previous.identity.epoch == plannedEpoch && previous.identity.source == source &&
            previous.ownerLease.Matches(plannedOwner) &&
            (previous.axisMask & bit) != 0U && (previous.validAxisMask & bit) != 0U &&
            m_g00ProducerQueueTailEpoch == plannedEpoch &&
            m_g00ProducerQueueTailOwnerLease.Matches(plannedOwner) &&
            (m_g00ProducerQueueTailValidMask & bit) != 0U &&
            Same(previous.endPulse[index], pulse) && Same(m_g00ProducerQueueTailPulse[index], pulse) &&
            Same(previous.endMCS[index], tail)) return true;
        const double direct = pulse / ppu;
        const double legacy = pulse * axis.finalLead / axis.resolution_PPR;
        if (tail * ppu == pulse || tail == direct || tail == legacy) return true;
        if (axis.axisType != AxisType::ROTARY || !Positive(axis.rotaryModulo)) return false;
        double modulo = std::fmod(legacy, axis.rotaryModulo);
        if (modulo < 0.0) modulo += axis.rotaryModulo;
        return std::isfinite(modulo) && tail == modulo;
    };
    for (unsigned index = 0U; index < 8U; ++index)
    {
        const double original = commandedMCSTail[index];
        if (!std::isfinite(original))
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
        workspace.originalTail[index] = workspace.physicalEndMCS[index] = original;
        input.startPulse[index] = original;
        input.pulsePerNative[index] = 1.0;
        const bool present = index < m_pContexts->size() && (*m_pContexts)[index].isExist;
        if (present)
        {
            const AxisContext& axis = (*m_pContexts)[index];
            double ppu = 0.0;
            if (axis.axisIndex != static_cast<int>(index) || axis.isVirtualAxis ||
                !TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead,
                    axis.axisType != AxisType::LINEAR, ppu) ||
                m_pCoordMgr->GetInvalidSoftwareTravelLimitMask(axis) != 0U)
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            input.startPulse[index] = axis.logicalCmdPos.Load();
            input.pulsePerNative[index] = ppu;
            if (!std::isfinite(input.startPulse[index]) ||
                !baselineBound(index, axis, input.startPulse[index], ppu))
            {
                result.code = MotionFeedLineCode::NOT_READY;
                traceNotReady("BASELINE", static_cast<int>(index));
                return false;
            }
            result.validAxisMask |= 1U << index;
        }
        input.geometry.startMCS[index] = input.startPulse[index] / input.pulsePerNative[index];
        input.travelMinNative[index] = input.travelMaxNative[index] = input.geometry.startMCS[index];
        if (index != 0U && index != 1U && index != role && !(mixedLinear && index == 2U)) continue;
        const AxisContext& axis = (*m_pContexts)[index];
        const double velocity = axis.maxVel_PPS / input.pulsePerNative[index];
        if (!Positive(velocity) || !Positive(axis.G00_acc_time) || !Positive(axis.G00_dec_time) ||
            !std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.001 || axis.Stop_dec_time > 60.0)
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
        plannedAxisRates[index] = {{ velocity, axis.G00_acc_time,
            axis.G00_dec_time, axis.Stop_dec_time }};
        input.geometry.maxVelocityNative[index] = velocity;
        input.geometry.maxAccelerationNative[index] = velocity / axis.G00_acc_time;
        input.geometry.maxDecelerationNative[index] = velocity / axis.G00_dec_time;
        input.geometry.accTime = Max(input.geometry.accTime, axis.G00_acc_time);
        input.geometry.decTime = Max(input.geometry.decTime, axis.G00_dec_time);
        if ((groupMask & (1U << index)) != 0U)
            input.stopSeconds = Max(input.stopSeconds, axis.Stop_dec_time);
        input.travelMinNative[index] = -(std::numeric_limits<double>::max)();
        input.travelMaxNative[index] = (std::numeric_limits<double>::max)();
        if (axis.isHomed)
        {
            const bool enabled[3] = {
                axis.travelLimit1Enable && m_pCoordMgr->IsProgrammableTravelLimitEnabled(),
                axis.travelLimit2Enable, axis.travelLimit3Enable };
            const double lower[3] = { axis.travelLimit1Negative_unit,
                axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit };
            const double upper[3] = { axis.travelLimit1Positive_unit,
                axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit };
            for (unsigned range = 0U; range < 3U; ++range)
                if (enabled[range])
                {
                    input.travelMinNative[index] = Max(input.travelMinNative[index], lower[range]);
                    input.travelMaxNative[index] = Min(input.travelMaxNative[index], upper[range]);
                }
        }
    }
    const auto axisContextCurrent = [&]() noexcept -> bool
    {
        // Keep the accepted C-only transaction contract unchanged. The new
        // compressed linear+C variants independently retain the four live rates,
        // pulse scales and effective travel bounds that created its proof.
        if (!mixedLinear) return true;
        if (m_pContexts == nullptr || m_pContexts->size() < 4U || m_pContexts->size() > 8U)
            return false;
        for (unsigned index = 0U; index < 4U; ++index)
        {
            const AxisContext& axis = (*m_pContexts)[index];
            double ppu = 0.0;
            if (!axis.isExist || axis.isVirtualAxis || axis.axisIndex != static_cast<int>(index) ||
                axis.axisType != (index < 3U ? AxisType::LINEAR : AxisType::ROTARY) ||
                !TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead,
                    axis.axisType != AxisType::LINEAR, ppu) ||
                !Same(ppu, input.pulsePerNative[index]) ||
                !Same(axis.maxVel_PPS / ppu, plannedAxisRates[index][0]) ||
                !Same(axis.G00_acc_time, plannedAxisRates[index][1]) ||
                !Same(axis.G00_dec_time, plannedAxisRates[index][2]) ||
                !Same(axis.Stop_dec_time, plannedAxisRates[index][3]) ||
                m_pCoordMgr->GetInvalidSoftwareTravelLimitMask(axis) != 0U) return false;
            double lowerBound = -(std::numeric_limits<double>::max)();
            double upperBound = (std::numeric_limits<double>::max)();
            if (axis.isHomed)
            {
                const bool enabled[3] = {
                    axis.travelLimit1Enable && m_pCoordMgr->IsProgrammableTravelLimitEnabled(),
                    axis.travelLimit2Enable, axis.travelLimit3Enable };
                const double lower[3] = { axis.travelLimit1Negative_unit,
                    axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit };
                const double upper[3] = { axis.travelLimit1Positive_unit,
                    axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit };
                for (unsigned range = 0U; range < 3U; ++range)
                    if (enabled[range])
                    {
                        lowerBound = Max(lowerBound, lower[range]);
                        upperBound = Min(upperBound, upper[range]);
                    }
            }
            if (!Same(lowerBound, input.travelMinNative[index]) ||
                !Same(upperBound, input.travelMaxNative[index])) return false;
        }
        return true;
    };
    if (!axisContextCurrent())
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    NCEccentricCFeedTarget target{};
    NCEccentricCZFeedTarget zTarget{};
    std::array<NCEccentricCXYFeedTarget, 2U> xyTarget{};
    const AxisContext& rotary = (*m_pContexts)[role];
    if (!TryResolveNCEccentricCFeedTarget(frozen, programmedC, input.startPulse[role],
        input.pulsePerNative[role], plannedShortestPath, plannedRotaryModulo, target) ||
        (authoredZ && !TryResolveNCEccentricCZFeedTarget(frozen, programmedZ,
            input.startPulse[2], input.pulsePerNative[2], zTarget)))
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    for (unsigned axis = 0U; axis < 2U; ++axis)
        if ((linearMask & (1U << axis)) != 0U &&
            !TryResolveNCEccentricCXYFeedTarget(frozen, axis, programmedXYZ[axis],
                input.startPulse[axis], input.pulsePerNative[axis],
                input.geometry.startMCS[role], target.endMCS, xyTarget[axis]))
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
    input.geometry.xDeltaMM = authoredXY && frozen.distanceMode == 91 ?
        programmedXYZ[0] : xyTarget[0].deltaMM;
    input.geometry.yDeltaMM = authoredXY && frozen.distanceMode == 91 ?
        programmedXYZ[1] : xyTarget[1].deltaMM;
    input.geometry.zDeltaMM = authoredZ ? zTarget.deltaMM : 0.0;
    input.geometry.sweepDeg = target.sweepDeg;
    if (!TryResolveNCEccentricCLinearFeedRate(input.geometry.xDeltaMM,
        input.geometry.yDeltaMM, input.geometry.zDeltaMM, target.sweepDeg,
        programmedFeed, input.geometry.feedDegMin) ||
        !CanonicalizeMotionEccentricCLinearLimits(input))
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    // G90 proves the authored translated target as well as the physical
    // canonical branch and its complete stopping extension below.
    if (frozen.distanceMode == 90 &&
        (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(rotary, target.requestedMCS) ||
            (authoredZ && !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(
                (*m_pContexts)[2], zTarget.requestedMCS))))
    {
        result.travelLimitRejected = true;
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    if (frozen.distanceMode == 90)
        for (unsigned axis = 0U; axis < 2U; ++axis)
            if ((linearMask & (1U << axis)) != 0U &&
                !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(
                    (*m_pContexts)[axis], xyTarget[axis].requestedMCS))
            {
                result.travelLimitRejected = true;
                result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
                return false;
            }
    const NCEccentricCRuntimeCode geometry = PrepareNCEccentricCRuntime(input, workspace.runtime);
    result.geometryCode = static_cast<std::uint32_t>(geometry);
    if (geometry != NCEccentricCRuntimeCode::PREPARED)
    {
        result.travelLimitRejected = geometry == NCEccentricCRuntimeCode::INVALID_TRAVEL_BOUNDS ||
            geometry == NCEccentricCRuntimeCode::STOP_ENVELOPE_REJECTED;
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    const auto& runtime = workspace.runtime;
    if (!Same(runtime.AuthoredPath().endMCS[role], target.endMCS) ||
        !Same(runtime.EndPulse()[role], target.endPulse) ||
        (authoredZ && (!Same(runtime.AuthoredPath().endMCS[2], zTarget.endMCS) ||
            !Same(runtime.EndPulse()[2], zTarget.endPulse))))
    {
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    for (unsigned axis = 0U; axis < 2U; ++axis)
        if ((linearMask & (1U << axis)) != 0U &&
            (!Same(runtime.AuthoredPath().endMCS[axis], xyTarget[axis].endMCS) ||
                !Same(runtime.EndPulse()[axis], xyTarget[axis].endPulse)))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
    for (unsigned index = 0U; index < 8U; ++index)
    {
        if ((result.validAxisMask & (1U << index)) == 0U) continue;
        const AxisContext& axis = (*m_pContexts)[index];
        if (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, runtime.ExtendedPath().minMCS[index]) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, runtime.ExtendedPath().maxMCS[index]) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, runtime.MinimumPulse()[index] / input.pulsePerNative[index]) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, runtime.MaximumPulse()[index] / input.pulsePerNative[index]))
        {
            result.travelLimitRejected = true;
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
        if ((groupMask & (1U << index)) != 0U)
            workspace.physicalEndMCS[index] = runtime.AuthoredPath().endMCS[index];
    }
    // The geometry proof never grants permission. Stamp a real identity only
    // after the stopped source, anchor and complete stop envelope are proved.
    auto& command = workspace.command;
    command.sourceLinePC = m_pendingSourcePC;
    AssignExecutionIdentity(command, plannedEpoch, source, plannedOwner);
    result.identity = command.execution;
    result.ownerLease = plannedOwner;
    result.translationGeneration = frozen.generation;
    if (!BuildMotionEccentricCCommand(input, result.identity, plannedOwner, command, workspace.transport))
    {
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    const auto tupleStable = [&]() noexcept -> bool
    {
        const std::uint32_t acknowledged1 = m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
        const std::uint64_t state1 = m_motionOwnerState.load(std::memory_order_acquire);
        const MotionExecutionEpoch epoch1 = GetCurrentExecutionEpoch();
        const MotionOwnerLease owner1 = UnpackMotionOwnerState(state1);
        const MotionExecutionEpoch epoch2 = GetCurrentExecutionEpoch();
        const std::uint64_t state2 = m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease owner2 = UnpackMotionOwnerState(state2);
        const std::uint32_t acknowledged2 = m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
        // Before enqueue the stopped producer requires an unreserved word.
        // Once published, the legitimate RT consumer may already reserve the
        // same owner/epoch while loading or stepping. Those transient bits do
        // not change its lease, safety ticket, source or execution identity.
        const bool ownerPublicationCurrent = acknowledged1 == acknowledged2 &&
            !UnpackMotionOwnerSafetyHandshake(state1) && !UnpackMotionOwnerSafetyHandshake(state2) &&
            !UnpackMotionOwnerSafetyActionPending(state1) && !UnpackMotionOwnerSafetyActionPending(state2) &&
            UnpackMotionOwnerSafetyRequestTicket(state1) == acknowledged1 &&
            UnpackMotionOwnerSafetyRequestTicket(state2) == acknowledged1 &&
            (result.commandAccepted || (state1 == state2 &&
                state2 == PackMotionOwnerState(plannedOwner.owner, plannedOwner.generation,
                    acknowledged1, false, false)));
        return result.identity.IsAssigned() && result.ownerLease.Matches(plannedOwner) &&
            result.identity.source == source && result.identity.sourceBlockId == m_pendingSourcePC &&
            epoch1 == plannedEpoch && epoch2 == plannedEpoch &&
            owner1.IsValid() && owner2.IsValid() && owner1.Matches(plannedOwner) && owner2.Matches(plannedOwner) &&
            ownerPublicationCurrent &&
            !HasPendingExecutionEpochChange() && !HasPendingSafetyOrRecoveryRequests() &&
            !AlarmManager::GetInstance().HasAlarm() &&
            AlarmManager::GetInstance().GetUpdateCount() == alarmRevision &&
            AlarmManager::MotionAdmissionBaseState(AlarmManager::GetInstance().GetMotionSafetyIntentState()) == alarmIntent &&
            sourceCurrent() && axisContextCurrent();
    };
    if (!entryGate(tupleStable(), 21U) ||
        !entryGate(FeedLineNormalOverride(*this, false, &overrideDiagnostic), 22U) ||
        !entryGate(m_programBlockMotionCaptureActive, 23U) ||
        !entryGate(!m_programBlockMotionCapture.overflow, 24U) ||
        !entryGate(m_programBlockMotionCapture.count == 0U, 25U))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        traceNotReady("PRE_ENQUEUE", -1);
        return false;
    }
    for (unsigned index = 0U; index < 8U; ++index)
        if (!Same(commandedMCSTail[index], workspace.originalTail[index]) ||
            ((result.validAxisMask & (1U << index)) != 0U &&
                !Same((*m_pContexts)[index].logicalCmdPos.Load(), input.startPulse[index])))
        {
            result.code = MotionFeedLineCode::NOT_READY;
            traceNotReady("TAIL_CHANGED", static_cast<int>(index));
            return false;
        }
    result.commandAccepted = TryEnqueueMotionCommand(command);
    if (!result.commandAccepted)
    {
        result.code = MotionFeedLineCode::PRODUCER_REJECTED;
        return false;
    }
    const auto revokeAccepted = [&](MotionFeedLineCode code) noexcept -> bool
    {
        result.code = code;
        result.valid = false;
        m_rotaryFeedProducerTail.Clear();
        m_zcFeedProducerTail.Clear();
        m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
        m_g00ProducerQueueTailValidMask = 0U;
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch());
        RequestEmergencyStopAllAxes();
        return false;
    };
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    if (!m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 1U)
        return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    const MotionProgramBlockSubmission& submission = m_programBlockMotionCapture.submissions[0U];
    result.captureBound = submission.translationGeneration == result.translationGeneration &&
        submission.producerAccepted && submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        submission.identity.epoch == result.identity.epoch &&
        submission.identity.segmentId == result.identity.segmentId &&
        submission.identity.source == result.identity.source &&
        submission.identity.sourceBlockId == result.identity.sourceBlockId;
    if (!result.captureBound) return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    m_g00ProducerQueueTailPulse = runtime.EndPulse();
    for (unsigned index = 0U; index < 8U; ++index)
    {
        if ((result.validAxisMask & (1U << index)) != 0U)
            (*m_pContexts)[index].lastQueuedPulse.Store(runtime.EndPulse()[index]);
        commandedMCSTail[index] = workspace.physicalEndMCS[index];
    }
    result.tailCommitted = true;
    m_g00ProducerQueueTailValidMask = result.validAxisMask;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    m_g00ProducerQueueTailEpoch = result.identity.epoch;
    m_g00ProducerQueueTailOwnerLease = result.ownerLease;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    m_zcFeedProducerTail.endMCS = workspace.physicalEndMCS;
    m_zcFeedProducerTail.endPulse = runtime.EndPulse();
    m_zcFeedProducerTail.identity = result.identity;
    m_zcFeedProducerTail.ownerLease = result.ownerLease;
    m_zcFeedProducerTail.translationGeneration = result.translationGeneration;
    // This private mask records all proven native bases, not command axes.
    m_zcFeedProducerTail.axisMask = result.validAxisMask;
    m_zcFeedProducerTail.validAxisMask = result.validAxisMask;
    m_zcFeedProducerTail.valid = true;
    m_rotaryFeedProducerTail = m_zcFeedProducerTail;
    m_rotaryFeedProducerTail.axisMask = 1U << role;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    result.code = MotionFeedLineCode::COMMITTED;
    result.valid = true;
    return true;
}

#undef BASE79G_PRODUCER_NOINLINE
