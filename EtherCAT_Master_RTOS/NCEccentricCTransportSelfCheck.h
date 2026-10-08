#pragma once

#include "MotionEccentricCTransport.h"
#include "NCEccentricCProfileSelfCheck.h"
#include <limits>
#include <memory>
#include <new>

// BASE79B_FIX1: isolated packet diagnostics during startup / NC LOAD only.
// No MotionCore instance, queue, live axis, role write or execution admission.
struct NCEccentricCTransportSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

namespace NCEccentricCTransportSelfCheckDetail
{
    struct Workspace
    {
        MotionEccentricCTransportWorkspace transport{};
        NCEccentricCRuntimeInput input = NCEccentricCProfileSelfCheckDetail::Input();
        NCEccentricCRuntimeValue original{};
        NCEccentricCProfileValue decoded{};
        MotionCommand command{}, copy{}, bad{};
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
    };

    inline std::uint64_t Fingerprint(const MotionCommand& command) noexcept
    { return FoldMotionEccentricCTransportFingerprint(14695981039346656037ULL, command); }

    inline bool RejectedByNativeRoutes(const MotionCommand& c) noexcept
    {
        return !IsMotionFixedTranslationToolSourceAllowed(c) && !IsMotionFixedRotationPathAllowed(c) &&
            !IsMotionFixedTranslationWorkSourceAllowed(c) && !IsMotionFixedTranslationRotationSourceAllowed(c) &&
            !IsMotionFixedTranslationScaleMirrorSourceAllowed(c) && !IsMotionFixedTranslationPolarSourceAllowed(c) &&
            !IsMotionFixedTranslationCutterSourceAllowed(c) && !IsMotionArcPlaneGroupMapping(c, c.axisCount, c.axisIndices) &&
            !IsMotionBaseArcPlaneSourceAllowed(c) && !IsMotionRotaryFeedSourceAllowed(c) &&
            !IsMotionZCFeedSourceAllowed(c) && !IsMotionXYZCFeedSourceAllowed(c) &&
            !IsMotionXYZCUVFeedSourceAllowed(c) && !IsMotionFixedTranslationSourceAllowed(c);
    }
}

// Keep the private heap workspace out of the bounded caller stack, including
// optimized builds. Failure is diagnostic only while dynamic admission is shut.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
inline NCEccentricCTransportSelfCheckResult RunNCEccentricCTransportSelfCheck() noexcept
{
    using namespace NCEccentricCTransportSelfCheckDetail;
    NCEccentricCTransportSelfCheckResult result{};
    const auto check = [&](bool condition) noexcept -> bool
    {
        ++result.checks;
        if (!condition) { result.passed = false; result.failedCheck = result.checks; }
        return condition;
    };
    const std::unique_ptr<Workspace> workspace(new (std::nothrow) Workspace);
    if (!check(workspace && !NCEccentricCDynamicMotionAdmission)) return result;
    auto& w = *workspace;
    w.identity.epoch = 1U; w.identity.segmentId = 3ULL;
    w.identity.sourceBlockId = 17; w.identity.source = MotionCommandSource::NC_MEMORY;
    w.owner.owner = MotionOwner::AUTO; w.owner.generation = 1U;
    w.input.geometry.source.distanceMode = 91;

    // BASE79I: both distance modes must preserve their source metadata while
    // carrying the same resolved curve, H signs and C directions.
    for (int distance : {91, 90})
      for (int mode : {43, 44})
        for (double direction : {1.0, -1.0})
        {
            w.input.geometry.source.distanceMode = distance;
            w.input.geometry.source.toolLengthMode = mode;
            w.input.geometry.sweepDeg = direction * 0.01;
            if (!check(PrepareNCEccentricCRuntime(w.input, w.original) == NCEccentricCRuntimeCode::PREPARED &&
                BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.command, w.transport))) return result;
            if (!check(w.command.sourceIsAbsoluteMode == (distance == 90) &&
                w.command.axisCount == 3 && w.command.axisIndices[0] == 0 &&
                w.command.axisIndices[1] == 1 && w.command.axisIndices[2] == 3 &&
                RejectedByNativeRoutes(w.command))) return result;
            w.copy = w.command;
            if (!check(Fingerprint(w.copy) == Fingerprint(w.command) &&
                PrepareMotionEccentricCCommand(w.copy, w.decoded, w.transport))) return result;
            if (!check(NCEccentricCDetail::Same(w.original.StartPulse(), w.decoded.Runtime().StartPulse()) &&
                NCEccentricCDetail::Same(w.original.EndPulse(), w.decoded.Runtime().EndPulse()) &&
                SameNCTranslationSnapshot(w.input.geometry.source, w.decoded.Runtime().AuthoredPath().source))) return result;
            // References into the destination must survive builder clearing.
            if (!check(BuildMotionEccentricCCommand(w.input, w.copy.execution, w.copy.ownerLease, w.copy, w.transport) &&
                Fingerprint(w.copy) == Fingerprint(w.command))) return result;
        }

    w.input.geometry.source.distanceMode = 91;
    // No XY offset: compact transport contains only the configured C role.
    w.input.geometry.source.toolLengthMode = 43; w.input.geometry.sweepDeg = 0.01;
    w.input.geometry.source.toolOffsetMM[0] = w.input.geometry.source.toolOffsetMM[1] = 0.0;
    if (!check(BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.command, w.transport) &&
        w.command.axisCount == 1 && w.command.axisIndices[0] == 3 &&
        PrepareMotionEccentricCCommand(w.command, w.decoded, w.transport) &&
        w.decoded.Runtime().AuthoredPath().generatedMask == 0U)) return result;
    w.bad = w.command; w.bad.mem_transformMatrix[0][0] = std::numeric_limits<double>::quiet_NaN();
    if (!check(!PrepareMotionEccentricCCommand(w.bad, w.decoded, w.transport) && !w.decoded.IsValid())) return result;

    w.input.geometry.source.toolOffsetMM[0] = 2.0; w.input.geometry.source.toolOffsetMM[1] = 1.0;
    if (!check(BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.command, w.transport))) return result;
    const auto fingerprint = Fingerprint(w.command);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (unsigned defect = 0U; defect <  60U; ++defect)
    {
        w.bad = w.command;
        switch (defect)
        {
        case 0U: w.bad.pathCoreEccentricCFeedExactStop = false; break;
        case 1U: w.bad.execution.epoch = MOTION_EXECUTION_EPOCH_INVALID; break;
        case 2U: w.bad.execution.segmentId = MOTION_SEGMENT_ID_INVALID; break;
        case 3U: w.bad.execution.sourceBlockId = MOTION_SOURCE_BLOCK_ID_INVALID; break;
        case 4U: w.bad.execution.source = MotionCommandSource::NC_MDI; break;
        case 5U: ++w.bad.sourceLinePC; break;
        case 6U: w.bad.ownerLease.owner = MotionOwner::MDI; break;
        case 7U: w.bad.ownerLease.generation = MOTION_OWNER_GENERATION_INVALID; break;
        case 8U: w.bad.commandPathMode = MotionCommandPathMode::UNSPECIFIED; break;
        case 9U: w.bad.sourceIsAbsoluteMode = true; break;
        case 10U: w.bad.sourceG162Active = false; break;
        case 11U: w.bad.sourceTranslation.axisIdentity.eccentricEnabled = 0U; break;
        case 12U: w.bad.sourceTranslation.axisIdentity.electrodeAxisPlusOne = 0U; break;
        case 13U: w.bad.sourceTranslation.axisIdentity.systemMode = 0U; break;
        case 14U: w.bad.sourceTranslation.distanceMode = 90; break;
        case 15U: w.bad.sourceTranslation.unitsMode = 20; break;
        case 16U: w.bad.sourceTranslation.runToken = 0ULL; break;
        case 17U: ++w.bad.sourceWCS; break;
        case 18U: w.bad.sourceToolLengthMode = 49; break;
        case 19U: ++w.bad.sourceHCode; break;
        case 20U: w.bad.axisCount = 8; break;
        case 21U: w.bad.axisIndices[1] = 0; break;
        case 22U: w.bad.axisIndices[7] = 3; break;
        case 23U: w.bad.targetPos[7] = 1.0; break;
        case 24U: w.bad.targetPos[0] = std::nextafter(w.bad.targetPos[0], std::numeric_limits<double>::infinity()); break;
        case 25U: w.bad.mem_startPos[7] = nan; break;
        case 26U: w.bad.mem_ratio[7] = 0.0; break;
        case 27U: w.bad.mem_ratio[0] = nan; break;
        case 28U: w.bad.mem_radius = 0.0; break;
        case 29U: w.bad.mem_radius = nan; break;
        case 30U: w.bad.mem_startAngle = 0.0; break;
        case 31U: w.bad.accTime = 0.0; break;
        case 32U: w.bad.decTime = 0.0; break;
        case 33U: w.bad.mem_centerX = 0.0; break;
        case 34U: w.bad.mem_totalDist = w.bad.mem_centerY - 1.0; break;
        case 35U: w.bad.mem_transformOrigin[0] = nan; break;
        case 36U: w.bad.mem_transformOrigin[2] = -1.0; break;
        case 37U: w.bad.mem_transformMatrix[0][0] = 0.0; break;
        case 38U: w.bad.mem_transformMatrix[1][1] = nan; break;
        case 39U: w.bad.mem_transformMatrix[2][2] = -1.0; break;
        case 40U: w.bad.targetVel = std::nextafter(w.bad.targetVel, 0.0); break;
        case 41U: w.bad.mem_enableTransform = true; break;
        case 42U: w.bad.pathCoreFeedExactStop = true; break;
        case 43U: w.bad.pathCoreRotaryFeedExactStop = true; break;
        case 44U: w.bad.pathCoreZCFeedExactStop = true; break;
        case 45U: w.bad.pathCoreXYZCFeedExactStop = true; break;
        case 46U: w.bad.pathCoreXYZCUVFeedExactStop = true; break;
        case 47U: w.bad.cncFeedLookahead = true; break;
        case 48U: w.bad.cncCornerBlend = true; break;
        case 49U: w.bad.pathCoreRetainedTraversal = true; break;
        case 50U: w.bad.pathCoreRetainedReverse = true; break;
        case 51U: w.bad.replayTerminalAlreadyPublished = true; break;
        case 52U: w.bad.pathCorePlanarCircle = true; break;
        case 53U: w.bad.pathCoreFullCircle = true; break;
        case 54U: w.bad.sourcePlaneMode = 18; break;
        case 55U: w.bad.sourceG68Active = true; break;
        case 56U: w.bad.sourceG168Active = true; break;
        case 57U: w.bad.sourceG51Active = true; break;
        case 58U: w.bad.sourceG16Active = true; break;
        default: w.bad.sourceTranslation.toolOffsetMM[0] = 3.0; break;
        }
        // A failed decode must clear a previously valid output, without
        // modifying the intact packet or accepting a stale profile.
        if (!check(PrepareMotionEccentricCCommand(w.command, w.decoded, w.transport) &&
            !PrepareMotionEccentricCCommand(w.bad, w.decoded, w.transport) && !w.decoded.IsValid() &&
            Fingerprint(w.bad) != fingerprint && Fingerprint(w.command) == fingerprint)) return result;
    }

    for (unsigned defect = 0U; defect < 5U; ++defect)
    {
        w.input = NCEccentricCProfileSelfCheckDetail::Input();
        w.input.geometry.source.distanceMode = 91;
        switch (defect)
        {
        case 0U: w.input.firSamples = 2U; break;
        case 1U: w.input.cycleSeconds = 0.001; break;
        case 2U: w.input.geometry.source.axisIdentity.eccentricEnabled = 0U; break;
        case 3U: w.input.geometry.source.distanceMode = 92; break;
        default: w.input.geometry.feedDegMin = -1.0; break;
        }
        w.bad = w.command;
        if (!check(!BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.bad, w.transport) &&
            !w.bad.pathCoreEccentricCFeedExactStop && !w.bad.execution.IsAssigned() &&
            !w.bad.ownerLease.IsValid() && w.bad.axisCount == 0)) return result;
    }
    // BASE79K: both Z+C distance modes carry resolved deltas and preserve
    // pitch, compact order, stop envelope and conservative linear ceilings.
    for (int distance : {91, 90}) for (int mode : {43, 44}) for (double cDirection : {1.0, -1.0})
      for (double zDirection : {1.0, -1.0}) for (bool generated : {false, true})
    {
        w.input = NCEccentricCProfileSelfCheckDetail::Input();
        w.input.geometry.source.distanceMode = distance;
        w.input.geometry.source.toolLengthMode = mode;
        w.input.geometry.source.toolOffsetMM[0] = generated ? 1.0 : 0.0;
        w.input.geometry.source.toolOffsetMM[1] = 0.0;
        w.input.geometry.sweepDeg = cDirection * 8.0;
        w.input.geometry.zDeltaMM = zDirection * 0.1;
        w.input.geometry.feedDegMin = 48.0;
        w.input.geometry.maxVelocityNative[0] = 80.0;
        w.input.geometry.maxVelocityNative[1] = 60.0;
        w.input.geometry.maxVelocityNative[2] = 90.0;
        w.input.geometry.maxAccelerationNative[1] = 8000.0;
        w.input.geometry.maxDecelerationNative[2] = 7000.0;
        if (!check(CanonicalizeMotionEccentricCLinearLimits(w.input) &&
            w.input.geometry.maxVelocityNative[0] == 60.0 &&
            w.input.geometry.maxAccelerationNative[2] == 8000.0 &&
            w.input.geometry.maxDecelerationNative[1] == 7000.0 &&
            PrepareNCEccentricCRuntime(w.input, w.original) == NCEccentricCRuntimeCode::PREPARED &&
            BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.command, w.transport))) return result;
        const int zSlot = generated ? 2 : 0;
        if (!check(w.command.dir == 2 && w.command.sourceIsAbsoluteMode == (distance == 90) &&
            w.command.axisCount == (generated ? 4 : 2) &&
            w.command.axisIndices[zSlot] == 2 && w.command.axisIndices[zSlot + 1] == 3 &&
            (!generated || (w.command.axisIndices[0] == 0 && w.command.axisIndices[1] == 1)) &&
            w.original.AuthoredPath().authoredMask == 12U &&
            w.original.AuthoredPath().groupMask == (generated ? 15U : 12U) &&
            RejectedByNativeRoutes(w.command))) return result;
        w.copy = w.command;
        if (!check(Fingerprint(w.copy) == Fingerprint(w.command) &&
            PrepareMotionEccentricCCommand(w.copy, w.decoded, w.transport) &&
            NCEccentricCDetail::Same(w.original.EndPulse(), w.decoded.Runtime().EndPulse()) &&
            NCEccentricCDetail::Same(w.original.MinimumPulse(), w.decoded.Runtime().MinimumPulse()) &&
            NCEccentricCDetail::Same(w.original.MaximumPulse(), w.decoded.Runtime().MaximumPulse()) &&
            NCEccentricCDetail::Same(w.original.ExtendedPath().zDeltaMM,
                w.decoded.Runtime().ExtendedPath().zDeltaMM))) return result;
        if (!check(BuildMotionEccentricCCommand(w.input, w.copy.execution, w.copy.ownerLease, w.copy, w.transport) &&
            Fingerprint(w.copy) == Fingerprint(w.command))) return result;
        w.bad = w.command; w.bad.sourceIsAbsoluteMode = !w.bad.sourceIsAbsoluteMode;
        if (!check(!PrepareMotionEccentricCCommand(w.bad, w.decoded, w.transport) &&
            !w.decoded.IsValid() && Fingerprint(w.bad) != Fingerprint(w.command))) return result;
    }
    const auto zcFingerprint = Fingerprint(w.command);
    for (unsigned defect = 0U; defect < 16U; ++defect)
    {
        w.bad = w.command;
        switch (defect)
        {
        case 0U: w.bad.dir = 0; break;
        case 1U: w.bad.dir = 1; break;
        case 2U: w.bad.sourceTranslation.distanceMode = 91; break;
        case 3U: w.bad.mem_transformMatrix[2][0] = 0.0; break;
        case 4U: w.bad.mem_transformMatrix[2][0] = nan; break;
        case 5U: w.bad.mem_transformMatrix[2][1] = nan; break;
        case 6U: w.bad.mem_transformMatrix[2][2] = nan; break;
        case 7U: w.bad.mem_transformMatrix[2][1] = w.bad.mem_transformMatrix[2][2] + 1.0; break;
        case 8U: w.bad.mem_transformMatrix[0][0] = 0.0; break;
        case 9U: w.bad.mem_transformMatrix[1][1] = nan; break;
        case 10U: w.bad.axisIndices[2] = 3; break;
        case 11U: w.bad.axisCount = 3; break;
        case 12U: w.bad.targetPos[2] = std::nextafter(w.bad.targetPos[2], 0.0); break;
        case 13U: w.bad.mem_transformMatrix[2][0] *= -1.0; break;
        case 14U: w.bad.mem_transformMatrix[2][1] = w.original.AuthoredPath().endMCS[2]; break;
        default: w.bad.mem_transformMatrix[0][2] = -1.0; break;
        }
        if (!check(PrepareMotionEccentricCCommand(w.command, w.decoded, w.transport) &&
            !PrepareMotionEccentricCCommand(w.bad, w.decoded, w.transport) && !w.decoded.IsValid() &&
            Fingerprint(w.bad) != zcFingerprint && Fingerprint(w.command) == zcFingerprint)) return result;
    }
    w.input.geometry.maxVelocityNative[2] += 1.0;
    if (!check(!BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.bad, w.transport) &&
        !w.bad.pathCoreEccentricCFeedExactStop)) return result;
    if (!check(CanonicalizeMotionEccentricCLinearLimits(w.input))) return result;
    w.input.geometry.source.distanceMode = 92;
    if (!check(!BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.bad, w.transport) &&
        !w.bad.pathCoreEccentricCFeedExactStop)) return result;
    // BASE79M: both modes carry resolved sparse XYZ independently of generated XY.
    // All six masks include X or Y; held Z still has transported limits.
    for (int distance : {91, 90})
      for (unsigned mask : {1U, 2U, 3U, 5U, 6U, 7U}) for (bool generated : {false, true})
    {
        w.input = NCEccentricCProfileSelfCheckDetail::Input();
        w.input.geometry.source.distanceMode = distance;
        w.input.geometry.source.toolLengthMode = (mask & 1U) != 0U ? 43 : 44;
        w.input.geometry.source.toolOffsetMM[0] = generated ? 1.0 : 0.0;
        w.input.geometry.source.toolOffsetMM[1] = 0.0;
        w.input.geometry.xDeltaMM = (mask & 1U) != 0U ? 0.06 : 0.0;
        w.input.geometry.yDeltaMM = (mask & 2U) != 0U ? -0.08 : 0.0;
        w.input.geometry.zDeltaMM = (mask & 4U) != 0U ? 0.04 : 0.0;
        w.input.geometry.sweepDeg = generated ? -8.0 : 8.0;
        w.input.geometry.feedDegMin = 48.0;
        w.input.geometry.maxVelocityNative[1] = 60.0;
        w.input.geometry.maxAccelerationNative[2] = 8000.0;
        w.input.geometry.maxDecelerationNative[0] = 7000.0;
        if (!check(CanonicalizeMotionEccentricCLinearLimits(w.input) &&
            PrepareNCEccentricCRuntime(w.input, w.original) == NCEccentricCRuntimeCode::PREPARED &&
            BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.command, w.transport))) return result;
        const unsigned group = 8U | mask | (generated ? 3U : 0U);
        if (!check(w.command.dir == 3 && w.command.sourceIsAbsoluteMode == (distance == 90) &&
            w.command.axisCount == MotionEccentricCTransportDetail::AxisCount(group) &&
            w.original.AuthoredPath().authoredMask == (8U | mask) &&
            w.original.AuthoredPath().groupMask == group && RejectedByNativeRoutes(w.command))) return result;
        w.copy = w.command;
        if (!check(Fingerprint(w.copy) == Fingerprint(w.command) &&
            PrepareMotionEccentricCCommand(w.copy, w.decoded, w.transport) &&
            NCEccentricCDetail::Same(w.original.EndPulse(), w.decoded.Runtime().EndPulse()) &&
            NCEccentricCDetail::Same(w.original.MinimumPulse(), w.decoded.Runtime().MinimumPulse()) &&
            NCEccentricCDetail::Same(w.original.MaximumPulse(), w.decoded.Runtime().MaximumPulse()) &&
            NCEccentricCDetail::Same(w.original.ExtendedPath().xDeltaMM, w.decoded.Runtime().ExtendedPath().xDeltaMM) &&
            NCEccentricCDetail::Same(w.original.ExtendedPath().yDeltaMM, w.decoded.Runtime().ExtendedPath().yDeltaMM) &&
            NCEccentricCDetail::Same(w.original.ExtendedPath().zDeltaMM, w.decoded.Runtime().ExtendedPath().zDeltaMM))) return result;
        if (!check(BuildMotionEccentricCCommand(w.input, w.copy.execution, w.copy.ownerLease, w.copy, w.transport) &&
            Fingerprint(w.copy) == Fingerprint(w.command))) return result;
        w.bad = w.command; w.bad.sourceTranslation.distanceMode = distance == 90 ? 91 : 90;
        if (!check(!PrepareMotionEccentricCCommand(w.bad, w.decoded, w.transport) && !w.decoded.IsValid())) return result;
    }
    const auto xyzcFingerprint = Fingerprint(w.command);
    for (unsigned defect = 0U; defect < 24U; ++defect)
    {
        w.bad = w.command;
        switch (defect)
        {
        case 0U: w.bad.dir = 0; break;
        case 1U: w.bad.dir = 2; break;
        case 2U: w.bad.dir = 4; break;
        case 3U: w.bad.centerPos[0] = nan; break;
        case 4U: w.bad.centerPos[1] = nan; break;
        case 5U: w.bad.centerPos[0] = w.bad.centerPos[1] = 0.0; break;
        case 6U: w.bad.centerPos[0] *= -1.0; break;
        case 7U: w.bad.centerPos[1] *= -1.0; break;
        case 8U: w.bad.mem_transformMatrix[2][0] *= -1.0; break;
        case 9U: w.bad.mem_transformMatrix[2][1] = nan; break;
        case 10U: w.bad.mem_transformMatrix[2][2] = nan; break;
        case 11U: w.bad.mem_transformMatrix[2][1] = w.bad.mem_transformMatrix[2][2] + 1.0; break;
        case 12U: w.bad.axisCount = 3; break;
        case 13U: w.bad.axisIndices[1] = 0; break;
        case 14U: w.bad.targetPos[0] = std::nextafter(w.bad.targetPos[0], 0.0); break;
        case 15U: w.bad.targetPos[1] = std::nextafter(w.bad.targetPos[1], 0.0); break;
        case 16U: w.bad.mem_transformMatrix[0][0] = 0.0; break;
        case 17U: w.bad.mem_transformMatrix[0][1] = nan; break;
        case 18U: w.bad.mem_transformMatrix[0][2] = -1.0; break;
        case 19U: w.bad.mem_transformMatrix[1][0] = 0.0; break;
        case 20U: w.bad.axisIndices[7] = 1; break;
        case 21U: w.bad.targetPos[7] = 1.0; break;
        case 22U: w.bad.startRadius = 1.0; break;
        default: w.bad.cncPrefixVelocityPPS = 1.0; break;
        }
        if (!check(PrepareMotionEccentricCCommand(w.command, w.decoded, w.transport) &&
            !PrepareMotionEccentricCCommand(w.bad, w.decoded, w.transport) && !w.decoded.IsValid() &&
            Fingerprint(w.bad) != xyzcFingerprint && Fingerprint(w.command) == xyzcFingerprint)) return result;
    }
    w.input.geometry.maxVelocityNative[2] += 1.0;
    if (!check(!BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.bad, w.transport) &&
        !w.bad.pathCoreEccentricCFeedExactStop)) return result;
    if (!check(CanonicalizeMotionEccentricCLinearLimits(w.input))) return result;
    w.input.geometry.source.distanceMode = 92;
    if (!check(!BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.bad, w.transport) &&
        !w.bad.pathCoreEccentricCFeedExactStop)) return result;
    return result;
}
