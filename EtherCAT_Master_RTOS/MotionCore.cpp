#include "MotionCore.h"
#include <algorithm> // for std::abs, std::sqrt, std::max, std::min
#include <cstring>
#include <iostream>  // for debug prints if needed
#include <limits>
#include "EtherCatMaster.h"
#include "CoordinateManager.h"
#include "SHM_Types.h"
#include "AlarmManager.h"
#include "NCPathCoreFeedArc.h" // BY fixed planar circle consumer geometry
#include "NCTranslationArcPrecision.h"
#include "MotionRetainedInterval.h" // CA canonical interval transport and evaluation

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

    constexpr std::uint64_t MOTION_OWNER_VALUE_MASK =
        0x000000000000000FULL;
    constexpr std::uint64_t MOTION_OWNER_SAFETY_HANDSHAKE =
        0x0000000000000010ULL;
    constexpr std::uint64_t MOTION_OWNER_OUTPUT_COMMIT_RESERVED =
        0x0000000000000020ULL;
    constexpr std::uint64_t MOTION_OWNER_SAFETY_ACTION_PENDING =
        0x0000000000000040ULL;
    constexpr std::uint64_t MOTION_OWNER_FRAME_SEND_RESERVED =
        0x0000000000000080ULL;
    constexpr std::uint64_t MOTION_OWNER_EPOCH_COMMIT_RESERVED =
        0x0000000000000100ULL;
    // Historical name retained locally to minimize churn.  This mask now
    // covers every packed-owner reservation that must block owner transfer,
    // output publication, and Safety takeover.
    constexpr std::uint64_t MOTION_OWNER_ANY_OUTPUT_RESERVATION =
        MOTION_OWNER_OUTPUT_COMMIT_RESERVED |
        MOTION_OWNER_FRAME_SEND_RESERVED |
        MOTION_OWNER_EPOCH_COMMIT_RESERVED;
    constexpr unsigned MOTION_OWNER_SAFETY_TICKET_SHIFT = 9U;
    constexpr std::uint64_t MOTION_OWNER_SAFETY_TICKET_MASK =
        0x00000000FFFFFE00ULL;
    constexpr std::uint32_t MOTION_OWNER_SAFETY_TICKET_MAX =
        0x007FFFFFU;
    constexpr unsigned MOTION_OWNER_BOUNDED_CAS_ATTEMPTS = 32U;
    constexpr std::uint64_t FRAME_SAFETY_INTENT_BEGIN_DELTA =
        0x0000000100000001ULL;
    constexpr std::uint8_t RESET_RELEASE_AUTH_EMPTY = 0U;
    constexpr std::uint8_t RESET_RELEASE_AUTH_AVAILABLE = 1U;
    constexpr std::uint8_t RESET_RELEASE_AUTH_CLAIMED = 2U;
    constexpr std::uint8_t RESET_RELEASE_AUTH_CONSUMED = 3U;

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
    constexpr unsigned RESET_SAFETY_BATCH_TICKET_SHIFT = 33U;
    constexpr std::uint64_t RESET_SAFETY_BATCH_TICKET_MASK =
        0x00FFFFFE00000000ULL;
    constexpr std::uint64_t RESET_SAFETY_BATCH_PUBLISH_RESERVED =
        0x4000000000000000ULL;
    constexpr std::uint64_t RESET_SAFETY_BATCH_PRESENT =
        0x8000000000000000ULL;

    constexpr std::uint64_t EMERGENCY_STOP_REQUEST_EPOCH_MASK =
        0x00000000FFFFFFFFULL;
    constexpr std::uint64_t EMERGENCY_STOP_REQUEST_EVIDENCE_REQUIRED =
        0x0000000100000000ULL;
    constexpr std::uint64_t EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED =
        0x4000000000000000ULL;
    constexpr std::uint64_t EMERGENCY_STOP_REQUEST_PENDING =
        0x8000000000000000ULL;

    std::uint64_t PackEmergencyStopRequest(
        MotionExecutionEpoch causalEpoch,
        bool evidenceRequired) noexcept
    {
        return
            static_cast<std::uint64_t>(causalEpoch) |
            (evidenceRequired
                ? EMERGENCY_STOP_REQUEST_EVIDENCE_REQUIRED
                : 0ULL) |
            EMERGENCY_STOP_REQUEST_PENDING;
    }

    MotionExecutionEpoch UnpackEmergencyStopRequestEpoch(
        std::uint64_t request) noexcept
    {
        return static_cast<MotionExecutionEpoch>(
            request & EMERGENCY_STOP_REQUEST_EPOCH_MASK);
    }

    bool UnpackEmergencyStopRequestEvidenceRequired(
        std::uint64_t request) noexcept
    {
        return
            (request &
                EMERGENCY_STOP_REQUEST_EVIDENCE_REQUIRED) != 0ULL;
    }

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
        bool requestResetAllFaults,
        std::uint32_t safetyRequestTicket,
        bool reserved) noexcept
    {
        return
            RESET_SAFETY_BATCH_PRESENT |
            static_cast<std::uint64_t>(publishedEpoch) |
            ((static_cast<std::uint64_t>(safetyRequestTicket) <<
                RESET_SAFETY_BATCH_TICKET_SHIFT) &
                RESET_SAFETY_BATCH_TICKET_MASK) |
            (requestResetAllFaults
                ? RESET_SAFETY_BATCH_RESET_FAULTS
                : 0ULL) |
            (reserved
                ? RESET_SAFETY_BATCH_PUBLISH_RESERVED
                : 0ULL);
    }

    MotionExecutionEpoch UnpackResetSafetyBatchEpoch(
        std::uint64_t packed) noexcept
    {
        return static_cast<MotionExecutionEpoch>(
            packed & RESET_SAFETY_BATCH_EPOCH_MASK);
    }

    std::uint32_t UnpackResetSafetyBatchTicket(
        std::uint64_t packed) noexcept
    {
        return static_cast<std::uint32_t>(
            (packed & RESET_SAFETY_BATCH_TICKET_MASK) >>
            RESET_SAFETY_BATCH_TICKET_SHIFT);
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
        if (!SameNCTranslationSnapshot(lhs.sourceTranslation, rhs.sourceTranslation) ||
            lhsAxisCount == 0 ||
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

    // Source-derived arithmetic tolerance is recomputed from the immutable
    // packet; no new packet bytes, live coordinate tables or execution authority.
    bool TryGetMotionArcSourceRoundoffPulse(const MotionCommand& command,
        const std::vector<AxisContext>* contexts, double& outputPulse, double& outputMM) noexcept
    {
        outputPulse = outputMM = 0.0;
        if (IsNCTranslationSnapshotEmpty(command.sourceTranslation)) return true;
        float sourceRoundoffMM = 0.0F;
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(command.sourcePlaneMode, plane)) return false;
        const unsigned planeSlots[2] = { plane.u, plane.v };
        if (!IsMotionFixedTranslationSourceAllowed(command) || contexts == nullptr ||
            contexts->size() <= plane.u || contexts->size() <= plane.v || contexts->size() > MAX_AXES ||
            !TryGetNCTranslationArcRoundoffMM(command.sourceTranslation, sourceRoundoffMM, command.pathCoreFullCircle)) return false;
        double pulsePerMM = 0.0;
        for (std::size_t axisIndex = 0U; axisIndex < 2U; ++axisIndex)
        {
            const AxisContext& axis = (*contexts)[planeSlots[axisIndex]];
            if (!axis.isExist || axis.axisType != AxisType::LINEAR ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0) return false;
            const double scale = axis.resolution_PPR / axis.finalLead;
            if (!std::isfinite(scale) || scale <= 0.0 ||
                (axisIndex != 0U && scale != pulsePerMM)) return false;
            pulsePerMM = scale;
        }
        const double value = static_cast<double>(sourceRoundoffMM) * pulsePerMM;
        if (!std::isfinite(value) || value < 0.0 ||
            (sourceRoundoffMM > 0.0F && value == 0.0)) return false;
        outputPulse = value;
        outputMM = static_cast<double>(sourceRoundoffMM);
        return true;
    }

    bool IsMotionArcPulsePrecisionWithinBudget(const std::vector<AxisContext>* contexts,
        double sx, double sy, double ex, double ey, double cx, double cy,
        double radius, double sourceRoundoffMM, int activePlane = 17) noexcept
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(activePlane, plane) || contexts == nullptr ||
            contexts->size() <= plane.u || contexts->size() <= plane.v) return false;
        const AxisContext& axis = (*contexts)[plane.u];
        const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
        if (!std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
            !std::isfinite(pulsePerMM) || pulsePerMM <= 0.0 ||
            !std::isfinite(sourceRoundoffMM) || sourceRoundoffMM < 0.0) return false;
        double magnitude = 0.0;
        const double values[] = { sx, sy, ex, ey, cx, cy, radius };
        for (double value : values)
        {
            if (!std::isfinite(value)) return false;
            magnitude = (std::max)(magnitude, std::fabs(value));
        }
        const double nativeBudgetMM =
            (64.0 * std::numeric_limits<double>::epsilon() * magnitude) / pulsePerMM;
        const double combinedMM = 2.0 * (nativeBudgetMM + sourceRoundoffMM);
        return std::isfinite(combinedMM) && combinedMM <= 1.0e-7;
    }

    // BZ uses the two remaining dir-padding flags and existing mem_* bytes.
    // These values are canonical path data, never a legacy history snapshot.
    bool IsMotionCommandRetainedGeometryValid(const MotionCommand& command,
        const std::vector<AxisContext>* contexts) noexcept
    {
        if (!command.pathCoreRetainedTraversal)
            return !command.pathCoreRetainedReverse;
        if (contexts == nullptr || contexts->empty() || contexts->size() > MAX_AXES ||
            command.axisCount <= 0 || command.axisCount > MAX_AXES)
            return false;
        if (command.replayTerminalAlreadyPublished || command.mem_enableTransform ||
            command.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
            !IsMotionFixedTranslationSourceAllowed(command) || !IsMotionFixedTranslationToolSourceAllowed(command) ||
            command.sourceToolRadiusMode != 40 || !command.sourceIsAbsoluteMode ||
            !IsMotionFixedTranslationRotationSourceAllowed(command) || !IsMotionFixedTranslationWorkSourceAllowed(command) || command.sourceG51Active ||
            command.sourceMirrorMask != 0U || command.sourceG16Active || command.sourceG162Active ||
            command.sourcePlaneMode != 17 || command.targetVel < 0.0 ||
            command.accTime <= 0.0 || command.decTime <= 0.0 ||
            !std::isfinite(command.mem_totalDist) || command.mem_totalDist < 0.0)
            return false;
        for (int axis = 0; axis < MAX_AXES; ++axis)
        {
            if (!std::isfinite(command.mem_startPos[axis]) || !std::isfinite(command.mem_ratio[axis]))
                return false;
        }
        double startU = 0.0, endU = 0.0, intervalDistance = 0.0;
        if (!GetMotionRetainedInterval(command, startU, endU, intervalDistance))
            return false;
        double length = 0.0;
        bool pulseChanges = false;
        for (int slot = 0; slot < command.axisCount; ++slot)
        {
            const int axis = command.axisIndices[slot];
            if (axis < 0 || axis > 2 || axis >= static_cast<int>(contexts->size()) ||
                (*contexts)[axis].axisType != AxisType::LINEAR ||
                !std::isfinite((*contexts)[axis].maxVel_PPS) || (*contexts)[axis].maxVel_PPS <= 0.0)
                return false;
            double expectedStart = 0.0, expectedTarget = 0.0;
            if (!EvaluateMotionRetainedPulseCanonical(command, static_cast<std::size_t>(axis), startU, expectedStart) ||
                !EvaluateMotionRetainedPulseCanonical(command, static_cast<std::size_t>(axis), endU, expectedTarget) ||
                command.targetPos[slot] != expectedTarget)
                return false;
            pulseChanges = pulseChanges || expectedStart != expectedTarget;
            const double delta = command.mem_ratio[axis] - command.mem_startPos[axis];
            if (!std::isfinite(delta)) return false;
            length = std::hypot(length, delta);
            const double projection = command.pathCorePlanarCircle ? command.targetVel :
                command.mem_totalDist == 0.0 ? 0.0 : std::abs(command.targetVel * (delta / command.mem_totalDist));
            if (!std::isfinite(projection) || projection > (*contexts)[axis].maxVel_PPS ||
                (!command.pathCorePlanarCircle && delta != 0.0 &&
                    (delta / command.mem_totalDist == 0.0 || projection == 0.0)))
                return false;
        }
        if (command.mem_totalDist > 0.0 &&
            !(command.pathCoreFullCircle && std::abs(endU - startU) == 1.0) && !pulseChanges)
            return false;
        for (int axis = 0; axis < MAX_AXES; ++axis)
        {
            bool selected = false;
            for (int slot = 0; slot < command.axisCount; ++slot)
                selected = selected || command.axisIndices[slot] == axis;
            if (!selected && command.mem_startPos[axis] != command.mem_ratio[axis])
                return false;
        }
        if (command.mode == InterpolationMode::LINEAR)
        {
            const double error = std::abs(length - command.mem_totalDist);
            return !command.pathCorePlanarCircle && !command.pathCoreFullCircle &&
                std::isfinite(length) && error <= 64.0 * std::numeric_limits<double>::epsilon() *
                (std::max)(1.0, length) &&
                ((length == 0.0 && command.mem_totalDist == 0.0 && command.targetVel == 0.0) ||
                    (length > 0.0 && command.mem_totalDist > 0.0 && command.targetVel >= 1.0));
        }
        double sourceRoundoffPulse = 0.0, sourceRoundoffMM = 0.0;
        if (!TryGetMotionArcSourceRoundoffPulse(command, contexts, sourceRoundoffPulse, sourceRoundoffMM) ||
            !IsMotionArcPulsePrecisionWithinBudget(contexts,
                command.mem_startPos[0], command.mem_startPos[1], command.mem_ratio[0], command.mem_ratio[1],
                command.mem_centerX, command.mem_centerY, command.mem_radius, sourceRoundoffMM)) return false;
        // Validate the queued canonical circle without deriving a replacement
        // radius, start angle or sweep. A corrupted angle cannot authorize an
        // interior jump merely because its target endpoint still looks valid.
        for (int axis = 0; axis < 2; ++axis)
        {
            if (axis >= static_cast<int>(contexts->size())) return false;
            const AxisContext& context = (*contexts)[axis];
            const double ppm = context.resolution_PPR / context.finalLead;
            if (!std::isfinite(context.resolution_PPR) || context.resolution_PPR <= 0.0 ||
                !std::isfinite(context.finalLead) || context.finalLead <= 0.0 ||
                !std::isfinite(ppm) || ppm <= 0.0) return false;
            const double center = axis == 0 ? command.mem_centerX : command.mem_centerY;
            const double scale = (std::max)(1.0, (std::max)(std::abs(center),
                (std::max)(std::abs(command.mem_radius), (std::max)(std::abs(command.mem_startPos[axis]), std::abs(command.mem_ratio[axis])))));
            for (int end = 0; end < 2; ++end)
            {
                const double angle = command.mem_startAngle + (end == 0 ? 0.0 : command.mem_totalAngle);
                const double evaluated = center + command.mem_radius * (axis == 0 ? std::cos(angle) : std::sin(angle));
                const double expected = end == 0 ? command.mem_startPos[axis] : command.mem_ratio[axis];
                const double error = std::abs(evaluated - expected);
                if (!std::isfinite(evaluated) ||
                    error > 64.0 * std::numeric_limits<double>::epsilon() * scale + sourceRoundoffPulse || error / ppm > 5e-8)
                    return false;
            }
        }
        const double originalDirection = command.pathCoreRetainedReverse ? -command.dir : command.dir;
        const double twoPi = 2.0 * std::acos(-1.0);
        return command.pathCorePlanarCircle && command.targetVel >= 1.0 &&
            std::isfinite(command.mem_radius) && command.mem_radius > 0.0 &&
            command.mem_radius == command.startRadius &&
            std::isfinite(command.mem_centerX) && std::isfinite(command.mem_centerY) &&
            command.mem_centerX == command.centerPos[0] && command.mem_centerY == command.centerPos[1] &&
            std::isfinite(command.mem_startAngle) && std::abs(command.mem_startAngle) <= twoPi &&
            std::isfinite(command.mem_totalAngle) && command.mem_totalAngle * originalDirection > 0.0 &&
            std::abs(command.mem_totalAngle) <= twoPi && command.mem_totalDist > 0.0 &&
            std::isfinite(command.mem_radius * std::abs(command.mem_totalAngle)) &&
            command.mem_totalDist == command.mem_radius * std::abs(command.mem_totalAngle) &&
            (command.pathCoreFullCircle ? (std::abs(command.mem_totalAngle) == twoPi &&
                command.mem_startPos[0] == command.mem_ratio[0] && command.mem_startPos[1] == command.mem_ratio[1])
                : std::abs(command.mem_totalAngle) < twoPi);
    }

    bool DoesMotionRetainedStartMatch(const MotionCommand& command,
        const std::vector<AxisContext>* contexts) noexcept
    {
        if (contexts == nullptr || contexts->empty() || contexts->size() > MAX_AXES)
            return false;
        double startU = 0.0, endU = 0.0, intervalDistance = 0.0;
        if (!GetMotionRetainedInterval(command, startU, endU, intervalDistance)) return false;
        for (std::size_t axis = 0U; axis < contexts->size(); ++axis)
        {
            const AxisContext& context = (*contexts)[axis];
            if (!context.isExist) continue;
            const double actual = context.logicalCmdPos;
            double expected = 0.0;
            if (!EvaluateMotionRetainedPulseCanonical(command, axis, startU, expected)) return false;
            double scale = (std::max)(1.0, (std::max)(std::abs(command.mem_startPos[axis]), std::abs(command.mem_ratio[axis])));
            if (command.pathCorePlanarCircle && axis < 2U)
                scale = (std::max)(scale, (std::max)(std::abs(command.centerPos[axis]), command.mem_radius));
            const double error = std::abs(actual - expected);
            const double ppm = context.resolution_PPR / context.finalLead;
            if (!std::isfinite(actual) || !std::isfinite(context.resolution_PPR) || context.resolution_PPR <= 0.0 ||
                !std::isfinite(context.finalLead) || context.finalLead <= 0.0 ||
                !std::isfinite(ppm) || ppm <= 0.0 ||
                error > 64.0 * std::numeric_limits<double>::epsilon() * scale || error / ppm > 1e-7)
                return false;
        }
        return true;
    }

    // DG: the existing scalar planner uses a bounded planar circle.  Split
    // its feed-derived acceleration budget between tangential and centripetal
    // components.  No physical axis limit or configured timing is rewritten.
    bool ComputeCncPathDynamics(const MotionCommand& command,
        double& speed, double& acceleration, double& deceleration) noexcept
    {
        speed = command.targetVel;
        acceleration = command.accTime < 0.0001 ? 1.0e10 : speed / command.accTime;
        const double t = command.decTime < 0.0 ? command.accTime : command.decTime;
        deceleration = t < 0.0001 ? 1.0e10 : speed / t;
        if (!std::isfinite(speed) || speed < 1.0 ||
            !std::isfinite(acceleration) || acceleration <= 0.0 ||
            !std::isfinite(deceleration) || deceleration <= 0.0) return false;
        if (command.pathCorePlanarCircle || command.cncCornerBlend)
        {
            if (!std::isfinite(command.startRadius) || command.startRadius <= 0.0) return false;
            acceleration *= 0.5;
            deceleration *= 0.5;
            const double cap = std::sqrt((std::min)(acceleration, deceleration)) * std::sqrt(command.startRadius);
            speed = (std::min)(speed, cap);
        }
        return std::isfinite(speed) && speed >= 1.0 && acceleration > 0.0 && deceleration > 0.0;
    }

    bool ResolveCncCornerBlend(const MotionCommand& c, double& prefix) noexcept
    {
        prefix = 0.0;
        if (!c.cncCornerBlend || !c.cncFeedLookahead || c.mode != InterpolationMode::LINEAR ||
            c.pathCorePlanarCircle || c.pathCoreFullCircle || c.pathCoreRetainedTraversal ||
            c.pathCoreRetainedReverse || c.mem_enableTransform || c.axisCount != 2 ||
            c.axisIndices[0] != 0 || c.axisIndices[1] != 1 || (c.dir != -1 && c.dir != 1) ||
            !std::isfinite(c.mem_radius) || c.mem_radius <= 0.0 || c.startRadius != c.mem_radius || c.endRadius != c.mem_radius ||
            !std::isfinite(c.mem_startAngle) || !std::isfinite(c.mem_totalAngle) ||
            std::fabs(c.mem_totalAngle) < 0.0872664625997164 || std::fabs(c.mem_totalAngle) > 2.356194490192345 ||
            (c.dir == 1) != (c.mem_totalAngle > 0.0) || !std::isfinite(c.mem_totalDist) || c.mem_totalDist <= 0.0 ||
            c.mem_centerX != c.centerPos[0] || c.mem_centerY != c.centerPos[1]) return false;
        prefix = std::hypot(c.mem_ratio[0] - c.mem_startPos[0], c.mem_ratio[1] - c.mem_startPos[1]);
        if (!std::isfinite(prefix) || prefix <= 0.0) return false;
        const double total = prefix + c.mem_radius * std::fabs(c.mem_totalAngle);
        if (std::fabs(total - c.mem_totalDist) > 128.0 * std::numeric_limits<double>::epsilon() * (std::max)(1.0, total)) return false;
        // DJ_FIX1: the shared radius/angle carry roundoff from both XY axes.
        // Match the planar-circle resolver's two-coordinate scale instead of
        // letting a near-zero endpoint erase the other axis's subtraction error.
        // The 64-epsilon factor, exact stored entry, tangent, mapping, identity
        // and source-seam checks remain unchanged; no packet value is modified.
        double reconstructionScale = (std::max)(1.0, c.mem_radius);
        for (unsigned i = 0U; i < 2U; ++i)
        {
            if (!std::isfinite(c.centerPos[i]) || !std::isfinite(c.targetPos[i]) ||
                !std::isfinite(c.mem_ratio[i])) return false;
            reconstructionScale = (std::max)(reconstructionScale,
                (std::max)(std::fabs(c.mem_ratio[i]),
                    (std::max)(std::fabs(c.centerPos[i]), std::fabs(c.targetPos[i]))));
        }
        for (unsigned i = 0U; i < 2U; ++i)
        {
            const double angle = c.mem_startAngle + c.mem_totalAngle;
            const double tangent = i == 0U ? -double(c.dir) * std::sin(c.mem_startAngle) : double(c.dir) * std::cos(c.mem_startAngle);
            const double expected = c.centerPos[i] + c.mem_radius * (i == 0U ? std::cos(angle) : std::sin(angle));
            const double entry = c.centerPos[i] + c.mem_radius * (i == 0U ? std::cos(c.mem_startAngle) : std::sin(c.mem_startAngle));
            if (!std::isfinite(c.mem_startPos[i]) || !std::isfinite(c.mem_ratio[i]) || !std::isfinite(expected) ||
                !std::isfinite(c.targetPos[i]) || std::fabs(expected - c.targetPos[i]) > 64.0 * std::numeric_limits<double>::epsilon() * reconstructionScale ||
                entry != c.mem_ratio[i] || std::fabs((entry - c.mem_startPos[i]) / prefix - tangent) > 1e-11) return false;
        }
        return true;
    }

    bool ResolveCncPlanarCircle(const MotionCommand& command,
        const std::vector<AxisContext>* contexts, NCPathCoreArcPulseGeometry& circle) noexcept
    {
        double sourceRoundoffPulse = 0.0, sourceRoundoffMM = 0.0;
        if (!TryGetMotionArcSourceRoundoffPulse(command, contexts, sourceRoundoffPulse, sourceRoundoffMM) ||
            !IsMotionArcPulsePrecisionWithinBudget(contexts,
                command.mem_startPos[0], command.mem_startPos[1], command.targetPos[0], command.targetPos[1],
                command.centerPos[0], command.centerPos[1], command.startRadius, sourceRoundoffMM)) return false;
        // ED: full-circle closure is immutable packet geometry, including signed zero.
        if (command.pathCoreFullCircle &&
            (std::memcmp(&command.mem_startPos[0], &command.targetPos[0], sizeof(double)) != 0 ||
                std::memcmp(&command.mem_startPos[1], &command.targetPos[1], sizeof(double)) != 0)) return false;
        if (!command.cncFeedLookahead || !command.pathCorePlanarCircle ||
            command.pathCoreRetainedTraversal || command.axisCount != 2 ||
            command.axisIndices[0] != 0 || command.axisIndices[1] != 1 ||
            !((command.mode == InterpolationMode::CIRCULAR_CCW && command.dir == 1) ||
                (command.mode == InterpolationMode::CIRCULAR_CW && command.dir == -1)) ||
            command.startRadius != command.endRadius || command.mem_radius != command.startRadius ||
            !ResolveNCPathCorePlanarCirclePulse(command.mem_startPos[0], command.mem_startPos[1],
                command.targetPos[0], command.targetPos[1], command.centerPos[0], command.centerPos[1],
                command.startRadius, command.dir, command.pathCoreFullCircle, circle, sourceRoundoffPulse)) return false;
        return command.mem_startAngle == circle.startAngle && command.mem_totalAngle == circle.sweepRadians &&
            command.mem_totalDist == circle.lengthPulse;
    }

    bool ResolveCutterPlanarCircle(const MotionCommand& command,
        const std::vector<AxisContext>* contexts, NCPathCoreArcPulseGeometry& circle) noexcept
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(command.sourcePlaneMode, plane)) return false;
        // mem_startPos is PHYSICAL XYZ; target/centre arrays are packet u/v.
        // Validate the exact same metric and circle used by the producer.
        double sourceRoundoffPulse = 0.0, sourceRoundoffMM = 0.0;
        if (!TryGetMotionArcSourceRoundoffPulse(command, contexts, sourceRoundoffPulse, sourceRoundoffMM) ||
            !IsMotionArcPulsePrecisionWithinBudget(contexts,
                command.mem_startPos[plane.u], command.mem_startPos[plane.v], command.targetPos[0], command.targetPos[1],
                command.centerPos[0], command.centerPos[1], command.startRadius, sourceRoundoffMM,
                command.sourcePlaneMode)) return false;
        // BASE-PLANE-24: G17 permits NC-proved G90/G16 seam circles,
        // not Cartesian full circles or an arbitrary same-endpoint packet.
        // Recheck the shared source scope AND bit-exact physical closure here
        // before the consumer/LoadNextCommand accepts a complete revolution.
        if (command.pathCoreFullCircle &&
            (!IsNCTranslationCutterArcNotationAllowed(command.sourcePlaneMode,
                command.sourceTranslation.distanceMode, command.sourceTranslation.polarMode, true) ||
                std::memcmp(&command.mem_startPos[plane.u], &command.targetPos[0], sizeof(double)) != 0 ||
                std::memcmp(&command.mem_startPos[plane.v], &command.targetPos[1], sizeof(double)) != 0)) return false;
        if (command.sourceTranslation.cutterMode == 40 ||
            !IsMotionFixedTranslationCutterSourceAllowed(command) || !command.pathCorePlanarCircle ||
            command.startRadius != command.endRadius || command.mem_radius != command.startRadius ||
            !std::isfinite(command.mem_startPos[plane.normal]) ||
            !ResolveNCPathCorePlanarCirclePulse(command.mem_startPos[plane.u], command.mem_startPos[plane.v],
                command.targetPos[0], command.targetPos[1], command.centerPos[0], command.centerPos[1],
                command.startRadius, command.dir, command.pathCoreFullCircle, circle, sourceRoundoffPulse)) return false;
        return command.mem_startAngle == circle.startAngle && command.mem_totalAngle == circle.sweepRadians &&
            command.mem_totalDist == circle.lengthPulse;
    }

    bool CutterCircleStartMatches(const MotionCommand& command,
        const std::vector<AxisContext>* contexts) noexcept
    {
        if (contexts == nullptr || contexts->size() < 3U) return false;
        for (std::size_t i = 0U; i < 3U; ++i)
        {
            const AxisContext& axis = (*contexts)[i];
            const double actual = axis.logicalCmdPos.Load();
            if (!axis.isExist || axis.axisType != AxisType::LINEAR ||
                !std::isfinite(actual) || !std::isfinite(command.mem_startPos[i]) ||
                std::memcmp(&actual, &command.mem_startPos[i], sizeof(double)) != 0) return false;
        }
        return true;
    }

    bool CncCircleStartMatches(const MotionCommand& command,
        const std::vector<AxisContext>* contexts) noexcept
    {
        if (contexts == nullptr || contexts->size() < 2U) return false;
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            const auto& axis = (*contexts)[i];
            const double actual = axis.logicalCmdPos.Load();
            const double expected = command.mem_startPos[i];
            const double ppm = axis.resolution_PPR / axis.finalLead;
            const double scale = (std::max)(1.0, (std::max)(command.startRadius,
                (std::max)(std::abs(command.centerPos[i]), (std::max)(std::abs(expected), std::abs(command.targetPos[i])))));
            const double error = std::abs(actual - expected);
            if (!std::isfinite(actual) || !std::isfinite(expected) || !std::isfinite(ppm) || ppm <= 0.0 ||
                error > 64.0 * std::numeric_limits<double>::epsilon() * scale || error / ppm > 5e-8) return false;
        }
        return true;
    }

    bool IsMotionCommandConsumerGeometryValid(
        const MotionCommand& command,
        const std::vector<AxisContext>* contexts) noexcept
    {
        if (!IsMotionBaseArcPlaneSourceAllowed(command)) return false;
        if ((!IsNCTranslationSnapshotEmpty(command.sourceTranslation) ||
                command.sourceToolRadiusMode != 40) &&
            !IsMotionFixedTranslationSourceAllowed(command)) return false;
        // DT records native exact-stop G01 provenance, not new Motion authority.
        // Terminal history copies keep the marker; the runtime tail rule below
        // excludes replay. Reject incompatible source geometry, not that copy.
        if (command.pathCoreFeedExactStop &&
            (command.mode != InterpolationMode::LINEAR ||
                command.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
                command.axisCount < 1 || command.axisCount > 3 ||
                command.cncFeedLookahead || command.cncCornerBlend ||
                command.pathCorePlanarCircle || command.pathCoreFullCircle ||
                command.pathCoreRetainedTraversal || command.pathCoreRetainedReverse ||
                command.execution.source != MotionCommandSource::NC_MEMORY || command.ownerLease.owner != MotionOwner::AUTO ||
                !IsMotionFixedTranslationRotationSourceAllowed(command) || !IsMotionFixedTranslationWorkSourceAllowed(command) ||
                !IsMotionFixedTranslationScaleMirrorSourceAllowed(command) ||
                !IsMotionFixedTranslationPolarSourceAllowed(command) || command.sourceG162Active ||
                (!command.sourceIsAbsoluteMode && IsNCTranslationSnapshotEmpty(command.sourceTranslation)) ||
                !IsMotionFixedTranslationSourceAllowed(command) || !IsNCArcPlaneCode(command.sourcePlaneMode) ||
                !IsMotionFixedTranslationToolSourceAllowed(command) || !IsMotionFixedTranslationCutterSourceAllowed(command))) return false;

        if (command.cncFeedLookahead &&
            ((command.mode != InterpolationMode::LINEAR && !command.pathCorePlanarCircle) ||
                command.axisCount < 1 || command.axisCount > 3 ||
                command.commandPathMode != MotionCommandPathMode::CONTINUOUS ||
                command.pathCoreRetainedTraversal || command.pathCoreRetainedReverse ||
                (command.mode == InterpolationMode::LINEAR && command.pathCorePlanarCircle) ||
                command.replayTerminalAlreadyPublished ||
                command.execution.source != MotionCommandSource::NC_MEMORY || command.ownerLease.owner != MotionOwner::AUTO ||
                !IsMotionFixedTranslationRotationSourceAllowed(command) || !IsMotionFixedTranslationWorkSourceAllowed(command) ||
                !IsMotionFixedTranslationScaleMirrorSourceAllowed(command) ||
                !IsMotionFixedTranslationPolarSourceAllowed(command) || command.sourceG162Active ||
                !command.sourceIsAbsoluteMode || !IsMotionFixedTranslationSourceAllowed(command) || command.sourcePlaneMode != 17 ||
                !IsMotionFixedTranslationToolSourceAllowed(command) || command.sourceToolRadiusMode != 40)) return false;

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

        if (!IsMotionCommandRetainedGeometryValid(command, contexts))
            return false;

        NCArcPlaneAxes arcPlane{};
        if (command.pathCorePlanarCircle && !TryGetNCArcPlaneAxes(command.sourcePlaneMode, arcPlane)) return false;
        // Plane-bearing flags authorize only the bounded native circle contract.
        // Legacy LINEAR / spiral commands keep both flags false.
        if (command.pathCoreFullCircle && !command.pathCorePlanarCircle)
        {
            return false;
        }
        if (command.pathCorePlanarCircle &&
            (command.axisCount != 2 ||
                command.axisIndices[0] != static_cast<int>(arcPlane.u) ||
                command.axisIndices[1] != static_cast<int>(arcPlane.v) ||
                command.commandPathMode != (command.cncFeedLookahead ?
                    MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP) ||
                !std::isfinite(command.startRadius) || command.startRadius <= 0.0 ||
                command.startRadius != command.endRadius ||
                !((command.mode == InterpolationMode::CIRCULAR_CW && command.dir == -1) ||
                    (command.mode == InterpolationMode::CIRCULAR_CCW && command.dir == 1))))
        {
            return false;
        }

        // DK metadata is legal only on the existing qualified Q compound.
        // Reject malformed transport; never silently clamp it into permission.
        if (!std::isfinite(command.cncPrefixVelocityPPS) ||
            (command.cncCornerBlend ?
                (command.cncPrefixVelocityPPS != 0.0 && command.cncPrefixVelocityPPS < command.targetVel) :
                command.cncPrefixVelocityPPS != 0.0)) return false;
        if (command.cncCornerBlend && (!command.cncFeedLookahead || command.pathCorePlanarCircle)) return false;
        if (command.cncFeedLookahead && command.mode == InterpolationMode::LINEAR && !command.cncCornerBlend &&
            (command.dir != 0 || command.startRadius != 0.0 || command.endRadius != 0.0 ||
                command.mem_radius != 0.0 || command.mem_totalAngle != 0.0)) return false;
        if (command.cncFeedLookahead && (command.pathCorePlanarCircle || command.cncCornerBlend))
        {
            NCPathCoreArcPulseGeometry circle{};
            double speed = 0.0, acc = 0.0, dec = 0.0;
            double prefix = 0.0;
            if (!(command.cncCornerBlend ? ResolveCncCornerBlend(command, prefix) : ResolveCncPlanarCircle(command, contexts, circle)) ||
                !ComputeCncPathDynamics(command, speed, acc, dec)) return false;
            double ppm = 0.0;
            for (std::size_t i = 0U; i < 2U; ++i)
            {
                if (contexts->size() <= i) return false;
                const auto& axis = (*contexts)[i];
                const double next = axis.resolution_PPR / axis.finalLead;
                if (!std::isfinite(next) || next <= 0.0 || (i != 0U && ppm != next) ||
                    !std::isfinite(axis.maxVel_PPS) || command.targetVel > axis.maxVel_PPS) return false;
                // The authored prefix uses the same native XY scalar metric.
                // It may exceed packet min, never either live axis cap or F100.
                if (command.cncPrefixVelocityPPS != 0.0 &&
                    (command.cncPrefixVelocityPPS > axis.maxVel_PPS ||
                        command.cncPrefixVelocityPPS / next > (100.0 / 60.0) *
                            (1.0 + 16.0 * std::numeric_limits<double>::epsilon()))) return false;
                ppm = next;
            }
        }

        if (command.pathCorePlanarCircle && command.sourceTranslation.cutterMode != 40)
        {
            NCPathCoreArcPulseGeometry circle{};
            if (!ResolveCutterPlanarCircle(command, contexts, circle) || contexts->size() < 3U ||
                command.targetVel < 1.0 || command.accTime <= 0.0 || command.decTime <= 0.0)
                return false;
            double ppm = 0.0;
            unsigned planeAxesSeen = 0U;
            for (std::size_t i = 0U; i < 3U; ++i)
            {
                const AxisContext& axis = (*contexts)[i];
                const double scale = axis.resolution_PPR / axis.finalLead;
                if (!axis.isExist || axis.axisType != AxisType::LINEAR ||
                    !std::isfinite(scale) || scale <= 0.0 ||
                    !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
                    !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0) return false;
                if (i == arcPlane.u || i == arcPlane.v)
                {
                    if ((planeAxesSeen != 0U && scale != ppm) || !std::isfinite(axis.maxVel_PPS) ||
                        command.targetVel > axis.maxVel_PPS ||
                        command.targetVel / scale > (100.0 / 60.0) *
                            (1.0 + 16.0 * std::numeric_limits<double>::epsilon())) return false;
                    ppm = scale;
                    ++planeAxesSeen;
                }
            }
            if (planeAxesSeen != 2U) return false;
        }
        // BASE-PLANE-20: G17 polar circles use the same axis metric and speed proof.
        if (command.pathCorePlanarCircle && (command.sourcePlaneMode != 17 || command.sourceG16Active))
        {
            if (command.targetVel < 1.0 || command.accTime <= 0.0 || command.decTime <= 0.0) return false;
            double ppm = 0.0;
            for (unsigned component = 0U; component < 2U; ++component)
            {
                const unsigned slot = component == 0U ? arcPlane.u : arcPlane.v;
                if (contexts->size() <= slot) return false;
                const AxisContext& axis = (*contexts)[slot];
                const double scale = axis.resolution_PPR / axis.finalLead;
                if (!axis.isExist || axis.axisType != AxisType::LINEAR ||
                    !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
                    !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                    !std::isfinite(scale) || scale <= 0.0 ||
                    (component != 0U && scale != ppm) || !std::isfinite(axis.maxVel_PPS) ||
                    command.targetVel > axis.maxVel_PPS ||
                    command.targetVel / scale > (100.0 / 60.0) *
                        (1.0 + 16.0 * std::numeric_limits<double>::epsilon())) return false;
                ppm = scale;
            }
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
            if ((command.cncFeedLookahead || command.pathCoreFeedExactStop ||
                !IsNCTranslationSnapshotEmpty(command.sourceTranslation)) && (axisIndex > 2 ||
                (*contexts)[axisIndex].axisType != AxisType::LINEAR)) return false;
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

    // A front command may wait only for a physically recoverable settle
    // condition.  Faults, invalid numerical state, an orphaned interpolation
    // state, or a startup-lag block remain hard integrity failures and are
    // deliberately allowed to reach the existing post-pop AL3021 path.
    // This classifier is read-only and is used solely before the SPSC pop.
    bool IsIncomingPhysicalAxisReadinessTransient(
        const AxisContext& axis) noexcept
    {
        const bool finiteCommandState =
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
            axis.inPositionWindow_Pulse > 0.0;
        const bool settlingState =
            axis.state == MotionState::MotionState_IDLE ||
            axis.state == MotionState::MotionState_MOVING ||
            axis.state == MotionState::MotionState_STOPPING ||
            axis.state == MotionState::MotionState_VELOCITY ||
            axis.state == MotionState::MotionState_MPG;
        return
            axis.isExist &&
            !axis.startupLagPrematureMotionBlocked &&
            !axis.isFault &&
            !axis.isLagAlarm &&
            finiteCommandState &&
            settlingState;
    }

    bool IsMotionCommandHistorySnapshotValid(
        const MotionCommand& command) noexcept
    {
        const int axisCount = ClampMotionAxisCount(command.axisCount);
        if (command.pathCoreRetainedTraversal || command.pathCoreRetainedReverse ||
            axisCount <= 0 ||
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


bool MotionCore::IsExactResetNCSettleAuthorityCurrent(
    const MotionNCSettleRequest& request) const noexcept
{
    if (request.profile != MotionNCSettleProfile::RESET_ALL ||
        request.requestSequence ==
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        request.safetyRequestTicket == 0U ||
        request.safetyProvenanceGeneration == 0ULL)
    {
        return false;
    }

    MotionExecutionEpoch exactEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    return
        TryGetResetSafetyMotionOwnerEpoch(
            request.ownerLease,
            request.safetyRequestTicket,
            request.safetyProvenanceGeneration,
            exactEpoch) == ResetSafetyAuthorityStatus::ACQUIRED &&
        exactEpoch == request.executionEpoch;
}


bool MotionCore::TryAcquireExactResetNCSettleReservation(
    const MotionNCSettleRequest& request,
    std::uint64_t& reservedOwnerState) noexcept
{
    reservedOwnerState = 0ULL;
    if (!IsExactResetNCSettleAuthorityCurrent(request))
    {
        return false;
    }

    std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if (!UnpackMotionOwnerState(ownerState).Matches(request.ownerLease) ||
        UnpackMotionOwnerSafetyRequestTicket(ownerState) !=
        request.safetyRequestTicket ||
        UnpackMotionOwnerSafetyHandshake(ownerState) ||
        UnpackMotionOwnerSafetyActionPending(ownerState) ||
        (ownerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
    {
        return false;
    }

    const std::uint64_t desired =
        ownerState | MOTION_OWNER_EPOCH_COMMIT_RESERVED;
    if (!m_motionOwnerState.compare_exchange_strong(
        ownerState,
        desired,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    reservedOwnerState = desired;
    if (m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire) !=
        request.safetyProvenanceGeneration ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_motionOwnerState.load(std::memory_order_acquire) != desired ||
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire) !=
        request.safetyRequestTicket)
    {
        ReleaseExactResetNCSettleReservation(reservedOwnerState);
        reservedOwnerState = 0ULL;
        return false;
    }

    return true;
}


void MotionCore::ReleaseExactResetNCSettleReservation(
    std::uint64_t reservedOwnerState) noexcept
{
    if ((reservedOwnerState &
        MOTION_OWNER_EPOCH_COMMIT_RESERVED) == 0ULL)
    {
        return;
    }

    std::uint64_t expected = reservedOwnerState;
    const std::uint64_t released =
        reservedOwnerState &
        ~MOTION_OWNER_EPOCH_COMMIT_RESERVED;
    if (!m_motionOwnerState.compare_exchange_strong(
        expected,
        released,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_motionOwnerState.fetch_and(
            ~MOTION_OWNER_EPOCH_COMMIT_RESERVED,
            std::memory_order_acq_rel);
    }
}


bool MotionCore::TryPublishNCResetSafetyReleaseAuthorization() noexcept
{
    const MotionNCSettleRequest& request =
        m_activeResetNCSettleRequest;
    if (m_ncResetRebasePhase !=
        MotionNCResetRebasePhase::ACKNOWLEDGED ||
        !IsExactResetNCSettleAuthorityCurrent(request) ||
        !m_ncResetRebaseAckProducer.acknowledged ||
        !m_ncResetRebaseAckProducer.acked ||
        m_ncResetRebaseAckProducer.requestSequence !=
        request.requestSequence)
    {
        return false;
    }

    if (m_ncResetReleaseAuthState.load(
        std::memory_order_acquire) !=
        RESET_RELEASE_AUTH_EMPTY)
    {
        return false;
    }

    m_ncResetReleaseAuthWriteSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);
    m_ncResetReleaseAuthRequestSequence.store(
        request.requestSequence,
        std::memory_order_relaxed);
    m_ncResetReleaseAuthPackedIdentity.store(
        (static_cast<std::uint64_t>(
            request.ownerLease.generation) << 32U) |
        static_cast<std::uint64_t>(request.executionEpoch),
        std::memory_order_relaxed);
    m_ncResetReleaseAuthSafetyTicket.store(
        request.safetyRequestTicket,
        std::memory_order_relaxed);
    m_ncResetReleaseAuthDrainGeneration.store(
        request.safetyProvenanceGeneration,
        std::memory_order_relaxed);
    m_ncResetReleaseAuthWriteSequence.fetch_add(
        1ULL,
        std::memory_order_release);

    // The payload is not consumable until the complete Reset tuple is proved
    // again after publication. NC can only claim AVAILABLE, never the
    // seqlock payload while it is being replaced or after ACK was revoked.
    if (m_ncResetRebasePhase !=
        MotionNCResetRebasePhase::ACKNOWLEDGED ||
        !IsExactResetNCSettleAuthorityCurrent(request) ||
        !m_ncResetRebaseAckProducer.acknowledged ||
        !m_ncResetRebaseAckProducer.acked ||
        m_ncResetRebaseAckProducer.requestSequence !=
        request.requestSequence)
    {
        return false;
    }

    std::uint8_t expectedState = RESET_RELEASE_AUTH_EMPTY;
    return m_ncResetReleaseAuthState.compare_exchange_strong(
        expectedState,
        RESET_RELEASE_AUTH_AVAILABLE,
        std::memory_order_release,
        std::memory_order_acquire);
}


bool MotionCore::TryGetNCResetSafetyReleaseAuthorization(
    MotionNCSettleRequestSequence requestSequence,
    MotionNCResetSafetyReleaseAuthorization& authorization) const noexcept
{
    authorization = MotionNCResetSafetyReleaseAuthorization{};
    if (requestSequence ==
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID)
    {
        return false;
    }

    const std::uint8_t entryState =
        m_ncResetReleaseAuthState.load(std::memory_order_acquire);
    if (entryState != RESET_RELEASE_AUTH_AVAILABLE &&
        entryState != RESET_RELEASE_AUTH_CLAIMED)
    {
        return false;
    }

    for (unsigned attempt = 0U; attempt < 16U; ++attempt)
    {
        const std::uint64_t sequenceBefore =
            m_ncResetReleaseAuthWriteSequence.load(
                std::memory_order_acquire);
        if ((sequenceBefore & 1ULL) != 0ULL)
        {
            continue;
        }

        MotionNCResetSafetyReleaseAuthorization candidate{};
        candidate.requestSequence =
            m_ncResetReleaseAuthRequestSequence.load(
                std::memory_order_relaxed);
        const std::uint64_t packedIdentity =
            m_ncResetReleaseAuthPackedIdentity.load(
                std::memory_order_relaxed);
        candidate.executionEpoch =
            static_cast<MotionExecutionEpoch>(
                packedIdentity & 0xFFFFFFFFULL);
        candidate.ownerGeneration =
            static_cast<MotionOwnerGeneration>(
                packedIdentity >> 32U);
        candidate.safetyRequestTicket =
            m_ncResetReleaseAuthSafetyTicket.load(
                std::memory_order_relaxed);
        candidate.drainRevocationGeneration =
            m_ncResetReleaseAuthDrainGeneration.load(
                std::memory_order_relaxed);

        const std::uint64_t sequenceAfter =
            m_ncResetReleaseAuthWriteSequence.load(
                std::memory_order_acquire);
        const std::uint8_t exitState =
            m_ncResetReleaseAuthState.load(std::memory_order_acquire);
        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1ULL) == 0ULL &&
            exitState == entryState &&
            candidate.requestSequence == requestSequence &&
            candidate.IsValid())
        {
            authorization = candidate;
            return true;
        }
    }

    return false;
}


bool MotionCore::TryClaimNCResetSafetyReleaseAuthorization(
    MotionNCSettleRequestSequence requestSequence,
    MotionNCResetSafetyReleaseAuthorization& authorization) noexcept
{
    authorization = MotionNCResetSafetyReleaseAuthorization{};
    std::uint8_t expectedState = RESET_RELEASE_AUTH_AVAILABLE;
    if (!m_ncResetReleaseAuthState.compare_exchange_strong(
        expectedState,
        RESET_RELEASE_AUTH_CLAIMED,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    if (!TryGetNCResetSafetyReleaseAuthorization(
        requestSequence,
        authorization))
    {
        (void)ReleaseNCResetSafetyReleaseAuthorizationClaim();
        return false;
    }

    return true;
}


bool MotionCore::TryInvalidateNCResetSafetyReleaseAuthorization() noexcept
{
    std::uint8_t state =
        m_ncResetReleaseAuthState.load(std::memory_order_acquire);
    if (state == RESET_RELEASE_AUTH_EMPTY)
    {
        return true;
    }
    if (state == RESET_RELEASE_AUTH_CLAIMED ||
        state == RESET_RELEASE_AUTH_CONSUMED)
    {
        return false;
    }

    return m_ncResetReleaseAuthState.compare_exchange_strong(
        state,
        RESET_RELEASE_AUTH_EMPTY,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}


bool MotionCore::ReleaseNCResetSafetyReleaseAuthorizationClaim() noexcept
{
    std::uint8_t expectedState = RESET_RELEASE_AUTH_CLAIMED;
    return m_ncResetReleaseAuthState.compare_exchange_strong(
        expectedState,
        RESET_RELEASE_AUTH_EMPTY,
        std::memory_order_release,
        std::memory_order_acquire);
}


bool MotionCore::ConsumeNCResetSafetyReleaseAuthorizationClaim() noexcept
{
    std::uint8_t expectedState = RESET_RELEASE_AUTH_CLAIMED;
    return m_ncResetReleaseAuthState.compare_exchange_strong(
        expectedState,
        RESET_RELEASE_AUTH_CONSUMED,
        std::memory_order_release,
        std::memory_order_acquire);
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
    std::uint32_t safetyRequestTicket,
    std::uint64_t safetyProvenanceGeneration,
    const MotionNCResetExecutionState& executionState,
    bool unsupportedFaultOrEstop) noexcept
{
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !ownerLease.IsValid() ||
        safetyRequestTicket == 0U ||
        safetyProvenanceGeneration == 0ULL)
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    MotionNCSettleRequest request{};
    request.requestSequence = AllocateNCSettleRequestSequence();
    request.profile = MotionNCSettleProfile::RESET_ALL;
    request.executionEpoch = executionEpoch;
    request.ownerLease = ownerLease;
    request.safetyRequestTicket = safetyRequestTicket;
    request.safetyProvenanceGeneration =
        safetyProvenanceGeneration;
    request.resetExecutionState = executionState;
    request.unsupportedFaultOrEstop = unsupportedFaultOrEstop;

    std::uint64_t reservedOwnerState = 0ULL;
    if (!TryAcquireExactResetNCSettleReservation(
        request,
        reservedOwnerState))
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    ReleaseExactResetNCSettleReservation(reservedOwnerState);
    // Never expose the request while its own owner reservation is visible to
    // the 250 us consumer: a pop in that window would look like an unrelated
    // supersession.  Revalidate once after release, then publish.  Any Safety
    // event racing the release-push is carried by ticket/provenance in the
    // payload and is rejected again at dequeue and commit.
    if (!IsExactResetNCSettleAuthorityCurrent(request))
    {
        return MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    const bool submitted = SubmitNCSettleRequest(request);
    if (!submitted)
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
        InvalidateServoOutputImageProof();
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
    InvalidateCncLineEndpointProof();
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

    if (!IsExactResetNCSettleAuthorityCurrent(request))
    {
        return MotionNCSettleBlocker::SAFETY_OR_RECOVERY_PENDING;
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
    if (m_ncResetReleaseAuthState.load(std::memory_order_acquire) ==
        RESET_RELEASE_AUTH_CONSUMED)
    {
        // A successful NC release consumes the exact ACK once. Retire the RT
        // transaction normally on the following pass while preserving its
        // ACK/history for the release gate and diagnostics; owner NONE is the
        // expected result here, not a superseding lifecycle incident.
        MotionNCSettleTracker& consumedTracker =
            m_ncSettleTrackers[static_cast<std::size_t>(
                MotionNCSettleProfile::RESET_ALL)];
        consumedTracker.candidate = false;
        consumedTracker.settled = false;
        consumedTracker.dwellCycles = 0U;
        consumedTracker.requestAccepted = false;
        m_activeResetNCSettleRequest = MotionNCSettleRequest{};
        m_ncResetRebasePhase = MotionNCResetRebasePhase::IDLE;
        m_ncResetScalarRebaseApplied = false;
        m_ncResetBufferClearAxisSlot = 0U;
        m_ncResetBufferClearElement = 0U;

        std::uint8_t expectedState = RESET_RELEASE_AUTH_CONSUMED;
        if (!m_ncResetReleaseAuthState.compare_exchange_strong(
            expectedState,
            RESET_RELEASE_AUTH_EMPTY,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return true;
        }
    }

    // NC owns a release authorization only after an AVAILABLE -> CLAIMED
    // transition. While that one-shot token is claimed, RT freezes the
    // Reset settle/rebase producer instead of interpreting NC's temporary
    // EPOCH reservation as a superseding owner change. Otherwise revoke
    // any older unclaimed token before this cycle can mutate ACK state.
    if (!TryInvalidateNCResetSafetyReleaseAuthorization())
    {
        return true;
    }

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
                observedOwnerLease) &&
            IsExactResetNCSettleAuthorityCurrent(activeRequest);
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
        const bool exactResetTupleCurrent =
            request.profile == MotionNCSettleProfile::RESET_ALL &&
            tupleCurrent &&
            IsExactResetNCSettleAuthorityCurrent(request);

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

            // CG FIX1: G180/G04 and pure logic can HOLD before any Motion
            // segment exists in this epoch. Bind a drained NC program to all
            // physical axes; retain the full 200-cycle stop proof below.
            const bool pureProgramScope =
                tupleCurrent && !requestIdentityCurrent &&
                (request.ownerLease.owner == MotionOwner::AUTO ||
                    request.ownerLease.owner == MotionOwner::MDI ||
                    request.ownerLease.owner == MotionOwner::MANUAL_AUTO) &&
                !m_Group.isActive && m_Group.cmdQueue.ingress_size() == 0U &&
                m_Group.cmdQueue.replay_size() == 0U &&
                m_axisCommandChannel.command_size() == 0U &&
                m_Group.pathMode != PathMode::PATH_SERVO &&
                m_Group.pathMode != PathMode::JUMP_TRACKING &&
                m_Group.jumpManager.state == JumpState::IDLE;
            tracker.scopeMask = pureProgramScope
                ? BuildExistingNCAxisMask()
                : currentScopeMask;
            if (requestIdentityCurrent && tracker.scopeMask != 0U)
            {
                m_ncLastGroupScopeMask = tracker.scopeMask;
                m_ncLastGroupScopeExecutionEpoch = request.executionEpoch;
                m_ncLastGroupScopeExecutionIdentity =
                    tracker.executionIdentity;
            }
            else if (requestIdentityCurrent && m_ncLastGroupScopeMask != 0U &&
                m_ncLastGroupScopeExecutionEpoch == request.executionEpoch &&
                MotionExecutionIdentityExactlyMatches(
                    m_ncLastGroupScopeExecutionIdentity,
                    tracker.executionIdentity))
            {
                tracker.scopeMask = m_ncLastGroupScopeMask;
            }
            tracker.requestAccepted =
                tupleCurrent &&
                (requestIdentityCurrent || pureProgramScope) &&
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
                    exactResetTupleCurrent &&
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
                    else if (!exactResetTupleCurrent)
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
        const MotionNCSettleBlocker currentCommitBlocker =
            ValidateNCResetCommitSeam();
        if (currentCommitBlocker != MotionNCSettleBlocker::NONE)
        {
            BlockNCResetCommit(currentCommitBlocker);
            return true;
        }

        std::uint64_t reservedOwnerState = 0ULL;
        if (!TryAcquireExactResetNCSettleReservation(
            m_activeResetNCSettleRequest,
            reservedOwnerState))
        {
            BlockNCResetCommit(
                MotionNCSettleBlocker::
                SAFETY_OR_RECOVERY_PENDING);
            return true;
        }

        if (!m_ncResetScalarRebaseApplied)
        {
            // Commit-time reservation is held through the scalar rebase and
            // this pass's bounded buffer-clear chunk.  Every later chunk
            // reacquires and revalidates the same exact Reset identity.
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

        ReleaseExactResetNCSettleReservation(
            reservedOwnerState);

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
    MotionOwnerGeneration generation,
    std::uint32_t safetyRequestTicket,
    bool safetyHandshakeInProgress,
    bool safetyActionPending) noexcept
{
    return
        (static_cast<std::uint64_t>(generation) << 32U) |
        ((static_cast<std::uint64_t>(safetyRequestTicket) <<
            MOTION_OWNER_SAFETY_TICKET_SHIFT) &
            MOTION_OWNER_SAFETY_TICKET_MASK) |
        (safetyHandshakeInProgress
            ? MOTION_OWNER_SAFETY_HANDSHAKE
            : 0ULL) |
        (safetyActionPending
            ? MOTION_OWNER_SAFETY_ACTION_PENDING
            : 0ULL) |
        (static_cast<std::uint64_t>(
            static_cast<std::uint8_t>(owner)) &
            MOTION_OWNER_VALUE_MASK);
}


MotionOwnerLease MotionCore::UnpackMotionOwnerState(
    std::uint64_t packed) noexcept
{
    MotionOwnerLease lease{};

    lease.owner =
        static_cast<MotionOwner>(
            static_cast<std::uint8_t>(
                packed & MOTION_OWNER_VALUE_MASK));

    lease.generation =
        static_cast<MotionOwnerGeneration>(
            packed >> 32U);

    return lease;
}


std::uint32_t MotionCore::UnpackMotionOwnerSafetyRequestTicket(
    std::uint64_t packed) noexcept
{
    return static_cast<std::uint32_t>(
        (packed & MOTION_OWNER_SAFETY_TICKET_MASK) >>
        MOTION_OWNER_SAFETY_TICKET_SHIFT);
}


bool MotionCore::UnpackMotionOwnerSafetyHandshake(
    std::uint64_t packed) noexcept
{
    return
        (packed & MOTION_OWNER_SAFETY_HANDSHAKE) != 0ULL;
}


bool MotionCore::UnpackMotionOwnerSafetyActionPending(
    std::uint64_t packed) noexcept
{
    return
        (packed & MOTION_OWNER_SAFETY_ACTION_PENDING) != 0ULL;
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

    const std::uint64_t packed =
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease current =
        UnpackMotionOwnerState(packed);
    return
        !UnpackMotionOwnerSafetyHandshake(packed) &&
        current.Matches(lease);
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

    if (requestedOwner == MotionOwner::SAFETY)
    {
        outLease = TakeSafetyMotionOwner();
        return outLease.IsValid();
    }

    std::uint64_t currentPacked =
        m_motionOwnerState.load(
            std::memory_order_acquire);

    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        const MotionOwnerLease currentLease =
            UnpackMotionOwnerState(currentPacked);
        const std::uint32_t safetyTicket =
            UnpackMotionOwnerSafetyRequestTicket(currentPacked);

        if (UnpackMotionOwnerSafetyHandshake(currentPacked) ||
            UnpackMotionOwnerSafetyActionPending(currentPacked) ||
            (currentPacked &
                MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
            safetyTicket != m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire))
        {
            return false;
        }

        // 同一個邏輯 Owner 重複 Acquire 視為冪等操作，不產生新世代。
        if (currentLease.owner == requestedOwner &&
            currentLease.generation != MOTION_OWNER_GENERATION_INVALID)
        {
            outLease = currentLease;
            return true;
        }

        if (currentLease.owner != MotionOwner::NONE &&
            currentLease.owner != MotionOwner::IDLE_HOLD)
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
                requestedLease.generation,
                safetyTicket,
                false);

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

    return false;
}


bool MotionCore::TryEnterProgramEndIdleHold(
    const MotionOwnerLease& programLease,
    MotionExecutionEpoch expectedEpoch) noexcept
{
    if ((programLease.owner != MotionOwner::AUTO &&
        programLease.owner != MotionOwner::MDI &&
        programLease.owner != MotionOwner::MANUAL_AUTO) ||
        !HasExactExecutionDrainAcknowledgement(expectedEpoch, programLease))
    {
        return false;
    }
    const auto exactSettledProof = [&programLease, expectedEpoch](
        const MotionNCSettleSnapshot& sample) noexcept -> bool
    {
        return sample.profile == MotionNCSettleProfile::GROUP_COMPLETION &&
            sample.publicationGeneration != 0ULL && sample.proofSequence != 0ULL &&
            sample.runtimeObserved && sample.runtimeCycleValid &&
            sample.runtimeCycleContiguous && sample.groupDrained && !sample.groupActive &&
            sample.settled && sample.scopeMask != 0U && !sample.safetyOrRecoveryPending &&
            sample.commandQueueDepth == 0U && sample.commandIngressDepth == 0U &&
            sample.commandReplayDepth == 0U && sample.executionEpoch == expectedEpoch &&
            sample.owner == programLease.owner && sample.ownerGeneration == programLease.generation &&
            sample.requiredCycles == MOTION_NC_SETTLE_REQUIRED_CYCLES &&
            sample.dwellCycles >= sample.requiredCycles;
    };
    MotionNCSettleSnapshot proof{};
    MotionNCSettleCounters counters{};
    if (!TryGetNCSettleEvidence(MotionNCSettleProfile::GROUP_COMPLETION,
        proof, counters) || !exactSettledProof(proof))
    {
        return false;
    }
    AlarmManager& alarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation admission{};
    if (!alarms.BeginMotionAdmission(alarms.GetUpdateCount(), admission))
    {
        return false;
    }
    std::uint64_t ownerState = m_motionOwnerState.load(std::memory_order_acquire);
    std::uint64_t execution = m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t baseExecution = execution;
    const std::uint32_t safetyTicket = UnpackMotionOwnerSafetyRequestTicket(ownerState);
    bool entered = false;
    bool postCommitCurrent = true;
    bool epochReleaseFailed = false;
    MotionOwnerLease holdLease{};
    holdLease.owner = MotionOwner::IDLE_HOLD;
    holdLease.generation = NextMotionOwnerGeneration(programLease.generation);
    if (UnpackMotionOwnerState(ownerState).Matches(programLease) &&
        !UnpackMotionOwnerSafetyHandshake(ownerState) &&
        !UnpackMotionOwnerSafetyActionPending(ownerState) &&
        (ownerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL &&
        safetyTicket == m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire) &&
        UnpackExecutionEpochPublication(execution) == expectedEpoch &&
        (execution & (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) == 0ULL &&
        !HasPendingSafetyOrRecoveryRequests() &&
        HasExactExecutionDrainAcknowledgement(expectedEpoch, programLease) &&
        alarms.IsMotionAdmissionCurrent(admission) &&
        m_executionEpochPublication.compare_exchange_strong(execution,
            baseExecution | EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED,
            std::memory_order_acq_rel, std::memory_order_acquire))
    {
        // Re-read the RT stop proof after reserving this exact epoch.
        if (TryGetNCSettleEvidence(MotionNCSettleProfile::GROUP_COMPLETION,
            proof, counters) && exactSettledProof(proof) &&
            alarms.IsMotionAdmissionCurrent(admission) &&
            !HasPendingSafetyOrRecoveryRequests())
        {
            // Bind epoch before owner publication, including the first RT pass.
            m_programEndIdleHoldGrant.store(
                (static_cast<std::uint64_t>(holdLease.generation) << 32U) |
                static_cast<std::uint64_t>(expectedEpoch),
                std::memory_order_release);
            entered = m_motionOwnerState.compare_exchange_strong(ownerState,
                PackMotionOwnerState(holdLease.owner, holdLease.generation,
                    safetyTicket, false),
                std::memory_order_acq_rel, std::memory_order_acquire);
        }
        std::uint64_t reserved = baseExecution | EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED;
        if (!m_executionEpochPublication.compare_exchange_strong(reserved,
            baseExecution, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            // Exact-token release only: never clear another publisher's reservation.
            epochReleaseFailed = true;
            postCommitCurrent = false;
        }
    }
    if (!alarms.EndMotionAdmission(admission) ||
        !IsMotionOwnerLeaseCurrent(holdLease) ||
        GetCurrentExecutionEpoch() != expectedEpoch ||
        HasPendingSafetyOrRecoveryRequests())
    {
        postCommitCurrent = false;
    }
    if (epochReleaseFailed)
    {
        RtPrintf("[IDLE-CJ] FAILED reason=EPOCH_RESERVATION_LOST epoch=%u\n", expectedEpoch);
        if (!alarms.HasAlarm()) alarms.Trigger(AlarmManager::IDLE_POSITION_HOLD_FAILED);
    }
    if (!entered)
    {
        return false;
    }
    // A successful CAS consumed the program lease. Even an immediately
    // cancelled hold is a committed handoff, never an NC retry of that lease.
    if (!postCommitCurrent)
    {
        (void)ReleaseMotionOwner(holdLease);
    }
    RtPrintf("[IDLE-CJ] ENTER owner=%u generation=%u epoch=%u mask=%u current=%u\n",
        static_cast<unsigned>(holdLease.owner), holdLease.generation,
        expectedEpoch, proof.scopeMask, postCommitCurrent ? 1U : 0U);
    return true;
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

    if (requestedOwner == MotionOwner::SAFETY)
    {
        if (!IsMotionOwnerLeaseCurrent(currentLease))
        {
            return false;
        }
        outLease = TakeSafetyMotionOwner();
        return outLease.IsValid();
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
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease observedLease =
        UnpackMotionOwnerState(expectedPacked);
    const std::uint32_t safetyTicket =
        UnpackMotionOwnerSafetyRequestTicket(expectedPacked);
    if (!observedLease.Matches(currentLease) ||
        UnpackMotionOwnerSafetyHandshake(expectedPacked) ||
        UnpackMotionOwnerSafetyActionPending(expectedPacked) ||
        (expectedPacked &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        safetyTicket != m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire))
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
            requestedLease.generation,
            safetyTicket,
            false);

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
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease observedLease =
        UnpackMotionOwnerState(expectedPacked);
    const std::uint32_t safetyTicket =
        UnpackMotionOwnerSafetyRequestTicket(expectedPacked);
    if (!observedLease.Matches(lease) ||
        UnpackMotionOwnerSafetyHandshake(expectedPacked) ||
        UnpackMotionOwnerSafetyActionPending(expectedPacked) ||
        (expectedPacked &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
    {
        return false;
    }

    if (safetyTicket !=
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire))
    {
        return false;
    }

    if (lease.owner == MotionOwner::SAFETY)
    {
        if (m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
            m_safetyRecoveryRequestInProgress.load(
                std::memory_order_acquire) ||
            HasPendingSafetyOrRecoveryRequests() ||
            m_safetyControlledStopInProgress)
        {
            return false;
        }
    }

    const std::uint64_t releasedPacked =
        PackMotionOwnerState(
            MotionOwner::NONE,
            NextMotionOwnerGeneration(lease.generation),
            safetyTicket,
            false);

    return
        m_motionOwnerState.compare_exchange_strong(
            expectedPacked,
            releasedPacked,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
}


MotionCore::SafetyMotionOwnerReleaseStatus
MotionCore::TryReleaseSafetyMotionOwner(
    const MotionOwnerLease& lease,
    const MotionNCResetSafetyReleaseAuthorization& authorization) noexcept
{
    const auto authorizationMatches =
        [&authorization](
            const MotionNCResetSafetyReleaseAuthorization& candidate)
        noexcept
    {
        return
            candidate.requestSequence ==
            authorization.requestSequence &&
            candidate.drainRevocationGeneration ==
            authorization.drainRevocationGeneration &&
            candidate.executionEpoch ==
            authorization.executionEpoch &&
            candidate.ownerGeneration ==
            authorization.ownerGeneration &&
            candidate.safetyRequestTicket ==
            authorization.safetyRequestTicket;
    };

    if (!authorization.IsValid() ||
        !lease.IsValid() ||
        lease.owner != MotionOwner::SAFETY ||
        authorization.ownerGeneration != lease.generation)
    {
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }

    // Classify a bounded retry without weakening the exact Reset identity.
    // A reservation or an incomplete Epoch/ACK publication is transient only
    // while the complete lease/ticket/provenance tuple is still current.
    // Any later Safety producer makes the old release permanently stale.
    const auto classifyCurrentTuple = [this, &lease, &authorization]()
        noexcept -> SafetyMotionOwnerReleaseStatus
    {
        const auto ownerIdentityMatches =
            [this, &lease, &authorization](std::uint64_t ownerState)
            noexcept
        {
            return
                UnpackMotionOwnerState(ownerState).Matches(lease) &&
                UnpackMotionOwnerSafetyRequestTicket(ownerState) ==
                authorization.safetyRequestTicket &&
                !UnpackMotionOwnerSafetyHandshake(ownerState) &&
                !UnpackMotionOwnerSafetyActionPending(ownerState);
        };
        const auto epochIdentityMatches =
            [&lease, &authorization](
                std::uint64_t acknowledgement,
                std::uint64_t publication) noexcept
        {
            return
                static_cast<MotionOwnerGeneration>(
                    acknowledgement >> 32U) == lease.generation &&
                static_cast<MotionExecutionEpoch>(
                    acknowledgement & 0xFFFFFFFFULL) ==
                authorization.executionEpoch &&
                UnpackExecutionEpochPublication(publication) ==
                authorization.executionEpoch &&
                UnpackExecutionEpochPublicationSource(publication) ==
                MotionCommandSource::SAFETY;
        };

        if (m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire) !=
            authorization.drainRevocationGeneration ||
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) != 0U ||
            m_safetyRecoveryRequestInProgress.load(
                std::memory_order_acquire) ||
            HasPendingSafetyOrRecoveryRequests() ||
            m_safetyControlledStopInProgress)
        {
            return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
        }

        const std::uint64_t currentOwnerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        const std::uint64_t currentAcknowledgement =
            m_safetyOwnerEpochAcknowledgement.load(
                std::memory_order_acquire);
        const std::uint64_t currentPublication =
            m_executionEpochPublication.load(
                std::memory_order_acquire);
        if (!ownerIdentityMatches(currentOwnerState) ||
            m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire) !=
            authorization.safetyRequestTicket ||
            !epochIdentityMatches(
                currentAcknowledgement,
                currentPublication) ||
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) !=
            authorization.drainRevocationGeneration ||
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) != 0U ||
            m_safetyRecoveryRequestInProgress.load(
                std::memory_order_acquire) ||
            HasPendingSafetyOrRecoveryRequests() ||
            m_safetyControlledStopInProgress ||
            !ownerIdentityMatches(
                m_motionOwnerState.load(std::memory_order_acquire)) ||
            m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire) !=
            authorization.safetyRequestTicket ||
            !epochIdentityMatches(
                m_safetyOwnerEpochAcknowledgement.load(
                    std::memory_order_acquire),
                m_executionEpochPublication.load(
                    std::memory_order_acquire)))
        {
            return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
        }

        return SafetyMotionOwnerReleaseStatus::DEFERRED;
    };

    const std::uint8_t authorizationEntryState =
        m_ncResetReleaseAuthState.load(std::memory_order_acquire);
    MotionNCResetSafetyReleaseAuthorization publishedAuthorization{};
    if (!TryClaimNCResetSafetyReleaseAuthorization(
        authorization.requestSequence,
        publishedAuthorization))
    {
        const std::uint8_t authorizationExitState =
            m_ncResetReleaseAuthState.load(std::memory_order_acquire);
        if (authorizationEntryState == RESET_RELEASE_AUTH_CONSUMED ||
            authorizationExitState == RESET_RELEASE_AUTH_CONSUMED)
        {
            return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
        }

        MotionNCResetSafetyReleaseAuthorization transientAuthorization{};
        if ((authorizationEntryState == RESET_RELEASE_AUTH_AVAILABLE ||
            authorizationEntryState == RESET_RELEASE_AUTH_CLAIMED ||
            authorizationExitState == RESET_RELEASE_AUTH_AVAILABLE ||
            authorizationExitState == RESET_RELEASE_AUTH_CLAIMED) &&
            TryGetNCResetSafetyReleaseAuthorization(
                authorization.requestSequence,
                transientAuthorization) &&
            authorizationMatches(transientAuthorization))
        {
            return classifyCurrentTuple();
        }

        // A competing claimant or this routine's previous reservation retry
        // may leave the authorization briefly EMPTY before RT republishes the
        // same ACK. Take one fixed-cost seqlock snapshot to distinguish that
        // exact transient from a different authorization. An unstable writer
        // is also retryable only while the live owner tuple remains exact.
        const std::uint64_t sequenceBefore =
            m_ncResetReleaseAuthWriteSequence.load(
                std::memory_order_acquire);
        MotionNCResetSafetyReleaseAuthorization snapshot{};
        snapshot.requestSequence =
            m_ncResetReleaseAuthRequestSequence.load(
                std::memory_order_relaxed);
        const std::uint64_t packedIdentity =
            m_ncResetReleaseAuthPackedIdentity.load(
                std::memory_order_relaxed);
        snapshot.executionEpoch =
            static_cast<MotionExecutionEpoch>(
                packedIdentity & 0xFFFFFFFFULL);
        snapshot.ownerGeneration =
            static_cast<MotionOwnerGeneration>(
                packedIdentity >> 32U);
        snapshot.safetyRequestTicket =
            m_ncResetReleaseAuthSafetyTicket.load(
                std::memory_order_relaxed);
        snapshot.drainRevocationGeneration =
            m_ncResetReleaseAuthDrainGeneration.load(
                std::memory_order_relaxed);
        const std::uint64_t sequenceAfter =
            m_ncResetReleaseAuthWriteSequence.load(
                std::memory_order_acquire);
        if (sequenceBefore != sequenceAfter ||
            (sequenceAfter & 1ULL) != 0ULL)
        {
            return classifyCurrentTuple();
        }
        if (snapshot.IsValid() && authorizationMatches(snapshot))
        {
            return classifyCurrentTuple();
        }

        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }
    if (!authorizationMatches(publishedAuthorization))
    {
        (void)ReleaseNCResetSafetyReleaseAuthorizationClaim();
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }

    const auto releaseAuthorizationClaim = [this]() noexcept
    {
        (void)ReleaseNCResetSafetyReleaseAuthorizationClaim();
    };

    MotionExecutionEpoch exactEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    const ResetSafetyAuthorityStatus exactEpochStatus =
        TryGetResetSafetyMotionOwnerEpoch(
            lease,
            authorization.safetyRequestTicket,
            authorization.drainRevocationGeneration,
            exactEpoch);
    if (exactEpochStatus != ResetSafetyAuthorityStatus::ACQUIRED ||
        exactEpoch != authorization.executionEpoch)
    {
        releaseAuthorizationClaim();
        if (exactEpochStatus == ResetSafetyAuthorityStatus::DEFERRED)
        {
            return classifyCurrentTuple();
        }
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }

    std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if (!UnpackMotionOwnerState(ownerState).Matches(lease) ||
        UnpackMotionOwnerSafetyRequestTicket(ownerState) !=
        authorization.safetyRequestTicket)
    {
        releaseAuthorizationClaim();
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }
    if (UnpackMotionOwnerSafetyHandshake(ownerState) ||
        UnpackMotionOwnerSafetyActionPending(ownerState))
    {
        releaseAuthorizationClaim();
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }
    if ((ownerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
    {
        releaseAuthorizationClaim();
        return classifyCurrentTuple();
    }

    const std::uint64_t reservedOwnerState =
        ownerState | MOTION_OWNER_EPOCH_COMMIT_RESERVED;
    if (!m_motionOwnerState.compare_exchange_strong(
        ownerState,
        reservedOwnerState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        releaseAuthorizationClaim();
        return classifyCurrentTuple();
    }

    const auto releaseReservation = [this,
        reservedOwnerState]() noexcept
    {
        std::uint64_t expected = reservedOwnerState;
        if (!m_motionOwnerState.compare_exchange_strong(
            expected,
            reservedOwnerState &
            ~MOTION_OWNER_EPOCH_COMMIT_RESERVED,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            m_motionOwnerState.fetch_and(
                ~MOTION_OWNER_EPOCH_COMMIT_RESERVED,
                std::memory_order_acq_rel);
        }
    };

    if (m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire) !=
        authorization.drainRevocationGeneration ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        HasPendingSafetyOrRecoveryRequests() ||
        m_safetyControlledStopInProgress)
    {
        releaseReservation();
        releaseAuthorizationClaim();
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }
    if (m_motionOwnerState.load(std::memory_order_acquire) !=
        reservedOwnerState)
    {
        releaseReservation();
        releaseAuthorizationClaim();
        return classifyCurrentTuple();
    }

    const std::uint64_t releasedOwnerState =
        PackMotionOwnerState(
            MotionOwner::NONE,
            NextMotionOwnerGeneration(lease.generation),
            authorization.safetyRequestTicket,
            false,
            false);
    std::uint64_t expectedReservedOwnerState =
        reservedOwnerState;
    if (!m_motionOwnerState.compare_exchange_strong(
        expectedReservedOwnerState,
        releasedOwnerState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        releaseReservation();
        releaseAuthorizationClaim();
        return classifyCurrentTuple();
    }

    // The owner CAS is the release linearization point, but an independent
    // Safety producer can begin while the EPOCH reservation is held: it
    // advances provenance first, fails to replace the reserved owner, and
    // leaves a fail-closed mailbox for RT.  Re-prove the complete tuple after
    // publishing NONE so the caller cannot enter READY or retire the
    // cross-scan zero-output hold across that interleaving.  The released
    // owner is deliberately not restored on failure; the newer Safety work
    // owns the subsequent transition.
    if (m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire) !=
        authorization.drainRevocationGeneration ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        HasPendingSafetyOrRecoveryRequests() ||
        m_safetyControlledStopInProgress ||
        m_motionOwnerState.load(std::memory_order_acquire) !=
        releasedOwnerState)
    {
        releaseAuthorizationClaim();
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }

    if (!ConsumeNCResetSafetyReleaseAuthorizationClaim())
    {
        return SafetyMotionOwnerReleaseStatus::SUPERSEDED;
    }

    return SafetyMotionOwnerReleaseStatus::RELEASED;
}


bool MotionCore::ReleaseSafetyMotionOwner(
    const MotionOwnerLease& lease,
    const MotionNCResetSafetyReleaseAuthorization& authorization) noexcept
{
    return TryReleaseSafetyMotionOwner(lease, authorization) ==
        SafetyMotionOwnerReleaseStatus::RELEASED;
}


bool MotionCore::EnsureSafetyMotionOwnerEpoch(
    const MotionOwnerLease& safetyLease,
    std::uint32_t safetyRequestTicket) noexcept
{
    if (!safetyLease.IsValid() ||
        safetyLease.owner != MotionOwner::SAFETY ||
        safetyRequestTicket == 0U)
    {
        return false;
    }

    const auto PackedGeneration =
        [](std::uint64_t packed) noexcept
        -> MotionOwnerGeneration
    {
        return static_cast<MotionOwnerGeneration>(
            packed >> 32U);
    };
    const auto PackedEpoch =
        [](std::uint64_t packed) noexcept
        -> MotionExecutionEpoch
    {
        return static_cast<MotionExecutionEpoch>(
            packed & 0xFFFFFFFFULL);
    };
    const auto PackGenerationEpoch =
        [](MotionOwnerGeneration generation,
            MotionExecutionEpoch epoch) noexcept -> std::uint64_t
    {
        return
            (static_cast<std::uint64_t>(generation) << 32U) |
            static_cast<std::uint64_t>(epoch);
    };
    const auto IsNewerOwnerGeneration =
        [](MotionOwnerGeneration candidate,
            MotionOwnerGeneration observed) noexcept -> bool
    {
        if (candidate == MOTION_OWNER_GENERATION_INVALID)
        {
            return false;
        }
        if (observed == MOTION_OWNER_GENERATION_INVALID)
        {
            return true;
        }

        // Owner generations advance modulo 2^32. The signed half-range
        // comparison is the standard serial-number ordering and prevents
        // a delayed helper from replacing a later takeover generation.
        return static_cast<std::int32_t>(candidate - observed) > 0;
    };

    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        std::uint64_t ownerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        if (!UnpackMotionOwnerState(ownerState).Matches(safetyLease))
        {
            return false;
        }

        const std::uint32_t currentTicket =
            UnpackMotionOwnerSafetyRequestTicket(ownerState);
        if (currentTicket == 0U ||
            currentTicket == m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire))
        {
            return false;
        }
        safetyRequestTicket = currentTicket;

        std::uint64_t acknowledgement =
            m_safetyOwnerEpochAcknowledgement.load(
                std::memory_order_acquire);
        std::uint64_t publication =
            m_executionEpochPublication.load(
                std::memory_order_acquire);

        // An acknowledged generation is complete only after the same packed
        // Owner state leaves HANDSHAKE. Release rejects HANDSHAKE, so no old
        // helper can publish a SAFETY Epoch after the lease was released.
        if (PackedGeneration(acknowledgement) ==
            safetyLease.generation &&
            PackedEpoch(acknowledgement) !=
            MOTION_EXECUTION_EPOCH_INVALID &&
            UnpackExecutionEpochPublicationSource(publication) ==
            MotionCommandSource::SAFETY)
        {
            if (!UnpackMotionOwnerSafetyHandshake(ownerState))
            {
                return IsMotionOwnerLeaseCurrent(safetyLease);
            }

            const std::uint64_t stableOwnerState =
                ownerState & ~MOTION_OWNER_SAFETY_HANDSHAKE;
            if (m_motionOwnerState.compare_exchange_strong(
                ownerState,
                stableOwnerState,
                std::memory_order_acq_rel,
                std::memory_order_acquire))
            {
                return IsMotionOwnerLeaseCurrent(safetyLease);
            }
            continue;
        }

        if (!UnpackMotionOwnerSafetyHandshake(ownerState))
        {
            return false;
        }

        // Every helper for one owner generation shares one exact expected
        // Epoch claim. They all attempt the same successor CAS, so a delayed
        // helper cannot publish an extra SAFETY Epoch after E-stop evidence
        // has already correlated the first one.
        std::uint64_t claim = m_safetyOwnerEpochClaim.load(
            std::memory_order_acquire);
        if (PackedGeneration(claim) != safetyLease.generation)
        {
            if (!IsNewerOwnerGeneration(
                safetyLease.generation,
                PackedGeneration(claim)))
            {
                return false;
            }

            if ((publication &
                EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
            {
                publication = m_executionEpochPublication.load(
                    std::memory_order_acquire);
                continue;
            }

            const std::uint64_t desiredClaim =
                PackGenerationEpoch(
                    safetyLease.generation,
                    UnpackExecutionEpochPublication(publication));
            if (!m_safetyOwnerEpochClaim.compare_exchange_strong(
                claim,
                desiredClaim,
                std::memory_order_acq_rel,
                std::memory_order_acquire))
            {
                continue;
            }
            claim = desiredClaim;
        }

        const MotionExecutionEpoch expectedEpoch = PackedEpoch(claim);
        if (expectedEpoch == MOTION_EXECUTION_EPOCH_INVALID)
        {
            return false;
        }

        // Re-read both completion and publication immediately before the
        // one-shot CAS. A helper that was descheduled after reading the claim
        // observes the winning helper here and returns without Epoch churn.
        acknowledgement = m_safetyOwnerEpochAcknowledgement.load(
            std::memory_order_acquire);
        if (PackedGeneration(acknowledgement) ==
            safetyLease.generation &&
            PackedEpoch(acknowledgement) !=
            MOTION_EXECUTION_EPOCH_INVALID)
        {
            continue;
        }

        publication = m_executionEpochPublication.load(
            std::memory_order_acquire);
        if ((publication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
        {
            continue;
        }

        MotionExecutionEpoch observedEpoch =
            UnpackExecutionEpochPublication(publication);
        MotionCommandSource observedSource =
            UnpackExecutionEpochPublicationSource(publication);

        if (observedEpoch == expectedEpoch)
        {
            const MotionExecutionEpoch successorEpoch =
                (expectedEpoch ==
                    (std::numeric_limits<MotionExecutionEpoch>::max)())
                ? 1U
                : static_cast<MotionExecutionEpoch>(expectedEpoch + 1U);
            const std::uint64_t desiredPublication =
                PackExecutionEpochPublication(
                    successorEpoch,
                    MotionCommandSource::SAFETY,
                    false,
                    true);

            if (!m_executionEpochPublication.compare_exchange_strong(
                publication,
                desiredPublication,
                std::memory_order_acq_rel,
                std::memory_order_acquire))
            {
                continue;
            }

            observedEpoch = successorEpoch;
            observedSource = MotionCommandSource::SAFETY;
        }
        else if (observedSource != MotionCommandSource::SAFETY)
        {
            // The only ordinary publisher allowed to cross this seam is the
            // already-in-flight one-shot producer. Advance the shared claim
            // to its exact result; every helper now targets one common SAFETY
            // successor rather than independently incrementing the Epoch.
            const std::uint64_t advancedClaim =
                PackGenerationEpoch(
                    safetyLease.generation,
                    observedEpoch);
            (void)m_safetyOwnerEpochClaim.compare_exchange_strong(
                claim,
                advancedClaim,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            continue;
        }

        ownerState = m_motionOwnerState.load(std::memory_order_acquire);
        if (!UnpackMotionOwnerState(ownerState).Matches(safetyLease) ||
            !UnpackMotionOwnerSafetyHandshake(ownerState) ||
            observedSource != MotionCommandSource::SAFETY)
        {
            return false;
        }

        const std::uint64_t desiredAcknowledgement =
            PackGenerationEpoch(
                safetyLease.generation,
                observedEpoch);
        acknowledgement = m_safetyOwnerEpochAcknowledgement.load(
            std::memory_order_acquire);
        if (IsNewerOwnerGeneration(
            safetyLease.generation,
            PackedGeneration(acknowledgement)))
        {
            (void)m_safetyOwnerEpochAcknowledgement.compare_exchange_strong(
                acknowledgement,
                desiredAcknowledgement,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
        }
    }

    return false;
}


std::uint32_t MotionCore::PublishSafetyMotionRequestTicket(
    bool requiresRTApplication) noexcept
{
    std::uint64_t observed =
        m_motionOwnerState.load(std::memory_order_acquire);
    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        if ((observed &
            (MOTION_OWNER_FRAME_SEND_RESERVED |
                MOTION_OWNER_EPOCH_COMMIT_RESERVED)) != 0ULL)
        {
            // The non-zero process-data frame owns the physical send seam.
            // The request has not linearized yet; its caller retains the
            // revocation window/mailbox and the next bounded attempt/pass
            // publishes after EndServoOutputFrameAtSendPoint releases it.
            return 0U;
        }
        const MotionOwnerLease lease = UnpackMotionOwnerState(observed);
        const std::uint32_t previous =
            UnpackMotionOwnerSafetyRequestTicket(observed);

        // Every BeginRevocation window performs a value-changing CAS on this
        // same packed word, including pure authority helpers. This is the
        // cross-thread ordering edge which prevents a parallel Release/ACK
        // from passing a producer paused before its takeover/mutation.
        const std::uint32_t next =
            previous >= MOTION_OWNER_SAFETY_TICKET_MAX
            ? 1U
            : previous + 1U;
        const bool actionPending =
            UnpackMotionOwnerSafetyActionPending(observed) ||
            requiresRTApplication;
        const std::uint64_t desired =
            PackMotionOwnerState(
                lease.owner,
                lease.generation,
                next,
                UnpackMotionOwnerSafetyHandshake(observed),
                actionPending) |
            (observed & MOTION_OWNER_OUTPUT_COMMIT_RESERVED);
        if (m_motionOwnerState.compare_exchange_strong(
            observed,
            desired,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return next;
        }
    }
    return 0U;
}


bool MotionCore::EnsureSafetyMotionActionTicket(
    std::uint32_t& safetyRequestTicket) noexcept
{
    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint32_t currentTicket =
        UnpackMotionOwnerSafetyRequestTicket(ownerState);
    const std::uint32_t acknowledged =
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire);

    if (currentTicket != 0U &&
        currentTicket != acknowledged &&
        UnpackMotionOwnerSafetyActionPending(ownerState))
    {
        // Every caller holds the drain-revocation publisher count.  Therefore
        // an existing unacknowledged ACTION ticket cannot be completed while
        // this producer is still publishing its mailbox/direct mutation, and
        // it is safe to join that exact incident without a second CAS loop.
        safetyRequestTicket = currentTicket;
        return true;
    }

    safetyRequestTicket = 0U;
    return false;
}


bool MotionCore::CompleteSafetyMotionActionTicket(
    std::uint32_t safetyRequestTicket) noexcept
{
    if (safetyRequestTicket == 0U)
    {
        return false;
    }

    std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        if (UnpackMotionOwnerSafetyRequestTicket(ownerState) !=
            safetyRequestTicket)
        {
            // A newer producer now owns ACTION_PENDING. Never clear its bit;
            // this producer is still covered by the revocation count until
            // it returns, and any deferred mailbox it published is itself an
            // acknowledgement fence.
            return false;
        }

        if (!UnpackMotionOwnerSafetyActionPending(ownerState))
        {
            return true;
        }

        const std::uint64_t completedOwnerState =
            ownerState & ~MOTION_OWNER_SAFETY_ACTION_PENDING;
        if (m_motionOwnerState.compare_exchange_strong(
            ownerState,
            completedOwnerState,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return true;
        }
    }

    return false;
}


bool MotionCore::HasUnacknowledgedSafetyMotionRequest() const noexcept
{
    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint32_t ticket =
        UnpackMotionOwnerSafetyRequestTicket(ownerState);
    return
        (ticket != 0U &&
            ticket != m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire)) ||
        UnpackMotionOwnerSafetyActionPending(ownerState);
}


MotionOwnerLease MotionCore::TryTakeSafetyMotionOwnerForTicket(
    std::uint32_t safetyRequestTicket) noexcept
{
    MotionOwnerLease invalidLease{};
    if (safetyRequestTicket == 0U)
    {
        return invalidLease;
    }

    std::uint64_t currentPacked =
        m_motionOwnerState.load(std::memory_order_acquire);
    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        const std::uint32_t currentTicket =
            UnpackMotionOwnerSafetyRequestTicket(currentPacked);
        if ((currentPacked &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
        {
            return invalidLease;
        }
        if (currentTicket == 0U ||
            currentTicket == m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire))
        {
            return invalidLease;
        }
        safetyRequestTicket = currentTicket;

        const MotionOwnerLease currentLease =
            UnpackMotionOwnerState(currentPacked);
        if (currentLease.owner == MotionOwner::SAFETY &&
            currentLease.generation != MOTION_OWNER_GENERATION_INVALID)
        {
            if (!UnpackMotionOwnerSafetyHandshake(currentPacked))
            {
                return currentLease;
            }
            if (EnsureSafetyMotionOwnerEpoch(
                currentLease,
                safetyRequestTicket))
            {
                return currentLease;
            }
            // EnsureSafetyMotionOwnerEpoch has its own fixed budget.  Do not
            // nest it inside all outer takeover attempts; the next 250 us
            // pass can safely resume the still-visible handshake.
            return invalidLease;
        }

        MotionOwnerLease safetyLease{};
        safetyLease.owner = MotionOwner::SAFETY;
        safetyLease.generation =
            NextMotionOwnerGeneration(currentLease.generation);
        const std::uint64_t safetyPacked =
            PackMotionOwnerState(
                safetyLease.owner,
                safetyLease.generation,
                safetyRequestTicket,
                true,
                UnpackMotionOwnerSafetyActionPending(currentPacked));
        if (m_motionOwnerState.compare_exchange_strong(
            currentPacked,
            safetyPacked,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            if (EnsureSafetyMotionOwnerEpoch(
                safetyLease,
                safetyRequestTicket))
            {
                return safetyLease;
            }
            return invalidLease;
        }
    }
    return invalidLease;
}


void MotionCore::BeginExecutionDrainAcknowledgementRevocation() noexcept
{
    m_executionDrainRevocationPublishersInProgress.fetch_add(
        1U,
        std::memory_order_acq_rel);
    m_frameSafetyIntentState.fetch_add(
        FRAME_SAFETY_INTENT_BEGIN_DELTA,
        std::memory_order_acq_rel);
    // Count closes the J.5 acknowledgement seam first; the packed state then
    // closes the whole-PDO seam; generation remains the exact settle token.
    m_executionDrainRevocationGeneration.fetch_add(
        1ULL,
        std::memory_order_acq_rel);
}


bool MotionCore::BeginResetSafetyProvenanceOperation(
    std::uint64_t expectedProvenanceGeneration,
    std::uint64_t& operationProvenanceGeneration) noexcept
{
    m_executionDrainRevocationPublishersInProgress.fetch_add(
        1U,
        std::memory_order_acq_rel);
    m_frameSafetyIntentState.fetch_add(
        FRAME_SAFETY_INTENT_BEGIN_DELTA,
        std::memory_order_acq_rel);

    const std::uint64_t previousProvenanceGeneration =
        m_executionDrainRevocationGeneration.fetch_add(
            1ULL,
            std::memory_order_acq_rel);
    operationProvenanceGeneration =
        previousProvenanceGeneration + 1ULL;
    return
        previousProvenanceGeneration ==
        expectedProvenanceGeneration;
}


void MotionCore::EndExecutionDrainAcknowledgementRevocation() noexcept
{
    m_executionDrainRevocationPublishersInProgress.fetch_sub(
        1U,
        std::memory_order_release);
    m_frameSafetyIntentState.fetch_sub(
        1ULL,
        std::memory_order_release);
}


std::uint64_t MotionCore::BeginResetSafetyOutputHold() noexcept
{
    // Keep the active-count half nonzero across 10 ms scans.  The sequence
    // half changes at the same RMW, so a frame which began immediately before
    // this call also fails its final send-point comparison and is scrubbed.
    m_frameSafetyIntentState.fetch_add(
        FRAME_SAFETY_INTENT_BEGIN_DELTA,
        std::memory_order_acq_rel);
    return m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire);
}


bool MotionCore::IsResetSafetyOutputHoldEstablished() const noexcept
{
    const std::uint64_t safetyIntentState =
        m_frameSafetyIntentState.load(std::memory_order_acquire);
    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);

    // A frame whose send-point reservation is still held has already passed
    // Finalize but may not yet have reached the NIC.  Keep the operator edge
    // deferred until EndServoOutputFrameAfterSend() retires that transaction.
    // The active output hold installed before this check prevents every later
    // non-zero frame from acquiring a new reservation; a contender which read
    // the old state just before the hold will fail its final safety-state
    // proof and be scrubbed.
    return
        static_cast<std::uint32_t>(safetyIntentState) != 0U &&
        (ownerState & MOTION_OWNER_FRAME_SEND_RESERVED) == 0ULL;
}


void MotionCore::EndResetSafetyOutputHold() noexcept
{
    // Leave the sequence half advanced; only retire this Reset's one active
    // hold.  The caller owns the matching begin/end lifetime.
    m_frameSafetyIntentState.fetch_sub(
        1ULL,
        std::memory_order_release);
}


std::uint64_t MotionCore::MarkResetSafetyOperatorEdge() noexcept
{
    // This short, balanced publisher is the exact linearization point for
    // every explicit Reset edge, including a new edge which reuses an
    // already-active cross-scan output hold after terminal BLOCKED.
    m_executionDrainRevocationPublishersInProgress.fetch_add(
        1U,
        std::memory_order_acq_rel);
    m_frameSafetyIntentState.fetch_add(
        FRAME_SAFETY_INTENT_BEGIN_DELTA,
        std::memory_order_acq_rel);
    const std::uint64_t generation =
        m_executionDrainRevocationGeneration.fetch_add(
            1ULL,
            std::memory_order_acq_rel) + 1ULL;
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
    return generation;
}


std::uint64_t MotionCore::GetSafetyProvenanceGeneration() const noexcept
{
    return m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire);
}


void MotionCore::RevokeExecutionDrainAcknowledgement() noexcept
{
    BeginExecutionDrainAcknowledgementRevocation();
    EndExecutionDrainAcknowledgementRevocation();
}


MotionOwnerLease MotionCore::TakeSafetyMotionOwner() noexcept
{
    BeginExecutionDrainAcknowledgementRevocation();
    const std::uint32_t ticket =
        PublishSafetyMotionRequestTicket(false);
    const MotionOwnerLease lease =
        TryTakeSafetyMotionOwnerForTicket(ticket);
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
    return lease;
}


MotionOwnerLease MotionCore::BeginNewSafetyMotionOwnerGeneration() noexcept
{
    const MotionOwnerGeneration entryGeneration =
        UnpackMotionOwnerState(
            m_motionOwnerState.load(
                std::memory_order_acquire)).generation;
    return ContinueNewSafetyMotionOwnerGeneration(entryGeneration);
}


MotionOwnerLease MotionCore::ContinueNewSafetyMotionOwnerGeneration(
    MotionOwnerGeneration entryGeneration) noexcept
{
    BeginExecutionDrainAcknowledgementRevocation();
    MotionOwnerLease result{};
    std::uint64_t currentPacked =
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease entryObservedLease =
        UnpackMotionOwnerState(currentPacked);
    if (entryObservedLease.owner == MotionOwner::SAFETY &&
        entryObservedLease.generation != entryGeneration &&
        !UnpackMotionOwnerSafetyHandshake(currentPacked) &&
        (currentPacked & MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL &&
        IsMotionOwnerLeaseCurrent(entryObservedLease))
    {
        EndExecutionDrainAcknowledgementRevocation();
        TryAcknowledgeAppliedSafetyMotionRequests();
        return entryObservedLease;
    }
    if (entryObservedLease.generation != entryGeneration &&
        entryObservedLease.owner != MotionOwner::SAFETY)
    {
        EndExecutionDrainAcknowledgementRevocation();
        return result;
    }

    std::uint32_t ticket =
        UnpackMotionOwnerSafetyRequestTicket(currentPacked);
    if (ticket == 0U ||
        ticket == m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire))
    {
        ticket = PublishSafetyMotionRequestTicket(false);
        currentPacked = m_motionOwnerState.load(
            std::memory_order_acquire);
    }

    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        ticket = UnpackMotionOwnerSafetyRequestTicket(currentPacked);
        if (ticket == 0U ||
            ticket == m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire))
        {
            ticket = PublishSafetyMotionRequestTicket(false);
            currentPacked = m_motionOwnerState.load(
                std::memory_order_acquire);
            continue;
        }

        if ((currentPacked &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
        {
            currentPacked = m_motionOwnerState.load(
                std::memory_order_acquire);
            continue;
        }

        const MotionOwnerLease currentLease =
            UnpackMotionOwnerState(currentPacked);

        // The continuation may join a newer SAFETY takeover, but it must
        // never mint yet another generation after an unrelated owner has
        // superseded the original Reset baseline.
        if (currentLease.generation != entryGeneration &&
            currentLease.owner != MotionOwner::SAFETY)
        {
            break;
        }

        // A concurrent takeover which already advanced beyond our entry
        // generation satisfies the freshness contract only after its exact
        // handshake/Epoch publication is complete. Never fall back to the
        // idempotent same-generation Take path here.
        if (currentLease.owner == MotionOwner::SAFETY &&
            currentLease.generation != entryGeneration)
        {
            if (EnsureSafetyMotionOwnerEpoch(currentLease, ticket))
            {
                result = currentLease;
                break;
            }
            currentPacked = m_motionOwnerState.load(
                std::memory_order_acquire);
            continue;
        }

        if (UnpackMotionOwnerSafetyHandshake(currentPacked))
        {
            if (currentLease.owner == MotionOwner::SAFETY)
            {
                (void)EnsureSafetyMotionOwnerEpoch(
                    currentLease,
                    ticket);
            }
            currentPacked = m_motionOwnerState.load(
                std::memory_order_acquire);
            continue;
        }

        MotionOwnerLease safetyLease{};
        safetyLease.owner = MotionOwner::SAFETY;
        safetyLease.generation =
            NextMotionOwnerGeneration(currentLease.generation);
        const std::uint64_t safetyPacked =
            PackMotionOwnerState(
                safetyLease.owner,
                safetyLease.generation,
                ticket,
                true,
                UnpackMotionOwnerSafetyActionPending(currentPacked));
        if (m_motionOwnerState.compare_exchange_strong(
            currentPacked,
            safetyPacked,
            std::memory_order_acq_rel,
            std::memory_order_acquire) &&
            EnsureSafetyMotionOwnerEpoch(safetyLease, ticket))
        {
            result = safetyLease;
            break;
        }
    }

    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
    return result;
}


MotionCore::ResetSafetyAuthorityResult
MotionCore::TryCaptureResetSafetyAuthorityBaseline(
    std::uint64_t expectedProvenanceGeneration) const noexcept
{
    ResetSafetyAuthorityResult result{};
    result.provenanceGeneration = expectedProvenanceGeneration;

    const std::uint64_t entryProvenance =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    if (entryProvenance != expectedProvenanceGeneration)
    {
        result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        return result;
    }

    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint32_t ticket =
        UnpackMotionOwnerSafetyRequestTicket(ownerState);
    const std::uint32_t acknowledgedTicket =
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire);
    const std::uint64_t executionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const bool hasPendingSafetyOrRecovery =
        HasPendingSafetyOrRecoveryRequests();

    if (m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire) != entryProvenance)
    {
        result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        return result;
    }

    if (m_motionOwnerState.load(std::memory_order_acquire) != ownerState ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        ticket != acknowledgedTicket ||
        UnpackMotionOwnerSafetyHandshake(ownerState) ||
        UnpackMotionOwnerSafetyActionPending(ownerState) ||
        (ownerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        (executionPublication &
            (EXECUTION_EPOCH_PUBLICATION_PENDING |
                EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL ||
        hasPendingSafetyOrRecovery)
    {
        return result;
    }

    result.lease = UnpackMotionOwnerState(ownerState);
    result.requestTicket = ticket;
    result.status = ResetSafetyAuthorityStatus::ACQUIRED;
    return result;
}


MotionCore::ResetSafetyAuthorityResult
MotionCore::ContinueResetSafetyMotionOwnerGeneration(
    MotionOwnerGeneration entryGeneration,
    std::uint32_t baselineTicket,
    std::uint32_t requestTicket,
    std::uint64_t expectedProvenanceGeneration) noexcept
{
    ResetSafetyAuthorityResult result{};
    result.requestTicket = requestTicket;

    if (!BeginResetSafetyProvenanceOperation(
        expectedProvenanceGeneration,
        result.provenanceGeneration))
    {
        result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        EndExecutionDrainAcknowledgementRevocation();
        TryAcknowledgeAppliedSafetyMotionRequests();
        return result;
    }

    const MotionOwnerGeneration targetGeneration =
        NextMotionOwnerGeneration(entryGeneration);
    MotionOwnerLease resetSafetyLease{};
    resetSafetyLease.owner = MotionOwner::SAFETY;
    resetSafetyLease.generation = targetGeneration;

    // The first successful operation performs one indivisible transition
    // from the exact drained baseline to RESET-owned SAFETY authority.  No
    // ticket-only state is ever observable, so RT cannot acknowledge a Reset
    // request before its target generation/handshake exists.
    if (result.requestTicket == 0U)
    {
        std::uint64_t baselineState =
            m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease baselineLease =
            UnpackMotionOwnerState(baselineState);
        const std::uint32_t observedBaselineTicket =
            UnpackMotionOwnerSafetyRequestTicket(baselineState);
        const std::uint32_t acknowledgedTicket =
            m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire);

        if (baselineLease.generation != entryGeneration ||
            observedBaselineTicket != baselineTicket ||
            acknowledgedTicket != baselineTicket ||
            UnpackMotionOwnerSafetyHandshake(baselineState) ||
            UnpackMotionOwnerSafetyActionPending(baselineState))
        {
            result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        }
        else if ((baselineState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL &&
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) ==
            result.provenanceGeneration &&
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) == 1U)
        {
            const std::uint32_t nextTicket =
                baselineTicket >= MOTION_OWNER_SAFETY_TICKET_MAX
                ? 1U
                : baselineTicket + 1U;
            const std::uint64_t resetSafetyState =
                PackMotionOwnerState(
                    resetSafetyLease.owner,
                    resetSafetyLease.generation,
                    nextTicket,
                    true,
                    false);
            std::uint64_t expectedBaselineState = baselineState;
            if (m_motionOwnerState.compare_exchange_strong(
                expectedBaselineState,
                resetSafetyState,
                std::memory_order_acq_rel,
                std::memory_order_acquire))
            {
                result.requestTicket = nextTicket;
            }
            else
            {
                const MotionOwnerLease observedLease =
                    UnpackMotionOwnerState(expectedBaselineState);
                const std::uint32_t observedTicket =
                    UnpackMotionOwnerSafetyRequestTicket(
                        expectedBaselineState);
                if (observedLease.generation != entryGeneration ||
                    observedTicket != baselineTicket ||
                    m_executionDrainRevocationGeneration.load(
                        std::memory_order_acquire) !=
                    result.provenanceGeneration)
                {
                    result.status =
                        ResetSafetyAuthorityStatus::SUPERSEDED;
                }
            }
        }
    }

    if (result.status != ResetSafetyAuthorityStatus::SUPERSEDED &&
        result.requestTicket != 0U)
    {
        const std::uint64_t currentState =
            m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease currentLease =
            UnpackMotionOwnerState(currentState);
        const std::uint32_t currentTicket =
            UnpackMotionOwnerSafetyRequestTicket(currentState);

        if (!currentLease.Matches(resetSafetyLease) ||
            currentTicket != result.requestTicket ||
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) !=
            result.provenanceGeneration)
        {
            result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        }
        else
        {
            MotionExecutionEpoch exactEpoch =
                MOTION_EXECUTION_EPOCH_INVALID;
            const ResetSafetyAuthorityStatus exactStatus =
                TryGetResetSafetyMotionOwnerEpoch(
                    resetSafetyLease,
                    result.requestTicket,
                    result.provenanceGeneration,
                    exactEpoch);
            if (exactStatus == ResetSafetyAuthorityStatus::ACQUIRED)
            {
                result.lease = resetSafetyLease;
                result.status = ResetSafetyAuthorityStatus::ACQUIRED;
            }
            else if (exactStatus ==
                ResetSafetyAuthorityStatus::SUPERSEDED)
            {
                result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
            }
            else if ((currentState &
                MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL &&
                UnpackMotionOwnerSafetyHandshake(currentState) &&
                EnsureSafetyMotionOwnerEpoch(
                    resetSafetyLease,
                    result.requestTicket))
            {
                const std::uint64_t afterEnsure =
                    m_motionOwnerState.load(std::memory_order_acquire);
                if (UnpackMotionOwnerState(afterEnsure).Matches(
                    resetSafetyLease) &&
                    UnpackMotionOwnerSafetyRequestTicket(afterEnsure) ==
                    result.requestTicket &&
                    m_executionDrainRevocationGeneration.load(
                        std::memory_order_acquire) ==
                    result.provenanceGeneration)
                {
                    result.lease = resetSafetyLease;
                    result.status =
                        ResetSafetyAuthorityStatus::ACQUIRED;
                }
                else
                {
                    result.status =
                        ResetSafetyAuthorityStatus::SUPERSEDED;
                }
            }
        }
    }

    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
    return result;
}


void MotionCore::TryAcknowledgeAppliedSafetyMotionRequests() noexcept
{
    const auto hasUnappliedSafetyMailbox = [this]() noexcept -> bool
    {
        return
            HasPendingExecutionEpochChange() ||
            (m_emergencyStopRequestPublication.load(
                std::memory_order_acquire) &
                EMERGENCY_STOP_REQUEST_PENDING) != 0ULL ||
            m_resetAllFaultsPending.load(std::memory_order_acquire) ||
            m_stopGroupPending.load(std::memory_order_acquire) ||
            m_resetSafetyBatchPending.load(
                std::memory_order_acquire) != 0ULL ||
            m_axisFaultResetPendingMask.load(
                std::memory_order_acquire) != 0U;
    };

    if (m_executionDrainRevocationPublishersInProgress.load(
        std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        m_safetyControlledStopInProgress ||
        hasUnappliedSafetyMailbox())
    {
        return;
    }

    std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    MotionOwnerLease entryLease =
        UnpackMotionOwnerState(entryOwnerState);
    const std::uint32_t ticket =
        UnpackMotionOwnerSafetyRequestTicket(entryOwnerState);
    if (entryLease.owner != MotionOwner::SAFETY ||
        !entryLease.IsValid() ||
        UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
        (entryOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        ticket == 0U)
    {
        return;
    }

    if (UnpackMotionOwnerSafetyActionPending(entryOwnerState))
    {
        // BeginRevocation is sequenced before every action-ticket CAS. The
        // acquire of that same packed word therefore makes the producer
        // count visible here. Only a genuinely orphaned completion bit may
        // be cleared after all publishers/mailboxes/direct mutations have
        // left their explicit gates.
        if (m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
            m_safetyRecoveryRequestInProgress.load(
                std::memory_order_acquire) ||
            m_safetyControlledStopInProgress ||
            hasUnappliedSafetyMailbox() ||
            !CompleteSafetyMotionActionTicket(ticket))
        {
            return;
        }

        entryOwnerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        entryLease = UnpackMotionOwnerState(entryOwnerState);
        if (entryLease.owner != MotionOwner::SAFETY ||
            !entryLease.IsValid() ||
            UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
            UnpackMotionOwnerSafetyActionPending(entryOwnerState) ||
            (entryOwnerState &
                MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
            UnpackMotionOwnerSafetyRequestTicket(entryOwnerState) !=
            ticket)
        {
            return;
        }
    }

    // Only advance from the exact acknowledgement observed for this helper.
    // A delayed helper must never overwrite a later incident's ACK with an
    // older ticket after wrap/coalescing or concurrent RT/NC observation.
    std::uint32_t expectedAcknowledgement =
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire);
    if (expectedAcknowledgement == ticket)
    {
        return;
    }

    if (m_motionOwnerState.load(std::memory_order_acquire) !=
        entryOwnerState ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        m_safetyControlledStopInProgress ||
        hasUnappliedSafetyMailbox())
    {
        return;
    }

    if (!m_safetyRequestAcknowledgedTicket.compare_exchange_strong(
        expectedAcknowledgement,
        ticket,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return;
    }

    // A request that races the acknowledgement changes the same packed Owner
    // word first. Its new ticket therefore remains outstanding and prevents
    // release even though the older ticket was just acknowledged.
    const std::uint64_t exitOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if (exitOwnerState != entryOwnerState ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        m_safetyControlledStopInProgress ||
        hasUnappliedSafetyMailbox())
    {
        return;
    }
}


bool MotionCore::IsSafetyControlledStopAuthorized(
    int contextAxisSlot) const noexcept
{
    if (!m_safetyControlledStopInProgress ||
        contextAxisSlot < 0 ||
        m_pContexts == nullptr ||
        contextAxisSlot >= static_cast<int>(m_pContexts->size()) ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        m_safetyRecoveryRequestInProgress.load(
            std::memory_order_acquire) ||
        (m_emergencyStopRequestPublication.load(
            std::memory_order_acquire) &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL ||
        m_resetAllFaultsPending.load(std::memory_order_acquire) ||
        m_stopGroupPending.load(std::memory_order_acquire) ||
        m_resetSafetyBatchPending.load(
            std::memory_order_acquire) != 0ULL ||
        m_axisFaultResetPendingMask.load(
            std::memory_order_acquire) != 0U ||
        (m_p1MappingIntegrityAlarmRequestPublication.load(
            std::memory_order_acquire) &
            P1_MAPPING_ALARM_PENDING) != 0ULL ||
        !IsMotionOwnerLeaseCurrent(
            m_safetyControlledStopOwnerLease))
    {
        return false;
    }

    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if (UnpackMotionOwnerSafetyHandshake(ownerState) ||
        UnpackMotionOwnerSafetyActionPending(ownerState) ||
        UnpackMotionOwnerSafetyRequestTicket(ownerState) !=
        m_safetyControlledStopRequestTicket ||
        UnpackMotionOwnerState(ownerState).owner != MotionOwner::SAFETY)
    {
        return false;
    }

    const std::uint64_t publication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    if ((publication &
        (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL ||
        UnpackExecutionEpochPublication(publication) !=
        m_safetyControlledStopEpoch ||
        UnpackExecutionEpochPublicationSource(publication) !=
        MotionCommandSource::SAFETY)
    {
        return false;
    }

    const int safeGroupAxisCount =
        ClampMotionAxisCount(m_Group.axisCount);
    for (int slot = 0; slot < safeGroupAxisCount; ++slot)
    {
        if (m_Group.axisIndices[slot] == contextAxisSlot)
        {
            return true;
        }
    }
    return false;
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


bool MotionCore::TryPublishEmergencyStopMailbox(
    MotionExecutionEpoch causalExecutionEpoch,
    bool evidenceRequired) noexcept
{
    std::uint64_t requestPublication =
        m_emergencyStopRequestPublication.load(
            std::memory_order_acquire);
    for (unsigned attempt = 0U;
        attempt < MOTION_OWNER_BOUNDED_CAS_ATTEMPTS;
        ++attempt)
    {
        if ((requestPublication &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL)
        {
            // A direct RT containment that already stopped active execution
            // must not lose its causal P->H evidence behind an older ordinary
            // deferred request. Upgrade that pending request once, preserving
            // the first already-required incident thereafter.
            if (!evidenceRequired ||
                UnpackEmergencyStopRequestEvidenceRequired(
                    requestPublication))
            {
                return false;
            }

            const std::uint64_t upgradedRequest =
                PackEmergencyStopRequest(
                    causalExecutionEpoch,
                    true);
            if (m_emergencyStopRequestPublication.compare_exchange_strong(
                requestPublication,
                upgradedRequest,
                std::memory_order_acq_rel,
                std::memory_order_acquire))
            {
                return false;
            }
            continue;
        }

        const std::uint64_t desiredRequest =
            PackEmergencyStopRequest(
                causalExecutionEpoch,
                evidenceRequired);
        if (m_emergencyStopRequestPublication.compare_exchange_strong(
            requestPublication,
            desiredRequest,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return true;
        }
    }

    return PublishEmergencyStopMailboxContentionFallback(
        causalExecutionEpoch,
        evidenceRequired);
}


bool MotionCore::PublishEmergencyStopMailboxContentionFallback(
    MotionExecutionEpoch causalExecutionEpoch,
    bool evidenceRequired) noexcept
{
    if (!evidenceRequired)
    {
        // An ordinary stop needs a visible physical-action fence, not an
        // exact P->H evidence edge.  Never fabricate EVIDENCE_REQUIRED with
        // an epoch-zero or unrelated payload after the bounded CAS budget.
        const std::uint64_t previous =
            m_emergencyStopRequestPublication.fetch_or(
                EMERGENCY_STOP_REQUEST_PENDING,
                std::memory_order_acq_rel);
        return
            (previous & EMERGENCY_STOP_REQUEST_PENDING) == 0ULL;
    }

    // Bit 62 is a same-word publication reservation.  The first exact-
    // evidence producer closes the consumer claim seam in one bounded RMW,
    // then publishes its complete causal Epoch.  The 250 us consumer never
    // waits: it simply defers a RESERVED mailbox to the following pass.
    const std::uint64_t previous =
        m_emergencyStopRequestPublication.fetch_or(
            EMERGENCY_STOP_REQUEST_PENDING |
            EMERGENCY_STOP_REQUEST_EVIDENCE_REQUIRED |
            EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED,
            std::memory_order_acq_rel);

    if ((previous &
        EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED) != 0ULL)
    {
        // First reserved evidence incident wins.  A second publisher must
        // not overwrite its exact causal Epoch.
        return false;
    }

    if ((previous & EMERGENCY_STOP_REQUEST_PENDING) != 0ULL &&
        (previous &
            EMERGENCY_STOP_REQUEST_EVIDENCE_REQUIRED) != 0ULL)
    {
        m_emergencyStopRequestPublication.fetch_and(
            ~EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED,
            std::memory_order_release);
        return false;
    }

    // Empty or ordinary content is now protected from both consumer claims
    // and stale producer CAS.  Publish the mapping incident's exact P and
    // release the reservation atomically with this complete word.
    m_emergencyStopRequestPublication.store(
        PackEmergencyStopRequest(
            causalExecutionEpoch,
            true),
        std::memory_order_release);
    return
        (previous & EMERGENCY_STOP_REQUEST_PENDING) == 0ULL;
}


bool MotionCore::TryClaimEmergencyStopMailbox(
    std::uint64_t& claimedRequest) noexcept
{
    claimedRequest = 0ULL;
    std::uint64_t observed =
        m_emergencyStopRequestPublication.load(
            std::memory_order_acquire);
    if ((observed & EMERGENCY_STOP_REQUEST_PENDING) == 0ULL ||
        (observed &
            EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED) != 0ULL)
    {
        return false;
    }

    if (!m_emergencyStopRequestPublication.compare_exchange_strong(
        observed,
        0ULL,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    claimedRequest = observed;
    return true;
}


bool MotionCore::TryClaimEmergencyStopMailboxForDirectContainment(
    std::uint64_t& claimedRequest,
    bool& evidenceReservationHeld) noexcept
{
    claimedRequest = 0ULL;
    evidenceReservationHeld = false;

    std::uint64_t observed =
        m_emergencyStopRequestPublication.load(
            std::memory_order_acquire);
    if ((observed & EMERGENCY_STOP_REQUEST_PENDING) == 0ULL ||
        (observed &
            EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED) != 0ULL)
    {
        return false;
    }

    claimedRequest = observed;
    if (UnpackEmergencyStopRequestEvidenceRequired(observed))
    {
        const std::uint64_t reserved =
            observed |
            EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED;
        if (!m_emergencyStopRequestPublication.compare_exchange_strong(
            observed,
            reserved,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            claimedRequest = 0ULL;
            return false;
        }
        evidenceReservationHeld = true;
        return true;
    }

    if (!m_emergencyStopRequestPublication.compare_exchange_strong(
        observed,
        0ULL,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        claimedRequest = 0ULL;
        return false;
    }
    return true;
}


bool MotionCore::FinalizeDirectContainmentEmergencyStopMailbox(
    std::uint64_t claimedRequest,
    bool evidenceAlreadyPersistent) noexcept
{
    if ((claimedRequest & EMERGENCY_STOP_REQUEST_PENDING) == 0ULL ||
        !UnpackEmergencyStopRequestEvidenceRequired(claimedRequest))
    {
        return false;
    }

    std::uint64_t expected =
        claimedRequest |
        EMERGENCY_STOP_REQUEST_PUBLISH_RESERVED;
    const std::uint64_t desired =
        evidenceAlreadyPersistent
        ? 0ULL
        : claimedRequest;
    return m_emergencyStopRequestPublication.compare_exchange_strong(
        expected,
        desired,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

void MotionCore::RequestEmergencyStopAllAxes() noexcept
{
    // A real emergency event always wins over the benign RESET smooth-stop
    // ingress.  Publish that outcome before taking the regular Safety ticket
    // so the NC RESET continuation can never mistake containment for a
    // physically-proved deceleration completion.
    SupersedeResetControlledStop();

    m_emergencyStopRequestAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    BeginExecutionDrainAcknowledgementRevocation();
    // Capture the causal predecessor before publishing the takeover ticket.
    // A concurrent 250 us helper may immediately publish the SAFETY
    // successor once the packed ticket becomes visible.
    const MotionExecutionEpoch causalEpoch =
        GetCurrentExecutionEpoch();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    const bool published =
        TryPublishEmergencyStopMailbox(causalEpoch);

    if (!published)
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
    // Every safety mailbox shares one ticketed incident. Repeated level-
    // sensitive requests can advance the ticket, but never refresh Owner
    // Generation or Epoch while SAFETY already owns the incident.
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
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
    // AL3021 is an Alarm/E-stop containment path, never a continuation of an
    // operator RESET controlled stop.
    SupersedeResetControlledStop();

    BeginExecutionDrainAcknowledgementRevocation();
    const MotionExecutionEpoch causalEpoch =
        GetCurrentExecutionEpoch();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);

    // The 250 us containment may run before the 10 ms NC observer. Publish a
    // formal Alarm first so J.6.3.2 can correlate the pre-latched E-stop and
    // SAFETY ownership is released only through the normal Reset lifecycle.
    TryPublishGroupMappingIntegrityAlarmRequest(
        causalEpoch);

    // This is already the RT consumer.  When the ticket-bound SAFETY
    // takeover completes now, apply and retire the action directly without
    // leaving an emergency-stop mailbox behind for the next RT pass.  When
    // an output/Epoch reservation temporarily blocks that takeover, keep one
    // evidence-required mailbox so the later handshake can publish the exact
    // causal P->H edge after the immediate physical containment below.
    m_emergencyStopRequestAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    const MotionOwnerLease directSafetyLease =
        TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    const bool directSafetyAuthority =
        directSafetyLease.IsValid() &&
        directSafetyLease.owner == MotionOwner::SAFETY &&
        IsMotionOwnerLeaseCurrent(directSafetyLease);
    std::uint64_t claimedEmergencyStopRequest = 0ULL;
    bool directEvidenceReservationHeld = false;
    bool directMailboxClaimed = false;
    if (directSafetyAuthority)
    {
        // The direct mapping-integrity stop subsumes an older deferred
        // all-axis E-stop.  Exact P->H evidence is never exchanged to zero:
        // reserve that complete word in place until this direct stop proves
        // the same evidence is already persistent or restores it unchanged.
        directMailboxClaimed =
            TryClaimEmergencyStopMailboxForDirectContainment(
                claimedEmergencyStopRequest,
                directEvidenceReservationHeld);
    }
    if (directSafetyAuthority)
    {
        m_emergencyStopRequestPublishedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        if (TryPublishEmergencyStopMailbox(causalEpoch, true))
        {
            m_emergencyStopRequestPublishedCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_emergencyStopRequestCoalescedCount.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
    }
    EmergencyStopAllAxesImpl(
        forceExecutionInvalidation,
        causalEpoch);
    if (directSafetyAuthority)
    {
        bool directMailboxFinalized = true;
        if (directEvidenceReservationHeld)
        {
            const MotionExecutionEpoch claimedCausalEpoch =
                UnpackEmergencyStopRequestEpoch(
                    claimedEmergencyStopRequest);
            const MotionExecutionEpoch currentSafetyEpoch =
                GetCurrentExecutionEpoch();
            const MotionEmergencyStopEpochInvalidationEvidence evidence =
                GetEmergencyStopEpochInvalidationEvidence();
            const bool claimedEvidenceAlreadyPersistent =
                evidence.fromExecutionEpoch == claimedCausalEpoch &&
                evidence.toExecutionEpoch == currentSafetyEpoch &&
                evidence.invalidationCount != 0ULL;
            directMailboxFinalized =
                FinalizeDirectContainmentEmergencyStopMailbox(
                    claimedEmergencyStopRequest,
                    claimedEvidenceAlreadyPersistent);
        }
        else if (!directMailboxClaimed &&
            (m_emergencyStopRequestPublication.load(
                std::memory_order_acquire) &
                EMERGENCY_STOP_REQUEST_PENDING) != 0ULL)
        {
            // A producer/consumer reservation won the claim seam.  Leave the
            // action ticket open; the ordinary RT consumer will finish that
            // exact mailbox once the same-word reservation is released.
            directMailboxFinalized = false;
        }

        // Never clear ACTION_PENDING from a newer producer.  If it advanced
        // the ticket without publishing its own mailbox (for example a pure
        // authority request), leave one no-evidence RT fence: the direct stop
        // already performed the physical action, and the later pass only
        // retires the inherited action phase.
        if (directMailboxFinalized &&
            !CompleteSafetyMotionActionTicket(safetyRequestTicket))
        {
            (void)TryPublishEmergencyStopMailbox(
                GetCurrentExecutionEpoch(),
                false);
        }
    }
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
}

void MotionCore::RequestAxisFaultReset(int axisIndex) noexcept
{
    if (axisIndex < 0 || axisIndex >= 32)
    {
        return;
    }

    const std::uint32_t mask =
        static_cast<std::uint32_t>(1U << static_cast<unsigned>(axisIndex));
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    (void)m_axisFaultResetPendingMask.fetch_or(
        mask,
        std::memory_order_acq_rel);
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
}

void MotionCore::RequestResetAllFaults() noexcept
{
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    m_resetAllFaultsPending.store(true, std::memory_order_release);
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
}

void MotionCore::RequestStopGroup() noexcept
{
    // The legacy StopGroup producer owns an immediate Safety ticket.  It is
    // intentionally distinct from RESET's RT-only smooth-stop ingress.
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    m_stopGroupPending.store(true, std::memory_order_release);
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
}

void MotionCore::RequestResetControlledStop() noexcept
{
    // RESET of an ordinary program move is not an Alarm/E-stop.  Do not
    // publish a SAFETY ticket from this producer thread: doing so lets the
    // final PDO fence observe the new ticket before RT has built the
    // same-direction deceleration trajectory, creating a 250 us zero pulse.
    //
    // RT consumes this single bit at the beginning of UpdateInterpolation(),
    // then creates the ticket, owner lease, Epoch and StopMove state as one
    // serial RT operation.  Emergency/alarm producers retain their existing
    // immediate ticket-and-zero-output behavior.
    // The phase is the linearization state visible to the 10 ms NC task.
    // Only IDLE can begin a fresh operator transaction; repeated Reset taps
    // are idempotent and a concurrent Alarm/E-stop can atomically supersede
    // the request without a two-boolean reactivation race.
    std::uint32_t expected =
        static_cast<std::uint32_t>(ResetControlledStopPhase::IDLE);
    // The phase itself is the pre-ticket RT mailbox.  Do not publish a second
    // boolean here: RT may otherwise observe PENDING before the boolean store
    // and incorrectly supersede a valid button press.
    (void)m_resetControlledStopPhase.compare_exchange_strong(
        expected,
        static_cast<std::uint32_t>(ResetControlledStopPhase::PENDING),
        std::memory_order_release,
        std::memory_order_acquire);
}

ResetControlledStopPhase MotionCore::GetResetControlledStopPhase() const noexcept
{
    return static_cast<ResetControlledStopPhase>(
        m_resetControlledStopPhase.load(std::memory_order_acquire));
}

bool MotionCore::ConsumeCompletedResetControlledStop() noexcept
{
    std::uint32_t expected =
        static_cast<std::uint32_t>(ResetControlledStopPhase::COMPLETED);
    return m_resetControlledStopPhase.compare_exchange_strong(
        expected,
        static_cast<std::uint32_t>(ResetControlledStopPhase::IDLE),
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

bool MotionCore::RetireSupersededResetControlledStop() noexcept
{
    // Only a second, explicit operator RESET may retire a terminally
    // superseded smooth-stop request.  It has no motion effect; it merely
    // makes the ordinary, already fail-closed Reset batch eligible to begin.
    std::uint32_t expected =
        static_cast<std::uint32_t>(ResetControlledStopPhase::SUPERSEDED);
    return m_resetControlledStopPhase.compare_exchange_strong(
        expected,
        static_cast<std::uint32_t>(ResetControlledStopPhase::IDLE),
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

void MotionCore::RequestResetSafetyBatch(
    MotionExecutionEpoch publishedEpoch,
    bool requestResetAllFaults) noexcept
{
    // The normal RESET batch is only allowed after the smooth-stop phase has
    // reached COMPLETED and NC has consumed it back to IDLE.  Any overlapping
    // batch therefore supersedes the in-flight button press.
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    const MotionOwnerLease safetyLease =
        TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    const std::uint64_t provenanceGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);

    bool batchPublished = false;
    bool batchCommitted = false;
    const std::uint64_t reservedBatch =
        PackResetSafetyBatch(
            publishedEpoch,
            requestResetAllFaults,
            safetyRequestTicket,
            true);
    const std::uint64_t committedBatch =
        reservedBatch &
        ~RESET_SAFETY_BATCH_PUBLISH_RESERVED;
    std::uint64_t emptyBatch = 0ULL;
    if (safetyRequestTicket != 0U &&
        safetyLease.IsValid() &&
        safetyLease.owner == MotionOwner::SAFETY &&
        publishedEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        publishedEpoch == GetCurrentExecutionEpoch() &&
        m_resetSafetyBatchPending.compare_exchange_strong(
            emptyBatch,
            reservedBatch,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
    {
        m_resetSafetyBatchProvenanceGeneration.store(
            provenanceGeneration,
            std::memory_order_release);
        const std::uint64_t ownerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        const bool exactPublication =
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) == provenanceGeneration &&
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) == 1U &&
            UnpackMotionOwnerState(ownerState).Matches(safetyLease) &&
            UnpackMotionOwnerSafetyRequestTicket(ownerState) ==
            safetyRequestTicket &&
            GetCurrentExecutionEpoch() == publishedEpoch;
        std::uint64_t expectedReserved = reservedBatch;
        if (exactPublication &&
            m_resetSafetyBatchPending.compare_exchange_strong(
                expectedReserved,
                committedBatch,
                std::memory_order_acq_rel,
                std::memory_order_acquire) &&
            (batchCommitted = true) &&
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) == provenanceGeneration &&
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) == 1U)
        {
            batchPublished = true;
        }

        if (!batchCommitted)
        {
            m_resetSafetyBatchProvenanceGeneration.store(
                0ULL,
                std::memory_order_release);
            expectedReserved = reservedBatch;
            (void)m_resetSafetyBatchPending.compare_exchange_strong(
                expectedReserved,
                0ULL,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
        }
    }

    if (!batchPublished)
    {
        // Never merge or overwrite an occupied/reserved batch slot.  A
        // bounded publication failure is contained by a persistent E-stop
        // mailbox instead of an unbounded producer loop.
        (void)TryPublishEmergencyStopMailbox(
            GetCurrentExecutionEpoch(),
            false);
    }
    EndExecutionDrainAcknowledgementRevocation();
}


MotionCore::ResetSafetyAuthorityResult
MotionCore::RequestExactResetSafetyBatch(
    MotionExecutionEpoch publishedEpoch,
    const MotionOwnerLease& safetyLease,
    std::uint32_t parentRequestTicket,
    std::uint64_t expectedProvenanceGeneration,
    bool requestResetAllFaults) noexcept
{
    SupersedeResetControlledStop();

    ResetSafetyAuthorityResult result{};
    result.lease = safetyLease;
    result.requestTicket = parentRequestTicket;

    if (!BeginResetSafetyProvenanceOperation(
        expectedProvenanceGeneration,
        result.provenanceGeneration))
    {
        result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        EndExecutionDrainAcknowledgementRevocation();
        TryAcknowledgeAppliedSafetyMotionRequests();
        return result;
    }

    MotionExecutionEpoch exactEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    result.status = TryGetResetSafetyMotionOwnerEpoch(
        safetyLease,
        parentRequestTicket,
        result.provenanceGeneration,
        exactEpoch);
    if (result.status == ResetSafetyAuthorityStatus::ACQUIRED &&
        exactEpoch != publishedEpoch)
    {
        result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
    }

    if (result.status == ResetSafetyAuthorityStatus::ACQUIRED)
    {
        std::uint64_t parentState =
            m_motionOwnerState.load(std::memory_order_acquire);
        if (m_resetSafetyBatchPending.load(
            std::memory_order_acquire) != 0ULL)
        {
            result.status = ResetSafetyAuthorityStatus::DEFERRED;
        }
        else if (!UnpackMotionOwnerState(parentState).Matches(
            safetyLease) ||
            UnpackMotionOwnerSafetyRequestTicket(parentState) !=
            parentRequestTicket)
        {
            result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        }
        else if ((parentState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
        {
            result.status = ResetSafetyAuthorityStatus::DEFERRED;
        }
        else if (m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire) !=
            result.provenanceGeneration ||
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) != 1U)
        {
            result.status = ResetSafetyAuthorityStatus::SUPERSEDED;
        }
        else
        {
            const std::uint32_t childTicket =
                parentRequestTicket >= MOTION_OWNER_SAFETY_TICKET_MAX
                ? 1U
                : parentRequestTicket + 1U;
            const std::uint64_t childState =
                PackMotionOwnerState(
                    safetyLease.owner,
                    safetyLease.generation,
                    childTicket,
                    false,
                    true);
            std::uint64_t expectedParentState = parentState;
            if (!m_motionOwnerState.compare_exchange_strong(
                expectedParentState,
                childState,
                std::memory_order_acq_rel,
                std::memory_order_acquire))
            {
                if (UnpackMotionOwnerState(expectedParentState).Matches(
                    safetyLease) &&
                    UnpackMotionOwnerSafetyRequestTicket(
                        expectedParentState) == parentRequestTicket &&
                    (expectedParentState &
                        MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL &&
                    m_executionDrainRevocationGeneration.load(
                        std::memory_order_acquire) ==
                    result.provenanceGeneration)
                {
                    result.status =
                        ResetSafetyAuthorityStatus::DEFERRED;
                }
                else
                {
                    result.status =
                        ResetSafetyAuthorityStatus::SUPERSEDED;
                }
            }
            else
            {
                result.requestTicket = childTicket;

                // Reserve the single batch slot before publishing its full
                // provenance sidecar.  Neither a producer nor the RT
                // consumer may overwrite/claim this word while RESERVED.
                const std::uint64_t reservedBatch =
                    PackResetSafetyBatch(
                        publishedEpoch,
                        requestResetAllFaults,
                        childTicket,
                        true);
                const std::uint64_t committedBatch =
                    reservedBatch &
                    ~RESET_SAFETY_BATCH_PUBLISH_RESERVED;
                std::uint64_t emptyBatch = 0ULL;
                bool slotReserved =
                    m_resetSafetyBatchPending.compare_exchange_strong(
                        emptyBatch,
                        reservedBatch,
                        std::memory_order_acq_rel,
                        std::memory_order_acquire);
                bool batchCommitted = false;
                if (!slotReserved)
                {
                    result.status =
                        ResetSafetyAuthorityStatus::SUPERSEDED;
                }
                else
                {
                    m_resetSafetyBatchProvenanceGeneration.store(
                        result.provenanceGeneration,
                        std::memory_order_release);
                }

                if (m_executionDrainRevocationGeneration.load(
                    std::memory_order_acquire) !=
                    result.provenanceGeneration ||
                    m_executionDrainRevocationPublishersInProgress.load(
                        std::memory_order_acquire) != 1U)
                {
                    result.status =
                        ResetSafetyAuthorityStatus::SUPERSEDED;
                }
                else if (slotReserved)
                {
                    const std::uint64_t publishedOwnerState =
                        m_motionOwnerState.load(
                            std::memory_order_acquire);
                    std::uint64_t expectedReserved = reservedBatch;
                    if (UnpackMotionOwnerState(
                        publishedOwnerState).Matches(safetyLease) &&
                        UnpackMotionOwnerSafetyRequestTicket(
                            publishedOwnerState) == childTicket &&
                        m_resetSafetyBatchPending.compare_exchange_strong(
                            expectedReserved,
                            committedBatch,
                            std::memory_order_acq_rel,
                            std::memory_order_acquire))
                    {
                        batchCommitted = true;
                    }
                    else
                    {
                        result.status =
                            ResetSafetyAuthorityStatus::SUPERSEDED;
                    }
                }

                const std::uint64_t postCommitOwnerState =
                    m_motionOwnerState.load(
                        std::memory_order_acquire);
                if (batchCommitted &&
                    (m_executionDrainRevocationGeneration.load(
                        std::memory_order_acquire) !=
                        result.provenanceGeneration ||
                        m_executionDrainRevocationPublishersInProgress.load(
                            std::memory_order_acquire) != 1U ||
                        !UnpackMotionOwnerState(
                            postCommitOwnerState).Matches(
                                safetyLease) ||
                        UnpackMotionOwnerSafetyRequestTicket(
                            postCommitOwnerState) != childTicket))
                {
                    result.status =
                        ResetSafetyAuthorityStatus::SUPERSEDED;
                }

                if (slotReserved &&
                    !batchCommitted &&
                    result.status != ResetSafetyAuthorityStatus::ACQUIRED)
                {
                    // A producer may roll back only its never-published
                    // RESERVED word. Once committed, ownership belongs to the
                    // RT consumer; it validates the sidecar and fails closed.
                    m_resetSafetyBatchProvenanceGeneration.store(
                        0ULL,
                        std::memory_order_release);
                    std::uint64_t expectedReserved = reservedBatch;
                    (void)m_resetSafetyBatchPending.compare_exchange_strong(
                        expectedReserved,
                        0ULL,
                        std::memory_order_acq_rel,
                        std::memory_order_acquire);
                }
            }
        }
    }

    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
    return result;
}

bool MotionCore::HasPendingSafetyIntent() const noexcept
{
    return
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        HasUnacknowledgedSafetyMotionRequest() ||
        m_safetyRecoveryRequestInProgress.load(std::memory_order_acquire) ||
        (m_emergencyStopRequestPublication.load(
            std::memory_order_acquire) &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL ||
        m_resetAllFaultsPending.load(std::memory_order_acquire) ||
        m_stopGroupPending.load(std::memory_order_acquire) ||
        m_resetSafetyBatchPending.load(std::memory_order_acquire) != 0ULL ||
        m_axisFaultResetPendingMask.load(std::memory_order_acquire) != 0U ||
        (m_p1MappingIntegrityAlarmRequestPublication.load(
            std::memory_order_acquire) &
            P1_MAPPING_ALARM_PENDING) != 0ULL;
}

bool MotionCore::HasResetControlledStopPriorityWinner() const noexcept
{
    // This deliberately excludes RESET's own phase/ticket.  It names only
    // independent Safety producers that must win over a benign operator
    // controlled-stop request.  In particular, HasPendingSafetyIntent() is
    // not enough here because an already materialized Alarm has no required
    // mailbox bit yet.
    return
        AlarmManager::GetInstance().HasAlarm() ||
        (m_emergencyStopRequestPublication.load(
            std::memory_order_acquire) &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL ||
        m_resetAllFaultsPending.load(std::memory_order_acquire) ||
        m_stopGroupPending.load(std::memory_order_acquire) ||
        m_resetSafetyBatchPending.load(std::memory_order_acquire) != 0ULL ||
        m_axisFaultResetPendingMask.load(std::memory_order_acquire) != 0U ||
        (m_p1MappingIntegrityAlarmRequestPublication.load(
            std::memory_order_acquire) &
            P1_MAPPING_ALARM_PENDING) != 0ULL;
}


bool MotionCore::HasPendingSafetyOrRecoveryRequests() const noexcept
{
    return
        HasPendingSafetyIntent() ||
        HasPendingExecutionEpochChange();
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


bool MotionCore::TryClaimResetSafetyBatch(
    std::uint64_t& claimedBatch,
    std::uint64_t& provenanceGeneration) noexcept
{
    claimedBatch = 0ULL;
    provenanceGeneration = 0ULL;
    std::uint64_t observed =
        m_resetSafetyBatchPending.load(std::memory_order_acquire);
    if ((observed & RESET_SAFETY_BATCH_PRESENT) == 0ULL ||
        (observed &
            RESET_SAFETY_BATCH_PUBLISH_RESERVED) != 0ULL)
    {
        return false;
    }

    const std::uint64_t sidecar =
        m_resetSafetyBatchProvenanceGeneration.load(
            std::memory_order_acquire);
    const std::uint64_t reserved =
        observed | RESET_SAFETY_BATCH_PUBLISH_RESERVED;
    if (!m_resetSafetyBatchPending.compare_exchange_strong(
        observed,
        reserved,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    claimedBatch =
        reserved & ~RESET_SAFETY_BATCH_PUBLISH_RESERVED;
    provenanceGeneration = sidecar;
    return true;
}


bool MotionCore::FinalizeClaimedResetSafetyBatch(
    std::uint64_t claimedBatch) noexcept
{
    if ((claimedBatch & RESET_SAFETY_BATCH_PRESENT) == 0ULL ||
        (claimedBatch &
            RESET_SAFETY_BATCH_PUBLISH_RESERVED) != 0ULL)
    {
        return false;
    }

    const std::uint64_t reserved =
        claimedBatch | RESET_SAFETY_BATCH_PUBLISH_RESERVED;
    // Clear the sidecar while this consumer still owns the nonzero reserved
    // word.  No producer can reuse the slot before the following exact CAS.
    m_resetSafetyBatchProvenanceGeneration.store(
        0ULL,
        std::memory_order_release);
    std::uint64_t expected = reserved;
    return m_resetSafetyBatchPending.compare_exchange_strong(
        expected,
        0ULL,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}


void MotionCore::AbortActiveExecutionForResetSafetyBatch() noexcept
{
    // Reset installs a cross-scan whole-PDO zero-output hold before it
    // publishes this batch.  A conventional StopGroupImpl(false) would set a
    // virtual controlled-deceleration trajectory, but the final PDO fence is
    // deliberately not allowed to transmit that non-zero trajectory.  The
    // old group would then remain logically active forever, preventing the
    // Reset settle/rebase transaction from ever being armed.
    //
    // This is the exact Reset-only counterpart of the abort-active Epoch
    // consumer path: preserve terminal ABORT accounting, cancel future
    // history, and make all command state zero.  It does not raise an E-stop,
    // clear a fault, or claim that the physical axes have already stopped;
    // the following formal RESET_ALL settle proof still waits for measured
    // standstill before rebase and release.
    AbortTrackedMotionCommand();

    // Old-Epoch queue entries are retired by the bounded consumer on the
    // subsequent pass after this batch ticket is acknowledged.  History can
    // be retired immediately because it is RT-owned and cannot cross Reset.
    m_Group.historyQueue.clear();

    m_Group.isActive = false;
    m_safetyControlledStopInProgress = false;
    m_safetyControlledStopOwnerLease = MotionOwnerLease{};
    m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_safetyControlledStopRequestTicket = 0U;

    // Cancel jump/path producers as part of the same Reset abort.  Leaving a
    // JUMP_TRACKING or PATH_SERVO mode live after the group is inactive would
    // correctly make Reset fail closed as "unsupported", but would require a
    // second Reset even though this exact batch already owns the cancellation.
    m_Group.pathMode = PathMode::EXACT_STOP;
    m_Group.pathServoVel = 0.0;
    m_Group.jumpManager.state = JumpState::IDLE;
    m_Group.jumpManager.currentOffset = 0.0;
    m_Group.jumpManager.targetOffset = 0.0;
    m_Group.jumpManager.jumpVel = 0.0;
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

    // Match the existing abort-active Epoch retirement semantics.  Invalid
    // values, faults, and E-stop states are intentionally left visible so
    // RESET_ALL proof remains fail-closed instead of silently masking them.
    if (!m_Group.virtualAxis.isFault &&
        !m_Group.virtualAxis.isLagAlarm &&
        m_Group.virtualAxis.state != MotionState::MotionState_ERROR &&
        m_Group.virtualAxis.state != MotionState::MotionState_ESTOP)
    {
        m_Group.virtualAxis.state = MotionState::MotionState_IDLE;
        (void)TryCanonicalizeIdleAxisCommandState(
            m_Group.virtualAxis);
    }

    if (m_pContexts != nullptr)
    {
        const int safeGroupAxisCount =
            (std::max)(0, (std::min)(m_Group.axisCount, MAX_AXES));
        for (int groupAxis = 0;
            groupAxis < safeGroupAxisCount;
            ++groupAxis)
        {
            const int axisIndex = m_Group.axisIndices[groupAxis];
            if (axisIndex < 0 ||
                axisIndex >= static_cast<int>(m_pContexts->size()))
            {
                continue;
            }

            (void)TryCanonicalizeInactivePhysicalAxisCommandState(
                (*m_pContexts)[axisIndex]);
        }
    }

    InvalidateCncLineEndpointProof();
    m_Group.currentCmd.execution = MotionExecutionIdentity{};
    m_Group.currentCmd.ownerLease = MotionOwnerLease{};
}


void MotionCore::ApplyPendingSafetyAndRecoveryRequests() noexcept
{
    const bool hasPendingRequest =
        (m_emergencyStopRequestPublication.load(
            std::memory_order_acquire) &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL ||
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

    std::uint64_t safetyOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    std::uint32_t safetyRequestTicket =
        UnpackMotionOwnerSafetyRequestTicket(safetyOwnerState);
    if (safetyRequestTicket == 0U ||
        safetyRequestTicket == m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire))
    {
        // The mailbox is already observable, so it is the exact action
        // publication fence. A replacement ticket does not need a producer-
        // owned ACTION_PENDING bit; the mailbox/in-progress gates keep it
        // unacknowledged until this RT pass applies it.
        safetyRequestTicket =
            PublishSafetyMotionRequestTicket(false);
    }
    const MotionOwnerLease safetyLease =
        TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    if (!safetyLease.IsValid() ||
        safetyLease.owner != MotionOwner::SAFETY ||
        !IsMotionOwnerLeaseCurrent(safetyLease))
    {
        // Bounded RT attempt failed (normally only a transient reservation or
        // CAS collision). Keep every mailbox bit intact for the next pass;
        // the outstanding ticket and final PDO fence already fail closed.
        m_safetyRecoveryRequestInProgress.store(
            false,
            std::memory_order_release);
        return;
    }

    // Claim the exact action only after a mailbox is observable and while
    // the RT in-progress gate is held. If the bounded CAS loses, leave every
    // mailbox intact; applying the SAFETY Epoch before this claim would erase
    // the causal P queue needed by J.6 E-stop evidence.
    if (m_executionDrainRevocationPublishersInProgress.load(
        std::memory_order_acquire) != 0U)
    {
        m_safetyRecoveryRequestInProgress.store(
            false,
            std::memory_order_release);
        return;
    }

    if (UnpackMotionOwnerSafetyActionPending(
        m_motionOwnerState.load(std::memory_order_acquire)) &&
        !CompleteSafetyMotionActionTicket(safetyRequestTicket))
    {
        m_safetyRecoveryRequestInProgress.store(
            false,
            std::memory_order_release);
        return;
    }

    const bool emergencyStopMailboxPending =
        (m_emergencyStopRequestPublication.load(
            std::memory_order_acquire) &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL;
    std::uint64_t emergencyStopRequest = 0ULL;
    if (emergencyStopMailboxPending &&
        !TryClaimEmergencyStopMailbox(emergencyStopRequest))
    {
        // RESERVED or concurrently changed: retain strict E-stop priority
        // and every lower-priority recovery mailbox for the next RT pass.
        m_safetyRecoveryRequestInProgress.store(
            false,
            std::memory_order_release);
        return;
    }
    if ((emergencyStopRequest &
        EMERGENCY_STOP_REQUEST_PENDING) != 0ULL)
    {
        SupersedeResetControlledStop();
        m_resetAllFaultsPending.store(false, std::memory_order_release);
        m_stopGroupPending.store(false, std::memory_order_release);
        std::uint64_t discardedBatch = 0ULL;
        std::uint64_t discardedBatchProvenance = 0ULL;
        if (TryClaimResetSafetyBatch(
            discardedBatch,
            discardedBatchProvenance))
        {
            (void)FinalizeClaimedResetSafetyBatch(discardedBatch);
        }
        m_axisFaultResetPendingMask.store(0U, std::memory_order_release);
        EmergencyStopAllAxesImpl(
            UnpackEmergencyStopRequestEvidenceRequired(
                emergencyStopRequest),
            UnpackEmergencyStopRequestEpoch(
                emergencyStopRequest));
        (void)CompleteSafetyMotionActionTicket(
            safetyRequestTicket);
        m_safetyRecoveryRequestInProgress.store(false, std::memory_order_release);
        TryAcknowledgeAppliedSafetyMotionRequests();
        return;
    }

    std::uint64_t resetSafetyBatch = 0ULL;
    std::uint64_t resetSafetyBatchProvenance = 0ULL;
    const bool resetSafetyBatchPending =
        m_resetSafetyBatchPending.load(
            std::memory_order_acquire) != 0ULL;
    if (resetSafetyBatchPending &&
        !TryClaimResetSafetyBatch(
            resetSafetyBatch,
            resetSafetyBatchProvenance))
    {
        // Producer-RESERVED or concurrently changed: do not inspect a
        // partial sidecar and do not let lower-priority recovery overtake it.
        m_safetyRecoveryRequestInProgress.store(
            false,
            std::memory_order_release);
        return;
    }

    if ((resetSafetyBatch & RESET_SAFETY_BATCH_PRESENT) != 0ULL)
    {
        SupersedeResetControlledStop();
        const MotionExecutionEpoch coveredEpoch =
            UnpackResetSafetyBatchEpoch(resetSafetyBatch);
        const std::uint32_t coveredTicket =
            UnpackResetSafetyBatchTicket(resetSafetyBatch);
        const std::uint64_t currentOwnerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        const std::uint64_t currentExecutionPublication =
            m_executionEpochPublication.load(
                std::memory_order_acquire);

        const bool epochCoverageValid =
            coveredEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            coveredEpoch ==
            UnpackExecutionEpochPublication(
                currentExecutionPublication) &&
            UnpackExecutionEpochPublicationSource(
                currentExecutionPublication) ==
            MotionCommandSource::SAFETY &&
            (currentExecutionPublication &
                (EXECUTION_EPOCH_PUBLICATION_PENDING |
                    EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) == 0ULL;
        const bool provenanceValid =
            resetSafetyBatchProvenance != 0ULL &&
            resetSafetyBatchProvenance ==
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) &&
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) == 0U;
        const MotionOwnerLease currentSafetyLease =
            UnpackMotionOwnerState(currentOwnerState);
        const bool ownerTicketValid =
            coveredTicket != 0U &&
            coveredTicket == safetyRequestTicket &&
            currentSafetyLease.owner == MotionOwner::SAFETY &&
            currentSafetyLease.IsValid() &&
            UnpackMotionOwnerSafetyRequestTicket(currentOwnerState) ==
            coveredTicket &&
            !UnpackMotionOwnerSafetyHandshake(currentOwnerState) &&
            !UnpackMotionOwnerSafetyActionPending(currentOwnerState) &&
            (currentOwnerState &
                MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL;

        if (!epochCoverageValid ||
            !provenanceValid ||
            !ownerTicketValid)
        {
            // Another lifecycle event intervened.  Never retarget this Reset
            // batch to a replacement Epoch: stop fail-closed and let the
            // exact NC Reset release gate remain blocked until a new explicit
            // operator transaction is created.
            EmergencyStopAllAxesImpl(true);
            (void)FinalizeClaimedResetSafetyBatch(resetSafetyBatch);
            (void)CompleteSafetyMotionActionTicket(
                safetyRequestTicket);
            m_safetyRecoveryRequestInProgress.store(
                false,
                std::memory_order_release);
            TryAcknowledgeAppliedSafetyMotionRequests();
            return;
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

        // The operator Reset output hold forces final PDO velocity to zero
        // before this exact batch reaches RT.  Retire the active command as
        // an abort instead of starting a controlled stop whose PDO output is
        // intentionally fenced; physical standstill is proved later by the
        // existing RESET_ALL settle/rebase gate.
        AbortActiveExecutionForResetSafetyBatch();
        if (!FinalizeClaimedResetSafetyBatch(resetSafetyBatch))
        {
            EmergencyStopAllAxesImpl(true);
        }
    }

    if (m_resetAllFaultsPending.exchange(false, std::memory_order_acq_rel))
    {
        SupersedeResetControlledStop();
        ResetAllFaultsImpl(true);
    }

    const std::uint32_t resetMask =
        m_axisFaultResetPendingMask.exchange(0U, std::memory_order_acq_rel);

    if (resetMask != 0U && m_pContexts != nullptr)
    {
        SupersedeResetControlledStop();
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
        SupersedeResetControlledStop();
        StopGroupImpl(true);

        // The stop mailbox is consumed after UpdateInterpolation()'s normal
        // Epoch seam. Consume the successor now, while the exact SAFETY
        // action ticket has already been completed above, so this same PDO
        // pass emits the deceleration command rather than one forced zero.
        ApplyPendingExecutionEpochChange();
    }

    (void)CompleteSafetyMotionActionTicket(safetyRequestTicket);
    m_safetyRecoveryRequestInProgress.store(false, std::memory_order_release);
    TryAcknowledgeAppliedSafetyMotionRequests();
}

void MotionCore::SupersedeResetControlledStop() noexcept
{
    std::uint32_t observed =
        m_resetControlledStopPhase.load(std::memory_order_acquire);
    for (;;)
    {
        const ResetControlledStopPhase phase =
            static_cast<ResetControlledStopPhase>(observed);
        if (phase == ResetControlledStopPhase::IDLE ||
            phase == ResetControlledStopPhase::SUPERSEDED)
        {
            return;
        }

        if (m_resetControlledStopPhase.compare_exchange_weak(
            observed,
            static_cast<std::uint32_t>(
                ResetControlledStopPhase::SUPERSEDED),
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return;
        }
    }
}

void MotionCore::CompleteResetControlledStop() noexcept
{
    std::uint32_t expected =
        static_cast<std::uint32_t>(ResetControlledStopPhase::ACTIVE);
    if (m_resetControlledStopPhase.compare_exchange_strong(
        expected,
        static_cast<std::uint32_t>(
            ResetControlledStopPhase::COMPLETED),
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return;
    }

    // A reset issued in the small post-command/pre-physical-active window
    // can coherently prove that no group motion remains.  It still needs a
    // terminal success result for NC, but never overwrites SUPERSEDED.
    expected = static_cast<std::uint32_t>(
        ResetControlledStopPhase::APPLYING);
    (void)m_resetControlledStopPhase.compare_exchange_strong(
        expected,
        static_cast<std::uint32_t>(
            ResetControlledStopPhase::COMPLETED),
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

void MotionCore::ApplyPendingResetControlledStopRequest() noexcept
{
    std::uint32_t expectedPhase =
        static_cast<std::uint32_t>(ResetControlledStopPhase::PENDING);
    if (!m_resetControlledStopPhase.compare_exchange_strong(
        expectedPhase,
        static_cast<std::uint32_t>(
            ResetControlledStopPhase::APPLYING),
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return;
    }

    // PENDING was atomically claimed above.  A true Alarm/E-stop/reset batch
    // is always the winner and turns this button press terminal rather than
    // letting it enter the PDO hold path.
    if (m_resetControlledStopPhase.load(std::memory_order_acquire) !=
        static_cast<std::uint32_t>(
            ResetControlledStopPhase::APPLYING) ||
        HasPendingSafetyIntent() ||
        HasResetControlledStopPriorityWinner())
    {
        SupersedeResetControlledStop();
        return;
    }

    // Only the 250 us consumer is allowed to turn the benign RESET ingress
    // into a SAFETY owner/ticket transition. Therefore the first final PDO
    // command which can observe this ticket is emitted after StopMove has
    // already selected the controlled-deceleration trajectory.
    BeginExecutionDrainAcknowledgementRevocation();
    const std::uint64_t resetProvenanceGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t publishedSafetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    std::uint32_t safetyRequestTicket = publishedSafetyRequestTicket;
    if (publishedSafetyRequestTicket == 0U ||
        !EnsureSafetyMotionActionTicket(safetyRequestTicket) ||
        safetyRequestTicket != publishedSafetyRequestTicket)
    {
        m_safetyRecoveryRequestInProgress.store(
            false,
            std::memory_order_release);
        SupersedeResetControlledStop();
        EmergencyStopAllAxesImpl(true);
        EndExecutionDrainAcknowledgementRevocation();
        return;
    }

    const MotionOwnerLease safetyLease =
        TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    const auto ownsExactResetSafetyAction = [&]() noexcept -> bool
    {
        const std::uint64_t ownerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease ownerLease =
            UnpackMotionOwnerState(ownerState);
        return
            m_resetControlledStopPhase.load(
                std::memory_order_acquire) ==
            static_cast<std::uint32_t>(
                ResetControlledStopPhase::APPLYING) &&
            m_executionDrainRevocationGeneration.load(
                std::memory_order_acquire) == resetProvenanceGeneration &&
            m_executionDrainRevocationPublishersInProgress.load(
                std::memory_order_acquire) == 1U &&
            !HasResetControlledStopPriorityWinner() &&
            safetyLease.IsValid() &&
            safetyLease.owner == MotionOwner::SAFETY &&
            safetyLease.Matches(ownerLease) &&
            IsMotionOwnerLeaseCurrent(safetyLease) &&
            UnpackMotionOwnerSafetyRequestTicket(ownerState) ==
            safetyRequestTicket &&
            !UnpackMotionOwnerSafetyHandshake(ownerState) &&
            UnpackMotionOwnerSafetyActionPending(ownerState) &&
            (ownerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL;
    };

    if (!ownsExactResetSafetyAction())
    {
        // TryTakeSafetyMotionOwnerForTicket deliberately rewrites only its
        // local ticket when a newer Safety incident wins.  Never use an
        // ambiguous lease to build a RESET controlled-stop exception; fail
        // closed and make the NC result terminally BLOCKED instead.
        SupersedeResetControlledStop();
        EndExecutionDrainAcknowledgementRevocation();
        EmergencyStopAllAxesImpl(true);
        return;
    }

    // This is a short RT-only transaction. It prevents acknowledgement/release
    // helpers from observing a half-built controlled stop; it is cleared
    // before UpdateAllMotion can evaluate the controlled-stop authorization.
    m_safetyRecoveryRequestInProgress.store(
        true,
        std::memory_order_release);

    StopGroupImpl(true);

    // StopGroupImpl publishes the SAFETY successor Epoch after the normal
    // top-of-pass Epoch seam.  A higher-priority producer may have arrived
    // while it ran, so prove the whole owner/ticket/provenance tuple again
    // before completing an action.  RESET must never complete another
    // safety event's ticket.
    const bool exactActionStillOwned = ownsExactResetSafetyAction();
    const bool actionCompleted =
        exactActionStillOwned &&
        CompleteSafetyMotionActionTicket(safetyRequestTicket);
    if (actionCompleted)
    {
        // Same-pass Epoch application is the key seam: final PDO can now see
        // both the exact completed Safety ticket and StopMove's deceleration
        // trajectory, rather than emitting a forced zero frame first.
        ApplyPendingExecutionEpochChange();
    }

    m_safetyRecoveryRequestInProgress.store(
        false,
        std::memory_order_release);
    EndExecutionDrainAcknowledgementRevocation();

    if (!actionCompleted)
    {
        // A newer Safety action owns the packed ticket now. Do not apply an
        // ambiguous Epoch; the ordinary Safety path will contain it and this
        // Reset press remains terminally superseded.
        SupersedeResetControlledStop();
        EmergencyStopAllAxesImpl(true);
        return;
    }

    if (!m_safetyControlledStopInProgress)
    {
        // Inactive/no-queue group: RT consumed the request coherently and no
        // physical controlled-stop proof remains to collect.
        TryAcknowledgeAppliedSafetyMotionRequests();
        CompleteResetControlledStop();
        return;
    }

    const int firstGroupAxis =
        m_Group.axisCount > 0 ? m_Group.axisIndices[0] : -1;
    if (firstGroupAxis < 0 ||
        !IsSafetyControlledStopAuthorized(firstGroupAxis))
    {
        // A concurrent immediate Safety producer or Epoch reservation won a
        // post-claim seam. The final PDO fence stays fail-closed; do not let
        // this Reset continuation mistake that containment for completion.
        SupersedeResetControlledStop();
        return;
    }

    std::uint32_t applying =
        static_cast<std::uint32_t>(ResetControlledStopPhase::APPLYING);
    if (!m_resetControlledStopPhase.compare_exchange_strong(
        applying,
        static_cast<std::uint32_t>(
            ResetControlledStopPhase::ACTIVE),
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        // The only legal concurrent winner is a higher-priority supersede.
        // Existing Safety fences handle its output; never overwrite it.
        return;
    }

    TryAcknowledgeAppliedSafetyMotionRequests();
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
            applied = MoveToPosition(
                axis, command.value0, command.value1, command.value2, command.value3);
            axis.useShortestPath = originalShortestPath;
            if (!applied) rejectReason = MotionRejectReason::INVALID_GEOMETRY;
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
            applied = ApplyMachineHome(
                axis,
                command.value0,
                command.value1,
                command.ownerLease);
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


MotionCore::ResetSafetyAuthorityStatus
MotionCore::TryGetResetSafetyMotionOwnerEpoch(
    const MotionOwnerLease& safetyLease,
    std::uint32_t requestTicket,
    std::uint64_t expectedProvenanceGeneration,
    MotionExecutionEpoch& executionEpoch) const noexcept
{
    executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    if (!safetyLease.IsValid() ||
        safetyLease.owner != MotionOwner::SAFETY ||
        requestTicket == 0U)
    {
        return ResetSafetyAuthorityStatus::SUPERSEDED;
    }

    const std::uint64_t entryProvenance =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    if (entryProvenance != expectedProvenanceGeneration)
    {
        return ResetSafetyAuthorityStatus::SUPERSEDED;
    }

    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if (!UnpackMotionOwnerState(ownerState).Matches(safetyLease) ||
        UnpackMotionOwnerSafetyRequestTicket(ownerState) != requestTicket)
    {
        return ResetSafetyAuthorityStatus::SUPERSEDED;
    }

    const std::uint32_t acknowledgedTicket =
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire);
    const std::uint64_t acknowledgement =
        m_safetyOwnerEpochAcknowledgement.load(
            std::memory_order_acquire);
    const std::uint64_t publication =
        m_executionEpochPublication.load(
            std::memory_order_acquire);

    if (m_executionDrainRevocationGeneration.load(
        std::memory_order_acquire) != entryProvenance ||
        m_motionOwnerState.load(std::memory_order_acquire) != ownerState)
    {
        return ResetSafetyAuthorityStatus::SUPERSEDED;
    }

    if (UnpackMotionOwnerSafetyHandshake(ownerState) ||
        UnpackMotionOwnerSafetyActionPending(ownerState) ||
        (ownerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        acknowledgedTicket != requestTicket)
    {
        return ResetSafetyAuthorityStatus::DEFERRED;
    }

    const MotionOwnerGeneration acknowledgedGeneration =
        static_cast<MotionOwnerGeneration>(acknowledgement >> 32U);
    const MotionExecutionEpoch acknowledgedEpoch =
        static_cast<MotionExecutionEpoch>(
            acknowledgement & 0xFFFFFFFFULL);
    if (acknowledgedGeneration != safetyLease.generation ||
        acknowledgedEpoch == MOTION_EXECUTION_EPOCH_INVALID)
    {
        return ResetSafetyAuthorityStatus::DEFERRED;
    }

    if ((publication &
        (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL)
    {
        return ResetSafetyAuthorityStatus::DEFERRED;
    }

    if (UnpackExecutionEpochPublication(publication) !=
        acknowledgedEpoch ||
        UnpackExecutionEpochPublicationSource(publication) !=
        MotionCommandSource::SAFETY)
    {
        return ResetSafetyAuthorityStatus::SUPERSEDED;
    }

    executionEpoch = acknowledgedEpoch;
    return ResetSafetyAuthorityStatus::ACQUIRED;
}


bool MotionCore::TryGetSafetyMotionOwnerEpoch(
    const MotionOwnerLease& safetyLease,
    MotionExecutionEpoch& executionEpoch) const noexcept
{
    executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    if (!safetyLease.IsValid() ||
        safetyLease.owner != MotionOwner::SAFETY)
    {
        return false;
    }

    const std::uint64_t ownerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t acknowledgement =
        m_safetyOwnerEpochAcknowledgement.load(
            std::memory_order_acquire);
    const std::uint64_t publication =
        m_executionEpochPublication.load(
            std::memory_order_acquire);
    const MotionOwnerGeneration acknowledgedGeneration =
        static_cast<MotionOwnerGeneration>(acknowledgement >> 32U);
    const MotionExecutionEpoch acknowledgedEpoch =
        static_cast<MotionExecutionEpoch>(
            acknowledgement & 0xFFFFFFFFULL);

    if (!UnpackMotionOwnerState(ownerState).Matches(safetyLease) ||
        UnpackMotionOwnerSafetyHandshake(ownerState) ||
        acknowledgedGeneration != safetyLease.generation ||
        acknowledgedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        UnpackExecutionEpochPublication(publication) !=
        acknowledgedEpoch ||
        UnpackExecutionEpochPublicationSource(publication) !=
        MotionCommandSource::SAFETY)
    {
        return false;
    }

    executionEpoch = acknowledgedEpoch;
    return true;
}


MotionExecutionEpoch MotionCore::PublishNewExecutionEpoch(
    MotionCommandSource source,
    bool abortActiveCommand) noexcept
{
    const std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease entryOwnerLease =
        UnpackMotionOwnerState(entryOwnerState);
    const MotionOwner requiredOwner =
        ResolveMotionOwnerForSource(source);

    if (UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
        (entryOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
    {
        return MOTION_EXECUTION_EPOCH_INVALID;
    }

    if (source == MotionCommandSource::SAFETY)
    {
        if (entryOwnerLease.owner != MotionOwner::SAFETY ||
            !entryOwnerLease.IsValid())
        {
            return MOTION_EXECUTION_EPOCH_INVALID;
        }
    }
    else
    {
        if (HasUnacknowledgedSafetyMotionRequest() ||
            entryOwnerLease.owner == MotionOwner::SAFETY ||
            (entryOwnerLease.owner != MotionOwner::NONE &&
                requiredOwner != MotionOwner::NONE &&
                entryOwnerLease.owner != requiredOwner))
        {
            return MOTION_EXECUTION_EPOCH_INVALID;
        }
    }

    std::uint64_t currentPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    if ((currentPublication &
        EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
    {
        m_lifecycleCommitReservationPublisherWaitCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        return MOTION_EXECUTION_EPOCH_INVALID;
    }

    if (m_motionOwnerState.load(std::memory_order_acquire) !=
        entryOwnerState)
    {
        return MOTION_EXECUTION_EPOCH_INVALID;
    }

    const MotionExecutionEpoch currentEpoch =
        UnpackExecutionEpochPublication(currentPublication);
    const MotionExecutionEpoch nextEpoch =
        (currentEpoch ==
            (std::numeric_limits<MotionExecutionEpoch>::max)())
        ? 1U
        : static_cast<MotionExecutionEpoch>(currentEpoch + 1U);
    const std::uint64_t desiredPublication =
        PackExecutionEpochPublication(
            nextEpoch,
            source,
            abortActiveCommand,
            true);

    // One exact attempt only. A stale lifecycle producer never rebuilds from
    // a later SAFETY Epoch. If a Safety takeover starts after the precheck,
    // either this CAS linearizes first and the handshake publishes after it,
    // or this exact CAS loses and returns INVALID.
    if (!m_executionEpochPublication.compare_exchange_strong(
        currentPublication,
        desiredPublication,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return MOTION_EXECUTION_EPOCH_INVALID;
    }

    if (m_motionOwnerState.load(std::memory_order_acquire) !=
        entryOwnerState)
    {
        return MOTION_EXECUTION_EPOCH_INVALID;
    }
    return nextEpoch;
}


bool MotionCore::TryAcquireLifecycleCommitReservation(
    const MotionExecutionIdentity& execution,
    std::uint64_t& reservationToken) noexcept
{
    reservationToken = 0ULL;
    m_lifecycleCommitReservationAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    const std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease entryOwnerLease =
        UnpackMotionOwnerState(entryOwnerState);
    const MotionOwner requiredOwner =
        ResolveMotionOwnerForSource(execution.source);
    if (UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(entryOwnerState) ||
        (entryOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        UnpackMotionOwnerSafetyRequestTicket(entryOwnerState) !=
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire) ||
        HasPendingSafetyOrRecoveryRequests() ||
        requiredOwner == MotionOwner::NONE ||
        entryOwnerLease.owner != requiredOwner ||
        !entryOwnerLease.IsValid())
    {
        m_lifecycleCommitReservationBlockedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        return false;
    }

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

    if (m_motionOwnerState.load(std::memory_order_acquire) !=
        entryOwnerState)
    {
        std::uint64_t expectedReserved = reserved;
        (void)m_executionEpochPublication.compare_exchange_strong(
            expectedReserved,
            reserved &
            ~EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
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
    const MotionCommand& command,
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
    submission.identity = command.execution;
    submission.translationGeneration = command.sourceTranslation.generation;
    submission.commandPathMode = command.commandPathMode;
    submission.producerAccepted = producerAccepted;
    submission.immediateRejectReason = immediateRejectReason;
}


bool MotionCore::BindProgramBlockQueueTailReceipt(
    MotionQueueTailCommitReceipt& receipt) noexcept
{
    if (!m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.count == 0U)
    {
        return false;
    }

    for (std::size_t offset = 0U;
        offset < m_programBlockMotionCapture.count;
        ++offset)
    {
        const std::size_t index =
            m_programBlockMotionCapture.count - 1U - offset;
        MotionProgramBlockSubmission& submission =
            m_programBlockMotionCapture.submissions[index];
        const MotionExecutionIdentity& identity = submission.identity;
        if (identity.epoch != receipt.identity.epoch ||
            identity.segmentId != receipt.identity.segmentId ||
            identity.sourceBlockId != receipt.identity.sourceBlockId ||
            identity.source != receipt.identity.source)
        {
            continue;
        }

        receipt.captureBound = true;
        submission.queueTailReceipt = receipt;
        return true;
    }
    return false;
}


MotionQueueTailTransactionSequence
MotionCore::AllocateQueueTailTransactionSequence() noexcept
{
    MotionQueueTailTransactionSequence sequence =
        m_nextQueueTailTransactionSequence.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    if (sequence == MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID)
    {
        sequence = m_nextQueueTailTransactionSequence.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    return sequence;
}


void MotionCore::PublishQueueTailTransactionReceipt(
    MotionQueueTailCommitReceipt& receipt,
    bool invalidInput) noexcept
{
    (void)BindProgramBlockQueueTailReceipt(receipt);

    m_queueTailWriteSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);
    m_queueTailAttempts.fetch_add(1ULL, std::memory_order_relaxed);

    const bool committed = receipt.IsCommitted();
    const bool rejectedPreserved = receipt.IsRejectedAndPreserved();
    if (receipt.commandAccepted)
    {
        m_queueTailCommandAccepted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_queueTailCommandRejected.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    if (committed)
    {
        m_queueTailCommitted.fetch_add(1ULL, std::memory_order_relaxed);
        m_queueTailCommandedMCSCommitted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_queueTailLastQueuedPulseCommitted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_queueTailRapidOverrideCommitted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_queueTailEndpointExact.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        if (receipt.captureBound)
        {
            m_queueTailCaptureBound.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        m_lastQueueTailCommittedFingerprint.store(
            receipt.committedFingerprint,
            std::memory_order_relaxed);
    }
    else if (rejectedPreserved)
    {
        m_queueTailRejectPreserved.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_queueTailMismatches.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    if (invalidInput)
    {
        m_queueTailInvalidInputs.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    m_lastQueueTailTransactionSequence.store(
        receipt.transactionSequence,
        std::memory_order_relaxed);
    m_lastQueueTailExecutionEpoch.store(
        receipt.identity.epoch,
        std::memory_order_relaxed);
    m_lastQueueTailSegmentId.store(
        receipt.identity.segmentId,
        std::memory_order_relaxed);
    m_lastQueueTailAxisMask.store(
        receipt.axisMask,
        std::memory_order_relaxed);
    m_queueTailWriteSequence.fetch_add(
        1ULL,
        std::memory_order_release);
}


static std::uint64_t FoldCommandPathModeTransportFingerprintValue(
    std::uint64_t fingerprint,
    std::uint64_t value) noexcept
{
    return (fingerprint ^ value) * 1099511628211ULL;
}


static std::uint64_t FoldCommandPathModeTransportFingerprint(
    std::uint64_t fingerprint,
    const MotionCommand& command) noexcept
{
    fingerprint = FoldCommandPathModeTransportFingerprintValue(
        fingerprint,
        static_cast<std::uint64_t>(command.execution.epoch));
    fingerprint = FoldCommandPathModeTransportFingerprintValue(
        fingerprint,
        static_cast<std::uint64_t>(command.execution.segmentId));
    fingerprint = FoldCommandPathModeTransportFingerprintValue(
        fingerprint,
        static_cast<std::uint64_t>(
            static_cast<std::int64_t>(command.sourceLinePC)));
    fingerprint = FoldCommandPathModeTransportFingerprintValue(
        fingerprint,
        static_cast<std::uint64_t>(command.commandPathMode));
    fingerprint = FoldCommandPathModeTransportFingerprintValue(
        fingerprint,
        static_cast<std::uint64_t>(command.axisCount));

    if (command.cncFeedLookahead)
        fingerprint = FoldCommandPathModeTransportFingerprintValue(fingerprint, 0x434E434445ULL);
    if (command.pathCoreFeedExactStop)
        fingerprint = FoldCommandPathModeTransportFingerprintValue(fingerprint, 0x434E434454ULL);

    const int boundedAxisCount =
        (command.axisCount < 0)
        ? 0
        : ((command.axisCount > MAX_AXES)
            ? MAX_AXES
            : command.axisCount);
    for (int slot = 0; slot < boundedAxisCount; ++slot)
    {
        fingerprint = FoldCommandPathModeTransportFingerprintValue(
            fingerprint,
            static_cast<std::uint64_t>(
                static_cast<std::int64_t>(command.axisIndices[slot])));
    }
    if (command.cncCornerBlend)
        fingerprint = FoldCommandPathModeTransportFingerprintValue(fingerprint, 0x434E434448ULL);
    if (command.cncPrefixVelocityPPS != 0.0)
    {
        std::uint64_t prefixBits = 0ULL;
        std::memcpy(&prefixBits, &command.cncPrefixVelocityPPS, sizeof(prefixBits));
        fingerprint = FoldCommandPathModeTransportFingerprintValue(fingerprint, 0x434E43444BULL);
        fingerprint = FoldCommandPathModeTransportFingerprintValue(fingerprint, prefixBits);
    }
    return fingerprint;
}


void MotionCore::ObserveCommandPathModeProducer(
    const MotionCommand& command,
    bool accepted) noexcept
{
    m_pathModeProducerSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);

    if (accepted)
    {
        m_pathModeProducerAccepted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_pathModeProducerRejected.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    if (!accepted)
    {
        m_pathModeProducerSequence.fetch_add(
            1ULL,
            std::memory_order_release);
        return;
    }

    switch (command.commandPathMode)
    {
    case MotionCommandPathMode::EXACT_STOP:
        m_pathModeProducerExactStop.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathMode::CONTINUOUS:
        m_pathModeProducerContinuous.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathMode::UNSPECIFIED:
        m_pathModeProducerUnspecified.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    default:
        m_pathModeProducerInvalid.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    }

    const std::uint64_t producerFingerprint =
        m_pathModeProducerFingerprint.load(std::memory_order_relaxed);
    m_pathModeProducerFingerprint.store(
        FoldCommandPathModeTransportFingerprint(
            producerFingerprint,
            command),
        std::memory_order_relaxed);

    m_pathModeProducerSequence.fetch_add(
        1ULL,
        std::memory_order_release);
}


void MotionCore::CommitCommandPathModeConsumerAuthority(
    const MotionCommand& command,
    MotionCommandPathModeAuthorityDecision decision) noexcept
{
    m_pathModeConsumerSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);

    m_pathModeAuthorityAttempts.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    switch (decision)
    {
    case MotionCommandPathModeAuthorityDecision::APPLY_EXACT_STOP:
        m_Group.pathMode = PathMode::EXACT_STOP;
        m_pathModeAuthorityApplied.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeAuthorityExactStop.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathModeAuthorityDecision::APPLY_CONTINUOUS:
        m_Group.pathMode = PathMode::CONTINUOUS;
        m_pathModeAuthorityApplied.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeAuthorityContinuous.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathModeAuthorityDecision::LEGACY_FALLBACK:
        m_pathModeAuthorityLegacyFallbacks.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathModeAuthorityDecision::REPLAY_BYPASS:
        m_pathModeAuthorityReplayBypasses.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathModeAuthorityDecision::REJECT_DRIVER_OVERRIDE:
        m_pathModeAuthorityDriverBlocks.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeConsumerSequence.fetch_add(
            1ULL,
            std::memory_order_release);
        return;
    case MotionCommandPathModeAuthorityDecision::REJECT_INVALID:
    default:
        m_pathModeAuthorityInvalidRejects.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeConsumerSequence.fetch_add(
            1ULL,
            std::memory_order_release);
        return;
    }

    m_pathModeConsumerCommitted.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    if (command.replayTerminalAlreadyPublished)
    {
        m_pathModeConsumerReplayCommitted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_pathModeConsumerIngressCommitted.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    const std::uint64_t consumerFingerprint =
        m_pathModeConsumerFingerprint.load(std::memory_order_relaxed);
    m_pathModeConsumerFingerprint.store(
        FoldCommandPathModeTransportFingerprint(
            consumerFingerprint,
            command),
        std::memory_order_relaxed);

    switch (command.commandPathMode)
    {
    case MotionCommandPathMode::EXACT_STOP:
        m_pathModeConsumerExactStop.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathMode::CONTINUOUS:
        m_pathModeConsumerContinuous.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        break;
    case MotionCommandPathMode::UNSPECIFIED:
        m_pathModeConsumerUnspecified.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeConsumerSequence.fetch_add(
            1ULL,
            std::memory_order_release);
        return;
    default:
        m_pathModeConsumerInvalid.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeConsumerSequence.fetch_add(
            1ULL,
            std::memory_order_release);
        return;
    }

    if (m_Group.pathMode == PathMode::PATH_SERVO ||
        m_Group.pathMode == PathMode::JUMP_TRACKING)
    {
        m_pathModeDriverOverrideObservations.fetch_add(
            1ULL,
            std::memory_order_relaxed);
        m_pathModeConsumerSequence.fetch_add(
            1ULL,
            std::memory_order_release);
        return;
    }

    const bool matchesAppliedRuntime =
        (command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
            m_Group.pathMode == PathMode::EXACT_STOP) ||
        (command.commandPathMode == MotionCommandPathMode::CONTINUOUS &&
            m_Group.pathMode == PathMode::CONTINUOUS);

    if (matchesAppliedRuntime)
    {
        m_pathModeLegacyMatches.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_pathModeLegacyMismatches.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    m_pathModeConsumerSequence.fetch_add(
        1ULL,
        std::memory_order_release);
}


void MotionCore::ObserveCommandPathModeAuthorityReject(
    MotionCommandPathModeAuthorityDecision decision) noexcept
{
    m_pathModeConsumerSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);
    m_pathModeAuthorityAttempts.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    if (decision ==
        MotionCommandPathModeAuthorityDecision::REJECT_DRIVER_OVERRIDE)
    {
        m_pathModeAuthorityDriverBlocks.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_pathModeAuthorityInvalidRejects.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }

    m_pathModeConsumerSequence.fetch_add(
        1ULL,
        std::memory_order_release);
}


bool MotionCore::RejectFrontCommandForPathModeAuthority(
    const MotionCommand& peekedCommand,
    MotionCommandPathModeAuthorityDecision decision) noexcept
{
    if (HasPendingExecutionEpochChange())
    {
        return false;
    }

    MotionCommand rejectedCommand{};
    if (!m_Group.cmdQueue.ConsumerTryPop(rejectedCommand))
    {
        return false;
    }

    const bool exactPeekedIdentity =
        MotionExecutionIdentityExactlyMatches(
            peekedCommand.execution,
            rejectedCommand.execution) &&
        peekedCommand.ownerLease.owner ==
        rejectedCommand.ownerLease.owner &&
        peekedCommand.ownerLease.generation ==
        rejectedCommand.ownerLease.generation;
    const MotionRejectReason authorizationFailure =
        GetCommandAuthorizationFailure(rejectedCommand);

    if (authorizationFailure != MotionRejectReason::NONE)
    {
        RejectMotionCommand(
            rejectedCommand,
            authorizationFailure,
            0U);
        return true;
    }

    if (!exactPeekedIdentity || HasPendingExecutionEpochChange())
    {
        RejectMotionCommand(
            rejectedCommand,
            MotionRejectReason::STALE_EPOCH,
            0U);
        return true;
    }

    ObserveCommandPathModeAuthorityReject(decision);
    m_p1DroppedAxisRetirementFailureCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    m_p1LastOrphanAxisIndex.store(-1, std::memory_order_release);
    TriggerGroupMappingIntegrityEmergencyStop(-1, true);
    RejectMotionCommand(
        rejectedCommand,
        MotionRejectReason::INVALID_GEOMETRY,
        static_cast<std::uint32_t>(
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
    return true;
}


void MotionCore::RejectNonGeometryProducerMotionCommand(
    MotionCommand command,
    MotionExecutionEpoch executionEpoch,
    MotionCommandSource commandSource,
    const MotionOwnerLease& ownerLease,
    MotionRejectReason rejectReason,
    MotionExecutionIdentity* producedIdentity,
    MotionOwnerLease* producedOwnerLease) noexcept
{
    if (rejectReason != MotionRejectReason::STALE_EPOCH &&
        rejectReason != MotionRejectReason::OWNER_CONFLICT &&
        rejectReason != MotionRejectReason::NOT_READY)
    {
        rejectReason = MotionRejectReason::NOT_READY;
    }

    AssignExecutionIdentity(
        command,
        executionEpoch,
        commandSource,
        ownerLease);

    if (producedIdentity != nullptr)
    {
        *producedIdentity = command.execution;
    }
    if (producedOwnerLease != nullptr)
    {
        *producedOwnerLease = command.ownerLease;
    }

    m_lastRejectedSegmentId.store(
        command.execution.segmentId,
        std::memory_order_relaxed);

    // A producer-side planning baseline can become unavailable without the
    // submitted geometry itself being malformed.  Preserve ordinary Motion
    // accounting and feedback, but do not convert tuple drift / NOT_READY
    // into a mapping-integrity Alarm or an RT Emergency Stop request.
    TryQueueProducerFeedbackNotice(
        command,
        MotionFeedbackType::REJECTED,
        rejectReason,
        0U,
        0.0);

    ObserveCommandPathModeProducer(command, false);
    RecordProgramBlockMotionSubmission(
        command,
        false,
        rejectReason);
}

void MotionCore::RejectInvalidProducerMotionCommand(
    MotionCommand command,
    MotionExecutionEpoch executionEpoch,
    MotionCommandSource commandSource,
    const MotionOwnerLease& ownerLease,
    MotionExecutionIdentity* producedIdentity,
    MotionOwnerLease* producedOwnerLease) noexcept
{
    m_p1InvalidProducerRejectCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    AssignExecutionIdentity(
        command,
        executionEpoch,
        commandSource,
        ownerLease);

    if (producedIdentity != nullptr)
    {
        *producedIdentity = command.execution;
    }
    if (producedOwnerLease != nullptr)
    {
        *producedOwnerLease = command.ownerLease;
    }

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

    ObserveCommandPathModeProducer(command, false);
    RecordProgramBlockMotionSubmission(
        command,
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


// Axis configuration is established before runtime startup. RT only reads its
// own bounded physical context vector; NC letters, role and system mode are
// frozen in the atomic source publication, never fetched through NC pointers.
bool MotionCore::IsNCTranslationAxisIdentityCurrent(
    const NCTranslationSnapshot& snapshot) const noexcept
{
    if (!IsNCAxisIdentitySnapshotValid(snapshot.axisIdentity) ||
        m_pContexts == nullptr || m_pContexts->size() > 8U) return false;
    for (std::size_t slot = 0U; slot < 8U; ++slot)
    {
        const bool exists = slot < m_pContexts->size() && (*m_pContexts)[slot].isExist;
        if ((exists ? 1U : 0U) != snapshot.axisIdentity.exists[slot]) return false;
        // Missing and configured-but-disabled axes use the same canonical
        // absent identity. Do not read GetAxisContext's shared dummy here.
        if (!exists) continue;
        const AxisContext& axis = (*m_pContexts)[slot];
        const std::uint32_t type = static_cast<std::uint32_t>(axis.axisType);
        if (axis.axisIndex != static_cast<int>(slot) ||
            snapshot.axisIdentity.physicalIndexPlusOne[slot] != slot + 1U ||
            type > static_cast<std::uint32_t>(AxisType::ROTARY_CONTINUOUS) ||
            type != snapshot.axisIdentity.axisType[slot]) return false;
    }
    return true;
}


MotionRejectReason MotionCore::GetCommandAuthorizationFailure(
    const MotionCommand& command) const noexcept
{
    if (HasUnacknowledgedSafetyMotionRequest())
    {
        return MotionRejectReason::OWNER_CONFLICT;
    }

    if (!IsCommandFromCurrentEpoch(command))
    {
        return MotionRejectReason::STALE_EPOCH;
    }

    if (!IsCommandOwnerLeaseCurrent(command))
    {
        return MotionRejectReason::OWNER_CONFLICT;
    }

    // A live fixed run requires its exact immutable descriptor on every MEMORY
    // command. Manual/safety sources and unextended legacy G00 remain separate.
    const bool emptyTranslation = IsNCTranslationSnapshotEmpty(command.sourceTranslation);
    if ((!emptyTranslation && (command.execution.source != MotionCommandSource::NC_MEMORY ||
            command.ownerLease.owner != MotionOwner::AUTO ||
            !IsMotionFixedTranslationSourceAllowed(command) ||
            !MatchesNCTranslation(command.sourceTranslation))) ||
        (emptyTranslation && command.execution.source == MotionCommandSource::NC_MEMORY &&
            (GetActiveTranslationGeneration() != 0ULL || command.sourceToolLengthMode != 49 ||
                command.sourceG168Active || command.sourceWCode != 0 || command.sourceG68Active ||
                command.sourceG51Active || command.sourceMirrorMask != 0U || command.sourceG16Active)))
        return MotionRejectReason::NOT_READY;

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

    InvalidateCncLineEndpointProof();
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

    const bool completedPublished = PublishMotionFeedbackForCommand(
        command,
        MotionFeedbackType::COMPLETED,
        MotionRejectReason::NONE,
        0U,
        1.0);

    if (completedPublished &&
        MotionExecutionIdentityExactlyMatches(m_feedbackTrackedIdentity, command.execution) &&
        MotionExecutionIdentityExactlyMatches(m_cncLineEndpointCandidate, command.execution) &&
        m_feedbackTrackedOwnerLease.Matches(command.ownerLease))
        m_cncLineEndpointCompletedSegment.store(command.execution.segmentId, std::memory_order_release);
    m_feedbackTrackedTerminal = true;
}


void MotionCore::AbortTrackedMotionCommand() noexcept
{
    InvalidateCncLineEndpointProof();
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
    InvalidateCncLineEndpointProof();
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
        InvalidateCncLineEndpointProof();
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

        ObserveCommandPathModeProducer(command, false);
        RecordProgramBlockMotionSubmission(
            command,
            false,
            authorizationFailure);
        return false;
    }

    if (m_Group.cmdQueue.ProducerTryPush(command))
    {
        ObserveCommandPathModeProducer(command, true);
        RecordProgramBlockMotionSubmission(
            command,
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

    ObserveCommandPathModeProducer(command, false);
    RecordProgramBlockMotionSubmission(
        command,
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


bool MotionCore::TryPublishOwnerAuthorizedAbortingExecutionEpoch(
    MotionCommandSource source,
    MotionExecutionEpoch expectedEpoch,
    const MotionOwnerLease& expectedOwnerLease,
    MotionExecutionEpoch& publishedEpoch) noexcept
{
    publishedEpoch = MOTION_EXECUTION_EPOCH_INVALID;

    if (expectedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !expectedOwnerLease.IsValid() ||
        ResolveMotionOwnerForSource(source) !=
        expectedOwnerLease.owner ||
        !IsMotionOwnerLeaseCurrent(expectedOwnerLease) ||
        HasUnacknowledgedSafetyMotionRequest())
    {
        return false;
    }

    const std::uint64_t expectedDrainRevocationGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    if (m_executionDrainRevocationPublishersInProgress.load(
        std::memory_order_acquire) != 0U)
    {
        return false;
    }

    const std::uint64_t expectedOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if (!UnpackMotionOwnerState(expectedOwnerState).Matches(
        expectedOwnerLease) ||
        UnpackMotionOwnerSafetyHandshake(expectedOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(expectedOwnerState) ||
        UnpackMotionOwnerSafetyRequestTicket(expectedOwnerState) !=
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire) ||
        (expectedOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL)
    {
        return false;
    }

    // Reserve the packed Owner word before touching the execution Epoch.
    // Safety ticket publication refuses this bit.  Therefore either Safety
    // linearizes first and this exact CAS fails, or AUTO publishes its one
    // ABORTING Epoch first and Safety remains visibly deferred until the bit
    // is released.  No ticket can appear between the owner proof and Epoch
    // CAS.
    const std::uint64_t reservedOwnerState =
        expectedOwnerState | MOTION_OWNER_EPOCH_COMMIT_RESERVED;
    std::uint64_t ownerReservationExpected = expectedOwnerState;
    if (!m_motionOwnerState.compare_exchange_strong(
        ownerReservationExpected,
        reservedOwnerState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    const auto releaseOwnerReservation = [this,
        expectedOwnerState,
        reservedOwnerState]() noexcept -> bool
    {
        std::uint64_t expected = reservedOwnerState;
        if (m_motionOwnerState.compare_exchange_strong(
            expected,
            expectedOwnerState,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return true;
        }
        m_motionOwnerState.fetch_and(
            ~MOTION_OWNER_EPOCH_COMMIT_RESERVED,
            std::memory_order_acq_rel);
        return false;
    };

    // A command producer gets one exact CAS attempt.  It must never retry
    // from a newer Epoch: otherwise a stale AUTO producer could publish after
    // (and supersede) a concurrent Reset or SAFETY lifecycle boundary.
    std::uint64_t expectedPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    if ((expectedPublication &
        EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL ||
        UnpackExecutionEpochPublication(expectedPublication) !=
        expectedEpoch)
    {
        (void)releaseOwnerReservation();
        return false;
    }

    // Close owner transfer before the one-shot Epoch CAS.  If SAFETY takes
    // ownership after this check, its lifecycle publisher either wins first
    // (making this exact CAS fail) or publishes the following Epoch.  This
    // producer never loops past it and RT never waits.
    if (m_motionOwnerState.load(std::memory_order_acquire) !=
        reservedOwnerState ||
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire) !=
        expectedDrainRevocationGeneration ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U)
    {
        (void)releaseOwnerReservation();
        return false;
    }

    const MotionExecutionEpoch nextEpoch =
        (expectedEpoch ==
            (std::numeric_limits<MotionExecutionEpoch>::max)())
        ? 1U
        : static_cast<MotionExecutionEpoch>(expectedEpoch + 1U);
    const std::uint64_t desiredPublication =
        PackExecutionEpochPublication(
            nextEpoch,
            source,
            true,
            true);

    if (!m_executionEpochPublication.compare_exchange_strong(
        expectedPublication,
        desiredPublication,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        (void)releaseOwnerReservation();
        return false;
    }

    publishedEpoch = nextEpoch;

    if (!releaseOwnerReservation())
    {
        return false;
    }

    // The Epoch may have linearized immediately before an owner preemption.
    // In that case do not enqueue under the retired lease; the concurrent
    // SAFETY publisher is ordered after this one-shot publication.
    return
        m_motionOwnerState.load(std::memory_order_acquire) ==
        expectedOwnerState &&
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire) ==
        expectedDrainRevocationGeneration &&
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) == 0U;
}


void MotionCore::DiscardStaleQueuedCommands()
{
    MotionCommand queuedCommand{};

    while (m_staleCommandDiscardBudgetRemaining > 0U)
    {
        // Preserve the exact pre-takeover queue until the Safety action has
        // consumed its causal evidence. Ticket/action publication precedes
        // the SAFETY Epoch, so this acquire gate closes the race even if the
        // Epoch becomes current between two queue observations.
        if (HasUnacknowledgedSafetyMotionRequest())
        {
            return;
        }

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

        if (HasUnacknowledgedSafetyMotionRequest())
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

    const MotionCommandSource pendingSource =
        UnpackExecutionEpochPublicationSource(publication);
    if (pendingSource == MotionCommandSource::SAFETY)
    {
        // An action-bearing Safety request publishes its ticket before the
        // takeover Epoch and leaves ACTION_PENDING set until the RT consumer
        // has an observable mailbox/direct mutation. Do not erase the causal
        // predecessor queue before that action is claimed.
        const std::uint64_t ownerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        if (UnpackMotionOwnerSafetyActionPending(ownerState))
        {
            return;
        }
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
        m_safetyControlledStopInProgress = false;
        m_safetyControlledStopOwnerLease = MotionOwnerLease{};
        m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_safetyControlledStopRequestTicket = 0U;

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
        InvalidateCncLineEndpointProof();
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

    const auto writeTarget =
        [&](int32_t exactValue)
    {
        bool structuredWritten = false;
        if (m_pStructuredServoReadShadowMaster != nullptr)
        {
            structuredWritten =
                m_pStructuredServoReadShadowMaster->
                TryWriteMotionServoOutputCommandStructured(
                    axisIndex,
                    MotionServoOutputCommandField::TargetVelocity,
                    static_cast<int64_t>(exactValue),
                    output);
        }
        if (!structuredWritten)
        {
            output->TargetVelocity = exactValue;
        }
        if (m_pStructuredServoReadShadowMaster != nullptr)
        {
            m_pStructuredServoReadShadowMaster->
                ObserveMotionServoOutputCommandSeamWriteShadow(
                    axisIndex,
                    MotionServoOutputCommandField::TargetVelocity,
                    static_cast<int64_t>(exactValue));
        }
    };

    // Zero is always allowed and never needs a reservation. A non-zero PDO
    // command is a tiny transaction against the same packed Owner word used
    // by Safety request tickets. If a ticket linearizes while the write is in
    // progress it changes the exact word, makes the release CAS fail, and the
    // command is overwritten with zero before this RT call returns.
    if (value == 0)
    {
        writeTarget(0);
        return;
    }

    const std::uint64_t entryPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const MotionOwnerLease entryLease =
        UnpackMotionOwnerState(entryOwnerState);
    const std::uint32_t entryTicket =
        UnpackMotionOwnerSafetyRequestTicket(entryOwnerState);
    const bool controlledStopAuthorized =
        IsSafetyControlledStopAuthorized(axisIndex);
    const bool entryGroupAuthorized =
        !m_Group.isActive ||
        GetCommandAuthorizationFailure(m_Group.currentCmd) ==
        MotionRejectReason::NONE ||
        controlledStopAuthorized;
    if ((entryPublication &
        (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL ||
        (entryOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(entryOwnerState) ||
        (HasPendingSafetyOrRecoveryRequests() &&
            !controlledStopAuthorized) ||
        (entryTicket != m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire) &&
            !controlledStopAuthorized) ||
        (entryLease.owner == MotionOwner::SAFETY &&
            !controlledStopAuthorized) ||
        !entryGroupAuthorized)
    {
        writeTarget(0);
        return;
    }

    const std::uint64_t reservedOwnerState =
        entryOwnerState | MOTION_OWNER_OUTPUT_COMMIT_RESERVED;
    if (!m_motionOwnerState.compare_exchange_strong(
        entryOwnerState,
        reservedOwnerState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        writeTarget(0);
        return;
    }

    int32_t finalValue = value;
    if (m_executionEpochPublication.load(std::memory_order_acquire) !=
        entryPublication ||
        m_motionOwnerState.load(std::memory_order_acquire) !=
        reservedOwnerState)
    {
        finalValue = 0;
    }
    writeTarget(finalValue);

    std::uint64_t expectedReservedState = reservedOwnerState;
    if (!m_motionOwnerState.compare_exchange_strong(
        expectedReservedState,
        reservedOwnerState &
        ~MOTION_OWNER_OUTPUT_COMMIT_RESERVED,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        // A Safety ticket changed the packed word during the output call.
        // Roll the externally visible command back before releasing only our
        // reservation bit; all newer Owner/ticket information is preserved.
        if (finalValue != 0)
        {
            writeTarget(0);
        }
        m_motionOwnerState.fetch_and(
            ~MOTION_OWNER_OUTPUT_COMMIT_RESERVED,
            std::memory_order_acq_rel);
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

bool MotionCore::ApplyMachineHome(
    AxisContext& axis,
    double capturedReferencePulse,
    double homeOffsetUnit,
    const MotionOwnerLease& ownerLease)
{
    if (!axis.isExist ||
        axis.state != MotionState::MotionState_IDLE ||
        !std::isfinite(capturedReferencePulse) ||
        !std::isfinite(homeOffsetUnit) ||
        axis.resolution_PPR <= 0.0 ||
        std::abs(axis.finalLead) < 1.0e-12 ||
        !ownerLease.IsValid() ||
        ownerLease.owner != MotionOwner::HOME ||
        !IsMotionOwnerLeaseCurrent(ownerLease))
    {
        return false;
    }

    const double pulsePerUnit = axis.resolution_PPR / std::abs(axis.finalLead);
    const double homeOffsetPulse = homeOffsetUnit * pulsePerUnit;
    const double oldOffset = axis.machineCoordinateOffsetPulse;
    const double newOffset = capturedReferencePulse - homeOffsetPulse;
    const double shift = oldOffset - newOffset;
    const double oldLogicalCmdPos = axis.logicalCmdPos.Load();
    const double oldLastQueuedPulse = axis.lastQueuedPulse.Load();
    const double shiftedCurrentActPos = axis.currentActPos + shift;
    const double shiftedCurrentCmdPos = axis.currentCmdPos + shift;
    const double shiftedLogicalCmdPos = oldLogicalCmdPos + shift;
    const double shiftedPlanningPos = axis.planningPos + shift;
    const double shiftedFinalTargetPos = axis.finalTargetPos + shift;
    const double shiftedStartCmdPos = axis.startCmdPos + shift;
    const double shiftedLastQueuedPulse = oldLastQueuedPulse + shift;
    const double shiftedLastActPos = axis.lastActPos + shift;

    if (!std::isfinite(pulsePerUnit) ||
        !std::isfinite(homeOffsetPulse) ||
        !std::isfinite(oldOffset) ||
        !std::isfinite(newOffset) ||
        !std::isfinite(shift) ||
        !std::isfinite(oldLogicalCmdPos) ||
        !std::isfinite(oldLastQueuedPulse) ||
        !std::isfinite(shiftedCurrentActPos) ||
        !std::isfinite(shiftedCurrentCmdPos) ||
        !std::isfinite(shiftedLogicalCmdPos) ||
        !std::isfinite(shiftedPlanningPos) ||
        !std::isfinite(shiftedFinalTargetPos) ||
        !std::isfinite(shiftedStartCmdPos) ||
        !std::isfinite(shiftedLastQueuedPulse) ||
        !std::isfinite(shiftedLastActPos))
    {
        return false;
    }

    // The two cross-thread mirrors commit before every RT-private scalar.
    // Each edge is one strong CAS; contention is an ownership breach, not a
    // condition under which the 250 us thread may retry or spin.
    if (!axis.logicalCmdPos.TryCompareExchange(
        oldLogicalCmdPos,
        shiftedLogicalCmdPos))
    {
        EmergencyStopAllAxes();
        return false;
    }
    if (!axis.lastQueuedPulse.TryCompareExchange(
        oldLastQueuedPulse,
        shiftedLastQueuedPulse))
    {
        (void)axis.logicalCmdPos.TryCompareExchange(
            shiftedLogicalCmdPos,
            oldLogicalCmdPos);
        EmergencyStopAllAxes();
        return false;
    }

    // Close owner transfer after the pair commit but before any RT-private
    // field is changed.  Exact rollback is bounded; regardless of rollback
    // success, containment makes the rejected HOME command fail closed.
    if (!IsMotionOwnerLeaseCurrent(ownerLease))
    {
        (void)axis.lastQueuedPulse.TryCompareExchange(
            shiftedLastQueuedPulse,
            oldLastQueuedPulse);
        (void)axis.logicalCmdPos.TryCompareExchange(
            shiftedLogicalCmdPos,
            oldLogicalCmdPos);
        EmergencyStopAllAxes();
        return false;
    }

    axis.machineCoordinateOffsetPulse = newOffset;
    axis.currentActPos = shiftedCurrentActPos;
    axis.currentCmdPos = shiftedCurrentCmdPos;
    axis.planningPos = shiftedPlanningPos;
    axis.finalTargetPos = shiftedFinalTargetPos;
    axis.startCmdPos = shiftedStartCmdPos;
    axis.lastActPos = shiftedLastActPos;

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

    if (!IsMotionOwnerLeaseCurrent(ownerLease))
    {
        EmergencyStopAllAxes();
        return false;
    }
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


bool MotionCore::TryPopIdleHoldDiagnostic(IdleHoldDiagnosticEvent& event) noexcept
{
    // Exactly one consumer: HMI_Bridge::ProcessTask_1000ms, Priority 50.
    return m_idleHoldDiagnostics.events.ConsumerTryPop(event);
}

std::uint32_t MotionCore::GetIdleHoldDiagnosticDroppedCount() const noexcept
{
    return m_idleHoldDiagnostics.dropped.load(std::memory_order_acquire);
}

void MotionCore::QueueIdleHoldDiagnostic(IdleHoldDiagnosticEventType eventType,
    IdleHoldDiagnosticReason reason, int axisIndex,
    MotionOwnerLease nextLease) noexcept
{
    // RT sole producer. Capture immutable values only; never format, allocate,
    // wait, retry, or give a diagnostic failure authority over Motion.
    IdleHoldDiagnostics& diagnostics = m_idleHoldDiagnostics;
    IdleHoldDiagnosticEvent& event = diagnostics.producerEvent;
    const IdlePositionHoldState& hold = m_idlePositionHold;
    event.runtimeTick = m_ncSettleRuntimeCycleTick;
    event.cmdPulseBits = 0ULL;
    event.actPulseBits = 0ULL;
    event.windowPulseBits = 0ULL;
    if (diagnostics.nextSequence != (std::numeric_limits<std::uint64_t>::max)())
    {
        ++diagnostics.nextSequence;
    }
    event.sequence = diagnostics.nextSequence;
    event.epoch = hold.epoch;
    event.generation = hold.lease.generation;
    event.mask = hold.requiredMask;
    event.nextGeneration = nextLease.generation;
    event.axisIndex = axisIndex;
    event.owner = hold.lease.owner;
    event.nextOwner = nextLease.owner;
    event.eventType = eventType;
    event.reason = reason;
    if (eventType == IdleHoldDiagnosticEventType::REFERENCE &&
        axisIndex >= 0 && axisIndex < MAX_AXES && m_pContexts != nullptr &&
        static_cast<std::size_t>(axisIndex) < m_pContexts->size())
    {
        const std::size_t slot = static_cast<std::size_t>(axisIndex);
        std::memcpy(&event.cmdPulseBits, &hold.reference[slot], sizeof(event.cmdPulseBits));
        std::memcpy(&event.actPulseBits, &(*m_pContexts)[slot].currentActPos,
            sizeof(event.actPulseBits));
        std::memcpy(&event.windowPulseBits, &hold.window[slot], sizeof(event.windowPulseBits));
    }
    if (!diagnostics.events.ProducerTryPush(event))
    {
        // Single RT writer: a saturating load/store needs no CAS retry loop.
        const std::uint32_t dropped = diagnostics.dropped.load(std::memory_order_relaxed);
        if (dropped != (std::numeric_limits<std::uint32_t>::max)())
        {
            diagnostics.dropped.store(dropped + 1U, std::memory_order_release);
        }
    }
}

void MotionCore::CancelIdlePositionHold(IdleHoldDiagnosticReason reason, bool fault,
    int axisIndex) noexcept
{
    IdlePositionHoldState& hold = m_idlePositionHold;
    const bool firstCancellation = !hold.cancelled;
    hold.cancelled = true;
    hold.active = false;
    hold.frameMask = 0U;
    (void)ZeroAllServoTargetVelocityForFrame();
    (void)ReleaseMotionOwner(hold.lease);
    if (fault && !AlarmManager::GetInstance().HasAlarm())
    {
        AlarmManager::GetInstance().Trigger(
            AlarmManager::IDLE_POSITION_HOLD_FAILED, 0, axisIndex);
    }
    // All control and alarm actions precede best-effort diagnostics.
    if (firstCancellation)
    {
        QueueIdleHoldDiagnostic(fault ? IdleHoldDiagnosticEventType::FAILED :
            IdleHoldDiagnosticEventType::CANCELLED, reason, axisIndex);
    }
}

void MotionCore::PrepareIdlePositionHoldPass() noexcept
{
    IdlePositionHoldState& hold = m_idlePositionHold;
    hold.frameMask = 0U;
    hold.passRequested = false;
    const MotionOwnerLease lease = GetMotionOwnerLease();
    if (lease.owner != MotionOwner::IDLE_HOLD)
    {
        if (hold.lease.IsValid())
        {
            QueueIdleHoldDiagnostic(IdleHoldDiagnosticEventType::RELEASED,
                IdleHoldDiagnosticReason::NONE, -1, lease);
        }
        hold = IdlePositionHoldState{};
        return;
    }
    hold.passRequested = true;
    if (!hold.lease.Matches(lease))
    {
        hold = IdlePositionHoldState{};
        hold.passRequested = true;
        hold.lease = lease;
        const std::uint64_t grant = m_programEndIdleHoldGrant.load(std::memory_order_acquire);
        hold.epoch = static_cast<MotionExecutionEpoch>(grant);
        if (static_cast<MotionOwnerGeneration>(grant >> 32U) != lease.generation)
        {
            CancelIdlePositionHold(IdleHoldDiagnosticReason::GRANT_MISMATCH, true);
            return;
        }
        hold.requiredMask = BuildExistingNCAxisMask();
    }
    if (hold.cancelled)
    {
        (void)ReleaseMotionOwner(hold.lease);
        return;
    }
    const std::uint64_t publication = m_executionEpochPublication.load(std::memory_order_acquire);
    if (UnpackExecutionEpochPublication(publication) != hold.epoch ||
        (publication & EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL ||
        HasPendingSafetyOrRecoveryRequests() || AlarmManager::GetInstance().HasAlarm())
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::AUTHORITY_CHANGED, false);
        return;
    }
    if ((publication & EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
    {
        return;
    }
    if (m_pContexts == nullptr || m_pDrives == nullptr ||
        m_pContexts->size() != m_pDrives->size() ||
        m_pContexts->size() > MAX_AXES || hold.requiredMask == 0U ||
        BuildExistingNCAxisMask() != hold.requiredMask || m_Group.isActive ||
        m_Group.cmdQueue.ingress_size() != 0U || m_Group.cmdQueue.replay_size() != 0U ||
        !std::isfinite(m_Group.virtualAxis.currentCmdVel) ||
        !std::isfinite(m_Group.virtualAxis.logicalCmdVel) ||
        m_Group.virtualAxis.currentCmdVel != 0.0 ||
        m_Group.virtualAxis.logicalCmdVel != 0.0)
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::GROUP_OR_MAPPING_CHANGED, true);
        return;
    }
    hold.runtimeTick = m_ncSettleRuntimeCycleTick;
}

void MotionCore::UpdateIdlePositionHoldAxis(ServoOutput* output, AxisContext& axis,
    const MotionServoInputSnapshot& input) noexcept
{
    IdlePositionHoldState& hold = m_idlePositionHold;
    WriteServoTargetVelocityCommand(output, axis.axisIndex, 0);
    axis.pid.prevError = axis.pid.integralAcc = 0.0;
    axis.Pid_IDLE.prevError = axis.Pid_IDLE.integralAcc = 0.0;
    axis.Pid_G00.prevError = axis.Pid_G00.integralAcc = 0.0;
    axis.currentActVel = (axis.currentActPos - axis.lastActPos) / CYCLE_TIME_SEC;
    axis.lastActPos = axis.currentActPos;
    if (!hold.passRequested || hold.cancelled || !hold.lease.IsValid() ||
        !IsMotionOwnerLeaseCurrent(hold.lease) ||
        !m_ncSettleRuntimeObserved || !m_ncSettleRuntimeCycleValid ||
        !m_ncSettleRuntimeCycleContiguous ||
        hold.runtimeTick != m_ncSettleRuntimeCycleTick)
    {
        return;
    }
    const std::uint64_t publication = m_executionEpochPublication.load(std::memory_order_acquire);
    if (UnpackExecutionEpochPublication(publication) != hold.epoch ||
        (publication & (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL ||
        HasPendingSafetyOrRecoveryRequests() || AlarmManager::GetInstance().HasAlarm())
    {
        return;
    }
    if ((input.StatusWord & 0x006FU) != 0x0027U ||
        input.ModesOfOperationDisplay != 9 || axis.targetMode != 9 || !axis.isServoOn)
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::SERVO_OR_MODE_LOST, false, axis.axisIndex);
        return;
    }
    if (axis.axisIndex < 0 || axis.axisIndex >= MAX_AXES || output == nullptr ||
        m_pContexts == nullptr || static_cast<std::size_t>(axis.axisIndex) >= m_pContexts->size() ||
        &(*m_pContexts)[axis.axisIndex] != &axis)
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::AXIS_MAPPING, true, axis.axisIndex);
        return;
    }
    const std::size_t index = static_cast<std::size_t>(axis.axisIndex);
    const std::uint32_t bit = 1U << index;
    if (!std::isfinite(axis.currentCompOffset_unit))
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::COMPENSATION_NONFINITE, true, axis.axisIndex);
        return;
    }
    if ((hold.requiredMask & bit) == 0U || axis.isVirtualAxis ||
        axis.axisType != AxisType::LINEAR || axis.fbMode != FeedbackSource::MOTOR_ENCODER ||
        axis.enableBacklash || axis.enablePitch || axis.currentCompOffset_unit != 0.0)
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::UNSUPPORTED_SCOPE, hold.active, axis.axisIndex);
        return;
    }
    if (axis.isFault || axis.isLagAlarm || axis.homeRuntime.active ||
        axis.state != MotionState::MotionState_IDLE ||
        !std::isfinite(axis.currentCmdPos) || !std::isfinite(axis.currentActPos) ||
        !std::isfinite(axis.machineCoordinateOffsetPulse) ||
        !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
        !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
        !std::isfinite(axis.inPositionWindow_Pulse) || axis.inPositionWindow_Pulse <= 0.0 ||
        !std::isfinite(axis.Pid_IDLE.Kp) || axis.Pid_IDLE.Kp <= 0.0 ||
        !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
        !std::isfinite(axis.currentCmdVel) || axis.currentCmdVel != 0.0 ||
        !std::isfinite(axis.logicalCmdVel) || axis.logicalCmdVel != 0.0)
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::AXIS_CONFIG_OR_COMMAND, true, axis.axisIndex);
        return;
    }
    const double unitsPerPulse = axis.finalLead / axis.resolution_PPR;
    const double cap = (std::min)(axis.maxVel_PPS, 0.1 / unitsPerPulse);
    const bool reverse = axis.isReverse != axis.Axis_Reverse;
    if (!std::isfinite(unitsPerPulse) || unitsPerPulse <= 0.0 ||
        !std::isfinite(cap) || cap <= 0.0 ||
        cap > static_cast<double>((std::numeric_limits<std::int32_t>::max)()))
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::VELOCITY_CONVERSION, true, axis.axisIndex);
        return;
    }
    if ((hold.capturedMask & bit) == 0U)
    {
        hold.reference[index] = axis.currentCmdPos;
        hold.unitsPerPulse[index] = unitsPerPulse;
        hold.window[index] = axis.inPositionWindow_Pulse;
        hold.kp[index] = axis.Pid_IDLE.Kp;
        hold.maxVelocity[index] = axis.maxVel_PPS;
        hold.machineOffset[index] = axis.machineCoordinateOffsetPulse;
        if (reverse) hold.reverseMask |= bit;
        hold.capturedMask |= bit;
    }
    if (axis.currentCmdPos != hold.reference[index] ||
        unitsPerPulse != hold.unitsPerPulse[index] ||
        axis.inPositionWindow_Pulse != hold.window[index] ||
        axis.Pid_IDLE.Kp != hold.kp[index] || axis.maxVel_PPS != hold.maxVelocity[index] ||
        axis.machineCoordinateOffsetPulse != hold.machineOffset[index] ||
        reverse != ((hold.reverseMask & bit) != 0U))
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::REFERENCE_OR_CONFIG_CHANGED, true, axis.axisIndex);
        return;
    }
    const double error = hold.reference[index] - axis.currentActPos;
    const double bound = hold.window[index] * (hold.active ? 2.0 : 1.0);
    if (!std::isfinite(error) || !std::isfinite(bound) || std::abs(error) > bound)
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::FOLLOWING_ERROR, true, axis.axisIndex);
        return;
    }
    double velocity = error * hold.kp[index];
    if (!std::isfinite(velocity))
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::CORRECTION_NONFINITE, true, axis.axisIndex);
        return;
    }
    velocity = (std::max)(-cap, (std::min)(cap, velocity));
    if (m_pCoordMgr != nullptr) m_pCoordMgr->UpdateSoftwareTravelLimitState(axis);
    const bool positiveAllowed = !axis.hardLimitPositive &&
        (m_pCoordMgr == nullptr || m_pCoordMgr->CanMoveSoftwarePositive(axis));
    const bool negativeAllowed = !axis.hardLimitNegative &&
        (m_pCoordMgr == nullptr || m_pCoordMgr->CanMoveSoftwareNegative(axis));
    if ((velocity > 0.0 && !positiveAllowed) || (velocity < 0.0 && !negativeAllowed))
    {
        CancelIdlePositionHold(IdleHoldDiagnosticReason::TRAVEL_LIMIT, true, axis.axisIndex);
        return;
    }
    if (reverse) velocity = -velocity;
    WriteServoTargetVelocityCommand(output, axis.axisIndex, static_cast<std::int32_t>(velocity));
    hold.frameMask |= bit;
    if (!hold.active && hold.frameMask == hold.requiredMask &&
        hold.capturedMask == hold.requiredMask)
    {
        hold.active = true;
        QueueIdleHoldDiagnostic(IdleHoldDiagnosticEventType::ACTIVE);
        for (std::size_t slot = 0U; slot < MAX_AXES; ++slot)
        {
            if ((hold.requiredMask & (1U << slot)) == 0U) continue;
            QueueIdleHoldDiagnostic(IdleHoldDiagnosticEventType::REFERENCE,
                IdleHoldDiagnosticReason::NONE, static_cast<int>(slot));
        }
    }
}

bool MotionCore::IsIdlePositionHoldImageCurrent(std::uint64_t ownerState,
    std::uint64_t executionPublication) const noexcept
{
    const IdlePositionHoldState& hold = m_idlePositionHold;
    const std::uint64_t grant = m_programEndIdleHoldGrant.load(std::memory_order_acquire);
    return hold.passRequested && hold.active && !hold.cancelled && hold.requiredMask != 0U &&
        hold.frameMask == hold.requiredMask && hold.capturedMask == hold.requiredMask &&
        hold.lease.owner == MotionOwner::IDLE_HOLD &&
        UnpackMotionOwnerState(ownerState).Matches(hold.lease) &&
        UnpackExecutionEpochPublication(executionPublication) == hold.epoch &&
        grant == ((static_cast<std::uint64_t>(hold.lease.generation) << 32U) | hold.epoch) &&
        !UnpackMotionOwnerSafetyHandshake(ownerState) &&
        !UnpackMotionOwnerSafetyActionPending(ownerState) &&
        UnpackMotionOwnerSafetyRequestTicket(ownerState) ==
        m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire) &&
        m_ncSettleRuntimeObserved && m_ncSettleRuntimeCycleValid &&
        m_ncSettleRuntimeCycleContiguous && hold.runtimeTick == m_ncSettleRuntimeCycleTick &&
        !m_Group.isActive && m_Group.cmdQueue.ingress_size() == 0U &&
        m_Group.cmdQueue.replay_size() == 0U && !HasPendingSafetyOrRecoveryRequests();
}


void MotionCore::InvalidateServoOutputImageProof() noexcept
{
    ++m_servoOutputImageProofGeneration;
    m_servoOutputImageProof = ServoOutputImageProof{};
    m_servoOutputImageProof.generation =
        m_servoOutputImageProofGeneration;
}


bool MotionCore::ZeroAllServoTargetVelocityForFrame() noexcept
{
    if (m_pDrives == nullptr)
    {
        return false;
    }

    const std::size_t boundedDriveCount =
        (std::min)(
            m_pDrives->size(),
            static_cast<std::size_t>(MAX_AXES));
    bool complete = m_pDrives->size() <= MAX_AXES;
    for (std::size_t slot = 0U;
        slot < boundedDriveCount;
        ++slot)
    {
        ENI_ServoDrive& drive = (*m_pDrives)[slot];
        if (drive.pOutput == nullptr)
        {
            complete = false;
            continue;
        }

        const int axisIndex =
            m_pContexts != nullptr && slot < m_pContexts->size()
            ? (*m_pContexts)[slot].axisIndex
            : static_cast<int>(slot);
        WriteServoTargetVelocityCommand(
            drive.pOutput,
            axisIndex,
            0);
    }

    return complete;
}


bool MotionCore::PublishServoOutputImageProof(
    std::uint64_t ownerState,
    std::uint64_t executionPublication,
    std::uint64_t frameSafetyIntentState,
    std::uint64_t alarmSafetyIntentState,
    std::uint32_t alarmUpdateCount,
    ServoOutputImageProofMode mode) noexcept
{
    ServoOutputImageProof proof{};
    proof.ownerState = ownerState;
    proof.executionPublication = executionPublication;
    proof.frameSafetyIntentState = frameSafetyIntentState;
    proof.admissionCorrectionGeneration = mode == ServoOutputImageProofMode::NORMAL
        ? m_pathAdmissionCorrectionFrameGeneration : 0ULL;
    proof.alarmSafetyIntentState = alarmSafetyIntentState;
    proof.alarmUpdateCount = alarmUpdateCount;
    proof.generation = ++m_servoOutputImageProofGeneration;
    proof.mode = mode;

    if (m_pDrives == nullptr ||
        m_pDrives->size() > MAX_AXES ||
        m_pDrives->size() > 32U)
    {
        proof.mode = ServoOutputImageProofMode::INVALID;
        m_servoOutputImageProof = proof;
        return false;
    }

    for (std::size_t slot = 0U;
        slot < m_pDrives->size();
        ++slot)
    {
        const ENI_ServoDrive& drive = (*m_pDrives)[slot];
        if (drive.pOutput == nullptr)
        {
            proof.mode = ServoOutputImageProofMode::INVALID;
            m_servoOutputImageProof = proof;
            return false;
        }

        proof.slotMask |=
            static_cast<std::uint32_t>(1U << slot);
        proof.targetVelocity[slot] =
            drive.pOutput->TargetVelocity;
    }

    m_servoOutputImageProof = proof;
    return true;
}


bool MotionCore::BeginServoOutputFrameAtSendPoint(
    ServoOutputFrameReservation& reservation) noexcept
{
    reservation = ServoOutputFrameReservation{};
    const ServoOutputImageProof proof =
        m_servoOutputImageProof;
    const std::uint64_t entrySafetyIntentState =
        m_frameSafetyIntentState.load(
            std::memory_order_acquire);

    const auto scrubAndInvalidate = [this]() noexcept -> bool
    {
        const bool scrubbed =
            ZeroAllServoTargetVelocityForFrame();
        InvalidateServoOutputImageProof();
        return scrubbed;
    };

    if (proof.mode == ServoOutputImageProofMode::INVALID ||
        proof.generation == 0ULL ||
        static_cast<std::uint32_t>(entrySafetyIntentState) != 0U ||
        m_pDrives == nullptr ||
        m_pDrives->size() > MAX_AXES ||
        m_pDrives->size() > 32U)
    {
        return scrubAndInvalidate();
    }

    std::uint32_t observedMask = 0U;
    bool observedNonZero = false;
    for (std::size_t slot = 0U;
        slot < m_pDrives->size();
        ++slot)
    {
        const ENI_ServoDrive& drive = (*m_pDrives)[slot];
        if (drive.pOutput == nullptr)
        {
            return scrubAndInvalidate();
        }

        observedMask |=
            static_cast<std::uint32_t>(1U << slot);
        const std::int32_t targetVelocity =
            drive.pOutput->TargetVelocity;
        if (targetVelocity != proof.targetVelocity[slot])
        {
            return scrubAndInvalidate();
        }
        observedNonZero = observedNonZero || targetVelocity != 0;
    }

    if (observedMask != proof.slotMask)
    {
        return scrubAndInvalidate();
    }

    if (!observedNonZero)
    {
        return
            proof.mode == ServoOutputImageProofMode::ZERO_ONLY
            ? true
            : scrubAndInvalidate();
    }

    if (proof.mode != ServoOutputImageProofMode::NORMAL &&
        proof.mode != ServoOutputImageProofMode::CONTROLLED_STOP &&
        proof.mode != ServoOutputImageProofMode::IDLE_HOLD)
    {
        return scrubAndInvalidate();
    }

    // Owner/Epoch can remain byte-identical when a bounded Safety producer
    // is blocked and completes only a short revocation publication.  Bind
    // every non-zero image to the full sequence+active word captured by the
    // Motion pass so such an incident permanently invalidates the old proof.
    if (entrySafetyIntentState != proof.frameSafetyIntentState)
    {
        return scrubAndInvalidate();
    }

    std::uint64_t baseOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t baseExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    if (baseOwnerState != proof.ownerState ||
        baseExecutionPublication != proof.executionPublication ||
        UnpackMotionOwnerSafetyHandshake(baseOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(baseOwnerState) ||
        (baseOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        (baseExecutionPublication &
            (EXECUTION_EPOCH_PUBLICATION_PENDING |
                EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL)
    {
        return scrubAndInvalidate();
    }

    if (proof.mode == ServoOutputImageProofMode::IDLE_HOLD)
    {
        if (!IsIdlePositionHoldImageCurrent(
            baseOwnerState, baseExecutionPublication))
        {
            return scrubAndInvalidate();
        }
    }
    else if (proof.mode == ServoOutputImageProofMode::NORMAL)
    {
        if (proof.admissionCorrectionGeneration != 0ULL &&
            proof.admissionCorrectionGeneration != m_pathHoldGeneration.load(std::memory_order_acquire))
            return scrubAndInvalidate();
        const MotionOwnerLease lease =
            UnpackMotionOwnerState(baseOwnerState);
        if (lease.owner == MotionOwner::NONE ||
            lease.owner == MotionOwner::SAFETY ||
            lease.owner == MotionOwner::IDLE_HOLD ||
            UnpackMotionOwnerSafetyRequestTicket(baseOwnerState) !=
            m_safetyRequestAcknowledgedTicket.load(
                std::memory_order_acquire) ||
            HasPendingSafetyOrRecoveryRequests())
        {
            return scrubAndInvalidate();
        }
    }
    else
    {
        for (std::size_t slot = 0U;
            slot < m_pDrives->size();
            ++slot)
        {
            if (proof.targetVelocity[slot] == 0)
            {
                continue;
            }

            if (m_pContexts == nullptr ||
                slot >= m_pContexts->size() ||
                !IsSafetyControlledStopAuthorized(
                    (*m_pContexts)[slot].axisIndex))
            {
                return scrubAndInvalidate();
            }
        }
    }

    AlarmManager& frameAlarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation alarmAdmission{};
    if (!frameAlarms.BeginMotionAdmission(
        proof.alarmUpdateCount,
        alarmAdmission) ||
        alarmAdmission.baseState != proof.alarmSafetyIntentState)
    {
        if (alarmAdmission.acquired)
        {
            (void)frameAlarms.EndMotionAdmission(alarmAdmission);
        }
        return scrubAndInvalidate();
    }

    const std::uint64_t reservedOwnerState =
        baseOwnerState | MOTION_OWNER_FRAME_SEND_RESERVED;
    if (!m_motionOwnerState.compare_exchange_strong(
        baseOwnerState,
        reservedOwnerState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        (void)frameAlarms.EndMotionAdmission(alarmAdmission);
        return scrubAndInvalidate();
    }

    std::uint64_t expectedExecutionPublication =
        baseExecutionPublication;
    const std::uint64_t reservedExecutionPublication =
        baseExecutionPublication |
        EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED;
    if (!m_executionEpochPublication.compare_exchange_strong(
        expectedExecutionPublication,
        reservedExecutionPublication,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        const bool scrubbed = ZeroAllServoTargetVelocityForFrame();
        m_motionOwnerState.fetch_and(
            ~MOTION_OWNER_FRAME_SEND_RESERVED,
            std::memory_order_acq_rel);
        (void)frameAlarms.EndMotionAdmission(alarmAdmission);
        InvalidateServoOutputImageProof();
        return scrubbed;
    }

    reservation.alarmAdmission = alarmAdmission;
    reservation.baseOwnerState = baseOwnerState;
    reservation.reservedOwnerState = reservedOwnerState;
    reservation.baseExecutionPublication =
        baseExecutionPublication;
    reservation.reservedExecutionPublication =
        reservedExecutionPublication;
    reservation.safetyIntentState =
        entrySafetyIntentState;
    reservation.acquired = true;
    return true;
}


bool MotionCore::FinalizeServoOutputFrameAtSendPoint(
    ServoOutputFrameReservation& reservation) noexcept
{
    if (!reservation.acquired)
    {
        return true;
    }

    if ((m_servoOutputImageProof.mode == ServoOutputImageProofMode::IDLE_HOLD &&
        !IsIdlePositionHoldImageCurrent(reservation.baseOwnerState,
            reservation.baseExecutionPublication)) ||
        (m_servoOutputImageProof.mode == ServoOutputImageProofMode::NORMAL &&
            m_servoOutputImageProof.admissionCorrectionGeneration != 0ULL &&
            m_servoOutputImageProof.admissionCorrectionGeneration !=
            m_pathHoldGeneration.load(std::memory_order_acquire)) ||
        m_motionOwnerState.load(std::memory_order_acquire) !=
        reservation.reservedOwnerState ||
        m_executionEpochPublication.load(
            std::memory_order_acquire) !=
        reservation.reservedExecutionPublication ||
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire) != 0U ||
        !AlarmManager::GetInstance().IsMotionAdmissionCurrent(
            reservation.alarmAdmission) ||
        // Keep this exact packed-state read as the final atomic observation
        // before frame commit. BeginRevocation changes sequence+active count
        // in one RMW, so Frame Begin can never capture a half-published
        // Safety intent.
        m_frameSafetyIntentState.load(
            std::memory_order_acquire) !=
        reservation.safetyIntentState)
    {
        const bool scrubbed = ZeroAllServoTargetVelocityForFrame();
        reservation.recopyRequired = true;
        m_motionOwnerState.fetch_and(
            ~MOTION_OWNER_FRAME_SEND_RESERVED,
            std::memory_order_acq_rel);
        std::uint64_t expectedReservedExecution =
            reservation.reservedExecutionPublication;
        if (!m_executionEpochPublication.compare_exchange_strong(
            expectedReservedExecution,
            reservation.baseExecutionPublication,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            m_executionEpochPublication.fetch_and(
                ~EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED,
                std::memory_order_acq_rel);
        }
        InvalidateServoOutputImageProof();
        (void)AlarmManager::GetInstance().EndMotionAdmission(
            reservation.alarmAdmission);
        reservation.acquired = false;
        return scrubbed;
    }

    // Keep both reservations through the actual NIC SendPacket call. Safety
    // ticket publication refuses FRAME_SEND_RESERVED, and ordinary lifecycle
    // publishers refuse the execution commit bit, so the physical frame is
    // ordered before every request that begins while SendPacket is in flight.
    return true;
}


void MotionCore::EndServoOutputFrameAfterSend(
    ServoOutputFrameReservation& reservation) noexcept
{
    if (!reservation.acquired)
    {
        return;
    }

    std::uint64_t expectedReservedOwner =
        reservation.reservedOwnerState;
    if (!m_motionOwnerState.compare_exchange_strong(
        expectedReservedOwner,
        reservation.baseOwnerState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_motionOwnerState.fetch_and(
            ~MOTION_OWNER_FRAME_SEND_RESERVED,
            std::memory_order_acq_rel);
    }

    std::uint64_t expectedReservedExecution =
        reservation.reservedExecutionPublication;
    if (!m_executionEpochPublication.compare_exchange_strong(
        expectedReservedExecution,
        reservation.baseExecutionPublication,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_executionEpochPublication.fetch_and(
            ~EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED,
            std::memory_order_acq_rel);
    }

    (void)AlarmManager::GetInstance().EndMotionAdmission(
        reservation.alarmAdmission);

    reservation.acquired = false;
}


// 檔案：MotionCore.cpp
void MotionCore::UpdateAllMotion()//更新全部軸狀態 逐步激磁
{
    m_pathAdmissionCorrectionFrameGeneration = 0ULL;
    InvalidateServoOutputImageProof();
    AlarmManager& motionAlarms = AlarmManager::GetInstance();
    const std::uint64_t imageEntryAlarmSafetyIntentState =
        motionAlarms.GetMotionSafetyIntentState();
    const std::uint32_t imageEntryAlarmUpdateCount =
        motionAlarms.GetUpdateCount();
    const bool imageEntryAlarmPresent = motionAlarms.HasAlarm();
    const std::uint64_t imageEntrySafetyIntentState =
        m_frameSafetyIntentState.load(std::memory_order_acquire);
    const std::uint64_t imageEntryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t imageEntryExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    m_ncSettleMotionPassCompleted = false;
    PrepareIdlePositionHoldPass();

    // 1. 防呆：確保指標沒丟失
    if (m_pDrives == nullptr || m_pContexts == nullptr) {
        if (ZeroAllServoTargetVelocityForFrame())
        {
            (void)PublishServoOutputImageProof(
                m_motionOwnerState.load(std::memory_order_acquire),
                m_executionEpochPublication.load(
                    std::memory_order_acquire),
                m_frameSafetyIntentState.load(
                    std::memory_order_acquire),
                motionAlarms.GetMotionSafetyIntentState(),
                motionAlarms.GetUpdateCount(),
                ServoOutputImageProofMode::ZERO_ONLY);
        }
        PublishStartupLagArmingEvidence();
        PublishStopSettleEvidence();
        return;
    }

    // 2. 防呆：確保兩個清單長度一致
    if (m_pDrives->size() != m_pContexts->size()) {
        // 這裡可以丟個錯誤 log
        if (ZeroAllServoTargetVelocityForFrame())
        {
            (void)PublishServoOutputImageProof(
                m_motionOwnerState.load(std::memory_order_acquire),
                m_executionEpochPublication.load(
                    std::memory_order_acquire),
                m_frameSafetyIntentState.load(
                    std::memory_order_acquire),
                motionAlarms.GetMotionSafetyIntentState(),
                motionAlarms.GetUpdateCount(),
                ServoOutputImageProofMode::ZERO_ONLY);
        }
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





    const std::uint64_t imageExitExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t imageExitOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t imageExitSafetyIntentState =
        m_frameSafetyIntentState.load(std::memory_order_acquire);
    const std::uint64_t imageExitAlarmSafetyIntentState =
        motionAlarms.GetMotionSafetyIntentState();
    const std::uint32_t imageExitAlarmUpdateCount =
        motionAlarms.GetUpdateCount();
    const bool imageExitAlarmPresent = motionAlarms.HasAlarm();
    bool imageHasNonZeroTargetVelocity = false;
    bool imageMappingComplete =
        m_pDrives->size() <= MAX_AXES &&
        m_pDrives->size() <= 32U;
    for (std::size_t slot = 0U;
        slot < m_pDrives->size();
        ++slot)
    {
        if ((*m_pDrives)[slot].pOutput == nullptr)
        {
            imageMappingComplete = false;
            continue;
        }
        imageHasNonZeroTargetVelocity =
            imageHasNonZeroTargetVelocity ||
            (*m_pDrives)[slot].pOutput->TargetVelocity != 0;
    }

    ServoOutputImageProofMode imageProofMode =
        ServoOutputImageProofMode::INVALID;
    bool imageAuthorized =
        imageMappingComplete && motionInputComplete;
    if (!imageHasNonZeroTargetVelocity)
    {
        imageProofMode = ServoOutputImageProofMode::ZERO_ONLY;
    }
    else
    {
        imageAuthorized = imageAuthorized &&
            imageEntryAlarmSafetyIntentState ==
            imageExitAlarmSafetyIntentState &&
            static_cast<std::uint32_t>(
                imageExitAlarmSafetyIntentState) == 0U &&
            imageEntryAlarmUpdateCount == imageExitAlarmUpdateCount &&
            !imageEntryAlarmPresent &&
            !imageExitAlarmPresent &&
            imageEntrySafetyIntentState ==
            imageExitSafetyIntentState &&
            static_cast<std::uint32_t>(
                imageExitSafetyIntentState) == 0U &&
            imageEntryOwnerState == imageExitOwnerState &&
            imageEntryExecutionPublication ==
            imageExitExecutionPublication &&
            !UnpackMotionOwnerSafetyHandshake(
                imageExitOwnerState) &&
            !UnpackMotionOwnerSafetyActionPending(
                imageExitOwnerState) &&
            (imageExitOwnerState &
                MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL &&
            (imageExitExecutionPublication &
                (EXECUTION_EPOCH_PUBLICATION_PENDING |
                    EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) == 0ULL;

        const MotionOwnerLease imageLease =
            UnpackMotionOwnerState(imageExitOwnerState);
        if (imageLease.owner == MotionOwner::SAFETY)
        {
            imageProofMode =
                ServoOutputImageProofMode::CONTROLLED_STOP;
            for (std::size_t slot = 0U;
                slot < m_pDrives->size();
                ++slot)
            {
                if ((*m_pDrives)[slot].pOutput == nullptr ||
                    (*m_pDrives)[slot].pOutput->TargetVelocity == 0)
                {
                    continue;
                }
                imageAuthorized = imageAuthorized &&
                    slot < m_pContexts->size() &&
                    IsSafetyControlledStopAuthorized(
                        (*m_pContexts)[slot].axisIndex);
            }
        }
        else if (imageLease.owner == MotionOwner::IDLE_HOLD)
        {
            imageProofMode = ServoOutputImageProofMode::IDLE_HOLD;
            imageAuthorized = imageAuthorized &&
                IsIdlePositionHoldImageCurrent(imageExitOwnerState,
                    imageExitExecutionPublication);
        }
        else
        {
            imageProofMode = ServoOutputImageProofMode::NORMAL;
            imageAuthorized = imageAuthorized &&
                imageLease.owner != MotionOwner::NONE &&
                UnpackMotionOwnerSafetyRequestTicket(
                    imageExitOwnerState) ==
                m_safetyRequestAcknowledgedTicket.load(
                    std::memory_order_acquire) &&
                !HasPendingSafetyOrRecoveryRequests();
        }
    }

    if (m_pathAdmissionCorrectionFrameGeneration != 0ULL &&
        m_pathAdmissionCorrectionFrameGeneration != m_pathHoldGeneration.load(std::memory_order_acquire))
        imageAuthorized = false;

    if (m_idlePositionHold.passRequested &&
        !IsIdlePositionHoldImageCurrent(imageExitOwnerState,
            imageExitExecutionPublication))
    {
        imageAuthorized = false;
    }

    if (!imageAuthorized)
    {
        if (ZeroAllServoTargetVelocityForFrame())
        {
            (void)PublishServoOutputImageProof(
                m_motionOwnerState.load(std::memory_order_acquire),
                m_executionEpochPublication.load(
                    std::memory_order_acquire),
                m_frameSafetyIntentState.load(
                    std::memory_order_acquire),
                motionAlarms.GetMotionSafetyIntentState(),
                motionAlarms.GetUpdateCount(),
                ServoOutputImageProofMode::ZERO_ONLY);
        }
    }
    else
    {
        (void)PublishServoOutputImageProof(
            imageExitOwnerState,
            imageExitExecutionPublication,
            imageExitSafetyIntentState,
            imageExitAlarmSafetyIntentState,
            imageExitAlarmUpdateCount,
            imageProofMode);
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

// Compatibility utility: currentPos, targetPos and modulo must share one unit.
// Motion pulse admission uses TryResolveMotionTargetPulse instead.
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
bool MotionCore::MoveToPosition(AxisContext& axis, double targetPos, double targetVel, double acc_time, double dec_time)
{

    // Resolve against the same start that the existing idle/error rebase uses.
    // Validate first: failure must not alter any axis trajectory or position.
    const bool rebaseStart = axis.state == MotionState::MotionState_IDLE ||
        axis.state == MotionState::MotionState_ERROR;
    const double motionStartPulse = rebaseStart ? axis.currentActPos : axis.currentCmdPos;
    const bool rotaryShortestPath =
        axis.axisType == AxisType::ROTARY && axis.useShortestPath;
    double pulsePerUnit = 1.0;
    if (!std::isfinite(targetVel) || !std::isfinite(acc_time) ||
        !std::isfinite(dec_time) || !std::isfinite(axis.maxVel_PPS) ||
        axis.maxVel_PPS <= 0.0 ||
        (rotaryShortestPath && !TryGetMotionPulsePerUnit(
            axis.resolution_PPR, axis.finalLead, true, pulsePerUnit)) ||
        !TryResolveMotionTargetPulse(motionStartPulse, targetPos, pulsePerUnit,
            rotaryShortestPath, axis.rotaryModulo, targetPos))
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, 0, axis.axisIndex);
        return false;
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
    // Commit this value only after derived acceleration validation below.

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

    if (!std::isfinite(calc_acc) || !std::isfinite(calc_dec))
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, 0, axis.axisIndex);
        return false;
    }
    if (rebaseStart)
    {
        axis.currentCmdPos = motionStartPulse;
        axis.logicalCmdPos = motionStartPulse;
        axis.currentCmdVel = 0.0;
    }
    axis.programmedVel_PPS = targetVel;

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
    return true;
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
        double stopDist =
            (currentSpeed * currentSpeed) /
            (2.0 * axis.dec_PPS2);

        // A continuous segment may carry a non-zero handover velocity. A
        // RESET stop is always P0: it must reach zero rather than inherit a
        // previous P1 endpoint speed.
        axis.targetEndVel = 0.0;

        // currentCmdVel is normally the correct sign. During a filtered
        // S-curve tail it can already be near zero while logical/target
        // velocity still owns the planned direction, so use the same bounded
        // fallback hierarchy that selected currentSpeed above.
        double stopDirectionVelocity = axis.currentCmdVel;
        if (std::abs(stopDirectionVelocity) < 0.1)
        {
            stopDirectionVelocity = axis.logicalCmdVel;
        }
        if (std::abs(stopDirectionVelocity) < 0.1)
        {
            stopDirectionVelocity = axis.targetVelocity;
        }

        if (stopDirectionVelocity > 0.0)
        {
            double proposedTarget =
                axis.currentCmdPos + stopDist;

            // Calc_Trajectory_Trapezoidal() chooses direction from
            // finalTargetPos - planningPos. Usually currentCmdPos is the
            // correct filtered output anchor. If filter lag makes that
            // proposed endpoint cross the raw planning coordinate, preserve
            // the original direction by moving it to the positive side of
            // planningPos before the next trajectory step.
            if (proposedTarget <= axis.planningPos)
            {
                proposedTarget = axis.planningPos + stopDist;
            }
            axis.finalTargetPos = proposedTarget;
        }
        else if (stopDirectionVelocity < 0.0)
        {
            double proposedTarget =
                axis.currentCmdPos - stopDist;

            // Symmetric negative-direction guard. Do not let a filtered
            // output/planner offset turn a deceleration request into a
            // reverse-direction trajectory on the following 250 us pass.
            if (proposedTarget >= axis.planningPos)
            {
                proposedTarget = axis.planningPos - stopDist;
            }
            axis.finalTargetPos = proposedTarget;
        }
        else
        {
            axis.finalTargetPos =
                axis.planningPos;
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
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    const MotionExecutionEpoch causalEpoch =
        GetCurrentExecutionEpoch();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    m_emergencyStopRequestAttemptCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);
    if (TryPublishEmergencyStopMailbox(causalEpoch))
    {
        m_emergencyStopRequestPublishedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_emergencyStopRequestCoalescedCount.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    EmergencyStopAllAxesImpl(false, causalEpoch);
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
}

void MotionCore::EmergencyStopAllAxesImpl(
    bool forceExecutionInvalidation,
    MotionExecutionEpoch causalExecutionEpoch) noexcept
{
    // A real Alarm/E-stop supersedes a benign operator RESET stop before it
    // can be claimed or resumed. The normal Reset flow will later use its
    // explicit zero-output safety batch; it must never resurrect this
    // controlled-deceleration request after emergency containment.
    SupersedeResetControlledStop();

    // =========================================================
    // 1. 使目前執行世代失效
    //
    // Emergency 訊號可能每個 Scan 都持續呼叫本函式。只有仍有
    // Active / Queued Motion 時才切換 Epoch，避免空機時無限遞增。
    // =========================================================
    const MotionExecutionEpoch runtimeEpochBeforeStop =
        GetCurrentExecutionEpoch();
    const MotionExecutionEpoch executionEpochBeforeStop =
        causalExecutionEpoch != MOTION_EXECUTION_EPOCH_INVALID
        ? causalExecutionEpoch
        : runtimeEpochBeforeStop;
    MotionCommand queuedCommand{};
    // Alarm blocks NC dispatch and the queue is FIFO. After the first
    // invalidation, a stale front therefore proves the remaining visible
    // backlog belongs to the retired Epoch; repeated E-stop requests drain it
    // with the bounded stale budget but must not publish another Epoch.
    const bool queuedExecutionCurrent =
        TryPeekNextMotionCommand(queuedCommand) &&
        queuedCommand.execution.IsAssigned() &&
        queuedCommand.execution.epoch == executionEpochBeforeStop;
    const bool hadRuntimeExecutionToInvalidate =
        forceExecutionInvalidation ||
        m_Group.isActive ||
        queuedExecutionCurrent;

    // Safety 以新 Generation 搶占控制權；重複急停時保持同一份 Lease。
    // Capture the current-Epoch work first: after this takeover an AUTO
    // command is intentionally owner-conflicted and cannot prove whether the
    // stop still needs one (and only one) Epoch invalidation.
    MotionOwnerLease safetyLease = GetMotionOwnerLease();
    if (safetyLease.owner != MotionOwner::SAFETY ||
        !IsMotionOwnerLeaseCurrent(safetyLease))
    {
        const std::uint64_t ownerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        safetyLease = TryTakeSafetyMotionOwnerForTicket(
            UnpackMotionOwnerSafetyRequestTicket(ownerState));
    }

    MotionExecutionEpoch executionEpochAfterStop =
        GetCurrentExecutionEpoch();

    // A fresh takeover handshake is itself the one causal invalidation. Do
    // not publish a second SAFETY Epoch: a late same-generation helper would
    // otherwise move current past the exact J.6 from->to evidence. Only when
    // SAFETY already owned the unchanged Epoch and live work still exists is
    // one additional invalidation required.
    if (hadRuntimeExecutionToInvalidate &&
        executionEpochAfterStop == executionEpochBeforeStop)
    {
        executionEpochAfterStop = BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
        if (executionEpochAfterStop == MOTION_EXECUTION_EPOCH_INVALID)
        {
            const std::uint64_t publication =
                m_executionEpochPublication.load(
                    std::memory_order_acquire);
            if (UnpackExecutionEpochPublicationSource(publication) ==
                MotionCommandSource::SAFETY)
            {
                executionEpochAfterStop =
                    UnpackExecutionEpochPublication(publication);
            }
        }
    }

    const bool recordedExecutionInvalidation =
        hadRuntimeExecutionToInvalidate &&
        safetyLease.IsValid() &&
        safetyLease.owner == MotionOwner::SAFETY &&
        executionEpochBeforeStop != MOTION_EXECUTION_EPOCH_INVALID &&
        executionEpochAfterStop != MOTION_EXECUTION_EPOCH_INVALID &&
        executionEpochAfterStop != executionEpochBeforeStop;
    if (recordedExecutionInvalidation)
    {
        ++m_emergencyStopEpochInvalidationCount;
    }

    // 防止 UpdateInterpolation 繼續對實體軸寫入新的命令。
    m_Group.isActive = false;
    m_safetyControlledStopInProgress = false;
    m_safetyControlledStopOwnerLease = MotionOwnerLease{};
    m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_safetyControlledStopRequestTicket = 0U;


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
        recordedExecutionInvalidation;

    // Stage NC-0.2J.6.3.2: preserve the last application that *really*
    // invalidated execution.  Level-sensitive C5/axis-protection inputs can
    // apply E-stop again after the group is already stopped; those no-op
    // applications must not overwrite the only exact old-Epoch -> new-Epoch
    // correlation available to the later 10 ms J.6 observer.
    if (recordedExecutionInvalidation)
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
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    if (!EnsureSafetyMotionActionTicket(safetyRequestTicket))
    {
        EmergencyStopAllAxesImpl(true);
        EndExecutionDrainAcknowledgementRevocation();
        return;
    }
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    ResetAllFaultsImpl(true);
    (void)CompleteSafetyMotionActionTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
}

void MotionCore::ResetAllFaultsImpl(
    bool publishExecutionEpoch)
{
    // Fault-reset/batch recovery owns a whole-PDO zero-output contract and
    // therefore supersedes any pre-ticket smooth RESET request.
    SupersedeResetControlledStop();

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
    m_safetyControlledStopInProgress = false;
    m_safetyControlledStopOwnerLease = MotionOwnerLease{};
    m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_safetyControlledStopRequestTicket = 0U;
    // Future Command 由新 Epoch 在 250 us Consumer 端淘汰。
    ResetFault(m_Group.virtualAxis); // 👈 現在有了 isVirtualAxis 保護，這裡也安全了！

    if (m_Group.jumpManager.state != JumpState::IDLE) {
        m_Group.jumpManager.state = JumpState::IDLE;
        m_Group.jumpManager.currentOffset = 0.0;
        m_Group.jumpManager.jumpVel = 0.0;
    }
}
void MotionCore::InvalidateCncLineEndpointProof() noexcept
{
    m_cncLineEndpointCandidate = MotionExecutionIdentity{};
    m_cncLineEndpointCompletedSegment.store(MOTION_SEGMENT_ID_INVALID, std::memory_order_release);
}

// Runtime-only scope for the G90 queued LINEAR terminal, including fixed XY rotation.
// A tangent ARC/blend predecessor may transfer a sub-cycle distance into the
// original loaded line frame. Only the stopped natural terminal can prove it.
bool MotionCore::IsCncLineEndpointScope() const noexcept
{
    const MotionCommand& source = m_Group.currentCmd;
    const auto& lookahead = m_cncFeedLookahead;
    if (!source.cncFeedLookahead || source.mode != InterpolationMode::LINEAR ||
        source.commandPathMode != MotionCommandPathMode::CONTINUOUS ||
        m_Group.pathMode != PathMode::CONTINUOUS ||
        source.sourceTranslation.distanceMode != 90 || !source.sourceIsAbsoluteMode ||
        source.sourceG162Active || !IsMotionFixedTranslationSourceAllowed(source) ||
        !lookahead.loaded || lookahead.blendEntered || source.cncCornerBlend ||
        m_Group.virtualAxis.targetEndVel != 0.0 ||
        !MotionExecutionIdentityExactlyMatches(lookahead.identity, source.execution) ||
        !lookahead.lease.Matches(source.ownerLease) ||
        !std::isfinite(lookahead.entryCarry) || lookahead.entryCarry < 0.0 ||
        !IsMotionCommandConsumerGeometryValid(source, m_pContexts)) return false;
    if (lookahead.handoffFrom == MOTION_SEGMENT_ID_INVALID)
    {
        if (lookahead.entryCarry != 0.0) return false;
    }
    else if (lookahead.handoffFrom == (std::numeric_limits<MotionSegmentId>::max)() ||
        source.execution.segmentId != lookahead.handoffFrom + 1ULL)
        return false;

    // The LINEAR loader uses the predecessor's immutable native endpoint for a
    // carried start. Unlike ARC packets, LINEAR mem_* is not a canonical frame.
    // Rebuild its original length in the loader's hypot order; do not infer
    // completion from the virtual finalTargetPos overwritten on entering IDLE.
    if (lookahead.entryCarry != 0.0)
    {
        if (m_Group.mode != source.mode || m_Group.axisCount != source.axisCount)
            return false;
        double distance = 0.0;
        for (int slot = 0; slot < source.axisCount; ++slot)
        {
            const double delta = source.targetPos[slot] - m_Group.startPos[slot];
            if (m_Group.axisIndices[slot] != source.axisIndices[slot] ||
                !std::isfinite(m_Group.startPos[slot]) || !std::isfinite(delta)) return false;
            distance = std::hypot(distance, delta);
        }
        if (!std::isfinite(distance) || lookahead.entryCarry >= distance) return false;
    }
    return true;
}

// A queued circle retains its original canonical native frame even when its
// first RT sample carries distance from a tangent predecessor. Only its stopped
// natural terminal may establish the endpoint proof; a nonzero exit cannot.
bool MotionCore::IsCncArcEndpointScope() const noexcept
{
    const MotionCommand& source = m_Group.currentCmd;
    const auto& lookahead = m_cncFeedLookahead;
    NCPathCoreArcPulseGeometry circle{};
    if (!source.cncFeedLookahead || !source.pathCorePlanarCircle ||
        source.cncCornerBlend || source.commandPathMode != MotionCommandPathMode::CONTINUOUS ||
        m_Group.pathMode != PathMode::CONTINUOUS ||
        source.sourceTranslation.distanceMode != 90 || !source.sourceIsAbsoluteMode ||
        !lookahead.loaded || lookahead.blendEntered ||
        !MotionExecutionIdentityExactlyMatches(lookahead.identity, source.execution) ||
        !lookahead.lease.Matches(source.ownerLease) ||
        m_Group.virtualAxis.targetEndVel != 0.0 ||
        !std::isfinite(lookahead.entryCarry) || lookahead.entryCarry < 0.0 ||
        !IsMotionCommandConsumerGeometryValid(source, m_pContexts) ||
        !ResolveCncPlanarCircle(source, m_pContexts, circle) || lookahead.entryCarry >= circle.lengthPulse)
        return false;
    if (lookahead.handoffFrom == MOTION_SEGMENT_ID_INVALID)
    {
        if (lookahead.entryCarry != 0.0) return false;
    }
    else if (lookahead.handoffFrom == (std::numeric_limits<MotionSegmentId>::max)() ||
        source.execution.segmentId != lookahead.handoffFrom + 1ULL)
        return false;
    return m_Group.mode == source.mode && m_Group.axisCount == 2 &&
        m_Group.axisIndices[0] == 0 && m_Group.axisIndices[1] == 1 &&
        std::memcmp(m_Group.startPos, source.mem_startPos, 2U * sizeof(double)) == 0 &&
        m_Group.centerX == source.centerPos[0] && m_Group.centerY == source.centerPos[1] &&
        m_Group.radius == circle.radius && m_Group.startAngle == circle.startAngle &&
        m_Group.totalAngle == circle.sweepRadians && m_Group.totalDist3D == circle.lengthPulse;
}

// Called only with the exact RT drain acknowledgement and lifecycle reservation.
// currentCmd is then stable. The marker belongs to that immutable packet: every
// command replacement invalidates it BEFORE replacing any part of the packet.
bool MotionCore::HasCompletedCncLineEndpointProof(const MotionCommand& source,
    const NCTranslationSnapshot& previous, MotionExecutionEpoch epoch) const noexcept
{
    NCPathCoreArcPulseGeometry circle{};
    const bool queuedLine = source.mode == InterpolationMode::LINEAR &&
        !source.pathCorePlanarCircle && !source.pathCoreFullCircle &&
        IsMotionCommandConsumerGeometryValid(source, m_pContexts);
    const bool queuedArc = source.pathCorePlanarCircle &&
        (source.mode == InterpolationMode::CIRCULAR_CW || source.mode == InterpolationMode::CIRCULAR_CCW) &&
        IsMotionCommandConsumerGeometryValid(source, m_pContexts) && ResolveCncPlanarCircle(source, m_pContexts, circle);
    if (!source.execution.IsAssigned() || source.execution.epoch != epoch ||
        source.execution.source != MotionCommandSource::NC_MEMORY ||
        !source.ownerLease.IsValid() || source.ownerLease.owner != MotionOwner::AUTO ||
        m_cncLineEndpointCompletedSegment.load(std::memory_order_acquire) != source.execution.segmentId ||
        !source.cncFeedLookahead || (!queuedLine && !queuedArc) ||
        source.commandPathMode != MotionCommandPathMode::CONTINUOUS ||
        source.axisCount < 1 || source.axisCount > 3 || source.cncCornerBlend ||
        source.pathCoreRetainedTraversal ||
        source.pathCoreRetainedReverse || source.replayTerminalAlreadyPublished ||
        !IsNCTranslationSnapshotValid(source.sourceTranslation) ||
        source.sourceTranslation.distanceMode != 90 ||
        source.sourceTranslation.generation > previous.generation ||
        source.sourceTranslation.revision > previous.revision ||
        previous.generation - source.sourceTranslation.generation !=
            previous.revision - source.sourceTranslation.revision) return false;
    // Same-generation proof has no intervening selector that could explain a
    // changed source. Older generations are reconciled only by the sanctioned
    // selected-frame fields below; native geometry and EXT remain immutable.
    if (source.sourceTranslation.generation == previous.generation &&
        !SameNCTranslationSnapshot(source.sourceTranslation, previous)) return false;
    NCTranslationSnapshot expected = source.sourceTranslation;
    // Standalone units/stroke/distance/WCS/H/G68/WORK/scale/mirror/polar/cutter selections preserve the accepted native endpoint.
    // The old immutable packet still owns its completion marker. Only these
    // selected-frame fields may differ; EXT remains frozen.
    // The original queued packet must be G90; fixed rotation requires canonical XY geometry.
    expected.distanceMode = previous.distanceMode;
    expected.unitsMode = previous.unitsMode;
    expected.polarMode = previous.polarMode;
    expected.storedStrokeMode = previous.storedStrokeMode;
    expected.cutterMode = previous.cutterMode;
    expected.cutterD = previous.cutterD;
    expected.cutterRadiusMM = previous.cutterRadiusMM;
    expected.wcsCode = previous.wcsCode;
    std::memcpy(expected.wcsOffsetMM, previous.wcsOffsetMM, sizeof(expected.wcsOffsetMM));
    expected.toolLengthMode = previous.toolLengthMode;
    expected.toolHCode = previous.toolHCode;
    std::memcpy(expected.toolOffsetMM, previous.toolOffsetMM, sizeof(expected.toolOffsetMM));
    expected.rotationMode = previous.rotationMode;
    expected.rotationPlane = previous.rotationPlane;
    std::memcpy(expected.rotationCenterMM, previous.rotationCenterMM, sizeof(expected.rotationCenterMM));
    expected.rotationAngleDeg = previous.rotationAngleDeg;
    expected.workMode = previous.workMode;
    expected.workWCode = previous.workWCode;
    std::memcpy(expected.workOffset, previous.workOffset, sizeof(expected.workOffset));
    std::memcpy(expected.workRotationCenterMM, previous.workRotationCenterMM,
        sizeof(expected.workRotationCenterMM));
    expected.scalingMode = previous.scalingMode;
    expected.scalingFactor = previous.scalingFactor;
    std::memcpy(expected.scalingCenterMM, previous.scalingCenterMM, sizeof(expected.scalingCenterMM));
    expected.mirrorMask = previous.mirrorMask;
    std::memcpy(expected.mirrorCenterMM, previous.mirrorCenterMM, sizeof(expected.mirrorCenterMM));
    expected.generation = previous.generation;
    expected.revision = previous.revision;
    return SameNCTranslationSnapshot(expected, previous);
}

// Fixed XYZ NC lines must finish on the same native target bits their producer
// committed, including G90 before a later G91 distance-mode handoff. This scope
// excludes replay/EDM and accepts only the separately proved queued LINEAR scope.
bool MotionCore::IsFixedPlanarLineEndpointScope() const noexcept
{
    const MotionCommand& source = m_Group.currentCmd;
    if (m_pContexts == nullptr || !m_Group.isActive ||
        m_Group.mode != InterpolationMode::LINEAR || source.mode != InterpolationMode::LINEAR ||
        (!IsCncLineEndpointScope() &&
            (source.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
                m_Group.pathMode != PathMode::EXACT_STOP || source.cncFeedLookahead)) ||
        source.axisCount < 1 || source.axisCount > 3 || source.axisCount != m_Group.axisCount ||
        !source.execution.IsAssigned() || source.execution.source != MotionCommandSource::NC_MEMORY ||
        !source.ownerLease.IsValid() || source.ownerLease.owner != MotionOwner::AUTO ||
        !IsNCTranslationSnapshotValid(source.sourceTranslation) ||
        source.cncCornerBlend || source.pathCorePlanarCircle ||
        source.pathCoreFullCircle || source.pathCoreRetainedTraversal || source.pathCoreRetainedReverse ||
        source.replayTerminalAlreadyPublished || m_Group.enableHistory || m_Group.enableTransform ||
        m_Group.jumpManager.state != JumpState::IDLE || m_pathHold.sourceSeen ||
        m_safetyControlledStopInProgress || IsPathCoreHoldExcursionDriving()) return false;
    unsigned mask = 0U;
    for (int slot = 0; slot < source.axisCount; ++slot)
    {
        const int axis = source.axisIndices[slot];
        if (axis < 0 || axis > 2 || static_cast<std::size_t>(axis) >= m_pContexts->size() ||
            m_Group.axisIndices[slot] != axis || (mask & (1U << axis)) != 0U ||
            !(*m_pContexts)[axis].isExist || (*m_pContexts)[axis].axisType != AxisType::LINEAR) return false;
        mask |= 1U << axis;
    }
    return true;
}

bool MotionCore::TryCompleteFixedPlanarLineEndpoint(AxisCommand& command) noexcept
{
    AxisContext& virtualAxis = m_Group.virtualAxis;
    const MotionCommand& source = m_Group.currentCmd;
    if (!IsFixedPlanarLineEndpointScope() || virtualAxis.state != MotionState::MotionState_IDLE)
        return false;
    if (HasPendingSafetyOrRecoveryRequests() ||
        GetCommandAuthorizationFailure(source) != MotionRejectReason::NONE) return false;
    LifecycleCommitReservationGuard endpointCommit(*this, source.execution);
    if (!endpointCommit.IsAcquired() || HasPendingSafetyOrRecoveryRequests() ||
        GetCommandAuthorizationFailure(source) != MotionRejectReason::NONE) return false;

    // IDLE canonicalization overwrites finalTargetPos with currentCmdPos.
    // Rebuild the ORIGINAL loaded length from its unchanged start and packet
    // targets, using exactly the LoadNextCommand hypot order. Never trust that
    // overwritten scalar as proof of reaching the program endpoint.
    double distance = 0.0;
    bool valid = virtualAxis.isVirtualAxis && virtualAxis.inPosition &&
        !virtualAxis.isFault && !virtualAxis.isLagAlarm &&
        std::isfinite(command.instantCmdPos) && std::isfinite(command.instantCmdVel) &&
        virtualAxis.currentCmdPos == command.instantCmdPos &&
        virtualAxis.planningPos == virtualAxis.currentCmdPos &&
        virtualAxis.finalTargetPos == virtualAxis.currentCmdPos &&
        virtualAxis.currentCmdVel == 0.0 && virtualAxis.logicalCmdVel == 0.0 &&
        virtualAxis.targetVelocity == 0.0 && virtualAxis.targetEndVel == 0.0 &&
        command.instantCmdVel == 0.0 && virtualAxis.bufferSum == 0.0 &&
        std::all_of(virtualAxis.velBuffer.begin(), virtualAxis.velBuffer.end(),
            [](double velocity) { return velocity == 0.0; });
    for (int slot = 0; valid && slot < source.axisCount; ++slot)
    {
        const double delta = source.targetPos[slot] - m_Group.startPos[slot];
        valid = std::isfinite(source.targetPos[slot]) && std::isfinite(m_Group.startPos[slot]) &&
            std::isfinite(delta) && std::isfinite(m_Group.ratio[slot]);
        distance = std::hypot(distance, delta);
        valid = valid && std::isfinite(distance);
    }
    const double error = std::abs(distance - command.instantCmdPos);
    // Same finite relative/physical rounding limits as the existing EC arc
    // completion. This closes numerical residue only, never a partial stop.
    valid = valid && std::isfinite(error) && error <= 1e-12 * distance;
    for (int slot = 0; valid && slot < source.axisCount; ++slot)
    {
        const AxisContext& axis = (*m_pContexts)[source.axisIndices[slot]];
        const double delta = source.targetPos[slot] - m_Group.startPos[slot];
        const double mapped = m_Group.startPos[slot] + command.instantCmdPos * m_Group.ratio[slot];
        const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
        const double scalarErrorMM = error / pulsePerMM;
        const double axisErrorMM = std::abs(source.targetPos[slot] - mapped) / pulsePerMM;
        valid = !axis.isFault && !axis.isLagAlarm &&
            std::isfinite(axis.resolution_PPR) && axis.resolution_PPR > 0.0 &&
            std::isfinite(axis.finalLead) && axis.finalLead > 0.0 &&
            std::isfinite(pulsePerMM) && pulsePerMM > 0.0 && std::isfinite(mapped) &&
            (distance > 0.0 ? m_Group.ratio[slot] == delta / distance : delta == 0.0) &&
            std::isfinite(scalarErrorMM) && scalarErrorMM <= 5e-8 &&
            std::isfinite(axisErrorMM) && axisErrorMM <= 5e-8;
    }
    if (!valid)
    {
        endpointCommit.Release();
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        return false;
    }
    // All axes and the complete lifecycle tuple passed before the first write.
    virtualAxis.currentCmdPos = distance;
    virtualAxis.planningPos = distance;
    virtualAxis.finalTargetPos = distance;
    command.instantCmdPos = distance;
    for (int slot = 0; slot < source.axisCount; ++slot)
    {
        AxisContext& axis = (*m_pContexts)[source.axisIndices[slot]];
        axis.logicalCmdPos = source.targetPos[slot];
        axis.logicalCmdVel = 0.0;
    }
    if (source.cncFeedLookahead)
        m_cncLineEndpointCandidate = source.execution;
    return true;
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

    // DR: the sub-pulse endpoint shortcut must not create a raw CNC
    // velocity above the active cruise/curve ceiling. Keep the existing
    // step-reachable and nonzero-seam cases; defer only an over-limit
    // tolerance snap to the normal acceleration/deceleration planner.
    if (planDist < 0.001 && planDist > stepDist &&
        axis.isVirtualAxis && &axis == &m_Group.virtualAxis && m_Group.isActive &&
        m_Group.currentCmd.cncFeedLookahead &&
        m_Group.pathMode == PathMode::CONTINUOUS &&
        !m_Group.enableHistory && !m_Group.enableTransform &&
        m_Group.jumpManager.state == JumpState::IDLE && !m_pathHold.sourceSeen &&
        !m_safetyControlledStopInProgress && !IsPathCoreHoldExcursionDriving() &&
        std::abs(axis.targetEndVel) <= 0.1 &&
        planDist / dt > (std::max)(0.0, axis.cruiseVel_PPS))
    {
        isPlanDone = false;
    }


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
            if (axis.isVirtualAxis && &axis == &m_Group.virtualAxis &&
                m_Group.currentCmd.cncFeedLookahead && !m_safetyControlledStopInProgress)
            {
                // DE never jumps to an endpoint speed or re-accelerates through HOLD.
                const double target = (std::min)(std::abs(axis.targetEndVel), axis.cruiseVel_PPS);
                const double old = std::abs(axis.currentCmdVel);
                const double next = old > target ? (std::max)(target, old - axis.dec_PPS2 * dt) :
                    (std::min)(target, old + axis.acc_PPS2 * dt);
                axis.currentCmdVel = dir * next;
            }
            else axis.currentCmdVel = dir * std::abs(axis.targetEndVel);
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

        const auto& prefix = m_cncFeedLookahead;
        const bool djPrefix = axis.isVirtualAxis && &axis == &m_Group.virtualAxis &&
            prefix.loaded && prefix.prefixEnabled && m_Group.currentCmd.cncCornerBlend &&
            MotionExecutionIdentityExactlyMatches(prefix.identity, m_Group.currentCmd.execution) &&
            prefix.lease.Matches(m_Group.currentCmd.ownerLease) &&
            !m_safetyControlledStopInProgress && !HasPendingExecutionEpochChange() &&
            !HasPendingSafetyOrRecoveryRequests() &&
            GetCommandAuthorizationFailure(m_Group.currentCmd) == MotionRejectReason::NONE &&
            m_Group.pathMode == PathMode::CONTINUOUS && !m_Group.enableHistory && !m_Group.enableTransform &&
            m_Group.jumpManager.state == JumpState::IDLE && !m_pathHold.sourceSeen && !IsPathCoreHoldExcursionDriving();
        if (djPrefix)
        {
            // Bound against BOTH raw and filtered progress. A full peak-speed
            // filter window is consumed at arc speed before the true arc entry.
            const double position = (std::max)(axis.planningPos, axis.currentCmdPos);
            const double distance = (std::max)(0.0, prefix.prefixLength - position - prefix.prefixReserve);
            const double ddt = axis.dec_PPS2 * dt;
            const double square = axis.maxVel_PPS * axis.maxVel_PPS +
                2.0 * axis.dec_PPS2 * distance + ddt * ddt;
            if (std::isfinite(position) && std::isfinite(square) && square >= 0.0 &&
                std::isfinite(m_Group.feedrateOverride))
            {
                const double allowed = (std::max)(axis.maxVel_PPS, std::sqrt(square) - ddt);
                const double overrideValue = (std::max)(0.0, (std::min)(1.0, m_Group.feedrateOverride));
                max_v = (std::min)(prefix.prefixLimit, allowed) * overrideValue;
            }
        }

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
        double brakingDistance = planDist;
        if (axis.isVirtualAxis && &axis == &m_Group.virtualAxis &&
            m_Group.currentCmd.cncFeedLookahead && !m_safetyControlledStopInProgress && v_end > 0.1)
        {
            // DE reserves a full inherited velocity-filter window before a
            // nonzero seam. The filtered output must not overtake a raw
            // deceleration that was planned only against the geometric end.
            const double reserve = djPrefix ? prefix.prefixReserve :
                axis.maxVel_PPS * dt * (double(axis.velBuffer.size()) + 4.0);
            brakingDistance = (std::max)(0.0, planDist - reserve);
        }
        double max_allowable_vel = std::sqrt(v_end * v_end + 2.0 * dec * brakingDistance);


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
        if (axis.isVirtualAxis && &axis == &m_Group.virtualAxis &&
            m_Group.currentCmd.cncFeedLookahead && !m_safetyControlledStopInProgress)
        {
            auto& zero = m_cncFeedLookahead.zeroInputCycles;
            if (axis.currentCmdVel != 0.0) zero = 0U;
            else if (zero < axis.velBuffer.size()) ++zero;
            if (zero >= axis.velBuffer.size())
            {
                // Every slot has really received zero; remove only accumulated
                // floating-sum residue, without scanning or snapping position.
                axis.bufferSum = 0.0;
                finalOutputVel = 0.0;
            }
        }
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
        // DT and fixed XYZ NC lines: the sub-PPS FIR tail is still
        // motion, including G90 ordinary G00 before G91. Drain it before IDLE can
        // erase the final samples and invalidate the next sparse endpoint.
        const MotionCommand& dtCommand = m_Group.currentCmd;
        const bool dtFeedTail = isHandoverReady && axis.isVirtualAxis &&
            &axis == &m_Group.virtualAxis && m_Group.isActive &&
            (dtCommand.pathCoreFeedExactStop || IsFixedPlanarLineEndpointScope()) &&
            dtCommand.mode == InterpolationMode::LINEAR &&
            dtCommand.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
            m_Group.pathMode == PathMode::EXACT_STOP &&
            !dtCommand.cncFeedLookahead && !dtCommand.cncCornerBlend &&
            !dtCommand.pathCorePlanarCircle && !dtCommand.pathCoreFullCircle &&
            !dtCommand.pathCoreRetainedTraversal && !dtCommand.pathCoreRetainedReverse &&
            !dtCommand.replayTerminalAlreadyPublished &&
            dtCommand.execution.IsAssigned() && dtCommand.execution.source == MotionCommandSource::NC_MEMORY &&
            dtCommand.ownerLease.IsValid() && dtCommand.ownerLease.owner == MotionOwner::AUTO &&
            !m_Group.enableHistory && !m_Group.enableTransform &&
            m_Group.jumpManager.state == JumpState::IDLE && !m_pathHold.sourceSeen &&
            !m_safetyControlledStopInProgress && !IsPathCoreHoldExcursionDriving() &&
            !HasPendingSafetyOrRecoveryRequests() &&
            GetCommandAuthorizationFailure(dtCommand) == MotionRejectReason::NONE;
        // EC: ordinary native plane arcs have the same sub-PPS FIR tail as
        // DT lines. Only this exact NC source may extend the drain gate;
        // controlled stops, retained/EDM paths and queued arcs keep theirs.
        const bool ecArcTail = isHandoverReady && axis.isVirtualAxis &&
            &axis == &m_Group.virtualAxis && m_Group.isActive &&
            dtCommand.pathCorePlanarCircle && !dtCommand.pathCoreFeedExactStop &&
            (dtCommand.mode == InterpolationMode::CIRCULAR_CW ||
                dtCommand.mode == InterpolationMode::CIRCULAR_CCW) &&
            m_Group.mode == dtCommand.mode &&
            IsMotionArcPlaneGroupMapping(dtCommand, m_Group.axisCount, m_Group.axisIndices) &&
            dtCommand.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
            m_Group.pathMode == PathMode::EXACT_STOP &&
            !dtCommand.cncFeedLookahead && !dtCommand.cncCornerBlend &&
            !dtCommand.pathCoreRetainedTraversal && !dtCommand.pathCoreRetainedReverse &&
            !dtCommand.replayTerminalAlreadyPublished &&
            dtCommand.execution.IsAssigned() && dtCommand.execution.source == MotionCommandSource::NC_MEMORY &&
            dtCommand.ownerLease.IsValid() && dtCommand.ownerLease.owner == MotionOwner::AUTO &&
            !m_Group.enableHistory && !m_Group.enableTransform &&
            m_Group.jumpManager.state == JumpState::IDLE && !m_pathHold.sourceSeen &&
            !m_safetyControlledStopInProgress && !IsPathCoreHoldExcursionDriving() &&
            !HasPendingSafetyOrRecoveryRequests() &&
            GetCommandAuthorizationFailure(dtCommand) == MotionRejectReason::NONE;
        if (isHandoverReady && axis.isVirtualAxis &&
            &axis == &m_Group.virtualAxis && m_Group.isActive &&
            (dtFeedTail || ecArcTail || m_Group.currentCmd.pathCoreRetainedTraversal || m_Group.currentCmd.cncFeedLookahead ||
                IsPathCoreHoldExcursionDriving() ||
                (m_pathHold.sourceSeen &&
                    (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::ARMED ||
                        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::COMPLETE) &&
                    IsPathCoreHoldSourceCurrent())) &&
            !m_safetyControlledStopInProgress && axis.velBuffer.size() > 1)
        {
            // BZ_FIX1: sub-pulse paths can have a filtered velocity below
            // 1 PPS while their last step is still inside the filter. Drain
            // every pending sample before IDLE erases the remaining tail.
            // CR_FIX2: a bound NORMAL source is ARMED, not excursion-driving.
            // CS: COMPLETE also uses the ordinary planner for the original
            // remainder after its exact return. Both must drain the same tail
            // before the next retained source uses the command endpoint.
            // DG marked paths also drain the remaining sub-PPS samples at a
            // zero junction, before a canonical queued circle may start.
            // Keep the exact source/owner/safety guards; do not snap axes or
            // relax geometry checks.
            // An endpoint clamp has already cleared this buffer and passes.
            isHandoverReady = std::all_of(axis.velBuffer.begin(), axis.velBuffer.end(),
                [](double velocity) { return velocity == 0.0; });
        }
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
MotionPathCoreAdmissionCorrectionDecision MotionCore::ResolvePathCoreAdmissionPositionCorrection(
    AxisContext& axis, const AxisCommand& command,
    const MotionServoInputSnapshot& input, double& velocityPPS) noexcept
{
    using Decision = MotionPathCoreAdmissionCorrectionDecision;
    using Reason = MotionPathCoreAdmissionCorrectionReason;
    velocityPPS = 0.0;
    auto& s = m_pathHold.status;
    if (!s.admissionPending || axis.axisIndex < 0 || axis.axisIndex >= MAX_AXES ||
        (m_pathAdmissionCorrectionScopeMask & (1U << static_cast<unsigned>(axis.axisIndex))) == 0U)
        return Decision::NOT_APPLICABLE;

    const auto finish = [this, &s, &axis, &velocityPPS](Decision decision, Reason reason) noexcept
    {
        if (axis.axisIndex == s.admissionWaitAxis)
        {
            s.admissionCorrectionTick = m_ncSettleRuntimeCycleTick;
            s.admissionCorrectionDecision = decision;
            s.admissionCorrectionReason = reason;
            s.admissionCorrectionKp = axis.Pid_IDLE.Kp;
            s.admissionCorrectionVelocityPPS = velocityPPS;
            if (decision == Decision::AUTHORIZED)
            {
                if (s.admissionCorrectionCycles == 0ULL)
                    s.admissionCorrectionFirstIntegral = axis.pid.integralAcc;
                if (s.admissionCorrectionCycles != (std::numeric_limits<std::uint32_t>::max)())
                    ++s.admissionCorrectionCycles;
            }
        }
        if (decision == Decision::AUTHORIZED && axis.axisIndex >= 0 && axis.axisIndex < MAX_AXES)
            s.admissionCorrectionMask |= 1U << static_cast<unsigned>(axis.axisIndex);
        return decision;
    };
    if (!m_ncSettleRuntimeObserved || !m_ncSettleRuntimeCycleValid ||
        !m_ncSettleRuntimeCycleContiguous || m_ncSettleRuntimeCycleTick == 0ULL ||
        s.admissionWaitTick != m_ncSettleRuntimeCycleTick)
        return finish(Decision::BLOCKED, Reason::STALE_RUNTIME);
    if (m_Group.isActive || m_pathHold.sourceSeen || m_pathHold.startPending ||
        s.phase != MotionPathCoreHoldExcursionPhase::ARMED ||
        m_pathHold.generation == 0ULL ||
        m_pathHold.generation != m_pathHoldGeneration.load(std::memory_order_acquire) ||
        s.requestGeneration != m_pathHold.generation ||
        s.ownerLease.owner != MotionOwner::AUTO || !IsMotionOwnerLeaseCurrent(s.ownerLease) ||
        s.identity.epoch != GetCurrentExecutionEpoch() || HasPendingExecutionEpochChange() ||
        HasPendingSafetyOrRecoveryRequests() || AlarmManager::GetInstance().HasAlarm())
        return finish(Decision::BLOCKED, Reason::AUTHORITY);
    if (m_pContexts == nullptr || axis.axisIndex < 0 || axis.axisIndex >= MAX_AXES ||
        static_cast<std::size_t>(axis.axisIndex) >= m_pContexts->size() ||
        &(*m_pContexts)[axis.axisIndex] != &axis ||
        !TryPeekNextMotionCommand(m_pathHoldAdmissionFront) ||
        !MotionExecutionIdentityExactlyMatches(m_pathHoldAdmissionFront.execution, s.identity) ||
        !m_pathHoldAdmissionFront.ownerLease.Matches(s.ownerLease) ||
        GetCommandAuthorizationFailure(m_pathHoldAdmissionFront) != MotionRejectReason::NONE ||
        !IsMotionCommandConsumerGeometryValid(m_pathHoldAdmissionFront, m_pContexts) ||
        s.holdRequestSequence != 0ULL || s.completedHoldRequestSequence != 0ULL ||
        s.retreatCount != 0ULL || s.returnCount != 0ULL)
        return finish(Decision::BLOCKED, Reason::SOURCE_IDENTITY);
    if (!MotionCommandHasAxis(m_pathHoldAdmissionFront, axis.axisIndex))
        return finish(Decision::BLOCKED, Reason::SOURCE_IDENTITY);
    if (axis.isVirtualAxis || axis.axisType != AxisType::LINEAR ||
        axis.fbMode != FeedbackSource::MOTOR_ENCODER || axis.enablePitch || axis.enableBacklash ||
        !std::isfinite(axis.currentCompOffset_unit) || axis.currentCompOffset_unit != 0.0)
        return finish(Decision::BLOCKED, Reason::AXIS_SCOPE);
    if ((input.StatusWord & 0x006FU) != 0x0027U || input.ModesOfOperationDisplay != 9 ||
        axis.targetMode != 9 || !axis.isServoOn)
        return finish(Decision::BLOCKED, Reason::RAW_SERVO);
    if (!IsPathCoreAdmissionWaitAxisHealthy(axis) || axis.homeRuntime.active)
        return finish(Decision::BLOCKED, Reason::AXIS_STATE);
    if (axis.currentCmdVel != 0.0 || axis.logicalCmdVel != 0.0 ||
        axis.targetVelocity != 0.0 || axis.targetEndVel != 0.0 ||
        axis.currentCmdPos != axis.logicalCmdPos || axis.currentCmdPos != axis.planningPos ||
        axis.currentCmdPos != axis.finalTargetPos || command.instantCmdPos != axis.currentCmdPos ||
        command.instantCmdVel != 0.0)
        return finish(Decision::BLOCKED, Reason::COMMAND_CHANGED);
    if (!std::isfinite(axis.Pid_IDLE.Kp) || axis.Pid_IDLE.Kp <= 0.0 ||
        !std::isfinite(axis.pid.integralAcc) || !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
        !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
        !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0)
        return finish(Decision::BLOCKED, Reason::NUMERIC);
    const double unitsPerPulse = axis.finalLead / axis.resolution_PPR;
    const double cap = (std::min)(axis.maxVel_PPS, 0.1 / unitsPerPulse);
    const double error = axis.currentCmdPos - axis.currentActPos;
    const double proportional = error * axis.Pid_IDLE.Kp;
    if (!std::isfinite(unitsPerPulse) || unitsPerPulse <= 0.0 ||
        !std::isfinite(cap) || cap <= 0.0 ||
        cap > static_cast<double>((std::numeric_limits<std::int32_t>::max)()) ||
        !std::isfinite(error) || !std::isfinite(proportional))
        return finish(Decision::BLOCKED, Reason::NUMERIC);
    const double correction = (std::max)(-cap, (std::min)(cap, proportional));
    if (m_pCoordMgr != nullptr) m_pCoordMgr->UpdateSoftwareTravelLimitState(axis);
    if ((correction > 0.0 && (axis.hardLimitPositive ||
        (m_pCoordMgr != nullptr && !m_pCoordMgr->CanMoveSoftwarePositive(axis)))) ||
        (correction < 0.0 && (axis.hardLimitNegative ||
            (m_pCoordMgr != nullptr && !m_pCoordMgr->CanMoveSoftwareNegative(axis)))))
        return finish(Decision::BLOCKED, Reason::TRAVEL_LIMIT);
    // Revalidate after the bounded geometry and travel checks. Final image/send
    // checks additionally bind this generation to the output frame.
    if (m_pathHold.generation != m_pathHoldGeneration.load(std::memory_order_acquire) ||
        !IsMotionOwnerLeaseCurrent(s.ownerLease) || HasPendingExecutionEpochChange() ||
        s.identity.epoch != GetCurrentExecutionEpoch() || HasPendingSafetyOrRecoveryRequests() ||
        AlarmManager::GetInstance().HasAlarm())
        return finish(Decision::BLOCKED, Reason::AUTHORITY);
    velocityPPS = correction;
    m_pathAdmissionCorrectionFrameGeneration = m_pathHold.generation;
    return finish(Decision::AUTHORIZED, Reason::NONE);
}

template <typename DriveType>
void MotionCore::Run_Servo_Loop(DriveType& servo, AxisContext& axis, const AxisCommand& cmd,
    const MotionServoInputSnapshot& input)
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

        // An active malformed range has no permitted direction. Fence every
        // axis through the existing Safety mailbox once per alarm incident,
        // and keep this output zero on every cycle without advancing PID or
        // rebasing any commanded position. HOME=false retains its bypass.
        if (m_pCoordMgr->GetInvalidSoftwareTravelLimitMask(axis) != 0U)
        {
            if (!AlarmManager::GetInstance().HasAlarm())
            {
                AlarmManager::GetInstance().Trigger(
                    AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG,
                    0, axis.axisIndex);
                RequestEmergencyStopAllAxes();
            }
            WriteServoTargetVelocityCommand(servo.pOutput, axis.axisIndex, 0);
            return;
        }
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
                AlarmManager::GetInstance().Trigger(
                    m_pCoordMgr->GetSoftwareTravelLimitAlarmCode(
                        axis, AlarmManager::OVER_TRAVEL),
                    0, axis.axisIndex);
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


    // =========================================================
    // NC-0.2K.6.3 START idle-PID authority fence
    //
    // A non-zero servo image is deliberately scrubbed at the physical send
    // seam while Motion has no owner.  The historical loop nevertheless kept
    // integrating the stationary position error into axis.pid.integralAcc.
    // That internal value was therefore invisible on the wire until Cycle
    // Start acquired AUTO ownership; the first newly-authorized frame could
    // then release the accumulated correction as a full-scale velocity pulse
    // before the first NC block was dispatched.
    //
    // Keep feedback acquisition, actual-velocity measurement and travel-limit
    // observation above active while unowned, but do not evolve a controller
    // whose output is contractually forbidden from reaching the drive.  This
    // is an authority-state reset only: it does not rebase command position,
    // change planner state, or modify G00 / Reset deceleration behavior.
    // =========================================================
    const MotionOwnerLease servoLoopOwnerLease =
        GetMotionOwnerLease();
    if (servoLoopOwnerLease.owner == MotionOwner::NONE ||
        servoLoopOwnerLease.owner == MotionOwner::IDLE_HOLD)
    {
        axis.pid.prevError = 0.0;
        axis.pid.integralAcc = 0.0;
        axis.Pid_IDLE.prevError = 0.0;
        axis.Pid_IDLE.integralAcc = 0.0;
        axis.Pid_G00.prevError = 0.0;
        axis.Pid_G00.integralAcc = 0.0;

        WriteServoTargetVelocityCommand(
            servo.pOutput,
            axis.axisIndex,
            0);
        return;
    }


    double admissionCorrectionVelocity = 0.0;
    const auto admissionCorrection = ResolvePathCoreAdmissionPositionCorrection(
        axis, cmd, input, admissionCorrectionVelocity);
    if (admissionCorrection == MotionPathCoreAdmissionCorrectionDecision::BLOCKED)
    {
        WriteServoTargetVelocityCommand(servo.pOutput, axis.axisIndex, 0);
        return;
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


    // CU_FIX1: only the exact live queued-source wait discards prior motion I.
    // Every other owner/state retains its existing PI calculation.
    double p_term = error * axis.pid.Kp;
    double i_term = 0.0;
    double finalVel = 0.0;
    if (admissionCorrection == MotionPathCoreAdmissionCorrectionDecision::AUTHORIZED)
    {
        axis.pid.prevError = 0.0;
        axis.pid.integralAcc = 0.0;
        finalVel = admissionCorrectionVelocity;
    }
    else
    {
        axis.pid.integralAcc += error * CYCLE_TIME_SEC;
        if (axis.pid.integralAcc > axis.pid.MaxIntegral) axis.pid.integralAcc = axis.pid.MaxIntegral;
        if (axis.pid.integralAcc < -axis.pid.MaxIntegral) axis.pid.integralAcc = -axis.pid.MaxIntegral;
        i_term = axis.pid.integralAcc * axis.pid.Ki;
        const double d_term = 0.0;
        finalVel = (cmd.instantCmdVel * axis.pid.Kvff) + (p_term + i_term + d_term);
    }

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
            if (admissionCorrection == MotionPathCoreAdmissionCorrectionDecision::AUTHORIZED)
            {
                // A newly observed travel block cannot rebase the held CMD.
                if (axis.axisIndex == m_pathHold.status.admissionCorrectionAxis)
                {
                    m_pathHold.status.admissionCorrectionDecision = MotionPathCoreAdmissionCorrectionDecision::BLOCKED;
                    m_pathHold.status.admissionCorrectionReason = MotionPathCoreAdmissionCorrectionReason::TRAVEL_LIMIT;
                    m_pathHold.status.admissionCorrectionVelocityPPS = 0.0;
                }
                WriteServoTargetVelocityCommand(servo.pOutput, axis.axisIndex, 0);
                return;
            }
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

    if (m_idlePositionHold.passRequested ||
        GetMotionOwnerLease().owner == MotionOwner::IDLE_HOLD)
    {
        UpdateIdlePositionHoldAxis(servo.pOutput, axis, input);
        return;
    }

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
        if (m_pathHold.status.admissionPending)
        {
            AxisCommand heldCommand{};
            heldCommand.instantCmdPos = axis.currentCmdPos;
            heldCommand.instantCmdVel = 0.0;
            double ignoredVelocity = 0.0;
            if (ResolvePathCoreAdmissionPositionCorrection(axis, heldCommand, input, ignoredVelocity) !=
                MotionPathCoreAdmissionCorrectionDecision::NOT_APPLICABLE)
            {
                WriteServoTargetVelocityCommand(servo.pOutput, axis.axisIndex, 0);
                return;
            }
        }
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

    // NC-0.2K.6.2 final command fence. Freezing UpdateInterpolation alone is
    // insufficient because an INTERPOLATING AxisContext would otherwise keep
    // forwarding its previous non-zero velocity to the PDO. Every outstanding
    // Safety request, SAFETY owner phase, or unauthorized active Group emits
    // an immediate zero at the final servo-command seam. The sole exception
    // is an RT-owned controlled stop bound to the exact SAFETY lease, Epoch,
    // request ticket and physical group member.
    const MotionOwnerLease runtimeOwnerLease = GetMotionOwnerLease();
    const bool controlledStopAuthorized =
        IsSafetyControlledStopAuthorized(contextAxisSlot);
    const bool activeGroupUnauthorized =
        m_Group.isActive &&
        GetCommandAuthorizationFailure(m_Group.currentCmd) !=
        MotionRejectReason::NONE;
    if (!controlledStopAuthorized &&
        (HasUnacknowledgedSafetyMotionRequest() ||
            runtimeOwnerLease.owner == MotionOwner::SAFETY ||
            activeGroupUnauthorized))
    {
        WriteServoTargetVelocityCommand(
            servo.pOutput,
            axis.axisIndex,
            0);
        return;
    }

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




    Run_Servo_Loop(servo, axis, cmd, input);
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

void MotionCore::LineMove(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos,
    double targetVel,
    double acc_time,
    double dec_time,
    BufferMode mode,
    MotionCommandPathMode commandPathMode)
{
    // ABORTING opens a new execution Epoch.  Capture one data-race-free
    // logical baseline before publication so an accepted legacy/special
    // LineMove can rebuild every existing axis in the new producer tail.
    // The captured values are not authoritative unless ingress accepts.
    std::array<double, MAX_AXES> abortingLogicalSnapshot{};
    std::uint32_t abortingLogicalValidMask = 0U;
    if (mode == BufferMode::ABORTING && m_pContexts != nullptr)
    {
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
                continue;
            }

            abortingLogicalSnapshot[axisSlot] = logicalPulse;
            abortingLogicalValidMask |=
                (1U << static_cast<unsigned>(axisSlot));
        }
    }

    MotionExecutionIdentity producedIdentity{};
    MotionOwnerLease producedOwnerLease{};
    const bool accepted = TryLineMove(
        axes,
        targetPos,
        targetVel,
        acc_time,
        dec_time,
        mode,
        commandPathMode,
        &producedIdentity,
        &producedOwnerLease,
        MOTION_EXECUTION_EPOCH_INVALID,
        nullptr);
    if (!accepted)
    {
        return;
    }

    const bool preserveBufferedTail =
        mode == BufferMode::BUFFERED &&
        m_g00ProducerQueueTailEpoch == producedIdentity.epoch &&
        m_g00ProducerQueueTailOwnerLease.IsValid() &&
        m_g00ProducerQueueTailOwnerLease.Matches(producedOwnerLease);

    std::uint32_t committedValidMask =
        preserveBufferedTail
        ? m_g00ProducerQueueTailValidMask
        : 0U;

    if (mode == BufferMode::ABORTING)
    {
        m_g00ProducerQueueTailPulse.fill(0.0);
        committedValidMask = 0U;

        for (std::size_t axisSlot = 0U;
            axisSlot < static_cast<std::size_t>(MAX_AXES);
            ++axisSlot)
        {
            const std::uint32_t axisBit =
                (1U << static_cast<unsigned>(axisSlot));
            if ((abortingLogicalValidMask & axisBit) == 0U)
            {
                continue;
            }

            const double logicalPulse =
                abortingLogicalSnapshot[axisSlot];
            m_g00ProducerQueueTailPulse[axisSlot] = logicalPulse;
            (*m_pContexts)[axisSlot].lastQueuedPulse.Store(logicalPulse);
            committedValidMask |= axisBit;
        }
    }
    else if (!preserveBufferedTail)
    {
        m_g00ProducerQueueTailPulse.fill(0.0);
    }

    // Active targets supersede the ABORTING logical baseline, or extend the
    // same-tag BUFFERED tail.  No legacy producer endpoint is visible before
    // its MotionCommand has crossed the ingress linearization point.
    for (std::size_t slot = 0U; slot < axes.size(); ++slot)
    {
        const std::size_t axisSlot =
            static_cast<std::size_t>(axes[slot]);
        const double targetPulse = targetPos[slot];
        m_g00ProducerQueueTailPulse[axisSlot] = targetPulse;
        (*m_pContexts)[axisSlot].lastQueuedPulse.Store(targetPulse);
        committedValidMask |=
            (1U << static_cast<unsigned>(axisSlot));
    }

    m_g00ProducerQueueTailValidMask = committedValidMask;
    m_g00ProducerQueueTailEpoch = producedIdentity.epoch;
    m_g00ProducerQueueTailOwnerLease = producedOwnerLease;
}

bool MotionCore::TryLineMove(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos,
    double targetVel,
    double acc_time,
    double dec_time,
    BufferMode mode,
    MotionCommandPathMode commandPathMode,
    MotionExecutionIdentity* producedIdentity,
    MotionOwnerLease* producedOwnerLease,
    MotionExecutionEpoch plannedTailEpoch,
    const MotionOwnerLease* plannedTailOwnerLease,
    bool cncFeedLookahead,
    const NCPathCoreRetainedGeometry* cncCorner,
    double cncPrefixVelocityPPS,
    bool pathCoreFeedExactStop) noexcept
{
    if (producedIdentity != nullptr)
    {
        *producedIdentity = MotionExecutionIdentity{};
    }
    if (producedOwnerLease != nullptr)
    {
        *producedOwnerLease = MotionOwnerLease{};
    }
    // Capture one producer tuple before validation. Even malformed producer
    // input must receive an identity, a terminal REJECTED notice and a formal
    // Alarm/E-stop transaction; it must never disappear before M30 accounting.
    const MotionCommandSource commandSource =
        m_pendingCommandSource.load(
            std::memory_order_acquire);
    const MotionOwnerLease entryOwnerLease =
        GetMotionOwnerLease();
    const MotionExecutionEpoch entryEpoch =
        GetCurrentExecutionEpoch();

    const bool plannedTailRequested =
        plannedTailEpoch != MOTION_EXECUTION_EPOCH_INVALID ||
        plannedTailOwnerLease != nullptr;
    const bool plannedTailWellFormed =
        plannedTailEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        plannedTailOwnerLease != nullptr &&
        plannedTailOwnerLease->IsValid();

    MotionExecutionEpoch commandEpoch =
        plannedTailWellFormed
        ? plannedTailEpoch
        : entryEpoch;
    MotionOwnerLease commandOwnerLease =
        plannedTailWellFormed
        ? *plannedTailOwnerLease
        : entryOwnerLease;

    MotionCommand invalidCommand{};
    invalidCommand.mode = InterpolationMode::LINEAR;
    invalidCommand.sourceLinePC = m_pendingSourcePC;
    invalidCommand.commandPathMode = commandPathMode;

    // Permit only the native XY queue lane under a frozen rotation. A supplied
    // Q compound must also pass full canonical geometry validation below, before
    // an aborting epoch can be published; exact-stop retains its existing lane.
    const bool fixedRotatedQueuedLine = commandSource == MotionCommandSource::NC_MEMORY &&
        IsPendingFixedTranslationSourceAllowed() && plannedTailWellFormed &&
        commandOwnerLease.owner == MotionOwner::AUTO && m_pendingIsAbsoluteMode &&
        m_pendingTranslation.distanceMode == 90 && !m_pendingG162Active &&
        cncFeedLookahead && commandPathMode == MotionCommandPathMode::CONTINUOUS &&
        !pathCoreFeedExactStop && (cncCorner != nullptr || cncPrefixVelocityPPS == 0.0) &&
        axes.size() == 2U && axes[0] == 0 && axes[1] == 1;
    // BASE-PLANE-2 producer gate precedes every aborting epoch/queue commit.
    // Native XYZ lines do not inherit the circle's ordered (u,v) axis mapping.
    const bool basePlaneLinear = m_pendingPlaneMode != 17;
    const bool basePlaneLinearAllowed = commandSource == MotionCommandSource::NC_MEMORY &&
        plannedTailWellFormed && commandOwnerLease.owner == MotionOwner::AUTO &&
        IsNCTranslationSnapshotValid(m_pendingTranslation) &&
        IsNCTranslationBaseArcPlaneFrame(m_pendingTranslation) &&
        m_pendingPlaneMode == m_pendingTranslation.rotationPlane &&
        IsNCNativeXYZLinearMapping(static_cast<int>(axes.size()), axes.data()) &&
        mode == BufferMode::ABORTING && commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        !cncFeedLookahead && cncCorner == nullptr && cncPrefixVelocityPPS == 0.0;
    if ((basePlaneLinear && !basePlaneLinearAllowed) ||
        !IsPendingCommandTranslationValid(commandSource) ||
        (commandSource == MotionCommandSource::NC_MEMORY && m_pendingToolRadMode != 40 &&
            (!pathCoreFeedExactStop || !plannedTailWellFormed ||
                commandOwnerLease.owner != MotionOwner::AUTO ||
                mode != BufferMode::ABORTING || commandPathMode != MotionCommandPathMode::EXACT_STOP ||
                cncFeedLookahead || cncCorner != nullptr || cncPrefixVelocityPPS != 0.0 ||
                !IsNCPlaneLinearPairMapping(m_pendingPlaneMode,
                    static_cast<int>(axes.size()), axes.data()))) ||
        (commandSource == MotionCommandSource::NC_MEMORY && NCTranslationHasPlanarRotation(m_pendingTranslation) &&
            !fixedRotatedQueuedLine && (commandPathMode != MotionCommandPathMode::EXACT_STOP ||
                cncFeedLookahead || cncCorner != nullptr)) ||
        (plannedTailRequested && !plannedTailWellFormed) ||
        (pathCoreFeedExactStop && (!plannedTailWellFormed || cncFeedLookahead || cncCorner != nullptr ||
            mode != BufferMode::ABORTING || commandPathMode != MotionCommandPathMode::EXACT_STOP ||
            commandSource != MotionCommandSource::NC_MEMORY || commandOwnerLease.owner != MotionOwner::AUTO)))
    {
        RejectInvalidProducerMotionCommand(
            invalidCommand,
            entryEpoch,
            commandSource,
            entryOwnerLease,
            producedIdentity,
            producedOwnerLease);
        return false;
    }

    if (m_pContexts == nullptr ||
        axes.empty() ||
        axes.size() > static_cast<std::size_t>(MAX_AXES) ||
        targetPos.size() != axes.size() ||
        !IsValidMotionCommandPathMode(commandPathMode) ||
        !std::isfinite(targetVel) ||
        !std::isfinite(acc_time) ||
        !std::isfinite(dec_time) ||
        !std::isfinite(cncPrefixVelocityPPS) ||
        (cncCorner == nullptr ? cncPrefixVelocityPPS != 0.0 :
            (cncPrefixVelocityPPS != 0.0 && cncPrefixVelocityPPS < std::abs(targetVel))))
    {
        RejectInvalidProducerMotionCommand(
            invalidCommand,
            commandEpoch,
            commandSource,
            commandOwnerLease,
            producedIdentity,
            producedOwnerLease);
        return false;
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
            (commandSource == MotionCommandSource::NC_MEMORY &&
                !IsNCTranslationSnapshotEmpty(m_pendingTranslation) &&
                (axisIndex > 2 || (*m_pContexts)[axisIndex].axisType != AxisType::LINEAR)) ||
            !std::isfinite(targetPos[slot]))
        {
            RejectInvalidProducerMotionCommand(
                invalidCommand,
                commandEpoch,
                commandSource,
                commandOwnerLease,
                producedIdentity,
                producedOwnerLease);
            return false;
        }
        seenAxis[static_cast<std::size_t>(axisIndex)] = true;
    }

    // 1. 建立一個全新的任務包裹
    MotionCommand cmd{};
    cmd.mode = InterpolationMode::LINEAR;
    cmd.axisCount = (int)axes.size();
    cmd.commandPathMode = commandPathMode;
    cmd.cncFeedLookahead = cncFeedLookahead;
    cmd.pathCoreFeedExactStop = pathCoreFeedExactStop;
    cmd.cncPrefixVelocityPPS = cncPrefixVelocityPPS;

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
    if (commandSource == MotionCommandSource::NC_MEMORY) cmd.sourceTranslation = m_pendingTranslation;

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

    if (cncCorner != nullptr)
    {
        if (!cncFeedLookahead || cncCorner->kind != NCPathCoreRetainedKind::LINE_ARC ||
            !IsNCPathCoreRetainedGeometryValid(*cncCorner) || axes.size() != 2U || axes[0] != 0 || axes[1] != 1 ||
            targetPos[0] != cncCorner->endPulse[0] || targetPos[1] != cncCorner->endPulse[1])
        {
            RejectInvalidProducerMotionCommand(invalidCommand, commandEpoch, commandSource,
                commandOwnerLease, producedIdentity, producedOwnerLease);
            return false;
        }
        // DK producer validates its extra speed permission before an aborting
        // epoch can be published. TryEnqueue checks authority, not this geometry.
        if (cncPrefixVelocityPPS != 0.0)
        {
            double ppm = 0.0;
            for (unsigned i = 0U; i < 2U; ++i)
            {
                const auto& axis = (*m_pContexts)[i];
                const double next = axis.resolution_PPR / axis.finalLead;
                if (!std::isfinite(next) || next <= 0.0 || (i != 0U && ppm != next) ||
                    !std::isfinite(axis.maxVel_PPS) || cncPrefixVelocityPPS > axis.maxVel_PPS ||
                    cncPrefixVelocityPPS / next > (100.0 / 60.0) *
                        (1.0 + 16.0 * std::numeric_limits<double>::epsilon()))
                {
                    RejectInvalidProducerMotionCommand(invalidCommand, commandEpoch, commandSource,
                        commandOwnerLease, producedIdentity, producedOwnerLease);
                    return false;
                }
                ppm = next;
            }
        }
        cmd.cncCornerBlend = true;
        cmd.dir = cncCorner->direction;
        cmd.startRadius = cmd.endRadius = cmd.mem_radius = cncCorner->radiusPulse;
        cmd.mem_startAngle = cncCorner->startAngle; cmd.mem_totalAngle = cncCorner->sweepRadians;
        cmd.mem_totalDist = cncCorner->lengthPulse;
        cmd.centerPos[0] = cmd.mem_centerX = cncCorner->centerPulse[0];
        cmd.centerPos[1] = cmd.mem_centerY = cncCorner->centerPulse[1];
        for (unsigned i = 0U; i < 2U; ++i)
        {
            cmd.mem_startPos[i] = cncCorner->startPulse[i];
            cmd.mem_ratio[i] = cncCorner->centerPulse[i] + cncCorner->radiusPulse *
                (i == 0U ? std::cos(cncCorner->startAngle) : std::sin(cncCorner->startAngle));
        }
    }

    // The rotated Q lane proves the actual native packet against the current
    // axis configuration before any aborting epoch or transport publication.
    // These private source tags authorize validation only; the execution
    // identity is assigned after the existing epoch/owner transaction below.
    if (cncCorner != nullptr && commandSource == MotionCommandSource::NC_MEMORY &&
        NCTranslationHasPlanarRotation(m_pendingTranslation))
    {
        cmd.execution.source = commandSource;
        cmd.ownerLease = commandOwnerLease;
        if (!IsMotionCommandConsumerGeometryValid(cmd, m_pContexts))
        {
            RejectInvalidProducerMotionCommand(invalidCommand, commandEpoch, commandSource,
                commandOwnerLease, producedIdentity, producedOwnerLease);
            return false;
        }
    }

    // 2. 判斷是「乖乖排隊」還是「緊急覆寫」？
    if (mode == BufferMode::ABORTING)
    {
        const MotionExecutionEpoch expectedEpoch =
            plannedTailWellFormed
            ? plannedTailEpoch
            : entryEpoch;
        const MotionOwnerLease expectedOwnerLease =
            plannedTailWellFormed
            ? *plannedTailOwnerLease
            : entryOwnerLease;

        const bool plannedTupleMatchesEntry =
            expectedEpoch == entryEpoch &&
            expectedOwnerLease.Matches(entryOwnerLease);

        MotionExecutionEpoch publishedEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        if (!plannedTupleMatchesEntry ||
            !TryPublishOwnerAuthorizedAbortingExecutionEpoch(
                commandSource,
                expectedEpoch,
                expectedOwnerLease,
                publishedEpoch))
        {
            const bool ownerStillCurrent =
                IsMotionOwnerLeaseCurrent(expectedOwnerLease);
            const MotionExecutionEpoch currentEpoch =
                GetCurrentExecutionEpoch();
            const MotionRejectReason rejectReason =
                !ownerStillCurrent
                ? MotionRejectReason::OWNER_CONFLICT
                : (currentEpoch != expectedEpoch &&
                    currentEpoch != publishedEpoch)
                ? MotionRejectReason::STALE_EPOCH
                : MotionRejectReason::NOT_READY;

            RejectNonGeometryProducerMotionCommand(
                invalidCommand,
                publishedEpoch != MOTION_EXECUTION_EPOCH_INVALID
                ? publishedEpoch
                : expectedEpoch,
                commandSource,
                expectedOwnerLease,
                rejectReason,
                producedIdentity,
                producedOwnerLease);
            return false;
        }

        commandEpoch = publishedEpoch;
        commandOwnerLease = expectedOwnerLease;

        // Future Queue / History 與 Active Abort 都由 250 us Consumer
        // 在 Epoch 邊界套用；Producer 不再修改 Motion Runtime 容器。
    }

    // 3. 在最終 Epoch 確定後配置 Identity，再把包裹推入倉庫。
    AssignExecutionIdentity(
        cmd,
        commandEpoch,
        commandSource,
        commandOwnerLease);
    if (producedIdentity != nullptr)
    {
        *producedIdentity = cmd.execution;
    }
    if (producedOwnerLease != nullptr)
    {
        *producedOwnerLease = cmd.ownerLease;
    }

    return TryEnqueueMotionCommand(cmd);
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
// DC: one late-successor attempt per loaded NC_MEMORY/P1 straight segment.
// This promotes only a cruise-reachable, same-speed collinear pair whose next
// segment can stop on its own. It is not a multi-block or contour-blend planner.
bool MotionCore::IsCncP1LateJunctionScope() const noexcept
{
    const MotionCommand& cmd = m_Group.currentCmd;
    // DM: NC_MEMORY/AUTO and NC_MDI/MDI use the same bounded late-arrival
    // proof. Never admit a source under the other mode's owner lease.
    const bool matchedNcOwner =
        (cmd.execution.source == MotionCommandSource::NC_MEMORY &&
            cmd.ownerLease.owner == MotionOwner::AUTO) ||
        (cmd.execution.source == MotionCommandSource::NC_MDI &&
            cmd.ownerLease.owner == MotionOwner::MDI);
    return cmd.execution.IsAssigned() && matchedNcOwner && cmd.ownerLease.IsValid() &&
        cmd.commandPathMode == MotionCommandPathMode::CONTINUOUS && !cmd.cncFeedLookahead &&
        cmd.mode == InterpolationMode::LINEAR && cmd.axisCount > 0 && cmd.axisCount <= 3 &&
        !cmd.pathCoreRetainedTraversal && !cmd.replayTerminalAlreadyPublished &&
        !cmd.sourceG68Active && IsMotionFixedTranslationWorkSourceAllowed(cmd) && !cmd.sourceG51Active &&
        cmd.sourceMirrorMask == 0U && !cmd.sourceG16Active &&
        m_Group.pathMode == PathMode::CONTINUOUS && !m_Group.enableHistory &&
        !m_Group.enableTransform && m_Group.jumpManager.state == JumpState::IDLE &&
        !m_pathHold.sourceSeen && !IsPathCoreHoldExcursionDriving();
}

void MotionCore::QueueCncP1Diagnostic(CncP1Event event, CncP1Reason reason,
    const MotionCommand* next, double nextLength) noexcept
{
    CncP1Diagnostic& d = m_cncP1Late.producerEvent;
    d = CncP1Diagnostic{};
    const MotionCommand& cmd = m_Group.currentCmd;
    const AxisContext& axis = m_Group.virtualAxis;
    d.runtimeTick = m_ncSettleRuntimeCycleTick;
    d.tickValid = m_ncSettleRuntimeObserved && m_ncSettleRuntimeCycleValid &&
        m_ncSettleRuntimeCycleContiguous && d.runtimeTick != 0ULL;
    d.sequence = ++m_cncP1Late.sequence;
    d.epoch = cmd.execution.epoch;
    d.segment = cmd.execution.segmentId;
    d.sourcePC = cmd.sourceLinePC;
    d.owner = cmd.ownerLease.owner;
    d.generation = cmd.ownerLease.generation;
    d.axisMask = BuildMotionCommandAxisMask(cmd);
    d.queueDepth = static_cast<std::uint32_t>(m_Group.cmdQueue.size());
    d.commandVelocity = axis.currentCmdVel;
    d.endVelocity = axis.targetEndVel;
    d.remainingPulse = axis.finalTargetPos - axis.planningPos;
    d.nextLengthPulse = nextLength;
    d.event = event;
    d.reason = reason;
    if (next != nullptr)
    {
        d.nextSegment = next->execution.segmentId;
        d.nextSourcePC = next->sourceLinePC;
    }
    if (!m_cncP1Late.events.ProducerTryPush(d))
        m_cncP1Late.dropped.fetch_add(1U, std::memory_order_relaxed);
}

bool MotionCore::TryPopCncP1Diagnostic(CncP1Diagnostic& event) noexcept
{
    return m_cncP1Late.events.ConsumerTryPop(event);
}

std::uint32_t MotionCore::GetCncP1DiagnosticDroppedCount() const noexcept
{
    return m_cncP1Late.dropped.load(std::memory_order_acquire);
}

void MotionCore::ArmCncP1LateJunction(bool nextVisible, const MotionCommand& next) noexcept
{
    m_cncP1Late.loaded = false;
    m_cncP1Late.pending = false;
    if (!IsCncP1LateJunctionScope()) return;
    m_cncP1Late.identity = m_Group.currentCmd.execution;
    m_cncP1Late.lease = m_Group.currentCmd.ownerLease;
    m_cncP1Late.loaded = true;
    m_cncP1Late.pending = !nextVisible && m_Group.virtualAxis.targetEndVel == 0.0;
    QueueCncP1Diagnostic(nextVisible ? CncP1Event::LOAD_READY : CncP1Event::LOAD_EMPTY,
        CncP1Reason::NONE, nextVisible ? &next : nullptr);
}

void MotionCore::FinishCncP1Diagnostic() noexcept
{
    if (!m_cncP1Late.loaded ||
        !MotionExecutionIdentityExactlyMatches(m_cncP1Late.identity, m_Group.currentCmd.execution) ||
        !m_cncP1Late.lease.Matches(m_Group.currentCmd.ownerLease)) return;
    QueueCncP1Diagnostic(CncP1Event::LEAVE,
        m_cncP1Late.pending ? CncP1Reason::QUEUE_EMPTY : CncP1Reason::NONE);
    m_cncP1Late.pending = false;
    m_cncP1Late.loaded = false;
}

void MotionCore::RefreshCncP1LateJunction() noexcept
{
    if (!m_cncP1Late.pending) return;
    AxisContext& v = m_Group.virtualAxis;
    const MotionCommand& cmd = m_Group.currentCmd;
    if (!m_Group.isActive || !IsCncP1LateJunctionScope() ||
        !MotionExecutionIdentityExactlyMatches(m_cncP1Late.identity, cmd.execution) ||
        !m_cncP1Late.lease.Matches(cmd.ownerLease))
    {
        m_cncP1Late.pending = false;
        return;
    }
    const auto keepStop = [this](CncP1Reason reason, const MotionCommand* next = nullptr,
        double length = 0.0) noexcept
    {
        m_cncP1Late.pending = false;
        QueueCncP1Diagnostic(CncP1Event::KEEP_STOP, reason, next, length);
    };
    if (m_safetyControlledStopInProgress || HasPendingSafetyOrRecoveryRequests() ||
        HasPendingExecutionEpochChange() || GetCommandAuthorizationFailure(cmd) != MotionRejectReason::NONE)
    {
        keepStop(CncP1Reason::AUTHORITY);
        return;
    }
    if (m_Group.feedrateOverride != 1.0)
    {
        keepStop(CncP1Reason::OVERRIDE);
        return;
    }
    if (v.state != MotionState::MotionState_MOVING || v.inPosition || v.targetEndVel != 0.0)
    {
        keepStop(CncP1Reason::TERMINAL);
        return;
    }
    MotionCommand& next = m_cncP1Late.successor;
    if (!TryPeekNextMotionCommand(next)) return; // No waiting and no queue scan.
    m_cncP1Late.pending = false; // Exactly one visible-front attempt for this source.
    if (!next.execution.IsAssigned() || GetCommandAuthorizationFailure(next) != MotionRejectReason::NONE ||
        next.execution.epoch != cmd.execution.epoch || !next.ownerLease.Matches(cmd.ownerLease) ||
        next.execution.source != cmd.execution.source ||
        next.execution.segmentId == cmd.execution.segmentId)
    {
        keepStop(CncP1Reason::AUTHORITY, &next);
        return;
    }
    if (next.cncFeedLookahead || next.commandPathMode != MotionCommandPathMode::CONTINUOUS ||
        next.mode != InterpolationMode::LINEAR || next.pathCoreRetainedTraversal ||
        next.replayTerminalAlreadyPublished || next.sourceG68Active ||
        !IsMotionFixedTranslationWorkSourceAllowed(next) ||
        next.sourceG51Active || next.sourceMirrorMask != 0U || next.sourceG16Active ||
        next.sourceG162Active != cmd.sourceG162Active)
    {
        keepStop(CncP1Reason::SCOPE, &next);
        return;
    }
    if (!MotionCommandsHaveIdenticalAxisMapping(cmd, next))
    {
        keepStop(CncP1Reason::MAPPING, &next);
        return;
    }
    if (!IsMotionCommandConsumerGeometryValid(next, m_pContexts) ||
        !IsMotionCommandConsumerGeometryValid(cmd, m_pContexts))
    {
        keepStop(CncP1Reason::GEOMETRY, &next);
        return;
    }
    double currentLength = 0.0, nextLength = 0.0;
    for (int slot = 0; slot < cmd.axisCount; ++slot)
    {
        const int index = cmd.axisIndices[slot];
        if ((*m_pContexts)[index].axisType != AxisType::LINEAR)
        {
            keepStop(CncP1Reason::SCOPE, &next);
            return;
        }
        currentLength = std::hypot(currentLength, cmd.targetPos[slot] - m_Group.startPos[slot]);
        nextLength = std::hypot(nextLength, next.targetPos[slot] - cmd.targetPos[slot]);
    }
    if (!std::isfinite(currentLength) || !std::isfinite(nextLength) ||
        currentLength <= 0.0 || nextLength <= 0.0)
    {
        keepStop(CncP1Reason::GEOMETRY, &next, nextLength);
        return;
    }
    for (int slot = 0; slot < cmd.axisCount; ++slot)
    {
        const double a = (cmd.targetPos[slot] - m_Group.startPos[slot]) / currentLength;
        const double b = (next.targetPos[slot] - cmd.targetPos[slot]) / nextLength;
        // DV_FIX1: a stopped multi-axis positioning move can leave a tiny
        // floating residue on a nominally stationary axis. Use the existing
        // unit-tangent tolerance for zero components too; an exact-zero
        // classification must not override that same bounded comparison.
        // Targets, axis mapping and all speed/stopping-distance proofs stay exact.
        if (!std::isfinite(a) || !std::isfinite(b) ||
            std::abs(a - b) > 32.0 * std::numeric_limits<double>::epsilon())
        {
            keepStop(CncP1Reason::DIRECTION, &next, nextLength);
            return;
        }
    }
    // DO: an equal/faster successor can inherit the current cruise.
    // Keep the full current-cruise and next-stop distance proofs below;
    // a downshift still requires a separate raw/FIR deceleration proof.
    const double speed = v.cruiseVel_PPS;
    if (!std::isfinite(speed) || speed <= 0.1 || speed != cmd.targetVel ||
        !std::isfinite(next.targetVel) || next.targetVel < cmd.targetVel ||
        next.accTime != cmd.accTime || next.decTime != cmd.decTime ||
        !std::isfinite(v.currentCmdVel) || v.currentCmdVel < 0.0 || v.currentCmdVel > speed ||
        !std::isfinite(v.acc_PPS2) || v.acc_PPS2 <= 0.0 ||
        !std::isfinite(v.dec_PPS2) || v.dec_PPS2 <= 0.0 || v.velBuffer.size() > 4096U)
    {
        keepStop(CncP1Reason::SPEED, &next, nextLength);
        return;
    }
    if (next.targetVel > speed)
    {
        // Match LoadNextCommand, including its time clamp and dec fallback.
        // A finite higher F must not overflow the future dynamics or weaken
        // the current-speed stopping proof used for the next segment.
        const double nextAcc = next.accTime < 0.0001 ? 1e10 : next.targetVel / next.accTime;
        const double nextDecTime = next.decTime < 0.0 ? next.accTime : next.decTime;
        const double nextDec = nextDecTime < 0.0001 ? 1e10 : next.targetVel / nextDecTime;
        if (!std::isfinite(nextAcc) || !std::isfinite(nextDec) ||
            nextAcc <= 0.0 || nextDec <= 0.0 ||
            nextAcc < v.acc_PPS2 || nextDec < v.dec_PPS2)
        {
            keepStop(CncP1Reason::SPEED, &next, nextLength);
            return;
        }
    }
    // Conservative full-filter-distance reserve, not a contour error allowance.
    const double margin = speed * ((static_cast<double>(v.velBuffer.size()) + 4.0) * CYCLE_TIME_SEC) + 1.0;
    const double accelerate = (speed - v.currentCmdVel) * (speed + v.currentCmdVel) / (2.0 * v.acc_PPS2);
    const double brake = (speed / (2.0 * v.dec_PPS2)) * speed;
    const double remaining = v.finalTargetPos - v.planningPos;
    if (!std::isfinite(margin) || !std::isfinite(accelerate) || !std::isfinite(brake) ||
        !std::isfinite(remaining) || !std::isfinite(v.planningPos) || v.planningPos < 0.0 ||
        remaining <= accelerate + margin)
    {
        keepStop(CncP1Reason::CURRENT_DISTANCE, &next, nextLength);
        return;
    }
    if (nextLength <= brake + margin)
    {
        keepStop(CncP1Reason::NEXT_DISTANCE, &next, nextLength);
        return;
    }
    if (HasPendingSafetyOrRecoveryRequests() || HasPendingExecutionEpochChange() ||
        GetCommandAuthorizationFailure(cmd) != MotionRejectReason::NONE ||
        GetCommandAuthorizationFailure(next) != MotionRejectReason::NONE)
    {
        keepStop(CncP1Reason::AUTHORITY, &next, nextLength);
        return;
    }
    v.targetEndVel = speed; // Existing planner/filtered handoff; never force current velocity.
    QueueCncP1Diagnostic(CncP1Event::PROMOTED, CncP1Reason::NONE, &next, nextLength);
}

// DF: bounded per-command-feed collinear horizon. Unknown continuation ends at zero.
bool MotionCore::BuildCncFeedStopPlan(const std::array<double, 4U>& length,
    const std::array<double, 4U>& speed, const std::array<double, 4U>& acceleration,
    const std::array<double, 4U>& deceleration, std::size_t count, double entry,
    std::array<double, 5U>& boundary) noexcept
{
    boundary.fill(0.0);
    if (count == 0U || count > 4U || !std::isfinite(entry) || entry < 0.0) return false;
    for (std::size_t i = 0U; i < count; ++i)
    {
        if (!std::isfinite(length[i]) || length[i] < 0.0 || !std::isfinite(speed[i]) || speed[i] <= 0.0 ||
            !std::isfinite(acceleration[i]) || acceleration[i] <= 0.0 ||
            !std::isfinite(deceleration[i]) || deceleration[i] <= 0.0) return false;
        if (i != 0U) boundary[i] = (std::min)(speed[i - 1U], speed[i]);
    }
    for (std::size_t i = count - 1U; i > 0U; --i)
    {
        const double square = boundary[i + 1U] * boundary[i + 1U] + 2.0 * deceleration[i] * length[i];
        if (!std::isfinite(square) || square < 0.0) { boundary.fill(0.0); return false; }
        boundary[i] = (std::min)(boundary[i], std::sqrt(square));
    }
    boundary[0U] = entry;
    for (std::size_t i = 0U; i + 1U < count; ++i)
    {
        const double square = boundary[i] * boundary[i] + 2.0 * acceleration[i] * length[i];
        if (!std::isfinite(square) || square < 0.0) { boundary.fill(0.0); return false; }
        boundary[i + 1U] = (std::min)(boundary[i + 1U], std::sqrt(square));
    }
    return true;
}

void MotionCore::QueueCncFeedPlanDiagnostic(CncFeedPlanEvent event) noexcept
{
    auto& s = m_cncFeedLookahead;
    auto& e = s.event;
    const auto& v = m_Group.virtualAxis;
    e = s.plan;
    e.runtimeTick = m_ncSettleRuntimeCycleTick;
    e.tickValid = m_ncSettleRuntimeObserved && m_ncSettleRuntimeCycleValid &&
        m_ncSettleRuntimeCycleContiguous && e.runtimeTick != 0ULL;
    e.sequence = ++s.sequence;
    e.identity = m_Group.currentCmd.execution;
    e.lease = m_Group.currentCmd.ownerLease;
    e.axisMask = BuildMotionCommandAxisMask(m_Group.currentCmd);
    e.commandVelocity = v.currentCmdVel;
    e.outputVelocity = v.logicalCmdVel;
    e.endVelocity = v.targetEndVel;
    e.cruiseVelocity = v.maxVel_PPS;
    e.remainingPulse = (std::max)(0.0, v.finalTargetPos - v.planningPos);
    e.event = event;
    e.prefixLengthPulse = s.prefixLength;
    e.prefixRemainingPulse = s.prefixEnabled ? (std::max)(0.0, s.prefixLength - v.currentCmdPos) : 0.0;
    e.prefixLimitPPS = s.prefixLimit;
    e.prefixReservePulse = s.prefixReserve;
    e.authoredPrefixPPS = m_Group.currentCmd.cncPrefixVelocityPPS;
    if (!s.events.ProducerTryPush(e)) s.dropped.fetch_add(1U, std::memory_order_relaxed);
}

bool MotionCore::TryPopCncFeedPlanDiagnostic(CncFeedPlanDiagnostic& event) noexcept
{
    return m_cncFeedLookahead.events.ConsumerTryPop(event);
}

std::uint32_t MotionCore::GetCncFeedPlanDiagnosticDroppedCount() const noexcept
{
    return m_cncFeedLookahead.dropped.load(std::memory_order_acquire);
}

void MotionCore::FinishCncFeedLookahead() noexcept
{
    auto& s = m_cncFeedLookahead;
    if (s.loaded && MotionExecutionIdentityExactlyMatches(s.identity, m_Group.currentCmd.execution) &&
        s.lease.Matches(m_Group.currentCmd.ownerLease)) QueueCncFeedPlanDiagnostic(CncFeedPlanEvent::LEAVE);
    s.loaded = false;
}

void MotionCore::RefreshCncFeedLookahead(bool loading) noexcept
{
    auto& s = m_cncFeedLookahead;
    const auto& cmd = m_Group.currentCmd;
    auto& v = m_Group.virtualAxis;
    if (loading)
    {
        s.loaded = false;
        s.zeroInputCycles = 0U; s.blendEntered = false;
        s.prefixLength = s.prefixLimit = s.prefixReserve = s.prefixPreviousRaw = 0.0;
        s.prefixEnabled = s.prefixFastSeen = s.prefixBrakeSeen = false;
        s.prefixAuthoredSeen = false;
        s.observedDepth = (std::numeric_limits<std::size_t>::max)();
        s.plan = CncFeedPlanDiagnostic{};
    }
    if (!cmd.cncFeedLookahead || (cmd.mode != InterpolationMode::LINEAR && !cmd.pathCorePlanarCircle) || cmd.axisCount < 1 || cmd.axisCount > 3 ||
        !m_Group.isActive || m_safetyControlledStopInProgress ||
        HasPendingExecutionEpochChange() || HasPendingSafetyOrRecoveryRequests() ||
        GetCommandAuthorizationFailure(cmd) != MotionRejectReason::NONE) return;
    if (!loading && (!s.loaded || !MotionExecutionIdentityExactlyMatches(s.identity, cmd.execution) ||
        !s.lease.Matches(cmd.ownerLease))) return;
    if (m_Group.pathMode != PathMode::CONTINUOUS || m_Group.enableHistory || m_Group.enableTransform ||
        m_Group.jumpManager.state != JumpState::IDLE || m_pathHold.sourceSeen || IsPathCoreHoldExcursionDriving()) return;
    if (!std::isfinite(v.maxVel_PPS) || v.maxVel_PPS <= 0.0 || !std::isfinite(v.acc_PPS2) || v.acc_PPS2 <= 0.0 ||
        !std::isfinite(v.dec_PPS2) || v.dec_PPS2 <= 0.0 || v.velBuffer.size() > 400U ||
        !std::isfinite(v.currentCmdVel) || v.currentCmdVel < 0.0 || !std::isfinite(v.planningPos) ||
        !std::isfinite(v.finalTargetPos) || v.finalTargetPos <= 0.0) return;
    if (loading)
    {
        s.identity = cmd.execution; s.lease = cmd.ownerLease; s.loaded = true;
        // DK only changes the loaded prefix ceiling. Arc acceleration, curve
        // cap, future-source limits and every source seam remain DI/DJ values.
        double prefix = 0.0;
        if (cmd.cncCornerBlend && ResolveCncCornerBlend(cmd, prefix))
        {
            bool boundedHistory = v.currentCmdVel <= v.maxVel_PPS * (1.0 + 1e-12);
            double historyPeak = 0.0;
            // One scan of at most 400 slots on load, no extra per-cycle scan.
            for (std::size_t i = 0U; i < v.velBuffer.size(); ++i)
            {
                if (!std::isfinite(v.velBuffer[i]) || v.velBuffer[i] < 0.0) boundedHistory = false;
                else historyPeak = (std::max)(historyPeak, v.velBuffer[i]);
            }
            // DL keeps the full-authored and packet/no-boost gates. Only a
            // finite, distance-limited authored miss above a proven packet
            // schedule may select an intermediate loaded-prefix peak.
            const double authored = cmd.cncPrefixVelocityPPS == 0.0 ? cmd.targetVel : cmd.cncPrefixVelocityPPS;
            const double available = prefix - (std::max)(v.planningPos, v.currentCmdPos);
            bool authoredDistanceLimited = false;
            bool packetDistanceLimited = false;
            for (unsigned attempt = 0U; attempt < 2U && !s.prefixEnabled; ++attempt)
            {
                const double candidate = attempt == 0U ? authored : cmd.targetVel;
                if (attempt != 0U && authored == cmd.targetVel) break;
                if (!std::isfinite(candidate) || candidate < cmd.targetVel ||
                    candidate <= v.maxVel_PPS * (1.0 + 1e-12)) continue;
                const double reserve = candidate * CYCLE_TIME_SEC * (double(v.velBuffer.size()) + 8.0);
                const double rise = (candidate * candidate - v.currentCmdVel * v.currentCmdVel) / (2.0 * v.acc_PPS2);
                const double fall = (candidate * candidate - v.maxVel_PPS * v.maxVel_PPS) / (2.0 * v.dec_PPS2);
                const double required = reserve + (std::max)(0.0, rise) + fall;
                if (!boundedHistory || historyPeak > candidate * (1.0 + 1e-12) ||
                    !std::isfinite(reserve) || !std::isfinite(rise) || !std::isfinite(fall) ||
                    !std::isfinite(required)) continue;
                if (!(available > required))
                {
                    // DP: both authored and packet candidates passed every
                    // finite/history gate and failed only on distance.
                    // DQ: explicit equal/rising-F metadata uses one identical
                    // authored/packet candidate. Legacy zero metadata stays out.
                    packetDistanceLimited = std::isfinite(available) &&
                        ((attempt == 1U && authoredDistanceLimited) ||
                         (attempt == 0U && cmd.cncPrefixVelocityPPS > 0.0 &&
                          cmd.cncPrefixVelocityPPS == cmd.targetVel));
                    authoredDistanceLimited = attempt == 0U && std::isfinite(available) &&
                        cmd.cncPrefixVelocityPPS > cmd.targetVel;
                    continue;
                }
                double selected = candidate;
                double selectedReserve = reserve;
                if (attempt == 1U && authoredDistanceLimited)
                {
                    // Known-fitting lower endpoint, known-non-fitting upper.
                    // At most 24 scalar probes on load, no additional history
                    // scan, per-cycle search, packet edit, or source boundary.
                    double lower = candidate, upper = authored;
                    bool finiteSearch = true;
                    const auto fits = [&](double peak, double& peakReserve) noexcept
                    {
                        peakReserve = peak * CYCLE_TIME_SEC * (double(v.velBuffer.size()) + 8.0);
                        const double peakRise = (peak * peak - v.currentCmdVel * v.currentCmdVel) / (2.0 * v.acc_PPS2);
                        const double peakFall = (peak * peak - v.maxVel_PPS * v.maxVel_PPS) / (2.0 * v.dec_PPS2);
                        const double peakRequired = peakReserve + (std::max)(0.0, peakRise) + peakFall;
                        if (!std::isfinite(peakReserve) || !std::isfinite(peakRise) ||
                            !std::isfinite(peakFall) || !std::isfinite(peakRequired))
                        {
                            finiteSearch = false;
                            return false;
                        }
                        return available > peakRequired;
                    };
                    for (unsigned probe = 0U; probe < 24U; ++probe)
                    {
                        const double middle = lower + (upper - lower) * 0.5;
                        if (!(middle > lower && middle < upper)) break;
                        double probeReserve = 0.0;
                        if (fits(middle, probeReserve)) lower = middle;
                        else upper = middle;
                        if (!finiteSearch) break;
                    }
                    double verifiedReserve = 0.0;
                    if (finiteSearch && lower > candidate && lower < authored &&
                        fits(lower, verifiedReserve))
                    {
                        selected = lower;
                        selectedReserve = verifiedReserve;
                    }
                }
                s.prefixLength = prefix; s.prefixLimit = selected; s.prefixReserve = selectedReserve;
                s.prefixPreviousRaw = v.currentCmdVel; s.prefixEnabled = true;
            }
            // DP/DQ: a qualified Q prefix may be too short even for the
            // packet ceiling. Search below that ceiling only when the curve
            // speed itself fits and bounds the entire inherited raw/FIR state.
            // Full-authored, packet, and DL selections above stay unchanged.
            if (!s.prefixEnabled && packetDistanceLimited && boundedHistory &&
                v.currentCmdVel <= v.maxVel_PPS && historyPeak <= v.maxVel_PPS)
            {
                double lower = v.maxVel_PPS, upper = cmd.targetVel;
                bool finiteSearch = true;
                const auto fits = [&](double peak, double& peakReserve) noexcept
                {
                    peakReserve = peak * CYCLE_TIME_SEC * (double(v.velBuffer.size()) + 8.0);
                    const double peakRise = (peak * peak - v.currentCmdVel * v.currentCmdVel) / (2.0 * v.acc_PPS2);
                    const double peakFall = (peak * peak - v.maxVel_PPS * v.maxVel_PPS) / (2.0 * v.dec_PPS2);
                    const double peakRequired = peakReserve + (std::max)(0.0, peakRise) + peakFall;
                    if (!std::isfinite(peakReserve) || !std::isfinite(peakRise) ||
                        !std::isfinite(peakFall) || !std::isfinite(peakRequired))
                    {
                        finiteSearch = false;
                        return false;
                    }
                    return available > peakRequired;
                };
                double lowerReserve = 0.0;
                if (fits(lower, lowerReserve))
                {
                    // One load-time search: at most 24 scalar probes, then
                    // revalidate the selected peak with the same strict proof.
                    for (unsigned probe = 0U; probe < 24U; ++probe)
                    {
                        const double middle = lower + (upper - lower) * 0.5;
                        if (!(middle > lower && middle < upper)) break;
                        double probeReserve = 0.0;
                        if (fits(middle, probeReserve)) lower = middle;
                        else upper = middle;
                        if (!finiteSearch) break;
                    }
                    double verifiedReserve = 0.0;
                    if (finiteSearch && lower > v.maxVel_PPS * (1.0 + 1e-12) &&
                        lower < cmd.targetVel && fits(lower, verifiedReserve))
                    {
                        s.prefixLength = prefix; s.prefixLimit = lower; s.prefixReserve = verifiedReserve;
                        s.prefixPreviousRaw = v.currentCmdVel; s.prefixEnabled = true;
                    }
                }
            }
        }
    }
    if (v.inPosition || v.state != MotionState::MotionState_MOVING) return;
    const std::size_t depth = (std::min)(std::size_t(3U), m_Group.cmdQueue.size());
    if (!loading && s.observedDepth == depth) return;
    s.observedDepth = depth;
    s.lengths.fill(0.0); s.speeds.fill(v.maxVel_PPS);
    s.accelerations.fill(v.acc_PPS2); s.decelerations.fill(v.dec_PPS2);
    const double stepReserve = v.maxVel_PPS * CYCLE_TIME_SEC * 4.0;
    const double reserve = v.maxVel_PPS * CYCLE_TIME_SEC * (double(v.velBuffer.size()) + 4.0);
    const double remaining = (std::max)(0.0, v.finalTargetPos - v.planningPos);
    s.lengths[0U] = (std::max)(0.0, remaining - stepReserve);
    std::array<double, 3U> previous{}, tangent{};
    for (int i = 0; i < cmd.axisCount; ++i)
    {
        previous[std::size_t(i)] = cmd.targetPos[i];
        tangent[std::size_t(i)] = m_Group.ratio[i];
    }
    std::uint32_t circleMask = 0U, blendMask = cmd.cncCornerBlend ? 1U : 0U;
    if (cmd.pathCorePlanarCircle || cmd.cncCornerBlend)
    {
        circleMask = 1U;
        const double dx = cmd.targetPos[0] - cmd.centerPos[0];
        const double dy = cmd.targetPos[1] - cmd.centerPos[1];
        const double r = std::hypot(dx, dy);
        if (!std::isfinite(r) || r <= 0.0) return;
        tangent[0] = -double(cmd.dir) * dy / r; tangent[1] = double(cmd.dir) * dx / r;
    }
    s.nominalSpeeds.fill(0.0); s.nominalSpeeds[0U] = cmd.targetVel;
    MotionSegmentId previousSegment = cmd.execution.segmentId;
    int previousPC = cmd.sourceLinePC;
    CncFeedPlanStop stop = CncFeedPlanStop::QUEUE_END;
    std::size_t count = 1U;
    double horizon = s.lengths[0U];
    double prefixSpeed = v.maxVel_PPS;
    for (std::size_t i = 0U; i < depth; ++i)
    {
        auto& next = s.scratch;
        if (!TryPeekQueuedMotionCommandAt(i, next)) break;
        if (GetCommandAuthorizationFailure(next) != MotionRejectReason::NONE ||
            next.execution.epoch != cmd.execution.epoch || !next.ownerLease.Matches(cmd.ownerLease) ||
            previousSegment == (std::numeric_limits<MotionSegmentId>::max)() ||
            next.execution.segmentId != previousSegment + 1ULL || previousPC == (std::numeric_limits<int>::max)() ||
            next.sourceLinePC != previousPC + 1 || next.execution.sourceBlockId != next.sourceLinePC)
        {
            stop = CncFeedPlanStop::AUTHORITY; break;
        }
        if (!next.cncFeedLookahead || !IsMotionCommandConsumerGeometryValid(next, m_pContexts))
        {
            stop = CncFeedPlanStop::SCOPE; break;
        }
        if (!MotionCommandsHaveIdenticalAxisMapping(cmd, next)) { stop = CncFeedPlanStop::MAPPING; break; }
        const auto same = [](double a, double b) noexcept
        { return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1.0e-12 * (std::max)(1.0, (std::max)(std::abs(a), std::abs(b))); };
        // F may change; timing/configuration changes remain a stop boundary.
        // Match the actual LoadNextCommand acceleration law for EACH packet.
        if (!std::isfinite(next.targetVel) || next.targetVel < 1.0 ||
            !same(next.accTime, cmd.accTime) || !same(next.decTime, cmd.decTime))
        {
            stop = CncFeedPlanStop::SPEED; break;
        }
        double nextSpeed = 0.0, nextAcc = 0.0, nextDec = 0.0;
        if (!ComputeCncPathDynamics(next, nextSpeed, nextAcc, nextDec))
        {
            stop = CncFeedPlanStop::SPEED; break;
        }
        // A downshift can inherit a faster filter history. Never budget a
        // future segment using only its smaller local F.
        const double nextPrefixSpeed = (std::max)(prefixSpeed, nextSpeed);
        const double nextReserve = nextPrefixSpeed * CYCLE_TIME_SEC * (double(v.velBuffer.size()) + 4.0);
        if (!std::isfinite(nextReserve)) { stop = CncFeedPlanStop::SPEED; break; }
        double length = 0.0;
        std::array<double, 3U> startTangent{}, endTangent{};
        if (next.cncCornerBlend)
        {
            double prefix = 0.0;
            if (next.mem_startPos[0] != previous[0] || next.mem_startPos[1] != previous[1] ||
                !ResolveCncCornerBlend(next, prefix)) {
                stop = CncFeedPlanStop::SCOPE; break;
            }
            length = next.mem_totalDist;
            startTangent[0] = (next.mem_ratio[0] - next.mem_startPos[0]) / prefix;
            startTangent[1] = (next.mem_ratio[1] - next.mem_startPos[1]) / prefix;
            const double angle = next.mem_startAngle + next.mem_totalAngle;
            endTangent[0] = -double(next.dir) * std::sin(angle); endTangent[1] = double(next.dir) * std::cos(angle);
        }
        else if (next.pathCorePlanarCircle)
        {
            NCPathCoreArcPulseGeometry circle{};
            if (next.mem_startPos[0] != previous[0] || next.mem_startPos[1] != previous[1] ||
                !ResolveCncPlanarCircle(next, m_pContexts, circle)) {
                stop = CncFeedPlanStop::SCOPE; break;
            }
            length = circle.lengthPulse;
            const double sx = previous[0] - next.centerPos[0], sy = previous[1] - next.centerPos[1];
            const double ex = next.targetPos[0] - next.centerPos[0], ey = next.targetPos[1] - next.centerPos[1];
            const double sr = std::hypot(sx, sy), er = std::hypot(ex, ey);
            if (!std::isfinite(sr) || !std::isfinite(er) || sr <= 0.0 || er <= 0.0)
            {
                stop = CncFeedPlanStop::SCOPE; break;
            }
            startTangent[0] = -double(next.dir) * sy / sr; startTangent[1] = double(next.dir) * sx / sr;
            endTangent[0] = -double(next.dir) * ey / er; endTangent[1] = double(next.dir) * ex / er;
        }
        else
        {
            for (int j = 0; j < cmd.axisCount; ++j)
            {
                startTangent[std::size_t(j)] = next.targetPos[j] - previous[std::size_t(j)];
                length = std::hypot(length, startTangent[std::size_t(j)]);
            }
            if (std::isfinite(length) && length > 0.0)
                for (int j = 0; j < cmd.axisCount; ++j) startTangent[std::size_t(j)] /= length;
            endTangent = startTangent;
        }
        if (!std::isfinite(length) || length <= nextReserve) { stop = CncFeedPlanStop::SHORT_SEGMENT; break; }
        bool tangentMatch = true;
        for (int j = 0; j < cmd.axisCount; ++j)
            if (!same(startTangent[std::size_t(j)], tangent[std::size_t(j)])) tangentMatch = false;
        if (!tangentMatch) { stop = CncFeedPlanStop::DIRECTION; break; }
        tangent = endTangent;
        if (next.pathCorePlanarCircle || next.cncCornerBlend) circleMask |= (1U << static_cast<unsigned>(count));
        if (next.cncCornerBlend) blendMask |= (1U << static_cast<unsigned>(count));
        s.lengths[count] = length - nextReserve;
        s.speeds[count] = nextSpeed;
        s.nominalSpeeds[count] = next.targetVel;
        s.accelerations[count] = nextAcc;
        s.decelerations[count] = nextDec;
        prefixSpeed = nextPrefixSpeed;
        horizon += s.lengths[count];
        ++count;
        previousSegment = next.execution.segmentId; previousPC = next.sourceLinePC;
        for (int j = 0; j < cmd.axisCount; ++j) previous[std::size_t(j)] = next.targetPos[j];
        if (count == 4U) stop = CncFeedPlanStop::HORIZON;
    }
    if (!BuildCncFeedStopPlan(s.lengths, s.speeds, s.accelerations, s.decelerations, count,
        v.currentCmdVel, s.boundaries)) return;
    const double proposed = count > 1U ? s.boundaries[1U] : 0.0;
    // New arrivals can relax only a still-reachable target. No backward edits of
    // queued packets, no waiting for the NC task, and no deletion of a stop fence.
    const bool late = !loading && remaining <= reserve;
    const bool increase = proposed > v.targetEndVel + 0.1 && !late;
    if (HasPendingExecutionEpochChange() || HasPendingSafetyOrRecoveryRequests() ||
        GetCommandAuthorizationFailure(cmd) != MotionRejectReason::NONE) return;
    if (increase) v.targetEndVel = proposed;
    s.plan.horizon = static_cast<std::uint32_t>(count);
    s.plan.horizonPulse = horizon; s.plan.reservePulse = reserve;
    s.plan.lastSegment = previousSegment;
    s.plan.stop = late && proposed > v.targetEndVel + 0.1 ? CncFeedPlanStop::LATE : stop;
    s.plan.exitVelocity.fill(0.0);
    s.plan.nominalVelocity.fill(0.0); s.plan.limitedVelocity.fill(0.0);
    s.plan.circleMask = circleMask; s.plan.blendMask = blendMask;
    s.plan.radiusPulse = (cmd.pathCorePlanarCircle || cmd.cncCornerBlend) ? cmd.startRadius : 0.0;
    s.plan.entryCarry = s.entryCarry;
    s.plan.handoffFrom = s.handoffFrom;
    for (std::size_t i = 0U; i < count; ++i)
    {
        s.plan.exitVelocity[i] = s.boundaries[i + 1U];
        s.plan.nominalVelocity[i] = s.nominalSpeeds[i];
        s.plan.limitedVelocity[i] = s.speeds[i];
    }
    s.plan.exitVelocity[0U] = v.targetEndVel;
    QueueCncFeedPlanDiagnostic(loading ? CncFeedPlanEvent::LOAD :
        increase ? CncFeedPlanEvent::EXTEND : CncFeedPlanEvent::KEEP_PLAN);
}

void MotionCore::LoadNextCommand(bool cncBoundaryCrossing)
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

    if (!m_Group.isActive)
    {
        m_safetyControlledStopInProgress = false;
        m_safetyControlledStopOwnerLease = MotionOwnerLease{};
        m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_safetyControlledStopRequestTicket = 0U;
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

    // DG same-cycle path split: no physical command beyond the old endpoint
    // has been distributed yet. Transfer the sub-cycle residual along the
    // next authorized primitive, not along an extension of the old tangent.
    std::array<double, MAX_AXES> crossingStart{};
    double crossingDistance = 0.0;
    MotionSegmentId crossingFrom = MOTION_SEGMENT_ID_INVALID;
    if (cncBoundaryCrossing)
    {
        const MotionCommand& previous = m_Group.currentCmd;
        const auto& proof = m_cncFeedLookahead.plan;
        crossingDistance = vAxis.currentCmdPos - vAxis.finalTargetPos;
        const double oneStep = vAxis.maxVel_PPS * 0.00025;
        if (!m_Group.isActive || !vAxis.inPosition || m_safetyControlledStopInProgress ||
            HasPendingSafetyOrRecoveryRequests() || m_Group.enableTransform ||
            m_Group.jumpManager.state != JumpState::IDLE || m_pathHold.unionActive ||
            !previous.cncFeedLookahead || !frontCommand.cncFeedLookahead ||
            !(previous.pathCorePlanarCircle || frontCommand.pathCorePlanarCircle || previous.cncCornerBlend || frontCommand.cncCornerBlend) ||
            !MotionCommandsHaveIdenticalAxisMapping(previous, frontCommand) ||
            GetCommandAuthorizationFailure(previous) != MotionRejectReason::NONE ||
            previous.execution.epoch != frontCommand.execution.epoch ||
            !previous.ownerLease.Matches(frontCommand.ownerLease) ||
            previous.execution.segmentId == (std::numeric_limits<MotionSegmentId>::max)() ||
            frontCommand.execution.segmentId != previous.execution.segmentId + 1ULL ||
            frontCommand.execution.sourceBlockId != previous.execution.sourceBlockId + 1U ||
            !m_cncFeedLookahead.loaded ||
            !MotionExecutionIdentityExactlyMatches(m_cncFeedLookahead.identity, previous.execution) ||
            !m_cncFeedLookahead.lease.Matches(previous.ownerLease) || proof.horizon < 2U ||
            vAxis.targetEndVel <= 0.1 || !std::isfinite(crossingDistance) || crossingDistance < 0.0 ||
            !std::isfinite(oneStep) || crossingDistance > oneStep * (1.0 + 1e-10)) return;
        for (int slot = 0; slot < previous.axisCount; ++slot)
        {
            crossingStart[slot] = previous.targetPos[slot];
            if (!std::isfinite(crossingStart[slot]) ||
                ((frontCommand.pathCorePlanarCircle || frontCommand.cncCornerBlend) && frontCommand.mem_startPos[slot] != crossingStart[slot])) return;
        }
        crossingFrom = previous.execution.segmentId;
    }
    else if (m_Group.isActive && vAxis.inPosition &&
        m_Group.currentCmd.cncFeedLookahead && frontCommand.cncFeedLookahead &&
        m_Group.currentCmd.mode == InterpolationMode::LINEAR &&
        frontCommand.mode == InterpolationMode::LINEAR &&
        !m_Group.currentCmd.cncCornerBlend && !frontCommand.cncCornerBlend &&
        !m_Group.currentCmd.pathCorePlanarCircle && !frontCommand.pathCorePlanarCircle &&
        vAxis.targetEndVel > 0.1 && vAxis.currentCmdVel > 0.0 && vAxis.logicalCmdVel > 0.0 &&
        !m_safetyControlledStopInProgress && !HasPendingSafetyOrRecoveryRequests() &&
        !HasPendingExecutionEpochChange() && !m_pathHold.unionActive &&
        m_cncFeedLookahead.loaded && m_cncFeedLookahead.plan.horizon >= 2U &&
        MotionExecutionIdentityExactlyMatches(m_cncFeedLookahead.identity, m_Group.currentCmd.execution) &&
        m_cncFeedLookahead.lease.Matches(m_Group.currentCmd.ownerLease) &&
        MotionCommandsHaveIdenticalAxisMapping(m_Group.currentCmd, frontCommand) &&
        GetCommandAuthorizationFailure(m_Group.currentCmd) == MotionRejectReason::NONE &&
        m_Group.currentCmd.execution.epoch == frontCommand.execution.epoch &&
        m_Group.currentCmd.ownerLease.Matches(frontCommand.ownerLease) &&
        m_Group.currentCmd.execution.segmentId != (std::numeric_limits<MotionSegmentId>::max)() &&
        frontCommand.execution.segmentId == m_Group.currentCmd.execution.segmentId + 1ULL &&
        m_Group.currentCmd.execution.sourceBlockId != (std::numeric_limits<decltype(m_Group.currentCmd.execution.sourceBlockId)>::max)() &&
        frontCommand.execution.sourceBlockId == m_Group.currentCmd.execution.sourceBlockId + 1U)
    {
        // EK: identify the successful pure-line predecessor in diagnostics.
        // The existing line loader still uses its actual start and zero carry;
        // this does not select the DG geometric boundary-crossing path.
        crossingFrom = m_Group.currentCmd.execution.segmentId;
    }

    const bool pathModeDriverOverrideActive =
        m_Group.pathMode == PathMode::PATH_SERVO ||
        m_Group.pathMode == PathMode::JUMP_TRACKING;
    const MotionCommandPathModeAuthorityDecision
        frontPathModeAuthorityDecision =
        ResolveMotionCommandPathModeAuthorityDecision(
            frontCommand.commandPathMode,
            frontCommand.replayTerminalAlreadyPublished,
            pathModeDriverOverrideActive);
    if (frontPathModeAuthorityDecision ==
        MotionCommandPathModeAuthorityDecision::REJECT_INVALID ||
        frontPathModeAuthorityDecision ==
        MotionCommandPathModeAuthorityDecision::REJECT_DRIVER_OVERRIDE)
    {
        // K.6.1 fail-closed boundary.  Consume only the exact peeked identity,
        // publish its terminal rejection and open the existing Alarm 3021
        // lifecycle transaction.  Never let a normal G00 steal driver mode.
        (void)RejectFrontCommandForPathModeAuthority(
            frontCommand,
            frontPathModeAuthorityDecision);
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
    // 3. Pre-pop admission for a newly claimed physical axis.
    //
    // The NC Program-Start gate deliberately samples readiness before it
    // promotes the run to RUN.  That proof cannot reserve AxisContext across
    // the NC -> 250 us handoff, so a legitimate settling sample may become
    // temporarily not-ready before this RT consumer sees the first command.
    // Never consume that command and turn the transient into AL3021.  Leave
    // the exact front identity in the SPSC queue and retry on the next RT
    // pass.  No mapping, planner, lifecycle, or physical-output state has
    // changed at this point, so deferral is fail-closed.
    //
    // Geometry, path-mode authority, Epoch/Owner authorization, and the
    // outgoing exact-stop proof have already run above.  The post-pop check
    // below remains an invariant backstop; this preflight closes the only
    // normal transient readiness window without weakening those hard faults.
    // ======================================================
    for (int slot = 0; slot < frontCommand.axisCount; ++slot)
    {
        const int axisIndex = frontCommand.axisIndices[slot];
        const bool incomingOnly =
            !hasOutgoingCommand ||
            !MotionCommandHasAxis(m_Group.currentCmd, axisIndex);
        if (!incomingOnly)
        {
            continue;
        }

        const AxisContext& incomingAxis = (*m_pContexts)[axisIndex];
        if (!IsIncomingPhysicalAxisReadyForGroup(incomingAxis) &&
            IsIncomingPhysicalAxisReadinessTransient(incomingAxis))
        {
            // Distinguish a live, exact queued source waiting only for physical
            // following from an unavailable source. This never admits motion.
            const double following = std::abs(incomingAxis.currentCmdPos - incomingAxis.currentActPos);
            if (!m_Group.isActive && !m_pathHold.sourceSeen &&
                m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::ARMED &&
                m_pathHold.generation == m_pathHoldGeneration.load(std::memory_order_acquire) &&
                MotionExecutionIdentityExactlyMatches(frontCommand.execution, m_pathHold.status.identity) &&
                frontCommand.ownerLease.Matches(m_pathHold.status.ownerLease) &&
                IsPathCoreAdmissionWaitAxisHealthy(incomingAxis) &&
                std::isfinite(following) && following > incomingAxis.inPositionWindow_Pulse &&
                m_ncSettleRuntimeObserved && m_ncSettleRuntimeCycleValid &&
                m_ncSettleRuntimeCycleContiguous && m_ncSettleRuntimeCycleTick != 0ULL)
            {
                if (m_pathHold.status.admissionCorrectionAxis != axisIndex)
                {
                    ResetPathCoreAdmissionCorrectionDiagnostic();
                    m_pathHold.status.admissionCorrectionAxis = axisIndex;
                }
                m_pathAdmissionCorrectionScopeMask = BuildMotionCommandAxisMask(frontCommand);
                m_pathHold.status.admissionPending = true;
                m_pathHold.status.admissionWaitTick = m_ncSettleRuntimeCycleTick;
                m_pathHold.status.admissionWaitAxis = axisIndex;
                m_pathHold.status.admissionFollowingError = following;
                m_pathHold.status.admissionWindowPulse = incomingAxis.inPositionWindow_Pulse;
            }
            return;
        }
    }

    // ======================================================
    // 4. 由 250 us Consumer 取得下一條 Motion Command
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

    const MotionCommandPathModeAuthorityDecision
        commandPathModeAuthorityDecision =
        ResolveMotionCommandPathModeAuthorityDecision(
            cmd.commandPathMode,
            cmd.replayTerminalAlreadyPublished,
            pathModeDriverOverrideActive);

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
        commandPathModeAuthorityDecision !=
        frontPathModeAuthorityDecision ||
        !IsMotionCommandConsumerGeometryValid(cmd, m_pContexts) ||
        (cmd.pathCorePlanarCircle && cmd.sourceTranslation.cutterMode != 40 &&
            !CutterCircleStartMatches(cmd, m_pContexts)) ||
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
        FinishCncP1Diagnostic();
        FinishCncFeedLookahead();
        CompleteTrackedMotionCommand(
            m_Group.currentCmd);
    }

    if (m_Group.isActive &&
        m_Group.enableHistory && !m_Group.currentCmd.pathCoreRetainedTraversal &&
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
    InvalidateCncLineEndpointProof();
    m_Group.currentCmd = cmd;

    // Stage NC-0.2K.6.1: the fully authorized, post-pop command becomes the
    // planner authority only after the lifecycle/mapping handoff commits.
    // Producer code never writes m_Group.pathMode for G00.  Replay and
    // UNSPECIFIED decisions preserve their accepted legacy/driver authority.
    CommitCommandPathModeConsumerAuthority(
        m_Group.currentCmd,
        commandPathModeAuthorityDecision);


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
    if (cmd.pathCoreRetainedTraversal)
    {
        // Canonical values remain the complete ORIGINAL source. Only the
        // scalar travel distance and parameter boundaries use the interval.
        double startU = 0.0, endU = 0.0, intervalDistance = 0.0;
        if (!GetMotionRetainedInterval(cmd, startU, endU, intervalDistance))
        {
            FailDerivedConsumerGeometry();
            return;
        }
        if (m_Group.pathMode != PathMode::EXACT_STOP || m_Group.enableHistory ||
            m_Group.enableTransform || m_Group.jumpManager.state != JumpState::IDLE ||
            !DoesMotionRetainedStartMatch(cmd, m_pContexts))
        {
            FailDerivedConsumerGeometry();
            return;
        }
        for (int slot = 0; slot < cmd.axisCount; ++slot)
        {
            const int axis = cmd.axisIndices[slot];
            m_Group.axisIndices[slot] = axis;
            if (!EvaluateMotionRetainedPulseCanonical(cmd, static_cast<std::size_t>(axis), startU, m_Group.startPos[slot]))
            {
                FailDerivedConsumerGeometry();
                return;
            }
            m_Group.ratio[slot] = cmd.mem_totalDist == 0.0 ? 0.0 :
                (cmd.mem_ratio[axis] - cmd.mem_startPos[axis]) / cmd.mem_totalDist *
                (cmd.pathCoreRetainedReverse ? -1.0 : 1.0);
        }
        m_Group.radius = cmd.mem_radius;
        m_Group.startAngle = cmd.mem_startAngle;
        m_Group.totalAngle = cmd.mem_totalAngle;
        m_Group.centerX = cmd.mem_centerX;
        m_Group.centerY = cmd.mem_centerY;
        m_Group.totalDist3D = intervalDistance;
        vAxis.finalTargetPos = intervalDistance;
        if (intervalDistance == 0.0)
        {
            CompleteWithoutMotion();
            return;
        }
        if (HasInvalidZeroVelocity()) return;
        vAxis.inPosition = false;
        for (int slot = 0; slot < cmd.axisCount; ++slot)
        {
            AxisContext& realAxis = (*m_pContexts)[cmd.axisIndices[slot]];
            realAxis.state = MotionState::MotionState_INTERPOLATING;
            realAxis.inPosition = false;
        }
    }
    else if (cmd.cncCornerBlend)
    {
        double prefix = 0.0;
        if (!ResolveCncCornerBlend(cmd, prefix) || m_Group.enableTransform || m_Group.enableHistory ||
            (!cncBoundaryCrossing && !CncCircleStartMatches(cmd, m_pContexts)))
        {
            FailDerivedConsumerGeometry(); return;
        }
        if (HasInvalidZeroVelocity()) return;
        for (unsigned i = 0U; i < 2U; ++i)
        {
            m_Group.axisIndices[i] = cmd.axisIndices[i];
            m_Group.startPos[i] = cmd.mem_startPos[i];
            m_Group.ratio[i] = (cmd.mem_ratio[i] - cmd.mem_startPos[i]) / prefix;
            AxisContext& axis = (*m_pContexts)[i];
            axis.state = MotionState::MotionState_INTERPOLATING; axis.inPosition = false;
        }
        m_Group.radius = cmd.mem_radius; m_Group.startAngle = cmd.mem_startAngle; m_Group.totalAngle = cmd.mem_totalAngle;
        m_Group.centerX = cmd.mem_centerX; m_Group.centerY = cmd.mem_centerY;
        m_Group.totalDist3D = cmd.mem_totalDist; vAxis.finalTargetPos = cmd.mem_totalDist; vAxis.inPosition = false;
    }
    else if (m_Group.mode == InterpolationMode::LINEAR)
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
            m_Group.startPos[i] = cncBoundaryCrossing ? crossingStart[i] : realAxis.logicalCmdPos.Load();


            // ----------------------------------------------
            // 目標
            // ----------------------------------------------
            double actualTarget = cmd.targetPos[i];


            // ----------------------------------------------
            // Rotary shortest path
            // ----------------------------------------------
            if (realAxis.axisType == AxisType::ROTARY && realAxis.useShortestPath)
            {
                double pulsePerUnit = 0.0;
                if (!TryGetMotionPulsePerUnit(realAxis.resolution_PPR, realAxis.finalLead,
                    true, pulsePerUnit) ||
                    !TryResolveMotionTargetPulse(m_Group.startPos[i], actualTarget,
                        pulsePerUnit, true, realAxis.rotaryModulo, actualTarget))
                {
                    derivedGeometryValid = false;
                    break;
                }
                const double resolvedTargetUnits = actualTarget / pulsePerUnit;
                if (!std::isfinite(resolvedTargetUnits))
                {
                    derivedGeometryValid = false;
                    break;
                }
                if (m_pCoordMgr != nullptr &&
                    !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(realAxis, resolvedTargetUnits))
                {
                    const int travelAlarm = m_pCoordMgr->GetSoftwareTravelLimitAlarmCode(
                        realAxis, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    AlarmManager::GetInstance().Trigger(travelAlarm, cmd.sourceLinePC, idx);
                    if (travelAlarm == AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG)
                    {
                        // Release the lifecycle reservation before publishing
                        // Safety. The following geometry cleanup then retires
                        // this command by its superseded authority, without a
                        // second, unrelated mapping-integrity alarm.
                        lifecycleCommit.Release();
                        RequestEmergencyStopAllAxes();
                    }
                    derivedGeometryValid = false;
                    break;
                }


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
        // DQ_FIX1: CNC lookahead lines need real interpolation/START even
        // inside the physical settling window. Preserve ordinary positioning
        // and the mathematical tiny-distance guard below.
        if (alreadyAtTarget && !cmd.cncFeedLookahead)
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
        m_Group.startPos[0] = cncBoundaryCrossing ? crossingStart[0] : (*m_pContexts)[axisX].logicalCmdPos.Load();

        m_Group.startPos[1] = cncBoundaryCrossing ? crossingStart[1] : (*m_pContexts)[axisY].logicalCmdPos.Load();


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


        double totalDist3D = 0.0;
        if (cmd.pathCorePlanarCircle)
        {
            NCPathCoreArcPulseGeometry circle{};
            double sourceRoundoffPulse = 0.0, sourceRoundoffMM = 0.0;
            if (!TryGetMotionArcSourceRoundoffPulse(cmd, m_pContexts, sourceRoundoffPulse, sourceRoundoffMM) ||
                !IsMotionArcPulsePrecisionWithinBudget(m_pContexts, sx, sy, ex, ey, cx, cy,
                    cmd.startRadius, sourceRoundoffMM, cmd.sourcePlaneMode))
            {
                FailDerivedConsumerGeometry();
                return;
            }
            // ED: queued full circles use the immutable closed start/end below.
            // At a zero-speed seam the sampled start may carry prior FIR roundoff;
            // retain CncCircleStartMatches bounds before using canonical geometry.
            if (m_Group.pathMode != (cmd.cncFeedLookahead ? PathMode::CONTINUOUS : PathMode::EXACT_STOP) ||
                m_Group.enableTransform || (cmd.cncFeedLookahead && !cncBoundaryCrossing && !CncCircleStartMatches(cmd, m_pContexts)) ||
                (cmd.sourceTranslation.cutterMode != 40 &&
                    (!ResolveCutterPlanarCircle(cmd, m_pContexts, circle) || !CutterCircleStartMatches(cmd, m_pContexts))) ||
                (!(cmd.cncFeedLookahead && cmd.pathCoreFullCircle) &&
                    !ResolveNCPathCorePlanarCirclePulse(sx, sy, ex, ey, cx, cy,
                        cmd.startRadius, cmd.dir, cmd.pathCoreFullCircle, circle, sourceRoundoffPulse)))
            {
                FailDerivedConsumerGeometry();
                return;
            }
            if (cmd.cncFeedLookahead)
            {
                if (!ResolveCncPlanarCircle(cmd, m_pContexts, circle)) { FailDerivedConsumerGeometry(); return; }
                m_Group.startPos[0] = cmd.mem_startPos[0]; m_Group.startPos[1] = cmd.mem_startPos[1];
            }
            if (cmd.sourceTranslation.cutterMode != 40)
            {
                if (!ResolveCutterPlanarCircle(cmd, m_pContexts, circle)) { FailDerivedConsumerGeometry(); return; }
                // Group startPos is packet-ordered; the saved proof is XYZ.
                // Source/mapping validation above has proved both indices.
                m_Group.startPos[0] = cmd.mem_startPos[cmd.axisIndices[0]];
                m_Group.startPos[1] = cmd.mem_startPos[cmd.axisIndices[1]];
            }
            // Both radii are intentionally identical. A rounding seam in the
            // observed start must never select the variable-radius integral.
            m_Group.startAngle = circle.startAngle;
            m_Group.totalAngle = circle.sweepRadians;
            m_Group.radius = circle.radius;
            m_Group.currentCmd.startRadius = circle.radius;
            m_Group.currentCmd.endRadius = circle.radius;
            totalDist3D = circle.lengthPulse;
        }
        else
        {
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
            totalDist3D = CalcSpiralArcLengthAtProgress(1.0, startRadius, endRadius, totalAngle, deltaZ);

        }

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

    if (cncBoundaryCrossing && crossingDistance >= vAxis.finalTargetPos)
    {
        FailDerivedConsumerGeometry(); return;
    }

    // S-Curve 前一段殘留距離
    vAxis.planningPos = trappedDist + crossingDistance;

    // 新路徑 Virtual Position 從 0 開始
    vAxis.currentCmdPos = crossingDistance;


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

    if (cmd.cncFeedLookahead && (cmd.pathCorePlanarCircle || cmd.cncCornerBlend))
    {
        if (!ComputeCncPathDynamics(cmd, vAxis.maxVel_PPS, vAxis.acc_PPS2, vAxis.dec_PPS2))
        {
            FailDerivedConsumerGeometry(); return;
        }
        vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;
    }

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

    const bool cncNextVisible = !cmd.cncFeedLookahead && m_Group.pathMode == PathMode::CONTINUOUS &&
        TryPeekNextMotionCommand(nextCmd);
    if (cncNextVisible && !nextCmd.cncFeedLookahead &&
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
    ArmCncP1LateJunction(cncNextVisible, nextCmd);
    m_cncFeedLookahead.entryCarry = crossingDistance;
    m_cncFeedLookahead.handoffFrom = crossingFrom;
    RefreshCncFeedLookahead(true);


    // ======================================================
    // 13. History 幾何快照
    // ======================================================
    if (m_Group.enableHistory && !m_Group.currentCmd.pathCoreRetainedTraversal)
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
            (commandSource == MotionCommandSource::NC_MEMORY &&
                !IsNCTranslationSnapshotEmpty(m_pendingTranslation) &&
                (axisIndex > 2 || (*m_pContexts)[axisIndex].axisType != AxisType::LINEAR)) ||
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

    if (!IsPendingCommandTranslationValid(commandSource) ||
        (commandSource == MotionCommandSource::NC_MEMORY && m_pendingToolRadMode != 40))
    {
        RejectNonGeometryProducerMotionCommand(invalidCommand, commandEpoch,
            commandSource, commandOwnerLease, MotionRejectReason::NOT_READY);
        return;
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
    if (commandSource == MotionCommandSource::NC_MEMORY) cmd.sourceTranslation = m_pendingTranslation;
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
        MotionExecutionEpoch publishedEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        if (!TryPublishOwnerAuthorizedAbortingExecutionEpoch(
            commandSource,
            commandEpoch,
            commandOwnerLease,
            publishedEpoch))
        {
            const bool ownerStillCurrent =
                IsMotionOwnerLeaseCurrent(commandOwnerLease);
            const MotionExecutionEpoch currentEpoch =
                GetCurrentExecutionEpoch();
            const MotionRejectReason rejectReason =
                !ownerStillCurrent
                ? MotionRejectReason::OWNER_CONFLICT
                : (currentEpoch != commandEpoch &&
                    currentEpoch != publishedEpoch)
                ? MotionRejectReason::STALE_EPOCH
                : MotionRejectReason::NOT_READY;

            RejectNonGeometryProducerMotionCommand(
                invalidCommand,
                publishedEpoch != MOTION_EXECUTION_EPOCH_INVALID
                ? publishedEpoch
                : commandEpoch,
                commandSource,
                commandOwnerLease,
                rejectReason);
            return;
        }

        commandEpoch = publishedEpoch;

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
    SupersedeResetControlledStop();
    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    if (!EnsureSafetyMotionActionTicket(safetyRequestTicket))
    {
        EmergencyStopAllAxesImpl(true);
        EndExecutionDrainAcknowledgementRevocation();
        return;
    }
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);
    StopGroupImpl(true);
    (void)CompleteSafetyMotionActionTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();
}

void MotionCore::StopGroupImpl(
    bool publishExecutionEpoch)
{
    const MotionOwnerLease safetyLease = GetMotionOwnerLease();
    if (safetyLease.owner != MotionOwner::SAFETY ||
        !IsMotionOwnerLeaseCurrent(safetyLease))
    {
        // Never create a controlled-stop exception without exact SAFETY
        // authority. The RT E-stop path still zeroes every axis even if its
        // bounded handshake must be completed on the next pass.
        EmergencyStopAllAxesImpl(true);
        return;
    }

    const bool hadExecutionToInvalidate =
        m_Group.isActive ||
        !m_Group.cmdQueue.empty();

    MotionExecutionEpoch controlledStopEpoch =
        GetCurrentExecutionEpoch();

    if (publishExecutionEpoch && hadExecutionToInvalidate)
    {
        controlledStopEpoch = BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
        if (controlledStopEpoch == MOTION_EXECUTION_EPOCH_INVALID)
        {
            EmergencyStopAllAxesImpl(true);
            return;
        }
    }

    // 尚未開始的 Future Queue 已由新 Epoch 取消；
    // 實際淘汰由 250 us Consumer 執行。

    if (!m_Group.isActive)
    {
        m_safetyControlledStopInProgress = false;
        m_safetyControlledStopOwnerLease = MotionOwnerLease{};
        m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_safetyControlledStopRequestTicket = 0U;

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

    // This RT-owned token spans the whole controlled-stop lifecycle. The
    // virtual state is not a sufficient proxy: MOVING truncates its target
    // first, and it may reach IDLE while physical axes are still converging.
    m_safetyControlledStopInProgress = true;
    m_safetyControlledStopOwnerLease = safetyLease;
    m_safetyControlledStopEpoch = controlledStopEpoch;
    m_safetyControlledStopRequestTicket =
        UnpackMotionOwnerSafetyRequestTicket(
            m_motionOwnerState.load(std::memory_order_acquire));

    // Jump and PATH_SERVO own independent velocity drivers that can overwrite
    // StopMove on the next line. They are not eligible for the controlled-
    // stop exception; contain them with the all-axis E-stop path instead.
    if (m_Group.jumpManager.state != JumpState::IDLE ||
        m_Group.pathMode == PathMode::PATH_SERVO ||
        m_Group.pathMode == PathMode::JUMP_TRACKING)
    {
        EmergencyStopAllAxesImpl(false);
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
    // Direct group emergency containment is equivalent to the all-axis
    // emergency path for a staged operator RESET: discard the benign
    // controlled-stop request rather than allowing it to revive afterward.
    SupersedeResetControlledStop();

    BeginExecutionDrainAcknowledgementRevocation();
    std::uint32_t safetyRequestTicket =
        PublishSafetyMotionRequestTicket(true);
    (void)EnsureSafetyMotionActionTicket(safetyRequestTicket);
    (void)TryTakeSafetyMotionOwnerForTicket(safetyRequestTicket);

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

    m_safetyControlledStopInProgress = false;
    m_safetyControlledStopOwnerLease = MotionOwnerLease{};
    m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_safetyControlledStopRequestTicket = 0U;
    (void)CompleteSafetyMotionActionTicket(safetyRequestTicket);
    EndExecutionDrainAcknowledgementRevocation();
    TryAcknowledgeAppliedSafetyMotionRequests();

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

    InvalidateCncLineEndpointProof();
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
    // A waiting heartbeat must be earned anew by this pass's real load gate.
    m_pathAdmissionCorrectionScopeMask = 0U;
    m_pathHold.status.admissionPending = false;
    m_pathHold.status.admissionWaitTick = 0ULL;
    m_pathHold.status.admissionWaitAxis = -1;
    m_pathHold.status.admissionFollowingError = 0.0;
    m_pathHold.status.admissionWindowPulse = 0.0;
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

    // RESET during a clean G00/G01 is a controlled-stop request, not an
    // immediate safety-zero request. Its producer only publishes a
    // pre-ticket bit; RT creates and consumes the exact SAFETY ticket/Epoch
    // here before any final PDO command is constructed.
    ApplyPendingResetControlledStopRequest();

    // A direct Safety takeover or a request producer paused between ticket
    // publication and its mailbox write is still an observable stop intent.
    // Help the bounded handshake here; never acknowledge a producer that is
    // still inside the revocation window.
    if (HasUnacknowledgedSafetyMotionRequest())
    {
        const std::uint64_t ownerState =
            m_motionOwnerState.load(std::memory_order_acquire);
        (void)TryTakeSafetyMotionOwnerForTicket(
            UnpackMotionOwnerSafetyRequestTicket(ownerState));
        ApplyPendingExecutionEpochChange();
        TryAcknowledgeAppliedSafetyMotionRequests();
    }

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

    if (HasUnacknowledgedSafetyMotionRequest() &&
        !m_safetyControlledStopInProgress)
    {
        return;
    }

    if (!m_Group.isActive)
    {
        m_safetyControlledStopInProgress = false;
        m_safetyControlledStopOwnerLease = MotionOwnerLease{};
        m_safetyControlledStopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_safetyControlledStopRequestTicket = 0U;
    }

    // NC-0.2K.6.2: an owner takeover and its request mailbox are separate
    // atomic publications. If the request producer is preempted after the
    // SAFETY Owner/Epoch handshake, the former AUTO currentCmd is already
    // unauthorized and must not advance for even one more Runtime pass.
    //
    // Keep the active geometry and virtual velocity intact while fenced so a
    // later STOP request can still enter the existing controlled-deceleration
    // path. Once StopGroupImpl() changes the virtual axis to STOPPING, that
    // bounded stop trajectory is the only unauthorized active state allowed
    // to continue. Emergency/Reset paths deactivate or similarly stop it.
    if (m_Group.isActive &&
        !m_safetyControlledStopInProgress &&
        GetCommandAuthorizationFailure(m_Group.currentCmd) !=
        MotionRejectReason::NONE)
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

    // BZ canonical storage has no history/B2/transform interpretation.
    if (m_Group.isActive && m_Group.currentCmd.pathCoreRetainedTraversal &&
        (m_Group.enableHistory || m_Group.enableTransform ||
            m_Group.jumpManager.state != JumpState::IDLE ||
            m_Group.pathMode != PathMode::EXACT_STOP))
    {
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        return;
    }

    // DH geometry has no legacy history/transform interpretation. Ordinary
    // HOLD uses [0,1] override; an unsupported speed-up must not bypass the
    // compound's curvature cap. The protected safety-stop path stays prior.
    if (m_Group.isActive && m_Group.currentCmd.cncCornerBlend && !m_safetyControlledStopInProgress &&
        (m_Group.enableHistory || m_Group.enableTransform || m_Group.jumpManager.state != JumpState::IDLE ||
            m_Group.pathMode != PathMode::CONTINUOUS || !std::isfinite(m_Group.feedrateOverride) ||
            m_Group.feedrateOverride < 0.0 || m_Group.feedrateOverride > 1.0))
    {
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        return;
    }

    if (IsPathCoreHoldExcursionDriving() && !m_safetyControlledStopInProgress &&
        GetCommandAuthorizationFailure(m_Group.currentCmd) == MotionRejectReason::NONE &&
        (m_Group.enableHistory || m_Group.enableTransform ||
            m_Group.jumpManager.state != JumpState::IDLE || m_Group.pathMode != PathMode::EXACT_STOP))
    {
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        return;
    }

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
            (m_pathHold.unionActive ? IsPathCoreHoldEffectiveMappingValid() :
                m_Group.currentCmd.axisCount == m_Group.axisCount);

        for (int slot = 0;
            currentGroupMappingValid && slot < safeGroupAxisCount;
            ++slot)
        {
            const int axisIndex = m_Group.axisIndices[slot];
            if (axisIndex < 0 ||
                axisIndex >= static_cast<int>(m_pContexts->size()) ||
                axisIndex >= MAX_AXES ||
                (!m_pathHold.unionActive && m_Group.currentCmd.axisIndices[slot] != axisIndex) ||
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

    if (m_safetyControlledStopInProgress &&
        !IsSafetyControlledStopAuthorized(m_Group.axisIndices[0]))
    {
        EmergencyStopAllAxesImpl(false);
        return;
    }




    // ======================================================
    // 🌟 [司機 A] 跳刀狀態機接管 (優先權最高)
    // ======================================================
    if (m_safetyControlledStopInProgress)
    {
        // Dedicated RT safety-stop driver: never dequeue, cross History, or
        // consult PATH_SERVO/Jump velocity sources while the stale command is
        // being retired. StopMove selected either the original trapezoidal
        // deceleration target or STOPPING velocity ramp.
        if (vAxis.state == MotionState::MotionState_STOPPING)
        {
            Calc_Trajectory_Velocity(vAxis, vCmd);
        }
        else
        {
            Calc_Trajectory_Trapezoidal(vAxis, vCmd);
        }
    }
    else if (jm.state != JumpState::IDLE)
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
    else if (ProcessPathCoreHoldExcursion(vCmd))
    {
        // CB owns only the scalar planner; original source geometry and identity remain live.
        if (!m_Group.isActive || HasPendingSafetyOrRecoveryRequests() ||
            GetCommandAuthorizationFailure(m_Group.currentCmd) != MotionRejectReason::NONE) return;
    }
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
        bool cncBoundaryFrameReady = false;
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
                MotionCommand boundaryNext{};
                const bool carriedFrame = m_Group.isActive && m_Group.currentCmd.cncFeedLookahead &&
                    vAxis.targetEndVel > 0.1 && TryPeekNextMotionCommand(boundaryNext) &&
                    boundaryNext.cncFeedLookahead &&
                    (m_Group.currentCmd.pathCorePlanarCircle || boundaryNext.pathCorePlanarCircle || m_Group.currentCmd.cncCornerBlend || boundaryNext.cncCornerBlend);
                if (carriedFrame)
                {
                    const MotionExecutionIdentity outgoing = m_Group.currentCmd.execution;
                    const double outputVelocity = vAxis.logicalCmdVel;
                    LoadNextCommand(true);
                    if (!m_Group.isActive || MotionExecutionIdentityExactlyMatches(outgoing, m_Group.currentCmd.execution)) return;
                    cncBoundaryFrameReady = true;
                    vCmd.instantCmdPos = vAxis.currentCmdPos;
                    vCmd.instantCmdVel = outputVelocity;
                }
                else LoadNextCommand();
            }
        }

        if (!m_Group.isActive) return;
        if (!cncBoundaryFrameReady)
        {
            vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;
            RefreshCncP1LateJunction();
            if (m_Group.currentCmd.cncFeedLookahead)
                vAxis.cruiseVel_PPS = (std::min)(vAxis.cruiseVel_PPS, vAxis.maxVel_PPS);
            RefreshCncFeedLookahead();

            if (vAxis.state == MotionState::MotionState_STOPPING)
            {
                Calc_Trajectory_Velocity(vAxis, vCmd);
            }
            else
            {
                Calc_Trajectory_Trapezoidal(vAxis, vCmd);
            }
            if (m_Group.currentCmd.cncFeedLookahead && vAxis.inPosition &&
                vAxis.targetEndVel > 0.1 && !m_safetyControlledStopInProgress &&
                m_Group.pathMode == PathMode::CONTINUOUS && !HasPendingSafetyOrRecoveryRequests())
            {
                MotionCommand next{};
                if (TryPeekNextMotionCommand(next) && next.cncFeedLookahead &&
                    (m_Group.currentCmd.pathCorePlanarCircle || next.pathCorePlanarCircle || m_Group.currentCmd.cncCornerBlend || next.cncCornerBlend))
                {
                    const MotionExecutionIdentity outgoing = m_Group.currentCmd.execution;
                    const double outputVelocity = vCmd.instantCmdVel;
                    vAxis.logicalCmdVel = outputVelocity;
                    LoadNextCommand(true);
                    if (!m_Group.isActive || MotionExecutionIdentityExactlyMatches(outgoing, m_Group.currentCmd.execution))
                    {
                        // The existing lifecycle/authorization boundary may deny this
                        // handoff (including its existing stale rejection). Do not
                        // publish an overshoot or integrate another scalar sample.
                        return;
                    }
                    // The filter was advanced exactly once this cycle. Geometry
                    // now distributes only the carried residual on the new path.
                    vCmd.instantCmdPos = vAxis.currentCmdPos;
                    vCmd.instantCmdVel = outputVelocity;
                }
            }
        }
    }



    // =========================================================
    // 🔴 獨立的時光機 (向後跨節)：必須放在司機分流的外面！
    // =========================================================
    if (m_pathHold.unionActive && (!std::isfinite(vAxis.currentCmdPos) || vAxis.currentCmdPos < 0.0))
    {
        TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        return;
    }
    if (!m_safetyControlledStopInProgress &&
        vAxis.currentCmdPos < 0.0)
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

                InvalidateCncLineEndpointProof();
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
    if (!m_safetyControlledStopInProgress &&
        jm.state != JumpState::IDLE)
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
    if (m_pathHold.unionActive)
    {
        if (!MapPathCoreHoldExcursionGeometry(vCmd))
        {
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
            return;
        }
    }
    else if (m_Group.currentCmd.pathCoreRetainedTraversal)
    {
        // Reject a non-finite planner output before clamping; min/max would
        // otherwise turn NaN into a plausible endpoint parameter.
        if (!std::isfinite(vCmd.instantCmdPos) || !std::isfinite(vCmd.instantCmdVel))
        {
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
            return;
        }
        const MotionCommand& retained = m_Group.currentCmd;
        double startU = 0.0, endU = 0.0, intervalDistance = 0.0;
        if (!GetMotionRetainedInterval(retained, startU, endU, intervalDistance))
        {
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
            return;
        }
        if (vAxis.state == MotionState::MotionState_IDLE &&
            !m_safetyControlledStopInProgress)
        {
            // BZ_FIX1: the filtered planner can finish a few ULPs short,
            // then canonicalize its finalTargetPos to currentCmdPos. Close
            // only normal retained completion to the immutable path length
            // before distributing the exact saved endpoint. A controlled
            // stop must retain its partial position.
            if (!m_Group.isActive || HasPendingSafetyOrRecoveryRequests() ||
                GetCommandAuthorizationFailure(retained) != MotionRejectReason::NONE)
            {
                return;
            }
            LifecycleCommitReservationGuard endpointCommit(*this, retained.execution);
            if (!endpointCommit.IsAcquired() || HasPendingSafetyOrRecoveryRequests() ||
                GetCommandAuthorizationFailure(retained) != MotionRejectReason::NONE)
            {
                return;
            }
            const double distance = intervalDistance;
            const double error = std::abs(distance - vCmd.instantCmdPos);
            // CA partial spans accumulate independent filtered integration
            // residue. The finite host matrix reached 100.95 epsilon * span.
            // Keep BZ full-range 64 unchanged; partial intervals allow 128,
            // still bounded by the physical cap and every lifecycle guard.
            const double roundingFactor = std::abs(endU - startU) == 1.0 ? 64.0 : 128.0;
            bool endpointValid = m_Group.pathMode == PathMode::EXACT_STOP &&
                vAxis.isVirtualAxis && vAxis.inPosition && !vAxis.isFault && !vAxis.isLagAlarm &&
                std::isfinite(distance) && distance > 0.0 && std::isfinite(error) &&
                error <= roundingFactor * std::numeric_limits<double>::epsilon() * distance &&
                vAxis.currentCmdPos == vCmd.instantCmdPos &&
                vAxis.planningPos == vAxis.currentCmdPos &&
                vAxis.finalTargetPos == vAxis.currentCmdPos &&
                vAxis.currentCmdVel == 0.0 && vAxis.logicalCmdVel == 0.0 &&
                vAxis.targetVelocity == 0.0 && vAxis.targetEndVel == 0.0 &&
                vCmd.instantCmdVel == 0.0;
            for (int slot = 0; endpointValid && slot < m_Group.axisCount; ++slot)
            {
                const AxisContext& realAxis = (*m_pContexts)[m_Group.axisIndices[slot]];
                const double pulsePerMM = realAxis.resolution_PPR / realAxis.finalLead;
                const double errorMM = error / pulsePerMM;
                endpointValid = std::isfinite(realAxis.resolution_PPR) && realAxis.resolution_PPR > 0.0 &&
                    std::isfinite(realAxis.finalLead) && realAxis.finalLead > 0.0 &&
                    std::isfinite(pulsePerMM) && pulsePerMM > 0.0 &&
                    std::isfinite(errorMM) && errorMM <= 5e-8;
            }
            if (!endpointValid)
            {
                endpointCommit.Release();
                TriggerGroupMappingIntegrityEmergencyStop(-1, true);
                return;
            }
            vAxis.currentCmdPos = distance;
            vAxis.planningPos = distance;
            vAxis.finalTargetPos = distance;
            vCmd.instantCmdPos = distance;
        }
        const double progress = intervalDistance <= 0.0 ? 1.0 :
            (std::max)(0.0, (std::min)(1.0, vCmd.instantCmdPos / intervalDistance));
        const double u = MotionRetainedParameterAtProgress(startU, endU, progress);
        // Normalize the geometric derivative before applying velocity; v/L
        // can overflow for a valid extremely short retained line.
        const double traversalVelocity = vCmd.instantCmdVel * (retained.pathCoreRetainedReverse ? -1.0 : 1.0);
        const double angle = retained.mem_startAngle + retained.mem_totalAngle * u;
        const double cosine = retained.pathCorePlanarCircle ? std::cos(angle) : 0.0;
        const double sine = retained.pathCorePlanarCircle ? std::sin(angle) : 0.0;
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
        {
            const int axis = m_Group.axisIndices[slot];
            AxisContext& realAxis = (*m_pContexts)[axis];
            const double delta = retained.mem_ratio[axis] - retained.mem_startPos[axis];
            double evaluatedPulse = 0.0;
            if (!EvaluateMotionRetainedPulseCanonical(retained, static_cast<std::size_t>(axis), u, evaluatedPulse))
            {
                TriggerGroupMappingIntegrityEmergencyStop(-1, true);
                return;
            }
            realAxis.logicalCmdPos = evaluatedPulse;
            const double derivative = !retained.pathCorePlanarCircle ? delta : axis == 0 ?
                -retained.mem_radius * sine * retained.mem_totalAngle : retained.mem_radius * cosine * retained.mem_totalAngle;
            realAxis.logicalCmdVel = retained.mem_totalDist <= 0.0 ? 0.0 :
                (derivative / retained.mem_totalDist) * traversalVelocity;
        }
    }
    else if (m_Group.currentCmd.cncCornerBlend)
    {
        const MotionCommand& c = m_Group.currentCmd;
        const double prefix = std::hypot(c.mem_ratio[0] - c.mem_startPos[0], c.mem_ratio[1] - c.mem_startPos[1]);
        const double distance = (std::max)(0.0, (std::min)(c.mem_totalDist, vCmd.instantCmdPos));
        const bool inArc = distance >= prefix;
        const double angle = c.mem_startAngle + (inArc ? (distance - prefix) / c.mem_radius * double(c.dir) : 0.0);
        for (unsigned i = 0U; i < 2U; ++i)
        {
            AxisContext& axis = (*m_pContexts)[i];
            double position = 0.0, tangent = 0.0;
            if (!inArc) { tangent = (c.mem_ratio[i] - c.mem_startPos[i]) / prefix; position = c.mem_startPos[i] + distance * tangent; }
            else {
                position = c.centerPos[i] + c.mem_radius * (i == 0U ? std::cos(angle) : std::sin(angle));
                tangent = i == 0U ? -double(c.dir) * std::sin(angle) : double(c.dir) * std::cos(angle);
            }
            if (distance == 0.0) position = c.mem_startPos[i];
            if (distance == c.mem_totalDist) position = c.targetPos[i];
            if (!std::isfinite(position) || !std::isfinite(tangent) || !std::isfinite(vCmd.instantCmdVel))
            {
                TriggerGroupMappingIntegrityEmergencyStop(-1, true); return;
            }
            axis.logicalCmdPos = position; axis.logicalCmdVel = tangent * vCmd.instantCmdVel;
        }
        auto& prefixState = m_cncFeedLookahead;
        if (prefixState.prefixEnabled && prefixState.loaded &&
            MotionExecutionIdentityExactlyMatches(prefixState.identity, c.execution) &&
            prefixState.lease.Matches(c.ownerLease) && !m_safetyControlledStopInProgress)
        {
            // Observe actual filtered output, not just a requested ceiling.
            vAxis.logicalCmdVel = vCmd.instantCmdVel;
            if (!inArc && !prefixState.prefixFastSeen &&
                vAxis.currentCmdVel > vAxis.maxVel_PPS * 1.01 && vCmd.instantCmdVel > vAxis.maxVel_PPS * 1.01)
            {
                prefixState.prefixFastSeen = true;
                QueueCncFeedPlanDiagnostic(CncFeedPlanEvent::PREFIX_FAST);
            }
            if (!inArc && !prefixState.prefixAuthoredSeen &&
                prefixState.prefixLimit > c.targetVel * 1.01 &&
                vAxis.currentCmdVel > c.targetVel * 1.01 && vCmd.instantCmdVel > c.targetVel * 1.01)
            {
                prefixState.prefixAuthoredSeen = true;
                QueueCncFeedPlanDiagnostic(CncFeedPlanEvent::PREFIX_AUTHORED);
            }
            if (!inArc && prefixState.prefixFastSeen && !prefixState.prefixBrakeSeen &&
                m_Group.feedrateOverride >= 1.0 &&
                vAxis.currentCmdVel < prefixState.prefixPreviousRaw - vAxis.dec_PPS2 * CYCLE_TIME_SEC * 0.1)
            {
                prefixState.prefixBrakeSeen = true;
                QueueCncFeedPlanDiagnostic(CncFeedPlanEvent::PREFIX_BRAKE);
            }
            prefixState.prefixPreviousRaw = vAxis.currentCmdVel;
        }
        if (inArc && !m_cncFeedLookahead.blendEntered)
        {
            m_cncFeedLookahead.blendEntered = true;
            vAxis.logicalCmdVel = vCmd.instantCmdVel;
            QueueCncFeedPlanDiagnostic(CncFeedPlanEvent::BLEND_ENTER);
        }
    }
    else if (m_Group.mode == InterpolationMode::LINEAR)
    {
        if (IsFixedPlanarLineEndpointScope() && vAxis.state == MotionState::MotionState_IDLE)
        {
            if (!TryCompleteFixedPlanarLineEndpoint(vCmd)) return;
        }
        else
        {
            for (int i = 0; i < m_Group.axisCount; ++i) {
                int idx = m_Group.axisIndices[i];
                AxisContext& realAxis = (*m_pContexts)[idx];
                realAxis.logicalCmdPos = m_Group.startPos[i] + (vCmd.instantCmdPos * m_Group.ratio[i]);
                realAxis.logicalCmdVel = vCmd.instantCmdVel * m_Group.ratio[i];
            }
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
        // EC: PID1073 left an ordinary full circle a few scalar ULPs
        // short after IDLE canonicalization. Complete only the drained,
        // authorized normal source to its loaded immutable path length.
        // The existing progress>=1 branch then emits its exact target bits.
        const MotionCommand& ecCommand = m_Group.currentCmd;
        const bool ecArcTerminal = vAxis.state == MotionState::MotionState_IDLE &&
            m_Group.isActive && ecCommand.pathCorePlanarCircle && !ecCommand.pathCoreFeedExactStop &&
            (ecCommand.mode == InterpolationMode::CIRCULAR_CW ||
                ecCommand.mode == InterpolationMode::CIRCULAR_CCW) &&
            m_Group.mode == ecCommand.mode &&
            IsMotionArcPlaneGroupMapping(ecCommand, m_Group.axisCount, m_Group.axisIndices) &&
            ((ecCommand.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
                m_Group.pathMode == PathMode::EXACT_STOP && !ecCommand.cncFeedLookahead) ||
                IsCncArcEndpointScope()) && !ecCommand.cncCornerBlend &&
            !ecCommand.pathCoreRetainedTraversal && !ecCommand.pathCoreRetainedReverse &&
            !ecCommand.replayTerminalAlreadyPublished &&
            ecCommand.execution.IsAssigned() && ecCommand.execution.source == MotionCommandSource::NC_MEMORY &&
            ecCommand.ownerLease.IsValid() && ecCommand.ownerLease.owner == MotionOwner::AUTO &&
            !m_Group.enableHistory && !m_Group.enableTransform &&
            m_Group.jumpManager.state == JumpState::IDLE && !m_pathHold.sourceSeen &&
            !m_safetyControlledStopInProgress && !IsPathCoreHoldExcursionDriving();
        if (ecArcTerminal)
        {
            if (HasPendingSafetyOrRecoveryRequests() ||
                GetCommandAuthorizationFailure(ecCommand) != MotionRejectReason::NONE)
            {
                return;
            }
            LifecycleCommitReservationGuard endpointCommit(*this, ecCommand.execution);
            if (!endpointCommit.IsAcquired() || HasPendingSafetyOrRecoveryRequests() ||
                GetCommandAuthorizationFailure(ecCommand) != MotionRejectReason::NONE)
            {
                return;
            }
            const double distance = m_Group.totalDist3D;
            const double error = std::abs(distance - vCmd.instantCmdPos);
            bool endpointValid = vAxis.isVirtualAxis && vAxis.inPosition &&
                !vAxis.isFault && !vAxis.isLagAlarm &&
                std::isfinite(distance) && distance > 1e-12 && std::isfinite(error) &&
                error <= 1e-12 * distance &&
                std::isfinite(startRadius) && startRadius > 0.0 && startRadius == endRadius &&
                startRadius == m_Group.radius && std::isfinite(m_Group.startAngle) &&
                std::isfinite(totalAngle) && totalAngle != 0.0 &&
                distance == startRadius * std::abs(totalAngle) &&
                m_Group.centerX == ecCommand.centerPos[0] && m_Group.centerY == ecCommand.centerPos[1] &&
                std::isfinite(ecCommand.targetPos[0]) && std::isfinite(ecCommand.targetPos[1]) &&
                std::isfinite(ecCommand.centerPos[0]) && std::isfinite(ecCommand.centerPos[1]) &&
                vAxis.currentCmdPos == vCmd.instantCmdPos &&
                vAxis.planningPos == vAxis.currentCmdPos && vAxis.finalTargetPos == vAxis.currentCmdPos &&
                vAxis.currentCmdVel == 0.0 && vAxis.logicalCmdVel == 0.0 &&
                vAxis.targetVelocity == 0.0 && vAxis.targetEndVel == 0.0 && vCmd.instantCmdVel == 0.0 &&
                vAxis.bufferSum == 0.0 &&
                std::all_of(vAxis.velBuffer.begin(), vAxis.velBuffer.end(),
                    [](double velocity) { return velocity == 0.0; });
            for (int slot = 0; endpointValid && slot < 2; ++slot)
            {
                const AxisContext& realAxis = (*m_pContexts)[m_Group.axisIndices[slot]];
                const double pulsePerMM = realAxis.resolution_PPR / realAxis.finalLead;
                const double errorMM = error / pulsePerMM;
                endpointValid = std::isfinite(realAxis.resolution_PPR) && realAxis.resolution_PPR > 0.0 &&
                    std::isfinite(realAxis.finalLead) && realAxis.finalLead > 0.0 &&
                    std::isfinite(pulsePerMM) && pulsePerMM > 0.0 &&
                    std::isfinite(errorMM) && errorMM <= 5e-8;
            }
            if (!endpointValid)
            {
                endpointCommit.Release();
                TriggerGroupMappingIntegrityEmergencyStop(-1, true);
                return;
            }
            vAxis.currentCmdPos = distance;
            vAxis.planningPos = distance;
            vAxis.finalTargetPos = distance;
            vCmd.instantCmdPos = distance;
            if (ecCommand.cncFeedLookahead)
            {
                // Emit both exact packet endpoints under the same reservation
                // before recording the RT-local candidate. Only successful
                // tracked COMPLETED feedback publishes its NC-visible marker.
                realX.logicalCmdPos = ecCommand.targetPos[0];
                realY.logicalCmdPos = ecCommand.targetPos[1];
                realX.logicalCmdVel = 0.0;
                realY.logicalCmdVel = 0.0;
                m_cncLineEndpointCandidate = ecCommand.execution;
            }
        }
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
        if (m_Group.currentCmd.pathCorePlanarCircle && progressRatio <= 0.0)
        {
            realX.logicalCmdPos = m_Group.startPos[0];
            realY.logicalCmdPos = m_Group.startPos[1];
        }
        else if (m_Group.currentCmd.pathCorePlanarCircle && progressRatio >= 1.0)
        {
            realX.logicalCmdPos = m_Group.currentCmd.targetPos[0];
            realY.logicalCmdPos = m_Group.currentCmd.targetPos[1];
        }
        else
        {
            realX.logicalCmdPos = m_Group.centerX + currentRadius * cosAngle;
            realY.logicalCmdPos = m_Group.centerY + currentRadius * sinAngle;
        }


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
        // BY fixed XY circle already has its exact finite dS/du. Avoid
        // squaring a representable pulse radius into infinity.
        const double pathMetric = m_Group.currentCmd.pathCorePlanarCircle
            ? m_Group.totalDist3D
            : std::sqrt(deltaRadius * deltaRadius + currentRadius * currentRadius * totalAngle * totalAngle + deltaZ * deltaZ);


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
    if (vAxis.state == MotionState::MotionState_IDLE &&
        (!IsPathCoreHoldExcursionDriving() || m_safetyControlledStopInProgress))
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
            if (m_safetyControlledStopInProgress)
            {
                // The prior segment was already terminalized ABORTED when
                // the SAFETY Epoch was applied. Retire its RT geometry here
                // without Complete(), History insertion, or an unauthorized
                // lifecycle reservation.
                if (!IsSafetyControlledStopAuthorized(
                    m_Group.axisIndices[0]) ||
                    !TryCanonicalizeIdleAxisCommandState(vAxis))
                {
                    EmergencyStopAllAxesImpl(false);
                    return;
                }

                for (int i = 0; i < m_Group.axisCount; ++i)
                {
                    const int idx = m_Group.axisIndices[i];
                    if (!TryCanonicalizeInactivePhysicalAxisCommandState(
                        (*m_pContexts)[idx]))
                    {
                        TriggerGroupMappingIntegrityEmergencyStop(idx, true);
                        return;
                    }
                }

                m_Group.isActive = false;
                InvalidateCncLineEndpointProof();
                m_Group.currentCmd.execution = MotionExecutionIdentity{};
                m_Group.currentCmd.ownerLease = MotionOwnerLease{};
                m_safetyControlledStopInProgress = false;
                m_safetyControlledStopOwnerLease = MotionOwnerLease{};
                m_safetyControlledStopEpoch =
                    MOTION_EXECUTION_EPOCH_INVALID;
                m_safetyControlledStopRequestTicket = 0U;
                TryAcknowledgeAppliedSafetyMotionRequests();
                // Release the NC Reset continuation only after every
                // physical group axis has met the existing in-position
                // proof.  A higher-priority action may already have changed
                // the phase to SUPERSEDED; the CAS helper deliberately does
                // not overwrite that terminal result.
                CompleteResetControlledStop();
                return;
            }

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

            FinishCncP1Diagnostic();
            FinishCncFeedLookahead();
            CompleteTrackedMotionCommand(
                m_Group.currentCmd);

            if (m_Group.enableHistory && !m_Group.currentCmd.pathCoreRetainedTraversal &&
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
    // Invalidate the identity tag first.  Pulses and the valid mask are only
    // producer-owned staging data until a coherent Epoch/Owner sample is
    // published at the end of this function.
    m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
    m_g00ProducerQueueTailValidMask = 0U;
    m_g00ProducerQueueTailPulse.fill(0.0);

    if (m_pContexts == nullptr || m_pContexts->empty())
    {
        return;
    }

    // Sample each packed identity word exactly once at entry.  Reading the
    // Epoch and PENDING bit from the same atomic word closes the otherwise
    // possible split read where an Epoch publisher starts between them.
    const std::uint64_t entryExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if ((entryExecutionPublication &
        EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL ||
        (entryExecutionPublication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
    {
        return;
    }

    const MotionExecutionEpoch entryEpoch =
        UnpackExecutionEpochPublication(entryExecutionPublication);
    const MotionOwnerLease entryOwnerLease =
        UnpackMotionOwnerState(entryOwnerState);
    if (entryEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !entryOwnerLease.IsValid())
    {
        return;
    }

    std::array<double, MAX_AXES> stagedTailPulse{};
    std::uint32_t validMask = 0U;
    for (std::size_t axisSlot = 0U;
        axisSlot < m_pContexts->size();
        ++axisSlot)
    {
        AxisContext& axis = (*m_pContexts)[axisSlot];
        if (!axis.isExist)
        {
            continue;
        }

        // An enabled axis that cannot be represented in the fixed producer
        // sidecar makes the entire baseline unusable.
        if (axisSlot >= static_cast<std::size_t>(MAX_AXES))
        {
            return;
        }

        // logicalCmdPos is written by the RT owner.  At this drained lifecycle
        // seam it is stationary, but it must still be sampled atomically so
        // the C++ memory model never observes a torn/data-racing double.
        const double referencePulse = axis.logicalCmdPos.Load();
        if (!std::isfinite(referencePulse))
        {
            return;
        }

        stagedTailPulse[axisSlot] = referencePulse;
        validMask |=
            (1U << static_cast<unsigned>(axisSlot));
    }

    // Double-sample the exact packed Epoch and Owner words.  Any publication,
    // pending transition, active RT terminal-commit reservation, or
    // owner-generation transfer during the axis snapshot rejects the whole
    // baseline without waiting on RT.
    const std::uint64_t exitExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t exitOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if ((exitExecutionPublication &
        EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL ||
        (exitExecutionPublication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL ||
        exitExecutionPublication != entryExecutionPublication ||
        exitOwnerState != entryOwnerState)
    {
        return;
    }

    // Commit only the coherent staged sample.  The legacy per-axis atomic
    // mirror and all sidecar values precede the identity tag publication.
    for (std::size_t axisSlot = 0U;
        axisSlot < m_pContexts->size();
        ++axisSlot)
    {
        AxisContext& axis = (*m_pContexts)[axisSlot];
        if (axis.isExist)
        {
            axis.lastQueuedPulse.Store(stagedTailPulse[axisSlot]);
        }
    }
    m_g00ProducerQueueTailPulse = stagedTailPulse;
    m_g00ProducerQueueTailValidMask = validMask;
    m_g00ProducerQueueTailOwnerLease = entryOwnerLease;
    m_g00ProducerQueueTailEpoch = entryEpoch;
}


bool MotionCore::TryGetSynchronizedG00QueueTailMCS(
    double(&outputMCS)[MAX_AXES]) const noexcept
{
    std::fill_n(outputMCS, MAX_AXES, 0.0);
    std::array<double, MAX_AXES> convertedMCS{};

    if (m_pContexts == nullptr || m_pContexts->empty())
    {
        return false;
    }

    const std::uint64_t entryExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if ((entryExecutionPublication &
        EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL ||
        (entryExecutionPublication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL)
    {
        return false;
    }

    const MotionExecutionEpoch entryEpoch =
        UnpackExecutionEpochPublication(entryExecutionPublication);
    const MotionOwnerLease entryOwnerLease =
        UnpackMotionOwnerState(entryOwnerState);
    if (entryEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !entryOwnerLease.IsValid() ||
        m_g00ProducerQueueTailEpoch != entryEpoch ||
        !m_g00ProducerQueueTailOwnerLease.IsValid() ||
        !m_g00ProducerQueueTailOwnerLease.Matches(entryOwnerLease))
    {
        return false;
    }

    const std::array<double, MAX_AXES> stagedTailPulse =
        m_g00ProducerQueueTailPulse;
    const std::uint32_t stagedValidMask =
        m_g00ProducerQueueTailValidMask;
    const MotionExecutionEpoch stagedTailEpoch =
        m_g00ProducerQueueTailEpoch;
    const MotionOwnerLease stagedTailOwnerLease =
        m_g00ProducerQueueTailOwnerLease;

    for (const double tailPulse : stagedTailPulse)
    {
        if (!std::isfinite(tailPulse))
        {
            return false;
        }
    }

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
            (1U << static_cast<unsigned>(axisSlot));
        const double tailPulse =
            stagedTailPulse[axisSlot];
        if ((stagedValidMask & axisBit) == 0U ||
            !std::isfinite(axis.resolution_PPR) ||
            !std::isfinite(axis.finalLead) ||
            axis.resolution_PPR == 0.0)
        {
            return false;
        }

        double mcsUnit =
            tailPulse * axis.finalLead / axis.resolution_PPR;
        if (!std::isfinite(mcsUnit))
        {
            return false;
        }

        // Match Reset rebase semantics: modulo-normalize only a configured
        // finite rotary range.  Continuous rotary axes remain unwrapped.
        if (axis.axisType == AxisType::ROTARY &&
            std::isfinite(axis.rotaryModulo) &&
            axis.rotaryModulo > 0.0)
        {
            mcsUnit = std::fmod(mcsUnit, axis.rotaryModulo);
            if (mcsUnit < 0.0)
            {
                mcsUnit += axis.rotaryModulo;
            }
        }
        if (!std::isfinite(mcsUnit))
        {
            return false;
        }

        convertedMCS[axisSlot] = mcsUnit;
    }

    for (std::size_t axisSlot = axisCount;
        axisSlot < m_pContexts->size();
        ++axisSlot)
    {
        if ((*m_pContexts)[axisSlot].isExist)
        {
            return false;
        }
    }

    // Close the conversion seam against a concurrent Epoch publication or
    // owner transfer.  Recheck both packed words and the producer tag; copy
    // output only when the entry and exit observations are identical and no
    // execution transition is pending.
    const std::uint64_t exitExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t exitOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    if ((exitExecutionPublication &
        EXECUTION_EPOCH_PUBLICATION_PENDING) != 0ULL ||
        (exitExecutionPublication &
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) != 0ULL ||
        exitExecutionPublication != entryExecutionPublication ||
        exitOwnerState != entryOwnerState ||
        m_g00ProducerQueueTailEpoch != stagedTailEpoch ||
        !m_g00ProducerQueueTailOwnerLease.Matches(
            stagedTailOwnerLease) ||
        stagedTailEpoch != entryEpoch ||
        !stagedTailOwnerLease.Matches(entryOwnerLease) ||
        m_g00ProducerQueueTailValidMask != stagedValidMask)
    {
        return false;
    }

    std::copy(
        convertedMCS.begin(),
        convertedMCS.end(),
        outputMCS);
    return true;
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
    // A claimed Reset release token freezes both the ACK and its evidence
    // image until the NC-side owner CAS has completed. With no claim, make
    // every older token non-consumable before evaluating this fresh sample;
    // ACKNOWLEDGED republishes a new one at the end of the same RT pass.
    if (!TryInvalidateNCResetSafetyReleaseAuthorization())
    {
        payload.ncSettleSnapshots = m_ncSettlePublishedSnapshots;
        payload.ncSettleCounters = m_ncSettleProducerCounters;
        payload.resetRebaseAck = m_ncResetRebaseAckProducer;
        return;
    }

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
            const bool pureFeedHoldScope =
                profile == MotionNCSettleProfile::FEED_HOLD_GROUP &&
                (!tracker.executionIdentity.IsAssigned() ||
                    tracker.executionIdentity.epoch != tracker.executionEpoch);
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
                // A pure logic / Macro program can legally reach M30 without
                // ever dispatching a Motion Segment.  In that case
                // m_Group.currentCmd.execution remains unassigned even though
                // the exact Program owner/epoch is current and every Motion
                // queue is drained.  Leaving scopeMask at zero makes the
                // formal GROUP_COMPLETION proof permanently SCOPE_EMPTY, so
                // M30 can never enter P_END.
                //
                // Give only this no-segment, fully drained execution a
                // machine-wide scope anchored by the current non-NONE owner
                // lease and execution epoch.  If a Motion Segment is later
                // dispatched, currentGroupIdentityCorrelated becomes true;
                // identityChanged resets this candidate and the normal exact
                // Segment/Epoch/Owner proof takes over.  Motion programs keep
                // the existing safety contract unchanged.
                const bool pureProgramExecutionScope =
                    !currentGroupIdentityCorrelated &&
                    currentOwnerLease.IsValid() &&
                    currentOwnerLease.owner != MotionOwner::NONE &&
                    currentExecutionEpoch !=
                    MOTION_EXECUTION_EPOCH_INVALID &&
                    groupDrained;

                const std::uint32_t scopeMask =
                    (currentGroupIdentityCorrelated ||
                        pureProgramExecutionScope)
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

            // -------------------------------------------------------------
            // NC-0.2K.7.5.1 - Feed Hold read-ahead member transition rebind
            //
            // A PROGRAM Feed Hold request is scoped to the active Motion
            // group, not to only the single segment which happened to be at
            // m_Group.currentCmd when the 250 us consumer accepted it.  With
            // ordinary G00 read-ahead, that member can finish during the
            // controlled deceleration and the already admitted next member
            // can become current before the 200-cycle settle proof rises.
            //
            // The previous exact-identity check then held the request at
            // EXECUTION_EPOCH_MISMATCH forever even though the request,
            // Program owner, execution epoch and zero-override hold were all
            // still current.  Cycle Start correctly remained deferred, but
            // no later cycle could ever produce the ACK which releases it.
            //
            // Rebind only this FEED_HOLD_GROUP tracker, and only while every
            // authority condition which created the request is still exact:
            //
            //   - same accepted request sequence;
            //   - same execution epoch and owner lease;
            //   - current group identity is assigned to that epoch;
            //   - group is still active under zero Feed Override;
            //   - virtual command velocity is already stopped;
            //   - no Safety / recovery request is pending.
            //
            // Rebinding revokes any partial/old proof and restarts the full
            // dwell against the new member and scope.  It does not set
            // settled, acknowledge Feed Hold, apply Resume, write Motion, or
            // weaken any following-error / excursion / fault predicate below.
            // -------------------------------------------------------------
            if (profile == MotionNCSettleProfile::FEED_HOLD_GROUP &&
                tracker.requestSequence !=
                MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
                tracker.requestAccepted &&
                tracker.executionEpoch == currentExecutionEpoch &&
                tracker.ownerLease.Matches(currentOwnerLease) &&
                tracker.executionIdentity.IsAssigned() &&
                tracker.executionIdentity.epoch == tracker.executionEpoch &&
                currentGroupIdentityCorrelated &&
                !MotionExecutionIdentityExactlyMatches(
                    tracker.executionIdentity,
                    currentGroupIdentity) &&
                m_Group.isActive &&
                snapshot.overrideZero &&
                snapshot.virtualCommandStopped &&
                correlatedCurrentGroupMask != 0U &&
                !snapshot.safetyOrRecoveryPending)
            {
                const bool scopeChanged =
                    tracker.scopeMask != correlatedCurrentGroupMask;
                ResetNCSettleCandidate(
                    profile,
                    MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH,
                    true,
                    scopeChanged);
                tracker.executionIdentity = currentGroupIdentity;
                tracker.scopeMask = correlatedCurrentGroupMask;
                tracker.requestAccepted = true;
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
                    currentOwnerLease) &&
                IsExactResetNCSettleAuthorityCurrent(
                    m_activeResetNCSettleRequest);
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
                    // A no-segment request never adopts a later segment or
                    // a different machine mask. Its original tuple stays fixed.
                    if (pureFeedHoldScope)
                    {
                        const bool pureScopeCurrent =
                            !currentGroupIdentityCorrelated && groupDrained &&
                            m_axisCommandChannel.command_size() == 0U &&
                            (currentOwnerLease.owner == MotionOwner::AUTO ||
                                currentOwnerLease.owner == MotionOwner::MDI ||
                                currentOwnerLease.owner == MotionOwner::MANUAL_AUTO) &&
                            m_Group.pathMode != PathMode::PATH_SERVO &&
                            m_Group.pathMode != PathMode::JUMP_TRACKING &&
                            m_Group.jumpManager.state == JumpState::IDLE &&
                            tracker.scopeMask != 0U &&
                            tracker.scopeMask == existingAxisMask &&
                            MotionExecutionIdentityExactlyMatches(
                                tracker.executionIdentity,
                                currentGroupIdentity);
                        if (!pureScopeCurrent &&
                            blocker == MotionNCSettleBlocker::NONE)
                        {
                            blocker = MotionNCSettleBlocker::SCOPE_CHANGED;
                        }
                    }
                    else
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
                            !pureFeedHoldScope &&
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

    if (m_ncResetRebasePhase ==
        MotionNCResetRebasePhase::ACKNOWLEDGED)
    {
        // Retry the bounded seqlock publication on each fresh RT sample.  A
        // transient owner/frame reservation can delay authorization, but an
        // older request sequence can never authorize this Reset's release.
        (void)TryPublishNCResetSafetyReleaseAuthorization();
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
    const std::uint64_t drainRevocationGenerationAtEntry =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t drainRevocationPublishersAtEntry =
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire);

    MotionStopSettlePublicationPayload payload{};
    MotionStopSettleSnapshot& snapshot = payload.snapshot;
    MotionProgramStartReadinessSnapshot programStartReadiness{};
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
        (m_emergencyStopRequestPublication.load(
            std::memory_order_acquire) &
            EMERGENCY_STOP_REQUEST_PENDING) != 0ULL;
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
            ++programStartReadiness.existingAxisCount;

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

            // This is deliberately the exact same predicate used by
            // LoadNextCommand() when an incoming command claims a physical
            // axis. A fresh program is held at the NC boundary if any
            // existing axis would otherwise trip the AL3021 containment path
            // on its first command. Do not substitute raw actual velocity or
            // final PDO TargetVelocity here: the IDLE servo loop can report
            // noisy derivative feedback and legitimate small holding output
            // while the axis is already safe to admit.
            if (IsIncomingPhysicalAxisReadyForGroup(axis))
            {
                ++programStartReadiness.readyAxisCount;
            }
            else
            {
                ++programStartReadiness.notReadyAxisCount;
                if (programStartReadiness.firstNotReadyAxisIndex < 0)
                {
                    programStartReadiness.firstNotReadyAxisIndex =
                        publishedAxisIndex;
                }
            }

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
    UpdatePathCoreHoldExcursionEvidence();

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

    // Publish the compact per-axis readiness proof before the paired stop
    // bank becomes active. Readers require this exact generation/sample pair;
    // a writer that races between either read produces a mismatch and Start
    // simply remains pending for the next NC scan.
    programStartReadiness.stopPublicationGeneration =
        snapshot.publicationGeneration;
    programStartReadiness.stopSampleSequence = snapshot.sampleSequence;
    std::array<
        std::uint64_t,
        MOTION_PROGRAM_START_READINESS_PUBLICATION_WORD_COUNT>
        programStartReadinessWords{};
    std::memcpy(
        programStartReadinessWords.data(),
        &programStartReadiness,
        sizeof(programStartReadiness));

    MotionProgramStartReadinessAtomicBank& programStartReadinessBank =
        m_programStartReadinessPublicationBanks[
            static_cast<std::size_t>(
                snapshot.publicationGeneration & 1ULL)];
    programStartReadinessBank.writeSequence.fetch_add(
        1ULL,
        std::memory_order_acq_rel);
    for (std::size_t i = 0U;
        i < programStartReadinessWords.size();
        ++i)
    {
        programStartReadinessBank.words[i].store(
            programStartReadinessWords[i],
            std::memory_order_relaxed);
    }
    std::atomic_thread_fence(std::memory_order_release);
    programStartReadinessBank.writeSequence.fetch_add(
        1ULL,
        std::memory_order_release);
    m_programStartReadinessPublicationGeneration.store(
        snapshot.publicationGeneration,
        std::memory_order_release);

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

    const std::uint64_t drainRevocationGenerationAtExit =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t drainRevocationPublishersAtExit =
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire);
    if (drainRevocationPublishersAtEntry == 0U &&
        drainRevocationPublishersAtExit == 0U &&
        drainRevocationGenerationAtEntry ==
        drainRevocationGenerationAtExit)
    {
        // The coherent bank above is a fresh RT observation of this exact
        // request-generation. A publisher that starts after these checks
        // changes either the in-progress count or current generation, so the
        // exact-drain reader still fails closed at its exit seam.
        m_executionDrainObservedRevocationGeneration.store(
            drainRevocationGenerationAtEntry,
            std::memory_order_release);
    }
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


bool MotionCore::TryReadProgramStartReadinessPublication(
    MotionProgramStartReadinessSnapshot& readiness) const noexcept
{
    std::array<
        std::uint64_t,
        MOTION_PROGRAM_START_READINESS_PUBLICATION_WORD_COUNT> words{};

    for (std::uint32_t attempt = 0U; attempt < 16U; ++attempt)
    {
        const std::uint64_t generationBefore =
            m_programStartReadinessPublicationGeneration.load(
                std::memory_order_acquire);
        const MotionProgramStartReadinessAtomicBank& publicationBank =
            m_programStartReadinessPublicationBanks[
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
            m_programStartReadinessPublicationGeneration.load(
                std::memory_order_acquire);
        if (bankSequenceBefore == bankSequenceAfter &&
            (bankSequenceAfter & 1ULL) == 0ULL &&
            generationBefore == generationAfter)
        {
            MotionProgramStartReadinessSnapshot candidate{};
            std::memcpy(&candidate, words.data(), sizeof(candidate));
            if (candidate.stopPublicationGeneration == generationBefore)
            {
                readiness = candidate;
                return true;
            }
        }
    }

    readiness = MotionProgramStartReadinessSnapshot{};
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


MotionCommandPathModeTransportSnapshot
MotionCore::GetCommandPathModeTransportSnapshot() const noexcept
{
    MotionCommandPathModeTransportSnapshot snapshot{};

    snapshot.producerSnapshotCoherent = false;
    for (std::size_t attempt = 0U; attempt < 8U; ++attempt)
    {
        const std::uint64_t begin =
            m_pathModeProducerSequence.load(std::memory_order_acquire);
        if ((begin & 1ULL) != 0ULL)
        {
            continue;
        }

        snapshot.producerAccepted =
            m_pathModeProducerAccepted.load(std::memory_order_relaxed);
        snapshot.producerRejected =
            m_pathModeProducerRejected.load(std::memory_order_relaxed);
        snapshot.producerExactStop =
            m_pathModeProducerExactStop.load(std::memory_order_relaxed);
        snapshot.producerContinuous =
            m_pathModeProducerContinuous.load(std::memory_order_relaxed);
        snapshot.producerUnspecified =
            m_pathModeProducerUnspecified.load(std::memory_order_relaxed);
        snapshot.producerInvalid =
            m_pathModeProducerInvalid.load(std::memory_order_relaxed);
        snapshot.producerFingerprint =
            m_pathModeProducerFingerprint.load(std::memory_order_relaxed);

        const std::uint64_t end =
            m_pathModeProducerSequence.load(std::memory_order_acquire);
        if (begin == end && (end & 1ULL) == 0ULL)
        {
            snapshot.producerSnapshotCoherent = true;
            break;
        }
    }

    snapshot.consumerSnapshotCoherent = false;
    for (std::size_t attempt = 0U; attempt < 8U; ++attempt)
    {
        const std::uint64_t begin =
            m_pathModeConsumerSequence.load(std::memory_order_acquire);
        if ((begin & 1ULL) != 0ULL)
        {
            continue;
        }

        snapshot.consumerCommitted =
            m_pathModeConsumerCommitted.load(std::memory_order_relaxed);
        snapshot.consumerIngressCommitted =
            m_pathModeConsumerIngressCommitted.load(
                std::memory_order_relaxed);
        snapshot.consumerReplayCommitted =
            m_pathModeConsumerReplayCommitted.load(
                std::memory_order_relaxed);
        snapshot.consumerExactStop =
            m_pathModeConsumerExactStop.load(std::memory_order_relaxed);
        snapshot.consumerContinuous =
            m_pathModeConsumerContinuous.load(std::memory_order_relaxed);
        snapshot.consumerUnspecified =
            m_pathModeConsumerUnspecified.load(std::memory_order_relaxed);
        snapshot.consumerInvalid =
            m_pathModeConsumerInvalid.load(std::memory_order_relaxed);
        snapshot.consumerFingerprint =
            m_pathModeConsumerFingerprint.load(std::memory_order_relaxed);
        snapshot.legacyModeMatches =
            m_pathModeLegacyMatches.load(std::memory_order_relaxed);
        snapshot.legacyModeMismatches =
            m_pathModeLegacyMismatches.load(std::memory_order_relaxed);
        snapshot.driverOverrideObservations =
            m_pathModeDriverOverrideObservations.load(
                std::memory_order_relaxed);
        snapshot.authorityAttempts =
            m_pathModeAuthorityAttempts.load(std::memory_order_relaxed);
        snapshot.authorityApplied =
            m_pathModeAuthorityApplied.load(std::memory_order_relaxed);
        snapshot.authorityExactStop =
            m_pathModeAuthorityExactStop.load(std::memory_order_relaxed);
        snapshot.authorityContinuous =
            m_pathModeAuthorityContinuous.load(std::memory_order_relaxed);
        snapshot.authorityLegacyFallbacks =
            m_pathModeAuthorityLegacyFallbacks.load(
                std::memory_order_relaxed);
        snapshot.authorityReplayBypasses =
            m_pathModeAuthorityReplayBypasses.load(
                std::memory_order_relaxed);
        snapshot.authorityDriverBlocks =
            m_pathModeAuthorityDriverBlocks.load(
                std::memory_order_relaxed);
        snapshot.authorityInvalidRejects =
            m_pathModeAuthorityInvalidRejects.load(
                std::memory_order_relaxed);

        const std::uint64_t end =
            m_pathModeConsumerSequence.load(std::memory_order_acquire);
        if (begin == end && (end & 1ULL) == 0ULL)
        {
            snapshot.consumerSnapshotCoherent = true;
            break;
        }
    }

    const std::uint64_t producerClassified =
        snapshot.producerExactStop +
        snapshot.producerContinuous +
        snapshot.producerUnspecified +
        snapshot.producerInvalid;
    const std::uint64_t consumerClassified =
        snapshot.consumerExactStop +
        snapshot.consumerContinuous +
        snapshot.consumerUnspecified +
        snapshot.consumerInvalid;
    const std::uint64_t explicitConsumerClassified =
        snapshot.consumerExactStop +
        snapshot.consumerContinuous;
    const std::uint64_t legacyCompared =
        snapshot.legacyModeMatches +
        snapshot.legacyModeMismatches +
        snapshot.driverOverrideObservations;
    const std::uint64_t authorityAccounted =
        snapshot.authorityApplied +
        snapshot.authorityLegacyFallbacks +
        snapshot.authorityReplayBypasses +
        snapshot.authorityDriverBlocks +
        snapshot.authorityInvalidRejects;

    snapshot.accountingValid =
        snapshot.producerSnapshotCoherent &&
        snapshot.consumerSnapshotCoherent &&
        snapshot.producerAccepted == producerClassified &&
        snapshot.consumerCommitted == consumerClassified &&
        snapshot.consumerCommitted ==
        snapshot.consumerIngressCommitted +
        snapshot.consumerReplayCommitted &&
        explicitConsumerClassified == legacyCompared &&
        snapshot.authorityAttempts == authorityAccounted &&
        snapshot.authorityApplied ==
        snapshot.authorityExactStop +
        snapshot.authorityContinuous &&
        snapshot.consumerCommitted ==
        snapshot.authorityApplied +
        snapshot.authorityLegacyFallbacks +
        snapshot.authorityReplayBypasses;

    snapshot.commandLocalPayloadPresent =
        snapshot.producerExactStop +
        snapshot.producerContinuous != 0ULL;

    snapshot.transportReady =
        snapshot.commandLocalPayloadPresent &&
        snapshot.accountingValid &&
        snapshot.producerRejected == 0ULL &&
        snapshot.producerInvalid == 0ULL &&
        snapshot.consumerInvalid == 0ULL &&
        snapshot.producerUnspecified == 0ULL &&
        snapshot.consumerUnspecified == 0ULL &&
        snapshot.consumerReplayCommitted == 0ULL &&
        snapshot.authorityReplayBypasses == 0ULL &&
        snapshot.authorityDriverBlocks == 0ULL &&
        snapshot.authorityInvalidRejects == 0ULL &&
        snapshot.authorityLegacyFallbacks == 0ULL &&
        snapshot.driverOverrideObservations == 0ULL &&
        snapshot.legacyModeMismatches == 0ULL &&
        snapshot.authorityApplied ==
        snapshot.consumerIngressCommitted &&
        snapshot.authorityExactStop ==
        snapshot.consumerExactStop &&
        snapshot.authorityContinuous ==
        snapshot.consumerContinuous &&
        snapshot.producerAccepted ==
        snapshot.consumerIngressCommitted &&
        snapshot.producerExactStop ==
        snapshot.consumerExactStop &&
        snapshot.producerContinuous ==
        snapshot.consumerContinuous &&
        snapshot.producerFingerprint ==
        snapshot.consumerFingerprint;

    // K.6.1 is the bounded Consumer-authority cutover.  Runtime influence is
    // evidence-based: it becomes true only after at least one explicit mode
    // was committed into the planner.
    snapshot.shadowOnly = false;
    snapshot.consumerAuthority = true;
    snapshot.runtimeInfluence = snapshot.authorityApplied != 0ULL;
    snapshot.cutoverAttempted = snapshot.authorityAttempts != 0ULL;
    return snapshot;
}


MotionQueueTailTransactionSnapshot
MotionCore::GetQueueTailTransactionSnapshot() const noexcept
{
    MotionQueueTailTransactionSnapshot snapshot{};
    snapshot.snapshotCoherent = false;

    for (std::size_t attempt = 0U; attempt < 8U; ++attempt)
    {
        const std::uint64_t begin =
            m_queueTailWriteSequence.load(std::memory_order_acquire);
        if ((begin & 1ULL) != 0ULL)
        {
            continue;
        }

        snapshot.writeSequence = begin;
        snapshot.attempts =
            m_queueTailAttempts.load(std::memory_order_relaxed);
        snapshot.commandAccepted =
            m_queueTailCommandAccepted.load(std::memory_order_relaxed);
        snapshot.commandRejected =
            m_queueTailCommandRejected.load(std::memory_order_relaxed);
        snapshot.committed =
            m_queueTailCommitted.load(std::memory_order_relaxed);
        snapshot.rejectPreserved =
            m_queueTailRejectPreserved.load(std::memory_order_relaxed);
        snapshot.commandedMCSCommitted =
            m_queueTailCommandedMCSCommitted.load(
                std::memory_order_relaxed);
        snapshot.lastQueuedPulseCommitted =
            m_queueTailLastQueuedPulseCommitted.load(
                std::memory_order_relaxed);
        snapshot.rapidOverrideCommitted =
            m_queueTailRapidOverrideCommitted.load(
                std::memory_order_relaxed);
        snapshot.endpointExact =
            m_queueTailEndpointExact.load(std::memory_order_relaxed);
        snapshot.captureBound =
            m_queueTailCaptureBound.load(std::memory_order_relaxed);
        snapshot.invalidInputs =
            m_queueTailInvalidInputs.load(std::memory_order_relaxed);
        snapshot.mismatches =
            m_queueTailMismatches.load(std::memory_order_relaxed);
        snapshot.lastTransactionSequence =
            m_lastQueueTailTransactionSequence.load(
                std::memory_order_relaxed);
        snapshot.lastExecutionEpoch =
            m_lastQueueTailExecutionEpoch.load(std::memory_order_relaxed);
        snapshot.lastSegmentId =
            m_lastQueueTailSegmentId.load(std::memory_order_relaxed);
        snapshot.lastAxisMask =
            m_lastQueueTailAxisMask.load(std::memory_order_relaxed);
        snapshot.lastCommittedFingerprint =
            m_lastQueueTailCommittedFingerprint.load(
                std::memory_order_relaxed);

        const std::uint64_t end =
            m_queueTailWriteSequence.load(std::memory_order_acquire);
        if (begin == end && (end & 1ULL) == 0ULL)
        {
            snapshot.writeSequence = end;
            snapshot.snapshotCoherent = true;
            break;
        }
    }

    snapshot.accountingValid =
        snapshot.snapshotCoherent &&
        snapshot.attempts ==
        snapshot.commandAccepted + snapshot.commandRejected &&
        snapshot.commandAccepted == snapshot.committed &&
        snapshot.commandRejected == snapshot.rejectPreserved &&
        snapshot.committed == snapshot.commandedMCSCommitted &&
        snapshot.committed == snapshot.lastQueuedPulseCommitted &&
        snapshot.committed == snapshot.rapidOverrideCommitted &&
        snapshot.committed == snapshot.endpointExact &&
        snapshot.captureBound <= snapshot.committed &&
        snapshot.invalidInputs <= snapshot.commandRejected &&
        snapshot.mismatches == 0ULL;

    snapshot.authoritative = true;
    snapshot.shadowOnly = false;
    snapshot.cutoverAttempted = snapshot.attempts != 0ULL;
    snapshot.runtimeInfluence = snapshot.committed != 0ULL;
    snapshot.ready =
        snapshot.committed != 0ULL &&
        snapshot.captureBound == snapshot.committed &&
        snapshot.accountingValid &&
        snapshot.commandRejected == 0ULL &&
        snapshot.invalidInputs == 0ULL &&
        snapshot.mismatches == 0ULL;
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


bool MotionCore::HasExactExecutionDrainAcknowledgement(
    MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease) const noexcept
{
    return HasExactExecutionDrainAcknowledgementImpl(executionEpoch, ownerLease, false);
}

bool MotionCore::HasExactExecutionDrainAcknowledgementImpl(
    MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease,
    bool ownsCommitReservation) const noexcept
{
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !ownerLease.IsValid())
    {
        return false;
    }

    const std::uint64_t entryExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t entryDrainRevocationGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint64_t entryObservedDrainRevocationGeneration =
        m_executionDrainObservedRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t entryDrainRevocationPublishers =
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire);
    const MotionOwnerLease entryOwnerLease =
        UnpackMotionOwnerState(entryOwnerState);
    // Only the transition caller may admit its own acquired reservation.
    // Public drain queries continue to reject every reservation as before.
    const std::uint64_t expectedReservation = ownsCommitReservation
        ? EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED : 0ULL;
    if ((entryExecutionPublication &
        (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != expectedReservation ||
        UnpackExecutionEpochPublication(entryExecutionPublication) !=
        executionEpoch ||
        !entryOwnerLease.Matches(ownerLease) ||
        UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(entryOwnerState) ||
        (entryOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        UnpackMotionOwnerSafetyRequestTicket(entryOwnerState) !=
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire) ||
        entryDrainRevocationPublishers != 0U ||
        entryDrainRevocationGeneration !=
        entryObservedDrainRevocationGeneration ||
        HasPendingSafetyOrRecoveryRequests())
    {
        return false;
    }

    MotionNCSettleSnapshot snapshot{};
    MotionNCSettleCounters counters{};
    if (!TryGetNCSettleEvidence(
        MotionNCSettleProfile::GROUP_COMPLETION,
        snapshot,
        counters))
    {
        return false;
    }

    // This is an RT-published acknowledgement, not a live NC-side peek.  Its
    // exact Epoch/Owner identity proves that ApplyPendingExecutionEpochChange
    // has completed and that the drained observation does not belong to the
    // preceding lifecycle generation.
    if (snapshot.publicationGeneration == 0ULL ||
        snapshot.profile != MotionNCSettleProfile::GROUP_COMPLETION ||
        snapshot.executionEpoch != executionEpoch ||
        snapshot.owner != ownerLease.owner ||
        snapshot.ownerGeneration != ownerLease.generation ||
        !snapshot.runtimeObserved ||
        !snapshot.runtimeCycleValid ||
        !snapshot.runtimeCycleContiguous ||
        snapshot.groupActive ||
        !snapshot.groupDrained ||
        snapshot.safetyOrRecoveryPending ||
        snapshot.commandQueueDepth != 0U ||
        snapshot.commandIngressDepth != 0U ||
        snapshot.commandReplayDepth != 0U)
    {
        return false;
    }

    // Close the publication-read seam with exact packed-word equality.  Split
    // getters could otherwise observe E, then miss a complete E+1 publish/apply
    // that clears PENDING again before the final boolean check.
    const std::uint64_t exitExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t exitOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t exitDrainRevocationGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint64_t exitObservedDrainRevocationGeneration =
        m_executionDrainObservedRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t exitDrainRevocationPublishers =
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire);
    return
        exitExecutionPublication == entryExecutionPublication &&
        exitOwnerState == entryOwnerState &&
        exitDrainRevocationPublishers == 0U &&
        exitDrainRevocationGeneration ==
        entryDrainRevocationGeneration &&
        exitObservedDrainRevocationGeneration ==
        entryObservedDrainRevocationGeneration &&
        exitDrainRevocationGeneration ==
        exitObservedDrainRevocationGeneration &&
        (exitExecutionPublication &
            (EXECUTION_EPOCH_PUBLICATION_PENDING |
                EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) == expectedReservation &&
        !HasPendingSafetyOrRecoveryRequests();
}


MotionNCTranslationTransitionResult MotionCore::TryTransitionNCTranslation(const NCTranslationSnapshot& previous,
    const NCTranslationSnapshot& next, MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease) noexcept
{
    if (!IsNCTranslationSnapshotValid(previous) || !IsNCTranslationSnapshotValid(next) ||
        previous.generation == (std::numeric_limits<std::uint64_t>::max)() ||
        previous.revision == (std::numeric_limits<std::uint64_t>::max)() ||
        next.generation != previous.generation + 1ULL ||
        next.revision != previous.revision + 1ULL ||
        !ownerLease.IsValid() || ownerLease.owner != MotionOwner::AUTO) return MotionNCTranslationTransitionResult::DEFERRED;
    // Exactly one standalone selection per drained transaction.
    // EXT and all fields outside that selection remain frozen.
    const bool planeChanged = previous.rotationPlane != next.rotationPlane;
    // A plane change must never reinterpret an active G68 frame, even from a
    // direct publisher that did not pass through the NC whole-block decoder.
    if (planeChanged && (previous.rotationMode != 69 || next.rotationMode != 69 ||
        previous.polarMode != 15 || next.polarMode != 15 ||
        NCTranslationHasWorkPlaneRotation(previous) ||
        NCTranslationHasWorkPlaneRotation(next)))
        return MotionNCTranslationTransitionResult::DEFERRED;
    const bool distanceChanged = previous.distanceMode != next.distanceMode;
    const bool unitsChanged = previous.unitsMode != next.unitsMode;
    const bool strokeChanged = previous.storedStrokeMode != next.storedStrokeMode;
    const bool polarChanged = previous.polarMode != next.polarMode;
    const bool cutterChanged = previous.cutterMode != next.cutterMode ||
        previous.cutterD != next.cutterD ||
        std::memcmp(&previous.cutterRadiusMM, &next.cutterRadiusMM, sizeof(double)) != 0;
    const bool workCoordinateChanged = previous.wcsCode != next.wcsCode;
    const bool toolLengthChanged = previous.toolLengthMode != next.toolLengthMode ||
        previous.toolHCode != next.toolHCode;
    const bool planarRotationChanged = previous.rotationMode != next.rotationMode ||
        std::memcmp(previous.rotationCenterMM, next.rotationCenterMM, sizeof(next.rotationCenterMM)) != 0 ||
        std::memcmp(&previous.rotationAngleDeg, &next.rotationAngleDeg, sizeof(double)) != 0;
    const bool scalingChanged = previous.scalingMode != next.scalingMode ||
        std::memcmp(&previous.scalingFactor, &next.scalingFactor, sizeof(double)) != 0 ||
        std::memcmp(previous.scalingCenterMM, next.scalingCenterMM, sizeof(next.scalingCenterMM)) != 0;
    const bool mirrorChanged = previous.mirrorMask != next.mirrorMask ||
        std::memcmp(previous.mirrorCenterMM, next.mirrorCenterMM, sizeof(next.mirrorCenterMM)) != 0;
    const bool workCompensationChanged = previous.workMode != next.workMode ||
        previous.workWCode != next.workWCode ||
        std::memcmp(previous.workOffset, next.workOffset, sizeof(next.workOffset)) != 0 ||
        std::memcmp(previous.workRotationCenterMM, next.workRotationCenterMM,
            sizeof(next.workRotationCenterMM)) != 0;
    if (static_cast<unsigned>(distanceChanged) + static_cast<unsigned>(workCoordinateChanged) +
        static_cast<unsigned>(toolLengthChanged) + static_cast<unsigned>(planarRotationChanged) +
        static_cast<unsigned>(workCompensationChanged) + static_cast<unsigned>(unitsChanged) +
        static_cast<unsigned>(scalingChanged) + static_cast<unsigned>(mirrorChanged) +
        static_cast<unsigned>(polarChanged) + static_cast<unsigned>(cutterChanged) +
        static_cast<unsigned>(strokeChanged) + static_cast<unsigned>(planeChanged) != 1U)
        return MotionNCTranslationTransitionResult::DEFERRED;
    // A selected contour owns one immutable frame and one physical radius.
    // Cancel it before changing its side, D row, radius or coordinate frame.
    if (previous.cutterMode != 40 &&
        (!cutterChanged || next.cutterMode != 40))
        return MotionNCTranslationTransitionResult::DEFERRED;
    if (planarRotationChanged)
    {
        const double canonicalZero = 0.0;
        if ((next.rotationMode == 68 && previous.distanceMode != 90) ||
            (next.rotationMode == 69 &&
                (std::memcmp(&next.rotationCenterMM[0], &canonicalZero, sizeof(double)) != 0 ||
                 std::memcmp(&next.rotationCenterMM[1], &canonicalZero, sizeof(double)) != 0 ||
                 std::memcmp(&next.rotationAngleDeg, &canonicalZero, sizeof(double)) != 0)))
            return MotionNCTranslationTransitionResult::DEFERRED;
    }
    if (workCompensationChanged)
    {
        const double canonicalZero = 0.0;
        const bool cancelled = next.workMode == 169;
        double workAngle = 0.0;
        if (!TryGetNCTranslationWorkPlaneAngle(next, workAngle))
            return MotionNCTranslationTransitionResult::DEFERRED;
        if (next.workMode == 168 && workAngle != 0.0 && previous.distanceMode != 90)
            return MotionNCTranslationTransitionResult::DEFERRED;
        if (cancelled || workAngle == 0.0)
        {
            for (unsigned axis = 0U; axis < 2U; ++axis)
                if (std::memcmp(&next.workRotationCenterMM[axis], &canonicalZero, sizeof(double)) != 0)
                    return MotionNCTranslationTransitionResult::DEFERRED;
        }
        if (cancelled)
        {
            for (unsigned axis = 0U; axis < 8U; ++axis)
                if (std::memcmp(&next.workOffset[axis], &canonicalZero, sizeof(double)) != 0)
                    return MotionNCTranslationTransitionResult::DEFERRED;
        }
    }
    NCTranslationSnapshot expected = previous;
    if (planeChanged) expected.rotationPlane = next.rotationPlane;
    else if (distanceChanged) expected.distanceMode = next.distanceMode;
    else if (unitsChanged) expected.unitsMode = next.unitsMode;
    else if (strokeChanged) expected.storedStrokeMode = next.storedStrokeMode;
    else if (polarChanged) expected.polarMode = next.polarMode;
    else if (cutterChanged)
    {
        expected.cutterMode = next.cutterMode;
        expected.cutterD = next.cutterD;
        expected.cutterRadiusMM = next.cutterRadiusMM;
    }
    else if (workCoordinateChanged)
    {
        expected.wcsCode = next.wcsCode;
        std::memcpy(expected.wcsOffsetMM, next.wcsOffsetMM, sizeof(expected.wcsOffsetMM));
    }
    else if (toolLengthChanged)
    {
        expected.toolLengthMode = next.toolLengthMode;
        expected.toolHCode = next.toolHCode;
        std::memcpy(expected.toolOffsetMM, next.toolOffsetMM, sizeof(expected.toolOffsetMM));
    }
    else if (planarRotationChanged)
    {
        expected.rotationMode = next.rotationMode;
        std::memcpy(expected.rotationCenterMM, next.rotationCenterMM, sizeof(expected.rotationCenterMM));
        expected.rotationAngleDeg = next.rotationAngleDeg;
    }
    else if (scalingChanged)
    {
        expected.scalingMode = next.scalingMode;
        expected.scalingFactor = next.scalingFactor;
        std::memcpy(expected.scalingCenterMM, next.scalingCenterMM, sizeof(expected.scalingCenterMM));
    }
    else if (mirrorChanged)
    {
        expected.mirrorMask = next.mirrorMask;
        std::memcpy(expected.mirrorCenterMM, next.mirrorCenterMM, sizeof(expected.mirrorCenterMM));
    }
    else
    {
        expected.workMode = next.workMode;
        expected.workWCode = next.workWCode;
        std::memcpy(expected.workOffset, next.workOffset, sizeof(expected.workOffset));
        std::memcpy(expected.workRotationCenterMM, next.workRotationCenterMM,
            sizeof(expected.workRotationCenterMM));
    }
    expected.generation = next.generation;
    expected.revision = next.revision;
    if (!SameNCTranslationSnapshot(expected, next) || !MatchesNCTranslation(previous) ||
        !HasExactExecutionDrainAcknowledgement(executionEpoch, ownerLease)) return MotionNCTranslationTransitionResult::DEFERRED;

    // This identity is only an epoch reservation key. It is never submitted,
    // allocated as a real segment, entered in a ledger or published as feedback.
    MotionExecutionIdentity reservationIdentity{};
    reservationIdentity.epoch = executionEpoch;
    reservationIdentity.segmentId = 1ULL; // Existing reservation requires IsAssigned().
    reservationIdentity.source = MotionCommandSource::NC_MEMORY;
    LifecycleCommitReservationGuard transition(*this, reservationIdentity);
    if (!transition.IsAcquired() || GetQueueSize() != 0U ||
        GetCommandIngressSize() != 0U || GetCommandReplaySize() != 0U ||
        !MatchesNCTranslation(previous) ||
        !HasExactExecutionDrainAcknowledgementImpl(executionEpoch, ownerLease, true)) return MotionNCTranslationTransitionResult::DEFERRED;

    // Completed queued LINEAR and canonical ARC endpoints carry the same exact
    // native endpoint contract as ordinary fixed XYZ commands. Inspect
    // the now-stable last physical source, including an older epoch retained
    // across a zero-point command. Consecutive mode-only blocks keep this run
    // and lease but intentionally have newer translation generations.
    const MotionCommand& predecessor = m_Group.currentCmd;
    if (predecessor.execution.IsAssigned() &&
        predecessor.execution.source == MotionCommandSource::NC_MEMORY &&
        predecessor.ownerLease.Matches(ownerLease) &&
        predecessor.sourceTranslation.runToken == previous.runToken &&
        (predecessor.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
            predecessor.cncFeedLookahead || predecessor.cncCornerBlend ||
            predecessor.pathCoreRetainedTraversal || predecessor.pathCoreRetainedReverse ||
            predecessor.replayTerminalAlreadyPublished) &&
        !HasCompletedCncLineEndpointProof(predecessor, previous, executionEpoch))
        return MotionNCTranslationTransitionResult::UNSUPPORTED_PREDECESSOR;

    // A single NC writer owns this publication. Valid next=previous+1 and an
    // exact current match make this retire/publish monotonic. RT has no active
    // or queued source, and epoch replacement is excluded by the reservation.
    m_translationPublication.Retire();
    if (!PublishNCTranslation(next)) return MotionNCTranslationTransitionResult::DEFERRED;
    if (predecessor.cncFeedLookahead && predecessor.pathCorePlanarCircle &&
        predecessor.execution.IsAssigned() && predecessor.execution.epoch == executionEpoch &&
        predecessor.execution.source == MotionCommandSource::NC_MEMORY &&
        predecessor.ownerLease.Matches(ownerLease) &&
        predecessor.sourceTranslation.runToken == previous.runToken)
    {
        // Capture stable packet scalars, then release the lifecycle reservation
        // before NC-side console I/O. No I/O is added to the 250 us path.
        const MotionSegmentId completedSegment = predecessor.execution.segmentId;
        const int completedSourcePC = predecessor.sourceLinePC;
        transition.Release();
        RtPrintf("[COORD][ARC-ENDPOINT] run=%llu epoch=%llu seg=%llu sourcePC=%d generation=%llu completed=1 drained=1\n",
            static_cast<unsigned long long>(previous.runToken),
            static_cast<unsigned long long>(executionEpoch),
            static_cast<unsigned long long>(completedSegment), completedSourcePC,
            static_cast<unsigned long long>(next.generation));
    }
    return MotionNCTranslationTransitionResult::ACCEPTED;
}


bool MotionCore::HasExactProgramStartQuiescenceAcknowledgement(
    MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease) const noexcept
{
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !ownerLease.IsValid())
    {
        return false;
    }

    // Keep the same entry seam as HasExactExecutionDrainAcknowledgement.
    // A program start is only allowed to extend that exact Epoch/Owner proof;
    // it must never rely on a newer or split publication.
    const std::uint64_t entryExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t entryOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t entryDrainRevocationGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint64_t entryObservedDrainRevocationGeneration =
        m_executionDrainObservedRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t entryDrainRevocationPublishers =
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire);
    const MotionOwnerLease entryOwnerLease =
        UnpackMotionOwnerState(entryOwnerState);
    if ((entryExecutionPublication &
        (EXECUTION_EPOCH_PUBLICATION_PENDING |
            EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL ||
        UnpackExecutionEpochPublication(entryExecutionPublication) !=
        executionEpoch ||
        !entryOwnerLease.Matches(ownerLease) ||
        UnpackMotionOwnerSafetyHandshake(entryOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(entryOwnerState) ||
        (entryOwnerState &
            MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        UnpackMotionOwnerSafetyRequestTicket(entryOwnerState) !=
        m_safetyRequestAcknowledgedTicket.load(
            std::memory_order_acquire) ||
        entryDrainRevocationPublishers != 0U ||
        entryDrainRevocationGeneration !=
        entryObservedDrainRevocationGeneration ||
        HasPendingSafetyOrRecoveryRequests())
    {
        return false;
    }

    // Read the formal execution-drain proof and the physical stop evidence
    // from one atomic RT publication bank.  Calling two public getters here
    // could combine different 250 us samples and let a fresh program start
    // cross a post-Reset coast-down seam.
    MotionStopSettlePublicationPayload payload{};
    if (!TryReadStopSettlePublication(payload))
    {
        return false;
    }

    MotionProgramStartReadinessSnapshot readiness{};
    if (!TryReadProgramStartReadinessPublication(readiness))
    {
        return false;
    }

    const MotionNCSettleSnapshot& drain =
        payload.ncSettleSnapshots[static_cast<std::size_t>(
            MotionNCSettleProfile::GROUP_COMPLETION)];
    const MotionStopSettleSnapshot& stop = payload.snapshot;

    // Preserve every formal identity and ownership requirement of the
    // generic drain acknowledgement.  Do not require drain.settled here:
    // a brand-new Epoch has no group scope yet, so it legitimately reports a
    // drained empty scope before the first command is admitted.
    if (drain.publicationGeneration == 0ULL ||
        drain.profile != MotionNCSettleProfile::GROUP_COMPLETION ||
        drain.executionEpoch != executionEpoch ||
        drain.owner != ownerLease.owner ||
        drain.ownerGeneration != ownerLease.generation ||
        !drain.runtimeObserved ||
        !drain.runtimeCycleValid ||
        !drain.runtimeCycleContiguous ||
        drain.groupActive ||
        !drain.groupDrained ||
        drain.safetyOrRecoveryPending ||
        drain.commandQueueDepth != 0U ||
        drain.commandIngressDepth != 0U ||
        drain.commandReplayDepth != 0U)
    {
        return false;
    }

    // The generic drain proof deliberately does not require physical
    // convergence.  Program start does, but its physical proof must match the
    // one used by LoadNextCommand() for an incoming axis.  The raw 250 us
    // derivative velocity and a small final PDO TargetVelocity are both
    // advisory in an IDLE servo: treating them as motion caused a stationary
    // system to remain in START_PENDING forever.  Instead, require every
    // existing axis to satisfy the exact incoming-axis predicate (servo,
    // startup-lag state, fault state, canonical IDLE, in-position state,
    // finite command data, zero commanded velocity and following window).
    // This holds a fresh G00 at the NC boundary whenever LoadNextCommand()
    // would otherwise invoke the AL3021 fail-closed containment path.
    if (stop.publicationGeneration == 0ULL ||
        stop.publicationGeneration != drain.publicationGeneration ||
        stop.sampleSequence == 0ULL ||
        stop.groupActive ||
        stop.commandQueueDepth != 0U ||
        stop.commandIngressDepth != 0U ||
        stop.commandReplayDepth != 0U ||
        stop.nonIdleAxisCount != 0U ||
        stop.commandMovingAxisCount != 0U ||
        stop.outsideInPositionWindowAxisCount != 0U ||
        readiness.stopPublicationGeneration !=
        stop.publicationGeneration ||
        readiness.stopSampleSequence != stop.sampleSequence ||
        readiness.existingAxisCount == 0U ||
        readiness.notReadyAxisCount != 0U ||
        readiness.readyAxisCount != readiness.existingAxisCount)
    {
        return false;
    }

    // Close the same lifecycle seam after the physical proof.  Any change
    // while reading invalidates the admission and leaves Program Start
    // pending for the next 10 ms NC scan.
    const std::uint64_t exitExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    const std::uint64_t exitOwnerState =
        m_motionOwnerState.load(std::memory_order_acquire);
    const std::uint64_t exitDrainRevocationGeneration =
        m_executionDrainRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint64_t exitObservedDrainRevocationGeneration =
        m_executionDrainObservedRevocationGeneration.load(
            std::memory_order_acquire);
    const std::uint32_t exitDrainRevocationPublishers =
        m_executionDrainRevocationPublishersInProgress.load(
            std::memory_order_acquire);
    return
        exitExecutionPublication == entryExecutionPublication &&
        exitOwnerState == entryOwnerState &&
        exitDrainRevocationPublishers == 0U &&
        exitDrainRevocationGeneration ==
        entryDrainRevocationGeneration &&
        exitObservedDrainRevocationGeneration ==
        entryObservedDrainRevocationGeneration &&
        exitDrainRevocationGeneration ==
        exitObservedDrainRevocationGeneration &&
        (exitExecutionPublication &
            (EXECUTION_EPOCH_PUBLICATION_PENDING |
                EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) == 0ULL &&
        !HasPendingSafetyOrRecoveryRequests();
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
                m_pathHold.unionActive ? m_Group.axisCount : m_Group.currentCmd.axisCount,
                MAX_AXES));

    for (int j = 0; j < commandAxisCount; ++j)
    {
        const int axisIndex =
            m_pathHold.unionActive ? m_Group.axisIndices[j] : m_Group.currentCmd.axisIndices[j];

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
template void MotionCore::Run_Servo_Loop<ENI_ServoDrive>(ENI_ServoDrive&, AxisContext&, const AxisCommand&,
    const MotionServoInputSnapshot&);
