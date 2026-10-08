#pragma once

#include "MotionEccentricCTransport.h"

// BASE79D: read-only pre-start binding, after decoding this exact packet with
// PrepareMotionEccentricCCommand. This is neither packet decoding nor output
// authorization. The RT consumer must separately acquire/recheck the existing
// lifecycle reservation before any group/cursor/axis write. Live admission is
// still closed; these helpers never change a role, H table, axis or queue.
#if defined(_MSC_VER)
#define BASE79D_BIND_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79D_BIND_NOINLINE __attribute__((noinline))
#else
#define BASE79D_BIND_NOINLINE
#endif

BASE79D_BIND_NOINLINE inline bool IsMotionEccentricCStartSourceBound(
    const MotionCommand& command, const NCEccentricCRuntimeValue& runtime) noexcept
{
    using namespace NCEccentricCDetail;
    if (!runtime.IsValid() || !IsMotionEccentricCFeedSourceAllowed(command) ||
        runtime.FirSamples() != 1U || runtime.CycleSeconds() != 0.00025 ||
        !SameNCTranslationSnapshot(command.sourceTranslation, runtime.AuthoredPath().source)) return false;
    const auto& path = runtime.AuthoredPath();
    const auto& extended = runtime.ExtendedPath();
    const bool mixed = MotionEccentricCTransportDetail::MixedLayout(command);
    const bool xy = MotionEccentricCTransportDetail::AuthoredXY(command);
    if (path.electrodeAxis >= 8U ||
        !Same(path.xDeltaMM, xy ? command.centerPos[0] : 0.0) ||
        !Same(path.yDeltaMM, xy ? command.centerPos[1] : 0.0) ||
        !Same(path.zDeltaMM, mixed ? command.mem_transformMatrix[2][0] : 0.0) ||
        !Same(command.mem_radius, path.sweepDeg) || !Same(command.mem_startAngle, path.feedDegMin) ||
        !Same(command.accTime, path.accTime) || !Same(command.decTime, path.decTime) ||
        !Same(command.mem_centerX, runtime.StopSeconds()) ||
        !Same(command.targetVel, runtime.MaximumScalarVelocityPulsePerSec())) return false;
    std::uint32_t mask = 0U;
    for (int slot = 0; slot < command.axisCount; ++slot)
    {
        const unsigned axis = static_cast<unsigned>(command.axisIndices[slot]);
        if (axis >= 8U || !Same(command.targetPos[slot], runtime.EndPulse()[axis])) return false;
        mask |= 1U << axis;
    }
    if (mask != path.groupMask) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (!Same(command.mem_startPos[axis], runtime.StartPulse()[axis]) ||
            !Same(command.mem_ratio[axis], runtime.PulsePerNative()[axis])) return false;
    for (unsigned slot = 0U; slot < (mixed ? 4U : 3U); ++slot)
    {
        const unsigned axis = slot < (mixed ? 3U : 2U) ? slot : path.electrodeAxis;
        const double lower = MotionEccentricCTransportDetail::Lower(command, axis);
        const double upper = MotionEccentricCTransportDetail::Upper(command, axis);
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper ||
            extended.minMCS[axis] < lower || extended.maxMCS[axis] > upper ||
            runtime.MinimumPulse()[axis] / runtime.PulsePerNative()[axis] < lower ||
            runtime.MaximumPulse()[axis] / runtime.PulsePerNative()[axis] > upper ||
            !Positive(MotionEccentricCTransportDetail::Limit(command, axis, 0U)) ||
            !Positive(MotionEccentricCTransportDetail::Limit(command, axis, 1U)) ||
            !Positive(MotionEccentricCTransportDetail::Limit(command, axis, 2U)) ||
            extended.requiredVelocityNative[axis] > MotionEccentricCTransportDetail::Limit(command, axis, 0U) ||
            extended.requiredAccelerationNative[axis] > MotionEccentricCTransportDetail::Limit(command, axis, 1U) ||
            extended.requiredDecelerationNative[axis] > MotionEccentricCTransportDetail::Limit(command, axis, 2U)) return false;
    }
    return true;
}

// The caller obtains softwareEnvelopeAllowed by querying the existing travel
// service for BOTH extrema of the extended native curve and outward-rounded
// pulse envelope. Checking only start/end misses internal extrema and STOP.
BASE79D_BIND_NOINLINE inline bool IsMotionEccentricCStartAxisBound(
    const MotionCommand& command, const NCEccentricCRuntimeValue& runtime,
    unsigned index, const AxisContext* axis, bool softwareEnvelopeAllowed) noexcept
{
    using namespace NCEccentricCDetail;
    if (!runtime.IsValid() || index >= 8U) return false;
    const auto& path = runtime.AuthoredPath();
    const auto& identity = path.source.axisIdentity;
    const bool present = axis != nullptr && axis->isExist;
    if (present != (identity.exists[index] == 1U)) return false;
    if (!present) return true;
    double ppu = 0.0;
    if (axis->axisIndex != static_cast<int>(index) || axis->isVirtualAxis ||
        identity.physicalIndexPlusOne[index] != index + 1U ||
        static_cast<std::uint32_t>(axis->axisType) != identity.axisType[index] ||
        !TryGetMotionPulsePerUnit(axis->resolution_PPR, axis->finalLead,
            axis->axisType != AxisType::LINEAR, ppu) ||
        !Same(ppu, runtime.PulsePerNative()[index]) ||
        !Same(axis->logicalCmdPos.Load(), runtime.StartPulse()[index]) ||
        !std::isfinite(axis->logicalCmdVel) || axis->logicalCmdVel != 0.0 ||
        !std::isfinite(axis->currentCmdVel) || axis->currentCmdVel != 0.0 ||
        axis->state != MotionState::MotionState_IDLE || axis->isFault || axis->isLagAlarm ||
        !softwareEnvelopeAllowed) return false;
    const bool moving = (path.groupMask & (1U << index)) != 0U;
    // Dir3 compresses all XYZ ceilings even for an un-authored linear axis.
    // Rebind that ceiling before start without requiring a held axis servo
    // or adopting its stop time into the moving group's stop contract.
    const bool compressedLinear = MotionEccentricCTransportDetail::AuthoredXY(command) && index < 3U;
    if (!moving && !compressedLinear) return true;
    const double velocity = axis->maxVel_PPS / ppu;
    if (!Positive(velocity) || !Positive(axis->G00_acc_time) || !Positive(axis->G00_dec_time) ||
        (moving && (!axis->isServoOn || !std::isfinite(axis->Stop_dec_time) ||
            axis->Stop_dec_time < 0.001 || axis->Stop_dec_time > runtime.StopSeconds()))) return false;
    const double acceleration = velocity / axis->G00_acc_time;
    const double deceleration = velocity / axis->G00_dec_time;
    const auto& extended = runtime.ExtendedPath();
    return Positive(acceleration) && Positive(deceleration) &&
        Positive(MotionEccentricCTransportDetail::Limit(command, index, 0U)) &&
        Positive(MotionEccentricCTransportDetail::Limit(command, index, 1U)) &&
        Positive(MotionEccentricCTransportDetail::Limit(command, index, 2U)) &&
        MotionEccentricCTransportDetail::Limit(command, index, 0U) <= velocity &&
        MotionEccentricCTransportDetail::Limit(command, index, 1U) <= acceleration &&
        MotionEccentricCTransportDetail::Limit(command, index, 2U) <= deceleration &&
        extended.requiredVelocityNative[index] <= velocity &&
        extended.requiredAccelerationNative[index] <= acceleration &&
        extended.requiredDecelerationNative[index] <= deceleration;
}

#undef BASE79D_BIND_NOINLINE
