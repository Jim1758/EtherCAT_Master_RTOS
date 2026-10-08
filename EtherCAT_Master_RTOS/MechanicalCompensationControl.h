#pragma once
// PBC-3E: the exact frame used by both the normal PID and stopped hold.
// Bounded scalar validation only; no model mutation, allocation or I/O.
#include "MechanicalCompensationCoordinates.h"
#include <cstring>

namespace pbc
{
struct ControlFrame
{
    double servoCommandPulse = 0.0;
    double servoVelocityPPS = 0.0;
    double followingErrorPulse = 0.0;
    double offsetVelocityPPS = 0.0;
};

inline bool SameControlDouble(double a, double b) noexcept
{
    // Equality alone would hide a change between +0 and -0 on the OFF path.
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

inline bool SameControlCoordinateFrame(const CoordinateFrame& a,
    const CoordinateFrame& b) noexcept
{
    return SameControlDouble(a.nominalCommandPulse, b.nominalCommandPulse) &&
        SameControlDouble(a.nominalVelocityPPS, b.nominalVelocityPPS) &&
        SameControlDouble(a.servoCommandPulse, b.servoCommandPulse) &&
        SameControlDouble(a.servoVelocityPPS, b.servoVelocityPPS) &&
        SameControlDouble(a.pulsePerUnit, b.pulsePerUnit) &&
        SameControlDouble(a.offsetUnit, b.offsetUnit) &&
        SameControlDouble(a.offsetPulse, b.offsetPulse) &&
        SameControlDouble(a.offsetVelocityPPS, b.offsetVelocityPPS) &&
        SameControlDouble(a.remainingUnit, b.remainingUnit) &&
        a.enabled == b.enabled && a.valid == b.valid && a.settled == b.settled;
}

inline bool ResolveControlFrame(const CoordinateFrame& frame,
    double expectedNominalPos, double expectedNominalVel,
    double rawFeedbackPulse, ControlFrame& destination) noexcept
{
    if (!IsCoordinateFrameValid(frame) || !std::isfinite(rawFeedbackPulse) ||
        !SameControlDouble(frame.nominalCommandPulse, expectedNominalPos) ||
        !SameControlDouble(frame.nominalVelocityPPS, expectedNominalVel))
        return false;
    if (!frame.enabled &&
        (!SameControlDouble(frame.servoCommandPulse, expectedNominalPos) ||
            !SameControlDouble(frame.servoVelocityPPS, expectedNominalVel)))
        return false;
    ControlFrame next{};
    // Copies retain OFF signed-zero and rounding exactly. Enabled frame
    // composition has already checked nominal + applied/proposed offset.
    next.servoCommandPulse = frame.servoCommandPulse;
    next.servoVelocityPPS = frame.servoVelocityPPS;
    next.followingErrorPulse = next.servoCommandPulse - rawFeedbackPulse;
    next.offsetVelocityPPS = frame.offsetVelocityPPS;
    if (!std::isfinite(next.followingErrorPulse)) return false;
    destination = next;
    return true;
}

inline bool CheckControlFrameContract(unsigned& passed) noexcept
{
    passed = 0U;
    const auto check = [&passed](bool ok) noexcept { if (ok) ++passed; return ok; };
    Config config{}; config.pulsePerUnit = 1000.0;
    Input input{}; input.nominalPulse = -0.0; input.nominalVelocityPPS = -0.0;
    Output output{}; output.servoPulse = -0.0; output.servoVelocityPPS = -0.0;
    CoordinateFrame frame{}; ControlFrame control{};
    if (!check(BuildCoordinateFrame(config, input, output, frame))) return false;
    if (!check(ResolveControlFrame(frame, -0.0, -0.0, 0.0, control))) return false;
    if (!check(std::signbit(control.servoCommandPulse) && std::signbit(control.servoVelocityPPS))) return false;
    if (!check(!ResolveControlFrame(frame, 0.0, -0.0, 0.0, control))) return false;
    config.pitch = true; input.nominalPulse = 10000.0; input.nominalVelocityPPS = 100.0;
    output.offsetUnit = 0.02; output.offsetVelocityPPS = 10.0;
    output.servoPulse = 10020.0; output.servoVelocityPPS = 110.0;
    if (!check(BuildCoordinateFrame(config, input, output, frame))) return false;
    if (!check(ResolveControlFrame(frame, 10000.0, 100.0, 10018.0, control))) return false;
    if (!check(control.followingErrorPulse == 2.0 && control.servoVelocityPPS == 110.0 &&
        control.offsetVelocityPPS == 10.0)) return false;
    const ControlFrame saved = control;
    if (!check(!ResolveControlFrame(frame, 10001.0, 100.0, 10018.0, control))) return false;
    if (!check(control.servoCommandPulse == saved.servoCommandPulse &&
        control.followingErrorPulse == saved.followingErrorPulse)) return false;
    if (!check(!ResolveControlFrame(frame, 10000.0, 100.0,
        (std::numeric_limits<double>::quiet_NaN)(), control))) return false;
    frame.servoCommandPulse += 1.0;
    if (!check(!ResolveControlFrame(frame, 10000.0, 100.0, 10018.0, control))) return false;
    return true;
}
}
