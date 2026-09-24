#pragma once
// PBC-2: mechanical compensation coordinate and completion contract.
// RT local values only. This is NOT a driver-send acknowledgement and is NOT
// permission to enable compensation. No allocation, locks or I/O below.
#include "MechanicalCompensationModel.h"

namespace pbc
{
struct CoordinateFrame
{
    double nominalCommandPulse = 0.0;
    double nominalVelocityPPS = 0.0;
    double servoCommandPulse = 0.0;
    double servoVelocityPPS = 0.0;
    double pulsePerUnit = 0.0;
    double offsetUnit = 0.0;
    double offsetPulse = 0.0;
    double offsetVelocityPPS = 0.0;
    double remainingUnit = 0.0;
    bool enabled = false;
    bool valid = false;
    bool settled = false;
};

inline bool IsCoordinateFrameValid(const CoordinateFrame& f) noexcept
{
    return f.valid && std::isfinite(f.nominalCommandPulse) &&
        std::isfinite(f.nominalVelocityPPS) && std::isfinite(f.servoCommandPulse) &&
        std::isfinite(f.servoVelocityPPS) && std::isfinite(f.pulsePerUnit) &&
        f.pulsePerUnit > 0.0 && std::isfinite(f.offsetUnit) &&
        std::isfinite(f.offsetPulse) && std::isfinite(f.offsetVelocityPPS) &&
        std::isfinite(f.remainingUnit) && f.remainingUnit >= 0.0 &&
        f.offsetPulse == f.offsetUnit * f.pulsePerUnit &&
        f.servoCommandPulse == f.nominalCommandPulse + f.offsetPulse &&
        f.servoVelocityPPS == f.nominalVelocityPPS + f.offsetVelocityPPS &&
        (!f.settled || f.remainingUnit == 0.0) &&
        (f.enabled || (f.offsetUnit == 0.0 && f.offsetVelocityPPS == 0.0 &&
            f.remainingUnit == 0.0));
}

// Compose into a candidate first; rejection leaves the caller's frame intact.
inline bool BuildCoordinateFrame(const Config& c, const Input& input,
    const Output& output, CoordinateFrame& destination) noexcept
{
    CoordinateFrame next{};
    next.nominalCommandPulse = input.nominalPulse;
    next.nominalVelocityPPS = input.nominalVelocityPPS;
    next.servoCommandPulse = output.servoPulse;
    next.servoVelocityPPS = output.servoVelocityPPS;
    next.pulsePerUnit = c.pulsePerUnit;
    next.offsetUnit = output.offsetUnit;
    next.offsetPulse = output.offsetUnit * c.pulsePerUnit;
    next.offsetVelocityPPS = output.offsetVelocityPPS;
    next.remainingUnit = output.remainingUnit;
    next.enabled = c.pitch || c.backlash;
    next.settled = output.settled;
    next.valid = true;
    if (!IsCoordinateFrameValid(next)) return false;
    destination = next;
    return true;
}

inline bool HasDisabledCoordinateIdentity(bool enabled, double offsetUnit,
    const CoordinateFrame& frame) noexcept
{
    // Preserve the historical OFF path without any unit conversion. A stale
    // active frame/offset cannot silently become the OFF identity.
    return !enabled && offsetUnit == 0.0 && !frame.enabled &&
        frame.offsetUnit == 0.0 && frame.offsetPulse == 0.0 &&
        frame.offsetVelocityPPS == 0.0 && frame.remainingUnit == 0.0;
}

inline bool MatchesAppliedCoordinateFrame(bool enabled, double offsetUnit,
    double resolution, double lead, const CoordinateFrame& frame) noexcept
{
    if (!enabled || !frame.enabled || !IsCoordinateFrameValid(frame) ||
        !std::isfinite(resolution) || resolution <= 0.0 ||
        !std::isfinite(lead) || lead <= 0.0 || frame.offsetUnit != offsetUnit)
        return false;
    const double scale = resolution / lead;
    return std::isfinite(scale) && scale > 0.0 && frame.pulsePerUnit == scale;
}

inline double NominalFeedbackPulse(bool enabled, double offsetUnit,
    double rawFeedbackPulse, double resolution, double lead,
    const CoordinateFrame& frame) noexcept
{
    if (HasDisabledCoordinateIdentity(enabled, offsetUnit, frame))
        return rawFeedbackPulse;
    if (!MatchesAppliedCoordinateFrame(enabled, offsetUnit, resolution, lead, frame) ||
        !std::isfinite(rawFeedbackPulse))
        return (std::numeric_limits<double>::quiet_NaN)();
    const double nominal = rawFeedbackPulse - frame.offsetPulse;
    return std::isfinite(nominal) ? nominal :
        (std::numeric_limits<double>::quiet_NaN)();
}

inline double FollowingErrorPulse(bool enabled, double offsetUnit,
    double nominalCommandPulse, double rawFeedbackPulse,
    double resolution, double lead, const CoordinateFrame& frame) noexcept
{
    if (HasDisabledCoordinateIdentity(enabled, offsetUnit, frame))
        return nominalCommandPulse - rawFeedbackPulse;
    if (!MatchesAppliedCoordinateFrame(enabled, offsetUnit, resolution, lead, frame) ||
        !std::isfinite(nominalCommandPulse) || !std::isfinite(rawFeedbackPulse))
        return (std::numeric_limits<double>::infinity)();
    // Use the SAME arithmetic order as the servo loop, not an independently
    // rounded inverse transform. Raw feedback remains available for limits.
    const double servoTarget = nominalCommandPulse + frame.offsetPulse;
    const double error = servoTarget - rawFeedbackPulse;
    return std::isfinite(error) ? error :
        (std::numeric_limits<double>::infinity)();
}

inline bool CoordinateCompletionReady(bool enabled, double offsetUnit,
    double nominalCommandPulse, double resolution, double lead,
    const CoordinateFrame& frame) noexcept
{
    if (HasDisabledCoordinateIdentity(enabled, offsetUnit, frame)) return true;
    return MatchesAppliedCoordinateFrame(enabled, offsetUnit, resolution, lead, frame) &&
        std::isfinite(nominalCommandPulse) &&
        frame.nominalCommandPulse == nominalCommandPulse &&
        frame.settled && frame.remainingUnit == 0.0 && frame.offsetVelocityPPS == 0.0;
}

// A pure reference-boundary value, for the later HOME/RESET/Servo integration.
// Never mutate actual feedback, clear an offset, or rebase an axis here.
struct NominalRebasePlan
{
    double rawFeedbackPulse = 0.0;
    double nominalCommandPulse = 0.0;
    double retainedOffsetPulse = 0.0;
};
inline bool PrepareNominalRebase(bool enabled, double offsetUnit,
    double rawFeedbackPulse, double resolution, double lead,
    const CoordinateFrame& frame, NominalRebasePlan& destination) noexcept
{
    const double nominal = NominalFeedbackPulse(enabled, offsetUnit,
        rawFeedbackPulse, resolution, lead, frame);
    if (!std::isfinite(nominal) || !std::isfinite(rawFeedbackPulse)) return false;
    NominalRebasePlan next{};
    next.rawFeedbackPulse = rawFeedbackPulse;
    next.nominalCommandPulse = nominal;
    next.retainedOffsetPulse = enabled ? frame.offsetPulse : 0.0;
    destination = next;
    return true;
}

inline bool AxisModel::StepCoordinateFrame(const Input& input,
    CoordinateFrame& frame, Diagnostic& d) noexcept
{
    State candidateState{};
    Output candidateOutput{};
    CoordinateFrame candidateFrame{};
    if (!EvaluateStep(input, candidateState, candidateOutput, d)) return false;
    if (!BuildCoordinateFrame(m_config, input, candidateOutput, candidateFrame))
    {
        d.error = Error::RuntimeNumber;
        return false;
    }
    m_state = candidateState;
    frame = candidateFrame;
    return true;
}

// Boot-only deterministic check of this exact coordinate implementation.
// All values are local synthetic values: no AxisContext, drive, PDO, HOME bit,
// parameters or compensation model used by Motion can be modified here.
inline bool CheckCoordinateContract(unsigned& passed) noexcept
{
    passed = 0U;
    const auto check = [&passed](bool result) noexcept
    { if (result) ++passed; return result; };
    Config c{};
    c.pitch = true;
    c.pulsePerUnit = 1000.0;
    Input in{};
    in.nominalPulse = 10000.0;
    Output out{};
    out.servoPulse = 10020.0;
    out.offsetUnit = 0.020;
    CoordinateFrame f{};
    if (!check(BuildCoordinateFrame(c, in, out, f))) return false;
    if (!check(NominalFeedbackPulse(true, 0.020, 10020.0, 1000.0, 1.0, f) == 10000.0)) return false;
    if (!check(FollowingErrorPulse(true, 0.020, 10000.0, 10020.0, 1000.0, 1.0, f) == 0.0)) return false;
    if (!check(CoordinateCompletionReady(true, 0.020, 10000.0, 1000.0, 1.0, f))) return false;
    if (!check(!CoordinateCompletionReady(true, 0.020, 10001.0, 1000.0, 1.0, f))) return false;
    CoordinateFrame pending = f;
    pending.remainingUnit = 0.001;
    pending.settled = false;
    if (!check(!CoordinateCompletionReady(true, 0.020, 10000.0, 1000.0, 1.0, pending))) return false;
    CoordinateFrame moving = f;
    moving.offsetVelocityPPS = 1.0;
    moving.servoVelocityPPS = 1.0;
    if (!check(!CoordinateCompletionReady(true, 0.020, 10000.0, 1000.0, 1.0, moving))) return false;
    if (!check(!std::isfinite(NominalFeedbackPulse(true, 0.021, 10020.0, 1000.0, 1.0, f)))) return false;
    if (!check(!std::isfinite(FollowingErrorPulse(true, 0.020, 10000.0, 10020.0, 2000.0, 1.0, f)))) return false;
    NominalRebasePlan plan{};
    if (!check(PrepareNominalRebase(true, 0.020, 10020.0, 1000.0, 1.0, f, plan))) return false;
    if (!check(plan.nominalCommandPulse + plan.retainedOffsetPulse == plan.rawFeedbackPulse)) return false;
    NominalRebasePlan again{};
    if (!check(PrepareNominalRebase(true, 0.020, plan.rawFeedbackPulse, 1000.0, 1.0, f, again) &&
        again.nominalCommandPulse == plan.nominalCommandPulse)) return false;
    CoordinateFrame off{};
    if (!check(NominalFeedbackPulse(false, 0.0, -123.0, 0.0, 0.0, off) == -123.0)) return false;
    if (!check(FollowingErrorPulse(false, 0.0, 10.0, 9.0, 0.0, 0.0, off) == 1.0)) return false;
    if (!check(CoordinateCompletionReady(false, 0.0, 10.0, 0.0, 0.0, off))) return false;
    if (!check(!CoordinateCompletionReady(false, 0.0, 10000.0, 1000.0, 1.0, f))) return false;
    CoordinateFrame unchanged = f;
    out.servoPulse += 1.0;
    if (!check(!BuildCoordinateFrame(c, in, out, unchanged) &&
        unchanged.servoCommandPulse == f.servoCommandPulse)) return false;
    return true;
}

}
