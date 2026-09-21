#include "MotionCore.h"

void MotionCore::G161_Move(const std::vector<int>& axes,
    const std::vector<double>& nativeTargets, BufferMode mode)
{
    // Compatibility callers share the guarded admission and success-only pulse
    // tail. The dedicated profile accepts exact-stop ABORTING positioning only.
    (void)TryG00MoveInternal(axes, nativeTargets, mode,
        MotionCommandPathMode::EXACT_STOP, G00_overrideRatio,
        nullptr, false, nullptr, false, false, 161);
}
