#include "MotionCore.h"

void MotionCore::G32_Move(const std::vector<int>& axes,
    const std::vector<double>& nativeTargets,
    const std::vector<double>* intermediateTargets, BufferMode mode)
{
    // BASE41 refuses both legs together until batch ingress is atomic.
    // Never publish a first leg, tail, path-mode change or aborting epoch here.
    if (intermediateTargets != nullptr)
    {
        MotionCommand rejected{};
        rejected.mode = InterpolationMode::LINEAR;
        rejected.sourceLinePC = m_pendingSourcePC;
        rejected.commandPathMode = MotionCommandPathMode::EXACT_STOP;
        RejectInvalidProducerMotionCommand(rejected, GetCurrentExecutionEpoch(),
            m_pendingCommandSource.load(std::memory_order_acquire),
            GetMotionOwnerLease(), nullptr, nullptr);
        return;
    }
    (void)TryG00MoveInternal(axes, nativeTargets, mode,
        MotionCommandPathMode::EXACT_STOP, G00_overrideRatio,
        nullptr, false, nullptr, false, false, 32);
}
