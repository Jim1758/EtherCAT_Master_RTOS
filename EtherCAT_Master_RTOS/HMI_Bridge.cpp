#include "HMI_Bridge.h"
#include "SHMManager.h"
#include "GlobalConfig.h" 
#include "NCManager.h"
#include "AlarmManager.h"
#include "EtherCatPdoRuntimeInvalidCorrelation.h"
#include <cstring> 
#include "PLCManager.h"
#include "NCPLCMap.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <rtapi.h>

#if defined(_MSC_VER)
#define HMI_DIAG_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define HMI_DIAG_NOINLINE __attribute__((noinline))
#else
#define HMI_DIAG_NOINLINE
#endif

namespace HMI_Bridge
{
    namespace
    {
        const char* IdleHoldDiagnosticReasonToName(
            MotionCore::IdleHoldDiagnosticReason reason) noexcept
        {
            using Reason = MotionCore::IdleHoldDiagnosticReason;
            switch (reason)
            {
            case Reason::NONE: return "NONE";
            case Reason::GRANT_MISMATCH: return "GRANT_MISMATCH";
            case Reason::AUTHORITY_CHANGED: return "AUTHORITY_CHANGED";
            case Reason::GROUP_OR_MAPPING_CHANGED: return "GROUP_OR_MAPPING_CHANGED";
            case Reason::SERVO_OR_MODE_LOST: return "SERVO_OR_MODE_LOST";
            case Reason::AXIS_MAPPING: return "AXIS_MAPPING";
            case Reason::COMPENSATION_NONFINITE: return "COMPENSATION_NONFINITE";
            case Reason::UNSUPPORTED_SCOPE: return "UNSUPPORTED_SCOPE";
            case Reason::AXIS_CONFIG_OR_COMMAND: return "AXIS_CONFIG_OR_COMMAND";
            case Reason::VELOCITY_CONVERSION: return "VELOCITY_CONVERSION";
            case Reason::REFERENCE_OR_CONFIG_CHANGED: return "REFERENCE_OR_CONFIG_CHANGED";
            case Reason::FOLLOWING_ERROR: return "FOLLOWING_ERROR";
            case Reason::CORRECTION_NONFINITE: return "CORRECTION_NONFINITE";
            case Reason::TRAVEL_LIMIT: return "TRAVEL_LIMIT";
            default: return "UNKNOWN";
            }
        }

        HMI_DIAG_NOINLINE void DrainIdleHoldDiagnostics(MotionCore& motion)
        {
            // CJ FIX1 sole consumer, on the existing Priority-50 1000 ms task.
            // No live AxisContext/hold reads: even delayed rows retain RT values.
            // Fixed workspace is included in MotionCore's 1 KiB diagnostic budget.
            static MotionCore::IdleHoldDiagnosticEvent event{};
            static std::uint32_t lastReportedDropped = 0U;
            using EventType = MotionCore::IdleHoldDiagnosticEventType;
            for (std::size_t consumed = 0U;
                consumed < MotionCore::IDLE_HOLD_DIAGNOSTIC_DRAIN_BUDGET; ++consumed)
            {
                if (!motion.TryPopIdleHoldDiagnostic(event)) break;
                switch (event.eventType)
                {
                case EventType::ACTIVE:
                    RtPrintf("[IDLE-CJ] ACTIVE owner=%u generation=%u epoch=%u mask=%u capUMs=100 rtTick=%llu diagSeq=%llu diag=BASE48_DIAG1\n",
                        static_cast<unsigned>(event.owner), event.generation,
                        event.epoch, event.mask,
                        static_cast<unsigned long long>(event.runtimeTick),
                        static_cast<unsigned long long>(event.sequence));
                    break;
                case EventType::REFERENCE:
                    RtPrintf("[IDLE-CJ] REFERENCE axis=%u generation=%u epoch=%u cmdPulseBits=%08X%08X actPulseBits=%08X%08X windowPulseBits=%08X%08X rtTick=%llu diagSeq=%llu\n",
                        static_cast<unsigned>(event.axisIndex), event.generation, event.epoch,
                        static_cast<unsigned>(event.cmdPulseBits >> 32U),
                        static_cast<unsigned>(event.cmdPulseBits),
                        static_cast<unsigned>(event.actPulseBits >> 32U),
                        static_cast<unsigned>(event.actPulseBits),
                        static_cast<unsigned>(event.windowPulseBits >> 32U),
                        static_cast<unsigned>(event.windowPulseBits),
                        static_cast<unsigned long long>(event.runtimeTick),
                        static_cast<unsigned long long>(event.sequence));
                    break;
                case EventType::RELEASED:
                    RtPrintf("[IDLE-CJ] RELEASED owner=%u generation=%u epoch=%u nextOwner=%u nextGeneration=%u rtTick=%llu diagSeq=%llu\n",
                        static_cast<unsigned>(event.owner), event.generation, event.epoch,
                        static_cast<unsigned>(event.nextOwner), event.nextGeneration,
                        static_cast<unsigned long long>(event.runtimeTick),
                        static_cast<unsigned long long>(event.sequence));
                    break;
                case EventType::CANCELLED:
                case EventType::FAILED:
                    RtPrintf("[IDLE-CJ] %s reason=%s owner=%u generation=%u epoch=%u mask=%u axis=%d rtTick=%llu diagSeq=%llu\n",
                        event.eventType == EventType::FAILED ? "FAILED" : "CANCELLED",
                        IdleHoldDiagnosticReasonToName(event.reason),
                        static_cast<unsigned>(event.owner), event.generation,
                        event.epoch, event.mask, event.axisIndex,
                        static_cast<unsigned long long>(event.runtimeTick),
                        static_cast<unsigned long long>(event.sequence));
                    if (event.eventType == EventType::FAILED &&
                        event.reason == MotionCore::IdleHoldDiagnosticReason::FOLLOWING_ERROR)
                    {
                        RtPrintf("[IDLE-CJ] FAULT_POSITION axis=%d generation=%u epoch=%u cmdPulseBits=%08X%08X actPulseBits=%08X%08X windowPulseBits=%08X%08X rtTick=%llu diagSeq=%llu\n",
                            event.axisIndex, event.generation, event.epoch,
                            static_cast<unsigned>(event.cmdPulseBits >> 32U), static_cast<unsigned>(event.cmdPulseBits),
                            static_cast<unsigned>(event.actPulseBits >> 32U), static_cast<unsigned>(event.actPulseBits),
                            static_cast<unsigned>(event.windowPulseBits >> 32U), static_cast<unsigned>(event.windowPulseBits),
                            static_cast<unsigned long long>(event.runtimeTick), static_cast<unsigned long long>(event.sequence));
                    }
                    break;
                case EventType::FOLLOWING_ERROR_SAMPLE:
                    RtPrintf("[IDLE-CJ] FAULT_SAMPLE axis=%d generation=%u epoch=%u prevActPulseBits=%08X%08X boundPulseBits=%08X%08X previousImagePpsBits=%08X%08X rtTick=%llu diagSeq=%llu\n",
                        event.axisIndex, event.generation, event.epoch,
                        static_cast<unsigned>(event.cmdPulseBits >> 32U), static_cast<unsigned>(event.cmdPulseBits),
                        static_cast<unsigned>(event.actPulseBits >> 32U), static_cast<unsigned>(event.actPulseBits),
                        static_cast<unsigned>(event.windowPulseBits >> 32U), static_cast<unsigned>(event.windowPulseBits),
                        static_cast<unsigned long long>(event.runtimeTick), static_cast<unsigned long long>(event.sequence));
                    break;
                case EventType::FOLLOWING_ERROR_CONTROL:
                    RtPrintf("[IDLE-CJ] FAULT_CONTROL axis=%d generation=%u epoch=%u kpBits=%08X%08X unitsPerPulseBits=%08X%08X capPpsBits=%08X%08X reverse=%u rtTick=%llu diagSeq=%llu\n",
                        event.axisIndex, event.generation, event.epoch,
                        static_cast<unsigned>(event.cmdPulseBits >> 32U), static_cast<unsigned>(event.cmdPulseBits),
                        static_cast<unsigned>(event.actPulseBits >> 32U), static_cast<unsigned>(event.actPulseBits),
                        static_cast<unsigned>(event.windowPulseBits >> 32U), static_cast<unsigned>(event.windowPulseBits),
                        event.nextGeneration,
                        static_cast<unsigned long long>(event.runtimeTick), static_cast<unsigned long long>(event.sequence));
                    break;
                }
            }
            // At most one additional loss row per drain. Never reset the RT counter.
            const std::uint32_t dropped = motion.GetIdleHoldDiagnosticDroppedCount();
            if (dropped != lastReportedDropped)
            {
                RtPrintf("[IDLE-CJ] DIAG_DROPPED total=%u capacity=%u drainBudget=%u\n",
                    dropped, static_cast<unsigned>(MotionCore::IDLE_HOLD_DIAGNOSTIC_CAPACITY),
                    static_cast<unsigned>(MotionCore::IDLE_HOLD_DIAGNOSTIC_DRAIN_BUDGET));
                lastReportedDropped = dropped;
            }
        }

        long long CncP1MilliValue(double value) noexcept
        {
            if (!std::isfinite(value) || std::abs(value) > 9000000000000000.0)
                return (-9223372036854775807LL - 1LL);
            return static_cast<long long>(value * 1000.0);
        }

        const char* CncP1EventName(MotionCore::CncP1Event event) noexcept
        {
            using E = MotionCore::CncP1Event;
            switch (event)
            {
            case E::LOAD_EMPTY: return "LOAD_EMPTY";
            case E::LOAD_READY: return "LOAD_READY";
            case E::PROMOTED: return "PROMOTED";
            case E::KEEP_STOP: return "KEEP_STOP";
            case E::LEAVE: return "LEAVE";
            default: return "UNKNOWN";
            }
        }
        const char* CncP1ReasonName(MotionCore::CncP1Reason reason) noexcept
        {
            using R = MotionCore::CncP1Reason;
            switch (reason)
            {
            case R::NONE: return "NONE";
            case R::SCOPE: return "SCOPE";
            case R::AUTHORITY: return "AUTHORITY";
            case R::MAPPING: return "MAPPING";
            case R::GEOMETRY: return "GEOMETRY";
            case R::DIRECTION: return "DIRECTION";
            case R::SPEED: return "SPEED";
            case R::CURRENT_DISTANCE: return "CURRENT_DISTANCE";
            case R::NEXT_DISTANCE: return "NEXT_DISTANCE";
            case R::TERMINAL: return "TERMINAL";
            case R::OVERRIDE: return "OVERRIDE";
            case R::QUEUE_EMPTY: return "QUEUE_EMPTY";
            default: return "UNKNOWN";
            }
        }
        const char* CncFeedPlanEventName(MotionCore::CncFeedPlanEvent event) noexcept
        {
            using E = MotionCore::CncFeedPlanEvent;
            switch (event) {
            case E::LOAD: return "LOAD"; case E::EXTEND: return "EXTEND";
            case E::KEEP_PLAN: return "KEEP_PLAN"; case E::LEAVE: return "LEAVE";
            case E::BLEND_ENTER: return "BLEND_ENTER";
            case E::PREFIX_FAST: return "PREFIX_FAST";
            case E::PREFIX_BRAKE: return "PREFIX_BRAKE";
            case E::PREFIX_AUTHORED: return "PREFIX_AUTHORED";
            default: return "INVALID";
            }
        }
        const char* CncFeedPlanStopName(MotionCore::CncFeedPlanStop stop) noexcept
        {
            using S = MotionCore::CncFeedPlanStop;
            switch (stop) {
            case S::QUEUE_END: return "QUEUE_END"; case S::HORIZON: return "HORIZON";
            case S::SCOPE: return "SCOPE"; case S::AUTHORITY: return "AUTHORITY";
            case S::MAPPING: return "MAPPING"; case S::DIRECTION: return "DIRECTION";
            case S::SPEED: return "SPEED"; case S::SHORT_SEGMENT: return "SHORT_SEGMENT";
            case S::LATE: return "LATE"; default: return "INVALID";
            }
        }
        HMI_DIAG_NOINLINE void DrainCncFeedPlanDiagnostics(MotionCore& motion)
        {
            static MotionCore::CncFeedPlanDiagnostic event{};
            static std::uint32_t lastDropped = 0U;
            for (std::size_t i = 0U; i < MotionCore::CNC_FEED_PLAN_DRAIN_BUDGET; ++i)
            {
                if (!motion.TryPopCncFeedPlanDiagnostic(event)) break;
                if (event.blendMask & 1U)
                {
                    // DQ adds explicit equal/rising-F below-packet peaks; DP
                    // retains descending-F peaks. DL/DK classifications remain.
                    // Every value and tag is derived from the same RT event.
                    const char* prefixTag = event.prefixLimitPPS > event.cruiseVelocity &&
                        event.prefixLimitPPS < event.nominalVelocity[0] ?
                        (event.authoredPrefixPPS > 0.0 &&
                            event.authoredPrefixPPS == event.nominalVelocity[0] ? "CNC-DQ" : "CNC-DP") :
                        event.prefixLimitPPS > event.nominalVelocity[0] &&
                        event.prefixLimitPPS < event.authoredPrefixPPS ? "CNC-DL" : "CNC-DK";
                    RtPrintf("[%s] event=%s rtTick=%llu tickValid=%u seq=%llu epoch=%u seg=%llu pc=%d owner=%u gen=%u packetPPSm=%lld authoredPPSm=%lld selectedPPSm=%lld rawPPSm=%lld outPPSm=%lld arcPPSm=%lld leftPm=%lld\n",
                        prefixTag, CncFeedPlanEventName(event.event), static_cast<unsigned long long>(event.runtimeTick),
                        event.tickValid ? 1U : 0U, static_cast<unsigned long long>(event.sequence),
                        static_cast<unsigned>(event.identity.epoch), static_cast<unsigned long long>(event.identity.segmentId),
                        static_cast<int>(event.identity.sourceBlockId), static_cast<unsigned>(event.lease.owner),
                        static_cast<unsigned>(event.lease.generation), CncP1MilliValue(event.nominalVelocity[0]),
                        CncP1MilliValue(event.authoredPrefixPPS), CncP1MilliValue(event.prefixLimitPPS),
                        CncP1MilliValue(event.commandVelocity), CncP1MilliValue(event.outputVelocity),
                        CncP1MilliValue(event.cruiseVelocity), CncP1MilliValue(event.prefixRemainingPulse));
                    if (event.event != MotionCore::CncFeedPlanEvent::PREFIX_AUTHORED)
                        RtPrintf("[CNC-DJ] event=%s rtTick=%llu tickValid=%u seq=%llu epoch=%u seg=%llu pc=%d owner=%u gen=%u rawPPSm=%lld outPPSm=%lld arcPPSm=%lld prefixPPSm=%lld prefixPm=%lld leftPm=%lld reservePm=%lld endPPSm=%lld\n",
                            CncFeedPlanEventName(event.event), static_cast<unsigned long long>(event.runtimeTick),
                            event.tickValid ? 1U : 0U, static_cast<unsigned long long>(event.sequence),
                            static_cast<unsigned>(event.identity.epoch), static_cast<unsigned long long>(event.identity.segmentId),
                            static_cast<int>(event.identity.sourceBlockId), static_cast<unsigned>(event.lease.owner),
                            static_cast<unsigned>(event.lease.generation), CncP1MilliValue(event.commandVelocity),
                            CncP1MilliValue(event.outputVelocity), CncP1MilliValue(event.cruiseVelocity),
                            CncP1MilliValue(event.prefixLimitPPS), CncP1MilliValue(event.prefixLengthPulse),
                            CncP1MilliValue(event.prefixRemainingPulse), CncP1MilliValue(event.prefixReservePulse),
                            CncP1MilliValue(event.endVelocity));
                }
                if (event.event == MotionCore::CncFeedPlanEvent::PREFIX_FAST ||
                    event.event == MotionCore::CncFeedPlanEvent::PREFIX_BRAKE ||
                    event.event == MotionCore::CncFeedPlanEvent::PREFIX_AUTHORED) continue;
                // EE: one popped RT event is emitted as four bounded rows. Join
                // CNC-DH / PLAN / FEED / GEOM only by the same (seq, epoch, seg)
                // within one runtime capture; all values below use this event.
                // Console loss can leave partial records: missing rows/fields
                // remain unknown, never copied from a neighbouring event. A
                // DIAG_DROPPED change reports ring loss, not console completeness.
                RtPrintf("[CNC-DH] event=%s stop=%s rtTick=%llu tickValid=%u seq=%llu epoch=%u seg=%llu pc=%d owner=%u gen=%u cmdPPSm=%lld outPPSm=%lld endPPSm=%lld fromSeg=%llu\n",
                    CncFeedPlanEventName(event.event), CncFeedPlanStopName(event.stop),
                    static_cast<unsigned long long>(event.runtimeTick), event.tickValid ? 1U : 0U,
                    static_cast<unsigned long long>(event.sequence), static_cast<unsigned>(event.identity.epoch),
                    static_cast<unsigned long long>(event.identity.segmentId), static_cast<int>(event.identity.sourceBlockId),
                    static_cast<unsigned>(event.lease.owner), static_cast<unsigned>(event.lease.generation),
                    CncP1MilliValue(event.commandVelocity), CncP1MilliValue(event.outputVelocity),
                    CncP1MilliValue(event.endVelocity), static_cast<unsigned long long>(event.handoffFrom));
                RtPrintf("[CNC-DH-PLAN] seq=%llu epoch=%u seg=%llu horizon=%u lastSeg=%llu cruisePPSm=%lld remainPm=%lld horizonPm=%lld reservePm=%lld\n",
                    static_cast<unsigned long long>(event.sequence), static_cast<unsigned>(event.identity.epoch),
                    static_cast<unsigned long long>(event.identity.segmentId), event.horizon,
                    static_cast<unsigned long long>(event.lastSegment), CncP1MilliValue(event.cruiseVelocity),
                    CncP1MilliValue(event.remainingPulse), CncP1MilliValue(event.horizonPulse), CncP1MilliValue(event.reservePulse));
                RtPrintf("[CNC-DH-FEED] seq=%llu epoch=%u seg=%llu exit0PPSm=%lld exit1PPSm=%lld exit2PPSm=%lld exit3PPSm=%lld nominal0PPSm=%lld nominal1PPSm=%lld nominal2PPSm=%lld nominal3PPSm=%lld limit0PPSm=%lld limit1PPSm=%lld limit2PPSm=%lld limit3PPSm=%lld\n",
                    static_cast<unsigned long long>(event.sequence), static_cast<unsigned>(event.identity.epoch),
                    static_cast<unsigned long long>(event.identity.segmentId),
                    CncP1MilliValue(event.exitVelocity[0]), CncP1MilliValue(event.exitVelocity[1]),
                    CncP1MilliValue(event.exitVelocity[2]), CncP1MilliValue(event.exitVelocity[3]),
                    CncP1MilliValue(event.nominalVelocity[0]), CncP1MilliValue(event.nominalVelocity[1]),
                    CncP1MilliValue(event.nominalVelocity[2]), CncP1MilliValue(event.nominalVelocity[3]),
                    CncP1MilliValue(event.limitedVelocity[0]), CncP1MilliValue(event.limitedVelocity[1]),
                    CncP1MilliValue(event.limitedVelocity[2]), CncP1MilliValue(event.limitedVelocity[3]));
                RtPrintf("[CNC-DH-GEOM] seq=%llu epoch=%u seg=%llu mask=%u circleMask=%u blendMask=%u radiusPm=%lld carryPm=%lld\n",
                    static_cast<unsigned long long>(event.sequence), static_cast<unsigned>(event.identity.epoch),
                    static_cast<unsigned long long>(event.identity.segmentId), event.axisMask,
                    static_cast<unsigned>(event.circleMask), static_cast<unsigned>(event.blendMask),
                    CncP1MilliValue(event.radiusPulse), CncP1MilliValue(event.entryCarry));
            }
            const std::uint32_t dropped = motion.GetCncFeedPlanDiagnosticDroppedCount();
            if (dropped != lastDropped) {
                RtPrintf("[CNC-DH] DIAG_DROPPED total=%u capacity=%u drainBudget=%u\n", dropped,
                    static_cast<unsigned>(MotionCore::CNC_FEED_PLAN_CAPACITY),
                    static_cast<unsigned>(MotionCore::CNC_FEED_PLAN_DRAIN_BUDGET));
                lastDropped = dropped;
            }
        }

        HMI_DIAG_NOINLINE void DrainCncP1Diagnostics(MotionCore& motion)
        {
            static MotionCore::CncP1Diagnostic event{};
            static std::uint32_t lastDropped = 0U;
            for (std::size_t i = 0U; i < MotionCore::CNC_P1_DIAGNOSTIC_DRAIN_BUDGET; ++i)
            {
                if (!motion.TryPopCncP1Diagnostic(event)) break;
                RtPrintf("[CNC-DC] event=%s reason=%s rtTick=%llu tickValid=%u seq=%llu epoch=%u seg=%llu pc=%d owner=%u gen=%u mask=%u q=%u nextSeg=%llu nextPC=%d cmdPPSm=%lld endPPSm=%lld remainPm=%lld nextLenPm=%lld\n",
                    CncP1EventName(event.event), CncP1ReasonName(event.reason),
                    static_cast<unsigned long long>(event.runtimeTick), event.tickValid ? 1U : 0U,
                    static_cast<unsigned long long>(event.sequence), event.epoch,
                    static_cast<unsigned long long>(event.segment), event.sourcePC,
                    static_cast<unsigned>(event.owner), event.generation, event.axisMask, event.queueDepth,
                    static_cast<unsigned long long>(event.nextSegment), event.nextSourcePC,
                    CncP1MilliValue(event.commandVelocity), CncP1MilliValue(event.endVelocity),
                    CncP1MilliValue(event.remainingPulse), CncP1MilliValue(event.nextLengthPulse));
            }
            const std::uint32_t dropped = motion.GetCncP1DiagnosticDroppedCount();
            if (dropped != lastDropped)
            {
                RtPrintf("[CNC-DC] DIAG_DROPPED total=%u capacity=%u drainBudget=%u\n", dropped,
                    static_cast<unsigned>(MotionCore::CNC_P1_DIAGNOSTIC_CAPACITY),
                    static_cast<unsigned>(MotionCore::CNC_P1_DIAGNOSTIC_DRAIN_BUDGET));
                lastDropped = dropped;
            }
        }

        const char* NCBlockLifecycleStateToDiagnosticName(
            NCBlockLifecycleState state) noexcept
        {
            switch (state)
            {
            case NCBlockLifecycleState::DISPATCHED: return "DISPATCHED";
            case NCBlockLifecycleState::PROGRAM_COMMITTED: return "PROGRAM_COMMIT";
            case NCBlockLifecycleState::PROGRAM_ONLY_COMPLETED: return "PROGRAM_ONLY_DONE";
            case NCBlockLifecycleState::MOTION_PENDING: return "MOTION_PENDING";
            case NCBlockLifecycleState::MOTION_ACCEPTED: return "MOTION_ACCEPTED";
            case NCBlockLifecycleState::MOTION_ACTIVE: return "MOTION_ACTIVE";
            case NCBlockLifecycleState::MOTION_HELD: return "MOTION_HELD";
            case NCBlockLifecycleState::MOTION_COMPLETED: return "MOTION_DONE";
            case NCBlockLifecycleState::MOTION_REJECTED: return "MOTION_REJECTED";
            case NCBlockLifecycleState::MOTION_CANCELLED: return "MOTION_CANCELLED";
            case NCBlockLifecycleState::MOTION_ABORTED: return "MOTION_ABORTED";
            case NCBlockLifecycleState::MOTION_FAULTED: return "MOTION_FAULTED";
            case NCBlockLifecycleState::NC_DISPATCH_FAILED: return "NC_DISPATCH_FAILED";
            case NCBlockLifecycleState::TRACKING_OVERFLOW: return "TRACKING_OVERFLOW";
            case NCBlockLifecycleState::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockMotionBoundaryStateToDiagnosticName(
            NCBlockMotionBoundaryState state) noexcept
        {
            switch (state)
            {
            case NCBlockMotionBoundaryState::NOT_TRACKED: return "NOT_TRACKED";
            case NCBlockMotionBoundaryState::PENDING: return "PENDING";
            case NCBlockMotionBoundaryState::SUCCEEDED: return "SUCCEEDED";
            case NCBlockMotionBoundaryState::FAILED: return "FAILED";
            case NCBlockMotionBoundaryState::TRACKING_OVERFLOW: return "OVERFLOW";
            case NCBlockMotionBoundaryState::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockWaitKindToDiagnosticName(
            NCBlockWaitKind kind) noexcept
        {
            switch (kind)
            {
            case NCBlockWaitKind::MOTION_HANDLER: return "MOTION_CB";
            case NCBlockWaitKind::MOTION_QUEUE_DRAIN: return "MOTION_DRAIN";
            case NCBlockWaitKind::AUXILIARY_CALLBACK: return "AUX_CB";
            case NCBlockWaitKind::PROGRAM_FLOW_DRAIN: return "FLOW_DRAIN";
            case NCBlockWaitKind::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockCompletionComparisonToDiagnosticName(
            NCBlockCompletionComparison comparison) noexcept
        {
            switch (comparison)
            {
            case NCBlockCompletionComparison::AGREE_WAITING: return "AGREE_WAIT";
            case NCBlockCompletionComparison::AGREE_READY: return "AGREE_READY";
            case NCBlockCompletionComparison::LEDGER_READY_LEGACY_WAITING: return "LEDGER_EARLY";
            case NCBlockCompletionComparison::LEGACY_READY_LEDGER_PENDING: return "LEGACY_EARLY";
            case NCBlockCompletionComparison::LEDGER_FAILED_LEGACY_WAITING: return "LEDGER_FAIL_WAIT";
            case NCBlockCompletionComparison::LEGACY_READY_LEDGER_FAILED: return "RELEASE_ON_FAIL";
            case NCBlockCompletionComparison::NOT_MOTION_TRACKED: return "NOT_TRACKED";
            case NCBlockCompletionComparison::TRACKING_OVERFLOW: return "OVERFLOW";
            case NCBlockCompletionComparison::MISSING_LIFECYCLE: return "MISSING";
            case NCBlockCompletionComparison::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockCompletionGateDecisionToDiagnosticName(
            NCBlockCompletionGateDecision decision) noexcept
        {
            switch (decision)
            {
            case NCBlockCompletionGateDecision::LEGACY_BYPASS_WAIT: return "BYPASS_WAIT";
            case NCBlockCompletionGateDecision::LEGACY_BYPASS_READY: return "BYPASS_READY";
            case NCBlockCompletionGateDecision::WAIT_BOTH: return "WAIT_BOTH";
            case NCBlockCompletionGateDecision::WAIT_LEGACY: return "WAIT_LEGACY";
            case NCBlockCompletionGateDecision::BLOCK_LEDGER_PENDING: return "BLOCK_PENDING";
            case NCBlockCompletionGateDecision::RELEASE_DUAL_KEY: return "RELEASE_DUAL";
            case NCBlockCompletionGateDecision::BLOCK_LEDGER_FAILED: return "BLOCK_FAILED";
            case NCBlockCompletionGateDecision::BLOCK_TRACKING_OVERFLOW: return "BLOCK_OVERFLOW";
            case NCBlockCompletionGateDecision::BLOCK_MISSING_LIFECYCLE: return "BLOCK_MISSING";
            case NCBlockCompletionGateDecision::BLOCK_NOT_TRACKED: return "BLOCK_NOT_TRACKED";
            case NCBlockCompletionGateDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCProgramEndCauseToDiagnosticName(
            NCProgramEndCause cause) noexcept
        {
            switch (cause)
            {
            case NCProgramEndCause::NATURAL_EOF: return "EOF";
            case NCProgramEndCause::M02: return "M02";
            case NCProgramEndCause::M30: return "M30";
            case NCProgramEndCause::NONE:
            default: return "NONE";
            }
        }

        const char* NCProgramEndPhaseToDiagnosticName(
            NCProgramEndPhase phase) noexcept
        {
            switch (phase)
            {
            case NCProgramEndPhase::RUN_ACTIVE: return "RUN_ACTIVE";
            case NCProgramEndPhase::DRAINING: return "DRAINING";
            case NCProgramEndPhase::READY_TO_FINALIZE: return "READY";
            case NCProgramEndPhase::FINALIZED: return "FINALIZED";
            case NCProgramEndPhase::CANCELLED: return "CANCELLED";
            case NCProgramEndPhase::FAIL_CLOSED: return "FAIL_CLOSED";
            case NCProgramEndPhase::START_BLOCKED: return "START_BLOCKED";
            case NCProgramEndPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCProgramEndDecisionToDiagnosticName(
            NCProgramEndDecision decision) noexcept
        {
            switch (decision)
            {
            case NCProgramEndDecision::RUN_STARTED: return "RUN_STARTED";
            case NCProgramEndDecision::RUN_START_BLOCKED_DIRTY: return "START_DIRTY";
            case NCProgramEndDecision::END_REQUESTED: return "END_REQUESTED";
            case NCProgramEndDecision::WAIT_ACTIVE_BLOCKS: return "WAIT_BLOCKS";
            case NCProgramEndDecision::WAIT_AXIS_COMMAND: return "WAIT_AXISQ";
            case NCProgramEndDecision::WAIT_AXIS_RESULT: return "WAIT_AXIS_RESULT";
            case NCProgramEndDecision::WAIT_COMMAND_INGRESS: return "WAIT_INGRESS";
            case NCProgramEndDecision::WAIT_COMMAND_REPLAY: return "WAIT_REPLAY";
            case NCProgramEndDecision::WAIT_COMMAND_QUEUE: return "WAIT_CMDQ";
            case NCProgramEndDecision::WAIT_FEEDBACK_NOTICE: return "WAIT_NOTICE";
            case NCProgramEndDecision::WAIT_FEEDBACK: return "WAIT_FEEDBACK";
            case NCProgramEndDecision::WAIT_FEEDBACK_SEQUENCE: return "WAIT_FB_SEQ";
            case NCProgramEndDecision::WAIT_CALLBACK: return "WAIT_CALLBACK";
            case NCProgramEndDecision::WAIT_COMPLETION_BINDING: return "WAIT_BINDING";
            case NCProgramEndDecision::WAIT_SAFETY_REQUEST: return "WAIT_SAFETY";
            case NCProgramEndDecision::WAIT_GROUP_STANDSTILL: return "WAIT_STANDSTILL";
            case NCProgramEndDecision::WAIT_STABLE_CONFIRMATION: return "WAIT_STABLE";
            case NCProgramEndDecision::READY_TO_FINALIZE: return "READY_FINALIZE";
            case NCProgramEndDecision::FAIL_EXECUTION_EPOCH_CHANGED: return "FAIL_EPOCH";
            case NCProgramEndDecision::FAIL_OWNER_LEASE_LOST: return "FAIL_OWNER";
            case NCProgramEndDecision::FAIL_INTEGRITY_COUNTER_ADVANCED: return "FAIL_INTEGRITY";
            case NCProgramEndDecision::FINALIZED: return "FINALIZED";
            case NCProgramEndDecision::CANCELLED: return "CANCELLED";
            case NCProgramEndDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCGMBlockPostActionToDiagnosticName(
            NCGMBlockPostAction action) noexcept
        {
            switch (action)
            {
            case NCGMBlockPostAction::PROGRAM_STOP_M00: return "M00_STOP";
            case NCGMBlockPostAction::OPTIONAL_STOP_M01: return "M01_STOP";
            case NCGMBlockPostAction::CALL_M98: return "M98_CALL";
            case NCGMBlockPostAction::RETURN_M99: return "M99_RETURN";
            case NCGMBlockPostAction::PROGRAM_END_M02: return "M02_END";
            case NCGMBlockPostAction::PROGRAM_END_M30: return "M30_END";
            case NCGMBlockPostAction::NONE:
            default: return "NONE";
            }
        }

        const char* NCGMBlockTransactionPhaseToDiagnosticName(
            NCGMBlockTransactionPhase phase) noexcept
        {
            switch (phase)
            {
            case NCGMBlockTransactionPhase::WAITING: return "WAITING";
            case NCGMBlockTransactionPhase::READY_TO_FINALIZE: return "READY";
            case NCGMBlockTransactionPhase::FINALIZED: return "FINALIZED";
            case NCGMBlockTransactionPhase::CANCELLED: return "CANCELLED";
            case NCGMBlockTransactionPhase::FAILED: return "FAILED";
            case NCGMBlockTransactionPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCPreDispatchBarrierKindToDiagnosticName(
            NCPreDispatchBarrierKind kind) noexcept
        {
            switch (kind)
            {
            case NCPreDispatchBarrierKind::MACRO_EOF: return "MACRO_EOF";
            case NCPreDispatchBarrierKind::ASSIGNMENT: return "ASSIGN";
            case NCPreDispatchBarrierKind::GOTO_CONTROL: return "GOTO";
            case NCPreDispatchBarrierKind::MACRO_DEPENDENCY: return "MACRO_DEP";
            case NCPreDispatchBarrierKind::M00: return "M00";
            case NCPreDispatchBarrierKind::M01: return "M01";
            case NCPreDispatchBarrierKind::M02: return "M02";
            case NCPreDispatchBarrierKind::M30: return "M30";
            case NCPreDispatchBarrierKind::M98: return "M98";
            case NCPreDispatchBarrierKind::M99: return "M99";
            case NCPreDispatchBarrierKind::G_CODE_BARRIER: return "G_CODE";
            case NCPreDispatchBarrierKind::SINGLE_BLOCK_BARRIER: return "SINGLE_BLOCK";
            case NCPreDispatchBarrierKind::BLOCK_BARRIER: return "BLOCK";
            case NCPreDispatchBarrierKind::NONE:
            default: return "NONE";
            }
        }

        const char* NCSingleBlockCandidateKindToDiagnosticName(
            NCSingleBlockCandidateKind kind) noexcept
        {
            switch (kind)
            {
            case NCSingleBlockCandidateKind::PROGRAM_CONTROL: return "CONTROL";
            case NCSingleBlockCandidateKind::G_M_BLOCK: return "G_M";
            case NCSingleBlockCandidateKind::ADDRESS_BLOCK: return "ADDRESS";
            case NCSingleBlockCandidateKind::NONE:
            default: return "NONE";
            }
        }

        const char* NCSingleBlockShadowPhaseToDiagnosticName(
            NCSingleBlockShadowPhase phase) noexcept
        {
            switch (phase)
            {
            case NCSingleBlockShadowPhase::ARMED: return "ARMED";
            case NCSingleBlockShadowPhase::WAITING_BOUNDARY: return "WAITING";
            case NCSingleBlockShadowPhase::BOUNDARY_READY: return "READY";
            case NCSingleBlockShadowPhase::LEGACY_HOLD_CONFIRMED: return "HOLD_OK";
            case NCSingleBlockShadowPhase::LEGACY_HOLD_MISMATCH: return "HOLD_MISMATCH";
            case NCSingleBlockShadowPhase::PROGRAM_END_SUPPRESSED: return "END_SUPPRESSED";
            case NCSingleBlockShadowPhase::CONTROLLED_HOLD_CONFIRMED: return "CONTROLLED_HOLD";
            case NCSingleBlockShadowPhase::CANCELLED: return "CANCELLED";
            case NCSingleBlockShadowPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCSingleBlockShadowDecisionToDiagnosticName(
            NCSingleBlockShadowDecision decision) noexcept
        {
            switch (decision)
            {
            case NCSingleBlockShadowDecision::NOT_ELIGIBLE: return "NOT_ELIGIBLE";
            case NCSingleBlockShadowDecision::WAIT_LIFECYCLE: return "WAIT_LIFECYCLE";
            case NCSingleBlockShadowDecision::WAIT_PROGRAM_COMMIT: return "WAIT_COMMIT";
            case NCSingleBlockShadowDecision::WAIT_TRANSACTION: return "WAIT_TXN";
            case NCSingleBlockShadowDecision::WAIT_CALLBACK: return "WAIT_CALLBACK";
            case NCSingleBlockShadowDecision::WAIT_MOTION: return "WAIT_MOTION";
            case NCSingleBlockShadowDecision::READY_FOR_HOLD: return "READY_HOLD";
            case NCSingleBlockShadowDecision::AGREE_HOLD: return "AGREE_HOLD";
            case NCSingleBlockShadowDecision::LEGACY_EARLY_HOLD: return "LEGACY_EARLY";
            case NCSingleBlockShadowDecision::LEGACY_HOLD_WITHOUT_ARM: return "PHANTOM_HOLD";
            case NCSingleBlockShadowDecision::PROGRAM_END_SUPPRESSED: return "END_SUPPRESSED";
            case NCSingleBlockShadowDecision::MOTION_FAILED: return "MOTION_FAILED";
            case NCSingleBlockShadowDecision::TRACKING_OVERFLOW: return "OVERFLOW";
            case NCSingleBlockShadowDecision::TRANSACTION_FAILED: return "TXN_FAILED";
            case NCSingleBlockShadowDecision::RESUMED: return "RESUMED";
            case NCSingleBlockShadowDecision::CANCELLED: return "CANCELLED";
            case NCSingleBlockShadowDecision::CONTROLLED_HOLD_APPLIED: return "CONTROLLED_HOLD";
            case NCSingleBlockShadowDecision::CONTROLLED_HOLD_BEFORE_BOUNDARY: return "CONTROLLED_EARLY";
            case NCSingleBlockShadowDecision::CONTROLLED_RESUMED: return "CONTROLLED_RESUME";
            case NCSingleBlockShadowDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCSingleBlockHoldGatePhaseToDiagnosticName(
            NCSingleBlockHoldGatePhase phase) noexcept
        {
            switch (phase)
            {
            case NCSingleBlockHoldGatePhase::BYPASSED: return "BYPASS";
            case NCSingleBlockHoldGatePhase::ARMED: return "ARMED";
            case NCSingleBlockHoldGatePhase::WAITING_BOUNDARY: return "WAITING";
            case NCSingleBlockHoldGatePhase::HOLD_READY: return "HOLD_READY";
            case NCSingleBlockHoldGatePhase::HOLD_APPLIED: return "HOLD";
            case NCSingleBlockHoldGatePhase::RESUMED: return "RESUMED";
            case NCSingleBlockHoldGatePhase::PROGRAM_END_SUPPRESSED: return "END_SUPPRESSED";
            case NCSingleBlockHoldGatePhase::BLOCKED: return "BLOCKED";
            case NCSingleBlockHoldGatePhase::CANCELLED: return "CANCELLED";
            case NCSingleBlockHoldGatePhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCSingleBlockHoldGateDecisionToDiagnosticName(
            NCSingleBlockHoldGateDecision decision) noexcept
        {
            switch (decision)
            {
            case NCSingleBlockHoldGateDecision::LEGACY_BYPASS_DISABLED: return "BYPASS_DISABLED";
            case NCSingleBlockHoldGateDecision::LEGACY_BYPASS_EXPLICIT_STOP: return "BYPASS_EXPLICIT";
            case NCSingleBlockHoldGateDecision::CONTROL_ARMED: return "CONTROL_ARMED";
            case NCSingleBlockHoldGateDecision::WAIT_BOUNDARY: return "WAIT_BOUNDARY";
            case NCSingleBlockHoldGateDecision::READY_TO_HOLD: return "READY_HOLD";
            case NCSingleBlockHoldGateDecision::HOLD_APPLIED: return "HOLD_APPLIED";
            case NCSingleBlockHoldGateDecision::RESUME_APPLIED: return "RESUME_APPLIED";
            case NCSingleBlockHoldGateDecision::PROGRAM_END_SUPPRESSED: return "END_SUPPRESSED";
            case NCSingleBlockHoldGateDecision::BOUNDARY_MOTION_FAILED: return "MOTION_FAILED";
            case NCSingleBlockHoldGateDecision::BOUNDARY_TRANSACTION_FAILED: return "TXN_FAILED";
            case NCSingleBlockHoldGateDecision::BOUNDARY_TRACKING_OVERFLOW: return "OVERFLOW";
            case NCSingleBlockHoldGateDecision::BOUNDARY_CANCELLED: return "BOUNDARY_CANCELLED";
            case NCSingleBlockHoldGateDecision::CANCELLED: return "CANCELLED";
            case NCSingleBlockHoldGateDecision::SUPERSEDED: return "SUPERSEDED";
            case NCSingleBlockHoldGateDecision::ROLLBACK_DISABLED: return "ROLLBACK_DISABLED";
            case NCSingleBlockHoldGateDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCFeedHoldSourceToDiagnosticName(
            NCFeedHoldSource source) noexcept
        {
            switch (source)
            {
            case NCFeedHoldSource::PROGRAM: return "PROGRAM";
            case NCFeedHoldSource::HOME: return "HOME";
            case NCFeedHoldSource::NONE:
            default: return "NONE";
            }
        }

        const char* NCFeedHoldShadowPhaseToDiagnosticName(
            NCFeedHoldShadowPhase phase) noexcept
        {
            switch (phase)
            {
            case NCFeedHoldShadowPhase::REQUESTED: return "REQUESTED";
            case NCFeedHoldShadowPhase::DECELERATING: return "DECEL";
            case NCFeedHoldShadowPhase::STOPPED_UNSTABLE: return "UNSTABLE";
            case NCFeedHoldShadowPhase::ACKNOWLEDGED: return "ACK";
            case NCFeedHoldShadowPhase::RESUME_REQUESTED: return "RESUME_REQ";
            case NCFeedHoldShadowPhase::RESUMED: return "RESUMED";
            case NCFeedHoldShadowPhase::CANCELLED: return "CANCELLED";
            case NCFeedHoldShadowPhase::FAILED: return "FAILED";
            case NCFeedHoldShadowPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCFeedHoldShadowDecisionToDiagnosticName(
            NCFeedHoldShadowDecision decision) noexcept
        {
            switch (decision)
            {
            case NCFeedHoldShadowDecision::REQUEST_LATCHED: return "REQUEST_LATCHED";
            case NCFeedHoldShadowDecision::WAIT_OVERRIDE_ZERO: return "WAIT_OVERRIDE";
            case NCFeedHoldShadowDecision::WAIT_COMMAND_STOP: return "WAIT_COMMAND";
            case NCFeedHoldShadowDecision::WAIT_ACTUAL_STOP: return "WAIT_ACTUAL";
            case NCFeedHoldShadowDecision::WAIT_HOME_PAUSED: return "WAIT_HOME";
            case NCFeedHoldShadowDecision::WAIT_STABLE: return "WAIT_STABLE";
            case NCFeedHoldShadowDecision::ACK_READY: return "ACK_READY";
            case NCFeedHoldShadowDecision::LEGACY_HOLD_EARLY: return "LEGACY_EARLY";
            case NCFeedHoldShadowDecision::LEGACY_HOLD_AGREE: return "LEGACY_AGREE";
            case NCFeedHoldShadowDecision::RESUME_BEFORE_ACK: return "RESUME_EARLY";
            case NCFeedHoldShadowDecision::RESUME_AFTER_ACK: return "RESUME_AFTER_ACK";
            case NCFeedHoldShadowDecision::RESUMED: return "RESUMED";
            case NCFeedHoldShadowDecision::OWNER_CHANGED: return "OWNER_CHANGED";
            case NCFeedHoldShadowDecision::EPOCH_CHANGED: return "EPOCH_CHANGED";
            case NCFeedHoldShadowDecision::MOTION_FAULT: return "MOTION_FAULT";
            case NCFeedHoldShadowDecision::ACK_LOST: return "ACK_LOST";
            case NCFeedHoldShadowDecision::CANCELLED: return "CANCELLED";
            case NCFeedHoldShadowDecision::SUPERSEDED: return "SUPERSEDED";
            case NCFeedHoldShadowDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCFeedHoldResumeGatePhaseToDiagnosticName(
            NCFeedHoldResumeGatePhase phase) noexcept
        {
            switch (phase)
            {
            case NCFeedHoldResumeGatePhase::BYPASSED: return "BYPASS";
            case NCFeedHoldResumeGatePhase::DEFERRED: return "DEFERRED";
            case NCFeedHoldResumeGatePhase::RELEASE_READY: return "RELEASE";
            case NCFeedHoldResumeGatePhase::APPLIED: return "APPLIED";
            case NCFeedHoldResumeGatePhase::CANCELLED: return "CANCELLED";
            case NCFeedHoldResumeGatePhase::BLOCKED: return "BLOCKED";
            case NCFeedHoldResumeGatePhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCFeedHoldResumeGateDecisionToDiagnosticName(
            NCFeedHoldResumeGateDecision decision) noexcept
        {
            switch (decision)
            {
            case NCFeedHoldResumeGateDecision::LEGACY_BYPASS_DISABLED:
                return "BYPASS_DISABLED";
            case NCFeedHoldResumeGateDecision::LEGACY_BYPASS_NOT_PROGRAM_FEED_HOLD:
                return "BYPASS_OTHER_HOLD";
            case NCFeedHoldResumeGateDecision::DEFER_UNTIL_ACK:
                return "DEFER_UNTIL_ACK";
            case NCFeedHoldResumeGateDecision::APPLY_IMMEDIATE_AFTER_ACK:
                return "APPLY_AFTER_ACK";
            case NCFeedHoldResumeGateDecision::RELEASE_ON_ACK:
                return "RELEASE_ON_ACK";
            case NCFeedHoldResumeGateDecision::DUPLICATE_DEFERRED_REQUEST:
                return "DUP_DEFERRED";
            case NCFeedHoldResumeGateDecision::DUPLICATE_RELEASE_READY_REQUEST:
                return "DUP_RELEASE";
            case NCFeedHoldResumeGateDecision::BOUNDARY_FAILED:
                return "BOUNDARY_FAILED";
            case NCFeedHoldResumeGateDecision::BOUNDARY_CANCELLED:
                return "BOUNDARY_CANCELLED";
            case NCFeedHoldResumeGateDecision::RESUME_APPLIED:
                return "RESUME_APPLIED";
            case NCFeedHoldResumeGateDecision::CANCELLED:
                return "CANCELLED";
            case NCFeedHoldResumeGateDecision::SUPERSEDED:
                return "SUPERSEDED";
            case NCFeedHoldResumeGateDecision::ROLLBACK_DISABLED:
                return "ROLLBACK_DISABLED";
            case NCFeedHoldResumeGateDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCOrdinaryG00FeedHoldCohortPhaseToDiagnosticName(
            NCOrdinaryG00FeedHoldCohortPhase phase) noexcept
        {
            switch (phase)
            {
            case NCOrdinaryG00FeedHoldCohortPhase::K73_BYPASSED:
                return "BYPASS";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_CAPTURED:
                return "CAPTURED";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_HOLD_ACKNOWLEDGED:
                return "HOLD_ACK";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_REQUESTED:
                return "RESUME_REQ";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_APPLIED:
                return "RESUME_APPLIED";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_PENDING:
                return "TERM_PENDING";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_COMPLETE:
                return "TERM_COMPLETE";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_CANCELLED:
                return "CANCELLED";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_FAILED:
                return "FAILED";
            case NCOrdinaryG00FeedHoldCohortPhase::K73_IDLE:
            default:
                return "IDLE";
            }
        }

        const char* NCOrdinaryG00FeedHoldCohortDecisionToDiagnosticName(
            NCOrdinaryG00FeedHoldCohortDecision decision) noexcept
        {
            switch (decision)
            {
            case NCOrdinaryG00FeedHoldCohortDecision::K73_BYPASS_NOT_PROGRAM:
                return "BYPASS_NOT_PROGRAM";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_BYPASS_NOT_TWO_ACTIVE:
                return "BYPASS_NOT_TWO";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_CAPTURED_EXACT:
                return "CAPTURED_EXACT";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_HOLD_ACKNOWLEDGED:
                return "HOLD_ACK";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_RESUME_BEFORE_ACK:
                return "RESUME_BEFORE_ACK";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_RESUME_AFTER_ACK:
                return "RESUME_AFTER_ACK";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_RESUME_APPLIED:
                return "RESUME_APPLIED";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_WAIT_FIRST_TERMINAL:
                return "WAIT_FIRST_TERM";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_WAIT_SECOND_TERMINAL:
                return "WAIT_SECOND_TERM";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_ALL_COMPLETED_EXACT:
                return "ALL_COMPLETED";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_INTERRUPTED_TERMINAL_EXACT:
                return "INTERRUPTED_EXACT";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_BOUNDARY:
                return "INVALID_BOUNDARY";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_REGISTRY:
                return "INVALID_REGISTRY";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_MEMBER:
                return "INVALID_MEMBER";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_SESSION_MISMATCH:
                return "SESSION_MISMATCH";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_IDENTITY_CONFLICT:
                return "IDENTITY_CONFLICT";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_OWNER_MISMATCH:
                return "OWNER_MISMATCH";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_LEDGER_REJECTED:
                return "LEDGER_REJECTED";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_TERMINAL_OUT_OF_ORDER:
                return "TERM_OUT_OF_ORDER";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_DUPLICATE_TERMINAL:
                return "DUP_TERMINAL";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_BOUNDARY_MISMATCH:
                return "BOUNDARY_MISMATCH";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_GATE_MISMATCH:
                return "GATE_MISMATCH";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_CANCELLED:
                return "CANCELLED";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_SUPERSEDED:
                return "SUPERSEDED";
            case NCOrdinaryG00FeedHoldCohortDecision::K73_NONE:
            default:
                return "NONE";
            }
        }

        const char*
            NCOrdinaryG00FeedHoldCohortCutoverPhaseToDiagnosticName(
                NCOrdinaryG00FeedHoldCohortCutoverPhase phase) noexcept
        {
            switch (phase)
            {
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED:
                return "BYPASS";
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::BOUND:
                return "BOUND";
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::WAIT_TERMINAL:
                return "WAIT_TERMINAL";
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED:
                return "RELEASED";
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::LEGACY_FALLBACK:
                return "LEGACY_FALLBACK";
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::DISABLED:
                return "DISABLED";
            case NCOrdinaryG00FeedHoldCohortCutoverPhase::IDLE:
            default:
                return "IDLE";
            }
        }

        const char*
            NCOrdinaryG00FeedHoldCohortCutoverDecisionToDiagnosticName(
                NCOrdinaryG00FeedHoldCohortCutoverDecision decision) noexcept
        {
            switch (decision)
            {
            case NCOrdinaryG00FeedHoldCohortCutoverDecision::DISABLED:
                return "DISABLED";
                case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                BYPASS_NO_EXACT_COHORT:
                    return "BYPASS_NO_EXACT";
                case NCOrdinaryG00FeedHoldCohortCutoverDecision::COHORT_BOUND:
                    return "COHORT_BOUND";
                case NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_HOLD_ACK:
                    return "WAIT_HOLD_ACK";
                case NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_RESUME:
                    return "WAIT_RESUME";
                    case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    WAIT_FIRST_TERMINAL:
                        return "WAIT_FIRST_TERM";
                        case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                        WAIT_SECOND_TERMINAL:
                            return "WAIT_SECOND_TERM";
                            case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                            ALLOW_READ_AHEAD:
                                return "ALLOW_READ_AHEAD";
                                case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                                FALLBACK_INTERRUPTED:
                                    return "FALLBACK_INTERRUPTED";
                                    case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                                    FALLBACK_CANCELLED:
                                        return "FALLBACK_CANCELLED";
                                        case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                                        FALLBACK_FAILED_PROOF:
                                            return "FALLBACK_FAILED_PROOF";
                                            case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                                            FALLBACK_REGISTRY:
                                                return "FALLBACK_REGISTRY";
                                                case NCOrdinaryG00FeedHoldCohortCutoverDecision::
                                                FALLBACK_ACCOUNTING:
                                                    return "FALLBACK_ACCOUNTING";
                                                case NCOrdinaryG00FeedHoldCohortCutoverDecision::SESSION_RESET:
                                                    return "SESSION_RESET";
                                                case NCOrdinaryG00FeedHoldCohortCutoverDecision::NONE:
                                                default:
                                                    return "NONE";
            }
        }

        const char*
            NCOrdinaryG00FeedHoldCohortRearmPhaseToDiagnosticName(
                NCOrdinaryG00FeedHoldCohortRearmPhase phase) noexcept
        {
            switch (phase)
            {
            case NCOrdinaryG00FeedHoldCohortRearmPhase::K75_FIRST_BOUND:
                return "FIRST_BOUND";
            case NCOrdinaryG00FeedHoldCohortRearmPhase::K75_FIRST_RELEASED:
                return "FIRST_RELEASED";
            case NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_BOUND:
                return "SECOND_BOUND";
            case NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_RELEASED:
                return "SECOND_RELEASED";
            case NCOrdinaryG00FeedHoldCohortRearmPhase::K75_FAILED:
                return "FAILED";
            case NCOrdinaryG00FeedHoldCohortRearmPhase::K75_IDLE:
            default:
                return "IDLE";
            }
        }

        const char*
            NCOrdinaryG00FeedHoldCohortRearmDecisionToDiagnosticName(
                NCOrdinaryG00FeedHoldCohortRearmDecision decision) noexcept
        {
            switch (decision)
            {
                case NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_FIRST_GENERATION_BOUND:
                    return "FIRST_BOUND";
                    case NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_WAIT_FIRST_RELEASE:
                        return "WAIT_FIRST_RELEASE";
                        case NCOrdinaryG00FeedHoldCohortRearmDecision::
                        K75_FIRST_GENERATION_RELEASED:
                            return "FIRST_RELEASED";
                            case NCOrdinaryG00FeedHoldCohortRearmDecision::
                            K75_SECOND_GENERATION_BOUND:
                                return "SECOND_BOUND";
                                case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                K75_WAIT_SECOND_RELEASE:
                                    return "WAIT_SECOND_RELEASE";
                                case NCOrdinaryG00FeedHoldCohortRearmDecision::K75_REARM_PROVEN:
                                    return "REARM_PROVEN";
                                case NCOrdinaryG00FeedHoldCohortRearmDecision::K75_EARLY_REARM:
                                    return "EARLY_REARM";
                                    case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                    K75_SESSION_MISMATCH:
                                        return "SESSION_MISMATCH";
                                        case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                        K75_COHORT_SEQUENCE_MISMATCH:
                                            return "COHORT_SEQ_MISMATCH";
                                            case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                            K75_BOUNDARY_SEQUENCE_MISMATCH:
                                                return "BOUNDARY_SEQ_MISMATCH";
                                                case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                                K75_GATE_SEQUENCE_MISMATCH:
                                                    return "GATE_SEQ_MISMATCH";
                                                    case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                                    K75_MEMBER_IDENTITY_REUSE:
                                                        return "MEMBER_REUSE";
                                                        case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                                        K75_REGISTRY_MISMATCH:
                                                            return "REGISTRY_MISMATCH";
                                                            case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                                            K75_PROOF_MISMATCH:
                                                                return "PROOF_MISMATCH";
                                                                case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                                                K75_FALLBACK_OBSERVED:
                                                                    return "FALLBACK_OBSERVED";
                                                                    case NCOrdinaryG00FeedHoldCohortRearmDecision::
                                                                    K75_ACCOUNTING_MISMATCH:
                                                                        return "ACCOUNTING_MISMATCH";
                                                                    case NCOrdinaryG00FeedHoldCohortRearmDecision::K75_NONE:
                                                                    default:
                                                                        return "NONE";
            }
        }

        const char*
            NCOrdinaryG00FeedHoldRearmCutoverPhaseToDiagnosticName(
                NCOrdinaryG00FeedHoldRearmCutoverPhase phase) noexcept
        {
            switch (phase)
            {
            case NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_BYPASSED:
                return "BYPASSED";
            case NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_SECOND_BOUND:
                return "SECOND_BOUND";
                case NCOrdinaryG00FeedHoldRearmCutoverPhase::
                K76_WAIT_SECOND_RELEASE:
                    return "WAIT_SECOND_RELEASE";
                    case NCOrdinaryG00FeedHoldRearmCutoverPhase::
                    K76_SECOND_RELEASED:
                        return "SECOND_RELEASED";
                        case NCOrdinaryG00FeedHoldRearmCutoverPhase::
                        K76_LEGACY_FALLBACK:
                            return "LEGACY_FALLBACK";
                        case NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_DISABLED:
                            return "DISABLED";
                        case NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_IDLE:
                        default:
                            return "IDLE";
            }
        }

        const char*
            NCOrdinaryG00FeedHoldRearmCutoverDecisionToDiagnosticName(
                NCOrdinaryG00FeedHoldRearmCutoverDecision decision) noexcept
        {
            switch (decision)
            {
            case NCOrdinaryG00FeedHoldRearmCutoverDecision::K76_DISABLED:
                return "DISABLED";
                case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_BYPASS_BEFORE_SECOND_GENERATION:
                    return "BYPASS_BEFORE_SECOND";
                    case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                    K76_SECOND_GENERATION_BOUND:
                        return "SECOND_BOUND";
                        case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                        K76_WAIT_K74_RELEASE:
                            return "WAIT_K74_RELEASE";
                            case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                            K76_WAIT_K75_REARM_PROOF:
                                return "WAIT_K75_PROOF";
                                case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                K76_ALLOW_READ_AHEAD:
                                    return "ALLOW_READ_AHEAD";
                                    case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                    K76_FALLBACK_K75_FAILED:
                                        return "FALLBACK_K75_FAILED";
                                        case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                        K76_FALLBACK_K74:
                                            return "FALLBACK_K74";
                                            case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                            K76_FALLBACK_SESSION:
                                                return "FALLBACK_SESSION";
                                                case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                                K76_FALLBACK_SEQUENCE:
                                                    return "FALLBACK_SEQUENCE";
                                                    case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                                    K76_FALLBACK_IDENTITY:
                                                        return "FALLBACK_IDENTITY";
                                                        case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                                        K76_FALLBACK_REGISTRY:
                                                            return "FALLBACK_REGISTRY";
                                                            case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                                            K76_FALLBACK_ACCOUNTING:
                                                                return "FALLBACK_ACCOUNTING";
                                                                case NCOrdinaryG00FeedHoldRearmCutoverDecision::
                                                                K76_SESSION_RESET:
                                                                    return "SESSION_RESET";
                                                                case NCOrdinaryG00FeedHoldRearmCutoverDecision::K76_NONE:
                                                                default:
                                                                    return "NONE";
            }
        }

        const char* NCLifecycleInterruptionCauseToDiagnosticName(
            NCLifecycleInterruptionCause cause) noexcept
        {
            switch (cause)
            {
            case NCLifecycleInterruptionCause::RESET: return "RESET";
            case NCLifecycleInterruptionCause::ALARM: return "ALARM";
            case NCLifecycleInterruptionCause::PROGRAM_REPLACED: return "PROGRAM_REPLACE";
            case NCLifecycleInterruptionCause::MDI_REPLACED: return "MDI_REPLACE";
            case NCLifecycleInterruptionCause::MANUAL_AUTO_REPLACED: return "MANUAL_REPLACE";
            case NCLifecycleInterruptionCause::DYNAMIC_CODE_REPLACED: return "DYNAMIC_REPLACE";
            case NCLifecycleInterruptionCause::GOTO_EPOCH: return "GOTO_EPOCH";
            case NCLifecycleInterruptionCause::MOTION_REJECTED: return "MOTION_REJECT";
            case NCLifecycleInterruptionCause::MOTION_CANCELLED: return "MOTION_CANCEL";
            case NCLifecycleInterruptionCause::MOTION_ABORTED: return "MOTION_ABORT";
            case NCLifecycleInterruptionCause::MOTION_FAULTED: return "MOTION_FAULT";
            case NCLifecycleInterruptionCause::NONE:
            default: return "NONE";
            }
        }

        const char* NCLifecycleInterruptionPhaseToDiagnosticName(
            NCLifecycleInterruptionPhase phase) noexcept
        {
            switch (phase)
            {
            case NCLifecycleInterruptionPhase::REQUESTED: return "REQUESTED";
            case NCLifecycleInterruptionPhase::EPOCH_PUBLISHED: return "EPOCH_PUBLISHED";
            case NCLifecycleInterruptionPhase::DRAINING: return "DRAINING";
            case NCLifecycleInterruptionPhase::STABLE_CONFIRMATION: return "STABLE";
            case NCLifecycleInterruptionPhase::QUIESCENT: return "QUIESCENT";
            case NCLifecycleInterruptionPhase::EVIDENCE_GAP: return "EVIDENCE_GAP";
            case NCLifecycleInterruptionPhase::SUPERSEDED: return "SUPERSEDED";
            case NCLifecycleInterruptionPhase::ALARM_STOP_CLOSED: return "ALARM_STOP_CLOSED";
            case NCLifecycleInterruptionPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCLifecycleInterruptionDecisionToDiagnosticName(
            NCLifecycleInterruptionDecision decision) noexcept
        {
            switch (decision)
            {
            case NCLifecycleInterruptionDecision::REQUEST_LATCHED: return "REQUEST_LATCHED";
            case NCLifecycleInterruptionDecision::WAIT_EPOCH_PUBLICATION: return "WAIT_EPOCH";
            case NCLifecycleInterruptionDecision::EPOCH_PUBLICATION_OBSERVED: return "EPOCH_OBSERVED";
            case NCLifecycleInterruptionDecision::WAIT_ACTIVE_BLOCKS: return "WAIT_BLOCKS";
            case NCLifecycleInterruptionDecision::WAIT_AXIS_COMMAND: return "WAIT_AXISQ";
            case NCLifecycleInterruptionDecision::WAIT_AXIS_RESULT: return "WAIT_AXISR";
            case NCLifecycleInterruptionDecision::WAIT_COMMAND_INGRESS: return "WAIT_INGRESS";
            case NCLifecycleInterruptionDecision::WAIT_COMMAND_REPLAY: return "WAIT_REPLAY";
            case NCLifecycleInterruptionDecision::WAIT_COMMAND_QUEUE: return "WAIT_CMDQ";
            case NCLifecycleInterruptionDecision::WAIT_FEEDBACK_NOTICE: return "WAIT_NOTICE";
            case NCLifecycleInterruptionDecision::WAIT_FEEDBACK: return "WAIT_FEEDBACK";
            case NCLifecycleInterruptionDecision::WAIT_FEEDBACK_SEQUENCE: return "WAIT_FB_SEQ";
            case NCLifecycleInterruptionDecision::WAIT_CALLBACK: return "WAIT_CALLBACK";
            case NCLifecycleInterruptionDecision::WAIT_COMPLETION_BINDING: return "WAIT_BINDING";
            case NCLifecycleInterruptionDecision::WAIT_SAFETY_REQUEST: return "WAIT_SAFETY";
            case NCLifecycleInterruptionDecision::WAIT_GROUP_STANDSTILL: return "WAIT_STANDSTILL";
            case NCLifecycleInterruptionDecision::WAIT_STABLE_CONFIRMATION: return "WAIT_STABLE";
            case NCLifecycleInterruptionDecision::QUIESCENT_PROVED: return "QUIESCENT_PROVED";
            case NCLifecycleInterruptionDecision::EVIDENCE_FEEDBACK_OVERFLOW: return "GAP_FB_OVERFLOW";
            case NCLifecycleInterruptionDecision::EVIDENCE_NOTICE_OVERFLOW: return "GAP_NOTICE_OVERFLOW";
            case NCLifecycleInterruptionDecision::EVIDENCE_SEQUENCE_GAP: return "GAP_FB_SEQUENCE";
            case NCLifecycleInterruptionDecision::EVIDENCE_LEDGER_INTEGRITY: return "GAP_LEDGER";
            case NCLifecycleInterruptionDecision::EVIDENCE_LEDGER_REJECTED: return "GAP_LEDGER_REJECT";
            case NCLifecycleInterruptionDecision::EPOCH_SUPERSEDED: return "EPOCH_SUPERSEDED";
            case NCLifecycleInterruptionDecision::SUPERSEDED: return "SUPERSEDED";
            case NCLifecycleInterruptionDecision::WAIT_ALARM_STOP_ACKNOWLEDGEMENT: return "WAIT_ALARM_ACK";
            case NCLifecycleInterruptionDecision::WAIT_ALARM_STOP_TERMINAL: return "WAIT_ALARM_TERM";
            case NCLifecycleInterruptionDecision::WAIT_ALARM_STOP_STABLE: return "WAIT_ALARM_STABLE";
            case NCLifecycleInterruptionDecision::ALARM_STOP_CLOSED: return "ALARM_STOP_CLOSED";
            case NCLifecycleInterruptionDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCAlarmEmergencyStopTriggerToDiagnosticName(
            NCAlarmEmergencyStopTrigger trigger) noexcept
        {
            switch (trigger)
            {
            case NCAlarmEmergencyStopTrigger::EMERGENCY_STOP: return "EMERGENCY_STOP";
            case NCAlarmEmergencyStopTrigger::AXIS_PROTECTION: return "AXIS_PROTECT";
            case NCAlarmEmergencyStopTrigger::HARD_LIMIT: return "HARD_LIMIT";
            case NCAlarmEmergencyStopTrigger::DRIVE_FAULT: return "DRIVE_FAULT";
            case NCAlarmEmergencyStopTrigger::LAG_ERROR: return "LAG_ERROR";
            case NCAlarmEmergencyStopTrigger::NC_PROGRAM: return "NC_PROGRAM";
            case NCAlarmEmergencyStopTrigger::EDM_PROCESS: return "EDM_PROCESS";
            case NCAlarmEmergencyStopTrigger::SYSTEM: return "SYSTEM";
            case NCAlarmEmergencyStopTrigger::AXIS: return "AXIS";
            case NCAlarmEmergencyStopTrigger::UNKNOWN: return "UNKNOWN";
            case NCAlarmEmergencyStopTrigger::NONE:
            default: return "NONE";
            }
        }

        const char* NCAlarmEmergencyStopPhaseToDiagnosticName(
            NCAlarmEmergencyStopPhase phase) noexcept
        {
            switch (phase)
            {
            case NCAlarmEmergencyStopPhase::REQUESTED: return "REQUESTED";
            case NCAlarmEmergencyStopPhase::WAITING_RT: return "WAITING_RT";
            case NCAlarmEmergencyStopPhase::ACKNOWLEDGED: return "ACKNOWLEDGED";
            case NCAlarmEmergencyStopPhase::CLEARED: return "CLEARED";
            case NCAlarmEmergencyStopPhase::EVIDENCE_GAP: return "EVIDENCE_GAP";
            case NCAlarmEmergencyStopPhase::SUPERSEDED: return "SUPERSEDED";
            case NCAlarmEmergencyStopPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCAlarmEmergencyStopDecisionToDiagnosticName(
            NCAlarmEmergencyStopDecision decision) noexcept
        {
            switch (decision)
            {
            case NCAlarmEmergencyStopDecision::ALARM_LATCHED: return "ALARM_LATCHED";
            case NCAlarmEmergencyStopDecision::WAIT_PUBLICATION: return "WAIT_PUBLICATION";
            case NCAlarmEmergencyStopDecision::WAIT_REQUEST: return "WAIT_REQUEST";
            case NCAlarmEmergencyStopDecision::WAIT_RT_APPLY: return "WAIT_RT_APPLY";
            case NCAlarmEmergencyStopDecision::WAIT_SAFETY_OWNER: return "WAIT_SAFETY_OWNER";
            case NCAlarmEmergencyStopDecision::WAIT_EXECUTION_EPOCH: return "WAIT_EPOCH";
            case NCAlarmEmergencyStopDecision::WAIT_GROUP_STOP: return "WAIT_GROUP_STOP";
            case NCAlarmEmergencyStopDecision::WAIT_AXIS_SAFE_STATE: return "WAIT_AXIS_SAFE";
            case NCAlarmEmergencyStopDecision::WAIT_COMMAND_ZERO: return "WAIT_COMMAND_ZERO";
            case NCAlarmEmergencyStopDecision::WAIT_TARGET_SEALED: return "WAIT_TARGET_SEALED";
            case NCAlarmEmergencyStopDecision::STOP_ACKNOWLEDGED: return "STOP_ACKNOWLEDGED";
            case NCAlarmEmergencyStopDecision::ALARM_CLEARED_BEFORE_ACK: return "CLEARED_EARLY";
            case NCAlarmEmergencyStopDecision::EVIDENCE_LIFECYCLE_GAP: return "GAP_LIFECYCLE";
            case NCAlarmEmergencyStopDecision::EVIDENCE_LIFECYCLE_REPLACED: return "GAP_REPLACED";
            case NCAlarmEmergencyStopDecision::EVIDENCE_AXIS_SCOPE_CHANGED: return "GAP_AXIS_SCOPE";
            case NCAlarmEmergencyStopDecision::SUPERSEDED: return "SUPERSEDED";
            case NCAlarmEmergencyStopDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCResetReleaseGatePhaseToDiagnosticName(
            NCResetReleaseGatePhase phase) noexcept
        {
            switch (phase)
            {
            case NCResetReleaseGatePhase::ARMED: return "ARMED";
            case NCResetReleaseGatePhase::WAITING_QUIESCENCE: return "WAIT_QUIET";
            case NCResetReleaseGatePhase::RELEASE_READY: return "RELEASE_READY";
            case NCResetReleaseGatePhase::RELEASED: return "RELEASED";
            case NCResetReleaseGatePhase::BLOCKED: return "BLOCKED";
            case NCResetReleaseGatePhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCResetReleaseGateDecisionToDiagnosticName(
            NCResetReleaseGateDecision decision) noexcept
        {
            switch (decision)
            {
            case NCResetReleaseGateDecision::CONTROL_ARMED: return "CONTROL_ARMED";
            case NCResetReleaseGateDecision::WAIT_QUIESCENCE_PROOF: return "WAIT_QUIET_PROOF";
            case NCResetReleaseGateDecision::READY_TO_RELEASE: return "READY_TO_RELEASE";
            case NCResetReleaseGateDecision::RELEASE_APPLIED: return "RELEASE_APPLIED";
            case NCResetReleaseGateDecision::INVALID_BOUNDARY: return "INVALID_BOUNDARY";
            case NCResetReleaseGateDecision::BOUNDARY_SEQUENCE_MISMATCH: return "BOUNDARY_SEQ_MISMATCH";
            case NCResetReleaseGateDecision::BOUNDARY_CAUSE_MISMATCH: return "BOUNDARY_CAUSE_MISMATCH";
            case NCResetReleaseGateDecision::BOUNDARY_EVIDENCE_GAP: return "BOUNDARY_EVIDENCE_GAP";
            case NCResetReleaseGateDecision::BOUNDARY_SUPERSEDED: return "BOUNDARY_SUPERSEDED";
            case NCResetReleaseGateDecision::BOUNDARY_INCOMPLETE: return "BOUNDARY_INCOMPLETE";
            case NCResetReleaseGateDecision::EPOCH_MISMATCH: return "EPOCH_MISMATCH";
            case NCResetReleaseGateDecision::SAFETY_LEASE_INVALID: return "SAFETY_LEASE_INVALID";
            case NCResetReleaseGateDecision::SAFETY_LEASE_LOST: return "SAFETY_LEASE_LOST";
            case NCResetReleaseGateDecision::OWNER_RELEASE_FAILED: return "OWNER_RELEASE_FAILED";
            case NCResetReleaseGateDecision::POST_INTERRUPTION_DISPATCH: return "POST_RESET_DISPATCH";
            case NCResetReleaseGateDecision::WAIT_REBASE_ACK: return "WAIT_REBASE_ACK";
            case NCResetReleaseGateDecision::ACK_MISMATCH: return "ACK_MISMATCH";
            case NCResetReleaseGateDecision::REBASE_FAILED: return "REBASE_FAILED";
            case NCResetReleaseGateDecision::NONE:
            default: return "NONE";
            }
        }

        const char* MotionNCSettleProfileToDiagnosticName(
            MotionNCSettleProfile profile) noexcept
        {
            switch (profile)
            {
            case MotionNCSettleProfile::GROUP_COMPLETION: return "GROUP";
            case MotionNCSettleProfile::FEED_HOLD_GROUP: return "FEED_HOLD";
            case MotionNCSettleProfile::RESET_ALL: return "RESET_ALL";
            case MotionNCSettleProfile::COUNT:
            default: return "UNKNOWN";
            }
        }

        const char* MotionNCSettleBlockerToDiagnosticName(
            MotionNCSettleBlocker blocker) noexcept
        {
            switch (blocker)
            {
            case MotionNCSettleBlocker::RUNTIME_NOT_OBSERVED: return "NO_RUNTIME";
            case MotionNCSettleBlocker::RUNTIME_INVALID: return "RUNTIME_INVALID";
            case MotionNCSettleBlocker::RUNTIME_GAP: return "RUNTIME_GAP";
            case MotionNCSettleBlocker::REQUEST_MISSING: return "NO_REQUEST";
            case MotionNCSettleBlocker::EXECUTION_EPOCH_MISMATCH: return "EPOCH";
            case MotionNCSettleBlocker::OWNER_LEASE_MISMATCH: return "OWNER";
            case MotionNCSettleBlocker::SCOPE_CHANGED: return "SCOPE";
            case MotionNCSettleBlocker::GROUP_ACTIVE: return "GROUP_ACTIVE";
            case MotionNCSettleBlocker::COMMAND_QUEUE: return "COMMAND_QUEUE";
            case MotionNCSettleBlocker::FEED_OVERRIDE_NONZERO: return "FEED_OVERRIDE";
            case MotionNCSettleBlocker::SAFETY_OR_RECOVERY_PENDING: return "SAFETY_PENDING";
            case MotionNCSettleBlocker::AXIS_FAULT_OR_ESTOP: return "FAULT_ESTOP";
            case MotionNCSettleBlocker::AXIS_SERVO_OFF: return "SERVO_OFF";
            case MotionNCSettleBlocker::AXIS_STATE: return "AXIS_STATE";
            case MotionNCSettleBlocker::COMMAND_VELOCITY: return "CMD_VELOCITY";
            case MotionNCSettleBlocker::COMMAND_POSITION_CHANGED: return "CMD_POSITION";
            case MotionNCSettleBlocker::IN_POSITION_WINDOW_INVALID: return "WINDOW_INVALID";
            case MotionNCSettleBlocker::FOLLOWING_ERROR: return "FOLLOWING_ERROR";
            case MotionNCSettleBlocker::ACTUAL_EXCURSION: return "ACT_EXCURSION";
            case MotionNCSettleBlocker::RESET_UNSUPPORTED: return "RESET_UNSUPPORTED";
            case MotionNCSettleBlocker::COMPENSATION_ACTIVE: return "COMPENSATION";
            case MotionNCSettleBlocker::PATH_RUNTIME_UNSUPPORTED: return "PATH_RUNTIME";
            case MotionNCSettleBlocker::REBASE_IN_PROGRESS: return "REBASE_ACTIVE";
            case MotionNCSettleBlocker::REBASE_VERIFY_FAILED: return "REBASE_VERIFY";
            case MotionNCSettleBlocker::PUBLICATION_BUSY: return "PUBLICATION_BUSY";
            case MotionNCSettleBlocker::NONE:
            default: return "NONE";
            }
        }

        const char* MotionNCResetRebasePhaseToDiagnosticName(
            MotionNCResetRebasePhase phase) noexcept
        {
            switch (phase)
            {
            case MotionNCResetRebasePhase::WAIT_PREPROOF: return "WAIT_PREPROOF";
            case MotionNCResetRebasePhase::CLEARING_BUFFERS: return "CLEARING";
            case MotionNCResetRebasePhase::WAIT_POSTPROOF: return "WAIT_POSTPROOF";
            case MotionNCResetRebasePhase::ACKNOWLEDGED: return "ACKNOWLEDGED";
            case MotionNCResetRebasePhase::BLOCKED: return "BLOCKED";
            case MotionNCResetRebasePhase::SUPERSEDED: return "SUPERSEDED";
            case MotionNCResetRebasePhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* MotionFeedbackTypeToDiagnosticName(
            MotionFeedbackType type) noexcept
        {
            switch (type)
            {
            case MotionFeedbackType::ACCEPTED: return "ACCEPTED";
            case MotionFeedbackType::REJECTED: return "REJECTED";
            case MotionFeedbackType::STARTED: return "STARTED";
            case MotionFeedbackType::PROGRESS: return "PROGRESS";
            case MotionFeedbackType::HELD: return "HELD";
            case MotionFeedbackType::RESUMED: return "RESUMED";
            case MotionFeedbackType::COMPLETED: return "COMPLETED";
            case MotionFeedbackType::CANCELLED: return "CANCELLED";
            case MotionFeedbackType::FAULTED: return "FAULTED";
            case MotionFeedbackType::ABORTED: return "ABORTED";
            case MotionFeedbackType::NONE:
            default: return "NONE";
            }
        }

        const char* MotionStopSettleBlockerToDiagnosticName(
            MotionStopSettlePrimaryBlocker blocker) noexcept
        {
            switch (blocker)
            {
            case MotionStopSettlePrimaryBlocker::GROUP_ACTIVE: return "GROUP_ACTIVE";
            case MotionStopSettlePrimaryBlocker::COMMAND_QUEUE: return "COMMAND_QUEUE";
            case MotionStopSettlePrimaryBlocker::AXIS_NOT_IDLE: return "AXIS_STATE";
            case MotionStopSettlePrimaryBlocker::AXIS_COMMAND_VELOCITY: return "CMD_VELOCITY";
            case MotionStopSettlePrimaryBlocker::AXIS_ACTUAL_VELOCITY: return "ACT_VELOCITY";
            case MotionStopSettlePrimaryBlocker::NONE:
            default: return "NONE";
            }
        }

        const char* MotionStateToDiagnosticName(
            MotionState state) noexcept
        {
            switch (state)
            {
            case MotionState::MotionState_MOVING: return "MOVING";
            case MotionState::MotionState_STOPPING: return "STOPPING";
            case MotionState::MotionState_ERROR: return "ERROR";
            case MotionState::MotionState_VELOCITY: return "VELOCITY";
            case MotionState::MotionState_INTERPOLATING: return "INTERPOLATING";
            case MotionState::MotionState_ESTOP: return "ESTOP";
            case MotionState::MotionState_MPG: return "MPG";
            case MotionState::MotionState_IDLE:
            default: return "IDLE";
            }
        }

        std::uint64_t ScaleNonNegativeDiagnosticValue(
            double value,
            double scale) noexcept
        {
            if (!std::isfinite(value) ||
                !std::isfinite(scale) ||
                value <= 0.0 ||
                scale <= 0.0)
            {
                return 0ULL;
            }

            const double scaled = value * scale;
            const double maximum =
                static_cast<double>(
                    (std::numeric_limits<std::uint64_t>::max)());

            if (!std::isfinite(scaled) || scaled >= maximum)
            {
                return
                    (std::numeric_limits<std::uint64_t>::max)();
            }

            return
                static_cast<std::uint64_t>(scaled + 0.5);
        }

        const char* MotionOwnerToDiagnosticName(
            MotionOwner owner) noexcept
        {
            switch (owner)
            {
            case MotionOwner::NONE:        return "NONE";
            case MotionOwner::AUTO:        return "AUTO";
            case MotionOwner::MDI:         return "MDI";
            case MotionOwner::MANUAL_AUTO: return "MANUAL_AUTO";
            case MotionOwner::JOG:         return "JOG";
            case MotionOwner::MPG:         return "MPG";
            case MotionOwner::HOME:        return "HOME";
            case MotionOwner::EDM_PATH:    return "EDM_PATH";
            case MotionOwner::EDM_RETRACT: return "EDM_RETRACT";
            case MotionOwner::RECOVERY:    return "RECOVERY";
            case MotionOwner::SAFETY:      return "SAFETY";
            case MotionOwner::IDLE_HOLD:   return "IDLE_HOLD";
            default:                       return "UNKNOWN";
            }
        }

        const char* PdoRuntimeInvalidEventToDiagnosticName(
            EtherCatPdoRuntimeInvalidEventKind eventKind) noexcept
        {
            switch (eventKind)
            {
            case EtherCatPdoRuntimeInvalidEventKind::NONE:
                return "NONE";
            case EtherCatPdoRuntimeInvalidEventKind::INVALID_CYCLE:
                return "INVALID";
                case EtherCatPdoRuntimeInvalidEventKind::
                RECOVERED_AFTER_INVALID:
                    return "RECOVERED";
                case EtherCatPdoRuntimeInvalidEventKind::VALID_TICK_GAP:
                    return "VALID_TICK_GAP";
                    case EtherCatPdoRuntimeInvalidEventKind::
                    VALIDITY_CONTRACT_MISMATCH:
                        return "CONTRACT_MISMATCH";
                        case EtherCatPdoRuntimeInvalidEventKind::
                        VALIDITY_CONTRACT_RESTORED:
                            return "CONTRACT_RESTORED";
                        default:
                            return "UNKNOWN";
            }
        }

        const char* PdoRuntimeInvalidReasonToDiagnosticName(
            std::uint32_t reasonMask) noexcept
        {
            const bool negative =
                (reasonMask &
                    ECAT_PDO_INVALID_REASON_LRW_CALL_NEGATIVE) != 0U;
            const bool mismatch =
                (reasonMask &
                    ECAT_PDO_INVALID_REASON_LRW_WKC_MISMATCH) != 0U;
            const bool dcInvalid =
                (reasonMask &
                    ECAT_PDO_INVALID_REASON_DC_WKC_INVALID) != 0U;
            const bool contractMismatch =
                (reasonMask &
                    ECAT_PDO_INVALID_REASON_VALIDITY_CONTRACT_MISMATCH) != 0U;

            if (contractMismatch && !negative && !mismatch && !dcInvalid)
            {
                return "VALIDITY_CONTRACT_MISMATCH";
            }
            if (negative && dcInvalid)
            {
                return "COMBINED_NEGATIVE+DC_WKC_INVALID";
            }
            if (mismatch && dcInvalid)
            {
                return "LRW_WKC_MISMATCH+DC_WKC_INVALID";
            }
            if (negative)
            {
                return "COMBINED_NEGATIVE";
            }
            if (mismatch)
            {
                return "LRW_WKC_MISMATCH";
            }
            if (dcInvalid)
            {
                return "DC_WKC_INVALID";
            }
            return reasonMask == ECAT_PDO_INVALID_REASON_NONE
                ? "NONE"
                : "UNKNOWN_MASK";
        }

        std::uint64_t FoldDiagnosticEventToken(
            std::uint64_t token,
            std::uint64_t value) noexcept
        {
            // Small allocation-free FNV-1a fold.  This is only an HMI
            // transition detector; it is not a lifecycle identity.
            token ^= value;
            return token * 1099511628211ULL;
        }

        std::uint64_t SubtractDiagnosticCounterFloor(
            std::uint64_t raw,
            std::uint64_t expected) noexcept
        {
            return raw > expected ? raw - expected : 0ULL;
        }

        std::uint64_t AddDiagnosticCounterSaturating(
            std::uint64_t lhs,
            std::uint64_t rhs) noexcept
        {
            const std::uint64_t maximum =
                (std::numeric_limits<std::uint64_t>::max)();
            return rhs > maximum - lhs ? maximum : lhs + rhs;
        }

        HMI_DIAG_NOINLINE std::uint64_t BuildNCSettleDiagnosticEventToken(
            bool coherent,
            const MotionNCSettleSnapshot& snapshot,
            const MotionNCSettleCounters& counters) noexcept
        {
            std::uint64_t token = 1469598103934665603ULL;
            token = FoldDiagnosticEventToken(token, coherent ? 1ULL : 0ULL);
            token = FoldDiagnosticEventToken(
                token,
                static_cast<std::uint64_t>(snapshot.profile));
            token = FoldDiagnosticEventToken(token, snapshot.requestSequence);
            token = FoldDiagnosticEventToken(token, snapshot.proofSequence);
            token = FoldDiagnosticEventToken(
                token,
                static_cast<std::uint64_t>(snapshot.blocker));
            token = FoldDiagnosticEventToken(
                token,
                static_cast<std::uint64_t>(
                    static_cast<std::int64_t>(snapshot.blockerAxisIndex)));
            token = FoldDiagnosticEventToken(token, snapshot.executionEpoch);
            token = FoldDiagnosticEventToken(
                token,
                static_cast<std::uint64_t>(snapshot.owner));
            token = FoldDiagnosticEventToken(token, snapshot.ownerGeneration);
            token = FoldDiagnosticEventToken(token, snapshot.scopeMask);

            std::uint64_t stateBits = 0ULL;
            stateBits |= snapshot.runtimeObserved ? (1ULL << 0U) : 0ULL;
            stateBits |= snapshot.runtimeCycleValid ? (1ULL << 1U) : 0ULL;
            stateBits |= snapshot.runtimeCycleContiguous ? (1ULL << 2U) : 0ULL;
            stateBits |= snapshot.requestAccepted ? (1ULL << 3U) : 0ULL;
            stateBits |= snapshot.groupDrained ? (1ULL << 4U) : 0ULL;
            stateBits |= snapshot.candidate ? (1ULL << 5U) : 0ULL;
            stateBits |= snapshot.settled ? (1ULL << 6U) : 0ULL;
            stateBits |= snapshot.rebasePreProofPassed ? (1ULL << 7U) : 0ULL;
            stateBits |= snapshot.rebasePostProofPassed ? (1ULL << 8U) : 0ULL;
            token = FoldDiagnosticEventToken(token, stateBits);

            // Do not fold sampleSequence, dwellCycles, raw velocity, PDO,
            // candidate sample counts, or any counter that advances each
            // healthy 250 us cycle.  Those values remain printable evidence
            // but can never create a one-line-per-second idle wash.
            token = FoldDiagnosticEventToken(token, counters.proofRiseCount);
            token = FoldDiagnosticEventToken(token, counters.proofRevokeCount);
            token = FoldDiagnosticEventToken(
                token,
                counters.invalidRuntimeCycleCount);
            token = FoldDiagnosticEventToken(token, counters.runtimeGapCount);
            token = FoldDiagnosticEventToken(token, counters.requestRejectCount);
            token = FoldDiagnosticEventToken(
                token,
                counters.publicationReadFailureCount);
            return token;
        }

        HMI_DIAG_NOINLINE void PrintNCSettleDiagnostic(
            bool coherent,
            const MotionNCSettleSnapshot& snapshot,
            const MotionNCSettleCounters& counters) noexcept
        {
            const std::uint64_t commandPps =
                ScaleNonNegativeDiagnosticValue(
                    snapshot.maxCommandVelocityAbsPps,
                    1.0);
            const std::uint64_t followingErrorX1000 =
                ScaleNonNegativeDiagnosticValue(
                    snapshot.worstFollowingErrorAbsPulse,
                    1000.0);
            const std::uint64_t followingWindowX1000 =
                ScaleNonNegativeDiagnosticValue(
                    snapshot.worstFollowingWindowPulse,
                    1000.0);
            const std::uint64_t excursionX1000 =
                ScaleNonNegativeDiagnosticValue(
                    snapshot.worstActualExcursionPulse,
                    1000.0);
            const std::uint64_t excursionLimitX1000 =
                ScaleNonNegativeDiagnosticValue(
                    snapshot.worstActualExcursionLimitPulse,
                    1000.0);
            const std::uint64_t advisoryActualPps =
                ScaleNonNegativeDiagnosticValue(
                    snapshot.advisoryMaxActualVelocityAbsPps,
                    1.0);

            RtPrintf(
                "[NC02J5-SET] P:%s Coh:%u Pub:%llu Req:%llu Proof:%llu "
                "Epoch:%u Owner:%s/%u Mask:%u Obs:%u Valid:%u Contig:%u ",
                MotionNCSettleProfileToDiagnosticName(snapshot.profile),
                coherent ? 1U : 0U,
                static_cast<unsigned long long>(
                    snapshot.publicationGeneration),
                static_cast<unsigned long long>(snapshot.requestSequence),
                static_cast<unsigned long long>(snapshot.proofSequence),
                static_cast<unsigned int>(snapshot.executionEpoch),
                MotionOwnerToDiagnosticName(snapshot.owner),
                static_cast<unsigned int>(snapshot.ownerGeneration),
                static_cast<unsigned int>(snapshot.scopeMask),
                snapshot.runtimeObserved ? 1U : 0U,
                snapshot.runtimeCycleValid ? 1U : 0U,
                snapshot.runtimeCycleContiguous ? 1U : 0U);
            RtPrintf(
                "Accept:%u Drain:%u Cand:%u Set:%u Pre:%u Post:%u "
                "Block:%s Ax:%d Dwell:%u/%u\n",
                snapshot.requestAccepted ? 1U : 0U,
                snapshot.groupDrained ? 1U : 0U,
                snapshot.candidate ? 1U : 0U,
                snapshot.settled ? 1U : 0U,
                snapshot.rebasePreProofPassed ? 1U : 0U,
                snapshot.rebasePostProofPassed ? 1U : 0U,
                MotionNCSettleBlockerToDiagnosticName(snapshot.blocker),
                snapshot.blockerAxisIndex,
                static_cast<unsigned int>(snapshot.dwellCycles),
                static_cast<unsigned int>(snapshot.requiredCycles));

            RtPrintf(
                "[NC02J5-SET] P:%s Ev CmdPps:%llu CmdAx:%d "
                "FollowX1000:%llu/%llu FollowAx:%d "
                "ExcX1000:%llu/%llu ExcAx:%d RawPps:%llu PDO:%u ",
                MotionNCSettleProfileToDiagnosticName(snapshot.profile),
                static_cast<unsigned long long>(commandPps),
                snapshot.worstCommandVelocityAxisIndex,
                static_cast<unsigned long long>(followingErrorX1000),
                static_cast<unsigned long long>(followingWindowX1000),
                snapshot.worstFollowingErrorAxisIndex,
                static_cast<unsigned long long>(excursionX1000),
                static_cast<unsigned long long>(excursionLimitX1000),
                snapshot.worstActualExcursionAxisIndex,
                static_cast<unsigned long long>(advisoryActualPps),
                static_cast<unsigned int>(
                    snapshot.advisoryMaxPdoTargetVelocityAbs));
            RtPrintf(
                "Start:%llu Reset:%llu Rise:%llu Revoke:%llu "
                "Invalid:%llu Gap:%llu IdReset:%llu ScopeReset:%llu "
                "Reject:%llu ReadFail:%llu\n",
                static_cast<unsigned long long>(
                    counters.candidateStartCount),
                static_cast<unsigned long long>(
                    counters.candidateResetCount),
                static_cast<unsigned long long>(counters.proofRiseCount),
                static_cast<unsigned long long>(counters.proofRevokeCount),
                static_cast<unsigned long long>(
                    counters.invalidRuntimeCycleCount),
                static_cast<unsigned long long>(counters.runtimeGapCount),
                static_cast<unsigned long long>(counters.identityResetCount),
                static_cast<unsigned long long>(counters.scopeResetCount),
                static_cast<unsigned long long>(counters.requestRejectCount),
                static_cast<unsigned long long>(
                    counters.publicationReadFailureCount));
        }

        // NC-0.2K.7.8.2: fixed Priority-50 diagnostic workspace.
        // Single caller, single writer, process lifetime, non-reentrant.
        struct Hmi1000msDiagnosticWorkspace
        {
            EtherCatPdoRuntimeInvalidCorrelationSnapshot pdoInvalidCorrelation{};
            EtherCatPdoSafetyStopCauseSnapshot pdoSafetyStopCause{};
            MotionStartupLagArmingEvidence startupLagArming{};
            MotionP1HandoverSafetySnapshot p1HandoverSafety{};
            MotionLifecycleCommitReservationSnapshot lifecycleCommit{};
            MotionCommandPathModeTransportSnapshot commandPathModeTransport{};
            MotionQueueTailTransactionSnapshot queueTailTransaction{};
            MotionStopSettleSnapshot stopSettleSnapshot{};
            MotionStopSettleCounters stopSettleCounters{};
            MotionNCSettleSnapshot groupNCSettleSnapshot{};
            MotionNCSettleCounters groupNCSettleCounters{};
            MotionNCSettleSnapshot feedHoldNCSettleSnapshot{};
            MotionNCSettleCounters feedHoldNCSettleCounters{};
            MotionNCSettleSnapshot resetNCSettleSnapshot{};
            MotionNCSettleCounters resetNCSettleCounters{};
            MotionNCResetRebaseAck resetRebaseAck{};
            MotionOwnerLease ownerLease{};
            NCBlockLifecycleSnapshot blockLifecycle{};
            NCBlockLifecycleCounters blockCounters{};
            NCBlockCompletionBoundarySnapshot completionBoundary{};
            NCBlockCompletionBoundaryCounters completionCounters{};
            NCProgramEndGateSnapshot programEndSnapshot{};
            NCProgramEndGateCounters programEndCounters{};
            NCGMBlockTransactionSnapshot gmTransactionSnapshot{};
            NCGMBlockTransactionCounters gmTransactionCounters{};
            NCPreDispatchBarrierSnapshot preDispatchBarrierSnapshot{};
            NCPreDispatchBarrierCounters preDispatchBarrierCounters{};
            NCPreparedBlockQueueSnapshot preparedQueueSnapshot{};
            NCPreparedBlockQueueCounters preparedQueueCounters{};
            NCPreparedHeadEquivalenceSnapshot preparedEquivalenceSnapshot{};
            NCPreparedHeadEquivalenceCounters preparedEquivalenceCounters{};
            NCPreparedHeadCutoverSnapshot preparedCutoverSnapshot{};
            NCPreparedHeadCutoverCounters preparedCutoverCounters{};
            NCPreparedPreResolveAdmissionSnapshot preparedPreResolveSnapshot{};
            NCPreparedPreResolveAdmissionCounters preparedPreResolveCounters{};
            NCPreparedResolverBypassSnapshot preparedResolverBypassSnapshot{};
            NCPreparedResolverBypassCounters preparedResolverBypassCounters{};
            NCOrdinaryG00AdmissionSnapshot ordinaryG00AdmissionSnapshot{};
            NCOrdinaryG00AdmissionCounters ordinaryG00AdmissionCounters{};
            NCOrdinaryG00InflightRegistrySnapshot ordinaryG00InflightRegistrySnapshot{};
            NCOrdinaryG00InflightRegistryCounters ordinaryG00InflightRegistryCounters{};
            NCOrdinaryG00ReadAheadSnapshot ordinaryG00ReadAheadSnapshot{};
            NCOrdinaryG00ReadAheadCounters ordinaryG00ReadAheadCounters{};
            NCOrdinaryG00FeedHoldCohortSnapshot ordinaryG00FeedHoldCohortSnapshot{};
            NCOrdinaryG00FeedHoldCohortCounters ordinaryG00FeedHoldCohortCounters{};
            NCOrdinaryG00FeedHoldCohortCutoverSnapshot ordinaryG00FeedHoldCohortCutoverSnapshot{};
            NCOrdinaryG00FeedHoldCohortCutoverCounters ordinaryG00FeedHoldCohortCutoverCounters{};
            NCOrdinaryG00FeedHoldCohortRearmSnapshot ordinaryG00FeedHoldCohortRearmSnapshot{};
            NCOrdinaryG00FeedHoldCohortRearmCounters ordinaryG00FeedHoldCohortRearmCounters{};
            NCOrdinaryG00FeedHoldRearmCutoverSnapshot ordinaryG00FeedHoldRearmCutoverSnapshot{};
            NCOrdinaryG00FeedHoldRearmCutoverCounters ordinaryG00FeedHoldRearmCutoverCounters{};
            NCOrdinaryG00FeedHoldRollingRearmSnapshot ordinaryG00FeedHoldRollingRearmSnapshot{};
            NCOrdinaryG00FeedHoldRollingRearmCounters ordinaryG00FeedHoldRollingRearmCounters{};
            NCOrdinaryG00FeedHoldRollingCutoverSnapshot ordinaryG00FeedHoldRollingCutoverSnapshot{};
            NCOrdinaryG00FeedHoldRollingCutoverCounters ordinaryG00FeedHoldRollingCutoverCounters{};
            NCPreparedBlockEntrySnapshot preparedQueueHead{};
            NCPreparedBlockEntrySnapshot preparedQueueTail{};
            NCSingleBlockShadowSnapshot singleBlockShadowSnapshot{};
            NCSingleBlockShadowCounters singleBlockShadowCounters{};
            NCSingleBlockHoldGateSnapshot singleBlockGateSnapshot{};
            NCSingleBlockHoldGateCounters singleBlockGateCounters{};
            NCFeedHoldBoundarySnapshot feedHoldSnapshot{};
            NCFeedHoldBoundaryCounters feedHoldCounters{};
            NCFeedHoldResumeGateSnapshot feedHoldGateSnapshot{};
            NCFeedHoldResumeGateCounters feedHoldGateCounters{};
            NCLifecycleInterruptionSnapshot lifecycleInterruptionSnapshot{};
            NCLifecycleInterruptionCounters lifecycleInterruptionCounters{};
            NCResetReleaseGateSnapshot resetReleaseGateSnapshot{};
            NCResetReleaseGateCounters resetReleaseGateCounters{};
            NCAlarmEmergencyStopSnapshot alarmEmergencyStopSnapshot{};
            NCAlarmEmergencyStopCounters alarmEmergencyStopCounters{};
            volatile NCPathCoreInputHandoffCompactState
                pathCoreInputHandoffCompactState{
                    NCPathCoreInputHandoffCompactState::NOT_RUNNING };
            volatile std::uint32_t pathCoreInputHandoffChangeToken{};
            volatile bool pathCoreInputHandoffChanged{};
            bool pdoInvalidCorrelationCoherent{};
            bool pdoSafetyStopCauseCoherent{};
            bool groupNCSettleCoherent{};
            bool feedHoldNCSettleCoherent{};
            bool resetNCSettleCoherent{};
            bool hasBlockLifecycle{};
            bool hasPreparedQueueHead{};
            bool hasPreparedQueueTail{};
        };

        HMI_DIAG_NOINLINE bool PrintGapActiveCompactDiagnosticSameThread(
            NCManager& nc, MotionCore& motion,
            Hmi1000msDiagnosticWorkspace& workspace) noexcept
        {
            if (!nc.IsGapPathSimulationActiveSameThread()) return false;
            workspace.feedHoldNCSettleSnapshot = {};
            workspace.feedHoldNCSettleCounters = {};
            const bool coherent = motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::FEED_HOLD_GROUP,
                workspace.feedHoldNCSettleSnapshot,
                workspace.feedHoldNCSettleCounters);
            const MotionNCSettleSnapshot& snapshot = workspace.feedHoldNCSettleSnapshot;
            RtPrintf("[GAP-CO-FIX1-J5] Coh:%u Req:%llu Proof:%llu Epoch:%u Owner:%u/%u "
                "Mask:%u Valid:%u Contig:%u Set:%u Dwell:%u/%u rtTick:%llu bulkDeferred=1\n",
                coherent ? 1U : 0U,
                static_cast<unsigned long long>(coherent ? snapshot.requestSequence : 0ULL),
                static_cast<unsigned long long>(coherent ? snapshot.proofSequence : 0ULL),
                static_cast<unsigned>(coherent ? snapshot.executionEpoch : 0U),
                coherent ? static_cast<unsigned>(snapshot.owner) : 0U,
                static_cast<unsigned>(coherent ? snapshot.ownerGeneration : 0U),
                static_cast<unsigned>(coherent ? snapshot.scopeMask : 0U),
                coherent && snapshot.runtimeCycleValid ? 1U : 0U,
                coherent && snapshot.runtimeCycleContiguous ? 1U : 0U,
                coherent && snapshot.settled ? 1U : 0U,
                static_cast<unsigned>(coherent ? snapshot.dwellCycles : 0U),
                static_cast<unsigned>(coherent ? snapshot.requiredCycles : 0U),
                static_cast<unsigned long long>(coherent ? snapshot.runtimeCycleTick : 0ULL));
            return true;
        }

        HMI_DIAG_NOINLINE void CapturePathCoreCompactDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            // NC-0.2L.1E2: only scalar state/token values enter the fixed
            // process-lifetime workspace. No by-value diagnostic, stack
            // snapshot, counter family or text output is introduced.
            const std::uint32_t previousToken =
                workspace.pathCoreInputHandoffChangeToken;
            const std::uint32_t currentToken =
                nc.GetPathCoreInputHandoffChangeToken();

            workspace.pathCoreInputHandoffCompactState =
                nc.GetPathCoreInputHandoffCompactState();
            workspace.pathCoreInputHandoffChangeToken = currentToken;
            workspace.pathCoreInputHandoffChanged =
                currentToken != previousToken;
        }

        HMI_DIAG_NOINLINE void CaptureTransportDiagnosticFamily(
            MotionCore& motion,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.pdoInvalidCorrelation = {};
            workspace.pdoInvalidCorrelationCoherent =
                TryReadEtherCatPdoRuntimeInvalidCorrelation(
                    workspace.pdoInvalidCorrelation);
            workspace.pdoSafetyStopCause = {};
            workspace.pdoSafetyStopCauseCoherent =
                TryReadEtherCatPdoSafetyStopCause(workspace.pdoSafetyStopCause);
            workspace.startupLagArming = motion.GetStartupLagArmingEvidence();
            workspace.p1HandoverSafety = motion.GetP1HandoverSafetySnapshot();
            workspace.lifecycleCommit =
                motion.GetLifecycleCommitReservationSnapshot();
            workspace.commandPathModeTransport =
                motion.GetCommandPathModeTransportSnapshot();
            workspace.queueTailTransaction =
                motion.GetQueueTailTransactionSnapshot();
        }

        HMI_DIAG_NOINLINE void CaptureMotionSettleDiagnosticFamily(
            MotionCore& motion,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.stopSettleSnapshot = {};
            workspace.stopSettleCounters = {};
            motion.GetStopSettleEvidence(
                workspace.stopSettleSnapshot,
                workspace.stopSettleCounters);
            workspace.groupNCSettleSnapshot = {};
            workspace.groupNCSettleCounters = {};
            workspace.groupNCSettleCoherent = motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::GROUP_COMPLETION,
                workspace.groupNCSettleSnapshot,
                workspace.groupNCSettleCounters);
            workspace.feedHoldNCSettleSnapshot = {};
            workspace.feedHoldNCSettleCounters = {};
            workspace.feedHoldNCSettleCoherent = motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::FEED_HOLD_GROUP,
                workspace.feedHoldNCSettleSnapshot,
                workspace.feedHoldNCSettleCounters);
            workspace.resetNCSettleSnapshot = {};
            workspace.resetNCSettleCounters = {};
            workspace.resetNCSettleCoherent = motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::RESET_ALL,
                workspace.resetNCSettleSnapshot,
                workspace.resetNCSettleCounters);
            workspace.resetRebaseAck = motion.GetNCResetRebaseAck();
            workspace.ownerLease = motion.GetMotionOwnerLease();
        }

        HMI_DIAG_NOINLINE void CaptureNCBlockDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.blockLifecycle = {};
            workspace.hasBlockLifecycle =
                nc.GetLastBlockLifecycleSnapshot(workspace.blockLifecycle);
            workspace.blockCounters = nc.GetBlockLifecycleCounters();
            workspace.completionBoundary =
                nc.GetLastBlockCompletionBoundarySnapshot();
            workspace.completionCounters =
                nc.GetBlockCompletionBoundaryCounters();
            workspace.programEndSnapshot = nc.GetProgramEndGateSnapshot();
            workspace.programEndCounters = nc.GetProgramEndGateCounters();
            workspace.gmTransactionSnapshot = nc.GetGMBlockTransactionSnapshot();
            workspace.gmTransactionCounters = nc.GetGMBlockTransactionCounters();
            workspace.preDispatchBarrierSnapshot =
                nc.GetPreDispatchBarrierSnapshot();
            workspace.preDispatchBarrierCounters =
                nc.GetPreDispatchBarrierCounters();
        }

        HMI_DIAG_NOINLINE void CaptureNCPreparedQueueDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.preparedQueueSnapshot = nc.GetPreparedBlockQueueSnapshot();
            workspace.preparedQueueCounters = nc.GetPreparedBlockQueueCounters();
            workspace.preparedEquivalenceSnapshot = nc.GetPreparedHeadEquivalenceSnapshot();
            workspace.preparedEquivalenceCounters = nc.GetPreparedHeadEquivalenceCounters();
        }

        HMI_DIAG_NOINLINE void CaptureNCPreparedAdmissionDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.preparedCutoverSnapshot = nc.GetPreparedHeadCutoverSnapshot();
            workspace.preparedCutoverCounters = nc.GetPreparedHeadCutoverCounters();
            workspace.preparedPreResolveSnapshot = nc.GetPreparedPreResolveAdmissionSnapshot();
            workspace.preparedPreResolveCounters = nc.GetPreparedPreResolveAdmissionCounters();
            workspace.preparedResolverBypassSnapshot = nc.GetPreparedResolverBypassSnapshot();
            workspace.preparedResolverBypassCounters = nc.GetPreparedResolverBypassCounters();
            workspace.ordinaryG00AdmissionSnapshot = nc.GetOrdinaryG00AdmissionSnapshot();
            workspace.ordinaryG00AdmissionCounters = nc.GetOrdinaryG00AdmissionCounters();
        }

        HMI_DIAG_NOINLINE void CaptureNCPreparedReadAheadDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.ordinaryG00InflightRegistrySnapshot = nc.GetOrdinaryG00InflightRegistrySnapshot();
            workspace.ordinaryG00InflightRegistryCounters = nc.GetOrdinaryG00InflightRegistryCounters();
            workspace.ordinaryG00ReadAheadSnapshot = nc.GetOrdinaryG00ReadAheadSnapshot();
            workspace.ordinaryG00ReadAheadCounters = nc.GetOrdinaryG00ReadAheadCounters();
        }

        HMI_DIAG_NOINLINE void CaptureNCPreparedHoldDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.ordinaryG00FeedHoldCohortSnapshot = nc.GetOrdinaryG00FeedHoldCohortSnapshot();
            workspace.ordinaryG00FeedHoldCohortCounters = nc.GetOrdinaryG00FeedHoldCohortCounters();
            workspace.ordinaryG00FeedHoldCohortCutoverSnapshot = nc.GetOrdinaryG00FeedHoldCohortCutoverSnapshot();
            workspace.ordinaryG00FeedHoldCohortCutoverCounters = nc.GetOrdinaryG00FeedHoldCohortCutoverCounters();
            workspace.ordinaryG00FeedHoldCohortRearmSnapshot = nc.GetOrdinaryG00FeedHoldCohortRearmSnapshot();
            workspace.ordinaryG00FeedHoldCohortRearmCounters = nc.GetOrdinaryG00FeedHoldCohortRearmCounters();
            workspace.ordinaryG00FeedHoldRearmCutoverSnapshot = nc.GetOrdinaryG00FeedHoldRearmCutoverSnapshot();
            workspace.ordinaryG00FeedHoldRearmCutoverCounters = nc.GetOrdinaryG00FeedHoldRearmCutoverCounters();
            workspace.ordinaryG00FeedHoldRollingRearmSnapshot = nc.GetOrdinaryG00FeedHoldRollingRearmSnapshot();
            workspace.ordinaryG00FeedHoldRollingRearmCounters = nc.GetOrdinaryG00FeedHoldRollingRearmCounters();
            workspace.ordinaryG00FeedHoldRollingCutoverSnapshot = nc.GetOrdinaryG00FeedHoldRollingCutoverSnapshot();
            workspace.ordinaryG00FeedHoldRollingCutoverCounters = nc.GetOrdinaryG00FeedHoldRollingCutoverCounters();
            workspace.preparedQueueHead = {};
            workspace.preparedQueueTail = {};
            workspace.hasPreparedQueueHead = nc.GetPreparedBlockQueueEntry(0U, workspace.preparedQueueHead);
            workspace.hasPreparedQueueTail =
                workspace.preparedQueueSnapshot.depth != 0U &&
                nc.GetPreparedBlockQueueEntry(
                    static_cast<std::size_t>(workspace.preparedQueueSnapshot.depth - 1U),
                    workspace.preparedQueueTail);
        }

        HMI_DIAG_NOINLINE void CaptureNCHoldSafetyDiagnosticFamily(
            NCManager& nc,
            Hmi1000msDiagnosticWorkspace& workspace)
        {
            workspace.singleBlockShadowSnapshot = nc.GetSingleBlockShadowSnapshot();
            workspace.singleBlockShadowCounters = nc.GetSingleBlockShadowCounters();
            workspace.singleBlockGateSnapshot = nc.GetSingleBlockHoldGateSnapshot();
            workspace.singleBlockGateCounters = nc.GetSingleBlockHoldGateCounters();
            workspace.feedHoldSnapshot = nc.GetFeedHoldBoundarySnapshot();
            workspace.feedHoldCounters = nc.GetFeedHoldBoundaryCounters();
            workspace.feedHoldGateSnapshot = nc.GetFeedHoldResumeGateSnapshot();
            workspace.feedHoldGateCounters = nc.GetFeedHoldResumeGateCounters();
            workspace.lifecycleInterruptionSnapshot = nc.GetLifecycleInterruptionSnapshot();
            workspace.lifecycleInterruptionCounters = nc.GetLifecycleInterruptionCounters();
            workspace.resetReleaseGateSnapshot = nc.GetResetReleaseGateSnapshot();
            workspace.resetReleaseGateCounters = nc.GetResetReleaseGateCounters();
            workspace.alarmEmergencyStopSnapshot = nc.GetAlarmEmergencyStopSnapshot();
            workspace.alarmEmergencyStopCounters = nc.GetAlarmEmergencyStopCounters();
        }

        template <typename DiagnosticWriter>
        HMI_DIAG_NOINLINE void RunHmiDiagnosticOutputFamily(
            DiagnosticWriter&& writer)
        {
            // Compile-time dispatch only: no allocation and no retained
            // callable. The noinline boundary confines each family's
            // RtPrintf argument staging to its own bounded frame.
            writer();
        }

        HMI_DIAG_NOINLINE std::uint64_t
            BuildCommandPathModeChangeToken(
                const MotionCommandPathModeTransportSnapshot& snapshot) noexcept
        {
            const MotionCommandPathModeTransportSnapshot& commandPathModeTransport = snapshot;

            std::uint64_t token = 0ULL;
            const auto fold = [&token](std::uint64_t value) noexcept
            {
                token = FoldDiagnosticEventToken(token, value);
            };
            fold(commandPathModeTransport.producerAccepted);
            fold(commandPathModeTransport.producerRejected);
            fold(commandPathModeTransport.producerExactStop);
            fold(commandPathModeTransport.producerContinuous);
            fold(commandPathModeTransport.producerUnspecified);
            fold(commandPathModeTransport.producerInvalid);
            fold(commandPathModeTransport.producerFingerprint);
            fold(commandPathModeTransport.consumerCommitted);
            fold(commandPathModeTransport.consumerIngressCommitted);
            fold(commandPathModeTransport.consumerReplayCommitted);
            fold(commandPathModeTransport.consumerExactStop);
            fold(commandPathModeTransport.consumerContinuous);
            fold(commandPathModeTransport.consumerUnspecified);
            fold(commandPathModeTransport.consumerInvalid);
            fold(commandPathModeTransport.consumerFingerprint);
            fold(commandPathModeTransport.legacyModeMatches);
            fold(commandPathModeTransport.legacyModeMismatches);
            fold(commandPathModeTransport.driverOverrideObservations);
            fold(commandPathModeTransport.authorityAttempts);
            fold(commandPathModeTransport.authorityApplied);
            fold(commandPathModeTransport.authorityExactStop);
            fold(commandPathModeTransport.authorityContinuous);
            fold(commandPathModeTransport.authorityLegacyFallbacks);
            fold(commandPathModeTransport.authorityReplayBypasses);
            fold(commandPathModeTransport.authorityDriverBlocks);
            fold(commandPathModeTransport.authorityInvalidRejects);
            fold(commandPathModeTransport.commandLocalPayloadPresent ? 1ULL : 0ULL);
            fold(commandPathModeTransport.transportReady ? 1ULL : 0ULL);
            fold(commandPathModeTransport.producerSnapshotCoherent ? 1ULL : 0ULL);
            fold(commandPathModeTransport.consumerSnapshotCoherent ? 1ULL : 0ULL);
            fold(commandPathModeTransport.accountingValid ? 1ULL : 0ULL);
            return token;
        }

        HMI_DIAG_NOINLINE std::uint64_t
            BuildQueueTailTransactionChangeToken(
                const MotionQueueTailTransactionSnapshot& snapshot) noexcept
        {
            const MotionQueueTailTransactionSnapshot& queueTailTransaction = snapshot;

            std::uint64_t token = 0ULL;
            const auto fold = [&token](std::uint64_t value) noexcept
            {
                token = FoldDiagnosticEventToken(token, value);
            };
            fold(queueTailTransaction.writeSequence);
            fold(queueTailTransaction.attempts);
            fold(queueTailTransaction.commandAccepted);
            fold(queueTailTransaction.commandRejected);
            fold(queueTailTransaction.committed);
            fold(queueTailTransaction.rejectPreserved);
            fold(queueTailTransaction.commandedMCSCommitted);
            fold(queueTailTransaction.lastQueuedPulseCommitted);
            fold(queueTailTransaction.rapidOverrideCommitted);
            fold(queueTailTransaction.endpointExact);
            fold(queueTailTransaction.captureBound);
            fold(queueTailTransaction.invalidInputs);
            fold(queueTailTransaction.mismatches);
            fold(queueTailTransaction.lastTransactionSequence);
            fold(queueTailTransaction.lastExecutionEpoch);
            fold(queueTailTransaction.lastSegmentId);
            fold(queueTailTransaction.lastAxisMask);
            fold(queueTailTransaction.lastCommittedFingerprint);
            fold(queueTailTransaction.authoritative ? 1ULL : 0ULL);
            fold(queueTailTransaction.shadowOnly ? 1ULL : 0ULL);
            fold(queueTailTransaction.cutoverAttempted ? 1ULL : 0ULL);
            fold(queueTailTransaction.runtimeInfluence ? 1ULL : 0ULL);
            fold(queueTailTransaction.ready ? 1ULL : 0ULL);
            fold(queueTailTransaction.snapshotCoherent ? 1ULL : 0ULL);
            fold(queueTailTransaction.accountingValid ? 1ULL : 0ULL);
            return token;
        }

        HMI_DIAG_NOINLINE std::uint64_t
            BuildPreparedDiagnosticsChangeToken(
                const Hmi1000msDiagnosticWorkspace& workspace,
                std::uint64_t preparedEquivalenceStateBits) noexcept
        {
            const NCPreparedHeadEquivalenceSnapshot& preparedEquivalenceSnapshot = workspace.preparedEquivalenceSnapshot;
            const NCPreparedHeadEquivalenceCounters& preparedEquivalenceCounters = workspace.preparedEquivalenceCounters;
            const NCPreparedResolverBypassSnapshot& preparedResolverBypassSnapshot = workspace.preparedResolverBypassSnapshot;
            const NCPreparedResolverBypassCounters& preparedResolverBypassCounters = workspace.preparedResolverBypassCounters;
            const NCOrdinaryG00AdmissionSnapshot& ordinaryG00AdmissionSnapshot = workspace.ordinaryG00AdmissionSnapshot;
            const NCOrdinaryG00AdmissionCounters& ordinaryG00AdmissionCounters = workspace.ordinaryG00AdmissionCounters;
            const NCOrdinaryG00InflightRegistrySnapshot& ordinaryG00InflightRegistrySnapshot = workspace.ordinaryG00InflightRegistrySnapshot;
            const NCOrdinaryG00InflightRegistryCounters& ordinaryG00InflightRegistryCounters = workspace.ordinaryG00InflightRegistryCounters;
            const NCOrdinaryG00ReadAheadSnapshot& ordinaryG00ReadAheadSnapshot = workspace.ordinaryG00ReadAheadSnapshot;
            const NCOrdinaryG00ReadAheadCounters& ordinaryG00ReadAheadCounters = workspace.ordinaryG00ReadAheadCounters;
            const NCOrdinaryG00FeedHoldCohortSnapshot& ordinaryG00FeedHoldCohortSnapshot = workspace.ordinaryG00FeedHoldCohortSnapshot;
            const NCOrdinaryG00FeedHoldCohortCounters& ordinaryG00FeedHoldCohortCounters = workspace.ordinaryG00FeedHoldCohortCounters;
            const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& ordinaryG00FeedHoldCohortCutoverSnapshot = workspace.ordinaryG00FeedHoldCohortCutoverSnapshot;
            const NCOrdinaryG00FeedHoldCohortCutoverCounters& ordinaryG00FeedHoldCohortCutoverCounters = workspace.ordinaryG00FeedHoldCohortCutoverCounters;
            const NCOrdinaryG00FeedHoldCohortRearmSnapshot& ordinaryG00FeedHoldCohortRearmSnapshot = workspace.ordinaryG00FeedHoldCohortRearmSnapshot;
            const NCOrdinaryG00FeedHoldCohortRearmCounters& ordinaryG00FeedHoldCohortRearmCounters = workspace.ordinaryG00FeedHoldCohortRearmCounters;
            const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& ordinaryG00FeedHoldRearmCutoverSnapshot = workspace.ordinaryG00FeedHoldRearmCutoverSnapshot;
            const NCOrdinaryG00FeedHoldRearmCutoverCounters& ordinaryG00FeedHoldRearmCutoverCounters = workspace.ordinaryG00FeedHoldRearmCutoverCounters;
            const NCOrdinaryG00FeedHoldRollingRearmSnapshot& ordinaryG00FeedHoldRollingRearmSnapshot = workspace.ordinaryG00FeedHoldRollingRearmSnapshot;
            const NCOrdinaryG00FeedHoldRollingRearmCounters& ordinaryG00FeedHoldRollingRearmCounters = workspace.ordinaryG00FeedHoldRollingRearmCounters;
            const NCOrdinaryG00FeedHoldRollingCutoverSnapshot& ordinaryG00FeedHoldRollingCutoverSnapshot = workspace.ordinaryG00FeedHoldRollingCutoverSnapshot;
            const NCOrdinaryG00FeedHoldRollingCutoverCounters& ordinaryG00FeedHoldRollingCutoverCounters = workspace.ordinaryG00FeedHoldRollingCutoverCounters;
            std::uint64_t token = 0ULL;
            const auto fold = [&token](std::uint64_t value) noexcept
            {
                token = FoldDiagnosticEventToken(token, value);
            };
            fold(preparedEquivalenceSnapshot.session);
            fold(preparedEquivalenceSnapshot.entrySequence);
            fold(static_cast<std::uint64_t>(preparedEquivalenceSnapshot.scope));
            fold(preparedEquivalenceSnapshot.cacheGeneration);
            fold(preparedEquivalenceSnapshot.frameId);
            fold(preparedEquivalenceSnapshot.executionEpoch);
            fold(preparedEquivalenceSnapshot.commitExecutionEpoch);
            fold(preparedEquivalenceSnapshot.programFlowGeneration);
            fold(preparedEquivalenceSnapshot.owner);
            fold(preparedEquivalenceSnapshot.panelMask);
            fold(preparedEquivalenceSnapshot.ownerGeneration);
            fold(preparedEquivalenceSnapshot.dispatchId);
            fold(preparedEquivalenceSnapshot.commitSequence);
            fold(preparedEquivalenceSnapshot.qualifiedSession);
            fold(static_cast<std::uint64_t>(preparedEquivalenceSnapshot.state));
            fold(static_cast<std::uint64_t>(
                preparedEquivalenceSnapshot.lastInvalidation));
            fold(static_cast<std::uint64_t>(
                preparedEquivalenceSnapshot.mismatchFlags));
            fold(preparedEquivalenceStateBits);
            fold(preparedEquivalenceSnapshot.sessionPeakDepth);
            fold(preparedEquivalenceCounters.candidates);
            fold(preparedEquivalenceCounters.matched);
            fold(preparedEquivalenceCounters.mismatched);
            fold(preparedEquivalenceCounters.invalidated);
            fold(preparedEquivalenceCounters.ineligible);
            fold(preparedEquivalenceCounters.lifetimePeakDepth);
            fold(preparedEquivalenceCounters.multiBlockObservations);
            fold(preparedEquivalenceCounters.queueMismatches);
            fold(preparedEquivalenceCounters.sourceMismatches);
            fold(preparedEquivalenceCounters.pcMismatches);
            fold(preparedEquivalenceCounters.lineMismatches);
            fold(preparedEquivalenceCounters.blockMismatches);
            fold(preparedEquivalenceCounters.planMismatches);
            fold(preparedEquivalenceCounters.classificationMismatches);
            fold(preparedEquivalenceCounters.drainMismatches);
            fold(preparedEquivalenceCounters.modalBeforeMismatches);
            fold(preparedEquivalenceCounters.modalAfterMismatches);
            fold(preparedEquivalenceCounters.lifecycleMismatches);
            fold(preparedEquivalenceCounters.staleTokens);
            fold(preparedEquivalenceCounters.resolveMismatches);
            fold(preparedEquivalenceCounters.upstreamMismatches);
            fold(preparedEquivalenceCounters.ledgerMismatches);
            fold(preparedEquivalenceCounters.bindIdentityMismatches);
            fold(preparedEquivalenceCounters.runtimeFailures);
            fold(preparedEquivalenceCounters.sessionTransitions);
            fold(preparedEquivalenceCounters.qualifiedSessions);
            fold(preparedEquivalenceCounters.wouldUse);
            fold(preparedEquivalenceCounters.useAttempts);
            fold(preparedEquivalenceCounters.cutoverAttempts);
            fold(preparedEquivalenceCounters.runtimeInfluence);
            fold(preparedResolverBypassSnapshot.publicationSequence);
            fold(static_cast<std::uint64_t>(
                preparedResolverBypassSnapshot.decision));
            fold(preparedResolverBypassCounters.selected);
            fold(preparedResolverBypassCounters.proofVerified);
            fold(preparedResolverBypassCounters.qualificationCandidates);
            fold(preparedResolverBypassCounters.qualifiedPureModal);
            fold(preparedResolverBypassCounters.qualifiedG00P1);
            fold(preparedResolverBypassCounters.runtimeFailures);
            fold(preparedResolverBypassCounters.proofMismatches);
            fold(preparedResolverBypassCounters.revocations);
            fold(preparedResolverBypassCounters.runtimeInfluence);
            fold(preparedResolverBypassCounters.resolverBypasses);
            fold(preparedResolverBypassSnapshot.g00NoPQualifiedSession);
            fold(preparedResolverBypassSnapshot.commitExecutionEpoch);
            fold(preparedResolverBypassSnapshot.legacyDrainRequired ? 1ULL : 0ULL);
            fold(preparedResolverBypassSnapshot.legacyDrainSatisfied ? 1ULL : 0ULL);
            fold(preparedResolverBypassSnapshot.deferredForDrain ? 1ULL : 0ULL);
            fold(preparedResolverBypassSnapshot.callbackRequired ? 1ULL : 0ULL);
            fold(preparedResolverBypassSnapshot.callbackActiveAtCommit ? 1ULL : 0ULL);
            fold(preparedResolverBypassSnapshot.waitPhaseObserved ? 1ULL : 0ULL);
            fold(preparedResolverBypassSnapshot.callbackCompletionObserved
                ? 1ULL : 0ULL);
            fold(preparedResolverBypassCounters.selectedPureModal);
            fold(preparedResolverBypassCounters.selectedG00P1);
            fold(preparedResolverBypassCounters.selectedG00NoP);
            fold(preparedResolverBypassCounters.proofVerifiedPureModal);
            fold(preparedResolverBypassCounters.proofVerifiedG00P1);
            fold(preparedResolverBypassCounters.proofVerifiedG00NoP);
            fold(preparedResolverBypassCounters.drainWaitSamples);
            fold(preparedResolverBypassCounters.callbackWaitSamples);
            fold(preparedResolverBypassCounters.callbackCompletions);
            fold(preparedResolverBypassCounters.ordinaryWaitPhases);
            fold(preparedResolverBypassCounters.qualifiedG00NoP);
            fold(ordinaryG00AdmissionSnapshot.publicationSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00AdmissionSnapshot.decision));
            fold(ordinaryG00AdmissionSnapshot.session);
            fold(ordinaryG00AdmissionSnapshot.entrySequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00AdmissionSnapshot.owner));
            fold(ordinaryG00AdmissionSnapshot.ownerGeneration);
            fold(static_cast<std::uint64_t>(
                ordinaryG00AdmissionSnapshot.panelMask));
            fold(ordinaryG00AdmissionSnapshot.dispatchId);
            fold(ordinaryG00AdmissionSnapshot.commitSequence);
            fold(ordinaryG00AdmissionSnapshot.busySamples);
            fold(ordinaryG00AdmissionSnapshot.blockerMask);
            fold(ordinaryG00AdmissionSnapshot.pending ? 1ULL : 0ULL);
            fold(ordinaryG00AdmissionSnapshot.permanentLockout ? 1ULL : 0ULL);
            fold(ordinaryG00AdmissionSnapshot.accountingValid ? 1ULL : 0ULL);
            fold(ordinaryG00AdmissionSnapshot.legacyCallbackCompleted ? 1ULL : 0ULL);
            fold(ordinaryG00AdmissionSnapshot.legacyUpstreamProofVerified
                ? 1ULL : 0ULL);
            fold(ordinaryG00AdmissionCounters.uniqueEvaluations);
            fold(ordinaryG00AdmissionCounters.warmup);
            fold(ordinaryG00AdmissionCounters.candidates);
            fold(ordinaryG00AdmissionCounters.initialDrained);
            fold(ordinaryG00AdmissionCounters.initialBusy);
            fold(ordinaryG00AdmissionCounters.busySamples);
            fold(ordinaryG00AdmissionCounters.legacySelections);
            fold(ordinaryG00AdmissionCounters.legacyCommitBound);
            fold(ordinaryG00AdmissionCounters.callbackCompleted);
            fold(ordinaryG00AdmissionCounters.legacyCompleted);
            fold(ordinaryG00AdmissionCounters.rejected);
            fold(ordinaryG00AdmissionCounters.mismatches);
            fold(ordinaryG00AdmissionCounters.revocations);
            fold(ordinaryG00AdmissionCounters.runtimeInfluence);
            fold(ordinaryG00AdmissionCounters.resolverBypasses);
            fold(ordinaryG00AdmissionCounters.cutoverAttempts);
            fold(ordinaryG00AdmissionCounters.inflightRegistryBound);
            fold(ordinaryG00AdmissionSnapshot.inflightRegistrySequence);
            fold(ordinaryG00AdmissionSnapshot.inflightExecutionEpoch);
            fold(ordinaryG00AdmissionSnapshot.inflightSegmentId);
            fold(ordinaryG00AdmissionSnapshot.inflightRegistryProven
                ? 1ULL : 0ULL);
            fold(ordinaryG00InflightRegistrySnapshot.publicationSequence);
            fold(ordinaryG00InflightRegistrySnapshot.lastRegistrySequence);
            fold(ordinaryG00InflightRegistrySnapshot.currentSession);
            fold(ordinaryG00InflightRegistrySnapshot.activeEntries);
            fold(ordinaryG00InflightRegistrySnapshot.revokedPendingEntries);
            fold(ordinaryG00InflightRegistrySnapshot.permanentLockout
                ? 1ULL : 0ULL);
            fold(ordinaryG00InflightRegistrySnapshot.accountingValid
                ? 1ULL : 0ULL);
            fold(ordinaryG00InflightRegistryCounters.registrationAttempts);
            fold(ordinaryG00InflightRegistryCounters.registered);
            fold(ordinaryG00InflightRegistryCounters.registrationRejected);
            fold(ordinaryG00InflightRegistryCounters.capacityOverflow);
            fold(ordinaryG00InflightRegistryCounters.terminal);
            fold(ordinaryG00InflightRegistryCounters.completed);
            fold(ordinaryG00InflightRegistryCounters.rejected);
            fold(ordinaryG00InflightRegistryCounters.cancelled);
            fold(ordinaryG00InflightRegistryCounters.aborted);
            fold(ordinaryG00InflightRegistryCounters.faulted);
            fold(ordinaryG00InflightRegistryCounters.feedbackSequenceGaps);
            fold(ordinaryG00InflightRegistryCounters.ledgerOrphans);
            fold(ordinaryG00InflightRegistryCounters.identityConflicts);
            fold(ordinaryG00InflightRegistryCounters.ownerMismatches);
            fold(ordinaryG00InflightRegistryCounters.duplicateTerminal);
            fold(ordinaryG00InflightRegistryCounters.terminalConflict);
            fold(ordinaryG00InflightRegistryCounters.entriesRevoked);
            fold(ordinaryG00InflightRegistryCounters.revokedTerminals);
            fold(ordinaryG00InflightRegistryCounters.runtimeInfluence);
            fold(ordinaryG00InflightRegistryCounters.motionWrites);
            fold(ordinaryG00InflightRegistryCounters.readAheadRegistered);
            fold(ordinaryG00ReadAheadSnapshot.publicationSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00ReadAheadSnapshot.decision));
            fold(ordinaryG00ReadAheadSnapshot.session);
            fold(ordinaryG00ReadAheadSnapshot.warmupSession);
            fold(ordinaryG00ReadAheadSnapshot.dispatchId);
            fold(ordinaryG00ReadAheadSnapshot.commitSequence);
            fold(ordinaryG00ReadAheadSnapshot.identity.epoch);
            fold(ordinaryG00ReadAheadSnapshot.identity.segmentId);
            fold(static_cast<std::uint64_t>(
                ordinaryG00ReadAheadSnapshot.chainSourcePC + 1));
            fold(ordinaryG00ReadAheadSnapshot.contiguousChain ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.pending ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.permanentLockout ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.registryReady ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.registryHealthy ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.registryHealthFenced
                ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.registryHealthLockout
                ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadSnapshot.accountingValid ? 1ULL : 0ULL);
            fold(ordinaryG00ReadAheadCounters.selected);
            fold(ordinaryG00ReadAheadCounters.committed);
            fold(ordinaryG00ReadAheadCounters.capacityWaits);
            fold(ordinaryG00ReadAheadCounters.registryHealthFences);
            fold(ordinaryG00ReadAheadCounters.registryHealthLockouts);
            fold(ordinaryG00ReadAheadCounters.runtimeFailures);
            fold(ordinaryG00ReadAheadCounters.proofMismatches);
            fold(ordinaryG00ReadAheadCounters.runtimeInfluence);
            fold(ordinaryG00FeedHoldCohortSnapshot.publicationSequence);
            fold(ordinaryG00FeedHoldCohortSnapshot.cohortSequence);
            fold(ordinaryG00FeedHoldCohortSnapshot.boundarySequence);
            fold(ordinaryG00FeedHoldCohortSnapshot.gateSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldCohortSnapshot.phase));
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldCohortSnapshot.decision));
            fold(ordinaryG00FeedHoldCohortSnapshot.terminalCount);
            fold(ordinaryG00FeedHoldCohortSnapshot.failed ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortSnapshot.accountingValid ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortCounters.cohortsCaptured);
            fold(ordinaryG00FeedHoldCohortCounters.terminalCohorts);
            fold(ordinaryG00FeedHoldCohortCounters.failures);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.publicationSequence);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.cohortSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldCohortCutoverSnapshot.phase));
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldCohortCutoverSnapshot.decision));
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.terminalCount);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.registryActiveEntries);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.waiting ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.released ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.fallbackLegacy
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortCutoverSnapshot.accountingValid
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.cohortsBound);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.waitFirstTerminal);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.waitSecondTerminal);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.releases);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.allowReadAhead);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.fallbackLegacy);
            fold(ordinaryG00FeedHoldCohortCutoverCounters.runtimeInfluence);
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.publicationSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldCohortRearmSnapshot.phase));
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldCohortRearmSnapshot.decision));
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.session);
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.generationCount);
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.releaseCount);
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.rearmProven ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.failed ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortRearmSnapshot.accountingValid
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldCohortRearmCounters.generationsCaptured);
            fold(ordinaryG00FeedHoldCohortRearmCounters.firstReleased);
            fold(ordinaryG00FeedHoldCohortRearmCounters.secondReleased);
            fold(ordinaryG00FeedHoldCohortRearmCounters.earlyRearm);
            fold(ordinaryG00FeedHoldCohortRearmCounters.sessionMismatches);
            fold(ordinaryG00FeedHoldCohortRearmCounters.memberIdentityReuse);
            fold(ordinaryG00FeedHoldCohortRearmCounters.proofMismatches);
            fold(ordinaryG00FeedHoldCohortRearmCounters.sessionResets);
            fold(ordinaryG00FeedHoldCohortRearmCounters.failures);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.publicationSequence);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.
                rearmPublicationSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldRearmCutoverSnapshot.phase));
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldRearmCutoverSnapshot.decision));
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.session);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.cohortSequence);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.boundarySequence);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.gateSequence);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.generationCount);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.releaseCount);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.bound ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.waiting ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.released ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.fallbackLegacy
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.sessionLockout
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverSnapshot.accountingValid
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.secondGenerationsBound);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.waitK74Release != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.waitK75RearmProof != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.releases);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.allowReadAhead != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.fallbackLegacy != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.sessionResets);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.failures);
            fold(ordinaryG00FeedHoldRearmCutoverCounters.runtimeInfluence != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.publicationSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                seedK75PublicationSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                seedK76PublicationSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                seedK73PublicationSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                seedK74PublicationSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                seedRegistryPublicationSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldRollingRearmSnapshot.phase));
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldRollingRearmSnapshot.decision));
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.session);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.lineageDigest);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.lastCohortSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.lastBoundarySequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.lastGateSequence);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                registrySequenceHighWater);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.entrySequenceHighWater);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.dispatchIdHighWater);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.commitSequenceHighWater);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.segmentIdHighWater);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.rollingCaptureCount);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.rollingReleaseCount);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.generationCount);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.releaseCount);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.
                activeGenerationOrdinal);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.seeded ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.tracking ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.active ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.releasedContinuity
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.failed ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmSnapshot.accountingValid
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingRearmCounters.seedAccepted);
            fold(ordinaryG00FeedHoldRollingRearmCounters.rollingCohortEdges);
            fold(ordinaryG00FeedHoldRollingRearmCounters.nonExactBypasses);
            fold(ordinaryG00FeedHoldRollingRearmCounters.rollingCaptures);
            fold(ordinaryG00FeedHoldRollingRearmCounters.rollingReleases);
            fold(ordinaryG00FeedHoldRollingRearmCounters.earlyEdges);
            fold(ordinaryG00FeedHoldRollingRearmCounters.sessionMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.
                executionLeaseMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.sequenceMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.
                sequenceWrapUnsupported);
            fold(ordinaryG00FeedHoldRollingRearmCounters.lineageMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.registryMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.accountingMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.
                activeInvariantMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.ordinalExhausted);
            fold(ordinaryG00FeedHoldRollingRearmCounters.proofMismatches);
            fold(ordinaryG00FeedHoldRollingRearmCounters.fallbackObserved);
            fold(ordinaryG00FeedHoldRollingRearmCounters.sessionResets);
            fold(ordinaryG00FeedHoldRollingRearmCounters.resetWhileActive);
            fold(ordinaryG00FeedHoldRollingRearmCounters.failures);
            fold(ordinaryG00FeedHoldRollingRearmCounters.runtimeInfluence);
            fold(ordinaryG00FeedHoldRollingRearmCounters.motionWrites);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.publicationSequence);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.
                rollingPublicationSequence);
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldRollingCutoverSnapshot.phase));
            fold(static_cast<std::uint64_t>(
                ordinaryG00FeedHoldRollingCutoverSnapshot.decision));
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.session);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.generationOrdinal);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.
                lastReleasedGenerationOrdinal);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.bound ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.waiting ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.released ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.fallbackLegacy
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.sessionLockout
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverSnapshot.accountingValid
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.generationsBound);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.waitK74Release != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.
                waitK77ContinuityProof != 0ULL ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.releases);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.allowReadAhead != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.fallbackLegacy != 0ULL
                ? 1ULL : 0ULL);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.sessionResets);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.failures);
            fold(ordinaryG00FeedHoldRollingCutoverCounters.runtimeInfluence != 0ULL
                ? 1ULL : 0ULL);
            return token;
        }

        Hmi1000msDiagnosticWorkspace g_hmi1000msDiagnosticWorkspace{};
    }
    // =========================================================================
    // 🚀 1. ProcessTask (每一圈執行) - 處理高優先級命令與即時座標
    // =========================================================================
    void ProcessTask(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

        // 🌟 [新增] 1. 取得當前的公英制倍率 (公制=1.0, 英制=1/25.4)
        double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;


        // --- 狀態迴圈更新 ---
        pShm->API_Status.SHM_API_RunCount = nc->API_RunCount++;
        pShm->NC_Status.SHM_NC_RunCount = nc->NC_RunCount;

        pShm->NC_Status.SHM_m_mode = static_cast<int32_t>(nc->m_mode);

        pShm->NC_Status.SHM_NC_State = static_cast<int32_t>(nc->GetState());
        pShm->NC_Status.SHM_EDM_State = static_cast<int32_t>(nc->m_edmState);


        pShm->NC_Status.m_isSingleBlockEnabled = nc->m_isSingleBlockEnabled;
        pShm->NC_Status.m_isOptionalStopEnabled = nc->m_isOptionalStopEnabled;
        pShm->NC_Status.m_isBlockSkipEnabled = nc->m_isBlockSkipEnabled;


        // --- 即時座標廣播 ---
        double currentWCS[8] = { 0.0 };
        nc->CoordSys.GetActualWCS(currentWCS);

        // 🌟 [神級修復] 2. 將 DTG 的計算提拔到迴圈外部！避免重複運算與變數遮蔽(Shadowing) Bug
        double currentDTG[8] = { 0.0 };
        nc->CoordSys.GetDistanceToGo(currentDTG, nc);

        for (int i = 0; i < 8; i++)
        {
            // 取得軸的 Context
            auto& axis = nc->m_motion.GetAxisContext(i);
            AxisType type = axis.axisType;

            // 🌟 3. 判斷是否為旋轉軸 (旋轉軸永遠是度數 deg，絕對不可以套用 inch 轉換)
            double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

            double currentComp = axis.currentCompOffset_unit;

            // 🌟 4. 扣除補償量，並套用公英制倍率
            double displayMCS = (nc->CoordSys.actualMCS[i] - currentComp) * axisScale;
            double displayWCS = (currentWCS[i] - currentComp) * axisScale;

            // 把 DTG 塞進共享記憶體，同樣套用倍率
            pShm->NC_Status.DistanceToGo[i] = currentDTG[i] * axisScale;

            // ========================================================
            // 🌟 旋轉軸的 0~360 度顯示處理
            // ========================================================
            if (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS)
            {
                // 對 360 取餘數 (折疊座標)
                displayMCS = std::fmod(displayMCS, 360.0);
                displayWCS = std::fmod(displayWCS, 360.0);

                // 防呆：如果是負角度 (例如 -10 度)，轉回正的 350 度
                if (displayMCS < 0.0) displayMCS += 360.0;
                if (displayWCS < 0.0) displayWCS += 360.0;
            }

            // 寫入 Shared Memory 廣播給人機
            pShm->NC_Status.actualMCS[i] = displayMCS;
            pShm->NC_Status.actualWCS[i] = displayWCS;


            pShm->NC_Status.manualFrameEnabled = nc->CoordSys.IsManualFrameEnabled() ? 1 : 0;
            pShm->NC_Status.manualFrameYawDeg = nc->CoordSys.GetManualFrameYaw();
            pShm->NC_Status.manualFramePitchDeg = nc->CoordSys.GetManualFramePitch();
            pShm->NC_Status.manualFrameRollDeg = nc->CoordSys.GetManualFrameRoll();
        }


        // --- HMI 控制指令交握 ---
        if (pShm->varCmd.writeReq)
        {
            nc->MacroSys.SetVar(pShm->varCmd.prefix, pShm->varCmd.index, pShm->varCmd.writeValue);
            pShm->varCmd.writeReq = false;
        }
        // CJ LOAD FIX1: consume the request group before dispatching START.
        const bool closeRequested = pShm->NC_Command.Close_System ||
            nc->Close_System_Com_flag;
        const bool resetRequested = pShm->NC_Command.reset;
        const bool holdRequested = pShm->NC_Command.feedHold;
        const bool modeRequested = pShm->NC_Command.reqChangeMode;
        const bool loadRequested = pShm->NC_Command.loadProgramReq;
        const bool startRequested = pShm->NC_Command.cycleStart;
        if (startRequested) pShm->NC_Command.cycleStart = false;

        if (pShm->NC_Command.Close_System)
        {
            nc->Close_System_Com_flag = true;
            pShm->NC_Command.Close_System = false;
        }
        if (resetRequested)
        {
            nc->Reset();
            pShm->NC_Command.reset = false;
        }
        if (holdRequested)
        {
            if (!resetRequested && !closeRequested) nc->FeedHold();
            pShm->NC_Command.feedHold = false;
        }
        if (modeRequested)
        {
            if (!resetRequested && !holdRequested && !closeRequested)
            {
                nc->ChangeMode(static_cast<NCOperationMode>(
                    pShm->NC_Command.targetMode));
            }
            pShm->NC_Command.reqChangeMode = false;
        }

        if (loadRequested)
        {
            const size_t safeLength = strnlen(
                pShm->NC_Command.loadprogramName,
                sizeof(pShm->NC_Command.loadprogramName));
            const std::string requestedName(
                pShm->NC_Command.loadprogramName, safeLength);
            const std::string requestedPath =
                GlobalConfig::GetInstance().NCProgramDir + requestedName + ".nc";
            const char* priorityReason = closeRequested ? "CLOSE_REQUEST" :
                resetRequested ? "RESET_REQUEST" :
                holdRequested ? "HOLD_REQUEST" :
                modeRequested ? "MODE_REQUEST" : nullptr;
            const bool loaded = priorityReason != nullptr
                ? nc->RejectProgramLoad(priorityReason, requestedPath)
                : nc->LoadProgram(requestedPath);
            RtPrintf("[HMI-LOAD-CJ] result=%s requested=%s actual=%s "
                "startDiscarded=%u\n",
                loaded ? "SUCCESS" : "REJECTED",
                requestedName.c_str(), nc->m_mainProgramName.c_str(),
                (startRequested || pShm->NC_Command.cycleStart) ? 1U : 0U);
            pShm->NC_Command.loadProgramReq = false;
            std::memset(pShm->NC_Command.loadprogramName, 0,
                sizeof(pShm->NC_Command.loadprogramName));
            // START published before LOAD finishes also needs a fresh press.
            pShm->NC_Command.cycleStart = false;
        }
        else if (startRequested && !closeRequested && !resetRequested &&
            !holdRequested && !modeRequested &&
            !pShm->NC_Command.Close_System && !pShm->NC_Command.reset &&
            !pShm->NC_Command.feedHold && !pShm->NC_Command.reqChangeMode &&
            !pShm->NC_Command.loadProgramReq)
        {
            nc->CycleStart();
        }
        // CJ LOAD FIX1: end of ordered control request dispatch.


        //單步執行切換
        if (pShm->NC_Command.Set_isSingleBlockEnabled_ON)
        {
            g_PLC->Set_C(NCPLC::C::SINGLE_BLOCK, true);

            nc->SetSingleBlockEnabled(true);
            pShm->NC_Command.Set_isSingleBlockEnabled_ON = false;
        }
        if (pShm->NC_Command.Set_isSingleBlockEnabled_OFF)
        {
            g_PLC->Set_C(NCPLC::C::SINGLE_BLOCK, false);
            nc->SetSingleBlockEnabled(false);
            pShm->NC_Command.Set_isSingleBlockEnabled_OFF = false;
        }
        //選擇性暫停切換
        if (pShm->NC_Command.Set_isOptionalStopEnabled_ON)
        {
            g_PLC->Set_C(NCPLC::C::OPTIONAL_STOP, true);
            nc->m_isOptionalStopEnabled = true;
            pShm->NC_Command.Set_isOptionalStopEnabled_ON = false;
        }
        if (pShm->NC_Command.Set_isOptionalStopEnabled_OFF)
        {
            g_PLC->Set_C(NCPLC::C::OPTIONAL_STOP, false);
            nc->m_isOptionalStopEnabled = false;
            pShm->NC_Command.Set_isOptionalStopEnabled_OFF = false;
        }
        //選擇性跳躍切換
        if (pShm->NC_Command.Set_isBlockSkipEnabled_ON)
        {
            g_PLC->Set_C(NCPLC::C::BLOCK_SKIP, true);
            nc->m_isBlockSkipEnabled = true;
            pShm->NC_Command.Set_isBlockSkipEnabled_ON = false;
        }
        if (pShm->NC_Command.Set_isBlockSkipEnabled_OFF)
        {
            g_PLC->Set_C(NCPLC::C::BLOCK_SKIP, false);
            nc->m_isBlockSkipEnabled = false;
            pShm->NC_Command.Set_isBlockSkipEnabled_OFF = false;
        }



        if (pShm->Coord_Command.reqSwitchWCS)
        {
            nc->CoordSys.SetWCS(pShm->Coord_Command.targetWCS_GCode, nc, false);
            pShm->Coord_Command.reqSwitchWCS = false;
        }

        if (pShm->Coord_Command.reqSetManualFrame)
        {
            // 1. Read Requested Values
            const bool requestedEnabled = pShm->Coord_Command.manualFrameEnabled;
            const double requestedYawDeg = pShm->Coord_Command.manualFrameYawDeg;
            const double requestedPitchDeg = pShm->Coord_Command.manualFramePitchDeg;
            const double requestedRollDeg = pShm->Coord_Command.manualFrameRollDeg;

            // 2. Parameter Validation (Shared Memory 是外部輸入邊界。NaN/INF 絕對不能進 CoordinateManager)
            const bool parameterValid = std::isfinite(requestedYawDeg) && std::isfinite(requestedPitchDeg) && std::isfinite(requestedRollDeg);

            if (!parameterValid)
            {
                pShm->Coord_Command.reqSetManualFrame = false; // 無效命令直接丟棄
            }
            else
            {
                // 3. Determine Whether Manual XYZ May Be Moving
                // NCPLCManager 的 Manual Motion Gate 只允許 IDLE / READY。
                // 因此只有在 IDLE / READY 時，VELOCITY / MOVING / MPG 才需要視為 Manual Motion 並安全停止。
                // RUN 中的 NC Program 不受 Manual Frame 影響，不可以因為修改 Manual Frame 而停止加工。
                const NCState ncState = nc->GetState();
                const bool manualOperationState = (ncState == NCState::IDLE || ncState == NCState::READY);
                bool waitingForManualStop = false;

                const MotionOwnerLease manualLease =
                    nc->GetMotion().GetMotionOwnerLease();
                const bool manualOwnerActive =
                    nc->GetMotion().IsMotionOwnerLeaseCurrent(manualLease) &&
                    (manualLease.owner == MotionOwner::JOG ||
                        manualLease.owner == MotionOwner::MPG);
                const bool incompatibleOwnerActive =
                    manualLease.IsValid() && !manualOwnerActive;

                // HOME / AUTO / SAFETY motion must never be stopped by a
                // Manual Frame request. Keep the request pending until that
                // owner has released Motion.
                if (manualOperationState &&
                    (nc->Homing.IsActive() || incompatibleOwnerActive))
                {
                    waitingForManualStop = true;
                }
                else if (manualOperationState && manualOwnerActive)
                {
                    const MotionCommandSource manualSource =
                        ResolveMotionCommandSourceForOwner(manualLease.owner);

                    // Manual Frame 只作用 XYZ，所以只處理 Machine X/Y/Z。
                    for (int axisIndex = 0; axisIndex < 3; ++axisIndex)
                    {
                        AxisContext& axis = nc->GetMotion().GetAxisContext(axisIndex);
                        if (!axis.isExist) continue;

                        if (axis.state == MotionState::MotionState_VELOCITY) // Continuous / Fine JOG
                        {
                            nc->GetMotion().SubmitAxisStopMove(axisIndex, axis.JOG_dec_time, manualSource, manualLease);
                            waitingForManualStop = true;
                            continue;
                        }

                        if (axis.state == MotionState::MotionState_MOVING) // INCH JOG
                        {
                            nc->GetMotion().SubmitAxisStopMove(axisIndex, axis.INCH_dec_time, manualSource, manualLease);
                            waitingForManualStop = true;
                            continue;
                        }

                        if (axis.state == MotionState::MotionState_MPG) // MPG
                        {
                            nc->GetMotion().SubmitAxisStopMove(axisIndex, axis.JOG_dec_time, manualSource, manualLease);
                            waitingForManualStop = true;
                            continue;
                        }

                        if (axis.state == MotionState::MotionState_STOPPING) // Stop still in progress
                        {
                            waitingForManualStop = true;
                            continue;
                        }
                    }
                }

                // 4. Apply Manual Frame
                // Manual Motion 尚未完全停止: Request 保持 true，下一個 ProcessTask Scan 再檢查。
                // 全部停止: 正式套用。
                if (!waitingForManualStop)
                {
                    nc->CoordSys.SetManualFrameAngles(requestedYawDeg, requestedPitchDeg, requestedRollDeg);
                    nc->CoordSys.SetManualFrameEnabled(requestedEnabled); // Enabled 最後才更新，避免先 Enable 但 Angle 尚未完整更新的短暫狀態
                    pShm->Coord_Command.reqSetManualFrame = false; // Request 最後清除
                }
            }
        }

        //PLC----------------------------------------------------------------------
        if (g_PLC != nullptr)
        {
            pShm->PLC_Status.SHM_PLC_RunCount = g_PLC->PLC_RunCount;

            // PLC 狀態全廣播
            g_PLC->ExportPLCStatus(&pShm->PLC_Status);
        }

        // 🌟 處理 PLC 點位寫入
        if (pShm->PLC_Command.writeReq)
        {
            if (g_PLC != nullptr)
            {
                g_PLC->SetMemory(pShm->PLC_Command.regionPrefix, pShm->PLC_Command.index, pShm->PLC_Command.writeValue);
            }
            pShm->PLC_Command.writeReq = false;
        }

        // 🌟 處理 PLC 變數名稱直接寫入
        if (pShm->PLC_Command.writeByNameReq)
        {
            pShm->PLC_Command.varName[63] = '\0';
            if (g_PLC != nullptr)
            {
                g_PLC->SetVar(std::string(pShm->PLC_Command.varName), pShm->PLC_Command.writeValue);
            }
            pShm->PLC_Command.writeByNameReq = false;
        }

    }

    // =========================================================================
    // 🐢 2. ProcessTask_100ms - 處理大資料表格與字串 (HMI 畫面刷新用)
    // =========================================================================
    void ProcessTask_100ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;


        // --- 警報檢查 (自帶條件判斷，極快) ---
       // 1. 取得最新狀態
        uint32_t currentUpdateCount = AlarmManager::GetInstance().GetUpdateCount();

        // 2. 如果計數器有變，代表有新警報或警報剛被清除
        if (pShm->Alarm_Status.alarmUpdateCount != currentUpdateCount)
        {
            auto& am = AlarmManager::GetInstance();

            // 更新數量
            pShm->Alarm_Status.activeAlarmCount = am.GetAlarmCount();

            // 清空舊資料
            std::memset(pShm->Alarm_Status.activeAlarms, 0, sizeof(pShm->Alarm_Status.activeAlarms));

            // 🌟 將軸索引預設填滿 -1
            std::fill(std::begin(pShm->Alarm_Status.activeAlarmAxes),
                std::end(pShm->Alarm_Status.activeAlarmAxes), -1);

            // 🌟 把錯誤碼和軸號，一對一打包送進 SHM
            for (int i = 0; i < pShm->Alarm_Status.activeAlarmCount; i++) {
                pShm->Alarm_Status.activeAlarms[i] = am.GetAlarmId(i);
                pShm->Alarm_Status.activeAlarmAxes[i] = am.GetAlarmAxisIndex(i); // 抓取軸號
            }

            // 更新完成
            pShm->Alarm_Status.alarmUpdateCount = currentUpdateCount;
        }



        // --- 字串複製 (程式名稱) ---
        std::strncpy(pShm->NC_Status.mainProgName, nc->m_mainProgramName.c_str(), 63);
        pShm->NC_Status.mainProgName[63] = '\0';

        pShm->NC_Status.mainCurrentLine = nc->GetActivePC();
        std::strncpy(pShm->NC_Status.macroProgName, nc->m_macroProgramName.c_str(), 63);
        pShm->NC_Status.macroProgName[63] = '\0';


        // ==========================================================
            // 🌟 雙指標神同步 (Dual PC Synchronization) - 智慧混合版
            // ==========================================================

            // 1. 取得大腦的指標 (Interpreter PC)
        int interpreterMainPC = nc->GetBasePC();
        int interpreterMacroPC = nc->m_macroProgramPC;

        // 2. 取得實體馬達的指標 (Motion PC)
        int physicalPC = nc->GetMotion().GetPhysicalExecutionPC();

        // 3. 🌟 【神級判斷】：手腳追上大腦了嗎？
        // 如果 IsGroupDone() 為 true，代表底層倉庫全空，馬達完全靜止。
        // 這意味著目前的指令是「非運動指令」(如 M00, G04, 巨集變數)，機台正在執行大腦的狀態！
        bool isMachineIdle = nc->GetMotion().IsGroupDone();

        // 4. 判斷目前這張單子是屬於主程式還是副程式
        bool isMacroRunning = !nc->m_macroStack.empty();

        if (isMacroRunning) {
            // 在跑副程式
            // 🌟 如果機台靜止，游標顯示大腦卡住的地方(如 M00)；如果機台在動，顯示馬達正在跑的路徑
            pShm->NC_Status.macroCurrentLine = isMachineIdle ? interpreterMacroPC : physicalPC;
            pShm->NC_Status.mainCurrentLine = interpreterMainPC; // 主程式永遠顯示呼叫副程式的那一行
        }
        else {
            // 在跑主程式
            pShm->NC_Status.mainCurrentLine = isMachineIdle ? interpreterMainPC : physicalPC;
            pShm->NC_Status.macroCurrentLine = -1;
        }

        int interpreterWCS = nc->CoordSys.GetCurrentWCSGCode(); // 大腦的座標系
        int physicalWCS = nc->GetMotion().GetPhysicalExecutionWCS(); // 馬達的座標系

        pShm->NC_Status.currentWCS_GCode = isMachineIdle ? interpreterWCS : physicalWCS;//坐標系


        // 3. 🌟 刀具長度補正顯示 (Tool Length Comp)
        int interpreterToolMode = nc->CoordSys.toolLengthMode;
        int physicalToolMode = nc->GetMotion().GetPhysicalExecutionToolMode();

        int interpreterHCode = nc->CoordSys.currentHCode;
        int physicalHCode = nc->GetMotion().GetPhysicalExecutionHCode();

        //刀具號
        pShm->NC_Status.currentTCode = nc->CoordSys.currentTCode;
        //工件號
        pShm->NC_Status.currentWorkpieceNum = nc->CoordSys.currentWorkpieceNum;

        // 如果機台靜止(大腦卡住)，顯示大腦狀態；如果機台在跑，顯示馬達標籤狀態！
        // 假設 pShm->NC_Status 有這兩個變數供 UI 綁定
        pShm->NC_Status.currentToolLengthMode = isMachineIdle ? interpreterToolMode : physicalToolMode;
        pShm->NC_Status.currentHCode = isMachineIdle ? interpreterHCode : physicalHCode;
        // Frozen inverse coordinates and their modal labels share one source,
        // including HOLD and RESET cleanup before complete retirement.
        if (nc->CoordSys.IsTranslationRunFrozen())
        {
            const NCTranslationSnapshot frozenDisplay = nc->CoordSys.GetTranslationSnapshot();
            if (IsNCTranslationSnapshotValid(frozenDisplay))
            {
                pShm->NC_Status.currentWCS_GCode = frozenDisplay.wcsCode;
                pShm->NC_Status.currentToolLengthMode = frozenDisplay.toolLengthMode;
                pShm->NC_Status.currentHCode = frozenDisplay.toolHCode;
            }
        }

        // 4. 🌟 刀徑補正顯示 (Tool Radius Comp)
        int interpreterTRadMode = nc->CoordSys.toolRadiusMode;
        int physicalTRadMode = nc->GetMotion().GetPhysicalExecutionToolRadiusMode();

        int interpreterDCode = nc->CoordSys.currentDCode;
        int physicalDCode = nc->GetMotion().GetPhysicalExecutionDCode();

        // 🌟 取得大腦與馬達的 isAbsoluteMode
        bool interpreterAbs = nc->CoordSys.isAbsoluteMode;
        bool physicalAbs = nc->GetMotion().GetPhysicalExecutionIsAbsoluteMode();

        // 根據機台是否靜止，決定 UI 要聽誰的，然後轉成 1 或 0 傳給 HMI
        bool finalAbs = isMachineIdle ? interpreterAbs : physicalAbs;
        pShm->NC_Status.isAbsoluteMode = finalAbs ? 1 : 0;


        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentToolRadiusMode = isMachineIdle ? interpreterTRadMode : physicalTRadMode;
        pShm->NC_Status.currentDCode = isMachineIdle ? interpreterDCode : physicalDCode;


        // 🌟 G68 旋轉狀態與角度顯示
        bool interpreterG68 = nc->CoordSys.isG68Active;
        bool physicalG68 = nc->GetMotion().GetPhysicalExecutionG68Active();

        double interpreterG68Angle = nc->CoordSys.g68Angle;
        double physicalG68Angle = nc->GetMotion().GetPhysicalExecutionG68Angle();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG68State = (isMachineIdle ? interpreterG68 : physicalG68) ? 1 : 0;
        pShm->NC_Status.currentG68Angle = isMachineIdle ? interpreterG68Angle : physicalG68Angle;



        // 🌟 G168 狀態與 W 碼顯示
        bool interpreterG168 = nc->CoordSys.isWorkpieceRotationActive;
        bool physicalG168 = nc->GetMotion().GetPhysicalExecutionG168Active();

        int interpreterWCode = nc->CoordSys.currentWCode;
        int physicalWCode = nc->GetMotion().GetPhysicalExecutionWCode();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG168State = (isMachineIdle ? interpreterG168 : physicalG168) ? 1 : 0;
        pShm->NC_Status.currentWCode = isMachineIdle ? interpreterWCode : physicalWCode;
        if (nc->CoordSys.IsTranslationRunFrozen())
        {
            const NCTranslationSnapshot frozenDisplay = nc->CoordSys.GetTranslationSnapshot();
            if (IsNCTranslationSnapshotValid(frozenDisplay))
            {
                pShm->NC_Status.currentG168State = frozenDisplay.workMode == 168 ? 1 : 0;
                pShm->NC_Status.currentWCode = frozenDisplay.workWCode;
                // Coordinate inverse and rotation labels use the same source,
                // also while RESET cleans live modes before source retirement.
                pShm->NC_Status.currentG68State = frozenDisplay.rotationMode == 68 ? 1 : 0;
                pShm->NC_Status.currentG68Angle = frozenDisplay.rotationAngleDeg;
            }
        }


        // 🌟 7. G51 縮放狀態顯示
        bool interpreterG51 = nc->CoordSys.isScalingActive;
        bool physicalG51 = nc->GetMotion().GetPhysicalExecutionG51Active();

        double interpreterScale = nc->CoordSys.scaleFactor;
        double physicalScale = nc->GetMotion().GetPhysicalExecutionScaleRatio();

        // 🌟 8. G151 / G150 鏡像狀態顯示

        // 算出大腦目前的 Mask
        uint8_t interpreterMirrorMask = 0;
        for (int i = 0; i < 8; i++) {
            if (nc->CoordSys.isMirrorActive[i]) {
                interpreterMirrorMask |= (1 << i);
            }
        }

        // 拿取馬達目前的 Mask
        uint8_t physicalMirrorMask = nc->GetMotion().GetPhysicalExecutionMirrorMask();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentMirrorMask = isMachineIdle ? interpreterMirrorMask : physicalMirrorMask;

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG51State = (isMachineIdle ? interpreterG51 : physicalG51) ? 1 : 0;
        pShm->NC_Status.currentScaleRatio = isMachineIdle ? interpreterScale : physicalScale;

        // 🌟 9. G16 極座標狀態顯示
        bool interpreterG16 = nc->CoordSys.isPolarCoordinateActive;
        bool physicalG16 = nc->GetMotion().GetPhysicalExecutionG16Active();

        // 寫入 SHM 供 HMI 讀取燈號
        bool finalG16 = isMachineIdle ? interpreterG16 : physicalG16;
        pShm->NC_Status.currentG16State = finalG16 ? 1 : 0;


        // 🌟 G162 偏心補償與 G17/18/19 平面顯示
        bool interpreterG162 = nc->CoordSys.isCAxisOffsetRotationEnabled;
        bool physicalG162 = nc->GetMotion().GetPhysicalExecutionG162Active();

        int interpreterPlane = nc->CoordSys.activePlane;
        int physicalPlane = nc->GetMotion().GetPhysicalExecutionPlaneMode();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG162State = (isMachineIdle ? interpreterG162 : physicalG162) ? 1 : 0;
        pShm->NC_Status.currentPlaneMode = isMachineIdle ? interpreterPlane : physicalPlane;

        pShm->NC_Status.currentG20State = nc->CoordSys.isInchMode;

        bool m_programmableTravelLimitEnabled = nc->CoordSys.m_programmableTravelLimitEnabled;
        pShm->NC_Status.m_programmableTravelLimitEnabled = m_programmableTravelLimitEnabled;





        if (pShm->String_Command.reqLoadCode)//處理動態程式碼載入請求 (來自 String_Command)
        {
            pShm->String_Command.codeContent[2047] = '\0'; // 安全結尾防溢位
            std::string codeStr(pShm->String_Command.codeContent);

            // 呼叫統一的 API，它會自己看現在是 MDI 還是 MANUAL
            nc->LoadDynamicCode(codeStr);

            // 處理完畢，降下旗標並清空緩衝區
            pShm->String_Command.reqLoadCode = false;
            std::memset(pShm->String_Command.codeContent, 0, sizeof(pShm->String_Command.codeContent));
        }

        // =========================================================
             // 🌟 處理 HMI 寫入座標偏移量 (reqWriteOffset)
             // =========================================================
        if (pShm->Coord_Command.reqWriteOffset)
        {
            bool offsetWriteAccepted = false;
            const int row = pShm->Coord_Command.rowIndex;
            const int axis = pShm->Coord_Command.axisIndex;
            const int type = pShm->Coord_Command.offsetType;
            const double requestedValue = pShm->Coord_Command.writeValue;
            if (axis < 0 || axis >= 8)
            {
                offsetWriteAccepted = nc->CoordSys.ApplyCoordinateTableValues(type, row, nullptr, nullptr, nc, false);
            }
            else
            {
                const double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;
                const AxisType axisType = nc->m_motion.GetAxisContext(axis).axisType;
                const double axisScale = (axisType == AxisType::ROTARY ||
                    axisType == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;
                // WORK columns describe XYZ / angles / reserved, not axes.
                const double fieldScale = type == 3 ? (axis < 3 ? unitScale : 1.0) : axisScale;
                const double internalValue = requestedValue / fieldScale;
                if (type == 10)
                {
                    offsetWriteAccepted = nc->CoordSys.ApplyCoordinateOrigin(axis, internalValue, nc, false);
                }
                else
                {
                    bool fields[8] = {};
                    double values[8] = {};
                    fields[axis] = true;
                    values[axis] = internalValue;
                    offsetWriteAccepted = nc->CoordSys.ApplyCoordinateTableValues(type, row, fields, values, nc, false);
                }
            }
            if (!offsetWriteAccepted && pShm->Coord_Command.reqSave)
            {
                RtPrintf("[COORD][REJECT] op=HMI_SAVE reason=EDIT_REJECTED saveType=%d\n",
                    pShm->Coord_Command.saveType);
                pShm->Coord_Command.reqSave = false;
            }
            // Rejection is explicit in the console; no ABI field is added and
            // a rejected HMI edit neither changes the run nor raises an alarm.
            pShm->Coord_Command.reqWriteOffset = false;
        }

        if (pShm->Coord_Command.reqSave)
        {
            switch (pShm->Coord_Command.saveType)
            {
            case 0: nc->CoordSys.SaveAllParameters(); break;
            case 1: nc->CoordSys.SaveWCSStatus(); break;
            case 2: nc->CoordSys.SaveExtOffset(); break;
            case 3: nc->CoordSys.SaveWCSTable(); break;
            case 4: nc->CoordSys.SaveToolOffset(); break;
            case 5: nc->CoordSys.SaveWorkOffset(); break;
            }
            pShm->Coord_Command.reqSave = false;
        }





        //軸狀態-------------------------------------------------------
        // =======================================================
        // 🌟 [修改這裡] 軸狀態：直接請 MotionCore 把資料填入 pShm 
        // =======================================================
        // 取代你原本手寫的 for 迴圈與 m_pContexts
        nc->GetMotion().ExportDebugInfo(pShm->axisDebug, true);






    }

    // =========================================================================
    // 🚶 3. ProcessTask_500ms - 中慢速任務 (半秒 1次)
    // =========================================================================
    void ProcessTask_500ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;


        // 🌟 [新增] 1. 取得當前的公英制倍率
        double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;


        // --- Macro 變數全廣播 (約 18KB) ---
        pShm->macroStatus.currentCallDepth = nc->MacroSys.GetCurrentDepth();

        if (pShm->varCmd.refresh_global)
        {
            memcpy(pShm->macroStatus.globalVars, nc->MacroSys.GetGlobalVarsArray(), sizeof(pShm->macroStatus.globalVars));
            pShm->varCmd.refresh_global = false;
        }

        if (pShm->varCmd.refresh_system)
        {
            memcpy(pShm->macroStatus.sysVars, nc->MacroSys.GetSysVarsArray(), sizeof(pShm->macroStatus.sysVars));
            pShm->varCmd.refresh_system = false;
        }

        if (pShm->varCmd.refresh_local)
        {
            for (int i = 0; i < 8; i++)
            {
                memcpy(pShm->macroStatus.localVars[i], nc->MacroSys.GetLocalVarsArray(i), sizeof(double) * 101);
            }
            pShm->varCmd.refresh_local = false;
        }

        // =====================================================================
                // 🌟 2. 座標與補償表格拷貝 (捨棄 memcpy，改用迴圈逐軸套用單位轉換)
                // =====================================================================
        for (int col = 0; col < 8; col++)
        {
            // 判斷是否為旋轉軸
            AxisType type = nc->m_motion.GetAxisContext(col).axisType;
            double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

            // A. 外部偏移 (EXT)
            pShm->Coord_Table.extOffset[col] = nc->CoordSys.extOffset[col] * axisScale;

            // B. G54 ~ G59 表格
            for (size_t row = 0; row < 60 && row < nc->CoordSys.m_WCSTable.size(); ++row) {
                pShm->Coord_Table.wcsTable[row][col] = nc->CoordSys.m_WCSTable[row][col] * axisScale;
            }

            // C. 刀具長度/半徑補正表
            for (size_t row = 0; row < 100 && row < nc->CoordSys.m_ToolOffset.size(); ++row) {
                pShm->Coord_Table.toolOffset[row][col] = nc->CoordSys.m_ToolOffset[row][col] * axisScale;
            }

            // D. G168 獨立工件補正表
            for (size_t row = 0; row < 100 && row < nc->CoordSys.m_WorkOffset.size(); ++row) {
                const double workScale = col < 3 ? unitScale : 1.0;
                pShm->Coord_Table.workOffset[row][col] = nc->CoordSys.m_WorkOffset[row][col] * workScale;
            }
        }




        //PLC----------------------------------------------------------------------
        //重置PLC邏輯檔案
        if (pShm->PLC_Command.ReloadLogicProgram)
        {
            g_PLC->ReloadLogicProgram();//重置PLC邏輯檔案
            pShm->PLC_Command.ReloadLogicProgram = false;
        }

        // V7.5.3 - PLC Runtime Diagnostics read-only broadcast.
        // Kept outside SHM_PLC_Status so PLC_Command offset remains unchanged.
        if (g_PLC != nullptr)
        {
            const PLCRuntimeDiagnostics diag = g_PLC->GetRuntimeDiagnostics();

            pShm->PLC_Diagnostics.RuntimeFault = diag.faultActive ? 1 : 0;
            pShm->PLC_Diagnostics.LastFaultCode = diag.lastFaultCode;
            pShm->PLC_Diagnostics.LastOpcode = diag.lastOpcode;
            pShm->PLC_Diagnostics.LastOperandRegion = diag.lastOperandRegion;
            pShm->PLC_Diagnostics.LastTaskIndex = diag.lastTaskIndex;
            pShm->PLC_Diagnostics.LastInstructionIndex = diag.lastInstructionIndex;
            pShm->PLC_Diagnostics.LastOperandAddress = diag.lastOperandAddress;
            pShm->PLC_Diagnostics.LastFaultRunCount = diag.lastFaultRunCount;
            pShm->PLC_Diagnostics.TotalFaultCount = diag.totalFaultCount;
            pShm->PLC_Diagnostics.InvalidOperandCount = diag.invalidOperandCount;
            pShm->PLC_Diagnostics.InvalidTimerIndexCount = diag.invalidTimerIndexCount;
            pShm->PLC_Diagnostics.InvalidCounterIndexCount = diag.invalidCounterIndexCount;
            pShm->PLC_Diagnostics.UnknownOpcodeCount = diag.unknownOpcodeCount;
            pShm->PLC_Diagnostics.RuntimeStateMismatchCount = diag.runtimeStateMismatchCount;
            pShm->PLC_Diagnostics.InvalidFlowRowCount = diag.invalidFlowRowCount;

            // V7.6.0 - actual accepted logic.bin identity.
            const PLCLogicVerificationState logicState = g_PLC->GetLogicVerificationState();
            pShm->PLC_Diagnostics.LoadedLogicCrc32 = logicState.loadedLogicCrc32;
            pShm->PLC_Diagnostics.LoadedLogicSize = logicState.loadedLogicSize;
            pShm->PLC_Diagnostics.LogicLoadGeneration = logicState.logicLoadGeneration;
            pShm->PLC_Diagnostics.LastLogicLoadResult = logicState.lastLogicLoadResult;

            // V7.6.4.1 Runtime Scan Health
            const PLCScanHealth scanHealth = g_PLC->GetScanHealth();
            pShm->PLC_Diagnostics.ScanLastCycleUs = scanHealth.lastCycleUs;
            pShm->PLC_Diagnostics.ScanWorstCycleUs = scanHealth.worstCycleUs;
            pShm->PLC_Diagnostics.ScanCycleBudgetUs = scanHealth.cycleBudgetUs;
            pShm->PLC_Diagnostics.ScanCycleOverrunCount = scanHealth.cycleOverrunCount;
            pShm->PLC_Diagnostics.ScanLastTaskUs = scanHealth.lastTaskUs;
            pShm->PLC_Diagnostics.ScanWorstTaskUs = scanHealth.worstTaskUs;
            pShm->PLC_Diagnostics.ScanLastTaskBudgetUs = scanHealth.lastTaskBudgetUs;
            pShm->PLC_Diagnostics.ScanLastTaskIndex = scanHealth.lastTaskIndex;
            pShm->PLC_Diagnostics.ScanWorstTaskIndex = scanHealth.worstTaskIndex;
            pShm->PLC_Diagnostics.ScanTaskOverrunCount = scanHealth.taskOverrunCount;
            pShm->PLC_Diagnostics.ScanMeasuredCycleCount = scanHealth.measuredCycleCount;
            pShm->PLC_Diagnostics.ScanLastOverrunRunCount = scanHealth.lastOverrunRunCount;
            pShm->PLC_Diagnostics.ScanLastOverrunTaskIndex = scanHealth.lastOverrunTaskIndex;
        }


    }

    // =========================================================================
    // 🐌 3. ProcessTask_1000ms - 慢速背景任務 (1秒 1次)
    // =========================================================================
    void ProcessTask_1000ms(NCManager* nc)
    {
        if (nc == nullptr) return;
        MotionCore& motion = nc->GetMotion();
        // CO_FIX1: defer bulk output while GAP needs fresh supervisory service.
        // Early return preserves all ordinary diagnostic event tokens.
        if (PrintGapActiveCompactDiagnosticSameThread(
            *nc, motion, g_hmi1000msDiagnosticWorkspace)) return;
        // CJ FIX1: unconditional drain before SHM/ordinary diagnostic gates.
        DrainIdleHoldDiagnostics(motion);
        DrainCncP1Diagnostics(motion);
        DrainCncFeedPlanDiagnostics(motion);
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr) return;

        // =============================================================
        // Stage NC-0.1F.2 - No-motion Runtime Acceptance Diagnostics
        //
        // This code runs only in the existing 1000 ms supervisory task.
        // It never writes AxisContext and never consumes either command
        // or feedback rings.  It only reads atomic / snapshot counters.
        //
        // Output policy:
        //   1. Print one full startup sample, then full reports on events.
        //   2. Safety/transport/NC mode-state edges are events; an unchanged
        //      pending safety latch is NOT a reason to repeat the full dump.
        //   3. After ten quiet samples print one compact integer-only heartbeat.
        // Snapshot capture, bounded event drains and error tracking still run
        // every second. Static snapshot storage/noinline output families stay
        // intact; this gate does not change NC, safety, motion or PDO state.
        // =============================================================
        Hmi1000msDiagnosticWorkspace& workspace =
            g_hmi1000msDiagnosticWorkspace;
        CapturePathCoreCompactDiagnosticFamily(*nc, workspace);
        CaptureTransportDiagnosticFamily(motion, workspace);

        EtherCatPdoRuntimeInvalidCorrelationSnapshot&
            pdoInvalidCorrelation = workspace.pdoInvalidCorrelation;
        const bool pdoInvalidCorrelationCoherent =
            workspace.pdoInvalidCorrelationCoherent;

        EtherCatPdoSafetyStopCauseSnapshot& pdoSafetyStopCause =
            workspace.pdoSafetyStopCause;
        const bool pdoSafetyStopCauseCoherent =
            workspace.pdoSafetyStopCauseCoherent;

        MotionStartupLagArmingEvidence& startupLagArming = workspace.startupLagArming;
        MotionP1HandoverSafetySnapshot& p1HandoverSafety = workspace.p1HandoverSafety;
        MotionLifecycleCommitReservationSnapshot& lifecycleCommit = workspace.lifecycleCommit;
        MotionCommandPathModeTransportSnapshot& commandPathModeTransport = workspace.commandPathModeTransport;
        MotionQueueTailTransactionSnapshot& queueTailTransaction = workspace.queueTailTransaction;

        std::uint64_t p1HandoverChangeToken = 1469598103934665603ULL;
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.mappingBoundaryStops);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.droppedAxisRetirements);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.droppedAxisRetirementFailures);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.orphanAxisContainments);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.invalidProducerRejects);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.mappingIntegrityAlarmRequests);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.mappingIntegrityAlarmPending ? 1ULL : 0ULL);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.lastPreviousAxisMask);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.lastNextAxisMask);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            p1HandoverSafety.lastMappingIntegrityAlarmExecutionEpoch);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            static_cast<std::uint64_t>(
                p1HandoverSafety.lastOrphanAxisIndex + 1));
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.attempts);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.acquired);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.blockedByLifecycle);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.compareExchangeLost);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.released);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.releaseFailures);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.publisherWaits);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.currentExecutionEpoch);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.reservationActive ? 1ULL : 0ULL);
        p1HandoverChangeToken = FoldDiagnosticEventToken(
            p1HandoverChangeToken,
            lifecycleCommit.executionEpochPending ? 1ULL : 0ULL);

        const std::uint64_t commandPathModeChangeToken =
            BuildCommandPathModeChangeToken(commandPathModeTransport);

        const std::uint64_t queueTailTransactionChangeToken =
            BuildQueueTailTransactionChangeToken(queueTailTransaction);

        CaptureMotionSettleDiagnosticFamily(motion, workspace);

        MotionStopSettleSnapshot& stopSettleSnapshot = workspace.stopSettleSnapshot;
        MotionStopSettleCounters& stopSettleCounters = workspace.stopSettleCounters;
        MotionNCSettleSnapshot& groupNCSettleSnapshot = workspace.groupNCSettleSnapshot;
        MotionNCSettleCounters& groupNCSettleCounters = workspace.groupNCSettleCounters;
        const bool groupNCSettleCoherent =
            workspace.groupNCSettleCoherent;

        MotionNCSettleSnapshot& feedHoldNCSettleSnapshot = workspace.feedHoldNCSettleSnapshot;
        MotionNCSettleCounters& feedHoldNCSettleCounters = workspace.feedHoldNCSettleCounters;
        const bool feedHoldNCSettleCoherent =
            workspace.feedHoldNCSettleCoherent;

        MotionNCSettleSnapshot& resetNCSettleSnapshot = workspace.resetNCSettleSnapshot;
        MotionNCSettleCounters& resetNCSettleCounters = workspace.resetNCSettleCounters;
        const bool resetNCSettleCoherent =
            workspace.resetNCSettleCoherent;

        MotionNCResetRebaseAck& resetRebaseAck = workspace.resetRebaseAck;
        MotionOwnerLease& ownerLease = workspace.ownerLease;

        const bool safetyPending =
            motion.HasPendingSafetyOrRecoveryRequests();

        const std::size_t axisCommandDepth =
            motion.GetAxisCommandMailboxDepth();
        const std::size_t commandIngressDepth =
            motion.GetCommandIngressSize();
        const std::size_t commandReplayDepth =
            motion.GetCommandReplaySize();
        const std::size_t feedbackDepth =
            motion.GetMotionFeedbackDepth();
        const std::size_t feedbackNoticeDepth =
            motion.GetMotionFeedbackProducerNoticeDepth();

        const std::uint64_t axisQueueFull =
            motion.GetAxisCommandQueueFullCount();
        const std::uint64_t axisResultOverflow =
            motion.GetAxisCommandResultOverflowCount();
        const std::uint64_t ownerConflictReject =
            motion.GetMotionOwnerConflictRejectCount();
        const std::uint64_t staleDiscard =
            motion.GetStaleCommandDiscardCount();
        const std::uint64_t commandQueueFull =
            motion.GetCommandQueueFullRejectCount();
        const std::uint64_t replayOverflow =
            motion.GetCommandReplayOverflowCount();
        const std::uint64_t feedbackOverflow =
            motion.GetMotionFeedbackOverflowCount();
        const std::uint64_t feedbackNoticeOverflow =
            motion.GetMotionFeedbackProducerNoticeOverflowCount();

        const std::uint64_t feedbackProcessed =
            nc->GetProcessedMotionFeedbackCount();
        const std::uint64_t feedbackSequenceGap =
            nc->GetMotionFeedbackSequenceGapCount();
        const std::uint64_t feedbackAccepted =
            nc->GetAcceptedMotionFeedbackCount();
        const std::uint64_t feedbackStarted =
            nc->GetStartedMotionFeedbackCount();
        const std::uint64_t feedbackCompleted =
            nc->GetCompletedMotionFeedbackCount();
        const std::uint64_t feedbackRejected =
            nc->GetRejectedMotionFeedbackCount();
        const std::uint64_t feedbackCancelled =
            nc->GetCancelledMotionFeedbackCount();
        const std::uint64_t feedbackAborted =
            nc->GetAbortedMotionFeedbackCount();
        const std::uint64_t feedbackFaulted =
            nc->GetFaultedMotionFeedbackCount();

        CaptureNCBlockDiagnosticFamily(*nc, workspace);

        NCBlockLifecycleSnapshot& blockLifecycle = workspace.blockLifecycle;
        const bool hasBlockLifecycle = workspace.hasBlockLifecycle;
        NCBlockLifecycleCounters& blockCounters = workspace.blockCounters;
        NCBlockCompletionBoundarySnapshot& completionBoundary = workspace.completionBoundary;
        NCBlockCompletionBoundaryCounters& completionCounters = workspace.completionCounters;
        NCProgramEndGateSnapshot& programEndSnapshot = workspace.programEndSnapshot;
        NCProgramEndGateCounters& programEndCounters = workspace.programEndCounters;
        NCGMBlockTransactionSnapshot& gmTransactionSnapshot = workspace.gmTransactionSnapshot;
        NCGMBlockTransactionCounters& gmTransactionCounters = workspace.gmTransactionCounters;
        NCPreDispatchBarrierSnapshot& preDispatchBarrierSnapshot = workspace.preDispatchBarrierSnapshot;
        NCPreDispatchBarrierCounters& preDispatchBarrierCounters = workspace.preDispatchBarrierCounters;

        CaptureNCPreparedQueueDiagnosticFamily(*nc, workspace);
        CaptureNCPreparedAdmissionDiagnosticFamily(*nc, workspace);
        CaptureNCPreparedReadAheadDiagnosticFamily(*nc, workspace);
        CaptureNCPreparedHoldDiagnosticFamily(*nc, workspace);

        NCPreparedBlockQueueSnapshot& preparedQueueSnapshot = workspace.preparedQueueSnapshot;
        NCPreparedBlockQueueCounters& preparedQueueCounters = workspace.preparedQueueCounters;
        NCPreparedHeadEquivalenceSnapshot& preparedEquivalenceSnapshot = workspace.preparedEquivalenceSnapshot;
        NCPreparedHeadEquivalenceCounters& preparedEquivalenceCounters = workspace.preparedEquivalenceCounters;
        NCPreparedHeadCutoverSnapshot& preparedCutoverSnapshot = workspace.preparedCutoverSnapshot;
        NCPreparedHeadCutoverCounters& preparedCutoverCounters = workspace.preparedCutoverCounters;
        NCPreparedPreResolveAdmissionSnapshot& preparedPreResolveSnapshot = workspace.preparedPreResolveSnapshot;
        NCPreparedPreResolveAdmissionCounters& preparedPreResolveCounters = workspace.preparedPreResolveCounters;
        NCPreparedResolverBypassSnapshot& preparedResolverBypassSnapshot = workspace.preparedResolverBypassSnapshot;
        NCPreparedResolverBypassCounters& preparedResolverBypassCounters = workspace.preparedResolverBypassCounters;
        NCOrdinaryG00AdmissionSnapshot& ordinaryG00AdmissionSnapshot = workspace.ordinaryG00AdmissionSnapshot;
        NCOrdinaryG00AdmissionCounters& ordinaryG00AdmissionCounters = workspace.ordinaryG00AdmissionCounters;
        NCOrdinaryG00InflightRegistrySnapshot& ordinaryG00InflightRegistrySnapshot = workspace.ordinaryG00InflightRegistrySnapshot;
        NCOrdinaryG00InflightRegistryCounters& ordinaryG00InflightRegistryCounters = workspace.ordinaryG00InflightRegistryCounters;
        NCOrdinaryG00ReadAheadSnapshot& ordinaryG00ReadAheadSnapshot = workspace.ordinaryG00ReadAheadSnapshot;
        NCOrdinaryG00ReadAheadCounters& ordinaryG00ReadAheadCounters = workspace.ordinaryG00ReadAheadCounters;
        NCOrdinaryG00FeedHoldCohortSnapshot& ordinaryG00FeedHoldCohortSnapshot = workspace.ordinaryG00FeedHoldCohortSnapshot;
        NCOrdinaryG00FeedHoldCohortCounters& ordinaryG00FeedHoldCohortCounters = workspace.ordinaryG00FeedHoldCohortCounters;
        NCOrdinaryG00FeedHoldCohortCutoverSnapshot& ordinaryG00FeedHoldCohortCutoverSnapshot = workspace.ordinaryG00FeedHoldCohortCutoverSnapshot;
        NCOrdinaryG00FeedHoldCohortCutoverCounters& ordinaryG00FeedHoldCohortCutoverCounters = workspace.ordinaryG00FeedHoldCohortCutoverCounters;
        NCOrdinaryG00FeedHoldCohortRearmSnapshot& ordinaryG00FeedHoldCohortRearmSnapshot = workspace.ordinaryG00FeedHoldCohortRearmSnapshot;
        NCOrdinaryG00FeedHoldCohortRearmCounters& ordinaryG00FeedHoldCohortRearmCounters = workspace.ordinaryG00FeedHoldCohortRearmCounters;
        NCOrdinaryG00FeedHoldRearmCutoverSnapshot& ordinaryG00FeedHoldRearmCutoverSnapshot = workspace.ordinaryG00FeedHoldRearmCutoverSnapshot;
        NCOrdinaryG00FeedHoldRearmCutoverCounters& ordinaryG00FeedHoldRearmCutoverCounters = workspace.ordinaryG00FeedHoldRearmCutoverCounters;
        NCOrdinaryG00FeedHoldRollingRearmSnapshot& ordinaryG00FeedHoldRollingRearmSnapshot = workspace.ordinaryG00FeedHoldRollingRearmSnapshot;
        NCOrdinaryG00FeedHoldRollingRearmCounters& ordinaryG00FeedHoldRollingRearmCounters = workspace.ordinaryG00FeedHoldRollingRearmCounters;
        NCOrdinaryG00FeedHoldRollingCutoverSnapshot& ordinaryG00FeedHoldRollingCutoverSnapshot = workspace.ordinaryG00FeedHoldRollingCutoverSnapshot;
        NCOrdinaryG00FeedHoldRollingCutoverCounters& ordinaryG00FeedHoldRollingCutoverCounters = workspace.ordinaryG00FeedHoldRollingCutoverCounters;
        NCPreparedBlockEntrySnapshot& preparedQueueHead = workspace.preparedQueueHead;
        NCPreparedBlockEntrySnapshot& preparedQueueTail = workspace.preparedQueueTail;
        const bool hasPreparedQueueHead = workspace.hasPreparedQueueHead;
        const bool hasPreparedQueueTail = workspace.hasPreparedQueueTail;

        CaptureNCHoldSafetyDiagnosticFamily(*nc, workspace);

        NCSingleBlockShadowSnapshot& singleBlockShadowSnapshot = workspace.singleBlockShadowSnapshot;
        NCSingleBlockShadowCounters& singleBlockShadowCounters = workspace.singleBlockShadowCounters;
        NCSingleBlockHoldGateSnapshot& singleBlockGateSnapshot = workspace.singleBlockGateSnapshot;
        NCSingleBlockHoldGateCounters& singleBlockGateCounters = workspace.singleBlockGateCounters;
        NCFeedHoldBoundarySnapshot& feedHoldSnapshot = workspace.feedHoldSnapshot;
        NCFeedHoldBoundaryCounters& feedHoldCounters = workspace.feedHoldCounters;
        NCFeedHoldResumeGateSnapshot& feedHoldGateSnapshot = workspace.feedHoldGateSnapshot;
        NCFeedHoldResumeGateCounters& feedHoldGateCounters = workspace.feedHoldGateCounters;
        NCLifecycleInterruptionSnapshot& lifecycleInterruptionSnapshot = workspace.lifecycleInterruptionSnapshot;
        NCLifecycleInterruptionCounters& lifecycleInterruptionCounters = workspace.lifecycleInterruptionCounters;
        NCResetReleaseGateSnapshot& resetReleaseGateSnapshot = workspace.resetReleaseGateSnapshot;
        NCResetReleaseGateCounters& resetReleaseGateCounters = workspace.resetReleaseGateCounters;
        NCAlarmEmergencyStopSnapshot& alarmEmergencyStopSnapshot = workspace.alarmEmergencyStopSnapshot;
        NCAlarmEmergencyStopCounters& alarmEmergencyStopCounters = workspace.alarmEmergencyStopCounters;

        const std::uint64_t lifecycleChangeToken =
            blockCounters.dispatched +
            blockCounters.programCommitted +
            blockCounters.motionSegmentsBound +
            blockCounters.feedbackAccepted +
            blockCounters.feedbackStarted +
            blockCounters.feedbackCompleted +
            blockCounters.feedbackRejected +
            blockCounters.feedbackCancelled +
            blockCounters.feedbackAborted +
            blockCounters.feedbackFaulted +
            blockCounters.blockCompleted +
            blockCounters.blockFailed +
            blockCounters.orphanFeedback +
            blockCounters.motionCaptureOverflow;

        const std::uint64_t completionBoundaryChangeToken =
            completionCounters.bindings +
            completionCounters.releaseChecks +
            completionCounters.agreeRelease +
            completionCounters.ledgerReadyBeforeLegacy +
            completionCounters.legacyEarlyRelease +
            completionCounters.ledgerFailureObserved +
            completionCounters.releaseOnLedgerFailure +
            completionCounters.trackingOverflow +
            completionCounters.missingLifecycle +
            completionCounters.nonMotionWait +
            completionCounters.supersededBindings +
            completionCounters.dualKeyRelease +
            completionCounters.blockedLegacyEarly +
            completionCounters.blockedLedgerFailure +
            completionCounters.blockedTrackingOverflow +
            completionCounters.blockedMissingLifecycle +
            completionCounters.blockedNotTracked +
            completionCounters.failClosedBindings;

        const std::uint64_t programEndChangeToken =
            programEndSnapshot.sequence +
            programEndCounters.runStartAttempts +
            programEndCounters.requests +
            programEndCounters.evaluations +
            programEndCounters.readyToFinalize +
            programEndCounters.finalized +
            programEndCounters.cancelled +
            programEndCounters.failClosed;

        const std::uint64_t gmTransactionChangeToken =
            gmTransactionSnapshot.sequence +
            gmTransactionCounters.started +
            gmTransactionCounters.evaluations +
            gmTransactionCounters.readyTransitions +
            gmTransactionCounters.finalized +
            gmTransactionCounters.cancelled +
            gmTransactionCounters.finalizeFailed;

        const std::uint64_t preDispatchBarrierChangeToken =
            preDispatchBarrierSnapshot.sequence +
            preDispatchBarrierCounters.activations +
            preDispatchBarrierCounters.evaluations +
            preDispatchBarrierCounters.cleared;

        // Exclude the observer's scan/publication counters from this token.
        // They advance by design; K.1 diagnostics should print when the
        // Prepared window or one of its proofs actually changes.
        const std::uint64_t preparedQueueChangeToken =
            preparedQueueSnapshot.session +
            preparedQueueCounters.sessions +
            preparedQueueCounters.prepared +
            preparedQueueCounters.dispatchMatched +
            preparedQueueCounters.commitMatched +
            preparedQueueCounters.retired +
            preparedQueueCounters.invalidatedEntries +
            preparedQueueCounters.invalidations +
            preparedQueueCounters.correlatedEpochAdvances +
            preparedQueueCounters.barrierStops +
            preparedQueueCounters.eofStops +
            preparedQueueCounters.capacityStops +
            preparedQueueCounters.overwritePrevented +
            preparedQueueCounters.cursorRegressions +
            preparedQueueCounters.planDiscontinuities +
            preparedQueueCounters.expectedFlowCutovers +
            preparedQueueCounters.dispatchMismatches +
            preparedQueueCounters.commitMismatches +
            preparedQueueCounters.staleRuntimeProofs +
            preparedQueueCounters.identityFailures +
            preparedQueueCounters.cutoverAttempts;

        // K.2 scan/publication/replay counters advance during normal
        // observation and are deliberately excluded.  Terminal proof,
        // Session qualification and every failure counter must force a log.
        std::uint64_t preparedEquivalenceStateBits = 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.candidate ? (1ULL << 0U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.pending ? (1ULL << 1U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.resolved ? (1ULL << 2U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.dispatchBound ? (1ULL << 3U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.commitBound ? (1ULL << 4U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.ledgerDispatchMatch
            ? (1ULL << 5U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.ledgerCommitMatch
            ? (1ULL << 6U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.upstreamDispatchCommitMatch
            ? (1ULL << 7U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.retirementMatch
            ? (1ULL << 8U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.upstreamProofMatch
            ? (1ULL << 9U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.matched ? (1ULL << 10U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.readinessQualified
            ? (1ULL << 11U) : 0ULL;
        preparedEquivalenceStateBits |=
            preparedEquivalenceSnapshot.runtimeWaitCallbackActive
            ? (1ULL << 12U) : 0ULL;

        const std::uint64_t preparedEquivalenceChangeToken =
            BuildPreparedDiagnosticsChangeToken(
                workspace,
                preparedEquivalenceStateBits);

        const std::uint64_t singleBlockShadowChangeToken =
            singleBlockShadowSnapshot.sequence +
            singleBlockShadowCounters.armAttempts +
            singleBlockShadowCounters.armed +
            singleBlockShadowCounters.notEligible +
            singleBlockShadowCounters.evaluations +
            singleBlockShadowCounters.boundaryReady +
            singleBlockShadowCounters.legacyHolds +
            singleBlockShadowCounters.agreeHolds +
            singleBlockShadowCounters.legacyEarlyHolds +
            singleBlockShadowCounters.legacyHoldWithoutArm +
            singleBlockShadowCounters.legacyMissingHold +
            singleBlockShadowCounters.controlledHolds +
            singleBlockShadowCounters.controlledAgreeHolds +
            singleBlockShadowCounters.controlledEarlyHoldAttempts +
            singleBlockShadowCounters.controlledResumed +
            singleBlockShadowCounters.programEndSuppressed +
            singleBlockShadowCounters.cancelled;

        const std::uint64_t singleBlockGateChangeToken =
            singleBlockGateSnapshot.sequence +
            singleBlockGateSnapshot.boundarySequence +
            singleBlockGateCounters.requestAttempts +
            singleBlockGateCounters.controlledArms +
            singleBlockGateCounters.waitBoundarySamples +
            singleBlockGateCounters.holdReady +
            singleBlockGateCounters.holdApplied +
            singleBlockGateCounters.resumeApplied +
            singleBlockGateCounters.programEndSuppressed +
            singleBlockGateCounters.blockedMotionFailure +
            singleBlockGateCounters.blockedTransactionFailure +
            singleBlockGateCounters.blockedTrackingOverflow +
            singleBlockGateCounters.blockedCancelled +
            singleBlockGateCounters.cancelled +
            singleBlockGateCounters.superseded +
            singleBlockGateCounters.rollbackDisabled;

        const std::uint64_t feedHoldChangeToken =
            feedHoldSnapshot.sequence +
            feedHoldCounters.requestAttempts +
            feedHoldCounters.requestsLatched +
            feedHoldCounters.evaluations +
            feedHoldCounters.acknowledged +
            feedHoldCounters.legacyHolds +
            feedHoldCounters.legacyEarlyHolds +
            feedHoldCounters.resumeRequests +
            feedHoldCounters.resumed +
            feedHoldCounters.ownerChanged +
            feedHoldCounters.executionEpochChanged +
            feedHoldCounters.motionFault +
            feedHoldCounters.acknowledgeLost +
            feedHoldCounters.cancelled +
            feedHoldCounters.superseded;

        const std::uint64_t feedHoldGateChangeToken =
            feedHoldGateSnapshot.sequence +
            feedHoldGateSnapshot.boundarySequence +
            feedHoldGateCounters.requestAttempts +
            feedHoldGateCounters.deferredBeforeAcknowledge +
            feedHoldGateCounters.immediateAfterAcknowledge +
            feedHoldGateCounters.releaseOnAcknowledge +
            feedHoldGateCounters.resumeApplied +
            feedHoldGateCounters.blockedBoundaryFailed +
            feedHoldGateCounters.blockedBoundaryCancelled +
            feedHoldGateCounters.cancelled +
            feedHoldGateCounters.superseded +
            feedHoldGateCounters.rollbackDisabled;

        const std::uint64_t lifecycleInterruptionChangeToken =
            lifecycleInterruptionSnapshot.sequence +
            lifecycleInterruptionCounters.requestAttempts +
            lifecycleInterruptionCounters.requestsLatched +
            lifecycleInterruptionCounters.epochPublicationsObserved +
            lifecycleInterruptionCounters.alarmStopAcknowledgements +
            lifecycleInterruptionCounters.runtimeAlarmEpochChanges +
            lifecycleInterruptionCounters.expectedAlarmAborts +
            lifecycleInterruptionCounters.expectedAlarmPreReadRejects +
            lifecycleInterruptionCounters.expectedAlarmOwnerConflictRejects +
            lifecycleInterruptionCounters.expectedAlarmStaleEpochRejects +
            lifecycleInterruptionCounters.expectedResetPreReadRejects +
            lifecycleInterruptionCounters.expectedResetOwnerConflictRejects +
            lifecycleInterruptionCounters.expectedResetStaleEpochRejects +
            lifecycleInterruptionCounters.alarmStopsClosed +
            lifecycleInterruptionCounters.terminalRejected +
            lifecycleInterruptionCounters.terminalCancelled +
            lifecycleInterruptionCounters.terminalAborted +
            lifecycleInterruptionCounters.terminalFaulted +
            lifecycleInterruptionCounters.postInterruptionDispatch +
            lifecycleInterruptionCounters.evaluations +
            lifecycleInterruptionCounters.quiescent +
            lifecycleInterruptionCounters.evidenceGap +
            lifecycleInterruptionCounters.superseded;

        const std::uint64_t resetReleaseGateChangeToken =
            resetReleaseGateSnapshot.sequence +
            resetReleaseGateSnapshot.boundarySequence +
            resetReleaseGateCounters.armAttempts +
            resetReleaseGateCounters.armed +
            resetReleaseGateCounters.supersededArms +
            resetReleaseGateCounters.evaluations +
            resetReleaseGateCounters.waitQuiescence +
            resetReleaseGateCounters.releaseReady +
            resetReleaseGateCounters.releaseAttempts +
            resetReleaseGateCounters.released +
            resetReleaseGateCounters.blockedInvalidBoundary +
            resetReleaseGateCounters.blockedBoundarySequence +
            resetReleaseGateCounters.blockedBoundaryCause +
            resetReleaseGateCounters.blockedEvidenceGap +
            resetReleaseGateCounters.blockedSuperseded +
            resetReleaseGateCounters.blockedIncomplete +
            resetReleaseGateCounters.blockedEpochMismatch +
            resetReleaseGateCounters.blockedSafetyLease +
            resetReleaseGateCounters.blockedOwnerRelease +
            resetReleaseGateCounters.blockedPostInterruptionDispatch;

        const std::uint64_t alarmEmergencyStopChangeToken =
            alarmEmergencyStopSnapshot.sequence +
            static_cast<std::uint64_t>(alarmEmergencyStopSnapshot.phase) +
            static_cast<std::uint64_t>(alarmEmergencyStopSnapshot.decision) +
            alarmEmergencyStopSnapshot.requestPublishedCurrent +
            alarmEmergencyStopSnapshot.rtApplyCurrent +
            alarmEmergencyStopSnapshot.epochInvalidationCurrent +
            static_cast<std::uint64_t>(
                alarmEmergencyStopSnapshot.
                lastInvalidatedFromExecutionEpoch) +
            static_cast<std::uint64_t>(
                alarmEmergencyStopSnapshot.
                lastInvalidatedToExecutionEpoch) +
            alarmEmergencyStopSnapshot.lastInvalidationCount +
            alarmEmergencyStopCounters.requestAttempts +
            alarmEmergencyStopCounters.requestsLatched +
            alarmEmergencyStopCounters.acknowledged +
            alarmEmergencyStopCounters.preLatchedCorrelations +
            alarmEmergencyStopCounters.preLatchedAcknowledged +
            alarmEmergencyStopCounters.clearedBeforeAcknowledge +
            alarmEmergencyStopCounters.lifecycleEvidenceGap +
            alarmEmergencyStopCounters.lifecycleReplaced +
            alarmEmergencyStopCounters.axisScopeChanged +
            alarmEmergencyStopCounters.superseded;

        const std::uint64_t transportRawErrorTotal =
            axisQueueFull +
            axisResultOverflow +
            ownerConflictReject +
            staleDiscard +
            commandQueueFull +
            replayOverflow +
            feedbackOverflow +
            feedbackNoticeOverflow +
            feedbackSequenceGap +
            feedbackRejected +
            feedbackFaulted;

        // NC-0.2J.6.3.1 / NC-0.2K.2.2.1: retain layered transport accounting
        // (cause counter plus feedback terminal) for every unexpected failure.
        // Remove both layers only for cumulative Alarm or RESET pre-read
        // retirements proved by their exact lifecycle boundary.  RESET credit
        // is committed only after QUIESCENT_PROVED; unrelated errors remain
        // fail-closed.
        const std::uint64_t expectedOwnerConflictReject =
            AddDiagnosticCounterSaturating(
                lifecycleInterruptionCounters.
                expectedAlarmOwnerConflictRejects,
                lifecycleInterruptionCounters.
                expectedResetOwnerConflictRejects);
        const std::uint64_t expectedStaleDiscard =
            AddDiagnosticCounterSaturating(
                lifecycleInterruptionCounters.
                expectedAlarmStaleEpochRejects,
                lifecycleInterruptionCounters.
                expectedResetStaleEpochRejects);
        const std::uint64_t expectedFeedbackRejected =
            AddDiagnosticCounterSaturating(
                lifecycleInterruptionCounters.
                expectedAlarmPreReadRejects,
                lifecycleInterruptionCounters.
                expectedResetPreReadRejects);
        const std::uint64_t unexpectedOwnerConflictReject =
            SubtractDiagnosticCounterFloor(
                ownerConflictReject,
                expectedOwnerConflictReject);
        const std::uint64_t unexpectedStaleDiscard =
            SubtractDiagnosticCounterFloor(
                staleDiscard,
                expectedStaleDiscard);
        const std::uint64_t unexpectedFeedbackRejected =
            SubtractDiagnosticCounterFloor(
                feedbackRejected,
                expectedFeedbackRejected);
        const std::uint64_t transportErrorTotal =
            axisQueueFull +
            axisResultOverflow +
            unexpectedOwnerConflictReject +
            unexpectedStaleDiscard +
            commandQueueFull +
            replayOverflow +
            feedbackOverflow +
            feedbackNoticeOverflow +
            feedbackSequenceGap +
            unexpectedFeedbackRejected +
            feedbackFaulted;

        const std::uint64_t j5SetInvalid =
            groupNCSettleCounters.invalidRuntimeCycleCount +
            feedHoldNCSettleCounters.invalidRuntimeCycleCount +
            resetNCSettleCounters.invalidRuntimeCycleCount;
        const std::uint64_t j5SetGap =
            groupNCSettleCounters.runtimeGapCount +
            feedHoldNCSettleCounters.runtimeGapCount +
            resetNCSettleCounters.runtimeGapCount;
        const std::uint64_t j5SetReject =
            groupNCSettleCounters.requestRejectCount +
            feedHoldNCSettleCounters.requestRejectCount +
            resetNCSettleCounters.requestRejectCount;
        const std::uint64_t j5SetReadFailure =
            groupNCSettleCounters.publicationReadFailureCount +
            feedHoldNCSettleCounters.publicationReadFailureCount +
            resetNCSettleCounters.publicationReadFailureCount +
            (groupNCSettleCoherent ? 0ULL : 1ULL) +
            (feedHoldNCSettleCoherent ? 0ULL : 1ULL) +
            (resetNCSettleCoherent ? 0ULL : 1ULL);

        const std::uint64_t j5RebaseFailure =
            (resetRebaseAck.blocked ? 1ULL : 0ULL) +
            (resetRebaseAck.unsupportedFaultOrEstop ? 1ULL : 0ULL) +
            (resetRebaseAck.compensationBlocked ? 1ULL : 0ULL);

        const std::uint64_t j5ResetGateFailure =
            resetReleaseGateCounters.blockedInvalidBoundary +
            resetReleaseGateCounters.blockedBoundarySequence +
            resetReleaseGateCounters.blockedBoundaryCause +
            resetReleaseGateCounters.blockedEvidenceGap +
            resetReleaseGateCounters.blockedSuperseded +
            resetReleaseGateCounters.blockedIncomplete +
            resetReleaseGateCounters.blockedEpochMismatch +
            resetReleaseGateCounters.blockedSafetyLease +
            resetReleaseGateCounters.blockedOwnerRelease +
            resetReleaseGateCounters.blockedPostInterruptionDispatch +
            resetReleaseGateCounters.ackMismatch +
            resetReleaseGateCounters.rebaseFailed;

        const std::uint64_t j5FeedHoldFailure =
            feedHoldCounters.ownerChanged +
            feedHoldCounters.executionEpochChanged +
            feedHoldCounters.motionFault +
            feedHoldCounters.acknowledgeLost +
            feedHoldGateCounters.legacyBypassDisabled +
            feedHoldGateCounters.blockedBoundaryFailed +
            feedHoldGateCounters.blockedBoundaryCancelled +
            feedHoldGateCounters.rollbackDisabled;

        const std::uint64_t j5ProgramEndFailure =
            programEndCounters.runStartBlocked +
            programEndCounters.rejectedRequests +
            programEndCounters.failClosed +
            programEndCounters.epochMismatch +
            programEndCounters.ownerLeaseLost +
            programEndCounters.integrityFailure;

        const std::uint64_t j5FailureTotal =
            j5SetInvalid +
            j5SetGap +
            j5SetReject +
            j5SetReadFailure +
            j5RebaseFailure +
            j5ResetGateFailure +
            j5FeedHoldFailure +
            j5ProgramEndFailure +
            transportErrorTotal;

        const bool pdoInvalidProfileCorrelationMatched =
            pdoInvalidCorrelationCoherent &&
            groupNCSettleCoherent &&
            feedHoldNCSettleCoherent &&
            resetNCSettleCoherent &&
            pdoInvalidCorrelation.validityContractMismatchCount == 0ULL &&
            groupNCSettleCounters.invalidRuntimeCycleCount ==
            pdoInvalidCorrelation.invalidCycleCount &&
            feedHoldNCSettleCounters.invalidRuntimeCycleCount ==
            pdoInvalidCorrelation.invalidCycleCount &&
            resetNCSettleCounters.invalidRuntimeCycleCount ==
            pdoInvalidCorrelation.invalidCycleCount &&
            groupNCSettleCounters.runtimeGapCount ==
            pdoInvalidCorrelation.expectedNCSettleGapCount &&
            feedHoldNCSettleCounters.runtimeGapCount ==
            pdoInvalidCorrelation.expectedNCSettleGapCount &&
            resetNCSettleCounters.runtimeGapCount ==
            pdoInvalidCorrelation.expectedNCSettleGapCount;

        const std::uint32_t pdoInvalidDisplayedReasonMask =
            pdoInvalidCorrelation.lastEventKind ==
            EtherCatPdoRuntimeInvalidEventKind::
            VALIDITY_CONTRACT_MISMATCH
            ? pdoInvalidCorrelation.lastEventReasonMask
            : pdoInvalidCorrelation.lastInvalidReasonMask;

        std::uint64_t j5EventToken = 1469598103934665603ULL;
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            BuildNCSettleDiagnosticEventToken(
                groupNCSettleCoherent,
                groupNCSettleSnapshot,
                groupNCSettleCounters));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            BuildNCSettleDiagnosticEventToken(
                feedHoldNCSettleCoherent,
                feedHoldNCSettleSnapshot,
                feedHoldNCSettleCounters));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            BuildNCSettleDiagnosticEventToken(
                resetNCSettleCoherent,
                resetNCSettleSnapshot,
                resetNCSettleCounters));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoInvalidCorrelationCoherent ? 1ULL : 0ULL);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoInvalidCorrelation.eventSequence);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoInvalidCorrelation.validityContractMismatchCount);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoInvalidProfileCorrelationMatched ? 1ULL : 0ULL);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoSafetyStopCauseCoherent ? 1ULL : 0ULL);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoSafetyStopCause.publicationSequence);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            pdoSafetyStopCause.alarmRequestCount);

        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetRebaseAck.requestSequence);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            static_cast<std::uint64_t>(resetRebaseAck.phase));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            static_cast<std::uint64_t>(resetRebaseAck.failureBlocker));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetRebaseAck.executionEpoch);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetRebaseAck.ownerGeneration);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetRebaseAck.requestedAxisMask);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetRebaseAck.appliedAxisMask);

        std::uint64_t rebaseStateBits = 0ULL;
        rebaseStateBits |= resetRebaseAck.requestAccepted ? (1ULL << 0U) : 0ULL;
        rebaseStateBits |= resetRebaseAck.rebaseApplied ? (1ULL << 1U) : 0ULL;
        rebaseStateBits |= resetRebaseAck.postVerifyPassed ? (1ULL << 2U) : 0ULL;
        rebaseStateBits |= resetRebaseAck.acknowledged ? (1ULL << 3U) : 0ULL;
        rebaseStateBits |= resetRebaseAck.acked ? (1ULL << 4U) : 0ULL;
        rebaseStateBits |= resetRebaseAck.blocked ? (1ULL << 5U) : 0ULL;
        rebaseStateBits |= resetRebaseAck.superseded ? (1ULL << 6U) : 0ULL;
        rebaseStateBits |=
            resetRebaseAck.unsupportedFaultOrEstop ? (1ULL << 7U) : 0ULL;
        rebaseStateBits |=
            resetRebaseAck.compensationBlocked ? (1ULL << 8U) : 0ULL;
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            rebaseStateBits);

        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetReleaseGateSnapshot.sequence);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            static_cast<std::uint64_t>(resetReleaseGateSnapshot.phase));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            static_cast<std::uint64_t>(resetReleaseGateSnapshot.decision));
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetReleaseGateSnapshot.expectedResetRequestSequence);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetReleaseGateSnapshot.ackRequestSequence);

        std::uint64_t resetGateAckBits = 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.ackObserved ? (1ULL << 0U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.ackSequenceMatched ? (1ULL << 1U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.ackEpochMatched ? (1ULL << 2U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.ackOwnerMatched ? (1ULL << 3U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.ackAxisMaskMatched ? (1ULL << 4U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.ackPhaseAcknowledged ? (1ULL << 5U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.rebaseAckMatched ? (1ULL << 6U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.releaseApplied ? (1ULL << 7U) : 0ULL;
        resetGateAckBits |=
            resetReleaseGateSnapshot.blocked ? (1ULL << 8U) : 0ULL;
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            resetGateAckBits);
        j5EventToken = FoldDiagnosticEventToken(
            j5EventToken,
            j5FailureTotal);

        const bool transportBusy =
            axisCommandDepth != 0U ||
            commandIngressDepth != 0U ||
            commandReplayDepth != 0U ||
            feedbackDepth != 0U ||
            feedbackNoticeDepth != 0U;

        static std::uint32_t startupSamples = 0U;
        static std::uint32_t quietDiagnosticSamples = 0U;
        static std::uint64_t diagnosticSamples = 0ULL;
        static std::uint64_t fullDiagnosticReports = 0ULL;
        static bool previousSafetyPending = false;
        static bool previousTransportBusy = false;
        static int previousDiagnosticNCState = -1;
        static int previousDiagnosticMode = -1;
        const int diagnosticNCState = static_cast<int>(nc->GetState());
        const int diagnosticMode = static_cast<int>(nc->GetMode());
        const bool operatingStateChanged = startupSamples != 0U &&
            (safetyPending != previousSafetyPending ||
                transportBusy != previousTransportBusy ||
                diagnosticNCState != previousDiagnosticNCState ||
                diagnosticMode != previousDiagnosticMode);
        static MotionOwner previousOwner = MotionOwner::NONE;
        static MotionOwnerGeneration previousGeneration =
            MOTION_OWNER_GENERATION_INVALID;
        static std::uint64_t previousTransportErrorTotal = 0ULL;
        static std::uint64_t previousLifecycleChangeToken = 0ULL;
        static std::uint64_t previousCompletionBoundaryChangeToken = 0ULL;
        static std::uint64_t previousProgramEndChangeToken = 0ULL;
        static std::uint64_t previousGMTransactionChangeToken = 0ULL;
        static std::uint64_t previousPreDispatchBarrierChangeToken = 0ULL;
        static std::uint64_t previousPreparedQueueChangeToken = 0ULL;
        static std::uint64_t previousPreparedEquivalenceChangeToken = 0ULL;
        static std::uint64_t
            previousOrdinaryG00FeedHoldRollingCutoverPublication = 0ULL;
        static std::uint64_t previousSingleBlockShadowChangeToken = 0ULL;
        static std::uint64_t previousSingleBlockGateChangeToken = 0ULL;
        static std::uint64_t previousFeedHoldChangeToken = 0ULL;
        static std::uint64_t previousFeedHoldGateChangeToken = 0ULL;
        static std::uint64_t previousLifecycleInterruptionChangeToken = 0ULL;
        static std::uint64_t previousResetReleaseGateChangeToken = 0ULL;
        static std::uint64_t previousAlarmEmergencyStopChangeToken = 0ULL;
        static std::uint64_t previousJ5EventToken = 0ULL;
        static std::uint64_t previousP1HandoverChangeToken = 0ULL;
        static std::uint64_t previousCommandPathModeChangeToken = 0ULL;
        static std::uint64_t previousQueueTailTransactionChangeToken = 0ULL;

        const bool ownerChanged =
            startupSamples != 0U &&
            (ownerLease.owner != previousOwner ||
                ownerLease.generation != previousGeneration);

        const bool errorCounterChanged =
            startupSamples != 0U &&
            transportErrorTotal != previousTransportErrorTotal;

        const bool lifecycleChanged =
            startupSamples != 0U &&
            lifecycleChangeToken != previousLifecycleChangeToken;

        const bool completionBoundaryChanged =
            startupSamples != 0U &&
            completionBoundaryChangeToken !=
            previousCompletionBoundaryChangeToken;

        const bool programEndChanged =
            startupSamples != 0U &&
            programEndChangeToken != previousProgramEndChangeToken;

        const bool ordinaryG00FeedHoldRollingCutoverChanged =
            startupSamples != 0U &&
            ordinaryG00FeedHoldRollingCutoverSnapshot.publicationSequence !=
            previousOrdinaryG00FeedHoldRollingCutoverPublication;

        // K.7.8 is intentionally event-gated. Other lifecycle events may
        // request the shared block without changing K.7.8. Always publish
        // its own events and the final counters at P_END.
        const bool shouldPrintK78 =
            startupSamples == 0U ||
            ordinaryG00FeedHoldRollingCutoverChanged ||
            programEndChanged;

        const bool gmTransactionChanged =
            startupSamples != 0U &&
            gmTransactionChangeToken != previousGMTransactionChangeToken;

        const bool preDispatchBarrierChanged =
            startupSamples != 0U &&
            preDispatchBarrierChangeToken !=
            previousPreDispatchBarrierChangeToken;

        const bool preparedQueueChanged =
            startupSamples != 0U &&
            preparedQueueChangeToken != previousPreparedQueueChangeToken;

        const bool preparedEquivalenceChanged =
            startupSamples != 0U &&
            preparedEquivalenceChangeToken !=
            previousPreparedEquivalenceChangeToken;

        const bool singleBlockShadowChanged =
            startupSamples != 0U &&
            singleBlockShadowChangeToken !=
            previousSingleBlockShadowChangeToken;

        const bool singleBlockGateChanged =
            startupSamples != 0U &&
            singleBlockGateChangeToken !=
            previousSingleBlockGateChangeToken;

        const bool feedHoldChanged =
            startupSamples != 0U &&
            feedHoldChangeToken != previousFeedHoldChangeToken;

        const bool feedHoldGateChanged =
            startupSamples != 0U &&
            feedHoldGateChangeToken != previousFeedHoldGateChangeToken;

        const bool lifecycleInterruptionChanged =
            startupSamples != 0U &&
            lifecycleInterruptionChangeToken !=
            previousLifecycleInterruptionChangeToken;

        const bool resetReleaseGateChanged =
            startupSamples != 0U &&
            resetReleaseGateChangeToken !=
            previousResetReleaseGateChangeToken;

        const bool alarmEmergencyStopChanged =
            startupSamples != 0U &&
            alarmEmergencyStopChangeToken !=
            previousAlarmEmergencyStopChangeToken;

        const bool j5EventChanged =
            startupSamples != 0U &&
            j5EventToken != previousJ5EventToken;

        const bool p1HandoverChanged =
            startupSamples != 0U &&
            p1HandoverChangeToken != previousP1HandoverChangeToken;

        const bool commandPathModeChanged =
            startupSamples != 0U &&
            commandPathModeChangeToken !=
            previousCommandPathModeChangeToken;

        const bool queueTailTransactionChanged =
            startupSamples != 0U &&
            queueTailTransactionChangeToken !=
            previousQueueTailTransactionChangeToken;

        const bool shouldPrintJ5 =
            startupSamples == 0U ||
            j5EventChanged;

        const bool shouldPrintJ6 =
            startupSamples == 0U ||
            alarmEmergencyStopChanged;

        const bool shouldPrint =
            startupSamples == 0U ||
            lifecycleChanged ||
            completionBoundaryChanged ||
            programEndChanged ||
            ordinaryG00FeedHoldRollingCutoverChanged ||
            gmTransactionChanged ||
            preDispatchBarrierChanged ||
            preparedQueueChanged ||
            preparedEquivalenceChanged ||
            singleBlockShadowChanged ||
            singleBlockGateChanged ||
            feedHoldChanged ||
            feedHoldGateChanged ||
            lifecycleInterruptionChanged ||
            resetReleaseGateChanged ||
            alarmEmergencyStopChanged ||
            p1HandoverChanged ||
            commandPathModeChanged ||
            queueTailTransactionChanged ||
            shouldPrintJ5 ||
            ownerChanged ||
            errorCounterChanged ||
            operatingStateChanged;

        diagnosticSamples = AddDiagnosticCounterSaturating(diagnosticSamples, 1ULL);
        if (shouldPrint)
        {
            quietDiagnosticSamples = 0U;
            fullDiagnosticReports = AddDiagnosticCounterSaturating(fullDiagnosticReports, 1ULL);
        }
        else if (++quietDiagnosticSamples >= 10U)
        {
            quietDiagnosticSamples = 0U;
            RunHmiDiagnosticOutputFamily([&]()
                {
                    RtPrintf(
                        "[NC-DIAG] Policy:ARC_FIX1 Sample:%llu Full:%llu "
                        "NC:%d Mode:%d Owner:%u/%u Safety:%u Busy:%u "
                        "RTSample:%llu J5Fail:%llu PDOInvalid:%llu\n",
                        static_cast<unsigned long long>(diagnosticSamples),
                        static_cast<unsigned long long>(fullDiagnosticReports),
                        diagnosticNCState, diagnosticMode,
                        static_cast<unsigned int>(ownerLease.owner),
                        static_cast<unsigned int>(ownerLease.generation),
                        safetyPending ? 1U : 0U, transportBusy ? 1U : 0U,
                        static_cast<unsigned long long>(groupNCSettleSnapshot.sampleSequence),
                        static_cast<unsigned long long>(j5FailureTotal),
                        static_cast<unsigned long long>(pdoInvalidCorrelation.invalidCycleCount));
                });
        }

        if (shouldPrint)
        {
            RunHmiDiagnosticOutputFamily([&]()
                {
                    RtPrintf(
                        "[NC02J64-LAG] Existing:%02X Ready:%02X Aligned:%02X "
                        "Armed:%02X Pending:%02X Blocked:%02X Stable:%u/%u ",
                        static_cast<unsigned int>(
                            startupLagArming.existingAxisMask),
                        static_cast<unsigned int>(
                            startupLagArming.feedbackReadyAxisMask),
                        static_cast<unsigned int>(
                            startupLagArming.positionAlignedAxisMask),
                        static_cast<unsigned int>(
                            startupLagArming.lagArmedAxisMask),
                        static_cast<unsigned int>(
                            startupLagArming.pendingAxisMask),
                        static_cast<unsigned int>(
                            startupLagArming.prematureMotionBlockedAxisMask),
                        static_cast<unsigned int>(
                            startupLagArming.minimumStableSampleCount),
                        static_cast<unsigned int>(
                            startupLagArming.stableSamplesRequired));
                    RtPrintf(
                        "Align:%llu Arm:%llu Reset:%llu Early:%llu All:%u\n",
                        static_cast<unsigned long long>(
                            startupLagArming.alignmentEvents),
                        static_cast<unsigned long long>(
                            startupLagArming.armingTransitions),
                        static_cast<unsigned long long>(
                            startupLagArming.readinessResets),
                        static_cast<unsigned long long>(
                            startupLagArming.prematureMotionBlocks),
                        startupLagArming.allExistingAxesArmed ? 1U : 0U);

                    RtPrintf(
                        "[NC02K21-P1] MapStop:%llu Retire:%llu RetireFail:%llu "
                        "Orphan:%llu Invalid:%llu AlarmReq:%llu AlarmEpoch:%u "
                        "Pending:%u "
                        "Prev:%02X Next:%02X LastAx:%d Pass:%u\n",
                        static_cast<unsigned long long>(
                            p1HandoverSafety.mappingBoundaryStops),
                        static_cast<unsigned long long>(
                            p1HandoverSafety.droppedAxisRetirements),
                        static_cast<unsigned long long>(
                            p1HandoverSafety.droppedAxisRetirementFailures),
                        static_cast<unsigned long long>(
                            p1HandoverSafety.orphanAxisContainments),
                        static_cast<unsigned long long>(
                            p1HandoverSafety.invalidProducerRejects),
                        static_cast<unsigned long long>(
                            p1HandoverSafety.mappingIntegrityAlarmRequests),
                        static_cast<unsigned int>(
                            p1HandoverSafety.
                            lastMappingIntegrityAlarmExecutionEpoch),
                        p1HandoverSafety.mappingIntegrityAlarmPending ? 1U : 0U,
                        static_cast<unsigned int>(
                            p1HandoverSafety.lastPreviousAxisMask),
                        static_cast<unsigned int>(
                            p1HandoverSafety.lastNextAxisMask),
                        p1HandoverSafety.lastOrphanAxisIndex,
                        (p1HandoverSafety.droppedAxisRetirementFailures == 0ULL &&
                            p1HandoverSafety.orphanAxisContainments == 0ULL &&
                            p1HandoverSafety.invalidProducerRejects == 0ULL &&
                            !p1HandoverSafety.mappingIntegrityAlarmPending)
                        ? 1U
                        : 0U);

                    const bool lifecycleCommitBalanced =
                        lifecycleCommit.acquired ==
                        lifecycleCommit.released +
                        (lifecycleCommit.reservationActive ? 1ULL : 0ULL);
                    RtPrintf(
                        "[NC02K22-CAS] Try:%llu Acq:%llu Block:%llu Lost:%llu "
                        "Rel:%llu RelFail:%llu PubWait:%llu Active:%u Pending:%u "
                        "Epoch:%u Pass:%u\n",
                        static_cast<unsigned long long>(lifecycleCommit.attempts),
                        static_cast<unsigned long long>(lifecycleCommit.acquired),
                        static_cast<unsigned long long>(
                            lifecycleCommit.blockedByLifecycle),
                        static_cast<unsigned long long>(
                            lifecycleCommit.compareExchangeLost),
                        static_cast<unsigned long long>(lifecycleCommit.released),
                        static_cast<unsigned long long>(
                            lifecycleCommit.releaseFailures),
                        static_cast<unsigned long long>(
                            lifecycleCommit.publisherWaits),
                        lifecycleCommit.reservationActive ? 1U : 0U,
                        lifecycleCommit.executionEpochPending ? 1U : 0U,
                        static_cast<unsigned int>(
                            lifecycleCommit.currentExecutionEpoch),
                        (lifecycleCommitBalanced &&
                            lifecycleCommit.releaseFailures == 0ULL)
                        ? 1U
                        : 0U);

                    RtPrintf(
                        "[NC01F-RT] NC:%d Mode:%d Owner:%s(%u) Gen:%u "
                        "Safety:%u AxisQ:%llu CmdIn:%llu Replay:%llu "
                        "FbQ:%llu Notice:%llu\n",
                        static_cast<int>(nc->GetState()),
                        static_cast<int>(nc->GetMode()),
                        MotionOwnerToDiagnosticName(ownerLease.owner),
                        static_cast<unsigned int>(ownerLease.owner),
                        static_cast<unsigned int>(ownerLease.generation),
                        safetyPending ? 1U : 0U,
                        static_cast<unsigned long long>(axisCommandDepth),
                        static_cast<unsigned long long>(commandIngressDepth),
                        static_cast<unsigned long long>(commandReplayDepth),
                        static_cast<unsigned long long>(feedbackDepth),
                        static_cast<unsigned long long>(feedbackNoticeDepth));

                    RtPrintf(
                        "[NC02K6-PATH] Payload:%u Ready:%u Shadow:%u Apply:%u "
                        "Influence:%u Cutover:%u Coh:%u/%u Acct:%u\n",
                        commandPathModeTransport.commandLocalPayloadPresent ? 1U : 0U,
                        commandPathModeTransport.transportReady ? 1U : 0U,
                        commandPathModeTransport.shadowOnly ? 1U : 0U,
                        commandPathModeTransport.consumerAuthority ? 1U : 0U,
                        commandPathModeTransport.runtimeInfluence ? 1U : 0U,
                        commandPathModeTransport.cutoverAttempted ? 1U : 0U,
                        commandPathModeTransport.producerSnapshotCoherent ? 1U : 0U,
                        commandPathModeTransport.consumerSnapshotCoherent ? 1U : 0U,
                        commandPathModeTransport.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K6-CNT] Submit:%llu Accept:%llu Reject:%llu "
                        "PE:%llu PC:%llu PU:%llu PI:%llu Observe:%llu ",
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerAccepted +
                            commandPathModeTransport.producerRejected),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerAccepted),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerRejected),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerExactStop),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerContinuous),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerUnspecified),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerInvalid),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerCommitted));
                    RtPrintf(
                        "Ingress:%llu Replay:%llu CE:%llu CC:%llu CU:%llu CI:%llu "
                        "Match:%llu Mis:%llu Driver:%llu PF:%016llX CF:%016llX\n",
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerIngressCommitted),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerReplayCommitted),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerExactStop),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerContinuous),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerUnspecified),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerInvalid),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.legacyModeMatches),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.legacyModeMismatches),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.driverOverrideObservations),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.producerFingerprint),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.consumerFingerprint));

                    RtPrintf(
                        "[NC02K61-AUTH] Try:%llu Apply:%llu Exact:%llu Cont:%llu "
                        "Legacy:%llu Replay:%llu DriverBlock:%llu Invalid:%llu "
                        "Acct:%u\n",
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityAttempts),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityApplied),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityExactStop),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityContinuous),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityLegacyFallbacks),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityReplayBypasses),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityDriverBlocks),
                        static_cast<unsigned long long>(
                            commandPathModeTransport.authorityInvalidRejects),
                        commandPathModeTransport.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K62-TAIL] Try:%llu Accept:%llu Reject:%llu "
                        "Commit:%llu Preserve:%llu MCS:%llu Pulse:%llu "
                        "Ovr:%llu Exact:%llu Bound:%llu Invalid:%llu Mis:%llu ",
                        static_cast<unsigned long long>(
                            queueTailTransaction.attempts),
                        static_cast<unsigned long long>(
                            queueTailTransaction.commandAccepted),
                        static_cast<unsigned long long>(
                            queueTailTransaction.commandRejected),
                        static_cast<unsigned long long>(
                            queueTailTransaction.committed),
                        static_cast<unsigned long long>(
                            queueTailTransaction.rejectPreserved),
                        static_cast<unsigned long long>(
                            queueTailTransaction.commandedMCSCommitted),
                        static_cast<unsigned long long>(
                            queueTailTransaction.lastQueuedPulseCommitted),
                        static_cast<unsigned long long>(
                            queueTailTransaction.rapidOverrideCommitted),
                        static_cast<unsigned long long>(
                            queueTailTransaction.endpointExact),
                        static_cast<unsigned long long>(
                            queueTailTransaction.captureBound),
                        static_cast<unsigned long long>(
                            queueTailTransaction.invalidInputs),
                        static_cast<unsigned long long>(
                            queueTailTransaction.mismatches));
                    RtPrintf(
                        "Ready:%u Coh:%u Acct:%u\n",
                        queueTailTransaction.ready ? 1U : 0U,
                        queueTailTransaction.snapshotCoherent ? 1U : 0U,
                        queueTailTransaction.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC01F-CNT] AxisQFull:%llu AxisResOv:%llu "
                        "OwnerReject:%llu Stale:%llu CmdFull:%llu ReplayOv:%llu "
                        "FbOv:%llu NoticeOv:%llu FbProc:%llu FbGap:%llu ",
                        static_cast<unsigned long long>(axisQueueFull),
                        static_cast<unsigned long long>(axisResultOverflow),
                        static_cast<unsigned long long>(ownerConflictReject),
                        static_cast<unsigned long long>(staleDiscard),
                        static_cast<unsigned long long>(commandQueueFull),
                        static_cast<unsigned long long>(replayOverflow),
                        static_cast<unsigned long long>(feedbackOverflow),
                        static_cast<unsigned long long>(feedbackNoticeOverflow),
                        static_cast<unsigned long long>(feedbackProcessed),
                        static_cast<unsigned long long>(feedbackSequenceGap));
                    RtPrintf(
                        "FbAcc:%llu FbStart:%llu FbDone:%llu FbReject:%llu "
                        "FbCancel:%llu FbAbort:%llu FbFault:%llu\n",
                        static_cast<unsigned long long>(feedbackAccepted),
                        static_cast<unsigned long long>(feedbackStarted),
                        static_cast<unsigned long long>(feedbackCompleted),
                        static_cast<unsigned long long>(feedbackRejected),
                        static_cast<unsigned long long>(feedbackCancelled),
                        static_cast<unsigned long long>(feedbackAborted),
                        static_cast<unsigned long long>(feedbackFaulted));

                });
            RunHmiDiagnosticOutputFamily([&]()
                {
                    RtPrintf(
                        "[NC02D-BLK] D:%llu Scope:%u Cache:%llu Frame:%llu "
                        "PC:%d Line:%d State:%s Commit:%u Seg:%u ",
                        static_cast<unsigned long long>(
                            hasBlockLifecycle ? blockLifecycle.dispatchId : 0ULL),
                        static_cast<unsigned int>(
                            hasBlockLifecycle ? blockLifecycle.programTarget.scope :
                            NCProgramScope::NONE),
                        static_cast<unsigned long long>(
                            hasBlockLifecycle ? blockLifecycle.programTarget.cacheGeneration : 0ULL),
                        static_cast<unsigned long long>(
                            hasBlockLifecycle ? blockLifecycle.programTarget.frameId : 0ULL),
                        hasBlockLifecycle ? blockLifecycle.programTarget.sourcePC : -1,
                        hasBlockLifecycle ? blockLifecycle.sourceLineNumber : 0,
                        NCBlockLifecycleStateToDiagnosticName(
                            hasBlockLifecycle ? blockLifecycle.state :
                            NCBlockLifecycleState::NONE),
                        hasBlockLifecycle && blockLifecycle.programCommitted ? 1U : 0U,
                        hasBlockLifecycle ?
                        static_cast<unsigned int>(blockLifecycle.motionSegmentCount) : 0U);
                    RtPrintf(
                        "Acc:%u Start:%u Done:%u Term:%u Fail:%u Ov:%u\n",
                        hasBlockLifecycle ?
                        static_cast<unsigned int>(blockLifecycle.motionAcceptedCount) : 0U,
                        hasBlockLifecycle ?
                        static_cast<unsigned int>(blockLifecycle.motionStartedCount) : 0U,
                        hasBlockLifecycle ?
                        static_cast<unsigned int>(blockLifecycle.motionCompletedCount) : 0U,
                        hasBlockLifecycle ?
                        static_cast<unsigned int>(blockLifecycle.motionTerminalCount) : 0U,
                        hasBlockLifecycle ?
                        static_cast<unsigned int>(blockLifecycle.motionFailedCount) : 0U,
                        hasBlockLifecycle && blockLifecycle.motionCaptureOverflow ? 1U : 0U);

                    RtPrintf(
                        "[NC02D-CNT] Dispatch:%llu Commit:%llu ProgOnly:%llu "
                        "MotionBlk:%llu Seg:%llu PAcc:%llu PRej:%llu "
                        "FAcc:%llu FStart:%llu FDone:%llu FRej:%llu ",
                        static_cast<unsigned long long>(blockCounters.dispatched),
                        static_cast<unsigned long long>(blockCounters.programCommitted),
                        static_cast<unsigned long long>(blockCounters.programOnlyCompleted),
                        static_cast<unsigned long long>(blockCounters.motionBlocks),
                        static_cast<unsigned long long>(blockCounters.motionSegmentsBound),
                        static_cast<unsigned long long>(blockCounters.producerAccepted),
                        static_cast<unsigned long long>(blockCounters.producerRejected),
                        static_cast<unsigned long long>(blockCounters.feedbackAccepted),
                        static_cast<unsigned long long>(blockCounters.feedbackStarted),
                        static_cast<unsigned long long>(blockCounters.feedbackCompleted),
                        static_cast<unsigned long long>(blockCounters.feedbackRejected));
                    RtPrintf(
                        "FCancel:%llu FAbort:%llu FFault:%llu "
                        "BlkDone:%llu BlkFail:%llu NCFail:%llu "
                        "CaptureOv:%llu Orphan:%llu DupTerm:%llu Conflict:%llu "
                        "Active:%u\n",
                        static_cast<unsigned long long>(blockCounters.feedbackCancelled),
                        static_cast<unsigned long long>(blockCounters.feedbackAborted),
                        static_cast<unsigned long long>(blockCounters.feedbackFaulted),
                        static_cast<unsigned long long>(blockCounters.blockCompleted),
                        static_cast<unsigned long long>(blockCounters.blockFailed),
                        static_cast<unsigned long long>(blockCounters.ncDispatchFailed),
                        static_cast<unsigned long long>(blockCounters.motionCaptureOverflow),
                        static_cast<unsigned long long>(blockCounters.orphanFeedback),
                        static_cast<unsigned long long>(blockCounters.duplicateTerminalFeedback),
                        static_cast<unsigned long long>(blockCounters.terminalFeedbackConflict),
                        static_cast<unsigned int>(blockCounters.activeBlocks));

                    RtPrintf(
                        "[NC02E-BND] D:%llu Kind:%s Legacy:%s Ledger:%s "
                        "Compare:%s State:%s Seg:%u Done:%u Term:%u Fail:%u\n",
                        static_cast<unsigned long long>(completionBoundary.dispatchId),
                        NCBlockWaitKindToDiagnosticName(completionBoundary.waitKind),
                        completionBoundary.legacyReady ? "READY" : "WAIT",
                        NCBlockMotionBoundaryStateToDiagnosticName(
                            completionBoundary.ledgerBoundary),
                        NCBlockCompletionComparisonToDiagnosticName(
                            completionBoundary.comparison),
                        NCBlockLifecycleStateToDiagnosticName(
                            completionBoundary.lifecycleState),
                        static_cast<unsigned int>(completionBoundary.motionSegmentCount),
                        static_cast<unsigned int>(completionBoundary.motionCompletedCount),
                        static_cast<unsigned int>(completionBoundary.motionTerminalCount),
                        static_cast<unsigned int>(completionBoundary.motionFailedCount));

                    RtPrintf(
                        "[NC02E-CNT] Bind:%llu Motion:%llu Aux:%llu Flow:%llu "
                        "Obs:%llu Release:%llu Agree:%llu LedgerEarly:%llu "
                        "LegacyEarly:%llu LedgerFail:%llu ReleaseFail:%llu ",
                        static_cast<unsigned long long>(completionCounters.bindings),
                        static_cast<unsigned long long>(completionCounters.motionBindings),
                        static_cast<unsigned long long>(completionCounters.auxiliaryBindings),
                        static_cast<unsigned long long>(completionCounters.programFlowBindings),
                        static_cast<unsigned long long>(completionCounters.observations),
                        static_cast<unsigned long long>(completionCounters.releaseChecks),
                        static_cast<unsigned long long>(completionCounters.agreeRelease),
                        static_cast<unsigned long long>(completionCounters.ledgerReadyBeforeLegacy),
                        static_cast<unsigned long long>(completionCounters.legacyEarlyRelease),
                        static_cast<unsigned long long>(completionCounters.ledgerFailureObserved),
                        static_cast<unsigned long long>(completionCounters.releaseOnLedgerFailure));
                    RtPrintf(
                        "Overflow:%llu Missing:%llu NonMotion:%llu Supersede:%llu\n",
                        static_cast<unsigned long long>(completionCounters.trackingOverflow),
                        static_cast<unsigned long long>(completionCounters.missingLifecycle),
                        static_cast<unsigned long long>(completionCounters.nonMotionWait),
                        static_cast<unsigned long long>(completionCounters.supersededBindings));

                    RtPrintf(
                        "[NC02F-GATE] D:%llu Eligible:%u Applied:%u Legacy:%s "
                        "Ledger:%s Decision:%s Effective:%s FailClosed:%u\n",
                        static_cast<unsigned long long>(completionBoundary.dispatchId),
                        completionBoundary.guardEligible ? 1U : 0U,
                        completionBoundary.guardApplied ? 1U : 0U,
                        completionBoundary.legacyReady ? "READY" : "WAIT",
                        NCBlockMotionBoundaryStateToDiagnosticName(
                            completionBoundary.ledgerBoundary),
                        NCBlockCompletionGateDecisionToDiagnosticName(
                            completionBoundary.gateDecision),
                        completionBoundary.effectiveReady ? "READY" : "WAIT",
                        completionBoundary.failClosed ? 1U : 0U);

                    RtPrintf(
                        "[NC02F-CNT] Eval:%llu Eligible:%llu Bypass:%llu "
                        "Wait:%llu Release:%llu BlockEarly:%llu BlockFail:%llu "
                        "BlockOv:%llu BlockMissing:%llu BlockNotTracked:%llu "
                        "FailClosed:%llu\n",
                        static_cast<unsigned long long>(completionCounters.guardEvaluations),
                        static_cast<unsigned long long>(completionCounters.guardEligibleSamples),
                        static_cast<unsigned long long>(completionCounters.guardBypassSamples),
                        static_cast<unsigned long long>(completionCounters.guardWaitSamples),
                        static_cast<unsigned long long>(completionCounters.dualKeyRelease),
                        static_cast<unsigned long long>(completionCounters.blockedLegacyEarly),
                        static_cast<unsigned long long>(completionCounters.blockedLedgerFailure),
                        static_cast<unsigned long long>(completionCounters.blockedTrackingOverflow),
                        static_cast<unsigned long long>(completionCounters.blockedMissingLifecycle),
                        static_cast<unsigned long long>(completionCounters.blockedNotTracked),
                        static_cast<unsigned long long>(completionCounters.failClosedBindings));
                    RtPrintf(
                        "[NC02G-END] Run:%llu Req:%llu Cause:%s Phase:%s Decision:%s "
                        "Scope:%u Cache:%llu RunEpoch:%u ReqEpoch:%u ",
                        static_cast<unsigned long long>(programEndSnapshot.run.runId),
                        static_cast<unsigned long long>(programEndSnapshot.requestId),
                        NCProgramEndCauseToDiagnosticName(programEndSnapshot.cause),
                        NCProgramEndPhaseToDiagnosticName(programEndSnapshot.phase),
                        NCProgramEndDecisionToDiagnosticName(programEndSnapshot.decision),
                        static_cast<unsigned int>(programEndSnapshot.run.scope),
                        static_cast<unsigned long long>(programEndSnapshot.run.cacheGeneration),
                        static_cast<unsigned int>(programEndSnapshot.run.executionEpoch),
                        static_cast<unsigned int>(programEndSnapshot.requestExecutionEpoch));
                    RtPrintf(
                        "RunOwner:%u/%u ReqOwner:%u/%u CurOwner:%u/%u "
                        "PC:%d Line:%d D:%llu Pending:%u Ready:%u Fail:%u\n",
                        static_cast<unsigned int>(programEndSnapshot.run.owner),
                        static_cast<unsigned int>(programEndSnapshot.run.ownerGeneration),
                        static_cast<unsigned int>(programEndSnapshot.requestOwner),
                        static_cast<unsigned int>(programEndSnapshot.requestOwnerGeneration),
                        static_cast<unsigned int>(programEndSnapshot.currentOwner),
                        static_cast<unsigned int>(programEndSnapshot.currentOwnerGeneration),
                        programEndSnapshot.sourcePC,
                        programEndSnapshot.sourceLineNumber,
                        static_cast<unsigned long long>(programEndSnapshot.markerDispatchId),
                        programEndSnapshot.requestPending ? 1U : 0U,
                        programEndSnapshot.readyToFinalize ? 1U : 0U,
                        programEndSnapshot.failClosed ? 1U : 0U);

                    RtPrintf(
                        "[NC02G-DRN] Active:%u AxisQ:%llu AxisR:%llu CmdQ:%llu "
                        "CmdIn:%llu Replay:%llu FbQ:%llu Notice:%llu ",
                        static_cast<unsigned int>(programEndSnapshot.activeBlocks),
                        static_cast<unsigned long long>(programEndSnapshot.axisCommandDepth),
                        static_cast<unsigned long long>(programEndSnapshot.axisResultDepth),
                        static_cast<unsigned long long>(programEndSnapshot.commandQueueDepth),
                        static_cast<unsigned long long>(programEndSnapshot.commandIngressDepth),
                        static_cast<unsigned long long>(programEndSnapshot.commandReplayDepth),
                        static_cast<unsigned long long>(programEndSnapshot.feedbackDepth),
                        static_cast<unsigned long long>(programEndSnapshot.feedbackNoticeDepth));
                    RtPrintf(
                        "FbPub:%llu FbCon:%llu FbSync:%u Stable:%u/%u "
                        "Stand:%u Safety:%u Wait:%u Bind:%u OwnerOK:%u Integrity:%llu\n",
                        static_cast<unsigned long long>(programEndSnapshot.lastPublishedFeedbackSequence),
                        static_cast<unsigned long long>(programEndSnapshot.lastConsumedFeedbackSequence),
                        programEndSnapshot.feedbackSequenceSynchronized ? 1U : 0U,
                        static_cast<unsigned int>(programEndSnapshot.stablePasses),
                        static_cast<unsigned int>(NC_PROGRAM_END_STABLE_PASSES_REQUIRED),
                        programEndSnapshot.groupStandstill ? 1U : 0U,
                        programEndSnapshot.safetyOrRecoveryPending ? 1U : 0U,
                        programEndSnapshot.waitCallbackActive ? 1U : 0U,
                        programEndSnapshot.completionBindingActive ? 1U : 0U,
                        programEndSnapshot.ownerLeaseCurrent ? 1U : 0U,
                        static_cast<unsigned long long>(programEndSnapshot.integrityDelta));

                    RtPrintf(
                        "[NC02G-CNT] StartTry:%llu Started:%llu StartBlock:%llu "
                        "Req:%llu EOF:%llu M02:%llu M30:%llu ReqReject:%llu ",
                        static_cast<unsigned long long>(programEndCounters.runStartAttempts),
                        static_cast<unsigned long long>(programEndCounters.runsStarted),
                        static_cast<unsigned long long>(programEndCounters.runStartBlocked),
                        static_cast<unsigned long long>(programEndCounters.requests),
                        static_cast<unsigned long long>(programEndCounters.naturalEofRequests),
                        static_cast<unsigned long long>(programEndCounters.m02Requests),
                        static_cast<unsigned long long>(programEndCounters.m30Requests),
                        static_cast<unsigned long long>(programEndCounters.rejectedRequests));
                    RtPrintf(
                        "Eval:%llu Ready:%llu Final:%llu Cancel:%llu Fail:%llu "
                        "EpochFail:%llu OwnerFail:%llu IntegrityFail:%llu\n",
                        static_cast<unsigned long long>(programEndCounters.evaluations),
                        static_cast<unsigned long long>(programEndCounters.readyToFinalize),
                        static_cast<unsigned long long>(programEndCounters.finalized),
                        static_cast<unsigned long long>(programEndCounters.cancelled),
                        static_cast<unsigned long long>(programEndCounters.failClosed),
                        static_cast<unsigned long long>(programEndCounters.epochMismatch),
                        static_cast<unsigned long long>(programEndCounters.ownerLeaseLost),
                        static_cast<unsigned long long>(programEndCounters.integrityFailure));

                    RtPrintf(
                        "[NC02G-WAIT] Blocks:%llu AxisQ:%llu AxisR:%llu CmdQ:%llu "
                        "Ingress:%llu Replay:%llu Notice:%llu Feedback:%llu FbSeq:%llu ",
                        static_cast<unsigned long long>(programEndCounters.waitActiveBlocks),
                        static_cast<unsigned long long>(programEndCounters.waitAxisCommand),
                        static_cast<unsigned long long>(programEndCounters.waitAxisResult),
                        static_cast<unsigned long long>(programEndCounters.waitCommandQueue),
                        static_cast<unsigned long long>(programEndCounters.waitCommandIngress),
                        static_cast<unsigned long long>(programEndCounters.waitCommandReplay),
                        static_cast<unsigned long long>(programEndCounters.waitFeedbackNotice),
                        static_cast<unsigned long long>(programEndCounters.waitFeedback),
                        static_cast<unsigned long long>(programEndCounters.waitFeedbackSequence));
                    RtPrintf(
                        "Callback:%llu Binding:%llu Safety:%llu Standstill:%llu Stable:%llu\n",
                        static_cast<unsigned long long>(programEndCounters.waitCallback),
                        static_cast<unsigned long long>(programEndCounters.waitCompletionBinding),
                        static_cast<unsigned long long>(programEndCounters.waitSafetyRequest),
                        static_cast<unsigned long long>(programEndCounters.waitGroupStandstill),
                        static_cast<unsigned long long>(programEndCounters.waitStableConfirmation));


                    RtPrintf(
                        "[NC02H-TXN] Seq:%llu D:%llu Phase:%s Post:%s Active:%u "
                        "GReq:%u GDone:%u MReq:%u MDone:%u PC:%d Line:%d ",
                        static_cast<unsigned long long>(gmTransactionSnapshot.sequence),
                        static_cast<unsigned long long>(gmTransactionSnapshot.dispatchId),
                        NCGMBlockTransactionPhaseToDiagnosticName(
                            gmTransactionSnapshot.phase),
                        NCGMBlockPostActionToDiagnosticName(
                            gmTransactionSnapshot.postAction),
                        gmTransactionSnapshot.active ? 1U : 0U,
                        gmTransactionSnapshot.gWaitRequired ? 1U : 0U,
                        gmTransactionSnapshot.gWaitComplete ? 1U : 0U,
                        gmTransactionSnapshot.mWaitRequired ? 1U : 0U,
                        gmTransactionSnapshot.mWaitComplete ? 1U : 0U,
                        gmTransactionSnapshot.sourcePC,
                        gmTransactionSnapshot.sourceLineNumber);
                    RtPrintf(
                        "M:%d P:%d L:%d Main:%u Applied:%u\n",
                        gmTransactionSnapshot.mCode,
                        gmTransactionSnapshot.pValue,
                        gmTransactionSnapshot.repeatCount,
                        gmTransactionSnapshot.fromMainProgram ? 1U : 0U,
                        gmTransactionSnapshot.postActionApplied ? 1U : 0U);

                    RtPrintf(
                        "[NC02H-CNT] Start:%llu GComp:%llu MComp:%llu Dual:%llu "
                        "Eval:%llu GWait:%llu MWait:%llu Ready:%llu Final:%llu ",
                        static_cast<unsigned long long>(gmTransactionCounters.started),
                        static_cast<unsigned long long>(gmTransactionCounters.gWaitComponents),
                        static_cast<unsigned long long>(gmTransactionCounters.mWaitComponents),
                        static_cast<unsigned long long>(gmTransactionCounters.dualComponentTransactions),
                        static_cast<unsigned long long>(gmTransactionCounters.evaluations),
                        static_cast<unsigned long long>(gmTransactionCounters.gWaitSamples),
                        static_cast<unsigned long long>(gmTransactionCounters.mWaitSamples),
                        static_cast<unsigned long long>(gmTransactionCounters.readyTransitions),
                        static_cast<unsigned long long>(gmTransactionCounters.finalized));
                    RtPrintf(
                        "Cancel:%llu Fail:%llu M00:%llu M01:%llu M98:%llu "
                        "M99:%llu M02:%llu M30:%llu\n",
                        static_cast<unsigned long long>(gmTransactionCounters.cancelled),
                        static_cast<unsigned long long>(gmTransactionCounters.finalizeFailed),
                        static_cast<unsigned long long>(gmTransactionCounters.m00Stops),
                        static_cast<unsigned long long>(gmTransactionCounters.m01Stops),
                        static_cast<unsigned long long>(gmTransactionCounters.m98Calls),
                        static_cast<unsigned long long>(gmTransactionCounters.m99Returns),
                        static_cast<unsigned long long>(gmTransactionCounters.m02Ends),
                        static_cast<unsigned long long>(gmTransactionCounters.m30Ends));

                    RtPrintf(
                        "[NC02J4-BAR] Seq:%llu Active:%u Kind:%s PC:%d Line:%d "
                        "M:%d Samples:%llu Ms:%llu CmdQ:%llu QWait:%u Stand:%u\n",
                        static_cast<unsigned long long>(
                            preDispatchBarrierSnapshot.sequence),
                        preDispatchBarrierSnapshot.active ? 1U : 0U,
                        NCPreDispatchBarrierKindToDiagnosticName(
                            preDispatchBarrierSnapshot.kind),
                        preDispatchBarrierSnapshot.sourcePC,
                        preDispatchBarrierSnapshot.sourceLineNumber,
                        preDispatchBarrierSnapshot.mCode,
                        static_cast<unsigned long long>(
                            preDispatchBarrierSnapshot.waitSamples),
                        static_cast<unsigned long long>(
                            preDispatchBarrierSnapshot.waitSamples * 10ULL),
                        static_cast<unsigned long long>(
                            preDispatchBarrierSnapshot.commandQueueDepth),
                        preDispatchBarrierSnapshot.commandQueuePending ? 1U : 0U,
                        preDispatchBarrierSnapshot.groupStandstill ? 1U : 0U);

                    RtPrintf(
                        "[NC02J4-BCNT] Arm:%llu Eval:%llu CmdQ:%llu Stand:%llu Clear:%llu\n",
                        static_cast<unsigned long long>(
                            preDispatchBarrierCounters.activations),
                        static_cast<unsigned long long>(
                            preDispatchBarrierCounters.evaluations),
                        static_cast<unsigned long long>(
                            preDispatchBarrierCounters.waitCommandQueue),
                        static_cast<unsigned long long>(
                            preDispatchBarrierCounters.waitGroupStandstill),
                        static_cast<unsigned long long>(
                            preDispatchBarrierCounters.cleared));

                });
            RunHmiDiagnosticOutputFamily([&]()
                {
                    RtPrintf(
                        "[NC02K1-PREP] Pub:%llu Sess:%llu Active:%u Depth:%u/%u "
                        "RuntimePC:%d PlanPC:%d TailPC:%d DispatchPC:%d CommitPC:%d BasePC:%d BaseBlock:%u ",
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.publicationSequence),
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.session),
                        preparedQueueSnapshot.active ? 1U : 0U,
                        static_cast<unsigned int>(preparedQueueSnapshot.depth),
                        static_cast<unsigned int>(preparedQueueSnapshot.capacity),
                        preparedQueueSnapshot.runtimeCurrentPC,
                        preparedQueueSnapshot.nextPlanPC,
                        preparedQueueSnapshot.tailPC,
                        preparedQueueSnapshot.dispatchPC,
                        preparedQueueSnapshot.commitPC,
                        preparedQueueSnapshot.committedBaselinePC,
                        preparedQueueSnapshot.committedBaselinePlanningBlocked
                        ? 1U : 0U);
                    RtPrintf(
                        "Stop:%s Barrier:%s BPC:%d Valid:%u Order:%u Account:%u "
                        "Shadow:%u\n",
                        NCPreparedQueueStopReasonToDiagnosticName(
                            preparedQueueSnapshot.stopReason),
                        NCPreparedBarrierKindToDiagnosticName(
                            preparedQueueSnapshot.barrierKind),
                        preparedQueueSnapshot.barrierPC,
                        preparedQueueSnapshot.valid ? 1U : 0U,
                        preparedQueueSnapshot.cursorOrderValid ? 1U : 0U,
                        preparedQueueSnapshot.accountingValid ? 1U : 0U,
                        preparedQueueSnapshot.shadowOnly ? 1U : 0U);

                    RtPrintf(
                        "[NC02K1-EDGE] Head:%u PC:%d Line:%d Class:%s Disp:%u Commit:%u "
                        "Tail:%u PC:%d Line:%d Class:%s Drain:%u Stop:%u\n",
                        hasPreparedQueueHead ? 1U : 0U,
                        hasPreparedQueueHead ? preparedQueueHead.sourcePC : -1,
                        hasPreparedQueueHead ? preparedQueueHead.sourceLineNumber : 0,
                        NCPreparedBlockClassToDiagnosticName(
                            preparedQueueHead.classification.blockClass),
                        hasPreparedQueueHead && preparedQueueHead.dispatchObserved
                        ? 1U : 0U,
                        hasPreparedQueueHead && preparedQueueHead.commitObserved
                        ? 1U : 0U,
                        hasPreparedQueueTail ? 1U : 0U,
                        hasPreparedQueueTail ? preparedQueueTail.sourcePC : -1,
                        hasPreparedQueueTail ? preparedQueueTail.sourceLineNumber : 0,
                        NCPreparedBlockClassToDiagnosticName(
                            preparedQueueTail.classification.blockClass),
                        hasPreparedQueueTail &&
                        preparedQueueTail.classification.legacyDrainRequired
                        ? 1U : 0U,
                        hasPreparedQueueTail &&
                        preparedQueueTail.classification.planningStopsHere
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K1-MOD] Seed:G%d G%d G%d WCS:G%d G%d "
                        "Tail:G%d G%d G%d WCS:G%d G%d G68:%u G168:%u ",
                        preparedQueueSnapshot.seedModal.distanceMode,
                        preparedQueueSnapshot.seedModal.unitsMode,
                        preparedQueueSnapshot.seedModal.planeMode,
                        preparedQueueSnapshot.seedModal.workCoordinateCode,
                        preparedQueueSnapshot.seedModal.storedStrokeMode,
                        preparedQueueSnapshot.tailModal.distanceMode,
                        preparedQueueSnapshot.tailModal.unitsMode,
                        preparedQueueSnapshot.tailModal.planeMode,
                        preparedQueueSnapshot.tailModal.workCoordinateCode,
                        preparedQueueSnapshot.tailModal.storedStrokeMode,
                        preparedQueueSnapshot.tailModal.g68Active ? 1U : 0U,
                        preparedQueueSnapshot.tailModal.g168Active ? 1U : 0U);
                    RtPrintf(
                        "Scale:%u Mirror:%02X Polar:%u COff:%u G66:%u "
                        "SeedV:%u TailV:%u MCS:%u G00F:%llu\n",
                        preparedQueueSnapshot.tailModal.scalingActive ? 1U : 0U,
                        static_cast<unsigned int>(
                            preparedQueueSnapshot.tailModal.mirrorMask),
                        preparedQueueSnapshot.tailModal.polarActive ? 1U : 0U,
                        preparedQueueSnapshot.tailModal.cAxisOffsetRotationEnabled
                        ? 1U : 0U,
                        preparedQueueSnapshot.tailModal.modalMacroActive ? 1U : 0U,
                        preparedQueueSnapshot.seedModal.imageValid ? 1U : 0U,
                        preparedQueueSnapshot.tailModal.imageValid ? 1U : 0U,
                        preparedQueueSnapshot.tailModal.commandedMCSValid ? 1U : 0U,
                        static_cast<unsigned long long>(
                            ScaleNonNegativeDiagnosticValue(
                                preparedQueueSnapshot.tailModal.g00OverrideRatio,
                                100.0)));

                    RtPrintf(
                        "[NC02K1-CUR] Scope:%u Cache:%llu Frame:%llu Epoch:%llu Flow:%llu ",
                        static_cast<unsigned int>(preparedQueueSnapshot.source.scope),
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.source.cacheGeneration),
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.source.frameId),
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.source.executionEpoch),
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.source.programFlowGeneration));
                    RtPrintf(
                        "Owner:%u/%llu LastInv:%s Panel:%u/%u/%u EOF:%u Cap:%u\n",
                        static_cast<unsigned int>(preparedQueueSnapshot.source.owner),
                        static_cast<unsigned long long>(
                            preparedQueueSnapshot.source.ownerGeneration),
                        NCPreparedInvalidationReasonToDiagnosticName(
                            preparedQueueSnapshot.lastInvalidationReason),
                        preparedQueueSnapshot.source.panel.blockSkipEnabled ? 1U : 0U,
                        preparedQueueSnapshot.source.panel.singleBlockEnabled ? 1U : 0U,
                        preparedQueueSnapshot.source.panel.optionalStopEnabled ? 1U : 0U,
                        preparedQueueSnapshot.eofLatched ? 1U : 0U,
                        preparedQueueSnapshot.capacityLatched ? 1U : 0U);

                    RtPrintf(
                        "[NC02K1-CNT] Eval:%llu Pub:%llu Sess:%llu Prep:%llu "
                        "Disp:%llu Commit:%llu Retire:%llu InvEntry:%llu Depth:%u ",
                        static_cast<unsigned long long>(preparedQueueCounters.evaluations),
                        static_cast<unsigned long long>(preparedQueueCounters.publications),
                        static_cast<unsigned long long>(preparedQueueCounters.sessions),
                        static_cast<unsigned long long>(preparedQueueCounters.prepared),
                        static_cast<unsigned long long>(preparedQueueCounters.dispatchMatched),
                        static_cast<unsigned long long>(preparedQueueCounters.commitMatched),
                        static_cast<unsigned long long>(preparedQueueCounters.retired),
                        static_cast<unsigned long long>(preparedQueueCounters.invalidatedEntries),
                        static_cast<unsigned int>(preparedQueueSnapshot.depth));
                    RtPrintf(
                        "Acct:%u Inv:%llu Alarm:%llu Reset:%llu Source:%llu "
                        "Epoch:%llu EpochOK:%llu Owner:%llu Frame:%llu Panel:%llu Barrier:%llu ",
                        preparedQueueSnapshot.accountingValid ? 1U : 0U,
                        static_cast<unsigned long long>(preparedQueueCounters.invalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.alarmInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.resetInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.sourceInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.epochInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.correlatedEpochAdvances),
                        static_cast<unsigned long long>(preparedQueueCounters.ownerInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.frameInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.panelInvalidations),
                        static_cast<unsigned long long>(preparedQueueCounters.barrierStops));
                    RtPrintf(
                        "EOF:%llu Cap:%llu Over:%llu Cursor:%llu PlanGap:%llu FlowOK:%llu "
                        "DMis:%llu CMis:%llu Stale:%llu IdFail:%llu Cutover:%llu\n",
                        static_cast<unsigned long long>(preparedQueueCounters.eofStops),
                        static_cast<unsigned long long>(preparedQueueCounters.capacityStops),
                        static_cast<unsigned long long>(preparedQueueCounters.overwritePrevented),
                        static_cast<unsigned long long>(preparedQueueCounters.cursorRegressions),
                        static_cast<unsigned long long>(preparedQueueCounters.planDiscontinuities),
                        static_cast<unsigned long long>(preparedQueueCounters.expectedFlowCutovers),
                        static_cast<unsigned long long>(preparedQueueCounters.dispatchMismatches),
                        static_cast<unsigned long long>(preparedQueueCounters.commitMismatches),
                        static_cast<unsigned long long>(preparedQueueCounters.staleRuntimeProofs),
                        static_cast<unsigned long long>(preparedQueueCounters.identityFailures),
                        static_cast<unsigned long long>(preparedQueueCounters.cutoverAttempts));

                    RtPrintf(
                        "[NC02K2-EQV] Pub:%llu State:%s Inv:%s Sess:%llu Entry:%llu "
                        "Scope:%u Cache:%llu Frame:%llu Epoch:%llu PostEpoch:%llu Flow:%llu ",
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.publicationSequence),
                        NCPreparedHeadEquivalenceStateToDiagnosticName(
                            preparedEquivalenceSnapshot.state),
                        NCPreparedHeadEquivalenceInvalidationToDiagnosticName(
                            preparedEquivalenceSnapshot.lastInvalidation),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.session),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.entrySequence),
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.scope),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.cacheGeneration),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.frameId),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.executionEpoch),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.commitExecutionEpoch),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.programFlowGeneration));
                    RtPrintf(
                        "Owner:%u/%llu Panel:%02X PC:%d Line:%d Class:%s Barrier:%s "
                        "Depth:%u SPeak:%u LPeak:%u ",
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.owner),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.ownerGeneration),
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.panelMask),
                        preparedEquivalenceSnapshot.sourcePC,
                        preparedEquivalenceSnapshot.sourceLineNumber,
                        NCPreparedBlockClassToDiagnosticName(
                            preparedEquivalenceSnapshot.blockClass),
                        NCPreparedBarrierKindToDiagnosticName(
                            preparedEquivalenceSnapshot.barrierKind),
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.preparedDepth),
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.sessionPeakDepth),
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.lifetimePeakDepth));
                    RtPrintf(
                        "QualSess:%llu Cand:%u Pend:%u Res:%u Disp:%llu Commit:%llu "
                        "Flags:%08X Block:%u Plan:%u ClassOK:%u Drain:%u ModB:%u ",
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.qualifiedSession),
                        preparedEquivalenceSnapshot.candidate ? 1U : 0U,
                        preparedEquivalenceSnapshot.pending ? 1U : 0U,
                        preparedEquivalenceSnapshot.resolved ? 1U : 0U,
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.dispatchId),
                        static_cast<unsigned long long>(
                            preparedEquivalenceSnapshot.commitSequence),
                        static_cast<unsigned int>(
                            preparedEquivalenceSnapshot.mismatchFlags),
                        preparedEquivalenceSnapshot.blockMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.planMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.classificationMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.drainMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.modalBeforeMatch ? 1U : 0U);
                    RtPrintf(
                        "ModA:%u LedD:%u LedC:%u Wait:%u UpDC:%u Ret:%u Up:%u Life:%u "
                        "Match:%u Acct:%u ",
                        preparedEquivalenceSnapshot.modalAfterMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.ledgerDispatchMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.ledgerCommitMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.runtimeWaitCallbackActive
                        ? 1U : 0U,
                        preparedEquivalenceSnapshot.upstreamDispatchCommitMatch
                        ? 1U : 0U,
                        preparedEquivalenceSnapshot.retirementMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.upstreamProofMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.lifecycleMatch ? 1U : 0U,
                        preparedEquivalenceSnapshot.matched ? 1U : 0U,
                        preparedEquivalenceSnapshot.accountingValid ? 1U : 0U);
                    RtPrintf(
                        "Qualified:%u Shadow:%u Influence:%u Cutover:%u\n",
                        preparedEquivalenceSnapshot.readinessQualified ? 1U : 0U,
                        preparedEquivalenceSnapshot.shadowOnly ? 1U : 0U,
                        preparedEquivalenceSnapshot.runtimeInfluence ? 1U : 0U,
                        preparedEquivalenceSnapshot.cutoverApplied ? 1U : 0U);

                    RtPrintf(
                        "[NC02K2-CNT] Obs:%llu Pub:%llu Cand:%llu Match:%llu Mis:%llu "
                        "Inv:%llu Inel:%llu Replay:%llu Bypass:%llu Peak:%llu Multi:%llu ",
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.observations),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.publications),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.candidates),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.matched),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.mismatched),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.invalidated),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.ineligible),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.replays),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.queueBypasses),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.lifetimePeakDepth),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.multiBlockObservations));
                    RtPrintf(
                        "QMis:%llu SrcMis:%llu PCMis:%llu LineMis:%llu BlockMis:%llu "
                        "PlanMis:%llu ClassMis:%llu DrainMis:%llu ModBMis:%llu ",
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.queueMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.sourceMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.pcMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.lineMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.blockMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.planMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.classificationMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.drainMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.modalBeforeMismatches));
                    RtPrintf(
                        "ModAMis:%llu LifeMis:%llu Stale:%llu ResolveMis:%llu "
                        "UpMis:%llu LedgerMis:%llu BindIdMis:%llu RuntimeFail:%llu "
                        "SessTrans:%llu QualSess:%llu WouldUse:%llu Use:%llu ",
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.modalAfterMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.lifecycleMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.staleTokens),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.resolveMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.upstreamMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.ledgerMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.bindIdentityMismatches),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.runtimeFailures),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.sessionTransitions),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.qualifiedSessions),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.wouldUse),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.useAttempts));
                    RtPrintf(
                        "Cutover:%llu Influence:%llu\n",
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.cutoverAttempts),
                        static_cast<unsigned long long>(
                            preparedEquivalenceCounters.runtimeInfluence));

                    RtPrintf(
                        "[NC02K3-GATE] Pub:%llu Decision:%s Revoke:%s Enabled:%u Attempt:%u "
                        "Apply:%u Legacy:%u Sess:%llu Entry:%llu QualSess:%llu ",
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.publicationSequence),
                        NCPreparedHeadCutoverDecisionToDiagnosticName(
                            preparedCutoverSnapshot.decision),
                        NCPreparedHeadCutoverRevocationToDiagnosticName(
                            preparedCutoverSnapshot.lastRevocation),
                        preparedCutoverSnapshot.enabled ? 1U : 0U,
                        preparedCutoverSnapshot.attempted ? 1U : 0U,
                        preparedCutoverSnapshot.applied ? 1U : 0U,
                        preparedCutoverSnapshot.legacyRetained ? 1U : 0U,
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.session),
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.entrySequence),
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.qualifiedSession));
                    RtPrintf(
                        "Qualified:%u Armed:%u Quarantine:%u Lock:%u Scope:%u Cache:%llu Frame:%llu ",
                        preparedCutoverSnapshot.sessionQualified ? 1U : 0U,
                        preparedCutoverSnapshot.qualificationArmed ? 1U : 0U,
                        preparedCutoverSnapshot.sessionQuarantined ? 1U : 0U,
                        preparedCutoverSnapshot.permanentLockout ? 1U : 0U,
                        static_cast<unsigned int>(preparedCutoverSnapshot.scope),
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.cacheGeneration),
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.frameId));
                    RtPrintf(
                        "Epoch:%llu Flow:%llu Owner:%u/%llu Panel:%02X PC:%d Line:%d Disp:%llu ",
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.executionEpoch),
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.programFlowGeneration),
                        static_cast<unsigned int>(preparedCutoverSnapshot.owner),
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.ownerGeneration),
                        static_cast<unsigned int>(preparedCutoverSnapshot.panelMask),
                        preparedCutoverSnapshot.sourcePC,
                        preparedCutoverSnapshot.sourceLineNumber,
                        static_cast<unsigned long long>(
                            preparedCutoverSnapshot.dispatchId));
                    RtPrintf(
                        "Class:%s Queue:%u Token:%u Source:%u PCLine:%u ClassOK:%u "
                        "Exact:%u Recheck:%u Prepared:%u Influence:%u Cutover:%u Acct:%u\n",
                        NCPreparedBlockClassToDiagnosticName(
                            preparedCutoverSnapshot.blockClass),
                        preparedCutoverSnapshot.queueExact ? 1U : 0U,
                        preparedCutoverSnapshot.tokenExact ? 1U : 0U,
                        preparedCutoverSnapshot.sourceExact ? 1U : 0U,
                        preparedCutoverSnapshot.pcLineExact ? 1U : 0U,
                        preparedCutoverSnapshot.classEligible ? 1U : 0U,
                        preparedCutoverSnapshot.equivalenceExact ? 1U : 0U,
                        preparedCutoverSnapshot.valueRevalidated ? 1U : 0U,
                        preparedCutoverSnapshot.preparedValueSelected ? 1U : 0U,
                        preparedCutoverSnapshot.runtimeInfluence ? 1U : 0U,
                        preparedCutoverSnapshot.cutoverApplied ? 1U : 0U,
                        preparedCutoverSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K3-CNT] Eval:%llu Pub:%llu Attempt:%llu Apply:%llu "
                        "Fallback:%llu Disabled:%llu NoHead:%llu Queue:%llu "
                        "Upstream:%llu Warmup:%llu Quarantine:%llu Duplicate:%llu ",
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.evaluations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.publications),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.attempts),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.applied),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.legacyFallbacks),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.disabled),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.noHead),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.queueRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.upstreamRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.unqualified),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.quarantined),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.duplicateRejected));
                    RtPrintf(
                        "Token:%llu Source:%llu PCLine:%llu Class:%llu Exact:%llu "
                        "RuntimeFail:%llu Arm:%llu Revoke:%llu DisRev:%llu QRev:%llu ",
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.tokenRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.sourceRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.pcLineRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.classRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.equivalenceRejected),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.runtimeFailures),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.arms),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.revocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.disabledRevocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.queueRevocations));
                    RtPrintf(
                        "AlarmRev:%llu ResetRev:%llu EndRev:%llu SrcRev:%llu UpRev:%llu "
                        "Use:%llu Cutover:%llu Influence:%llu\n",
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.alarmRevocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.resetRevocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.programEndRevocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.sourceRevocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.upstreamRevocations),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.applied),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.applied),
                        static_cast<unsigned long long>(
                            preparedCutoverCounters.applied));

                    RtPrintf(
                        "[NC02K4-ADM] Pub:%llu Decision:%s Revoke:%s Sess:%llu Entry:%llu "
                        "Scope:%u Cache:%llu Frame:%llu Epoch:%llu Flow:%llu Owner:%u/%llu ",
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.publicationSequence),
                        NCPreparedPreResolveAdmissionDecisionToDiagnosticName(
                            preparedPreResolveSnapshot.decision),
                        NCPreparedPreResolveAdmissionRevocationToDiagnosticName(
                            preparedPreResolveSnapshot.lastRevocation),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.session),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.entrySequence),
                        static_cast<unsigned int>(preparedPreResolveSnapshot.scope),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.cacheGeneration),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.frameId),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.executionEpoch),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.programFlowGeneration),
                        static_cast<unsigned int>(preparedPreResolveSnapshot.owner),
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.ownerGeneration));
                    RtPrintf(
                        "Panel:%02X PC:%d Line:%d Disp:%llu Class:%s Head:%u Queue:%u ",
                        static_cast<unsigned int>(preparedPreResolveSnapshot.panelMask),
                        preparedPreResolveSnapshot.sourcePC,
                        preparedPreResolveSnapshot.sourceLineNumber,
                        static_cast<unsigned long long>(
                            preparedPreResolveSnapshot.dispatchId),
                        NCPreparedBlockClassToDiagnosticName(
                            preparedPreResolveSnapshot.blockClass),
                        preparedPreResolveSnapshot.hasHead ? 1U : 0U,
                        preparedPreResolveSnapshot.queueExact ? 1U : 0U);
                    RtPrintf(
                        "Up:%u Qual:%u Token:%u Source:%u PCLine:%u Modal:%u ClassOK:%u ",
                        preparedPreResolveSnapshot.upstreamHealthy ? 1U : 0U,
                        preparedPreResolveSnapshot.sessionQualified ? 1U : 0U,
                        preparedPreResolveSnapshot.tokenExact ? 1U : 0U,
                        preparedPreResolveSnapshot.sourceExact ? 1U : 0U,
                        preparedPreResolveSnapshot.pcLineExact ? 1U : 0U,
                        preparedPreResolveSnapshot.modalExact ? 1U : 0U,
                        preparedPreResolveSnapshot.classEligible ? 1U : 0U);
                    RtPrintf(
                        "DrainReq:%u DrainOK:%u Cand:%u Legacy:%u Eq:%u K3:%u Confirm:%u "
                        "Lock:%u Shadow:%u Influence:%u Bypass:%u Acct:%u\n",
                        preparedPreResolveSnapshot.legacyDrainRequired ? 1U : 0U,
                        preparedPreResolveSnapshot.legacyDrainSatisfied ? 1U : 0U,
                        preparedPreResolveSnapshot.candidate ? 1U : 0U,
                        preparedPreResolveSnapshot.legacyResolveObserved ? 1U : 0U,
                        preparedPreResolveSnapshot.equivalenceObserved ? 1U : 0U,
                        preparedPreResolveSnapshot.cutoverObserved ? 1U : 0U,
                        preparedPreResolveSnapshot.confirmed ? 1U : 0U,
                        preparedPreResolveSnapshot.permanentLockout ? 1U : 0U,
                        preparedPreResolveSnapshot.shadowOnly ? 1U : 0U,
                        preparedPreResolveSnapshot.runtimeInfluence ? 1U : 0U,
                        preparedPreResolveSnapshot.resolverBypassed ? 1U : 0U,
                        preparedPreResolveSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K4-CNT] Eval:%llu Pub:%llu Cand:%llu Confirm:%llu "
                        "Reject:%llu Wait:%llu NoHead:%llu Queue:%llu Up:%llu ",
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.evaluations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.publications),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.candidates),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.confirmed),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.rejections),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.drainWaitSamples),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.noHead),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.queueRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.upstreamRejected));
                    RtPrintf(
                        "Session:%llu Token:%llu Source:%llu PCLine:%llu Modal:%llu "
                        "Class:%llu ResolveFail:%llu PostMis:%llu CandInv:%llu ConfirmFail:%llu ",
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.sessionRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.tokenRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.sourceRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.pcLineRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.modalRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.classRejected),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.legacyResolveFailures),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.postResolveMismatches),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.candidateInvalidations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.confirmedRuntimeFailures));
                    RtPrintf(
                        "Revoke:%llu QRev:%llu AlarmRev:%llu ResetRev:%llu EndRev:%llu "
                        "SrcRev:%llu UpRev:%llu Influence:%llu Bypass:%llu\n",
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.revocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.queueRevocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.alarmRevocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.resetRevocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.programEndRevocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.sourceRevocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.upstreamRevocations),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            preparedPreResolveCounters.resolverBypasses));

                    RtPrintf(
                        "[NC02K41-BYP] Pub:%llu Decision:%s Revoke:%s Lane:%s "
                        "Sess:%llu Entry:%llu PC:%d Line:%d Disp:%llu Commit:%llu ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.publicationSequence),
                        NCPreparedResolverBypassDecisionToDiagnosticName(
                            preparedResolverBypassSnapshot.decision),
                        NCPreparedResolverBypassRevocationToDiagnosticName(
                            preparedResolverBypassSnapshot.lastRevocation),
                        NCPreparedResolverBypassLaneToDiagnosticName(
                            preparedResolverBypassSnapshot.lane),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.session),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.entrySequence),
                        preparedResolverBypassSnapshot.sourcePC,
                        preparedResolverBypassSnapshot.sourceLineNumber,
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.dispatchId),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.commitSequence));
                    RtPrintf(
                        "QPure:%llu QP1:%llu Enable:%u Attempt:%u Select:%u "
                        "Bind:%u CommitOK:%u Proof:%u Queue:%u Up:%u LaneQ:%u ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.
                            pureModalQualifiedSession),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.g00P1QualifiedSession),
                        preparedResolverBypassSnapshot.enabled ? 1U : 0U,
                        preparedResolverBypassSnapshot.attempted ? 1U : 0U,
                        preparedResolverBypassSnapshot.selected ? 1U : 0U,
                        preparedResolverBypassSnapshot.dispatchBound ? 1U : 0U,
                        preparedResolverBypassSnapshot.commitBound ? 1U : 0U,
                        preparedResolverBypassSnapshot.upstreamProofVerified
                        ? 1U : 0U,
                        preparedResolverBypassSnapshot.queueExact ? 1U : 0U,
                        preparedResolverBypassSnapshot.upstreamHealthy ? 1U : 0U,
                        preparedResolverBypassSnapshot.laneQualified ? 1U : 0U);
                    RtPrintf(
                        "Token:%u Source:%u PCLine:%u Modal:%u Class:%u Rebuild:%u "
                        "Pending:%u Lock:%u Legacy:%u Influence:%u Bypass:%u Acct:%u\n",
                        preparedResolverBypassSnapshot.tokenExact ? 1U : 0U,
                        preparedResolverBypassSnapshot.sourceExact ? 1U : 0U,
                        preparedResolverBypassSnapshot.pcLineExact ? 1U : 0U,
                        preparedResolverBypassSnapshot.modalExact ? 1U : 0U,
                        preparedResolverBypassSnapshot.classEligible ? 1U : 0U,
                        preparedResolverBypassSnapshot.literalRebuiltExact
                        ? 1U : 0U,
                        preparedResolverBypassSnapshot.pending ? 1U : 0U,
                        preparedResolverBypassSnapshot.permanentLockout ? 1U : 0U,
                        preparedResolverBypassSnapshot.legacyResolverRetained
                        ? 1U : 0U,
                        preparedResolverBypassSnapshot.runtimeInfluence ? 1U : 0U,
                        preparedResolverBypassSnapshot.resolverBypassed ? 1U : 0U,
                        preparedResolverBypassSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K41-CNT] Eval:%llu Pub:%llu Select:%llu Fallback:%llu "
                        "Bind:%llu Commit:%llu Proof:%llu Wait:%llu Disable:%llu ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.evaluations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.publications),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.selected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.legacyFallbacks),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.dispatchBound),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.commitBound),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofVerified),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofWaitSamples),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.disabled));
                    RtPrintf(
                        "Busy:%llu NoHead:%llu Queue:%llu Up:%llu Session:%llu "
                        "Token:%llu Source:%llu PCLine:%llu Modal:%llu Class:%llu ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.busy),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.noHead),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.queueRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.upstreamRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.sessionRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.tokenRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.sourceRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.pcLineRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.modalRejected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.classRejected));
                    RtPrintf(
                        "QualCand:%llu QualModal:%llu QualP1:%llu QualFail:%llu "
                        "QualInv:%llu RuntimeFail:%llu ProofMis:%llu PendingInv:%llu "
                        "Revoke:%llu DisRev:%llu QRev:%llu AlarmRev:%llu ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.
                            qualificationCandidates),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.qualifiedPureModal),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.qualifiedG00P1),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.qualificationFailures),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.
                            qualificationInvalidations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.runtimeFailures),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofMismatches),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.
                            invalidatedPendingBypasses),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.revocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.disabledRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.queueRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.alarmRevocations));
                    RtPrintf(
                        "ResetRev:%llu EndRev:%llu SrcRev:%llu RunRev:%llu "
                        "ProofRev:%llu Prepared:%llu Influence:%llu Bypass:%llu\n",
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.resetRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.programEndRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.sourceRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.runtimeRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofRevocations),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.preparedSelections),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.resolverBypasses));

                    RtPrintf(
                        "[NC02K42-NOP] Pub:%llu Decision:%s Lane:%s Sess:%llu "
                        "Entry:%llu QNoP:%llu DrainReq:%u DrainOK:%u DrainWait:%u ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.publicationSequence),
                        NCPreparedResolverBypassDecisionToDiagnosticName(
                            preparedResolverBypassSnapshot.decision),
                        NCPreparedResolverBypassLaneToDiagnosticName(
                            preparedResolverBypassSnapshot.lane),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.session),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.entrySequence),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.g00NoPQualifiedSession),
                        preparedResolverBypassSnapshot.legacyDrainRequired ? 1U : 0U,
                        preparedResolverBypassSnapshot.legacyDrainSatisfied ? 1U : 0U,
                        preparedResolverBypassSnapshot.deferredForDrain ? 1U : 0U);
                    RtPrintf(
                        "CbReq:%u CbCommit:%u WaitSeen:%u CbDone:%u Epoch:%llu "
                        "CommitEpoch:%llu Disp:%llu Commit:%llu Pending:%u Lock:%u "
                        "Acct:%u\n",
                        preparedResolverBypassSnapshot.callbackRequired ? 1U : 0U,
                        preparedResolverBypassSnapshot.callbackActiveAtCommit
                        ? 1U : 0U,
                        preparedResolverBypassSnapshot.waitPhaseObserved ? 1U : 0U,
                        preparedResolverBypassSnapshot.callbackCompletionObserved
                        ? 1U : 0U,
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.executionEpoch),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.commitExecutionEpoch),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.dispatchId),
                        static_cast<unsigned long long>(
                            preparedResolverBypassSnapshot.commitSequence),
                        preparedResolverBypassSnapshot.pending ? 1U : 0U,
                        preparedResolverBypassSnapshot.permanentLockout ? 1U : 0U,
                        preparedResolverBypassSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K42-CNT] DrainWait:%llu CbWait:%llu CbDone:%llu "
                        "WaitPhase:%llu QualNoP:%llu SelPure:%llu SelP1:%llu "
                        "SelNoP:%llu ProofPure:%llu ProofP1:%llu ProofNoP:%llu ",
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.drainWaitSamples),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.callbackWaitSamples),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.callbackCompletions),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.ordinaryWaitPhases),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.qualifiedG00NoP),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.selectedPureModal),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.selectedG00P1),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.selectedG00NoP),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofVerifiedPureModal),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofVerifiedG00P1),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofVerifiedG00NoP));
                    RtPrintf(
                        "Select:%llu Proof:%llu\n",
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.selected),
                        static_cast<unsigned long long>(
                            preparedResolverBypassCounters.proofVerified));

                });
            RunHmiDiagnosticOutputFamily([&]()
                {
                    RtPrintf(
                        "[NC02K5-ADM] Pub:%llu Decision:%s Revoke:%s Sess:%llu "
                        "Entry:%llu Owner:%u/%llu Panel:%X PC:%d Line:%d ",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.publicationSequence),
                        NCOrdinaryG00AdmissionDecisionToDiagnosticName(
                            ordinaryG00AdmissionSnapshot.decision),
                        NCOrdinaryG00AdmissionRevocationToDiagnosticName(
                            ordinaryG00AdmissionSnapshot.lastRevocation),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.session),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.entrySequence),
                        static_cast<unsigned int>(
                            ordinaryG00AdmissionSnapshot.owner),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.ownerGeneration),
                        static_cast<unsigned int>(
                            ordinaryG00AdmissionSnapshot.panelMask),
                        ordinaryG00AdmissionSnapshot.sourcePC,
                        ordinaryG00AdmissionSnapshot.sourceLineNumber);
                    RtPrintf(
                        "Disp:%llu Commit:%llu Depth:%llu "
                        "Still:%u BusyS:%llu Block:0x%X Q:%u Env:%u Axis:%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.dispatchId),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.commitSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.initialQueueDepth),
                        ordinaryG00AdmissionSnapshot.initialGroupStandstill
                        ? 1U : 0U,
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.busySamples),
                        static_cast<unsigned int>(
                            ordinaryG00AdmissionSnapshot.blockerMask),
                        ordinaryG00AdmissionSnapshot.upstreamQualified ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.simpleG90Envelope ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.configuredAxisPresent
                        ? 1U : 0U);
                    RtPrintf(
                        "Project:%u Drained:%u Busy:%u Select:%u Bind:%u "
                        "CommitOK:%u Flight:%u FSeq:%llu FEpoch:%u FSeg:%llu ",
                        ordinaryG00AdmissionSnapshot.projected ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.initialDrained ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.initialBusy ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacySelected ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacyDispatchBound ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacyCommitBound ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.inflightRegistryProven
                        ? 1U : 0U,
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.inflightRegistrySequence),
                        static_cast<unsigned int>(
                            ordinaryG00AdmissionSnapshot.inflightExecutionEpoch),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.inflightSegmentId));
                    RtPrintf(
                        "Cb:%u CbDone:%u EpochAdv:%u Proof:%u "
                        "Done:%u Shadow:%u "
                        "Pending:%u Ready:%u Influence:%u Bypass:%u Lock:%u "
                        "Acct:%u\n",
                        ordinaryG00AdmissionSnapshot.legacyCallbackObserved
                        ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacyCallbackCompleted
                        ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacyEpochAdvanced ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacyUpstreamProofVerified
                        ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.legacyCompleted ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.shadowOnly ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.pending ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.cutoverReady ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.runtimeInfluence ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.resolverBypassed ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.permanentLockout ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K5-ENV] BaseFail:0x%013llX ModalFail:0x%04llX "
                        "BusyFail:0x%02X DrainedFail:0x%02X BDecision:%s "
                        "ResolverInput:%u Selected:%u BypassField:%u LaneQ:%u "
                        "Deferred:%u DrainOK:%u\n",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.
                            baseEvidenceFailureMask),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionSnapshot.
                            modalEnvelopeFailureMask),
                        static_cast<unsigned int>(
                            ordinaryG00AdmissionSnapshot.busyRouteFailureMask),
                        static_cast<unsigned int>(
                            ordinaryG00AdmissionSnapshot.drainedRouteFailureMask),
                        NCPreparedResolverBypassDecisionToDiagnosticName(
                            ordinaryG00AdmissionSnapshot.observedBypassDecision),
                        ordinaryG00AdmissionSnapshot.resolverBypassedInput ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.observedBypassSelected ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.
                        observedBypassResolverBypassed ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.
                        observedBypassLaneQualified ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.
                        observedBypassDeferredForDrain ? 1U : 0U,
                        ordinaryG00AdmissionSnapshot.
                        observedLegacyDrainSatisfied ? 1U : 0U);

                    RtPrintf(
                        "[NC02K5-CNT] Scan:%llu Pub:%llu Unique:%llu Warmup:%llu "
                        "Project:%llu Cand:%llu Drained:%llu BusyToken:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.scans),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.publications),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.uniqueEvaluations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.warmup),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.projected),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.candidates),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.initialDrained),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.initialBusy));
                    RtPrintf(
                        "BusyS:%llu Select:%llu Bind:%llu Commit:%llu Wait:%llu "
                        "FlightBind:%llu CbDone:%llu Done:%llu Reject:%llu "
                        "MissingPath:%llu MissingTail:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.busySamples),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.legacySelections),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.legacyDispatchBound),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.legacyCommitBound),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.completionWaitSamples),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.inflightRegistryBound),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.callbackCompleted),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.legacyCompleted),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.rejected),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.missingCommandPathMode),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.
                            missingTransactionalEndpoint));
                    RtPrintf(
                        "MissingFlight:%llu Mis:%llu RuntimeFail:%llu "
                        "PendingInv:%llu Revoke:%llu QRev:%llu AlarmRev:%llu "
                        "ResetRev:%llu EndRev:%llu SrcRev:%llu Cutover:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.missingInflightRegistry),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.mismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.runtimeFailures),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.invalidatedPending),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.revocations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.queueRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.alarmRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.resetRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.programEndRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.sourceRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.cutoverAttempts));
                    RtPrintf(
                        "Influence:%llu Bypass:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00AdmissionCounters.resolverBypasses));

                    const NCOrdinaryG00InflightEntrySnapshot& inflightLast =
                        ordinaryG00InflightRegistrySnapshot.lastEntry;
                    RtPrintf(
                        "[NC02K63-FLIGHT] Pub:%llu Seq:%llu Sess:%llu Entry:%llu "
                        "Disp:%llu Commit:%llu Epoch:%u Seg:%llu SrcBlk:%d ",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistrySnapshot.publicationSequence),
                        static_cast<unsigned long long>(
                            inflightLast.registrySequence),
                        static_cast<unsigned long long>(inflightLast.session),
                        static_cast<unsigned long long>(inflightLast.entrySequence),
                        static_cast<unsigned long long>(inflightLast.dispatchId),
                        static_cast<unsigned long long>(inflightLast.commitSequence),
                        static_cast<unsigned int>(inflightLast.identity.epoch),
                        static_cast<unsigned long long>(
                            inflightLast.identity.segmentId),
                        static_cast<int>(inflightLast.identity.sourceBlockId));
                    RtPrintf(
                        "Owner:%u/%u State:%s Revoke:%s Active:%u Term:%u "
                        "Revoked:%u Slots:%u ActiveN:%u RevPend:%u Peak:%u ",
                        static_cast<unsigned int>(inflightLast.ownerLease.owner),
                        static_cast<unsigned int>(inflightLast.ownerLease.generation),
                        NCOrdinaryG00InflightStateToDiagnosticName(
                            inflightLast.state),
                        NCOrdinaryG00InflightRevocationToDiagnosticName(
                            inflightLast.revocation),
                        inflightLast.active ? 1U : 0U,
                        inflightLast.terminal ? 1U : 0U,
                        inflightLast.revoked ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.occupiedEntries,
                        ordinaryG00InflightRegistrySnapshot.activeEntries,
                        ordinaryG00InflightRegistrySnapshot.revokedPendingEntries,
                        ordinaryG00InflightRegistrySnapshot.peakActiveEntries);
                    RtPrintf(
                        "Ready:%u Lock:%u Bounded:%u Shadow:%u Influence:%u "
                        "MotionWrite:%u Acct:%u\n",
                        ordinaryG00InflightRegistrySnapshot.ready ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.permanentLockout
                        ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.bounded ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.shadowOnly ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.runtimeInfluence
                        ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.motionWrite ? 1U : 0U,
                        ordinaryG00InflightRegistrySnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K63-FCNT] Try:%llu Reg:%llu Reject:%llu Invalid:%llu "
                        "DupId:%llu Overflow:%llu Reuse:%llu Obs:%llu Match:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.registrationAttempts),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.registered),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.registrationRejected),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.invalidRegistration),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.duplicateIdentity),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.capacityOverflow),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.slotReuses),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.feedbackObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.feedbackMatched));
                    RtPrintf(
                        "Ignore:%llu Accept:%llu Start:%llu Prog:%llu Held:%llu "
                        "Resume:%llu Term:%llu Done:%llu RejectT:%llu Cancel:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.feedbackIgnored),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.accepted),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.started),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.progress),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.held),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.resumed),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.terminal),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.completed),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.rejected),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.cancelled));
                    RtPrintf(
                        "Abort:%llu Fault:%llu SeqInv:%llu Gap:%llu ActiveGap:%llu "
                        "Orphan:%llu IdConflict:%llu OwnerMis:%llu DupTerm:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.aborted),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.faulted),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.invalidFeedbackSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.feedbackSequenceGaps),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.activeSequenceGapFailures),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.ledgerOrphans),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.identityConflicts),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.ownerMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.duplicateTerminal));
                    RtPrintf(
                        "TermConflict:%llu PostTerm:%llu Rev:%llu RevTerm:%llu "
                        "QRev:%llu AlarmRev:%llu ResetRev:%llu EndRev:%llu "
                        "SrcRev:%llu RunRev:%llu FbOv:%llu NoticeOv:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.terminalConflict),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.postTerminalFeedback),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.entriesRevoked),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.revokedTerminals),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.queueRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.alarmRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.resetRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.programEndRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.sourceRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.runtimeRevocations),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.feedbackOverflowObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.
                            producerNoticeOverflowObserved));
                    RtPrintf(
                        "LedBlkOv:%llu LedSegOv:%llu "
                        "LedDup:%llu LedConflict:%llu RunFail:%llu Fail:%llu "
                        "Active:%llu Peak:%llu Influence:%llu MotionWrite:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.
                            ledgerActiveBlockOverwriteObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.
                            ledgerActiveSegmentIndexOverwriteObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.ledgerDuplicateObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.ledgerConflictObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.runtimeFailures),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.permanentFailures),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.activeEntries),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.peakActiveEntries),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.motionWrites));

                    RtPrintf(
                        "[NC02K7-RA] Pub:%llu Decision:%s Sess:%llu WarmSess:%llu "
                        "Entry:%llu PC:%d Line:%d ChainPC:%d Disp:%llu Commit:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.publicationSequence),
                        NCOrdinaryG00ReadAheadDecisionToDiagnosticName(
                            ordinaryG00ReadAheadSnapshot.decision),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.session),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.warmupSession),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.entrySequence),
                        ordinaryG00ReadAheadSnapshot.sourcePC,
                        ordinaryG00ReadAheadSnapshot.sourceLineNumber,
                        ordinaryG00ReadAheadSnapshot.chainSourcePC,
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.dispatchId),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.commitSequence));
                    RtPrintf(
                        "Epoch:%u Seg:%llu ActiveAt:%u QDepth:%llu Limit:%u "
                        "Cand:%u Env:%u RegReady:%u RegHealth:%u RegFence:%u ",
                        static_cast<unsigned int>(
                            ordinaryG00ReadAheadSnapshot.identity.epoch),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.identity.segmentId),
                        ordinaryG00ReadAheadSnapshot.activeEntriesAtSelection,
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadSnapshot.queueDepthAtSelection),
                        ordinaryG00ReadAheadSnapshot.activeLimit,
                        ordinaryG00ReadAheadSnapshot.candidateExact ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.envelopeExact ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.registryReady ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.registryHealthy ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.registryHealthFenced
                        ? 1U : 0U);
                    RtPrintf(
                        "HealthLock:%u Warm:%u Chain:%u Cap:%u Sel:%u Bind:%u Auth:%u "
                        "CommitOK:%u Reg:%u Buffered:%u Exact:%u Stable:%u ",
                        ordinaryG00ReadAheadSnapshot.registryHealthLockout
                        ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.legacyWarmupProven ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.contiguousChain ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.capacityAvailable ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.selected ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.dispatchBound ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.motionAuthorized ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.commitBound ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.registryBound ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.bufferedTransport ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.exactStop ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.stableExecutionEpoch ? 1U : 0U);
                    RtPrintf(
                        "NoCb:%u Txn:%u Pending:%u Lock:%u Influence:%u "
                        "Bypass:%u MotionWrite:%u Acct:%u\n",
                        ordinaryG00ReadAheadSnapshot.noPerBlockCallback ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.transactionalEndpoint ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.pending ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.permanentLockout ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.runtimeInfluence ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.resolverBypassed ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.motionWrite ? 1U : 0U,
                        ordinaryG00ReadAheadSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K7-CNT] Eval:%llu Pub:%llu Ineligible:%llu "
                        "WarmWait:%llu RegFence:%llu HealthLock:%llu Discont:%llu "
                        "CapWait:%llu Select:%llu Bind:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.evaluations),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.publications),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.ineligible),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.legacyWarmupWaits),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.registryHealthFences),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.registryHealthLockouts),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.discontinuityFallbacks),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.capacityWaits),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.selected),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.dispatchBound));
                    RtPrintf(
                        "Auth:%llu Commit:%llu Reg:%llu Revoked:%llu "
                        "RuntimeFail:%llu Mis:%llu Warmup:%llu MaxActive:%llu "
                        "Influence:%llu Bypass:%llu MotionWrite:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.motionAuthorized),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.committed),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.registryBound),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.revokedPending),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.runtimeFailures),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.proofMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.sessionWarmups),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.maxActiveObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.resolverBypasses),
                        static_cast<unsigned long long>(
                            ordinaryG00ReadAheadCounters.motionWrites));
                    RtPrintf(
                        "RegTry:%llu RegOK:%llu RegReject:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.
                            readAheadRegistrationAttempts),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.
                            readAheadRegistered),
                        static_cast<unsigned long long>(
                            ordinaryG00InflightRegistryCounters.
                            readAheadRegistrationRejected));

                    RtPrintf(
                        "[NC02K73-FHC] Pub:%llu Coh:%llu BSeq:%llu GSeq:%llu "
                        "Phase:%s Decision:%s Sess:%llu Epoch:%u Owner:%u/%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortSnapshot.publicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortSnapshot.cohortSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortSnapshot.boundarySequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortSnapshot.gateSequence),
                        NCOrdinaryG00FeedHoldCohortPhaseToDiagnosticName(
                            ordinaryG00FeedHoldCohortSnapshot.phase),
                        NCOrdinaryG00FeedHoldCohortDecisionToDiagnosticName(
                            ordinaryG00FeedHoldCohortSnapshot.decision),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortSnapshot.session),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortSnapshot.executionEpoch),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortSnapshot.owner),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortSnapshot.ownerGeneration));
                    RtPrintf(
                        "ActiveAt:%u Members:%u Term:%u Ack:%u Req:%u Early:%u "
                        "Applied:%u Order:%u CohortDone:%u AllDone:%u Active:%u ",
                        ordinaryG00FeedHoldCohortSnapshot.activeEntriesAtCapture,
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortSnapshot.memberCount),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortSnapshot.terminalCount),
                        ordinaryG00FeedHoldCohortSnapshot.holdAcknowledged ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.resumeRequested ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.
                        resumeBeforeAcknowledge ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.resumeApplied ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.terminalOrderValid ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.
                        terminalCohortComplete ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.allMembersCompleted
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.active ? 1U : 0U);
                    RtPrintf(
                        "Cancel:%u Fail:%u Shadow:%u Influence:%u MotionWrite:%u "
                        "Acct:%u\n",
                        ordinaryG00FeedHoldCohortSnapshot.cancelled ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.failed ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.shadowOnly ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.runtimeInfluence ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.motionWrite ? 1U : 0U,
                        ordinaryG00FeedHoldCohortSnapshot.accountingValid ? 1U : 0U);

                    for (std::size_t memberIndex = 0U;
                        memberIndex < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
                        ++memberIndex)
                    {
                        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member =
                            ordinaryG00FeedHoldCohortSnapshot.members[memberIndex];
                        RtPrintf(
                            "[NC02K73-M%u] Reg:%llu Entry:%llu Disp:%llu "
                            "Commit:%llu PC:%d Line:%d Epoch:%u Seg:%llu ",
                            static_cast<unsigned int>(memberIndex),
                            static_cast<unsigned long long>(member.registrySequence),
                            static_cast<unsigned long long>(member.entrySequence),
                            static_cast<unsigned long long>(member.dispatchId),
                            static_cast<unsigned long long>(member.commitSequence),
                            member.sourcePC,
                            member.sourceLineNumber,
                            static_cast<unsigned int>(member.identity.epoch),
                            static_cast<unsigned long long>(member.identity.segmentId));
                        RtPrintf(
                            "SrcBlk:%d State:%s Last:%s FB:%llu Held:%u "
                            "Resumed:%u Term:%u Ord:%u PreAck:%u PreResume:%u "
                            "Done:%u Exact:%u\n",
                            static_cast<int>(member.identity.sourceBlockId),
                            NCOrdinaryG00InflightStateToDiagnosticName(
                                member.currentState),
                            MotionFeedbackTypeToDiagnosticName(
                                member.lastFeedbackType),
                            static_cast<unsigned long long>(
                                member.lastFeedbackSequence),
                            member.heldObserved ? 1U : 0U,
                            member.resumedObserved ? 1U : 0U,
                            member.terminalObserved ? 1U : 0U,
                            static_cast<unsigned int>(member.terminalOrdinal),
                            member.terminalBeforeAcknowledge ? 1U : 0U,
                            member.terminalBeforeResume ? 1U : 0U,
                            member.terminalCompleted ? 1U : 0U,
                            member.exact ? 1U : 0U);
                    }

                    RtPrintf(
                        "[NC02K73-CNT] Try:%llu Prog:%llu BypassOther:%llu "
                        "BypassN:%llu Capture:%llu BTrans:%llu Gate:%llu Ack:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.captureAttempts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.programRequests),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.bypassNotProgram),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.bypassNotTwoActive),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.cohortsCaptured),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.boundaryTransitions),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.gateCorrelations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.holdAcknowledged));
                    RtPrintf(
                        "Early:%llu After:%llu Applied:%llu FbObs:%llu Match:%llu "
                        "Ignore:%llu Held:%llu Resume:%llu Term:%llu Done:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.
                            resumeBeforeAcknowledge),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.
                            resumeAfterAcknowledge),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.resumeApplied),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.feedbackObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.feedbackMatched),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.feedbackIgnored),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.held),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.resumed),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.terminals),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.completedTerminals));
                    RtPrintf(
                        "Interrupt:%llu PreAck:%llu PreResume:%llu Cohort:%llu "
                        "AllDone:%llu IntCohort:%llu Cancel:%llu Super:%llu "
                        "Fail:%llu Influence:%llu MotionWrite:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.interruptedTerminals),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.
                            terminalBeforeAcknowledge),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.terminalBeforeResume),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.terminalCohorts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.allCompletedCohorts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.interruptedCohorts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.cancelled),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.superseded),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.motionWrites));

                    RtPrintf(
                        "[NC02K73-FAIL] Boundary:%llu Registry:%llu Member:%llu "
                        "Session:%llu Identity:%llu Owner:%llu Ledger:%llu "
                        "Order:%llu DupTerm:%llu BMis:%llu GMis:%llu Acct:%u\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.invalidBoundary),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.invalidRegistry),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.invalidMember),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.sessionMismatch),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.identityConflict),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.ownerMismatch),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.ledgerRejected),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.terminalOutOfOrder),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.duplicateTerminal),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.boundaryMismatch),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCounters.gateMismatch),
                        ordinaryG00FeedHoldCohortSnapshot.accountingValid ? 1U : 0U);

                    RtPrintf(
                        "[NC02K74-GATE] Pub:%llu Coh:%llu BSeq:%llu GSeq:%llu "
                        "Phase:%s Decision:%s Sess:%llu Epoch:%u ActiveAt:%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.
                            publicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.cohortSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.boundarySequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.gateSequence),
                        NCOrdinaryG00FeedHoldCohortCutoverPhaseToDiagnosticName(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.phase),
                        NCOrdinaryG00FeedHoldCohortCutoverDecisionToDiagnosticName(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.decision),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.session),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.executionEpoch),
                        ordinaryG00FeedHoldCohortCutoverSnapshot.
                        activeEntriesAtCapture);
                    RtPrintf(
                        "ActiveN:%u Members:%u Term:%u Ack:%u Req:%u Applied:%u "
                        "Exact:%u Bound:%u Wait:%u Release:%u Fallback:%u ",
                        ordinaryG00FeedHoldCohortCutoverSnapshot.
                        registryActiveEntries,
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.memberCount),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortCutoverSnapshot.terminalCount),
                        ordinaryG00FeedHoldCohortCutoverSnapshot.
                        holdAcknowledged ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.
                        resumeRequested ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.
                        resumeApplied ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.exactCohort
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.bound ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.waiting ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.released ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.fallbackLegacy
                        ? 1U : 0U);
                    RtPrintf(
                        "Lock:%u Enabled:%u Influence:%u Bypass:%u MotionWrite:%u "
                        "Acct:%u\n",
                        ordinaryG00FeedHoldCohortCutoverSnapshot.sessionLockout
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.enabled ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.runtimeInfluence
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.resolverBypassed
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.motionWrite
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K74-CNT] Obs:%llu Bind:%llu BypassOff:%llu "
                        "BypassNoCoh:%llu Stale:%llu Eval:%llu Bypass:%llu "
                        "WaitAck:%llu WaitResume:%llu WaitM0:%llu WaitM1:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.observations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.cohortsBound),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.bypassDisabled),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            bypassNoExactCohort),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            staleObservations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.admissionChecks),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            admissionBypasses),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            waitHoldAcknowledge),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.waitResume),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            waitFirstTerminal),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            waitSecondTerminal));
                    RtPrintf(
                        "Release:%llu Allow:%llu Fallback:%llu Int:%llu "
                        "Cancel:%llu Proof:%llu RegMis:%llu AcctMis:%llu "
                        "SessReset:%llu Influence:%llu ResolverBypass:%llu "
                        "MotionWrite:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.releases),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.allowReadAhead),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.fallbackLegacy),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            interruptedCohorts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            cancelledCohorts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.failedProofs),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            registryMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.sessionResets),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            resolverBypasses),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.motionWrites));

                    RtPrintf(
                        "[NC02K74-FAIL] Proof:%llu Reg:%llu Acct:%llu "
                        "Lock:%u Fallback:%u AcctOK:%u\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.failedProofs),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            registryMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortCutoverCounters.
                            accountingMismatches),
                        ordinaryG00FeedHoldCohortCutoverSnapshot.sessionLockout
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.fallbackLegacy
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortCutoverSnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K75-RRM] Pub:%llu Phase:%s Decision:%s Sess:%llu "
                        "Gen:%u Rel:%u SameSess:%u Lease:%u CohSeq:%u BSeq:%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmSnapshot.
                            publicationSequence),
                        NCOrdinaryG00FeedHoldCohortRearmPhaseToDiagnosticName(
                            ordinaryG00FeedHoldCohortRearmSnapshot.phase),
                        NCOrdinaryG00FeedHoldCohortRearmDecisionToDiagnosticName(
                            ordinaryG00FeedHoldCohortRearmSnapshot.decision),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmSnapshot.session),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortRearmSnapshot.generationCount),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldCohortRearmSnapshot.releaseCount),
                        ordinaryG00FeedHoldCohortRearmSnapshot.sameSession ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.sameExecutionLease
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.
                        cohortSequenceMonotonic ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.
                        boundarySequenceMonotonic ? 1U : 0U);
                    RtPrintf(
                        "GSeq:%u Isolate:%u NoOverlap:%u Proven:%u Fail:%u "
                        "Shadow:%u Influence:%u MotionWrite:%u Acct:%u\n",
                        ordinaryG00FeedHoldCohortRearmSnapshot.
                        gateSequenceMonotonic ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.memberIsolation
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.noOverlap ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.rearmProven ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.failed ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.shadowOnly ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.runtimeInfluence
                        ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.motionWrite ? 1U : 0U,
                        ordinaryG00FeedHoldCohortRearmSnapshot.accountingValid
                        ? 1U : 0U);

                    for (std::size_t generationIndex = 0U;
                        generationIndex <
                        NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS;
                        ++generationIndex)
                    {
                        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot&
                            generation =
                            ordinaryG00FeedHoldCohortRearmSnapshot.
                            generations[generationIndex];
                        RtPrintf(
                            "[NC02K75-G%u] Coh:%llu BSeq:%llu GSeq:%llu "
                            "Sess:%llu Epoch:%u Members:%u Term:%u Captured:%u ",
                            static_cast<unsigned int>(generationIndex),
                            static_cast<unsigned long long>(
                                generation.cohortSequence),
                            static_cast<unsigned long long>(
                                generation.boundarySequence),
                            static_cast<unsigned long long>(generation.gateSequence),
                            static_cast<unsigned long long>(generation.session),
                            static_cast<unsigned int>(generation.executionEpoch),
                            static_cast<unsigned int>(generation.memberCount),
                            static_cast<unsigned int>(generation.terminalCount),
                            generation.captured ? 1U : 0U);
                        RtPrintf(
                            "Release:%u Exact:%u Order:%u Done:%u RegEmpty:%u "
                            "E0:%llu D0:%llu S0:%llu E1:%llu D1:%llu S1:%llu\n",
                            generation.released ? 1U : 0U,
                            generation.exact ? 1U : 0U,
                            generation.terminalOrderValid ? 1U : 0U,
                            generation.allMembersCompleted ? 1U : 0U,
                            generation.registryEmptyAtRelease ? 1U : 0U,
                            static_cast<unsigned long long>(
                                generation.memberEntrySequence[0]),
                            static_cast<unsigned long long>(
                                generation.memberDispatchId[0]),
                            static_cast<unsigned long long>(
                                generation.memberIdentity[0].segmentId),
                            static_cast<unsigned long long>(
                                generation.memberEntrySequence[1]),
                            static_cast<unsigned long long>(
                                generation.memberDispatchId[1]),
                            static_cast<unsigned long long>(
                                generation.memberIdentity[1].segmentId));
                    }

                    RtPrintf(
                        "[NC02K75-CNT] Obs:%llu Edge:%llu Bypass:%llu Gen:%llu "
                        "FirstB:%llu FirstR:%llu SecondB:%llu SecondR:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.observations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.cohortEdges),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.nonExactBypasses),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            generationsCaptured),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.firstBound),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.firstReleased),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.secondBound),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.secondReleased));
                    RtPrintf(
                        "Stable:%llu Stale:%llu Early:%llu Sess:%llu Lease:%llu "
                        "CohSeq:%llu BSeq:%llu GSeq:%llu Reuse:%llu Reg:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            stableObservations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            staleObservations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.earlyRearm),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            sessionMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            executionLeaseMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            cohortSequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            boundarySequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            gateSequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            memberIdentityReuse),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            registryMismatches));
                    RtPrintf(
                        "Proof:%llu Fallback:%llu Acct:%llu SessReset:%llu "
                        "Fail:%llu "
                        "Influence:%llu MotionWrite:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.proofMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.fallbackObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.sessionResets),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.motionWrites));

                    RtPrintf(
                        "[NC02K75-FAIL] Early:%llu Sess:%llu Lease:%llu "
                        "CohSeq:%llu BSeq:%llu GSeq:%llu Reuse:%llu Reg:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.earlyRearm),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            sessionMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            executionLeaseMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            cohortSequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            boundarySequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            gateSequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            memberIdentityReuse),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            registryMismatches));
                    RtPrintf(
                        "Proof:%llu Fallback:%llu Acct:%llu Fail:%llu AcctOK:%u\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.proofMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.fallbackObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldCohortRearmCounters.failures),
                        ordinaryG00FeedHoldCohortRearmSnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K76-GATE] Pub:%llu RPub:%llu Phase:%s Decision:%s "
                        "Sess:%llu Coh:%llu BSeq:%llu GSeq:%llu Gen:%u Rel:%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.
                            publicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.
                            rearmPublicationSequence),
                        NCOrdinaryG00FeedHoldRearmCutoverPhaseToDiagnosticName(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.phase),
                        NCOrdinaryG00FeedHoldRearmCutoverDecisionToDiagnosticName(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.decision),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.session),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.cohortSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.
                            boundarySequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.gateSequence),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.generationCount),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRearmCutoverSnapshot.releaseCount));
                    RtPrintf(
                        "Exact:%u Bound:%u Wait:%u Release:%u Fallback:%u "
                        "Lock:%u Enabled:%u Influence:%u MotionWrite:%u Acct:%u\n",
                        ordinaryG00FeedHoldRearmCutoverSnapshot.
                        exactSecondGeneration ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.bound ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.waiting ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.released ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.fallbackLegacy
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.sessionLockout
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.enabled ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.runtimeInfluence
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.motionWrite
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K76-CNT] Obs:%llu Bind:%llu BypassOff:%llu "
                        "BypassPre:%llu Stale:%llu Eval:%llu EvalBypass:%llu "
                        "Wait74:%llu Wait75:%llu Release:%llu Allow:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.observations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            secondGenerationsBound),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.bypassDisabled),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            bypassBeforeSecondGeneration),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            staleObservations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.admissionChecks),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            admissionBypasses),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.waitK74Release),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            waitK75RearmProof),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.releases),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.allowReadAhead));
                    RtPrintf(
                        "Fallback:%llu K75Fail:%llu K74Fail:%llu Sess:%llu "
                        "Seq:%llu Identity:%llu Reg:%llu Acct:%llu "
                        "SessReset:%llu Fail:%llu Influence:%llu MotionWrite:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.fallbackLegacy),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.k75Failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.k74Failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            sessionMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            sequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            identityMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            registryMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.sessionResets),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.motionWrites));

                    RtPrintf(
                        "[NC02K76-FAIL] K75:%llu K74:%llu Sess:%llu Seq:%llu "
                        "Identity:%llu Reg:%llu Acct:%llu Fail:%llu Lock:%u "
                        "Fallback:%u AcctOK:%u\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.k75Failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.k74Failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            sessionMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            sequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            identityMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            registryMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRearmCutoverCounters.failures),
                        ordinaryG00FeedHoldRearmCutoverSnapshot.sessionLockout
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.fallbackLegacy
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRearmCutoverSnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K77-ROLL] Pub:%llu Phase:%s Decision:%s Sess:%llu "
                        "Gen:%llu Rel:%llu RollCap:%llu RollRel:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            publicationSequence),
                        NCOrdinaryG00FeedHoldRollingRearmPhaseName(
                            ordinaryG00FeedHoldRollingRearmSnapshot.phase),
                        NCOrdinaryG00FeedHoldRollingRearmDecisionName(
                            ordinaryG00FeedHoldRollingRearmSnapshot.decision),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.session),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            generationCount),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.releaseCount),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            rollingCaptureCount),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            rollingReleaseCount));
                    RtPrintf(
                        "ActiveOrd:%llu Seed:%u Track:%u Active:%u Continuity:%u "
                        "SameSess:%u SameLease:%u SeqMono:%u Lineage:%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            activeGenerationOrdinal),
                        ordinaryG00FeedHoldRollingRearmSnapshot.seeded ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.tracking
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.active ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.
                        releasedContinuity ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.sameSession
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.sameExecutionLease
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.sequenceMonotonic
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.fullLineageValid
                        ? 1U : 0U);
                    RtPrintf(
                        "NoOverlap:%u Fail:%u Bounded:%u Shadow:%u Influence:%u "
                        "MotionWrite:%u Acct:%u\n",
                        ordinaryG00FeedHoldRollingRearmSnapshot.noOverlap
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.failed ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.bounded ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.shadowOnly
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.runtimeInfluence
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.motionWrite
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingRearmSnapshot.accountingValid
                        ? 1U : 0U);

                    RtPrintf(
                        "[NC02K77-LINE] K75Pub:%llu K76Pub:%llu K73Pub:%llu "
                        "K74Pub:%llu RegPub:%llu SeedCoh:%llu SeedB:%llu "
                        "SeedG:%llu LastCoh:%llu LastB:%llu LastG:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedK75PublicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedK76PublicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedK73PublicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedK74PublicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedRegistryPublicationSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedCohortSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedBoundarySequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            seedGateSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            lastCohortSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            lastBoundarySequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            lastGateSequence));
                    RtPrintf(
                        "RegHW:%llu EntryHW:%llu DispatchHW:%llu CommitHW:%llu "
                        "SegmentHW:%llu Digest:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            registrySequenceHighWater),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            entrySequenceHighWater),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            dispatchIdHighWater),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            commitSequenceHighWater),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.
                            segmentIdHighWater),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmSnapshot.lineageDigest));

                    const NCOrdinaryG00FeedHoldRollingGenerationSnapshot&
                        ordinaryG00FeedHoldRollingGeneration =
                        ordinaryG00FeedHoldRollingRearmSnapshot.active
                        ? ordinaryG00FeedHoldRollingRearmSnapshot.currentActive
                        : ordinaryG00FeedHoldRollingRearmSnapshot.lastReleased;

                    RtPrintf(
                        "[NC02K77-GEN] Ord:%llu Coh:%llu BSeq:%llu GSeq:%llu "
                        "Cap73:%llu Cap74:%llu CapReg:%llu Rel73:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            generationOrdinal),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.cohortSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.boundarySequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.gateSequence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            cohortPublicationAtCapture),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            cutoverPublicationAtCapture),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            registryPublicationAtCapture),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            cohortPublicationAtRelease));
                    RtPrintf(
                        "Rel74:%llu RelReg:%llu Sess:%llu Epoch:%u Owner:%u "
                        "OwnerGen:%u Members:%u Term:%u Seed:%u Rolling:%u ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            cutoverPublicationAtRelease),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.
                            registryPublicationAtRelease),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.session),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRollingGeneration.executionEpoch),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRollingGeneration.owner),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRollingGeneration.ownerGeneration),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRollingGeneration.memberCount),
                        static_cast<unsigned int>(
                            ordinaryG00FeedHoldRollingGeneration.terminalCount),
                        ordinaryG00FeedHoldRollingGeneration.seedEvidence
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.rollingEvidence
                        ? 1U : 0U);
                    RtPrintf(
                        "Captured:%u Released:%u Hold:%u ResumeReq:%u "
                        "ResumeApplied:%u Order:%u Done:%u TermFB:%u "
                        "RegEmpty:%u Digest:%llu\n",
                        ordinaryG00FeedHoldRollingGeneration.captured ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.released ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.holdAcknowledged
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.resumeRequested
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.resumeApplied
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.terminalOrderValid
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.allMembersCompleted
                        ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.
                        terminalFeedbackComplete ? 1U : 0U,
                        ordinaryG00FeedHoldRollingGeneration.
                        registryEmptyAtRelease ? 1U : 0U,
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingGeneration.lineageDigest));

                    for (std::size_t rollingMemberIndex = 0U;
                        rollingMemberIndex <
                        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
                        ++rollingMemberIndex)
                    {
                        const NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot&
                            rollingMember = ordinaryG00FeedHoldRollingGeneration.
                            members[rollingMemberIndex];
                        RtPrintf(
                            "[NC02K77-M%u] Reg:%llu Entry:%llu Dispatch:%llu "
                            "Commit:%llu PC:%d Line:%d Epoch:%u Segment:%llu ",
                            static_cast<unsigned int>(rollingMemberIndex),
                            static_cast<unsigned long long>(
                                rollingMember.registrySequence),
                            static_cast<unsigned long long>(
                                rollingMember.entrySequence),
                            static_cast<unsigned long long>(
                                rollingMember.dispatchId),
                            static_cast<unsigned long long>(
                                rollingMember.commitSequence),
                            rollingMember.sourcePC,
                            rollingMember.sourceLineNumber,
                            static_cast<unsigned int>(rollingMember.identity.epoch),
                            static_cast<unsigned long long>(
                                rollingMember.identity.segmentId));
                        RtPrintf(
                            "SourceBlock:%d Source:%u LeaseOwner:%u LeaseGen:%u "
                            "Fingerprint:%llu Exact:%u TermFB:%u TermDone:%u "
                            "TermOrd:%u TermSeq:%llu TermType:%u\n",
                            static_cast<int>(rollingMember.identity.sourceBlockId),
                            static_cast<unsigned int>(rollingMember.identity.source),
                            static_cast<unsigned int>(rollingMember.ownerLease.owner),
                            static_cast<unsigned int>(
                                rollingMember.ownerLease.generation),
                            static_cast<unsigned long long>(
                                rollingMember.lineageFingerprint),
                            rollingMember.exact ? 1U : 0U,
                            rollingMember.terminalFeedbackReceived ? 1U : 0U,
                            rollingMember.terminalCompleted ? 1U : 0U,
                            static_cast<unsigned int>(rollingMember.terminalOrdinal),
                            static_cast<unsigned long long>(
                                rollingMember.terminalFeedbackSequence),
                            static_cast<unsigned int>(
                                rollingMember.terminalFeedbackType));
                    }

                    RtPrintf(
                        "[NC02K77-CNT] Obs:%llu SeedTry:%llu Wait75:%llu "
                        "Wait76:%llu WaitCurrent:%llu SeedOK:%llu Edge:%llu "
                        "Bypass:%llu Cap:%llu Rel:%llu Stable:%llu Early:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.observations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.seedAttempts),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.waitK75Seed),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.waitK76Seed),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            waitCurrentCohortSeed),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.seedAccepted),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            rollingCohortEdges),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            nonExactBypasses),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.rollingCaptures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.rollingReleases),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            stableObservations),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.earlyEdges));
                    RtPrintf(
                        "Sess:%llu Lease:%llu Seq:%llu Wrap:%llu Lineage:%llu "
                        "Reg:%llu Acct:%llu ActiveInv:%llu Ordinal:%llu "
                        "Proof:%llu Fallback:%llu SessReset:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            sessionMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            executionLeaseMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            sequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            sequenceWrapUnsupported),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            lineageMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            registryMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            activeInvariantMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            ordinalExhausted),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.proofMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.fallbackObserved),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.sessionResets));
                    RtPrintf(
                        "ResetActive:%llu Fail:%llu Influence:%llu "
                        "MotionWrite:%llu\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.resetWhileActive),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.motionWrites));

                    RtPrintf(
                        "[NC02K77-FAIL] Early:%llu Sess:%llu Lease:%llu "
                        "Seq:%llu Wrap:%llu Lineage:%llu Reg:%llu Acct:%llu "
                        "ActiveInv:%llu Ordinal:%llu Proof:%llu Fallback:%llu ",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.earlyEdges),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            sessionMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            executionLeaseMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            sequenceMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            sequenceWrapUnsupported),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            lineageMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            registryMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            accountingMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.
                            activeInvariantMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.ordinalExhausted),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.proofMismatches),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.fallbackObserved));
                    RtPrintf(
                        "ResetActive:%llu Fail:%llu Influence:%llu "
                        "MotionWrite:%llu AcctOK:%u\n",
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.resetWhileActive),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.failures),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.runtimeInfluence),
                        static_cast<unsigned long long>(
                            ordinaryG00FeedHoldRollingRearmCounters.motionWrites),
                        ordinaryG00FeedHoldRollingRearmSnapshot.accountingValid
                        ? 1U : 0U);

                    if (shouldPrintK78)
                    {
                        RtPrintf(
                            "[NC02K78-GATE] Pub:%llu RPub:%llu K74Pub:%llu "
                            "RegPub:%llu Phase:%s Decision:%s Sess:%llu Ord:%llu ",
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                publicationSequence),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                rollingPublicationSequence),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                cohortCutoverPublicationSequence),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                registryPublicationSequence),
                            NCOrdinaryG00FeedHoldRollingCutoverPhaseName(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.phase),
                            NCOrdinaryG00FeedHoldRollingCutoverDecisionName(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.decision),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.session),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                generationOrdinal));
                        RtPrintf(
                            "LastRel:%llu Gen:%llu Rel:%llu Coh:%llu BSeq:%llu "
                            "GSeq:%llu Exact:%u Seed:%u Bound:%u Wait:%u Release:%u ",
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                lastReleasedGenerationOrdinal),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                generationCount),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.releaseCount),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.cohortSequence),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.
                                boundarySequence),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverSnapshot.gateSequence),
                            ordinaryG00FeedHoldRollingCutoverSnapshot.exactGeneration
                            ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.trackingSeeded
                            ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.bound ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.waiting ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.released ? 1U : 0U);
                        RtPrintf(
                            "Fallback:%u Lock:%u Enabled:%u Influence:%u "
                            "MotionWrite:%u Acct:%u\n",
                            ordinaryG00FeedHoldRollingCutoverSnapshot.fallbackLegacy
                            ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.sessionLockout
                            ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.enabled ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.runtimeInfluence
                            ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.motionWrite
                            ? 1U : 0U,
                            ordinaryG00FeedHoldRollingCutoverSnapshot.accountingValid
                            ? 1U : 0U);

                        RtPrintf(
                            "[NC02K78-CNT] Obs:%llu Seed:%llu Bind:%llu "
                            "BypassOff:%llu BypassSeed:%llu BypassIdle:%llu "
                            "Stale:%llu Eval:%llu EvalBypass:%llu Wait74:%llu ",
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.observations),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.trackingSeeds),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                generationsBound),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                bypassDisabled),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.bypassWaitSeed),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                bypassNoActiveGeneration),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                staleObservations),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.admissionChecks),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                admissionBypasses),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.waitK74Release));
                        RtPrintf(
                            "Wait77:%llu Release:%llu Allow:%llu Fallback:%llu "
                            "K77Fail:%llu K74Fail:%llu Sess:%llu Lease:%llu ",
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                waitK77ContinuityProof),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.releases),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.allowReadAhead),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.fallbackLegacy),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.k77Failures),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.k74Failures),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                sessionMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                executionLeaseMismatches));
                        RtPrintf(
                            "Seq:%llu Lineage:%llu Identity:%llu Reg:%llu Acct:%llu "
                            "ActiveInv:%llu SessReset:%llu Fail:%llu Influence:%llu "
                            "MotionWrite:%llu\n",
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                sequenceMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                lineageMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                identityMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                registryMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                accountingMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                activeInvariantMismatches),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.sessionResets),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.failures),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.
                                runtimeInfluence),
                            static_cast<unsigned long long>(
                                ordinaryG00FeedHoldRollingCutoverCounters.motionWrites));
                    }

                });
            RunHmiDiagnosticOutputFamily([&]()
                {
                    const std::uint64_t stopCommandMaxPps =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.maxAxisCommandVelocityAbsPps,
                            1.0);
                    const std::uint64_t stopActualMaxPps =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.maxAxisActualVelocityAbsPps,
                            1.0);
                    const std::uint64_t stopCommandDeadbandPps =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.commandVelocityDeadbandPps,
                            1.0);
                    const std::uint64_t stopActualDeadbandPps =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.actualVelocityDeadbandPps,
                            1.0);
                    const std::uint64_t followingErrorNm =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.worstFollowingErrorAbsMm,
                            1000000.0);
                    const std::uint64_t followingWindowNm =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.worstFollowingErrorWindowMm,
                            1000000.0);
                    const std::uint64_t followingRatioX1000 =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.worstFollowingErrorWindowRatio,
                            1000.0);
                    const std::uint64_t groupFollowingErrorNm =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.worstGroupFollowingErrorAbsMm,
                            1000000.0);
                    const std::uint64_t groupFollowingWindowNm =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.worstGroupFollowingErrorWindowMm,
                            1000000.0);
                    const std::uint64_t groupFollowingRatioX1000 =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.worstGroupFollowingErrorWindowRatio,
                            1000.0);
                    const std::uint64_t maximumStopDecTimeMs =
                        ScaleNonNegativeDiagnosticValue(
                            stopSettleSnapshot.maxConfiguredStopDecTimeSec,
                            1000.0);

                    RtPrintf(
                        "[NC02J4-SET] Gen:%llu Valid:%u Sample:%llu Stand:%u Block:%s "
                        "BAx:%d BState:%s GActive:%u Q:%u/%u/%u GAxes:%u ",
                        static_cast<unsigned long long>(
                            stopSettleSnapshot.publicationGeneration),
                        stopSettleSnapshot.publicationGeneration != 0ULL ? 1U : 0U,
                        static_cast<unsigned long long>(
                            stopSettleSnapshot.sampleSequence),
                        stopSettleSnapshot.standstill ? 1U : 0U,
                        MotionStopSettleBlockerToDiagnosticName(
                            stopSettleSnapshot.primaryBlocker),
                        stopSettleSnapshot.primaryAxisIndex,
                        MotionStateToDiagnosticName(
                            stopSettleSnapshot.primaryAxisState),
                        stopSettleSnapshot.groupActive ? 1U : 0U,
                        static_cast<unsigned int>(
                            stopSettleSnapshot.commandQueueDepth),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.commandIngressDepth),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.commandReplayDepth),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.groupAxisCount));
                    RtPrintf(
                        "Axes:%u NonIdle:%u CmdMove:%u ActMove:%u PDO:%u/%u\n",
                        static_cast<unsigned int>(
                            stopSettleSnapshot.existingAxisCount),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.nonIdleAxisCount),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.commandMovingAxisCount),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.actualMovingAxisCount),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.pdoTargetVelocityNonzeroAxisCount),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.pdoTargetVelocitySampledAxisCount));

                    RtPrintf(
                        "[NC02J4-AX] CmdAx:%d CmdMax:%llu/%llu ActAx:%d "
                        "ActMax:%llu/%llu PDOAx:%d PDOMax:%u FollowAx:%d ",
                        stopSettleSnapshot.worstCommandVelocityAxisIndex,
                        static_cast<unsigned long long>(stopCommandMaxPps),
                        static_cast<unsigned long long>(stopCommandDeadbandPps),
                        stopSettleSnapshot.worstActualVelocityAxisIndex,
                        static_cast<unsigned long long>(stopActualMaxPps),
                        static_cast<unsigned long long>(stopActualDeadbandPps),
                        stopSettleSnapshot.worstFinalPdoTargetVelocityAxisIndex,
                        static_cast<unsigned int>(
                            stopSettleSnapshot.maxFinalPdoTargetVelocityAbs),
                        stopSettleSnapshot.worstFollowingErrorAxisIndex);
                    RtPrintf(
                        "ErrNm:%llu WinNm:%llu RatioX1000:%llu Outside:%u "
                        "GFollowAx:%d GErrNm:%llu GWinNm:%llu GRatioX1000:%llu "
                        "GOutside:%u StopAx:%d StopMs:%llu\n",
                        static_cast<unsigned long long>(followingErrorNm),
                        static_cast<unsigned long long>(followingWindowNm),
                        static_cast<unsigned long long>(followingRatioX1000),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.outsideInPositionWindowAxisCount),
                        stopSettleSnapshot.worstGroupFollowingErrorAxisIndex,
                        static_cast<unsigned long long>(groupFollowingErrorNm),
                        static_cast<unsigned long long>(groupFollowingWindowNm),
                        static_cast<unsigned long long>(groupFollowingRatioX1000),
                        static_cast<unsigned int>(
                            stopSettleSnapshot.groupOutsideInPositionWindowAxisCount),
                        stopSettleSnapshot.maxStopDecTimeAxisIndex,
                        static_cast<unsigned long long>(maximumStopDecTimeMs));

                    RtPrintf(
                        "[NC02J4-MCNT] Sample:%llu Stand:%llu Block:%llu "
                        "GActive:%llu CmdQ:%llu State:%llu CmdVel:%llu ActVel:%llu "
                        "BTrans:%llu STrans:%llu Into:%llu Out:%llu\n",
                        static_cast<unsigned long long>(
                            stopSettleCounters.sampleCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.standstillSampleCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.blockedSampleCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.groupActiveBlockerCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.commandQueueBlockerCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.axisNotIdleBlockerCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.axisCommandVelocityBlockerCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.axisActualVelocityBlockerCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.primaryBlockerTransitionCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.standstillTransitionCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.transitionIntoStandstillCount),
                        static_cast<unsigned long long>(
                            stopSettleCounters.transitionOutOfStandstillCount));

                    const bool resetStandstillSkew =
                        lifecycleInterruptionSnapshot.active &&
                        lifecycleInterruptionSnapshot.groupStandstill !=
                        stopSettleSnapshot.standstill;
                    const bool programEndStandstillSkew =
                        programEndSnapshot.requestPending &&
                        programEndSnapshot.groupStandstill !=
                        stopSettleSnapshot.standstill;
                    const bool barrierStandstillSkew =
                        preDispatchBarrierSnapshot.active &&
                        preDispatchBarrierSnapshot.groupStandstill !=
                        stopSettleSnapshot.standstill;

                    RtPrintf(
                        "[NC02J4-XCHK] RT:%u Reset:%u/%u Skew:%u "
                        "PEnd:%u/%u Skew:%u Bar:%u/%u Skew:%u FH:%u/%u\n",
                        stopSettleSnapshot.standstill ? 1U : 0U,
                        lifecycleInterruptionSnapshot.active ? 1U : 0U,
                        lifecycleInterruptionSnapshot.groupStandstill ? 1U : 0U,
                        resetStandstillSkew ? 1U : 0U,
                        programEndSnapshot.requestPending ? 1U : 0U,
                        programEndSnapshot.groupStandstill ? 1U : 0U,
                        programEndStandstillSkew ? 1U : 0U,
                        preDispatchBarrierSnapshot.active ? 1U : 0U,
                        preDispatchBarrierSnapshot.groupStandstill ? 1U : 0U,
                        barrierStandstillSkew ? 1U : 0U,
                        feedHoldSnapshot.active ? 1U : 0U,
                        feedHoldSnapshot.motion.actualStopped ? 1U : 0U);

                    RtPrintf(
                        "[NC02I-SB] Seq:%llu D:%llu Phase:%s Decision:%s Kind:%s ",
                        static_cast<unsigned long long>(singleBlockShadowSnapshot.sequence),
                        static_cast<unsigned long long>(singleBlockShadowSnapshot.dispatchId),
                        NCSingleBlockShadowPhaseToDiagnosticName(
                            singleBlockShadowSnapshot.phase),
                        NCSingleBlockShadowDecisionToDiagnosticName(
                            singleBlockShadowSnapshot.decision),
                        NCSingleBlockCandidateKindToDiagnosticName(
                            singleBlockShadowSnapshot.candidateKind));
                    RtPrintf(
                        "Active:%u PC:%d Line:%d Commit:%u Cb:%u/%u Txn:%u/%u ",
                        singleBlockShadowSnapshot.active ? 1U : 0U,
                        singleBlockShadowSnapshot.sourcePC,
                        singleBlockShadowSnapshot.sourceLineNumber,
                        singleBlockShadowSnapshot.programCommitted ? 1U : 0U,
                        singleBlockShadowSnapshot.callbackComplete ? 1U : 0U,
                        singleBlockShadowSnapshot.callbackRequired ? 1U : 0U,
                        singleBlockShadowSnapshot.transactionComplete ? 1U : 0U,
                        singleBlockShadowSnapshot.transactionRequired ? 1U : 0U);
                    RtPrintf(
                        "Motion:%s Done:%u Fail:%u Ready:%u Pause:%u Hold:%u Ctrl:%u EndSup:%u\n",
                        NCBlockMotionBoundaryStateToDiagnosticName(
                            singleBlockShadowSnapshot.motionState),
                        singleBlockShadowSnapshot.motionComplete ? 1U : 0U,
                        singleBlockShadowSnapshot.motionFailed ? 1U : 0U,
                        singleBlockShadowSnapshot.boundaryReady ? 1U : 0U,
                        singleBlockShadowSnapshot.legacyPausePending ? 1U : 0U,
                        singleBlockShadowSnapshot.legacyHoldObserved ? 1U : 0U,
                        singleBlockShadowSnapshot.controlledHoldObserved ? 1U : 0U,
                        singleBlockShadowSnapshot.programEndSuppressed ? 1U : 0U);

                    RtPrintf(
                        "[NC02I-CNT] ArmTry:%llu Armed:%llu NotEligible:%llu Eval:%llu "
                        "WaitLife:%llu WaitCommit:%llu WaitTxn:%llu WaitCb:%llu "
                        "WaitMotion:%llu Ready:%llu LegacyHold:%llu Agree:%llu ",
                        static_cast<unsigned long long>(singleBlockShadowCounters.armAttempts),
                        static_cast<unsigned long long>(singleBlockShadowCounters.armed),
                        static_cast<unsigned long long>(singleBlockShadowCounters.notEligible),
                        static_cast<unsigned long long>(singleBlockShadowCounters.evaluations),
                        static_cast<unsigned long long>(singleBlockShadowCounters.waitLifecycle),
                        static_cast<unsigned long long>(singleBlockShadowCounters.waitProgramCommit),
                        static_cast<unsigned long long>(singleBlockShadowCounters.waitTransaction),
                        static_cast<unsigned long long>(singleBlockShadowCounters.waitCallback),
                        static_cast<unsigned long long>(singleBlockShadowCounters.waitMotion),
                        static_cast<unsigned long long>(singleBlockShadowCounters.boundaryReady),
                        static_cast<unsigned long long>(singleBlockShadowCounters.legacyHolds),
                        static_cast<unsigned long long>(singleBlockShadowCounters.agreeHolds));
                    RtPrintf(
                        "Early:%llu Phantom:%llu Missing:%llu CtrlHold:%llu "
                        "CtrlAgree:%llu CtrlEarly:%llu CtrlResume:%llu MotionFail:%llu "
                        "Overflow:%llu TxnFail:%llu EndSup:%llu Resume:%llu ",
                        static_cast<unsigned long long>(singleBlockShadowCounters.legacyEarlyHolds),
                        static_cast<unsigned long long>(singleBlockShadowCounters.legacyHoldWithoutArm),
                        static_cast<unsigned long long>(singleBlockShadowCounters.legacyMissingHold),
                        static_cast<unsigned long long>(singleBlockShadowCounters.controlledHolds),
                        static_cast<unsigned long long>(singleBlockShadowCounters.controlledAgreeHolds),
                        static_cast<unsigned long long>(singleBlockShadowCounters.controlledEarlyHoldAttempts),
                        static_cast<unsigned long long>(singleBlockShadowCounters.controlledResumed),
                        static_cast<unsigned long long>(singleBlockShadowCounters.motionFailures),
                        static_cast<unsigned long long>(singleBlockShadowCounters.trackingOverflow),
                        static_cast<unsigned long long>(singleBlockShadowCounters.transactionFailures),
                        static_cast<unsigned long long>(singleBlockShadowCounters.programEndSuppressed),
                        static_cast<unsigned long long>(singleBlockShadowCounters.resumed));
                    RtPrintf(
                        "Cancel:%llu Supersede:%llu\n",
                        static_cast<unsigned long long>(singleBlockShadowCounters.cancelled),
                        static_cast<unsigned long long>(singleBlockShadowCounters.superseded));

                    RtPrintf(
                        "[NC02I-SG] Seq:%llu BSeq:%llu Enabled:%u Phase:%s Decision:%s "
                        "Active:%u D:%llu PC:%d Line:%d Kind:%s Match:%u Ready:%u ",
                        static_cast<unsigned long long>(singleBlockGateSnapshot.sequence),
                        static_cast<unsigned long long>(singleBlockGateSnapshot.boundarySequence),
                        singleBlockGateSnapshot.enabled ? 1U : 0U,
                        NCSingleBlockHoldGatePhaseToDiagnosticName(
                            singleBlockGateSnapshot.phase),
                        NCSingleBlockHoldGateDecisionToDiagnosticName(
                            singleBlockGateSnapshot.decision),
                        singleBlockGateSnapshot.active ? 1U : 0U,
                        static_cast<unsigned long long>(singleBlockGateSnapshot.dispatchId),
                        singleBlockGateSnapshot.sourcePC,
                        singleBlockGateSnapshot.sourceLineNumber,
                        NCSingleBlockCandidateKindToDiagnosticName(
                            singleBlockGateSnapshot.candidateKind),
                        singleBlockGateSnapshot.boundaryMatched ? 1U : 0U,
                        singleBlockGateSnapshot.boundaryReady ? 1U : 0U);
                    RtPrintf(
                        "HoldReady:%u Hold:%u Resume:%u Explicit:%u EndSup:%u "
                        "Blocked:%u Cancel:%u\n",
                        singleBlockGateSnapshot.holdReady ? 1U : 0U,
                        singleBlockGateSnapshot.holdApplied ? 1U : 0U,
                        singleBlockGateSnapshot.resumeApplied ? 1U : 0U,
                        singleBlockGateSnapshot.explicitStopBypass ? 1U : 0U,
                        singleBlockGateSnapshot.programEndSuppressed ? 1U : 0U,
                        singleBlockGateSnapshot.blocked ? 1U : 0U,
                        singleBlockGateSnapshot.cancelled ? 1U : 0U);

                    RtPrintf(
                        "[NC02I-SCNT] Req:%llu Armed:%llu BypassOff:%llu "
                        "BypassStop:%llu Wait:%llu Ready:%llu Hold:%llu Resume:%llu "
                        "EndSup:%llu BlockMotion:%llu BlockTxn:%llu BlockOv:%llu ",
                        static_cast<unsigned long long>(singleBlockGateCounters.requestAttempts),
                        static_cast<unsigned long long>(singleBlockGateCounters.controlledArms),
                        static_cast<unsigned long long>(singleBlockGateCounters.legacyBypassDisabled),
                        static_cast<unsigned long long>(singleBlockGateCounters.legacyBypassExplicitStop),
                        static_cast<unsigned long long>(singleBlockGateCounters.waitBoundarySamples),
                        static_cast<unsigned long long>(singleBlockGateCounters.holdReady),
                        static_cast<unsigned long long>(singleBlockGateCounters.holdApplied),
                        static_cast<unsigned long long>(singleBlockGateCounters.resumeApplied),
                        static_cast<unsigned long long>(singleBlockGateCounters.programEndSuppressed),
                        static_cast<unsigned long long>(singleBlockGateCounters.blockedMotionFailure),
                        static_cast<unsigned long long>(singleBlockGateCounters.blockedTransactionFailure),
                        static_cast<unsigned long long>(singleBlockGateCounters.blockedTrackingOverflow));
                    RtPrintf(
                        "BlockCancel:%llu Cancel:%llu Super:%llu Rollback:%llu\n",
                        static_cast<unsigned long long>(singleBlockGateCounters.blockedCancelled),
                        static_cast<unsigned long long>(singleBlockGateCounters.cancelled),
                        static_cast<unsigned long long>(singleBlockGateCounters.superseded),
                        static_cast<unsigned long long>(singleBlockGateCounters.rollbackDisabled));

                    const std::uint64_t feedrateOverrideX1000 =
                        ScaleNonNegativeDiagnosticValue(
                            feedHoldSnapshot.motion.feedrateOverride,
                            1000.0);
                    const std::uint64_t maximumCommandVelocityPps =
                        ScaleNonNegativeDiagnosticValue(
                            feedHoldSnapshot.motion.maxAxisCommandVelocityPps,
                            1.0);
                    const std::uint64_t maximumActualVelocityPps =
                        ScaleNonNegativeDiagnosticValue(
                            feedHoldSnapshot.motion.maxAxisActualVelocityPps,
                            1.0);

                    // RTX64 RtPrintf does not reliably support floating-point format
                    // specifiers. Passing doubles to %.3f / %.0f desynchronizes the
                    // remaining varargs and can make a later %s consume a non-pointer,
                    // causing 0xC0000005 inside the diagnostic formatter. Convert all
                    // floating-point values to scaled integers before RtPrintf.
                    RtPrintf(
                        "[NC02I-FH] Seq:%llu Src:%s Phase:%s Decision:%s Active:%u "
                        "PC:%d D:%llu Legacy:%u Ack:%u Stable:%u/%u ",
                        static_cast<unsigned long long>(feedHoldSnapshot.sequence),
                        NCFeedHoldSourceToDiagnosticName(feedHoldSnapshot.source),
                        NCFeedHoldShadowPhaseToDiagnosticName(feedHoldSnapshot.phase),
                        NCFeedHoldShadowDecisionToDiagnosticName(feedHoldSnapshot.decision),
                        feedHoldSnapshot.active ? 1U : 0U,
                        feedHoldSnapshot.requestPC,
                        static_cast<unsigned long long>(feedHoldSnapshot.dispatchId),
                        feedHoldSnapshot.legacyHoldEntered ? 1U : 0U,
                        feedHoldSnapshot.acknowledged ? 1U : 0U,
                        static_cast<unsigned int>(feedHoldSnapshot.stableSamples),
                        static_cast<unsigned int>(feedHoldSnapshot.requiredStableSamples));
                    RtPrintf(
                        "OvrX1000:%llu CmdStop:%u ActStop:%u HomePause:%u "
                        "CmdMaxPps:%llu ActMaxPps:%llu Q:%u/%u/%u ",
                        static_cast<unsigned long long>(feedrateOverrideX1000),
                        feedHoldSnapshot.motion.commandStopped ? 1U : 0U,
                        feedHoldSnapshot.motion.actualStopped ? 1U : 0U,
                        feedHoldSnapshot.homePaused ? 1U : 0U,
                        static_cast<unsigned long long>(maximumCommandVelocityPps),
                        static_cast<unsigned long long>(maximumActualVelocityPps),
                        static_cast<unsigned int>(feedHoldSnapshot.motion.commandQueueDepth),
                        static_cast<unsigned int>(feedHoldSnapshot.motion.commandIngressDepth),
                        static_cast<unsigned int>(feedHoldSnapshot.motion.commandReplayDepth));
                    RtPrintf(
                        "Owner:%s/%u->%s/%u Epoch:%llu->%llu Fail:%u\n",
                        MotionOwnerToDiagnosticName(feedHoldSnapshot.requestOwner),
                        static_cast<unsigned int>(feedHoldSnapshot.requestOwnerGeneration),
                        MotionOwnerToDiagnosticName(feedHoldSnapshot.currentOwner),
                        static_cast<unsigned int>(feedHoldSnapshot.currentOwnerGeneration),
                        static_cast<unsigned long long>(feedHoldSnapshot.requestExecutionEpoch),
                        static_cast<unsigned long long>(feedHoldSnapshot.currentExecutionEpoch),
                        feedHoldSnapshot.failed ? 1U : 0U);

                    RtPrintf(
                        "[NC02I-FCNT] ReqTry:%llu Req:%llu Prog:%llu Home:%llu Eval:%llu "
                        "WaitOvr:%llu WaitCmd:%llu WaitAct:%llu WaitHome:%llu WaitStable:%llu ",
                        static_cast<unsigned long long>(feedHoldCounters.requestAttempts),
                        static_cast<unsigned long long>(feedHoldCounters.requestsLatched),
                        static_cast<unsigned long long>(feedHoldCounters.programRequests),
                        static_cast<unsigned long long>(feedHoldCounters.homeRequests),
                        static_cast<unsigned long long>(feedHoldCounters.evaluations),
                        static_cast<unsigned long long>(feedHoldCounters.waitOverrideZero),
                        static_cast<unsigned long long>(feedHoldCounters.waitCommandStop),
                        static_cast<unsigned long long>(feedHoldCounters.waitActualStop),
                        static_cast<unsigned long long>(feedHoldCounters.waitHomePaused),
                        static_cast<unsigned long long>(feedHoldCounters.waitStable));
                    RtPrintf(
                        "Ack:%llu Legacy:%llu Early:%llu Agree:%llu "
                        "ResumeReq:%llu ResumeEarly:%llu ResumeAck:%llu Resumed:%llu "
                        "OwnerFail:%llu EpochFail:%llu MotionFail:%llu AckLost:%llu ",
                        static_cast<unsigned long long>(feedHoldCounters.acknowledged),
                        static_cast<unsigned long long>(feedHoldCounters.legacyHolds),
                        static_cast<unsigned long long>(feedHoldCounters.legacyEarlyHolds),
                        static_cast<unsigned long long>(feedHoldCounters.legacyAgreeHolds),
                        static_cast<unsigned long long>(feedHoldCounters.resumeRequests),
                        static_cast<unsigned long long>(feedHoldCounters.resumeBeforeAcknowledge),
                        static_cast<unsigned long long>(feedHoldCounters.resumeAfterAcknowledge),
                        static_cast<unsigned long long>(feedHoldCounters.resumed),
                        static_cast<unsigned long long>(feedHoldCounters.ownerChanged),
                        static_cast<unsigned long long>(feedHoldCounters.executionEpochChanged),
                        static_cast<unsigned long long>(feedHoldCounters.motionFault),
                        static_cast<unsigned long long>(feedHoldCounters.acknowledgeLost));
                    RtPrintf(
                        "Cancel:%llu Super:%llu\n",
                        static_cast<unsigned long long>(feedHoldCounters.cancelled),
                        static_cast<unsigned long long>(feedHoldCounters.superseded));


                    RtPrintf(
                        "[NC02I-FG] Seq:%llu BSeq:%llu Enabled:%u Phase:%s Decision:%s "
                        "Active:%u Pending:%u Ack:%u Release:%u Applied:%u ",
                        static_cast<unsigned long long>(feedHoldGateSnapshot.sequence),
                        static_cast<unsigned long long>(feedHoldGateSnapshot.boundarySequence),
                        feedHoldGateSnapshot.enabled ? 1U : 0U,
                        NCFeedHoldResumeGatePhaseToDiagnosticName(
                            feedHoldGateSnapshot.phase),
                        NCFeedHoldResumeGateDecisionToDiagnosticName(
                            feedHoldGateSnapshot.decision),
                        feedHoldGateSnapshot.active ? 1U : 0U,
                        feedHoldGateSnapshot.deferredUntilAcknowledge ? 1U : 0U,
                        feedHoldGateSnapshot.acknowledgeObserved ? 1U : 0U,
                        feedHoldGateSnapshot.releaseReady ? 1U : 0U,
                        feedHoldGateSnapshot.resumeApplied ? 1U : 0U);
                    RtPrintf(
                        "Blocked:%u Cancel:%u Src:%s PC:%d D:%llu "
                        "Owner:%s/%u Epoch:%llu BActive:%u BFail:%u BCancel:%u\n",
                        feedHoldGateSnapshot.blocked ? 1U : 0U,
                        feedHoldGateSnapshot.cancelled ? 1U : 0U,
                        NCFeedHoldSourceToDiagnosticName(feedHoldGateSnapshot.source),
                        feedHoldGateSnapshot.requestPC,
                        static_cast<unsigned long long>(feedHoldGateSnapshot.dispatchId),
                        MotionOwnerToDiagnosticName(feedHoldGateSnapshot.owner),
                        static_cast<unsigned int>(feedHoldGateSnapshot.ownerGeneration),
                        static_cast<unsigned long long>(feedHoldGateSnapshot.executionEpoch),
                        feedHoldGateSnapshot.boundaryActive ? 1U : 0U,
                        feedHoldGateSnapshot.boundaryFailed ? 1U : 0U,
                        feedHoldGateSnapshot.boundaryCancelled ? 1U : 0U);

                    RtPrintf(
                        "[NC02I-GCNT] Req:%llu BypassOff:%llu BypassOther:%llu "
                        "Deferred:%llu Immediate:%llu Duplicate:%llu Release:%llu "
                        "Applied:%llu BlockFail:%llu BlockCancel:%llu Cancel:%llu ",
                        static_cast<unsigned long long>(feedHoldGateCounters.requestAttempts),
                        static_cast<unsigned long long>(feedHoldGateCounters.legacyBypassDisabled),
                        static_cast<unsigned long long>(feedHoldGateCounters.legacyBypassNotProgramFeedHold),
                        static_cast<unsigned long long>(feedHoldGateCounters.deferredBeforeAcknowledge),
                        static_cast<unsigned long long>(feedHoldGateCounters.immediateAfterAcknowledge),
                        static_cast<unsigned long long>(feedHoldGateCounters.duplicateRequests),
                        static_cast<unsigned long long>(feedHoldGateCounters.releaseOnAcknowledge),
                        static_cast<unsigned long long>(feedHoldGateCounters.resumeApplied),
                        static_cast<unsigned long long>(feedHoldGateCounters.blockedBoundaryFailed),
                        static_cast<unsigned long long>(feedHoldGateCounters.blockedBoundaryCancelled),
                        static_cast<unsigned long long>(feedHoldGateCounters.cancelled));
                    RtPrintf(
                        "Super:%llu Rollback:%llu\n",
                        static_cast<unsigned long long>(feedHoldGateCounters.superseded),
                        static_cast<unsigned long long>(feedHoldGateCounters.rollbackDisabled));

                    if (shouldPrintJ6)
                    {
                        RtPrintf(
                            "[NC02J6-ALM] Seq:%llu Code:%d Axis:%d Trigger:%s "
                            "Phase:%s Decision:%s Active:%u Ack:%u Alarm:%u "
                            "Upd:%u Count:%d LSeq:%llu\n",
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.sequence),
                            static_cast<int>(
                                alarmEmergencyStopSnapshot.alarmCode),
                            static_cast<int>(
                                alarmEmergencyStopSnapshot.alarmAxisIndex),
                            NCAlarmEmergencyStopTriggerToDiagnosticName(
                                alarmEmergencyStopSnapshot.trigger),
                            NCAlarmEmergencyStopPhaseToDiagnosticName(
                                alarmEmergencyStopSnapshot.phase),
                            NCAlarmEmergencyStopDecisionToDiagnosticName(
                                alarmEmergencyStopSnapshot.decision),
                            alarmEmergencyStopSnapshot.active ? 1U : 0U,
                            alarmEmergencyStopSnapshot.acknowledged ? 1U : 0U,
                            alarmEmergencyStopSnapshot.alarmActive ? 1U : 0U,
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.alarmUpdateCount),
                            static_cast<int>(
                                alarmEmergencyStopSnapshot.alarmCount),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.lifecycleSequence));

                        RtPrintf(
                            "[NC02J6-RT] Pub:%llu Req:%llu>%llu Apply:%llu>%llu "
                            "Delta:%llu Inv:%llu>%llu Epoch:%llu>%llu Last:%llu ",
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.motionPublicationGeneration),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.requestPublishedBaseline),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.requestPublishedCurrent),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.rtApplyBaseline),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.rtApplyCurrent),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.rtApplyDelta),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.epochInvalidationBaseline),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.epochInvalidationCurrent),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.requestExecutionEpoch),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.currentExecutionEpoch),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.lastAppliedExecutionEpoch));
                        RtPrintf(
                            "Need:%u Owner:%s/%u LastOwner:%s/%u Match:%u\n",
                            alarmEmergencyStopSnapshot.epochChangeRequired ? 1U : 0U,
                            MotionOwnerToDiagnosticName(
                                alarmEmergencyStopSnapshot.currentOwner),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.currentOwnerGeneration),
                            MotionOwnerToDiagnosticName(
                                alarmEmergencyStopSnapshot.lastAppliedOwner),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.lastAppliedOwnerGeneration),
                            alarmEmergencyStopSnapshot.safetyOwnerMatched ? 1U : 0U);

                        RtPrintf(
                            "[NC02J632-INV] From:%llu To:%llu Count:%llu "
                            "Correlated:%u PreLatched:%u\n",
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.
                                lastInvalidatedFromExecutionEpoch),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.
                                lastInvalidatedToExecutionEpoch),
                            static_cast<unsigned long long>(
                                alarmEmergencyStopSnapshot.
                                lastInvalidationCount),
                            alarmEmergencyStopSnapshot.
                            epochInvalidationCorrelated ? 1U : 0U,
                            alarmEmergencyStopSnapshot.
                            preLatchedRTApplication ? 1U : 0U);

                        RtPrintf(
                            "[NC02J6-AX] Expect:%08X Current:%08X ESTOP:%08X "
                            "Error:%08X Cmd0:%08X Seal:%08X PDO:%08X/%08X ",
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.expectedAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.currentAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.estopAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.errorAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.commandZeroAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.targetSealedAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.pdoZeroAxisMask),
                            static_cast<unsigned int>(
                                alarmEmergencyStopSnapshot.pdoSampledAxisMask));
                        RtPrintf(
                            "Group:%u AxisSafe:%u CmdAll0:%u SealAll:%u PDOAll0:%u "
                            "Coh:%u ReqSeen:%u RTAck:%u EpochOK:%u FbSync:%u "
                            "Gap:%u Post:%u\n",
                            alarmEmergencyStopSnapshot.groupStopApplied ? 1U : 0U,
                            alarmEmergencyStopSnapshot.allExistingAxesSafe ? 1U : 0U,
                            alarmEmergencyStopSnapshot.allExistingAxisCommandsZero ? 1U : 0U,
                            alarmEmergencyStopSnapshot.allExistingAxisTargetsSealed ? 1U : 0U,
                            alarmEmergencyStopSnapshot.allSampledPdoTargetVelocitiesZero ? 1U : 0U,
                            alarmEmergencyStopSnapshot.emergencyEvidenceCoherent ? 1U : 0U,
                            alarmEmergencyStopSnapshot.emergencyRequestObserved ? 1U : 0U,
                            alarmEmergencyStopSnapshot.rtApplyObserved ? 1U : 0U,
                            alarmEmergencyStopSnapshot.executionEpochMatched ? 1U : 0U,
                            alarmEmergencyStopSnapshot.feedbackSequenceSynchronized ? 1U : 0U,
                            alarmEmergencyStopSnapshot.lifecycleEvidenceGap ? 1U : 0U,
                            alarmEmergencyStopSnapshot.postAlarmDispatchObserved ? 1U : 0U);

                        RtPrintf(
                            "[NC02J6-CNT] ReqTry:%llu Req:%llu EStop:%llu "
                            "Protect:%llu Limit:%llu Drive:%llu Lag:%llu NC:%llu "
                            "EDM:%llu Other:%llu Eval:%llu WaitPub:%llu ",
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.requestAttempts),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.requestsLatched),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.emergencyStopTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.axisProtectionTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.hardLimitTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.driveFaultTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.lagErrorTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.ncProgramTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.edmProcessTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.otherTriggers),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.evaluations),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitPublication));
                        RtPrintf(
                            "WaitReq:%llu WaitRT:%llu WaitOwner:%llu WaitEpoch:%llu "
                            "WaitGroup:%llu WaitAxis:%llu WaitCmd:%llu WaitSeal:%llu "
                            "Ack:%llu PreCorr:%llu PreAck:%llu ClearEarly:%llu ",
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitRequest),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitRTApply),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitSafetyOwner),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitExecutionEpoch),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitGroupStop),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitAxisSafeState),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitCommandZero),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.waitTargetSealed),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.acknowledged),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.preLatchedCorrelations),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.preLatchedAcknowledged),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.clearedBeforeAcknowledge));
                        RtPrintf(
                            "Gap:%llu Replace:%llu "
                            "AxisScope:%llu Super:%llu\n",
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.lifecycleEvidenceGap),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.lifecycleReplaced),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.axisScopeChanged),
                            static_cast<unsigned long long>(alarmEmergencyStopCounters.superseded));
                    }

                });
            RunHmiDiagnosticOutputFamily([&]()
                {
                    RtPrintf(
                        "[NC02J-INT] Seq:%llu Cause:%s Phase:%s Decision:%s Active:%u ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.sequence),
                        NCLifecycleInterruptionCauseToDiagnosticName(
                            lifecycleInterruptionSnapshot.cause),
                        NCLifecycleInterruptionPhaseToDiagnosticName(
                            lifecycleInterruptionSnapshot.phase),
                        NCLifecycleInterruptionDecisionToDiagnosticName(
                            lifecycleInterruptionSnapshot.decision),
                        lifecycleInterruptionSnapshot.active ? 1U : 0U);
                    RtPrintf(
                        "EpochExp:%u Epoch:%llu>%llu>%llu Owner:%s/%u>%s/%u "
                        "ExecOwner:%s/%u ",
                        lifecycleInterruptionSnapshot.expectsEpochChange ? 1U : 0U,
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.requestExecutionEpoch),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.publishedExecutionEpoch),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.currentExecutionEpoch),
                        MotionOwnerToDiagnosticName(
                            lifecycleInterruptionSnapshot.requestOwner),
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.requestOwnerGeneration),
                        MotionOwnerToDiagnosticName(
                            lifecycleInterruptionSnapshot.currentOwner),
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.currentOwnerGeneration),
                        MotionOwnerToDiagnosticName(
                            lifecycleInterruptionSnapshot.requestExecutionOwner),
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.
                            requestExecutionOwnerGeneration));
                    RtPrintf(
                        "D:%llu PC:%d Blocks:%u>%u Term:%s TSeq:%llu Seg:%llu "
                        "SrcBlk:%d LedgerOK:%u TermSeen:%u AlarmAck:%u ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.requestDispatchId),
                        lifecycleInterruptionSnapshot.requestPC,
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.requestActiveBlocks),
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.activeBlocks),
                        MotionFeedbackTypeToDiagnosticName(
                            lifecycleInterruptionSnapshot.lastTerminalFeedbackType),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.lastTerminalFeedbackSequence),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.lastTerminalIdentity.segmentId),
                        static_cast<int>(
                            lifecycleInterruptionSnapshot.lastTerminalIdentity.sourceBlockId),
                        lifecycleInterruptionSnapshot.terminalFeedbackLedgerAccepted ? 1U : 0U,
                        lifecycleInterruptionSnapshot.terminalFailureObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.alarmStopAcknowledged ? 1U : 0U);
                    RtPrintf(
                        "RTEpoch:%u PreLatched:%u ExpAbort:%u ExpRetire:%u "
                        "PreWin:%u ClassOK:%u StopClosed:%u "
                        "ResetRetire:%u ResetClass:%u ResetCommit:%u "
                        "UnexpectedEpoch:%u PostDispatch:%u\n",
                        lifecycleInterruptionSnapshot.runtimeAlarmEpochChangeObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.alarmStopPreLatchedRTApplication ? 1U : 0U,
                        lifecycleInterruptionSnapshot.expectedAlarmAbortObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.expectedAlarmPreReadRejectObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.expectedAlarmPreLatchedTerminalObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.alarmTerminalClassificationValid ? 1U : 0U,
                        lifecycleInterruptionSnapshot.alarmStopClosed ? 1U : 0U,
                        lifecycleInterruptionSnapshot.expectedResetPreReadRejectObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.resetTerminalClassificationValid ? 1U : 0U,
                        lifecycleInterruptionSnapshot.resetRetirementCommitted ? 1U : 0U,
                        lifecycleInterruptionSnapshot.unexpectedEpochChangeObserved ? 1U : 0U,
                        lifecycleInterruptionSnapshot.postInterruptionDispatchObserved ? 1U : 0U);

                    RtPrintf(
                        "[NC02J-DRN] AxisQ:%llu AxisR:%llu CmdQ:%llu In:%llu "
                        "Replay:%llu Fb:%llu Notice:%llu FbSeq:%llu/%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.axisCommandDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.axisResultDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.commandQueueDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.commandIngressDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.commandReplayDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackNoticeDepth),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.lastPublishedFeedbackSequence),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.lastConsumedFeedbackSequence));
                    RtPrintf(
                        "Safety:%u Cb:%u Bind:%u Stand:%u Stable:%u/%u "
                        "Ready:%u Quiet:%u Gap:%u Dispatch:%llu DltFail:%llu ",
                        lifecycleInterruptionSnapshot.safetyOrRecoveryPending ? 1U : 0U,
                        lifecycleInterruptionSnapshot.waitCallbackActive ? 1U : 0U,
                        lifecycleInterruptionSnapshot.completionBindingActive ? 1U : 0U,
                        lifecycleInterruptionSnapshot.groupStandstill ? 1U : 0U,
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.stableSamples),
                        static_cast<unsigned int>(
                            lifecycleInterruptionSnapshot.requiredStableSamples),
                        lifecycleInterruptionSnapshot.quiescentReady ? 1U : 0U,
                        lifecycleInterruptionSnapshot.quiescent ? 1U : 0U,
                        lifecycleInterruptionSnapshot.evidenceGap ? 1U : 0U,
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.dispatchDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.unexpectedBlockFailureDelta));
                    RtPrintf(
                        "RawFail:%llu ExpAbort:%llu ExpReject:%llu UnexpReject:%llu "
                        "ExpOwner:%llu ExpStale:%llu PreAbort:%llu PreReject:%llu "
                        "RstReject:%llu RstOwner:%llu RstStale:%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.blockFailureDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedAlarmAbortDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedAlarmPreReadRejectDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.unexpectedFeedbackRejectedDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedAlarmOwnerConflictRejectDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedAlarmStaleEpochRejectDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedAlarmPreLatchedAbortDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedAlarmPreLatchedRejectDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedResetPreReadRejectDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedResetOwnerConflictRejectDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.expectedResetStaleEpochRejectDelta));
                    RtPrintf(
                        "Rej:%llu Cancel:%llu "
                        "Abort:%llu Fault:%llu Ledger:%llu "
                        "FbOv:%llu NoticeOv:%llu SeqGap:%llu\n",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackRejectedDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackCancelledDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackAbortedDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackFaultedDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.ledgerIntegrityDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackOverflowDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackNoticeOverflowDelta),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionSnapshot.feedbackSequenceGapDelta));

                    RtPrintf(
                        "[NC02J-CNT] ReqTry:%llu Req:%llu Reset:%llu Alarm:%llu "
                        "Prog:%llu MDI:%llu Manual:%llu Dynamic:%llu Goto:%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.requestAttempts),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.requestsLatched),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.resetRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.alarmRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.programReplaceRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.mdiReplaceRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.manualAutoReplaceRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.dynamicCodeReplaceRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.gotoEpochRequests));
                    RtPrintf(
                        "TermReq:%llu EpochExp:%llu EpochObs:%llu Unexpected:%llu "
                        "EpochSuper:%llu AlarmAck:%llu RTEpoch:%llu ExpAbort:%llu "
                        "ExpReject:%llu ExpOwner:%llu ExpStale:%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.terminalTriggeredRequests),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.epochPublicationsExpected),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.epochPublicationsObserved),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.unexpectedEpochChanges),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.epochSuperseded),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.alarmStopAcknowledgements),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.runtimeAlarmEpochChanges),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedAlarmAborts),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedAlarmPreReadRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedAlarmOwnerConflictRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedAlarmStaleEpochRejects));
                    RtPrintf(
                        "PreAbort:%llu PreReject:%llu RstReject:%llu "
                        "RstOwner:%llu RstStale:%llu StopClose:%llu "
                        "TRej:%llu TCancel:%llu TAbort:%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedAlarmPreLatchedAborts),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedAlarmPreLatchedRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedResetPreReadRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedResetOwnerConflictRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.expectedResetStaleEpochRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.alarmStopsClosed),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.terminalRejected),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.terminalCancelled),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.terminalAborted));
                    RtPrintf(
                        "TFault:%llu LedgerReject:%llu PostDispatch:%llu Eval:%llu Quiet:%llu "
                        "Gap:%llu Super:%llu\n",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.terminalFaulted),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.terminalLedgerRejected),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.postInterruptionDispatch),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.evaluations),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.quiescent),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.evidenceGap),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.superseded));

                    RtPrintf(
                        "[NC02J631-TRN] RawTotal:%llu Unexpected:%llu "
                        "RawOwner:%llu RawStale:%llu RawFbRej:%llu "
                        "ExpOwner:%llu ExpStale:%llu ExpFbRej:%llu "
                        "AlarmOwner:%llu AlarmStale:%llu AlarmFb:%llu ",
                        static_cast<unsigned long long>(transportRawErrorTotal),
                        static_cast<unsigned long long>(transportErrorTotal),
                        static_cast<unsigned long long>(ownerConflictReject),
                        static_cast<unsigned long long>(staleDiscard),
                        static_cast<unsigned long long>(feedbackRejected),
                        static_cast<unsigned long long>(expectedOwnerConflictReject),
                        static_cast<unsigned long long>(expectedStaleDiscard),
                        static_cast<unsigned long long>(expectedFeedbackRejected),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.
                            expectedAlarmOwnerConflictRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.
                            expectedAlarmStaleEpochRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.
                            expectedAlarmPreReadRejects));
                    RtPrintf(
                        "ResetOwner:%llu ResetStale:%llu ResetFb:%llu "
                        "ResidualOwner:%llu ResidualStale:%llu ResidualFb:%llu\n",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.
                            expectedResetOwnerConflictRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.
                            expectedResetStaleEpochRejects),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.
                            expectedResetPreReadRejects),
                        static_cast<unsigned long long>(
                            unexpectedOwnerConflictReject),
                        static_cast<unsigned long long>(unexpectedStaleDiscard),
                        static_cast<unsigned long long>(unexpectedFeedbackRejected));

                    RtPrintf(
                        "[NC02J-WAIT] Epoch:%llu Blocks:%llu AxisQ:%llu AxisR:%llu "
                        "In:%llu Replay:%llu CmdQ:%llu Notice:%llu Fb:%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitEpochPublication),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitActiveBlocks),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitAxisCommand),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitAxisResult),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitCommandIngress),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitCommandReplay),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitCommandQueue),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitFeedbackNotice),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitFeedback));
                    RtPrintf(
                        "FbSeq:%llu Cb:%llu Bind:%llu Safety:%llu Stand:%llu "
                        "Stable:%llu AlarmAck:%llu AlarmTerm:%llu AlarmStable:%llu "
                        "GapFb:%llu GapNotice:%llu GapSeq:%llu ",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitFeedbackSequence),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitCallback),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitCompletionBinding),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitSafetyRequest),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitGroupStandstill),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitStableConfirmation),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitAlarmStopAcknowledgement),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitAlarmStopTerminal),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.waitAlarmStopStable),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.feedbackOverflowEvidenceGap),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.feedbackNoticeOverflowEvidenceGap),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.feedbackSequenceEvidenceGap));
                    RtPrintf(
                        "GapLedger:%llu GapReject:%llu\n",
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.ledgerIntegrityEvidenceGap),
                        static_cast<unsigned long long>(
                            lifecycleInterruptionCounters.ledgerRejectedEvidenceGap));

                    RtPrintf(
                        "[NC02J-RG] Seq:%llu BSeq:%llu Phase:%s Decision:%s "
                        "Active:%u Epoch:%llu/%llu/%llu Match:%u/%u ",
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.sequence),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.boundarySequence),
                        NCResetReleaseGatePhaseToDiagnosticName(
                            resetReleaseGateSnapshot.phase),
                        NCResetReleaseGateDecisionToDiagnosticName(
                            resetReleaseGateSnapshot.decision),
                        resetReleaseGateSnapshot.active ? 1U : 0U,
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.expectedExecutionEpoch),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.boundaryPublishedExecutionEpoch),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.boundaryCurrentExecutionEpoch),
                        resetReleaseGateSnapshot.publishedEpochMatched ? 1U : 0U,
                        resetReleaseGateSnapshot.currentEpochMatched ? 1U : 0U);
                    RtPrintf(
                        "Safety:%s/%u Lease:%u/%u BOwner:%s/%u OMatch:%u "
                        "BPhase:%s BDecision:%s Stable:%u/%u Stand:%u ",
                        MotionOwnerToDiagnosticName(
                            resetReleaseGateSnapshot.safetyOwner),
                        static_cast<unsigned int>(
                            resetReleaseGateSnapshot.safetyOwnerGeneration),
                        resetReleaseGateSnapshot.safetyLeaseValid ? 1U : 0U,
                        resetReleaseGateSnapshot.safetyLeaseCurrent ? 1U : 0U,
                        MotionOwnerToDiagnosticName(
                            resetReleaseGateSnapshot.boundaryCurrentOwner),
                        static_cast<unsigned int>(
                            resetReleaseGateSnapshot.boundaryCurrentOwnerGeneration),
                        resetReleaseGateSnapshot.boundaryOwnerMatched ? 1U : 0U,
                        NCLifecycleInterruptionPhaseToDiagnosticName(
                            resetReleaseGateSnapshot.boundaryPhase),
                        NCLifecycleInterruptionDecisionToDiagnosticName(
                            resetReleaseGateSnapshot.boundaryDecision),
                        static_cast<unsigned int>(
                            resetReleaseGateSnapshot.stableSamples),
                        static_cast<unsigned int>(
                            resetReleaseGateSnapshot.requiredStableSamples),
                        resetReleaseGateSnapshot.groupStandstill ? 1U : 0U);
                    RtPrintf(
                        "Quiet:%u Ready:%u Attempt:%u Applied:%u Block:%u\n",
                        resetReleaseGateSnapshot.quiescenceProved ? 1U : 0U,
                        resetReleaseGateSnapshot.releaseReady ? 1U : 0U,
                        resetReleaseGateSnapshot.releaseAttempted ? 1U : 0U,
                        resetReleaseGateSnapshot.releaseApplied ? 1U : 0U,
                        resetReleaseGateSnapshot.blocked ? 1U : 0U);

                    RtPrintf(
                        "[NC02J-RCNT] ArmTry:%llu Arm:%llu ReArm:%llu "
                        "Eval:%llu Wait:%llu Ready:%llu RelTry:%llu Released:%llu ",
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.armAttempts),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.armed),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.supersededArms),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.evaluations),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.waitQuiescence),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.releaseReady),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.releaseAttempts),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.released));
                    RtPrintf(
                        "Inv:%llu Seq:%llu Cause:%llu Gap:%llu Super:%llu "
                        "Incomplete:%llu Epoch:%llu Lease:%llu ReleaseFail:%llu "
                        "PostDispatch:%llu\n",
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedInvalidBoundary),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedBoundarySequence),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedBoundaryCause),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedEvidenceGap),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedSuperseded),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedIncomplete),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedEpochMismatch),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedSafetyLease),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedOwnerRelease),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.blockedPostInterruptionDispatch));
                });
        }

        if (shouldPrintJ5)
        {
            RunHmiDiagnosticOutputFamily([&]()
                {
                    // J.5 lines are transition/event driven.  J.4 raw standstill,
                    // encoder velocity, and PDO diagnostics above remain available
                    // for A/B evidence, but none of them can trigger this block.
                    PrintNCSettleDiagnostic(
                        groupNCSettleCoherent,
                        groupNCSettleSnapshot,
                        groupNCSettleCounters);
                    PrintNCSettleDiagnostic(
                        feedHoldNCSettleCoherent,
                        feedHoldNCSettleSnapshot,
                        feedHoldNCSettleCounters);
                    PrintNCSettleDiagnostic(
                        resetNCSettleCoherent,
                        resetNCSettleSnapshot,
                        resetNCSettleCounters);

                    RtPrintf(
                        "[NC02K721-PDO] Coh:%u Seq:%llu Event:%s Tick:%llu "
                        "Invalid:%llu Episode:%llu Recover:%llu TickGap:%llu "
                        "ExpGap:%llu J5Inv:%llu/%llu/%llu ",
                        pdoInvalidCorrelationCoherent ? 1U : 0U,
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.eventSequence),
                        PdoRuntimeInvalidEventToDiagnosticName(
                            pdoInvalidCorrelation.lastEventKind),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastEventRuntimeCycleTick),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.invalidCycleCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.invalidEpisodeCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.recoveryAfterInvalidCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.validTickGapCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.expectedNCSettleGapCount),
                        static_cast<unsigned long long>(
                            groupNCSettleCounters.invalidRuntimeCycleCount),
                        static_cast<unsigned long long>(
                            feedHoldNCSettleCounters.invalidRuntimeCycleCount),
                        static_cast<unsigned long long>(
                            resetNCSettleCounters.invalidRuntimeCycleCount));
                    RtPrintf(
                        "J5Gap:%llu/%llu/%llu Corr:%u\n",
                        static_cast<unsigned long long>(
                            groupNCSettleCounters.runtimeGapCount),
                        static_cast<unsigned long long>(
                            feedHoldNCSettleCounters.runtimeGapCount),
                        static_cast<unsigned long long>(
                            resetNCSettleCounters.runtimeGapCount),
                        pdoInvalidProfileCorrelationMatched ? 1U : 0U);

                    RtPrintf(
                        "[NC02K721-SRC] Reason:%s Mask:%02X EMask:%02X "
                        "ELRW:%d/%d EDC:%d EPath:%llu "
                        "LastInv:%llu LastRec:%llu Gap:%llu>%llu ",
                        PdoRuntimeInvalidReasonToDiagnosticName(
                            pdoInvalidDisplayedReasonMask),
                        static_cast<unsigned int>(
                            pdoInvalidDisplayedReasonMask),
                        static_cast<unsigned int>(
                            pdoInvalidCorrelation.lastEventReasonMask),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastEventActualLrwWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastEventExpectedLrwWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastEventDcWkc),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastEventCombinedPathNs),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastInvalidRuntimeCycleTick),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastRecoveryRuntimeCycleTick),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastGapFromRuntimeCycleTick),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastGapToRuntimeCycleTick));
                    RtPrintf(
                        "LRW:%d/%d DC:%d Rec:%d/%d/%d Req:%u Sub:%u ",
                        static_cast<int>(
                            pdoInvalidCorrelation.lastInvalidActualLrwWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastInvalidExpectedLrwWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastInvalidDcWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastRecoveryActualLrwWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastRecoveryExpectedLrwWkc),
                        static_cast<int>(
                            pdoInvalidCorrelation.lastRecoveryDcWkc),
                        pdoInvalidCorrelation.dcReferenceRequired ? 1U : 0U,
                        static_cast<unsigned int>(
                            pdoInvalidCorrelation.lastEventSubTick));
                    RtPrintf(
                        "PathNs:%llu/%llu Consec:%u Max:%u Len:%u "
                        "Negative:%llu WkcMis:%llu DcBad:%llu Both:%llu "
                        "Contract:%llu RT:%llu/%llu\n",
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastInvalidCombinedPathNs),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lastRecoveryCombinedPathNs),
                        static_cast<unsigned int>(
                            pdoInvalidCorrelation.currentConsecutiveInvalidCycles),
                        static_cast<unsigned int>(
                            pdoInvalidCorrelation.maximumConsecutiveInvalidCycles),
                        static_cast<unsigned int>(
                            pdoInvalidCorrelation.lastRecoveredEpisodeLength),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lrwCallNegativeCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.lrwWkcMismatchCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.dcWkcInvalidCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.combinedLrwDcInvalidCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.validityContractMismatchCount),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.
                            runtimePdoNegativeTotalAtLastEvent),
                        static_cast<unsigned long long>(
                            pdoInvalidCorrelation.
                            runtimePdoWkcErrorTotalAtLastEvent));

                    RtPrintf(
                        "[NC02K722-ALM] Coh:%u Latched:%u Seq:%llu Req:%llu "
                        "Episode:%llu RePub:%llu Tick:%llu InvEpisode:%llu ",
                        pdoSafetyStopCauseCoherent ? 1U : 0U,
                        pdoSafetyStopCause.publicationSequence != 0ULL ? 1U : 0U,
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.publicationSequence),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.alarmRequestCount),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.alarmedInvalidEpisodeCount),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.alarmRepublishCount),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.runtimeCycleTick),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.invalidEpisodeCount));
                    RtPrintf(
                        "Alarm:%d Reason:%s Mask:%02X LRW:%d/%d DC:%d "
                        "DCReq:%u Sub:%u PathNs:%llu Consec:%u Contain:%u ",
                        static_cast<int>(pdoSafetyStopCause.alarmCode),
                        PdoRuntimeInvalidReasonToDiagnosticName(
                            pdoSafetyStopCause.reasonMask),
                        static_cast<unsigned int>(
                            pdoSafetyStopCause.reasonMask),
                        static_cast<int>(pdoSafetyStopCause.actualLrwWkc),
                        static_cast<int>(pdoSafetyStopCause.expectedLrwWkc),
                        static_cast<int>(pdoSafetyStopCause.dcWkc),
                        pdoSafetyStopCause.dcReferenceRequired ? 1U : 0U,
                        static_cast<unsigned int>(pdoSafetyStopCause.subTick),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.combinedPathNs),
                        static_cast<unsigned int>(
                            pdoSafetyStopCause.consecutiveInvalidCycles),
                        pdoSafetyStopCause.safetyContainmentApplied ? 1U : 0U);
                    RtPrintf(
                        "RePubNow:%u RT:%llu/%llu\n",
                        pdoSafetyStopCause.alarmRequestRepublished ? 1U : 0U,
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.runtimePdoNegativeTotal),
                        static_cast<unsigned long long>(
                            pdoSafetyStopCause.runtimePdoWkcErrorTotal));

                    RtPrintf(
                        "[NC02J5-RB] Kind:ACK Req:%llu Phase:%s Block:%s "
                        "Epoch:%u Owner:%s/%u Mask:%u/%u Accept:%u Apply:%u ",
                        static_cast<unsigned long long>(
                            resetRebaseAck.requestSequence),
                        MotionNCResetRebasePhaseToDiagnosticName(
                            resetRebaseAck.phase),
                        MotionNCSettleBlockerToDiagnosticName(
                            resetRebaseAck.failureBlocker),
                        static_cast<unsigned int>(
                            resetRebaseAck.executionEpoch),
                        MotionOwnerToDiagnosticName(resetRebaseAck.owner),
                        static_cast<unsigned int>(
                            resetRebaseAck.ownerGeneration),
                        static_cast<unsigned int>(
                            resetRebaseAck.requestedAxisMask),
                        static_cast<unsigned int>(
                            resetRebaseAck.appliedAxisMask),
                        resetRebaseAck.requestAccepted ? 1U : 0U,
                        resetRebaseAck.rebaseApplied ? 1U : 0U);
                    RtPrintf(
                        "Post:%u Ack:%u Acked:%u Blocked:%u Super:%u "
                        "FaultEstop:%u Comp:%u\n",
                        resetRebaseAck.postVerifyPassed ? 1U : 0U,
                        resetRebaseAck.acknowledged ? 1U : 0U,
                        resetRebaseAck.acked ? 1U : 0U,
                        resetRebaseAck.blocked ? 1U : 0U,
                        resetRebaseAck.superseded ? 1U : 0U,
                        resetRebaseAck.unsupportedFaultOrEstop ? 1U : 0U,
                        resetRebaseAck.compensationBlocked ? 1U : 0U);

                    RtPrintf(
                        "[NC02J5-RB] Kind:GATE Seq:%llu BSeq:%llu Phase:%s "
                        "Decision:%s Active:%u Req:%llu/%llu AckEpoch:%u ",
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.sequence),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.boundarySequence),
                        NCResetReleaseGatePhaseToDiagnosticName(
                            resetReleaseGateSnapshot.phase),
                        NCResetReleaseGateDecisionToDiagnosticName(
                            resetReleaseGateSnapshot.decision),
                        resetReleaseGateSnapshot.active ? 1U : 0U,
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.expectedResetRequestSequence),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.ackRequestSequence),
                        static_cast<unsigned int>(
                            resetReleaseGateSnapshot.ackExecutionEpoch));
                    RtPrintf(
                        "AckOwner:%s/%u AckMask:%llu/%llu AckPhase:%s ",
                        MotionOwnerToDiagnosticName(
                            resetReleaseGateSnapshot.ackOwner),
                        static_cast<unsigned int>(
                            resetReleaseGateSnapshot.ackOwnerGeneration),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.ackRequestedAxisMask),
                        static_cast<unsigned long long>(
                            resetReleaseGateSnapshot.ackAppliedAxisMask),
                        MotionNCResetRebasePhaseToDiagnosticName(
                            resetReleaseGateSnapshot.ackPhase));
                    RtPrintf(
                        "Obs:%u Match:%u/%u/%u/%u/%u Accept:%u Apply:%u ",
                        resetReleaseGateSnapshot.ackObserved ? 1U : 0U,
                        resetReleaseGateSnapshot.ackSequenceMatched ? 1U : 0U,
                        resetReleaseGateSnapshot.ackEpochMatched ? 1U : 0U,
                        resetReleaseGateSnapshot.ackOwnerMatched ? 1U : 0U,
                        resetReleaseGateSnapshot.ackAxisMaskMatched ? 1U : 0U,
                        resetReleaseGateSnapshot.ackPhaseAcknowledged ? 1U : 0U,
                        resetReleaseGateSnapshot.ackRequestAccepted ? 1U : 0U,
                        resetReleaseGateSnapshot.ackRebaseApplied ? 1U : 0U);
                    RtPrintf(
                        "Post:%u Ack:%u RebaseMatch:%u Release:%u Block:%u "
                        "WaitAck:%llu AckFail:%llu RbFail:%llu\n",
                        resetReleaseGateSnapshot.ackPostVerifyPassed ? 1U : 0U,
                        resetReleaseGateSnapshot.ackAcknowledged ? 1U : 0U,
                        resetReleaseGateSnapshot.rebaseAckMatched ? 1U : 0U,
                        resetReleaseGateSnapshot.releaseApplied ? 1U : 0U,
                        resetReleaseGateSnapshot.blocked ? 1U : 0U,
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.waitRebaseAck),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.ackMismatch),
                        static_cast<unsigned long long>(
                            resetReleaseGateCounters.rebaseFailed));

                    RtPrintf(
                        "[NC02J5-FAIL] Total:%llu SetInvalid:%llu SetGap:%llu "
                        "SetReject:%llu SetRead:%llu Rebase:%llu ResetGate:%llu "
                        "FeedHold:%llu ProgramEnd:%llu Transport:%llu\n",
                        static_cast<unsigned long long>(j5FailureTotal),
                        static_cast<unsigned long long>(j5SetInvalid),
                        static_cast<unsigned long long>(j5SetGap),
                        static_cast<unsigned long long>(j5SetReject),
                        static_cast<unsigned long long>(j5SetReadFailure),
                        static_cast<unsigned long long>(j5RebaseFailure),
                        static_cast<unsigned long long>(j5ResetGateFailure),
                        static_cast<unsigned long long>(j5FeedHoldFailure),
                        static_cast<unsigned long long>(j5ProgramEndFailure),
                        static_cast<unsigned long long>(transportErrorTotal));
                });
        }

        startupSamples = 1U;
        previousSafetyPending = safetyPending;
        previousTransportBusy = transportBusy;
        previousDiagnosticNCState = diagnosticNCState;
        previousDiagnosticMode = diagnosticMode;

        previousOwner = ownerLease.owner;
        previousGeneration = ownerLease.generation;
        previousTransportErrorTotal = transportErrorTotal;
        previousLifecycleChangeToken = lifecycleChangeToken;
        previousCompletionBoundaryChangeToken =
            completionBoundaryChangeToken;
        previousProgramEndChangeToken = programEndChangeToken;
        previousGMTransactionChangeToken = gmTransactionChangeToken;
        previousPreDispatchBarrierChangeToken =
            preDispatchBarrierChangeToken;
        previousPreparedQueueChangeToken =
            preparedQueueChangeToken;
        previousPreparedEquivalenceChangeToken =
            preparedEquivalenceChangeToken;
        previousOrdinaryG00FeedHoldRollingCutoverPublication =
            ordinaryG00FeedHoldRollingCutoverSnapshot.publicationSequence;
        previousSingleBlockShadowChangeToken =
            singleBlockShadowChangeToken;
        previousSingleBlockGateChangeToken =
            singleBlockGateChangeToken;
        previousFeedHoldChangeToken =
            feedHoldChangeToken;
        previousFeedHoldGateChangeToken =
            feedHoldGateChangeToken;
        previousLifecycleInterruptionChangeToken =
            lifecycleInterruptionChangeToken;
        previousResetReleaseGateChangeToken =
            resetReleaseGateChangeToken;
        previousAlarmEmergencyStopChangeToken =
            alarmEmergencyStopChangeToken;
        previousJ5EventToken = j5EventToken;
        previousP1HandoverChangeToken = p1HandoverChangeToken;
        previousCommandPathModeChangeToken =
            commandPathModeChangeToken;
        previousQueueTailTransactionChangeToken =
            queueTailTransactionChangeToken;
    }
}

#undef HMI_DIAG_NOINLINE
