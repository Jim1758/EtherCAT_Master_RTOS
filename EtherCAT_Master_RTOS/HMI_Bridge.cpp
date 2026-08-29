#include "HMI_Bridge.h"
#include "SHMManager.h"
#include "GlobalConfig.h" 
#include "NCManager.h"
#include "AlarmManager.h"
#include <cstring> 
#include "PLCManager.h"
#include "NCPLCMap.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <rtapi.h>

namespace HMI_Bridge
{
    namespace
    {
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
            default:                       return "UNKNOWN";
            }
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

        std::uint64_t BuildNCSettleDiagnosticEventToken(
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

        void PrintNCSettleDiagnostic(
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
                "Epoch:%u Owner:%s/%u Mask:%u Obs:%u Valid:%u Contig:%u "
                "Accept:%u Drain:%u Cand:%u Set:%u Pre:%u Post:%u "
                "Block:%s Ax:%d Dwell:%u/%u\n",
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
                snapshot.runtimeCycleContiguous ? 1U : 0U,
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
                "ExcX1000:%llu/%llu ExcAx:%d RawPps:%llu PDO:%u "
                "Start:%llu Reset:%llu Rise:%llu Revoke:%llu "
                "Invalid:%llu Gap:%llu IdReset:%llu ScopeReset:%llu "
                "Reject:%llu ReadFail:%llu\n",
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
                    snapshot.advisoryMaxPdoTargetVelocityAbs),
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

        pShm->NC_Status.SHM_NC_State = static_cast<int32_t>(nc->m_state);
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
        if (pShm->NC_Command.cycleStart) { nc->CycleStart(); pShm->NC_Command.cycleStart = false; }
        if (pShm->NC_Command.feedHold) { nc->FeedHold(); pShm->NC_Command.feedHold = false; }
        if (pShm->NC_Command.reset) { nc->Reset(); pShm->NC_Command.reset = false; }
        if (pShm->NC_Command.Close_System) { nc->Close_System_Com_flag = true; pShm->NC_Command.Close_System = false; }


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
            nc->CoordSys.SetWCS(pShm->Coord_Command.targetWCS_GCode, nc);
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


        if (pShm->NC_Command.reqChangeMode)//處理 OP 模式切換請求 (來自 NC_Command)
        {
            nc->ChangeMode(static_cast<NCOperationMode>(pShm->NC_Command.targetMode));
            pShm->NC_Command.reqChangeMode = false;
        }

        if (pShm->NC_Command.loadProgramReq)
        {
            size_t safeLength = strnlen(pShm->NC_Command.loadprogramName, 256);
            std::string NC_loadprogramName(pShm->NC_Command.loadprogramName, safeLength);
            nc->LoadProgram(GlobalConfig::GetInstance().NCProgramDir + NC_loadprogramName + ".nc");
            pShm->NC_Command.loadProgramReq = false;
            std::memset(pShm->NC_Command.loadprogramName, 0, sizeof(pShm->NC_Command.loadprogramName));
        }



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
            // 🌟 宣告 unitScale 讓這個區塊也能用公英制轉換
            double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;

            int row = pShm->Coord_Command.rowIndex;
            int axis = pShm->Coord_Command.axisIndex;
            double val = pShm->Coord_Command.writeValue;

            if (axis >= 0 && axis < 8)
            {
                // 🌟 取得對應軸的屬性，旋轉軸(度數)不套用英制轉換
                AxisType type = nc->m_motion.GetAxisContext(axis).axisType;
                double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

                // 🌟 關鍵：將 HMI 輸入的數值 (可能為 inch) 除以倍率，還原回系統底層的公制 (mm)
                double systemVal = val / axisScale;

                switch (pShm->Coord_Command.offsetType)
                {
                case 0: nc->CoordSys.extOffset[axis] = systemVal; break;
                case 1: if (row >= 0 && row < 60)  nc->CoordSys.m_WCSTable[row][axis] = systemVal; break;
                case 2: if (row >= 0 && row < 100) nc->CoordSys.m_ToolOffset[row][axis] = systemVal; break;
                case 3: if (row >= 0 && row < 100) nc->CoordSys.m_WorkOffset[row][axis] = systemVal; break;
                case 10:
                    // G54 等原點設定功能：此處 systemVal 是從 HMI 傳入的期望座標
                    nc->CoordSys.m_WCSTable[nc->CoordSys.currentWCSIndex][axis] =
                        nc->CoordSys.actualMCS[axis] - nc->CoordSys.extOffset[axis] - systemVal;
                    break;
                }
            }
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
                pShm->Coord_Table.workOffset[row][col] = nc->CoordSys.m_WorkOffset[row][col] * axisScale;
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
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

        // =============================================================
        // Stage NC-0.1F.2 - No-motion Runtime Acceptance Diagnostics
        //
        // This code runs only in the existing 1000 ms supervisory task.
        // It never writes AxisContext and never consumes either command
        // or feedback rings.  It only reads atomic / snapshot counters.
        //
        // Output policy:
        //   1. Print the first 15 one-second samples after startup.
        //   2. Afterwards, print only while a transport is busy, Safety is
        //      pending, Owner changes, or an error counter changes.
        //
        // This keeps the acceptance trace useful without creating an
        // unlimited one-line-per-second production log.
        // =============================================================
        MotionCore& motion = nc->GetMotion();

        const MotionStartupLagArmingEvidence startupLagArming =
            motion.GetStartupLagArmingEvidence();

        MotionStopSettleSnapshot stopSettleSnapshot{};
        MotionStopSettleCounters stopSettleCounters{};
        motion.GetStopSettleEvidence(
            stopSettleSnapshot,
            stopSettleCounters);

        MotionNCSettleSnapshot groupNCSettleSnapshot{};
        MotionNCSettleCounters groupNCSettleCounters{};
        const bool groupNCSettleCoherent =
            motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::GROUP_COMPLETION,
                groupNCSettleSnapshot,
                groupNCSettleCounters);

        MotionNCSettleSnapshot feedHoldNCSettleSnapshot{};
        MotionNCSettleCounters feedHoldNCSettleCounters{};
        const bool feedHoldNCSettleCoherent =
            motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::FEED_HOLD_GROUP,
                feedHoldNCSettleSnapshot,
                feedHoldNCSettleCounters);

        MotionNCSettleSnapshot resetNCSettleSnapshot{};
        MotionNCSettleCounters resetNCSettleCounters{};
        const bool resetNCSettleCoherent =
            motion.TryGetNCSettleEvidence(
                MotionNCSettleProfile::RESET_ALL,
                resetNCSettleSnapshot,
                resetNCSettleCounters);

        const MotionNCResetRebaseAck resetRebaseAck =
            motion.GetNCResetRebaseAck();

        const MotionOwnerLease ownerLease =
            motion.GetMotionOwnerLease();

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

        NCBlockLifecycleSnapshot blockLifecycle{};
        const bool hasBlockLifecycle =
            nc->GetLastBlockLifecycleSnapshot(blockLifecycle);
        const NCBlockLifecycleCounters blockCounters =
            nc->GetBlockLifecycleCounters();

        const NCBlockCompletionBoundarySnapshot completionBoundary =
            nc->GetLastBlockCompletionBoundarySnapshot();
        const NCBlockCompletionBoundaryCounters completionCounters =
            nc->GetBlockCompletionBoundaryCounters();

        const NCProgramEndGateSnapshot programEndSnapshot =
            nc->GetProgramEndGateSnapshot();
        const NCProgramEndGateCounters programEndCounters =
            nc->GetProgramEndGateCounters();

        const NCGMBlockTransactionSnapshot gmTransactionSnapshot =
            nc->GetGMBlockTransactionSnapshot();
        const NCGMBlockTransactionCounters gmTransactionCounters =
            nc->GetGMBlockTransactionCounters();

        const NCPreDispatchBarrierSnapshot preDispatchBarrierSnapshot =
            nc->GetPreDispatchBarrierSnapshot();
        const NCPreDispatchBarrierCounters preDispatchBarrierCounters =
            nc->GetPreDispatchBarrierCounters();

        const NCSingleBlockShadowSnapshot singleBlockShadowSnapshot =
            nc->GetSingleBlockShadowSnapshot();
        const NCSingleBlockShadowCounters singleBlockShadowCounters =
            nc->GetSingleBlockShadowCounters();

        const NCSingleBlockHoldGateSnapshot singleBlockGateSnapshot =
            nc->GetSingleBlockHoldGateSnapshot();
        const NCSingleBlockHoldGateCounters singleBlockGateCounters =
            nc->GetSingleBlockHoldGateCounters();

        const NCFeedHoldBoundarySnapshot feedHoldSnapshot =
            nc->GetFeedHoldBoundarySnapshot();
        const NCFeedHoldBoundaryCounters feedHoldCounters =
            nc->GetFeedHoldBoundaryCounters();

        const NCFeedHoldResumeGateSnapshot feedHoldGateSnapshot =
            nc->GetFeedHoldResumeGateSnapshot();
        const NCFeedHoldResumeGateCounters feedHoldGateCounters =
            nc->GetFeedHoldResumeGateCounters();

        const NCLifecycleInterruptionSnapshot lifecycleInterruptionSnapshot =
            nc->GetLifecycleInterruptionSnapshot();
        const NCLifecycleInterruptionCounters lifecycleInterruptionCounters =
            nc->GetLifecycleInterruptionCounters();

        const NCResetReleaseGateSnapshot resetReleaseGateSnapshot =
            nc->GetResetReleaseGateSnapshot();
        const NCResetReleaseGateCounters resetReleaseGateCounters =
            nc->GetResetReleaseGateCounters();

        const NCAlarmEmergencyStopSnapshot alarmEmergencyStopSnapshot =
            nc->GetAlarmEmergencyStopSnapshot();
        const NCAlarmEmergencyStopCounters alarmEmergencyStopCounters =
            nc->GetAlarmEmergencyStopCounters();

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

        // Stage NC-0.2J.6.3.1: retain the existing layered transport accounting
        // (cause counter plus feedback terminal) for every unexpected failure.
        // Remove only the two matching layers of a pre-read retirement proved
        // by the exact Alarm lifecycle; unrelated errors remain fail-closed.
        const std::uint64_t unexpectedOwnerConflictReject =
            SubtractDiagnosticCounterFloor(
                ownerConflictReject,
                lifecycleInterruptionCounters.
                expectedAlarmOwnerConflictRejects);
        const std::uint64_t unexpectedStaleDiscard =
            SubtractDiagnosticCounterFloor(
                staleDiscard,
                lifecycleInterruptionCounters.
                expectedAlarmStaleEpochRejects);
        const std::uint64_t unexpectedFeedbackRejected =
            SubtractDiagnosticCounterFloor(
                feedbackRejected,
                lifecycleInterruptionCounters.
                expectedAlarmPreReadRejects);
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
        static MotionOwner previousOwner = MotionOwner::NONE;
        static MotionOwnerGeneration previousGeneration =
            MOTION_OWNER_GENERATION_INVALID;
        static std::uint64_t previousTransportErrorTotal = 0ULL;
        static std::uint64_t previousLifecycleChangeToken = 0ULL;
        static std::uint64_t previousCompletionBoundaryChangeToken = 0ULL;
        static std::uint64_t previousProgramEndChangeToken = 0ULL;
        static std::uint64_t previousGMTransactionChangeToken = 0ULL;
        static std::uint64_t previousPreDispatchBarrierChangeToken = 0ULL;
        static std::uint64_t previousSingleBlockShadowChangeToken = 0ULL;
        static std::uint64_t previousSingleBlockGateChangeToken = 0ULL;
        static std::uint64_t previousFeedHoldChangeToken = 0ULL;
        static std::uint64_t previousFeedHoldGateChangeToken = 0ULL;
        static std::uint64_t previousLifecycleInterruptionChangeToken = 0ULL;
        static std::uint64_t previousResetReleaseGateChangeToken = 0ULL;
        static std::uint64_t previousAlarmEmergencyStopChangeToken = 0ULL;
        static std::uint64_t previousJ5EventToken = 0ULL;

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

        const bool gmTransactionChanged =
            startupSamples != 0U &&
            gmTransactionChangeToken != previousGMTransactionChangeToken;

        const bool preDispatchBarrierChanged =
            startupSamples != 0U &&
            preDispatchBarrierChangeToken !=
            previousPreDispatchBarrierChangeToken;

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

        const bool shouldPrintJ5 =
            startupSamples == 0U ||
            j5EventChanged;

        const bool shouldPrintJ6 =
            startupSamples == 0U ||
            alarmEmergencyStopChanged;

        const bool shouldPrint =
            startupSamples < 15U ||
            lifecycleChanged ||
            completionBoundaryChanged ||
            programEndChanged ||
            gmTransactionChanged ||
            preDispatchBarrierChanged ||
            singleBlockShadowChanged ||
            singleBlockGateChanged ||
            feedHoldChanged ||
            feedHoldGateChanged ||
            lifecycleInterruptionChanged ||
            resetReleaseGateChanged ||
            alarmEmergencyStopChanged ||
            shouldPrintJ5 ||
            ownerChanged ||
            errorCounterChanged ||
            safetyPending ||
            transportBusy;

        if (shouldPrint)
        {
            RtPrintf(
                "[NC02J64-LAG] Existing:%02X Ready:%02X Aligned:%02X "
                "Armed:%02X Pending:%02X Blocked:%02X Stable:%u/%u "
                "Align:%llu Arm:%llu Reset:%llu Early:%llu All:%u\n",
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
                    startupLagArming.stableSamplesRequired),
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
                "[NC01F-CNT] AxisQFull:%llu AxisResOv:%llu "
                "OwnerReject:%llu Stale:%llu CmdFull:%llu ReplayOv:%llu "
                "FbOv:%llu NoticeOv:%llu FbProc:%llu FbGap:%llu "
                "FbAcc:%llu FbStart:%llu FbDone:%llu FbReject:%llu "
                "FbCancel:%llu FbAbort:%llu FbFault:%llu\n",
                static_cast<unsigned long long>(axisQueueFull),
                static_cast<unsigned long long>(axisResultOverflow),
                static_cast<unsigned long long>(ownerConflictReject),
                static_cast<unsigned long long>(staleDiscard),
                static_cast<unsigned long long>(commandQueueFull),
                static_cast<unsigned long long>(replayOverflow),
                static_cast<unsigned long long>(feedbackOverflow),
                static_cast<unsigned long long>(feedbackNoticeOverflow),
                static_cast<unsigned long long>(feedbackProcessed),
                static_cast<unsigned long long>(feedbackSequenceGap),
                static_cast<unsigned long long>(feedbackAccepted),
                static_cast<unsigned long long>(feedbackStarted),
                static_cast<unsigned long long>(feedbackCompleted),
                static_cast<unsigned long long>(feedbackRejected),
                static_cast<unsigned long long>(feedbackCancelled),
                static_cast<unsigned long long>(feedbackAborted),
                static_cast<unsigned long long>(feedbackFaulted));

            RtPrintf(
                "[NC02D-BLK] D:%llu Scope:%u Cache:%llu Frame:%llu "
                "PC:%d Line:%d State:%s Commit:%u Seg:%u "
                "Acc:%u Start:%u Done:%u Term:%u Fail:%u Ov:%u\n",
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
                static_cast<unsigned int>(blockLifecycle.motionSegmentCount) : 0U,
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
                "FAcc:%llu FStart:%llu FDone:%llu FRej:%llu "
                "FCancel:%llu FAbort:%llu FFault:%llu "
                "BlkDone:%llu BlkFail:%llu NCFail:%llu "
                "CaptureOv:%llu Orphan:%llu DupTerm:%llu Conflict:%llu "
                "Active:%u\n",
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
                static_cast<unsigned long long>(blockCounters.feedbackRejected),
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
                "LegacyEarly:%llu LedgerFail:%llu ReleaseFail:%llu "
                "Overflow:%llu Missing:%llu NonMotion:%llu Supersede:%llu\n",
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
                static_cast<unsigned long long>(completionCounters.releaseOnLedgerFailure),
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
                "Scope:%u Cache:%llu RunEpoch:%u ReqEpoch:%u "
                "RunOwner:%u/%u ReqOwner:%u/%u CurOwner:%u/%u "
                "PC:%d Line:%d D:%llu Pending:%u Ready:%u Fail:%u\n",
                static_cast<unsigned long long>(programEndSnapshot.run.runId),
                static_cast<unsigned long long>(programEndSnapshot.requestId),
                NCProgramEndCauseToDiagnosticName(programEndSnapshot.cause),
                NCProgramEndPhaseToDiagnosticName(programEndSnapshot.phase),
                NCProgramEndDecisionToDiagnosticName(programEndSnapshot.decision),
                static_cast<unsigned int>(programEndSnapshot.run.scope),
                static_cast<unsigned long long>(programEndSnapshot.run.cacheGeneration),
                static_cast<unsigned int>(programEndSnapshot.run.executionEpoch),
                static_cast<unsigned int>(programEndSnapshot.requestExecutionEpoch),
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
                "CmdIn:%llu Replay:%llu FbQ:%llu Notice:%llu "
                "FbPub:%llu FbCon:%llu FbSync:%u Stable:%u/%u "
                "Stand:%u Safety:%u Wait:%u Bind:%u OwnerOK:%u Integrity:%llu\n",
                static_cast<unsigned int>(programEndSnapshot.activeBlocks),
                static_cast<unsigned long long>(programEndSnapshot.axisCommandDepth),
                static_cast<unsigned long long>(programEndSnapshot.axisResultDepth),
                static_cast<unsigned long long>(programEndSnapshot.commandQueueDepth),
                static_cast<unsigned long long>(programEndSnapshot.commandIngressDepth),
                static_cast<unsigned long long>(programEndSnapshot.commandReplayDepth),
                static_cast<unsigned long long>(programEndSnapshot.feedbackDepth),
                static_cast<unsigned long long>(programEndSnapshot.feedbackNoticeDepth),
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
                "Req:%llu EOF:%llu M02:%llu M30:%llu ReqReject:%llu "
                "Eval:%llu Ready:%llu Final:%llu Cancel:%llu Fail:%llu "
                "EpochFail:%llu OwnerFail:%llu IntegrityFail:%llu\n",
                static_cast<unsigned long long>(programEndCounters.runStartAttempts),
                static_cast<unsigned long long>(programEndCounters.runsStarted),
                static_cast<unsigned long long>(programEndCounters.runStartBlocked),
                static_cast<unsigned long long>(programEndCounters.requests),
                static_cast<unsigned long long>(programEndCounters.naturalEofRequests),
                static_cast<unsigned long long>(programEndCounters.m02Requests),
                static_cast<unsigned long long>(programEndCounters.m30Requests),
                static_cast<unsigned long long>(programEndCounters.rejectedRequests),
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
                "Ingress:%llu Replay:%llu Notice:%llu Feedback:%llu FbSeq:%llu "
                "Callback:%llu Binding:%llu Safety:%llu Standstill:%llu Stable:%llu\n",
                static_cast<unsigned long long>(programEndCounters.waitActiveBlocks),
                static_cast<unsigned long long>(programEndCounters.waitAxisCommand),
                static_cast<unsigned long long>(programEndCounters.waitAxisResult),
                static_cast<unsigned long long>(programEndCounters.waitCommandQueue),
                static_cast<unsigned long long>(programEndCounters.waitCommandIngress),
                static_cast<unsigned long long>(programEndCounters.waitCommandReplay),
                static_cast<unsigned long long>(programEndCounters.waitFeedbackNotice),
                static_cast<unsigned long long>(programEndCounters.waitFeedback),
                static_cast<unsigned long long>(programEndCounters.waitFeedbackSequence),
                static_cast<unsigned long long>(programEndCounters.waitCallback),
                static_cast<unsigned long long>(programEndCounters.waitCompletionBinding),
                static_cast<unsigned long long>(programEndCounters.waitSafetyRequest),
                static_cast<unsigned long long>(programEndCounters.waitGroupStandstill),
                static_cast<unsigned long long>(programEndCounters.waitStableConfirmation));


            RtPrintf(
                "[NC02H-TXN] Seq:%llu D:%llu Phase:%s Post:%s Active:%u "
                "GReq:%u GDone:%u MReq:%u MDone:%u PC:%d Line:%d "
                "M:%d P:%d L:%d Main:%u Applied:%u\n",
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
                gmTransactionSnapshot.sourceLineNumber,
                gmTransactionSnapshot.mCode,
                gmTransactionSnapshot.pValue,
                gmTransactionSnapshot.repeatCount,
                gmTransactionSnapshot.fromMainProgram ? 1U : 0U,
                gmTransactionSnapshot.postActionApplied ? 1U : 0U);

            RtPrintf(
                "[NC02H-CNT] Start:%llu GComp:%llu MComp:%llu Dual:%llu "
                "Eval:%llu GWait:%llu MWait:%llu Ready:%llu Final:%llu "
                "Cancel:%llu Fail:%llu M00:%llu M01:%llu M98:%llu "
                "M99:%llu M02:%llu M30:%llu\n",
                static_cast<unsigned long long>(gmTransactionCounters.started),
                static_cast<unsigned long long>(gmTransactionCounters.gWaitComponents),
                static_cast<unsigned long long>(gmTransactionCounters.mWaitComponents),
                static_cast<unsigned long long>(gmTransactionCounters.dualComponentTransactions),
                static_cast<unsigned long long>(gmTransactionCounters.evaluations),
                static_cast<unsigned long long>(gmTransactionCounters.gWaitSamples),
                static_cast<unsigned long long>(gmTransactionCounters.mWaitSamples),
                static_cast<unsigned long long>(gmTransactionCounters.readyTransitions),
                static_cast<unsigned long long>(gmTransactionCounters.finalized),
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
                "BAx:%d BState:%s GActive:%u Q:%u/%u/%u GAxes:%u "
                "Axes:%u NonIdle:%u CmdMove:%u ActMove:%u PDO:%u/%u\n",
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
                    stopSettleSnapshot.groupAxisCount),
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
                "ActMax:%llu/%llu PDOAx:%d PDOMax:%u FollowAx:%d "
                "ErrNm:%llu WinNm:%llu RatioX1000:%llu Outside:%u "
                "GFollowAx:%d GErrNm:%llu GWinNm:%llu GRatioX1000:%llu "
                "GOutside:%u StopAx:%d StopMs:%llu\n",
                stopSettleSnapshot.worstCommandVelocityAxisIndex,
                static_cast<unsigned long long>(stopCommandMaxPps),
                static_cast<unsigned long long>(stopCommandDeadbandPps),
                stopSettleSnapshot.worstActualVelocityAxisIndex,
                static_cast<unsigned long long>(stopActualMaxPps),
                static_cast<unsigned long long>(stopActualDeadbandPps),
                stopSettleSnapshot.worstFinalPdoTargetVelocityAxisIndex,
                static_cast<unsigned int>(
                    stopSettleSnapshot.maxFinalPdoTargetVelocityAbs),
                stopSettleSnapshot.worstFollowingErrorAxisIndex,
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
                "[NC02I-SB] Seq:%llu D:%llu Phase:%s Decision:%s Kind:%s "
                "Active:%u PC:%d Line:%d Commit:%u Cb:%u/%u Txn:%u/%u "
                "Motion:%s Done:%u Fail:%u Ready:%u Pause:%u Hold:%u Ctrl:%u EndSup:%u\n",
                static_cast<unsigned long long>(singleBlockShadowSnapshot.sequence),
                static_cast<unsigned long long>(singleBlockShadowSnapshot.dispatchId),
                NCSingleBlockShadowPhaseToDiagnosticName(
                    singleBlockShadowSnapshot.phase),
                NCSingleBlockShadowDecisionToDiagnosticName(
                    singleBlockShadowSnapshot.decision),
                NCSingleBlockCandidateKindToDiagnosticName(
                    singleBlockShadowSnapshot.candidateKind),
                singleBlockShadowSnapshot.active ? 1U : 0U,
                singleBlockShadowSnapshot.sourcePC,
                singleBlockShadowSnapshot.sourceLineNumber,
                singleBlockShadowSnapshot.programCommitted ? 1U : 0U,
                singleBlockShadowSnapshot.callbackComplete ? 1U : 0U,
                singleBlockShadowSnapshot.callbackRequired ? 1U : 0U,
                singleBlockShadowSnapshot.transactionComplete ? 1U : 0U,
                singleBlockShadowSnapshot.transactionRequired ? 1U : 0U,
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
                "WaitMotion:%llu Ready:%llu LegacyHold:%llu Agree:%llu "
                "Early:%llu Phantom:%llu Missing:%llu CtrlHold:%llu "
                "CtrlAgree:%llu CtrlEarly:%llu CtrlResume:%llu MotionFail:%llu "
                "Overflow:%llu TxnFail:%llu EndSup:%llu Resume:%llu "
                "Cancel:%llu Supersede:%llu\n",
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
                static_cast<unsigned long long>(singleBlockShadowCounters.agreeHolds),
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
                static_cast<unsigned long long>(singleBlockShadowCounters.resumed),
                static_cast<unsigned long long>(singleBlockShadowCounters.cancelled),
                static_cast<unsigned long long>(singleBlockShadowCounters.superseded));

            RtPrintf(
                "[NC02I-SG] Seq:%llu BSeq:%llu Enabled:%u Phase:%s Decision:%s "
                "Active:%u D:%llu PC:%d Line:%d Kind:%s Match:%u Ready:%u "
                "HoldReady:%u Hold:%u Resume:%u Explicit:%u EndSup:%u "
                "Blocked:%u Cancel:%u\n",
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
                singleBlockGateSnapshot.boundaryReady ? 1U : 0U,
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
                "EndSup:%llu BlockMotion:%llu BlockTxn:%llu BlockOv:%llu "
                "BlockCancel:%llu Cancel:%llu Super:%llu Rollback:%llu\n",
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
                static_cast<unsigned long long>(singleBlockGateCounters.blockedTrackingOverflow),
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
                "PC:%d D:%llu Legacy:%u Ack:%u Stable:%u/%u "
                "OvrX1000:%llu CmdStop:%u ActStop:%u HomePause:%u "
                "CmdMaxPps:%llu ActMaxPps:%llu Q:%u/%u/%u "
                "Owner:%s/%u->%s/%u Epoch:%llu->%llu Fail:%u\n",
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
                static_cast<unsigned int>(feedHoldSnapshot.requiredStableSamples),
                static_cast<unsigned long long>(feedrateOverrideX1000),
                feedHoldSnapshot.motion.commandStopped ? 1U : 0U,
                feedHoldSnapshot.motion.actualStopped ? 1U : 0U,
                feedHoldSnapshot.homePaused ? 1U : 0U,
                static_cast<unsigned long long>(maximumCommandVelocityPps),
                static_cast<unsigned long long>(maximumActualVelocityPps),
                static_cast<unsigned int>(feedHoldSnapshot.motion.commandQueueDepth),
                static_cast<unsigned int>(feedHoldSnapshot.motion.commandIngressDepth),
                static_cast<unsigned int>(feedHoldSnapshot.motion.commandReplayDepth),
                MotionOwnerToDiagnosticName(feedHoldSnapshot.requestOwner),
                static_cast<unsigned int>(feedHoldSnapshot.requestOwnerGeneration),
                MotionOwnerToDiagnosticName(feedHoldSnapshot.currentOwner),
                static_cast<unsigned int>(feedHoldSnapshot.currentOwnerGeneration),
                static_cast<unsigned long long>(feedHoldSnapshot.requestExecutionEpoch),
                static_cast<unsigned long long>(feedHoldSnapshot.currentExecutionEpoch),
                feedHoldSnapshot.failed ? 1U : 0U);

            RtPrintf(
                "[NC02I-FCNT] ReqTry:%llu Req:%llu Prog:%llu Home:%llu Eval:%llu "
                "WaitOvr:%llu WaitCmd:%llu WaitAct:%llu WaitHome:%llu WaitStable:%llu "
                "Ack:%llu Legacy:%llu Early:%llu Agree:%llu "
                "ResumeReq:%llu ResumeEarly:%llu ResumeAck:%llu Resumed:%llu "
                "OwnerFail:%llu EpochFail:%llu MotionFail:%llu AckLost:%llu "
                "Cancel:%llu Super:%llu\n",
                static_cast<unsigned long long>(feedHoldCounters.requestAttempts),
                static_cast<unsigned long long>(feedHoldCounters.requestsLatched),
                static_cast<unsigned long long>(feedHoldCounters.programRequests),
                static_cast<unsigned long long>(feedHoldCounters.homeRequests),
                static_cast<unsigned long long>(feedHoldCounters.evaluations),
                static_cast<unsigned long long>(feedHoldCounters.waitOverrideZero),
                static_cast<unsigned long long>(feedHoldCounters.waitCommandStop),
                static_cast<unsigned long long>(feedHoldCounters.waitActualStop),
                static_cast<unsigned long long>(feedHoldCounters.waitHomePaused),
                static_cast<unsigned long long>(feedHoldCounters.waitStable),
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
                static_cast<unsigned long long>(feedHoldCounters.acknowledgeLost),
                static_cast<unsigned long long>(feedHoldCounters.cancelled),
                static_cast<unsigned long long>(feedHoldCounters.superseded));


            RtPrintf(
                "[NC02I-FG] Seq:%llu BSeq:%llu Enabled:%u Phase:%s Decision:%s "
                "Active:%u Pending:%u Ack:%u Release:%u Applied:%u "
                "Blocked:%u Cancel:%u Src:%s PC:%d D:%llu "
                "Owner:%s/%u Epoch:%llu BActive:%u BFail:%u BCancel:%u\n",
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
                feedHoldGateSnapshot.resumeApplied ? 1U : 0U,
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
                "Applied:%llu BlockFail:%llu BlockCancel:%llu Cancel:%llu "
                "Super:%llu Rollback:%llu\n",
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
                static_cast<unsigned long long>(feedHoldGateCounters.cancelled),
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
                    "Delta:%llu Inv:%llu>%llu Epoch:%llu>%llu Last:%llu "
                    "Need:%u Owner:%s/%u LastOwner:%s/%u Match:%u\n",
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
                        alarmEmergencyStopSnapshot.lastAppliedExecutionEpoch),
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
                    "Error:%08X Cmd0:%08X Seal:%08X PDO:%08X/%08X "
                    "Group:%u AxisSafe:%u CmdAll0:%u SealAll:%u PDOAll0:%u "
                    "Coh:%u ReqSeen:%u RTAck:%u EpochOK:%u FbSync:%u "
                    "Gap:%u Post:%u\n",
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
                        alarmEmergencyStopSnapshot.pdoSampledAxisMask),
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
                    "EDM:%llu Other:%llu Eval:%llu WaitPub:%llu "
                    "WaitReq:%llu WaitRT:%llu WaitOwner:%llu WaitEpoch:%llu "
                    "WaitGroup:%llu WaitAxis:%llu WaitCmd:%llu WaitSeal:%llu "
                    "Ack:%llu PreCorr:%llu PreAck:%llu ClearEarly:%llu "
                    "Gap:%llu Replace:%llu "
                    "AxisScope:%llu Super:%llu\n",
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
                    static_cast<unsigned long long>(alarmEmergencyStopCounters.waitPublication),
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
                    static_cast<unsigned long long>(alarmEmergencyStopCounters.clearedBeforeAcknowledge),
                    static_cast<unsigned long long>(alarmEmergencyStopCounters.lifecycleEvidenceGap),
                    static_cast<unsigned long long>(alarmEmergencyStopCounters.lifecycleReplaced),
                    static_cast<unsigned long long>(alarmEmergencyStopCounters.axisScopeChanged),
                    static_cast<unsigned long long>(alarmEmergencyStopCounters.superseded));
            }

            RtPrintf(
                "[NC02J-INT] Seq:%llu Cause:%s Phase:%s Decision:%s Active:%u "
                "EpochExp:%u Epoch:%llu>%llu>%llu Owner:%s/%u>%s/%u "
                "ExecOwner:%s/%u "
                "D:%llu PC:%d Blocks:%u>%u Term:%s TSeq:%llu Seg:%llu "
                "SrcBlk:%d LedgerOK:%u TermSeen:%u AlarmAck:%u "
                "RTEpoch:%u ExpAbort:%u ExpRetire:%u ClassOK:%u StopClosed:%u "
                "UnexpectedEpoch:%u PostDispatch:%u\n",
                static_cast<unsigned long long>(
                    lifecycleInterruptionSnapshot.sequence),
                NCLifecycleInterruptionCauseToDiagnosticName(
                    lifecycleInterruptionSnapshot.cause),
                NCLifecycleInterruptionPhaseToDiagnosticName(
                    lifecycleInterruptionSnapshot.phase),
                NCLifecycleInterruptionDecisionToDiagnosticName(
                    lifecycleInterruptionSnapshot.decision),
                lifecycleInterruptionSnapshot.active ? 1U : 0U,
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
                    requestExecutionOwnerGeneration),
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
                lifecycleInterruptionSnapshot.alarmStopAcknowledged ? 1U : 0U,
                lifecycleInterruptionSnapshot.runtimeAlarmEpochChangeObserved ? 1U : 0U,
                lifecycleInterruptionSnapshot.expectedAlarmAbortObserved ? 1U : 0U,
                lifecycleInterruptionSnapshot.expectedAlarmPreReadRejectObserved ? 1U : 0U,
                lifecycleInterruptionSnapshot.alarmTerminalClassificationValid ? 1U : 0U,
                lifecycleInterruptionSnapshot.alarmStopClosed ? 1U : 0U,
                lifecycleInterruptionSnapshot.unexpectedEpochChangeObserved ? 1U : 0U,
                lifecycleInterruptionSnapshot.postInterruptionDispatchObserved ? 1U : 0U);

            RtPrintf(
                "[NC02J-DRN] AxisQ:%llu AxisR:%llu CmdQ:%llu In:%llu "
                "Replay:%llu Fb:%llu Notice:%llu FbSeq:%llu/%llu "
                "Safety:%u Cb:%u Bind:%u Stand:%u Stable:%u/%u "
                "Ready:%u Quiet:%u Gap:%u Dispatch:%llu DltFail:%llu "
                "RawFail:%llu ExpAbort:%llu ExpReject:%llu UnexpReject:%llu "
                "ExpOwner:%llu ExpStale:%llu Rej:%llu Cancel:%llu "
                "Abort:%llu Fault:%llu Ledger:%llu "
                "FbOv:%llu NoticeOv:%llu SeqGap:%llu\n",
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
                    lifecycleInterruptionSnapshot.lastConsumedFeedbackSequence),
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
                    lifecycleInterruptionSnapshot.unexpectedBlockFailureDelta),
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
                "Prog:%llu MDI:%llu Manual:%llu Dynamic:%llu Goto:%llu "
                "TermReq:%llu EpochExp:%llu EpochObs:%llu Unexpected:%llu "
                "EpochSuper:%llu AlarmAck:%llu RTEpoch:%llu ExpAbort:%llu "
                "ExpReject:%llu ExpOwner:%llu ExpStale:%llu StopClose:%llu "
                "TRej:%llu TCancel:%llu TAbort:%llu "
                "TFault:%llu LedgerReject:%llu PostDispatch:%llu Eval:%llu Quiet:%llu "
                "Gap:%llu Super:%llu\n",
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
                    lifecycleInterruptionCounters.gotoEpochRequests),
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
                    lifecycleInterruptionCounters.expectedAlarmStaleEpochRejects),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.alarmStopsClosed),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.terminalRejected),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.terminalCancelled),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.terminalAborted),
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
                "ResidualOwner:%llu ResidualStale:%llu ResidualFb:%llu\n",
                static_cast<unsigned long long>(transportRawErrorTotal),
                static_cast<unsigned long long>(transportErrorTotal),
                static_cast<unsigned long long>(ownerConflictReject),
                static_cast<unsigned long long>(staleDiscard),
                static_cast<unsigned long long>(feedbackRejected),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.
                    expectedAlarmOwnerConflictRejects),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.
                    expectedAlarmStaleEpochRejects),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.
                    expectedAlarmPreReadRejects),
                static_cast<unsigned long long>(
                    unexpectedOwnerConflictReject),
                static_cast<unsigned long long>(unexpectedStaleDiscard),
                static_cast<unsigned long long>(unexpectedFeedbackRejected));

            RtPrintf(
                "[NC02J-WAIT] Epoch:%llu Blocks:%llu AxisQ:%llu AxisR:%llu "
                "In:%llu Replay:%llu CmdQ:%llu Notice:%llu Fb:%llu "
                "FbSeq:%llu Cb:%llu Bind:%llu Safety:%llu Stand:%llu "
                "Stable:%llu AlarmAck:%llu AlarmTerm:%llu AlarmStable:%llu "
                "GapFb:%llu GapNotice:%llu GapSeq:%llu "
                "GapLedger:%llu GapReject:%llu\n",
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
                    lifecycleInterruptionCounters.waitFeedback),
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
                    lifecycleInterruptionCounters.feedbackSequenceEvidenceGap),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.ledgerIntegrityEvidenceGap),
                static_cast<unsigned long long>(
                    lifecycleInterruptionCounters.ledgerRejectedEvidenceGap));

            RtPrintf(
                "[NC02J-RG] Seq:%llu BSeq:%llu Phase:%s Decision:%s "
                "Active:%u Epoch:%llu/%llu/%llu Match:%u/%u "
                "Safety:%s/%u Lease:%u/%u BOwner:%s/%u OMatch:%u "
                "BPhase:%s BDecision:%s Stable:%u/%u Stand:%u "
                "Quiet:%u Ready:%u Attempt:%u Applied:%u Block:%u\n",
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
                resetReleaseGateSnapshot.currentEpochMatched ? 1U : 0U,
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
                resetReleaseGateSnapshot.groupStandstill ? 1U : 0U,
                resetReleaseGateSnapshot.quiescenceProved ? 1U : 0U,
                resetReleaseGateSnapshot.releaseReady ? 1U : 0U,
                resetReleaseGateSnapshot.releaseAttempted ? 1U : 0U,
                resetReleaseGateSnapshot.releaseApplied ? 1U : 0U,
                resetReleaseGateSnapshot.blocked ? 1U : 0U);

            RtPrintf(
                "[NC02J-RCNT] ArmTry:%llu Arm:%llu ReArm:%llu "
                "Eval:%llu Wait:%llu Ready:%llu RelTry:%llu Released:%llu "
                "Inv:%llu Seq:%llu Cause:%llu Gap:%llu Super:%llu "
                "Incomplete:%llu Epoch:%llu Lease:%llu ReleaseFail:%llu "
                "PostDispatch:%llu\n",
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
                    resetReleaseGateCounters.released),
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
        }

        if (shouldPrintJ5)
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
                "[NC02J5-RB] Kind:ACK Req:%llu Phase:%s Block:%s "
                "Epoch:%u Owner:%s/%u Mask:%u/%u Accept:%u Apply:%u "
                "Post:%u Ack:%u Acked:%u Blocked:%u Super:%u "
                "FaultEstop:%u Comp:%u\n",
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
                resetRebaseAck.rebaseApplied ? 1U : 0U,
                resetRebaseAck.postVerifyPassed ? 1U : 0U,
                resetRebaseAck.acknowledged ? 1U : 0U,
                resetRebaseAck.acked ? 1U : 0U,
                resetRebaseAck.blocked ? 1U : 0U,
                resetRebaseAck.superseded ? 1U : 0U,
                resetRebaseAck.unsupportedFaultOrEstop ? 1U : 0U,
                resetRebaseAck.compensationBlocked ? 1U : 0U);

            RtPrintf(
                "[NC02J5-RB] Kind:GATE Seq:%llu BSeq:%llu Phase:%s "
                "Decision:%s Active:%u Req:%llu/%llu AckEpoch:%u "
                "AckOwner:%s/%u AckMask:%llu/%llu AckPhase:%s "
                "Obs:%u Match:%u/%u/%u/%u/%u Accept:%u Apply:%u "
                "Post:%u Ack:%u RebaseMatch:%u Release:%u Block:%u "
                "WaitAck:%llu AckFail:%llu RbFail:%llu\n",
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
                    resetReleaseGateSnapshot.ackExecutionEpoch),
                MotionOwnerToDiagnosticName(
                    resetReleaseGateSnapshot.ackOwner),
                static_cast<unsigned int>(
                    resetReleaseGateSnapshot.ackOwnerGeneration),
                static_cast<unsigned long long>(
                    resetReleaseGateSnapshot.ackRequestedAxisMask),
                static_cast<unsigned long long>(
                    resetReleaseGateSnapshot.ackAppliedAxisMask),
                MotionNCResetRebasePhaseToDiagnosticName(
                    resetReleaseGateSnapshot.ackPhase),
                resetReleaseGateSnapshot.ackObserved ? 1U : 0U,
                resetReleaseGateSnapshot.ackSequenceMatched ? 1U : 0U,
                resetReleaseGateSnapshot.ackEpochMatched ? 1U : 0U,
                resetReleaseGateSnapshot.ackOwnerMatched ? 1U : 0U,
                resetReleaseGateSnapshot.ackAxisMaskMatched ? 1U : 0U,
                resetReleaseGateSnapshot.ackPhaseAcknowledged ? 1U : 0U,
                resetReleaseGateSnapshot.ackRequestAccepted ? 1U : 0U,
                resetReleaseGateSnapshot.ackRebaseApplied ? 1U : 0U,
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
        }

        if (startupSamples < 15U)
        {
            ++startupSamples;
        }

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
    }
}
