#pragma once

#include "MotionCore.h"
#include "NCLifecycleInterruptionBoundary.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2J.6.1/J.6.3.2 - Alarm / Emergency-stop RT acknowledgement
// shadow
//
// This observer does not change Alarm, E-stop or Reset behavior.  It correlates
// one Alarm latch with the existing Motion emergency request and the coherent
// 250 us evidence publication.  It answers whether the RT consumer actually
// applied a stop to the same SAFETY lease/Epoch and sealed every existing axis
// against the pre-Alarm target.
//
// PDO TargetVelocity is recorded as advisory evidence only in J.6.1.  The
// current controller intentionally remains position-holding in ESTOP, so this
// shadow must not pretend that a software E-stop is a certified hardwired
// safety function or grant recovery permission.
// =============================================================================

enum class NCAlarmEmergencyStopTrigger : std::uint8_t
{
    NONE = 0,
    EMERGENCY_STOP = 1,
    AXIS_PROTECTION = 2,
    HARD_LIMIT = 3,
    DRIVE_FAULT = 4,
    LAG_ERROR = 5,
    NC_PROGRAM = 6,
    EDM_PROCESS = 7,
    SYSTEM = 8,
    AXIS = 9,
    UNKNOWN = 10
};


enum class NCAlarmEmergencyStopPhase : std::uint8_t
{
    IDLE = 0,
    REQUESTED = 1,
    WAITING_RT = 2,
    ACKNOWLEDGED = 3,
    CLEARED = 4,
    EVIDENCE_GAP = 5,
    SUPERSEDED = 6
};


enum class NCAlarmEmergencyStopDecision : std::uint8_t
{
    NONE = 0,
    ALARM_LATCHED = 1,
    WAIT_PUBLICATION = 2,
    WAIT_REQUEST = 3,
    WAIT_RT_APPLY = 4,
    WAIT_SAFETY_OWNER = 5,
    WAIT_EXECUTION_EPOCH = 6,
    WAIT_GROUP_STOP = 7,
    WAIT_AXIS_SAFE_STATE = 8,
    WAIT_COMMAND_ZERO = 9,
    WAIT_TARGET_SEALED = 10,
    STOP_ACKNOWLEDGED = 11,
    ALARM_CLEARED_BEFORE_ACK = 12,
    EVIDENCE_LIFECYCLE_GAP = 13,
    EVIDENCE_LIFECYCLE_REPLACED = 14,
    EVIDENCE_AXIS_SCOPE_CHANGED = 15,
    SUPERSEDED = 16
};


struct NCAlarmEmergencyStopSample
{
    bool alarmActive = false;
    std::uint32_t alarmUpdateCount = 0U;
    std::int32_t alarmCount = 0;
    std::int32_t alarmCode = 0;
    std::int32_t alarmAxisIndex = -1;

    MotionExecutionEpoch executionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease ownerLease{};

    bool emergencyEvidenceCoherent = false;
    MotionEmergencyStopEvidence emergency{};
    MotionEmergencyStopCounters emergencyCounters{};
    MotionEmergencyStopEpochInvalidationEvidence epochInvalidation{};
    NCLifecycleInterruptionSnapshot lifecycle{};
};


struct NCAlarmEmergencyStopSnapshot
{
    std::uint64_t sequence = 0ULL;
    NCAlarmEmergencyStopTrigger trigger =
        NCAlarmEmergencyStopTrigger::NONE;
    NCAlarmEmergencyStopPhase phase =
        NCAlarmEmergencyStopPhase::IDLE;
    NCAlarmEmergencyStopDecision decision =
        NCAlarmEmergencyStopDecision::NONE;

    std::uint32_t alarmUpdateCount = 0U;
    std::int32_t alarmCount = 0;
    std::int32_t alarmCode = 0;
    std::int32_t alarmAxisIndex = -1;

    std::uint64_t lifecycleSequence = 0ULL;
    std::uint64_t motionPublicationGeneration = 0ULL;
    MotionExecutionEpoch requestExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch currentExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch lastAppliedExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch lastInvalidatedFromExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch lastInvalidatedToExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    std::uint64_t lastInvalidationCount = 0ULL;

    MotionOwner requestOwner = MotionOwner::NONE;
    MotionOwnerGeneration requestOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner currentOwner = MotionOwner::NONE;
    MotionOwnerGeneration currentOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner lastAppliedOwner = MotionOwner::NONE;
    MotionOwnerGeneration lastAppliedOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    std::uint64_t requestPublishedBaseline = 0ULL;
    std::uint64_t requestPublishedCurrent = 0ULL;
    std::uint64_t rtApplyBaseline = 0ULL;
    std::uint64_t rtApplyCurrent = 0ULL;
    std::uint64_t rtApplyDelta = 0ULL;
    std::uint64_t epochInvalidationBaseline = 0ULL;
    std::uint64_t epochInvalidationCurrent = 0ULL;

    std::uint32_t expectedAxisMask = 0U;
    std::uint32_t currentAxisMask = 0U;
    std::uint32_t estopAxisMask = 0U;
    std::uint32_t errorAxisMask = 0U;
    std::uint32_t commandZeroAxisMask = 0U;
    std::uint32_t targetSealedAxisMask = 0U;
    std::uint32_t pdoSampledAxisMask = 0U;
    std::uint32_t pdoZeroAxisMask = 0U;

    std::uint32_t activeBlocks = 0U;
    std::uint64_t commandQueueDepth = 0ULL;
    std::uint64_t commandIngressDepth = 0ULL;
    std::uint64_t commandReplayDepth = 0ULL;
    std::uint64_t feedbackDepth = 0ULL;
    std::uint64_t feedbackNoticeDepth = 0ULL;

    bool active = false;
    bool alarmActive = false;
    bool axisScopeCaptured = false;
    bool hadExecutionToInvalidate = false;
    bool epochChangeRequired = false;
    bool emergencyEvidenceCoherent = false;
    bool emergencyRequestObserved = false;
    bool rtApplyObserved = false;
    bool safetyOwnerMatched = false;
    bool executionEpochMatched = false;
    bool epochInvalidationCorrelated = false;
    bool preLatchedRTApplication = false;
    bool groupStopApplied = false;
    bool allExistingAxesSafe = false;
    bool allExistingAxisCommandsZero = false;
    bool allExistingAxisTargetsSealed = false;
    bool allSampledPdoTargetVelocitiesZero = false;
    bool feedbackSequenceSynchronized = false;
    bool lifecycleEvidenceGap = false;
    bool postAlarmDispatchObserved = false;
    bool acknowledged = false;
    bool clearedBeforeAcknowledge = false;
    bool evidenceGap = false;
    bool superseded = false;
};


struct NCAlarmEmergencyStopCounters
{
    std::uint64_t requestAttempts = 0ULL;
    std::uint64_t requestsLatched = 0ULL;
    std::uint64_t emergencyStopTriggers = 0ULL;
    std::uint64_t axisProtectionTriggers = 0ULL;
    std::uint64_t hardLimitTriggers = 0ULL;
    std::uint64_t driveFaultTriggers = 0ULL;
    std::uint64_t lagErrorTriggers = 0ULL;
    std::uint64_t ncProgramTriggers = 0ULL;
    std::uint64_t edmProcessTriggers = 0ULL;
    std::uint64_t otherTriggers = 0ULL;

    std::uint64_t evaluations = 0ULL;
    std::uint64_t waitPublication = 0ULL;
    std::uint64_t waitRequest = 0ULL;
    std::uint64_t waitRTApply = 0ULL;
    std::uint64_t waitSafetyOwner = 0ULL;
    std::uint64_t waitExecutionEpoch = 0ULL;
    std::uint64_t waitGroupStop = 0ULL;
    std::uint64_t waitAxisSafeState = 0ULL;
    std::uint64_t waitCommandZero = 0ULL;
    std::uint64_t waitTargetSealed = 0ULL;

    std::uint64_t acknowledged = 0ULL;
    std::uint64_t preLatchedCorrelations = 0ULL;
    std::uint64_t preLatchedAcknowledged = 0ULL;
    std::uint64_t clearedBeforeAcknowledge = 0ULL;
    std::uint64_t lifecycleEvidenceGap = 0ULL;
    std::uint64_t lifecycleReplaced = 0ULL;
    std::uint64_t axisScopeChanged = 0ULL;
    std::uint64_t superseded = 0ULL;
};


static_assert(
    std::is_trivially_copyable<NCAlarmEmergencyStopSample>::value,
    "NCAlarmEmergencyStopSample must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCAlarmEmergencyStopSnapshot>::value,
    "NCAlarmEmergencyStopSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCAlarmEmergencyStopCounters>::value,
    "NCAlarmEmergencyStopCounters must remain trivially copyable.");


class NCAlarmEmergencyStopBoundaryShadow
{
public:
    NCAlarmEmergencyStopBoundaryShadow() noexcept = default;

    void Begin(const NCAlarmEmergencyStopSample& sample) noexcept;
    void Observe(const NCAlarmEmergencyStopSample& sample) noexcept;
    void Supersede() noexcept;

    bool IsActive() const noexcept
    {
        return m_snapshot.active;
    }

    NCAlarmEmergencyStopSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCAlarmEmergencyStopCounters GetCounters() const noexcept
    {
        return m_counters;
    }

    static NCAlarmEmergencyStopTrigger ClassifyAlarmCode(
        std::int32_t alarmCode) noexcept;

private:
    std::uint64_t AllocateSequence() noexcept;
    bool HasExactEpochInvalidation(
        const NCAlarmEmergencyStopSample& sample) const noexcept;
    void UpdateSample(const NCAlarmEmergencyStopSample& sample) noexcept;
    void SetWait(NCAlarmEmergencyStopDecision decision) noexcept;
    void MarkEvidenceGap(NCAlarmEmergencyStopDecision decision) noexcept;

    NCAlarmEmergencyStopSnapshot m_snapshot{};
    NCAlarmEmergencyStopCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;
};
