#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// FIX6 temporary unloaded-motor test policy. Set this to 0 for the whole
// project, then Clean/Rebuild, to restore six-axis runtime FOLLOWING trips.
// It does not relax Arm, completion, return-to-anchor, stop or RESET release.
#ifndef EDM94_EMPTY_LOAD_IGNORE_ROTARY_FOLLOWING
#define EDM94_EMPTY_LOAD_IGNORE_ROTARY_FOLLOWING 1
#endif
static_assert(EDM94_EMPTY_LOAD_IGNORE_ROTARY_FOLLOWING == 0 ||
    EDM94_EMPTY_LOAD_IGNORE_ROTARY_FOLLOWING == 1, "EDM94 following policy must be 0 or 1");

// EDM94: a bounded scope around authored ordinary XYZCUV G01 commands.
// This guard creates no reference, owner, coordinate, PID reset or PDO write.
// The Motion owner supplies coherent facts; the send owner supplies exact
// successful serialized-image receipts. Neither an accepted command nor a
// numerical zero is physical stop evidence.
namespace EDM94
{
    static constexpr unsigned AxisCount = 6U;
    static constexpr std::uint32_t AllAxes = 63U;
    static constexpr bool IgnoreRotaryFollowingTripForEmptyLoad =
        EDM94_EMPTY_LOAD_IGNORE_ROTARY_FOLLOWING != 0;
    static constexpr std::uint32_t FollowingTripAxisMask =
        IgnoreRotaryFollowingTripForEmptyLoad ? 0x07U : AllAxes;
    static constexpr std::uint32_t FollowingObserveOnlyAxisMask =
        AllAxes & ~FollowingTripAxisMask;
    static constexpr std::uint32_t RequiredStableCycles = 200U;
    // Admission at a completed endpoint, not the zero-output physical stop proof.
    static constexpr std::uint32_t BoundaryReadyCycles = 200U;
    static constexpr std::uint64_t MaximumDurationUs = 120000000ULL;
    // Only the explicitly bound five-millimetre commissioning profile uses
    // this longer window. Ordinary GAP/motion retains the original deadline.
    static constexpr double VisibleFlushReferenceMmSec = 0.065;
    static constexpr double VisibleXYZMachiningReferenceMmSec = 0.002;
    static constexpr double VisibleSixRotaryReferenceNativeSec = 0.1;
    static constexpr double VisibleFlushAdmissionSeconds = 175.0;
    static constexpr std::uint64_t VisibleFlushMaximumDurationUs = 180000000ULL;
    // Explicit frozen identity, not the duration, selects an extended lift.
    inline double VisibleFlushHeightMm(unsigned profile) noexcept
    { return profile == 1U ? 5.0 : (profile == 2U || profile == 3U || profile == 4U) ? 2.0 : 0.0; }
    inline std::uint64_t VisibleFlushDurationUs(unsigned profile) noexcept
    { return profile == 1U ? VisibleFlushMaximumDurationUs : (profile == 2U || profile == 3U || profile == 4U) ? MaximumDurationUs : 0ULL; }
    inline unsigned VisibleFlushRequiredMask(unsigned profile) noexcept
    { return profile == 1U ? 4U : profile == 2U ? 7U : profile == 3U ? AllAxes : profile == 4U ? 7U : 0U; }

    enum class State : std::uint8_t
    { Idle, Armed, Running, Finishing, Zeroing, Stopped, Cancelled, Faulted, Released };
    enum class Reason : std::uint8_t
    {
        None, Configuration, Identity, Clock, Source, ConfigurationChanged,
        Following, Excursion, Output, Serialized, Send, Cancelled, Duration,
        Release, MotionNotObserved
    };

    struct Identity
    {
        std::uint64_t session = 0ULL;
        std::uint64_t epoch = 0ULL;
        std::uint32_t ownerGeneration = 0U;
    };
    inline bool SameIdentity(const Identity& a, const Identity& b) noexcept
    { return a.session == b.session && a.epoch == b.epoch && a.ownerGeneration == b.ownerGeneration; }
    inline bool ValidIdentity(const Identity& value) noexcept
    { return value.session != 0ULL && value.epoch != 0ULL && value.ownerGeneration != 0U; }

    // FIX7: bounded post-PI correction headroom for unloaded-motor diagnosis.
    // Authored G1 feed, excursion and following/stop windows remain separate.
    inline double NativeVelocityLimit(unsigned axis) noexcept { return axis < 3U ? 0.1 : 3.0; }
    inline double NativeExcursionLimit(unsigned axis) noexcept { return axis < 3U ? 0.025 : 0.5; }

    // FIX10: retain the previous I-term (pulse/s) when a completed EDM94
    // endpoint selects configured IDLE Ki. integralAcc remains pulse-seconds.
    // The caller owns the exact endpoint/authority fence; this grants no output.
    inline bool TransferEndpointIntegral(double oldKi, double newKi,
        double integralBefore, double maxIntegral, double& transferred) noexcept
    {
        if (!std::isfinite(oldKi) || oldKi < 0.0 || !std::isfinite(newKi) || newKi < 0.0 ||
            !std::isfinite(integralBefore) || !std::isfinite(maxIntegral) || maxIntegral < 0.0)
            return false;
        if (oldKi == newKi) { transferred = integralBefore; return true; }
        double value = 0.0;
        if (oldKi != 0.0 && newKi != 0.0)
        {
            const double previousOutput = oldKi * integralBefore;
            if (!std::isfinite(previousOutput)) return false;
            value = previousOutput / newKi;
            if (!std::isfinite(value)) return false;
        }
        if (value > maxIntegral) value = maxIntegral;
        if (value < -maxIntegral) value = -maxIntegral;
        transferred = value;
        return true;
    }

    // FIX2: immutable attempted-arm observations. These diagnostics grant no
    // authority, output receipt, movement evidence or physical-stop credit.
    enum AdmissionGate : std::uint32_t
    {
        AdmissionGate_PDO = 1U << 0U,
        AdmissionGate_FRESH = 1U << 1U,
        AdmissionGate_AUTHORITY = 1U << 2U,
        AdmissionGate_HEALTH = 1U << 3U,
        AdmissionGate_GROUP_IDLE = 1U << 4U,
        AdmissionGate_COMMAND_ZERO = 1U << 5U,
        AdmissionGate_CLOCK = 1U << 6U,
        AdmissionGate_IDENTITY = 1U << 7U,
        AdmissionGate_CONFIGURATION = 1U << 8U
    };
    inline const char* AdmissionGateName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionGate_PDO) return "PDO";
        if (failures & AdmissionGate_FRESH) return "FRESH";
        if (failures & AdmissionGate_AUTHORITY) return "AUTHORITY";
        if (failures & AdmissionGate_HEALTH) return "HEALTH";
        if (failures & AdmissionGate_GROUP_IDLE) return "GROUP_IDLE";
        if (failures & AdmissionGate_COMMAND_ZERO) return "COMMAND_ZERO";
        if (failures & AdmissionGate_CLOCK) return "CLOCK";
        if (failures & AdmissionGate_IDENTITY) return "IDENTITY";
        if (failures & AdmissionGate_CONFIGURATION) return "CONFIGURATION";
        return failures ? "UNKNOWN" : "NONE";
    }
    enum AdmissionAuthority : std::uint32_t
    {
        AdmissionAuthority_OWNER = 1U << 0U,
        AdmissionAuthority_HANDSHAKE = 1U << 1U,
        AdmissionAuthority_SAFETY_ACTION = 1U << 2U,
        AdmissionAuthority_OUTPUT_RESERVED = 1U << 3U,
        AdmissionAuthority_EPOCH_PENDING = 1U << 4U,
        AdmissionAuthority_EPOCH_RESERVED = 1U << 5U,
        AdmissionAuthority_COMMAND_SOURCE = 1U << 6U,
        AdmissionAuthority_PENDING_SAFETY = 1U << 7U,
        AdmissionAuthority_UNACKNOWLEDGED_SAFETY = 1U << 8U,
        AdmissionAuthority_ALARM = 1U << 9U,
        AdmissionAuthority_GROUP_AUTHORIZATION = 1U << 10U,
        AdmissionAuthority_OWNER_CHANGED = 1U << 11U,
        AdmissionAuthority_EPOCH_CHANGED = 1U << 12U,
        AdmissionAuthority_ZERO_IDENTITY = 1U << 13U
    };
    inline const char* AdmissionAuthorityName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionAuthority_OWNER) return "OWNER";
        if (failures & AdmissionAuthority_HANDSHAKE) return "HANDSHAKE";
        if (failures & AdmissionAuthority_SAFETY_ACTION) return "SAFETY_ACTION";
        if (failures & AdmissionAuthority_OUTPUT_RESERVED) return "OUTPUT_RESERVED";
        if (failures & AdmissionAuthority_EPOCH_PENDING) return "EPOCH_PENDING";
        if (failures & AdmissionAuthority_EPOCH_RESERVED) return "EPOCH_RESERVED";
        if (failures & AdmissionAuthority_COMMAND_SOURCE) return "COMMAND_SOURCE";
        if (failures & AdmissionAuthority_PENDING_SAFETY) return "PENDING_SAFETY";
        if (failures & AdmissionAuthority_UNACKNOWLEDGED_SAFETY) return "UNACKNOWLEDGED_SAFETY";
        if (failures & AdmissionAuthority_ALARM) return "ALARM";
        if (failures & AdmissionAuthority_GROUP_AUTHORIZATION) return "GROUP_AUTHORIZATION";
        if (failures & AdmissionAuthority_OWNER_CHANGED) return "OWNER_CHANGED";
        if (failures & AdmissionAuthority_EPOCH_CHANGED) return "EPOCH_CHANGED";
        if (failures & AdmissionAuthority_ZERO_IDENTITY) return "ZERO_IDENTITY";
        return failures ? "UNKNOWN" : "NONE";
    }
    enum AdmissionSharedHealth : std::uint32_t
    {
        AdmissionSharedHealth_MAPPING = 1U << 0U,
        AdmissionSharedHealth_Z_FIXTURE = 1U << 1U,
        AdmissionSharedHealth_GAP_PHYSICAL = 1U << 2U,
        AdmissionSharedHealth_PHYSICAL_MOTION = 1U << 3U,
        AdmissionSharedHealth_COORDINATED_SESSION = 1U << 4U,
        AdmissionSharedHealth_SERIALIZED_MAPPING = 1U << 5U,
        AdmissionSharedHealth_GROUP_GEOMETRY = 1U << 6U
    };
    inline const char* AdmissionSharedHealthName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionSharedHealth_MAPPING) return "MAPPING";
        if (failures & AdmissionSharedHealth_Z_FIXTURE) return "Z_FIXTURE";
        if (failures & AdmissionSharedHealth_GAP_PHYSICAL) return "GAP_PHYSICAL";
        if (failures & AdmissionSharedHealth_PHYSICAL_MOTION) return "PHYSICAL_MOTION";
        if (failures & AdmissionSharedHealth_COORDINATED_SESSION) return "COORDINATED_SESSION";
        if (failures & AdmissionSharedHealth_SERIALIZED_MAPPING) return "SERIALIZED_MAPPING";
        if (failures & AdmissionSharedHealth_GROUP_GEOMETRY) return "GROUP_GEOMETRY";
        return failures ? "UNKNOWN" : "NONE";
    }
    enum AdmissionGroup : std::uint32_t
    {
        AdmissionGroup_ACTIVE = 1U << 0U,
        AdmissionGroup_QUEUE = 1U << 1U,
        AdmissionGroup_AXIS_MAILBOX = 1U << 2U,
        AdmissionGroup_HOLD_SOURCE = 1U << 3U,
        AdmissionGroup_HOLD_START = 1U << 4U,
        AdmissionGroup_HOLD_ADMISSION = 1U << 5U,
        AdmissionGroup_AXIS_NOT_IDLE = 1U << 6U
    };
    inline const char* AdmissionGroupName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionGroup_ACTIVE) return "ACTIVE";
        if (failures & AdmissionGroup_QUEUE) return "QUEUE";
        if (failures & AdmissionGroup_AXIS_MAILBOX) return "AXIS_MAILBOX";
        if (failures & AdmissionGroup_HOLD_SOURCE) return "HOLD_SOURCE";
        if (failures & AdmissionGroup_HOLD_START) return "HOLD_START";
        if (failures & AdmissionGroup_HOLD_ADMISSION) return "HOLD_ADMISSION";
        if (failures & AdmissionGroup_AXIS_NOT_IDLE) return "AXIS_NOT_IDLE";
        return failures ? "UNKNOWN" : "NONE";
    }
    enum AdmissionSource : std::uint32_t
    {
        AdmissionSource_MAPPING = 1U << 0U,
        AdmissionSource_INCOMPLETE = 1U << 1U,
        AdmissionSource_UNOBSERVED = 1U << 2U,
        AdmissionSource_INVALID_PDO = 1U << 3U,
        AdmissionSource_NONCONTIGUOUS = 1U << 4U,
        AdmissionSource_MISSING_AXIS = 1U << 5U
    };
    inline const char* AdmissionSourceName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionSource_MAPPING) return "MAPPING";
        if (failures & AdmissionSource_INCOMPLETE) return "INCOMPLETE";
        if (failures & AdmissionSource_UNOBSERVED) return "UNOBSERVED";
        if (failures & AdmissionSource_INVALID_PDO) return "INVALID_PDO";
        if (failures & AdmissionSource_NONCONTIGUOUS) return "NONCONTIGUOUS";
        if (failures & AdmissionSource_MISSING_AXIS) return "MISSING_AXIS";
        return failures ? "UNKNOWN" : "NONE";
    }
    enum AdmissionRequest : std::uint32_t
    {
        AdmissionRequest_SESSION_REUSED = 1U << 0U,
        AdmissionRequest_EPOCH_CHANGED = 1U << 1U,
        AdmissionRequest_LEASE_CHANGED = 1U << 2U,
        AdmissionRequest_CANCELLED = 1U << 3U
    };
    inline const char* AdmissionRequestName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionRequest_SESSION_REUSED) return "SESSION_REUSED";
        if (failures & AdmissionRequest_EPOCH_CHANGED) return "EPOCH_CHANGED";
        if (failures & AdmissionRequest_LEASE_CHANGED) return "LEASE_CHANGED";
        if (failures & AdmissionRequest_CANCELLED) return "CANCELLED";
        return failures ? "UNKNOWN" : "NONE";
    }
    enum AdmissionAxisHealth : std::uint32_t
    {
        AdmissionAxisHealth_INDEX = 1U << 0U,
        AdmissionAxisHealth_EXISTENCE = 1U << 1U,
        AdmissionAxisHealth_TYPE = 1U << 2U,
        AdmissionAxisHealth_FEEDBACK = 1U << 3U,
        AdmissionAxisHealth_STARTUP_LAG = 1U << 4U,
        AdmissionAxisHealth_FAULT = 1U << 5U,
        AdmissionAxisHealth_HOME_ACTIVE = 1U << 6U,
        AdmissionAxisHealth_STATE = 1U << 7U,
        AdmissionAxisHealth_TRAVEL_BLOCK = 1U << 8U,
        AdmissionAxisHealth_COMPENSATION = 1U << 9U,
        AdmissionAxisHealth_STATUS_WORD = 1U << 10U,
        AdmissionAxisHealth_MODE = 1U << 11U,
        AdmissionAxisHealth_PULSE_CONVERSION = 1U << 12U,
        AdmissionAxisHealth_POSITION_WINDOW = 1U << 13U,
        AdmissionAxisHealth_MAX_LAG = 1U << 14U,
        AdmissionAxisHealth_LAG_DISABLED = 1U << 15U,
        AdmissionAxisHealth_MAX_VELOCITY = 1U << 16U,
        AdmissionAxisHealth_G00_GAIN_FINITE = 1U << 17U,
        AdmissionAxisHealth_IDLE_GAIN_FINITE = 1U << 18U,
        AdmissionAxisHealth_G00_GAIN_SIGN = 1U << 19U,
        AdmissionAxisHealth_IDLE_GAIN_SIGN = 1U << 20U,
        AdmissionAxisHealth_MAX_INTEGRAL = 1U << 21U,
        AdmissionAxisHealth_TRAVEL_RANGE = 1U << 22U,
        AdmissionAxisHealth_WRITABLE = 1U << 23U,
        AdmissionAxisHealth_FLUSH_GAIN_FINITE = 1U << 24U,
        AdmissionAxisHealth_FLUSH_GAIN_SIGN = 1U << 25U
    };
    inline const char* AdmissionAxisHealthName(std::uint32_t failures) noexcept
    {
        if (failures & AdmissionAxisHealth_INDEX) return "INDEX";
        if (failures & AdmissionAxisHealth_EXISTENCE) return "EXISTENCE";
        if (failures & AdmissionAxisHealth_TYPE) return "TYPE";
        if (failures & AdmissionAxisHealth_FEEDBACK) return "FEEDBACK";
        if (failures & AdmissionAxisHealth_STARTUP_LAG) return "STARTUP_LAG";
        if (failures & AdmissionAxisHealth_FAULT) return "FAULT";
        if (failures & AdmissionAxisHealth_HOME_ACTIVE) return "HOME_ACTIVE";
        if (failures & AdmissionAxisHealth_STATE) return "STATE";
        if (failures & AdmissionAxisHealth_TRAVEL_BLOCK) return "TRAVEL_BLOCK";
        if (failures & AdmissionAxisHealth_COMPENSATION) return "COMPENSATION";
        if (failures & AdmissionAxisHealth_STATUS_WORD) return "STATUS_WORD";
        if (failures & AdmissionAxisHealth_MODE) return "MODE";
        if (failures & AdmissionAxisHealth_PULSE_CONVERSION) return "PULSE_CONVERSION";
        if (failures & AdmissionAxisHealth_POSITION_WINDOW) return "POSITION_WINDOW";
        if (failures & AdmissionAxisHealth_MAX_LAG) return "MAX_LAG";
        if (failures & AdmissionAxisHealth_LAG_DISABLED) return "LAG_DISABLED";
        if (failures & AdmissionAxisHealth_MAX_VELOCITY) return "MAX_VELOCITY";
        if (failures & AdmissionAxisHealth_G00_GAIN_FINITE) return "G00_GAIN_FINITE";
        if (failures & AdmissionAxisHealth_IDLE_GAIN_FINITE) return "IDLE_GAIN_FINITE";
        if (failures & AdmissionAxisHealth_G00_GAIN_SIGN) return "G00_GAIN_SIGN";
        if (failures & AdmissionAxisHealth_IDLE_GAIN_SIGN) return "IDLE_GAIN_SIGN";
        if (failures & AdmissionAxisHealth_MAX_INTEGRAL) return "MAX_INTEGRAL";
        if (failures & AdmissionAxisHealth_TRAVEL_RANGE) return "TRAVEL_RANGE";
        if (failures & AdmissionAxisHealth_WRITABLE) return "WRITABLE";
        if (failures & AdmissionAxisHealth_FLUSH_GAIN_FINITE) return "FLUSH_GAIN_FINITE";
        if (failures & AdmissionAxisHealth_FLUSH_GAIN_SIGN) return "FLUSH_GAIN_SIGN";
        return failures ? "UNKNOWN" : "NONE";
    }
    struct AxisAdmissionDiagnostic
    {
        double commandVelocity = 0.0;
        double logicalVelocity = 0.0;
        double targetVelocity = 0.0;
        double followingError = 0.0;
        std::uint32_t healthFailures = 0U;
        std::int32_t mode = 0;
        std::int16_t state = 0;
        std::uint16_t statusWord = 0U;
    };
    struct AdmissionDiagnostic
    {
        std::uint64_t tick = 0ULL;
        std::uint64_t nowUs = 0ULL;
        std::uint32_t failedGates = 0U;
        std::uint32_t authorityFailures = 0U;
        std::uint32_t sharedHealthFailures = 0U;
        std::uint32_t groupFailures = 0U;
        std::uint32_t sourceFailures = 0U;
        std::uint32_t requestFailures = 0U;
        std::uint32_t inputMask = 0U;
        std::int32_t pendingSource = 0;
        AxisAdmissionDiagnostic axis[AxisCount]{};
    };
    static_assert(std::is_trivially_copyable<AdmissionDiagnostic>::value, "Admission diagnostic must be POD.");
    static_assert(sizeof(AdmissionDiagnostic) <= 336U, "Admission diagnostic storage budget.");

    // FIX3: observations only. No diagnostic field grants execution or stop authority.
    enum class FaultSite : std::uint32_t
    {
        None, EngineFault, TickIdentity, TickAuthority,
        BeginFrameIdentity, BeginProofIdentity, SealGenericScrub,
        SealFrameIdentity, SealProposalIdentity, SealAuthorityIdentity,
        TickClock, HandoffClock, HandoffDeadline, SendClock, SessionClock
    };
    inline const char* FaultSiteName(std::uint32_t site) noexcept
    {
        switch (static_cast<FaultSite>(site))
        {
        case FaultSite::None: return "NONE";
        case FaultSite::EngineFault: return "ENGINE_FAULT";
        case FaultSite::TickIdentity: return "TICK_IDENTITY";
        case FaultSite::TickAuthority: return "TICK_AUTHORITY";
        case FaultSite::BeginFrameIdentity: return "BEGIN_FRAME_IDENTITY";
        case FaultSite::BeginProofIdentity: return "BEGIN_PROOF_IDENTITY";
        case FaultSite::SealGenericScrub: return "SEAL_GENERIC_SCRUB";
        case FaultSite::SealFrameIdentity: return "SEAL_FRAME_IDENTITY";
        case FaultSite::SealProposalIdentity: return "SEAL_PROPOSAL_IDENTITY";
        case FaultSite::SealAuthorityIdentity: return "SEAL_AUTHORITY_IDENTITY";
        case FaultSite::TickClock: return "TICK_CLOCK";
        case FaultSite::HandoffClock: return "HANDOFF_CLOCK";
        case FaultSite::HandoffDeadline: return "HANDOFF_DEADLINE";
        case FaultSite::SendClock: return "SEND_CLOCK";
        case FaultSite::SessionClock: return "SESSION_CLOCK";
        default: return "UNKNOWN";
        }
    }
    enum RuntimeFlag : std::uint32_t
    {
        RuntimeFlag_SNAPSHOT_VALID = 1U << 0U,
        RuntimeFlag_SEND_CONTEXT_VALID = 1U << 1U,
        RuntimeFlag_SCRUBBED = 1U << 2U,
        RuntimeFlag_ACQUIRED = 1U << 3U,
        RuntimeFlag_FINAL_CHECKED = 1U << 4U,
        RuntimeFlag_NORMAL_ELIGIBLE = 1U << 5U,
        RuntimeFlag_FORCE_ZERO_BEFORE_FAULT = 1U << 6U,
        RuntimeFlag_PROOF_CONTEXT_VALID = 1U << 7U
    };
    enum StopBlock : std::uint32_t
    {
        StopBlock_CLOCK = 1U << 0U, StopBlock_SOURCE = 1U << 1U,
        StopBlock_IDENTITY = 1U << 2U, StopBlock_AUTHORITY = 1U << 3U,
        StopBlock_CONFIGURATION = 1U << 4U, StopBlock_HEALTH = 1U << 5U,
        StopBlock_GROUP_ACTIVE = 1U << 6U, StopBlock_COMMAND_NONZERO = 1U << 7U,
        StopBlock_ENCODER_RANGE = 1U << 8U, StopBlock_SEND = 1U << 9U,
        StopBlock_NOT_ZEROING = 1U << 10U, StopBlock_FOLLOWING_OR_ENVELOPE = 1U << 11U
    };
    inline const char* StopBlockName(std::uint32_t mask) noexcept
    {
        if (mask & StopBlock_CLOCK) return "CLOCK";
        if (mask & StopBlock_SOURCE) return "SOURCE";
        if (mask & StopBlock_IDENTITY) return "IDENTITY";
        if (mask & StopBlock_AUTHORITY) return "AUTHORITY";
        if (mask & StopBlock_CONFIGURATION) return "CONFIGURATION";
        if (mask & StopBlock_HEALTH) return "HEALTH";
        if (mask & StopBlock_GROUP_ACTIVE) return "GROUP_ACTIVE";
        if (mask & StopBlock_COMMAND_NONZERO) return "COMMAND_NONZERO";
        if (mask & StopBlock_ENCODER_RANGE) return "ENCODER_RANGE";
        if (mask & StopBlock_SEND) return "SEND";
        if (mask & StopBlock_NOT_ZEROING) return "NOT_ZEROING";
        if (mask & StopBlock_FOLLOWING_OR_ENVELOPE) return "FOLLOWING_OR_ENVELOPE";
        return mask ? "UNKNOWN" : "NONE";
    }
    enum ReleaseBlock : std::uint32_t
    {
        ReleaseBlock_NOT_EVALUATED = 1U << 0U,
        ReleaseBlock_RESET_PRECHECK = 1U << 1U,
        ReleaseBlock_FRAME_CHANGED = 1U << 2U,
        ReleaseBlock_STOP_NOT_PROVEN = 1U << 3U,
        ReleaseBlock_FINAL_PROOF = 1U << 4U
    };
    struct RuntimeDiagnostic
    {
        Identity frameIdentity{};
        std::uint64_t tick = 0ULL, nowUs = 0ULL;
        std::uint64_t proofTick = 0ULL, proofGeneration = 0ULL;
        std::uint64_t liveOwnerState = 0ULL, liveExecutionPublication = 0ULL;
        std::uint64_t proofOwnerState = 0ULL, proofExecutionPublication = 0ULL;
        std::uint64_t reservationOwnerState = 0ULL, reservationExecutionPublication = 0ULL;
        std::uint32_t site = 0U, reason = 0U, flags = 0U;
        std::uint32_t authorityFailures = 0U, sharedHealthFailures = 0U;
        std::uint32_t sourceFailures = 0U, groupFailures = 0U, inputMask = 0U;
        std::uint32_t configChangedMask = 0U, mappingChangedMask = 0U;
        std::uint32_t healthFailures[AxisCount]{};
    };
    struct StopDiagnostic
    {
        std::uint64_t sampledTick = 0ULL, lastRangeResetTick = 0ULL;
        // Preserved failed range BEFORE resetting the dwell; this is not
        // the latest encoder position or a stop-completion measurement.
        double rangePulse[AxisCount]{};
        std::uint32_t blockedMask = 0U, rangeExceededMask = 0U, rangeResetCount = 0U;
        std::uint32_t releaseBlockedMask = ReleaseBlock_NOT_EVALUATED;
        std::uint32_t configChangedMask = 0U, mappingChangedMask = 0U, unhealthyMask = 0U;
        std::uint32_t authorityFailures = 0U, sharedHealthFailures = 0U;
        std::uint32_t sourceFailures = 0U, groupFailures = 0U;
    };
    static_assert(sizeof(RuntimeDiagnostic) + sizeof(StopDiagnostic) <= 300U, "FIX3 diagnostic budget.");

    // FIX2 observation only. The controller sample is a local, pre-guard
    // proposal; the separately dated accepted image is not a drive ACK.
    struct AxisControlSample
    {
        Identity identity{};
        std::uint64_t tick = 0ULL, nowUs = 0ULL;
        double commandPulse = 0.0, actualPulse = 0.0, referenceVelocityPps = 0.0;
        double kp = 0.0, ki = 0.0, kvff = 0.0;
        // Candidate integral after Evaluate; a later output clamp may roll it back.
        double integralBefore = 0.0, integralAfter = 0.0;
        double pTerm = 0.0, iTerm = 0.0, logicalOutputVelocityPps = 0.0;
    };
    enum FaultAxisFlag : std::uint32_t
    {
        FaultAxis_FRAME = 1U, FaultAxis_CONTROL = 2U,
        FaultAxis_ACCEPTED_IMAGE = 4U, FaultAxis_CONTROL_PRIOR = 8U
    };
    struct FaultAxisDiagnostic
    {
        AxisControlSample control{};
        Identity acceptedIdentity{};
        std::uint64_t acceptedTick = 0ULL, acceptedSourceNowUs = 0ULL;
        double commandPulse = 0.0, actualPulse = 0.0, followingErrorPulse = 0.0;
        double windowPulse = 0.0, pulsePerUnit = 0.0;
        std::int32_t acceptedCsv = 0, hardwareSign = 0;
        std::uint32_t flags = 0U, axis = AxisCount;
    };

    struct AxisConfig
    {
        double pulsePerUnit = 0.0;
        double commandAnchor = 0.0;
        double actualAnchor = 0.0;
        double followingWindowPulse = 0.0;
        std::uint64_t configFingerprint = 0ULL;
        std::uint64_t mappingFingerprint = 0ULL;
        std::int32_t hardwareSign = 0;
    };
    // Budget the configured PI's complete frozen following window and
    // integral bound before admitting the visible profile. This grants no
    // output: the existing serialized cap and following trip remain active.
    inline bool VisibleServoHeadroomFits(double pulsePerUnit, double serializedCapPps,
        double followingWindowPulse, double kp, double ki, double kvff, double maxIntegral,
        double referenceMmSec = VisibleFlushReferenceMmSec) noexcept
    {
        if (!std::isfinite(pulsePerUnit) || pulsePerUnit <= 0.0 ||
            !std::isfinite(serializedCapPps) || serializedCapPps < 1.0 ||
            serializedCapPps > static_cast<double>((std::numeric_limits<std::int32_t>::max)()) ||
            std::floor(serializedCapPps) != serializedCapPps ||
            !std::isfinite(followingWindowPulse) || followingWindowPulse <= 0.0 ||
            !std::isfinite(kp) || kp < 0.0 || !std::isfinite(ki) || ki < 0.0 ||
            !std::isfinite(kvff) || kvff < 0.0 ||
            !std::isfinite(maxIntegral) || maxIntegral < 0.0 ||
            !std::isfinite(referenceMmSec) || referenceMmSec <= 0.0 ||
            referenceMmSec > VisibleFlushReferenceMmSec) return false;
        const double nativeCap = pulsePerUnit * NativeVelocityLimit(2U);
        const double feedforward = referenceMmSec * pulsePerUnit * kvff;
        const double proportional = kp * followingWindowPulse;
        const double integral = ki * maxIntegral;
        const double reserve = 0.004 * pulsePerUnit;
        const double required = feedforward + proportional + integral;
        const double available = serializedCapPps - reserve;
        return std::isfinite(nativeCap) && serializedCapPps <= nativeCap &&
            std::isfinite(feedforward) && std::isfinite(proportional) && std::isfinite(integral) &&
            std::isfinite(reserve) && std::isfinite(required) && std::isfinite(available) &&
            available > 0.0 && required <= available;
    }

    // Only the bound two-mm XYZ profile's machining X/Y axes use the
    // configured CNC PI with its existing post-PI cap and I rollback.
    // The complete P/I bound must be arithmetically finite; it is not an
    // assertion that every allowed following error avoids saturation.
    inline bool BoundedXYMachiningServoFits(double pulsePerUnit, double serializedCapPps,
        double followingWindowPulse, double kp, double ki, double kd,
        double kvff, double maxIntegral) noexcept
    {
        if (!std::isfinite(pulsePerUnit) || pulsePerUnit <= 0.0 ||
            !std::isfinite(serializedCapPps) || serializedCapPps < 1.0 ||
            serializedCapPps > static_cast<double>((std::numeric_limits<std::int32_t>::max)()) ||
            std::floor(serializedCapPps) != serializedCapPps ||
            !std::isfinite(followingWindowPulse) || followingWindowPulse <= 0.0 ||
            !std::isfinite(kp) || kp <= 0.0 || !std::isfinite(ki) || ki < 0.0 ||
            !std::isfinite(kd) || kd < 0.0 || !std::isfinite(kvff) || kvff < 0.0 ||
            !std::isfinite(maxIntegral) || maxIntegral < 0.0) return false;
        const double nativeCap = pulsePerUnit * NativeVelocityLimit(2U);
        const double nativeWindow = 0.001 * pulsePerUnit;
        const double feedforward = VisibleXYZMachiningReferenceMmSec * pulsePerUnit * kvff;
        const double proportional = kp * followingWindowPulse;
        const double integral = ki * maxIntegral;
        const double worstSum = feedforward + proportional + integral;
        const double reserve = 0.004 * pulsePerUnit;
        const double available = serializedCapPps - reserve;
        return std::isfinite(nativeCap) && serializedCapPps <= nativeCap &&
            std::isfinite(nativeWindow) && followingWindowPulse <= nativeWindow &&
            std::isfinite(feedforward) && std::isfinite(proportional) && std::isfinite(integral) &&
            std::isfinite(worstSum) && std::isfinite(reserve) && std::isfinite(available) &&
            available > 0.0 && feedforward <= available;
    }

    // The explicit six-axis two-mm profile keeps the ordinary rotary CSV
    // cap and post-PI rollback. Its fixed XYZCUV line is below 0.1 native
    // units/s on each rotary component; C/U/V stay fixed during the B1 lift.
    // Finite PI bounds validate configuration without assuming an unloaded
    // rotary following window can always avoid saturation.
    inline bool BoundedRotaryMachiningServoFits(unsigned axis, double pulsePerUnit,
        double serializedCapPps, double followingWindowPulse, double kp, double ki,
        double kd, double kvff, double maxIntegral) noexcept
    {
        if (axis < 3U || axis >= AxisCount ||
            !std::isfinite(pulsePerUnit) || pulsePerUnit <= 0.0 ||
            !std::isfinite(serializedCapPps) || serializedCapPps < 1.0 ||
            serializedCapPps > static_cast<double>((std::numeric_limits<std::int32_t>::max)()) ||
            std::floor(serializedCapPps) != serializedCapPps ||
            !std::isfinite(followingWindowPulse) || followingWindowPulse <= 0.0 ||
            !std::isfinite(kp) || kp <= 0.0 || !std::isfinite(ki) || ki < 0.0 ||
            !std::isfinite(kd) || kd < 0.0 || !std::isfinite(kvff) || kvff < 0.0 ||
            !std::isfinite(maxIntegral) || maxIntegral < 0.0) return false;
        const double nativeCap = pulsePerUnit * NativeVelocityLimit(axis);
        const double nativeWindow = 0.005 * pulsePerUnit;
        const double feedforward = VisibleSixRotaryReferenceNativeSec * pulsePerUnit * kvff;
        const double proportional = kp * followingWindowPulse;
        const double integral = ki * maxIntegral;
        const double worstSum = feedforward + proportional + integral;
        const double reserve = 0.004 * pulsePerUnit;
        const double available = serializedCapPps - reserve;
        return std::isfinite(nativeCap) && serializedCapPps <= nativeCap &&
            std::isfinite(nativeWindow) && followingWindowPulse <= nativeWindow &&
            std::isfinite(feedforward) && std::isfinite(proportional) && std::isfinite(integral) &&
            std::isfinite(worstSum) && std::isfinite(reserve) && std::isfinite(available) &&
            available > 0.0 && feedforward <= available;
    }

    // FIX12: reuse the formal NC settle range policy on the tighter frozen
    // EDM94 following window. This is bounded encoder motion over the dwell,
    // not proof of mathematically zero velocity or a following-trip bypass.
    inline double StopRangeLimitPulse(unsigned axis, const AxisConfig& config) noexcept
    {
        if (axis >= AxisCount || !std::isfinite(config.pulsePerUnit) || config.pulsePerUnit <= 0.0 ||
            !std::isfinite(config.followingWindowPulse) || config.followingWindowPulse <= 0.0)
            return 0.0;
        const double nativeWindow = (axis < 3U ? 0.001 : 0.005) * config.pulsePerUnit;
        if (!std::isfinite(nativeWindow) || nativeWindow <= 0.0) return 0.0;
        const double window = config.followingWindowPulse < nativeWindow ? config.followingWindowPulse : nativeWindow;
        const double quarter = 0.25 * window;
        const double floor = quarter > 4.0 ? quarter : 4.0;
        return 0.5 * window < floor ? 0.5 * window : floor;
    }
    struct Config
    {
        AxisConfig axis[AxisCount]{};
        std::uint64_t maxDurationUs = MaximumDurationUs;
        std::uint64_t maxSourceGapUs = 2000ULL;
        bool visibleZFlush158 = false; // Frozen B1 commissioning-family permission; ABI retained.
        std::uint8_t commissioningProfile = 0U; // Existing padding: 0 ordinary, 1 five-mm Z, 2 two-mm XYZ, 3 two-mm XYZCUV, 4 G76 two-mm B1.
    };
    struct AxisFrame
    {
        double commandPulse = 0.0;
        double actualPulse = 0.0;
        double followingErrorPulse = 0.0;
        double inPositionWindowPulse = 0.0;
        std::uint64_t configFingerprint = 0ULL;
        std::uint64_t mappingFingerprint = 0ULL;
        std::int32_t hardwareSign = 0;
        bool writable = false;
        bool healthy = false;
    };
    struct Frame
    {
        AdmissionDiagnostic admission{};
        Identity identity{};
        AxisFrame axis[AxisCount]{};
        std::uint64_t tick = 0ULL;
        std::uint64_t nowUs = 0ULL;
        std::uint32_t boundaryReadyMask = 0U; // Exact RT incoming-axis readiness predicate.
        bool clockValid = false;
        bool pdoValid = false;
        bool sourceFresh = false;
        bool groupIdle = false;
        bool allCommandZero = false;
        bool visibleZFlushActive158 = false; // Current mapped B1 flush only; never a GAP permit.
        // The caller proves exact current owner/epoch authority. During normal
        // G01 only the frozen AUTO generation and legitimate NC_MEMORY epochs
        // are allowed. Cancellation/release are zero-only safety lifecycles.
        bool authorityCurrent = false;
        bool healthy = false;
        std::uint8_t mappedFlushProfile = 0U; // Existing padding; current mapped family identity, or zero.
    };
    struct AxisEvidence
    {
        double actualMinPulse = 0.0;
        double actualMaxPulse = 0.0;
        double commandMinPulse = 0.0;
        double commandMaxPulse = 0.0;
        double currentActualPulse = 0.0;
        double currentCommandPulse = 0.0;
        double peakPreGuardCsvPps = 0.0; // Absolute request before this fixture's clamp; not a send receipt.
        std::uint64_t capClampCount = 0ULL; // Clamp calls, not elapsed cycles or movement evidence.
        std::uint64_t nonzeroSentCount = 0ULL;
        std::uint32_t peakCsv = 0U;
    };
    enum class CompletionReleasePhase : std::uint32_t
    { None, Requested, AwaitingZero, Released, Revoked };
    struct Feedback
    {
        RuntimeDiagnostic firstFault{};
        FaultAxisDiagnostic firstFaultAxis{};
        StopDiagnostic stop{};
        AdmissionDiagnostic admission{};
        Identity identity{};
        Identity currentIdentity{};
        // A normal M30 retires only its own IDLE_HOLD -> NONE generation.
        // This receipt never grants AUTO authority or queues a START.
        CompletionReleasePhase completionPhase = CompletionReleasePhase::None;
        std::uint64_t completionEpoch = 0ULL;
        std::uint32_t completionIdleOwnerGeneration = 0U;
        std::uint32_t completionNoneOwnerGeneration = 0U;
        bool completionReleased = false;
        Config config{};
        AxisEvidence axis[AxisCount]{};
        std::int32_t expectedCsv[AxisCount]{};
        std::uint64_t firstTick = 0ULL;
        std::uint64_t lastTick = 0ULL;
        std::uint64_t firstNowUs = 0ULL;
        std::uint64_t lastNowUs = 0ULL;
        std::uint64_t outputTick = 0ULL;
        std::uint64_t lastSentTick = 0ULL;
        std::uint64_t sentFrames = 0ULL;
        std::uint64_t cancelledDiscardedFrames = 0ULL;
        std::uint64_t lastTransportTick = 0ULL;
        std::uint64_t sealedFrames = 0ULL;
        std::uint64_t nicAttemptedFrames = 0ULL;
        std::uint64_t nicAcceptedFrames = 0ULL;
        std::uint64_t acceptedZeroFrames = 0ULL;
        std::uint32_t expectedMask = 0U;
        std::uint32_t saturatedMask = 0U;
        std::uint32_t boundaryReadyCycles = 0U;
        std::uint32_t boundaryNotReadyMask = AllAxes;
        std::uint32_t nonzeroSentMask = 0U;
        std::uint32_t actualMovedMask = 0U;
        std::uint32_t commandMovedMask = 0U;
        // Frozen at admission. All six axes remain in every source, health,
        // output, following and physical zero/settlement check.
        std::uint32_t requiredMovementMask = AllAxes;
        std::uint32_t settledCycles = 0U;
        std::uint32_t requiredCycles = RequiredStableCycles;
        unsigned rejectedAxis = AxisCount;
        State state = State::Idle;
        Reason reason = Reason::None;
        bool armAccepted = false;
        bool armedReady = false;
        bool active = false;
        bool forceZero = false;
        bool finishRequested = false;
        bool cancelled = false;
        bool faulted = false;
        bool stopped = false;
        bool released = false;
        bool pass = false;
        bool lastTransportSealed = false;
        bool lastTransportAttempted = false;
        bool lastTransportAccepted = false;
        bool lastTransportAllZero = false;
        bool handoffZero = false;
        std::uint32_t handoffStarted = 0U, handoffCompleted = 0U;
        // EDM156: the original admission anchors still define the envelope.
        // Only the last accepted NC row may seal a different finish target.
        std::array<double, AxisCount> finishTargetPulse{};
        bool boundedProgram = false;
        bool finishTargetBound = false;
    };
    static_assert(std::is_trivially_copyable<Feedback>::value, "EDM94 feedback must be POD.");
    static_assert(sizeof(Feedback) <= 4096U, "EDM94 feedback publication budget.");

    // Movement evidence concerns the admitted authored axes. Stationary axes
    // can still receive ordinary holding PI output and encoder corrections.
    // That activity cannot substitute for the commanded axis's real motion.
    inline bool MovementProven(const Feedback& feedback, unsigned expectedMask) noexcept
    {
        const bool validMask = feedback.boundedProgram ?
            (expectedMask <= AllAxes && (expectedMask & 7U) != 0U) :
            (expectedMask == AllAxes || expectedMask == (1U << 2U) || expectedMask == 7U);
        // A B1 excursion can move Z even when no nominal NC row moves Z.
        // It cannot replace evidence for any admitted nominal moving axis.
        const unsigned allowedMask = feedback.boundedProgram ? expectedMask | 4U : expectedMask;
        return validMask &&
            feedback.requiredMovementMask == expectedMask &&
            ((feedback.nonzeroSentMask | feedback.actualMovedMask | feedback.commandMovedMask) & ~AllAxes) == 0U &&
            (feedback.nonzeroSentMask & expectedMask) == expectedMask &&
            (feedback.actualMovedMask & expectedMask) == expectedMask &&
            (feedback.commandMovedMask & expectedMask) == expectedMask &&
            (feedback.commandMovedMask & ~allowedMask) == 0U;
    }

    inline const char* StateName(State value) noexcept
    {
        switch (value)
        {
        case State::Idle: return "IDLE"; case State::Armed: return "ARMED";
        case State::Running: return "RUNNING"; case State::Finishing: return "FINISHING";
        case State::Zeroing: return "ZEROING"; case State::Stopped: return "STOPPED";
        case State::Cancelled: return "CANCELLED"; case State::Faulted: return "FAULTED";
        case State::Released: return "RELEASED";
        }
        return "INVALID";
    }
    inline const char* ReasonName(Reason value) noexcept
    {
        switch (value)
        {
        case Reason::None: return "NONE"; case Reason::Configuration: return "CONFIGURATION";
        case Reason::Identity: return "IDENTITY"; case Reason::Clock: return "CLOCK";
        case Reason::Source: return "SOURCE"; case Reason::ConfigurationChanged: return "CONFIGURATION_CHANGED";
        case Reason::Following: return "FOLLOWING"; case Reason::Excursion: return "EXCURSION";
        case Reason::Output: return "OUTPUT"; case Reason::Serialized: return "SERIALIZED";
        case Reason::Send: return "SEND"; case Reason::Cancelled: return "CANCELLED";
        case Reason::Duration: return "DURATION"; case Reason::Release: return "RELEASE";
        case Reason::MotionNotObserved: return "MOTION_NOT_OBSERVED";
        }
        return "INVALID";
    }

    class Engine
    {
    public:
        const Feedback& Current() const noexcept { return m_feedback; }
        const Config& FrozenConfig() const noexcept { return m_feedback.config; }
        const Frame& LastFrame() const noexcept { return m_lastFrame; }
        bool Active() const noexcept { return m_feedback.active; }
        bool ForceZero() const noexcept { return m_feedback.active && m_feedback.forceZero; }
        bool HandoffZero() const noexcept { return Active() && m_feedback.handoffZero && !m_feedback.forceZero; }
        void BeginHandoffZero() noexcept
        {
            if (!Active() || m_feedback.forceZero || m_feedback.handoffZero) return;
            ResetBoundaryReadiness();
            m_feedback.handoffZero = true; m_feedback.armedReady = false;
            if (m_feedback.handoffStarted != (std::numeric_limits<std::uint32_t>::max)()) ++m_feedback.handoffStarted;
            ResetStopEvidence();
        }
        void CompleteHandoffZero() noexcept
        {
            if (!HandoffZero()) return;
            m_feedback.handoffZero = false;
            if (m_feedback.handoffCompleted != (std::numeric_limits<std::uint32_t>::max)()) ++m_feedback.handoffCompleted;
        }
        const RuntimeDiagnostic& CurrentRuntimeDiagnostic() const noexcept { return m_runtimeDiagnostic; }
        void SetFaultContext(const RuntimeDiagnostic& diagnostic) noexcept
        { if (!m_feedback.faulted) m_runtimeDiagnostic = diagnostic; }
        void ObserveSendProof(std::uint64_t tick, std::uint64_t generation,
            std::uint64_t owner, std::uint64_t epoch) noexcept
        {
            if (!Active() || m_feedback.faulted) return;
            m_runtimeDiagnostic.flags |= RuntimeFlag_PROOF_CONTEXT_VALID;
            m_runtimeDiagnostic.proofTick = tick; m_runtimeDiagnostic.proofGeneration = generation;
            m_runtimeDiagnostic.proofOwnerState = owner; m_runtimeDiagnostic.proofExecutionPublication = epoch;
        }
        void ObserveReleasePrecheck(bool resetComplete, bool unchangedStop) noexcept
        {
            m_feedback.stop.releaseBlockedMask =
                (resetComplete ? 0U : ReleaseBlock_RESET_PRECHECK) |
                (resetComplete && !unchangedStop ? ReleaseBlock_FRAME_CHANGED : 0U) |
                (m_feedback.stopped && m_feedback.settledCycles >= RequiredStableCycles ? 0U : ReleaseBlock_STOP_NOT_PROVEN);
        }

        bool Arm(const Identity& identity, const Config& config, const Frame& frame,
            unsigned requiredMovementMask = AllAxes, bool boundedProgram = false, bool visibleZFlush158 = false) noexcept
        {
            if (Active()) return false;
            m_feedback = Feedback{};
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            { m_axisControl[axis] = AxisControlSample{}; m_acceptedCsv[axis] = 0; }
            m_acceptedIdentity = Identity{}; m_acceptedTick = m_acceptedSourceNowUs = 0ULL;
            const bool validMovementMask = (config.visibleZFlush158 == visibleZFlush158) &&
                (!visibleZFlush158 || (boundedProgram && requiredMovementMask == VisibleFlushRequiredMask(config.commissioningProfile))) &&
                !frame.visibleZFlushActive158 && !frame.mappedFlushProfile &&
                (boundedProgram ? (requiredMovementMask <= AllAxes && (requiredMovementMask & 7U) != 0U) :
                (requiredMovementMask == AllAxes || requiredMovementMask == (1U << 2U) || requiredMovementMask == 7U));
            m_feedback.boundedProgram = boundedProgram;
            // Snapshot only this admission attempt. Tick/Send/Cancel/Release
            // never replace it; accepted-session evidence below is unchanged.
            AdmissionDiagnostic admission = frame.admission;
            admission.tick = frame.tick; admission.nowUs = frame.nowUs;
            admission.failedGates =
                (frame.pdoValid ? 0U : AdmissionGate_PDO) |
                (frame.sourceFresh ? 0U : AdmissionGate_FRESH) |
                (frame.authorityCurrent ? 0U : AdmissionGate_AUTHORITY) |
                (frame.healthy ? 0U : AdmissionGate_HEALTH) |
                (frame.groupIdle ? 0U : AdmissionGate_GROUP_IDLE) |
                (frame.allCommandZero ? 0U : AdmissionGate_COMMAND_ZERO) |
                (frame.clockValid && frame.tick && frame.nowUs ? 0U : AdmissionGate_CLOCK) |
                (ValidIdentity(identity) && SameIdentity(identity, frame.identity) ? 0U : AdmissionGate_IDENTITY) |
                (ValidConfig(config) && validMovementMask ? 0U : AdmissionGate_CONFIGURATION);
            m_feedback.admission = admission;
            m_feedback.identity = identity;
            m_feedback.currentIdentity = frame.identity;
            m_feedback.config = config;
            ObserveFrameDiagnostic(frame);
            m_pendingSeal = false;
            m_sealed = false;
            m_discardedTick = 0ULL;
            m_stopEligible = false;
            m_haveStableRange = false;
            if (!ValidIdentity(identity) || !SameIdentity(identity, frame.identity))
                return RejectArm(Reason::Identity);
            if (!ValidConfig(config) || !validMovementMask) return RejectArm(Reason::Configuration);
            m_feedback.requiredMovementMask = requiredMovementMask;
            if (!frame.clockValid || frame.tick == 0ULL || frame.nowUs == 0ULL)
                return RejectArm(Reason::Clock);
            if (!frame.pdoValid || !frame.sourceFresh || !frame.authorityCurrent || !frame.healthy ||
                !frame.groupIdle || !frame.allCommandZero) return RejectArm(Reason::Source);
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                if (!CurrentAxis(frame, axis) || !frame.axis[axis].healthy ||
                    frame.axis[axis].commandPulse != config.axis[axis].commandAnchor ||
                    frame.axis[axis].actualPulse != config.axis[axis].actualAnchor ||
                    !InFollowingWindow(frame, axis))
                { m_feedback.rejectedAxis = axis; m_feedback.admission.failedGates |= AdmissionGate_CONFIGURATION;
                    return RejectArm(Reason::Configuration); }
                AxisEvidence& e = m_feedback.axis[axis];
                e.actualMinPulse = e.actualMaxPulse = e.currentActualPulse = frame.axis[axis].actualPulse;
                e.commandMinPulse = e.commandMaxPulse = e.currentCommandPulse = frame.axis[axis].commandPulse;
            }
            m_feedback.firstTick = m_feedback.lastTick = frame.tick;
            m_feedback.firstNowUs = m_feedback.lastNowUs = frame.nowUs;
            m_feedback.armAccepted = m_feedback.active = m_feedback.armedReady = true;
            m_feedback.state = State::Armed;
            m_lastFrame = frame;
            return BeginOutput(frame.tick);
        }

        bool Tick(const Frame& frame) noexcept
        {
            if (!Active()) return false;
            const std::uint32_t boundaryCycles = m_feedback.boundaryReadyCycles;
            bool sameBoundaryReference = SameIdentity(frame.identity, m_lastFrame.identity);
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
                sameBoundaryReference = sameBoundaryReference &&
                    frame.axis[axis].commandPulse == m_lastFrame.axis[axis].commandPulse;
            // Every failed/early-return sample revokes the readiness witness.
            ResetBoundaryReadiness();
            ObserveFrameDiagnostic(frame);
            m_stopEligible = false;
            if (m_pendingSeal || (m_feedback.lastSentTick != m_feedback.lastTick &&
                !(m_discardedTick == m_feedback.lastTick && m_feedback.cancelled &&
                    m_feedback.forceZero && !m_feedback.faulted))) Fault(Reason::Send);
            m_pendingSeal = false;
            m_sealed = false;
            const bool clock = frame.clockValid && frame.tick != 0ULL &&
                m_feedback.lastTick != (std::numeric_limits<std::uint64_t>::max)() &&
                frame.tick == m_feedback.lastTick + 1ULL && frame.nowUs > m_feedback.lastNowUs &&
                frame.nowUs - m_feedback.lastNowUs <= m_feedback.config.maxSourceGapUs;
            if (!clock)
            {
                m_feedback.stop.blockedMask |= StopBlock_CLOCK;
                Fault(Reason::Clock, AxisCount, FaultSite::TickClock);
                // A discontinuity revokes motion permanently, but must not
                // make later zero-only stop evidence impossible. Re-anchor
                // only to a newer identified cycle; the offending cycle never
                // contributes to the 200-cycle stop window.
                if (frame.tick > m_feedback.lastTick && frame.tick != 0ULL)
                {
                    m_feedback.lastTick = frame.tick;
                    if (frame.clockValid && frame.nowUs > m_feedback.lastNowUs)
                        m_feedback.lastNowUs = frame.nowUs;
                    m_feedback.currentIdentity = frame.identity;
                    m_lastFrame = frame;
                    (void)BeginOutput(frame.tick);
                }
                return false;
            }
            m_feedback.lastTick = frame.tick;
            m_feedback.lastNowUs = frame.nowUs;
            if (m_feedback.forceZero && !sameBoundaryReference)
                ResetStopEvidence(); // One dwell belongs to one exact identity and unchanged six-axis command reference.
            m_feedback.currentIdentity = frame.identity;
            m_lastFrame = frame;
            if (!frame.pdoValid || !frame.sourceFresh)
            { Fault(Reason::Source); (void)BeginOutput(frame.tick); return false; }
            if (!ValidIdentity(frame.identity) || frame.identity.session != m_feedback.identity.session ||
                (!m_feedback.forceZero && frame.identity.ownerGeneration != m_feedback.identity.ownerGeneration))
            { m_feedback.stop.blockedMask |= StopBlock_IDENTITY;
                Fault(Reason::Identity, AxisCount, FaultSite::TickIdentity); (void)BeginOutput(frame.tick); return false; }
            if (!frame.authorityCurrent && !HandoffZero())
            {
                // An already revoked scope may observe the ordinary RESET /
                // SAFETY handshake in progress. Keep a complete zero image,
                // but grant no physical dwell until current authority is
                // coherent again. This is not a new cancellation fault.
                if (!m_feedback.forceZero) Fault(Reason::Identity, AxisCount, FaultSite::TickAuthority);
                else ResetStopEvidence();
                (void)BeginOutput(frame.tick);
                return false;
            }
            if (!m_feedback.forceZero && (frame.nowUs < m_feedback.firstNowUs ||
                frame.nowUs - m_feedback.firstNowUs > m_feedback.config.maxDurationUs)) Fault(Reason::Duration);
            bool allCurrent = true, allNear = true, allTripNear = true, allHealthy = frame.healthy, inEnvelope = true;
            unsigned firstFollowingAxis = AxisCount;
            std::uint32_t boundaryBlocked = (frame.boundaryReadyMask & ~AllAxes) != 0U ?
                AllAxes : AllAxes & ~frame.boundaryReadyMask;
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                if (!CurrentAxis(frame, axis))
                { allCurrent = false; Fault(Reason::ConfigurationChanged, axis); continue; }
                allHealthy = allHealthy && frame.axis[axis].healthy;
                const bool axisWithinFollowingWindow = InFollowingWindow(frame, axis);
                if (!axisWithinFollowingWindow) boundaryBlocked |= 1U << axis;
                allNear = allNear && axisWithinFollowingWindow; // All six axes still required for completion/stop.
                if ((FollowingTripAxisMask & (1U << axis)) != 0U)
                {
                    if (!axisWithinFollowingWindow && firstFollowingAxis == AxisCount) firstFollowingAxis = axis;
                    allTripNear = allTripNear && axisWithinFollowingWindow;
                }
                const AxisConfig& c = m_feedback.config.axis[axis];
                const AxisFrame& a = frame.axis[axis];
                const double limit = NativeExcursionLimit(axis) * c.pulsePerUnit;
                const bool mappedProfile = frame.visibleZFlushActive158 &&
                    frame.mappedFlushProfile == m_feedback.config.commissioningProfile;
                const bool visible = axis == 2U && m_feedback.config.visibleZFlush158 &&
                    mappedProfile && !m_feedback.finishRequested;
                const double upper = visible ?
                    (VisibleFlushHeightMm(m_feedback.config.commissioningProfile) + NativeExcursionLimit(axis)) * c.pulsePerUnit : limit;
                const double commandDelta = a.commandPulse - c.commandAnchor;
                const double actualDelta = a.actualPulse - c.actualAnchor;
                if (commandDelta < -limit || commandDelta > upper || actualDelta < -limit || actualDelta > upper ||
                    (frame.visibleZFlushActive158 && (!m_feedback.config.visibleZFlush158 || !mappedProfile)) ||
                    (!frame.visibleZFlushActive158 && frame.mappedFlushProfile != 0U))
                { inEnvelope = false; Fault(Reason::Excursion, axis); }
                if (HandoffZero()) continue; // A temporary zero bridge earns no movement evidence.
                AxisEvidence& e = m_feedback.axis[axis];
                e.currentActualPulse = a.actualPulse;
                e.currentCommandPulse = a.commandPulse;
                if (a.actualPulse < e.actualMinPulse) e.actualMinPulse = a.actualPulse;
                if (a.actualPulse > e.actualMaxPulse) e.actualMaxPulse = a.actualPulse;
                if (a.commandPulse < e.commandMinPulse) e.commandMinPulse = a.commandPulse;
                if (a.commandPulse > e.commandMaxPulse) e.commandMaxPulse = a.commandPulse;
                if (e.actualMaxPulse - e.actualMinPulse >= 1.0) m_feedback.actualMovedMask |= 1U << axis;
                if (e.commandMaxPulse - e.commandMinPulse >= 1.0) m_feedback.commandMovedMask |= 1U << axis;
            }
            if (!allCurrent) { ResetStopEvidence(); (void)BeginOutput(frame.tick); return false; }
            if (!m_feedback.forceZero && !allHealthy) Fault(Reason::Source);
            if (!m_feedback.forceZero && !allTripNear) Fault(Reason::Following, firstFollowingAxis);
            if (!m_feedback.forceZero && m_feedback.finishRequested && frame.groupIdle &&
                frame.allCommandZero && allNear && allHealthy && inEnvelope)
            {
                if (!MovementProven()) Fault(Reason::MotionNotObserved);
                else
                {
                    bool returned = true;
                    for (unsigned axis = 0U; axis < AxisCount; ++axis)
                    {
                        const AxisConfig& c = m_feedback.config.axis[axis];
                        const double target = m_feedback.boundedProgram ? m_feedback.finishTargetPulse[axis] : c.commandAnchor;
                        returned = returned && (!m_feedback.boundedProgram || m_feedback.finishTargetBound) &&
                            std::abs(frame.axis[axis].commandPulse - target) <= c.followingWindowPulse &&
                            std::abs(frame.axis[axis].actualPulse - target) <= c.followingWindowPulse;
                    }
                    if (!returned) Fault(Reason::Excursion);
                    else { m_feedback.forceZero = true; m_feedback.state = State::Zeroing; }
                }
            }
            if (!m_feedback.forceZero && (!frame.groupIdle || !frame.allCommandZero))
                m_feedback.state = m_feedback.finishRequested ? State::Finishing : State::Running;
            m_stopEligible = m_feedback.forceZero && frame.groupIdle && frame.allCommandZero && allHealthy &&
                (m_feedback.faulted || m_feedback.cancelled || (allNear && inEnvelope));
            if (!m_feedback.forceZero) m_feedback.stop.blockedMask |= StopBlock_NOT_ZEROING;
            if (!(m_feedback.faulted || m_feedback.cancelled || (allNear && inEnvelope)))
                m_feedback.stop.blockedMask |= StopBlock_FOLLOWING_OR_ENVELOPE;
            if (!m_stopEligible) ResetStopEvidence();
            else ObserveStableRange(frame);
            // Keep the existing capped endpoint PID active while NC waits.
            // Both the ordinary incoming predicate and the tighter all-six
            // fixture windows must hold on every contiguous RT observation.
            if (!m_feedback.forceZero && !m_feedback.finishRequested && !HandoffZero())
            {
                m_feedback.boundaryNotReadyMask = boundaryBlocked;
                if (sameBoundaryReference && frame.authorityCurrent && frame.groupIdle &&
                    frame.allCommandZero && allCurrent && allNear && allHealthy && inEnvelope &&
                    boundaryBlocked == 0U)
                    m_feedback.boundaryReadyCycles = boundaryCycles < BoundaryReadyCycles ?
                        boundaryCycles + 1U : BoundaryReadyCycles;
            }
            m_feedback.armedReady = !m_feedback.forceZero && !m_feedback.finishRequested && !HandoffZero();
            return BeginOutput(frame.tick);
        }

        bool BeginOutput(std::uint64_t tick) noexcept
        {
            if (!Active() || tick == 0ULL || tick != m_feedback.lastTick) return false;
            m_feedback.outputTick = tick;
            m_feedback.expectedMask = 0U;
            m_feedback.saturatedMask = 0U;
            m_sealed = false;
            for (unsigned axis = 0U; axis < AxisCount; ++axis) m_feedback.expectedCsv[axis] = 0;
            return true;
        }

        // Diagnostic only: called before the physical writer's integer clamp,
        // and before the standalone engine clamp. Re-observing a capped value
        // cannot add another clamp count. This grants no output/proof credit.
        void ObserveVelocityRequest(unsigned axis, double requested, double cap) noexcept
        {
            if (!Active() || m_feedback.forceZero || HandoffZero() || axis >= AxisCount ||
                !std::isfinite(requested) || !std::isfinite(cap) || cap < 1.0) return;
            AxisEvidence& evidence = m_feedback.axis[axis];
            const double requestMagnitude = std::abs(requested);
            if (requestMagnitude > evidence.peakPreGuardCsvPps)
                evidence.peakPreGuardCsvPps = requestMagnitude;
            if (requestMagnitude > cap && evidence.capClampCount != (std::numeric_limits<std::uint64_t>::max)())
                ++evidence.capClampCount;
        }

        // No output, authority or following/settle policy is changed here.
        void ObserveAxisControlSample(unsigned axis, const AxisControlSample& sample) noexcept
        {
            if (!Active() || m_feedback.faulted || axis >= AxisCount ||
                !ValidIdentity(sample.identity) || sample.identity.session != m_feedback.identity.session ||
                sample.tick == 0ULL || sample.nowUs == 0ULL) return;
            m_axisControl[axis] = sample;
        }

        bool GuardVelocity(unsigned axis, double& logicalVelocity) noexcept
        {
            if (!Active()) return true;
            if (axis >= AxisCount || m_feedback.outputTick != m_feedback.lastTick || m_sealed)
            { logicalVelocity = 0.0; Fault(Reason::Output, axis); return false; }
            if (!std::isfinite(logicalVelocity))
            { logicalVelocity = 0.0; Fault(Reason::Output, axis); return false; }
            if (m_feedback.forceZero || HandoffZero()) logicalVelocity = 0.0;
            else
            {
                const double cap = NativeVelocityLimit(axis) * m_feedback.config.axis[axis].pulsePerUnit;
                ObserveVelocityRequest(axis, logicalVelocity, cap);
                if (logicalVelocity > cap) { logicalVelocity = cap; m_feedback.saturatedMask |= 1U << axis; }
                if (logicalVelocity < -cap) { logicalVelocity = -cap; m_feedback.saturatedMask |= 1U << axis; }
            }
            const double native = logicalVelocity * static_cast<double>(m_feedback.config.axis[axis].hardwareSign);
            if (!std::isfinite(native) || native > static_cast<double>((std::numeric_limits<std::int32_t>::max)()) ||
                native < static_cast<double>((std::numeric_limits<std::int32_t>::min)()))
            { logicalVelocity = 0.0; Fault(Reason::Output, axis); return false; }
            m_feedback.expectedCsv[axis] = static_cast<std::int32_t>(native);
            m_feedback.expectedMask |= 1U << axis;
            return true;
        }

        bool SealOutput(std::uint64_t tick, const std::int32_t (&csv)[AxisCount]) noexcept
        {
            if (!Active() || tick == 0ULL || tick != m_feedback.outputTick || tick != m_feedback.lastTick ||
                tick <= m_feedback.lastSentTick || tick == m_discardedTick || m_sealed ||
                (!m_feedback.forceZero && m_feedback.expectedMask != AllAxes))
            { if (Active()) Fault(Reason::Serialized); return false; }
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                const std::int32_t expected = m_feedback.forceZero ? 0 : m_feedback.expectedCsv[axis];
                if (csv[axis] != expected) { Fault(Reason::Serialized, axis); return false; }
            }
            m_sealed = true;
            m_pendingSeal = true;
            m_sealedIdentity = m_feedback.currentIdentity;
            for (unsigned axis = 0U; axis < AxisCount; ++axis) m_sealedCsv[axis] = csv[axis];
            return true;
        }

        bool ObserveSent(std::uint64_t tick, const std::int32_t (&csv)[AxisCount], bool sent) noexcept
        {
            if (!Active() || !m_sealed || !m_pendingSeal || tick == 0ULL ||
                tick != m_feedback.outputTick || tick != m_feedback.lastTick ||
                !SameIdentity(m_sealedIdentity, m_feedback.currentIdentity) || tick <= m_feedback.lastSentTick)
            { if (Active()) Fault(Reason::Send); return false; }
            m_pendingSeal = false;
            if (!sent) { Fault(Reason::Send); ResetStopEvidence(); return false; }
            // Validate the complete receipt before committing any per-axis
            // transmission counters; a late-axis mismatch is a group reject.
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
                if (csv[axis] != m_sealedCsv[axis])
                { Fault(Reason::Serialized, axis); ResetStopEvidence(); return false; }
            bool allZero = true;
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                allZero = allZero && csv[axis] == 0;
                const std::uint32_t magnitude = csv[axis] < 0 ?
                    static_cast<std::uint32_t>(-static_cast<std::int64_t>(csv[axis])) : static_cast<std::uint32_t>(csv[axis]);
                AxisEvidence& e = m_feedback.axis[axis];
                if (magnitude > e.peakCsv) e.peakCsv = magnitude;
                if (csv[axis] != 0)
                {
                    m_feedback.nonzeroSentMask |= 1U << axis;
                    if (e.nonzeroSentCount != (std::numeric_limits<std::uint64_t>::max)()) ++e.nonzeroSentCount;
                }
            }
            // Preserve only a complete exact image whose same-tick transport
            // was sealed, attempted and NIC-accepted. Source time is RX/image
            // time, not the NIC completion time and never a drive ACK.
            if (m_feedback.lastTransportTick == tick && m_feedback.lastTransportSealed &&
                m_feedback.lastTransportAttempted && m_feedback.lastTransportAccepted)
            {
                m_acceptedIdentity = m_sealedIdentity; m_acceptedTick = tick;
                m_acceptedSourceNowUs = m_feedback.lastNowUs;
                for (unsigned axis = 0U; axis < AxisCount; ++axis) m_acceptedCsv[axis] = csv[axis];
            }
            m_feedback.lastSentTick = tick;
            if (m_feedback.sentFrames != (std::numeric_limits<std::uint64_t>::max)()) ++m_feedback.sentFrames;
            if (allZero && m_stopEligible && m_haveStableRange)
            {
                if (m_feedback.settledCycles < RequiredStableCycles) ++m_feedback.settledCycles;
                if (m_feedback.settledCycles == RequiredStableCycles)
                {
                    m_feedback.stopped = true;
                    m_feedback.pass = m_feedback.finishRequested && !m_feedback.cancelled && !m_feedback.faulted && MovementProven();
                    if (m_feedback.pass) m_feedback.state = State::Stopped;
                }
            }
            else
            {
                if (!allZero) m_feedback.stop.blockedMask |= StopBlock_SEND;
                ResetStopEvidence();
            }
            return true;
        }

        // Diagnostic-only milestones from EndServoOutputFrameAfterSend.
        // NIC acceptance does not imply exact-image acceptance or encoder stop.
        // These counters cannot authorize motion, completion, or release.
        bool ObserveTransport(std::uint64_t tick, bool sealed, bool attempted,
            bool accepted, bool allZero) noexcept
        {
            if (!Active() || tick == 0ULL || tick != m_feedback.outputTick ||
                tick != m_feedback.lastTick || tick <= m_feedback.lastTransportTick ||
                (accepted && !attempted)) return false;
            m_feedback.lastTransportTick = tick;
            m_feedback.lastTransportSealed = sealed;
            m_feedback.lastTransportAttempted = attempted;
            m_feedback.lastTransportAccepted = accepted;
            m_feedback.lastTransportAllZero = sealed && allZero;
            IncrementIf(m_feedback.sealedFrames, sealed);
            IncrementIf(m_feedback.nicAttemptedFrames, attempted);
            IncrementIf(m_feedback.nicAcceptedFrames, accepted);
            IncrementIf(m_feedback.acceptedZeroFrames, sealed && attempted && accepted && allZero);
            return true;
        }

        // The sender may intentionally suppress an exact previously prepared
        // moving image when cancellation arrives before serialization seal.
        // This is NOT a NIC receipt and cannot contribute to physical dwell.
        // A failed seal, unknown payload or attempted NIC failure uses Fault.
        bool DiscardCancelledFrame(std::uint64_t tick) noexcept
        {
            if (!Active() || !m_feedback.cancelled || !m_feedback.forceZero || m_feedback.faulted ||
                m_sealed || m_pendingSeal || tick == 0ULL || tick != m_feedback.lastTick ||
                tick != m_feedback.outputTick || tick <= m_feedback.lastSentTick ||
                tick == m_discardedTick) return false;
            m_discardedTick = tick;
            m_feedback.expectedMask = 0U;
            for (unsigned axis = 0U; axis < AxisCount; ++axis) m_feedback.expectedCsv[axis] = 0;
            if (m_feedback.cancelledDiscardedFrames != (std::numeric_limits<std::uint64_t>::max)())
                ++m_feedback.cancelledDiscardedFrames;
            ResetStopEvidence();
            return true;
        }

        bool BindFinishTarget(const std::array<double, AxisCount>& target) noexcept
        {
            if (!Active() || !m_feedback.boundedProgram || m_feedback.finishTargetBound ||
                m_feedback.finishRequested || m_feedback.forceZero || m_feedback.cancelled || m_feedback.faulted) return false;
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                const AxisConfig& config = m_feedback.config.axis[axis];
                const double limit = NativeExcursionLimit(axis) * config.pulsePerUnit;
                if (!std::isfinite(target[axis]) || !std::isfinite(limit) ||
                    std::abs(target[axis] - config.commandAnchor) > limit) return false;
            }
            m_feedback.finishTargetPulse = target;
            m_feedback.finishTargetBound = true;
            return true;
        }

        void Finish() noexcept
        {
            if (!Active() || m_feedback.forceZero) return;
            if (m_feedback.boundedProgram && !m_feedback.finishTargetBound)
            { Fault(Reason::Identity); return; }
            ResetBoundaryReadiness();
            m_feedback.finishRequested = true;
            m_feedback.armedReady = false;
            m_feedback.state = State::Finishing;
        }
        void Cancel() noexcept
        {
            if (!Active()) return;
            ResetBoundaryReadiness();
            if (m_feedback.cancelled) return;
            m_feedback.cancelled = true;
            m_feedback.forceZero = true;
            m_feedback.armedReady = false;
            m_feedback.pass = false;
            if (!m_feedback.faulted) { m_feedback.state = State::Cancelled; m_feedback.reason = Reason::Cancelled; }
            ResetStopEvidence();
        }
        void Fault(Reason reason, unsigned axis = AxisCount, FaultSite site = FaultSite::EngineFault) noexcept
        {
            if (!Active()) return;
            ResetBoundaryReadiness();
            if (!m_feedback.faulted)
            {
                m_feedback.firstFault = m_runtimeDiagnostic;
                CaptureFirstFaultAxis(axis);
                m_feedback.firstFault.reason = static_cast<std::uint32_t>(reason);
                if (m_feedback.firstFault.site == 0U) m_feedback.firstFault.site = static_cast<std::uint32_t>(site);
                if (m_feedback.forceZero) m_feedback.firstFault.flags |= RuntimeFlag_FORCE_ZERO_BEFORE_FAULT;
                m_feedback.reason = reason; m_feedback.rejectedAxis = axis;
            }
            switch (reason)
            {
            case Reason::Clock: m_feedback.stop.blockedMask |= StopBlock_CLOCK; break;
            case Reason::Source: m_feedback.stop.blockedMask |= StopBlock_SOURCE; break;
            case Reason::Identity: m_feedback.stop.blockedMask |= StopBlock_IDENTITY; break;
            case Reason::Configuration: case Reason::ConfigurationChanged:
                m_feedback.stop.blockedMask |= StopBlock_CONFIGURATION; break;
            case Reason::Send: case Reason::Serialized: case Reason::Output:
                m_feedback.stop.blockedMask |= StopBlock_SEND; break;
            case Reason::Following: case Reason::Excursion:
                m_feedback.stop.blockedMask |= StopBlock_FOLLOWING_OR_ENVELOPE; break;
            default: break;
            }
            m_feedback.faulted = true;
            m_feedback.forceZero = true;
            m_feedback.armedReady = false;
            m_feedback.pass = false;
            m_feedback.state = State::Faulted;
            ResetStopEvidence();
        }

        void ObserveCompletionRelease(CompletionReleasePhase phase,
            std::uint64_t epoch, std::uint32_t idleGeneration,
            std::uint32_t noneGeneration) noexcept
        {
            if (!Active() || phase == CompletionReleasePhase::Released) return;
            m_feedback.completionPhase = phase;
            m_feedback.completionEpoch = epoch;
            m_feedback.completionIdleOwnerGeneration = idleGeneration;
            m_feedback.completionNoneOwnerGeneration = noneGeneration;
        }

        bool CanReleaseCompleted(const Identity& identity, const Frame& frame,
            bool normalCompletionReady) const noexcept
        {
            if (!normalCompletionReady || !m_feedback.finishRequested ||
                !m_feedback.pass || m_feedback.cancelled || m_feedback.faulted ||
                !MovementProven() ||
                m_feedback.completionPhase != CompletionReleasePhase::AwaitingZero ||
                frame.identity.epoch != m_feedback.completionEpoch ||
                frame.identity.ownerGeneration != m_feedback.completionNoneOwnerGeneration ||
                !CanReleaseStopProof(identity, frame, true)) return false;
            return true;
        }
        bool ReleaseCompleted(const Identity& identity, const Frame& frame,
            bool normalCompletionReady) noexcept
        {
            if (!CanReleaseCompleted(identity, frame, normalCompletionReady) ||
                !ReleaseStopProof(identity, frame, true)) return false;
            m_feedback.completionReleased = true;
            m_feedback.completionPhase = CompletionReleasePhase::Released;
            return true;
        }

        // explicitResetReady is supplied only after the EXISTING RESET settle
        // and rebase proof. This operation never performs the rebase itself.
        bool Release(const Identity& identity, const Frame& frame, bool explicitResetReady) noexcept
        { return ReleaseStopProof(identity, frame, explicitResetReady); }
        bool Release(const Identity& identity, bool explicitResetReady) noexcept
        { return Release(identity, m_lastFrame, explicitResetReady); }

    private:
        bool ReleaseStopProof(const Identity& identity, const Frame& frame, bool permitted) noexcept
        {
            if (m_feedback.stop.releaseBlockedMask == 0U)
                m_feedback.stop.releaseBlockedMask = ReleaseBlock_FINAL_PROOF;
            if (!CanReleaseStopProof(identity, frame, permitted)) return false;
            m_feedback.stop.releaseBlockedMask = 0U;
            m_feedback.active = false;
            m_feedback.armedReady = false;
            m_feedback.released = true;
            m_feedback.state = State::Released;
            return true;
        }
        bool CanReleaseStopProof(const Identity& identity, const Frame& frame, bool permitted) const noexcept
        {
            if (!Active() || !permitted || !m_feedback.stopped ||
                m_feedback.settledCycles < RequiredStableCycles || !m_feedback.forceZero ||
                !m_stopEligible || !m_haveStableRange || !m_sealed || m_pendingSeal ||
                !ValidIdentity(identity) || identity.session != m_feedback.identity.session ||
                !SameIdentity(identity, frame.identity) || frame.tick != m_feedback.lastTick ||
                !SameIdentity(m_sealedIdentity, frame.identity) ||
                !SameIdentity(frame.identity, m_lastFrame.identity) ||
                frame.nowUs != m_feedback.lastNowUs || m_feedback.lastSentTick != frame.tick ||
                frame.visibleZFlushActive158 != m_lastFrame.visibleZFlushActive158 ||
                frame.mappedFlushProfile != m_lastFrame.mappedFlushProfile ||
                !frame.clockValid || !frame.pdoValid || !frame.sourceFresh || !frame.authorityCurrent ||
                !frame.groupIdle || !frame.allCommandZero || !frame.healthy ||
                !m_lastFrame.clockValid || !m_lastFrame.pdoValid || !m_lastFrame.sourceFresh ||
                !m_lastFrame.authorityCurrent || !m_lastFrame.groupIdle ||
                !m_lastFrame.allCommandZero || !m_lastFrame.healthy) return false;
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
                if (!CurrentAxis(frame, axis) || !frame.axis[axis].healthy || !m_lastFrame.axis[axis].healthy || !InFollowingWindow(frame, axis) ||
                    frame.axis[axis].commandPulse != m_lastFrame.axis[axis].commandPulse ||
                    frame.axis[axis].actualPulse != m_lastFrame.axis[axis].actualPulse ||
                    frame.axis[axis].followingErrorPulse != m_lastFrame.axis[axis].followingErrorPulse) return false;
            return true;
        }
        void ResetBoundaryReadiness() noexcept
        {
            m_feedback.boundaryReadyCycles = 0U;
            m_feedback.boundaryNotReadyMask = AllAxes;
        }
        static void IncrementIf(std::uint64_t& value, bool condition) noexcept
        {
            if (condition && value != (std::numeric_limits<std::uint64_t>::max)()) ++value;
        }
        static bool ValidConfig(const Config& config) noexcept
        {
            if (config.commissioningProfile > 4U ||
                config.visibleZFlush158 != (config.commissioningProfile != 0U) ||
                config.maxDurationUs == 0ULL ||
                (config.visibleZFlush158 ? config.maxDurationUs != VisibleFlushDurationUs(config.commissioningProfile) :
                    config.maxDurationUs > MaximumDurationUs) ||
                config.maxSourceGapUs == 0ULL || config.maxSourceGapUs > 2000ULL) return false;
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                const AxisConfig& c = config.axis[axis];
                const double cap = NativeVelocityLimit(axis) * c.pulsePerUnit;
                const double excursion = NativeExcursionLimit(axis) * c.pulsePerUnit;
                const double stopRange = StopRangeLimitPulse(axis, c);
                if (!std::isfinite(c.pulsePerUnit) || c.pulsePerUnit <= 0.0 ||
                    !std::isfinite(c.commandAnchor) || !std::isfinite(c.actualAnchor) ||
                    !std::isfinite(c.followingWindowPulse) || c.followingWindowPulse <= 0.0 ||
                    c.followingWindowPulse > excursion || !std::isfinite(cap) || cap < 1.0 ||
                    cap > static_cast<double>((std::numeric_limits<std::int32_t>::max)()) ||
                    !std::isfinite(excursion) || excursion <= 0.0 || !std::isfinite(stopRange) || stopRange <= 0.0 ||
                    std::abs(c.commandAnchor - c.actualAnchor) > c.followingWindowPulse ||
                    c.configFingerprint == 0ULL || c.mappingFingerprint == 0ULL ||
                    (c.hardwareSign != -1 && c.hardwareSign != 1)) return false;
                for (unsigned previous = 0U; previous < axis; ++previous)
                    if (c.mappingFingerprint == config.axis[previous].mappingFingerprint) return false;
            }
            return true;
        }
        bool CurrentAxis(const Frame& frame, unsigned axis) const noexcept
        {
            const AxisConfig& c = m_feedback.config.axis[axis];
            const AxisFrame& a = frame.axis[axis];
            return std::isfinite(a.commandPulse) && std::isfinite(a.actualPulse) &&
                std::isfinite(a.followingErrorPulse) && std::isfinite(a.inPositionWindowPulse) &&
                a.inPositionWindowPulse == c.followingWindowPulse && a.configFingerprint == c.configFingerprint &&
                a.mappingFingerprint == c.mappingFingerprint && a.hardwareSign == c.hardwareSign && a.writable;
        }
        bool InFollowingWindow(const Frame& frame, unsigned axis) const noexcept
        {
            const AxisConfig& c = m_feedback.config.axis[axis];
            const AxisFrame& a = frame.axis[axis];
            return std::abs(a.followingErrorPulse) <= c.followingWindowPulse &&
                std::abs(a.commandPulse - a.actualPulse) <= c.followingWindowPulse;
        }
        bool MovementProven() const noexcept
        { return EDM94::MovementProven(m_feedback, m_feedback.requiredMovementMask); }
        void ObserveFrameDiagnostic(const Frame& frame) noexcept
        {
            m_runtimeDiagnostic = RuntimeDiagnostic{};
            auto& d = m_runtimeDiagnostic; auto& stop = m_feedback.stop;
            d.frameIdentity = frame.identity; d.tick = frame.tick; d.nowUs = frame.nowUs;
            d.flags = RuntimeFlag_SNAPSHOT_VALID;
            d.authorityFailures = stop.authorityFailures = frame.admission.authorityFailures;
            d.sharedHealthFailures = stop.sharedHealthFailures = frame.admission.sharedHealthFailures;
            d.sourceFailures = stop.sourceFailures = frame.admission.sourceFailures;
            d.groupFailures = stop.groupFailures = frame.admission.groupFailures;
            d.inputMask = frame.admission.inputMask;
            stop.sampledTick = frame.tick;
            stop.configChangedMask = stop.mappingChangedMask = stop.unhealthyMask = 0U;
            stop.blockedMask =
                (frame.clockValid ? 0U : StopBlock_CLOCK) |
                (frame.pdoValid && frame.sourceFresh ? 0U : StopBlock_SOURCE) |
                (frame.authorityCurrent ? 0U : StopBlock_AUTHORITY) |
                (frame.healthy ? 0U : StopBlock_HEALTH) |
                (frame.groupIdle ? 0U : StopBlock_GROUP_ACTIVE) |
                (frame.allCommandZero ? 0U : StopBlock_COMMAND_NONZERO);
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                d.healthFailures[axis] = frame.admission.axis[axis].healthFailures;
                if (frame.axis[axis].configFingerprint != m_feedback.config.axis[axis].configFingerprint ||
                    frame.axis[axis].hardwareSign != m_feedback.config.axis[axis].hardwareSign ||
                    frame.axis[axis].inPositionWindowPulse != m_feedback.config.axis[axis].followingWindowPulse)
                    d.configChangedMask |= 1U << axis;
                if (frame.axis[axis].mappingFingerprint != m_feedback.config.axis[axis].mappingFingerprint)
                    d.mappingChangedMask |= 1U << axis;
                if (!frame.axis[axis].healthy || !frame.axis[axis].writable) stop.unhealthyMask |= 1U << axis;
            }
            stop.configChangedMask = d.configChangedMask; stop.mappingChangedMask = d.mappingChangedMask;
            if (stop.configChangedMask || stop.mappingChangedMask) stop.blockedMask |= StopBlock_CONFIGURATION;
            if (stop.unhealthyMask) stop.blockedMask |= StopBlock_HEALTH;
        }
        void CaptureFirstFaultAxis(unsigned axis) noexcept
        {
            auto& d = m_feedback.firstFaultAxis;
            d = FaultAxisDiagnostic{}; d.axis = axis;
            if (axis >= AxisCount) return;
            const auto& fault = m_feedback.firstFault;
            if (m_lastFrame.tick != fault.tick || m_lastFrame.nowUs != fault.nowUs ||
                !SameIdentity(m_lastFrame.identity, fault.frameIdentity)) return;
            const auto& a = m_lastFrame.axis[axis]; const auto& c = m_feedback.config.axis[axis];
            d.commandPulse = a.commandPulse; d.actualPulse = a.actualPulse;
            d.followingErrorPulse = a.followingErrorPulse; d.windowPulse = c.followingWindowPulse;
            d.pulsePerUnit = c.pulsePerUnit; d.hardwareSign = c.hardwareSign;
            d.flags = FaultAxis_FRAME;
            const auto& control = m_axisControl[axis];
            const bool sameTick = control.tick == fault.tick && control.nowUs == fault.nowUs &&
                control.commandPulse == a.commandPulse && control.actualPulse == a.actualPulse;
            const bool priorTick = control.tick != (std::numeric_limits<std::uint64_t>::max)() &&
                control.tick + 1ULL == fault.tick && control.nowUs < fault.nowUs &&
                fault.nowUs - control.nowUs <= m_feedback.config.maxSourceGapUs;
            const bool finite = std::isfinite(control.commandPulse) && std::isfinite(control.actualPulse) &&
                std::isfinite(control.referenceVelocityPps) && std::isfinite(control.kp) &&
                std::isfinite(control.ki) && std::isfinite(control.kvff) &&
                std::isfinite(control.integralBefore) && std::isfinite(control.integralAfter) &&
                std::isfinite(control.pTerm) && std::isfinite(control.iTerm) &&
                std::isfinite(control.logicalOutputVelocityPps);
            if (finite && control.tick != 0ULL && control.nowUs != 0ULL &&
                SameIdentity(control.identity, fault.frameIdentity) && (sameTick || priorTick))
            {
                d.control = control; d.flags |= FaultAxis_CONTROL;
                if (!sameTick) d.flags |= FaultAxis_CONTROL_PRIOR;
            }
            if (m_acceptedTick != 0ULL && m_acceptedTick <= fault.tick &&
                m_acceptedSourceNowUs <= fault.nowUs && SameIdentity(m_acceptedIdentity, fault.frameIdentity))
            {
                d.acceptedIdentity = m_acceptedIdentity; d.acceptedTick = m_acceptedTick;
                d.acceptedSourceNowUs = m_acceptedSourceNowUs; d.acceptedCsv = m_acceptedCsv[axis];
                d.flags |= FaultAxis_ACCEPTED_IMAGE;
            }
        }
        bool RejectArm(Reason reason) noexcept
        { m_feedback.reason = reason; m_feedback.faulted = true; m_feedback.state = State::Faulted; return false; }
        void ResetStopEvidence() noexcept
        {
            m_feedback.settledCycles = 0U;
            m_feedback.stopped = false;
            m_feedback.pass = false;
            m_haveStableRange = false;
        }
        void ObserveStableRange(const Frame& frame) noexcept
        {
            bool stable = m_haveStableRange;
            std::uint32_t exceeded = 0U;
            double spans[AxisCount]{};
            for (unsigned axis = 0U; axis < AxisCount; ++axis)
            {
                const double value = frame.axis[axis].actualPulse;
                const double limit = StopRangeLimitPulse(axis, m_feedback.config.axis[axis]);
                if (!m_haveStableRange) m_stableMin[axis] = m_stableMax[axis] = value;
                else
                {
                    if (value < m_stableMin[axis]) m_stableMin[axis] = value;
                    if (value > m_stableMax[axis]) m_stableMax[axis] = value;
                    if (m_stableMax[axis] - m_stableMin[axis] > limit) stable = false;
                    spans[axis] = m_stableMax[axis] - m_stableMin[axis];
                    if (spans[axis] > limit) exceeded |= 1U << axis;
                }
            }
            if (exceeded != 0U)
            {
                m_feedback.stop.blockedMask |= StopBlock_ENCODER_RANGE;
                m_feedback.stop.rangeExceededMask = exceeded;
                m_feedback.stop.lastRangeResetTick = frame.tick;
                if (m_feedback.stop.rangeResetCount != (std::numeric_limits<std::uint32_t>::max)())
                    ++m_feedback.stop.rangeResetCount;
                for (unsigned axis = 0U; axis < AxisCount; ++axis) m_feedback.stop.rangePulse[axis] = spans[axis];
            }
            if (!stable)
            {
                ResetStopEvidence();
                for (unsigned axis = 0U; axis < AxisCount; ++axis)
                    m_stableMin[axis] = m_stableMax[axis] = frame.axis[axis].actualPulse;
            }
            m_haveStableRange = true;
        }

        AxisControlSample m_axisControl[AxisCount]{};
        Identity m_acceptedIdentity{};
        std::uint64_t m_acceptedTick = 0ULL, m_acceptedSourceNowUs = 0ULL;
        std::int32_t m_acceptedCsv[AxisCount]{};
        RuntimeDiagnostic m_runtimeDiagnostic{};
        Feedback m_feedback{};
        Frame m_lastFrame{};
        Identity m_sealedIdentity{};
        std::int32_t m_sealedCsv[AxisCount]{};
        double m_stableMin[AxisCount]{};
        double m_stableMax[AxisCount]{};
        bool m_pendingSeal = false;
        bool m_sealed = false;
        std::uint64_t m_discardedTick = 0ULL;
        bool m_stopEligible = false;
        bool m_haveStableRange = false;
    };
}
