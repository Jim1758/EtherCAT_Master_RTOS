#pragma once

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.6 - MotionCommand-local G00 Path-Mode Transport Shadow
//
// BufferMode describes how a command enters the transport.  This enum describes
// the terminal path policy carried by that individual command.  Keeping the two
// dimensions independent is required for a future BUFFERED + EXACT_STOP G00.
//
// UNSPECIFIED deliberately preserves the accepted legacy group-global policy
// for every command family outside the K.6 G00 envelope.  PATH_SERVO and
// JUMP_TRACKING remain group/runtime driver modes and can never enter this
// command-local transport enum.
// =============================================================================
enum class MotionCommandPathMode : std::uint8_t
{
    UNSPECIFIED = 0U,
    EXACT_STOP = 1U,
    CONTINUOUS = 2U
};

constexpr std::uint64_t MOTION_COMMAND_PATH_MODE_FINGERPRINT_SEED =
1469598103934665603ULL;

inline bool IsValidMotionCommandPathMode(
    MotionCommandPathMode mode) noexcept
{
    return
        mode == MotionCommandPathMode::UNSPECIFIED ||
        mode == MotionCommandPathMode::EXACT_STOP ||
        mode == MotionCommandPathMode::CONTINUOUS;
}

inline bool IsExplicitMotionCommandPathMode(
    MotionCommandPathMode mode) noexcept
{
    return
        mode == MotionCommandPathMode::EXACT_STOP ||
        mode == MotionCommandPathMode::CONTINUOUS;
}

// Stage NC-0.2K.6.1: bounded decision used by the 250 us Consumer before it
// commits a MotionCommand into the active planner state.  The decision is
// intentionally independent of PathMode so this transport contract does not
// depend on MotionCore declaration order.
enum class MotionCommandPathModeAuthorityDecision : std::uint8_t
{
    LEGACY_FALLBACK = 0U,
    APPLY_EXACT_STOP,
    APPLY_CONTINUOUS,
    REPLAY_BYPASS,
    REJECT_DRIVER_OVERRIDE,
    REJECT_INVALID
};

inline MotionCommandPathModeAuthorityDecision
ResolveMotionCommandPathModeAuthorityDecision(
    MotionCommandPathMode mode,
    bool replayTransportCopy,
    bool driverOverrideActive) noexcept
{
    if (!IsValidMotionCommandPathMode(mode))
    {
        return MotionCommandPathModeAuthorityDecision::REJECT_INVALID;
    }

    // B2 owns replay execution semantics.  A replayed terminal copy carries
    // the byte for transport integrity, but K.6.1 must never replace
    // JUMP_TRACKING/PATH_SERVO authority with a normal G00 policy.
    if (replayTransportCopy)
    {
        return MotionCommandPathModeAuthorityDecision::REPLAY_BYPASS;
    }

    // UNSPECIFIED is the compatibility envelope for every command family not
    // migrated to command-local authority yet.  It preserves the accepted
    // runtime policy, including an intentional driver mode.
    if (mode == MotionCommandPathMode::UNSPECIFIED)
    {
        return MotionCommandPathModeAuthorityDecision::LEGACY_FALLBACK;
    }

    // A new ingress G00 may not silently steal authority from PATH_SERVO or
    // JUMP_TRACKING.  MotionCore consumes and rejects it through the existing
    // Alarm 3021 fail-closed transaction.
    if (driverOverrideActive)
    {
        return
            MotionCommandPathModeAuthorityDecision::REJECT_DRIVER_OVERRIDE;
    }

    return
        mode == MotionCommandPathMode::EXACT_STOP
        ? MotionCommandPathModeAuthorityDecision::APPLY_EXACT_STOP
        : MotionCommandPathModeAuthorityDecision::APPLY_CONTINUOUS;
}

inline const char* MotionCommandPathModeToDiagnosticName(
    MotionCommandPathMode mode) noexcept
{
    switch (mode)
    {
    case MotionCommandPathMode::EXACT_STOP:
        return "EXACT";
    case MotionCommandPathMode::CONTINUOUS:
        return "CONT";
    case MotionCommandPathMode::UNSPECIFIED:
        return "UNSPEC";
    default:
        return "INVALID";
    }
}

struct MotionCommandPathModeTransportSnapshot
{
    std::uint64_t producerAccepted = 0ULL;
    std::uint64_t producerRejected = 0ULL;
    std::uint64_t producerExactStop = 0ULL;
    std::uint64_t producerContinuous = 0ULL;
    std::uint64_t producerUnspecified = 0ULL;
    std::uint64_t producerInvalid = 0ULL;
    std::uint64_t producerFingerprint =
        MOTION_COMMAND_PATH_MODE_FINGERPRINT_SEED;

    std::uint64_t consumerCommitted = 0ULL;
    std::uint64_t consumerIngressCommitted = 0ULL;
    std::uint64_t consumerReplayCommitted = 0ULL;
    std::uint64_t consumerExactStop = 0ULL;
    std::uint64_t consumerContinuous = 0ULL;
    std::uint64_t consumerUnspecified = 0ULL;
    std::uint64_t consumerInvalid = 0ULL;
    std::uint64_t consumerFingerprint =
        MOTION_COMMAND_PATH_MODE_FINGERPRINT_SEED;

    std::uint64_t legacyModeMatches = 0ULL;
    std::uint64_t legacyModeMismatches = 0ULL;
    std::uint64_t driverOverrideObservations = 0ULL;

    std::uint64_t authorityAttempts = 0ULL;
    std::uint64_t authorityApplied = 0ULL;
    std::uint64_t authorityExactStop = 0ULL;
    std::uint64_t authorityContinuous = 0ULL;
    std::uint64_t authorityLegacyFallbacks = 0ULL;
    std::uint64_t authorityReplayBypasses = 0ULL;
    std::uint64_t authorityDriverBlocks = 0ULL;
    std::uint64_t authorityInvalidRejects = 0ULL;

    bool commandLocalPayloadPresent = false;
    bool transportReady = false;
    bool shadowOnly = false;
    bool consumerAuthority = true;
    bool runtimeInfluence = false;
    bool cutoverAttempted = false;
    bool producerSnapshotCoherent = true;
    bool consumerSnapshotCoherent = true;
    bool accountingValid = true;
};

static_assert(
    sizeof(MotionCommandPathMode) == sizeof(std::uint8_t),
    "Command-local path mode must remain a one-byte transport field.");
static_assert(
    std::is_trivially_copyable<
    MotionCommandPathModeTransportSnapshot>::value,
    "K.6.1 transport diagnostics must remain trivially copyable.");
