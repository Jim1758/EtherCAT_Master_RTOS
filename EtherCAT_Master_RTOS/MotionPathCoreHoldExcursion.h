#pragma once
#include <cstdint>
#include <type_traits>
#include "MotionExecutionContract.h"
#include "NCPathCoreRetainedPath.h"

// CD: completed geometry is copied at source binding before excursion admission. No old
// execution authority crosses this fixed producer/RT boundary.
struct MotionPathCoreHoldExcursionView
{
    std::array<NCPathCoreRetainedGeometry, 16U> completed{};
    NCPathCoreRetainedGeometry original{};
    std::uint32_t completedCount = 0U, validAxisMask = 0U;
    std::array<double, 8U> pulsePerMM{};
    std::uint64_t translationGeneration = 0ULL;
};
static_assert(std::is_trivially_copyable<MotionPathCoreHoldExcursionView>::value,
    "CD immutable view must remain fixed POD.");

// CB: producer/RT control metadata only; MotionCommand and shared-memory ABI stay unchanged.
enum class MotionPathCoreHoldExcursionPhase : std::uint32_t
{
    IDLE, ARMED, RETREATING, RETURNING, COMPLETE, REJECTED, INVALIDATED, WAIT_RETURN
};
// CU_FIX1: diagnostics describe a computed holding correction, never a wire proof.
enum class MotionPathCoreAdmissionCorrectionDecision : std::uint8_t
{
    NOT_APPLICABLE = 0U, BLOCKED = 1U, AUTHORIZED = 2U
};
enum class MotionPathCoreAdmissionCorrectionReason : std::uint8_t
{
    NONE = 0U, STALE_RUNTIME, AUTHORITY, SOURCE_IDENTITY, AXIS_SCOPE,
    AXIS_STATE, RAW_SERVO, COMMAND_CHANGED, NUMERIC, TRAVEL_LIMIT
};
struct MotionPathCoreHoldExcursionSnapshot
{
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    std::uint64_t translationGeneration = 0ULL;
    MotionPathCoreHoldExcursionPhase phase = MotionPathCoreHoldExcursionPhase::IDLE;
    std::uint32_t reason = 0U;
    std::uint32_t cycleLimit = 1U;
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t transitionSequence = 0ULL;
    std::uint64_t holdRequestSequence = 0ULL;
    std::uint64_t completedHoldRequestSequence = 0ULL;
    std::uint64_t requestGeneration = 0ULL;
    std::uint64_t retreatCount = 0ULL;
    std::uint64_t returnCount = 0ULL;
    double heldS = 0.0;
    double retreatS = 0.0;
    double returnedS = 0.0;
    double lengthPulse = 0.0;
    double distanceMM = 0.0;
    double feedMMMin = 0.0;
    std::uint32_t historyCount = 0U, activeOrdinal = 0U, retreatOrdinal = 0U;
    std::uint64_t seamCount = 0ULL;
    double activeS = 0.0, retreatLocalS = 0.0;
    bool crossSegment = false;
    bool ready = false;
    bool boundaryOnly = false;
    bool requireReturnAuthorization = false;
    // CR_FIX2: live queued admission evidence; activeS is invalid while pending.
    bool admissionPending = false;
    std::uint64_t admissionWaitTick = 0ULL;
    std::int32_t admissionWaitAxis = -1;
    double admissionFollowingError = 0.0, admissionWindowPulse = 0.0;
    // Internal POD publication grows; rebuild every consumer together.
    // One diagnostic slot follows admissionWaitAxis; a changed axis resets it.
    std::uint64_t admissionCorrectionTick = 0ULL;
    std::uint32_t admissionCorrectionCycles = 0U;
    std::int32_t admissionCorrectionAxis = -1;
    std::uint32_t admissionCorrectionMask = 0U;
    MotionPathCoreAdmissionCorrectionDecision admissionCorrectionDecision =
        MotionPathCoreAdmissionCorrectionDecision::NOT_APPLICABLE;
    MotionPathCoreAdmissionCorrectionReason admissionCorrectionReason =
        MotionPathCoreAdmissionCorrectionReason::NONE;
    double admissionCorrectionFirstIntegral = 0.0;
    double admissionCorrectionKp = 0.0, admissionCorrectionVelocityPPS = 0.0;
};
static_assert(std::is_trivially_copyable<MotionPathCoreHoldExcursionSnapshot>::value,
    "CB publication must remain fixed POD.");
