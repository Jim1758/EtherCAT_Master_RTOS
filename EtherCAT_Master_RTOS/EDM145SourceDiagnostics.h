#pragma once
#include <cstdint>
#include <limits>
#include <type_traits>

// First failed send admission only. This receipt is diagnostic evidence and
// never grants authority, extends a deadline, or changes source readiness.
namespace EDM145
{
    enum AwaitGate : std::uint32_t
    {
        AwaitNone = 0U, AwaitProfile, AwaitState, AwaitMotion, AwaitFrame,
        AwaitEpoch, AwaitPhysical, AwaitTick, AwaitBinding, AwaitClockOrder,
        AwaitCaptureAge, AwaitObservationAge, AwaitAxisCount, AwaitCommand
    };
    inline const char* AwaitGateName(std::uint32_t gate) noexcept
    {
        switch (gate)
        {
        case AwaitNone: return "NONE";
        case AwaitProfile: return "PROFILE";
        case AwaitState: return "STATE";
        case AwaitMotion: return "MOTION";
        case AwaitFrame: return "FRAME";
        case AwaitEpoch: return "EPOCH";
        case AwaitPhysical: return "PHYSICAL";
        case AwaitTick: return "TICK";
        case AwaitBinding: return "BINDING";
        case AwaitClockOrder: return "CLOCK_ORDER";
        case AwaitCaptureAge: return "CAPTURE_AGE";
        case AwaitObservationAge: return "OBSERVATION_AGE";
        case AwaitAxisCount: return "AXIS_COUNT";
        case AwaitCommand: return "COMMAND";
        default: return "UNKNOWN";
        }
    }
    enum DecisionFlag : std::uint32_t
    {
        SourceCurrent = 1U, AwaitEvaluated = 2U, AwaitCurrent = 4U, MappingCurrent = 8U
    };
    enum StateFlag : std::uint32_t
    {
        SourceReady = 1U, PhysicalActive = 2U, PhysicalReady = 4U,
        PhysicalFaulted = 8U, RowActive = 16U, GroupActive = 32U,
        FrameZero = 64U, FrameIdle = 128U, FrameAuthority = 256U,
        HandoffPending = 512U, HandoffCommitted = 1024U, Participating = 2048U
    };
    struct SourceFailure
    {
        std::uint64_t session=0, tick=0, sendNowUs=0, frameNowUs=0, physicalNowUs=0;
        std::uint64_t sourceCapturedUs=0, baselineCapturedUs=0, baselineObservedUs=0, admittedUs=0;
        std::uint64_t sourceMaxAgeUs=0, baselineMaxAgeUs=0, shadowMaxAgeUs=0, shadowMaxTickGapUs=0;
        std::uint64_t expectedEpoch=0, liveEpoch=0, observedEpoch=0;
        std::uint64_t sourcePublication=0, sourceCycle=0, sourceFresh=0, sourceHeld=0;
        std::uint32_t awaitFailure=AwaitNone, decisionFlags=0, stateFlags=0;
        std::uint32_t positionMismatchMask=0, velocityMismatchMask=0, physicalReason=0;
    };
    inline std::uint64_t AgeUs(std::uint64_t now, std::uint64_t captured) noexcept
    {
        return captured && now>=captured ? now-captured : (std::numeric_limits<std::uint64_t>::max)();
    }
    bool ReadSourceFailure(SourceFailure& output) noexcept;
    static_assert(std::is_trivially_copyable<SourceFailure>::value, "EDM145 evidence is copied by value");
    static_assert(sizeof(SourceFailure)<=256U, "EDM145 first failure receipt must remain bounded");
}
