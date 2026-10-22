#pragma once

#include <cstdint>
#include <type_traits>

// Diagnostic receipt only. Neither this snapshot nor its publication can
// authorize an arm, a mapped command, a physical completion or a PDO write.
namespace EDM192Arm
{
    enum class Stage : std::uint32_t
    { None=0U, Consumed=1U, RequestShapeRejected=2U, HeadroomRejected=3U, EngineRejected=4U, Accepted=5U };
    enum class Cause : std::uint32_t
    { None=0U, RequestShape=1U, ContextUnavailable=2U, HeadroomCheck=3U, PhysicalGuard=4U, RecoveryProbe=5U };
    enum class Pid : std::uint32_t { None=0U, G00=1U, Flush=2U };
    static constexpr unsigned NoAxis = 6U;

    struct Snapshot
    {
        std::uint64_t session, armEpoch, liveEpoch, tick, nowUs;
        std::uint32_t owner, ownerGeneration, fixture, profile, axis, engineReason, requestFailures;
        Stage stage;
        Cause cause;
        Pid pid;
        double pulsePerUnit, followingWindowPulse, maxVelocityPps, serializedCapPps;
        double kp, ki, kd, kvff, maxIntegral;
    };
    struct Scope
    {
        std::uint64_t session, armEpoch, liveEpoch;
        std::uint32_t owner, ownerGeneration, fixture, profile;
    };
    static_assert(std::is_trivial<Snapshot>::value && std::is_standard_layout<Snapshot>::value,
        "Arm diagnostics must remain a plain snapshot");
    static_assert(std::is_trivially_copyable<Snapshot>::value && sizeof(Snapshot)<=256U,
        "Arm diagnostics must remain bounded and copyable");

    inline bool ValidStage(Stage stage) noexcept
    { return stage>=Stage::Consumed && stage<=Stage::Accepted; }
    inline bool IsReject(Stage stage) noexcept
    { return stage==Stage::RequestShapeRejected || stage==Stage::HeadroomRejected || stage==Stage::EngineRejected; }
    inline bool MatchesScope(const Snapshot& value,const Scope& scope) noexcept
    {
        return ValidStage(value.stage) && value.tick && value.nowUs && scope.session && scope.armEpoch &&
            scope.liveEpoch==scope.armEpoch && scope.owner && scope.ownerGeneration &&
            value.session==scope.session && value.armEpoch==scope.armEpoch && value.liveEpoch==scope.liveEpoch &&
            value.owner==scope.owner && value.ownerGeneration==scope.ownerGeneration;
    }
    inline bool MatchesRecipe(const Snapshot& value,const Scope& scope) noexcept
    { return value.fixture==scope.fixture && value.profile==scope.profile; }
    inline bool IsNewPublication(std::uint64_t publication,std::uint64_t floor,std::uint64_t last) noexcept
    { return publication && publication>floor && publication>last; }
    inline const char* StageName(Stage stage) noexcept
    {
        switch(stage) {
        case Stage::Consumed:return "CONSUMED";
        case Stage::RequestShapeRejected:return "REQUEST_SHAPE_REJECTED";
        case Stage::HeadroomRejected:return "HEADROOM_REJECTED";
        case Stage::EngineRejected:return "ENGINE_REJECTED";
        case Stage::Accepted:return "ACCEPTED";
        default:return "NONE";
        }
    }
    inline const char* CauseName(Cause cause) noexcept
    {
        switch(cause) {
        case Cause::RequestShape:return "REQUEST_SHAPE";
        case Cause::ContextUnavailable:return "CONTEXT_UNAVAILABLE";
        case Cause::HeadroomCheck:return "HEADROOM_CHECK";
        case Cause::PhysicalGuard:return "PHYSICAL_GUARD";
        case Cause::RecoveryProbe:return "RECOVERY_PROBE";
        default:return "NONE";
        }
    }
    inline const char* PidName(Pid pid) noexcept
    { return pid==Pid::G00?"G00":pid==Pid::Flush?"FLUSH":"NONE"; }

    // One Motion writer, existing EDM53 channel, NC diagnostic reader.
    bool Read(Snapshot& snapshot,std::uint64_t& publication) noexcept;
}
