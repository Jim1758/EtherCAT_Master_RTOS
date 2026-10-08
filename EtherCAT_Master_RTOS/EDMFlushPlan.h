#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// EDM20 pure, bounded logical planning only. There is deliberately no
// MotionCore, I/O, discharge-permit, position, trajectory, or PID mutation here.
namespace EDM20
{
    struct FlushProfile
    {
        // Native units are explicit: all distances use mm, all speeds use mm/min.
        double finalApproachMmPerMin = 0.5; // Pump_Stg_Spd
        double centerRetractMmPerMin = 2.5; // Pump_1_Spd (B3/B4)
        double mainRetractMmPerMin = 5.0;   // Pump_2_Spd
        double centerReturnMmPerMin = 2.5;  // Pump_3_Spd (B3/B4)
        double mainReturnMmPerMin = 5.0;    // Pump_4_Spd
        double pathRetractMmPerMin = 1.0;   // Pump_5_Spd
        double initialSlowMmPerMin = 1.0;   // Pump_Low_Spd, B1 only
        // Pump_Hand_Spd is manual Jog Ref3 -> Ref2 recovery, not HOLD speed.
        double manualReturnMmPerMin = 1.0;
        double finalApproachDistanceMm = 0.020; // Pump_Std_Dist
        double pathDistanceMm = 0.0;           // Pump_Path_Dist
        // Distance not supplied by the machine owner: zero disables it.
        double initialSlowDistanceMm = 0.0;
    };

    enum class FlushMode : std::uint8_t
    {
        B0 = 0, // Opposite / along machining direction.
        B1 = 1, // Machine-frame Z+ retreat / Z- return.
        B2 = 2, // Retrace already executed path; geometry not implemented here.
        B3 = 3, // Orbit center then machining-axis retreat; geometry required.
        B4 = 4  // 45-degree center approach then axis retreat; geometry required.
    };

    enum class FlushDirection : std::uint8_t
    { OppositeMachining = 0, AlongMachining, MachinePositiveZ, MachineNegativeZ };
    enum class FlushLeg : std::uint8_t { Retreat = 0, Return = 1 };
    enum class FlushSegmentKind : std::uint8_t
    { InitialSlowRetreat = 0, MainRetreat, MainReturn, FinalApproach };

    struct FlushFrameInfo
    {
        // These are prerequisites for interpreting the logical direction enum,
        // not proofs that a physical trajectory is safe or admitted.
        bool machiningDirectionConfirmed = false;
        bool machineZDirectionConfirmed = false;
    };

    struct FlushRequest
    {
        FlushMode mode = FlushMode::B0;
        // E6 raw units have not been established. A caller must convert from a
        // confirmed unit and explicitly acknowledge it before requesting a plan.
        double jumpHeightMm = 0.0;
        bool jumpHeightConfirmed = false;
        double speedOverridePercent = 100.0;
        // E11 is not automatically interpreted as percent. Only an explicitly
        // confirmed percentage is multiplied when useFinalOverride is true.
        bool useFinalOverride = false;
        double finalOverridePercent = 100.0;
        FlushFrameInfo frame{};
    };

    struct FlushSegment
    {
        FlushSegmentKind kind = FlushSegmentKind::MainRetreat;
        FlushLeg leg = FlushLeg::Retreat;
        FlushDirection direction = FlushDirection::OppositeMachining;
        double distanceMm = 0.0;
        double speedMmPerMin = 0.0;
        // Reserved for a future confirmed path-return plan. B0/B1 path=0
        // segments currently produced here have fixed profile speed only.
        bool requiresGapServo = false;
    };

    constexpr std::size_t MaximumFlushSegments = 8U;
    struct FlushPlan
    {
        std::array<FlushSegment, MaximumFlushSegments> segments{};
        std::size_t count = 0U;
        double totalRetreatMm = 0.0;
        double totalReturnMm = 0.0;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    };

    enum class FlushPlanStatus : std::uint8_t
    {
        ReadyLogicalOnly = 0,
        InvalidProfile,
        InvalidRequest,
        UnconfirmedJumpUnits,
        NeedsAxisFrame,
        NeedsPathGeometry,
        NeedsOrbitGeometry,
        PausedByOverride,
        SpeedNotRepresentable
    };

    struct FlushPlanResult
    {
        FlushPlanStatus status = FlushPlanStatus::InvalidRequest;
        FlushPlan plan{};
    };

    inline bool ValidateFlushProfile(const FlushProfile& value) noexcept
    {
        const double speeds[] = { value.finalApproachMmPerMin,
            value.centerRetractMmPerMin, value.mainRetractMmPerMin,
            value.centerReturnMmPerMin, value.mainReturnMmPerMin,
            value.pathRetractMmPerMin, value.initialSlowMmPerMin,
            value.manualReturnMmPerMin };
        for (double speed : speeds)
            if (!std::isfinite(speed) || speed <= 0.0) return false;
        const double distances[] = { value.finalApproachDistanceMm,
            value.pathDistanceMm, value.initialSlowDistanceMm };
        for (double distance : distances)
            if (!std::isfinite(distance) || distance < 0.0) return false;
        return true;
    }

    namespace FlushPlanDetail
    {
        inline bool Percent(double value) noexcept
        { return std::isfinite(value) && value >= 0.0 && value <= 100.0; }

        inline bool Append(FlushPlan& plan, FlushSegmentKind kind, FlushLeg leg,
            FlushDirection direction, double distance, double speed) noexcept
        {
            if (distance == 0.0) return true; // Never emit zero-length stages.
            if (plan.count >= MaximumFlushSegments || !std::isfinite(distance) ||
                distance < 0.0 || !std::isfinite(speed) || speed <= 0.0) return false;
            FlushSegment& segment = plan.segments[plan.count++];
            segment.kind = kind; segment.leg = leg; segment.direction = direction;
            segment.distanceMm = distance; segment.speedMmPerMin = speed;
            return true;
        }
    }

    inline FlushPlanResult BuildFlushPlan(const FlushProfile& profile,
        const FlushRequest& request) noexcept
    {
        FlushPlanResult result{};
        if (!ValidateFlushProfile(profile))
        { result.status = FlushPlanStatus::InvalidProfile; return result; }
        if (!std::isfinite(request.jumpHeightMm) || request.jumpHeightMm < 0.0 ||
            !FlushPlanDetail::Percent(request.speedOverridePercent) ||
            !FlushPlanDetail::Percent(request.finalOverridePercent) ||
            static_cast<unsigned>(request.mode) > static_cast<unsigned>(FlushMode::B4))
            return result;
        if (!request.jumpHeightConfirmed)
        { result.status = FlushPlanStatus::UnconfirmedJumpUnits; return result; }
        // A flag is not trajectory geometry. B2/B3/B4 have no implementation
        // in this phase; no fictitious straight-line substitute is generated.
        if (request.mode == FlushMode::B3 || request.mode == FlushMode::B4)
        { result.status = FlushPlanStatus::NeedsOrbitGeometry; return result; }
        // Nonzero Pump_Path_Dist needs the actual executed path, even in B0/B1.
        // We do not guess whether that distance is within/additional to jump.
        if (request.mode == FlushMode::B2 || profile.pathDistanceMm > 0.0)
        { result.status = FlushPlanStatus::NeedsPathGeometry; return result; }
        const bool isB1 = request.mode == FlushMode::B1;
        if (isB1 ? !request.frame.machineZDirectionConfirmed :
            !request.frame.machiningDirectionConfirmed)
        { result.status = FlushPlanStatus::NeedsAxisFrame; return result; }

        // Partition each leg independently. Even when the jump is shorter than
        // either configured slow distance, each leg covers exactly jumpHeight.
        const double height = request.jumpHeightMm;
        const double slow = isB1 && profile.initialSlowDistanceMm < height ?
            profile.initialSlowDistanceMm : (isB1 ? height : 0.0);
        const double approach = profile.finalApproachDistanceMm < height ?
            profile.finalApproachDistanceMm : height;
        if (height == 0.0)
        { result.status = FlushPlanStatus::ReadyLogicalOnly; return result; }
        if (request.speedOverridePercent == 0.0 ||
            (request.useFinalOverride && approach > 0.0 && request.finalOverridePercent == 0.0))
        { result.status = FlushPlanStatus::PausedByOverride; return result; }

        const double ratio = request.speedOverridePercent / 100.0;
        const double finalRatio = request.useFinalOverride ?
            request.finalOverridePercent / 100.0 : 1.0;
        const FlushDirection retreat = isB1 ? FlushDirection::MachinePositiveZ :
            FlushDirection::OppositeMachining;
        const FlushDirection returnDirection = isB1 ? FlushDirection::MachineNegativeZ :
            FlushDirection::AlongMachining;
        FlushPlan candidate{};
        const bool ok = FlushPlanDetail::Append(candidate,
                FlushSegmentKind::InitialSlowRetreat, FlushLeg::Retreat,
                retreat, slow, profile.initialSlowMmPerMin * ratio) &&
            FlushPlanDetail::Append(candidate, FlushSegmentKind::MainRetreat,
                FlushLeg::Retreat, retreat, height - slow, profile.mainRetractMmPerMin * ratio) &&
            FlushPlanDetail::Append(candidate, FlushSegmentKind::MainReturn,
                FlushLeg::Return, returnDirection, height - approach, profile.mainReturnMmPerMin * ratio) &&
            FlushPlanDetail::Append(candidate, FlushSegmentKind::FinalApproach,
                FlushLeg::Return, returnDirection, approach,
                profile.finalApproachMmPerMin * ratio * finalRatio);
        if (!ok)
        { result.status = FlushPlanStatus::SpeedNotRepresentable; return result; }
        candidate.totalRetreatMm = candidate.totalReturnMm = height;
        result.plan = candidate;
        result.status = FlushPlanStatus::ReadyLogicalOnly;
        return result;
    }

    inline const char* FlushPlanStatusName(FlushPlanStatus value) noexcept
    {
        switch (value)
        {
        case FlushPlanStatus::ReadyLogicalOnly: return "READY_LOGICAL_ONLY";
        case FlushPlanStatus::InvalidProfile: return "INVALID_PROFILE";
        case FlushPlanStatus::InvalidRequest: return "INVALID_REQUEST";
        case FlushPlanStatus::UnconfirmedJumpUnits: return "UNCONFIRMED_JUMP_UNITS";
        case FlushPlanStatus::NeedsAxisFrame: return "NEEDS_AXIS_FRAME";
        case FlushPlanStatus::NeedsPathGeometry: return "NEEDS_PATH_GEOMETRY";
        case FlushPlanStatus::NeedsOrbitGeometry: return "NEEDS_ORBIT_GEOMETRY";
        case FlushPlanStatus::PausedByOverride: return "PAUSED_BY_OVERRIDE";
        case FlushPlanStatus::SpeedNotRepresentable: return "SPEED_NOT_REPRESENTABLE";
        }
        return "UNKNOWN";
    }
}
