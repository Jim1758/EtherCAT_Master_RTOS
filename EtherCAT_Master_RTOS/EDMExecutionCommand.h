#pragma once

#include <cmath>
#include <cstdint>

// EDM50: bounded, stateless finite-command conversion in one axis's units.
// The backend owns scope, source freshness, authority and execution. This
// helper neither authorizes motion nor produces an application/stop receipt.
// Production binds linear Z only; explicit units never mix linear/rotary norms.
namespace EDM50
{
    enum class Backend : std::uint8_t { None, LocalShadow, RTShadow, LocalRTMotion };
    enum class AxisUnit : std::uint8_t { None, Millimeter, Degree };
    enum class Reason : std::uint8_t
    {
        None, Domain, Axis, Unit, Scale, InvalidRule, InvalidTarget,
        TargetBound, NativeRange, Direction, NativeSpeed
    };
    struct AxisBinding
    {
        Backend backend=Backend::None;
        AxisUnit unit=AxisUnit::None;
        std::uint8_t axisIndex=255U;
        double nativePerUnit=0.0;
    };
    struct TargetRequest
    {
        AxisBinding axis{};
        double firstOriginNative=0.0, relativeTargetUnits=0.0;
        double targetHalfUnits=0.0, outerHalfUnits=0.0;
    };
    struct TargetResult
    {
        Reason reason=Reason::None;
        double nativeTarget=0.0;
        bool ready=false;
        bool Ready() const noexcept { return ready; }
    };
    struct CurveMotionRequest
    {
        AxisBinding axis{};
        double nativeTarget=0.0;
        double actualNative=0.0, commandNative=0.0, planningNative=0.0;
        double positionToleranceUnits=0.0;
        int nativeDirection=0;
        double positiveRateUnitsPerMinute=0.0;
        double maximumNativeRate=0.0, maximumOutputUnitsPerSecond=0.0;
        double accelerationTimeSec=0.0, decelerationTimeSec=0.0;
    };
    struct CurveMotionResult
    {
        Reason reason=Reason::None;
        double nativeVelocity=0.0;
        bool ready=false;
        bool Ready() const noexcept { return ready; }
    };
    inline bool Native(double value) noexcept
    { return std::isfinite(value) && std::fabs(value)<=2147483647.0; }
    inline Reason ValidateAxis(const AxisBinding& axis) noexcept
    {
        if(axis.backend!=Backend::LocalRTMotion) return Reason::Domain;
        if(axis.axisIndex>=8U) return Reason::Axis;
        if(axis.unit!=AxisUnit::Millimeter && axis.unit!=AxisUnit::Degree) return Reason::Unit;
        if(!std::isfinite(axis.nativePerUnit) || axis.nativePerUnit<=0.0) return Reason::Scale;
        return Reason::None;
    }
    // Arithmetic only: legacy rates already passed the frozen Arm contract.
    // Keep this order, including multiplication before division, bit-exact.
    inline double ConvertRate(double positiveRateUnitsPerMinute,double nativePerUnit) noexcept
    { return positiveRateUnitsPerMinute*nativePerUnit/60.0; }
    inline TargetResult TranslateTarget(const TargetRequest& request) noexcept
    {
        TargetResult result{};
        const auto reject=[&result](Reason reason) noexcept -> TargetResult
        { result.reason=reason; return result; };
        const Reason axis=ValidateAxis(request.axis);
        if(axis!=Reason::None) return reject(axis);
        if(!std::isfinite(request.targetHalfUnits) || request.targetHalfUnits<=0.0 ||
            !std::isfinite(request.outerHalfUnits) || request.outerHalfUnits<=request.targetHalfUnits)
            return reject(Reason::InvalidRule);
        if(!Native(request.firstOriginNative)) return reject(Reason::NativeRange);
        if(!std::isfinite(request.relativeTargetUnits)) return reject(Reason::InvalidTarget);
        if(std::fabs(request.relativeTargetUnits)>request.targetHalfUnits) return reject(Reason::TargetBound);
        const double outer=request.outerHalfUnits*request.axis.nativePerUnit;
        const double relative=request.relativeTargetUnits*request.axis.nativePerUnit;
        const double target=request.firstOriginNative+relative;
        if(!std::isfinite(outer) || outer<=0.0 ||
            (request.relativeTargetUnits!=0.0 && relative==0.0) ||
            !Native(target) || std::fabs(target-request.firstOriginNative)>outer)
            return reject(Reason::NativeRange);
        result.nativeTarget=target; result.ready=true; return result;
    }
    inline CurveMotionResult EvaluateCurveMotion(const CurveMotionRequest& request) noexcept
    {
        CurveMotionResult result{};
        const auto reject=[&result](Reason reason) noexcept -> CurveMotionResult
        { result.reason=reason; return result; };
        const Reason axis=ValidateAxis(request.axis);
        if(axis!=Reason::None) return reject(axis);
        if(!Native(request.nativeTarget) || !Native(request.actualNative) ||
            !Native(request.commandNative) || !Native(request.planningNative)) return reject(Reason::NativeRange);
        if(!std::isfinite(request.positionToleranceUnits) || request.positionToleranceUnits<=0.0 ||
            !std::isfinite(request.maximumNativeRate) || request.maximumNativeRate<=0.0 ||
            !std::isfinite(request.maximumOutputUnitsPerSecond) || request.maximumOutputUnitsPerSecond<=0.0 ||
            !std::isfinite(request.accelerationTimeSec) || request.accelerationTimeSec<=0.0 ||
            !std::isfinite(request.decelerationTimeSec) || request.decelerationTimeSec<=0.0)
            return reject(Reason::InvalidRule);
        const double tolerance=request.positionToleranceUnits*request.axis.nativePerUnit;
        const double cap=request.maximumOutputUnitsPerSecond*request.axis.nativePerUnit;
        if(!std::isfinite(tolerance) || tolerance<=0.0 || !std::isfinite(cap) || cap<=0.0)
            return reject(Reason::InvalidRule);
        if(request.nativeDirection!=-1 && request.nativeDirection!=1) return reject(Reason::Direction);
        const double direction=static_cast<double>(request.nativeDirection);
        if((request.nativeTarget-request.actualNative)*direction<=tolerance ||
            (request.nativeTarget-request.commandNative)*direction<=tolerance ||
            (request.nativeTarget-request.planningNative)*direction<=tolerance) return reject(Reason::Direction);
        if(!std::isfinite(request.positiveRateUnitsPerMinute) || request.positiveRateUnitsPerMinute<=0.0)
            return reject(Reason::NativeSpeed);
        const double velocity=ConvertRate(request.positiveRateUnitsPerMinute,request.axis.nativePerUnit);
        const double acceleration=velocity/request.accelerationTimeSec;
        const double deceleration=velocity/request.decelerationTimeSec;
        if(!Native(velocity) || velocity<=1.0 || velocity>request.maximumNativeRate || velocity>cap ||
            !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
            acceleration<=10.0 || deceleration<=10.0)
            return reject(Reason::NativeSpeed);
        result.nativeVelocity=velocity; result.ready=true; return result;
    }
}
