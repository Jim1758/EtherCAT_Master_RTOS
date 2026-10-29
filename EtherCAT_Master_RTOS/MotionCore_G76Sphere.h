#pragma once
#include "NCG76SphereCycle.h"
#include "MotionExecutionContract.h"
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>

// EDM286 FIX1: a stationary rotary axis owns two representations: the NC
// native spelling (which may be modulo-reduced) and its accepted raw pulse.
// Prove the pair once; never reconstruct a stationary command from the angle.
namespace MotionG76SphereFixedAxes
{
inline bool SameBits(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
inline bool MatchesNative(double native,double pulse,double resolution,double lead,
    bool moduloAxis,double modulo) noexcept
{
    if(!std::isfinite(native)||!std::isfinite(pulse)||!std::isfinite(resolution)||resolution<=0.0||
        !std::isfinite(lead)||lead<=0.0||(moduloAxis&&(!std::isfinite(modulo)||modulo<=0.0)))return false;
    const double ppu=resolution/lead,rawNative=pulse*lead/resolution,forward=native*ppu;
    if(!std::isfinite(ppu)||ppu<=0.0||!std::isfinite(rawNative)||!std::isfinite(forward))return false;
    if(forward==pulse||native==rawNative)return true;
    if(!moduloAxis)return false;
    double represented=std::fmod(rawNative,modulo);if(represented<0.0)represented+=modulo;
    return std::isfinite(represented)&&native==represented;
}
inline bool FeedbackNative(double referenceNative,double referencePulse,double observedPulse,
    double ppu,double& native) noexcept
{
    if(!std::isfinite(referenceNative)||!std::isfinite(referencePulse)||!std::isfinite(observedPulse)||
        !std::isfinite(ppu)||ppu<=0.0)return false;
    const double delta=observedPulse-referencePulse;
    if(!std::isfinite(delta))return false;
    // Keep signed zero and the exact frozen spelling at the reference. Do not
    // modulo the delta: a real whole-turn error must remain a whole-turn error.
    native=delta==0.0?referenceNative:referenceNative+delta/ppu;
    return std::isfinite(native);
}
inline bool CommandPulse(double native,double referenceNative,double referencePulse,double& pulse) noexcept
{
    if(!std::isfinite(native)||!std::isfinite(referenceNative)||!std::isfinite(referencePulse)||
        !SameBits(native,referenceNative))return false;
    pulse=referencePulse;return true;
}
}

enum class MotionG76SphereAdmissionGate : std::uint8_t
{
    None,Request,Frame,Compensation,OwnerDrain,AxisConfiguration,NativeBaseline,
    Travel,RuntimeRequest,Reservation,Publication,Enqueue
};
struct MotionG76SphereAdmission
{
    MotionG76SphereAdmissionGate gate=MotionG76SphereAdmissionGate::None;
    int axis=-1;
};
inline const char* MotionG76SphereAdmissionGateName(MotionG76SphereAdmissionGate gate) noexcept
{
    switch(gate){
    case MotionG76SphereAdmissionGate::None:return "NONE";
    case MotionG76SphereAdmissionGate::Request:return "REQUEST";
    case MotionG76SphereAdmissionGate::Frame:return "FRAME";
    case MotionG76SphereAdmissionGate::Compensation:return "COMPENSATION";
    case MotionG76SphereAdmissionGate::OwnerDrain:return "OWNER_OR_DRAIN";
    case MotionG76SphereAdmissionGate::AxisConfiguration:return "AXIS_CONFIG";
    case MotionG76SphereAdmissionGate::NativeBaseline:return "NATIVE_BASELINE";
    case MotionG76SphereAdmissionGate::Travel:return "TRAVEL";
    case MotionG76SphereAdmissionGate::RuntimeRequest:return "RUNTIME_REQUEST";
    case MotionG76SphereAdmissionGate::Reservation:return "RESERVATION";
    case MotionG76SphereAdmissionGate::Publication:return "PUBLICATION";
    case MotionG76SphereAdmissionGate::Enqueue:return "ENQUEUE";
    default:return "UNKNOWN";}
}

// EDM286: one custom AIR parent. Internal layers/HOLD legs never enter the
// NC ledger. All feedback is published by the RT owner as a bounded value.
struct MotionG76SphereFeedback
{
    NCG76SphereCycle::Feedback cycle{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease owner{};
    std::array<double,8U> endpointMCS{},commandMCS{},velocityNative{},actualMCS{};
    // Diagnostic copy of the producer's installed-window-clamped tolerances;
    // feedback never grants start authority or changes the frozen request.
    double positionToleranceMm=0.0,rotaryToleranceDeg=0.0;
    std::uint64_t session=0ULL,inputSequence=0ULL,holdSequence=0ULL,startSequence=0ULL;
    std::uint32_t runtimeFault=0U;
    bool accepted=false,started=false,terminal=false,completed=false,endpointCommitted=false;
};
