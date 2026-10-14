#pragma once
#include "EDMGapServoRuntime.h"
#include <cmath>
#include <cstdint>
#include <limits>

// EDM57 B1: SIM, discharge OFF. Process requests only; this component has no
// axis/PDO setters and cannot certify a physical Stop, Disarm or release.
namespace EDM57
{
// The owner binds effective E values before Arm. The RT copy carries only a
// bounded identity/value snapshot; it never reads the catalog or mutable rows.
struct RecipeIdentity
{
    std::uint32_t catalogRevision=0, definitionRevision=0, profileId=0;
    std::uint32_t tableId=0, tableRevision=0;
    std::uint16_t eCode=0, fieldIds[5]{};
    std::uint64_t generation=0;
};
inline bool SameIdentity(const RecipeIdentity&a,const RecipeIdentity&b) noexcept
{
    if(a.catalogRevision!=b.catalogRevision || a.definitionRevision!=b.definitionRevision ||
        a.profileId!=b.profileId || a.tableId!=b.tableId || a.tableRevision!=b.tableRevision ||
        a.eCode!=b.eCode || a.generation!=b.generation) return false;
    for(unsigned i=0;i<5;++i) if(a.fieldIds[i]!=b.fieldIds[i]) return false;
    return true;
}
inline bool ValidateIdentity(const RecipeIdentity&i) noexcept
{
    if(!i.catalogRevision || !i.definitionRevision || !i.profileId || !i.tableId ||
        i.tableId>99999999U || !i.tableRevision || !i.eCode || i.eCode>1000U || !i.generation) return false;
    for(unsigned n=0;n<5;++n)
    {
        if(!i.fieldIds[n]) return false;
        for(unsigned previous=0;previous<n;++previous) if(i.fieldIds[n]==i.fieldIds[previous]) return false;
    }
    return true;
}
inline bool IsServoFlushProfile(unsigned p) noexcept { return p>=49U && p<=52U; }
inline bool IsBoundProfile(unsigned p) noexcept { return (p>=42U && p<=45U) || IsServoFlushProfile(p); }
inline bool IsFlushProfile(unsigned p) noexcept { return (p>=38U && p<=45U) || IsServoFlushProfile(p); }
inline bool IsBlockedProfile(unsigned p) noexcept { return p==39U || p==43U || p==50U; }
inline bool IsPersistentProfile(unsigned p) noexcept { return p==40U || p==44U || p==51U; }
inline bool IsStaleProfile(unsigned p) noexcept { return p==41U || p==45U || p==52U; }
struct Recipe
{
    std::uint32_t revision=0, deepInterval=0, cycleCount=0;
    std::uint64_t workTimeUs=0, topDwellUs=0;
    double baseHeightMm=0, deepMultiplier=0, finalApproachMm=0;
    double finalApproachMmMin=0, liftMmMin=0, returnMmMin=0, workDepthMm=0;
    double jumpSpeedOverridePercent=0;
    RecipeIdentity binding{};
};
inline Recipe FixtureRecipe() noexcept
{
    Recipe r{}; r.revision=1; r.deepInterval=3; r.cycleCount=6;
    r.workTimeUs=200000; r.topDwellUs=100000; r.baseHeightMm=.06;
    r.deepMultiplier=1.5; r.finalApproachMm=.02; r.finalApproachMmMin=6;
    r.liftMmMin=r.returnMmMin=30; r.workDepthMm=.18; r.jumpSpeedOverridePercent=100;
    return r;
}
inline bool SameRecipe(const Recipe&a,const Recipe&b) noexcept
{
    return a.revision==b.revision && a.deepInterval==b.deepInterval && a.cycleCount==b.cycleCount &&
        a.workTimeUs==b.workTimeUs && a.topDwellUs==b.topDwellUs && a.baseHeightMm==b.baseHeightMm &&
        a.deepMultiplier==b.deepMultiplier && a.finalApproachMm==b.finalApproachMm &&
        a.finalApproachMmMin==b.finalApproachMmMin && a.liftMmMin==b.liftMmMin &&
        a.returnMmMin==b.returnMmMin && a.workDepthMm==b.workDepthMm &&
        a.jumpSpeedOverridePercent==b.jumpSpeedOverridePercent && SameIdentity(a.binding,b.binding);
}
inline bool ValidateRecipe(const Recipe&r) noexcept
{
    // Legacy P38..P41 keep their exact fixed recipe and zero binding identity.
    // E5 work duration = us; E6 height = mm; E7 speed override = percent;
    // E21 every N lifts; E22 deep-lift multiplier = dimensionless.
    return SameRecipe(r,FixtureRecipe());
}
inline std::uint32_t ExpectedDeepCycles(const Recipe&r,std::uint32_t cycles) noexcept
{ return r.deepInterval ? cycles/r.deepInterval : 0U; }
inline bool CanonicalMillionths(double value,std::uint64_t&units) noexcept
{
    // E values arrive from signed fixed-point millionths. Reconstruct only
    // that exact domain: check bounds before conversion and demand a round
    // trip, rather than adding an epsilon or clamping a physical height.
    if(!std::isfinite(value) || value<0.0 || value>100.0) return false;
    const double integral=std::floor(value*1000000.0+.5);
    const std::uint64_t candidate=static_cast<std::uint64_t>(integral);
    if(static_cast<double>(candidate)/1000000.0!=value) return false;
    units=candidate; return true;
}
inline double LiftHeight(const Recipe&r,bool deep) noexcept
{
    if(!deep) return r.baseHeightMm;
    if(r.revision!=2U && r.revision!=3U) return r.baseHeightMm*r.deepMultiplier;
    std::uint64_t height=0,multiplier=0;
    if(!CanonicalMillionths(r.baseHeightMm,height) || !CanonicalMillionths(r.deepMultiplier,multiplier))
        return (std::numeric_limits<double>::quiet_NaN)();
    // Each recovered operand is at most 1e8, so the product fits uint64.
    return static_cast<double>(height*multiplier)/1000000000000.0;
}
inline double MaximumLiftHeight(const Recipe&r) noexcept
{
    if(!r.deepInterval) return r.baseHeightMm;
    const double deep=LiftHeight(r,true);
    if(!std::isfinite(deep)) return deep;
    return r.baseHeightMm>deep?r.baseHeightMm:deep;
}
inline bool ValidateBoundRecipe(const Recipe&r) noexcept
{
    // EDM58 admission is deliberately narrower than the general E table.
    // No clamp or unconfirmed-unit conversion may gain physical authority.
    if(r.revision!=2U || !ValidateIdentity(r.binding) || r.cycleCount!=6U ||
        r.workTimeUs<50000U || r.workTimeUs>1000000U || r.topDwellUs!=100000U ||
        !std::isfinite(r.baseHeightMm) || r.baseHeightMm<.03 || r.baseHeightMm>.09 ||
        !std::isfinite(r.jumpSpeedOverridePercent) || r.jumpSpeedOverridePercent<50.0 || r.jumpSpeedOverridePercent>100.0 ||
        r.deepInterval>20U || !std::isfinite(r.deepMultiplier) ||
        r.deepMultiplier<(r.deepInterval?1.0:0.0) || r.deepMultiplier>2.0 ||
        r.finalApproachMm!=.02 || r.workDepthMm!=.18 ||
        r.liftMmMin!=30.0*r.jumpSpeedOverridePercent/100.0 ||
        r.returnMmMin!=30.0*r.jumpSpeedOverridePercent/100.0 ||
        r.finalApproachMmMin!=6.0*r.jumpSpeedOverridePercent/100.0) return false;
    std::uint64_t height=0,multiplier=0;
    if(!CanonicalMillionths(r.baseHeightMm,height) || !CanonicalMillionths(r.deepMultiplier,multiplier)) return false;
    return !r.deepInterval || height*multiplier<=120000000000ULL;
}
inline bool ValidateServoFlushRecipe(const Recipe&r) noexcept
{
    if(r.revision!=3U || r.workDepthMm!=.125) return false;
    Recipe legacy=r; legacy.revision=2U; legacy.workDepthMm=.18;
    return ValidateBoundRecipe(legacy);
}
inline bool ValidateRecipeForProfile(const Recipe&r,unsigned profile) noexcept
{
    if(IsServoFlushProfile(profile)) return ValidateServoFlushRecipe(r);
    return IsBoundProfile(profile)?ValidateBoundRecipe(r):
        (profile>=38U && profile<=41U && ValidateRecipe(r));
}
enum class Phase : std::uint32_t
{ ApproachWork, WorkSettle, Work, CaptureWorkPoint, Lift, TopSettle,
  TopDwell, Return, ApproachBoundarySettle, SlowApproach, ReturnSettle,
  ReturnToOrigin, OriginSettle, Terminal };
inline const char* PhaseName(std::uint32_t value) noexcept
{
    switch(static_cast<Phase>(value))
    {
    case Phase::ApproachWork:return "APPROACH_WORK"; case Phase::WorkSettle:return "WORK_SETTLE";
    case Phase::Work:return "WORK"; case Phase::CaptureWorkPoint:return "CAPTURE_WORK";
    case Phase::Lift:return "LIFT"; case Phase::TopSettle:return "TOP_SETTLE";
    case Phase::TopDwell:return "TOP_DWELL"; case Phase::Return:return "RETURN";
    case Phase::ApproachBoundarySettle:return "APPROACH_BOUNDARY_SETTLE";
    case Phase::SlowApproach:return "SLOW_APPROACH"; case Phase::ReturnSettle:return "RETURN_SETTLE";
    case Phase::ReturnToOrigin:return "RETURN_TO_ORIGIN"; case Phase::OriginSettle:return "ORIGIN_SETTLE";
    case Phase::Terminal:return "TERMINAL";
    }
    return "UNKNOWN";
}
inline const char* PhaseName(Phase p) noexcept { return PhaseName(static_cast<std::uint32_t>(p)); }
struct Feedback
{
    std::uint32_t phase=0, cycles=0, deepCycles=0, blockedCount=0;
    std::uint32_t resumedReturnCount=0, slowApproachCount=0, topDwellCount=0;
    std::uint64_t phaseSinceUs=0, stableSinceUs=0, blockedSinceUs=0, blockedSequence=0;
    double workPointMm=0, topMm=0, destinationMm=0, liftHeightMm=0;
    bool returnBlocked=false, persistentLatched=false, complete=false;
};
struct Readback
{
    std::uint64_t nowUs=0;
    double actualMm=0, commandMm=0, planningMm=0;
    double velocityMmS=0, logicalVelocityMmS=0, targetVelocityMmS=0;
};
struct Request
{
    double destinationMm=0, speedMmMin=0;
    bool complete=false, persistentShort=false;
};
class Runtime
{
public:
    const Feedback& Snapshot() const noexcept { return f_; }
    const Recipe& FrozenRecipe() const noexcept { return recipe_; }
    bool Begin(const Recipe&r,unsigned profile,const Readback&o,bool persistent) noexcept
    {
        if(!ValidateRecipeForProfile(r,profile) || !o.nowUs) return false;
        recipe_=r; profile_=profile; f_=Feedback{}; stableLow_=stableHigh_=0;
        f_.persistentLatched=persistent; f_.workPointMm=r.workDepthMm;
        Set((o.commandMm!=0.0 || std::fabs(o.actualMm)>.001 || persistent)?
            Phase::ReturnToOrigin:Phase::ApproachWork,o.nowUs);
        return true;
    }
    double SourceVoltage(const Readback&o) noexcept
    {
        const Phase p=static_cast<Phase>(f_.phase);
        if(IsPersistentProfile(profile_) && (p==Phase::TopSettle || p==Phase::TopDwell)) f_.persistentLatched=true;
        if(f_.persistentLatched) return 20.0;
        if(IsBlockedProfile(profile_) && f_.cycles==0 && (p==Phase::TopSettle || p==Phase::TopDwell))
            return !f_.blockedSinceUs || o.nowUs-f_.blockedSinceUs<100000?20.0:50.0;
        if(IsServoFlushProfile(profile_) && p==Phase::Work)
        {
            const std::uint64_t elapsed=o.nowUs-f_.phaseSinceUs;
            // Bounded SIM excitation exercises the machining servo's positive
            // and negative curve while physical discharge remains OFF.
            return elapsed>=recipe_.workTimeUs/4U && elapsed<3U*recipe_.workTimeUs/4U?50.0:80.0;
        }
        return p==Phase::ApproachWork || p==Phase::WorkSettle || p==Phase::Work ||
            p==Phase::CaptureWorkPoint?80.0:50.0;
    }
    bool InjectStale(const Readback&o) const noexcept
    {
        return IsStaleProfile(profile_) && static_cast<Phase>(f_.phase)==Phase::Lift &&
            f_.workPointMm-o.actualMm>=.01 && o.velocityMmS<0.0;
    }
    Request Step(const Readback&o,const EDM55::Output&source) noexcept
    {
        Phase p=static_cast<Phase>(f_.phase);
        const bool fresh=source.valid && source.fault==EDM55::Fault::None && source.sequence &&
            o.nowUs>=source.capturedAtUs && o.nowUs-source.capturedAtUs<=5000;
        if(f_.persistentLatched && source.shortActive && p!=Phase::ReturnToOrigin && p!=Phase::OriginSettle)
        { Set(Phase::ReturnToOrigin,o.nowUs); p=Phase::ReturnToOrigin; }
        switch(p)
        {
        case Phase::ApproachWork:
            f_.destinationMm=recipe_.workDepthMm;
            if(CommandAt(o,f_.destinationMm)) Set(Phase::WorkSettle,o.nowUs);
            break;
        case Phase::WorkSettle:
            if(Settled(o,recipe_.workDepthMm)) Set(Phase::Work,o.nowUs);
            break;
        case Phase::Work:
            if(o.nowUs-f_.phaseSinceUs>=recipe_.workTimeUs)
            {
                if(IsServoFlushProfile(profile_)) f_.destinationMm=BrakingEndpoint(o);
                Set(Phase::CaptureWorkPoint,o.nowUs);
            }
            break;
        case Phase::CaptureWorkPoint:
            if(Settled(o,IsServoFlushProfile(profile_)?f_.destinationMm:recipe_.workDepthMm))
            {
                // Anchor to observed Z only after actual/command/planning
                // stability and zero command speed; never to a virtual tick.
                f_.workPointMm=o.actualMm;
                f_.liftHeightMm=LiftHeight(recipe_,recipe_.deepInterval && (f_.cycles+1)%recipe_.deepInterval==0);
                f_.topMm=f_.workPointMm-f_.liftHeightMm;
                Set(Phase::Lift,o.nowUs);
            }
            break;
        case Phase::Lift:
            f_.destinationMm=f_.topMm;
            if(CommandAt(o,f_.topMm)) Set(Phase::TopSettle,o.nowUs);
            break;
        case Phase::TopSettle:
            if(Settled(o,f_.topMm)) Set(Phase::TopDwell,o.nowUs);
            break;
        case Phase::TopDwell:
            if(!At(o,f_.topMm)) { Set(Phase::TopSettle,o.nowUs); break; }
            if(o.nowUs-f_.phaseSinceUs>=recipe_.topDwellUs)
            {
                const bool blocked=!fresh || source.shortActive || source.feedInhibited;
                if(blocked)
                {
                    f_.returnBlocked=true;
                    if(!f_.blockedSinceUs && fresh && source.shortActive)
                    { f_.blockedSinceUs=o.nowUs; f_.blockedSequence=source.sequence; ++f_.blockedCount; }
                }
                else if(!IsBlockedProfile(profile_) || f_.cycles!=0 ||
                    (f_.blockedCount && source.sequence>f_.blockedSequence && source.rawVoltageV==50.0))
                {
                    if(f_.returnBlocked) ++f_.resumedReturnCount;
                    f_.returnBlocked=false; ++f_.topDwellCount; Set(Phase::Return,o.nowUs);
                }
            }
            break;
        case Phase::Return:
            f_.destinationMm=f_.workPointMm-recipe_.finalApproachMm;
            if(CommandAt(o,f_.destinationMm)) Set(Phase::ApproachBoundarySettle,o.nowUs);
            break;
        case Phase::ApproachBoundarySettle:
            // Stop and prove actual readback at the boundary. Merely changing
            // 30 to 6 at the last .02 mm cannot meet the braking distance.
            if(Settled(o,f_.workPointMm-recipe_.finalApproachMm) && fresh &&
                !source.shortActive && !source.feedInhibited)
            { ++f_.slowApproachCount; Set(Phase::SlowApproach,o.nowUs); }
            break;
        case Phase::SlowApproach:
            f_.destinationMm=f_.workPointMm;
            if(CommandAt(o,f_.workPointMm)) Set(Phase::ReturnSettle,o.nowUs);
            break;
        case Phase::ReturnSettle:
            if(Settled(o,f_.workPointMm) && fresh && !source.shortActive && !source.feedInhibited)
            {
                ++f_.cycles;
                if(recipe_.deepInterval && f_.cycles%recipe_.deepInterval==0) ++f_.deepCycles;
                Set(f_.cycles>=recipe_.cycleCount?Phase::ReturnToOrigin:
                    IsServoFlushProfile(profile_)?Phase::Work:Phase::ApproachWork,o.nowUs);
            }
            break;
        case Phase::ReturnToOrigin:
            f_.destinationMm=0;
            if(CommandAt(o,0)) Set(Phase::OriginSettle,o.nowUs);
            break;
        case Phase::OriginSettle:
            // EDM60 delegates its final physical qualification to the same
            // FIX2 inner origin gate used by the bound machining controller.
            // This local state machine never claims that stronger receipt.
            if(IsServoFlushProfile(profile_)) break;
            if(Settled(o,0))
            {
                if(f_.persistentLatched)
                {
                    if(fresh && source.shortActive && source.rawVoltageV==20.0)
                    { Set(Phase::Terminal,o.nowUs); return {0,0,false,true}; }
                }
                else if(f_.cycles>=recipe_.cycleCount)
                { f_.complete=true; Set(Phase::Terminal,o.nowUs); return {0,0,true,false}; }
                else Set(Phase::ApproachWork,o.nowUs);
            }
            break;
        case Phase::Terminal: return {0,0,f_.complete,f_.persistentLatched};
        }
        p=static_cast<Phase>(f_.phase);
        Request result{};
        switch(p)
        {
        case Phase::Work:
            if(IsServoFlushProfile(profile_))
            {
                result.speedMmMin=source.speedMmPerMin;
                result.destinationMm=result.speedMmMin>=0.0?.249:MaximumLiftHeight(recipe_)+.002;
            }
            else result.destinationMm=f_.destinationMm;
            break;
        case Phase::ApproachWork: result.destinationMm=recipe_.workDepthMm; result.speedMmMin=o.commandMm>recipe_.workDepthMm?-recipe_.returnMmMin:recipe_.returnMmMin; break;
        case Phase::Lift: result.destinationMm=f_.topMm; result.speedMmMin=-recipe_.liftMmMin; break;
        case Phase::Return: result.destinationMm=f_.workPointMm-recipe_.finalApproachMm; result.speedMmMin=recipe_.returnMmMin; break;
        case Phase::SlowApproach: result.destinationMm=f_.workPointMm; result.speedMmMin=recipe_.finalApproachMmMin; break;
        case Phase::ReturnToOrigin: result.destinationMm=0; result.speedMmMin=o.commandMm<0?recipe_.returnMmMin:-recipe_.returnMmMin; break;
        default: result.destinationMm=f_.destinationMm; break;
        }
        f_.destinationMm=result.destinationMm;
        // GAP detector gates each individual downward candidate, including the
        // very first return candidate after a top dwell. Speed shaping belongs
        // to the surrounding exact-commit physical controller.
        if(result.speedMmMin>0 && (!fresh || source.shortActive || source.feedInhibited)) result.speedMmMin=0;
        return result;
    }
    // The caller supplies the independently earned FIX2 origin predicate.
    // Retained low intent still requires a current, debounced 20V sample.
    Request CompleteOrigin(const Readback&o,const EDM55::Output&source) noexcept
    {
        if(!IsServoFlushProfile(profile_) || static_cast<Phase>(f_.phase)!=Phase::OriginSettle)
            return {};
        const bool fresh=source.valid && source.fault==EDM55::Fault::None && source.sequence &&
            o.nowUs>=source.capturedAtUs && o.nowUs-source.capturedAtUs<=5000;
        if(f_.persistentLatched)
        {
            if(fresh && source.shortActive && source.feedInhibited && source.rawVoltageV==20.0)
            { Set(Phase::Terminal,o.nowUs); return {0,0,false,true}; }
        }
        else if(f_.cycles>=recipe_.cycleCount)
        { f_.complete=true; Set(Phase::Terminal,o.nowUs); return {0,0,true,false}; }
        else Set(Phase::ApproachWork,o.nowUs);
        return {};
    }
private:
    // Same fixed acceleration and 250 us trapezoidal integration as EDM56.
    // Freeze this endpoint once before deceleration, not at every moving tick.
    static double BrakingEndpoint(const Readback&o) noexcept
    {
        constexpr double dt=.00025, acceleration=.5/.3, step=acceleration*dt;
        const double speed=std::fabs(o.velocityMmS);
        const double whole=std::floor(speed/step);
        const double remainder=speed-whole*step;
        const double distance=dt*(whole*speed-.5*step*whole*whole+.5*remainder);
        return o.commandMm+(o.velocityMmS<0.0?-distance:distance);
    }
    Recipe recipe_{}; Feedback f_{}; unsigned profile_=0;
    double stableLow_=0,stableHigh_=0;
    void Set(Phase p,std::uint64_t now) noexcept
    { f_.phase=static_cast<std::uint32_t>(p); f_.phaseSinceUs=now; f_.stableSinceUs=0; }
    static bool CommandAt(const Readback&o,double target) noexcept
    { return std::fabs(o.commandMm-target)<=1e-12 && std::fabs(o.planningMm-target)<=1e-12 &&
        o.velocityMmS==0 && o.logicalVelocityMmS==0 && o.targetVelocityMmS==0; }
    static bool At(const Readback&o,double target) noexcept
    { return CommandAt(o,target) && std::isfinite(o.actualMm) && std::fabs(o.actualMm-target)<=.001; }
    bool Settled(const Readback&o,double target) noexcept
    {
        if(!At(o,target)) { f_.stableSinceUs=0; return false; }
        if(!f_.stableSinceUs) { f_.stableSinceUs=o.nowUs; stableLow_=stableHigh_=o.actualMm; }
        if(o.actualMm<stableLow_) stableLow_=o.actualMm;
        if(o.actualMm>stableHigh_) stableHigh_=o.actualMm;
        if(stableHigh_-stableLow_>.001)
        { f_.stableSinceUs=o.nowUs; stableLow_=stableHigh_=o.actualMm; }
        return o.nowUs-f_.stableSinceUs>=50000;
    }
};
static_assert(sizeof(RecipeIdentity)==40U,"EDM58 bounded recipe identity");
static_assert(sizeof(Recipe)==136U,"EDM58 bounded frozen recipe");
static_assert(sizeof(Feedback)<=160U,"EDM57 bounded telemetry");
}
