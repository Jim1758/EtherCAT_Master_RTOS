#pragma once
#include "NCG76SphereRuntimeGeometry.h"
#include "NCG76SphereRetreat.h"
#include "NCG76SphereDeepFlush.h"
#include "HoldConfig.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>

class MotionCore;

// EDM286: one bounded analytic P50 parent. Pure commanded trajectory only;
// ownership, frame identity, PDO provenance, power inhibition and final-send
// evidence are independently supplied/proved by the production adapter.
namespace NCG76SphereCycle
{
using Point8 = std::array<double,8U>;
using Point3 = std::array<double,3U>;
using Geometry = NCG76SphereRuntimeGeometry::Geometry;

enum class Phase : std::uint8_t
{
    Idle, InitialDepth, InitialRetreat, CircleEntry, Orbit, ReturnCenter,
    AdvanceCenter, Settle, HoldBrake, HoldSettle, HoldToCenter, HoldToStart,
    HeldAtStart, ResumeToCenter, ResumeToAnchor, FaultBrake, FaultSettle,
    Complete, Fault,
    // Appended to retain the published values of all existing phases. A HOLD
    // on a resume leg brakes that excursion without replacing the original
    // nominal checkpoint, then returns through its center to the start.
    ResumeBrake,
    // EDM294: AIR-only E5 excursion. Nominal geometry remains one parent.
    FlushBrake, FlushSettle, FlushLift, FlushReturn, FlushHoldBrake,
    // EDM300: one previously completed nominal leg, within this same parent.
    GapHistorySettle, GapHistory,
    // EDM301: E5-selected center and same-parent retained-path excursions.
    FlushToCenter, FlushFromCenter, FlushPathRetreat, FlushPathReturn,
    // EDM331: opt-in common machining HOLD recovery and inspection detour.
    RecoveryBrake, RecoverySettle, RecoveryMove,
    InspectionJog, InspectionJogBrake, InspectionJogReturn
};
enum class Fault : std::uint8_t
{
    None, InvalidRequest, SourceLost, PdoLost, PowerNotInhibited, Clock,
    PhysicalSequence, IntentSequence, Numeric, TravelLimit, SettleTimeout,
    CycleTimeout, RepeatedHold, Cancelled
};
enum class FinishPhase : std::uint8_t { Disabled, Machining, ForcedFlush, Recheck, Passed, Fault };
enum class FlushOrigin : std::uint8_t { None, Periodic, Finish };

struct Request
{
    Geometry geometry{};
    std::uint64_t session = 0ULL;
    double feedMmMin = 0.0, returnFeedMmMin = 0.0;
    double velocityCapMmS = 0.0, accelerationMmS2 = 0.0, decelerationMmS2 = 0.0;
    Point8 travelMin{}, travelMax{};
    std::uint32_t travelMask = 0U, retreatMode = 0U;
    double positionToleranceMm = 0.001;
    // Fixed C/U/V use the specified +/-0.005 degree rotary arrival tolerance.
    // The RT adapter still tightens this to each installed in-position window.
    double rotaryToleranceDeg = 0.005;
    unsigned settleSamples = 3U;
    double minimumSettleSeconds = 0.05, settleTimeoutSeconds = 5.0, maximumCycleSeconds = 86400.0;
    std::uint64_t minimumTickUs = 50ULL, maximumTickUs = 1000ULL;
    // EDM308: line/arc machining uses the frozen condition and pump speed
    // envelope. An authored F is optional and caps nominal GAP motion only;
    // it never slows an independent flushing or return pump.
    bool parameterFeeds = false;
    double nominalFeedCapMmMin = 0.0;
};

struct Input
{
    std::uint64_t tick = 0ULL, nowUs = 0ULL, physicalSequence = 0ULL;
    std::uint64_t holdSequence = 0ULL, startSequence = 0ULL;
    Point8 actualMCS{};
    bool sourceValid = false, pdoValid = false, powerInhibited = false;
    bool hold = false, cancel = false;
    bool physicalSettled = false, zeroFinalSend = false;
    std::uint64_t zeroSendTick = 0ULL;
    std::uint64_t flushRequestSequence = 0ULL;
    double signedGapSpeedMmMin = 0.0;
    bool gapValid = false;
};

struct Output
{
    Point8 commandMCS{}, velocityNative{}, accelerationNative{};
    bool active = false, complete = false, stopped = true;
    bool faulted = false, emergencyStopRequired = false, cancelled = false;
    bool zeroDemand = true;
};

// EDM306 FIX1: first-fault evidence only. This snapshot never changes the
// admitted timing limits or rebases a parent after a delayed callback.
enum ClockFaultBits : std::uint32_t
{
    TickNonIncreasing = 1U, TickGap = 2U, TimeNonIncreasing = 4U,
    IntervalTooShort = 8U, IntervalTooLong = 16U
};
struct ClockFaultSnapshot
{
    std::uint64_t previousTick = 0ULL, currentTick = 0ULL;
    std::uint64_t previousNowUs = 0ULL, currentNowUs = 0ULL;
    std::uint64_t previousPhysicalSequence = 0ULL, currentPhysicalSequence = 0ULL;
    std::uint64_t deltaUs = 0ULL, minimumTickUs = 0ULL, maximumTickUs = 0ULL;
    std::uint32_t reasonBits = 0U;
    bool deltaValid = false, valid = false;
};
static_assert(sizeof(ClockFaultSnapshot) <= 128U,
    "P50 first clock-fault evidence must remain bounded value data");
static_assert(std::is_trivially_copyable<ClockFaultSnapshot>::value,
    "P50 first clock-fault evidence cannot acquire ownership");

struct Feedback
{
    Phase phase = Phase::Idle, nominalPhase = Phase::InitialDepth;
    Fault fault = Fault::None;
    std::uint64_t session = 0ULL, tick = 0ULL, holdSequence = 0ULL, resumeSequence = 0ULL;
    unsigned layerIndex = 0U, completedLayers = 0U;
    double nominalProgressMm = 0.0, nominalLengthMm = 0.0;
    double phaseProgressMm = 0.0, phaseLengthMm = 0.0, speedMmS = 0.0;
    bool held = false, holdActive = false;
    std::uint64_t flushRequestSequence = 0ULL, flushCompletedSequence = 0ULL, flushAbandonedSequence = 0ULL;
    unsigned flushCount = 0U;
    std::uint64_t flushOrdinal = 0ULL;
    double flushSelectedHeightMm = 0.0;
    bool flushDeepActive = false;
    unsigned retreatMode = 0U;
    std::uint64_t flushLegSequence = 0ULL;
    double flushPathStationMm = 0.0, flushAnchorStationMm = 0.0;
    bool flushActive = false, flushCheckpointSaved = false;
    double flushSelectedFeedMmMin = 0.0, flushTargetFeedMmMin = 0.0, flushRemainingMm = 0.0;
    bool flushApproachActive = false, flushInitialSlowActive = false;
    // RT-owned generation distinguishes a new held-position requalification
    // even if NC observes it only after the full settle has finished again.
    std::uint64_t heldRequalificationSequence = 0ULL;
    // Signed nominal velocity is separate from the legacy speed magnitude.
    double signedSpeedMmS = 0.0;
    bool gapMotionEnabled = false, gapApplied = false;
    bool gapHistoryActive = false;
    // EDM309: one completed primitive is replayed as an excursion owned by
    // the current parent. These fields never retire/complete its old NC row.
    bool previousParentHistoryReady = false, previousParentHistoryActive = false;
    double previousParentProgressMm = 0.0, previousParentLengthMm = 0.0;
    std::uint64_t historyBoundarySequence = 0ULL, historyReturnSequence = 0ULL;
    ClockFaultSnapshot clockFault{};
    std::uint32_t holdDeclaredCount=0U,holdEffectiveCount=0U,holdAvailableCount=0U;
    std::uint32_t inspectionTraceCount=0U;
    std::uint8_t holdRecoveryPhase=0U;
    bool holdRecoveryEnabled=false,holdRecoveryPaused=false,inspectionJogActive=false,inspectionTraceValid=false;
    // EDM335: per-parent consecutive finish checks, separate from E5 work and
    // periodic/deep-flush ordinals. A forced return alone is not a passed check.
    std::uint32_t finishRequiredCount=0U,finishVerifiedCount=0U;
    std::uint64_t finishAttemptCount=0ULL,finishForcedCompletedCount=0ULL,finishReworkCount=0ULL;
    std::uint8_t finishPhase=0U,flushOrigin=0U;
    bool finishActive=false;
};
static_assert(sizeof(Feedback) <= 512U, "P50 pure-cycle feedback remains bounded");

// Separate from Request: the existing motion publication remains below its
// unchanged 4096-byte bound. Height is physical native mm, not authored scale.
struct FlushConfig
{
    double heightMm = 0.0, feedMmMin = 0.0;
    // Cooked native mm/min: the NC recipe binder applies percentages once.
    double centerRetreatFeedMmMin = 0.0, axisRetreatFeedMmMin = 0.0;
    double centerReturnFeedMmMin = 0.0, axisReturnFeedMmMin = 0.0, pathRetreatFeedMmMin = 0.0;
    double finalApproachFeedMmMin = 0.0, finalApproachDistanceMm = 0.0;
    double initialSlowFeedMmMin = 0.0, initialSlowDistanceMm = 0.0;
    std::uint32_t deepFlushCycleInterval = 0U;
    double deepFlushHeightMm = 0.0;
    bool enabled = false, independentFeedsEnabled = false, finalApproachEnabled = false, initialSlowEnabled = false, deepFlushEnabled = false;
};

// Optional SIM-only signed progress control. This changes neither Request nor
// the admitted geometry/travel envelope. Cross-leg mode is one adjacent,
// already completed nominal leg only. EDM309 may additionally borrow one
// RT-proved completed primitive during admission; its old NC row stays closed.
struct GapConfig {
    bool enabled = false, crossLegRetreatEnabled = false;
    // EDM307: RT-proved continuity across completed line/arc parents only.
    // The current parent's count remains local; this offset selects deep
    // flush ordinals without resetting at every short G01/G02/G03 block.
    std::uint64_t completedFlushOffset = 0ULL;
    // Admission-only external storage. Begin copies the geometry into the
    // Cycle and clears this pointer before any periodic interpolation work.
    const Geometry* previousPrimitive = nullptr;
};

// Validation is an admission operation, not part of the 250-us tick.
inline bool ValidateRequest(const Request& request) noexcept;
inline bool ValidateFlushConfig(const Request& request,const FlushConfig& config) noexcept;

class Cycle
{
public:
    bool Begin(const Request& request,const Input& input,const FlushConfig* flush = nullptr,
        const GapConfig* gap = nullptr) noexcept;
    Output Step(const Input& input) noexcept;
    const Feedback& Current() const noexcept { return feedback_; }
    const Output& Command() const noexcept { return output_; }
    const NCG76SphereRuntimeGeometry::Geometry& Geometry() const noexcept { return request_.geometry; }
    bool ZeroDemand() const noexcept { return output_.zeroDemand; }
    bool ConfigureFinishVerification(std::uint32_t requiredCount) noexcept;
    // completedOldestFirst is immutable RT-owned storage. It must outlive
    // this admitted parent, including its HOLD, inspection and return route.
    bool ConfigureHoldRecovery(const NCHoldRecovery::Config& config,std::uint32_t declaredCount,
        const NCG76SphereRuntimeGeometry::Geometry* completedOldestFirst,std::uint32_t completedCount) noexcept;
    bool BeginInspectionJog(unsigned axis,double signedVelocityMmMin) noexcept;
    bool EndInspectionJog() noexcept;
    bool CanInspectionJog() const noexcept;
    bool InspectionTravelAvailable() const noexcept;
    bool IsHoldRecoveryPaused() const noexcept
    {return recovery_.active&&(recovery_.stage==RecoveryStage::PauseBraking||
        recovery_.stage==RecoveryStage::PauseSettle||recovery_.stage==RecoveryStage::Paused);}
    const Point8& HoldRetreatAnchor() const noexcept {return recovery_.retreatAnchor;}
    const Point8& HoldProcessPoint() const noexcept {return recovery_.nominalAnchor;}
    const Point8& HoldPausedPoint() const noexcept {return recovery_.processPoint;}
    void Reset() noexcept
    {
        // The Cycle is RT-owned persistent storage. Reconstruct it in place,
        // without materializing another complete Cycle on the 32 KiB stack.
        static_assert(std::is_trivially_destructible<Cycle>::value,
            "P50 reset must not acquire destruction side effects");
        this->~Cycle();::new (static_cast<void*>(this)) Cycle{};
    }

private:
    friend class ::MotionCore;
    // The sole production friend supplies its immutable, already producer-
    // and consumer-admitted geometry cache. Public callers take full checks.
    bool ConfigureHoldRecoveryAdmitted(const NCHoldRecovery::Config& config,std::uint32_t declaredCount,
        const NCG76SphereRuntimeGeometry::Geometry* completedOldestFirst,std::uint32_t completedCount) noexcept;
    bool ConfigureInspectionTravel(const Point8& minimum,const Point8& maximum,std::uint32_t mask) noexcept;
    bool IsInspectionTravelActive() const noexcept;
    enum class SettleAction : std::uint8_t
    { BeginNominal, NextNominal, HoldCenter, HoldStart, Held, ResumeCenter, ResumeAnchor, RestoreNominal, Complete, Fault,
      CaptureFlush, FlushReturn, RestoreAfterFlush, EnterGapHistory, RestoreGapHistory, NextFlushLeg, RestorePrimitivePause,
      RecoveryContinue, FinishCheck };
    struct ProfilePart
    {
        double length=0.0, initialSpeed=0.0, finalSpeed=0.0, peak=0.0;
        double accelerationTime=0.0, cruiseTime=0.0, decelerationTime=0.0;
        double accelerationDistance=0.0, duration=0.0;
    };
    struct Leg
    {
        Point8 start{},end{},center{};
        Point3 direction{};
        double length = 0.0, position = 0.0, speed = 0.0;
        double speedLimit = 0.0, acceleration = 0.0, deceleration = 0.0;
        double basePosition = 0.0, elapsed = 0.0, peak = 0.0;
        double accelerationTime = 0.0, cruiseTime = 0.0, decelerationTime = 0.0;
        double accelerationDistance = 0.0;
        double accelerationNow = 0.0;
        double stopPosition = 0.0, stopTime = 0.0, stopElapsed = 0.0, stopDirection = 1.0;
        bool circle = false;
        unsigned circleLayer = 0U;
        double circleStartAngle = 0.0, circleEndAngle = NCG76SphereRuntimeGeometry::TwoPi;
        ProfilePart approachPrefix{}, approachTail{};
        double approachStart = 0.0, approachSpeed = 0.0;
        bool approachProfile = false;
    };
    bool InsideTravel(const Point8& point) const noexcept;
    bool AtCommand(const Point8& actual) const noexcept;
    bool StartProfile(Leg& leg) noexcept;
    bool BuildProfilePart(double length,double startSpeed,double endSpeed,double speedLimit,
        double acceleration,double deceleration,ProfilePart& part) noexcept;
    bool ConfigureFlushProfile(Phase phase) noexcept;
    bool FlushOutboundStartDistance(Phase phase,double& distance) const noexcept;
    double FlushFeed(Phase phase) const noexcept;
    bool EvaluateProfilePart(const ProfilePart& part,double elapsed,double& position,double& speed,double& acceleration) const noexcept;
    bool StartLine(const Point8& end,Phase phase,double feed) noexcept;
    bool StartNominal(Phase phase) noexcept;
    bool EvaluateLeg() noexcept;
    bool AdvanceLeg(double dt,bool brake,bool& finished) noexcept;
    bool AdvanceGap(double dt,double signedTarget,bool& finished) noexcept;
    bool StartBrake(Phase phase) noexcept;
    void StartSettle(Phase phase,SettleAction action,const Input& input) noexcept;
    bool Settled(const Input& input,double dt) noexcept;
    void ContinueAfterSettle(const Input& input) noexcept;
    void NextNominal(const Input& input) noexcept;
    void CheckFinishEndpoint(const Input& input) noexcept;
    void BeginFinishFlush(const Input& input) noexcept;
    bool ObserveFinishProgress(double previousPosition) noexcept;
    void BeginHold(const Input& input) noexcept;
    void BeginPrimitiveHold(const Input& input) noexcept;
    void CapturePrimitiveHold(const Input& input) noexcept;
    void SaveHoldAnchor(const Input& input) noexcept;
    void BeginFlush(const Input& input) noexcept;
    void CaptureFlush(const Input& input) noexcept;
    bool FlushDistanceAvailable() const noexcept;
    double NextFlushHeight(bool& deep,std::uint64_t& ordinal) const noexcept;
    bool StartFlushPathLeg(bool reverse) noexcept;
    void NextFlushLeg(const Input& input) noexcept;
    bool StartFlushLine(const Point8& end,Phase phase) noexcept;
    void HoldDuringFlush(const Input& input) noexcept;
    bool CanEnterGapHistory() const noexcept;
    void BeginGapHistory(const Input& input) noexcept;
    bool SeedPreviousPrimitive(const NCG76SphereRuntimeGeometry::Geometry& geometry) noexcept;
    const NCG76SphereRuntimeGeometry::Geometry& ActiveLegGeometry() const noexcept
    {return recovery_.evaluationGeometry?*recovery_.evaluationGeometry:
        previousPrimitiveLegActive_?previousPrimitive_:request_.geometry;}
    double LegCoordinateError() const noexcept
    {return (std::max)(request_.geometry.coordinateErrorBoundMm,ActiveLegGeometry().coordinateErrorBoundMm);}
    void Fail(Fault fault,bool emergency=true) noexcept;
    void RefreshOutput() noexcept;
    enum class RecoveryStage:std::uint8_t
    {None,Capture,BridgeOutPath,BridgeOutLines,Retreat,Held,Jog,JogStopping,JogSettle,
     JogReturn,Forward,BridgeInPath,BridgeInLines,Restore,PauseBraking,PauseSettle,Paused};
    struct RecoveryState {
        NCHoldRecovery::Config config{};
        const NCG76SphereRuntimeGeometry::Geometry* history=nullptr;
        const NCG76SphereRuntimeGeometry::Geometry* evaluationGeometry=nullptr;
        const NCG76SphereRuntimeGeometry::Geometry* bridgeGeometry=nullptr;
        std::uint32_t historyCount=0U,targetOrdinal=0U,ordinal=0U;
        RecoveryStage stage=RecoveryStage::None,pausedStage=RecoveryStage::None;
        Phase sourcePhase=Phase::Idle,pausedMovePhase=Phase::Idle;
        SettleAction sourceAction=SettleAction::BeginNominal;
        Leg sourceLeg{},pausedMoveLeg{};
        Feedback sourceFeedback{};
        Point8 processPoint{},nominalAnchor{},retreatAnchor{},jogStart{};
        Point8 inspectionMinimum{},inspectionMaximum{};
        std::array<Point8,3U> bridgePoints{};
        // Stop-delimited, actually commanded XYZ detours. No endpoint chord
        // or incomplete journal is allowed to authorize inspection return.
        std::array<Point8,64U> inspectionPoints{};
        unsigned bridgeCount=0U,bridgeIndex=0U,inspectionCount=0U,inspectionIndex=0U;
        double nominalStation=0.0,pathStation=0.0,pathTarget=0.0;
        double legStartStation=0.0,legEndStation=0.0;
        double bridgeStartStation=0.0,bridgeEndStation=0.0;
        double inspectionFeed=0.0;
        bool enabled=false,active=false,captured=false,brakingCapture=false,jogStop=false,inspectionTravelReady=false;
    } recovery_{};
    const NCG76SphereRuntimeGeometry::Geometry& RecoveryGeometry(unsigned ordinal) const noexcept;
    bool RecoveryTotal(const NCG76SphereRuntimeGeometry::Geometry&,double&) const noexcept;
    bool RecoveryStation(Phase,unsigned,const Leg&,double&) const noexcept;
    bool StartRecoveryLine(const Point8&,bool returning,Phase=Phase::RecoveryMove) noexcept;
    bool StartRecoveryPath(const NCG76SphereRuntimeGeometry::Geometry&,double,double,bool returning) noexcept;
    bool BeginRecoveryHold(const Input&) noexcept;
    bool CaptureRecoveryStop(const Input&) noexcept;
    void ContinueRecovery(const Input&) noexcept;
    void DriveRecoveryPath(const Input&,bool returning) noexcept;
    void RestoreRecoveryCheckpoint(const Input&) noexcept;
    void StepRecovery(const Input&,double,bool holdEdge) noexcept;
    void RefreshRecoveryFeedback() noexcept;

    Request request_{};
    FlushConfig flush_{};
    GapConfig gap_{};
    Output output_{};
    Feedback feedback_{};
    Leg leg_{}, savedLeg_{};
    // Persistent, bounded history: no stack array, allocator or prior parent.
    Leg previousLeg_{}, historyNominalLeg_{};
    Phase previousPhase_ = Phase::Idle;
    unsigned previousLayer_ = 0U;
    bool previousLegValid_ = false;
    NCG76SphereRuntimeGeometry::Geometry previousPrimitive_{};
    bool previousPrimitiveLegActive_ = false;
    struct FinishState { bool forcedFlush=false,returnedPending=false,reworkLatched=false; } finish_{};
    Point8 savedAnchor_{}, savedCenter_{};
    Point8 flushTop_{}, flushWaypoint_{};
    double flushPathTarget_ = 0.0, flushPathLegStart_ = 0.0, flushPathLegEnd_ = 0.0;
    unsigned flushRouteIndex_ = 0U;
    Phase savedPhase_ = Phase::InitialDepth;
    // G01/G02/G03 HOLD pauses the current real leg in place. No nominal or
    // excursion chord is generated, and a flush keeps its selected identity.
    Phase primitivePausedPhase_ = Phase::Idle;
    SettleAction primitivePausedAction_ = SettleAction::BeginNominal;
    bool primitivePaused_ = false;
    SettleAction settleAction_ = SettleAction::BeginNominal;
    std::uint64_t lastTick_ = 0ULL,lastNowUs_ = 0ULL,lastPhysical_ = 0ULL;
    std::uint64_t lastHold_ = 0ULL,lastStart_ = 0ULL,heldStartFloor_ = 0ULL,settleTick_ = 0ULL,settlePhysical_ = 0ULL;
    std::uint64_t flushHighWater_ = 0ULL;
    unsigned settledSamples_ = 0U;
    double elapsed_ = 0.0,settleElapsed_ = 0.0,settleProvedSeconds_ = 0.0;
    bool initialized_ = false,lastHoldLevel_ = false;
};
} // namespace NCG76SphereCycle

namespace NCG76SphereCycle
{
namespace Detail
{
inline bool Positive(double value) noexcept { return std::isfinite(value) && value>0.0; }
template<std::size_t N> inline bool Finite(const std::array<double,N>& point) noexcept
{ for(double value:point)if(!std::isfinite(value))return false;return true; }
inline double Norm(const Point3& value) noexcept
{ return std::hypot(std::hypot(value[0],value[1]),value[2]); }
inline std::uint32_t ClassifyClockFault(std::uint64_t previousTick,
    std::uint64_t previousNowUs,const Input& input,
    std::uint64_t minimumTickUs,std::uint64_t maximumTickUs) noexcept
{
    std::uint32_t reasons = 0U;
    if(input.tick <= previousTick) reasons |= TickNonIncreasing;
    else if(input.tick - previousTick != 1ULL) reasons |= TickGap;
    if(input.nowUs <= previousNowUs) reasons |= TimeNonIncreasing;
    else {
        const std::uint64_t delta = input.nowUs - previousNowUs;
        if(delta < minimumTickUs) reasons |= IntervalTooShort;
        if(delta > maximumTickUs) reasons |= IntervalTooLong;
    }
    return reasons;
}
inline bool IsNominal(Phase phase) noexcept
{ return phase>=Phase::InitialDepth && phase<=Phase::AdvanceCenter; }
inline bool IsRecovery(Phase phase) noexcept
{return phase>=Phase::RecoveryBrake&&phase<=Phase::InspectionJogReturn;}
inline bool IsExcursion(Phase phase) noexcept
{ return phase==Phase::HoldToCenter || phase==Phase::HoldToStart || phase==Phase::ResumeToCenter || phase==Phase::ResumeToAnchor ||
    phase==Phase::FlushLift || phase==Phase::FlushReturn || phase==Phase::FlushToCenter ||
    phase==Phase::FlushFromCenter || phase==Phase::FlushPathRetreat || phase==Phase::FlushPathReturn; }
inline bool IsSettling(Phase phase) noexcept
{ return phase==Phase::Settle || phase==Phase::HoldSettle || phase==Phase::FaultSettle || phase==Phase::FlushSettle ||
    phase==Phase::GapHistorySettle || phase==Phase::RecoverySettle; }
inline bool IsFlushWorkPhase(Phase phase) noexcept
{ return phase==Phase::InitialDepth || phase==Phase::CircleEntry || phase==Phase::Orbit; }
inline bool IsFlushMotionPhase(Phase phase) noexcept
{return phase==Phase::FlushToCenter||phase==Phase::FlushLift||phase==Phase::FlushReturn||
    phase==Phase::FlushFromCenter||phase==Phase::FlushPathRetreat||phase==Phase::FlushPathReturn;}
inline bool IsFlushReturnPhase(Phase phase) noexcept
{return phase==Phase::FlushReturn||phase==Phase::FlushFromCenter||phase==Phase::FlushPathReturn;}
// A stopped interior cursor can be closer to an endpoint than the global
// station representation can retain. Coalesce only that station identity,
// under the existing physical arithmetic budget; never rewrite the cursor or
// its exact saved command. The final return still targets that saved command.
inline bool ResolveStoppedFlushStation(const Geometry& geometry,NCG76SphereRetreat::Stage stage,
    unsigned layer,double progress,double length,const Point8& anchor,double height,double& station) noexcept
{
    if(!Positive(length)||!std::isfinite(progress)||progress<0.0||progress>length||
       !Finite(anchor)||!Positive(height))return false;
    const double fraction=progress/length;
    if(NCG76SphereRetreat::Station(geometry,stage,layer,fraction,station))return true;
    if(!(fraction>0.0&&fraction<1.0))return false;
    NCG76SphereRetreat::Span span{};
    if(!NCG76SphereRetreat::GetSpan(geometry,stage,layer,span))return false;
    const double rounded=span.start+(span.end-span.start)*fraction;
    const bool atStart=rounded==span.start,atEnd=rounded==span.end;
    if(!atStart&&!atEnd)return false;
    const double residual=atStart?progress:length-progress;
    const double budget=(std::min)(geometry.arithmeticErrorBoundMm,height/64.0);
    const double endpointStation=atStart?span.start:span.end;
    const double interiorStation=std::nextafter(endpointStation,atStart?
        std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity());
    const double stationUlp=std::fabs(interiorStation-endpointStation);
    if(!Positive(budget)||!Positive(stationUlp)||!std::isfinite(residual)||residual<0.0||
       residual>(std::min)(budget,stationUlp))return false;
    Point8 endpoint{},center{};
    if(!NCG76SphereRetreat::Evaluate(geometry,span,atStart?0.0:1.0,endpoint,center))return false;
    // Exact coordinate aliases can retain the endpoint station unchanged.
    if(std::memcmp(endpoint.data(),anchor.data(),sizeof(Point8))==0){station=rounded;return true;}
    if(std::memcmp(endpoint.data()+3U,anchor.data()+3U,5U*sizeof(double))!=0)return false;
    const double projectionBudget=(std::min)(budget,stationUlp);
    Point3 error{};for(unsigned axis=0U;axis<3U;++axis)error[axis]=endpoint[axis]-anchor[axis];
    if(!Finite(error)||Norm(error)>projectionBudget)return false;
    // A distinct stopped point must never gain unvisited station distance.
    // Use the representable floor, and prove its coordinate projection under
    // the SAME admitted error budget; no new physical phase or cursor change.
    const double floor=atStart?span.start:interiorStation;
    if(atEnd&&!(floor>span.start&&floor<span.end))return false;
    Point8 projected{};
    if(!NCG76SphereRetreat::Evaluate(geometry,span,NCG76SphereRetreat::Fraction(span,floor),projected,center)||
       std::memcmp(projected.data()+3U,anchor.data()+3U,5U*sizeof(double))!=0)return false;
    for(unsigned axis=0U;axis<3U;++axis)error[axis]=projected[axis]-anchor[axis];
    const double projection=Norm(error),target=floor-height;
    const double distanceError=std::fabs((floor-target)-height);
    if(!Finite(error)||!std::isfinite(target)||!std::isfinite(distanceError)||
       projection>projectionBudget||projection+distanceError>budget)return false;
    station=floor;return true;
}
}

inline bool ValidateRequest(const Request& r) noexcept
{
    if(r.parameterFeeds){
        if((r.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
            r.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
           (r.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc&&r.retreatMode!=2U)||
           !std::isfinite(r.nominalFeedCapMmMin)||r.nominalFeedCapMmMin<0.0||r.nominalFeedCapMmMin>100.0)return false;
    }else if(r.nominalFeedCapMmMin!=0.0)return false;
    if(!r.session || !NCG76SphereRuntimeGeometry::Valid(r.geometry) ||
       !Detail::Positive(r.feedMmMin) || r.feedMmMin!=r.geometry.feedMmMin ||
       !Detail::Positive(r.returnFeedMmMin) || r.returnFeedMmMin>r.feedMmMin ||
       !Detail::Positive(r.velocityCapMmS) || !Detail::Positive(r.accelerationMmS2) ||
       !Detail::Positive(r.decelerationMmS2) || !std::isfinite(r.accelerationMmS2+r.decelerationMmS2) ||
       !std::isfinite(1.0/r.accelerationMmS2+1.0/r.decelerationMmS2) ||
       !Detail::Positive(r.positionToleranceMm) || !Detail::Positive(r.rotaryToleranceDeg) ||
       !r.settleSamples || r.settleSamples>1000000U ||
       !std::isfinite(r.minimumSettleSeconds) || r.minimumSettleSeconds<0.0 ||
       !Detail::Positive(r.settleTimeoutSeconds) || !Detail::Positive(r.maximumCycleSeconds) ||
       r.minimumSettleSeconds>r.settleTimeoutSeconds ||
       r.maximumCycleSeconds<r.settleTimeoutSeconds || !r.minimumTickUs ||
       r.minimumTickUs>r.maximumTickUs || r.maximumTickUs>1000000ULL ||
       r.settleTimeoutSeconds<=static_cast<double>(r.maximumTickUs)*1e-6 ||
       !NCG76SphereRetreat::ValidMode(r.geometry,r.retreatMode) || (r.travelMask&~0xffU)!=0U || !Detail::Finite(r.travelMin) || !Detail::Finite(r.travelMax)) return false;
    for(unsigned a=0U;a<8U;++a)
        if((r.travelMask&(1U<<a)) &&
           (r.travelMin[a]>=r.travelMax[a] || r.geometry.boundsMinMCS[a]<r.travelMin[a] ||
            r.geometry.boundsMaxMCS[a]>r.travelMax[a])) return false;
    return true;
}

inline double MaximumFlushHeightMm(const Request& request) noexcept
{
    return request.parameterFeeds&&
        (request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line||
         request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc)?10.0:2.0;
}

// Both the ordinary and periodically selected height must independently fit
// the parent path, full travel envelope and arithmetic budget. Checking only
// max(height) would miss a smaller B4 height below the admitted sphere radius.
inline bool ValidateFlushHeight(const Request& r,double height) noexcept
{
    if(!Detail::Positive(height)||height>MaximumFlushHeightMm(r)||!NCG76SphereRetreat::ValidMode(r.geometry,r.retreatMode))return false;
    // Zero G01 can only settle and complete. Typed configuration remains
    // checked, while no unused lift or retained-path capacity is invented.
    if(r.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&r.geometry.travelMm==0.0)return true;
    if(r.retreatMode==2U){
        double maximum=0.0;
        if(!NCG76SphereRetreat::MaximumFlushStation(r.geometry,maximum)||height>maximum)return false;
        const double ulp=std::nextafter(maximum,std::numeric_limits<double>::infinity())-maximum;
        const double delta=maximum-(maximum-height);
        const double budget=(std::min)(r.geometry.arithmeticErrorBoundMm,height/64.0);
        return Detail::Positive(budget)&&ulp<=budget&&height>64.0*(std::max)(ulp,r.geometry.coordinateErrorBoundMm)&&
            Detail::Positive(delta)&&std::fabs(delta-height)<=budget;
    }
    if(r.retreatMode==4U&&height<r.geometry.radiusMm)return false;
    // G02/G03 B0 freezes its instantaneous tangent only after braking.
    // Admit every possible tangent direction for this immutable arc before
    // motion starts. B1 has the separate, fixed machine +Z meaning.
    if(r.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc&&r.retreatMode==0U){
        Point3 error{};bool resolved=false;
        for(unsigned a=0U;a<3U;++a){
            const double low=r.geometry.boundsMinMCS[a]-height,high=r.geometry.boundsMaxMCS[a]+height;
            const double outwardLow=std::nextafter(low,-std::numeric_limits<double>::infinity());
            const double outwardHigh=std::nextafter(high,std::numeric_limits<double>::infinity());
            if(!std::isfinite(outwardLow)||!std::isfinite(outwardHigh)||
               ((r.travelMask&(1U<<a))&&(outwardLow<r.travelMin[a]||outwardHigh>r.travelMax[a])))return false;
            const double lo=r.geometry.boundsMinMCS[a]-low,hi=high-r.geometry.boundsMaxMCS[a];
            resolved=resolved||lo>0.0||hi>0.0;
            error[a]=(std::max)(std::fabs(lo-height),std::fabs(hi-height));
        }
        const double budget=(std::min)(r.geometry.arithmeticErrorBoundMm,height/64.0);
        return resolved&&Detail::Positive(budget)&&Detail::Norm(error)<=budget;
    }
    Point3 lift{};
    if(r.retreatMode==1U)lift[2]=height;
    else for(unsigned a=0U;a<3U;++a)lift[a]=-r.geometry.axis[a]*height;
    if(!Detail::Finite(lift) || !Detail::Positive(Detail::Norm(lift)))return false;
    bool resolvedLow=false,resolvedHigh=false;Point3 rounding{};
    for(unsigned a=0U;a<3U;++a)
    {
        const double low=r.geometry.boundsMinMCS[a]+(std::min)(0.0,lift[a]);
        const double high=r.geometry.boundsMaxMCS[a]+(std::max)(0.0,lift[a]);
        if(!std::isfinite(low) || !std::isfinite(high))return false;
        const double outwardLow=std::nextafter(low,-std::numeric_limits<double>::infinity());
        const double outwardHigh=std::nextafter(high,std::numeric_limits<double>::infinity());
        if(!std::isfinite(outwardLow) || !std::isfinite(outwardHigh))return false;
        if((r.travelMask&(1U<<a)) && (outwardLow<r.travelMin[a] || outwardHigh>r.travelMax[a]))return false;
        const double lowDelta=(r.geometry.boundsMinMCS[a]+lift[a])-r.geometry.boundsMinMCS[a];
        const double highDelta=(r.geometry.boundsMaxMCS[a]+lift[a])-r.geometry.boundsMaxMCS[a];
        if(!std::isfinite(lowDelta) || !std::isfinite(highDelta))return false;
        rounding[a]=(std::max)(std::fabs(lowDelta-lift[a]),std::fabs(highDelta-lift[a]));
        resolvedLow=resolvedLow||lowDelta!=0.0;resolvedHigh=resolvedHigh||highDelta!=0.0;
    }
    const double budget=(std::min)(r.geometry.arithmeticErrorBoundMm,height/64.0);
    return resolvedLow && resolvedHigh && Detail::Positive(budget) && Detail::Norm(rounding)<=budget;
}

inline bool ValidateFlushConfig(const Request& r,const FlushConfig& config) noexcept
{
    const double maximumHeight=MaximumFlushHeightMm(r);
    if(!config.enabled)return !config.independentFeedsEnabled&&!config.finalApproachEnabled&&!config.initialSlowEnabled&&!config.deepFlushEnabled&&
        config.deepFlushCycleInterval==0U&&config.deepFlushHeightMm==0.0&&
        (r.retreatMode==0U||(r.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&r.geometry.travelMm==0.0))&&ValidateRequest(r);
    if(config.independentFeedsEnabled){
        const double feeds[]={config.centerRetreatFeedMmMin,config.axisRetreatFeedMmMin,
            config.centerReturnFeedMmMin,config.axisReturnFeedMmMin,config.pathRetreatFeedMmMin};
        for(double feed:feeds)if(!Detail::Positive(feed)||feed>r.feedMmMin)return false;
    }
    if(config.finalApproachEnabled&&(!Detail::Positive(config.finalApproachFeedMmMin)||
        config.finalApproachFeedMmMin>r.feedMmMin||!std::isfinite(config.finalApproachDistanceMm)||
        config.finalApproachDistanceMm<0.0))return false;
    if(config.initialSlowEnabled){
        if(!config.independentFeedsEnabled||!Detail::Positive(config.initialSlowFeedMmMin)||
           !Detail::Positive(config.initialSlowFeedMmMin/60.0)||config.initialSlowFeedMmMin>r.feedMmMin||
           !Detail::Positive(config.initialSlowDistanceMm)||config.initialSlowDistanceMm>900.0)return false;
    }else if(config.initialSlowFeedMmMin!=0.0||config.initialSlowDistanceMm!=0.0)return false;
    if(config.deepFlushEnabled){
        if(!config.independentFeedsEnabled||config.deepFlushCycleInterval==0U||config.deepFlushCycleInterval>20U||
           !Detail::Positive(config.deepFlushHeightMm)||config.deepFlushHeightMm>maximumHeight)return false;
    }else if(config.deepFlushCycleInterval!=0U||config.deepFlushHeightMm!=0.0)return false;
    if(!ValidateRequest(r) || !Detail::Positive(config.heightMm) || config.heightMm>maximumHeight ||
       !Detail::Positive(config.feedMmMin) || config.feedMmMin>r.feedMmMin ||
       r.settleTimeoutSeconds<=(std::max)(0.05,r.minimumSettleSeconds)+static_cast<double>(r.maximumTickUs)*1.e-6)return false;
    return ValidateFlushHeight(r,config.heightMm)&&
        (!config.deepFlushEnabled||ValidateFlushHeight(r,config.deepFlushHeightMm));
}

inline bool Cycle::InsideTravel(const Point8& point) const noexcept
{
    if(!Detail::Finite(point))return false;
    const bool inspection=IsInspectionTravelActive();
    for(unsigned a=0U;a<8U;++a){
        if(inspection&&a<3U){
            if(point[a]<recovery_.inspectionMinimum[a]||point[a]>recovery_.inspectionMaximum[a])return false;
        }else if((request_.travelMask&(1U<<a)) && (point[a]<request_.travelMin[a] || point[a]>request_.travelMax[a]))return false;
    }
    return true;
}
inline bool Cycle::AtCommand(const Point8& actual) const noexcept
{
    if(!Detail::Finite(actual))return false;
    for(unsigned a=0U;a<8U;++a)
        if(std::fabs(actual[a]-output_.commandMCS[a])>(a<3U?request_.positionToleranceMm:request_.rotaryToleranceDeg))return false;
    return true;
}
inline void Cycle::Fail(Fault fault,bool emergency) noexcept
{
    if(recovery_.enabled){
        recovery_.active=false;recovery_.evaluationGeometry=nullptr;
        feedback_.holdRecoveryPhase=10U;feedback_.inspectionJogActive=false;
        feedback_.inspectionTraceValid=false;
    }
    if(feedback_.flushActive)
    {
        if(!finish_.forcedFlush)feedback_.flushAbandonedSequence=feedback_.flushRequestSequence;
        feedback_.flushActive=false;
    }
    if(feedback_.finishRequiredCount){feedback_.finishActive=false;feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::Fault);}
    finish_.forcedFlush=finish_.returnedPending=false;
    feedback_.flushCheckpointSaved=false;
    feedback_.gapHistoryActive=false;
    feedback_.previousParentHistoryReady=false;previousPrimitiveLegActive_=false;
    feedback_.fault=fault;feedback_.phase=Phase::Fault;feedback_.held=false;
    leg_.speed=0.0;leg_.accelerationNow=0.0;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);
    output_.emergencyStopRequired=emergency;output_.cancelled=fault==Fault::Cancelled;
    RefreshOutput();
}
inline void Cycle::RefreshOutput() noexcept
{
    const auto phase=feedback_.phase;
    if(feedback_.flushActive&&feedback_.flushCheckpointSaved&&
       (phase==Phase::FlushPathRetreat||phase==Phase::FlushPathReturn||phase==Phase::FlushHoldBrake))
    {
        const double fraction=leg_.length==0.0?1.0:leg_.position/leg_.length;
        feedback_.flushPathStationMm=leg_.position==leg_.length?flushPathLegEnd_:
            flushPathLegStart_+(flushPathLegEnd_-flushPathLegStart_)*fraction;
    }
    feedback_.flushSelectedFeedMmMin=feedback_.flushTargetFeedMmMin=feedback_.flushRemainingMm=0.0;
    feedback_.flushApproachActive=feedback_.flushInitialSlowActive=false;
    if(feedback_.flushActive&&feedback_.flushCheckpointSaved&&Detail::IsFlushMotionPhase(phase)){
        feedback_.flushSelectedFeedMmMin=leg_.speedLimit*60.0;
        feedback_.flushTargetFeedMmMin=feedback_.flushSelectedFeedMmMin;
        feedback_.flushRemainingMm=phase==Phase::FlushPathReturn||phase==Phase::FlushPathRetreat?
            feedback_.flushAnchorStationMm-feedback_.flushPathStationMm:leg_.length-leg_.position;
        if(flush_.finalApproachEnabled&&flush_.finalApproachDistanceMm>0.0&&Detail::IsFlushReturnPhase(phase)){
            feedback_.flushApproachActive=feedback_.flushRemainingMm<=flush_.finalApproachDistanceMm;
            const double slow=(std::min)(leg_.speedLimit,flush_.finalApproachFeedMmMin/60.0);
            const double distance=(std::max)(0.0,leg_.approachStart-leg_.position);
            const double cap=std::sqrt(slow*slow+2.0*leg_.deceleration*distance);
            feedback_.flushTargetFeedMmMin=60.0*(std::min)(leg_.speedLimit,cap);
        }
        // E20 is measured from the original stopped anchor along the whole
        // outbound route, including B3/B4's center/diagonal leg. A geometry
        // corner never restarts the initial low-speed distance.
        if(flush_.initialSlowEnabled&&flush_.initialSlowDistanceMm>0.0&&
           (phase==Phase::FlushLift||phase==Phase::FlushToCenter||phase==Phase::FlushPathRetreat)){
            double start=0.0;
            if(FlushOutboundStartDistance(phase,start)){
                const double distance=start+leg_.position;
                feedback_.flushInitialSlowActive=distance<flush_.initialSlowDistanceMm||
                    (distance==flush_.initialSlowDistanceMm&&leg_.position==leg_.length&&leg_.approachStart==leg_.length);
                if(feedback_.flushInitialSlowActive)feedback_.flushTargetFeedMmMin=60.0*
                    (std::min)(leg_.speedLimit,flush_.initialSlowFeedMmMin/60.0);
            }
        }
    }
    output_.active=initialized_ && phase!=Phase::Idle && phase!=Phase::Complete && phase!=Phase::Fault;
    output_.complete=phase==Phase::Complete;output_.faulted=phase==Phase::Fault;
    output_.stopped=leg_.speed==0.0;
    output_.zeroDemand=Detail::IsSettling(phase) || phase==Phase::HeldAtStart ||
        phase==Phase::Complete || phase==Phase::Fault || phase==Phase::Idle;
    feedback_.held=phase==Phase::HeldAtStart;
    feedback_.phaseProgressMm=leg_.position;feedback_.phaseLengthMm=leg_.length;feedback_.speedMmS=std::fabs(leg_.speed);
    feedback_.signedSpeedMmS=leg_.speed;
    feedback_.previousParentHistoryActive=feedback_.gapHistoryActive&&previousPrimitiveLegActive_;
    feedback_.previousParentLengthMm=feedback_.previousParentHistoryReady?previousPrimitive_.travelMm:0.0;
    feedback_.previousParentProgressMm=feedback_.previousParentHistoryReady?
        (previousPrimitiveLegActive_?leg_.position:previousPrimitive_.travelMm):0.0;
    if((!Detail::IsFlushWorkPhase(phase)&&phase!=Phase::GapHistory)||feedback_.holdActive||feedback_.flushActive)feedback_.gapApplied=false;
    // History movement is an excursion around the unchanged current forward
    // cursor. Its old layer/stage must not earn new work or layer completion.
    if(feedback_.gapHistoryActive)
    {feedback_.nominalProgressMm=historyNominalLeg_.position;feedback_.nominalLengthMm=historyNominalLeg_.length;}
    else if(feedback_.flushActive && feedback_.flushCheckpointSaved)
    {feedback_.nominalProgressMm=savedLeg_.position;feedback_.nominalLengthMm=savedLeg_.length;}
    else if(Detail::IsNominal(phase) || phase==Phase::HoldBrake || phase==Phase::FlushBrake ||
        (phase==Phase::FlushSettle && !feedback_.flushCheckpointSaved) || (phase==Phase::Settle && !feedback_.holdActive))
    {feedback_.nominalProgressMm=leg_.position;feedback_.nominalLengthMm=leg_.length;}
    else if(feedback_.holdActive)
    {feedback_.nominalProgressMm=savedLeg_.position;feedback_.nominalLengthMm=savedLeg_.length;}
    if(recovery_.enabled)RefreshRecoveryFeedback();
}
inline bool Cycle::StartProfile(Leg& leg) noexcept
{
    leg.approachProfile=false;leg.approachStart=leg.approachSpeed=0.0;
    leg.basePosition=leg.position;leg.elapsed=0.0;leg.speed=0.0;leg.accelerationNow=0.0;
    const double remaining=leg.length-leg.position;
    if(!std::isfinite(remaining) || remaining<0.0)return false;
    if(remaining==0.0)
    {leg.peak=leg.accelerationTime=leg.cruiseTime=leg.decelerationTime=leg.accelerationDistance=0.0;return true;}
    const double reciprocal=1.0/leg.acceleration+1.0/leg.deceleration;
    const double triangle=std::sqrt((2.0*remaining)/reciprocal);
    leg.peak=(std::min)(leg.speedLimit,triangle);
    if(!Detail::Positive(leg.peak))return false;
    leg.accelerationTime=leg.peak/leg.acceleration;
    leg.decelerationTime=leg.peak/leg.deceleration;
    leg.accelerationDistance=0.5*leg.accelerationTime*leg.peak;
    const double decelerationDistance=0.5*leg.decelerationTime*leg.peak;
    const double cruiseDistance=(std::max)(0.0,remaining-leg.accelerationDistance-decelerationDistance);
    leg.cruiseTime=cruiseDistance/leg.peak;
    const double total=leg.accelerationTime+leg.cruiseTime+leg.decelerationTime;
    return Detail::Positive(leg.accelerationTime) && Detail::Positive(leg.decelerationTime) &&
        std::isfinite(leg.accelerationDistance) && std::isfinite(decelerationDistance) &&
        std::isfinite(leg.cruiseTime) && leg.cruiseTime>=0.0 && Detail::Positive(total) && total<=request_.maximumCycleSeconds;
}
// A staged flush profile has two analytic parts sharing one nonzero boundary
// velocity. A timer callback crossing E13/E20 evaluates the second part directly;
// no artificial zero velocity, extra phase or physical settle is inserted.
inline bool Cycle::BuildProfilePart(double length,double startSpeed,double endSpeed,double speedLimit,
    double acceleration,double deceleration,ProfilePart& part) noexcept
{
    part=ProfilePart{};part.length=length;part.initialSpeed=startSpeed;part.finalSpeed=endSpeed;
    if(!std::isfinite(length)||length<0.0||!std::isfinite(startSpeed)||startSpeed<0.0||
       !std::isfinite(endSpeed)||endSpeed<0.0||!Detail::Positive(speedLimit)||
       !Detail::Positive(acceleration)||!Detail::Positive(deceleration)||startSpeed>speedLimit||endSpeed>speedLimit)return false;
    if(length==0.0)return startSpeed==endSpeed;
    const double peakSquared=(2.0*length+startSpeed*(startSpeed/acceleration)+
        endSpeed*(endSpeed/deceleration))/(1.0/acceleration+1.0/deceleration);
    if(!std::isfinite(peakSquared)||peakSquared<=0.0)return false;
    part.peak=(std::min)(speedLimit,std::sqrt(peakSquared));
    // Endpoint speed was independently bounded by both adjacent lengths.
    // Remove only square-root roundoff when that bound is exactly active.
    part.peak=(std::max)(part.peak,(std::max)(startSpeed,endSpeed));
    part.accelerationTime=(part.peak-startSpeed)/acceleration;
    part.decelerationTime=(part.peak-endSpeed)/deceleration;
    part.accelerationDistance=(part.peak+startSpeed)*part.accelerationTime*0.5;
    const double decelDistance=(part.peak+endSpeed)*part.decelerationTime*0.5;
    const double rounding=64.0*(std::numeric_limits<double>::epsilon)()*(std::max)(length,
        (std::max)(part.accelerationDistance,decelDistance));
    const double cruiseDistance=length-part.accelerationDistance-decelDistance;
    if(!std::isfinite(cruiseDistance)||cruiseDistance < -rounding)return false;
    part.cruiseTime=(std::max)(0.0,cruiseDistance)/part.peak;
    part.duration=part.accelerationTime+part.cruiseTime+part.decelerationTime;
    return std::isfinite(part.accelerationDistance)&&std::isfinite(part.cruiseTime)&&Detail::Positive(part.duration)&&
        part.duration<=request_.maximumCycleSeconds;
}
inline bool Cycle::EvaluateProfilePart(const ProfilePart& part,double elapsed,double& position,double& speed,double& acceleration) const noexcept
{
    if(!std::isfinite(elapsed)||elapsed<0.0)return false;
    if(elapsed>=part.duration){position=part.length;speed=part.finalSpeed;acceleration=0.0;return true;}
    if(elapsed<part.accelerationTime){
        position=part.initialSpeed*elapsed+0.5*leg_.acceleration*elapsed*elapsed;
        speed=part.initialSpeed+leg_.acceleration*elapsed;acceleration=leg_.acceleration;
    }else if(elapsed<part.accelerationTime+part.cruiseTime){
        position=part.accelerationDistance+part.peak*(elapsed-part.accelerationTime);
        speed=part.peak;acceleration=0.0;
    }else{
        const double remaining=part.duration-elapsed;
        position=part.length-part.finalSpeed*remaining-0.5*leg_.deceleration*remaining*remaining;
        speed=part.finalSpeed+leg_.deceleration*remaining;acceleration=-leg_.deceleration;
    }
    return std::isfinite(position)&&std::isfinite(speed)&&std::isfinite(acceleration);
}
inline double Cycle::FlushFeed(Phase phase) const noexcept
{
    if(!flush_.independentFeedsEnabled)return flush_.feedMmMin;
    double feed=flush_.feedMmMin;
    if(phase==Phase::FlushToCenter)feed=flush_.centerRetreatFeedMmMin;
    else if(phase==Phase::FlushFromCenter)feed=flush_.centerReturnFeedMmMin;
    else if(phase==Phase::FlushLift)feed=flush_.axisRetreatFeedMmMin;
    else if(phase==Phase::FlushReturn||phase==Phase::FlushPathReturn)feed=flush_.axisReturnFeedMmMin;
    else if(phase==Phase::FlushPathRetreat)feed=flush_.pathRetreatFeedMmMin;
    return (std::min)(feed,request_.feedMmMin);
}
// Physical path distance already completed before this outbound leg. All
// sources are frozen per flush; no catalog, allocation or layer walk on RT.
inline bool Cycle::FlushOutboundStartDistance(Phase phase,double& distance) const noexcept
{
    distance=0.0;
    if(phase==Phase::FlushPathRetreat)distance=feedback_.flushAnchorStationMm-flushPathLegStart_;
    else if(phase==Phase::FlushLift&&(request_.retreatMode==3U||request_.retreatMode==4U)){
        Point3 radial{};for(unsigned a=0U;a<3U;++a)radial[a]=flushWaypoint_[a]-savedAnchor_[a];
        distance=Detail::Norm(radial);
    }else if(phase!=Phase::FlushLift&&phase!=Phase::FlushToCenter)return false;
    return std::isfinite(distance)&&distance>=0.0;
}
inline bool Cycle::ConfigureFlushProfile(Phase phase) noexcept
{
    const bool outward=phase==Phase::FlushLift||phase==Phase::FlushToCenter||phase==Phase::FlushPathRetreat;
    const bool initial=outward&&flush_.initialSlowEnabled&&flush_.initialSlowDistanceMm>0.0;
    const bool final=Detail::IsFlushReturnPhase(phase)&&flush_.finalApproachEnabled&&flush_.finalApproachDistanceMm>0.0;
    const double remaining=leg_.length-leg_.position;
    if((!initial&&!final)||remaining==0.0)return true;
    if(!std::isfinite(remaining)||remaining<0.0){Fail(Fault::Numeric);return false;}
    double prefix=0.0,slow=0.0;
    if(initial){
        double completed=0.0;
        if(!FlushOutboundStartDistance(phase,completed)){Fail(Fault::Numeric);return false;}
        completed+=leg_.position;
        prefix=(std::min)(remaining,(std::max)(0.0,flush_.initialSlowDistanceMm-completed));
        slow=(std::min)(leg_.speedLimit,flush_.initialSlowFeedMmMin/60.0);
    }else{
        double tail=(std::min)(leg_.length,flush_.finalApproachDistanceMm);
        if(phase==Phase::FlushPathReturn){
            const double zone=feedback_.flushAnchorStationMm-flush_.finalApproachDistanceMm;
            if(zone>=flushPathLegEnd_)tail=0.0;
            else if(zone<=flushPathLegStart_)tail=leg_.length;
            else tail=leg_.length*((flushPathLegEnd_-zone)/(flushPathLegEnd_-flushPathLegStart_));
        }
        prefix=(std::min)(remaining,(std::max)(0.0,leg_.length-tail-leg_.position));
        slow=(std::min)(leg_.speedLimit,flush_.finalApproachFeedMmMin/60.0);
    }
    if(!std::isfinite(prefix)||prefix<0.0||prefix>remaining||!Detail::Positive(slow))
    {Fail(Fault::Numeric);return false;}
    // The shared pair always follows geometric travel order. Outbound is a
    // low-speed prefix then normal tail; return is normal then E13 slow tail.
    // A timer crossing either boundary evaluates the second part directly.
    leg_.approachStart=leg_.position+prefix;leg_.approachSpeed=slow;
    if((initial?prefix==0.0:prefix==remaining)||slow>=leg_.speedLimit)return true;
    const double tail=remaining-prefix;
    const double boundary=(std::min)(slow,(std::min)(std::sqrt(leg_.acceleration)*std::sqrt(2.0*prefix),
        std::sqrt(leg_.deceleration)*std::sqrt(2.0*tail)));
    if(!BuildProfilePart(prefix,0.0,boundary,initial?slow:leg_.speedLimit,leg_.acceleration,leg_.deceleration,leg_.approachPrefix)||
       !BuildProfilePart(tail,boundary,0.0,initial?leg_.speedLimit:slow,leg_.acceleration,leg_.deceleration,leg_.approachTail)||
       leg_.approachPrefix.duration+leg_.approachTail.duration>request_.maximumCycleSeconds)
    {Fail(Fault::Numeric);return false;}
    leg_.approachProfile=true;return true;
}
inline bool Cycle::StartLine(const Point8& end,Phase phase,double feed) noexcept
{
    if(!InsideTravel(end)) {Fail(Fault::TravelLimit);return false;}
    Leg next{};next.start=output_.commandMCS;next.end=end;
    for(unsigned a=0U;a<3U;++a)next.direction[a]=end[a]-next.start[a];
    for(unsigned a=3U;a<8U;++a)
        if(end[a]!=request_.geometry.startMCS[a] || next.start[a]!=request_.geometry.startMCS[a])
        {Fail(Fault::Numeric);return false;}
    next.length=Detail::Norm(next.direction);
    if(!std::isfinite(next.length)) {Fail(Fault::Numeric);return false;}
    if(next.length>0.0)for(double& value:next.direction)value/=next.length;
    next.speedLimit=(std::min)(feed/60.0,request_.velocityCapMmS);
    next.acceleration=request_.accelerationMmS2;next.deceleration=request_.decelerationMmS2;
    if(!Detail::Positive(next.speedLimit) || !StartProfile(next)) {Fail(Fault::Numeric);return false;}
    leg_=next;feedback_.phase=phase;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);
    RefreshOutput();return true;
}
inline bool Cycle::StartNominal(Phase phase) noexcept
{
    if(request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        if(phase!=Phase::InitialDepth||feedback_.layerIndex!=0U){Fail(Fault::Numeric);return false;}
        feedback_.nominalPhase=phase;
        const double nominalFeed=request_.parameterFeeds&&request_.nominalFeedCapMmMin>0.0?
            (std::min)(request_.feedMmMin,request_.nominalFeedCapMmMin):request_.feedMmMin;
        if(request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line)
            return StartLine(request_.geometry.endMCS,phase,nominalFeed);
        if(request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc){Fail(Fault::Numeric);return false;}
        Leg next{};next.circle=true;next.start=request_.geometry.startMCS;next.end=request_.geometry.endMCS;
        next.center=request_.geometry.startMCS;
        for(unsigned a=0U;a<3U;++a)next.center[a]=request_.geometry.arcCenterMCS[a];
        next.circleEndAngle=request_.geometry.arcSweepRad;next.length=request_.geometry.travelMm;
        const double normalBudget=0.5*(std::min)(request_.accelerationMmS2,request_.decelerationMmS2);
        next.acceleration=next.deceleration=normalBudget;
        const double radialSpeed=std::sqrt(normalBudget)*std::sqrt(request_.geometry.radiusMm);
        next.speedLimit=(std::min)((std::min)(nominalFeed/60.0,request_.velocityCapMmS),radialSpeed);
        if(!Detail::Positive(next.length)||!Detail::Positive(next.speedLimit)||!Detail::Positive(radialSpeed)||!StartProfile(next))
        {Fail(Fault::Numeric);return false;}
        leg_=next;feedback_.phase=phase;
        output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();return true;
    }
    NCG76SphereRuntimeGeometry::Layer layer{};
    if(!NCG76SphereRuntimeGeometry::LayerAt(request_.geometry,feedback_.layerIndex,layer))
    {Fail(Fault::Numeric);return false;}
    feedback_.nominalPhase=phase;
    if(phase==Phase::InitialDepth)return StartLine(request_.geometry.endMCS,phase,request_.feedMmMin);
    if(phase==Phase::InitialRetreat)return StartLine(layer.center,phase,request_.feedMmMin);
    if(phase==Phase::CircleEntry)
    {
        Point8 point{};Point3 tangent{},curvature{};
        if(!NCG76SphereRuntimeGeometry::CircleAt(request_.geometry,feedback_.layerIndex,0.0,point,tangent,curvature))
        {Fail(Fault::Numeric);return false;}
        if(!StartLine(point,phase,request_.feedMmMin))return false;
        leg_.center=layer.center;return true;
    }
    if(phase==Phase::Orbit)
    {
        Leg next{};next.circle=true;next.circleLayer=feedback_.layerIndex;next.start=next.end=output_.commandMCS;next.center=layer.center;
        next.length=layer.arcLengthMm;
        // Around a circle an individual axis may decelerate while the scalar
        // path speed accelerates. Use the smaller physical cap in both scalar
        // directions; reserve half for tangent and half for normal v^2/r.
        const double normalBudget=0.5*(std::min)(request_.accelerationMmS2,request_.decelerationMmS2);
        next.acceleration=next.deceleration=normalBudget;
        const double radialSpeed=std::sqrt(normalBudget)*std::sqrt(layer.radiusMm);
        next.speedLimit=(std::min)((std::min)(request_.feedMmMin/60.0,request_.velocityCapMmS),radialSpeed);
        if(!Detail::Positive(next.length) || !Detail::Positive(radialSpeed) || !Detail::Positive(next.speedLimit) ||
           !StartProfile(next)) {Fail(Fault::Numeric);return false;}
        leg_=next;feedback_.phase=phase;
        output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();return true;
    }
    if(phase==Phase::ReturnCenter)
    {
        if(!StartLine(layer.center,phase,request_.feedMmMin))return false;
        leg_.center=layer.center;return true;
    }
    if(phase==Phase::AdvanceCenter)
    {
        NCG76SphereRuntimeGeometry::Layer next{};
        if(!NCG76SphereRuntimeGeometry::LayerAt(request_.geometry,feedback_.layerIndex+1U,next))
        {Fail(Fault::Numeric);return false;}
        return StartLine(next.center,phase,request_.feedMmMin);
    }
    Fail(Fault::Numeric);return false;
}
inline bool Cycle::EvaluateLeg() noexcept
{
    Point3 tangent{},curvature{};
    if(leg_.circle)
    {
        const double fraction=leg_.position/leg_.length;
        const double angle=leg_.position==0.0?leg_.circleStartAngle:leg_.position==leg_.length?leg_.circleEndAngle:
            (std::max)((std::min)(leg_.circleStartAngle,leg_.circleEndAngle),
                (std::min)((std::max)(leg_.circleStartAngle,leg_.circleEndAngle),
                    leg_.circleStartAngle+fraction*(leg_.circleEndAngle-leg_.circleStartAngle)));
        const auto& geometry=ActiveLegGeometry();
        if(!NCG76SphereRuntimeGeometry::CircleAt(geometry,leg_.circleLayer,angle,
            output_.commandMCS,tangent,curvature))return false;
        if(leg_.position==leg_.length)output_.commandMCS=leg_.end;
        else if(leg_.position==0.0)output_.commandMCS=leg_.start;
        if(leg_.circleEndAngle<leg_.circleStartAngle)for(double& x:tangent)x=-x;
    }
    else
    {
        tangent=leg_.direction;
        if(leg_.position==leg_.length)output_.commandMCS=leg_.end;
        else if(leg_.position==0.0)output_.commandMCS=leg_.start;
        else
        {
            output_.commandMCS=leg_.start;
            for(unsigned a=0U;a<3U;++a)output_.commandMCS[a]=leg_.start[a]+leg_.position*leg_.direction[a];
        }
    }
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);
    for(unsigned a=0U;a<3U;++a)
    {
        output_.velocityNative[a]=tangent[a]*leg_.speed;
        output_.accelerationNative[a]=tangent[a]*leg_.accelerationNow+curvature[a]*leg_.speed*leg_.speed;
    }
    return InsideTravel(output_.commandMCS) && Detail::Finite(output_.velocityNative) && Detail::Finite(output_.accelerationNative);
}
inline bool Cycle::AdvanceLeg(double dt,bool brake,bool& finished) noexcept
{
    finished=false;
    const double previous=leg_.position;
    if(brake)
    {
        leg_.stopElapsed+=dt;
        if(leg_.stopElapsed>=leg_.stopTime)
        {leg_.position=leg_.stopPosition;leg_.speed=0.0;leg_.accelerationNow=0.0;finished=true;}
        else
        {
            const double remaining=leg_.stopTime-leg_.stopElapsed;
            leg_.position=leg_.stopPosition-leg_.stopDirection*0.5*leg_.deceleration*remaining*remaining;
            leg_.speed=leg_.stopDirection*leg_.deceleration*remaining;leg_.accelerationNow=-leg_.stopDirection*leg_.deceleration;
        }
    }
    else if(leg_.approachProfile)
    {
        leg_.elapsed+=dt;
        const double total=leg_.approachPrefix.duration+leg_.approachTail.duration;
        if(leg_.elapsed>=total)
        {leg_.position=leg_.length;leg_.speed=0.0;leg_.accelerationNow=0.0;finished=true;}
        else if(leg_.elapsed<leg_.approachPrefix.duration)
        {
            if(!EvaluateProfilePart(leg_.approachPrefix,leg_.elapsed,
                leg_.position,leg_.speed,leg_.accelerationNow))return false;
            leg_.position+=leg_.basePosition;
        }
        else
        {
            if(!EvaluateProfilePart(leg_.approachTail,leg_.elapsed-leg_.approachPrefix.duration,
                leg_.position,leg_.speed,leg_.accelerationNow))return false;
            leg_.position+=leg_.approachStart;
        }
    }
    else
    {
        leg_.elapsed+=dt;
        const double total=leg_.accelerationTime+leg_.cruiseTime+leg_.decelerationTime;
        if(leg_.elapsed>=total)
        {leg_.position=leg_.length;leg_.speed=0.0;leg_.accelerationNow=0.0;finished=true;}
        else if(leg_.elapsed<leg_.accelerationTime)
        {
            leg_.position=leg_.basePosition+0.5*leg_.acceleration*leg_.elapsed*leg_.elapsed;
            leg_.speed=leg_.acceleration*leg_.elapsed;leg_.accelerationNow=leg_.acceleration;
        }
        else if(leg_.elapsed<leg_.accelerationTime+leg_.cruiseTime)
        {
            leg_.position=leg_.basePosition+leg_.accelerationDistance+leg_.peak*(leg_.elapsed-leg_.accelerationTime);
            leg_.speed=leg_.peak;leg_.accelerationNow=0.0;
        }
        else
        {
            const double remaining=total-leg_.elapsed;
            leg_.position=leg_.length-0.5*leg_.deceleration*remaining*remaining;
            leg_.speed=leg_.deceleration*remaining;leg_.accelerationNow=-leg_.deceleration;
        }
    }
    const double tolerance=(std::max)(8.0*LegCoordinateError(),
        64.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,leg_.length));
    const bool reverse=brake&&leg_.stopDirection<0.0;
    if(!std::isfinite(leg_.position) || !std::isfinite(leg_.speed) || (!reverse&&leg_.speed<0.0) ||
       leg_.position<0.0-tolerance || leg_.position>leg_.length+tolerance ||
       (reverse?leg_.position>previous+tolerance:leg_.position<previous-tolerance))return false;
    leg_.position=reverse?(std::max)(0.0,(std::min)(previous,leg_.position)):
        (std::min)(leg_.length,(std::max)(previous,leg_.position));
    return EvaluateLeg();
}
inline bool Cycle::AdvanceGap(double dt,double signedTarget,bool& finished) noexcept
{
    finished=false;feedback_.gapApplied=true;
    if(!Detail::Positive(dt)||!std::isfinite(signedTarget)||!std::isfinite(leg_.speed)||
       !std::isfinite(leg_.position)||leg_.position<0.0||leg_.position>leg_.length)return false;
    const double acceleration=(std::min)(leg_.acceleration,leg_.deceleration);
    const double target=(std::max)(-leg_.speedLimit,(std::min)(leg_.speedLimit,signedTarget/60.0));
    const double old=leg_.speed,speed=std::fabs(old);
    // Reversal is always two distinct passes: brake to zero, then accelerate
    // in the new direction. HOLD/flush may interrupt either side of that zero.
    const bool reversing=old!=0.0&&target*old<0.0;
    const double goal=reversing?0.0:std::fabs(target);
    const double direction=old<0.0?-1.0:old>0.0?1.0:target<0.0?-1.0:1.0;
    const double remaining=direction>0.0?leg_.length-leg_.position:leg_.position;
    const double tolerance=(std::max)(8.0*LegCoordinateError(),
        64.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,leg_.length));
    const double stopDistance=0.5*speed*(speed/acceleration),step=acceleration*dt;
    if(!Detail::Positive(acceleration)||!std::isfinite(stopDistance)||!std::isfinite(step)||
       stopDistance>remaining+tolerance)return false;
    if(speed==0.0&&(goal==0.0||remaining==0.0))
    {
        leg_.accelerationNow=0.0;
        if(remaining==0.0&&direction>0.0&&target>0.0)finished=true;
        return EvaluateLeg();
    }
    const double speedRounding=64.0*std::numeric_limits<double>::epsilon()*(std::max)(speed,step);
    // Once the endpoint stop envelope is tight, stay on the exact maximum
    // deceleration solution. Re-solving its quadratic at tiny remaining
    // distances magnifies harmless coordinate rounding into a speed error.
    if(remaining-stopDistance<=tolerance&&speed>step+speedRounding)
    {
        const double next=speed-step;
        leg_.position+=direction*0.5*(speed+next)*dt;
        leg_.speed=direction*next;leg_.accelerationNow=-direction*acceleration;
        if(leg_.position<0.0-tolerance||leg_.position>leg_.length+tolerance)return false;
        leg_.position=(std::max)(0.0,(std::min)(leg_.length,leg_.position));
        return EvaluateLeg();
    }
    // If braking ends within this callback, integrate only its real duration
    // and hold for the rest. A full-tick trapezoid would overshoot a nearby
    // endpoint even though the continuous stop is physically feasible.
    if(speed<=step+speedRounding&&(remaining<0.5*speed*dt||remaining-stopDistance<=tolerance))
    {
        leg_.position+=direction*stopDistance;leg_.speed=0.0;leg_.accelerationNow=0.0;
        if(remaining-stopDistance<=tolerance)
        {leg_.position=direction>0.0?leg_.length:0.0;finished=direction>0.0&&target>0.0;}
        return EvaluateLeg();
    }
    // The prospective velocity must still be able to stop AFTER this entire
    // trapezoidal step: dt*(v0+v1)/2 + v1^2/(2*a) <= remaining.
    // The stable positive root avoids subtracting almost equal quantities.
    const double c=2.0*acceleration*remaining-step*speed;
    if(!std::isfinite(c)||c<0.0)return false;
    const double divisor=std::sqrt(step*step+4.0*c)+step;
    const double endpointCap=divisor>0.0?2.0*c/divisor:0.0;
    const double lower=(std::max)(0.0,speed-step),upper=speed+step;
    // Near a boundary, a distance-rounding residual is magnified by division
    // through a tiny speed. Compare the unavoidable next-step STOP DISTANCE,
    // in the admitted geometry's metric error budget, rather than adding an
    // arbitrary speed tolerance or allowing stronger acceleration.
    const double unavoidable=0.5*(speed+lower)*dt+0.5*lower*(lower/acceleration);
    if(!std::isfinite(endpointCap)||!std::isfinite(unavoidable)||unavoidable>remaining+tolerance)return false;
    const double next=(std::max)(lower,(std::min)(goal,(std::min)(upper,endpointCap)));
    leg_.position+=direction*0.5*(speed+next)*dt;
    leg_.speed=direction*next;leg_.accelerationNow=direction*(next-speed)/dt;
    if(leg_.position<0.0-tolerance||leg_.position>leg_.length+tolerance)return false;
    leg_.position=(std::max)(0.0,(std::min)(leg_.length,leg_.position));
    return EvaluateLeg();
}
inline bool Cycle::StartBrake(Phase phase) noexcept
{
    const double stopTime=std::fabs(leg_.speed)/leg_.deceleration;
    double stopPosition=leg_.position+0.5*stopTime*leg_.speed;
    const double tolerance=(std::max)(8.0*LegCoordinateError(),
        64.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,leg_.length));
    if(!std::isfinite(stopTime) || !std::isfinite(stopPosition) ||
       stopPosition<0.0-tolerance || stopPosition>leg_.length+tolerance)
    {Fail(Fault::Numeric);return false;}
    stopPosition=(std::max)(0.0,(std::min)(leg_.length,stopPosition));
    leg_.stopTime=stopTime;leg_.stopPosition=stopPosition;leg_.stopElapsed=0.0;
    leg_.stopDirection=leg_.speed<0.0?-1.0:1.0;
    feedback_.phase=phase;RefreshOutput();return true;
}
inline void Cycle::StartSettle(Phase phase,SettleAction action,const Input& input) noexcept
{
    // EDM329: a retained B2 HOLD can finish its final brake step here. Seal
    // that stopped path station before changing to HoldSettle; otherwise
    // the command is at the stop while feedback retains the previous tick.
    const bool retainedPathHold=feedback_.phase==Phase::FlushHoldBrake&&request_.retreatMode==2U&&
        feedback_.flushActive&&feedback_.flushCheckpointSaved;
    if(feedback_.phase==Phase::FlushPathRetreat||feedback_.phase==Phase::FlushPathReturn||retainedPathHold)
    {
        const double fraction=leg_.length==0.0?1.0:leg_.position/leg_.length;
        feedback_.flushPathStationMm=leg_.position==leg_.length?flushPathLegEnd_:
            flushPathLegStart_+(flushPathLegEnd_-flushPathLegStart_)*fraction;
    }
    feedback_.phase=phase;settleAction_=action;settleTick_=input.tick;settlePhysical_=input.physicalSequence;
    settledSamples_=0U;settleElapsed_=0.0;settleProvedSeconds_=0.0;
    leg_.speed=0.0;leg_.accelerationNow=0.0;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();
}
inline bool Cycle::Settled(const Input& input,double dt) noexcept
{
    settleElapsed_+=dt;
    if(settleElapsed_>request_.settleTimeoutSeconds) {Fail(Fault::SettleTimeout);return false;}
    const bool proof=input.tick>settleTick_ && input.physicalSequence>settlePhysical_ &&
        input.physicalSettled && input.zeroFinalSend && input.zeroSendTick>=settleTick_ &&
        input.zeroSendTick<input.tick && input.tick-input.zeroSendTick==1ULL && AtCommand(input.actualMCS);
    settledSamples_=proof?settledSamples_+1U:0U;
    settleProvedSeconds_=proof?settleProvedSeconds_+dt:0.0;
    // Flush motion always needs the full physical zero proof, including pure
    // callers whose legacy AIR Request uses a smaller test-only settle count.
    const bool strict=feedback_.phase==Phase::RecoverySettle||feedback_.phase==Phase::FlushSettle||feedback_.phase==Phase::GapHistorySettle||
        (request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere&&feedback_.phase==Phase::HoldSettle);
    const unsigned requiredSamples=strict?(std::max)(200U,request_.settleSamples):request_.settleSamples;
    const double requiredSeconds=strict?(std::max)(0.05,request_.minimumSettleSeconds):request_.minimumSettleSeconds;
    return settledSamples_>=requiredSamples && settleProvedSeconds_>=requiredSeconds;
}
inline void Cycle::NextNominal(const Input& input) noexcept
{
    // Only a normal forward leg which has completed its ordinary physical
    // stop proof becomes history. HOLD/flush/replayed motion never replaces it.
    if(gap_.crossLegRetreatEnabled&&request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere)
    {
        previousLegValid_=Detail::IsNominal(feedback_.nominalPhase)&&leg_.speed==0.0&&
            leg_.position==leg_.length&&leg_.length>0.0;
        if(previousLegValid_)
        {previousLeg_=leg_;previousPhase_=feedback_.nominalPhase;previousLayer_=feedback_.layerIndex;}
    }
    if(request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        if(feedback_.nominalPhase!=Phase::InitialDepth){Fail(Fault::Numeric);return;}
        if(feedback_.finishRequiredCount){CheckFinishEndpoint(input);return;}
        output_.commandMCS=request_.geometry.endMCS;feedback_.phase=Phase::Complete;
        leg_.speed=0.0;RefreshOutput();return;
    }
    switch(feedback_.nominalPhase)
    {
    case Phase::InitialDepth:(void)StartNominal(Phase::InitialRetreat);break;
    case Phase::InitialRetreat:(void)StartNominal(Phase::CircleEntry);break;
    case Phase::CircleEntry:(void)StartNominal(Phase::Orbit);break;
    case Phase::Orbit:(void)StartNominal(Phase::ReturnCenter);break;
    case Phase::ReturnCenter:(void)StartNominal(Phase::AdvanceCenter);break;
    case Phase::AdvanceCenter:
        ++feedback_.completedLayers;
        if(feedback_.layerIndex+1U==request_.geometry.circleLayerCount)
        {
            output_.commandMCS=request_.geometry.endMCS;feedback_.phase=Phase::Complete;
            leg_.speed=0.0;RefreshOutput();
        }
        else {++feedback_.layerIndex;(void)StartNominal(Phase::CircleEntry);}
        break;
    default:Fail(Fault::Numeric);break;
    }
    (void)input;
}
inline void Cycle::CapturePrimitiveHold(const Input& input) noexcept
{
    // The current leg remains persistent storage throughout this hold. Its
    // exact stopped scalar cursor and analytic arc angles are never replaced
    // with an excursion to the block's start or a straight return chord.
    if(!feedback_.flushCheckpointSaved){
        savedLeg_=leg_;savedPhase_=feedback_.nominalPhase;
        savedAnchor_=savedCenter_=output_.commandMCS;
    }
    StartSettle(Phase::HoldSettle,SettleAction::Held,input);
}
inline void Cycle::BeginPrimitiveHold(const Input& input) noexcept
{
    feedback_.holdSequence=input.holdSequence;
    if(primitivePaused_){
        if(feedback_.phase==Phase::HeldAtStart)heldStartFloor_=input.startSequence;
        else if(feedback_.phase==Phase::HoldSettle&&settleAction_==SettleAction::RestorePrimitivePause)
            StartSettle(Phase::HoldSettle,SettleAction::Held,input);
        return;
    }
    primitivePaused_=true;primitivePausedPhase_=feedback_.phase;primitivePausedAction_=settleAction_;
    feedback_.holdActive=true;
    if(leg_.speed!=0.0)(void)StartBrake(feedback_.flushActive?Phase::FlushHoldBrake:Phase::HoldBrake);
    else CapturePrimitiveHold(input);
}
inline void Cycle::SaveHoldAnchor(const Input& input) noexcept
{
    if(primitivePaused_){CapturePrimitiveHold(input);return;}
    savedLeg_=leg_;savedPhase_=feedback_.gapHistoryActive?Phase::GapHistory:feedback_.nominalPhase;
    savedAnchor_=output_.commandMCS;
    savedCenter_=feedback_.gapHistoryActive?historyNominalLeg_.center:
        (savedPhase_==Phase::CircleEntry || savedPhase_==Phase::Orbit || savedPhase_==Phase::ReturnCenter)
        ?leg_.center:savedAnchor_;
    feedback_.holdActive=true;
    StartSettle(Phase::HoldSettle,SettleAction::HoldCenter,input);
}
inline void Cycle::BeginHold(const Input& input) noexcept
{
    if(request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){BeginPrimitiveHold(input);return;}
    feedback_.holdSequence=input.holdSequence;feedback_.holdActive=true;
    if(leg_.speed==0.0)SaveHoldAnchor(input);
    else (void)StartBrake(Phase::HoldBrake);
}
inline double Cycle::NextFlushHeight(bool& deep,std::uint64_t& ordinal) const noexcept
{
    ordinal=gap_.completedFlushOffset+static_cast<std::uint64_t>(feedback_.flushCount)+1ULL;
    deep=flush_.deepFlushEnabled&&ordinal%flush_.deepFlushCycleInterval==0ULL;
    return deep?flush_.deepFlushHeightMm:flush_.heightMm;
}
inline void Cycle::BeginFlush(const Input& input) noexcept
{
    // Freeze the successful-completion ordinal and exact chosen height before
    // braking. HOLD/abandon retains this evidence but never advances count.
    feedback_.flushSelectedHeightMm=NextFlushHeight(feedback_.flushDeepActive,feedback_.flushOrdinal);
    flushHighWater_=input.flushRequestSequence;
    feedback_.flushRequestSequence=flushHighWater_;
    feedback_.flushOrigin=static_cast<std::uint8_t>(FlushOrigin::Periodic);
    feedback_.flushActive=true;feedback_.flushCheckpointSaved=false;
    if(leg_.speed==0.0)StartSettle(Phase::FlushSettle,SettleAction::CaptureFlush,input);
    else (void)StartBrake(Phase::FlushBrake);
}
inline bool Cycle::FlushDistanceAvailable() const noexcept
{
    if(request_.retreatMode!=2U)return true;
    if(leg_.speed<0.0||!(leg_.length>0.0))return false;
    double station=0.0;
    const auto stage=static_cast<NCG76SphereRetreat::Stage>(static_cast<unsigned>(feedback_.nominalPhase)-
        static_cast<unsigned>(Phase::InitialDepth));
    bool deep=false;std::uint64_t ordinal=0ULL;
    const double height=NextFlushHeight(deep,ordinal);
    return NCG76SphereRetreat::Station(request_.geometry,stage,feedback_.layerIndex,
        leg_.position/leg_.length,station)&&station>=height;
}
inline bool Cycle::StartFlushLine(const Point8& end,Phase phase) noexcept
{
    if(feedback_.flushLegSequence==(std::numeric_limits<std::uint64_t>::max)())
    {Fail(Fault::Numeric);return false;}
    if(!StartLine(end,phase,FlushFeed(phase))||!ConfigureFlushProfile(phase))return false;
    if(leg_.length>0.0)++feedback_.flushLegSequence;
    return true;
}
inline bool Cycle::StartFlushPathLeg(bool reverse) noexcept
{
    using namespace NCG76SphereRetreat;
    const double start=feedback_.flushPathStationMm;
    Span span{};
    if(!Locate(request_.geometry,start,reverse,span)) {Fail(Fault::Numeric);return false;}
    const double finish=reverse?(std::max)(flushPathTarget_,span.start):
        (std::min)(feedback_.flushAnchorStationMm,span.end);
    const double from=Fraction(span,start),to=Fraction(span,finish);
    Point8 end{},center{};
    if(!(reverse?finish<start:finish>start)||!Evaluate(request_.geometry,span,to,end,center))
    {Fail(Fault::Numeric);return false;}
    if(!reverse&&finish==feedback_.flushAnchorStationMm)end=savedAnchor_;
    flushPathLegStart_=start;flushPathLegEnd_=finish;
    const auto phase=reverse?Phase::FlushPathRetreat:Phase::FlushPathReturn;
    const bool primitiveArc=request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc;
    if(span.stage!=Stage::Orbit&&!primitiveArc)return StartFlushLine(end,phase);
    NCG76SphereRuntimeGeometry::Layer layer{};
    if((!primitiveArc&&!NCG76SphereRuntimeGeometry::LayerAt(request_.geometry,span.layer,layer))||!InsideTravel(end))
    {Fail(Fault::Numeric);return false;}
    const double arcAngle=primitiveArc?request_.geometry.arcSweepRad:NCG76SphereRuntimeGeometry::TwoPi;
    const double arcRadius=primitiveArc?request_.geometry.radiusMm:layer.radiusMm;
    Leg next{};next.circle=true;next.circleLayer=span.layer;next.start=output_.commandMCS;next.end=end;
    next.center=center;next.circleStartAngle=from*arcAngle;
    next.circleEndAngle=to*arcAngle;
    next.length=std::fabs(to-from)*span.length;
    next.acceleration=next.deceleration=0.5*(std::min)(request_.accelerationMmS2,request_.decelerationMmS2);
    const double radialSpeed=std::sqrt(next.acceleration)*std::sqrt(arcRadius);
    next.speedLimit=(std::min)((std::min)(FlushFeed(phase)/60.0,request_.velocityCapMmS),radialSpeed);
    if(!Detail::Positive(next.length)||!Detail::Positive(next.speedLimit)||!StartProfile(next)||
       feedback_.flushLegSequence==(std::numeric_limits<std::uint64_t>::max)())
    {Fail(Fault::Numeric);return false;}
    leg_=next;feedback_.phase=phase;if(!ConfigureFlushProfile(phase))return false;++feedback_.flushLegSequence;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();return true;
}
inline void Cycle::CaptureFlush(const Input& input) noexcept
{
    // Stop evidence, never the earlier observer candidate, seals this anchor.
    savedLeg_=leg_;savedPhase_=feedback_.nominalPhase;savedAnchor_=output_.commandMCS;
    savedCenter_=(savedPhase_==Phase::CircleEntry||savedPhase_==Phase::Orbit||savedPhase_==Phase::ReturnCenter)
        ?leg_.center:savedAnchor_;
    feedback_.flushCheckpointSaved=true;flushRouteIndex_=0U;
    if(request_.retreatMode==2U){
        const auto stage=static_cast<NCG76SphereRetreat::Stage>(static_cast<unsigned>(savedPhase_)-
            static_cast<unsigned>(Phase::InitialDepth));
        double station=0.0;
        if(!Detail::ResolveStoppedFlushStation(request_.geometry,stage,feedback_.layerIndex,
            savedLeg_.position,savedLeg_.length,savedAnchor_,feedback_.flushSelectedHeightMm,station)||
            station<feedback_.flushSelectedHeightMm)
        {Fail(Fault::Numeric);return;}
        feedback_.flushAnchorStationMm=feedback_.flushPathStationMm=station;
        flushPathTarget_=station-feedback_.flushSelectedHeightMm;
        if(!std::isfinite(flushPathTarget_)||flushPathTarget_<0.0||flushPathTarget_>=station||
           std::fabs((station-flushPathTarget_)-feedback_.flushSelectedHeightMm)>
               (std::min)(request_.geometry.arithmeticErrorBoundMm,feedback_.flushSelectedHeightMm/64.0))
        {Fail(Fault::Numeric);return;}
        (void)StartFlushPathLeg(true);return;
    }
    flushTop_=request_.retreatMode<=1U?savedAnchor_:savedCenter_;
    Point3 liftDirection=request_.geometry.axis;
    if(request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc&&request_.retreatMode==0U){
        Point8 nominal{};Point3 curvature{};
        if(!NCG76SphereRuntimeGeometry::NominalAt(request_.geometry,savedLeg_.position/savedLeg_.length,
            nominal,liftDirection,curvature)){Fail(Fault::Numeric);return;}
    }
    Point3 actualLift{},error{};
    for(unsigned a=0U;a<3U;++a){
        const double lift=request_.retreatMode==1U?(a==2U?feedback_.flushSelectedHeightMm:0.0):
            -liftDirection[a]*feedback_.flushSelectedHeightMm;
        const double baseline=flushTop_[a];flushTop_[a]=baseline+lift;
        actualLift[a]=flushTop_[a]-baseline;error[a]=actualLift[a]-lift;
    }
    const double budget=(std::min)(request_.geometry.arithmeticErrorBoundMm,feedback_.flushSelectedHeightMm/64.0);
    if(!InsideTravel(flushTop_)||!Detail::Finite(error)||!Detail::Positive(Detail::Norm(actualLift))||Detail::Norm(error)>budget)
    {Fail(Fault::TravelLimit);return;}
    if(request_.retreatMode<=1U){(void)StartFlushLine(flushTop_,Phase::FlushLift);return;}
    flushWaypoint_=savedCenter_;
    if(request_.retreatMode==4U){
        Point3 radial{};for(unsigned a=0U;a<3U;++a)radial[a]=savedAnchor_[a]-savedCenter_[a];
        double radius=Detail::Norm(radial);
        if(!std::isfinite(radius)||radius>feedback_.flushSelectedHeightMm+budget){Fail(Fault::Numeric);return;}
        // H==r has one apex. Coalesce only within the admitted fixed physical
        // arithmetic budget; a floating norm must not invent an extra tiny leg.
        if(std::fabs(feedback_.flushSelectedHeightMm-radius)<=budget)radius=feedback_.flushSelectedHeightMm;
        for(unsigned a=0U;a<3U;++a)flushWaypoint_[a]-=request_.geometry.axis[a]*radius;
    }
    if(!InsideTravel(flushWaypoint_)){Fail(Fault::TravelLimit);return;}
    // Coalesce exact zero legs; every nonzero corner still earns a fresh dwell.
    if(flushWaypoint_!=savedAnchor_){(void)StartFlushLine(flushWaypoint_,Phase::FlushToCenter);return;}
    flushRouteIndex_=1U;
    if(flushTop_!=flushWaypoint_){(void)StartFlushLine(flushTop_,Phase::FlushLift);return;}
    // A positive admitted E6 prevents the entire center route being stationary.
    Fail(Fault::Numeric);(void)input;
}
inline void Cycle::NextFlushLeg(const Input& input) noexcept
{
    if(request_.retreatMode==2U){
        feedback_.flushPathStationMm=flushPathLegEnd_;
        if(flushRouteIndex_==0U){
            if(feedback_.flushPathStationMm==flushPathTarget_)flushRouteIndex_=1U;
            (void)StartFlushPathLeg(flushRouteIndex_==0U);return;
        }
        if(feedback_.flushPathStationMm!=feedback_.flushAnchorStationMm){(void)StartFlushPathLeg(false);return;}
        settleAction_=SettleAction::RestoreAfterFlush;ContinueAfterSettle(input);return;
    }
    if(request_.retreatMode<=1U){
        if(flushRouteIndex_==0U){flushRouteIndex_=1U;(void)StartFlushLine(savedAnchor_,Phase::FlushReturn);return;}
        settleAction_=SettleAction::RestoreAfterFlush;ContinueAfterSettle(input);return;
    }
    // Route 0 anchor->waypoint, 1 waypoint->top, 2 top->waypoint,
    // 3 waypoint->anchor. Exact coincident points require no invented motion.
    while(++flushRouteIndex_<4U){
        const Point8& endpoint=flushRouteIndex_==1U?flushTop_:flushRouteIndex_==2U?flushWaypoint_:savedAnchor_;
        if(endpoint==output_.commandMCS)continue;
        const auto phase=flushRouteIndex_==1U?Phase::FlushLift:
            flushRouteIndex_==2U?Phase::FlushReturn:Phase::FlushFromCenter;
        (void)StartFlushLine(endpoint,phase);return;
    }
    settleAction_=SettleAction::RestoreAfterFlush;ContinueAfterSettle(input);
}
inline void Cycle::HoldDuringFlush(const Input& input) noexcept
{
    if(request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){BeginPrimitiveHold(input);return;}
    const bool checkpoint=feedback_.flushCheckpointSaved;
    const auto phase=feedback_.phase;
    feedback_.flushAbandonedSequence=feedback_.flushRequestSequence;
    feedback_.flushActive=false;feedback_.holdSequence=input.holdSequence;feedback_.holdActive=true;
    if(!checkpoint)
    {
        // Still on the nominal path. Continue the already established brake
        // to its real stop, then capture the same checkpoint ordinary HOLD uses.
        if(phase==Phase::FlushBrake)
        {feedback_.phase=Phase::HoldBrake;RefreshOutput();}
        else SaveHoldAnchor(input);
    }
    else
    {
        // The current line is only the flush excursion. Never replace the
        // saved nominal Leg/anchor with this line when cancelling the flush.
        if(leg_.speed!=0.0)(void)StartBrake(Phase::FlushHoldBrake);
        else StartSettle(Phase::HoldSettle,SettleAction::HoldCenter,input);
    }
}
inline bool Cycle::CanEnterGapHistory() const noexcept
{
    if(!gap_.enabled||!gap_.crossLegRetreatEnabled||feedback_.gapHistoryActive||
       !previousLegValid_||leg_.speed!=0.0||leg_.position!=0.0||
       previousLeg_.speed!=0.0||previousLeg_.position!=previousLeg_.length||
       previousLeg_.end!=leg_.start||output_.commandMCS!=leg_.start)return false;
    if(feedback_.previousParentHistoryReady)
        return request_.retreatMode==2U&&feedback_.nominalPhase==Phase::InitialDepth&&
            request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere;
    if(previousLeg_.circle)return false;
    if(feedback_.nominalPhase==Phase::Orbit)
        return previousPhase_==Phase::CircleEntry&&previousLayer_==feedback_.layerIndex;
    if(feedback_.nominalPhase==Phase::CircleEntry)
        return feedback_.layerIndex==0U?
            previousPhase_==Phase::InitialRetreat&&previousLayer_==0U:
            previousPhase_==Phase::AdvanceCenter&&previousLayer_+1U==feedback_.layerIndex;
    return false;
}
inline bool Cycle::SeedPreviousPrimitive(const NCG76SphereRuntimeGeometry::Geometry& geometry) noexcept
{
    using K=NCG76SphereRuntimeGeometry::Kind;
    if(!gap_.enabled||!gap_.crossLegRetreatEnabled||request_.retreatMode!=2U||
       (request_.geometry.kind!=K::Line&&request_.geometry.kind!=K::Arc)||
       !Detail::Positive(request_.geometry.travelMm)||
       (geometry.kind!=K::Line&&geometry.kind!=K::Arc)||!Detail::Positive(geometry.travelMm)||
       !NCG76SphereRuntimeGeometry::Valid(geometry)||
       !SameNCTranslationSnapshot(geometry.frame.translation,request_.geometry.frame.translation)||
       std::memcmp(geometry.endMCS.data(),request_.geometry.startMCS.data(),sizeof(Point8))!=0)return false;
    for(unsigned a=0U;a<8U;++a){
        if(!std::isfinite(geometry.boundsMinMCS[a])||!std::isfinite(geometry.boundsMaxMCS[a])||
           geometry.boundsMinMCS[a]>geometry.boundsMaxMCS[a]||
           ((request_.travelMask&(1U<<a))&&
               (geometry.boundsMinMCS[a]<request_.travelMin[a]||geometry.boundsMaxMCS[a]>request_.travelMax[a])))return false;
        if(a>=3U&&std::memcmp(&geometry.startMCS[a],&request_.geometry.startMCS[a],sizeof(double))!=0)return false;
    }
    previousPrimitive_=geometry;previousLeg_=Leg{};
    auto& prior=previousLeg_;prior.start=geometry.startMCS;prior.end=geometry.endMCS;
    prior.center=geometry.startMCS;prior.length=prior.position=geometry.travelMm;
    const double nominalFeed=request_.parameterFeeds&&request_.nominalFeedCapMmMin>0.0?
        (std::min)(request_.feedMmMin,request_.nominalFeedCapMmMin):request_.feedMmMin;
    prior.speedLimit=(std::min)((std::min)(nominalFeed,geometry.feedMmMin)/60.0,request_.velocityCapMmS);
    if(geometry.kind==K::Arc){
        prior.circle=true;prior.circleEndAngle=geometry.arcSweepRad;
        for(unsigned a=0U;a<3U;++a)prior.center[a]=geometry.arcCenterMCS[a];
        const double normalBudget=0.5*(std::min)(request_.accelerationMmS2,request_.decelerationMmS2);
        prior.acceleration=prior.deceleration=normalBudget;
        prior.speedLimit=(std::min)(prior.speedLimit,std::sqrt(normalBudget)*std::sqrt(geometry.radiusMm));
    }else{
        prior.direction=geometry.axis;
        prior.acceleration=request_.accelerationMmS2;prior.deceleration=request_.decelerationMmS2;
    }
    if(!Detail::Positive(prior.speedLimit)||!Detail::Positive(prior.acceleration)||!Detail::Positive(prior.deceleration))return false;
    previousPhase_=Phase::InitialDepth;previousLayer_=0U;previousLegValid_=true;
    feedback_.previousParentHistoryReady=true;return true;
}
inline void Cycle::BeginGapHistory(const Input& input) noexcept
{
    if(!CanEnterGapHistory())return;
    historyNominalLeg_=leg_;feedback_.gapHistoryActive=true;
    StartSettle(Phase::GapHistorySettle,SettleAction::EnterGapHistory,input);
}
inline void Cycle::ContinueAfterSettle(const Input& input) noexcept
{
    switch(settleAction_)
    {
    case SettleAction::BeginNominal:
        feedback_.phase=feedback_.nominalPhase;RefreshOutput();break;
    case SettleAction::RecoveryContinue:ContinueRecovery(input);break;
    case SettleAction::NextNominal:NextNominal(input);break;
    case SettleAction::FinishCheck:CheckFinishEndpoint(input);break;
    case SettleAction::HoldCenter:(void)StartLine(savedCenter_,Phase::HoldToCenter,request_.returnFeedMmMin);break;
    case SettleAction::HoldStart:(void)StartLine(request_.geometry.startMCS,Phase::HoldToStart,request_.returnFeedMmMin);break;
    case SettleAction::Held:
        feedback_.phase=Phase::HeldAtStart;heldStartFloor_=input.startSequence;RefreshOutput();break;
    case SettleAction::ResumeCenter:(void)StartLine(savedCenter_,Phase::ResumeToCenter,request_.returnFeedMmMin);break;
    case SettleAction::ResumeAnchor:(void)StartLine(savedAnchor_,Phase::ResumeToAnchor,request_.returnFeedMmMin);break;
    case SettleAction::RestorePrimitivePause:
        if(!primitivePaused_||request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere)
        {Fail(Fault::Numeric);break;}
        feedback_.holdActive=false;primitivePaused_=false;
        if(Detail::IsSettling(primitivePausedPhase_)){
            StartSettle(primitivePausedPhase_,primitivePausedAction_,input);break;
        }
        if(primitivePausedPhase_==Phase::FlushBrake){
            StartSettle(Phase::FlushSettle,SettleAction::CaptureFlush,input);break;
        }
        if(!StartProfile(leg_)){Fail(Fault::Numeric);break;}
        feedback_.phase=primitivePausedPhase_;
        if(Detail::IsFlushMotionPhase(primitivePausedPhase_)&&!ConfigureFlushProfile(primitivePausedPhase_))break;
        RefreshOutput();break;
    case SettleAction::RestoreNominal:
        leg_=savedLeg_;leg_.speed=0.0;leg_.accelerationNow=0.0;
        if(!StartProfile(leg_)) {Fail(Fault::Numeric);break;}
        output_.commandMCS=savedAnchor_;
        if(savedPhase_!=Phase::GapHistory)feedback_.nominalPhase=savedPhase_;
        feedback_.holdActive=false;
        feedback_.flushCheckpointSaved=false;
        feedback_.phase=savedPhase_;RefreshOutput();break;
    case SettleAction::Complete:feedback_.phase=Phase::Complete;RefreshOutput();break;
    case SettleAction::Fault:Fail(Fault::RepeatedHold,false);break;
    case SettleAction::CaptureFlush:CaptureFlush(input);break;
    case SettleAction::FlushReturn:(void)StartLine(savedAnchor_,Phase::FlushReturn,flush_.feedMmMin);break;
    case SettleAction::NextFlushLeg:NextFlushLeg(input);break;
    case SettleAction::RestoreAfterFlush:
        if((finish_.forcedFlush&&feedback_.finishForcedCompletedCount==(std::numeric_limits<std::uint64_t>::max)())||
           (!finish_.forcedFlush&&feedback_.flushCount==(std::numeric_limits<unsigned>::max)())) {Fail(Fault::Numeric);break;}
        leg_=savedLeg_;leg_.speed=0.0;leg_.accelerationNow=0.0;
        if(!StartProfile(leg_)) {Fail(Fault::Numeric);break;}
        output_.commandMCS=savedAnchor_;feedback_.nominalPhase=savedPhase_;feedback_.phase=savedPhase_;
        feedback_.flushActive=false;feedback_.flushCheckpointSaved=false;
        feedback_.flushOrigin=static_cast<std::uint8_t>(FlushOrigin::None);
        if(finish_.forcedFlush){
            finish_.forcedFlush=false;finish_.returnedPending=true;++feedback_.finishForcedCompletedCount;
            feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::Recheck);
            StartSettle(Phase::Settle,SettleAction::FinishCheck,input);break;
        }
        feedback_.flushCompletedSequence=feedback_.flushRequestSequence;++feedback_.flushCount;
        RefreshOutput();break;
    case SettleAction::EnterGapHistory:
        if(!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin)) {Fail(Fault::SourceLost);break;}
        if(input.signedGapSpeedMmMin>=0.0)
        {
            feedback_.gapHistoryActive=false;feedback_.phase=feedback_.nominalPhase;RefreshOutput();break;
        }
        if(feedback_.historyBoundarySequence==(std::numeric_limits<std::uint64_t>::max)()) {Fail(Fault::Numeric);break;}
        ++feedback_.historyBoundarySequence;leg_=previousLeg_;
        previousPrimitiveLegActive_=feedback_.previousParentHistoryReady;
        leg_.speed=leg_.accelerationNow=0.0;feedback_.phase=Phase::GapHistory;
        if(!EvaluateLeg()) {Fail(Fault::Numeric);break;}
        RefreshOutput();break;
    case SettleAction::RestoreGapHistory:
        if(!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin)) {Fail(Fault::SourceLost);break;}
        if(input.signedGapSpeedMmMin<=0.0)
        {feedback_.phase=Phase::GapHistory;RefreshOutput();break;}
        if(feedback_.historyReturnSequence==(std::numeric_limits<std::uint64_t>::max)()) {Fail(Fault::Numeric);break;}
        ++feedback_.historyReturnSequence;leg_=historyNominalLeg_;
        previousPrimitiveLegActive_=false;
        leg_.speed=leg_.accelerationNow=0.0;output_.commandMCS=leg_.start;
        feedback_.gapHistoryActive=false;feedback_.phase=feedback_.nominalPhase;RefreshOutput();break;
    }
}
inline bool Cycle::Begin(const Request& request,const Input& input,const FlushConfig* flush,const GapConfig* gap) noexcept
{
    if(output_.active)return false;
    const Request frozen=request;
    const FlushConfig frozenFlush=flush?*flush:FlushConfig{};
    const GapConfig frozenGap=gap?*gap:GapConfig{};
    // A caller must supply its retained external geometry, never a reference
    // into this object that Reset is about to reconstruct in place.
    if(frozenGap.previousPrimitive&&NCG76SphereRuntimeGeometry::Detail::Overlap(
        this,sizeof(*this),frozenGap.previousPrimitive,sizeof(*frozenGap.previousPrimitive)))return false;
    Reset();initialized_=true;request_=frozen;flush_=frozenFlush;gap_=frozenGap;
    gap_.previousPrimitive=nullptr;
    feedback_.session=frozen.session;feedback_.tick=input.tick;feedback_.gapMotionEnabled=frozenGap.enabled;
    feedback_.retreatMode=frozen.retreatMode;
    if(Detail::Finite(frozen.geometry.startMCS))output_.commandMCS=frozen.geometry.startMCS;
    if((frozenGap.crossLegRetreatEnabled&&!frozenGap.enabled)||
       (frozenGap.completedFlushOffset!=0ULL&&(!frozenGap.enabled||
            frozen.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere))||
       frozenGap.completedFlushOffset>(std::numeric_limits<std::uint64_t>::max)()-
            static_cast<std::uint64_t>((std::numeric_limits<unsigned>::max)())-1ULL||
       !ValidateFlushConfig(frozen,frozenFlush) ||
       !input.tick || !input.nowUs || !input.physicalSequence ||
       !Detail::Finite(input.actualMCS)) {Fail(Fault::InvalidRequest,false);return false;}
    if(frozenGap.previousPrimitive&&!SeedPreviousPrimitive(*frozenGap.previousPrimitive))
        {Fail(Fault::InvalidRequest,false);return false;}
    if(input.cancel){Fail(Fault::Cancelled,false);return false;}
    if(!input.sourceValid){Fail(Fault::SourceLost);return false;}
    if(!input.pdoValid){Fail(Fault::PdoLost);return false;}
    if(!input.powerInhibited){Fail(Fault::PowerNotInhibited);return false;}
    if(!InsideTravel(input.actualMCS) || !AtCommand(input.actualMCS)) {Fail(Fault::TravelLimit,false);return false;}
    lastTick_=input.tick;lastNowUs_=input.nowUs;lastPhysical_=input.physicalSequence;
    lastHold_=input.holdSequence;lastStart_=input.startSequence;lastHoldLevel_=input.hold;
    if(!StartNominal(Phase::InitialDepth))return false;
    if(frozen.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&frozen.geometry.travelMm==0.0){
        StartSettle(Phase::Settle,SettleAction::Complete,input);
        if(input.hold)BeginHold(input);
    }else if(input.hold)BeginHold(input);
    else StartSettle(Phase::Settle,SettleAction::BeginNominal,input);
    return !output_.faulted;
}
inline Output Cycle::Step(const Input& input) noexcept
{
    if(!initialized_ || !output_.active)return output_;
    feedback_.tick=input.tick;feedback_.gapApplied=false;
    if(input.cancel){Fail(Fault::Cancelled,false);return output_;}
    if(!input.sourceValid){Fail(Fault::SourceLost);return output_;}
    if(!input.pdoValid){Fail(Fault::PdoLost);return output_;}
    if(!input.powerInhibited){Fail(Fault::PowerNotInhibited);return output_;}
    const auto clockReasons=Detail::ClassifyClockFault(lastTick_,lastNowUs_,input,
        request_.minimumTickUs,request_.maximumTickUs);
    if(clockReasons)
    {
        if(!feedback_.clockFault.valid) {
            auto& evidence=feedback_.clockFault;
            evidence.previousTick=lastTick_;evidence.currentTick=input.tick;
            evidence.previousNowUs=lastNowUs_;evidence.currentNowUs=input.nowUs;
            evidence.previousPhysicalSequence=lastPhysical_;
            evidence.currentPhysicalSequence=input.physicalSequence;
            evidence.minimumTickUs=request_.minimumTickUs;
            evidence.maximumTickUs=request_.maximumTickUs;
            evidence.reasonBits=clockReasons;
            evidence.deltaValid=input.nowUs>lastNowUs_;
            evidence.deltaUs=evidence.deltaValid?input.nowUs-lastNowUs_:0ULL;
            evidence.valid=true;
        }
        Fail(Fault::Clock);return output_;
    }
    if(input.physicalSequence<=lastPhysical_){Fail(Fault::PhysicalSequence);return output_;}
    if(input.holdSequence<lastHold_ || input.startSequence<lastStart_){Fail(Fault::IntentSequence);return output_;}
    if(!InsideTravel(input.actualMCS)){Fail(Fault::TravelLimit);return output_;}
    const double dt=static_cast<double>(input.nowUs-lastNowUs_)*1e-6;
    const bool holdEdge=(input.hold && !lastHoldLevel_) || input.holdSequence>lastHold_;
    lastTick_=input.tick;lastNowUs_=input.nowUs;lastPhysical_=input.physicalSequence;
    lastHold_=input.holdSequence;lastStart_=input.startSequence;lastHoldLevel_=input.hold;
    elapsed_+=dt;
    if(!std::isfinite(elapsed_) || elapsed_>request_.maximumCycleSeconds){Fail(Fault::CycleTimeout);return output_;}

    // Opt-in recovery is an excursion owned by this same unfinished parent.
    // Nominal/flush state is captured once; no old parent is re-opened here.
    if(recovery_.enabled){
        if(holdEdge&&!recovery_.active&&!feedback_.holdActive){
            if(!BeginRecoveryHold(input))return output_;
        }
        if(recovery_.active){StepRecovery(input,dt,holdEdge);return output_;}
    }

    if(holdEdge)
    {
        feedback_.holdSequence=input.holdSequence;
        if(primitivePaused_)BeginPrimitiveHold(input);
        // Entry has not switched geometry yet. Abandon that request and hold
        // the current forward floor; a later negative input may request again.
        if(feedback_.phase==Phase::GapHistorySettle&&settleAction_==SettleAction::EnterGapHistory)
        {feedback_.gapHistoryActive=false;feedback_.phase=feedback_.nominalPhase;}
        if(feedback_.flushActive)HoldDuringFlush(input);
        else if(!feedback_.holdActive && (Detail::IsNominal(feedback_.phase) || feedback_.phase==Phase::Settle ||
            feedback_.phase==Phase::GapHistory || feedback_.phase==Phase::GapHistorySettle))
            BeginHold(input);
        else if(feedback_.phase==Phase::HeldAtStart)
            heldStartFloor_=input.startSequence;
        else if(feedback_.holdActive &&
            (feedback_.phase==Phase::ResumeToCenter || feedback_.phase==Phase::ResumeToAnchor ||
             (feedback_.phase==Phase::HoldSettle &&
              (settleAction_==SettleAction::ResumeCenter || settleAction_==SettleAction::ResumeAnchor ||
               settleAction_==SettleAction::RestoreNominal))))
        {
            // Cancel only the resume excursion. savedLeg_/savedAnchor_ remain
            // the original machining checkpoint; SaveHoldAnchor would replace
            // it with this return leg and lose nominal progress.
            if(leg_.speed!=0.0)(void)StartBrake(Phase::ResumeBrake);
            else StartSettle(Phase::HoldSettle,SettleAction::HoldCenter,input);
        }
        // HOLD while already braking/returning to the start is idempotent.
        // The return remains the requested HOLD behavior. On arrival Held
        // captures the latest START floor, so an earlier START cannot resume.
    }
    if(output_.faulted)return output_;
    // The observer's due decision is from the preceding coherent callback.
    // That callback may have finished the last eligible nominal leg already.
    // Consume its pending id at the leg's ordinary endpoint settle too, before
    // advancing the nominal state; a terminal parent can never reopen here.
    const bool flushBoundary=feedback_.phase==Phase::Settle && settleAction_==SettleAction::NextNominal &&
        Detail::IsFlushWorkPhase(feedback_.nominalPhase);
    if(flush_.enabled && input.flushRequestSequence>flushHighWater_ && !feedback_.flushActive &&
       !feedback_.holdActive && !feedback_.gapHistoryActive && !feedback_.finishActive && !input.hold && (Detail::IsFlushWorkPhase(feedback_.phase) || flushBoundary) && FlushDistanceAvailable())
        BeginFlush(input);
    if(output_.faulted)return output_;
    const Phase phase=feedback_.phase;
    if(Detail::IsSettling(phase))
    {
        if(Settled(input,dt))ContinueAfterSettle(input);
    }
    else if(phase==Phase::HeldAtStart)
    {
        // EDM297: arrival tolerance is a settle condition, not a software
        // travel boundary. A held encoder can drift just outside that window
        // while final PDO is zero. Keep the exact original-start reference
        // and let the adapter release its zero clamp so the unchanged PID can
        // reacquire it. This also withdraws held readiness until a completely
        // new physical/zero-send dwell succeeds; the saved nominal checkpoint
        // is never replaced by this stationary correction.
        if(!AtCommand(input.actualMCS))
        {
            if(feedback_.heldRequalificationSequence==(std::numeric_limits<std::uint64_t>::max)())
            {Fail(Fault::Numeric);return output_;}
            ++feedback_.heldRequalificationSequence;
            StartSettle(Phase::HoldSettle,SettleAction::Held,input);
        }
        else if(!input.hold && input.startSequence>heldStartFloor_)
        {
            feedback_.resumeSequence=input.startSequence;
            StartSettle(Phase::HoldSettle,primitivePaused_?SettleAction::RestorePrimitivePause:SettleAction::ResumeCenter,input);
        }
    }
    else if(Detail::IsNominal(phase) || Detail::IsExcursion(phase) || phase==Phase::GapHistory || phase==Phase::HoldBrake ||
            phase==Phase::FaultBrake || phase==Phase::ResumeBrake || phase==Phase::FlushBrake || phase==Phase::FlushHoldBrake)
    {
        bool finished=false;
        const bool gapStep=gap_.enabled&&(Detail::IsFlushWorkPhase(phase)||phase==Phase::GapHistory)&&
            !feedback_.holdActive&&!feedback_.flushActive;
        if(gapStep&&!input.gapValid){Fail(Fault::SourceLost);return output_;}
        const double previousPosition=leg_.position;
        const bool advanced=gapStep?AdvanceGap(dt,input.signedGapSpeedMmMin,finished):
            AdvanceLeg(dt,phase==Phase::HoldBrake || phase==Phase::FaultBrake || phase==Phase::ResumeBrake ||
                phase==Phase::FlushBrake || phase==Phase::FlushHoldBrake,finished);
        if(!advanced){Fail(Fault::Numeric);return output_;}
        if(gapStep&&phase==Phase::InitialDepth&&!ObserveFinishProgress(previousPosition))return output_;
        if(gapStep&&!finished&&phase!=Phase::GapHistory&&input.signedGapSpeedMmMin<0.0&&CanEnterGapHistory())
            BeginGapHistory(input);
        if(finished)
        {
            if(phase==Phase::GapHistory)StartSettle(Phase::GapHistorySettle,SettleAction::RestoreGapHistory,input);
            else if(phase==Phase::HoldBrake)SaveHoldAnchor(input);
            else if(phase==Phase::FlushHoldBrake&&primitivePaused_)CapturePrimitiveHold(input);
            else if(phase==Phase::ResumeBrake || phase==Phase::FlushHoldBrake)StartSettle(Phase::HoldSettle,SettleAction::HoldCenter,input);
            else if(phase==Phase::FlushBrake)StartSettle(Phase::FlushSettle,SettleAction::CaptureFlush,input);
            else if(phase==Phase::FlushLift||phase==Phase::FlushReturn||phase==Phase::FlushToCenter||
                phase==Phase::FlushFromCenter||phase==Phase::FlushPathRetreat||phase==Phase::FlushPathReturn)
                StartSettle(Phase::FlushSettle,SettleAction::NextFlushLeg,input);
            else if(phase==Phase::FaultBrake)StartSettle(Phase::FaultSettle,SettleAction::Fault,input);
            else if(phase==Phase::HoldToCenter)StartSettle(Phase::HoldSettle,SettleAction::HoldStart,input);
            else if(phase==Phase::HoldToStart)StartSettle(Phase::HoldSettle,SettleAction::Held,input);
            else if(phase==Phase::ResumeToCenter)StartSettle(Phase::HoldSettle,SettleAction::ResumeAnchor,input);
            else if(phase==Phase::ResumeToAnchor)StartSettle(Phase::HoldSettle,SettleAction::RestoreNominal,input);
            else StartSettle(Phase::Settle,SettleAction::NextNominal,input);
        }
    }
    else Fail(Fault::Numeric);
    RefreshOutput();return output_;
}
} // namespace NCG76SphereCycle

#include "NCG76SphereHoldRecovery.h"
#include "NCG76SphereFinishVerification.h"
