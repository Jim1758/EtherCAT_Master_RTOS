#pragma once
#include "G80Config.h"
#include <array>
#include <cstdint>

namespace G80
{
using Point6 = std::array<double, AxisCount>;

enum class Phase : std::uint32_t
{
    Idle, InitialRelease, Approach, FastSeek, SlowSeek, ContactBrake,
    ContactSettle, Retreat, ReleaseSettle, FaultBrake, Complete, Fault
};
enum class Fault : std::uint32_t
{
    None, Busy, Disabled, InvalidRequest, SourceLost, PdoLost,
    NonMonotonicTick, SourceSequenceRegression, InitialContact,
    CycleTimeout, SearchDistanceExceeded, ReleaseTimeout, SettleTimeout,
    MeanErrorExceeded, ApproachContact, TravelLimit, Cancelled
};

struct Request
{
    std::uint64_t session = 0;
    Config config{};
    std::uint32_t inputSource = 0U; // 0 physical GAP; 1 explicit per-command Q1 simulation.
    Point6 startMCS{};
    // Native ratios: XYZ are mm and CUV are degrees. No mixed-unit norm.
    Point6 direction{};
    Point6 velocityCapsNative{};       // native units / second, all > 0
    // Independent G80 ramp envelopes, optionally reduced by a separately
    // defined machine/hardware acceleration limit. Never derive these from a
    // G00 profile time. Cycle still uses the current leg's speed / G80 time.
    Point6 accelerationCapsNative{};  // native units / second squared, all > 0
    Point6 decelerationCapsNative{};  // native units / second squared, all > 0
    Point6 travelMin{}, travelMax{};
    std::uint32_t travelMask = 0;
    double slowPercent = 100.0;
    bool fastFirst = false, finalRetreat = false;
    bool hasApproach = false;
    Point6 approachEndMCS{};
    // G00 contributes only this leading leg's requested speed. Once paired
    // into the G80 parent, its ramps use the frozen G80 times for the cycle.
    Point6 rapidVelocityNative{};     // native units / second, moving axes > 0
};
struct Input
{
    std::uint64_t tick = 0, sourceSequence = 0, nowUs = 0;
    bool sourceValid = false, pdoValid = false, hold = false, cancel = false;
    Point6 actualMCS{};
    double voltageV = 0.0;
};
struct Output
{
    Point6 commandMCS{}, velocityNative{};
    bool active = false, complete = false, stopped = true;
    bool faulted = false, emergencyStopRequired = false;
};
struct Feedback
{
    Phase phase = Phase::Idle;
    Fault fault = Fault::None;
    std::uint64_t session = 0, tick = 0;
    std::uint32_t slowCount = 0, targetCount = 0;
    bool contact = false, held = false, fastCaptured = false;
    bool statisticsValid = false;
    // EDM278 diagnostic only: exact command velocities at the first normal
    // approach-to-seek boundary, before consuming any remaining tick time.
    // XYZ use mm/s; C/U/V use deg/s. Never infer actual servo velocity here.
    // HOLD overshoot/resume catch-up and rejected transitions have no boundary.
    bool approachJunctionValid = false;
    std::uint64_t approachJunctionTick = 0;
    Point6 approachExitVelocityNative{}, seekEntryVelocityNative{};
    Point6 meanMCS{}, lastContactMCS{}, fastContactMCS{};
    Point6 meanCommandedMCS{}, lastCommandedContactMCS{};
    Point6 meanAbsDeviation{}, maxDeviation{}, spread{};
    double linearMeanError = 0.0, rotaryMeanError = 0.0;
    double linearMaxDeviation = 0.0, rotaryMaxDeviation = 0.0;
};

// Single-session, fixed-storage, no allocation, logging, IO, or wall clock calls.
// Call exactly once per SampleSeconds. Runtime must independently validate the
// physical GAP source age and apply emergencyStopRequired immediately.
class Cycle
{
public:
    bool Begin(const Request& request, const Input& input) noexcept;
    Output Step(const Input& input) noexcept;
    const Feedback& Current() const noexcept { return feedback_; }
    const Output& Command() const noexcept { return output_; }
    void Reset() noexcept;
private:
    bool Validate(const Request& request, const Input& input) const noexcept;
    bool InsideTravel(const Point6& point) const noexcept;
    bool InsideSearch(const Point6& point) const noexcept;
    double SeekSpeed(bool fast) const noexcept;
    void StartSeek(double initialSpeed = 0.0) noexcept;
    void StartApproach() noexcept;
    double ForwardRoom(const Point6& point) const noexcept;
    void PrepareApproachBrake() noexcept;
    void StartRetreat() noexcept;
    bool StartLeg(const Point6& delta, double length, double speed,
        double exitSpeed, double initialSpeed) noexcept;
    double AdvanceLeg(double seconds, bool brake, bool& reached) noexcept;
    void SetFault(Fault fault, bool emergency = false) noexcept;
    void UpdateSignal(const Input& input, bool fresh) noexcept;
    void ResetSettle() noexcept;
    bool Settled(const Input& input, bool fresh) noexcept;
    bool ComputeStatistics() noexcept;
    void Finish() noexcept;
    void RefreshOutput() noexcept;

    Request request_{};
    Output output_{};
    Feedback feedback_{};
    std::array<Point6, MaximumSlowTouches> touches_{};
    std::array<Point6, MaximumSlowTouches> commandedTouches_{};
    Point6 searchOrigin_{}, legOrigin_{}, legDirection_{};
    Point6 firstLowPose_{}, firstLowCommand_{}, settleMin_{}, settleMax_{};
    double linearNorm_ = 0.0, rotaryNorm_ = 0.0, searchLimit_ = 0.0;
    double legLength_ = 0.0, legPosition_ = 0.0, speed_ = 0.0;
    double speedLimit_ = 0.0, acceleration_ = 0.0, deceleration_ = 0.0;
    double exitSpeed_ = 0.0, approachScale_ = 0.0, approachLength_ = 0.0;
    double elapsed_ = 0.0, phaseElapsed_ = 0.0, settleElapsed_ = 0.0;
    std::uint64_t lastTick_ = 0, lastSource_ = 0, lastNowUs_ = 0;
    std::uint32_t lowCount_ = 0, releaseCount_ = 0;
    bool initialized_ = false, fastSeek_ = false, finalRetreat_ = false;
    bool settleStarted_ = false, touchStored_ = false, seekTravelBound_ = false;
    bool holdBraking_ = false;
};
}
