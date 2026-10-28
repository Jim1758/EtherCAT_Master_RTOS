#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

// TUNE01 configuration only. Loading never changes live motion or PID gains.
namespace PositionAutoTune
{
constexpr std::size_t kAxisCount = 6U; // X, Y, Z, C, U, V (NC A0..A5)
constexpr std::uint32_t kMaxSamples = 131072U;
constexpr double kSamplePeriodSeconds = 0.00025;
constexpr double kTwoPointMaximumDurationSeconds = 3600.0;

enum class MachineProfile : std::uint32_t
{
    Bench = 0U,
    Machine = 1U
};

// Legacy *Mm names are preserved for parameter-file compatibility. Values are
// native position units: mm for LINEAR, degrees for ROTARY; speed is unit/min.
// The installed AxisConfig type/scale is authoritative, not the axis letter.
struct AxisConfig
{
    bool enabled = true;
    // TUNE07: 0 preserves the legacy global policy; 1/2/3 select a bounded
    // stable/standard/fast response preference for this axis only.
    std::uint32_t tuningStrategy = 0U;
    // Native mm/degree goal, 0 = unspecified. Assessment only: this never
    // changes a guard, the test settling window or machine in-position limits.
    double targetPositionMm = 0.0;
    // TUNE09: requested-speed constant-velocity |command-actual| goal,
    // native mm/degree. Zero leaves it unspecified; never a motion guard.
    double cruiseLagTargetMm = 0.0;
    double excursionMm = 1.0;
    double speedMmMin = 60.0;
    double accelerationSeconds = 0.25;
    double decelerationSeconds = 0.25;
    double dwellSeconds = 0.3;
    std::uint32_t cycles = 1U;
    double followingLimitMm = 0.2;
    double settleWindowMm = 0.005;
    double settleHoldSeconds = 0.1;
    double settleTimeoutSeconds = 3.0;
    double kpMin = 1.0;
    double kpMax = 200.0;
    double kvffMin = 0.5;
    double kvffMax = 1.5;
    double maxOvershootMm = 0.02;
    double maxRmsErrorMm = 0.05;
    // Taught machine coordinates, in native mm/deg without rotary wrapping.
    // Q1 uses these exact endpoints as its full stroke; excursionMm and
    // endPadding apply only to the legacy origin-relative envelope.
    bool rangeEnabled = false;
    double rangeMinMCS = 0.0;
    double rangeMaxMCS = 0.0;
    double endPadding = 0.0;
    // TUNE03: IDLE is assessed after each move using a continuous stopped
    // command / IDLE-bank tail. These are native mm or degree units too.
    bool idleEnabled = true;
    double idleKpMin = 1.0;
    double idleKpMax = 200.0;
    double idleKiMin = 0.0;
    double idleKiMax = 100.0;
    double idleIntegralTimeSeconds = 2.0;
    double idleObservationSeconds = 4.0;
    double idleMaxRmsErrorMm = 0.005;
    double idleMaxJitterMm = 0.01;
};

struct Config
{
    bool enabled = false;
    MachineProfile machineProfile = MachineProfile::Bench;
    std::string machineId = "BENCH_A3_01";
    std::array<AxisConfig, kAxisCount> axis{};
    double maxDurationSeconds = 30.0;
    // Q1 starts with this budget (legacy default 120 s). A full-stroke run
    // extends it from predicted time, capped at one hour without moving A/B.
    double twoPointMaxDurationSeconds = 120.0;
    // Legacy Q0 parameter-file capacity; Q1 capture capacity is separate.
    std::uint32_t maxSamples = kMaxSamples;
    double modelFitMinimum = 0.9;
    double minimumVelocityMmS = 0.05;
    double candidateResponseFactor = 0.2;
    double maxKpIncreaseRatio = 1.25;

    Config() { axis[1].settleWindowMm = 0.008; }
};

enum class ConfigLoadStatus : std::uint32_t
{
    Loaded = 0U,
    MissingDisabled = 1U,
    Invalid = 2U
};

// One cycle is origin -> +excursion -> origin -> -excursion -> origin.
// Includes four profiled moves, dwell and minimum settle hold at each end.
// A stalled axis can take longer; RT must enforce timeout and sample capacity.
double PredictAxisDurationSeconds(const AxisConfig& axis) noexcept;
double EffectiveTwoPointDurationSeconds(const Config& config, double predictedSeconds) noexcept;
double EffectiveCandidateResponseFactor(const Config& config, const AxisConfig& axis) noexcept;
double EffectiveMaxKpIncreaseRatio(const Config& config, const AxisConfig& axis) noexcept;
const char* TuningStrategyName(std::uint32_t strategy) noexcept;
bool ValidateConfig(const Config& config, std::string& error) noexcept;

// Non-RT only. UTF-8 BOM is accepted; key/value text must be ASCII.
// Exact, case-sensitive keys; // comments; duplicate/unknown keys are errors.
// Missing or invalid files return a disabled default configuration.
ConfigLoadStatus LoadConfig(const std::string& path, Config& config,
    std::string& error) noexcept;
}
