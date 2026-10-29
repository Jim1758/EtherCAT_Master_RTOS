#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <type_traits>

// File parsing is NC/startup-only. The frozen value contains no pointers,
// strings or live parameter references and can be copied into an RT request.
namespace G80
{
constexpr unsigned AxisCount = 6U; // X Y Z C U V
constexpr std::uint32_t MaximumSlowTouches = 64U;
constexpr std::uint32_t MaximumStableSamples = 4000U;
constexpr double SampleSeconds = 0.00025;

struct AxisGains
{
    double kp = 100.0;
    double ki = 0.0;
    double kd = 0.0;
    double kvff = 1.0;
    double maxIntegral = 0.0;
    // mm on X/Y/Z and degrees on C/U/V. Does not replace machine protection.
    double followingLimitNative = 0.1;
};

struct Config
{
    // All defaults are adjustable commissioning values, not machine approval.
    // Explicit opt-in is required after checking the contact voltage and travel.
    bool enabled = false;
    std::uint32_t slowTouches = 3U;
    std::uint32_t contactStableSamples = 2U;
    std::uint32_t releaseStableSamples = 4U;
    double contactVoltageV = 10.0;
    double releaseHysteresisV = 2.0;
    double slowSpeedMmMin = 60.0;
    double fastSpeedMmMin = 300.0;
    double slowSpeedDegMin = 60.0;
    double fastSpeedDegMin = 300.0;
    double retreatMm = 0.2;
    double retreatDeg = 0.2;
    double maxMeanErrorMm = 0.002;
    double maxMeanErrorDeg = 0.005;
    double maxSearchMm = 10.0;
    double maxSearchDeg = 10.0;
    double maxCycleSeconds = 120.0;
    double releaseTimeoutSeconds = 5.0;
    double settleTimeoutSeconds = 5.0;
    // G80-only ramp times, independently frozen per cycle. These apply to its
    // paired rapid approach, fast/slow seeks, retreat and controlled braking.
    // A standalone G00 retains its own profile. No G00 ramp value belongs here.
    double accelerationSeconds = 0.2;
    double decelerationSeconds = 0.2;
    double settleWindowMm = 0.002;
    double settleWindowDeg = 0.005;
    double settleHoldSeconds = 0.05;
    double derivativeFilterSeconds = 0.002;
    // Used only by the explicitly selected, per-command G80 Q1 source.
    // These fields never switch an ordinary G80 away from physical GAP.
    std::uint32_t simulationScenario = 0U;
    double simulationEdgeDistanceMm = 0.5, simulationEdgeDistanceDeg = 1.0;
    double simulationSpreadMm = 0.001, simulationSpreadDeg = 0.001;
    double simulationLossDistanceMm = 0.25, simulationLossDistanceDeg = 0.5;
    std::array<AxisGains, AxisCount> gains{};
};

static_assert(std::is_trivially_copyable<Config>::value,
    "G80 parameters must remain an RT-copyable value.");

enum class ConfigLoadStatus : std::uint32_t
{
    Loaded = 0U,
    MissingDisabled = 1U,
    Invalid = 2U
};

// NC/startup only. All 61 motion keys are required. The seven Simulation keys
// are optional as one complete group. Keys are case-sensitive and unique.
// UTF-8 BOM and // comments are accepted; unknown keys are rejected.
// Missing/invalid input resets the output to disabled commissioning defaults.
bool ValidateConfig(const Config& config, std::string& error) noexcept;
// Pure per-axis process-ramp caps from installed native maximum speed and the
// two G80 times only. XYZ caps are mm/s^2; CUV caps are deg/s^2. A caller may
// further reduce either cap using a separately defined machine/hardware limit,
// never a G00 profile time. Failure leaves both output values unchanged.
bool BuildNativeRampCaps(const Config& config, double maximumVelocityNative,
    double& accelerationNative, double& decelerationNative) noexcept;
ConfigLoadStatus LoadConfig(const std::string& path, Config& config,
    std::string& error) noexcept;
}
