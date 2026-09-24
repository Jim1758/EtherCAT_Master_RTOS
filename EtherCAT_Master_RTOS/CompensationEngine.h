#pragma once
#include "MechanicalCompensationCoordinates.h"
#include <atomic>
#include <vector>

struct AxisContext;
struct AxisCommand;

// PBC-2: nominal feedback / completion read-side integration is installed.
// Non-zero runtime injection remains LOCKED. HOME, RESET, Servo transitions,
// owner/send authority and held-position control still require integration.
// Changing this constant alone is NOT an implementation of those contracts.
class CompensationEngine
{
public:
    static constexpr bool MotionIntegrationReleased = false;
    CompensationEngine() = default;

    bool InitAxisCompensation(int axisIndex, bool enBacklash, double b_Pos,
        double b_Neg, double b_Speed, bool enPitch, double startPos,
        double step, double p_Speed);
    bool SetAxisPolicy(int axisIndex, const pbc::Policy& policy);
    bool SetPitchTables(int axisIndex, const std::vector<double>& pos,
        const std::vector<double>& neg);
    bool SetPitchTablePos(int axisIndex, const std::vector<double>& errors);
    bool SetPitchTableNeg(int axisIndex, const std::vector<double>& errors);

    // Boot thread only. All axis candidates validate before any commit.
    // Success seals the configuration for the lifetime of this engine.
    bool FinalizeConfiguration(const std::vector<AxisContext>& axes, pbc::Diagnostic& d);
    bool HasEnabledPitch() const noexcept;
    bool IsSealed() const noexcept { return m_sealed.load(std::memory_order_acquire); }
    unsigned EnabledAxisCount() const noexcept;
    unsigned CoordinateContractChecks() const noexcept { return m_coordinateContractChecks; }

    // Existing caller compatibility. The only released runtime path in PBC-1
    // is all-disabled/no-offset and is bit-for-bit command preserving.
    bool ApplyCompensation(int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt);

private:
    struct Pending
    {
        pbc::Config config{};
        std::vector<double> positive;
        std::vector<double> negative;
        bool initialized = false;
        bool explicitPolicy = false;
    };
    std::array<Pending, pbc::AxisCount> m_pending{};
    std::array<pbc::AxisModel, pbc::AxisCount> m_models{};
    std::array<bool, pbc::AxisCount> m_runtimeFaultReported{};
    std::atomic<bool> m_sealed{ false };
    unsigned m_coordinateContractChecks = 0U; // boot writer only
    bool CanConfigure(int axisIndex) const noexcept;
};
