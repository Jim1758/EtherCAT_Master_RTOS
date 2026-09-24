#include "CompensationEngine.h"
#include "MotionCore.h"
#include "AlarmManager.h"
#include <cmath>

bool CompensationEngine::CanConfigure(int axisIndex) const noexcept
{
    return axisIndex >= 0 && axisIndex < static_cast<int>(pbc::AxisCount) && !IsSealed();
}

bool CompensationEngine::InitAxisCompensation(int axisIndex, bool enBacklash,
    double b_Pos, double b_Neg, double b_Speed, bool enPitch,
    double startPos, double step, double p_Speed)
{
    if (!CanConfigure(axisIndex)) return false;
    Pending& next = m_pending[static_cast<std::size_t>(axisIndex)];
    next = Pending{}; // explicit boot configuration starts with empty paired tables
    next.config.backlash = enBacklash;
    next.config.backlashPositive = b_Pos;
    next.config.backlashNegative = b_Neg;
    next.config.backlashSpeed = b_Speed;
    next.config.pitch = enPitch;
    next.config.pitchStart = startPos;
    next.config.pitchStep = step;
    next.config.pitchSpeed = p_Speed;
    next.initialized = true;
    return true;
}

bool CompensationEngine::SetAxisPolicy(int axisIndex, const pbc::Policy& policy)
{
    if (!CanConfigure(axisIndex)) return false;
    m_pending[static_cast<std::size_t>(axisIndex)].config.policy = policy;
    m_pending[static_cast<std::size_t>(axisIndex)].explicitPolicy = true;
    return true;
}

namespace
{
    bool ValidTableVector(const std::vector<double>& table) noexcept
    {
        if (table.size() > pbc::MaxTableRows) return false;
        for (double value : table) if (!std::isfinite(value)) return false;
        return true;
    }
}

bool CompensationEngine::SetPitchTables(int axisIndex,
    const std::vector<double>& pos, const std::vector<double>& neg)
{
    if (!CanConfigure(axisIndex) || !ValidTableVector(pos) || !ValidTableVector(neg)) return false;
    try
    {
        std::vector<double> nextPos(pos), nextNeg(neg);
        Pending& pending = m_pending[static_cast<std::size_t>(axisIndex)];
        pending.positive.swap(nextPos);
        pending.negative.swap(nextNeg);
        return true;
    }
    catch (...) { return false; }
}

bool CompensationEngine::SetPitchTablePos(int axisIndex, const std::vector<double>& values)
{
    if (!CanConfigure(axisIndex) || !ValidTableVector(values)) return false;
    try
    {
        std::vector<double> candidate(values);
        m_pending[static_cast<std::size_t>(axisIndex)].positive.swap(candidate);
        return true;
    }
    catch (...) { return false; }
}

bool CompensationEngine::SetPitchTableNeg(int axisIndex, const std::vector<double>& values)
{
    if (!CanConfigure(axisIndex) || !ValidTableVector(values)) return false;
    try
    {
        std::vector<double> candidate(values);
        m_pending[static_cast<std::size_t>(axisIndex)].negative.swap(candidate);
        return true;
    }
    catch (...) { return false; }
}

bool CompensationEngine::HasEnabledPitch() const noexcept
{
    for (const Pending& p : m_pending) if (p.initialized && p.config.pitch) return true;
    return false;
}
unsigned CompensationEngine::EnabledAxisCount() const noexcept
{
    unsigned count = 0U;
    for (const Pending& p : m_pending)
        if (p.initialized && (p.config.pitch || p.config.backlash)) ++count;
    return count;
}

bool CompensationEngine::FinalizeConfiguration(
    const std::vector<AxisContext>& axes, pbc::Diagnostic& d)
{
    d = pbc::Diagnostic{};
    if (IsSealed()) { d.error = pbc::Error::ConfigurationSealed; return false; }
    if (axes.empty() || axes.size() > pbc::AxisCount)
    { d.error = pbc::Error::AxisIndex; return false; }
    if (!pbc::CheckCoordinateContract(m_coordinateContractChecks))
    { d.error = pbc::Error::CoordinateContract; return false; }
    std::array<pbc::AxisModel, pbc::AxisCount> candidates{};
    int firstEnabledAxis = -1;
    for (std::size_t i = 0U; i < axes.size(); ++i)
    {
        const AxisContext& axis = axes[i];
        const Pending& pending = m_pending[i];
        if (axis.axisIndex != static_cast<int>(i) || !pending.initialized)
        { d.error = pbc::Error::AxisIndex; d.axis = static_cast<int>(i); return false; }
        pbc::Config config = pending.config;
        if (axis.enablePitch != config.pitch || axis.enableBacklash != config.backlash)
        { d.error = pbc::Error::NotConfigured; d.axis = static_cast<int>(i); return false; }
        if (!axis.isExist && (config.pitch || config.backlash))
        { d.error = pbc::Error::AxisIndex; d.axis = static_cast<int>(i); return false; }
        config.rotary = axis.axisType == AxisType::ROTARY || axis.axisType == AxisType::ROTARY_CONTINUOUS;
        config.motorEncoder = axis.fbMode == FeedbackSource::MOTOR_ENCODER;
        config.period = axis.rotaryModulo;
        config.pulsePerUnit = (std::isfinite(axis.resolution_PPR) && axis.resolution_PPR > 0.0 &&
            std::isfinite(axis.finalLead) && axis.finalLead > 0.0) ?
            axis.resolution_PPR / axis.finalLead : 0.0;
        if (!pending.explicitPolicy) config.policy.periodic = config.rotary;
        if (!candidates[i].Configure(config, pending.positive, pending.negative, d))
        { d.axis = static_cast<int>(i); return false; }
        if ((config.pitch || config.backlash) && firstEnabledAxis < 0)
            firstEnabledAxis = static_cast<int>(i);
    }
    // No calibration file, NC command or flag may bypass a missing Motion
    // integration. Error specificity above is preserved (e.g. Data7 X=8.5).
    if (firstEnabledAxis >= 0 && !MotionIntegrationReleased)
    {
        d.error = pbc::Error::IntegrationPending;
        d.axis = firstEnabledAxis;
        return false;
    }
    m_models.swap(candidates);
    m_sealed.store(true, std::memory_order_release);
    return true;
}

bool CompensationEngine::ApplyCompensation(
    int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt)
{
    (void)dt;
    const bool indexValid = axisIndex >= 0 && axisIndex < static_cast<int>(pbc::AxisCount);
    if (indexValid && IsSealed() && !axis.enablePitch && !axis.enableBacklash &&
        pbc::HasDisabledCoordinateIdentity(false, axis.currentCompOffset_unit,
            axis.mechanicalCompensationFrame) &&
        m_models[static_cast<std::size_t>(axisIndex)].IsConfigured() &&
        !m_models[static_cast<std::size_t>(axisIndex)].IsEnabled())
    {
        return true; // no arithmetic, command mutation, allocation, lock or log
    }

    // Defensive containment of forbidden live reconfiguration. Normal PBC-1
    // startup rejects it BEFORE EtherCAT starts. Existing final-frame Alarm
    // authority also fences output; this does not release the old active path.
    axis.isFault = true;
    axis.pid.integralAcc = 0.0;
    axis.pid.prevError = 0.0;
    cmd.instantCmdPos = axis.currentActPos;
    cmd.instantCmdVel = 0.0;
    if (!indexValid || !m_runtimeFaultReported[static_cast<std::size_t>(axisIndex)])
    {
        if (indexValid) m_runtimeFaultReported[static_cast<std::size_t>(axisIndex)] = true;
        AlarmManager::GetInstance().Trigger(AlarmManager::MECHANICAL_COMPENSATION_RUNTIME_REJECTED,
            0, axisIndex);
    }
    return false;
}
