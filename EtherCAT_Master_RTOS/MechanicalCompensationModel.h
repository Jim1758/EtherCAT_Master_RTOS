#pragma once
// PBC-1 / 2026-09-24. Platform-independent mechanical compensation model.
// Native units: LINEAR=mm; ROTARY=degree. Table values are signed COMMAND
// corrections, not raw measurement errors. No allocation, I/O or locks in Step.
// This is a model contract, NOT permission to enable the legacy MotionCore seam.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace pbc
{
constexpr std::size_t AxisCount = 8U;
constexpr std::size_t MaxTableRows = 65536U;

enum class Error : unsigned
{
    None = 0, AxisIndex, FileOpen, FileRead, EmptyTable, TableColumns,
    TableNumber, TooManyRows, LineTooLong, ParameterFormat, DuplicateParameter,
    ParameterNumber, ParameterRange, Geometry, FeedbackUnsupported, TablePair,
    OffsetLimit, SlopeLimit, PeriodicGrid, PeriodicClosure, CombinedDirectional,
    ConfigurationSealed, NotConfigured, IntegrationPending, RuntimeNumber,
    RuntimeVelocity, Allocation, CoordinateContract
};

inline const char* ErrorName(Error e) noexcept
{
    switch (e)
    {
    case Error::None: return "OK";
    case Error::AxisIndex: return "AXIS_INDEX";
    case Error::FileOpen: return "FILE_OPEN";
    case Error::FileRead: return "FILE_READ";
    case Error::EmptyTable: return "TABLE_REQUIRES_TWO_ROWS";
    case Error::TableColumns: return "TABLE_REQUIRES_EXACTLY_8_COLUMNS";
    case Error::TableNumber: return "TABLE_NUMBER_INVALID";
    case Error::TooManyRows: return "TABLE_ROW_LIMIT";
    case Error::LineTooLong: return "LINE_LENGTH_LIMIT";
    case Error::ParameterFormat: return "PARAMETER_FORMAT";
    case Error::DuplicateParameter: return "DUPLICATE_PARAMETER";
    case Error::ParameterNumber: return "PARAMETER_NUMBER_INVALID";
    case Error::ParameterRange: return "PARAMETER_RANGE";
    case Error::Geometry: return "AXIS_GEOMETRY_INVALID";
    case Error::FeedbackUnsupported: return "MOTOR_ENCODER_REQUIRED";
    case Error::TablePair: return "DIRECTIONAL_TABLE_PAIR_INVALID";
    case Error::OffsetLimit: return "COMPENSATION_OFFSET_LIMIT";
    case Error::SlopeLimit: return "PITCH_TABLE_SLOPE_LIMIT";
    case Error::PeriodicGrid: return "PERIODIC_GRID_INVALID";
    case Error::PeriodicClosure: return "PERIODIC_TABLE_NOT_CLOSED";
    case Error::CombinedDirectional: return "DIRECTIONAL_PITCH_AND_BACKLASH_NOT_ACKNOWLEDGED";
    case Error::ConfigurationSealed: return "BOOT_ONLY_CONFIGURATION_SEALED";
    case Error::NotConfigured: return "CONFIGURATION_NOT_COMMITTED";
    case Error::IntegrationPending: return "MOTION_INTEGRATION_NOT_RELEASED";
    case Error::RuntimeNumber: return "RUNTIME_NUMBER_INVALID";
    case Error::RuntimeVelocity: return "NOMINAL_VELOCITY_EXCEEDS_AXIS_LIMIT";
    case Error::Allocation: return "CONFIGURATION_ALLOCATION_FAILED";
    case Error::CoordinateContract: return "COORDINATE_CONTRACT_SELF_CHECK_FAILED";
    }
    return "UNKNOWN";
}

struct Diagnostic
{
    Error error = Error::None;
    int axis = -1;
    std::size_t line = 0U; // source line for IO; 1-based table row for model validation
    std::size_t column = 0U;
};

struct Policy
{
    // Conservative commissioning guards, not a claim about any machine's
    // measured calibration. These never convert um->mm or arcsec->degrees.
    double maxAbsOffset = 0.1;
    double maxAbsSlope = 0.01;
    bool periodic = false;
    bool allowDirectionalPitchWithBacklash = false;
};

struct Config
{
    bool backlash = false;
    bool pitch = false;
    double backlashPositive = 0.0;
    double backlashNegative = 0.0;
    double backlashSpeed = 3.0;
    double pitchStart = 0.0;
    double pitchStep = 10.0;
    double pitchSpeed = 3.0;
    double pulsePerUnit = 1.0;
    double period = 360.0;
    bool rotary = false;
    bool motorEncoder = true;
    Policy policy{};
};

struct State
{
    int direction = 1;
    double backlash = 0.0;
    double pitch = 0.0;
    double targetBacklash = 0.0;
    double targetPitch = 0.0;
    double lastNominalPulse = 0.0;
    bool lastNominalValid = false;
    bool armedByMotion = false;
    bool settled = true;
};

struct Input
{
    double nominalPulse = 0.0;
    double nominalVelocityPPS = 0.0;
    double maxVelocityPPS = 1.0;
    double dt = 0.00025;
    bool servoOn = false;
    bool homed = false;
    bool mayAdvance = false; // owner/hold/home/safety policy belongs to Motion
};

struct Output
{
    double servoPulse = 0.0;
    double servoVelocityPPS = 0.0;
    double offsetUnit = 0.0;
    double offsetVelocityPPS = 0.0;
    double remainingUnit = 0.0;
    bool settled = true;
    int direction = 1;
};

inline double Approach(double current, double target, double maximumDelta) noexcept
{
    // Exact arrival; deliberately no legacy 0.0001-unit deadband.
    if (std::abs(target - current) <= maximumDelta) return target;
    return target > current ? current + maximumDelta : current - maximumDelta;
}

struct CoordinateFrame; // PBC-2 local command/feedback frame

class AxisModel
{
public:
    const Config& Configuration() const noexcept { return m_config; }
    const State& Runtime() const noexcept { return m_state; }
    bool IsConfigured() const noexcept { return m_configured; }
    bool IsEnabled() const noexcept { return m_config.backlash || m_config.pitch; }
    const std::vector<double>& PositiveTable() const noexcept { return m_positive; }
    const std::vector<double>& NegativeTable() const noexcept { return m_negative; }

    // BOOT / HOST only. Candidate validation and copying precede the commit.
    // The caller must seal configurations before publishing them to the RT loop.
    bool Configure(const Config& config, const std::vector<double>& positive,
        const std::vector<double>& negative, Diagnostic& d)
    {
        d = Diagnostic{};
        std::size_t last = 0U;
        if (!Validate(config, positive, negative, last, d)) return false;
        try
        {
            std::vector<double> nextPositive(positive);
            std::vector<double> nextNegative(negative);
            m_positive.swap(nextPositive);
            m_negative.swap(nextNegative);
            m_config = config;
            m_last = last;
            m_state = State{};
            m_configured = true;
            return true;
        }
        catch (...) { d.error = Error::Allocation; return false; }
    }

    // Explicit reference-boundary operation. Motion must first rebase the
    // nominal command consistently; calling this alone on a running axis is
    // NOT a safe HOME / Servo-off / RESET implementation.
    void ResetAtReference() noexcept { m_state = State{}; }

    bool Lookup(double nominalUnit, int direction, double& correction) const noexcept
    {
        if (!m_configured || !std::isfinite(nominalUnit)) return false;
        if (!m_config.pitch) { correction = 0.0; return true; }
        const std::vector<double>& table = direction < 0 ? m_negative : m_positive;
        double relative = 0.0;
        if (m_config.policy.periodic)
        {
            // Reduce before subtracting: avoids overflow of finite q-start.
            relative = std::fmod(nominalUnit, m_config.period) -
                std::fmod(m_config.pitchStart, m_config.period);
            relative = std::fmod(relative, m_config.period);
            if (relative < 0.0) relative += m_config.period;
            if (relative >= m_config.period) relative = 0.0;
        }
        else
        {
            if (nominalUnit <= m_config.pitchStart) { correction = table.front(); return true; }
            const double end = m_config.pitchStart +
                static_cast<double>(m_last) * m_config.pitchStep;
            if (nominalUnit >= end) { correction = table[m_last]; return true; }
            relative = nominalUnit - m_config.pitchStart;
        }
        const double index = relative / m_config.pitchStep;
        if (!std::isfinite(index) || index < 0.0) return false;
        if (index >= static_cast<double>(m_last)) { correction = table[m_last]; return true; }
        // The floating bounds above precede EVERY integer conversion.
        const std::size_t i = static_cast<std::size_t>(index);
        const double fraction = index - static_cast<double>(i);
        correction = table[i] * (1.0 - fraction) + table[i + 1U] * fraction;
        return std::isfinite(correction);
    }

    // Bounded pure-model RT operation; output and state remain untouched on
    // any rejection. This API intentionally has no EtherCAT / AxisContext access.
    bool Step(const Input& input, Output& output, Diagnostic& d) noexcept
    {
        State next{};
        Output candidate{};
        if (!EvaluateStep(input, next, candidate, d)) return false;
        m_state = next;
        output = candidate;
        return true;
    }

    // PBC-2: defined in MechanicalCompensationCoordinates.h. Both model state
    // and the full coordinate frame commit only after validation succeeds.
    bool StepCoordinateFrame(const Input& input, CoordinateFrame& frame,
        Diagnostic& d) noexcept;

private:
    bool EvaluateStep(const Input& input, State& nextState, Output& output,
        Diagnostic& d) const noexcept
    {
        d = Diagnostic{};
        if (!m_configured) { d.error = Error::NotConfigured; return false; }
        if (!std::isfinite(input.nominalPulse) || !std::isfinite(input.nominalVelocityPPS) ||
            !std::isfinite(input.dt) || input.dt <= 0.0 || input.dt > 0.01)
        { d.error = Error::RuntimeNumber; return false; }
        State next = m_state;
        Output result{};
        result.servoPulse = input.nominalPulse;
        result.servoVelocityPPS = input.nominalVelocityPPS;
        if (!IsEnabled()) { nextState = next; output = result; return true; }
        if (!std::isfinite(input.maxVelocityPPS) || input.maxVelocityPPS <= 0.0)
        { d.error = Error::RuntimeNumber; return false; }
        if (std::abs(input.nominalVelocityPPS) > input.maxVelocityPPS)
        { d.error = Error::RuntimeVelocity; return false; }
        const double nominalUnit = input.nominalPulse / m_config.pulsePerUnit;
        if (!std::isfinite(nominalUnit)) { d.error = Error::RuntimeNumber; return false; }
        const double previousOffset = next.backlash + next.pitch;
        if (!(input.servoOn && input.homed && input.mayAdvance))
        {
            // Retain the actually applied offset. Do NOT secretly clear it,
            // and do NOT continue a ramp while the owner forbids output.
            next.lastNominalValid = false;
            // PBC-2: a pause is NOT a completed correction. Preserve targets,
            // remaining distance and prior motion arming for an authorized
            // resume, even if the nominal endpoint itself has stopped.
            // Actual reference changes still require ResetAtReference at a
            // lifecycle boundary; this model cannot authorize that boundary.
            next.settled = next.backlash == next.targetBacklash &&
                next.pitch == next.targetPitch;
        }
        else
        {
            double displacement = 0.0;
            if (next.lastNominalValid)
            {
                displacement = input.nominalPulse - next.lastNominalPulse;
                if (!std::isfinite(displacement)) { d.error = Error::RuntimeNumber; return false; }
            }
            if (displacement > 0.0) { next.direction = 1; next.armedByMotion = true; }
            else if (displacement < 0.0) { next.direction = -1; next.armedByMotion = true; }
            else if (input.nominalVelocityPPS > 0.0) { next.direction = 1; next.armedByMotion = true; }
            else if (input.nominalVelocityPPS < 0.0) { next.direction = -1; next.armedByMotion = true; }
            next.lastNominalPulse = input.nominalPulse;
            next.lastNominalValid = true;
            // Servo ON / HOME alone does not initiate a calibration motion.
            if (next.armedByMotion)
            {
                next.targetBacklash = m_config.backlash ?
                    (next.direction > 0 ? m_config.backlashPositive : -m_config.backlashNegative) : 0.0;
                if (!Lookup(nominalUnit, next.direction, next.targetPitch))
                { d.error = Error::RuntimeNumber; return false; }
                const double candidateBacklash = m_config.backlash ? Approach(next.backlash, next.targetBacklash,
                    m_config.backlashSpeed * input.dt) : 0.0;
                const double candidatePitch = m_config.pitch ? Approach(next.pitch, next.targetPitch,
                    m_config.pitchSpeed * input.dt) : 0.0;
                const double changeBacklash = candidateBacklash - next.backlash;
                const double changePitch = candidatePitch - next.pitch;
                const double requestedChange = changeBacklash + changePitch;
                // Budget only the compensation part. Nominal trajectory is
                // never silently clipped. A pending ramp must block completion
                // when this model is integrated into Motion in the next stage.
                const double minimumChange = (-input.maxVelocityPPS - input.nominalVelocityPPS) *
                    input.dt / m_config.pulsePerUnit;
                const double maximumChange = (input.maxVelocityPPS - input.nominalVelocityPPS) *
                    input.dt / m_config.pulsePerUnit;
                double scale = 1.0;
                if (requestedChange > maximumChange && requestedChange > 0.0)
                    scale = maximumChange / requestedChange;
                else if (requestedChange < minimumChange && requestedChange < 0.0)
                    scale = minimumChange / requestedChange;
                scale = (std::max)(0.0, (std::min)(1.0, scale));
                next.backlash = scale == 1.0 ? candidateBacklash : next.backlash + changeBacklash * scale;
                next.pitch = scale == 1.0 ? candidatePitch : next.pitch + changePitch * scale;
                next.settled = (next.backlash == next.targetBacklash && next.pitch == next.targetPitch);
            }
        }
        result.offsetUnit = next.backlash + next.pitch;
        result.offsetVelocityPPS = (result.offsetUnit - previousOffset) *
            m_config.pulsePerUnit / input.dt;
        result.servoPulse += result.offsetUnit * m_config.pulsePerUnit;
        result.servoVelocityPPS += result.offsetVelocityPPS;
        result.remainingUnit = std::abs(next.targetBacklash - next.backlash) +
            std::abs(next.targetPitch - next.pitch);
        result.settled = next.settled;
        result.direction = next.direction;
        if (!std::isfinite(result.servoPulse) || !std::isfinite(result.servoVelocityPPS) ||
            !std::isfinite(result.offsetUnit) || !std::isfinite(result.remainingUnit) ||
            std::abs(result.offsetUnit) > m_config.policy.maxAbsOffset + 1.0e-12 ||
            std::abs(result.servoVelocityPPS) > input.maxVelocityPPS * (1.0 + 1.0e-12))
        { d.error = Error::RuntimeNumber; return false; }
        nextState = next;
        output = result;
        return true;
    }

private:
    static bool Validate(const Config& c, const std::vector<double>& p,
        const std::vector<double>& n, std::size_t& last, Diagnostic& d) noexcept
    {
        const auto fail = [&d](Error e, std::size_t row = 0U) noexcept
        { d.error = e; d.line = row; return false; };
        if (!std::isfinite(c.policy.maxAbsOffset) || c.policy.maxAbsOffset <= 0.0 ||
            c.policy.maxAbsOffset > 1.0 || !std::isfinite(c.policy.maxAbsSlope) ||
            c.policy.maxAbsSlope <= 0.0 || c.policy.maxAbsSlope > 0.1)
            return fail(Error::ParameterRange);
        // Dormant tables are stored but never treated as calibrated / usable.
        if (!(c.backlash || c.pitch)) return true;
        if (!c.motorEncoder) return fail(Error::FeedbackUnsupported);
        if (!std::isfinite(c.pulsePerUnit) || c.pulsePerUnit <= 0.0 ||
            (c.rotary && (!std::isfinite(c.period) || c.period <= 0.0))) return fail(Error::Geometry);
        if (c.backlash && (!std::isfinite(c.backlashPositive) || c.backlashPositive < 0.0 ||
            !std::isfinite(c.backlashNegative) || c.backlashNegative < 0.0 ||
            !std::isfinite(c.backlashSpeed) || c.backlashSpeed <= 0.0 || c.backlashSpeed > 100.0))
            return fail(Error::ParameterRange);
        double back = c.backlash ? (std::max)(c.backlashPositive, c.backlashNegative) : 0.0;
        if (back > c.policy.maxAbsOffset) return fail(Error::OffsetLimit);
        if (!c.pitch) return true;
        if (!std::isfinite(c.pitchStart) || !std::isfinite(c.pitchStep) || c.pitchStep <= 0.0 ||
            !std::isfinite(c.pitchSpeed) || c.pitchSpeed <= 0.0 || c.pitchSpeed > 100.0)
            return fail(Error::ParameterRange);
        if (p.size() < 2U || n.size() != p.size() || p.size() > MaxTableRows)
            return fail(Error::TablePair);
        last = p.size() - 1U;
        if (c.policy.periodic)
        {
            if (!c.rotary) return fail(Error::PeriodicGrid);
            const double cells = c.period / c.pitchStep;
            if (!std::isfinite(cells) || cells < 1.0 || cells > static_cast<double>(last))
                return fail(Error::PeriodicGrid);
            const double rounded = std::floor(cells + 0.5);
            if (std::abs(cells - rounded) > 1.0e-9) return fail(Error::PeriodicGrid);
            last = static_cast<std::size_t>(rounded);
        }
        else
        {
            const double end = c.pitchStart + static_cast<double>(last) * c.pitchStep;
            if (!std::isfinite(end) || end <= c.pitchStart) return fail(Error::Geometry);
        }
        bool directional = false;
        for (std::size_t i = 0U; i < p.size(); ++i)
        {
            if (!std::isfinite(p[i]) || !std::isfinite(n[i])) return fail(Error::TableNumber, i + 1U);
            if ((std::max)(std::abs(p[i]), std::abs(n[i])) + back > c.policy.maxAbsOffset)
                return fail(Error::OffsetLimit, i + 1U);
            if (i <= last && i > 0U &&
                ((std::abs(p[i] - p[i - 1U]) / c.pitchStep > c.policy.maxAbsSlope) ||
                 (std::abs(n[i] - n[i - 1U]) / c.pitchStep > c.policy.maxAbsSlope)))
                return fail(Error::SlopeLimit, i + 1U);
            if (std::abs(p[i] - n[i]) > 1.0e-12) directional = true;
            if (c.policy.periodic && i >= last &&
                (std::abs(p[i] - p[0]) > 1.0e-12 || std::abs(n[i] - n[0]) > 1.0e-12))
                return fail(Error::PeriodicClosure, i + 1U);
        }
        if (directional && back > 0.0 && !c.policy.allowDirectionalPitchWithBacklash)
            return fail(Error::CombinedDirectional);
        return true;
    }

    Config m_config{};
    State m_state{};
    std::vector<double> m_positive;
    std::vector<double> m_negative;
    std::size_t m_last = 0U;
    bool m_configured = false;
};
}
