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
    std::vector<AxisContext>& axes, pbc::Diagnostic& d)
{
    d = pbc::Diagnostic{};
    if (IsSealed()) { d.error = pbc::Error::ConfigurationSealed; return false; }
    if (axes.empty() || axes.size() > pbc::AxisCount)
    { d.error = pbc::Error::AxisIndex; return false; }
    if (!pbc::CheckCoordinateContract(m_coordinateContractChecks))
    { d.error = pbc::Error::CoordinateContract; return false; }
    if (!pbc::CheckControlFrameContract(m_xControlChecks))
    { d.error = pbc::Error::CoordinateContract; return false; }
    // PBC-3B model checks are boot-only; no live axis or PDO is touched.
    try
    {
        if (!pbc::CheckLifecycleContract(m_lifecycleContractChecks))
        { d.error = pbc::Error::LifecycleContract; return false; }
        if (!pbc::CheckXReferenceContract(m_xReferenceChecks))
        { d.error = pbc::Error::LifecycleContract; return false; }
    }
    catch (...) { d.error = pbc::Error::Allocation; return false; }
    std::array<pbc::AxisLifecycle, pbc::AxisCount> candidates{};
    bool zeroOnlyXCandidate = false;
    bool pitchOnlyXCandidate = false;
    bool backlashOnlyXCandidate = false;
    bool combinedXCandidate = false;
    bool asymmetricCombinedXCandidate = false;
    bool directionalPitchOnlyXCandidate = false;
    bool directionalCombinedXCandidate = false;
    int firstUnreleasedAxis = -1;
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
        if (config.pitch || config.backlash)
        {
            bool zeroCalibration = config.backlashPositive == 0.0 &&
                config.backlashNegative == 0.0;
            for (double value : pending.positive) zeroCalibration = zeroCalibration && value == 0.0;
            for (double value : pending.negative) zeroCalibration = zeroCalibration && value == 0.0;
            // All staged profiles bind every physical/configuration scalar.
            // Calibration admission is separate from the numerical boot frame.
            const bool physicalX = i == 0U &&
                axis.isExist && !axis.isVirtualAxis && axis.axisType == AxisType::LINEAR &&
                axis.fbMode == FeedbackSource::MOTOR_ENCODER &&
                axis.backlashAmount_Pos_mm == config.backlashPositive &&
                axis.backlashAmount_Neg_mm == config.backlashNegative &&
                axis.backlashSpeed == config.backlashSpeed &&
                axis.pitchStartPos_mm == config.pitchStart && axis.pitchStep_mm == config.pitchStep &&
                axis.pitchSpeed_mm_s == config.pitchSpeed &&
                std::isfinite(config.backlashSpeed) && config.backlashSpeed > 0.0 && config.backlashSpeed <= 100.0 &&
                std::isfinite(config.pitchStart) && std::isfinite(config.pitchStep) && config.pitchStep > 0.0 &&
                std::isfinite(config.pitchSpeed) && config.pitchSpeed > 0.0 && config.pitchSpeed <= 100.0 &&
                std::isfinite(axis.currentCmdPos) && std::isfinite(axis.currentActPos) &&
                axis.currentCmdVel == 0.0 && axis.logicalCmdVel == 0.0 &&
                axis.currentCompOffset_unit == 0.0 &&
                !axis.mechanicalCompensationFrame.valid && !axis.mechanicalCompensationFrame.enabled &&
                pbc::SameControlCoordinateFrame(axis.mechanicalCompensationFrame, pbc::CoordinateFrame{});
            const bool zeroPhysicalX = ZeroOnlyXIntegrationReleased && physicalX && zeroCalibration;
            bool boundedSymmetricPitch = config.pitch && !config.backlash &&
                config.backlashPositive == 0.0 && config.backlashNegative == 0.0 &&
                !config.policy.periodic && config.pitchSpeed > 0.0 && config.pitchSpeed <= 0.1 &&
                pending.positive.size() >= 2U && pending.positive.size() == pending.negative.size();
            for (std::size_t row = 0U; row < pending.positive.size() && boundedSymmetricPitch; ++row)
            {
                boundedSymmetricPitch = pending.positive[row] == pending.negative[row] &&
                    std::abs(pending.positive[row]) <= 0.010 &&
                    (row == 0U || std::abs(pending.positive[row] - pending.positive[row - 1U]) /
                        config.pitchStep <= 0.01);
            }
            const bool pitchPhysicalX = PitchOnlyXIntegrationReleased && physicalX &&
                !zeroPhysicalX && boundedSymmetricPitch;
            // Require an explicit complete zero pair even with pitch OFF.
            // Otherwise missing files or a retained K table could silently
            // qualify as an untested mixed/dormant calibration profile.
            bool zeroPitchPair = pending.positive.size() >= 2U &&
                pending.positive.size() == pending.negative.size();
            for (std::size_t row = 0U; row < pending.positive.size() && zeroPitchPair; ++row)
                zeroPitchPair = pending.positive[row] == 0.0 && pending.negative[row] == 0.0;
            const bool boundedSymmetricBacklash = config.backlash && !config.pitch &&
                config.backlashPositive > 0.0 && config.backlashPositive <= 0.010 &&
                config.backlashPositive == config.backlashNegative &&
                config.backlashSpeed > 0.0 && config.backlashSpeed <= 0.1 &&
                !config.policy.periodic && zeroPitchPair;
            const bool backlashPhysicalX = BacklashOnlyXIntegrationReleased && physicalX &&
                !zeroPhysicalX && boundedSymmetricBacklash;
            // M admits the tested sum of two nonzero calibrations, not arbitrary
            // mixed profiles. Bound each possible component sum, including the
            // branch whose backlash sign reinforces a negative pitch value.
            bool boundedSymmetricCombined = config.pitch && config.backlash &&
                config.backlashPositive > 0.0 &&
                config.backlashPositive == config.backlashNegative &&
                config.backlashSpeed > 0.0 && config.backlashSpeed <= 0.1 &&
                config.pitchSpeed > 0.0 && config.pitchSpeed <= 0.1 &&
                !config.policy.periodic && pending.positive.size() >= 2U &&
                pending.positive.size() == pending.negative.size();
            bool nonzeroCombinedPitch = false;
            for (std::size_t row = 0U; row < pending.positive.size() && boundedSymmetricCombined; ++row)
            {
                const double value = pending.positive[row];
                nonzeroCombinedPitch = nonzeroCombinedPitch || value != 0.0;
                boundedSymmetricCombined = value == pending.negative[row] &&
                    std::abs(value) + config.backlashPositive <= 0.010 &&
                    (row == 0U || std::abs(value - pending.positive[row - 1U]) /
                        config.pitchStep <= 0.01);
            }
            const bool combinedPhysicalX = CombinedXIntegrationReleased && physicalX &&
                !zeroPhysicalX && boundedSymmetricCombined && nonzeroCombinedPitch;
            // N is a separate profile: unequal backlash magnitudes with the
            // same pitch lookup for both directions. The larger branch bounds
            // both signs, including negative pitch and reverse travel.
            const double largestBacklash = config.backlashPositive > config.backlashNegative ?
                config.backlashPositive : config.backlashNegative;
            bool boundedAsymmetricCombined = config.pitch && config.backlash &&
                config.backlashPositive > 0.0 && config.backlashNegative > 0.0 &&
                config.backlashPositive != config.backlashNegative &&
                config.backlashSpeed > 0.0 && config.backlashSpeed <= 0.1 &&
                config.pitchSpeed > 0.0 && config.pitchSpeed <= 0.1 &&
                !config.policy.periodic && pending.positive.size() >= 2U &&
                pending.positive.size() == pending.negative.size();
            bool nonzeroAsymmetricCombinedPitch = false;
            for (std::size_t row = 0U; row < pending.positive.size() && boundedAsymmetricCombined; ++row)
            {
                const double value = pending.positive[row];
                nonzeroAsymmetricCombinedPitch = nonzeroAsymmetricCombinedPitch || value != 0.0;
                boundedAsymmetricCombined = value == pending.negative[row] &&
                    std::abs(value) + largestBacklash <= 0.010 &&
                    (row == 0U || std::abs(value - pending.positive[row - 1U]) /
                        config.pitchStep <= 0.01);
            }
            const bool asymmetricCombinedPhysicalX = AsymmetricCombinedXIntegrationReleased && physicalX &&
                !zeroPhysicalX && boundedAsymmetricCombined && nonzeroAsymmetricCombinedPitch;
            // O selects signed pitch values by nominal direction. Validate
            // both tables, including negative-only slopes and extrema. Keeping
            // backlash OFF/zero avoids admitting a second reversal correction.
            bool boundedDirectionalPitch = config.pitch && !config.backlash &&
                config.backlashPositive == 0.0 && config.backlashNegative == 0.0 &&
                !config.policy.periodic && config.pitchSpeed > 0.0 && config.pitchSpeed <= 0.1 &&
                pending.positive.size() >= 2U && pending.positive.size() == pending.negative.size();
            bool distinctPitchPair = false;
            for (std::size_t row = 0U; row < pending.positive.size() && boundedDirectionalPitch; ++row)
            {
                const double positive = pending.positive[row];
                const double negative = pending.negative[row];
                distinctPitchPair = distinctPitchPair || positive != negative;
                boundedDirectionalPitch = std::abs(positive) <= 0.010 && std::abs(negative) <= 0.010 &&
                    (row == 0U ||
                        (std::abs(positive - pending.positive[row - 1U]) / config.pitchStep <= 0.01 &&
                         std::abs(negative - pending.negative[row - 1U]) / config.pitchStep <= 0.01));
            }
            const bool directionalPitchPhysicalX = DirectionalPitchOnlyXIntegrationReleased && physicalX &&
                !zeroPhysicalX && boundedDirectionalPitch && distinctPitchPair;
            // P deliberately combines two independent calibrated components.
            // Require acknowledgement even for an exact PN difference below
            // the model's directional-comparison tolerance. No implicit policy
            // opt-in, table re-signing or removal of a measured reversal term.
            bool boundedDirectionalCombined = config.pitch && config.backlash &&
                pending.explicitPolicy && config.policy.allowDirectionalPitchWithBacklash &&
                config.backlashPositive > 0.0 && config.backlashNegative > 0.0 &&
                config.backlashPositive != config.backlashNegative &&
                config.backlashSpeed > 0.0 && config.backlashSpeed <= 0.1 &&
                config.pitchSpeed > 0.0 && config.pitchSpeed <= 0.1 &&
                !config.policy.periodic && pending.positive.size() >= 2U &&
                pending.positive.size() == pending.negative.size();
            bool distinctCombinedPitchPair = false;
            for (std::size_t row = 0U; row < pending.positive.size() && boundedDirectionalCombined; ++row)
            {
                const double positive = pending.positive[row];
                const double negative = pending.negative[row];
                const double largestPitch = std::abs(positive) > std::abs(negative) ?
                    std::abs(positive) : std::abs(negative);
                distinctCombinedPitchPair = distinctCombinedPitchPair || positive != negative;
                boundedDirectionalCombined = largestPitch + largestBacklash <= 0.010 &&
                    (row == 0U ||
                        (std::abs(positive - pending.positive[row - 1U]) / config.pitchStep <= 0.01 &&
                         std::abs(negative - pending.negative[row - 1U]) / config.pitchStep <= 0.01));
            }
            const bool directionalCombinedPhysicalX = DirectionalCombinedXIntegrationReleased && physicalX &&
                !zeroPhysicalX && boundedDirectionalCombined && distinctCombinedPitchPair;
            if (i == 0U)
            {
                zeroOnlyXCandidate = zeroPhysicalX;
                pitchOnlyXCandidate = pitchPhysicalX;
                backlashOnlyXCandidate = backlashPhysicalX;
                combinedXCandidate = combinedPhysicalX;
                asymmetricCombinedXCandidate = asymmetricCombinedPhysicalX;
                directionalPitchOnlyXCandidate = directionalPitchPhysicalX;
                directionalCombinedXCandidate = directionalCombinedPhysicalX;
            }
            if (!zeroPhysicalX && !pitchPhysicalX && !backlashPhysicalX &&
                !combinedPhysicalX && !asymmetricCombinedPhysicalX && !directionalPitchPhysicalX &&
                !directionalCombinedPhysicalX && firstUnreleasedAxis < 0)
                firstUnreleasedAxis = static_cast<int>(i);
        }
    }
    // Only the explicitly bounded X profiles pass the general lock.
    // Error specificity above is preserved (e.g. X=8.5).
    if (firstUnreleasedAxis >= 0 && !MotionIntegrationReleased)
    {
        d.error = pbc::Error::IntegrationPending;
        d.axis = firstUnreleasedAxis;
        return false;
    }
    pbc::CoordinateFrame zeroBootstrap{};
    if (zeroOnlyXCandidate || pitchOnlyXCandidate || backlashOnlyXCandidate || combinedXCandidate || asymmetricCombinedXCandidate || directionalPitchOnlyXCandidate || directionalCombinedXCandidate)
    {
        pbc::Config config = m_pending[0].config;
        config.pulsePerUnit = axes[0].resolution_PPR / axes[0].finalLead;
        pbc::Input input{}; input.nominalPulse = axes[0].currentCmdPos;
        pbc::Output output{}; output.servoPulse = input.nominalPulse;
        if (!pbc::BuildCoordinateFrame(config, input, output, zeroBootstrap))
        { d.error = pbc::Error::CoordinateContract; d.axis = 0; return false; }
    }
    m_models.swap(candidates);
    m_xZeroOnlyAdmitted = zeroOnlyXCandidate;
    m_xPitchOnlyAdmitted = pitchOnlyXCandidate;
    m_xBacklashOnlyAdmitted = backlashOnlyXCandidate;
    m_xCombinedAdmitted = combinedXCandidate;
    m_xAsymmetricCombinedAdmitted = asymmetricCombinedXCandidate;
    m_xDirectionalPitchOnlyAdmitted = directionalPitchOnlyXCandidate;
    m_xDirectionalCombinedAdmitted = directionalCombinedXCandidate;
    m_xZeroBootstrapFrame = zeroBootstrap;
    // Publish a validated numerical zero transform before pre-RT WCS/NC/HMI
    // reads. This does not send motion, increment referenceGeneration or HOME.
    if (zeroOnlyXCandidate || pitchOnlyXCandidate || backlashOnlyXCandidate || combinedXCandidate || asymmetricCombinedXCandidate || directionalPitchOnlyXCandidate || directionalCombinedXCandidate) axes[0].mechanicalCompensationFrame = zeroBootstrap;
    m_configuredXAxis = &axes[0];
    m_xConfiguredResolution = axes[0].resolution_PPR;
    m_xConfiguredLead = axes[0].finalLead;
    m_xConfiguredReverse = axes[0].isReverse;
    m_xConfiguredAxisReverse = axes[0].Axis_Reverse;
    m_xConfiguredPitch = axes[0].enablePitch;
    m_xConfiguredBacklash = axes[0].enableBacklash;
    m_sealed.store(true, std::memory_order_release);
    return true;
}

bool CompensationEngine::ApplyCompensation(
    int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt)
{
    (void)dt;
    const bool indexValid = axisIndex >= 0 && axisIndex < static_cast<int>(pbc::AxisCount);
    const bool sealed = IsSealed();
    // Admitted enabled profiles use the normal control proposal/send transaction.
    // This legacy seam never injects a second offset or mutates the command.
    if (axisIndex == 0 && (IsXHomeControlCoordinateIdentity(axis) ||
        (IsXStagedProfileConfiguration(axis) && HasKnownXAppliedFrame(axis)))) return true;
    if (!(sealed && (m_xZeroOnlyAdmitted || m_xPitchOnlyAdmitted || m_xBacklashOnlyAdmitted || m_xCombinedAdmitted || m_xAsymmetricCombinedAdmitted || m_xDirectionalPitchOnlyAdmitted || m_xDirectionalCombinedAdmitted) && &axis == m_configuredXAxis) &&
        indexValid && sealed && !axis.enablePitch && !axis.enableBacklash &&
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

// These APIs add a real reference transaction at existing stopped-coordinate
// seams. They never establish the external hardware HOME bit.
bool CompensationEngine::IsXConfiguredPhysicalScope(const AxisContext& axis) const noexcept
{
    return IsSealed() && &axis == m_configuredXAxis && axis.axisIndex == 0 &&
        axis.isExist && !axis.isVirtualAxis && axis.axisType == AxisType::LINEAR &&
        axis.fbMode == FeedbackSource::MOTOR_ENCODER &&
        axis.resolution_PPR == m_xConfiguredResolution && axis.finalLead == m_xConfiguredLead &&
        axis.isReverse == m_xConfiguredReverse && axis.Axis_Reverse == m_xConfiguredAxisReverse &&
        axis.enablePitch == m_xConfiguredPitch && axis.enableBacklash == m_xConfiguredBacklash &&
        m_models[0].IsConfigured() &&
        std::isfinite(axis.resolution_PPR) && axis.resolution_PPR > 0.0 &&
        std::isfinite(axis.finalLead) && axis.finalLead > 0.0 &&
        std::isfinite(axis.currentActPos) && std::isfinite(axis.currentCmdPos) &&
        (!(m_xZeroOnlyAdmitted || m_xPitchOnlyAdmitted || m_xBacklashOnlyAdmitted || m_xCombinedAdmitted || m_xAsymmetricCombinedAdmitted || m_xDirectionalPitchOnlyAdmitted || m_xDirectionalCombinedAdmitted) ||
            (axis.backlashAmount_Pos_mm == m_pending[0].config.backlashPositive &&
                axis.backlashAmount_Neg_mm == m_pending[0].config.backlashNegative &&
                axis.backlashSpeed == m_pending[0].config.backlashSpeed &&
                axis.pitchStartPos_mm == m_pending[0].config.pitchStart &&
                axis.pitchStep_mm == m_pending[0].config.pitchStep &&
                axis.pitchSpeed_mm_s == m_pending[0].config.pitchSpeed));
}

bool CompensationEngine::IsXConfiguredDisabledIdentity(const AxisContext& axis) const noexcept
{
    return IsXReferenceScope(axis);
}

bool CompensationEngine::IsXZeroOnlyConfiguration(const AxisContext& axis) const noexcept
{
    // Table/policy storage is private and sealed; IsXConfiguredPhysicalScope
    // additionally compares every mutable zero-profile compensation scalar.
    return IsSealed() && m_xZeroOnlyAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXZeroOnlyCoordinateIdentity(const AxisContext& axis) const noexcept
{
    if (!IsXZeroOnlyConfiguration(axis) || axis.currentCompOffset_unit != 0.0 ||
        m_models[0].Phase() == pbc::LifecyclePhase::OutputUncertain) return false;
    const pbc::State& state = m_models[0].Runtime();
    if (state.backlash != 0.0 || state.pitch != 0.0 || state.targetBacklash != 0.0 ||
        state.targetPitch != 0.0 || !state.settled) return false;
    const pbc::CoordinateFrame& frame = axis.mechanicalCompensationFrame;
    if (m_models[0].ReferenceGeneration() == 0ULL)
        return m_models[0].Phase() == pbc::LifecyclePhase::NeedsReference &&
            frame.enabled && pbc::IsCoordinateFrameValid(frame) &&
            pbc::SameControlCoordinateFrame(frame, m_xZeroBootstrapFrame);
    return frame.enabled && pbc::IsCoordinateFrameValid(frame) &&
        frame.offsetUnit == 0.0 && frame.offsetPulse == 0.0 &&
        frame.offsetVelocityPPS == 0.0 && frame.remainingUnit == 0.0 && frame.settled &&
        frame.pulsePerUnit == axis.resolution_PPR / axis.finalLead &&
        frame.servoCommandPulse == frame.nominalCommandPulse &&
        frame.servoVelocityPPS == frame.nominalVelocityPPS &&
        pbc::SameControlCoordinateFrame(frame, m_models[0].CommittedFrame());
}

bool CompensationEngine::IsXPitchOnlyConfiguration(const AxisContext& axis) const noexcept
{
    return IsSealed() && m_xPitchOnlyAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXBacklashOnlyConfiguration(const AxisContext& axis) const noexcept
{
    return IsSealed() && m_xBacklashOnlyAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXCombinedConfiguration(const AxisContext& axis) const noexcept
{
    return IsSealed() && m_xCombinedAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXAsymmetricCombinedConfiguration(const AxisContext& axis) const noexcept
{
    return IsSealed() && m_xAsymmetricCombinedAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXDirectionalPitchOnlyConfiguration(const AxisContext& axis) const noexcept
{
    return IsSealed() && m_xDirectionalPitchOnlyAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXDirectionalCombinedConfiguration(const AxisContext& axis) const noexcept
{
    return IsSealed() && m_xDirectionalCombinedAdmitted && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::IsXStagedProfileConfiguration(const AxisContext& axis) const noexcept
{
    return IsXStagedProfileAdmitted() && IsXConfiguredPhysicalScope(axis);
}

bool CompensationEngine::HasXExactZeroFrame(const AxisContext& axis) const noexcept
{
    if (!IsXStagedProfileConfiguration(axis) || axis.currentCompOffset_unit != 0.0 ||
        m_models[0].Phase() == pbc::LifecyclePhase::OutputUncertain) return false;
    const pbc::State& state = m_models[0].Runtime();
    if (state.backlash != 0.0 || state.pitch != 0.0 || state.targetBacklash != 0.0 ||
        state.targetPitch != 0.0 || !state.settled) return false;
    const pbc::CoordinateFrame& frame = axis.mechanicalCompensationFrame;
    if (!frame.enabled || !pbc::IsCoordinateFrameValid(frame) ||
        frame.offsetUnit != 0.0 || frame.offsetPulse != 0.0 ||
        frame.offsetVelocityPPS != 0.0 || frame.remainingUnit != 0.0 || !frame.settled ||
        frame.pulsePerUnit != axis.resolution_PPR / axis.finalLead ||
        frame.servoCommandPulse != frame.nominalCommandPulse ||
        frame.servoVelocityPPS != frame.nominalVelocityPPS) return false;
    if (m_models[0].ReferenceGeneration() == 0ULL)
        return m_models[0].Phase() == pbc::LifecyclePhase::NeedsReference &&
            !IsXPhysicalHomeEstablished() &&
            pbc::SameControlCoordinateFrame(frame, m_xZeroBootstrapFrame);
    return pbc::SameControlCoordinateFrame(frame, m_models[0].CommittedFrame());
}

bool CompensationEngine::IsXBootstrapCoordinateIdentity(const AxisContext& axis) const noexcept
{
    return m_models[0].ReferenceGeneration() == 0ULL && !m_models[0].HasPending() &&
        !m_xReferencePending.prepared && !m_xCyclePending.prepared && HasXExactZeroFrame(axis);
}

bool CompensationEngine::IsXHomeEntryCoordinateIdentity(const AxisContext& axis) const noexcept
{
    return IsXZeroOnlyCoordinateIdentity(axis) ||
        (!IsXPhysicalHomeEstablished() && HasXExactZeroFrame(axis));
}

bool CompensationEngine::IsXHomeControlCoordinateIdentity(const AxisContext& axis) const noexcept
{
    return IsXZeroOnlyCoordinateIdentity(axis) || HasXExactZeroFrame(axis);
}

bool CompensationEngine::IsXKnownAppliedControlFrame(const AxisContext& axis) const noexcept
{
    return HasKnownXAppliedFrame(axis);
}

bool CompensationEngine::RefreshXBootstrapFrame(AxisContext& axis,
    pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    if (!IsXBootstrapCoordinateIdentity(axis) || m_models[0].HasPending() ||
        m_xReferencePending.prepared || m_xCyclePending.prepared ||
        axis.currentCmdVel != 0.0 || axis.logicalCmdVel != 0.0)
    { d.error = pbc::Error::LifecycleRejected; return false; }
    pbc::Config config = m_pending[0].config;
    config.pulsePerUnit = axis.resolution_PPR / axis.finalLead;
    pbc::Input input{}; input.nominalPulse = axis.currentCmdPos;
    pbc::Output output{}; output.servoPulse = input.nominalPulse;
    pbc::CoordinateFrame candidate{};
    if (!pbc::BuildCoordinateFrame(config, input, output, candidate))
    { d.error = pbc::Error::CoordinateContract; return false; }
    m_xZeroBootstrapFrame = candidate;
    axis.mechanicalCompensationFrame = candidate;
    return true;
}

bool CompensationEngine::RefreshXZeroOnlyBootstrapFrame(AxisContext& axis,
    pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    if (!IsXZeroOnlyCoordinateIdentity(axis) || m_models[0].ReferenceGeneration() != 0ULL ||
        m_models[0].Phase() != pbc::LifecyclePhase::NeedsReference || m_models[0].HasPending() ||
        m_xReferencePending.prepared || m_xCyclePending.prepared ||
        axis.currentCmdVel != 0.0 || axis.logicalCmdVel != 0.0)
    { d.error = pbc::Error::LifecycleRejected; return false; }
    pbc::Config config = m_pending[0].config;
    config.pulsePerUnit = axis.resolution_PPR / axis.finalLead;
    pbc::Input input{}; input.nominalPulse = axis.currentCmdPos;
    pbc::Output output{}; output.servoPulse = input.nominalPulse;
    pbc::CoordinateFrame candidate{};
    if (!pbc::BuildCoordinateFrame(config, input, output, candidate))
    { d.error = pbc::Error::CoordinateContract; return false; }
    m_xZeroBootstrapFrame = candidate;
    axis.mechanicalCompensationFrame = candidate;
    return true;
}

bool CompensationEngine::IsXReferenceScope(const AxisContext& axis) const noexcept
{
    return IsXConfiguredPhysicalScope(axis) && !axis.enablePitch && !axis.enableBacklash &&
        !m_models[0].IsEnabled() &&
        pbc::HasDisabledCoordinateIdentity(false, axis.currentCompOffset_unit,
            axis.mechanicalCompensationFrame);
}

bool CompensationEngine::HasKnownXAppliedFrame(const AxisContext& axis) const noexcept
{
    return IsXConfiguredPhysicalScope(axis) && (axis.enablePitch || axis.enableBacklash) &&
        m_models[0].IsEnabled() && m_models[0].ReferenceGeneration() != 0ULL &&
        m_models[0].Phase() != pbc::LifecyclePhase::OutputUncertain &&
        pbc::MatchesAppliedCoordinateFrame(true, axis.currentCompOffset_unit,
            axis.resolution_PPR, axis.finalLead, axis.mechanicalCompensationFrame) &&
        pbc::SameControlCoordinateFrame(axis.mechanicalCompensationFrame,
            m_models[0].CommittedFrame());
}

bool CompensationEngine::IsXKnownOffsetReferenceReady(const AxisContext& axis) const noexcept
{
    return HasKnownXAppliedFrame(axis) && !m_models[0].HasPending() &&
        !m_xCyclePending.prepared && !m_xReferencePending.prepared;
}

bool CompensationEngine::IsXFrozenStopReady(const AxisContext& axis) const noexcept
{
    const pbc::CoordinateFrame& frame = m_models[0].CommittedFrame();
    // NC dwell is observed after the next output image is prepared. An exact
    // frozen continuation does not invalidate the prior accepted stop frame;
    // the pending candidate itself is never promoted into stop evidence.
    const bool pendingCompatible = !m_models[0].HasPending() ? !m_xCyclePending.prepared :
        m_xCyclePending.prepared &&
        m_xCyclePending.authority.identity.referenceGeneration == m_models[0].ReferenceGeneration() &&
        (m_xCyclePending.authority.mode == pbc::CycleMode::Hold ||
            m_xCyclePending.authority.mode == pbc::CycleMode::ControlledStop) &&
        pbc::SameControlCoordinateFrame(m_xCyclePending.frame, frame);
    return HasKnownXAppliedFrame(axis) && !m_xReferencePending.prepared && pendingCompatible &&
        m_models[0].Phase() == pbc::LifecyclePhase::Held &&
        m_xLastAppliedCycle.tick != 0ULL &&
        m_xLastAppliedCycle.referenceGeneration == m_models[0].ReferenceGeneration() &&
        (m_xLastAppliedCycleMode == pbc::CycleMode::Hold ||
            m_xLastAppliedCycleMode == pbc::CycleMode::ControlledStop) &&
        frame.nominalCommandPulse == axis.currentCmdPos &&
        frame.nominalVelocityPPS == 0.0 && frame.servoVelocityPPS == 0.0 &&
        frame.offsetVelocityPPS == 0.0;
}

bool CompensationEngine::PrepareXReferenceAuthority(const AxisContext& axis,
    const pbc::ReferenceAuthority& authority, XReferenceTransaction& transaction,
    pbc::Diagnostic& d, bool allowKnownOffset) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    transaction = XReferenceTransaction{};
    if ((!IsXReferenceScope(axis) && !IsXHomeEntryCoordinateIdentity(axis) &&
            !(allowKnownOffset && IsXKnownOffsetReferenceReady(axis))) ||
        m_xReferenceSequence == (std::numeric_limits<std::uint64_t>::max)())
    { d.error = pbc::Error::LifecycleRejected; return false; }
    transaction.authority = authority;
    transaction.authority.requestSequence = ++m_xReferenceSequence;
    return true;
}

bool CompensationEngine::PrepareXResetReference(const AxisContext& axis,
    const pbc::ReferenceAuthority& authority, XReferenceTransaction& transaction,
    pbc::Diagnostic& d) noexcept
{
    if (!PrepareXReferenceAuthority(axis, authority, transaction, d, true)) return false;
    const pbc::ReferenceAction action = m_models[0].ReferenceGeneration() == 0ULL ?
        pbc::ReferenceAction::Initial : pbc::ReferenceAction::ResetRetain;
    transaction.prepared = m_models[0].PrepareReference(action, axis.currentActPos,
        0.0, transaction.authority, transaction.plan, d);
    if (transaction.prepared) m_xReferencePending = transaction;
    d.axis = 0;
    return transaction.prepared;
}

bool CompensationEngine::PrepareXRetainRecoveryReference(const AxisContext& axis,
    XRetainRecoveryKind kind, const pbc::ReferenceAuthority& authority,
    XReferenceTransaction& transaction, pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    transaction = XReferenceTransaction{};
    if ((kind != XRetainRecoveryKind::Servo && kind != XRetainRecoveryKind::Fault &&
            kind != XRetainRecoveryKind::Limit) ||
        authority.homeOwner || !IsXKnownOffsetReferenceReady(axis))
    { d.error = pbc::Error::LifecycleRejected; return false; }
    if (!PrepareXReferenceAuthority(axis, authority, transaction, d, true)) return false;
    transaction.recoveryKind = kind;
    // ServoRetain denotes the same offset-only inverse for a qualified fault
    // or limit recovery. It neither clears HOME nor makes an uncertain offset known.
    transaction.prepared = m_models[0].PrepareReference(pbc::ReferenceAction::ServoRetain,
        axis.currentActPos, 0.0, transaction.authority, transaction.plan, d);
    if (transaction.prepared) m_xReferencePending = transaction;
    d.axis = 0;
    return transaction.prepared;
}

bool CompensationEngine::PrepareXHomeReference(const AxisContext& axis,
    double coordinateShiftPulse, const pbc::ReferenceAuthority& authority,
    XReferenceTransaction& transaction, pbc::Diagnostic& d) noexcept
{
    if (IsXStagedProfileAdmitted() && !IsXHomeEntryCoordinateIdentity(axis))
    {
        transaction = XReferenceTransaction{};
        d = pbc::Diagnostic{}; d.axis = 0; d.error = pbc::Error::LifecycleRejected;
        return false;
    }
    if (!PrepareXReferenceAuthority(axis, authority, transaction, d)) return false;
    transaction.prepared = m_models[0].PrepareHomeReferencePreservingCommand(
        axis.currentActPos, axis.currentCmdPos, coordinateShiftPulse,
        transaction.authority, transaction.plan, d);
    if (transaction.prepared) m_xReferencePending = transaction;
    d.axis = 0;
    return transaction.prepared;
}

bool CompensationEngine::FinishXReference(AxisContext& axis,
    const XReferenceTransaction& transaction, const pbc::ReferenceAuthority& current,
    pbc::ReferenceCommitResult result, pbc::Diagnostic& d) noexcept
{
    const bool applied = result == pbc::ReferenceCommitResult::Applied;
    const pbc::ReferencePlan& expected = m_xReferencePending.plan;
    const pbc::ReferencePlan& supplied = transaction.plan;
    const bool planMatches = transaction.prepared && m_xReferencePending.prepared &&
        transaction.recoveryKind == m_xReferencePending.recoveryKind &&
        supplied.ticket == expected.ticket && supplied.action == expected.action &&
        supplied.nominalCommandPulse == expected.nominalCommandPulse &&
        supplied.rawFeedbackPulse == expected.rawFeedbackPulse &&
        supplied.coordinateShiftPulse == expected.coordinateShiftPulse &&
        supplied.offsetPulse == expected.offsetPulse;
    // An untrusted/tampered cancellation must not consume a different recovery
    // transaction. Reported application still follows the fail-uncertain path.
    if (m_xReferencePending.recoveryKind != XRetainRecoveryKind::None &&
        !planMatches && result == pbc::ReferenceCommitResult::NotApplied)
    { d = pbc::Diagnostic{}; d.axis = 0; d.error = pbc::Error::LifecycleRejected; return false; }
    const bool recoveryReference = expected.action == pbc::ReferenceAction::ServoRetain &&
        (m_xReferencePending.recoveryKind == XRetainRecoveryKind::Servo ||
            m_xReferencePending.recoveryKind == XRetainRecoveryKind::Fault ||
            m_xReferencePending.recoveryKind == XRetainRecoveryKind::Limit);
    const bool referenceScope = IsXReferenceScope(axis) || IsXHomeEntryCoordinateIdentity(axis) ||
        ((expected.action == pbc::ReferenceAction::ResetRetain || recoveryReference) &&
            HasKnownXAppliedFrame(axis));
    const bool scalarsMatch = !applied || (planMatches && referenceScope &&
        axis.currentCmdPos == expected.nominalCommandPulse &&
        axis.currentActPos == expected.rawFeedbackPulse);
    const pbc::ReferenceCommitResult verifiedResult =
        (!transaction.prepared || !scalarsMatch) &&
            result != pbc::ReferenceCommitResult::NotApplied ?
            pbc::ReferenceCommitResult::Uncertain : result;
    const bool firstPhysicalHome = IsXStagedProfileAdmitted() &&
        expected.action == pbc::ReferenceAction::HomeClear;
    const bool finished = m_models[0].FinishReference(
        transaction.prepared ? transaction.plan.ticket : 0ULL,
        current, verifiedResult, d);
    d.axis = 0;
    if (!m_models[0].HasPending()) m_xReferencePending = XReferenceTransaction{};
    if (finished && applied)
    {
        if (firstPhysicalHome)
            m_xPhysicalHomeEstablished.store(true, std::memory_order_release);
        m_xLastReferenceCommitTick = current.tick;
        m_xLastAppliedCycle = pbc::CycleIdentity{};
        m_xLastAppliedCycleMode = pbc::CycleMode::Motion;
        // Keep raw feedback and all nominal command scalars owned by Motion.
        axis.mechanicalCompensationFrame = m_models[0].CommittedFrame();
        axis.currentCompOffset_unit = axis.mechanicalCompensationFrame.offsetUnit;
    }
    else if (m_models[0].Phase() == pbc::LifecyclePhase::OutputUncertain &&
        &axis == m_configuredXAxis)
    {
        m_xLastAppliedCycle = pbc::CycleIdentity{};
        m_xLastAppliedCycleMode = pbc::CycleMode::Motion;
        axis.mechanicalCompensationFrame.valid = false;
    }
    return finished;
}

bool CompensationEngine::TryNominalSnapPulse(int axisIndex, const AxisContext& axis,
    double& nominalPulse) const noexcept
{
    if (axisIndex == 0 && (IsXZeroOnlyCoordinateIdentity(axis) ||
        IsXBootstrapCoordinateIdentity(axis)))
    { nominalPulse = axis.currentActPos; return true; }
    if (IsSealed() && (m_xZeroOnlyAdmitted || m_xPitchOnlyAdmitted || m_xBacklashOnlyAdmitted || m_xCombinedAdmitted || m_xAsymmetricCombinedAdmitted || m_xDirectionalPitchOnlyAdmitted || m_xDirectionalCombinedAdmitted) &&
        &axis == m_configuredXAxis) return false;
    if (!IsSealed() || axisIndex < 0 || axisIndex >= static_cast<int>(pbc::AxisCount) ||
        axis.axisIndex != axisIndex || !axis.isExist || axis.isVirtualAxis ||
        !m_models[static_cast<std::size_t>(axisIndex)].IsConfigured() ||
        m_models[static_cast<std::size_t>(axisIndex)].IsEnabled() ||
        axis.enablePitch || axis.enableBacklash ||
        !pbc::HasDisabledCoordinateIdentity(false, axis.currentCompOffset_unit,
            axis.mechanicalCompensationFrame) || !std::isfinite(axis.currentActPos))
        return false;
    nominalPulse = axis.currentActPos;
    return true;
}

// PBC-3E cycle state is private to this configured physical X. The public
// transaction is a transport copy, never authority to substitute a new frame.
bool CompensationEngine::MatchesPendingXCycle(const XCycleTransaction& tx) const noexcept
{
    const XCycleTransaction& expected = m_xCyclePending;
    return tx.prepared && expected.prepared && tx.ticket == expected.ticket &&
        tx.authority.identity == expected.authority.identity &&
        tx.authority.mode == expected.authority.mode &&
        tx.authority.sourceCurrent == expected.authority.sourceCurrent &&
        tx.authority.rawServoReady == expected.authority.rawServoReady &&
        tx.authority.homed == expected.authority.homed &&
        pbc::SameControlCoordinateFrame(tx.frame, expected.frame);
}

CompensationEngine::XCyclePrepareResult CompensationEngine::PrepareXCycle(
    const AxisContext& axis, const AxisCommand& nominalCommand,
    const pbc::CycleAuthority& authority, double dt,
    XCycleTransaction& transaction, pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    transaction = XCycleTransaction{};
    if ((!IsXReferenceScope(axis) && !IsXHomeControlCoordinateIdentity(axis) && !HasKnownXAppliedFrame(axis)) ||
        !std::isfinite(nominalCommand.instantCmdPos) ||
        !std::isfinite(nominalCommand.instantCmdVel) || !std::isfinite(dt) ||
        dt <= 0.0 || dt > 0.01 || m_xCyclePending.prepared)
    {
        d.error = pbc::Error::LifecycleRejected;
        return XCyclePrepareResult::Rejected;
    }
    // Startup, a real HOME operation, and the reference commit's own tick have
    // no cycle candidate. OFF control remains the historical identity path.
    // These bypasses do not fabricate a reference or HOME qualification.
    if (m_models[0].ReferenceGeneration() == 0ULL || !axis.isHomed ||
        (IsXStagedProfileAdmitted() && !IsXPhysicalHomeEstablished()) ||
        axis.homeRuntime.active || authority.mode == pbc::CycleMode::HomeFrozen ||
        (m_xLastReferenceCommitTick != 0ULL &&
            authority.identity.tick == m_xLastReferenceCommitTick))
        return XCyclePrepareResult::Bypassed;
    if (!axis.isServoOn || axis.targetMode != 9 || axis.isFault || axis.isLagAlarm ||
        authority.homed != axis.isHomed || !std::isfinite(axis.maxVel_PPS) ||
        axis.maxVel_PPS <= 0.0)
    {
        d.error = pbc::Error::LifecycleRejected;
        return XCyclePrepareResult::Rejected;
    }
    pbc::Input input{};
    input.nominalPulse = nominalCommand.instantCmdPos;
    input.nominalVelocityPPS = nominalCommand.instantCmdVel;
    input.maxVelocityPPS = axis.maxVel_PPS;
    input.dt = dt;
    XCycleTransaction next{}; next.authority = authority;
    if (!m_models[0].PrepareCycle(input, authority, next.frame, next.ticket, d))
    {
        d.axis = 0;
        return XCyclePrepareResult::Rejected;
    }
    next.prepared = true;
    m_xCyclePending = next;
    transaction = next;
    return XCyclePrepareResult::Prepared;
}

bool CompensationEngine::SealXCycle(const XCycleTransaction& transaction,
    const pbc::CycleIdentity& current, const pbc::CoordinateFrame& actualFrame,
    pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    if (!MatchesPendingXCycle(transaction) ||
        !(current == m_xCyclePending.authority.identity) ||
        !pbc::SameControlCoordinateFrame(actualFrame, m_xCyclePending.frame))
    { d.error = pbc::Error::LifecycleRejected; return false; }
    const bool result = m_models[0].SealCycle(transaction.ticket, current, actualFrame, d);
    d.axis = 0;
    return result;
}

bool CompensationEngine::CancelXCycle(const XCycleTransaction& transaction,
    pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    if (!MatchesPendingXCycle(transaction))
    { d.error = pbc::Error::LifecycleRejected; return false; }
    const bool result = m_models[0].DiscardCycleProposal(transaction.ticket,
        transaction.authority.identity, d);
    d.axis = 0;
    if (result) m_xCyclePending = XCycleTransaction{};
    return result;
}

bool CompensationEngine::FinishXCycle(AxisContext& axis,
    const XCycleTransaction& transaction, const pbc::CycleIdentity& current,
    pbc::SendResult outcome, pbc::Diagnostic& d) noexcept
{
    d = pbc::Diagnostic{}; d.axis = 0;
    const bool matched = MatchesPendingXCycle(transaction) &&
        current == m_xCyclePending.authority.identity;
    const bool scopeCurrent = IsXReferenceScope(axis) || IsXHomeControlCoordinateIdentity(axis) || HasKnownXAppliedFrame(axis);
    bool result = false;
    switch (outcome)
    {
    case pbc::SendResult::Discarded:
    case pbc::SendResult::FenceAccepted:
        // The original candidate was not adopted. A successful NIC zero fence
        // is explicitly different from successful delivery of this candidate.
        if (!matched) { d.error = pbc::Error::LifecycleRejected; return false; }
        result = m_models[0].DiscardCycleProposal(transaction.ticket, current, d);
        break;
    case pbc::SendResult::OutputUncertain:
        if (matched && scopeCurrent && !m_models[0].IsEnabled() &&
            pbc::HasDisabledCoordinateIdentity(false, axis.currentCompOffset_unit,
                axis.mechanicalCompensationFrame))
        {
            // Transport still records OutputUncertain. No claim of 'not sent'
            // is made here: only a provably zero-offset proposal is discarded.
            // Thus a NIC diagnostic cannot make ordinary OFF RESET need HOME.
            result = m_models[0].DiscardCycleProposal(transaction.ticket, current, d);
        }
        else
            result = m_models[0].FinishSend(transaction.prepared ? transaction.ticket : 0ULL,
                current, true, false, d);
        break;
    case pbc::SendResult::HandoffAccepted:
        // A transport acceptance with a substituted transaction or mutated
        // physical scope cannot be attributed to our candidate. Treat that
        // verified outcome as uncertain rather than committing caller data.
        result = m_models[0].FinishSend(transaction.prepared ? transaction.ticket : 0ULL,
            current, true, matched && scopeCurrent, d);
        if (result)
        {
            m_xLastAppliedCycle = transaction.authority.identity;
            m_xLastAppliedCycleMode = transaction.authority.mode;
            axis.mechanicalCompensationFrame = m_models[0].CommittedFrame();
            axis.currentCompOffset_unit = axis.mechanicalCompensationFrame.offsetUnit;
        }
        break;
    default:
        d.error = pbc::Error::LifecycleRejected;
        return false;
    }
    d.axis = 0;
    if (!m_models[0].HasPending()) m_xCyclePending = XCycleTransaction{};
    if (m_models[0].Phase() == pbc::LifecyclePhase::OutputUncertain &&
        &axis == m_configuredXAxis)
    {
        m_xLastAppliedCycle = pbc::CycleIdentity{};
        m_xLastAppliedCycleMode = pbc::CycleMode::Motion;
        axis.mechanicalCompensationFrame.valid = false;
    }
    return result;
}
