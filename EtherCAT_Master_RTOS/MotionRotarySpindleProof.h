#pragma once
#include "MotionExecutionContract.h"
#include "MotionServoHandoffDiagnostic.h"
#include "NCSpindleDispatch.h"
#include <cmath>
#include <cstdint>
#include <limits>

// EDM289: bounded RT predicates shared with the live spindle adapter. A local
// SendPacket acceptance is only output evidence; every completion also needs
// the subsequent coherent physical input and a continuous 200-sample window.
namespace MotionRotarySpindleProof
{
inline bool ExactSend(const MotionServoHandoffSample& sample, MotionExecutionEpoch epoch,
    const MotionOwnerLease& owner, unsigned axisSlot, std::uint64_t tick,
    std::int32_t expectedHardwarePps) noexcept
{
    if (axisSlot >= 8U || !epoch || !tick || !owner.IsValid() || owner.owner != MotionOwner::AUTO ||
        !sample.sequence || !sample.frameGeneration || sample.idleScope || !sample.attempted ||
        !sample.apiAccepted || !sample.finalChecked || sample.scrubbed || sample.epoch != epoch ||
        sample.owner != static_cast<std::uint32_t>(owner.owner) || sample.ownerGeneration != owner.generation ||
        sample.sourceTick != tick) return false;
    const std::uint32_t bit = 1U << axisSlot;
    return (sample.slotMask & bit) != 0U && (sample.observedMask & bit) != 0U &&
        (sample.sourceInputMask & bit) != 0U &&
        (sample.sourceStatusWord[axisSlot] & 0x006FU) == 0x0027U && sample.sourceMode[axisSlot] == 9 &&
        sample.proposedVelocity[axisSlot] == expectedHardwarePps &&
        sample.copiedVelocity[axisSlot] == expectedHardwarePps;
}

// A safety cancellation proves the serialized final zero, not the original
// AUTO proposal. The caller must also retain the exact final serialized seal
// and bind the cancellation's lifecycle to the current safety stop ticket.
inline bool ExactSafetyZeroSend(const MotionServoHandoffSample& sample, MotionExecutionEpoch epoch,
    const MotionOwnerLease& owner, unsigned axisSlot, std::uint64_t tick) noexcept
{
    if (axisSlot >= 8U || !epoch || !tick || !owner.IsValid() || owner.owner != MotionOwner::SAFETY ||
        !sample.sequence || !sample.frameGeneration || sample.idleScope || !sample.attempted ||
        !sample.apiAccepted || !sample.finalChecked || sample.epoch != epoch ||
        sample.owner != static_cast<std::uint32_t>(owner.owner) || sample.ownerGeneration != owner.generation ||
        sample.sourceTick != tick) return false;
    const std::uint32_t bit = 1U << axisSlot;
    return (sample.slotMask & bit) != 0U && (sample.observedMask & bit) != 0U &&
        (sample.sourceInputMask & bit) != 0U &&
        (sample.sourceStatusWord[axisSlot] & 0x006FU) == 0x0027U && sample.sourceMode[axisSlot] == 9 &&
        sample.copiedVelocity[axisSlot] == 0;
}

// Compute the signed modular difference without an out-of-range unsigned to
// signed cast. Exactly half the 32-bit encoder count range is ambiguous;
// DecodeRpm rejects it rather than choosing an unprovable travel direction.
inline std::int32_t RawDelta(std::uint32_t raw, std::uint32_t previousRaw) noexcept
{
    const std::uint32_t delta = raw - previousRaw;
    const std::int64_t signedDelta = delta <= static_cast<std::uint32_t>((std::numeric_limits<std::int32_t>::max)()) ?
        static_cast<std::int64_t>(delta) : static_cast<std::int64_t>(delta) - 0x100000000LL;
    return static_cast<std::int32_t>(signedDelta);
}

inline bool DecodeRpm(std::uint32_t raw, std::uint32_t previousRaw,
    double pulsesPerOutputRevolution, bool driveReverse, bool axisReverse, double& rpm) noexcept
{
    rpm = 0.0;
    if (!std::isfinite(pulsesPerOutputRevolution) || pulsesPerOutputRevolution <= 0.0 ||
        raw - previousRaw == 0x80000000U) return false;
    double candidate = static_cast<double>(RawDelta(raw, previousRaw)) /
        pulsesPerOutputRevolution / NCRotarySpindle::CycleSeconds * 60.0;
    if (driveReverse != axisReverse) candidate = -candidate;
    if (!std::isfinite(candidate)) return false;
    rpm = candidate;
    return true;
}

class ReceiptWindow
{
public:
    bool Reset(const NCSpindleDispatch::Packet& packet, std::uint64_t appliedTick) noexcept
    {
        m_packet = {}; m_appliedTick = 0ULL; m_valid = false; ClearSamples();
        m_observedTick = m_observedPhysicalSequence = 0ULL;
        if (!packet.authority.Valid() || !packet.sequence || packet.spindle >= NCRotarySpindle::Count ||
            packet.axisSlot >= 8U || !std::isfinite(packet.signedRpm) || !appliedTick ||
            appliedTick == (std::numeric_limits<std::uint64_t>::max)()) return false;
        m_packet = packet; m_appliedTick = m_observedTick = appliedTick; m_valid = true;
        return true;
    }

    // actualRpm must come from this tick's coherent physical encoder sample.
    // priorCommandRpm and finalSendRequestSequence belong to the exact image
    // whose final send is validated at inputTick-1, not the next output proposal.
    bool Observe(std::uint64_t inputTick, std::uint64_t physicalSequence, double actualRpm,
        double priorCommandRpm, bool exactFinalSend, std::uint64_t finalSendTick,
        std::uint64_t finalSendRequestSequence, bool configurationCurrent, bool authorityCurrent,
        double stoppedToleranceRpm, double readyToleranceRpm) noexcept
    {
        const bool numeric = std::isfinite(actualRpm) && std::isfinite(priorCommandRpm) &&
            std::isfinite(stoppedToleranceRpm) && stoppedToleranceRpm > 0.0 &&
            std::isfinite(readyToleranceRpm) && readyToleranceRpm > 0.0;
        const bool safetyCancellation = m_packet.signedRpm == 0.0 &&
            (m_packet.kind == NCSpindleDispatch::Kind::AlarmStop ||
             m_packet.kind == NCSpindleDispatch::Kind::ResetStop ||
             m_packet.kind == NCSpindleDispatch::Kind::AuthorityLostStop);
        const bool sameTolerance = !m_window.samples ||
            (stoppedToleranceRpm == m_stoppedTolerance && readyToleranceRpm == m_readyTolerance);
        const bool adjacent = !m_window.samples ||
            (m_window.lastTick != (std::numeric_limits<std::uint64_t>::max)() &&
             m_window.lastPhysicalSequence != (std::numeric_limits<std::uint64_t>::max)() &&
             inputTick == m_window.lastTick + 1ULL && physicalSequence == m_window.lastPhysicalSequence + 1ULL);
        const bool fresh = inputTick > m_observedTick && physicalSequence > m_observedPhysicalSequence;
        if (inputTick > m_observedTick) m_observedTick = inputTick;
        if (physicalSequence > m_observedPhysicalSequence) m_observedPhysicalSequence = physicalSequence;
        const double trackingError = numeric ? std::fabs(actualRpm - m_packet.signedRpm) : 0.0;
        if (!m_valid || !numeric || !sameTolerance || !adjacent || !fresh || !inputTick || !physicalSequence ||
            inputTick <= m_appliedTick || !exactFinalSend || !configurationCurrent || (!authorityCurrent && !safetyCancellation) ||
            finalSendTick != inputTick - 1ULL || finalSendRequestSequence != m_packet.sequence ||
            priorCommandRpm != m_packet.signedRpm || !std::isfinite(trackingError) ||
            (m_packet.signedRpm == 0.0 ? std::fabs(actualRpm) > stoppedToleranceRpm : trackingError > readyToleranceRpm) ||
            m_window.samples == (std::numeric_limits<unsigned>::max)()) {
            ClearSamples(); return false;
        }
        if (!m_window.samples) {
            m_window.firstTick = inputTick; m_window.firstPhysicalSequence = physicalSequence;
            m_stoppedTolerance = stoppedToleranceRpm; m_readyTolerance = readyToleranceRpm;
        }
        ++m_window.samples;
        m_window.lastTick = inputTick; m_window.lastPhysicalSequence = physicalSequence;
        const double absolute = std::fabs(actualRpm);
        if (absolute > m_window.maximumAbsActualRpm) m_window.maximumAbsActualRpm = absolute;
        if (trackingError > m_window.maximumAbsTrackingErrorRpm) m_window.maximumAbsTrackingErrorRpm = trackingError;
        m_window.continuous = m_window.sourceValid = m_window.configurationCurrent = true;
        m_window.finalSendValidated = m_window.commandMatchedEverySample = true;
        m_actualRpm = actualRpm; m_commandRpm = priorCommandRpm;
        return m_window.Valid();
    }

    // Preserve publication/lifecycle decisions owned by the caller, and always
    // clear the proof fields on failure so a previously complete window cannot
    // survive a later bad sample or a replacement packet.
    bool Populate(NCSpindleDispatch::Receipt& receipt) const noexcept
    {
        receipt.packet = {}; receipt.window = {};
        receipt.appliedTick = receipt.appliedRequestSequence = 0ULL;
        receipt.firstFinalSendTick = receipt.lastFinalSendTick = receipt.finalSendRequestSequence = 0ULL;
        receipt.actualRpm = receipt.commandRpm = 0.0;
        receipt.zeroDemand = receipt.zeroSentEverySample = false;
        if (!m_valid || !m_window.Valid()) return false;
        receipt.packet = m_packet; receipt.window = m_window;
        receipt.appliedTick = m_appliedTick; receipt.appliedRequestSequence = m_packet.sequence;
        receipt.firstFinalSendTick = m_window.firstTick - 1ULL;
        receipt.lastFinalSendTick = m_window.lastTick - 1ULL;
        receipt.finalSendRequestSequence = m_packet.sequence;
        receipt.actualRpm = m_actualRpm; receipt.commandRpm = m_commandRpm;
        receipt.zeroDemand = receipt.zeroSentEverySample = m_packet.signedRpm == 0.0;
        return true;
    }

private:
    NCSpindleDispatch::Packet m_packet{};
    NCSpindleDispatch::Window m_window{};
    std::uint64_t m_appliedTick = 0ULL;
    std::uint64_t m_observedTick = 0ULL, m_observedPhysicalSequence = 0ULL;
    double m_actualRpm = 0.0, m_commandRpm = 0.0;
    double m_stoppedTolerance = 0.0, m_readyTolerance = 0.0;
    bool m_valid = false;
    void ClearSamples() noexcept
    {
        m_window = {}; m_actualRpm = m_commandRpm = 0.0;
        m_stoppedTolerance = m_readyTolerance = 0.0;
    }
};
}
