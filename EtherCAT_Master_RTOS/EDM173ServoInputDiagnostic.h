#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

// Observation only. The PDO/Motion owner supplies already-read input and
// controller state; this recorder cannot write a PDO, grant motion authority,
// change a guard, allocate, print, or read a clock. Publication is external.
// "raw" is a sampled pInput memory value, not proof of what the drive sent.
// MissingRaw means this diagnostic observation was unavailable; no trigger
// here is itself a motion fault or a claim about physical drive health.
namespace EDM173InputDiag
{
constexpr unsigned AxisCount = 6U;
constexpr std::uint32_t AllAxes = (1U << AxisCount) - 1U;

enum AxisFlag : std::uint8_t
{
    ServoOn = 1U << 0U,
    Fault = 1U << 1U,
    LagAlarm = 1U << 2U,
    ResetRequest = 1U << 3U
};

enum Trigger : std::uint32_t
{
    MissingInput = 1U << 0U,
    MissingRaw = 1U << 1U,
    RawMismatch = 1U << 2U,
    NotReady = 1U << 3U,
    InternalBad = 1U << 4U,
    Incomplete = 1U << 5U
};

struct AxisSample
{
    std::int32_t consumedPosition = 0, rawPosition = 0;
    std::uint16_t consumedStatus = 0U, rawStatus = 0U;
    std::uint16_t controlBefore = 0U, controlAfter = 0U;
    std::int16_t stateBefore = 0, stateAfter = 0;
    std::int8_t consumedMode = 0, rawMode = 0;
    std::uint8_t flagsBefore = 0U, flagsAfter = 0U;
    bool rawValid = false, outputValid = false, semantic = false;
    // The adapter knows MotionState. No enum value is assumed here.
    // Supply isFault || isLagAlarm || state==ERROR || state==ESTOP before
    // and after the existing UpdateMotion/UpdateServoState calls.
    bool internalBadBefore = false, internalBadAfter = false;
};

struct Frame
{
    std::uint64_t session = 0ULL, tick = 0ULL, nowUs = 0ULL, epoch = 0ULL;
    std::uint32_t inputMask = 0U, rawMask = 0U, semanticMask = 0U;
    std::uint32_t rawMismatchMask = 0U, notReadyMask = 0U;
    std::uint32_t internalFaultMask = 0U, triggerMask = 0U;
    bool complete = false;
    AxisSample axis[AxisCount]{};
};

struct Evidence
{
    std::uint64_t session = 0ULL, frames = 0ULL;
    bool havePrevious = false, haveFirst = false;
    // Previous is the last complete healthy observation of this session,
    // not necessarily tick-1. Its own tick/time must be printed explicitly.
    Frame previous{}, first{};
};

static_assert(std::is_trivially_copyable<Evidence>::value,
    "Servo input evidence must be a value snapshot.");
static_assert(sizeof(Evidence) <= 2048U,
    "Servo input evidence must fit its bounded diagnostic channel.");

class Recorder
{
public:
    void Reset() noexcept
    {
        m_evidence = Evidence{};
        m_frame = Frame{};
        m_open = false;
    }

    // Return false once the first failure is frozen, so the adapter can skip
    // all additional observations. A different session resets both frames.
    bool Begin(std::uint64_t session, std::uint64_t tick,
        std::uint64_t nowUs, std::uint64_t epoch) noexcept
    {
        if (session == 0ULL || tick == 0ULL) return false;
        if (m_evidence.session == session && m_evidence.haveFirst) return false;
        m_open = false;
        if (m_evidence.session != session)
        {
            Reset();
            m_evidence.session = session;
        }
        m_frame = Frame{};
        m_frame.session = session;
        m_frame.tick = tick;
        m_frame.nowUs = nowUs;
        m_frame.epoch = epoch;
        m_open = true;
        return true;
    }

    void RecordAxis(unsigned index, const AxisSample& sample) noexcept
    {
        if (!m_open || index >= AxisCount) return;
        m_frame.axis[index] = sample;
        const std::uint32_t bit = 1U << index;
        m_frame.inputMask |= bit;
        // A repeated observation replaces this axis consistently, including
        // availability masks; no earlier value may describe the new sample.
        m_frame.rawMask &= ~bit;
        m_frame.semanticMask &= ~bit;
        m_frame.rawMismatchMask &= ~bit;
        m_frame.notReadyMask &= ~bit;
        m_frame.internalFaultMask &= ~bit;
        if (sample.rawValid) m_frame.rawMask |= bit;
        if (sample.semantic) m_frame.semanticMask |= bit;
        if (sample.rawValid &&
            (sample.consumedStatus != sample.rawStatus ||
             sample.consumedMode != sample.rawMode ||
             sample.consumedPosition != sample.rawPosition))
            m_frame.rawMismatchMask |= bit;
        if ((sample.consumedStatus & 0x006fU) != 0x0027U ||
            sample.consumedMode != 9)
            m_frame.notReadyMask |= bit;
        if (sample.internalBadBefore || sample.internalBadAfter)
            m_frame.internalFaultMask |= bit;
    }

    // Returns true only when this call freezes the first abnormal frame.
    // The return value is diagnostic and must never drive a control decision.
    bool End(bool complete) noexcept
    {
        if (!m_open) return false;
        m_open = false;
        ++m_evidence.frames;
        m_frame.complete = complete;
        std::uint32_t triggers = 0U;
        if (!complete) triggers |= Incomplete;
        if (m_frame.inputMask != AllAxes) triggers |= MissingInput;
        if (m_frame.rawMask != AllAxes) triggers |= MissingRaw;
        if (m_frame.rawMismatchMask != 0U) triggers |= RawMismatch;
        if (m_frame.notReadyMask != 0U) triggers |= NotReady;
        if (m_frame.internalFaultMask != 0U) triggers |= InternalBad;
        m_frame.triggerMask = triggers;
        if (triggers != 0U)
        {
            m_evidence.first = m_frame;
            m_evidence.haveFirst = true;
            return true;
        }
        m_evidence.previous = m_frame;
        m_evidence.havePrevious = true;
        return false;
    }

    const Evidence& Snapshot() const noexcept { return m_evidence; }

private:
    Evidence m_evidence{};
    Frame m_frame{};
    bool m_open = false;
};
} // namespace EDM173InputDiag
