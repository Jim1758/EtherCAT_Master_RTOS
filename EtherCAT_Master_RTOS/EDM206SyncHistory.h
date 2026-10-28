#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <type_traits>
#include "EDMSixAxisPhysicalGuard.h"

// EDM275 also records the sealed short-flush B1 test; control authority is unchanged.
// Observation only. One RT writer and one NC reader; no control authority,
// clock read, allocation, printing, mutex, retry loop or whole-history copy.
// The ring lives in startup-owned Motion storage, never on the RT stack.
namespace EDM206
{
constexpr unsigned Capacity = 128U;
constexpr unsigned AxisCount = 3U;
constexpr std::uint32_t XYZ = 7U;
enum RowFlag : std::uint32_t {
    Complete = 1U, FrameCurrent = 2U, MappingCurrent = 4U,
    SourceIntervalValid = 8U, FaultSample = 16U, PartialFault = 32U,
    SourceFresh = 64U, ForceZero = 128U, ClockValid = 256U
};
enum AxisFlag : std::uint32_t {
    ControlCurrent = 1U, WrittenCurrent = 2U, FlushSelected = 4U,
    FrameAvailable = 8U, ReferenceCurrent = 16U
};
// Same consumed input pass; raw velocity/object units have not been verified.
enum class InputSource : std::uint8_t { Unknown, Legacy, ControlledSemantic, RetiredSemantic, EmergencyFallback };
enum InputFlag : std::uint16_t {
    ConsumedValid=1U, MappedValid=2U, Decoded=4U, FirstCycle=8U,
    RawVelocityValid=16U, RawObjectValid=32U
};
struct InputObservation
{
    std::uint64_t session=0ULL,tick=0ULL,sourceUs=0ULL;
    double unwrappedPulse=0.0,rawLogicalPulse=0.0,preFixtureActualPulse=0.0;
    std::int32_t consumedRaw=0,mappedRaw=0,pulseDelta=0,rawVelocity=0;
    std::uint32_t previousRaw=0U,rawObject2510=0U;
    std::uint16_t flags=0U;
    InputSource source=InputSource::Unknown;
};
enum ConfigFlag : std::uint32_t { InputOffsetValid=1U, OutputOffsetValid=2U };
struct AxisConfig
{
    std::uint64_t sampleTick=0ULL,sampleUs=0ULL,inputAddress=0ULL,outputAddress=0ULL;
    double resolutionPPR=0.0,finalLead=0.0,machineOffsetPulse=0.0;
    std::int32_t slot=-1,axisIndex=-1,slaveIndex=-1,fbMode=-1,axisType=-1;
    std::int32_t inputOffset=-1,outputOffset=-1;
    std::uint32_t flags=0U;
    bool isReverse=false,axisReverse=false;
};
struct InputCapture { EDM94::Identity identity{}; InputObservation input{}; AxisConfig config{}; };
struct Axis
{
    InputObservation input{};
    EDM94::AxisControlSample control{}; // Exact LOCAL_PRE_GUARD_PI sample.
    double kd = 0.0;
    double frameCommandPulse = 0.0, frameActualPulse = 0.0, followingErrorPulse = 0.0;
    double pulsePerUnit = 0.0, strictWindowPulse = 0.0, committedReferencePps = 0.0;
    std::uint64_t writtenTick = 0ULL;
    std::int32_t writtenCsv = 0, hardwareSign = 0;
    std::uint32_t flags = 0U; // writtenCsv is IMAGE_BEFORE_GROUP_GUARD, not sent.
};
struct Accepted
{
    EDM94::Identity identity{};
    std::uint64_t tick = 0ULL, sourceUs = 0ULL, frameGeneration = 0ULL;
    std::array<std::int32_t, AxisCount> csv{};
    // Valid only after sealed + attempted + NIC accepted + exact ObserveSent.
    // SourceUs belongs to the associated frame, never the current send clock.
    bool valid = false, allSixZero = false;
};
struct Row
{
    std::uint64_t session = 0ULL, tick = 0ULL, sourceUs = 0ULL, sourceDeltaUs = 0ULL;
    std::uint64_t epoch = 0ULL;
    std::uint32_t ownerGeneration = 0U, phase = 0U, child = 0U, mode = 0U;
    std::uint32_t flags = 0U, controlMask = 0U, writtenMask = 0U, inputMask = 0U;
    std::array<Axis, AxisCount> axis{};
    Accepted priorAccepted{};
};
struct Header
{
    std::uint64_t session = 0ULL, faultTick = 0ULL, faultUs = 0ULL;
    std::uint64_t firstTick = 0ULL, lastTick = 0ULL, totalRows = 0ULL;
    std::uint64_t containmentTick = 0ULL, containmentSourceUs = 0ULL;
    std::uint32_t reason = 0U, axis = EDM94::AxisCount, site = 0U, mode = 0U;
    std::uint32_t count = 0U, capacity = Capacity;
    bool faultRowPartial = false, zeroImageAccepted = false;
    // FAULT_SAMPLE only: retained from that row's input pass, never reread at Freeze.
    std::uint32_t configMask=0U;
    std::array<AxisConfig,AxisCount> config{};
};
static_assert(ATOMIC_INT_LOCK_FREE == 2, "Diagnostic ownership requires lock-free unsigned atomics.");
static_assert(std::is_trivially_copyable<Row>::value, "History rows are immutable value records.");
static_assert(sizeof(Row) <= 1024U, "One bounded history row must remain small.");

class Recorder
{
public:
    // Only the RT writer calls Observe*. Session changes may discard a live
    // untriggered ring; a frozen/published/reader-owned ring is never replaced.
    void ObserveInput(unsigned index, const InputCapture& capture, unsigned mode) noexcept
    {
        const auto& input=capture.input;
        if(index>=AxisCount || !input.tick || !input.sourceUs ||
            (input.flags&(ConsumedValid|Decoded))!=(ConsumedValid|Decoded) ||
            (input.source<InputSource::Legacy || input.source>InputSource::EmergencyFallback) ||
            !EDM94::ValidIdentity(capture.identity) || capture.identity.session!=input.session ||
            !Start(input.session,mode))return;
        m_pendingInput[index]=capture;
    }
    void ObserveControl(unsigned index, const EDM94::AxisControlSample& sample,
        double kd, bool flushSelected, unsigned mode) noexcept
    {
        if (index >= AxisCount || !Start(sample.identity.session, mode)) return;
        Axis& a = m_pending[index]; a = Axis{};
        a.control = sample; a.kd = kd;
        if (flushSelected) a.flags |= FlushSelected;
    }
    void ObserveWritten(unsigned index, std::uint64_t session, unsigned mode,
        std::uint64_t tick, std::int32_t csv) noexcept
    {
        if (index >= AxisCount || !Start(session, mode)) return;
        m_pending[index].writtenTick = tick; m_pending[index].writtenCsv = csv;
    }
    void ObserveFrame(const EDM94::Frame& frame, const EDM94::Config& config,
        unsigned mode, unsigned phase, unsigned child, bool complete,
        bool mappingCurrent, const std::array<double, 6U>& reference, bool forceZero) noexcept
    {
        if (!Start(frame.identity.session, mode) || !frame.tick ||
            (m_count && frame.tick <= Last().tick)) return;
        const std::uint32_t intervalFlags = Complete | FrameCurrent | SourceFresh | ClockValid;
        const bool intervalCurrent = m_count && complete && frame.sourceFresh && frame.clockValid &&
            (Last().flags & intervalFlags) == intervalFlags &&
            Last().epoch == frame.identity.epoch && Last().ownerGeneration == frame.identity.ownerGeneration;
        Row& row = Append(frame.identity, frame.tick, frame.nowUs, mode, phase, child);
        if (!intervalCurrent) { row.flags &= ~SourceIntervalValid; row.sourceDeltaUs = 0ULL; }
        row.flags |= FrameCurrent;
        if (complete) row.flags |= Complete;
        if (mappingCurrent) row.flags |= MappingCurrent;
        if (frame.sourceFresh) row.flags |= SourceFresh;
        if (frame.clockValid) row.flags |= ClockValid;
        if (forceZero) row.flags |= ForceZero;
        for (unsigned i = 0U; i < AxisCount; ++i) {
            Axis& a = row.axis[i]; const auto& f = frame.axis[i]; const auto& c = config.axis[i];
            a.frameCommandPulse = f.commandPulse; a.frameActualPulse = f.actualPulse;
            a.followingErrorPulse = f.followingErrorPulse; a.pulsePerUnit = c.pulsePerUnit;
            a.strictWindowPulse = c.followingWindowPulse; a.hardwareSign = c.hardwareSign;
            a.flags |= FrameAvailable;
            if (mappingCurrent) { a.committedReferencePps = reference[i]; a.flags |= ReferenceCurrent; }
        }
    }
    // A fault before the complete Motion pass retains an honest partial row.
    // No missing field is filled from another tick; masks declare availability.
    void Freeze(const EDM94::RuntimeDiagnostic& fault, unsigned faultAxis, unsigned mode,
        unsigned phase, unsigned child) noexcept
    {
        if (!(fault.flags & EDM94::RuntimeFlag_SNAPSHOT_VALID) || !fault.tick ||
            !Start(fault.frameIdentity.session, mode)) return;
        if (!m_count || Last().tick != fault.tick) {
            if (m_count && Last().tick > fault.tick) return; // stale diagnostic is not this ring's fault
            Row& partial = Append(fault.frameIdentity, fault.tick, fault.nowUs, mode, phase, child);
            partial.flags |= PartialFault;
            partial.flags &= ~SourceIntervalValid; partial.sourceDeltaUs = 0ULL;
        }
        Row& row = Last(); row.flags |= FaultSample;
        // The inherited E206 row may predate an early same-tick identity fault.
        // New source/config fields must not claim that different fault sample.
        if(row.session!=fault.frameIdentity.session || row.epoch!=fault.frameIdentity.epoch ||
            row.ownerGeneration!=fault.frameIdentity.ownerGeneration || row.sourceUs!=fault.nowUs) {
            row.inputMask=0U;for(auto& axis:row.axis)axis.input=InputObservation{};
            m_lastConfig={};m_lastConfigMask=0U;
        }
        m_header = Header{}; m_header.session = m_session;
        m_header.faultTick = fault.tick; m_header.faultUs = fault.nowUs;
        m_header.reason = fault.reason; m_header.axis = faultAxis; m_header.site = fault.site;
        m_header.mode = mode; m_header.count = m_count; m_header.totalRows = m_totalRows;
        m_first = (m_next + Capacity - m_count) % Capacity;
        m_header.firstTick = m_rows[m_first].tick; m_header.lastTick = row.tick;
        m_header.faultRowPartial = (row.flags & PartialFault) != 0U;
        m_header.configMask=m_lastConfigMask;
        m_header.config=m_lastConfig;
        m_lastFrozenSession = m_session;
        m_state.store(Frozen, std::memory_order_release);
    }
    // Actual exact NIC receipt, dated from its own already-captured frame.
    // Updating this RT-private cache never mutates any frozen row.
    void ObserveAccepted(const EDM94::Identity& identity, std::uint64_t tick,
        std::uint64_t sourceUs, std::uint64_t generation,
        const std::int32_t (&csv)[EDM94::AxisCount], bool exactAccepted) noexcept
    {
        if (!exactAccepted || !identity.session || !tick || !sourceUs || !generation) return;
        if (m_accepted.identity.session == identity.session && tick <= m_accepted.tick) return;
        m_accepted = Accepted{}; m_accepted.identity = identity;
        m_accepted.tick = tick; m_accepted.sourceUs = sourceUs; m_accepted.frameGeneration = generation;
        m_accepted.valid = true; m_accepted.allSixZero = true;
        for (unsigned i = 0U; i < EDM94::AxisCount; ++i) {
            if (i < AxisCount) m_accepted.csv[i] = csv[i];
            m_accepted.allSixZero = m_accepted.allSixZero && csv[i] == 0;
        }
    }
    void PublishContained(std::uint64_t session, bool formalStop, bool forceZero,
        bool commandGroupIdle, bool commandZero) noexcept
    {
        if (m_state.load(std::memory_order_relaxed) != Frozen || session != m_header.session ||
            !formalStop || !forceZero || !commandGroupIdle || !commandZero ||
            !m_accepted.valid || !m_accepted.allSixZero || m_accepted.identity.session != session ||
            m_accepted.tick < m_header.faultTick) return;
        m_header.containmentTick = m_accepted.tick; m_header.containmentSourceUs = m_accepted.sourceUs;
        m_header.zeroImageAccepted = true;
        m_state.store(Ready, std::memory_order_release);
    }
    // One NC reader owns all payload until EndRead. Claim is one CAS, no spin.
    bool BeginRead(Header& out) noexcept
    {
        unsigned expected = Ready;
        if (!m_state.compare_exchange_strong(expected, Reading,
            std::memory_order_acquire, std::memory_order_relaxed)) return false;
        out = m_header; return true;
    }
    bool ReadRow(std::uint64_t session, unsigned index, Row& out) const noexcept
    {
        if (m_state.load(std::memory_order_acquire) != Reading ||
            session != m_header.session || index >= m_header.count) return false;
        out = m_rows[(m_first + index) % Capacity]; return true;
    }
    void EndRead(std::uint64_t session) noexcept
    {
        if (m_state.load(std::memory_order_relaxed) == Reading && session == m_header.session)
            m_state.store(Idle, std::memory_order_release);
    }
private:
    enum : unsigned { Idle, Recording, Frozen, Ready, Reading };
    bool Start(std::uint64_t session, unsigned mode) noexcept
    {
        if (!session || (mode != 1U && mode != 3U && mode != 4U) || session == m_lastFrozenSession) return false;
        const unsigned state = m_state.load(std::memory_order_acquire);
        if (state != Idle && state != Recording) return false;
        if (state == Idle || m_session != session) {
            m_session = session; m_next = m_count = 0U; m_totalRows = 0ULL;
            for (auto& a : m_pending) a = Axis{};
            for(auto& input:m_pendingInput)input=InputCapture{};
            m_lastConfig={};m_lastConfigMask=0U;
            m_state.store(Recording, std::memory_order_relaxed);
        }
        return true;
    }
    Row& Last() noexcept { return m_rows[(m_next + Capacity - 1U) % Capacity]; }
    Row& Append(const EDM94::Identity& identity, std::uint64_t tick,
        std::uint64_t sourceUs, unsigned mode, unsigned phase, unsigned child) noexcept
    {
        const std::uint64_t priorTick = m_count ? Last().tick : 0ULL;
        const std::uint64_t priorUs = m_count ? Last().sourceUs : 0ULL;
        Row& row = m_rows[m_next]; row = Row{};
        row.session = identity.session; row.tick = tick; row.sourceUs = sourceUs;
        row.epoch = identity.epoch; row.ownerGeneration = identity.ownerGeneration;
        row.mode = mode; row.phase = phase; row.child = child;
        if (priorTick && tick == priorTick + 1ULL && sourceUs > priorUs) {
            row.sourceDeltaUs = sourceUs - priorUs; row.flags |= SourceIntervalValid;
        }
        m_lastConfig={};m_lastConfigMask=0U;
        for (unsigned i = 0U; i < AxisCount; ++i) {
            const auto& pending = m_pending[i]; Axis& a = row.axis[i];
            if (pending.control.tick == tick && pending.control.nowUs == sourceUs &&
                EDM94::SameIdentity(pending.control.identity, identity)) {
                a = pending; a.flags |= ControlCurrent; row.controlMask |= 1U << i;
            }
            if (pending.writtenTick == tick) {
                a.writtenTick = tick; a.writtenCsv = pending.writtenCsv;
                a.flags |= WrittenCurrent; row.writtenMask |= 1U << i;
            } else { a.writtenTick = 0ULL; a.writtenCsv = 0; }
            const auto& captured=m_pendingInput[i];const auto& input=captured.input;
            if(EDM94::SameIdentity(captured.identity,identity) && input.session==identity.session && input.tick==tick && input.sourceUs==sourceUs &&
                (input.flags&(ConsumedValid|Decoded))==(ConsumedValid|Decoded) &&
                input.source!=InputSource::Unknown) {
                a.input=input;row.inputMask|=1U<<i;
                const auto& config=captured.config;
                if(config.sampleTick==tick && config.sampleUs==sourceUs && config.slot==static_cast<std::int32_t>(i)) {
                    m_lastConfig[i]=config;m_lastConfigMask|=1U<<i;
                }
            }
        }
        if (m_accepted.valid && EDM94::SameIdentity(m_accepted.identity, identity) &&
            m_accepted.tick < tick && m_accepted.sourceUs < sourceUs) row.priorAccepted = m_accepted;
        m_next = (m_next + 1U) % Capacity;
        if (m_count < Capacity) ++m_count;
        if (m_totalRows != (std::numeric_limits<std::uint64_t>::max)()) ++m_totalRows;
        return row;
    }
    std::array<Row, Capacity> m_rows{};
    std::array<Axis, AxisCount> m_pending{};
    std::array<InputCapture,AxisCount> m_pendingInput{};
    std::array<AxisConfig,AxisCount> m_lastConfig{};
    std::uint32_t m_lastConfigMask=0U;
    Accepted m_accepted{}; Header m_header{};
    std::atomic<unsigned> m_state{Idle};
    std::uint64_t m_session = 0ULL, m_lastFrozenSession = 0ULL, m_totalRows = 0ULL;
    unsigned m_next = 0U, m_count = 0U, m_first = 0U;
};
} // namespace EDM206
