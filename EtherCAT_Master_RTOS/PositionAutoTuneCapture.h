#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

// TUNE01: observation of a single ordinary CNC axis. No gain application,
// trajectory generation, physical-stop assertion or drive acknowledgement.
namespace PositionAutoTune {
constexpr std::size_t CaptureCapacity = 524288U;
constexpr double CapturePeriodSeconds = 0.00025;
// Four continuous RT intervals: nominal 1 ms. This is not a hard wall-clock
// stop deadline; source continuity separately permits at most 1 ms per tick.
constexpr std::size_t ActualVelocityIntervals = 4U;
constexpr std::uint64_t ActualVelocityWindowUs = 1000ULL;
// TUNE09: commanded-speed evidence, not differentiated encoder velocity.
// Exclude the first 50 ms after reaching a stable requested-speed plateau;
// each motion leg then needs 50 ms of qualified cruise observation.
constexpr std::uint32_t CruiseQualificationVersion = 1U;
constexpr std::uint64_t CruiseStableWindowUs = 50000ULL;
constexpr std::uint64_t CruiseMinimumObservedUs = 50000ULL;
constexpr double CruiseTargetRelativeTolerance = 0.001;
constexpr double CruiseStableRelativeTolerance = 0.00001;
enum class CaptureState : std::uint32_t { Idle=0, Preparing=1, Armed=2, Running=3, Complete=4, Fault=5 };
enum class CaptureReason : std::uint32_t {
    None=0, Cancelled=1, Configuration=2, Source=3, Clock=4, Authority=5,
    Input=6, GainsChanged=7, Boundary=8, Following=9, Velocity=10,
    Capacity=11, Timeout=12, MissingSample=13, Output=14, Interference=15,
    ActualVelocity=16, EncoderJump=17
};
enum class GainBank : std::uint32_t { Unknown=0, Motion=1, Idle=2 };
struct CaptureGains { double kp=0.0, ki=0.0, kd=0.0, kvff=0.0; };
struct CaptureRequest {
    std::uint64_t session=0ULL;
    std::int32_t axis=-1;
    std::uint32_t axisType=0U; // Frozen installed type: 0=linear mm, 1=rotary degrees.
    std::uint32_t owner=0U, ownerGeneration=0U, epoch=0U;
    // Legacy name: pulses per native position unit (mm or degree).
    double pulsesPerMm=0.0, originPulse=0.0;
    double excursionPulse=0.0, boundaryMarginPulse=0.0;
    double followingLimitPulse=0.0, maximumVelocityPps=0.0;
    // TUNE10: maximumVelocityPps bounds the guarded servo output, including
    // feedback correction. The trajectory/encoder guard keeps its own lower
    // ceiling. Zero preserves the earlier one-ceiling replay contract.
    double maximumReferenceVelocityPps=0.0;
    // 0 preserves compatibility with old replay callers (no cruise evidence).
    // The requested test speed is distinct from the wider RT velocity guard.
    double cruiseVelocityPps=0.0;
    double settleWindowPulse=0.0, settleVelocityPps=0.0;
    std::uint64_t maximumTicks=0ULL;
    // CSV storage cadence only. Raw guards, settling and latest publication
    // always run at 250 us. Stride one retains the identification data grid.
    std::uint32_t sampleStride=1U;
    double approachStartPulse=0.0;
    // Q1: originPulse is endpoint A; excursionPulse is the complete A->B
    // stroke. Command endpoints and servo tracking bounds are distinct.
    bool pointToPoint=false;
    // One continuously guarded capture covers approach and test. The approach
    // envelope includes approachStartPulse; measurement uses the A/B envelope.
    bool approachEnabled=false;
    std::uint64_t measurementSettleHoldUs=0ULL;
    CaptureGains motionGains{}, idleGains{};
};
double ReferenceVelocityLimitPps(const CaptureRequest& request) noexcept;
// All accepted raw measurement cycles, independent of the stored CSV stride.
// Terminal acquire publishes these fields to the single NC lifecycle owner.
struct CaptureTrackingMoments {
    std::uint64_t sampleCount=0ULL, observedUs=0ULL;
    // One qualification per continuous signed Motion-bank interval. Ordinary
    // A/B legs have one such interval; NC segment validation remains separate.
    std::uint32_t qualifiedLegCount=0U;
    double peakAbsoluteErrorPulse=0.0, meanErrorPulse=0.0, meanSquaredErrorPulse=0.0;
};
struct CaptureStatistics {
    std::uint64_t session=0ULL, firstCycle=0ULL, lastCycle=0ULL;
    std::uint64_t firstTimestampUs=0ULL, lastTimestampUs=0ULL;
    std::uint64_t sampleCount=0ULL, movingSampleCount=0ULL;
    bool valid=false, complete=false, anySaturated=false;
    double peakFollowingErrorPulse=0.0, meanSquaredFollowingErrorPulse=0.0;
    double meanSquaredMovingFollowingErrorPulse=0.0, peakReferencePps=0.0;
    std::uint32_t cruiseQualificationVersion=0U;
    CaptureTrackingMoments cruisePositive{}, cruiseNegative{}, nonCruiseMoving{};
};
// TUNE17 detached observation of accepted formal-measurement samples only.
// All fields are 64-bit words; publication never reads mutable RT structs.
struct CaptureLiveTelemetry {
    std::uint64_t session=0ULL, cycle=0ULL, timestampUs=0ULL, firstTimestampUs=0ULL;
    std::uint64_t sampleCount=0ULL, movingSampleCount=0ULL, flags=0ULL, observedEndpointCount=0ULL;
    double currentFollowingErrorPulse=0.0, peakFollowingErrorPulse=0.0;
    double meanSquaredMovingFollowingErrorPulse=0.0, maximumOvershootPulse=0.0, cruisePeakPulse=0.0;
};
static_assert(sizeof(CaptureLiveTelemetry)==13U*sizeof(std::uint64_t),"TUNE17 atomic telemetry words");
struct CaptureMeasurementBoundary {
    std::uint64_t session=0ULL;
    std::size_t sampleOffset=0U;
    std::uint64_t cycle=0ULL, timestampUs=0ULL;
    std::uint64_t approachDurationUs=0ULL;
};
// Values are logical pulse / pulse-second. outputLogicalPps is the local
// post-guard PDO IMAGE, not proof that a packet or drive accepted the value.
struct CaptureSample {
    std::uint64_t session=0ULL, cycle=0ULL, timestampUs=0ULL;
    double commandPulse=0.0, actualPulse=0.0, referencePps=0.0;
    double actualVelocityPps=0.0, proposedLogicalPps=0.0, outputLogicalPps=0.0;
    double pTerm=0.0, iTerm=0.0, feedforwardPps=0.0;
    double kp=0.0, ki=0.0, kd=0.0, kvff=0.0, integral=0.0;
    std::uint32_t motionState=0U, gainBank=0U;
    std::uint32_t valid=0U, saturated=0U;
    std::uint32_t settledCycles=0U, reserved=0U;
    std::uint64_t settledDurationUs=0ULL;
    // Continuous fixed-command IDLE samples inside the position window.
    // Separate from the drift-qualified settling hold: observation of bounded
    // holding dither is data, not proof of zero physical motion.
    std::uint64_t idleWindowDurationUs=0ULL;
};
struct CaptureCycle {
    std::uint64_t cycle=0ULL, timestampUs=0ULL;
    std::uint32_t owner=0U, ownerGeneration=0U, epoch=0U;
    bool sourceValid=false, clockValid=false, safetyGood=false, layoutGood=false;
};
enum class CaptureFaultStage : std::uint32_t {
    None=0, Begin=1, Stage=2, AllowOutput=3, End=4, External=5
};
// Diagnostic only. Recorded raw values and the explicit CSV columns stay
// unchanged. CaptureSample is internal, not the HMI shared-memory ABI.
// A signed position step is absent on the first captured sample.
struct CaptureVelocityEvidence {
    double estimatedPps=0.0, limitPps=0.0;
    double positionStepPulse=0.0, rawStepPulse=0.0, stepLimitPulse=0.0;
    std::uint64_t elapsedUs=0ULL, estimationUs=0ULL;
    std::uint32_t intervals=0U;
    bool present=false, windowReady=false, hasPositionStep=false;
};
// First failure only; diagnostic data grants no motion/stop authority. A
// rejected Stage sample keeps its supplied fields (which need not carry a
// session/cycle stamp); use session and cycle below to date that evidence.
struct CaptureFaultSnapshot {
    std::uint64_t session=0ULL, lastGoodCycle=0ULL, lastGoodTimestampUs=0ULL, sampleCount=0ULL;
    CaptureReason reason=CaptureReason::None;
    CaptureFaultStage stage=CaptureFaultStage::None;
    CaptureCycle cycle{};
    CaptureSample sample{};
    CaptureVelocityEvidence velocity{};
    // Stage: supplied pulsesPerMm/inputValid. AllowOutput: supplied velocity/
    // finite check. End: supplied image velocity/imageValid. Absent otherwise.
    double observedValue=0.0;
    std::int32_t observedAxis=-1;
    bool hasCycle=false, hasSample=false, cycleOpen=false, sampleStaged=false;
    bool hasObservedValue=false, observedValid=false;
    bool approachActive=false;
};
class CaptureRecorder {
public:
    CaptureRecorder() noexcept = default;
    // Single NC/control owner. Allocate once outside RT; zero capacity is
    // never presented as successful capture. Terminal buffers remain valid
    // until the next successful Start, which only that owner can request.
    bool Start(const CaptureRequest& request) noexcept;
    void RequestFinish() noexcept;
    bool RequestMeasurement(std::uint64_t session) noexcept;
    bool TryReadMeasurementBoundary(std::uint64_t session,CaptureMeasurementBoundary& boundary) const noexcept;
    void Cancel(CaptureReason reason=CaptureReason::Cancelled) noexcept;
    CaptureState State() const noexcept { return static_cast<CaptureState>(m_state.load(std::memory_order_acquire)); }
    CaptureReason Reason() const noexcept { return static_cast<CaptureReason>(m_reason.load(std::memory_order_acquire)); }
    bool Active() const noexcept { const auto s=State();return s==CaptureState::Armed||s==CaptureState::Running; }
    bool Terminal() const noexcept { const auto s=State();return s==CaptureState::Complete||s==CaptureState::Fault; }
    std::size_t SampleCount() const noexcept { return m_publishedCount.load(std::memory_order_acquire); }
    bool TryReadLatest(CaptureSample& sample) const noexcept;
    // NC lifecycle owner only, before its next Start. Fault acquire publishes
    // an immutable snapshot; absent fields must not be treated as evidence.
    bool TryReadFault(CaptureFaultSnapshot& snapshot) const noexcept;
    bool TryReadStatistics(CaptureStatistics& statistics) const noexcept;
    bool TryReadLiveTelemetry(CaptureLiveTelemetry& telemetry) const noexcept;
    // NC owner, before submit; describes observation only and never grants motion.
    void SetTelemetryLeg(std::uint64_t session,std::uint64_t legIndex,
        std::uint64_t firstCycle,double startPulse,double targetPulse) noexcept;
    const CaptureSample* Samples() const noexcept { return Terminal()?m_samples.get():nullptr; }
    const CaptureRequest* CompletedRequest() const noexcept { return Terminal()?&m_request:nullptr; }
    // RT only: frozen request is available after Active acquire.
    const CaptureRequest* RuntimeRequest() const noexcept { return Active()?&m_request:nullptr; }
    bool RuntimeBegin(const CaptureCycle& cycle) noexcept;
    bool RuntimeStage(CaptureSample sample, double pulsesPerMm, bool inputValid) noexcept;
    bool RuntimeAllowOutput(int axis, double logicalVelocityPps) noexcept;
    bool RuntimeEnd(double imageLogicalVelocityPps, bool imageValid) noexcept;
    bool RuntimeFail(CaptureReason reason) noexcept;
    bool RuntimeNeedsStop() const noexcept { return m_runtimeStopPending.load(std::memory_order_acquire); }
    void RuntimeAcknowledgeStop() noexcept { m_runtimeStopPending.store(false,std::memory_order_release); }
private:
    bool RuntimeFailAt(CaptureReason reason, CaptureFaultStage stage,
        const CaptureCycle* cycle, const CaptureSample* sample,
        bool hasObservedValue=false, double observedValue=0.0,
        bool observedValid=false, int observedAxis=-1,
        const CaptureVelocityEvidence* velocity=nullptr) noexcept;
    void PublishLatest(const CaptureSample& sample) noexcept;
    void PublishLiveTelemetry(bool terminal,bool complete) noexcept;
    void ObserveTelemetryLeg() noexcept;
    static constexpr std::size_t LiveWords=sizeof(CaptureLiveTelemetry)/sizeof(std::uint64_t);
    std::atomic<std::uint64_t> m_liveSequence{0ULL},m_liveWords[LiveWords]{};
    // session, one-based leg, first eligible cycle, start pulse, target pulse.
    std::atomic<std::uint64_t> m_legSequence{0ULL},m_legWords[5]{};
    std::uint64_t m_legPublishedIndex=0ULL; // NC owner only
    std::uint64_t m_legConsumedSequence=0ULL,m_liveLegIndex=0ULL,m_liveLegFirstCycle=0ULL;
    std::uint64_t m_liveEndpointCount=0ULL,m_livePublishedCount=0ULL;
    double m_liveTargetPulse=0.0,m_liveStartPulse=0.0,m_liveOvershootPulse=0.0,m_liveCurrentErrorPulse=0.0;
    bool m_liveLegEndpointSeen=false,m_liveOvershootCoverage=true;
    static constexpr std::size_t LatestWords=sizeof(CaptureSample)/sizeof(std::uint64_t);
    // Raw history is separate from CSV storage. Eighty 250 us intervals
    // remain nominal 20 ms at every sampleStride, with no gap in RT guards.
    static constexpr std::size_t SettleDriftIntervals=80U;
    static constexpr std::size_t RawHistoryCapacity=SettleDriftIntervals+1U;
    std::atomic<std::uint32_t> m_state{0U}, m_reason{0U}, m_cancelReason{0U};
    std::atomic<bool> m_finish{false};
    std::atomic<std::size_t> m_publishedCount{0U};
    std::atomic<std::uint64_t> m_latestSequence{0ULL};
    std::atomic<std::uint64_t> m_latest[LatestWords]{};
    CaptureRequest m_request{};
    std::unique_ptr<CaptureSample[]> m_samples;
    CaptureSample m_pending{};
    CaptureCycle m_cycle{};
    CaptureFaultSnapshot m_fault{};
    CaptureStatistics m_statistics{};
    CaptureSample m_rawHistory[RawHistoryCapacity]{};
    std::uint64_t m_rawCount=0ULL, m_storageAnchorRawCount=0ULL;
    CaptureMeasurementBoundary m_measurementBoundary{};
    std::atomic<std::uint64_t> m_measurementRequestSession{0ULL};
    std::atomic<bool> m_measurementBoundaryPublished{false};
    bool m_measurementActive=true;
    std::size_t m_count=0U;
    std::uint64_t m_firstCycle=0ULL, m_firstUs=0ULL, m_lastCycle=0ULL, m_lastUs=0ULL;
    std::uint64_t m_settledSinceUs=0ULL, m_idleWindowSinceUs=0ULL;
    std::uint32_t m_settledCycles=0U;
    std::size_t m_settleEligibleSamples=0U;
    double m_settleMinimumPulse=0.0, m_settleMaximumPulse=0.0;
    double m_previousCommand=0.0;
    int m_trackingDirection=0;
    std::uint64_t m_cruiseStableSinceUs=0ULL, m_cruiseObservedSinceUs=0ULL;
    double m_cruiseReferenceMin=0.0, m_cruiseReferenceMax=0.0;
    bool m_cruiseLegQualified=false;
    bool m_cycleOpen=false, m_staged=false;
    std::atomic<bool> m_runtimeStopPending{false};
};
CaptureRecorder& Capture() noexcept;
const char* CaptureReasonName(CaptureReason reason) noexcept;
const char* CaptureFaultStageName(CaptureFaultStage stage) noexcept;
}
