#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include "PositionAutoTuneConfig.h"
#include "PositionAutoTuneCapture.h"

// TUNE01 non-RT analysis. No live gain application, file access or motion.
namespace PositionAutoTune {
struct AnalysisSegment {
    std::uint64_t firstCycle=0ULL, lastCycle=0ULL, commandCompleteCycle=0ULL;
    double startPulse=0.0, targetPulse=0.0;
};
enum class AnalysisReason : std::uint32_t {
    None=0, Configuration=1, Incomplete=2, InvalidSample=3, Discontinuity=4,
    Saturated=5, Segment=6, Quality=7, Excitation=8, Model=9,
    Validation=10, CandidateBounds=11, UnsupportedGains=12, Allocation=13,
    LongStrokeMetricsOnly=14
};
// Saved-report diagnostics only. Append new explanations without renumbering
// these values or the live AnalysisReason wire codes.
enum class AnalysisDetail : std::uint32_t {
    None=0, InsufficientSteadyPlateau=1, InvalidSteadyGain=2,
    InconsistentSteadyGain=3, SteadyGainOutsideModelRange=4,
    FeedforwardOutsideBounds=5, NoValidDynamicModel=6,
    TrainingTimeFitBelowMinimum=7, HeldOutTimeFitBelowMinimum=8,
    ModelTimeConstantUnresolved=9, ModelFeedforwardDisagreement=10,
    KpOutsideBounds=11, TrainingFrequencyUnsupported=12,
    HeldOutFrequencyUnsupported=13, FrequencyRepeatMismatch=14,
    FineClockUnsupported=15, ResolutionPhaseUnsupported=16, ResolutionMismatch=17
};
enum class IdleAnalysisReason : std::uint32_t {
    None=0, Disabled=1, Incomplete=2, InvalidSample=3, Saturated=4,
    Quality=5, Model=6, CandidateBounds=7, UnsupportedGains=8, Configuration=9
};
struct AnalysisResult {
    bool measurementsValid=false, qualityPassed=false, modelValid=false;
    bool kpValid=false, kvffValid=false, candidateValid=false;
    AnalysisReason reason=AnalysisReason::Incomplete;
    AnalysisDetail analysisDetail=AnalysisDetail::None;
    // Counts of accepted 2 ms plateau points: train+/train-/held+/held-.
    std::array<std::uint32_t,4> steadyPlateauCounts{};
    // Peak recorded reference velocity (not differentiated encoder speed).
    // Valid same-session samples are useful here even in an aborted capture;
    // this diagnostic does not change its measurement-completeness verdict.
    double measuredPeakSpeedNativePerMin=0.0;
    // TUNE06: bounded training-only 2 ms -> 1 ms resolution attempt. A true
    // fallback flag also records an attempt rejected by a later fine-grid
    // guard; these diagnostics never grant candidate/application permission.
    double analysisIntervalSeconds=0.002;
    bool resolutionFallbackUsed=false;
    std::uint32_t resolutionPhaseModels=0U;
    // Maximum difference from frozen fine phase zero, over the original
    // coarse model and the other three fine training models. -1=unavailable.
    double resolutionGainMismatch=-1.0, resolutionResponseMismatch=-1.0;
    // Training-selected Kp before held-out validation; zero=unavailable.
    double selectedTrainingKp=0.0;
    CaptureGains measured{}, suggested{};
    std::uint64_t sampleCount=0ULL;
    std::uint32_t segmentCount=0U;
    double durationSeconds=0.0;
    // Legacy metric suffixes: native mm for linear or degrees for rotary.
    double peakErrorMm=0.0, rmsMovingErrorMm=0.0, rmsIdleErrorMm=0.0;
    double maxOvershootMm=0.0, maxSettlingSeconds=0.0, maxSettledJitterMm=0.0;
    double plantGain=0.0, tauSeconds=0.0, delaySeconds=0.0;
    double trainingFit=0.0, validationFit=0.0, excitationHz=0.0;
    // Non-RT diagnostics. A negative ratio/error means it was not available.
    // Select bandwidth on training legs only, then validate once on held-out
    // legs. A reduction is a conservative candidate, not live verification.
    double requestedExcitationHz=0.0;
    std::uint32_t bandwidthReductions=0U;
    double trainingExcitationRatio=-1.0, validationExcitationRatio=-1.0;
    double trainingFrequencyError=-1.0, validationFrequencyError=-1.0;
    double frequencyRepeatError=-1.0;
    bool idleCandidateValid=false, idleMeasurementsValid=false, idleQualityPassed=false;
    IdleAnalysisReason idleReason=IdleAnalysisReason::Incomplete;
    CaptureGains measuredIdle{}, suggestedIdle{};
    // Worst leg's final observed IDLE window. Bias is the largest absolute
    // mean error; ObservedSeconds is the shortest continuous IDLE tail.
    double idleTailRmsMm=0.0, idlePeakBiasMm=0.0, idleJitterMm=0.0, idleObservedSeconds=0.0;
    // Verification horizon for the gains actually used, not a newly suggested
    // pair: at least two Kp/Ki periods when Ki>0, otherwise the configured tail.
    double idleRequiredObservationSeconds=0.0;
};

// TUNE07: descriptive assessment of the recorded endpoint IDLE windows only.
// This does not grant candidate permission or prove table/load accuracy.
struct PositionTargetAssessment {
    bool available=false, met=false;
    double errorUpperBoundMm=0.0;
};
PositionTargetAssessment EvaluatePositionTarget(const AxisConfig& axis,
    const AnalysisResult& result, bool measurementComplete) noexcept;

// TUNE09: independently report the recorded constant-speed LAG goal. This
// assessment never changes the safety following limit, settling requirement,
// model estimation or suggested gains. Partial diagnostics survive an abort;
// a target pass requires a complete test and the expected qualified moving
// intervals in both directions. NC segment/model validation stays separate.
struct CruiseTrackingMetrics {
    std::uint64_t sampleCount=0ULL;
    std::uint32_t qualifiedLegCount=0U;
    double observedSeconds=0.0, peakAbsoluteMm=0.0, rmsMm=0.0, meanMm=0.0;
};
struct CruiseTrackingAssessment {
    bool statisticsValid=false, complete=false, targetSpecified=false;
    bool available=false, targetMet=false;
    double targetMm=0.0, requestedSpeedNativePerMin=0.0, peakAbsoluteMm=0.0;
    std::uint32_t expectedLegsPerDirection=0U;
    CruiseTrackingMetrics positive{}, negative{}, nonCruise{};
};
CruiseTrackingAssessment EvaluateCruiseTracking(const AxisConfig& axis,
    const CaptureRequest& request, const CaptureStatistics& statistics,
    bool measurementComplete) noexcept;

// All segment cycle ranges are inclusive and must be ordered/non-overlapping.
// commandCompleteCycle is a confirmed endpoint with zero reference velocity
// and commanded position at target.  Analysis walks backwards to the first
// contiguous endpoint sample, removing control-task polling quantization.
// This command endpoint is not an actual-stop proof.
// Each cycle supplies four legs: origin -> +E -> origin -> -E -> origin.
// Q1 pointToPoint instead supplies A -> B -> A -> B -> A, with origin=A and
// excursion=B-A. Endpoint order is strict; rotary coordinates stay unwrapped.
// Both positive and negative held-out legs must pass model validation.
// Suggested Ki=0 is a retained policy, NOT an identified optimal integral gain.
// candidateValid means an UNVERIFIED recommendation; it never means validated
// live gains.  Independent P2 testing and identity matching belong to the caller.
AnalysisResult Analyze(const Config& config, const CaptureRequest& request,
    const CaptureSample* samples, std::size_t sampleCount,
    const AnalysisSegment* segments, std::size_t segmentCount) noexcept;
// TUNE08: stride > 1 stores a sparse display trace while the RT recorder
// retains full-rate tracking statistics. This validates those statistics and
// raw-cycle segment markers only; it never identifies a plant, evaluates an
// IDLE tail, grants candidate permission or establishes a quality/P2 pass.
// rmsMovingErrorMm here uses RT motion-bank samples with |referencePps| > 1,
// not the full-rate analyzer's command-complete segment partition.
AnalysisResult AnalyzeDecimated(const Config& config, const CaptureRequest& request,
    const CaptureStatistics& statistics, const CaptureSample* samples,
    std::size_t sampleCount, const AnalysisSegment* segments,
    std::size_t segmentCount) noexcept;
const char* AnalysisReasonName(AnalysisReason reason) noexcept;
const char* AnalysisDetailName(AnalysisDetail detail) noexcept;
const char* IdleAnalysisReasonName(IdleAnalysisReason reason) noexcept;
}
