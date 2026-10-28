#pragma once
#include "PositionAutoTuneAnalysis.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace PositionAutoTune {
// Relative comparisons describe this recorded P1/P2 combination only. They
// are separate from absolute limit verification and are not an optimum or a
// claim of statistical significance across repeated tests.
enum class PerformanceComparison : std::uint32_t {
    Unavailable=0U, Improved=1U, Equivalent=2U, Regressed=3U, Mixed=4U
};
enum class AdoptionAdvice : std::uint32_t {
    Unavailable=0U, RecommendCandidate=1U, KeepOriginal=2U, ReviewTradeoff=3U
};
struct JobRequest {
    Config config{};
    CaptureRequest capture{};
    CaptureStatistics statistics{};
    unsigned mode=0U;
    bool measurementComplete=false;
    // Frozen approach observations only; they never authorize motion or
    // turn an incomplete capture into a valid measurement.
    bool autoApproachUsed=false;
    double approachStartNative=0.0, approachTargetNative=0.0, approachSeconds=0.0;
    std::uint64_t measurementStartCycle=0ULL;
    std::string failureReason;
    std::uint64_t session=0ULL, configIdentityHash=0ULL;
    // Frozen at P1/P2 NC admission; no live-result ABI change.
    std::uint64_t cruiseBaselineRevision=0ULL;
    std::vector<AnalysisSegment> segments;
    std::string outputDir;
};
struct JobResult {
    AnalysisResult analysis{}, baselineMetrics{};
    bool baselineSaved=false, rawSaved=false, summarySaved=false;
    bool baselineAvailable=false, baselineMatched=false, candidateMatched=false;
    // Local worker/report diagnostics only; no change to the live result ABI.
    // Disabled ranges retain the V3 origin-independent baseline policy.
    bool baselineOriginChecked=false, baselineOriginMatched=false;
    double baselineOriginNative=0.0, currentOriginNative=0.0, baselineOriginTolerance=0.0;
    bool comparisonValid=false, verified=false, cancelled=false;
    bool idleCandidateMatched=false, motionVerified=false, idleVerified=false;
    bool motionContextCompatible=false, idleContextCompatible=false;
    PerformanceComparison motionComparison=PerformanceComparison::Unavailable;
    PerformanceComparison idleComparison=PerformanceComparison::Unavailable;
    AdoptionAdvice adoptionAdvice=AdoptionAdvice::Unavailable;
    double comparisonPositionTolerance=0.0, comparisonTimeToleranceSeconds=0.0;
    // TUNE16 independent measured cruise comparison, never model validation.
    bool cruiseBaselineSaved=false, cruiseBaselineAvailable=false, cruiseBaselineMatched=false;
    bool cruiseComparisonAvailable=false, cruiseP2Verified=false;
    std::uint64_t cruiseBaselineSession=0ULL;
    double cruiseComparisonTolerance=0.0;
    CruiseTrackingMetrics cruiseBeforePositive{}, cruiseBeforeNegative{};
    CaptureGains cruiseBeforeMotionGains{}, cruiseBeforeIdleGains{};
    PerformanceComparison cruisePositiveComparison=PerformanceComparison::Unavailable;
    PerformanceComparison cruiseNegativeComparison=PerformanceComparison::Unavailable;
    PerformanceComparison cruiseOverallComparison=PerformanceComparison::Unavailable;
    std::string cruiseComparisonReason="CURRENT_MEASUREMENT_INVALID";
    std::string detail;
};
// NC admission invalidates prior P1 eligibility even if capture never starts.
// These hooks only touch atomics; they never perform disk I/O.
void InvalidateCruiseBaseline(int axis) noexcept;
std::uint64_t CruiseBaselineRevision(int axis) noexcept;
// NC owner only. Input must be a terminal Capture buffer. The owner MUST NOT
// call Capture.Start until JobBusy() is false and PollJob has consumed the job.
// All analysis / formatting / disk IO runs on a separate low-priority worker.
bool StartJob(const JobRequest&,const CaptureSample*,std::size_t) noexcept;
bool PollJob(std::uint64_t session,std::shared_ptr<const JobResult>&) noexcept;
bool JobBusy() noexcept;
void CancelJob(std::uint64_t session) noexcept;
// System shutdown only, outside cyclic tasks; waits for the worker to exit.
void ShutdownJobs() noexcept;
}
