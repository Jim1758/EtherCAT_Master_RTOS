#pragma once

#include "EDMCoordinatedGapDecision.h"
#include "EDMSixAxisGapTuning.h"
#include "EDMSixAxisGapNoise.h"
#include "EDMSixAxisGapBurst.h"
#include "EDMSixAxisGapContinuous.h"
#include "EDMSixAxisGapEndurance.h"
#include "EDMSixAxisGapPairedPath.h"
#include "EDMSixAxisGapFlushPath.h"
#include <array>
#include <algorithm>
#include <type_traits>

// EDM96: one bounded scalar on the admitted six-axis pulse metric. Acquisition
// advances exactly once before the planner; held reads retain producer proof.
// This value type does not own axes, filters, output or discharge permission.
namespace EDM96
{
constexpr std::uint64_t SourcePeriodUs = 1000ULL;
constexpr std::uint64_t MaximumSourceAgeUs = 1500ULL;
constexpr std::uint64_t DirectionDwellUs = 20000ULL;
constexpr std::uint64_t ScriptPhaseUs = 600000ULL;
constexpr std::uint64_t SegmentDeadlineUs = 10000000ULL;
constexpr std::uint64_t SegmentDeadlineTicks = 40000ULL;
constexpr unsigned MaximumSteps = 4U;
constexpr double SampleSeconds = 0.00025;

enum class Reason : std::uint8_t
{ None, Identity, Clock, Source, Decision, Lifecycle, Geometry, Dynamics, Bounds, Endpoint, Deadline, Output, External, Noise, Burst };
inline const char* ReasonName(Reason reason) noexcept
{
    switch (reason) {
    case Reason::None: return "NONE"; case Reason::Identity: return "IDENTITY";
    case Reason::Clock: return "CLOCK"; case Reason::Source: return "SOURCE";
    case Reason::Decision: return "DECISION"; case Reason::Lifecycle: return "LIFECYCLE";
    case Reason::Geometry: return "GEOMETRY"; case Reason::Dynamics: return "DYNAMICS";
    case Reason::Bounds: return "BOUNDS"; case Reason::Endpoint: return "ENDPOINT";
    case Reason::Deadline: return "DEADLINE"; case Reason::Output: return "OUTPUT";
    case Reason::External: return "EXTERNAL";
    case Reason::Noise: return "NOISE";
    case Reason::Burst: return "BURST";
    } return "UNKNOWN";
}
struct Feedback
{
    std::uint64_t session = 0ULL, epoch = 0ULL;
    std::uint32_t ownerGeneration = 0U;
    unsigned expectedStep = 0U;
    std::uint64_t tick = 0ULL, nowUs = 0ULL;
    std::uint64_t sourceSequence = 0ULL, sourceCapturedUs = 0ULL;
    std::uint64_t freshFrames = 0ULL, heldFrames = 0ULL, maximumAgeUs = 0ULL;
    std::uint64_t acceptedSegments = 0ULL, completedSegments = 0ULL;
    std::uint64_t reverseSegments = 0ULL, zeroCrossings = 0ULL;
    std::uint64_t reverseTicks = 0ULL, returnForwardTicks = 0ULL;
    double voltageV = 0.0, q = 0.0, v = 0.0, length = 0.0;
    unsigned phase = 0U;
    int direction = 0;
    bool active = false, faulted = false, segmentActive = false;
    bool segmentCompleted = false, gateReady = false;
    Reason reason = Reason::None;
    EDM88::Reason sourceReason = EDM88::Reason::None;
    EDM89::Reason decisionReason = EDM89::Reason::None;
    bool tunedMode = false;
    EDM97::Tuning tuning{};
    std::uint64_t configHash = 0ULL, saturationFrames = 0ULL;
    double errorV = 0.0, rawSpeedMmMin = 0.0, limitedSpeedMmMin = 0.0, targetRatio = 0.0;
    bool noiseMode = false;
    std::uint64_t noiseFreshFrames = 0ULL, quietZeroTicks = 0ULL;
    std::uint64_t quietWindows = 0ULL, quietSignChanges = 0ULL;
    bool burstMode = false, continuousMode = false, enduranceMode = false, pairedMode = false;
    bool flushMode = false;
    EDM105::Phase flushPhase = EDM105::Phase::None;
    std::uint64_t pulseLowFreshFrames = 0ULL, suppressedPulseWindows = 0ULL;
    std::uint64_t qualifiedRetreatWindows = 0ULL, reverseEpisodes = 0ULL;
    std::uint64_t flushStarted = 0ULL, flushCompleted = 0ULL, flushZeroTicks = 0ULL;
    double flushAnchor = 0.0, flushRetreat = 0.0, flushReturned = 0.0;
};
static_assert(std::is_trivially_copyable<Feedback>::value, "EDM96 feedback is copied by value");
static_assert(sizeof(Feedback) <= 384U, "EDM96/97/98/99/100/102/104/105 feedback remains bounded");

inline bool ValidIncrement(const std::array<double, 6U>& increment, double feed) noexcept
{
    if (feed != 0.12) return false;
    for (unsigned i = 0U; i < 6U; ++i) {
        const double magnitude = std::fabs(increment[i]);
        if (!std::isfinite(increment[i]) || magnitude < (i < 3U ? 0.001 : 0.02) ||
            magnitude > (i < 3U ? 0.002 : 0.04)) return false;
    }
    return true;
}

class Segment
{
public:
    const Feedback& Current() const noexcept { return feedback_; }
    const EDM97::Tuning& FrozenTuning() const noexcept { return tuning_; }
    bool IsTuned() const noexcept { return feedback_.tunedMode; }
    bool IsNoiseTuned() const noexcept { return feedback_.noiseMode; }
    bool IsBurstTuned() const noexcept { return feedback_.burstMode; }
    bool IsContinuousBurstTuned() const noexcept { return feedback_.continuousMode; }
    bool IsEnduranceBurstTuned() const noexcept { return feedback_.enduranceMode; }
    bool IsPairedBurstTuned() const noexcept { return feedback_.pairedMode; }
    bool IsFlushBurstTuned() const noexcept { return feedback_.flushMode; }
    bool Begin(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, false, false, false, false, false, false, false, EDM97::Tuning{}); }
    bool BeginTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, false, false, false, false, false, false, tuning); }
    bool BeginNoiseTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, true, false, false, false, false, false, tuning); }
    bool BeginBurstTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, false, true, false, false, false, false, tuning); }
    bool BeginContinuousBurstTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, false, true, true, false, false, false, tuning); }
    bool BeginEnduranceBurstTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, false, true, true, true, false, false, tuning); }
    bool BeginPairedBurstTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, false, true, true, true, true, false, tuning); }
    bool BeginFlushBurstTuned(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, const EDM97::Tuning& tuning) noexcept
    { return BeginMode(session, epoch, owner, tick, nowUs, true, false, true, true, true, true, true, tuning); }
private:
    unsigned PerModeStepLimit() const noexcept
    { return feedback_.flushMode ? EDM105::MaximumSteps :
        feedback_.pairedMode ? EDM104::MaximumSteps :
        feedback_.enduranceMode ? EDM102::MaximumSteps :
        feedback_.continuousMode ? EDM100::MaximumSteps : MaximumSteps; }
    bool BeginMode(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs, bool tuned, bool noise, bool burst, bool continuous, bool endurance, bool paired, bool flush,
        EDM97::Tuning tuning) noexcept
    {
        if (feedback_.active || !session || session <= latestSession_ || !epoch ||
            epoch > (std::numeric_limits<std::uint32_t>::max)() || !owner || !tick || !nowUs)
            return Fail(Reason::Identity);
        if (tuned && !EDM97::ValidTuning(tuning)) return Fail(Reason::Dynamics);
        if (noise && (!tuned || !EDM98::ValidTuning(tuning))) return Fail(Reason::Noise);
        if (burst && (!tuned || noise || !EDM99::ValidTuning(tuning))) return Fail(Reason::Burst);
        if (continuous && (!tuned || noise || !burst || !EDM100::ValidTuning(tuning)))
            return Fail(Reason::Burst);
        if (endurance && (!continuous || !tuned || noise || !burst || !EDM102::ValidTuning(tuning)))
            return Fail(Reason::Burst);
        if (paired && (!endurance || !continuous || !tuned || noise || !burst || !EDM104::ValidTuning(tuning)))
            return Fail(Reason::Burst);
        if (flush && (!paired || !endurance || !continuous || !tuned || noise || !burst || !EDM105::ValidTuning(tuning)))
            return Fail(Reason::Burst);
        latestSession_ = session; feedback_ = Feedback{}; frame_ = EDM88::Frame{};
        tuning_ = tuning; feedback_.tunedMode = tuned; feedback_.tuning = tuning_;
        feedback_.noiseMode = noise;
        feedback_.burstMode = burst;
        feedback_.continuousMode = continuous;
        feedback_.enduranceMode = endurance;
        feedback_.pairedMode = paired;
        feedback_.flushMode = flush;
        flushAnchor_ = flushTarget_ = 0.0; flushDwellTick_ = flushDwellUs_ = 0ULL;
        feedback_.configHash = tuned ? EDM97::ConfigHash(tuning_) : 0ULL;
        feedback_.session = session; feedback_.epoch = epoch; feedback_.ownerGeneration = owner;
        armUs_ = nowUs; lastTick_ = tick - 1ULL; lastUs_ = nowUs - 1ULL;
        lastStepTick_ = segmentTick_ = segmentUs_ = 0ULL; forward_ = {};
        progress_ = speed_ = maximumSpeed_ = acceleration_ = deceleration_ = 0.0;
        lastNonzeroDirection_ = 0; zeroSinceDirection_ = false;
        reverseObserved_ = returnedForward_ = false; segmentCrossings_ = 0U;
        ResetQuietWindow();
        ResetBurstWindows();
        EDMGapServo::Profile profile{};
        if (tuned) profile = EDM97::MakeProfile(tuning_);
        profile.shortMachiningV = 35.0; profile.shortHysteresisV = 5.0;
        profile.shortEnterMs = profile.shortExitMs = 20U;
        if (!source_.Begin(EDM88::SimulatedSource(session)) || !decision_.Begin(profile))
            return Fail(Reason::Source);
        feedback_.active = true;
        return ObserveSource(tick, nowUs, true);
    }
public:
    bool ObserveSource(std::uint64_t tick, std::uint64_t nowUs, bool currentFrame = true) noexcept
    {
        if (!feedback_.active || feedback_.faulted) return false;
        if (!currentFrame || !tick || tick != lastTick_ + 1ULL || nowUs <= lastUs_ || nowUs < armUs_)
            return Fail(Reason::Clock);
        if (!frame_.sequence || (nowUs >= frame_.capturedAtUs &&
            nowUs - frame_.capturedAtUs >= SourcePeriodUs)) {
            if (frame_.sequence == (std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::Clock);
            frame_.identity = source_.FrozenConfig().identity;
            ++frame_.sequence; frame_.capturedAtUs = nowUs; frame_.valid = true;
            const std::uint64_t elapsed = feedback_.segmentActive ? nowUs - segmentUs_ : 0ULL;
            if (feedback_.burstMode)
                frame_.voltageV = feedback_.segmentActive ? EDM99::Voltage(elapsed) : 50.0;
            else if (feedback_.noiseMode)
                frame_.voltageV = feedback_.segmentActive ? EDM98::Voltage(elapsed, frame_.sequence) : 50.0;
            else if (feedback_.tunedMode)
                frame_.voltageV = !feedback_.segmentActive || elapsed < EDM97::ScriptPhaseUs ? 50.0 :
                    elapsed < 2ULL * EDM97::ScriptPhaseUs ? 45.0 :
                    elapsed < 3ULL * EDM97::ScriptPhaseUs ? 20.0 :
                    elapsed < 4ULL * EDM97::ScriptPhaseUs ? 25.0 : 50.0;
            else
                frame_.voltageV = feedback_.segmentActive && elapsed >= ScriptPhaseUs &&
                    elapsed < 2ULL * ScriptPhaseUs ? 20.0 : 50.0;
        }
        return ObserveConversion(tick, nowUs, frame_);
    }

    // Deterministic source-fault hook; the production caller uses ObserveSource.
    bool ObserveConversion(std::uint64_t tick, std::uint64_t nowUs, const EDM88::Frame& conversion) noexcept
    {
        if (!feedback_.active || feedback_.faulted) return false;
        feedback_.gateReady = false;
        if (!tick || tick != lastTick_ + 1ULL || nowUs <= lastUs_ || nowUs < armUs_ ||
            conversion.capturedAtUs < armUs_) return Fail(Reason::Clock);
        lastTick_ = feedback_.tick = tick; lastUs_ = feedback_.nowUs = nowUs;
        if (!source_.Observe(conversion, nowUs)) {
            feedback_.sourceReason = source_.Current().reason; return Fail(Reason::Source);
        }
        if (!decision_.Observe(conversion)) {
            feedback_.decisionReason = decision_.Current().reason; return Fail(Reason::Decision);
        }
        const auto& source = source_.Current();
        const bool freshConversion = conversion.sequence != feedback_.sourceSequence;
        const std::uint64_t previousCapturedUs = feedback_.sourceCapturedUs;
        feedback_.sourceSequence = conversion.sequence; feedback_.sourceCapturedUs = conversion.capturedAtUs;
        feedback_.freshFrames = source.freshFrames; feedback_.heldFrames = source.heldFrames;
        feedback_.maximumAgeUs = source.maximumAgeUs; feedback_.voltageV = conversion.voltageV;
        const std::uint64_t phaseElapsed = conversion.capturedAtUs >= segmentUs_ ?
            conversion.capturedAtUs - segmentUs_ : 0ULL;
        if (feedback_.burstMode)
            feedback_.phase = !feedback_.segmentActive ? 0U : EDM99::Phase(phaseElapsed);
        else if (feedback_.noiseMode) {
            const unsigned phase = !feedback_.segmentActive ? 0U : EDM98::Phase(phaseElapsed);
            if (!ObserveQuietPhase(phase)) return false;
            feedback_.phase = phase;
        } else if (feedback_.tunedMode)
            feedback_.phase = !feedback_.segmentActive ? 0U :
                phaseElapsed < EDM97::ScriptPhaseUs ? 1U :
                phaseElapsed < 2ULL * EDM97::ScriptPhaseUs ? 2U :
                phaseElapsed < 3ULL * EDM97::ScriptPhaseUs ? 3U :
                phaseElapsed < 4ULL * EDM97::ScriptPhaseUs ? 4U : 5U;
        else
            feedback_.phase = !feedback_.segmentActive ? 0U :
                phaseElapsed < ScriptPhaseUs ? 1U : phaseElapsed < 2ULL * ScriptPhaseUs ? 2U : 3U;
        const auto state = decision_.Current().state;
        feedback_.direction = state == EDMGapServo::ShortState::Clear ? 1 :
            state == EDMGapServo::ShortState::Active ? -1 : 0;
        if (feedback_.tunedMode) {
            const auto curve = EDMGapServo::Evaluate(EDM97::MakeProfile(tuning_), conversion.voltageV,
                EDM97::ReferenceVoltageV, 100.0);
            if (!curve.valid) return Fail(Reason::Dynamics);
            feedback_.errorV = curve.errorV; feedback_.rawSpeedMmMin = curve.rawSpeedMmPerMin;
            feedback_.limitedSpeedMmMin = curve.limitedSpeedMmPerMin;
            feedback_.targetRatio = feedback_.direction > 0 && curve.limitedSpeedMmPerMin >= 0.0 ?
                curve.limitedSpeedMmPerMin / EDM97::MaximumFeedMmMin :
                feedback_.direction < 0 && curve.limitedSpeedMmPerMin <= 0.0 ?
                curve.limitedSpeedMmPerMin / EDM97::MaximumFeedMmMin : 0.0;
            if (feedback_.segmentActive && freshConversion && curve.saturated) ++feedback_.saturationFrames;
            if (feedback_.noiseMode && feedback_.segmentActive && EDM98::IsQuietPhase(feedback_.phase)) {
                if (curve.rawSpeedMmPerMin != 0.0 || curve.limitedSpeedMmPerMin != 0.0 ||
                    feedback_.targetRatio != 0.0) return Fail(Reason::Noise);
                if (freshConversion) ++feedback_.noiseFreshFrames;
            }
        }
        if (feedback_.burstMode && feedback_.segmentActive &&
            !ObserveBurstConversion(conversion, freshConversion, previousCapturedUs, phaseElapsed)) return false;
        feedback_.gateReady = !feedback_.segmentActive && feedback_.expectedStep < PerModeStepLimit() &&
            feedback_.acceptedSegments == feedback_.completedSegments && feedback_.direction == 1 &&
            conversion.capturedAtUs - armUs_ >= DirectionDwellUs;
        return true;
    }

    // Length/velocity/dynamics are the caller's admitted scalar pulse metric.
    // The immutable NC vector is in mm/degree solely for shape validation.
    bool BeginSegment(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        unsigned step, std::uint64_t nextEpoch, const std::array<double, 6U>& increment,
        double feed, double length, double maximumVelocity, double acceleration,
        double deceleration, std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (!feedback_.active || feedback_.faulted || feedback_.segmentActive || !feedback_.gateReady ||
            session != feedback_.session || epoch != feedback_.epoch || owner != feedback_.ownerGeneration ||
            step != feedback_.expectedStep || step >= PerModeStepLimit() || tick != feedback_.tick || nowUs != feedback_.nowUs ||
            nextEpoch != (epoch == (std::numeric_limits<std::uint32_t>::max)() ? 1ULL : epoch + 1ULL))
            return Fail(Reason::Identity);
        if (feedback_.flushMode && (feedback_.flushStarted != (step + 1ULL) / 2ULL ||
            feedback_.flushCompleted != (step + 1ULL) / 2ULL)) return Fail(Reason::Lifecycle);
        if (!ValidIncrement(increment, feed)) return Fail(Reason::Geometry);
        if (step == 0U || (feedback_.pairedMode && step % 2U == 0U)) forward_ = increment;
        for (unsigned i = 0U; i < 6U; ++i)
            if (increment[i] != (step % 2U == 0U ? forward_[i] : -forward_[i])) return Fail(Reason::Geometry);
        if (!std::isfinite(length) || !std::isfinite(maximumVelocity) ||
            !std::isfinite(acceleration) || !std::isfinite(deceleration) || length <= 0.0 ||
            maximumVelocity <= 0.0 || acceleration <= 0.0 || deceleration <= 0.0)
            return Fail(Reason::Dynamics);
        maximumSpeed_ = maximumVelocity / length;
        acceleration_ = (std::min)(acceleration / length, maximumSpeed_ / 0.05);
        deceleration_ = (std::min)(deceleration / length, maximumSpeed_ / 0.05);
        if (!std::isfinite(maximumSpeed_) || !std::isfinite(acceleration_) || !std::isfinite(deceleration_) ||
            maximumSpeed_ <= 0.0 || acceleration_ <= 0.0 || deceleration_ <= 0.0 ||
            acceleration_ * SampleSeconds <= 0.0 || deceleration_ * SampleSeconds <= 0.0 ||
            !std::isfinite(StopDistance(maximumSpeed_)))
            return Fail(Reason::Dynamics);
        feedback_.epoch = nextEpoch; ++feedback_.expectedStep; ++feedback_.acceptedSegments;
        feedback_.length = length; feedback_.q = feedback_.v = 0.0;
        feedback_.segmentActive = true; feedback_.segmentCompleted = false; feedback_.gateReady = false;
        feedback_.phase = 1U;
        if (feedback_.flushMode) {
            feedback_.flushPhase = step % 2U == 0U ? EDM105::Phase::AwaitTrigger : EDM105::Phase::None;
            feedback_.flushAnchor = feedback_.flushRetreat = feedback_.flushReturned = 0.0;
            flushAnchor_ = flushTarget_ = 0.0; flushDwellTick_ = flushDwellUs_ = 0ULL;
        }
        progress_ = speed_ = 0.0; segmentTick_ = lastStepTick_ = tick; segmentUs_ = nowUs;
        lastNonzeroDirection_ = 0; zeroSinceDirection_ = true;
        reverseObserved_ = returnedForward_ = false; segmentCrossings_ = 0U;
        ResetQuietWindow();
        ResetBurstWindows();
        return true;
    }

    // The acceptance frame remains zero. First step is the next RT tick and
    // every subsequent step consumes the source proof from that same tick.
    bool Step(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (feedback_.flushMode && feedback_.active && !feedback_.faulted && feedback_.segmentActive) {
            using EDM105::Phase;
            const Phase phase = feedback_.flushPhase;
            const std::uint64_t expected = (feedback_.acceptedSegments + 1ULL) / 2ULL;
            if (!feedback_.acceptedSegments || feedback_.acceptedSegments > EDM105::MaximumSteps ||
                phase > Phase::Complete || feedback_.flushStarted > EDM105::ExpectedFlushCycles)
                return Fail(Reason::Lifecycle);
            if (feedback_.acceptedSegments % 2ULL == 0ULL) {
                if (phase != Phase::None || feedback_.flushStarted != expected ||
                    feedback_.flushCompleted != expected) return Fail(Reason::Lifecycle);
            } else if (phase == Phase::None ||
                feedback_.flushStarted != (phase == Phase::AwaitTrigger ? expected - 1ULL : expected) ||
                feedback_.flushCompleted != (phase == Phase::Complete ? expected : expected - 1ULL))
                return Fail(Reason::Lifecycle);
            if (feedback_.flushPhase == EDM105::Phase::AwaitTrigger &&
                feedback_.phase == 7U && progress_ >= EDM105::TriggerProgress &&
                burstQualifiedMask_ == 3U && burstReverseMask_ == 3U && burstForwardMask_ == 3U &&
                segmentCrossings_ == 4U && reverseObserved_ && returnedForward_) {
                if (feedback_.flushStarted != feedback_.flushCompleted ||
                    feedback_.flushStarted >= EDM105::ExpectedFlushCycles) return Fail(Reason::Lifecycle);
                ++feedback_.flushStarted; feedback_.flushPhase = EDM105::Phase::Brake;
            }
            if (feedback_.flushPhase >= EDM105::Phase::Brake &&
                feedback_.flushPhase <= EDM105::Phase::DwellReturn) return StepFlush(tick, nowUs);
        }
        return StepGap(tick, nowUs);
    }
private:
    bool StepGap(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (!feedback_.active || feedback_.faulted || !feedback_.segmentActive) return false;
        if (tick != feedback_.tick || nowUs != feedback_.nowUs || tick != lastStepTick_ + 1ULL ||
            tick <= segmentTick_ || nowUs <= segmentUs_) return Fail(Reason::Clock);
        lastStepTick_ = tick;
        if (nowUs - segmentUs_ > SegmentDeadlineUs || tick - segmentTick_ > SegmentDeadlineTicks)
            return Fail(Reason::Deadline);
        const double desired = feedback_.tunedMode ? feedback_.targetRatio * maximumSpeed_ :
            feedback_.direction > 0 ? maximumSpeed_ :
            feedback_.direction < 0 ? -0.25 * maximumSpeed_ : 0.0;
        if (feedback_.burstMode && feedback_.phase == 2U &&
            (desired < 0.0 || speed_ < 0.0)) return Fail(Reason::Burst);
        if (feedback_.noiseMode && EDM98::IsQuietPhase(feedback_.phase) && desired != 0.0)
            return Fail(Reason::Noise);
        // A sign-changing request always brakes to a full zero-output frame;
        // only a later frame may accelerate in the opposite direction.
        double next = desired;
        if (speed_ != 0.0 && desired * speed_ < 0.0) next = 0.0;
        const double rate = std::fabs(next) > std::fabs(speed_) && next * speed_ >= 0.0 ? acceleration_ : deceleration_;
        const double change = rate * SampleSeconds;
        next = (std::max)(speed_ - change, (std::min)(speed_ + change, next));
        if (speed_ > 0.0 && next < 0.0) next = 0.0;
        if (speed_ < 0.0 && next > 0.0) next = 0.0;
        if (!FenceEndpoint(next)) return false;
        const double advance = 0.5 * (speed_ + next) * SampleSeconds;
        const double updated = progress_ + advance;
        const double roundoff = Roundoff();
        if (!std::isfinite(updated) || updated < -roundoff || updated > 1.0 + roundoff)
            return Fail(Reason::Bounds);
        const double previousProgress = progress_, previousSpeed = speed_;
        progress_ = (std::max)(0.0, (std::min)(1.0, updated));
        speed_ = next;
        const int sign = speed_ > 0.0 ? 1 : speed_ < 0.0 ? -1 : 0;
        if (feedback_.noiseMode && !StepQuietWindow(tick, nowUs, previousProgress, previousSpeed, sign))
            return false;
        if (!sign) zeroSinceDirection_ = true;
        else {
            if (lastNonzeroDirection_ && lastNonzeroDirection_ != sign) {
                if (!zeroSinceDirection_) return Fail(Reason::Dynamics);
                if (feedback_.burstMode && !ObserveBurstReversal(sign)) return false;
                ++feedback_.zeroCrossings; ++segmentCrossings_;
            }
            lastNonzeroDirection_ = sign; zeroSinceDirection_ = false;
            if (sign < 0) {
                if (!reverseObserved_) { reverseObserved_ = true; ++feedback_.reverseSegments; }
                ++feedback_.reverseTicks;
            } else if (reverseObserved_) { returnedForward_ = true; ++feedback_.returnForwardTicks; }
        }
        // Only floating-point roundoff (<=64 eps of the scalar length) can be
        // collapsed, and only at zero velocity. No physical displacement or
        // nonzero velocity is snapped to the endpoint.
        if (speed_ == 0.0 && 1.0 - progress_ <= roundoff &&
            (!feedback_.flushMode || feedback_.flushPhase != EDM105::Phase::AwaitTrigger)) {
            if (feedback_.flushMode && (feedback_.flushStarted != (feedback_.acceptedSegments + 1ULL) / 2ULL ||
                feedback_.flushCompleted != (feedback_.acceptedSegments + 1ULL) / 2ULL))
                return Fail(Reason::Lifecycle);
            if (feedback_.burstMode && (feedback_.suppressedPulseWindows !=
                EDM99::PulseWindows * feedback_.acceptedSegments ||
                feedback_.qualifiedRetreatWindows != 2ULL * feedback_.acceptedSegments ||
                feedback_.reverseEpisodes != 2ULL * feedback_.acceptedSegments ||
                burstQualifiedMask_ != 3U || burstReverseMask_ != 3U || burstForwardMask_ != 3U))
                return Fail(Reason::Burst);
            if (feedback_.noiseMode && (feedback_.quietWindows != 2ULL * feedback_.acceptedSegments ||
                feedback_.quietSignChanges != 0ULL)) return Fail(Reason::Noise);
            if (!reverseObserved_ || !returnedForward_ ||
                segmentCrossings_ != (feedback_.burstMode ? 4U : 2U) ||
                feedback_.phase != (feedback_.burstMode ? 7U : feedback_.tunedMode ? 5U : 3U))
                return Fail(Reason::Endpoint);
            progress_ = 1.0; feedback_.segmentActive = false; feedback_.segmentCompleted = true;
            ++feedback_.completedSegments;
        }
        feedback_.q = progress_ == 1.0 ? feedback_.length : progress_ * feedback_.length;
        feedback_.v = speed_ * feedback_.length;
        if (!std::isfinite(feedback_.q) || !std::isfinite(feedback_.v)) return Fail(Reason::Bounds);
        return true;
    }
public:
    void Stop() noexcept
    {
        feedback_.active = feedback_.segmentActive = feedback_.segmentCompleted = feedback_.gateReady = false;
        feedback_.v = 0.0; speed_ = 0.0;
        feedback_.flushPhase = EDM105::Phase::None;
    }
    void Fault(Reason reason = Reason::External) noexcept { (void)Fail(reason); }
private:
    Feedback feedback_{}; EDM88::Qualifier source_{}; EDM89::Decision decision_{};
    EDM97::Tuning tuning_{};
    EDM88::Frame frame_{}; std::array<double, 6U> forward_{};
    std::uint64_t latestSession_ = 0ULL, armUs_ = 0ULL, lastTick_ = 0ULL, lastUs_ = 0ULL;
    std::uint64_t segmentTick_ = 0ULL, segmentUs_ = 0ULL, lastStepTick_ = 0ULL;
    double progress_ = 0.0, speed_ = 0.0, maximumSpeed_ = 0.0, acceleration_ = 0.0, deceleration_ = 0.0;
    int lastNonzeroDirection_ = 0;
    unsigned segmentCrossings_ = 0U;
    bool zeroSinceDirection_ = false, reverseObserved_ = false, returnedForward_ = false;
    std::uint64_t quietZeroStartUs_ = 0ULL, quietZeroStartTick_ = 0ULL, quietZeroRun_ = 0ULL;
    double quietStableProgress_ = 0.0;
    unsigned quietPhase_ = 0U;
    int quietLastNonzeroSign_ = 0;
    bool quietProved_ = false;
    unsigned burstPhase_ = 0U, pulseWindow_ = 0U;
    unsigned burstQualifiedMask_ = 0U, burstReverseMask_ = 0U, burstForwardMask_ = 0U;
    std::uint64_t burstEnterBaseline_ = 0ULL, pulseAbortBaseline_ = 0ULL;
    EDM89::Mark pulseEntry_{};
    bool pulseLowSeen_ = false, pulseAbortSeen_ = false;
    double flushAnchor_ = 0.0, flushTarget_ = 0.0;
    std::uint64_t flushDwellTick_ = 0ULL, flushDwellUs_ = 0ULL;
    // All flush positions are commanded scalar coordinates, never encoder proof.
    bool StepFlush(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (tick != feedback_.tick || nowUs != feedback_.nowUs || tick != lastStepTick_ + 1ULL ||
            tick <= segmentTick_ || nowUs <= segmentUs_) return Fail(Reason::Clock);
        lastStepTick_ = tick;
        if (nowUs - segmentUs_ > SegmentDeadlineUs || tick - segmentTick_ > SegmentDeadlineTicks)
            return Fail(Reason::Deadline);
        if (feedback_.phase != 7U || burstQualifiedMask_ != 3U || burstReverseMask_ != 3U ||
            burstForwardMask_ != 3U || segmentCrossings_ != 4U ||
            feedback_.acceptedSegments % 2ULL != 1ULL ||
            feedback_.flushStarted != feedback_.flushCompleted + 1ULL ||
            feedback_.flushStarted != (feedback_.acceptedSegments + 1ULL) / 2ULL)
            return Fail(Reason::Lifecycle);
        using EDM105::Phase;
        const Phase phase = feedback_.flushPhase;
        if (phase == Phase::DwellRetreat || phase == Phase::DwellReturn) {
            if (speed_ != 0.0 || progress_ != flushTarget_ || tick <= flushDwellTick_ ||
                nowUs <= flushDwellUs_) return Fail(Reason::Dynamics);
            ++feedback_.flushZeroTicks;
            if (tick - flushDwellTick_ >= EDM105::CommandDwellTicks &&
                nowUs - flushDwellUs_ >= EDM105::CommandDwellUs) {
                if (phase == Phase::DwellRetreat) {
                    feedback_.flushPhase = Phase::Return; flushTarget_ = flushAnchor_;
                } else {
                    if (progress_ != flushAnchor_ || feedback_.flushReturned != feedback_.flushAnchor)
                        return Fail(Reason::Endpoint);
                    ++feedback_.flushCompleted; feedback_.flushPhase = Phase::Complete;
                    zeroSinceDirection_ = true;
                }
            }
            feedback_.v = 0.0;
            return true;
        }
        const double desired = phase == Phase::Brake ? 0.0 :
            phase == Phase::Retreat ? -EDM105::SpeedRatio * maximumSpeed_ :
            phase == Phase::Return ? EDM105::SpeedRatio * maximumSpeed_ : 0.0;
        if (phase != Phase::Brake && phase != Phase::Retreat && phase != Phase::Return)
            return Fail(Reason::Lifecycle);
        double next = desired;
        if (speed_ != 0.0 && desired * speed_ < 0.0) next = 0.0;
        const double rate = std::fabs(next) > std::fabs(speed_) && next * speed_ >= 0.0 ? acceleration_ : deceleration_;
        const double change = rate * SampleSeconds;
        next = (std::max)(speed_ - change, (std::min)(speed_ + change, next));
        if (speed_ > 0.0 && next < 0.0) next = 0.0;
        if (speed_ < 0.0 && next > 0.0) next = 0.0;
        if (phase == Phase::Brake) { if (!FenceEndpoint(next)) return false; }
        else if (!FenceFlushTarget(next)) return false;
        const double updated = progress_ + 0.5 * (speed_ + next) * SampleSeconds;
        const double roundoff = Roundoff();
        if (!std::isfinite(updated) || updated < -roundoff || updated > 1.0 + roundoff ||
            (phase == Phase::Retreat && updated < flushTarget_ - roundoff) ||
            (phase == Phase::Return && updated > flushTarget_ + roundoff)) return Fail(Reason::Bounds);
        progress_ = (std::max)(0.0, (std::min)(1.0, updated)); speed_ = next;
        if (speed_ == 0.0) {
            if (phase == Phase::Brake) {
                if (progress_ < EDM105::TriggerProgress || progress_ < EDM105::StrokeRatio)
                    return Fail(Reason::Bounds);
                flushAnchor_ = progress_; flushTarget_ = flushAnchor_ - EDM105::StrokeRatio;
                feedback_.flushAnchor = flushAnchor_ * feedback_.length;
                feedback_.flushPhase = Phase::Retreat;
            } else if (std::fabs(progress_ - flushTarget_) <= roundoff) {
                progress_ = flushTarget_;
                if (phase == Phase::Retreat) {
                    feedback_.flushRetreat = progress_ * feedback_.length;
                    feedback_.flushPhase = Phase::DwellRetreat;
                } else {
                    feedback_.flushReturned = progress_ * feedback_.length;
                    feedback_.flushPhase = Phase::DwellReturn;
                }
                flushDwellTick_ = tick; flushDwellUs_ = nowUs;
            }
        }
        feedback_.q = progress_ == 1.0 ? feedback_.length : progress_ * feedback_.length;
        feedback_.v = speed_ * feedback_.length;
        if (!std::isfinite(feedback_.q) || !std::isfinite(feedback_.v)) return Fail(Reason::Bounds);
        return true;
    }
    bool FenceFlushTarget(double& next) noexcept
    {
        const int sign = feedback_.flushPhase == EDM105::Phase::Retreat ? -1 : 1;
        if (speed_ * sign < 0.0 || next * sign < 0.0) return Fail(Reason::Dynamics);
        const double distance = sign * (flushTarget_ - progress_);
        const double current = std::fabs(speed_);
        double candidate = std::fabs(next);
        const double minimum = (std::max)(0.0, current - deceleration_ * SampleSeconds);
        const double roundoff = Roundoff();
        if (distance < -roundoff || StopDistance(current) > distance + roundoff) return Fail(Reason::Bounds);
        const double candidateNeed = 0.5 * (current + candidate) * SampleSeconds + StopDistance(candidate);
        if (candidateNeed > distance) {
            const double budget = distance - 0.5 * current * SampleSeconds;
            if (budget < -roundoff) return Fail(Reason::Bounds);
            const double drop = deceleration_ * SampleSeconds;
            const double scaled = (std::max)(0.0, budget) / (drop * SampleSeconds);
            const double steps = (std::max)(1.0, std::ceil((std::sqrt(1.0 + 8.0 * scaled) - 1.0) / 2.0));
            candidate = ((std::max)(0.0, budget) / SampleSeconds +
                0.5 * drop * steps * (steps - 1.0)) / steps;
            if (!std::isfinite(candidate) || candidate < minimum - roundoff / SampleSeconds)
                return Fail(Reason::Bounds);
            candidate = (std::max)(minimum, (std::min)(std::fabs(next), candidate));
        }
        next = sign * candidate;
        return std::isfinite(next) || Fail(Reason::Dynamics);
    }
    void ResetBurstWindows() noexcept
    {
        burstPhase_ = feedback_.segmentActive ? 1U : 0U; pulseWindow_ = 0U;
        burstQualifiedMask_ = burstReverseMask_ = burstForwardMask_ = 0U;
        burstEnterBaseline_ = decision_.Current().enterCount;
        pulseAbortBaseline_ = 0ULL; pulseEntry_ = EDM89::Mark{};
        pulseLowSeen_ = pulseAbortSeen_ = false;
    }
    bool CompletePulseWindow() noexcept
    {
        if (!pulseWindow_) return true;
        if (!pulseLowSeen_ || !pulseAbortSeen_ || !pulseEntry_.sequence ||
            decision_.Current().enterCount != burstEnterBaseline_ ||
            feedback_.targetRatio < 0.0 || speed_ < 0.0) return Fail(Reason::Burst);
        ++feedback_.suppressedPulseWindows;
        return true;
    }
    bool ObserveBurstConversion(const EDM88::Frame& conversion, bool fresh,
        std::uint64_t previousCapturedUs, std::uint64_t elapsed) noexcept
    {
        const auto& evidence = decision_.Current();
        using EDMGapServo::ShortState;
        if (conversion.voltageV != EDM99::Voltage(elapsed)) return Fail(Reason::Burst);
        if (!fresh) return true;
        if (conversion.capturedAtUs <= previousCapturedUs ||
            conversion.capturedAtUs < segmentUs_ || feedback_.phase < burstPhase_ ||
            feedback_.phase > burstPhase_ + 1U) return Fail(Reason::Burst);
        burstPhase_ = feedback_.phase;
        const unsigned window = EDM99::PulseWindow(elapsed);
        if (window != pulseWindow_) {
            if (!CompletePulseWindow()) return false;
            if (window && window != pulseWindow_ + 1U) return Fail(Reason::Burst);
            if (!window && pulseWindow_ && pulseWindow_ != EDM99::PulseWindows) return Fail(Reason::Burst);
            pulseWindow_ = window; pulseEntry_ = EDM89::Mark{};
            pulseLowSeen_ = pulseAbortSeen_ = false;
            pulseAbortBaseline_ = evidence.entryAborts;
        }
        if (window) {
            if (evidence.enterCount != burstEnterBaseline_ || evidence.shortActive ||
                feedback_.targetRatio < 0.0 || speed_ < 0.0) return Fail(Reason::Burst);
            if (EDM99::IsPulseLow(elapsed)) {
                if (pulseAbortSeen_ || evidence.state != ShortState::Entering ||
                    !evidence.pending.sequence || !evidence.pending.capturedAtUs)
                    return Fail(Reason::Burst);
                if (!pulseLowSeen_) pulseEntry_ = evidence.pending;
                if (!EDM89::SameMark(pulseEntry_, evidence.pending)) return Fail(Reason::Burst);
                pulseLowSeen_ = true; ++feedback_.pulseLowFreshFrames;
            } else {
                if (!pulseLowSeen_ || evidence.state != ShortState::Clear ||
                    evidence.entryAborts != pulseAbortBaseline_ + 1ULL ||
                    conversion.sequence <= pulseEntry_.sequence ||
                    conversion.capturedAtUs <= pulseEntry_.capturedAtUs ||
                    (!pulseAbortSeen_ && conversion.capturedAtUs - pulseEntry_.capturedAtUs >= DirectionDwellUs))
                    return Fail(Reason::Burst);
                pulseAbortSeen_ = true;
            }
        }
        if (feedback_.phase == 4U || feedback_.phase == 6U) {
            const unsigned bit = feedback_.phase == 4U ? 1U : 2U;
            const std::uint64_t before = burstEnterBaseline_ + (bit == 1U ? 0ULL : 1ULL);
            if (!(burstQualifiedMask_ & bit)) {
                if (evidence.state == ShortState::Active) {
                    if (evidence.enterCount != before + 1ULL ||
                        evidence.enterStart.capturedAtUs < segmentUs_ +
                            (bit == 1U ? EDM99::FirstRetreatStartUs : EDM99::SecondRetreatStartUs) ||
                        evidence.enterConfirm.sequence != conversion.sequence ||
                        evidence.enterConfirm.capturedAtUs != conversion.capturedAtUs ||
                        evidence.enterConfirm.capturedAtUs - evidence.enterStart.capturedAtUs < DirectionDwellUs ||
                        feedback_.targetRatio >= 0.0 || (bit == 2U && !(burstForwardMask_ & 1U)))
                        return Fail(Reason::Burst);
                    burstQualifiedMask_ |= bit; ++feedback_.qualifiedRetreatWindows;
                } else if (evidence.state != ShortState::Entering || evidence.enterCount != before)
                    return Fail(Reason::Burst);
            } else if (evidence.state != ShortState::Active || evidence.enterCount != before + 1ULL)
                return Fail(Reason::Burst);
        } else if (evidence.enterCount != burstEnterBaseline_ +
            ((burstQualifiedMask_ & 1U) ? 1ULL : 0ULL) + ((burstQualifiedMask_ & 2U) ? 1ULL : 0ULL))
            return Fail(Reason::Burst);
        return true;
    }
    bool ObserveBurstReversal(int sign) noexcept
    {
        if (sign < 0) {
            const unsigned bit = feedback_.phase == 4U ? 1U : feedback_.phase == 6U ? 2U : 0U;
            if (!bit || !(burstQualifiedMask_ & bit) || (burstReverseMask_ & bit) ||
                (bit == 2U && !(burstForwardMask_ & 1U))) return Fail(Reason::Burst);
            burstReverseMask_ |= bit; ++feedback_.reverseEpisodes;
        } else {
            const unsigned bit = feedback_.phase == 5U ? 1U : feedback_.phase == 7U ? 2U : 0U;
            if (!bit || !(burstReverseMask_ & bit) || (burstForwardMask_ & bit)) return Fail(Reason::Burst);
            burstForwardMask_ |= bit;
        }
        return true;
    }
    void ResetQuietWindow() noexcept
    {
        quietZeroStartUs_ = quietZeroStartTick_ = quietZeroRun_ = 0ULL;
        quietStableProgress_ = 0.0; quietPhase_ = 0U;
        quietLastNonzeroSign_ = 0; quietProved_ = false;
    }
    bool ObserveQuietPhase(unsigned phase) noexcept
    {
        if (quietPhase_ == phase) return true;
        if (quietPhase_ && !quietProved_) return Fail(Reason::Noise);
        ResetQuietWindow();
        if (EDM98::IsQuietPhase(phase)) quietPhase_ = phase;
        return true;
    }
    // Zero means no scalar displacement and zero commanded velocity. It is not
    // encoder standstill, drive settling, or an additional physical permission.
    bool StepQuietWindow(std::uint64_t tick, std::uint64_t nowUs,
        double previousProgress, double previousSpeed, int sign) noexcept
    {
        if (!EDM98::IsQuietPhase(feedback_.phase)) return true;
        if (quietPhase_ != feedback_.phase) return Fail(Reason::Noise);
        if (sign) {
            if (quietLastNonzeroSign_ && sign != quietLastNonzeroSign_) {
                ++feedback_.quietSignChanges; return Fail(Reason::Noise);
            }
            quietLastNonzeroSign_ = sign;
        }
        if (speed_ == 0.0 && previousSpeed == 0.0 && progress_ == previousProgress) {
            if (!quietZeroRun_) {
                quietZeroStartUs_ = nowUs; quietZeroStartTick_ = tick;
                quietStableProgress_ = progress_;
            } else if (progress_ != quietStableProgress_) return Fail(Reason::Noise);
            ++quietZeroRun_; ++feedback_.quietZeroTicks;
            if (!quietProved_ && quietZeroRun_ >= EDM98::QuietProofTicks &&
                tick - quietZeroStartTick_ >= EDM98::QuietProofTicks &&
                nowUs - quietZeroStartUs_ >= EDM98::QuietProofUs) {
                quietProved_ = true; ++feedback_.quietWindows;
            }
        } else {
            if (quietProved_) return Fail(Reason::Noise);
            quietZeroStartUs_ = quietZeroStartTick_ = quietZeroRun_ = 0ULL;
        }
        return true;
    }
    static double Roundoff() noexcept { return 64.0 * (std::numeric_limits<double>::epsilon)(); }
    // Exact stopping distance for full 250 us trapezoidal steps, including the
    // last partial velocity drop. Thus the final zero frame stays in [0,L].
    double StopDistance(double speed) const noexcept
    {
        if (speed <= 0.0) return 0.0;
        const double drop = deceleration_ * SampleSeconds;
        const double residual = std::fmod(speed, drop);
        return 0.5 * speed * (speed / deceleration_) +
            0.5 * SampleSeconds * residual * (1.0 - residual / drop);
    }
    bool FenceEndpoint(double& next) noexcept
    {
        const int sign = speed_ > 0.0 ? 1 : speed_ < 0.0 ? -1 : next > 0.0 ? 1 : next < 0.0 ? -1 : 0;
        if (!sign) return true;
        const double distance = sign > 0 ? 1.0 - progress_ : progress_;
        const double current = std::fabs(speed_);
        double candidate = std::fabs(next);
        const double minimum = (std::max)(0.0, current - deceleration_ * SampleSeconds);
        const double roundoff = Roundoff();
        if (StopDistance(current) > distance + roundoff) return Fail(Reason::Bounds);
        const double candidateNeed = 0.5 * (current + candidate) * SampleSeconds + StopDistance(candidate);
        if (candidateNeed > distance) {
            const double budget = distance - 0.5 * current * SampleSeconds;
            if (budget < -roundoff) return Fail(Reason::Bounds);
            const double drop = deceleration_ * SampleSeconds;
            const double scaled = (std::max)(0.0, budget) / (drop * SampleSeconds);
            const double steps = (std::max)(1.0, std::ceil((std::sqrt(1.0 + 8.0 * scaled) - 1.0) / 2.0));
            candidate = ((std::max)(0.0, budget) / SampleSeconds +
                0.5 * drop * steps * (steps - 1.0)) / steps;
            if (!std::isfinite(candidate) || candidate < minimum - roundoff / SampleSeconds)
                return Fail(Reason::Bounds);
            // Compensate only cancellation at the final sub-ulp endpoint.
            candidate = (std::max)(minimum, (std::min)(std::fabs(next), candidate));
        }
        next = sign * candidate;
        return std::isfinite(next) || Fail(Reason::Dynamics);
    }
    bool Fail(Reason reason) noexcept
    {
        if (!feedback_.faulted) feedback_.reason = reason;
        feedback_.faulted = true; feedback_.active = feedback_.segmentActive = feedback_.segmentCompleted = false;
        feedback_.gateReady = false; feedback_.v = 0.0; speed_ = 0.0;
        feedback_.flushPhase = EDM105::Phase::None;
        return false;
    }
};
static_assert(sizeof(Segment) <= 1280U, "EDM96 pure segment storage remains bounded");
}
