#pragma once
// PBC-3H: RT-local HOME stopped evidence. This proves a bounded stationary raw
// interval only. It is not a physical capture, HOME authority, reference commit
// receipt or permission to clear an uncertain compensation output.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

struct MotionPbcHomeStopKey
{
    std::uint64_t ownerGeneration = 0ULL;
    std::uint64_t epoch = 0ULL;
    std::uint64_t referenceGeneration = 0ULL;
    std::uint64_t incidentGeneration = 0ULL;
    std::uint64_t sourceIdentity = 0ULL;
    double resolutionPPR = 0.0;
    double finalLead = 0.0;
    double machineOffsetPulse = 0.0;
    double windowPulse = 0.0;
    double commandPulse = 0.0;
    double logicalCommandPulse = 0.0;
    // PBC-3J: even zero-valued enable modes belong to the proof identity.
    bool enablePitch = false;
    bool enableBacklash = false;
    bool isReverse = false;
    bool axisReverse = false;
};

struct MotionPbcHomeStopSample
{
    std::uint64_t tick = 0ULL;
    std::uint32_t rawPosition = 0U;
    double unwrappedPulse = 0.0;
    bool valid = false;
    bool contiguous = false;
    // The real Motion adapter supplies HOME ownership, idle, fresh operation-
    // enabled input, fault/alarm checks and all nominal velocity == 0 guards.
    // A software actual-velocity value is deliberately not an input.
    bool eligible = false;
};

class MotionPbcHomeStopProof
{
public:
    static constexpr unsigned RequiredSamples = 200U;

    // Single RT writer, fixed storage, no retry, heap, lock or external IO.
    bool Observe(const MotionPbcHomeStopKey& key,
        const MotionPbcHomeStopSample& sample) noexcept
    {
        const std::uint64_t previousTick = lastObservedTick_;
        if (sample.tick > lastObservedTick_) lastObservedTick_ = sample.tick;
        if (sample.tick == 0ULL || sample.tick <= previousTick ||
            !sample.valid || !sample.contiguous || !sample.eligible ||
            !ValidKey(key) || !ValidRaw(sample.rawPosition, sample.unwrappedPulse))
        {
            Invalidate();
            return false;
        }

        const bool adjacent = previousTick != 0ULL &&
            previousTick != (std::numeric_limits<std::uint64_t>::max)() &&
            sample.tick == previousTick + 1ULL;
        if (samples_ == 0U || !adjacent || !SameKey(key_, key))
        {
            Start(key, sample);
            return false;
        }

        // Verify the unwrapped evidence against the very same raw32 samples.
        // A half-range step is ambiguous, and a fabricated unchanged software
        // position must not turn a changing encoder into stopped evidence.
        const std::uint32_t rawDelta = sample.rawPosition - lastRawPosition_;
        if (rawDelta == 0x80000000U)
        {
            Invalidate();
            return false;
        }
        const std::int64_t signedDelta = rawDelta <= 0x7fffffffU ?
            static_cast<std::int64_t>(rawDelta) :
            -static_cast<std::int64_t>(0x100000000ULL - rawDelta);
        if (sample.unwrappedPulse - lastUnwrappedPulse_ != static_cast<double>(signedDelta))
        {
            Invalidate();
            return false;
        }

        const double minimum = (std::min)(minimumPulse_, sample.unwrappedPulse);
        const double maximum = (std::max)(maximumPulse_, sample.unwrappedPulse);
        const double excursion = maximum - minimum;
        const double limit = ExcursionLimit(key.windowPulse);
        if (!std::isfinite(excursion) || !std::isfinite(limit) || excursion > limit)
        {
            // A new stationary interval may begin here, but the old interval
            // can never be reused after drift (including after saturation).
            Start(key, sample);
            return false;
        }

        minimumPulse_ = minimum;
        maximumPulse_ = maximum;
        lastRawPosition_ = sample.rawPosition;
        lastUnwrappedPulse_ = sample.unwrappedPulse;
        lastSampleTick_ = sample.tick;
        if (samples_ < RequiredSamples) ++samples_;
        return samples_ == RequiredSamples;
    }

    // HOME mailbox processing precedes the current input pass. Therefore the
    // evidence must be the immediately preceding completed RT sample, with no
    // intervening scalar/key change. Caller must recheck current eligibility
    // and its real coordinate/owner/epoch/alarm reservation separately.
    bool IsReady(std::uint64_t currentTick, const MotionPbcHomeStopKey& key,
        std::uint32_t currentRawPosition, double currentUnwrappedPulse) const noexcept
    {
        return samples_ == RequiredSamples && currentTick != 0ULL &&
            lastSampleTick_ != (std::numeric_limits<std::uint64_t>::max)() &&
            lastSampleTick_ + 1ULL == currentTick && lastObservedTick_ == lastSampleTick_ &&
            ValidKey(key) && SameKey(key_, key) &&
            ValidRaw(currentRawPosition, currentUnwrappedPulse) &&
            currentRawPosition == lastRawPosition_ &&
            SameDouble(currentUnwrappedPulse, lastUnwrappedPulse_);
    }

    void Invalidate() noexcept
    {
        samples_ = 0U;
        lastSampleTick_ = 0ULL;
        // Preserve the observed high-water tick: revoked or duplicate samples
        // cannot be replayed to rebuild evidence within one RT pass.
    }
    void Consume() noexcept { Invalidate(); }
    unsigned Samples() const noexcept { return samples_; }

private:
    MotionPbcHomeStopKey key_{};
    std::uint64_t lastObservedTick_ = 0ULL;
    std::uint64_t lastSampleTick_ = 0ULL;
    std::uint32_t lastRawPosition_ = 0U;
    double lastUnwrappedPulse_ = 0.0;
    double minimumPulse_ = 0.0;
    double maximumPulse_ = 0.0;
    unsigned samples_ = 0U;

    static bool SameDouble(double a, double b) noexcept
    { return std::memcmp(&a, &b, sizeof(double)) == 0; }

    static bool ValidKey(const MotionPbcHomeStopKey& key) noexcept
    {
        return key.ownerGeneration != 0ULL && key.epoch != 0ULL && key.sourceIdentity != 0ULL &&
            std::isfinite(key.resolutionPPR) && key.resolutionPPR > 0.0 &&
            std::isfinite(key.finalLead) && key.finalLead > 0.0 &&
            std::isfinite(key.machineOffsetPulse) &&
            std::isfinite(key.windowPulse) && key.windowPulse > 0.0 &&
            std::isfinite(key.commandPulse) && std::isfinite(key.logicalCommandPulse);
    }

    static bool ValidRaw(std::uint32_t raw, double unwrapped) noexcept
    {
        // Exact single-pulse accounting ends outside the binary64 integer
        // range. Refuse a stopped proof rather than silently rounding a pulse.
        constexpr double MaximumExactPulse = 9007199254740991.0;
        return std::isfinite(unwrapped) && std::abs(unwrapped) <= MaximumExactPulse &&
            std::trunc(unwrapped) == unwrapped &&
            static_cast<std::uint32_t>(static_cast<std::int64_t>(unwrapped)) == raw;
    }

    static bool SameKey(const MotionPbcHomeStopKey& a,
        const MotionPbcHomeStopKey& b) noexcept
    {
        return a.ownerGeneration == b.ownerGeneration && a.epoch == b.epoch &&
            a.referenceGeneration == b.referenceGeneration &&
            a.incidentGeneration == b.incidentGeneration && a.sourceIdentity == b.sourceIdentity &&
            SameDouble(a.resolutionPPR, b.resolutionPPR) && SameDouble(a.finalLead, b.finalLead) &&
            SameDouble(a.machineOffsetPulse, b.machineOffsetPulse) &&
            SameDouble(a.windowPulse, b.windowPulse) && SameDouble(a.commandPulse, b.commandPulse) &&
            SameDouble(a.logicalCommandPulse, b.logicalCommandPulse) &&
            a.enablePitch == b.enablePitch && a.enableBacklash == b.enableBacklash &&
            a.isReverse == b.isReverse && a.axisReverse == b.axisReverse;
    }

    static double ExcursionLimit(double window) noexcept
    { return (std::min)(0.5 * window, (std::max)(4.0, 0.25 * window)); }

    void Start(const MotionPbcHomeStopKey& key,
        const MotionPbcHomeStopSample& sample) noexcept
    {
        key_ = key;
        lastSampleTick_ = sample.tick;
        lastRawPosition_ = sample.rawPosition;
        lastUnwrappedPulse_ = minimumPulse_ = maximumPulse_ = sample.unwrappedPulse;
        samples_ = 1U;
    }
};

// Boot-only local arithmetic/state checks. No axis, owner, reference or IO is
// modified. Comprehensive adversarial tests live outside production sources.
inline bool CheckMotionPbcHomeStopProof(unsigned& checks) noexcept
{
    checks = 0U;
    bool pass = true;
    const auto check = [&checks, &pass](bool result) noexcept
    { ++checks; pass = pass && result; };
    MotionPbcHomeStopKey key{};
    key.ownerGeneration = key.epoch = key.sourceIdentity = 1ULL;
    key.resolutionPPR = 1000.0; key.finalLead = 1.0;
    key.windowPulse = 16.0; key.commandPulse = key.logicalCommandPulse = 100.0;
    MotionPbcHomeStopSample sample{};
    sample.valid = sample.contiguous = sample.eligible = true;
    MotionPbcHomeStopProof proof;
    for (unsigned i = 1U; i < MotionPbcHomeStopProof::RequiredSamples; ++i)
    {
        sample.tick = i;
        sample.rawPosition = 100U + (i & 1U) * 2U;
        sample.unwrappedPulse = static_cast<double>(sample.rawPosition);
        (void)proof.Observe(key, sample);
    }
    check(proof.Samples() == MotionPbcHomeStopProof::RequiredSamples - 1U);
    check(!proof.IsReady(200ULL, key, sample.rawPosition, sample.unwrappedPulse));
    sample.tick = 200ULL; sample.rawPosition = 100U; sample.unwrappedPulse = 100.0;
    check(proof.Observe(key, sample));
    check(proof.IsReady(201ULL, key, 100U, 100.0));
    check(!proof.IsReady(202ULL, key, 100U, 100.0));
    check(!proof.IsReady(201ULL, key, 102U, 102.0));
    MotionPbcHomeStopKey changed = key;
    changed.referenceGeneration = 1ULL;
    check(!proof.IsReady(201ULL, changed, 100U, 100.0));
    changed = key; changed.machineOffsetPulse = -0.0;
    check(!proof.IsReady(201ULL, changed, 100U, 100.0));
    sample.tick = 201ULL; sample.rawPosition = 105U; sample.unwrappedPulse = 105.0;
    check(!proof.Observe(key, sample) && proof.Samples() == 1U);
    sample.tick = 202ULL; sample.valid = false;
    check(!proof.Observe(key, sample) && proof.Samples() == 0U);
    sample.valid = true;
    check(!proof.Observe(key, sample) && proof.Samples() == 0U);
    for (unsigned i = 203U; i <= 402U; ++i)
    { sample.tick = i; (void)proof.Observe(key, sample); }
    check(proof.IsReady(403ULL, key, 105U, 105.0));
    proof.Consume();
    check(!proof.IsReady(403ULL, key, 105U, 105.0) && proof.Samples() == 0U);
    sample.tick = 403ULL; sample.rawPosition = 106U; // mismatched unwrap
    check(!proof.Observe(key, sample) && proof.Samples() == 0U);
    return pass;
}
