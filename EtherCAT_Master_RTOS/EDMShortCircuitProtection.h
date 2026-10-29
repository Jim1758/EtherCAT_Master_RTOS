#pragma once

#include "EDMGapServo.h"
#include <cmath>
#include <cstdint>
#include <type_traits>

// EDM280 physical GAP collision qualification and operation policy only.
// No allocation, IO, wall clock, alarm calls, axis ownership or discharge permit.
// A physical-input adapter must independently validate routing/calibration/PDO.
namespace EDMShortCircuitProtection
{
struct Config
{
    std::uint64_t revision = 1ULL;
    double thresholdV = 10.0, hysteresisV = 1.0;
    std::uint64_t enterUs = 2000ULL, exitUs = 5000ULL;
    std::uint64_t maxAgeUs = 5000ULL, maxGapUs = 5000ULL;
};

struct Input
{
    std::uint64_t sequence = 0ULL, nowUs = 0ULL, capturedAtUs = 0ULL;
    double voltageV = 0.0;
    bool valid = false;
    std::uint64_t sourceIdentity = 0ULL, sourceRevision = 0ULL;
};

struct Snapshot
{
    bool valid = false, shortActive = false, blocked = true;
    std::uint64_t sequence = 0ULL;
    double voltageV = 0.0;
    // RT adapter owns this incident counter; pure detection leaves it zero.
    std::uint64_t manualStopSequence = 0ULL;
};

inline bool ValidateConfig(const Config& config) noexcept
{
    return config.revision != 0ULL && std::isfinite(config.thresholdV) &&
        config.thresholdV >= 0.0 && std::isfinite(config.hysteresisV) &&
        config.hysteresisV >= 0.0 && std::isfinite(config.thresholdV + config.hysteresisV) &&
        config.maxAgeUs != 0ULL && config.maxGapUs != 0ULL && config.maxGapUs <= config.maxAgeUs;
}

inline bool SameConfig(const Config& a, const Config& b) noexcept
{
    return a.revision == b.revision && a.thresholdV == b.thresholdV &&
        a.hysteresisV == b.hysteresisV && a.enterUs == b.enterUs && a.exitUs == b.exitUs &&
        a.maxAgeUs == b.maxAgeUs && a.maxGapUs == b.maxGapUs;
}

// Deliberately ignores shortIdleEnabled/shortMachiningEnabled: C20 OFF requires
// this collision detector. G80 voltage/time settings are not an input here.
// Only the idle threshold, hysteresis and two dwell times belong to this policy.
// Failure preserves output. Source adapter may subsequently tighten age/gap.
inline bool MakeConfig(const EDMGapServo::Profile& profile, Config& output) noexcept
{
    Config candidate{};
    candidate.revision = static_cast<std::uint64_t>(profile.revision);
    candidate.thresholdV = profile.shortIdleV;
    candidate.hysteresisV = profile.shortHysteresisV;
    candidate.enterUs = static_cast<std::uint64_t>(profile.shortEnterMs) * 1000ULL;
    candidate.exitUs = static_cast<std::uint64_t>(profile.shortExitMs) * 1000ULL;
    if (!ValidateConfig(candidate)) return false;
    output = candidate;
    return true;
}

class Detector
{
public:
    bool Configure(const Config& config) noexcept
    {
        if (!ValidateConfig(config))
        {
            configured_ = false;
            Invalidate();
            return false;
        }
        if (configured_ && SameConfig(config_, config)) return true;
        config_ = config;
        configured_ = true;
        Invalidate(); // A changed profile never clears an accepted short.
        return true;
    }

    const Snapshot& Current() const noexcept { return snapshot_; }

    Snapshot Step(const Input& input) noexcept
    {
        snapshot_.sequence = input.sequence;
        snapshot_.voltageV = std::isfinite(input.voltageV) ? input.voltageV : 0.0;
        if (!configured_ || !input.valid || !std::isfinite(input.voltageV) || input.voltageV < 0.0 ||
            input.sequence == 0ULL || input.capturedAtUs > input.nowUs ||
            input.nowUs - input.capturedAtUs > config_.maxAgeUs)
            return Invalidate();
        if (haveSample_ && input.nowUs < lastNowUs_) return Invalidate();

        if (haveSource_ && (sourceIdentity_ != input.sourceIdentity || sourceRevision_ != input.sourceRevision))
        {
            // Rebaseline the new source, but make the change visible as invalid
            // in this very RT tick. It cannot silently publish clear or count
            // dwell accumulated under another source/calibration identity.
            sourceIdentity_ = input.sourceIdentity;
            sourceRevision_ = input.sourceRevision;
            lastSequence_ = input.sequence;
            lastCapturedAtUs_ = input.capturedAtUs;
            lastNowUs_ = input.nowUs;
            haveSample_ = true;
            return Invalidate();
        }
        if (haveSample_)
        {
            if (input.sequence <= lastSequence_ || input.nowUs < lastNowUs_ ||
                input.capturedAtUs <= lastCapturedAtUs_)
                return Invalidate();
            if (input.capturedAtUs - lastCapturedAtUs_ > config_.maxGapUs)
            {
                // An outage does not qualify dwell; retain a new monotonic
                // baseline so subsequent genuinely fresh samples can recover.
                lastSequence_ = input.sequence;
                lastCapturedAtUs_ = input.capturedAtUs;
                lastNowUs_ = input.nowUs;
                return Invalidate();
            }
        }
        haveSource_ = true;
        sourceIdentity_ = input.sourceIdentity;
        sourceRevision_ = input.sourceRevision;
        haveSample_ = true;
        lastSequence_ = input.sequence;
        lastCapturedAtUs_ = input.capturedAtUs;
        lastNowUs_ = input.nowUs;

        if (!snapshot_.shortActive)
        {
            if (input.voltageV <= config_.thresholdV)
            {
                if (pending_ != Pending::Entering)
                {
                    pending_ = Pending::Entering;
                    pendingSinceUs_ = input.capturedAtUs;
                }
                if (input.capturedAtUs - pendingSinceUs_ >= config_.enterUs)
                {
                    snapshot_.shortActive = true;
                    haveAcceptedLevel_ = true;
                    recoveryRequired_ = false;
                    pending_ = Pending::None;
                }
            }
            else
            {
                pending_ = Pending::None;
                if (input.voltageV > config_.thresholdV + config_.hysteresisV)
                {
                    haveAcceptedLevel_ = true;
                    recoveryRequired_ = false;
                }
            }
        }
        else
        {
            // Fresh evidence may retain an accepted short immediately; clearing
            // it always requires the complete strictly-above-hysteresis dwell.
            haveAcceptedLevel_ = true;
            recoveryRequired_ = false;
            if (input.voltageV > config_.thresholdV + config_.hysteresisV)
            {
                if (pending_ != Pending::Exiting)
                {
                    pending_ = Pending::Exiting;
                    pendingSinceUs_ = input.capturedAtUs;
                }
                if (input.capturedAtUs - pendingSinceUs_ >= config_.exitUs)
                {
                    snapshot_.shortActive = false;
                    pending_ = Pending::None;
                }
            }
            else pending_ = Pending::None;
        }
        // Startup/recovery low samples inside enter dwell must not masquerade
        // as a valid clear. A new high sample or a qualified short proves level.
        snapshot_.valid = haveAcceptedLevel_ && !recoveryRequired_;
        snapshot_.blocked = !snapshot_.valid || snapshot_.shortActive;
        return snapshot_;
    }

private:
    enum class Pending : std::uint8_t { None, Entering, Exiting };
    Snapshot Invalidate() noexcept
    {
        snapshot_.valid = false;
        snapshot_.blocked = true;
        pending_ = Pending::None;
        recoveryRequired_ = true;
        return snapshot_;
    }
    Config config_{};
    Snapshot snapshot_{};
    Pending pending_ = Pending::None;
    std::uint64_t pendingSinceUs_ = 0ULL;
    std::uint64_t lastSequence_ = 0ULL, lastCapturedAtUs_ = 0ULL, lastNowUs_ = 0ULL;
    std::uint64_t sourceIdentity_ = 0ULL, sourceRevision_ = 0ULL;
    bool configured_ = false, haveSource_ = false, haveSample_ = false;
    bool haveAcceptedLevel_ = false, recoveryRequired_ = true;
};

// These values are provenance, not the generic Motion command kind. Producers
// explicitly preserve the authored G-code through optimized/two-leg moves.
enum class Operation : std::uint8_t
{
    Unspecified = 0, G00 = 1, G07 = 7, G28 = 28, G30 = 30,
    G32 = 32, G53 = 53, G161 = 161, Jog = 240, Inch = 241, Mpg = 242
};

enum class Response : std::uint8_t
{
    Continue, ControlledStop, EmergencyStopAlarm, InvalidSource
};

inline bool IsManual(Operation operation) noexcept
{
    return operation == Operation::Jog || operation == Operation::Inch || operation == Operation::Mpg;
}

inline bool IsProtectedPositioning(Operation operation) noexcept
{
    return operation == Operation::G00 || operation == Operation::G07 || operation == Operation::G28 ||
        operation == Operation::G30 || operation == Operation::G53;
}

// C20 ON bypasses only a qualified SHORT for manual movement. It never makes
// stale/missing/changed physical-source evidence valid and never bypasses NC.
// G32/G161 retain forced-target behavior on a valid short. No new policy is
// invented for Unspecified/other process motion; callers preserve their guards.
inline Response Evaluate(Operation operation, const Snapshot& snapshot, bool manualC20Bypass) noexcept
{
    // Existing EDM process/other motion keeps its own source-validity guards.
    // Forced-target G32/G161 bypass only a valid SHORT, never source invalidity.
    const bool scoped = IsManual(operation) || IsProtectedPositioning(operation) ||
        operation == Operation::G32 || operation == Operation::G161;
    if (!scoped) return Response::Continue;
    if (!snapshot.valid) return Response::InvalidSource;
    if (!snapshot.shortActive) return Response::Continue;
    if (IsProtectedPositioning(operation)) return Response::EmergencyStopAlarm;
    if (IsManual(operation) && !manualC20Bypass) return Response::ControlledStop;
    return Response::Continue;
}

static_assert(std::is_trivially_copyable<Config>::value && std::is_trivially_copyable<Input>::value &&
    std::is_trivially_copyable<Snapshot>::value, "Collision protection uses fixed RT-copyable values.");
}
