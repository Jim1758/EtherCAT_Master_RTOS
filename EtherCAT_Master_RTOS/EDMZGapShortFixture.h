#pragma once

#include "EDMGapServo.h"
#include <limits>

// EDM37 isolated SIM-GAP short detector for the unloaded single-Z fixture.
// RT samples supply its clock/identity. This class owns no axis, source switch,
// AD/COND/PID setting, physical permit or discharge output. The NC owner must
// prove a real stopped retreat before it supplies the recovery voltage.
namespace EDM37
{
    enum class GapShortFault : std::uint8_t
    {
        None, InvalidIdentity, IdentityExhausted, InvalidVoltage,
        ContradictorySample, SequenceRegressed, ClockRegressed,
        ServiceGap, DetectorRejected
    };

    struct GapShortSnapshot
    {
        bool valid = false;
        bool fresh = false;
        bool feedInhibited = true;
        bool shortActive = false;
        EDMGapServo::ShortState state = EDMGapServo::ShortState::Invalid;
        GapShortFault fault = GapShortFault::None;
        std::uint64_t sampleTick = 0U;
        std::uint64_t sampleMonotonicUs = 0U;
        std::uint64_t sampleTimeMs = 0U;
        double voltage = 0.0;
    };

    class GapShortFixture
    {
    public:
        void Reset() noexcept
        {
            detector_.Reset();
            snapshot_ = GapShortSnapshot{};
            haveSample_ = false;
        }

        const GapShortSnapshot& Snapshot() const noexcept { return snapshot_; }

        const GapShortSnapshot& Observe(std::uint64_t tick,
            std::uint64_t monotonicUs, double voltage) noexcept
        {
            snapshot_.fresh = false;
            if (snapshot_.fault != GapShortFault::None) return snapshot_;
            if (!tick || !monotonicUs) return Fail(GapShortFault::InvalidIdentity);
            if (tick == (std::numeric_limits<std::uint64_t>::max)() ||
                monotonicUs == (std::numeric_limits<std::uint64_t>::max)())
                return Fail(GapShortFault::IdentityExhausted);
            if (!std::isfinite(voltage)) return Fail(GapShortFault::InvalidVoltage);
            if (haveSample_)
            {
                if (tick < snapshot_.sampleTick) return Fail(GapShortFault::SequenceRegressed);
                if (tick == snapshot_.sampleTick)
                {
                    if (monotonicUs != snapshot_.sampleMonotonicUs || voltage != snapshot_.voltage)
                        return Fail(GapShortFault::ContradictorySample);
                    return snapshot_; // Cached identity: no detector call or dwell credit.
                }
                if (monotonicUs <= snapshot_.sampleMonotonicUs)
                    return Fail(GapShortFault::ClockRegressed);
                // Match the NC fixture's active service deadline. A missed
                // observation cannot become short-entry or recovery evidence.
                if (monotonicUs - snapshot_.sampleMonotonicUs >= 50000U)
                    return Fail(GapShortFault::ServiceGap);
            }

            EDMGapServo::Sample sample{};
            sample.valid = true;
            sample.voltage = voltage;
            sample.sequence = tick;
            // Retain ShortDetector's integral source-millisecond semantics.
            // A 2/5 ms source-time dwell can be shorter in microseconds by
            // less than 1 ms when its first sample is between ms boundaries.
            sample.nowMs = monotonicUs / 1000U;
            const auto result = detector_.Step(FixedProfile(), sample, true);
            if (!result.valid) return Fail(GapShortFault::DetectorRejected);
            snapshot_.valid = true;
            snapshot_.fresh = true;
            snapshot_.feedInhibited = result.feedInhibited;
            snapshot_.shortActive = result.shortActive;
            snapshot_.state = result.state;
            snapshot_.sampleTick = tick;
            snapshot_.sampleMonotonicUs = monotonicUs;
            snapshot_.sampleTimeMs = sample.nowMs;
            snapshot_.voltage = voltage;
            haveSample_ = true;
            return snapshot_;
        }

    private:
        EDMGapServo::ShortDetector detector_{};
        GapShortSnapshot snapshot_{};
        bool haveSample_ = false;

        static EDMGapServo::Profile FixedProfile() noexcept
        {
            EDMGapServo::Profile profile{};
            profile.shortMachiningEnabled = true;
            profile.shortMachiningV = 30.0;
            profile.shortHysteresisV = 1.0;
            profile.shortEnterMs = 2U;
            profile.shortExitMs = 5U;
            return profile;
        }

        const GapShortSnapshot& Fail(GapShortFault fault) noexcept
        {
            // A source fault is terminal until an explicit new-test Reset.
            // Retain the last accepted identity for diagnostics; no motion
            // direction or successful clear is inferred from invalid input.
            snapshot_.fault = fault;
            snapshot_.valid = snapshot_.fresh = snapshot_.shortActive = false;
            snapshot_.feedInhibited = true;
            snapshot_.state = EDMGapServo::ShortState::Invalid;
            return snapshot_;
        }
    };
}
