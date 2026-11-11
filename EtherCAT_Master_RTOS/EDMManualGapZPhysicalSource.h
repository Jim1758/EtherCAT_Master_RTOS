#pragma once
#include "EDMGapPhysicalSource.h"
#include "EDMCoordinatedGapSource.h"

// EDM382: acquisition-only, session-owned raw ADC adapter. It deliberately
// does not consume the diagnostic EDM64 curve's output/stop state.
namespace EDM382Source {
    constexpr std::uint64_t MaximumAgeUs = 1500ULL;
    enum class TransportGapResult { Waiting, Ready, Fault };
    class Source {
    public:
        bool Begin(const EDMGapInput::Profile& profile, const EDMAnalogInput::Snapshot& baseline,
            bool read, std::uint64_t session, std::uint64_t nowUs, std::uint64_t frequency) noexcept {
            if (!session || session <= highWater_)return Fail();
            highWater_ = session; active_ = ready_ = transportGap_ = transportRetry_ = false; frame_ = {}; config_ = {};
            if (!read || !EDM63::ValidProfile(profile) || EDM63::SampleStatus(baseline, true) != EDM63::Reason::None ||
                !baseline.capturedAtUs || nowUs<baseline.capturedAtUs || nowUs - baseline.capturedAtUs>MaximumAgeUs)return Fail();
            auto admission = EDM63::MakeAdmission(profile, baseline, session, nowUs, frequency);
            if (!adapter_.Begin(admission))return Fail();
            config_.enabled = true; config_.identity.kind = EDM88::SourceKind::Physical;
            config_.identity.producerGeneration = baseline.producerGeneration;
            config_.identity.deviceId = baseline.deviceId; config_.identity.channelId = baseline.channelId;
            config_.identity.configRevision = profile.profileRevision; config_.identity.calibrationRevision = profile.calibrationRevision;
            config_.samplePeriodUs = 250ULL; config_.maxAgeUs = MaximumAgeUs;
            if (!EDM88::ValidPhysical(config_))return Fail();
            baselineCaptured_ = baseline.capturedAtUs; active_ = true; return true;
        }
        bool Observe(const EDMAnalogInput::Snapshot& sample, bool read, std::uint64_t nowUs,
            bool clockValid, std::uint64_t frequency) noexcept {
            if (!active_)return false;
            if (transportGap_)return Fail();
            const auto old = ready_ ? frame_.capturedAtUs : baselineCaptured_;
            if (!clockValid || nowUs<old || nowUs - old>MaximumAgeUs)return Fail();
            const auto receipt = adapter_.Observe(sample, read, nowUs, clockValid, frequency);
            if (receipt.state == EDM63::State::AwaitFresh)return true;
            if (!receipt.Usable() || receipt.reason != EDM63::Reason::None || !receipt.ageKnown || receipt.ageUs > MaximumAgeUs)
                return Fail();
            frame_.identity = config_.identity; frame_.sequence = sample.sequence; frame_.capturedAtUs = sample.capturedAtUs;
            frame_.voltageV = static_cast<double>(receipt.conversion.voltageMv) / 1000.0; frame_.valid = true;
            ready_ = true; return true;
        }
        bool BeginTransportGap(std::uint64_t nowUs, std::uint64_t frequency) noexcept {
            if (!CurrentAt(nowUs, frequency) || transportGap_)return false;
            transportGap_ = true; transportRetry_ = false; gapStartUs_ = gapLastUs_ = nowUs; gapSequence_ = frame_.sequence;
            ready_ = false; frame_.valid = false; return true;
        }
        // Retry is withdrawal-only: an old retained ADC may fence a pending gap,
        // but never grants readiness or motion. The wrapper owns the absolute pause
        // budget; each newly accepted conversion still proves its actual 1500us age.
        bool RestartTransportGap(std::uint64_t nowUs, std::uint64_t frequency) noexcept {
            const auto& receipt = adapter_.Snapshot();
            const auto& admission = adapter_.FrozenAdmission();
            if (!active_ || !nowUs || !frequency || frequency != admission.observerFrequency ||
                frequency != receipt.sample.qpcFrequency || !receipt.Usable() || receipt.reason != EDM63::Reason::None ||
                !EDM63::SameRoute(EDM63::ProducerRoute(receipt.sample), admission.route) ||
                !frame_.sequence || frame_.sequence != receipt.sample.sequence ||
                !frame_.capturedAtUs || frame_.capturedAtUs != receipt.sample.capturedAtUs ||
                nowUs < frame_.capturedAtUs)return false;
            if (transportGap_) {
                if (!gapStartUs_ || !gapLastUs_ || nowUs < gapLastUs_ || ready_ || frame_.valid ||
                    gapSequence_ != frame_.sequence)return false;
                if (nowUs == gapLastUs_)return true;
            }
            else if (!CurrentAt(nowUs, frequency))return false;
            transportGap_ = transportRetry_ = true; gapStartUs_ = gapLastUs_ = nowUs; gapSequence_ = frame_.sequence;
            ready_ = false; frame_.valid = false; return true;
        }
        TransportGapResult ObserveAfterTransportGap(const EDMAnalogInput::Snapshot& sample, bool read,
            std::uint64_t nowUs, bool clockValid, std::uint64_t frequency) noexcept {
            if (!active_ || !transportGap_ || !clockValid || nowUs <= gapLastUs_ || nowUs - gapStartUs_ > 100000ULL ||
                nowUs < frame_.capturedAtUs || (!transportRetry_ && nowUs - frame_.capturedAtUs > MaximumAgeUs) ||
                !read || frequency != adapter_.FrozenAdmission().observerFrequency ||
                !EDM63::SameRoute(EDM63::ProducerRoute(sample), adapter_.FrozenAdmission().route) ||
                EDM63::SampleStatus(sample, read) != EDM63::Reason::None || sample.sequence < gapSequence_) {
                Fail(); return TransportGapResult::Fault;
            }
            gapLastUs_ = nowUs;
            if (sample.sequence == gapSequence_) {
                if (!EDM63::SameSample(sample, adapter_.Snapshot().sample)) { Fail(); return TransportGapResult::Fault; }
                return TransportGapResult::Waiting;
            }
            const auto receipt = adapter_.Observe(sample, read, nowUs, clockValid, frequency);
            if (!receipt.Usable() || receipt.reason != EDM63::Reason::None || !receipt.ageKnown || receipt.ageUs > MaximumAgeUs ||
                sample.capturedAtUs <= gapStartUs_) {
                Fail(); return TransportGapResult::Fault;
            }
            frame_.identity = config_.identity; frame_.sequence = sample.sequence; frame_.capturedAtUs = sample.capturedAtUs;
            frame_.voltageV = static_cast<double>(receipt.conversion.voltageMv) / 1000.0; frame_.valid = true;
            ready_ = true; transportGap_ = transportRetry_ = false; return TransportGapResult::Ready;
        }
        bool CurrentAt(std::uint64_t nowUs, std::uint64_t frequency) const noexcept {
            return active_ && ready_ && frame_.valid && frequency == adapter_.Snapshot().sample.qpcFrequency &&
                nowUs >= frame_.capturedAtUs && nowUs - frame_.capturedAtUs <= MaximumAgeUs;
        }
        bool Ready()const noexcept { return active_ && ready_; }
        const EDM88::Frame& CurrentFrame()const noexcept { return frame_; }
        const EDM88::SourceConfig& Config()const noexcept { return config_; }
        // EDM386_GAP_FAULT_TRACE_BEGIN
        // Read-only PDO-owner observation of the adapter's actual retained receipt.
        // Its sample can precede the current ADC if the wrapper precheck failed.
        const EDM63::Receipt& DiagnosticReceipt()const noexcept { return adapter_.Snapshot(); }
        // EDM386_GAP_FAULT_TRACE_END
        void Clear() noexcept { adapter_.Cancel(); active_ = ready_ = transportGap_ = transportRetry_ = false; frame_ = {}; config_ = {}; }
    private:
        bool Fail()noexcept { active_ = ready_ = transportGap_ = transportRetry_ = false; frame_.valid = false; return false; }
        EDM63::PhysicalAdapter adapter_{}; EDM88::SourceConfig config_{}; EDM88::Frame frame_{};
        std::uint64_t highWater_ = 0, baselineCaptured_ = 0, gapStartUs_ = 0, gapLastUs_ = 0, gapSequence_ = 0;
        bool active_ = false, ready_ = false, transportGap_ = false, transportRetry_ = false;
    };
}
