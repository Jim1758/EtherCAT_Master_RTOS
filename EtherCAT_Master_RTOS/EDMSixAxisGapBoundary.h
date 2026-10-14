#pragma once

#include "EDMCoordinatedGapDecision.h"
#include <array>
#include <type_traits>

// EDM95: a simulated GAP decision authorizes ONE authored exact-stop G1 at
// its boundary. The independent 50/20 V source keeps running during G1;
// ordinary direction changes affect only the next boundary. This value type
// creates no trajectory, physical stop proof, PID value or discharge output.
namespace EDM95
{
constexpr std::uint64_t SourcePeriodUs = 1000ULL;
constexpr std::uint64_t PhaseUs = 2000000ULL;
constexpr std::uint64_t OfferWindowUs = 1000000ULL;
constexpr std::uint64_t DirectionDwellUs = 20000ULL;
constexpr std::uint64_t PermitAgeUs = 200000ULL;
constexpr std::uint64_t MaximumSourceAgeUs = 1500ULL;
constexpr std::uint64_t PermitLifetimeUs = PermitAgeUs;
constexpr std::uint64_t SourceDwellUs = DirectionDwellUs;
constexpr std::uint64_t OfferPeriodUs = 50000ULL;
constexpr unsigned MaximumSteps = 4U;

enum class Reason : std::uint8_t
{ None, Identity, Clock, Source, Decision, Permit, Lifecycle, External };
inline const char* ReasonName(Reason reason) noexcept
{
    switch (reason) {
    case Reason::None: return "NONE"; case Reason::Identity: return "IDENTITY";
    case Reason::Clock: return "CLOCK"; case Reason::Source: return "SOURCE";
    case Reason::Decision: return "DECISION"; case Reason::Permit: return "PERMIT";
    case Reason::Lifecycle: return "LIFECYCLE"; case Reason::External: return "EXTERNAL";
    } return "UNKNOWN";
}
struct Permit
{
    std::uint64_t session = 0ULL, epoch = 0ULL;
    std::uint32_t ownerGeneration = 0U;
    unsigned step = 0U;
    std::uint64_t phaseGeneration = 0ULL, sourceSequence = 0ULL;
    std::uint64_t sourceCapturedUs = 0ULL, issuedUs = 0ULL, tick = 0ULL;
};
inline bool SamePermit(const Permit& a, const Permit& b) noexcept
{
    return a.session == b.session && a.epoch == b.epoch &&
        a.ownerGeneration == b.ownerGeneration && a.step == b.step &&
        a.phaseGeneration == b.phaseGeneration && a.sourceSequence == b.sourceSequence &&
        a.sourceCapturedUs == b.sourceCapturedUs && a.issuedUs == b.issuedUs && a.tick == b.tick;
}
inline bool EmptyPermit(const Permit& value) noexcept
{ return SamePermit(value, Permit{}); }
struct Feedback
{
    std::uint64_t session = 0ULL, epoch = 0ULL;
    std::uint32_t ownerGeneration = 0U;
    unsigned expectedStep = 0U;
    std::uint64_t tick = 0ULL, nowUs = 0ULL, phaseGeneration = 0ULL;
    std::uint64_t sourceSequence = 0ULL, sourceCapturedUs = 0ULL;
    std::uint64_t freshFrames = 0ULL, heldFrames = 0ULL, maximumAgeUs = 0ULL;
    std::uint64_t acceptedSegments = 0ULL, forwardSegments = 0ULL, retreatSegments = 0ULL;
    double voltageV = 0.0;
    int direction = 0;
    bool gateReady = false, active = false, faulted = false;
    Reason reason = Reason::None;
    EDM88::Reason sourceReason = EDM88::Reason::None;
    EDM89::Reason decisionReason = EDM89::Reason::None;
    Permit token{};
};
static_assert(std::is_trivially_copyable<Feedback>::value, "EDM95 feedback is copied by value");
static_assert(sizeof(Feedback) <= 256U, "EDM95 feedback remains bounded");

inline bool ValidIncrement(const std::array<double, 6U>& increment, double feed) noexcept
{
    if (feed != 0.12) return false;
    for (unsigned i = 0U; i < 6U; ++i)
        if (!std::isfinite(increment[i]) || increment[i] == 0.0 ||
            std::fabs(increment[i]) > (i < 3U ? 0.002 : 0.04)) return false;
    return true;
}

class Boundary
{
public:
    const Feedback& Current() const noexcept { return feedback_; }
    bool Begin(std::uint64_t session, std::uint64_t epoch, std::uint32_t owner,
        std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (feedback_.active || !session || session <= latestSession_ || !epoch || !owner || !tick || !nowUs)
            return Fail(Reason::Identity);
        latestSession_ = session; feedback_ = Feedback{}; frame_ = EDM88::Frame{};
        feedback_.session = session; feedback_.epoch = epoch; feedback_.ownerGeneration = owner;
        startUs_ = nowUs; lastTick_ = tick - 1ULL; lastUs_ = nowUs - 1ULL;
        firstDirectionUs_ = 0ULL; lastPhase_ = 0ULL; forward_ = {};
        offers_ = {}; offerIndex_ = 0U; lastOfferUs_ = 0ULL;
        EDMGapServo::Profile profile{};
        profile.shortMachiningV = 35.0; profile.shortHysteresisV = 5.0;
        profile.shortEnterMs = profile.shortExitMs = 20U;
        if (!source_.Begin(EDM88::SimulatedSource(session)) || !decision_.Begin(profile))
            return Fail(Reason::Source);
        feedback_.active = true;
        return Observe(tick, nowUs, true);
    }

    // Production generates a conversion only when 1 ms of the RT source
    // clock elapsed. No skipped conversions are backfilled or timestamped
    // with the observer's time. Held reads retain the complete original frame.
    bool Observe(std::uint64_t tick, std::uint64_t nowUs, bool currentFrame,
        bool offerAllowed = true) noexcept
    {
        if (!feedback_.active || feedback_.faulted) return false;
        if (!currentFrame || !tick || tick != lastTick_ + 1ULL || nowUs <= lastUs_ || nowUs < startUs_)
            return Fail(Reason::Clock);
        if (!frame_.sequence || (nowUs >= frame_.capturedAtUs && nowUs - frame_.capturedAtUs >= SourcePeriodUs)) {
            if (frame_.sequence == (std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::Clock);
            frame_.identity = source_.FrozenConfig().identity;
            ++frame_.sequence; frame_.capturedAtUs = nowUs; frame_.valid = true;
            frame_.voltageV = ((nowUs - startUs_) / PhaseUs) % 2ULL == 0ULL ? 50.0 : 20.0;
        }
        return ObserveConversion(tick, nowUs, frame_, offerAllowed);
    }

    // The same qualification path is exposed for deterministic source-fault
    // tests. It never writes an AxisContext or creates physical authority.
    bool ObserveConversion(std::uint64_t tick, std::uint64_t nowUs, const EDM88::Frame& conversion,
        bool offerAllowed = true) noexcept
    {
        if (!feedback_.active || feedback_.faulted) return false;
        feedback_.gateReady = false; feedback_.token = Permit{};
        if (!tick || tick != lastTick_ + 1ULL || nowUs <= lastUs_ || nowUs < startUs_ ||
            conversion.capturedAtUs < startUs_)
            return Fail(Reason::Clock);
        lastTick_ = feedback_.tick = tick; lastUs_ = feedback_.nowUs = nowUs;
        if (!source_.Observe(conversion, nowUs)) {
            feedback_.sourceReason = source_.Current().reason;
            return Fail(Reason::Source);
        }
        if (!decision_.Observe(conversion)) {
            feedback_.decisionReason = decision_.Current().reason;
            return Fail(Reason::Decision);
        }
        const auto& source = source_.Current();
        feedback_.sourceSequence = conversion.sequence; feedback_.sourceCapturedUs = conversion.capturedAtUs;
        feedback_.freshFrames = source.freshFrames; feedback_.heldFrames = source.heldFrames;
        feedback_.maximumAgeUs = source.maximumAgeUs; feedback_.voltageV = conversion.voltageV;
        feedback_.phaseGeneration = (conversion.capturedAtUs - startUs_) / PhaseUs + 1ULL;
        const auto state = decision_.Current().state;
        const int direction = state == EDMGapServo::ShortState::Clear ? 1 :
            state == EDMGapServo::ShortState::Active ? -1 : 0;
        if (feedback_.phaseGeneration != lastPhase_) {
            lastPhase_ = feedback_.phaseGeneration; firstDirectionUs_ = conversion.capturedAtUs;
        }
        feedback_.direction = direction;
        const std::uint64_t phaseTime = (nowUs - startUs_) % PhaseUs;
        const bool matching = direction == (feedback_.expectedStep % 2U == 0U ? 1 : -1);
        feedback_.gateReady = offerAllowed && feedback_.expectedStep < MaximumSteps && matching &&
            conversion.capturedAtUs >= firstDirectionUs_ &&
            conversion.capturedAtUs - firstDirectionUs_ >= DirectionDwellUs &&
            phaseTime < OfferWindowUs && feedback_.phaseGeneration == (nowUs - startUs_) / PhaseUs + 1ULL;
        if (feedback_.gateReady) {
            if (!lastOfferUs_ || nowUs - lastOfferUs_ >= OfferPeriodUs) {
                offerIndex_ = (offerIndex_ + 1U) % offers_.size();
                auto& token = offers_[offerIndex_];
                token.session = feedback_.session; token.epoch = feedback_.epoch;
                token.ownerGeneration = feedback_.ownerGeneration; token.step = feedback_.expectedStep;
                token.phaseGeneration = feedback_.phaseGeneration; token.sourceSequence = conversion.sequence;
                token.sourceCapturedUs = conversion.capturedAtUs; token.issuedUs = nowUs; token.tick = tick;
                lastOfferUs_ = nowUs;
            }
            feedback_.token = offers_[offerIndex_];
        }
        return true;
    }

    bool Accept(const Permit& token, std::uint64_t nextEpoch,
        const std::array<double, 6U>& increment, double feed) noexcept
    {
        bool offered = false;
        for (const auto& issued : offers_) offered = offered || SamePermit(issued, token);
        if (!feedback_.active || feedback_.faulted ||
            !offered ||
            feedback_.direction != (feedback_.expectedStep % 2U == 0U ? 1 : -1) ||
            !token.session || token.session != feedback_.session || token.epoch != feedback_.epoch ||
            token.ownerGeneration != feedback_.ownerGeneration || token.step != feedback_.expectedStep ||
            token.step >= MaximumSteps || !nextEpoch ||
            nextEpoch != (token.epoch == (std::numeric_limits<std::uint32_t>::max)() ? 1ULL : token.epoch + 1ULL) ||
            !token.phaseGeneration || token.phaseGeneration != feedback_.phaseGeneration ||
            !token.sourceSequence || token.sourceSequence > feedback_.sourceSequence ||
            !token.sourceCapturedUs || token.sourceCapturedUs < startUs_ ||
            token.sourceCapturedUs > feedback_.sourceCapturedUs || token.issuedUs < token.sourceCapturedUs ||
            token.issuedUs - token.sourceCapturedUs > source_.FrozenConfig().maxAgeUs ||
            token.issuedUs > feedback_.nowUs || feedback_.nowUs - token.issuedUs > PermitAgeUs ||
            !token.tick || token.tick > feedback_.tick ||
            (token.sourceCapturedUs - startUs_) / PhaseUs + 1ULL != token.phaseGeneration ||
            (token.issuedUs - startUs_) % PhaseUs >= OfferWindowUs ||
            !ValidIncrement(increment, feed)) return Fail(Reason::Permit);
        if (token.step == 0U) forward_ = increment;
        for (unsigned i = 0U; i < 6U; ++i)
            if (increment[i] != (token.step % 2U == 0U ? forward_[i] : -forward_[i]))
                return Fail(Reason::Permit);
        ++feedback_.acceptedSegments;
        if (token.step % 2U == 0U) ++feedback_.forwardSegments; else ++feedback_.retreatSegments;
        ++feedback_.expectedStep; feedback_.epoch = nextEpoch;
        feedback_.gateReady = false; feedback_.token = Permit{};
        offers_ = {}; lastOfferUs_ = 0ULL;
        return true;
    }
    void Stop() noexcept { feedback_.active = false; feedback_.gateReady = false; feedback_.token = Permit{}; offers_ = {}; }
    void Fault(Reason reason = Reason::External) noexcept { (void)Fail(reason); }
private:
    Feedback feedback_{}; EDM88::Qualifier source_{}; EDM89::Decision decision_{};
    EDM88::Frame frame_{}; std::array<double, 6U> forward_{};
    std::uint64_t latestSession_ = 0ULL, startUs_ = 0ULL, lastTick_ = 0ULL, lastUs_ = 0ULL;
    std::uint64_t firstDirectionUs_ = 0ULL, lastPhase_ = 0ULL;
    std::array<Permit, 5U> offers_{};
    std::size_t offerIndex_ = 0U;
    std::uint64_t lastOfferUs_ = 0ULL;
    bool Fail(Reason reason) noexcept
    {
        if (!feedback_.faulted) feedback_.reason = reason;
        feedback_.faulted = true; feedback_.active = false;
        feedback_.gateReady = false; feedback_.token = Permit{};
        return false;
    }
};
}
