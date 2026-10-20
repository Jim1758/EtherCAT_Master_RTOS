#pragma once
#include "EDMSixAxisCommonCurve.h"

// EDM126 numerical traversal. Admission, GAP qualification, actual output
// commit, and physical settling remain caller responsibilities. History owns
// geometry only; visiting it cannot dispatch or complete an NC instruction.
namespace EDM126
{
using Point = EDM112::Point;
constexpr unsigned HistoryCapacity = 16U;
constexpr unsigned RequiredSeamStableCycles = 200U;
constexpr std::uint64_t MaximumSourceAgeUs = 1500ULL;
constexpr double SampleSeconds = 0.00025;
enum class Reason : std::uint8_t { None, Scope, Source, Geometry, Limits, Sequence,
    Seam, Clock, Uncommitted, Bounds, Dynamics, State, Physical, Overflow };
struct Source
{
    EDM109::Scope scope{};
    std::uint64_t ordinal = 0ULL, dispatch = 0ULL, epoch = 0ULL, segment = 0ULL;
    int sourcePC = -1;
};
inline bool ValidSource(const Source& s) noexcept
{ return EDM109::ValidScope(s.scope) && s.dispatch && s.epoch && s.segment && s.sourcePC >= 0; }
inline bool SameSource(const Source& a, const Source& b) noexcept
{
    return EDM109::SameScope(a.scope,b.scope) && a.ordinal == b.ordinal &&
        a.dispatch == b.dispatch && a.epoch == b.epoch && a.segment == b.segment && a.sourcePC == b.sourcePC;
}
struct Limits
{
    Point maxVelocityPps{}, maxAccelerationPps2{};
    double maxVelocityMmSec = 0.0, accelerationMmSec2 = 0.0, decelerationMmSec2 = 0.0;
};
struct Settings
{
    double maximumRetreatMm = 0.0;
    unsigned seamStableCycles = RequiredSeamStableCycles;
};
struct Feedback
{
    Source authorizedSource{}, cursorSource{};
    std::uint64_t tick = 0ULL, nowUs = 0ULL;
    double localMm = 0.0, lengthMm = 0.0, stationMm = 0.0;
    double velocityMmSec = 0.0, accelerationMmSec2 = 0.0;
    Point positionPulse{}, velocityPps{}, accelerationPps2{};
    unsigned historyCount = 0U, historyDepth = 0U, maximumHistoryDepth = 0U;
    std::uint64_t completedRows = 0ULL, backwardCrossings = 0ULL, forwardCrossings = 0ULL;
    unsigned seamStableCycles = 0U;
    bool active = false, awaitingCurrent = false, awaitingSeam = false;
    bool atCurrentEnd = false, atRetreatLimit = false, completeReady = false;
    bool stopRequested = false, stopped = false, faulted = false;
    Reason reason = Reason::None;
};

// A target is an exact retained geometry identity and local canonical coordinate.
// It contains no output authority and cannot create a history entry.
struct Cursor { Source source{}; double localMm = 0.0; };
inline bool SameCursor(const Cursor& a, const Cursor& b) noexcept
{ return SameSource(a.source,b.source) && a.localMm == b.localMm; }

inline bool Positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }
inline int Sign(double x) noexcept { return x > 0.0 ? 1 : x < 0.0 ? -1 : 0; }
inline double Roundoff(double scale) noexcept
{ return 64.0 * (std::numeric_limits<double>::epsilon)() * (std::max)(1.0, std::fabs(scale)); }

// Built once at admission. Evaluate never rebuilds EDM112 geometry.
class CachedMapping
{
public:
    bool Bind(const EDM112::Curve& curve, const Limits& limits) noexcept
    {
        valid_ = false;
        if (!EDM112::ValidCurve(curve) || !Positive(limits.maxVelocityMmSec) ||
            !Positive(limits.accelerationMmSec2) || !Positive(limits.decelerationMmSec2)) return false;
        NCArcPlaneAxes axes{};
        if (curve.kind == EDM112::Kind::Arc && !TryGetNCArcPlaneAxes(static_cast<int>(curve.plane), axes)) return false;
        double speed = limits.maxVelocityMmSec, accel = limits.accelerationMmSec2, decel = limits.decelerationMmSec2;
        for (unsigned i = 0U; i < 6U; ++i) {
            if (!Positive(limits.maxVelocityPps[i]) || !Positive(limits.maxAccelerationPps2[i])) return false;
            double d1 = std::fabs((curve.line.endPulse[i] - curve.line.startPulse[i]) / curve.lengthMm), d2 = 0.0;
            if (curve.kind == EDM112::Kind::Arc && (i == axes.u || i == axes.v)) {
                d1 = curve.radiusMm * std::fabs(curve.sweepRadians) * curve.line.pulsePerNative[i] / curve.lengthMm;
                d2 = d1 * std::fabs(curve.sweepRadians) / curve.lengthMm;
            }
            if (!std::isfinite(d1) || !std::isfinite(d2)) return false;
            if (d1 > 0.0) {
                speed = (std::min)(speed, limits.maxVelocityPps[i] / d1);
                const double tangential = (d2 > 0.0 ? 0.5 : 1.0) * limits.maxAccelerationPps2[i] / d1;
                accel = (std::min)(accel, tangential); decel = (std::min)(decel, tangential);
            }
            if (d2 > 0.0) speed = (std::min)(speed, std::sqrt(0.5 * limits.maxAccelerationPps2[i] / d2));
        }
        if (!Positive(speed) || !Positive(accel) || !Positive(decel) ||
            !Positive(decel * SampleSeconds) || !std::isfinite(speed * speed / decel)) return false;
        curve_ = curve; limits_ = limits; axes_ = axes;
        speed_ = speed * (1.0 - 8.0 * (std::numeric_limits<double>::epsilon)());
        acceleration_ = accel; deceleration_ = decel; valid_ = true; return true;
    }
    bool Evaluate(double q, double v, double a, Point& p, Point& velocity, Point& acceleration) const noexcept
    {
        if (!valid_ || !std::isfinite(q) || q < 0.0 || q > curve_.lengthMm ||
            !std::isfinite(v) || !std::isfinite(a) || &p == &velocity || &p == &acceleration || &velocity == &acceleration) return false;
        const double u = q == curve_.lengthMm ? 1.0 : q / curve_.lengthMm;
        Point candidate{}, d1{}, d2{}, cv{}, ca{};
        for (unsigned i = 0U; i < 6U; ++i) {
            const double delta = curve_.line.endPulse[i] - curve_.line.startPulse[i];
            candidate[i] = delta == 0.0 ? curve_.line.startPulse[i] : curve_.line.startPulse[i] + delta * u;
            d1[i] = delta / curve_.lengthMm;
        }
        if (curve_.kind == EDM112::Kind::Arc) {
            const double angle = curve_.startAngle + u * curve_.sweepRadians;
            const double c = std::cos(angle), s = std::sin(angle);
            const unsigned slots[2U] = {axes_.u, axes_.v};
            for (unsigned j = 0U; j < 2U; ++j) {
                const unsigned i = slots[j]; const double trig = j ? s : c;
                candidate[i] = curve_.line.startPulse[i] + (curve_.centerOffsetMm[j] + curve_.radiusMm * trig) * curve_.line.pulsePerNative[i];
                d1[i] = ((j ? c : -s) * curve_.radiusMm * curve_.sweepRadians) * curve_.line.pulsePerNative[i] / curve_.lengthMm;
                d2[i] = ((-trig * curve_.radiusMm * curve_.sweepRadians) * curve_.sweepRadians) * curve_.line.pulsePerNative[i] / curve_.lengthMm / curve_.lengthMm;
            }
        }
        if (q == 0.0) candidate = curve_.line.startPulse;
        else if (q == curve_.lengthMm) candidate = curve_.line.endPulse;
        for (unsigned i = 0U; i < 6U; ++i) {
            cv[i] = d1[i] * v; ca[i] = d1[i] * a + d2[i] * v * v;
            if (!std::isfinite(candidate[i]) || !std::isfinite(cv[i]) || !std::isfinite(ca[i]) ||
                std::fabs(cv[i]) > limits_.maxVelocityPps[i] + Roundoff(limits_.maxVelocityPps[i]) ||
                std::fabs(ca[i]) > limits_.maxAccelerationPps2[i] + Roundoff(limits_.maxAccelerationPps2[i])) return false;
        }
        p = candidate; velocity = cv; acceleration = ca; return true;
    }
    // Raw cached tangent, normalized by XYZ native distance. Rotary axes
    // share that same parameter; no geometry builder or motion limit is run.
    bool TangentPulsePerMm(double q, Point& tangent) const noexcept
    {
        if (!valid_ || !std::isfinite(q) || q < 0.0 || q > curve_.lengthMm) return false;
        Point d1{};
        for (unsigned i=0U;i<6U;++i) d1[i]=(curve_.line.endPulse[i]-curve_.line.startPulse[i])/curve_.lengthMm;
        if (curve_.kind==EDM112::Kind::Arc) {
            const double u=q==curve_.lengthMm?1.0:q/curve_.lengthMm;
            const double angle=curve_.startAngle+u*curve_.sweepRadians;
            const double c=std::cos(angle),s=std::sin(angle);
            d1[axes_.u]=-s*curve_.radiusMm*curve_.sweepRadians*curve_.line.pulsePerNative[axes_.u]/curve_.lengthMm;
            d1[axes_.v]= c*curve_.radiusMm*curve_.sweepRadians*curve_.line.pulsePerNative[axes_.v]/curve_.lengthMm;
        }
        double norm=0.0;
        for (unsigned i=0U;i<3U;++i) norm=std::hypot(norm,d1[i]/curve_.line.pulsePerNative[i]);
        if (!Positive(norm)) return false;
        for (unsigned i=0U;i<6U;++i) {
            d1[i]/=norm;
            if (!std::isfinite(d1[i])) return false;
        }
        tangent=d1;return true;
    }
    const EDM112::Curve& Geometry() const noexcept { return curve_; }
    double MaximumSpeed() const noexcept { return speed_; }
    double Acceleration() const noexcept { return acceleration_; }
    double Deceleration() const noexcept { return deceleration_; }
    const Limits& FrozenLimits() const noexcept { return limits_; }
private:
    EDM112::Curve curve_{}; Limits limits_{}; NCArcPlaneAxes axes_{};
    double speed_ = 0.0, acceleration_ = 0.0, deceleration_ = 0.0; bool valid_ = false;
};

template<unsigned Capacity> class BasicTraversal
{
    static_assert(Capacity >= 1U && Capacity <= HistoryCapacity, "bounded history storage");
public:
    bool Begin(const EDM109::Scope& scope, const Settings& settings) noexcept
    {
        if (begun_ || !EDM109::ValidScope(scope) || !Positive(settings.maximumRetreatMm) ||
            settings.seamStableCycles != RequiredSeamStableCycles) return Fail(Reason::Scope);
        scope_ = scope; settings_ = settings; begun_ = true;
        feedback_ = Feedback{}; feedback_.awaitingCurrent = true; return true;
    }
    bool BindCurrent(const EDM112::Curve& curve, const Source& source, const Limits& limits,
        std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (compound_) return Fail(Reason::Source);
        if (!begun_ || currentValid_ || pending_ || stop_ || feedback_.faulted || !tick || !nowUs) return Fail(Reason::State);
        if (!ValidSource(source) || !EDM109::SameScope(scope_, source.scope)) return Fail(Reason::Source);
        Row row{};
        if (!row.mapping.Bind(curve, limits)) return Fail(Reason::Geometry);
        row.source = source;
        if (count_) {
            const Row& previous = History(count_ - 1U);
            if (previous.source.ordinal == (std::numeric_limits<std::uint64_t>::max)() ||
                source.ordinal != previous.source.ordinal + 1ULL || source.dispatch <= previous.source.dispatch ||
                source.epoch <= previous.source.epoch || source.segment <= previous.source.segment) return Fail(Reason::Sequence);
            if (!EDM109::SamePulse(previous.mapping.Geometry().line.endPulse, curve.line.startPulse) ||
                !EDM109::SamePulse(previous.mapping.Geometry().line.pulsePerNative, curve.line.pulsePerNative)) return Fail(Reason::Seam);
            if (tick < feedback_.tick || nowUs < feedback_.nowUs) return Fail(Reason::Clock);
            row.beginMm = previous.beginMm + previous.mapping.Geometry().lengthMm;
        }
        if (!std::isfinite(row.beginMm + curve.lengthMm) || row.beginMm + curve.lengthMm <= row.beginMm) return Fail(Reason::Bounds);
        current_ = row; currentValid_ = true; cursor_ = count_;
        feedback_.authorizedSource = source; feedback_.localMm = 0.0;
        feedback_.velocityMmSec = feedback_.accelerationMmSec2 = 0.0;
        feedback_.tick = tick; feedback_.nowUs = nowUs;
        feedback_.active = true; feedback_.awaitingCurrent = false;
        feedback_.seamStableCycles = 0U; lastTarget_ = 0.0;
        if (!Refresh()) return false;
        // Bind is legal only at a caller-authenticated stationary start.
        committed_ = feedback_; committedCursor_ = cursor_; committedValid_ = true;
        frontierMm_ = (std::max)(frontierMm_, row.beginMm);
        committedFrontierMm_ = frontierMm_; firstSample_ = true; return true;
    }
    // EDM173 only. A compound plan retains one real NC parent and epoch;
    // the Motion owner additionally proves every Source against its sealed
    // genuine child identity. The fingerprint never grants Motion authority.
    bool BeginCompound(const EDM109::Scope& scope, const Settings& settings,
        std::uint64_t fingerprint, std::uint64_t parentDispatch, std::uint64_t epoch,
        int sourcePC, unsigned count) noexcept
    {
        if (!fingerprint || !parentDispatch || !epoch || sourcePC < 0 || count != 25U)
            return Fail(Reason::Source);
        if (!Begin(scope,settings)) return false;
        compound_ = true; compoundFingerprint_ = fingerprint; compoundDispatch_ = parentDispatch;
        compoundEpoch_ = epoch; compoundPC_ = sourcePC; compoundCount_ = count;
        return true;
    }
    bool BindCompoundCurrent(const EDM112::Curve& curve, const Source& source, const Limits& limits,
        std::uint64_t tick, std::uint64_t nowUs, std::uint64_t fingerprint,
        unsigned childOrdinal, unsigned count) noexcept
    {
        if (!compound_ || !begun_ || currentValid_ || pending_ || stop_ || feedback_.faulted || !tick || !nowUs)
            return Fail(Reason::State);
        if (!fingerprint || fingerprint != compoundFingerprint_ || count != compoundCount_ ||
            count != 25U || childOrdinal >= count || childOrdinal != compoundBound_ ||
            compoundAppended_ != childOrdinal || !ValidSource(source) ||
            !EDM109::SameScope(scope_,source.scope) || source.dispatch != compoundDispatch_ ||
            source.epoch != compoundEpoch_ || source.sourcePC != compoundPC_ || source.ordinal != childOrdinal ||
            source.segment <= compoundLastSegment_) return Fail(Reason::Source);
        Row row{};
        if (!row.mapping.Bind(curve,limits)) return Fail(Reason::Geometry);
        row.source = source;
        if (childOrdinal != 0U)
        {
            if (!count_ || !committedValid_ || committed_.velocityMmSec != 0.0 ||
                committed_.velocityPps != Point{} || committed_.positionPulse != curve.line.startPulse ||
                tick < committed_.tick || nowUs < committed_.nowUs) return Fail(Reason::Seam);
            const Row& previous = History(count_-1U);
            if (previous.source.ordinal+1ULL != source.ordinal || previous.source.dispatch != source.dispatch ||
                previous.source.epoch != source.epoch || previous.source.sourcePC != source.sourcePC ||
                !EDM109::SamePulse(previous.mapping.Geometry().line.endPulse,curve.line.startPulse) ||
                !EDM109::SamePulse(previous.mapping.Geometry().line.pulsePerNative,curve.line.pulsePerNative))
                return Fail(Reason::Seam);
            row.beginMm = previous.beginMm + previous.mapping.Geometry().lengthMm;
        }
        if (!std::isfinite(row.beginMm+curve.lengthMm) || row.beginMm+curve.lengthMm <= row.beginMm)
            return Fail(Reason::Bounds);
        current_ = row; currentValid_ = true; cursor_ = count_;
        feedback_.authorizedSource = source; feedback_.localMm = 0.0;
        feedback_.velocityMmSec = feedback_.accelerationMmSec2 = 0.0;
        feedback_.tick = tick; feedback_.nowUs = nowUs;
        feedback_.active = true; feedback_.awaitingCurrent = false; feedback_.seamStableCycles = 0U;
        lastTarget_ = 0.0;
        if (!Refresh()) return false;
        committed_ = feedback_; committedCursor_ = cursor_; committedValid_ = true;
        frontierMm_ = (std::max)(frontierMm_,row.beginMm); committedFrontierMm_ = frontierMm_;
        firstSample_ = true; compoundLastSegment_ = source.segment; ++compoundBound_;
        return true;
    }
    bool Step(std::uint64_t tick, std::uint64_t nowUs, double signedTargetMmSec, bool physicalSettled, const Cursor* target = nullptr) noexcept
    {
        if (!currentValid_ || suspended_ || stop_ || feedback_.faulted) return Fail(Reason::State);
        if (!std::isfinite(signedTargetMmSec)) return Fail(Reason::Dynamics);
        unsigned targetIndex = 0U;
        if (target && (!Locate(*target,targetIndex) || !Positive(signedTargetMmSec))) return Fail(Reason::Source);
        if (target) signedTargetMmSec *= targetIndex < cursor_ ? -1.0 : targetIndex > cursor_ ? 1.0 :
            static_cast<double>(Sign(target->localMm-feedback_.localMm));
        if (!Clock(tick, nowUs)) return false;
        lastTarget_ = signedTargetMmSec;
        feedback_.completeReady = feedback_.awaitingSeam = false;
        const Row& row = CursorRow(); const double length = row.mapping.Geometry().lengthMm;
        const int direction = Sign(signedTargetMmSec);
        if (feedback_.velocityMmSec == 0.0) {
            const bool backward = direction < 0 && feedback_.localMm == 0.0 && cursor_ > 0U && row.beginMm > RetreatFloor();
            const bool forward = direction > 0 && feedback_.localMm == length && cursor_ < count_;
            if (backward || forward) {
                feedback_.awaitingSeam = true;
                // A proposal never counts as physical evidence: the previous
                // committed image must be stationary at this exact seam.
                const bool same = committedValid_ && committedCursor_ == cursor_ &&
                    committed_.velocityMmSec == 0.0 && committed_.localMm == feedback_.localMm;
                feedback_.seamStableCycles = physicalSettled && same ? feedback_.seamStableCycles + 1U : 0U;
                if (feedback_.seamStableCycles >= settings_.seamStableCycles) {
                    if ((backward && feedback_.backwardCrossings == (std::numeric_limits<std::uint64_t>::max)()) ||
                        (forward && feedback_.forwardCrossings == (std::numeric_limits<std::uint64_t>::max)())) return Fail(Reason::Overflow);
                    if (backward) { --cursor_; ++feedback_.backwardCrossings; feedback_.localMm = CursorRow().mapping.Geometry().lengthMm; }
                    else { ++cursor_; ++feedback_.forwardCrossings; feedback_.localMm = 0.0; }
                    feedback_.seamStableCycles = 0U; feedback_.awaitingSeam = false;
                }
                feedback_.accelerationMmSec2 = 0.0;
                return Propose();
            }
        }
        feedback_.seamStableCycles = 0U;
        if (!Integrate(signedTargetMmSec, false, target && targetIndex == cursor_ ? target : nullptr)) return false;
        feedback_.completeReady = !target && cursor_ == count_ && feedback_.localMm == length &&
            feedback_.velocityMmSec == 0.0 && direction > 0;
        return Propose();
    }
    // Refresh only an already committed terminal endpoint. No integration,
    // history crossing or new completion is permitted while hardware settles.
    bool StepCompletedEndpoint(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (!currentValid_ || suspended_ || stop_ || pending_ || !committedValid_ || feedback_.faulted ||
            !committed_.completeReady || !feedback_.completeReady || cursor_ != count_ ||
            committedCursor_ != cursor_ || feedback_.tick != committed_.tick ||
            feedback_.localMm != current_.mapping.Geometry().lengthMm ||
            feedback_.positionPulse != committed_.positionPulse || feedback_.velocityMmSec != 0.0 ||
            feedback_.velocityPps != Point{} || feedback_.historyDepth || feedback_.awaitingSeam)
            return Fail(Reason::State);
        if (!Clock(tick, nowUs)) return false;
        feedback_.accelerationMmSec2 = 0.0;
        return Propose();
    }
    bool CommitMapped(std::uint64_t tick) noexcept
    {
        if (!pending_ || tick != feedback_.tick || feedback_.faulted) return Fail(Reason::Uncommitted);
        frontierMm_ = (std::max)(frontierMm_, feedback_.stationMm);
        committed_ = feedback_; committedCursor_ = cursor_; committedFrontierMm_ = frontierMm_;
        committedValid_ = true; pending_ = false; return true;
    }
    bool AppendCompleted(std::uint64_t boundaryTick, std::uint64_t commitId) noexcept
    {
        if (compound_) return Fail(Reason::Source);
        if (!currentValid_ || suspended_ || pending_ || stop_ || feedback_.faulted || cursor_ != count_ ||
            !committedValid_ || !committed_.completeReady || committed_.velocityMmSec != 0.0 ||
            committed_.localMm != current_.mapping.Geometry().lengthMm ||
            boundaryTick < committed_.tick || !commitId || commitId <= lastBoundaryCommit_) return Fail(Reason::State);
        if (feedback_.completedRows == (std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::Overflow);
        if (count_ == Capacity) { head_ = (head_ + 1U) % Capacity; --count_; }
        history_[(head_ + count_) % Capacity] = current_; ++count_;
        cursor_ = count_ - 1U; currentValid_ = false; lastBoundaryCommit_ = commitId;
        ++feedback_.completedRows; feedback_.historyCount = count_;
        feedback_.completeReady = feedback_.active = false; feedback_.awaitingCurrent = true;
        committed_ = feedback_; committedCursor_ = cursor_; return true;
    }
    bool AppendCompoundCompleted(std::uint64_t boundaryTick, std::uint64_t fingerprint,
        unsigned completedChildOrdinal) noexcept
    {
        if (!compound_ || !fingerprint || fingerprint != compoundFingerprint_ ||
            completedChildOrdinal >= compoundCount_ || completedChildOrdinal != compoundAppended_ ||
            completedChildOrdinal+1U != compoundBound_ || !currentValid_ || suspended_ || pending_ || stop_ ||
            feedback_.faulted || cursor_ != count_ || !committedValid_ || !committed_.completeReady ||
            committed_.velocityMmSec != 0.0 || committed_.velocityPps != Point{} ||
            committed_.localMm != current_.mapping.Geometry().lengthMm ||
            committed_.positionPulse != current_.mapping.Geometry().line.endPulse ||
            current_.source.ordinal != completedChildOrdinal || current_.source.dispatch != compoundDispatch_ ||
            current_.source.epoch != compoundEpoch_ || current_.source.sourcePC != compoundPC_ ||
            boundaryTick < committed_.tick) return Fail(Reason::State);
        if (feedback_.completedRows == (std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::Overflow);
        if (count_ == Capacity) { head_ = (head_+1U)%Capacity; --count_; }
        history_[(head_+count_)%Capacity] = current_; ++count_;
        cursor_ = count_-1U; currentValid_ = false; ++compoundAppended_;
        ++feedback_.completedRows; feedback_.historyCount = count_;
        feedback_.completeReady = feedback_.active = false; feedback_.awaitingCurrent = true;
        committed_ = feedback_; committedCursor_ = cursor_;
        return true;
    }
    bool IsCompound() const noexcept { return compound_; }
    std::uint64_t CompoundFingerprint() const noexcept { return compoundFingerprint_; }
    unsigned CompoundBoundRows() const noexcept { return compoundBound_; }
    unsigned CompoundCompletedRows() const noexcept { return compoundAppended_; }
    bool BeginStop(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        // An authenticated SAFETY request may follow fenced, unmapped cycles
        // or replace an earlier request with an outstanding stop proposal.
        // Rebase from the exact committed image without replaying that time.
        // Retain the bounded capture age: a long physical zero fence must
        // not restart an old nonzero reference. The caller additionally
        // proves the fresh physical clock and SAFETY tuple.
        if (!committedValid_ || !currentValid_ || suspended_ || tick < committed_.tick || nowUs < committed_.nowUs ||
            nowUs - committed_.nowUs > MaximumSourceAgeUs)
            return Fail(Reason::Clock);
        feedback_ = committed_; cursor_ = committedCursor_; frontierMm_ = committedFrontierMm_;
        feedback_.tick = tick; feedback_.nowUs = nowUs; feedback_.faulted = false; feedback_.reason = Reason::None;
        feedback_.stopRequested = true; feedback_.completeReady = feedback_.awaitingSeam = false;
        feedback_.seamStableCycles = 0U; stop_ = true; pending_ = false; firstSample_ = true;
        feedback_.stopped = feedback_.velocityMmSec == 0.0; return true;
    }
    bool StepStop(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (!stop_ || feedback_.faulted || !Clock(tick, nowUs)) return false;
        if (!sameTick_ && !Integrate(0.0, true)) return false;
        feedback_.stopped = feedback_.velocityMmSec == 0.0;
        feedback_.completeReady = false; return Propose();
    }
    bool StepTo(std::uint64_t tick, std::uint64_t nowUs, double speedMmSec,
        const Cursor& target, bool physicalSettled) noexcept
    { return Step(tick,nowUs,speedMmSec,physicalSettled,&target); }
    bool CaptureCursor(Cursor& result) const noexcept
    {
        if (!currentValid_ || pending_ || !committedValid_ || feedback_.faulted) return false;
        result.source=committed_.cursorSource; result.localMm=committed_.localMm; return true;
    }
    bool ResolveStation(double station, Cursor& result) const noexcept
    {
        if (!currentValid_ || !std::isfinite(station) || station < RetreatFloor() ||
            station > current_.beginMm+current_.mapping.Geometry().lengthMm) return false;
        for (unsigned i=0U;i<=count_;++i) {
            const Row& row=i==count_?current_:History(i);
            if (station >= row.beginMm && station <= row.beginMm+row.mapping.Geometry().lengthMm) {
                result.source=row.source; result.localMm=station-row.beginMm;
                if (result.localMm>row.mapping.Geometry().lengthMm) result.localMm=row.mapping.Geometry().lengthMm;
                return true;
            }
        }
        return false;
    }
    // EDM132 absolute retreat resolves a seam to its newer row at local zero.
    // This remains reachable when the seam itself is the retreat floor;
    // entering the older row there would require crossing below that floor.
    // The legacy resolver above deliberately retains its original identity.
    bool ResolveRetreatStation(double station, Cursor& result) const noexcept
    {
        if (!currentValid_ || !std::isfinite(station) || station < RetreatFloor() ||
            station > current_.beginMm+current_.mapping.Geometry().lengthMm) return false;
        for (unsigned slot=count_+1U;slot>0U;--slot) {
            const unsigned i=slot-1U;
            const Row& row=i==count_?current_:History(i);
            const double end=row.beginMm+row.mapping.Geometry().lengthMm;
            if (station>=row.beginMm && station<=end) {
                result.source=row.source;result.localMm=station-row.beginMm;
                if (station==end) result.localMm=row.mapping.Geometry().lengthMm;
                return true;
            }
        }
        return false;
    }
    bool EvaluateCursorAtRest(const Cursor& target, Point& position, double* station=nullptr) const noexcept
    {
        unsigned index=0U;if (!Locate(target,index)) return false;
        const Row& row=index==count_?current_:History(index);
        Point velocity{},acceleration{};
        if (!row.mapping.Evaluate(target.localMm,0.0,0.0,position,velocity,acceleration)) return false;
        if (station) *station=row.beginMm+target.localMm;
        return true;
    }
    bool PreviousCursorTarget(double fraction, Cursor& result) const noexcept
    {
        if (!currentValid_ || pending_ || !cursor_ || !std::isfinite(fraction) || fraction<0.0 || fraction>1.0) return false;
        const Row& row=History(cursor_-1U); result.source=row.source;
        result.localMm=row.mapping.Geometry().lengthMm*fraction;
        return row.beginMm+result.localMm>=RetreatFloor();
    }
    const CachedMapping& CursorMapping() const noexcept { return CursorRow().mapping; }
    double MinimumStationMm() const noexcept { return RetreatFloor(); }
    bool Suspended() const noexcept { return suspended_; }
    bool SuspendAtRest(const Cursor& anchor) noexcept
    {
        Cursor current{};
        if (suspended_ || stop_ || !CaptureCursor(current) || !SameCursor(current,anchor) ||
            committed_.velocityMmSec!=0.0 || feedback_.tick!=committed_.tick) return Fail(Reason::State);
        suspended_=true; return true;
    }
    bool ResumeAtRest(const Cursor& anchor, const Point& mappedPulse, unsigned physicalStable,
        std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        Cursor current{};
        if (!suspended_ || stop_ || !CaptureCursor(current) || !SameCursor(current,anchor) ||
            committed_.velocityMmSec!=0.0 || !EDM109::SamePulse(mappedPulse,committed_.positionPulse) ||
            physicalStable<RequiredSeamStableCycles || tick<committed_.tick || nowUs<committed_.nowUs)
            return Fail(Reason::State);
        // Only a frozen zero cursor can be rebased after a separately mapped
        // excursion. The first proposal remains exactly zero at the anchor.
        feedback_=committed_; feedback_.tick=tick; feedback_.nowUs=nowUs;
        feedback_.completeReady=false; feedback_.seamStableCycles=0U;
        firstSample_=true; suspended_=false; return true;
    }
    const Feedback& Current() const noexcept { return feedback_; }
    const Feedback& Committed() const noexcept { return committed_; }
    void Clear() noexcept
    {
        // Geometry slots need not be cleared; count and scope fence access.
        scope_ = EDM109::Scope{}; settings_ = Settings{};
        feedback_ = Feedback{}; committed_ = Feedback{};
        head_ = count_ = cursor_ = committedCursor_ = 0U;
        lastBoundaryCommit_ = 0ULL; frontierMm_ = committedFrontierMm_ = lastTarget_ = 0.0;
        begun_ = currentValid_ = committedValid_ = pending_ = stop_ = firstSample_ = sameTick_ = suspended_ = false;
        compound_ = false; compoundFingerprint_ = compoundDispatch_ = compoundEpoch_ = compoundLastSegment_ = 0ULL;
        compoundPC_ = -1; compoundCount_ = compoundBound_ = compoundAppended_ = 0U;
    }
private:
    struct Row { CachedMapping mapping{}; Source source{}; double beginMm = 0.0; };
    Row history_[Capacity]{}, current_{};
    EDM109::Scope scope_{}; Settings settings_{}; Feedback feedback_{}, committed_{};
    unsigned head_ = 0U, count_ = 0U, cursor_ = 0U, committedCursor_ = 0U;
    std::uint64_t lastBoundaryCommit_ = 0ULL;
    double frontierMm_ = 0.0, committedFrontierMm_ = 0.0, lastTarget_ = 0.0;
    bool begun_ = false, currentValid_ = false, committedValid_ = false, pending_ = false;
    bool stop_ = false, firstSample_ = false, sameTick_ = false, suspended_ = false;
    bool compound_ = false;
    std::uint64_t compoundFingerprint_ = 0ULL, compoundDispatch_ = 0ULL, compoundEpoch_ = 0ULL, compoundLastSegment_ = 0ULL;
    int compoundPC_ = -1;
    unsigned compoundCount_ = 0U, compoundBound_ = 0U, compoundAppended_ = 0U;
    const Row& History(unsigned index) const noexcept { return history_[(head_ + index) % Capacity]; }
    const Row& CursorRow() const noexcept { return cursor_ == count_ ? current_ : History(cursor_); }
    bool Locate(const Cursor& target, unsigned& index) const noexcept
    {
        if (!ValidSource(target.source) || !std::isfinite(target.localMm) || !currentValid_ ||
            target.source.ordinal>current_.source.ordinal || current_.source.ordinal-target.source.ordinal>count_) return false;
        index=count_-static_cast<unsigned>(current_.source.ordinal-target.source.ordinal);
        const Row& row=index==count_?current_:History(index);
        return SameSource(target.source,row.source) && target.localMm>=0.0 && target.localMm<=row.mapping.Geometry().lengthMm &&
            row.beginMm+target.localMm>=RetreatFloor();
    }
    double RetreatFloor() const noexcept
    { return (std::max)(count_ ? History(0U).beginMm : current_.beginMm, frontierMm_ - settings_.maximumRetreatMm); }
    bool Fail(Reason reason) noexcept
    { feedback_.faulted = true; feedback_.reason = reason; feedback_.completeReady = false; return false; }
    bool Clock(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (pending_) return Fail(Reason::Uncommitted);
        sameTick_ = firstSample_ && tick == feedback_.tick && nowUs == feedback_.nowUs;
        if (!sameTick_ && (feedback_.tick == (std::numeric_limits<std::uint64_t>::max)() ||
            tick != feedback_.tick + 1ULL || nowUs <= feedback_.nowUs ||
            nowUs - feedback_.nowUs > MaximumSourceAgeUs)) return Fail(Reason::Clock);
        firstSample_ = false; feedback_.tick = tick; feedback_.nowUs = nowUs; return true;
    }
    static double StopDistance(double speed, double decel) noexcept
    {
        if (speed <= 0.0) return 0.0;
        const double drop = decel * SampleSeconds, residual = std::fmod(speed, drop);
        return 0.5 * speed * (speed / decel) + 0.5 * SampleSeconds * residual * (1.0 - residual / drop);
    }
    bool Integrate(double target, bool stopping, const Cursor* destination = nullptr) noexcept
    {
        if (sameTick_) { feedback_.accelerationMmSec2 = 0.0; return true; }
        const Row& row = CursorRow(); const CachedMapping& map = row.mapping;
        const double length = map.Geometry().lengthMm, old = feedback_.velocityMmSec;
        target = (std::max)(-map.MaximumSpeed(), (std::min)(map.MaximumSpeed(), target));
        if (old * target < 0.0) target = 0.0;
        const bool accelerating = old == 0.0 || (Sign(old) == Sign(target) && std::fabs(target) > std::fabs(old));
        const double rate = accelerating && !stopping ? map.Acceleration() : map.Deceleration();
        double next = old + (std::max)(-rate * SampleSeconds, (std::min)(rate * SampleSeconds, target - old));
        if (std::fabs(target - old) <= rate * SampleSeconds) next = target;
        const int direction = Sign(old) ? Sign(old) : Sign(next);
        const double floor = (std::max)(0.0, RetreatFloor() - row.beginMm);
        double lower = (std::min)(length, floor), upper = length;
        if (destination) {
            if (destination->localMm < feedback_.localMm) lower=(std::max)(lower,destination->localMm);
            else if (destination->localMm > feedback_.localMm) upper=destination->localMm;
            else lower=upper=destination->localMm;
        }
        const double tolerance = Roundoff(length);
        if (direction) {
            const double distance = direction > 0 ? upper - feedback_.localMm : feedback_.localMm - lower;
            const double speed = std::fabs(old), minimum = (std::max)(0.0, speed - map.Deceleration() * SampleSeconds);
            double candidate = std::fabs(next);
            if (distance < -tolerance || StopDistance(speed, map.Deceleration()) > distance + tolerance) return Fail(Reason::Bounds);
            if (0.5 * (speed + candidate) * SampleSeconds + StopDistance(candidate, map.Deceleration()) > distance) {
                const double budget = distance - 0.5 * speed * SampleSeconds;
                if (budget < -tolerance) return Fail(Reason::Bounds);
                const double drop = map.Deceleration() * SampleSeconds;
                const double scaled = (std::max)(0.0, budget) / (drop * SampleSeconds);
                const double steps = (std::max)(1.0, std::ceil((std::sqrt(1.0 + 8.0 * scaled) - 1.0) / 2.0));
                candidate = ((std::max)(0.0, budget) / SampleSeconds + 0.5 * drop * steps * (steps - 1.0)) / steps;
                if (!std::isfinite(candidate) || candidate < minimum - tolerance / SampleSeconds) return Fail(Reason::Bounds);
                candidate = (std::max)(minimum, (std::min)(std::fabs(next), candidate));
                next = direction * candidate;
            }
            // A rounded braking speed can retain a sub-roundoff residual
            // after its final deceleration sample. Permit zero only when
            // that residual's entire sample displacement is below the
            // endpoint roundoff budget; Evaluate still enforces every axis
            // acceleration limit on the resulting exact-zero sample.
            if (std::fabs(next)*SampleSeconds <= tolerance &&
                minimum*SampleSeconds <= tolerance &&
                std::fabs(distance-0.5*speed*SampleSeconds) <= tolerance) next=0.0;
        }
        double q = feedback_.localMm + 0.5 * (old + next) * SampleSeconds;
        if (next == 0.0) {
            if (std::fabs(q - upper) <= tolerance) q = upper;
            else if (std::fabs(q - lower) <= tolerance) q = lower;
        }
        if (!std::isfinite(q) || q < lower - tolerance || q > upper + tolerance) return Fail(Reason::Bounds);
        // Only zero-speed sub-roundoff endpoint correction is permitted.
        if (q < lower || q > upper) return Fail(Reason::Bounds);
        feedback_.localMm = q; feedback_.velocityMmSec = next;
        feedback_.accelerationMmSec2 = (next - old) / SampleSeconds; return true;
    }
    bool Refresh() noexcept
    {
        const Row& row = CursorRow();
        feedback_.cursorSource = row.source; feedback_.lengthMm = row.mapping.Geometry().lengthMm;
        feedback_.stationMm = row.beginMm + feedback_.localMm;
        feedback_.historyCount = count_; feedback_.historyDepth = count_ - cursor_;
        feedback_.maximumHistoryDepth = (std::max)(feedback_.maximumHistoryDepth, feedback_.historyDepth);
        feedback_.atCurrentEnd = cursor_ == count_ && feedback_.localMm == feedback_.lengthMm && feedback_.velocityMmSec == 0.0;
        feedback_.atRetreatLimit = feedback_.velocityMmSec == 0.0 && feedback_.stationMm <= RetreatFloor() + Roundoff(feedback_.stationMm);
        if (!row.mapping.Evaluate(feedback_.localMm, feedback_.velocityMmSec, feedback_.accelerationMmSec2,
            feedback_.positionPulse, feedback_.velocityPps, feedback_.accelerationPps2)) return Fail(Reason::Dynamics);
        return true;
    }
    bool Propose() noexcept { if (!Refresh()) return false; pending_ = true; return true; }
};
using Traversal = BasicTraversal<HistoryCapacity>;
static_assert(sizeof(Feedback) <= 1024U, "EDM126 feedback stays bounded");
}
