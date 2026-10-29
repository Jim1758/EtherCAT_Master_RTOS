#include "G80Cycle.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace G80
{
namespace
{
constexpr double Tiny = 1e-12;
bool Positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }
bool Nonnegative(double x) noexcept { return std::isfinite(x) && x >= 0.0; }
double Norm(const Point6& p, unsigned first) noexcept
{
    return std::hypot(std::hypot(p[first], p[first + 1]), p[first + 2]);
}
bool Finite(const Point6& p) noexcept
{
    for (double x : p) if (!std::isfinite(x)) return false;
    return true;
}
}

void Cycle::Reset() noexcept { *this = Cycle{}; }

bool Cycle::Validate(const Request& r, const Input& in) const noexcept
{
    const Config& c = r.config;
    if (r.session == 0 || r.inputSource > 1U || !Finite(r.startMCS) || !Finite(r.direction) ||
        !Finite(in.actualMCS) || !std::isfinite(in.voltageV) ||
        !Positive(r.slowPercent) ||
        !Positive((r.slowPercent / 100.0) * c.slowSpeedMmMin) ||
        !Positive((r.slowPercent / 100.0) * c.slowSpeedDegMin) ||
        (r.travelMask & ~0x3fU) != 0 ||
        c.slowTouches == 0 || c.slowTouches > MaximumSlowTouches ||
        c.contactStableSamples == 0 || c.contactStableSamples > MaximumStableSamples ||
        c.releaseStableSamples == 0 || c.releaseStableSamples > MaximumStableSamples ||
        !Nonnegative(c.contactVoltageV) || !Nonnegative(c.releaseHysteresisV) ||
        !Positive(c.slowSpeedMmMin) || !Positive(c.fastSpeedMmMin) ||
        !Positive(c.slowSpeedDegMin) || !Positive(c.fastSpeedDegMin) ||
        !Positive(c.retreatMm) || !Positive(c.retreatDeg) ||
        !Nonnegative(c.maxMeanErrorMm) || !Nonnegative(c.maxMeanErrorDeg) ||
        !Positive(c.maxSearchMm) || !Positive(c.maxSearchDeg) ||
        !Positive(c.maxCycleSeconds) || !Positive(c.releaseTimeoutSeconds) ||
        !Positive(c.settleTimeoutSeconds) || !Positive(c.accelerationSeconds) ||
        !Positive(c.decelerationSeconds) ||
        c.accelerationSeconds < SampleSeconds || c.decelerationSeconds < SampleSeconds ||
        c.accelerationSeconds > c.maxCycleSeconds || c.decelerationSeconds > c.maxCycleSeconds ||
        !Positive(c.settleWindowMm) ||
        !Positive(c.settleWindowDeg) || !Positive(c.settleHoldSeconds)) return false;
    const double ln = Norm(r.direction, 0), rn = Norm(r.direction, 3);
    if (!std::isfinite(ln) || !std::isfinite(rn) || (ln == 0.0 && rn == 0.0)) return false;
    if (r.hasApproach && !Finite(r.approachEndMCS)) return false;
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        if (!Positive(r.velocityCapsNative[i]) || !Positive(r.accelerationCapsNative[i]) ||
            !Positive(r.decelerationCapsNative[i])) return false;
        if (r.hasApproach && (!std::isfinite(r.approachEndMCS[i] - r.startMCS[i]) ||
            (r.approachEndMCS[i] != r.startMCS[i] && !Positive(r.rapidVelocityNative[i])))) return false;
        if ((r.travelMask & (1U << i)) != 0 &&
            (!std::isfinite(r.travelMin[i]) || !std::isfinite(r.travelMax[i]) ||
             r.travelMin[i] >= r.travelMax[i])) return false;
    }
    return true;
}

bool Cycle::InsideTravel(const Point6& p) const noexcept
{
    for (unsigned i = 0; i < AxisCount; ++i)
        if ((request_.travelMask & (1U << i)) != 0 &&
            (p[i] < request_.travelMin[i] - Tiny || p[i] > request_.travelMax[i] + Tiny))
            return false;
    return true;
}

bool Cycle::InsideSearch(const Point6& p) const noexcept
{
    Point6 distance{};
    for (unsigned i = 0; i < AxisCount; ++i)
        if (request_.direction[i] != 0.0) distance[i] = p[i] - searchOrigin_[i];
    // Retreat behind the frozen search origin is permitted. Each forward group
    // is independently bounded in its native units, including lateral error.
    for (unsigned first = 0; first < AxisCount; first += 3)
    {
        double dot = 0.0;
        const double norm = first == 0 ? linearNorm_ : rotaryNorm_;
        if (norm == 0.0) continue;
        for (unsigned i = first; i < first + 3; ++i) dot += distance[i] * (request_.direction[i] / norm);
        const double limit = first == 0 ? request_.config.maxSearchMm : request_.config.maxSearchDeg;
        if (dot > 0.0 && Norm(distance, first) > limit + Tiny) return false;
    }
    return true;
}

bool Cycle::Begin(const Request& r, const Input& in) noexcept
{
    if (output_.active) return false; // Never reset or replace an active parent.
    Reset();
    initialized_ = true;
    request_ = r;
    feedback_.session = r.session;
    feedback_.tick = in.tick;
    feedback_.targetCount = r.config.slowTouches;
    if (Finite(r.startMCS)) output_.commandMCS = r.startMCS;
    if (!r.config.enabled) { SetFault(Fault::Disabled); return false; }
    if (!Validate(r, in)) { SetFault(Fault::InvalidRequest); return false; }
    if (!in.pdoValid) { SetFault(Fault::PdoLost, true); return false; }
    if (!in.sourceValid) { SetFault(Fault::SourceLost, true); return false; }
    if (!InsideTravel(r.startMCS) || !InsideTravel(in.actualMCS) ||
        (r.hasApproach && !InsideTravel(r.approachEndMCS)))
    { SetFault(Fault::TravelLimit); return false; }
    if (in.voltageV < r.config.contactVoltageV)
    { SetFault(Fault::InitialContact); return false; }
    // A common scalar rescaling preserves the native ratios and avoids numerical
    // dependence on how the author spells an otherwise identical direction.
    double largest = 0.0;
    for (double d : request_.direction) largest = std::max(largest, std::abs(d));
    for (double& d : request_.direction) d /= largest;
    linearNorm_ = Norm(request_.direction, 0);
    rotaryNorm_ = Norm(request_.direction, 3);
    searchLimit_ = std::numeric_limits<double>::max();
    if (linearNorm_ > 0.0) searchLimit_ = r.config.maxSearchMm / linearNorm_;
    if (rotaryNorm_ > 0.0) searchLimit_ = std::min(searchLimit_, r.config.maxSearchDeg / rotaryNorm_);
    searchOrigin_ = r.hasApproach ? r.approachEndMCS : r.startMCS;
    fastSeek_ = r.fastFirst;
    const double slow = SeekSpeed(false), fast = SeekSpeed(true);
    if (!Positive(slow) || slow <= Tiny || !Positive(fast) || fast <= Tiny ||
        !std::isfinite(ForwardRoom(searchOrigin_)))
    { SetFault(Fault::InvalidRequest); return false; }
    // Validate the complete first leg before admitting an RT session. The same
    // bounded checks run again for every later leg without allocating memory.
    if (r.hasApproach) StartApproach(); else StartSeek();
    if (feedback_.fault != Fault::None) return false;
    phaseElapsed_ = 0.0;
    lastTick_ = in.tick; lastSource_ = in.sourceSequence; lastNowUs_ = in.nowUs;
    feedback_.held = in.hold;
    feedback_.phase = Phase::InitialRelease;
    output_.active = true;
    // Begin's sample establishes the sequence baseline; release requires fresh samples.
    RefreshOutput();
    return true;
}

double Cycle::SeekSpeed(bool fast) const noexcept
{
    const Config& c = request_.config;
    const double scale = fast ? 1.0 : request_.slowPercent / 100.0;
    double rate = std::numeric_limits<double>::max();
    if (linearNorm_ > 0.0) rate = scale * (fast ? c.fastSpeedMmMin : c.slowSpeedMmMin) / (60.0 * linearNorm_);
    if (rotaryNorm_ > 0.0) rate = std::min(rate,
        scale * (fast ? c.fastSpeedDegMin : c.slowSpeedDegMin) / (60.0 * rotaryNorm_));
    for (unsigned i = 0; i < AxisCount; ++i)
        if (request_.direction[i] != 0.0)
            rate = std::min(rate, request_.velocityCapsNative[i] / std::abs(request_.direction[i]));
    return rate;
}

bool Cycle::StartLeg(const Point6& direction, double length, double speed,
    double exitSpeed, double initialSpeed) noexcept
{
    if (!Finite(direction) || !Positive(length) || !Positive(speed) || speed <= Tiny ||
        !Nonnegative(exitSpeed) || !Nonnegative(initialSpeed))
    { SetFault(Fault::InvalidRequest); return false; }
    Point6 endpoint{};
    double speedLimit = speed;
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        endpoint[i] = output_.commandMCS[i] + direction[i] * length;
        if (direction[i] != 0.0)
            speedLimit = std::min(speedLimit, request_.velocityCapsNative[i] / std::abs(direction[i]));
    }
    if (!Finite(endpoint)) { SetFault(Fault::InvalidRequest); return false; }
    // Every leg in this parent, including a paired rapid approach, uses the
    // two independent G80 ramp times. The caller's native caps may only make
    // that ramp more restrictive; G00 times are not part of this contract.
    double acceleration = speedLimit / request_.config.accelerationSeconds;
    double deceleration = speedLimit / request_.config.decelerationSeconds;
    for (unsigned i = 0; i < AxisCount; ++i)
        if (direction[i] != 0.0)
        {
            acceleration = std::min(acceleration, request_.accelerationCapsNative[i] / std::abs(direction[i]));
            deceleration = std::min(deceleration, request_.decelerationCapsNative[i] / std::abs(direction[i]));
        }
    if (!Positive(speedLimit) || speedLimit <= Tiny || !Positive(acceleration) ||
        !Positive(deceleration) || !std::isfinite(speedLimit * speedLimit) ||
        !std::isfinite(initialSpeed * initialSpeed) ||
        !std::isfinite(acceleration + deceleration) ||
        !std::isfinite(2.0 * acceleration * deceleration * length) ||
        !std::isfinite(deceleration * speedLimit * speedLimit) ||
        !std::isfinite(acceleration * exitSpeed * exitSpeed))
    { SetFault(Fault::InvalidRequest); return false; }
    // Commit only after all arithmetic is safe. A rejected future leg leaves
    // the previous valid trajectory available for controlled fault braking.
    legOrigin_ = output_.commandMCS; legDirection_ = direction;
    legLength_ = length; legPosition_ = 0.0;
    speedLimit_ = speedLimit; acceleration_ = acceleration; deceleration_ = deceleration;
    speed_ = initialSpeed;
    exitSpeed_ = std::min(exitSpeed, speedLimit_);
    exitSpeed_ = std::min(exitSpeed_, std::sqrt(initialSpeed * initialSpeed + 2.0 * acceleration_ * length));
    phaseElapsed_ = 0.0;
    return true;
}

double Cycle::ForwardRoom(const Point6& point) const noexcept
{
    double room = std::numeric_limits<double>::max();
    // Solve each native group separately with a unit direction for that group.
    // Squaring a tiny mixed-group ratio would underflow and lose its bound.
    for (unsigned first = 0; first < AxisCount; first += 3)
    {
        const double norm = first == 0 ? linearNorm_ : rotaryNorm_;
        if (norm == 0.0) continue;
        Point6 offset{}, unit{}, perpendicular{};
        double projection = 0.0;
        for (unsigned i = first; i < first + 3; ++i)
            if (request_.direction[i] != 0.0)
            {
                offset[i] = point[i] - searchOrigin_[i];
                unit[i] = request_.direction[i] / norm;
                projection += offset[i] * unit[i];
            }
        for (unsigned i = first; i < first + 3; ++i)
            perpendicular[i] = offset[i] - projection * unit[i];
        const double lateral = Norm(perpendicular, first);
        const double limit = first == 0 ? request_.config.maxSearchMm : request_.config.maxSearchDeg;
        if (!std::isfinite(lateral) || lateral > limit) return 0.0;
        const double forward = std::sqrt((limit - lateral) * (limit + lateral)) - projection;
        room = std::min(room, forward / norm);
    }
    for (unsigned i = 0; i < AxisCount; ++i)
        if ((request_.travelMask & (1U << i)) != 0 && request_.direction[i] != 0.0)
        {
            const double edge = request_.direction[i] > 0.0 ? request_.travelMax[i] : request_.travelMin[i];
            room = std::min(room, (edge - point[i]) / request_.direction[i]);
        }
    return std::max(0.0, room);
}

void Cycle::PrepareApproachBrake() noexcept
{
    if (feedback_.phase == Phase::Approach && approachScale_ > 0.0)
    {
        // A nonzero collinear junction has already reserved this stopping room.
        legLength_ = approachLength_ + ForwardRoom(searchOrigin_) / approachScale_;
    }
}

void Cycle::StartApproach() noexcept
{
    Point6 delta{};
    approachLength_ = 0.0;
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        delta[i] = request_.approachEndMCS[i] - request_.startMCS[i];
        approachLength_ = std::max(approachLength_, std::abs(delta[i]));
    }
    if (approachLength_ == 0.0) { StartSeek(); return; }
    for (double& d : delta) d /= approachLength_;
    double rapid = std::numeric_limits<double>::max();
    bool moving = false, collinear = true;
    approachScale_ = 0.0;
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        if (delta[i] != 0.0) { moving = true; rapid = std::min(rapid, request_.rapidVelocityNative[i] / std::abs(delta[i])); }
        if (std::abs(request_.direction[i]) == 1.0)
            approachScale_ = delta[i] / request_.direction[i];
    }
    for (unsigned i = 0; i < AxisCount; ++i)
        if (std::abs(delta[i] - approachScale_ * request_.direction[i]) >
            32.0 * std::numeric_limits<double>::epsilon() *
            std::max(std::abs(delta[i]), std::abs(approachScale_ * request_.direction[i]))) collinear = false;
    if (!moving) { StartSeek(); return; }
    if (!collinear || approachScale_ <= 0.0) approachScale_ = 0.0;
    double exit = 0.0;
    if (approachScale_ > 0.0)
    {
        const double seekSpeed = SeekSpeed(fastSeek_);
        double seekDeceleration = seekSpeed / request_.config.decelerationSeconds;
        for (unsigned i = 0; i < AxisCount; ++i)
            if (request_.direction[i] != 0.0)
                seekDeceleration = std::min(seekDeceleration,
                    request_.decelerationCapsNative[i] / std::abs(request_.direction[i]));
        // Braking at the junction must fit both the seek envelope and travel.
        exit = std::min(seekSpeed, std::sqrt(2.0 * seekDeceleration * ForwardRoom(searchOrigin_))) / approachScale_;
    }
    if (!StartLeg(delta, approachLength_, rapid, exit, 0.0)) return;
    if (approachScale_ > 0.0)
        exitSpeed_ = std::min(exitSpeed_, std::sqrt(2.0 * deceleration_ *
            ForwardRoom(searchOrigin_) / approachScale_));
    feedback_.phase = Phase::Approach;
}

void Cycle::StartSeek(double initialSpeed) noexcept
{
    double remaining = ForwardRoom(output_.commandMCS);
    seekTravelBound_ = false;
    for (unsigned i = 0; i < AxisCount; ++i)
        if ((request_.travelMask & (1U << i)) != 0 && request_.direction[i] != 0.0)
        {
            const double boundary = request_.direction[i] > 0.0 ? request_.travelMax[i] : request_.travelMin[i];
            const double q = (boundary - output_.commandMCS[i]) / request_.direction[i];
            if (q <= remaining + Tiny) { remaining = std::min(remaining, q); seekTravelBound_ = true; }
        }
    if (remaining <= Tiny) { SetFault(seekTravelBound_ ? Fault::TravelLimit : Fault::SearchDistanceExceeded); return; }
    if (!StartLeg(request_.direction, remaining, SeekSpeed(fastSeek_), 0.0, initialSpeed)) return;
    lowCount_ = 0; releaseCount_ = 0; touchStored_ = false;
    feedback_.contact = false;
    feedback_.phase = fastSeek_ ? Phase::FastSeek : Phase::SlowSeek;
}

void Cycle::StartRetreat() noexcept
{
    double q = std::numeric_limits<double>::max();
    if (linearNorm_ > 0.0) q = request_.config.retreatMm / linearNorm_;
    if (rotaryNorm_ > 0.0) q = std::min(q, request_.config.retreatDeg / rotaryNorm_);
    Point6 delta{}, end = output_.commandMCS;
    double limit = std::numeric_limits<double>::max();
    for (unsigned i = 0; i < AxisCount; ++i)
        if (request_.direction[i] != 0.0)
        {
            end[i] = feedback_.lastContactMCS[i] - request_.direction[i] * q;
            delta[i] = end[i] - output_.commandMCS[i];
            // An inconsistent touch/settle pose must never turn a commanded
            // retreat into a forward approach on any selected native axis.
            if (delta[i] * request_.direction[i] > 0.0)
            { SetFault(Fault::InvalidRequest); return; }
        }
    if (!InsideTravel(end)) { SetFault(Fault::TravelLimit); return; }
    double length = 0.0;
    for (double d : delta) length = std::max(length, std::abs(d));
    if (length <= Tiny) { SetFault(Fault::ReleaseTimeout); return; }
    for (double& d : delta) d /= length;
    const double ln = Norm(delta, 0), rn = Norm(delta, 3);
    if (ln > 0.0) limit = (request_.slowPercent / 100.0) * request_.config.slowSpeedMmMin / (60.0 * ln);
    if (rn > 0.0) limit = std::min(limit, (request_.slowPercent / 100.0) * request_.config.slowSpeedDegMin / (60.0 * rn));
    if (ln == 0.0 && rn == 0.0) { SetFault(Fault::ReleaseTimeout); return; }
    if (!StartLeg(delta, length, limit, 0.0, 0.0)) return;
    releaseCount_ = 0; lowCount_ = 0;
    ResetSettle();
    feedback_.phase = Phase::Retreat;
}

// Integrate exact constant-acceleration pieces; crossing an endpoint returns the
// remaining sub-tick time so a collinear G00 -> G80 junction does not insert a dwell.
double Cycle::AdvanceLeg(double seconds, bool brake, bool& reached) noexcept
{
    reached = false;
    for (unsigned iteration = 0; iteration < 8 && seconds > 1e-15; ++iteration)
    {
        const double remaining = std::max(0.0, legLength_ - legPosition_);
        if (remaining <= Tiny)
        {
            legPosition_ = legLength_;
            if (!brake) speed_ = exitSpeed_;
            else if (speed_ * speed_ / (2.0 * deceleration_) <= Tiny) speed_ = 0.0;
            else SetFault(Fault::SearchDistanceExceeded, true);
            reached = true; break;
        }
        double accel = 0.0, duration = seconds;
        if (brake)
        {
            if (speed_ <= Tiny) { speed_ = 0.0; break; }
            accel = -deceleration_;
            duration = std::min(duration, speed_ / deceleration_);
        }
        else
        {
            const double brakeDistance = std::max(0.0,
                (speed_ * speed_ - exitSpeed_ * exitSpeed_) / (2.0 * deceleration_));
            if (speed_ > exitSpeed_ + Tiny && remaining <= brakeDistance + 1e-11)
            {
                accel = -deceleration_;
                duration = std::min(duration, (speed_ - exitSpeed_) / deceleration_);
            }
            else if (speed_ > speedLimit_ + Tiny)
            {
                accel = -deceleration_;
                duration = std::min(duration, (speed_ - speedLimit_) / deceleration_);
            }
            else if (speed_ < speedLimit_ - Tiny)
            {
                const double peak = std::sqrt(std::max(0.0,
                    (2.0 * acceleration_ * deceleration_ * remaining +
                     deceleration_ * speed_ * speed_ + acceleration_ * exitSpeed_ * exitSpeed_) /
                    (acceleration_ + deceleration_)));
                if (peak <= speed_ + Tiny)
                { accel = -deceleration_; duration = std::min(duration, std::max(0.0, (speed_ - exitSpeed_) / deceleration_)); }
                else
                { accel = acceleration_; duration = std::min(duration, (std::min(speedLimit_, peak) - speed_) / acceleration_); }
            }
            else
            {
                const double cruise = speed_ > Tiny ? (remaining - brakeDistance) / speed_ : 0.0;
                duration = std::min(duration, std::max(0.0, cruise));
            }
        }
        if (duration <= 1e-15)
        {
            // At a profile event, selecting the following branch must make progress.
            if (speed_ <= Tiny) { speed_ = 0.0; break; }
            accel = speed_ > exitSpeed_ + Tiny ? -deceleration_ : 0.0;
            duration = seconds;
        }
        double distance = speed_ * duration + 0.5 * accel * duration * duration;
        if (distance >= remaining)
        {
            const double discriminant = std::max(0.0, speed_ * speed_ + 2.0 * accel * remaining);
            // Rationalized root is stable when acceleration or remaining is small.
            const double divisor = speed_ + std::sqrt(discriminant);
            duration = divisor > Tiny ? 2.0 * remaining / divisor : 0.0;
            duration = std::min(duration, seconds);
            distance = remaining;
            reached = true;
        }
        legPosition_ += std::max(0.0, distance);
        speed_ = std::max(0.0, speed_ + accel * duration);
        seconds = std::max(0.0, seconds - duration);
        if (reached)
        {
            legPosition_ = legLength_;
            if (!brake) speed_ = exitSpeed_;
            else if (speed_ * speed_ / (2.0 * deceleration_) <= Tiny) speed_ = 0.0;
            else SetFault(Fault::SearchDistanceExceeded, true);
            break;
        }
    }
    for (unsigned i = 0; i < AxisCount; ++i)
        output_.commandMCS[i] = legOrigin_[i] + legDirection_[i] * legPosition_;
    return seconds;
}

void Cycle::SetFault(Fault fault, bool emergency) noexcept
{
    if (feedback_.fault == Fault::None) feedback_.fault = fault;
    PrepareApproachBrake();
    output_.faulted = true;
    output_.emergencyStopRequired = output_.emergencyStopRequired || emergency;
    if (emergency)
    {
        speed_ = 0.0; output_.velocityNative.fill(0.0);
        output_.active = false; output_.stopped = true;
        feedback_.phase = Phase::Fault;
    }
    else if (speed_ > Tiny)
    { feedback_.phase = Phase::FaultBrake; output_.active = true; }
    else
    {
        speed_ = 0.0; output_.velocityNative.fill(0.0);
        feedback_.phase = Phase::Fault; output_.active = false; output_.stopped = true;
    }
}

void Cycle::UpdateSignal(const Input& in, bool fresh) noexcept
{
    if (!fresh) return;
    const double threshold = request_.config.contactVoltageV;
    const bool low = in.voltageV < threshold;
    const bool released = in.voltageV > threshold + request_.config.releaseHysteresisV;
    if (released)
    { if (releaseCount_ < request_.config.releaseStableSamples) ++releaseCount_; }
    else releaseCount_ = 0;
    if (feedback_.phase == Phase::Approach && low)
    { SetFault(Fault::ApproachContact); return; }
    if (feedback_.phase == Phase::InitialRelease && low)
    { SetFault(Fault::InitialContact); return; }
    if (feedback_.phase != Phase::FastSeek && feedback_.phase != Phase::SlowSeek) return;
    if (!low) { lowCount_ = 0; return; }
    if (lowCount_ == 0) {
        firstLowPose_ = in.actualMCS;
        // UpdateSignal precedes this tick's trajectory advance. Capture the
        // command paired with the first low input, not the later debounce/stop.
        firstLowCommand_ = output_.commandMCS;
    }
    if (lowCount_ < request_.config.contactStableSamples) ++lowCount_;
    if (lowCount_ >= request_.config.contactStableSamples)
    {
        feedback_.contact = true;
        feedback_.lastContactMCS = firstLowPose_;
        feedback_.lastCommandedContactMCS = firstLowCommand_;
        if (fastSeek_)
        { feedback_.fastCaptured = true; feedback_.fastContactMCS = firstLowPose_; }
        else if (!touchStored_)
        {
            touches_[feedback_.slowCount] = firstLowPose_;
            commandedTouches_[feedback_.slowCount] = firstLowCommand_;
            ++feedback_.slowCount; touchStored_ = true;
        }
        feedback_.phase = Phase::ContactBrake;
        phaseElapsed_ = 0.0;
        ResetSettle();
    }
}

void Cycle::ResetSettle() noexcept
{ settleStarted_ = false; settleElapsed_ = 0.0; }

bool Cycle::Settled(const Input& in, bool fresh) noexcept
{
    if (!fresh) return false;
    bool within = true;
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        const double window = i < 3 ? request_.config.settleWindowMm : request_.config.settleWindowDeg;
        if (std::abs(in.actualMCS[i] - output_.commandMCS[i]) > window) within = false;
    }
    if (!within) { ResetSettle(); return false; }
    if (!settleStarted_)
    { settleMin_ = in.actualMCS; settleMax_ = in.actualMCS; settleStarted_ = true; settleElapsed_ = 0.0; }
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        settleMin_[i] = std::min(settleMin_[i], in.actualMCS[i]);
        settleMax_[i] = std::max(settleMax_[i], in.actualMCS[i]);
        const double window = i < 3 ? request_.config.settleWindowMm : request_.config.settleWindowDeg;
        if (settleMax_[i] - settleMin_[i] > window)
        { settleMin_ = in.actualMCS; settleMax_ = in.actualMCS; settleElapsed_ = 0.0; return false; }
    }
    settleElapsed_ += SampleSeconds;
    return settleElapsed_ + Tiny >= request_.config.settleHoldSeconds;
}

bool Cycle::ComputeStatistics() noexcept
{
    const unsigned count = feedback_.slowCount;
    if (count == 0) return false;
    feedback_.meanMCS.fill(0.0); feedback_.meanAbsDeviation.fill(0.0);
    feedback_.meanCommandedMCS.fill(0.0);
    feedback_.maxDeviation.fill(0.0); feedback_.spread.fill(0.0);
    feedback_.linearMeanError = feedback_.rotaryMeanError = 0.0;
    feedback_.linearMaxDeviation = feedback_.rotaryMaxDeviation = 0.0;
    for (unsigned n = 0; n < count; ++n)
        for (unsigned i = 0; i < AxisCount; ++i) {
            feedback_.meanMCS[i] += touches_[n][i] / count;
            feedback_.meanCommandedMCS[i] += commandedTouches_[n][i] / count;
        }
    for (unsigned i = 0; i < AxisCount; ++i)
    {
        double lo = touches_[0][i], hi = lo;
        for (unsigned n = 0; n < count; ++n)
        {
            const double dev = std::abs(touches_[n][i] - feedback_.meanMCS[i]);
            feedback_.meanAbsDeviation[i] += dev / count;
            feedback_.maxDeviation[i] = std::max(feedback_.maxDeviation[i], dev);
            lo = std::min(lo, touches_[n][i]); hi = std::max(hi, touches_[n][i]);
        }
        feedback_.spread[i] = hi - lo;
    }
    for (unsigned n = 0; n < count; ++n)
    {
        Point6 dev{};
        for (unsigned i = 0; i < AxisCount; ++i)
            if (request_.direction[i] != 0.0) dev[i] = touches_[n][i] - feedback_.meanMCS[i];
        const double linear = Norm(dev, 0), rotary = Norm(dev, 3);
        feedback_.linearMeanError += linear / count; feedback_.rotaryMeanError += rotary / count;
        feedback_.linearMaxDeviation = std::max(feedback_.linearMaxDeviation, linear);
        feedback_.rotaryMaxDeviation = std::max(feedback_.rotaryMaxDeviation, rotary);
    }
    feedback_.statisticsValid = true;
    return feedback_.linearMeanError <= request_.config.maxMeanErrorMm + Tiny &&
        feedback_.rotaryMeanError <= request_.config.maxMeanErrorDeg + Tiny;
}

void Cycle::Finish() noexcept
{
    speed_ = 0.0; feedback_.phase = Phase::Complete;
    output_.active = false; output_.complete = true; output_.stopped = true;
    output_.velocityNative.fill(0.0);
}

void Cycle::RefreshOutput() noexcept
{
    if (output_.emergencyStopRequired)
    {
        speed_ = 0.0; output_.active = false; output_.complete = false;
        feedback_.phase = Phase::Fault;
    }
    output_.stopped = speed_ <= Tiny;
    for (unsigned i = 0; i < AxisCount; ++i) output_.velocityNative[i] = legDirection_[i] * speed_;
    output_.faulted = feedback_.fault != Fault::None;
}

Output Cycle::Step(const Input& in) noexcept
{
    if (!initialized_ || !output_.active) return output_;
    feedback_.tick = in.tick;
    if (in.hold) holdBraking_ = true;
    else if (speed_ <= Tiny) holdBraking_ = false;
    const bool held = in.hold || holdBraking_;
    feedback_.held = held;
    if (!in.pdoValid) SetFault(Fault::PdoLost, true);
    else if (!in.sourceValid || !Finite(in.actualMCS) || !std::isfinite(in.voltageV)) SetFault(Fault::SourceLost, true);
    else if (in.tick <= lastTick_ || in.nowUs < lastNowUs_) SetFault(Fault::NonMonotonicTick, true);
    else if (in.sourceSequence < lastSource_) SetFault(Fault::SourceSequenceRegression, true);
    if (!output_.active) return output_;
    const bool fresh = in.sourceSequence > lastSource_;
    lastTick_ = in.tick; lastSource_ = in.sourceSequence; lastNowUs_ = in.nowUs;
    if (!(held && speed_ <= Tiny))
    { elapsed_ += SampleSeconds; phaseElapsed_ += SampleSeconds; }
    if (in.cancel) SetFault(Fault::Cancelled);
    if (!InsideTravel(in.actualMCS)) SetFault(Fault::TravelLimit);
    if ((feedback_.phase == Phase::FastSeek || feedback_.phase == Phase::SlowSeek ||
         feedback_.phase == Phase::ContactBrake || feedback_.phase == Phase::ContactSettle) &&
        !InsideSearch(in.actualMCS)) SetFault(Fault::SearchDistanceExceeded);
    if (elapsed_ > request_.config.maxCycleSeconds) SetFault(Fault::CycleTimeout);
    if (!output_.active) return output_;
    UpdateSignal(in, fresh);
    if (!output_.active) return output_;
    bool reached = false;
    switch (feedback_.phase)
    {
    case Phase::InitialRelease:
        if (phaseElapsed_ > request_.config.releaseTimeoutSeconds) SetFault(Fault::ReleaseTimeout);
        else if (!held && releaseCount_ >= request_.config.releaseStableSamples)
        { if (request_.hasApproach) StartApproach(); else StartSeek(); }
        break;
    case Phase::Approach:
    {
        if (held) PrepareApproachBrake();
        else
        {
            if (legPosition_ >= approachLength_)
            {
                StartSeek(approachScale_ > 0.0 ? speed_ * approachScale_ : 0.0);
                if (output_.active) AdvanceLeg(SampleSeconds, false, reached);
                break;
            }
            legLength_ = approachLength_;
            exitSpeed_ = std::min(exitSpeed_, std::sqrt(speed_ * speed_ +
                2.0 * acceleration_ * (legLength_ - legPosition_)));
        }
        const double left = AdvanceLeg(SampleSeconds, held, reached);
        if (!output_.active) break;
        if (reached && !held)
        {
            const double junctionSpeed = approachScale_ > 0.0 ? speed_ * approachScale_ : 0.0;
            Point6 exitVelocity{};
            for (unsigned i = 0; i < AxisCount; ++i)
                exitVelocity[i] = legDirection_[i] * speed_;
            StartSeek(junctionSpeed);
            if (feedback_.fault == Fault::None && output_.active &&
                (feedback_.phase == Phase::FastSeek || feedback_.phase == Phase::SlowSeek))
            {
                feedback_.approachJunctionTick = in.tick;
                feedback_.approachExitVelocityNative = exitVelocity;
                for (unsigned i = 0; i < AxisCount; ++i)
                    feedback_.seekEntryVelocityNative[i] = legDirection_[i] * speed_;
                feedback_.approachJunctionValid = true;
            }
            if (output_.active && left > 0.0) AdvanceLeg(left, false, reached);
        }
        break;
    }
    case Phase::FastSeek:
    case Phase::SlowSeek:
        AdvanceLeg(SampleSeconds, held, reached);
        if (!output_.active) break;
        if (reached && !held) SetFault(seekTravelBound_ ? Fault::TravelLimit : Fault::SearchDistanceExceeded);
        break;
    case Phase::ContactBrake:
        AdvanceLeg(SampleSeconds, true, reached);
        if (!output_.active) break;
        if (speed_ <= Tiny && !held)
        { feedback_.phase = Phase::ContactSettle; phaseElapsed_ = 0.0; ResetSettle(); }
        break;
    case Phase::ContactSettle:
        if (phaseElapsed_ > request_.config.settleTimeoutSeconds) SetFault(Fault::SettleTimeout);
        else if (Settled(in, fresh) && !held)
        {
            if (fastSeek_) { fastSeek_ = false; finalRetreat_ = false; StartRetreat(); }
            else if (feedback_.slowCount < request_.config.slowTouches)
            { finalRetreat_ = false; StartRetreat(); }
            else if (!ComputeStatistics()) SetFault(Fault::MeanErrorExceeded);
            else if (request_.finalRetreat) { finalRetreat_ = true; StartRetreat(); }
            else Finish();
        }
        break;
    case Phase::Retreat:
        AdvanceLeg(SampleSeconds, held, reached);
        if (!output_.active) break;
        if (reached && !held)
        { feedback_.phase = Phase::ReleaseSettle; phaseElapsed_ = 0.0; ResetSettle(); }
        break;
    case Phase::ReleaseSettle:
    {
        const bool stable = Settled(in, fresh);
        if (releaseCount_ < request_.config.releaseStableSamples && phaseElapsed_ > request_.config.releaseTimeoutSeconds)
            SetFault(Fault::ReleaseTimeout);
        else if (!stable && phaseElapsed_ > request_.config.settleTimeoutSeconds) SetFault(Fault::SettleTimeout);
        else if (stable && releaseCount_ >= request_.config.releaseStableSamples && !held)
        { if (finalRetreat_) Finish(); else StartSeek(); }
        break;
    }
    case Phase::FaultBrake:
        AdvanceLeg(SampleSeconds, true, reached);
        if (speed_ <= Tiny) SetFault(feedback_.fault);
        break;
    default: break;
    }
    if (!InsideTravel(output_.commandMCS)) SetFault(Fault::TravelLimit, true);
    if ((feedback_.phase == Phase::FastSeek || feedback_.phase == Phase::SlowSeek ||
         feedback_.phase == Phase::ContactBrake) && !InsideSearch(output_.commandMCS))
        SetFault(Fault::SearchDistanceExceeded, true);
    RefreshOutput();
    return output_;
}
}
