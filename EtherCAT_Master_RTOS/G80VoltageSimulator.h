#pragma once
#include "G80Cycle.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace G80
{
// Explicit test source only. This value never creates a physical ADC receipt.
enum class SimulationScenario : std::uint32_t
{
    Normal=0U, NoContact=1U, StuckContact=2U, ExcessSpread=3U, SourceLoss=4U
};
struct SimulationSettings
{
    std::uint32_t scenario=0U;
    double edgeDistanceMm=0.5,edgeDistanceDeg=0.5;
    // Alternating +amplitude/-amplitude on slow touches; peak-to-peak is 2x.
    double spreadMm=0.01,spreadDeg=0.01;
    double sourceLossDistanceMm=0.25,sourceLossDistanceDeg=0.25;
};
struct SimulationSample
{
    bool valid=false,contact=false;
    double voltageV=0.0,projectedDistance=0.0,planeDistance=0.0;
    std::uint64_t sequence=0ULL,session=0ULL;
    std::uint32_t scenario=0U;
};

// One RT owner, one frozen G80 session. Voltage depends on actual native pose
// against a virtual plane, never elapsed time, planned pose, or an NC position
// cache. XYZ projection is mm; pure CUV projection is degrees. A mixed request
// uses its XYZ plane while CUV still follows the real G80 motion/axis guards.
class VoltageSimulator
{
public:
    bool Begin(const Request& request,const SimulationSettings& settings) noexcept
    {
        // Do not let a second configuration replace an admitted live source.
        if(initialized_ || !request.session || request.session<=sessionHighWater_) return false;
        Reset();sessionHighWater_=request.session;
        current_.session=request.session;current_.scenario=settings.scenario;
        if(!request.session || settings.scenario>static_cast<std::uint32_t>(SimulationScenario::SourceLoss) ||
            !Positive(settings.edgeDistanceMm) || !Positive(settings.edgeDistanceDeg) ||
            !Nonnegative(settings.spreadMm) || !Nonnegative(settings.spreadDeg) ||
            !Positive(settings.sourceLossDistanceMm) || !Positive(settings.sourceLossDistanceDeg) ||
            !Positive(request.config.contactVoltageV) || !Nonnegative(request.config.releaseHysteresisV) ||
            !Positive(request.config.retreatMm) || !Positive(request.config.retreatDeg) ||
            !Positive(request.config.maxSearchMm) || !Positive(request.config.maxSearchDeg) ||
            request.config.slowTouches==0U || request.config.slowTouches>MaximumSlowTouches)
            return false;
        Point6 direction{};
        double largest=0.0;
        for(unsigned i=0U;i<AxisCount;++i) {
            if(!std::isfinite(request.direction[i]) || !std::isfinite(request.startMCS[i]) ||
                (request.hasApproach && !std::isfinite(request.approachEndMCS[i]))) return false;
            largest=(std::max)(largest,std::fabs(request.direction[i]));
        }
        if(!Positive(largest)) return false;
        for(unsigned i=0U;i<AxisCount;++i) direction[i]=request.direction[i]/largest;
        const double linear=Norm(direction,0U),rotary=Norm(direction,3U);
        if(!std::isfinite(linear) || !std::isfinite(rotary) || (linear==0.0 && rotary==0.0)) return false;
        first_=linear>0.0?0U:3U;
        const double selectedNorm=first_==0U?linear:rotary;
        for(unsigned i=first_;i<first_+3U;++i) normal_[i]=direction[i]/selectedNorm;
        origin_=request.hasApproach?request.approachEndMCS:request.startMCS;
        edge_=first_==0U?settings.edgeDistanceMm:settings.edgeDistanceDeg;
        spread_=first_==0U?settings.spreadMm:settings.spreadDeg;
        lossDistance_=first_==0U?settings.sourceLossDistanceMm:settings.sourceLossDistanceDeg;
        double range=first_==0U?request.config.maxSearchMm:request.config.maxSearchDeg;
        double retreat=first_==0U?request.config.retreatMm:request.config.retreatDeg;
        if(first_==0U && rotary>0.0) {
            // The engine's common scalar retreat/search is limited by both
            // native families. A tiny XYZ:C ratio cannot hide a short retreat.
            range=(std::min)(range,(request.config.maxSearchDeg/rotary)*linear);
            retreat=(std::min)(retreat,(request.config.retreatDeg/rotary)*linear);
        }
        if(!Positive(range) || !Positive(retreat)) return false;
        scenario_=static_cast<SimulationScenario>(settings.scenario);
        if(scenario_!=SimulationScenario::NoContact && scenario_!=SimulationScenario::SourceLoss &&
            !(edge_<range)) return false;
        if(scenario_==SimulationScenario::ExcessSpread &&
            (request.config.slowTouches<2U || !Positive(spread_) ||
             !std::isfinite(2.0*spread_) || !(2.0*spread_<retreat) ||
             !(spread_<edge_) || !std::isfinite(edge_+spread_) || !(edge_+spread_<range))) return false;
        // This case tests release failure; a one-touch K0 cycle never releases.
        if(scenario_==SimulationScenario::StuckContact && !request.fastFirst &&
            request.config.slowTouches==1U && !request.finalRetreat) return false;
        // Guarantee loss occurs during forward seek before a normal contact
        // can stop advancement, rather than a setting that can never trigger.
        if(scenario_==SimulationScenario::SourceLoss &&
            (!(lossDistance_<edge_) || !(lossDistance_<range))) return false;
        low_=request.config.contactVoltageV*0.5;
        const double release=request.config.contactVoltageV+request.config.releaseHysteresisV;
        high_=release+(std::max)(1.0,request.config.contactVoltageV*0.1);
        if(!std::isfinite(release) || !std::isfinite(high_) || !(high_>release) ||
            !std::isfinite(low_) || !(low_<request.config.contactVoltageV)) return false;
        targetTouches_=request.config.slowTouches;
        plane_=edge_;current_.planeDistance=plane_;
        current_.voltageV=high_;initialized_=true;
        return true;
    }

    SimulationSample Sample(const Input& actual,const Feedback& cycle) noexcept
    {
        if(!initialized_ || failed_) return current_;
        if(!actual.sourceValid || !actual.pdoValid || !actual.tick || !actual.nowUs ||
            (cycle.session!=0ULL && cycle.session!=current_.session) || cycle.slowCount>targetTouches_)
            return Fail();
        for(double value:actual.actualMCS) if(!std::isfinite(value)) return Fail();
        if(haveSample_) {
            if(actual.tick==lastInput_.tick) {
                // Re-reading the same PDO is idempotent; it cannot create a
                // fresh voltage sample or a second debounce observation.
                if(actual.nowUs!=lastInput_.nowUs ||
                    std::memcmp(actual.actualMCS.data(),lastInput_.actualMCS.data(),sizeof(Point6))!=0)
                    return Fail();
                return current_;
            }
            if(lastInput_.tick==(std::numeric_limits<std::uint64_t>::max)() ||
                actual.tick!=lastInput_.tick+1ULL || actual.nowUs<=lastInput_.nowUs ||
                actual.nowUs-lastInput_.nowUs>5000ULL) return Fail();
        }
        if(current_.sequence==(std::numeric_limits<std::uint64_t>::max)()) return Fail();
        if(scenario_==SimulationScenario::ExcessSpread && cycle.phase==Phase::SlowSeek &&
            previousPhase_!=Phase::SlowSeek) {
            if(cycle.slowCount>=targetTouches_) return Fail();
            // A slow touch is counted in ContactBrake; changing a plane there
            // would fake release. Only the next real SlowSeek entry changes it.
            plane_=edge_+((cycle.slowCount&1U)==0U?spread_:-spread_);
        }
        double projection=0.0;
        for(unsigned i=first_;i<first_+3U;++i) {
            const double offset=actual.actualMCS[i]-origin_[i];
            const double term=offset*normal_[i];
            if(!std::isfinite(offset) || !std::isfinite(term)) return Fail();
            projection+=term;
        }
        if(!std::isfinite(projection)) return Fail();
        lastInput_=actual;haveSample_=true;previousPhase_=cycle.phase;
        ++current_.sequence;
        current_.projectedDistance=projection;current_.planeDistance=plane_;
        if(scenario_==SimulationScenario::SourceLoss && projection>=lossDistance_) return Fail();
        bool contact=projection>=plane_;
        if(scenario_==SimulationScenario::NoContact) contact=false;
        if(scenario_==SimulationScenario::StuckContact) {
            stuck_=stuck_ || contact;contact=stuck_;
        }
        current_.contact=contact;current_.voltageV=contact?low_:high_;current_.valid=true;
        return current_;
    }

    // Retiring a source does not authorize replaying the same session id.
    void Reset() noexcept
    {const auto highWater=sessionHighWater_;*this=VoltageSimulator{};sessionHighWater_=highWater;}
    const SimulationSample& Current() const noexcept { return current_; }

private:
    static bool Positive(double value) noexcept {return std::isfinite(value) && value>0.0;}
    static bool Nonnegative(double value) noexcept {return std::isfinite(value) && value>=0.0;}
    static double Norm(const Point6& point,unsigned first) noexcept
    {return std::hypot(std::hypot(point[first],point[first+1U]),point[first+2U]);}
    SimulationSample Fail() noexcept
    {failed_=true;current_.valid=false;return current_;}
    SimulationSample current_{};
    Input lastInput_{};
    Point6 origin_{},normal_{};
    SimulationScenario scenario_=SimulationScenario::Normal;
    Phase previousPhase_=Phase::Idle;
    double edge_=0.0,spread_=0.0,lossDistance_=0.0,plane_=0.0,low_=0.0,high_=0.0;
    std::uint64_t sessionHighWater_=0ULL;
    unsigned first_=0U;
    std::uint32_t targetTouches_=0U;
    bool initialized_=false,failed_=false,haveSample_=false,stuck_=false;
};
}
