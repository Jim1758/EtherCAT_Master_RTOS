#pragma once
#include "EDMCoordinatedGapSource.h"
#include "EDMSixAxisGapModeCoordinator.h"

// Explicit unloaded diagnostic control source. Physical PDO acquisition stays
// independent. This observer supplies voltage; it cannot move or commit a path.
namespace EDM139
{
constexpr unsigned SelectedRowsMask=(1U<<2U)|(1U<<4U)|(1U<<8U)|(1U<<9U);
enum class Phase : std::uint8_t { Idle, Forward, Retreat, Recover, Complete };
struct Evidence
{
    EDM88::Frame controlFrame{};
    std::uint64_t tick=0ULL,observedCommitTick=0ULL;
    double anchorStationMm=0.0;
    unsigned row=0U,startedMask=0U,restoredMask=0U,completedMask=0U;
    unsigned backwardMask=0U,forwardMask=0U,backwardCrossings=0U,forwardCrossings=0U;
    unsigned retreatAxisMask=0U,returnAxisMask=0U;
    Phase phase=Phase::Idle;
    bool enabled=false,rowValid=false,faulted=false;
};
inline bool Selected(unsigned row) noexcept
{return row<16U&&(SelectedRowsMask&(1U<<row))!=0U;}
inline double HighVoltage(double referenceV) noexcept{return referenceV+40.0;}
inline double LowVoltage(double referenceV) noexcept{return (std::max)(0.0,referenceV-20.0);}
inline bool SameEvidence(const Evidence& a,const Evidence& b) noexcept
{
    return EDM88::SameFrame(a.controlFrame,b.controlFrame)&&a.tick==b.tick&&
        a.observedCommitTick==b.observedCommitTick&&a.anchorStationMm==b.anchorStationMm&&
        a.row==b.row&&a.startedMask==b.startedMask&&a.restoredMask==b.restoredMask&&
        a.completedMask==b.completedMask&&a.backwardMask==b.backwardMask&&a.forwardMask==b.forwardMask&&
        a.backwardCrossings==b.backwardCrossings&&a.forwardCrossings==b.forwardCrossings&&
        a.retreatAxisMask==b.retreatAxisMask&&a.returnAxisMask==b.returnAxisMask&&
        a.phase==b.phase&&a.enabled==b.enabled&&a.rowValid==b.rowValid&&a.faulted==b.faulted;
}
inline bool ValidEvidence(const Evidence& e,std::uint64_t session,double referenceV) noexcept
{
    if(!e.enabled||e.faulted||!session||!std::isfinite(referenceV)||referenceV<15.0||referenceV>200.0||
        !e.tick||!e.controlFrame.valid||!e.controlFrame.sequence||!e.controlFrame.capturedAtUs||
        !EDM88::SameIdentity(e.controlFrame.identity,EDM88::SimulatedSource(session).identity)||
        e.row>=16U||!std::isfinite(e.anchorStationMm)||e.anchorStationMm<0.0||
        (e.startedMask&~SelectedRowsMask)||(e.restoredMask&~e.startedMask)||
        (e.completedMask&~e.restoredMask)||(e.backwardMask&~e.startedMask)||
        (e.forwardMask&~e.backwardMask)||(e.completedMask&~e.forwardMask)||
        e.backwardCrossings>4U||e.forwardCrossings>e.backwardCrossings||
        e.retreatAxisMask>63U||e.returnAxisMask>63U||
        static_cast<unsigned>(e.phase)>static_cast<unsigned>(Phase::Complete))return false;
    const bool low=e.phase==Phase::Retreat;
    return e.controlFrame.voltageV==(low?LowVoltage(referenceV):HighVoltage(referenceV))&&
        ((!e.rowValid||!Selected(e.row))?e.phase==Phase::Idle:e.phase!=Phase::Idle);
}
class Stimulus
{
public:
    bool Begin(std::uint64_t session,double referenceV) noexcept
    {
        *this=Stimulus{};
        if(!session||!std::isfinite(referenceV)||referenceV<15.0||referenceV>200.0)return Fail();
        session_=session;referenceV_=referenceV;e_.enabled=true;return true;
    }
    bool Enabled() const noexcept{return e_.enabled;}
    const Evidence& Current() const noexcept{return e_;}
    bool Matches(std::uint64_t session,double referenceV) const noexcept
    {return e_.enabled&&!e_.faulted&&session_==session&&referenceV_==referenceV;}
    bool CurrentAt(std::uint64_t session,double referenceV,std::uint64_t tick,std::uint64_t nowUs) const noexcept
    {return Matches(session,referenceV)&&ValidEvidence(e_,session,referenceV)&&e_.tick==tick&&e_.controlFrame.capturedAtUs==nowUs;}
    bool BeginRow(unsigned row) noexcept
    {
        if(!e_.enabled||e_.faulted||row>=16U||(!e_.rowValid&&row!=0U)||
            (e_.rowValid&&(row!=e_.row+1U||e_.phase==Phase::Retreat||e_.phase==Phase::Recover||
                (Selected(e_.row)&&e_.phase!=Phase::Complete))))return Fail();
        if(pendingRowValid_)return Fail();
        pendingRow_=row;pendingRowValid_=true;return true;
    }
    bool Observe(const EDM126::Feedback& f,EDM128::Phase coordination,
        std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!e_.enabled||e_.faulted||!tick||!nowUs||
            (e_.tick&&(tick!=e_.tick+1ULL||nowUs<=e_.controlFrame.capturedAtUs))||
            e_.controlFrame.sequence==(std::numeric_limits<std::uint64_t>::max)())return Fail();
        if(pendingRowValid_) {
            e_.row=pendingRow_;e_.rowValid=true;e_.anchorStationMm=0.0;
            e_.phase=Selected(e_.row)?Phase::Forward:Phase::Idle;
            previousValid_=false;pendingRowValid_=false;
        }
        if(e_.rowValid) {
            if(!f.active||f.faulted||!f.tick||f.tick>tick||!f.nowUs||f.nowUs>nowUs||
                f.authorizedSource.scope.session!=session_||f.cursorSource.scope.session!=session_||
                f.authorizedSource.ordinal!=e_.row||f.cursorSource.ordinal>e_.row||
                f.historyDepth!=e_.row-f.cursorSource.ordinal||!EDM126::Positive(f.lengthMm)||
                !std::isfinite(f.localMm)||f.localMm<0.0||f.localMm>f.lengthMm||
                !std::isfinite(f.stationMm)||f.stationMm<0.0)return Fail();
            const bool gap=coordination==EDM128::Phase::Gap;
            const unsigned bit=1U<<e_.row;
            if(Selected(e_.row)&&gap&&previousValid_&&previousPhase_==EDM128::Phase::Gap) {
                if(f.backwardCrossings<previous_.backwardCrossings||f.forwardCrossings<previous_.forwardCrossings)return Fail();
                const auto back=f.backwardCrossings-previous_.backwardCrossings;
                const auto forward=f.forwardCrossings-previous_.forwardCrossings;
                if(back>1U||forward>1U)return Fail();
                if(back) {if(e_.phase!=Phase::Retreat||(e_.backwardMask&bit))return Fail();e_.backwardMask|=bit;++e_.backwardCrossings;}
                if(forward) {if(e_.phase!=Phase::Recover||!(e_.backwardMask&bit)||(e_.forwardMask&bit))return Fail();e_.forwardMask|=bit;++e_.forwardCrossings;}
                unsigned moved=0U;
                for(unsigned axis=0U;axis<6U;++axis)if(f.positionPulse[axis]!=previous_.positionPulse[axis])moved|=1U<<axis;
                if(e_.phase==Phase::Retreat&&f.velocityMmSec<0.0)e_.retreatAxisMask|=moved;
                if(e_.phase==Phase::Recover&&f.velocityMmSec>0.0)e_.returnAxisMask|=moved;
            }
            if(Selected(e_.row)&&gap) {
                if(e_.phase==Phase::Forward&&!f.historyDepth&&f.localMm>=0.1*f.lengthMm) {
                    if(!f.historyCount)return Fail();
                    e_.anchorStationMm=f.stationMm;e_.startedMask|=bit;e_.phase=Phase::Retreat;
                } else if(e_.phase==Phase::Retreat&&f.historyDepth==1U&&f.localMm<=0.8*f.lengthMm) {
                    if(!(e_.backwardMask&bit))return Fail();
                    e_.restoredMask|=bit;e_.phase=Phase::Recover;
                } else if(e_.phase==Phase::Recover&&!f.historyDepth&&f.stationMm>=e_.anchorStationMm) {
                    if(!(e_.forwardMask&bit))return Fail();
                    e_.completedMask|=bit;e_.phase=Phase::Complete;
                }
            }
            previous_=f;previousPhase_=coordination;previousValid_=true;e_.observedCommitTick=f.tick;
        }
        e_.tick=tick;e_.controlFrame.identity=EDM88::SimulatedSource(session_).identity;
        ++e_.controlFrame.sequence;e_.controlFrame.capturedAtUs=nowUs;e_.controlFrame.valid=true;
        e_.controlFrame.voltageV=e_.phase==Phase::Retreat?LowVoltage(referenceV_):HighVoltage(referenceV_);
        return ValidEvidence(e_,session_,referenceV_);
    }
private:
    bool Fail() noexcept{e_.faulted=true;return false;}
    Evidence e_{};EDM126::Feedback previous_{};
    std::uint64_t session_=0ULL;double referenceV_=0.0;
    EDM128::Phase previousPhase_=EDM128::Phase::Gap;bool previousValid_=false;
    unsigned pendingRow_=0U;bool pendingRowValid_=false;
};
}
