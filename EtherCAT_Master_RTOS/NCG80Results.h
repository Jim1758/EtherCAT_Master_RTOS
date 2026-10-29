#pragma once
#include "NCG80Block.h"
#include "G80Cycle.h"
#include <array>
#include <atomic>
#include <cstdint>

namespace NCG80
{
constexpr int ResultFirst=701,ResultLast=899;
constexpr unsigned ResultCount=199U,SchemaMagic=27701U,SchemaVersion=1U;
using Results=std::array<double,ResultCount>;
enum CoordinateValid : std::uint32_t {
    ActualMeanMachine=1U,ActualMeanWork=2U,CommandedMeanMachine=4U,
    CommandedMeanWork=8U,RotationRemoved=16U,ToolRemoved=32U,BothRemoved=64U
};
struct Extension {
    std::uint32_t source=0U,simScenario=0U,coordinateValidMask=0U;
    std::uint64_t frameGeneration=0ULL,frameRevision=0ULL,inputSequence=0ULL;
    int wcsCode=0,rotationMode=0,workMode=0,toolLengthMode=0,hCode=0;
    double rotationAngle=0.0,simProjectedDistance=0.0,simPlaneDistance=0.0;
    bool held=false;
    G80::Point6 meanCommandedMCS{},meanCommandedWCS{},rotationRemovedMCS{},toolRemovedMCS{},bothRemovedMCS{};
};
inline Results BuildResults(const Program& p,const G80::Feedback& f,
    const G80::Point6& meanWCS,std::uint32_t machineMask,
    unsigned phase,unsigned fault,bool success,double voltage,
    std::uint64_t sequence,std::uint64_t session=0ULL,unsigned runtimeFault=0U,
    unsigned alarmCode=0U,bool active=false,const Extension& ext=Extension{}) noexcept
{
    Results r{};
    const auto put=[&r](int n,double v) noexcept {r[static_cast<unsigned>(n-ResultFirst)]=v;};
    put(701,SchemaMagic);put(702,SchemaVersion);put(703,ext.source);put(704,ext.simScenario);
    put(705,ext.coordinateValidMask);put(706,ext.held?1.0:0.0);
    put(708,static_cast<double>(ext.frameGeneration));put(709,static_cast<double>(ext.frameRevision));
    put(710,ext.wcsCode);put(711,ext.rotationMode);put(712,ext.workMode);
    put(713,ext.toolLengthMode);put(714,ext.hCode);put(715,ext.rotationAngle);
    put(790,ext.simProjectedDistance);put(791,ext.simPlaneDistance);
    put(792,static_cast<double>(ext.inputSequence));put(799,static_cast<double>(sequence));
    put(801,static_cast<double>(sequence));put(899,static_cast<double>(sequence));
    put(802,phase);put(803,fault);put(804,success?1.0:0.0);
    put(805,f.slowCount);put(806,f.targetCount);put(807,f.fastCaptured?1.0:0.0);
    put(808,f.linearMeanError);put(809,f.rotaryMeanError);put(810,f.linearMaxDeviation);
    put(811,p.authoredMask);put(812,machineMask);put(813,p.p);put(814,p.j);put(815,p.k);
    put(816,p.a);put(817,p.feedPercent);put(818,f.contact?1.0:0.0);put(819,voltage);
    put(890,static_cast<double>(session));put(891,f.rotaryMaxDeviation);
    put(892,runtimeFault);put(893,alarmCode);put(894,active?1.0:0.0);
    for(unsigned i=0;i<6U;++i){
        put(720+i,ext.meanCommandedMCS[i]);put(730+i,ext.meanCommandedWCS[i]);
        put(740+i,ext.rotationRemovedMCS[i]);put(750+i,ext.toolRemovedMCS[i]);put(760+i,ext.bothRemovedMCS[i]);
        put(820+i,f.meanMCS[i]);put(830+i,meanWCS[i]);put(840+i,f.lastContactMCS[i]);
        put(850+i,0.0);put(860+i,f.meanAbsDeviation[i]);put(870+i,f.maxDeviation[i]);
        put(880+i,f.fastContactMCS[i]);
    }
    return r;
}
inline void SetMachineDirection(Results& r,const G80::Point6& direction) noexcept
{for(unsigned i=0;i<6U;++i)r[850U+i-ResultFirst]=direction[i];}

// One supervisory writer owns both the NC image and its SHM mirror. A reader
// accepts only matching nonzero 799/801/899 before and after its full copy.
// Invalidate first; activate 801 last so in-progress payload never looks ready.
template<class Store> inline void PublishImage(const Results& image,Store store) noexcept
{
    const double sequence=image[801-ResultFirst];
    store(801,0.0);store(899,0.0);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    for(int n=ResultFirst;n<ResultLast;++n)
        if(n!=799&&n!=801)store(n,image[static_cast<unsigned>(n-ResultFirst)]);
    store(799,sequence);store(899,sequence);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    store(801,sequence);
}
}
