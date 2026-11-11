#pragma once
#include "EDMAxisGainProfiles.h"
#include "NCG76SphereCycle.h"

// EDM382: immutable startup banks only for the explicit physical GAP parent.
// EDM384 extends the same immutable bank to installed XYZ/C/U/V native slots.
// Absent axes require no gain resolution and can never select a bank.
// Neither selecting a bank nor validating these values grants motion authority.
namespace EDM382Gains {
enum class Bank : std::uint8_t {None,Discharge,Flush};
inline Bank Resolve(const NCG76SphereCycle::Feedback& f) noexcept {
 using Phase=NCG76SphereCycle::Phase;
 if(f.fault!=NCG76SphereCycle::Fault::None||f.holdActive||f.held||f.inspectionJogActive)return Bank::None;
 if(f.flushActive){switch(f.phase){
  case Phase::FlushBrake:case Phase::FlushSettle:case Phase::FlushLift:case Phase::FlushReturn:
  case Phase::FlushToCenter:case Phase::FlushFromCenter:case Phase::FlushPathRetreat:case Phase::FlushPathReturn:
   return Bank::Flush;
  default:return Bank::None;
 }}
 switch(f.phase){case Phase::InitialDepth:case Phase::Settle:case Phase::GapHistorySettle:case Phase::GapHistory:
  return Bank::Discharge;
 default:return Bank::None;}
}
class BootBanks {
public:
 bool Configure(const std::array<EDM20::GainResolution,8U>& discharge,
     const std::array<EDM20::GainResolution,8U>& flush,std::uint32_t axisMask=7U) noexcept {
  if(attempted_)return false;
  attempted_=true;
  if((axisMask&7U)!=7U||(axisMask&~0x3fU)!=0U)return false;
  for(unsigned axis=0;axis<6;++axis)if((axisMask&(1U<<axis))&&(discharge[axis].status!=EDM20::GainResolveStatus::Ready||
      flush[axis].status!=EDM20::GainResolveStatus::Ready||!EDM20::ValidateAxisGains(discharge[axis].gains)||
      !EDM20::ValidateAxisGains(flush[axis].gains)))return false;
  for(unsigned axis=0;axis<6;++axis)if(axisMask&(1U<<axis)){discharge_[axis]=discharge[axis].gains;flush_[axis]=flush[axis].gains;}
  mask_=axisMask;configured_=true;return true;
 }
 bool Configured()const noexcept{return configured_;}
 std::uint32_t ConfiguredMask()const noexcept{return configured_?mask_:0U;}
 const EDM20::AxisGains* Select(Bank bank,unsigned axis)const noexcept {
  if(!configured_||axis>=6U||!(mask_&(1U<<axis)))return nullptr;
  return bank==Bank::Discharge?&discharge_[axis]:(bank==Bank::Flush?&flush_[axis]:nullptr);
 }
private:
 std::array<EDM20::AxisGains,6U> discharge_{},flush_{};std::uint32_t mask_=0U;bool configured_=false,attempted_=false;
};
}
