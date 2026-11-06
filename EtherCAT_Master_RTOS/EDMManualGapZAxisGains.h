#pragma once
#include "EDMAxisGainProfiles.h"
#include "NCG76SphereCycle.h"

// EDM382: immutable startup banks only for the explicit physical GAP parent.
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
     const std::array<EDM20::GainResolution,8U>& flush) noexcept {
  if(attempted_)return false;
  attempted_=true;
  for(unsigned axis=0;axis<3;++axis)if(discharge[axis].status!=EDM20::GainResolveStatus::Ready||
      flush[axis].status!=EDM20::GainResolveStatus::Ready||!EDM20::ValidateAxisGains(discharge[axis].gains)||
      !EDM20::ValidateAxisGains(flush[axis].gains))return false;
  for(unsigned axis=0;axis<3;++axis){discharge_[axis]=discharge[axis].gains;flush_[axis]=flush[axis].gains;}
  configured_=true;return true;
 }
 bool Configured()const noexcept{return configured_;}
 const EDM20::AxisGains* Select(Bank bank,unsigned axis)const noexcept {
  if(!configured_||axis>=3)return nullptr;
  return bank==Bank::Discharge?&discharge_[axis]:(bank==Bank::Flush?&flush_[axis]:nullptr);
 }
private:
 std::array<EDM20::AxisGains,3U> discharge_{},flush_{};bool configured_=false,attempted_=false;
};
}
