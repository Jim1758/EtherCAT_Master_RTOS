#pragma once
#include "EDMGapPhysicalSource.h"
#include "EDMCoordinatedGapSource.h"

// EDM382: acquisition-only, session-owned raw ADC adapter. It deliberately
// does not consume the diagnostic EDM64 curve's output/stop state.
namespace EDM382Source {
constexpr std::uint64_t MaximumAgeUs=1500ULL;
class Source {
public:
 bool Begin(const EDMGapInput::Profile& profile,const EDMAnalogInput::Snapshot& baseline,
     bool read,std::uint64_t session,std::uint64_t nowUs,std::uint64_t frequency) noexcept {
  if(!session||session<=highWater_)return Fail();
  highWater_=session;active_=ready_=false;frame_={};config_={};
  if(!read||!EDM63::ValidProfile(profile)||EDM63::SampleStatus(baseline,true)!=EDM63::Reason::None||
     !baseline.capturedAtUs||nowUs<baseline.capturedAtUs||nowUs-baseline.capturedAtUs>MaximumAgeUs)return Fail();
  auto admission=EDM63::MakeAdmission(profile,baseline,session,nowUs,frequency);
  if(!adapter_.Begin(admission))return Fail();
  config_.enabled=true;config_.identity.kind=EDM88::SourceKind::Physical;
  config_.identity.producerGeneration=baseline.producerGeneration;
  config_.identity.deviceId=baseline.deviceId;config_.identity.channelId=baseline.channelId;
  config_.identity.configRevision=profile.profileRevision;config_.identity.calibrationRevision=profile.calibrationRevision;
  config_.samplePeriodUs=250ULL;config_.maxAgeUs=MaximumAgeUs;
  if(!EDM88::ValidPhysical(config_))return Fail();
  baselineCaptured_=baseline.capturedAtUs;active_=true;return true;
 }
 bool Observe(const EDMAnalogInput::Snapshot& sample,bool read,std::uint64_t nowUs,
     bool clockValid,std::uint64_t frequency) noexcept {
  if(!active_)return false;
  const auto old=ready_?frame_.capturedAtUs:baselineCaptured_;
  if(!clockValid||nowUs<old||nowUs-old>MaximumAgeUs)return Fail();
  const auto receipt=adapter_.Observe(sample,read,nowUs,clockValid,frequency);
  if(receipt.state==EDM63::State::AwaitFresh)return true;
  if(!receipt.Usable()||receipt.reason!=EDM63::Reason::None||!receipt.ageKnown||receipt.ageUs>MaximumAgeUs)
   return Fail();
  frame_.identity=config_.identity;frame_.sequence=sample.sequence;frame_.capturedAtUs=sample.capturedAtUs;
  frame_.voltageV=static_cast<double>(receipt.conversion.voltageMv)/1000.0;frame_.valid=true;
  ready_=true;return true;
 }
 bool CurrentAt(std::uint64_t nowUs,std::uint64_t frequency) const noexcept {
  return active_&&ready_&&frame_.valid&&frequency==adapter_.Snapshot().sample.qpcFrequency&&
   nowUs>=frame_.capturedAtUs&&nowUs-frame_.capturedAtUs<=MaximumAgeUs;
 }
 bool Ready()const noexcept{return active_&&ready_;}
 const EDM88::Frame& CurrentFrame()const noexcept{return frame_;}
 const EDM88::SourceConfig& Config()const noexcept{return config_;}
 void Clear() noexcept {adapter_.Cancel();active_=ready_=false;frame_={};config_={};}
private:
 bool Fail()noexcept{active_=ready_=false;frame_.valid=false;return false;}
 EDM63::PhysicalAdapter adapter_{};EDM88::SourceConfig config_{};EDM88::Frame frame_{};
 std::uint64_t highWater_=0,baselineCaptured_=0;bool active_=false,ready_=false;
};
}
