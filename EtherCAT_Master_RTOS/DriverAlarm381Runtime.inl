// Included exactly once in System_EDM_SINKER_MODE.cpp. Numeric RT capture;
// sparse trace formatting is restricted to the existing P50 supervisor.
// Runtime producers: engineering bridge or this monitor, under explicit ownership.
// Legacy blocking producers execute only during startup, before Initialize().
#include "DriverAlarm381.h"
#include "EtherCatPdoRuntimeInvalidCorrelation.h"
#include <atomic>
#include <cmath>
#include <cstring>
namespace DriverAlarm381 {
namespace {
struct RtObservation {
 std::uint64_t tick;
 std::int32_t lrw,expected,dc;
 std::uint32_t valid,count,quiet,owner;
 std::uint32_t status[AxisCapacity],statusValid[AxisCapacity];
};
static_assert(sizeof(RtObservation)%8==0,"atomic observation words");
constexpr std::size_t RtWords=sizeof(RtObservation)/8;
std::atomic<std::uint32_t> rtSequence{0};
std::atomic<bool> captureEnabled{false};
std::atomic<std::uint64_t> rtWords[RtWords]{};
static_assert(ATOMIC_LLONG_LOCK_FREE==2,"x64 lock-free diagnostic words required");
HANDLE mappingHandle=nullptr;
Image* mapping=nullptr;
Image cache{};
RtObservation latest{};
struct RowState {
 std::uint64_t due=0,generation=0,lastFailureLog=0,lastSubmitEpisode=0,lastResultEpisode=0;
 std::uint32_t lastFailureReason=0,lastResultCode=0,lastSubmitStatus=0,lastResultStatus=0;
 bool fault=false,warning=false,wasValid=false,submitLogged=false,resultLogged=false,failureLogged=false;
} rowState[AxisCapacity]{};
bool initialized=false,sourceAdvanced=false,owned=false,timedOut=false;
std::uint32_t activeAxis=0,roundRobin=0;
std::uint64_t lastRtTick=0,lastRtAdvance=0,quietSince=0,lastStart=0,startMs=0,activeGeneration=0,activeEpisode=0;
std::uint32_t activeStatus=0;
std::uint32_t lastDeferReason=0,lastDeferOwner=0,lastDeferNc=0;
std::uint64_t lastDeferLog=0,lastAlarmSummary=0;
bool deferPending=false,deferLogged=false,alarmSummaryLogged=false;
bool clockObserved=false,clockUnavailableLogged=false,quietObserved=false;
std::uint64_t lastClockMs=0;
// Use the same validated RTX QPC clock as the P50 health publisher. A
// Windows GetTickCount64 value is not a valid RTSS scheduling clock here.
bool ReadNow(std::uint64_t& now) noexcept {
 if(!RuntimeLogNowMs(now)||(clockObserved&&now<lastClockMs))return false;
 clockObserved=true;lastClockMs=now;return true;
}
// No trace helper is called by CaptureRt or the EtherCAT/PDO owner.
void TraceDeferred(EtherCatMaster* m,std::uint64_t now,std::uint32_t reason) noexcept {
 const auto nc=m->m_NC?static_cast<std::uint32_t>(m->m_NC->GetState()):0xffffffffU;
 const auto owner=static_cast<std::uint32_t>(m->m_Motion.GetMotionOwner());
 if(reason!=lastDeferReason||owner!=lastDeferOwner||nc!=lastDeferNc)deferPending=true;
 lastDeferReason=reason;lastDeferOwner=owner;lastDeferNc=nc;
 if(!deferPending||(deferLogged&&now-lastDeferLog<5000ULL))return;
 deferPending=false;deferLogged=true;lastDeferLog=now;
 RtPrintf("[EDM381-FIX2] event=DEFER reason=%u nc=%u owner=%u quiet=%u source=%u pdo=%u slot=%d queries=%u nowMs=%llu rtTick=%llu clock=RTX_QPC authority=READ_ONLY\n",
  reason,nc,owner,(cache.header.flags&CommandQuiet)?1U:0U,(cache.header.flags&SourceCoherent)?1U:0U,
  (cache.header.flags&HeaderPdoValid)?1U:0U,m->m_asyncCmd.status,cache.header.queryCount,
  static_cast<unsigned long long>(now),static_cast<unsigned long long>(cache.header.sourcePdoTick));
}
void TraceAlarmSummary(EtherCatMaster* m,std::uint64_t now) noexcept {
 if(!m->m_NC||m->m_NC->GetState()!=NCState::ALARM) { alarmSummaryLogged=false;return; }
 if(alarmSummaryLogged&&now-lastAlarmSummary<10000ULL)return;
 alarmSummaryLogged=true;lastAlarmSummary=now;
 RtPrintf("[EDM381-FIX2] event=ALARM_SUMMARY nc=%u owner=%u quiet=%u source=%u pdo=%u owned=%u slot=%d axes=%u queries=%u failures=%u states=%u/%u/%u/%u/%u/%u/%u/%u nowMs=%llu rtTick=%llu clock=RTX_QPC authority=READ_ONLY\n",
  static_cast<std::uint32_t>(m->m_NC->GetState()),static_cast<std::uint32_t>(m->m_Motion.GetMotionOwner()),
  (cache.header.flags&CommandQuiet)?1U:0U,(cache.header.flags&SourceCoherent)?1U:0U,(cache.header.flags&HeaderPdoValid)?1U:0U,
  owned?1U:0U,m->m_asyncCmd.status,cache.header.axisCount,cache.header.queryCount,cache.header.queryFailures,
  cache.rows[0].queryState,cache.rows[1].queryState,cache.rows[2].queryState,cache.rows[3].queryState,
  cache.rows[4].queryState,cache.rows[5].queryState,cache.rows[6].queryState,cache.rows[7].queryState,
  static_cast<unsigned long long>(now),static_cast<unsigned long long>(cache.header.sourcePdoTick));
}
bool ReadRt(RtObservation& target) noexcept {
 for(unsigned attempt=0;attempt<3;++attempt) {
  auto first=rtSequence.load(std::memory_order_acquire);if(first&1U)continue;
  std::uint64_t words[RtWords]{};
  for(std::size_t i=0;i<RtWords;++i)words[i]=rtWords[i].load(std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_acquire);
  if(first==rtSequence.load(std::memory_order_acquire)) { std::memcpy(&target,words,sizeof(target));return true; }
 }
 return false;
}
void Publish() noexcept {
 if(!mapping)return;
 ++cache.header.heartbeat;
 const auto odd=(mapping->header.sequence+1U)|1U;
 InterlockedExchange(reinterpret_cast<volatile LONG*>(&mapping->header.sequence),static_cast<LONG>(odd));MemoryBarrier();
 // Do not overwrite the publication fence during the bounded copy.
 std::memcpy(mapping,&cache,offsetof(Header,sequence));
 std::memcpy(reinterpret_cast<unsigned char*>(mapping)+offsetof(Header,runtimeId),
             reinterpret_cast<const unsigned char*>(&cache)+offsetof(Header,runtimeId),sizeof(Image)-offsetof(Header,runtimeId));
 MemoryBarrier();InterlockedExchange(reinterpret_cast<volatile LONG*>(&mapping->header.sequence),static_cast<LONG>(odd+1U?odd+1U:2U));
}
bool HasA3Name(const char* name) noexcept {
 char folded[129]{};for(unsigned i=0;i<128&&name[i];++i) { char c=name[i];folded[i]=(c>='a'&&c<='z')?static_cast<char>(c-32):c; }
 return std::strstr(folded,"A3-E")!=nullptr||std::strstr(folded,"A3E")!=nullptr;
}
void Failure(AxisRow& row,std::uint32_t reason,std::uint64_t now) noexcept {
 row.flags&=~NativeCurrent;row.readFailureReason=reason;row.queryState=ReadFailed;++cache.header.queryFailures;
 auto& state=rowState[row.axisIndex];const auto n=row.attemptCount;state.due=now+(n>=3?30000ULL:(n<=1?1000ULL:4000ULL));
 if(!state.failureLogged||(now-state.lastFailureLog>=1000ULL&&
    (reason!=state.lastFailureReason||now-state.lastFailureLog>=30000ULL))) {
  state.failureLogged=true;state.lastFailureReason=reason;state.lastFailureLog=now;
  RtPrintf("[EDM381-FIX2] event=READ_FAILED axis=%u slave=%u reason=%u attempt=%u lastTerminalWkc=%d lastAbort=0x%08X nextMs=%llu authority=READ_ONLY\n",
   row.axisIndex,row.slavePosition,reason,row.attemptCount,row.lastWkc,row.abortCode,
   static_cast<unsigned long long>(state.due-now));
 }
}
bool SameSlot(const EtherCatMaster* m) noexcept {
 const auto& c=m->m_asyncCmd;const auto& row=cache.rows[activeAxis];
 return c.type==static_cast<int>(EcatCmdType::CMD_SDO_READ)&&c.slaveAddr==row.slavePosition&&c.index==0x2001&&c.subIndex==0;
}
void Poll(EtherCatMaster* m,std::uint64_t now) noexcept {
 if(!owned)return;
 auto& row=cache.rows[activeAxis];auto& command=m->m_asyncCmd;
 const int status=command.status;
 if(status==static_cast<int>(EcatCmdStatus::ECAT_STATUS_PENDING)) {
  if(!timedOut&&now-startMs>=1500ULL) { timedOut=true;Failure(row,Timeout,now); }
  return; // The PDO owner must finish naturally; ownership is retained while draining.
 }
 if(status!=static_cast<int>(EcatCmdStatus::ECAT_STATUS_DONE)&&status!=static_cast<int>(EcatCmdStatus::ECAT_STATUS_ERROR)) {
  // No runtime producer is permitted to replace this owned slot. Never reclaim an unknown slot.
  Failure(row,IdentityMismatch,now);return;
 }
 const bool matches=SameSlot(m);
 const bool sameEpisode=rowState[activeAxis].generation==activeGeneration&&row.faultEpisode==activeEpisode&&
  (row.flags&(Fault|Warning))==activeStatus&&(row.flags&(PdoValid|StatuswordValid))==(PdoValid|StatuswordValid);
 const int wkc=command.resultWKC,size=command.dataSize;const std::uint32_t value=command.dataValue;
 const auto abortCode=matches?ReadMatchingTerminalSdoAbortCode(m,SHM_ECAT_SERVICE_OP_SDO_READ):0U;
 if(matches) { MemoryBarrier();command.status=static_cast<int>(EcatCmdStatus::ECAT_STATUS_IDLE); }
 owned=false;
 row.lastWkc=wkc;row.abortCode=abortCode;
 if(!matches) { Failure(row,IdentityMismatch,now);return; }
 if(timedOut||!sameEpisode) {
  row.flags&=~NativeCurrent;if(!timedOut){row.queryState=Stale;row.readFailureReason=PdoUnavailable;}
  RtPrintf("[EDM381-FIX2] event=RESULT_DISCARDED axis=%u slave=%u timedOut=%u sameEpisode=%u wkc=%d size=%d code=0x%04X authority=READ_ONLY\n",
   row.axisIndex,row.slavePosition,timedOut?1U:0U,sameEpisode?1U:0U,wkc,size,value);return;
 }
 if(abortCode) { Failure(row,Abort,now);return; }
 if(status!=static_cast<int>(EcatCmdStatus::ECAT_STATUS_DONE)||wkc<=0) { Failure(row,Wkc,now);return; }
 if(size!=2) { Failure(row,Size,now);return; }
 if(value>0xffffU) { Failure(row,CodeRange,now);return; }
 if(!value&&(row.flags&(Fault|Warning))) { Failure(row,ZeroDuringFault,now);return; }
 row.nativeCode=value;row.lastGoodAtMs=now;row.readFailureReason=None;row.flags|=NativeCurrent;
 row.queryState=value?Current:NoAlarm;row.attemptCount=0;rowState[activeAxis].due=now+5000ULL;
 if(value) { row.lastKnownNativeCode=value;row.lastKnownAtMs=now;row.lastKnownEpisode=row.faultEpisode;row.flags|=HasLastNonzero; }
 auto& state=rowState[activeAxis];
 if(!state.resultLogged||state.lastResultCode!=value||state.lastResultEpisode!=row.faultEpisode||
    state.lastResultStatus!=(row.flags&(Fault|Warning))||state.failureLogged) {
  RtPrintf("[EDM381-FIX2] event=RESULT axis=%u slave=%u index=0x2001 sub=0 size=%d wkc=%d code=0x%04X fault=%u warning=%u episode=%llu authority=READ_ONLY\n",
   row.axisIndex,row.slavePosition,size,wkc,value,(row.flags&Fault)?1U:0U,(row.flags&Warning)?1U:0U,
   static_cast<unsigned long long>(row.faultEpisode));
 }
 state.resultLogged=true;state.lastResultCode=value;state.lastResultEpisode=row.faultEpisode;
 state.lastResultStatus=row.flags&(Fault|Warning);state.failureLogged=false;
}
void WithdrawClock(EtherCatMaster* m) noexcept {
 if(!clockUnavailableLogged) {
  clockUnavailableLogged=true;
  RtPrintf("[EDM381-FIX2] event=CLOCK_UNAVAILABLE clock=RTX_QPC lastGoodMs=%llu rtTick=%llu action=WITHDRAW_CURRENT_AND_DEFER authority=READ_ONLY\n",
   static_cast<unsigned long long>(lastClockMs),static_cast<unsigned long long>(cache.header.sourcePdoTick));
 }
 sourceAdvanced=false;lastRtTick=lastRtAdvance=0;quietObserved=false;quietSince=0;
 cache.header.flags=Ready;
 for(std::uint32_t i=0;i<cache.header.axisCount;++i) {
  auto& row=cache.rows[i];auto& state=rowState[i];
  if(state.wasValid)++state.generation;state.wasValid=false;
  row.flags&=~(PdoValid|StatuswordValid|Fault|Warning|NativeCurrent);
 }
 // A terminal result may be drained without a valid clock, but a pending
 // PDO-owned operation is never reclaimed or promoted to current data.
 if(owned&&(m->m_asyncCmd.status==static_cast<int>(EcatCmdStatus::ECAT_STATUS_DONE)||
            m->m_asyncCmd.status==static_cast<int>(EcatCmdStatus::ECAT_STATUS_ERROR)))Poll(m,lastClockMs);
 for(std::uint32_t i=0;i<cache.header.axisCount;++i)if(cache.rows[i].flags&Supported) {
  cache.rows[i].queryState=Unavailable;cache.rows[i].readFailureReason=ServiceUnavailable;
 }
 Publish();
}
}
void CaptureRt(EtherCatMaster* m,int lrw,int dc,bool valid) noexcept {
 if(!captureEnabled.load(std::memory_order_acquire)||!m)return;
 RtObservation value{};value.tick=m->tickCount_PDO;value.lrw=lrw;value.expected=m->EXPECTED_WKC_PDO;value.dc=dc;
 value.valid=valid?1U:0U;value.quiet=1U;value.owner=static_cast<std::uint32_t>(m->m_Motion.GetMotionOwner());
 const auto count=(m->m_Axes.size()<m->m_ServoList.size()?m->m_Axes.size():m->m_ServoList.size());
 value.count=static_cast<std::uint32_t>(count<AxisCapacity?count:AxisCapacity);
 for(std::uint32_t i=0;i<value.count;++i) {
  const auto& axis=m->m_Axes[i];if(!axis.isExist)continue;
  if(!std::isfinite(axis.currentCmdVel)||!std::isfinite(axis.targetVelocity)||!std::isfinite(axis.logicalCmdVel)||
     std::fabs(axis.currentCmdVel)>1e-9||std::fabs(axis.targetVelocity)>1e-9||std::fabs(axis.logicalCmdVel)>1e-9)value.quiet=0U;
  const auto ptr=reinterpret_cast<std::uintptr_t>(m->m_ServoList[i].pInput);
  const auto base=reinterpret_cast<std::uintptr_t>(m->m_IoMap);
  if(valid&&base&&ptr>=base&&m->m_IoMapSize>=static_cast<int>(sizeof(ServoInput))&&
     ptr-base<=static_cast<std::uintptr_t>(m->m_IoMapSize-sizeof(ServoInput))) {
   std::uint16_t word=0;std::memcpy(&word,m->m_ServoList[i].pInput,sizeof(word));value.status[i]=word;value.statusValid[i]=1U;
  }
 }
 std::uint64_t words[RtWords]{};std::memcpy(words,&value,sizeof(value));
 rtSequence.fetch_add(1U,std::memory_order_acq_rel);
 for(std::size_t i=0;i<RtWords;++i)rtWords[i].store(words[i],std::memory_order_relaxed);
 rtSequence.fetch_add(1U,std::memory_order_release);
}
void Initialize(EtherCatMaster* m) noexcept {
 if(initialized||!m)return;initialized=true;
 for(auto& state:rowState)state=RowState{};latest={};
 sourceAdvanced=false;owned=false;timedOut=false;lastRtTick=lastRtAdvance=quietSince=lastStart=0;roundRobin=0;
 clockObserved=clockUnavailableLogged=quietObserved=false;lastClockMs=0;
 lastDeferReason=lastDeferOwner=lastDeferNc=0;lastDeferLog=lastAlarmSummary=0;
 deferPending=deferLogged=alarmSummaryLogged=false;
 cache={};cache.header.magic=Magic;cache.header.version=Version;cache.header.bytes=sizeof(Image);
 std::uint64_t initialNow=0;const bool initialClockValid=ReadNow(initialNow);cache.header.nowMs=initialClockValid?initialNow:0;
 LARGE_INTEGER qpc{};RtQueryPerformanceCounter(&qpc);cache.header.runtimeId=static_cast<std::uint64_t>(qpc.QuadPart);if(!cache.header.runtimeId)cache.header.runtimeId=initialNow+1ULL;
 if(!m->m_pEni) { RtPrintf("[EDM381-FIX2] event=UNAVAILABLE reason=NO_ENI authority=READ_ONLY\n");return; }
 const auto& slaves=m->m_pEni->GetSlaves();
 const auto count=(m->m_Axes.size()<m->m_ServoList.size()?m->m_Axes.size():m->m_ServoList.size());
 cache.header.axisCount=static_cast<std::uint32_t>(count<AxisCapacity?count:AxisCapacity);
 for(std::uint32_t i=0;i<cache.header.axisCount;++i) {
  auto& r=cache.rows[i];const auto& drive=m->m_ServoList[i];r.axisIndex=i;r.slavePosition=static_cast<std::uint32_t>(drive.slaveIndex);
  if(!m->m_Axes[i].isExist||drive.slaveIndex<0||static_cast<std::size_t>(drive.slaveIndex)>=slaves.size())continue;
  const auto& slave=slaves[drive.slaveIndex];r.flags=Configured;r.vendorId=slave.vendorId;r.productCode=slave.productCode;
  r.revision=::m_slaveInfo[drive.slaveIndex].Revision_No;r.configuredAddress=slave.hasConfiguredAddress?slave.configuredAddress:slave.configAddrIn;
  const bool eligible=r.vendorId==0x1ddU&&r.productCode==0x6010U&&r.revision==0x30000U&&HasA3Name(slave.name)&&slave.runtimeMailbox.present;
  r.profileId=eligible?1U:0U;r.queryState=eligible?NeverRead:NotSupported;r.queryIndex=eligible?0x2001U:0U;r.querySubIndex=0;r.querySize=eligible?2U:0U;
  if(eligible)r.flags|=Supported;r.identity=(static_cast<std::uint64_t>(r.revision)<<32)|r.productCode;
  RtPrintf("[EDM381-FIX2] event=PROFILE axis=%u slave=%u address=0x%04X vid=0x%08X pid=0x%08X rev=0x%08X profile=%u supported=%u authority=READ_ONLY\n",
   i,r.slavePosition,r.configuredAddress,r.vendorId,r.productCode,r.revision,r.profileId,eligible?1U:0U);
 }
 void* location=nullptr;mappingHandle=RtCreateSharedMemory(PAGE_READWRITE,0,sizeof(Image),L"OSCARMAX_DRIVER_ALARM_DIAG",&location);
 SHARED_MEMORY_INFO info{};
 if(mappingHandle&&location&&RtQuerySharedMemory(mappingHandle,&info)&&info.dwSize>=sizeof(Image)&&!(reinterpret_cast<std::uintptr_t>(location)&7U)) {
  mapping=static_cast<Image*>(location);if(cache.header.runtimeId==mapping->header.runtimeId)++cache.header.runtimeId;Publish();
  captureEnabled.store(true,std::memory_order_release);
 } else { if(mappingHandle)RtCloseHandle(mappingHandle);mappingHandle=nullptr; }
 RtPrintf("[EDM381-FIX2] event=READY mapping=%u bytes=%u axes=%u periodMs=5000 startSpacingMs=250 alarmSafetyRead=1 faultBitRequired=0 clock=RTX_QPC clockValid=%u nowMs=%llu authority=READ_ONLY\n",
  mapping?1U:0U,static_cast<unsigned>(sizeof(Image)),cache.header.axisCount,initialClockValid?1U:0U,static_cast<unsigned long long>(initialNow));
}
void Shutdown() noexcept {
 captureEnabled.store(false,std::memory_order_release);
 std::uint64_t now=0;if(ReadNow(now))cache.header.nowMs=now;
 cache.header.flags=0;Publish();mapping=nullptr;
 if(mappingHandle)RtCloseHandle(mappingHandle);mappingHandle=nullptr;initialized=false;
}
bool OwnsCommand() noexcept { return owned; }
void Tick(EtherCatMaster* m) noexcept {
 if(!initialized||!m)return;
 std::uint64_t now=0;if(!ReadNow(now)) { WithdrawClock(m);return; }
 if(clockUnavailableLogged) {
  clockUnavailableLogged=false;
  RtPrintf("[EDM381-FIX2] event=CLOCK_RECOVERED clock=RTX_QPC nowMs=%llu action=RECONFIRM_SOURCE authority=READ_ONLY\n",static_cast<unsigned long long>(now));
 }
 auto& h=cache.header;h.nowMs=now;
 RtObservation observed{};if(ReadRt(observed)&&observed.tick) {
  if(observed.tick!=lastRtTick) { sourceAdvanced=lastRtTick!=0;lastRtTick=observed.tick;lastRtAdvance=now; }latest=observed;
 }
 const bool coherent=sourceAdvanced&&now>=lastRtAdvance&&now-lastRtAdvance<=120ULL;
 const auto runtime=m->GetRuntimeSnapshot();const bool runtimeReady=runtime.stage==EtherCatRuntimeStage::Running&&runtime.result!=EtherCatRuntimeResult::Fail;
 const bool pdo=coherent&&latest.valid&&runtimeReady;
 h.sourcePdoTick=latest.tick;h.lrwActual=latest.lrw;h.lrwExpected=latest.expected;h.dcWkc=latest.dc;
 h.flags=Ready|(coherent?SourceCoherent:0U)|(pdo?HeaderPdoValid:0U)|(runtimeReady?RuntimeReady:0U);
 h.reasonMask=0;if(coherent&&!latest.valid) { if(latest.lrw<0)h.reasonMask|=1U;if(latest.lrw!=latest.expected)h.reasonMask|=2U;if(!h.reasonMask)h.reasonMask=8U; }
 if(pdo&&latest.quiet) {
  if(!quietObserved) { quietSince=now;quietObserved=true; }
  if(now-quietSince>=100ULL)h.flags|=CommandQuiet;
 }else { quietSince=0;quietObserved=false; }
 EtherCatPdoRuntimeInvalidCorrelationSnapshot correlation{};
 if(TryReadEtherCatPdoRuntimeInvalidCorrelation(correlation)) {
  h.consecutiveInvalid=correlation.currentConsecutiveInvalidCycles;h.latchedReasonMask=correlation.lastInvalidReasonMask;
  h.communicationEpisode=correlation.invalidEpisodeCount;h.lastInvalidTick=correlation.lastInvalidRuntimeCycleTick;
  if(coherent&&!latest.valid&&correlation.currentConsecutiveInvalidCycles)h.reasonMask=correlation.lastInvalidReasonMask;
  h.lastInvalidLrwActual=correlation.lastInvalidActualLrwWkc;h.lastInvalidLrwExpected=correlation.lastInvalidExpectedLrwWkc;h.lastInvalidDc=correlation.lastInvalidDcWkc;
 }
 for(std::uint32_t i=0;i<h.axisCount;++i) {
  auto& row=cache.rows[i];auto& state=rowState[i];if(!(row.flags&Configured))continue;
  const bool good=pdo&&i<latest.count&&latest.statusValid[i];
  row.flags&=~(PdoValid|StatuswordValid|Fault|Warning);
  if(good) {
   row.statusword=latest.status[i];row.flags|=PdoValid|StatuswordValid;
   const bool fault=(row.statusword&8U)!=0,warning=(row.statusword&128U)!=0;
   if(fault)row.flags|=Fault;if(warning)row.flags|=Warning;
   if(fault!=state.fault||warning!=state.warning) {
    ++state.generation;row.flags&=~NativeCurrent;row.attemptCount=0;state.due=now;
    if((fault&&!state.fault)||(warning&&!state.warning&&!fault)) { ++row.faultEpisode;row.lastFaultAtMs=now; }
    state.fault=fault;state.warning=warning;if(row.flags&Supported)row.queryState=Deferred;
   }
   if((row.flags&NativeCurrent)&&now-row.lastGoodAtMs>6000ULL) { row.flags&=~NativeCurrent;row.queryState=Stale; }
  } else {
   if(state.wasValid)++state.generation;row.flags&=~NativeCurrent;
   if(row.flags&Supported) { row.queryState=Unavailable;row.readFailureReason=PdoUnavailable; }
  }
  if(good&&!state.wasValid) {
   row.attemptCount=0;state.due=now;
   if((row.flags&Supported)&&!(owned&&activeAxis==i)&&row.queryState!=ReadFailed&&
      (row.readFailureReason==PdoUnavailable||row.readFailureReason==ServiceUnavailable)) {
    row.queryState=Deferred;row.readFailureReason=None;
   }
  }
  state.wasValid=good;
 }
 Poll(m,now);
 if(!GetEtherCatServiceSharedMemoryData())for(std::uint32_t i=0;i<h.axisCount;++i) {
  auto& row=cache.rows[i];if((row.flags&Supported)&&!(row.flags&NativeCurrent)&&!(owned&&activeAxis==i)) { row.queryState=Deferred;row.readFailureReason=ServiceUnavailable; }
 }
 TraceAlarmSummary(m,now);
 Publish();
}
void TrySubmitBackground(EtherCatMaster* m) noexcept {
 if(!initialized||!mapping||!m||owned)return;
 std::uint64_t now=0;if(!ReadNow(now)) { WithdrawClock(m);return; }
 auto& h=cache.header;h.nowMs=now;
 if((h.flags&(SourceCoherent|HeaderPdoValid|RuntimeReady))!=(SourceCoherent|HeaderPdoValid|RuntimeReady)) { TraceDeferred(m,now,1U);return; }
 if(now-lastStart<250ULL)return;
 if(m->m_asyncCmd.status!=static_cast<int>(EcatCmdStatus::ECAT_STATUS_IDLE)) { TraceDeferred(m,now,2U);return; }
 const auto ncState=m->m_NC?m->m_NC->GetState():NCState::NOT_READY;
 const bool baseQuiet=(h.flags&CommandQuiet)!=0&&ncState!=NCState::RUN;
 // Admission checks the current atomic owner as well as the owner sampled
 // with the quiet RT observation. In-flight reads retain/drain ownership.
 const auto currentOwner=static_cast<std::uint32_t>(m->m_Motion.GetMotionOwner());
 if(currentOwner!=latest.owner) { TraceDeferred(m,now,3U);return; }
 int selected=-1;
 // Fault and warning requests take priority over rotating idle scans.
 for(unsigned priority=0;priority<2&&selected<0;++priority)for(std::uint32_t n=0;n<h.axisCount;++n) {
  const auto i=(roundRobin+n)%h.axisCount;auto& row=cache.rows[i];const bool active=(row.flags&(Fault|Warning))!=0;
  if(!(row.flags&Supported)||rowState[i].due>now||active!=(priority==0))continue;
  const bool ncIdle=ncState==NCState::READY||ncState==NCState::IDLE||ncState==NCState::P_END;
  const bool ownerIdle=currentOwner==static_cast<std::uint32_t>(MotionOwner::NONE)||
   (ncIdle&&currentOwner==static_cast<std::uint32_t>(MotionOwner::IDLE_HOLD));
  // AL3005 is latched by the NC. The current PDO Fault bit may have cleared
  // before this diagnostic sample, so it must not veto a read-only query.
  const bool stoppedAlarm=ncState==NCState::ALARM&&currentOwner==static_cast<std::uint32_t>(MotionOwner::SAFETY);
  if(!baseQuiet||(!ownerIdle&&!stoppedAlarm)) {
   if(row.queryState!=ReadFailed) { row.queryState=Deferred;row.readFailureReason=MotionDeferred; }
   TraceDeferred(m,now,ncState==NCState::RUN?4U:!(h.flags&CommandQuiet)?5U:6U);
   continue;
  }
  if(row.attemptCount>=3U)row.attemptCount=0U; // New bounded round after the 30 s cooldown.
  selected=static_cast<int>(i);break;
 }
 if(selected<0){Publish();return;}
 activeAxis=static_cast<std::uint32_t>(selected);auto& row=cache.rows[activeAxis];auto& command=m->m_asyncCmd;
 auto& state=rowState[activeAxis];
 if(!state.submitLogged||state.lastSubmitEpisode!=row.faultEpisode||state.lastSubmitStatus!=(row.flags&(Fault|Warning))||row.readFailureReason!=None) {
  RtPrintf("[EDM381-FIX2] event=QUERY axis=%u slave=%u index=0x2001 sub=0 size=2 nc=%u owner=%u quiet=1 fault=%u warning=%u attempt=%u authority=READ_ONLY\n",
   row.axisIndex,row.slavePosition,static_cast<std::uint32_t>(ncState),currentOwner,(row.flags&Fault)?1U:0U,
   (row.flags&Warning)?1U:0U,row.attemptCount>=3U?1U:row.attemptCount+1U);
 }
 state.submitLogged=true;state.lastSubmitEpisode=row.faultEpisode;state.lastSubmitStatus=row.flags&(Fault|Warning);
 lastDeferReason=0;deferPending=false;
 activeGeneration=rowState[activeAxis].generation;activeEpisode=row.faultEpisode;activeStatus=row.flags&(Fault|Warning);
 command.type=static_cast<int>(EcatCmdType::CMD_SDO_READ);command.slaveAddr=static_cast<std::uint16_t>(row.slavePosition);
 command.index=0x2001;command.subIndex=0;command.dataValue=0;command.dataSize=2;command.resultWKC=0;
 owned=true;timedOut=false;startMs=lastStart=now;roundRobin=(activeAxis+1U)%h.axisCount;
 ++row.attemptCount;++h.queryCount;row.queryState=Pending;row.readFailureReason=None;row.flags&=~NativeCurrent;
 MemoryBarrier();command.status=static_cast<int>(EcatCmdStatus::ECAT_STATUS_PENDING);Publish();
}
}
