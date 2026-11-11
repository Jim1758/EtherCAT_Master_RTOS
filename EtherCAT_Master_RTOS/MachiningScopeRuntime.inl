// Included exactly once by EtherCatMaster_DC_Runtime.cpp after its platform
// headers. Only P50 mutates handles/controller state; only PDO mutates DATA.
// No locks, waits, allocation, clock calls or output changes in Begin/Commit.
#include "MachiningScopeContract.h"
#include <cmath>
#include <limits>
namespace MachiningScope379
{
namespace
{
    HANDLE dataHandle=nullptr,controlHandle=nullptr;
    Data* data=nullptr;
    Control* control=nullptr;
    std::atomic<std::uint64_t> desired{0ULL},desiredOwner{0ULL},desiredRequest{0ULL},leaseUntilNs{0ULL};
    // PDO-only values below, including the disabled transition flag.
    std::uint64_t observedConfig=0ULL,sessionCount=0ULL,lastSampleTick=0ULL,lastClockNs=0ULL;
    // P50-only values.
    std::uint64_t controllerOwner=0ULL,lastHeartbeat=0ULL,lastRequest=0ULL,ownerObservedNs=0ULL;
    std::uint32_t commandGeneration=0U;
    EDMGapInput::Profile gapProfile{};
    bool gapProfileValid=false;
    // One NC/P50 writer; bounded atomic snapshot, observed only while scope runs.
    std::atomic<std::uint32_t> ncStateSequence{0U},ncProcessPhase{0U};
    std::atomic<std::uint64_t> ncObservedNs{0ULL};
    static_assert(ATOMIC_LLONG_LOCK_FREE==2,"x64 lock-free scope configuration required");

    void HeaderBegin() noexcept
    {
        auto* seq=reinterpret_cast<volatile LONG*>(&data->header.sequence);
        InterlockedExchange(seq,static_cast<LONG>((data->header.sequence+1U)|1U));
        MemoryBarrier();
    }
    void HeaderEnd() noexcept
    {
        const auto even=data->header.sequence+1U;
        MemoryBarrier();InterlockedExchange(reinterpret_cast<volatile LONG*>(&data->header.sequence),
            static_cast<LONG>(even ? even : 2U));
    }
    void EndCapture(std::uint32_t state,std::uint64_t tick) noexcept
    {
        if(data && observedConfig)
        {
            HeaderBegin();data->header.state=state;data->header.lastPdoTick=tick;HeaderEnd();
        }
        observedConfig=0ULL;
    }
    void ReleaseOwner(std::uint64_t owner) noexcept
    {
        desired.store(0ULL,std::memory_order_release);
        if(control && owner)
            InterlockedCompareExchange64(reinterpret_cast<volatile LONGLONG*>(&control->ownerToken),0LL,static_cast<LONGLONG>(owner));
        controllerOwner=lastHeartbeat=lastRequest=ownerObservedNs=0ULL;
    }
    bool CreateMapping(const wchar_t* name,std::uint32_t bytes,HANDLE& handle,void*& location) noexcept
    {
        location=nullptr;handle=RtCreateSharedMemory(PAGE_READWRITE,0,bytes,name,&location);
        if(!handle || !location) return false;
        SHARED_MEMORY_INFO info{};
        // Native allocation reporting may include page rounding. Access only
        // our contract-sized prefix; never accept an undersized mapping.
        return RtQuerySharedMemory(handle,&info) && info.dwSize>=bytes &&
            (reinterpret_cast<std::uintptr_t>(location)&7U)==0U;
    }
}

bool Initialize() noexcept
{
    if(data || control) return data && control;
    LARGE_INTEGER frequency{},counter{};
    if(!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart<=0 ||
        !RtQueryPerformanceCounter(&counter) || counter.QuadPart<=0) return false;
    void* dataLocation=nullptr;void* controlLocation=nullptr;
    if(!CreateMapping(L"OSCARMAX_MACHINING_SCOPE_DATA_V2",sizeof(Data),dataHandle,dataLocation) ||
        !CreateMapping(L"OSCARMAX_MACHINING_SCOPE_CONTROL_V2",sizeof(Control),controlHandle,controlLocation))
    {
        if(dataHandle) RtCloseHandle(dataHandle);
        if(controlHandle) RtCloseHandle(controlHandle);
        dataHandle=controlHandle=nullptr;
        return false;
    }
    auto* mapped=static_cast<Data*>(dataLocation);
    // Fence retained readers before resetting an existing allocation. The old
    // mapping may outlive an RTOS process through an open Windows handle.
    const std::uint64_t previousBootId=mapped->header.bootId;
    const std::uint32_t odd=(mapped->header.sequence+1U)|1U;
    InterlockedExchange(reinterpret_cast<volatile LONG*>(&mapped->header.sequence),static_cast<LONG>(odd));
    MemoryBarrier();
    std::memset(dataLocation,0,offsetof(Header,sequence));
    std::memset(static_cast<unsigned char*>(dataLocation)+offsetof(Header,state),0,
        sizeof(Data)-offsetof(Header,state));
    mapped->header.magic=DataMagic;mapped->header.version=Version;
    mapped->header.structBytes=sizeof(Data);mapped->header.headerBytes=sizeof(Header);
    mapped->header.recordBytes=sizeof(Record);mapped->header.capacity=Capacity;
    mapped->header.bootId=static_cast<std::uint64_t>(counter.QuadPart);
    if(mapped->header.bootId==previousBootId) ++mapped->header.bootId;
    mapped->header.qpcFrequency=static_cast<std::uint64_t>(frequency.QuadPart);
    mapped->header.reserved[0]=ProcessStateCapability|EdmDetailsCapability;
    auto* mailbox=static_cast<Control*>(controlLocation);
    // Clear owner before rewriting control. Readers must validate DATA boot ID
    // both before/after a command, and echo it in each leased request.
    InterlockedExchange64(reinterpret_cast<volatile LONGLONG*>(&mailbox->ownerToken),0LL);
    MemoryBarrier();std::memset(controlLocation,0,sizeof(Control));
    mailbox->magic=ControlMagic;mailbox->version=Version;mailbox->structBytes=sizeof(Control);
    mailbox->bootId=mapped->header.bootId;
    MemoryBarrier();
    InterlockedExchange(reinterpret_cast<volatile LONG*>(&mapped->header.sequence),static_cast<LONG>(odd+1U ? odd+1U : 2U));
    data=mapped;control=mailbox;
    return true;
}

void ShutdownAfterCallbacksStopped() noexcept
{
    desired.store(0ULL,std::memory_order_release);
    if(data)
    {
        HeaderBegin();data->header.state=Stopped;HeaderEnd();
    }
    if(control) InterlockedExchange64(reinterpret_cast<volatile LONGLONG*>(&control->ownerToken),0LL);
    data=nullptr;control=nullptr;observedConfig=0ULL;
    if(dataHandle) RtCloseHandle(dataHandle);
    if(controlHandle) RtCloseHandle(controlHandle);
    dataHandle=controlHandle=nullptr;
}

void PublishNcState(std::uint32_t state,std::uint64_t nowNs) noexcept
{
    ncStateSequence.fetch_add(1U,std::memory_order_acq_rel);
    ncProcessPhase.store(static_cast<std::uint32_t>(NcProcessPhase(state)),std::memory_order_relaxed);
    ncObservedNs.store(nowNs,std::memory_order_relaxed);
    ncStateSequence.fetch_add(1U,std::memory_order_release);
}

void ConfigureGapBeforeStart(const EDMGapInput::Profile& profile) noexcept
{ gapProfile=profile;gapProfileValid=EDMGapInput::Validate(profile) && profile.source==EDMGap::Source::PHYSICAL; }

bool ClientClaimed() noexcept
{ return control && reinterpret_cast<const volatile Control*>(control)->ownerToken!=0ULL; }

void Service(std::uint32_t axes,std::uint64_t nowNs) noexcept
{
    if(!data || !control) return;
    const auto owner=static_cast<std::uint64_t>(InterlockedCompareExchange64(
        reinterpret_cast<volatile LONGLONG*>(&control->ownerToken),0LL,0LL));
    if(!owner)
    {
        if(controllerOwner) ReleaseOwner(controllerOwner);
        return;
    }
    // A client may claim between the caller's cheap test and this read.
    // Defer until the next loop's clock instead of revoking that new claim.
    if(!nowNs) return;
    if(owner!=controllerOwner)
    {
        // A claim never inherits a previous client's accepted command/lease.
        desired.store(0ULL,std::memory_order_release);
        controllerOwner=owner;ownerObservedNs=nowNs;lastHeartbeat=lastRequest=0ULL;
    }
    if(!nowNs || nowNs>0xFFFFFFFFFFFFFFFFULL-LeaseNs || nowNs<ownerObservedNs || nowNs-ownerObservedNs>=LeaseNs)
    { ReleaseOwner(owner);return; }
    const auto* wire=reinterpret_cast<const volatile Control*>(control);
    if(wire->magic!=ControlMagic || wire->version!=Version || wire->structBytes!=sizeof(Control)) return;
    const std::uint32_t before=wire->sequence;
    if(!before || (before&1U)) return;
    MemoryBarrier();
    const std::uint32_t enabled=wire->enabled,axis=wire->axisIndex,period=wire->periodTicks;
    const std::uint64_t heartbeat=wire->heartbeat,request=wire->requestId;
    const std::uint64_t boot=wire->echoBootId,echoOwner=wire->echoOwnerToken;
    MemoryBarrier();
    if(before!=wire->sequence || (wire->sequence&1U) || owner!=wire->ownerToken ||
        boot!=data->header.bootId || boot!=wire->bootId || echoOwner!=owner) return;
    if(enabled>1U || !heartbeat || !request || axis>=axes || axis>=8U || !ValidPeriod(period)) return;
    if(heartbeat<lastHeartbeat || (lastRequest && request<lastRequest)) return;
    if(!enabled)
    { ReleaseOwner(owner);return; }
    if(heartbeat!=lastHeartbeat)
    {
        lastHeartbeat=heartbeat;ownerObservedNs=nowNs;
        leaseUntilNs.store(nowNs+LeaseNs,std::memory_order_release);
    }
    if(request==lastRequest) return; // A late keepalive cannot re-arm timeout.
    lastRequest=request;
    if(commandGeneration==0xFFFFFFFFU) { ReleaseOwner(owner);return; }
    ++commandGeneration;
    // Publish a disabled fence before changing the associated request fields.
    desired.store(0ULL,std::memory_order_release);
    desiredOwner.store(owner,std::memory_order_release);
    desiredRequest.store(request,std::memory_order_release);
    const std::uint64_t config=(static_cast<std::uint64_t>(commandGeneration)<<32U)|
        (static_cast<std::uint64_t>(axis)<<16U)|(static_cast<std::uint64_t>(period)<<8U)|1ULL;
    desired.store(config,std::memory_order_release);
}

bool Enabled() noexcept
{ return desired.load(std::memory_order_relaxed)!=0ULL || observedConfig!=0ULL; }

bool Begin(std::uint64_t tick,std::uint64_t nowNs,std::uint32_t& axis) noexcept
{
    const auto config=desired.load(std::memory_order_acquire);
    if(!config) { EndCapture(Stopped,tick);return false; }
    const auto expiry=leaseUntilNs.load(std::memory_order_acquire);
    if(!nowNs || nowNs>=expiry || (observedConfig && nowNs<=lastClockNs))
    {
        auto expected=config;
        (void)desired.compare_exchange_strong(expected,0ULL,std::memory_order_acq_rel);
        EndCapture(LeaseExpired,tick);return false;
    }
    lastClockNs=nowNs;
    if(config!=observedConfig)
    {
        const auto owner=desiredOwner.load(std::memory_order_acquire);
        const auto request=desiredRequest.load(std::memory_order_acquire);
        if(!data || !owner || !request || config!=desired.load(std::memory_order_acquire)) return false;
        if(sessionCount==0xFFFFFFFFFFFFFFFFULL) return false;
        ++sessionCount;observedConfig=config;lastSampleTick=0ULL;
        HeaderBegin();auto& h=data->header;h.state=Active;h.session=sessionCount;
        h.ownerToken=owner;h.requestId=request;h.writeCount=0ULL;
        h.lastPdoTick=tick;h.axisIndex=static_cast<std::uint32_t>((config>>16U)&0xFFULL);
        h.periodTicks=static_cast<std::uint32_t>((config>>8U)&0xFFULL);HeaderEnd();
    }
    const auto period=static_cast<std::uint32_t>((config>>8U)&0xFFULL);
    if(lastSampleTick && tick>=lastSampleTick && tick-lastSampleTick<period) return false;
    lastSampleTick=tick;axis=static_cast<std::uint32_t>((config>>16U)&0xFFULL);return true;
}

void Commit(Record& value) noexcept
{
    if(!data || !observedConfig || data->header.writeCount==0xFFFFFFFFFFFFFFFFULL) return;
    const auto number=data->header.writeCount+1ULL;
    value.session=sessionCount;value.sampleNumber=number;
    auto& slot=data->records[(number-1ULL)%Capacity];
    auto* sequence=reinterpret_cast<volatile LONG*>(&slot.sequence);
    const auto odd=(slot.sequence+1U)|1U;
    InterlockedExchange(sequence,static_cast<LONG>(odd));MemoryBarrier();
    std::memcpy(reinterpret_cast<unsigned char*>(&slot)+sizeof(std::uint32_t),
        reinterpret_cast<const unsigned char*>(&value)+sizeof(std::uint32_t),sizeof(Record)-sizeof(std::uint32_t));
    MemoryBarrier();InterlockedExchange(sequence,static_cast<LONG>(odd+1U ? odd+1U : 2U));
    HeaderBegin();data->header.writeCount=number;data->header.lastPdoTick=value.pdoTick;HeaderEnd();
}

void CaptureFromPdo(EtherCatMaster* master,std::uint64_t tick,std::int64_t receiptQpc,
    std::uint64_t nowNs,bool valid) noexcept
{
    std::uint32_t axisIndex=0U;
    if(!Begin(tick,nowNs,axisIndex)) return;
    Record row{};const double unavailable=(std::numeric_limits<double>::quiet_NaN)();
    row.commandPosition=row.actualPosition=row.lag=row.commandVelocity=row.actualVelocity=row.motorRpm=unavailable;
    row.gapVoltage=row.gapReferenceVoltage=row.gapBoardVoltage=row.filteredGapVoltage=unavailable;
    row.pdoTick=tick;row.axisIndex=axisIndex;row.processAgeUs=0xFFFFFFFFU;
    if(valid&&master)
    {
        // Current G76/common-path RT control takes precedence over the legacy
        // coordinator and the explicitly aged NC/P50 fallback.
        if(master->m_Motion.ReadMachiningScopeEdmFromOwner(tick,row))
            row.processAgeUs=0U;
        else if(master->m_Motion.ReadMachiningScopeProcessFromOwner(row.processPhase,row.processFlags))
            row.processAgeUs=0U;
        else
        {
            const auto before=ncStateSequence.load(std::memory_order_acquire);
            if(before!=0U&&(before&1U)==0U)
            {
                const auto phase=ncProcessPhase.load(std::memory_order_relaxed);
                const auto observed=ncObservedNs.load(std::memory_order_relaxed);
                std::atomic_thread_fence(std::memory_order_acquire);
                const auto after=ncStateSequence.load(std::memory_order_acquire);
                if(before==after&&observed&&nowNs>=observed&&
                    nowNs-observed<=static_cast<std::uint64_t>(ProcessMaximumAgeUs)*1000ULL&&phase)
                {
                    row.processPhase=phase;row.processFlags=ProcessValid|ProcessSourceNc;
                    row.processAgeUs=static_cast<std::uint32_t>((nowNs-observed)/1000ULL);
                }
            }
        }
    }
    if(receiptQpc>0) { row.sampleQpc=static_cast<std::uint64_t>(receiptQpc);row.flags|=TimestampValid; }
    if(valid) row.flags|=PdoValid;
    if(master && axisIndex<master->m_Axes.size())
    {
        const auto& axis=master->m_Axes[axisIndex];
        if(axis.axisType!=AxisType::LINEAR) row.flags|=Rotary;
        if(valid && axis.isExist && axis.resolution_PPR>0.0 && axis.finalLead>0.0 &&
            std::isfinite(axis.resolution_PPR) && std::isfinite(axis.finalLead))
        {
            const double scale=axis.finalLead/axis.resolution_PPR;
            row.commandPosition=axis.currentCmdPos*scale;
            row.actualPosition=axis.GetNominalActualPositionPulse()*scale;
            row.lag=axis.GetMechanicalFollowingErrorPulse()*scale;
            row.commandVelocity=axis.currentCmdVel*scale*60.0;
            row.actualVelocity=axis.currentActVel*scale*60.0;
            row.motorRpm=axis.currentActVel*60.0/axis.resolution_PPR;
            if(std::isfinite(row.commandPosition) && std::isfinite(row.actualPosition) && std::isfinite(row.lag) &&
                std::isfinite(row.commandVelocity) && std::isfinite(row.actualVelocity) && std::isfinite(row.motorRpm)) row.flags|=AxisValid;
        }
        const auto& input=master->m_gapAnalogInputPublisher.SnapshotFromOwner();
        row.gapSequence=input.sequence;row.gapAdIndex=input.adIndex;
        row.gapRaw=input.rawCode;row.gapReason=static_cast<std::uint32_t>(input.reason);
        row.gapSource=input.configured ? 1U : 0U;
        // Last-good receipt may be older on invalid PDO. Do not stamp it with
        // this callback's new QPC or mark its voltage valid.
        if(input.rtSampleValid && receiptQpc>0) row.gapQpc=static_cast<std::uint64_t>(receiptQpc);
        if(valid && gapProfileValid && input.rtSampleValid && input.transportValid && input.rtClockValid &&
            input.adIndex==gapProfile.adIndex &&
            (gapProfile.expectedDeviceId==0U || input.deviceId==gapProfile.expectedDeviceId))
        {
            const auto converted=EDMGapInput::ConvertRawValidated(gapProfile,input.rawCode);
            if(converted.boardVoltageValid) row.gapBoardVoltage=converted.boardVoltageMv/1000.0;
            if(converted.inputStatus==EDMGapInput::InputStatus::Valid && converted.gapInRange)
            { row.gapVoltage=converted.voltageMv/1000.0;row.flags|=GapValid; }
        }
        // A fresh RT control observation above may supply the reference used
        // by the consumed curve. Legacy/NC-only paths keep it NaN.
    }
    Commit(row);
}
}
