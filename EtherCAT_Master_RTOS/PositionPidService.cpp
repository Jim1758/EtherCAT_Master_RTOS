#include "PositionPidService.h"
#include "NCManager.h"
#include "PositionAutoTuneCapture.h"
#include "PositionAutoTuneJob.h"
#include <windows.h>
#include <rtapi.h>
#include <atomic>
#include <cstring>

namespace
{
    enum class Transfer : unsigned { Idle, Queued, Processing, Done };
    HANDLE g_mapping=NULL;
    PositionPidData* g_data=nullptr; // initialize/shutdown only while workers are stopped
    std::atomic<bool> g_enabled{false},g_accepting{false};
    std::atomic<NCManager*> g_nc{nullptr};
    std::atomic<Transfer> g_transfer{Transfer::Idle};
    PositionPidRequest g_request{}; // immutable between Queued and Done
    std::uint64_t g_queuedCycle=0ULL;
    LONGLONG g_frequency=0LL,g_deadline=0LL; // immutable after Queued publication
    PositionPidAck g_runtimeAck{},g_ack{};
    std::uint64_t g_epoch=0ULL;
    std::uint32_t g_heartbeat=0U;
    struct RuntimeImage { PositionPidContext Context; PositionPidValues Values; };
    static_assert(sizeof(RuntimeImage)%8U==0U,"Runtime PID atomic words");
    constexpr std::size_t WordCount=sizeof(RuntimeImage)/8U;
    std::atomic<std::uint64_t> g_sequence{0ULL},g_words[WordCount]{};
    RuntimeImage g_lastImage{}; // NC owned, never reads AxisContext
    struct History { bool used=false; PositionPidRequest request{}; PositionPidAck ack{}; };
    History g_history[16]{};
    unsigned g_nextHistory=0U;
    LONG Exchange(std::uint32_t& p,std::uint32_t v) noexcept
    { return InterlockedExchange(reinterpret_cast<volatile LONG*>(&p),static_cast<LONG>(v)); }
    LONG Compare(std::uint32_t& p,std::uint32_t v,std::uint32_t e) noexcept
    { return InterlockedCompareExchange(reinterpret_cast<volatile LONG*>(&p),static_cast<LONG>(v),static_cast<LONG>(e)); }
    std::uint32_t Read(std::uint32_t& p) noexcept { return static_cast<std::uint32_t>(Compare(p,0U,0U)); }
    void Detail(PositionPidAck& ack,const char* text) noexcept
    {
        std::memset(ack.Detail,0,sizeof(ack.Detail));
        for(std::size_t i=0U;i+1U<sizeof(ack.Detail)&&text[i]!='\0';++i)ack.Detail[i]=text[i];
    }
    bool ReadRuntime(RuntimeImage& image) noexcept
    {
        for(unsigned attempt=0U;attempt<3U;++attempt)
        {
            const auto before=g_sequence.load(std::memory_order_seq_cst);
            if(!before || (before&1ULL))continue;
            std::uint64_t words[WordCount]{};
            for(std::size_t i=0;i<WordCount;++i)words[i]=g_words[i].load(std::memory_order_seq_cst);
            if(before!=g_sequence.load(std::memory_order_seq_cst))continue;
            std::memcpy(&image,words,sizeof(image));return true;
        }
        return false;
    }
    void Publish(bool complete) noexcept
    {
        (void)ReadRuntime(g_lastImage);
        PositionPidContext context=g_lastImage.Context;
        if(!PositionPidNcIdleNow())
        {context.CanApplyMask=0U;context.BlockReason=static_cast<std::uint32_t>(PositionPidReason::NcBusy);}
        if(g_transfer.load(std::memory_order_acquire)!=Transfer::Idle)
        {context.CanApplyMask=0U;context.BlockReason=static_cast<std::uint32_t>(PositionPidReason::MotionBusy);}
        const auto odd=(Read(g_data->Header.Sequence)+1U)|1U;
        Exchange(g_data->Header.Sequence,odd);MemoryBarrier();
        g_data->Header.Flags=POSITION_PID_PUBLISHED;g_data->Header.Heartbeat=++g_heartbeat;
        g_data->Context=context;g_data->Values=g_lastImage.Values;g_data->Ack=g_ack;
        MemoryBarrier();Exchange(g_data->Header.Sequence,odd+1U);
        if(complete)Exchange(g_data->Mailbox.State,static_cast<std::uint32_t>(PositionPidMailboxState::Complete));
    }
    void Remember(const PositionPidRequest& request) noexcept
    { auto& h=g_history[g_nextHistory++%16U];h.used=true;h.request=request;h.ack=g_ack; }
    void Reject(const PositionPidRequest& request,PositionPidStatus status,PositionPidReason reason,const char* detail,bool remember) noexcept
    {
        g_ack=PositionPidAck{};g_ack.SessionId=request.SessionId;g_ack.RequestId=request.RequestId;
        g_ack.RuntimeEpoch=g_epoch;g_ack.Generation=g_lastImage.Context.Generation;
        g_ack.Status=static_cast<std::uint32_t>(status);g_ack.Reason=static_cast<std::uint32_t>(reason);
        g_ack.Axis=request.Axis;g_ack.Bank=request.Bank;
        if(request.Axis<POSITION_PID_AXIS_COUNT && request.Bank<2U)
            g_ack.Applied=g_lastImage.Values.Axis[request.Axis][request.Bank];
        Detail(g_ack,detail);if(remember)Remember(request);Publish(true);
    }
}

bool InitializePositionPidService() noexcept
{
    if(g_data)return true;
    LARGE_INTEGER frequency{};
    if(!QueryPerformanceFrequency(&frequency) || frequency.QuadPart<=0)return false;
    g_frequency=frequency.QuadPart;
    void* location=nullptr;
    g_mapping=RtCreateSharedMemory(PAGE_READWRITE,0,POSITION_PID_SIZE,L"OSCARMAX_POSITION_PID",&location);
    if(g_mapping==NULL || !location)
    {if(g_mapping!=NULL)RtCloseHandle(g_mapping);g_mapping=NULL;return false;}
    g_data=static_cast<PositionPidData*>(location);
    const auto previous=g_data->Header.RuntimeEpoch;
    const auto odd=(Read(g_data->Header.Sequence)+1U)|1U;
    Exchange(g_data->Header.Sequence,odd);MemoryBarrier();
    LARGE_INTEGER counter{};QueryPerformanceCounter(&counter);
    g_epoch=static_cast<std::uint64_t>(counter.QuadPart)^(static_cast<std::uint64_t>(GetCurrentProcessId())<<32U);
    if(!g_epoch || g_epoch==previous)g_epoch=previous+1ULL;
    if(!g_epoch)g_epoch=1ULL;
    std::memset(g_data,0,offsetof(PositionPidHeader,Sequence));
    std::memset(reinterpret_cast<unsigned char*>(g_data)+16U,0,POSITION_PID_SIZE-16U);
    g_data->Header.Magic=POSITION_PID_MAGIC;g_data->Header.VersionMajor=1U;
    g_data->Header.StructSize=POSITION_PID_SIZE;g_data->Header.RuntimeEpoch=g_epoch;
    g_ack=PositionPidAck{};g_runtimeAck=PositionPidAck{};g_lastImage=RuntimeImage{};
    g_lastImage.Context.BlockReason=static_cast<std::uint32_t>(PositionPidReason::RuntimeUnavailable);
    g_heartbeat=0U;g_nextHistory=0U;for(auto& h:g_history)h=History{};
    g_sequence.store(0ULL,std::memory_order_seq_cst);
    for(auto& word:g_words)word.store(0ULL,std::memory_order_seq_cst);
    g_transfer.store(Transfer::Idle,std::memory_order_release);
    MemoryBarrier();Exchange(g_data->Header.Sequence,odd+1U);
    g_enabled.store(true,std::memory_order_release);return true;
}
void ShutdownPositionPidService() noexcept
{
    // SHMManager shutdown happens only after cyclic workers have exited.
    g_enabled.store(false,std::memory_order_release);g_accepting.store(false,std::memory_order_release);g_nc.store(nullptr,std::memory_order_release);
    if(g_data)
    {
        const auto odd=(Read(g_data->Header.Sequence)+1U)|1U;
        Exchange(g_data->Header.Sequence,odd);MemoryBarrier();
        g_data->Header.Flags=POSITION_PID_SHUTTING_DOWN;g_data->Context.CanApplyMask=0U;
        MemoryBarrier();Exchange(g_data->Header.Sequence,odd+1U);
    }
    g_data=nullptr;if(g_mapping!=NULL){RtCloseHandle(g_mapping);g_mapping=NULL;}
}
bool PositionPidRuntimeEnabled() noexcept { return g_enabled.load(std::memory_order_acquire); }
bool PositionPidNcIdleNow() noexcept
{
    NCManager* nc=g_nc.load(std::memory_order_acquire);
    if(!nc || !g_enabled.load(std::memory_order_acquire) || !g_accepting.load(std::memory_order_acquire))return false;
    const auto state=nc->GetState();
    return (state==NCState::READY || state==NCState::IDLE || state==NCState::P_END) &&
        !PositionAutoTune::Capture().Active() && !PositionAutoTune::JobBusy();
}
bool PositionPidRuntimeDeadlineCurrent() noexcept
{
    LARGE_INTEGER now{};
    return g_deadline>0 && QueryPerformanceCounter(&now) && now.QuadPart>0 && now.QuadPart<=g_deadline;
}
bool PositionPidRuntimeTake(PositionPidRequest& request,std::uint64_t& queuedCycle) noexcept
{
    auto expected=Transfer::Queued;
    if(!g_enabled.load(std::memory_order_acquire) ||
        !g_transfer.compare_exchange_strong(expected,Transfer::Processing,std::memory_order_acq_rel))return false;
    request=g_request;queuedCycle=g_queuedCycle;return true;
}
void PositionPidRuntimePublish(const PositionPidContext& context,const PositionPidValues& values) noexcept
{
    RuntimeImage image{};image.Context=context;image.Values=values;
    std::uint64_t words[WordCount]{};std::memcpy(words,&image,sizeof(image));
    const auto odd=(g_sequence.load(std::memory_order_seq_cst)+1ULL)|1ULL;
    g_sequence.store(odd,std::memory_order_seq_cst);
    for(std::size_t i=0;i<WordCount;++i)g_words[i].store(words[i],std::memory_order_seq_cst);
    g_sequence.store(odd+1ULL,std::memory_order_seq_cst);
}
void PositionPidRuntimeComplete(const PositionPidRequest& r,PositionPidStatus status,PositionPidReason reason,
    std::uint64_t generation,const PositionPidGains& applied) noexcept
{
    g_runtimeAck=PositionPidAck{};g_runtimeAck.SessionId=r.SessionId;g_runtimeAck.RequestId=r.RequestId;
    g_runtimeAck.RuntimeEpoch=r.RuntimeEpoch;g_runtimeAck.Generation=generation;
    g_runtimeAck.Status=static_cast<std::uint32_t>(status);g_runtimeAck.Reason=static_cast<std::uint32_t>(reason);
    g_runtimeAck.Axis=r.Axis;g_runtimeAck.Bank=r.Bank;g_runtimeAck.Applied=applied;
    Detail(g_runtimeAck,status==PositionPidStatus::Success?"Runtime gains applied; disk unchanged.":"Rejected; runtime gains unchanged.");
    g_transfer.store(Transfer::Done,std::memory_order_release);
}
bool ProcessPositionPidServiceSameThread(NCManager& nc) noexcept
{
    if(!g_data)return false;
    g_nc.store(&nc,std::memory_order_release);
    g_accepting.store(!nc.Close_System_Com_flag,std::memory_order_release);
    const auto pending=g_transfer.load(std::memory_order_acquire);
    if(pending==Transfer::Done)
    {
        // A success receipt must be published with an observation containing
        // that commit. A contended bounded RT-image read may fail; leave the
        // mailbox Processing and retry on the next NC scan rather than expose
        // a success ACK beside the previous gains/generation. The RT commit
        // is already complete, so ordinary NC and safety work can continue.
        if(g_runtimeAck.Status==static_cast<std::uint32_t>(PositionPidStatus::Success) &&
            (!ReadRuntime(g_lastImage) ||
             g_lastImage.Context.Generation<g_runtimeAck.Generation))
        {Publish(false);return false;}
        g_ack=g_runtimeAck;Remember(g_request);
        g_transfer.store(Transfer::Idle,std::memory_order_release);Publish(true);return false;
    }
    if(pending!=Transfer::Idle)
    {
        if(pending==Transfer::Queued && !PositionPidRuntimeDeadlineCurrent())
        {
            auto expected=Transfer::Queued;
            if(g_transfer.compare_exchange_strong(expected,Transfer::Idle,std::memory_order_acq_rel))
            {Reject(g_request,PositionPidStatus::Stale,PositionPidReason::Expired,"Request expired before RT admission.",true);return false;}
        }
        Publish(false);
        // Never defer RESET, alarm handling, or shutdown for this UI request.
        const auto state=nc.GetState();
        return state!=NCState::RESET_STATE && state!=NCState::ALARM && !nc.Close_System_Com_flag;
    }
    (void)ReadRuntime(g_lastImage);
    const auto ready=static_cast<std::uint32_t>(PositionPidMailboxState::Ready);
    if(static_cast<std::uint32_t>(Compare(g_data->Mailbox.State,static_cast<std::uint32_t>(PositionPidMailboxState::Processing),ready))!=ready)
    {Publish(false);return false;}
    const auto before=Read(g_data->Mailbox.RequestSequence);MemoryBarrier();
    const PositionPidRequest request=g_data->Request;MemoryBarrier();
    if(!before || (before&1U) || before!=Read(g_data->Mailbox.RequestSequence) || !PositionPid::Canonical(request))
    {Reject(request,PositionPidStatus::InvalidCommand,PositionPidReason::InvalidGain,"Malformed or torn request.",false);return false;}
    if(request.RuntimeEpoch!=g_epoch)
    {Reject(request,PositionPidStatus::Stale,PositionPidReason::Conflict,"Core restarted; read fresh gains.",false);return false;}
    for(const auto& h:g_history)
    {
        if(!h.used || h.request.SessionId!=request.SessionId || h.request.RequestId!=request.RequestId)continue;
        if(std::memcmp(&h.request,&request,sizeof(request))==0){g_ack=h.ack;Publish(true);return false;}
        Reject(request,PositionPidStatus::Stale,PositionPidReason::Conflict,"Request ID reused with different values.",false);return false;
    }
    if(!PositionPidNcIdleNow())
    {Reject(request,PositionPidStatus::NotAllowed,PositionPidReason::NcBusy,"NC, tuning or report worker is busy.",true);return false;}
    if(!g_lastImage.Context.Generation || !g_lastImage.Context.RuntimeCycle)
    {Reject(request,PositionPidStatus::Unavailable,PositionPidReason::RuntimeUnavailable,"No current RT PID observation.",true);return false;}
    if(request.ExpectedGeneration!=g_lastImage.Context.Generation)
    {Reject(request,PositionPidStatus::Stale,PositionPidReason::Conflict,"Gains changed; read and confirm again.",true);return false;}
    // The next RT pass rechecks all mutable guards and the generation. No
    // success is published until its gain commit and coherent readback finish.
    LARGE_INTEGER now{};
    if(!QueryPerformanceCounter(&now) || now.QuadPart<=0 || now.QuadPart>INT64_MAX-g_frequency)
    {Reject(request,PositionPidStatus::Unavailable,PositionPidReason::RuntimeUnavailable,"No monotonic request deadline.",true);return false;}
    g_deadline=now.QuadPart+g_frequency/4LL; // 250 ms, independent of RT cycle advancement
    g_request=request;g_queuedCycle=g_lastImage.Context.RuntimeCycle;
    g_transfer.store(Transfer::Queued,std::memory_order_release);Publish(false);return true;
}
