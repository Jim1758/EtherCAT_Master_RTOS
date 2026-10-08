#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

// EDM52 owns the existing NC -> RT cancellation publication, not a command
// queue or motion permit. Unavailable reads cannot establish Stop evidence.
namespace EDM52
{
    enum class ReadStatus : std::uint8_t { Ready, Busy, Closed };
    struct Snapshot
    {
        ReadStatus status=ReadStatus::Busy;
        std::uint64_t revision=0U,throughSession=0U;
        bool Ready()const noexcept{return status==ReadStatus::Ready;}
    };
    class CancelFence
    {
    public:
        void RequestStop(std::uint64_t session)noexcept
        {
            if(!session)return;
            for(unsigned attempt=0U;attempt<4U;++attempt)
            {
                std::uint64_t phase=phase_.load(std::memory_order_acquire);
                if(phase==Maximum())return;
                if(phase&1U)continue;
                const std::uint64_t through=throughSession_.load(std::memory_order_acquire);
                if(phase_.load(std::memory_order_acquire)!=phase)continue;
                // Older work cannot manufacture a stop for a rearmed newer
                // session. Equal-session requests still need a new revision.
                if(session<through)return;
                if(phase>=Maximum()-2U){Close();return;}
                if(!phase_.compare_exchange_strong(phase,phase+1U,
                    std::memory_order_acq_rel,std::memory_order_acquire))continue;
                throughSession_.store(session,std::memory_order_release);
                std::uint64_t busy=phase+1U;
                // A contending writer may close the fence; never reopen it.
                if(!phase_.compare_exchange_strong(busy,phase+2U,
                    std::memory_order_acq_rel,std::memory_order_acquire))Close();
                return;
            }
            Close();
        }
        Snapshot Read()const noexcept
        {
            for(unsigned attempt=0U;attempt<4U;++attempt)
            {
                const std::uint64_t before=phase_.load(std::memory_order_acquire);
                if(before==Maximum())return Closed();
                if(before&1U)continue;
                const std::uint64_t through=throughSession_.load(std::memory_order_acquire);
                const std::uint64_t after=phase_.load(std::memory_order_acquire);
                if(after==Maximum())return Closed();
                if(before!=after)continue;
                Snapshot result{};result.status=ReadStatus::Ready;
                result.revision=before/2U;result.throughSession=through;return result;
            }
            return Snapshot{};
        }
    private:
        std::atomic<std::uint64_t> phase_{0U},throughSession_{0U};
        static constexpr std::uint64_t Maximum()noexcept
        {return (std::numeric_limits<std::uint64_t>::max)();}
        void Close()noexcept{phase_.store(Maximum(),std::memory_order_release);}
        static Snapshot Closed()noexcept
        {Snapshot result{};result.status=ReadStatus::Closed;return result;}
    };
    inline bool Valid(const Snapshot& value)noexcept
    {return value.Ready()&&((value.revision==0U)==(value.throughSession==0U));}
    // A conservative output fence. Recording cancellation or advancing an
    // observed revision additionally requires Valid(value); Busy is no proof.
    inline bool AffectsSince(const Snapshot& value,std::uint64_t session,
        std::uint64_t capturedRevision)noexcept
    {
        return !Valid(value)||!session||value.revision<capturedRevision||
            (value.revision>capturedRevision&&session<=value.throughSession);
    }
    inline bool RetiresSession(const Snapshot& value,std::uint64_t session)noexcept
    {return !Valid(value)||!session||session<value.throughSession;}
    static_assert(ATOMIC_LLONG_LOCK_FREE==2,"EDM52 requires lock-free 64-bit cancellation words.");
}
