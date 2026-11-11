#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <type_traits>

// Passive, single-RT-writer / single-P50-reader response capture. This header
// has no clocks, I/O, allocation, waits or motion authority. A ready or owned
// capture is never overwritten, including when another NC session starts.
namespace EDMG01ResponseTrace
{
    constexpr unsigned Capacity = 1536U;
    constexpr unsigned PreCapacity = 512U;
    constexpr unsigned PostCapacity = 1024U; // Includes the trigger sample.
    constexpr std::uint32_t NoTrigger = (std::numeric_limits<std::uint32_t>::max)();

    enum Stage : std::uint32_t
    { Input=1U, Motion=2U, Control=4U, LocalImage=8U, Send=16U, Return=32U };
    enum Flag : std::uint32_t
    {
        PdoValid=1U, CombinedReturn=2U, SourceValid=4U, Nominal=8U,
        Short=16U, Suspended=32U, GapApplied=64U, Hold=128U, Flush=256U,
        HardwareReverse=512U, RawAdValid=1024U, NewCurveValid=2048U,
        UsedDemandValid=4096U, CollisionValid=8192U
    };
    enum SendFlag : std::uint32_t
    { Attempted=1U, ApiAccepted=2U, FinalChecked=4U, Scrubbed=8U, Idle=16U, Scoped=32U };
    enum Reason : std::uint32_t
    { Full=1U, Held=2U, Terminal=3U, InputFault=4U, ResetOrScopeEnd=5U };

    struct Sample
    {
        std::uint64_t tick=0ULL, nowUs=0ULL;
        std::uint64_t usedCurveTick=0ULL, usedSequence=0ULL, usedCapturedUs=0ULL;
        std::uint64_t newCurveTick=0ULL, newSequence=0ULL, newCapturedUs=0ULL;
        std::uint64_t txSequence=0ULL, txSourceTick=0ULL, txFrame=0ULL, returnTick=0ULL;
        double usedVoltageV=0.0, newVoltageV=0.0, usedDemandMmMin=0.0, nextDemandMmMin=0.0;
        double pathMmMin=0.0, zReferenceMmMin=0.0, commandPulse=0.0, actualPulse=0.0;
        double posDiffPps=0.0, ppu=0.0, servoCommandPulse=0.0, servoReferencePps=0.0;
        double pTerm=0.0, iTerm=0.0, kp=0.0, ki=0.0, kvff=0.0, hardwareProposalPps=0.0;
        std::uint32_t stages=0U, flags=0U, sendFlags=0U, phase=0U, rawAd=0U, sourceStatus=0U;
        std::int32_t rawActualPosition=0, localCsvPps=0, txProposedPps=0, txCopiedPps=0;
        std::int32_t sourceMode=0;
        bool returnPdoValid=false, returnCombined=false;
    };

    struct Header
    {
        std::uint64_t session=0ULL, triggerTick=0ULL, firstTick=0ULL, lastTick=0ULL;
        std::uint64_t gapCount=0ULL, skippedSessions=0ULL, totalRows=0ULL;
        std::uint32_t count=0U, preCount=0U, postCount=0U, triggerIndex=NoTrigger;
        std::uint32_t reason=0U, capacity=Capacity;
    };

    static_assert(PreCapacity+PostCapacity==Capacity, "Capture dimensions must agree.");
    static_assert(ATOMIC_INT_LOCK_FREE==2 && ATOMIC_LLONG_LOCK_FREE==2,
        "Diagnostic ownership and counters require lock-free atomic words.");
    static_assert(std::is_trivially_copyable<Sample>::value && sizeof(Sample)<=384U,
        "One trace row must remain a bounded plain value.");

    class Recorder
    {
    public:
        // RT writer only. Each distinct session is attempted once. A session
        // skipped while an earlier capture is pending stays skipped after drain.
        bool Begin(std::uint64_t session) noexcept
        {
            if(!session)return false;
            const unsigned state=state_.load(std::memory_order_acquire);
            if(session==session_)return state==Recording;
            if(session==lastAttemptedSession_)
                return state==Recording && session_==session;
            lastAttemptedSession_=session;
            if(state!=Free) {
                IncrementAtomic(skippedSessions_);
                return false;
            }
            session_=session; next_=count_=first_=preCount_=postCount_=0U;
            triggerIndex_=NoTrigger; triggerTick_=lastTick_=gapCount_=totalRows_=0ULL;
            triggered_=havePending_=false;
            pending_=Sample{}; header_=Header{};
            state_.store(Recording,std::memory_order_relaxed);
            return true;
        }

        // RT writer only. The new input annotates the prior row's return;
        // callers must not describe that return as response to this new TX.
        void Advance(std::uint64_t tick,std::uint64_t nowUs,
            bool pdoValid,bool combinedReturn) noexcept
        {
            if(state_.load(std::memory_order_acquire)!=Recording)return;
            if(!tick || !nowUs || (lastTick_ && tick<=lastTick_)) {
                Increment(gapCount_); return;
            }
            if(lastTick_ && (lastTick_==(std::numeric_limits<std::uint64_t>::max)() ||
                tick!=lastTick_+1ULL))Increment(gapCount_);
            lastTick_=tick;
            if(havePending_) {
                pending_.returnTick=tick;
                pending_.returnPdoValid=pdoValid;
                pending_.returnCombined=combinedReturn;
                pending_.stages|=Return;
                Seal();
            }
            if(state_.load(std::memory_order_relaxed)!=Recording)return;
            pending_=Sample{}; pending_.tick=tick; pending_.nowUs=nowUs;
            pending_.stages=Input;
            pending_.flags=(pdoValid?PdoValid:0U)|(combinedReturn?CombinedReturn:0U);
            havePending_=true;
        }

        // RT writer only. The pointer is temporary: do not keep it past the
        // current callback or an Advance/Stop call.
        Sample* Pending(std::uint64_t tick) noexcept
        {
            return state_.load(std::memory_order_relaxed)==Recording && havePending_ &&
                pending_.tick==tick ? &pending_ : nullptr;
        }
        std::uint64_t ActiveSession() const noexcept { return session_; } // RT only.
        std::uint64_t SkippedSessions() const noexcept
        { return skippedSessions_.load(std::memory_order_acquire); }

        // RT writer only. Retains a partial final row without inventing a
        // future return. Trigger bookkeeping still runs for that final row.
        void Stop(Reason reason) noexcept
        {
            if(state_.load(std::memory_order_acquire)!=Recording)return;
            if(havePending_)Seal();
            if(state_.load(std::memory_order_relaxed)==Recording)Freeze(reason);
        }
        // RT writer only; the caller decides when supervisory dumping is safe.
        void AllowDump() noexcept
        {
            if(state_.load(std::memory_order_relaxed)==Frozen)
                state_.store(Ready,std::memory_order_release);
        }

        // Sole P50 reader. Claim once, then read immutable rows until EndRead.
        bool BeginRead(Header& out) noexcept
        {
            unsigned expected=Ready;
            if(!state_.compare_exchange_strong(expected,Reading,
                std::memory_order_acquire,std::memory_order_relaxed))return false;
            out=header_; return true;
        }
        bool ReadRow(std::uint64_t session,unsigned index,Sample& out) const noexcept
        {
            if(state_.load(std::memory_order_acquire)!=Reading ||
                session!=header_.session || index>=header_.count)return false;
            out=rows_[(first_+index)%Capacity]; return true;
        }
        void EndRead(std::uint64_t session) noexcept
        {
            if(state_.load(std::memory_order_relaxed)==Reading && session==header_.session)
                state_.store(Free,std::memory_order_release);
        }

    private:
        enum State : unsigned { Free,Recording,Frozen,Ready,Reading };
        static void Increment(std::uint64_t& n) noexcept
        { if(n!=(std::numeric_limits<std::uint64_t>::max)())++n; }
        static void IncrementAtomic(std::atomic<std::uint64_t>& value) noexcept
        {
            // Only the RT writer increments; readers only load.
            std::uint64_t n=value.load(std::memory_order_relaxed);
            Increment(n); value.store(n,std::memory_order_release);
        }
        void Seal() noexcept
        {
            if(!havePending_)return;
            const bool trigger=!triggered_ && (pending_.stages&Motion)!=0U &&
                (pending_.flags&Nominal)!=0U && pending_.nextDemandMmMin<0.0;
            if(trigger) {
                triggered_=true; preCount_=count_; triggerIndex_=preCount_;
                triggerTick_=pending_.tick;
                first_=(next_+Capacity-count_)%Capacity;
            }
            rows_[next_]=pending_; next_=(next_+1U)%Capacity;
            Increment(totalRows_); havePending_=false;
            if(triggered_) {
                ++postCount_; ++count_;
                if(postCount_==PostCapacity)Freeze(Full);
            } else {
                if(count_<PreCapacity)++count_;
                first_=(next_+Capacity-count_)%Capacity;
            }
        }
        void Freeze(Reason reason) noexcept
        {
            header_=Header{}; header_.session=session_; header_.count=count_;
            header_.preCount=triggered_?preCount_:count_; header_.postCount=postCount_;
            header_.triggerIndex=triggerIndex_; header_.triggerTick=triggerTick_;
            header_.reason=static_cast<std::uint32_t>(reason); header_.gapCount=gapCount_;
            header_.skippedSessions=skippedSessions_.load(std::memory_order_acquire);
            header_.totalRows=totalRows_;
            if(count_) {
                header_.firstTick=rows_[first_].tick;
                header_.lastTick=rows_[(first_+count_-1U)%Capacity].tick;
            }
            state_.store(Frozen,std::memory_order_release);
        }

        std::array<Sample,Capacity> rows_{};
        Sample pending_{}; Header header_{};
        std::atomic<unsigned> state_{Free};
        std::atomic<std::uint64_t> skippedSessions_{0ULL};
        std::uint64_t session_=0ULL,lastAttemptedSession_=0ULL,lastTick_=0ULL;
        std::uint64_t triggerTick_=0ULL,gapCount_=0ULL,totalRows_=0ULL;
        unsigned next_=0U,count_=0U,first_=0U,preCount_=0U,postCount_=0U;
        std::uint32_t triggerIndex_=NoTrigger;
        bool triggered_=false,havePending_=false;
    };
}
