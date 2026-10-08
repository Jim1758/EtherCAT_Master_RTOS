#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// EDM53 owns the existing RT -> NC value publication. One RT writer and
// concurrent readers share fixed atomic storage; this is not a motion permit.
namespace EDM53
{
    enum class Status : std::uint8_t { Empty, Ready, Busy, Closed };
    struct Stamp
    {
        Status status=Status::Empty;
        std::uint64_t publication=0U;
        bool Ready()const noexcept{return status==Status::Ready&&publication!=0U;}
    };

    template<class Value>
    class Channel
    {
        static_assert(std::is_trivially_copyable<Value>::value&&sizeof(Value)<=2048U,
            "EDM feedback must remain a bounded, trivially copyable value.");
        static_assert(ATOMIC_LLONG_LOCK_FREE==2,
            "EDM feedback requires always lock-free 64-bit atomic words.");
    public:
        bool Publish(const Value& value)noexcept
        {
            if(Current().status==Status::Closed)return false;
            if(nextPublication_>=MaximumPublication())
            {
                // Exhaustion cannot leave an old receipt indefinitely Ready.
                // Only the RT writer closes; no later publication reopens it.
                publication_.store(ClosedPublication(),std::memory_order_release);
                return false;
            }
            const std::uint64_t publication=++nextPublication_;
            std::array<std::uint64_t,WordCount> words{};
            std::memcpy(words.data(),&value,sizeof(value));
            Bank& bank=banks_[static_cast<std::size_t>(publication&1U)];
            bank.sequence.store(publication*2U+1U,std::memory_order_release);
            for(std::size_t i=0U;i<words.size();++i)
                bank.words[i].store(words[i],std::memory_order_release);
            bank.sequence.store(publication*2U,std::memory_order_release);
            publication_.store(publication,std::memory_order_release);
            return true;
        }

        Stamp Read(Value& value)const noexcept
        {
            value=Value{};
            for(unsigned attempt=0U;attempt<4U;++attempt)
            {
                const Stamp stamp=Current();
                if(!stamp.Ready())return stamp;
                const Bank& bank=banks_[static_cast<std::size_t>(stamp.publication&1U)];
                const std::uint64_t expected=stamp.publication*2U;
                if(bank.sequence.load(std::memory_order_acquire)!=expected)continue;
                std::array<std::uint64_t,WordCount> words{};
                for(std::size_t i=0U;i<words.size();++i)
                    words[i]=bank.words[i].load(std::memory_order_acquire);
                // Acquiring a new payload word also observes the writer's
                // preceding busy phase; mixed bank generations cannot pass.
                if(bank.sequence.load(std::memory_order_acquire)!=expected||
                    publication_.load(std::memory_order_acquire)!=stamp.publication)continue;
                std::memcpy(static_cast<void*>(&value),words.data(),sizeof(value));
                return stamp;
            }
            const Stamp current=Current();
            return current.status==Status::Closed?current:Stamp{Status::Busy,0U};
        }

        Stamp Current()const noexcept
        {
            const std::uint64_t publication=publication_.load(std::memory_order_acquire);
            if(!publication)return Stamp{};
            if(publication>MaximumPublication())return Stamp{Status::Closed,0U};
            return Stamp{Status::Ready,publication};
        }
    private:
        static constexpr std::size_t WordCount=(sizeof(Value)+7U)/8U;
        static constexpr std::uint64_t ClosedPublication()noexcept
        {return (std::numeric_limits<std::uint64_t>::max)();}
        static constexpr std::uint64_t MaximumPublication()noexcept
        {return ClosedPublication()/2U;}
        struct Bank
        {
            std::atomic<std::uint64_t> sequence{0U};
            std::array<std::atomic<std::uint64_t>,WordCount> words{};
            Bank()noexcept
            {for(auto& word:words)word.store(0U,std::memory_order_relaxed);}
        };
        std::array<Bank,2U> banks_{};
        std::atomic<std::uint64_t> publication_{0U};
        std::uint64_t nextPublication_=0U; // Single RT writer only.
    };
}
