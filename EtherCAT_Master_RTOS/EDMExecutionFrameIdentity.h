#pragma once
// EDM54: exact configuration/mapping identity owned by one serialized RT
// execution context. NC receives only the resulting generation in coherent
// feedback. This component supplies no motion, source or stop authority.
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace EDM54
{
enum class Status : std::uint8_t { Empty, Ready, Exhausted };

struct Stamp
{
    Status status = Status::Empty;
    std::uint64_t generation = 0ULL;
    bool Ready() const noexcept
    { return status == Status::Ready && generation != 0ULL; }
};

// Compare every word, including reserved zero words. A hash collision, numeric
// floating-point equality or a return to an earlier image must not reuse an
// identity. At most WordCount comparisons and one fixed-size copy are needed.
template<std::size_t WordCount>
class FrameIdentity
{
    static_assert(WordCount > 0U && WordCount <= 64U,
        "EDM frame identity requires 1..64 exact words.");
public:
    using Image = std::array<std::uint64_t, WordCount>;

    FrameIdentity() noexcept = default;
    FrameIdentity(const FrameIdentity&) = delete;
    FrameIdentity& operator=(const FrameIdentity&) = delete;

    Stamp Observe(const Image& image) noexcept
    {
        if (exhausted_) return Current();
        if (generation_ != 0ULL && image == words_) return Current();
        if (generation_ == (std::numeric_limits<std::uint64_t>::max)())
        {
            // Keep the last accepted image private, but never expose its
            // generation again. Reverting the input cannot reopen this owner.
            exhausted_ = true;
            return Current();
        }
        words_ = image;
        ++generation_;
        return Current();
    }

    // RT-owner inspection only; this is not an inter-thread publication API.
    Stamp Current() const noexcept
    {
        Stamp stamp{};
        if (exhausted_) stamp.status = Status::Exhausted;
        else if (generation_ != 0ULL)
        { stamp.status = Status::Ready; stamp.generation = generation_; }
        return stamp;
    }

private:
    Image words_{};
    std::uint64_t generation_ = 0ULL;
    bool exhausted_ = false;
};
}
