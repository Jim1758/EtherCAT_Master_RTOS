#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

// EDM94: validates the final EtherCAT bytes.  A successful NIC handoff is
// evidence of this software boundary only, never a drive acknowledgement.
namespace EDM94Serialized
{
    constexpr std::size_t AxisCount = 6U;
    constexpr std::size_t OutputBytes = 9U;
    constexpr std::size_t InputBytes = 23U;

    struct Mapping
    {
        std::array<std::uintptr_t, AxisCount> output{};
        std::array<std::uintptr_t, AxisCount> input{};
        std::array<std::int32_t, AxisCount> slave{};
        std::array<std::int32_t, AxisCount> cap{};
    };

    struct Output
    {
        std::int32_t velocity = 0;
        std::uint16_t control = 0U;
        std::uint16_t probe = 0U;
        std::int8_t mode = 0;
    };

    struct Receipt
    {
        std::uint64_t session = 0ULL;
        std::uint64_t sourceTick = 0ULL;
        std::uint64_t frameGeneration = 0ULL;
        std::array<Output, AxisCount> expected{};
        bool held = false;
        bool normalEligible = false;
        bool sealed = false;
        bool allZero = false;
        // Set only when an exact old proposal was fenced by the newly
        // observed cancellation before any NIC call; never a send failure.
        bool cancelDiscard = false;
    };

    enum class Result : std::uint8_t
    {
        None, Mapping, Identity, Payload, Limit, Authority
    };
    enum class Handoff : std::uint8_t
    {
        Discarded, Accepted, ZeroContainmentAccepted, Uncertain
    };

    inline bool InCap(std::int32_t velocity, std::int32_t cap) noexcept
    {
        // Signed comparisons avoid abs(INT32_MIN) and narrowing overflow.
        return cap > 0 && velocity <= cap && velocity >= -cap;
    }

    inline bool ValidMapping(const Mapping& map) noexcept
    {
        std::array<std::uintptr_t, AxisCount * 2U> begin{}, end{};
        const std::uintptr_t maximum = (std::numeric_limits<std::uintptr_t>::max)();
        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            if (map.slave[axis] < 0 || map.cap[axis] <= 0) return false;
            for (std::size_t prior = 0U; prior < axis; ++prior)
                if (map.slave[prior] == map.slave[axis]) return false;
            begin[axis * 2U] = map.output[axis];
            begin[axis * 2U + 1U] = map.input[axis];
            for (std::size_t kind = 0U; kind < 2U; ++kind)
            {
                const std::size_t index = axis * 2U + kind;
                const std::size_t bytes = kind == 0U ? OutputBytes : InputBytes;
                if (begin[index] == 0U || begin[index] > maximum - bytes) return false;
                end[index] = begin[index] + bytes;
                for (std::size_t prior = 0U; prior < index; ++prior)
                    if (begin[index] < end[prior] && begin[prior] < end[index]) return false;
            }
        }
        return true;
    }

    inline bool SameMapping(const Mapping& frozen, const Mapping& current) noexcept
    {
        return frozen.output == current.output && frozen.input == current.input &&
            frozen.slave == current.slave && frozen.cap == current.cap;
    }

    inline bool Within(std::uintptr_t address, std::size_t bytes,
        std::uintptr_t base, std::size_t size) noexcept
    {
        return base != 0U && address >= base && address - base <= size &&
            bytes <= size - static_cast<std::size_t>(address - base);
    }

    inline std::uint16_t Read16(const std::uint8_t* bytes) noexcept
    {
        return static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[0]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U));
    }

    inline std::int32_t ReadSigned32(const std::uint8_t* bytes) noexcept
    {
        const std::uint32_t value = static_cast<std::uint32_t>(bytes[0]) |
            (static_cast<std::uint32_t>(bytes[1]) << 8U) |
            (static_cast<std::uint32_t>(bytes[2]) << 16U) |
            (static_cast<std::uint32_t>(bytes[3]) << 24U);
        // The subtraction is done wide; no implementation-defined unsigned
        // to signed conversion is needed for negative EtherCAT CSV values.
        return value <= 0x7fffffffU ? static_cast<std::int32_t>(value) :
            static_cast<std::int32_t>(static_cast<std::int64_t>(value) - 0x100000000LL);
    }

    inline bool SameOutput(const Output& a, const Output& b) noexcept
    {
        return a.velocity == b.velocity && a.control == b.control &&
            a.probe == b.probe && a.mode == b.mode;
    }

    inline Output ReadOutput(const std::uint8_t* bytes) noexcept
    {
        Output out{};
        out.control = Read16(bytes);
        out.velocity = ReadSigned32(bytes + 2U);
        out.probe = Read16(bytes + 6U);
        const int mode = bytes[8U] <= 127U ? static_cast<int>(bytes[8U]) :
            static_cast<int>(bytes[8U]) - 256;
        out.mode = static_cast<std::int8_t>(mode);
        return out;
    }

    inline Result Seal(Receipt& receipt, const Mapping& frozen, const Mapping& current,
        const std::uint8_t* ioMapBase, std::size_t ioMapSize,
        const std::uint8_t* payload, std::size_t payloadSize) noexcept
    {
        receipt.sealed = false;
        receipt.allZero = false;
        if (!receipt.held || receipt.session == 0ULL || receipt.sourceTick == 0ULL ||
            receipt.frameGeneration == 0ULL) return Result::Identity;
        if (!ValidMapping(frozen) || !SameMapping(frozen, current)) return Result::Mapping;
        if (ioMapBase == nullptr || payload == nullptr || ioMapSize != payloadSize)
            return Result::Payload;
        const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(ioMapBase);
        bool allZero = true;
        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            if (!Within(current.output[axis], OutputBytes, base, ioMapSize) ||
                !Within(current.input[axis], InputBytes, base, ioMapSize)) return Result::Mapping;
            const std::size_t offset = static_cast<std::size_t>(current.output[axis] - base);
            const Output wire = ReadOutput(payload + offset);
            const Output local = ReadOutput(ioMapBase + offset);
            if (!SameOutput(wire, receipt.expected[axis]) || !SameOutput(local, wire))
                return Result::Payload;
            if (!InCap(wire.velocity, frozen.cap[axis])) return Result::Limit;
            if (wire.velocity != 0)
            {
                allZero = false;
                if (!receipt.normalEligible || wire.mode != 9 || wire.control != 0x000fU ||
                    wire.probe != 0U) return Result::Authority;
            }
        }
        receipt.allZero = allZero;
        receipt.sealed = true;
        return Result::None;
    }

    inline Handoff Finish(const Receipt& receipt, bool attempted, bool succeeded) noexcept
    {
        if (!attempted) return succeeded ? Handoff::Uncertain : Handoff::Discarded;
        if (!succeeded || !receipt.held || !receipt.sealed) return Handoff::Uncertain;
        if (receipt.normalEligible) return Handoff::Accepted;
        return receipt.allZero ? Handoff::ZeroContainmentAccepted : Handoff::Uncertain;
    }
}
