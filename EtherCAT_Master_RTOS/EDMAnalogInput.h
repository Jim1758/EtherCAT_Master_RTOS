#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// Selected GAP A/D acquisition. The PDO owner is the only writer. Consumers
// receive an atomic copy; they never access the live EtherCAT process image.
namespace EDMAnalogInput
{
    enum class Reason : std::uint32_t
    {
        NotConfigured = 0U, BindingMissing = 1U, UnsupportedLayout = 2U,
        IdentityMismatch = 3U, AwaitSample = 4U, TransportInvalid = 5U,
        ClockInvalid = 6U, Valid = 7U, SnapshotBusy = 8U, ReadError = 9U,
        ContractInvalid = 10U
    };

    inline const char* ReasonName(Reason reason) noexcept
    {
        switch (reason)
        {
        case Reason::NotConfigured: return "NOT_CONFIGURED";
        case Reason::BindingMissing: return "BINDING_MISSING";
        case Reason::UnsupportedLayout: return "UNSUPPORTED_LAYOUT";
        case Reason::IdentityMismatch: return "IDENTITY_MISMATCH";
        case Reason::AwaitSample: return "AWAIT_SAMPLE";
        case Reason::TransportInvalid: return "TRANSPORT_INVALID";
        case Reason::ClockInvalid: return "CLOCK_INVALID";
        case Reason::Valid: return "VALID";
        case Reason::SnapshotBusy: return "SNAPSHOT_BUSY";
        case Reason::ReadError: return "READ_ERROR";
        case Reason::ContractInvalid: return "CONTRACT_INVALID";
        }
        return "UNKNOWN";
    }

    struct Layout
    {
        bool analogInput = false;
        const char* dataType = "";
        const char* sampleMode = "";
        std::int32_t inputBitOffset = -1, slaveIndex = -1;
        std::uint32_t inputBitLength = 0U, elementBits = 0U, channelCount = 0U, deviceId = 0U;
    };

    struct Binding
    {
        std::uint32_t adIndex = 0U, deviceId = 0U, channelId = 0U, channelCount = 0U;
        std::int32_t slaveIndex = -1, inputBitOffset = -1;
        bool configured = false, unsignedRaw = false;
        Reason reason = Reason::NotConfigured;
    };

    // AD numbering is 1-based, in runtime slave/binding order. Array bindings
    // contribute ChannelCount entries. Unsupported channels retain their index
    // rather than silently shifting every subsequent AD number.
    inline Binding Resolve(const Layout* layouts, std::size_t count,
        std::uint32_t adIndex, std::uint32_t expectedDeviceId, std::uint32_t imageBytes,
        bool validatedCompositeContract) noexcept
    {
        Binding result{};
        result.adIndex = result.channelId = adIndex;
        const Layout* selected = nullptr;
        std::uint32_t selectedChannel = 0U;
        std::uint64_t total = 0ULL;
        if (layouts == nullptr && count != 0U) { result.reason = Reason::BindingMissing; return result; }
        for (std::size_t i = 0U; i < count; ++i)
        {
            const auto& layout = layouts[i];
            if (!layout.analogInput) continue;
            const std::uint32_t channels = layout.channelCount == 0U ? 1U : layout.channelCount;
            if (adIndex != 0U && adIndex > total && static_cast<std::uint64_t>(adIndex) <= total + channels)
            {
                selected = &layout;
                selectedChannel = static_cast<std::uint32_t>(adIndex - total - 1ULL);
            }
            total += channels;
        }
        result.channelCount = total > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<std::uint32_t>(total);
        if (adIndex == 0U) return result;
        if (selected == nullptr) { result.reason = Reason::BindingMissing; return result; }
        result.deviceId = selected->deviceId;
        result.slaveIndex = selected->slaveIndex;
        const bool signedRaw = selected->dataType != nullptr && std::strcmp(selected->dataType, "Int16") == 0;
        result.unsignedRaw = selected->dataType != nullptr && std::strcmp(selected->dataType, "UInt16") == 0;
        const std::uint64_t selectedBit = selected->inputBitOffset >= 0
            ? static_cast<std::uint64_t>(selected->inputBitOffset) + static_cast<std::uint64_t>(selectedChannel) * 16ULL
            : (std::numeric_limits<std::uint64_t>::max)();
        if ((!signedRaw && !result.unsignedRaw) || selected->sampleMode == nullptr ||
            std::strcmp(selected->sampleMode, "Cyclic") != 0 || selected->elementBits != 16U ||
            selected->channelCount == 0U || selected->inputBitOffset < 0 ||
            selected->inputBitLength != static_cast<std::uint64_t>(selected->channelCount) * 16ULL ||
            selectedBit > 0x7FFFFFFFULL || selectedBit + 16ULL > static_cast<std::uint64_t>(imageBytes) * 8ULL ||
            static_cast<std::uint64_t>(selected->inputBitOffset) + selected->inputBitLength >
                static_cast<std::uint64_t>(imageBytes) * 8ULL || selected->slaveIndex < 0 || selected->deviceId == 0U)
        { result.reason = Reason::UnsupportedLayout; return result; }
        result.inputBitOffset = static_cast<std::int32_t>(selectedBit);
        if (expectedDeviceId != 0U && expectedDeviceId != result.deviceId)
        { result.reason = Reason::IdentityMismatch; return result; }
        // Global map bounds alone do not prove the range belongs to this slave.
        // Boot-time relative-to-absolute + slave-envelope + descriptor-pointer
        // audits are mandatory before promoting a shadow descriptor to live AD.
        if (!validatedCompositeContract)
        { result.reason = Reason::ContractInvalid; return result; }
        result.configured = true;
        result.reason = Reason::AwaitSample;
        return result;
    }

    // Byte and bit extraction avoids unaligned typed loads and preserves UInt16
    // codes above 32767. The selected channel can begin at any mapped bit.
    inline bool ReadRaw16(const std::uint8_t* image, std::uint32_t imageBytes,
        std::int32_t inputBitOffset, bool unsignedRaw, std::int32_t& rawCode) noexcept
    {
        rawCode = 0;
        if (image == nullptr || inputBitOffset < 0 ||
            static_cast<std::uint64_t>(inputBitOffset) + 16ULL > static_cast<std::uint64_t>(imageBytes) * 8ULL)
            return false;
        const std::uint32_t offset = static_cast<std::uint32_t>(inputBitOffset);
        const std::uint32_t byte = offset / 8U, shift = offset % 8U;
        std::uint32_t packed = static_cast<std::uint32_t>(image[byte]) |
            (static_cast<std::uint32_t>(image[byte + 1U]) << 8U);
        if (shift != 0U) packed |= static_cast<std::uint32_t>(image[byte + 2U]) << 16U;
        const std::uint32_t code = (packed >> shift) & 0xFFFFU;
        rawCode = unsignedRaw || code <= 0x7FFFU ? static_cast<std::int32_t>(code)
            : static_cast<std::int32_t>(code) - 65536;
        return true;
    }

    struct Snapshot
    {
        std::uint32_t adIndex = 0U, deviceId = 0U, channelId = 0U, channelCount = 0U;
        std::int32_t slaveIndex = -1, inputBitOffset = -1, rawCode = 0;
        std::uint64_t sequence = 0ULL, capturedAtMs = 0ULL, observedAtMs = 0ULL;
        bool configured = false, hasSample = false, transportValid = false, clockValid = false, unsignedRaw = false;
        Reason reason = Reason::NotConfigured;
    };
    static_assert(std::is_trivially_copyable<Snapshot>::value, "A/D snapshot must be a plain atomic-copy payload.");
    static_assert(ATOMIC_LLONG_LOCK_FREE == 2, "The x64 PDO snapshot must use lock-free atomics.");

    class Publisher
    {
    public:
        void ConfigureBeforeStart(const Binding& binding) noexcept
        {
            m_writer = Snapshot{};
            m_writer.adIndex = binding.adIndex;
            m_writer.deviceId = binding.deviceId;
            m_writer.channelId = binding.channelId;
            m_writer.channelCount = binding.channelCount;
            m_writer.slaveIndex = binding.slaveIndex;
            m_writer.inputBitOffset = binding.inputBitOffset;
            m_writer.configured = binding.configured;
            m_writer.unsignedRaw = binding.unsignedRaw;
            m_writer.reason = binding.reason;
            Publish();
        }

        // Only valid LRW + a monotonic QPC timestamp creates a sample. Invalid
        // cycles publish their status immediately while retaining last-good
        // raw/sequence/capture time; an unchanged voltage is still a new sample.
        void ObserveFromPdo(std::int32_t rawCode, bool readSucceeded,
            std::int64_t qpcTicks, std::uint64_t frequency, bool transportValid) noexcept
        {
            if (!m_writer.configured) return;
            m_writer.transportValid = transportValid;
            std::uint64_t nowMs = 0ULL;
            m_writer.clockValid = QpcMilliseconds(qpcTicks, frequency, nowMs) && nowMs >= m_writer.observedAtMs;
            if (m_writer.clockValid) m_writer.observedAtMs = nowMs;
            if (!transportValid) m_writer.reason = Reason::TransportInvalid;
            else if (!m_writer.clockValid) m_writer.reason = Reason::ClockInvalid;
            else if (!readSucceeded) m_writer.reason = Reason::ReadError;
            else
            {
                m_writer.rawCode = rawCode;
                m_writer.capturedAtMs = nowMs;
                if (m_writer.sequence != (std::numeric_limits<std::uint64_t>::max)()) ++m_writer.sequence;
                m_writer.hasSample = true;
                m_writer.reason = Reason::Valid;
            }
            Publish();
        }

        bool Read(Snapshot& result) const noexcept
        {
            for (std::uint32_t attempt = 0U; attempt < 3U; ++attempt)
            {
                const std::uint64_t before = m_sequence.load(std::memory_order_acquire);
                if ((before & 1ULL) != 0ULL) continue;
                std::uint64_t words[WordCount]{};
                for (std::size_t i = 0U; i < WordCount; ++i) words[i] = m_words[i].load(std::memory_order_relaxed);
                std::atomic_thread_fence(std::memory_order_acquire);
                const std::uint64_t after = m_sequence.load(std::memory_order_acquire);
                if (before == after && (after & 1ULL) == 0ULL)
                {
                    std::memcpy(&result, words, sizeof(result));
                    return true;
                }
            }
            result = Snapshot{};
            result.reason = Reason::SnapshotBusy;
            return false;
        }

        static bool QpcMilliseconds(std::int64_t ticks, std::uint64_t frequency, std::uint64_t& nowMs) noexcept
        {
            nowMs = 0ULL;
            const std::uint64_t maximum = (std::numeric_limits<std::uint64_t>::max)();
            if (ticks < 0 || frequency == 0ULL || frequency > maximum / 1000ULL) return false;
            const std::uint64_t value = static_cast<std::uint64_t>(ticks);
            const std::uint64_t seconds = value / frequency;
            const std::uint64_t fraction = (value % frequency) * 1000ULL / frequency;
            if (seconds > (maximum - fraction) / 1000ULL) return false;
            nowMs = seconds * 1000ULL + fraction;
            return true;
        }

    private:
        static constexpr std::size_t WordCount = (sizeof(Snapshot) + sizeof(std::uint64_t) - 1U) / sizeof(std::uint64_t);
        void Publish() noexcept
        {
            std::uint64_t words[WordCount]{};
            std::memcpy(words, &m_writer, sizeof(m_writer));
            const std::uint64_t odd = m_sequence.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
            for (std::size_t i = 0U; i < WordCount; ++i) m_words[i].store(words[i], std::memory_order_relaxed);
            m_sequence.store(odd + 1ULL, std::memory_order_release);
        }
        Snapshot m_writer{};
        alignas(8) std::atomic<std::uint64_t> m_sequence{0ULL};
        alignas(8) std::atomic<std::uint64_t> m_words[WordCount]{};
    };
}
