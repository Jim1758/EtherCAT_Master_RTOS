#include "NCPathCoreCommandedChordStore.h"
#include <cstring>
#include <limits>

namespace
{
    using Store = NCPathCoreCommandedChordStoreV1;
    using Code = NCPathCoreCommandedChordStoreCode;
    using State = NCPathCoreCommandedChordStoreState;
    using Segment = NCPathCoreCommandedChordSegmentV1;
    using Pair = NCPathCoreAcceptedInputPairRelation;

    bool SameValue(const Segment& a, const Segment& b) noexcept
    {
        // Compare semantic fields and endpoint REPRESENTATIONS, not padding.
        if (a.localIdentity.geometryPublicationSequence != b.localIdentity.geometryPublicationSequence ||
            a.localIdentity.acceptedInputChainGeneration != b.localIdentity.acceptedInputChainGeneration ||
            a.axisMask != b.axisMask || a.acceptedInputRunLength != b.acceptedInputRunLength ||
            a.schemaVersion != b.schemaVersion || a.sourcePairRelation != b.sourcePairRelation ||
            a.kind != b.kind || a.frame != b.frame || a.extent != b.extent ||
            a.reserved[0U] != b.reserved[0U] || a.reserved[1U] != b.reserved[1U])
            return false;
        for (std::size_t i = 0U; i < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++i)
        {
            if (std::memcmp(&a.startMCS[i], &b.startMCS[i], sizeof(double)) != 0 ||
                std::memcmp(&a.endMCS[i], &b.endMCS[i], sizeof(double)) != 0)
                return false;
        }
        return true;
    }

    bool Joined(const Segment& tail, const Segment& next) noexcept
    {
        for (std::size_t i = 0U; i < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++i)
            if (tail.endMCS[i] != next.startMCS[i]) return false;
        return true;
    }
}

constexpr std::uint32_t NCPathCoreCommandedChordStoreV1::Capacity;

NCPathCoreCommandedChordStoreV1::NCPathCoreCommandedChordStoreV1(
    const std::uint64_t ownerTag) noexcept : m_ownerTag(ownerTag)
{
}

void Store::ClearPayload() noexcept
{
    // Lifecycle work only; never call from PDO or a high-frequency observer.
    for (std::size_t i = 0U; i < m_values.size(); ++i) m_values[i].Clear();
    m_count = 0U;
}

Code Store::Fault(const Code code) noexcept
{
    if (m_state != State::FAULTED) m_firstFault = code;
    m_state = State::FAULTED;
    return code;
}

Code Store::BeginLifetime(const std::uint64_t lifetime) noexcept
{
    if (m_ownerTag == 0ULL) return Fault(Code::INVALID_OWNER);
    if (m_lastAcceptedLifetime == (std::numeric_limits<std::uint64_t>::max)())
        return Fault(Code::LIFETIME_EXHAUSTED);
    if (lifetime == 0ULL) return Fault(Code::INVALID_LIFETIME);
    if (lifetime <= m_lastAcceptedLifetime) return Fault(Code::LIFETIME_NOT_NEW);
    ClearPayload();
    m_lastAcceptedLifetime = lifetime;
    m_firstFault = Code::NONE;
    m_state = State::OPEN;
    return Code::OPENED;
}

void Store::Invalidate() noexcept
{
    ClearPayload();
    m_state = State::CLOSED;
    m_firstFault = Code::NONE;
    // Do NOT clear lifetime high-water mark or owner tag.
}

void Store::WriteHandle(const std::uint32_t ordinal,
    NCPathCoreCommandedChordStoreHandleV1& output) const noexcept
{
    output.ownerTag = m_ownerTag;
    output.lifetime = m_lastAcceptedLifetime;
    output.ordinal = ordinal;
    output.reserved = 0U;
    output.localIdentity = m_values[ordinal - 1U].localIdentity;
}

Code Store::Append(const Segment& value,
    NCPathCoreCommandedChordStoreHandleV1& output) noexcept
{
    output.Clear();
    if (m_state != State::OPEN) return Code::NOT_OPEN;
    if (!IsValidCommandedChordSegmentValue(value)) return Fault(Code::INVALID_VALUE);
    if (m_count == 0U)
    {
        if ((value.sourcePairRelation != Pair::FIRST_INPUT &&
            value.sourcePairRelation != Pair::CHAIN_BOUNDARY) ||
            value.acceptedInputRunLength != 1U)
            return Fault(Code::MISSING_HEAD);
    }
    else
    {
        const Segment& tail = m_values[m_count - 1U];
        if (HasSameCommandedChordLocalIdentitySameOwnerLifetime(tail.localIdentity, value.localIdentity))
        {
            if (!SameValue(tail, value)) return Fault(Code::KEY_CONFLICT);
            WriteHandle(m_count, output);
            return Code::ALREADY_RETAINED;
        }
        if (value.localIdentity.acceptedInputChainGeneration !=
            tail.localIdentity.acceptedInputChainGeneration)
            return Fault(Code::CHAIN_CHANGE);
        const std::uint64_t nextPublication =
            tail.localIdentity.geometryPublicationSequence == (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL : tail.localIdentity.geometryPublicationSequence + 1ULL;
        if (value.localIdentity.geometryPublicationSequence != nextPublication)
            return Fault(Code::SOURCE_GAP);
        const std::uint32_t nextRun =
            tail.acceptedInputRunLength == (std::numeric_limits<std::uint32_t>::max)()
            ? tail.acceptedInputRunLength : tail.acceptedInputRunLength + 1U;
        if (value.sourcePairRelation != Pair::CONTIGUOUS_PAIR ||
            value.acceptedInputRunLength != nextRun)
            return Fault(Code::RUN_DISCONTINUITY);
        if (!Joined(tail, value)) return Fault(Code::DISCONNECTED_CHORD);
    }
    if (m_count >= Capacity) return Fault(Code::CAPACITY_EXCEEDED);
    m_values[m_count] = value; // Direct copy; no segment-size stack temporary.
    ++m_count;
    WriteHandle(m_count, output);
    return Code::APPENDED;
}

Code Store::GetHandleAtOrdinal(const std::uint32_t ordinal,
    NCPathCoreCommandedChordStoreHandleV1& output) const noexcept
{
    output.Clear();
    if (m_state != State::OPEN) return Code::NOT_OPEN;
    if (ordinal == 0U || ordinal > m_count) return Code::ORDINAL_OUTSIDE;
    if (!IsValidCommandedChordSegmentValue(m_values[ordinal - 1U]))
        return Code::RETAINED_VALUE_INVALID;
    WriteHandle(ordinal, output);
    return Code::HANDLE_READ;
}

Code Store::Read(const NCPathCoreCommandedChordStoreHandleV1& handle,
    Segment& output) const noexcept
{
    output.Clear();
    if (m_state != State::OPEN) return Code::NOT_OPEN;
    if (handle.ownerTag != m_ownerTag || handle.lifetime != m_lastAcceptedLifetime ||
        handle.reserved != 0U || handle.ordinal == 0U || handle.ordinal > m_count)
        return Code::HANDLE_REJECTED;
    const Segment& value = m_values[handle.ordinal - 1U];
    if (!HasSameCommandedChordLocalIdentitySameOwnerLifetime(handle.localIdentity, value.localIdentity))
        return Code::HANDLE_REJECTED;
    if (!IsValidCommandedChordSegmentValue(value)) return Code::RETAINED_VALUE_INVALID;
    output = value;
    return Code::VALUE_READ;
}

void Store::Describe(NCPathCoreCommandedChordStoreInfoV1& output) const noexcept
{
    output.Clear();
    output.ownerTag = m_ownerTag;
    output.lastAcceptedLifetime = m_lastAcceptedLifetime;
    output.storedCount = m_count;
    output.readableCount = m_state == State::OPEN ? m_count : 0U;
    output.capacity = Capacity;
    output.state = m_state;
    output.firstFault = m_firstFault;
}
