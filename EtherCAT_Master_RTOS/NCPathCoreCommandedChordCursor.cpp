#include "NCPathCoreCommandedChordCursor.h"

namespace
{
    using Store = NCPathCoreCommandedChordStoreV1;
    using StoreCode = NCPathCoreCommandedChordStoreCode;
    using State = NCPathCoreCommandedChordStoreState;
    using Info = NCPathCoreCommandedChordStoreInfoV1;
    using Handle = NCPathCoreCommandedChordStoreHandleV1;
    using Cursor = NCPathCoreCommandedChordCursorV1;
    using Code = NCPathCoreCommandedChordCursorCode;

    Code DescribeOpen(const Store& store, Info& info) noexcept
    {
        store.Describe(info);
        switch (info.state)
        {
        case State::OPEN: return Code::NONE;
        case State::CLOSED: return Code::STORE_CLOSED;
        case State::FAULTED: return Code::STORE_FAULTED;
        default: return Code::STORE_REJECTED;
        }
    }

    Code MapUnexpectedStoreRejection(const StoreCode code) noexcept
    {
        return code == StoreCode::RETAINED_VALUE_INVALID
            ? Code::RETAINED_VALUE_INVALID : Code::STORE_REJECTED;
    }

    bool SameBinding(const Handle& left, const Handle& right) noexcept
    {
        // Compare fields, never padding. Caller establishes owner uniqueness.
        return left.ownerTag == right.ownerTag &&
            left.lifetime == right.lifetime &&
            left.ordinal == right.ordinal &&
            left.reserved == right.reserved &&
            left.localIdentity.geometryPublicationSequence ==
            right.localIdentity.geometryPublicationSequence &&
            left.localIdentity.acceptedInputChainGeneration ==
            right.localIdentity.acceptedInputChainGeneration;
    }

    Code ValidateCurrent(const Store& store, const Cursor& current,
        Info& info) noexcept
    {
        const Code stateCode = DescribeOpen(store, info);
        if (stateCode != Code::NONE) return stateCode;
        const Handle& handle = current.binding;
        if (handle.ownerTag == 0ULL || handle.ownerTag != info.ownerTag ||
            handle.lifetime == 0ULL || handle.lifetime != info.lastAcceptedLifetime ||
            handle.reserved != 0U || handle.ordinal == 0U ||
            handle.ordinal > info.readableCount ||
            handle.localIdentity.geometryPublicationSequence == 0ULL ||
            handle.localIdentity.acceptedInputChainGeneration == 0ULL)
            return Code::CURSOR_REJECTED;

        Handle fresh{}; // Only 40 bytes; no Segment snapshot for validation.
        const StoreCode code = store.GetHandleAtOrdinal(handle.ordinal, fresh);
        if (code != StoreCode::HANDLE_READ) return MapUnexpectedStoreRejection(code);
        return SameBinding(handle, fresh) ? Code::NONE : Code::CURSOR_REJECTED;
    }

    Code Bind(const Store& store, const std::uint32_t requestedOrdinal,
        const bool useTail, Cursor& output) noexcept
    {
        output.Clear();
        Info info{};
        const Code stateCode = DescribeOpen(store, info);
        if (stateCode != Code::NONE) return stateCode;
        if (info.readableCount == 0U) return Code::EMPTY_STORE;
        const std::uint32_t ordinal = useTail ? info.readableCount : requestedOrdinal;
        if (ordinal == 0U || ordinal > info.readableCount) return Code::ORDINAL_OUTSIDE;
        const StoreCode code = store.GetHandleAtOrdinal(ordinal, output.binding);
        if (code == StoreCode::HANDLE_READ) return Code::BOUND;
        output.Clear();
        return MapUnexpectedStoreRejection(code);
    }

    Code Move(const Store& store, const Cursor& current, const bool forward,
        Cursor& output) noexcept
    {
        output.Clear();
        Info info{};
        const Code validation = ValidateCurrent(store, current, info);
        if (validation != Code::NONE) return validation;

        // Only a valid current binding can produce a normal boundary result.
        const std::uint32_t ordinal = current.binding.ordinal;
        if ((forward && ordinal == info.readableCount) || (!forward && ordinal == 1U))
        {
            output = current;
            return forward ? Code::AT_TAIL : Code::AT_HEAD;
        }
        // Valid current/neighbor ordinals lie in [1,32]; never wrap or scan.
        const std::uint32_t targetOrdinal = forward ? ordinal + 1U : ordinal - 1U;
        const StoreCode code = store.GetHandleAtOrdinal(targetOrdinal, output.binding);
        if (code == StoreCode::HANDLE_READ)
            return forward ? Code::MOVED_NEXT : Code::MOVED_PREVIOUS;
        output.Clear();
        return MapUnexpectedStoreRejection(code);
    }
}

NCPathCoreCommandedChordCursorCode BindCommandedChordCursorHead(
    const NCPathCoreCommandedChordStoreV1& store,
    NCPathCoreCommandedChordCursorV1& output) noexcept
{
    return Bind(store, 1U, false, output);
}

NCPathCoreCommandedChordCursorCode BindCommandedChordCursorTail(
    const NCPathCoreCommandedChordStoreV1& store,
    NCPathCoreCommandedChordCursorV1& output) noexcept
{
    return Bind(store, 0U, true, output);
}

NCPathCoreCommandedChordCursorCode BindCommandedChordCursorAtOrdinal(
    const NCPathCoreCommandedChordStoreV1& store, const std::uint32_t ordinal,
    NCPathCoreCommandedChordCursorV1& output) noexcept
{
    return Bind(store, ordinal, false, output);
}

NCPathCoreCommandedChordCursorCode ReadCommandedChordCursorCurrent(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& current,
    NCPathCoreCommandedChordSegmentV1& output) noexcept
{
    output.Clear();
    Info info{};
    const Code validation = ValidateCurrent(store, current, info);
    if (validation != Code::NONE) return validation;
    // Copy directly into the caller's disjoint heap workspace.
    const StoreCode code = store.Read(current.binding, output);
    if (code == StoreCode::VALUE_READ) return Code::VALUE_READ;
    output.Clear();
    return MapUnexpectedStoreRejection(code);
}

NCPathCoreCommandedChordCursorCode MoveCommandedChordCursorNext(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& current,
    NCPathCoreCommandedChordCursorV1& output) noexcept
{
    return Move(store, current, true, output);
}

NCPathCoreCommandedChordCursorCode MoveCommandedChordCursorPrevious(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& current,
    NCPathCoreCommandedChordCursorV1& output) noexcept
{
    return Move(store, current, false, output);
}
