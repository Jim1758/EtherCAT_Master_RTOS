#include "NCBlockLifecycleLedger.h"

#include <algorithm>
#include <limits>

namespace
{
    double ClampBlockProgress(double value) noexcept
    {
        if (!(value >= 0.0))
        {
            return 0.0;
        }
        if (value > 1.0)
        {
            return 1.0;
        }
        return value;
    }

    bool SameMotionIdentity(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId;
    }
}

bool NCBlockLifecycleLedger::IsProgramTargetValid(
    const NCProgramCommitSnapshot& target) noexcept
{
    return
        target.scope != NCProgramScope::NONE &&
        target.cacheGeneration != NC_PROGRAM_CACHE_GENERATION_INVALID &&
        target.sourcePC >= 0 &&
        (target.scope != NCProgramScope::MACRO ||
            target.frameId != NC_PROGRAM_FRAME_ID_INVALID);
}

std::size_t NCBlockLifecycleLedger::BlockSlotIndex(
    NCBlockDispatchId dispatchId) noexcept
{
    return static_cast<std::size_t>(
        dispatchId % NC_BLOCK_LIFECYCLE_CAPACITY);
}

std::size_t NCBlockLifecycleLedger::SegmentSlotIndex(
    MotionSegmentId segmentId) noexcept
{
    return static_cast<std::size_t>(
        segmentId % NC_BLOCK_SEGMENT_INDEX_CAPACITY);
}

NCBlockDispatchId NCBlockLifecycleLedger::AllocateDispatchId() noexcept
{
    NCBlockDispatchId id = m_nextDispatchId++;
    if (id == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        id = m_nextDispatchId++;
    }
    return id;
}

NCBlockLifecycleSnapshot* NCBlockLifecycleLedger::FindMutable(
    NCBlockDispatchId dispatchId) noexcept
{
    if (dispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        return nullptr;
    }

    NCBlockLifecycleSnapshot& snapshot =
        m_blocks[BlockSlotIndex(dispatchId)];
    return snapshot.dispatchId == dispatchId ? &snapshot : nullptr;
}

const NCBlockLifecycleSnapshot* NCBlockLifecycleLedger::Find(
    NCBlockDispatchId dispatchId) const noexcept
{
    if (dispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        return nullptr;
    }

    const NCBlockLifecycleSnapshot& snapshot =
        m_blocks[BlockSlotIndex(dispatchId)];
    return snapshot.dispatchId == dispatchId ? &snapshot : nullptr;
}

void NCBlockLifecycleLedger::ClearSegmentBindings(
    const NCBlockLifecycleSnapshot& snapshot) noexcept
{
    for (std::uint16_t i = 0U;
        i < snapshot.motionSegmentCount &&
        i < NC_BLOCK_MAX_MOTION_SEGMENTS;
        ++i)
    {
        const MotionExecutionIdentity& identity =
            snapshot.motionSegments[i].identity;
        if (!identity.IsAssigned())
        {
            continue;
        }

        SegmentIndexEntry& index =
            m_segmentIndex[SegmentSlotIndex(identity.segmentId)];
        if (index.valid &&
            index.dispatchId == snapshot.dispatchId &&
            SameMotionIdentity(index.identity, identity))
        {
            index = SegmentIndexEntry{};
        }
    }
}

NCBlockDispatchId NCBlockLifecycleLedger::BeginBlock(
    const NCProgramCommitSnapshot& programTarget,
    int sourceLineNumber) noexcept
{
    if (!IsProgramTargetValid(programTarget))
    {
        return NC_BLOCK_DISPATCH_ID_INVALID;
    }

    const NCBlockDispatchId dispatchId = AllocateDispatchId();
    NCBlockLifecycleSnapshot& slot =
        m_blocks[BlockSlotIndex(dispatchId)];

    if (slot.IsValid())
    {
        ClearSegmentBindings(slot);
        if (!slot.terminal)
        {
            ++m_counters.activeBlockOverwrite;
            if (m_counters.activeBlocks > 0U)
            {
                --m_counters.activeBlocks;
            }
        }
    }

    slot = NCBlockLifecycleSnapshot{};
    slot.dispatchId = dispatchId;
    slot.programTarget = programTarget;
    slot.sourceLineNumber = sourceLineNumber;
    slot.state = NCBlockLifecycleState::DISPATCHED;

    ++m_counters.dispatched;
    ++m_counters.activeBlocks;
    m_lastDispatchedId = dispatchId;
    return dispatchId;
}

bool NCBlockLifecycleLedger::MarkProgramCommitted(
    NCBlockDispatchId dispatchId,
    const NCProgramCommitSnapshot& programCommit) noexcept
{
    NCBlockLifecycleSnapshot* snapshot = FindMutable(dispatchId);
    if (snapshot == nullptr || !programCommit.IsValid())
    {
        return false;
    }

    if (!snapshot->programCommitted)
    {
        ++m_counters.programCommitted;
    }

    snapshot->programCommit = programCommit;
    snapshot->programCommitted = true;
    m_lastProgramCommittedId = dispatchId;
    RefreshAggregate(*snapshot);
    return true;
}

void NCBlockLifecycleLedger::IndexMotionSegment(
    NCBlockDispatchId dispatchId,
    std::uint16_t segmentIndex,
    const MotionExecutionIdentity& identity) noexcept
{
    SegmentIndexEntry& slot =
        m_segmentIndex[SegmentSlotIndex(identity.segmentId)];

    if (slot.valid &&
        (!SameMotionIdentity(slot.identity, identity) ||
            slot.dispatchId != dispatchId))
    {
        const NCBlockLifecycleSnapshot* oldBlock =
            Find(slot.dispatchId);
        if (oldBlock != nullptr && !oldBlock->terminal)
        {
            ++m_counters.activeSegmentIndexOverwrite;
        }
    }

    slot.identity = identity;
    slot.dispatchId = dispatchId;
    slot.segmentIndex = segmentIndex;
    slot.valid = true;
}

bool NCBlockLifecycleLedger::BindMotionSegment(
    NCBlockDispatchId dispatchId,
    const MotionExecutionIdentity& identity,
    bool producerAccepted,
    MotionRejectReason immediateRejectReason) noexcept
{
    NCBlockLifecycleSnapshot* snapshot = FindMutable(dispatchId);
    if (snapshot == nullptr || !identity.IsAssigned())
    {
        return false;
    }

    for (std::uint16_t i = 0U;
        i < snapshot->motionSegmentCount;
        ++i)
    {
        if (SameMotionIdentity(
            snapshot->motionSegments[i].identity,
            identity))
        {
            return true;
        }
    }

    if (snapshot->motionSegmentCount >=
        NC_BLOCK_MAX_MOTION_SEGMENTS)
    {
        MarkMotionCaptureOverflow(dispatchId);
        return false;
    }

    const bool firstMotion = snapshot->motionSegmentCount == 0U;
    const std::uint16_t segmentIndex = snapshot->motionSegmentCount++;
    NCBlockMotionSegmentSnapshot& segment =
        snapshot->motionSegments[segmentIndex];

    segment.identity = identity;
    segment.producerAccepted = producerAccepted;
    segment.immediateRejectReason = immediateRejectReason;
    segment.lastRejectReason = immediateRejectReason;

    if (producerAccepted)
    {
        ++m_counters.producerAccepted;
    }
    else
    {
        ++m_counters.producerRejected;
        segment.terminal = true;
        segment.rejected = true;
        segment.lastFeedbackType = MotionFeedbackType::REJECTED;
    }

    if (firstMotion)
    {
        ++m_counters.motionBlocks;
    }
    ++m_counters.motionSegmentsBound;

    IndexMotionSegment(dispatchId, segmentIndex, identity);
    RefreshAggregate(*snapshot);
    return true;
}

bool NCBlockLifecycleLedger::MarkMotionCaptureOverflow(
    NCBlockDispatchId dispatchId) noexcept
{
    NCBlockLifecycleSnapshot* snapshot = FindMutable(dispatchId);
    if (snapshot == nullptr)
    {
        return false;
    }

    if (!snapshot->motionCaptureOverflow)
    {
        snapshot->motionCaptureOverflow = true;
        ++m_counters.motionCaptureOverflow;
    }

    RefreshAggregate(*snapshot);
    return true;
}

bool NCBlockLifecycleLedger::MarkNCDispatchFailed(
    NCBlockDispatchId dispatchId,
    std::uint32_t errorCode) noexcept
{
    NCBlockLifecycleSnapshot* snapshot = FindMutable(dispatchId);
    if (snapshot == nullptr)
    {
        return false;
    }

    snapshot->ncDispatchFailed = true;
    snapshot->ncErrorCode = errorCode;
    if (!snapshot->terminal)
    {
        ++m_counters.ncDispatchFailed;
        Finalize(
            *snapshot,
            NCBlockLifecycleState::NC_DISPATCH_FAILED,
            false);
    }
    return true;
}

NCBlockMotionSegmentSnapshot* NCBlockLifecycleLedger::FindMotionSegment(
    const MotionExecutionIdentity& identity,
    NCBlockLifecycleSnapshot*& block) noexcept
{
    block = nullptr;
    if (!identity.IsAssigned())
    {
        return nullptr;
    }

    SegmentIndexEntry& index =
        m_segmentIndex[SegmentSlotIndex(identity.segmentId)];
    if (!index.valid ||
        !SameMotionIdentity(index.identity, identity))
    {
        return nullptr;
    }

    NCBlockLifecycleSnapshot* snapshot =
        FindMutable(index.dispatchId);
    if (snapshot == nullptr ||
        index.segmentIndex >= snapshot->motionSegmentCount)
    {
        return nullptr;
    }

    NCBlockMotionSegmentSnapshot& segment =
        snapshot->motionSegments[index.segmentIndex];
    if (!SameMotionIdentity(segment.identity, identity))
    {
        return nullptr;
    }

    block = snapshot;
    return &segment;
}

bool NCBlockLifecycleLedger::ApplyMotionFeedback(
    const MotionFeedbackEvent& event) noexcept
{
    NCBlockLifecycleSnapshot* block = nullptr;
    NCBlockMotionSegmentSnapshot* segment =
        FindMotionSegment(event.identity, block);

    if (segment == nullptr || block == nullptr)
    {
        ++m_counters.orphanFeedback;
        return false;
    }

    if (event.sequence != MOTION_FEEDBACK_SEQUENCE_INVALID &&
        segment->lastFeedbackSequence == event.sequence)
    {
        return true;
    }

    // Producer 端 Queue Full / Owner Conflict 會先同步標記 REJECTED，
    // Runtime 之後仍會透過 Producer Notice Ring 回報正式 REJECTED。
    // 第一筆正式確認不是重複 Terminal，也不應產生 Conflict。
    const bool confirmsProducerRejection =
        segment->terminal &&
        segment->rejected &&
        !segment->producerAccepted &&
        segment->feedbackEventCount == 0U &&
        event.type == MotionFeedbackType::REJECTED;

    segment->lastFeedbackSequence = event.sequence;
    segment->lastFeedbackType = event.type;
    segment->lastRejectReason = event.rejectReason;
    segment->owner = event.owner;
    segment->ownerGeneration = event.ownerGeneration;
    segment->errorCode = event.errorCode;
    segment->progress = ClampBlockProgress(event.progress);
    ++segment->feedbackEventCount;

    switch (event.type)
    {
    case MotionFeedbackType::ACCEPTED:
        ++m_counters.feedbackAccepted;
        segment->runtimeAccepted = true;
        break;

    case MotionFeedbackType::STARTED:
        ++m_counters.feedbackStarted;
        segment->started = true;
        segment->held = false;
        break;

    case MotionFeedbackType::PROGRESS:
        segment->started = true;
        break;

    case MotionFeedbackType::HELD:
        segment->held = true;
        break;

    case MotionFeedbackType::RESUMED:
        segment->started = true;
        segment->held = false;
        break;

    case MotionFeedbackType::COMPLETED:
        ++m_counters.feedbackCompleted;
        if (segment->terminal)
        {
            ++m_counters.duplicateTerminalFeedback;
            if (!segment->completed)
            {
                ++m_counters.terminalFeedbackConflict;
            }
        }
        else
        {
            segment->terminal = true;
            segment->completed = true;
            segment->progress = 1.0;
        }
        break;

    case MotionFeedbackType::REJECTED:
        ++m_counters.feedbackRejected;
        if (segment->terminal)
        {
            if (confirmsProducerRejection)
            {
                if (segment->immediateRejectReason != MotionRejectReason::NONE &&
                    event.rejectReason != MotionRejectReason::NONE &&
                    segment->immediateRejectReason != event.rejectReason)
                {
                    ++m_counters.terminalFeedbackConflict;
                }
            }
            else
            {
                ++m_counters.duplicateTerminalFeedback;
                if (!segment->rejected)
                {
                    ++m_counters.terminalFeedbackConflict;
                }
            }
        }
        else
        {
            segment->terminal = true;
            segment->rejected = true;
        }
        break;

    case MotionFeedbackType::CANCELLED:
        ++m_counters.feedbackCancelled;
        if (segment->terminal)
        {
            ++m_counters.duplicateTerminalFeedback;
            if (!segment->cancelled)
            {
                ++m_counters.terminalFeedbackConflict;
            }
        }
        else
        {
            segment->terminal = true;
            segment->cancelled = true;
        }
        break;

    case MotionFeedbackType::ABORTED:
        ++m_counters.feedbackAborted;
        if (segment->terminal)
        {
            ++m_counters.duplicateTerminalFeedback;
            if (!segment->aborted)
            {
                ++m_counters.terminalFeedbackConflict;
            }
        }
        else
        {
            segment->terminal = true;
            segment->aborted = true;
        }
        break;

    case MotionFeedbackType::FAULTED:
        ++m_counters.feedbackFaulted;
        if (segment->terminal)
        {
            ++m_counters.duplicateTerminalFeedback;
            if (!segment->faulted)
            {
                ++m_counters.terminalFeedbackConflict;
            }
        }
        else
        {
            segment->terminal = true;
            segment->faulted = true;
        }
        break;

    case MotionFeedbackType::NONE:
    default:
        break;
    }

    RefreshAggregate(*block);
    return true;
}

void NCBlockLifecycleLedger::RefreshAggregate(
    NCBlockLifecycleSnapshot& snapshot) noexcept
{
    snapshot.motionAcceptedCount = 0U;
    snapshot.motionStartedCount = 0U;
    snapshot.motionCompletedCount = 0U;
    snapshot.motionTerminalCount = 0U;
    snapshot.motionFailedCount = 0U;

    bool anyHeld = false;
    bool anyFaulted = false;
    bool anyAborted = false;
    bool anyCancelled = false;
    bool anyRejected = false;

    for (std::uint16_t i = 0U;
        i < snapshot.motionSegmentCount;
        ++i)
    {
        const NCBlockMotionSegmentSnapshot& segment =
            snapshot.motionSegments[i];
        if (segment.runtimeAccepted)
        {
            ++snapshot.motionAcceptedCount;
        }
        if (segment.started)
        {
            ++snapshot.motionStartedCount;
        }
        if (segment.completed)
        {
            ++snapshot.motionCompletedCount;
        }
        if (segment.terminal)
        {
            ++snapshot.motionTerminalCount;
        }
        if (segment.rejected || segment.cancelled ||
            segment.aborted || segment.faulted)
        {
            ++snapshot.motionFailedCount;
        }

        anyHeld = anyHeld || segment.held;
        anyFaulted = anyFaulted || segment.faulted;
        anyAborted = anyAborted || segment.aborted;
        anyCancelled = anyCancelled || segment.cancelled;
        anyRejected = anyRejected || segment.rejected;
    }

    if (snapshot.terminal)
    {
        return;
    }

    if (snapshot.ncDispatchFailed)
    {
        Finalize(
            snapshot,
            NCBlockLifecycleState::NC_DISPATCH_FAILED,
            false);
        return;
    }

    if (snapshot.programCommitted && snapshot.motionSegmentCount == 0U)
    {
        if (snapshot.motionCaptureOverflow)
        {
            Finalize(
                snapshot,
                NCBlockLifecycleState::TRACKING_OVERFLOW,
                false);
        }
        else
        {
            ++m_counters.programOnlyCompleted;
            Finalize(
                snapshot,
                NCBlockLifecycleState::PROGRAM_ONLY_COMPLETED,
                true);
        }
        return;
    }

    const bool allMotionTerminal =
        snapshot.motionSegmentCount != 0U &&
        snapshot.motionTerminalCount == snapshot.motionSegmentCount;

    if (snapshot.programCommitted && allMotionTerminal)
    {
        if (snapshot.motionCaptureOverflow)
        {
            Finalize(
                snapshot,
                NCBlockLifecycleState::TRACKING_OVERFLOW,
                false);
        }
        else if (snapshot.motionCompletedCount ==
            snapshot.motionSegmentCount)
        {
            m_lastMotionCompletedId = snapshot.dispatchId;
            Finalize(
                snapshot,
                NCBlockLifecycleState::MOTION_COMPLETED,
                true);
        }
        else if (anyFaulted)
        {
            Finalize(snapshot, NCBlockLifecycleState::MOTION_FAULTED, false);
        }
        else if (anyAborted)
        {
            Finalize(snapshot, NCBlockLifecycleState::MOTION_ABORTED, false);
        }
        else if (anyCancelled)
        {
            Finalize(snapshot, NCBlockLifecycleState::MOTION_CANCELLED, false);
        }
        else
        {
            Finalize(snapshot, NCBlockLifecycleState::MOTION_REJECTED, false);
        }
        return;
    }

    if (anyHeld)
    {
        snapshot.state = NCBlockLifecycleState::MOTION_HELD;
    }
    else if (snapshot.motionStartedCount != 0U)
    {
        snapshot.state = NCBlockLifecycleState::MOTION_ACTIVE;
    }
    else if (snapshot.motionAcceptedCount != 0U)
    {
        snapshot.state = NCBlockLifecycleState::MOTION_ACCEPTED;
    }
    else if (snapshot.motionSegmentCount != 0U)
    {
        snapshot.state = NCBlockLifecycleState::MOTION_PENDING;
    }
    else if (snapshot.programCommitted)
    {
        snapshot.state = NCBlockLifecycleState::PROGRAM_COMMITTED;
    }
    else
    {
        snapshot.state = NCBlockLifecycleState::DISPATCHED;
    }
}

void NCBlockLifecycleLedger::Finalize(
    NCBlockLifecycleSnapshot& snapshot,
    NCBlockLifecycleState terminalState,
    bool success) noexcept
{
    if (snapshot.terminal)
    {
        return;
    }

    snapshot.state = terminalState;
    snapshot.terminal = true;
    snapshot.terminalSuccess = success;
    snapshot.terminalFailure = !success;

    if (m_counters.activeBlocks > 0U)
    {
        --m_counters.activeBlocks;
    }

    if (success)
    {
        ++m_counters.blockCompleted;
    }
    else
    {
        ++m_counters.blockFailed;
    }

    m_lastTerminalId = snapshot.dispatchId;
}

void NCBlockLifecycleLedger::CopySnapshot(
    NCBlockDispatchId dispatchId,
    NCBlockLifecycleSnapshot& snapshot,
    bool& result) const noexcept
{
    const NCBlockLifecycleSnapshot* found = Find(dispatchId);
    if (found == nullptr)
    {
        result = false;
        return;
    }

    snapshot = *found;
    result = true;
}

bool NCBlockLifecycleLedger::TryGetSnapshot(
    NCBlockDispatchId dispatchId,
    NCBlockLifecycleSnapshot& snapshot) const noexcept
{
    bool result = false;
    CopySnapshot(dispatchId, snapshot, result);
    return result;
}

bool NCBlockLifecycleLedger::GetLastDispatchedSnapshot(
    NCBlockLifecycleSnapshot& snapshot) const noexcept
{
    return TryGetSnapshot(m_lastDispatchedId, snapshot);
}

bool NCBlockLifecycleLedger::GetLastProgramCommittedSnapshot(
    NCBlockLifecycleSnapshot& snapshot) const noexcept
{
    return TryGetSnapshot(m_lastProgramCommittedId, snapshot);
}

bool NCBlockLifecycleLedger::GetLastMotionCompletedSnapshot(
    NCBlockLifecycleSnapshot& snapshot) const noexcept
{
    return TryGetSnapshot(m_lastMotionCompletedId, snapshot);
}

bool NCBlockLifecycleLedger::GetLastTerminalSnapshot(
    NCBlockLifecycleSnapshot& snapshot) const noexcept
{
    return TryGetSnapshot(m_lastTerminalId, snapshot);
}


bool NCBlockLifecycleLedger::GetMotionBoundarySnapshot(
    NCBlockDispatchId dispatchId,
    NCBlockMotionBoundarySnapshot& boundary) const noexcept
{
    const NCBlockLifecycleSnapshot* snapshot = Find(dispatchId);
    if (snapshot == nullptr)
    {
        return false;
    }

    boundary = NCBlockMotionBoundarySnapshot{};
    boundary.dispatchId = snapshot->dispatchId;
    boundary.lifecycleState = snapshot->state;
    boundary.motionSegmentCount = snapshot->motionSegmentCount;
    boundary.motionCompletedCount = snapshot->motionCompletedCount;
    boundary.motionTerminalCount = snapshot->motionTerminalCount;
    boundary.motionFailedCount = snapshot->motionFailedCount;
    boundary.programCommitted = snapshot->programCommitted;
    boundary.lifecycleTerminal = snapshot->terminal;
    boundary.lifecycleSuccess = snapshot->terminalSuccess;

    if (snapshot->motionCaptureOverflow ||
        snapshot->state == NCBlockLifecycleState::TRACKING_OVERFLOW)
    {
        boundary.state = NCBlockMotionBoundaryState::TRACKING_OVERFLOW;
    }
    else if (snapshot->motionFailedCount != 0U ||
        snapshot->ncDispatchFailed ||
        (snapshot->terminal && snapshot->terminalFailure))
    {
        boundary.state = NCBlockMotionBoundaryState::FAILED;
    }
    else if (snapshot->motionSegmentCount == 0U)
    {
        boundary.state = NCBlockMotionBoundaryState::NOT_TRACKED;
    }
    else if (snapshot->programCommitted &&
        snapshot->motionTerminalCount == snapshot->motionSegmentCount &&
        snapshot->motionCompletedCount == snapshot->motionSegmentCount)
    {
        boundary.state = NCBlockMotionBoundaryState::SUCCEEDED;
    }
    else
    {
        boundary.state = NCBlockMotionBoundaryState::PENDING;
    }

    return true;
}
