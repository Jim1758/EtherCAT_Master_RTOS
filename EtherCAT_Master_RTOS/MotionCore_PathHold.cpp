#include "MotionCore.h"
#include <algorithm>
#include <limits>

namespace
{
    bool CBIdentity(const MotionExecutionIdentity& a, const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.source == b.source && a.sourceBlockId == b.sourceBlockId;
    }
    bool CBPositive(double v) noexcept { return std::isfinite(v) && v > 0.0; }
    bool CCEligible(const MotionPathCoreHoldExcursionSnapshot& s) noexcept
    {
        return (s.phase == MotionPathCoreHoldExcursionPhase::ARMED ||
            s.phase == MotionPathCoreHoldExcursionPhase::COMPLETE) &&
            s.cycleLimit >= 1U && s.cycleLimit <= 32U &&
            s.returnCount < s.cycleLimit&& s.retreatCount == s.returnCount;
    }
    bool CLReturnEligible(const MotionPathCoreHoldExcursionSnapshot& s) noexcept
    {
        return s.phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN &&
            s.requireReturnAuthorization && s.cycleLimit == 1U &&
            s.retreatCount == 1ULL && s.returnCount == 0ULL;
    }
    bool CDViewValid(const MotionPathCoreHoldExcursionView& view) noexcept
    {
        if (view.completedCount == 0U || view.completedCount > view.completed.size() ||
            view.validAxisMask == 0U || (view.validAxisMask & ~255U) != 0U ||
            !IsNCPathCoreRetainedGeometryValid(view.original) || view.original.point) return false;
        for (std::uint32_t axis = 0U; axis < 8U; ++axis)
            if ((view.validAxisMask & (1U << axis)) != 0U && !CBPositive(view.pulsePerMM[axis])) return false;
        for (std::uint32_t row = 0U; row <= view.completedCount; ++row)
        {
            const auto& g = row == view.completedCount ? view.original : view.completed[row];
            if (!IsNCPathCoreRetainedGeometryValid(g) || (g.axisMask & ~view.validAxisMask) != 0U ||
                (!g.point && !CBPositive(g.lengthPulse / g.lengthMM))) return false;
            if (row != 0U && !AreNCPathCoreRetainedEndpointsConnected(view.completed[row - 1U],
                g, view.validAxisMask, view.pulsePerMM)) return false;
        }
        return true;
    }
    double CDPulseMagnitude(const NCPathCoreRetainedGeometry& g, std::uint32_t axis) noexcept
    {
        double scale = (std::max)(std::abs(g.startPulse[axis]), std::abs(g.endPulse[axis]));
        if (g.kind == NCPathCoreRetainedKind::ARC && axis < 2U)
            scale = (std::max)(scale, (std::max)(std::abs(g.centerPulse[axis]), g.radiusPulse));
        return scale;
    }
    bool CDPulseNear(double actual, double expected, const NCPathCoreRetainedGeometry& g,
        std::uint32_t axis, double ppm, const NCPathCoreRetainedGeometry* previous = nullptr) noexcept
    {
        double scale = CDPulseMagnitude(g, axis);
        if (previous != nullptr)
        {
            const double previousScale = CDPulseMagnitude(*previous, axis);
            if (!std::isfinite(previousScale)) return false;
            scale = (std::max)(scale, previousScale);
        }
        const double error = std::abs(actual - expected);
        return std::isfinite(actual) && std::isfinite(expected) && CBPositive(ppm) &&
            std::isfinite(scale) && std::isfinite(error) && error <= 64.0 * std::numeric_limits<double>::epsilon() * scale &&
            error / ppm <= 5e-8;
    }
}

bool MotionCore::BindPathCoreHoldExcursion(const MotionExecutionIdentity& identity,
    const MotionOwnerLease& lease, double lengthMM, double lengthPulse,
    double distanceMM, double feedMMMin,
    double sourceFeedMMMin, double sourceVelocityPPS, std::uint32_t cycleLimit,
    const MotionPathCoreHoldExcursionView* crossView, bool requireReturnAuthorization) noexcept
{
    if (!identity.IsAssigned() || identity.source != MotionCommandSource::NC_MEMORY ||
        !lease.IsValid() || lease.owner != MotionOwner::AUTO ||
        !CBPositive(lengthMM) || !CBPositive(lengthPulse) || !CBPositive(distanceMM) ||
        !CBPositive(feedMMMin) || !CBPositive(sourceFeedMMMin) || sourceFeedMMMin > 100.0 ||
        feedMMMin > sourceFeedMMMin || !CBPositive(sourceVelocityPPS) ||
        !CBPositive(lengthPulse / lengthMM) || cycleLimit < 1U || cycleLimit > 32U ||
        (requireReturnAuthorization && cycleLimit != 1U)) return false;
    // FIX2: Use the admitted source speed as the sole PPS reference.
    // Re-converting F through rounded geometry can differ by one ULP even
    // when both NC feeds are exactly F6. Keep the semantic F bound exact.
    const double excursionVelocity = sourceVelocityPPS * (feedMMMin / sourceFeedMMMin);
    if (!CBPositive(excursionVelocity) || excursionVelocity > sourceVelocityPPS) return false;
    if (crossView != nullptr && (!CDViewValid(*crossView) ||
        crossView->original.lengthMM != lengthMM || crossView->original.lengthPulse != lengthPulse)) return false;
    auto& r = m_pathHoldProducerRequest;
    r.start = false; r.returnStart = false; r.requireReturnAuthorization = requireReturnAuthorization;
    r.settleSequence = 0ULL; r.expectedTransitionSequence = 0ULL;
    r.identity = identity; r.lease = lease;
    r.generation = m_pathHoldGeneration.load(std::memory_order_acquire);
    r.lengthMM = lengthMM; r.lengthPulse = lengthPulse; r.distanceMM = distanceMM; r.feedMMMin = feedMMMin;
    r.sourceFeedMMMin = sourceFeedMMMin; r.sourceVelocityPPS = sourceVelocityPPS;
    r.cycleLimit = cycleLimit;
    r.crossSegment = crossView != nullptr;
    if (crossView != nullptr) r.crossView = *crossView;
    return m_pathHoldRequests.ProducerTryPush(r);
}

bool MotionCore::RequestPathCoreHoldExcursion(const MotionExecutionIdentity& identity,
    const MotionOwnerLease& lease, MotionNCSettleRequestSequence settleSequence) noexcept
{
    const auto s = GetPathCoreHoldExcursionSnapshot();
    const auto generation = m_pathHoldGeneration.load(std::memory_order_acquire);
    if (settleSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        (!CCEligible(s) && !CLReturnEligible(s)) || !s.ready || s.requestGeneration != generation ||
        !CBIdentity(s.identity, identity) || !s.ownerLease.Matches(lease) ||
        !IsMotionOwnerLeaseCurrent(lease) || HasPendingSafetyOrRecoveryRequests() ||
        identity.epoch != GetCurrentExecutionEpoch() || HasPendingExecutionEpochChange() ||
        s.holdRequestSequence != settleSequence || settleSequence <= s.completedHoldRequestSequence ||
        s.transitionSequence == (std::numeric_limits<std::uint64_t>::max)()) return false;
    // One ticket per published transition, even if several NC calls occur
    // before RT dequeues the first request. Sequence wrap fails closed.
    if (m_pathHoldStartTicket.generation == generation &&
        ((CLReturnEligible(s) ? s.transitionSequence < m_pathHoldStartTicket.expectedTransitionSequence :
            s.transitionSequence <= m_pathHoldStartTicket.expectedTransitionSequence) ||
            settleSequence <= m_pathHoldStartTicket.settleSequence)) return false;
    auto& r = m_pathHoldProducerRequest;
    r.crossSegment = false; r.returnStart = CLReturnEligible(s);
    r.identity = identity; r.lease = lease; r.start = true; r.settleSequence = settleSequence;
    r.generation = generation; r.expectedTransitionSequence = s.transitionSequence;
    if (!m_pathHoldRequests.ProducerTryPush(r)) return false;
    m_pathHoldStartTicket = r;
    return true;
}

bool MotionCore::CommitPathCoreHoldExcursion(const MotionExecutionIdentity& identity,
    const MotionOwnerLease& lease, MotionNCSettleRequestSequence settleSequence) noexcept
{
    const auto snapshot = GetPathCoreHoldExcursionSnapshot();
    const auto generation = m_pathHoldGeneration.load(std::memory_order_acquire);
    const auto& ticket = m_pathHoldStartTicket; // same NC producer as Request
    const bool exactTransition = snapshot.transitionSequence == ticket.expectedTransitionSequence ||
        (snapshot.phase == MotionPathCoreHoldExcursionPhase::ARMED &&
            ticket.expectedTransitionSequence != (std::numeric_limits<std::uint64_t>::max)() &&
            snapshot.transitionSequence == ticket.expectedTransitionSequence + 1ULL);
    if (!identity.IsAssigned() || identity.epoch != GetCurrentExecutionEpoch() ||
        !CBIdentity(snapshot.identity, identity) || !snapshot.ownerLease.Matches(lease) ||
        !CBIdentity(ticket.identity, identity) || !ticket.lease.Matches(lease) ||
        ticket.generation != generation || snapshot.requestGeneration != generation ||
        ticket.settleSequence != settleSequence || !exactTransition ||
        snapshot.holdRequestSequence != settleSequence ||
        (ticket.returnStart ? !CLReturnEligible(snapshot) : !CCEligible(snapshot)) ||
        settleSequence <= snapshot.completedHoldRequestSequence ||
        !IsMotionOwnerLeaseCurrent(lease) || HasPendingSafetyOrRecoveryRequests() ||
        HasPendingExecutionEpochChange() ||
        settleSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID) return false;
    m_pathHoldCommittedRequest.store(settleSequence, std::memory_order_release);
    return true;
}

void MotionCore::CancelPathCoreHoldExcursion() noexcept
{
    m_pathHoldGeneration.fetch_add(1ULL, std::memory_order_acq_rel);
}

MotionPathCoreHoldExcursionSnapshot MotionCore::GetPathCoreHoldExcursionSnapshot() const noexcept
{
    for (unsigned attempt = 0U; attempt < 3U; ++attempt)
    {
        const auto generation = m_pathHoldPublication.load(std::memory_order_acquire);
        if (generation == 0ULL) return {};
        const auto& bank = m_pathHoldBanks[generation & 1ULL];
        const auto before = bank.sequence.load(std::memory_order_acquire);
        if ((before & 1ULL) != 0ULL) continue;
        std::array<std::uint64_t, PATH_HOLD_WORD_COUNT> words{};
        for (std::size_t i = 0U; i < words.size(); ++i) words[i] = bank.words[i].load(std::memory_order_acquire);
        std::atomic_thread_fence(std::memory_order_acquire);
        const auto after = bank.sequence.load(std::memory_order_acquire);
        if (before == after && (after & 1ULL) == 0ULL &&
            generation == m_pathHoldPublication.load(std::memory_order_acquire))
        {
            MotionPathCoreHoldExcursionSnapshot result{};
            std::memcpy(&result, words.data(), sizeof(result));
            if (result.publicationSequence == generation) return result;
        }
    }
    return {};
}

void MotionCore::PublishPathCoreHoldExcursionSnapshot() noexcept
{
    auto& bank = m_pathHoldBanks[(m_pathHoldNextPublication + 1ULL) & 1ULL];
    bank.sequence.fetch_add(1ULL, std::memory_order_acq_rel);
    m_pathHold.status.publicationSequence = m_pathHoldNextPublication + 1ULL;
    std::array<std::uint64_t, PATH_HOLD_WORD_COUNT> words{};
    std::memcpy(words.data(), &m_pathHold.status, sizeof(m_pathHold.status));
    for (std::size_t i = 0U; i < words.size(); ++i) bank.words[i].store(words[i], std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_release);
    bank.sequence.fetch_add(1ULL, std::memory_order_release);
    m_pathHoldPublication.store(++m_pathHoldNextPublication, std::memory_order_release);
}

void MotionCore::SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase phase, std::uint32_t reason) noexcept
{
    m_pathHold.status.phase = phase; m_pathHold.status.reason = reason;
    ++m_pathHold.status.transitionSequence; m_pathHold.status.ready = false; m_pathHold.status.boundaryOnly = false;
}

bool MotionCore::IsPathCoreHoldSourceCurrent() const noexcept
{
    const auto& c = m_Group.currentCmd;
    return m_Group.isActive && CBIdentity(c.execution, m_pathHold.status.identity) &&
        c.ownerLease.Matches(m_pathHold.status.ownerLease) &&
        GetCommandAuthorizationFailure(c) == MotionRejectReason::NONE &&
        !HasPendingSafetyOrRecoveryRequests() && !HasPendingExecutionEpochChange() &&
        m_Group.pathMode == PathMode::EXACT_STOP && !m_Group.enableHistory && !m_Group.enableTransform &&
        m_Group.jumpManager.state == JumpState::IDLE && !c.pathCoreRetainedTraversal &&
        !c.replayTerminalAlreadyPublished && c.sourceWCS == 54 && c.sourceToolLengthMode == 49 &&
        c.sourceToolRadiusMode == 40 && c.sourceIsAbsoluteMode && c.sourcePlaneMode == 17 &&
        !c.sourceG68Active && !c.sourceG168Active && !c.sourceG51Active &&
        c.sourceMirrorMask == 0U && !c.sourceG16Active && !c.sourceG162Active &&
        (c.mode == InterpolationMode::LINEAR || c.pathCorePlanarCircle) &&
        m_Group.axisCount > 0 && m_Group.axisCount <= 3 &&
        m_Group.cmdQueue.ingress_size() == 0U && m_Group.cmdQueue.replay_size() == 0U;
}

bool MotionCore::IsPathCoreHoldStrictlyStopped() const noexcept
{
    if (!IsPathCoreHoldSourceCurrent() || m_pContexts == nullptr) return false;
    const auto& v = m_Group.virtualAxis;
    if (!std::isfinite(v.currentCmdPos) || !std::isfinite(v.planningPos) ||
        v.currentCmdVel != 0.0 || v.logicalCmdVel != 0.0 || v.targetEndVel != 0.0 ||
        v.bufferSum != 0.0 || v.isFault || v.isLagAlarm ||
        !std::all_of(v.velBuffer.begin(), v.velBuffer.end(), [](double q) { return q == 0.0; })) return false;
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
    {
        const int index = m_Group.axisIndices[slot];
        if (index < 0 || index >= static_cast<int>(m_pContexts->size())) return false;
        const auto& a = (*m_pContexts)[index];
        if (index >= 3 || a.axisType != AxisType::LINEAR ||
            a.currentCmdVel != 0.0 || a.logicalCmdVel != 0.0 || a.isFault || a.isLagAlarm || !a.isServoOn ||
            !CBPositive(a.inPositionWindow_Pulse) || !std::isfinite(a.currentCmdPos) ||
            !std::isfinite(a.currentActPos) || std::abs(a.currentCmdPos - a.currentActPos) > a.inPositionWindow_Pulse) return false;
    }
    return true;
}

bool MotionCore::IsPathCoreHoldExcursionDriving() const noexcept
{
    const auto phase = m_pathHold.status.phase;
    return m_pathHold.prelaunchRejected || m_pathHold.unionActive || m_pathHold.crossGeometryActive || m_pathHold.movementOwned || m_pathHold.startPending || phase == MotionPathCoreHoldExcursionPhase::RETREATING ||
        phase == MotionPathCoreHoldExcursionPhase::RETURNING || phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN ||
        phase == MotionPathCoreHoldExcursionPhase::REJECTED;
}

bool MotionCore::IsPathCoreHoldEffectiveMappingValid() const noexcept
{
    if (!m_pathHold.unionActive || !m_pathHold.status.crossSegment) return false;
    const auto& c = m_Group.currentCmd;
    if (!CBIdentity(c.execution, m_pathHold.status.identity) ||
        !c.ownerLease.Matches(m_pathHold.status.ownerLease)) return false;
    if (m_Group.axisCount < m_pathHold.originalAxisCount || m_Group.axisCount > 3 ||
        c.axisCount != m_pathHold.originalAxisCount) return false;
    std::uint32_t mask = 0U;
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
    {
        const int axis = m_Group.axisIndices[slot];
        if (axis < 0 || axis > 2 || (mask & (1U << axis)) != 0U ||
            (slot < m_pathHold.originalAxisCount &&
                (axis != m_pathHold.originalAxisIndices[slot] || axis != c.axisIndices[slot]))) return false;
        mask |= 1U << axis;
    }
    return mask == m_pathHold.unionAxisMask;
}

bool MotionCore::ValidatePathCoreHoldCrossSource() const noexcept
{
    if (!m_pathHold.status.crossSegment) return true;
    const auto& view = m_pathHold.crossView;
    const auto& source = view.original;
    if (!CDViewValid(view) || m_pContexts == nullptr) return false;
    std::uint32_t mask = 0U;
    for (int slot = 0; slot < m_Group.currentCmd.axisCount; ++slot)
    {
        const int axis = m_Group.currentCmd.axisIndices[slot];
        if (axis < 0 || axis > 2 || axis >= static_cast<int>(m_pContexts->size())) return false;
        mask |= 1U << axis;
        if (!CDPulseNear(m_Group.startPos[slot], source.startPulse[axis], source, axis, view.pulsePerMM[axis]) ||
            !CDPulseNear(m_Group.currentCmd.targetPos[slot], source.endPulse[axis], source, axis, view.pulsePerMM[axis])) return false;
    }
    if (mask != source.axisMask || (source.kind == NCPathCoreRetainedKind::ARC) != m_Group.currentCmd.pathCorePlanarCircle)
        return false;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if ((view.validAxisMask & (1U << axis)) == 0U) continue;
        if (axis >= m_pContexts->size()) return false;
        const auto& a = (*m_pContexts)[axis];
        if (!a.isExist || !CBPositive(a.resolution_PPR) || !CBPositive(a.finalLead) ||
            a.resolution_PPR / a.finalLead != view.pulsePerMM[axis]) return false;
    }
    return true;
}

bool MotionCore::ExpandPathCoreHoldAxisUnion() noexcept
{
    if (!m_pathHold.status.crossSegment || m_pathHold.unionActive) return true;
    if (!ValidatePathCoreHoldCrossSource()) return false;
    std::uint32_t mask = m_pathHold.crossView.original.axisMask;
    for (std::uint32_t row = 0U; row < m_pathHold.crossView.completedCount; ++row)
        mask |= m_pathHold.crossView.completed[row].axisMask;
    if ((mask & ~7U) != 0U) return false;
    // Extra axes are admitted while still IDLE. Their first motion requires
    // the same NEW 200-cycle proof as every original source axis.
    for (int axis = 0; axis < 3; ++axis)
    {
        if ((mask & (1U << axis)) == 0U) continue;
        if (axis >= static_cast<int>(m_pContexts->size())) return false;
        const auto& a = (*m_pContexts)[axis];
        if (!a.isExist || a.axisType != AxisType::LINEAR || !a.isServoOn || a.isFault || a.isLagAlarm ||
            !CBPositive(a.maxVel_PPS) || !CBPositive(a.inPositionWindow_Pulse) ||
            !std::isfinite(a.currentCmdPos) || !std::isfinite(a.currentActPos) ||
            a.currentCmdVel != 0.0 || a.logicalCmdVel != 0.0 ||
            std::abs(a.currentCmdPos - a.currentActPos) > a.inPositionWindow_Pulse) return false;
        if ((m_pathHold.crossView.original.axisMask & (1U << axis)) == 0U &&
            (a.state != MotionState::MotionState_IDLE || a.targetVelocity != 0.0 || a.targetEndVel != 0.0 ||
                a.bufferSum != 0.0 || !std::all_of(a.velBuffer.begin(), a.velBuffer.end(), [](double q) { return q == 0.0; }) ||
                !CDPulseNear(a.currentCmdPos, m_pathHold.crossView.original.startPulse[axis],
                    m_pathHold.crossView.original, axis, m_pathHold.crossView.pulsePerMM[axis]))) return false;
    }
    m_pathHold.originalAxisCount = m_Group.axisCount;
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
        m_pathHold.originalAxisIndices[slot] = m_Group.axisIndices[slot];
    m_pathHold.unionAxisMask = mask;
    for (int axis = 0; axis < 3; ++axis)
    {
        if ((mask & (1U << axis)) == 0U) continue;
        auto& a = (*m_pContexts)[axis];
        m_pathHold.heldCommand[axis] = a.currentCmdPos;
        if ((m_pathHold.crossView.original.axisMask & (1U << axis)) != 0U) continue;
        m_Group.axisIndices[m_Group.axisCount++] = axis;
        a.logicalCmdPos = a.currentCmdPos; a.logicalCmdVel = 0.0;
        a.state = MotionState::MotionState_INTERPOLATING; a.inPosition = false;
    }
    m_pathHold.unionActive = true;
    return true;
}

void MotionCore::RestorePathCoreHoldAxisUnion() noexcept
{
    if (!m_pathHold.unionActive) return;
    for (int slot = m_pathHold.originalAxisCount; slot < m_Group.axisCount; ++slot)
    {
        auto& a = (*m_pContexts)[m_Group.axisIndices[slot]];
        a.state = MotionState::MotionState_IDLE; a.inPosition = true;
        a.planningPos = a.currentCmdPos; a.finalTargetPos = a.currentCmdPos;
        a.currentCmdVel = 0.0; a.logicalCmdVel = 0.0; a.targetVelocity = 0.0; a.targetEndVel = 0.0;
    }
    m_Group.axisCount = m_pathHold.originalAxisCount;
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
        m_Group.axisIndices[slot] = m_pathHold.originalAxisIndices[slot];
    m_pathHold.unionActive = false; m_pathHold.crossGeometryActive = false;
}

bool MotionCore::BeginPathCoreHoldSpan(std::uint32_t ordinal, double startS, double targetS) noexcept
{
    const auto& view = m_pathHold.crossView;
    if (ordinal > view.completedCount || m_pathHold.status.activeOrdinal > view.completedCount ||
        !IsPathCoreHoldStrictlyStopped()) return false;
    const auto& previous = m_pathHold.status.activeOrdinal == view.completedCount ?
        view.original : view.completed[m_pathHold.status.activeOrdinal];
    const bool original = ordinal == view.completedCount;
    const auto& row = original ? view.original : view.completed[ordinal];
    const double length = original ? m_pathHold.status.lengthPulse : row.lengthPulse;
    if (!CBPositive(length) || !std::isfinite(startS) || !std::isfinite(targetS) ||
        startS < 0.0 || startS > length || targetS < 0.0 || targetS > length || startS == targetS) return false;
    const double u = startS == length ? 1.0 : startS / length;
    double velocity = original ? m_pathHold.originalExcursionVelocity : (std::min)(
        m_pathHold.originalExcursionVelocity, (m_pathHold.status.feedMMMin / 60.0) * (row.lengthPulse / row.lengthMM));
    if (!CBPositive(velocity)) return false;
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
    {
        const int axis = m_Group.axisIndices[slot];
        const auto& a = (*m_pContexts)[axis];
        double expected = 0.0;
        if (!EvaluateNCPathCoreRetainedPulseCanonical(row, axis, u, expected)) return false;
        if (original && startS == 0.0)
        {
            if (slot < m_pathHold.originalAxisCount) expected = m_Group.startPos[slot];
            else expected = m_pathHold.heldCommand[axis];
        }
        // A seam carries rounding from both rows, including an arc axis
        // omitted by the original command. Keep both numeric and mm caps.
        if (!CDPulseNear(a.currentCmdPos, expected, row, axis, view.pulsePerMM[axis],
            ordinal == m_pathHold.status.activeOrdinal ? nullptr : &previous)) return false;
        const double projection = row.kind == NCPathCoreRetainedKind::ARC && axis < 2 ? velocity :
            std::abs((row.endPulse[axis] - row.startPulse[axis]) / length) * velocity;
        if (!std::isfinite(projection) || !CBPositive(a.maxVel_PPS) || projection > a.maxVel_PPS) return false;
    }
    auto& v = m_Group.virtualAxis;
    m_pathHold.status.activeOrdinal = ordinal;
    m_pathHold.crossGeometryActive = !original;
    m_pathHold.spanStartS = startS; m_pathHold.goal = targetS;
    m_pathHold.excursionVelocity = velocity;
    v.currentCmdPos = startS; v.planningPos = startS; v.finalTargetPos = targetS;
    v.maxVel_PPS = velocity;
    v.acc_PPS2 = (std::min)(m_pathHold.savedAcc, velocity / (std::max)(0.0001, m_Group.currentCmd.accTime));
    v.dec_PPS2 = (std::min)(m_pathHold.savedDec, velocity / (std::max)(0.0001,
        m_Group.currentCmd.decTime < 0.0 ? m_Group.currentCmd.accTime : m_Group.currentCmd.decTime));
    v.currentCmdVel = 0.0; v.logicalCmdVel = 0.0; v.targetEndVel = 0.0; v.targetVelocity = 0.0;
    v.inPosition = false; v.state = MotionState::MotionState_MOVING;
    m_pathHold.endpoint = false; m_pathHold.endpointSettled = false;
    m_pathHold.settleCycles = 0U; m_pathHold.lastSettleTick = 0ULL;
    return true;
}

bool MotionCore::MapPathCoreHoldExcursionGeometry(const AxisCommand& command) noexcept
{
    // The original execution identity remains live even when Safety is
    // retiring its geometry. Never fall back to the source map mid-stop.
    if (!m_pathHold.unionActive || !IsPathCoreHoldEffectiveMappingValid() || m_pContexts == nullptr ||
        !std::isfinite(command.instantCmdPos) || !std::isfinite(command.instantCmdVel)) return false;
    std::array<double, MAX_AXES> position{}, velocity{};
    const double s = command.instantCmdPos;
    if (m_pathHold.crossGeometryActive)
    {
        if (m_pathHold.status.activeOrdinal >= m_pathHold.crossView.completedCount) return false;
        const auto& row = m_pathHold.crossView.completed[m_pathHold.status.activeOrdinal];
        if (!CBPositive(row.lengthPulse) || s < 0.0 || s > row.lengthPulse) return false;
        const double u = s == row.lengthPulse ? 1.0 : s / row.lengthPulse;
        const double angle = row.startAngle + row.sweepRadians * u;
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
        {
            const int axis = m_Group.axisIndices[slot];
            if (!EvaluateNCPathCoreRetainedPulseCanonical(row, axis, u, position[axis])) return false;
            const double derivative = row.kind != NCPathCoreRetainedKind::ARC || axis > 1 ?
                row.endPulse[axis] - row.startPulse[axis] : axis == 0 ?
                -row.radiusPulse * std::sin(angle) * row.sweepRadians :
                row.radiusPulse * std::cos(angle) * row.sweepRadians;
            velocity[axis] = (derivative / row.lengthPulse) * command.instantCmdVel;
        }
    }
    else
    {
        if (s < 0.0 || s > m_pathHold.status.lengthPulse) return false;
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
            position[m_Group.axisIndices[slot]] = m_pathHold.heldCommand[m_Group.axisIndices[slot]];
        if (m_Group.mode == InterpolationMode::LINEAR)
        {
            for (int slot = 0; slot < m_pathHold.originalAxisCount; ++slot)
            {
                const int axis = m_Group.axisIndices[slot];
                // Keep the exact original operations, including their order.
                position[axis] = m_Group.startPos[slot] + command.instantCmdPos * m_Group.ratio[slot];
                velocity[axis] = command.instantCmdVel * m_Group.ratio[slot];
            }
        }
        else if (m_Group.currentCmd.pathCorePlanarCircle && m_pathHold.originalAxisCount == 2)
        {
            const double progress = s <= 0.0 ? 0.0 : s >= m_Group.totalDist3D ? 1.0 : s / m_Group.totalDist3D;
            const double startRadius = m_Group.currentCmd.startRadius;
            const double deltaRadius = m_Group.currentCmd.endRadius - startRadius;
            const double radius = startRadius + deltaRadius * progress;
            const double angle = m_Group.startAngle + m_Group.totalAngle * progress;
            const double cosine = std::cos(angle), sine = std::sin(angle);
            for (int slot = 0; slot < 2; ++slot)
            {
                const int axis = m_Group.axisIndices[slot];
                position[axis] = progress <= 0.0 ? m_Group.startPos[slot] : progress >= 1.0 ?
                    m_Group.currentCmd.targetPos[slot] : slot == 0 ?
                    m_Group.centerX + radius * cosine : m_Group.centerY + radius * sine;
                const double derivative = slot == 0 ? deltaRadius * cosine - radius * sine * m_Group.totalAngle :
                    deltaRadius * sine + radius * cosine * m_Group.totalAngle;
                const double progressVelocity = m_Group.totalDist3D > 1e-12 ? command.instantCmdVel / m_Group.totalDist3D : 0.0;
                velocity[axis] = derivative * progressVelocity;
            }
        }
        else return false;
    }
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
    {
        const int axis = m_Group.axisIndices[slot];
        const auto& a = (*m_pContexts)[axis];
        if (!a.isExist || !a.isServoOn || a.isFault || a.isLagAlarm ||
            (a.state != MotionState::MotionState_INTERPOLATING &&
                !(m_safetyControlledStopInProgress && a.state == MotionState::MotionState_STOPPING)) ||
            !std::isfinite(position[axis]) || !std::isfinite(velocity[axis]) ||
            !CBPositive(a.resolution_PPR) || !CBPositive(a.finalLead) ||
            a.resolution_PPR / a.finalLead != m_pathHold.crossView.pulsePerMM[axis] ||
            !CBPositive(a.maxVel_PPS) || std::abs(velocity[axis]) > a.maxVel_PPS) return false;
    }
    for (int slot = 0; slot < m_Group.axisCount; ++slot)
    {
        const int axis = m_Group.axisIndices[slot];
        (*m_pContexts)[axis].logicalCmdPos = position[axis];
        (*m_pContexts)[axis].logicalCmdVel = velocity[axis];
    }
    return true;
}

bool MotionCore::ClosePathCoreHoldEndpoint(AxisCommand& command) noexcept
{
    auto& v = m_Group.virtualAxis;
    const double error = std::abs(v.currentCmdPos - m_pathHold.goal);
    const double scale = (std::max)(std::abs(m_pathHold.goal), std::abs(m_pathHold.crossGeometryActive ?
        m_pathHold.spanStartS : m_pathHold.status.heldS));
    const double pulsePerMM = m_pathHold.crossGeometryActive ?
        m_pathHold.crossView.completed[m_pathHold.status.activeOrdinal].lengthPulse /
        m_pathHold.crossView.completed[m_pathHold.status.activeOrdinal].lengthMM :
        m_pathHold.status.lengthPulse / m_pathHold.lengthMM;
    if (v.state != MotionState::MotionState_IDLE || !v.inPosition ||
        !IsPathCoreHoldSourceCurrent() || v.currentCmdVel != 0.0 || v.logicalCmdVel != 0.0 ||
        v.targetEndVel != 0.0 || v.targetVelocity != 0.0 || v.bufferSum != 0.0 ||
        !std::all_of(v.velBuffer.begin(), v.velBuffer.end(), [](double q) { return q == 0.0; }) ||
        v.planningPos != v.currentCmdPos || v.finalTargetPos != v.currentCmdPos ||
        !std::isfinite(error) || // The 288-case real planner matrix measured 196.29 epsilon at a
        // 400-sample filtered near-start return. CB alone permits 256;
        // the independent physical cap and strict dry/normal gates remain.
        error > 256.0 * std::numeric_limits<double>::epsilon() * scale ||
        error / pulsePerMM > 5e-8) return false;
    v.currentCmdPos = m_pathHold.goal; v.planningPos = m_pathHold.goal; v.finalTargetPos = m_pathHold.goal;
    command.instantCmdPos = m_pathHold.goal; command.instantCmdVel = 0.0;
    return true;
}

bool MotionCore::ProcessPathCoreHoldExcursion(AxisCommand& command) noexcept
{
    if (m_pathHold.prelaunchRejected && !m_Group.isActive) m_pathHold.prelaunchRejected = false;
    const auto generation = m_pathHoldGeneration.load(std::memory_order_acquire);
    if (m_pathHold.generation != 0ULL && generation != m_pathHold.generation)
    {
        if ((m_pathHold.movementOwned || m_pathHold.unionActive) && m_Group.isActive &&
            CBIdentity(m_Group.currentCmd.execution, m_pathHold.status.identity) &&
            GetCommandAuthorizationFailure(m_Group.currentCmd) == MotionRejectReason::NONE)
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        m_pathHold.movementOwned = false;
        m_pathHold.generation = generation;
        m_pathHold.startPending = false;
        SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::INVALIDATED, 1U);
        if (m_pathHold.prelaunchRejected)
        {
            command.instantCmdPos = m_Group.virtualAxis.currentCmdPos; command.instantCmdVel = 0.0;
            return true;
        }
        return false;
    }
    auto& request = m_pathHoldConsumeRequest;
    if (m_pathHoldRequests.ConsumerTryPop(request) && request.generation == generation &&
        request.generation == m_pathHoldGeneration.load(std::memory_order_acquire))
    {
        // A duplicate receipt cannot reset this source's cumulative budget,
        // including duplicates queued before the first bind is published.
        // Ignore it without disturbing an in-flight excursion or checkpoint.
        const bool duplicateBind = !request.start && request.generation == m_pathHold.generation &&
            CBIdentity(request.identity, m_pathHold.status.identity);
        if (!request.start && !duplicateBind)
        {
            if (IsPathCoreHoldExcursionDriving())
            {
                TriggerGroupMappingIntegrityEmergencyStop(-1, true); return true;
            }
            const auto transitions = m_pathHold.status.transitionSequence;
            // Fixed RT-owned POD reset in place; avoid a second complete runtime
            // object on the 250 us stack. Restore nonzero defaults explicitly.
            static_assert(std::is_trivially_copyable<PathCoreHoldRuntime>::value,
                "CB in-place clear requires POD runtime state.");
            std::memset(static_cast<void*>(&m_pathHold), 0, sizeof(m_pathHold));
            m_pathHold.status.identity.sourceBlockId = MOTION_SOURCE_BLOCK_ID_INVALID;
            m_pathHold.status.transitionSequence = transitions;
            m_pathHold.generation = generation;
            m_pathHold.status.identity = request.identity; m_pathHold.status.ownerLease = request.lease;
            m_pathHold.status.lengthPulse = request.lengthPulse;
            m_pathHold.status.cycleLimit = request.cycleLimit;
            m_pathHold.status.requireReturnAuthorization = request.requireReturnAuthorization;
            m_pathHold.status.crossSegment = request.crossSegment;
            if (request.crossSegment) m_pathHold.crossView = request.crossView;
            m_pathHold.status.historyCount = request.crossSegment ? request.crossView.completedCount : 0U;
            m_pathHold.status.activeOrdinal = m_pathHold.status.historyCount;
            m_pathHold.status.retreatOrdinal = m_pathHold.status.historyCount;
            m_pathHold.status.requestGeneration = generation;
            m_pathHoldCommittedRequest.store(0ULL, std::memory_order_release);
            m_pathHold.status.distanceMM = request.distanceMM; m_pathHold.status.feedMMMin = request.feedMMMin;
            m_pathHold.lengthMM = request.lengthMM;
            m_pathHold.sourceVelocityPPS = request.sourceVelocityPPS;
            m_pathHold.excursionVelocity = request.sourceVelocityPPS * (request.feedMMMin / request.sourceFeedMMMin);
            m_pathHold.originalExcursionVelocity = m_pathHold.excursionVelocity;
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::ARMED);
        }
        else if (request.start && request.returnStart && CLReturnEligible(m_pathHold.status) &&
            request.generation == m_pathHold.generation &&
            request.expectedTransitionSequence == m_pathHold.status.transitionSequence &&
            request.settleSequence > m_pathHold.lastAcceptedStartSequence &&
            request.settleSequence > m_pathHold.status.completedHoldRequestSequence &&
            request.settleSequence == m_pathHold.status.holdRequestSequence &&
            request.settleSequence == m_activeFeedHoldNCSettleRequest.requestSequence &&
            CBIdentity(request.identity, m_pathHold.status.identity) &&
            request.lease.Matches(m_pathHold.status.ownerLease) && IsPathCoreHoldSourceCurrent())
        {
            m_pathHold.lastAcceptedStartSequence = request.settleSequence;
            m_pathHold.startPending = true;
            m_pathHold.endpointSettled = false; m_pathHold.settleCycles = 0U;
            m_pathHold.lastSettleTick = 0ULL;
            m_pathHold.actualMinimum.fill(0.0); m_pathHold.actualMaximum.fill(0.0);
        }
        else if (request.start && !request.returnStart && CCEligible(m_pathHold.status) &&
            !m_pathHold.startPending && !m_pathHold.movementOwned &&
            request.generation == m_pathHold.generation &&
            request.expectedTransitionSequence == m_pathHold.status.transitionSequence &&
            request.settleSequence > m_pathHold.lastAcceptedStartSequence &&
            request.settleSequence > m_pathHold.status.completedHoldRequestSequence &&
            request.settleSequence == m_pathHold.status.holdRequestSequence &&
            CBIdentity(request.identity, m_pathHold.status.identity) &&
            request.lease.Matches(m_pathHold.status.ownerLease) && IsPathCoreHoldSourceCurrent())
        {
            // Only this fresh request rearms the completed source. Never clear
            // cumulative counts, immutable geometry, receipt speed or identity.
            m_pathHold.lastAcceptedStartSequence = request.settleSequence;
            m_pathHold.startCaptured = false;
            m_pathHold.endpoint = false; m_pathHold.endpointSettled = false;
            m_pathHold.settleCycles = 0U; m_pathHold.lastSettleTick = 0ULL;
            m_pathHold.goal = 0.0;
            m_pathHold.status.heldS = 0.0; m_pathHold.status.retreatS = 0.0;
            m_pathHold.status.returnedS = 0.0;
            m_pathHold.heldCommand.fill(0.0);
            m_pathHold.actualMinimum.fill(0.0); m_pathHold.actualMaximum.fill(0.0);
            if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::COMPLETE)
                SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::ARMED);
            m_pathHold.startPending = true;
            m_pathHold.status.holdRequestSequence = request.settleSequence;
        }
    }
    if (m_pathHold.prelaunchRejected)
    {
        // NC publishes the parameter alarm before its asynchronous Safety
        // retirement. Cancellation revokes tickets, never this stopped fence.
        if (!CBIdentity(m_Group.currentCmd.execution, m_pathHold.status.identity) ||
            !m_Group.currentCmd.ownerLease.Matches(m_pathHold.status.ownerLease))
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        command.instantCmdPos = m_Group.virtualAxis.currentCmdPos; command.instantCmdVel = 0.0;
        return true;
    }
    if ((m_pathHold.movementOwned || m_pathHold.unionActive) &&
        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::INVALIDATED)
    {
        if (m_Group.isActive && CBIdentity(m_Group.currentCmd.execution, m_pathHold.status.identity) &&
            GetCommandAuthorizationFailure(m_Group.currentCmd) == MotionRejectReason::NONE)
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        return true;
    }
    if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::IDLE ||
        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::INVALIDATED) return false;
    if (!CBIdentity(m_Group.currentCmd.execution, m_pathHold.status.identity)) return false;
    if (!IsPathCoreHoldSourceCurrent())
    {
        if (IsPathCoreHoldExcursionDriving())
        {
            TriggerGroupMappingIntegrityEmergencyStop(-1, true);
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::INVALIDATED, 2U);
        }
        return false;
    }
    auto& v = m_Group.virtualAxis;
    if (!m_pathHold.sourceSeen)
    {
        const double error = std::abs(v.finalTargetPos - m_pathHold.status.lengthPulse);
        if (!CBPositive(v.finalTargetPos) || error > 128.0 * std::numeric_limits<double>::epsilon() * m_pathHold.status.lengthPulse ||
            error / (m_pathHold.status.lengthPulse / m_pathHold.lengthMM) > 5e-8)
        {
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 7U); return false;
        }
        if (!ValidatePathCoreHoldCrossSource())
        {
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 8U); return false;
        }
        m_pathHold.status.lengthPulse = v.finalTargetPos;
        // Geometry reconciliation changes only path length. The receipt speed
        // remains bound to the original immutable MotionCommand below.
        m_pathHold.sourceSeen = true;
    }
    // Recompute an empty filter's exact sum; repeated add/subtract can leave a
    // tiny nonzero residue despite every actual sample being zero. No position changes.
    if (CCEligible(m_pathHold.status) &&
        m_Group.feedrateOverride == 0.0 && v.currentCmdVel == 0.0 &&
        std::all_of(v.velBuffer.begin(), v.velBuffer.end(), [](double q) { return q == 0.0; }))
    {
        LifecycleCommitReservationGuard emptyFilter(*this, m_Group.currentCmd.execution);
        if (emptyFilter.IsAcquired()) v.bufferSum = 0.0;
    }
    if (m_pathHold.startPending && m_pathHold.status.phase != MotionPathCoreHoldExcursionPhase::WAIT_RETURN)
    {
        command.instantCmdPos = v.currentCmdPos; command.instantCmdVel = 0.0;
        if (m_pathHoldCommittedRequest.load(std::memory_order_acquire) != m_pathHold.status.holdRequestSequence ||
            !(m_Group.feedrateOverride > 0.0) || !m_ncSettleRuntimeObserved ||
            !m_ncSettleRuntimeCycleValid || !m_ncSettleRuntimeCycleContiguous) return true;
        // FIX1: Request admitted an exact published J.5 Hold proof, then
        // Commit authorized this one-shot release. J.5 correctly revokes its
        // mutable settled flag when NC releases override, possibly before this
        // RT pass consumes Request. Fence the scalar and acquire a NEW complete
        // 200-cycle stopped proof here; never reuse a revoked settled flag.
        if (!(v.currentCmdPos > 0.0 && v.currentCmdPos < m_pathHold.status.lengthPulse) ||
            !(v.finalTargetPos == m_pathHold.status.lengthPulse) ||
            !CBPositive(m_pathHold.sourceVelocityPPS) || !CBPositive(m_pathHold.excursionVelocity) ||
            m_pathHold.sourceVelocityPPS != std::abs(m_Group.currentCmd.targetVel) ||
            !(m_pathHold.excursionVelocity <= std::abs(m_Group.currentCmd.targetVel)))
        {
            m_pathHold.startPending = false;
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 3U);
            return true;
        }
        LifecycleCommitReservationGuard guard(*this, m_Group.currentCmd.execution);
        if (!guard.IsAcquired()) return true;
        if (!IsPathCoreHoldStrictlyStopped()) return true;
        if (!m_pathHold.startCaptured)
        {
            if (!ExpandPathCoreHoldAxisUnion())
            {
                m_pathHold.startPending = false; SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 8U); return true;
            }
            m_pathHold.status.heldS = v.currentCmdPos;
            for (int slot = 0; slot < m_Group.axisCount; ++slot)
                m_pathHold.heldCommand[m_Group.axisIndices[slot]] = (*m_pContexts)[m_Group.axisIndices[slot]].currentCmdPos;
            m_pathHold.settleCycles = 0U; m_pathHold.lastSettleTick = 0ULL;
            m_pathHold.endpointSettled = false; m_pathHold.startCaptured = true;
        }
        bool checkpointUnchanged = v.currentCmdPos == m_pathHold.status.heldS;
        for (int slot = 0; checkpointUnchanged && slot < m_Group.axisCount; ++slot)
            checkpointUnchanged = (*m_pContexts)[m_Group.axisIndices[slot]].currentCmdPos == m_pathHold.heldCommand[m_Group.axisIndices[slot]];
        if (!checkpointUnchanged)
        {
            m_pathHold.startPending = false;
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 3U);
            return true;
        }
        if (!m_pathHold.endpointSettled || !IsPathCoreHoldStrictlyStopped() ||
            m_pathHold.lastSettleTick + 1ULL != m_ncSettleRuntimeCycleTick) return true;
        // Recheck the latest completed physical sample before launch. Runtime
        // interpolation precedes this tick's actual-position decode, so the
        // adjacent proof covers completed samples through the previous tick.
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
        {
            const auto& a = (*m_pContexts)[m_Group.axisIndices[slot]];
            const double limit = (std::min)(0.5 * a.inPositionWindow_Pulse,
                (std::max)(4.0, 0.25 * a.inPositionWindow_Pulse));
            if ((std::max)(m_pathHold.actualMaximum[slot], a.currentActPos) -
                (std::min)(m_pathHold.actualMinimum[slot], a.currentActPos) > limit)
            {
                m_pathHold.settleCycles = 0U; m_pathHold.endpointSettled = false;
                return true;
            }
        }
        const double ratio = m_pathHold.status.lengthPulse / m_pathHold.lengthMM;
        const double availableMM = v.currentCmdPos / ratio;
        const double retreat = (std::min)(m_pathHold.status.distanceMM, availableMM) * ratio;
        double target = m_pathHold.status.distanceMM >= availableMM ? 0.0 : v.currentCmdPos - retreat;
        m_pathHold.status.retreatOrdinal = m_pathHold.status.historyCount;
        m_pathHold.status.retreatLocalS = target;
        if (m_pathHold.status.crossSegment && m_pathHold.status.distanceMM > availableMM)
        {
            double remaining = m_pathHold.status.distanceMM - availableMM;
            bool selected = false;
            for (std::uint32_t cursor = m_pathHold.crossView.completedCount; cursor != 0U; --cursor)
            {
                const auto& row = m_pathHold.crossView.completed[cursor - 1U];
                if (row.point) continue; // At most sixteen canonical rows.
                if (remaining <= row.lengthMM)
                {
                    const double local = remaining == row.lengthMM ? 0.0 :
                        row.lengthPulse - remaining * (row.lengthPulse / row.lengthMM);
                    if (!std::isfinite(local) || local < 0.0 || !(local < row.lengthPulse)) break;
                    m_pathHold.status.retreatOrdinal = cursor - 1U;
                    m_pathHold.status.retreatLocalS = local;
                    selected = true; break;
                }
                remaining -= row.lengthMM;
            }
            if (!selected)
            {
                // No excursion span has moved. Retire the temporary added
                // axes now, then hold the original command stationary until
                // actual Safety/group retirement even if NC cancels metadata.
                RestorePathCoreHoldAxisUnion();
                m_pathHold.prelaunchRejected = true;
                m_pathHold.startPending = false;
                SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 9U);
                return true;
            }
        }
        if (!std::isfinite(target) || !(target >= 0.0 && target < v.currentCmdPos))
        {
            m_pathHold.startPending = false;
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 4U);
            return true;
        }
        m_pathHold.status.heldS = v.currentCmdPos; m_pathHold.status.retreatS = target;
        m_pathHold.status.activeOrdinal = m_pathHold.status.historyCount;
        m_pathHold.excursionVelocity = m_pathHold.originalExcursionVelocity;
        m_pathHold.spanStartS = v.currentCmdPos;
        m_pathHold.savedPlanning = v.planningPos; m_pathHold.savedFinalTarget = v.finalTargetPos;
        m_pathHold.savedMaxVelocity = v.maxVel_PPS; m_pathHold.savedCruise = v.cruiseVel_PPS;
        m_pathHold.savedAcc = v.acc_PPS2; m_pathHold.savedDec = v.dec_PPS2;
        m_pathHold.savedTargetVelocity = v.targetVelocity; m_pathHold.savedTargetEndVelocity = v.targetEndVel;
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
            m_pathHold.heldCommand[m_Group.axisIndices[slot]] = (*m_pContexts)[m_Group.axisIndices[slot]].currentCmdPos;
        m_pathHold.goal = target; v.finalTargetPos = target; v.planningPos = v.currentCmdPos;
        v.maxVel_PPS = m_pathHold.excursionVelocity;
        v.acc_PPS2 = (std::min)(v.acc_PPS2, m_pathHold.excursionVelocity / (std::max)(0.0001, m_Group.currentCmd.accTime));
        v.dec_PPS2 = (std::min)(v.dec_PPS2, m_pathHold.excursionVelocity / (std::max)(0.0001,
            m_Group.currentCmd.decTime < 0.0 ? m_Group.currentCmd.accTime : m_Group.currentCmd.decTime));
        v.targetEndVel = 0.0; v.targetVelocity = 0.0; v.inPosition = false; v.state = MotionState::MotionState_MOVING;
        m_pathHold.startPending = false; m_pathHold.endpoint = false; m_pathHold.endpointSettled = false; m_pathHold.movementOwned = true;
        SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::RETREATING);
    }
    if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN)
    {
        command.instantCmdPos = v.currentCmdPos; command.instantCmdVel = 0.0;
        if (!m_pathHold.startPending || !m_pathHold.endpoint || !m_pathHold.endpointSettled ||
            m_pathHoldCommittedRequest.load(std::memory_order_acquire) != m_pathHold.status.holdRequestSequence ||
            m_pathHold.lastAcceptedStartSequence != m_pathHold.status.holdRequestSequence ||
            m_activeFeedHoldNCSettleRequest.requestSequence != m_pathHold.status.holdRequestSequence ||
            !(m_Group.feedrateOverride > 0.0) || !m_ncSettleRuntimeObserved ||
            !m_ncSettleRuntimeCycleValid || !m_ncSettleRuntimeCycleContiguous ||
            m_pathHold.lastSettleTick + 1ULL != m_ncSettleRuntimeCycleTick) return true;
        LifecycleCommitReservationGuard guard(*this, m_Group.currentCmd.execution);
        if (!guard.IsAcquired() || !IsPathCoreHoldStrictlyStopped() || !ValidatePathCoreHoldCrossSource() ||
            v.currentCmdPos != m_pathHold.goal || !(m_Group.feedrateOverride > 0.0) ||
            m_pathHold.generation != m_pathHoldGeneration.load(std::memory_order_acquire) ||
            m_pathHoldCommittedRequest.load(std::memory_order_acquire) != m_pathHold.status.holdRequestSequence ||
            m_activeFeedHoldNCSettleRequest.requestSequence != m_pathHold.status.holdRequestSequence) return true;
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
        {
            const auto& axis = (*m_pContexts)[m_Group.axisIndices[slot]];
            const double limit = (std::min)(0.5 * axis.inPositionWindow_Pulse,
                (std::max)(4.0, 0.25 * axis.inPositionWindow_Pulse));
            if ((std::max)(m_pathHold.actualMaximum[slot], axis.currentActPos) -
                (std::min)(m_pathHold.actualMinimum[slot], axis.currentActPos) > limit)
            {
                m_pathHold.settleCycles = 0U; m_pathHold.endpointSettled = false; return true;
            }
        }
        if (m_pathHold.status.crossSegment && m_pathHold.crossGeometryActive)
        {
            const double goal = m_pathHold.crossView.completed[m_pathHold.status.activeOrdinal].lengthPulse;
            if (!BeginPathCoreHoldSpan(m_pathHold.status.activeOrdinal, v.currentCmdPos, goal))
            {
                SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 10U); return true;
            }
        }
        else
        {
            m_pathHold.goal = m_pathHold.status.heldS; m_pathHold.spanStartS = v.currentCmdPos;
            v.finalTargetPos = m_pathHold.goal; v.planningPos = v.currentCmdPos;
            v.state = MotionState::MotionState_MOVING; v.inPosition = false;
            m_pathHold.endpoint = false; m_pathHold.endpointSettled = false; m_pathHold.settleCycles = 0U;
        }
        m_pathHold.startPending = false;
        SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::RETURNING);
    }
    if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::ARMED ||
        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::COMPLETE)
    {
        // NC can publish Request/Commit/override after this pass checked the
        // request ring. A previously published ready proof still owns a stopped
        // checkpoint: do not let the ordinary planner consume that release.
        if (m_pathHold.status.ready)
        {
            command.instantCmdPos = v.currentCmdPos; command.instantCmdVel = 0.0;
            return true;
        }
        return false;
    }
    if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::REJECTED)
    {
        v.cruiseVel_PPS = 0.0;
        Calc_Trajectory_Trapezoidal(v, command);
        return true;
    }
    if (m_pathHold.endpoint)
    {
        command.instantCmdPos = v.currentCmdPos; command.instantCmdVel = 0.0;
        if (!m_pathHold.endpointSettled || !(m_Group.feedrateOverride > 0.0) ||
            !m_ncSettleRuntimeObserved || !m_ncSettleRuntimeCycleValid || !m_ncSettleRuntimeCycleContiguous) return true;
        LifecycleCommitReservationGuard guard(*this, m_Group.currentCmd.execution);
        if (!guard.IsAcquired()) return true;
        if (m_pathHold.status.crossSegment)
        {
            if (!IsPathCoreHoldStrictlyStopped() || !ValidatePathCoreHoldCrossSource() ||
                m_pathHold.lastSettleTick + 1ULL != m_ncSettleRuntimeCycleTick) return true;
            for (int slot = 0; slot < m_Group.axisCount; ++slot)
            {
                const auto& axis = (*m_pContexts)[m_Group.axisIndices[slot]];
                const double limit = (std::min)(0.5 * axis.inPositionWindow_Pulse,
                    (std::max)(4.0, 0.25 * axis.inPositionWindow_Pulse));
                if ((std::max)(m_pathHold.actualMaximum[slot], axis.currentActPos) -
                    (std::min)(m_pathHold.actualMinimum[slot], axis.currentActPos) > limit)
                {
                    m_pathHold.settleCycles = 0U; m_pathHold.endpointSettled = false; return true;
                }
            }
            const bool retreating = m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::RETREATING;
            const bool cross = retreating ? m_pathHold.status.activeOrdinal > m_pathHold.status.retreatOrdinal :
            m_pathHold.status.activeOrdinal < m_pathHold.status.historyCount;
            if (cross)
            {
                std::uint32_t next = m_pathHold.status.activeOrdinal;
                if (retreating)
                {
                    do { --next; } while (next > m_pathHold.status.retreatOrdinal && m_pathHold.crossView.completed[next].point);
                }
                else
                {
                    do { ++next; } while (next < m_pathHold.status.historyCount && m_pathHold.crossView.completed[next].point);
                }
                const bool original = next == m_pathHold.status.historyCount;
                const double length = original ? m_pathHold.status.lengthPulse : m_pathHold.crossView.completed[next].lengthPulse;
                const double start = retreating ? length : 0.0;
                const double goal = retreating ? (next == m_pathHold.status.retreatOrdinal ? m_pathHold.status.retreatLocalS : 0.0) :
                    original ? m_pathHold.status.heldS : length;
                if (!BeginPathCoreHoldSpan(next, start, goal))
                {
                    SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 10U); return true;
                }
                ++m_pathHold.status.seamCount;
                // Keep the original lifecycle pending. The new span starts
                // only after the preceding row's fresh physical proof.
                command.instantCmdPos = v.currentCmdPos; command.instantCmdVel = 0.0;
                return true;
            }
        }
        if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::RETREATING)
        {
            ++m_pathHold.status.retreatCount;
            if (m_pathHold.status.requireReturnAuthorization)
            {
                const auto nextRequest = m_nextNCSettleRequestSequence.load(std::memory_order_acquire);
                const std::uint64_t fence = nextRequest == 0ULL ? (std::numeric_limits<std::uint64_t>::max)() : nextRequest - 1ULL;
                m_pathHold.lastAcceptedStartSequence = (std::max)(m_pathHold.lastAcceptedStartSequence, fence);
                m_pathHold.startPending = false;
                m_pathHold.endpointSettled = false; m_pathHold.settleCycles = 0U;
                m_pathHold.lastSettleTick = 0ULL;
                SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::WAIT_RETURN);
                return true;
            }
            if (m_pathHold.status.crossSegment && m_pathHold.crossGeometryActive)
            {
                const double goal = m_pathHold.crossView.completed[m_pathHold.status.activeOrdinal].lengthPulse;
                if (!BeginPathCoreHoldSpan(m_pathHold.status.activeOrdinal, v.currentCmdPos, goal))
                {
                    SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 10U); return true;
                }
            }
            else
            {
                m_pathHold.goal = m_pathHold.status.heldS;
                m_pathHold.spanStartS = v.currentCmdPos;
                v.finalTargetPos = m_pathHold.goal; v.planningPos = v.currentCmdPos;
                v.state = MotionState::MotionState_MOVING; v.inPosition = false;
                m_pathHold.endpoint = false; m_pathHold.endpointSettled = false; m_pathHold.settleCycles = 0U;
            }
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::RETURNING);
        }
        else
        {
            for (int slot = 0; slot < m_Group.axisCount; ++slot)
            {
                if ((*m_pContexts)[m_Group.axisIndices[slot]].currentCmdPos != m_pathHold.heldCommand[m_Group.axisIndices[slot]])
                {
                    SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 5U); return true;
                }
            }
            m_pathHold.status.returnedS = v.currentCmdPos;
            RestorePathCoreHoldAxisUnion();
            v.planningPos = m_pathHold.savedPlanning; v.finalTargetPos = m_pathHold.savedFinalTarget;
            v.maxVel_PPS = m_pathHold.savedMaxVelocity; v.cruiseVel_PPS = m_pathHold.savedCruise;
            v.acc_PPS2 = m_pathHold.savedAcc; v.dec_PPS2 = m_pathHold.savedDec;
            v.targetVelocity = m_pathHold.savedTargetVelocity; v.targetEndVel = m_pathHold.savedTargetEndVelocity;
            v.state = MotionState::MotionState_MOVING; v.inPosition = false;
            m_pathHold.movementOwned = false;
            ++m_pathHold.status.returnCount;
            // Fence every request allocated before this completion, including
            // a Hold during either leg that has not yet reached the J.5 tracker.
            // The allocator is global to J.5 profiles; fencing unrelated older
            // requests is conservative and avoids inspecting its SPSC storage.
            const auto nextRequest = m_nextNCSettleRequestSequence.load(std::memory_order_acquire);
            const auto& holdTracker = m_ncSettleTrackers[
                static_cast<std::size_t>(MotionNCSettleProfile::FEED_HOLD_GROUP)];
            const auto& holdProof = m_ncSettlePublishedSnapshots[
                static_cast<std::size_t>(MotionNCSettleProfile::FEED_HOLD_GROUP)];
            std::uint64_t fence = nextRequest == 0ULL ? (std::numeric_limits<std::uint64_t>::max)() : nextRequest - 1ULL;
            fence = (std::max)(fence, m_pathHold.status.completedHoldRequestSequence);
            fence = (std::max)(fence, m_pathHold.lastAcceptedStartSequence);
            fence = (std::max)(fence, m_activeFeedHoldNCSettleRequest.requestSequence);
            fence = (std::max)(fence, holdTracker.requestSequence);
            fence = (std::max)(fence, holdProof.requestSequence);
            m_pathHold.status.completedHoldRequestSequence = fence;
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::COMPLETE);
            return true;
        }
    }
    v.cruiseVel_PPS = m_pathHold.excursionVelocity * (std::max)(0.0, (std::min)(1.0, m_Group.feedrateOverride));
    Calc_Trajectory_Trapezoidal(v, command);
    if (v.state == MotionState::MotionState_IDLE)
    {
        LifecycleCommitReservationGuard guard(*this, m_Group.currentCmd.execution);
        if (!guard.IsAcquired()) return true;
        if (!ClosePathCoreHoldEndpoint(command))
        {
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::REJECTED, 6U); return true;
        }
        m_pathHold.endpoint = true; m_pathHold.endpointSettled = false; m_pathHold.settleCycles = 0U;
    }
    return true;
}

void MotionCore::UpdatePathCoreHoldExcursionEvidence() noexcept
{
    const auto generation = m_pathHoldGeneration.load(std::memory_order_acquire);
    if (m_pathHold.generation != 0ULL && (generation != m_pathHold.generation ||
        m_pathHold.status.identity.epoch != GetCurrentExecutionEpoch() ||
        !IsMotionOwnerLeaseCurrent(m_pathHold.status.ownerLease) || HasPendingSafetyOrRecoveryRequests()))
    {
        m_pathHold.startPending = false;
        if (m_pathHold.status.phase != MotionPathCoreHoldExcursionPhase::INVALIDATED)
            SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase::INVALIDATED, 1U);
    }
    if (!m_Group.isActive)
    {
        m_pathHold.movementOwned = false;
        m_pathHold.unionActive = false; m_pathHold.crossGeometryActive = false;
        m_pathHold.prelaunchRejected = false;
    }
    m_pathHold.status.activeS = m_Group.virtualAxis.currentCmdPos;
    m_pathHold.status.ready = false;
    m_pathHold.status.boundaryOnly = false;
    if (((CCEligible(m_pathHold.status) && !m_pathHold.startPending) ||
        CLReturnEligible(m_pathHold.status)) && IsPathCoreHoldStrictlyStopped())
    {
        const auto& proof = m_ncSettlePublishedSnapshots[static_cast<std::size_t>(MotionNCSettleProfile::FEED_HOLD_GROUP)];
        const auto& tracker = m_ncSettleTrackers[static_cast<std::size_t>(MotionNCSettleProfile::FEED_HOLD_GROUP)];
        const bool settled = proof.settled && proof.requestAccepted && proof.runtimeCycleValid &&
            proof.runtimeCycleContiguous && proof.overrideZero &&
            proof.requestSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
            proof.requestSequence == tracker.requestSequence &&
            proof.requestSequence == m_activeFeedHoldNCSettleRequest.requestSequence &&
            proof.requestSequence > m_pathHold.status.completedHoldRequestSequence &&
            proof.requestSequence > m_pathHold.lastAcceptedStartSequence &&
            CBIdentity(tracker.executionIdentity, m_pathHold.status.identity) &&
            tracker.ownerLease.Matches(m_pathHold.status.ownerLease);
        const bool returnWait = CLReturnEligible(m_pathHold.status);
        m_pathHold.status.ready = settled && (returnWait ?
            (m_pathHold.endpoint && m_Group.virtualAxis.currentCmdPos == m_pathHold.goal) :
            (m_Group.virtualAxis.currentCmdPos > 0.0 && m_Group.virtualAxis.currentCmdPos < m_pathHold.status.lengthPulse));
        m_pathHold.status.boundaryOnly = !returnWait && settled && (m_Group.virtualAxis.currentCmdPos == 0.0 ||
            m_Group.virtualAxis.currentCmdPos == m_pathHold.status.lengthPulse);
        if (!returnWait || (proof.requestAccepted && proof.requestSequence > m_pathHold.lastAcceptedStartSequence &&
            proof.requestSequence == m_activeFeedHoldNCSettleRequest.requestSequence))
            m_pathHold.status.holdRequestSequence = proof.requestSequence;
    }
    if (m_pathHold.startPending && m_pathHold.startCaptured &&
        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::ARMED)
    {
        bool candidate = m_pathHoldCommittedRequest.load(std::memory_order_acquire) ==
            m_pathHold.status.holdRequestSequence && m_Group.feedrateOverride > 0.0 &&
            m_ncSettleRuntimeObserved && m_ncSettleRuntimeCycleValid &&
            m_ncSettleRuntimeCycleContiguous && m_ncSettleMotionPassCompleted &&
            IsPathCoreHoldStrictlyStopped() && m_Group.virtualAxis.currentCmdPos == m_pathHold.status.heldS;
        const bool fresh = m_pathHold.lastSettleTick != m_ncSettleRuntimeCycleTick;
        if (fresh)
        {
            if (m_pathHold.settleCycles != 0U && m_ncSettleRuntimeCycleTick != m_pathHold.lastSettleTick + 1ULL)
                m_pathHold.settleCycles = 0U;
            for (int slot = 0; candidate && slot < m_Group.axisCount; ++slot)
            {
                const auto& a = (*m_pContexts)[m_Group.axisIndices[slot]];
                const double window = a.inPositionWindow_Pulse;
                candidate = a.currentCmdPos == m_pathHold.heldCommand[m_Group.axisIndices[slot]] && CBPositive(window) &&
                    std::isfinite(a.currentActPos) && std::abs(a.currentCmdPos - a.currentActPos) <= window;
                if (m_pathHold.settleCycles == 0U)
                    m_pathHold.actualMinimum[slot] = m_pathHold.actualMaximum[slot] = a.currentActPos;
                m_pathHold.actualMinimum[slot] = (std::min)(m_pathHold.actualMinimum[slot], a.currentActPos);
                m_pathHold.actualMaximum[slot] = (std::max)(m_pathHold.actualMaximum[slot], a.currentActPos);
                const double limit = (std::min)(0.5 * window, (std::max)(4.0, 0.25 * window));
                candidate = candidate && m_pathHold.actualMaximum[slot] - m_pathHold.actualMinimum[slot] <= limit;
            }
            m_pathHold.settleCycles = candidate ? (std::min)(MOTION_NC_SETTLE_REQUIRED_CYCLES, m_pathHold.settleCycles + 1U) : 0U;
            m_pathHold.endpointSettled = m_pathHold.settleCycles >= MOTION_NC_SETTLE_REQUIRED_CYCLES;
            m_pathHold.lastSettleTick = m_ncSettleRuntimeCycleTick;
        }
        if (!candidate)
        {
            m_pathHold.settleCycles = 0U;
            m_pathHold.endpointSettled = false;
        }
    }
    if (m_pathHold.endpoint && (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::RETREATING ||
        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::RETURNING ||
        m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN))
    {
        bool candidate = m_ncSettleRuntimeObserved && m_ncSettleRuntimeCycleValid &&
            m_ncSettleRuntimeCycleContiguous && m_ncSettleMotionPassCompleted &&
            IsPathCoreHoldStrictlyStopped() && m_Group.virtualAxis.currentCmdPos == m_pathHold.goal;
        if (m_pathHold.status.phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN)
            candidate = candidate && m_pathHold.startPending && m_Group.feedrateOverride > 0.0 &&
            m_pathHoldCommittedRequest.load(std::memory_order_acquire) == m_pathHold.status.holdRequestSequence &&
            m_pathHold.lastAcceptedStartSequence == m_pathHold.status.holdRequestSequence &&
            m_activeFeedHoldNCSettleRequest.requestSequence == m_pathHold.status.holdRequestSequence;
        const bool fresh = m_pathHold.lastSettleTick != m_ncSettleRuntimeCycleTick;
        if (fresh)
        {
            if (m_pathHold.settleCycles != 0U && m_ncSettleRuntimeCycleTick != m_pathHold.lastSettleTick + 1ULL)
                m_pathHold.settleCycles = 0U;
            for (int slot = 0; candidate && slot < m_Group.axisCount; ++slot)
            {
                const auto& a = (*m_pContexts)[m_Group.axisIndices[slot]];
                const double window = a.inPositionWindow_Pulse;
                candidate = CBPositive(window) && std::isfinite(a.currentCmdPos) && std::isfinite(a.currentActPos) &&
                    std::abs(a.currentCmdPos - a.currentActPos) <= window;
                if (m_pathHold.settleCycles == 0U)
                    m_pathHold.actualMinimum[slot] = m_pathHold.actualMaximum[slot] = a.currentActPos;
                m_pathHold.actualMinimum[slot] = (std::min)(m_pathHold.actualMinimum[slot], a.currentActPos);
                m_pathHold.actualMaximum[slot] = (std::max)(m_pathHold.actualMaximum[slot], a.currentActPos);
                const double limit = (std::min)(0.5 * window, (std::max)(4.0, 0.25 * window));
                candidate = candidate && m_pathHold.actualMaximum[slot] - m_pathHold.actualMinimum[slot] <= limit;
            }
            m_pathHold.settleCycles = candidate ? (std::min)(MOTION_NC_SETTLE_REQUIRED_CYCLES, m_pathHold.settleCycles + 1U) : 0U;
            m_pathHold.endpointSettled = m_pathHold.settleCycles >= MOTION_NC_SETTLE_REQUIRED_CYCLES;
            m_pathHold.lastSettleTick = m_ncSettleRuntimeCycleTick;
        }
        if (!candidate) m_pathHold.endpointSettled = false;
    }
    PublishPathCoreHoldExcursionSnapshot();
}
