// EDM267 FIX1: independent NC compilation unit; original function bodies preserved.
#include "NCManager_GapInternal.h"

using namespace NCGapDetail;

bool NCManager::IsEDM130RecipePreambleSameThread(const NCParsedBlock& block, int pc) const noexcept
{
    if (pc < 0 || m_edm94.latched || !EDM77::Detail::Clean(block)) return false;
    bool mutation = false;
    if (!EDM77::Detail::Operation(block, mutation))
    {
        // Keep the older sequence fixture's seven-field allowlist intact.
        // Reuse its exact literal grammar only for this opt-in's five fields.
        if (block.gCount != 1 || !EDM77::Detail::EqualLiteral(block.gExpressions[0], 39U) ||
            (!EDM77::Detail::EqualLiteral(block.expression('P'), 11U) &&
                !EDM77::Detail::EqualLiteral(block.expression('P'), 12U) &&
                !EDM77::Detail::EqualLiteral(block.expression('P'), 13U) &&
                !EDM77::Detail::EqualLiteral(block.expression('P'), 19U) &&
                !EDM77::Detail::EqualLiteral(block.expression('P'), 20U))) return false;
        NCParsedBlock syntax = block;
        syntax.paramExpressions['P' - 'A'] = "5";
        if (!EDM77::Detail::Operation(syntax, mutation)) return false;
    }
    const auto& cache = GetBaseProgramCache();
    if (!cache.GetGeneration() || cache.Size() > 128U) return false;
    unsigned arms = 0U, selectedArm = 0U;
    int sequenceFirst=-1;std::array<unsigned,2U> sequenceArms{};std::array<unsigned,4U> chainKinds{};
    const bool historySequence=E237ProgramShape(cache,sequenceFirst,sequenceArms);
    if(historySequence)sequenceArms[0U]=E237ArmForKind(sequenceArms[0U]);
    const bool chain=!historySequence&&E236ProgramShape(cache,sequenceFirst,chainKinds);
    if(chain)sequenceArms[0U]=E236ArmForKind(chainKinds[0U]);
    const bool sequence=historySequence||chain||E234ProgramShape(cache,sequenceFirst,sequenceArms)||E233ProgramShape(cache,sequenceFirst,sequenceArms,
        &NCManager::IsEDM173BracketShapeValid,&NCManager::IsEDM173OrbitBlockShapeValid);
    if(sequence&&pc<sequenceFirst-1){arms=1U;selectedArm=sequenceArms[0U];}
    for (std::size_t i = 0U; !sequence && i < cache.Size(); ++i)
    {
        const auto* line = cache.TryGetLine(static_cast<int>(i));
        EDM97::Tuning tuning{}; EDM107::FlushProfile profile{};
        if (!line) continue;
        const unsigned candidates[] = {174U, 176U, 178U, 180U, 182U, 184U, 186U, 188U, 190U, 192U, 194U, 196U, 198U, 200U, 202U, 204U, 206U, 208U, 210U, 212U, 214U, 216U, 218U, 220U, 222U, 224U, 226U, 228U, 230U, 232U, 234U, 236U, 238U, 240U, 242U, 244U, 246U, 248U, 250U, 252U, 254U, 256U, 258U, 260U, 262U, 264U, 266U, 268U, 270U, 272U, 274U, 276U, 278U, 280U, 282U, 284U, 286U, 288U, 290U, 292U, 294U, 296U, 298U, 300U};
        for (unsigned arm : candidates)
        {
            if (!E108ProfileOnlyNPQRSIJK(line->parsedBlock, arm, tuning, profile)) continue;
            if (static_cast<int>(i) <= pc || ++arms != 1U) return false;
            selectedArm = arm;
        }
    }
    if (arms != 1U) return false;
    // New recipe fields belong only to their explicit future opt-in profile.
    // Semantic identity and units are checked again by the admission binder.
    if (block.gCount == 1 && EDM77::Detail::EqualLiteral(block.gExpressions[0], 39U))
    {
        const bool finalSpeed = EDM77::Detail::EqualLiteral(block.expression('P'), 11U);
        const bool polarity = EDM77::Detail::EqualLiteral(block.expression('P'), 12U);
        const bool approach = EDM77::Detail::EqualLiteral(block.expression('P'), 13U);
        const bool initialSpeed = EDM77::Detail::EqualLiteral(block.expression('P'), 19U);
        const bool initialDistance = EDM77::Detail::EqualLiteral(block.expression('P'), 20U);
        const bool staged = finalSpeed || polarity || approach || initialSpeed || initialDistance;
        const bool reference = EDM77::Detail::EqualLiteral(block.expression('P'), 9U);
        const bool machining = EDM77::Detail::EqualLiteral(block.expression('P'), 8U);
        const bool speed = EDM77::Detail::EqualLiteral(block.expression('P'), 7U);
        const bool height = EDM77::Detail::EqualLiteral(block.expression('P'), 6U);
        const bool interval = EDM77::Detail::EqualLiteral(block.expression('P'), 21U);
        const bool multiplier = EDM77::Detail::EqualLiteral(block.expression('P'), 22U);
        if ((staged && !E137FlushStageArm(selectedArm)) || (!staged && !reference && !machining && !speed && !height && !interval && !multiplier && !EDM77::Detail::EqualLiteral(block.expression('P'), 5U)) ||
            (reference && !E135ReferenceArm(selectedArm)) || (machining && !E134MachiningArm(selectedArm)) || (speed && !E131SpeedArm(selectedArm)) || (height && !E132HeightArm(selectedArm)) ||
            ((interval || multiplier) && !E133DeepArm(selectedArm))) return false;
        const auto* catalog = m_recipe.GetCatalog();
        if (!catalog || catalog->schemaVersion != 2U) return false;
        const char* semantic = finalSpeed ? "FinalApproachSpeed" : polarity ? "ElectrodePolarity" : approach ? "DecelerationDistance" :
            initialSpeed ? "LargeAreaMachiningSpeed" : initialDistance ? "LargeAreaFlushDistance" : reference ? "GapVoltageSetpoint" : machining ? "MachiningServoOverride" : interval ? "DeepFlushCycleInterval" : multiplier ? "DeepFlushHeightMultiplier" :
            height ? "JumpHeight" : speed ? "JumpSpeedOverride" : "DischargeWorkTime";
        const char* unit = (finalSpeed || initialSpeed) ? "%" : polarity ? "code" : (approach || initialDistance) ? "mm" : reference ? "V" : machining ? "%" : interval ? "cycle" : multiplier ? "x" : height ? "mm" : speed ? "%" : "s";
        unsigned matches = 0U;
        for (const auto& field : catalog->fields)
        {
            if (!EDM58::BindingDetail::LabelEquals(field.semanticKey, sizeof(field.semanticKey), semantic)) continue;
            if (++matches != 1U || field.id != (finalSpeed ? 11U : polarity ? 12U : approach ? 13U : initialSpeed ? 19U : initialDistance ? 20U : reference ? 9U : machining ? 8U : interval ? 21U : multiplier ? 22U : height ? 6U : speed ? 7U : 5U) || !field.enabled) return false;
            const bool confirmed = field.unitConfirmed && EDM58::BindingDetail::LabelEquals(field.unit, sizeof(field.unit), unit);
            const bool explicitLegacy = staged && EDM77::Detail::EqualLiteral(block.expression('Q'), 3U) &&
                EDM137Units::LegacyExplicitDefinition(field, semantic, unit);
            if (!confirmed && !explicitLegacy) return false;
        }
        if (matches != 1U) return false;
    }
    return true;
}

// EDM141: frozen expressions and actual resolved numbers share the same local
// commit receipts and MEMORY scope. No result here grants an RT permit.
bool NCManager::MatchesEDM141MotionSameThread(const NCParsedBlock& parsed,
    const NCBlock& resolved, int pc) noexcept
{
    const auto& s = m_edm94; const auto& p = m_edm140;
    if (!E141MotionArm(s.gapHistoryArm) || !p.sealed ||
        m_edm141.plannedRows != EDM141::RowCapacity || !m_edm141.expressionRows ||
        !s.latched || !s.ready || !s.everArmed || s.armPending || s.finishPending || s.complete ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !s.gapHistoryMode || !s.gapWorkTime || !s.gapPhysicalAd ||
        p.session != s.session || p.run != s.run || p.cache != s.cache || p.armEpoch != s.armEpoch ||
        p.ownerGeneration != s.lease.generation || m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        m_isSingleBlockEnabled || Close_System_Com_flag || !m_externalReadyInterlock ||
        !m_macroStack.empty() || m_isG66Active || Homing.IsActive() || AlarmManager::GetInstance().HasAlarm() ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_programMotionLease.Matches(s.lease) || !m_motion.IsMotionOwnerLeaseCurrent(s.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_edm130.bound ||
        !EDM130::ValidRecipe(m_edm130.recipe) ||
        m_edm130.recipe.diagnosticControlSource != E149ControlSource(s.gapHistoryArm) ||
        pc < 0 || GetBasePCValue() != pc || p.next > p.count || p.count > EDM140::AssignmentCapacity) return false;
    unsigned row = 0U;
    while (row < m_edm141.plannedRows && m_edm141.rows[row].pc != pc) ++row;
    if (row >= m_edm141.plannedRows || s.pc[row] != pc ||
        (s.next != row && !(s.next == row + 1U && m_edm116.pending && m_edm116.selected == s.next))) return false;
    const auto& frozen = m_edm141.rows[row];
    if (frozen.assignmentCount != p.next || frozen.definedMask != p.definedMask ||
        (p.next && (!p.lastCommit || m_programCommittedPC < p.pc[p.next - 1U])) ||
        !EDM141::MatchesResolved(parsed, resolved, frozen)) return false;
    const auto* cached = GetBaseProgramCache().TryGetLine(pc);
    if (!cached || !EDM141::Matches(cached->parsedBlock, frozen)) return false;
    for (unsigned local = 0U; local < 2U; ++local)
        if ((p.definedMask & (1U << local)) && (p.values[local] != frozen.localValues[local] ||
            MacroSys.GetVar('#', static_cast<int>(local + 1U)) != p.values[local])) return false;
    return true;
}

bool NCManager::IsEDM141MotionCandidateSameThread(const NCParsedBlock& parsed, int pc) noexcept
{
    const auto& s = m_edm94;
    if (!E141MotionArm(s.gapHistoryArm) || s.next >= m_edm141.plannedRows ||
        s.next >= EDM141::RowCapacity || m_edm141.verifiedRows != s.next || s.pc[s.next] != pc) return false;
    const auto& frozen = m_edm141.rows[s.next];
    // Build a numeric comparison image only. The existing runtime Resolver
    // must independently produce the same values after its drain barrier.
    NCBlock expected{}; expected.isEmpty = false; expected.hasG = true;
    expected.gCode = frozen.gcode; expected.gCount = 1; expected.gCodes[0] = frozen.gcode;
    for (unsigned word = 0U; word < EDM141::WordCount; ++word)
        if (frozen.presentMask & (1U << word))
        {
            const unsigned address = EDM141::Words[word] - 'A';
            expected.hasParam[address] = true; expected.param[address] = frozen.values[word];
        }
    return MatchesEDM141MotionSameThread(parsed, expected, pc);
}

// EDM140: a stopped scalar bookkeeping row cannot advance or replace an EDM path.
bool NCManager::IsEDM140AssignmentCandidateSameThread(const NCParsedBlock& block, int pc) const noexcept
{
    const auto& s = m_edm94; const auto& p = m_edm140;
    if (!E140AssignmentArm(s.gapHistoryArm) || !p.sealed || !p.count ||
        p.count > EDM140::AssignmentCapacity || p.next >= p.count || pc < 0 ||
        p.pc[p.next] != pc || GetBasePCValue() != pc || !EDM140::Matches(block, p.specifications[p.next]) ||
        p.session != s.session || p.run != s.run || p.cache != s.cache ||
        p.armEpoch != s.armEpoch || p.ownerGeneration != s.lease.generation ||
        !s.latched || !s.ready || !s.everArmed || s.armPending || s.finishPending || s.complete ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !s.gapHistoryMode || !s.gapWorkTime || !s.gapPhysicalAd ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY || m_isSingleBlockEnabled ||
        Close_System_Com_flag || !m_externalReadyInterlock || !m_macroStack.empty() ||
        m_isG66Active || Homing.IsActive() || AlarmManager::GetInstance().HasAlarm() ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_programMotionLease.Matches(s.lease) || !m_motion.IsMotionOwnerLeaseCurrent(s.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !CoordSys.IsTranslationRunFrozen() ||
        !CoordSys.IsTranslationRunCurrent() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsXYZCUVFeedProfileCurrentSameThread() || !m_edm116.initialTranslationValid ||
        !SameNCTranslationSnapshot(m_edm116.initialTranslation, CoordSys.GetTranslationSnapshot()) ||
        !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()) ||
        !m_edm130.bound || !EDM130::ValidRecipe(m_edm130.recipe) ||
        m_edm130.recipe.diagnosticControlSource != E149ControlSource(s.gapHistoryArm))
        return false;
    const auto* cached = GetBaseProgramCache().TryGetLine(pc);
    return cached && EDM140::Matches(cached->parsedBlock, p.specifications[p.next]);
}

bool NCManager::AuthorizeEDM140AssignmentSameThread(const NCParsedBlock& block, int pc) noexcept
{
    if (!IsEDM140AssignmentCandidateSameThread(block, pc)) return false;
    const auto& s = m_edm94; const auto& p = m_edm140; const auto& rows = m_edm116;
    if (!s.next || s.next >= s.moves || s.next != p.afterRows[p.next] || rows.pending ||
        m_pathFeed.pending || m_pathArc.pending ||
        rows.selected != s.next || rows.consumed != s.next || rows.acknowledgedRows != s.next ||
        (p.specifications[p.next].readMask & ~p.definedMask) != 0U ||
        m_motion.GetQueueSize() || !m_motion.IsGroupNCDrained() ||
        !IsEDM114CurrentSameThread(s.next) || !IsEDM130CurrentSameThread(s.next)) return false;
    for (unsigned local = 0U; local < 2U; ++local)
        if ((p.definedMask & (1U << local)) && MacroSys.GetVar('#', static_cast<int>(local + 1U)) != p.values[local])
            return false;
    return true;
}

bool NCManager::ValidateEDM140AssignmentSameThread(const NCParsedBlock& block, int pc,
    const NCMacroAssignmentCommit& assignment) const noexcept
{
    return IsEDM140AssignmentCandidateSameThread(block, pc) &&
        EDM140::ValidResult(m_edm140.specifications[m_edm140.next],
            assignment.prefix, assignment.index, assignment.value) &&
        (!E141MotionArm(m_edm94.gapHistoryArm) ||
            assignment.value == m_edm141.assignmentValues[m_edm140.next]);
}

bool NCManager::CompleteEDM140AssignmentSameThread(const NCParsedBlock& block, int pc,
    const NCMacroAssignmentCommit& assignment, const NCProgramCommitSnapshot& commit) noexcept
{
    auto& p = m_edm140; const auto& s = m_edm94;
    // CommitProgramBlock records the exact row; dispatch PC advances later
    // through the existing shared path. Bind its actual committed snapshot.
    if (!E140AssignmentArm(s.gapHistoryArm) || !p.sealed || !p.count ||
        p.count > EDM140::AssignmentCapacity || p.next >= p.count || p.pc[p.next] != pc ||
        GetBasePCValue() != pc || m_programCommittedPC != pc ||
        !EDM140::Matches(block, p.specifications[p.next]) ||
        !EDM140::ValidResult(p.specifications[p.next], assignment.prefix, assignment.index, assignment.value) ||
        (E141MotionArm(s.gapHistoryArm) && assignment.value != m_edm141.assignmentValues[p.next]) ||
        !commit.IsValid() || commit.scope != NCProgramScope::MEMORY || commit.cacheGeneration != p.cache ||
        commit.sourcePC != pc || commit.sequence <= p.lastCommit ||
        m_lastProgramCommit.scope != commit.scope || m_lastProgramCommit.cacheGeneration != commit.cacheGeneration ||
        m_lastProgramCommit.frameId != commit.frameId || m_lastProgramCommit.sourcePC != commit.sourcePC ||
        m_lastProgramCommit.sequence != commit.sequence || p.session != s.session ||
        p.run != s.run || p.cache != s.cache || p.armEpoch != s.armEpoch ||
        p.ownerGeneration != s.lease.generation || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        s.cache != GetBaseProgramCache().GetGeneration() || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || s.cancelled || s.resetSeen || s.resetCommitted ||
        s.releaseRequested || s.rtFaultReported || !m_motion.IsMotionOwnerLeaseCurrent(s.lease) ||
        MacroSys.GetVar(assignment.prefix, assignment.index) != assignment.value) return false;
    p.values[static_cast<unsigned>(assignment.index - 1)] = assignment.value;
    p.definedMask |= 1U << static_cast<unsigned>(assignment.index - 1);
    p.lastCommit = commit.sequence; ++p.next;
    char record[481]{};
    const int length = std::snprintf(record, sizeof(record),
        "[EDM140] event=LOCAL_COMMITTED session=%llu index=%u/%u pc=%d afterRows=%u target=%d value=%.9g commit=%llu B=%u retainedRows=%u powerControl=NC_PLC_S92\n",
        static_cast<unsigned long long>(s.session), p.next, p.count, pc, s.next,
        assignment.index, assignment.value, static_cast<unsigned long long>(commit.sequence),
        static_cast<unsigned>(m_edm127.current.mode), m_edm116.acknowledgedRows);
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM140] event=LOG_FORMAT_ERROR part=LOCAL_COMMITTED\n");
    return true;
}

bool NCManager::PreflightEDM158VisibleTravelSameThread() noexcept
{
    const auto& s = m_edm94; const auto& r = m_edm130.recipe;
    if (!EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) || !m_edm130.bound ||
        !E158VisiblePlan(s) || !E156BoundPlanCurrent(s, r) || !EDM130::IsVisibleFlush(r) ||
        !EDM130::ValidRecipe(r) || !m_edm127.authoredInline[0] ||
        m_edm127.expectedMode[0] != EDM116::Mode::B1 ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    // The detached B1 leg uses the existing coordinator and CNC mapper. Its
    // dedicated reference speed leaves post-PI correction headroom. The
    // hardware CSV cap stays 0.1 mm/s; the final stage still obeys E11/E13.
    const double approach = EDM130::FrozenApproachDistanceMm(r);
    const double reference = EDM130::FrozenGapReferenceVoltageV(r);
    const auto forward = EDMGapServo::Evaluate(EDM97::MakeProfile(s.gapTuning, r), reference + 38.0,
        reference, EDM130::FrozenMachiningOverridePercent(r));
    const double machiningSpeed = (std::min)(0.12, forward.limitedSpeedMmPerMin) / 60.0;
    if (!std::isfinite(approach) || approach < 0.0 || !forward.valid ||
        !std::isfinite(machiningSpeed) || machiningSpeed <= 0.0) return false;
    double normalSpeed = 0.0, acceleration = 0.0, maximumStopTime = 0.2;
    double csvCap = 0.0, followingWindow = 0.0, worstPiPps = 0.0, sparePps = 0.0;
    std::array<double, 6U> origin{}, end{}, top{};
    for (unsigned axisIndex = 0U; axisIndex < 6U; ++axisIndex)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(axisIndex));
        const double scale = axis.resolution_PPR / axis.finalLead;
        if (!axis.isExist || axis.axisIndex != static_cast<int>(axisIndex) ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
            !std::isfinite(scale) || scale <= 0.0 || !std::isfinite(axis.currentCmdPos)) return false;
        if (!std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.0 ||
            !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
            !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0) return false;
        maximumStopTime = (std::max)(maximumStopTime, axis.Stop_dec_time < 0.001 ? 0.2 : axis.Stop_dec_time);
        maximumStopTime = (std::max)(maximumStopTime, axis.G00_acc_time < 0.001 ? 0.2 : axis.G00_acc_time);
        maximumStopTime = (std::max)(maximumStopTime, axis.G00_dec_time < 0.001 ? 0.2 : axis.G00_dec_time);
        if (axisIndex == 2U)
        {
            if (!std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0) return false;
            // Match the frozen serialized integer CSV cap and BindEDM114Mapping
            // per-axis acceleration budget; never override a configured limit.
            const double cap = std::floor((std::min)(axis.maxVel_PPS, scale * 0.1));
            const double accTime = axis.G00_acc_time < 0.001 ? 0.2 : axis.G00_acc_time;
            const double decTime = axis.G00_dec_time < 0.001 ? 0.2 : axis.G00_dec_time;
            if (!std::isfinite(axis.inPositionWindow_Pulse) || axis.inPositionWindow_Pulse <= 0.0 ||
                !std::isfinite(axis.pid.MaxLag) || axis.pid.MaxLag <= 0.0) return false;
            followingWindow = (std::min)(axis.inPositionWindow_Pulse,
                (std::min)(axis.pid.MaxLag, scale * 0.001));
            if (!EDM94::VisibleServoHeadroomFits(scale, cap, followingWindow,
                axis.Pid_G00.Kp, axis.Pid_G00.Ki, axis.Pid_G00.Kvff, axis.pid.MaxIntegral)) return false;
            if (!EDM94::VisibleServoHeadroomFits(scale, cap, followingWindow,
                axis.Pid_FLUSH.Kp, axis.Pid_FLUSH.Ki, axis.Pid_FLUSH.Kvff, axis.pid.MaxIntegral))
            {
                RtPrintf("[EDM165] event=ADMISSION_REJECT session=%llu reason=FLUSH_SERVO_HEADROOM axis=%u profile=158 kpBits=%llu kiBits=%llu kdBits=%llu kvffBits=%llu armIssued=0\n",
                    static_cast<unsigned long long>(s.session), axisIndex,
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(axis.Pid_FLUSH.Kp)),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(axis.Pid_FLUSH.Ki)),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(axis.Pid_FLUSH.Kd)),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(axis.Pid_FLUSH.Kvff)));
                return false;
            }
            csvCap = cap;
            normalSpeed = (std::min)(EDM94::VisibleFlushReferenceMmSec, cap / scale);
            worstPiPps = normalSpeed * scale * axis.Pid_G00.Kvff + followingWindow * axis.Pid_G00.Kp +
                axis.pid.MaxIntegral * axis.Pid_G00.Ki;
            sparePps = cap - worstPiPps;
            acceleration = EDM128::ReservedVisibleAcceleration158(normalSpeed, (std::min)(cap / accTime, cap / decTime) / scale);
            if (cap < 1.0 || !std::isfinite(normalSpeed) || normalSpeed <= 0.0 ||
                !std::isfinite(acceleration) || acceleration <= 0.0) return false;
        }
        origin[axisIndex] = axis.currentCmdPos / scale;
        end[axisIndex] = (axis.currentCmdPos + s.increment[0][axisIndex] * scale) / scale;
        top[axisIndex] = (axis.currentCmdPos + (axisIndex == 2U ? 5.0 * scale : 0.0)) / scale;
        const double actual = axis.GetNominalActualPositionPulse() / scale;
        const double actualEnd = actual + s.increment[0][axisIndex];
        const double actualTop = actual + (axisIndex == 2U ? 5.0 : 0.0);
        if (!std::isfinite(origin[axisIndex]) || !std::isfinite(end[axisIndex]) ||
            !std::isfinite(top[axisIndex]) || !std::isfinite(actual) || !std::isfinite(actualEnd) || !std::isfinite(actualTop) ||
            CoordSys.GetInvalidSoftwareTravelLimitMask(axis) != 0U ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, origin[axisIndex]) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, end[axisIndex]) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, top[axisIndex]) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, actual) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, actualEnd) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, actualTop)) return false;
        // Match the existing physical guard's enabled range checks even before
        // HOME or when a programmable stroke check is disabled in the NC frame.
        const bool enabled[3] = {axis.travelLimit1Enable, axis.travelLimit2Enable, axis.travelLimit3Enable};
        const double low[3] = {axis.travelLimit1Negative_unit, axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit};
        const double high[3] = {axis.travelLimit1Positive_unit, axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit};
        for (unsigned limit = 0U; limit < 3U; ++limit)
            if (enabled[limit] && (!std::isfinite(low[limit]) || !std::isfinite(high[limit]) ||
                low[limit] >= high[limit] || origin[axisIndex] < low[limit] || origin[axisIndex] > high[limit] ||
                end[axisIndex] < low[limit] || end[axisIndex] > high[limit] ||
                top[axisIndex] < low[limit] || top[axisIndex] > high[limit] ||
                actual < low[limit] || actual > high[limit] || actualEnd < low[limit] || actualEnd > high[limit] ||
                actualTop < low[limit] || actualTop > high[limit])) return false;
    }
    const double finalSpeed = (std::min)(normalSpeed, EDM130::FrozenFinalApproachSpeedMmMin(r) / 60.0);
    if (!std::isfinite(finalSpeed) || finalSpeed <= 0.0) return false;
    const double budget = 2.0 * EDM130::FrozenJumpHeightMm(r) / normalSpeed +
        4.0 * normalSpeed / acceleration + 2.0 * approach / finalSpeed + 3.0 * 0.1 +
        2.0 * maximumStopTime + 0.0015 / machiningSpeed + 5.0;
    if (!std::isfinite(budget) || budget > EDM94::VisibleFlushAdmissionSeconds) return false;
    RtPrintf("[EDM158] event=FULL_LIFT_PREFLIGHT build=FIX4 session=%llu profile=1 rows=1 B=1 source=SOFTWARE_AUTO_WAVE E5us=500000 liftMicrometres=5000 normalMicrometresPerSec=65 hardDeadlineSeconds=180 admissionBudgetSeconds=175 originZBits=%llu endZBits=%llu topZBits=%llu budgetSecondsBits=%llu normalMmSecBits=%llu accelerationMmSec2Bits=%llu cycles=EXACTLY_ONE resumeAfterReturn=REMAINING_NC hold=CANCEL_RESET_REQUIRED S92=INHIBITED S93=INHIBITED\n",
        static_cast<unsigned long long>(s.session),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(origin[2])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(end[2])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(top[2])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(budget)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(normalSpeed)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(acceleration)));
    RtPrintf("[EDM158] event=SERVO_HEADROOM build=FIX4 session=%llu capPpsBits=%llu windowBits=%llu worstPiPpsBits=%llu sparePpsBits=%llu proof=FULL_FROZEN_FOLLOWING_WINDOW gains=CONFIGURED discharge=0\n",
        static_cast<unsigned long long>(s.session),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(csvCap)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(followingWindow)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(worstPiPps)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(sparePps)));
    return true;
}

bool NCManager::PreflightEDM160BoundedTravelSameThread() noexcept
{
    const auto& s = m_edm94; const auto& r = m_edm130.recipe;
    const auto bits = [](double value) noexcept
    { return static_cast<unsigned long long>(EDM130::CurveDoubleBits(value)); };
    const auto reject = [&](const char* reason, int row, int axis, int limit) noexcept
    {
        RtPrintf("[EDM160] event=TRAVEL_PREFLIGHT_REJECT session=%llu reason=%s row=%d axis=%d limit=%d phase=PRE_ARM armIssued=0 authority=NONE\n",
            static_cast<unsigned long long>(s.session), reason, row, axis, limit);
        return false;
    };
    if (!EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) || r.commissioningProfile != 0U ||
        !m_edm130.bound || !s.moves || s.moves > EDM114::RequiredRows ||
        !E156BoundPlanCurrent(s, r) || !EDM130::ValidRecipe(r) || !EDM130::HasBoundHeight(r) ||
        !EDM130::HasBoundDeep(r) || r.deepFlushCycleInterval != 0U ||
        r.diagnosticControlSource != EDM149::SourceForArm(s.gapHistoryArm) ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return reject("SCOPE_OR_RECIPE", -1, -1, -1);
    const double height = EDM130::EffectiveJumpHeightMm(r, 1ULL);
    if (!std::isfinite(height) || height <= 0.0) return reject("EFFECTIVE_HEIGHT", -1, -1, -1);
    bool anyB0 = false, anyB1 = false;
    for (unsigned row = 0U; row < s.moves; ++row)
    {
        const auto mode = m_edm127.expectedMode[row];
        if (!EDM116::ValidMode(mode)) return reject("MODAL_MODE", static_cast<int>(row), -1, -1);
        anyB0 = anyB0 || mode == EDM116::Mode::B0;
        anyB1 = anyB1 || mode == EDM116::Mode::B1;
    }
    // The active row's B policy may flush at a cursor in an older retained
    // curve after GAP recovery. Apply every admitted local mode to every
    // curve, not only to the B word originally authored on that curve.
    std::array<double, 6U> cumulative{}, nominalLow{}, nominalHigh{}, travelLow{}, travelHigh{};
    for (unsigned row = 0U; row < s.moves; ++row)
    {
        const auto* authored = GetBaseProgramCache().TryGetLine(s.pc[row]);
        if (!authored) return reject("CACHE_PC", static_cast<int>(row), -1, -1);
        NCParsedBlock geometry = authored->parsedBlock;
        if (geometry.has('B') != m_edm127.authoredInline[row]) return reject("RAW_B_PRESENCE", static_cast<int>(row), -1, -1);
        if (geometry.has('B'))
        {
            double word = 0.0; EDM116::Mode mode{};
            if (!E94Number(geometry.expression('B'), word) || !EDM116::ParseMode(word, mode) ||
                mode != m_edm127.expectedMode[row]) return reject("RAW_B_MODE", static_cast<int>(row), -1, -1);
            geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
        }
        std::array<double, 6U> delta{}; double feed = 0.0, code = 0.0;
        if (!E114Move(geometry, row, delta, feed, s.gapModeFixture) ||
            !E94Number(geometry.gExpressions[0], code) || code != s.authoredGCode[row] ||
            delta != s.increment[row] || feed != s.feed[row]) return reject("ROW_LITERAL", static_cast<int>(row), -1, -1);
        EDM112::Curve curve{}; EDM112::Bounds bounds{};
        if (code == 1.0)
        {
            EDM109::Point start{}, end{}, scale{};
            for (unsigned axis = 0U; axis < 6U; ++axis)
            { scale[axis] = 1000000.0; end[axis] = delta[axis] * scale[axis]; }
            EDM109::Line line{};
            if (!EDM109::BuildLine(start, end, scale, line) || !EDM112::BuildLine(line, curve))
                return reject("LINE_CURVE", static_cast<int>(row), -1, -1);
        }
        else
        {
            std::array<double, 2U> center{};
            if (!E155ArcCenter(geometry, center, s.gapModeFixture) ||
                !EDM114::BuildBoundedXYArc(static_cast<int>(code), delta, center, feed, curve))
                return reject("ARC_CURVE", static_cast<int>(row), -1, -1);
        }
        if (!EDM112::WholeCurveBounds(curve, bounds) || !bounds.valid ||
            !std::isfinite(curve.lengthMm) || curve.lengthMm <= 0.0)
            return reject("WHOLE_CURVE_BOUNDS", static_cast<int>(row), -1, -1);
        for (unsigned axis = 0U; axis < 6U; ++axis)
        {
            const double low = cumulative[axis] + bounds.minNative[axis];
            const double high = cumulative[axis] + bounds.maxNative[axis];
            if (!std::isfinite(low) || !std::isfinite(high) || low > high)
                return reject("NATIVE_BOUNDS", static_cast<int>(row), static_cast<int>(axis), -1);
            nominalLow[axis] = (std::min)(nominalLow[axis], low);
            nominalHigh[axis] = (std::max)(nominalHigh[axis], high);
            double flushLow = low, flushHigh = high;
            if (anyB0)
            {
                if (code == 1.0)
                {
                    // Native XYZ displacement defines millimetres of path;
                    // C/U/V keep their own degrees per millimetre of tangent.
                    const double shift = -height * delta[axis] / curve.lengthMm;
                    if (!std::isfinite(shift)) return reject("B0_TANGENT", static_cast<int>(row), static_cast<int>(axis), -1);
                    flushLow = (std::min)(flushLow, low + shift);
                    flushHigh = (std::max)(flushHigh, high + shift);
                }
                else if (axis < 2U)
                { flushLow -= height; flushHigh += height; }
            }
            if (anyB1 && axis == 2U) flushHigh = (std::max)(flushHigh, high + height);
            // B2 remains inside this same complete retained curve union.
            travelLow[axis] = (std::min)(travelLow[axis], flushLow);
            travelHigh[axis] = (std::max)(travelHigh[axis], flushHigh);
            cumulative[axis] += delta[axis];
            const double limit = axis < 3U ? 0.025 : 0.5;
            if (!std::isfinite(cumulative[axis]) || !std::isfinite(travelLow[axis]) || !std::isfinite(travelHigh[axis]) ||
                nominalLow[axis] < -limit || nominalHigh[axis] > limit ||
                travelLow[axis] < -limit || travelHigh[axis] > limit)
            {
                reject("FROZEN_GUARD_ENVELOPE", static_cast<int>(row), static_cast<int>(axis), -1);
                RtPrintf("[EDM160] event=TRAVEL_DETAIL session=%llu part=GUARD_ENVELOPE row=%u axis=%u nominalLowBits=%llu nominalHighBits=%llu motionLowBits=%llu motionHighBits=%llu limitBits=%llu heightBits=%llu\n",
                    static_cast<unsigned long long>(s.session), row, axis, bits(nominalLow[axis]), bits(nominalHigh[axis]),
                    bits(travelLow[axis]), bits(travelHigh[axis]), bits(limit), bits(height));
                return false;
            }
        }
    }
    for (unsigned axisIndex = 0U; axisIndex < 6U; ++axisIndex)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(axisIndex));
        const double scale = axis.resolution_PPR / axis.finalLead;
        if (!axis.isExist || axis.axisIndex != static_cast<int>(axisIndex) ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 || !std::isfinite(scale) || scale <= 0.0 ||
            !std::isfinite(axis.currentCmdPos)) return reject("AXIS_NATIVE_CONFIG", -1, static_cast<int>(axisIndex), -1);
        const double command = axis.currentCmdPos / scale;
        const double actual = axis.GetNominalActualPositionPulse() / scale;
        const bool enabled[3] = {axis.travelLimit1Enable, axis.travelLimit2Enable, axis.travelLimit3Enable};
        const double lower[3] = {axis.travelLimit1Negative_unit, axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit};
        const double upper[3] = {axis.travelLimit1Positive_unit, axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit};
        const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(axis);
        if (invalidMask != 0U)
        {
            reject("COORD_LIMIT_CONFIG", -1, static_cast<int>(axisIndex), -1);
            RtPrintf("[EDM160] event=TRAVEL_DETAIL session=%llu part=LIMIT_CONFIG axis=%u invalidMask=%u commandBits=%llu actualBits=%llu low1Bits=%llu high1Bits=%llu low2Bits=%llu high2Bits=%llu low3Bits=%llu high3Bits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex, invalidMask, bits(command), bits(actual),
                bits(lower[0]), bits(upper[0]), bits(lower[1]), bits(upper[1]), bits(lower[2]), bits(upper[2]));
            return false;
        }
        const double points[6] = {command, actual, command + travelLow[axisIndex], command + travelHigh[axisIndex],
            actual + travelLow[axisIndex], actual + travelHigh[axisIndex]};
        const char* sources[6] = {"COMMAND_ORIGIN", "ACTUAL_ORIGIN", "COMMAND_LOW", "COMMAND_HIGH", "ACTUAL_LOW", "ACTUAL_HIGH"};
        for (unsigned pointIndex = 0U; pointIndex < 6U; ++pointIndex)
        {
            const double point = points[pointIndex];
            if (!std::isfinite(point) || !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, point))
            {
                reject(!std::isfinite(point) ? "TRAVEL_POINT_NONFINITE" : "COORD_TARGET_LIMIT", -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM160] event=TRAVEL_DETAIL session=%llu part=TARGET axis=%u pointIndex=%u source=%s commandBits=%llu actualBits=%llu pointBits=%llu motionLowBits=%llu motionHighBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, pointIndex, sources[pointIndex], bits(command), bits(actual),
                    bits(point), bits(travelLow[axisIndex]), bits(travelHigh[axisIndex]));
                return false;
            }
            for (unsigned limit = 0U; limit < 3U; ++limit)
                if (enabled[limit] && (!std::isfinite(lower[limit]) || !std::isfinite(upper[limit]) ||
                    lower[limit] >= upper[limit] || point < lower[limit] || point > upper[limit]))
                {
                    reject("ENABLED_TARGET_LIMIT", -1, static_cast<int>(axisIndex), static_cast<int>(limit));
                    RtPrintf("[EDM160] event=TRAVEL_DETAIL session=%llu part=ENABLED_TARGET axis=%u limit=%u pointIndex=%u source=%s commandBits=%llu actualBits=%llu pointBits=%llu lowBits=%llu highBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, limit, pointIndex, sources[pointIndex], bits(command), bits(actual),
                        bits(point), bits(lower[limit]), bits(upper[limit]));
                    return false;
                }
        }
    }
    RtPrintf("[EDM160] event=TRAVEL_PREFLIGHT session=%llu rows=%u requiredMask=%u heightBits=%llu anyB0=%u anyB1=%u geometry=WHOLE_CURVE bounds=ALL_PLAN_HISTORY_POLICY_UNION origins=COMMAND_AND_ACTUAL limits=ALL_ENABLED phase=PRE_ARM authority=NONE\n",
        static_cast<unsigned long long>(s.session), s.moves, EDM130::ProgramMovementMask(r), bits(height), anyB0 ? 1U : 0U, anyB1 ? 1U : 0U);
    return true;
}

bool NCManager::PreflightEDM159XYZVisibleTravelSameThread() noexcept
{
    const auto& s = m_edm94; const auto& r = m_edm130.recipe;
    // FIX1: one first-failure report per NC admission call. These observations
    // neither consume the M00 ticket nor issue an arm or movement permission.
    const auto bits = [](double value) noexcept
    { return static_cast<unsigned long long>(EDM130::CurveDoubleBits(value)); };
    const auto reject = [&](const char* reason, int row, int axis, int limit) noexcept
    {
        RtPrintf("[EDM159] event=ADMISSION_REJECT build=FIX2 session=%llu reason=%s row=%d axis=%d limit=%d phase=PRE_ARM armIssued=0 motionPermission=NOT_GRANTED dischargePermission=NOT_GRANTED\n",
            static_cast<unsigned long long>(s.session), reason, row, axis, limit);
        return false;
    };
    if (!EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm)) return reject("ARM_SCOPE", -1, -1, -1);
    if (!m_edm130.bound) return reject("RECIPE_UNBOUND", -1, -1, -1);
    if (!E159VisiblePlan(s)) return reject("EXACT_THREE_ROW_PLAN", -1, -1, -1);
    if (!E156BoundPlanCurrent(s, r)) return reject("BOUND_PLAN", -1, -1, -1);
    if (!EDM130::IsVisibleFlush159(r)) return reject("COMMISSIONING_PROFILE", -1, -1, -1);
    if (!EDM130::ValidRecipe(r)) return reject("RECIPE_VALIDITY", -1, -1, -1);
    if (!IsPathCoreLiveNativeConfigCurrentSameThread()) return reject("NATIVE_CONFIG_CURRENT", -1, -1, -1);
    for (unsigned row = 0U; row < 3U; ++row)
        if (m_edm127.authoredInline[row] != (row == 0U) ||
            m_edm127.expectedMode[row] != EDM116::Mode::B1 ||
            m_edm127.expectedOriginPC[row] != m_edm127.expectedOriginPC[0] ||
            m_edm127.expectedOriginRevision[row] != m_edm127.expectedOriginRevision[0])
        {
            const char* reason = m_edm127.authoredInline[row] != (row == 0U) ? "MODAL_INLINE" :
                m_edm127.expectedMode[row] != EDM116::Mode::B1 ? "MODAL_B1" :
                m_edm127.expectedOriginPC[row] != m_edm127.expectedOriginPC[0] ? "MODAL_ORIGIN_PC" : "MODAL_ORIGIN_REVISION";
            reject(reason, static_cast<int>(row), -1, -1);
            RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=MODAL row=%u inlineB=%u mode=%u originPC=%d firstOriginPC=%d originRevision=%llu firstOriginRevision=%llu\n",
                static_cast<unsigned long long>(s.session), row, m_edm127.authoredInline[row] ? 1U : 0U,
                static_cast<unsigned>(m_edm127.expectedMode[row]), m_edm127.expectedOriginPC[row], m_edm127.expectedOriginPC[0],
                static_cast<unsigned long long>(m_edm127.expectedOriginRevision[row]),
                static_cast<unsigned long long>(m_edm127.expectedOriginRevision[0]));
            return false;
        }
    // Rebuild every frozen literal with the existing common LINEAR6/ARC2
    // geometry. Whole-arc extrema, including omitted endpoint components,
    // define the envelope; an endpoint chord is not an arc travel proof.
    std::array<double, 6U> cumulative{}, low{}, high{};
    std::array<double, 3U> lengths{};
    double nominalLength = 0.0;
    for (unsigned row = 0U; row < 3U; ++row)
    {
        const auto* authored = GetBaseProgramCache().TryGetLine(s.pc[row]);
        if (!authored)
        {
            reject("CACHE_PC", static_cast<int>(row), -1, -1);
            RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=CACHE row=%u sourcePC=%d\n",
                static_cast<unsigned long long>(s.session), row, s.pc[row]);
            return false;
        }
        NCParsedBlock geometry = authored->parsedBlock;
        if (geometry.has('B') != m_edm127.authoredInline[row]) return reject("RAW_B_PRESENCE", static_cast<int>(row), -1, -1);
        if (geometry.has('B'))
        {
            double word = 0.0;
            if (!E94Number(geometry.expression('B'), word) || word != 1.0) return reject("RAW_B1_LITERAL", static_cast<int>(row), -1, -1);
            geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
        }
        std::array<double, 6U> delta{}; double feed = 0.0;
        if (!E114Move(geometry, row, delta, feed, s.gapModeFixture)) return reject("ROW_LITERAL", static_cast<int>(row), -1, -1);
        if (delta != s.increment[row]) return reject("ROW_INCREMENT", static_cast<int>(row), -1, -1);
        if (feed != s.feed[row]) return reject("ROW_FEED", static_cast<int>(row), -1, -1);
        EDM112::Curve curve{}; EDM112::Bounds bounds{};
        if (row == 0U)
        {
            EDM109::Point start{}, end{}, scale{};
            for (unsigned axis = 0U; axis < 6U; ++axis)
            { scale[axis] = 1000000.0; end[axis] = delta[axis] * scale[axis]; }
            EDM109::Line line{};
            if (!EDM109::BuildLine(start, end, scale, line) || !EDM112::BuildLine(line, curve))
                return reject("LINE_CURVE", static_cast<int>(row), -1, -1);
        }
        else
        {
            std::array<double, 2U> center{};
            if (!E155ArcCenter(geometry, center, s.gapModeFixture) ||
                center[0] != (row == 1U ? 0.001 : -0.001) || center[1] != 0.0 ||
                !EDM114::BuildBoundedXYArc(static_cast<int>(s.authoredGCode[row]), delta, center, feed, curve))
                return reject("ARC_CURVE", static_cast<int>(row), -1, -1);
        }
        if (!EDM112::WholeCurveBounds(curve, bounds) || !bounds.valid ||
            !std::isfinite(curve.lengthMm) || curve.lengthMm <= 0.0)
            return reject("WHOLE_CURVE_BOUNDS", static_cast<int>(row), -1, -1);
        lengths[row] = curve.lengthMm; nominalLength += curve.lengthMm;
        for (unsigned axis = 0U; axis < 6U; ++axis)
        {
            const double rowLow = cumulative[axis] + bounds.minNative[axis];
            const double rowHigh = cumulative[axis] + bounds.maxNative[axis];
            if (!std::isfinite(rowLow) || !std::isfinite(rowHigh) || rowLow > rowHigh)
                return reject("ROW_NATIVE_BOUNDS", static_cast<int>(row), static_cast<int>(axis), -1);
            low[axis] = (std::min)(low[axis], rowLow); high[axis] = (std::max)(high[axis], rowHigh);
            cumulative[axis] += delta[axis];
            if (!std::isfinite(cumulative[axis]) ||
                low[axis] < (axis < 3U ? -0.025 : -0.5) ||
                high[axis] > (axis < 3U ? 0.025 : 0.5))
            {
                reject("NOMINAL_ENVELOPE", static_cast<int>(row), static_cast<int>(axis), -1);
                RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=NOMINAL_ENVELOPE row=%u axis=%u cumulativeBits=%llu lowBits=%llu highBits=%llu limitBits=%llu\n",
                    static_cast<unsigned long long>(s.session), row, axis, bits(cumulative[axis]), bits(low[axis]), bits(high[axis]), bits(axis < 3U ? 0.025 : 0.5));
                return false;
            }
        }
    }
    const double reference = EDM130::FrozenGapReferenceVoltageV(r);
    const auto profile = EDM97::MakeProfile(s.gapTuning, r);
    const auto forward = EDMGapServo::Evaluate(profile, reference + 38.0, reference,
        EDM130::FrozenMachiningOverridePercent(r));
    // AUTO_WAVE's low phase is ref-20V with at most +1V noise. Budget the
    // slower -19V case; the initial 160 zero samples use the same short path.
    const auto backward = EDMGapServo::Evaluate(profile, reference - 19.0, reference,
        EDM130::FrozenMachiningOverridePercent(r));
    const double forwardSpeed = (std::min)(0.12, forward.limitedSpeedMmPerMin) / 60.0;
    const double backwardSpeed = (std::min)(0.03, -backward.limitedSpeedMmPerMin) / 60.0;
    if (!forward.valid || !backward.valid || !std::isfinite(nominalLength) || nominalLength <= 0.0 ||
        !std::isfinite(forwardSpeed) || forwardSpeed <= 0.0 ||
        !std::isfinite(backwardSpeed) || backwardSpeed <= 0.0)
    {
        reject("GAP_CURVE_SPEED", -1, -1, -1);
        RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=GAP_SPEED forwardValid=%u backwardValid=%u nominalLengthBits=%llu forwardBits=%llu backwardBits=%llu referenceBits=%llu\n",
            static_cast<unsigned long long>(s.session), forward.valid ? 1U : 0U, backward.valid ? 1U : 0U,
            bits(nominalLength), bits(forwardSpeed), bits(backwardSpeed), bits(reference));
        return false;
    }
    double normalSpeed = 0.0, acceleration = 0.0, maximumConfiguredTime = 0.2;
    double capPps[3]{}, windowPulse[3]{}, worstPiPps[3]{}, sparePps[3]{};
    double feedforwardPps[3]{}, availablePps[3]{};
    for (unsigned axisIndex = 0U; axisIndex < 6U; ++axisIndex)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(axisIndex));
        const double scale = axis.resolution_PPR / axis.finalLead;
        if (!axis.isExist || axis.axisIndex != static_cast<int>(axisIndex) ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 || !std::isfinite(scale) || scale <= 0.0 ||
            !std::isfinite(axis.currentCmdPos) || !std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.0 ||
            !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
            !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0)
        {
            const char* reason = !axis.isExist ? "AXIS_ABSENT" : axis.axisIndex != static_cast<int>(axisIndex) ? "AXIS_IDENTITY" :
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ? "AXIS_LEAD" :
                !std::isfinite(scale) || scale <= 0.0 ? "AXIS_PPU" : !std::isfinite(axis.currentCmdPos) ? "AXIS_COMMAND" :
                !std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.0 ? "AXIS_STOP_TIME" :
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ? "AXIS_ACCEL_TIME" : "AXIS_DECEL_TIME";
            reject(reason, -1, static_cast<int>(axisIndex), -1);
            RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=AXIS_CONFIG axis=%u exists=%u configuredAxis=%d leadBits=%llu pprBits=%llu ppuBits=%llu commandPulseBits=%llu stopBits=%llu accBits=%llu decBits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex, axis.isExist ? 1U : 0U, axis.axisIndex,
                bits(axis.finalLead), bits(axis.resolution_PPR), bits(scale), bits(axis.currentCmdPos),
                bits(axis.Stop_dec_time), bits(axis.G00_acc_time), bits(axis.G00_dec_time));
            return false;
        }
        const double accTime = axis.G00_acc_time < 0.001 ? 0.2 : axis.G00_acc_time;
        const double decTime = axis.G00_dec_time < 0.001 ? 0.2 : axis.G00_dec_time;
        maximumConfiguredTime = (std::max)(maximumConfiguredTime, axis.Stop_dec_time < 0.001 ? 0.2 : axis.Stop_dec_time);
        maximumConfiguredTime = (std::max)(maximumConfiguredTime, (std::max)(accTime, decTime));
        if (axisIndex < 3U)
        {
            if (!std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.inPositionWindow_Pulse) || axis.inPositionWindow_Pulse <= 0.0 ||
                !std::isfinite(axis.pid.MaxLag) || axis.pid.MaxLag <= 0.0)
            {
                const char* reason = !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ? "AXIS_VELOCITY_CAP" :
                    !std::isfinite(axis.inPositionWindow_Pulse) || axis.inPositionWindow_Pulse <= 0.0 ? "RAW_POSITION_WINDOW" : "RAW_MAX_LAG";
                reject(reason, -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=RAW_WINDOW axis=%u maxVelBits=%llu inPositionBits=%llu maxLagBits=%llu ppuBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(axis.maxVel_PPS), bits(axis.inPositionWindow_Pulse), bits(axis.pid.MaxLag), bits(scale));
                return false;
            }
            const double cap = std::floor((std::min)(axis.maxVel_PPS, scale * EDM94::NativeVelocityLimit(axisIndex)));
            const double window = (std::min)(axis.inPositionWindow_Pulse, (std::min)(axis.pid.MaxLag, scale * 0.001));
            const double referenceSpeed = axisIndex == 2U ? EDM94::VisibleFlushReferenceMmSec :
                EDM94::VisibleXYZMachiningReferenceMmSec;
            // FIX2: normal low-feed XY keeps configured PI and the existing
            // per-tick following/output guards. The Z lift retains its complete
            // frozen-window headroom requirement below.
            if (axisIndex < 2U)
            {
                if (!EDM94::BoundedXYMachiningServoFits(scale, cap, window, axis.Pid_G00.Kp,
                    axis.Pid_G00.Ki, axis.Pid_G00.Kd, axis.Pid_G00.Kvff, axis.pid.MaxIntegral))
                {
                    reject("XY_CONFIGURED_SERVO_ADMISSION", -1, static_cast<int>(axisIndex), -1);
                    const double ff = referenceSpeed * scale * axis.Pid_G00.Kvff;
                    const double p = window * axis.Pid_G00.Kp, i = axis.pid.MaxIntegral * axis.Pid_G00.Ki;
                    const double reserve = 0.004 * scale, available = cap - reserve;
                    RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=XY_SERVO_CONFIG axis=%u ppuBits=%llu rawCapBits=%llu integerCapBits=%llu referenceBits=%llu rawInPositionBits=%llu rawMaxLagBits=%llu windowBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, bits(scale), bits(axis.maxVel_PPS), bits(cap), bits(referenceSpeed),
                        bits(axis.inPositionWindow_Pulse), bits(axis.pid.MaxLag), bits(window));
                    RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=XY_SERVO_GAINS axis=%u kpBits=%llu kiBits=%llu kdBits=%llu kvffBits=%llu maxIntegralBits=%llu values=CURRENT_NATIVE_CHECKED gainWrite=NONE\n",
                        static_cast<unsigned long long>(s.session), axisIndex, bits(axis.Pid_G00.Kp), bits(axis.Pid_G00.Ki), bits(axis.Pid_G00.Kd), bits(axis.Pid_G00.Kvff), bits(axis.pid.MaxIntegral));
                    RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=XY_SERVO_SUM axis=%u ffBits=%llu pBits=%llu iBits=%llu theoreticalSumBits=%llu reserveBits=%llu availableBits=%llu sumRole=OBSERVATION_ONLY admissionBound=FEEDFORWARD_ONLY\n",
                        static_cast<unsigned long long>(s.session), axisIndex, bits(ff), bits(p), bits(i), bits(ff + p + i), bits(reserve), bits(available));
                    return false;
                }
            }
            else if (!EDM94::VisibleServoHeadroomFits(scale, cap, window, axis.Pid_G00.Kp, axis.Pid_G00.Ki,
                axis.Pid_G00.Kvff, axis.pid.MaxIntegral, referenceSpeed))
            {
                reject("SERVO_HEADROOM", -1, static_cast<int>(axisIndex), -1);
                const double ff = referenceSpeed * scale * axis.Pid_G00.Kvff;
                const double p = window * axis.Pid_G00.Kp, i = axis.pid.MaxIntegral * axis.Pid_G00.Ki;
                const double reserve = 0.004 * scale, available = cap - reserve;
                RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=HEADROOM_CONFIG axis=%u ppuBits=%llu rawCapBits=%llu integerCapBits=%llu referenceBits=%llu rawInPositionBits=%llu rawMaxLagBits=%llu windowBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(scale), bits(axis.maxVel_PPS), bits(cap), bits(referenceSpeed),
                    bits(axis.inPositionWindow_Pulse), bits(axis.pid.MaxLag), bits(window));
                RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=HEADROOM_GAINS axis=%u kpBits=%llu kiBits=%llu kvffBits=%llu maxIntegralBits=%llu values=CURRENT_NATIVE_CHECKED gainWrite=NONE\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(axis.Pid_G00.Kp), bits(axis.Pid_G00.Ki), bits(axis.Pid_G00.Kvff), bits(axis.pid.MaxIntegral));
                RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=HEADROOM_SUM axis=%u ffBits=%llu pBits=%llu iBits=%llu requiredBits=%llu reserveBits=%llu availableBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(ff), bits(p), bits(i), bits(ff + p + i), bits(reserve), bits(available));
                return false;
            }
            const bool flushHeadroom = axisIndex < 2U ?
                EDM94::BoundedXYMachiningServoFits(scale, cap, window, axis.Pid_FLUSH.Kp,
                    axis.Pid_FLUSH.Ki, axis.Pid_FLUSH.Kd, axis.Pid_FLUSH.Kvff, axis.pid.MaxIntegral) :
                EDM94::VisibleServoHeadroomFits(scale, cap, window, axis.Pid_FLUSH.Kp,
                    axis.Pid_FLUSH.Ki, axis.Pid_FLUSH.Kvff, axis.pid.MaxIntegral, referenceSpeed);
            if (!flushHeadroom)
            {
                reject("FLUSH_SERVO_HEADROOM", -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM165] event=ADMISSION_DETAIL session=%llu part=FLUSH_SERVO_GAINS axis=%u profile=159 kpBits=%llu kiBits=%llu kdBits=%llu kvffBits=%llu values=CONFIGURED gainWrite=NONE\n",
                    static_cast<unsigned long long>(s.session), axisIndex,
                    bits(axis.Pid_FLUSH.Kp), bits(axis.Pid_FLUSH.Ki), bits(axis.Pid_FLUSH.Kd), bits(axis.Pid_FLUSH.Kvff));
                return false;
            }
            capPps[axisIndex] = cap; windowPulse[axisIndex] = window;
            feedforwardPps[axisIndex] = referenceSpeed * scale * axis.Pid_G00.Kvff;
            availablePps[axisIndex] = cap - 0.004 * scale;
            worstPiPps[axisIndex] = referenceSpeed * scale * axis.Pid_G00.Kvff +
                window * axis.Pid_G00.Kp + axis.pid.MaxIntegral * axis.Pid_G00.Ki;
            sparePps[axisIndex] = cap - worstPiPps[axisIndex];
            if (axisIndex == 2U)
            {
                normalSpeed = (std::min)(EDM94::VisibleFlushReferenceMmSec, cap / scale);
                acceleration = EDM128::ReservedVisibleAcceleration(normalSpeed,
                    (std::min)(cap / accTime, cap / decTime) / scale, 2.0);
                if (!std::isfinite(normalSpeed) || normalSpeed <= 0.0 ||
                    !std::isfinite(acceleration) || acceleration <= 0.0)
                {
                    reject("Z_LIFT_DYNAMICS", -1, static_cast<int>(axisIndex), -1);
                    RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=Z_DYNAMICS speedBits=%llu accelerationBits=%llu capBits=%llu ppuBits=%llu accTimeBits=%llu decTimeBits=%llu\n",
                        static_cast<unsigned long long>(s.session), bits(normalSpeed), bits(acceleration), bits(cap), bits(scale), bits(accTime), bits(decTime));
                    return false;
                }
            }
        }
        const double command = axis.currentCmdPos / scale;
        const double actual = axis.GetNominalActualPositionPulse() / scale;
        // E5 may pause only committed path geometry. Include a 2mm positive-Z
        // lift at every admitted position, for both command and actual origin.
        const double lift = axisIndex == 2U ? 2.0 : 0.0;
        const double points[6] = {command, actual, command + low[axisIndex], command + high[axisIndex] + lift,
            actual + low[axisIndex], actual + high[axisIndex] + lift};
        const unsigned invalidLimitMask = CoordSys.GetInvalidSoftwareTravelLimitMask(axis);
        if (invalidLimitMask != 0U)
        {
            reject("COORD_LIMIT_CONFIG", -1, static_cast<int>(axisIndex), -1);
            RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=COORD_LIMIT_CONFIG axis=%u invalidMask=%u commandBits=%llu actualBits=%llu nominalLowBits=%llu nominalHighBits=%llu liftBits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex, invalidLimitMask, bits(command), bits(actual), bits(low[axisIndex]), bits(high[axisIndex]), bits(lift));
            RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=INVALID_LIMIT_RANGES axis=%u enabledMask=%u low1Bits=%llu high1Bits=%llu low2Bits=%llu high2Bits=%llu low3Bits=%llu high3Bits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex,
                (axis.travelLimit1Enable ? 1U : 0U) | (axis.travelLimit2Enable ? 2U : 0U) | (axis.travelLimit3Enable ? 4U : 0U),
                bits(axis.travelLimit1Negative_unit), bits(axis.travelLimit1Positive_unit), bits(axis.travelLimit2Negative_unit),
                bits(axis.travelLimit2Positive_unit), bits(axis.travelLimit3Negative_unit), bits(axis.travelLimit3Positive_unit));
            return false;
        }
        const bool enabled[3] = {axis.travelLimit1Enable, axis.travelLimit2Enable, axis.travelLimit3Enable};
        const double lower[3] = {axis.travelLimit1Negative_unit, axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit};
        const double upper[3] = {axis.travelLimit1Positive_unit, axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit};
        const char* pointSources[6] = {"COMMAND_ORIGIN", "ACTUAL_ORIGIN", "COMMAND_PATH_LOW", "COMMAND_PATH_HIGH_LIFT", "ACTUAL_PATH_LOW", "ACTUAL_PATH_HIGH_LIFT"};
        for (unsigned pointIndex = 0U; pointIndex < 6U; ++pointIndex)
        {
            const double point = points[pointIndex];
            if (!std::isfinite(point) || !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, point))
            {
                reject(!std::isfinite(point) ? "TRAVEL_POINT_NONFINITE" : "COORD_TARGET_LIMIT", -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=TRAVEL_POINT axis=%u pointIndex=%u source=%s commandBits=%llu actualBits=%llu pointBits=%llu nominalLowBits=%llu nominalHighBits=%llu liftBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, pointIndex, pointSources[pointIndex], bits(command), bits(actual), bits(point), bits(low[axisIndex]), bits(high[axisIndex]), bits(lift));
                for (unsigned limit = 0U; limit < 3U; ++limit)
                    RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=COORD_TARGET_LIMIT axis=%u limit=%u enabled=%u lowBits=%llu highBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, limit, enabled[limit] ? 1U : 0U, bits(lower[limit]), bits(upper[limit]));
                return false;
            }
            for (unsigned limit = 0U; limit < 3U; ++limit)
                if (enabled[limit] && (!std::isfinite(lower[limit]) || !std::isfinite(upper[limit]) ||
                    lower[limit] >= upper[limit] || point < lower[limit] || point > upper[limit]))
                {
                    reject("ENABLED_TARGET_LIMIT", -1, static_cast<int>(axisIndex), static_cast<int>(limit));
                    RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=ENABLED_TARGET_LIMIT axis=%u limit=%u pointIndex=%u source=%s commandBits=%llu actualBits=%llu pointBits=%llu lowBits=%llu highBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, limit, pointIndex, pointSources[pointIndex], bits(command), bits(actual), bits(point), bits(lower[limit]), bits(upper[limit]));
                    return false;
                }
        }
    }
    const double approach = EDM130::FrozenApproachDistanceMm(r);
    const double finalSpeed = (std::min)(normalSpeed, EDM130::FrozenFinalApproachSpeedMmMin(r) / 60.0);
    if (!std::isfinite(approach) || approach < 0.0 || !std::isfinite(finalSpeed) || finalSpeed <= 0.0)
    {
        reject("FINAL_APPROACH", -1, 2, -1);
        RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=FINAL_APPROACH approachBits=%llu finalSpeedBits=%llu normalSpeedBits=%llu configuredFinalMmMinBits=%llu\n",
            static_cast<unsigned long long>(s.session), bits(approach), bits(finalSpeed), bits(normalSpeed), bits(EDM130::FrozenFinalApproachSpeedMmMin(r)));
        return false;
    }
    // Row index2's existing source can visit at most its current arc and the
    // immediately preceding arc. Charge both complete arcs in each direction,
    // plus all authored forward geometry and every group acceleration/stop.
    const double historyLength = lengths[1] + lengths[2];
    const double budget = 2.0 * EDM130::FrozenJumpHeightMm(r) / normalSpeed + 4.0 * normalSpeed / acceleration +
        2.0 * approach / finalSpeed + 3.0 * 0.1 + 10.0 * maximumConfiguredTime +
        nominalLength / forwardSpeed + historyLength / backwardSpeed + historyLength / forwardSpeed + 0.04 + 5.0;
    if (!std::isfinite(historyLength) || historyLength <= 0.0 || !std::isfinite(budget) || budget > 115.0)
    {
        reject("SESSION_BUDGET", -1, -1, -1);
        RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=BUDGET_LENGTH row0Bits=%llu row1Bits=%llu row2Bits=%llu nominalBits=%llu historyBits=%llu heightBits=%llu approachBits=%llu\n",
            static_cast<unsigned long long>(s.session), bits(lengths[0]), bits(lengths[1]), bits(lengths[2]), bits(nominalLength), bits(historyLength), bits(EDM130::FrozenJumpHeightMm(r)), bits(approach));
        RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=BUDGET_DYNAMICS forwardBits=%llu backwardBits=%llu finalBits=%llu liftSpeedBits=%llu accelerationBits=%llu maxConfiguredTimeBits=%llu\n",
            static_cast<unsigned long long>(s.session), bits(forwardSpeed), bits(backwardSpeed), bits(finalSpeed), bits(normalSpeed), bits(acceleration), bits(maximumConfiguredTime));
        RtPrintf("[EDM159] event=ADMISSION_DETAIL build=FIX2 session=%llu part=BUDGET_TOTAL computedBits=%llu ceilingBits=%llu admissionCeilingSeconds=115 hardDeadlineSeconds=120\n",
            static_cast<unsigned long long>(s.session), bits(budget), bits(115.0));
        return false;
    }
    RtPrintf("[EDM159] event=PLAN_PREFLIGHT session=%llu rows=3 mask=7 source=SOFTWARE_WAVE heightMicrometres=2000 referenceMicrometresPerSec=65 hardDeadlineSeconds=120 admissionBudgetSeconds=115 nominalLengthBits=%llu historyLengthBits=%llu budgetSecondsBits=%llu accelerationBits=%llu discharge=0\n",
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(EDM130::CurveDoubleBits(nominalLength)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(historyLength)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(budget)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(acceleration)));
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        if (axis < 2U)
        {
            RtPrintf("[EDM159] event=XY_SERVO_ADMISSION build=FIX2 session=%llu axis=%c capPpsBits=%llu windowBits=%llu ffBits=%llu availableBits=%llu policy=CONFIGURED_PI_RUNTIME_GUARDED protection=POST_PI_CAP_AND_I_ROLLBACK gainWrite=NONE discharge=0\n",
                static_cast<unsigned long long>(s.session), "XYZ"[axis], bits(capPps[axis]), bits(windowPulse[axis]), bits(feedforwardPps[axis]), bits(availablePps[axis]));
            RtPrintf("[EDM159] event=XY_SERVO_OBSERVATION build=FIX2 session=%llu axis=%c theoreticalSumBits=%llu theoreticalCapDifferenceBits=%llu sumRole=OBSERVATION_ONLY admissionBound=FEEDFORWARD_ONLY\n",
                static_cast<unsigned long long>(s.session), "XYZ"[axis], bits(worstPiPps[axis]), bits(sparePps[axis]));
        }
        else
            RtPrintf("[EDM159] event=SERVO_HEADROOM session=%llu axis=%c capPpsBits=%llu windowBits=%llu worstPiPpsBits=%llu sparePpsBits=%llu proof=FULL_FROZEN_FOLLOWING_WINDOW gains=CONFIGURED discharge=0\n",
                static_cast<unsigned long long>(s.session), "XYZ"[axis],
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(capPps[axis])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(windowPulse[axis])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(worstPiPps[axis])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(sparePps[axis])));
    }
    return true;
}


bool NCManager::PreflightEDM167SixVisibleTravelSameThread() noexcept
{
    const auto& s = m_edm94; const auto& r = m_edm130.recipe;
    // FIX1: one first-failure report per NC admission call. These observations
    // neither consume the M00 ticket nor issue an arm or movement permission.
    const auto bits = [](double value) noexcept
    { return static_cast<unsigned long long>(EDM130::CurveDoubleBits(value)); };
    const auto reject = [&](const char* reason, int row, int axis, int limit) noexcept
    {
        RtPrintf("[EDM167] event=ADMISSION_REJECT build=SIX_AXIS session=%llu reason=%s row=%d axis=%d limit=%d phase=PRE_ARM armIssued=0 motionPermission=NOT_GRANTED dischargePermission=NOT_GRANTED\n",
            static_cast<unsigned long long>(s.session), reason, row, axis, limit);
        return false;
    };
    if (!EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm)) return reject("ARM_SCOPE", -1, -1, -1);
    if (!m_edm130.bound) return reject("RECIPE_UNBOUND", -1, -1, -1);
    if (!E167VisiblePlan(s)) return reject("EXACT_THREE_ROW_PLAN", -1, -1, -1);
    if (!E156BoundPlanCurrent(s, r)) return reject("BOUND_PLAN", -1, -1, -1);
    if (!EDM130::IsVisibleFlush167(r)) return reject("COMMISSIONING_PROFILE", -1, -1, -1);
    if (!EDM130::ValidRecipe(r)) return reject("RECIPE_VALIDITY", -1, -1, -1);
    if (!IsPathCoreLiveNativeConfigCurrentSameThread()) return reject("NATIVE_CONFIG_CURRENT", -1, -1, -1);
    for (unsigned row = 0U; row < 3U; ++row)
        if (m_edm127.authoredInline[row] != (row == 0U) ||
            m_edm127.expectedMode[row] != EDM116::Mode::B1 ||
            m_edm127.expectedOriginPC[row] != m_edm127.expectedOriginPC[0] ||
            m_edm127.expectedOriginRevision[row] != m_edm127.expectedOriginRevision[0])
        {
            const char* reason = m_edm127.authoredInline[row] != (row == 0U) ? "MODAL_INLINE" :
                m_edm127.expectedMode[row] != EDM116::Mode::B1 ? "MODAL_B1" :
                m_edm127.expectedOriginPC[row] != m_edm127.expectedOriginPC[0] ? "MODAL_ORIGIN_PC" : "MODAL_ORIGIN_REVISION";
            reject(reason, static_cast<int>(row), -1, -1);
            RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=MODAL row=%u inlineB=%u mode=%u originPC=%d firstOriginPC=%d originRevision=%llu firstOriginRevision=%llu\n",
                static_cast<unsigned long long>(s.session), row, m_edm127.authoredInline[row] ? 1U : 0U,
                static_cast<unsigned>(m_edm127.expectedMode[row]), m_edm127.expectedOriginPC[row], m_edm127.expectedOriginPC[0],
                static_cast<unsigned long long>(m_edm127.expectedOriginRevision[row]),
                static_cast<unsigned long long>(m_edm127.expectedOriginRevision[0]));
            return false;
        }
    // Rebuild every frozen literal with the existing common LINEAR6/ARC2
    // geometry. Whole-arc extrema, including omitted endpoint components,
    // define the envelope; an endpoint chord is not an arc travel proof.
    std::array<double, 6U> cumulative{}, low{}, high{}, maximumNativeReference{};
    std::array<double, 3U> lengths{};
    double nominalLength = 0.0;
    for (unsigned row = 0U; row < 3U; ++row)
    {
        const auto* authored = GetBaseProgramCache().TryGetLine(s.pc[row]);
        if (!authored)
        {
            reject("CACHE_PC", static_cast<int>(row), -1, -1);
            RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=CACHE row=%u sourcePC=%d\n",
                static_cast<unsigned long long>(s.session), row, s.pc[row]);
            return false;
        }
        NCParsedBlock geometry = authored->parsedBlock;
        if (geometry.has('B') != m_edm127.authoredInline[row]) return reject("RAW_B_PRESENCE", static_cast<int>(row), -1, -1);
        if (geometry.has('B'))
        {
            double word = 0.0;
            if (!E94Number(geometry.expression('B'), word) || word != 1.0) return reject("RAW_B1_LITERAL", static_cast<int>(row), -1, -1);
            geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
        }
        std::array<double, 6U> delta{}; double feed = 0.0;
        if (!E114Move(geometry, row, delta, feed, s.gapModeFixture)) return reject("ROW_LITERAL", static_cast<int>(row), -1, -1);
        if (delta != s.increment[row]) return reject("ROW_INCREMENT", static_cast<int>(row), -1, -1);
        if (feed != s.feed[row]) return reject("ROW_FEED", static_cast<int>(row), -1, -1);
        EDM112::Curve curve{}; EDM112::Bounds bounds{};
        if (row == 0U)
        {
            EDM109::Point start{}, end{}, scale{};
            for (unsigned axis = 0U; axis < 6U; ++axis)
            { scale[axis] = 1000000.0; end[axis] = delta[axis] * scale[axis]; }
            EDM109::Line line{};
            if (!EDM109::BuildLine(start, end, scale, line) || !EDM112::BuildLine(line, curve))
                return reject("LINE_CURVE", static_cast<int>(row), -1, -1);
        }
        else
        {
            std::array<double, 2U> center{};
            if (!E155ArcCenter(geometry, center, s.gapModeFixture) ||
                center[0] != (row == 1U ? 0.001 : -0.001) || center[1] != 0.0 ||
                !EDM114::BuildBoundedXYArc(static_cast<int>(s.authoredGCode[row]), delta, center, feed, curve))
                return reject("ARC_CURVE", static_cast<int>(row), -1, -1);
        }
        if (!EDM112::WholeCurveBounds(curve, bounds) || !bounds.valid ||
            !std::isfinite(curve.lengthMm) || curve.lengthMm <= 0.0)
            return reject("WHOLE_CURVE_BOUNDS", static_cast<int>(row), -1, -1);
        lengths[row] = curve.lengthMm; nominalLength += curve.lengthMm;
        for (unsigned axis = 0U; axis < 6U; ++axis)
        {
            const double rowLow = cumulative[axis] + bounds.minNative[axis];
            const double rowHigh = cumulative[axis] + bounds.maxNative[axis];
            if (!std::isfinite(rowLow) || !std::isfinite(rowHigh) || rowLow > rowHigh)
                return reject("ROW_NATIVE_BOUNDS", static_cast<int>(row), static_cast<int>(axis), -1);
            low[axis] = (std::min)(low[axis], rowLow); high[axis] = (std::max)(high[axis], rowHigh);
            // LINEAR6 length is XYZ millimetres. Rotary degrees retain
            // their own derivative; they must not inherit a .002 deg/s bound.
            const double referenceNative = row == 0U ?
                std::fabs(delta[axis]) / curve.lengthMm * EDM94::VisibleXYZMachiningReferenceMmSec :
                (axis < 2U ? EDM94::VisibleXYZMachiningReferenceMmSec : 0.0);
            const double referenceCeiling = axis < 3U ? EDM94::VisibleXYZMachiningReferenceMmSec :
                EDM94::VisibleSixRotaryReferenceNativeSec;
            if (!std::isfinite(referenceNative) || referenceNative < 0.0 || referenceNative > referenceCeiling)
                return reject("NATIVE_TANGENT_SPEED", static_cast<int>(row), static_cast<int>(axis), -1);
            maximumNativeReference[axis] = (std::max)(maximumNativeReference[axis], referenceNative);
            cumulative[axis] += delta[axis];
            if (!std::isfinite(cumulative[axis]) ||
                low[axis] < (axis < 3U ? -0.025 : -0.5) ||
                high[axis] > (axis < 3U ? 0.025 : 0.5))
            {
                reject("NOMINAL_ENVELOPE", static_cast<int>(row), static_cast<int>(axis), -1);
                RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=NOMINAL_ENVELOPE row=%u axis=%u cumulativeBits=%llu lowBits=%llu highBits=%llu limitBits=%llu\n",
                    static_cast<unsigned long long>(s.session), row, axis, bits(cumulative[axis]), bits(low[axis]), bits(high[axis]), bits(axis < 3U ? 0.025 : 0.5));
                return false;
            }
        }
    }
    const double reference = EDM130::FrozenGapReferenceVoltageV(r);
    const auto profile = EDM97::MakeProfile(s.gapTuning, r);
    const auto forward = EDMGapServo::Evaluate(profile, reference + 38.0, reference,
        EDM130::FrozenMachiningOverridePercent(r));
    // AUTO_WAVE's low phase is ref-20V with at most +1V noise. Budget the
    // slower -19V case; the initial 160 zero samples use the same short path.
    const auto backward = EDMGapServo::Evaluate(profile, reference - 19.0, reference,
        EDM130::FrozenMachiningOverridePercent(r));
    const double forwardSpeed = (std::min)(0.12, forward.limitedSpeedMmPerMin) / 60.0;
    const double backwardSpeed = (std::min)(0.03, -backward.limitedSpeedMmPerMin) / 60.0;
    if (!forward.valid || !backward.valid || !std::isfinite(nominalLength) || nominalLength <= 0.0 ||
        !std::isfinite(forwardSpeed) || forwardSpeed <= 0.0 ||
        !std::isfinite(backwardSpeed) || backwardSpeed <= 0.0)
    {
        reject("GAP_CURVE_SPEED", -1, -1, -1);
        RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=GAP_SPEED forwardValid=%u backwardValid=%u nominalLengthBits=%llu forwardBits=%llu backwardBits=%llu referenceBits=%llu\n",
            static_cast<unsigned long long>(s.session), forward.valid ? 1U : 0U, backward.valid ? 1U : 0U,
            bits(nominalLength), bits(forwardSpeed), bits(backwardSpeed), bits(reference));
        return false;
    }
    double normalSpeed = 0.0, acceleration = 0.0, maximumConfiguredTime = 0.2;
    double capPps[6]{}, windowPulse[6]{}, worstPiPps[6]{}, sparePps[6]{};
    double feedforwardPps[6]{}, availablePps[6]{};
    for (unsigned axisIndex = 0U; axisIndex < 6U; ++axisIndex)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(axisIndex));
        const double scale = axis.resolution_PPR / axis.finalLead;
        if (axis.axisType != (axisIndex < 3U ? AxisType::LINEAR : AxisType::ROTARY) || axis.isVirtualAxis)
            return reject("AXIS_NATIVE_KIND", -1, static_cast<int>(axisIndex), -1);
        if (!std::isfinite(axis.Pid_G00.Kd) || axis.Pid_G00.Kd < 0.0 ||
            !std::isfinite(axis.Pid_FLUSH.Kd) || axis.Pid_FLUSH.Kd < 0.0)
            return reject("AXIS_DERIVATIVE_GAIN", -1, static_cast<int>(axisIndex), -1);
        if (!axis.isExist || axis.axisIndex != static_cast<int>(axisIndex) ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 || !std::isfinite(scale) || scale <= 0.0 ||
            !std::isfinite(axis.currentCmdPos) || !std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.0 ||
            !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
            !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0)
        {
            const char* reason = !axis.isExist ? "AXIS_ABSENT" : axis.axisIndex != static_cast<int>(axisIndex) ? "AXIS_IDENTITY" :
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ? "AXIS_LEAD" :
                !std::isfinite(scale) || scale <= 0.0 ? "AXIS_PPU" : !std::isfinite(axis.currentCmdPos) ? "AXIS_COMMAND" :
                !std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.0 ? "AXIS_STOP_TIME" :
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ? "AXIS_ACCEL_TIME" : "AXIS_DECEL_TIME";
            reject(reason, -1, static_cast<int>(axisIndex), -1);
            RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=AXIS_CONFIG axis=%u exists=%u configuredAxis=%d leadBits=%llu pprBits=%llu ppuBits=%llu commandPulseBits=%llu stopBits=%llu accBits=%llu decBits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex, axis.isExist ? 1U : 0U, axis.axisIndex,
                bits(axis.finalLead), bits(axis.resolution_PPR), bits(scale), bits(axis.currentCmdPos),
                bits(axis.Stop_dec_time), bits(axis.G00_acc_time), bits(axis.G00_dec_time));
            return false;
        }
        const double accTime = axis.G00_acc_time < 0.001 ? 0.2 : axis.G00_acc_time;
        const double decTime = axis.G00_dec_time < 0.001 ? 0.2 : axis.G00_dec_time;
        maximumConfiguredTime = (std::max)(maximumConfiguredTime, axis.Stop_dec_time < 0.001 ? 0.2 : axis.Stop_dec_time);
        maximumConfiguredTime = (std::max)(maximumConfiguredTime, (std::max)(accTime, decTime));
        {
            // Every moving axis uses its own integer CSV cap and native
            // following window; the Z leg retains the complete PI headroom.
            if (!std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.inPositionWindow_Pulse) || axis.inPositionWindow_Pulse <= 0.0 ||
                !std::isfinite(axis.pid.MaxLag) || axis.pid.MaxLag <= 0.0)
            {
                const char* reason = !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ? "AXIS_VELOCITY_CAP" :
                    !std::isfinite(axis.inPositionWindow_Pulse) || axis.inPositionWindow_Pulse <= 0.0 ? "RAW_POSITION_WINDOW" : "RAW_MAX_LAG";
                reject(reason, -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=RAW_WINDOW axis=%u maxVelBits=%llu inPositionBits=%llu maxLagBits=%llu ppuBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(axis.maxVel_PPS), bits(axis.inPositionWindow_Pulse), bits(axis.pid.MaxLag), bits(scale));
                return false;
            }
            const double cap = std::floor((std::min)(axis.maxVel_PPS, scale * EDM94::NativeVelocityLimit(axisIndex)));
            const double window = (std::min)(axis.inPositionWindow_Pulse, (std::min)(axis.pid.MaxLag, scale * (axisIndex < 3U ? 0.001 : 0.005)));
            const double referenceSpeed = axisIndex == 2U ? EDM94::VisibleFlushReferenceMmSec :
                axisIndex < 3U ? EDM94::VisibleXYZMachiningReferenceMmSec : EDM94::VisibleSixRotaryReferenceNativeSec;
            // FIX2: normal low-feed XY keeps configured PI and the existing
            // per-tick following/output guards. The Z lift retains its complete
            // frozen-window headroom requirement below.
            if (axisIndex < 2U)
            {
                if (!EDM94::BoundedXYMachiningServoFits(scale, cap, window, axis.Pid_G00.Kp,
                    axis.Pid_G00.Ki, axis.Pid_G00.Kd, axis.Pid_G00.Kvff, axis.pid.MaxIntegral))
                {
                    reject("XY_CONFIGURED_SERVO_ADMISSION", -1, static_cast<int>(axisIndex), -1);
                    const double ff = referenceSpeed * scale * axis.Pid_G00.Kvff;
                    const double p = window * axis.Pid_G00.Kp, i = axis.pid.MaxIntegral * axis.Pid_G00.Ki;
                    const double reserve = 0.004 * scale, available = cap - reserve;
                    RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=XY_SERVO_CONFIG axis=%u ppuBits=%llu rawCapBits=%llu integerCapBits=%llu referenceBits=%llu rawInPositionBits=%llu rawMaxLagBits=%llu windowBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, bits(scale), bits(axis.maxVel_PPS), bits(cap), bits(referenceSpeed),
                        bits(axis.inPositionWindow_Pulse), bits(axis.pid.MaxLag), bits(window));
                    RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=XY_SERVO_GAINS axis=%u kpBits=%llu kiBits=%llu kdBits=%llu kvffBits=%llu maxIntegralBits=%llu values=CURRENT_NATIVE_CHECKED gainWrite=NONE\n",
                        static_cast<unsigned long long>(s.session), axisIndex, bits(axis.Pid_G00.Kp), bits(axis.Pid_G00.Ki), bits(axis.Pid_G00.Kd), bits(axis.Pid_G00.Kvff), bits(axis.pid.MaxIntegral));
                    RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=XY_SERVO_SUM axis=%u ffBits=%llu pBits=%llu iBits=%llu theoreticalSumBits=%llu reserveBits=%llu availableBits=%llu sumRole=OBSERVATION_ONLY admissionBound=FEEDFORWARD_ONLY\n",
                        static_cast<unsigned long long>(s.session), axisIndex, bits(ff), bits(p), bits(i), bits(ff + p + i), bits(reserve), bits(available));
                    return false;
                }
            }
            else if (axisIndex >= 3U)
            {
                if (!EDM94::BoundedRotaryMachiningServoFits(axisIndex, scale, cap, window,
                    axis.Pid_G00.Kp, axis.Pid_G00.Ki, axis.Pid_G00.Kd, axis.Pid_G00.Kvff, axis.pid.MaxIntegral))
                    return reject("ROTARY_CONFIGURED_SERVO_ADMISSION", -1, static_cast<int>(axisIndex), -1);
            }
            else if (!EDM94::VisibleServoHeadroomFits(scale, cap, window, axis.Pid_G00.Kp, axis.Pid_G00.Ki,
                axis.Pid_G00.Kvff, axis.pid.MaxIntegral, referenceSpeed))
            {
                reject("SERVO_HEADROOM", -1, static_cast<int>(axisIndex), -1);
                const double ff = referenceSpeed * scale * axis.Pid_G00.Kvff;
                const double p = window * axis.Pid_G00.Kp, i = axis.pid.MaxIntegral * axis.Pid_G00.Ki;
                const double reserve = 0.004 * scale, available = cap - reserve;
                RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=HEADROOM_CONFIG axis=%u ppuBits=%llu rawCapBits=%llu integerCapBits=%llu referenceBits=%llu rawInPositionBits=%llu rawMaxLagBits=%llu windowBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(scale), bits(axis.maxVel_PPS), bits(cap), bits(referenceSpeed),
                    bits(axis.inPositionWindow_Pulse), bits(axis.pid.MaxLag), bits(window));
                RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=HEADROOM_GAINS axis=%u kpBits=%llu kiBits=%llu kvffBits=%llu maxIntegralBits=%llu values=CURRENT_NATIVE_CHECKED gainWrite=NONE\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(axis.Pid_G00.Kp), bits(axis.Pid_G00.Ki), bits(axis.Pid_G00.Kvff), bits(axis.pid.MaxIntegral));
                RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=HEADROOM_SUM axis=%u ffBits=%llu pBits=%llu iBits=%llu requiredBits=%llu reserveBits=%llu availableBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, bits(ff), bits(p), bits(i), bits(ff + p + i), bits(reserve), bits(available));
                return false;
            }
            const bool flushHeadroom = axisIndex < 2U ?
                EDM94::BoundedXYMachiningServoFits(scale, cap, window, axis.Pid_FLUSH.Kp,
                    axis.Pid_FLUSH.Ki, axis.Pid_FLUSH.Kd, axis.Pid_FLUSH.Kvff, axis.pid.MaxIntegral) :
                axisIndex >= 3U ? EDM94::BoundedRotaryMachiningServoFits(axisIndex, scale, cap, window,
                    axis.Pid_FLUSH.Kp, axis.Pid_FLUSH.Ki, axis.Pid_FLUSH.Kd, axis.Pid_FLUSH.Kvff, axis.pid.MaxIntegral) :
                EDM94::VisibleServoHeadroomFits(scale, cap, window, axis.Pid_FLUSH.Kp,
                    axis.Pid_FLUSH.Ki, axis.Pid_FLUSH.Kvff, axis.pid.MaxIntegral, referenceSpeed);
            if (!flushHeadroom)
            {
                reject("FLUSH_SERVO_HEADROOM", -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM165] event=ADMISSION_DETAIL session=%llu part=FLUSH_SERVO_GAINS axis=%u profile=167 kpBits=%llu kiBits=%llu kdBits=%llu kvffBits=%llu values=CONFIGURED gainWrite=NONE\n",
                    static_cast<unsigned long long>(s.session), axisIndex,
                    bits(axis.Pid_FLUSH.Kp), bits(axis.Pid_FLUSH.Ki), bits(axis.Pid_FLUSH.Kd), bits(axis.Pid_FLUSH.Kvff));
                return false;
            }
            capPps[axisIndex] = cap; windowPulse[axisIndex] = window;
            feedforwardPps[axisIndex] = referenceSpeed * scale * axis.Pid_G00.Kvff;
            availablePps[axisIndex] = cap - 0.004 * scale;
            worstPiPps[axisIndex] = referenceSpeed * scale * axis.Pid_G00.Kvff +
                window * axis.Pid_G00.Kp + axis.pid.MaxIntegral * axis.Pid_G00.Ki;
            sparePps[axisIndex] = cap - worstPiPps[axisIndex];
            if (axisIndex == 2U)
            {
                normalSpeed = (std::min)(EDM94::VisibleFlushReferenceMmSec, cap / scale);
                acceleration = EDM128::ReservedVisibleAcceleration(normalSpeed,
                    (std::min)(cap / accTime, cap / decTime) / scale, 2.0);
                if (!std::isfinite(normalSpeed) || normalSpeed <= 0.0 ||
                    !std::isfinite(acceleration) || acceleration <= 0.0)
                {
                    reject("Z_LIFT_DYNAMICS", -1, static_cast<int>(axisIndex), -1);
                    RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=Z_DYNAMICS speedBits=%llu accelerationBits=%llu capBits=%llu ppuBits=%llu accTimeBits=%llu decTimeBits=%llu\n",
                        static_cast<unsigned long long>(s.session), bits(normalSpeed), bits(acceleration), bits(cap), bits(scale), bits(accTime), bits(decTime));
                    return false;
                }
            }
        }
        const double command = axis.currentCmdPos / scale;
        const double actual = axis.GetNominalActualPositionPulse() / scale;
        // E5 may pause only committed path geometry. Include a 2mm positive-Z
        // lift at every admitted position, for both command and actual origin.
        const double lift = axisIndex == 2U ? 2.0 : 0.0;
        // Rotary following is observed under the existing empty-load policy.
        // Reserve the full existing native guard excursion once around every
        // nominal angular point, for command and nominal actual origins.
        const double nativeMargin = axisIndex < 3U ? 0.0 : EDM94::NativeExcursionLimit(axisIndex);
        const double points[6] = {command, actual, command + low[axisIndex] - nativeMargin,
            command + high[axisIndex] + lift + nativeMargin,
            actual + low[axisIndex] - nativeMargin, actual + high[axisIndex] + lift + nativeMargin};
        const unsigned invalidLimitMask = CoordSys.GetInvalidSoftwareTravelLimitMask(axis);
        if (invalidLimitMask != 0U)
        {
            reject("COORD_LIMIT_CONFIG", -1, static_cast<int>(axisIndex), -1);
            RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=COORD_LIMIT_CONFIG axis=%u invalidMask=%u commandBits=%llu actualBits=%llu nominalLowBits=%llu nominalHighBits=%llu liftBits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex, invalidLimitMask, bits(command), bits(actual), bits(low[axisIndex]), bits(high[axisIndex]), bits(lift));
            RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=INVALID_LIMIT_RANGES axis=%u enabledMask=%u low1Bits=%llu high1Bits=%llu low2Bits=%llu high2Bits=%llu low3Bits=%llu high3Bits=%llu\n",
                static_cast<unsigned long long>(s.session), axisIndex,
                (axis.travelLimit1Enable ? 1U : 0U) | (axis.travelLimit2Enable ? 2U : 0U) | (axis.travelLimit3Enable ? 4U : 0U),
                bits(axis.travelLimit1Negative_unit), bits(axis.travelLimit1Positive_unit), bits(axis.travelLimit2Negative_unit),
                bits(axis.travelLimit2Positive_unit), bits(axis.travelLimit3Negative_unit), bits(axis.travelLimit3Positive_unit));
            return false;
        }
        const bool enabled[3] = {axis.travelLimit1Enable, axis.travelLimit2Enable, axis.travelLimit3Enable};
        const double lower[3] = {axis.travelLimit1Negative_unit, axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit};
        const double upper[3] = {axis.travelLimit1Positive_unit, axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit};
        const char* pointSources[6] = {"COMMAND_ORIGIN", "ACTUAL_ORIGIN", "COMMAND_PATH_LOW", "COMMAND_PATH_HIGH_LIFT", "ACTUAL_PATH_LOW", "ACTUAL_PATH_HIGH_LIFT"};
        for (unsigned pointIndex = 0U; pointIndex < 6U; ++pointIndex)
        {
            const double point = points[pointIndex];
            if (!std::isfinite(point) || !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, point))
            {
                reject(!std::isfinite(point) ? "TRAVEL_POINT_NONFINITE" : "COORD_TARGET_LIMIT", -1, static_cast<int>(axisIndex), -1);
                RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=TRAVEL_POINT axis=%u pointIndex=%u source=%s commandBits=%llu actualBits=%llu pointBits=%llu nominalLowBits=%llu nominalHighBits=%llu liftBits=%llu\n",
                    static_cast<unsigned long long>(s.session), axisIndex, pointIndex, pointSources[pointIndex], bits(command), bits(actual), bits(point), bits(low[axisIndex]), bits(high[axisIndex]), bits(lift));
                for (unsigned limit = 0U; limit < 3U; ++limit)
                    RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=COORD_TARGET_LIMIT axis=%u limit=%u enabled=%u lowBits=%llu highBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, limit, enabled[limit] ? 1U : 0U, bits(lower[limit]), bits(upper[limit]));
                return false;
            }
            for (unsigned limit = 0U; limit < 3U; ++limit)
                if (enabled[limit] && (!std::isfinite(lower[limit]) || !std::isfinite(upper[limit]) ||
                    lower[limit] >= upper[limit] || point < lower[limit] || point > upper[limit]))
                {
                    reject("ENABLED_TARGET_LIMIT", -1, static_cast<int>(axisIndex), static_cast<int>(limit));
                    RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=ENABLED_TARGET_LIMIT axis=%u limit=%u pointIndex=%u source=%s commandBits=%llu actualBits=%llu pointBits=%llu lowBits=%llu highBits=%llu\n",
                        static_cast<unsigned long long>(s.session), axisIndex, limit, pointIndex, pointSources[pointIndex], bits(command), bits(actual), bits(point), bits(lower[limit]), bits(upper[limit]));
                    return false;
                }
        }
    }
    const double approach = EDM130::FrozenApproachDistanceMm(r);
    const double finalSpeed = (std::min)(normalSpeed, EDM130::FrozenFinalApproachSpeedMmMin(r) / 60.0);
    if (!std::isfinite(approach) || approach < 0.0 || !std::isfinite(finalSpeed) || finalSpeed <= 0.0)
    {
        reject("FINAL_APPROACH", -1, 2, -1);
        RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=FINAL_APPROACH approachBits=%llu finalSpeedBits=%llu normalSpeedBits=%llu configuredFinalMmMinBits=%llu\n",
            static_cast<unsigned long long>(s.session), bits(approach), bits(finalSpeed), bits(normalSpeed), bits(EDM130::FrozenFinalApproachSpeedMmMin(r)));
        return false;
    }
    // Row index2's existing source can visit at most its current arc and the
    // immediately preceding arc. Charge both complete arcs in each direction,
    // plus all authored forward geometry and every group acceleration/stop.
    const double historyLength = lengths[1] + lengths[2];
    const double budget = 2.0 * EDM130::FrozenJumpHeightMm(r) / normalSpeed + 4.0 * normalSpeed / acceleration +
        2.0 * approach / finalSpeed + 3.0 * 0.1 + 10.0 * maximumConfiguredTime +
        nominalLength / forwardSpeed + historyLength / backwardSpeed + historyLength / forwardSpeed + 0.04 + 5.0;
    if (!std::isfinite(historyLength) || historyLength <= 0.0 || !std::isfinite(budget) || budget > 115.0)
    {
        reject("SESSION_BUDGET", -1, -1, -1);
        RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=BUDGET_LENGTH row0Bits=%llu row1Bits=%llu row2Bits=%llu nominalBits=%llu historyBits=%llu heightBits=%llu approachBits=%llu\n",
            static_cast<unsigned long long>(s.session), bits(lengths[0]), bits(lengths[1]), bits(lengths[2]), bits(nominalLength), bits(historyLength), bits(EDM130::FrozenJumpHeightMm(r)), bits(approach));
        RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=BUDGET_DYNAMICS forwardBits=%llu backwardBits=%llu finalBits=%llu liftSpeedBits=%llu accelerationBits=%llu maxConfiguredTimeBits=%llu\n",
            static_cast<unsigned long long>(s.session), bits(forwardSpeed), bits(backwardSpeed), bits(finalSpeed), bits(normalSpeed), bits(acceleration), bits(maximumConfiguredTime));
        RtPrintf("[EDM167] event=ADMISSION_DETAIL build=SIX_AXIS session=%llu part=BUDGET_TOTAL computedBits=%llu ceilingBits=%llu admissionCeilingSeconds=115 hardDeadlineSeconds=120\n",
            static_cast<unsigned long long>(s.session), bits(budget), bits(115.0));
        return false;
    }
    RtPrintf("[EDM167] event=PLAN_PREFLIGHT session=%llu rows=3 mask=63 source=SOFTWARE_WAVE heightMicrometres=2000 referenceMicrometresPerSec=65 hardDeadlineSeconds=120 admissionBudgetSeconds=115 nominalLengthBits=%llu historyLengthBits=%llu budgetSecondsBits=%llu accelerationBits=%llu discharge=0\n",
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(EDM130::CurveDoubleBits(nominalLength)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(historyLength)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(budget)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(acceleration)));
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& configured = m_motion.GetAxisContext(static_cast<int>(axis));
        RtPrintf("[EDM167] event=NATIVE_MOTION_PREFLIGHT session=%llu axis=%c tangentNativeSecBits=%llu referenceNativeSecBits=%llu hardwareSign=%d unit=%s gains=G00_AND_FLUSH discharge=0\n",
            static_cast<unsigned long long>(s.session), "XYZCUV"[axis], bits(maximumNativeReference[axis]),
            bits(axis == 2U ? EDM94::VisibleFlushReferenceMmSec : axis < 3U ? EDM94::VisibleXYZMachiningReferenceMmSec : EDM94::VisibleSixRotaryReferenceNativeSec),
            configured.isReverse != configured.Axis_Reverse ? -1 : 1, axis < 3U ? "MM" : "DEG");
        if (axis != 2U)
        {
            RtPrintf("[EDM167] event=MACHINING_SERVO_ADMISSION build=SIX_AXIS session=%llu axis=%c capPpsBits=%llu windowBits=%llu ffBits=%llu availableBits=%llu policy=CONFIGURED_PI_RUNTIME_GUARDED protection=POST_PI_CAP_AND_I_ROLLBACK gainWrite=NONE discharge=0\n",
                static_cast<unsigned long long>(s.session), "XYZCUV"[axis], bits(capPps[axis]), bits(windowPulse[axis]), bits(feedforwardPps[axis]), bits(availablePps[axis]));
            RtPrintf("[EDM167] event=MACHINING_SERVO_OBSERVATION build=SIX_AXIS session=%llu axis=%c theoreticalSumBits=%llu theoreticalCapDifferenceBits=%llu sumRole=OBSERVATION_ONLY admissionBound=FEEDFORWARD_ONLY\n",
                static_cast<unsigned long long>(s.session), "XYZCUV"[axis], bits(worstPiPps[axis]), bits(sparePps[axis]));
        }
        else
            RtPrintf("[EDM167] event=SERVO_HEADROOM session=%llu axis=%c capPpsBits=%llu windowBits=%llu worstPiPpsBits=%llu sparePpsBits=%llu proof=FULL_FROZEN_FOLLOWING_WINDOW gains=CONFIGURED discharge=0\n",
                static_cast<unsigned long long>(s.session), "XYZCUV"[axis],
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(capPps[axis])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(windowPulse[axis])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(worstPiPps[axis])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(sparePps[axis])));
    }
    return true;
}

bool NCManager::BindEDM130WorkTimeSameThread() noexcept
{
    const auto* historyArm=GetBaseProgramCache().TryGetLine(m_edm94.armPC);
    if(historyArm&&E237ArmSequence(historyArm->parsedBlock,m_edm94.gapHistoryArm))return BindEDM237RecipeSameThread();
    const auto* mixedArm=GetBaseProgramCache().TryGetLine(m_edm94.armPC);
    if(mixedArm&&E234ArmSequence(mixedArm->parsedBlock,m_edm94.gapHistoryArm))return BindEDM234RecipeSameThread();
    m_edm130 = EDM130NCState{};
    EDMRecipe::Summary summary{};
    if (!m_edm94.gapWorkTime || m_edm94.gapPhysicalAd != E149PhysicalExpected(m_edm94.gapHistoryArm) || !m_edmProcessReady ||
        !ReadEDMRecipeSummarySameThread(summary)) return false;
    const bool speed = E131SpeedArm(m_edm94.gapHistoryArm);
    const bool height = E132HeightArm(m_edm94.gapHistoryArm);
    const bool deep = E133DeepArm(m_edm94.gapHistoryArm);
    const bool machining = E134MachiningArm(m_edm94.gapHistoryArm);
    const bool reference = E135ReferenceArm(m_edm94.gapHistoryArm);
    const bool processCurve = E136CurveArm(m_edm94.gapHistoryArm);
    const bool flushStages = E137FlushStageArm(m_edm94.gapHistoryArm);
    if (!E130TimedArm(m_edm94.gapHistoryArm) ||
        (speed && m_edm94.gapFlushParameters.speedRatio != 0.5) ||
        (height && (m_edm94.gapFlushParameters.strokeRatio != 0.1 || m_edm94.gapFlushParameters.dwellUs != 100000ULL))) return false;
    const auto binding = flushStages ? EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(
        m_recipe, m_edmProcessProfile.machineProfileId, m_edmProcessProfile, m_edmProcessTuningGeneration,
        (EDM149::IsSixVisibleFlushTestArm(m_edm94.gapHistoryArm) ? EDM130::VisibleFlushProfile167 :
            EDM149::IsXYZVisibleFlushTestArm(m_edm94.gapHistoryArm) ? EDM130::VisibleFlushProfile159 :
            EDM149::IsVisibleFlushTestArm(m_edm94.gapHistoryArm) ? EDM130::VisibleFlushProfile158 : 0U)) :
        processCurve ? EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceAndCurve(
        m_recipe, m_edmProcessProfile.machineProfileId, m_edmProcessProfile, m_edmProcessTuningGeneration) :
        reference ? EDM130::BindWorkTimeHeightSpeedDeepMachiningAndReference(m_recipe, m_edmProcessProfile.machineProfileId) :
        machining ? EDM130::BindWorkTimeHeightSpeedDeepAndMachining(m_recipe, m_edmProcessProfile.machineProfileId) :
        deep ? EDM130::BindWorkTimeHeightSpeedAndDeepFlush(m_recipe, m_edmProcessProfile.machineProfileId) :
        height ? EDM130::BindWorkTimeHeightAndSpeed(m_recipe, m_edmProcessProfile.machineProfileId) :
        speed ? EDM130::BindWorkTimeAndSpeed(m_recipe, m_edmProcessProfile.machineProfileId) :
        EDM130::BindWorkTime(m_recipe, m_edmProcessProfile.machineProfileId);
    if (!binding.ready || EDM130::HasBoundSpeed(binding.recipe) != speed ||
        EDM130::HasBoundHeight(binding.recipe) != height || EDM130::HasBoundDeep(binding.recipe) != deep ||
        EDM130::HasBoundMachiningSpeed(binding.recipe) != machining ||
        EDM130::HasBoundReferenceVoltage(binding.recipe) != reference ||
        EDM130::HasBoundCurve(binding.recipe) != processCurve ||
        EDM130::HasBoundFlushStages(binding.recipe) != flushStages)
    {
        RtPrintf("[EDM130] event=RECIPE_BIND result=REJECT reason=%s field=%u armIssued=0 powerControl=NC_PLC_S92\n",
            EDM58::BindingErrorName(binding.error), static_cast<unsigned>(binding.failedFieldId));
        return false;
    }
    // No mutable catalog pointer crosses the RT boundary. An HMI edit after
    // this admission affects the next explicit G180, never an active clock.
    m_edm130.recipe = binding.recipe;
    m_edm130.recipe.diagnosticControlSource = static_cast<std::uint16_t>(E149ControlSource(m_edm94.gapHistoryArm));
    if (EDM149::IsBoundedProgramTestArm(m_edm94.gapHistoryArm))
    {
        m_edm130.recipe.planRows = static_cast<std::uint16_t>(m_edm94.moves);
        m_edm130.recipe.planMovementMask = static_cast<std::uint16_t>(E156NominalMovementMask(m_edm94));
    }
    if (EDM149::IsFlushHoldTestArm(m_edm94.gapHistoryArm) ||
        EDM149::IsManualVoltageTestArm(m_edm94.gapHistoryArm))
    {
        const auto& cache = GetBaseProgramCache();
        const auto* arm = cache.TryGetLine(m_edm94.armPC);
        EDM97::Tuning tuning{}; EDM107::FlushProfile profile{};
        if (cache.GetGeneration() != m_edm94.cache || !arm ||
            !E108ProfileOnlyNPQRSIJK(arm->parsedBlock, m_edm94.gapHistoryArm, tuning, profile) ||
            !EDM97::SameTuning(tuning, m_edm94.gapTuning) ||
            !EDM107::SameFlushProfile(profile, m_edm94.gapFlushParameters)) return false;
        m_edm130.recipe.holdRetreatMode = arm->parsedBlock.has('H') ? 1U : 0U;
        double selection = 0.0;
        if (arm->parsedBlock.has('L') && (!E94Number(arm->parsedBlock.expression('L'), selection) ||
            (selection != 1.0 && selection != 2.0 && selection != 3.0) ||
            !EDM130::ValidFlushHoldSelection(m_edm94.gapHistoryArm, static_cast<unsigned>(selection)))) return false;
        m_edm130.recipe.flushHoldTest = static_cast<std::uint16_t>(selection);
        if (m_edm130.recipe.flushHoldTest >= 2U)
        {
            // The immutable admitted modal plan must contain the requested
            // B checkpoint. No live selector or later NC edit may supply it.
            const unsigned target = EDM130::FlushHoldTargetMode(m_edm130.recipe.flushHoldTest);
            if (target >= 3U || !m_edm94.moves || m_edm94.moves > m_edm127.expectedMode.size()) return false;
            bool present = false;
            for (unsigned row = 0U; row < m_edm94.moves; ++row)
                present = present || static_cast<unsigned>(m_edm127.expectedMode[row]) == target;
            if (!present) return false;
        }
    }
    if (!E156BoundPlanCurrent(m_edm94, m_edm130.recipe) || !EDM130::ValidRecipe(m_edm130.recipe) ||
        (EDM149::IsTestArm(m_edm94.gapHistoryArm) &&
            (m_edm130.recipe.workTimeUs > 2000000ULL ||
                (!EDM130::IsOneShotCommissioning(m_edm130.recipe) && m_edm130.recipe.jumpHeightScaled > 1000ULL) ||
                m_edm130.recipe.deepFlushCycleInterval != 0U))) return false;
    if (!EDM97::ValidMotionTestCurve(m_edm94.gapTuning,m_edm130.recipe)) return false;
    if (speed) m_edm94.gapFlushParameters.speedRatio = EDM130::FrozenSpeedRatio(m_edm130.recipe);
    if (!EDM107::ValidFlushProfile(m_edm94.gapFlushParameters) ||
        !EDM130::MatchesFlushSpeed(m_edm130.recipe, m_edm94.gapFlushParameters.speedRatio) ||
        !EDM130::MatchesFlushHeight(m_edm130.recipe, m_edm94.gapFlushParameters.strokeRatio, m_edm94.gapFlushParameters.dwellUs)) return false;
    m_edm130.bound = true;
    const auto& r = m_edm130.recipe;
    m_edm169 = EDM169NCState{};
    if (EDM149::IsManualVoltageTestArm(m_edm94.gapHistoryArm))
    {
        m_edm169.session = m_edm94.session;
        m_edm169.voltageV = EDM130::FrozenGapReferenceVoltageV(r);
        if (!EDM169::ValidVoltageV(m_edm169.voltageV)) return false;
        MacroSys.SetVar('@', 990, m_edm169.voltageV);
        RtPrintf("[EDM169] event=MANUAL_VOLTAGE_ARMED session=%llu run=%llu cache=%llu epoch=%llu owner=%llu variable=@990 seededV=%.9g inputSequence=0 source=SOFTWARE_MANUAL initialWaitUs=30000000 sessionLimitUs=120000000 write=FRESH_HMI_EVENT_ONLY S92=0 S93=0\n",
            static_cast<unsigned long long>(m_edm94.session), static_cast<unsigned long long>(m_edm94.run),
            static_cast<unsigned long long>(m_edm94.cache), static_cast<unsigned long long>(m_edm94.armEpoch),
            static_cast<unsigned long long>(m_edm94.lease.generation), m_edm169.voltageV);
    }
    // Static configured banks only; transient servo state stays RT-owned.
    for (unsigned axisIndex = 0U; axisIndex < 6U; ++axisIndex)
    {
        const auto& gain = m_motion.GetAxisContext(static_cast<int>(axisIndex)).Pid_FLUSH;
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM165] event=FLUSH_PID_ARMED session=%llu axis=%c bank=FLUSH kp=%.9g ki=%.9g kd=%.9g kvff=%.9g configured=STATIC select=E5_OR_H1_EXCURSION actuator=EXISTING_SERVO_LOOP movementProof=NONE\n",
            static_cast<unsigned long long>(m_edm94.session), "XYZCUV"[axisIndex],
            gain.Kp, gain.Ki, gain.Kd, gain.Kvff);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM165] event=LOG_FORMAT_ERROR part=FLUSH_PID_ARMED\n");
    }
    if (r.holdRetreatMode && (EDM130::IsVisibleFlush159(r) || EDM130::IsVisibleFlush167(r)))
    {
        RtPrintf("[EDM166] event=VISIBLE_FLUSH_HOLD_ARMED session=%llu armP=%u H=1 L=%u heightMicrometres=2000 trigger=FIRST_E5_OUTBOUND_25PCT resume=MANUAL_FRESH_START return=SAME_FLUSH_THEN_ORIGINAL_E5_ANCHOR gapHold=CANCEL pausedJog=0 S92=0 S93=0\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, static_cast<unsigned>(r.flushHoldTest));
    }
    else if (r.holdRetreatMode)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM163] event=HOLD_PATH_ARMED session=%llu armP=%u H=%u source=%s E6mm=%.6f freeze=ADMISSION apply=NEXT_G180 confirmation=M00_FRESH_START_REQUIRED return=EXACT_PATH pausedJog=0 S92=0 S93=0\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm,
            static_cast<unsigned>(r.holdRetreatMode), EDM149::SourceName(r.diagnosticControlSource),
            EDM130::FrozenJumpHeightMm(r));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM163] event=LOG_FORMAT_ERROR part=HOLD_PATH_ARMED\n");
    }
    if (r.flushHoldTest)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[%s] event=AUTO_HOLD_ARMED session=%llu armP=%u H=1 L=%u targetB=%u trigger=%s confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START S92=0 S93=0\n",
            r.flushHoldTest >= 2U ? "EDM168" : "EDM165", static_cast<unsigned long long>(m_edm94.session),
            m_edm94.gapHistoryArm, static_cast<unsigned>(r.flushHoldTest), EDM130::FlushHoldTargetMode(r.flushHoldTest),
            r.flushHoldTest == 1U ? "FIRST_E5_OUTBOUND_25PCT" : r.flushHoldTest == 2U ?
                "FIRST_B1_E5_OUTBOUND_25PCT" : "FIRST_CROSSED_B2_E5_OUTBOUND_25PCT");
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM165] event=LOG_FORMAT_ERROR part=AUTO_HOLD_ARMED\n");
    }
    RtPrintf("[EDM130] event=RECIPE_FROZEN session=%llu table=%u E=%u field=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu E5us=%llu freeze=ADMISSION apply=NEXT_G180 clock=COMMITTED_GAP_ONLY powerControl=NC_PLC_S92\n",
        static_cast<unsigned long long>(m_edm94.session), r.tableId, static_cast<unsigned>(r.eCode),
        static_cast<unsigned>(r.fieldId), r.catalogRevision, r.tableRevision, r.profileId, r.definitionRevision,
        static_cast<unsigned long long>(r.generation), static_cast<unsigned long long>(r.workTimeUs));
    if (speed)
    {
        // RTX RtPrintf does not render these precision-qualified floats.
        // Format through the C runtime, then pass the bounded string to RTX.
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record), "[EDM131] event=RECIPE_FROZEN session=%llu armP=%u table=%u E=%u E5field=%u E7field=%u generation=%llu E5us=%llu E7pct=%.6f baseJpct=50 effectiveJpct=%.6f ratio=%.9f freeze=ADMISSION apply=NEXT_G180 speedScope=FLUSH_ONLY powerControl=NC_PLC_S92 build=FIX1\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, r.tableId,
            static_cast<unsigned>(r.eCode), static_cast<unsigned>(r.fieldId), static_cast<unsigned>(r.speedFieldId),
            static_cast<unsigned long long>(r.generation), static_cast<unsigned long long>(r.workTimeUs),
            static_cast<double>(r.jumpSpeedPercentScaled) / 1000000.0,
            m_edm94.gapFlushParameters.speedRatio * 100.0, m_edm94.gapFlushParameters.speedRatio);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM131] event=LOG_FORMAT_ERROR part=RECIPE_FROZEN build=FIX1\n");
    }
    if (height)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM132] event=RECIPE_FROZEN session=%llu armP=%u table=%u E=%u E5field=%u E6field=%u E7field=%u generation=%llu E5us=%llu E6mm=%.6f E7pct=%.6f baseJpct=50 effectiveJpct=%.6f freeze=ADMISSION apply=NEXT_G180 heightScope=FLUSH_ONLY geometry=ABSOLUTE_MM powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, r.tableId,
            static_cast<unsigned>(r.eCode), static_cast<unsigned>(r.fieldId), static_cast<unsigned>(r.heightFieldId),
            static_cast<unsigned>(r.speedFieldId), static_cast<unsigned long long>(r.generation),
            static_cast<unsigned long long>(r.workTimeUs), EDM130::FrozenJumpHeightMm(r),
            static_cast<double>(r.jumpSpeedPercentScaled) / 1000000.0,
            m_edm94.gapFlushParameters.speedRatio * 100.0);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM132] event=LOG_FORMAT_ERROR part=RECIPE_FROZEN\n");
    }
    if (deep)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM133] event=RECIPE_FROZEN session=%llu armP=%u table=%u E=%u E5field=%u E6field=%u E7field=%u E21field=%u E22field=%u generation=%llu E5us=%llu baseMm=%.6f E7pct=%.6f everyN=%u multiplier=%.6f freeze=ADMISSION apply=NEXT_G180 count=COMPLETED_GLOBAL powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, r.tableId,
            static_cast<unsigned>(r.eCode), static_cast<unsigned>(r.fieldId), static_cast<unsigned>(r.heightFieldId),
            static_cast<unsigned>(r.speedFieldId), static_cast<unsigned>(r.intervalFieldId), static_cast<unsigned>(r.multiplierFieldId),
            static_cast<unsigned long long>(r.generation), static_cast<unsigned long long>(r.workTimeUs),
            EDM130::FrozenJumpHeightMm(r), static_cast<double>(r.jumpSpeedPercentScaled) / 1000000.0,
            static_cast<unsigned>(r.deepFlushCycleInterval), EDM130::FrozenDeepMultiplier(r));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM133] event=LOG_FORMAT_ERROR part=RECIPE_FROZEN\n");
        RtPrintf("[EDM133-FIX1] event=CAUSE_CAPTURE_READY session=%llu sourceSite=1 orphanSites=2/3 observationOnly=1 controlChanges=0 powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session));
    }
    if (machining)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM134] event=RECIPE_FROZEN session=%llu armP=%u table=%u E=%u E8field=%u E8pct=%.6f generation=%llu freeze=ADMISSION apply=NEXT_G180 scope=GAP_CURVE_PRELIMIT E7scope=FLUSH_ONLY powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, r.tableId,
            static_cast<unsigned>(r.eCode), static_cast<unsigned>(r.machiningFieldId),
            EDM130::FrozenMachiningOverridePercent(r), static_cast<unsigned long long>(r.generation));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM134] event=LOG_FORMAT_ERROR part=RECIPE_FROZEN\n");
    }
    if (reference)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM135] event=RECIPE_FROZEN session=%llu armP=%u table=%u E=%u E9field=%u referenceV=%.6f E8pct=%.6f generation=%llu freeze=ADMISSION apply=NEXT_G180 scope=GAP_CURVE_AND_DECISION thresholdScope=FROZEN_CONFIG powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, r.tableId,
            static_cast<unsigned>(r.eCode), static_cast<unsigned>(r.referenceFieldId),
            EDM130::FrozenGapReferenceVoltageV(r), EDM130::FrozenMachiningOverridePercent(r),
            static_cast<unsigned long long>(r.generation));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM135] event=LOG_FORMAT_ERROR part=RECIPE_FROZEN\n");
    }
    if (processCurve)
    {
        // The NC owner captures this shape once. Later process API edits affect
        // only the next explicit G180; Q/R/S remain the legacy fixture syntax.
        const auto& c = r.curve;
        char record[481]{};
        int length = std::snprintf(record, sizeof(record),
            "[EDM136] event=CURVE_FROZEN part=IDENTITY session=%llu armP=%u processRev=%u machineProfile=%u tuningGen=%llu recipeGen=%llu shapeSource=PROCESS_PROFILE freeze=ADMISSION apply=NEXT_G180 QRS=LEGACY_FIXTURE_ONLY scale=0.001 feedCap=0.12 retreatCap=0.03 powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm,
            c.processRevision, c.machineProfileId, static_cast<unsigned long long>(c.tuningGeneration),
            static_cast<unsigned long long>(r.generation));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM136] event=LOG_FORMAT_ERROR part=CURVE_IDENTITY\n");
        length = std::snprintf(record, sizeof(record),
            "[EDM136] event=CURVE_FROZEN part=SHAPE session=%llu P=%.17g/%.17g/%.17g N=%.17g/%.17g/%.17g Pbreak=%.17g/%.17g Nbreak=%.17g/%.17g deadband=%.17g shapeSource=PROCESS_PROFILE boundaryUnit=ERROR_MAGNITUDE_V\n",
            static_cast<unsigned long long>(m_edm94.session), c.positiveGain[0], c.positiveGain[1], c.positiveGain[2],
            c.negativeGain[0], c.negativeGain[1], c.negativeGain[2], c.positiveBreakV[0], c.positiveBreakV[1],
            c.negativeBreakV[0], c.negativeBreakV[1], c.deadbandV);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM136] event=LOG_FORMAT_ERROR part=CURVE_SHAPE\n");
    }
    if (flushStages)
    {
        const auto& f = r.flushStages;
        char record[481]{};
        int length = std::snprintf(record, sizeof(record),
            "[EDM137] event=FLUSH_STAGES_FROZEN part=RETURN session=%llu armP=%u E11pct=%.6f E13mm=%.6f pumpStgMmMin=%.9g finalMmMin=%.9g freeze=ADMISSION apply=NEXT_G180 speedCap=E7_BOUNDED_RETURN distanceClamp=EVENT_HEIGHT powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm,
            static_cast<double>(f.finalSpeedPercentScaled) / 1000000.0, EDM130::FrozenApproachDistanceMm(r),
            f.finalApproachMmPerMin, EDM130::FrozenFinalApproachSpeedMmMin(r));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM137] event=LOG_FORMAT_ERROR part=FLUSH_STAGES_RETURN\n");
        length = std::snprintf(record, sizeof(record),
            "[EDM137] event=FLUSH_STAGES_FROZEN part=RETRACT session=%llu armP=%u E19pct=%.6f E20mm=%.6f pumpLowMmMin=%.9g initialMmMin=%.9g initialEnabled=%u E12=%u polarityS=93 E15=EXTERNAL_CIRCUIT speedCap=E7_BOUNDED_RETURN distanceClamp=EVENT_HEIGHT powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm,
            static_cast<double>(f.initialSpeedPercentScaled) / 1000000.0, EDM130::FrozenInitialSlowDistanceMm(r),
            f.initialSlowMmPerMin, EDM130::FrozenInitialSlowSpeedMmMin(r), f.initialDistanceScaled ? 1U : 0U,
            static_cast<unsigned>(EDM130::FrozenPolarity(r)));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM137] event=LOG_FORMAT_ERROR part=FLUSH_STAGES_RETRACT\n");
    }
    return true;
}

bool NCManager::IsEDM129SourceCurrentSameThread(const EDM126::MotionFeedback* history, EDM146::Failure* observation) const noexcept
{
    (void)EDM146::Fail(observation, EDM146::Gate::PhysicalProof);
    const auto& s = m_edm94;
    if (m_edm130.bound && EDM130::UsesSoftwareVoltage(m_edm130.recipe))
    {
        EDM126::MotionFeedback sample{};
        if (!history && !m_motion.ReadEDM126Feedback(sample)) return false;
        const auto& h = history ? *history : sample;
        return EDM149::IsTestArm(s.gapHistoryArm) && E149ControlSource(s.gapHistoryArm) == m_edm130.recipe.diagnosticControlSource &&
            !s.gapPhysicalAd && h.enabled && !h.physicalAd && !h.faulted && h.reason == EDM126::Reason::None &&
            h.session == s.session && h.armEpoch == s.armEpoch && h.ownerGeneration == s.lease.generation &&
            (s.complete || (h.active && h.liveEpoch == m_motion.GetCurrentExecutionEpoch())) &&
            h.sourceReady && h.sourceReason == EDM88::Reason::None && h.tick && h.nowUs &&
            h.sourceSequence && h.sourceCapturedUs && h.nowUs >= h.sourceCapturedUs &&
            h.nowUs - h.sourceCapturedUs <= EDM126::MaximumSourceAgeUs &&
            std::isfinite(h.voltage) && h.voltage >= 0.0 && h.voltage <= 300.0 &&
            h.controlSequence == h.sourceSequence && h.controlCapturedUs == h.sourceCapturedUs && h.controlVoltageV == h.voltage;
    }
    EDM129::MotionFeedback f{};
    if (!s.gapPhysicalAd || !s.gapHistoryServo || !s.gapHistoryMode ||
        (s.gapWorkTime ? !E130TimedArm(s.gapHistoryArm) : (s.gapHistoryArm != 170U && s.gapHistoryArm != 172U))) return false;
    if (!m_motion.ReadEDM129Feedback(f)) return EDM146::Fail(observation, EDM146::Gate::PhysicalRead);
    if (!f.enabled || f.faulted || !f.sourceReady ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        !f.tick || !f.nowUs || (!s.complete && (!f.active || f.liveEpoch != m_motion.GetCurrentExecutionEpoch())) ||
        !EDM129::ValidBinding(f.binding) ||
        (m_edm129.sourceBound ? !EDM129::SameBinding(f.binding, m_edm129.sourceBinding) :
            (s.next != 0U || s.finishPending || s.complete))) return false;
    const auto config = EDM129::SourceConfig(f.binding);
    const auto& e = f.source; const auto& sample = e.frame;
    const bool valid = EDM88::ValidPhysical(config) && e.session == s.session && e.ready && !e.faulted &&
        (s.complete || e.active) && e.reason == EDM129::SourceReason::None &&
        e.nowUs != 0ULL && e.nowUs <= f.nowUs && e.sourcePublication > f.binding.baselinePublication &&
        e.sourceCycle > f.binding.baselineCycle && e.freshFrames != 0ULL &&
        e.firstSourceCapturedUs > f.binding.admittedUs && e.firstSourceCapturedUs <= sample.capturedAtUs &&
        e.maximumAgeUs <= config.maxAgeUs && sample.valid &&
        sample.identity.kind == EDM88::SourceKind::Physical && EDM88::SameIdentity(sample.identity, config.identity) &&
        sample.sequence > f.binding.baselineSequence && sample.capturedAtUs > f.binding.admittedUs &&
        sample.capturedAtUs <= e.nowUs && sample.capturedAtUs <= f.nowUs &&
        f.nowUs - sample.capturedAtUs <= config.maxAgeUs &&
        std::isfinite(sample.voltageV) && sample.voltageV >= 0.0 && sample.voltageV <= 300.0;
    if (!valid) return false;
    const bool diagnostic = E139DiagnosticArm(s.gapHistoryArm);
    if (diagnostic)
    {
        const auto& d = f.diagnostic;
        if (!m_edm130.bound || m_edm130.recipe.diagnosticControlSource != 1U ||
            !EDM139::ValidEvidence(d, s.session, EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe)) ||
            d.tick != f.tick || d.controlFrame.capturedAtUs > f.nowUs ||
            f.nowUs - d.controlFrame.capturedAtUs > config.maxAgeUs) return false;
        if (history)
        {
            const auto& h = *history;
            if (!h.controlSequence || !h.controlCapturedUs ||
                (h.controlVoltageV != EDM139::HighVoltage(EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe)) &&
                    h.controlVoltageV != EDM139::LowVoltage(EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe))) ||
                h.nowUs < h.controlCapturedUs || h.nowUs - h.controlCapturedUs > config.maxAgeUs ||
                (h.controlSequence == d.controlFrame.sequence ?
                    (h.controlCapturedUs != d.controlFrame.capturedAtUs || h.controlVoltageV != d.controlFrame.voltageV) :
                    (h.controlSequence < d.controlFrame.sequence ? h.controlCapturedUs >= d.controlFrame.capturedAtUs :
                        h.controlCapturedUs <= d.controlFrame.capturedAtUs))) return false;
        }
    }
    else if (f.diagnostic.enabled) return false;
    if (!history) return true;
    // Observer channels may straddle one RT publication. Equal producer
    // sequences must have exactly the same payload; different sequences must
    // preserve producer time ordering and remain within the same age budget.
    const auto& h = *history;
    if (!h.physicalAd || h.session != f.session || h.armEpoch != f.armEpoch || h.liveEpoch != f.liveEpoch ||
        h.ownerGeneration != f.ownerGeneration || h.sourceSequence <= f.binding.baselineSequence ||
        h.sourceCapturedUs <= f.binding.admittedUs || h.nowUs < h.sourceCapturedUs ||
        h.nowUs - h.sourceCapturedUs > config.maxAgeUs) return false;
    const bool payload = h.sourceSequence == sample.sequence ?
        h.sourceCapturedUs == sample.capturedAtUs && h.voltage == sample.voltageV :
        h.sourceSequence < sample.sequence ? h.sourceCapturedUs < sample.capturedAtUs :
            h.sourceCapturedUs > sample.capturedAtUs;
    if (!payload) return EDM146::Fail(observation, EDM146::Gate::SourcePayload,
        h.tick, f.tick, h.nowUs, f.nowUs);
    // Own-source validity and payload ordering passed. A scheduler pause
    // between reads may leave only observer-to-observer age inconsistent.
    if ((h.nowUs > f.nowUs ? h.nowUs - f.nowUs : f.nowUs - h.nowUs) > config.maxAgeUs)
        return EDM146::Fail(observation, EDM146::Gate::SourceSkew, h.tick, f.tick, h.nowUs, f.nowUs);
    return true;
}

bool NCManager::IsEDM130CurrentSameThread(unsigned completed, EDM146::Failure* observation) const noexcept
{
    (void)EDM146::Fail(observation, EDM146::Gate::HistoryProof);
    const auto& s = m_edm94; const auto& c = m_edm116;
    EDM128::MotionFeedback f{}; EDM126::MotionFeedback h{};
    const unsigned fixture = E149HistoryFixture(s.gapHistoryArm);
    if (!s.gapWorkTime || !m_edm130.bound || !EDM130::ValidRecipe(m_edm130.recipe) ||
        EDM130::HasBoundSpeed(m_edm130.recipe) != E131SpeedArm(s.gapHistoryArm) ||
        EDM130::HasBoundHeight(m_edm130.recipe) != E132HeightArm(s.gapHistoryArm) ||
        EDM130::HasBoundDeep(m_edm130.recipe) != E133DeepArm(s.gapHistoryArm) ||
        EDM130::HasBoundMachiningSpeed(m_edm130.recipe) != E134MachiningArm(s.gapHistoryArm) ||
        EDM130::HasBoundReferenceVoltage(m_edm130.recipe) != E135ReferenceArm(s.gapHistoryArm) ||
        EDM130::HasBoundCurve(m_edm130.recipe) != E136CurveArm(s.gapHistoryArm) ||
        EDM130::HasBoundFlushStages(m_edm130.recipe) != E137FlushStageArm(s.gapHistoryArm) ||
        m_edm130.recipe.diagnosticControlSource != E149ControlSource(s.gapHistoryArm) ||
        (E132HeightArm(s.gapHistoryArm) && (s.gapFlushParameters.strokeRatio != 0.1 ||
            s.gapFlushParameters.dwellUs != 100000ULL)) ||
        !EDM130::MatchesFlushSpeed(m_edm130.recipe, s.gapFlushParameters.speedRatio) ||
        !EDM130::MatchesFlushHeight(m_edm130.recipe, s.gapFlushParameters.strokeRatio, s.gapFlushParameters.dwellUs) || s.gapPhysicalAd != E149PhysicalExpected(s.gapHistoryArm) || !s.gapHistoryMode || !s.gapHistoryServo || !s.gapPhysicalCurve || !s.gapSegmentProfile ||
        !s.gapGainProfile || s.gapBoundaryProfile || s.gapNoiseProfile || s.gapBurstProfile ||
        s.gapContinuousProfile || s.gapEnduranceProfile || s.gapPairedProfile || s.gapCommonPath ||
        s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
        s.gapTuning.positiveGain != 8.0 || s.gapTuning.negativeGain != 1.0 || s.gapTuning.deadbandV != 1.0 ||
        s.gapModeFlush || s.gapFlushProfile || s.gapFlushParameterized ||
        s.gapModeInterruption || s.gapModeInterruptionArm || s.gapCurveB2 || s.gapFlushInterruptArm ||
        !E130TimedArm(s.gapHistoryArm) ||
        s.gapModeFixture != E149ModalFixture(fixture) ||
        !EDM107::ValidFlushProfile(s.gapFlushParameters) || !s.latched || s.armPending || !s.everArmed ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || completed > s.moves ||
        c.consumed != completed || c.selected != completed + (c.pending ? 1U : 0U) || c.selected > s.moves ||
        c.acknowledgedRows > completed || completed - c.acknowledgedRows > 1U ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration()) return false;
    if (!m_motion.ReadEDM130Feedback(f)) return EDM146::Fail(observation, EDM146::Gate::TimingRead);
    if (!f.enabled || !f.workTimeFlush || !EDM130::SameRecipe(f.timerRecipe, m_edm130.recipe) ||
        !EDM130::MatchesFlushSpeed(f.timerRecipe, f.flushProfile.speedRatio) ||
        !EDM130::MatchesFlushHeight(f.timerRecipe, f.flushProfile.strokeRatio, f.flushProfile.dwellUs) || f.physicalAd != s.gapPhysicalAd || f.faulted || f.fixture != fixture || !EDM107::SameFlushProfile(f.flushProfile, s.gapFlushParameters) ||
        f.session != s.session || f.armEpoch != s.armEpoch ||
        f.ownerGeneration != s.lease.generation || f.authorizedRows != completed || f.completedRows != completed ||
        !f.tick || !f.nowUs || (!s.complete && f.liveEpoch != m_motion.GetCurrentExecutionEpoch())) return false;
    // Never retry over an invalid first snapshot merely because the next
    // publication was unavailable. Its own live state/identity passed above.
    if (!m_motion.ReadEDM126Feedback(h)) return EDM146::Fail(observation, EDM146::Gate::HistoryRead);
    if (!h.enabled || h.physicalAd != s.gapPhysicalAd || h.faulted || h.reason != EDM126::Reason::None || h.fixture != fixture ||
        h.session != f.session || h.armEpoch != f.armEpoch || h.liveEpoch != f.liveEpoch ||
        h.ownerGeneration != f.ownerGeneration || h.authorizedRows != completed || h.completedRows != completed ||
        h.rowActive || h.rowCompleted != (completed != 0U) ||
        !h.sourceReady || h.sourceReason != EDM88::Reason::None || !h.sourceSequence || !h.sourceCapturedUs ||
        h.nowUs < h.sourceCapturedUs || h.nowUs - h.sourceCapturedUs > EDM126::MaximumSourceAgeUs ||
        f.lastMappingTick != h.lastMappingTick || f.lastMappingUs != h.lastMappingUs) return false;
    if (E134MachiningArm(s.gapHistoryArm))
    {
        const bool diagnostic = E139DiagnosticArm(s.gapHistoryArm);
        const double controlVoltage = diagnostic ? h.controlVoltageV : h.voltage;
        const auto controlSequence = diagnostic ? h.controlSequence : h.sourceSequence;
        const auto controlCapturedUs = diagnostic ? h.controlCapturedUs : h.sourceCapturedUs;
        // Each curve is checked against its own producer sample. Hysteretic
        // short decisions can request zero; equal samples must agree exactly.
        const auto curve = EDMGapServo::Evaluate(EDM97::MakeProfile(s.gapTuning, m_edm130.recipe), controlVoltage,
            EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe), EDM130::FrozenMachiningOverridePercent(m_edm130.recipe));
        const double ratio = curve.valid ? curve.limitedSpeedMmPerMin / EDM97::MaximumFeedMmMin : 0.0;
        EDM96::Feedback gap{};
        (void)EDM146::Fail(observation, EDM146::Gate::GapProof);
        if (!curve.valid || !std::isfinite(h.targetRatio) || (h.targetRatio != 0.0 && h.targetRatio != ratio)) return false;
        // Validate the already observed curve before a different channel can
        // report a retryable read/skew condition.
        if (!IsEDM129SourceCurrentSameThread(&h, observation)) return false;
        (void)EDM146::Fail(observation, EDM146::Gate::GapProof);
        if (!m_motion.ReadEDM96Feedback(gap)) return EDM146::Fail(observation, EDM146::Gate::GapRead);
        if (!IsEDM97TuningCurrentSameThread(gap) ||
            gap.session != s.session || gap.ownerGeneration != s.lease.generation ||
            !gap.tick || !gap.nowUs || gap.epoch != h.liveEpoch ||
            (diagnostic && gap.voltageV != EDM139::HighVoltage(EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe)) &&
                gap.voltageV != EDM139::LowVoltage(EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe))) ||
            !gap.sourceSequence || (!diagnostic && s.gapPhysicalAd && gap.sourceSequence <= m_edm129.sourceBinding.baselineSequence) ||
            (s.gapPhysicalAd && gap.sourceCapturedUs <= m_edm129.sourceBinding.admittedUs) ||
            gap.nowUs < gap.sourceCapturedUs || gap.nowUs - gap.sourceCapturedUs > EDM126::MaximumSourceAgeUs) return false;
        const bool payloadMismatch = controlSequence == gap.sourceSequence ?
            (controlCapturedUs != gap.sourceCapturedUs || controlVoltage != gap.voltageV || h.targetRatio != gap.targetRatio) :
            (controlSequence < gap.sourceSequence ? controlCapturedUs >= gap.sourceCapturedUs :
                controlCapturedUs <= gap.sourceCapturedUs);
        if (payloadMismatch) return EDM146::Fail(observation, EDM146::Gate::GapPayload,
            h.tick, gap.tick, h.nowUs, gap.nowUs);
        if ((h.nowUs > gap.nowUs ? h.nowUs - gap.nowUs : gap.nowUs - h.nowUs) > EDM126::MaximumSourceAgeUs)
            return EDM146::Fail(observation, EDM146::Gate::GapSkew, h.tick, gap.tick, h.nowUs, gap.nowUs);
    }
    else if (!IsEDM129SourceCurrentSameThread(&h, observation)) return false;
    (void)EDM146::Fail(observation, EDM146::Gate::HistoryProof);
    if (E141MotionArm(s.gapHistoryArm) && completed == s.moves &&
        (m_edm141.plannedRows != s.moves || m_edm141.verifiedRows != s.moves || !m_edm141.expressionRows)) return false;
    if (E140AssignmentArm(s.gapHistoryArm) && completed == s.moves &&
        (!m_edm140.sealed || !m_edm140.count || m_edm140.count > EDM140::AssignmentCapacity ||
            m_edm140.next != m_edm140.count || m_edm140.session != s.session ||
            m_edm140.run != s.run || m_edm140.cache != s.cache || m_edm140.armEpoch != s.armEpoch ||
            m_edm140.ownerGeneration != s.lease.generation || !m_edm140.lastCommit)) return false;
    if (E139DiagnosticArm(s.gapHistoryArm) && completed == s.moves)
    {
        EDM129::MotionFeedback source{};
        if (!m_motion.ReadEDM129Feedback(source) || !source.enabled || source.faulted || !source.sourceReady ||
            source.session != s.session ||
            source.armEpoch != s.armEpoch || source.ownerGeneration != s.lease.generation ||
            !EDM139::ValidEvidence(source.diagnostic, s.session, EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe))) return false;
        const auto& d = source.diagnostic;
        if (!d.rowValid || d.row + 1U != completed || !d.observedCommitTick ||
            d.observedCommitTick > d.tick || d.tick != source.tick ||
            d.controlFrame.capturedAtUs > source.nowUs ||
            source.nowUs - d.controlFrame.capturedAtUs > EDM126::MaximumSourceAgeUs ||
            d.startedMask != EDM139::SelectedRowsMask || d.restoredMask != EDM139::SelectedRowsMask ||
            d.completedMask != EDM139::SelectedRowsMask || d.backwardMask != EDM139::SelectedRowsMask ||
            d.forwardMask != EDM139::SelectedRowsMask || d.backwardCrossings != 4U || d.forwardCrossings != 4U ||
            d.retreatAxisMask != 63U || d.returnAxisMask != 63U) return false;
    }
    const auto& e = f.coordinator; const auto& t = h.traversal;
    const unsigned requiredRetreats = EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) ? EDM149::SelectedRetreatCount(s.moves) : 4U;
    if (completed == s.moves && EDM130::UsesSoftwareVoltage(m_edm130.recipe) &&
        !EDM149::IsManualVoltageTestArm(s.gapHistoryArm) &&
        (t.backwardCrossings < requiredRetreats || t.forwardCrossings < requiredRetreats)) return false;
    if (!EDM128::ValidSpeedProof(e, completed ? m_edm130.recipe.flushStages : EDM130::FlushStages{})) return false;
    // No height proof exists before a row is authorized. Legacy profiles must
    // retain zero E6 evidence; EDM132 rows bind proof to the frozen actual E6.
    if (!EDM128::ValidHeightProof(e, completed ? EDM130::FrozenJumpHeightMm(m_edm130.recipe) : 0.0,
        completed ? m_edm130.recipe.deepFlushCycleInterval : 0U,
        completed ? EDM130::FrozenDeepMultiplier(m_edm130.recipe) : 0.0,
        completed && EDM130::HasBoundDeep(m_edm130.recipe))) return false;
    const auto& w = e.work;
    if (E133DeepArm(s.gapHistoryArm) && e.eventIndex &&
        (e.effectiveJumpHeightMm != EDM130::EffectiveJumpHeightMm(m_edm130.recipe, e.eventIndex) ||
            e.deepCycle != EDM130::IsDeepFlushEvent(m_edm130.recipe, e.eventIndex))) return false;
    const std::uint64_t events = w.completedEvents;
    std::uint64_t modeEventTotal = 0ULL;
    for (unsigned mode = 0U; mode < EDM116::ModeCount; ++mode) modeEventTotal += e.modeEvents[mode];
    if (EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) && completed == s.moves &&
        (w.triggeredEvents != 1ULL || events != 1ULL || e.eventCount != 1U ||
            e.modeEvents[0] != 0U || e.modeEvents[1] != 1U || e.modeEvents[2] != 0U ||
            !e.exactReturn || !e.rowEventComplete)) return false;
    // Earlier row anchors correctly reset at BeginRow. The retained timer and
    // coordinator counters prove the completed first-row excursion cumulatively.
    if ((EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) || EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm)) && completed == s.moves &&
        (w.triggeredEvents != 1ULL || events != 1ULL || e.eventCount != 1U || e.qualifiedDwells != 3ULL ||
            e.modeEvents[0] != 0U || e.modeEvents[1] != 1U || e.modeEvents[2] != 0U)) return false;
    if (!EDM130::ValidWorkEvidence(w) || (completed ? w.workTimeUs != m_edm130.recipe.workTimeUs : EDM130::IsTimed(w)) ||
        e.phase != EDM128::Phase::Gap || e.stableCycles || e.stableStartUs || e.faulted || e.stopped || e.detached ||
        (completed == s.moves && events == 0ULL &&
            !(EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) && EDM130::HasBoundProgramPlan(m_edm130.recipe) &&
                w.totalWorkUs < w.workTimeUs && !w.triggeredEvents)) || w.triggeredEvents != events || e.eventCount != events || e.qualifiedDwells != 3ULL * events ||
        modeEventTotal != events ||
        t.faulted || t.stopRequested || t.stopped || t.reason != EDM126::Reason::None ||
        t.historyDepth || t.backwardCrossings != t.forwardCrossings ||
        (!E132HeightArm(s.gapHistoryArm) && t.backwardCrossings < e.modeEvents[2]) ||
        t.maximumHistoryDepth > (completed ? completed - 1U : 0U) ||
        (t.backwardCrossings && !t.maximumHistoryDepth) || (!t.backwardCrossings && t.maximumHistoryDepth) ||
        e.liveAnchor != (completed != 0U)) return false;
    // Retain independent row-boundary totals. Only this authored row's modal B
    // may account for newly completed events; history visits never replay B.
    const bool newRow = c.acknowledgedRows < completed;
    const auto& prior = m_edm130.retainedWork;
    if (w.totalWorkUs < prior.totalWorkUs || w.completedEvents < prior.completedEvents ||
        w.triggeredEvents < prior.triggeredEvents || w.lastTriggerTick < prior.lastTriggerTick ||
        w.lastTriggerUs < prior.lastTriggerUs || w.lastCompleteTick < prior.lastCompleteTick ||
        w.lastCompleteUs < prior.lastCompleteUs || (!newRow && !EDM130::SameWorkEvidence(w, prior))) return false;
    const std::uint64_t added = events - prior.completedEvents;
    for (unsigned mode = 0U; mode < EDM116::ModeCount; ++mode)
    {
        const unsigned rowMode = newRow ? static_cast<unsigned>(m_edm127.expectedMode[completed - 1U]) : EDM116::ModeCount;
        const std::uint64_t expected = m_edm130.retainedModeEvents[mode] + (mode == rowMode ? added : 0ULL);
        if (e.modeEvents[mode] != expected) return false;
    }
    for (unsigned row = 0U; row < c.selected; ++row)
    {
        const auto& policy = c.policies[row];
        if (!EDM116::ValidPolicy(policy) || policy.fixture != s.gapModeFixture ||
            policy.mode != m_edm127.expectedMode[row] || policy.modeOriginPC != m_edm127.expectedOriginPC[row] ||
            policy.modeOriginRevision != m_edm127.expectedOriginRevision[row] || policy.selectorPC != s.pc[row] ||
            policy.revision != row + 1ULL || policy.selectorDispatch <= s.armDispatch ||
            !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
            (row && policy.selectorDispatch <= c.policies[row - 1U].selectorDispatch)) return false;
    }
    if (!completed)
        return !f.lastMappingTick && !f.lastMappingUs && !e.anchorCaptured && !e.exactReturn &&
            !e.rowEventComplete && EDM116::SamePolicy(f.rowPolicy, EDM116::Policy{}) &&
            EDM116::SamePolicy(e.policy, EDM116::Policy{}) &&
            EDM126::SameSource(h.authorizedSource, EDM126::Source{});
    const unsigned step = completed - 1U;
    const auto& policy = c.policies[step]; const auto& source = h.authorizedSource;
    if (!EDM126::ValidSource(source) || source.ordinal != step || source.sourcePC != s.pc[step] ||
        source.scope.session != s.session || source.scope.run != s.run || source.scope.cache != s.cache ||
        source.scope.ownerGeneration != s.lease.generation || source.dispatch != c.issuedDispatch[step] ||
        source.epoch != c.issuedEpoch[step] || !EDM116::SamePolicy(f.rowPolicy, policy) ||
        !EDM116::SamePolicy(e.policy, policy) || !f.lastMappingTick || !f.lastMappingUs ||
        f.lastMappingTick > f.tick || f.lastMappingUs > f.nowUs ||
        !EDM126::SameSource(t.authorizedSource, source) || !EDM126::SameSource(t.cursorSource, source) ||
        !t.completeReady || !t.atCurrentEnd || t.localMm != t.lengthMm || t.velocityMmSec != 0.0 ||
        t.tick != f.lastMappingTick || t.nowUs != f.lastMappingUs ||
        t.completedRows != step || t.historyCount != step) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis) if (t.velocityPps[axis] != 0.0) return false;
    for (unsigned row = 0U; row < c.acknowledgedRows; ++row)
    {
        if (!EDM116::SamePolicy(c.completedPolicies[row], c.policies[row]) ||
            !EDM116::PolicyMatchesSource(c.completedPolicies[row], c.completedSources[row]) ||
            c.completedSources[row].sourcePC != s.pc[row] || c.completedSources[row].step != row ||
            c.completedSources[row].dispatch != c.issuedDispatch[row] ||
            c.completedSources[row].epoch != c.issuedEpoch[row] ||
            !EDM109::SameScope(c.completedSources[row].scope, source.scope) ||
            (row && !EDM116::PolicyFollowsCompleted(c.completedPolicies[row], c.completedPolicies[row - 1U],
                c.completedSources[row - 1U]))) return false;
    }
    // The exact returned anchor is separate from the canonical NC row endpoint.
    // The coordinator must finish its excursion before the live row can complete.
    const bool event = newRow ? added != 0ULL : m_edm130.retainedRowEvent;
    if (e.rowEventComplete != event || e.anchorCaptured != event || e.exactReturn != event) return false;
    if (event && (!EDM126::SameSource(e.anchor.authorizedSource, source) ||
        e.anchor.historyDepth > step || !e.anchor.active || e.anchor.faulted || e.anchor.stopRequested ||
        e.anchor.stopped || e.anchor.awaitingCurrent || e.anchor.awaitingSeam || e.anchor.reason != EDM126::Reason::None ||
        e.anchor.velocityMmSec != 0.0 || e.anchor.accelerationMmSec2 != 0.0 ||
        (e.anchor.completeReady && (e.anchor.historyDepth != 0U || !e.anchor.atCurrentEnd ||
            e.anchor.localMm != e.anchor.lengthMm || !EDM126::SameSource(e.anchor.cursorSource, source))) ||
        !EDM126::ValidSource(e.anchor.cursorSource) ||
        e.anchor.cursorSource.ordinal > step || !EDM128::Zero(e.anchor) ||
        !e.anchor.tick || !e.anchor.nowUs || e.anchor.tick >= f.lastMappingTick || e.anchor.nowUs >= f.lastMappingUs ||
        !std::isfinite(e.anchor.localMm) || !std::isfinite(e.anchor.lengthMm) ||
        e.anchor.lengthMm <= 0.0 || e.anchor.localMm < 0.0 || e.anchor.localMm > e.anchor.lengthMm)) return false;
    if (event)
    {
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (!std::isfinite(e.anchor.positionPulse[axis]) || e.anchor.velocityPps[axis] != 0.0 ||
                e.anchor.accelerationPps2[axis] != 0.0) return false;
        const auto& anchor = e.anchor.cursorSource;
        if (!EDM109::SameScope(anchor.scope, source.scope) ||
            e.anchor.historyDepth != step - static_cast<unsigned>(anchor.ordinal)) return false;
        if (anchor.ordinal == step)
        { if (!EDM126::SameSource(anchor, source)) return false; }
        else
        {
            const auto& old = c.completedSources[static_cast<unsigned>(anchor.ordinal)];
            if (!EDM114::ValidSource(old) || !EDM109::SameScope(anchor.scope, old.scope) ||
                anchor.sourcePC != old.sourcePC || anchor.dispatch != old.dispatch || anchor.epoch != old.epoch ||
                anchor.segment != old.segment || anchor.ordinal != old.step) return false;
        }
    }
    if (event && E132HeightArm(s.gapHistoryArm) &&
        !E132GeometryProof(s, GetBaseProgramCache(), e, completed, E141MotionArm(s.gapHistoryArm) ? &m_edm141.rows : nullptr)) return false;
    if (event && E132HeightArm(s.gapHistoryArm) && policy.mode == EDM116::Mode::B2)
    {
        // The B2 target must name admitted geometry in this exact retained NC
        // history. The shared proof separately checks the frozen metric H.
        const auto& target = e.targetSource;
        if (!e.targetCaptured || !e.topCaptured || !e.exactReturn || !EDM126::ValidSource(target) ||
            !EDM109::SameScope(target.scope, source.scope) || target.ordinal > e.anchor.cursorSource.ordinal ||
            target.ordinal > step || !std::isfinite(e.targetLocalMm) || e.targetLocalMm < 0.0) return false;
        if (target.ordinal == step)
        { if (!EDM126::SameSource(target, source)) return false; }
        else
        {
            const auto& old = c.completedSources[static_cast<unsigned>(target.ordinal)];
            if (!EDM114::ValidSource(old) || !EDM109::SameScope(target.scope, old.scope) ||
                target.sourcePC != old.sourcePC || target.dispatch != old.dispatch || target.epoch != old.epoch ||
                target.segment != old.segment || target.ordinal != old.step) return false;
        }
        if (policy.mode == EDM116::Mode::B2)
        {
            const double distance = e.anchor.stationMm - e.targetStationMm;
            const double heightMm = E133DeepArm(s.gapHistoryArm) ? e.effectiveJumpHeightMm : EDM130::FrozenJumpHeightMm(m_edm130.recipe);
            if (!std::isfinite(distance) || std::fabs(distance - heightMm) > EDM126::Roundoff(e.anchor.stationMm)) return false;
        }
    }
    if (event && (!w.lastCompleteTick || w.lastTriggerTick >= e.anchor.tick ||
        e.anchor.tick >= w.lastCompleteTick || w.lastTriggerUs >= e.anchor.nowUs ||
        e.anchor.nowUs >= w.lastCompleteUs || w.lastCompleteTick > f.lastMappingTick ||
        w.lastCompleteUs > f.lastMappingUs || w.lastTriggerElapsedUs != m_edm130.recipe.workTimeUs)) return false;
    return true;
}

bool NCManager::IsEDM129CurrentSameThread(unsigned completed) const noexcept
{
    if (m_edm94.gapWorkTime) return IsEDM130CurrentSameThread(completed);
    const auto& s = m_edm94; const auto& c = m_edm116;
    EDM128::MotionFeedback f{}; EDM126::MotionFeedback h{};
    const unsigned fixture = s.gapHistoryArm == 170U ? 0U : 1U;
    if (!s.gapPhysicalAd || !s.gapHistoryMode || !s.gapHistoryServo || !s.gapPhysicalCurve || !s.gapSegmentProfile ||
        !s.gapGainProfile || s.gapBoundaryProfile || s.gapNoiseProfile || s.gapBurstProfile ||
        s.gapContinuousProfile || s.gapEnduranceProfile || s.gapPairedProfile || s.gapCommonPath ||
        s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
        s.gapTuning.positiveGain != 8.0 || s.gapTuning.negativeGain != 1.0 || s.gapTuning.deadbandV != 1.0 ||
        s.gapModeFlush || s.gapFlushProfile || s.gapFlushParameterized ||
        s.gapModeInterruption || s.gapModeInterruptionArm || s.gapCurveB2 || s.gapFlushInterruptArm ||
        (s.gapWorkTime ? !E130TimedArm(s.gapHistoryArm) : (s.gapHistoryArm != 170U && s.gapHistoryArm != 172U)) ||
        s.gapModeFixture != E149ModalFixture(fixture) ||
        !EDM107::ValidFlushProfile(s.gapFlushParameters) || !s.latched || s.armPending || !s.everArmed ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || completed > s.moves ||
        c.consumed != completed || c.selected != completed + (c.pending ? 1U : 0U) || c.selected > s.moves ||
        c.acknowledgedRows > completed || completed - c.acknowledgedRows > 1U ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.ReadEDM128Feedback(f) || !m_motion.ReadEDM126Feedback(h) ||
        !f.enabled || f.workTimeFlush || !EDM130::EmptyRecipe(f.timerRecipe) ||
        !EDM130::SameWorkEvidence(f.coordinator.work, EDM130::WorkEvidence{}) || !f.physicalAd || f.faulted || f.fixture != fixture || !EDM107::SameFlushProfile(f.flushProfile, s.gapFlushParameters) ||
        f.session != s.session || f.armEpoch != s.armEpoch ||
        f.ownerGeneration != s.lease.generation || f.authorizedRows != completed || f.completedRows != completed ||
        !f.tick || !f.nowUs || (!s.complete && f.liveEpoch != m_motion.GetCurrentExecutionEpoch()) ||
        !h.enabled || !h.physicalAd || h.faulted || h.reason != EDM126::Reason::None || h.fixture != fixture ||
        h.session != f.session || h.armEpoch != f.armEpoch || h.liveEpoch != f.liveEpoch ||
        h.ownerGeneration != f.ownerGeneration || h.authorizedRows != completed || h.completedRows != completed ||
        h.rowActive || h.rowCompleted != (completed != 0U) ||
        !h.sourceReady || h.sourceReason != EDM88::Reason::None || !h.sourceSequence || !h.sourceCapturedUs ||
        h.nowUs < h.sourceCapturedUs || h.nowUs - h.sourceCapturedUs > EDM126::MaximumSourceAgeUs ||
        f.lastMappingTick != h.lastMappingTick || f.lastMappingUs != h.lastMappingUs) return false;
    if (!IsEDM129SourceCurrentSameThread(&h)) return false;
    const auto& e = f.coordinator; const auto& t = h.traversal;
    const unsigned events = (completed + 1U) / 2U;
    if (e.phase != EDM128::Phase::Gap || e.stableCycles || e.stableStartUs || e.faulted || e.stopped || e.detached || e.eventCount != events || e.qualifiedDwells != 3U * events ||
        t.faulted || t.stopRequested || t.stopped || t.reason != EDM126::Reason::None ||
        t.historyDepth || t.backwardCrossings != t.forwardCrossings ||
        t.backwardCrossings < (completed > 2U ? 1ULL : 0ULL) + (completed > 4U ? 1ULL : 0ULL) ||
        t.maximumHistoryDepth > (completed ? completed - 1U : 0U) ||
        (t.backwardCrossings && !t.maximumHistoryDepth) ||
        (!t.backwardCrossings && t.maximumHistoryDepth) || e.liveAnchor != (completed != 0U)) return false;
    std::array<unsigned, EDM116::ModeCount> expectedModes{};
    for (unsigned row = 0U; row < completed; row += 2U)
    {
        if (!EDM116::ValidMode(m_edm127.expectedMode[row])) return false;
        ++expectedModes[static_cast<unsigned>(m_edm127.expectedMode[row])];
    }
    for (unsigned mode = 0U; mode < EDM116::ModeCount; ++mode)
        if (e.modeEvents[mode] != expectedModes[mode]) return false;
    for (unsigned row = 0U; row < c.selected; ++row)
    {
        const auto& policy = c.policies[row];
        if (!EDM116::ValidPolicy(policy) || policy.fixture != s.gapModeFixture ||
            policy.mode != m_edm127.expectedMode[row] || policy.modeOriginPC != m_edm127.expectedOriginPC[row] ||
            policy.modeOriginRevision != m_edm127.expectedOriginRevision[row] || policy.selectorPC != s.pc[row] ||
            policy.revision != row + 1ULL || policy.selectorDispatch <= s.armDispatch ||
            !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
            (row && policy.selectorDispatch <= c.policies[row - 1U].selectorDispatch)) return false;
    }
    if (!completed)
        return !f.lastMappingTick && !f.lastMappingUs && !e.anchorCaptured && !e.exactReturn &&
            !e.rowEventComplete && EDM116::SamePolicy(f.rowPolicy, EDM116::Policy{}) &&
            EDM116::SamePolicy(e.policy, EDM116::Policy{}) &&
            EDM126::SameSource(h.authorizedSource, EDM126::Source{});
    const unsigned step = completed - 1U;
    const auto& policy = c.policies[step]; const auto& source = h.authorizedSource;
    if (!EDM126::ValidSource(source) || source.ordinal != step || source.sourcePC != s.pc[step] ||
        source.scope.session != s.session || source.scope.run != s.run || source.scope.cache != s.cache ||
        source.scope.ownerGeneration != s.lease.generation || source.dispatch != c.issuedDispatch[step] ||
        source.epoch != c.issuedEpoch[step] || !EDM116::SamePolicy(f.rowPolicy, policy) ||
        !EDM116::SamePolicy(e.policy, policy) || !f.lastMappingTick || !f.lastMappingUs ||
        f.lastMappingTick > f.tick || f.lastMappingUs > f.nowUs ||
        !EDM126::SameSource(t.authorizedSource, source) || !EDM126::SameSource(t.cursorSource, source) ||
        !t.completeReady || !t.atCurrentEnd || t.localMm != t.lengthMm || t.velocityMmSec != 0.0 ||
        t.tick != f.lastMappingTick || t.nowUs != f.lastMappingUs ||
        t.completedRows != step || t.historyCount != step) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis) if (t.velocityPps[axis] != 0.0) return false;
    for (unsigned row = 0U; row < c.acknowledgedRows; ++row)
    {
        if (!EDM116::SamePolicy(c.completedPolicies[row], c.policies[row]) ||
            !EDM116::PolicyMatchesSource(c.completedPolicies[row], c.completedSources[row]) ||
            c.completedSources[row].sourcePC != s.pc[row] || c.completedSources[row].step != row ||
            c.completedSources[row].dispatch != c.issuedDispatch[row] ||
            c.completedSources[row].epoch != c.issuedEpoch[row] ||
            !EDM109::SameScope(c.completedSources[row].scope, source.scope) ||
            (row && !EDM116::PolicyFollowsCompleted(c.completedPolicies[row], c.completedPolicies[row - 1U],
                c.completedSources[row - 1U]))) return false;
    }
    // The exact returned anchor is separate from the canonical NC row endpoint.
    // The coordinator must finish its excursion before the live row can complete.
    const bool event = (step & 1U) == 0U;
    if (e.rowEventComplete != event || e.anchorCaptured != event || e.exactReturn != event) return false;
    if (event && (!EDM126::SameSource(e.anchor.authorizedSource, source) ||
        e.anchor.historyDepth != 0U || e.anchor.velocityMmSec != 0.0 ||
        !EDM126::ValidSource(e.anchor.cursorSource) ||
        e.anchor.cursorSource.ordinal != step || !EDM128::Zero(e.anchor) ||
        !e.anchor.tick || !e.anchor.nowUs || e.anchor.tick >= f.lastMappingTick || e.anchor.nowUs >= f.lastMappingUs ||
        !std::isfinite(e.anchor.localMm) || !std::isfinite(e.anchor.lengthMm) ||
        e.anchor.localMm <= 0.0 || e.anchor.localMm >= e.anchor.lengthMm)) return false;
    if (event && !EDM126::SameSource(e.anchor.cursorSource, source)) return false;
    return true;
}

namespace
{
    void E147OperatorCue(const char* action, const EDM147::Snapshot& e,
        unsigned required, double requestedV) noexcept
    {
        char record[481]{};
        const int length=std::snprintf(record,sizeof(record),
            "[EDM147] event=OPERATOR action=%s session=%llu row=%u cursor=%u depth=%u ADdisplayV=%.3f referenceV=%.3f requestedV=%.1f completed=%llu qualified=%llu required=%u gapBack=%llu gapForward=%llu retreatAxes=%02X returnAxes=%02X retreatKinds=%X returnKinds=%X tick=%llu authority=NONE\n",
            action,static_cast<unsigned long long>(e.session),e.row+1U,e.cursorRow+1U,e.historyDepth,
            e.voltageV,e.referenceV,requestedV,static_cast<unsigned long long>(e.cyclesCompleted),
            static_cast<unsigned long long>(e.qualifiedCycles),required,
            static_cast<unsigned long long>(e.gapBackwardCrossings),static_cast<unsigned long long>(e.gapForwardCrossings),
            e.cycleRetreatAxisMask,e.cycleReturnAxisMask,e.cycleRetreatKindMask,e.cycleReturnKindMask,
            static_cast<unsigned long long>(e.tick));
        if(length>0&&length<=480&&static_cast<std::size_t>(length)<sizeof(record))RtPrintf("%s",record);
        else RtPrintf("[EDM147] event=LOG_FORMAT_ERROR part=OPERATOR\n");
    }
}

void NCManager::ServiceEDM147PhysicalGapSameThread() noexcept
{
    const auto& s=m_edm94;
    if(!E146PhysicalMotionArm(s.gapHistoryArm)||!s.latched||!s.ready||s.armPending||
        s.finishPending||s.complete||s.cancelled||s.resetSeen||s.resetCommitted||
        s.releaseRequested||s.rtFaultReported||m_state!=NCState::RUN||
        m_mode!=NCOperationMode::MEMORY||s.feedback.faulted||s.feedback.cancelled||
        s.feedback.forceZero||s.feedback.released)return;
    EDM147::Snapshot e{};
    if(!EDM147::ReadEvidence(e)||!e.begun||e.session!=s.session||!e.tick||
        (s.feedback.lastTick>=e.tick&&s.feedback.lastTick-e.tick>400ULL))return;
    auto& observer=m_edm147;
    if(observer.session!=s.session) {observer=EDM147NCState{};observer.session=s.session;}
    const unsigned required=s.gapHistoryArm==224U?2U:1U;
    if(!e.valid) {
        if(!observer.invalidLogged) {E147OperatorCue("EVIDENCE_INVALID",e,required,75.0);observer.invalidLogged=true;}
        return;
    }
    if(e.cyclesStarted!=observer.started) {
        observer.started=e.cyclesStarted;observer.lowPrompted=true;observer.restorePrompted=false;
        E147OperatorCue("LOW_CONFIRMED",e,required,20.0);
    }
    if(e.interruptedCycles!=observer.interrupted) {
        observer.interrupted=e.interruptedCycles;observer.lowPrompted=false;observer.restorePrompted=false;
        E147OperatorCue("RESTORE_HIGH_INTERRUPTED",e,required,75.0);
    }
    if(e.cyclesCompleted!=observer.completed) {
        const bool qualified=e.qualifiedCycles>observer.qualified;
        observer.completed=e.cyclesCompleted;observer.qualified=e.qualifiedCycles;
        observer.lowPrompted=false;observer.restorePrompted=false;
        E147OperatorCue(qualified?"CYCLE_COMPLETE":"PARTIAL_RETURN",e,required,75.0);
    }
    if(e.cycleActive&&e.restoreReady&&!observer.restorePrompted) {
        observer.restorePrompted=true;E147OperatorCue("RESTORE_HIGH",e,required,75.0);
    }
    const bool activeRow=e.row+1U==s.next&&m_edm116.consumed==s.next&&
        m_edm116.acknowledgedRows+1U==m_edm116.consumed;
    if(activeRow&&!e.cycleActive&&e.historyReady&&e.phase==EDM128::Phase::Gap&&
        e.direction==1&&e.targetMmSec>0.0&&e.row>=2U&&e.row<=13U&&
        e.qualifiedCycles<required&&!observer.lowPrompted) {
        observer.lowPrompted=true;
        E147OperatorCue(e.cyclesCompleted?"LOW_AGAIN":"LOW_NOW",e,required,20.0);
    }
}

void NCManager::LogEDM147PhysicalGapSameThread(const char* event) const noexcept
{
    const auto& s=m_edm94;
    if(!E146PhysicalMotionArm(s.gapHistoryArm)||!event||
        (std::strcmp(event,"PASS")&&std::strcmp(event,"NORMAL_RELEASE_REQUEST")&&
            std::strcmp(event,"NORMAL_RELEASED")&&std::strcmp(event,"REJECT")&&
            std::strcmp(event,"CANCEL_REQUEST")))return;
    EDM147::Snapshot e{};
    const bool read=EDM147::ReadEvidence(e)&&e.begun&&e.session==s.session;
    if(!read)e=EDM147::Snapshot{};
    const unsigned required=s.gapHistoryArm==224U?2U:1U;
    const bool runFailed=!std::strcmp(event,"REJECT")||!std::strcmp(event,"CANCEL_REQUEST")||
        s.rtFaultReported||s.resetSeen||s.resetCommitted;
    const char* result=runFailed?"RUN_FAILED":!read?"UNAVAILABLE":!e.valid?"INVALID":
        !e.cyclesStarted?"NOT_EXERCISED":
        (e.cycleActive||e.qualifiedCycles<required)?"INCOMPLETE":"VERIFIED";
    char record[481]{};
    int length=std::snprintf(record,sizeof(record),
        "[EDM147] event=PHYSICAL_GAP result=%s motion=%.24s session=%llu read=%u started=%llu completed=%llu qualified=%llu required=%u interrupted=%llu gapBack=%llu gapForward=%llu reverseSamples=%llu forwardSamples=%llu source=PHYSICAL_AD phaseFilter=GAP_ONLY scope=COMMITTED_MAPPING authority=NONE\n",
        result,event,static_cast<unsigned long long>(s.session),read?1U:0U,
        static_cast<unsigned long long>(e.cyclesStarted),static_cast<unsigned long long>(e.cyclesCompleted),
        static_cast<unsigned long long>(e.qualifiedCycles),required,static_cast<unsigned long long>(e.interruptedCycles),
        static_cast<unsigned long long>(e.gapBackwardCrossings),static_cast<unsigned long long>(e.gapForwardCrossings),
        static_cast<unsigned long long>(e.reverseMappedSamples),static_cast<unsigned long long>(e.forwardMappedSamples));
    if(length>0&&length<=480&&static_cast<std::size_t>(length)<sizeof(record))RtPrintf("%s",record);
    else RtPrintf("[EDM147] event=LOG_FORMAT_ERROR part=PHYSICAL_GAP\n");
    length=std::snprintf(record,sizeof(record),
        "[EDM147] event=PHYSICAL_GAP_STATE motion=%.24s session=%llu valid=%u reason=%s activeCycle=%u row=%u cursor=%u depth=%u maxGapDepth=%u retreatAxes=%02X returnAxes=%02X retreatKinds=%X returnKinds=%X ADdisplayV=%.3f referenceV=%.3f anchorMm=%.9g stationMm=%.9g sourceSeq=%llu captureUs=%llu tick=%llu authority=NONE\n",
        event,static_cast<unsigned long long>(s.session),e.valid?1U:0U,EDM147::ReasonName(e.reason),e.cycleActive?1U:0U,
        e.row+1U,e.cursorRow+1U,e.historyDepth,e.maximumGapDepth,e.retreatAxisMask,e.returnAxisMask,
        e.retreatKindMask,e.returnKindMask,e.voltageV,e.referenceV,e.anchorStationMm,e.stationMm,
        static_cast<unsigned long long>(e.sourceSequence),static_cast<unsigned long long>(e.sourceCapturedUs),
        static_cast<unsigned long long>(e.tick));
    if(length>0&&length<=480&&static_cast<std::size_t>(length)<sizeof(record))RtPrintf("%s",record);
    else RtPrintf("[EDM147] event=LOG_FORMAT_ERROR part=PHYSICAL_GAP_STATE\n");
}

void NCManager::LogEDM130SameThread(const char* event, unsigned step) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm116;
    if (!s.gapWorkTime || !s.gapHistoryMode) return;
    if (EDM149::IsTestArm(s.gapHistoryArm))
    {
        EDM128::MotionFeedback timed{}; EDM126::MotionFeedback history{};
        const bool read = m_motion.ReadEDM130Feedback(timed) && m_motion.ReadEDM126Feedback(history);
        char record[481]{};
        const int length = std::snprintf(record,sizeof(record),"[%s] event=%.24s session=%llu mode=%s ADrequired=%u read=%u rows=%u/%u phase=%s workUs=%llu triggered=%llu completed=%llu exactReturn=%u voltageV=%.6f targetRatio=%.6f motionAxes=%s S92=INHIBITED S93=INHIBITED discharge=0\n",
            EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) ? "EDM167" : EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) ? "EDM159" : EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) ? "EDM158" : EDM149::IsPauseTestArm(s.gapHistoryArm) ? "EDM157" : EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) ? "EDM156" : EDM149::IsSparseArcTestArm(s.gapHistoryArm) ? "EDM155" : EDM149::IsSparseTestArm(s.gapHistoryArm) ? "EDM154" : EDM149::IsOrderTestArm(s.gapHistoryArm) ? "EDM153" : EDM149::IsArcTestArm(s.gapHistoryArm) ? "EDM152" : EDM149::IsSixTestArm(s.gapHistoryArm) ? "EDM151" : EDM149::IsXYZTestArm(s.gapHistoryArm) ? "EDM150" : "EDM149", event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), EDM149::SourceName(E149ControlSource(s.gapHistoryArm)), s.gapPhysicalAd ? 1U : 0U, read ? 1U : 0U,
            c.consumed,c.acknowledgedRows,EDM128::PhaseName(timed.coordinator.phase),
            static_cast<unsigned long long>(timed.coordinator.work.totalWorkUs),
            static_cast<unsigned long long>(timed.coordinator.work.triggeredEvents),
            static_cast<unsigned long long>(timed.coordinator.work.completedEvents),timed.coordinator.exactReturn ? 1U : 0U,
            history.voltage,history.targetRatio,(EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) || EDM149::IsSparseArcTestArm(s.gapHistoryArm) || EDM149::IsSparseTestArm(s.gapHistoryArm) || EDM149::IsOrderTestArm(s.gapHistoryArm) || EDM149::IsArcTestArm(s.gapHistoryArm) || EDM149::IsSixTestArm(s.gapHistoryArm)) ? "XYZCUV" : EDM149::IsXYZTestArm(s.gapHistoryArm) ? "XYZ" : "Z");
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s",record);
        else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=%s\n", EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) ? "EDM167" : EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) ? "EDM159" : EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) ? "EDM158" : EDM149::IsPauseTestArm(s.gapHistoryArm) ? "EDM157" : EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) ? "EDM156" : EDM149::IsSparseArcTestArm(s.gapHistoryArm) ? "EDM155" : EDM149::IsSparseTestArm(s.gapHistoryArm) ? "EDM154" : EDM149::IsOrderTestArm(s.gapHistoryArm) ? "EDM153" : EDM149::IsArcTestArm(s.gapHistoryArm) ? "EDM152" : EDM149::IsSixTestArm(s.gapHistoryArm) ? "EDM151" : "EDM149", (EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) || EDM149::IsSparseArcTestArm(s.gapHistoryArm) || EDM149::IsSparseTestArm(s.gapHistoryArm) || EDM149::IsOrderTestArm(s.gapHistoryArm) || EDM149::IsArcTestArm(s.gapHistoryArm) || EDM149::IsSixTestArm(s.gapHistoryArm)) ? "XYZCUV_MOTION" : "Z_MOTION");
        // Observations from the two already read RT snapshots. A newly issued
        // row can precede its RT application, so report the actual RT policy's
        // revision/PC instead of relabelling it as the requested NC step.
        const bool identityMatch = read && timed.enabled && history.enabled &&
            timed.session == s.session && history.session == s.session &&
            timed.armEpoch == s.armEpoch && history.armEpoch == s.armEpoch &&
            timed.ownerGeneration == s.lease.generation && history.ownerGeneration == s.lease.generation;
        const auto& policy = timed.rowPolicy;
        if (identityMatch && EDM116::ValidPolicy(policy) && policy.revision <= s.moves)
        {
            const unsigned row = static_cast<unsigned>(policy.revision - 1ULL);
            const bool planMatch = policy.fixture == s.gapModeFixture && policy.selectorPC == s.pc[row] &&
                policy.mode == m_edm127.expectedMode[row] && policy.modeOriginPC == m_edm127.expectedOriginPC[row] &&
                policy.modeOriginRevision == m_edm127.expectedOriginRevision[row];
            const int rowLength = std::snprintf(record, sizeof(record),
                "[EDM160] event=MODE_ROW motion=%.24s session=%llu row=%u requestStep=%u G=%u B=%u input=%s sourcePC=%d dispatch=%llu originPC=%d originDispatch=%llu originRevision=%llu rowRevision=%llu planMatch=%u source=RT_ROW_POLICY authority=NONE\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), row, step,
                static_cast<unsigned>(s.authoredGCode[row]), static_cast<unsigned>(policy.mode),
                m_edm127.authoredInline[row] ? "EXPLICIT" : "INHERITED", policy.selectorPC,
                static_cast<unsigned long long>(policy.selectorDispatch), policy.modeOriginPC,
                static_cast<unsigned long long>(policy.modeOriginDispatch), static_cast<unsigned long long>(policy.modeOriginRevision),
                static_cast<unsigned long long>(policy.revision), planMatch ? 1U : 0U);
            if (rowLength > 0 && rowLength <= 480 && static_cast<std::size_t>(rowLength) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM160] event=LOG_FORMAT_ERROR part=MODE_ROW\n");
        }
        const auto& mode = timed.coordinator; const auto& traversal = history.traversal;
        const int totalsLength = std::snprintf(record, sizeof(record),
            "[EDM160] event=MODE_TOTALS motion=%.24s session=%llu read=%u identityMatch=%u timerTick=%llu historyTick=%llu B0=%u B1=%u B2=%u triggered=%llu completed=%llu events=%u dwells=%u back=%llu forward=%llu depth=%u maxDepth=%u source=RT_SNAPSHOTS authority=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), read ? 1U : 0U, identityMatch ? 1U : 0U,
            static_cast<unsigned long long>(timed.tick), static_cast<unsigned long long>(history.tick),
            static_cast<unsigned>(mode.modeEvents[0]), static_cast<unsigned>(mode.modeEvents[1]), static_cast<unsigned>(mode.modeEvents[2]),
            static_cast<unsigned long long>(mode.work.triggeredEvents), static_cast<unsigned long long>(mode.work.completedEvents),
            static_cast<unsigned>(mode.eventCount), static_cast<unsigned>(mode.qualifiedDwells),
            static_cast<unsigned long long>(traversal.backwardCrossings), static_cast<unsigned long long>(traversal.forwardCrossings),
            traversal.historyDepth, traversal.maximumHistoryDepth);
        if (totalsLength > 0 && totalsLength <= 480 && static_cast<std::size_t>(totalsLength) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM160] event=LOG_FORMAT_ERROR part=MODE_TOTALS\n");
        const auto anchorRow = mode.anchor.cursorSource.ordinal, targetRow = mode.targetSource.ordinal;
        EmitEDM161B2History(s.session, timed, m_edm130.recipe, event, identityMatch,
            anchorRow < s.moves ? static_cast<unsigned>(s.authoredGCode[static_cast<unsigned>(anchorRow)]) : 0U,
            targetRow < s.moves ? static_cast<unsigned>(s.authoredGCode[static_cast<unsigned>(targetRow)]) : 0U);
        if (read && EDM130::HasBoundFlushStages(timed.timerRecipe)) EmitEDM137FlushStage(s.session,timed,event,
            EDM130::SameRecipe(timed.timerRecipe,m_edm130.recipe));
        return;
    }
    LogEDM147PhysicalGapSameThread(event);
    char record[481]{}; int length = 0;
    if (step < EDM114::RequiredRows)
    {
        const auto& p = c.policies[step];
        length = std::snprintf(record, sizeof(record),
            "[EDM130] event=%.24s session=%llu row=%u G=%d B=%u input=%s sourcePC=%d dispatch=%llu originPC=%d originDispatch=%llu originRevision=%llu rowRevision=%llu replay=0 powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), step,
            EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]), static_cast<unsigned>(p.mode),
            m_edm127.authoredInline[step] ? "EXPLICIT" : "INHERITED", p.selectorPC,
            static_cast<unsigned long long>(p.selectorDispatch), p.modeOriginPC,
            static_cast<unsigned long long>(p.modeOriginDispatch), static_cast<unsigned long long>(p.modeOriginRevision),
            static_cast<unsigned long long>(p.revision));
    }
    else
    {
        EDM128::MotionFeedback f{}; EDM126::MotionFeedback h{};
        const bool read = m_motion.ReadEDM130Feedback(f) && m_motion.ReadEDM126Feedback(h);
        length = std::snprintf(record, sizeof(record),
            "[EDM130] event=%.24s session=%llu armP=%u read=%u rows=%u/%u explicitRows=%u inheritedRows=%u standaloneB=%u line=%u arc=%u B0=%u B1=%u B2=%u events=%u dwells=%u back=%llu forward=%llu depth=%u maxDepth=%u fault=%u replay=0 source=%s powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), s.gapHistoryArm, read ? 1U : 0U,
            c.consumed, c.acknowledgedRows, m_edm127.explicitRows, m_edm127.inheritedRows, m_edm127.standaloneSelections,
            m_edm114.retained.lineRows, m_edm114.retained.arcRows,
            static_cast<unsigned>(f.coordinator.modeEvents[0]), static_cast<unsigned>(f.coordinator.modeEvents[1]),
            static_cast<unsigned>(f.coordinator.modeEvents[2]), static_cast<unsigned>(f.coordinator.eventCount),
            static_cast<unsigned>(f.coordinator.qualifiedDwells),
            static_cast<unsigned long long>(h.traversal.backwardCrossings), static_cast<unsigned long long>(h.traversal.forwardCrossings),
            h.traversal.historyDepth, h.traversal.maximumHistoryDepth, (f.faulted || h.faulted) ? 1U : 0U,
            E139DiagnosticArm(s.gapHistoryArm) ? "SIMULATED_AD" : "PHYSICAL_AD");
    }
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM130] event=LOG_FORMAT_ERROR\n");
    EDM129::MotionFeedback physical{}; EDM126::MotionFeedback gap{};
    const bool sourceRead = m_motion.ReadEDM129Feedback(physical);
    const bool gapRead = m_motion.ReadEDM126Feedback(gap);
    const auto& sample = physical.source.frame;
    const bool ageKnown = sample.capturedAtUs && physical.nowUs >= sample.capturedAtUs;
    double referenceV = EDM97::ReferenceVoltageV;
    if (E135ReferenceArm(s.gapHistoryArm))
    {
        EDM128::MotionFeedback timed{};
        const bool referenceKnown = m_motion.ReadEDM130Feedback(timed) && timed.enabled && timed.workTimeFlush &&
            timed.session == s.session && timed.armEpoch == s.armEpoch && timed.ownerGeneration == s.lease.generation &&
            EDM130::ValidRecipe(timed.timerRecipe) && EDM130::HasBoundReferenceVoltage(timed.timerRecipe);
        referenceV = referenceKnown ? EDM130::FrozenGapReferenceVoltageV(timed.timerRecipe) :
            (std::numeric_limits<double>::quiet_NaN)();
    }
    length = std::snprintf(record, sizeof(record),
        "[EDM130] event=AD_SOURCE result=%.24s session=%llu read=%u source=PHYSICAL_AD receipt=%s ready=%u reason=%s AD=%u device=%u channel=%u seq=%llu captureUs=%llu ageKnown=%u ageUs=%llu V=%.3f refV=%.3f target=%.6f gapRead=%u sourceTick=%llu gapTick=%llu fresh=%llu held=%llu maxAgeUs=%llu powerControl=NC_PLC_S92\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), sourceRead ? 1U : 0U,
        physical.active ? "LIVE" : "RETIRED", physical.sourceReady ? 1U : 0U, EDM129::SourceReasonName(physical.source.reason),
        physical.binding.route.adIndex, physical.binding.route.deviceId, physical.binding.route.channelId,
        static_cast<unsigned long long>(sample.sequence), static_cast<unsigned long long>(sample.capturedAtUs),
        ageKnown ? 1U : 0U, static_cast<unsigned long long>(ageKnown ? physical.nowUs - sample.capturedAtUs : 0ULL),
        sample.voltageV, referenceV, gap.targetRatio, gapRead ? 1U : 0U,
        static_cast<unsigned long long>(physical.tick), static_cast<unsigned long long>(gap.tick),
        static_cast<unsigned long long>(physical.source.freshFrames), static_cast<unsigned long long>(physical.source.heldFrames),
        static_cast<unsigned long long>(physical.source.maximumAgeUs));
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM130] event=LOG_FORMAT_ERROR part=AD_SOURCE\n");

    if (E141MotionArm(s.gapHistoryArm))
    {
        length = std::snprintf(record, sizeof(record),
            "[EDM141] event=MOTION_SUMMARY result=%.24s session=%llu planned=%u verified=%u expressionRows=%u assignments=%u local1=%.9g local2=%.9g powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), m_edm141.plannedRows,
            m_edm141.verifiedRows, m_edm141.expressionRows, m_edm140.next, m_edm140.values[0], m_edm140.values[1]);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM141] event=LOG_FORMAT_ERROR part=MOTION_SUMMARY\n");
    }
    if (E140AssignmentArm(s.gapHistoryArm))
    {
        length = std::snprintf(record, sizeof(record),
            "[EDM140] event=LOCAL_SUMMARY result=%.24s session=%llu planned=%u committed=%u defined=%u value1=%.9g value2=%.9g lastCommit=%llu modalB=%u powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), m_edm140.count,
            m_edm140.next, m_edm140.definedMask, m_edm140.values[0], m_edm140.values[1],
            static_cast<unsigned long long>(m_edm140.lastCommit), static_cast<unsigned>(m_edm127.current.mode));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM140] event=LOG_FORMAT_ERROR part=LOCAL_SUMMARY\n");
    }
    if (E139DiagnosticArm(s.gapHistoryArm))
    {
        const auto& d = physical.diagnostic;
        length = std::snprintf(record, sizeof(record),
            "[EDM139] event=GAP_DIAGNOSTIC result=%.24s session=%llu source=SIMULATED_AD physicalMonitor=PHYSICAL_AD row=%u phase=%u controlV=%.3f physicalV=%.3f seq=%llu tick=%llu started=%u restored=%u completed=%u backMask=%u forwardMask=%u back=%u forward=%u retreatAxes=%u returnAxes=%u powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), d.row,
            static_cast<unsigned>(d.phase), d.controlFrame.voltageV, sample.voltageV,
            static_cast<unsigned long long>(d.controlFrame.sequence), static_cast<unsigned long long>(d.tick),
            d.startedMask, d.restoredMask, d.completedMask, d.backwardMask, d.forwardMask,
            d.backwardCrossings, d.forwardCrossings, d.retreatAxisMask, d.returnAxisMask);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM139] event=LOG_FORMAT_ERROR part=GAP_DIAGNOSTIC\n");
    }
    if (E134MachiningArm(s.gapHistoryArm))
    {
        const bool diagnostic = E139DiagnosticArm(s.gapHistoryArm);
        const double controlVoltage = diagnostic ? gap.controlVoltageV : gap.voltage;
        const auto controlSequence = diagnostic ? gap.controlSequence : gap.sourceSequence;
        const auto controlCapturedUs = diagnostic ? gap.controlCapturedUs : gap.sourceCapturedUs;
        EDM128::MotionFeedback timed{}; EDM96::Feedback curve{};
        const bool current = m_edm130.bound && EDM130::ValidRecipe(m_edm130.recipe) &&
            EDM130::HasBoundMachiningSpeed(m_edm130.recipe) && m_motion.ReadEDM130Feedback(timed) &&
            timed.enabled && timed.workTimeFlush && timed.session == s.session && timed.armEpoch == s.armEpoch &&
            timed.ownerGeneration == s.lease.generation && EDM130::SameRecipe(timed.timerRecipe, m_edm130.recipe) &&
            gapRead && m_motion.ReadEDM96Feedback(curve) && curve.session == s.session &&
            curve.ownerGeneration == s.lease.generation && curve.epoch == gap.liveEpoch && curve.tick && curve.nowUs &&
            curve.sourceSequence && (diagnostic || curve.sourceSequence > physical.binding.baselineSequence) &&
            curve.sourceCapturedUs > physical.binding.admittedUs && curve.nowUs >= curve.sourceCapturedUs &&
            curve.nowUs - curve.sourceCapturedUs <= EDM126::MaximumSourceAgeUs &&
            (gap.nowUs > curve.nowUs ? gap.nowUs - curve.nowUs : curve.nowUs - gap.nowUs) <= EDM126::MaximumSourceAgeUs &&
            (controlSequence == curve.sourceSequence ? (controlCapturedUs == curve.sourceCapturedUs &&
                controlVoltage == curve.voltageV && gap.targetRatio == curve.targetRatio) :
                (controlSequence < curve.sourceSequence ? controlCapturedUs < curve.sourceCapturedUs :
                    controlCapturedUs > curve.sourceCapturedUs)) &&
            IsEDM97TuningCurrentSameThread(curve) && IsEDM129SourceCurrentSameThread(&gap);
        if (current)
        {
            length = std::snprintf(record, sizeof(record),
                E136CurveArm(s.gapHistoryArm) ?
                "[EDM134] event=GAP_CURVE result=%.24s session=%llu row=%u tick=%llu sourceSeq=%llu E8field=%u E8pct=%.6f voltageV=%.6f rawMmMin=%.12g limitedMmMin=%.9f targetRatio=%.9f proof=FROZEN_PRELIMIT scope=OBSERVER authority=NONE powerControl=NC_PLC_S92\n" :
                "[EDM134] event=GAP_CURVE result=%.24s session=%llu row=%u tick=%llu sourceSeq=%llu E8field=%u E8pct=%.6f voltageV=%.6f rawMmMin=%.9f limitedMmMin=%.9f targetRatio=%.9f proof=FROZEN_PRELIMIT scope=OBSERVER authority=NONE powerControl=NC_PLC_S92\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                static_cast<unsigned long long>(curve.tick), static_cast<unsigned long long>(curve.sourceSequence),
                static_cast<unsigned>(timed.timerRecipe.machiningFieldId), EDM130::FrozenMachiningOverridePercent(timed.timerRecipe),
                curve.voltageV, curve.rawSpeedMmMin, curve.limitedSpeedMmMin, curve.targetRatio);
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM134] event=LOG_FORMAT_ERROR part=GAP_CURVE\n");
            if (E135ReferenceArm(s.gapHistoryArm))
            {
                const double reference = EDM130::FrozenGapReferenceVoltageV(timed.timerRecipe);
                const auto profile = EDM97::MakeProfile(s.gapTuning, timed.timerRecipe);
                length = std::snprintf(record, sizeof(record),
                    "[EDM135] event=GAP_REFERENCE result=%.24s session=%llu row=%u tick=%llu sourceSeq=%llu E9field=%u referenceV=%.6f voltageV=%.6f errorV=%.6f decisionThresholdV=%.6f hysteresisV=%.6f enterMs=%u exitMs=%u direction=%d thresholdScope=FROZEN_CONFIG curveProof=OWN_SOURCE authority=NONE\n",
                    event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                    static_cast<unsigned long long>(curve.tick), static_cast<unsigned long long>(curve.sourceSequence),
                    static_cast<unsigned>(timed.timerRecipe.referenceFieldId), reference, curve.voltageV, curve.errorV,
                    profile.shortMachiningV, profile.shortHysteresisV, profile.shortEnterMs, profile.shortExitMs, curve.direction);
                if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
                else RtPrintf("[EDM135] event=LOG_FORMAT_ERROR part=GAP_REFERENCE\n");
                if (E136CurveArm(s.gapHistoryArm))
                {
                    const auto evaluated = EDMGapServo::Evaluate(profile, curve.voltageV, reference,
                        EDM130::FrozenMachiningOverridePercent(timed.timerRecipe));
                    length = std::snprintf(record, sizeof(record),
                        "[EDM136] event=PROCESS_CURVE result=%.24s session=%llu row=%u tick=%llu sourceSeq=%llu processRev=%u tuningGen=%llu segment=%u saturated=%u voltageV=%.9g errorV=%.9g rawMmMin=%.12g limitedMmMin=%.12g targetRatio=%.12g shapeSource=PROCESS_PROFILE proof=FROZEN_OWN_SOURCE authority=NONE\n",
                        event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                        static_cast<unsigned long long>(curve.tick), static_cast<unsigned long long>(curve.sourceSequence),
                        timed.timerRecipe.curve.processRevision, static_cast<unsigned long long>(timed.timerRecipe.curve.tuningGeneration),
                        static_cast<unsigned>(evaluated.segment), evaluated.saturated ? 1U : 0U,
                        curve.voltageV, curve.errorV, curve.rawSpeedMmMin, curve.limitedSpeedMmMin, curve.targetRatio);
                    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
                    else RtPrintf("[EDM136] event=LOG_FORMAT_ERROR part=PROCESS_CURVE\n");
                }
            }
        }
        else RtPrintf("[EDM134] event=GAP_CURVE result=%.24s session=%llu evidence=UNAVAILABLE authority=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session));
    }

    // Read the RT-latched pre-containment observations independently of the
    // live recipe: cancellation may already have retired its NC-side copy.
    if (E133DeepArm(s.gapHistoryArm) && gapRead && gap.session == s.session)
    {
        const EDM126::FirstCauseRecord* causes[2U] = { &gap.firstSource, &gap.firstOrphan };
        for (unsigned cause = 0U; cause < 2U; ++cause)
        {
            const auto& d = *causes[cause];
            if (d.site == EDM126::FirstCauseSite::None) continue;
            const char* kind = cause ? "ORPHAN" : "SOURCE";
            length = std::snprintf(record, sizeof(record),
                "[EDM133-FIX1] event=FIRST_CAUSE result=%.24s session=%llu kind=%s site=%u axis=%d tick=%llu nowUs=%llu evaluated=%08X failed=%08X flags=%08X guardReason=%u cmdFailure=%u observationOnly=1\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), kind,
                static_cast<unsigned>(d.site), static_cast<int>(d.axis), static_cast<unsigned long long>(d.tick),
                static_cast<unsigned long long>(d.nowUs), d.evaluatedGates, d.failedGates, d.flags,
                static_cast<unsigned>(d.guardReason), static_cast<unsigned>(d.commandFailure));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM133-FIX1] event=LOG_FORMAT_ERROR part=FIRST_CAUSE\n");
            length = std::snprintf(record, sizeof(record),
                "[EDM133-FIX1] event=CAUSE_SCOPE session=%llu kind=%s tick=%llu epoch=%llu expectedEpoch=%llu liveEpoch=%llu owner=%u ownerGen=%llu committedTick=%llu publishedTick=%llu\n",
                static_cast<unsigned long long>(s.session), kind, static_cast<unsigned long long>(d.tick),
                static_cast<unsigned long long>(d.executionEpoch), static_cast<unsigned long long>(d.expectedEpoch),
                static_cast<unsigned long long>(d.liveEpoch), static_cast<unsigned>(d.owner),
                static_cast<unsigned long long>(d.ownerGeneration), static_cast<unsigned long long>(d.committedTick),
                static_cast<unsigned long long>(d.publishedTick));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM133-FIX1] event=LOG_FORMAT_ERROR part=CAUSE_SCOPE\n");
            length = std::snprintf(record, sizeof(record),
                "[EDM133-FIX1] event=CAUSE_MAPPING session=%llu kind=%s tick=%llu groupActive=%u groupAxes=%d cmdAxes=%d members=%02X interpolating=%02X inputMask=%02X status=%04X/%04X/%04X/%04X/%04X/%04X modes=%d/%d/%d/%d/%d/%d\n",
                static_cast<unsigned long long>(s.session), kind, static_cast<unsigned long long>(d.tick),
                (d.flags & EDM126::FirstCauseGroupActive) ? 1U : 0U,
                static_cast<int>(d.groupAxisCount), static_cast<int>(d.commandAxisCount),
                static_cast<unsigned>(d.memberMask), static_cast<unsigned>(d.interpolatingMask),
                static_cast<unsigned>(d.inputMask), static_cast<unsigned>(d.statusWord[0]), static_cast<unsigned>(d.statusWord[1]),
                static_cast<unsigned>(d.statusWord[2]), static_cast<unsigned>(d.statusWord[3]),
                static_cast<unsigned>(d.statusWord[4]), static_cast<unsigned>(d.statusWord[5]),
                static_cast<int>(d.modes[0]), static_cast<int>(d.modes[1]), static_cast<int>(d.modes[2]),
                static_cast<int>(d.modes[3]), static_cast<int>(d.modes[4]), static_cast<int>(d.modes[5]));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM133-FIX1] event=LOG_FORMAT_ERROR part=CAUSE_MAPPING\n");
            length = std::snprintf(record, sizeof(record),
                "[EDM133-FIX1] event=CAUSE_MEMBERS session=%llu kind=%s tick=%llu groupOrder=%06X cmdOrder=%06X exist=%02X config=%02X mapping=%02X fault=%02X servoOn=%02X axisIndex=%02X order=LOW_NIBBLE_SLOT0_INVALID_F\n",
                static_cast<unsigned long long>(s.session), kind, static_cast<unsigned long long>(d.tick),
                d.orderedGroupAxes, d.orderedCommandAxes, static_cast<unsigned>(d.existMask),
                static_cast<unsigned>(d.configMatchMask), static_cast<unsigned>(d.mappingMatchMask),
                static_cast<unsigned>(d.faultMask), static_cast<unsigned>(d.servoOnMask),
                static_cast<unsigned>(d.axisIndexMatchMask));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM133-FIX1] event=LOG_FORMAT_ERROR part=CAUSE_MEMBERS\n");
            length = std::snprintf(record, sizeof(record),
                "[EDM133-FIX1] event=CAUSE_COORD session=%llu kind=%s tick=%llu committedPhase=%u proposedPhase=%u publishedPhase=%u eventIndex=%llu privateTriggered=%llu privateCompleted=%llu publicTriggered=%llu publicCompleted=%llu\n",
                static_cast<unsigned long long>(s.session), kind, static_cast<unsigned long long>(d.tick),
                static_cast<unsigned>(d.committedPhase), static_cast<unsigned>(d.proposedPhase),
                static_cast<unsigned>(d.publishedPhase), static_cast<unsigned long long>(d.eventIndex),
                static_cast<unsigned long long>(d.privateTriggered), static_cast<unsigned long long>(d.privateCompleted),
                static_cast<unsigned long long>(d.publicTriggered), static_cast<unsigned long long>(d.publicCompleted));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM133-FIX1] event=LOG_FORMAT_ERROR part=CAUSE_COORD\n");
        }
    }

    EDM128::MotionFeedback timed{};
    if (m_motion.ReadEDM130Feedback(timed) && timed.enabled && timed.workTimeFlush && timed.session == s.session)
    {
        if (E137FlushStageArm(s.gapHistoryArm))
            EmitEDM137FlushStage(s.session, timed, event, EDM130::SameRecipe(timed.timerRecipe, m_edm130.recipe));
        if (E138CrossSegmentArm(s.gapHistoryArm))
        {
            const auto& e = timed.coordinator;
            const bool planned = e.anchorCaptured && e.targetCaptured &&
                e.policy.mode == EDM116::Mode::B2 && e.targetSource.ordinal < e.anchor.cursorSource.ordinal;
            const bool verified = planned && e.topCaptured && e.exactReturn && e.rowEventComplete &&
                e.eventIndex != 0ULL && e.work.completedEvents >= e.eventIndex;
            length = std::snprintf(record, sizeof(record),
                "[EDM138] event=B2_HISTORY result=%.24s session=%llu eventIndex=%llu phase=%s B=%u anchorCaptured=%u targetCaptured=%u anchorRow=%llu anchorPC=%d targetRow=%llu targetPC=%d crossingPlanned=%u topCaptured=%u exactReturn=%u rowEventComplete=%u verifiedCross=%u scope=OBSERVER authority=NONE\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session),
                static_cast<unsigned long long>(e.eventIndex), EDM128::PhaseName(e.phase),
                static_cast<unsigned>(e.policy.mode), e.anchorCaptured ? 1U : 0U, e.targetCaptured ? 1U : 0U,
                static_cast<unsigned long long>(e.anchor.cursorSource.ordinal), e.anchor.cursorSource.sourcePC,
                static_cast<unsigned long long>(e.targetSource.ordinal), e.targetSource.sourcePC,
                planned ? 1U : 0U, e.topCaptured ? 1U : 0U, e.exactReturn ? 1U : 0U,
                e.rowEventComplete ? 1U : 0U, verified ? 1U : 0U);
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM138] event=LOG_FORMAT_ERROR part=B2_HISTORY\n");
        }
        if (E131SpeedArm(s.gapHistoryArm))
        {
            const auto& recipe = timed.timerRecipe;
            length = std::snprintf(record, sizeof(record),
                "[EDM131] event=FLUSH_SPEED result=%.24s session=%llu row=%u rowBound=%u B=%u phase=%s E7field=%u E7pct=%.6f baseJpct=50 armRatio=%.9f rowRatio=%.9f profileMatches=%u recipeMatches=%u speedScope=FLUSH_ONLY powerControl=NC_PLC_S92\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                timed.authorizedRows ? 1U : 0U, static_cast<unsigned>(timed.rowPolicy.mode), EDM128::PhaseName(timed.coordinator.phase),
                static_cast<unsigned>(recipe.speedFieldId), static_cast<double>(recipe.jumpSpeedPercentScaled) / 1000000.0,
                timed.flushProfile.speedRatio, timed.rowPolicy.profile.speedRatio,
                EDM130::MatchesFlushSpeed(recipe, timed.flushProfile.speedRatio) ? 1U : 0U,
                EDM130::SameRecipe(recipe, m_edm130.recipe) ? 1U : 0U);
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM131] event=LOG_FORMAT_ERROR part=FLUSH_SPEED\n");
        }
        if (E132HeightArm(s.gapHistoryArm))
        {
            const auto& e = timed.coordinator;
            const double heightMm = E133DeepArm(s.gapHistoryArm) && e.eventIndex ? e.effectiveJumpHeightMm : EDM130::FrozenJumpHeightMm(timed.timerRecipe);
            length = std::snprintf(record, sizeof(record),
                "[EDM132] event=FLUSH_HEIGHT result=%.24s session=%llu row=%u B=%u phase=%s E6field=%u requestedMm=%.6f actualMm=%.9f targetCaptured=%u topCaptured=%u targetRow=%llu targetPC=%d targetLocalMm=%.9f targetStationMm=%.9f anchorStationMm=%.9f exactReturn=%u proofValid=%u distanceSource=MAPPED powerControl=NC_PLC_S92\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                static_cast<unsigned>(timed.rowPolicy.mode), EDM128::PhaseName(e.phase),
                static_cast<unsigned>(timed.timerRecipe.heightFieldId), heightMm, e.actualJumpHeightMm,
                e.targetCaptured ? 1U : 0U, e.topCaptured ? 1U : 0U,
                static_cast<unsigned long long>(e.targetSource.ordinal), e.targetSource.sourcePC,
                e.targetLocalMm, e.targetStationMm, e.anchor.stationMm, e.exactReturn ? 1U : 0U,
                EDM128::ValidHeightProof(e, timed.authorizedRows ? EDM130::FrozenJumpHeightMm(timed.timerRecipe) : 0.0,
                    timed.authorizedRows ? timed.timerRecipe.deepFlushCycleInterval : 0U,
                    timed.authorizedRows ? EDM130::FrozenDeepMultiplier(timed.timerRecipe) : 0.0,
                    timed.authorizedRows && EDM130::HasBoundDeep(timed.timerRecipe)) ? 1U : 0U);
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM132] event=LOG_FORMAT_ERROR part=FLUSH_HEIGHT\n");
            length = std::snprintf(record, sizeof(record),
                "[EDM132] event=HEIGHT_TOP result=%.24s session=%llu row=%u B=%u topPulse=%.9f/%.9f/%.9f/%.9f/%.9f/%.9f tangentPulsePerMm=%.9f/%.9f/%.9f/%.9f/%.9f/%.9f powerControl=NC_PLC_S92\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                static_cast<unsigned>(timed.rowPolicy.mode), e.topPositionPulse[0], e.topPositionPulse[1],
                e.topPositionPulse[2], e.topPositionPulse[3], e.topPositionPulse[4], e.topPositionPulse[5],
                e.tangentPulsePerMm[0], e.tangentPulsePerMm[1], e.tangentPulsePerMm[2],
                e.tangentPulsePerMm[3], e.tangentPulsePerMm[4], e.tangentPulsePerMm[5]);
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM132] event=LOG_FORMAT_ERROR part=HEIGHT_TOP\n");
        }
        if (E133DeepArm(s.gapHistoryArm))
        {
            const auto& e = timed.coordinator; const auto& recipe = timed.timerRecipe;
            length = std::snprintf(record, sizeof(record),
                "[EDM133] event=FLUSH_SCHEDULE result=%.24s session=%llu row=%u B=%u phase=%s bound=%u baseMm=%.6f everyN=%u multiplier=%.6f eventIndex=%llu deep=%u effectiveMm=%.9f completedDeep=%llu triggered=%llu completed=%llu targetCaptured=%u topCaptured=%u exactReturn=%u powerControl=NC_PLC_S92\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
                static_cast<unsigned>(timed.rowPolicy.mode), EDM128::PhaseName(e.phase), e.deepBound ? 1U : 0U,
                EDM130::FrozenJumpHeightMm(recipe), static_cast<unsigned>(recipe.deepFlushCycleInterval),
                EDM130::FrozenDeepMultiplier(recipe), static_cast<unsigned long long>(e.eventIndex), e.deepCycle ? 1U : 0U,
                e.effectiveJumpHeightMm, static_cast<unsigned long long>(e.completedDeepEvents),
                static_cast<unsigned long long>(e.work.triggeredEvents), static_cast<unsigned long long>(e.work.completedEvents),
                e.targetCaptured ? 1U : 0U, e.topCaptured ? 1U : 0U, e.exactReturn ? 1U : 0U);
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM133] event=LOG_FORMAT_ERROR part=FLUSH_SCHEDULE\n");
        }
        const auto& w = timed.coordinator.work;
        length = std::snprintf(record, sizeof(record),
            "[EDM130] event=WORK_TIME build=FIX1 result=%.24s session=%llu row=%u B=%u phase=%s E5us=%llu elapsedUs=%llu totalWorkUs=%llu triggered=%llu completed=%llu due=%u triggerElapsedUs=%llu triggerTotalUs=%llu triggerTick=%llu triggerUs=%llu completeTick=%llu completeUs=%llu clock=COMMITTED_GAP_ONLY powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), timed.completedRows,
            static_cast<unsigned>(timed.rowPolicy.mode), EDM128::PhaseName(timed.coordinator.phase),
            static_cast<unsigned long long>(timed.timerRecipe.workTimeUs), static_cast<unsigned long long>(w.elapsedUs),
            static_cast<unsigned long long>(w.totalWorkUs), static_cast<unsigned long long>(w.triggeredEvents),
            static_cast<unsigned long long>(w.completedEvents), w.due ? 1U : 0U,
            static_cast<unsigned long long>(w.lastTriggerElapsedUs), static_cast<unsigned long long>(w.lastTriggerTotalUs),
            static_cast<unsigned long long>(w.lastTriggerTick), static_cast<unsigned long long>(w.lastTriggerUs),
            static_cast<unsigned long long>(w.lastCompleteTick), static_cast<unsigned long long>(w.lastCompleteUs));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM130] event=LOG_FORMAT_ERROR part=WORK_TIME\n");
    }

}

void NCManager::LogEDM129SameThread(const char* event, unsigned step) const noexcept
{
    if (m_edm94.gapWorkTime) { LogEDM130SameThread(event, step); return; }
    const auto& s = m_edm94; const auto& c = m_edm116;
    if (!s.gapPhysicalAd || !s.gapHistoryMode) return;
    char record[481]{}; int length = 0;
    if (step < EDM114::RequiredRows)
    {
        const auto& p = c.policies[step];
        length = std::snprintf(record, sizeof(record),
            "[EDM129] event=%.24s session=%llu row=%u G=%d B=%u input=%s sourcePC=%d dispatch=%llu originPC=%d originDispatch=%llu originRevision=%llu rowRevision=%llu replay=0 dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), step,
            EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]), static_cast<unsigned>(p.mode),
            m_edm127.authoredInline[step] ? "EXPLICIT" : "INHERITED", p.selectorPC,
            static_cast<unsigned long long>(p.selectorDispatch), p.modeOriginPC,
            static_cast<unsigned long long>(p.modeOriginDispatch), static_cast<unsigned long long>(p.modeOriginRevision),
            static_cast<unsigned long long>(p.revision));
    }
    else
    {
        EDM128::MotionFeedback f{}; EDM126::MotionFeedback h{};
        const bool read = m_motion.ReadEDM128Feedback(f) && m_motion.ReadEDM126Feedback(h);
        length = std::snprintf(record, sizeof(record),
            "[EDM129] event=%.24s session=%llu armP=%u read=%u rows=%u/%u explicitRows=%u inheritedRows=%u standaloneB=%u line=%u arc=%u B0=%u B1=%u B2=%u events=%u dwells=%u back=%llu forward=%llu depth=%u maxDepth=%u fault=%u replay=0 source=PHYSICAL_AD dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), s.gapHistoryArm, read ? 1U : 0U,
            c.consumed, c.acknowledgedRows, m_edm127.explicitRows, m_edm127.inheritedRows, m_edm127.standaloneSelections,
            m_edm114.retained.lineRows, m_edm114.retained.arcRows,
            static_cast<unsigned>(f.coordinator.modeEvents[0]), static_cast<unsigned>(f.coordinator.modeEvents[1]),
            static_cast<unsigned>(f.coordinator.modeEvents[2]), static_cast<unsigned>(f.coordinator.eventCount),
            static_cast<unsigned>(f.coordinator.qualifiedDwells),
            static_cast<unsigned long long>(h.traversal.backwardCrossings), static_cast<unsigned long long>(h.traversal.forwardCrossings),
            h.traversal.historyDepth, h.traversal.maximumHistoryDepth, (f.faulted || h.faulted) ? 1U : 0U);
    }
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM129] event=LOG_FORMAT_ERROR\n");
    EDM129::MotionFeedback physical{}; EDM126::MotionFeedback gap{};
    const bool sourceRead = m_motion.ReadEDM129Feedback(physical);
    const bool gapRead = m_motion.ReadEDM126Feedback(gap);
    const auto& sample = physical.source.frame;
    const bool ageKnown = sample.capturedAtUs && physical.nowUs >= sample.capturedAtUs;
    length = std::snprintf(record, sizeof(record),
        "[EDM129] event=AD_SOURCE result=%.24s session=%llu read=%u source=PHYSICAL_AD receipt=%s ready=%u reason=%s AD=%u device=%u channel=%u seq=%llu captureUs=%llu ageKnown=%u ageUs=%llu V=%.3f refV=%.3f target=%.6f gapRead=%u sourceTick=%llu gapTick=%llu fresh=%llu held=%llu maxAgeUs=%llu dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), sourceRead ? 1U : 0U,
        physical.active ? "LIVE" : "RETIRED", physical.sourceReady ? 1U : 0U, EDM129::SourceReasonName(physical.source.reason),
        physical.binding.route.adIndex, physical.binding.route.deviceId, physical.binding.route.channelId,
        static_cast<unsigned long long>(sample.sequence), static_cast<unsigned long long>(sample.capturedAtUs),
        ageKnown ? 1U : 0U, static_cast<unsigned long long>(ageKnown ? physical.nowUs - sample.capturedAtUs : 0ULL),
        sample.voltageV, EDM97::ReferenceVoltageV, gap.targetRatio, gapRead ? 1U : 0U,
        static_cast<unsigned long long>(physical.tick), static_cast<unsigned long long>(gap.tick),
        static_cast<unsigned long long>(physical.source.freshFrames), static_cast<unsigned long long>(physical.source.heldFrames),
        static_cast<unsigned long long>(physical.source.maximumAgeUs));
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM129] event=LOG_FORMAT_ERROR part=AD_SOURCE\n");

}

bool NCManager::AuthorizeEDM129SameThread(const NCBlock& block, int gcode,
    const std::array<double, 2U>& center) noexcept
{
    auto& s = m_edm94; auto& c = m_edm116;
    if ((!s.gapPhysicalAd && !s.gapWorkTime) || !s.gapHistoryMode || !s.gapHistoryServo || s.gapModeFlush || !s.gapPhysicalCurve ||
        !s.next || s.next > s.moves || !s.latched || !s.ready || s.armPending || s.cancelled ||
        s.resetSeen || s.resetCommitted || s.finishPending || s.complete || s.releaseRequested || s.rtFaultReported)
        return false;
    const unsigned step = s.next - 1U;
    const int pc = GetActiveDispatchPC(); const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    const auto& policy = c.policies[step];
    EDM127::Selection expected = m_edm127.current; double word = 0.0;
    if (!authored || pc != s.pc[step] || authored->parsedBlock.has('B') != m_edm127.authoredInline[step] ||
        (m_edm127.authoredInline[step] &&
            (!E94Number(authored->parsedBlock.expression('B'), word) ||
                !EDM127::Select(m_edm127.current, word, pc, m_currentExecutingBlockDispatchId, expected))) ||
        !EDM127::Matches(expected, policy) || !EDM127::Matches(m_edm127.pending, policy) ||
        !c.pending || c.selected != step + 1U || c.consumed != step || c.acknowledgedRows != step ||
        !EDM116::PolicyMatchesBinding(policy, step, pc, m_currentExecutingBlockDispatchId) ||
        policy.fixture != s.gapModeFixture || policy.mode != m_edm127.expectedMode[step] ||
        policy.modeOriginPC != m_edm127.expectedOriginPC[step] ||
        policy.modeOriginRevision != m_edm127.expectedOriginRevision[step] ||
        !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
        (step && (!EDM116::PolicyFollowsCompleted(policy, c.completedPolicies[step - 1U], c.completedSources[step - 1U]) ||
            !EDM114::SameSource(c.completedSources[step - 1U], m_edm114.retained.source))) ||
        !IsEDM114CurrentSameThread(step) || !IsEDM126CurrentSameThread(step) || !IsEDM129CurrentSameThread(step))
        return false;
    if (!step && (!c.initialTranslationValid || !m_edm126.initialBoundaryReady ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        !IsFixedTranslationTravelCurrentSameThread() || !IsXYZCUVFeedProfileCurrentSameThread() ||
        !SameNCTranslationSnapshot(c.initialTranslation, CoordSys.GetTranslationSnapshot()) ||
        !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()))) return false;
    const auto epoch = m_motion.GetCurrentExecutionEpoch();
    const bool authorized = s.gapWorkTime ?
        m_motion.AuthorizeEDM130Feed(s.session, s.run, s.cache, m_currentExecutingBlockDispatchId,
            pc, step, gcode, s.increment[step], center, block.val('F'), epoch, s.lease, s.gapTuning,
            m_edm114.retained.source, m_edm114.completionCommit, m_edm114.completionTick, policy, m_edm130.recipe) :
        m_motion.AuthorizeEDM129Feed(s.session, s.run, s.cache, m_currentExecutingBlockDispatchId,
            pc, step, gcode, s.increment[step], center, block.val('F'), epoch, s.lease, s.gapTuning,
            m_edm114.retained.source, m_edm114.completionCommit, m_edm114.completionTick, policy);
    if (!authorized) return false;
    c.issuedDispatch[step] = m_currentExecutingBlockDispatchId;
    c.issuedEpoch[step] = epoch == (std::numeric_limits<std::uint32_t>::max)() ? 1ULL : epoch + 1ULL;
    c.pending = false; c.consumed = step + 1U;
    m_edm127.current = m_edm127.pending;
    if (m_edm127.authoredInline[step]) ++m_edm127.explicitRows; else ++m_edm127.inheritedRows;
    m_edm127.lastPC = pc; m_edm127.lastDispatch = m_currentExecutingBlockDispatchId;
    m_edm127.pending = EDM127::Selection{};
    LogEDM129SameThread("ROW_AUTHORIZED", step);
    return true;
}

bool NCManager::RetainEDM129CompletedSameThread(const EDM94::Feedback& guard) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm116;
    if ((!s.gapPhysicalAd && !s.gapWorkTime) || !s.gapHistoryMode || !s.gapHistoryServo || !s.latched || !s.ready || s.armPending ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || !s.next || s.next > s.moves || c.pending ||
        c.selected != s.next || c.consumed != s.next || !IsEDM114CurrentSameThread(s.next) ||
        !IsEDM126CurrentSameThread(s.next) || !IsEDM129CurrentSameThread(s.next)) return false;
    if (c.acknowledgedRows == s.next)
        return EDM114::SameSource(c.completedSources[s.next - 1U], m_edm114.retained.source) &&
            EDM116::SamePolicy(c.completedPolicies[s.next - 1U], c.policies[s.next - 1U]);
    const unsigned step = s.next - 1U;
    if (c.acknowledgedRows + 1U != s.next || guard.identity.session != s.session ||
        guard.identity.epoch != s.armEpoch || guard.identity.ownerGeneration != s.lease.generation ||
        guard.currentIdentity.session != s.session || guard.currentIdentity.epoch != m_edm114.retained.source.epoch ||
        guard.currentIdentity.ownerGeneration != s.lease.generation || guard.lastTick != m_edm114.completionTick ||
        guard.lastNowUs < m_edm114.retained.lastMappingUs || !guard.active || !guard.armAccepted ||
        !guard.armedReady || guard.faulted || guard.cancelled || guard.forceZero || guard.handoffZero ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        !EDM116::PolicyMatchesSource(c.policies[step], m_edm114.retained.source)) return false;
    c.completedPolicies[step] = c.policies[step]; c.completedSources[step] = m_edm114.retained.source;
    if (s.gapWorkTime)
    {
        EDM128::MotionFeedback timed{};
        if (!m_motion.ReadEDM130Feedback(timed) || !timed.workTimeFlush || timed.session != s.session ||
            timed.completedRows != s.next || timed.authorizedRows != s.next || timed.faulted ||
            !EDM130::SameRecipe(timed.timerRecipe, m_edm130.recipe) ||
            !EDM130::ValidWorkEvidence(timed.coordinator.work)) return false;
        m_edm130.retainedWork = timed.coordinator.work;
        for (unsigned mode = 0U; mode < EDM116::ModeCount; ++mode) m_edm130.retainedModeEvents[mode] = timed.coordinator.modeEvents[mode];
        m_edm130.retainedRowEvent = timed.coordinator.rowEventComplete;
    }
    c.acknowledgedRows = s.next;
    LogEDM129SameThread("COORDINATOR_ROW_RETAINED", EDM114::RequiredRows);
    return true;
}

bool NCManager::IsEDM128CurrentSameThread(unsigned completed) const noexcept
{
    if (m_edm94.gapWorkTime) return IsEDM130CurrentSameThread(completed);
    if (m_edm94.gapPhysicalAd) return IsEDM129CurrentSameThread(completed);
    const auto& s = m_edm94; const auto& c = m_edm116;
    EDM128::MotionFeedback f{}; EDM126::MotionFeedback h{};
    const unsigned fixture = s.gapHistoryArm == 166U ? 0U : 1U;
    if (!s.gapHistoryMode || !s.gapHistoryServo || !s.gapPhysicalCurve || !s.gapSegmentProfile ||
        !s.gapGainProfile || s.gapBoundaryProfile || s.gapNoiseProfile || s.gapBurstProfile ||
        s.gapContinuousProfile || s.gapEnduranceProfile || s.gapPairedProfile || s.gapCommonPath ||
        s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
        s.gapTuning.positiveGain != 8.0 || s.gapTuning.negativeGain != 1.0 || s.gapTuning.deadbandV != 1.0 ||
        s.gapModeFlush || s.gapFlushProfile || s.gapFlushParameterized ||
        s.gapModeInterruption || s.gapModeInterruptionArm || s.gapCurveB2 || s.gapFlushInterruptArm ||
        (s.gapHistoryArm != 166U && s.gapHistoryArm != 168U) ||
        s.gapModeFixture != E149ModalFixture(fixture) ||
        !EDM107::ValidFlushProfile(s.gapFlushParameters) || !s.latched || s.armPending || !s.everArmed ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || completed > s.moves ||
        c.consumed != completed || c.selected != completed + (c.pending ? 1U : 0U) || c.selected > s.moves ||
        c.acknowledgedRows > completed || completed - c.acknowledgedRows > 1U ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.ReadEDM128Feedback(f) || !m_motion.ReadEDM126Feedback(h) ||
        !f.enabled || f.workTimeFlush || !EDM130::EmptyRecipe(f.timerRecipe) ||
        !EDM130::SameWorkEvidence(f.coordinator.work, EDM130::WorkEvidence{}) || f.physicalAd || f.faulted || f.fixture != fixture || !EDM107::SameFlushProfile(f.flushProfile, s.gapFlushParameters) ||
        f.session != s.session || f.armEpoch != s.armEpoch ||
        f.ownerGeneration != s.lease.generation || f.authorizedRows != completed || f.completedRows != completed ||
        !f.tick || !f.nowUs || (!s.complete && f.liveEpoch != m_motion.GetCurrentExecutionEpoch()) ||
        !h.enabled || h.physicalAd || h.faulted || h.reason != EDM126::Reason::None || h.fixture != fixture ||
        h.session != f.session || h.armEpoch != f.armEpoch || h.liveEpoch != f.liveEpoch ||
        h.ownerGeneration != f.ownerGeneration || h.authorizedRows != completed || h.completedRows != completed ||
        h.rowActive || h.rowCompleted != (completed != 0U) ||
        !h.sourceReady || h.sourceReason != EDM88::Reason::None || !h.sourceSequence || !h.sourceCapturedUs ||
        h.nowUs < h.sourceCapturedUs || h.nowUs - h.sourceCapturedUs > EDM126::MaximumSourceAgeUs ||
        f.lastMappingTick != h.lastMappingTick || f.lastMappingUs != h.lastMappingUs) return false;
    const auto& e = f.coordinator; const auto& t = h.traversal;
    const unsigned events = (completed + 1U) / 2U;
    if (e.liveAnchor || e.phase != EDM128::Phase::Gap || e.stableCycles || e.stableStartUs || e.faulted || e.stopped || e.detached || e.eventCount != events || e.qualifiedDwells != 3U * events ||
        t.faulted || t.stopRequested || t.stopped || t.reason != EDM126::Reason::None ||
        t.historyDepth || t.backwardCrossings != EDM128::ExpectedCrossings(completed) ||
        t.forwardCrossings != EDM128::ExpectedCrossings(completed) ||
        t.maximumHistoryDepth != EDM128::ExpectedMaximumDepth(completed)) return false;
    std::array<unsigned, EDM116::ModeCount> expectedModes{};
    for (unsigned row = 0U; row < completed; row += 2U)
    {
        if (!EDM116::ValidMode(m_edm127.expectedMode[row])) return false;
        ++expectedModes[static_cast<unsigned>(m_edm127.expectedMode[row])];
    }
    for (unsigned mode = 0U; mode < EDM116::ModeCount; ++mode)
        if (e.modeEvents[mode] != expectedModes[mode]) return false;
    for (unsigned row = 0U; row < c.selected; ++row)
    {
        const auto& policy = c.policies[row];
        if (!EDM116::ValidPolicy(policy) || policy.fixture != s.gapModeFixture ||
            policy.mode != m_edm127.expectedMode[row] || policy.modeOriginPC != m_edm127.expectedOriginPC[row] ||
            policy.modeOriginRevision != m_edm127.expectedOriginRevision[row] || policy.selectorPC != s.pc[row] ||
            policy.revision != row + 1ULL || policy.selectorDispatch <= s.armDispatch ||
            !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
            (row && policy.selectorDispatch <= c.policies[row - 1U].selectorDispatch)) return false;
    }
    if (!completed)
        return !f.lastMappingTick && !f.lastMappingUs && !e.anchorCaptured && !e.exactReturn &&
            !e.rowEventComplete && EDM116::SamePolicy(f.rowPolicy, EDM116::Policy{}) &&
            EDM116::SamePolicy(e.policy, EDM116::Policy{}) &&
            EDM126::SameSource(h.authorizedSource, EDM126::Source{});
    const unsigned step = completed - 1U;
    const auto& policy = c.policies[step]; const auto& source = h.authorizedSource;
    if (!EDM126::ValidSource(source) || source.ordinal != step || source.sourcePC != s.pc[step] ||
        source.scope.session != s.session || source.scope.run != s.run || source.scope.cache != s.cache ||
        source.scope.ownerGeneration != s.lease.generation || source.dispatch != c.issuedDispatch[step] ||
        source.epoch != c.issuedEpoch[step] || !EDM116::SamePolicy(f.rowPolicy, policy) ||
        !EDM116::SamePolicy(e.policy, policy) || !f.lastMappingTick || !f.lastMappingUs ||
        f.lastMappingTick > f.tick || f.lastMappingUs > f.nowUs ||
        !EDM126::SameSource(t.authorizedSource, source) || !EDM126::SameSource(t.cursorSource, source) ||
        !t.completeReady || !t.atCurrentEnd || t.localMm != t.lengthMm || t.velocityMmSec != 0.0 ||
        t.tick != f.lastMappingTick || t.nowUs != f.lastMappingUs ||
        t.completedRows != step || t.historyCount != step) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis) if (t.velocityPps[axis] != 0.0) return false;
    for (unsigned row = 0U; row < c.acknowledgedRows; ++row)
    {
        if (!EDM116::SamePolicy(c.completedPolicies[row], c.policies[row]) ||
            !EDM116::PolicyMatchesSource(c.completedPolicies[row], c.completedSources[row]) ||
            c.completedSources[row].sourcePC != s.pc[row] || c.completedSources[row].step != row ||
            c.completedSources[row].dispatch != c.issuedDispatch[row] ||
            c.completedSources[row].epoch != c.issuedEpoch[row] ||
            !EDM109::SameScope(c.completedSources[row].scope, source.scope) ||
            (row && !EDM116::PolicyFollowsCompleted(c.completedPolicies[row], c.completedPolicies[row - 1U],
                c.completedSources[row - 1U]))) return false;
    }
    // The exact returned anchor is separate from the canonical NC row endpoint.
    // The coordinator must finish its excursion before the live row can complete.
    const bool event = (step & 1U) == 0U;
    if (e.rowEventComplete != event || e.anchorCaptured != event || e.exactReturn != event) return false;
    if (event && (!EDM126::SameSource(e.anchor.authorizedSource, source) ||
        e.anchor.historyDepth != (step ? 1U : 0U) || e.anchor.velocityMmSec != 0.0 ||
        !EDM126::ValidSource(e.anchor.cursorSource) ||
        e.anchor.cursorSource.ordinal != (step ? step - 1U : step) || !EDM128::Zero(e.anchor) ||
        !e.anchor.tick || !e.anchor.nowUs || e.anchor.tick >= f.lastMappingTick || e.anchor.nowUs >= f.lastMappingUs ||
        !std::isfinite(e.anchor.localMm) || !std::isfinite(e.anchor.lengthMm) ||
        e.anchor.localMm <= 0.0 || e.anchor.localMm >= e.anchor.lengthMm)) return false;
    if (event && step)
    {
        const auto& previous = c.completedSources[step - 1U]; const auto& cursor = e.anchor.cursorSource;
        if (!EDM109::SameScope(cursor.scope, previous.scope) || cursor.dispatch != previous.dispatch ||
            cursor.epoch != previous.epoch || cursor.segment != previous.segment ||
            cursor.sourcePC != previous.sourcePC || cursor.ordinal != previous.step) return false;
    }
    else if (event && !EDM126::SameSource(e.anchor.cursorSource, source)) return false;
    return true;
}

void NCManager::LogEDM128SameThread(const char* event, unsigned step) const noexcept
{
    if (m_edm94.gapWorkTime) { LogEDM130SameThread(event, step); return; }
    if (m_edm94.gapPhysicalAd) { LogEDM129SameThread(event, step); return; }
    const auto& s = m_edm94; const auto& c = m_edm116;
    if (!s.gapHistoryMode) return;
    char record[481]{}; int length = 0;
    if (step < EDM114::RequiredRows)
    {
        const auto& p = c.policies[step];
        length = std::snprintf(record, sizeof(record),
            "[EDM128] event=%.24s session=%llu row=%u G=%d B=%u input=%s sourcePC=%d dispatch=%llu originPC=%d originDispatch=%llu originRevision=%llu rowRevision=%llu replay=0 dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), step,
            EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]), static_cast<unsigned>(p.mode),
            m_edm127.authoredInline[step] ? "EXPLICIT" : "INHERITED", p.selectorPC,
            static_cast<unsigned long long>(p.selectorDispatch), p.modeOriginPC,
            static_cast<unsigned long long>(p.modeOriginDispatch), static_cast<unsigned long long>(p.modeOriginRevision),
            static_cast<unsigned long long>(p.revision));
    }
    else
    {
        EDM128::MotionFeedback f{}; EDM126::MotionFeedback h{};
        const bool read = m_motion.ReadEDM128Feedback(f) && m_motion.ReadEDM126Feedback(h);
        length = std::snprintf(record, sizeof(record),
            "[EDM128] event=%.24s session=%llu armP=%u read=%u rows=%u/%u explicitRows=%u inheritedRows=%u standaloneB=%u line=%u arc=%u B0=%u B1=%u B2=%u events=%u dwells=%u back=%llu forward=%llu depth=%u maxDepth=%u fault=%u replay=0 source=RT_SYNTHETIC dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), s.gapHistoryArm, read ? 1U : 0U,
            c.consumed, c.acknowledgedRows, m_edm127.explicitRows, m_edm127.inheritedRows, m_edm127.standaloneSelections,
            m_edm114.retained.lineRows, m_edm114.retained.arcRows,
            static_cast<unsigned>(f.coordinator.modeEvents[0]), static_cast<unsigned>(f.coordinator.modeEvents[1]),
            static_cast<unsigned>(f.coordinator.modeEvents[2]), static_cast<unsigned>(f.coordinator.eventCount),
            static_cast<unsigned>(f.coordinator.qualifiedDwells),
            static_cast<unsigned long long>(h.traversal.backwardCrossings), static_cast<unsigned long long>(h.traversal.forwardCrossings),
            h.traversal.historyDepth, h.traversal.maximumHistoryDepth, (f.faulted || h.faulted) ? 1U : 0U);
    }
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM128] event=LOG_FORMAT_ERROR\n");
}

bool NCManager::AuthorizeEDM128SameThread(const NCBlock& block, int gcode,
    const std::array<double, 2U>& center) noexcept
{
    if (m_edm94.gapWorkTime || m_edm94.gapPhysicalAd) return AuthorizeEDM129SameThread(block, gcode, center);
    auto& s = m_edm94; auto& c = m_edm116;
    if (!s.gapHistoryMode || !s.gapHistoryServo || s.gapModeFlush || !s.gapPhysicalCurve ||
        !s.next || s.next > s.moves || !s.latched || !s.ready || s.armPending || s.cancelled ||
        s.resetSeen || s.resetCommitted || s.finishPending || s.complete || s.releaseRequested || s.rtFaultReported)
        return false;
    const unsigned step = s.next - 1U;
    const int pc = GetActiveDispatchPC(); const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    const auto& policy = c.policies[step];
    EDM127::Selection expected = m_edm127.current; double word = 0.0;
    if (!authored || pc != s.pc[step] || authored->parsedBlock.has('B') != m_edm127.authoredInline[step] ||
        (m_edm127.authoredInline[step] &&
            (!E94Number(authored->parsedBlock.expression('B'), word) ||
                !EDM127::Select(m_edm127.current, word, pc, m_currentExecutingBlockDispatchId, expected))) ||
        !EDM127::Matches(expected, policy) || !EDM127::Matches(m_edm127.pending, policy) ||
        !c.pending || c.selected != step + 1U || c.consumed != step || c.acknowledgedRows != step ||
        !EDM116::PolicyMatchesBinding(policy, step, pc, m_currentExecutingBlockDispatchId) ||
        policy.fixture != s.gapModeFixture || policy.mode != m_edm127.expectedMode[step] ||
        policy.modeOriginPC != m_edm127.expectedOriginPC[step] ||
        policy.modeOriginRevision != m_edm127.expectedOriginRevision[step] ||
        !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
        (step && (!EDM116::PolicyFollowsCompleted(policy, c.completedPolicies[step - 1U], c.completedSources[step - 1U]) ||
            !EDM114::SameSource(c.completedSources[step - 1U], m_edm114.retained.source))) ||
        !IsEDM114CurrentSameThread(step) || !IsEDM126CurrentSameThread(step) || !IsEDM128CurrentSameThread(step))
        return false;
    if (!step && (!c.initialTranslationValid || !m_edm126.initialBoundaryReady ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        !IsFixedTranslationTravelCurrentSameThread() || !IsXYZCUVFeedProfileCurrentSameThread() ||
        !SameNCTranslationSnapshot(c.initialTranslation, CoordSys.GetTranslationSnapshot()) ||
        !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()))) return false;
    const auto epoch = m_motion.GetCurrentExecutionEpoch();
    if (!m_motion.AuthorizeEDM128Feed(s.session, s.run, s.cache, m_currentExecutingBlockDispatchId,
        pc, step, gcode, s.increment[step], center, block.val('F'), epoch, s.lease, s.gapTuning,
        m_edm114.retained.source, m_edm114.completionCommit, m_edm114.completionTick, policy)) return false;
    c.issuedDispatch[step] = m_currentExecutingBlockDispatchId;
    c.issuedEpoch[step] = epoch == (std::numeric_limits<std::uint32_t>::max)() ? 1ULL : epoch + 1ULL;
    c.pending = false; c.consumed = step + 1U;
    m_edm127.current = m_edm127.pending;
    if (m_edm127.authoredInline[step]) ++m_edm127.explicitRows; else ++m_edm127.inheritedRows;
    m_edm127.lastPC = pc; m_edm127.lastDispatch = m_currentExecutingBlockDispatchId;
    m_edm127.pending = EDM127::Selection{};
    LogEDM128SameThread("ROW_AUTHORIZED", step);
    return true;
}

bool NCManager::RetainEDM128CompletedSameThread(const EDM94::Feedback& guard) noexcept
{
    if (m_edm94.gapWorkTime || m_edm94.gapPhysicalAd) return RetainEDM129CompletedSameThread(guard);
    const auto& s = m_edm94; auto& c = m_edm116;
    if (!s.gapHistoryMode || !s.gapHistoryServo || !s.latched || !s.ready || s.armPending ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || !s.next || s.next > s.moves || c.pending ||
        c.selected != s.next || c.consumed != s.next || !IsEDM114CurrentSameThread(s.next) ||
        !IsEDM126CurrentSameThread(s.next) || !IsEDM128CurrentSameThread(s.next)) return false;
    if (c.acknowledgedRows == s.next)
        return EDM114::SameSource(c.completedSources[s.next - 1U], m_edm114.retained.source) &&
            EDM116::SamePolicy(c.completedPolicies[s.next - 1U], c.policies[s.next - 1U]);
    const unsigned step = s.next - 1U;
    if (c.acknowledgedRows + 1U != s.next || guard.identity.session != s.session ||
        guard.identity.epoch != s.armEpoch || guard.identity.ownerGeneration != s.lease.generation ||
        guard.currentIdentity.session != s.session || guard.currentIdentity.epoch != m_edm114.retained.source.epoch ||
        guard.currentIdentity.ownerGeneration != s.lease.generation || guard.lastTick != m_edm114.completionTick ||
        guard.lastNowUs < m_edm114.retained.lastMappingUs || !guard.active || !guard.armAccepted ||
        !guard.armedReady || guard.faulted || guard.cancelled || guard.forceZero || guard.handoffZero ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        !EDM116::PolicyMatchesSource(c.policies[step], m_edm114.retained.source)) return false;
    c.completedPolicies[step] = c.policies[step]; c.completedSources[step] = m_edm114.retained.source;
    c.acknowledgedRows = s.next;
    LogEDM128SameThread("COORDINATOR_ROW_RETAINED", EDM114::RequiredRows);
    return true;
}

bool NCManager::IsEDM126CurrentSameThread(unsigned completed) const noexcept
{
    const auto& s = m_edm94;
    EDM126::MotionFeedback f{};
    const bool combined = s.gapHistoryMode;
    const unsigned fixture = E149HistoryFixture(s.gapHistoryArm);
    const bool profile = combined ?
        (s.gapWorkTime ? (E130TimedArm(s.gapHistoryArm) && s.gapPhysicalAd == E149PhysicalExpected(s.gapHistoryArm)) :
            s.gapPhysicalAd ? (s.gapHistoryArm == 170U || s.gapHistoryArm == 172U) :
            (s.gapHistoryArm == 166U || s.gapHistoryArm == 168U)) &&
            s.gapModeFixture == E149ModalFixture(fixture) &&
            EDM107::ValidFlushProfile(s.gapFlushParameters) :
        !s.gapPhysicalAd && (s.gapHistoryArm == 158U || s.gapHistoryArm == 160U) &&
            s.gapModeFixture == (fixture ? EDM116::Fixture::ArcPairComplement124 : EDM116::Fixture::ArcPair124) &&
            EDM107::IsDefaultFlushProfile(s.gapFlushParameters);
    if (!s.gapHistoryServo || !s.gapPhysicalCurve || !s.gapSegmentProfile || !s.gapGainProfile ||
        s.gapBoundaryProfile || s.gapNoiseProfile || s.gapBurstProfile || s.gapContinuousProfile ||
        s.gapEnduranceProfile || s.gapPairedProfile || s.gapFlushProfile || s.gapFlushParameterized ||
        s.gapCommonPath || s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
        s.gapCurveB2 || s.gapModeFlush || s.gapModeInterruption || s.gapModeInterruptionArm || s.gapFlushInterruptArm ||
        s.gapTuning.positiveGain != 8.0 || s.gapTuning.negativeGain != 1.0 || s.gapTuning.deadbandV != 1.0 ||
        !s.latched || s.armPending || !s.everArmed ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || completed > s.moves ||
        !profile ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_motion.ReadEDM126Feedback(f) || !f.enabled || f.physicalAd != s.gapPhysicalAd || f.faulted || f.reason != EDM126::Reason::None ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        (!s.complete && f.liveEpoch != m_motion.GetCurrentExecutionEpoch()) || f.fixture != fixture ||
        f.authorizedRows != completed || f.completedRows != completed || f.rowActive ||
        f.rowCompleted != (completed != 0U) || !f.tick || !f.nowUs ||
        !f.sourceReady || f.sourceReason != EDM88::Reason::None || !f.sourceSequence || !f.sourceCapturedUs ||
        f.nowUs < f.sourceCapturedUs || f.nowUs - f.sourceCapturedUs > EDM126::MaximumSourceAgeUs ||
        !std::isfinite(f.voltage) || !std::isfinite(f.targetRatio)) return false;
    const auto& t = f.traversal;
    if (t.faulted || t.reason != EDM126::Reason::None || t.stopRequested || t.stopped ||
        t.historyCount > EDM126::HistoryCapacity || t.historyDepth || t.maximumHistoryDepth > EDM126::HistoryCapacity ||
        t.forwardCrossings > t.backwardCrossings || t.velocityMmSec != 0.0 ||
        !std::isfinite(t.localMm) || !std::isfinite(t.lengthMm) || !std::isfinite(t.stationMm)) return false;
    if (!completed)
        return EDM126::SameSource(f.authorizedSource, EDM126::Source{}) &&
            EDM126::SameSource(t.authorizedSource, EDM126::Source{}) &&
            EDM126::SameSource(t.cursorSource, EDM126::Source{}) && !t.historyCount && !t.completedRows &&
            !t.maximumHistoryDepth && !t.backwardCrossings && !t.forwardCrossings &&
            !f.lastMappingTick && !f.lastMappingUs && (!combined || IsEDM128CurrentSameThread(0U));
    EDM114::Feedback row{};
    if (!m_motion.ReadEDM114Feedback(row) || !(combined ? E128Counts(row, completed, s.gapModeFixture, s.gapWorkTime, &s.authoredGCode, s.moves) : E126Counts(row, completed, s.gapModeFixture)) ||
        row.session != s.session || row.armEpoch != s.armEpoch || row.ownerGeneration != s.lease.generation ||
        !EDM126::ValidSource(f.authorizedSource) || !EDM114::ValidSource(row.source) ||
        !EDM109::SameScope(f.authorizedSource.scope, row.source.scope) ||
        f.authorizedSource.ordinal != completed - 1U || f.authorizedSource.sourcePC != s.pc[completed - 1U] ||
        f.authorizedSource.dispatch != row.source.dispatch || f.authorizedSource.epoch != row.source.epoch ||
        f.authorizedSource.segment != row.source.segment || row.source.step != completed - 1U ||
        row.source.sourcePC != f.authorizedSource.sourcePC || row.source.scope.session != s.session ||
        row.source.scope.run != s.run || row.source.scope.cache != s.cache ||
        row.source.scope.ownerGeneration != s.lease.generation ||
        !EDM126::SameSource(f.authorizedSource, t.authorizedSource) ||
        !EDM126::SameSource(t.cursorSource, f.authorizedSource) ||
        !f.lastMappingTick || f.lastMappingTick != row.lastMappingTick ||
        !f.lastMappingUs || f.lastMappingUs != row.lastMappingUs ||
        f.lastMappingTick > f.tick || f.lastMappingUs > f.nowUs ||
        f.authorizedSource.epoch != f.liveEpoch ||
        t.completedRows != completed - 1U || t.historyCount != completed - 1U ||
        t.maximumHistoryDepth > completed - 1U || !t.active || t.awaitingCurrent || t.awaitingSeam ||
        !t.completeReady || t.tick != f.lastMappingTick || t.nowUs != f.lastMappingUs ||
        t.localMm != t.lengthMm || t.lengthMm <= 0.0 || !t.atCurrentEnd ||
        !EDM109::SamePulse(t.positionPulse, row.endPulse)) return false;
    std::uint64_t configTag = 1469598103934665603ULL, mappingTag = configTag;
    for (unsigned a = 0U; a < 6U; ++a)
    {
        const auto& frozen = s.feedback.config.axis[a];
        if (!EDM109::SameDouble(row.pulsePerNative[a], frozen.pulsePerUnit)) return false;
        configTag = (configTag ^ frozen.configFingerprint) * 1099511628211ULL;
        mappingTag = (mappingTag ^ frozen.mappingFingerprint) * 1099511628211ULL;
    }
    if (row.source.scope.configTag != (configTag ? configTag : 1ULL) ||
        row.source.scope.mappingTag != (mappingTag ? mappingTag : 1ULL) ||
        (!s.complete && (!CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
            row.source.scope.translationGeneration != CoordSys.GetTranslationSnapshot().generation))) return false;
    // M30 deliberately changes the live epoch and releases the frozen frame.
    // Only the already acknowledged sixteen-row receipt can survive retirement.
    if (s.complete && (completed != s.moves || m_edm114.acknowledgedRows != completed ||
        !m_edm114.completionCommit || m_edm114.completionTick < f.lastMappingTick ||
        !E114SameFeedback(row, m_edm114.retained))) return false;
    for (unsigned a = 0U; a < 6U; ++a)
        if (t.velocityPps[a] != 0.0) return false;
    // EDM128 verifies its independent coordinator and exact combined seam schedule.
    if (combined) return IsEDM128CurrentSameThread(completed);
    // EDM126 requests two history seams on rows 2, 4 and 8. These are observed
    // crossings, never an NC instruction replay or a motion schedule.
    const std::uint64_t expectedCrossings = 2ULL * ((completed > 2U ? 1U : 0U) +
        (completed > 4U ? 1U : 0U) + (completed > 8U ? 1U : 0U));
    return t.backwardCrossings == expectedCrossings && t.forwardCrossings == expectedCrossings &&
        t.maximumHistoryDepth == (expectedCrossings ? 2U : 0U);
}

void NCManager::LogEDM126SameThread(const char* event) const noexcept
{
    const auto& s = m_edm94;
    if (!s.gapHistoryServo) return;
    if (s.gapHistoryMode) { LogEDM128SameThread(event, EDM114::RequiredRows); return; }
    EDM126::MotionFeedback f{};
    const bool read = m_motion.ReadEDM126Feedback(f);
    char record[481]{};
    const int length = std::snprintf(record, sizeof(record),
        "[EDM126] event=%.32s session=%llu armP=%u tick=%llu read=%u authorized=%u completed=%u back=%llu forward=%llu depth=%u maxDepth=%u fault=%u reason=%u dispatchReplay=0 flush=0 nativeArc=G17_XY sixAxis=G01 source=RT_SYNTHETIC dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), s.gapHistoryArm,
        static_cast<unsigned long long>(f.tick), read ? 1U : 0U, f.authorizedRows, f.completedRows,
        static_cast<unsigned long long>(f.traversal.backwardCrossings), static_cast<unsigned long long>(f.traversal.forwardCrossings),
        f.traversal.historyDepth, f.traversal.maximumHistoryDepth, f.faulted ? 1U : 0U, static_cast<unsigned>(f.reason));
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM126] event=LOG_FORMAT_ERROR part=HISTORY\n");
}

bool NCManager::WaitEDM126InitialBoundarySameThread() noexcept
{
    auto& s = m_edm94; auto& c = m_edm126;
    const auto epoch = m_motion.GetCurrentExecutionEpoch();
    if (!s.gapHistoryServo || !s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending || !s.everArmed ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || s.next ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        epoch != s.armEpoch || !m_motion.IsMotionOwnerLeaseCurrent(s.lease))
    { RejectEDM94SameThread("EDM126_INITIAL_SCOPE", m_gapDryRun.sourceLine); return false; }
    if (s.gapWorkTime && EDM149::IsTestArm(s.gapHistoryArm) && !s.gapPhysicalAd && !IsEDM129SourceCurrentSameThread()) return false;
    if (s.gapPhysicalAd && !m_edm129.sourceBound)
    {
        EDM129::MotionFeedback source{};
        if (!IsEDM129SourceCurrentSameThread() || !m_motion.ReadEDM129Feedback(source) ||
            !source.enabled || source.faulted || !source.sourceReady || !source.active ||
            source.session != s.session || source.armEpoch != s.armEpoch ||
            source.liveEpoch != epoch || source.ownerGeneration != s.lease.generation ||
            !EDM129::ValidBinding(source.binding)) return false;
        m_edm129.sourceBinding = source.binding; m_edm129.sourceBound = true;
        LogEDM129SameThread("PHYSICAL_SOURCE_BOUND", EDM114::RequiredRows);
    }
    EDM94::Feedback guard{};
    const bool read = m_motion.ReadEDM94Feedback(guard);
    unsigned flags = 0U;
    // Publication may be temporarily busy; keep the same bounded wait rather
    // than treating a missing observer read as a new authorization.
    if (!IsEDM114CurrentSameThread(0U) || !IsEDM126CurrentSameThread(0U) ||
        (s.gapHistoryMode && !IsEDM128CurrentSameThread(0U))) flags |= 128U;
    const bool identity = read && guard.identity.session == s.session && guard.identity.epoch == s.armEpoch &&
        guard.identity.ownerGeneration == s.lease.generation && guard.currentIdentity.session == s.session &&
        guard.currentIdentity.epoch == epoch && guard.currentIdentity.ownerGeneration == s.lease.generation;
    if (!identity) flags |= 1U;
    if (!read || !guard.active || !guard.armAccepted || !guard.armedReady || guard.forceZero || guard.handoffZero ||
        guard.faulted || guard.cancelled || guard.finishRequested || guard.released) flags |= 2U;
    const bool fresh = identity && guard.lastTick != 0ULL && guard.lastTick >= s.lastTick &&
        guard.lastTick > c.initialBoundaryTick && guard.lastNowUs != 0ULL && guard.lastNowUs > c.initialBoundaryUs;
    if (!fresh) flags |= 4U;
    if (!read || guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask) flags |= 8U;
    if (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() || m_motion.GetCommandReplaySize() ||
        m_motion.GetAxisCommandMailboxDepth() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(epoch, s.lease)) flags |= 16U;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& frozen = s.feedback.config.axis[axis]; const auto& current = guard.config.axis[axis];
        const auto& observed = guard.axis[axis];
        const double maxWindow = (axis < 3U ? 0.001 : 0.005) * frozen.pulsePerUnit;
        const bool config = read && std::isfinite(frozen.pulsePerUnit) && frozen.pulsePerUnit > 0.0 &&
            std::isfinite(frozen.followingWindowPulse) && frozen.followingWindowPulse > 0.0 &&
            std::isfinite(maxWindow) && frozen.followingWindowPulse <= maxWindow &&
            current.pulsePerUnit == frozen.pulsePerUnit && current.followingWindowPulse == frozen.followingWindowPulse &&
            current.commandAnchor == frozen.commandAnchor && current.actualAnchor == frozen.actualAnchor &&
            current.hardwareSign == frozen.hardwareSign && current.configFingerprint == frozen.configFingerprint &&
            current.mappingFingerprint == frozen.mappingFingerprint;
        if (!config || !std::isfinite(observed.currentCommandPulse) ||
            !EDM109::SameDouble(observed.currentCommandPulse, frozen.commandAnchor)) flags |= 32U;
        if (!config || !std::isfinite(observed.currentActualPulse) ||
            std::fabs(observed.currentActualPulse - frozen.commandAnchor) > frozen.followingWindowPulse) flags |= 64U;
    }
    if (fresh) { c.initialBoundaryTick = guard.lastTick; c.initialBoundaryUs = guard.lastNowUs; }
    const bool ready = flags == 0U;
    if (!c.initialBoundaryLogged || ready)
    {
        RtPrintf(E137FlushStageArm(s.gapHistoryArm) ? "[EDM137] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : E136CurveArm(s.gapHistoryArm) ? "[EDM136] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : E135ReferenceArm(s.gapHistoryArm) ? "[EDM135] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : E134MachiningArm(s.gapHistoryArm) ? "[EDM134] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : E133DeepArm(s.gapHistoryArm) ? "[EDM133] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : E132HeightArm(s.gapHistoryArm) ? "[EDM132] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : E131SpeedArm(s.gapHistoryArm) ? "[EDM131] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : s.gapWorkTime ? "[EDM130] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" : s.gapPhysicalAd ? "[EDM129] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" :
            s.gapHistoryMode ? "[EDM128] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n" :
            "[EDM126] event=INITIAL_BOUNDARY_%s session=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u nextMotionSubmitted=0\n",
            ready ? "READY" : "WAIT", static_cast<unsigned long long>(s.session),
            static_cast<unsigned long long>(guard.lastTick), flags, guard.boundaryReadyCycles, EDM94::BoundaryReadyCycles);
        c.initialBoundaryLogged = true;
    }
    if (ready) { c.initialTranslation = CoordSys.GetTranslationSnapshot(); c.initialTranslationValid = true; }
    c.initialBoundaryReady = ready;
    return ready;
}

bool NCManager::WaitEDM116InitialBoundarySameThread() noexcept
{
    auto& s = m_edm94; auto& c = m_edm116;
    const auto epoch = m_motion.GetCurrentExecutionEpoch();
    if (!s.gapModeFlush || !s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending || !s.everArmed ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || s.next || c.selected || c.consumed || c.pending ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        epoch != s.armEpoch || !m_motion.IsMotionOwnerLeaseCurrent(s.lease))
    { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_INITIAL_SCOPE" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_INITIAL_SCOPE" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_INITIAL_SCOPE" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_INITIAL_SCOPE" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_INITIAL_SCOPE" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_INITIAL_SCOPE" : m_edm94.gapModeInterruption ? "EDM118_INITIAL_SCOPE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_INITIAL_SCOPE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_INITIAL_SCOPE" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_INITIAL_SCOPE" : "EDM116_INITIAL_SCOPE"), m_gapDryRun.sourceLine); return false; }
    // P129/P132 remains the waiting block. A qualified synthetic source alone
    // cannot authorize the first P131 before the genuine RT incoming dwell.
    EDM94::Feedback guard{};
    const bool read = m_motion.ReadEDM94Feedback(guard);
    unsigned flags = 0U;
    // Publication may be temporarily busy; keep the same bounded wait rather
    // than treating a missing observer read as a new authorization.
    if (!IsEDM114CurrentSameThread(0U) || !IsEDM116CurrentSameThread(0U)) flags |= 128U;
    const bool identity = read && guard.identity.session == s.session && guard.identity.epoch == s.armEpoch &&
        guard.identity.ownerGeneration == s.lease.generation && guard.currentIdentity.session == s.session &&
        guard.currentIdentity.epoch == epoch && guard.currentIdentity.ownerGeneration == s.lease.generation;
    if (!identity) flags |= 1U;
    if (!read || !guard.active || !guard.armAccepted || !guard.armedReady || guard.forceZero || guard.handoffZero ||
        guard.faulted || guard.cancelled || guard.finishRequested || guard.released) flags |= 2U;
    const bool fresh = identity && guard.lastTick != 0ULL && guard.lastTick >= s.lastTick &&
        guard.lastTick > c.initialBoundaryTick && guard.lastNowUs != 0ULL && guard.lastNowUs > c.initialBoundaryUs;
    if (!fresh) flags |= 4U;
    if (!read || guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask) flags |= 8U;
    if (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() || m_motion.GetCommandReplaySize() ||
        m_motion.GetAxisCommandMailboxDepth() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(epoch, s.lease)) flags |= 16U;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& frozen = s.feedback.config.axis[axis]; const auto& current = guard.config.axis[axis];
        const auto& observed = guard.axis[axis];
        const double maxWindow = (axis < 3U ? 0.001 : 0.005) * frozen.pulsePerUnit;
        const bool config = read && std::isfinite(frozen.pulsePerUnit) && frozen.pulsePerUnit > 0.0 &&
            std::isfinite(frozen.followingWindowPulse) && frozen.followingWindowPulse > 0.0 &&
            std::isfinite(maxWindow) && frozen.followingWindowPulse <= maxWindow &&
            current.pulsePerUnit == frozen.pulsePerUnit && current.followingWindowPulse == frozen.followingWindowPulse &&
            current.commandAnchor == frozen.commandAnchor && current.actualAnchor == frozen.actualAnchor &&
            current.hardwareSign == frozen.hardwareSign && current.configFingerprint == frozen.configFingerprint &&
            current.mappingFingerprint == frozen.mappingFingerprint;
        if (!config || !std::isfinite(observed.currentCommandPulse) ||
            !EDM109::SameDouble(observed.currentCommandPulse, frozen.commandAnchor)) flags |= 32U;
        if (!config || !std::isfinite(observed.currentActualPulse) ||
            std::fabs(observed.currentActualPulse - frozen.commandAnchor) > frozen.followingWindowPulse) flags |= 64U;
    }
    if (fresh) { c.initialBoundaryTick = guard.lastTick; c.initialBoundaryUs = guard.lastNowUs; }
    const bool ready = flags == 0U;
    if (!c.initialBoundaryLogged || ready)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE\n" : s.gapModeInterruption ? "[EDM118] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE\n" : "[EDM116] event=INITIAL_BOUNDARY_%s build=FIX1 session=%llu epoch=%llu tick=%llu flags=%02X rtReadyCycles=%u required=%u rtNotReadyMask=%08X frameFrozen=%u selected=0 consumed=0 nextMotionSubmitted=0 dischargeCommand=NONE\n"),
            ready ? "READY" : "WAIT", static_cast<unsigned long long>(s.session),
            static_cast<unsigned long long>(epoch), static_cast<unsigned long long>(guard.lastTick), flags,
            guard.boundaryReadyCycles, EDM94::BoundaryReadyCycles, guard.boundaryNotReadyMask,
            CoordSys.IsTranslationRunFrozen() ? 1U : 0U);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("%s", (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY\n" : s.gapModeInterruption ? "[EDM118] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY\n" : "[EDM116] event=LOG_FORMAT_ERROR build=FIX1 part=INITIAL_BOUNDARY\n"));
        c.initialBoundaryLogged = true;
    }
    c.initialBoundaryReady = ready;
    return ready;
}

namespace
{
    void E146LogBoundary(const char* phase, std::uint64_t session, unsigned step, int pc,
        std::uint64_t dispatch, bool ready, unsigned attempts,
        const EDM146::Failure& first, const EDM146::Failure& last) noexcept
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM146] event=BOUNDARY_OBSERVATION phase=%s result=%s session=%llu row=%u PC=%d dispatch=%llu attempts=%u first=%s last=%s tickA=%llu tickB=%llu nowA=%llu nowB=%llu authority=NONE\n",
            phase, ready ? "RECOVERED" : "REJECT", static_cast<unsigned long long>(session), step, pc,
            static_cast<unsigned long long>(dispatch), attempts, EDM146::Name(first.gate), EDM146::Name(last.gate),
            static_cast<unsigned long long>(first.tickA), static_cast<unsigned long long>(first.tickB),
            static_cast<unsigned long long>(first.nowA), static_cast<unsigned long long>(first.nowB));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM146] event=LOG_FORMAT_ERROR part=BOUNDARY_OBSERVATION\n");
    }
}

bool NCManager::IsEDM127SelectionBoundarySameThread(int pc) noexcept
{
    const auto check = [this, pc](EDM146::Failure& failure) noexcept {
        return CheckEDM146SelectionBoundarySameThread(pc, failure);
    };
    const auto report = [this, pc](bool ready, unsigned attempts,
        const EDM146::Failure& first, const EDM146::Failure& last) noexcept {
        E146LogBoundary("SELECTION", m_edm94.session, m_edm94.next, pc,
            m_currentExecutingBlockDispatchId, ready, attempts, first, last);
    };
    // Only the physical timed-history lane may retry explicitly unavailable
    // or independently valid but time-skewed observations. Every attempt
    // starts the full original live proof; no sleep, dispatch or retained permit.
    if (m_edm94.gapWorkTime) return EDM146::FreshBoundary(check, report);
    EDM146::Failure failure{};
    const bool ready = check(failure);
    if (!ready) report(false, 1U, failure, failure);
    return ready;
}

bool NCManager::CheckEDM146SelectionBoundarySameThread(int pc, EDM146::Failure& observation) noexcept
{
    (void)EDM146::Fail(&observation, EDM146::Gate::BoundaryState);
    const auto& s = m_edm94; const auto& c = m_edm116;
    const unsigned step = s.next;
    if (!EDM116::IsModalFixture(s.gapModeFixture) || (!s.gapModeFlush && !s.gapHistoryMode) || !s.gapPhysicalCurve ||
        !s.latched || !s.ready || s.armPending || !s.everArmed || s.cancelled || s.resetSeen ||
        s.resetCommitted || s.finishPending || s.complete || s.releaseRequested || s.rtFaultReported ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY || m_isSingleBlockEnabled ||
        !m_macroStack.empty() || m_isG66Active || Homing.IsActive() || Close_System_Com_flag ||
        !m_externalReadyInterlock || AlarmManager::GetInstance().HasAlarm() ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        step >= s.moves || step >= EDM114::RequiredRows || pc > s.pc[step] || pc <= m_edm127.lastPC ||
        !m_currentExecutingBlockDispatchId || m_currentExecutingBlockDispatchId <= m_edm127.lastDispatch ||
        !EDM127::Valid(m_edm127.current) || c.pending || c.selected != step || c.consumed != step ||
        (s.gapHistoryMode && c.acknowledgedRows != step)) return false;
    if (!IsEDM114CurrentSameThread(step)) return EDM146::Fail(&observation, EDM146::Gate::PathProof);
    (void)EDM146::Fail(&observation, EDM146::Gate::HistoryProof);
    if (!(s.gapWorkTime ? IsEDM130CurrentSameThread(step, &observation) :
        s.gapHistoryMode ? IsEDM128CurrentSameThread(step) : IsEDM116CurrentSameThread(step))) return false;
    (void)EDM146::Fail(&observation, EDM146::Gate::BoundaryState);
    if (m_pathFeed.pending || m_pathArc.pending || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() || m_motion.GetCommandReplaySize() || m_motion.GetAxisCommandMailboxDepth() ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    (void)EDM146::Fail(&observation, EDM146::Gate::Selection);
    // Validate the committed selection before an inline B can replace it.
    // The latest executed block is either the completed row, the arm, or an
    // authenticated standalone B; none may lend a different origin/revision.
    if (step && m_edm127.lastPC == s.pc[step - 1U])
    {
        if (!EDM127::Matches(m_edm127.current, c.completedPolicies[step - 1U])) return false;
    }
    else if (m_edm127.lastPC == s.armPC)
    {
        if (step || m_edm127.current.mode != EDM116::Mode::B0 || m_edm127.current.pc != s.armPC ||
            m_edm127.current.dispatch != s.armDispatch || m_edm127.current.revision != 1ULL) return false;
    }
    else
    {
        const auto* selection = GetBaseProgramCache().TryGetLine(m_edm127.lastPC);
        EDM116::Mode selected{};
        if (!selection || !E127Standalone(selection->parsedBlock, selected) || selected != m_edm127.current.mode ||
            m_edm127.current.pc != m_edm127.lastPC || m_edm127.current.dispatch != m_edm127.lastDispatch ||
            (step && m_edm127.current.pc <= s.pc[step - 1U])) return false;
    }
    const auto source = CoordSys.GetTranslationSnapshot();
    const bool translation = CoordSys.IsTranslationRunFrozen() ?
        CoordSys.IsTranslationRunCurrent() && IsFixedTranslationTravelCurrentSameThread() &&
            IsXYZCUVFeedProfileCurrentSameThread() && m_motion.MatchesNCTranslation(source) &&
            (!step || source.generation == m_edm114.retained.source.scope.translationGeneration) :
        step == 0U && (s.gapHistoryMode ? m_edm126.initialBoundaryReady : c.initialBoundaryReady) && CoordSys.IsTranslationRunBound() &&
            CoordSys.IsTranslationAxisIdentityCurrent() && IsNCTranslationSnapshotValid(source) &&
            source.runToken == s.run && IsNCXYZCUVFeedNeutralFrame(source) && source.distanceMode == 91 &&
            m_motion.GetActiveTranslationGeneration() == 0ULL;
    EDM94::Feedback guard{}; const auto epoch = m_motion.GetCurrentExecutionEpoch();
    if (!translation || (!step && !(s.gapHistoryMode ? m_edm126.initialBoundaryReady : c.initialBoundaryReady)))
        return EDM146::Fail(&observation, EDM146::Gate::Translation);
    if (!m_motion.ReadEDM94Feedback(guard)) return EDM146::Fail(&observation, EDM146::Gate::BoundaryRead);
    (void)EDM146::Fail(&observation, EDM146::Gate::BoundaryProof, guard.lastTick, s.lastTick,
        guard.lastNowUs, 0ULL);
    if (guard.identity.session != s.session ||
        guard.identity.epoch != s.armEpoch || guard.identity.ownerGeneration != s.lease.generation ||
        guard.currentIdentity.session != s.session || guard.currentIdentity.epoch != epoch ||
        guard.currentIdentity.ownerGeneration != s.lease.generation || !guard.active || !guard.armAccepted ||
        !guard.armedReady || guard.forceZero || guard.handoffZero || guard.faulted || guard.cancelled ||
        guard.finishRequested || guard.released || !guard.lastTick || !guard.lastNowUs ||
        guard.lastTick < s.lastTick || guard.lastTick < m_edm114.completionTick ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(epoch, s.lease)) return false;
    (void)EDM146::Fail(&observation, EDM146::Gate::AxisProof, guard.lastTick, s.lastTick,
        guard.lastNowUs, 0ULL);
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& frozen = s.feedback.config.axis[axis]; const auto& current = guard.config.axis[axis];
        const double target = step ? m_edm114.retained.endPulse[axis] : frozen.commandAnchor;
        if (current.configFingerprint != frozen.configFingerprint || current.mappingFingerprint != frozen.mappingFingerprint ||
            !EDM109::SameDouble(current.pulsePerUnit, frozen.pulsePerUnit) ||
            !EDM109::SameDouble(guard.axis[axis].currentCommandPulse, target)) return false;
    }
    return true;
}

bool NCManager::PrepareEDM127RowSameThread(const NCBlock& block) noexcept
{
    auto& s = m_edm94; auto& c = m_edm116; const unsigned step = s.next;
    const int pc = GetActiveDispatchPC(); const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    const auto reject = [&](EDM146::Gate gate) noexcept {
        EDM146::Failure failure{}; (void)EDM146::Fail(&failure, gate);
        E146LogBoundary("ROW", s.session, step, pc, m_currentExecutingBlockDispatchId,
            false, 1U, failure, failure);
        return false;
    };
    if (!IsEDM127SelectionBoundarySameThread(pc)) return false;
    if (pc != s.pc[step] || !authored ||
        !(E94Matches(authored->parsedBlock, block) || MatchesEDM141MotionSameThread(authored->parsedBlock, block, pc)) || block.has('B') != m_edm127.authoredInline[step]) return reject(EDM146::Gate::RowBinding);
    NCParsedBlock geometry = authored->parsedBlock;
    if (E141MotionArm(s.gapHistoryArm) &&
        !EDM141::MakeLiteral(authored->parsedBlock, m_edm141.rows[step], geometry)) return reject(EDM146::Gate::Geometry);
    geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
    std::array<double, 6U> delta{}; double feed = 0.0;
    if (!E114Move(geometry, step, delta, feed, s.gapModeFixture, E148AuthoredLineArm(s.gapHistoryArm)) ||
        delta != s.increment[step] || feed != s.feed[step] ||
        (EDM116::IsAuthoredOrderFixture(s.gapModeFixture) && block.gCode != s.authoredGCode[step])) return reject(EDM146::Gate::Geometry);
    EDM127::Selection candidate = m_edm127.current;
    if (block.has('B') && !EDM127::Select(m_edm127.current, block.val('B'), pc,
        m_currentExecutingBlockDispatchId, candidate)) return reject(EDM146::Gate::ModeOrigin);
    if (candidate.mode != m_edm127.expectedMode[step] || candidate.pc != m_edm127.expectedOriginPC[step] ||
        candidate.revision != m_edm127.expectedOriginRevision[step]) return reject(EDM146::Gate::ModeOrigin);
    EDM116::Policy policy{}; policy.mode = candidate.mode; policy.fixture = s.gapModeFixture;
    policy.profile = s.gapFlushParameters; policy.selectorPC = pc;
    policy.selectorDispatch = m_currentExecutingBlockDispatchId; policy.revision = step + 1ULL;
    policy.modeOriginPC = candidate.pc; policy.modeOriginDispatch = candidate.dispatch;
    policy.modeOriginRevision = candidate.revision;
    if (!EDM116::PolicyMatchesBinding(policy, step, pc, m_currentExecutingBlockDispatchId) ||
        (step && !EDM116::PolicyFollowsCompleted(policy, c.policies[step - 1U], c.completedSources[step - 1U]))) return reject(EDM146::Gate::Policy);
    if (!step)
    { c.initialTranslation = CoordSys.GetTranslationSnapshot(); c.initialTranslationValid = true; }
    m_edm127.pending = candidate;
    c.policies[step] = policy; c.selected = step + 1U; c.pending = true;
    return true;
}

bool NCManager::ProcessEDM127BlockSameThread(const NCBlock& input, NCBlock& geometry, bool& settingOnly) noexcept
{
    settingOnly = false; geometry = input;
    const auto& s = m_edm94;
    if (!s.latched || !EDM116::IsModalFixture(s.gapModeFixture)) return true;
    const int pc = GetActiveDispatchPC(); const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    if (!authored || !(E94Matches(authored->parsedBlock, input) ||
        MatchesEDM141MotionSameThread(authored->parsedBlock, input, pc))) return false;
    EDM116::Mode mode{};
    if (E127Standalone(authored->parsedBlock, mode))
    {
        EDM127::Selection candidate{};
        if (!IsEDM127SelectionBoundarySameThread(pc) || !EDM127::Select(m_edm127.current,
            input.val('B'), pc, m_currentExecutingBlockDispatchId, candidate)) return false;
        m_edm127.current = candidate; m_edm127.lastPC = pc;
        m_edm127.lastDispatch = m_currentExecutingBlockDispatchId; ++m_edm127.standaloneSelections;
        settingOnly = true; LogEDM127SameThread("B_SELECTED", EDM114::RequiredRows); return true;
    }
    if (input.hasG && input.gCount == 1 && input.gCode >= 1 && input.gCode <= 3)
    {
        if (!s.next || s.next > s.moves || !m_edm116.pending || m_edm116.selected != s.next ||
            m_edm116.consumed + 1U != s.next || pc != s.pc[s.next - 1U] ||
            !EDM116::PolicyMatchesBinding(m_edm116.policies[s.next - 1U], s.next - 1U, pc, m_currentExecutingBlockDispatchId) ||
            !EDM127::Matches(m_edm127.pending, m_edm116.policies[s.next - 1U])) return false;
        geometry.hasParam['B'-'A'] = false; geometry.param['B'-'A'] = 0.0;
        return true;
    }
    return !input.has('B');
}

void NCManager::LogEDM127SameThread(const char* event, unsigned step) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm116;
    if (!EDM116::IsModalFixture(s.gapModeFixture)) return;
    if (s.gapHistoryMode) { LogEDM128SameThread(event, step); return; }
    char record[481]{}; int length = 0;
    if (step < EDM114::RequiredRows)
    {
        const auto& p = c.policies[step];
        length = std::snprintf(record, sizeof(record),
            "[EDM127] event=%.24s session=%llu row=%u G=%d B=%u input=%s sourcePC=%d dispatch=%llu originPC=%d originDispatch=%llu originRevision=%llu rowRevision=%llu replay=0 dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), step,
            EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]), static_cast<unsigned>(p.mode),
            m_edm127.authoredInline[step] ? "EXPLICIT" : "INHERITED", p.selectorPC,
            static_cast<unsigned long long>(p.selectorDispatch), p.modeOriginPC,
            static_cast<unsigned long long>(p.modeOriginDispatch), static_cast<unsigned long long>(p.modeOriginRevision),
            static_cast<unsigned long long>(p.revision));
    }
    else
    {
        const auto& f = c.feedback;
        length = std::snprintf(record, sizeof(record),
            "[EDM127] event=%.24s session=%llu armP=%u rows=%u/%u explicitRows=%u inheritedRows=%u standaloneB=%u effectiveB=%u originPC=%d originRevision=%llu B0=%u B1=%u B2=%u events=%u/%u dwells=%u line=%u arc=%u default=B0 gap=RT_SYNTHETIC nativeArc=G17_XY sixAxis=G01 dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), E116ArmP(s.gapModeFixture),
            c.consumed,c.acknowledgedRows,m_edm127.explicitRows,m_edm127.inheritedRows,m_edm127.standaloneSelections,
            static_cast<unsigned>(m_edm127.current.mode),m_edm127.current.pc,
            static_cast<unsigned long long>(m_edm127.current.revision),f.modeCompleted[0],f.modeCompleted[1],f.modeCompleted[2],
            f.eventsStarted,f.eventsCompleted,f.completedDwellsTotal,m_edm114.retained.lineRows,m_edm114.retained.arcRows);
    }
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s",record);
    else RtPrintf("[EDM127] event=LOG_FORMAT_ERROR\n");
}

bool NCManager::IsEDM116SelectorCurrentSameThread(const NCBlock& block) noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm116;
    const unsigned step = s.next;
    const int pc = GetActiveDispatchPC();
    const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    EDM116::Mode mode{};
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    // The first selector precedes the first G01, which alone freezes and
    // publishes the frame. Admit only this bound, current, unpublished source;
    // subsequent selectors still require the completed row's frozen frame.
    const bool translationCurrent = CoordSys.IsTranslationRunFrozen() ?
        (CoordSys.IsTranslationRunCurrent() && IsFixedTranslationTravelCurrentSameThread() &&
            IsXYZCUVFeedProfileCurrentSameThread() && m_motion.MatchesNCTranslation(source) &&
            (!step || source.generation == m_edm114.retained.source.scope.translationGeneration)) :
        (step == 0U && c.initialBoundaryReady && CoordSys.IsTranslationRunBound() &&
            CoordSys.IsTranslationAxisIdentityCurrent() && IsNCTranslationSnapshotValid(source) &&
            source.runToken == s.run && IsNCXYZCUVFeedNeutralFrame(source) && source.distanceMode == 91 &&
            m_motion.GetActiveTranslationGeneration() == 0ULL);
    return s.gapModeFlush && EDM116::ValidFixture(s.gapModeFixture) && s.gapPhysicalCurve && !s.gapCurveB2 && s.latched && s.ready &&
        !s.armPending && s.everArmed && !s.cancelled && !s.resetSeen && !s.resetCommitted &&
        !s.finishPending && !s.complete && !s.releaseRequested && !s.rtFaultReported &&
        m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY && !m_isSingleBlockEnabled &&
        !Close_System_Com_flag && m_externalReadyInterlock &&
        m_macroStack.empty() && !m_isG66Active && !Homing.IsActive() && !AlarmManager::GetInstance().HasAlarm() &&
        s.cache == GetBaseProgramCache().GetGeneration() && s.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        step < s.moves && step < EDM114::RequiredRows && pc == s.selectorPC[step] &&
        (!s.gapModeInterruption || (s.gapModeInterruptionArm == m_edm118.armP &&
            step <= (EDM125::ValidProfile(EDM125::ProfileForArm(s.gapModeInterruptionArm)) ? EDM125::TargetStep : EDM123::ValidProfile(EDM123::ProfileForArm(s.gapModeInterruptionArm)) ? EDM123::TargetStep(EDM123::ProfileForArm(s.gapModeInterruptionArm)) : EDM122::ValidProfile(EDM122::ProfileForArm(s.gapModeInterruptionArm)) ? EDM122::TargetStep(EDM122::ProfileForArm(s.gapModeInterruptionArm)) : EDM118::TargetStep))) &&
        c.selected == step && c.consumed == step && !c.pending &&
        authored && E94Matches(authored->parsedBlock, block) && E116Selector(authored->parsedBlock, mode) &&
        mode == EDM116::ExpectedMode(step) && IsGapDryRunBlockShapeValid(block) &&
        !m_pathFeed.pending && !m_pathArc.pending && m_motion.IsGroupDone() &&
        !m_motion.GetCommandIngressSize() && !m_motion.GetCommandReplaySize() && !m_motion.GetAxisCommandMailboxDepth() &&
        m_motion.IsMotionOwnerLeaseCurrent(s.lease) && !m_motion.HasPendingSafetyOrRecoveryRequests() &&
        translationCurrent && (step || c.initialBoundaryReady) && IsPathCoreLiveNativeConfigCurrentSameThread();
}

bool NCManager::CommitEDM116ModeSameThread(const NCBlock& block) noexcept
{
    auto& s = m_edm94; auto& c = m_edm116;
    if (!IsEDM116SelectorCurrentSameThread(block) || !IsEDM114CurrentSameThread(s.next) ||
        !IsEDM116CurrentSameThread(s.next)) return false;
    const unsigned step = s.next;
    EDM94::Feedback guard{};
    const auto epoch = m_motion.GetCurrentExecutionEpoch();
    if (!m_currentExecutingBlockDispatchId || m_currentExecutingBlockDispatchId <= s.armDispatch ||
        (step && (m_currentExecutingBlockDispatchId <= m_edm114.retained.source.dispatch ||
            !ReadEDM96CompletedSegmentsSameThread(step))) ||
        !m_motion.ReadEDM94Feedback(guard) || guard.identity.session != s.session ||
        guard.identity.epoch != s.armEpoch || guard.identity.ownerGeneration != s.lease.generation ||
        guard.currentIdentity.session != s.session || guard.currentIdentity.epoch != epoch ||
        guard.currentIdentity.ownerGeneration != s.lease.generation || !guard.active || !guard.armAccepted ||
        !guard.armedReady || guard.forceZero || guard.handoffZero || guard.faulted || guard.cancelled ||
        guard.finishRequested || guard.released || !guard.lastTick || !guard.lastNowUs ||
        guard.lastTick < s.lastTick || guard.lastTick < m_edm114.completionTick ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(epoch, s.lease)) return false;
    for (unsigned a = 0U; a < 6U; ++a)
    {
        const auto& frozen = s.feedback.config.axis[a]; const auto& current = guard.config.axis[a];
        const double target = step ? m_edm114.retained.endPulse[a] : frozen.commandAnchor;
        if (current.configFingerprint != frozen.configFingerprint || current.mappingFingerprint != frozen.mappingFingerprint ||
            !EDM109::SameDouble(current.pulsePerUnit, frozen.pulsePerUnit) ||
            !EDM109::SameDouble(guard.axis[a].currentCommandPulse, target)) return false;
    }
    EDM116::Policy policy{};
    if (!EDM116::ParseMode(block.val('B'), policy.mode)) return false;
    policy.fixture = s.gapModeFixture;
    policy.profile = s.gapFlushParameters; policy.selectorPC = GetActiveDispatchPC();
    policy.selectorDispatch = m_currentExecutingBlockDispatchId; policy.revision = step + 1ULL;
    if (!EDM116::ValidPolicy(policy)) return false;
    if (!step)
    {
        c.initialTranslation = CoordSys.GetTranslationSnapshot();
        c.initialTranslationValid = true;
    }
    c.policies[step] = policy; c.selected = step + 1U; c.pending = true;
    return true;
}

bool NCManager::AuthorizeEDM116SameThread(const NCBlock& block, int gcode,
    const std::array<double, 2U>& center) noexcept
{
    auto& s = m_edm94; auto& c = m_edm116;
    if (!s.gapModeFlush || !s.gapPhysicalCurve || s.gapCurveB2 || !s.next || s.next > s.moves) return false;
    const unsigned step = s.next - 1U; const auto& policy = c.policies[step];
    if (EDM116::IsModalFixture(s.gapModeFixture))
    {
        const int pc = GetActiveDispatchPC(); const auto* authored = GetBaseProgramCache().TryGetLine(pc);
        EDM127::Selection expected = m_edm127.current;
        double word = 0.0;
        if (!authored || authored->parsedBlock.has('B') != m_edm127.authoredInline[step] ||
            (m_edm127.authoredInline[step] &&
                (!E94Number(authored->parsedBlock.expression('B'), word) ||
                    !EDM127::Select(m_edm127.current, word, pc, m_currentExecutingBlockDispatchId, expected))) ||
            !EDM127::Matches(expected, policy) || !EDM127::Matches(m_edm127.pending, policy) ||
            policy.mode != m_edm127.expectedMode[step] || policy.modeOriginPC != m_edm127.expectedOriginPC[step] ||
            policy.modeOriginRevision != m_edm127.expectedOriginRevision[step] ||
            (step && !EDM116::PolicyFollowsCompleted(policy, c.completedPolicies[step - 1U], c.completedSources[step - 1U])))
            return false;
    }
    if (s.gapModeInterruption && (s.gapModeInterruptionArm != m_edm118.armP ||
        step > (EDM125::ValidProfile(EDM125::ProfileForArm(s.gapModeInterruptionArm)) ? EDM125::TargetStep : EDM123::ValidProfile(EDM123::ProfileForArm(s.gapModeInterruptionArm)) ? EDM123::TargetStep(EDM123::ProfileForArm(s.gapModeInterruptionArm)) : EDM122::ValidProfile(EDM122::ProfileForArm(s.gapModeInterruptionArm)) ? EDM122::TargetStep(EDM122::ProfileForArm(s.gapModeInterruptionArm)) : EDM118::TargetStep))) return false;
    if (!c.pending || c.selected != step + 1U || c.consumed != step ||
        !EDM116::ValidPolicy(policy) || policy.fixture != s.gapModeFixture || policy.mode != EDM116::ExpectedMode(step) ||
        policy.revision != step + 1ULL || policy.selectorPC != s.selectorPC[step] ||
        !EDM116::PolicyMatchesBinding(policy, step, GetActiveDispatchPC(), m_currentExecutingBlockDispatchId) ||
        !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
        !IsEDM116CurrentSameThread(step) || !BuildEDM116CompletionTokenSameThread(step)) return false;
    // The ordinary first-feed producer has now frozen/published exactly the
    // candidate observed by P131. Settings never publish motion authority.
    if (!step && (!c.initialTranslationValid || !CoordSys.IsTranslationRunFrozen() ||
        !CoordSys.IsTranslationRunCurrent() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsXYZCUVFeedProfileCurrentSameThread() ||
        !SameNCTranslationSnapshot(c.initialTranslation, CoordSys.GetTranslationSnapshot()) ||
        !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()))) return false;
    const auto epoch = m_motion.GetCurrentExecutionEpoch();
    if (!m_motion.AuthorizeEDM116Feed(s.session, s.run, s.cache, m_currentExecutingBlockDispatchId,
        GetActiveDispatchPC(), step, gcode, s.increment[step], center, block.val('F'),
        epoch, s.lease, s.gapTuning, s.gapFlushParameters, policy, c.token)) return false;
    c.issuedDispatch[step] = m_currentExecutingBlockDispatchId;
    c.issuedEpoch[step] = epoch == (std::numeric_limits<std::uint32_t>::max)() ? 1ULL : epoch + 1ULL;
    c.pending = false; c.consumed = step + 1U;
    if (EDM116::IsModalFixture(s.gapModeFixture))
    {
        m_edm127.current = m_edm127.pending;
        if (m_edm127.authoredInline[step]) ++m_edm127.explicitRows; else ++m_edm127.inheritedRows;
        m_edm127.lastPC = GetActiveDispatchPC(); m_edm127.lastDispatch = m_currentExecutingBlockDispatchId;
        m_edm127.pending = EDM127::Selection{};
        LogEDM127SameThread("ROW_AUTHORIZED", step);
    }
    return true;
}

bool NCManager::BuildEDM116CompletionTokenSameThread(unsigned step) noexcept
{
    auto& s = m_edm94; auto& c = m_edm116;
    c.token = EDM115::CompletionToken{};
    if (!s.gapModeFlush || !s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || step >= EDM114::RequiredRows || s.next != step + 1U ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || !IsEDM114CurrentSameThread(step) ||
        !IsEDM116CurrentSameThread(step)) return false;
    if (!EDM115::IsTriggerStep(step)) return true;
    const unsigned ordinal = EDM115::EventOrdinal(step), index = ordinal - 1U;
    const auto& prior = m_edm114.retained;
    EDM94::Feedback guard{};
    if (c.retainedEvents != step / 2U || !EDM115::EmptyToken(c.tokens[index]) ||
        !m_motion.ReadEDM94Feedback(guard) || !ReadEDM96CompletedSegmentsSameThread(step) ||
        m_edm114.acknowledgedRows != step || !m_edm114.completionTick || !m_edm114.completionCommit ||
        prior.source.step != step - 1U || prior.source.sourcePC != s.pc[step - 1U] ||
        prior.source.epoch != m_motion.GetCurrentExecutionEpoch() ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != prior.source.epoch || guard.currentIdentity.ownerGeneration != s.lease.generation ||
        !guard.active || !guard.armAccepted || !guard.armedReady || guard.forceZero || guard.handoffZero ||
        guard.faulted || guard.cancelled || guard.finishRequested || guard.released ||
        guard.lastTick < m_edm114.completionTick || guard.lastTick < s.lastTick ||
        !guard.lastNowUs || guard.lastNowUs < prior.lastMappingUs ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(prior.source.epoch, s.lease) ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        CoordSys.GetTranslationSnapshot().generation != prior.source.scope.translationGeneration ||
        !IsXYZCUVFeedProfileCurrentSameThread() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    std::uint64_t configTag = 1469598103934665603ULL, mappingTag = configTag;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& frozen = s.feedback.config.axis[axis]; const auto& observed = guard.config.axis[axis];
        if (observed.configFingerprint != frozen.configFingerprint || observed.mappingFingerprint != frozen.mappingFingerprint ||
            !EDM109::SameDouble(observed.pulsePerUnit, prior.pulsePerNative[axis]) ||
            !EDM109::SameDouble(observed.pulsePerUnit, frozen.pulsePerUnit) ||
            !EDM109::SameDouble(guard.axis[axis].currentCommandPulse, prior.endPulse[axis])) return false;
        configTag = (configTag ^ observed.configFingerprint) * 1099511628211ULL;
        mappingTag = (mappingTag ^ observed.mappingFingerprint) * 1099511628211ULL;
    }
    if (prior.source.scope.configTag != (configTag ? configTag : 1ULL) ||
        prior.source.scope.mappingTag != (mappingTag ? mappingTag : 1ULL)) return false;
    c.token.previous = prior.source;
    c.token.completion = EDM109::Completion{m_edm114.completionTick, m_edm114.completionCommit, true};
    c.token.mappingTick = prior.lastMappingTick; c.token.valid = true;
    if (!EDM115::ValidToken(c.token, step)) { c.token = EDM115::CompletionToken{}; return false; }
    c.tokens[index] = c.token; c.previousRows[index] = prior;
    return true;
}

bool NCManager::IsEDM116ReceiptSameThread(const EDM116::MotionFeedback& f, unsigned ordinal,
    const EDM114::Feedback& trigger, std::uint64_t completionTick, std::uint64_t completionUs) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm116; const auto& e = f.excursion;
    if (!s.gapModeFlush || !s.gapPhysicalCurve || ordinal == 0U || ordinal > EDM116::ExpectedEvents) return false;
    const unsigned index = ordinal - 1U, step = index * 2U;
    const auto& policy = c.policies[step]; const auto mode = EDM116::ExpectedMode(step);
    const bool b2 = mode == EDM116::Mode::B2;
    unsigned dwells = 0U; decltype(f.modeStarted) counts{};
    for (unsigned row = 0U; row <= step; row += 2U)
    { const auto selected = EDM116::ExpectedMode(row); ++counts[static_cast<unsigned>(selected)]; dwells += EDM116::DwellCount(selected); }
    if (!EDM116::ValidPolicy(policy) || policy.fixture != s.gapModeFixture || f.fixture != s.gapModeFixture ||
        policy.mode != mode || !EDM116::SamePolicy(f.policy, policy) ||
        !EDM116::SamePolicy(f.rowPolicy, policy) || !EDM116::PolicyMatchesSource(policy, f.trigger) ||
        policy.selectorPC != s.selectorPC[step] || !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
        !E114Counts(trigger, step + 1U, s.gapModeFixture) || !EDM114::SameSource(f.trigger, trigger.source) ||
        !EDM114::SameSource(f.rowSource, trigger.source) ||
        f.trigger.scope.session != s.session || f.trigger.scope.run != s.run || f.trigger.scope.cache != s.cache ||
        f.trigger.scope.ownerGeneration != s.lease.generation || f.trigger.step != step || f.trigger.sourcePC != s.pc[step] ||
        !f.enabled || !f.tokenValidated || !f.geometryCurrent || f.session != s.session ||
        f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        f.authorizedRows != step + 1U || f.completedRows != step + 1U ||
        f.eventsStarted != ordinal || f.eventsCompleted != ordinal || f.completedDwellsTotal != dwells ||
        f.anchorReturnsTotal != ordinal || f.modeStarted != counts || f.modeCompleted != counts ||
        f.currentKind != trigger.kind || !E115AnchorMatches(trigger, f.anchorQ, f.anchorPulse) ||
        !EDM109::SamePulse(f.anchorPulse, f.returnedPulse) ||
        !e.geometryValid || !e.complete || e.phase != EDM116::Phase::Complete || e.reason != EDM116::Reason::None ||
        !e.endpointReady || e.active || e.moving || e.faulted || e.stopRequested || e.stopped ||
        e.started != 1ULL || e.completed != 1ULL || e.anchorReturned != 1ULL ||
        e.completedDwells != EDM116::DwellCount(mode) || e.settledCycles < EDM116::RequiredSettledCycles ||
        !EDM109::SameDouble(e.progress, e.length) || !std::isfinite(e.length) || e.length <= 0.0 || e.v != 0.0 || e.a != 0.0 ||
        !std::isfinite(e.maximumVelocity) || e.maximumVelocity <= 0.0 ||
        !std::isfinite(e.acceleration) || e.acceleration <= 0.0 || !std::isfinite(e.deceleration) || e.deceleration <= 0.0 ||
        f.deadlineUs != EDM116::DeadlineUs(mode) || !e.startTick || !e.startUs ||
        e.tick <= e.startTick || e.nowUs <= e.startUs || e.nowUs - e.startUs >= f.deadlineUs ||
        e.tick - e.startTick >= f.deadlineUs / 250ULL || !f.lastPhysicalTick ||
        f.lastPhysicalTick < e.startTick || f.lastPhysicalTick >= e.tick || e.tick - f.lastPhysicalTick != 1ULL ||
        trigger.lastMappingTick <= e.tick || trigger.lastMappingUs <= e.nowUs ||
        completionTick < trigger.lastMappingTick || completionUs < trigger.lastMappingUs) return false;
    if (b2)
    {
        const unsigned b2Index = EDM115::EventOrdinal(step) - 1U;
        if (b2Index >= 2U) return false;
        const auto& token = c.tokens[b2Index]; const auto& previous = c.previousRows[b2Index];
        if (!EDM115::ValidToken(token, step) || !EDM115::SameToken(f.previous, token) ||
            !E114Counts(previous, step, s.gapModeFixture) || !EDM114::SameSource(token.previous, previous.source) ||
            !EDM109::SameScope(previous.source.scope, trigger.source.scope) ||
            previous.source.sourcePC != s.pc[step - 1U] ||
            f.trigger.dispatch <= previous.source.dispatch || f.trigger.epoch <= previous.source.epoch ||
            f.trigger.segment <= previous.source.segment || f.trigger.sourcePC <= previous.source.sourcePC ||
            token.mappingTick != previous.lastMappingTick || token.completion.tick < previous.lastMappingTick ||
            f.previousBoundaryTick != token.completion.tick || f.previousCommit != token.completion.commit ||
            f.previousKind != previous.kind ||
            (f.currentKind == f.previousKind && !(EDM116::IsArcPairFixture(s.gapModeFixture) &&
                step == 4U && f.currentKind == EDM114::Kind::Arc && f.previousKind == EDM114::Kind::Arc)) ||
            !EDM109::SamePulse(previous.endPulse, trigger.startPulse) ||
            !EDM109::SamePulse(previous.pulsePerNative, trigger.pulsePerNative) ||
            e.span != 3U || e.direction != 1 || e.rowKind != trigger.kind ||
            e.backwardCrossings != 1ULL || e.forwardCrossings != 1ULL ||
            !EDM109::SameDouble(e.q, f.anchorQ) || !EDM109::SameDouble(e.length, f.anchorQ) ||
            e.startTick <= token.completion.tick || e.startUs <= previous.lastMappingUs ||
            f.lastPhysicalTick <= token.completion.tick ||
            !E115AnchorMatches(previous, previous.pulseLength * (1.0 - EDM115::PreviousRetreatFraction), f.retreatPulse)) return false;
        double nativeLength = 0.0;
        if (previous.kind == EDM114::Kind::Arc) nativeLength = previous.pulseLength / previous.pulsePerNative[0];
        else for (unsigned a = 0U; a < 3U; ++a)
            nativeLength = std::hypot(nativeLength, (previous.endPulse[a] - previous.startPulse[a]) / previous.pulsePerNative[a]);
        if (!EDM109::SameDouble(f.strokeNativeMm, nativeLength * EDM115::PreviousRetreatFraction) ||
            f.topDwellTicks || f.topDwellUs || f.returnDwellTicks || f.returnDwellUs ||
            f.topSettledCycles || f.returnSettledCycles) return false;
    }
    else
    {
        EDM116::Point retreat{}; double stroke = 0.0;
        if (!EDM115::EmptyToken(f.previous) || f.previousBoundaryTick || f.previousCommit || f.previousKind != EDM114::Kind::None ||
            e.span != 1U || e.direction != -1 || e.rowKind != EDM114::Kind::Line || e.q != 0.0 ||
            e.backwardCrossings || e.forwardCrossings ||
            !EDM116::ComputeRetreat(trigger, f.anchorPulse, f.anchorQ, mode, policy.profile.strokeRatio, retreat, stroke) ||
            !EDM109::SameDouble(stroke, f.strokeNativeMm) || !EDM109::SamePulse(retreat, f.retreatPulse) ||
            f.topDwellTicks < EDM107::FlushDwellTicks(policy.profile) || f.topDwellUs < policy.profile.dwellUs ||
            f.returnDwellTicks < EDM107::FlushDwellTicks(policy.profile) || f.returnDwellUs < policy.profile.dwellUs ||
            f.topDwellTicks >= e.tick - e.startTick || f.returnDwellTicks >= e.tick - e.startTick ||
            f.topDwellUs >= e.nowUs - e.startUs || f.returnDwellUs >= e.nowUs - e.startUs ||
            f.topSettledCycles < EDM116::RequiredSettledCycles || f.returnSettledCycles < EDM116::RequiredSettledCycles)
            return false;
    }
    return true;
}

bool NCManager::IsEDM116CurrentSameThread(unsigned completed) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm116;
    if (!s.gapModeFlush || !s.gapPhysicalCurve || s.gapCurveB2 || !s.gapFlushProfile || !s.gapFlushParameterized ||
        s.gapCommonPath || s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
        s.gapFlushInterruptArm || !s.latched || s.armPending || !s.everArmed || !s.armEpoch ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || completed > s.moves || m_edm114.acknowledgedRows != completed ||
        c.acknowledgedRows != completed || c.consumed != completed || c.selected != completed + (c.pending ? 1U : 0U) || c.selected > s.moves ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.ReadEDM116Feedback(c.feedback)) return false;
    const auto& f = c.feedback; const auto& e = f.excursion;
    if (e.faulted && !c.failureLogged) { c.failureLogged = true; LogEDM116SameThread("MODE_FLUSH_FAIL"); }
    const unsigned expected = (completed + 1U) / 2U;
    unsigned dwells = 0U; decltype(f.modeStarted) counts{};
    for (unsigned row = 0U; row < completed; row += 2U)
    { const auto mode = EDM116::ExpectedMode(row); ++counts[static_cast<unsigned>(mode)]; dwells += EDM116::DwellCount(mode); }
    if (!EDM116::ValidFixture(s.gapModeFixture) || f.fixture != s.gapModeFixture ||
        !f.enabled || f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        e.faulted || e.stopRequested || e.stopped || e.active || e.moving || e.reason != EDM116::Reason::None ||
        c.retainedEvents != expected || f.eventsStarted != expected || f.eventsCompleted != expected ||
        f.authorizedRows != completed || f.completedRows != completed ||
        f.completedDwellsTotal != dwells || f.anchorReturnsTotal != expected || f.modeStarted != counts || f.modeCompleted != counts)
        return false;
    for (unsigned row = 0U; row < c.selected; ++row)
    {
        const auto& policy = c.policies[row];
        if (!EDM116::ValidPolicy(policy) || policy.fixture != s.gapModeFixture || policy.mode != EDM116::ExpectedMode(row) || policy.revision != row + 1ULL ||
            policy.selectorPC != s.selectorPC[row] ||
            (EDM116::IsModalFixture(s.gapModeFixture) ? policy.selectorPC != s.pc[row] : policy.selectorPC >= s.pc[row]) ||
            policy.selectorDispatch <= s.armDispatch ||
            (EDM116::IsModalFixture(s.gapModeFixture) &&
                (policy.mode != m_edm127.expectedMode[row] || policy.modeOriginPC != m_edm127.expectedOriginPC[row] ||
                    policy.modeOriginRevision != m_edm127.expectedOriginRevision[row])) ||
            !EDM107::SameFlushProfile(policy.profile, s.gapFlushParameters) ||
            (row && policy.selectorDispatch <= c.policies[row - 1U].selectorDispatch)) return false;
    }
    if (completed == 0U)
    {
        if (!EDM115::EmptyToken(f.previous) || !EDM114::SameSource(f.trigger, EDM114::Source{}) ||
            !EDM114::SameSource(f.rowSource, EDM114::Source{}) || !E115SameExcursion(e, EDM116::Feedback{}) ||
            !EDM116::SamePolicy(f.policy, EDM116::Policy{}) || !EDM116::SamePolicy(f.rowPolicy, EDM116::Policy{}) ||
            f.tokenValidated || f.geometryCurrent || f.currentKind != EDM114::Kind::None || f.previousKind != EDM114::Kind::None ||
            f.anchorQ != 0.0 || f.strokeNativeMm != 0.0 || f.deadlineUs || f.previousBoundaryTick || f.previousCommit ||
            f.lastPhysicalTick || f.rangeResets || f.topDwellTicks || f.topDwellUs || f.returnDwellTicks || f.returnDwellUs ||
            f.topSettledCycles || f.returnSettledCycles) return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (f.anchorPulse[a] != 0.0 || f.returnedPulse[a] != 0.0 || f.retreatPulse[a] != 0.0) return false;
        return true;
    }
    for (unsigned row = 0U; row < completed; ++row)
    {
        const auto& source = c.completedSources[row];
        if (!EDM116::SamePolicy(c.completedPolicies[row], c.policies[row]) ||
            !EDM116::PolicyMatchesSource(c.completedPolicies[row], source) || source.step != row ||
            (EDM116::IsModalFixture(s.gapModeFixture) && row &&
                !EDM116::PolicyFollowsCompleted(c.completedPolicies[row], c.completedPolicies[row - 1U], c.completedSources[row - 1U])) ||
            source.sourcePC != s.pc[row] || !c.issuedDispatch[row] || !c.issuedEpoch[row] ||
            source.dispatch != c.issuedDispatch[row] || source.epoch != c.issuedEpoch[row] ||
            !EDM109::SameScope(source.scope, m_edm114.retained.source.scope) ||
            (row && (source.dispatch <= c.completedSources[row - 1U].dispatch ||
                source.epoch <= c.completedSources[row - 1U].epoch || source.segment <= c.completedSources[row - 1U].segment ||
                c.completedPolicies[row].selectorDispatch <= c.completedSources[row - 1U].dispatch)) ||
            (!(row & 1U) && !EDM114::SameSource(source, c.receipts[row / 2U].trigger))) return false;
    }
    if (!EDM114::SameSource(f.rowSource, m_edm114.retained.source) ||
        !EDM114::SameSource(f.rowSource, c.completedSources[completed - 1U]) ||
        !EDM116::SamePolicy(f.rowPolicy, c.policies[completed - 1U]) ||
        !EDM116::PolicyMatchesSource(f.rowPolicy, f.rowSource) ||
        !E116SameEvent(f, c.receipts[expected - 1U])) return false;
    for (unsigned index = 0U; index < expected; ++index)
    {
        if (!c.completionCommit[index] || !IsEDM116ReceiptSameThread(c.receipts[index], index + 1U,
                c.triggerRows[index], c.completionTick[index], c.completionUs[index]) ||
            !EDM109::SameScope(m_edm114.retained.source.scope, c.triggerRows[index].source.scope)) return false;
        if (index)
        {
            const auto& prior = c.receipts[index - 1U]; const auto& current = c.receipts[index];
            if (prior.trigger.dispatch >= current.policy.selectorDispatch || prior.trigger.epoch >= current.trigger.epoch ||
                prior.trigger.segment >= current.trigger.segment || prior.trigger.sourcePC >= current.policy.selectorPC ||
                c.completionCommit[index - 1U] >= c.completionCommit[index] ||
                c.completionTick[index - 1U] >= current.excursion.startTick ||
                c.completionUs[index - 1U] >= current.excursion.startUs ||
                prior.excursion.tick >= current.excursion.startTick || prior.lastPhysicalTick >= current.lastPhysicalTick)
                return false;
        }
    }
    return true;
}

bool NCManager::RetainEDM116CompletedSameThread(const EDM94::Feedback& guard) noexcept
{
    auto& s = m_edm94; auto& c = m_edm116;
    if (!s.gapModeFlush || !s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending || s.cancelled ||
        s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported || s.finishPending || s.complete ||
        s.next == 0U || s.next > s.moves || c.pending || c.selected != s.next || c.consumed != s.next ||
        !IsEDM114CurrentSameThread(s.next)) return false;
    if (c.acknowledgedRows == s.next) return IsEDM116CurrentSameThread(s.next);
    const unsigned step = s.next - 1U; const bool event = EDM116::IsTriggerStep(step);
    const unsigned ordinal = (step + 2U) / 2U, index = ordinal - 1U;
    EDM96::Feedback gap{};
    if (c.acknowledgedRows + 1U != s.next || c.retainedEvents != (event ? index : ordinal) ||
        !m_motion.ReadEDM116Feedback(c.feedback) || c.feedback.fixture != s.gapModeFixture ||
        !m_motion.ReadEDM96Feedback(gap) || !ReadEDM96CompletedSegmentsSameThread(s.next) ||
        !EDM114::SameSource(c.feedback.rowSource, m_edm114.retained.source) ||
        !EDM116::SamePolicy(c.feedback.rowPolicy, c.policies[step]) ||
        !EDM116::PolicyMatchesSource(c.feedback.rowPolicy, c.feedback.rowSource) ||
        c.feedback.authorizedRows != s.next || c.feedback.completedRows != s.next ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != m_edm114.retained.source.epoch || guard.currentIdentity.ownerGeneration != s.lease.generation ||
        guard.lastTick != m_edm114.completionTick || guard.lastNowUs < m_edm114.retained.lastMappingUs ||
        guard.faulted || guard.cancelled || guard.forceZero || guard.handoffZero || !guard.active ||
        !guard.armAccepted || !guard.armedReady || guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles ||
        guard.boundaryNotReadyMask || gap.session != s.session || gap.ownerGeneration != s.lease.generation ||
        gap.epoch != m_edm114.retained.source.epoch || gap.expectedStep != s.next) return false;
    if (event)
    {
        if (gap.flushAnchor != c.feedback.anchorQ || gap.flushReturned != c.feedback.anchorQ ||
            !IsEDM116ReceiptSameThread(c.feedback, ordinal, m_edm114.retained, guard.lastTick, guard.lastNowUs)) return false;
        c.receipts[index] = c.feedback; c.triggerRows[index] = m_edm114.retained;
        c.completionTick[index] = m_edm114.completionTick; c.completionCommit[index] = m_edm114.completionCommit;
        c.completionUs[index] = guard.lastNowUs; c.retainedEvents = ordinal;
    }
    c.completedPolicies[step] = c.feedback.rowPolicy; c.completedSources[step] = c.feedback.rowSource;
    c.acknowledgedRows = s.next;
    if (!IsEDM116CurrentSameThread(s.next)) return false;
    if (event) LogEDM116SameThread("MODE_EVENT_RETAINED");
    return true;
}

void NCManager::LogEDM116SameThread(const char* event) const noexcept
{
    if (EDM116::IsModalFixture(m_edm94.gapModeFixture)) { LogEDM127SameThread(event, EDM114::RequiredRows); return; }
    const auto& s = m_edm94; const auto& c = m_edm116;
    const bool failure = event && std::strcmp(event, "MODE_FLUSH_FAIL") == 0;
    const auto& f = failure || !c.retainedEvents ? c.feedback : c.receipts[(std::min)(c.retainedEvents, 8U) - 1U];
    char record[481]{};
    const auto emit = [&](int length) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
        { if (RtPrintf("%s", record) < 0) RtPrintf("%s", (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=LOG_WRITE_ERROR profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=LOG_WRITE_ERROR profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=LOG_WRITE_ERROR profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=LOG_WRITE_ERROR profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=LOG_WRITE_ERROR profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=LOG_WRITE_ERROR profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=LOG_WRITE_ERROR\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=LOG_WRITE_ERROR\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=LOG_WRITE_ERROR\n" : s.gapModeInterruption ? "[EDM118] event=LOG_WRITE_ERROR\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=LOG_WRITE_ERROR fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=LOG_WRITE_ERROR fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=LOG_WRITE_ERROR\n" : "[EDM116] event=LOG_WRITE_ERROR\n")); }
        else RtPrintf("%s", (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=LOG_FORMAT_ERROR profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=LOG_FORMAT_ERROR profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=LOG_FORMAT_ERROR profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=LOG_FORMAT_ERROR profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=LOG_FORMAT_ERROR profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=LOG_FORMAT_ERROR profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=LOG_FORMAT_ERROR\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=LOG_FORMAT_ERROR\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=LOG_FORMAT_ERROR\n" : s.gapModeInterruption ? "[EDM118] event=LOG_FORMAT_ERROR\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=LOG_FORMAT_ERROR fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=LOG_FORMAT_ERROR fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=LOG_FORMAT_ERROR\n" : "[EDM116] event=LOG_FORMAT_ERROR\n"));
    };
    emit(std::snprintf(record, sizeof(record),
        (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=%.28s session=%llu P=154 finishP=155 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=%.28s session=%llu P=156 finishP=157 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=%.28s session=%llu P=146 finishP=147 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=%.28s session=%llu P=142 finishP=143 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=%.28s session=%llu P=148 finishP=149 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=%.28s session=%llu P=144 finishP=145 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=%.28s session=%llu P=140 finishP=141 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=%.28s session=%llu P=138 finishP=139 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=%.28s session=%llu P=136 finishP=137 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE\n" : s.gapModeInterruption ? "[EDM118] event=%.28s session=%llu P=134 finishP=135 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=%.28s session=%llu P=150 finishP=151 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=%.28s session=%llu P=152 finishP=153 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=%.28s session=%llu P=132 finishP=133 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE\n" : "[EDM116] event=%.28s session=%llu P=129 finishP=130 selected=%u consumed=%u events=%u/%u B0=%u B1=%u B2=%u dwells=%u anchors=%u normalRows=%u line=%u arc=%u CW=%u CCW=%u expectedEvents=8 expectedDwells=28 gap=RT_SYNTHETIC dischargeCommand=NONE\n"),
        event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), c.selected, c.consumed,
        f.eventsStarted, f.eventsCompleted, f.modeCompleted[0], f.modeCompleted[1], f.modeCompleted[2],
        f.completedDwellsTotal, f.anchorReturnsTotal, m_edm114.acknowledgedRows,
        m_edm114.retained.lineRows, m_edm114.retained.arcRows, m_edm114.retained.cwRows, m_edm114.retained.ccwRows));
    const unsigned count = failure ? 1U : (std::min)(c.retainedEvents, 8U);
    for (unsigned i = 0U; i < count; ++i)
    {
        const auto& r = failure ? c.feedback : c.receipts[i]; const auto& e = r.excursion;
        emit(std::snprintf(record, sizeof(record),
            (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu\n" : s.gapModeInterruption ? "[EDM118] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu\n" : "[EDM116] event=MODE_SOURCE result=%.28s session=%llu step=%u mode=%u selectorPC=%d selectorDispatch=%llu revision=%llu sourcePC=%d dispatch=%llu epoch=%llu currentKind=%u previousKind=%u previousCommit=%llu\n"),
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), r.trigger.step, static_cast<unsigned>(r.policy.mode),
            r.policy.selectorPC, static_cast<unsigned long long>(r.policy.selectorDispatch), static_cast<unsigned long long>(r.policy.revision),
            r.trigger.sourcePC, static_cast<unsigned long long>(r.trigger.dispatch), static_cast<unsigned long long>(r.trigger.epoch),
            static_cast<unsigned>(r.currentKind), static_cast<unsigned>(r.previousKind), static_cast<unsigned long long>(r.previousCommit)));
        emit(std::snprintf(record, sizeof(record),
            (s.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s profile=PREVIOUS_ARC_RETREAT armP=154\n" : s.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s profile=PREVIOUS_ARC_RETURN armP=156\n" : s.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s profile=B0 armP=146\n" : s.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s profile=B0 armP=142\n" : s.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s profile=B1 armP=148\n" : s.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s profile=B1 armP=144\n" : s.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s\n" : s.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s\n" : s.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s\n" : s.gapModeInterruption ? "[EDM118] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s\n" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s fixture=2\n" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s fixture=3\n" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s\n" : "[EDM116] event=MODE_PROOF session=%llu step=%u mode=%u dwells=%u settled=%u back=%llu forward=%llu returned=%llu anchorQ=%.12g strokeMm=%.12g topDwellUs=%llu returnDwellUs=%llu phase=%u reason=%u physicalTick=%llu endTick=%llu proof=%s\n"),
            static_cast<unsigned long long>(s.session), r.trigger.step, static_cast<unsigned>(r.policy.mode), e.completedDwells, e.settledCycles,
            static_cast<unsigned long long>(e.backwardCrossings), static_cast<unsigned long long>(e.forwardCrossings),
            static_cast<unsigned long long>(e.anchorReturned), r.anchorQ, r.strokeNativeMm,
            static_cast<unsigned long long>(r.topDwellUs), static_cast<unsigned long long>(r.returnDwellUs),
            static_cast<unsigned>(e.phase), static_cast<unsigned>(e.reason), static_cast<unsigned long long>(r.lastPhysicalTick),
            static_cast<unsigned long long>(e.tick), failure ? "INCOMPLETE" : "CANONICAL_ANCHOR_AND_FOLLOWING_200"));
    }
}

bool NCManager::BuildEDM115CompletionTokenSameThread(unsigned step) noexcept
{
    auto& s = m_edm94; auto& c = m_edm115;
    c.token = EDM115::CompletionToken{};
    if (!s.gapCurveB2 || !s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || step >= EDM114::RequiredRows || s.next != step + 1U ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || !IsEDM114CurrentSameThread(step) ||
        !IsEDM115CurrentSameThread(step)) return false;
    if (!EDM115::IsTriggerStep(step)) return true;
    const unsigned ordinal = EDM115::EventOrdinal(step), index = ordinal - 1U;
    const auto& prior = m_edm114.retained;
    EDM94::Feedback guard{};
    if (c.retainedEvents != index || !EDM115::EmptyToken(c.tokens[index]) ||
        !m_motion.ReadEDM94Feedback(guard) || !ReadEDM96CompletedSegmentsSameThread(step) ||
        m_edm114.acknowledgedRows != step || !m_edm114.completionTick || !m_edm114.completionCommit ||
        prior.source.step != step - 1U || prior.source.sourcePC != s.pc[step - 1U] ||
        prior.source.epoch != m_motion.GetCurrentExecutionEpoch() ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != prior.source.epoch || guard.currentIdentity.ownerGeneration != s.lease.generation ||
        !guard.active || !guard.armAccepted || !guard.armedReady || guard.forceZero || guard.handoffZero ||
        guard.faulted || guard.cancelled || guard.finishRequested || guard.released ||
        guard.lastTick < m_edm114.completionTick || guard.lastTick < s.lastTick ||
        !guard.lastNowUs || guard.lastNowUs < prior.lastMappingUs ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(prior.source.epoch, s.lease) ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        CoordSys.GetTranslationSnapshot().generation != prior.source.scope.translationGeneration ||
        !IsXYZCUVFeedProfileCurrentSameThread() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    std::uint64_t configTag = 1469598103934665603ULL, mappingTag = configTag;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& frozen = s.feedback.config.axis[axis]; const auto& observed = guard.config.axis[axis];
        if (observed.configFingerprint != frozen.configFingerprint || observed.mappingFingerprint != frozen.mappingFingerprint ||
            !EDM109::SameDouble(observed.pulsePerUnit, prior.pulsePerNative[axis]) ||
            !EDM109::SameDouble(observed.pulsePerUnit, frozen.pulsePerUnit) ||
            !EDM109::SameDouble(guard.axis[axis].currentCommandPulse, prior.endPulse[axis])) return false;
        configTag = (configTag ^ observed.configFingerprint) * 1099511628211ULL;
        mappingTag = (mappingTag ^ observed.mappingFingerprint) * 1099511628211ULL;
    }
    if (prior.source.scope.configTag != (configTag ? configTag : 1ULL) ||
        prior.source.scope.mappingTag != (mappingTag ? mappingTag : 1ULL)) return false;
    c.token.previous = prior.source;
    c.token.completion = EDM109::Completion{m_edm114.completionTick, m_edm114.completionCommit, true};
    c.token.mappingTick = prior.lastMappingTick; c.token.valid = true;
    if (!EDM115::ValidToken(c.token, step)) { c.token = EDM115::CompletionToken{}; return false; }
    c.tokens[index] = c.token; c.previousRows[index] = prior;
    return true;
}

bool NCManager::IsEDM115ReceiptSameThread(const EDM115::MotionFeedback& f, unsigned ordinal,
    const EDM114::Feedback& trigger, std::uint64_t completionTick, std::uint64_t completionUs) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm115; const auto& e = f.excursion;
    if (!s.gapCurveB2 || !s.gapPhysicalCurve || ordinal == 0U || ordinal > EDM115::ExpectedEvents) return false;
    const unsigned index = ordinal - 1U, step = ordinal == 1U ? 2U : 4U;
    const auto& token = c.tokens[index]; const auto& previous = c.previousRows[index];
    if (!EDM115::ValidToken(token, step) || !EDM115::SameToken(f.previous, token) ||
        !E114Counts(previous, step) || !E114Counts(trigger, step + 1U) ||
        !EDM114::SameSource(token.previous, previous.source) || !EDM114::SameSource(f.trigger, trigger.source) ||
        !EDM114::ValidSource(f.trigger) || !EDM109::SameScope(previous.source.scope, trigger.source.scope) ||
        f.trigger.scope.session != s.session || f.trigger.scope.run != s.run || f.trigger.scope.cache != s.cache ||
        f.trigger.scope.ownerGeneration != s.lease.generation || f.trigger.step != step ||
        f.trigger.sourcePC != s.pc[step] || previous.source.sourcePC != s.pc[step - 1U] ||
        f.trigger.dispatch <= previous.source.dispatch || f.trigger.epoch <= previous.source.epoch ||
        f.trigger.segment <= previous.source.segment || f.trigger.sourcePC <= previous.source.sourcePC ||
        token.mappingTick != previous.lastMappingTick || token.completion.tick < previous.lastMappingTick ||
        !f.enabled || !f.tokenValidated || !f.geometryCurrent ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        f.previousBoundaryTick != token.completion.tick || f.previousCommit != token.completion.commit ||
        f.eventsStarted != ordinal || f.eventsCompleted != ordinal ||
        f.completedDwellsTotal != 5U * ordinal || f.anchorReturnsTotal != ordinal ||
        f.currentKind != trigger.kind || f.previousKind != previous.kind ||
        f.currentKind != (ordinal == 1U ? EDM114::Kind::Arc : EDM114::Kind::Line) ||
        f.previousKind != (ordinal == 1U ? EDM114::Kind::Line : EDM114::Kind::Arc) ||
        !EDM109::SamePulse(previous.endPulse, trigger.startPulse) ||
        !EDM109::SamePulse(previous.pulsePerNative, trigger.pulsePerNative) ||
        !E115AnchorMatches(trigger, f.anchorQ, f.anchorPulse) || !EDM109::SamePulse(f.anchorPulse, f.returnedPulse) ||
        !e.geometryValid || !e.complete || e.phase != EDM115::Phase::Complete || e.reason != EDM115::Reason::None ||
        !e.endpointReady || e.span != 3U || e.active || e.moving || e.faulted || e.stopRequested || e.stopped ||
        e.started != 1ULL || e.completed != 1ULL || e.backwardCrossings != 1ULL ||
        e.forwardCrossings != 1ULL || e.anchorReturned != 1ULL || e.completedDwells != 5U ||
        e.settledCycles < EDM115::RequiredSettledCycles || e.rowKind != f.currentKind || e.direction != 1 ||
        !EDM109::SameDouble(e.q, f.anchorQ) || !EDM109::SameDouble(e.progress, e.length) ||
        !EDM109::SameDouble(e.length, f.anchorQ) || e.v != 0.0 || e.a != 0.0 ||
        !std::isfinite(e.maximumVelocity) || e.maximumVelocity <= 0.0 ||
        !std::isfinite(e.acceleration) || e.acceleration <= 0.0 || !std::isfinite(e.deceleration) || e.deceleration <= 0.0 ||
        !e.startTick || e.startTick <= token.completion.tick || !e.tick || e.tick <= e.startTick ||
        e.tick - e.startTick >= EDM115::MaximumDurationTicks || !e.startUs || e.startUs <= previous.lastMappingUs ||
        !e.nowUs || e.nowUs <= e.startUs || e.nowUs - e.startUs >= EDM115::MaximumDurationUs ||
        !f.lastPhysicalTick || f.lastPhysicalTick <= token.completion.tick || f.lastPhysicalTick >= e.tick ||
        e.tick - f.lastPhysicalTick != 1ULL || trigger.lastMappingTick <= e.tick || trigger.lastMappingUs <= e.nowUs ||
        completionTick < trigger.lastMappingTick || completionUs < trigger.lastMappingUs) return false;
    return true;
}

bool NCManager::IsEDM115CurrentSameThread(unsigned completed) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm115;
    if (!s.gapCurveB2 || !s.gapPhysicalCurve || !s.gapFlushProfile || !s.gapFlushParameterized ||
        s.gapCommonPath || s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
        s.gapFlushInterruptArm || !s.latched || s.armPending || !s.everArmed || !s.armEpoch ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || completed > s.moves || m_edm114.acknowledgedRows != completed ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.ReadEDM115Feedback(c.feedback)) return false;
    const auto& f = c.feedback; const auto& e = f.excursion;
    if (e.faulted && !c.failureLogged) { c.failureLogged = true; LogEDM115SameThread("CURVE_B2_FAIL"); }
    const unsigned expected = completed >= 5U ? 2U : completed >= 3U ? 1U : 0U;
    if (!f.enabled || f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        e.faulted || e.stopRequested || e.stopped || e.active || e.moving || e.reason != EDM115::Reason::None ||
        c.retainedEvents != expected || f.eventsStarted != expected || f.eventsCompleted != expected ||
        f.completedDwellsTotal != 5U * expected || f.anchorReturnsTotal != expected) return false;
    if (expected == 0U)
    {
        if (!EDM115::EmptyToken(f.previous) || !EDM114::SameSource(f.trigger, EDM114::Source{}) ||
            !E115SameExcursion(e, EDM115::Feedback{}) || f.tokenValidated || f.geometryCurrent ||
            f.currentKind != EDM114::Kind::None || f.previousKind != EDM114::Kind::None ||
            f.anchorQ != 0.0 || f.previousBoundaryTick || f.previousCommit || f.lastPhysicalTick || f.rangeResets)
            return false;
        for (unsigned a = 0U; a < 6U; ++a) if (f.anchorPulse[a] != 0.0 || f.returnedPulse[a] != 0.0) return false;
        return true;
    }
    if (!E115SameFeedback(f, c.receipts[expected - 1U])) return false;
    for (unsigned index = 0U; index < expected; ++index)
        if (!c.completionCommit[index] ||
            !IsEDM115ReceiptSameThread(c.receipts[index], index + 1U, c.triggerRows[index],
                c.completionTick[index], c.completionUs[index]) ||
            !EDM109::SameScope(m_edm114.retained.source.scope, c.triggerRows[index].source.scope)) return false;
    if (expected == 2U)
    {
        const auto& first = c.receipts[0]; const auto& last = c.receipts[1];
        if (first.trigger.dispatch >= last.trigger.dispatch || first.trigger.epoch >= last.trigger.epoch ||
            first.trigger.segment >= last.trigger.segment || first.trigger.sourcePC >= last.trigger.sourcePC ||
            c.completionCommit[0] >= c.tokens[1].completion.commit || c.tokens[1].completion.commit >= c.completionCommit[1] ||
            c.completionTick[0] >= c.tokens[1].completion.tick || c.tokens[1].completion.tick >= c.completionTick[1] ||
            first.excursion.tick >= last.excursion.startTick || first.excursion.nowUs >= last.excursion.startUs ||
            first.lastPhysicalTick >= last.lastPhysicalTick) return false;
    }
    return true;
}

bool NCManager::RetainEDM115CompletedSameThread(const EDM94::Feedback& guard) noexcept
{
    auto& s = m_edm94; auto& c = m_edm115;
    if (!s.gapCurveB2 || !s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending || s.cancelled ||
        s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported || s.finishPending || s.complete ||
        s.next == 0U || s.next > s.moves || !IsEDM114CurrentSameThread(s.next)) return false;
    const unsigned step = s.next - 1U;
    if (!EDM115::IsTriggerStep(step)) return IsEDM115CurrentSameThread(s.next);
    const unsigned ordinal = EDM115::EventOrdinal(step), index = ordinal - 1U;
    if (c.retainedEvents == ordinal) return IsEDM115CurrentSameThread(s.next);
    EDM96::Feedback gap{};
    if (c.retainedEvents != index || !m_motion.ReadEDM115Feedback(c.feedback) ||
        !m_motion.ReadEDM96Feedback(gap) || !ReadEDM96CompletedSegmentsSameThread(s.next) ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != m_edm114.retained.source.epoch || guard.currentIdentity.ownerGeneration != s.lease.generation ||
        guard.lastTick != m_edm114.completionTick || guard.lastNowUs < m_edm114.retained.lastMappingUs ||
        guard.faulted || guard.cancelled || guard.forceZero || guard.handoffZero || !guard.active ||
        !guard.armAccepted || !guard.armedReady || guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles ||
        guard.boundaryNotReadyMask || gap.session != s.session || gap.ownerGeneration != s.lease.generation ||
        gap.epoch != m_edm114.retained.source.epoch || gap.expectedStep != s.next ||
        gap.flushAnchor != c.feedback.anchorQ || gap.flushReturned != c.feedback.anchorQ ||
        !IsEDM115ReceiptSameThread(c.feedback, ordinal, m_edm114.retained, guard.lastTick, guard.lastNowUs)) return false;
    c.receipts[index] = c.feedback; c.triggerRows[index] = m_edm114.retained;
    c.completionTick[index] = m_edm114.completionTick; c.completionCommit[index] = m_edm114.completionCommit;
    c.completionUs[index] = guard.lastNowUs; c.retainedEvents = ordinal;
    if (!IsEDM115CurrentSameThread(s.next)) return false;
    LogEDM115SameThread("CURVE_B2_RETAINED");
    return true;
}

void NCManager::LogEDM115SameThread(const char* event) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm115;
    const bool failure = event && std::strcmp(event, "CURVE_B2_FAIL") == 0;
    char record[481]{};
    const auto emit = [&](int length) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
        { if (RtPrintf("%s", record) < 0) RtPrintf("%s", "[EDM115] event=LOG_WRITE_ERROR\n"); }
        else RtPrintf("%s", "[EDM115] event=LOG_FORMAT_ERROR\n");
    };
    const auto& total = failure ? c.feedback : c.retainedEvents ? c.receipts[(std::min)(c.retainedEvents, 2U) - 1U] : c.feedback;
    emit(std::snprintf(record, sizeof(record),
        "[EDM115] event=%.32s session=%llu P=127 finishP=128 eventsStarted=%u eventsCompleted=%u dwells=%u anchorReturns=%u normalRows=%u normalLine=%u normalArc=%u physicalB2=1 triggerSteps=2,4 previousDepthRatio=0.1 expectedEvents=2 expectedDwells=10 requiredDwell=200 gap=RT_SYNTHETIC dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), total.eventsStarted, total.eventsCompleted,
        total.completedDwellsTotal, total.anchorReturnsTotal, m_edm114.acknowledgedRows,
        m_edm114.retained.lineRows, m_edm114.retained.arcRows));
    const unsigned count = failure ? 1U : (std::min)(c.retainedEvents, 2U);
    for (unsigned i = 0U; i < count; ++i)
    {
        const auto& f = failure ? c.feedback : c.receipts[i]; const auto& e = f.excursion;
        emit(std::snprintf(record, sizeof(record),
            "[EDM115] event=B2_EVENT result=%.32s session=%llu step=%u currentKind=%u previousKind=%u epoch=%llu dispatch=%llu previousCommit=%llu mappingTick=%llu physicalTick=%llu endTick=%llu physicalB2=1\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), f.trigger.step,
            static_cast<unsigned>(f.currentKind), static_cast<unsigned>(f.previousKind),
            static_cast<unsigned long long>(f.trigger.epoch), static_cast<unsigned long long>(f.trigger.dispatch),
            static_cast<unsigned long long>(f.previousCommit), static_cast<unsigned long long>(f.previous.mappingTick),
            static_cast<unsigned long long>(f.lastPhysicalTick), static_cast<unsigned long long>(e.tick)));
        emit(std::snprintf(record, sizeof(record),
            "[EDM115] event=B2_PROOF result=%.32s session=%llu step=%u dwells=%u settled=%u backward=%llu forward=%llu anchorReturned=%llu anchorQ=%.12g phase=%u reason=%u physicalB2=1 anchor=%s proof=%s dischargeCommand=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(s.session), f.trigger.step, e.completedDwells,
            e.settledCycles, static_cast<unsigned long long>(e.backwardCrossings), static_cast<unsigned long long>(e.forwardCrossings),
            static_cast<unsigned long long>(e.anchorReturned), f.anchorQ, static_cast<unsigned>(e.phase), static_cast<unsigned>(e.reason),
            failure ? "UNPROVEN" : "CANONICAL_ROW_EXACT", failure ? "INCOMPLETE" : "FOLLOWING_WINDOW_200"));
    }
}

bool NCManager::GuardEDM114ArcSameThread(const NCBlock& block) noexcept
{
    auto& s = m_edm94;
    if (!s.latched) return !s.program;
    if (!s.gapPhysicalCurve || !s.ready || s.armPending || s.cancelled || s.finishPending || s.complete ||
        s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported || s.gapWaiting ||
        s.next == 0U || s.next > s.moves || m_isSingleBlockEnabled ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        !IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) ||
        CoordSys.GetTranslationSnapshot().distanceMode != 91 || CoordSys.activePlane != 17 ||
        !IsXYZCUVFeedProfileCurrentSameThread() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    const unsigned step = s.next - 1U;
    const int pc = GetActiveDispatchPC();
    if (!EDM116::IsArcStep(step, s.gapModeFixture, s.authoredGCode[step]) || !IsEDM114CurrentSameThread(step) ||
        pc != s.pc[step] || !m_currentExecutingBlockDispatchId ||
        block.gCode != EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]) ||
        !block.has('F') || block.val('F') != s.feed[step]) return false;
    if (s.gapCurveB2 && !IsEDM115CurrentSameThread(step)) return false;
    const bool sparse = s.gapModeFixture == EDM116::Fixture::ModalSparseArc155 || s.gapModeFixture == EDM116::Fixture::ModalProgram156 || s.gapModeFixture == EDM116::Fixture::ModalPause157 || s.gapModeFixture == EDM116::Fixture::ModalXYZVisibleFlush159 || s.gapModeFixture == EDM116::Fixture::ModalSixVisibleFlush167;
    std::array<double, 2U> center{};
    if (sparse)
    {
        const auto* authored = GetBaseProgramCache().TryGetLine(pc);
        if (!((s.gapModeFixture == EDM116::Fixture::ModalSparseArc155 && EDM149::IsSparseArcTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::SparseArcHistoryFixture) ||
            (s.gapModeFixture == EDM116::Fixture::ModalProgram156 && EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::ProgramHistoryFixture) ||
            (s.gapModeFixture == EDM116::Fixture::ModalPause157 && EDM149::IsPauseTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::PauseHistoryFixture) ||
            (s.gapModeFixture == EDM116::Fixture::ModalSixVisibleFlush167 && EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::SixVisibleFlushHistoryFixture) ||
            (s.gapModeFixture == EDM116::Fixture::ModalXYZVisibleFlush159 && EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::XYZVisibleFlushHistoryFixture)) || !authored) return false;
        NCParsedBlock geometry = authored->parsedBlock;
        if (geometry.has('B') != m_edm127.authoredInline[step]) return false;
        if (geometry.has('B'))
        {
            double value = 0.0; EDM116::Mode mode = EDM116::Mode::Invalid;
            if (!E94Number(geometry.expression('B'), value) || !EDM116::ParseMode(value, mode) ||
                mode != m_edm127.expectedMode[step]) return false;
            geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
        }
        std::array<double, 6U> literalIncrement{}; double literalFeed = 0.0;
        if (!E94Matches(geometry, block) || !E114Move(geometry, step, literalIncrement, literalFeed, s.gapModeFixture) ||
            literalIncrement != s.increment[step] || literalFeed != s.feed[step] ||
            !E155ArcCenter(geometry, center, s.gapModeFixture)) return false;
    }
    else
    {
        if (!block.has('X') || !block.has('Y') || !block.has('I') || !block.has('J')) return false;
        center = {block.val('I'), block.val('J')};
    }
    if (!EDM116::ValidArcRecipe(block.gCode, step, s.increment[step], center, block.val('F'), s.gapModeFixture) ||
        (block.has('X') ? block.val('X') != s.increment[step][0] : s.increment[step][0] != 0.0) ||
        (block.has('Y') ? block.val('Y') != s.increment[step][1] : s.increment[step][1] != 0.0) ||
        center != m_pathArcCenterOffset) return false;
    std::array<double, 6U> target{};
    for (unsigned a = 0U; a < 6U; ++a)
    {
        if (a >= 2U && block.has("XYZCUV"[a])) return false;
        target[a] = m_motion.GetAxisContext(static_cast<int>(a)).logicalCmdPos.Load() +
            s.increment[step][a] * s.feedback.config.axis[a].pulsePerUnit;
    }
    const MotionExecutionEpoch epoch = m_motion.GetCurrentExecutionEpoch();
    EDM96::Feedback gap{};
    if (!m_motion.CheckEDM94Endpoint(target, epoch, s.lease) || !m_motion.ReadEDM96Feedback(gap) ||
        gap.session != s.session || gap.epoch != epoch || gap.ownerGeneration != s.lease.generation ||
        gap.expectedStep != step || !gap.active || gap.faulted || gap.reason != EDM96::Reason::None ||
        !IsEDM97TuningCurrentSameThread(gap)) return false;
    if (s.gapHistoryMode) return AuthorizeEDM128SameThread(block, block.gCode, center);
    if (s.gapHistoryServo)
        return IsEDM126CurrentSameThread(step) && m_motion.AuthorizeEDM126Feed(s.session, s.run, s.cache,
            m_currentExecutingBlockDispatchId, pc, step, block.gCode, s.increment[step], center,
            s.feed[step], epoch, s.lease, s.gapTuning, m_edm114.retained.source,
            m_edm114.completionCommit, m_edm114.completionTick);
    if (s.gapModeFlush) return AuthorizeEDM116SameThread(block, block.gCode, center);
    if (s.gapCurveB2)
    {
        if (!BuildEDM115CompletionTokenSameThread(step)) return false;
        return m_motion.AuthorizeEDM115Feed(s.session, s.run, s.cache, m_currentExecutingBlockDispatchId,
            pc, step, block.gCode, s.increment[step], center, s.feed[step], epoch, s.lease,
            s.gapTuning, s.gapFlushParameters, m_edm115.token);
    }
    return m_motion.AuthorizeEDM114Feed(s.session, s.run, s.cache, m_currentExecutingBlockDispatchId,
        pc, step, block.gCode, s.increment[step], center, s.feed[step], epoch, s.lease,
        s.gapTuning, s.gapFlushParameters);
}

bool NCManager::IsEDM114CurrentSameThread(unsigned completed) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm114;
    if (!s.gapPhysicalCurve || (!s.gapHistoryServo && (!s.gapFlushProfile || !s.gapFlushParameterized)) || s.gapCommonPath || s.gapB2CrossSegment ||
        s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime || s.gapFlushInterruptArm ||
        !s.latched || s.armPending || !s.everArmed || !s.armEpoch || !E156BoundPlanCurrent(s, m_edm130.recipe) ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        completed > s.moves || c.acknowledgedRows != completed ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_motion.ReadEDM114Feedback(c.feedback)) return false;
    const auto& f = c.feedback;
    if (f.failed && !c.failureLogged)
    { c.failureLogged = true; LogEDM114SameThread("PHYSICAL_CURVE_FAIL"); }
    if (!(s.gapHistoryMode ? E128Counts(f, completed, s.gapModeFixture, s.gapWorkTime, &s.authoredGCode, s.moves) : s.gapHistoryServo ? E126Counts(f, completed, s.gapModeFixture) : E114Counts(f, completed, s.gapModeFixture)) || f.session != s.session || f.armEpoch != s.armEpoch ||
        f.ownerGeneration != s.lease.generation) return false;
    if (completed == 0U)
    {
        if (!EDM114::SameSource(f.source, EDM114::Source{}) || f.lastTick || f.lastMappingTick || f.lastMappingUs ||
            f.samples || f.forwardSamples || f.reverseSamples || f.flushSamples ||
            f.rowSamples || f.rowForward || f.rowReverse || f.rowFlush ||
            f.pulseLength != 0.0 || f.radiusPulse != 0.0 || f.startAngle != 0.0 || f.sweepRadians != 0.0 ||
            f.centerPulse[0] != 0.0 || f.centerPulse[1] != 0.0 || c.completionTick || c.completionCommit || c.arcFlushRows)
            return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (f.startPulse[a] != 0.0 || f.endPulse[a] != 0.0 || f.pulsePerNative[a] != 0.0) return false;
        c.retained = f; return true;
    }
    if (!EDM114::ValidSource(f.source) || f.source.scope.session != s.session ||
        f.source.scope.run != s.run || f.source.scope.cache != s.cache ||
        f.source.scope.ownerGeneration != s.lease.generation || f.source.step != completed - 1U ||
        f.source.sourcePC != s.pc[completed - 1U] || !c.completionTick || !c.completionCommit ||
        f.lastMappingTick > c.completionTick || c.arcFlushRows != (s.gapHistoryServo ? 0U : EDM116::ExpectedArcFlushRows(completed, s.gapModeFixture)) ||
        !E114SameFeedback(f, c.retained)) return false;
    return true;
}

bool NCManager::RetainEDM114CompletedSameThread(const EDM94::Feedback& guard) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm114; const auto& b = s.boundary;
    if (!s.gapPhysicalCurve || (!s.gapHistoryServo && (!s.gapFlushProfile || !s.gapFlushParameterized)) || s.gapCommonPath ||
        s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime || s.gapFlushInterruptArm ||
        !s.latched || !s.ready || s.armPending || s.cancelled || s.resetSeen ||
        s.resetCommitted || s.releaseRequested || s.rtFaultReported || s.finishPending || s.complete ||
        s.next == 0U || s.next > s.moves || !E156BoundPlanCurrent(s, m_edm130.recipe) ||
        !b.dispatch || !b.commit || !b.sequence || !b.identity.IsAssigned() ||
        b.identity.source != MotionCommandSource::NC_MEMORY ||
        b.identity.sourceBlockId != static_cast<MotionSourceBlockId>(s.pc[s.next - 1U]) ||
        !m_motion.ReadEDM114Feedback(c.feedback)) return false;
    const auto& f = c.feedback; const auto& prior = c.retained;
    const unsigned step = s.next - 1U;
    if (f.failed && !c.failureLogged)
    { c.failureLogged = true; LogEDM114SameThread("PHYSICAL_CURVE_FAIL"); }
    if (!(s.gapHistoryMode ? E128Counts(f, s.next, s.gapModeFixture, s.gapWorkTime, &s.authoredGCode, s.moves) : s.gapHistoryServo ? E126Counts(f, s.next, s.gapModeFixture) : E114Counts(f, s.next, s.gapModeFixture)) || f.session != s.session || f.armEpoch != s.armEpoch ||
        f.ownerGeneration != s.lease.generation || !EDM114::ValidSource(f.source) ||
        f.source.scope.session != s.session || f.source.scope.run != s.run || f.source.scope.cache != s.cache ||
        f.source.scope.ownerGeneration != s.lease.generation || f.source.scope.translationGeneration != b.translation ||
        f.source.dispatch != b.dispatch || f.source.epoch != b.identity.epoch || f.source.segment != b.identity.segmentId ||
        f.source.sourcePC != s.pc[step] || f.source.step != step ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != f.source.epoch || guard.currentIdentity.ownerGeneration != s.lease.generation ||
        !guard.active || !guard.armAccepted || !guard.armedReady || guard.forceZero || guard.handoffZero ||
        guard.faulted || guard.cancelled || guard.finishRequested || guard.released ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        f.lastMappingTick > guard.lastTick || f.lastMappingUs > guard.lastNowUs) return false;
    std::uint64_t configTag = 1469598103934665603ULL, mappingTag = configTag;
    for (unsigned a = 0U; a < 6U; ++a)
    {
        const auto& frozen = s.feedback.config.axis[a]; const auto& seen = guard.config.axis[a];
        if (!EDM109::SameDouble(f.pulsePerNative[a], frozen.pulsePerUnit) ||
            !EDM109::SameDouble(f.pulsePerNative[a], seen.pulsePerUnit) ||
            !EDM109::SameDouble(f.endPulse[a], b.endPulse[a]) ||
            seen.configFingerprint != frozen.configFingerprint || seen.mappingFingerprint != frozen.mappingFingerprint)
            return false;
        configTag = (configTag ^ seen.configFingerprint) * 1099511628211ULL;
        mappingTag = (mappingTag ^ seen.mappingFingerprint) * 1099511628211ULL;
    }
    if (f.source.scope.configTag != (configTag ? configTag : 1ULL) ||
        f.source.scope.mappingTag != (mappingTag ? mappingTag : 1ULL)) return false;
    if (EDM116::IsArcStep(step, s.gapModeFixture, s.authoredGCode[step]))
    {
        const auto& r = m_pathArcMotion.receipt;
        if (f.kind != EDM114::Kind::Arc || m_pathArc.dispatch != b.dispatch || m_pathArc.commit != b.commit ||
            r.identity.epoch != f.source.epoch || r.identity.segmentId != f.source.segment ||
            !r.valid || !r.arc.valid || r.arc.axisMask != 3U || r.arc.plane != 17U || r.arc.fullCircle ||
            r.arc.direction != (EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]) == 2 ? -1 : 1) ||
            !EDM109::SameDouble(f.pulseLength, r.arc.lengthPulse) ||
            !EDM109::SameDouble(f.radiusPulse, r.arc.radiusPulse) ||
            !EDM109::SameDouble(f.startAngle, r.arc.startAngle) ||
            !EDM109::SameDouble(f.sweepRadians, r.arc.sweepRadians) ||
            !EDM109::SameDouble(f.centerPulse[0], r.arc.centerPulse[0]) ||
            !EDM109::SameDouble(f.centerPulse[1], r.arc.centerPulse[1])) return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (!EDM109::SameDouble(f.startPulse[a], r.arc.startPulse[a]) ||
                !EDM109::SameDouble(f.endPulse[a], r.arc.endPulse[a])) return false;
    }
    else
    {
        const auto& r = m_pathFeedMotion.receipt;
        if (f.kind != EDM114::Kind::Line || m_pathFeed.dispatch != b.dispatch || m_pathFeed.commit != b.commit ||
            r.identity.epoch != f.source.epoch || r.identity.segmentId != f.source.segment ||
            !r.valid || !r.xyzcuvFeed || !r.xyzcuv.valid || r.xyzcuv.axisMask != 63U ||
            !EDM109::SameDouble(f.pulseLength, r.xyzcuv.lengthPulse) || f.radiusPulse != 0.0 ||
            f.startAngle != 0.0 || f.sweepRadians != 0.0 || f.centerPulse[0] != 0.0 || f.centerPulse[1] != 0.0) return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (!EDM109::SameDouble(f.startPulse[a], r.xyzcuv.startPulse[a]) ||
                !EDM109::SameDouble(f.endPulse[a], r.xyzcuv.endPulse[a]) ||
                !EDM109::SameDouble(f.pulsePerNative[a], r.xyzcuv.pulsePerUnit[a])) return false;
    }
    if (c.acknowledgedRows == s.next)
        return c.completionCommit == b.commit && c.completionTick <= guard.lastTick && IsEDM114CurrentSameThread(s.next);
    if (c.acknowledgedRows + 1U != s.next || !prior.enabled || prior.failed ||
        prior.session != s.session || prior.armEpoch != s.armEpoch || prior.ownerGeneration != s.lease.generation ||
        prior.completedRows != c.acknowledgedRows || f.lastMappingTick <= prior.lastMappingTick ||
        f.lastMappingUs <= prior.lastMappingUs || b.commit <= c.completionCommit || guard.lastTick <= c.completionTick ||
        f.samples < prior.samples || f.samples - prior.samples != f.rowSamples ||
        f.forwardSamples < prior.forwardSamples || f.forwardSamples - prior.forwardSamples != f.rowForward ||
        f.reverseSamples < prior.reverseSamples || f.reverseSamples - prior.reverseSamples != f.rowReverse ||
        f.flushSamples < prior.flushSamples || f.flushSamples - prior.flushSamples != f.rowFlush) return false;
    if (c.acknowledgedRows != 0U && (!EDM109::SameScope(f.source.scope, prior.source.scope) ||
        f.source.dispatch <= prior.source.dispatch || f.source.epoch <= prior.source.epoch ||
        f.source.segment <= prior.source.segment || f.source.sourcePC <= prior.source.sourcePC ||
        !EDM109::SamePulse(f.startPulse, prior.endPulse))) return false;
    if (!s.gapHistoryServo && EDM116::IsArcStep(step, s.gapModeFixture, s.authoredGCode[step]) && !(step & 1U)) ++c.arcFlushRows;
    c.retained = f; c.acknowledgedRows = s.next;
    c.completionTick = guard.lastTick; c.completionCommit = b.commit;
    if(m_edm237.authorized&&s.gapHistoryArm==274U&&
        !BindEDM237ParentEpochSameThread(f.source.epoch,f.source.dispatch,f.source.sourcePC))return false;
    if(m_edm234.authorized&&s.gapHistoryArm==274U&&
        !BindEDM234ParentEpochSameThread(f.source.epoch,f.source.dispatch,f.source.sourcePC))return false;
    LogEDM114SameThread("PHYSICAL_CURVE_ROW");
    return true;
}

void NCManager::LogEDM114SameThread(const char* event) const noexcept
{
    if (m_edm94.gapHistoryServo) { LogEDM126SameThread(event); return; }
    if (EDM116::IsModalFixture(m_edm94.gapModeFixture)) { LogEDM127SameThread(event, EDM114::RequiredRows); return; }
    const auto& f = event && std::strcmp(event, "PHYSICAL_CURVE_FAIL") == 0 ? m_edm114.feedback : m_edm114.retained;
    char record[481]{};
    const char* format = m_edm94.gapModeFlush ?
        (m_edm94.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=%.32s session=%llu P=154 finishP=155 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE profile=PREVIOUS_ARC_RETREAT armP=154\n" : m_edm94.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=%.32s session=%llu P=156 finishP=157 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE profile=PREVIOUS_ARC_RETURN armP=156\n" : m_edm94.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=%.32s session=%llu P=146 finishP=147 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE profile=B0 armP=146\n" : m_edm94.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=%.32s session=%llu P=142 finishP=143 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE profile=B0 armP=142\n" : m_edm94.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=%.32s session=%llu P=148 finishP=149 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE profile=B1 armP=148\n" : m_edm94.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=%.32s session=%llu P=144 finishP=145 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE profile=B1 armP=144\n" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=%.32s session=%llu P=140 finishP=141 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE\n" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=%.32s session=%llu P=138 finishP=139 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE\n" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=%.32s session=%llu P=136 finishP=137 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE\n" : m_edm94.gapModeInterruption ? "[EDM118] event=%.32s session=%llu P=134 finishP=135 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE\n" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=%.32s session=%llu P=150 finishP=151 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE fixture=2\n" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=%.32s session=%llu P=152 finishP=153 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE fixture=3\n" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=%.32s session=%llu P=132 finishP=133 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE\n" : "[EDM116] event=%.32s session=%llu P=129 finishP=130 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 modeFlush=1 dischargeCommand=NONE\n") : m_edm94.gapCurveB2 ?
        "[EDM115] event=%.32s session=%llu P=127 finishP=128 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 crossRowB2=1 dischargeCommand=NONE\n" :
        "[EDM114] event=%.32s session=%llu P=125 finishP=126 rows=%u line=%u arc=%u cw=%u ccw=%u endpoints=%u flushCycles=%u arcFlush=%u step=%u reason=%u tick=%llu samples=%llu forward=%llu reverse=%llu flush=%llu physicalArc=1 packet=ARC2 crossRowB2=0 dischargeCommand=NONE\n";
    const int length = std::snprintf(record, sizeof(record), format,
        event ? event : "UNKNOWN", static_cast<unsigned long long>(m_edm94.session), f.completedRows,
        f.lineRows, f.arcRows, f.cwRows, f.ccwRows, f.endpointRows, (m_edm114.acknowledgedRows + 1U) / 2U, m_edm114.arcFlushRows,
        f.source.step, static_cast<unsigned>(f.reason), static_cast<unsigned long long>(f.lastMappingTick),
        static_cast<unsigned long long>(f.samples), static_cast<unsigned long long>(f.forwardSamples),
        static_cast<unsigned long long>(f.reverseSamples), static_cast<unsigned long long>(f.flushSamples));
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
    {
        if (RtPrintf("%s", record) < 0) RtPrintf("%s", m_edm94.gapModeFlush ? (m_edm94.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=LOG_WRITE_ERROR profile=PREVIOUS_ARC_RETREAT armP=154\n" : m_edm94.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=LOG_WRITE_ERROR profile=PREVIOUS_ARC_RETURN armP=156\n" : m_edm94.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=LOG_WRITE_ERROR profile=B0 armP=146\n" : m_edm94.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=LOG_WRITE_ERROR profile=B0 armP=142\n" : m_edm94.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=LOG_WRITE_ERROR profile=B1 armP=148\n" : m_edm94.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=LOG_WRITE_ERROR profile=B1 armP=144\n" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=LOG_WRITE_ERROR\n" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=LOG_WRITE_ERROR\n" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=LOG_WRITE_ERROR\n" : m_edm94.gapModeInterruption ? "[EDM118] event=LOG_WRITE_ERROR\n" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=LOG_WRITE_ERROR fixture=2\n" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=LOG_WRITE_ERROR fixture=3\n" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=LOG_WRITE_ERROR\n" : "[EDM116] event=LOG_WRITE_ERROR\n") : m_edm94.gapCurveB2 ? "[EDM115] event=LOG_WRITE_ERROR\n" : "[EDM114] event=LOG_WRITE_ERROR\n");
    }
    else RtPrintf("%s", m_edm94.gapModeFlush ? (m_edm94.gapModeInterruptionArm == EDM125::RetreatArmP ? "[EDM125] event=LOG_FORMAT_ERROR profile=PREVIOUS_ARC_RETREAT armP=154\n" : m_edm94.gapModeInterruptionArm == EDM125::ReturnArmP ? "[EDM125] event=LOG_FORMAT_ERROR profile=PREVIOUS_ARC_RETURN armP=156\n" : m_edm94.gapModeInterruptionArm == EDM123::B0ArmP ? "[EDM123] event=LOG_FORMAT_ERROR profile=B0 armP=146\n" : m_edm94.gapModeInterruptionArm == EDM122::B0ArmP ? "[EDM122] event=LOG_FORMAT_ERROR profile=B0 armP=142\n" : m_edm94.gapModeInterruptionArm == EDM123::B1ArmP ? "[EDM123] event=LOG_FORMAT_ERROR profile=B1 armP=148\n" : m_edm94.gapModeInterruptionArm == EDM122::B1ArmP ? "[EDM122] event=LOG_FORMAT_ERROR profile=B1 armP=144\n" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "[EDM121] event=LOG_FORMAT_ERROR\n" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "[EDM120] event=LOG_FORMAT_ERROR\n" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "[EDM119] event=LOG_FORMAT_ERROR\n" : m_edm94.gapModeInterruption ? "[EDM118] event=LOG_FORMAT_ERROR\n" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "[EDM124] event=LOG_FORMAT_ERROR fixture=2\n" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "[EDM124] event=LOG_FORMAT_ERROR fixture=3\n" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "[EDM117] event=LOG_FORMAT_ERROR\n" : "[EDM116] event=LOG_FORMAT_ERROR\n") : m_edm94.gapCurveB2 ? "[EDM115] event=LOG_FORMAT_ERROR\n" : "[EDM114] event=LOG_FORMAT_ERROR\n");
}

bool NCManager::WaitEDM114ArcBoundarySameThread()
{
    auto& s = m_edm94; auto& b = s.boundary;
    const auto& p = m_pathArc; const auto& r = m_pathArcMotion.receipt;
    if (!s.gapPhysicalCurve || !s.latched || !s.ready || s.armPending || s.cancelled || s.finishPending || s.complete ||
        s.resetSeen || s.resetCommitted || s.releaseRequested || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || Close_System_Com_flag ||
        !m_externalReadyInterlock || AlarmManager::GetInstance().HasAlarm())
    { b = EDM94NCState::FeedBoundary{}; return false; }
    const bool stepValid = s.next > 0U && s.next <= s.moves;
    const unsigned step = stepValid ? s.next - 1U : 0U;
    bool scope = stepValid && EDM116::IsArcStep(step, s.gapModeFixture, s.authoredGCode[step]) && p.armed && p.pending && p.bound && p.completed &&
        p.consumerAccepted && p.consumerStarted && p.dispatch && p.commit && p.lastSequence &&
        p.run == s.run && p.cache == s.cache && s.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        s.cache == GetBaseProgramCache().GetGeneration() && p.dispatch == m_waitingBlockDispatchId &&
        p.sourcePC == GetActiveDispatchPC() && p.sourcePC == s.pc[step] &&
        r.valid && r.commandAccepted && r.tailCommitted && r.captureBound && !r.travelLimitRejected &&
        r.code == MotionFeedArcCode::COMMITTED &&
        r.geometryCode == static_cast<std::uint32_t>(NCPathCoreFeedArcCode::BUILT_ARC) && r.arc.valid &&
        r.arc.axisMask == 3U && r.validAxisMask == 63U && r.arc.plane == 17U && !r.arc.fullCircle &&
        r.arc.direction == (EDM116::ExpectedGCode(step, s.gapModeFixture, s.authoredGCode[step]) == 2 ? -1 : 1) && r.arc.feedMMMin == s.feed[step] &&
        r.identity.IsAssigned() && r.identity.source == MotionCommandSource::NC_MEMORY &&
        r.identity.sourceBlockId == static_cast<MotionSourceBlockId>(p.sourcePC) &&
        r.identity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        r.ownerLease.Matches(s.lease) && r.ownerLease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(r.ownerLease) && r.translationGeneration &&
        CoordSys.IsTranslationRunFrozen() && CoordSys.IsTranslationRunCurrent() &&
        CoordSys.GetTranslationSnapshot().generation == r.translationGeneration &&
        IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) &&
        CoordSys.GetTranslationSnapshot().distanceMode == 91 && CoordSys.activePlane == 17 &&
        IsXYZCUVFeedProfileCurrentSameThread() && IsFixedTranslationTravelCurrentSameThread() &&
        IsPathCoreLiveNativeConfigCurrentSameThread();
    for (unsigned a = 0U; a < 6U; ++a)
    {
        scope = scope && std::isfinite(r.arc.startPulse[a]) && std::isfinite(r.arc.endPulse[a]) &&
            std::isfinite(r.arc.startMCS[a]) && std::isfinite(r.arc.endMCS[a]);
        if (a < 2U)
        {
            // The native start may be reconstructed from the accepted pulse.
            // Bound only this derived subtraction; authored literals and the
            // actual pulse receipt retained below remain exact.
            const double scale = (std::max)(1.0, (std::max)(std::fabs(s.increment[step][a]),
                (std::max)(std::fabs(r.arc.startMCS[a]), std::fabs(r.arc.endMCS[a]))));
            const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() * scale;
            scope = scope && std::isfinite(tolerance) && tolerance <= 5.0e-8 &&
                std::fabs((r.arc.endMCS[a] - r.arc.startMCS[a]) - s.increment[step][a]) <= tolerance;
        }
        else scope = scope && EDM109::SameDouble(r.arc.startMCS[a], r.arc.endMCS[a]) &&
            EDM109::SameDouble(r.arc.startPulse[a], r.arc.endPulse[a]);
    }
    if (EDM116::IsBoundedArcFixture(s.gapModeFixture))
    {
        // Re-read only this immutable admitted cache row. Neither a later NC
        // selection nor a merely valid arc may replace the frozen literal IJ.
        const auto* authored = GetBaseProgramCache().TryGetLine(p.sourcePC);
        std::array<double, 6U> literalIncrement{}; std::array<double, 2U> literalCenter{};
        double literalFeed = 0.0;
        bool literalScope = scope && s.gapHistoryServo && s.gapHistoryMode && s.gapWorkTime &&
            ((s.gapModeFixture == EDM116::Fixture::ModalArc152 && EDM149::IsArcTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::ArcHistoryFixture) ||
                (s.gapModeFixture != EDM116::Fixture::ModalXYZVisibleFlush159 && s.gapModeFixture != EDM116::Fixture::ModalSixVisibleFlush167 &&
                    EDM116::IsAuthoredOrderFixture(s.gapModeFixture) && EDM149::IsOrderTestArm(s.gapHistoryArm) &&
                    E149HistoryFixture(s.gapHistoryArm) == EDM149::OrderHistoryFixture) ||
                (s.gapModeFixture == EDM116::Fixture::ModalSparse154 && EDM149::IsSparseTestArm(s.gapHistoryArm) &&
                    E149HistoryFixture(s.gapHistoryArm) == EDM149::SparseHistoryFixture) ||
                (s.gapModeFixture == EDM116::Fixture::ModalSparseArc155 && EDM149::IsSparseArcTestArm(s.gapHistoryArm) &&
                    E149HistoryFixture(s.gapHistoryArm) == EDM149::SparseArcHistoryFixture) ||
                (s.gapModeFixture == EDM116::Fixture::ModalProgram156 && EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) &&
                    E149HistoryFixture(s.gapHistoryArm) == EDM149::ProgramHistoryFixture) ||
                (s.gapModeFixture == EDM116::Fixture::ModalPause157 && EDM149::IsPauseTestArm(s.gapHistoryArm) &&
                    E149HistoryFixture(s.gapHistoryArm) == EDM149::PauseHistoryFixture) ||
                (s.gapModeFixture == EDM116::Fixture::ModalSixVisibleFlush167 && EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::SixVisibleFlushHistoryFixture) ||
            (s.gapModeFixture == EDM116::Fixture::ModalXYZVisibleFlush159 && EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) &&
                    E149HistoryFixture(s.gapHistoryArm) == EDM149::XYZVisibleFlushHistoryFixture)) &&
            s.cache == GetBaseProgramCache().GetGeneration() && p.sourcePC == s.pc[step] && authored;
        if (literalScope)
        {
            NCParsedBlock geometry = authored->parsedBlock;
            literalScope = geometry.has('B') == m_edm127.authoredInline[step];
            if (geometry.has('B'))
            {
                double value = 0.0; EDM116::Mode mode = EDM116::Mode::Invalid;
                literalScope = literalScope && E94Number(geometry.expression('B'), value) &&
                    EDM116::ParseMode(value, mode) && mode == m_edm127.expectedMode[step];
                geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
            }
            literalScope = literalScope && E114Move(geometry, step, literalIncrement, literalFeed, s.gapModeFixture) &&
                literalIncrement == s.increment[step] && literalFeed == s.feed[step] &&
                (!EDM116::IsAuthoredOrderFixture(s.gapModeFixture) ||
                    EDM77::Detail::EqualLiteral(geometry.gExpressions[0], s.authoredGCode[step])) &&
                E155ArcCenter(geometry, literalCenter, s.gapModeFixture);
        }
        scope = literalScope && r.arc.centerMCS[0] == r.arc.startMCS[0] + literalCenter[0] &&
            r.arc.centerMCS[1] == r.arc.startMCS[1] + literalCenter[1];
    }
    else scope = scope && r.arc.centerMCS[0] == r.arc.startMCS[0] + ((step & 1U) ? -0.001 : 0.001) &&
        r.arc.centerMCS[1] == r.arc.startMCS[1];
    scope = scope && std::isfinite(r.arc.radiusPulse) && r.arc.radiusPulse > 0.0 &&
        std::isfinite(r.arc.lengthPulse) && r.arc.lengthPulse > 0.0 &&
        std::isfinite(r.arc.sweepRadians) && std::fabs(r.arc.sweepRadians) <= 2.0 * EDM114::Pi;
    if (scope && b.dispatch)
    {
        scope = b.dispatch == p.dispatch && b.commit == p.commit && b.sequence == p.lastSequence &&
            b.translation == r.translationGeneration && b.identity.epoch == r.identity.epoch &&
            b.identity.segmentId == r.identity.segmentId && b.identity.sourceBlockId == r.identity.sourceBlockId &&
            b.identity.source == r.identity.source;
        for (unsigned a = 0U; a < 6U; ++a) scope = scope && EDM109::SameDouble(b.endPulse[a], r.arc.endPulse[a]);
    }
    if (!scope)
    { b = EDM94NCState::FeedBoundary{}; RejectEDM94SameThread("EDM114_ARC_BOUNDARY_SCOPE", p.sourceLine); return false; }
    if (!b.dispatch)
    {
        b.identity = r.identity; b.dispatch = p.dispatch; b.commit = p.commit;
        b.sequence = p.lastSequence; b.translation = r.translationGeneration;
        for (unsigned a = 0U; a < 6U; ++a) b.endPulse[a] = r.arc.endPulse[a];
    }
    return WaitEDM94BoundaryProofSameThread(p.sourceLine);
}

bool NCManager::WaitEDM94FeedBoundarySameThread()
{
    auto& s = m_edm94; auto& b = s.boundary;
    const auto& p = m_pathFeed; const auto& r = m_pathFeedMotion.receipt;
    const bool stepValid = s.next > 0U && s.next <= s.moves;
    const unsigned step = stepValid ? s.next - 1U : 0U;
    // This wait keeps the completed G1's Epoch/AUTO correction authority.
    // It publishes no handoff, new segment, finish request or PDO command.
    if (!s.latched || !s.ready || s.armPending || s.cancelled || s.finishPending || s.complete ||
        s.resetSeen || s.resetCommitted || s.releaseRequested || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || Close_System_Com_flag ||
        !m_externalReadyInterlock || AlarmManager::GetInstance().HasAlarm())
    { b = EDM94NCState::FeedBoundary{}; return false; }
    bool scope = stepValid && p.armed && p.pending && p.bound && p.completed &&
        p.consumerAccepted && p.consumerStarted && p.dispatch != 0ULL && p.commit != 0ULL && p.lastSequence != 0ULL &&
        p.run == s.run && p.cache == s.cache && s.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        // ExecuteBlock's dispatch field is cleared before asynchronous waits.
        // The completion binding retains the exact dispatch until callback retirement.
        s.cache == GetBaseProgramCache().GetGeneration() && p.dispatch == m_waitingBlockDispatchId &&
        p.sourcePC == GetActiveDispatchPC() && p.sourcePC == s.pc[step] &&
        r.valid && r.commandAccepted && r.tailCommitted && r.captureBound && r.xyzcuvFeed &&
        !r.eccentricFeed && !r.rotaryFeed && !r.zcFeed && !r.xyzcFeed &&
        r.xyzcuv.valid && !r.xyzcuv.absolute && r.xyzcuv.axisMask == 63U && (r.validAxisMask & 63U) == 63U &&
        r.identity.IsAssigned() && r.identity.source == MotionCommandSource::NC_MEMORY &&
        r.identity.sourceBlockId == static_cast<MotionSourceBlockId>(p.sourcePC) &&
        r.identity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        r.ownerLease.Matches(s.lease) && r.ownerLease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(r.ownerLease) && r.translationGeneration != 0ULL &&
        CoordSys.IsTranslationRunFrozen() && CoordSys.IsTranslationRunCurrent() &&
        CoordSys.GetTranslationSnapshot().generation == r.translationGeneration &&
        IsXYZCUVFeedProfileCurrentSameThread() && IsFixedTranslationTravelCurrentSameThread() &&
        IsPathCoreLiveNativeConfigCurrentSameThread() && r.xyzcuv.feedMMMin == s.feed[step];
    for (unsigned a = 0U; a < 6U; ++a)
        scope = scope && std::isfinite(r.xyzcuv.endPulse[a]) &&
            r.xyzcuv.deltaNative[a] == s.increment[step][a] &&
            r.xyzcuv.pulsePerUnit[a] == s.feedback.config.axis[a].pulsePerUnit;
    if (scope && b.dispatch != 0ULL)
    {
        scope = b.dispatch == p.dispatch && b.commit == p.commit && b.sequence == p.lastSequence &&
            b.translation == r.translationGeneration && b.identity.epoch == r.identity.epoch &&
            b.identity.segmentId == r.identity.segmentId && b.identity.sourceBlockId == r.identity.sourceBlockId &&
            b.identity.source == r.identity.source;
        for (unsigned a = 0U; a < 6U; ++a) scope = scope && b.endPulse[a] == r.xyzcuv.endPulse[a];
    }
    if (!scope)
    {
        char record[640]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM94] event=BOUNDARY_SCOPE_CHECK build=FIX16 session=%llu dispatch=%llu waitingDispatch=%llu executingDispatch=%llu sourcePC=%d activePC=%d expectedPC=%d commit=%llu sequence=%llu epoch=%llu currentEpoch=%llu segment=%llu ownerGen=%u expectedOwnerGen=%u step=%u moves=%u pathFlags=%02X\n",
            static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(p.dispatch),
            static_cast<unsigned long long>(m_waitingBlockDispatchId), static_cast<unsigned long long>(m_currentExecutingBlockDispatchId),
            p.sourcePC, GetActiveDispatchPC(), stepValid ? s.pc[step] : -1,
            static_cast<unsigned long long>(p.commit), static_cast<unsigned long long>(p.lastSequence),
            static_cast<unsigned long long>(r.identity.epoch), static_cast<unsigned long long>(m_motion.GetCurrentExecutionEpoch()),
            static_cast<unsigned long long>(r.identity.segmentId), r.ownerLease.generation, s.lease.generation, step, s.moves,
            (p.armed ? 1U : 0U) | (p.pending ? 2U : 0U) | (p.bound ? 4U : 0U) | (p.completed ? 8U : 0U) |
            (p.consumerAccepted ? 16U : 0U) | (p.consumerStarted ? 32U : 0U));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM94] event=LOG_FORMAT_ERROR scope=BOUNDARY_SCOPE_CHECK\n");
        b = EDM94NCState::FeedBoundary{}; RejectEDM94SameThread("BOUNDARY_SCOPE", p.sourceLine); return false;
    }
    if (b.dispatch == 0ULL)
    {
        b.identity = r.identity; b.dispatch = p.dispatch; b.commit = p.commit;
        b.sequence = p.lastSequence; b.translation = r.translationGeneration;
        for (unsigned a = 0U; a < 6U; ++a) b.endPulse[a] = r.xyzcuv.endPulse[a];
    }
    return WaitEDM94BoundaryProofSameThread(p.sourceLine);
}

bool NCManager::WaitEDM94BoundaryProofSameThread(int sourceLine)
{
    auto& s = m_edm94; auto& b = s.boundary;
    const unsigned step = s.next - 1U;
    ++b.polls;
    // Flags: read, frozen/current identity, active RT state, clock, new tick,
    // canonical incoming readiness, continuous RT dwell, command endpoint, lag.
    unsigned flags = 0U, lagMask = 0U;
    EDM94::Feedback observed{};
    const bool read = m_motion.ReadEDM94Feedback(observed);
    const bool identity = read && observed.identity.session == s.session &&
        observed.identity.epoch == s.armEpoch && observed.identity.ownerGeneration == s.lease.generation &&
        observed.currentIdentity.session == s.session && observed.currentIdentity.epoch == b.identity.epoch &&
        observed.currentIdentity.ownerGeneration == s.lease.generation;
    if (!read) flags |= 1U;
    if (!identity) flags |= 2U;
    if (!read || !observed.active || !observed.armAccepted || !observed.armedReady || observed.forceZero ||
        observed.handoffZero || observed.faulted || observed.cancelled || observed.finishRequested || observed.released) flags |= 4U;
    const bool clock = identity && observed.lastNowUs != 0ULL &&
        (b.lastNowUs == 0ULL || observed.lastNowUs > b.lastNowUs);
    const bool fresh = identity && observed.lastTick != 0ULL && observed.lastTick > b.lastTick &&
        observed.lastTick >= s.lastTick;
    if (!clock) flags |= 8U;
    if (!fresh) flags |= 16U;
    if (!m_motion.HasExactProgramStartQuiescenceAcknowledgement(b.identity.epoch, s.lease)) flags |= 32U;
    if (!read || observed.boundaryReadyCycles < EDM94::BoundaryReadyCycles || observed.boundaryNotReadyMask != 0U) flags |= 64U;
    for (unsigned a = 0U; a < 6U; ++a)
    {
        const auto& c = s.feedback.config.axis[a]; const auto& seen = observed.config.axis[a];
        const auto& e = observed.axis[a];
        const double maxWindow = (a < 3U ? 0.001 : 0.005) * c.pulsePerUnit;
        const bool config = std::isfinite(c.pulsePerUnit) && c.pulsePerUnit > 0.0 &&
            std::isfinite(c.followingWindowPulse) && c.followingWindowPulse > 0.0 &&
            std::isfinite(maxWindow) && c.followingWindowPulse <= maxWindow &&
            seen.pulsePerUnit == c.pulsePerUnit && seen.followingWindowPulse == c.followingWindowPulse &&
            seen.commandAnchor == c.commandAnchor && seen.actualAnchor == c.actualAnchor &&
            seen.hardwareSign == c.hardwareSign && seen.configFingerprint == c.configFingerprint &&
            seen.mappingFingerprint == c.mappingFingerprint;
        if (!read || !config || !std::isfinite(e.currentCommandPulse) ||
            e.currentCommandPulse != b.endPulse[a]) flags |= 128U;
        if (!read || !config || !std::isfinite(e.currentActualPulse) || !std::isfinite(e.currentCommandPulse) ||
            std::fabs(e.currentCommandPulse - e.currentActualPulse) > c.followingWindowPulse ||
            std::fabs(b.endPulse[a] - e.currentActualPulse) > c.followingWindowPulse) lagMask |= 1U << a;
    }
    if (lagMask != 0U) flags |= 256U;
    if (s.gapSegmentProfile && !ReadEDM96CompletedSegmentsSameThread(s.next)) flags |= 512U;
    if (clock && fresh)
    {
        if (b.firstNowUs == 0ULL) b.firstNowUs = observed.lastNowUs;
        b.lastNowUs = observed.lastNowUs; b.lastTick = observed.lastTick;
    }
    const std::uint64_t elapsed = b.firstNowUs != 0ULL && b.lastNowUs >= b.firstNowUs ?
        b.lastNowUs - b.firstNowUs : 0ULL;
    const bool timeout = b.polls >= 500U || elapsed >= 5000000ULL;
    const bool ready = flags == 0U && !timeout;
    if ((b.polls == 1U || ready || timeout) &&
        !(s.gapBoundaryProfile && s.gapWaiting && ready))
        RtPrintf("[EDM94] event=BOUNDARY_%s build=FIX16 session=%llu dispatch=%llu epoch=%llu segment=%llu step=%u flags=%03X lagMask=%02X polls=%u tick=%llu elapsedUs=%llu rtReadyCycles=%u required=%u rtNotReadyMask=%08X witness=RT_INCOMING_READINESS_DWELL stopProof=0\n",
            timeout ? "TIMEOUT" : ready ? "READY" : "WAIT", static_cast<unsigned long long>(s.session),
            static_cast<unsigned long long>(b.dispatch), static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.identity.segmentId), step, flags, lagMask, b.polls,
            static_cast<unsigned long long>(observed.lastTick), static_cast<unsigned long long>(elapsed),
            observed.boundaryReadyCycles, EDM94::BoundaryReadyCycles, observed.boundaryNotReadyMask);
    if (timeout)
    { RejectEDM94SameThread("BOUNDARY_SETTLE_TIMEOUT", sourceLine); return false; }
    if (!ready) return false;
    if (s.gapPhysicalCurve && !RetainEDM114CompletedSameThread(observed))
    { RejectEDM94SameThread("EDM114_COMPLETED_PATH_PROOF", sourceLine); return false; }
    if (s.gapHistoryServo && !IsEDM126CurrentSameThread(s.next))
    { RejectEDM94SameThread("EDM126_COMPLETED_HISTORY_PROOF", sourceLine); return false; }
    if (s.gapHistoryMode && !RetainEDM128CompletedSameThread(observed))
    { RejectEDM94SameThread("EDM128_COMPLETED_COORDINATOR_PROOF", sourceLine); return false; }
    if (s.gapModeFlush && !RetainEDM116CompletedSameThread(observed))
    { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_COMPLETED_MODE_PROOF" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_COMPLETED_MODE_PROOF" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_COMPLETED_MODE_PROOF" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_COMPLETED_MODE_PROOF" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_COMPLETED_MODE_PROOF" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_COMPLETED_MODE_PROOF" : m_edm94.gapModeInterruption ? "EDM118_COMPLETED_MODE_PROOF" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_COMPLETED_MODE_PROOF" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_COMPLETED_MODE_PROOF" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_COMPLETED_MODE_PROOF" : "EDM116_COMPLETED_MODE_PROOF"), sourceLine); return false; }
    if (s.gapCurveB2 && !RetainEDM115CompletedSameThread(observed))
    { RejectEDM94SameThread("EDM115_COMPLETED_B2_PROOF", sourceLine); return false; }
    if (s.gapCommonPath && !RetainEDM109CompletedSameThread(observed))
    { RejectEDM94SameThread("EDM109_COMPLETED_PATH_PROOF", sourceLine); return false; }
    if (s.next == s.moves && !IsEDM168AutoHoldCheckpointCompleteSameThread())
    { RejectEDM94SameThread("EDM168_CHECKPOINT_NOT_COMPLETED", sourceLine); return false; }
    if (s.next == s.moves && m_edm130.recipe.flushHoldTest >= 2U)
        RtPrintf("[EDM168] event=CHECKPOINT_VERIFIED session=%llu L=%u B=%u completedModeEvents=%u queued=1 resumed=1 rows=%u proof=RETAINED_FINAL_BOUNDARY S92=0 S93=0\n",
            static_cast<unsigned long long>(s.session), static_cast<unsigned>(m_edm130.recipe.flushHoldTest),
            EDM130::FlushHoldTargetMode(m_edm130.recipe.flushHoldTest),
            m_edm130.retainedModeEvents[EDM130::FlushHoldTargetMode(m_edm130.recipe.flushHoldTest)], s.moves);
    if (s.gapBoundaryProfile && !s.gapWaiting)
        RtPrintf("[EDM95] event=PHYSICAL_BOUNDARY_READY session=%llu step=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY\n",
            static_cast<unsigned long long>(s.session), step, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick), s.next == s.moves ? "P88_FINISH" : "WAIT_GAP_DIRECTION");
    if (s.gapFlushProfile)
        RtPrintf("[EDM105] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u closedPairs=%u flushCycles=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM105_SEGMENT_COMPLETE proofScope=COMMAND_SCALAR_ONLY\n",
            static_cast<unsigned long long>(s.session), s.next, s.next / 2U, (s.next + 1U) / 2U,
            static_cast<unsigned long long>(b.identity.epoch), static_cast<unsigned long long>(b.lastTick),
            s.next == s.moves ? (s.gapFlushInterruptArm == 111U ? "FINISH_P112" : s.gapFlushInterruptArm == 113U ? "FINISH_P114" :
                s.gapFlushInterruptArm == 105U ? "FINISH_P106" : s.gapFlushInterruptArm == 107U ? "FINISH_P108" :
                s.gapModeFlush ? (s.gapModeInterruption ? "STOP_ONLY_NO_FINISH" : s.gapModeFixture == EDM116::Fixture::ArcPair124 ? "FINISH_P151" : s.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "FINISH_P153" : s.gapModeFixture == EDM116::Fixture::Complement117 ? "FINISH_P133" : "FINISH_P130") : s.gapCurveB2 ? "FINISH_P128" : s.gapPhysicalCurve ? "FINISH_P126" : s.gapCurveRuntime ? "FINISH_P124" : s.gapCurveGeometry ? "FINISH_P122" : s.gapB2Repeat ? "FINISH_P120" : s.gapB2CrossSegment ? "FINISH_P118" : s.gapCommonPath ? "FINISH_P116" : s.gapFlushParameterized ? "FINISH_P110" : "FINISH_P104") : "NEXT_AUTHORED_G1");
    else if (s.gapPairedProfile)
        RtPrintf("[EDM104] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u closedPairs=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM104_SEGMENT_COMPLETE proofScope=COMMAND_SCALAR_ONLY\n",
            static_cast<unsigned long long>(s.session), s.next, s.next / 2U, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick),
            s.next == s.moves ? "FINISH_P102" : "NEXT_AUTHORED_G1");
    else if (s.gapEnduranceProfile)
        RtPrintf("[EDM102] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM102_SEGMENT_COMPLETE proofScope=COMMAND_SCALAR_ONLY\n",
            static_cast<unsigned long long>(s.session), s.next, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick),
            s.next == s.moves ? "FINISH_P100" : "NEXT_AUTHORED_G1");
    else if (s.gapContinuousProfile)
        RtPrintf("[EDM100] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM100_SEGMENT_COMPLETE proofScope=COMMAND_SCALAR_ONLY\n",
            static_cast<unsigned long long>(s.session), s.next, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick),
            s.next == s.moves ? "FINISH_P98" : "NEXT_AUTHORED_G1");
    else if (s.gapBurstProfile)
        RtPrintf("[EDM99] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM99_SEGMENT_COMPLETE proofScope=COMMAND_SCALAR_ONLY\n",
            static_cast<unsigned long long>(s.session), s.next, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick),
            s.next == s.moves ? "FINISH_P96" : "NEXT_AUTHORED_G1");
    else if (s.gapNoiseProfile)
        RtPrintf("[EDM98] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM98_SEGMENT_COMPLETE quietScope=COMMAND_SCALAR_ONLY\n",
            static_cast<unsigned long long>(s.session), s.next, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick),
            s.next == s.moves ? "FINISH_P94" : "NEXT_AUTHORED_G1");
    else if (s.gapGainProfile)
        RtPrintf("[EDM97] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM97_SEGMENT_COMPLETE\n",
            static_cast<unsigned long long>(s.session), s.next, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick),
            s.gapHistoryMode ? (s.next == s.moves ? (s.gapWorkTime ? (s.gapHistoryArm == 290U ? "FINISH_P291" : s.gapHistoryArm == 288U ? "FINISH_P289" : s.gapHistoryArm == 286U ? "FINISH_P287" : s.gapHistoryArm == 284U ? "FINISH_P285" : s.gapHistoryArm == 278U ? "FINISH_P279" : s.gapHistoryArm == 280U ? "FINISH_P281" : s.gapHistoryArm == 282U ? "FINISH_P283" : s.gapHistoryArm == 272U ? "FINISH_P273" : s.gapHistoryArm == 274U ? "FINISH_P275" : s.gapHistoryArm == 276U ? "FINISH_P277" : s.gapHistoryArm == 266U ? "FINISH_P267" : s.gapHistoryArm == 268U ? "FINISH_P269" : s.gapHistoryArm == 270U ? "FINISH_P271" : s.gapHistoryArm == 260U ? "FINISH_P261" : s.gapHistoryArm == 262U ? "FINISH_P263" : s.gapHistoryArm == 264U ? "FINISH_P265" : s.gapHistoryArm == 254U ? "FINISH_P255" : s.gapHistoryArm == 256U ? "FINISH_P257" : s.gapHistoryArm == 258U ? "FINISH_P259" : s.gapHistoryArm == 248U ? "FINISH_P249" : s.gapHistoryArm == 250U ? "FINISH_P251" : s.gapHistoryArm == 252U ? "FINISH_P253" : s.gapHistoryArm == 242U ? "FINISH_P243" : s.gapHistoryArm == 244U ? "FINISH_P245" : s.gapHistoryArm == 246U ? "FINISH_P247" : s.gapHistoryArm == 236U ? "FINISH_P237" : s.gapHistoryArm == 238U ? "FINISH_P239" : s.gapHistoryArm == 240U ? "FINISH_P241" : s.gapHistoryArm == 230U ? "FINISH_P231" : s.gapHistoryArm == 232U ? "FINISH_P233" : s.gapHistoryArm == 234U ? "FINISH_P235" : s.gapHistoryArm == 226U ? "FINISH_P227" : s.gapHistoryArm == 228U ? "FINISH_P229" : s.gapHistoryArm == 222U ? "FINISH_P223" : s.gapHistoryArm == 224U ? "FINISH_P225" : s.gapHistoryArm == 218U ? "FINISH_P219" : s.gapHistoryArm == 220U ? "FINISH_P221" : s.gapHistoryArm == 214U ? "FINISH_P215" : s.gapHistoryArm == 216U ? "FINISH_P217" : s.gapHistoryArm == 210U ? "FINISH_P211" : s.gapHistoryArm == 212U ? "FINISH_P213" : s.gapHistoryArm == 206U ? "FINISH_P207" : s.gapHistoryArm == 208U ? "FINISH_P209" : s.gapHistoryArm == 202U ? "FINISH_P203" : s.gapHistoryArm == 204U ? "FINISH_P205" : s.gapHistoryArm == 198U ? "FINISH_P199" : s.gapHistoryArm == 200U ? "FINISH_P201" : s.gapHistoryArm == 194U ? "FINISH_P195" : s.gapHistoryArm == 196U ? "FINISH_P197" : s.gapHistoryArm == 190U ? "FINISH_P191" : s.gapHistoryArm == 192U ? "FINISH_P193" : s.gapHistoryArm == 186U ? "FINISH_P187" : s.gapHistoryArm == 188U ? "FINISH_P189" : s.gapHistoryArm == 182U ? "FINISH_P183" : s.gapHistoryArm == 184U ? "FINISH_P185" : s.gapHistoryArm == 178U ? "FINISH_P179" : s.gapHistoryArm == 180U ? "FINISH_P181" : s.gapHistoryArm == 174U ? "FINISH_P175" : "FINISH_P177") : s.gapPhysicalAd ? (s.gapHistoryArm == 170U ? "FINISH_P171" : "FINISH_P173") : s.gapHistoryArm == 166U ? "FINISH_P167" : "FINISH_P169") : "NEXT_AUTHORED_CURVE") :
                s.next == s.moves ? "FINISH_P92" : "NEXT_AUTHORED_G1");
    else if (s.gapSegmentProfile)
        RtPrintf("[EDM96] event=PHYSICAL_BOUNDARY_READY session=%llu completed=%u epoch=%llu tick=%llu next=%s witness=EDM94_CURRENT_BOUNDARY_AND_EDM96_SEGMENT_COMPLETE\n",
            static_cast<unsigned long long>(s.session), s.next, static_cast<unsigned long long>(b.identity.epoch),
            static_cast<unsigned long long>(b.lastTick), s.next == s.moves ? "P90_FINISH" : "NEXT_AUTHORED_G1");
    b = EDM94NCState::FeedBoundary{};
    // Recheck the complete physical-boundary proof on every NC scan while
    // waiting for GAP. Its own immutable deadline never resets with b.
    return !s.gapBoundaryProfile || s.next == s.moves || WaitEDM95BoundarySameThread(s.next);
}

bool NCManager::WaitEDM95BoundarySameThread(unsigned step)
{
    auto& s = m_edm94;
    const MotionExecutionEpoch epoch = m_motion.GetCurrentExecutionEpoch();
    if (!s.gapBoundaryProfile || !s.latched || !s.ready || s.armPending || s.cancelled ||
        s.finishPending || s.complete || s.resetSeen || s.resetCommitted || s.releaseRequested ||
        step >= s.moves || step != s.next || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isSingleBlockEnabled || Close_System_Com_flag ||
        !m_externalReadyInterlock || AlarmManager::GetInstance().HasAlarm() ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || !m_motion.IsGroupDone() ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        (s.gapWaiting && (s.gapWaitStep != step || s.gapWaitEpoch != epoch)))
    { RejectEDM94SameThread("EDM95_GAP_WAIT_SCOPE", m_gapDryRun.sourceLine); return false; }
    if (!s.gapWaiting)
    {
        s.gapWaiting = true; s.gapPermitReady = false;
        s.gapWaitStep = step; s.gapWaitEpoch = epoch;
        s.gapWaitFirstUs = s.gapWaitLastUs = s.gapWaitLastTick = 0ULL;
        s.gapWaitPolls = 1U;
        RtPrintf("[EDM95] event=WAIT_GAP_DIRECTION session=%llu step=%u direction=%s epoch=%llu budgetUs=5000000 source=RT_SYNTHETIC geometry=AUTHORED_G1 nextMotionSubmitted=0\n",
            static_cast<unsigned long long>(s.session), step, (step & 1U) ? "RETREAT" : "FORWARD",
            static_cast<unsigned long long>(epoch));
    }
    EDM95::Feedback seen{};
    const bool read = m_motion.ReadEDM95Feedback(seen);
    const bool identity = read && seen.session == s.session && seen.epoch == epoch &&
        seen.ownerGeneration == s.lease.generation && seen.expectedStep == step;
    if (read && seen.session == s.session && seen.faulted)
    { RejectEDM94SameThread("EDM95_GAP_SOURCE_FAULT", m_gapDryRun.sourceLine); return false; }
    const bool fresh = identity && seen.tick != 0ULL && seen.tick > s.gapWaitLastTick &&
        seen.nowUs != 0ULL && seen.nowUs > s.gapWaitLastUs;
    if (fresh)
    {
        if (s.gapWaitFirstUs == 0ULL) s.gapWaitFirstUs = seen.nowUs;
        s.gapWaitLastUs = seen.nowUs; s.gapWaitLastTick = seen.tick;
    }
    const bool timeout = s.gapWaitPolls >= 500U ||
        (s.gapWaitFirstUs != 0ULL && s.gapWaitLastUs >= s.gapWaitFirstUs &&
            s.gapWaitLastUs - s.gapWaitFirstUs >= 5000000ULL);
    if (timeout)
    { RejectEDM94SameThread("EDM95_GAP_WAIT_TIMEOUT", m_gapDryRun.sourceLine); return false; }
    const auto& permit = seen.token;
    const bool permitCurrent = fresh && seen.active && !seen.faulted && seen.gateReady &&
        seen.direction == ((step & 1U) ? -1 : 1) &&
        seen.sourceSequence != 0ULL && seen.sourceCapturedUs != 0ULL &&
        seen.nowUs >= seen.sourceCapturedUs && seen.nowUs - seen.sourceCapturedUs <= EDM95::MaximumSourceAgeUs &&
        permit.session == s.session && permit.epoch == epoch && permit.ownerGeneration == s.lease.generation &&
        permit.step == step && permit.phaseGeneration == seen.phaseGeneration &&
        permit.sourceSequence != 0ULL && permit.sourceSequence <= seen.sourceSequence &&
        permit.sourceCapturedUs != 0ULL && permit.sourceCapturedUs <= seen.sourceCapturedUs &&
        permit.issuedUs >= permit.sourceCapturedUs && permit.issuedUs <= seen.nowUs &&
        seen.nowUs - permit.issuedUs <= EDM95::PermitLifetimeUs / 2ULL &&
        permit.tick != 0ULL && permit.tick <= seen.tick;
    if (!permitCurrent) return false;
    s.gapFeedback = seen; s.gapPermitReady = true; s.gapWaiting = false;
    RtPrintf("[EDM95] event=GAP_GRANT session=%llu step=%u direction=%s phase=%llu sourceSequence=%llu sourceUs=%llu issuedUs=%llu tick=%llu epoch=%llu polls=%u oneUse=1 source=RT_SYNTHETIC midSegmentReverse=0\n",
        static_cast<unsigned long long>(s.session), step, (step & 1U) ? "RETREAT" : "FORWARD",
        static_cast<unsigned long long>(permit.phaseGeneration), static_cast<unsigned long long>(permit.sourceSequence),
        static_cast<unsigned long long>(permit.sourceCapturedUs), static_cast<unsigned long long>(permit.issuedUs),
        static_cast<unsigned long long>(permit.tick), static_cast<unsigned long long>(permit.epoch), s.gapWaitPolls);
    return true;
}

bool NCManager::IsEDM97TuningCurrentSameThread(const EDM96::Feedback& gap) const noexcept
{
    const auto& s = m_edm94;
    if (s.gapHistoryServo)
    {
        const bool combined = s.gapHistoryMode;
        const unsigned fixture = E149HistoryFixture(s.gapHistoryArm);
        const bool profile = combined ?
            (s.gapWorkTime ? (E130TimedArm(s.gapHistoryArm) && s.gapPhysicalAd == E149PhysicalExpected(s.gapHistoryArm)) :
            s.gapPhysicalAd ? (s.gapHistoryArm == 170U || s.gapHistoryArm == 172U) :
            (s.gapHistoryArm == 166U || s.gapHistoryArm == 168U)) &&
                s.gapModeFixture == E149ModalFixture(fixture) &&
                EDM107::ValidFlushProfile(s.gapFlushParameters) :
            !s.gapPhysicalAd && (s.gapHistoryArm == 158U || s.gapHistoryArm == 160U) &&
                s.gapModeFixture == (fixture ? EDM116::Fixture::ArcPairComplement124 : EDM116::Fixture::ArcPair124) &&
                EDM107::IsDefaultFlushProfile(s.gapFlushParameters);
        if (!profile ||
            !s.gapPhysicalCurve || !s.gapSegmentProfile || !s.gapGainProfile || !E156BoundPlanCurrent(s, m_edm130.recipe) ||
            s.gapBoundaryProfile || s.gapNoiseProfile || s.gapBurstProfile || s.gapContinuousProfile ||
            s.gapEnduranceProfile || s.gapPairedProfile || s.gapFlushProfile || s.gapFlushParameterized ||
            s.gapCommonPath || s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime ||
            s.gapCurveB2 || s.gapModeFlush || s.gapModeInterruption || s.gapModeInterruptionArm || s.gapFlushInterruptArm ||
            !gap.tunedMode || gap.noiseMode || gap.burstMode || gap.continuousMode || gap.enduranceMode ||
            gap.pairedMode || gap.flushMode || gap.flushPhase != EDM105::Phase::None ||
            gap.flushStarted || gap.flushCompleted || gap.flushZeroTicks || gap.flushAnchor != 0.0 ||
            gap.flushRetreat != 0.0 || gap.flushReturned != 0.0 ||
            !EDM97::ValidTuning(s.gapTuning) || s.gapTuning.positiveGain != 8.0 ||
            s.gapTuning.negativeGain != 1.0 || s.gapTuning.deadbandV != 1.0 ||
            !EDM97::SameTuning(s.gapTuning, gap.tuning) || gap.configHash != EDM97::ConfigHash(s.gapTuning) ||
            !std::isfinite(gap.errorV) || !std::isfinite(gap.rawSpeedMmMin) ||
            !std::isfinite(gap.limitedSpeedMmMin) || !std::isfinite(gap.targetRatio) ||
            gap.limitedSpeedMmMin < -EDM97::MaximumRetreatMmMin || gap.limitedSpeedMmMin > EDM97::MaximumFeedMmMin ||
            gap.targetRatio < -EDM97::MaximumRetreatMmMin / EDM97::MaximumFeedMmMin || gap.targetRatio > 1.0 ||
            gap.acceptedSegments > s.moves || gap.completedSegments > gap.acceptedSegments ||
            gap.acceptedSegments - gap.completedSegments > 1ULL || gap.expectedStep != gap.completedSegments)
            return false;
        if (E134MachiningArm(s.gapHistoryArm))
        {
            if (!m_edm130.bound || !EDM130::ValidRecipe(m_edm130.recipe) ||
                !EDM130::HasBoundMachiningSpeed(m_edm130.recipe) ||
                EDM130::HasBoundReferenceVoltage(m_edm130.recipe) != E135ReferenceArm(s.gapHistoryArm) ||
                EDM130::HasBoundCurve(m_edm130.recipe) != E136CurveArm(s.gapHistoryArm) ||
                EDM130::HasBoundFlushStages(m_edm130.recipe) != E137FlushStageArm(s.gapHistoryArm) ||
                m_edm130.recipe.diagnosticControlSource != E149ControlSource(s.gapHistoryArm) ||
                (E139DiagnosticArm(s.gapHistoryArm) &&
                    gap.voltageV != EDM139::HighVoltage(EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe)) &&
                    gap.voltageV != EDM139::LowVoltage(EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe)))) return false;
            const auto curve = EDMGapServo::Evaluate(EDM97::MakeProfile(s.gapTuning, m_edm130.recipe), gap.voltageV,
                EDM130::FrozenGapReferenceVoltageV(m_edm130.recipe), EDM130::FrozenMachiningOverridePercent(m_edm130.recipe));
            const double ratio = curve.valid && ((gap.direction > 0 && curve.limitedSpeedMmPerMin >= 0.0) ||
                (gap.direction < 0 && curve.limitedSpeedMmPerMin <= 0.0)) ?
                curve.limitedSpeedMmPerMin / EDM97::MaximumFeedMmMin : 0.0;
            if (!curve.valid || gap.errorV != curve.errorV || gap.rawSpeedMmMin != curve.rawSpeedMmPerMin ||
                gap.limitedSpeedMmMin != curve.limitedSpeedMmPerMin || gap.targetRatio != ratio) return false;
        }
        EDM126::MotionFeedback f{};
        return m_motion.ReadEDM126Feedback(f) && f.enabled && f.physicalAd == s.gapPhysicalAd && !f.faulted && f.reason == EDM126::Reason::None &&
            // These observer channels may straddle an RT completion/handoff.
            // Exact live epoch and N/N counts are checked by the admission and boundary callers.
            f.session == s.session && f.armEpoch == s.armEpoch && f.liveEpoch != 0ULL &&
            f.ownerGeneration == s.lease.generation && f.session == gap.session && f.ownerGeneration == gap.ownerGeneration &&
            f.fixture == fixture &&
            f.authorizedRows <= s.moves && f.completedRows <= f.authorizedRows &&
            f.authorizedRows - f.completedRows <= 1U;
    }
    if (s.gapHistoryArm || s.gapPhysicalAd) return false;
    const bool interruptionProfile = s.gapModeInterruptionArm == EDM118::ArmP ||
        s.gapModeInterruptionArm == EDM119::ArmP || s.gapModeInterruptionArm == EDM120::ArmP ||
        s.gapModeInterruptionArm == EDM121::ArmP ||
        EDM122::ValidProfile(EDM122::ProfileForArm(s.gapModeInterruptionArm)) ||
        EDM123::ValidProfile(EDM123::ProfileForArm(s.gapModeInterruptionArm)) ||
        EDM125::ValidProfile(EDM125::ProfileForArm(s.gapModeInterruptionArm));
    if (s.gapModeInterruption != interruptionProfile ||
        (s.gapModeInterruption && (!s.gapModeFlush ||
            s.gapModeFixture != (EDM125::ValidProfile(EDM125::ProfileForArm(s.gapModeInterruptionArm)) ? EDM125::TargetFixture(EDM125::ProfileForArm(s.gapModeInterruptionArm)) :
                EDM123::ValidProfile(EDM123::ProfileForArm(s.gapModeInterruptionArm)) ? EDM123::TargetFixture :
                EDM122::ValidProfile(EDM122::ProfileForArm(s.gapModeInterruptionArm)) ? EDM122::TargetFixture :
                s.gapModeInterruptionArm == EDM121::ArmP ? EDM121::TargetFixture :
                s.gapModeInterruptionArm == EDM120::ArmP ? EDM120::TargetFixture :
                s.gapModeInterruptionArm == EDM119::ArmP ? EDM119::TargetFixture : EDM118::TargetFixture)))) return false;
    if ((s.gapModeFlush && (!s.gapPhysicalCurve || s.gapCurveB2)) || (s.gapCurveB2 && !s.gapPhysicalCurve) || (s.gapPhysicalCurve && (!s.gapFlushProfile || !s.gapFlushParameterized || s.gapFlushInterruptArm != 0U ||
            s.gapCommonPath || s.gapB2CrossSegment || s.gapB2Repeat || s.gapCurveGeometry || s.gapCurveRuntime)) ||
        (s.gapCurveRuntime && !s.gapCurveGeometry) || (s.gapCurveGeometry && !s.gapB2Repeat) || (s.gapB2Repeat && !s.gapB2CrossSegment) || (s.gapB2CrossSegment && !s.gapCommonPath) ||
        (s.gapCommonPath && (!s.gapFlushProfile || !s.gapFlushParameterized || s.gapFlushInterruptArm != 0U)) ||
        !EDM107::ValidFlushProfile(s.gapFlushParameters) || !EDM107::ValidFlushProfile(gap.flushProfile) ||
        !EDM107::SameFlushProfile(s.gapFlushParameters, gap.flushProfile) ||
        (s.gapFlushInterruptArm != 0U && !s.gapFlushProfile) ||
        (!s.gapFlushParameterized && (!EDM107::IsDefaultFlushProfile(s.gapFlushParameters) ||
            (s.gapFlushInterruptArm != 0U && s.gapFlushInterruptArm != 105U && s.gapFlushInterruptArm != 107U))) ||
        (s.gapFlushParameterized && (!s.gapFlushProfile ||
            (s.gapFlushInterruptArm != 0U && s.gapFlushInterruptArm != 111U && s.gapFlushInterruptArm != 113U)))) return false;
    if (gap.flushMode != s.gapFlushProfile ||
        (s.gapFlushProfile && (!s.gapPairedProfile || !s.gapEnduranceProfile || !s.gapContinuousProfile ||
            !s.gapBurstProfile || !s.gapGainProfile || !s.gapSegmentProfile || s.gapNoiseProfile ||
            s.gapBoundaryProfile || s.moves != EDM105::MaximumSteps ||
            static_cast<unsigned>(gap.flushPhase) > static_cast<unsigned>(EDM105::Phase::Complete) ||
            gap.flushStarted > EDM105::ExpectedFlushCycles || gap.flushCompleted > gap.flushStarted ||
            gap.flushStarted - gap.flushCompleted > 1ULL ||
            gap.flushStarted > (gap.acceptedSegments + 1ULL) / 2ULL ||
            gap.flushCompleted < (gap.completedSegments + 1ULL) / 2ULL || gap.flushZeroTicks > gap.tick ||
            !std::isfinite(gap.flushAnchor) || !std::isfinite(gap.flushRetreat) || !std::isfinite(gap.flushReturned) ||
            gap.flushAnchor < 0.0 || gap.flushRetreat < 0.0 || gap.flushReturned < 0.0)) ||
        (!s.gapFlushProfile && (gap.flushPhase != EDM105::Phase::None || gap.flushStarted != 0ULL ||
            gap.flushCompleted != 0ULL || gap.flushZeroTicks != 0ULL || gap.flushAnchor != 0.0 ||
            gap.flushRetreat != 0.0 || gap.flushReturned != 0.0))) return false;
    if (gap.pairedMode != s.gapPairedProfile ||
        (s.gapPairedProfile && (!s.gapEnduranceProfile || !s.gapContinuousProfile || !s.gapBurstProfile ||
            !s.gapGainProfile || !s.gapSegmentProfile || s.gapNoiseProfile || s.gapBoundaryProfile ||
            s.moves != EDM104::MaximumSteps))) return false;
    if (gap.enduranceMode != s.gapEnduranceProfile ||
        (s.gapEnduranceProfile && (!s.gapContinuousProfile || !s.gapBurstProfile || !s.gapGainProfile ||
            !s.gapSegmentProfile || s.gapNoiseProfile || s.gapBoundaryProfile || s.moves != EDM102::MaximumSteps))) return false;
    if (gap.continuousMode != s.gapContinuousProfile ||
        (s.gapContinuousProfile && (!s.gapBurstProfile || !s.gapGainProfile ||
            !s.gapSegmentProfile || s.gapNoiseProfile || s.gapBoundaryProfile))) return false;
    if (gap.burstMode != s.gapBurstProfile ||
        (s.gapBurstProfile && (!s.gapGainProfile || !s.gapSegmentProfile || s.gapNoiseProfile || s.gapBoundaryProfile ||
            gap.acceptedSegments > (s.gapEnduranceProfile ? EDM102::MaximumSteps : s.gapContinuousProfile ? EDM100::MaximumSteps : 4ULL) ||
            gap.completedSegments > gap.acceptedSegments || gap.phase > 7U ||
            gap.freshFrames > gap.tick || gap.pulseLowFreshFrames > gap.freshFrames ||
            gap.suppressedPulseWindows > gap.pulseLowFreshFrames ||
            gap.suppressedPulseWindows > 4ULL * gap.acceptedSegments ||
            gap.qualifiedRetreatWindows > gap.freshFrames ||
            gap.qualifiedRetreatWindows > 2ULL * gap.acceptedSegments ||
            gap.reverseEpisodes > gap.qualifiedRetreatWindows ||
            gap.reverseEpisodes > 2ULL * gap.acceptedSegments ||
            gap.reverseSegments > gap.acceptedSegments || gap.zeroCrossings > gap.tick ||
            gap.zeroCrossings > 4ULL * gap.acceptedSegments ||
            gap.reverseTicks > gap.tick || gap.returnForwardTicks > gap.tick)) ||
        (!s.gapBurstProfile && (gap.pulseLowFreshFrames != 0ULL || gap.suppressedPulseWindows != 0ULL ||
            gap.qualifiedRetreatWindows != 0ULL || gap.reverseEpisodes != 0ULL))) return false;
    if (gap.noiseMode != s.gapNoiseProfile ||
        (s.gapNoiseProfile && (!s.gapGainProfile || !s.gapSegmentProfile ||
            !EDM98::ValidTuning(s.gapTuning) || !EDM98::ValidTuning(gap.tuning) ||
            gap.acceptedSegments > 4ULL || gap.completedSegments > gap.acceptedSegments ||
            gap.noiseFreshFrames > gap.freshFrames || gap.quietZeroTicks > gap.tick ||
            gap.quietWindows > 2ULL * gap.acceptedSegments || gap.quietSignChanges != 0ULL)) ||
        (!s.gapNoiseProfile && (gap.noiseFreshFrames != 0ULL || gap.quietZeroTicks != 0ULL ||
            gap.quietWindows != 0ULL || gap.quietSignChanges != 0ULL))) return false;
    if (!s.gapGainProfile)
        return !gap.tunedMode && EDM97::EmptyTuning(gap.tuning) && gap.configHash == 0ULL;
    return gap.tunedMode && EDM97::ValidTuning(s.gapTuning) && EDM97::ValidTuning(gap.tuning) &&
        EDM97::SameTuning(gap.tuning, s.gapTuning) && gap.configHash == EDM97::ConfigHash(s.gapTuning) &&
        std::isfinite(gap.errorV) && std::isfinite(gap.rawSpeedMmMin) &&
        std::isfinite(gap.limitedSpeedMmMin) && std::isfinite(gap.targetRatio) &&
        gap.limitedSpeedMmMin >= -EDM97::MaximumRetreatMmMin &&
        gap.limitedSpeedMmMin <= EDM97::MaximumFeedMmMin &&
        gap.targetRatio >= -EDM97::MaximumRetreatMmMin / EDM97::MaximumFeedMmMin && gap.targetRatio <= 1.0 &&
        gap.saturationFrames <= gap.freshFrames;
}

void NCManager::LogEDM97SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    char record[1024]{};
    int length = std::snprintf(record, sizeof(record),
        "[EDM97] event=%.32s session=%llu epoch=%llu ownerGen=%u step=%u phase=%u direction=%s voltage=%.9g qPulse=%.12g vPulsePerSec=%.12g lengthPulse=%.12g accepted=%llu completed=%llu reverseSegments=%llu zeroCrossings=%llu reverseTicks=%llu returnForwardTicks=%llu tick=%llu sourceSequence=%llu sourceUs=%llu freshFrames=%llu heldFrames=%llu maxAgeUs=%llu active=%u segmentActive=%u segmentCompleted=%u faulted=%u reason=%s midSegmentReverse=1 output=GUARDED_CSV dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(gap.session),
        static_cast<unsigned long long>(gap.epoch), static_cast<unsigned>(gap.ownerGeneration), gap.expectedStep,
        gap.phase, gap.direction > 0 ? "FORWARD" : gap.direction < 0 ? "RETREAT" : "QUALIFYING",
        gap.voltageV, gap.q, gap.v, gap.length, static_cast<unsigned long long>(gap.acceptedSegments),
        static_cast<unsigned long long>(gap.completedSegments), static_cast<unsigned long long>(gap.reverseSegments),
        static_cast<unsigned long long>(gap.zeroCrossings), static_cast<unsigned long long>(gap.reverseTicks),
        static_cast<unsigned long long>(gap.returnForwardTicks), static_cast<unsigned long long>(gap.tick),
        static_cast<unsigned long long>(gap.sourceSequence), static_cast<unsigned long long>(gap.sourceCapturedUs),
        static_cast<unsigned long long>(gap.freshFrames), static_cast<unsigned long long>(gap.heldFrames),
        static_cast<unsigned long long>(gap.maximumAgeUs), gap.active ? 1U : 0U, gap.segmentActive ? 1U : 0U,
        gap.segmentCompleted ? 1U : 0U, gap.faulted ? 1U : 0U, EDM96::ReasonName(gap.reason));
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM97] event=LOG_FORMAT_ERROR scope=SEGMENT output=NONE\n");
    length = std::snprintf(record, sizeof(record),
        "[EDM97] event=GAIN_EVIDENCE result=%.32s session=%llu tick=%llu phase=%u voltage=%.9g referenceV=35 errorV=%.9g rawSpeedMmMin=%.12g limitedSpeedMmMin=%.12g targetRatio=%.12g Q=%.9g R=%.9g S=%.9g configHash=%llu saturationFreshFrames=%llu freshFrames=%llu tunedMode=%u tuningCurrent=%u scalarTrajectory=1 gainUnit=DIRECT_CURVE dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(gap.session),
        static_cast<unsigned long long>(gap.tick), gap.phase, gap.voltageV, gap.errorV,
        gap.rawSpeedMmMin, gap.limitedSpeedMmMin, gap.targetRatio, gap.tuning.positiveGain,
        gap.tuning.negativeGain, gap.tuning.deadbandV, static_cast<unsigned long long>(gap.configHash),
        static_cast<unsigned long long>(gap.saturationFrames), static_cast<unsigned long long>(gap.freshFrames),
        gap.tunedMode ? 1U : 0U, IsEDM97TuningCurrentSameThread(gap) ? 1U : 0U);
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM97] event=LOG_FORMAT_ERROR scope=GAIN_EVIDENCE output=NONE\n");
}

void NCManager::LogEDM98SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    char record[1024]{};
    int length = std::snprintf(record, sizeof(record),
        "[EDM98] event=%.32s session=%llu epoch=%llu ownerGen=%u step=%u phase=%u direction=%s voltage=%.9g qPulse=%.12g vPulsePerSec=%.12g lengthPulse=%.12g accepted=%llu completed=%llu reverseSegments=%llu zeroCrossings=%llu reverseTicks=%llu returnForwardTicks=%llu tick=%llu sourceSequence=%llu sourceUs=%llu freshFrames=%llu heldFrames=%llu maxAgeUs=%llu active=%u segmentActive=%u segmentCompleted=%u faulted=%u reason=%s midSegmentReverse=1 output=GUARDED_CSV dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(gap.session),
        static_cast<unsigned long long>(gap.epoch), static_cast<unsigned>(gap.ownerGeneration), gap.expectedStep,
        gap.phase, gap.direction > 0 ? "FORWARD" : gap.direction < 0 ? "RETREAT" : "QUALIFYING_OR_QUIET",
        gap.voltageV, gap.q, gap.v, gap.length, static_cast<unsigned long long>(gap.acceptedSegments),
        static_cast<unsigned long long>(gap.completedSegments), static_cast<unsigned long long>(gap.reverseSegments),
        static_cast<unsigned long long>(gap.zeroCrossings), static_cast<unsigned long long>(gap.reverseTicks),
        static_cast<unsigned long long>(gap.returnForwardTicks), static_cast<unsigned long long>(gap.tick),
        static_cast<unsigned long long>(gap.sourceSequence), static_cast<unsigned long long>(gap.sourceCapturedUs),
        static_cast<unsigned long long>(gap.freshFrames), static_cast<unsigned long long>(gap.heldFrames),
        static_cast<unsigned long long>(gap.maximumAgeUs), gap.active ? 1U : 0U, gap.segmentActive ? 1U : 0U,
        gap.segmentCompleted ? 1U : 0U, gap.faulted ? 1U : 0U, EDM96::ReasonName(gap.reason));
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM98] event=LOG_FORMAT_ERROR scope=SEGMENT output=NONE\n");
    length = std::snprintf(record, sizeof(record),
        "[EDM98] event=GAIN_EVIDENCE result=%.32s session=%llu tick=%llu phase=%u voltage=%.9g referenceV=35 errorV=%.9g rawSpeedMmMin=%.12g limitedSpeedMmMin=%.12g targetRatio=%.12g Q=%.9g R=%.9g S=%.9g configHash=%llu saturationFreshFrames=%llu freshFrames=%llu tunedMode=%u noiseMode=%u tuningCurrent=%u scalarTrajectory=1 gainUnit=DIRECT_CURVE dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(gap.session),
        static_cast<unsigned long long>(gap.tick), gap.phase, gap.voltageV, gap.errorV,
        gap.rawSpeedMmMin, gap.limitedSpeedMmMin, gap.targetRatio, gap.tuning.positiveGain,
        gap.tuning.negativeGain, gap.tuning.deadbandV, static_cast<unsigned long long>(gap.configHash),
        static_cast<unsigned long long>(gap.saturationFrames), static_cast<unsigned long long>(gap.freshFrames),
        gap.tunedMode ? 1U : 0U, gap.noiseMode ? 1U : 0U, IsEDM97TuningCurrentSameThread(gap) ? 1U : 0U);
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM98] event=LOG_FORMAT_ERROR scope=GAIN_EVIDENCE output=NONE\n");
    length = std::snprintf(record, sizeof(record),
        "[EDM98] event=DEADBAND_EVIDENCE result=%.32s session=%llu tick=%llu phase=%u voltage=%.9g S=%.9g noiseFreshFrames=%llu quietZeroTicks=%llu quietWindows=%llu quietSignChanges=%llu quietRequiredTicks=800 quietRequiredUs=200000 quietScope=COMMAND_SCALAR_ONLY qPulse=%.12g vPulsePerSec=%.12g encoderZeroProven=0 noiseMode=%u tuningCurrent=%u dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(gap.session),
        static_cast<unsigned long long>(gap.tick), gap.phase, gap.voltageV, gap.tuning.deadbandV,
        static_cast<unsigned long long>(gap.noiseFreshFrames), static_cast<unsigned long long>(gap.quietZeroTicks),
        static_cast<unsigned long long>(gap.quietWindows), static_cast<unsigned long long>(gap.quietSignChanges),
        gap.q, gap.v, gap.noiseMode ? 1U : 0U, IsEDM97TuningCurrentSameThread(gap) ? 1U : 0U);
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM98] event=LOG_FORMAT_ERROR scope=DEADBAND_EVIDENCE output=NONE\n");
}

void NCManager::LogEDM99SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    LogEDM99SegmentRecords("EDM99", event, gap, IsEDM97TuningCurrentSameThread(gap));
}

void NCManager::LogEDM100SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    LogEDM99SegmentRecords("EDM100", event, gap, IsEDM97TuningCurrentSameThread(gap));
}

void NCManager::LogEDM102SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    LogEDM99SegmentRecords("EDM102", event, gap, IsEDM97TuningCurrentSameThread(gap));
}

void NCManager::LogEDM104SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    LogEDM99SegmentRecords("EDM104", event, gap, IsEDM97TuningCurrentSameThread(gap));
}

void NCManager::LogEDM106InterruptionSameThread(const char* event, const char* reason) const noexcept
{
    const auto& s = m_edm94;
    if (!s.gapFlushProfile || !s.session) return;
    EDM96::Feedback gap{};
    const bool read = m_motion.ReadEDM96Feedback(gap);
    const bool identity = read && gap.session == s.session &&
        gap.ownerGeneration == s.lease.generation && gap.epoch != 0ULL;
    const bool tuning = identity && IsEDM97TuningCurrentSameThread(gap);
    const auto& r = gap.interruption;
    const bool fixture108 = s.gapFlushParameterized && (s.gapFlushInterruptArm == 111U || s.gapFlushInterruptArm == 113U);
    const char* const stage = fixture108 ? "EDM108" : "EDM106";
    const EDM105::Phase target = (s.gapFlushInterruptArm == 105U || s.gapFlushInterruptArm == 111U) ? EDM105::Phase::Retreat :
        (s.gapFlushInterruptArm == 107U || s.gapFlushInterruptArm == 113U) ? EDM105::Phase::Return : EDM105::Phase::None;
    const bool frozen = identity && !gap.active && !gap.segmentActive &&
        !gap.segmentCompleted && !gap.gateReady && gap.v == 0.0;
    const bool valid = tuning && frozen && r.captured && r.sourceTick != 0ULL &&
        r.plannerTick != 0ULL && r.plannerTick <= r.sourceTick && r.sourceTick <= gap.tick &&
        r.nowUs != 0ULL && r.nowUs <= gap.nowUs &&
        static_cast<unsigned>(r.phase) <= static_cast<unsigned>(EDM105::Phase::Complete) &&
        std::isfinite(r.q) && std::isfinite(r.v) && r.q >= 0.0 && r.q <= gap.length &&
        gap.acceptedSegments > gap.completedSegments;
    // NC request phase and RT-latched phase are distinct observations. Only a
    // matching moving planner state in that exact requested segment is coverage.
    const bool requestedSegment = valid && s.gapFlushInterruptRequested &&
        s.gapFlushInterruptSampleTick != 0ULL && gap.epoch == s.gapFlushInterruptEpoch &&
        r.sourceTick >= s.gapFlushInterruptSampleTick &&
        gap.acceptedSegments == 1ULL && gap.completedSegments == 0ULL && gap.expectedStep == 1U;
    const bool targetMatch = requestedSegment && target != EDM105::Phase::None &&
        r.phase == target && (target == EDM105::Phase::Retreat ? r.v < 0.0 : r.v > 0.0);
    const auto& guard = s.feedback;
    const bool cachedStop = valid && guard.identity.session == s.session &&
        guard.identity.epoch == s.armEpoch && guard.identity.ownerGeneration == s.lease.generation &&
        guard.lastTick >= r.sourceTick && guard.stopped && guard.forceZero;
    const char* const coverage = !s.gapFlushInterruptArm ? "NOT_REQUESTED" :
        !s.gapFlushInterruptRequested ? "NOT_TRIGGERED" : !valid ? "AWAIT_VALID_RT_RECORD" :
        targetMatch ? "TARGET_PHASE_OBSERVED" : "TARGET_NOT_PROVEN";
    char record[481]{};
    const char* const result = event ? event : "UNKNOWN";
    const auto session = static_cast<unsigned long long>(s.session);
    const auto emit = [&](int length, const char* part) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
            RtPrintf("%s", record);
        else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=%.16s session=%llu output=NONE\n", stage, part, session);
    };
    emit(std::snprintf(record, sizeof(record),
        "[%s] event=FLUSH_INTERRUPTION stage=%.32s part=STATE session=%llu armP=%u target=%s requested=%u read=%u identity=%u tuning=%u captured=%u valid=%u phase=%s plannerFault=%u coverage=%s\n",
        stage, result, session, s.gapFlushInterruptArm, EDM105::PhaseName(target),
        s.gapFlushInterruptRequested ? 1U : 0U, read ? 1U : 0U, identity ? 1U : 0U,
        tuning ? 1U : 0U, identity && r.captured ? 1U : 0U, valid ? 1U : 0U,
        identity && r.captured ? EDM105::PhaseName(r.phase) : "UNAVAILABLE",
        identity && r.captured && r.fault ? 1U : 0U, coverage), "STATE");
    if (fixture108)
    {
        // Guidance observes existing event sites only. It grants no motion or recovery authority.
        EDM94::Feedback currentGuard{};
        const bool guardRead = m_motion.ReadEDM94Feedback(currentGuard);
        const bool physicalStopped = cachedStop && guardRead && !r.fault && !gap.faulted && !s.rtFaultReported &&
            gap.reason == EDM96::Reason::None && currentGuard.identity.session == s.session &&
            currentGuard.identity.epoch == s.armEpoch && currentGuard.identity.ownerGeneration == s.lease.generation &&
            !currentGuard.faulted && currentGuard.cancelled && currentGuard.armAccepted &&
            currentGuard.stopped && currentGuard.forceZero && currentGuard.lastTick >= guard.lastTick &&
            currentGuard.lastTick >= s.lastTick && currentGuard.lastTick >= r.sourceTick &&
            currentGuard.requiredCycles >= EDM94::RequiredStableCycles &&
            currentGuard.settledCycles >= currentGuard.requiredCycles && s.cancelled && s.latched;
        const bool targetStopped = targetMatch && physicalStopped;
        const bool resetPending = s.resetSeen || s.resetCommitted || s.releaseRequested;
        const bool resetReleased = s.resetSeen && s.resetCommitted && s.releaseRequested &&
            currentGuard.released && !currentGuard.active && physicalStopped;
        const char* outcome = nullptr; const char* next = nullptr;
        bool observedStartBlocked = false;
        if (std::strcmp(result, "ARM_REQUEST") == 0)
        { outcome = "ARM_REQUESTED"; next = "WAIT_FOR_AUTO_HOLD"; }
        else if (std::strcmp(result, "CANCEL_STOPPED") == 0)
        {
            outcome = targetStopped ? "TARGET_STOPPED" : "TARGET_NOT_PROVEN";
            next = resetPending ? "WAIT_RESET_RELEASE" : targetStopped ?
                "PRESS_START_ONCE_TO_CHECK_RESET_GATE" : "PRESS_RESET_AND_SEND_LOG";
        }
        else if (std::strcmp(result, "START_BLOCKED") == 0)
        {
            observedStartBlocked = true; // Only the real CycleStart rejection calls this event.
            if (reason && std::strcmp(reason, "RESET_RELEASE_PENDING") == 0)
            { outcome = "RESET_RELEASE_PENDING"; next = "WAIT_RESET_RELEASE"; }
            else if (reason && std::strcmp(reason, "RESET_REQUIRED") == 0 && !resetPending && m_state == NCState::HOLD)
            {
                outcome = "START_BLOCKED_OBSERVED";
                next = targetStopped ? "PRESS_RESET_THEN_LOAD_NEXT_TEST" : "PRESS_RESET_AND_SEND_LOG";
            }
            else { outcome = "START_BLOCKED_STATE_UNPROVEN"; next = "SEND_LOG_CHECK_RESET_STATE"; }
        }
        else if (std::strcmp(result, "RELEASED") == 0)
        {
            outcome = resetReleased ? "RESET_RELEASED" : "RESET_RELEASE_NOT_PROVEN";
            next = !resetReleased ? "SEND_LOG_CHECK_RESET_STATE" : targetStopped ?
                "LOAD_NEXT_TEST_THEN_PRESS_START" : "SEND_LOG_TARGET_NOT_PROVEN";
        }
        if (outcome)
            emit(std::snprintf(record, sizeof(record),
                "[EDM108] event=OPERATOR_GUIDE stage=%.32s result=%s session=%llu armP=%u target=%s targetMatched=%u physicalStopped=%u targetStopped=%u stable=%u required=%u observedStartBlocked=%u next=%s autoStart=0 autoReset=0 dischargeCommand=NONE\n",
                result, outcome, session, s.gapFlushInterruptArm, EDM105::PhaseName(target),
                targetMatch ? 1U : 0U, physicalStopped ? 1U : 0U, targetStopped ? 1U : 0U, currentGuard.settledCycles, currentGuard.requiredCycles,
                observedStartBlocked ? 1U : 0U, next), "OPERATOR_GUIDE");
    }
    if (!identity) return;
    emit(std::snprintf(record, sizeof(record),
        "[%s] event=FLUSH_INTERRUPTION stage=%.32s part=FRAME session=%llu epoch=%llu ownerGen=%u requestEpoch=%llu requestTick=%llu sourceTick=%llu plannerTick=%llu sourceNowUs=%llu scalarFrozen=%u cachedGuardTick=%llu cachedPhysicalStop=%u\n",
        stage, result, session, static_cast<unsigned long long>(gap.epoch), static_cast<unsigned>(gap.ownerGeneration),
        static_cast<unsigned long long>(s.gapFlushInterruptEpoch),
        static_cast<unsigned long long>(s.gapFlushInterruptSampleTick),
        static_cast<unsigned long long>(r.sourceTick), static_cast<unsigned long long>(r.plannerTick),
        static_cast<unsigned long long>(r.nowUs), frozen ? 1U : 0U,
        static_cast<unsigned long long>(guard.lastTick), cachedStop ? 1U : 0U), "FRAME");
    emit(std::snprintf(record, sizeof(record),
        "[%s] event=FLUSH_INTERRUPTION stage=%.32s part=COUNTS session=%llu step=%u accepted=%llu completed=%llu flushStarted=%llu flushCompleted=%llu currentPhase=%s proofScope=RETAINED_PLANNER_ONLY appliedCommandProven=0 encoderZeroProven=0\n",
        stage, result, session, gap.expectedStep, static_cast<unsigned long long>(gap.acceptedSegments),
        static_cast<unsigned long long>(gap.completedSegments), static_cast<unsigned long long>(gap.flushStarted),
        static_cast<unsigned long long>(gap.flushCompleted), EDM105::PhaseName(gap.flushPhase)), "COUNTS");
    if (!r.captured) return;
    emit(std::snprintf(record, sizeof(record),
        "[%s] event=FLUSH_INTERRUPTION stage=%.32s part=GEOMETRY session=%llu plannerQPulse=%.12g plannerVPulsePerSec=%.12g anchorPulse=%.12g retreatPulse=%.12g returnedPulse=%.12g firstRecordRetained=1 autoReturn=0 resumeFlush=0 restart=RESET_THEN_START_TOP dischargeCommand=NONE\n",
        stage, result, session, r.q, r.v, gap.flushAnchor, gap.flushRetreat, gap.flushReturned), "GEOMETRY");
}

void NCManager::LogEDM105SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    LogEDM99SegmentRecords("EDM105", event, gap, IsEDM97TuningCurrentSameThread(gap));
    char record[481]{};
    const auto session = static_cast<unsigned long long>(gap.session);
    const auto tick = static_cast<unsigned long long>(gap.tick);
    const char* result = event ? event : "UNKNOWN";
    const auto emit = [&](int length, const char* part) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
            RtPrintf("%s", record);
        else RtPrintf("[EDM105] event=LOG_FORMAT_ERROR part=%.16s session=%llu tick=%llu output=NONE\n",
            part, session, tick);
    };
    emit(std::snprintf(record, sizeof(record),
        "[EDM105] event=FLUSH_EVIDENCE result=%.32s part=STATE session=%llu tick=%llu step=%u flushMode=%u flushPhase=%s flushStarted=%llu flushCompleted=%llu flushZeroTicks=%llu expectedCycles=8 proofScope=COMMAND_SCALAR_ONLY dischargeCommand=NONE\n",
        result, session, tick, gap.expectedStep, gap.flushMode ? 1U : 0U, EDM105::PhaseName(gap.flushPhase),
        static_cast<unsigned long long>(gap.flushStarted), static_cast<unsigned long long>(gap.flushCompleted),
        static_cast<unsigned long long>(gap.flushZeroTicks)), "FLUSH_STATE");
    emit(std::snprintf(record, sizeof(record),
        "[EDM105] event=FLUSH_EVIDENCE result=%.32s part=GEOMETRY session=%llu tick=%llu anchorPulse=%.12g retreatPulse=%.12g returnedPulse=%.12g qPulse=%.12g vPulsePerSec=%.12g encoderZeroProven=0 proofScope=COMMAND_SCALAR_ONLY dischargeCommand=NONE\n",
        result, session, tick, gap.flushAnchor, gap.flushRetreat, gap.flushReturned, gap.q, gap.v), "FLUSH_GEOMETRY");
    emit(std::snprintf(record, sizeof(record),
        "[EDM105] event=FLUSH_EVIDENCE result=%.32s part=POLICY session=%llu tick=%llu outwardOnly=1 triggerProgress=0.5 strokeRatio=%.12g speedRatio=%.12g speedCapMmMin=%.12g commandDwellUs=%llu commandDwellTicks=%llu dwellSites=2 source=RT_SYNTHETIC hold=CANCEL_RESET_REQUIRED proofScope=COMMAND_SCALAR_ONLY dischargeCommand=NONE\n",
        result, session, tick, gap.flushProfile.strokeRatio, gap.flushProfile.speedRatio,
        EDM97::MaximumFeedMmMin * gap.flushProfile.speedRatio,
        static_cast<unsigned long long>(gap.flushProfile.dwellUs),
        static_cast<unsigned long long>(EDM107::FlushDwellTicks(gap.flushProfile))), "FLUSH_POLICY");
    if (m_edm94.gapFlushParameterized)
        emit(std::snprintf(record, sizeof(record),
            "[EDM107] event=PARAMETER_EVIDENCE result=%.32s session=%llu tick=%llu I=%.12g J=%.12g K=%llu frozenI=%.12g frozenJ=%.12g frozenK=%llu parametersCurrent=%u freeze=ADMISSION apply=NEXT_ARM proofScope=COMMAND_SCALAR_ONLY dischargeCommand=NONE\n",
            result, session, tick, gap.flushProfile.strokeRatio * 100.0, gap.flushProfile.speedRatio * 100.0,
            static_cast<unsigned long long>(gap.flushProfile.dwellUs / 1000ULL),
            m_edm94.gapFlushParameters.strokeRatio * 100.0, m_edm94.gapFlushParameters.speedRatio * 100.0,
            static_cast<unsigned long long>(m_edm94.gapFlushParameters.dwellUs / 1000ULL),
            EDM107::SameFlushProfile(m_edm94.gapFlushParameters, gap.flushProfile) ? 1U : 0U), "FLUSH_PARAMETERS");
}

void NCManager::LogEDM96SameThread(const char* event, const EDM96::Feedback& gap) const noexcept
{
    if (m_edm94.gapHistoryServo) { LogEDM126SameThread(event); return; }
    if (EDM116::IsModalFixture(m_edm94.gapModeFixture)) { LogEDM127SameThread(event, EDM114::RequiredRows); return; }
    if (m_edm94.gapFlushProfile)
    { LogEDM105SameThread(event, gap); return; }
    if (m_edm94.gapPairedProfile)
    { LogEDM104SameThread(event, gap); return; }
    if (m_edm94.gapEnduranceProfile)
    { LogEDM102SameThread(event, gap); return; }
    if (m_edm94.gapContinuousProfile)
    { LogEDM100SameThread(event, gap); return; }
    if (m_edm94.gapBurstProfile)
    { LogEDM99SameThread(event, gap); return; }
    if (m_edm94.gapNoiseProfile)
    { LogEDM98SameThread(event, gap); return; }
    if (m_edm94.gapGainProfile)
    { LogEDM97SameThread(event, gap); return; }
    char record[1024]{};
    const int length = std::snprintf(record, sizeof(record),
        "[EDM96] event=%.32s session=%llu epoch=%llu ownerGen=%u step=%u phase=%u direction=%s voltage=%.9g qPulse=%.12g vPulsePerSec=%.12g lengthPulse=%.12g accepted=%llu completed=%llu reverseSegments=%llu zeroCrossings=%llu reverseTicks=%llu returnForwardTicks=%llu tick=%llu sourceSequence=%llu sourceUs=%llu freshFrames=%llu heldFrames=%llu maxAgeUs=%llu active=%u segmentActive=%u segmentCompleted=%u faulted=%u reason=%s midSegmentReverse=1 output=GUARDED_CSV dischargeCommand=NONE\n",
        event ? event : "UNKNOWN", static_cast<unsigned long long>(gap.session),
        static_cast<unsigned long long>(gap.epoch), static_cast<unsigned>(gap.ownerGeneration), gap.expectedStep,
        gap.phase, gap.direction > 0 ? "FORWARD" : gap.direction < 0 ? "RETREAT" : "QUALIFYING",
        gap.voltageV, gap.q, gap.v, gap.length, static_cast<unsigned long long>(gap.acceptedSegments),
        static_cast<unsigned long long>(gap.completedSegments), static_cast<unsigned long long>(gap.reverseSegments),
        static_cast<unsigned long long>(gap.zeroCrossings), static_cast<unsigned long long>(gap.reverseTicks),
        static_cast<unsigned long long>(gap.returnForwardTicks), static_cast<unsigned long long>(gap.tick),
        static_cast<unsigned long long>(gap.sourceSequence), static_cast<unsigned long long>(gap.sourceCapturedUs),
        static_cast<unsigned long long>(gap.freshFrames), static_cast<unsigned long long>(gap.heldFrames),
        static_cast<unsigned long long>(gap.maximumAgeUs), gap.active ? 1U : 0U, gap.segmentActive ? 1U : 0U,
        gap.segmentCompleted ? 1U : 0U, gap.faulted ? 1U : 0U, EDM96::ReasonName(gap.reason));
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM96] event=LOG_FORMAT_ERROR output=NONE\n");
}

bool NCManager::ReadEDM96CompletedSegmentsSameThread(unsigned completed) const noexcept
{
    const auto& s = m_edm94;
    EDM96::Feedback gap{};
    if (s.gapHistoryServo)
        return completed > 0U && completed <= s.moves && m_motion.ReadEDM96Feedback(gap) &&
            IsEDM97TuningCurrentSameThread(gap) && IsEDM126CurrentSameThread(completed) &&
            gap.session == s.session && gap.epoch == m_motion.GetCurrentExecutionEpoch() &&
            gap.ownerGeneration == s.lease.generation && !gap.faulted && gap.reason == EDM96::Reason::None &&
            !gap.segmentActive && ((!s.finishPending && !s.complete) ? gap.active && gap.segmentCompleted : true) &&
            gap.expectedStep == completed && gap.acceptedSegments == completed && gap.completedSegments == completed &&
            gap.tick && gap.nowUs && gap.sourceSequence && gap.sourceCapturedUs &&
            gap.nowUs >= gap.sourceCapturedUs && gap.nowUs - gap.sourceCapturedUs <= EDM96::MaximumSourceAgeUs &&
            std::isfinite(gap.q) && std::isfinite(gap.v) && std::isfinite(gap.length) &&
            gap.length > 0.0 && gap.q == gap.length && gap.v == 0.0;
    if (!s.gapSegmentProfile || completed == 0U || completed > s.moves || completed > (s.gapEnduranceProfile ? EDM102::MaximumSteps : s.gapContinuousProfile ? EDM100::MaximumSteps : 4U) ||
        !m_motion.ReadEDM96Feedback(gap) || gap.session != s.session ||
        !IsEDM97TuningCurrentSameThread(gap) ||
        gap.epoch != m_motion.GetCurrentExecutionEpoch() || gap.ownerGeneration != s.lease.generation ||
        gap.faulted || gap.reason != EDM96::Reason::None || gap.segmentActive ||
        ((!s.finishPending && !s.complete) && (!gap.active || !gap.segmentCompleted)) ||
        gap.expectedStep != completed || gap.acceptedSegments != completed || gap.completedSegments != completed ||
        gap.reverseSegments != completed || gap.zeroCrossings != (s.gapBurstProfile ? 4ULL : 2ULL) * completed ||
        gap.reverseTicks < (s.gapBurstProfile ? 2ULL : 1ULL) * completed ||
        gap.returnForwardTicks < (s.gapBurstProfile ? 2ULL : 1ULL) * completed ||
        gap.tick == 0ULL || gap.nowUs == 0ULL || gap.sourceSequence == 0ULL || gap.sourceCapturedUs == 0ULL ||
        gap.nowUs < gap.sourceCapturedUs || gap.nowUs - gap.sourceCapturedUs > EDM96::MaximumSourceAgeUs ||
        !std::isfinite(gap.q) || !std::isfinite(gap.v) || !std::isfinite(gap.length) ||
        gap.length <= 0.0 || gap.q != gap.length || gap.v != 0.0) return false;
    if (s.gapModeFlush && (gap.flushZeroTicks != 0ULL || ((completed & 1U) && gap.flushRetreat != gap.flushAnchor))) return false;
    if (s.gapFlushProfile && (gap.flushStarted != (completed + 1ULL) / 2ULL ||
        gap.flushCompleted != (completed + 1ULL) / 2ULL ||
        (!s.gapModeFlush && gap.flushZeroTicks < 2ULL * EDM107::FlushDwellTicks(gap.flushProfile) * gap.flushCompleted) ||
        gap.flushPhase != ((completed & 1U) ? EDM105::Phase::Complete : EDM105::Phase::None) ||
        ((completed & 1U) ? (gap.flushAnchor <= 0.0 || gap.flushAnchor > gap.length ||
            (!s.gapModeFlush && gap.flushRetreat >= gap.flushAnchor) || gap.flushReturned != gap.flushAnchor) :
            (gap.flushAnchor != 0.0 || gap.flushRetreat != 0.0 || gap.flushReturned != 0.0)))) return false;
    if (s.gapNoiseProfile && (gap.quietWindows != 2ULL * completed ||
        gap.quietZeroTicks < 1600ULL * completed || gap.quietSignChanges != 0ULL ||
        gap.noiseFreshFrames < 2ULL * completed)) return false;
    if (s.gapBurstProfile && (gap.suppressedPulseWindows != 4ULL * completed ||
        gap.qualifiedRetreatWindows != 2ULL * completed || gap.reverseEpisodes != 2ULL * completed ||
        gap.pulseLowFreshFrames < 4ULL * completed)) return false;
    return true;
}

bool NCManager::WaitEDM96InitialSourceSameThread()
{
    auto& s = m_edm94;
    const MotionExecutionEpoch epoch = m_motion.GetCurrentExecutionEpoch();
    if (!s.gapSegmentProfile || !s.latched || !s.ready || s.armPending || s.cancelled ||
        s.finishPending || s.complete || s.resetSeen || s.resetCommitted || s.releaseRequested || s.next != 0U ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY || m_isSingleBlockEnabled ||
        Close_System_Com_flag || !m_externalReadyInterlock || AlarmManager::GetInstance().HasAlarm() ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || !m_motion.IsGroupDone() ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        (s.gapWaiting && s.gapWaitEpoch != epoch))
    { RejectEDM94SameThread("EDM96_SOURCE_WAIT_SCOPE", m_gapDryRun.sourceLine); return false; }
    if (!s.gapWaiting)
    {
        s.gapWaiting = true; s.gapWaitEpoch = epoch;
        s.gapWaitFirstUs = s.gapWaitLastUs = s.gapWaitLastTick = 0ULL; s.gapWaitPolls = 1U;
        RtPrintf("[%s] event=WAIT_QUALIFIED_SOURCE session=%llu epoch=%llu budgetUs=%llu nextMotionSubmitted=0\n",
            s.gapWorkTime ? (E137FlushStageArm(s.gapHistoryArm) ? "EDM137" : E136CurveArm(s.gapHistoryArm) ? "EDM136" : E135ReferenceArm(s.gapHistoryArm) ? "EDM135" : E134MachiningArm(s.gapHistoryArm) ? "EDM134" : E133DeepArm(s.gapHistoryArm) ? "EDM133" : E132HeightArm(s.gapHistoryArm) ? "EDM132" : E131SpeedArm(s.gapHistoryArm) ? "EDM131" : "EDM130") : s.gapPhysicalAd ? "EDM129" : s.gapFlushProfile ? "EDM105" : s.gapPairedProfile ? "EDM104" : s.gapEnduranceProfile ? "EDM102" : s.gapContinuousProfile ? "EDM100" : s.gapBurstProfile ? "EDM99" : s.gapNoiseProfile ? "EDM98" : s.gapGainProfile ? "EDM97" : "EDM96",
            static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(epoch),
            EDM149::IsManualVoltageTestArm(s.gapHistoryArm) ? 30000000ULL : 5000000ULL);
    }
    EDM96::Feedback gap{};
    const bool read = m_motion.ReadEDM96Feedback(gap);
    if (read && gap.session == s.session && gap.faulted)
    { LogEDM96SameThread("SOURCE_FAULT", gap); RejectEDM94SameThread("EDM96_GAP_SOURCE_FAULT", m_gapDryRun.sourceLine); return false; }
    if (read && gap.session == s.session && !IsEDM97TuningCurrentSameThread(gap))
    { RejectEDM94SameThread(s.gapFlushProfile ? "EDM105_TUNING_IDENTITY" : s.gapPairedProfile ? "EDM104_TUNING_IDENTITY" : s.gapEnduranceProfile ? "EDM102_TUNING_IDENTITY" : s.gapContinuousProfile ? "EDM100_TUNING_IDENTITY" : s.gapBurstProfile ? "EDM99_TUNING_IDENTITY" : s.gapNoiseProfile ? "EDM98_TUNING_IDENTITY" :
        "EDM97_TUNING_IDENTITY", m_gapDryRun.sourceLine); return false; }
    const bool identity = read && gap.session == s.session && gap.epoch == epoch &&
        gap.ownerGeneration == s.lease.generation && gap.expectedStep == 0U;
    const bool fresh = identity && gap.tick != 0ULL && gap.tick > s.gapWaitLastTick &&
        gap.nowUs != 0ULL && gap.nowUs > s.gapWaitLastUs;
    if (fresh)
    {
        if (s.gapWaitFirstUs == 0ULL) s.gapWaitFirstUs = gap.nowUs;
        s.gapWaitLastUs = gap.nowUs; s.gapWaitLastTick = gap.tick;
    }
    const bool manualVoltage = EDM149::IsManualVoltageTestArm(s.gapHistoryArm);
    if (s.gapWaitPolls >= (manualVoltage ? 3000U : 500U) || (s.gapWaitFirstUs != 0ULL &&
        s.gapWaitLastUs >= s.gapWaitFirstUs && s.gapWaitLastUs - s.gapWaitFirstUs >=
            (manualVoltage ? 30000000ULL : 5000000ULL)))
    { RejectEDM94SameThread("EDM96_SOURCE_WAIT_TIMEOUT", m_gapDryRun.sourceLine); return false; }
    if (!fresh || !gap.active || gap.faulted || gap.reason != EDM96::Reason::None ||
        gap.segmentActive || gap.segmentCompleted || !gap.gateReady || gap.direction != 1 ||
        gap.acceptedSegments != 0ULL || gap.completedSegments != 0ULL || gap.reverseSegments != 0ULL ||
        gap.zeroCrossings != 0ULL || gap.reverseTicks != 0ULL || gap.returnForwardTicks != 0ULL ||
        (s.gapNoiseProfile && (gap.noiseFreshFrames != 0ULL || gap.quietZeroTicks != 0ULL ||
            gap.quietWindows != 0ULL || gap.quietSignChanges != 0ULL)) ||
        (s.gapBurstProfile && (gap.pulseLowFreshFrames != 0ULL || gap.suppressedPulseWindows != 0ULL ||
            gap.qualifiedRetreatWindows != 0ULL || gap.reverseEpisodes != 0ULL)) ||
        (s.gapFlushProfile && (gap.flushPhase != EDM105::Phase::None || gap.flushStarted != 0ULL ||
            gap.flushCompleted != 0ULL || gap.flushZeroTicks != 0ULL || gap.flushAnchor != 0.0 ||
            gap.flushRetreat != 0.0 || gap.flushReturned != 0.0)) ||
        gap.sourceSequence == 0ULL || gap.sourceCapturedUs == 0ULL ||
        gap.nowUs < gap.sourceCapturedUs || gap.nowUs - gap.sourceCapturedUs > EDM96::MaximumSourceAgeUs)
        return false;
    if (s.gapHistoryServo && !WaitEDM126InitialBoundarySameThread()) return false;
    if (s.gapModeFlush && !WaitEDM116InitialBoundarySameThread()) return false;
    s.gapWaiting = false;
    LogEDM96SameThread("SOURCE_QUALIFIED", gap);
    return true;
}

bool NCManager::WaitEDM94SameThread()
{
    const bool wasHistoryParentReleasePending=m_edm237.releasePending;
    const bool wasMixedParentReleasePending=m_edm234.releasePending;
    // ProcessTask services an active GAP deadline before calling this wait.
    // Avoid counting that same scan twice; the legacy P85/P86 path is intact.
    if ((!m_edm94.gapBoundaryProfile && !m_edm94.gapSegmentProfile) || !m_edm94.gapWaiting)
        ServiceEDM94SameThread();
    auto& s = m_edm94; const auto& f = s.feedback;
    if(wasHistoryParentReleasePending&&!m_edm237.releasePending&&m_state==NCState::RUN&&s.cancelled)
    {RejectEDM94SameThread("EDM237_PARENT_RELEASE_CANCELLED",m_gapDryRun.sourceLine);return false;}
    if(m_edm237.releasePending)
    {
        if(ConsumeEDM237ParentReleaseSameThread())return true;
        if(m_edm237.releasePending&&++s.polls>1000U)RejectEDM94SameThread("EDM237_PARENT_RELEASE_TIMEOUT",m_gapDryRun.sourceLine);
        return false;
    }
    if(wasMixedParentReleasePending&&!m_edm234.releasePending&&m_state==NCState::RUN&&s.cancelled)
    {RejectEDM94SameThread("EDM234_PARENT_RELEASE_CANCELLED",m_gapDryRun.sourceLine);return false;}
    if(m_edm234.releasePending)
    {
        if(ConsumeEDM234ParentReleaseSameThread())return true;
        if(m_edm234.releasePending&&++s.polls>1000U)RejectEDM94SameThread("EDM234_PARENT_RELEASE_TIMEOUT",m_gapDryRun.sourceLine);
        return false;
    }
    if (!s.latched || s.cancelled || s.releaseRequested || m_state != NCState::RUN) return false;
    if (s.gapModeInterruption && s.finishPending)
    { RequestEDM118HoldSameThread(false, "NORMAL_FINISH_FORBIDDEN"); return false; }
    if (f.identity.session == s.session && f.identity.ownerGeneration == s.lease.generation)
    {
        if (s.armPending && f.armAccepted && f.armedReady && f.active && !f.forceZero && !f.faulted && !f.cancelled)
        {
            s.armPending = false; s.ready = true;
            LogEDM94SameThread("ARMED", "FROZEN_SIX_AXIS_PHYSICAL_GUARD");
            if (s.gapSegmentProfile) return WaitEDM96InitialSourceSameThread();
            return !s.gapBoundaryProfile || WaitEDM95BoundarySameThread(0U);
        }
        if (s.gapSegmentProfile && s.ready && !s.finishPending && s.next == 0U)
            return WaitEDM96InitialSourceSameThread();
        if (s.gapBoundaryProfile && s.ready && !s.finishPending && s.next == 0U)
            return WaitEDM95BoundarySameThread(0U);
        EDM95::Feedback gapEnd{};
        const bool gapComplete = !s.gapBoundaryProfile || (m_motion.ReadEDM95Feedback(gapEnd) &&
            gapEnd.session == s.session && gapEnd.epoch == m_motion.GetCurrentExecutionEpoch() &&
            gapEnd.ownerGeneration == s.lease.generation && !gapEnd.faulted &&
            gapEnd.expectedStep == s.moves && gapEnd.acceptedSegments == s.moves &&
            gapEnd.forwardSegments == s.moves / 2U && gapEnd.retreatSegments == s.moves / 2U);
        const bool segmentComplete = !s.gapSegmentProfile || ReadEDM96CompletedSegmentsSameThread(s.moves);
        if (s.finishPending && s.gapModeFlush && !IsEDM116CurrentSameThread(EDM114::RequiredRows))
        { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_FINAL_MODE_EVIDENCE" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_FINAL_MODE_EVIDENCE" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_FINAL_MODE_EVIDENCE" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_FINAL_MODE_EVIDENCE" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_FINAL_MODE_EVIDENCE" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_FINAL_MODE_EVIDENCE" : m_edm94.gapModeInterruption ? "EDM118_FINAL_MODE_EVIDENCE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_FINAL_MODE_EVIDENCE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_FINAL_MODE_EVIDENCE" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_FINAL_MODE_EVIDENCE" : "EDM116_FINAL_MODE_EVIDENCE"), m_gapDryRun.sourceLine); return false; }
        if (s.finishPending && s.gapCurveB2 && !IsEDM115CurrentSameThread(EDM114::RequiredRows))
        { RejectEDM94SameThread("EDM115_FINAL_B2_EVIDENCE", m_gapDryRun.sourceLine); return false; }
        if (s.finishPending && s.gapPhysicalCurve && !IsEDM114CurrentSameThread(m_edm94.moves))
        { RejectEDM94SameThread("EDM114_FINAL_CURVE_EVIDENCE", m_gapDryRun.sourceLine); return false; }
        if (s.finishPending && s.gapCurveRuntime && !IsEDM113RuntimeCurrentSameThread(EDM109::Capacity))
        { RejectEDM94SameThread("EDM113_FINAL_RUNTIME_EVIDENCE", m_gapDryRun.sourceLine); return false; }
        if (s.finishPending && !IsEDM168AutoHoldCheckpointCompleteSameThread())
        { RejectEDM94SameThread("EDM168_FINAL_CHECKPOINT_NOT_COMPLETED", m_gapDryRun.sourceLine); return false; }
        if (s.finishPending && !E156FinishTargetMatches(s, m_edm130.recipe, f, m_edm114.retained))
        { RejectEDM94SameThread("EDM156_FINAL_TARGET_RECEIPT", m_gapDryRun.sourceLine); return false; }
        if (s.finishPending && gapComplete && segmentComplete &&
            (!s.gapCurveGeometry || IsEDM112GeometryCurrentSameThread(true)) &&
            (!s.gapB2CrossSegment || (m_edm110.complete && IsEDM110CompleteSameThread(false))) &&
            (!s.gapCommonPath || (m_edm109.ledger.Valid() && m_edm109.ledger.Count() == EDM109::Capacity &&
                m_edm109.plans == EDM109::Capacity - 1U && m_edm109.mappedSegments == EDM109::Capacity)) && f.active && !f.released && f.pass && f.stopped && f.forceZero && f.finishRequested && !f.faulted && !f.cancelled &&
            EDM94::MovementProven(f, E156RequiredMovementMask(s, m_edm130.recipe)) && f.sentFrames >= 200ULL &&
            f.settledCycles >= f.requiredCycles && f.requiredCycles >= 200U && f.lastSentTick != 0ULL &&
            f.lastSentTick <= f.lastTick && m_motion.IsGroupDone())
        {
            s.finishPending = false; s.complete = true; s.terminalLogged = true;
            if(m_edm237.authorized&&!FinishEDM237ParentSameThread())return false;
            if(m_edm234.authorized&&!FinishEDM234ParentSameThread())return false;
            LogEDM94SameThread("PASS", "PHYSICAL_ZERO_WINDOW_WAIT_M30_RETIREMENT"); return true;
        }
    }
    if (++s.polls > 1000U) RejectEDM94SameThread("WAIT_LIMIT", m_gapDryRun.sourceLine);
    return false;
}

void NCManager::ResetEDM94SameThread() noexcept
{
    m_edm237 = EDM237NCState{};
    m_edm234 = EDM234NCState{};
    m_edm233 = EDM233NCState{};
    InvalidateEDM109CommonPathSameThread("RESET");
    if (!m_edm94.latched) return;
    if (m_edm94.gapModeInterruption && !m_edm118.terminalLogged)
    {
        m_edm118.resetInterrupted = m_edm118.terminalLogged = true;
        LogEDM118SameThread("STOP_UNPROVEN_RESET_REQUIRED", "RESET_BEFORE_STOP_PROOF");
    }
    m_edm94.resetSeen = true;
    CancelEDM94SameThread("RESET");
}

void NCManager::ReleaseEDM94AfterResetSameThread() noexcept
{
    if (!m_edm94.latched || !m_edm94.resetSeen) return;
    // This flag is set only by the existing successful Reset/rebase commit.
    // A later matching rejected-Arm receipt can retire this empty admission.
    m_edm94.resetCommitted = true;
    const auto& f = m_edm94.feedback;
    if (f.identity.session == m_edm94.session && f.identity.epoch == m_edm94.armEpoch &&
        f.identity.ownerGeneration == m_edm94.lease.generation &&
        !m_edm94.everArmed && !f.armAccepted && !f.active && f.faulted &&
        f.sentFrames == 0ULL && f.nonzeroSentMask == 0U && f.actualMovedMask == 0U && f.commandMovedMask == 0U)
    {
        LogEDM94SameThread("EMPTY_ADMISSION_RELEASED", "RESET_COMMITTED_NEVER_ARMED_NO_STOP_PROOF");
        m_edm94.latched = m_edm94.armPending = m_edm94.ready = m_edm94.finishPending = false;
        return;
    }
    if (m_edm94.releaseRequested) return;
    // The active RT guard independently requires its fresh physical zero window.
    m_edm94.releaseRequested = true;
    m_motion.RequestEDM94Release(m_edm94.session);
    LogEDM94SameThread("RELEASE_REQUEST", "EXISTING_RESET_REBASE_COMMITTED");
}

// A completed authored M00 plus a later successful explicit Cycle Start is an
// operator confirmation only. It is not proof of physical on-site presence.
