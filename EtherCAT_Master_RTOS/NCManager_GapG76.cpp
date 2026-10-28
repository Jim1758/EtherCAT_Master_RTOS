// EDM267 FIX1: independent NC compilation unit; original function bodies preserved.
#include "NCManager_GapInternal.h"
#include "EDM275G76Diagnostic.h"

using namespace NCGapDetail;

namespace
{
    const char* E275ObservationState(const EDM275::Snapshot& d) noexcept
    {
        if ((d.flags & EDM275::RowComplete) != 0U) return "ROW_END";
        const bool zero = (d.flags & EDM275::MappedVelocityZero) != 0U;
        if (zero && (d.flags & EDM275::PhysicalValid) == 0U) return "WAIT_PHYSICAL";
        const auto phase = static_cast<EDM128::Phase>(d.phase);
        const bool dwell = phase == EDM128::Phase::AnchorDwell || phase == EDM128::Phase::TopDwell ||
            phase == EDM128::Phase::ReturnDwell || phase == EDM128::Phase::CenterDwell ||
            phase == EDM128::Phase::CenterReturnDwell;
        if (zero && dwell) return "WAIT_DWELL";
        if (zero && (d.flags & (EDM275::AtCurrentEnd | EDM275::AwaitingSeam | EDM275::CompleteReady)) != 0U)
            return "ROW_END";
        return "MOVING";
    }

    void E275LogObservation(const EDM275::Snapshot& d, std::uint64_t publication) noexcept
    {
        // NC is the single diagnostic reader. A new session earns one initial
        // image; subsequent state and axis rows are limited to one set/second.
        static std::uint64_t loggedSession = 0ULL, loggedUs = 0ULL, loggedPublication = 0ULL;
        if (publication == loggedPublication || !d.session || !d.nowUs) return;
        if (d.session == loggedSession && (d.nowUs <= loggedUs || d.nowUs - loggedUs < 1000000ULL)) return;
        loggedSession = d.session; loggedUs = d.nowUs; loggedPublication = publication;
        const std::uint64_t stableElapsed = d.stableStartUs && d.nowUs >= d.stableStartUs ?
            d.nowUs - d.stableStartUs : 0ULL;
        PrintNCG76DiagnosticLine("[EDM275] event=OBSERVE pub=%llu session=%llu epoch=%llu gen=%llu tick=%llu phase=%s child=%u done=%u state=%s eval=%04X failed=%04X axes=%02X flags=%04X localMm=%.9g lengthMm=%.9g vMmSec=%.9g stable=%u elapsedUs=%llu dwellUs=%llu authority=NONE\n",
            static_cast<unsigned long long>(publication), static_cast<unsigned long long>(d.session),
            static_cast<unsigned long long>(d.epoch), static_cast<unsigned long long>(d.ownerGeneration),
            static_cast<unsigned long long>(d.tick), EDM128::PhaseName(static_cast<EDM128::Phase>(d.phase)),
            d.child, d.completedChildren, E275ObservationState(d), d.evaluated, d.failed,
            d.axisMask, d.flags, d.localMm, d.lengthMm, d.velocityMmSec, d.stableCycles,
            static_cast<unsigned long long>(stableElapsed), static_cast<unsigned long long>(d.dwellUs));
        PrintNCG76DiagnosticLine("[EDM275] event=CLOCK session=%llu tick=%llu sourceUs=%llu physicalTick=%llu mappedTick=%llu sentTick=%llu proofTick=%llu gates=FIRST_FAILURE_UNEVALUATED_IS_UNKNOWN range=CANDIDATE authority=NONE\n",
            static_cast<unsigned long long>(d.session), static_cast<unsigned long long>(d.tick),
            static_cast<unsigned long long>(d.nowUs), static_cast<unsigned long long>(d.physicalTick),
            static_cast<unsigned long long>(d.mappedTick), static_cast<unsigned long long>(d.lastSentTick),
            static_cast<unsigned long long>(d.proofTick));
        for (unsigned i = 0U; i < 6U; ++i)
        {
            const auto& a = d.axis[i];
            PrintNCG76DiagnosticLine("[EDM275] event=AXIS session=%llu tick=%llu axis=%c cmd=%.12g act=%.12g ref=%.12g lag=%.9g window=%.9g range=%.9g vCmd=%.9g vLogical=%.9g target=%.9g flags=%02X comp=%u unit=PULSE_PPS evidence=RT_SNAPSHOT authority=NONE\n",
                static_cast<unsigned long long>(d.session), static_cast<unsigned long long>(d.tick), "XYZCUV"[i],
                a.commandPulse, a.actualPulse, a.referencePulse, a.commandPulse - a.actualPulse,
                a.windowPulse, a.rangePulse, a.commandPps, a.logicalPps, a.targetPps,
                a.flags, (a.flags & EDM275::CompensationReady) != 0U ? 1U : 0U);
        }
    }
}

bool NCManager::IsEDM149ConfirmationBlockSameThread(const NCParsedBlock& block, int pc) const noexcept
{
    if (pc < 0 || pc == (std::numeric_limits<int>::max)() || !E149M00Shape(block)) return false;
    const auto* next = GetBaseProgramCache().TryGetLine(pc + 1);
    if (!next||!E251SquareConfirmationSource(GetBaseProgramCache(),pc+1)) return false;
    EDM97::Tuning tuning{}; EDM107::FlushProfile profile{};
    return E108ProfileOnlyNPQRSIJK(next->parsedBlock,230U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,232U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,234U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,236U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,238U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,240U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,242U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,244U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,246U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,248U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,250U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,252U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,254U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,256U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,258U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,260U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,262U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,264U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,266U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,268U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,270U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,272U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,274U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,276U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,278U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,280U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,282U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,284U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,286U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,288U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,290U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,292U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,294U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,296U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,298U,tuning,profile) ||
        E108ProfileOnlyNPQRSIJK(next->parsedBlock,300U,tuning,profile);
}

void NCManager::CaptureEDM149ConfirmationSameThread(int pc, NCBlockDispatchId dispatch) noexcept
{
    m_edm149Confirmation = EDM149Confirmation{};
    const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    const auto& transaction = m_gmBlockTransaction.snapshot;
    if (!transaction.active || !transaction.fromMainProgram || transaction.sourcePC != pc ||
        transaction.dispatchId != dispatch || transaction.phase != NCGMBlockTransactionPhase::READY_TO_FINALIZE ||
        transaction.postAction != NCGMBlockPostAction::PROGRAM_STOP_M00 ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY || !m_macroStack.empty() ||
        m_edm94.latched || !authored || !IsEDM149ConfirmationBlockSameThread(authored->parsedBlock,pc) ||
        !dispatch || !m_pathCoreLiveBookkeeping.currentRunToken ||
        !GetBaseProgramCache().GetGeneration() || m_isSingleBlockEnabled || AlarmManager::GetInstance().HasAlarm()) return;
    auto& c = m_edm149Confirmation;
    c.run = m_pathCoreLiveBookkeeping.currentRunToken; c.cache = GetBaseProgramCache().GetGeneration();
    c.dispatch = dispatch; c.armPC = pc + 1; c.startRequests = m_startDiagnosticRequests;
    const auto* arm = GetBaseProgramCache().TryGetLine(c.armPC);
    std::uint32_t selected = 0U;
    if (!arm || !EDM77::Detail::UnsignedLiteral(arm->parsedBlock.expression('P'),EDM173::B0ArmP,selected) ||
        (!EDM149::IsTestArm(selected) && !EDM173::IsArm(selected))) { c=EDM149Confirmation{}; return; }
    c.arm = selected; c.waiting = true;
    RtPrintf("[%s] event=OPERATOR_CONFIRMATION result=WAIT_M00_START armPC=%d run=%llu cache=%llu M00dispatch=%llu motionEnabled=0 physicalInterlock=0 discharge=0\n",
        EDM149::IsSixVisibleFlushTestArm(c.arm) ? "EDM167" : EDM149::IsXYZVisibleFlushTestArm(c.arm) ? "EDM159" : EDM149::IsVisibleFlushTestArm(c.arm) ? "EDM158" : EDM149::IsPauseTestArm(c.arm) ? "EDM157" : EDM149::IsBoundedProgramTestArm(c.arm) ? "EDM156" : EDM149::IsSparseArcTestArm(c.arm) ? "EDM155" : EDM149::IsSparseTestArm(c.arm) ? "EDM154" : EDM149::IsOrderTestArm(c.arm) ? "EDM153" : EDM149::IsArcTestArm(c.arm) ? "EDM152" : EDM149::IsSixTestArm(c.arm) ? "EDM151" : "EDM149", c.armPC,static_cast<unsigned long long>(c.run),static_cast<unsigned long long>(c.cache),static_cast<unsigned long long>(c.dispatch));
}

void NCManager::CommitEDM149ConfirmationResumeSameThread() noexcept
{
    auto& c = m_edm149Confirmation;
    if (!c.waiting) return;
    if (m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY || !m_macroStack.empty() ||
        c.run != m_pathCoreLiveBookkeeping.currentRunToken || c.cache != GetBaseProgramCache().GetGeneration() ||
        GetBasePC() != c.armPC || m_startDiagnosticRequests <= c.startRequests || m_isSingleBlockEnabled ||
        m_waitCallback != WaitForCycleStartCallback || AlarmManager::GetInstance().HasAlarm() ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    { c = EDM149Confirmation{}; return; }
    const auto* arm = GetBaseProgramCache().TryGetLine(c.armPC);
    if (!arm || !EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('P'),c.arm)||
        !E251SquareConfirmationSource(GetBaseProgramCache(),c.armPC))
    { c = EDM149Confirmation{}; return; }
    c.waiting = false; c.confirmed = true; c.startRequests = m_startDiagnosticRequests;
    if (EDM173::IsArm(c.arm)) { m_g76PowerInhibited=true; ForceEDMPLCOutputsOffSameThread("EDM173_CONFIRMED"); }
    RtPrintf("[%s] event=OPERATOR_CONFIRMATION result=START_CONFIRMED armPC=%d run=%llu cache=%llu M00dispatch=%llu physicalInterlock=0 discharge=0\n",
        EDM149::IsSixVisibleFlushTestArm(c.arm) ? "EDM167" : EDM149::IsXYZVisibleFlushTestArm(c.arm) ? "EDM159" : EDM149::IsVisibleFlushTestArm(c.arm) ? "EDM158" : EDM149::IsPauseTestArm(c.arm) ? "EDM157" : EDM149::IsBoundedProgramTestArm(c.arm) ? "EDM156" : EDM149::IsSparseArcTestArm(c.arm) ? "EDM155" : EDM149::IsSparseTestArm(c.arm) ? "EDM154" : EDM149::IsOrderTestArm(c.arm) ? "EDM153" : EDM149::IsArcTestArm(c.arm) ? "EDM152" : EDM149::IsSixTestArm(c.arm) ? "EDM151" : "EDM149", c.armPC,static_cast<unsigned long long>(c.run),static_cast<unsigned long long>(c.cache),static_cast<unsigned long long>(c.dispatch));
}

bool NCManager::ConsumeEDM149ConfirmationSameThread(int armPC) noexcept
{
    if(m_edm237.authorized)return ConsumeEDM237ConfirmationSameThread(armPC);
    if(m_edm234.program==236U)return ConsumeEDM236ConfirmationSameThread(armPC);
    if(m_edm234.authorized&&armPC==m_edm234.firstArmPC+3)
    {
        auto& q=m_edm234;const auto& s=m_edm94;
        if(!IsEDM234SequenceScopeCurrentSameThread()||!q.firstRecipeBound||!q.firstReleased||q.releasePending||q.secondStarted||
            !q.firstSession||!q.firstFinishDispatch||q.releaseTick<=q.releaseFloorTick||s.session<=q.firstSession||
            s.armPC!=armPC||s.gapHistoryArm!=q.arms[1U]||s.run!=q.run||s.cache!=q.cache||s.armEpoch!=q.epoch||
            !s.lease.Matches(q.lease)||s.latched||!m_edm130.bound||GetBasePC()!=armPC||GetActiveDispatchPC()!=armPC||
            m_currentExecutingBlockDispatchId<=q.firstFinishDispatch)return false;
        q.secondStarted=true;m_edm149Confirmation=EDM149Confirmation{};return true;
    }
    if(m_edm233.authorized&&armPC==m_edm233.firstArmPC+3)
    {
        // Retained whole-sequence consent is consumed only after the exact
        // first-parent zero-release receipt. No START/confirmation is invented.
        const auto& q=m_edm233;const auto& s=m_edm94;
        if(!IsEDM233SequenceScopeCurrentSameThread()||!q.firstReleased||q.releasePending||q.secondStarted||
            !q.firstSession||!q.releaseTick||!q.firstFinishDispatch||q.releaseTick<=q.releaseFloorTick||
            s.session<=q.firstSession||s.armPC!=armPC||s.gapHistoryArm!=q.arms[1U]||
            s.run!=q.run||s.cache!=q.cache||s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||s.latched||
            GetBasePC()!=armPC||GetActiveDispatchPC()!=armPC||m_currentExecutingBlockDispatchId<=q.firstFinishDispatch)return false;
        m_edm233.secondStarted=true;
        m_edm149Confirmation=EDM149Confirmation{};
        return true;
    }
    const auto c = m_edm149Confirmation;
    m_edm149Confirmation = EDM149Confirmation{};
    return c.confirmed && !c.waiting && c.run && c.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        c.cache && c.cache == GetBaseProgramCache().GetGeneration() && c.armPC == armPC && armPC > 0 &&
        c.arm == m_edm94.gapHistoryArm && (EDM149::IsTestArm(c.arm) || EDM173::IsArm(c.arm)) &&
        GetBasePC() == armPC && m_mode == NCOperationMode::MEMORY && m_state == NCState::RUN &&
        c.dispatch && m_currentExecutingBlockDispatchId > c.dispatch && c.startRequests == m_startDiagnosticRequests &&
        m_macroStack.empty() && !m_isSingleBlockEnabled && !AlarmManager::GetInstance().HasAlarm();
}

// EDM173 is one authored compound instruction. These NC receipts authorize no
// output by themselves; the existing RT GAP/coordinator/guard owns each sample.
bool NCManager::IsEDM234BracketShapeValid(const NCBlock& b) noexcept
{return E234BracketLiteralShape(b);}

bool NCManager::IsEDM173BracketShapeValid(const NCBlock& b) noexcept
{
    if (b.isEmpty || b.isGoto || b.isBlockSkip || !b.hasG || b.gCode!=180 || b.gCount!=1 ||
        b.gCodes[0]!=180 || b.mCount || !b.has('P') ||
        (b.val('P')!=292.0 && b.val('P')!=293.0 && b.val('P')!=294.0 && b.val('P')!=295.0 && b.val('P')!=296.0 && b.val('P')!=297.0 && b.val('P')!=298.0 && b.val('P')!=299.0 && b.val('P')!=300.0 && b.val('P')!=301.0)) return false;
    const bool arm=b.val('P')==292.0||b.val('P')==294.0||b.val('P')==296.0||b.val('P')==298.0||b.val('P')==300.0;
    const bool squareB1=(b.val('P')==292.0&&b.has('L')&&b.val('L')==250.0)||
        (b.val('P')==294.0&&b.has('L')&&b.val('L')==251.0)||
        (b.val('P')==296.0&&b.has('L')&&b.val('L')==252.0)||
        (b.val('P')==298.0&&b.has('L')&&b.val('L')==253.0)||
        (b.val('P')==292.0&&b.has('L')&&b.val('L')==255.0)||
        (b.val('P')==294.0&&b.has('L')&&b.val('L')==256.0)||
        (b.val('P')==296.0&&b.has('L')&&b.val('L')==257.0)||
        (b.val('P')==298.0&&b.has('L')&&b.val('L')==258.0)||
        ((b.val('P')==292.0||b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&
            (b.val('L')==260.0||b.val('L')==261.0||b.val('L')==262.0||b.val('L')==263.0))||
        (b.val('P')==294.0&&b.has('L')&&b.val('L')==264.0);
    const bool expansion=b.has('L')&&std::isfinite(b.val('L'))&&std::floor(b.val('L'))==b.val('L')&&
        (((b.val('P')==292.0||b.val('P')==296.0||b.val('P')==298.0)&&b.val('L')>=265.0&&b.val('L')<=268.0)||
         (b.val('P')==294.0&&b.val('L')>=269.0&&b.val('L')<=272.0));
    const bool fixedRadius=b.has('L')&&std::isfinite(b.val('L'))&&std::floor(b.val('L'))==b.val('L')&&
        (((b.val('P')==292.0||b.val('P')==296.0||b.val('P')==298.0)&&b.val('L')>=273.0&&b.val('L')<=276.0)||
         (b.val('P')==294.0&&b.val('L')>=277.0&&b.val('L')<=280.0));
    const bool fixedRadiusWork=b.has('L')&&std::isfinite(b.val('L'))&&std::floor(b.val('L'))==b.val('L')&&
        (((b.val('P')==292.0||b.val('P')==296.0||b.val('P')==298.0)&&b.val('L')>=281.0&&b.val('L')<=284.0)||
         (b.val('P')==294.0&&b.val('L')>=285.0&&b.val('L')<=288.0));
    const bool sharedShort=b.has('L')&&b.val('L')==static_cast<double>(EDM130::G76SharedShortArmTag274)&&
        (b.val('P')==292.0||b.val('P')==296.0||b.val('P')==298.0);
    const bool autoB2=b.val('P')==294.0&&b.has('L')&&(b.val('L')==3.0||b.val('L')==234.0||b.val('L')==235.0||b.val('L')==236.0||(b.val('L')==237.0||b.val('L')==238.0||b.val('L')==241.0||b.val('L')==242.0||b.val('L')==243.0||b.val('L')==244.0||b.val('L')==245.0||b.val('L')==246.0||b.val('L')==247.0||b.val('L')==248.0));
    const bool rejectBeforeArm=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&b.val('L')==216.0;
    const bool centerNominalHold=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&b.val('L')==1.0;
    const bool centerRepeat=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&b.val('L')==230.0;
    const bool centerSecondFlushHold=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&b.val('L')==231.0;
    const bool centerSecondReturnHold=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&b.val('L')==232.0;
    const bool parentSequence=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&b.val('L')==233.0;
    const bool centerHistorySequence=b.has('L')&&(((b.val('P')==296.0||b.val('P')==298.0)&&(b.val('L')==239.0||b.val('L')==240.0||b.val('L')==241.0||b.val('L')==242.0||b.val('L')==243.0||b.val('L')==244.0))||(b.val('P')==296.0&&b.val('L')==245.0)||(b.val('P')==298.0&&b.val('L')==246.0)||(b.val('P')==296.0&&b.val('L')==247.0)||(b.val('P')==298.0&&b.val('L')==248.0));
    const bool centerFlushHold=(b.val('P')==296.0||b.val('P')==298.0)&&b.has('L')&&
        (b.val('L')==2.0||b.val('L')==3.0||b.val('L')==4.0||b.val('L')==5.0||b.val('L')==6.0||b.val('L')==7.0||b.val('L')==8.0||b.val('L')==9.0||b.val('L')==10.0||b.val('L')==11.0);
    if(sharedShort)
    {
        if(b.gotoTarget!=-1)return false;
        for(unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(b.gCodes[i]!=0)return false;
        for(int code:b.mCode)if(code!=-1)return false;
        for(char word='A';word<='Z';++word)if(!b.has(word)&&!EDM109::SameDouble(b.val(word),0.0))return false;
    }
    for(char word='A';word<='Z';++word)
        if(b.has(word) && (!std::isfinite(b.val(word)) ||
            (word!='N' && word!='P' && !(word=='L'&&(autoB2||rejectBeforeArm||centerNominalHold||centerFlushHold||centerRepeat||centerSecondFlushHold||centerSecondReturnHold||parentSequence||centerHistorySequence||squareB1||expansion||fixedRadius||fixedRadiusWork||sharedShort)) &&
                (!arm || (word!='Q' && word!='R' && word!='S' && word!='I' && word!='J' && word!='K'))))) return false;
    return !arm || (b.has('Q')&&b.has('R')&&b.has('S')&&b.has('I')&&b.has('J')&&b.has('K')&&
        b.val('Q')==8.0&&b.val('R')==1.0&&b.val('S')==1.0&&b.val('I')==10.0&&b.val('J')==50.0&&b.val('K')==100.0);
}

bool NCManager::IsEDM173OrbitBlockShapeValid(const NCBlock& b) noexcept
{
    if(b.isEmpty||b.isGoto||b.isBlockSkip||!b.hasG||b.gCode!=76||b.gCount!=1||b.gCodes[0]!=76||b.mCount||
        !b.has('P')||(b.val('P')!=11.0&&!((b.val('P')==12.0&&b.has('B')&&(b.val('B')==1.0||b.val('B')==2.0||b.val('B')==3.0||b.val('B')==4.0))||
            (b.val('P')==13.0&&b.has('B')&&(b.val('B')==1.0||b.val('B')==2.0||b.val('B')==3.0||b.val('B')==4.0))||
            (b.val('P')==16.0&&b.has('B')&&(b.val('B')==1.0||b.val('B')==2.0||b.val('B')==3.0||b.val('B')==4.0)&&E261PolygonOrbitShape(b))||E263ExpansionOrbitShape(b)||E266FixedRadiusOrbitShape(b)||E274SharedShortOrbitShape(b)))||!b.has('R')||!b.has('F')||b.val('F')!=0.12||
        E181EffectiveOrbitMode(b)==255U||!(b.has('X')||b.has('Y')||b.has('Z'))) return false;
    // Authored G90 values are coordinates, not travel. The frozen-frame
    // builder proves the actual displacement before any arm publication.
    for(char word='A';word<='Z';++word) if(b.has(word))
        if(!std::isfinite(b.val(word))||(word!='N'&&word!='P'&&word!='R'&&word!='F'&&word!='B'&&word!='X'&&word!='Y'&&word!='Z'&&word!='K'&&!(word=='Q'&&(b.val('P')==6.0||b.val('P')==16.0||b.val('P')==26.0||b.val('P')==7.0||b.val('P')==17.0||b.val('P')==27.0)))) return false;
    const unsigned mode=E181EffectiveOrbitMode(b);
    const double radiusLimit=mode==3U||mode==4U?EDM173::MaximumCenterRadiusMm:EDM173::MaximumRadiusMm;
    return b.val('R')>0.0&&b.val('R')<=radiusLimit&&
        (!b.has('K')||NCG76::ValidShapeAngle(b.val('K')));
}

// This scope is an NC-only continuation grant, never an RT motion permit.
// The frozen translation and first fresh START must survive both parents.
bool NCManager::IsEDM234SequenceScopeCurrentSameThread() noexcept
{
    if(m_edm234.program==236U)return IsEDM236SequenceScopeCurrentSameThread();
    const auto& q=m_edm234;int first=-1;std::array<unsigned,2U> arms{};unsigned program=0U,ordinaryCode=0U;
    return q.authorized&&q.run&&q.cache&&q.confirmationDispatch&&q.startRequests&&q.firstArmPC>0&&
        q.epoch!=MOTION_EXECUTION_EPOCH_INVALID&&q.lease.IsValid()&&q.lease.owner==MotionOwner::AUTO&&
        q.run==m_pathCoreLiveBookkeeping.currentRunToken&&q.cache==GetBaseProgramCache().GetGeneration()&&
        q.epoch==m_motion.GetCurrentExecutionEpoch()&&q.lease.Matches(m_programMotionLease)&&
        m_motion.IsMotionOwnerLeaseCurrent(q.lease)&&!m_motion.HasPendingSafetyOrRecoveryRequests()&&
        q.startRequests==m_startDiagnosticRequests&&q.recipeGeneration==m_recipe.Current().generation&&
        q.processGeneration==m_edmProcessTuningGeneration&&m_state==NCState::RUN&&m_mode==NCOperationMode::MEMORY&&
        !Close_System_Com_flag&&m_externalReadyInterlock&&!m_isSingleBlockEnabled&&!m_isG66Active&&
        m_macroStack.empty()&&!Homing.IsActive()&&!AlarmManager::GetInstance().HasAlarm()&&
        m_resetContinuationPhase==ResetContinuationPhase::IDLE&&CoordSys.IsTranslationRunFrozen()&&
        IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot())&&CoordSys.GetTranslationSnapshot().distanceMode==91&&
        IsPathCoreLiveNativeConfigCurrentSameThread()&&IsFixedTranslationTravelCurrentSameThread()&&
        SameNCTranslationSnapshot(q.translation,CoordSys.GetTranslationSnapshot())&&m_motion.MatchesNCTranslation(q.translation)&&
        E234ProgramShape(GetBaseProgramCache(),first,arms,&program,&ordinaryCode)&&first==q.firstArmPC&&arms==q.arms&&
        program==q.program&&ordinaryCode==q.ordinaryGCode;
}

bool NCManager::IsEDM234NextArmCurrentSameThread(int pc,unsigned arm) noexcept
{
    if(m_edm234.program==236U)return IsEDM236NextArmCurrentSameThread(pc,arm);
    const auto& q=m_edm234;const auto& s=m_edm94;
    return IsEDM234SequenceScopeCurrentSameThread()&&q.firstRecipeBound&&q.firstReleased&&!q.releasePending&&!q.secondStarted&&
        q.firstSession&&q.releaseTick>q.releaseFloorTick&&q.firstFinishDispatch&&q.motionDispatch&&
        !s.latched&&s.session==q.firstSession&&s.complete&&s.terminalLogged&&!s.cancelled&&!s.resetSeen&&!s.resetCommitted&&
        !s.releaseRequested&&!s.rtFaultReported&&pc==q.firstArmPC+3&&arm==q.arms[1U]&&GetActiveDispatchPC()==pc&&
        m_currentExecutingBlockDispatchId>q.firstFinishDispatch&&m_motion.IsGroupDone()&&m_motion.IsGroupNCDrained()&&
        !m_motion.GetCommandIngressSize()&&!m_motion.GetCommandReplaySize()&&!m_motion.GetAxisCommandMailboxDepth()&&
        !m_g76Orbit->active&&!m_pathFeed.pending&&!m_pathArc.pending&&!m_pathReplay.pending&&!m_pathHold.armed&&!m_pathHold.bound;
}

bool NCManager::BeginEDM234SequenceSameThread(int pc) noexcept
{
    const auto* chainArm=GetBaseProgramCache().TryGetLine(pc);
    if(m_edm234.program==236U||(chainArm&&EDM77::Detail::EqualLiteral(chainArm->parsedBlock.expression('L'),236U)))return BeginEDM236SequenceSameThread(pc);
    int first=-1;std::array<unsigned,2U> arms{};unsigned program=0U,ordinaryCode=0U;
    if(!E234ProgramShape(GetBaseProgramCache(),first,arms,&program,&ordinaryCode))return false;
    if(pc==first+3)return IsEDM234NextArmCurrentSameThread(pc,arms[1U]);
    const auto& c=m_edm149Confirmation;
    if(pc!=first||!c.confirmed||c.waiting||c.armPC!=pc||c.arm!=arms[0U]||
        c.run!=m_pathCoreLiveBookkeeping.currentRunToken||c.cache!=GetBaseProgramCache().GetGeneration()||
        !c.dispatch||m_currentExecutingBlockDispatchId<=c.dispatch||c.startRequests!=m_startDiagnosticRequests||
        !CoordSys.IsTranslationRunFrozen()||!m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))return false;
    m_edm234=EDM234NCState{};auto& q=m_edm234;
    q.translation=CoordSys.GetTranslationSnapshot();q.lease=m_programMotionLease;q.epoch=m_motion.GetCurrentExecutionEpoch();
    q.run=c.run;q.cache=c.cache;q.confirmationDispatch=c.dispatch;q.startRequests=c.startRequests;
    q.recipeGeneration=m_recipe.Current().generation;q.processGeneration=m_edmProcessTuningGeneration;q.firstArmPC=first;q.arms=arms;q.program=program;q.ordinaryGCode=ordinaryCode;q.authorized=true;
    return IsEDM234SequenceScopeCurrentSameThread();
}

bool NCManager::PreflightEDM234LineSameThread(int pc) noexcept
{
    const auto* chainArm=GetBaseProgramCache().TryGetLine(pc);
    if(m_edm234.program==236U||(chainArm&&EDM77::Detail::EqualLiteral(chainArm->parsedBlock.expression('L'),236U)))return PreflightEDM236OrdinarySameThread(pc);
    int first=-1;std::array<unsigned,2U> arms{};unsigned program=0U,ordinaryCode=0U;const auto& cache=GetBaseProgramCache();
    if(!E234ProgramShape(cache,first,arms,&program,&ordinaryCode)||(pc!=first&&pc!=first+3)||arms[pc==first?0U:1U]!=274U||
        !FreezeEDM173TranslationSameThread()||!BeginEDM234SequenceSameThread(pc))return false;
    m_edm94=EDM94NCState{};auto& s=m_edm94;
    s.program=true;s.inspectedCache=cache.GetGeneration();s.gapHistoryArm=274U;s.moves=1U;
    s.gapHistoryServo=s.gapHistoryMode=s.gapWorkTime=s.gapSegmentProfile=s.gapGainProfile=s.gapPhysicalCurve=true;
    s.gapModeFixture=EDM116::Fixture::ModalProgram156;s.gapTuning={8.0,1.0,1.0};
    if(!EDM107::ParseFlushProfile(10.0,50.0,100.0,s.gapFlushParameters))return false;
    s.pc[0U]=pc+1;s.pc[1U]=pc+2;s.pc[2U]=pc+3;s.feed[0U]=0.12;
    if(ordinaryCode==1U)s.increment[0U][2U]=-0.0015;
    else{s.increment[0U][0U]=ordinaryCode==2U?0.001:-0.001;s.increment[0U][1U]=0.001;}
    s.authoredGCode[0U]=static_cast<std::uint8_t>(ordinaryCode);s.selectorPC[0U]=pc+1;
    m_edm127=EDM127NCState{};m_edm129=EDM129NCState{};m_edm140=EDM140NCState{};m_edm141=EDM141NCState{};
    m_edm127.expectedMode[0U]=EDM116::Mode::B1;m_edm127.expectedOriginPC[0U]=pc+1;
    m_edm127.expectedOriginRevision[0U]=2ULL;m_edm127.authoredInline[0U]=true;
    return true;
}

bool NCManager::BindEDM234RecipeSameThread() noexcept
{
    if(m_edm234.program==236U)return BindEDM236RecipeSameThread();
    m_edm130=EDM130NCState{};m_edm200=EDM200AutoHoldState{};
    const auto& s=m_edm94;auto& q=m_edm234;const auto& cache=GetBaseProgramCache();const auto* arm=cache.TryGetLine(s.armPC);
    const unsigned stage=q.firstReleased?1U:0U;const unsigned kind=E234ParentKind(s.gapHistoryArm,q.program,q.ordinaryGCode);
    EDM97::Tuning tuning{};EDM107::FlushProfile flush{};EDMRecipe::Summary summary{};
    if(!m_edmProcessReady||!kind||!arm||!E234ArmSequence(arm->parsedBlock,s.gapHistoryArm)||
        !E108ProfileOnlyNPQRSIJK(arm->parsedBlock,s.gapHistoryArm,tuning,flush)||!IsEDM234SequenceScopeCurrentSameThread()||
        q.releasePending||q.secondStarted||s.latched||s.armPC!=q.firstArmPC+static_cast<int>(stage*3U)||s.gapHistoryArm!=q.arms[stage]||
        s.run!=q.run||s.cache!=q.cache||s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||!s.session||
        !EDM97::SameTuning(tuning,s.gapTuning)||!EDM107::SameFlushProfile(flush,s.gapFlushParameters))return false;
    if(stage)
    {
        if(!q.firstRecipeBound||!q.firstSession||s.session<=q.firstSession||!q.firstFinishDispatch||
            !q.releaseTick||q.releaseTick<=q.releaseFloorTick||m_currentExecutingBlockDispatchId<=q.firstFinishDispatch||
            m_recipeRecoveryDisplayOnly||m_recipeHoldSeen||m_edmState==EDMState::NOT_READY||!m_recipe.Ready())return false;
        summary=m_recipe.Current();
        if(!summary.catalogReady||!summary.tableSelected||!summary.rowSelected||summary.generation!=q.recipeGeneration)return false;
    }
    else if(!ReadEDMRecipeSummarySameThread(summary))return false;
    const auto binding=kind==2U?
        EDM130::BindG76B2Gap175(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,m_edmProcessTuningGeneration):
        EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(m_recipe,m_edmProcessProfile.machineProfileId,
            m_edmProcessProfile,m_edmProcessTuningGeneration);
    if(!binding.ready)return false;
    auto recipe=binding.recipe;recipe.diagnosticControlSource=3U;recipe.planRows=kind==2U?25U:1U;recipe.planMovementMask=kind==2U?7U:kind==1U?4U:3U;
    if(!EDM130::ValidHistoryRecipeBinding(kind==2U?EDM173::HistoryFixture:EDM149::ProgramHistoryFixture,recipe)||
        recipe.workTimeUs!=500000ULL||recipe.jumpHeightScaled!=1000ULL||recipe.jumpSpeedPercentScaled!=100000000U||
        recipe.flushHoldTest||recipe.holdRetreatMode||recipe.deepFlushCycleInterval||
        recipe.deepFlushHeightMultiplierScaled!=1000000ULL||!EDM97::ValidMotionTestCurve(s.gapTuning,recipe))return false;
    if(stage)
    {
        const unsigned firstKind=E234ParentKind(q.arms[0U],q.program,q.ordinaryGCode);
        if(!firstKind||q.firstRecipe.commissioningProfile!=(firstKind==2U?EDM130::G76B2Profile175:0U)||
            q.firstRecipe.planRows!=(firstKind==2U?25U:1U)||q.firstRecipe.planMovementMask!=(firstKind==2U?7U:firstKind==1U?4U:3U)||
            !EDM130::ValidHistoryRecipeBinding(firstKind==2U?EDM173::HistoryFixture:EDM149::ProgramHistoryFixture,q.firstRecipe))return false;
        auto expected=q.firstRecipe;expected.commissioningProfile=recipe.commissioningProfile;
        expected.planRows=recipe.planRows;expected.planMovementMask=recipe.planMovementMask;
        if(!EDM130::SameRecipe(expected,recipe))return false;
    }
    else{q.firstRecipe=recipe;q.firstRecipeBound=true;q.firstSession=s.session;}
    if(!IsEDM234SequenceScopeCurrentSameThread())return false;
    m_edm130.recipe=recipe;m_edm94.gapFlushParameters.speedRatio=EDM130::FrozenSpeedRatio(recipe);
    if(!EDM130::MatchesFlushSpeed(recipe,m_edm94.gapFlushParameters.speedRatio)||
        !EDM130::MatchesFlushHeight(recipe,m_edm94.gapFlushParameters.strokeRatio,m_edm94.gapFlushParameters.dwellUs))return false;
    m_edm130.bound=true;q.motionDispatch=0ULL;
    PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_RECIPE_BOUND parent=%u kind=%u session=%llu firstSession=%llu generation=%llu E5policy=FRESH_PER_PARENT globalRecipeScope=UNCHANGED discharge=0\n",
        q.program,stage+1U,kind,static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(q.firstSession),static_cast<unsigned long long>(summary.generation));
    return true;
}

bool NCManager::BindEDM234ParentEpochSameThread(MotionExecutionEpoch epoch,std::uint64_t dispatch,int pc) noexcept
{
    if(m_edm234.program==236U)return BindEDM236ParentEpochSameThread(epoch,dispatch,pc);
    auto& q=m_edm234;const auto& s=m_edm94;const unsigned stage=q.secondStarted?1U:0U;
    const auto expected=s.armEpoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:s.armEpoch+1U;
    if(!q.authorized||!q.firstRecipeBound||q.releasePending||q.firstReleased!=q.secondStarted||q.motionDispatch||
        !s.latched||!s.ready||s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        s.armPC!=q.firstArmPC+static_cast<int>(stage*3U)||s.gapHistoryArm!=q.arms[stage]||
        s.run!=q.run||s.cache!=q.cache||!s.lease.Matches(q.lease)||q.epoch!=s.armEpoch||epoch!=expected||
        epoch!=m_motion.GetCurrentExecutionEpoch()||!dispatch||dispatch<=s.armDispatch||pc!=s.pc[0U]||
        (stage?s.session<=q.firstSession:s.session!=q.firstSession))return false;
    q.epoch=epoch;
    if(!IsEDM234SequenceScopeCurrentSameThread())return false;
    q.motionDispatch=dispatch;
    PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_EPOCH_BOUND parent=%u kind=%u session=%llu armEpoch=%llu liveEpoch=%llu dispatch=%llu proof=EXACT_NATIVE_OR_COMPLETED_GEOMETRY discharge=0\n",
        q.program,stage+1U,E234ParentKind(s.gapHistoryArm,q.program,q.ordinaryGCode),static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.armEpoch),
        static_cast<unsigned long long>(epoch),static_cast<unsigned long long>(dispatch));
    return true;
}

bool NCManager::FinishEDM234ParentSameThread()
{
    if(m_edm234.program==236U)return FinishEDM236ParentSameThread();
    auto& q=m_edm234;auto& s=m_edm94;const auto& f=s.feedback;
    if(!IsEDM234SequenceScopeCurrentSameThread()||!q.firstRecipeBound||!q.motionDispatch||
        !s.latched||!s.complete||!s.terminalLogged||s.armPC!=q.firstArmPC+(q.secondStarted?3:0)||
        s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=s.armPC+2||m_waitingBlockDispatchId!=q.currentFinishDispatch)
    {RejectEDM94SameThread("EDM234_FINISHED_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(!q.secondStarted)
    {
        const unsigned kind=E234ParentKind(s.gapHistoryArm,q.program,q.ordinaryGCode);
        if(q.firstReleased||s.session!=q.firstSession||!m_motion.RequestEDM234ParentRelease(s.session,s.armEpoch,s.lease,
            kind,s.run,s.cache,s.pc[0U],q.motionDispatch,m_edm130.recipe,q.program))
        {RejectEDM94SameThread("EDM234_PARENT_RELEASE_REQUEST",m_gapDryRun.sourceLine);return false;}
        q.releaseFloorTick=f.lastSentTick;q.releasePending=true;s.polls=0U;
        PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_FINISH parent=1 kind=%u session=%llu next=EXACT_ZERO_RETIREMENT discharge=0\n",q.program,kind,static_cast<unsigned long long>(s.session));
        return false;
    }
    if(!q.firstReleased||s.session<=q.firstSession)
    {RejectEDM94SameThread("EDM234_FIRST_PARENT_REQUIRED",m_gapDryRun.sourceLine);return false;}
    PrintNCG76DiagnosticLine("[EDM%u] event=SEQUENCE_COMPLETE parents=2 ordinaryRows=1 ordinaryG=%u G76children=25 firstSession=%llu secondSession=%llu history=CLEARED_AT_HANDOFF E5policy=FRESH_PER_PARENT discharge=0\n",
        q.program,q.ordinaryGCode,static_cast<unsigned long long>(q.firstSession),static_cast<unsigned long long>(s.session));
    return true;
}

bool NCManager::ConsumeEDM234ParentReleaseSameThread()
{
    if(m_edm234.program==236U)return ConsumeEDM236ParentReleaseSameThread();
    auto& q=m_edm234;auto& s=m_edm94;const auto& f=s.feedback;
    if(!q.releasePending)return false;
    const unsigned kind=E234ParentKind(q.arms[0U],q.program,q.ordinaryGCode);
    if(!IsEDM234SequenceScopeCurrentSameThread()||q.firstReleased||q.secondStarted||!q.firstRecipeBound||
        s.session!=q.firstSession||s.armPC!=q.firstArmPC||s.next!=2U||!s.latched||!s.complete||!s.terminalLogged||
        s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=q.firstArmPC+2||m_waitingBlockDispatchId!=q.firstFinishDispatch||
        f.faulted||f.cancelled||f.parentReleasePhase==EDM94::ParentReleasePhase::Revoked)
    {RejectEDM94SameThread("EDM234_PARENT_RELEASE_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(!f.parentReleased||f.parentReleasePhase!=EDM94::ParentReleasePhase::Released)return false;
    if(f.parentReleaseProgram!=q.program||f.parentReleaseKind!=kind||!f.released||f.active||!f.pass||!f.armAccepted||
        !f.finishRequested||!f.stopped||!f.forceZero||f.identity.session!=s.session||f.identity.epoch!=s.armEpoch||
        f.identity.ownerGeneration!=s.lease.generation||f.currentIdentity.session!=s.session||f.currentIdentity.epoch!=q.epoch||
        f.currentIdentity.ownerGeneration!=q.lease.generation||!f.parentReleaseTick||f.parentReleaseTick<=q.releaseFloorTick||
        f.parentReleaseTick!=f.lastTick||f.lastSentTick!=f.lastTick||f.requiredCycles<200U||f.settledCycles<f.requiredCycles||
        !E156FinishTargetMatches(s,m_edm130.recipe,f,m_edm114.retained)||!EDM130::SameRecipe(q.firstRecipe,m_edm130.recipe)||
        !m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||
        m_motion.GetAxisCommandMailboxDepth()||m_g76Orbit->active||m_pathFeed.pending||m_pathArc.pending)
    {RejectEDM94SameThread("EDM234_PARENT_RELEASE_RECEIPT",m_gapDryRun.sourceLine);return false;}
    q.releasePending=false;q.firstReleased=true;q.releaseTick=f.parentReleaseTick;
    PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_RELEASED parent=1 kind=%u session=%llu tick=%llu nextArmPC=%d sameAUTO=1 discharge=0\n",
        q.program,kind,static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(q.releaseTick),q.firstArmPC+3);
    s.latched=s.armPending=s.ready=s.finishPending=false;m_edm149Confirmation=EDM149Confirmation{};
    m_edm157=EDM157NCState{};m_edm200=EDM200AutoHoldState{};m_edm130.bound=false;
    return true;
}

bool NCManager::IsEDM236SequenceScopeCurrentSameThread() noexcept
{
    const auto& q=m_edm234;int first=-1;std::array<unsigned,4U> kinds{};
    if(!q.authorized||q.program!=236U||q.parentIndex>=4U||q.retiredMask!=((1U<<q.parentIndex)-1U)||
        !q.run||!q.cache||!q.confirmationDispatch||!q.startRequests||q.firstArmPC<=0||
        q.epoch==MOTION_EXECUTION_EPOCH_INVALID||!q.lease.IsValid()||q.lease.owner!=MotionOwner::AUTO||
        q.run!=m_pathCoreLiveBookkeeping.currentRunToken||q.cache!=GetBaseProgramCache().GetGeneration()||
        q.epoch!=m_motion.GetCurrentExecutionEpoch()||!q.lease.Matches(m_programMotionLease)||
        !m_motion.IsMotionOwnerLeaseCurrent(q.lease)||m_motion.HasPendingSafetyOrRecoveryRequests()||
        q.startRequests!=m_startDiagnosticRequests||q.recipeGeneration!=m_recipe.Current().generation||
        q.processGeneration!=m_edmProcessTuningGeneration||m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||
        Close_System_Com_flag||!m_externalReadyInterlock||m_isSingleBlockEnabled||m_isG66Active||
        !m_macroStack.empty()||Homing.IsActive()||AlarmManager::GetInstance().HasAlarm()||
        m_resetContinuationPhase!=ResetContinuationPhase::IDLE||!CoordSys.IsTranslationRunFrozen()||
        !IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot())||CoordSys.GetTranslationSnapshot().distanceMode!=91||
        !IsPathCoreLiveNativeConfigCurrentSameThread()||!IsFixedTranslationTravelCurrentSameThread()||
        !SameNCTranslationSnapshot(q.translation,CoordSys.GetTranslationSnapshot())||!m_motion.MatchesNCTranslation(q.translation)||
        !E236ProgramShape(GetBaseProgramCache(),first,kinds)||first!=q.firstArmPC||kinds!=q.kinds)return false;
    if(q.parentIndex==0U)
    {if(q.previousSession||q.awaitNext)return false;}
    else if(!q.firstRecipeBound||!q.firstSession||q.previousSession<q.firstSession||
        !q.firstFinishDispatch||!q.releaseTick||q.releaseTick<=q.previousReleaseFloorTick)return false;
    if(q.parentIndex>0U&&!q.stageStarted&&!q.awaitNext)return false;
    if(q.stageStarted&&(!q.stageRecipeBound||q.awaitNext))return false;
    if(q.awaitNext&&(!q.parentIndex||q.stageStarted))return false;
    if(q.releasePending&&(!q.stageStarted||!q.stageRecipeBound||q.parentIndex>=3U))return false;
    if(!q.stageRecipeBound)return !q.currentSession&&!q.stageStarted;
    if(!q.firstRecipeBound||!q.firstSession||!q.currentSession||
        (q.parentIndex?q.currentSession<=q.previousSession:q.currentSession!=q.firstSession)||
        !E236RecipeForKind(q.firstRecipe,q.kinds[0U])||!E236RecipeForKind(q.currentRecipe,q.kinds[q.parentIndex]))return false;
    auto expected=q.firstRecipe;expected.commissioningProfile=q.currentRecipe.commissioningProfile;
    expected.planRows=q.currentRecipe.planRows;expected.planMovementMask=q.currentRecipe.planMovementMask;
    return EDM130::SameRecipe(expected,q.currentRecipe);
}

bool NCManager::IsEDM236NextArmCurrentSameThread(int pc,unsigned arm) noexcept
{
    const auto& q=m_edm234;const auto& s=m_edm94;
    return IsEDM236SequenceScopeCurrentSameThread()&&q.parentIndex>0U&&q.parentIndex<4U&&q.awaitNext&&
        !q.stageRecipeBound&&!q.stageStarted&&!q.releasePending&&q.firstRecipeBound&&q.previousSession&&
        q.firstFinishDispatch&&q.motionDispatch&&!s.latched&&s.session==q.previousSession&&s.complete&&s.terminalLogged&&
        !s.cancelled&&!s.resetSeen&&!s.resetCommitted&&!s.releaseRequested&&!s.rtFaultReported&&
        pc==q.firstArmPC+static_cast<int>(q.parentIndex*3U)&&arm==E236ArmForKind(q.kinds[q.parentIndex])&&
        GetActiveDispatchPC()==pc&&m_currentExecutingBlockDispatchId>q.firstFinishDispatch&&
        m_motion.IsGroupDone()&&m_motion.IsGroupNCDrained()&&!m_motion.GetCommandIngressSize()&&
        !m_motion.GetCommandReplaySize()&&!m_motion.GetAxisCommandMailboxDepth()&&!m_g76Orbit->active&&
        !m_pathFeed.pending&&!m_pathArc.pending&&!m_pathReplay.pending&&!m_pathHold.armed&&!m_pathHold.bound;
}

bool NCManager::BeginEDM236SequenceSameThread(int pc) noexcept
{
    int first=-1;std::array<unsigned,4U> kinds{};
    if(!E236ProgramShape(GetBaseProgramCache(),first,kinds))return false;
    auto& q=m_edm234;
    if(q.authorized)
    {
        if(q.program!=236U||q.parentIndex>=4U)return false;
        if(q.parentIndex>0U)return IsEDM236NextArmCurrentSameThread(pc,E236ArmForKind(q.kinds[q.parentIndex]));
        return pc==first&&!q.firstRecipeBound&&!q.stageRecipeBound&&!q.stageStarted&&IsEDM236SequenceScopeCurrentSameThread();
    }
    const auto& c=m_edm149Confirmation;
    if(pc!=first||!c.confirmed||c.waiting||c.armPC!=pc||c.arm!=E236ArmForKind(kinds[0U])||
        c.run!=m_pathCoreLiveBookkeeping.currentRunToken||c.cache!=GetBaseProgramCache().GetGeneration()||
        !c.dispatch||m_currentExecutingBlockDispatchId<=c.dispatch||c.startRequests!=m_startDiagnosticRequests||
        !CoordSys.IsTranslationRunFrozen()||!m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))return false;
    q=EDM234NCState{};q.translation=CoordSys.GetTranslationSnapshot();q.lease=m_programMotionLease;
    q.epoch=m_motion.GetCurrentExecutionEpoch();q.run=c.run;q.cache=c.cache;q.confirmationDispatch=c.dispatch;
    q.startRequests=c.startRequests;q.recipeGeneration=m_recipe.Current().generation;
    q.processGeneration=m_edmProcessTuningGeneration;q.firstArmPC=first;q.kinds=kinds;q.program=236U;q.authorized=true;
    return IsEDM236SequenceScopeCurrentSameThread();
}

bool NCManager::PreflightEDM236OrdinarySameThread(int pc) noexcept
{
    int first=-1;std::array<unsigned,4U> kinds{};const auto& cache=GetBaseProgramCache();
    if(!E236ProgramShape(cache,first,kinds)||pc<first||(pc-first)%3||pc>first+9||
        kinds[static_cast<unsigned>(pc-first)/3U]==2U||
        !FreezeEDM173TranslationSameThread()||!BeginEDM236SequenceSameThread(pc))return false;
    const unsigned code=E236GCodeForKind(kinds[static_cast<unsigned>(pc-first)/3U]);
    m_edm94=EDM94NCState{};auto& s=m_edm94;
    s.program=true;s.inspectedCache=cache.GetGeneration();s.gapHistoryArm=274U;s.moves=1U;
    s.gapHistoryServo=s.gapHistoryMode=s.gapWorkTime=s.gapSegmentProfile=s.gapGainProfile=s.gapPhysicalCurve=true;
    s.gapModeFixture=EDM116::Fixture::ModalProgram156;s.gapTuning={8.0,1.0,1.0};
    if(!EDM107::ParseFlushProfile(10.0,50.0,100.0,s.gapFlushParameters))return false;
    s.pc[0U]=pc+1;s.pc[1U]=pc+2;s.pc[2U]=pc+3;s.feed[0U]=0.12;
    if(code==1U)s.increment[0U][2U]=-0.0015;
    else{s.increment[0U][0U]=code==2U?0.001:-0.001;s.increment[0U][1U]=0.001;}
    s.authoredGCode[0U]=static_cast<std::uint8_t>(code);s.selectorPC[0U]=pc+1;
    m_edm127=EDM127NCState{};m_edm129=EDM129NCState{};m_edm140=EDM140NCState{};m_edm141=EDM141NCState{};
    m_edm127.expectedMode[0U]=EDM116::Mode::B1;m_edm127.expectedOriginPC[0U]=pc+1;
    m_edm127.expectedOriginRevision[0U]=2ULL;m_edm127.authoredInline[0U]=true;
    return true;
}

bool NCManager::BindEDM236RecipeSameThread() noexcept
{
    m_edm130=EDM130NCState{};m_edm200=EDM200AutoHoldState{};
    const auto& s=m_edm94;auto& q=m_edm234;const auto& cache=GetBaseProgramCache();const auto* arm=cache.TryGetLine(s.armPC);
    if(!IsEDM236SequenceScopeCurrentSameThread()||q.stageRecipeBound||q.stageStarted||q.releasePending||
        !m_edmProcessReady||!arm||s.latched||s.armPC!=q.firstArmPC+static_cast<int>(q.parentIndex*3U)||
        s.gapHistoryArm!=E236ArmForKind(q.kinds[q.parentIndex])||s.run!=q.run||s.cache!=q.cache||
        s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||!s.session||
        (q.parentIndex?(!q.awaitNext||s.session<=q.previousSession):q.awaitNext))return false;
    const unsigned kind=q.kinds[q.parentIndex];EDM97::Tuning tuning{};EDM107::FlushProfile flush{};EDMRecipe::Summary summary{};
    if(!E234ArmSequence(arm->parsedBlock,s.gapHistoryArm)||!EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),236U)||
        !E108ProfileOnlyNPQRSIJK(arm->parsedBlock,s.gapHistoryArm,tuning,flush)||
        !EDM97::SameTuning(tuning,s.gapTuning)||!EDM107::SameFlushProfile(flush,s.gapFlushParameters))return false;
    if(q.parentIndex)
    {
        if(m_currentExecutingBlockDispatchId<=q.firstFinishDispatch||m_recipeRecoveryDisplayOnly||m_recipeHoldSeen||
            m_edmState==EDMState::NOT_READY||!m_recipe.Ready())return false;
        summary=m_recipe.Current();
        if(!summary.catalogReady||!summary.tableSelected||!summary.rowSelected||summary.generation!=q.recipeGeneration)return false;
    }
    else if(!ReadEDMRecipeSummarySameThread(summary))return false;
    const auto binding=kind==2U?
        EDM130::BindG76B2Gap175(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,m_edmProcessTuningGeneration):
        EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(m_recipe,m_edmProcessProfile.machineProfileId,
            m_edmProcessProfile,m_edmProcessTuningGeneration);
    if(!binding.ready)return false;
    auto recipe=binding.recipe;recipe.diagnosticControlSource=3U;recipe.planRows=kind==2U?25U:1U;
    recipe.planMovementMask=kind==2U?7U:kind==1U?4U:3U;
    if(!E236RecipeForKind(recipe,kind)||!EDM97::ValidMotionTestCurve(s.gapTuning,recipe))return false;
    if(q.parentIndex)
    {
        if(!q.firstRecipeBound||!E236RecipeForKind(q.firstRecipe,q.kinds[0U]))return false;
        auto expected=q.firstRecipe;expected.commissioningProfile=recipe.commissioningProfile;
        expected.planRows=recipe.planRows;expected.planMovementMask=recipe.planMovementMask;
        if(!EDM130::SameRecipe(expected,recipe))return false;
    }
    else{q.firstRecipe=recipe;q.firstRecipeBound=true;q.firstSession=s.session;}
    q.currentRecipe=recipe;q.currentSession=s.session;q.stageRecipeBound=true;q.motionDispatch=0ULL;
    if(!IsEDM236SequenceScopeCurrentSameThread())return false;
    m_edm130.recipe=recipe;m_edm94.gapFlushParameters.speedRatio=EDM130::FrozenSpeedRatio(recipe);
    if(!EDM130::MatchesFlushSpeed(recipe,m_edm94.gapFlushParameters.speedRatio)||
        !EDM130::MatchesFlushHeight(recipe,m_edm94.gapFlushParameters.strokeRatio,m_edm94.gapFlushParameters.dwellUs))return false;
    m_edm130.bound=true;
    PrintNCG76DiagnosticLine("[EDM236] event=PARENT_RECIPE_BOUND parent=%u kind=%u session=%llu firstSession=%llu previousSession=%llu generation=%llu retiredMask=%u E5policy=FRESH_PER_PARENT discharge=0\n",
        q.parentIndex+1U,kind,static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(q.firstSession),
        static_cast<unsigned long long>(q.previousSession),static_cast<unsigned long long>(summary.generation),q.retiredMask);
    return true;
}

bool NCManager::ConsumeEDM236ConfirmationSameThread(int armPC) noexcept
{
    auto& q=m_edm234;const auto& s=m_edm94;
    if(!IsEDM236SequenceScopeCurrentSameThread()||!q.stageRecipeBound||q.stageStarted||q.releasePending||
        !m_edm130.bound||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||s.latched||s.session!=q.currentSession||
        s.armPC!=armPC||armPC!=q.firstArmPC+static_cast<int>(q.parentIndex*3U)||
        s.gapHistoryArm!=E236ArmForKind(q.kinds[q.parentIndex])||s.run!=q.run||s.cache!=q.cache||
        s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||GetBasePC()!=armPC||GetActiveDispatchPC()!=armPC)return false;
    if(q.parentIndex)
    {
        if(!q.awaitNext||s.session<=q.previousSession||m_currentExecutingBlockDispatchId<=q.firstFinishDispatch)return false;
    }
    else
    {
        const auto& c=m_edm149Confirmation;
        if(!c.confirmed||c.waiting||c.run!=q.run||c.cache!=q.cache||c.armPC!=armPC||c.arm!=s.gapHistoryArm||
            c.dispatch!=q.confirmationDispatch||c.startRequests!=q.startRequests||
            m_currentExecutingBlockDispatchId<=c.dispatch)return false;
    }
    q.awaitNext=false;q.stageStarted=true;m_edm149Confirmation=EDM149Confirmation{};return true;
}

bool NCManager::BindEDM236ParentEpochSameThread(MotionExecutionEpoch epoch,std::uint64_t dispatch,int pc) noexcept
{
    auto& q=m_edm234;const auto& s=m_edm94;
    const auto expected=s.armEpoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:s.armEpoch+1U;
    if(!q.authorized||q.program!=236U||q.parentIndex>=4U||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending||
        !q.firstRecipeBound||q.motionDispatch||!s.latched||!s.ready||s.cancelled||s.resetSeen||s.resetCommitted||
        s.releaseRequested||s.rtFaultReported||s.session!=q.currentSession||
        s.armPC!=q.firstArmPC+static_cast<int>(q.parentIndex*3U)||s.gapHistoryArm!=E236ArmForKind(q.kinds[q.parentIndex])||
        s.run!=q.run||s.cache!=q.cache||!s.lease.Matches(q.lease)||q.epoch!=s.armEpoch||epoch!=expected||
        epoch!=m_motion.GetCurrentExecutionEpoch()||!dispatch||dispatch<=s.armDispatch||pc!=s.pc[0U]||
        !m_edm130.bound||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe))return false;
    q.epoch=epoch;if(!IsEDM236SequenceScopeCurrentSameThread())return false;q.motionDispatch=dispatch;
    PrintNCG76DiagnosticLine("[EDM236] event=PARENT_EPOCH_BOUND parent=%u kind=%u session=%llu armEpoch=%llu liveEpoch=%llu dispatch=%llu retiredMask=%u discharge=0\n",
        q.parentIndex+1U,q.kinds[q.parentIndex],static_cast<unsigned long long>(s.session),
        static_cast<unsigned long long>(s.armEpoch),static_cast<unsigned long long>(epoch),static_cast<unsigned long long>(dispatch),q.retiredMask);
    return true;
}

bool NCManager::FinishEDM236ParentSameThread()
{
    auto& q=m_edm234;auto& s=m_edm94;const auto& f=s.feedback;
    if(!IsEDM236SequenceScopeCurrentSameThread()||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending||
        !q.motionDispatch||!s.latched||!s.complete||!s.terminalLogged||s.session!=q.currentSession||
        s.armPC!=q.firstArmPC+static_cast<int>(q.parentIndex*3U)||s.gapHistoryArm!=E236ArmForKind(q.kinds[q.parentIndex])||
        s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=s.armPC+2||m_waitingBlockDispatchId!=q.currentFinishDispatch||
        q.currentFinishDispatch<=q.motionDispatch||!m_edm130.bound||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe))
    {RejectEDM94SameThread("EDM236_FINISHED_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(q.parentIndex<3U)
    {
        if(!m_motion.RequestEDM234ParentRelease(s.session,s.armEpoch,s.lease,q.kinds[q.parentIndex],s.run,s.cache,
            s.pc[0U],q.motionDispatch,q.currentRecipe,236U))
        {RejectEDM94SameThread("EDM236_PARENT_RELEASE_REQUEST",m_gapDryRun.sourceLine);return false;}
        q.releaseFloorTick=f.lastSentTick;q.releasePending=true;s.polls=0U;
        PrintNCG76DiagnosticLine("[EDM236] event=PARENT_FINISH parent=%u kind=%u session=%llu retiredMask=%u next=EXACT_ZERO_RETIREMENT discharge=0\n",
            q.parentIndex+1U,q.kinds[q.parentIndex],static_cast<unsigned long long>(s.session),q.retiredMask);
        return false;
    }
    PrintNCG76DiagnosticLine("[EDM236] event=SEQUENCE_COMPLETE parents=4 ordinaryRows=3 G76children=25 firstSession=%llu lastSession=%llu retiredMask=%u history=CLEARED_AT_EACH_HANDOFF E5policy=FRESH_PER_PARENT discharge=0\n",
        static_cast<unsigned long long>(q.firstSession),static_cast<unsigned long long>(q.currentSession),q.retiredMask);
    return true;
}

bool NCManager::ConsumeEDM236ParentReleaseSameThread()
{
    auto& q=m_edm234;auto& s=m_edm94;const auto& f=s.feedback;
    if(!q.releasePending)return false;
    if(!IsEDM236SequenceScopeCurrentSameThread()||q.parentIndex>=3U||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||
        !q.motionDispatch||s.session!=q.currentSession||s.armPC!=q.firstArmPC+static_cast<int>(q.parentIndex*3U)||
        s.gapHistoryArm!=E236ArmForKind(q.kinds[q.parentIndex])||s.next!=2U||!s.latched||!s.complete||!s.terminalLogged||
        s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=s.armPC+2||m_waitingBlockDispatchId!=q.currentFinishDispatch||
        f.faulted||f.cancelled||f.parentReleasePhase==EDM94::ParentReleasePhase::Revoked)
    {RejectEDM94SameThread("EDM236_PARENT_RELEASE_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(!f.parentReleased||f.parentReleasePhase!=EDM94::ParentReleasePhase::Released)return false;
    if(f.parentReleaseProgram!=236U||f.parentReleaseKind!=q.kinds[q.parentIndex]||!f.released||f.active||!f.pass||!f.armAccepted||
        !f.finishRequested||!f.stopped||!f.forceZero||f.identity.session!=s.session||f.identity.epoch!=s.armEpoch||
        f.identity.ownerGeneration!=s.lease.generation||f.currentIdentity.session!=s.session||f.currentIdentity.epoch!=q.epoch||
        f.currentIdentity.ownerGeneration!=q.lease.generation||!f.parentReleaseTick||f.parentReleaseTick<=q.releaseFloorTick||
        f.parentReleaseTick<=q.releaseTick||f.parentReleaseTick!=f.lastTick||f.lastSentTick!=f.lastTick||
        f.requiredCycles<200U||f.settledCycles<f.requiredCycles||
        !E156FinishTargetMatches(s,m_edm130.recipe,f,m_edm114.retained)||!m_edm130.bound||
        !EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||!m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||
        m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||m_motion.GetAxisCommandMailboxDepth()||
        m_g76Orbit->active||m_pathFeed.pending||m_pathArc.pending)
    {RejectEDM94SameThread("EDM236_PARENT_RELEASE_RECEIPT",m_gapDryRun.sourceLine);return false;}
    const unsigned retired=q.parentIndex;q.retiredMask|=1U<<retired;q.previousSession=q.currentSession;
    q.firstFinishDispatch=q.currentFinishDispatch;q.previousReleaseFloorTick=q.releaseFloorTick;
    q.releaseTick=f.parentReleaseTick;q.releasePending=false;
    ++q.parentIndex;q.currentSession=0ULL;q.currentRecipe=EDM130::Recipe{};
    q.stageRecipeBound=q.stageStarted=false;q.awaitNext=true;
    PrintNCG76DiagnosticLine("[EDM236] event=PARENT_RELEASED parent=%u kind=%u session=%llu tick=%llu retiredMask=%u nextArmPC=%d sameAUTO=1 discharge=0\n",
        retired+1U,q.kinds[retired],static_cast<unsigned long long>(q.previousSession),static_cast<unsigned long long>(q.releaseTick),
        q.retiredMask,q.firstArmPC+static_cast<int>(q.parentIndex*3U));
    s.latched=s.armPending=s.ready=s.finishPending=false;m_edm149Confirmation=EDM149Confirmation{};
    m_edm157=EDM157NCState{};m_edm200=EDM200AutoHoldState{};m_edm130.bound=false;
    return true;
}


// EDM237's continuation consent is NC-owned. The genuine ordinary GAP runtime
// still owns all four rows, one history bank, and the shared RT E5 clock.
// One stopped E6 edit belongs to the exact released parent. This grant never
// refreshes the generic recipe selection, opens motion, or changes its epoch.
bool NCManager::IsEDM240RecipeBridgeCurrentSameThread(int pc,bool afterApply) noexcept
{
    const auto& q=m_edm237;const auto& s=m_edm94;
    if(!E237BridgeRequired(q.program,q.parentIndex,q.kinds[0U])||q.parentIndex==0U||q.parentIndex>=4U||q.parentIndex>=q.parentCount||
        !IsEDM237SequenceScopeCurrentSameThread(false,afterApply)||!q.awaitNext||!q.firstReleased||
        q.stageRecipeBound||q.stageStarted||q.releasePending||!q.firstRecipeBound||!q.previousSession||
        !q.previousFinishDispatch||!q.motionDispatch||s.latched||s.armPending||s.ready||s.finishPending||
        s.session!=q.previousSession||!s.complete||!s.terminalLogged||s.cancelled||s.resetSeen||s.resetCommitted||
        s.releaseRequested||s.rtFaultReported||pc!=q.armPC[q.parentIndex]-1||GetBasePC()!=pc||GetActiveDispatchPC()!=pc||
        m_currentExecutingBlockDispatchId<=q.previousFinishDispatch||
        q.bridgeMask!=(E237RequiredBridgeMask(q.program,q.kinds[0U])&((1U<<(q.parentIndex-1U))-1U))||m_recipeControlExhausted||!m_recipeControlGeneration||
        m_recipeRecoveryDisplayOnly||m_recipeHoldSeen||!m_edmProcessReady||!m_recipe.Ready()||
        !IsEDMRecipeStoppedAuthorityCurrentSameThread()||m_motion.GetAxisCommandMailboxDepth()||
        m_g76Orbit->active||m_pathFeed.pending||m_pathArc.pending||m_pathReplay.pending||m_pathHold.armed||m_pathHold.bound)return false;
    // ServiceEDM94 stops sampling when the released session is unlatched;
    // retain and recheck its exact zero/200-cycle receipt before the edit.
    const auto& f=s.feedback;
    if(!f.parentReleased||f.parentReleasePhase!=EDM94::ParentReleasePhase::Released||f.parentReleaseProgram!=q.program||
        f.parentReleaseKind!=q.kinds[q.parentIndex-1U]||!f.released||f.active||!f.pass||!f.armAccepted||
        !f.finishRequested||!f.stopped||!f.forceZero||f.faulted||f.cancelled||
        f.identity.session!=q.previousSession||f.identity.epoch!=s.armEpoch||f.identity.ownerGeneration!=q.lease.generation||
        f.currentIdentity.session!=q.previousSession||f.currentIdentity.epoch!=q.epoch||f.currentIdentity.ownerGeneration!=q.lease.generation||
        f.parentReleaseTick!=q.releaseTick||f.lastTick!=q.releaseTick||f.lastSentTick!=q.releaseTick||
        f.requiredCycles<200U||f.settledCycles<f.requiredCycles)return false;
    const auto* source=GetBaseProgramCache().TryGetLine(pc);EDMRecipe::ParameterSnapshot height{};
    const unsigned heightKind=q.kinds[afterApply?q.parentIndex:q.parentIndex-1U];
    if(!source||!E240RecipeBridgeShape(source->parsedBlock,q.kinds[q.parentIndex],q.program)||
        !m_recipe.ReadField(6U,height)||height.actualValue!=static_cast<EDMRecipe::Value>((heightKind==2U||heightKind==5U)?1000ULL:200000ULL))return false;
    const auto& r=q.bridges[q.parentIndex-1U];
    if(r.committed||(afterApply&&!r.prepared))return false;
    if(!r.prepared)return !afterApply;
    const auto& p=m_recipeEditPrepared;auto expectedKey=r.beforeKey;expectedKey.generation=r.afterGeneration;
    return EDMRecipeControl::SameKey(expectedKey,r.afterKey)&&r.pc==pc&&r.line==source->sourceLineNumber&&r.dispatch==m_currentExecutingBlockDispatchId&&
        r.controlGeneration==m_recipeControlGeneration&&r.run==q.run&&r.cache==q.cache&&r.epoch==q.epoch&&
        r.lease.Matches(q.lease)&&r.releaseTick==q.releaseTick&&r.previousSession==q.previousSession&&
        r.finishDispatch==q.previousFinishDispatch&&r.beforeGeneration==q.recipeGeneration&&
        r.beforeGeneration<(std::numeric_limits<std::uint64_t>::max)()&&r.afterGeneration==r.beforeGeneration+1ULL&&
        p.sourcePC==pc&&p.line==r.line&&p.dispatch==r.dispatch&&p.request.requestId==r.dispatch&&
        p.request.fieldId==6U&&p.request.op==EDMRecipeControl::Operation::SetActual&&
        p.request.argument==static_cast<EDMRecipe::Value>((q.kinds[q.parentIndex]==2U||q.kinds[q.parentIndex]==5U)?1000ULL:200000ULL)&&
        p.context.run==q.run&&p.context.cache==q.cache&&p.context.epoch==q.epoch&&p.context.lease.Matches(q.lease)&&
        p.context.controlGeneration==m_recipeControlGeneration&&EDMRecipeControl::SameKey(p.context.key,r.beforeKey)&&
        EDMRecipeControl::SameKey(EDMRecipeControl::CaptureKey(m_recipe.Current()),afterApply?r.afterKey:r.beforeKey)&&
        r.previewRecipe.generation==r.afterGeneration&&E237RecipeForProgramKind(r.previewRecipe,q.kinds[q.parentIndex],q.program)&&
        E237SameParentRecipe(q.firstRecipe,r.previewRecipe,q.program);
}

bool NCManager::PrepareEDM240RecipeBridgeSameThread(const NCBlock& block,int pc,int line,NCBlockDispatchId dispatch)
{
    m_recipeEditPrepared.ready=false;auto& q=m_edm237;
    const auto reject=[&](const char* why){RejectEDM94SameThread(why,line);return false;};
    if(!IsEDM240RecipeBridgeCurrentSameThread(pc)||dispatch==NC_BLOCK_DISPATCH_ID_INVALID||
        dispatch!=m_currentExecutingBlockDispatchId)return reject("EDM240_BRIDGE_PREPARE_SCOPE");
    auto& receipt=q.bridges[q.parentIndex-1U];
    if(receipt.prepared||receipt.committed)return reject("EDM240_BRIDGE_REPLAY");
    const auto* source=GetBaseProgramCache().TryGetLine(pc);EDMRecipeControl::Request request{};
    if(!source||source->sourceLineNumber!=line||!DecodeEDMRecipeEditBlock(block,source->parsedBlock,request)||
        request.fieldId!=6U||request.op!=EDMRecipeControl::Operation::SetActual||
        request.argument!=static_cast<EDMRecipe::Value>((q.kinds[q.parentIndex]==2U||q.kinds[q.parentIndex]==5U)?1000ULL:200000ULL))
        return reject("EDM240_BRIDGE_SOURCE");
    request.requestId=dispatch;
    const auto key=EDMRecipeControl::CaptureKey(m_recipe.Current());
    if(!EDMRecipeControl::IsKeyValid(key)||key.generation!=q.recipeGeneration||
        key.generation==(std::numeric_limits<std::uint64_t>::max)())return reject("EDM240_BRIDGE_KEY");
    // The existing fixed NC-thread preview is discarded on every exit. No
    // controller-sized stack temporary or synthetic field remapping is used.
    auto& preview=m_edmRecipeSequencePreview;
    if(!preview.ForkPreviewFrom(m_recipe)){preview.DiscardPreview();return reject("EDM240_BRIDGE_PREVIEW_FORK");}
    EDMRecipeControl::Result result{};
    if(!EDMRecipeControl::Apply(preview,key,request,result)||!result.success||!result.applied||
        !result.afterFieldValid||result.afterField.actualValue!=request.argument||
        result.after.generation!=key.generation+1ULL)
    {preview.DiscardPreview();return reject("EDM240_BRIDGE_PREVIEW_EDIT");}
    const unsigned kind=q.kinds[q.parentIndex];
    const auto binding=kind==5U?
        EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(preview,m_edmProcessProfile.machineProfileId,
            m_edmProcessProfile,m_edmProcessTuningGeneration):kind==2U?
        EDM130::BindG76B2Gap175(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,m_edmProcessTuningGeneration):q.program==248U?
        EDM130::BindG76CenterSequence248(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B4Profile177):q.program==247U?
        EDM130::BindG76CenterSequence247(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B3Profile176):q.program==246U?
        EDM130::BindG76CenterSequence246(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B4Profile177):q.program==245U?
        EDM130::BindG76CenterSequence245(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B3Profile176):q.program==244U?
        EDM130::BindG76CenterSequence244(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==243U?
        EDM130::BindG76CenterSequence243(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==242U?
        EDM130::BindG76CenterSequence242(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==241U?
        EDM130::BindG76CenterSequence241(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):
        EDM130::BindG76CenterSequence240(preview,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177);
    auto recipe=binding.recipe;recipe.diagnosticControlSource=3U;recipe.planRows=kind==5U?4U:25U;recipe.planMovementMask=7U;
    const auto afterKey=EDMRecipeControl::CaptureKey(preview.Current());preview.DiscardPreview();
    if(!binding.ready||recipe.heightFieldId!=6U||recipe.generation!=key.generation+1ULL||
        !E237RecipeForProgramKind(recipe,kind,q.program)||!E237SameParentRecipe(q.firstRecipe,recipe,q.program)||
        !IsEDM240RecipeBridgeCurrentSameThread(pc)||!EDMRecipeControl::SameKey(key,EDMRecipeControl::CaptureKey(m_recipe.Current())))
        return reject("EDM240_BRIDGE_PREVIEW_IDENTITY");
    receipt=EDM237NCState::EDM240Bridge{};receipt.previewRecipe=recipe;receipt.beforeKey=key;receipt.afterKey=afterKey;
    receipt.lease=q.lease;receipt.epoch=q.epoch;receipt.run=q.run;receipt.cache=q.cache;receipt.dispatch=dispatch;
    receipt.controlGeneration=m_recipeControlGeneration;receipt.releaseTick=q.releaseTick;receipt.previousSession=q.previousSession;
    receipt.finishDispatch=q.previousFinishDispatch;receipt.beforeGeneration=key.generation;receipt.afterGeneration=afterKey.generation;
    receipt.pc=pc;receipt.line=line;receipt.prepared=true;
    auto& prepared=m_recipeEditPrepared;prepared.context.key=key;prepared.context.run=q.run;prepared.context.cache=q.cache;
    prepared.context.epoch=q.epoch;prepared.context.lease=q.lease;prepared.context.controlGeneration=m_recipeControlGeneration;
    prepared.request=request;prepared.sourcePC=pc;prepared.line=line;prepared.dispatch=dispatch;prepared.ready=true;
    if(IsEDM240RecipeBridgeCurrentSameThread(pc))return true;
    prepared.ready=false;receipt.prepared=false;return reject("EDM240_BRIDGE_PREPARE_REVOKED");
}

bool NCManager::AcceptEDM240RecipeBridgeSameThread(const EDMRecipeControl::Result& result,const EDM130::Recipe& recipe) noexcept
{
    auto& q=m_edm237;
    if(q.parentIndex==0U||q.parentIndex>=4U||!IsEDM240RecipeBridgeCurrentSameThread(q.armPC[q.parentIndex]-1,true))return false;
    auto& r=q.bridges[q.parentIndex-1U];
    if(!result.success||!result.applied||result.requestId!=r.dispatch||result.fieldId!=6U||
        result.op!=EDMRecipeControl::Operation::SetActual||!result.beforeFieldValid||!result.afterFieldValid||
        result.afterField.actualValue!=static_cast<EDMRecipe::Value>((q.kinds[q.parentIndex]==2U||q.kinds[q.parentIndex]==5U)?1000ULL:200000ULL)||
        !EDMRecipeControl::SameKey(EDMRecipeControl::CaptureKey(result.before),r.beforeKey)||
        !EDMRecipeControl::SameKey(EDMRecipeControl::CaptureKey(result.after),r.afterKey)||
        !EDM130::SameRecipe(recipe,r.previewRecipe)||!E237RecipeForProgramKind(recipe,q.kinds[q.parentIndex],q.program)||
        !E237SameParentRecipe(q.firstRecipe,recipe,q.program))return false;
    q.recipeGeneration=r.afterGeneration;r.prepared=false;r.committed=true;q.bridgeMask|=1U<<(q.parentIndex-1U);
    if(IsEDM237SequenceScopeCurrentSameThread())return true;
    // The live edit is never retried or rolled back. Revoke continuation if
    // concurrent safety intent arrived after the synchronous receipt proof.
    q.recipeGeneration=r.beforeGeneration;r.prepared=true;r.committed=false;q.bridgeMask&=~(1U<<(q.parentIndex-1U));return false;
}

bool NCManager::CommitEDM240RecipeBridgeSameThread(const NCBlock& block)
{
    auto& q=m_edm237;const int pc=GetActiveDispatchPC();const int line=m_recipeEditPrepared.line;
    const auto reject=[&](const char* why){m_recipeEditPrepared.ready=false;RejectEDM94SameThread(why,line);return false;};
    if(!m_recipeEditPrepared.ready||!IsEDM240RecipeBridgeCurrentSameThread(pc))return reject("EDM240_BRIDGE_COMMIT_SCOPE");
    auto& receipt=q.bridges[q.parentIndex-1U];const auto* source=GetBaseProgramCache().TryGetLine(pc);
    EDMRecipeControl::Request decoded{};
    if(!receipt.prepared||!source||!DecodeEDMRecipeEditBlock(block,source->parsedBlock,decoded)||
        decoded.fieldId!=m_recipeEditPrepared.request.fieldId||decoded.op!=m_recipeEditPrepared.request.op||
        decoded.argument!=m_recipeEditPrepared.request.argument)return reject("EDM240_BRIDGE_COMMIT_SOURCE");
    // Consume the one prepared token before the only live controller mutation.
    m_recipeEditPrepared.ready=false;EDMRecipeControl::Result result{};
    if(!EDMRecipeControl::Apply(m_recipe,receipt.beforeKey,m_recipeEditPrepared.request,result))
        return reject("EDM240_BRIDGE_LIVE_EDIT");
    const unsigned kind=q.kinds[q.parentIndex];
    const auto binding=kind==5U?
        EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(m_recipe,m_edmProcessProfile.machineProfileId,
            m_edmProcessProfile,m_edmProcessTuningGeneration):kind==2U?
        EDM130::BindG76B2Gap175(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,m_edmProcessTuningGeneration):q.program==248U?
        EDM130::BindG76CenterSequence248(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B4Profile177):q.program==247U?
        EDM130::BindG76CenterSequence247(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B3Profile176):q.program==246U?
        EDM130::BindG76CenterSequence246(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B4Profile177):q.program==245U?
        EDM130::BindG76CenterSequence245(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B3Profile176):q.program==244U?
        EDM130::BindG76CenterSequence244(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==243U?
        EDM130::BindG76CenterSequence243(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==242U?
        EDM130::BindG76CenterSequence242(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==241U?
        EDM130::BindG76CenterSequence241(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):
        EDM130::BindG76CenterSequence240(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177);
    auto recipe=binding.recipe;recipe.diagnosticControlSource=3U;recipe.planRows=kind==5U?4U:25U;recipe.planMovementMask=7U;
    if(!binding.ready||!AcceptEDM240RecipeBridgeSameThread(result,recipe))return reject("EDM240_BRIDGE_POST_COMMIT_SCOPE");
    PrintNCG76DiagnosticLine("[EDM%u] event=RECIPE_BRIDGE_COMMITTED parent=%u pc=%d dispatch=%llu beforeGeneration=%llu afterGeneration=%llu heightMicroMm=%llu bridgeMask=%u releaseTick=%llu motion=0 discharge=0\n",
        q.program,q.parentIndex+1U,pc,static_cast<unsigned long long>(receipt.dispatch),
        static_cast<unsigned long long>(receipt.beforeGeneration),static_cast<unsigned long long>(receipt.afterGeneration),
        static_cast<unsigned long long>(recipe.jumpHeightScaled),q.bridgeMask,static_cast<unsigned long long>(receipt.releaseTick));return true;
}

bool NCManager::IsEDM237SequenceScopeCurrentSameThread(bool nativeParentReceipt,bool recipeBridgeReceipt) noexcept
{
    const auto& q=m_edm237;int first=-1;std::array<unsigned,4U> kinds{};unsigned program=0U,count=0U;
    // EDM237 FIX2: a committed G76 native transaction publishes its own
    // next epoch before RT acknowledges EXECUTION_EPOCH_PUBLICATION_PENDING.
    // Only this call-local receipt may tolerate that acknowledgement window.
    // Every normal entry stays strict, and actual safety intent always wins.
    if(nativeParentReceipt)
    {
        if(!m_g76Orbit||!m_edm173||q.parentIndex>=q.parentCount||q.parentIndex>=4U||!E237NativeKind(q.kinds[q.parentIndex])||q.currentRowBound!=1U||
            !q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending)return false;
        const auto& g=*m_g76Orbit;const auto& r=g.receipt;const auto& scope=m_edm173->plan.scope;
        if(scope.session!=q.currentSession||scope.run!=q.run||scope.cache!=q.cache||
            scope.armEpoch!=q.currentArmEpoch||!scope.owner.Matches(q.lease)||
            scope.dispatch!=g.dispatch||scope.sourcePC!=g.pc||scope.line!=g.line)return false;
        if(g.run!=q.run||g.cache!=q.cache||g.dispatch!=q.motionDispatch||!g.dispatch||
            g.pc!=m_edm94.pc[0U]||!r.valid||!r.commandAccepted||!r.captureBound||!r.tailCommitted||
            r.count!=25U||r.axisMask!=7U||!r.ownerLease.Matches(q.lease)||!r.tail.IsCommitted()||
            !r.tail.identity.Matches(r.identities[24U])||!r.tail.ownerLease.Matches(q.lease)||
            r.tail.identity.source!=r.identities[24U].source||
            r.tail.identity.sourceBlockId!=r.identities[24U].sourceBlockId||
            !r.identities[0U].IsAssigned()||r.identities[0U].epoch!=q.epoch||
            r.identities[0U].source!=MotionCommandSource::NC_MEMORY||
            r.identities[0U].sourceBlockId!=static_cast<MotionSourceBlockId>(g.pc))return false;
    }
    // Only the synchronous post-Apply receipt may observe the one expected
    // recipe generation before adoption; all normal scope checks stay strict.
    if(recipeBridgeReceipt&&(nativeParentReceipt||!E237BridgeRequired(q.program,q.parentIndex,q.kinds[0U])||q.parentIndex==0U||q.parentIndex>=4U||
        !q.bridges[q.parentIndex-1U].prepared||q.bridges[q.parentIndex-1U].committed||
        q.bridges[q.parentIndex-1U].beforeGeneration!=q.recipeGeneration||
        q.recipeGeneration==(std::numeric_limits<std::uint64_t>::max)()||
        q.bridges[q.parentIndex-1U].afterGeneration!=q.recipeGeneration+1ULL))return false;
    if(!E240BridgeHistory(q))return false;
    if(!q.authorized||!q.run||!q.cache||!q.confirmationDispatch||!q.startRequests||q.firstArmPC<1||q.parentIndex>=q.parentCount||q.parentIndex>=4U||
        q.epoch==MOTION_EXECUTION_EPOCH_INVALID||!q.lease.IsValid()||q.lease.owner!=MotionOwner::AUTO||
        q.run!=m_pathCoreLiveBookkeeping.currentRunToken||q.cache!=GetBaseProgramCache().GetGeneration()||
        q.epoch!=m_motion.GetCurrentExecutionEpoch()||!q.lease.Matches(m_programMotionLease)||
        !m_motion.IsMotionOwnerLeaseCurrent(q.lease)||
        (nativeParentReceipt?m_motion.HasPendingSafetyIntent():m_motion.HasPendingSafetyOrRecoveryRequests())||
        q.startRequests!=m_startDiagnosticRequests||(recipeBridgeReceipt?q.recipeGeneration+1ULL:q.recipeGeneration)!=m_recipe.Current().generation||
        q.processGeneration!=m_edmProcessTuningGeneration||m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||
        Close_System_Com_flag||!m_externalReadyInterlock||m_isSingleBlockEnabled||m_isG66Active||
        !m_macroStack.empty()||Homing.IsActive()||AlarmManager::GetInstance().HasAlarm()||
        m_resetContinuationPhase!=ResetContinuationPhase::IDLE||!CoordSys.IsTranslationRunFrozen()||
        !IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot())||CoordSys.GetTranslationSnapshot().distanceMode!=91||
        !IsPathCoreLiveNativeConfigCurrentSameThread()||!IsFixedTranslationTravelCurrentSameThread()||
        !SameNCTranslationSnapshot(q.translation,CoordSys.GetTranslationSnapshot())||!m_motion.MatchesNCTranslation(q.translation)||
        !E237HistoryProgramShape(GetBaseProgramCache(),first,kinds,program,count)||first!=q.firstArmPC||kinds!=q.kinds||program!=q.program||count!=q.parentCount||
        q.armPC[0U]!=first||q.armPC[1U]!=first+static_cast<int>(E237ArmStride(program,kinds[0U],1U,kinds[0U]))||
        q.currentRowBound>E237RowsForKind(q.kinds[q.parentIndex])||
        q.retiredMask!=(1U<<q.parentIndex)-1U)return false;
    for(unsigned i=1U;i<q.parentCount;++i)
        if(q.armPC[i]!=q.armPC[i-1U]+static_cast<int>(E237ArmStride(q.program,q.kinds[i-1U],i,q.kinds[0U])))return false;
    if(q.parentIndex==0U)
    {if(q.firstReleased||q.awaitNext||q.previousSession||q.releaseTick||q.previousFinishDispatch||q.previousReleaseFloorTick)return false;}
    else if(!q.firstReleased||!q.firstRecipeBound||!q.firstSession||q.previousSession<q.firstSession||
        !q.firstFinishDispatch||q.previousFinishDispatch<q.firstFinishDispatch||
        (q.parentIndex==1U?(q.previousSession!=q.firstSession||q.previousFinishDispatch!=q.firstFinishDispatch):
            (q.previousSession<=q.firstSession||q.previousFinishDispatch<=q.firstFinishDispatch))||
        !q.releaseTick||q.releaseTick<=q.previousReleaseFloorTick)return false;
    if(q.awaitNext&&(!q.parentIndex||q.stageStarted||q.releasePending))return false;
    if(q.stageStarted&&(!q.stageRecipeBound||q.awaitNext))return false;
    if(q.releasePending&&(q.parentIndex+1U>=q.parentCount||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||
        (q.parentIndex&&q.releaseFloorTick<q.releaseTick)))return false;
    if(!q.stageRecipeBound)return !q.currentSession&&!q.stageStarted&&!q.currentRowBound;
    if(!q.firstRecipeBound||!q.firstSession||!q.currentSession||
        (q.parentIndex?q.currentSession<=q.previousSession:q.currentSession!=q.firstSession)||
        !E237RecipeForProgramKind(q.firstRecipe,q.kinds[0U],q.program)||!E237RecipeForProgramKind(q.currentRecipe,q.kinds[q.parentIndex],q.program))return false;
    return E237SameParentRecipe(q.firstRecipe,q.currentRecipe,q.program);
}

bool NCManager::IsEDM237NextArmCurrentSameThread(int pc,unsigned arm) noexcept
{
    const auto& q=m_edm237;const auto& s=m_edm94;
    return IsEDM237SequenceScopeCurrentSameThread()&&q.parentIndex>0U&&q.parentIndex<q.parentCount&&q.awaitNext&&q.firstReleased&&
        !q.stageRecipeBound&&!q.stageStarted&&!q.releasePending&&q.firstRecipeBound&&q.previousSession&&
        q.previousFinishDispatch&&q.motionDispatch&&!s.latched&&s.session==q.previousSession&&s.complete&&s.terminalLogged&&
        !s.cancelled&&!s.resetSeen&&!s.resetCommitted&&!s.releaseRequested&&!s.rtFaultReported&&
        pc==q.armPC[q.parentIndex]&&arm==E237ArmForKind(q.kinds[q.parentIndex])&&GetActiveDispatchPC()==pc&&
        m_currentExecutingBlockDispatchId>q.previousFinishDispatch&&
        (!E237RequiredBridgeMask(q.program,q.kinds[0U])||(q.bridgeMask==(E237RequiredBridgeMask(q.program,q.kinds[0U])&((1U<<q.parentIndex)-1U))&&
            (!E237BridgeRequired(q.program,q.parentIndex,q.kinds[0U])||m_currentExecutingBlockDispatchId>q.bridges[q.parentIndex-1U].dispatch)))&&m_motion.IsGroupDone()&&m_motion.IsGroupNCDrained()&&
        !m_motion.GetCommandIngressSize()&&!m_motion.GetCommandReplaySize()&&!m_motion.GetAxisCommandMailboxDepth()&&
        !m_g76Orbit->active&&!m_pathFeed.pending&&!m_pathArc.pending&&!m_pathReplay.pending&&!m_pathHold.armed&&!m_pathHold.bound;
}

bool NCManager::BeginEDM237SequenceSameThread(int pc) noexcept
{
    int first=-1;std::array<unsigned,4U> kinds{};unsigned program=0U,count=0U;
    if(!E237HistoryProgramShape(GetBaseProgramCache(),first,kinds,program,count))return false;
    auto& q=m_edm237;
    if(q.authorized)
    {
        if(q.parentIndex>=q.parentCount||q.parentIndex>=4U)return false;
        if(q.parentIndex)return IsEDM237NextArmCurrentSameThread(pc,E237ArmForKind(q.kinds[q.parentIndex]));
        return pc==first&&!q.firstRecipeBound&&!q.stageRecipeBound&&!q.stageStarted&&IsEDM237SequenceScopeCurrentSameThread();
    }
    const auto& c=m_edm149Confirmation;
    if(m_edm233.authorized||m_edm234.authorized||pc!=first||!c.confirmed||c.waiting||c.armPC!=pc||
        c.arm!=E237ArmForKind(kinds[0U])||c.run!=m_pathCoreLiveBookkeeping.currentRunToken||
        c.cache!=GetBaseProgramCache().GetGeneration()||!c.dispatch||m_currentExecutingBlockDispatchId<=c.dispatch||
        c.startRequests!=m_startDiagnosticRequests||!CoordSys.IsTranslationRunFrozen()||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))return false;
    q=EDM237NCState{};q.translation=CoordSys.GetTranslationSnapshot();q.lease=m_programMotionLease;
    q.epoch=m_motion.GetCurrentExecutionEpoch();q.run=c.run;q.cache=c.cache;q.confirmationDispatch=c.dispatch;
    q.startRequests=c.startRequests;q.recipeGeneration=m_recipe.Current().generation;
    if(E237RequiredBridgeMask(program,kinds[0U])&&q.recipeGeneration>(std::numeric_limits<std::uint64_t>::max)()-((program==242U||program==245U||program==246U||program==247U||program==248U)?1ULL:(program==243U||program==244U)?2ULL:3ULL))return false;
    q.processGeneration=m_edmProcessTuningGeneration;q.firstArmPC=first;q.kinds=kinds;
    q.program=program;q.parentCount=count;q.armPC[0U]=first;
    for(unsigned i=1U;i<count;++i)q.armPC[i]=q.armPC[i-1U]+static_cast<int>(E237ArmStride(program,kinds[i-1U],i,kinds[0U]));
    q.authorized=true;
    return IsEDM237SequenceScopeCurrentSameThread();
}

bool NCManager::PreflightEDM237OrdinarySameThread(int pc) noexcept
{
    int first=-1;std::array<unsigned,4U> kinds{};unsigned program=0U,count=0U;const auto& cache=GetBaseProgramCache();
    if(!E237HistoryProgramShape(cache,first,kinds,program,count))return false;
    int candidate=first;bool ordinary=false;
    for(unsigned i=0U;i<count;++i){if(pc==candidate&&kinds[i]==5U)ordinary=true;
        candidate+=static_cast<int>(E237ArmStride(program,kinds[i],i+1U,kinds[0U]));}
    if(!ordinary||!FreezeEDM173TranslationSameThread()||!BeginEDM237SequenceSameThread(pc))return false;
    m_edm94=EDM94NCState{};auto& s=m_edm94;
    s.program=true;s.inspectedCache=cache.GetGeneration();s.gapHistoryArm=274U;s.moves=4U;
    s.gapHistoryServo=s.gapHistoryMode=s.gapWorkTime=s.gapSegmentProfile=s.gapGainProfile=s.gapPhysicalCurve=true;
    s.gapModeFixture=EDM116::Fixture::ModalProgram156;s.gapTuning={8.0,1.0,1.0};
    if(!EDM107::ParseFlushProfile(10.0,50.0,100.0,s.gapFlushParameters))return false;
    m_edm127=EDM127NCState{};m_edm129=EDM129NCState{};m_edm140=EDM140NCState{};m_edm141=EDM141NCState{};
    const unsigned codes[4]={1U,2U,3U,1U};
    const EDM116::Mode modes[4]={EDM116::Mode::B0,EDM116::Mode::B1,EDM116::Mode::B2,EDM116::Mode::B2};
    for(unsigned row=0U;row<4U;++row)
    {
        s.pc[row]=pc+1+static_cast<int>(row);s.feed[row]=0.12;s.authoredGCode[row]=static_cast<std::uint8_t>(codes[row]);
        s.selectorPC[row]=s.pc[row];
        if(row==0U||row==3U)s.increment[row][2U]=-0.0015;
        else{s.increment[row][0U]=row==1U?0.001:-0.001;s.increment[row][1U]=0.001;}
        m_edm127.expectedMode[row]=modes[row];m_edm127.expectedOriginPC[row]=pc+1+static_cast<int>((std::min)(row,2U));
        m_edm127.expectedOriginRevision[row]=2ULL+(std::min)(row,2U);m_edm127.authoredInline[row]=row!=3U;
    }
    s.pc[4U]=pc+5;s.pc[5U]=pc+6;return true;
}

bool NCManager::BindEDM237RecipeSameThread() noexcept
{
    m_edm130=EDM130NCState{};m_edm200=EDM200AutoHoldState{};
    const auto& s=m_edm94;auto& q=m_edm237;const auto& cache=GetBaseProgramCache();const auto* arm=cache.TryGetLine(s.armPC);
    if(!IsEDM237SequenceScopeCurrentSameThread()||q.stageRecipeBound||q.stageStarted||q.releasePending||
        !m_edmProcessReady||!arm||s.latched||s.armPC!=q.armPC[q.parentIndex]||
        s.gapHistoryArm!=E237ArmForKind(q.kinds[q.parentIndex])||s.moves!=E237RowsForKind(q.kinds[q.parentIndex])||
        s.run!=q.run||s.cache!=q.cache||s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||!s.session||
        (q.parentIndex?(!q.awaitNext||s.session<=q.previousSession):q.awaitNext))return false;
    const unsigned kind=q.kinds[q.parentIndex];EDM97::Tuning tuning{};EDM107::FlushProfile flush{};EDMRecipe::Summary summary{};
    if(!E237ArmSequence(arm->parsedBlock,s.gapHistoryArm)||
        !E108ProfileOnlyNPQRSIJK(arm->parsedBlock,s.gapHistoryArm,tuning,flush)||
        !EDM97::SameTuning(tuning,s.gapTuning)||!EDM107::SameFlushProfile(flush,s.gapFlushParameters))return false;
    if(q.parentIndex)
    {
        if(m_currentExecutingBlockDispatchId<=q.previousFinishDispatch||m_recipeRecoveryDisplayOnly||m_recipeHoldSeen||
            m_edmState==EDMState::NOT_READY||!m_recipe.Ready())return false;
        summary=m_recipe.Current();
        if(!summary.catalogReady||!summary.tableSelected||!summary.rowSelected||summary.generation!=q.recipeGeneration)return false;
    }
    else if(!ReadEDMRecipeSummarySameThread(summary))return false;
    const auto binding=q.program==248U&&kind==7U?
        EDM130::BindG76CenterSequence248(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B4Profile177):q.program==247U&&kind==6U?
        EDM130::BindG76CenterSequence247(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B3Profile176):q.program==246U&&kind==7U?
        EDM130::BindG76CenterSequence246(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B4Profile177):q.program==245U&&kind==6U?
        EDM130::BindG76CenterSequence245(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,EDM130::G76B3Profile176):q.program==244U&&(kind==6U||kind==7U)?
        EDM130::BindG76CenterSequence244(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==243U&&(kind==6U||kind==7U)?
        EDM130::BindG76CenterSequence243(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==242U&&(kind==6U||kind==7U)?
        EDM130::BindG76CenterSequence242(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==241U&&(kind==6U||kind==7U)?
        EDM130::BindG76CenterSequence241(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):q.program==240U&&(kind==6U||kind==7U)?
        EDM130::BindG76CenterSequence240(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):(kind==6U||kind==7U)?
        EDM130::BindG76CenterSequence239(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,
            m_edmProcessTuningGeneration,kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177):kind==2U?
        EDM130::BindG76B2Gap175(m_recipe,m_edmProcessProfile.machineProfileId,m_edmProcessProfile,m_edmProcessTuningGeneration):
        EDM130::BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(m_recipe,m_edmProcessProfile.machineProfileId,
            m_edmProcessProfile,m_edmProcessTuningGeneration);
    if(!binding.ready)return false;
    auto recipe=binding.recipe;recipe.diagnosticControlSource=3U;recipe.planRows=E237NativeKind(kind)?25U:4U;recipe.planMovementMask=7U;
    if(!E237RecipeForProgramKind(recipe,kind,q.program)||!EDM97::ValidMotionTestCurve(s.gapTuning,recipe))return false;
    if(q.parentIndex)
    {
        if(!q.firstRecipeBound||!E237RecipeForProgramKind(q.firstRecipe,q.kinds[0U],q.program))return false;
        if(!E237SameParentRecipe(q.firstRecipe,recipe,q.program))return false;
    }
    else{q.firstRecipe=recipe;q.firstRecipeBound=true;q.firstSession=s.session;}
    q.currentRecipe=recipe;q.currentSession=s.session;q.stageRecipeBound=true;q.currentArmEpoch=s.armEpoch;
    q.motionDispatch=q.currentFinishDispatch=0ULL;q.currentRowBound=0U;
    if(!IsEDM237SequenceScopeCurrentSameThread())return false;
    m_edm130.recipe=recipe;m_edm94.gapFlushParameters.speedRatio=EDM130::FrozenSpeedRatio(recipe);
    if(!EDM130::MatchesFlushSpeed(recipe,m_edm94.gapFlushParameters.speedRatio)||
        !EDM130::MatchesFlushHeight(recipe,m_edm94.gapFlushParameters.strokeRatio,m_edm94.gapFlushParameters.dwellUs))return false;
    m_edm130.bound=true;
    PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_RECIPE_BOUND parent=%u kind=%u session=%llu rows=%u history=SAME_PARENT E5policy=SHARED_ROWS_FRESH_PARENT discharge=0 recipeGeneration=%llu bridgeMask=%u\n",
        q.program,q.parentIndex+1U,kind,static_cast<unsigned long long>(s.session),recipe.planRows,
        static_cast<unsigned long long>(recipe.generation),q.bridgeMask);
    return true;
}

bool NCManager::ConsumeEDM237ConfirmationSameThread(int armPC) noexcept
{
    auto& q=m_edm237;const auto& s=m_edm94;
    if(!IsEDM237SequenceScopeCurrentSameThread()||!q.stageRecipeBound||q.stageStarted||q.releasePending||
        !m_edm130.bound||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||s.latched||s.session!=q.currentSession||
        s.armPC!=armPC||armPC!=q.armPC[q.parentIndex]||s.gapHistoryArm!=E237ArmForKind(q.kinds[q.parentIndex])||
        s.run!=q.run||s.cache!=q.cache||s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||
        GetBasePC()!=armPC||GetActiveDispatchPC()!=armPC)return false;
    if(q.parentIndex)
    {if(!q.awaitNext||!q.firstReleased||s.session<=q.previousSession||m_currentExecutingBlockDispatchId<=q.previousFinishDispatch)return false;}
    else
    {
        const auto& c=m_edm149Confirmation;
        if(!c.confirmed||c.waiting||c.run!=q.run||c.cache!=q.cache||c.armPC!=armPC||c.arm!=s.gapHistoryArm||
            c.dispatch!=q.confirmationDispatch||c.startRequests!=q.startRequests||m_currentExecutingBlockDispatchId<=c.dispatch)return false;
    }
    q.awaitNext=false;q.stageStarted=true;m_edm149Confirmation=EDM149Confirmation{};return true;
}

bool NCManager::BindEDM237ParentEpochSameThread(MotionExecutionEpoch epoch,std::uint64_t dispatch,int pc) noexcept
{
    auto& q=m_edm237;const auto& s=m_edm94;
    const unsigned rows=q.parentIndex<q.parentCount&&q.parentIndex<4U?E237RowsForKind(q.kinds[q.parentIndex]):0U;
    const auto expected=q.epoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:q.epoch+1U;
    if(!q.authorized||q.parentIndex>=q.parentCount||q.parentIndex>=4U||!rows||q.currentRowBound>=rows||!q.stageRecipeBound||!q.stageStarted||
        q.awaitNext||q.releasePending||!q.firstRecipeBound||!s.latched||!s.ready||s.cancelled||s.resetSeen||
        s.resetCommitted||s.releaseRequested||s.rtFaultReported||s.session!=q.currentSession||
        s.armPC!=q.armPC[q.parentIndex]||s.gapHistoryArm!=E237ArmForKind(q.kinds[q.parentIndex])||
        s.run!=q.run||s.cache!=q.cache||!s.lease.Matches(q.lease)||s.armEpoch!=q.currentArmEpoch||epoch!=expected||
        epoch!=m_motion.GetCurrentExecutionEpoch()||!dispatch||dispatch<=s.armDispatch||
        (q.currentRowBound?dispatch<=q.motionDispatch:q.epoch!=s.armEpoch)||
        s.next!=q.currentRowBound+1U||pc!=s.pc[q.currentRowBound]||
        !m_edm130.bound||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe))return false;
    q.epoch=epoch;q.motionDispatch=dispatch;++q.currentRowBound;
    // The G76 caller has verified all 25 identities and exact native tail.
    // Ordinary row receipts and all other scope checks retain the strict gate.
    const bool nativeParentReceipt=E237NativeKind(q.kinds[q.parentIndex]);
    if(!IsEDM237SequenceScopeCurrentSameThread(nativeParentReceipt))return false;
    if(nativeParentReceipt)
        PrintNCG76DiagnosticLine("[EDM237_FIX2] event=NATIVE_EPOCH_RECEIPT session=%llu armEpoch=%llu acceptedEpoch=%llu dispatch=%llu children=25 scope=EXACT_COMMITTED_RECEIPT safetyGate=NO_PENDING_SAFETY aggregateCommit=STILL_REQUIRED discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.armEpoch),
            static_cast<unsigned long long>(epoch),static_cast<unsigned long long>(dispatch));
    PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_EPOCH_BOUND parent=%u kind=%u row=%u session=%llu armEpoch=%llu liveEpoch=%llu dispatch=%llu sameHistory=1 sameE5=1 discharge=0\n",
        q.program,q.parentIndex+1U,q.kinds[q.parentIndex],q.currentRowBound,static_cast<unsigned long long>(s.session),
        static_cast<unsigned long long>(s.armEpoch),static_cast<unsigned long long>(epoch),static_cast<unsigned long long>(dispatch));
    return true;
}

bool NCManager::FinishEDM237ParentSameThread()
{
    auto& q=m_edm237;auto& s=m_edm94;const auto& f=s.feedback;
    const unsigned rows=q.parentIndex<q.parentCount&&q.parentIndex<4U?E237RowsForKind(q.kinds[q.parentIndex]):0U;
    if(!IsEDM237SequenceScopeCurrentSameThread()||!rows||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending||
        !q.motionDispatch||q.currentRowBound!=rows||!s.latched||!s.complete||!s.terminalLogged||s.session!=q.currentSession||
        s.armPC!=q.armPC[q.parentIndex]||s.gapHistoryArm!=E237ArmForKind(q.kinds[q.parentIndex])||s.next!=rows+1U||
        s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=s.armPC+static_cast<int>(rows)+1||m_waitingBlockDispatchId!=q.currentFinishDispatch||
        q.currentFinishDispatch<=q.motionDispatch||!m_edm130.bound||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe))
    {RejectEDM94SameThread("EDM237_FINISHED_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(q.parentIndex+1U<q.parentCount)
    {
        if(!m_motion.RequestEDM234ParentRelease(s.session,s.armEpoch,s.lease,q.kinds[q.parentIndex],s.run,s.cache,
            s.pc[rows-1U],q.motionDispatch,q.currentRecipe,q.program))
        {RejectEDM94SameThread("EDM237_PARENT_RELEASE_REQUEST",m_gapDryRun.sourceLine);return false;}
        q.releaseFloorTick=f.lastSentTick;q.releasePending=true;s.polls=0U;
        PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_FINISH parent=%u kind=%u session=%llu next=EXACT_ZERO_200_RETIREMENT discharge=0\n",
            q.program,q.parentIndex+1U,q.kinds[q.parentIndex],static_cast<unsigned long long>(s.session));return false;
    }
    if(!IsEDM237FinalStateSameThread())
    {RejectEDM94SameThread("EDM237_FINAL_STATE",m_gapDryRun.sourceLine);return false;}
    PrintNCG76DiagnosticLine("[EDM%u] event=SEQUENCE_COMPLETE parents=%u ordinaryRows=%u G76children=%u retiredMask=%u bridgeMask=%u firstSession=%llu lastSession=%llu history=SHARED_WITHIN_ORDINARY_CLEARED_AT_HANDOFF E5policy=SHARED_ROWS_FRESH_PARENT discharge=0\n",
        q.program,q.parentCount,(q.program==241U||q.program==242U||q.program==243U||q.program==244U||q.program==247U||q.program==248U)?4U:q.program==239U?0U:q.parentCount*2U,
        ((q.program==241U||q.program==242U||q.program==243U||q.program==244U||q.program==247U||q.program==248U)?3U:q.program==239U?q.parentCount:q.parentCount/2U)*25U,q.retiredMask,q.bridgeMask,
        static_cast<unsigned long long>(q.firstSession),static_cast<unsigned long long>(q.currentSession));return true;
}

bool NCManager::ConsumeEDM237ParentReleaseSameThread()
{
    auto& q=m_edm237;auto& s=m_edm94;const auto& f=s.feedback;
    if(!q.releasePending)return false;
    const unsigned rows=q.parentIndex<4U?E237RowsForKind(q.kinds[q.parentIndex]):0U;
    if(!IsEDM237SequenceScopeCurrentSameThread()||q.parentIndex+1U>=q.parentCount||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||
        !q.motionDispatch||q.currentRowBound!=rows||s.session!=q.currentSession||
        s.armPC!=q.armPC[q.parentIndex]||s.next!=rows+1U||!s.latched||!s.complete||!s.terminalLogged||
        s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=q.armPC[q.parentIndex]+static_cast<int>(rows)+1||m_waitingBlockDispatchId!=q.currentFinishDispatch||
        f.faulted||f.cancelled||f.parentReleasePhase==EDM94::ParentReleasePhase::Revoked)
    {RejectEDM94SameThread("EDM237_PARENT_RELEASE_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(!f.parentReleased||f.parentReleasePhase!=EDM94::ParentReleasePhase::Released)return false;
    if(f.parentReleaseProgram!=q.program||f.parentReleaseKind!=q.kinds[q.parentIndex]||!f.released||f.active||!f.pass||!f.armAccepted||
        !f.finishRequested||!f.stopped||!f.forceZero||f.identity.session!=s.session||f.identity.epoch!=s.armEpoch||
        f.identity.ownerGeneration!=s.lease.generation||f.currentIdentity.session!=s.session||f.currentIdentity.epoch!=q.epoch||
        f.currentIdentity.ownerGeneration!=q.lease.generation||!f.parentReleaseTick||f.parentReleaseTick<=q.releaseFloorTick||
        f.parentReleaseTick!=f.lastTick||f.lastSentTick!=f.lastTick||f.requiredCycles<200U||f.settledCycles<f.requiredCycles||
        !E156FinishTargetMatches(s,m_edm130.recipe,f,m_edm114.retained)||!m_edm130.bound||
        !EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||!m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||
        m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||m_motion.GetAxisCommandMailboxDepth()||
        m_g76Orbit->active||m_pathFeed.pending||m_pathArc.pending)
    {RejectEDM94SameThread("EDM237_PARENT_RELEASE_RECEIPT",m_gapDryRun.sourceLine);return false;}
    const unsigned retiredParent=q.parentIndex;
    q.firstReleased=true;q.previousSession=q.currentSession;q.previousFinishDispatch=q.currentFinishDispatch;
    q.previousReleaseFloorTick=q.releaseFloorTick;q.releaseTick=f.parentReleaseTick;q.releasePending=false;
    q.retiredMask|=1U<<retiredParent;++q.parentIndex;
    q.currentSession=0ULL;q.currentRecipe=EDM130::Recipe{};q.currentRowBound=0U;q.b2TraceTick=0ULL;
    q.stageRecipeBound=q.stageStarted=false;q.awaitNext=true;
    PrintNCG76DiagnosticLine("[EDM%u] event=PARENT_RELEASED parent=%u kind=%u session=%llu tick=%llu nextArmPC=%d retiredMask=%u sameAUTO=1 freshSessionRequired=1 discharge=0\n",
        q.program,retiredParent+1U,q.kinds[retiredParent],static_cast<unsigned long long>(q.previousSession),
        static_cast<unsigned long long>(q.releaseTick),q.armPC[q.parentIndex],q.retiredMask);
    s.latched=s.armPending=s.ready=s.finishPending=false;m_edm149Confirmation=EDM149Confirmation{};
    m_edm157=EDM157NCState{};m_edm200=EDM200AutoHoldState{};m_edm130.bound=false;return true;
}

// Structural final-parent evidence remains valid after M30 commits owner NONE
// and a newer completion epoch; never call the RUN/live-scope gate here.
bool NCManager::IsEDM237FinalStateSameThread() const noexcept
{
    const auto& q=m_edm237;const auto& s=m_edm94;int first=-1;std::array<unsigned,4U> kinds{};unsigned program=0U,count=0U;
    if(!E240BridgeHistory(q)||(E237RequiredBridgeMask(q.program,q.kinds[0U])&&q.bridgeMask!=E237RequiredBridgeMask(q.program,q.kinds[0U])))return false;
    if(!q.authorized||(q.parentCount!=2U&&q.parentCount!=4U)||q.parentIndex+1U!=q.parentCount||
        q.retiredMask!=(1U<<q.parentIndex)-1U||!q.firstReleased||q.releasePending||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||
        !q.firstRecipeBound||!q.firstSession||q.previousSession<q.firstSession||s.session!=q.currentSession||
        q.currentSession<=q.previousSession||!q.releaseTick||q.releaseTick<=q.previousReleaseFloorTick||
        !q.firstFinishDispatch||q.previousFinishDispatch<q.firstFinishDispatch||!q.motionDispatch||q.currentFinishDispatch<=q.motionDispatch||
        q.currentRowBound!=E237RowsForKind(q.kinds[q.parentIndex])||s.moves!=E237RowsForKind(q.kinds[q.parentIndex])||s.armEpoch!=q.currentArmEpoch||
        s.armPC!=q.armPC[q.parentIndex]||s.gapHistoryArm!=E237ArmForKind(q.kinds[q.parentIndex])||
        s.armDispatch<=q.previousFinishDispatch||q.run!=s.run||q.cache!=s.cache||q.cache!=GetBaseProgramCache().GetGeneration()||
        !q.lease.Matches(s.lease)||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||
        !E237RecipeForProgramKind(q.firstRecipe,q.kinds[0U],q.program)||!E237RecipeForProgramKind(q.currentRecipe,q.kinds[q.parentIndex],q.program)||
        !E237HistoryProgramShape(GetBaseProgramCache(),first,kinds,program,count)||first!=q.firstArmPC||kinds!=q.kinds||program!=q.program||count!=q.parentCount||
        q.armPC[0U]!=first||q.armPC[1U]!=first+static_cast<int>(E237ArmStride(program,kinds[0U],1U,kinds[0U])))return false;
    for(unsigned i=1U;i<q.parentCount;++i)
        if(q.armPC[i]!=q.armPC[i-1U]+static_cast<int>(E237ArmStride(q.program,q.kinds[i-1U],i,q.kinds[0U])))return false;
    if(q.parentIndex==1U?(q.previousSession!=q.firstSession||q.previousFinishDispatch!=q.firstFinishDispatch):
        (q.previousSession<=q.firstSession||q.previousFinishDispatch<=q.firstFinishDispatch))return false;
    return E237SameParentRecipe(q.firstRecipe,q.currentRecipe,q.program);
}

bool NCManager::IsEDM233SequenceScopeCurrentSameThread() noexcept
{
    const auto& q=m_edm233;
    int first=-1;std::array<unsigned,2U> arms{};
    return q.authorized&&q.run&&q.cache&&q.confirmationDispatch&&q.startRequests&&q.firstArmPC>0&&
        q.epoch!=MOTION_EXECUTION_EPOCH_INVALID&&q.lease.IsValid()&&q.lease.owner==MotionOwner::AUTO&&
        q.run==m_pathCoreLiveBookkeeping.currentRunToken&&q.cache==GetBaseProgramCache().GetGeneration()&&
        q.epoch==m_motion.GetCurrentExecutionEpoch()&&q.lease.Matches(m_programMotionLease)&&
        m_motion.IsMotionOwnerLeaseCurrent(q.lease)&&!m_motion.HasPendingSafetyOrRecoveryRequests()&&
        q.startRequests==m_startDiagnosticRequests&&q.recipeGeneration==m_recipe.Current().generation&&
        q.processGeneration==m_edmProcessTuningGeneration&&m_state==NCState::RUN&&m_mode==NCOperationMode::MEMORY&&
        !Close_System_Com_flag&&m_externalReadyInterlock&&!m_isSingleBlockEnabled&&!m_isG66Active&&
        m_macroStack.empty()&&!Homing.IsActive()&&!AlarmManager::GetInstance().HasAlarm()&&
        m_resetContinuationPhase==ResetContinuationPhase::IDLE&&CoordSys.IsTranslationRunFrozen()&&
        IsG76OrbitFrameCurrentSameThread(true,true)&&IsPathCoreLiveNativeConfigCurrentSameThread()&&
        IsFixedTranslationTravelCurrentSameThread()&&
        SameNCTranslationSnapshot(q.translation,CoordSys.GetTranslationSnapshot())&&
        m_motion.MatchesNCTranslation(q.translation)&&
        E233ProgramShape(GetBaseProgramCache(),first,arms,&NCManager::IsEDM173BracketShapeValid,
            &NCManager::IsEDM173OrbitBlockShapeValid)&&first==q.firstArmPC&&arms==q.arms;
}

bool NCManager::IsEDM233SecondArmCurrentSameThread(int pc,unsigned arm) noexcept
{
    const auto& q=m_edm233;const auto& s=m_edm94;
    return IsEDM233SequenceScopeCurrentSameThread()&&q.firstReleased&&!q.releasePending&&!q.secondStarted&&
        q.firstSession&&q.releaseTick&&q.firstFinishDispatch&&q.releaseTick>q.releaseFloorTick&&
        !s.latched&&s.session==q.firstSession&&s.complete&&s.terminalLogged&&!s.cancelled&&
        !s.resetSeen&&!s.resetCommitted&&!s.releaseRequested&&!s.rtFaultReported&&
        pc==q.firstArmPC+3&&arm==q.arms[1U]&&GetActiveDispatchPC()==pc&&
        m_currentExecutingBlockDispatchId>q.firstFinishDispatch&&m_motion.IsGroupDone()&&m_motion.IsGroupNCDrained()&&
        !m_motion.GetCommandIngressSize()&&!m_motion.GetCommandReplaySize()&&!m_motion.GetAxisCommandMailboxDepth()&&
        !m_g76Orbit->active&&!m_pathFeed.pending&&!m_pathArc.pending&&!m_pathReplay.pending&&
        !m_pathHold.armed&&!m_pathHold.bound;
}

bool NCManager::BeginEDM233SequenceSameThread(int pc) noexcept
{
    int first=-1;std::array<unsigned,2U> arms{};
    if(!E233ProgramShape(GetBaseProgramCache(),first,arms,&NCManager::IsEDM173BracketShapeValid,
        &NCManager::IsEDM173OrbitBlockShapeValid))return false;
    if(pc==first+3)return IsEDM233SecondArmCurrentSameThread(pc,arms[1U]);
    const auto& c=m_edm149Confirmation;
    if(pc!=first||!c.confirmed||c.waiting||c.armPC!=pc||c.arm!=arms[0U]||
        c.run!=m_pathCoreLiveBookkeeping.currentRunToken||c.cache!=GetBaseProgramCache().GetGeneration()||
        !c.dispatch||m_currentExecutingBlockDispatchId<=c.dispatch||c.startRequests!=m_startDiagnosticRequests||
        !CoordSys.IsTranslationRunFrozen()||!m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))return false;
    m_edm233=EDM233NCState{};auto& q=m_edm233;
    q.translation=CoordSys.GetTranslationSnapshot();q.lease=m_programMotionLease;q.epoch=m_motion.GetCurrentExecutionEpoch();
    q.run=c.run;q.cache=c.cache;q.confirmationDispatch=c.dispatch;q.startRequests=c.startRequests;
    q.recipeGeneration=m_recipe.Current().generation;q.processGeneration=m_edmProcessTuningGeneration;
    q.firstArmPC=first;q.arms=arms;q.authorized=true;
    return IsEDM233SequenceScopeCurrentSameThread();
}

bool NCManager::ConsumeEDM233ParentReleaseSameThread()
{
    auto& q=m_edm233;auto& s=m_edm94;const auto& f=s.feedback;
    if(!q.releasePending)return false;
    if(!IsEDM233SequenceScopeCurrentSameThread()||q.firstReleased||q.secondStarted||
        s.session!=q.firstSession||s.armPC!=q.firstArmPC||s.next!=2U||!s.latched||!s.complete||
        !s.terminalLogged||s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        GetActiveDispatchPC()!=q.firstArmPC+2||m_waitingBlockDispatchId!=q.firstFinishDispatch||
        f.faulted||f.cancelled||f.parentReleasePhase==EDM94::ParentReleasePhase::Revoked)
    {RejectEDM94SameThread("EDM233_PARENT_RELEASE_SCOPE",m_gapDryRun.sourceLine);return false;}
    if(!f.parentReleased||f.parentReleasePhase!=EDM94::ParentReleasePhase::Released)return false;
    if(!f.released||f.active||!f.pass||!f.armAccepted||!f.finishRequested||!f.stopped||!f.forceZero||
        f.identity.session!=s.session||f.identity.epoch!=s.armEpoch||f.identity.ownerGeneration!=s.lease.generation||
        f.currentIdentity.session!=s.session||f.currentIdentity.epoch!=q.epoch||
        f.currentIdentity.ownerGeneration!=q.lease.generation||!f.parentReleaseTick||
        f.parentReleaseTick<=q.releaseFloorTick||f.parentReleaseTick!=f.lastTick||f.lastSentTick!=f.lastTick||
        f.requiredCycles<200U||f.settledCycles<f.requiredCycles||!E156FinishTargetMatches(s,m_edm130.recipe,f,m_edm114.retained)||
        !m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||m_motion.GetCommandIngressSize()||
        m_motion.GetCommandReplaySize()||m_motion.GetAxisCommandMailboxDepth()||m_g76Orbit->active)
    {RejectEDM94SameThread("EDM233_PARENT_RELEASE_RECEIPT",m_gapDryRun.sourceLine);return false;}
    q.releasePending=false;q.firstReleased=true;q.releaseTick=f.parentReleaseTick;
    PrintNCG76DiagnosticLine("[EDM233] event=PARENT_RELEASED parent=1 session=%llu tick=%llu nextArmPC=%d sameAUTO=1 freshSessionRequired=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(q.releaseTick),q.firstArmPC+3);
    s.latched=s.armPending=s.ready=s.finishPending=false;
    m_edm149Confirmation=EDM149Confirmation{};m_edm157=EDM157NCState{};m_edm200=EDM200AutoHoldState{};
    m_edm130.bound=false;
    return true;
}

bool NCManager::IsEDM173BracketBlockSameThread(const NCBlock& b) noexcept
{
    if(!IsEDM173BracketShapeValid(b)||m_mode!=NCOperationMode::MEMORY||m_state!=NCState::RUN||Close_System_Com_flag||
        !m_externalReadyInterlock||m_isSingleBlockEnabled||m_isG66Active||!m_macroStack.empty()||Homing.IsActive()||
        AlarmManager::GetInstance().HasAlarm()||m_motion.HasPendingSafetyOrRecoveryRequests()||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease)) return false;
    const auto& s=m_edm94; const int pc=GetActiveDispatchPC();
    const unsigned selector=static_cast<unsigned>(b.val('P'));
    if(EDM173::IsArm(selector))
    {
        if(!E251SquareConfirmationSource(GetBaseProgramCache(),pc))return false;
        if(b.has('L')&&(b.val('L')==237.0||b.val('L')==238.0||b.val('L')==239.0||b.val('L')==240.0||b.val('L')==241.0||b.val('L')==242.0||b.val('L')==243.0||b.val('L')==244.0||b.val('L')==245.0||b.val('L')==246.0||b.val('L')==247.0||b.val('L')==248.0)&&m_edm237.authorized)return IsEDM237NextArmCurrentSameThread(pc,selector);
        if(b.has('L')&&b.val('L')==236.0&&m_edm234.authorized)return IsEDM236NextArmCurrentSameThread(pc,selector);
        if(b.has('L')&&(b.val('L')==234.0||b.val('L')==235.0)&&m_edm234.authorized&&pc==m_edm234.firstArmPC+3)
            return IsEDM234NextArmCurrentSameThread(pc,selector);
        if(b.has('L')&&b.val('L')==233.0&&m_edm233.authorized&&pc==m_edm233.firstArmPC+3)
            return IsEDM233SecondArmCurrentSameThread(pc,selector);
        const auto& c=m_edm149Confirmation;
        return !s.latched&&c.confirmed&&!c.waiting&&c.arm==selector&&c.armPC==pc&&
            c.run==m_pathCoreLiveBookkeeping.currentRunToken&&c.cache==GetBaseProgramCache().GetGeneration()&&
            c.startRequests==m_startDiagnosticRequests&&c.dispatch&&m_currentExecutingBlockDispatchId>c.dispatch;
    }
    return s.latched&&EDM173::IsArm(s.gapHistoryArm)&&selector==EDM173::FinishForArm(s.gapHistoryArm)&&
        s.ready&&!s.cancelled&&!s.armPending&&!s.finishPending&&!s.complete&&
        s.run==m_pathCoreLiveBookkeeping.currentRunToken&&s.cache==GetBaseProgramCache().GetGeneration()&&
        s.lease.Matches(m_programMotionLease)&&s.moves==1U&&pc==s.pc[1U]&&m_edm173->completed;
}

bool NCManager::FreezeEDM173TranslationSameThread()
{
    if(!IsG76OrbitFrameCurrentSameThread(true,true)||!IsPathCoreLiveNativeConfigCurrentSameThread()||
        !EDM173::FrameSupported(CoordSys.GetTranslationSnapshot())) return false;
    if(CoordSys.IsTranslationRunFrozen())
    {
        if(!IsFixedTranslationTravelCurrentSameThread()||!m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot())) return false;
        // Native points already contain the frozen G68 transform, including
        // when an earlier stopped command froze this same frame.
        if(CoordSys.GetTranslationSnapshot().rotationMode==68)
            m_motion.SetCoordinateTransform(false,0.0,0.0,0.0,0.0,0.0,0.0);
        return true;
    }
    const auto candidate=CoordSys.GetTranslationSnapshot();
    if(!CoordSys.FreezeTranslationRun()) return false;
    for(unsigned axisIndex=0U;axisIndex<8U;++axisIndex)
    {
        const auto& axis=m_motion.GetAxisContext(static_cast<int>(axisIndex));auto& captured=m_fixedTranslationTravel[axisIndex];
        captured.minimum[0]=axis.travelLimit1Negative_unit;captured.minimum[1]=axis.travelLimit2Negative_unit;captured.minimum[2]=axis.travelLimit3Negative_unit;
        captured.maximum[0]=axis.travelLimit1Positive_unit;captured.maximum[1]=axis.travelLimit2Positive_unit;captured.maximum[2]=axis.travelLimit3Positive_unit;
        captured.enabled[0]=axis.travelLimit1Enable;captured.enabled[1]=axis.travelLimit2Enable;captured.enabled[2]=axis.travelLimit3Enable;
        captured.homed=axis.isHomed;captured.positionalRotary=axis.isExist&&axis.axisType==AxisType::ROTARY;
        captured.shortestPath=axis.useShortestPath;captured.rotaryModulo=axis.rotaryModulo;captured.maximumVelocity=axis.maxVel_PPS;
        captured.accelerationTime=axis.G00_acc_time;captured.decelerationTime=axis.G00_dec_time;captured.stopDecelerationTime=axis.Stop_dec_time;
    }
    const auto frozen=CoordSys.GetTranslationSnapshot();m_fixedTranslationTravelGeneration=frozen.generation;
    if(!SameNCTranslationSnapshot(candidate,frozen)||!m_motion.PublishNCTranslation(frozen)) return false;
    if(frozen.rotationMode==68)m_motion.SetCoordinateTransform(false,0.0,0.0,0.0,0.0,0.0,0.0);
    return true;
}

bool NCManager::PreflightEDM173ProgramSameThread(int armPC) noexcept
{
    auto& workspace=*m_edm173;
    workspace.prepared=false;workspace.coordinateRow=EDM179::Row{};workspace.coordinateScope=EDM173::Scope{};
    const auto& cache=GetBaseProgramCache();
    if(armPC<1||cache.Size()>128U||static_cast<std::size_t>(armPC)+3U>=cache.Size()) return false;
    int sequenceFirst=-1;std::array<unsigned,2U> sequenceArms{};std::array<unsigned,4U> chainKinds{};
    const bool historySequence=E237ProgramShape(cache,sequenceFirst,sequenceArms);
    const bool chain=!historySequence&&E236ProgramShape(cache,sequenceFirst,chainKinds);
    const bool mixedSequence=historySequence||chain||E234ProgramShape(cache,sequenceFirst,sequenceArms);
    const bool sequence=mixedSequence||E233ProgramShape(cache,sequenceFirst,sequenceArms,
        &NCManager::IsEDM173BracketShapeValid,&NCManager::IsEDM173OrbitBlockShapeValid);
    NCBlock arm{},orbit{},finish{},end{};
    const auto* armLine=cache.TryGetLine(armPC);
    const auto* a=cache.TryGetLine(armPC+1);const auto* b=cache.TryGetLine(armPC+2);const auto* c=cache.TryGetLine(armPC+3);
    if(!armLine||!a||!b||!c||!E251SquareConfirmationSource(cache,armPC)||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(armLine->parsedBlock,arm)||
        !IsEDM173BracketShapeValid(arm)||!EDM173::IsArm(static_cast<unsigned>(arm.val('P')))||
        (!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(a->parsedBlock,orbit)&&
            !EDM179::Parse(a->parsedBlock,workspace.coordinateRow.source,orbit))||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(b->parsedBlock,finish)||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(c->parsedBlock,end)||!IsEDM173OrbitBlockShapeValid(orbit)||
        !E251SquareSourcePair(armLine->parsedBlock,orbit)||
        ((orbit.has('K')||!orbit.has('B')||orbit.val('B')==0.0)&&!workspace.coordinateRow.source.expressionMask&&
            !E180LiteralAngleSourceMatches(a->parsedBlock,orbit))||
        E181EffectiveOrbitMode(orbit)!=EDM173::ModeForArm(static_cast<unsigned>(arm.val('P')))||
        !IsEDM173BracketShapeValid(finish)||finish.val('P')!=
            static_cast<double>(EDM173::FinishForArm(static_cast<unsigned>(arm.val('P'))))||
        (!sequence&&(end.gCount||end.mCount!=1||end.mCode[0]!=30||end.isGoto||end.isBlockSkip)))return false;
    if(sequence)
    {
        if(historySequence)
        {
            std::array<unsigned,4U> kinds{};unsigned program=0U,count=0U;int first=-1;
            if(!E237HistoryProgramShape(cache,first,kinds,program,count))return false;
            int candidate=first;bool matched=false;
            for(unsigned i=0U;i<count;++i){if(armPC==candidate&&E237NativeKind(kinds[i]))matched=true;
                candidate+=static_cast<int>(E237ArmStride(program,kinds[i],i+1U,kinds[0U]));}
            if(!matched||!E237ArmSequence(armLine->parsedBlock,static_cast<unsigned>(arm.val('P'))))return false;
        }
        else if((chain?(armPC<sequenceFirst||armPC>sequenceFirst+9||(armPC-sequenceFirst)%3||
                chainKinds[static_cast<unsigned>(armPC-sequenceFirst)/3U]!=2U):(armPC!=sequenceFirst&&armPC!=sequenceFirst+3))||
            !(mixedSequence?E234ArmSequence(armLine->parsedBlock,static_cast<unsigned>(arm.val('P'))):
                E233CenterParentSequence(armLine->parsedBlock,static_cast<unsigned>(arm.val('P')))))return false;
    }
    else
    {
        if(arm.has('L')&&(arm.val('L')==233.0||arm.val('L')==234.0||arm.val('L')==235.0||arm.val('L')==236.0||arm.val('L')==237.0||arm.val('L')==238.0||arm.val('L')==239.0||arm.val('L')==240.0||arm.val('L')==241.0||arm.val('L')==242.0||arm.val('L')==243.0||arm.val('L')==244.0||arm.val('L')==245.0||arm.val('L')==246.0||arm.val('L')==247.0||arm.val('L')==248.0))return false;
        for(char word='A';word<='Z';++word) if(end.has(word)&&(word!='N'||!std::isfinite(end.val(word)))) return false;
        for(std::size_t pc=static_cast<std::size_t>(armPC)+4U;pc<cache.Size();++pc)
        {const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line||!line->parsedBlock.isEmpty) return false;}
    }
    m_edm173->authored=orbit; m_edm173->prepared=true;m_edm173->published=m_edm173->parentCommitted=m_edm173->completed=false;
    return true;
}

// Resolve only the preflighted read-only coordinate row, after the accepted
// frame and M00 confirmation have been fixed. Runtime still resolves the
// original authored expressions independently before its dispatch guard.
bool NCManager::FreezeEDM179CoordinatesSameThread()
{
    auto& w=*m_edm173;const auto& s=m_edm94;
    w.coordinateRow.valid=false;w.coordinateScope=EDM173::Scope{};
    const auto& cache=GetBaseProgramCache();const auto generation=cache.GetGeneration();
    const auto* source=cache.TryGetLine(s.pc[0U]);
    const auto frozen=CoordSys.GetTranslationSnapshot();
    if(!w.prepared||!w.coordinateRow.source.expressionMask||!source||
        !EDM179::MatchesSource(source->parsedBlock,w.coordinateRow.source)||
        !EDM173::IsArm(s.gapHistoryArm)||!s.session||!s.run||s.run!=m_pathCoreLiveBookkeeping.currentRunToken||
        !generation||s.cache!=generation||s.armEpoch!=m_motion.GetCurrentExecutionEpoch()||
        !s.armDispatch||s.armPC<0||s.pc[0U]!=s.armPC+1||GetActiveDispatchPC()!=s.armPC||
        !s.lease.Matches(m_programMotionLease)||!m_motion.IsMotionOwnerLeaseCurrent(s.lease)||
        !CoordSys.IsTranslationRunFrozen()||!IsG76OrbitFrameCurrentSameThread(true,true)||
        !IsPathCoreLiveNativeConfigCurrentSameThread()||!EDM173::FrameSupported(frozen)||frozen.distanceMode!=90||
        m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||m_isSingleBlockEnabled||m_isG66Active||
        !m_macroStack.empty()||Close_System_Com_flag||!m_externalReadyInterlock||
        AlarmManager::GetInstance().HasAlarm()||m_motion.HasPendingSafetyOrRecoveryRequests()) return false;
    NCG76::Point8 start{},programStart{};
    for(unsigned axis=0U;axis<8U;++axis)start[axis]=CoordSys.commandedMCS[axis];
    if(!EDM173::Detail::TryOrbitProgramStart(frozen,start,programStart)) return false;
    // This is the ordinary system-variable publication, using the real
    // CoordinateManager getter. It does not substitute synthetic coordinates.
    UpdateSystemVariables();
    std::array<double,3U> coordinateStart{};
    for(unsigned axis=0U;axis<3U;++axis)
    {
        coordinateStart[axis]=programStart[axis];
        if(!EDM179::SameDouble(MacroSys.GetVar('$',static_cast<int>(100U+axis)),coordinateStart[axis])) return false;
    }
    NCBlock resolved{};NCExpressionResolveError error=NCExpressionResolveError::NONE;
    if(!NCExpressionResolver::ResolveBlock(source->parsedBlock,MathParser,resolved,error)||
        !IsEDM173OrbitBlockShapeValid(resolved)||
        !EDM179::Capture(source->parsedBlock,resolved,coordinateStart,s.pc[0U],w.coordinateRow)) return false;
    bool current=generation==cache.GetGeneration()&&SameNCTranslationSnapshot(frozen,CoordSys.GetTranslationSnapshot())&&
        s.armEpoch==m_motion.GetCurrentExecutionEpoch()&&m_motion.IsMotionOwnerLeaseCurrent(s.lease);
    for(unsigned axis=0U;axis<8U&&current;++axis)
        current=EDM179::SameDouble(start[axis],CoordSys.commandedMCS[axis]);
    if(!current){w.coordinateRow.valid=false;return false;}
    auto& scope=w.coordinateScope;
    scope.session=s.session;scope.run=s.run;scope.cache=s.cache;scope.dispatch=s.armDispatch;
    scope.sourcePC=s.pc[0U];scope.line=source->sourceLineNumber;scope.armEpoch=s.armEpoch;scope.owner=s.lease;
    if(!EDM173::ValidScope(scope)){w.coordinateRow.valid=false;scope=EDM173::Scope{};return false;}
    w.authored=resolved;
    return true;
}

bool NCManager::IsEDM179CoordinateCandidateSameThread(const NCParsedBlock& parsed,int pc) noexcept
{
    const auto& s=m_edm94;const auto& w=*m_edm173;const auto& scope=w.coordinateScope;
    if(!EDM179::Matches(parsed,w.coordinateRow)||!EDM173::ValidScope(scope)||
        pc<0||pc!=w.coordinateRow.pc||pc!=scope.sourcePC||pc!=s.pc[0U]||GetBasePCValue()!=pc||
        !EDM173::IsArm(s.gapHistoryArm)||!s.latched||!s.ready||!s.everArmed||s.next>1U||s.moves!=1U||
        s.armPending||s.finishPending||s.complete||s.cancelled||s.resetSeen||s.resetCommitted||
        s.releaseRequested||s.rtFaultReported||!w.prepared||w.published||
        scope.session!=s.session||scope.run!=s.run||scope.cache!=s.cache||scope.dispatch!=s.armDispatch||
        scope.armEpoch!=s.armEpoch||!scope.owner.Matches(s.lease)||
        s.run!=m_pathCoreLiveBookkeeping.currentRunToken||s.cache!=GetBaseProgramCache().GetGeneration()||
        s.armEpoch!=m_motion.GetCurrentExecutionEpoch()||!s.lease.Matches(m_programMotionLease)||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease)||m_motion.HasPendingSafetyOrRecoveryRequests()||
        m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||m_isSingleBlockEnabled||m_isG66Active||
        !m_macroStack.empty()||Close_System_Com_flag||!m_externalReadyInterlock||Homing.IsActive()||
        AlarmManager::GetInstance().HasAlarm()||!CoordSys.IsTranslationRunFrozen()||
        !IsG76OrbitFrameCurrentSameThread(true,true)||!IsPathCoreLiveNativeConfigCurrentSameThread()||
        w.nominal.translation.distanceMode!=90||
        !SameNCTranslationSnapshot(w.nominal.translation,CoordSys.GetTranslationSnapshot())) return false;
    const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if(!source||source->sourceLineNumber!=scope.line||!EDM179::Matches(source->parsedBlock,w.coordinateRow)) return false;
    for(unsigned axis=0U;axis<8U;++axis)
        if(!EDM179::SameDouble(CoordSys.commandedMCS[axis],w.nominal.startMCS[axis])) return false;
    NCG76::Point8 programStart{};
    if(!EDM173::Detail::TryOrbitProgramStart(w.nominal.translation,w.nominal.startMCS,programStart)) return false;
    for(unsigned axis=0U;axis<3U;++axis)
        if(!EDM179::SameDouble(programStart[axis],w.coordinateRow.programStart[axis])||
            !EDM179::SameDouble(MacroSys.GetVar('$',static_cast<int>(100U+axis)),w.coordinateRow.programStart[axis])) return false;
    return true;
}

bool NCManager::MatchesEDM179CoordinatesSameThread(const NCParsedBlock& parsed,const NCBlock& resolved,int pc) noexcept
{
    return IsEDM179CoordinateCandidateSameThread(parsed,pc)&&
        EDM179::MatchesResolved(parsed,resolved,m_edm173->coordinateRow);
}

bool NCManager::BindEDM173RecipeSameThread() noexcept
{
    const auto* historyArm=GetBaseProgramCache().TryGetLine(m_edm94.armPC);
    if(historyArm&&E237ArmSequence(historyArm->parsedBlock,m_edm94.gapHistoryArm))return BindEDM237RecipeSameThread();
    const auto* mixedArm=GetBaseProgramCache().TryGetLine(m_edm94.armPC);
    if(mixedArm&&E234ArmSequence(mixedArm->parsedBlock,m_edm94.gapHistoryArm))return BindEDM234RecipeSameThread();
    m_edm130=EDM130NCState{};EDMRecipe::Summary summary{};
    m_edm200=EDM200AutoHoldState{};
    if(!m_edmProcessReady||!EDM173::IsArm(m_edm94.gapHistoryArm))return false;
    const auto& cache=GetBaseProgramCache();const auto* arm=cache.TryGetLine(m_edm94.armPC);
    EDM97::Tuning tuning{};EDM107::FlushProfile profile{};
    if(cache.GetGeneration()!=m_edm94.cache||!arm||
        !E108ProfileOnlyNPQRSIJK(arm->parsedBlock,m_edm94.gapHistoryArm,tuning,profile)||
        !EDM97::SameTuning(tuning,m_edm94.gapTuning)||
        !EDM107::SameFlushProfile(profile,m_edm94.gapFlushParameters))return false;
    // The short architecture stroke is bound only from this exact authored
    // L233 arm. Legacy center binders keep their established height envelope.
    const bool architecture=E233CenterParentSequence(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool squareB1=E250SquareB1Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool squareB2=E251SquareB2Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool squareB3=E252SquareB3Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool squareB4=E253SquareB4Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool diamond=E255DiamondB1Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool diamondB2=E256DiamondB2Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool diamondB3=E257DiamondB3Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool diamondB4=E258DiamondB4Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool oneMm=E260OneMmArm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool polygonB2=E261PolygonB2Arm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool expansion=E263ExpansionArm(arm->parsedBlock,m_edm94.gapHistoryArm);
    const bool fixedRadius=E266FixedRadiusArm(arm->parsedBlock,m_edm94.gapHistoryArm);
    if(fixedRadius&&CoordSys.GetTranslationSnapshot().workMode!=169)return false;
    const bool fixedRadiusWork=E267FixedRadiusWorkArm(arm->parsedBlock,m_edm94.gapHistoryArm);
    if(fixedRadiusWork&&!E267FixedRadiusWorkFrame(CoordSys.GetTranslationSnapshot()))return false;
    if(!E251SquareConfirmationSource(cache,m_edm94.armPC))return false;
    const bool sharedShort=E274SharedShortArm(arm->parsedBlock,m_edm94.gapHistoryArm);
    unsigned sharedShortMode=0U;
    if(sharedShort)
    {
        const auto* source=m_edm94.armPC<(std::numeric_limits<int>::max)()?cache.TryGetLine(m_edm94.armPC+1):nullptr;
        NCBlock authored{};
        if(CoordSys.GetTranslationSnapshot().workMode!=169||!source||
            !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(source->parsedBlock,authored)||
            !IsEDM173OrbitBlockShapeValid(authored)||!E251SquareSourcePair(arm->parsedBlock,authored)||
            !E180LiteralAngleSourceMatches(source->parsedBlock,m_edm173->authored))return false;
        sharedShortMode=E274SharedShortMode(authored);
        if(!sharedShortMode)return false;
    }
    // EDM233_RECIPE_CONTINUATION_BEGIN
    // A completed first parent retains its immutable recipe/plan. Its exact
    // sequence grant, not the generic idle recipe reader, admits the second
    // parent after the native transaction advanced the execution epoch.
    const bool continuation=architecture&&m_edm233.authorized&&m_edm94.armPC==m_edm233.firstArmPC+3;
    EDM130::Recipe firstRecipe{};
    if(continuation)
    {
        const auto& q=m_edm233;const auto& s=m_edm94;
        if(!m_edm173||!IsEDM233SequenceScopeCurrentSameThread()||!q.firstReleased||q.releasePending||q.secondStarted||
            !q.firstSession||!q.firstFinishDispatch||!q.releaseTick||q.releaseTick<=q.releaseFloorTick||
            s.session<=q.firstSession||s.armPC!=q.firstArmPC+3||s.gapHistoryArm!=q.arms[1U]||
            s.run!=q.run||s.cache!=q.cache||s.armEpoch!=q.epoch||!s.lease.Matches(q.lease)||
            s.latched||s.next!=0U||s.moves!=1U||s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
            GetActiveDispatchPC()!=s.armPC||GetBasePC()!=s.armPC||m_currentExecutingBlockDispatchId<=q.firstFinishDispatch||
            m_recipeRecoveryDisplayOnly||m_recipeHoldSeen||m_edmState==EDMState::NOT_READY||!m_recipe.Ready())return false;
        const auto& first=m_edm173->plan;const auto& scope=first.scope;
        const MotionExecutionEpoch firstLiveEpoch=scope.armEpoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:scope.armEpoch+1U;
        const auto* firstSource=cache.TryGetLine(q.firstArmPC+1);
        if(!EDM173::Valid(first)||!EDM130::IsG76ParentSequence233(first.recipe)||
            !EDM173::RecipeMatchesArm(q.arms[0U],first.recipe)||!firstSource||
            scope.session!=q.firstSession||scope.run!=q.run||scope.cache!=q.cache||
            scope.sourcePC!=q.firstArmPC+1||scope.line!=firstSource->sourceLineNumber||
            !scope.owner.Matches(q.lease)||scope.armEpoch==MOTION_EXECUTION_EPOCH_INVALID||firstLiveEpoch!=q.epoch||
            !scope.dispatch||scope.dispatch>=q.firstFinishDispatch||
            !SameNCTranslationSnapshot(first.nominal.translation,q.translation))return false;
        summary=m_recipe.Current();
        if(!summary.catalogReady||!summary.tableSelected||!summary.rowSelected||
            !summary.generation||summary.generation!=q.recipeGeneration||summary.generation!=first.recipe.generation)return false;
        firstRecipe=first.recipe;
    }
    else if(!ReadEDMRecipeSummarySameThread(summary))return false;
    // The global recipe run/cache/epoch/lease and logical-only flag are never
    // changed by this continuation. Other profiles keep the original reader.
    // EDM233_RECIPE_CONTINUATION_END
    // EDM267_FIX2_BINDING_DISPATCH_BEGIN
    // Select scalar inputs first. The complete immutable capture is shared by
    // every shape; do not duplicate its large inline tree in a conditional result.
    unsigned bindingProfile=EDM130::VisibleFlushProfile173;
    unsigned bindingSelector=0U;
    if(sharedShort)
    {
        const unsigned bMode=EDM173::ModeForArm(m_edm94.gapHistoryArm);
        bindingProfile=bMode==1U?EDM130::VisibleFlushProfile173:bMode==3U?EDM130::G76B3Profile176:
            bMode==4U?EDM130::G76B4Profile177:0U;
        bindingSelector=EDM130::G76SharedShortSelector274(sharedShortMode);
        if(!bindingProfile||!bindingSelector)return false;
    }
    else if(fixedRadiusWork||fixedRadius||expansion)
    {
        const unsigned mode=fixedRadiusWork?E267FixedRadiusWorkMode(arm->parsedBlock):
            fixedRadius?E266FixedRadiusMode(arm->parsedBlock):E263ExpansionMode(arm->parsedBlock);
        const bool expansionMode=!fixedRadiusWork&&!fixedRadius;
        const unsigned shape=expansionMode?(mode==21U?0U:mode==22U?1U:mode==23U?2U:mode==26U?3U:4U):
            (mode==1U?0U:mode==2U?1U:mode==3U?2U:mode==6U?3U:4U);
        const unsigned bMode=EDM173::ModeForArm(m_edm94.gapHistoryArm);
        bindingProfile=bMode==1U?EDM130::VisibleFlushProfile173:bMode==2U?EDM130::G76B2Profile175:
            bMode==3U?EDM130::G76B3Profile176:bMode==4U?EDM130::G76B4Profile177:0U;
        if(shape>=4U||!bindingProfile)return false;
        bindingSelector=(fixedRadiusWork?(bMode==2U?285U:281U):
            fixedRadius?(bMode==2U?277U:273U):(bMode==2U?269U:265U))+shape;
    }
    else if(polygonB2){bindingProfile=EDM130::G76B2Profile175;bindingSelector=264U;}
    else if(oneMm)
    {
        const unsigned mode=E260OneMmMode(arm->parsedBlock);
        const unsigned bMode=EDM173::ModeForArm(m_edm94.gapHistoryArm);
        bindingSelector=mode==11U?260U:mode==12U?261U:mode==13U?262U:mode==16U?263U:0U;
        bindingProfile=bMode==1U?EDM130::VisibleFlushProfile173:bMode==3U?EDM130::G76B3Profile176:
            bMode==4U?EDM130::G76B4Profile177:0U;
        if(!bindingSelector||!bindingProfile)return false;
    }
    else if(diamondB4){bindingProfile=EDM130::G76B4Profile177;bindingSelector=258U;}
    else if(diamondB3){bindingProfile=EDM130::G76B3Profile176;bindingSelector=257U;}
    else if(diamondB2){bindingProfile=EDM130::G76B2Profile175;bindingSelector=256U;}
    else if(diamond){bindingProfile=EDM130::VisibleFlushProfile173;bindingSelector=255U;}
    else if(squareB4){bindingProfile=EDM130::G76B4Profile177;bindingSelector=253U;}
    else if(squareB3){bindingProfile=EDM130::G76B3Profile176;bindingSelector=252U;}
    else if(squareB2){bindingProfile=EDM130::G76B2Profile175;bindingSelector=251U;}
    else if(squareB1){bindingProfile=EDM130::VisibleFlushProfile173;bindingSelector=250U;}
    else if(architecture)
    {
        bindingProfile=m_edm94.gapHistoryArm==EDM173::B3ArmP?EDM130::G76B3Profile176:EDM130::G76B4Profile177;
        bindingSelector=233U;
    }
    else if(m_edm94.gapHistoryArm==EDM173::B0ArmP)bindingProfile=EDM130::G76B0Profile192;
    else if(m_edm94.gapHistoryArm==EDM173::B4ArmP)bindingProfile=EDM130::G76B4Profile177;
    else if(m_edm94.gapHistoryArm==EDM173::B3ArmP)bindingProfile=EDM130::G76B3Profile176;
    else if(m_edm94.gapHistoryArm==EDM173::B2ArmP)bindingProfile=EDM130::G76B2Profile175;
    const auto binding=EDM130::BindG76GapProfile175(m_recipe,m_edmProcessProfile.machineProfileId,
        m_edmProcessProfile,m_edmProcessTuningGeneration,bindingProfile,bindingSelector);
    // EDM267_FIX2_BINDING_DISPATCH_END
    if(!binding.ready) return false;
    auto& recipe=m_edm130.recipe;recipe=binding.recipe;recipe.diagnosticControlSource=3U;recipe.planRows=static_cast<std::uint16_t>(EDM173::CountForRecipe(binding.recipe));recipe.planMovementMask=7U;
    if((sharedShort&&(!EDM130::IsG76SharedShort274(recipe)||EDM130::G76PlanMode(recipe)!=sharedShortMode||
            recipe.flushHoldTest!=EDM130::G76SharedShortSelector274(sharedShortMode)))||
        (squareB1&&!EDM130::IsG76SquareB1_250(recipe))||(squareB2&&!EDM130::IsG76SquareB2_251(recipe))||
        (squareB3&&!EDM130::IsG76SquareB3_252(recipe))||(squareB4&&!EDM130::IsG76SquareB4_253(recipe))||
        (diamond&&!EDM130::IsG76DiamondB1_255(recipe))||(diamondB2&&!EDM130::IsG76DiamondB2_256(recipe))||
        (diamondB3&&!EDM130::IsG76DiamondB3_257(recipe))||(diamondB4&&!EDM130::IsG76DiamondB4_258(recipe))||
        (oneMm&&(!EDM130::IsG76OneMm260(recipe)||EDM130::G76PlanMode(recipe)!=E260OneMmMode(arm->parsedBlock)))||
        (polygonB2&&!EDM130::IsG76PolygonB2_261(recipe))||
        (expansion&&(!EDM130::IsG76Expansion263(recipe)||EDM130::G76PlanMode(recipe)!=E263ExpansionMode(arm->parsedBlock)))||
        (fixedRadius&&(!EDM130::IsG76FixedRadius266(recipe)||EDM130::G76PlanMode(recipe)!=E266FixedRadiusMode(arm->parsedBlock)))||
        (fixedRadiusWork&&(!EDM130::IsG76FixedRadiusWork267(recipe)||EDM130::G76PlanMode(recipe)!=E267FixedRadiusWorkMode(arm->parsedBlock))))return false;
    if(arm->parsedBlock.has('L')&&!squareB1&&!squareB2&&!squareB3&&!squareB4&&!diamond&&!diamondB2&&!diamondB3&&!diamondB4&&!oneMm&&!polygonB2&&!expansion&&!fixedRadius&&!fixedRadiusWork&&!sharedShort&&!E216RejectBeforeArm(arm->parsedBlock,m_edm94.gapHistoryArm))
    {
        if(E217CenterNominalHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=1U;
        else if(E218CenterFlushHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=2U;
        else if(E219CenterReturnHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=3U;
        else if(E220CenterAnchorHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=4U;
        else if(E221CenterStartHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=5U;
        else if(E222CenterTopDwellHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=6U;
        else if(E224CenterDwellHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=7U;
        else if(E225CenterReturnDwellHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=8U;
        else if(E226CenterFinalDwellHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=9U;
        else if(E227CenterInitialDwellHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=10U;
        else if(E228CenterBrakeHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=11U;
        else if(E230CenterRepeat(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=230U;
        else if(E231CenterSecondFlushHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=231U;
        else if(E232CenterSecondReturnHold(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=232U;
        else if(E233CenterParentSequence(arm->parsedBlock,m_edm94.gapHistoryArm))recipe.flushHoldTest=233U;
        else
        {
            if(m_edm94.gapHistoryArm!=EDM173::B2ArmP||arm->parsedBlock.has('H')||
                !EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),3U))return false;
            recipe.flushHoldTest=3U;
        }
    }
    if(!EDM130::ValidRecipe(recipe)||!EDM173::RecipeMatchesArm(m_edm94.gapHistoryArm,recipe)||
        !EDM130::ValidG76FlushSpeedPercent(recipe.commissioningProfile,recipe.jumpSpeedPercentScaled)||
        recipe.workTimeUs<250000ULL||recipe.workTimeUs>2000000ULL||
        recipe.deepFlushCycleInterval||recipe.holdRetreatMode||
        (recipe.flushHoldTest&&!EDM130::IsG76B2FlushHold200(recipe)&&!EDM130::IsG76CenterHold217(recipe)&&
            !EDM130::IsG76CenterAnyFlushHold232(recipe)&&!EDM130::IsG76CenterRepeat230(recipe)&&!EDM130::IsG76ParentSequence233(recipe)&&!EDM130::IsG76CommissioningRecipe(recipe))||
        !EDM97::ValidMotionTestCurve(m_edm94.gapTuning,recipe)) return false;
    m_edm94.gapFlushParameters.speedRatio=EDM130::FrozenSpeedRatio(recipe);
    if(!EDM130::MatchesFlushSpeed(recipe,m_edm94.gapFlushParameters.speedRatio)||
        !EDM130::MatchesFlushHeight(recipe,m_edm94.gapFlushParameters.strokeRatio,m_edm94.gapFlushParameters.dwellUs)) return false;
    if(continuation)
    {
        // Only the authored B3/B4 commissioning profile may differ. Rebuild
        // from the actual controller, then compare every frozen recipe field.
        firstRecipe.commissioningProfile=recipe.commissioningProfile;
        if(!EDM130::SameRecipe(firstRecipe,recipe)||!IsEDM233SequenceScopeCurrentSameThread())return false;
        PrintNCG76DiagnosticLine("[EDM233] event=RECIPE_CONTINUED firstSession=%llu secondSession=%llu generation=%llu armEpoch=%llu B=%u proof=FIRST_PARENT_FULL_RECIPE globalRecipeScope=UNCHANGED discharge=0\n",
            static_cast<unsigned long long>(m_edm233.firstSession),static_cast<unsigned long long>(m_edm94.session),
            static_cast<unsigned long long>(summary.generation),static_cast<unsigned long long>(m_edm94.armEpoch),EDM173::ModeForArm(m_edm94.gapHistoryArm));
    }
    m_edm130.bound=true;
    if(sharedShort)
        PrintNCG76DiagnosticLine("[EDM274] event=VECTOR_RECIPE_BOUND session=%llu L=%u selector=%u P=%u Q=%u B=%u children=%u E5us=%llu E6microMm=%llu power=INHIBITED discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),EDM130::G76SharedShortArmTag274,
            static_cast<unsigned>(recipe.flushHoldTest),sharedShortMode,
            NCG76Mode::IsPolygon(sharedShortMode)?static_cast<unsigned>(m_edm173->authored.val('Q')):0U,
            EDM173::ModeForArm(m_edm94.gapHistoryArm),EDM173::CountForRecipe(recipe),
            static_cast<unsigned long long>(recipe.workTimeUs),static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(fixedRadiusWork)
    {
        const auto& frame=CoordSys.GetTranslationSnapshot();
        PrintNCG76DiagnosticLine("[EDM267] event=FIXED_RADIUS_WORK_RECIPE_BOUND session=%llu L=%u modeP=%u Q=%u B=%u children=26 E5us=%llu E6microMm=%llu E7percent=100 advance=CONSTANT_RADIUS hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned>(recipe.flushHoldTest),
            EDM130::G76PlanMode(recipe),static_cast<unsigned>(m_edm173->authored.val('Q')),EDM173::ModeForArm(m_edm94.gapHistoryArm),
            static_cast<unsigned long long>(recipe.workTimeUs),static_cast<unsigned long long>(recipe.jumpHeightScaled));
        PrintNCG76DiagnosticLine("[EDM267] event=FIXED_RADIUS_WORK_FRAME session=%llu generation=%llu workMode=G168 W=%d workYaw=%.9f rotation=%d rotationDeg=%.9f tool=%d H=%d scale=%.9f mirror=%u frozen=%u discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(frame.generation),frame.workWCode,
            frame.workOffset[3],frame.rotationMode,frame.rotationAngleDeg,frame.toolLengthMode,frame.toolHCode,
            frame.scalingFactor,static_cast<unsigned>(frame.mirrorMask),CoordSys.IsTranslationRunFrozen()?1U:0U);
    }
    if(fixedRadius)
        PrintNCG76DiagnosticLine("[EDM266] event=FIXED_RADIUS_RECIPE_BOUND session=%llu L=%u modeP=%u Q=%u B=%u children=26 E5us=%llu E6microMm=%llu E7percent=100 advance=CONSTANT_RADIUS hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned>(recipe.flushHoldTest),
            EDM130::G76PlanMode(recipe),static_cast<unsigned>(m_edm173->authored.val('Q')),EDM173::ModeForArm(m_edm94.gapHistoryArm),
            static_cast<unsigned long long>(recipe.workTimeUs),static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(expansion)
        PrintNCG76DiagnosticLine("[EDM263] event=EXPANSION_RECIPE_BOUND session=%llu L=%u modeP=%u Q=%u B=%u children=26 E5us=%llu E6microMm=%llu E7percent=100 expansion=TARGET_PLANE_GROW hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned>(recipe.flushHoldTest),
            EDM130::G76PlanMode(recipe),static_cast<unsigned>(m_edm173->authored.val('Q')),EDM173::ModeForArm(m_edm94.gapHistoryArm),
            static_cast<unsigned long long>(recipe.workTimeUs),static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(EDM130::G76PlanMode(recipe)==16U)
        PrintNCG76DiagnosticLine("[EDM261] event=POLYGON_RECIPE_BOUND session=%llu L=%u modeP=16 Q=%u B=%u children=25 E5us=%llu E6microMm=%llu E7percent=100 hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned>(recipe.flushHoldTest),
            static_cast<unsigned>(m_edm173->authored.val('Q')),EDM173::ModeForArm(m_edm94.gapHistoryArm),
            static_cast<unsigned long long>(recipe.workTimeUs),static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(oneMm)
        PrintNCG76DiagnosticLine("[EDM260] event=ONE_MM_RECIPE_BOUND session=%llu armP=%u L=%u modeP=%u B=%u children=25 E5us=%llu E6microMm=%llu E7percent=100 radiusScope=AUTHORED hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            static_cast<unsigned>(recipe.flushHoldTest),EDM130::G76PlanMode(recipe),EDM173::ModeForArm(m_edm94.gapHistoryArm),
            static_cast<unsigned long long>(recipe.workTimeUs),static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(squareB1)
        PrintNCG76DiagnosticLine("[EDM250] event=P12_B1_RECIPE_BOUND session=%llu armP=292 L=250 modeP=12 shape=SQUARE B=1 children=25 E5us=%llu E6microMm=%llu E7percent=100 flushAxis=NATIVE_POSITIVE_Z hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(squareB2)
        PrintNCG76DiagnosticLine("[EDM251] event=P12_B2_RECIPE_BOUND session=%llu armP=294 L=251 modeP=12 shape=SQUARE B=2 children=25 E5us=%llu E6microMm=%llu flushPath=RETAINED_NOMINAL hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(squareB3)
        PrintNCG76DiagnosticLine("[EDM252] event=P12_B3_RECIPE_BOUND session=%llu armP=296 L=252 modeP=12 shape=SQUARE B=3 children=25 E5us=%llu E6microMm=%llu E7percent=100 flushAxis=SIGNED_MACHINING_REVERSE hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(squareB4)
        PrintNCG76DiagnosticLine("[EDM253] event=P12_B4_RECIPE_BOUND session=%llu armP=298 L=253 modeP=12 shape=SQUARE B=4 children=25 E5us=%llu E6microMm=%llu E7percent=100 centerJoin=DIAGONAL_45_DEG flushAxis=SIGNED_MACHINING_REVERSE hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(diamond)
        PrintNCG76DiagnosticLine("[EDM255] event=P13_B1_RECIPE_BOUND session=%llu armP=292 L=255 modeP=13 shape=DIAMOND B=1 children=25 E5us=%llu E6microMm=%llu E7percent=100 flushAxis=NATIVE_POSITIVE_Z hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(diamondB2)
        PrintNCG76DiagnosticLine("[EDM256] event=P13_B2_RECIPE_BOUND session=%llu armP=294 L=256 modeP=13 shape=DIAMOND B=2 children=25 E5us=%llu E6microMm=%llu E7percent=100 flushPath=RETAINED_NOMINAL hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(diamondB3)
        PrintNCG76DiagnosticLine("[EDM257] event=P13_B3_RECIPE_BOUND session=%llu armP=296 L=257 modeP=13 shape=DIAMOND B=3 children=25 E5us=%llu E6microMm=%llu E7percent=100 flushAxis=SIGNED_MACHINING_REVERSE hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(diamondB4)
        PrintNCG76DiagnosticLine("[EDM258] event=P13_B4_RECIPE_BOUND session=%llu armP=298 L=258 modeP=13 shape=DIAMOND B=4 children=25 E5us=%llu E6microMm=%llu E7percent=100 centerJoin=DIAGONAL_45_DEG flushAxis=SIGNED_MACHINING_REVERSE hold=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(recipe.workTimeUs),
            static_cast<unsigned long long>(recipe.jumpHeightScaled));
    if(EDM173::IsCenterRecipe(recipe))
        PrintNCG76DiagnosticLine("[EDM210] event=CENTER_FLUSH_RATE session=%llu mode=%u E7percent=%.9g axialScale=%.9g referenceCapMmSec=%.9g accelerationScale=%.9g hardDeadlineUs=%llu centerFeed=INDEPENDENT gains=CONFIGURED following=UNCHANGED discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),EDM173::ModeForArm(m_edm94.gapHistoryArm),
            static_cast<double>(recipe.jumpSpeedPercentScaled)/1000000.0,
            2.0*m_edm94.gapFlushParameters.speedRatio,
            EDM94::VisibleFlushReferenceMmSec*(2.0*m_edm94.gapFlushParameters.speedRatio),
            2.0*m_edm94.gapFlushParameters.speedRatio,
            static_cast<unsigned long long>(EDM94::VisibleFlushDurationUs(recipe.commissioningProfile)));
    if(EDM130::IsG76B2FlushHold200(recipe))
        RtPrintf("[EDM200] event=AUTO_HOLD_ARMED session=%llu armP=294 L=3 B=2 trigger=FIRST_CROSSED_B2_E5_OUTBOUND_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START holdRetreat=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session));
    if(EDM130::IsG76CenterHold217(recipe))
        PrintNCG76DiagnosticLine("[EDM217] event=AUTO_HOLD_ARMED session=%llu armP=%u L=1 B=%u trigger=NOMINAL_GAP_WORK_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterFlushHold218(recipe))
        PrintNCG76DiagnosticLine("[EDM218] event=AUTO_HOLD_ARMED session=%llu armP=%u L=2 B=%u trigger=CENTER_E5_AXIAL_OUTBOUND_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterReturnHold219(recipe))
        PrintNCG76DiagnosticLine("[EDM219] event=AUTO_HOLD_ARMED session=%llu armP=%u L=3 B=%u trigger=CENTER_E5_AXIAL_RETURN_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterAnchorHold220(recipe))
        PrintNCG76DiagnosticLine("[EDM220] event=AUTO_HOLD_ARMED session=%llu armP=%u L=4 B=%u trigger=CENTER_E5_TO_ANCHOR_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterStartHold221(recipe))
        PrintNCG76DiagnosticLine("[EDM221] event=AUTO_HOLD_ARMED session=%llu armP=%u L=5 B=%u trigger=CENTER_E5_TO_CENTER_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterTopDwellHold222(recipe))
        PrintNCG76DiagnosticLine("[EDM223] event=AUTO_HOLD_ARMED session=%llu armP=%u L=6 B=%u trigger=CENTER_E5_TOP_DWELL_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START dwellRestart=FULL holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterDwellHold224(recipe))
        PrintNCG76DiagnosticLine("[EDM224] event=AUTO_HOLD_ARMED session=%llu armP=%u L=7 B=%u trigger=CENTER_E5_CENTER_DWELL_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START dwellRestart=FULL holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterReturnDwellHold225(recipe))
        PrintNCG76DiagnosticLine("[EDM225] event=AUTO_HOLD_ARMED session=%llu armP=%u L=8 B=%u trigger=CENTER_E5_CENTER_RETURN_DWELL_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START dwellRestart=FULL holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterFinalDwellHold226(recipe))
        PrintNCG76DiagnosticLine("[EDM226] event=AUTO_HOLD_ARMED session=%llu armP=%u L=9 B=%u trigger=CENTER_E5_FINAL_RETURN_DWELL_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START dwellRestart=FULL holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterInitialDwellHold227(recipe))
        PrintNCG76DiagnosticLine("[EDM227] event=AUTO_HOLD_ARMED session=%llu armP=%u L=10 B=%u trigger=CENTER_E5_INITIAL_ANCHOR_DWELL_25PCT confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START dwellRestart=FULL holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    if(EDM130::IsG76CenterBrakeHold228(recipe))
        PrintNCG76DiagnosticLine("[EDM228] event=AUTO_HOLD_ARMED session=%llu armP=%u L=11 B=%u trigger=CENTER_E5_BRAKE_MOVING confirmation=M00_FRESH_START_REQUIRED resume=MANUAL_FRESH_START initialDwell=FULL_AFTER_START holdRetreat=0 pausedJog=0 discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            EDM173::ModeForArm(m_edm94.gapHistoryArm));
    return true;
}

bool NCManager::IsEDM173OrbitAllowedSameThread(const NCBlock& b) noexcept
{
    const auto& s=m_edm94;const auto& w=*m_edm173;const int pc=GetActiveDispatchPC();
    bool valid=(!m_edm237.authorized||(IsEDM237SequenceScopeCurrentSameThread()&&
        m_edm237.parentIndex<m_edm237.parentCount&&m_edm237.stageStarted&&m_edm237.stageRecipeBound&&!m_edm237.awaitNext&&
        E237NativeKind(m_edm237.kinds[m_edm237.parentIndex])&&s.armPC==m_edm237.armPC[m_edm237.parentIndex]&&
        s.session==m_edm237.currentSession))&&(!m_edm234.authorized||(IsEDM234SequenceScopeCurrentSameThread()&&
        s.armPC==m_edm234.firstArmPC+(m_edm234.program==236U?static_cast<int>(m_edm234.parentIndex*3U):(m_edm234.secondStarted?3:0))&&
        s.gapHistoryArm==294U&&(m_edm234.program!=236U||(m_edm234.stageStarted&&m_edm234.stageRecipeBound&&
            !m_edm234.awaitNext&&m_edm234.kinds[m_edm234.parentIndex]==2U&&s.session==m_edm234.currentSession))))&&
        (!EDM130::IsG76ParentSequence233(m_edm130.recipe)||
        (IsEDM233SequenceScopeCurrentSameThread()&&s.armPC==m_edm233.firstArmPC+(m_edm233.secondStarted?3:0)))&&
        IsEDM173OrbitBlockShapeValid(b)&&s.latched&&EDM173::IsArm(s.gapHistoryArm)&&
        b.val('P')==static_cast<double>(EDM130::G76PlanMode(m_edm130.recipe))&&
        E181EffectiveOrbitMode(b)==EDM173::ModeForArm(s.gapHistoryArm)&&
        E181EffectiveOrbitMode(w.authored)==EDM173::ModeForArm(s.gapHistoryArm)&&
        EDM173::RecipeMatchesArm(s.gapHistoryArm,m_edm130.recipe)&&s.ready&&s.everArmed&&
        !s.armPending&&!s.cancelled&&!s.resetSeen&&!s.releaseRequested&&!s.finishPending&&!s.complete&&!s.rtFaultReported&&
        s.moves==1U&&s.next==1U&&pc==s.pc[0U]&&s.run==m_pathCoreLiveBookkeeping.currentRunToken&&
        s.cache==GetBaseProgramCache().GetGeneration()&&s.lease.Matches(m_programMotionLease)&&w.prepared&&!w.published&&
        m_state==NCState::RUN&&m_mode==NCOperationMode::MEMORY&&!Close_System_Com_flag&&m_externalReadyInterlock&&
        !m_isSingleBlockEnabled&&!m_isG66Active&&m_macroStack.empty()&&!Homing.IsActive()&&!AlarmManager::GetInstance().HasAlarm()&&
        m_motion.IsMotionOwnerLeaseCurrent(s.lease)&&!m_motion.HasPendingSafetyOrRecoveryRequests()&&
        !m_g76Orbit->active&&!m_pathFeed.pending&&!m_pathArc.pending&&!m_pathReplay.pending&&!m_pathHold.armed&&!m_pathHold.bound&&
        IsG76OrbitFrameCurrentSameThread(true,true)&&IsPathCoreLiveNativeConfigCurrentSameThread();
    valid=valid&&(!EDM130::IsG76SharedShort274(m_edm130.recipe)||CoordSys.GetTranslationSnapshot().workMode==169);
    valid=valid&&(!EDM130::IsG76FixedRadiusWork267(m_edm130.recipe)||
        E267FixedRadiusWorkFrame(CoordSys.GetTranslationSnapshot()));
    const auto* authored=GetBaseProgramCache().TryGetLine(pc);
    const auto* arm=GetBaseProgramCache().TryGetLine(s.armPC);
    valid=valid&&arm&&E251SquareSourcePair(arm->parsedBlock,b);
    valid=valid&&authored&&(w.coordinateRow.source.expressionMask!=0U?
        MatchesEDM179CoordinatesSameThread(authored->parsedBlock,b,pc):E180LiteralAngleSourceMatches(authored->parsedBlock,b));
    for(char word='A';word<='Z'&&valid;++word)
        valid=b.has(word)==w.authored.has(word)&&(!b.has(word)||EDM109::SameDouble(b.val(word),w.authored.val(word)));
    return valid;
}

bool NCManager::GuardEDM173BlockSameThread(const NCBlock& b,int pc,int line)
{
    auto& s=m_edm94;const auto* source=GetBaseProgramCache().TryGetLine(pc);
    const bool coordinateRow=m_edm173->coordinateRow.source.expressionMask!=0U&&pc==s.pc[0U];
    if(!source||!(coordinateRow?MatchesEDM179CoordinatesSameThread(source->parsedBlock,b,pc):E180LiteralAngleSourceMatches(source->parsedBlock,b)))
    {RejectEDM94SameThread(coordinateRow?"EDM179_COORDINATE_SCOPE":"EDM173_LITERAL_SCOPE",line);return false;}
    if(b.isEmpty) return true;
    bool valid=(!EDM130::IsG76ParentSequence233(m_edm130.recipe)||
        (IsEDM233SequenceScopeCurrentSameThread()&&s.armPC==m_edm233.firstArmPC+(m_edm233.secondStarted?3:0)))&&
        s.latched&&EDM173::IsArm(s.gapHistoryArm)&&!s.cancelled&&!s.armPending&&!s.releaseRequested&&
        s.moves==1U&&s.next<3U&&s.run==m_pathCoreLiveBookkeeping.currentRunToken&&s.cache==GetBaseProgramCache().GetGeneration()&&
        s.lease.Matches(m_programMotionLease)&&m_motion.IsMotionOwnerLeaseCurrent(s.lease)&&pc==s.pc[s.next]&&(s.next<2U?s.ready:s.complete);
    if(valid&&s.next==0U) valid=IsEDM173OrbitBlockShapeValid(b)&&
        b.val('P')==static_cast<double>(EDM130::G76PlanMode(m_edm130.recipe))&&
        E181EffectiveOrbitMode(b)==EDM173::ModeForArm(s.gapHistoryArm)&&m_edm173->prepared&&!m_edm173->published;
    else if(valid&&s.next==1U) valid=IsEDM173BracketShapeValid(b)&&
        b.val('P')==static_cast<double>(EDM173::FinishForArm(s.gapHistoryArm))&&m_edm173->completed&&!m_g76Orbit->active;
    else if(valid) valid=s.complete&&s.terminalLogged&&b.gCount==0&&b.mCount==1&&b.mCode[0]==30;
    if(!valid){RejectEDM94SameThread("EDM173_BLOCK_ORDER_OR_COMPLETION",line);return false;}
    ++s.next;return true;
}

// Terminal-only admission after the exact finish wait and outer source guard.
// This grants no motion and does not release the RT owner: the existing M30
// finalizer still performs the ordinary completion/release transaction.
bool NCManager::IsEDM173TerminalBlockAllowedSameThread(const NCBlock& b) noexcept
{
    const auto& s=m_edm94;const int pc=GetActiveDispatchPC();
    if(m_edm237.authorized&&(!IsEDM237SequenceScopeCurrentSameThread()||
        !IsEDM237FinalStateSameThread()||!E237NativeKind(m_edm237.kinds[m_edm237.parentIndex])))return false;
    if(m_edm234.authorized&&m_edm234.program==236U)
    {
        const auto& q=m_edm234;
        if(!IsEDM236SequenceScopeCurrentSameThread()||q.parentIndex!=3U||q.retiredMask!=7U||!q.stageRecipeBound||
            !q.stageStarted||q.awaitNext||q.releasePending||q.kinds[3U]!=2U||s.armPC!=q.firstArmPC+9||
            s.session!=q.currentSession||s.session<=q.previousSession)return false;
    }
    else if(m_edm234.authorized&&(!IsEDM234SequenceScopeCurrentSameThread()||!m_edm234.firstReleased||
        !m_edm234.secondStarted||m_edm234.releasePending||s.armPC!=m_edm234.firstArmPC+3||s.session<=m_edm234.firstSession))return false;
    if(EDM130::IsG76ParentSequence233(m_edm130.recipe)&&
        (!IsEDM233SequenceScopeCurrentSameThread()||!m_edm233.firstReleased||!m_edm233.secondStarted||
            m_edm233.releasePending||s.armPC!=m_edm233.firstArmPC+3||s.session<=m_edm233.firstSession))return false;
    if(!s.latched||!EDM173::IsArm(s.gapHistoryArm)||!s.everArmed||!s.complete||!s.terminalLogged||
        s.moves!=1U||s.next!=3U||pc!=s.pc[2U]||s.cancelled||s.resetSeen||s.resetCommitted||
        s.armPending||s.finishPending||s.releaseRequested||s.completionReleaseRequested||s.rtFaultReported||
        !s.run||s.run!=m_pathCoreLiveBookkeeping.currentRunToken||!s.cache||s.cache!=GetBaseProgramCache().GetGeneration()||
        !s.lease.Matches(m_programMotionLease)||!m_motion.IsMotionOwnerLeaseCurrent(s.lease)||
        !m_edm173||!m_edm173->completed||!m_g76Orbit||m_g76Orbit->active||m_g76Orbit->mixed||
        m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||Close_System_Com_flag||!m_externalReadyInterlock||
        IsFeedHoldActive()||m_isSingleBlockEnabled||m_isG66Active||!m_macroStack.empty()||Homing.IsActive()||
        AlarmManager::GetInstance().HasAlarm()||m_motion.HasPendingSafetyOrRecoveryRequests()||
        m_pathFeed.pending||m_pathArc.pending||m_pathReplay.pending||m_pathHold.armed||m_pathHold.bound||
        !CoordSys.IsTranslationRunFrozen()||!IsG76OrbitFrameCurrentSameThread(true,true)||
        !IsFixedTranslationTravelCurrentSameThread()||!IsPathCoreLiveNativeConfigCurrentSameThread()||
        !SameNCTranslationSnapshot(m_edm173->plan.nominal.translation,CoordSys.GetTranslationSnapshot())||
        !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot())||
        !m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||
        m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()) return false;
    const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if(!source||!E94Matches(source->parsedBlock,b)||b.isEmpty||b.hasG||b.gCode!=-1||b.gCount!=0||
        b.mCount!=1||b.mCode[0]!=30||b.gotoTarget!=-1) return false;
    const auto& parsed=source->parsedBlock;
    if(!EDM77::Detail::OnlyAddresses(parsed,"N")||parsed.gCount!=0||parsed.mCount!=1||
        !EDM77::Detail::EqualLiteral(parsed.mExpressions[0U],30U)||
        parsed.assignment.prefix||!parsed.assignment.indexExpression.empty()||!parsed.assignment.valueExpression.empty()||
        parsed.gotoStatement.conditional||!parsed.gotoStatement.conditionExpression.empty()||
        !parsed.gotoStatement.targetExpression.empty()) return false;
    for(const auto& expression:parsed.gExpressions) if(!expression.empty()) return false;
    for(std::size_t i=1U;i<parsed.mExpressions.size();++i) if(!parsed.mExpressions[i].empty()) return false;
    for(int code:b.gCodes) if(code!=0) return false;
    for(std::size_t i=1U;i<sizeof(b.mCode)/sizeof(b.mCode[0U]);++i) if(b.mCode[i]!=-1) return false;
    for(char word='A';word<='Z';++word)
        if(!parsed.has(word)&&(!parsed.expression(word).empty()||!EDM109::SameDouble(b.val(word),0.0))) return false;
    return true;
}

WaitConditionFunc NCManager::StartEDM173SameThread(const NCBlock& b)
{
    const int pc=GetActiveDispatchPC();const auto* authored=GetBaseProgramCache().TryGetLine(pc);
    if(!authored||!E94Matches(authored->parsedBlock,b)||!IsEDM173BracketBlockSameThread(b))
    {RejectEDM94SameThread("EDM173_BRACKET_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    if(EDM173::IsFinish(static_cast<unsigned>(b.val('P'))))
    {
        auto& s=m_edm94;
        if(s.next!=2U||!m_edm173->completed||!E156FinishTargetMatches(s,m_edm130.recipe,s.feedback,m_edm114.retained)||
            !m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||m_g76Orbit->active)
        {RejectEDM94SameThread("EDM173_FINAL_PARENT_OR_TARGET",m_gapDryRun.sourceLine);return nullptr;}
        if(EDM130::IsG76ParentSequence233(m_edm130.recipe))
        {
            if(!IsEDM233SequenceScopeCurrentSameThread()||s.armPC!=m_edm233.firstArmPC+(m_edm233.secondStarted?3:0))
            {RejectEDM94SameThread("EDM233_FINISH_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
            if(!m_edm233.secondStarted)m_edm233.firstFinishDispatch=m_currentExecutingBlockDispatchId;
        }
        if(m_edm237.authorized)
        {
            if(!IsEDM237SequenceScopeCurrentSameThread()||!m_edm237.motionDispatch||
                m_edm237.currentRowBound!=E237RowsForKind(m_edm237.kinds[m_edm237.parentIndex]))
            {RejectEDM94SameThread("EDM237_FINISH_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
            m_edm237.currentFinishDispatch=m_currentExecutingBlockDispatchId;
            if(!m_edm237.parentIndex)m_edm237.firstFinishDispatch=m_currentExecutingBlockDispatchId;
        }
        if(m_edm234.authorized)
        {
            if(!IsEDM234SequenceScopeCurrentSameThread()||!m_edm234.motionDispatch)
            {RejectEDM94SameThread("EDM234_ORBIT_FINISH_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
            m_edm234.currentFinishDispatch=m_currentExecutingBlockDispatchId;
            if(m_edm234.program!=236U&&!m_edm234.secondStarted)m_edm234.firstFinishDispatch=m_currentExecutingBlockDispatchId;
        }
        s.ready=false;s.finishPending=true;s.polls=0U;m_motion.RequestEDM94Finish(s.session);
        RtPrintf("[EDM173] event=FINISH_REQUEST session=%llu children=%u E5completed=%u await=PHYSICAL_ZERO_200 discharge=0\n",
            static_cast<unsigned long long>(s.session),EDM173::CountForRecipe(m_edm130.recipe),EDM130::G76EventLimit(m_edm130.recipe));
        return [](NCManager* n){return n->WaitEDM173SameThread();};
    }
    if(m_edm94.latched||!m_pathFeed.armed||m_pathFeed.run!=m_pathCoreLiveBookkeeping.currentRunToken||
        m_pathFeed.cache!=GetBaseProgramCache().GetGeneration()||!PreflightEDM173ProgramSameThread(pc)||
        !m_programMotionLease.IsValid()||m_programMotionLease.owner!=MotionOwner::AUTO||
        !m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||
        m_motion.GetAxisCommandMailboxDepth()||m_gapDryRun.active||m_gapPath.active||m_gapWindow.active||
        m_pathFeed.pending||m_pathArc.pending||m_pathReplay.pending||m_pathHold.armed||m_pathHold.bound||
        m_cncFeed.active||m_cncFeed.selected||m_cncFeed.count||m_cutterLine.leadOutRequired||
        m_edm94Serial==(std::numeric_limits<std::uint64_t>::max)()||!FreezeEDM173TranslationSameThread())
    {RejectEDM94SameThread("EDM173_ARM_NOT_READY",m_gapDryRun.sourceLine);return nullptr;}
    const bool historySequence=b.has('L')&&(b.val('L')==237.0||b.val('L')==238.0||b.val('L')==239.0||b.val('L')==240.0||b.val('L')==241.0||b.val('L')==242.0||b.val('L')==243.0||b.val('L')==244.0||b.val('L')==245.0||b.val('L')==246.0||b.val('L')==247.0||b.val('L')==248.0);
    if(historySequence&&!BeginEDM237SequenceSameThread(pc))
    {RejectEDM94SameThread("EDM237_SEQUENCE_ADMISSION",m_gapDryRun.sourceLine);return nullptr;}
    const bool mixedSequence=b.has('L')&&(b.val('L')==234.0||b.val('L')==235.0||b.val('L')==236.0);
    if(mixedSequence&&!BeginEDM234SequenceSameThread(pc))
    {RejectEDM94SameThread("EDM234_SEQUENCE_ADMISSION",m_gapDryRun.sourceLine);return nullptr;}
    const bool parentSequence=b.has('L')&&b.val('L')==233.0;
    if(parentSequence&&!BeginEDM233SequenceSameThread(pc))
    {RejectEDM94SameThread("EDM233_SEQUENCE_ADMISSION",m_gapDryRun.sourceLine);return nullptr;}
    const auto inspected=GetBaseProgramCache().GetGeneration();m_edm94=EDM94NCState{};auto& s=m_edm94;
    s.program=true;s.inspectedCache=inspected;s.gapHistoryArm=static_cast<unsigned>(b.val('P'));s.gapWorkTime=true;s.moves=1U;s.authoredGCode[0U]=76U;s.feed[0U]=0.12;
    s.pc[0U]=pc+1;s.pc[1U]=pc+2;s.pc[2U]=pc+3;s.armPC=pc;s.armDispatch=m_currentExecutingBlockDispatchId;
    s.session=++m_edm94Serial;s.run=m_pathCoreLiveBookkeeping.currentRunToken;s.cache=inspected;
    s.armEpoch=m_motion.GetCurrentExecutionEpoch();s.lease=m_programMotionLease;s.gapTuning={8.0,1.0,1.0};
    if(!EDM107::ParseFlushProfile(10.0,50.0,100.0,s.gapFlushParameters))
    {RejectEDM94SameThread(mixedSequence?"EDM234_FLUSH_PROFILE_BIND":parentSequence?"EDM233_FLUSH_PROFILE_BIND":"EDM173_RECIPE_OR_CONFIRMATION",m_gapDryRun.sourceLine);return nullptr;}
    if(!BindEDM173RecipeSameThread())
    {RejectEDM94SameThread(mixedSequence?"EDM234_RECIPE_BIND":parentSequence?"EDM233_RECIPE_BIND":"EDM173_RECIPE_OR_CONFIRMATION",m_gapDryRun.sourceLine);return nullptr;}
    if(!ConsumeEDM149ConfirmationSameThread(pc))
    {RejectEDM94SameThread(mixedSequence?"EDM234_SEQUENCE_CONFIRMATION":parentSequence?"EDM233_SEQUENCE_CONFIRMATION":"EDM173_RECIPE_OR_CONFIRMATION",m_gapDryRun.sourceLine);return nullptr;}
    if(parentSequence)
    {
        if(!m_edm233.secondStarted)m_edm233.firstSession=s.session;
        else if(s.session<=m_edm233.firstSession)
        {RejectEDM94SameThread("EDM233_FRESH_SESSION_REQUIRED",m_gapDryRun.sourceLine);return nullptr;}
        PrintNCG76DiagnosticLine("[EDM233] event=PARENT_ARM_REQUEST parent=%u session=%llu firstSession=%llu run=%llu cache=%llu E5policy=FRESH_PER_PARENT children=25 sameAUTO=1 discharge=0\n",
            m_edm233.secondStarted?2U:1U,static_cast<unsigned long long>(s.session),
            static_cast<unsigned long long>(m_edm233.firstSession),static_cast<unsigned long long>(s.run),static_cast<unsigned long long>(s.cache));
    }
    auto& w=*m_edm173;
    if(w.coordinateRow.source.expressionMask!=0U&&!FreezeEDM179CoordinatesSameThread())
    {RejectEDM94SameThread("EDM179_COORDINATE_FREEZE",m_gapDryRun.sourceLine);return nullptr;}
    NCG76::PlanInput input{};NCG76::Point3 authoredXYZ{};unsigned authoredMask=0U;
    for(unsigned axis=0U;axis<3U;++axis)if(w.authored.has("XYZ"[axis]))
    {authoredXYZ[axis]=w.authored.val("XYZ"[axis]);authoredMask|=1U<<axis;}
    NCG76::Point8 start{};for(unsigned axis=0U;axis<8U;++axis)start[axis]=CoordSys.commandedMCS[axis];
    const auto frozen=CoordSys.GetTranslationSnapshot();
    if(!EDM173::BuildOrbitInput(frozen,start,authoredXYZ,authoredMask,w.authored.val('R'),w.authored.val('F'),input,
        w.authored.has('K')?w.authored.val('K'):0.0,EDM173::UsesTargetAxis(m_edm130.recipe),EDM173::IsCenterRecipe(m_edm130.recipe),
        (w.authored.val('P')==6.0||w.authored.val('P')==16.0||w.authored.val('P')==26.0||w.authored.val('P')==7.0||w.authored.val('P')==17.0||w.authored.val('P')==27.0)?static_cast<unsigned>(w.authored.val('Q')):0U))
    {RejectEDM94SameThread("EDM178_COORDINATE_INPUT",m_gapDryRun.sourceLine);return nullptr;}
    NCG76::ProgramPlan& program=m_g76Orbit->program;
    if(!NCG76::BuildMode(static_cast<std::uint32_t>(w.authored.val('P')),input,program)||!NCG76::BuildNative(program,frozen,start,w.nominal,true,true,EDM173::PreservesG90StationaryZ(m_edm130.recipe))||w.nominal.count!=EDM173::CountForRecipe(m_edm130.recipe))
    {RejectEDM94SameThread("EDM173_NOMINAL_PLAN",m_gapDryRun.sourceLine);return nullptr;}
    if(EDM173::IsB0Recipe(m_edm130.recipe))
    {
        const unsigned movementMask=EDM173::NominalMovementMask(w.nominal);
        if(!movementMask)
        {RejectEDM94SameThread("EDM193_NOMINAL_MOVEMENT",m_gapDryRun.sourceLine);return nullptr;}
        m_edm130.recipe.planMovementMask=static_cast<std::uint16_t>(movementMask);
        if(!EDM130::ValidHistoryRecipeBinding(EDM173::HistoryFixture,m_edm130.recipe))
        {RejectEDM94SameThread("EDM193_MOVEMENT_RECIPE",m_gapDryRun.sourceLine);return nullptr;}
        PrintNCG76DiagnosticLine("[EDM193] event=B0_MOVEMENT_FROZEN session=%llu parentPC=%d requiredMask=%u nominalChildren=25 ZcommandFixed=%u source=COMPENSATED_NATIVE_POINTS structuralMask=7 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],movementMask,movementMask==3U?1U:0U);
        if(!input.targetAxis&&frozen.distanceMode==90&&input.start[2U]==input.end[2U])
            PrintNCG76DiagnosticLine("[EDM194] event=G90_Z_ANCHOR session=%llu parentPC=%d frameGeneration=%llu startNativeZ=%.17g finalNativeZ=%.17g authoredZStationary=1 points=25 rule=EXACT_PROGRAM_Z discharge=0\n",
                static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
                start[2U],w.nominal.pointsMCS[EDM173::CountForRecipe(m_edm130.recipe)-1U][2U]);
    }
    if(EDM173::IsB2Recipe(m_edm130.recipe))
    {
        const unsigned movementMask=EDM173::NominalMovementMask(w.nominal);
        if(!movementMask)
        {RejectEDM94SameThread("EDM197_NOMINAL_MOVEMENT",m_gapDryRun.sourceLine);return nullptr;}
        m_edm130.recipe.planMovementMask=static_cast<std::uint16_t>(movementMask);
        if(!EDM130::ValidHistoryRecipeBinding(EDM173::HistoryFixture,m_edm130.recipe))
        {RejectEDM94SameThread("EDM197_MOVEMENT_RECIPE",m_gapDryRun.sourceLine);return nullptr;}
        PrintNCG76DiagnosticLine("[EDM197] event=B2_MOVEMENT_FROZEN session=%llu parentPC=%d requiredMask=%u nominalChildren=%u ZcommandFixed=%u pathHeightMm=0.001 source=COMPENSATED_NATIVE_POINTS structuralMask=7 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],movementMask,EDM173::CountForRecipe(m_edm130.recipe),movementMask==3U?1U:0U);
        PrintNCG76DiagnosticLine("[EDM201] event=TARGET_AXIS_FROZEN session=%llu parentPC=%d frameGeneration=%llu mode=2 programDeltaX=%.17g programDeltaY=%.17g programDeltaZ=%.17g K=%.17g plane=PERPENDICULAR_XYZ_TARGET source=RESOLVED_PROGRAM_TARGET requiredMask=%u nominalChildren=%u discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            input.end[0U]-input.start[0U],input.end[1U]-input.start[1U],input.end[2U]-input.start[2U],
            input.shapeAngleDeg,movementMask,EDM173::CountForRecipe(m_edm130.recipe));
    }
    if(input.targetAxis&&!EDM173::IsB2Recipe(m_edm130.recipe)&&!EDM173::IsCenterRecipe(m_edm130.recipe))
        PrintNCG76DiagnosticLine("[EDM202] event=TARGET_AXIS_FROZEN session=%llu parentPC=%d frameGeneration=%llu mode=%u programDeltaX=%.17g programDeltaY=%.17g programDeltaZ=%.17g K=%.17g plane=PERPENDICULAR_XYZ_TARGET source=RESOLVED_PROGRAM_TARGET requiredMask=%u nominalChildren=%u flushAxis=%s discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            E181EffectiveOrbitMode(w.authored),input.end[0U]-input.start[0U],input.end[1U]-input.start[1U],
            input.end[2U]-input.start[2U],input.shapeAngleDeg,static_cast<unsigned>(m_edm130.recipe.planMovementMask),EDM173::CountForRecipe(m_edm130.recipe),
            EDM173::IsB0Recipe(m_edm130.recipe)?"NATIVE_TANGENT_REVERSE":"NATIVE_PLUS_Z");
    if(EDM173::IsCenterRecipe(m_edm130.recipe))
        PrintNCG76DiagnosticLine("[EDM204] event=TARGET_AXIS_FROZEN session=%llu parentPC=%d frameGeneration=%llu mode=%u programDeltaX=%.17g programDeltaY=%.17g programDeltaZ=%.17g K=%.17g plane=PERPENDICULAR_XYZ_TARGET source=RESOLVED_PROGRAM_TARGET nominalChildren=%u center=XYZ flushAxis=NEGATIVE_COMPENSATED_XYZ_TARGET discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            E181EffectiveOrbitMode(w.authored),input.end[0U]-input.start[0U],input.end[1U]-input.start[1U],
            input.end[2U]-input.start[2U],input.shapeAngleDeg,EDM173::CountForRecipe(m_edm130.recipe));
    if(!EDM173::UsesNominalMovementMask(m_edm130.recipe)&&EDM173::PreservesG90StationaryZ(m_edm130.recipe)&&
        frozen.distanceMode==90&&input.start[2U]==input.end[2U])
        PrintNCG76DiagnosticLine("[EDM196] event=G90_Z_ANCHOR session=%llu parentPC=%d frameGeneration=%llu mode=%u startNativeZ=%.17g finalNativeZ=%.17g authoredZStationary=1 nominalPoints=25 centerAnchored=%u requiredMovementMask=%u flushNativeZ=%u rule=EXACT_PROGRAM_Z discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            E181EffectiveOrbitMode(w.authored),start[2U],w.nominal.pointsMCS[EDM173::CountForRecipe(m_edm130.recipe)-1U][2U],EDM173::IsCenterRecipe(m_edm130.recipe)?1U:0U,
            static_cast<unsigned>(m_edm130.recipe.planMovementMask),EDM173::IsCenterRecipe(m_edm130.recipe)?0U:1U);
    PrintNCG76DiagnosticLine("[EDM181] event=B_MODE_FROZEN session=%llu parentPC=%d source=%s authoredBPresent=%u mode=%u arm=P%u modalUnchanged=1 scope=G76_CURRENT_BLOCK discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc[0U],w.authored.has('B')?"EXPLICIT":"DEFAULT",
        w.authored.has('B')?1U:0U,E181EffectiveOrbitMode(w.authored),s.gapHistoryArm);
    PrintNCG76DiagnosticLine("[EDM182] event=TOOL_LENGTH_FROZEN session=%llu parentPC=%d frameGeneration=%llu mode=G%d H=%d rawX=%.12g rawY=%.12g rawZ=%.12g effectiveX=%.12g effectiveY=%.12g effectiveZ=%.12g source=EXISTING_H_ROW tableWrite=0 scope=FIXED_XYZ discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
        frozen.toolLengthMode,frozen.toolHCode,frozen.toolOffsetMM[0U],frozen.toolOffsetMM[1U],frozen.toolOffsetMM[2U],
        NCTranslationToolOffsetMM(frozen,0U),NCTranslationToolOffsetMM(frozen,1U),NCTranslationToolOffsetMM(frozen,2U));
    if(frozen.workMode==168&&frozen.workOffset[3U]!=0.0)
        PrintNCG76DiagnosticLine("[EDM184] event=WORK_XY_FROZEN session=%llu parentPC=%d frameGeneration=%llu mode=G%d W=%d X=%.12g Y=%.12g Z=%.12g yawDeg=%.12g centerMCSX=%.12g centerMCSY=%.12g source=EXISTING_W_ROW tableWrite=0 scope=FIXED_XY discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            frozen.workMode,frozen.workWCode,frozen.workOffset[0U],frozen.workOffset[1U],frozen.workOffset[2U],
            frozen.workOffset[3U],frozen.workRotationCenterMM[0U],frozen.workRotationCenterMM[1U]);
    else
        PrintNCG76DiagnosticLine("[EDM183] event=WORK_FROZEN session=%llu parentPC=%d frameGeneration=%llu mode=G%d W=%d X=%.12g Y=%.12g Z=%.12g XY=%.12g XZ=%.12g YZ=%.12g source=EXISTING_W_ROW tableWrite=0 scope=FIXED_XYZ discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            frozen.workMode,frozen.workWCode,frozen.workOffset[0U],frozen.workOffset[1U],frozen.workOffset[2U],
            frozen.workOffset[3U],frozen.workOffset[4U],frozen.workOffset[5U]);
    if((frozen.mirrorMask&4U)!=0U)
        PrintNCG76DiagnosticLine("[EDM188] event=MIRROR_XYZ_FROZEN session=%llu parentPC=%d frameGeneration=%llu mask=%u centerProgramX=%.12g centerProgramY=%.12g centerProgramZ=%.12g source=G151 frozen=1 scale=%.12g Zmirrored=1 flushHeightNative=1 scope=FIXED_XYZ discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            static_cast<unsigned>(frozen.mirrorMask),frozen.mirrorCenterMM[0U],frozen.mirrorCenterMM[1U],frozen.mirrorCenterMM[2U],
            frozen.scalingMode==51?frozen.scalingFactor:1.0);
    else if(frozen.mirrorMask!=0U)
        PrintNCG76DiagnosticLine("[EDM185] event=MIRROR_XY_FROZEN session=%llu parentPC=%d frameGeneration=%llu mask=%u centerProgramX=%.12g centerProgramY=%.12g source=G151 frozen=1 scale=%.12g Zmirrored=0 scope=FIXED_XY discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            static_cast<unsigned>(frozen.mirrorMask),frozen.mirrorCenterMM[0U],frozen.mirrorCenterMM[1U],
            frozen.scalingMode==51?frozen.scalingFactor:1.0);
    if(frozen.scalingMode==51)
        PrintNCG76DiagnosticLine("[EDM186] event=SCALE_FROZEN session=%llu parentPC=%d frameGeneration=%llu factor=%.12g centerProgramX=%.12g centerProgramY=%.12g centerProgramZ=%.12g radiusNativeMm=%.12g source=G51 frozen=1 flushHeightNative=1 scope=UNIFORM_XYZ discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
            frozen.scalingFactor,frozen.scalingCenterMM[0U],frozen.scalingCenterMM[1U],frozen.scalingCenterMM[2U],
            input.radiusMm*frozen.scalingFactor);
    PrintNCG76DiagnosticLine("[EDM180] event=ANGLE_FROZEN session=%llu parentPC=%d frameGeneration=%llu localAngleDeg=%.12g frameRotation=G%d frameG68AngleDeg=%.12g centerLine=UNCHANGED order=LOCAL_THEN_FRAME discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
        input.shapeAngleDeg,frozen.rotationMode,frozen.rotationMode==68?frozen.rotationAngleDeg:0.0);
    PrintNCG76DiagnosticLine("[EDM178] event=COORDINATES_FROZEN session=%llu parentPC=%d frameGeneration=%llu distance=G%d rotation=G%d angleDeg=%.12g mask=%u programStartX=%.12g programStartY=%.12g programStartZ=%.12g programEndX=%.12g programEndY=%.12g programEndZ=%.12g nativeFinalX=%.12g nativeFinalY=%.12g nativeFinalZ=%.12g discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(frozen.generation),
        frozen.distanceMode,frozen.rotationMode,frozen.rotationAngleDeg,authoredMask,
        input.start[0U],input.start[1U],input.start[2U],input.end[0U],input.end[1U],input.end[2U],
        w.nominal.pointsMCS[EDM173::CountForRecipe(m_edm130.recipe)-1U][0U],w.nominal.pointsMCS[EDM173::CountForRecipe(m_edm130.recipe)-1U][1U],w.nominal.pointsMCS[EDM173::CountForRecipe(m_edm130.recipe)-1U][2U]);
    if(w.coordinateRow.valid)
        PrintNCG76DiagnosticLine("[EDM179] event=TARGETS_FROZEN session=%llu parentPC=%d cache=%llu frameGeneration=%llu distance=G90 expressionMask=%u programStartX=%.12g programStartY=%.12g programStartZ=%.12g programEndX=%.12g programEndY=%.12g programEndZ=%.12g source=COMMANDED_WCS resolved=ACTUAL_NC_RESOLVER discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc[0U],static_cast<unsigned long long>(s.cache),
            static_cast<unsigned long long>(frozen.generation),w.coordinateRow.source.expressionMask,
            input.start[0U],input.start[1U],input.start[2U],input.end[0U],input.end[1U],input.end[2U]);
    w.loggedTriggers=w.loggedCompletions=0ULL;w.loggedPhase=0U;w.retained=EDM173::Feedback{};
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("EDM173_ARM");
    if(EDM173::IsArm(s.gapHistoryArm)) {
        const EDM192Arm::Scope admissionScope{s.session,static_cast<std::uint64_t>(s.armEpoch),
            static_cast<std::uint64_t>(m_motion.GetCurrentExecutionEpoch()),
            static_cast<std::uint32_t>(s.lease.owner),static_cast<std::uint32_t>(s.lease.generation),
            EDM173::HistoryFixture,m_edm130.recipe.commissioningProfile};
        PrimeEDM192AdmissionObserver(admissionScope);
    }
    const auto* armSource=GetBaseProgramCache().TryGetLine(s.armPC);
    const bool rejectBeforeArm=b.has('L')&&b.val('L')==216.0;
    if(!armSource||rejectBeforeArm!=E216RejectBeforeArm(armSource->parsedBlock,s.gapHistoryArm))
    {RejectEDM94SameThread("EDM216_PROBE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    if((b.has('L')&&b.val('L')==1.0)!=E217CenterNominalHold(armSource->parsedBlock,s.gapHistoryArm))
    {RejectEDM94SameThread("EDM217_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerFlushHold=E218CenterFlushHold(armSource->parsedBlock,s.gapHistoryArm);
    if((b.has('L')&&b.val('L')==2.0)!=centerFlushHold||
        centerFlushHold!=EDM130::IsG76CenterFlushHold218(m_edm130.recipe))
    {RejectEDM94SameThread("EDM218_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerReturnHold=E219CenterReturnHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==3.0)!=centerReturnHold||
        centerReturnHold!=EDM130::IsG76CenterReturnHold219(m_edm130.recipe))
    {RejectEDM94SameThread("EDM219_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerAnchorHold=E220CenterAnchorHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==4.0)!=centerAnchorHold||
        centerAnchorHold!=EDM130::IsG76CenterAnchorHold220(m_edm130.recipe))
    {RejectEDM94SameThread("EDM220_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerStartHold=E221CenterStartHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==5.0)!=centerStartHold||
        centerStartHold!=EDM130::IsG76CenterStartHold221(m_edm130.recipe))
    {RejectEDM94SameThread("EDM221_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerTopDwellHold=E222CenterTopDwellHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==6.0)!=centerTopDwellHold||
        centerTopDwellHold!=EDM130::IsG76CenterTopDwellHold222(m_edm130.recipe))
    {RejectEDM94SameThread("EDM223_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerDwellHold=E224CenterDwellHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==7.0)!=centerDwellHold||
        centerDwellHold!=EDM130::IsG76CenterDwellHold224(m_edm130.recipe))
    {RejectEDM94SameThread("EDM224_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerReturnDwellHold=E225CenterReturnDwellHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==8.0)!=centerReturnDwellHold||
        centerReturnDwellHold!=EDM130::IsG76CenterReturnDwellHold225(m_edm130.recipe))
    {RejectEDM94SameThread("EDM225_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerFinalDwellHold=E226CenterFinalDwellHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==9.0)!=centerFinalDwellHold||
        centerFinalDwellHold!=EDM130::IsG76CenterFinalDwellHold226(m_edm130.recipe))
    {RejectEDM94SameThread("EDM226_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerInitialDwellHold=E227CenterInitialDwellHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==10.0)!=centerInitialDwellHold||
        centerInitialDwellHold!=EDM130::IsG76CenterInitialDwellHold227(m_edm130.recipe))
    {RejectEDM94SameThread("EDM227_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerBrakeHold=E228CenterBrakeHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==11.0)!=centerBrakeHold||
        centerBrakeHold!=EDM130::IsG76CenterBrakeHold228(m_edm130.recipe))
    {RejectEDM94SameThread("EDM228_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerRepeat=E230CenterRepeat(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==230.0)!=centerRepeat||
        centerRepeat!=EDM130::IsG76CenterRepeat230(m_edm130.recipe))
    {RejectEDM94SameThread("EDM230_REPEAT_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerSecondFlushHold=E231CenterSecondFlushHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==231.0)!=centerSecondFlushHold||
        centerSecondFlushHold!=EDM130::IsG76CenterSecondFlushHold231(m_edm130.recipe))
    {RejectEDM94SameThread("EDM231_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerSecondReturnHold=E232CenterSecondReturnHold(armSource->parsedBlock,s.gapHistoryArm);
    if(((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            b.has('L')&&b.val('L')==232.0)!=centerSecondReturnHold||
        centerSecondReturnHold!=EDM130::IsG76CenterSecondReturnHold232(m_edm130.recipe))
    {RejectEDM94SameThread("EDM232_HOLD_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    if(parentSequence!=E233CenterParentSequence(armSource->parsedBlock,s.gapHistoryArm)||
        parentSequence!=EDM130::IsG76ParentSequence233(m_edm130.recipe))
    {RejectEDM94SameThread("EDM233_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool centerSequence=b.has('L')&&b.val('L')==239.0;
    if(centerSequence!=(E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),239U))||
        centerSequence!=EDM130::IsG76CenterSequence239(m_edm130.recipe))
    {RejectEDM94SameThread("EDM239_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool mixedCenterSequence=b.has('L')&&b.val('L')==240.0;
    if(mixedCenterSequence!=(E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),240U))||
        mixedCenterSequence!=EDM130::IsG76CenterSequence240(m_edm130.recipe))
    {RejectEDM94SameThread("EDM240_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool allKindsCenter=(s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&b.has('L')&&b.val('L')==241.0;
    if(allKindsCenter!=((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),241U))||
        allKindsCenter!=EDM130::IsG76CenterSequence241(m_edm130.recipe))
    {RejectEDM94SameThread("EDM241_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool adjacentCenter=(s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&b.has('L')&&b.val('L')==242.0;
    if(adjacentCenter!=((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),242U))||
        adjacentCenter!=EDM130::IsG76CenterSequence242(m_edm130.recipe))
    {RejectEDM94SameThread("EDM242_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool twoBridgeCenter=(s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&b.has('L')&&b.val('L')==243.0;
    if(twoBridgeCenter!=((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),243U))||
        twoBridgeCenter!=EDM130::IsG76CenterSequence243(m_edm130.recipe))
    {RejectEDM94SameThread("EDM243_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool repeatedCenter=(s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&b.has('L')&&b.val('L')==244.0;
    if(repeatedCenter!=((s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP)&&
            E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),244U))||
        repeatedCenter!=EDM130::IsG76CenterSequence244(m_edm130.recipe))
    {RejectEDM94SameThread("EDM244_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool ordinaryRepeatProgram=b.has('L')&&b.val('L')==245.0;
    const bool ordinaryRepeatCenter=ordinaryRepeatProgram&&s.gapHistoryArm==EDM173::B3ArmP;
    if((ordinaryRepeatProgram&&s.gapHistoryArm!=EDM173::B2ArmP&&s.gapHistoryArm!=EDM173::B3ArmP)||
        ordinaryRepeatCenter!=(s.gapHistoryArm==EDM173::B3ArmP&&E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),245U))||
        ordinaryRepeatCenter!=EDM130::IsG76CenterSequence245(m_edm130.recipe))
    {RejectEDM94SameThread("EDM245_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool ordinaryB4Program=b.has('L')&&b.val('L')==246.0;
    const bool ordinaryB4Center=ordinaryB4Program&&s.gapHistoryArm==EDM173::B4ArmP;
    if((ordinaryB4Program&&s.gapHistoryArm!=EDM173::B2ArmP&&s.gapHistoryArm!=EDM173::B4ArmP)||
        ordinaryB4Center!=(s.gapHistoryArm==EDM173::B4ArmP&&E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),246U))||
        ordinaryB4Center!=EDM130::IsG76CenterSequence246(m_edm130.recipe))
    {RejectEDM94SameThread("EDM246_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool repeatedB2Program=b.has('L')&&b.val('L')==247.0;
    const bool repeatedB2Center=repeatedB2Program&&s.gapHistoryArm==EDM173::B3ArmP;
    if((repeatedB2Program&&s.gapHistoryArm!=EDM173::B2ArmP&&s.gapHistoryArm!=EDM173::B3ArmP)||
        repeatedB2Center!=(s.gapHistoryArm==EDM173::B3ArmP&&E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),247U))||
        repeatedB2Center!=EDM130::IsG76CenterSequence247(m_edm130.recipe))
    {RejectEDM94SameThread("EDM247_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool repeatedB2B4Program=b.has('L')&&b.val('L')==248.0;
    const bool repeatedB2B4Center=repeatedB2B4Program&&s.gapHistoryArm==EDM173::B4ArmP;
    if((repeatedB2B4Program&&s.gapHistoryArm!=EDM173::B2ArmP&&s.gapHistoryArm!=EDM173::B4ArmP)||
        repeatedB2B4Center!=(s.gapHistoryArm==EDM173::B4ArmP&&E237ArmSequence(armSource->parsedBlock,s.gapHistoryArm)&&
            EDM77::Detail::EqualLiteral(armSource->parsedBlock.expression('L'),248U))||
        repeatedB2B4Center!=EDM130::IsG76CenterSequence248(m_edm130.recipe))
    {RejectEDM94SameThread("EDM248_SEQUENCE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool squareB1Program=b.has('L')&&b.val('L')==250.0;
    if(squareB1Program!=E250SquareB1Arm(armSource->parsedBlock,s.gapHistoryArm)||
        squareB1Program!=EDM130::IsG76SquareB1_250(m_edm130.recipe)||
        !E251SquareSourcePair(armSource->parsedBlock,w.authored))
    {RejectEDM94SameThread("EDM250_SQUARE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool squareB2Program=b.has('L')&&b.val('L')==251.0;
    if(squareB2Program!=E251SquareB2Arm(armSource->parsedBlock,s.gapHistoryArm)||
        squareB2Program!=EDM130::IsG76SquareB2_251(m_edm130.recipe))
    {RejectEDM94SameThread("EDM251_SQUARE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool squareB3Program=b.has('L')&&b.val('L')==252.0;
    if(squareB3Program!=E252SquareB3Arm(armSource->parsedBlock,s.gapHistoryArm)||
        squareB3Program!=EDM130::IsG76SquareB3_252(m_edm130.recipe))
    {RejectEDM94SameThread("EDM252_SQUARE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool squareB4Program=b.has('L')&&b.val('L')==253.0;
    if(squareB4Program!=E253SquareB4Arm(armSource->parsedBlock,s.gapHistoryArm)||
        squareB4Program!=EDM130::IsG76SquareB4_253(m_edm130.recipe))
    {RejectEDM94SameThread("EDM253_SQUARE_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool diamondProgram=b.has('L')&&b.val('L')==255.0;
    if(diamondProgram!=E255DiamondB1Arm(armSource->parsedBlock,s.gapHistoryArm)||
        diamondProgram!=EDM130::IsG76DiamondB1_255(m_edm130.recipe))
    {RejectEDM94SameThread("EDM255_DIAMOND_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool diamondB2Program=b.has('L')&&b.val('L')==256.0;
    if(diamondB2Program!=E256DiamondB2Arm(armSource->parsedBlock,s.gapHistoryArm)||
        diamondB2Program!=EDM130::IsG76DiamondB2_256(m_edm130.recipe))
    {RejectEDM94SameThread("EDM256_DIAMOND_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool diamondB3Program=b.has('L')&&b.val('L')==257.0;
    if(diamondB3Program!=E257DiamondB3Arm(armSource->parsedBlock,s.gapHistoryArm)||
        diamondB3Program!=EDM130::IsG76DiamondB3_257(m_edm130.recipe))
    {RejectEDM94SameThread("EDM257_DIAMOND_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool diamondB4Program=b.has('L')&&b.val('L')==258.0;
    if(diamondB4Program!=E258DiamondB4Arm(armSource->parsedBlock,s.gapHistoryArm)||
        diamondB4Program!=EDM130::IsG76DiamondB4_258(m_edm130.recipe))
    {RejectEDM94SameThread("EDM258_DIAMOND_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool oneMmProgram=b.has('L')&&(b.val('L')==260.0||b.val('L')==261.0||b.val('L')==262.0||b.val('L')==263.0);
    if(oneMmProgram!=E260OneMmArm(armSource->parsedBlock,s.gapHistoryArm)||
        oneMmProgram!=EDM130::IsG76OneMm260(m_edm130.recipe)||
        (oneMmProgram&&(b.val('L')!=static_cast<double>(m_edm130.recipe.flushHoldTest)||
            E260OneMmMode(armSource->parsedBlock)!=EDM130::G76PlanMode(m_edm130.recipe))))
    {RejectEDM94SameThread("EDM260_ONE_MM_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool polygonB2Program=b.has('L')&&b.val('L')==264.0;
    const auto* polygonSource=EDM130::G76PlanMode(m_edm130.recipe)==16U?
        GetBaseProgramCache().TryGetLine(s.pc[0U]):nullptr;
    if(polygonB2Program!=E261PolygonB2Arm(armSource->parsedBlock,s.gapHistoryArm)||
        polygonB2Program!=EDM130::IsG76PolygonB2_261(m_edm130.recipe)||
        (EDM130::G76PlanMode(m_edm130.recipe)==16U&&
            (!polygonSource||!E180LiteralAngleSourceMatches(polygonSource->parsedBlock,w.authored)||
             !w.authored.has('Q')||w.nominal.program.input.polygonSides!=static_cast<unsigned>(w.authored.val('Q')))))
    {RejectEDM94SameThread("EDM261_POLYGON_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool expansionProgram=b.has('L')&&b.val('L')>=265.0&&b.val('L')<=272.0&&std::floor(b.val('L'))==b.val('L');
    const auto* expansionSource=EDM130::IsG76Expansion263(m_edm130.recipe)?GetBaseProgramCache().TryGetLine(s.pc[0U]):nullptr;
    if(expansionProgram!=E263ExpansionArm(armSource->parsedBlock,s.gapHistoryArm)||
        expansionProgram!=EDM130::IsG76Expansion263(m_edm130.recipe)||
        (expansionProgram&&(!expansionSource||!E180LiteralAngleSourceMatches(expansionSource->parsedBlock,w.authored)||
            b.val('L')!=static_cast<double>(m_edm130.recipe.flushHoldTest)||
            E263ExpansionMode(armSource->parsedBlock)!=EDM130::G76PlanMode(m_edm130.recipe)||
            !E263ExpansionOrbitShape(w.authored)||w.nominal.count!=EDM173::CountForRecipe(m_edm130.recipe)||
            w.nominal.program.input.polygonSides!=(w.authored.val('P')==26.0?static_cast<unsigned>(w.authored.val('Q')):0U))))
    {RejectEDM94SameThread("EDM263_EXPANSION_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool fixedRadiusProgram=b.has('L')&&b.val('L')>=273.0&&b.val('L')<=280.0&&std::floor(b.val('L'))==b.val('L');
    const auto* fixedRadiusSource=EDM130::IsG76FixedRadius266(m_edm130.recipe)?GetBaseProgramCache().TryGetLine(s.pc[0U]):nullptr;
    if(fixedRadiusProgram!=E266FixedRadiusArm(armSource->parsedBlock,s.gapHistoryArm)||
        fixedRadiusProgram!=EDM130::IsG76FixedRadius266(m_edm130.recipe)||
        (fixedRadiusProgram&&(!fixedRadiusSource||!E180LiteralAngleSourceMatches(fixedRadiusSource->parsedBlock,w.authored)||
            b.val('L')!=static_cast<double>(m_edm130.recipe.flushHoldTest)||
            E266FixedRadiusMode(armSource->parsedBlock)!=EDM130::G76PlanMode(m_edm130.recipe)||
            !E266FixedRadiusOrbitShape(w.authored)||w.nominal.count!=EDM173::CountForRecipe(m_edm130.recipe)||
            w.nominal.program.input.polygonSides!=(w.authored.val('P')==6.0?static_cast<unsigned>(w.authored.val('Q')):0U))))
    {RejectEDM94SameThread("EDM266_FIXED_RADIUS_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool fixedRadiusWorkProgram=b.has('L')&&b.val('L')>=281.0&&b.val('L')<=288.0&&std::floor(b.val('L'))==b.val('L');
    const auto* fixedRadiusWorkSource=EDM130::IsG76FixedRadiusWork267(m_edm130.recipe)?GetBaseProgramCache().TryGetLine(s.pc[0U]):nullptr;
    if(fixedRadiusWorkProgram!=E267FixedRadiusWorkArm(armSource->parsedBlock,s.gapHistoryArm)||
        fixedRadiusWorkProgram!=EDM130::IsG76FixedRadiusWork267(m_edm130.recipe)||
        (fixedRadiusWorkProgram&&(!fixedRadiusWorkSource||!E180LiteralAngleSourceMatches(fixedRadiusWorkSource->parsedBlock,w.authored)||
            b.val('L')!=static_cast<double>(m_edm130.recipe.flushHoldTest)||
            E267FixedRadiusWorkMode(armSource->parsedBlock)!=EDM130::G76PlanMode(m_edm130.recipe)||
            !E266FixedRadiusOrbitShape(w.authored)||w.nominal.count!=EDM173::CountForRecipe(m_edm130.recipe)||
            !E267FixedRadiusWorkFrame(w.nominal.translation)||
            !SameNCTranslationSnapshot(w.nominal.translation,CoordSys.GetTranslationSnapshot())||
            w.nominal.program.input.polygonSides!=(w.authored.val('P')==6.0?static_cast<unsigned>(w.authored.val('Q')):0U))))
    {RejectEDM94SameThread("EDM267_FIXED_RADIUS_WORK_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    const bool sharedShortProgram=b.has('L')&&b.val('L')==static_cast<double>(EDM130::G76SharedShortArmTag274);
    const auto* sharedShortSource=sharedShortProgram?GetBaseProgramCache().TryGetLine(s.pc[0U]):nullptr;
    if(sharedShortProgram!=E274SharedShortArm(armSource->parsedBlock,s.gapHistoryArm)||
        sharedShortProgram!=EDM130::IsG76SharedShort274(m_edm130.recipe)||
        (sharedShortProgram&&(!sharedShortSource||s.pc[0U]!=s.armPC+1||
            !E274SharedShortPlanSource(armSource->parsedBlock,sharedShortSource->parsedBlock,w.authored,
                m_edm130.recipe,w.nominal,CoordSys.GetTranslationSnapshot()))))
    {RejectEDM94SameThread("EDM274_VECTOR_SOURCE_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
    if(!m_motion.SubmitEDM173Arm(s.session,s.armEpoch,s.lease,s.gapTuning,s.gapFlushParameters,m_edm130.recipe,rejectBeforeArm))
    {RejectEDM94SameThread("EDM173_ARM_PUBLICATION",m_gapDryRun.sourceLine);return nullptr;}
    s.latched=s.armPending=true;
    if(rejectBeforeArm)
        PrintNCG76DiagnosticLine("[EDM216] event=ARM_REJECT_PROBE_REQUEST session=%llu profile=%u B=%u expect=REQUEST_SHAPE_REJECTED cause=RECOVERY_PROBE phase=BEFORE_ENGINE_ARM output=NONE next=MANUAL_RESET_THEN_NORMAL_PROGRAM discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(m_edm130.recipe.commissioningProfile),
            EDM173::ModeForArm(s.gapHistoryArm));
    // E204 FIX1: observe configured banks once from the NC thread. No live
    // integral/history is read here and no PID value is changed. Missing
    // FLUSH configuration may legitimately inherit the G00 values.
    // EDM204_FIX1_CENTER_PID_CONFIG_BEGIN
    if(EDM173::IsCenterRecipe(m_edm130.recipe))
    {
        for(unsigned axisIndex=0U;axisIndex<3U;++axisIndex)
        {
            const auto& configured=m_motion.GetAxisContext(static_cast<int>(axisIndex));
            const auto& g00=configured.Pid_G00;const auto& flush=configured.Pid_FLUSH;
            const bool equal=g00.Kp==flush.Kp&&g00.Ki==flush.Ki&&g00.Kd==flush.Kd&&g00.Kvff==flush.Kvff;
            PrintNCG76DiagnosticLine("[EDM204_FIX1] event=CENTER_PID_CONFIG session=%llu mode=%u axis=%c g00Kp=%.9g g00Ki=%.9g g00Kd=%.9g g00Kvff=%.9g flushKp=%.9g flushKi=%.9g flushKd=%.9g flushKvff=%.9g banksEqual=%u values=STATIC gainWrite=NONE selection=ACCEPTED_CENTER_EVENT discharge=0\n",
                static_cast<unsigned long long>(s.session),EDM173::ModeForArm(s.gapHistoryArm),"XYZ"[axisIndex],
                g00.Kp,g00.Ki,g00.Kd,g00.Kvff,flush.Kp,flush.Ki,flush.Kd,flush.Kvff,equal?1U:0U);
        }
    }
    // EDM204_FIX1_CENTER_PID_CONFIG_END
    if(EDM130::G76EventLimit(m_edm130.recipe)==2U)
        PrintNCG76DiagnosticLine("[%s] event=TWO_E5_ARMED session=%llu B=%u L=%u expectedEvents=2 workTimeUs=%llu timer=COMMITTED_MACHINING_ONLY rearm=AFTER_EXACT_RETURN autoHold=%u discharge=0\n",
            centerSecondReturnHold?"EDM232":centerSecondFlushHold?"EDM231":"EDM230",static_cast<unsigned long long>(s.session),
            EDM173::ModeForArm(s.gapHistoryArm),static_cast<unsigned>(m_edm130.recipe.flushHoldTest),
            static_cast<unsigned long long>(m_edm130.recipe.workTimeUs),(centerSecondFlushHold||centerSecondReturnHold)?1U:0U);
    // Compiler macro evidence only; this does not certify optimization flags.
#if defined(_DEBUG)
    const char* const buildMode="DEBUG";
#elif defined(NDEBUG)
    const char* const buildMode="RELEASE";
#else
    const char* const buildMode="UNSPECIFIED";
#endif
    RtPrintf("[EDM173] event=ARM_REQUEST session=%llu run=%llu cache=%llu armEpoch=%llu source=AUTO_WAVE ADrequired=0 parentPC=%d children=%u E5us=%llu E6microMm=%llu motion=REAL_ETHERCAT S92=0 S93=0 coreBuild=%u diagPolicy=EDM231_FIX5 p50Bulk=QUIET_RUN_HOLD periodicTimingText=DEFER_RUN_HOLD scopeCheck=CALL_LOCAL_CENTER_FLUSH buildMode=%s buildEvidence=PREPROCESSOR_MACROS timing=ON rtEvidence=ON B=%u arm=P%u\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.run),static_cast<unsigned long long>(s.cache),
        static_cast<unsigned long long>(s.armEpoch),s.pc[0U],EDM173::CountForRecipe(m_edm130.recipe),static_cast<unsigned long long>(m_edm130.recipe.workTimeUs),
        static_cast<unsigned long long>(m_edm130.recipe.jumpHeightScaled),
        static_cast<unsigned>(m_motion.GetEDM173ImplementationRevision()),buildMode,EDM173::ModeForArm(s.gapHistoryArm),s.gapHistoryArm);
    return [](NCManager* n){return n->WaitEDM173SameThread();};
}

WaitConditionFunc NCManager::StartEDM173OrbitSameThread(const NCBlock& b)
{
    if(m_currentExecutingBlockDispatchId<=m_edm94.armDispatch||!IsEDM173OrbitAllowedSameThread(b)||!FreezeEDM173TranslationSameThread()||!IsPathCoreFeedConfigurationValid()||
        !IsFixedTranslationTravelCurrentSameThread()||!m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||
        m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize())
    {RejectEDM94SameThread("EDM173_PARENT_ADMISSION",m_gapDryRun.sourceLine);return nullptr;}
    ResetG76OrbitStateSameThread();auto& g=*m_g76Orbit;auto& w=*m_edm173;const auto& s=m_edm94;
    g.run=s.run;g.cache=s.cache;g.dispatch=m_currentExecutingBlockDispatchId;g.pc=GetActiveDispatchPC();
    const auto* source=GetBaseProgramCache().TryGetLine(g.pc);g.line=source?source->sourceLineNumber:0;
    g.mode=EDM173::ModeForArm(s.gapHistoryArm);g.sourceG=76;
    if(!CaptureCncFeedValueSameThread(b,g.feed)||g.feed.feedMMMin!=0.12)
    {RejectEDM94SameThread("EDM173_FEED_SCOPE",g.line);return nullptr;}
    g.native=w.nominal;g.program=w.nominal.program;
    EDM173::Scope scope{};scope.session=s.session;scope.run=s.run;scope.cache=s.cache;scope.dispatch=g.dispatch;
    scope.sourcePC=g.pc;scope.line=g.line;scope.armEpoch=s.armEpoch;scope.owner=s.lease;
    double centerReturnFeed=0.0,configuredCenterReturnFeed=0.0;
    if(EDM173::IsCenterRecipe(m_edm130.recipe))
    {
        const auto speeds=GlobalConfig::GetInstance().GetG76CenterReturnSpeeds();
        if(!EDM171Config::ValidCenterReturnSpeeds(speeds))
        {RejectEDM94SameThread(EDM173::IsB4Recipe(m_edm130.recipe)?"EDM177_CENTER_SPEED_CONFIG":"EDM176_CENTER_SPEED_CONFIG",g.line);return nullptr;}
        configuredCenterReturnFeed=EDM173::IsB4Recipe(m_edm130.recipe)?speeds.b4MmMin:speeds.b3MmMin;
        centerReturnFeed=configuredCenterReturnFeed>0.0?configuredCenterReturnFeed:g.feed.feedMMMin;
    }
    if(!EDM173::Build(scope,w.nominal,m_edm130.recipe,s.gapTuning,w.plan,centerReturnFeed))
    {RejectEDM94SameThread("EDM173_PLAN_SCOPE",g.line);return nullptr;}
    // EDM213: one NC-thread observation of the exact speed frozen above.
    // The controller's existing fixture/output caps remain independent.
    if(EDM173::IsCenterRecipe(w.plan.recipe))
        PrintNCG76DiagnosticLine("[EDM213] event=CENTER_RETURN_SPEED_FROZEN session=%llu mode=%u parameter=%s selection=%s configuredMmMin=%.12g nominalFeedMmMin=%.12g frozenRequestedMmMin=%.12g discharge=0\n",
            static_cast<unsigned long long>(s.session),EDM173::ModeForArm(s.gapHistoryArm),
            EDM173::IsB4Recipe(w.plan.recipe)?"G76_B4_CENTER_RETURN_MM_MIN":"G76_B3_CENTER_RETURN_MM_MIN",
            configuredCenterReturnFeed>0.0?"PARAMETER":"G76_F",configuredCenterReturnFeed,
            g.feed.feedMMMin,w.plan.centerReturnFeedMmMin);
    const unsigned count=EDM173::CountForRecipe(w.plan.recipe);
    NCG76Mixed::Point8 stationaryRotaryModulo{};
    for(unsigned axis=3U;axis<6U;++axis)
        stationaryRotaryModulo[axis]=m_fixedTranslationTravel[axis].rotaryModulo;
    const bool accepted=m_motion.TryG76GapMoveTransactionalTail(w.plan,CoordSys.commandedMCS,g.receipt,&stationaryRotaryModulo);const auto& r=g.receipt;
    bool valid=accepted&&r.valid&&r.commandAccepted&&r.captureBound&&r.tailCommitted&&r.count==count&&r.axisMask==7U&&
        r.ownerLease.Matches(s.lease)&&r.translationGeneration==w.nominal.translationGeneration&&
        r.tail.IsCommitted()&&r.tail.captureBound&&r.tail.endpointExact&&r.tail.accountingValid&&
        r.tail.axisMask==r.validAxisMask&&r.tail.ownerLease.Matches(r.ownerLease)&&
        r.validAxisMask!=0U&&(r.validAxisMask&7U)==7U;
    for(unsigned k=0U;k<count&&valid;++k)
    {
        const auto& id=r.identities[k];valid=id.IsAssigned()&&id.epoch==m_motion.GetCurrentExecutionEpoch()&&
            id.source==MotionCommandSource::NC_MEMORY&&id.sourceBlockId==static_cast<MotionSourceBlockId>(g.pc)&&
            (!k||id.segmentId>r.identities[k-1U].segmentId);
        for(unsigned axis=0U;axis<8U&&valid;++axis)valid=EDM109::SameDouble(r.legEndMCS[k][axis],w.nominal.pointsMCS[k][axis]);
    }
    valid=valid&&IsFixedTranslationTravelCurrentSameThread()&&
        EDM173::BuildNominal(w.plan,r.startPulse,r.pulsePerUnit,w.canonical,&stationaryRotaryModulo)&&w.canonical.count==count;
    for(unsigned k=0U;k<count&&valid;++k)
        for(unsigned axis=0U;axis<8U&&valid;++axis)
            valid=EDM109::SameDouble(r.legEndPulse[k][axis],w.canonical.geometry[k].endPulse[axis]);
    valid=valid&&r.tail.identity.Matches(r.identities[count-1U])&&r.tail.identity.source==r.identities[count-1U].source&&
        r.tail.identity.sourceBlockId==r.identities[count-1U].sourceBlockId;
    for(unsigned axis=0U;axis<8U&&valid;++axis)
        valid=EDM109::SameDouble(r.startMCS[axis],w.nominal.startMCS[axis])&&
            EDM109::SameDouble(r.endMCS[axis],w.nominal.pointsMCS[count-1U][axis])&&
            EDM109::SameDouble(CoordSys.commandedMCS[axis],r.endMCS[axis])&&
            EDM109::SameDouble(r.endPulse[axis],w.canonical.geometry[count-1U].endPulse[axis]);
    if(!valid)
    {
        RtPrintf("[EDM173_FIX1] event=PARENT_REJECT session=%llu parentPC=%d accepted=%u gate=%s reason=%u axis=%d segment=%d discharge=0\n",
            static_cast<unsigned long long>(s.session),g.pc,accepted?1U:0U,MotionG76AdmissionGateName(r.rejectionGate),
            static_cast<unsigned>(r.rejectionReason),r.rejectionAxis,r.rejectionSegment);
        RejectG76OrbitSameThread("EDM173_NATIVE_RECEIPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;
    }
    // EDM233_PARENT_EPOCH_BIND_BEGIN
    // The native transaction above owns the one authorized epoch advance.
    // Bind only its fully committed, exact 25-child receipt; the guard's arm
    // identity remains unchanged and the later NC aggregate commit is required.
    if(EDM130::IsG76ParentSequence233(w.plan.recipe))
    {
        auto& q=m_edm233;
        const MotionExecutionEpoch nativeEpoch=r.identities[0U].epoch;
        const MotionExecutionEpoch expectedEpoch=s.armEpoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:s.armEpoch+1U;
        const bool parentBound=q.authorized&&!q.releasePending&&q.firstReleased==q.secondStarted&&q.firstSession&&
            (q.secondStarted?s.session>q.firstSession:s.session==q.firstSession)&&
            q.epoch==s.armEpoch&&nativeEpoch==expectedEpoch&&nativeEpoch==m_motion.GetCurrentExecutionEpoch()&&
            s.armPC==q.firstArmPC+(q.secondStarted?3:0)&&s.gapHistoryArm==q.arms[q.secondStarted?1U:0U]&&
            s.run==q.run&&s.cache==q.cache&&s.lease.Matches(q.lease)&&
            g.run==q.run&&g.cache==q.cache&&g.pc==s.pc[0U]&&g.dispatch>s.armDispatch&&
            w.plan.scope.session==s.session&&w.plan.scope.run==q.run&&w.plan.scope.cache==q.cache&&
            w.plan.scope.armEpoch==s.armEpoch&&w.plan.scope.owner.Matches(q.lease)&&
            w.plan.scope.dispatch==g.dispatch&&w.plan.scope.sourcePC==g.pc&&w.plan.scope.line==g.line;
        if(!parentBound)
        {RejectG76OrbitSameThread("EDM233_PARENT_EPOCH_RECEIPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;}
        // This NC-owner mutation is provisional until all non-epoch sequence
        // fences are rechecked. A rejection clears the entire grant and stops
        // the already accepted parent through the existing rejection path.
        q.epoch=nativeEpoch;
        if(!IsEDM233SequenceScopeCurrentSameThread())
        {RejectG76OrbitSameThread("EDM233_PARENT_EPOCH_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;}
        PrintNCG76DiagnosticLine("[EDM233] event=PARENT_EPOCH_BOUND parent=%u session=%llu armEpoch=%llu liveEpoch=%llu dispatch=%llu children=25 proof=COMMITTED_NATIVE_RECEIPT aggregateCommit=STILL_REQUIRED discharge=0\n",
            q.secondStarted?2U:1U,static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.armEpoch),
            static_cast<unsigned long long>(nativeEpoch),static_cast<unsigned long long>(g.dispatch));
    }
    // EDM233_PARENT_EPOCH_BIND_END
    if(m_edm237.authorized&&!BindEDM237ParentEpochSameThread(r.identities[0U].epoch,g.dispatch,g.pc))
    {RejectG76OrbitSameThread("EDM237_PARENT_EPOCH_RECEIPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;}
    if(m_edm234.authorized&&!BindEDM234ParentEpochSameThread(r.identities[0U].epoch,g.dispatch,g.pc))
    {RejectG76OrbitSameThread("EDM234_PARENT_EPOCH_RECEIPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;}
    g.active=true;w.published=true;
    RtPrintf("[EDM173] event=PARENT_PUBLISHED session=%llu parentPC=%d dispatch=%llu children=%u fingerprint=%llu trigger=E5_COMMITTED_MACHINING_TIME discharge=0\n",
        static_cast<unsigned long long>(s.session),g.pc,static_cast<unsigned long long>(g.dispatch),count,static_cast<unsigned long long>(w.plan.fingerprint));
    return [](NCManager* n){return n->CompleteG76OrbitSameThread();};
}

bool NCManager::IsEDM173RuntimeCompleteSameThread() noexcept
{
    auto& w=*m_edm173;const auto& s=m_edm94;const auto& g=*m_g76Orbit;
    if(!w.published||!w.parentCommitted||!EDM173::Valid(w.plan)||!s.latched||!EDM173::IsArm(s.gapHistoryArm)||s.cancelled||
        !EDM173::RecipeMatchesArm(s.gapHistoryArm,w.plan.recipe)||g.mode!=EDM173::ModeForArm(s.gapHistoryArm)||
        s.resetSeen||s.releaseRequested||s.rtFaultReported||s.run!=g.run||s.cache!=g.cache||!s.lease.Matches(g.receipt.ownerLease)) return false;
    const unsigned count=EDM173::CountForRecipe(w.plan.recipe);
    if(g.receipt.count!=count||w.nominal.count!=count||w.nominal.modeP!=w.plan.nominal.modeP)return false;
    if(EDM130::IsG76FixedRadiusWork267(w.plan.recipe)&&
        (!E267FixedRadiusWorkFrame(w.nominal.translation)||!E267FixedRadiusWorkFrame(CoordSys.GetTranslationSnapshot())||
         !SameNCTranslationSnapshot(w.nominal.translation,CoordSys.GetTranslationSnapshot())))return false;
    if(EDM130::IsG76SharedShort274(w.plan.recipe))
    {
        const auto* arm=GetBaseProgramCache().TryGetLine(s.armPC);
        const auto* source=GetBaseProgramCache().TryGetLine(g.pc);
        if(!arm||!source||g.pc!=s.armPC+1||
            !E274SharedShortPlanSource(arm->parsedBlock,source->parsedBlock,w.authored,w.plan.recipe,
                w.nominal,CoordSys.GetTranslationSnapshot()))return false;
    }
    if(NCG76Mode::IsPolygon(EDM130::G76PlanMode(w.plan.recipe))||EDM130::IsG76SharedShort274(w.plan.recipe)||EDM130::IsG76Expansion263(w.plan.recipe)||EDM130::IsG76FixedRadius266(w.plan.recipe)||EDM130::IsG76FixedRadiusWork267(w.plan.recipe))
    {
        const auto* source=GetBaseProgramCache().TryGetLine(g.pc);
        if(!source||!E180LiteralAngleSourceMatches(source->parsedBlock,w.authored)||
            !IsEDM173OrbitBlockShapeValid(w.authored)||w.authored.val('P')!=static_cast<double>(EDM130::G76PlanMode(w.plan.recipe))||
            (NCG76Mode::IsPolygon(EDM130::G76PlanMode(w.plan.recipe))?static_cast<unsigned>(w.authored.val('Q')):0U)!=w.plan.nominal.program.input.polygonSides||
            w.nominal.program.input.polygonSides!=w.plan.nominal.program.input.polygonSides)return false;
    }
    EDM173::Feedback feedback{};EDM126::MotionFeedback history{};EDM128::MotionFeedback timed{};EDM114::Feedback geometry{};
    if(!m_motion.ReadEDM173Feedback(feedback)||!m_motion.ReadEDM126Feedback(history)||!m_motion.ReadEDM130Feedback(timed)||
        !m_motion.ReadEDM114Feedback(geometry)) return false;
    const auto& scope=feedback.scope;const auto& plan=w.plan;const auto& e=feedback.completedEvent;
    const bool b0=EDM173::IsB0Recipe(plan.recipe),b2=EDM173::IsB2Recipe(plan.recipe),
        b3=EDM173::IsB3Recipe(plan.recipe),b4=EDM173::IsB4Recipe(plan.recipe);
    const EDM116::Mode expectedMode=EDM173::ExpectedMode(plan.recipe);
    const unsigned mode=static_cast<unsigned>(expectedMode);
    const bool identity=scope.session==s.session&&scope.run==s.run&&scope.cache==s.cache&&scope.dispatch==g.dispatch&&
        scope.sourcePC==g.pc&&scope.line==g.line&&scope.armEpoch==s.armEpoch&&scope.owner.Matches(s.lease)&&
        feedback.fingerprint==plan.fingerprint&&feedback.liveEpoch==m_motion.GetCurrentExecutionEpoch();
    if(!identity||!feedback.enabled||!feedback.active||feedback.faulted||!feedback.tick||!feedback.eventRetained||
        feedback.authorizedChildren!=count||feedback.completedChildren!=count||!history.enabled||history.faulted||history.physicalAd||
        history.session!=s.session||history.armEpoch!=s.armEpoch||history.liveEpoch!=feedback.liveEpoch||history.ownerGeneration!=s.lease.generation||
        history.fixture!=14U||history.reason!=EDM126::Reason::None||!history.tick||!history.nowUs||!history.sourceSequence||history.authorizedRows!=count||history.completedRows!=count||history.rowActive||!history.rowCompleted||
        !history.sourceReady||history.sourceReason!=EDM88::Reason::None||!history.sourceCapturedUs||
        history.nowUs<history.sourceCapturedUs||history.nowUs-history.sourceCapturedUs>EDM126::MaximumSourceAgeUs||
        !timed.enabled||timed.faulted||!timed.workTimeFlush||timed.physicalAd||timed.session!=s.session||timed.armEpoch!=s.armEpoch||
        timed.liveEpoch!=feedback.liveEpoch||timed.ownerGeneration!=s.lease.generation||timed.fixture!=14U||
        timed.authorizedRows!=count||timed.completedRows!=count||!EDM130::SameRecipe(timed.timerRecipe,m_edm130.recipe)) return false;
    const unsigned expectedEvents=EDM130::G76EventLimit(plan.recipe);
    const auto& cursor=history.traversal;const auto& last=g.receipt.identities[count-1U];
    if(feedback.retainedEvents!=expectedEvents||cursor.faulted||cursor.stopRequested||cursor.stopped||
        !cursor.completeReady||cursor.historyDepth||cursor.velocityMmSec!=0.0||
        cursor.authorizedSource.ordinal!=count-1U||cursor.authorizedSource.dispatch!=g.dispatch||cursor.authorizedSource.sourcePC!=g.pc||
        cursor.authorizedSource.segment!=last.segmentId||cursor.authorizedSource.epoch!=last.epoch||
        !EDM126::SameSource(cursor.authorizedSource,cursor.cursorSource)||
        timed.coordinator.work.triggeredEvents!=expectedEvents||timed.coordinator.work.completedEvents!=expectedEvents||
        timed.coordinator.eventCount!=expectedEvents||timed.coordinator.modeEvents[mode]!=expectedEvents||
        timed.coordinator.policy.mode!=expectedMode||timed.coordinator.policy.fixture!=EDM116::Fixture::ModalG76Gap173||
        EDM116::PolicyRows(timed.coordinator.policy)!=count||EDM116::PolicyRows(timed.rowPolicy)!=count) return false;
    for(unsigned eventMode=0U;eventMode<EDM116::ModeCount;++eventMode)
        if(eventMode!=mode&&timed.coordinator.modeEvents[eventMode])return false;
    if(!b2&&(feedback.eventBackwardCrossings||feedback.eventForwardCrossings))return false;
    const auto& work=e.work;const auto& finalWork=timed.coordinator.work;
    if(work.workTimeUs!=plan.recipe.workTimeUs||finalWork.workTimeUs!=plan.recipe.workTimeUs||
        work.lastTriggerElapsedUs!=finalWork.lastTriggerElapsedUs||work.lastTriggerTotalUs!=finalWork.lastTriggerTotalUs||
        work.lastTriggerTick!=finalWork.lastTriggerTick||work.lastTriggerUs!=finalWork.lastTriggerUs||
        work.lastCompleteTick!=finalWork.lastCompleteTick||work.lastCompleteUs!=finalWork.lastCompleteUs||
        work.lastCompleteTick>feedback.tick||work.lastCompleteTick>history.tick||work.lastCompleteUs>history.nowUs||
        !EDM130::ValidWorkEvidence(finalWork)) return false;
    if(expectedEvents==2U&&!EDM173::FollowsCompletedEvent230(feedback.firstCompletedEvent,e))return false;
    const auto validateEvent=[&](const EDM128::Feedback& e,unsigned ordinal) noexcept -> bool {
        if(!ordinal||ordinal>expectedEvents||e.policy.mode!=expectedMode||
            e.policy.fixture!=EDM116::Fixture::ModalG76Gap173||EDM116::PolicyRows(e.policy)!=count||e.eventIndex!=ordinal||
            e.eventCount!=ordinal||e.modeEvents[mode]!=ordinal||e.work.triggeredEvents!=ordinal||
            e.work.completedEvents!=ordinal||!e.exactReturn||!e.rowEventComplete||!e.topCaptured||
            e.phase!=EDM128::Phase::Gap||e.detached||e.faulted||e.stopped||
            e.work.workTimeUs!=plan.recipe.workTimeUs||!EDM130::ValidWorkEvidence(e.work)||
            e.work.lastCompleteTick>feedback.tick||e.work.lastCompleteTick>history.tick||
            e.work.lastCompleteUs>history.nowUs||
            !EDM128::ValidHeightProof(e,EDM173::FrozenHeightMm(plan.recipe),0U,1.0,true)||
            !EDM128::ValidSpeedProof(e,m_edm130.recipe.flushStages))return false;
        // Modern commissioning events retain their complete dwell sequence and
        // surround the genuine stop anchor. The four current clocks match above.
        if(EDM130::IsG76CommissioningRecipe(plan.recipe)&&
            (e.qualifiedDwells!=((b3||b4)?5U:3U)||timed.coordinator.qualifiedDwells!=((b3||b4)?5U:3U)||
             !(e.work.lastTriggerTick<e.anchor.tick&&e.anchor.tick<e.work.lastCompleteTick&&
               e.work.lastTriggerUs<e.anchor.nowUs&&e.anchor.nowUs<e.work.lastCompleteUs)))return false;
        for(unsigned eventMode=0U;eventMode<EDM116::ModeCount;++eventMode)
            if(eventMode!=mode&&e.modeEvents[eventMode])return false;
        const auto& anchor=e.anchor.authorizedSource;const auto& anchorCursor=e.anchor.cursorSource;
        const unsigned authorizedChild=static_cast<unsigned>(anchor.ordinal),child=static_cast<unsigned>(anchorCursor.ordinal);
        const auto sourceFits=[&](const EDM126::Source& source,unsigned ordinal) noexcept {
            return ordinal<count&&source.ordinal==ordinal&&source.scope.session==s.session&&source.scope.run==s.run&&
                source.scope.cache==s.cache&&source.scope.ownerGeneration==s.lease.generation&&
                source.scope.translationGeneration==plan.nominal.translationGeneration&&source.scope.configTag&&source.scope.mappingTag&&
                source.dispatch==g.dispatch&&source.sourcePC==g.pc&&source.epoch==g.receipt.identities[ordinal].epoch&&
                source.segment==g.receipt.identities[ordinal].segmentId;
        };
        if(!sourceFits(cursor.authorizedSource,count-1U)||!sourceFits(anchor,authorizedChild)||!sourceFits(anchorCursor,child)||
            child>authorizedChild||!EDM109::SameScope(anchor.scope,anchorCursor.scope)||
            !EDM109::SameScope(anchor.scope,cursor.authorizedSource.scope)||
            !EDM116::PolicyMatchesBinding(e.policy,authorizedChild,g.pc,g.dispatch)) return false;
        EDM126::Point anchorStart{},anchorEnd{},anchorScale{},expectedAnchor{};
        for(unsigned axis=0U;axis<6U;++axis)
        {
            anchorStart[axis]=child?g.receipt.legEndPulse[child-1U][axis]:g.receipt.startPulse[axis];
            anchorEnd[axis]=g.receipt.legEndPulse[child][axis];anchorScale[axis]=g.receipt.pulsePerUnit[axis];
        }
        EDM109::Line anchorLine{};EDM112::Curve anchorCurve{};
        if(!EDM109::BuildLine(anchorStart,anchorEnd,anchorScale,anchorLine)||!EDM112::BuildLine(anchorLine,anchorCurve)||
            !std::isfinite(e.anchor.localMm)||e.anchor.localMm<0.0||e.anchor.localMm>anchorCurve.lengthMm||
            !EDM112::EvaluateUnit(anchorCurve,e.anchor.localMm/anchorCurve.lengthMm,expectedAnchor)) return false;
        for(unsigned axis=0U;axis<6U;++axis)
        {
            const double budget=64.0*(std::numeric_limits<double>::epsilon)()*(std::max)(1.0,std::fabs(expectedAnchor[axis]));
            if(!std::isfinite(e.anchor.positionPulse[axis])||std::fabs(e.anchor.positionPulse[axis]-expectedAnchor[axis])>budget) return false;
        }
        if(b0)
        {
            // Reconstruct the native tangent from the genuine stopped child,
            // independently of the runtime's captured direction. E6 is native
            // distance; the nominal child already contains every compensation.
            // Later children clear their local capture. The retained event owns
            // geometry; the final coordinator contributes work/count/dwell totals.
            if(e.detached||e.qualifiedDwells!=3U||timed.coordinator.qualifiedDwells!=3U||
                plan.recipe.planMovementMask!=EDM173::NominalMovementMask(plan.nominal))return false;
            const bool planar=plan.recipe.planMovementMask==3U;
            if(planar&&(anchorStart[2U]!=g.receipt.startPulse[2U]||anchorEnd[2U]!=g.receipt.startPulse[2U]||
                e.anchor.positionPulse[2U]!=g.receipt.startPulse[2U]||e.topPositionPulse[2U]!=g.receipt.startPulse[2U]||
                e.tangentPulsePerMm[2U]!=0.0))return false;
            EDM126::Point tangent{};double norm=0.0;
            for(unsigned axis=0U;axis<6U;++axis)
            {
                if(axis>=3U&&!EDM109::SameDouble(anchorStart[axis],anchorEnd[axis]))return false;
                tangent[axis]=(anchorEnd[axis]-anchorStart[axis])/anchorCurve.lengthMm;
                if(axis<3U)norm=std::hypot(norm,tangent[axis]/anchorScale[axis]);
            }
            if(!EDM126::Positive(norm))return false;
            for(unsigned axis=0U;axis<6U;++axis)
            {
                tangent[axis]/=norm;
                const double top=tangent[axis]==0.0?e.anchor.positionPulse[axis]:
                    e.anchor.positionPulse[axis]-EDM173::FrozenHeightMm(plan.recipe)*tangent[axis];
                if(!std::isfinite(tangent[axis])||!std::isfinite(top)||
                    !EDM109::SameDouble(e.tangentPulsePerMm[axis],tangent[axis])||
                    !EDM109::SameDouble(e.topPositionPulse[axis],top)||
                    (axis>=3U&&(!EDM109::SameDouble(tangent[axis],0.0)||
                        !EDM109::SameDouble(top,e.anchor.positionPulse[axis]))))return false;
            }
        }
        if(b2)
        {
            // The B2 apex is a cursor on the same sealed nominal parent, not a
            // detached Z lift or a newly published child. Rebuild both stations
            // from genuine child geometry; E6 is retained path length, not chord.
            const unsigned targetChild=static_cast<unsigned>(e.targetSource.ordinal);
            if(!sourceFits(e.targetSource,targetChild)||targetChild>child||
                ((EDM130::IsG76SquareB2_251(plan.recipe)||EDM130::IsG76DiamondB2_256(plan.recipe))&&targetChild==child)||
                !EDM109::SameScope(anchorCursor.scope,e.targetSource.scope)||
                e.qualifiedDwells!=3U||timed.coordinator.qualifiedDwells!=3U||
                feedback.eventBackwardCrossings!=static_cast<std::uint64_t>(child-targetChild)||
                feedback.eventForwardCrossings!=feedback.eventBackwardCrossings||
                plan.recipe.planMovementMask!=EDM173::NominalMovementMask(plan.nominal))return false;
            if(plan.recipe.planMovementMask==3U)
            {
                const double fixedZ=g.receipt.startPulse[2U];
                if(anchorStart[2U]!=fixedZ||anchorEnd[2U]!=fixedZ||
                    e.anchor.positionPulse[2U]!=fixedZ||e.topPositionPulse[2U]!=fixedZ)return false;
                for(unsigned k=0U;k<count;++k)
                    if(g.receipt.legEndPulse[k][2U]!=fixedZ)return false;
            }
            const auto canonicalCursor=[&](unsigned ordinal,double localMm,EDM126::Point& point,double& station) noexcept {
                station=0.0;
                EDM126::Point from{},to{};EDM109::Line line{};EDM112::Curve curve{};
                for(unsigned k=0U;k<=ordinal;++k)
                {
                    for(unsigned axis=0U;axis<6U;++axis)
                    {
                        from[axis]=k?g.receipt.legEndPulse[k-1U][axis]:g.receipt.startPulse[axis];
                        to[axis]=g.receipt.legEndPulse[k][axis];
                    }
                    if(!EDM109::BuildLine(from,to,anchorScale,line)||!EDM112::BuildLine(line,curve))return false;
                    if(k==ordinal)
                    {
                        if(!std::isfinite(localMm)||localMm<0.0||localMm>curve.lengthMm||
                            !EDM112::EvaluateUnit(curve,localMm/curve.lengthMm,point))return false;
                        station+=localMm;return std::isfinite(station);
                    }
                    station+=curve.lengthMm;
                    if(!std::isfinite(station))return false;
                }
                return false;
            };
            EDM126::Point canonicalAnchor{},canonicalTop{};double anchorStation=0.0,targetStation=0.0;
            if(!canonicalCursor(child,e.anchor.localMm,canonicalAnchor,anchorStation)||
                !canonicalCursor(targetChild,e.targetLocalMm,canonicalTop,targetStation)||
                std::fabs(e.anchor.stationMm-anchorStation)>EDM126::Roundoff(anchorStation)||
                std::fabs(e.targetStationMm-targetStation)>EDM126::Roundoff(targetStation)||
                std::fabs((anchorStation-targetStation)-EDM173::FrozenHeightMm(plan.recipe))>
                    EDM126::Roundoff(anchorStation))return false;
            for(unsigned axis=0U;axis<6U;++axis)
            {
                const double budget=64.0*(std::numeric_limits<double>::epsilon)()*(std::max)(1.0,std::fabs(canonicalTop[axis]));
                if(!std::isfinite(e.topPositionPulse[axis])||std::fabs(e.topPositionPulse[axis]-canonicalTop[axis])>budget)return false;
            }
        }
        EDM204::Geometry centerGeometry{};
        if(b3||b4)
        {
            NCG76::Point8 centerStartMCS{},centerEndMCS{};
            EDM126::Point nominalCenter{},axisNative{},axisPulse{};
            if(!EDM128::ValidB3Proof(e)||!e.centerCaptured||e.centerLegsCompleted!=15U||
                e.qualifiedDwells!=5U*ordinal||timed.coordinator.qualifiedDwells!=5U*expectedEvents||
                !EDM109::SameDouble(e.centerReturnMmMin,plan.centerReturnFeedMmMin)||
                !EDM173::CenterBoundary(plan,child,centerStartMCS)||
                !EDM173::CenterBoundary(plan,child+1U,centerEndMCS)||
                !EDM173::MachiningAxisNative(plan,axisNative))return false;
            for(unsigned axis=0U;axis<6U;++axis)
            {
                const double from=axis>=3U||centerStartMCS[axis]==plan.nominal.startMCS[axis]?g.receipt.startPulse[axis]:
                    centerStartMCS[axis]*g.receipt.pulsePerUnit[axis];
                const double to=axis>=3U||centerEndMCS[axis]==plan.nominal.startMCS[axis]?g.receipt.startPulse[axis]:
                    centerEndMCS[axis]*g.receipt.pulsePerUnit[axis];
                const double ratio=e.anchor.localMm/anchorCurve.lengthMm;
                nominalCenter[axis]=ratio==0.0?from:ratio==1.0?to:from+(to-from)*ratio;
                axisPulse[axis]=axisNative[axis]*anchorScale[axis];
                const double budget=EDM204::Roundoff(nominalCenter[axis]);
                if(!std::isfinite(nominalCenter[axis])||!std::isfinite(e.nominalCenterPositionPulse[axis])||
                    std::fabs(e.nominalCenterPositionPulse[axis]-nominalCenter[axis])>budget||
                    e.tangentPulsePerMm[axis]!=axisPulse[axis]||
                    (axis>=3U&&(!EDM109::SameDouble(nominalCenter[axis],e.anchor.positionPulse[axis])||
                        !EDM109::SameDouble(e.nominalCenterPositionPulse[axis],e.anchor.positionPulse[axis]))))return false;
            }
            // Reconstruct from the sealed signed XYZ target, never the child tangent
            // or the undirected orbit-plane normal. E6 is measured from nominal C.
            if(!EDM204::Build(e.anchor.positionPulse,nominalCenter,axisPulse,anchorScale,
                EDM173::FrozenHeightMm(plan.recipe),b3?3U:4U,centerGeometry))return false;
            for(unsigned axis=0U;axis<6U;++axis)
            {
                if(!std::isfinite(e.centerPositionPulse[axis])||!std::isfinite(e.topPositionPulse[axis])||
                    std::fabs(e.centerPositionPulse[axis]-centerGeometry.centerPulse[axis])>EDM204::Roundoff(centerGeometry.centerPulse[axis])||
                    std::fabs(e.topPositionPulse[axis]-centerGeometry.topPulse[axis])>EDM204::Roundoff(centerGeometry.topPulse[axis])||
                    (axis>=3U&&(!EDM109::SameDouble(e.centerPositionPulse[axis],e.anchor.positionPulse[axis])||
                        !EDM109::SameDouble(e.topPositionPulse[axis],e.anchor.positionPulse[axis]))))return false;
            }
        }
        else if(!EDM128::EmptyB3Proof(e))return false;
        for(unsigned axis=0U;axis<6U;++axis)
            if(!EDM109::SameDouble(cursor.positionPulse[axis],g.receipt.endPulse[axis])||
                e.heightPulsePerNative[axis]!=g.receipt.pulsePerUnit[axis]) return false;
        if(b0)
            PrintNCG76DiagnosticLine("[EDM192] event=B0_TANGENT_VERIFIED session=%llu parentPC=%d eventIndex=%u anchorChild=%u nativeTangentX=%.12g nativeTangentY=%.12g nativeTangentZ=%.12g heightMm=%.12g dwells=%u proof=GENUINE_SEALED_CHILD_NATIVE_TANGENT exactReturn=1 CUVfixed=1 discharge=0\n",
                static_cast<unsigned long long>(s.session),g.pc,ordinal,child+1U,
                e.tangentPulsePerMm[0U]/anchorScale[0U],e.tangentPulsePerMm[1U]/anchorScale[1U],
                e.tangentPulsePerMm[2U]/anchorScale[2U],e.actualJumpHeightMm,e.qualifiedDwells);
        if(b2)
            PrintNCG76DiagnosticLine("[EDM175] event=B2_PATH_VERIFIED session=%llu parentPC=%d eventIndex=%u anchorChild=%u targetChild=%u anchorStationMm=%.12g targetStationMm=%.12g pathHeightMm=%.12g eventBackward=%llu eventForward=%llu proof=GENUINE_SEALED_CHILD_CANONICAL_POINT_AND_STATION exactReturn=1 discharge=0\n",
                static_cast<unsigned long long>(s.session),g.pc,ordinal,child+1U,static_cast<unsigned>(e.targetSource.ordinal)+1U,
                e.anchor.stationMm,e.targetStationMm,e.actualJumpHeightMm,
                static_cast<unsigned long long>(feedback.eventBackwardCrossings),static_cast<unsigned long long>(feedback.eventForwardCrossings));
        if(b3||b4)
        {
            PrintNCG76DiagnosticLine("[EDM204] event=CENTER_AXIS_VERIFIED session=%llu parentPC=%d eventIndex=%u mode=%u anchorChild=%u machiningX=%.12g machiningY=%.12g machiningZ=%.12g radiusMm=%.12g axialSpanMm=%.12g withdrawalMm=%.12g centerF=%.12g settledLegs=4 dwells=%u exactReturn=1 CUVfixed=1 discharge=0\n",
                static_cast<unsigned long long>(s.session),g.pc,ordinal,mode,child+1U,
                e.tangentPulsePerMm[0U]/anchorScale[0U],e.tangentPulsePerMm[1U]/anchorScale[1U],e.tangentPulsePerMm[2U]/anchorScale[2U],
                centerGeometry.radiusMm,centerGeometry.axialSpanMm,centerGeometry.heightMm,
                (std::min)(e.centerReturnMmMin,EDM173::FeedMmMin),e.qualifiedDwells);
            PrintNCG76DiagnosticLine("[EDM204] event=CENTER_ORIGIN_POINTS session=%llu parentPC=%d eventIndex=%u mode=%u anchorXpulse=%.12g anchorYpulse=%.12g anchorZpulse=%.12g nominalCenterXpulse=%.12g nominalCenterYpulse=%.12g nominalCenterZpulse=%.12g discharge=0\n",
                static_cast<unsigned long long>(s.session),g.pc,ordinal,mode,e.anchor.positionPulse[0U],e.anchor.positionPulse[1U],e.anchor.positionPulse[2U],
                e.nominalCenterPositionPulse[0U],e.nominalCenterPositionPulse[1U],e.nominalCenterPositionPulse[2U]);
            PrintNCG76DiagnosticLine("[EDM204] event=CENTER_ROUTE_POINTS session=%llu parentPC=%d eventIndex=%u mode=%u waypointXpulse=%.12g waypointYpulse=%.12g waypointZpulse=%.12g topXpulse=%.12g topYpulse=%.12g topZpulse=%.12g discharge=0\n",
                static_cast<unsigned long long>(s.session),g.pc,ordinal,mode,e.centerPositionPulse[0U],e.centerPositionPulse[1U],e.centerPositionPulse[2U],
                e.topPositionPulse[0U],e.topPositionPulse[1U],e.topPositionPulse[2U]);
        }
        return true;
    };
    // Both exact-return receipts must independently rebuild from the same
    // sealed parent. A last-event total alone cannot replace the first proof.
    for(unsigned ordinal=1U;ordinal<=expectedEvents;++ordinal)
        if(!validateEvent(ordinal==expectedEvents?e:feedback.firstCompletedEvent,ordinal))return false;
    if(!geometry.enabled||geometry.failed||!geometry.completed||!geometry.rowComplete||geometry.completedRows!=count||
        geometry.session!=s.session||geometry.armEpoch!=s.armEpoch||geometry.ownerGeneration!=s.lease.generation||
        geometry.source.scope.run!=s.run||geometry.source.scope.cache!=s.cache||geometry.source.dispatch!=g.dispatch||
        geometry.source.sourcePC!=g.pc||geometry.source.step!=count-1U||geometry.source.segment!=last.segmentId||geometry.source.epoch!=last.epoch||
        !geometry.geometryValid||!EDM109::SameScope(geometry.source.scope,cursor.authorizedSource.scope)) return false;
    for(unsigned axis=0U;axis<6U;++axis)if(!EDM109::SameDouble(geometry.endPulse[axis],g.receipt.endPulse[axis])) return false;
    if(!IsEDM200AutoHoldCheckpointCompleteSameThread())
    {RejectEDM94SameThread("EDM200_FINAL_CHECKPOINT_NOT_COMPLETED",m_gapDryRun.sourceLine);return false;}
    w.retained=feedback;w.completed=true;m_edm114.retained=geometry;
    if(EDM130::IsG76B2FlushHold200(plan.recipe))
        RtPrintf("[EDM200] event=CHECKPOINT_COMPLETE session=%llu hold=%llu start=%llu eventIndex=1 children=25 B2events=1 exactReturn=1 proof=VALIDATED_PARENT_AND_RT_RUNNING discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(m_edm200.holdSequence),
            static_cast<unsigned long long>(m_edm200.resumeStartSequence));
    if(EDM130::IsG76CenterHold217(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM217] event=CHECKPOINT_COMPLETE session=%llu hold=%llu start=%llu B=%u holdEventIndex=0 children=25 E5events=1 exactReturn=1 proof=VALIDATED_PARENT_AND_RT_RUNNING discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(m_edm200.holdSequence),
            static_cast<unsigned long long>(m_edm200.resumeStartSequence),mode);
    if(EDM130::IsG76CenterAnyFlushHold232(plan.recipe))
        PrintNCG76DiagnosticLine("[%s] event=CHECKPOINT_COMPLETE session=%llu hold=%llu start=%llu B=%u holdEventIndex=%llu children=25 E5events=%u exactReturn=1 proof=VALIDATED_PARENT_AND_RT_RUNNING discharge=0\n",
            EDM130::IsG76CenterSecondReturnHold232(plan.recipe)?"EDM232":EDM130::IsG76CenterSecondFlushHold231(plan.recipe)?"EDM231":EDM130::IsG76CenterBrakeHold228(plan.recipe)?"EDM228":EDM130::IsG76CenterInitialDwellHold227(plan.recipe)?"EDM227":EDM130::IsG76CenterFinalDwellHold226(plan.recipe)?"EDM226":EDM130::IsG76CenterReturnDwellHold225(plan.recipe)?"EDM225":EDM130::IsG76CenterDwellHold224(plan.recipe)?"EDM224":EDM130::IsG76CenterTopDwellHold222(plan.recipe)?"EDM223":EDM130::IsG76CenterStartHold221(plan.recipe)?"EDM221":EDM130::IsG76CenterAnchorHold220(plan.recipe)?"EDM220":EDM130::IsG76CenterReturnHold219(plan.recipe)?"EDM219":"EDM218",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(m_edm200.holdSequence),
            static_cast<unsigned long long>(m_edm200.resumeStartSequence),mode,
            static_cast<unsigned long long>(m_edm200.eventIndex),expectedEvents);
    RtPrintf("[EDM173] event=GAP_E5_VERIFIED session=%llu parentPC=%d children=%u triggered=%u completed=%u B%uevents=%u heightMicroMm=%llu exactReturn=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),g.pc,count,expectedEvents,expectedEvents,mode,expectedEvents,
        static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76SharedShort274(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM274] event=VECTOR_GAP_E5_VERIFIED session=%llu P=%u Q=%u B=%u children=%u E5events=%u dwells=%u settledLegs=%u E6microMm=%llu exactReturn=1 CUVfixed=1 power=INHIBITED discharge=0\n",
            static_cast<unsigned long long>(s.session),EDM130::G76PlanMode(plan.recipe),
            plan.nominal.program.input.polygonSides,mode,count,expectedEvents,e.qualifiedDwells,(b3||b4)?4U:0U,
            static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76FixedRadiusWork267(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM267] event=FIXED_RADIUS_WORK_E5_VERIFIED session=%llu L=%u modeP=%u Q=%u B=%u children=26 E5events=1 dwells=%u settledLegs=%u E6microMm=%llu exactReturn=1 CUVfixed=1 workMode=G168 W=%d advance=CONSTANT_RADIUS hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(plan.recipe.flushHoldTest),EDM130::G76PlanMode(plan.recipe),
            plan.nominal.program.input.polygonSides,mode,e.qualifiedDwells,(b3||b4)?4U:0U,
            static_cast<unsigned long long>(plan.recipe.jumpHeightScaled),plan.nominal.translation.workWCode);
    if(EDM130::IsG76FixedRadius266(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM266] event=FIXED_RADIUS_E5_VERIFIED session=%llu L=%u modeP=%u Q=%u B=%u children=26 E5events=1 dwells=%u settledLegs=%u E6microMm=%llu exactReturn=1 CUVfixed=1 advance=CONSTANT_RADIUS hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(plan.recipe.flushHoldTest),EDM130::G76PlanMode(plan.recipe),
            plan.nominal.program.input.polygonSides,mode,e.qualifiedDwells,(b3||b4)?4U:0U,
            static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76Expansion263(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM263] event=EXPANSION_E5_VERIFIED session=%llu L=%u modeP=%u Q=%u B=%u children=26 E5events=1 dwells=%u settledLegs=%u E6microMm=%llu exactReturn=1 CUVfixed=1 expansion=TARGET_PLANE_GROW hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(plan.recipe.flushHoldTest),EDM130::G76PlanMode(plan.recipe),
            plan.nominal.program.input.polygonSides,mode,e.qualifiedDwells,(b3||b4)?4U:0U,
            static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::G76PlanMode(plan.recipe)==16U)
        PrintNCG76DiagnosticLine("[EDM261] event=POLYGON_E5_VERIFIED session=%llu L=%u modeP=16 Q=%u B=%u children=25 E5events=1 dwells=%u settledLegs=%u E6microMm=%llu exactReturn=1 CUVfixed=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(plan.recipe.flushHoldTest),
            plan.nominal.program.input.polygonSides,mode,e.qualifiedDwells,(b3||b4)?4U:0U,
            static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76OneMm260(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM260] event=ONE_MM_E5_VERIFIED session=%llu L=%u modeP=%u B=%u children=25 E5events=1 dwells=%u settledLegs=%u E6microMm=%llu exactReturn=1 CUVfixed=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(plan.recipe.flushHoldTest),
            EDM130::G76PlanMode(plan.recipe),mode,e.qualifiedDwells,(b3||b4)?4U:0U,
            static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76SquareB1_250(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM250] event=P12_B1_E5_VERIFIED session=%llu modeP=12 shape=SQUARE B=1 children=25 E5events=1 dwells=3 E6microMm=%llu exactReturn=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76SquareB2_251(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM251] event=P12_B2_E5_VERIFIED session=%llu modeP=12 shape=SQUARE B=2 children=25 E5events=1 dwells=3 pathMicroMm=%llu eventBackward=%llu eventForward=%llu exactReturn=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled),
            static_cast<unsigned long long>(feedback.eventBackwardCrossings),static_cast<unsigned long long>(feedback.eventForwardCrossings));
    if(EDM130::IsG76SquareB3_252(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM252] event=P12_B3_E5_VERIFIED session=%llu modeP=12 shape=SQUARE B=3 children=25 E5events=1 dwells=5 settledLegs=4 E6microMm=%llu exactReturn=1 CUVfixed=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76DiamondB3_257(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM257] event=P13_B3_E5_VERIFIED session=%llu modeP=13 shape=DIAMOND B=3 children=25 E5events=1 dwells=5 settledLegs=4 E6microMm=%llu exactReturn=1 CUVfixed=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76SquareB4_253(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM253] event=P12_B4_E5_VERIFIED session=%llu modeP=12 shape=SQUARE B=4 children=25 E5events=1 dwells=5 settledLegs=4 E6microMm=%llu centerJoin=DIAGONAL_45_DEG exactReturn=1 CUVfixed=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76DiamondB4_258(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM258] event=P13_B4_E5_VERIFIED session=%llu modeP=13 shape=DIAMOND B=4 children=25 E5events=1 dwells=5 settledLegs=4 E6microMm=%llu centerJoin=DIAGONAL_45_DEG exactReturn=1 CUVfixed=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76DiamondB1_255(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM255] event=P13_B1_E5_VERIFIED session=%llu modeP=13 shape=DIAMOND B=1 children=25 E5events=1 dwells=3 E6microMm=%llu exactReturn=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled));
    if(EDM130::IsG76DiamondB2_256(plan.recipe))
        PrintNCG76DiagnosticLine("[EDM256] event=P13_B2_E5_VERIFIED session=%llu modeP=13 shape=DIAMOND B=2 children=25 E5events=1 dwells=3 pathMicroMm=%llu eventBackward=%llu eventForward=%llu exactReturn=1 hold=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(plan.recipe.jumpHeightScaled),
            static_cast<unsigned long long>(feedback.eventBackwardCrossings),static_cast<unsigned long long>(feedback.eventForwardCrossings));
    if(expectedEvents==2U)
        PrintNCG76DiagnosticLine("[%s] event=TWO_E5_VERIFIED session=%llu B=%u expected=2 retained=2 dwells=10 firstCompleteTick=%llu secondTriggerTick=%llu secondWorkUs=%llu requiredWorkUs=%llu exactReturns=2 discharge=0\n",
            EDM130::IsG76CenterSecondReturnHold232(plan.recipe)?"EDM232":EDM130::IsG76CenterSecondFlushHold231(plan.recipe)?"EDM231":"EDM230",
            static_cast<unsigned long long>(s.session),mode,
            static_cast<unsigned long long>(feedback.firstCompletedEvent.work.lastCompleteTick),
            static_cast<unsigned long long>(e.work.lastTriggerTick),
            static_cast<unsigned long long>(e.work.lastTriggerTotalUs-feedback.firstCompletedEvent.work.totalWorkUs),
            static_cast<unsigned long long>(plan.recipe.workTimeUs));
    return true;
}

void NCManager::ServiceEDM173SameThread() noexcept
{
    // Read the independent bounded image before the phase-only early return.
    // Never inspect an RT AxisContext from this NC diagnostic path.
    if (EDM130::IsG76SharedShort274(m_edm130.recipe))
    {
        EDM275::Snapshot observation{}; std::uint64_t publication = 0ULL;
        const auto& identity = m_edm94.feedback.currentIdentity;
        if (EDM275::Read(observation, publication) && observation.session == m_edm94.session &&
            observation.ownerGeneration == m_edm94.lease.generation && identity.session == observation.session &&
            identity.ownerGeneration == observation.ownerGeneration && identity.epoch == observation.epoch)
            E275LogObservation(observation, publication);
    }
    auto& w=*m_edm173;EDM128::MotionFeedback f{};
    if(!m_motion.ReadEDM130Feedback(f)||f.session!=m_edm94.session||f.armEpoch!=m_edm94.armEpoch||
        f.ownerGeneration!=m_edm94.lease.generation||!EDM130::SameRecipe(f.timerRecipe,m_edm130.recipe)) return;
    const auto& event=f.coordinator;
    if(w.loggedTriggers==event.work.triggeredEvents&&w.loggedCompletions==event.work.completedEvents&&
        w.loggedPhase==static_cast<unsigned>(event.phase)) return;
    w.loggedTriggers=event.work.triggeredEvents;w.loggedCompletions=event.work.completedEvents;w.loggedPhase=static_cast<unsigned>(event.phase);
    RtPrintf("[EDM173] event=E5_PROGRESS session=%llu child=%u phase=%s triggered=%llu completed=%llu retained=REAL_RT_MAPPING discharge=0\n",
        static_cast<unsigned long long>(f.session),f.authorizedRows,EDM128::PhaseName(event.phase),
        static_cast<unsigned long long>(event.work.triggeredEvents),static_cast<unsigned long long>(event.work.completedEvents));
}

bool NCManager::WaitEDM173SameThread()
{
    const bool wasParentReleasePending=m_edm233.releasePending;
    const bool wasHistoryParentReleasePending=m_edm237.releasePending;
    const bool wasMixedParentReleasePending=m_edm234.releasePending;
    ServiceEDM94SameThread();auto& s=m_edm94;const auto& guard=s.feedback;
    if(wasParentReleasePending&&!m_edm233.releasePending&&m_state==NCState::RUN&&s.cancelled)
    {RejectEDM94SameThread("EDM233_PARENT_RELEASE_CANCELLED",m_gapDryRun.sourceLine);return false;}
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
    if(m_edm233.releasePending)
    {
        if(ConsumeEDM233ParentReleaseSameThread())return true;
        if(m_edm233.releasePending&&++s.polls>1000U)RejectEDM94SameThread("EDM233_PARENT_RELEASE_TIMEOUT",m_gapDryRun.sourceLine);
        return false;
    }
    if(!s.latched||!EDM173::IsArm(s.gapHistoryArm)||s.cancelled||s.releaseRequested||m_state!=NCState::RUN) return false;
    if(s.finishPending&&!IsEDM200AutoHoldCheckpointCompleteSameThread())
    {RejectEDM94SameThread("EDM200_FINISH_CHECKPOINT_NOT_COMPLETED",m_gapDryRun.sourceLine);return false;}
    if(s.armPending)
    {
        EDM126::MotionFeedback source{};
        if(guard.identity.session==s.session&&guard.identity.epoch==s.armEpoch&&guard.identity.ownerGeneration==s.lease.generation&&
            guard.currentIdentity.session==s.session&&guard.currentIdentity.epoch==s.armEpoch&&
            guard.currentIdentity.epoch==m_motion.GetCurrentExecutionEpoch()&&guard.currentIdentity.ownerGeneration==s.lease.generation&&
            guard.active&&guard.armAccepted&&guard.armedReady&&!guard.forceZero&&!guard.faulted&&!guard.cancelled&&!guard.handoffZero&&
            (!EDM173::UsesNominalMovementMask(m_edm130.recipe)||(guard.boundedProgram&&
                guard.config.commissioningProfile==m_edm130.recipe.commissioningProfile&&
                guard.requiredMovementMask==E156RequiredMovementMask(s,m_edm130.recipe)))&&
            guard.boundaryReadyCycles>=EDM94::BoundaryReadyCycles&&!guard.boundaryNotReadyMask&&
            m_motion.ReadEDM126Feedback(source)&&source.enabled&&source.active&&!source.faulted&&source.fixture==14U&&
            source.session==s.session&&source.armEpoch==s.armEpoch&&source.liveEpoch==s.armEpoch&&source.ownerGeneration==s.lease.generation&&source.sourceReady&&
            source.sourceReason==EDM88::Reason::None&&!source.authorizedRows&&!source.completedRows&&!source.rowActive)
        {
            s.armPending=false;s.ready=s.everArmed=true;s.polls=0U;
            // EDM212_CENTER_FOLLOWING_POLICY_BEGIN
            // NC-only, once after the actual RT arm receipt is accepted.
            if(EDM173::IsCenterRecipe(m_edm130.recipe))
            {
                const bool enabled=EDM94::EmptyLoadCenterDynamicFollowing;
                PrintNCG76DiagnosticLine("[EDM212] event=CENTER_FOLLOWING_POLICY session=%llu mode=%u enabled=%u runtimeTripMm=%.9g axes=XYZ phases=NOMINAL_MOVE_DWELL_FINISH commandCorridor=STRICT actualAllowance=RUNTIME_TRIP_MM arm=STRICT settle=STRICT stop=STRICT reset=STRICT source=UNCHANGED outputCap=UNCHANGED discharge=0\n",
                    static_cast<unsigned long long>(s.session),EDM173::ModeForArm(s.gapHistoryArm),enabled?1U:0U,
                    enabled?EDM94::CenterDynamicFollowingMm:0.0);
            }
            // EDM212_CENTER_FOLLOWING_POLICY_END
            RtPrintf("[EDM173] event=ARMED session=%llu source=AUTO_WAVE commonGAP=1 E5=1 discharge=0\n",static_cast<unsigned long long>(s.session));
            return true;
        }
    }
    else if(s.finishPending&&m_edm173->completed&&E156FinishTargetMatches(s,m_edm130.recipe,guard,m_edm114.retained)&&
        guard.identity.session==s.session&&guard.identity.epoch==s.armEpoch&&guard.identity.ownerGeneration==s.lease.generation&&
        guard.active&&guard.pass&&guard.stopped&&guard.forceZero&&guard.finishRequested&&!guard.faulted&&!guard.cancelled&&
        EDM94::MovementProven(guard,E156RequiredMovementMask(s,m_edm130.recipe))&&guard.sentFrames>=200ULL&&guard.requiredCycles>=200U&&
        guard.settledCycles>=guard.requiredCycles&&guard.lastSentTick&&guard.lastSentTick<=guard.lastTick&&m_motion.IsGroupDone())
    {s.finishPending=false;s.complete=s.terminalLogged=true;
        if(m_edm237.authorized&&!FinishEDM237ParentSameThread())return false;
            if(m_edm234.authorized&&!FinishEDM234ParentSameThread())return false;
        if(EDM130::IsG76ParentSequence233(m_edm130.recipe))
        {
            if(!IsEDM233SequenceScopeCurrentSameThread()||s.armPC!=m_edm233.firstArmPC+(m_edm233.secondStarted?3:0))
            {RejectEDM94SameThread("EDM233_FINISHED_SEQUENCE_SCOPE",m_gapDryRun.sourceLine);return false;}
            if(!m_edm233.secondStarted)
            {
                if(m_edm233.firstReleased||s.session!=m_edm233.firstSession||
                    !m_motion.RequestEDM233ParentRelease(s.session,s.armEpoch,s.lease))
                {RejectEDM94SameThread("EDM233_PARENT_RELEASE_REQUEST",m_gapDryRun.sourceLine);return false;}
                m_edm233.releaseFloorTick=guard.lastSentTick;m_edm233.releasePending=true;s.polls=0U;
                PrintNCG76DiagnosticLine("[EDM233] event=PARENT_FINISH parent=1 session=%llu children=25 E5completed=1 next=EXACT_ZERO_RETIREMENT discharge=0\n",static_cast<unsigned long long>(s.session));
                return false;
            }
            if(!m_edm233.firstReleased||s.session<=m_edm233.firstSession)
            {RejectEDM94SameThread("EDM233_FIRST_PARENT_RECEIPT_REQUIRED",m_gapDryRun.sourceLine);return false;}
            PrintNCG76DiagnosticLine("[EDM233] event=SEQUENCE_COMPLETE parents=2 children=50 E5completed=2 firstSession=%llu secondSession=%llu firstReleaseTick=%llu E5policy=FRESH_PER_PARENT discharge=0\n",
                static_cast<unsigned long long>(m_edm233.firstSession),static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(m_edm233.releaseTick));
        }
        LogEDM94SameThread("PASS",EDM130::IsG76ParentSequence233(m_edm130.recipe)?
            "EDM233_TWO_G76_PARENTS_GAP_E5_WAIT_M30":EDM130::IsG76CenterSecondReturnHold232(m_edm130.recipe)?
            "EDM232_G76_GAP_E5_TWO_EVENTS_RETURN_HOLD_WAIT_M30":EDM130::IsG76CenterSecondFlushHold231(m_edm130.recipe)?
            "EDM231_G76_GAP_E5_TWO_EVENTS_HOLD_WAIT_M30":EDM130::IsG76CenterRepeat230(m_edm130.recipe)?
            "EDM230_G76_GAP_E5_TWO_EVENTS_WAIT_M30":"EDM173_G76_GAP_E5_ONE_EVENT_WAIT_M30");return true;}
    if(++s.polls>1000U) RejectEDM94SameThread("EDM173_WAIT_TIMEOUT",m_gapDryRun.sourceLine);
    return false;
}
