// EDM267 FIX1: independent NC compilation unit; original function bodies preserved.
#include "NCManager_GapInternal.h"

using namespace NCGapDetail;

bool NCManager::IsEDM94TranslationBracketAllowedSameThread(const NCBlock& block) noexcept
{
    if(block.has('P')&&block.val('P')==274.0&&block.has('L')&&(block.val('L')==234.0||block.val('L')==235.0||block.val('L')==236.0||block.val('L')==237.0||block.val('L')==238.0||block.val('L')==240.0||block.val('L')==241.0||block.val('L')==242.0||block.val('L')==243.0||block.val('L')==244.0||block.val('L')==245.0||block.val('L')==246.0||block.val('L')==247.0||block.val('L')==248.0))
    {
        if(!IsEDM234BracketShapeValid(block)||m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||
            !IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot())||CoordSys.GetTranslationSnapshot().distanceMode!=91||
            AlarmManager::GetInstance().HasAlarm()||m_motion.HasPendingSafetyOrRecoveryRequests()||
            !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))return false;
        const int pc=GetActiveDispatchPC();
        if(m_edm237.authorized)return IsEDM237NextArmCurrentSameThread(pc,274U);
        if(m_edm234.authorized&&m_edm234.program==236U)return IsEDM236NextArmCurrentSameThread(pc,274U);
        if(m_edm234.authorized&&pc==m_edm234.firstArmPC+3)return IsEDM234NextArmCurrentSameThread(pc,274U);
        const auto& c=m_edm149Confirmation;
        return !m_edm94.latched&&c.confirmed&&!c.waiting&&c.arm==274U&&c.armPC==pc&&
            c.run==m_pathCoreLiveBookkeeping.currentRunToken&&c.cache==GetBaseProgramCache().GetGeneration()&&
            c.startRequests==m_startDiagnosticRequests&&c.dispatch&&m_currentExecutingBlockDispatchId>c.dispatch;
    }
    if (block.has('P') && (block.val('P')==292.0 || block.val('P')==293.0 || block.val('P')==294.0 || block.val('P')==295.0 || block.val('P')==296.0 || block.val('P')==297.0 || block.val('P')==298.0 || block.val('P')==299.0 || block.val('P')==300.0 || block.val('P')==301.0)) return IsEDM173BracketBlockSameThread(block);
    if (block.has('P') && block.val('P') == 131.0)
        return IsEDM116SelectorCurrentSameThread(block);
    if (block.isEmpty || block.isGoto || block.isBlockSkip || !block.hasG || block.gCode != 180 ||
        block.gCount != 1 || block.gCodes[0] != 180 || block.mCount != 0 || !block.has('P') ||
        (block.val('P') != 85.0 && block.val('P') != 86.0 &&
            block.val('P') != 87.0 && block.val('P') != 88.0 &&
            block.val('P') != 89.0 && block.val('P') != 90.0 &&
            block.val('P') != 91.0 && block.val('P') != 92.0 &&
            block.val('P') != 93.0 && block.val('P') != 94.0 &&
            block.val('P') != 95.0 && block.val('P') != 96.0 &&
            block.val('P') != 97.0 && block.val('P') != 98.0 &&
            block.val('P') != 99.0 && block.val('P') != 100.0 &&
            block.val('P') != 101.0 && block.val('P') != 102.0 &&
            block.val('P') != 103.0 && block.val('P') != 104.0 &&
            block.val('P') != 105.0 && block.val('P') != 106.0 &&
            block.val('P') != 107.0 && block.val('P') != 108.0 &&
            block.val('P') != 109.0 && block.val('P') != 110.0 &&
            block.val('P') != 111.0 && block.val('P') != 112.0 &&
            block.val('P') != 113.0 && block.val('P') != 114.0 && block.val('P') != 115.0 && block.val('P') != 116.0 && block.val('P') != 117.0 && block.val('P') != 118.0 && block.val('P') != 119.0 && block.val('P') != 120.0 && block.val('P') != 121.0 && block.val('P') != 122.0 && block.val('P') != 123.0 && block.val('P') != 124.0 && block.val('P') != 125.0 && block.val('P') != 126.0 && block.val('P') != 127.0 && block.val('P') != 128.0 && block.val('P') != 129.0 && block.val('P') != 130.0 && block.val('P') != 132.0 && block.val('P') != 133.0 && block.val('P') != 134.0 && block.val('P') != 135.0 && block.val('P') != 136.0 && block.val('P') != 137.0 && block.val('P') != 138.0 && block.val('P') != 139.0 && block.val('P') != 140.0 && block.val('P') != 141.0 && block.val('P') != 142.0 && block.val('P') != 143.0 && block.val('P') != 144.0 && block.val('P') != 145.0 && block.val('P') != 146.0 && block.val('P') != 147.0 && block.val('P') != 148.0 && block.val('P') != 149.0 && block.val('P') != 150.0 && block.val('P') != 151.0 && block.val('P') != 152.0 && block.val('P') != 153.0 && block.val('P') != 154.0 && block.val('P') != 155.0 && block.val('P') != 156.0 && block.val('P') != 157.0 && block.val('P') != 158.0 && block.val('P') != 159.0 && block.val('P') != 160.0 && block.val('P') != 161.0 && block.val('P') != 162.0 && block.val('P') != 163.0 && block.val('P') != 164.0 && block.val('P') != 165.0 && block.val('P') != 166.0 && block.val('P') != 167.0 && block.val('P') != 168.0 && block.val('P') != 169.0 && block.val('P') != 170.0 && block.val('P') != 171.0 && block.val('P') != 172.0 && block.val('P') != 173.0 && block.val('P') != 174.0 && block.val('P') != 175.0 && block.val('P') != 176.0 && block.val('P') != 177.0 && block.val('P') != 178.0 && block.val('P') != 179.0 && block.val('P') != 180.0 && block.val('P') != 181.0 && block.val('P') != 182.0 && block.val('P') != 183.0 && block.val('P') != 184.0 && block.val('P') != 185.0 && block.val('P') != 186.0 && block.val('P') != 187.0 && block.val('P') != 188.0 && block.val('P') != 189.0 && block.val('P') != 190.0 && block.val('P') != 191.0 && block.val('P') != 192.0 && block.val('P') != 193.0 && block.val('P') != 194.0 && block.val('P') != 195.0 && block.val('P') != 196.0 && block.val('P') != 197.0 && block.val('P') != 198.0 && block.val('P') != 199.0 && block.val('P') != 200.0 && block.val('P') != 201.0 && block.val('P') != 202.0 && block.val('P') != 203.0 && block.val('P') != 204.0 && block.val('P') != 205.0 && block.val('P') != 206.0 && block.val('P') != 207.0 && block.val('P') != 208.0 && block.val('P') != 209.0 && block.val('P') != 210.0 && block.val('P') != 211.0 && block.val('P') != 212.0 && block.val('P') != 213.0 && block.val('P') != 214.0 && block.val('P') != 215.0 && block.val('P') != 216.0 && block.val('P') != 217.0 && block.val('P') != 218.0 && block.val('P') != 219.0 && block.val('P') != 220.0 && block.val('P') != 221.0 && block.val('P') != 222.0 && block.val('P') != 223.0 && block.val('P') != 224.0 && block.val('P') != 225.0 && block.val('P') != 226.0 && block.val('P') != 227.0 && block.val('P') != 228.0 && block.val('P') != 229.0 && block.val('P') != 230.0 && block.val('P') != 231.0 && block.val('P') != 232.0 && block.val('P') != 233.0 && block.val('P') != 234.0 && block.val('P') != 236.0 && block.val('P') != 238.0 && block.val('P') != 240.0 && block.val('P') != 242.0 && block.val('P') != 244.0 && block.val('P') != 246.0 && block.val('P') != 248.0 && block.val('P') != 250.0 && block.val('P') != 252.0 && block.val('P') != 235.0 && block.val('P') != 237.0 && block.val('P') != 239.0 && block.val('P') != 241.0 && block.val('P') != 243.0 && block.val('P') != 245.0 && block.val('P') != 247.0 && block.val('P') != 249.0 && block.val('P') != 251.0 && block.val('P') != 253.0 && block.val('P') != 254.0 && block.val('P') != 255.0 && block.val('P') != 256.0 && block.val('P') != 257.0 && block.val('P') != 258.0 && block.val('P') != 259.0 && block.val('P') != 260.0 && block.val('P') != 261.0 && block.val('P') != 262.0 && block.val('P') != 263.0 && block.val('P') != 264.0 && block.val('P') != 265.0 && block.val('P') != 266.0 && block.val('P') != 267.0 && block.val('P') != 268.0 && block.val('P') != 269.0 && block.val('P') != 270.0 && block.val('P') != 271.0 && block.val('P') != 272.0 && block.val('P') != 273.0 && block.val('P') != 274.0 && block.val('P') != 275.0 && block.val('P') != 276.0 && block.val('P') != 277.0 && block.val('P') != 278.0 && block.val('P') != 279.0 && block.val('P') != 280.0 && block.val('P') != 281.0 && block.val('P') != 282.0 && block.val('P') != 283.0 && block.val('P') != 284.0 && block.val('P') != 286.0 && block.val('P') != 288.0 && block.val('P') != 290.0 && block.val('P') != 285.0 && block.val('P') != 287.0 && block.val('P') != 289.0 && block.val('P') != 291.0) || m_mode != NCOperationMode::MEMORY ||
        m_state != NCState::RUN || !m_macroStack.empty() || m_isG66Active || Homing.IsActive() ||
        !IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) ||
        CoordSys.GetTranslationSnapshot().distanceMode != 91 || !m_motion.IsGroupDone() ||
        m_motion.HasPendingSafetyOrRecoveryRequests()) return false;
    const bool historyArm = block.val('P') == 158.0 || block.val('P') == 160.0;
    const bool noiseArm = block.val('P') == 93.0;
    const bool burstArm = block.val('P') == 95.0;
    const bool continuousArm = block.val('P') == 97.0;
    const bool enduranceArm = block.val('P') == 99.0;
    const bool pairedArm = block.val('P') == 101.0;
    const bool parameterizedArm = block.val('P') == 109.0 || block.val('P') == 111.0 || block.val('P') == 113.0 || block.val('P') == 115.0 || block.val('P') == 117.0 || block.val('P') == 119.0 || block.val('P') == 121.0 || block.val('P') == 123.0 || block.val('P') == 125.0 || block.val('P') == 127.0 || block.val('P') == 129.0 || block.val('P') == 132.0 || block.val('P') == 134.0 || block.val('P') == 136.0 || block.val('P') == 138.0 || block.val('P') == 140.0 || block.val('P') == 142.0 || block.val('P') == 144.0 || block.val('P') == 146.0 || block.val('P') == 148.0 || block.val('P') == 150.0 || block.val('P') == 152.0 || block.val('P') == 154.0 || block.val('P') == 156.0 || block.val('P') == 162.0 || block.val('P') == 164.0 || block.val('P') == 166.0 || block.val('P') == 168.0 || block.val('P') == 170.0 || block.val('P') == 172.0 || block.val('P') == 174.0 || block.val('P') == 176.0 || block.val('P') == 178.0 || block.val('P') == 180.0 || block.val('P') == 182.0 || block.val('P') == 184.0 || block.val('P') == 186.0 || block.val('P') == 188.0 || block.val('P') == 190.0 || block.val('P') == 192.0 || block.val('P') == 194.0 || block.val('P') == 196.0 || block.val('P') == 198.0 || block.val('P') == 200.0 || block.val('P') == 202.0 || block.val('P') == 204.0 || block.val('P') == 206.0 || block.val('P') == 208.0 || block.val('P') == 210.0 || block.val('P') == 212.0 || block.val('P') == 214.0 || block.val('P') == 216.0 || block.val('P') == 218.0 || block.val('P') == 220.0 || block.val('P') == 222.0 || block.val('P') == 224.0 || block.val('P') == 226.0 || block.val('P') == 228.0 || block.val('P') == 230.0 || block.val('P') == 232.0 || block.val('P') == 234.0 || block.val('P') == 236.0 || block.val('P') == 238.0 || block.val('P') == 240.0 || block.val('P') == 242.0 || block.val('P') == 244.0 || block.val('P') == 246.0 || block.val('P') == 248.0 || block.val('P') == 250.0 || block.val('P') == 252.0 || block.val('P') == 254.0 || block.val('P') == 256.0 || block.val('P') == 258.0 || block.val('P') == 260.0 || block.val('P') == 262.0 || block.val('P') == 264.0 || block.val('P') == 266.0 || block.val('P') == 268.0 || block.val('P') == 270.0 || block.val('P') == 272.0 || block.val('P') == 274.0 || block.val('P') == 276.0 || block.val('P') == 278.0 || block.val('P') == 280.0 || block.val('P') == 282.0 || block.val('P') == 284.0 || (block.val('P') == 286.0 || block.val('P') == 288.0 || block.val('P') == 290.0);
    const bool flushArm = block.val('P') == 103.0 || block.val('P') == 105.0 || block.val('P') == 107.0 || parameterizedArm;
    const bool tuningArm = historyArm || block.val('P') == 91.0 || noiseArm || burstArm || continuousArm || enduranceArm || pairedArm || flushArm;
    // EDM163 FIX1: this outer G91 bracket runs before the G180 handler.
    // H1 also admits the exact XYZ 2 mm visible-flush profile; 5 mm stays excluded.
    const bool holdPathArm = EDM149::IsFlushHoldTestArm(static_cast<unsigned>(block.val('P'))) ||
            EDM149::IsManualVoltageTestArm(static_cast<unsigned>(block.val('P')));
    if (block.has('H') && (!holdPathArm || block.val('H') != 1.0)) return false;
    if (block.has('L') && (!holdPathArm || !block.has('H') || block.val('H') != 1.0 ||
            (block.val('L') != 1.0 && block.val('L') != 2.0 && block.val('L') != 3.0) ||
            !EDM130::ValidFlushHoldSelection(static_cast<unsigned>(block.val('P')), static_cast<unsigned>(block.val('L'))))) return false;
    for (char a = 'A'; a <= 'Z'; ++a)
        if (block.has(a) && ((a != 'N' && a != 'P' && ((a != 'H' && a != 'L') || !holdPathArm) &&
            (!tuningArm || (a != 'Q' && a != 'R' && a != 'S' &&
                    (!parameterizedArm || (a != 'I' && a != 'J' && a != 'K'))))) ||
            !std::isfinite(block.val(a)))) return false;
    const int pc = GetActiveDispatchPC();
    const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    EDM97::Tuning tuning{};
    EDM107::FlushProfile parameters{};
    if (!authored || !E94Matches(authored->parsedBlock, block) ||
        !(historyArm ? E126Profile(authored->parsedBlock, static_cast<unsigned>(block.val('P')), tuning) : parameterizedArm ? E108ProfileOnlyNPQRSIJK(authored->parsedBlock, static_cast<unsigned>(block.val('P')), tuning, parameters) :
            flushArm ? E106FlushProfile(authored->parsedBlock,
                block.val('P') == 105.0 ? 105U : block.val('P') == 107.0 ? 107U : 0U, tuning) :
            pairedArm ? E104ProfileOnlyNPQRS(authored->parsedBlock, tuning) :
            enduranceArm ? E102ProfileOnlyNPQRS(authored->parsedBlock, tuning) :
            continuousArm ? E100ProfileOnlyNPQRS(authored->parsedBlock, tuning) :
            burstArm ? E99ProfileOnlyNPQRS(authored->parsedBlock, tuning) :
            noiseArm ? E98ProfileOnlyNPQRS(authored->parsedBlock, tuning) :
            tuningArm ? E97ProfileOnlyNPQRS(authored->parsedBlock, tuning) :
            E94Profile(authored->parsedBlock, static_cast<unsigned>(block.val('P')))) ||
        !IsPathCoreLiveNativeConfigCurrentSameThread() ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        (CoordSys.IsTranslationRunFrozen() && (!CoordSys.IsTranslationRunCurrent() ||
            !IsFixedTranslationTravelCurrentSameThread() || !IsXYZCUVFeedProfileCurrentSameThread() ||
            !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot())))) return false;
    const bool arm = block.val('P') == 85.0 || block.val('P') == 87.0 ||
        block.val('P') == 89.0 || tuningArm;
    return arm ? !m_edm94.latched :
        m_edm94.latched && m_edm94.ready && !m_edm94.cancelled &&
        block.val('P') == (m_edm94.gapHistoryServo ? m_edm94.gapHistoryArm + 1U : m_edm94.gapFlushProfile ? (m_edm94.gapFlushInterruptArm ? static_cast<double>(m_edm94.gapFlushInterruptArm + 1U) : m_edm94.gapModeFlush ? (m_edm94.gapModeInterruption ? static_cast<double>(m_edm94.gapModeInterruptionArm + 1U) : E116FinishP(m_edm94.gapModeFixture)) : m_edm94.gapCurveB2 ? 128.0 : m_edm94.gapPhysicalCurve ? 126.0 : m_edm94.gapCurveRuntime ? 124.0 : m_edm94.gapCurveGeometry ? 122.0 : m_edm94.gapB2Repeat ? 120.0 : m_edm94.gapB2CrossSegment ? 118.0 : m_edm94.gapCommonPath ? 116.0 : m_edm94.gapFlushParameterized ? 110.0 : 104.0) : m_edm94.gapPairedProfile ? 102.0 : m_edm94.gapEnduranceProfile ? 100.0 : m_edm94.gapContinuousProfile ? 98.0 : m_edm94.gapBurstProfile ? 96.0 : m_edm94.gapNoiseProfile ? 94.0 : m_edm94.gapGainProfile ? 92.0 : m_edm94.gapSegmentProfile ? 90.0 :
            m_edm94.gapBoundaryProfile ? 88.0 : 86.0) &&
        pc == m_edm94.pc[m_edm94.moves] &&
        m_edm94.cache == GetBaseProgramCache().GetGeneration() &&
        m_edm94.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_motion.IsMotionOwnerLeaseCurrent(m_edm94.lease);
}

bool NCManager::InspectEDM94ProgramSameThread() noexcept
{
    const auto& cache = GetBaseProgramCache();
    if (m_edm94.inspectedCache == cache.GetGeneration()) return m_edm94.program;
    if (m_edm94.latched) return true; // Live scope is never overwritten by a new image.
    m_edm94.inspectedCache = cache.GetGeneration();
    m_edm94.program = false;
    // NC-only once per cache generation. Inspect expressions conservatively:
    // A dynamic G180 selector may select either physical bracket.
    for (std::size_t i = 0U; i < cache.Size(); ++i)
    {
        const auto* line = cache.TryGetLine(static_cast<int>(i));
        if (!line) continue;
        const auto& p = line->parsedBlock;
        if (!p.has('P')) continue;
        double selector = 0.0;
        const bool fixedSelector = E94Number(p.expression('P'), selector);
        const bool selected = fixedSelector &&
            (selector == 85.0 || selector == 86.0 || selector == 87.0 || selector == 88.0 ||
                selector == 89.0 || selector == 90.0 || selector == 91.0 || selector == 92.0 ||
                selector == 93.0 || selector == 94.0 || selector == 95.0 || selector == 96.0 ||
                selector == 97.0 || selector == 98.0 || selector == 99.0 || selector == 100.0 ||
                selector == 101.0 || selector == 102.0 || selector == 103.0 || selector == 104.0 ||
                selector == 105.0 || selector == 106.0 || selector == 107.0 || selector == 108.0 ||
                selector == 109.0 || selector == 110.0 ||
                selector == 111.0 || selector == 112.0 || selector == 113.0 || selector == 114.0 || selector == 115.0 || selector == 116.0 || selector == 117.0 || selector == 118.0 || selector == 119.0 || selector == 120.0 || selector == 121.0 || selector == 122.0 || selector == 123.0 || selector == 124.0 || selector == 125.0 || selector == 126.0 || selector == 127.0 || selector == 128.0 || selector == 129.0 || selector == 130.0 || selector == 131.0 || selector == 132.0 || selector == 133.0 || selector == 134.0 || selector == 135.0 || selector == 136.0 || selector == 137.0 || selector == 138.0 || selector == 139.0 || selector == 140.0 || selector == 141.0 || selector == 142.0 || selector == 143.0 || selector == 144.0 || selector == 145.0 || selector == 146.0 || selector == 147.0 || selector == 148.0 || selector == 149.0 || selector == 150.0 || selector == 151.0 || selector == 152.0 || selector == 153.0 || selector == 154.0 || selector == 155.0 || selector == 156.0 || selector == 157.0 || selector == 158.0 || selector == 159.0 || selector == 160.0 || selector == 161.0 || selector == 162.0 || selector == 163.0 || selector == 164.0 || selector == 165.0 || selector == 166.0 || selector == 167.0 || selector == 168.0 || selector == 169.0 || selector == 170.0 || selector == 171.0 || selector == 172.0 || selector == 173.0 || selector == 174.0 || selector == 175.0 || selector == 176.0 || selector == 177.0 || selector == 178.0 || selector == 179.0 || selector == 180.0 || selector == 181.0 || selector == 182.0 || selector == 183.0 || selector == 184.0 || selector == 185.0 || selector == 186.0 || selector == 187.0 || selector == 188.0 || selector == 189.0 || selector == 190.0 || selector == 191.0 || selector == 192.0 || selector == 193.0 || selector == 194.0 || selector == 195.0 || selector == 196.0 || selector == 197.0 || selector == 198.0 || selector == 199.0 || selector == 200.0 || selector == 201.0 || selector == 202.0 || selector == 203.0 || selector == 204.0 || selector == 205.0 || selector == 206.0 || selector == 207.0 || selector == 208.0 || selector == 209.0 || selector == 210.0 || selector == 211.0 || selector == 212.0 || selector == 213.0 || selector == 214.0 || selector == 215.0 || selector == 216.0 || selector == 217.0 || selector == 218.0 || selector == 219.0 || selector == 220.0 || selector == 221.0 || selector == 222.0 || selector == 223.0 || selector == 224.0 || selector == 225.0 || selector == 226.0 || selector == 227.0 || selector == 228.0 || selector == 229.0 || selector == 230.0 || selector == 231.0 || selector == 232.0 || selector == 233.0 || selector == 234.0 || selector == 236.0 || selector == 238.0 || selector == 240.0 || selector == 242.0 || selector == 244.0 || selector == 246.0 || selector == 248.0 || selector == 250.0 || selector == 252.0 || selector == 235.0 || selector == 237.0 || selector == 239.0 || selector == 241.0 || selector == 243.0 || selector == 245.0 || selector == 247.0 || selector == 249.0 || selector == 251.0 || selector == 253.0 || selector == 254.0 || selector == 255.0 || selector == 256.0 || selector == 257.0 || selector == 258.0 || selector == 259.0 || selector == 260.0 || selector == 261.0 || selector == 262.0 || selector == 263.0 || selector == 264.0 || selector == 265.0 || selector == 266.0 || selector == 267.0 || selector == 268.0 || selector == 269.0 || selector == 270.0 || selector == 271.0 || selector == 272.0 || selector == 273.0 || selector == 274.0 || selector == 275.0 || selector == 276.0 || selector == 277.0 || selector == 278.0 || selector == 279.0 || selector == 280.0 || selector == 281.0 || selector == 282.0 || selector == 283.0 || selector == 284.0 || selector == 285.0 || selector == 286.0 || selector == 287.0 || selector == 288.0 || selector == 289.0 || selector == 290.0 || selector == 291.0 || selector == 292.0 || selector == 293.0 || selector == 294.0 || selector == 295.0 || selector == 296.0 || selector == 297.0 || selector == 298.0 || selector == 299.0 || selector == 300.0 || selector == 301.0);
        bool g180 = false;
        for (int j = 0; j < p.gCount && j < NC_MAX_G_CODES_PER_BLOCK; ++j)
        {
            double code = 0.0;
            const bool fixedG = E94Number(p.gExpressions[j], code);
            g180 = g180 || (fixedG && code == 180.0) || (!fixedG && selected);
        }
        if (g180 && (!fixedSelector || selected))
        { m_edm94.program = true; break; }
    }
    return m_edm94.program;
}

bool NCManager::PreflightEDM94ProgramSameThread(int armPC) noexcept
{
    const auto& cache = GetBaseProgramCache();
    if (armPC < 0 || cache.Size() == 0U || cache.Size() > 128U) return false;
    // EDM237_PREFLIGHT_DISPATCH_BEGIN
    const auto* historyArm=cache.TryGetLine(armPC);
    if(historyArm&&E237ArmSequence(historyArm->parsedBlock,274U))return PreflightEDM237OrdinarySameThread(armPC);
    // EDM237_PREFLIGHT_DISPATCH_END
    // EDM234_PREFLIGHT_DISPATCH_BEGIN
    const auto* mixedArm=cache.TryGetLine(armPC);
    if(mixedArm&&E234ArmSequence(mixedArm->parsedBlock,274U))return PreflightEDM234LineSameThread(armPC);
    // EDM234_PREFLIGHT_DISPATCH_END
    m_edm94.moves = m_edm94.next = 0U;
    m_edm94.authoredGCode.fill(0U);
    m_edm127 = EDM127NCState{};
    m_edm129 = EDM129NCState{};
    m_edm130 = EDM130NCState{};
    m_edm140 = EDM140NCState{};
    m_edm140.pc.fill(-1);
    for (auto& row : m_edm141.rows) row = EDM141::Row{};
    m_edm141.assignmentValues.fill(0.0);
    m_edm141.plannedRows = m_edm141.expressionRows = m_edm141.verifiedRows = 0U;
    MacroEngine edm141Shadow; MacroParser edm141Evaluator(edm141Shadow);
    std::array<double, 2U> edm141Locals{}; unsigned edm141Defined = 0U;
    m_edm94.gapHistoryServo = false; m_edm94.gapHistoryMode = false; m_edm94.gapPhysicalAd = false; m_edm94.gapWorkTime = false; m_edm94.gapHistoryArm = 0U;
    m_edm94.gapBoundaryProfile = false;
    m_edm94.gapSegmentProfile = false;
    m_edm94.gapGainProfile = false;
    m_edm94.gapNoiseProfile = false;
    m_edm94.gapBurstProfile = false;
    m_edm94.gapContinuousProfile = false;
    m_edm94.gapEnduranceProfile = false;
    m_edm94.gapPairedProfile = false;
    m_edm94.gapFlushProfile = false;
    m_edm94.gapFlushParameterized = false;
    m_edm94.gapCommonPath = false;
    m_edm94.gapB2CrossSegment = false;
    m_edm94.gapB2Repeat = false;
    m_edm94.gapCurveGeometry = false;
    m_edm94.gapCurveRuntime = false;
    m_edm94.gapPhysicalCurve = false; m_edm94.gapCurveB2 = false; m_edm94.gapModeFlush = false;
    m_edm94.gapModeFixture = EDM116::Fixture::Canonical116;
    m_edm94.gapModeInterruption = false;
    m_edm94.gapModeInterruptionArm = 0U;
    m_edm94.selectorPC.fill(-1);
    m_edm94.gapFlushParameters = EDM107::FlushProfile{};
    m_edm94.gapFlushInterruptArm = 0U;
    m_edm94.gapFlushInterruptRequested = false;
    m_edm94.gapFlushInterruptSampleTick = m_edm94.gapFlushInterruptEpoch = 0ULL;
    m_edm94.gapTuning = EDM97::Tuning{};
    std::array<double, 6U> total{};
    unsigned phase = 0U;
    bool modePending = false;
    EDM127::Selection modal{}; modal.mode = EDM116::Mode::B0;
    int modalOriginPC = armPC; std::uint64_t modalRevision = 1ULL;
    double nominalSeconds = 0.0;
    for (std::size_t i = 0U; i < cache.Size(); ++i)
    {
        const auto* line = cache.TryGetLine(static_cast<int>(i));
        if (!line) return false;
        const auto& b = line->parsedBlock;
        if (!EDM77::Detail::Clean(b) &&
            !(phase == 1U && E141MotionArm(m_edm94.gapHistoryArm) && EDM141::Shape(b)))
        {
            EDM140::Specification specification{};
            if (phase != 1U || !E140AssignmentArm(m_edm94.gapHistoryArm) ||
                !EDM140::Parse(b, specification) || !m_edm94.moves ||
                m_edm94.moves >= EDM114::RequiredRows || m_edm140.count >= EDM140::AssignmentCapacity)
                return false;
            unsigned defined = 0U;
            for (unsigned prior = 0U; prior < m_edm140.count; ++prior)
                defined |= 1U << (m_edm140.specifications[prior].target - 1U);
            if ((specification.readMask & ~defined) != 0U) return false;
            const unsigned slot = m_edm140.count++;
            if (E141MotionArm(m_edm94.gapHistoryArm))
            {
                NCMacroAssignmentCommit assignment{}; NCExpressionResolveError error{};
                if (!NCExpressionResolver::ResolveAssignment(b, edm141Evaluator, assignment, error) ||
                    !EDM140::ValidResult(specification, assignment.prefix, assignment.index, assignment.value)) return false;
                // This private shadow never aliases the live MacroSys state.
                edm141Shadow.SetVar(assignment.prefix, assignment.index, assignment.value);
                edm141Locals[static_cast<unsigned>(assignment.index - 1)] = assignment.value;
                edm141Defined |= 1U << static_cast<unsigned>(assignment.index - 1);
                m_edm141.assignmentValues[slot] = assignment.value;
            }
            m_edm140.specifications[slot] = specification;
            m_edm140.pc[slot] = static_cast<int>(i);
            m_edm140.afterRows[slot] = m_edm94.moves;
            continue;
        }
        if (EDM77::Detail::Empty(b)) continue;
        if (phase == 0U)
        {
            EDM97::Tuning tuning{};
            EDM107::FlushProfile parameters{};
            const unsigned historyModeArm = E108ProfileOnlyNPQRSIJK(b, 166U, tuning, parameters) ? 166U :
                E108ProfileOnlyNPQRSIJK(b, 168U, tuning, parameters) ? 168U :
                E108ProfileOnlyNPQRSIJK(b, 170U, tuning, parameters) ? 170U :
                E108ProfileOnlyNPQRSIJK(b, 172U, tuning, parameters) ? 172U :
                E108ProfileOnlyNPQRSIJK(b, 174U, tuning, parameters) ? 174U :
                E108ProfileOnlyNPQRSIJK(b, 176U, tuning, parameters) ? 176U :
                E108ProfileOnlyNPQRSIJK(b, 178U, tuning, parameters) ? 178U :
                E108ProfileOnlyNPQRSIJK(b, 180U, tuning, parameters) ? 180U :
                E108ProfileOnlyNPQRSIJK(b, 182U, tuning, parameters) ? 182U :
                E108ProfileOnlyNPQRSIJK(b, 184U, tuning, parameters) ? 184U :
                E108ProfileOnlyNPQRSIJK(b, 186U, tuning, parameters) ? 186U :
                E108ProfileOnlyNPQRSIJK(b, 188U, tuning, parameters) ? 188U :
                E108ProfileOnlyNPQRSIJK(b, 190U, tuning, parameters) ? 190U :
                E108ProfileOnlyNPQRSIJK(b, 192U, tuning, parameters) ? 192U :
                E108ProfileOnlyNPQRSIJK(b, 194U, tuning, parameters) ? 194U :
                E108ProfileOnlyNPQRSIJK(b, 196U, tuning, parameters) ? 196U :
                E108ProfileOnlyNPQRSIJK(b, 198U, tuning, parameters) ? 198U :
                E108ProfileOnlyNPQRSIJK(b, 200U, tuning, parameters) ? 200U :
                E108ProfileOnlyNPQRSIJK(b, 202U, tuning, parameters) ? 202U :
                E108ProfileOnlyNPQRSIJK(b, 204U, tuning, parameters) ? 204U :
                E108ProfileOnlyNPQRSIJK(b, 206U, tuning, parameters) ? 206U :
                E108ProfileOnlyNPQRSIJK(b, 208U, tuning, parameters) ? 208U :
                E108ProfileOnlyNPQRSIJK(b, 210U, tuning, parameters) ? 210U :
                E108ProfileOnlyNPQRSIJK(b, 212U, tuning, parameters) ? 212U :
                E108ProfileOnlyNPQRSIJK(b, 214U, tuning, parameters) ? 214U :
                E108ProfileOnlyNPQRSIJK(b, 216U, tuning, parameters) ? 216U :
                E108ProfileOnlyNPQRSIJK(b, 218U, tuning, parameters) ? 218U :
                E108ProfileOnlyNPQRSIJK(b, 220U, tuning, parameters) ? 220U :
                E108ProfileOnlyNPQRSIJK(b, 222U, tuning, parameters) ? 222U :
                E108ProfileOnlyNPQRSIJK(b, 224U, tuning, parameters) ? 224U :
                E108ProfileOnlyNPQRSIJK(b, 226U, tuning, parameters) ? 226U :
                E108ProfileOnlyNPQRSIJK(b, 228U, tuning, parameters) ? 228U :
                E108ProfileOnlyNPQRSIJK(b, 230U, tuning, parameters) ? 230U :
                E108ProfileOnlyNPQRSIJK(b, 232U, tuning, parameters) ? 232U :
                E108ProfileOnlyNPQRSIJK(b, 234U, tuning, parameters) ? 234U :
                E108ProfileOnlyNPQRSIJK(b, 236U, tuning, parameters) ? 236U :
                E108ProfileOnlyNPQRSIJK(b, 238U, tuning, parameters) ? 238U :
                E108ProfileOnlyNPQRSIJK(b, 240U, tuning, parameters) ? 240U :
                E108ProfileOnlyNPQRSIJK(b, 242U, tuning, parameters) ? 242U :
                E108ProfileOnlyNPQRSIJK(b, 244U, tuning, parameters) ? 244U :
                E108ProfileOnlyNPQRSIJK(b, 246U, tuning, parameters) ? 246U :
                E108ProfileOnlyNPQRSIJK(b, 248U, tuning, parameters) ? 248U :
                E108ProfileOnlyNPQRSIJK(b, 250U, tuning, parameters) ? 250U :
                E108ProfileOnlyNPQRSIJK(b, 252U, tuning, parameters) ? 252U :
                E108ProfileOnlyNPQRSIJK(b, 254U, tuning, parameters) ? 254U :
                E108ProfileOnlyNPQRSIJK(b, 256U, tuning, parameters) ? 256U :
                E108ProfileOnlyNPQRSIJK(b, 258U, tuning, parameters) ? 258U :
                E108ProfileOnlyNPQRSIJK(b, 260U, tuning, parameters) ? 260U :
                E108ProfileOnlyNPQRSIJK(b, 262U, tuning, parameters) ? 262U :
                E108ProfileOnlyNPQRSIJK(b, 264U, tuning, parameters) ? 264U :
                E108ProfileOnlyNPQRSIJK(b, 266U, tuning, parameters) ? 266U :
                E108ProfileOnlyNPQRSIJK(b, 268U, tuning, parameters) ? 268U :
                E108ProfileOnlyNPQRSIJK(b, 270U, tuning, parameters) ? 270U :
                E108ProfileOnlyNPQRSIJK(b, 272U, tuning, parameters) ? 272U :
                E108ProfileOnlyNPQRSIJK(b, 274U, tuning, parameters) ? 274U :
                E108ProfileOnlyNPQRSIJK(b, 276U, tuning, parameters) ? 276U :
                E108ProfileOnlyNPQRSIJK(b, 278U, tuning, parameters) ? 278U :
                E108ProfileOnlyNPQRSIJK(b, 280U, tuning, parameters) ? 280U :
                E108ProfileOnlyNPQRSIJK(b, 282U, tuning, parameters) ? 282U :
                E108ProfileOnlyNPQRSIJK(b, 284U, tuning, parameters) ? 284U :
                E108ProfileOnlyNPQRSIJK(b, 286U, tuning, parameters) ? 286U :
                E108ProfileOnlyNPQRSIJK(b, 288U, tuning, parameters) ? 288U :
                E108ProfileOnlyNPQRSIJK(b, 290U, tuning, parameters) ? 290U : 0U;
            if (historyModeArm)
            {
                if (static_cast<int>(i) != armPC || tuning.positiveGain != 8.0 ||
                    tuning.negativeGain != 1.0 || tuning.deadbandV != 1.0) return false;
                if (E137FlushStageArm(historyModeArm))
                {
                    // Legacy raw catalog values become mm/% only after an
                    // explicit actual entry in this authored program, after
                    // its own table/E selection. Persistent presets alone
                    // cannot become a newly interpreted movement distance.
                    const auto* catalog = m_recipe.GetCatalog();
                    if (!catalog) return false;
                    const unsigned ids[] = {11U, 13U, 19U, 20U};
                    const char* semantics[] = {"FinalApproachSpeed", "DecelerationDistance", "LargeAreaMachiningSpeed", "LargeAreaFlushDistance"};
                    const char* units[] = {"%", "mm", "%", "mm"};
                    unsigned required = 0U, entered = 0U;
                    for (const auto& field : catalog->fields)
                        for (unsigned slot = 0U; slot < 4U; ++slot)
                            if (field.id == ids[slot] && EDM137Units::LegacyExplicitDefinition(field, semantics[slot], units[slot])) required |= 1U << slot;
                    bool table = false, row = false;
                    for (int prefix = 0; required && prefix < armPC; ++prefix)
                    {
                        const auto* item = cache.TryGetLine(prefix);
                        if (!item) return false;
                        const auto& p = item->parsedBlock;
                        if (p.gCount == 1 && EDM77::Detail::EqualLiteral(p.gExpressions[0], 38U))
                        { table = true; row = false; entered = 0U; }
                        else if (p.gCount == 0 && p.has('E'))
                        { row = table; entered = 0U; }
                        else if (p.gCount == 1 && EDM77::Detail::EqualLiteral(p.gExpressions[0], 39U))
                            for (unsigned slot = 0U; slot < 4U; ++slot)
                                if (EDM77::Detail::EqualLiteral(p.expression('P'), ids[slot]))
                                {
                                    entered &= ~(1U << slot);
                                    if (row && EDM77::Detail::EqualLiteral(p.expression('Q'), 3U)) entered |= 1U << slot;
                                }
                    }
                    if ((entered & required) != required) return false;
                }
                m_edm94.gapHistoryServo = m_edm94.gapHistoryMode = true;
                m_edm94.gapHistoryArm = historyModeArm;
                m_edm94.gapWorkTime = E130TimedArm(historyModeArm);
                m_edm94.gapPhysicalAd = (m_edm94.gapWorkTime && E149PhysicalExpected(historyModeArm)) || historyModeArm == 170U || historyModeArm == 172U;
                m_edm94.gapSegmentProfile = m_edm94.gapGainProfile = m_edm94.gapPhysicalCurve = true;
                m_edm94.gapModeFixture = E149ModalFixture(E149HistoryFixture(historyModeArm));
                m_edm94.gapTuning = tuning; m_edm94.gapFlushParameters = parameters;
                phase = 1U; continue;
            }
            const unsigned historyArm = E126Profile(b, 158U, tuning) ? 158U : E126Profile(b, 160U, tuning) ? 160U : 0U;
            if (historyArm)
            {
                if (static_cast<int>(i) != armPC) return false;
                m_edm94.gapHistoryServo = true; m_edm94.gapHistoryArm = historyArm;
                m_edm94.gapSegmentProfile = m_edm94.gapGainProfile = m_edm94.gapPhysicalCurve = true;
                m_edm94.gapModeFixture = historyArm == 158U ? EDM116::Fixture::ArcPair124 : EDM116::Fixture::ArcPairComplement124;
                m_edm94.gapTuning = tuning; phase = 1U; continue;
            }
            const unsigned parameterizedInterrupt = E108ProfileOnlyNPQRSIJK(b, 111U, tuning, parameters) ? 111U :
                E108ProfileOnlyNPQRSIJK(b, 113U, tuning, parameters) ? 113U : 0U;
            const unsigned interruptionArm = E108ProfileOnlyNPQRSIJK(b, 134U, tuning, parameters) ? 134U :
                E108ProfileOnlyNPQRSIJK(b, 136U, tuning, parameters) ? 136U :
                E108ProfileOnlyNPQRSIJK(b, 138U, tuning, parameters) ? 138U :
                E108ProfileOnlyNPQRSIJK(b, 140U, tuning, parameters) ? 140U :
                E108ProfileOnlyNPQRSIJK(b, EDM122::B0ArmP, tuning, parameters) ? EDM122::B0ArmP :
                E108ProfileOnlyNPQRSIJK(b, EDM122::B1ArmP, tuning, parameters) ? EDM122::B1ArmP :
                E108ProfileOnlyNPQRSIJK(b, EDM123::B0ArmP, tuning, parameters) ? EDM123::B0ArmP :
                E108ProfileOnlyNPQRSIJK(b, EDM123::B1ArmP, tuning, parameters) ? EDM123::B1ArmP :
                E108ProfileOnlyNPQRSIJK(b, EDM125::RetreatArmP, tuning, parameters) ? EDM125::RetreatArmP :
                E108ProfileOnlyNPQRSIJK(b, EDM125::ReturnArmP, tuning, parameters) ? EDM125::ReturnArmP : 0U;
            const bool modalArm = E108ProfileOnlyNPQRSIJK(b, 162U, tuning, parameters);
            const bool modalComplementArm = E108ProfileOnlyNPQRSIJK(b, 164U, tuning, parameters);
            const bool arcPairArm = modalArm || E108ProfileOnlyNPQRSIJK(b, 150U, tuning, parameters);
            const bool arcPairComplementArm = modalComplementArm || E108ProfileOnlyNPQRSIJK(b, 152U, tuning, parameters);
            const bool complementArm = interruptionArm == 134U || E108ProfileOnlyNPQRSIJK(b, 132U, tuning, parameters);
            const bool modeFlushArm = interruptionArm != 0U || arcPairArm || arcPairComplementArm ||
                complementArm || E108ProfileOnlyNPQRSIJK(b, 129U, tuning, parameters);
            const bool curveB2Arm = E108ProfileOnlyNPQRSIJK(b, 127U, tuning, parameters);
            const bool physicalCurveArm = modeFlushArm || curveB2Arm || E108ProfileOnlyNPQRSIJK(b, 125U, tuning, parameters);
            const bool curveRuntimeArm = E108ProfileOnlyNPQRSIJK(b, 123U, tuning, parameters);
            const bool curveGeometryArm = curveRuntimeArm || E108ProfileOnlyNPQRSIJK(b, 121U, tuning, parameters);
            const bool b2RepeatArm = curveGeometryArm || E108ProfileOnlyNPQRSIJK(b, 119U, tuning, parameters);
            const bool b2CrossSegmentArm = b2RepeatArm || E108ProfileOnlyNPQRSIJK(b, 117U, tuning, parameters);
            const bool commonPathArm = b2CrossSegmentArm || E108ProfileOnlyNPQRSIJK(b, 115U, tuning, parameters);
            const bool parameterizedArm = physicalCurveArm || commonPathArm || parameterizedInterrupt != 0U || E107ProfileOnlyNPQRSIJK(b, tuning, parameters);
            const unsigned interruptArm = parameterizedInterrupt ? parameterizedInterrupt :
                E106ProfileOnlyNPQRS(b, 105U, tuning) ? 105U : E106ProfileOnlyNPQRS(b, 107U, tuning) ? 107U : 0U;
            const bool flushArm = parameterizedArm || interruptArm != 0U || E105ProfileOnlyNPQRS(b, tuning);
            const bool pairedArm = E104ProfileOnlyNPQRS(b, tuning);
            const bool enduranceArm = E102ProfileOnlyNPQRS(b, tuning);
            const bool continuousArm = E100ProfileOnlyNPQRS(b, tuning);
            const bool burstArm = E99ProfileOnlyNPQRS(b, tuning);
            const bool noiseArm = E98ProfileOnlyNPQRS(b, tuning);
            const bool tuningArm = flushArm || pairedArm || enduranceArm || continuousArm || burstArm || noiseArm || E97ProfileOnlyNPQRS(b, tuning);
            if (E94Profile(b, 85U) || E94Profile(b, 87U) || E94Profile(b, 89U) || tuningArm)
            {
                if (static_cast<int>(i) != armPC) return false;
                m_edm94.gapBoundaryProfile = E94Profile(b, 87U);
                m_edm94.gapSegmentProfile = E94Profile(b, 89U) || tuningArm;
                m_edm94.gapGainProfile = tuningArm;
                m_edm94.gapNoiseProfile = noiseArm;
                m_edm94.gapBurstProfile = burstArm || continuousArm || enduranceArm || pairedArm || flushArm;
                m_edm94.gapContinuousProfile = continuousArm || enduranceArm || pairedArm || flushArm;
                m_edm94.gapEnduranceProfile = enduranceArm || pairedArm || flushArm;
                m_edm94.gapPairedProfile = pairedArm || flushArm;
                m_edm94.gapFlushProfile = flushArm;
                m_edm94.gapFlushParameterized = parameterizedArm;
                m_edm94.gapCommonPath = commonPathArm;
                m_edm94.gapB2CrossSegment = b2CrossSegmentArm;
                m_edm94.gapB2Repeat = b2RepeatArm;
                m_edm94.gapCurveGeometry = curveGeometryArm;
                m_edm94.gapCurveRuntime = curveRuntimeArm;
                m_edm94.gapPhysicalCurve = physicalCurveArm;
                m_edm94.gapCurveB2 = curveB2Arm;
                m_edm94.gapModeFlush = modeFlushArm;
                m_edm94.gapModeInterruption = interruptionArm != 0U;
                m_edm94.gapModeInterruptionArm = interruptionArm;
                m_edm94.gapModeFixture = modalArm ? EDM116::Fixture::Modal127 : modalComplementArm ? EDM116::Fixture::ModalComplement127 : EDM125::ValidProfile(EDM125::ProfileForArm(interruptionArm)) ?
                    EDM125::TargetFixture(EDM125::ProfileForArm(interruptionArm)) : arcPairArm ? EDM116::Fixture::ArcPair124 :
                    arcPairComplementArm ? EDM116::Fixture::ArcPairComplement124 :
                    complementArm ? EDM116::Fixture::Complement117 : EDM116::Fixture::Canonical116;
                m_edm94.gapFlushParameters = parameters;
                m_edm94.gapFlushInterruptArm = interruptArm;
                if (tuningArm) m_edm94.gapTuning = tuning;
                phase = 1U;
            }
            else if (!E94Preamble(b) && !IsEDM130RecipePreambleSameThread(b, static_cast<int>(i)) &&
                !IsEDM149ConfirmationBlockSameThread(b, static_cast<int>(i))) return false;
        }
        else if (phase == 1U)
        {
            if (E94Profile(b, m_edm94.gapHistoryServo ? m_edm94.gapHistoryArm + 1U : m_edm94.gapFlushProfile ? (m_edm94.gapFlushInterruptArm ? m_edm94.gapFlushInterruptArm + 1U : m_edm94.gapModeFlush ? (m_edm94.gapModeInterruption ? m_edm94.gapModeInterruptionArm + 1U : E116FinishP(m_edm94.gapModeFixture)) : m_edm94.gapCurveB2 ? 128U : m_edm94.gapPhysicalCurve ? 126U : m_edm94.gapCurveRuntime ? 124U : m_edm94.gapCurveGeometry ? 122U : m_edm94.gapB2Repeat ? 120U : m_edm94.gapB2CrossSegment ? 118U : m_edm94.gapCommonPath ? 116U : m_edm94.gapFlushParameterized ? 110U : 104U) : m_edm94.gapPairedProfile ? 102U : m_edm94.gapEnduranceProfile ? 100U : m_edm94.gapContinuousProfile ? 98U : m_edm94.gapBurstProfile ? 96U : m_edm94.gapNoiseProfile ? 94U : m_edm94.gapGainProfile ? 92U : m_edm94.gapSegmentProfile ? 90U :
                m_edm94.gapBoundaryProfile ? 88U : 86U))
            {
                if (m_edm94.moves == 0U || modePending) return false;
                m_edm94.pc[m_edm94.moves] = static_cast<int>(i); phase = 2U;
                continue;
            }
            const unsigned step = m_edm94.moves;
            NCParsedBlock edm141Literal{};
            const NCParsedBlock* admitted = &b;
            if (E141MotionArm(m_edm94.gapHistoryArm) && EDM141::Shape(b))
            {
                if (step >= EDM141::RowCapacity) return false;
                NCBlock resolved{}; NCExpressionResolveError error{};
                if (!NCExpressionResolver::ResolveBlock(b, edm141Evaluator, resolved, error) ||
                    !EDM141::Capture(b, resolved, edm141Locals, edm141Defined, m_edm140.count,
                        static_cast<int>(i), m_edm141.rows[step]) ||
                    !EDM141::MakeLiteral(b, m_edm141.rows[step], edm141Literal)) return false;
                ++m_edm141.plannedRows;
                if (m_edm141.rows[step].readMask) ++m_edm141.expressionRows;
                admitted = &edm141Literal;
            }
            NCParsedBlock modalGeometry{};
            const NCParsedBlock* geometry = admitted;
            if (EDM116::IsModalFixture(m_edm94.gapModeFixture))
            {
                EDM116::Mode authoredMode{};
                if (E127Standalone(b, authoredMode))
                {
                    if (EDM149::IsVisibleFlushTestArm(m_edm94.gapHistoryArm) ||
                        EDM149::IsXYZVisibleFlushTestArm(m_edm94.gapHistoryArm) ||
                        EDM149::IsSixVisibleFlushTestArm(m_edm94.gapHistoryArm)) return false;
                    if (step >= EDM114::RequiredRows || modalRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
                    modal.mode = authoredMode; modalOriginPC = static_cast<int>(i); ++modalRevision;
                    continue;
                }
                if (b.has('B'))
                {
                    double value = 0.0;
                    if (!E94Number(b.expression('B'), value) || !EDM116::ParseMode(value, authoredMode) ||
                        modalRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
                    modal.mode = authoredMode; modalOriginPC = static_cast<int>(i); ++modalRevision;
                    modalGeometry = *admitted; modalGeometry.hasParam['B'-'A'] = false;
                    modalGeometry.paramExpressions['B'-'A'].clear(); geometry = &modalGeometry;
                }
                // EDM138 retains the same bounded sixteen-row geometry but lets
                // its authored B0/B1/B2 choices exercise line/arc history seams.
                // Freeze each row's mode and origin below; older fixtures keep
                // their exact historical B sequence.
                if (step >= EDM114::RequiredRows || !EDM116::ValidMode(modal.mode) ||
                    (!E138CrossSegmentArm(m_edm94.gapHistoryArm) && modal.mode != EDM116::ExpectedMode(step))) return false;
                if (EDM149::IsVisibleFlushTestArm(m_edm94.gapHistoryArm) &&
                    (step != 0U || !b.has('B') || modal.mode != EDM116::Mode::B1)) return false;
                if ((EDM149::IsXYZVisibleFlushTestArm(m_edm94.gapHistoryArm) || EDM149::IsSixVisibleFlushTestArm(m_edm94.gapHistoryArm)) &&
                    (step >= 3U || b.has('B') != (step == 0U) || modal.mode != EDM116::Mode::B1)) return false;
                m_edm127.expectedMode[step] = modal.mode; m_edm127.expectedOriginPC[step] = modalOriginPC;
                m_edm127.expectedOriginRevision[step] = modalRevision;
                m_edm127.authoredInline[step] = b.has('B');
                m_edm94.selectorPC[step] = static_cast<int>(i);
            }
            if (m_edm94.gapModeFlush && !EDM116::IsModalFixture(m_edm94.gapModeFixture) && !modePending)
            {
                EDM116::Mode mode{};
                if (step >= EDM114::RequiredRows || !E116Selector(b, mode) || mode != EDM116::ExpectedMode(step)) return false;
                m_edm94.selectorPC[step] = static_cast<int>(i); modePending = true;
                continue;
            }
            if (step >= ((m_edm94.gapHistoryServo || m_edm94.gapEnduranceProfile) ? EDM102::MaximumSteps : 8U) ||
                !(m_edm94.gapPhysicalCurve ? E114Move(*geometry, step, m_edm94.increment[step], m_edm94.feed[step], m_edm94.gapModeFixture,
                    E148AuthoredLineArm(m_edm94.gapHistoryArm)) :
                    E94Move(b, m_edm94.increment[step], m_edm94.feed[step]))) return false;
            if (m_edm94.gapPhysicalCurve)
            {
                double code = 0.0;
                if (!E94Number(geometry->gExpressions[0], code) ||
                    (code != 1.0 && code != 2.0 && code != 3.0)) return false;
                m_edm94.authoredGCode[step] = static_cast<std::uint8_t>(code);
            }
            if (m_edm94.gapBoundaryProfile)
            {
                // The simulated GAP chooses a boundary direction for one
                // small authored segment. It does not synthesize geometry.
                if (step >= 4U || m_edm94.feed[step] != 0.12) return false;
                for (unsigned a = 0U; a < 6U; ++a)
                {
                    const double value = m_edm94.increment[step][a];
                    if (std::fabs(value) > (a < 3U ? 0.002 : 0.04) ||
                        (step != 0U && value != ((step & 1U) ?
                            -m_edm94.increment[0U][a] : m_edm94.increment[0U][a]))) return false;
                }
            }
            if (m_edm94.gapSegmentProfile && !m_edm94.gapPhysicalCurve)
            {
                if (step >= (m_edm94.gapEnduranceProfile ? EDM102::MaximumSteps : m_edm94.gapContinuousProfile ? EDM100::MaximumSteps : 4U) ||
                    m_edm94.feed[step] != 0.12) return false;
                for (unsigned a = 0U; a < 6U; ++a)
                {
                    const double value = m_edm94.increment[step][a];
                    const double magnitude = std::fabs(value);
                    if (magnitude < (a < 3U ? 0.001 : 0.02) ||
                        magnitude > (a < 3U ? 0.002 : 0.04)) return false;
                    if (m_edm94.gapPairedProfile)
                    {
                        if ((step & 1U) && value != -m_edm94.increment[step - 1U][a]) return false;
                    }
                    else if (step != 0U && value != ((step & 1U) ?
                        -m_edm94.increment[0U][a] : m_edm94.increment[0U][a])) return false;
                }
            }
            double authoredArcLengthMm = 0.0;
            if (m_edm94.gapPhysicalCurve && EDM116::IsArcStep(step, m_edm94.gapModeFixture, m_edm94.authoredGCode[step]))
            {
                if (EDM116::IsBoundedArcFixture(m_edm94.gapModeFixture))
                {
                    if (!E152ArcAdmission(*geometry, step, m_edm94.increment[step], m_edm94.feed[step],
                        total, authoredArcLengthMm, m_edm94.gapModeFixture)) return false;
                }
                else
                {
                    // Include both cardinal extrema of the fixed small circle,
                    // not merely its endpoint chord, in the arm-relative budget.
                    const double centerX = total[0] + ((step & 1U) ? -0.001 : 0.001);
                    if (std::fabs(centerX) + 0.001 > 0.025 || std::fabs(total[1]) + 0.001 > 0.025) return false;
                }
            }
            for (unsigned a = 0U; a < 6U; ++a)
            {
                total[a] += m_edm94.increment[step][a];
                if (!std::isfinite(total[a]) || std::fabs(total[a]) > (a < 3U ? .025 : .5)) return false;
            }
            const auto& d = m_edm94.increment[step];
            const double lengthMm = m_edm94.gapPhysicalCurve && EDM116::IsArcStep(step, m_edm94.gapModeFixture, m_edm94.authoredGCode[step]) ?
                (EDM116::IsBoundedArcFixture(m_edm94.gapModeFixture) ? authoredArcLengthMm : 0.001 * EDM114::Pi) :
                std::hypot(std::hypot(d[0], d[1]), d[2]);
            nominalSeconds += lengthMm * 60.0 / m_edm94.feed[step] + 2.0;
            // Admission allowance only; RT segment/session deadlines are unchanged.
            if (m_edm94.gapFlushProfile && !m_edm94.gapModeFlush && (step & 1U) == 0U) nominalSeconds += 2.5;
            if (!std::isfinite(nominalSeconds) || nominalSeconds > 90.0) return false;
            m_edm94.pc[step] = static_cast<int>(i); ++m_edm94.moves; modePending = false;
        }
        else if (phase == 2U)
        {
            if (!EDM77::Detail::Terminal(b)) return false;
            // This bracket requires an actual M30, not M2/EOF.
            if (!EDM77::Detail::EqualLiteral(b.mExpressions[0], 30U)) return false;
            m_edm94.pc[m_edm94.moves + 1U] = static_cast<int>(i); phase = 3U;
        }
        else return false;
    }
    if (phase != 3U || (E140AssignmentArm(m_edm94.gapHistoryArm) && !m_edm140.count)) return false;
    if (EDM149::IsVisibleFlushTestArm(m_edm94.gapHistoryArm) && !E158VisiblePlan(m_edm94)) return false;
    if (EDM149::IsXYZVisibleFlushTestArm(m_edm94.gapHistoryArm) && !E159VisiblePlan(m_edm94)) return false;
    if (E141MotionArm(m_edm94.gapHistoryArm) &&
        (m_edm141.plannedRows != EDM141::RowCapacity || !m_edm141.expressionRows)) return false;
    if (m_edm94.gapModeFlush && nominalSeconds + 6.0 * 3.0 + 2.0 * 20.0 > 115.0) return false;
    // P127 reserves two independent 20-second curve B2 budgets. Its explicit
    // 115-second admission ceiling leaves the unchanged 120-second RT guard intact.
    if (m_edm94.gapCurveB2 && nominalSeconds + 40.0 > 115.0) return false;
    // Each explicit B2 event reserves 20 seconds inside the unchanged 120-second RT session.
    if (m_edm94.gapB2CrossSegment &&
        nominalSeconds + (m_edm94.gapB2Repeat ? 40.0 : 20.0) > 110.0) return false;
    if (m_edm94.gapBoundaryProfile && m_edm94.moves != 2U && m_edm94.moves != 4U) return false;
    if (m_edm94.gapHistoryServo)
    {
        if (EDM149::IsBoundedProgramTestArm(m_edm94.gapHistoryArm))
        { if (!m_edm94.moves || m_edm94.moves > EDM114::RequiredRows || !E156NominalMovementMask(m_edm94)) return false; }
        else if (m_edm94.moves != EDM114::RequiredRows) return false;
    }
    else if (m_edm94.gapFlushProfile)
    { if (m_edm94.moves != EDM105::MaximumSteps) return false; }
    else if (m_edm94.gapPairedProfile)
    { if (m_edm94.moves != EDM104::MaximumSteps) return false; }
    else if (m_edm94.gapEnduranceProfile)
    { if (m_edm94.moves != EDM102::MaximumSteps) return false; }
    else if (m_edm94.gapSegmentProfile && m_edm94.moves != 2U && m_edm94.moves != 4U &&
        (!m_edm94.gapContinuousProfile || m_edm94.moves != EDM100::MaximumSteps)) return false;
    if (!EDM149::IsBoundedProgramTestArm(m_edm94.gapHistoryArm))
        for (double offset : total) if (std::fabs(offset) > 1.0e-12) return false;
    return true;
}

// EDM103_20261014: bounded process-lifetime NC observations, never authority.
// Only exact existing event sites call this recorder; replay cannot count events.
void NCManager::ObserveEDM103LifecycleSameThread(const char* event, const char* reason) const noexcept
{
    const auto& s = m_edm94;
    if (!s.gapEnduranceProfile || !s.session || !event) return;
    unsigned index = 8U;
    if (std::strcmp(event, "ARM_REQUEST") == 0) index = 0U;
    else if (std::strcmp(event, "CANCEL_REQUEST") == 0 && reason &&
        std::strcmp(reason, "MANUAL_HOLD") == 0) index = 1U;
    else if (std::strcmp(event, "START_BLOCKED") == 0 && reason)
    {
        if (std::strcmp(reason, "RESET_REQUIRED") == 0) index = 2U;
        else if (std::strcmp(reason, "RESET_RELEASE_PENDING") == 0) index = 3U;
    }
    else if (std::strcmp(event, "NORMAL_RELEASED") == 0) index = 4U;
    else if (std::strcmp(event, "RELEASED") == 0) index = 5U;
    if (index == 8U) return;
    auto& audit = m_edm103Audit;
    auto& r = audit.record[index];
    // Lifecycle events count once per session; START rejections once per core
    // received edge. A periodic diagnostic cannot manufacture a button press.
    if (r.count && r.session == s.session &&
        ((index != 2U && index != 3U) || r.rx == m_startDiagnosticRequests)) return;
    const auto maximum = (std::numeric_limits<std::uint64_t>::max)();
    if (r.count != maximum) ++r.count;
    if (audit.sequence != maximum) ++audit.sequence;
    r.sequence = audit.sequence; r.rx = m_startDiagnosticRequests;
    r.session = s.session; r.run = s.run; r.cache = s.cache;
    r.tick = s.feedback.lastTick; r.epoch = s.armEpoch;
    r.ownerGeneration = s.lease.generation;
    r.state = static_cast<unsigned>(static_cast<NCState>(m_state));
    // Snapshotted NC reset flags only; physical release remains in EDM94 proof.
    r.flags = (s.resetSeen ? 1U : 0U) | (s.resetCommitted ? 2U : 0U) |
        (s.releaseRequested ? 4U : 0U);
}

void NCManager::ObserveEDM103RunSameThread(bool load, unsigned originState) noexcept
{
    // These two hooks run only after existing LOAD / fresh START commits.
    // Scope is every NC program, not a claim that a new EDM arm was accepted.
    auto& audit = m_edm103Audit;
    auto& r = audit.record[load ? 6U : 7U];
    const auto maximum = (std::numeric_limits<std::uint64_t>::max)();
    if (r.count != maximum) ++r.count;
    if (audit.sequence != maximum) ++audit.sequence;
    const auto owner = m_motion.GetMotionOwnerLease();
    r.sequence = audit.sequence; r.rx = m_startDiagnosticRequests;
    r.session = m_edm94.session; r.run = m_pathCoreLiveBookkeeping.currentRunToken;
    r.cache = GetBaseProgramCache().GetGeneration(); r.tick = 0ULL;
    r.epoch = m_motion.GetCurrentExecutionEpoch(); r.ownerGeneration = owner.generation;
    r.state = static_cast<unsigned>(static_cast<NCState>(m_state));
    r.flags = originState;
}

void NCManager::LogEDM103AcceptanceSameThread() const noexcept
{
    const auto& audit = m_edm103Audit;
    if (!audit.record[0U].count) return;
    static const char* const parts[8U] = { "ARM_REQUEST", "MANUAL_HOLD", "BLOCKED_BEFORE_RESET",
        "BLOCKED_RELEASE_PENDING", "NORMAL_RELEASE", "RESET_RELEASE", "LOAD", "START" };
    char record[481]{};
    for (unsigned i = 0U; i < 8U; ++i)
    {
        const auto& r = audit.record[i];
        if (!r.count) continue;
        const bool saturated = r.count == (std::numeric_limits<std::uint64_t>::max)() ||
            audit.sequence == (std::numeric_limits<std::uint64_t>::max)();
        int length = 0;
        if (i < 6U)
            length = std::snprintf(record, sizeof(record),
                "[EDM103] event=ACCEPTANCE_HISTORY part=%s count=%llu seq=%llu rx=%llu session=%llu run=%llu cache=%llu tick=%llu armEpoch=%u ownerGen=%u nc=%u resetSeen=%u resetCommitted=%u releaseRequested=%u consumedStart=%u saturated=%u scope=SIX_AXIS_ENDURANCE_NC_EVENTS historical=1 authority=NONE\n",
                parts[i], static_cast<unsigned long long>(r.count), static_cast<unsigned long long>(r.sequence),
                static_cast<unsigned long long>(r.rx), static_cast<unsigned long long>(r.session),
                static_cast<unsigned long long>(r.run), static_cast<unsigned long long>(r.cache),
                static_cast<unsigned long long>(r.tick), r.epoch, r.ownerGeneration, r.state,
                (r.flags & 1U) ? 1U : 0U, (r.flags & 2U) ? 1U : 0U, (r.flags & 4U) ? 1U : 0U,
                (i == 2U || i == 3U) ? 1U : 0U, saturated ? 1U : 0U);
        else
            length = std::snprintf(record, sizeof(record),
                "[EDM103] event=ACCEPTANCE_HISTORY part=%s count=%llu seq=%llu rx=%llu priorSession=%llu run=%llu cache=%llu epoch=%u ownerGen=%u origin=%u nc=%u saturated=%u scope=NC_CORE_ALL_PROGRAMS historical=1 armProof=0 authority=NONE\n",
                parts[i], static_cast<unsigned long long>(r.count), static_cast<unsigned long long>(r.sequence),
                static_cast<unsigned long long>(r.rx), static_cast<unsigned long long>(r.session),
                static_cast<unsigned long long>(r.run), static_cast<unsigned long long>(r.cache),
                r.epoch, r.ownerGeneration, r.flags, r.state, saturated ? 1U : 0U);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
            RtPrintf("%s", record);
        else RtPrintf("[EDM103] event=LOG_FORMAT_ERROR part=%s authority=NONE\n", parts[i]);
    }
}

// EDM101_20261013: NC prints existing cached observations; this grants no readiness.
void NCManager::LogEDM101RecoverySameThread(const char* event, const char* reason) const noexcept
{
    const auto& s = m_edm94; const auto& f = s.feedback;
    char record[481]{};
    const char* const result = event ? event : "UNKNOWN";
    const auto session = static_cast<unsigned long long>(s.session);
    const auto tick = static_cast<unsigned long long>(f.lastTick);
    const bool identityMatch = s.session != 0ULL && s.armEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        s.lease.generation != MOTION_OWNER_GENERATION_INVALID && f.identity.session == s.session &&
        f.identity.epoch == s.armEpoch && f.identity.ownerGeneration == s.lease.generation;
    const bool tickCurrent = f.lastTick != 0ULL && f.lastTick >= s.lastTick;
    const bool emptyAdmission = std::strcmp(result, "EMPTY_ADMISSION_RELEASED") == 0;
    const auto emit = [&](int length, const char* part) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
            RtPrintf("%s", record);
        else RtPrintf("[EDM101] event=LOG_FORMAT_ERROR part=%.16s session=%llu tick=%llu output=NONE\n",
            part, session, tick);
    };
    emit(std::snprintf(record, sizeof(record),
        "[EDM101] event=RECOVERY_SNAPSHOT result=%.32s part=NC session=%llu tick=%llu reason=%.64s ncState=%u resetContinuationPhase=%u latched=%u cancelled=%u resetSeen=%u resetCommitted=%u releaseRequested=%u normalEndCommitted=%u completionReleaseRequested=%u everArmed=%u complete=%u\n",
        result, session, tick, reason ? reason : "UNKNOWN",
        static_cast<unsigned>(static_cast<NCState>(m_state)), static_cast<unsigned>(m_resetContinuationPhase),
        s.latched ? 1U : 0U, s.cancelled ? 1U : 0U, s.resetSeen ? 1U : 0U,
        s.resetCommitted ? 1U : 0U, s.releaseRequested ? 1U : 0U,
        s.normalEndCommitted ? 1U : 0U, s.completionReleaseRequested ? 1U : 0U,
        s.everArmed ? 1U : 0U, s.complete ? 1U : 0U), "NC");
    emit(std::snprintf(record, sizeof(record),
        "[EDM101] event=RECOVERY_SNAPSHOT result=%.32s part=RT session=%llu tick=%llu state=%s rtReason=%s forceZero=%u stopped=%u stable=%u required=%u faulted=%u cancelled=%u released=%u active=%u armAccepted=%u completionReleased=%u completionPhase=%u identityMatch=%u tickCurrent=%u cachedTick=%llu\n",
        result, session, tick, EDM94::StateName(f.state), EDM94::ReasonName(f.reason),
        f.forceZero ? 1U : 0U, f.stopped ? 1U : 0U, f.settledCycles, f.requiredCycles,
        f.faulted ? 1U : 0U, f.cancelled ? 1U : 0U, f.released ? 1U : 0U,
        f.active ? 1U : 0U, f.armAccepted ? 1U : 0U, f.completionReleased ? 1U : 0U,
        static_cast<unsigned>(f.completionPhase), identityMatch ? 1U : 0U, tickCurrent ? 1U : 0U,
        static_cast<unsigned long long>(s.lastTick)), "RT");
    emit(std::snprintf(record, sizeof(record),
        "[EDM101] event=RECOVERY_SNAPSHOT result=%.32s part=IDENTITY session=%llu tick=%llu ncArmEpoch=%llu ncOwnerGen=%u rtSession=%llu rtArmEpoch=%llu rtOwnerGen=%u currentSession=%llu currentEpoch=%llu currentOwnerGen=%u completionEpoch=%llu idleGen=%u noneGen=%u\n",
        result, session, tick, static_cast<unsigned long long>(s.armEpoch),
        static_cast<unsigned>(s.lease.generation), static_cast<unsigned long long>(f.identity.session),
        static_cast<unsigned long long>(f.identity.epoch), static_cast<unsigned>(f.identity.ownerGeneration),
        static_cast<unsigned long long>(f.currentIdentity.session), static_cast<unsigned long long>(f.currentIdentity.epoch),
        static_cast<unsigned>(f.currentIdentity.ownerGeneration), static_cast<unsigned long long>(s.completionEpoch),
        static_cast<unsigned>(s.completionIdleLease.generation), static_cast<unsigned>(s.completionNoneGeneration)), "IDENTITY");
    emit(std::snprintf(record, sizeof(record),
        "[EDM101] event=RECOVERY_SNAPSHOT result=%.32s part=COMPLETION session=%llu tick=%llu ncCompletionEpoch=%llu ncIdleGen=%u ncNoneGen=%u rtCompletionEpoch=%llu rtIdleGen=%u rtNoneGen=%u proofScope=CACHED_RECEIPT_IDENTITY_ONLY\n",
        result, session, tick, static_cast<unsigned long long>(s.completionEpoch),
        static_cast<unsigned>(s.completionIdleLease.generation), static_cast<unsigned>(s.completionNoneGeneration),
        static_cast<unsigned long long>(f.completionEpoch), static_cast<unsigned>(f.completionIdleOwnerGeneration),
        static_cast<unsigned>(f.completionNoneOwnerGeneration)), "COMPLETION");
    emit(std::snprintf(record, sizeof(record),
        "[EDM101] event=RECOVERY_SNAPSHOT result=%.32s part=POLICY session=%llu tick=%llu run=%llu cache=%llu armDispatch=%llu step=%u moves=%u hold=CANCEL_RESET_REQUIRED start=START_TOP_AFTER_RESET emptyAdmission=%u noStopProof=%u proofScope=SNAPSHOT_ONLY output=GUARDED_CSV dischargeCommand=NONE\n",
        result, session, tick, static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.cache),
        static_cast<unsigned long long>(s.armDispatch), s.next, s.moves,
        emptyAdmission ? 1U : 0U, emptyAdmission ? 1U : 0U), "POLICY");
}

void NCManager::LogEDM94SameThread(const char* event, const char* reason) const noexcept
{
    ObserveEDM103LifecycleSameThread(event, reason);
    const auto& s = m_edm94; const auto& f = s.feedback;
    PrintEDM145SourceFailure(s.session);
    // EDM101_20261013: retain every bracket field without native 511-byte clipping.
    // Each part includes the original event, session and cached RT tick.
    char record[481]{};
    const char* const result = event ? event : "UNKNOWN";
    const auto session = static_cast<unsigned long long>(s.session);
    const auto tick = static_cast<unsigned long long>(f.lastTick);
    const auto emit = [&](int length, const char* part) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
            RtPrintf("%s", record);
        else RtPrintf("[EDM94] event=LOG_FORMAT_ERROR part=%.16s session=%llu tick=%llu output=NONE\n",
            part, session, tick);
    };
    emit(std::snprintf(record, sizeof(record),
        "[EDM94] event=%.32s reason=%.64s part=STATE session=%llu tick=%llu state=%s rtReason=%s stable=%u required=%u armed=%u zero=%u stopped=%u pass=%u cancelled=%u faulted=%u released=%u\n",
        result, reason ? reason : "UNKNOWN", session, tick,
        EDM94::StateName(f.state), EDM94::ReasonName(f.reason), f.settledCycles, f.requiredCycles,
        f.armedReady ? 1U : 0U, f.forceZero ? 1U : 0U, f.stopped ? 1U : 0U,
        f.pass ? 1U : 0U, f.cancelled ? 1U : 0U, f.faulted ? 1U : 0U, f.released ? 1U : 0U), "STATE");
    emit(std::snprintf(record, sizeof(record),
        "[EDM94] event=BRACKET_EVIDENCE result=%.32s part=IDENTITY session=%llu tick=%llu run=%llu cache=%llu armDispatch=%llu step=%u moves=%u sentTick=%llu\n",
        result, session, tick, static_cast<unsigned long long>(s.run),
        static_cast<unsigned long long>(s.cache), static_cast<unsigned long long>(s.armDispatch),
        s.next, s.moves, static_cast<unsigned long long>(f.lastSentTick)), "IDENTITY");
    emit(std::snprintf(record, sizeof(record),
        "[EDM94] event=BRACKET_EVIDENCE result=%.32s part=OUTPUT session=%llu tick=%llu nonzeroSentMask=%u actualMovedMask=%u commandMovedMask=%u sentFrames=%llu handoffZero=%u handoffStarted=%u handoffCompleted=%u source=FIXED_NC gapControl=%s actualOutput=GUARDED_CSV powerControl=NC_PLC_S92\n",
        result, session, tick, f.nonzeroSentMask, f.actualMovedMask, f.commandMovedMask,
        static_cast<unsigned long long>(f.sentFrames), f.handoffZero ? 1U : 0U,
        f.handoffStarted, f.handoffCompleted, s.gapWorkTime ? (EDM149::IsTestArm(s.gapHistoryArm) ? ((EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) || EDM149::IsSparseArcTestArm(s.gapHistoryArm) || EDM149::IsSparseTestArm(s.gapHistoryArm) || EDM149::IsOrderTestArm(s.gapHistoryArm) || EDM149::IsArcTestArm(s.gapHistoryArm) || EDM149::IsSixTestArm(s.gapHistoryArm)) ? "XYZCUV_VOLTAGE_REAL_MOTION" : EDM149::IsXYZTestArm(s.gapHistoryArm) ? "XYZ_VOLTAGE_REAL_MOTION" : "Z_VOLTAGE_REAL_MOTION") : E137FlushStageArm(s.gapHistoryArm) ? (E139DiagnosticArm(s.gapHistoryArm) ? "SIMULATED_AD_STAGED_FLUSH_TIMED_HISTORY_GAP_SERVO" : "PHYSICAL_AD_STAGED_FLUSH_TIMED_HISTORY_GAP_SERVO") : E136CurveArm(s.gapHistoryArm) ? "PHYSICAL_AD_PROCESS_CURVE_TIMED_HISTORY_GAP_SERVO" : E135ReferenceArm(s.gapHistoryArm) ? "PHYSICAL_AD_E5_E6_E7_E8_E9_E21_E22_TIMED_HISTORY_GAP_SERVO" : E134MachiningArm(s.gapHistoryArm) ? "PHYSICAL_AD_E5_E6_E7_E8_E21_E22_TIMED_HISTORY_GAP_SERVO" : E133DeepArm(s.gapHistoryArm) ? "PHYSICAL_AD_E5_E6_E7_E21_E22_TIMED_HISTORY_GAP_SERVO" : E132HeightArm(s.gapHistoryArm) ? "PHYSICAL_AD_E5_E6_E7_TIMED_HISTORY_GAP_SERVO" : E131SpeedArm(s.gapHistoryArm) ? "PHYSICAL_AD_E5_E7_TIMED_HISTORY_GAP_SERVO" : "PHYSICAL_AD_E5_TIMED_HISTORY_GAP_SERVO") : s.gapPhysicalAd ? "PHYSICAL_AD_HISTORY_GAP_SERVO" : s.gapHistoryServo ? "SIM_HISTORY_GAP_SERVO" : s.gapFlushProfile ? "SIM_PAIRED_FLUSH_BURST_SERVO" :
            s.gapPairedProfile ? "SIM_PAIRED_CAPTURE_DWELL_BURST_SERVO" :
            s.gapEnduranceProfile ? "SIM_ENDURANCE_CAPTURE_DWELL_BURST_SERVO" :
            s.gapContinuousProfile ? "SIM_CONTINUOUS_CAPTURE_DWELL_BURST_SERVO" :
            s.gapBurstProfile ? "SIM_CAPTURE_DWELL_BURST_SERVO" : s.gapNoiseProfile ? "SIM_DEADBAND_NOISE_SERVO" : s.gapGainProfile ? "SIM_GAIN_SERVO" : s.gapSegmentProfile ? "SIM_SEGMENT_SERVO" :
            s.gapBoundaryProfile ? "SIM_BOUNDARY_DIRECTION" : "NOT_CONNECTED"), "OUTPUT");
    if (EDM149::IsTestArm(s.gapHistoryArm) && f.armAccepted && f.identity.session == s.session &&
        (std::strcmp(result, "ARMED") == 0 || std::strcmp(result, "FINISH_REQUEST") == 0 ||
            std::strcmp(result, "PASS") == 0 || std::strcmp(result, "RT_FAULT") == 0))
        emit(std::snprintf(record, sizeof(record),
            "[%s] event=MOVEMENT_PROOF result=%.32s session=%llu requiredMask=%u nonzeroSentMask=%u actualMovedMask=%u commandMovedMask=%u proof=%u safetyAxes=63 zeroStableRequired=%u\n",
            EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) ? "EDM167" : EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) ? "EDM159" : EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) ? "EDM158" : EDM149::IsPauseTestArm(s.gapHistoryArm) ? "EDM157" : EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) ? "EDM156" : EDM149::IsSparseArcTestArm(s.gapHistoryArm) ? "EDM155" : EDM149::IsSparseTestArm(s.gapHistoryArm) ? "EDM154" : EDM149::IsOrderTestArm(s.gapHistoryArm) ? "EDM153" : EDM149::IsArcTestArm(s.gapHistoryArm) ? "EDM152" : EDM149::IsSixTestArm(s.gapHistoryArm) ? "EDM151" : EDM149::IsXYZTestArm(s.gapHistoryArm) ? "EDM150" : "EDM149-FIX1", result, session, f.requiredMovementMask, f.nonzeroSentMask, f.actualMovedMask, f.commandMovedMask,
            EDM94::MovementProven(f, E156RequiredMovementMask(s, m_edm130.recipe)) ? 1U : 0U, f.requiredCycles), (EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) || EDM149::IsSparseArcTestArm(s.gapHistoryArm) || EDM149::IsSparseTestArm(s.gapHistoryArm) || EDM149::IsOrderTestArm(s.gapHistoryArm) || EDM149::IsArcTestArm(s.gapHistoryArm) || EDM149::IsSixTestArm(s.gapHistoryArm)) ? "XYZCUV_MOVEMENT" : "Z_MOVEMENT");
    if (s.gapContinuousProfile &&
        (std::strcmp(result, "CANCEL_REQUEST") == 0 || std::strcmp(result, "CANCEL_STOPPED") == 0 ||
            std::strcmp(result, "START_BLOCKED") == 0 || std::strcmp(result, "RELEASE_REQUEST") == 0 ||
            std::strcmp(result, "RELEASED") == 0 || std::strcmp(result, "EMPTY_ADMISSION_RELEASED") == 0 ||
            std::strcmp(result, "NORMAL_RELEASE_REQUEST") == 0 || std::strcmp(result, "NORMAL_RELEASED") == 0 ||
            std::strcmp(result, "RT_FAULT") == 0 || std::strcmp(result, "REJECT") == 0))
        LogEDM101RecoverySameThread(result, reason);
    if (s.gapModeInterruption &&
        (std::strcmp(result, "ARM_REQUEST") == 0 || std::strcmp(result, "START_BLOCKED") == 0 ||
            std::strcmp(result, "RELEASE_REQUEST") == 0 || std::strcmp(result, "RT_FAULT") == 0 ||
            std::strcmp(result, "REJECT") == 0))
        LogEDM118SameThread(result, reason);
    if (s.gapFlushProfile && !s.gapModeInterruption &&
        (std::strcmp(result, "ARM_REQUEST") == 0 || std::strcmp(result, "CANCEL_REQUEST") == 0 ||
            std::strcmp(result, "CANCEL_STOPPED") == 0 || std::strcmp(result, "START_BLOCKED") == 0 ||
            std::strcmp(result, "RELEASE_REQUEST") == 0 || std::strcmp(result, "RELEASED") == 0 ||
            std::strcmp(result, "EMPTY_ADMISSION_RELEASED") == 0 ||
            std::strcmp(result, "NORMAL_RELEASE_REQUEST") == 0 || std::strcmp(result, "NORMAL_RELEASED") == 0 ||
            std::strcmp(result, "RT_FAULT") == 0 || std::strcmp(result, "REJECT") == 0))
        LogEDM106InterruptionSameThread(result, reason);
    if (s.gapB2CrossSegment &&
        (std::strcmp(result, "CANCEL_REQUEST") == 0 || std::strcmp(result, "CANCEL_STOPPED") == 0 ||
            std::strcmp(result, "RT_FAULT") == 0 || std::strcmp(result, "REJECT") == 0 ||
            std::strcmp(result, "PASS") == 0 || std::strcmp(result, "NORMAL_RELEASED") == 0 ||
            std::strcmp(result, "RELEASED") == 0 || std::strcmp(result, "START_BLOCKED") == 0))
        LogEDM110SameThread(result);
    if (s.gapSegmentProfile)
    {
        EDM96::Feedback gap{};
        if (m_motion.ReadEDM96Feedback(gap) && gap.session == s.session)
            LogEDM96SameThread(event, gap);
    }
    if (s.gapBoundaryProfile)
    {
        EDM95::Feedback gap{};
        if (m_motion.ReadEDM95Feedback(gap) && gap.session == s.session)
            RtPrintf("[EDM95] event=BOUNDARY_EVIDENCE result=%s session=%llu expectedStep=%u accepted=%llu forward=%llu retreat=%llu tick=%llu sourceSequence=%llu freshFrames=%llu heldFrames=%llu maxAgeUs=%llu faulted=%u reason=%s output=GUARDED_CSV midSegmentReverse=0 physicalStopProof=EDM94 powerControl=NC_PLC_S92\n",
                event, static_cast<unsigned long long>(gap.session), gap.expectedStep,
                static_cast<unsigned long long>(gap.acceptedSegments), static_cast<unsigned long long>(gap.forwardSegments),
                static_cast<unsigned long long>(gap.retreatSegments), static_cast<unsigned long long>(gap.tick),
                static_cast<unsigned long long>(gap.sourceSequence), static_cast<unsigned long long>(gap.freshFrames),
                static_cast<unsigned long long>(gap.heldFrames), static_cast<unsigned long long>(gap.maximumAgeUs),
                gap.faulted ? 1U : 0U, EDM95::ReasonName(gap.reason));
    }
    if (s.normalEndCommitted || s.completionReleaseRequested)
    {
        RtPrintf("[EDM94] event=COMPLETION_RELEASE build=FIX16 result=%s session=%llu phase=%u requested=%u completed=%u endEpoch=%llu idleGen=%u noneGen=%u source=M30_ZERO_RETIREMENT\n",
            event, static_cast<unsigned long long>(s.session), static_cast<unsigned>(f.completionPhase),
            s.completionReleaseRequested ? 1U : 0U, f.completionReleased ? 1U : 0U,
            static_cast<unsigned long long>(s.completionEpoch), static_cast<unsigned>(s.completionIdleLease.generation),
            static_cast<unsigned>(s.completionNoneGeneration));
    }
    // The first RT fault remains frozen; FIX5 reports it even when owner loss
    // has already cancelled the NC bracket. All output remains NC-thread only.
    // stop fields describe the latest published RT observation. NC prints them.
    const bool runtime = std::strcmp(event, "REJECT") == 0 || std::strcmp(event, "RT_FAULT") == 0 ||
        std::strcmp(event, "RELEASE_REQUEST") == 0 || std::strcmp(event, "START_BLOCKED") == 0 ||
        std::strcmp(event, "CANCEL_STOPPED") == 0 || std::strcmp(event, "RELEASED") == 0 ||
        std::strcmp(event, "PASS") == 0 || std::strcmp(event, "NORMAL_RELEASE_REQUEST") == 0 ||
        std::strcmp(event, "NORMAL_RELEASED") == 0;
    if (runtime && f.armAccepted)
    {
        const auto& d = f.firstFault;
        if (d.site != 0U)
        {
            if (EDM173::IsArm(s.gapHistoryArm))
                PrintEDM173InputFailure(m_motion,s.session,d.tick);
            RtPrintf("[EDM94] event=RUNTIME_FAULT build=FIX16 result=%s session=%llu site=%s faultReason=%s faultAxis=%c tripAxisMask=%02X observeOnlyAxisMask=%02X faultTick=%llu frameUs=%llu frameSession=%llu frameEpoch=%llu frameOwnerGen=%u flags=%08X evidence=FIRST_FAULT_ONLY\n",
                event, static_cast<unsigned long long>(s.session), EDM94::FaultSiteName(d.site),
                EDM94::ReasonName(static_cast<EDM94::Reason>(d.reason)),
                f.rejectedAxis < 6U ? "XYZCUV"[f.rejectedAxis] : '-',
                static_cast<unsigned>(EDM94::FollowingTripAxisMask), static_cast<unsigned>(EDM94::FollowingObserveOnlyAxisMask),
                static_cast<unsigned long long>(d.tick), static_cast<unsigned long long>(d.nowUs),
                static_cast<unsigned long long>(d.frameIdentity.session), static_cast<unsigned long long>(d.frameIdentity.epoch),
                static_cast<unsigned>(d.frameIdentity.ownerGeneration), static_cast<unsigned>(d.flags));
            RtPrintf("[EDM94] event=RUNTIME_SEND session=%llu proofTick=%llu proofGeneration=%llu proofOwner=%016llX proofExecution=%016llX liveOwner=%016llX liveExecution=%016llX reservedBaseOwner=%016llX reservedBaseExecution=%016llX evidence=FIRST_FAULT_ONLY\n",
                static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(d.proofTick),
                static_cast<unsigned long long>(d.proofGeneration), static_cast<unsigned long long>(d.proofOwnerState),
                static_cast<unsigned long long>(d.proofExecutionPublication), static_cast<unsigned long long>(d.liveOwnerState),
                static_cast<unsigned long long>(d.liveExecutionPublication), static_cast<unsigned long long>(d.reservationOwnerState),
                static_cast<unsigned long long>(d.reservationExecutionPublication));
            RtPrintf("[EDM94] event=RUNTIME_GATES session=%llu authority=%08X sharedHealth=%08X source=%08X group=%08X inputMask=%02X configChangedMask=%02X mappingChangedMask=%02X healthX=%08X healthY=%08X healthZ=%08X healthC=%08X healthU=%08X healthV=%08X evidence=FIRST_FAULT_ONLY\n",
                static_cast<unsigned long long>(s.session), static_cast<unsigned>(d.authorityFailures),
                static_cast<unsigned>(d.sharedHealthFailures), static_cast<unsigned>(d.sourceFailures),
                static_cast<unsigned>(d.groupFailures), static_cast<unsigned>(d.inputMask),
                static_cast<unsigned>(d.configChangedMask), static_cast<unsigned>(d.mappingChangedMask),
                static_cast<unsigned>(d.healthFailures[0]), static_cast<unsigned>(d.healthFailures[1]),
                static_cast<unsigned>(d.healthFailures[2]), static_cast<unsigned>(d.healthFailures[3]),
                static_cast<unsigned>(d.healthFailures[4]), static_cast<unsigned>(d.healthFailures[5]));
            // FIX2: these values are frozen by RT at the FIRST fault. Local
            // PI and the prior NIC-accepted image retain their own timestamps.
            const auto& a = f.firstFaultAxis;
            const auto bits = [](double value) noexcept -> unsigned long long
            { return static_cast<unsigned long long>(EDM130::CurveDoubleBits(value)); };
            RtPrintf("[EDM94] event=FIRST_AXIS build=EDM158_FIX2 session=%llu faultTick=%llu axis=%c flags=%u sign=%d evidence=FIRST_FAULT_ONLY driveAck=0\n",
                static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(d.tick),
                a.axis < 6U ? "XYZCUV"[a.axis] : '-', static_cast<unsigned>(a.flags), static_cast<int>(a.hardwareSign));
            if ((a.flags & EDM94::FaultAxis_FRAME) != 0U)
                RtPrintf("[EDM94] event=FIRST_AXIS_FRAME session=%llu tick=%llu cmdBits=%llu actBits=%llu errorBits=%llu windowBits=%llu ppuBits=%llu\n",
                    static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(d.tick),
                    bits(a.commandPulse), bits(a.actualPulse), bits(a.followingErrorPulse), bits(a.windowPulse), bits(a.pulsePerUnit));
            if ((a.flags & EDM94::FaultAxis_CONTROL) != 0U)
            {
                const auto& c = a.control;
                RtPrintf("[EDM94] event=FIRST_AXIS_GAIN session=%llu controlTick=%llu sourceUs=%llu priorTick=%u kpBits=%llu kiBits=%llu kvffBits=%llu\n",
                    static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(c.tick),
                    static_cast<unsigned long long>(c.nowUs), (a.flags & EDM94::FaultAxis_CONTROL_PRIOR) != 0U ? 1U : 0U,
                    bits(c.kp), bits(c.ki), bits(c.kvff));
                RtPrintf("[EDM94] event=FIRST_AXIS_PI session=%llu controlTick=%llu cmdBits=%llu actBits=%llu refPpsBits=%llu pBits=%llu iBits=%llu\n",
                    static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(c.tick),
                    bits(c.commandPulse), bits(c.actualPulse), bits(c.referenceVelocityPps), bits(c.pTerm), bits(c.iTerm));
                RtPrintf("[EDM94] event=FIRST_AXIS_LOCAL session=%llu controlTick=%llu integralBeforeBits=%llu integralCandidateBits=%llu outputPpsBits=%llu image=LOCAL_PRE_GUARD_PI driveAck=0\n",
                    static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(c.tick),
                    bits(c.integralBefore), bits(c.integralAfter), bits(c.logicalOutputVelocityPps));
            }
            if ((a.flags & EDM94::FaultAxis_ACCEPTED_IMAGE) != 0U)
                RtPrintf("[EDM94] event=FIRST_AXIS_ACCEPTED session=%llu imageTick=%llu sourceUs=%llu epoch=%llu ownerGen=%u csv=%d image=NIC_ACCEPTED driveAck=0\n",
                    static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(a.acceptedTick),
                    static_cast<unsigned long long>(a.acceptedSourceNowUs), static_cast<unsigned long long>(a.acceptedIdentity.epoch),
                    static_cast<unsigned>(a.acceptedIdentity.ownerGeneration), static_cast<int>(a.acceptedCsv));
        }
        const auto& stop = f.stop;
        RtPrintf("[EDM94] event=STOP_CHECK build=FIX16 result=%s session=%llu sampledTick=%llu blocked=%08X firstBlock=%s releaseBlocked=%08X stable=%u required=%u stopped=%u evidence=LATEST_RT_SNAPSHOT\n",
            event, static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(stop.sampledTick),
            static_cast<unsigned>(stop.blockedMask), EDM94::StopBlockName(stop.blockedMask),
            static_cast<unsigned>(stop.releaseBlockedMask), f.settledCycles, f.requiredCycles, f.stopped ? 1U : 0U);
        RtPrintf("[EDM94] event=STOP_GATES session=%llu configChangedMask=%02X mappingChangedMask=%02X unhealthyMask=%02X normalAuthority=%08X sharedHealth=%08X source=%08X group=%08X evidence=LATEST_RT_SNAPSHOT\n",
            static_cast<unsigned long long>(s.session), static_cast<unsigned>(stop.configChangedMask),
            static_cast<unsigned>(stop.mappingChangedMask), static_cast<unsigned>(stop.unhealthyMask),
            static_cast<unsigned>(stop.authorityFailures), static_cast<unsigned>(stop.sharedHealthFailures),
            static_cast<unsigned>(stop.sourceFailures), static_cast<unsigned>(stop.groupFailures));
        char record[640]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM94] event=STOP_RANGE session=%llu rangeExceededMask=%02X rangeResetCount=%u lastRangeResetTick=%llu X=%.12g Y=%.12g Z=%.12g C=%.12g U=%.12g V=%.12g limitPolicy=FROZEN_AXIS_WINDOW unit=ENCODER_PULSE evidence=LAST_RANGE_RESET\n",
            static_cast<unsigned long long>(s.session), static_cast<unsigned>(stop.rangeExceededMask),
            static_cast<unsigned>(stop.rangeResetCount), static_cast<unsigned long long>(stop.lastRangeResetTick),
            stop.rangePulse[0], stop.rangePulse[1], stop.rangePulse[2], stop.rangePulse[3], stop.rangePulse[4], stop.rangePulse[5]);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM94] event=LOG_FORMAT_ERROR scope=STOP_RANGE\n");
        const int limitLength = std::snprintf(record, sizeof(record),
            "[EDM94] event=STOP_LIMITS build=FIX16 session=%llu X=%.12g Y=%.12g Z=%.12g C=%.12g U=%.12g V=%.12g unit=ENCODER_PULSE policy=NC_SETTLE_FROZEN_WINDOW span=WHOLE_DWELL required=%u allAxes=1 evidence=FROZEN_CONFIG\n",
            static_cast<unsigned long long>(s.session),
            EDM94::StopRangeLimitPulse(0U, f.config.axis[0]), EDM94::StopRangeLimitPulse(1U, f.config.axis[1]),
            EDM94::StopRangeLimitPulse(2U, f.config.axis[2]), EDM94::StopRangeLimitPulse(3U, f.config.axis[3]),
            EDM94::StopRangeLimitPulse(4U, f.config.axis[4]), EDM94::StopRangeLimitPulse(5U, f.config.axis[5]),
            EDM94::RequiredStableCycles);
        if (limitLength > 0 && static_cast<std::size_t>(limitLength) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM94] event=LOG_FORMAT_ERROR scope=STOP_LIMITS\n");
    }
    const bool config = std::strcmp(event, "ARMED") == 0;
    const bool terminal = std::strcmp(event, "PASS") == 0 || std::strcmp(event, "CANCEL_STOPPED") == 0 ||
        ((std::strcmp(event, "REJECT") == 0 || std::strcmp(event, "RT_FAULT") == 0) && f.faulted);
    if (!config && !terminal) return;
    // FIX2: this is the frozen attempted-Arm snapshot, not current motion or
    // stop evidence. Print from NC only, once at the admission result.
    const bool admission = config ||
        ((std::strcmp(event, "REJECT") == 0 || std::strcmp(event, "RT_FAULT") == 0) && f.faulted && !f.armAccepted);
    if (admission)
    {
        RtPrintf("[EDM94] event=FOLLOWING_POLICY build=FIX16 session=%llu rotaryRuntimeTrip=%s tripAxisMask=%02X observeOnlyAxisMask=%02X armFollowing=ALL_SIX completionFollowing=ALL_SIX stopProof=FROZEN_WINDOW_SPAN resetRelease=EXPLICIT_RESET_AND_STOP genericLagCheck=UNCHANGED scope=EDM94_BUILD_ONLY endpointIntegral=KI_OUTPUT_CONTINUITY pidGains=CONFIGURED\n",
            static_cast<unsigned long long>(s.session),
            EDM94::IgnoreRotaryFollowingTripForEmptyLoad ? "IGNORED_EMPTY_LOAD" : "ENFORCED",
            static_cast<unsigned>(EDM94::FollowingTripAxisMask),
            static_cast<unsigned>(EDM94::FollowingObserveOnlyAxisMask));
        const auto& d = f.admission;
        RtPrintf("[EDM94] event=ARM_CHECK build=FIX16 result=%s session=%llu sampledTick=%llu sampledUs=%llu gates=%08X firstGate=%s accepted=%u rejectedAxis=%c evidence=ADMISSION_ONLY\n",
            event, static_cast<unsigned long long>(s.session),
            static_cast<unsigned long long>(d.tick), static_cast<unsigned long long>(d.nowUs),
            static_cast<unsigned>(d.failedGates), EDM94::AdmissionGateName(d.failedGates), f.armAccepted ? 1U : 0U,
            f.rejectedAxis < 6U ? "XYZCUV"[f.rejectedAxis] : '-');
        RtPrintf("[EDM94] event=ARM_SOURCE session=%llu sourceFailures=%08X firstSource=%s authorityFailures=%08X firstAuthority=%s sharedHealthFailures=%08X firstShared=%s groupFailures=%08X firstGroup=%s requestFailures=%08X firstRequest=%s inputMask=%08X pendingSource=%d\n",
            static_cast<unsigned long long>(s.session),
            static_cast<unsigned>(d.sourceFailures), EDM94::AdmissionSourceName(d.sourceFailures),
            static_cast<unsigned>(d.authorityFailures), EDM94::AdmissionAuthorityName(d.authorityFailures),
            static_cast<unsigned>(d.sharedHealthFailures), EDM94::AdmissionSharedHealthName(d.sharedHealthFailures),
            static_cast<unsigned>(d.groupFailures), EDM94::AdmissionGroupName(d.groupFailures),
            static_cast<unsigned>(d.requestFailures), EDM94::AdmissionRequestName(d.requestFailures),
            static_cast<unsigned>(d.inputMask), static_cast<int>(d.pendingSource));
        for (unsigned a = 0U; a < 6U; ++a)
        {
            const auto& observed = d.axis[a]; const auto& c = f.config.axis[a];
            char record[640]{};
            const int length = std::snprintf(record, sizeof(record),
                "[EDM94] event=ARM_AXIS_CHECK session=%llu axis=%c healthFailures=%08X firstHealth=%s state=%d statusWord=%04X mode=%d commandVel=%.12g logicalVel=%.12g targetVel=%.12g errorPulse=%.12g windowPulse=%.12g commandAnchor=%.12g actualAnchor=%.12g pulsePerUnit=%.12g hardwareSign=%d evidence=ADMISSION_ONLY\n",
                static_cast<unsigned long long>(s.session), "XYZCUV"[a],
                static_cast<unsigned>(observed.healthFailures), EDM94::AdmissionAxisHealthName(observed.healthFailures),
                static_cast<int>(observed.state), static_cast<unsigned>(observed.statusWord), static_cast<int>(observed.mode),
                observed.commandVelocity, observed.logicalVelocity, observed.targetVelocity, observed.followingError,
                c.followingWindowPulse, c.commandAnchor, c.actualAnchor, c.pulsePerUnit, c.hardwareSign);
            if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM94] event=LOG_FORMAT_ERROR scope=ARM_CHECK axis=%u\n", a);
        }
    }
    RtPrintf("[EDM94] event=TRANSPORT result=%s session=%llu tick=%llu sealed=%llu nicAttempted=%llu nicAccepted=%llu acceptedZero=%llu lastSealed=%u lastAttempted=%u lastAccepted=%u lastAllZero=%u driveAck=0\n",
        event, static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(f.lastTransportTick),
        static_cast<unsigned long long>(f.sealedFrames), static_cast<unsigned long long>(f.nicAttemptedFrames),
        static_cast<unsigned long long>(f.nicAcceptedFrames), static_cast<unsigned long long>(f.acceptedZeroFrames),
        f.lastTransportSealed ? 1U : 0U, f.lastTransportAttempted ? 1U : 0U,
        f.lastTransportAccepted ? 1U : 0U, f.lastTransportAllZero ? 1U : 0U);
    for (unsigned a = 0U; a < 6U; ++a)
    {
        char record[640]{}; const auto& c = f.config.axis[a]; const auto& e = f.axis[a];
        const int length = config ? std::snprintf(record, sizeof(record),
            "[EDM94] event=AXIS_CONFIG session=%llu axis=%c pulsePerUnit=%.12g commandAnchor=%.12g actualAnchor=%.12g followingWindowPulse=%.12g hardwareSign=%d fixtureMaxUnitPerSec=%.12g maxExcursionNative=%.12g configFingerprint=%llu mappingFingerprint=%llu stopRangePulse=%.12g stopRangeNative=%.12g\n",
            static_cast<unsigned long long>(s.session), "XYZCUV"[a], c.pulsePerUnit, c.commandAnchor, c.actualAnchor,
            c.followingWindowPulse, c.hardwareSign, EDM94::NativeVelocityLimit(a), EDM94::NativeExcursionLimit(a),
            static_cast<unsigned long long>(c.configFingerprint), static_cast<unsigned long long>(c.mappingFingerprint),
            EDM94::StopRangeLimitPulse(a, c), c.pulsePerUnit > 0.0 ? EDM94::StopRangeLimitPulse(a, c) / c.pulsePerUnit : 0.0) :
            std::snprintf(record, sizeof(record),
            "[EDM94] event=AXIS_EVIDENCE result=%s session=%llu axis=%c commandMinPulse=%.12g commandMaxPulse=%.12g actualMinPulse=%.12g actualMaxPulse=%.12g currentCommandPulse=%.12g currentActualPulse=%.12g nonzeroSentCount=%llu peakCsv=%u peakPreGuardCsvPps=%.12g capClampCount=%llu stopped=%u\n",
            event, static_cast<unsigned long long>(s.session), "XYZCUV"[a], e.commandMinPulse, e.commandMaxPulse,
            e.actualMinPulse, e.actualMaxPulse, e.currentCommandPulse, e.currentActualPulse,
            static_cast<unsigned long long>(e.nonzeroSentCount), e.peakCsv, e.peakPreGuardCsvPps,
            static_cast<unsigned long long>(e.capClampCount), f.stopped ? 1U : 0U);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM94] event=LOG_FORMAT_ERROR axis=%u\n", a);
    }
}

bool NCManager::IsEDM199G76PauseScopeCurrentSameThread(const EDM157::Scope& scope) noexcept
{
    // A G76 NC row owns 25 RT children. Its retained callback and aggregate
    // receipt are the parent lifetime fence; s.next is not a child ordinal.
    const auto& s=m_edm94;
    if(!EDM157::ValidScope(scope)||!EDM157::IsG76NominalPause(scope)||
        (s.gapHistoryArm!=EDM173::B2ArmP&&s.gapHistoryArm!=EDM173::B3ArmP&&s.gapHistoryArm!=EDM173::B4ArmP)||
        !EDM173::RecipeMatchesArm(s.gapHistoryArm,scope.recipe)||
        static_cast<unsigned>(scope.policy.mode)!=EDM173::ModeForArm(s.gapHistoryArm)||!m_edm173||!m_g76Orbit||
        !s.program||!s.latched||!s.ready||!s.everArmed||!s.gapWorkTime||
        s.armPending||s.cancelled||s.resetSeen||s.resetCommitted||s.releaseRequested||
        s.rtFaultReported||s.finishPending||s.complete||s.gapWaiting||
        s.moves!=1U||s.next!=1U||s.authoredGCode[0U]!=76U||!m_edm130.bound||
        !E156BoundPlanCurrent(s,m_edm130.recipe)||!EDM130::SameRecipe(scope.recipe,m_edm130.recipe)||
        !s.lease.Matches(m_programMotionLease)||!m_motion.IsMotionOwnerLeaseCurrent(s.lease)||
        m_motion.HasPendingSafetyOrRecoveryRequests()||
        m_mode!=NCOperationMode::MEMORY||(m_state!=NCState::RUN&&m_state!=NCState::HOLD)||
        Close_System_Com_flag||!m_externalReadyInterlock||m_edmState==EDMState::NOT_READY||
        AlarmManager::GetInstance().HasAlarm()||m_isSingleBlockEnabled||m_isG66Active||
        !m_macroStack.empty()||Homing.IsActive()||!m_waitCallback||
        s.run!=m_pathCoreLiveBookkeeping.currentRunToken||s.cache!=GetBaseProgramCache().GetGeneration()||
        !CoordSys.IsTranslationRunFrozen()||!CoordSys.IsTranslationRunCurrent()||
        !IsG76OrbitFrameCurrentSameThread(true,true)||!IsXYZCUVFeedProfileCurrentSameThread()||
        !IsFixedTranslationTravelCurrentSameThread()||!IsPathCoreLiveNativeConfigCurrentSameThread()||
        m_motion.GetMotionFeedbackOverflowCount()!=0ULL||m_motion.GetMotionFeedbackProducerNoticeOverflowCount()!=0ULL||
        m_motionFeedbackSequenceGapCount!=0ULL) return false;
    const auto& w=*m_edm173;const auto& g=*m_g76Orbit;const auto& r=g.receipt;
    const auto& parent=w.plan.scope;const auto& source=scope.authorizedSource;
    if(!w.prepared||!w.published||!w.parentCommitted||w.completed||!g.active||!g.bound||g.completed||g.mixed||
        !g.commit||g.mode!=EDM173::ModeForArm(s.gapHistoryArm)||g.sourceG!=76||g.run!=s.run||g.cache!=s.cache||g.pc!=s.pc[0U]||
        g.pc!=GetActiveDispatchPC()||g.dispatch!=m_waitingBlockDispatchId||
        !r.valid||!r.commandAccepted||!r.captureBound||!r.tailCommitted||r.count!=EDM173::CountForRecipe(scope.recipe)||
        !r.ownerLease.Matches(s.lease)||!r.tail.IsCommitted()||!r.tail.captureBound||
        !r.tail.endpointExact||!r.tail.accountingValid||!r.tail.ownerLease.Matches(r.ownerLease)||
        parent.session!=s.session||parent.run!=s.run||parent.cache!=s.cache||parent.dispatch!=g.dispatch||
        parent.sourcePC!=g.pc||parent.line!=g.line||parent.armEpoch!=s.armEpoch||!parent.owner.Matches(s.lease)||
        scope.compoundFingerprint!=w.plan.fingerprint||scope.compoundArmEpoch!=s.armEpoch||
        scope.compoundLine!=g.line||!EDM130::SameRecipe(scope.recipe,w.plan.recipe)||
        source.scope.session!=s.session||source.scope.run!=s.run||source.scope.cache!=s.cache||
        source.scope.ownerGeneration!=s.lease.generation||scope.ownerGeneration!=s.lease.generation||
        source.scope.translationGeneration!=r.translationGeneration||
        source.scope.translationGeneration!=w.plan.nominal.translationGeneration||
        !SameNCTranslationSnapshot(w.plan.nominal.translation,CoordSys.GetTranslationSnapshot())||
        !m_motion.MatchesNCTranslation(w.plan.nominal.translation)||
        source.dispatch!=g.dispatch||source.sourcePC!=g.pc||source.ordinal>=EDM173::CountForRecipe(scope.recipe)||
        source.epoch!=m_motion.GetCurrentExecutionEpoch()||
        (m_edm157.active&&m_edm157.compoundCommit!=g.commit)) return false;
    const unsigned child=static_cast<unsigned>(source.ordinal);
    const auto& id=r.identities[child];
    // Do not require !done here: RT may complete this child after publishing
    // the retained RUNNING acknowledgement and before the next NC scan.
    // HOLD/START publication and RT consumption independently require the
    // exact live child. No parent callback can retire while NC remains HOLD.
    return g.accepted[child]&&g.started[child]&&id.IsAssigned()&&
        id.source==MotionCommandSource::NC_MEMORY&&id.epoch==source.epoch&&
        id.segmentId==source.segment&&id.sourceBlockId==static_cast<MotionSourceBlockId>(g.pc);
}

bool NCManager::IsEDM157ScopeCurrentSameThread(const EDM157::Scope& scope) noexcept
{
    const auto& s = m_edm94;
    if(EDM173::IsArm(s.gapHistoryArm)) return IsEDM199G76PauseScopeCurrentSameThread(scope);
    if (!EDM157::ValidScope(scope) || !EDM157::SupportsArm(s.gapHistoryArm) ||
        !EDM157::SupportsFixture(E149HistoryFixture(s.gapHistoryArm), s.gapModeFixture) ||
        !s.latched || !s.ready || !s.everArmed ||
        s.armPending || s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || !s.gapHistoryServo || !s.gapHistoryMode || !s.gapWorkTime ||
        !s.gapPhysicalCurve || s.gapWaiting || !s.next || s.next > s.moves || !m_edm130.bound ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || scope.historyFixture != E149HistoryFixture(s.gapHistoryArm) ||
        scope.policy.fixture != s.gapModeFixture ||
        scope.recipe.diagnosticControlSource != EDM149::SourceForArm(s.gapHistoryArm) ||
        !EDM130::SameRecipe(scope.recipe, m_edm130.recipe) ||
        scope.ownerGeneration != s.lease.generation || !s.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        scope.authorizedSource.epoch != m_motion.GetCurrentExecutionEpoch() ||
        m_mode != NCOperationMode::MEMORY || (m_state != NCState::RUN && m_state != NCState::HOLD) ||
        Close_System_Com_flag || !m_externalReadyInterlock || m_edmState == EDMState::NOT_READY ||
        AlarmManager::GetInstance().HasAlarm() || m_isSingleBlockEnabled || m_isG66Active ||
        !m_macroStack.empty() || Homing.IsActive() || !m_waitCallback ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        scope.authorizedSource.scope.translationGeneration != CoordSys.GetTranslationSnapshot().generation ||
        !IsXYZCUVFeedProfileCurrentSameThread() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    const unsigned step = s.next - 1U;
    const auto& source = scope.authorizedSource;
    if (source.scope.session != s.session || source.scope.run != s.run || source.scope.cache != s.cache ||
        source.ordinal != step || source.sourcePC != s.pc[step] ||
        source.dispatch != m_waitingBlockDispatchId ||
        !EDM116::SamePolicy(scope.policy, m_edm116.policies[step]) ||
        m_edm116.consumed != s.next || m_edm116.selected != s.next || m_edm116.pending) return false;
    const bool arc = s.authoredGCode[step] == 2U || s.authoredGCode[step] == 3U;
    const auto& identity = arc ? m_pathArcMotion.receipt.identity : m_pathFeedMotion.receipt.identity;
    const auto& lease = arc ? m_pathArcMotion.receipt.ownerLease : m_pathFeedMotion.receipt.ownerLease;
    const bool pending = arc ? (m_pathArc.pending && m_pathArc.bound && m_pathArc.consumerAccepted &&
        m_pathArc.consumerStarted && !m_pathArc.completed && m_pathArc.sourcePC == source.sourcePC &&
        m_pathArc.dispatch == source.dispatch) :
        (m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.consumerAccepted &&
            m_pathFeed.consumerStarted && !m_pathFeed.completed && m_pathFeed.sourcePC == source.sourcePC &&
            m_pathFeed.dispatch == source.dispatch);
    return pending && lease.Matches(s.lease) && identity.IsAssigned() &&
        identity.source == MotionCommandSource::NC_MEMORY && identity.epoch == source.epoch &&
        identity.segmentId == source.segment && identity.sourceBlockId == static_cast<MotionSourceBlockId>(source.sourcePC);
}

bool NCManager::CaptureEDM157ScopeSameThread(EDM157::Scope& scope) noexcept
{
    scope = EDM157::Scope{};
    EDM126::MotionFeedback history{}; EDM128::MotionFeedback mode{};
    const auto& s = m_edm94;
    const bool compound=s.gapHistoryArm==EDM173::B2ArmP||s.gapHistoryArm==EDM173::B3ArmP||s.gapHistoryArm==EDM173::B4ArmP;
    if (!m_motion.ReadEDM126Feedback(history) || !m_motion.ReadEDM130Feedback(mode) ||
        !history.enabled || !history.active || history.faulted || !history.sourceReady ||
        !history.rowActive || history.rowCompleted || history.completedRows + 1U != history.authorizedRows ||
        (compound ? (history.authorizedRows>EDM173::CountForRecipe(m_edm130.recipe)||history.authorizedRows!=history.authorizedSource.ordinal+1ULL) :
            history.authorizedRows!=s.next) || !history.tick || !history.nowUs ||
        history.session != s.session || history.armEpoch != s.armEpoch || history.ownerGeneration != s.lease.generation ||
        !mode.enabled || !mode.active || mode.faulted || !mode.workTimeFlush ||
        mode.fixture != (compound?EDM173::HistoryFixture:E149HistoryFixture(s.gapHistoryArm)) || history.fixture != mode.fixture ||
        mode.session != s.session || mode.armEpoch != s.armEpoch ||
        mode.ownerGeneration != s.lease.generation || mode.liveEpoch != history.liveEpoch ||
        mode.authorizedRows != history.authorizedRows || mode.completedRows != history.completedRows ||
        !EDM130::SameRecipe(mode.timerRecipe, m_edm130.recipe)) return false;
    scope.authorizedSource = history.authorizedSource;
    scope.ownerGeneration = history.ownerGeneration; scope.historyFixture = history.fixture;
    scope.recipe = mode.timerRecipe; scope.policy = mode.rowPolicy;
    if(compound)
    {
        EDM173::Feedback parent{};
        if(!m_edm173||!m_g76Orbit||history.authorizedSource.ordinal>=EDM173::CountForRecipe(m_edm130.recipe)||
            m_g76Orbit->done[static_cast<unsigned>(history.authorizedSource.ordinal)]||
            !m_motion.ReadEDM173Feedback(parent)||!parent.enabled||!parent.active||parent.faulted||
            !parent.tick||parent.liveEpoch!=history.liveEpoch||
            parent.authorizedChildren!=history.authorizedRows||parent.completedChildren!=history.completedRows||
            !EDM173::SameScope(parent.scope,m_edm173->plan.scope)||parent.fingerprint!=m_edm173->plan.fingerprint||
            (parent.tick>history.tick?parent.tick-history.tick:history.tick-parent.tick)>8ULL) return false;
        scope.compoundFingerprint=parent.fingerprint;scope.compoundArmEpoch=parent.scope.armEpoch;
        scope.compoundLine=parent.scope.line;
    }
    return EDM157::PauseModeAllowed(scope, mode.coordinator) && IsEDM157ScopeCurrentSameThread(scope);
}

void NCManager::LogEDM157SameThread(const char* event, const char* reason) const noexcept
{
    const auto& p = m_edm157;
    if(EDM157::IsG76NominalPause(p.scope))
    {
        const char* tag=EDM157::IsG76CenterNominalPause(p.scope)?
            (EDM130::IsG76CenterSecondReturnHold232(p.scope.recipe)?"EDM232":EDM130::IsG76CenterSecondFlushHold231(p.scope.recipe)?"EDM231":
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::Resume)?"EDM229":
                EDM130::IsG76CenterBrakeHold228(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::Brake)?"EDM228":
                EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::AnchorDwell)?"EDM227":
                EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::ReturnDwell)?"EDM226":
                EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::CenterReturnDwell)?"EDM225":
                EDM130::IsG76CenterDwellHold224(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::CenterDwell)?"EDM224":
                EDM130::IsG76CenterTopDwellHold222(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::TopDwell)?"EDM223":
                EDM130::IsG76CenterStartHold221(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::ToCenter)?"EDM221":
                EDM130::IsG76CenterAnchorHold220(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::ToAnchor)?"EDM220":
                EDM130::IsG76CenterReturnHold219(p.scope.recipe)||
                (p.feedback.flushPause&&p.feedback.flushPhase==EDM128::Phase::Returning)?"EDM219":
                p.feedback.flushPause||EDM130::IsG76CenterFlushHold218(p.scope.recipe)?"EDM218":"EDM217"):
            p.feedback.flushPause||EDM130::IsG76B2FlushHold200(p.scope.recipe)?"EDM200":"EDM199";
        char record[481]{};
        const int length=std::snprintf(record,sizeof(record),
            "[%s] event=%.24s reason=%.48s session=%llu hold=%llu startFloor=%llu resumeStart=%llu phase=%s stable=%u parentPC=%d dispatch=%llu commit=%llu fingerprint=%llu child=%llu epoch=%llu segment=%llu callbackRetained=1 pausedJog=0 discharge=0\n",
            tag,event?event:"UNKNOWN",reason?reason:"NONE",static_cast<unsigned long long>(m_edm94.session),
            static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(p.startFloor),
            static_cast<unsigned long long>(p.resumeStartSequence),EDM157::PhaseName(p.feedback.phase),p.feedback.stableCycles,
            p.pc,static_cast<unsigned long long>(p.scope.authorizedSource.dispatch),static_cast<unsigned long long>(p.compoundCommit),
            static_cast<unsigned long long>(p.scope.compoundFingerprint),static_cast<unsigned long long>(p.scope.authorizedSource.ordinal+1ULL),
            static_cast<unsigned long long>(p.scope.authorizedSource.epoch),static_cast<unsigned long long>(p.scope.authorizedSource.segment));
        if(length>0&&length<=480&&static_cast<std::size_t>(length)<sizeof(record))RtPrintf("%s",record);
        else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=PAUSE_SCOPE\n",tag);
        if(event&&(std::strcmp(event,"HOLD_READY")==0||std::strcmp(event,"RESUMED")==0))
        {
            const auto& f=p.feedback;
            const int poseLength=std::snprintf(record,sizeof(record),
                "[%s] event=PAUSE_POSE result=%.24s hold=%llu tick=%llu cursorRow=%llu historyDepth=%u localMmBits=%llu stationMmBits=%llu positionBits=%llu,%llu,%llu,%llu,%llu,%llu authority=NONE\n",
                tag,event,static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(f.tick),
                static_cast<unsigned long long>(f.cursorSource.ordinal),f.historyDepth,
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.localMm)),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.stationMm)),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.positionPulse[0U])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.positionPulse[1U])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.positionPulse[2U])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.positionPulse[3U])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.positionPulse[4U])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.positionPulse[5U])));
            if(poseLength>0&&poseLength<=480&&static_cast<std::size_t>(poseLength)<sizeof(record))RtPrintf("%s",record);
            else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=PAUSE_POSE\n",tag);
            if(EDM157::IsG76CenterNominalPause(p.scope)&&!f.flushPause)
            {
                const auto& work=f.nominalWork;
                const int workLength=std::snprintf(record,sizeof(record),
                    "[EDM217] event=NOMINAL_WORK_RECEIPT result=%.24s hold=%llu tick=%llu captured=%u elapsedUs=%llu totalWorkUs=%llu workTimeUs=%llu triggered=%llu completed=%llu evidence=RT_FROZEN_NOMINAL_GAP authority=NONE\n",
                    event,static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(f.tick),
                    f.nominalWorkCaptured?1U:0U,static_cast<unsigned long long>(work.elapsedUs),
                    static_cast<unsigned long long>(work.totalWorkUs),static_cast<unsigned long long>(p.scope.recipe.workTimeUs),
                    static_cast<unsigned long long>(work.triggeredEvents),static_cast<unsigned long long>(work.completedEvents));
                if(workLength>0&&workLength<=480&&static_cast<std::size_t>(workLength)<sizeof(record))RtPrintf("%s",record);
                else RtPrintf("[EDM217] event=LOG_FORMAT_ERROR part=NOMINAL_WORK_RECEIPT\n");
            }
            if(f.flushPause)
            {
                // These are the retained RT receipt, not a newer coordinator
                // sample. A delayed RUNNING ACK can follow a live phase change.
                const auto& flush=f.flush;
                const int timerLength=std::snprintf(record,sizeof(record),
                    "[%s] event=FLUSH_RECEIPT result=%.24s hold=%llu tick=%llu flushPhase=%s eventIndex=%llu stopAnchor=%u e5Anchor=%u target=%u top=%u exactReturn=%u dwells=%u workUs=%llu elapsedUs=%llu triggerTick=%llu triggerUs=%llu evidence=RT_FROZEN_FLUSH authority=NONE\n",
                    tag,event,static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(f.tick),
                    EDM128::PhaseName(f.flushPhase),static_cast<unsigned long long>(f.flushEventIndex),
                    f.anchorCaptured?1U:0U,flush.anchorCaptured?1U:0U,flush.targetCaptured?1U:0U,
                    flush.topCaptured?1U:0U,flush.exactReturn?1U:0U,flush.qualifiedDwells,
                    static_cast<unsigned long long>(flush.totalWorkUs),static_cast<unsigned long long>(flush.elapsedUs),
                    static_cast<unsigned long long>(flush.lastTriggerTick),static_cast<unsigned long long>(flush.lastTriggerUs));
                if(timerLength>0&&timerLength<=480&&static_cast<std::size_t>(timerLength)<sizeof(record))RtPrintf("%s",record);
                else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=FLUSH_RECEIPT\n",tag);
                const int pathLength=std::snprintf(record,sizeof(record),
                    "[%s] event=FLUSH_PATH result=%.24s hold=%llu anchorRow=%llu anchorSegment=%llu anchorLocalBits=%llu anchorStationBits=%llu targetRow=%llu targetSegment=%llu targetLocalBits=%llu targetStationBits=%llu evidence=RT_FROZEN_FLUSH authority=NONE\n",
                    tag,event,static_cast<unsigned long long>(p.holdSequence),
                    static_cast<unsigned long long>(flush.anchorOrdinal),static_cast<unsigned long long>(flush.anchorSegment),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(flush.anchorLocalMm)),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(flush.anchorStationMm)),
                    static_cast<unsigned long long>(flush.targetOrdinal),static_cast<unsigned long long>(flush.targetSegment),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(flush.targetLocalMm)),
                    static_cast<unsigned long long>(EDM130::CurveDoubleBits(flush.targetStationMm)));
                if(pathLength>0&&pathLength<=480&&static_cast<std::size_t>(pathLength)<sizeof(record))RtPrintf("%s",record);
                else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=FLUSH_PATH\n",tag);
            }
        }
        return;
    }
    const bool mainline = EDM149::IsFlushHoldTestArm(m_edm94.gapHistoryArm);
    const bool flushPause = p.feedback.flushPause;
    const bool pathReturn = !flushPause && p.scope.recipe.holdRetreatMode == 1U &&
        !p.scope.recipe.commissioningProfile;
    const char* tag = flushPause ? "EDM164" : pathReturn ? "EDM163" : "EDM162";
    // Observation only: use the existing timer snapshot and never make a
    // diagnostic read a motion/START grant. Held receipts expose E5 totals;
    // source samples continue advancing while machining time is inhibited.
    if (mainline)
    {
        EDM128::MotionFeedback timed{};
        const bool read = m_motion.ReadEDM130Feedback(timed);
        const bool identity = read && timed.enabled && timed.active && !timed.faulted && timed.workTimeFlush &&
            timed.session == m_edm94.session && timed.armEpoch == m_edm94.armEpoch &&
            timed.ownerGeneration == p.scope.ownerGeneration && timed.fixture == p.scope.historyFixture &&
            timed.liveEpoch == p.scope.authorizedSource.epoch &&
            timed.authorizedRows == p.scope.authorizedSource.ordinal + 1ULL &&
            EDM130::SameRecipe(timed.timerRecipe, p.scope.recipe) &&
            EDM116::SamePolicy(timed.rowPolicy, p.scope.policy) &&
            EDM116::SamePolicy(timed.coordinator.policy, p.scope.policy) &&
            EDM157::PauseModeAllowed(p.scope, timed.coordinator) && timed.tick && p.feedback.tick &&
            (timed.tick > p.feedback.tick ? timed.tick - p.feedback.tick : p.feedback.tick - timed.tick) <= 8ULL;
        const auto& work = timed.coordinator.work;
        char record[481]{};
        int length = std::snprintf(record, sizeof(record),
            "[%s] event=PAUSE_TIMER result=%.24s session=%llu hold=%llu read=%u identityMatch=%u tick=%llu pauseTick=%llu workUs=%llu elapsedUs=%llu triggered=%llu completed=%llu eligible=%u row=%llu B=%u authority=NONE\n",
            tag, event ? event : "UNKNOWN", static_cast<unsigned long long>(m_edm94.session),
            static_cast<unsigned long long>(p.holdSequence), read ? 1U : 0U, identity ? 1U : 0U,
            static_cast<unsigned long long>(timed.tick), static_cast<unsigned long long>(p.feedback.tick),
            static_cast<unsigned long long>(work.totalWorkUs), static_cast<unsigned long long>(work.elapsedUs),
            static_cast<unsigned long long>(work.triggeredEvents), static_cast<unsigned long long>(work.completedEvents),
            work.previousEligible ? 1U : 0U, static_cast<unsigned long long>(p.scope.authorizedSource.ordinal),
            static_cast<unsigned>(p.scope.policy.mode));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=PAUSE_TIMER\n", tag);
        length = std::snprintf(record, sizeof(record),
            "[%s] event=%.24s reason=%.48s session=%llu hold=%llu startFloor=%llu resumeStart=%llu phase=%s stable=%u row=%llu B=%u pc=%d NCstate=%u callbacksRetained=1 pausedJog=0 S92=0 S93=0\n",
            tag, event ? event : "UNKNOWN", reason ? reason : "NONE",
            static_cast<unsigned long long>(m_edm94.session), static_cast<unsigned long long>(p.holdSequence),
            static_cast<unsigned long long>(p.startFloor), static_cast<unsigned long long>(p.resumeStartSequence),
            EDM157::PhaseName(p.feedback.phase), p.feedback.stableCycles,
            static_cast<unsigned long long>(p.scope.authorizedSource.ordinal), static_cast<unsigned>(p.scope.policy.mode),
            p.pc, static_cast<unsigned>(static_cast<NCState>(m_state)));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=PAUSE_STATE\n", tag);
        length = std::snprintf(record, sizeof(record),
            "[%s] event=%s result=%.24s session=%llu hold=%llu tick=%llu cursorRow=%llu historyDepth=%u localMmBits=%llu stationMmBits=%llu positionBits=%llu,%llu,%llu,%llu,%llu,%llu authority=NONE\n",
            tag, flushPause ? "FLUSH_PAUSE_POSE" : pathReturn ? "MACHINING_ANCHOR" : "PAUSE_POSE", event ? event : "UNKNOWN",
            static_cast<unsigned long long>(m_edm94.session),
            static_cast<unsigned long long>(p.holdSequence), static_cast<unsigned long long>(p.feedback.tick),
            static_cast<unsigned long long>(p.feedback.cursorSource.ordinal), p.feedback.historyDepth,
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.localMm)),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.stationMm)),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[0])),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[1])),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[2])),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[3])),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[4])),
            static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[5])));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[%s] event=LOG_FORMAT_ERROR part=PAUSE_POSE\n", tag);
        if (flushPause)
        {
            length = std::snprintf(record, sizeof(record),
                "[EDM164] event=FLUSH_HOLD result=%.24s session=%llu hold=%llu flushPhase=%s eventIndex=%llu livePhase=%s anchor=%u exactE5Return=%u dwell=%u detached=%u workUs=%llu elapsedUs=%llu triggered=%llu completed=%llu authority=NONE\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(m_edm94.session),
                static_cast<unsigned long long>(p.holdSequence), EDM128::PhaseName(p.feedback.flushPhase),
                static_cast<unsigned long long>(p.feedback.flushEventIndex), EDM128::PhaseName(timed.coordinator.phase),
                p.feedback.anchorCaptured ? 1U : 0U, timed.coordinator.exactReturn ? 1U : 0U,
                timed.coordinator.stableCycles, timed.coordinator.detached ? 1U : 0U,
                static_cast<unsigned long long>(work.totalWorkUs), static_cast<unsigned long long>(work.elapsedUs),
                static_cast<unsigned long long>(work.triggeredEvents), static_cast<unsigned long long>(work.completedEvents));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM164] event=LOG_FORMAT_ERROR part=FLUSH_HOLD\n");
        }
        if (pathReturn)
        {
            EDM126::MotionFeedback live{};
            const bool liveRead = m_motion.ReadEDM126Feedback(live);
            const bool liveIdentity = liveRead && live.enabled && live.active && !live.faulted && live.rowActive &&
                !live.rowCompleted && live.session == m_edm94.session && live.armEpoch == m_edm94.armEpoch &&
                live.ownerGeneration == p.scope.ownerGeneration && live.fixture == p.scope.historyFixture &&
                live.liveEpoch == p.scope.authorizedSource.epoch &&
                EDM126::SameSource(live.authorizedSource, p.scope.authorizedSource) &&
                EDM157::ValidMapped(p.scope, live.traversal) && p.feedback.tick &&
                (live.traversal.tick > p.feedback.tick ? live.traversal.tick - p.feedback.tick :
                    p.feedback.tick - live.traversal.tick) <= 8ULL;
            const auto& f = p.feedback;
            length = std::snprintf(record, sizeof(record),
                "[EDM163] event=HOLD_PATH result=%.24s session=%llu hold=%llu anchor=%u planned=%u unavailable=%u topZero=%u exactReturn=%u interrupted=%u anchorRow=%llu targetRow=%llu targetLocalBits=%llu targetStationBits=%llu read=%u identityMatch=%u liveRow=%llu liveDepth=%u liveStationBits=%llu liveTick=%llu authority=NONE\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(m_edm94.session),
                static_cast<unsigned long long>(p.holdSequence), f.anchorCaptured ? 1U : 0U,
                f.retreatPlanned ? 1U : 0U, f.retreatUnavailable ? 1U : 0U, f.topCaptured ? 1U : 0U,
                f.exactReturn ? 1U : 0U, f.interrupted ? 1U : 0U,
                static_cast<unsigned long long>(f.cursorSource.ordinal),
                static_cast<unsigned long long>(f.retreatTarget.source.ordinal),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.retreatTarget.localMm)),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(f.retreatStationMm)),
                liveRead ? 1U : 0U, liveIdentity ? 1U : 0U,
                static_cast<unsigned long long>(live.traversal.cursorSource.ordinal), live.traversal.historyDepth,
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(live.traversal.stationMm)),
                static_cast<unsigned long long>(live.traversal.tick));
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
            else RtPrintf("[EDM163] event=LOG_FORMAT_ERROR part=HOLD_PATH\n");
        }
        return;
    }
    RtPrintf("[EDM157] event=%s reason=%s session=%llu hold=%llu startFloor=%llu resumeStart=%llu phase=%u stable=%u row=%llu pc=%d NCstate=%u cursorRow=%llu historyDepth=%u localMmBits=%llu stationMmBits=%llu positionBits=%llu,%llu,%llu,%llu,%llu,%llu callbacksRetained=1 pausedJog=0 S92=0 S93=0\n",
        event ? event : "UNKNOWN", reason ? reason : "NONE",
        static_cast<unsigned long long>(m_edm94.session), static_cast<unsigned long long>(p.holdSequence),
        static_cast<unsigned long long>(p.startFloor), static_cast<unsigned long long>(p.resumeStartSequence),
        static_cast<unsigned>(p.feedback.phase), p.feedback.stableCycles,
        static_cast<unsigned long long>(p.scope.authorizedSource.ordinal), p.pc, static_cast<unsigned>(static_cast<NCState>(m_state)),
        static_cast<unsigned long long>(p.feedback.cursorSource.ordinal), p.feedback.historyDepth,
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.localMm)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.stationMm)),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[0])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[1])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[2])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[3])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[4])),
        static_cast<unsigned long long>(EDM130::CurveDoubleBits(p.feedback.positionPulse[5])));
}

bool NCManager::TryHoldEDM157SameThread() noexcept
{
    if (!EDM157::SupportsArm(m_edm94.gapHistoryArm) || !m_edm94.latched ||
        m_edm94.cancelled || m_edm94.releaseRequested || m_edm94.complete) return false;
    EDM157::Scope scope{};
    if (m_edm157.active)
    {
        scope = m_edm157.scope;
        if (!IsEDM157ScopeCurrentSameThread(scope)) return false;
    }
    else if (m_state != NCState::RUN || !CaptureEDM157ScopeSameThread(scope)) return false;
    const bool renewCheckpoint=m_edm157.active&&m_edm200.queued&&!m_edm200.resumed&&
        m_edm200.holdSequence==m_edm157.holdSequence&&m_edm200.compoundCommit==m_edm157.compoundCommit&&
        EDM157::SameScope(m_edm200.scope,scope);
    if(renewCheckpoint&&m_edm157.feedback.phase!=EDM157::Phase::Idle&&
        (!EDM157::ValidPauseContext(m_edm157.feedback)||
            (EDM130::IsG76CenterHold217(scope.recipe)?
                (!E217NominalCheckpointReceipt(m_edm157.feedback)||m_edm200.eventIndex):
                EDM130::IsG76CenterAnyFlushHold232(scope.recipe)?
                (!E232CenterFlushCheckpointReceipt(m_edm157.feedback)||
                    m_edm200.eventIndex!=EDM130::G76CenterHoldEventIndex(scope.recipe)):
                (!m_edm157.feedback.flushPause||m_edm157.feedback.flushPhase!=EDM128::Phase::Outbound||
                    m_edm157.feedback.flushEventIndex!=m_edm200.eventIndex))))return false;
    if (m_edm157HoldSerial == (std::numeric_limits<std::uint64_t>::max)()) return false;
    const std::uint64_t sequence = ++m_edm157HoldSerial;
    if (!m_motion.RequestEDM157Hold(scope, sequence, m_startDiagnosticRequests)) return false;
    const bool entering = m_state == NCState::RUN;
    m_edm157 = EDM157NCState{}; auto& p = m_edm157;
    p.active = true; p.scope = scope; p.holdSequence = sequence; p.startFloor = m_startDiagnosticRequests;
    if(EDM157::IsG76NominalPause(scope))p.compoundCommit=m_g76Orbit->commit;
    if(renewCheckpoint)m_edm200.holdSequence=sequence;
    p.requestTick = m_edm94.feedback.lastTick; p.pc = GetActiveDispatchPC();
    ForceEDMPLCOutputsOffSameThread("EDM157_HOLD");
    CancelPathCoreHoldAutomaticSameThread("EDM157_HOLD");
    CancelFeedHoldBoundaryShadow(true);
    m_feedHoldResumeGate.Cancel(true);
    ClearHoldResumeAlarmAdmission(HoldResumeAdmissionKind::PROGRAM_HOLD);
    m_feedHoldNCSettleRequestSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    if (entering) PausePathCoreLiveRetentionSameThread();
    m_state = NCState::HOLD; m_pauseAfterBlock = false;
    FenceEDMRecipeHoldSameThread();
    LogEDM157SameThread("HOLD_REQUEST", entering ? "RETAIN_ACTIVE_CNC_ROW" : "NEW_HOLD_REVOKES_PENDING_START");
    return true;
}

void NCManager::ServiceEDM200AutoFlushHoldSameThread(bool feedbackCurrent)
{
    // L3 selects one observation checkpoint. Only the ordinary HOLD path
    // can submit a pause; physical zero and a new operator START remain required.
    const auto& s=m_edm94;const auto& recipe=m_edm130.recipe;
    if(!feedbackCurrent||!m_edm130.bound||s.gapHistoryArm!=EDM173::B2ArmP||
        !EDM130::IsG76B2FlushHold200(recipe)||!EDM130::ValidRecipe(recipe)||
        !s.session||m_edm200.queued||m_edm157.active||m_state!=NCState::RUN||
        !s.latched||!s.ready||!s.everArmed||s.armPending||s.cancelled||
        s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        s.finishPending||s.complete||m_resetContinuationPhase!=ResetContinuationPhase::IDLE||
        !s.feedback.active||s.feedback.faulted||s.feedback.forceZero||
        AlarmManager::GetInstance().HasAlarm())return;
    EDM128::MotionFeedback mode{};EDM126::MotionFeedback history{};
    const double height=EDM130::FrozenJumpHeightMm(recipe);
    if(!m_motion.ReadEDM130Feedback(mode)||!mode.enabled||!mode.active||mode.faulted||
        !mode.workTimeFlush||mode.fixture!=EDM173::HistoryFixture||
        mode.session!=s.session||mode.armEpoch!=s.armEpoch||mode.ownerGeneration!=s.lease.generation||
        mode.liveEpoch!=m_motion.GetCurrentExecutionEpoch()||!mode.authorizedRows||mode.authorizedRows>25U||
        mode.completedRows+1U!=mode.authorizedRows||!mode.tick||!mode.nowUs||!s.feedback.lastTick||
        (mode.tick>s.feedback.lastTick?mode.tick-s.feedback.lastTick:s.feedback.lastTick-mode.tick)>8ULL||
        !EDM130::SameRecipe(mode.timerRecipe,recipe)||!EDM116::SamePolicy(mode.rowPolicy,mode.coordinator.policy))return;
    const auto& event=mode.coordinator;
    if(event.policy.fixture!=EDM116::Fixture::ModalG76Gap173||event.policy.mode!=EDM116::Mode::B2||
        event.phase!=EDM128::Phase::Outbound||event.detached||!event.anchorCaptured||event.faulted||event.stopped||
        event.eventIndex!=1ULL||event.work.triggeredEvents!=1ULL||event.work.completedEvents||
        !EDM130::ValidWorkEvidence(event.work)||!std::isfinite(event.stageDistanceMm)||
        event.stageDistanceMm<height*0.25||event.stageDistanceMm>height+EDM126::Roundoff(height)||
        !EDM128::ValidHeightProof(event,height,recipe.deepFlushCycleInterval,
            EDM130::FrozenDeepMultiplier(recipe),EDM130::HasBoundDeep(recipe))||
        !EDM128::ValidSpeedProof(event,recipe.flushStages))return;
    EDM157::Scope scope{};
    if(!CaptureEDM157ScopeSameThread(scope)||!EDM157::PauseModeAllowed(scope,event)||
        !EDM130::SameRecipe(scope.recipe,recipe)||!EDM116::SamePolicy(scope.policy,mode.rowPolicy)||
        !EDM126::SameSource(scope.authorizedSource,event.anchor.authorizedSource)||
        !m_motion.ReadEDM126Feedback(history)||!history.enabled||!history.active||history.faulted||
        !history.sourceReady||!history.rowActive||history.rowCompleted||
        history.fixture!=mode.fixture||history.session!=mode.session||history.armEpoch!=mode.armEpoch||
        history.ownerGeneration!=mode.ownerGeneration||history.liveEpoch!=mode.liveEpoch||
        history.authorizedRows!=mode.authorizedRows||history.completedRows!=mode.completedRows||
        history.tick!=mode.tick||history.nowUs!=mode.nowUs||history.lastMappingTick!=history.tick||
        history.lastMappingUs!=history.nowUs||mode.lastMappingTick!=history.lastMappingTick||
        mode.lastMappingUs!=history.lastMappingUs||history.traversal.tick!=history.tick||
        history.traversal.nowUs!=history.nowUs||!EDM126::SameSource(history.authorizedSource,scope.authorizedSource)||
        !EDM157::ValidMapped(scope,history.traversal)||!EDM157::ValidMapped(scope,event.anchor)||
        history.traversal.cursorSource.ordinal>=event.anchor.cursorSource.ordinal)return;
    const bool accepted=TryHoldEDM157SameThread();
    // A second capture in TryHold must still name the same live compound child.
    // The RT receipt below must additionally confirm the requested flush phase.
    if(!accepted||!EDM157::SameScope(m_edm157.scope,scope)||!m_edm157.compoundCommit)
    {
        RejectEDM94SameThread("EDM200_AUTO_HOLD_REQUEST_REJECTED",m_gapDryRun.sourceLine);return;
    }
    m_edm200.scope=scope;m_edm200.compoundCommit=m_edm157.compoundCommit;
    m_edm200.holdSequence=m_edm157.holdSequence;m_edm200.eventIndex=event.eventIndex;m_edm200.queued=true;
    RtPrintf("[EDM200] event=AUTO_HOLD_REQUEST result=QUEUED session=%llu L=3 B=2 eventIndex=%llu phase=OUTBOUND tick=%llu hold=%llu startFloor=%llu cursorRow=%llu anchorRow=%llu resume=MANUAL_FRESH_START discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(event.eventIndex),
        static_cast<unsigned long long>(mode.tick),static_cast<unsigned long long>(m_edm157.holdSequence),
        static_cast<unsigned long long>(m_edm157.startFloor),
        static_cast<unsigned long long>(history.traversal.cursorSource.ordinal),
        static_cast<unsigned long long>(event.anchor.cursorSource.ordinal));
}

void NCManager::ServiceEDM217AutoNominalHoldSameThread(bool feedbackCurrent)
{
    // L1 selects a committed nominal-GAP work checkpoint. It requests the
    // ordinary HOLD transaction; only a fresh operator START can resume it.
    const auto& s=m_edm94;const auto& recipe=m_edm130.recipe;
    if(!feedbackCurrent||!m_edm130.bound||!EDM130::IsG76CenterHold217(recipe)||
        !EDM173::RecipeMatchesArm(s.gapHistoryArm,recipe)||!EDM130::ValidRecipe(recipe)||
        !s.session||m_edm200.queued||m_edm157.active||m_state!=NCState::RUN||
        !s.latched||!s.ready||!s.everArmed||s.armPending||s.cancelled||
        s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        s.finishPending||s.complete||m_resetContinuationPhase!=ResetContinuationPhase::IDLE||
        !s.feedback.active||s.feedback.faulted||s.feedback.forceZero||
        AlarmManager::GetInstance().HasAlarm())return;
    EDM128::MotionFeedback mode{};EDM126::MotionFeedback history{};
    if(!m_motion.ReadEDM130Feedback(mode)||!mode.enabled||!mode.active||mode.faulted||
        !mode.workTimeFlush||mode.fixture!=EDM173::HistoryFixture||
        mode.session!=s.session||mode.armEpoch!=s.armEpoch||mode.ownerGeneration!=s.lease.generation||
        mode.liveEpoch!=m_motion.GetCurrentExecutionEpoch()||!mode.authorizedRows||mode.authorizedRows>25U||
        mode.completedRows+1U!=mode.authorizedRows||!mode.tick||!mode.nowUs||!s.feedback.lastTick||
        (mode.tick>s.feedback.lastTick?mode.tick-s.feedback.lastTick:s.feedback.lastTick-mode.tick)>8ULL||
        !EDM130::SameRecipe(mode.timerRecipe,recipe)||!EDM116::SamePolicy(mode.rowPolicy,mode.coordinator.policy))return;
    const auto& event=mode.coordinator;const auto& work=event.work;
    if(event.policy.fixture!=EDM116::Fixture::ModalG76Gap173||
        static_cast<unsigned>(event.policy.mode)!=EDM173::ModeForArm(s.gapHistoryArm)||
        event.phase!=EDM128::Phase::Gap||event.detached||event.faulted||event.stopped||
        event.eventIndex||!EDM130::ValidWorkEvidence(work)||work.workTimeUs!=recipe.workTimeUs||
        work.triggeredEvents||work.completedEvents||work.due||
        work.elapsedUs<work.workTimeUs/4ULL||work.elapsedUs>=work.workTimeUs)return;
    EDM157::Scope scope{};
    if(!CaptureEDM157ScopeSameThread(scope)||!EDM157::PauseModeAllowed(scope,event)||
        !EDM130::SameRecipe(scope.recipe,recipe)||!EDM116::SamePolicy(scope.policy,mode.rowPolicy)||
        !m_motion.ReadEDM126Feedback(history)||!history.enabled||!history.active||history.faulted||
        !history.sourceReady||!history.rowActive||history.rowCompleted||
        history.fixture!=mode.fixture||history.session!=mode.session||history.armEpoch!=mode.armEpoch||
        history.ownerGeneration!=mode.ownerGeneration||history.liveEpoch!=mode.liveEpoch||
        history.authorizedRows!=mode.authorizedRows||history.completedRows!=mode.completedRows||
        history.tick!=mode.tick||history.nowUs!=mode.nowUs||history.lastMappingTick!=history.tick||
        history.lastMappingUs!=history.nowUs||mode.lastMappingTick!=history.lastMappingTick||
        mode.lastMappingUs!=history.lastMappingUs||history.traversal.tick!=history.tick||
        history.traversal.nowUs!=history.nowUs||!EDM126::SameSource(history.authorizedSource,scope.authorizedSource)||
        !EDM157::ValidMapped(scope,history.traversal))return;
    const bool accepted=TryHoldEDM157SameThread();
    // TryHold recaptures the live child. A different child cannot consume
    // this checkpoint even when both observations belong to the same parent.
    if(!accepted||!EDM157::SameScope(m_edm157.scope,scope)||!m_edm157.compoundCommit)
    {RejectEDM94SameThread("EDM217_AUTO_HOLD_REQUEST_REJECTED",m_gapDryRun.sourceLine);return;}
    m_edm200.scope=scope;m_edm200.compoundCommit=m_edm157.compoundCommit;
    m_edm200.holdSequence=m_edm157.holdSequence;m_edm200.eventIndex=0ULL;m_edm200.queued=true;
    PrintNCG76DiagnosticLine("[EDM217] event=AUTO_HOLD_REQUEST result=QUEUED session=%llu L=1 B=%u eventIndex=0 phase=GAP tick=%llu hold=%llu startFloor=%llu child=%llu elapsedUs=%llu workTimeUs=%llu resume=MANUAL_FRESH_START discharge=0\n",
        static_cast<unsigned long long>(s.session),EDM173::ModeForArm(s.gapHistoryArm),
        static_cast<unsigned long long>(mode.tick),static_cast<unsigned long long>(m_edm157.holdSequence),
        static_cast<unsigned long long>(m_edm157.startFloor),static_cast<unsigned long long>(scope.authorizedSource.ordinal+1ULL),
        static_cast<unsigned long long>(work.elapsedUs),static_cast<unsigned long long>(work.workTimeUs));
}

void NCManager::ServiceEDM218AutoCenterFlushHoldSameThread(bool feedbackCurrent)
{
    // L2-L5 retain moving-leg checkpoints; L6/L7 pause apex/outgoing center
    // dwells, L8 the returning center dwell and EDM226 L9 the final anchor dwell.
    // EDM227 L10 pauses initial settlement on the still attached main path.
    // EDM228 L11 retains the initial moving Brake until a fresh START.
    // EDM231 L231 pauses the second Outbound; EDM232 L232 pauses its Returning.
    // Both keep event 1 uninterrupted and retain the two-event cap.
    // START restarts the full dwell;
    // held wall time is never completion credit and no checkpoint grants JOG.
    const auto& s=m_edm94;const auto& recipe=m_edm130.recipe;
    const bool topDwell=EDM130::IsG76CenterTopDwellHold222(recipe);
    const bool centerDwell=EDM130::IsG76CenterDwellHold224(recipe);
    const bool centerReturnDwell=EDM130::IsG76CenterReturnDwellHold225(recipe);
    const bool finalDwell=EDM130::IsG76CenterFinalDwellHold226(recipe);
    const bool initialDwell=EDM130::IsG76CenterInitialDwellHold227(recipe);
    const bool braking=EDM130::IsG76CenterBrakeHold228(recipe);
    const bool preCapture=initialDwell||braking;
    const bool dwelling=topDwell||centerDwell||centerReturnDwell||finalDwell||initialDwell;
    const bool toCenter=EDM130::IsG76CenterStartHold221(recipe);
    const bool toAnchor=EDM130::IsG76CenterAnchorHold220(recipe);
    const bool secondReturning=EDM130::IsG76CenterSecondReturnHold232(recipe);
    const bool returning=EDM130::IsG76CenterReturnHold219(recipe)||secondReturning;
    const bool second=EDM130::IsG76CenterSecondFlushHold231(recipe);
    const std::uint64_t holdEvent=EDM130::G76CenterHoldEventIndex(recipe);
    if(!feedbackCurrent||!m_edm130.bound||!EDM173::RecipeMatchesArm(s.gapHistoryArm,recipe)||
        !EDM130::IsG76CenterAnyFlushHold232(recipe)||!EDM130::ValidRecipe(recipe)||
        !s.session||m_edm200.queued||m_edm157.active||m_state!=NCState::RUN||
        !s.latched||!s.ready||!s.everArmed||s.armPending||s.cancelled||
        s.resetSeen||s.resetCommitted||s.releaseRequested||s.rtFaultReported||
        s.finishPending||s.complete||m_resetContinuationPhase!=ResetContinuationPhase::IDLE||
        !s.feedback.active||s.feedback.faulted||s.feedback.forceZero||
        AlarmManager::GetInstance().HasAlarm())return;
    EDM128::MotionFeedback mode{};EDM126::MotionFeedback history{};
    const double height=EDM130::FrozenJumpHeightMm(recipe);
    if(!m_motion.ReadEDM130Feedback(mode)||!mode.enabled||!mode.active||mode.faulted||
        !mode.workTimeFlush||mode.fixture!=EDM173::HistoryFixture||
        mode.session!=s.session||mode.armEpoch!=s.armEpoch||mode.ownerGeneration!=s.lease.generation||
        mode.liveEpoch!=m_motion.GetCurrentExecutionEpoch()||!mode.authorizedRows||mode.authorizedRows>25U||
        mode.completedRows+1U!=mode.authorizedRows||!mode.tick||!mode.nowUs||!s.feedback.lastTick||
        (mode.tick>s.feedback.lastTick?mode.tick-s.feedback.lastTick:s.feedback.lastTick-mode.tick)>8ULL||
        !EDM130::SameRecipe(mode.timerRecipe,recipe)||!EDM116::SamePolicy(mode.rowPolicy,mode.coordinator.policy))return;
    const auto& event=mode.coordinator;double span=0.0,traveled=0.0;
    std::uint64_t dwellElapsedUs=0ULL;
    if((!braking&&!toCenter&&!dwelling&&(!(toAnchor?E220CenterAnchorSpan(event,span):EDM128::TrySpeedStageSpanMm(event,span))||!EDM126::Positive(span)))||
        event.policy.fixture!=EDM116::Fixture::ModalG76Gap173||
        static_cast<unsigned>(event.policy.mode)!=EDM173::ModeForArm(s.gapHistoryArm)||
        event.phase!=(braking?EDM128::Phase::Brake:initialDwell?EDM128::Phase::AnchorDwell:finalDwell?EDM128::Phase::ReturnDwell:centerReturnDwell?EDM128::Phase::CenterReturnDwell:centerDwell?EDM128::Phase::CenterDwell:topDwell?EDM128::Phase::TopDwell:toCenter?EDM128::Phase::ToCenter:toAnchor?EDM128::Phase::ToAnchor:returning?EDM128::Phase::Returning:EDM128::Phase::Outbound)||
        (preCapture?(event.detached||event.centerCaptured||event.anchorCaptured):
            (!event.detached||!event.centerCaptured||!event.anchorCaptured))||event.faulted||event.stopped||
        event.eventIndex!=holdEvent||event.work.triggeredEvents!=holdEvent||event.work.completedEvents!=holdEvent-1ULL||
        !EDM130::ValidWorkEvidence(event.work)||!std::isfinite(event.stageDistanceMm)||
        // Return samples publish remaining distance to that leg's endpoint.
        // A phase transition without a speed sample is never 25% travel.
        (!braking&&!toCenter&&!dwelling&&(((returning||toAnchor)?(event.speedStage==EDM128::SpeedStage::None||event.stageDistanceMm>span*0.75):
            event.stageDistanceMm<span*0.25)||event.stageDistanceMm>span+EDM126::Roundoff(span)))||
        !EDM128::ValidHeightProof(event,height,recipe.deepFlushCycleInterval,
            EDM130::FrozenDeepMultiplier(recipe),EDM130::HasBoundDeep(recipe))||
        !EDM128::ValidSpeedProof(event,recipe.flushStages))return;
    if(dwelling) {
        const auto duration=event.policy.profile.dwellUs;
        if(!duration||!event.stableCycles||!event.stableStartUs||event.stableStartUs>mode.nowUs)return;
        dwellElapsedUs=mode.nowUs-event.stableStartUs;
        if(dwellElapsedUs<duration/4ULL||dwellElapsedUs>=duration)return;
    }
    EDM157::Scope scope{};
    if(!CaptureEDM157ScopeSameThread(scope)||!EDM157::PauseModeAllowed(scope,event)||
        !EDM130::SameRecipe(scope.recipe,recipe)||!EDM116::SamePolicy(scope.policy,mode.rowPolicy)||
        (!preCapture&&!EDM126::SameSource(scope.authorizedSource,event.anchor.authorizedSource))||
        !m_motion.ReadEDM126Feedback(history)||!history.enabled||!history.active||history.faulted||
        !history.sourceReady||!history.rowActive||history.rowCompleted||
        history.fixture!=mode.fixture||history.session!=mode.session||history.armEpoch!=mode.armEpoch||
        history.ownerGeneration!=mode.ownerGeneration||history.liveEpoch!=mode.liveEpoch||
        history.authorizedRows!=mode.authorizedRows||history.completedRows!=mode.completedRows||
        history.tick!=mode.tick||history.nowUs!=mode.nowUs||history.lastMappingTick!=history.tick||
        history.lastMappingUs!=history.nowUs||mode.lastMappingTick!=history.lastMappingTick||
        mode.lastMappingUs!=history.lastMappingUs||history.traversal.tick!=history.tick||
        history.traversal.nowUs!=history.nowUs||!EDM126::SameSource(history.authorizedSource,scope.authorizedSource)||
        !EDM157::ValidMapped(scope,history.traversal)||
        (!preCapture&&(!EDM157::ValidMapped(scope,event.anchor)||
            !EDM126::SameSource(history.traversal.cursorSource,event.anchor.cursorSource)||
            history.traversal.localMm!=event.anchor.localMm||history.traversal.stationMm!=event.anchor.stationMm||
            history.traversal.historyDepth!=event.anchor.historyDepth)))return;
    // Main-path braking uses its constant deceleration. Leave at least two
    // 2ms freshness windows plus one RT sample before the predicted stop.
    // This margin is only a test trigger; the RT receipt must still be Brake.
    if(braking&&(!EDM126::Positive(history.traversal.velocityMmSec)||
        history.traversal.accelerationMmSec2>=0.0||
        history.traversal.velocityMmSec<=-history.traversal.accelerationMmSec2*0.00425))return;
    if(toCenter&&(!E221CenterStartProgress(event,history.traversal.positionPulse,span,traveled)||
        traveled<span*0.25))return;
    if(dwelling) {
        EDM204::Geometry geometry{};const auto& pose=history.traversal;
        // Initial dwell has no canonical E5 anchor yet. The current scoped,
        // mapped main-path pose is frozen by the RT transaction on acceptance.
        if(!EDM128::Zero(pose)||pose.accelerationMmSec2!=0.0||pose.accelerationPps2!=EDM126::Point{}||
            (!initialDwell&&(!EDM128::CenterAxisGeometry(event,geometry)||
                !EDM109::SamePulse(pose.positionPulse,finalDwell?geometry.anchorPulse:
                    (centerDwell||centerReturnDwell)?geometry.centerPulse:geometry.topPulse))))return;
    }
    const bool accepted=TryHoldEDM157SameThread();
    // A second capture in TryHold must still name the same live compound child.
    // The RT receipt below must additionally confirm the requested flush phase.
    if(!accepted||!EDM157::SameScope(m_edm157.scope,scope)||!m_edm157.compoundCommit)
    {
        RejectEDM94SameThread(secondReturning?"EDM232_AUTO_HOLD_REQUEST_REJECTED":second?"EDM231_AUTO_HOLD_REQUEST_REJECTED":braking?"EDM228_AUTO_HOLD_REQUEST_REJECTED":initialDwell?"EDM227_AUTO_HOLD_REQUEST_REJECTED":finalDwell?"EDM226_AUTO_HOLD_REQUEST_REJECTED":centerReturnDwell?"EDM225_AUTO_HOLD_REQUEST_REJECTED":centerDwell?"EDM224_AUTO_HOLD_REQUEST_REJECTED":topDwell?"EDM223_AUTO_HOLD_REQUEST_REJECTED":toCenter?"EDM221_AUTO_HOLD_REQUEST_REJECTED":toAnchor?"EDM220_AUTO_HOLD_REQUEST_REJECTED":returning?"EDM219_AUTO_HOLD_REQUEST_REJECTED":"EDM218_AUTO_HOLD_REQUEST_REJECTED",
            m_gapDryRun.sourceLine);return;
    }
    m_edm200.scope=scope;m_edm200.compoundCommit=m_edm157.compoundCommit;
    m_edm200.holdSequence=m_edm157.holdSequence;m_edm200.eventIndex=event.eventIndex;m_edm200.queued=true;
    if(braking)
        PrintNCG76DiagnosticLine("[EDM228] event=AUTO_HOLD_REQUEST result=QUEUED session=%llu L=11 B=%u eventIndex=1 phase=BRAKE tick=%llu hold=%llu startFloor=%llu child=%llu velocityMmSec=%.9g accelerationMmSec2=%.9g initialDwell=FULL_AFTER_START resume=MANUAL_FRESH_START discharge=0\n",
            static_cast<unsigned long long>(s.session),EDM173::ModeForArm(s.gapHistoryArm),
            static_cast<unsigned long long>(mode.tick),static_cast<unsigned long long>(m_edm157.holdSequence),
            static_cast<unsigned long long>(m_edm157.startFloor),
            static_cast<unsigned long long>(scope.authorizedSource.ordinal+1ULL),
            history.traversal.velocityMmSec,history.traversal.accelerationMmSec2);
    else if(dwelling)
        PrintNCG76DiagnosticLine("[%s] event=AUTO_HOLD_REQUEST result=QUEUED session=%llu L=%u B=%u eventIndex=1 phase=%s tick=%llu hold=%llu startFloor=%llu child=%llu dwellElapsedUs=%llu dwellUs=%llu dwellRestart=FULL resume=MANUAL_FRESH_START discharge=0\n",
            E228CenterDwellIntentTag(recipe),static_cast<unsigned long long>(s.session),static_cast<unsigned>(recipe.flushHoldTest),
            EDM173::ModeForArm(s.gapHistoryArm),EDM128::PhaseName(event.phase),static_cast<unsigned long long>(mode.tick),static_cast<unsigned long long>(m_edm157.holdSequence),
            static_cast<unsigned long long>(m_edm157.startFloor),
            static_cast<unsigned long long>(scope.authorizedSource.ordinal+1ULL),
            static_cast<unsigned long long>(dwellElapsedUs),static_cast<unsigned long long>(event.policy.profile.dwellUs));
    else PrintNCG76DiagnosticLine("[%s] event=AUTO_HOLD_REQUEST result=QUEUED session=%llu L=%u B=%u eventIndex=%llu phase=%s tick=%llu hold=%llu startFloor=%llu child=%llu distanceMm=%.9g %s=%.9g resume=MANUAL_FRESH_START discharge=0\n",
        secondReturning?"EDM232":second?"EDM231":toCenter?"EDM221":toAnchor?"EDM220":returning?"EDM219":"EDM218",static_cast<unsigned long long>(s.session),static_cast<unsigned>(recipe.flushHoldTest),
        EDM173::ModeForArm(s.gapHistoryArm),static_cast<unsigned long long>(event.eventIndex),EDM128::PhaseName(event.phase),
        static_cast<unsigned long long>(mode.tick),static_cast<unsigned long long>(m_edm157.holdSequence),
        static_cast<unsigned long long>(m_edm157.startFloor),
        static_cast<unsigned long long>(scope.authorizedSource.ordinal+1ULL),toCenter?traveled:event.stageDistanceMm,
        toCenter?"centerSpanMm":toAnchor?"anchorSpanMm":"axialSpanMm",span);
}

bool NCManager::IsEDM200AutoHoldCheckpointCompleteSameThread() const noexcept
{
    const auto& recipe=m_edm130.recipe;const auto& s=m_edm94;const auto& c=m_edm200;
    if(EDM130::IsG76CommissioningRecipe(recipe)&&EDM130::ValidRecipe(recipe))return true;
    if(!recipe.flushHoldTest||((EDM130::IsG76CenterRepeat230(recipe)||EDM130::IsG76ParentSequence233(recipe)||EDM130::IsG76CenterSequence239(recipe)||EDM130::IsG76CenterSequence240(recipe)||EDM130::IsG76CenterSequence241(recipe)||EDM130::IsG76CenterSequence242(recipe)||EDM130::IsG76CenterSequence243(recipe)||EDM130::IsG76CenterSequence244(recipe)||EDM130::IsG76CenterSequence245(recipe)||EDM130::IsG76CenterSequence246(recipe)||EDM130::IsG76CenterSequence247(recipe)||EDM130::IsG76CenterSequence248(recipe))&&EDM130::ValidRecipe(recipe)))return true;
    const bool center=EDM130::IsG76CenterHold217(recipe);
    const bool centerFlush=EDM130::IsG76CenterAnyFlushHold232(recipe);
    if(!EDM173::RecipeMatchesArm(s.gapHistoryArm,recipe)||!m_edm173||!m_g76Orbit||!m_edm130.bound||
        (!center&&!centerFlush&&!EDM130::IsG76B2FlushHold200(recipe))||!EDM130::ValidRecipe(recipe)||
        !EDM157::ValidScope(c.scope)||!c.queued||!c.resumed||!c.holdSequence||!c.resumeStartSequence||
        c.eventIndex!=(center?0ULL:EDM130::G76CenterHoldEventIndex(recipe))||c.scope.authorizedSource.scope.session!=s.session||
        !EDM130::SameRecipe(c.scope.recipe,recipe)||!EDM130::SameRecipe(m_edm173->plan.recipe,recipe)||
        c.scope.compoundFingerprint!=m_edm173->plan.fingerprint||c.scope.compoundArmEpoch!=s.armEpoch||
        c.compoundCommit!=m_g76Orbit->commit||c.scope.authorizedSource.dispatch!=m_g76Orbit->dispatch||
        c.scope.authorizedSource.sourcePC!=m_g76Orbit->pc||c.scope.compoundLine!=m_g76Orbit->line||
        c.scope.authorizedSource.scope.run!=s.run||c.scope.authorizedSource.scope.cache!=s.cache||
        c.scope.authorizedSource.scope.translationGeneration!=m_edm173->plan.nominal.translationGeneration||
        c.scope.ownerGeneration!=s.lease.generation||c.scope.authorizedSource.ordinal>=EDM173::CountForRecipe(recipe))return false;
    const unsigned child=static_cast<unsigned>(c.scope.authorizedSource.ordinal);
    const auto& id=m_g76Orbit->receipt.identities[child];
    return m_g76Orbit->accepted[child]&&m_g76Orbit->started[child]&&id.IsAssigned()&&
        id.source==MotionCommandSource::NC_MEMORY&&id.epoch==c.scope.authorizedSource.epoch&&
        id.segmentId==c.scope.authorizedSource.segment&&
        id.sourceBlockId==static_cast<MotionSourceBlockId>(m_g76Orbit->pc);
}

void NCManager::ServiceEDM165AutoFlushHoldSameThread(bool feedbackCurrent)
{
    // NC selects a checkpoint; the existing HOLD transaction retains all
    // trajectory, mapping, physical-zero and fresh-START authority.
    const auto& s = m_edm94;
    const auto& recipe = m_edm130.recipe;
    if(EDM130::IsG76CommissioningRecipe(recipe)&&EDM130::ValidRecipe(recipe))return;
    const unsigned selection = recipe.flushHoldTest;
    const unsigned target = EDM130::FlushHoldTargetMode(selection);
    const bool targeted = selection >= 2U;
    if (!feedbackCurrent || !m_edm130.bound || !EDM130::ValidFlushHoldSelection(s.gapHistoryArm, selection) ||
        recipe.holdRetreatMode != 1U || !EDM130::ValidRecipe(recipe) ||
        !EDM149::IsFlushHoldTestArm(s.gapHistoryArm) || !s.session ||
        m_edm165AutoHoldSession == s.session || m_edm157.active || m_state != NCState::RUN ||
        !s.latched || !s.ready || !s.everArmed || s.armPending || s.cancelled ||
        s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.finishPending || s.complete || m_resetContinuationPhase != ResetContinuationPhase::IDLE ||
        !s.feedback.active || s.feedback.faulted || s.feedback.forceZero ||
        AlarmManager::GetInstance().HasAlarm()) return;
    if (targeted && (s.gapHistoryArm != 274U || recipe.diagnosticControlSource != 3U ||
        recipe.commissioningProfile || s.gapModeFixture != EDM116::Fixture::ModalProgram156 ||
        E149HistoryFixture(s.gapHistoryArm) != EDM149::ProgramHistoryFixture)) return;
    const double height = EDM130::FrozenJumpHeightMm(recipe);
    EDM128::MotionFeedback mode{};
    if (!m_motion.ReadEDM130Feedback(mode) || !mode.enabled || !mode.active || mode.faulted ||
        !mode.workTimeFlush || mode.fixture != E149HistoryFixture(s.gapHistoryArm) ||
        mode.session != s.session || mode.armEpoch != s.armEpoch ||
        mode.ownerGeneration != s.lease.generation || mode.liveEpoch != m_motion.GetCurrentExecutionEpoch() ||
        mode.authorizedRows != s.next || mode.completedRows + 1U != mode.authorizedRows ||
        !mode.tick || !mode.nowUs || !s.feedback.lastTick ||
        (mode.tick > s.feedback.lastTick ? mode.tick - s.feedback.lastTick : s.feedback.lastTick - mode.tick) > 8ULL ||
        !EDM130::SameRecipe(mode.timerRecipe, recipe) ||
        !EDM116::SamePolicy(mode.rowPolicy, mode.coordinator.policy) ||
        mode.coordinator.phase != EDM128::Phase::Outbound ||
        !std::isfinite(mode.coordinator.stageDistanceMm) || mode.coordinator.stageDistanceMm < height * 0.25 ||
        mode.coordinator.stageDistanceMm > height + EDM126::Roundoff(height) ||
        (mode.coordinator.policy.mode != EDM116::Mode::B2 && !mode.coordinator.detached) ||
        !mode.coordinator.anchorCaptured || !EDM130::ValidWorkEvidence(mode.coordinator.work) ||
        mode.coordinator.faulted || mode.coordinator.stopped) return;
    const auto& event = mode.coordinator;
    if (!targeted)
    {
        // L1 remains the global first E5 event, regardless of B policy.
        if (event.eventIndex != 1ULL || event.work.triggeredEvents != 1ULL || event.work.completedEvents != 0ULL) return;
    }
    else if (target >= 3U || static_cast<unsigned>(event.policy.mode) != target ||
        event.eventIndex != event.work.triggeredEvents ||
        event.work.triggeredEvents <= event.work.completedEvents ||
        event.work.triggeredEvents - event.work.completedEvents != 1ULL ||
        (selection == 2U && event.modeEvents[1U] != 0U) ||
        !EDM128::ValidHeightProof(event, height, recipe.deepFlushCycleInterval,
            EDM130::FrozenDeepMultiplier(recipe), EDM130::HasBoundDeep(recipe)) ||
        !EDM128::ValidSpeedProof(event, recipe.flushStages)) return;
    EDM157::Scope scope{};
    if (!CaptureEDM157ScopeSameThread(scope) || !EDM157::PauseModeAllowed(scope, event) ||
        !EDM130::SameRecipe(scope.recipe, mode.timerRecipe) || !EDM116::SamePolicy(scope.policy, mode.rowPolicy) ||
        !EDM126::SameSource(scope.authorizedSource, event.anchor.authorizedSource)) return;
    EDM126::MotionFeedback history{};
    if (targeted)
    {
        if (!m_motion.ReadEDM126Feedback(history) || !history.enabled || !history.active || history.faulted ||
            !history.sourceReady || !history.rowActive || history.rowCompleted ||
            history.fixture != mode.fixture || history.session != mode.session || history.armEpoch != mode.armEpoch ||
            history.ownerGeneration != mode.ownerGeneration || history.liveEpoch != mode.liveEpoch ||
            history.authorizedRows != mode.authorizedRows || history.completedRows != mode.completedRows ||
            history.tick != mode.tick || history.nowUs != mode.nowUs ||
            history.lastMappingTick != history.tick || history.lastMappingUs != history.nowUs ||
            mode.lastMappingTick != history.lastMappingTick || mode.lastMappingUs != history.lastMappingUs ||
            history.traversal.tick != history.tick || history.traversal.nowUs != history.nowUs ||
            !EDM126::SameSource(history.authorizedSource, scope.authorizedSource) ||
            !EDM157::ValidMapped(scope, history.traversal)) return;
        // L3 pauses only a committed B2 excursion that already crossed into
        // an older retained row. Earlier non-crossing B2 events do not consume
        // this one-shot checkpoint; no future target or anchor substitutes.
        if (selection == 3U && (!EDM157::ValidMapped(scope, event.anchor) ||
            history.traversal.cursorSource.ordinal >= event.anchor.cursorSource.ordinal)) return;
    }
    const bool accepted = TryHoldEDM157SameThread();
    if (accepted) m_edm165AutoHoldSession = s.session;
    char record[481]{};
    const char* trigger = selection == 1U ? "FIRST_E5_OUTBOUND_25PCT" : selection == 2U ?
        "FIRST_B1_E5_OUTBOUND_25PCT" : "FIRST_CROSSED_B2_E5_OUTBOUND_25PCT";
    const int length = std::snprintf(record, sizeof(record),
        "[%s] event=AUTO_HOLD_REQUEST result=%s session=%llu L=%u B=%u eventIndex=%llu trigger=%s phase=OUTBOUND tick=%llu hold=%llu startFloor=%llu cursorRow=%llu anchorRow=%llu resume=MANUAL_FRESH_START S92=0 S93=0\n",
        targeted ? "EDM168" : "EDM165", accepted ? "QUEUED" : "REJECTED", static_cast<unsigned long long>(s.session),
        selection, static_cast<unsigned>(event.policy.mode), static_cast<unsigned long long>(event.eventIndex), trigger,
        static_cast<unsigned long long>(mode.tick), static_cast<unsigned long long>(m_edm157.holdSequence),
        static_cast<unsigned long long>(m_edm157.startFloor),
        static_cast<unsigned long long>(targeted ? history.traversal.cursorSource.ordinal : event.anchor.cursorSource.ordinal),
        static_cast<unsigned long long>(event.anchor.cursorSource.ordinal));
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM165] event=LOG_FORMAT_ERROR part=AUTO_HOLD_REQUEST\n");
    if (!accepted) RejectEDM94SameThread("EDM165_AUTO_HOLD_REQUEST_REJECTED", m_gapDryRun.sourceLine);
}

bool NCManager::IsEDM168AutoHoldCheckpointCompleteSameThread() const noexcept
{
    const auto& s = m_edm94; const auto& recipe = m_edm130.recipe;
    if (recipe.flushHoldTest < 2U) return true;
    const unsigned target = EDM130::FlushHoldTargetMode(recipe.flushHoldTest);
    return m_edm130.bound && EDM130::ValidRecipe(recipe) &&
        EDM130::ValidFlushHoldSelection(s.gapHistoryArm, recipe.flushHoldTest) &&
        s.gapHistoryArm == 274U && recipe.diagnosticControlSource == 3U && !recipe.commissioningProfile &&
        s.gapModeFixture == EDM116::Fixture::ModalProgram156 && E156BoundPlanCurrent(s, recipe) &&
        recipe.holdRetreatMode == 1U && target < 3U && s.session &&
        m_edm165AutoHoldSession == s.session && m_edm168AutoHoldResumedSession == s.session &&
        s.next == s.moves + (s.finishPending ? 1U : 0U) && m_edm116.acknowledgedRows == s.moves &&
        m_edm130.retainedModeEvents[target] != 0U;
}

bool NCManager::ServiceEDM157SameThread(bool feedbackCurrent)
{
    auto& p = m_edm157;
    if (!p.active) return false;
    if (!IsEDM157ScopeCurrentSameThread(p.scope) || GetActiveDispatchPC() != p.pc ||
        ((p.resumePending || p.startIntentPending) && (AlarmManager::GetInstance().GetUpdateCount() != p.alarmUpdate ||
            AlarmManager::MotionAdmissionBaseState(AlarmManager::GetInstance().GetMotionSafetyIntentState()) != p.alarmIntent ||
            m_motion.GetSafetyProvenanceGeneration() != p.safetyProvenance)))
    { LogEDM157SameThread("CANCEL", "PAUSE_SCOPE_CHANGED"); CancelEDM94SameThread("EDM157_PAUSE_SCOPE_CHANGED"); return false; }
    if (feedbackCurrent && (m_edm94.feedback.faulted || m_edm94.feedback.cancelled || m_edm94.feedback.forceZero ||
        !m_edm94.feedback.active || !m_edm94.feedback.armAccepted))
    { LogEDM157SameThread("CANCEL", "GUARD_REVOKED"); CancelEDM94SameThread("EDM157_GUARD_REVOKED"); return false; }
    EDM157::Feedback observed{};
    if (!m_motion.ReadEDM157Feedback(observed) || observed.holdSequence < p.holdSequence)
    {
        if (feedbackCurrent && m_edm94.feedback.lastTick > p.requestTick &&
            m_edm94.feedback.lastTick - p.requestTick > 4000ULL)
        { LogEDM157SameThread("CANCEL", "HOLD_ACK_TIMEOUT"); CancelEDM94SameThread("EDM157_HOLD_ACK_TIMEOUT"); return false; }
        return true;
    }
    if (observed.holdSequence != p.holdSequence || !EDM157::SameScope(observed.scope, p.scope) ||
        !EDM157::ValidPauseContext(observed) || (p.feedback.phase != EDM157::Phase::Idle &&
            !EDM157::SamePauseContext(observed, p.feedback)) ||
        observed.startFloor != p.startFloor || !observed.tick || !observed.nowUs || observed.tick < p.lastTick ||
        (p.startIntentPending&&observed.nowUs<p.feedback.nowUs)||
        (feedbackCurrent && m_edm94.feedback.lastTick > observed.tick &&
            m_edm94.feedback.lastTick - observed.tick >
                (observed.phase == EDM157::Phase::Running ? 4000ULL : 8ULL)) ||
        observed.phase == EDM157::Phase::Rejected || observed.phase == EDM157::Phase::Idle)
    { LogEDM157SameThread("CANCEL", "RT_PAUSE_IDENTITY_OR_PHASE"); CancelEDM94SameThread("EDM157_RT_PAUSE_REJECTED"); return false; }
    if(EDM130::IsG76B2FlushHold200(p.scope.recipe)&&m_edm200.queued&&!m_edm200.resumed&&
        (!observed.flushPause||observed.flushPhase!=EDM128::Phase::Outbound||
            observed.flushEventIndex!=m_edm200.eventIndex||m_edm200.eventIndex!=1ULL||
            m_edm200.holdSequence!=p.holdSequence||m_edm200.compoundCommit!=p.compoundCommit||
            !EDM157::SameScope(m_edm200.scope,p.scope)))
    {LogEDM157SameThread("CANCEL","AUTO_CHECKPOINT_CHANGED");CancelEDM94SameThread("EDM200_AUTO_CHECKPOINT_CHANGED");return false;}
    if(EDM130::IsG76CenterHold217(p.scope.recipe)&&m_edm200.queued&&!m_edm200.resumed&&
        (!E217NominalCheckpointReceipt(observed)||m_edm200.eventIndex||
            m_edm200.holdSequence!=p.holdSequence||m_edm200.compoundCommit!=p.compoundCommit||
            !EDM157::SameScope(m_edm200.scope,p.scope)))
    {LogEDM157SameThread("CANCEL","AUTO_CHECKPOINT_CHANGED");CancelEDM94SameThread("EDM217_AUTO_CHECKPOINT_CHANGED");return false;}
    if(EDM130::IsG76CenterAnyFlushHold232(p.scope.recipe)&&m_edm200.queued&&!m_edm200.resumed&&
        (!E232CenterFlushCheckpointReceipt(observed)||
            m_edm200.eventIndex!=EDM130::G76CenterHoldEventIndex(p.scope.recipe)||
            m_edm200.holdSequence!=p.holdSequence||m_edm200.compoundCommit!=p.compoundCommit||
            !EDM157::SameScope(m_edm200.scope,p.scope)))
    {LogEDM157SameThread("CANCEL","AUTO_CHECKPOINT_CHANGED");
        CancelEDM94SameThread(EDM130::IsG76CenterSecondReturnHold232(p.scope.recipe)?"EDM232_AUTO_CHECKPOINT_CHANGED":EDM130::IsG76CenterSecondFlushHold231(p.scope.recipe)?"EDM231_AUTO_CHECKPOINT_CHANGED":EDM130::IsG76CenterBrakeHold228(p.scope.recipe)?"EDM228_AUTO_CHECKPOINT_CHANGED":EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)?"EDM227_AUTO_CHECKPOINT_CHANGED":EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)?"EDM226_AUTO_CHECKPOINT_CHANGED":EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)?"EDM225_AUTO_CHECKPOINT_CHANGED":
            EDM130::IsG76CenterDwellHold224(p.scope.recipe)?"EDM224_AUTO_CHECKPOINT_CHANGED":
            EDM130::IsG76CenterTopDwellHold222(p.scope.recipe)?"EDM223_AUTO_CHECKPOINT_CHANGED":
            EDM130::IsG76CenterStartHold221(p.scope.recipe)?"EDM221_AUTO_CHECKPOINT_CHANGED":
            EDM130::IsG76CenterAnchorHold220(p.scope.recipe)?"EDM220_AUTO_CHECKPOINT_CHANGED":
            EDM130::IsG76CenterReturnHold219(p.scope.recipe)?"EDM219_AUTO_CHECKPOINT_CHANGED":"EDM218_AUTO_CHECKPOINT_CHANGED");return false;}
    // RUNNING is the one retained completion receipt of the requested START.
    // RT stops publishing it after the final zero proof; the 10 ms NC scan
    // can arrive after the normal eight-tick live-proof freshness window.
    // Require a current, healthy guard at or after that completion, keeping
    // the existing one-second ACK bound and every identity/START check below.
    // HELD admission and all in-progress physical proofs still use eight ticks.
    if (observed.phase == EDM157::Phase::Running &&
        (!feedbackCurrent || m_edm94.feedback.lastTick < observed.tick)) return true;
    // TryResume retained the actual Held receipt before queuing START. The
    // Running acknowledgement must retain its stopped pose. SamePauseContext
    // above also requires the same frozen nominal-work or flush receipt.
    if(EDM157::IsG76CenterNominalPause(p.scope)&&
        (p.startIntentPending||p.feedback.phase==EDM157::Phase::Held||p.feedback.phase==EDM157::Phase::Resuming)&&
        (!E217SameNominalPose(observed,p.feedback)||
            !EDM157::SameNominalWorkReceipt(observed.nominalWork,p.feedback.nominalWork)))
    {LogEDM157SameThread("CANCEL",observed.flushPause?"FLUSH_RECEIPT_CHANGED":"NOMINAL_RECEIPT_CHANGED");
        CancelEDM94SameThread(observed.flushPause?
            (EDM130::IsG76CenterSecondReturnHold232(p.scope.recipe)?"EDM232_FLUSH_RECEIPT_CHANGED":EDM130::IsG76CenterSecondFlushHold231(p.scope.recipe)?"EDM231_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::Resume?"EDM229_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::Brake?"EDM228_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::AnchorDwell?"EDM227_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::ReturnDwell?"EDM226_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::CenterReturnDwell?"EDM225_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::CenterDwell?"EDM224_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::TopDwell?"EDM223_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::ToCenter?"EDM221_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::ToAnchor?"EDM220_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::Returning?"EDM219_FLUSH_RECEIPT_CHANGED":"EDM218_FLUSH_RECEIPT_CHANGED"):
            "EDM217_NOMINAL_RECEIPT_CHANGED");return false;}
    if(observed.phase==EDM157::Phase::Running&&EDM157::IsG76CenterNominalPause(p.scope)&&
        ((p.feedback.phase!=EDM157::Phase::Held&&p.feedback.phase!=EDM157::Phase::Resuming)||
            p.feedback.stableCycles<EDM157::RequiredStableCycles||observed.stableCycles<EDM157::RequiredStableCycles||
            !E217SameNominalPose(observed,p.feedback)||
            !EDM157::SameNominalWorkReceipt(observed.nominalWork,p.feedback.nominalWork)))
    {LogEDM157SameThread("CANCEL",observed.flushPause?"FLUSH_RECEIPT_CHANGED":"NOMINAL_RECEIPT_CHANGED");
        CancelEDM94SameThread(observed.flushPause?
            (EDM130::IsG76CenterSecondReturnHold232(p.scope.recipe)?"EDM232_FLUSH_RECEIPT_CHANGED":EDM130::IsG76CenterSecondFlushHold231(p.scope.recipe)?"EDM231_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::Resume?"EDM229_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::Brake?"EDM228_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::AnchorDwell?"EDM227_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::ReturnDwell?"EDM226_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::CenterReturnDwell?"EDM225_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::CenterDwell?"EDM224_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::TopDwell?"EDM223_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::ToCenter?"EDM221_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::ToAnchor?"EDM220_FLUSH_RECEIPT_CHANGED":
                observed.flushPhase==EDM128::Phase::Returning?"EDM219_FLUSH_RECEIPT_CHANGED":"EDM218_FLUSH_RECEIPT_CHANGED"):
            "EDM217_NOMINAL_RECEIPT_CHANGED");return false;}
    const auto priorPhase = p.feedback.phase;
    p.feedback = observed; p.lastTick = observed.tick;
    if ((p.scope.recipe.holdRetreatMode || observed.flushPause) && priorPhase != observed.phase)
        LogEDM157SameThread(observed.flushPause ? "FLUSH_PHASE" : "PATH_PHASE", "EXISTING_CNC_COMMITTED_PATH");
    const bool ready = observed.phase == EDM157::Phase::Held &&
        observed.stableCycles >= EDM157::RequiredStableCycles;
    if (ready && !p.readyLogged) LogEDM157SameThread("HOLD_READY",
        observed.flushPause ? "FLUSH_STOPPED_ZERO_200_FRESH_START_REQUIRED" :
        !p.scope.recipe.holdRetreatMode ? "PHYSICAL_ZERO_200_FRESH_START_REQUIRED" :
        observed.retreatUnavailable ? "RETREAT_UNAVAILABLE_FRESH_START_REQUIRED" :
        observed.interrupted ? "INTERRUPTED_STOP_FRESH_START_TO_ANCHOR" : "RETRACTED_ZERO_200_FRESH_START_REQUIRED");
    p.readyLogged = ready;
    if (p.resumePending && observed.rejectedStartSequence == p.resumeStartSequence)
    {
        p.resumePending = false; p.resumeStartSequence = 0ULL;
        LogEDM157SameThread("START_BLOCKED", "RT_PROOF_CHANGED_FRESH_START_REQUIRED");
    }
    if (observed.phase == EDM157::Phase::Running)
    {
        if (!p.resumePending || observed.resumeStartSequence != p.resumeStartSequence ||
            observed.resumeStartSequence <= p.startFloor || m_state != NCState::HOLD ||
            ((p.scope.recipe.holdRetreatMode || observed.flushPause) &&
                (!observed.anchorCaptured || (!observed.flushPause && !observed.exactReturn) ||
                observed.stableCycles < EDM157::RequiredStableCycles)))
        { LogEDM157SameThread("CANCEL", "UNREQUESTED_RESUME_ACK"); CancelEDM94SameThread("EDM157_UNREQUESTED_RESUME_ACK"); return false; }
        if(EDM157::IsG76NominalPause(p.scope)&&observed.flushPause&&
            !EDM157::ValidG76FlushReceipt(observed))
        {LogEDM157SameThread("CANCEL","FLUSH_RECEIPT_CHANGED");CancelEDM94SameThread("EDM200_FLUSH_RECEIPT_CHANGED");return false;}
        if(EDM130::IsG76B2FlushHold200(p.scope.recipe)&&m_edm200.queued&&!m_edm200.resumed)
        {
            const auto& c=m_edm200;
            if(!observed.flushPause||observed.flushPhase!=EDM128::Phase::Outbound||
                observed.flushEventIndex!=c.eventIndex||c.eventIndex!=1ULL||
                c.holdSequence!=p.holdSequence||c.compoundCommit!=p.compoundCommit||
                !EDM157::SameScope(c.scope,p.scope)||
                observed.cursorSource.ordinal>=observed.flush.anchorOrdinal)
            {LogEDM157SameThread("CANCEL","AUTO_CHECKPOINT_CHANGED");CancelEDM94SameThread("EDM200_AUTO_CHECKPOINT_CHANGED");return false;}
            m_edm200.resumed=true;m_edm200.resumeStartSequence=p.resumeStartSequence;
            RtPrintf("[EDM200] event=CHECKPOINT_RESUMED session=%llu L=3 B=2 eventIndex=%llu hold=%llu start=%llu proof=VALIDATED_RT_RUNNING discharge=0\n",
                static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(observed.flushEventIndex),
                static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(p.resumeStartSequence));
        }
        if(EDM130::IsG76CenterHold217(p.scope.recipe)&&m_edm200.queued&&!m_edm200.resumed)
        {
            // Every scope, phase, pose, work and requested START check above
            // has passed. A queued or Held request alone earns no credit.
            m_edm200.resumed=true;m_edm200.resumeStartSequence=p.resumeStartSequence;
            PrintNCG76DiagnosticLine("[EDM217] event=CHECKPOINT_RESUMED session=%llu L=1 B=%u eventIndex=0 hold=%llu start=%llu samePose=1 workFrozen=1 proof=VALIDATED_RT_RUNNING discharge=0\n",
                static_cast<unsigned long long>(m_edm94.session),EDM173::ModeForArm(m_edm94.gapHistoryArm),
                static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(p.resumeStartSequence));
        }
        if(EDM130::IsG76CenterAnyFlushHold232(p.scope.recipe)&&m_edm200.queued&&!m_edm200.resumed)
        {
            m_edm200.resumed=true;m_edm200.resumeStartSequence=p.resumeStartSequence;
            PrintNCG76DiagnosticLine("[%s] event=CHECKPOINT_RESUMED session=%llu L=%u B=%u eventIndex=%llu hold=%llu start=%llu samePose=1 eventFrozen=1 proof=VALIDATED_RT_RUNNING discharge=0\n",
                EDM130::IsG76CenterSecondReturnHold232(p.scope.recipe)?"EDM232":EDM130::IsG76CenterSecondFlushHold231(p.scope.recipe)?"EDM231":EDM130::IsG76CenterBrakeHold228(p.scope.recipe)?"EDM228":EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)?"EDM227":EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)?"EDM226":EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)?"EDM225":EDM130::IsG76CenterDwellHold224(p.scope.recipe)?"EDM224":EDM130::IsG76CenterTopDwellHold222(p.scope.recipe)?"EDM223":EDM130::IsG76CenterStartHold221(p.scope.recipe)?"EDM221":EDM130::IsG76CenterAnchorHold220(p.scope.recipe)?"EDM220":EDM130::IsG76CenterReturnHold219(p.scope.recipe)?"EDM219":"EDM218",
                static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned>(p.scope.recipe.flushHoldTest),
                EDM173::ModeForArm(m_edm94.gapHistoryArm),static_cast<unsigned long long>(observed.flushEventIndex),
                static_cast<unsigned long long>(p.holdSequence),static_cast<unsigned long long>(p.resumeStartSequence));
        }
        m_state = NCState::RUN; m_pauseAfterBlock = false;
        ResumePathCoreLiveRetentionSameThread();
        LogEDM157SameThread("RESUMED", observed.flushPause ? "RT_ACK_FLUSH_SAME_POSE_RESUME_E5" : p.scope.recipe.holdRetreatMode ?
            "RT_ACK_EXACT_ANCHOR_AND_ZERO_200" : "RT_ACK_SAME_CURSOR_SCOPE");
        const unsigned selection = p.scope.recipe.flushHoldTest;
        const unsigned target = EDM130::FlushHoldTargetMode(selection);
        if (selection >= 2U && observed.flushPause && target < 3U &&
            EDM130::ValidFlushHoldSelection(m_edm94.gapHistoryArm, selection) &&
            m_edm165AutoHoldSession == m_edm94.session &&
            static_cast<unsigned>(p.scope.policy.mode) == target)
        {
            m_edm168AutoHoldResumedSession = m_edm94.session;
            RtPrintf("[EDM168] event=CHECKPOINT_RESUMED session=%llu L=%u B=%u eventIndex=%llu hold=%llu start=%llu proof=VALIDATED_RT_RUNNING S92=0 S93=0\n",
                static_cast<unsigned long long>(m_edm94.session), selection, target,
                static_cast<unsigned long long>(observed.flushEventIndex),
                static_cast<unsigned long long>(p.holdSequence), static_cast<unsigned long long>(p.resumeStartSequence));
        }
        p = EDM157NCState{};
        return false;
    }
    ServiceEDM223StartIntentSameThread(feedbackCurrent);
    return p.active;
}

bool NCManager::TryRetainEDM223StartSameThread() noexcept
{
    auto& p=m_edm157;
    // L6-L11 can also receive an ordinary manual HOLD before/after their automatic
    // checkpoint. Those pauses keep the existing immediate-START path.
    if((!EDM130::IsG76CenterTopDwellHold222(p.scope.recipe)&&!EDM130::IsG76CenterDwellHold224(p.scope.recipe)&&!EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)&&!EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)&&!EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)&&!EDM130::IsG76CenterBrakeHold228(p.scope.recipe))||
        (!p.startIntentPending&&(!m_edm200.queued||m_edm200.resumed)))return false;
    // Only CycleStart calls this capture path. Service may spend this exact
    // operator sequence, but cannot invent one or extend its one-second life.
    if(p.startIntentPending) {
        LogEDM157SameThread("START_PENDING","ORIGINAL_START_RETAINED_DEADLINE_UNCHANGED");return true;
    }
    auto& alarms=AlarmManager::GetInstance();
    const auto update=alarms.GetUpdateCount();
    const auto intent=AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState());
    const auto safety=m_motion.GetSafetyProvenanceGeneration();
    EDM157::Feedback observed{};EDM94::Feedback guard{};
    if(!m_motion.ReadEDM157Feedback(observed)||!m_motion.ReadEDM94Feedback(guard)||
        !EDM157::SameScope(observed.scope,p.scope)||!EDM157::ValidPauseContext(observed)||
        !E228CenterFlushCheckpointReceipt(observed)||
        (observed.phase!=EDM157::Phase::Qualifying&&observed.phase!=EDM157::Phase::Held)||
        observed.holdSequence!=p.holdSequence||observed.startFloor!=p.startFloor||
        observed.tick<p.lastTick||!E223StartGuardCurrent(guard,observed,true)||
        (p.feedback.phase!=EDM157::Phase::Idle&&
            (!EDM157::SamePauseContext(observed,p.feedback)||!E217SameNominalPose(observed,p.feedback)||
                !EDM157::SameNominalWorkReceipt(observed.nominalWork,p.feedback.nominalWork)))||
        !m_edm200.queued||m_edm200.resumed||m_edm200.eventIndex!=1ULL||
        m_edm200.holdSequence!=p.holdSequence||m_edm200.compoundCommit!=p.compoundCommit||
        !EDM157::SameScope(m_edm200.scope,p.scope)||GetActiveDispatchPC()!=p.pc||
        m_startDiagnosticRequests<=p.startFloor||m_startDiagnosticRequests<=p.startIntentSequence||
        m_startDiagnosticRequests<=observed.rejectedStartSequence||
        !IsEDM157ScopeCurrentSameThread(p.scope)||m_state!=NCState::HOLD||alarms.HasAlarm()||
        update!=alarms.GetUpdateCount()||
        intent!=AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState())||
        safety!=m_motion.GetSafetyProvenanceGeneration())
    {LogEDM157SameThread("START_BLOCKED",EDM130::IsG76CenterBrakeHold228(p.scope.recipe)?"CURRENT_BRAKE_STOP_OR_NEW_START_REQUIRED":EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)?"CURRENT_INITIAL_DWELL_OR_NEW_START_REQUIRED":EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)?"CURRENT_FINAL_DWELL_OR_NEW_START_REQUIRED":EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)?"CURRENT_CENTER_RETURN_DWELL_OR_NEW_START_REQUIRED":EDM130::IsG76CenterDwellHold224(p.scope.recipe)?"CURRENT_CENTER_DWELL_OR_NEW_START_REQUIRED":"CURRENT_TOP_DWELL_OR_NEW_START_REQUIRED");return true;}
    p.alarmUpdate=update;p.alarmIntent=intent;p.safetyProvenance=safety;
    p.startIntentSequence=m_startDiagnosticRequests;p.startIntentTick=observed.tick;p.startIntentUs=observed.nowUs;
    p.startIntentPending=true;p.feedback=observed;p.lastTick=observed.tick;
    PrintNCG76DiagnosticLine("[%s] event=START_INTENT_RETAINED session=%llu hold=%llu start=%llu phase=%s stable=%u waitLimitUs=1000000 resume=ON_CURRENT_ZERO_200 discharge=0\n",
        E228CenterDwellIntentTag(p.scope.recipe),static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(p.holdSequence),
        static_cast<unsigned long long>(p.startIntentSequence),EDM157::PhaseName(observed.phase),observed.stableCycles);
    ServiceEDM223StartIntentSameThread(true);
    return true;
}

void NCManager::ServiceEDM223StartIntentSameThread(bool feedbackCurrent) noexcept
{
    auto& p=m_edm157;
    if(!p.active||!p.startIntentPending)return;
    const auto& observed=p.feedback;auto& alarms=AlarmManager::GetInstance();
    if(p.resumePending||m_state!=NCState::HOLD||!IsEDM157ScopeCurrentSameThread(p.scope)||
        GetActiveDispatchPC()!=p.pc||(!EDM130::IsG76CenterTopDwellHold222(p.scope.recipe)&&!EDM130::IsG76CenterDwellHold224(p.scope.recipe)&&!EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)&&!EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)&&!EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)&&!EDM130::IsG76CenterBrakeHold228(p.scope.recipe))||
        !EDM157::SameScope(observed.scope,p.scope)||!EDM157::ValidPauseContext(observed)||
        !E228CenterFlushCheckpointReceipt(observed)||
        (observed.phase!=EDM157::Phase::Qualifying&&observed.phase!=EDM157::Phase::Held)||
        observed.holdSequence!=p.holdSequence||observed.startFloor!=p.startFloor||
        !m_edm200.queued||m_edm200.resumed||m_edm200.eventIndex!=1ULL||
        m_edm200.holdSequence!=p.holdSequence||m_edm200.compoundCommit!=p.compoundCommit||
        !EDM157::SameScope(m_edm200.scope,p.scope)||
        !p.startIntentSequence||p.startIntentSequence<=p.startFloor||
        p.startIntentSequence<=observed.rejectedStartSequence||p.startIntentSequence>m_startDiagnosticRequests||
        !p.startIntentTick||!p.startIntentUs||observed.tick<p.startIntentTick||observed.nowUs<p.startIntentUs||
        alarms.GetUpdateCount()!=p.alarmUpdate||alarms.HasAlarm()||
        AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState())!=p.alarmIntent||
        m_motion.GetSafetyProvenanceGeneration()!=p.safetyProvenance)
    {LogEDM157SameThread("CANCEL","RETAINED_START_CONTEXT_CHANGED");CancelEDM94SameThread(EDM130::IsG76CenterBrakeHold228(p.scope.recipe)?"EDM228_START_CONTEXT_CHANGED":EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)?"EDM227_START_CONTEXT_CHANGED":EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)?"EDM226_START_CONTEXT_CHANGED":EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)?"EDM225_START_CONTEXT_CHANGED":EDM130::IsG76CenterDwellHold224(p.scope.recipe)?"EDM224_START_CONTEXT_CHANGED":"EDM223_START_CONTEXT_CHANGED");return;}
    // Time and tick bounds both fail closed; repeated START cannot refresh
    // either. Keep the consumed sequence until a new HOLD resets the state.
    if(observed.nowUs-p.startIntentUs>=1000000ULL||observed.tick-p.startIntentTick>=4000ULL) {
        p.startIntentPending=false;
        PrintNCG76DiagnosticLine("[%s] event=START_INTENT_EXPIRED session=%llu hold=%llu start=%llu waitedUs=%llu action=REMAIN_HOLD_NEW_START_REQUIRED discharge=0\n",
            E228CenterDwellIntentTag(p.scope.recipe),static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(p.holdSequence),
            static_cast<unsigned long long>(p.startIntentSequence),static_cast<unsigned long long>(observed.nowUs-p.startIntentUs));
        return;
    }
    if(!feedbackCurrent)return;
    EDM94::Feedback guard{};
    if(!m_motion.ReadEDM94Feedback(guard)||!E223StartGuardCurrent(guard,observed,true)) {
        p.startIntentPending=false;LogEDM157SameThread("START_BLOCKED","RETAINED_START_GUARD_CHANGED_NEW_START_REQUIRED");return;
    }
    // A newer guard can cross the deadline while the pause publication is
    // still within the eight-tick freshness window. Neither clock may extend it.
    if(guard.lastNowUs>=p.startIntentUs&&guard.lastTick>=p.startIntentTick&&
        (guard.lastNowUs-p.startIntentUs>=1000000ULL||guard.lastTick-p.startIntentTick>=4000ULL)) {
        p.startIntentPending=false;
        PrintNCG76DiagnosticLine("[%s] event=START_INTENT_EXPIRED session=%llu hold=%llu start=%llu waitedUs=%llu action=REMAIN_HOLD_NEW_START_REQUIRED discharge=0\n",
            E228CenterDwellIntentTag(p.scope.recipe),static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(p.holdSequence),
            static_cast<unsigned long long>(p.startIntentSequence),static_cast<unsigned long long>(guard.lastNowUs-p.startIntentUs));
        return;
    }
    if(!E223StartGuardCurrent(guard,observed))return;
    if(observed.phase!=EDM157::Phase::Held||observed.stableCycles<EDM157::RequiredStableCycles)return;
    const std::uint64_t sequence=p.startIntentSequence;
    AlarmManager::MotionAdmissionReservation admission{};
    const auto result=alarms.TryBeginMotionAdmission(p.alarmUpdate,p.alarmIntent,admission,true);
    if(result!=AlarmManager::MotionAdmissionResult::ACQUIRED) {
        p.startIntentPending=false;LogEDM157SameThread("START_BLOCKED","RETAINED_START_ADMISSION_DENIED_NEW_START_REQUIRED");return;
    }
    const bool current=m_state==NCState::HOLD&&IsEDM157ScopeCurrentSameThread(p.scope)&&
        GetActiveDispatchPC()==p.pc&&!alarms.HasAlarm()&&alarms.GetUpdateCount()==p.alarmUpdate&&
        AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState())==p.alarmIntent&&
        m_motion.GetSafetyProvenanceGeneration()==p.safetyProvenance;
    const bool submitted=current&&m_motion.RequestEDM157Resume(p.scope,p.holdSequence,sequence);
    const bool ended=alarms.EndMotionAdmission(admission);
    // Once attempted, this ticket is spent. In particular, a queued request
    // or a later RT rejection must never be retried automatically.
    p.startIntentPending=false;
    if(!ended) {CancelEDM94SameThread(EDM130::IsG76CenterBrakeHold228(p.scope.recipe)?"EDM228_RESUME_ALARM_ADMISSION_REVOKED":EDM130::IsG76CenterInitialDwellHold227(p.scope.recipe)?"EDM227_RESUME_ALARM_ADMISSION_REVOKED":EDM130::IsG76CenterFinalDwellHold226(p.scope.recipe)?"EDM226_RESUME_ALARM_ADMISSION_REVOKED":EDM130::IsG76CenterReturnDwellHold225(p.scope.recipe)?"EDM225_RESUME_ALARM_ADMISSION_REVOKED":EDM130::IsG76CenterDwellHold224(p.scope.recipe)?"EDM224_RESUME_ALARM_ADMISSION_REVOKED":"EDM223_RESUME_ALARM_ADMISSION_REVOKED");return;}
    if(!submitted) {LogEDM157SameThread("START_BLOCKED","RESUME_NOT_QUEUED_FRESH_START_REQUIRED");return;}
    p.resumePending=true;p.resumeStartSequence=sequence;
    PrintNCG76DiagnosticLine("[%s] event=START_INTENT_SUBMITTED session=%llu hold=%llu start=%llu waitedUs=%llu stable=%u proof=CURRENT_HELD_ZERO_200 discharge=0\n",
        E228CenterDwellIntentTag(p.scope.recipe),static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(p.holdSequence),
        static_cast<unsigned long long>(sequence),static_cast<unsigned long long>(observed.nowUs-p.startIntentUs),observed.stableCycles);
    LogEDM157SameThread("RESUME_REQUEST","RETAINED_FRESH_START_CONTINUE_SAME_E5_EVENT");
}

bool NCManager::TryResumeEDM157SameThread() noexcept
{
    auto& p = m_edm157;
    if (!p.active) return false;
    if (m_state != NCState::HOLD || !IsEDM157ScopeCurrentSameThread(p.scope))
    { CancelEDM94SameThread("EDM157_START_SCOPE_CHANGED"); return true; }
    if (p.resumePending)
    { LogEDM157SameThread("START_BLOCKED", "RESUME_ACK_PENDING"); return true; }
    if(TryRetainEDM223StartSameThread())return true;
    EDM157::Feedback observed{};
    if (!m_motion.ReadEDM157Feedback(observed) || !EDM157::SameScope(observed.scope, p.scope) ||
        !EDM157::ValidPauseContext(observed) || (p.feedback.phase != EDM157::Phase::Idle &&
            !EDM157::SamePauseContext(observed, p.feedback)) ||
        observed.holdSequence != p.holdSequence || observed.startFloor != p.startFloor ||
        observed.phase != EDM157::Phase::Held || observed.stableCycles < EDM157::RequiredStableCycles ||
        !observed.tick || observed.tick < p.lastTick || !observed.nowUs ||
        (m_edm94.feedback.lastTick > observed.tick && m_edm94.feedback.lastTick - observed.tick > 8ULL) ||
        m_startDiagnosticRequests <= p.startFloor)
    { LogEDM157SameThread("START_BLOCKED", "WAIT_FRESH_PHYSICAL_ZERO_200"); return true; }
    auto& alarms = AlarmManager::GetInstance();
    const auto update = alarms.GetUpdateCount();
    const auto intent = AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState());
    const auto safety = m_motion.GetSafetyProvenanceGeneration();
    AlarmManager::MotionAdmissionReservation admission{};
    const auto result = alarms.TryBeginMotionAdmission(update, intent, admission, true);
    if (result != AlarmManager::MotionAdmissionResult::ACQUIRED)
    { LogEDM157SameThread("START_BLOCKED", "ALARM_ADMISSION_NOT_ACQUIRED"); return true; }
    const bool current = IsEDM157ScopeCurrentSameThread(p.scope) && GetActiveDispatchPC() == p.pc &&
        safety == m_motion.GetSafetyProvenanceGeneration() && !alarms.HasAlarm();
    const bool submitted = current && m_motion.RequestEDM157Resume(p.scope, p.holdSequence, m_startDiagnosticRequests);
    const bool ended = alarms.EndMotionAdmission(admission);
    if (!ended)
    { CancelEDM94SameThread("EDM157_RESUME_ALARM_ADMISSION_REVOKED"); return true; }
    if (!submitted)
    { LogEDM157SameThread("START_BLOCKED", "RESUME_NOT_QUEUED_FRESH_START_REQUIRED"); return true; }
    p.resumePending = true; p.resumeStartSequence = m_startDiagnosticRequests;
    p.alarmUpdate = update; p.alarmIntent = intent; p.safetyProvenance = safety;
    p.feedback = observed; p.lastTick = observed.tick;
    LogEDM157SameThread("RESUME_REQUEST", observed.flushPause ? "FRESH_START_CONTINUE_SAME_E5_EVENT" : p.scope.recipe.holdRetreatMode ?
        "FRESH_START_RETURN_ORIGINAL_ANCHOR" : "FRESH_START_AFTER_PHYSICAL_ZERO_200");
    return true;
}

void NCManager::CancelEDM94SameThread(const char* reason) noexcept
{
    m_edm237 = EDM237NCState{};
    m_edm234 = EDM234NCState{};
    m_edm233 = EDM233NCState{};
    m_edm157 = EDM157NCState{};
    m_edm149Confirmation = EDM149Confirmation{};
    if(m_edm173) { m_edm173->coordinateRow.valid=false; m_edm173->coordinateScope=EDM173::Scope{}; }
    // Capture the independent excursion cause before cancellation clears its NC receipt.
    if (m_edm94.gapModeFlush && !m_edm94.cancelled && !m_edm116.failureLogged &&
        m_motion.ReadEDM116Feedback(m_edm116.feedback) && m_edm116.feedback.excursion.faulted &&
        m_edm116.feedback.session == m_edm94.session && m_edm116.feedback.armEpoch == m_edm94.armEpoch &&
        m_edm116.feedback.ownerGeneration == m_edm94.lease.generation)
    { m_edm116.failureLogged = true; LogEDM116SameThread("MODE_FLUSH_FAIL"); }
    if (m_edm94.gapCurveB2 && !m_edm94.cancelled && !m_edm115.failureLogged &&
        m_motion.ReadEDM115Feedback(m_edm115.feedback) && m_edm115.feedback.excursion.faulted &&
        m_edm115.feedback.session == m_edm94.session && m_edm115.feedback.armEpoch == m_edm94.armEpoch &&
        m_edm115.feedback.ownerGeneration == m_edm94.lease.generation)
    { m_edm115.failureLogged = true; LogEDM115SameThread("CURVE_B2_FAIL"); }
    // Capture the independent mapper cause before normal cancellation clears its NC receipt.
    if (m_edm94.gapPhysicalCurve && !m_edm94.cancelled && !m_edm114.failureLogged &&
        m_motion.ReadEDM114Feedback(m_edm114.feedback) && m_edm114.feedback.failed &&
        m_edm114.feedback.session == m_edm94.session && m_edm114.feedback.armEpoch == m_edm94.armEpoch &&
        m_edm114.feedback.ownerGeneration == m_edm94.lease.generation)
    { m_edm114.failureLogged = true; LogEDM114SameThread("PHYSICAL_CURVE_FAIL"); }
    InvalidateEDM109CommonPathSameThread(reason ? reason : "CANCEL");
    m_edm94.boundary = EDM94NCState::FeedBoundary{};
    m_edm94.gapWaiting = m_edm94.gapPermitReady = false;
    // RT may have completed normal retirement immediately before HOLD/RESET.
    // Retire that exact receipt even though NC has not sampled it yet; the
    // caller still performs its full ordinary HOLD/RESET transaction.
    if (m_edm94.latched && m_edm94.completionReleaseRequested)
    {
        EDM94::Feedback observed{};
        if (m_motion.ReadEDM94Feedback(observed) && ConsumeEDM94CompletionReleaseSameThread(observed)) return;
    }
    m_edm94.normalEndCommitted = false;
    if (!m_edm94.latched || m_edm94.releaseRequested) return;
    m_motion.RequestEDM94Cancel(m_edm94.session, m_edm94.resetSeen);
    if (m_edm94.cancelled) return;
    m_edm94.cancelled = true; m_edm94.ready = false; m_edm94.complete = false;
    LogEDM94SameThread("CANCEL_REQUEST", reason ? reason : "UNKNOWN");
}

void NCManager::RejectEDM94SameThread(const char* reason, int line)
{
    CancelEDM94SameThread(reason);
    LogEDM94SameThread("REJECT", reason);
    AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, line);
    ChangeState(NCState::ALARM);
}

bool NCManager::GuardEDM94BlockSameThread(const NCBlock& block, int pc, int line)
{
    if(m_edm237.authorized)
    {
        const auto& q=m_edm237;const auto& state=m_edm94;
        if(!IsEDM237SequenceScopeCurrentSameThread())
        {RejectEDM94SameThread("EDM237_PARENT_SCOPE",line);return false;}
        if(!state.latched&&q.awaitNext)
        {
            const auto* source=GetBaseProgramCache().TryGetLine(pc);
            const bool bridge=E237BridgeRequired(q.program,q.parentIndex,q.kinds[0U])&&source&&E240RecipeBridgeShape(source->parsedBlock,q.kinds[q.parentIndex],q.program)&&
                E94Matches(source->parsedBlock,block)&&IsEDM240RecipeBridgeCurrentSameThread(pc);
            const bool valid=bridge||(source&&IsEDM234BracketShapeValid(block)&&E94Matches(source->parsedBlock,block)&&
                IsEDM237NextArmCurrentSameThread(pc,static_cast<unsigned>(block.val('P'))));
            if(!valid)RejectEDM94SameThread("EDM237_NEXT_ARM_SCOPE",line);
            return valid;
        }
        if(state.latched&&(!q.stageRecipeBound||!q.stageStarted||q.awaitNext||state.session!=q.currentSession||
            state.armPC!=q.armPC[q.parentIndex]||state.gapHistoryArm!=E237ArmForKind(q.kinds[q.parentIndex])||
            state.moves!=E237RowsForKind(q.kinds[q.parentIndex])||
            (state.next&&q.currentRowBound+1U<state.next)||
            (state.complete&&(q.parentIndex+1U!=q.parentCount||!q.firstReleased||q.releasePending))))
        {RejectEDM94SameThread("EDM237_PARENT_BLOCK_SCOPE",line);return false;}
    }
    if(m_edm234.authorized&&m_edm234.program==236U)
    {
        const auto& q=m_edm234;const auto& state=m_edm94;
        if(!IsEDM236SequenceScopeCurrentSameThread())
        {RejectEDM94SameThread("EDM236_PARENT_SCOPE",line);return false;}
        if(!state.latched&&q.awaitNext)
        {
            const auto* source=GetBaseProgramCache().TryGetLine(pc);
            const bool valid=source&&IsEDM234BracketShapeValid(block)&&E94Matches(source->parsedBlock,block)&&
                IsEDM236NextArmCurrentSameThread(pc,static_cast<unsigned>(block.val('P')));
            if(!valid)RejectEDM94SameThread("EDM236_NEXT_ARM_SCOPE",line);
            return valid;
        }
        if(state.latched&&(!q.stageRecipeBound||!q.stageStarted||q.awaitNext||state.session!=q.currentSession||
            state.armPC!=q.firstArmPC+static_cast<int>(q.parentIndex*3U)||
            state.gapHistoryArm!=E236ArmForKind(q.kinds[q.parentIndex])||(state.next==1U&&!q.motionDispatch)||
            (state.complete&&(q.parentIndex!=3U||q.retiredMask!=7U||q.releasePending))))
        {RejectEDM94SameThread("EDM236_PARENT_BLOCK_SCOPE",line);return false;}
    }
    else if(m_edm234.authorized)
    {
        const auto& q=m_edm234;const auto& state=m_edm94;
        if(!state.latched&&q.firstReleased&&!q.secondStarted)
        {
            const auto* source=GetBaseProgramCache().TryGetLine(pc);
            const bool valid=source&&IsEDM234BracketShapeValid(block)&&E94Matches(source->parsedBlock,block)&&
                IsEDM234NextArmCurrentSameThread(pc,static_cast<unsigned>(block.val('P')));
            if(!valid)RejectEDM94SameThread("EDM234_NEXT_ARM_SCOPE",line);
            return valid;
        }
        if(state.latched&&(!IsEDM234SequenceScopeCurrentSameThread()||
            state.armPC!=q.firstArmPC+(q.secondStarted?3:0)||state.gapHistoryArm!=q.arms[q.secondStarted?1U:0U]||
            (state.next==1U&&!q.motionDispatch)||
            (state.complete&&(!q.firstReleased||!q.secondStarted||q.releasePending||state.session<=q.firstSession))))
        {RejectEDM94SameThread("EDM234_PARENT_BLOCK_SCOPE",line);return false;}
    }
    if (m_edm94.latched && EDM173::IsArm(m_edm94.gapHistoryArm)) return GuardEDM173BlockSameThread(block,pc,line);
    if(m_edm233.authorized&&m_edm233.firstReleased&&!m_edm233.secondStarted)
    {
        const auto* source=GetBaseProgramCache().TryGetLine(pc);
        const bool valid=source&&IsEDM173BracketShapeValid(block)&&
            E233CenterParentSequence(source->parsedBlock,static_cast<unsigned>(block.val('P')))&&
            E94Matches(source->parsedBlock,block)&&
            IsEDM233SecondArmCurrentSameThread(pc,static_cast<unsigned>(block.val('P')));
        if(!valid)RejectEDM94SameThread("EDM233_SECOND_ARM_ORDER_OR_SCOPE",line);
        return valid;
    }
    if (!InspectEDM94ProgramSameThread() && !m_edm94.latched) return true;
    const auto* authored = GetBaseProgramCache().TryGetLine(pc);
    if (!authored || !(E94Matches(authored->parsedBlock, block) ||
        MatchesEDM141MotionSameThread(authored->parsedBlock, block, pc)))
    { RejectEDM94SameThread("LITERAL_OR_CACHE", line); return false; }
    if (block.isEmpty) return true;
    if (!m_edm94.latched)
    {
        // The shared E94 preamble remains G91-only. Permit only the new
        // coordinate setup rows before this program's unique EDM173 M00/arm.
        unsigned coordinateArm=0U;int coordinateArmPC=-1;
        const auto coordinateSetupAllowed=[&]() noexcept {
            if(!E178CoordinatePreamble(authored->parsedBlock)||!E94Matches(authored->parsedBlock,block)||
                ((EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0],43U)||
                    EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0],44U))&&
                    !E182ToolLengthSourceMatches(authored->parsedBlock,block))||
                (EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0],168U)&&
                    !E183WorkSourceMatches(authored->parsedBlock,block))||
                ((EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0],150U)||
                    EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0],151U))&&
                    !E185MirrorSourceMatches(authored->parsedBlock,block))||
                (EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0],51U)&&
                    !E186ScaleSourceMatches(authored->parsedBlock,block))||
                (E190WorkCoordinateSelection(authored->parsedBlock)&&
                    (!E190WorkCoordinateSourceMatches(authored->parsedBlock,block)||
                        !m_motion.IsGroupDone()||!m_motion.IsGroupNCDrained()||
                        m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||
                        m_motion.GetAxisCommandMailboxDepth()))||
                pc<0||pc!=GetActiveDispatchPC()||authored->sourceLineNumber!=line||
                m_mode!=NCOperationMode::MEMORY||m_state!=NCState::RUN||!m_pathCoreLiveBookkeeping.currentRunToken||
                !m_macroStack.empty()||m_isG66Active||m_edm149Confirmation.waiting||m_edm149Confirmation.confirmed) return false;
            const auto& cache=GetBaseProgramCache();const auto generation=cache.GetGeneration();
            if(!generation||generation!=m_edm94.inspectedCache||cache.Size()>128U) return false;
            int sequenceFirst=-1;std::array<unsigned,2U> sequenceArms{};std::array<unsigned,4U> chainKinds{};
            if(E237ProgramShape(cache,sequenceFirst,sequenceArms))
            {coordinateArm=E237ArmForKind(sequenceArms[0U]);coordinateArmPC=sequenceFirst;return pc<sequenceFirst-1&&generation==cache.GetGeneration();}
            if(E236ProgramShape(cache,sequenceFirst,chainKinds))
            {coordinateArm=E236ArmForKind(chainKinds[0U]);coordinateArmPC=sequenceFirst;return pc<sequenceFirst-1&&generation==cache.GetGeneration();}
            if(E234ProgramShape(cache,sequenceFirst,sequenceArms))
            {coordinateArm=sequenceArms[0U];coordinateArmPC=sequenceFirst;return pc<sequenceFirst-1&&generation==cache.GetGeneration();}
            if(E233ProgramShape(cache,sequenceFirst,sequenceArms,&NCManager::IsEDM173BracketShapeValid,
                &NCManager::IsEDM173OrbitBlockShapeValid))
            {
                coordinateArm=sequenceArms[0U];coordinateArmPC=sequenceFirst;
                return pc<sequenceFirst-1&&generation==cache.GetGeneration();
            }
            unsigned brackets=0U,confirmations=0U,coordinateExpressions=0U;
            int confirmationPC=-1,coordinateExpressionPC=-1;
            NCBlock candidate{}; EDM179::Specification coordinateSource{};
            for(std::size_t index=0U;index<cache.Size();++index)
            {
                const auto* source=cache.TryGetLine(static_cast<int>(index));
                if(!source) return false;
                const auto& parsed=source->parsedBlock;
                if(!EDM77::Detail::Clean(parsed))
                {
                    if(!EDM179::Parse(parsed,coordinateSource,candidate)||!IsEDM173OrbitBlockShapeValid(candidate)||
                        ++coordinateExpressions>1U) return false;
                    coordinateExpressionPC=static_cast<int>(index);
                }
                if(E149M00Shape(parsed)){++confirmations;confirmationPC=static_cast<int>(index);}
                for(int g=0;g<parsed.gCount;++g)
                {
                    double code=0.0;if(!E94Number(parsed.gExpressions[g],code)) return false;
                    if(code!=180.0) continue;
                    if(++brackets>2U||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed,candidate)||
                        !IsEDM173BracketShapeValid(candidate)) return false;
                    const unsigned selector=static_cast<unsigned>(candidate.val('P'));
                    if(EDM173::IsArm(selector))
                    {
                        if(coordinateArm!=0U) return false;
                        coordinateArm=selector;coordinateArmPC=static_cast<int>(index);
                    }
                }
            }
            if(brackets!=2U||coordinateArm==0U||confirmations!=1U||
                confirmationPC!=coordinateArmPC-1||pc>=confirmationPC||
                (coordinateExpressions!=0U&&coordinateExpressionPC!=coordinateArmPC+1)||
                static_cast<std::size_t>(coordinateArmPC)+3U>=cache.Size()) return false;
            NCBlock orbit{},finish{},end{};
            const auto* armLine=cache.TryGetLine(coordinateArmPC);
            const auto* orbitLine=cache.TryGetLine(coordinateArmPC+1);
            const auto* finishLine=cache.TryGetLine(coordinateArmPC+2);
            const auto* endLine=cache.TryGetLine(coordinateArmPC+3);
            if(!armLine||!orbitLine||!finishLine||!endLine||
                (!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(orbitLine->parsedBlock,orbit)&&
                    !EDM179::Parse(orbitLine->parsedBlock,coordinateSource,orbit))||
                !IsEDM173OrbitBlockShapeValid(orbit)||E181EffectiveOrbitMode(orbit)!=EDM173::ModeForArm(coordinateArm)||
                !E251SquareSourcePair(armLine->parsedBlock,orbit)||
                ((orbit.has('K')||!orbit.has('B')||orbit.val('B')==0.0)&&!orbitLine->parsedBlock.dependsOnMacroState&&
                    !E180LiteralAngleSourceMatches(orbitLine->parsedBlock,orbit))||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(finishLine->parsedBlock,finish)||
                !IsEDM173BracketShapeValid(finish)||finish.val('P')!=static_cast<double>(EDM173::FinishForArm(coordinateArm))||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(endLine->parsedBlock,end)||
                end.gCount!=0||end.mCount!=1||end.mCode[0]!=30||end.isGoto||end.isBlockSkip) return false;
            for(char word='A';word<='Z';++word)
                if(end.has(word)&&(word!='N'||!std::isfinite(end.val(word)))) return false;
            for(std::size_t index=static_cast<std::size_t>(coordinateArmPC)+4U;index<cache.Size();++index)
            {
                const auto* source=cache.TryGetLine(static_cast<int>(index));
                if(!source||!source->parsedBlock.isEmpty) return false;
            }
            return generation==cache.GetGeneration();
        };
        if(coordinateSetupAllowed())
        {
            RtPrintf("[EDM178_FIX1] event=PREARM_COORDINATE_ACCEPTED pc=%d line=%d G=%d arm=P%u armPC=%d cache=%llu authority=SETUP_ONLY motion=0 discharge=0\n",
                pc,line,block.gCode,coordinateArm,coordinateArmPC,
                static_cast<unsigned long long>(GetBaseProgramCache().GetGeneration()));
            return true;
        }
        EDM97::Tuning tuning{};
        EDM107::FlushProfile parameters{};
        if (IsEDM149ConfirmationBlockSameThread(authored->parsedBlock, pc) || IsEDM130RecipePreambleSameThread(authored->parsedBlock, pc) || E94Preamble(authored->parsedBlock) || E94Profile(authored->parsedBlock, 85U) ||
            E94Profile(authored->parsedBlock, 87U) || E94Profile(authored->parsedBlock, 89U) ||
            E126Profile(authored->parsedBlock, 158U, tuning) || E126Profile(authored->parsedBlock, 160U, tuning) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 162U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 164U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 166U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 168U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 170U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 172U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 174U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 176U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 178U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 180U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 182U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 184U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 186U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 188U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 190U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 192U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 194U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 196U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 198U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 200U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 202U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 204U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 206U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 208U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 210U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 212U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 214U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 216U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 218U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 220U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 222U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 224U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 226U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 228U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 230U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 232U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 234U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 236U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 238U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 240U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 242U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 244U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 246U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 248U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 250U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 252U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 254U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 256U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 258U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 260U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 262U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 264U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 266U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 268U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 270U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 272U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 274U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 276U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 278U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 280U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 282U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 284U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 286U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 288U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 290U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 292U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 294U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 296U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 298U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 300U, tuning, parameters) ||
            E97ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E98ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E99ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E100ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E102ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E104ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E105ProfileOnlyNPQRS(authored->parsedBlock, tuning) ||
            E106ProfileOnlyNPQRS(authored->parsedBlock, 105U, tuning) ||
            E106ProfileOnlyNPQRS(authored->parsedBlock, 107U, tuning) ||
            E107ProfileOnlyNPQRSIJK(authored->parsedBlock, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 111U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 113U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 115U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 117U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 119U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 121U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 123U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 125U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 127U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 129U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 132U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 134U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 136U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 138U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 140U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, EDM122::B0ArmP, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, EDM122::B1ArmP, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, EDM123::B0ArmP, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, EDM123::B1ArmP, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 150U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, 152U, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, EDM125::RetreatArmP, tuning, parameters) ||
            E108ProfileOnlyNPQRSIJK(authored->parsedBlock, EDM125::ReturnArmP, tuning, parameters)) return true;
        RejectEDM94SameThread("ARM_REQUIRED_START_TOP", line); return false;
    }
    if (m_edm94.gapModeInterruption && (m_edm94.gapModeInterruptionArm != m_edm118.armP ||
        m_edm94.next > (EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? EDM125::TargetStep : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? EDM123::TargetStep(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? EDM122::TargetStep(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) : EDM118::TargetStep)))
    {
        RequestEDM118HoldSameThread(false, "NEXT_BLOCK_FORBIDDEN_TARGET_MISSED");
        return false;
    }
    if (EDM116::IsModalFixture(m_edm94.gapModeFixture) && m_edm94.next < m_edm94.moves)
    {
        EDM116::Mode mode{};
        if (E127Standalone(authored->parsedBlock, mode))
        {
            if (pc != GetActiveDispatchPC() || !IsEDM127SelectionBoundarySameThread(pc))
            { RejectEDM94SameThread("EDM127_B_SCOPE", line); return false; }
            return true;
        }
        if (pc != m_edm94.pc[m_edm94.next] || !PrepareEDM127RowSameThread(block))
        { RejectEDM94SameThread("EDM127_MODAL_ROW", line); return false; }
    }
    if (m_edm94.gapHistoryMode && m_edm94.next >= m_edm94.moves &&
        (m_edm116.pending || m_edm116.selected != m_edm94.moves || m_edm116.consumed != m_edm94.moves ||
            m_edm116.acknowledgedRows != m_edm94.moves))
    { RejectEDM94SameThread("EDM128_ORPHAN_MODAL_ROW", line); return false; }
    if (m_edm94.gapModeFlush && block.has('P') && block.val('P') == 131.0)
    {
        if (pc != GetActiveDispatchPC() || !IsEDM116SelectorCurrentSameThread(block))
        { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_SELECTOR_SCOPE" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_SELECTOR_SCOPE" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_SELECTOR_SCOPE" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_SELECTOR_SCOPE" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_SELECTOR_SCOPE" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_SELECTOR_SCOPE" : m_edm94.gapModeInterruption ? "EDM118_SELECTOR_SCOPE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_SELECTOR_SCOPE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_SELECTOR_SCOPE" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_SELECTOR_SCOPE" : "EDM116_SELECTOR_SCOPE"), line); return false; }
        return true; // Setting blocks never advance the physical row cursor.
    }
    if (m_edm94.gapModeFlush && m_edm94.next < m_edm94.moves &&
        (!m_edm116.pending || m_edm116.selected != m_edm94.next + 1U || m_edm116.consumed != m_edm94.next))
    { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_SELECTOR_REQUIRED" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_SELECTOR_REQUIRED" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_SELECTOR_REQUIRED" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_SELECTOR_REQUIRED" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_SELECTOR_REQUIRED" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_SELECTOR_REQUIRED" : m_edm94.gapModeInterruption ? "EDM118_SELECTOR_REQUIRED" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_SELECTOR_REQUIRED" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_SELECTOR_REQUIRED" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_SELECTOR_REQUIRED" : "EDM116_SELECTOR_REQUIRED"), line); return false; }
    if (m_edm94.gapModeFlush && m_edm94.next >= m_edm94.moves &&
        (m_edm116.pending || m_edm116.selected != m_edm94.moves || m_edm116.consumed != m_edm94.moves))
    { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_ORPHAN_SELECTOR" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_ORPHAN_SELECTOR" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_ORPHAN_SELECTOR" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_ORPHAN_SELECTOR" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_ORPHAN_SELECTOR" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_ORPHAN_SELECTOR" : m_edm94.gapModeInterruption ? "EDM118_ORPHAN_SELECTOR" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_ORPHAN_SELECTOR" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_ORPHAN_SELECTOR" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_ORPHAN_SELECTOR" : "EDM116_ORPHAN_SELECTOR"), line); return false; }
    if (m_edm94.gapModeFlush && m_edm94.next == 0U &&
        (!m_edm116.initialTranslationValid ||
            !SameNCTranslationSnapshot(m_edm116.initialTranslation, CoordSys.GetTranslationSnapshot())))
    { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_INITIAL_FRAME_CHANGED" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_INITIAL_FRAME_CHANGED" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_INITIAL_FRAME_CHANGED" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_INITIAL_FRAME_CHANGED" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_INITIAL_FRAME_CHANGED" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_INITIAL_FRAME_CHANGED" : m_edm94.gapModeInterruption ? "EDM118_INITIAL_FRAME_CHANGED" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_INITIAL_FRAME_CHANGED" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_INITIAL_FRAME_CHANGED" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_INITIAL_FRAME_CHANGED" : "EDM116_INITIAL_FRAME_CHANGED"), line); return false; }
    if (m_edm94.gapHistoryServo && m_edm94.next == 0U &&
        (!m_edm126.initialBoundaryReady || !m_edm126.initialTranslationValid ||
            !SameNCTranslationSnapshot(m_edm126.initialTranslation, CoordSys.GetTranslationSnapshot())))
    { RejectEDM94SameThread("EDM126_INITIAL_FRAME_OR_BOUNDARY", line); return false; }
    if (m_edm94.cancelled || m_edm94.armPending || m_edm94.releaseRequested ||
        m_edm94.cache != GetBaseProgramCache().GetGeneration() ||
        m_edm94.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_edm94.next >= m_edm94.moves + 2U || pc != m_edm94.pc[m_edm94.next] ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_edm94.lease) ||
        (m_edm94.next < m_edm94.moves && !m_edm94.ready) ||
        (m_edm94.next == m_edm94.moves + 1U && !m_edm94.complete))
    { RejectEDM94SameThread("BLOCK_SCOPE_OR_ORDER", line); return false; }
    if (E141MotionArm(m_edm94.gapHistoryArm) && m_edm94.next < m_edm94.moves)
    {
        if (m_edm141.verifiedRows != m_edm94.next ||
            !MatchesEDM141MotionSameThread(authored->parsedBlock, block, pc))
        { RejectEDM94SameThread("EDM141_RESOLVED_ROW_ORDER", line); return false; }
        ++m_edm141.verifiedRows;
        const auto& row = m_edm141.rows[m_edm94.next];
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM141] event=MOTION_RESOLVED session=%llu row=%u pc=%d G=%d readMask=%u assignments=%u local1=%.9g local2=%.9g B=%u powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.next, pc, row.gcode,
            row.readMask, row.assignmentCount, row.localValues[0], row.localValues[1],
            static_cast<unsigned>(m_edm127.pending.mode));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM141] event=LOG_FORMAT_ERROR part=MOTION_RESOLVED\n");
    }
    m_edm94.boundary = EDM94NCState::FeedBoundary{};
    ++m_edm94.next;
    return true;
}

// EDM154 classifies only the already admitted current literal row. It grants
// no Motion permission; GuardEDM94FeedSameThread still consumes its exact token.
bool NCManager::IsEDM154SparseFeedSameThread() const noexcept
{
    const auto& s = m_edm94;
    if (!s.latched || !s.ready || !s.everArmed || s.armPending || s.cancelled || s.resetSeen ||
        s.resetCommitted || s.releaseRequested || s.rtFaultReported || s.finishPending || s.complete ||
        !s.gapHistoryServo || !s.gapHistoryMode || !s.gapWorkTime ||
        !((s.gapModeFixture == EDM116::Fixture::ModalSparse154 && EDM149::IsSparseTestArm(s.gapHistoryArm) &&
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
                E149HistoryFixture(s.gapHistoryArm) == EDM149::XYZVisibleFlushHistoryFixture) ||
            (s.gapModeFixture == EDM116::Fixture::ModalVisibleFlush158 && EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) &&
                E149HistoryFixture(s.gapHistoryArm) == EDM149::VisibleFlushHistoryFixture &&
                m_edm127.authoredInline[0] && m_edm127.expectedMode[0] == EDM116::Mode::B1)) ||
        !m_edm130.bound || !EDM130::IsMotionTest(m_edm130.recipe) || !EDM130::ValidRecipe(m_edm130.recipe) ||
        m_edm130.recipe.diagnosticControlSource != EDM149::SourceForArm(s.gapHistoryArm) ||
        s.gapPhysicalAd != EDM130::RequiresPhysicalVoltage(m_edm130.recipe) ||
        !E156BoundPlanCurrent(s, m_edm130.recipe) || s.next == 0U || s.next > s.moves ||
        s.cache != GetBaseProgramCache().GetGeneration() ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || !s.session || !s.run || !s.cache ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || !s.lease.Matches(m_programMotionLease)) return false;
    const unsigned step = s.next - 1U;
    return GetActiveDispatchPC() == s.pc[step] && s.authoredGCode[step] == 1U &&
        EDM149::ValidSparseIncrement(s.increment[step], s.feed[step]);
}

bool NCManager::IsEDM154SparseFeedBlockSameThread(const NCBlock& block) const noexcept
{
    const auto& source = CoordSys.GetTranslationSnapshot();
    if (!IsEDM154SparseFeedSameThread() || source.distanceMode != 91 || !IsNCXYZCUVFeedNeutralFrame(source) ||
        !IsNCXYZFeedSingleGBlock(block, 1) || !block.has('F')) return false;
    const unsigned step = m_edm94.next - 1U;
    const auto* authored = GetBaseProgramCache().TryGetLine(m_edm94.pc[step]);
    if (!authored) return false;
    NCParsedBlock geometry = authored->parsedBlock;
    if (geometry.has('B') != m_edm127.authoredInline[step]) return false;
    if (geometry.has('B'))
    {
        double value = 0.0; EDM116::Mode mode = EDM116::Mode::Invalid;
        if (!E94Number(geometry.expression('B'), value) || !EDM116::ParseMode(value, mode) ||
            mode != m_edm127.expectedMode[step]) return false;
        geometry.hasParam['B'-'A'] = false; geometry.paramExpressions['B'-'A'].clear();
    }
    std::array<double, 6U> literal{}; double feed = 0.0;
    if (!E114Move(geometry, step, literal, feed, m_edm94.gapModeFixture) ||
        literal != m_edm94.increment[step] || feed != m_edm94.feed[step] || block.val('F') != feed) return false;
    for (char letter = 'A'; letter <= 'Z'; ++letter)
        if (block.has(letter) && (std::strchr("NXYZCUVF", letter) == nullptr || !std::isfinite(block.val(letter)))) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const char letter = "XYZCUV"[axis];
        if (block.has(letter) != geometry.has(letter) ||
            (block.has(letter) ? block.val(letter) != literal[axis] : literal[axis] != 0.0)) return false;
    }
    return true;
}

bool NCManager::GuardEDM94FeedSameThread(const NCBlock& block) noexcept
{
    if (!m_edm94.latched) return !m_edm94.program;
    if (m_edm94.gapCurveGeometry && !IsEDM112GeometryCurrentSameThread(false)) return false;
    if (!m_edm94.ready || m_edm94.cancelled || m_edm94.finishPending || m_edm94.next == 0U ||
        m_edm94.next > m_edm94.moves || m_edm94.cache != GetBaseProgramCache().GetGeneration() ||
        m_edm94.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_edm94.lease)) return false;
    const unsigned step = m_edm94.next - 1U;
    if (m_edm94.gapPhysicalCurve && (EDM116::IsArcStep(step, m_edm94.gapModeFixture, m_edm94.authoredGCode[step]) ||
        (EDM116::IsAuthoredOrderFixture(m_edm94.gapModeFixture) && block.gCode != m_edm94.authoredGCode[step]) ||
        !IsEDM114CurrentSameThread(step))) return false;
    if (m_edm94.gapCurveB2 && !IsEDM115CurrentSameThread(step)) return false;
    if (m_edm94.gapCurveRuntime && !IsEDM113RuntimeCurrentSameThread(step)) return false;
    const int pc = GetActiveDispatchPC();
    if (pc != m_edm94.pc[step] || m_currentExecutingBlockDispatchId == 0ULL ||
        block.val('F') != m_edm94.feed[step]) return false;
    std::array<double, 6U> target{};
    for (unsigned a = 0U; a < 6U; ++a)
    {
        const bool present = block.has("XYZCUV"[a]);
        if ((!present && (!EDM149::IsTestArm(m_edm94.gapHistoryArm) || m_edm94.increment[step][a] != 0.0)) ||
            (present && block.val("XYZCUV"[a]) != m_edm94.increment[step][a])) return false;
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(a));
        target[a] = axis.logicalCmdPos.Load() + m_edm94.increment[step][a] * m_edm94.feedback.config.axis[a].pulsePerUnit;
    }
    const MotionExecutionEpoch epoch = m_motion.GetCurrentExecutionEpoch();
    if (!m_motion.CheckEDM94Endpoint(target, epoch, m_edm94.lease)) return false;
    if (m_edm94.gapSegmentProfile)
    {
        if (m_isSingleBlockEnabled || m_edm94.gapWaiting) return false;
        if (m_edm94.gapGainProfile)
        {
            if (!(m_edm94.gapNoiseProfile ? EDM98::ValidTuning(m_edm94.gapTuning) :
                EDM97::ValidTuning(m_edm94.gapTuning))) return false;
            EDM96::Feedback gap{};
            if (!m_motion.ReadEDM96Feedback(gap) || gap.session != m_edm94.session ||
                gap.epoch != epoch || gap.ownerGeneration != m_edm94.lease.generation ||
                gap.expectedStep != step || !gap.active || gap.faulted ||
                gap.reason != EDM96::Reason::None || !IsEDM97TuningCurrentSameThread(gap)) return false;
            if (m_edm94.gapHistoryMode) return AuthorizeEDM128SameThread(block, 1, std::array<double, 2U>{});
            if (m_edm94.gapHistoryServo)
                return IsEDM126CurrentSameThread(step) && m_motion.AuthorizeEDM126Feed(m_edm94.session,
                    m_edm94.run, m_edm94.cache, m_currentExecutingBlockDispatchId, pc, step, 1,
                    m_edm94.increment[step], std::array<double, 2U>{}, m_edm94.feed[step],
                    epoch, m_edm94.lease, m_edm94.gapTuning, m_edm114.retained.source,
                    m_edm114.completionCommit, m_edm114.completionTick);
            if (m_edm94.gapNoiseProfile)
                return m_motion.AuthorizeEDM98Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                    m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                    m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning);
            if (m_edm94.gapFlushProfile)
            {
                if (m_edm94.gapModeFlush)
                    return AuthorizeEDM116SameThread(block, 1, std::array<double, 2U>{});
                if (m_edm94.gapCurveB2)
                {
                    if (!BuildEDM115CompletionTokenSameThread(step)) return false;
                    return m_motion.AuthorizeEDM115Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                        m_currentExecutingBlockDispatchId, pc, step, 1, m_edm94.increment[step],
                        std::array<double, 2U>{}, m_edm94.feed[step], epoch, m_edm94.lease,
                        m_edm94.gapTuning, m_edm94.gapFlushParameters, m_edm115.token);
                }
                if (m_edm94.gapPhysicalCurve)
                    return m_motion.AuthorizeEDM114Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                        m_currentExecutingBlockDispatchId, pc, step, 1, m_edm94.increment[step],
                        std::array<double, 2U>{}, m_edm94.feed[step], epoch, m_edm94.lease,
                        m_edm94.gapTuning, m_edm94.gapFlushParameters);
                if (!BuildEDM110CompletionTokenSameThread(step)) return false;
                return m_motion.AuthorizeEDM105Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                    m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                    m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning, m_edm94.gapFlushParameters,
                    m_edm94.gapCommonPath, m_edm94.gapB2CrossSegment, m_edm110.token, m_edm94.gapB2Repeat, m_edm94.gapCurveRuntime);
            }
            if (m_edm94.gapPairedProfile)
                return m_motion.AuthorizeEDM104Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                    m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                    m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning);
            if (m_edm94.gapEnduranceProfile)
                return m_motion.AuthorizeEDM102Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                    m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                    m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning);
            if (m_edm94.gapContinuousProfile)
                return m_motion.AuthorizeEDM100Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                    m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                    m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning);
            if (m_edm94.gapBurstProfile)
                return m_motion.AuthorizeEDM99Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                    m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                    m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning);
            return m_motion.AuthorizeEDM97Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
                m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
                m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapTuning);
        }
        return m_motion.AuthorizeEDM96Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
            m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
            m_edm94.feed[step], epoch, m_edm94.lease);
    }
    if (m_edm94.gapBoundaryProfile)
    {
        if (m_isSingleBlockEnabled || !m_edm94.gapPermitReady ||
            m_edm94.gapWaitStep != step) return false;
        m_edm94.gapPermitReady = false; // The attempted authored block owns this copied offer.
        return m_motion.AuthorizeEDM95Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
            m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
            m_edm94.feed[step], epoch, m_edm94.lease, m_edm94.gapFeedback.token);
    }
    // One-use authority for this already inspected literal G1. The producer
    // validates its final geometry before publishing the exact epoch handoff.
    return m_motion.AuthorizeEDM94Feed(m_edm94.session, m_edm94.run, m_edm94.cache,
        m_currentExecutingBlockDispatchId, pc, step, m_edm94.increment[step],
        m_edm94.feed[step], epoch, m_edm94.lease);
}

WaitConditionFunc NCManager::StartEDM94SameThread(const NCBlock& block)
{
    if (block.val('P')==292.0 || block.val('P')==293.0 || block.val('P')==294.0 || block.val('P')==295.0 || block.val('P')==296.0 || block.val('P')==297.0 || block.val('P')==298.0 || block.val('P')==299.0 || block.val('P')==300.0 || block.val('P')==301.0) return StartEDM173SameThread(block);
    const int pc = GetActiveDispatchPC();
    if (block.val('P') == EDM125::RetreatFinishP || block.val('P') == EDM125::ReturnFinishP)
    {
        const unsigned arm = block.val('P') == EDM125::RetreatFinishP ? EDM125::RetreatArmP : EDM125::ReturnArmP;
        if (m_edm94.gapModeInterruption && m_edm94.latched && m_edm94.gapModeInterruptionArm == arm)
            RequestEDM118HoldSameThread(false, arm == EDM125::RetreatArmP ? "P155_TARGET_MISSED" : "P157_TARGET_MISSED");
        else RejectEDM94SameThread("EDM125_FINISH_SCOPE", m_gapDryRun.sourceLine);
        return nullptr;
    }
    if (block.val('P') == 135.0 || block.val('P') == 137.0 || block.val('P') == 139.0 || block.val('P') == 141.0 || block.val('P') == 143.0 || block.val('P') == 145.0 || block.val('P') == 147.0 || block.val('P') == 149.0)
    {
        const bool b0Return = block.val('P') == 147.0;
        const bool b1Return = block.val('P') == 149.0;
        const bool b0Local = block.val('P') == 143.0;
        const bool b1Local = block.val('P') == 145.0;
        const bool previousReturn = block.val('P') == 141.0;
        const bool arcRetreat = block.val('P') == 139.0;
        const bool retreat = block.val('P') == 137.0;
        if (m_edm94.gapModeInterruption && m_edm94.latched &&
            m_edm94.gapModeInterruptionArm == (b0Return ? EDM123::B0ArmP : b1Return ? EDM123::B1ArmP : b0Local ? EDM122::B0ArmP : b1Local ? EDM122::B1ArmP : previousReturn ? EDM121::ArmP : arcRetreat ? EDM120::ArmP : retreat ? EDM119::ArmP : EDM118::ArmP))
            RequestEDM118HoldSameThread(false, b0Return ? "P147_TARGET_MISSED" : b1Return ? "P149_TARGET_MISSED" : b0Local ? "P143_TARGET_MISSED" : b1Local ? "P145_TARGET_MISSED" : previousReturn ? "P141_TARGET_MISSED" : arcRetreat ? "P139_TARGET_MISSED" : retreat ? "P137_TARGET_MISSED" : "P135_TARGET_MISSED");
        else RejectEDM94SameThread(b0Return || b1Return ? "EDM123_FINISH_SCOPE" : b0Local || b1Local ? "EDM122_FINISH_SCOPE" : previousReturn ? "EDM121_FINISH_SCOPE" : arcRetreat ? "EDM120_FINISH_SCOPE" : retreat ? "EDM119_FINISH_SCOPE" : "EDM118_FINISH_SCOPE", m_gapDryRun.sourceLine);
        return nullptr;
    }
    if (block.val('P') == 131.0)
    {
        if (!CommitEDM116ModeSameThread(block)) RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_SELECTOR_COMMIT" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_SELECTOR_COMMIT" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_SELECTOR_COMMIT" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_SELECTOR_COMMIT" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_SELECTOR_COMMIT" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_SELECTOR_COMMIT" : m_edm94.gapModeInterruption ? "EDM118_SELECTOR_COMMIT" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_SELECTOR_COMMIT" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_SELECTOR_COMMIT" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_SELECTOR_COMMIT" : "EDM116_SELECTOR_COMMIT"), m_gapDryRun.sourceLine);
        return nullptr;
    }
    if (block.val('P') == 86.0 || block.val('P') == 88.0 || block.val('P') == 90.0 ||
        block.val('P') == 92.0 || block.val('P') == 94.0 || block.val('P') == 96.0 || block.val('P') == 98.0 || block.val('P') == 100.0 || block.val('P') == 102.0 || block.val('P') == 104.0 || block.val('P') == 106.0 || block.val('P') == 108.0 || block.val('P') == 110.0 || block.val('P') == 112.0 || block.val('P') == 114.0 || block.val('P') == 116.0 || block.val('P') == 118.0 || block.val('P') == 120.0 || block.val('P') == 122.0 || block.val('P') == 124.0 || block.val('P') == 126.0 || block.val('P') == 128.0 || block.val('P') == 130.0 || block.val('P') == 133.0 || block.val('P') == 151.0 || block.val('P') == 153.0 || block.val('P') == 159.0 || block.val('P') == 161.0 || block.val('P') == 163.0 || block.val('P') == 165.0 || block.val('P') == 167.0 || block.val('P') == 169.0 || block.val('P') == 171.0 || block.val('P') == 173.0 || block.val('P') == 175.0 || block.val('P') == 177.0 || block.val('P') == 179.0 || block.val('P') == 181.0 || block.val('P') == 183.0 || block.val('P') == 185.0 || block.val('P') == 187.0 || block.val('P') == 189.0 || block.val('P') == 191.0 || block.val('P') == 193.0 || block.val('P') == 195.0 || block.val('P') == 197.0 || block.val('P') == 199.0 || block.val('P') == 201.0 || block.val('P') == 203.0 || block.val('P') == 205.0 || block.val('P') == 207.0 || block.val('P') == 209.0 || block.val('P') == 211.0 || block.val('P') == 213.0 || block.val('P') == 215.0 || block.val('P') == 217.0 || block.val('P') == 219.0 || block.val('P') == 221.0 || block.val('P') == 223.0 || block.val('P') == 225.0 || block.val('P') == 227.0 || block.val('P') == 229.0 || block.val('P') == 231.0 || block.val('P') == 233.0 || block.val('P') == 235.0 || block.val('P') == 237.0 || block.val('P') == 239.0 || block.val('P') == 241.0 || block.val('P') == 243.0 || block.val('P') == 245.0 || block.val('P') == 247.0 || block.val('P') == 249.0 || block.val('P') == 251.0 || block.val('P') == 253.0 || block.val('P') == 255.0 || block.val('P') == 257.0 || block.val('P') == 259.0 || block.val('P') == 261.0 || block.val('P') == 263.0 || block.val('P') == 265.0 || block.val('P') == 267.0 || block.val('P') == 269.0 || block.val('P') == 271.0 || block.val('P') == 273.0 || block.val('P') == 275.0 || block.val('P') == 277.0 || block.val('P') == 279.0 || block.val('P') == 281.0 || block.val('P') == 283.0 || block.val('P') == 285.0 || (block.val('P') == 287.0 || block.val('P') == 289.0 || block.val('P') == 291.0))
    {
        if (m_edm94.gapGainProfile)
        {
            const auto* authored = GetBaseProgramCache().TryGetLine(pc);
            if (!authored || !E94Profile(authored->parsedBlock, m_edm94.gapHistoryServo ? m_edm94.gapHistoryArm + 1U : m_edm94.gapFlushProfile ? (m_edm94.gapFlushInterruptArm ? m_edm94.gapFlushInterruptArm + 1U : m_edm94.gapModeFlush ? (m_edm94.gapModeInterruption ? m_edm94.gapModeInterruptionArm + 1U : E116FinishP(m_edm94.gapModeFixture)) : m_edm94.gapCurveB2 ? 128U : m_edm94.gapPhysicalCurve ? 126U : m_edm94.gapCurveRuntime ? 124U : m_edm94.gapCurveGeometry ? 122U : m_edm94.gapB2Repeat ? 120U : m_edm94.gapB2CrossSegment ? 118U : m_edm94.gapCommonPath ? 116U : m_edm94.gapFlushParameterized ? 110U : 104U) : m_edm94.gapPairedProfile ? 102U : m_edm94.gapEnduranceProfile ? 100U : m_edm94.gapContinuousProfile ? 98U : m_edm94.gapBurstProfile ? 96U : m_edm94.gapNoiseProfile ? 94U : 92U) ||
                !E94Matches(authored->parsedBlock, block) || !IsGapDryRunBlockShapeValid(block))
            { RejectEDM94SameThread(m_edm94.gapFlushProfile ? "EDM105_FINISH_LITERAL_SCOPE" : m_edm94.gapPairedProfile ? "EDM104_FINISH_LITERAL_SCOPE" : m_edm94.gapEnduranceProfile ? "EDM102_FINISH_LITERAL_SCOPE" : m_edm94.gapContinuousProfile ? "EDM100_FINISH_LITERAL_SCOPE" : m_edm94.gapBurstProfile ? "EDM99_FINISH_LITERAL_SCOPE" : m_edm94.gapNoiseProfile ? "EDM98_FINISH_LITERAL_SCOPE" :
                "EDM97_FINISH_LITERAL_SCOPE", m_gapDryRun.sourceLine); return nullptr; }
        }
        if (!m_edm94.latched || !m_edm94.ready || m_edm94.cancelled ||
            block.val('P') != (m_edm94.gapHistoryServo ? m_edm94.gapHistoryArm + 1U : m_edm94.gapFlushProfile ? (m_edm94.gapFlushInterruptArm ? static_cast<double>(m_edm94.gapFlushInterruptArm + 1U) : m_edm94.gapModeFlush ? (m_edm94.gapModeInterruption ? static_cast<double>(m_edm94.gapModeInterruptionArm + 1U) : E116FinishP(m_edm94.gapModeFixture)) : m_edm94.gapCurveB2 ? 128.0 : m_edm94.gapPhysicalCurve ? 126.0 : m_edm94.gapCurveRuntime ? 124.0 : m_edm94.gapCurveGeometry ? 122.0 : m_edm94.gapB2Repeat ? 120.0 : m_edm94.gapB2CrossSegment ? 118.0 : m_edm94.gapCommonPath ? 116.0 : m_edm94.gapFlushParameterized ? 110.0 : 104.0) : m_edm94.gapPairedProfile ? 102.0 : m_edm94.gapEnduranceProfile ? 100.0 : m_edm94.gapContinuousProfile ? 98.0 : m_edm94.gapBurstProfile ? 96.0 : m_edm94.gapNoiseProfile ? 94.0 : m_edm94.gapGainProfile ? 92.0 : m_edm94.gapSegmentProfile ? 90.0 :
                m_edm94.gapBoundaryProfile ? 88.0 : 86.0) ||
            m_edm94.next != m_edm94.moves + 1U || pc != m_edm94.pc[m_edm94.moves] ||
            m_pathFeed.pending || (m_edm94.gapPhysicalCurve && m_pathArc.pending) || !m_motion.IsGroupDone() ||
            (m_edm94.gapSegmentProfile && !ReadEDM96CompletedSegmentsSameThread(m_edm94.moves)))
        { RejectEDM94SameThread("FINISH_SCOPE", m_gapDryRun.sourceLine); return nullptr; }
        // An interruption fixture must never retire as a normal full-cycle PASS.
        // This terminal fence also covers a missed sample or unavailable service gate.
        if (m_edm94.gapFlushInterruptArm)
        {
            m_edm94.gapFlushInterruptRequested = true;
            EDM96::Feedback gap{};
            if (m_motion.ReadEDM96Feedback(gap) && gap.session == m_edm94.session &&
                gap.epoch == m_motion.GetCurrentExecutionEpoch() && gap.ownerGeneration == m_edm94.lease.generation)
            {
                m_edm94.gapFlushInterruptSampleTick = gap.tick;
                m_edm94.gapFlushInterruptEpoch = gap.epoch;
            }
            const bool fixture108 = m_edm94.gapFlushParameterized &&
                (m_edm94.gapFlushInterruptArm == 111U || m_edm94.gapFlushInterruptArm == 113U);
            CancelEDM94SameThread(fixture108 ? "EDM108_TARGET_MISSED" : "EDM106_TARGET_MISSED");
            RtPrintf("[%s] event=TARGET_MISSED arm=%u session=%llu scope=FINISH_WITHOUT_INTERRUPT action=CANCEL_THEN_HOLD restart=RESET_REQUIRED\n",
                fixture108 ? "EDM108" : "EDM106", m_edm94.gapFlushInterruptArm, static_cast<unsigned long long>(m_edm94.session));
            FeedHold();
            return nullptr;
        }
        if (m_edm94.gapCommonPath && !IsEDM109CompleteSameThread())
        { RejectEDM94SameThread("EDM109_COMMON_PATH_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapB2CrossSegment && !IsEDM110CompleteSameThread(true))
        { RejectEDM94SameThread("EDM110_PHYSICAL_B2_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapCurveGeometry && !IsEDM112GeometryCurrentSameThread(true))
        { RejectEDM94SameThread("EDM112_CURVE_EVIDENCE_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapCurveRuntime && !IsEDM113RuntimeCurrentSameThread(EDM109::Capacity))
        { RejectEDM94SameThread("EDM113_RUNTIME_CURVE_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapHistoryServo && !IsEDM126CurrentSameThread(m_edm94.moves))
        { RejectEDM94SameThread("EDM126_HISTORY_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapPhysicalCurve && !IsEDM114CurrentSameThread(m_edm94.moves))
        { RejectEDM94SameThread("EDM114_PHYSICAL_CURVE_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapCurveB2 && !IsEDM115CurrentSameThread(EDM114::RequiredRows))
        { RejectEDM94SameThread("EDM115_CURVE_B2_INCOMPLETE", m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapModeFlush && !IsEDM116CurrentSameThread(EDM114::RequiredRows))
        { RejectEDM94SameThread((EDM125::ValidProfile(EDM125::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM125_MODE_FLUSH_INCOMPLETE" : EDM123::ValidProfile(EDM123::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM123_MODE_FLUSH_INCOMPLETE" : EDM122::ValidProfile(EDM122::ProfileForArm(m_edm94.gapModeInterruptionArm)) ? "EDM122_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeInterruptionArm == EDM121::ArmP ? "EDM121_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeInterruptionArm == EDM120::ArmP ? "EDM120_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeInterruptionArm == EDM119::ArmP ? "EDM119_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeInterruption ? "EDM118_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? "EDM124_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeFixture == EDM116::Fixture::ArcPairComplement124 ? "EDM124_MODE_FLUSH_INCOMPLETE" : m_edm94.gapModeFixture == EDM116::Fixture::Complement117 ? "EDM117_MODE_FLUSH_INCOMPLETE" : "EDM116_MODE_FLUSH_INCOMPLETE"), m_gapDryRun.sourceLine); return nullptr; }
        if (m_edm94.gapModeFlush) LogEDM116SameThread("MODE_FLUSH_COMPLETE");
        if (m_edm94.gapCurveB2) LogEDM115SameThread("CURVE_B2_COMPLETE");
        if (m_edm94.gapPhysicalCurve) LogEDM114SameThread("PHYSICAL_CURVE_COMPLETE");
        if (m_edm94.gapCurveRuntime) LogEDM113RuntimeSameThread();
        if (m_edm94.gapCurveGeometry)
            RtPrintf("[EDM112] event=COMMON_CURVE_STAGE_COMPLETE build=EDM112 session=%llu physicalG01=16 physicalB2=2 shadowG01=16 physicalArc=0 numeric=1 applyToMotion=0 dischargeCommand=NONE\n",
                static_cast<unsigned long long>(m_edm94.session));
        if (!E156FinishTargetMatches(m_edm94, m_edm130.recipe, m_edm94.feedback, m_edm114.retained))
        { RejectEDM94SameThread("EDM156_FINISH_TARGET_RECEIPT", m_gapDryRun.sourceLine); return nullptr; }
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
            {RejectEDM94SameThread("EDM234_LINE_FINISH_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
            m_edm234.currentFinishDispatch=m_currentExecutingBlockDispatchId;
            if(m_edm234.program!=236U&&!m_edm234.secondStarted)m_edm234.firstFinishDispatch=m_currentExecutingBlockDispatchId;
        }
        m_edm94.ready = false; m_edm94.finishPending = true; m_edm94.polls = 0U;
        m_motion.RequestEDM94Finish(m_edm94.session);
        LogEDM94SameThread("FINISH_REQUEST", "AWAIT_PHYSICAL_ZERO_STABILITY");
        return [](NCManager* nc) { return nc->WaitEDM94SameThread(); };
    }
    if (m_edm94.latched || !PreflightEDM94ProgramSameThread(pc) || m_mode != NCOperationMode::MEMORY ||
        m_state != NCState::RUN || Close_System_Com_flag || !m_macroStack.empty() || m_isG66Active || Homing.IsActive() ||
        !m_externalReadyInterlock || AlarmManager::GetInstance().HasAlarm() ||
        !IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) || CoordSys.GetTranslationSnapshot().distanceMode != 91 ||
        (CoordSys.IsTranslationRunFrozen() && (!m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()) ||
            !IsXYZCUVFeedProfileCurrentSameThread())) ||
        !m_programMotionLease.IsValid() || m_programMotionLease.owner != MotionOwner::AUTO ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() || m_motion.GetCommandReplaySize() ||
        m_motion.GetAxisCommandMailboxDepth() || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active ||
        m_pathFeed.pending || m_pathArc.pending || m_pathReplay.pending || m_pathHold.armed || m_pathHold.bound ||
        m_edmZFixture.cleanupPending || m_edmZFixture.resetPending || m_edmZFixture.failureAlarmPending ||
        m_edmCoordinatedRT.retirePending || (m_edmCoordinatedRT.submitted && !m_edmCoordinatedRT.retired) ||
        m_edm94Serial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectEDM94SameThread("PREFLIGHT_OR_NOT_READY", m_gapDryRun.sourceLine); return nullptr; }
    if (m_edm94.gapGainProfile)
    {
        const auto* authored = GetBaseProgramCache().TryGetLine(pc);
        EDM97::Tuning parsed{};
        const EDM97::Tuning decoded{ block.val('Q'), block.val('R'), block.val('S') };
        EDM107::FlushProfile parsedParameters{}, decodedParameters{};
        if ((m_edm94.gapFlushParameterized || m_edm94.gapHistoryMode) &&
            (!EDM107::ParseFlushProfile(block.val('I'), block.val('J'), block.val('K'), decodedParameters) ||
                !EDM107::SameFlushProfile(decodedParameters, m_edm94.gapFlushParameters)))
        { RejectEDM94SameThread("EDM107_ARM_DECODED_PARAMETERS", m_gapDryRun.sourceLine); return nullptr; }
        if (!authored || !(m_edm94.gapHistoryMode ? E108ProfileOnlyNPQRSIJK(authored->parsedBlock, m_edm94.gapHistoryArm, parsed, parsedParameters) : m_edm94.gapHistoryServo ? E126Profile(authored->parsedBlock, m_edm94.gapHistoryArm, parsed) : m_edm94.gapFlushParameterized ? E108ProfileOnlyNPQRSIJK(authored->parsedBlock,
                m_edm94.gapFlushInterruptArm ? m_edm94.gapFlushInterruptArm : m_edm94.gapModeFlush ? (m_edm94.gapModeInterruption ? m_edm94.gapModeInterruptionArm : E116ArmP(m_edm94.gapModeFixture)) : m_edm94.gapCurveB2 ? 127U : m_edm94.gapPhysicalCurve ? 125U : m_edm94.gapCurveRuntime ? 123U : m_edm94.gapCurveGeometry ? 121U : m_edm94.gapB2Repeat ? 119U : m_edm94.gapB2CrossSegment ? 117U : m_edm94.gapCommonPath ? 115U : 109U, parsed, parsedParameters) :
                m_edm94.gapFlushProfile ? E106FlushProfile(authored->parsedBlock, m_edm94.gapFlushInterruptArm, parsed) :
                m_edm94.gapPairedProfile ? E104ProfileOnlyNPQRS(authored->parsedBlock, parsed) :
                m_edm94.gapEnduranceProfile ? E102ProfileOnlyNPQRS(authored->parsedBlock, parsed) :
                m_edm94.gapContinuousProfile ? E100ProfileOnlyNPQRS(authored->parsedBlock, parsed) :
                m_edm94.gapBurstProfile ? E99ProfileOnlyNPQRS(authored->parsedBlock, parsed) :
                m_edm94.gapNoiseProfile ? E98ProfileOnlyNPQRS(authored->parsedBlock, parsed) :
                E97ProfileOnlyNPQRS(authored->parsedBlock, parsed)) ||
            !E94Matches(authored->parsedBlock, block) || !IsGapDryRunBlockShapeValid(block) ||
            !EDM97::SameTuning(parsed, m_edm94.gapTuning) || !EDM97::SameTuning(decoded, m_edm94.gapTuning) ||
            !EDM107::SameFlushProfile(parsedParameters, m_edm94.gapFlushParameters))
        { RejectEDM94SameThread(m_edm94.gapFlushProfile ? "EDM105_ARM_LITERAL_TUNING" : m_edm94.gapPairedProfile ? "EDM104_ARM_LITERAL_TUNING" : m_edm94.gapEnduranceProfile ? "EDM102_ARM_LITERAL_TUNING" : m_edm94.gapContinuousProfile ? "EDM100_ARM_LITERAL_TUNING" : m_edm94.gapBurstProfile ? "EDM99_ARM_LITERAL_TUNING" : m_edm94.gapNoiseProfile ? "EDM98_ARM_LITERAL_TUNING" :
            "EDM97_ARM_LITERAL_TUNING", m_gapDryRun.sourceLine); return nullptr; }
    }
    if (m_edm94.gapBoundaryProfile && m_isSingleBlockEnabled)
    { RejectEDM94SameThread("EDM95_CONTINUOUS_MODE_REQUIRED", m_gapDryRun.sourceLine); return nullptr; }
    if (m_edm94.gapSegmentProfile && m_isSingleBlockEnabled)
    { RejectEDM94SameThread("EDM96_CONTINUOUS_MODE_REQUIRED", m_gapDryRun.sourceLine); return nullptr; }
    // The preceding bracket is already released before a new arm is admitted.
    // HOLD/RESET ledger invalidation deliberately does not erase this separate evidence.
    m_edm118 = EDM118NCState{};
    InvalidateEDM109CommonPathSameThread("NEW_ARM");
    m_edm109.plans = m_edm109.mappedSegments = m_edm109.lastMappingTick = 0ULL;
    m_edm94.boundary = EDM94NCState::FeedBoundary{};
    m_edm94.session = ++m_edm94Serial;
    m_edm94.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_edm94.cache = GetBaseProgramCache().GetGeneration();
    m_edm94.armDispatch = m_currentExecutingBlockDispatchId; m_edm94.armPC = pc;
    m_edm94.lease = m_programMotionLease; m_edm94.armEpoch = m_motion.GetCurrentExecutionEpoch(); m_edm94.feedback = EDM94::Feedback{};
    if (EDM116::IsModalFixture(m_edm94.gapModeFixture))
    {
        m_edm127.current = EDM127::Initial(pc, m_edm94.armDispatch);
        m_edm127.lastPC = pc; m_edm127.lastDispatch = m_edm94.armDispatch;
    }
    if (m_edm94.gapModeInterruption)
    {
        m_edm118.armP = m_edm94.gapModeInterruptionArm;
        m_edm118.session = m_edm94.session; m_edm118.run = m_edm94.run; m_edm118.cache = m_edm94.cache;
        m_edm118.armEpoch = m_edm94.armEpoch; m_edm118.ownerGeneration = m_edm94.lease.generation;
    }
    m_edm94.ready = m_edm94.finishPending = m_edm94.complete = m_edm94.cancelled = false;
    m_edm94.resetSeen = m_edm94.resetCommitted = m_edm94.everArmed = false;
    m_edm94.releaseRequested = m_edm94.terminalLogged = m_edm94.rtFaultReported = false;
    m_edm94.normalEndCommitted = m_edm94.completionReleaseRequested = false;
    m_edm94.completionIdleLease = MotionOwnerLease{};
    m_edm94.completionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_edm94.completionNoneGeneration = MOTION_OWNER_GENERATION_INVALID;
    m_edm94.completionSafetyProvenance = m_edm94.completionAlarmSafetyIntent = 0ULL;
    m_edm94.completionAlarmUpdate = 0U;
    m_edm94.polls = 0U; m_edm94.lastTick = 0ULL;
    m_edm94.gapFeedback = EDM95::Feedback{};
    m_edm94.gapWaitEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_edm94.gapWaitFirstUs = m_edm94.gapWaitLastUs = m_edm94.gapWaitLastTick = 0ULL;
    m_edm94.gapLoggedAccepted = m_edm94.gapLoggedPhase = 0ULL;
    m_edm94.gapWaitStep = m_edm94.gapWaitPolls = 0U;
    m_edm94.gapWaiting = m_edm94.gapPermitReady = false;
    m_edm94.gapFlushInterruptRequested = false;
    m_edm94.gapFlushInterruptSampleTick = m_edm94.gapFlushInterruptEpoch = 0ULL;
    if (m_edm94.gapCurveGeometry && !RunEDM112GeometrySameThread())
    { RejectEDM94SameThread("EDM112_GEOMETRY_FIXTURE", m_gapDryRun.sourceLine); return nullptr; }
    if (m_edm94.gapWorkTime && !BindEDM130WorkTimeSameThread())
    { RejectEDM94SameThread("EDM130_WORK_TIME_BIND", m_gapDryRun.sourceLine); return nullptr; }
    if (EDM149::IsBoundedProgramTestArm(m_edm94.gapHistoryArm) && m_edm130.recipe.commissioningProfile == 0U &&
        !PreflightEDM160BoundedTravelSameThread())
    { RejectEDM94SameThread("EDM160_BOUNDED_TRAVEL_PREFLIGHT", m_gapDryRun.sourceLine); return nullptr; }
    if (EDM149::IsVisibleFlushTestArm(m_edm94.gapHistoryArm) && !PreflightEDM158VisibleTravelSameThread())
    { RejectEDM94SameThread("EDM158_FULL_LIFT_TRAVEL_PREFLIGHT", m_gapDryRun.sourceLine); return nullptr; }
    if (EDM149::IsXYZVisibleFlushTestArm(m_edm94.gapHistoryArm) && !PreflightEDM159XYZVisibleTravelSameThread())
    { RejectEDM94SameThread("EDM159_XYZ_LIFT_TRAVEL_PREFLIGHT", m_gapDryRun.sourceLine); return nullptr; }
    if (EDM149::IsSixVisibleFlushTestArm(m_edm94.gapHistoryArm) && !PreflightEDM167SixVisibleTravelSameThread())
    { RejectEDM94SameThread("EDM167_SIX_LIFT_TRAVEL_PREFLIGHT", m_gapDryRun.sourceLine); return nullptr; }
    if (E140AssignmentArm(m_edm94.gapHistoryArm))
    {
        if (!m_edm140.count || m_edm140.count > EDM140::AssignmentCapacity ||
            m_edm140.next || !m_edm130.bound ||
            m_edm130.recipe.diagnosticControlSource != E149ControlSource(m_edm94.gapHistoryArm))
        { RejectEDM94SameThread("EDM140_PLAN_BIND", m_gapDryRun.sourceLine); return nullptr; }
        m_edm140.session = m_edm94.session; m_edm140.run = m_edm94.run;
        m_edm140.cache = m_edm94.cache; m_edm140.armEpoch = m_edm94.armEpoch;
        m_edm140.ownerGeneration = m_edm94.lease.generation; m_edm140.sealed = true;
    }
    if (EDM149::IsTestArm(m_edm94.gapHistoryArm) && !ConsumeEDM149ConfirmationSameThread(pc))
    { RejectEDM94SameThread("EDM149_M00_START_CONFIRMATION_REQUIRED", m_gapDryRun.sourceLine); return nullptr; }
    // EDM247 FIX1 and EDM248: the ordinary recipe intentionally retains marker zero.
    // Only its current, confirmed sequence parent may request the longer budget.
    unsigned ordinarySequenceProgram=0U;
    if(m_edm237.authorized&&(m_edm237.program==247U||m_edm237.program==248U))
    {
        const auto& q=m_edm237;const auto& s=m_edm94;
        if(!IsEDM237SequenceScopeCurrentSameThread()||q.parentIndex>=q.parentCount||q.parentIndex>=4U||
            q.kinds[q.parentIndex]!=5U||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending||q.currentRowBound||
            !s.program||!s.gapHistoryServo||!s.gapHistoryMode||!s.gapWorkTime||s.gapHistoryArm!=274U||s.moves!=4U||
            s.session!=q.currentSession||s.armEpoch!=q.currentArmEpoch||s.armPC!=q.armPC[q.parentIndex]||
            s.run!=q.run||s.cache!=q.cache||!s.lease.Matches(q.lease)||s.armDispatch!=m_currentExecutingBlockDispatchId||
            !m_edm130.bound||!EDM130::SameRecipe(m_edm130.recipe,q.currentRecipe)||
            !E237RecipeForProgramKind(m_edm130.recipe,5U,q.program))
        {RejectEDM94SameThread(q.program==248U?"EDM248_ORDINARY_DURATION_SCOPE":"EDM247_ORDINARY_DURATION_SCOPE",m_gapDryRun.sourceLine);return nullptr;}
        ordinarySequenceProgram=q.program;
    }
    const bool submitted = m_edm94.gapWorkTime ?
        m_motion.SubmitEDM130Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease,
            m_edm94.gapTuning, m_edm94.gapFlushParameters, E149HistoryFixture(m_edm94.gapHistoryArm), m_edm130.recipe,
            EDM149::IsManualVoltageTestArm(m_edm94.gapHistoryArm) ? m_edm94.run : 0ULL,
            EDM149::IsManualVoltageTestArm(m_edm94.gapHistoryArm) ? m_edm94.cache : 0ULL,
            false,ordinarySequenceProgram) : m_edm94.gapPhysicalAd ?
        m_motion.SubmitEDM129Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease,
            m_edm94.gapTuning, m_edm94.gapFlushParameters, m_edm94.gapHistoryArm == 170U ? 0U : 1U) : m_edm94.gapHistoryMode ?
        m_motion.SubmitEDM128Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease,
            m_edm94.gapTuning, m_edm94.gapFlushParameters, m_edm94.gapHistoryArm == 166U ? 0U : 1U) : m_edm94.gapHistoryServo ?
        m_motion.SubmitEDM126Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease,
            m_edm94.gapTuning, m_edm94.gapModeFixture == EDM116::Fixture::ArcPair124 ? 0U : 1U) : m_edm94.gapModeFlush ?
        m_motion.SubmitEDM116Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease,
            m_edm94.gapTuning, m_edm94.gapFlushParameters, m_edm94.gapModeFixture) : m_edm94.gapCurveB2 ?
        m_motion.SubmitEDM115Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease,
            m_edm94.gapTuning, m_edm94.gapFlushParameters) : m_edm94.gapPhysicalCurve ?
        m_motion.SubmitEDM114Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning, m_edm94.gapFlushParameters) : m_edm94.gapFlushProfile ?
        m_motion.SubmitEDM105Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning, m_edm94.gapFlushParameters, m_edm94.gapCommonPath, m_edm94.gapB2CrossSegment, m_edm94.gapB2Repeat, m_edm94.gapCurveRuntime) : m_edm94.gapPairedProfile ?
        m_motion.SubmitEDM104Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning) : m_edm94.gapEnduranceProfile ?
        m_motion.SubmitEDM102Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning) : m_edm94.gapContinuousProfile ?
        m_motion.SubmitEDM100Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning) : m_edm94.gapBurstProfile ?
        m_motion.SubmitEDM99Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning) : m_edm94.gapNoiseProfile ?
        m_motion.SubmitEDM98Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning) : m_edm94.gapGainProfile ?
        m_motion.SubmitEDM97Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease, m_edm94.gapTuning) : m_edm94.gapSegmentProfile ?
        m_motion.SubmitEDM96Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease) : m_edm94.gapBoundaryProfile ?
        m_motion.SubmitEDM95Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease) :
        m_motion.SubmitEDM94Arm(m_edm94.session, m_edm94.armEpoch, m_edm94.lease);
    if (!submitted)
    { RejectEDM94SameThread("ARM_PUBLICATION", m_gapDryRun.sourceLine); return nullptr; }
    if(ordinarySequenceProgram==247U||ordinarySequenceProgram==248U)
        PrintNCG76DiagnosticLine("[EDM%u%s] event=PARENT_DURATION_BUDGET parent=%u kind=5 session=%llu maxDurationUs=%llu E5policy=UNCHANGED discharge=0\n",
            ordinarySequenceProgram,ordinarySequenceProgram==247U?"_FIX1":"",m_edm237.parentIndex+1U,static_cast<unsigned long long>(m_edm94.session),
            static_cast<unsigned long long>(EDM94::OrdinarySequenceMaximumDurationUs247));
    m_edm94.latched = m_edm94.armPending = true;
    if(m_edm94.gapHistoryServo)
        RtPrintf("[EDM145] event=BUILD scope=SOURCE_STARTUP_AND_LOCAL_PROOF_REUSE session=%llu thread=NC phase=ARM_REQUEST rtBuildPrint=0 noCrossTickCache=1\n",
            static_cast<unsigned long long>(m_edm94.session));
    if (E146PhysicalMotionArm(m_edm94.gapHistoryArm))
        RtPrintf("[EDM146] event=BUILD scope=PHYSICAL_AD_MACRO_MOTION session=%llu armP=%u finishP=%u thread=NC source=PHYSICAL_AD stimulus=OFF rows=16 expressions=FROZEN_AND_LIVE_CHECKED physicalReversalCoverage=OBSERVED_ONLY powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapHistoryArm, m_edm94.gapHistoryArm + 1U);
    if (E146PhysicalMotionArm(m_edm94.gapHistoryArm))
        RtPrintf("[EDM147] event=BUILD scope=PHYSICAL_AD_CROSS_ROW_RETURN session=%llu armP=%u source=PHYSICAL_AD stimulus=OFF requiredCycles=%u cues=LOW_NOW_RESTORE_HIGH_CYCLE_COMPLETE coverage=COMMITTED_MAPPING authority=NONE\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,
            m_edm94.gapHistoryArm==224U?2U:1U);
    if (EDM149::IsTestArm(m_edm94.gapHistoryArm))
        RtPrintf("[%s] event=BUILD armP=%u finishP=%u source=%s ADrequired=%u motion=REAL_ETHERCAT_%s rows=%u requiredMovementMask=%u feedMmMin=0.12 confirmation=M00_AND_FRESH_START physicalInterlock=0 S92=INHIBITED S93=INHIBITED discharge=0\n",
            EDM149::IsSixVisibleFlushTestArm(m_edm94.gapHistoryArm) ? "EDM167" : EDM149::IsXYZVisibleFlushTestArm(m_edm94.gapHistoryArm) ? "EDM159" : EDM149::IsVisibleFlushTestArm(m_edm94.gapHistoryArm) ? "EDM158" : EDM149::IsPauseTestArm(m_edm94.gapHistoryArm) ? "EDM157" : EDM149::IsBoundedProgramTestArm(m_edm94.gapHistoryArm) ? "EDM156" : EDM149::IsSparseArcTestArm(m_edm94.gapHistoryArm) ? "EDM155" : EDM149::IsSparseTestArm(m_edm94.gapHistoryArm) ? "EDM154" : EDM149::IsOrderTestArm(m_edm94.gapHistoryArm) ? "EDM153" : EDM149::IsArcTestArm(m_edm94.gapHistoryArm) ? "EDM152" : EDM149::IsSixTestArm(m_edm94.gapHistoryArm) ? "EDM151" : EDM149::IsXYZTestArm(m_edm94.gapHistoryArm) ? "EDM150" : "EDM149", m_edm94.gapHistoryArm,m_edm94.gapHistoryArm+1U,EDM149::SourceName(E149ControlSource(m_edm94.gapHistoryArm)),m_edm94.gapPhysicalAd ? 1U : 0U, (EDM149::IsBoundedProgramTestArm(m_edm94.gapHistoryArm) || EDM149::IsSparseArcTestArm(m_edm94.gapHistoryArm) || EDM149::IsSparseTestArm(m_edm94.gapHistoryArm) || EDM149::IsOrderTestArm(m_edm94.gapHistoryArm) || EDM149::IsArcTestArm(m_edm94.gapHistoryArm) || EDM149::IsSixTestArm(m_edm94.gapHistoryArm)) ? "XYZCUV" : EDM149::IsXYZTestArm(m_edm94.gapHistoryArm) ? "XYZ" : "Z", m_edm94.moves, E156RequiredMovementMask(m_edm94, m_edm130.recipe));
    if (E148AuthoredLineArm(m_edm94.gapHistoryArm))
        RtPrintf("[EDM148] event=BUILD scope=AUTHORED_SIX_AXIS_LINE session=%llu armP=%u finishP=%u thread=NC source=SIMULATED_CONTROL physicalMonitor=1 rows=16 lineXYZAbsMm=0.001..0.002 lineCUVAbsDeg=0.020..0.040 feedMmMin=0.12 arcRecipe=UNCHANGED expressions=FROZEN_AND_LIVE_CHECKED powerControl=NC_PLC_S92\n",
            static_cast<unsigned long long>(m_edm94.session),m_edm94.gapHistoryArm,m_edm94.gapHistoryArm+1U);
    LogEDM94SameThread("ARM_REQUEST", m_edm94.gapPhysicalCurve ? "ACTUAL_MOTORS_G91_G01_ARC2" : "ACTUAL_MOTORS_FIXED_G91_G1");
    if (m_edm94.gapModeFlush) LogEDM116SameThread("ARM_REQUEST");
    if (m_edm94.gapCurveB2) LogEDM115SameThread("ARM_REQUEST");
    if (m_edm94.gapPhysicalCurve) LogEDM114SameThread("ARM_REQUEST");
    else if (m_edm94.gapCurveRuntime)
    {
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM113] event=ARM_REQUEST build=EDM113 logger=FIX1 session=%llu P=123 finishP=124 expectedG01=16 expectedPhysicalB2=2 runtimeCurve=OBSERVATION physicalArc=0 applyToMotion=0 pauseDuringB2=1 dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session));
        EmitEDM113Record(record, length, "ARM_REQUEST");
    }
    else if (m_edm94.gapCurveGeometry)
        RtPrintf("[EDM112] event=ARM_REQUEST build=EDM112 session=%llu P=121 finishP=122 expectedG01=16 expectedPhysicalB2=2 expectedShadowG01=16 physicalArc=0 arcScope=NUMERIC_ONLY applyCurveToMotion=0 dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session));
    else if (m_edm94.gapB2Repeat)
        RtPrintf("[EDM111] event=ARM_REQUEST build=EDM111 session=%llu P=119 finishP=120 triggerSteps=2,4 trigger=DWELL_RETURN previousDepthRatio=0.1 physicalB2=REQUESTED proof=NONE expectedB2=2 expectedDwells=10 expectedSegments=16 expectedFlushCycles=8 numericLedger=SEPARATE dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session));
    else if (m_edm94.gapB2CrossSegment)
        RtPrintf("[EDM110] event=ARM_REQUEST session=%llu P=117 finishP=118 triggerStep=2 trigger=DWELL_RETURN previousDepthRatio=0.1 physicalB2=REQUESTED proof=NONE expectedB2=1 expectedSegments=16 expectedFlushCycles=8 numericLedger=SEPARATE dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session));
    else if (m_edm94.gapCommonPath)
        RtPrintf("[EDM109] event=ARM_REQUEST session=%llu P=115 finishP=116 source=COMPLETED_G01_ONLY physicalCommonMap=1 physicalB2=0 numericB2=1 historyCapacity=16 expectedPlans=15 dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session));
    if (m_edm94.gapFlushParameterized)
    {
        char record[481]{}; const auto& parameters = m_edm94.gapFlushParameters;
        const int length = std::snprintf(record, sizeof(record),
            "[EDM107] event=PARAMETERS_FROZEN session=%llu P=%u I=%.12g J=%.12g K=%llu strokeRatio=%.12g speedRatio=%.12g dwellUs=%llu dwellTicks=%llu apply=NEXT_ARM motion=REAL_SIX_AXIS gap=RT_SYNTHETIC dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapFlushInterruptArm ? m_edm94.gapFlushInterruptArm : m_edm94.gapModeFlush ? (m_edm94.gapModeInterruption ? m_edm94.gapModeInterruptionArm : E116ArmP(m_edm94.gapModeFixture)) : m_edm94.gapCurveB2 ? 127U : m_edm94.gapPhysicalCurve ? 125U : m_edm94.gapCurveRuntime ? 123U : m_edm94.gapCurveGeometry ? 121U : m_edm94.gapB2Repeat ? 119U : m_edm94.gapB2CrossSegment ? 117U : m_edm94.gapCommonPath ? 115U : 109U,
            parameters.strokeRatio * 100.0,
            parameters.speedRatio * 100.0, static_cast<unsigned long long>(parameters.dwellUs / 1000ULL),
            parameters.strokeRatio, parameters.speedRatio, static_cast<unsigned long long>(parameters.dwellUs),
            static_cast<unsigned long long>(EDM107::FlushDwellTicks(parameters)));
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM107] event=LOG_FORMAT_ERROR scope=PARAMETERS_FROZEN\n");
    }
    if (m_edm94.gapBoundaryProfile)
        RtPrintf("[EDM95] event=GAP_SOURCE session=%llu source=RT_SYNTHETIC voltageHigh=50 voltageLow=20 referenceV=35 phaseUs=2000000 producerUs=1000 dwellUs=20000 offerWindowUs=1000000 permitUs=200000 mode=BOUNDARY_DIRECTION_ONLY midSegmentReverse=0 steps=%u hold=CANCEL_RESET_REQUIRED dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.moves);
    if (m_edm94.gapSegmentProfile && !m_edm94.gapGainProfile)
        RtPrintf("[EDM96] event=GAP_SOURCE session=%llu source=RT_SYNTHETIC voltagePlan=50_600ms_20_600ms_50_STEADY producerUs=250 dwellUs=20000 segmentDeadlineUs=10000000 mode=WITHIN_SEGMENT_SERVO midSegmentReverse=1 steps=%u hold=CANCEL_RESET_REQUIRED dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.moves);
    if (m_edm94.gapGainProfile && !m_edm94.gapHistoryServo && !m_edm94.gapNoiseProfile && !m_edm94.gapBurstProfile)
    {
        char record[768]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM97] event=GAP_SOURCE session=%llu source=RT_SYNTHETIC voltagePlan=50_45_20_25_50 phaseUs=300000 producerUs=250 dwellUs=20000 segmentDeadlineUs=10000000 referenceV=35 scale=0.001 Q=%.9g R=%.9g S=%.9g configHash=%llu mode=VOLTAGE_ERROR_GAIN_SERVO midSegmentReverse=1 steps=%u immutablePerArm=1 gainUnit=DIRECT_CURVE hold=CANCEL_RESET_REQUIRED dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapTuning.positiveGain,
            m_edm94.gapTuning.negativeGain, m_edm94.gapTuning.deadbandV,
            static_cast<unsigned long long>(EDM97::ConfigHash(m_edm94.gapTuning)), m_edm94.moves);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM97] event=LOG_FORMAT_ERROR scope=GAP_SOURCE output=NONE\n");
    }
    if (m_edm94.gapNoiseProfile)
    {
        char record[896]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM98] event=GAP_SOURCE session=%llu source=RT_SYNTHETIC voltagePlan=50_600ms_NOISE_800ms_20_600ms_NOISE_800ms_50_STEADY noiseV=34.5_35.5 producerUs=250 noiseFreshUs=250 dwellUs=20000 quietRequiredTicks=800 quietRequiredUs=200000 quietScope=COMMAND_SCALAR_ONLY segmentDeadlineUs=10000000 referenceV=35 scale=0.001 Q=%.9g R=%.9g S=%.9g configHash=%llu mode=DEADBAND_NOISE_SERVO midSegmentReverse=1 steps=%u immutablePerArm=1 gainUnit=DIRECT_CURVE hold=CANCEL_RESET_REQUIRED dischargeCommand=NONE\n",
            static_cast<unsigned long long>(m_edm94.session), m_edm94.gapTuning.positiveGain,
            m_edm94.gapTuning.negativeGain, m_edm94.gapTuning.deadbandV,
            static_cast<unsigned long long>(EDM97::ConfigHash(m_edm94.gapTuning)), m_edm94.moves);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM98] event=LOG_FORMAT_ERROR scope=GAP_SOURCE output=NONE\n");
    }
    if (m_edm94.gapBurstProfile)
        LogEDM99GapSource(m_edm94.gapFlushProfile ? "EDM105" : m_edm94.gapPairedProfile ? "EDM104" : m_edm94.gapEnduranceProfile ? "EDM102" : m_edm94.gapContinuousProfile ? "EDM100" : "EDM99", m_edm94.session,
            m_edm94.gapTuning, m_edm94.moves, m_edm94.gapContinuousProfile, m_edm94.gapEnduranceProfile, m_edm94.gapPairedProfile);
    return [](NCManager* nc) { return nc->WaitEDM94SameThread(); };
}

bool NCManager::IsEDM94CompletionOwnerSameThread(const MotionOwnerLease& owner) const noexcept
{
    const auto& s = m_edm94;
    return s.normalEndCommitted && s.completionReleaseRequested &&
        s.completionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        s.completionIdleLease.owner == MotionOwner::IDLE_HOLD && s.completionIdleLease.IsValid() &&
        s.completionNoneGeneration != MOTION_OWNER_GENERATION_INVALID &&
        (owner.Matches(s.completionIdleLease) ||
            (owner.owner == MotionOwner::NONE && owner.generation == s.completionNoneGeneration));
}

bool NCManager::HasEDM94CompletionReleaseAcknowledgementSameThread(const EDM94::Feedback& f) const noexcept
{
    const auto& s = m_edm94;
    // Deliberately independent of current NC state/reset flags. An already
    // completed old transaction can be consumed after a newer RESET intent.
    return E156FinishTargetMatches(s, m_edm130.recipe, f, m_edm114.retained) &&
        s.completionReleaseRequested && s.session != 0ULL &&
        f.identity.session == s.session && f.identity.epoch == s.armEpoch &&
        f.identity.ownerGeneration == s.lease.generation &&
        f.lastTick != 0ULL && f.lastTick >= s.lastTick &&
        f.completionPhase == EDM94::CompletionReleasePhase::Released &&
        f.completionReleased && f.released && !f.active && !f.faulted && !f.cancelled &&
        f.completionEpoch == s.completionEpoch &&
        f.completionIdleOwnerGeneration == s.completionIdleLease.generation &&
        f.completionNoneOwnerGeneration == s.completionNoneGeneration &&
        f.currentIdentity.session == s.session && f.currentIdentity.epoch == s.completionEpoch &&
        f.currentIdentity.ownerGeneration == s.completionNoneGeneration;
}

bool NCManager::ConsumeEDM94CompletionReleaseSameThread(const EDM94::Feedback& f) noexcept
{
    if (!m_edm94.latched || !HasEDM94CompletionReleaseAcknowledgementSameThread(f)) return false;
    if(m_edm237.authorized&&!IsEDM237FinalStateSameThread())return false;
    if(m_edm234.authorized&&m_edm234.program==236U)
    {
        const auto& q=m_edm234;const auto& state=m_edm94;int first=-1;std::array<unsigned,4U> kinds{};
        if(q.parentIndex!=3U||q.retiredMask!=7U||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending||
            !q.firstRecipeBound||!q.firstSession||q.previousSession<q.firstSession||state.session!=q.currentSession||
            q.currentSession<=q.previousSession||state.armPC!=q.firstArmPC+9||
            q.run!=state.run||q.cache!=state.cache||q.cache!=GetBaseProgramCache().GetGeneration()||
            !EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||!E236RecipeForKind(q.currentRecipe,q.kinds[3U])||
            !E236ProgramShape(GetBaseProgramCache(),first,kinds)||first!=q.firstArmPC||kinds!=q.kinds)return false;
    }
    m_edm94.feedback = f;
    m_edm94.lastTick = f.lastTick;
    LogEDM94SameThread("NORMAL_RELEASED", "M30_AND_EXACT_NONE_PHYSICAL_ZERO");
    // Repeat the already accepted observation at the normal-release snapshot.
    // Cancellation can consume an old release after clearing this evidence;
    // it must not print a completion from an empty/retired observer record.
    const auto& runtime = m_edm113.retained;
    if (m_edm94.gapCurveRuntime && m_edm113.acknowledgedRows == EDM109::Capacity &&
        runtime.completed && !runtime.failed && runtime.completedRows == EDM109::Capacity &&
        runtime.session == m_edm94.session && runtime.armEpoch == m_edm94.armEpoch &&
        runtime.ownerGeneration == m_edm94.lease.generation)
        LogEDM113RuntimeSameThread();
    if (m_edm94.gapPhysicalCurve && m_edm114.acknowledgedRows == m_edm94.moves &&
        m_edm114.retained.completed && !m_edm114.retained.failed &&
        m_edm114.retained.session == m_edm94.session && m_edm114.retained.armEpoch == m_edm94.armEpoch &&
        m_edm114.retained.ownerGeneration == m_edm94.lease.generation)
        LogEDM114SameThread("NORMAL_RELEASED");
    if (m_edm94.gapCurveB2 && m_edm115.retainedEvents == EDM115::ExpectedEvents &&
        m_edm115.receipts[1].eventsCompleted == EDM115::ExpectedEvents &&
        m_edm115.receipts[1].session == m_edm94.session && m_edm115.receipts[1].armEpoch == m_edm94.armEpoch &&
        m_edm115.receipts[1].ownerGeneration == m_edm94.lease.generation && !m_edm115.receipts[1].excursion.faulted)
        LogEDM115SameThread("NORMAL_RELEASED");
    if (m_edm94.gapModeFlush && IsEDM116CurrentSameThread(EDM114::RequiredRows))
        LogEDM116SameThread("NORMAL_RELEASED");
    m_edm94.latched = m_edm94.armPending = m_edm94.ready = m_edm94.finishPending = false;
    if(m_edm237.authorized)m_edm237=EDM237NCState{};
    if(m_edm234.authorized&&m_edm234.program==236U)m_edm234=EDM234NCState{};
    else if(m_edm234.authorized&&m_edm234.secondStarted&&m_edm234.firstReleased&&
        m_edm94.session>m_edm234.firstSession)m_edm234=EDM234NCState{};
    if(m_edm233.authorized&&m_edm233.secondStarted&&m_edm233.firstReleased&&
        m_edm94.session>m_edm233.firstSession)m_edm233=EDM233NCState{};
    return true;
}

bool NCManager::IsEDM94CompletionSelectionCurrentSameThread() const noexcept
{
    const auto& s = m_edm94;
    if(m_edm237.authorized&&!IsEDM237FinalStateSameThread())return false;
    if(m_edm234.authorized&&m_edm234.program==236U)
    {
        const auto& q=m_edm234;int first=-1;std::array<unsigned,4U> kinds{};
        if(q.parentIndex!=3U||q.retiredMask!=7U||!q.stageRecipeBound||!q.stageStarted||q.awaitNext||q.releasePending||
            !q.firstRecipeBound||!q.firstSession||q.previousSession<q.firstSession||s.session!=q.currentSession||
            q.currentSession<=q.previousSession||s.armPC!=q.firstArmPC+9||q.run!=s.run||q.cache!=s.cache||
            q.cache!=GetBaseProgramCache().GetGeneration()||!EDM130::SameRecipe(q.currentRecipe,m_edm130.recipe)||
            !E236RecipeForKind(q.currentRecipe,q.kinds[3U])||!E236ProgramShape(GetBaseProgramCache(),first,kinds)||
            first!=q.firstArmPC||kinds!=q.kinds)return false;
    }
    if (!s.normalEndCommitted || !s.completionReleaseRequested || !s.complete ||
        !s.everArmed || !s.terminalLogged || s.armPending || s.ready || s.finishPending ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.moves == 0U || s.next != s.moves + 2U || m_mode != NCOperationMode::MEMORY ||
        (m_state != NCState::P_END && m_state != NCState::READY) || Close_System_Com_flag ||
        !m_externalReadyInterlock || m_edmState == EDMState::NOT_READY) return false;
    const auto& alarms = AlarmManager::GetInstance();
    const auto current = [&]() noexcept -> bool
    {
        return m_motion.GetCurrentExecutionEpoch() == s.completionEpoch &&
            IsEDM94CompletionOwnerSameThread(m_motion.GetMotionOwnerLease()) &&
            m_motion.GetSafetyProvenanceGeneration() == s.completionSafetyProvenance &&
            !m_motion.HasPendingSafetyOrRecoveryRequests() && !alarms.HasAlarm() &&
            alarms.GetUpdateCount() == s.completionAlarmUpdate &&
            AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState()) == s.completionAlarmSafetyIntent;
    };
    if (!current()) return false;
    EDM94::Feedback f{};
    if (!m_motion.ReadEDM94Feedback(f) || f.identity.session != s.session ||
        f.identity.epoch != s.armEpoch || f.identity.ownerGeneration != s.lease.generation ||
        f.lastTick == 0ULL || f.lastTick < s.lastTick || f.faulted || f.cancelled ||
        f.completionPhase == EDM94::CompletionReleasePhase::Revoked) return false;
    if (f.completionPhase != EDM94::CompletionReleasePhase::None &&
        (f.completionEpoch != s.completionEpoch ||
            f.completionIdleOwnerGeneration != s.completionIdleLease.generation ||
            f.completionNoneOwnerGeneration != s.completionNoneGeneration)) return false;
    if (!E156FinishTargetMatches(s, m_edm130.recipe, f, m_edm114.retained)) return false;
    const bool guardOrReceipt = HasEDM94CompletionReleaseAcknowledgementSameThread(f) ||
        (f.active && f.armAccepted && f.finishRequested && f.forceZero && !f.released);
    return guardOrReceipt && current();
}

bool NCManager::IsEDM94CompletionStartIdentityCurrentSameThread() const noexcept
{
    const auto& s = m_edm94;
    const auto& alarms = AlarmManager::GetInstance();
    return s.normalEndCommitted && s.completionReleaseRequested && s.complete &&
        !s.cancelled && !s.resetSeen && !s.resetCommitted && !s.releaseRequested && !s.rtFaultReported &&
        s.feedback.completionPhase != EDM94::CompletionReleasePhase::Revoked &&
        m_pendingProgramRunEDM94Session != 0ULL && m_pendingProgramRunEDM94Session == s.session &&
        m_pendingProgramRunEDM94Epoch == s.completionEpoch &&
        m_pendingProgramRunEDM94IdleLease.Matches(s.completionIdleLease) &&
        m_pendingProgramRunEDM94NoneGeneration == s.completionNoneGeneration &&
        m_motion.GetCurrentExecutionEpoch() == m_pendingProgramRunEDM94Epoch &&
        IsEDM94CompletionOwnerSameThread(m_motion.GetMotionOwnerLease()) &&
        m_motion.GetSafetyProvenanceGeneration() == s.completionSafetyProvenance &&
        !m_motion.HasPendingSafetyIntent() && !alarms.HasAlarm() &&
        alarms.GetUpdateCount() == s.completionAlarmUpdate &&
        AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState()) == s.completionAlarmSafetyIntent &&
        !Close_System_Com_flag && m_externalReadyInterlock && m_edmState != EDMState::NOT_READY;
}

void NCManager::CompleteEDM94ProgramEndSameThread(const NCProgramEndGateSnapshot& end,
    MotionExecutionEpoch executionEpoch) noexcept
{
    auto& s = m_edm94;
    if (s.gapModeInterruption) return; // Interruption profiles have no normal M30/restart completion authority.
    if (!s.latched || s.normalEndCommitted || end.cause != NCProgramEndCause::M30 ||
        m_state != NCState::P_END || m_mode != NCOperationMode::MEMORY ||
        !s.complete || !s.everArmed || !s.terminalLogged || s.armPending || s.ready || s.finishPending ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.moves == 0U || s.next != s.moves + 2U || end.sourcePC != s.pc[s.moves + 1U] ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        Close_System_Com_flag || !m_externalReadyInterlock || m_edmState == EDMState::NOT_READY ||
        (s.gapCurveGeometry && !IsEDM112GeometryCurrentSameThread(true))) return;
    // The completed observer receipt survives the AUTO -> IDLE_HOLD -> NONE
    // retirement. Validate its frozen row identity, not the current owner/epoch.
    if (s.gapCurveRuntime && !IsEDM113RuntimeCurrentSameThread(EDM109::Capacity))
    { CancelEDM94SameThread("EDM113_M30_RUNTIME_EVIDENCE"); return; }
    if (s.gapHistoryServo && !IsEDM126CurrentSameThread(m_edm94.moves))
    { CancelEDM94SameThread("EDM126_M30_HISTORY_INCOMPLETE"); return; }
    if (s.gapPhysicalCurve && !IsEDM114CurrentSameThread(m_edm94.moves))
    { CancelEDM94SameThread("EDM114_M30_CURVE_EVIDENCE"); return; }
    if (s.gapCurveB2 && !IsEDM115CurrentSameThread(EDM114::RequiredRows))
    { CancelEDM94SameThread("EDM115_M30_B2_EVIDENCE"); return; }
    if (s.gapModeFlush && !IsEDM116CurrentSameThread(EDM114::RequiredRows))
    { CancelEDM94SameThread("EDM116_M30_MODE_EVIDENCE"); return; }
    if (!E156FinishTargetMatches(s, m_edm130.recipe, s.feedback, m_edm114.retained))
    { CancelEDM94SameThread("EDM156_M30_TARGET_RECEIPT"); return; }
    auto& alarms = AlarmManager::GetInstance();
    const MotionOwnerLease idleLease = m_motion.GetMotionOwnerLease();
    const std::uint64_t safety = m_motion.GetSafetyProvenanceGeneration();
    const std::uint32_t alarmUpdate = alarms.GetUpdateCount();
    const std::uint64_t alarmIntent = AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState());
    if (idleLease.owner != MotionOwner::IDLE_HOLD || !idleLease.IsValid() ||
        executionEpoch == MOTION_EXECUTION_EPOCH_INVALID || executionEpoch != m_motion.GetCurrentExecutionEpoch() ||
        alarms.HasAlarm() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        safety != m_motion.GetSafetyProvenanceGeneration() || alarmUpdate != alarms.GetUpdateCount() ||
        alarmIntent != AlarmManager::MotionAdmissionBaseState(alarms.GetMotionSafetyIntentState())) return;
    s.normalEndCommitted = true;
    s.completionIdleLease = idleLease;
    s.completionEpoch = executionEpoch;
    s.completionSafetyProvenance = safety;
    s.completionAlarmUpdate = alarmUpdate;
    s.completionAlarmSafetyIntent = alarmIntent;
    s.completionReleaseRequested = m_motion.RequestEDM94CompletionRelease(s.session, s.armEpoch,
        s.lease, executionEpoch, idleLease, s.completionNoneGeneration);
    LogEDM94SameThread(s.completionReleaseRequested ? "NORMAL_RELEASE_REQUEST" : "NORMAL_RELEASE_REJECTED",
        s.completionReleaseRequested ? "M30_COMMITTED_WAIT_NONE_PHYSICAL_ZERO" : "RESET_REQUIRED");
}

void NCManager::RequestEDM118HoldSameThread(bool targetReached, const char* reason) noexcept
{
    auto& s = m_edm94; auto& c = m_edm118;
    if (!s.gapModeInterruption || !s.latched || c.requested) return;
    c.requested = true; c.targetMissed = !targetReached;
    c.session = s.session; c.run = s.run; c.cache = s.cache;
    c.armEpoch = s.armEpoch; c.ownerGeneration = s.lease.generation;
    c.polls = 0U; c.lastFeedbackTick = 0ULL;
    // The separate request survives CancelEDM94's ordinary ledger invalidation.
    // Cancellation publishes ForceZero before diagnostic formatting or HOLD.
    const bool previousReturn = c.armP == EDM121::ArmP;
    const bool arcRetreat = c.armP == EDM120::ArmP;
    const bool retreat = c.armP == EDM119::ArmP;
    CancelEDM94SameThread(c.armP == EDM125::RetreatArmP ? (targetReached ? "EDM125_B2_PREVIOUS_ARC_RETREAT_HOLD" : "EDM125_TARGET_MISSED") :
        c.armP == EDM125::ReturnArmP ? (targetReached ? "EDM125_B2_PREVIOUS_ARC_RETURN_HOLD" : "EDM125_TARGET_MISSED") :
        c.armP == EDM123::B0ArmP ? (targetReached ? "EDM123_B0_TANGENT_RETURN_HOLD" : "EDM123_TARGET_MISSED") :
        c.armP == EDM123::B1ArmP ? (targetReached ? "EDM123_B1_Z_RETURN_HOLD" : "EDM123_TARGET_MISSED") :
        c.armP == EDM122::B0ArmP ? (targetReached ? "EDM122_B0_TANGENT_RETREAT_HOLD" : "EDM122_TARGET_MISSED") :
        c.armP == EDM122::B1ArmP ? (targetReached ? "EDM122_B1_Z_RETREAT_HOLD" : "EDM122_TARGET_MISSED") :
        previousReturn ? (targetReached ? "EDM121_B2_PREVIOUS_RETURN_HOLD" : "EDM121_TARGET_MISSED") :
        arcRetreat ? (targetReached ? "EDM120_B2_ARC_RETREAT_HOLD" : "EDM120_TARGET_MISSED") :
        retreat ? (targetReached ? "EDM119_B2_RETREAT_HOLD" : "EDM119_TARGET_MISSED") :
        (targetReached ? "EDM118_B2_RETURN_HOLD" : "EDM118_TARGET_MISSED"));
    FeedHold();
    LogEDM118SameThread(targetReached ? "HOLD_REQUEST" : "TARGET_MISSED", reason);
    if (!targetReached)
    {
        c.terminalLogged = true;
        LogEDM118SameThread("STOP_UNPROVEN_RESET_REQUIRED", "TARGET_NOT_OBSERVED");
    }
}

bool NCManager::ServiceEDM118InterruptionSameThread(bool freshPhysical) noexcept
{
    auto& s = m_edm94; auto& c = m_edm118;
    if (!s.gapModeInterruption || !s.latched) return false;
    const auto arcProfile = EDM125::ProfileForArm(s.gapModeInterruptionArm);
    const bool previousArc = EDM125::ValidProfile(arcProfile);
    const auto localReturnProfile = EDM123::ProfileForArm(s.gapModeInterruptionArm);
    const bool localReturn = EDM123::ValidProfile(localReturnProfile);
    const auto localProfile = EDM122::ProfileForArm(s.gapModeInterruptionArm);
    const bool localRetreat = EDM122::ValidProfile(localProfile);
    const bool previousReturn = s.gapModeInterruptionArm == EDM121::ArmP;
    const bool arcRetreat = s.gapModeInterruptionArm == EDM120::ArmP;
    const bool retreat = s.gapModeInterruptionArm == EDM119::ArmP;
    const unsigned targetStep = previousArc ? EDM125::TargetStep : localReturn ? EDM123::TargetStep(localReturnProfile) : localRetreat ? EDM122::TargetStep(localProfile) : previousReturn ? EDM121::TargetStep : arcRetreat ? EDM120::TargetStep : retreat ? EDM119::TargetStep : EDM118::TargetStep;
    const unsigned targetSpan = previousArc ? EDM125::TargetSpan(arcProfile) : localReturn ? EDM123::TargetSpan : localRetreat ? EDM122::TargetSpan : previousReturn ? EDM121::TargetSpan : arcRetreat ? EDM120::TargetSpan : retreat ? EDM119::TargetSpan : EDM118::TargetSpan;
    const auto targetFixture = previousArc ? EDM125::TargetFixture(arcProfile) : localReturn ? EDM123::TargetFixture : localRetreat ? EDM122::TargetFixture : previousReturn ? EDM121::TargetFixture : arcRetreat ? EDM120::TargetFixture : retreat ? EDM119::TargetFixture : EDM118::TargetFixture;
    const bool profileCurrent = (previousArc || localReturn || localRetreat || previousReturn || arcRetreat || retreat || s.gapModeInterruptionArm == EDM118::ArmP) &&
        s.gapModeFixture == targetFixture;
    const auto& guard = s.feedback;
    // Only the two authored B2 trigger steps can index retained tokens.
    const unsigned tokenOrdinal = EDM115::EventOrdinal(targetStep);
    const bool targetTokenValid = (targetStep == 2U || targetStep == 4U) &&
        tokenOrdinal > 0U && tokenOrdinal <= m_edm116.tokens.size();
    const unsigned tokenIndex = targetTokenValid ? tokenOrdinal - 1U : 0U;
    // RESET release remains the existing exact successful Reset/rebase transaction.
    if (freshPhysical && profileCurrent && s.gapModeInterruptionArm == c.armP && !c.releasedLogged && s.resetSeen && s.resetCommitted && s.releaseRequested &&
        guard.identity.session == c.session && guard.identity.epoch == c.armEpoch &&
        guard.identity.ownerGeneration == c.ownerGeneration && guard.lastTick && guard.lastTick >= s.lastTick &&
        guard.released && !guard.active && guard.cancelled && guard.stopped && guard.forceZero && !guard.faulted &&
        guard.settledCycles >= EDM94::RequiredStableCycles && guard.settledCycles >= guard.requiredCycles)
    {
        c.releasedLogged = true;
        LogEDM118SameThread("RESET_RELEASED", c.stopPassed ? "EXACT_RESET_RELEASE_AFTER_STOP_PASS" : "EXACT_RESET_RELEASE_TARGET_UNPROVEN");
    }
    if (!c.requested)
    {
        if (c.terminalLogged || s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested) return false;
        const bool live = freshPhysical && profileCurrent && s.gapModeInterruptionArm == c.armP && s.gapModeFlush && s.gapPhysicalCurve && !s.gapCurveB2 &&
            s.gapModeFixture == targetFixture && s.ready && s.everArmed && !s.armPending &&
            !s.finishPending && !s.complete && !s.rtFaultReported && !s.normalEndCommitted &&
            !s.completionReleaseRequested && !s.gapWaiting && m_state == NCState::RUN &&
            m_mode == NCOperationMode::MEMORY && !m_isSingleBlockEnabled && !Close_System_Com_flag &&
            m_externalReadyInterlock && m_edmState != EDMState::NOT_READY &&
            !AlarmManager::GetInstance().HasAlarm() && !Homing.IsActive() && !m_isG66Active && m_macroStack.empty() &&
            m_resetContinuationPhase == ResetContinuationPhase::IDLE &&
            s.run == m_pathCoreLiveBookkeeping.currentRunToken && s.cache == GetBaseProgramCache().GetGeneration() &&
            s.lease.Matches(m_programMotionLease) && m_motion.IsMotionOwnerLeaseCurrent(s.lease) &&
            !m_motion.HasPendingSafetyOrRecoveryRequests() &&
            CoordSys.IsTranslationRunFrozen() && CoordSys.IsTranslationRunCurrent() &&
            IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) &&
            CoordSys.GetTranslationSnapshot().distanceMode == 91 && CoordSys.activePlane == 17 &&
            IsXYZCUVFeedProfileCurrentSameThread() && IsFixedTranslationTravelCurrentSameThread() &&
            IsPathCoreLiveNativeConfigCurrentSameThread() &&
            s.feedback.active && s.feedback.armAccepted && !s.feedback.faulted && !s.feedback.cancelled &&
            !s.feedback.forceZero && !s.feedback.released;
        if (!live) return false; // Ordinary lifecycle guards retain stop/ALARM authority.
        if (s.next > targetStep + 1U)
        { RequestEDM118HoldSameThread(false, "TARGET_ROW_PASSED"); return true; }
        if (s.next != targetStep + 1U) return false;
        EDM116::MotionFeedback mode{}; EDM96::Feedback gap{};
        if (!m_motion.ReadEDM116Feedback(mode) || !m_motion.ReadEDM96Feedback(gap)) return false;
        const auto& policy = m_edm116.policies[targetStep];
        const auto& source = mode.trigger; const auto& sample = mode.excursion;
        const bool sourceCurrent = mode.enabled && mode.fixture == targetFixture &&
            mode.session == s.session && mode.armEpoch == s.armEpoch && mode.ownerGeneration == s.lease.generation &&
            mode.authorizedRows == targetStep + 1U && mode.completedRows == targetStep &&
            mode.eventsStarted == EDM116::EventOrdinal(targetStep) &&
            mode.eventsCompleted + 1U == mode.eventsStarted &&
            EDM116::SamePolicy(mode.policy, policy) && EDM116::SamePolicy(mode.rowPolicy, policy) &&
            EDM116::PolicyMatchesSource(policy, source) && EDM114::SameSource(mode.rowSource, source) &&
            source.scope.session == s.session && source.scope.run == s.run && source.scope.cache == s.cache &&
            source.scope.ownerGeneration == s.lease.generation && source.step == targetStep &&
            source.sourcePC == s.pc[targetStep] && source.dispatch == m_edm116.issuedDispatch[targetStep] &&
            source.epoch == m_edm116.issuedEpoch[targetStep] && source.epoch == m_motion.GetCurrentExecutionEpoch() &&
            source.scope.translationGeneration == CoordSys.GetTranslationSnapshot().generation &&
            ((localReturn || localRetreat) ? (EDM114::SameSource(mode.previous.previous, EDM109::Source{}) &&
                EDM116::EmptyToken(mode.previous)) :
                (targetTokenValid && EDM114::SameSource(mode.previous.previous, m_edm114.retained.source) &&
                    EDM115::SameToken(mode.previous, m_edm116.tokens[tokenIndex]) &&
                    (!previousArc || (mode.tokenValidated && mode.geometryCurrent &&
                        mode.currentKind == EDM114::Kind::Arc && mode.previousKind == EDM114::Kind::Arc &&
                        m_edm114.retained.kind == EDM114::Kind::Arc &&
                        m_edm116.previousRows[tokenIndex].kind == EDM114::Kind::Arc &&
                        EDM114::SameSource(m_edm116.previousRows[tokenIndex].source, mode.previous.previous) &&
                        mode.previousBoundaryTick == m_edm116.tokens[tokenIndex].completion.tick &&
                        mode.previousCommit == m_edm116.tokens[tokenIndex].completion.commit)))) &&
            m_edm116.selected == targetStep + 1U && m_edm116.consumed == targetStep + 1U && !m_edm116.pending &&
            m_edm116.acknowledgedRows == targetStep && m_edm114.acknowledgedRows == targetStep &&
            gap.session == s.session && gap.epoch == source.epoch && gap.ownerGeneration == s.lease.generation &&
            gap.expectedStep == targetStep + 1U && gap.acceptedSegments == targetStep + 1ULL &&
            gap.completedSegments == targetStep && gap.segmentActive && !gap.segmentCompleted &&
            gap.active && !gap.faulted && gap.reason == EDM96::Reason::None && IsEDM97TuningCurrentSameThread(gap);
        // The mode event has not begun during the first half of this ordinary row.
        // Do not treat the preceding event's retained record as this event's target.
        if (!sourceCurrent)
        {
            if (mode.completedRows > targetStep || mode.authorizedRows > targetStep + 1U ||
                (mode.trigger.step == targetStep && mode.eventsStarted == EDM116::EventOrdinal(targetStep) && mode.excursion.geometryValid))
            { RequestEDM118HoldSameThread(false, "TARGET_SOURCE_OR_COUNTS"); return true; }
            return false;
        }
        if (previousArc ? EDM125::RequestWindow(arcProfile, policy, source, sample) : localReturn ? EDM123::RequestWindow(localReturnProfile, policy, source, sample) : localRetreat ? EDM122::RequestWindow(localProfile, policy, source, sample) :
            previousReturn ? EDM121::RequestWindow(policy, source, sample) :
            arcRetreat ? EDM120::RequestWindow(policy, source, sample) :
            retreat ? EDM119::RequestWindow(policy, source, sample) : EDM118::RequestWindow(policy, source, sample))
        {
            c.requestedPolicy = policy; c.requestedSource = source;
            c.requestedPrevious = mode.previous.previous; c.requestSample = sample;
            RequestEDM118HoldSameThread(true, previousArc ? (arcProfile == EDM125::Profile::PreviousArcRetreat ?
                "B2_PREVIOUS_G03_ARC_RETREAT_SPAN1_PROGRESS_25_50" : "B2_PREVIOUS_G02_ARC_RETURN_SPAN2_PROGRESS_25_50") : localReturnProfile == EDM123::Profile::B0TangentReturn ? "B0_G03_LOCAL_RETURN_PROGRESS_25_50" :
                localReturnProfile == EDM123::Profile::B1ZReturn ? "B1_G02_Z_RETURN_PROGRESS_25_50" : localProfile == EDM122::Profile::B0TangentRetreat ? "B0_G03_LOCAL_RETREAT_PROGRESS_25_50" :
                localProfile == EDM122::Profile::B1ZRetreat ? "B1_G02_Z_RETREAT_PROGRESS_25_50" : previousReturn ? "B2_G02_PREVIOUS_RETURN_SPAN2_PROGRESS_25_50" : arcRetreat ? "B2_G02_RETREAT_SPAN0_PROGRESS_25_50" :
                retreat ? "B2_G02_RETREAT_SPAN1_PROGRESS_25_50" : "B2_G03_RETURN_SPAN3_PROGRESS_25_50");
            return true;
        }
        const bool missed = sample.faulted || sample.stopRequested || sample.stopped || sample.complete ||
            sample.span > targetSpan ||
            (sample.span == targetSpan &&
                (!std::isfinite(sample.progress) || !std::isfinite(sample.length) || sample.length <= 0.0 ||
                    sample.progress > sample.length * (previousArc ? EDM125::RequestProgressEnd : localReturn ? EDM123::RequestProgressEnd : localRetreat ? EDM122::RequestProgressEnd : previousReturn ? EDM121::RequestProgressEnd : arcRetreat ? EDM120::RequestProgressEnd : retreat ? EDM119::RequestProgressEnd : EDM118::RequestProgressEnd) ||
                    sample.phase == EDM115::Phase::EndpointSettle || sample.phase == EDM115::Phase::Complete));
        if (missed) { RequestEDM118HoldSameThread(false, previousArc ? "PREVIOUS_ARC_WINDOW_PASSED" : localReturn ? "LOCAL_RETURN_WINDOW_PASSED" : localRetreat ? "LOCAL_RETREAT_WINDOW_PASSED" : previousReturn ? "PREVIOUS_RETURN_WINDOW_PASSED" : arcRetreat ? "ARC_RETREAT_WINDOW_PASSED" : retreat ? "RETREAT_WINDOW_PASSED" : "RETURN_WINDOW_PASSED"); return true; }
        return false;
    }

    if (c.terminalLogged) return false;
    const auto fail = [&](const char* reason) noexcept
    {
        c.terminalLogged = true; c.stopPassed = false;
        LogEDM118SameThread("STOP_UNPROVEN_RESET_REQUIRED", reason);
    };
    if (!profileCurrent || s.gapModeInterruptionArm != c.armP || c.targetMissed || c.resetInterrupted || s.resetSeen || s.resetCommitted || s.releaseRequested ||
        s.session != c.session || s.armEpoch != c.armEpoch || s.lease.generation != c.ownerGeneration ||
        s.run != c.run || s.cache != c.cache || s.next != targetStep + 1U ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !s.cancelled || s.ready || s.complete || m_state != NCState::HOLD || m_mode != NCOperationMode::MEMORY ||
        Close_System_Com_flag || !m_externalReadyInterlock || m_edmState == EDMState::NOT_READY ||
        Homing.IsActive() || m_isG66Active || !m_macroStack.empty() || m_isSingleBlockEnabled ||
        AlarmManager::GetInstance().HasAlarm() || s.rtFaultReported)
    { fail("LIFECYCLE_CHANGED_BEFORE_STOP_PROOF"); return false; }
    if (++c.polls > 1000U) { fail("STOP_PROOF_WAIT_LIMIT"); return false; }
    EDM118::Feedback observed{};
    if (!m_motion.ReadEDM118Feedback(observed)) return false;
    if (observed.publishedFailure || (observed.lastTick && observed.lastTick < c.lastFeedbackTick))
    { c.feedback = observed; fail("RT_CAPTURE_OR_CLOCK_FAILURE"); return false; }
    if (!observed.first.captured) return false;
    if (c.feedback.first.captured)
    {
        const auto& before = c.feedback.first; const auto& after = observed.first;
        if (!EDM116::SamePolicy(before.policy, after.policy) ||
            !EDM114::SameSource(before.trigger, after.trigger) || !EDM114::SameSource(before.previous, after.previous) ||
            !E115SameExcursion(before.initial, after.initial) || before.commandPulse != after.commandPulse ||
            before.commandVelocity != after.commandVelocity || before.armEpoch != after.armEpoch ||
            before.mappedTick != after.mappedTick || before.captureTick != after.captureTick || before.captureUs != after.captureUs ||
            before.safetyEpoch != after.safetyEpoch || before.safetyOwnerGeneration != after.safetyOwnerGeneration ||
            before.safetyTicket != after.safetyTicket || before.authorizedRows != after.authorizedRows ||
            before.completedRows != after.completedRows || before.eventsStarted != after.eventsStarted ||
            before.eventsCompleted != after.eventsCompleted)
        { fail("FIRST_COMMITTED_RECEIPT_CHANGED"); return false; }
    }
    c.feedback = observed;
    c.lastFeedbackTick = observed.lastTick;
    const auto& f = c.feedback; const auto& first = f.first;
    if (!(previousArc ? EDM125::TargetReceipt(arcProfile, first) : localReturn ? EDM123::TargetReceipt(localReturnProfile, first) : localRetreat ? EDM122::TargetReceipt(localProfile, first) : previousReturn ? EDM121::TargetReceipt(first) : arcRetreat ? EDM120::TargetReceipt(first) : retreat ? EDM119::TargetReceipt(first) : EDM118::TargetReceipt(first)) ||
        !(previousArc ? EDM125::RequestWindow(arcProfile, c.requestedPolicy, c.requestedSource, c.requestSample) : localReturn ? EDM123::RequestWindow(localReturnProfile, c.requestedPolicy, c.requestedSource, c.requestSample) : localRetreat ? EDM122::RequestWindow(localProfile, c.requestedPolicy, c.requestedSource, c.requestSample) :
            previousReturn ? EDM121::RequestWindow(c.requestedPolicy, c.requestedSource, c.requestSample) :
            arcRetreat ? EDM120::RequestWindow(c.requestedPolicy, c.requestedSource, c.requestSample) :
            retreat ? EDM119::RequestWindow(c.requestedPolicy, c.requestedSource, c.requestSample) :
            EDM118::RequestWindow(c.requestedPolicy, c.requestedSource, c.requestSample)) ||
        !EDM116::SamePolicy(first.policy, c.requestedPolicy) ||
        !EDM114::SameSource(first.trigger, c.requestedSource) || !EDM114::SameSource(first.previous, c.requestedPrevious) ||
        f.session != c.session || f.armEpoch != c.armEpoch || f.ownerGeneration != c.ownerGeneration ||
        first.armEpoch != c.armEpoch || first.mappedTick < c.requestSample.tick ||
        first.captureTick < first.mappedTick || first.captureUs < c.requestSample.nowUs ||
        first.initial.nowUs < c.requestSample.nowUs || first.initial.startTick != c.requestSample.startTick ||
        first.initial.startUs != c.requestSample.startUs || first.initial.progress < c.requestSample.progress ||
        (previousArc ? !EDM125::ProgressContinues(arcProfile, c.requestSample, first.initial) : localReturn ? !EDM123::ProgressContinues(c.requestSample, first.initial) : localRetreat ? !EDM122::ProgressContinues(c.requestSample, first.initial) :
            previousReturn ? !EDM121::ProgressContinues(c.requestSample, first.initial) :
            arcRetreat ? !EDM120::ProgressContinues(c.requestSample, first.initial) :
            retreat ? !EDM119::ProgressContinues(c.requestSample, first.initial) : first.initial.q < c.requestSample.q) ||
        !EDM109::SameDouble(first.initial.length, c.requestSample.length) ||
        !EDM109::SameDouble(first.initial.maximumVelocity, c.requestSample.maximumVelocity) ||
        !EDM109::SameDouble(first.initial.acceleration, c.requestSample.acceleration) ||
        !EDM109::SameDouble(first.initial.deceleration, c.requestSample.deceleration) ||
        f.lastTick < first.captureTick || f.safetyEpoch != first.safetyEpoch ||
        f.safetyOwnerGeneration != first.safetyOwnerGeneration || f.safetyTicket != first.safetyTicket)
    { fail("FIRST_COMMITTED_TARGET_NOT_PROVEN"); return false; }
    if (!f.stopAccepted || !f.stopped) return false;
    const auto& final = f.final;
    if (!final.geometryValid || !final.stopRequested || !final.stopped || final.faulted || final.active || final.moving ||
        final.complete || (previousArc && (final.endpointReady || final.progress <= 0.0 || final.progress >= final.length)) || (localReturn && (final.endpointReady || final.progress >= final.length || final.q <= 0.0)) ||
        final.phase != EDM115::Phase::Stopped || final.reason != EDM115::Reason::None ||
        final.span != targetSpan || final.direction != (previousArc ? EDM125::TargetDirection(arcProfile) : localReturn ? EDM123::TargetDirection : localRetreat ? EDM122::TargetDirection : previousReturn ? EDM121::TargetDirection : arcRetreat ? EDM120::TargetDirection : retreat ? EDM119::TargetDirection : 1) ||
        final.rowKind != (previousArc ? EDM125::TargetRowKind : localReturn ? EDM123::TargetRowKind : localRetreat ? EDM122::TargetRowKind : previousReturn ? EDM121::TargetRowKind : arcRetreat ? EDM120::TargetRowKind : retreat ? EDM119::TargetRowKind : EDM114::Kind::Arc) ||
        final.v != 0.0 || !std::isfinite(final.a) || final.completed != 0ULL || final.anchorReturned != 0ULL ||
        final.started != 1ULL || final.completedDwells != (previousArc ? EDM125::TargetDwells(arcProfile) : localReturn ? EDM123::TargetDwells : localRetreat ? EDM122::TargetDwells : previousReturn ? EDM121::TargetDwells : arcRetreat ? EDM120::TargetDwells : retreat ? EDM119::TargetDwells : 4U) ||
        final.startTick != first.initial.startTick || final.startUs != first.initial.startUs ||
        final.nowUs < first.captureUs || !std::isfinite(final.q) ||
        (previousArc ? !EDM125::ProgressContinues(arcProfile, first.initial, final) : localReturn ? !EDM123::ProgressContinues(first.initial, final) : localRetreat ? !EDM122::ProgressContinues(first.initial, final) :
            previousReturn ? !EDM121::ProgressContinues(first.initial, final) :
            arcRetreat ? !EDM120::ProgressContinues(first.initial, final) :
            retreat ? !EDM119::ProgressContinues(first.initial, final) : final.q < first.initial.q) ||
        !std::isfinite(final.progress) || final.progress < first.initial.progress || final.progress > final.length ||
        !EDM109::SameDouble(final.length, first.initial.length) ||
        (!previousArc && !localReturn && !previousReturn && !arcRetreat && !retreat && !EDM109::SameDouble(final.q, final.progress)) ||
        !EDM109::SameDouble(final.maximumVelocity, first.initial.maximumVelocity) ||
        !EDM109::SameDouble(final.acceleration, first.initial.acceleration) ||
        !EDM109::SameDouble(final.deceleration, first.initial.deceleration) ||
        final.backwardCrossings != (previousArc ? EDM125::TargetBackwardCrossings : localReturn ? EDM123::TargetBackwardCrossings : localRetreat ? EDM122::TargetBackwardCrossings : previousReturn ? EDM121::TargetBackwardCrossings : arcRetreat ? EDM120::TargetBackwardCrossings : retreat ? EDM119::TargetBackwardCrossings : 1ULL) ||
        final.forwardCrossings != (previousArc ? EDM125::TargetForwardCrossings : localReturn ? EDM123::TargetForwardCrossings : localRetreat ? EDM122::TargetForwardCrossings : previousReturn ? EDM121::TargetForwardCrossings : arcRetreat ? EDM120::TargetForwardCrossings : retreat ? EDM119::TargetForwardCrossings : 1ULL) ||
        !f.stoppedTick || f.stoppedTick != final.tick || f.stoppedTick < first.captureTick || f.lastTick < f.stoppedTick ||
        !EDM118::FinitePoint(f.stoppedPulse) || f.stoppedVelocity != EDM118::Point{})
    { fail("COMMITTED_EXECUTOR_STOP_NOT_PROVEN"); return false; }
    if (guard.faulted) { fail("PHYSICAL_GUARD_FAULT"); return false; }
    const auto owner = m_motion.GetMotionOwnerLease();
    const bool physical = freshPhysical && guard.identity.session == c.session && guard.identity.epoch == c.armEpoch &&
        guard.identity.ownerGeneration == c.ownerGeneration && guard.currentIdentity.session == c.session &&
        guard.currentIdentity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        guard.currentIdentity.ownerGeneration == owner.generation && guard.active && guard.armAccepted &&
        guard.cancelled && guard.forceZero && guard.stopped && !guard.pass && !guard.finishRequested && !guard.released &&
        guard.lastTick >= s.lastTick && guard.lastTick >= f.stoppedTick &&
        guard.requiredCycles >= EDM94::RequiredStableCycles && guard.settledCycles >= guard.requiredCycles &&
        guard.lastTransportSealed && guard.lastTransportAttempted && guard.lastTransportAccepted && guard.lastTransportAllZero &&
        guard.lastTransportTick == guard.lastSentTick && guard.lastSentTick == guard.lastTick &&
        m_motion.IsGroupDone() && !m_motion.HasPendingSafetyOrRecoveryRequests();
    if (!physical) return false;
    c.stopPassed = c.terminalLogged = true;
    LogEDM118SameThread("STOP_PASS_RESET_REQUIRED", "FIRST_COMMITTED_TARGET_AND_PHYSICAL_ZERO_200");
    return false;
}

void NCManager::LogEDM118SameThread(const char* event, const char* reason) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm118; const auto& f = c.feedback; const auto& r = f.first;
    const unsigned profileArm = s.latched ? c.armP : s.gapModeInterruptionArm;
    const bool previousReturn = profileArm == EDM121::ArmP;
    const bool arcRetreat = profileArm == EDM120::ArmP;
    const bool retreat = profileArm == EDM119::ArmP;
    char record[481]{};
    const auto emit = [&](int length) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("%s", (profileArm == EDM125::RetreatArmP ? "[EDM125] event=LOG_FORMAT_ERROR profile=PREVIOUS_ARC_RETREAT armP=154\n" : profileArm == EDM125::ReturnArmP ? "[EDM125] event=LOG_FORMAT_ERROR profile=PREVIOUS_ARC_RETURN armP=156\n" : profileArm == EDM123::B0ArmP ? "[EDM123] event=LOG_FORMAT_ERROR profile=B0 armP=146\n" : profileArm == EDM122::B0ArmP ? "[EDM122] event=LOG_FORMAT_ERROR profile=B0 armP=142\n" : profileArm == EDM123::B1ArmP ? "[EDM123] event=LOG_FORMAT_ERROR profile=B1 armP=148\n" : profileArm == EDM122::B1ArmP ? "[EDM122] event=LOG_FORMAT_ERROR profile=B1 armP=144\n" : previousReturn ? "[EDM121] event=LOG_FORMAT_ERROR\n" : arcRetreat ? "[EDM120] event=LOG_FORMAT_ERROR\n" : retreat ? "[EDM119] event=LOG_FORMAT_ERROR\n" : "[EDM118] event=LOG_FORMAT_ERROR\n"));
    };
    emit(std::snprintf(record, sizeof(record),
        (profileArm == EDM125::RetreatArmP ? "[EDM125] event=%.32s reason=%.48s session=%llu P=154 finishP=155 targetStep=4 targetSpan=1 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE profile=PREVIOUS_ARC_RETREAT armP=154\n" : profileArm == EDM125::ReturnArmP ? "[EDM125] event=%.32s reason=%.48s session=%llu P=156 finishP=157 targetStep=4 targetSpan=2 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE profile=PREVIOUS_ARC_RETURN armP=156\n" : profileArm == EDM123::B0ArmP ? "[EDM123] event=%.32s reason=%.48s session=%llu P=146 finishP=147 targetStep=6 targetSpan=1 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE profile=B0 armP=146\n" : profileArm == EDM122::B0ArmP ? "[EDM122] event=%.32s reason=%.48s session=%llu P=142 finishP=143 targetStep=6 targetSpan=0 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE profile=B0 armP=142\n" : profileArm == EDM123::B1ArmP ? "[EDM123] event=%.32s reason=%.48s session=%llu P=148 finishP=149 targetStep=10 targetSpan=1 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE profile=B1 armP=148\n" : profileArm == EDM122::B1ArmP ? "[EDM122] event=%.32s reason=%.48s session=%llu P=144 finishP=145 targetStep=10 targetSpan=0 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE profile=B1 armP=144\n" : previousReturn ? "[EDM121] event=%.32s reason=%.48s session=%llu P=140 finishP=141 targetStep=2 targetSpan=2 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE\n" : arcRetreat ? "[EDM120] event=%.32s reason=%.48s session=%llu P=138 finishP=139 targetStep=2 targetSpan=0 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE\n" : retreat ? "[EDM119] event=%.32s reason=%.48s session=%llu P=136 finishP=137 targetStep=2 targetSpan=1 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE\n" : "[EDM118] event=%.32s reason=%.48s session=%llu P=134 finishP=135 targetStep=2 targetSpan=3 requested=%u missed=%u stopPass=%u ncState=%u autoReset=0 autoStart=0 autoReturn=0 dischargeCommand=NONE\n"),
        event ? event : "UNKNOWN", reason ? reason : "NONE", static_cast<unsigned long long>(s.session),
        c.requested ? 1U : 0U, c.targetMissed ? 1U : 0U, c.stopPassed ? 1U : 0U,
        static_cast<unsigned>(static_cast<NCState>(m_state))));
    emit(std::snprintf(record, sizeof(record),
        (profileArm == EDM125::RetreatArmP ? "[EDM125] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g profile=PREVIOUS_ARC_RETREAT armP=154\n" : profileArm == EDM125::ReturnArmP ? "[EDM125] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g profile=PREVIOUS_ARC_RETURN armP=156\n" : profileArm == EDM123::B0ArmP ? "[EDM123] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g profile=B0 armP=146\n" : profileArm == EDM122::B0ArmP ? "[EDM122] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g profile=B0 armP=142\n" : profileArm == EDM123::B1ArmP ? "[EDM123] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g profile=B1 armP=148\n" : profileArm == EDM122::B1ArmP ? "[EDM122] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g profile=B1 armP=144\n" : previousReturn ? "[EDM121] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g\n" : arcRetreat ? "[EDM120] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g\n" : retreat ? "[EDM119] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g\n" : "[EDM118] event=HOLD_SOURCE session=%llu requestTick=%llu requestEpoch=%llu requestDispatch=%llu mappedTick=%llu captureTick=%llu stopTick=%llu captured=%u stopAccepted=%u captureFailure=%u actualSpan=%u actualDirection=%d actualV=%.12g\n"),
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(c.requestSample.tick),
        static_cast<unsigned long long>(c.requestedSource.epoch), static_cast<unsigned long long>(c.requestedSource.dispatch),
        static_cast<unsigned long long>(r.mappedTick), static_cast<unsigned long long>(r.captureTick),
        static_cast<unsigned long long>(f.stoppedTick), r.captured ? 1U : 0U, f.stopAccepted ? 1U : 0U,
        f.publishedFailure ? 1U : 0U, r.initial.span, r.initial.direction, r.initial.v));
    emit(std::snprintf(record, sizeof(record),
        (profileArm == EDM125::RetreatArmP ? "[EDM125] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0 profile=PREVIOUS_ARC_RETREAT armP=154\n" : profileArm == EDM125::ReturnArmP ? "[EDM125] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0 profile=PREVIOUS_ARC_RETURN armP=156\n" : profileArm == EDM123::B0ArmP ? "[EDM123] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0 profile=B0 armP=146\n" : profileArm == EDM122::B0ArmP ? "[EDM122] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0 profile=B0 armP=142\n" : profileArm == EDM123::B1ArmP ? "[EDM123] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0 profile=B1 armP=148\n" : profileArm == EDM122::B1ArmP ? "[EDM122] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0 profile=B1 armP=144\n" : previousReturn ? "[EDM121] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0\n" : arcRetreat ? "[EDM120] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0\n" : retreat ? "[EDM119] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0\n" : "[EDM118] event=HOLD_PROOF session=%llu executorStopped=%u physicalStopped=%u zero=%u stable=%u required=%u guardTick=%llu transportZero=%u faulted=%u resetSeen=%u resetCommitted=%u released=%u proofScope=COMMITTED_EXECUTOR_AND_PHYSICAL_ZERO_ONLY physicalDecelProven=0\n"),
        static_cast<unsigned long long>(s.session), f.stopped ? 1U : 0U, s.feedback.stopped ? 1U : 0U,
        s.feedback.forceZero ? 1U : 0U, s.feedback.settledCycles, s.feedback.requiredCycles,
        static_cast<unsigned long long>(s.feedback.lastTick), s.feedback.lastTransportAllZero ? 1U : 0U,
        s.feedback.faulted ? 1U : 0U, s.resetSeen ? 1U : 0U, s.resetCommitted ? 1U : 0U,
        s.feedback.released ? 1U : 0U));
}

bool NCManager::ServiceEDM106PhaseHoldSameThread(const EDM96::Feedback& gap) noexcept
{
    auto& s = m_edm94;
    if (!s.gapFlushInterruptArm || s.gapFlushInterruptRequested) return false;
    const bool fixture108 = s.gapFlushParameterized && (s.gapFlushInterruptArm == 111U || s.gapFlushInterruptArm == 113U);
    const bool selected = fixture108 || (!s.gapFlushParameterized &&
        (s.gapFlushInterruptArm == 105U || s.gapFlushInterruptArm == 107U));
    const bool live = selected && s.gapFlushProfile && s.latched && s.ready && s.everArmed &&
        !s.armPending && !s.finishPending && !s.complete && !s.cancelled && !s.resetSeen &&
        !s.resetCommitted && !s.releaseRequested && !s.rtFaultReported && !s.normalEndCommitted &&
        !s.completionReleaseRequested && !s.gapWaiting && m_state == NCState::RUN &&
        m_mode == NCOperationMode::MEMORY && !m_isSingleBlockEnabled && !Close_System_Com_flag &&
        m_externalReadyInterlock && m_edmState != EDMState::NOT_READY &&
        !AlarmManager::GetInstance().HasAlarm() && !Homing.IsActive() && !m_isG66Active && m_macroStack.empty() &&
        m_resetContinuationPhase == ResetContinuationPhase::IDLE &&
        s.run == m_pathCoreLiveBookkeeping.currentRunToken && s.cache == GetBaseProgramCache().GetGeneration() &&
        s.lease.Matches(m_programMotionLease) && m_motion.IsMotionOwnerLeaseCurrent(s.lease) &&
        !m_motion.HasPendingSafetyOrRecoveryRequests() &&
        s.feedback.active && s.feedback.armAccepted && !s.feedback.faulted && !s.feedback.cancelled &&
        !s.feedback.forceZero && !s.feedback.released &&
        gap.session == s.session && gap.epoch == m_motion.GetCurrentExecutionEpoch() &&
        gap.ownerGeneration == s.lease.generation && gap.tick != 0ULL && gap.nowUs != 0ULL &&
        gap.active && !gap.faulted && gap.reason == EDM96::Reason::None && IsEDM97TuningCurrentSameThread(gap);
    if (!live) return false; // Ordinary EDM94 guards retain all stop/ALARM authority.
    const EDM105::Phase target = (s.gapFlushInterruptArm == 105U || s.gapFlushInterruptArm == 111U) ?
        EDM105::Phase::Retreat : EDM105::Phase::Return;
    const bool missed = gap.expectedStep > 1U || gap.acceptedSegments > 1ULL || gap.completedSegments > 0ULL ||
        (gap.acceptedSegments == 1ULL && static_cast<unsigned>(gap.flushPhase) > static_cast<unsigned>(target));
    const bool moving = target == EDM105::Phase::Retreat ? gap.v < 0.0 : gap.v > 0.0;
    const bool reached = !missed && gap.expectedStep == 1U && gap.acceptedSegments == 1ULL &&
        gap.completedSegments == 0ULL && gap.segmentActive && !gap.segmentCompleted &&
        s.next == 1U && gap.flushStarted == 1ULL && gap.flushCompleted == 0ULL &&
        gap.flushPhase == target && std::isfinite(gap.q) && std::isfinite(gap.v) && moving;
    if (!missed && !reached) return false;
    s.gapFlushInterruptRequested = true;
    s.gapFlushInterruptSampleTick = gap.tick; s.gapFlushInterruptEpoch = gap.epoch;
    // Publish cancellation before diagnostic output can delay the target phase.
    // Cancel first also keeps FeedHold's MANUAL_HOLD path from counting a synthetic edge.
    CancelEDM94SameThread(fixture108 ? (reached ? "EDM108_PHASE_HOLD" : "EDM108_TARGET_MISSED") :
        (reached ? "EDM106_PHASE_HOLD" : "EDM106_TARGET_MISSED"));
    char record[481]{};
    const int length = std::snprintf(record, sizeof(record),
        "[%s] event=%s arm=%u session=%llu tick=%llu epoch=%llu step=%u target=%s sampledPhase=%s qPulse=%.12g vPulsePerSec=%.12g sampleOnly=1 consumeProof=RETAINED_RT_INTERRUPTION action=CANCEL_THEN_HOLD restart=RESET_REQUIRED dischargeCommand=NONE\n",
        fixture108 ? "EDM108" : "EDM106", reached ? "PHASE_HOLD_REQUEST" : "TARGET_MISSED", s.gapFlushInterruptArm,
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(gap.tick),
        static_cast<unsigned long long>(gap.epoch), gap.expectedStep, EDM105::PhaseName(target),
        EDM105::PhaseName(gap.flushPhase), gap.q, gap.v);
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[%s] event=LOG_FORMAT_ERROR scope=PHASE_HOLD_REQUEST\n", fixture108 ? "EDM108" : "EDM106");
    FeedHold();
    return true;
}

// The existing HMI variable transport carries only an explicitly scoped
// voltage write. No persistent @ value, START edge or axis command is polled.
bool NCManager::IsEDM169ManualScopeCurrentSameThread() const noexcept
{
    const auto& s = m_edm94;
    // After the frozen PASS, M30 retires AUTO through authenticated IDLE_HOLD
    // and NONE zero windows. A voltage heartbeat grants no new motion there.
    const bool completion = EDM149::IsManualVoltageTestArm(s.gapHistoryArm) && s.complete &&
        IsEDM94CompletionSelectionCurrentSameThread();
    const bool liveOwner = (m_state == NCState::RUN || m_state == NCState::HOLD) &&
        s.lease.Matches(m_programMotionLease) && m_motion.IsMotionOwnerLeaseCurrent(s.lease);
    return EDM149::IsManualVoltageTestArm(s.gapHistoryArm) && s.latched && s.session &&
        m_edm169.session == s.session && m_edm130.bound &&
        s.gapHistoryServo && s.gapHistoryMode && s.gapWorkTime && !s.gapPhysicalAd &&
        s.gapModeFixture == EDM116::Fixture::ModalProgram156 &&
        m_edm130.recipe.diagnosticControlSource == 5U && E156BoundPlanCurrent(s, m_edm130.recipe) &&
        !s.cancelled && !s.resetSeen && !s.resetCommitted && !s.releaseRequested && !s.rtFaultReported &&
        (liveOwner || completion) && m_mode == NCOperationMode::MEMORY &&
        !Close_System_Com_flag && m_externalReadyInterlock && !AlarmManager::GetInstance().HasAlarm() &&
        m_resetContinuationPhase == ResetContinuationPhase::IDLE &&
        s.run == m_pathCoreLiveBookkeeping.currentRunToken && s.cache == GetBaseProgramCache().GetGeneration() &&
        !m_motion.HasPendingSafetyOrRecoveryRequests() && s.feedback.identity.session == s.session &&
        s.feedback.identity.epoch == s.armEpoch && s.feedback.identity.ownerGeneration == s.lease.generation &&
        s.feedback.lastTick && s.feedback.lastNowUs && s.feedback.armAccepted &&
        !s.feedback.faulted && !s.feedback.cancelled && !s.feedback.released;
}

bool NCManager::QueueEDM169ManualVoltageSameThread(double voltageV, std::uint64_t inputSequence) noexcept
{
    if (!IsEDM169ManualScopeCurrentSameThread() || !EDM169::ValidVoltageV(voltageV) ||
        m_edm169.requestSequence == (std::numeric_limits<std::uint64_t>::max)()) return false;
    const auto& s = m_edm94;
    EDM169::Request request{};
    request.scope.session = s.session; request.scope.run = s.run; request.scope.cache = s.cache;
    request.scope.armEpoch = s.armEpoch; request.scope.ownerGeneration = s.lease.generation;
    request.scope.recipe = m_edm130.recipe;
    request.requestSequence = m_edm169.requestSequence + 1ULL;
    request.inputSequence = inputSequence; request.voltageV = voltageV;
    request.issuedTick = s.feedback.lastTick; request.issuedUs = s.feedback.lastNowUs;
    if (!EDM169::ValidRequest(request) || !m_motion.SubmitEDM169ManualVoltage(request)) return false;
    m_edm169.requestSequence = request.requestSequence;
    return true;
}

bool NCManager::SetEDM169ManualVoltageSameThread(double voltageV) noexcept
{
    const auto& s = m_edm94;
    if (!EDM149::IsManualVoltageTestArm(s.gapHistoryArm) || !s.latched) return false;
    const bool scope = IsEDM169ManualScopeCurrentSameThread() && s.ready && !s.armPending &&
        !s.finishPending && !s.complete;
    if (!scope || !EDM169::ValidVoltageV(voltageV) ||
        m_edm169.inputSequence == (std::numeric_limits<std::uint64_t>::max)())
    {
        RtPrintf("[EDM169] event=INPUT_REJECTED session=%llu variable=@990 voltageV=%.9g reason=%s previousValidV=%.9g inputSequence=%llu alarmRaised=0 motionAuthority=NONE\n",
            static_cast<unsigned long long>(s.session), voltageV,
            !scope ? "SCOPE_OR_LIFECYCLE" : "VOLTAGE_RANGE_OR_SEQUENCE", m_edm169.voltageV,
            static_cast<unsigned long long>(m_edm169.inputSequence));
        return false;
    }
    const auto inputSequence = m_edm169.inputSequence + 1ULL;
    if (!QueueEDM169ManualVoltageSameThread(voltageV, inputSequence)) return false;
    m_edm169.voltageV = voltageV; m_edm169.inputSequence = inputSequence;
    RtPrintf("[EDM169] event=INPUT_QUEUED session=%llu run=%llu cache=%llu epoch=%llu owner=%llu requestSequence=%llu inputSequence=%llu variable=@990 voltageV=%.9g source=SOFTWARE_MANUAL startAuthority=NONE S92=0 S93=0\n",
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(s.run),
        static_cast<unsigned long long>(s.cache), static_cast<unsigned long long>(s.armEpoch),
        static_cast<unsigned long long>(s.lease.generation), static_cast<unsigned long long>(m_edm169.requestSequence),
        static_cast<unsigned long long>(inputSequence), voltageV);
    return true;
}

void NCManager::ServiceEDM169ManualVoltageSameThread(bool feedbackCurrent) noexcept
{
    // A constant human value does not expire. The NC publisher heartbeat does,
    // and it remains required during HOLD and normal finish/M30 zero dwell.
    if (!feedbackCurrent || !QueueEDM169ManualVoltageSameThread(m_edm169.voltageV, m_edm169.inputSequence)) return;
    EDM169::Feedback applied{};
    const auto& s = m_edm94;
    if (!m_motion.ReadEDM169ManualVoltageFeedback(applied) || !applied.enabled || !applied.active || applied.faulted ||
        applied.scope.session != s.session || applied.scope.run != s.run || applied.scope.cache != s.cache ||
        applied.scope.armEpoch != s.armEpoch || applied.scope.ownerGeneration != s.lease.generation ||
        !EDM130::SameRecipe(applied.scope.recipe, m_edm130.recipe) || !applied.tick || !applied.nowUs ||
        !EDM169::ValidVoltageV(applied.voltageV) || applied.inputSequence > m_edm169.inputSequence ||
        (m_edm169.appliedLogged && applied.inputSequence <= m_edm169.loggedInputSequence)) return;
    m_edm169.appliedLogged = true; m_edm169.loggedInputSequence = applied.inputSequence;
    RtPrintf("[EDM169] event=INPUT_APPLIED session=%llu run=%llu cache=%llu epoch=%llu owner=%llu requestSequence=%llu inputSequence=%llu tick=%llu voltageV=%.9g publisherAgeUs=%llu source=RT_RECEIVER motion=EXISTING_CNC_GAP_SERVO S92=0 S93=0\n",
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(s.run),
        static_cast<unsigned long long>(s.cache), static_cast<unsigned long long>(s.armEpoch),
        static_cast<unsigned long long>(s.lease.generation), static_cast<unsigned long long>(applied.requestSequence),
        static_cast<unsigned long long>(applied.inputSequence), static_cast<unsigned long long>(applied.tick),
        applied.voltageV, static_cast<unsigned long long>(applied.publisherAgeUs));
}

void NCManager::ServiceEDM94SameThread()
{
    PollEDM206History(m_motion,static_cast<NCState>(m_state));
    if (!m_edm94.latched) return;
    auto& s = m_edm94;
    // EDM231 FIX3: an operator's recoverable G76 HOLD is still an active
    // session. Capture this before pause service can cancel its scope after a
    // new RT fault; that cancellation is not an operator STOP/RESET request.
    const bool operatorHoldFaultAlarm = m_state.load(std::memory_order_relaxed) == NCState::HOLD &&
        m_edm157.active && s.program && EDM173::IsArm(s.gapHistoryArm) &&
        !s.cancelled && !s.complete && !s.resetSeen && !s.resetCommitted &&
        !s.releaseRequested && !s.rtFaultReported &&
        m_resetContinuationPhase == ResetContinuationPhase::IDLE;
    // Sample before cancellation side effects. An RT fault may already have
    // revoked AUTO; logging the previous scan here would hide its first cause.
    EDM94::Feedback observed{};
    const bool feedbackRead = m_motion.ReadEDM94Feedback(observed);
    const bool sameSession = feedbackRead && observed.identity.session == s.session;
    const bool feedbackCurrent = sameSession && observed.lastTick >= s.lastTick &&
        observed.identity.epoch == s.armEpoch && observed.identity.ownerGeneration == s.lease.generation;
    // EDM118 never promotes a failed, rejected or repeated read to a new zero proof.
    // Preserve the general bracket's existing >= observer semantics below.
    const bool freshPhysical = feedbackCurrent && observed.lastTick > s.lastTick;
    if (feedbackCurrent)
    {
        s.feedback = observed;
        s.lastTick = observed.lastTick;
    }
    // EDM215: observe admission before the general fault handler can cancel
    // the scope. Readiness and RESET retirement still require guard receipts.
    if(s.armPending&&!s.cancelled&&EDM173::IsArm(s.gapHistoryArm)&&m_motion.IsMotionOwnerLeaseCurrent(s.lease))
    {
        const EDM192Arm::Scope admissionScope{s.session,static_cast<std::uint64_t>(s.armEpoch),
            static_cast<std::uint64_t>(m_motion.GetCurrentExecutionEpoch()),
            static_cast<std::uint32_t>(s.lease.owner),static_cast<std::uint32_t>(s.lease.generation),
            EDM173::HistoryFixture,m_edm130.recipe.commissioningProfile};
        if(PollEDM192AdmissionObserver(admissionScope))
        {RejectEDM94SameThread("EDM173_ARM_ADMISSION_REJECTED",m_gapDryRun.sourceLine);return;}
    }
    ServiceEDM169ManualVoltageSameThread(feedbackCurrent);
    if (EDM173::IsArm(s.gapHistoryArm) && feedbackCurrent) ServiceEDM173SameThread();
    if (s.gapWorkTime && !EDM173::IsArm(s.gapHistoryArm) && feedbackCurrent)
    {
        EDM128::MotionFeedback timed{};
        if (m_motion.ReadEDM130Feedback(timed) && timed.enabled && timed.workTimeFlush &&
            timed.session == s.session && EDM130::SameRecipe(timed.timerRecipe, m_edm130.recipe) &&
            (timed.coordinator.work.triggeredEvents != m_edm130.loggedTriggers ||
                timed.coordinator.work.completedEvents != m_edm130.loggedCompletions ||
                (E137FlushStageArm(s.gapHistoryArm) &&
                    (static_cast<std::uint32_t>(timed.coordinator.phase) != m_edm130.loggedFlushPhase ||
                        static_cast<std::uint32_t>(timed.coordinator.speedStage) != m_edm130.loggedSpeedStage))))
        {
            const bool timerEvent = timed.coordinator.work.triggeredEvents != m_edm130.loggedTriggers ||
                timed.coordinator.work.completedEvents != m_edm130.loggedCompletions;
            if (timerEvent) LogEDM130SameThread("TIMER_EVENT", EDM114::RequiredRows);
            else if (E137FlushStageArm(s.gapHistoryArm))
                EmitEDM137FlushStage(s.session, timed, "STAGE_CHANGED", true);
            m_edm130.loggedTriggers = timed.coordinator.work.triggeredEvents;
            m_edm130.loggedCompletions = timed.coordinator.work.completedEvents;
            m_edm130.loggedFlushPhase = static_cast<std::uint32_t>(timed.coordinator.phase);
            m_edm130.loggedSpeedStage = static_cast<std::uint32_t>(timed.coordinator.speedStage);
        }
    }
    // EDM237 FIX1 observes existing publications on the NC thread only.
    // Separate ticks are intentional: channel reads are not one atomic frame.
    // No inferred pair is promoted to physical evidence or control authority.
    if (feedbackCurrent && m_edm237.authorized && m_edm237.stageStarted &&
        m_edm237.parentIndex < m_edm237.parentCount && m_edm237.kinds[m_edm237.parentIndex] == 5U &&
        !s.cancelled && !s.releaseRequested &&
        (!m_edm237.b2TraceTick || observed.lastTick < m_edm237.b2TraceTick ||
            observed.lastTick - m_edm237.b2TraceTick >= 8000ULL))
    {
        m_edm237.b2TraceTick = observed.lastTick;
        EDM128::MotionFeedback timed{}; EDM126::MotionFeedback history{};
        const bool timerRead = m_motion.ReadEDM130Feedback(timed);
        const bool historyRead = m_motion.ReadEDM126Feedback(history);
        const bool identity = timerRead && historyRead && timed.enabled && history.enabled &&
            timed.session == s.session && history.session == s.session &&
            timed.armEpoch == s.armEpoch && history.armEpoch == s.armEpoch &&
            timed.ownerGeneration == s.lease.generation && history.ownerGeneration == s.lease.generation &&
            timed.liveEpoch == history.liveEpoch;
        if (!identity)
            PrintNCG76DiagnosticLine("[EDM237_FIX1] event=B2_TRACE_READ session=%llu tick=%llu timerRead=%u historyRead=%u identity=0 authority=NONE\n",
                static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(observed.lastTick),
                timerRead?1U:0U, historyRead?1U:0U);
        else if (timed.coordinator.policy.mode == EDM116::Mode::B2)
        {
            const auto& e = timed.coordinator; const auto& f = history.traversal;
            PrintNCG76DiagnosticLine("[EDM237_FIX1] event=B2_CURSOR session=%llu tick=%llu historyTick=%llu mappedTick=%llu phase=%s eventIndex=%llu row=%llu cursor=%llu depth=%u q=%.17g station=%.17g v=%.17g a=%.17g seam=%u stable=%u back=%llu forward=%llu fault=%u reason=%u authority=NONE\n",
                static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(observed.lastTick),
                static_cast<unsigned long long>(history.tick), static_cast<unsigned long long>(f.tick),
                EDM128::PhaseName(e.phase), static_cast<unsigned long long>(e.eventIndex),
                static_cast<unsigned long long>(f.authorizedSource.ordinal), static_cast<unsigned long long>(f.cursorSource.ordinal),
                f.historyDepth, f.localMm, f.stationMm, f.velocityMmSec, f.accelerationMmSec2,
                f.awaitingSeam?1U:0U, f.seamStableCycles,
                static_cast<unsigned long long>(f.backwardCrossings), static_cast<unsigned long long>(f.forwardCrossings),
                f.faulted?1U:0U, static_cast<unsigned>(f.reason));
            PrintNCG76DiagnosticLine("[EDM237_FIX1] event=B2_TARGET session=%llu timerTick=%llu phase=%s eventIndex=%llu captured=%u top=%u exactReturn=%u targetRow=%llu targetQ=%.17g targetStation=%.17g anchorRow=%llu anchorQ=%.17g anchorStation=%.17g targetSpeed=%.17g triggered=%llu completed=%llu authority=NONE\n",
                static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(timed.tick),
                EDM128::PhaseName(e.phase), static_cast<unsigned long long>(e.eventIndex),
                e.targetCaptured?1U:0U, e.topCaptured?1U:0U, e.exactReturn?1U:0U,
                static_cast<unsigned long long>(e.targetSource.ordinal), e.targetLocalMm, e.targetStationMm,
                static_cast<unsigned long long>(e.anchor.cursorSource.ordinal), e.anchor.localMm, e.anchor.stationMm,
                e.stageTargetMmSec, static_cast<unsigned long long>(e.work.triggeredEvents),
                static_cast<unsigned long long>(e.work.completedEvents));
        }
    }
    if (feedbackCurrent) ServiceEDM147PhysicalGapSameThread();
    if (ServiceEDM118InterruptionSameThread(freshPhysical)) return;
    if (feedbackCurrent && ConsumeEDM94CompletionReleaseSameThread(observed)) return;
    if (feedbackCurrent && observed.completionPhase == EDM94::CompletionReleasePhase::Revoked &&
        s.normalEndCommitted && !s.cancelled)
        CancelEDM94SameThread("NORMAL_RELEASE_REVOKED_RESET_REQUIRED");
    ServiceEDM200AutoFlushHoldSameThread(feedbackCurrent);
    ServiceEDM217AutoNominalHoldSameThread(feedbackCurrent);
    ServiceEDM218AutoCenterFlushHoldSameThread(feedbackCurrent);
    ServiceEDM165AutoFlushHoldSameThread(feedbackCurrent);
    const bool resumablePause = ServiceEDM157SameThread(feedbackCurrent);
    const bool preserveStop = s.resetSeen || s.resetCommitted || s.releaseRequested || s.complete ||
        m_state == NCState::HOLD || m_state == NCState::RESET_STATE ||
        m_resetContinuationPhase != ResetContinuationPhase::IDLE;
    if (!s.releaseRequested && !s.complete && !s.cancelled)
    {
        if (s.gapSegmentProfile)
        {
            if (m_isSingleBlockEnabled)
            { RejectEDM94SameThread("EDM96_CONTINUOUS_MODE_REQUIRED", m_gapDryRun.sourceLine); return; }
            if (s.gapWaiting && ++s.gapWaitPolls >=
                (EDM149::IsManualVoltageTestArm(s.gapHistoryArm) ? 3000U : 500U))
            { RejectEDM94SameThread("EDM96_SOURCE_WAIT_TIMEOUT", m_gapDryRun.sourceLine); return; }
            EDM96::Feedback gap{};
            if (m_motion.ReadEDM96Feedback(gap) && gap.session == s.session)
            {
                // EDM145: report the retained RT fault before a dependent
                // tuning validator rejects that already-faulted source.
                if (gap.faulted)
                {
                    LogEDM96SameThread("SOURCE_OR_SEGMENT_FAULT", gap);
                    if (m_state == NCState::RUN && !preserveStop)
                        RejectEDM94SameThread("EDM96_RT_SOURCE_OR_SEGMENT_FAULT", m_gapDryRun.sourceLine);
                    else CancelEDM94SameThread("EDM96_RT_FAULT_DURING_STOP");
                    return;
                }
                if (!IsEDM97TuningCurrentSameThread(gap))
                {
                    if (m_state == NCState::RUN && !preserveStop)
                        RejectEDM94SameThread(s.gapFlushProfile ? "EDM105_TUNING_IDENTITY" : s.gapPairedProfile ? "EDM104_TUNING_IDENTITY" : s.gapEnduranceProfile ? "EDM102_TUNING_IDENTITY" : s.gapContinuousProfile ? "EDM100_TUNING_IDENTITY" : s.gapBurstProfile ? "EDM99_TUNING_IDENTITY" : s.gapNoiseProfile ? "EDM98_TUNING_IDENTITY" :
                            "EDM97_TUNING_IDENTITY", m_gapDryRun.sourceLine);
                    else CancelEDM94SameThread(s.gapFlushProfile ? "EDM105_TUNING_IDENTITY_DURING_STOP" : s.gapPairedProfile ? "EDM104_TUNING_IDENTITY_DURING_STOP" : s.gapEnduranceProfile ? "EDM102_TUNING_IDENTITY_DURING_STOP" : s.gapContinuousProfile ? "EDM100_TUNING_IDENTITY_DURING_STOP" : s.gapBurstProfile ? "EDM99_TUNING_IDENTITY_DURING_STOP" : s.gapNoiseProfile ? "EDM98_TUNING_IDENTITY_DURING_STOP" :
                        "EDM97_TUNING_IDENTITY_DURING_STOP");
                    return;
                }
                if (ServiceEDM106PhaseHoldSameThread(gap)) return;
                if (s.gapWaiting && s.gapWaitFirstUs != 0ULL && gap.nowUs >= s.gapWaitFirstUs &&
                    gap.nowUs - s.gapWaitFirstUs >=
                        (EDM149::IsManualVoltageTestArm(s.gapHistoryArm) ? 30000000ULL : 5000000ULL))
                { RejectEDM94SameThread("EDM96_SOURCE_WAIT_TIMEOUT", m_gapDryRun.sourceLine); return; }
                const std::uint64_t phaseMark = s.gapFlushProfile ?
                    gap.acceptedSegments * 64ULL + gap.phase * 8ULL + static_cast<unsigned>(gap.flushPhase) :
                    gap.acceptedSegments * (s.gapGainProfile ? 8ULL : 4ULL) + gap.phase;
                if (phaseMark != s.gapLoggedPhase || gap.completedSegments != s.gapLoggedAccepted)
                {
                    LogEDM96SameThread("SEGMENT_PROGRESS", gap);
                    s.gapLoggedPhase = phaseMark; s.gapLoggedAccepted = gap.completedSegments;
                }
            }
        }
        if (s.gapBoundaryProfile && m_isSingleBlockEnabled)
        { RejectEDM94SameThread("EDM95_CONTINUOUS_MODE_REQUIRED", m_gapDryRun.sourceLine); return; }
        if (s.gapBoundaryProfile)
        {
            // Once a direction wait begins, its deadline is serviced on
            // every NC scan, including scans blocked by physical readiness.
            if (s.gapWaiting && ++s.gapWaitPolls >= 500U)
            { RejectEDM94SameThread("EDM95_GAP_WAIT_TIMEOUT", m_gapDryRun.sourceLine); return; }
            EDM95::Feedback gap{};
            if (m_motion.ReadEDM95Feedback(gap) && gap.session == s.session)
            {
                if (s.gapWaiting && s.gapWaitFirstUs != 0ULL && gap.nowUs >= s.gapWaitFirstUs &&
                    gap.nowUs - s.gapWaitFirstUs >= 5000000ULL)
                { RejectEDM94SameThread("EDM95_GAP_WAIT_TIMEOUT", m_gapDryRun.sourceLine); return; }
                if (gap.phaseGeneration != s.gapLoggedPhase)
                {
                    RtPrintf("[EDM95] event=GAP_SOURCE_PHASE session=%llu phase=%llu sourceSequence=%llu sourceUs=%llu voltage=%u direction=%s expectedStep=%u nextBoundaryOnly=1\n",
                        static_cast<unsigned long long>(gap.session), static_cast<unsigned long long>(gap.phaseGeneration),
                        static_cast<unsigned long long>(gap.sourceSequence), static_cast<unsigned long long>(gap.sourceCapturedUs),
                        gap.voltageV == 50.0 ? 50U : gap.voltageV == 20.0 ? 20U : 0U,
                        gap.direction > 0 ? "FORWARD" : gap.direction < 0 ? "RETREAT" : "QUALIFYING", gap.expectedStep);
                    s.gapLoggedPhase = gap.phaseGeneration;
                }
                if (gap.acceptedSegments != s.gapLoggedAccepted)
                {
                    RtPrintf("[EDM95] event=GAP_CONSUMED session=%llu accepted=%llu forward=%llu retreat=%llu expectedStep=%u epoch=%llu tick=%llu oneUse=1 output=AUTHORED_G1\n",
                        static_cast<unsigned long long>(gap.session), static_cast<unsigned long long>(gap.acceptedSegments),
                        static_cast<unsigned long long>(gap.forwardSegments), static_cast<unsigned long long>(gap.retreatSegments),
                        gap.expectedStep, static_cast<unsigned long long>(gap.epoch), static_cast<unsigned long long>(gap.tick));
                    s.gapLoggedAccepted = gap.acceptedSegments;
                }
            }
        }
        // Fixed diagnostic mask: close, mode, state, alarm, ready, cache, run, owner.
        const unsigned scopeFailures = (Close_System_Com_flag ? 1U : 0U) |
            (m_mode != NCOperationMode::MEMORY ? 2U : 0U) |
            ((m_state != NCState::RUN && !(m_state == NCState::HOLD && resumablePause)) ? 4U : 0U) |
            (AlarmManager::GetInstance().HasAlarm() ? 8U : 0U) |
            (!m_externalReadyInterlock ? 16U : 0U) |
            (s.cache != GetBaseProgramCache().GetGeneration() ? 32U : 0U) |
            (s.run != m_pathCoreLiveBookkeeping.currentRunToken ? 64U : 0U) |
            (!m_motion.IsMotionOwnerLeaseCurrent(s.lease) ? 128U : 0U);
        if (scopeFailures != 0U)
        {
            RtPrintf("[EDM94] event=CANCEL_SCOPE build=FIX16 session=%llu failures=%02X feedbackRead=%u sameSession=%u feedbackCurrent=%u observedTick=%llu cachedTick=%llu ncState=%u resetSeen=%u releaseRequested=%u\n",
                static_cast<unsigned long long>(s.session), scopeFailures, feedbackRead ? 1U : 0U,
                sameSession ? 1U : 0U, feedbackCurrent ? 1U : 0U,
                static_cast<unsigned long long>(observed.lastTick),
                static_cast<unsigned long long>(s.feedback.lastTick), static_cast<unsigned>(static_cast<NCState>(m_state)),
                s.resetSeen ? 1U : 0U, s.releaseRequested ? 1U : 0U);
            CancelEDM94SameThread("LIFECYCLE_OR_SCOPE");
        }
    }
    // A failed read must not suppress lifecycle cancellation.
    if (!feedbackRead) return;
    if (!sameSession)
    {
        if (!s.armPending && !s.cancelled && !preserveStop)
            RejectEDM94SameThread("FEEDBACK_SESSION", m_gapDryRun.sourceLine);
        return;
    }
    if (!feedbackCurrent)
    {
        // A stopped/recovering bracket cannot resume from rejected feedback.
        // Keep its existing stop transaction; an old sample must not relatch ALARM.
        if (!preserveStop) RejectEDM94SameThread("FEEDBACK_IDENTITY", m_gapDryRun.sourceLine);
        return;
    }
    const auto& f = s.feedback;
    s.everArmed = s.everArmed || f.armAccepted;
    if (f.faulted && !s.rtFaultReported)
    {
        // Cancellation is not acknowledgement of an RT fault. Report once even
        // if the preceding owner/lifecycle check (or an earlier scan) cancelled.
        s.rtFaultReported = true;
        // Do not use the now-cancelled pause scope as acknowledgement of this
        // fault. An actual RESET/release/completion observed meanwhile wins.
        const bool holdFaultAlarmCurrent = operatorHoldFaultAlarm &&
            (m_state == NCState::RUN || m_state == NCState::HOLD) &&
            !s.resetSeen && !s.resetCommitted && !s.releaseRequested && !s.complete &&
            m_resetContinuationPhase == ResetContinuationPhase::IDLE;
        const bool raiseAlarm = (m_state == NCState::RUN && !preserveStop) || holdFaultAlarmCurrent;
        if (!s.cancelled && !s.releaseRequested) CancelEDM94SameThread("RT_GUARD_FAULT");
        LogEDM94SameThread("RT_FAULT", raiseAlarm ? "RT_GUARD_FAULT_ALARM" : "RT_GUARD_FAULT_DURING_STOP");
        // EDM231 FIX3: read-only first-rejection evidence. A delayed or other
        // session's pause receipt cannot explain this guard's first fault.
        // A prior SOURCE/other-site fault on the same tick remains first.
        // Pre-Tick rejection belongs to the guard's preceding frame context;
        // keep that fault context separate from the callback capture tick.
        if (EDM173::IsArm(s.gapHistoryArm))
        {
            EDM157::Feedback pause{};
            const bool pauseRead = m_motion.ReadEDM157Feedback(pause);
            const auto& d = pause.firstFailure;
            const bool pauseFailureMatches = pauseRead &&
                f.firstFault.reason == static_cast<std::uint32_t>(EDM94::Reason::Identity) &&
                f.firstFault.site == static_cast<std::uint32_t>(EDM94::FaultSite::EngineFault) &&
                pause.scope.authorizedSource.scope.session == s.session &&
                d.site != EDM157::RejectSite::None && d.tick != 0ULL &&
                d.guardContextTick != 0ULL && d.guardContextTick == f.firstFault.tick;
            if (pauseFailureMatches)
            {
                PrintNCG76DiagnosticLine("[EDM231_FIX3] event=PAUSE_FIRST session=%llu faultTick=%llu captureTick=%llu nowUs=%llu site=%s priorPhase=%s physicalEvaluated=%08X physicalFailures=%08X physicalAxisMask=%02X proofEvaluated=%08X proofFailures=%08X observationOnly=1 authority=NONE\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(d.guardContextTick),
                    static_cast<unsigned long long>(d.tick),
                    static_cast<unsigned long long>(d.nowUs),EDM157::RejectSiteName(d.site),EDM157::PhaseName(d.priorPhase),
                    static_cast<unsigned>(d.physicalEvaluated),static_cast<unsigned>(d.physicalFailures),
                    static_cast<unsigned>(d.physicalAxisMask),static_cast<unsigned>(d.proofEvaluated),static_cast<unsigned>(d.proofFailures));
                PrintNCG76DiagnosticLine("[EDM231_FIX3] event=PAUSE_FIRST_TIMING session=%llu faultTick=%llu physicalDiagnosticTick=%llu qualifySeedTick=%llu resumeSeedTick=%llu physicalTick=%llu mappedTick=%llu sentTick=%llu transportTick=%llu proofTick=%llu observationOnly=1 authority=NONE\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(d.guardContextTick),
                    static_cast<unsigned long long>(d.physicalDiagnosticTick),static_cast<unsigned long long>(d.qualifySeedTick),
                    static_cast<unsigned long long>(d.resumeSeedTick),static_cast<unsigned long long>(d.physicalTick),
                    static_cast<unsigned long long>(d.mappedTick),static_cast<unsigned long long>(d.sentTick),
                    static_cast<unsigned long long>(d.transportTick),static_cast<unsigned long long>(d.proofTick));
            }
            else
                PrintNCG76DiagnosticLine("[EDM231_FIX3] event=PAUSE_FIRST session=%llu faultTick=%llu evidence=UNAVAILABLE pauseRead=%u observationOnly=1 authority=NONE\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.firstFault.tick),pauseRead?1U:0U);
        }
        if (raiseAlarm)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
            ChangeState(NCState::ALARM);
            return;
        }
    }
    if (s.resetCommitted && !s.everArmed && !f.active && f.faulted)
    { ReleaseEDM94AfterResetSameThread(); return; }
    if ((!s.gapModeInterruption || freshPhysical) &&
        f.released && s.releaseRequested && s.resetCommitted && f.identity.epoch == s.armEpoch &&
        f.identity.ownerGeneration == s.lease.generation && f.lastTick >= s.lastTick)
    {
        LogEDM94SameThread("RELEASED", "EXACT_RESET_AND_PHYSICAL_ZERO");
        s.latched = s.armPending = s.ready = s.finishPending = false;
        return;
    }
    if (f.cancelled && !s.cancelled) CancelEDM94SameThread("RT_CANCELLED");
    if (s.cancelled && f.stopped && f.forceZero && !s.terminalLogged)
    { s.terminalLogged = true; LogEDM94SameThread("CANCEL_STOPPED", "PHYSICAL_ZERO_WINDOW"); }
}

// Bounded NC-owner numerical work. This runs before publishing the physical arm,
// and its workspace/result never enters Motion, the PDO path or the RT servo loop.
bool NCManager::RunEDM112GeometrySameThread() noexcept
{
    const auto& s = m_edm94; auto& c = m_edm112;
    if (!s.gapCurveGeometry || !s.gapB2Repeat || !s.gapB2CrossSegment || !s.gapCommonPath ||
        !s.gapFlushProfile || !s.gapFlushParameterized || s.gapFlushInterruptArm != 0U ||
        !s.session || !s.run || !s.cache || !s.lease.IsValid() || s.lease.owner != MotionOwner::AUTO ||
        s.latched || s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || c.valid) return false;
    if (!EDM112::RunFixture(c.workspace, c.result) || !EDM112::IsFixtureResultCurrent(c.result) ||
        c.result.completedLines != 0U)
    {
        RtPrintf("[EDM112] event=CURVE_GEOMETRY_FAIL reason=%s failedCase=%u physicalArc=0 numeric=1 applyToMotion=0 authority=NONE\n",
            EDM112::ReasonName(c.result.reason), c.result.failedCase);
        return false;
    }
    c.session = s.session; c.run = s.run; c.cache = s.cache; c.ownerGeneration = s.lease.generation;
    c.valid = true;
    const auto& r = c.result;
    RtPrintf("[EDM112] event=CURVE_GEOMETRY_PASS build=EDM112 session=%llu cases=%u lineSamples=%u cwSamples=%u ccwSamples=%u planeMask=%u helixRotaryCases=%u physicalArc=0 numeric=1 applyToMotion=0 authority=NONE\n",
        static_cast<unsigned long long>(s.session), r.cases, r.lineSamples, r.cwSamples, r.ccwSamples,
        r.planeMask, r.helixRotaryCases);
    RtPrintf("[EDM112] event=CURVE_GEOMETRY_CHECKS session=%llu reverseSpans=%u b2Plans=%u endpointChecks=%u derivativeChecks=%u boundsChecks=%u negativeCases=%u physicalArc=0 numeric=1 applyToMotion=0 authority=NONE\n",
        static_cast<unsigned long long>(s.session), r.reverseSpans, r.b2Plans, r.endpointChecks,
        r.derivativeChecks, r.boundsChecks, r.negativeCases);
    return IsEDM112GeometryCurrentSameThread(false);
}

bool NCManager::IsEDM112GeometryCurrentSameThread(bool requireComplete) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm112;
    if (!s.gapCurveGeometry || !s.gapB2Repeat || !s.gapB2CrossSegment || !s.gapCommonPath ||
        !s.gapFlushProfile || !s.gapFlushParameterized || s.gapFlushInterruptArm != 0U ||
        !c.valid || !c.session || c.session != s.session || c.run != s.run || c.cache != s.cache ||
        !c.ownerGeneration || c.ownerGeneration != s.lease.generation ||
        s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested || s.rtFaultReported ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !EDM112::IsFixtureResultCurrent(c.result)) return false;
    if (!requireComplete) return true;
    const auto& ledger = m_edm109.ledger;
    return c.result.completedLines == EDM112::ExpectedCompletedLines &&
        c.result.completedLines == EDM109::Capacity && ledger.Valid() && ledger.Count() == EDM109::Capacity &&
        m_edm109.mappedSegments == EDM109::Capacity && m_edm109.plans == EDM109::Capacity - 1U &&
        ledger.FrozenScope().session == c.session && ledger.FrozenScope().run == c.run &&
        ledger.FrozenScope().cache == c.cache && ledger.FrozenScope().ownerGeneration == c.ownerGeneration;
}

// NC consumes a separate RT observation receipt only after the actual G01
// boundary is retained. Neither this receipt nor its hypothetical curve owns
// motion. Completed receipts remain valid through normal owner retirement.
bool NCManager::IsEDM113RuntimeCurrentSameThread(unsigned completed,
    std::uint64_t mappingTick) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm113;
    const auto& ledger = m_edm109.ledger;
    if (!s.gapCurveRuntime || !s.gapCurveGeometry || !s.gapB2Repeat ||
        !s.latched || s.armPending || !s.everArmed || s.moves != EDM109::Capacity ||
        !s.armEpoch || completed > EDM109::Capacity || c.acknowledgedRows > completed ||
        !IsEDM112GeometryCurrentSameThread(false) || !m_motion.ReadEDM113Feedback(c.feedback)) return false;
    const auto& f = c.feedback; const auto& prior = c.retained;
    if (f.failed && !c.failureLogged)
    {
        c.failureLogged = true;
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM113] event=RUNTIME_CURVE_FAIL logger=FIX1 session=%llu reason=%u step=%u tick=%llu lastSample=%llu begun=%u completed=%u expected=%u physicalArc=0 applyToMotion=0 action=NC_REJECT\n",
            static_cast<unsigned long long>(f.session), static_cast<unsigned>(f.reason), f.source.step,
            static_cast<unsigned long long>(f.lastTick), static_cast<unsigned long long>(f.lastSampleTick),
            f.begunRows, f.completedRows, completed);
        EmitEDM113Record(record, length, "RUNTIME_CURVE_FAIL");
    }
    if (!f.enabled || f.failed || f.reason != EDM113::Reason::None || f.PhysicalPermit() || f.suspended ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        f.begunRows != completed || f.completedRows != completed || f.endpointRows != completed ||
        f.cwRows != (completed + 1U) / 2U || f.ccwRows != completed / 2U ||
        f.completed != (completed == EDM109::Capacity) || f.rowComplete != (completed != 0U)) return false;
    const unsigned pauses = completed >= 5U ? 2U : completed >= 3U ? 1U : 0U;
    if (f.suspensions != pauses || f.resumptions != pauses) return false;
    if (completed == 0U)
    {
        // SubmitArm has no row/run/cache source yet. Do not demand a future Bind.
        if (mappingTick || c.acknowledgedRows || ledger.Valid() || ledger.Count() ||
            !EDM113::SameSource(f.source, EDM109::Source{}) ||
            f.bindTick || f.lastTick || f.lastSampleTick || f.lastCompletedMappingTick ||
            f.samples || f.forwardSamples || f.reverseSamples || f.flushSamples || f.suspendedTicks ||
            f.rowSamples || f.rowForwardSamples || f.rowReverseSamples || f.rowFlushSamples || f.rowSuspendedTicks ||
            f.sourcePulseLength != 0.0 || f.unitSpeedLimit != 0.0 || f.unitAccelerationLimit != 0.0 ||
            f.maxUnitSpeed != 0.0 || f.maxUnitAcceleration != 0.0) return false;
        for (unsigned axis = 0U; axis < EDM109::AxisCount; ++axis)
            if (f.maxAbsVelocityPps[axis] != 0.0 || f.maxAbsAccelerationPps2[axis] != 0.0) return false;
        c.retained = f; return true;
    }
    const bool fresh = c.acknowledgedRows + 1U == completed;
    if ((!fresh && c.acknowledgedRows != completed) || (fresh && !mappingTick) ||
        !ledger.Valid() || ledger.Count() != completed || m_edm112.result.completedLines != completed) return false;
    const auto* row = ledger.Row(completed - 1U);
    const std::uint64_t expectedMappingTick = mappingTick ? mappingTick : m_edm109.lastMappingTick;
    if (!row || !EDM113::ValidSource(f.source) || !EDM113::SameSource(f.source, row->source) ||
        !EDM109::SameScope(f.source.scope, ledger.FrozenScope()) ||
        f.source.scope.session != s.session || f.source.scope.run != s.run || f.source.scope.cache != s.cache ||
        f.source.scope.ownerGeneration != s.lease.generation || f.source.step != completed - 1U ||
        f.source.sourcePC != s.pc[completed - 1U] || !row->completion.boundaryValidated ||
        !row->completion.commit || !row->completion.tick || !expectedMappingTick ||
        f.lastCompletedMappingTick != expectedMappingTick || expectedMappingTick > row->completion.tick ||
        !f.bindTick || f.bindTick > expectedMappingTick ||
        f.lastSampleTick != expectedMappingTick || f.lastTick != expectedMappingTick ||
        !EDM109::SameDouble(f.sourcePulseLength, row->line.pulseLength) ||
        !std::isfinite(f.unitSpeedLimit) || f.unitSpeedLimit <= 0.0 ||
        !std::isfinite(f.unitAccelerationLimit) || f.unitAccelerationLimit <= 0.0 ||
        !std::isfinite(f.maxUnitSpeed) || f.maxUnitSpeed <= 0.0 ||
        !std::isfinite(f.maxUnitAcceleration) || f.maxUnitAcceleration <= 0.0 ||
        !f.rowSamples || !f.rowForwardSamples || !f.rowReverseSamples ||
        f.rowForwardSamples > f.rowSamples || f.rowReverseSamples > f.rowSamples - f.rowForwardSamples ||
        f.rowFlushSamples > f.rowSamples || f.rowSamples > expectedMappingTick - f.bindTick + 1ULL ||
        (!(f.source.step & 1U) && !f.rowFlushSamples) ||
        ((f.source.step == 2U || f.source.step == 4U) != (f.rowSuspendedTicks != 0ULL))) return false;
    for (unsigned axis = 0U; axis < EDM109::AxisCount; ++axis)
        if (!std::isfinite(f.maxAbsVelocityPps[axis]) || f.maxAbsVelocityPps[axis] <= 0.0 ||
            !std::isfinite(f.maxAbsAccelerationPps2[axis]) || f.maxAbsAccelerationPps2[axis] <= 0.0 ||
            f.maxAbsVelocityPps[axis] < prior.maxAbsVelocityPps[axis] ||
            f.maxAbsAccelerationPps2[axis] < prior.maxAbsAccelerationPps2[axis]) return false;
    if (fresh)
    {
        if (!prior.enabled || prior.failed || prior.session != s.session || prior.armEpoch != s.armEpoch ||
            prior.ownerGeneration != s.lease.generation || prior.completedRows != c.acknowledgedRows ||
            f.bindTick <= prior.lastCompletedMappingTick ||
            f.samples < prior.samples || f.samples - prior.samples != f.rowSamples ||
            f.forwardSamples < prior.forwardSamples || f.forwardSamples - prior.forwardSamples != f.rowForwardSamples ||
            f.reverseSamples < prior.reverseSamples || f.reverseSamples - prior.reverseSamples != f.rowReverseSamples ||
            f.flushSamples < prior.flushSamples || f.flushSamples - prior.flushSamples != f.rowFlushSamples ||
            f.suspendedTicks < prior.suspendedTicks || f.suspendedTicks - prior.suspendedTicks != f.rowSuspendedTicks ||
            f.maxUnitSpeed < prior.maxUnitSpeed || f.maxUnitAcceleration < prior.maxUnitAcceleration) return false;
        c.retained = f; c.acknowledgedRows = completed;
    }
    else
    {
        // Polling a completed boundary may re-read it, but can never add credit.
        if (!EDM113::SameSource(f.source, prior.source) || f.bindTick != prior.bindTick ||
            f.lastCompletedMappingTick != prior.lastCompletedMappingTick ||
            f.samples != prior.samples || f.forwardSamples != prior.forwardSamples ||
            f.reverseSamples != prior.reverseSamples || f.flushSamples != prior.flushSamples ||
            f.suspendedTicks != prior.suspendedTicks || f.rowSamples != prior.rowSamples ||
            f.rowForwardSamples != prior.rowForwardSamples || f.rowReverseSamples != prior.rowReverseSamples ||
            f.rowFlushSamples != prior.rowFlushSamples || f.rowSuspendedTicks != prior.rowSuspendedTicks ||
            !EDM109::SameDouble(f.unitSpeedLimit, prior.unitSpeedLimit) ||
            !EDM109::SameDouble(f.unitAccelerationLimit, prior.unitAccelerationLimit) ||
            !EDM109::SameDouble(f.maxUnitSpeed, prior.maxUnitSpeed) ||
            !EDM109::SameDouble(f.maxUnitAcceleration, prior.maxUnitAcceleration) ||
            !EDM109::SamePulse(f.maxAbsVelocityPps, prior.maxAbsVelocityPps) ||
            !EDM109::SamePulse(f.maxAbsAccelerationPps2, prior.maxAbsAccelerationPps2)) return false;
    }
    return true;
}

void NCManager::LogEDM113RuntimeSameThread() const noexcept
{
    const auto& f = m_edm113.retained;
    char record[481]{};
    int length = std::snprintf(record, sizeof(record),
        "[EDM113] event=RUNTIME_CURVE_COMPLETE build=EDM113 logger=FIX1 session=%llu rows=%u cw=%u ccw=%u endpoints=%u physicalG01=16 physicalB2=2 physicalArc=0 applyToMotion=0 reverseEveryRow=1 complete=1 pauseDuringB2=1 suspensions=%u resumptions=%u dischargeCommand=NONE\n",
        static_cast<unsigned long long>(f.session), f.completedRows, f.cwRows, f.ccwRows,
        f.endpointRows, f.suspensions, f.resumptions);
    EmitEDM113Record(record, length, "RUNTIME_CURVE_COMPLETE");
    length = std::snprintf(record, sizeof(record),
        "[EDM113] event=RUNTIME_CURVE_SAMPLES logger=FIX1 session=%llu total=%llu forward=%llu reverse=%llu flush=%llu suspendedTicks=%llu mappingTick=%llu dynamics=CHECKED softwareLimits=1 physicalArc=0 applyToMotion=0\n",
        static_cast<unsigned long long>(f.session), static_cast<unsigned long long>(f.samples),
        static_cast<unsigned long long>(f.forwardSamples), static_cast<unsigned long long>(f.reverseSamples),
        static_cast<unsigned long long>(f.flushSamples), static_cast<unsigned long long>(f.suspendedTicks),
        static_cast<unsigned long long>(f.lastCompletedMappingTick));
    EmitEDM113Record(record, length, "RUNTIME_CURVE_SAMPLES");
}

void NCManager::InvalidateEDM109CommonPathSameThread(const char* reason) noexcept
{
    m_edm126 = EDM126NCState{};
    m_edm129 = EDM129NCState{};
    m_edm130 = EDM130NCState{};
    m_edm169 = EDM169NCState{};
    m_edm127.current = m_edm127.pending = EDM127::Selection{};
    m_edm127.lastPC = -1; m_edm127.lastDispatch = 0ULL;
    m_edm127.explicitRows = m_edm127.inheritedRows = m_edm127.standaloneSelections = 0U;
    m_edm110.token = EDM110::CompletionToken{};
    m_edm110.feedback = EDM110::MotionFeedback{};
    m_edm110.first = EDM110::MotionFeedback{};
    m_edm110.complete = m_edm110.firstRetained = false;
    m_edm112.valid = false; m_edm112.result = EDM112::Result{};
    m_edm112.session = m_edm112.run = m_edm112.cache = m_edm112.ownerGeneration = 0ULL;
    m_edm113.feedback = EDM113::Feedback{}; m_edm113.retained = EDM113::Feedback{};
    m_edm113.acknowledgedRows = 0U; m_edm113.failureLogged = false;
    m_edm115.token = EDM115::CompletionToken{}; m_edm115.feedback = EDM115::MotionFeedback{};
    for (unsigned event = 0U; event < 2U; ++event)
    {
        m_edm115.tokens[event] = EDM115::CompletionToken{};
        m_edm115.previousRows[event] = EDM114::Feedback{}; m_edm115.triggerRows[event] = EDM114::Feedback{};
        m_edm115.receipts[event] = EDM115::MotionFeedback{};
        m_edm115.completionTick[event] = m_edm115.completionUs[event] = m_edm115.completionCommit[event] = 0ULL;
    }
    m_edm115.retainedEvents = 0U; m_edm115.failureLogged = false;
    m_edm116.token = EDM115::CompletionToken{}; m_edm116.feedback = EDM116::MotionFeedback{};
    for (auto& policy : m_edm116.policies) policy = EDM116::Policy{};
    for (auto& policy : m_edm116.completedPolicies) policy = EDM116::Policy{};
    for (auto& source : m_edm116.completedSources) source = EDM114::Source{};
    m_edm116.issuedDispatch.fill(0ULL); m_edm116.issuedEpoch.fill(0ULL);
    for (unsigned event = 0U; event < 8U; ++event)
    {
        m_edm116.triggerRows[event] = EDM114::Feedback{};
        m_edm116.receipts[event] = EDM116::MotionFeedback{};
        m_edm116.completionTick[event] = m_edm116.completionUs[event] = m_edm116.completionCommit[event] = 0ULL;
    }
    for (unsigned event = 0U; event < 2U; ++event)
    { m_edm116.tokens[event] = EDM115::CompletionToken{}; m_edm116.previousRows[event] = EDM114::Feedback{}; }
    m_edm116.selected = m_edm116.consumed = m_edm116.acknowledgedRows = m_edm116.retainedEvents = 0U;
    m_edm116.pending = m_edm116.failureLogged = false;
    m_edm116.initialBoundaryTick = m_edm116.initialBoundaryUs = 0ULL;
    m_edm116.initialBoundaryReady = m_edm116.initialTranslationValid = m_edm116.initialBoundaryLogged = false;
    m_edm114.feedback = EDM114::Feedback{}; m_edm114.retained = EDM114::Feedback{};
    m_edm114.completionTick = m_edm114.completionCommit = 0ULL;
    m_edm114.acknowledgedRows = m_edm114.arcFlushRows = 0U; m_edm114.failureLogged = false;
    auto& c = m_edm109;
    if (c.ledger.Valid())
        RtPrintf("[EDM109] event=HISTORY_INVALIDATED session=%llu retained=%u plans=%llu reason=%.64s physicalB2=0 numeric=1 dischargeCommand=NONE\n",
            static_cast<unsigned long long>(c.ledger.FrozenScope().session), c.ledger.Count(),
            static_cast<unsigned long long>(c.plans), reason ? reason : "UNKNOWN");
    c.ledger.Invalidate(); c.plan.valid = false; c.plan.count = 0U; c.line.valid = false;
    c.plans = c.mappedSegments = c.lastMappingTick = 0ULL;
}

bool NCManager::RetainEDM109CompletedSameThread(const EDM94::Feedback& observed) noexcept
{
    auto& s = m_edm94; auto& c = m_edm109;
    const auto& p = m_pathFeed; const auto& r = m_pathFeedMotion.receipt;
    if (!s.gapCommonPath || !s.gapFlushProfile || !s.gapFlushParameterized || s.gapFlushInterruptArm != 0U ||
        !s.latched || !s.ready || s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested ||
        s.rtFaultReported || s.finishPending || s.complete || s.next == 0U || s.next > s.moves ||
        s.moves != EDM109::Capacity || !m_motion.ReadEDM109Feedback(c.feedback)) return false;
    const auto& f = c.feedback;
    if (!f.enabled || !f.lineValid || !f.mappedOutput || !f.completed ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        f.run != s.run || f.cache != s.cache || f.dispatch != p.dispatch || f.sourcePC != p.sourcePC ||
        f.epoch != r.identity.epoch || f.segmentId != r.identity.segmentId ||
        f.translationGeneration != r.translationGeneration || f.step != s.next - 1U ||
        f.begunSegments != s.next || f.mappedSegments != s.next || f.mappedSamples < f.mappedSegments ||
        f.lastMappingTick == 0ULL || f.lastMappingTick > observed.lastTick ||
        f.lastMappingUs == 0ULL || f.lastMappingUs > observed.lastNowUs ||
        observed.faulted || observed.cancelled || observed.forceZero || !observed.active ||
        !observed.armedReady || observed.handoffZero || observed.boundaryNotReadyMask != 0U ||
        observed.boundaryReadyCycles < EDM94::BoundaryReadyCycles) return false;
    std::uint64_t configTag = 1469598103934665603ULL, mappingTag = configTag;
    for (unsigned axis = 0U; axis < EDM109::AxisCount; ++axis)
    {
        const auto& config = observed.config.axis[axis];
        if (!EDM109::SameDouble(f.startPulse[axis], r.xyzcuv.startPulse[axis]) ||
            !EDM109::SameDouble(f.endPulse[axis], r.xyzcuv.endPulse[axis]) ||
            !EDM109::SameDouble(f.pulsePerUnit[axis], r.xyzcuv.pulsePerUnit[axis]) ||
            !EDM109::SameDouble(f.pulsePerUnit[axis], config.pulsePerUnit) ||
            f.configFingerprint[axis] != config.configFingerprint ||
            f.mappingFingerprint[axis] != config.mappingFingerprint ||
            (c.ledger.Valid() && (f.configFingerprint[axis] != c.configFingerprint[axis] ||
                f.mappingFingerprint[axis] != c.mappingFingerprint[axis]))) return false;
        configTag = (configTag ^ f.configFingerprint[axis]) * 1099511628211ULL;
        mappingTag = (mappingTag ^ f.mappingFingerprint[axis]) * 1099511628211ULL;
    }
    // Tags summarize the exact per-axis comparisons above; they do not replace them.
    c.source.scope = EDM109::Scope{s.session, s.run, s.cache, s.lease.generation,
        configTag ? configTag : 1ULL, mappingTag ? mappingTag : 1ULL, r.translationGeneration};
    c.source.dispatch = p.dispatch; c.source.epoch = r.identity.epoch;
    c.source.segment = r.identity.segmentId; c.source.sourcePC = p.sourcePC; c.source.step = s.next - 1U;
    c.completion.tick = observed.lastTick; c.completion.commit = p.commit; c.completion.boundaryValidated = true;
    if (!EDM109::BuildLine(f.startPulse, f.endPulse, f.pulsePerUnit, c.line) ||
        !EDM109::SameDouble(c.line.pulseLength, f.pulseLength)) return false;
    if (c.ledger.Valid() && c.ledger.Count() == s.next)
    {
        // A repeated asynchronous wait may observe the same completed receipt.
        // It must never append or plan that source twice.
        const auto* row = c.ledger.Row(s.next - 1U);
        return row && EDM109::SameScope(row->source.scope, c.source.scope) &&
            row->source.dispatch == c.source.dispatch && row->source.epoch == c.source.epoch &&
            row->source.segment == c.source.segment && row->source.sourcePC == c.source.sourcePC &&
            row->source.step == c.source.step && row->completion.commit == p.commit &&
            row->completion.tick <= observed.lastTick && c.lastMappingTick == f.lastMappingTick &&
            EDM109::SamePulse(row->line.startPulse, c.line.startPulse) &&
            EDM109::SamePulse(row->line.endPulse, c.line.endPulse) &&
            EDM109::SamePulse(row->line.pulsePerNative, c.line.pulsePerNative) &&
            (!s.gapCurveGeometry || (IsEDM112GeometryCurrentSameThread(false) &&
                m_edm112.result.completedLines == c.ledger.Count())) &&
            (!s.gapCurveRuntime || IsEDM113RuntimeCurrentSameThread(s.next, f.lastMappingTick));
    }
    if (s.next == 1U && !c.ledger.Valid())
    {
        if (!c.ledger.Begin(c.source.scope)) return false;
        c.configFingerprint = f.configFingerprint; c.mappingFingerprint = f.mappingFingerprint;
    }
    if (!c.ledger.Valid() || c.ledger.Count() != s.next - 1U ||
        f.lastMappingTick <= c.lastMappingTick || !c.ledger.Append(c.line, c.source, c.completion)) return false;
    if (s.gapCurveGeometry &&
        (!IsEDM112GeometryCurrentSameThread(false) || m_edm112.result.completedLines + 1U != s.next ||
            !EDM112::CompareCompletedLine(c.line, m_edm112.workspace, m_edm112.result) ||
            m_edm112.result.completedLines != s.next))
    {
        m_edm112.valid = false;
        RtPrintf("[EDM112] event=COMPLETED_LINE_FAIL session=%llu step=%u compared=%u reason=%s failedCase=%u physicalArc=0 numeric=1 applyToMotion=0 authority=NONE\n",
            static_cast<unsigned long long>(s.session), f.step, m_edm112.result.completedLines,
            EDM112::ReasonName(m_edm112.result.reason), m_edm112.result.failedCase);
        return false;
    }
    // The actual G01 receipt is retained first. The observer can acknowledge
    // only that same completed row and its latched final mapping tick.
    if (s.gapCurveRuntime && !IsEDM113RuntimeCurrentSameThread(s.next, f.lastMappingTick)) return false;
    c.mappedSegments = f.mappedSegments; c.lastMappingTick = f.lastMappingTick;
    char record[481]{};
    const auto emit = [&](int length, const char* part) noexcept
    {
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM109] event=LOG_FORMAT_ERROR part=%.24s physicalB2=0\n", part);
    };
    emit(std::snprintf(record, sizeof(record),
        "[EDM109] event=COMPLETED_RETAINED session=%llu retained=%u step=%u dispatch=%llu commit=%llu epoch=%llu segment=%llu completionTick=%llu mappingTick=%llu mappedSegments=%llu source=ACTUAL_COMMON_G01 completedOnly=1 physicalB2=0 dischargeCommand=NONE\n",
        static_cast<unsigned long long>(s.session), c.ledger.Count(), f.step,
        static_cast<unsigned long long>(p.dispatch), static_cast<unsigned long long>(p.commit),
        static_cast<unsigned long long>(r.identity.epoch), static_cast<unsigned long long>(r.identity.segmentId),
        static_cast<unsigned long long>(observed.lastTick), static_cast<unsigned long long>(f.lastMappingTick),
        static_cast<unsigned long long>(c.mappedSegments)), "COMPLETED_RETAINED");
    if (c.ledger.Count() < 2U) return true;
    if (!EDM109::BuildB2Plan(c.ledger, c.plan) || !c.plan.valid || c.plan.PhysicalPermit() ||
        c.plan.count != 4U || c.plan.anchorOrdinal != c.ledger.Count() - 1U ||
        c.plan.targetOrdinal + 1U != c.plan.anchorOrdinal ||
        !EDM109::SamePulse(c.plan.anchorPulse, c.plan.returnedPulse)) return false;
    ++c.plans;
    emit(std::snprintf(record, sizeof(record),
        "[EDM109] event=B2_PLAN session=%llu plan=%llu retained=%u spans=%u anchorOrdinal=%u targetOrdinal=%u anchorMm=%.12g targetMm=%.12g anchorEndpointEqual=1 backwardCrossings=1 forwardCrossings=1 numeric=1 physicalB2=0 applyToMotion=0 stopEachSpan=1 dischargeCommand=NONE\n",
        static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(c.plans), c.ledger.Count(),
        c.plan.count, c.plan.anchorOrdinal, c.plan.targetOrdinal, c.plan.anchorMm, c.plan.targetMm), "B2_PLAN");
    for (unsigned span = 0U; span < c.plan.count; ++span)
    {
        const auto& part = c.plan.spans[span];
        if (!part.stopRequired) return false;
        emit(std::snprintf(record, sizeof(record),
            "[EDM109] event=B2_SPAN session=%llu plan=%llu span=%u ordinal=%u fromMm=%.12g toMm=%.12g distanceFrame=ROW_LOCAL direction=%s stopRequired=1 numeric=1 physicalB2=0 applyToMotion=0\n",
            static_cast<unsigned long long>(s.session), static_cast<unsigned long long>(c.plans), span,
            part.ordinal, part.startMm, part.endMm, part.endMm < part.startMm ? "REVERSE" : "FORWARD"), "B2_SPAN");
    }
    return true;
}

// This token retires one independently completed NC row; it never supplies geometry or output.
bool NCManager::BuildEDM110CompletionTokenSameThread(unsigned step) noexcept
{
    auto& s = m_edm94; auto& c = m_edm109;
    m_edm110.token = EDM110::CompletionToken{};
    if (!s.gapB2CrossSegment) return !s.gapB2Repeat;
    if (!s.gapCommonPath || !s.gapFlushProfile || !s.gapFlushParameterized || s.gapFlushInterruptArm != 0U ||
        !s.latched || !s.ready || s.cancelled || s.resetSeen || s.resetCommitted || s.releaseRequested ||
        s.rtFaultReported || s.finishPending || s.complete || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || step >= EDM109::Capacity || s.next != step + 1U ||
        s.cache != GetBaseProgramCache().GetGeneration() || s.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease)) return false;
    // Each explicit event consumes a different immediately preceding completed row.
    if (!EDM110::IsTriggerStep(step, s.gapB2Repeat)) return true;
    if (m_edm110.firstRetained) return false; // A retained first event cannot authorize another second event.
    const auto* previous = c.ledger.Row(step - 1U);
    EDM94::Feedback guard{};
    if (!c.ledger.Valid() || c.ledger.Count() != step || !previous ||
        !m_motion.ReadEDM109Feedback(c.feedback) || !m_motion.ReadEDM94Feedback(guard)) return false;
    const auto& f = c.feedback; const auto& source = previous->source;
    const auto& completion = previous->completion;
    if (!EDM109::SameScope(source.scope, c.ledger.FrozenScope()) ||
        source.scope.session != s.session || source.scope.run != s.run || source.scope.cache != s.cache ||
        source.scope.ownerGeneration != s.lease.generation || source.step != step - 1U || source.sourcePC != s.pc[step - 1U] ||
        !completion.boundaryValidated || !completion.tick || !completion.commit ||
        !f.enabled || !f.lineValid || !f.mappedOutput || !f.completed ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        f.run != s.run || f.cache != s.cache || f.dispatch != source.dispatch || f.sourcePC != source.sourcePC ||
        f.epoch != source.epoch || f.epoch != m_motion.GetCurrentExecutionEpoch() || f.segmentId != source.segment ||
        f.translationGeneration != source.scope.translationGeneration || f.step != step - 1U ||
        f.begunSegments != step || f.mappedSegments != step || f.mappedSamples < step ||
        !f.lastMappingTick || f.lastMappingTick != c.lastMappingTick || f.lastMappingTick > completion.tick ||
        !f.lastMappingUs || f.lastMappingUs > guard.lastNowUs ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != source.epoch || guard.currentIdentity.ownerGeneration != s.lease.generation ||
        guard.lastTick < completion.tick || guard.lastTick < s.lastTick || !guard.active || !guard.armAccepted ||
        !guard.armedReady || guard.forceZero || guard.faulted || guard.cancelled || guard.finishRequested ||
        guard.released || guard.handoffZero || guard.boundaryNotReadyMask != 0U ||
        guard.boundaryReadyCycles < EDM94::BoundaryReadyCycles ||
        !CoordSys.IsTranslationRunFrozen() || !CoordSys.IsTranslationRunCurrent() ||
        CoordSys.GetTranslationSnapshot().generation != source.scope.translationGeneration ||
        !IsXYZCUVFeedProfileCurrentSameThread() || !IsFixedTranslationTravelCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread() ||
        !EDM109::SameDouble(f.pulseLength, previous->line.pulseLength) ||
        !EDM109::SamePulse(f.startPulse, previous->line.startPulse) ||
        !EDM109::SamePulse(f.endPulse, previous->line.endPulse) ||
        !EDM109::SamePulse(f.pulsePerUnit, previous->line.pulsePerNative)) return false;
    for (unsigned axis = 0U; axis < EDM109::AxisCount; ++axis)
    {
        const auto& config = guard.config.axis[axis];
        if (f.configFingerprint[axis] != c.configFingerprint[axis] ||
            f.mappingFingerprint[axis] != c.mappingFingerprint[axis] ||
            f.configFingerprint[axis] != config.configFingerprint ||
            f.mappingFingerprint[axis] != config.mappingFingerprint ||
            !EDM109::SameDouble(f.pulsePerUnit[axis], config.pulsePerUnit)) return false;
    }
    if (step == 4U)
    {
        // Copy the first immutable completion before RT retires its geometry for the fresh event.
        if (!s.gapB2Repeat || !m_motion.ReadEDM110Feedback(m_edm110.feedback) ||
            !IsEDM110ReceiptSameThread(m_edm110.feedback, 2U, guard) ||
            m_edm110.feedback.excursion.tick >= completion.tick ||
            m_edm110.feedback.trigger.dispatch >= source.dispatch ||
            m_edm110.feedback.trigger.epoch >= source.epoch ||
            m_edm110.feedback.trigger.segment >= source.segment ||
            m_edm110.feedback.previousCommit >= completion.commit) return false;
        m_edm110.first = m_edm110.feedback; m_edm110.firstRetained = true;
        RtPrintf("[EDM111] event=FIRST_B2_RETAINED session=%llu triggerStep=2 nextTriggerStep=4 eventsStarted=1 eventsCompleted=1 dwells=5 proof=FOLLOWING_WINDOW_200\n",
            static_cast<unsigned long long>(s.session));
    }
    m_edm110.token.previous = source; m_edm110.token.completion = completion;
    m_edm110.token.mappingTick = f.lastMappingTick; m_edm110.token.valid = true;
    return true;
}

bool NCManager::IsEDM109CompleteSameThread() noexcept
{
    const auto& s = m_edm94; auto& c = m_edm109;
    if (!s.gapCommonPath || !c.ledger.Valid() || c.ledger.Count() != EDM109::Capacity ||
        c.plans != EDM109::Capacity - 1U || c.mappedSegments != EDM109::Capacity ||
        !c.plan.valid || c.plan.PhysicalPermit() || c.plan.count != 4U ||
        !EDM109::SamePulse(c.plan.anchorPulse, c.plan.returnedPulse) ||
        c.ledger.FrozenScope().session != s.session || c.ledger.FrozenScope().run != s.run ||
        c.ledger.FrozenScope().cache != s.cache || c.ledger.FrozenScope().ownerGeneration != s.lease.generation ||
        !m_motion.ReadEDM109Feedback(c.feedback)) return false;
    const auto& f = c.feedback; const auto* last = c.ledger.Row(EDM109::Capacity - 1U);
    bool current = last && f.enabled && f.lineValid && f.mappedOutput && f.completed &&
        f.session == s.session && f.armEpoch == s.armEpoch && f.ownerGeneration == s.lease.generation &&
        f.run == s.run && f.cache == s.cache && f.dispatch == last->source.dispatch &&
        f.sourcePC == last->source.sourcePC && f.epoch == last->source.epoch &&
        f.epoch == m_motion.GetCurrentExecutionEpoch() && f.segmentId == last->source.segment &&
        f.translationGeneration == last->source.scope.translationGeneration &&
        f.step + 1U == EDM109::Capacity && f.begunSegments == EDM109::Capacity &&
        f.mappedSegments == EDM109::Capacity && f.mappedSamples >= f.mappedSegments &&
        f.lastMappingTick == c.lastMappingTick && f.lastMappingTick != 0ULL && f.lastMappingUs != 0ULL &&
        EDM109::SameDouble(f.pulseLength, last->line.pulseLength) &&
        EDM109::SamePulse(f.startPulse, last->line.startPulse) && EDM109::SamePulse(f.endPulse, last->line.endPulse) &&
        EDM109::SamePulse(f.pulsePerUnit, last->line.pulsePerNative);
    for (unsigned axis = 0U; current && axis < EDM109::AxisCount; ++axis)
        current = f.configFingerprint[axis] == c.configFingerprint[axis] &&
            f.mappingFingerprint[axis] == c.mappingFingerprint[axis];
    if (current)
        RtPrintf("[EDM109] event=COMMON_PATH_COMPLETE session=%llu retained=16 mappedSegments=16 numericalB2Plans=15 anchorEndpointEqual=1 physicalCommonMap=1 physicalB2=0 numeric=1 next=%s dischargeCommand=NONE\n",
            static_cast<unsigned long long>(s.session), s.gapCurveRuntime ? "RUNTIME_CURVE_CHECK_P124" : s.gapCurveGeometry ? "CURVE_GEOMETRY_CHECK_P122" : s.gapB2Repeat ? "PHYSICAL_B2_CHECK_P120" : s.gapB2CrossSegment ? "PHYSICAL_B2_CHECK_P118" : "PHYSICAL_FINISH_P116");
    return current;
}

// A retained per-event receipt is evidence only. Recheck its exact completed ledger rows,
// frozen scope, physical zero proof and current guard before permitting a later event/finish.
bool NCManager::IsEDM110ReceiptSameThread(const EDM110::MotionFeedback& f, unsigned step,
    const EDM94::Feedback& guard) const noexcept
{
    const auto& s = m_edm94; const auto& c = m_edm109; const auto& e = f.excursion;
    if (!s.gapB2CrossSegment || !EDM110::IsTriggerStep(step, s.gapB2Repeat) ||
        !c.ledger.Valid() || c.ledger.Count() <= step || f.repeat != s.gapB2Repeat ||
        f.eventsStarted != EDM110::EventOrdinal(step) || f.eventsCompleted != EDM110::EventOrdinal(step)) return false;
    const auto* previous = c.ledger.Row(step - 1U); const auto* trigger = c.ledger.Row(step);
    if (!previous || !trigger || !f.enabled || !f.tokenValidated || !f.geometryCurrent ||
        f.session != s.session || f.armEpoch != s.armEpoch || f.ownerGeneration != s.lease.generation ||
        !EDM109::SameScope(f.trigger.scope, trigger->source.scope) ||
        !EDM109::SameScope(f.trigger.scope, c.ledger.FrozenScope()) ||
        f.trigger.scope.session != s.session || f.trigger.scope.run != s.run ||
        f.trigger.scope.cache != s.cache || f.trigger.scope.ownerGeneration != s.lease.generation ||
        f.trigger.dispatch != trigger->source.dispatch || f.trigger.epoch != trigger->source.epoch ||
        f.trigger.segment != trigger->source.segment || f.trigger.sourcePC != trigger->source.sourcePC ||
        f.trigger.step != step || trigger->source.step != step || f.trigger.sourcePC != s.pc[step] ||
        f.previousBoundaryTick != previous->completion.tick || f.previousCommit != previous->completion.commit ||
        !previous->completion.boundaryValidated || !trigger->completion.boundaryValidated ||
        !f.previousBoundaryTick || !f.previousCommit || !f.lastPhysicalTick ||
        f.lastPhysicalTick <= f.previousBoundaryTick || f.lastPhysicalTick > guard.lastTick ||
        f.lastPhysicalTick >= e.tick || e.tick - f.lastPhysicalTick != 1ULL ||
        !e.geometryValid || !e.complete || e.phase != EDM110::Phase::Complete ||
        e.reason != EDM110::Reason::None || !e.endpointReady || e.span != 3U ||
        e.active || e.moving || e.faulted || e.stopRequested || e.stopped ||
        e.started != 1ULL || e.completed != 1ULL || e.backwardCrossings != 1ULL ||
        e.forwardCrossings != 1ULL || e.anchorReturned != 1ULL || e.completedDwells != 5U ||
        e.settledCycles < EDM94::BoundaryReadyCycles || e.v != 0.0 || !std::isfinite(e.q) ||
        !e.tick || e.tick > guard.lastTick || e.tick > trigger->completion.tick ||
        !e.nowUs || e.nowUs > guard.lastNowUs ||
        !e.startTick || e.startTick <= f.previousBoundaryTick || e.startTick > e.tick ||
        !e.startUs || e.startUs > e.nowUs ||
        guard.identity.session != s.session || guard.identity.epoch != s.armEpoch ||
        guard.identity.ownerGeneration != s.lease.generation || guard.currentIdentity.session != s.session ||
        guard.currentIdentity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        guard.currentIdentity.ownerGeneration != s.lease.generation || guard.lastTick < s.lastTick ||
        !guard.active || !guard.armAccepted || guard.faulted || guard.cancelled || guard.released) return false;
    for (unsigned axis = 0U; axis < EDM109::AxisCount; ++axis)
    {
        const auto& config = guard.config.axis[axis];
        if (config.configFingerprint != c.configFingerprint[axis] ||
            config.mappingFingerprint != c.mappingFingerprint[axis] ||
            !EDM109::SameDouble(config.pulsePerUnit, trigger->line.pulsePerNative[axis])) return false;
    }
    return true;
}

void NCManager::LogEDM110ReceiptSameThread(const EDM110::MotionFeedback& f) const noexcept
{
    char record[481]{};
    const int length = std::snprintf(record, sizeof(record),
        "[EDM110] event=PHYSICAL_B2_COMPLETE session=%llu triggerStep=%u epoch=%llu segment=%llu dispatch=%llu started=1 completed=1 backwardCrossings=1 forwardCrossings=1 anchorReturned=1 dwells=5 physicalTick=%llu physicalB2=1 anchorProof=COMMAND_EXACT physicalProof=FOLLOWING_WINDOW_200 numericLedger=SEPARATE dischargeCommand=NONE\n",
        static_cast<unsigned long long>(m_edm94.session), f.trigger.step,
        static_cast<unsigned long long>(f.trigger.epoch), static_cast<unsigned long long>(f.trigger.segment),
        static_cast<unsigned long long>(f.trigger.dispatch), static_cast<unsigned long long>(f.lastPhysicalTick));
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM110] event=LOG_FORMAT_ERROR part=PHYSICAL_B2_COMPLETE\n");
}

bool NCManager::IsEDM110CompleteSameThread(bool log) noexcept
{
    const auto& s = m_edm94; auto& c = m_edm109; auto& b = m_edm110;
    EDM94::Feedback guard{}; EDM96::Feedback gap{};
    if (!s.gapB2CrossSegment || !s.gapCommonPath || !s.gapFlushProfile || !s.gapFlushParameterized ||
        s.gapFlushInterruptArm != 0U || !s.latched || s.cancelled || s.resetSeen || s.resetCommitted ||
        s.releaseRequested || s.rtFaultReported || m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        !c.ledger.Valid() || c.ledger.Count() != EDM109::Capacity || c.mappedSegments != EDM109::Capacity ||
        c.plans != EDM109::Capacity - 1U || c.ledger.FrozenScope().session != s.session ||
        s.run != m_pathCoreLiveBookkeeping.currentRunToken || s.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(s.lease) || !m_motion.ReadEDM110Feedback(b.feedback) ||
        !m_motion.ReadEDM94Feedback(guard) || !m_motion.ReadEDM96Feedback(gap) ||
        !IsEDM110ReceiptSameThread(b.feedback, s.gapB2Repeat ? 4U : 2U, guard) ||
        gap.session != s.session || gap.epoch != m_motion.GetCurrentExecutionEpoch() ||
        gap.ownerGeneration != s.lease.generation || gap.faulted || gap.reason != EDM96::Reason::None ||
        !IsEDM97TuningCurrentSameThread(gap) || gap.acceptedSegments != EDM109::Capacity ||
        gap.completedSegments != EDM109::Capacity || gap.flushStarted != EDM105::ExpectedFlushCycles ||
        gap.flushCompleted != EDM105::ExpectedFlushCycles) return false;
    if (s.gapB2Repeat)
    {
        const auto& first = b.first; const auto& last = b.feedback;
        if (!b.firstRetained || !IsEDM110ReceiptSameThread(first, 2U, guard) ||
            !EDM109::SameScope(first.trigger.scope, last.trigger.scope) ||
            first.trigger.dispatch >= last.trigger.dispatch || first.trigger.epoch >= last.trigger.epoch ||
            first.trigger.segment >= last.trigger.segment || first.trigger.sourcePC >= last.trigger.sourcePC ||
            first.previousCommit >= last.previousCommit ||
            first.excursion.tick >= last.previousBoundaryTick || first.excursion.tick >= last.excursion.startTick ||
            first.excursion.nowUs >= last.excursion.startUs ||
            first.lastPhysicalTick >= last.lastPhysicalTick) return false;
    }
    else if (b.firstRetained) return false;
    b.complete = true;
    if (log)
    {
        if (s.gapB2Repeat) LogEDM110ReceiptSameThread(b.first);
        LogEDM110ReceiptSameThread(b.feedback);
        if (s.gapB2Repeat)
            RtPrintf("[EDM111] event=TWO_B2_COMPLETE build=EDM111 session=%llu triggerSteps=2,4 eventsStarted=2 eventsCompleted=2 dwells=10 distinctCompletedRows=1 physicalProof=FOLLOWING_WINDOW_200 expectedSegments=16 expectedFlushCycles=8 dischargeCommand=NONE\n",
                static_cast<unsigned long long>(s.session));
    }
    return true;
}

void NCManager::LogEDM110SameThread(const char* event) const noexcept
{
    EDM110::MotionFeedback f{};
    const auto session = static_cast<unsigned long long>(m_edm94.session);
    if (!m_motion.ReadEDM110Feedback(f) || f.session != m_edm94.session ||
        f.armEpoch != m_edm94.armEpoch || f.ownerGeneration != m_edm94.lease.generation)
    {
        RtPrintf("[EDM110] event=B2_SNAPSHOT result=%.24s session=%llu evidence=UNAVAILABLE authority=NONE\n",
            event ? event : "UNKNOWN", session);
        return;
    }
    const auto& e = f.excursion; char record[481]{};
    const int length = std::snprintf(record, sizeof(record),
        "[EDM110] event=B2_SNAPSHOT result=%.24s session=%llu tick=%llu phase=%s span=%u started=%llu completed=%llu back=%llu forward=%llu anchor=%llu dwells=%u stable=%u fault=%u stopRequested=%u geometryCurrent=%u physicalB2=SNAPSHOT_ONLY authority=NONE\n",
        event ? event : "UNKNOWN", session, static_cast<unsigned long long>(e.tick), EDM110::PhaseName(e.phase), e.span,
        static_cast<unsigned long long>(e.started), static_cast<unsigned long long>(e.completed),
        static_cast<unsigned long long>(e.backwardCrossings), static_cast<unsigned long long>(e.forwardCrossings),
        static_cast<unsigned long long>(e.anchorReturned), e.completedDwells, e.settledCycles,
        e.faulted ? 1U : 0U, e.stopRequested ? 1U : 0U, f.geometryCurrent ? 1U : 0U);
    if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM110] event=LOG_FORMAT_ERROR part=B2_SNAPSHOT\n");
    const char* reason = "INVALID";
    switch (e.reason)
    {
    case EDM110::Reason::None: reason = "NONE"; break;
    case EDM110::Reason::Geometry: reason = "GEOMETRY"; break;
    case EDM110::Reason::Dynamics: reason = "DYNAMICS"; break;
    case EDM110::Reason::Clock: reason = "CLOCK"; break;
    case EDM110::Reason::Deadline: reason = "DEADLINE"; break;
    case EDM110::Reason::Dwell: reason = "DWELL"; break;
    case EDM110::Reason::Lifecycle: reason = "LIFECYCLE"; break;
    case EDM110::Reason::StopEnvelope: reason = "STOP_ENVELOPE"; break;
    }
    // Separate bounded record: preserve the core cause after cancellation
    // changes the phase to STOPPED; generic guard IDENTITY is insufficient.
    const int detailLength = std::snprintf(record, sizeof(record),
        "[EDM110] event=B2_DETAIL build=FIX1 result=%.24s session=%llu tick=%llu reason=%s startTick=%llu physicalTick=%llu rangeResets=%u authority=NONE\n",
        event ? event : "UNKNOWN", session, static_cast<unsigned long long>(e.tick), reason,
        static_cast<unsigned long long>(e.startTick), static_cast<unsigned long long>(f.lastPhysicalTick),
        f.rangeResets);
    if (detailLength > 0 && detailLength <= 480 && static_cast<std::size_t>(detailLength) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[EDM110] event=LOG_FORMAT_ERROR part=B2_DETAIL\n");
    if (m_edm94.gapB2Repeat)
        RtPrintf("[EDM111] event=B2_PROGRESS build=EDM111 result=%.24s session=%llu repeat=%u triggerStep=%u eventsStarted=%u eventsCompleted=%u firstRetained=%u authority=NONE\n",
            event ? event : "UNKNOWN", session, f.repeat ? 1U : 0U, f.trigger.step,
            f.eventsStarted, f.eventsCompleted, m_edm110.firstRetained ? 1U : 0U);
}

