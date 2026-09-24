// BX / Path Core V2-01. G01 is a distinct feed producer, not a G00 override.
#include "NCManager.h"
#include "AlarmManager.h"
#include "NCExpressionResolver.h"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <windows.h>
#include <rtapi.h>

#if defined(_MSC_VER)
#define NC_PATH_FEED_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_FEED_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_FEED_NOINLINE
#endif

namespace
{
    bool FeedTranslationCurrent(const CoordinateManager& coord, std::uint64_t generation) noexcept
    {
        return generation != 0ULL && coord.IsTranslationRunFrozen() &&
            coord.IsTranslationRunCurrent() && coord.GetTranslationSnapshot().generation == generation;
    }
    // An explicit G20 absolute word may be the current $100..$102 value.
    // Its mm -> inch -> mm round trip need not preserve the native double,
    // even when both spellings encode exactly the same pulse. Retain the
    // accepted endpoint only with exact source-word AND pulse equality.
    // This is not a small-displacement tolerance: neighbouring source words
    // and different pulse targets still enter the strict line builder intact.
    void FeedPreserveInchReadbackEndpoint(const NCBlock& block,
        const CoordinateManager& coord, MotionCore& motion,
        const MotionCncPathTail* predecessor, std::array<double, 8U>& candidate) noexcept
    {
        if (block.has('Q') || !coord.IsTranslationRunFrozen()) return;
        const NCTranslationSnapshot& source = coord.GetTranslationSnapshot();
        if (source.unitsMode != 20 || source.distanceMode != 90 ||
            source.polarMode != 15 || source.cutterMode != 40 ||
            !coord.IsTranslationRunCurrent()) return;
        double currentWCS[8] = {};
        coord.GetCommandedWCS(currentWCS);
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(source.rotationPlane, plane)) return;
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            // With rotation, one unchanged WCS word does not prove that
            // either physical plane component is stationary. Its normal is independent.
            if ((axis == plane.u || axis == plane.v) && NCTranslationHasPlanarRotation(source)) continue;
            const AxisContext& context = motion.GetAxisContext(static_cast<int>(axis));
            const char letter = source.axisIdentity.address[axis];
            if (letter < 'A' || letter > 'Z' || !block.has(letter) ||
                source.axisIdentity.exists[axis] != 1U || source.axisIdentity.nativeUnit[axis] != 1U ||
                !context.isExist || context.axisType != AxisType::LINEAR ||
                !std::isfinite(currentWCS[axis]) || !std::isfinite(candidate[axis]) ||
                !std::isfinite(coord.commandedMCS[axis]) ||
                !std::isfinite(context.resolution_PPR) || context.resolution_PPR <= 0.0 ||
                !std::isfinite(context.finalLead) || context.finalLead <= 0.0 ||
                block.val(letter) != currentWCS[axis] * (1.0 / 25.4)) continue;
            const double pulsePerMM = context.resolution_PPR / context.finalLead;
            const double currentPulse = coord.commandedMCS[axis] * pulsePerMM;
            const double targetPulse = candidate[axis] * pulsePerMM;
            const double baselinePulse = predecessor ? predecessor->endPulse[axis] :
                context.logicalCmdPos.Load();
            if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0 ||
                !std::isfinite(currentPulse) || !std::isfinite(targetPulse) ||
                currentPulse != targetPulse || currentPulse != baselinePulse ||
                (predecessor && predecessor->endMCS[axis] != coord.commandedMCS[axis])) continue;
            // Motion independently proves predecessor identity/admission and
            // publishes this same canonical MCS endpoint in its receipt.
            candidate[axis] = coord.commandedMCS[axis];
        }
    }

    // DS: bounded, side-effect-free syntax only. Future rows use this
    // classifier without reading any variable or resolving any expression.
    bool FeedDecimalLiteral(const std::string& expression) noexcept
    {
        if (expression.empty() || expression.size() > 64U) return false;
        std::size_t i = 0U;
        if (expression[i] == '+' || expression[i] == '-') ++i;
        bool digit = false, point = false;
        for (; i < expression.size(); ++i)
        {
            const char c = expression[i];
            if (c >= '0' && c <= '9') digit = true;
            else if (c == '.' && !point) point = true;
            else return false;
        }
        return digit;
    }

    // DV: legacy Global helper names now cover @, # and $. Index ranges
    // and value ownership stay with MacroVariableRules / MacroEngine.
    bool FeedDirectGlobalWord(const std::string& expression) noexcept
    {
        if (expression.size() < 2U || expression.size() > 5U ||
            (expression[0] != '@' && expression[0] != '#' && expression[0] != '$'))
            return false;
        int index = 0;
        for (std::size_t i = 1U; i < expression.size(); ++i)
        {
            const char c = expression[i];
            if (c < '0' || c > '9') return false;
            index = index * 10 + (c - '0');
        }
        return MacroVariableRules::IsValidIndex(expression[0], index);
    }

    // DU: bounded flat arithmetic syntax; only current-PC candidates are
    // evaluated by the existing resolver. Future rows never read variables.
    // No substrings, recursion, allocation or arithmetic evaluation here.
    bool FeedBoundedArithmeticWord(const std::string& expression, bool& hasGlobal) noexcept
    {
        hasGlobal = false;
        if (FeedDecimalLiteral(expression)) return true;
        if (FeedDirectGlobalWord(expression))
        {
            hasGlobal = true;
            return true;
        }
        if (expression.empty() || expression.size() > 64U) return false;
        std::size_t i = 0U;
        unsigned atoms = 0U;
        bool sawGlobal = false;
        while (i < expression.size())
        {
            if (++atoms > 8U) return false;
            if (expression[i] == '+' || expression[i] == '-') ++i;
            if (i == expression.size()) return false;
            if (expression[i] == '@' || expression[i] == '#' || expression[i] == '$')
            {
                const char prefix = expression[i++];
                unsigned digits = 0U;
                int index = 0;
                while (i < expression.size() && expression[i] >= '0' && expression[i] <= '9')
                {
                    if (++digits > 4U) return false;
                    index = index * 10 + (expression[i++] - '0');
                }
                if (digits == 0U || !MacroVariableRules::IsValidIndex(prefix, index)) return false;
                sawGlobal = true;
            }
            else
            {
                bool digit = false, point = false;
                while (i < expression.size())
                {
                    const char c = expression[i];
                    if (c >= '0' && c <= '9') digit = true;
                    else if (c == '.' && !point) point = true;
                    else break;
                    ++i;
                }
                if (!digit) return false;
            }
            if (i == expression.size())
            {
                hasGlobal = sawGlobal;
                return true;
            }
            const char op = expression[i++];
            if (op != '+' && op != '-' && op != '*' && op != '/') return false;
            if (i == expression.size()) return false;
        }
        return false;
    }

    // EG: all selected planar lines share the existing physical XY mapping
    // when both axes can participate. This also covers pure X-only/Y-only
    // chains, so their acceleration and deceleration obey both axes' limits.
    bool FeedPlanarXYMappingAvailable(MotionCore& motion) noexcept
    {
        for (int i = 0; i < 2; ++i)
        {
            const AxisContext& axis = motion.GetAxisContext(i);
            if (!axis.isExist || axis.axisType != AxisType::LINEAR ||
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
                !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
                !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0) return false;
            const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
            if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0) return false;
        }
        return true;
    }

    std::uint32_t FeedQueuedPhysicalMask(std::uint32_t mask, bool planarXYAvailable) noexcept
    {
        return planarXYAvailable && (mask == 1U || mask == 2U) ? 3U : mask;
    }

    bool FeedGlobalBlockSyntax(const NCParsedBlock& parsed, std::uint32_t& mask) noexcept
    {
        mask = 0U;
        if (parsed.isEmpty || parsed.isBlockSkip || parsed.error != NCParseError::NONE ||
            !parsed.dependsOnMacroState || parsed.controlType != NCParsedControlType::NONE ||
            parsed.gCount != 1 || parsed.mCount != 0) return false;
        const std::string& g = parsed.gExpressions[0];
        const bool line = g == "1" || g == "01";
        const bool arc = g == "2" || g == "02" || g == "3" || g == "03";
        if (!line && !arc) return false;
        // DY: P may use the same bounded variable arithmetic as XY/IJ/F.
        // Syntax only: never sample a future P. The current-PC resolver and
        // existing arc shape gate still require a finite value exactly 1.
        // Keep non-variable P spellings at the existing literal text 1.
        bool pHasVariable = false;
        const bool variableArcP = arc && parsed.has('P') &&
            FeedBoundedArithmeticWord(parsed.expression('P'), pHasVariable) && pHasVariable;
        // EB: at least one center offset is explicit; the missing one is
        // source-local zero in the arc producer, without sampling future values.
        if (arc && ((!parsed.has('I') && !parsed.has('J')) || !parsed.has('P') ||
            (parsed.expression('P') != "1" && !variableArcP))) return false;
        // DZ/EI: current G01 Q uses the same bounded variable syntax. Its
        // next row requires literal XY; a present F must also be literal.
        if (line && parsed.has('Q') &&
            (!parsed.has('X') || !parsed.has('Y') || parsed.has('Z'))) return false;
        // P or Q alone may be the macro dependency with literal geometry/F.
        bool global = variableArcP;
        for (int i = 0; i < 26; ++i)
        {
            if (!parsed.hasParam[static_cast<std::size_t>(i)]) continue;
            const char letter = static_cast<char>('A' + i);
            if (letter != 'X' && letter != 'Y' && letter != 'F' && letter != 'N' &&
                !(line && (letter == 'Z' || letter == 'Q')) &&
                !(arc && (letter == 'I' || letter == 'J' || letter == 'P'))) return false;
            if (arc && letter == 'P') continue; // Literal/bounded variable syntax checked above.
            const std::string& word = parsed.expression(letter);
            bool wordGlobal = false;
            if (letter == 'N')
            {
                if (!FeedDecimalLiteral(word)) return false;
            }
            else
            {
                if (!FeedBoundedArithmeticWord(word, wordGlobal)) return false;
                global = global || wordGlobal;
            }
            if (letter == 'X' || letter == 'Y' || letter == 'Z')
                mask |= 1U << static_cast<unsigned>(letter - 'X');
        }
        // EF: every planar arc moves XY even with one or both endpoint words omitted.
        // This geometry mask never changes the parsed/resolved presence snapshot.
        if (arc) mask = 3U;
        return global && mask != 0U;
    }

    // Fixed planar rotation may queue only the existing P1 planar arcs or
    // plain XY G01 or bounded full-XY G01 Q. The full shape/classifier gates
    // remain authoritative; Q still requires immutable next-row XY geometry.
    bool FeedFixedRotationQueuedBlockAllowed(const NCBlock& block) noexcept
    {
        return block.gCode == 2 || block.gCode == 3 ||
            (block.gCode == 1 && (block.has('X') || block.has('Y')) &&
                !block.has('Z') && !block.has('P') &&
                (!block.has('Q') || (block.has('X') && block.has('Y'))));
    }

    bool FeedFixedRotationQueuedSyntaxAllowed(const NCParsedBlock& parsed) noexcept
    {
        const std::string& g = parsed.gExpressions[0];
        return g == "2" || g == "02" || g == "3" || g == "03" ||
            ((g == "1" || g == "01") && (parsed.has('X') || parsed.has('Y')) &&
                !parsed.has('Z') && !parsed.has('P') &&
                (!parsed.has('Q') || (parsed.has('X') && parsed.has('Y'))));
    }

    // EA: a future Q is not needed to build the current corner. Keep the
    // original all-literal path; otherwise project only present literal
    // XY/F/N from a bounded variable-Q G01. No future dispatch is granted.
    bool FeedBuildCornerNextGeometry(const NCParsedBlock& parsed, NCBlock& block) noexcept
    {
        if (NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed, block)) return true;
        block = NCBlock{};
        std::uint32_t mask = 0U;
        if (!FeedGlobalBlockSyntax(parsed, mask) || mask != 3U || !parsed.has('Q') ||
            (parsed.gExpressions[0] != "1" && parsed.gExpressions[0] != "01")) return false;
        // All non-Q words must be decimal literals. Together with the global
        // syntax gate this proves Q is the sole macro dependency. Never call
        // the resolver or evaluate Q here; its own current PC owns that value.
        for (int i = 0; i < 26; ++i)
        {
            if (!parsed.hasParam[static_cast<std::size_t>(i)] || i == 'Q' - 'A') continue;
            const std::string& word = parsed.paramExpressions[static_cast<std::size_t>(i)];
            if (!FeedDecimalLiteral(word)) return false;
            errno = 0;
            char* end = nullptr;
            const char* begin = word.c_str();
            const double value = std::strtod(begin, &end);
            if (end == begin || *end != '\0' || errno == ERANGE || !std::isfinite(value)) return false;
            block.hasParam[i] = true;
            block.param[i] = value;
        }
        block.isEmpty = false;
        block.hasG = true;
        block.gCode = 1;
        block.gCount = 1;
        block.gCodes[0] = 1;
        return true;
    }

    bool FeedFullIdentity(const MotionExecutionIdentity& a,
        const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
    bool FeedSameSource(const NCProgramCommitSnapshot& a,
        const NCProgramCommitSnapshot& b) noexcept
    {
        return a.scope == b.scope && a.cacheGeneration == b.cacheGeneration &&
            a.frameId == b.frameId && a.sourcePC == b.sourcePC;
    }
    NC_PATH_FEED_NOINLINE
        bool FeedLedgerTransportHealthy(const NCBlockLifecycleLedger& ledger) noexcept
    {
        const NCBlockLifecycleCounters counters = ledger.GetCounters();
        return counters.activeBlockOverwrite == 0ULL &&
            counters.activeSegmentIndexOverwrite == 0ULL && counters.orphanFeedback == 0ULL &&
            counters.duplicateTerminalFeedback == 0ULL && counters.terminalFeedbackConflict == 0ULL;
    }
    std::uint64_t FeedDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

NC_PATH_FEED_NOINLINE
void NCManager::ClearCncModalFeedSameThread() noexcept
{
    // EL: report the retired committed F once, after all F snapshots clear.
    // This is an NC-thread lifecycle diagnostic; it does not retire RT work.
    const CncFeedValueSnapshot retiredFeed = m_cncModalFeed;
    m_cncModalFeed = CncFeedValueSnapshot{};
    m_pathFeed.capturedFeed = CncFeedValueSnapshot{};
    m_pathArc.capturedFeed = CncFeedValueSnapshot{};
    m_cncFeed.candidateFeed = CncFeedValueSnapshot{};
    if (retiredFeed.valid)
    {
        RtPrintf("[CNC-EL-FCLEAR] run=%llu cache=%llu owner=%u/%u commit=%llu dispatch=%llu pc=%d FBits=%llu flight=%u\n",
            static_cast<unsigned long long>(retiredFeed.run),
            static_cast<unsigned long long>(retiredFeed.cache),
            static_cast<unsigned int>(retiredFeed.ownerLease.owner),
            static_cast<unsigned int>(retiredFeed.ownerLease.generation),
            static_cast<unsigned long long>(retiredFeed.sourceCommit),
            static_cast<unsigned long long>(retiredFeed.sourceDispatch),
            retiredFeed.sourcePC,
            static_cast<unsigned long long>(FeedDoubleBits(retiredFeed.feedMMMin)),
            static_cast<unsigned int>(m_cncFeed.count));
    }
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsCncFeedValueCurrentSameThread(const CncFeedValueSnapshot& snapshot) const noexcept
{
    if (!snapshot.valid || !std::isfinite(snapshot.feedMMMin) ||
        snapshot.feedMMMin <= 0.0 || snapshot.feedMMMin > 100.0 ||
        snapshot.run == 0ULL || snapshot.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        snapshot.cache == 0ULL || snapshot.cache != GetBaseProgramCache().GetGeneration() ||
        !snapshot.ownerLease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        AlarmManager::GetInstance().HasAlarm()) return false;
    if (snapshot.programmed) return true;
    // A repeated numeric value is insufficient: the exact last committed
    // producer source must still own this omitted-F candidate.
    return !m_pathHold.armed && !m_pathHold.bound && !m_gapDryRun.active &&
        !m_gapPath.active && !m_gapWindow.active && m_cncModalFeed.valid &&
        snapshot.sourceCommit != 0ULL && snapshot.sourceDispatch != 0ULL && snapshot.sourcePC >= 0 &&
        snapshot.sourceCommit == m_cncModalFeed.sourceCommit &&
        snapshot.sourceDispatch == m_cncModalFeed.sourceDispatch &&
        snapshot.sourcePC == m_cncModalFeed.sourcePC &&
        snapshot.run == m_cncModalFeed.run && snapshot.cache == m_cncModalFeed.cache &&
        snapshot.ownerLease.Matches(m_cncModalFeed.ownerLease) &&
        FeedDoubleBits(snapshot.feedMMMin) == FeedDoubleBits(m_cncModalFeed.feedMMMin);
}

NC_PATH_FEED_NOINLINE
bool NCManager::CaptureCncFeedValueSameThread(const NCBlock& block,
    CncFeedValueSnapshot& snapshot) const noexcept
{
    snapshot = CncFeedValueSnapshot{};
    if (block.has('F'))
    {
        // F is authored in the current program unit; the committed modal
        // value is native mm/min and omitted F is never converted again.
        snapshot.feedMMMin = NCTranslationLengthToMM(block.val('F'), CoordSys.isInchMode ? 20 : 21);
        snapshot.run = m_pathCoreLiveBookkeeping.currentRunToken;
        snapshot.cache = GetBaseProgramCache().GetGeneration();
        snapshot.ownerLease = m_programMotionLease;
        snapshot.programmed = true;
        snapshot.valid = true;
    }
    else
    {
        snapshot = m_cncModalFeed;
        snapshot.programmed = false;
    }
    return IsCncFeedValueCurrentSameThread(snapshot);
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitCncModalFeedSameThread(const CncFeedValueSnapshot& snapshot,
    NCBlockDispatchId dispatch, std::uint64_t commit, int pc, int line) noexcept
{
    // Callers prove receipt, ledger and queue commit before reaching here.
    // Explicit GAP/PathHold feed values keep their previous independent path.
    if (m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active ||
        m_gapPath.active || m_gapWindow.active) return;
    m_cncModalFeed = snapshot;
    m_cncModalFeed.sourceCommit = commit;
    m_cncModalFeed.sourceDispatch = dispatch;
    m_cncModalFeed.sourcePC = pc;
    m_cncModalFeed.programmed = false;
    RtPrintf("[CNC-EH-FEED] run=%llu dispatch=%llu commit=%llu pc=%d line=%d programmedF=%u FBits=%llu sourceCommit=%llu sourceDispatch=%llu sourcePC=%d\n",
        static_cast<unsigned long long>(snapshot.run), static_cast<unsigned long long>(dispatch),
        static_cast<unsigned long long>(commit), pc, line, snapshot.programmed ? 1U : 0U,
        static_cast<unsigned long long>(FeedDoubleBits(snapshot.feedMMMin)),
        static_cast<unsigned long long>(snapshot.programmed ? commit : snapshot.sourceCommit),
        static_cast<unsigned long long>(snapshot.programmed ? dispatch : snapshot.sourceDispatch),
        snapshot.programmed ? pc : snapshot.sourcePC);
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreBasePlaneLinearBlockShapeValid(const NCBlock& block,
    int unitsMode, bool polar, int plane) noexcept
{
    NCArcPlaneAxes basis{};
    if (!TryGetNCArcPlaneAxes(plane, basis)) return false;
    if ((unitsMode != 20 && unitsMode != 21) || block.isEmpty || block.isGoto ||
        block.isBlockSkip || !block.hasG || (block.gCode != 0 && block.gCode != 1) ||
        block.gCount != 1 || block.gCodes[0] != block.gCode || block.mCount != 0 ||
        !(block.has('X') || block.has('Y') || block.has('Z'))) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
    {
        if (!block.has(word)) continue;
        if ((word != 'N' && word != 'F' && word != 'X' && word != 'Y' && word != 'Z') ||
            !std::isfinite(block.val(word))) return false;
        if ((word == 'X' || word == 'Y' || word == 'Z') &&
            (!polar || word != basis.vAddress) &&
            !std::isfinite(NCTranslationLengthToMM(block.val(word), unitsMode))) return false;
    }
    if (polar && block.has(basis.uAddress) && block.val(basis.uAddress) < 0.0) return false;
    // G00 F remains a percentage in both G20 and G21, not a length/feed.
    // A missing G01 F uses the existing accepted modal-F provenance gate.
    if (block.gCode == 1)
        return IsPathCoreFeedBlockShapeValid(block, true, unitsMode, polar, plane) && !block.has('Q');
    return !block.has('F') || (block.val('F') > 0.0 && block.val('F') <= 100.0);
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreFeedBlockShapeValid(const NCBlock& block, bool allowMissingFeed, int unitsMode, bool polar, int plane) noexcept
{
    NCArcPlaneAxes basis{};
    if (!TryGetNCArcPlaneAxes(plane, basis) || (plane != 17 && block.has('Q'))) return false;
    if (unitsMode != 20 && unitsMode != 21) return false;
    const double feedMMMin = block.has('F') ? NCTranslationLengthToMM(block.val('F'), unitsMode) : 0.0;
    const double toleranceMM = block.has('Q') ? NCTranslationLengthToMM(block.val('Q'), unitsMode) : 0.0;
    if (block.isEmpty || block.isGoto || !block.hasG || block.gCode != 1 ||
        block.gCount != 1 || block.gCodes[0] != 1 || block.mCount != 0 ||
        (!block.has('F') && !allowMissingFeed) ||
        (block.has('F') && (!std::isfinite(feedMMMin) ||
            feedMMMin <= 0.0 || feedMMMin > 100.0)) ||
        !(block.has('X') || block.has('Y') || block.has('Z'))) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && address != 'F' && address != 'X' && address != 'Y' && address != 'Z' && address != 'Q') return false;
        if (!std::isfinite(block.val(address))) return false;
        if ((address == 'X' || address == 'Y' || address == 'Z') &&
            (!polar || address != basis.vAddress) &&
            !std::isfinite(NCTranslationLengthToMM(block.val(address), unitsMode))) return false;
    }
    if (polar && block.has(basis.uAddress) && block.val(basis.uAddress) < 0.0) return false;
    if (block.has('Q') && (!block.has('X') || !block.has('Y') || block.has('Z') ||
        !std::isfinite(toleranceMM) || toleranceMM < 0.0001 || toleranceMM > 1.0)) return false;
    return true;
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreFeedInputOmission(const NCBlock& block) const noexcept
{
    return m_pathFeed.armed && m_pathFeed.explicitFeed && !block.hasG && block.gCount == 0 &&
        (block.has('X') || block.has('Y') || block.has('Z') || block.has('F'));
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsCutterContourEndAllowedSameThread(int sourceLine)
{
    if (CoordSys.toolRadiusMode == 40 && !m_cutterLine.leadOutRequired) return true;
    RtPrintf("[CUTTER][REJECT] reason=INCOMPLETE_CONTOUR_EOF line=%d beforeCommit=1\n", sourceLine);
    AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter, sourceLine);
    ChangeState(NCState::ALARM);
    return false;
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreFeedConfigurationValid() noexcept
{
    const NCTranslationSnapshot translation = CoordSys.GetTranslationSnapshot();
    if (!IsNCTranslationDistanceModeAllowed(CoordSys.isAbsoluteMode, translation) ||
        !IsNCTranslationUnitModeAllowed(CoordSys.isInchMode, translation) || !IsNCTranslationSourceAllowed(CoordSys.GetCurrentWCSGCode(), translation) ||
        !IsNCArcPlaneCode(CoordSys.activePlane) || CoordSys.activePlane != translation.rotationPlane ||
        !IsNCTranslationToolModeAllowed(CoordSys.toolLengthMode, translation) ||
        CoordSys.currentHCode != translation.toolHCode ||
        CoordSys.toolRadiusMode != translation.cutterMode ||
        (translation.cutterMode != 40 && (CoordSys.currentDCode != translation.cutterD ||
            FeedDoubleBits(CoordSys.GetActiveToolRadius()) != FeedDoubleBits(translation.cutterRadiusMM))) ||
        !IsNCTranslationRotationModeAllowed(CoordSys.isG68Active, CoordSys.g68Angle,
            CoordSys.activePlane, translation) ||
        !IsNCTranslationWorkModeAllowed(CoordSys.isWorkpieceRotationActive, CoordSys.currentWCode, translation) ||
        !IsNCTranslationScaleMirrorModeAllowed(CoordSys.isScalingActive, CoordSys.isMirrorActive, translation) ||
        !IsNCTranslationPolarModeAllowed(CoordSys.isPolarCoordinateActive, translation) || CoordSys.isCAxisOffsetRotationEnabled ||
        m_axisNames[0] != 'X' || m_axisNames[1] != 'Y' || m_axisNames[2] != 'Z' ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    return true;
}

NC_PATH_FEED_NOINLINE
void NCManager::ArmPathCoreFeedSameThread() noexcept
{
    ClearCncModalFeedSameThread();
    m_cncFeed.Clear();
    m_pathFeed = PathFeedState{};
    m_cutterLine = CutterLineState{};
    m_pathFeedMotion.receipt.Clear();
    m_pathFeed.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_pathFeed.cache = GetBaseProgramCache().GetGeneration();
    m_pathFeed.lastSequence = m_lastConsumedMotionFeedbackSequence;
    m_pathFeed.armed = m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY &&
        m_pathFeed.run != 0ULL && m_pathFeed.cache != 0ULL && m_programMotionLease.IsValid();
}

NC_PATH_FEED_NOINLINE
void NCManager::InvalidatePathCoreFeedSameThread(bool byGoto) noexcept
{
    m_cutterLine = CutterLineState{};
    ClearCncModalFeedSameThread();
    InvalidateCncFeedSameThread();
    const bool wasPending = m_pathFeed.pending;
    m_pathFeed.armed = false;
    m_pathFeed.invalidatedByGoto = byGoto;
    m_pathFeed.pending = false;
    m_pathFeed.bound = false;
    m_pathFeed.completed = false;
    m_pathFeed.explicitFeed = false;
    m_pathFeedMotion.receipt.valid = false;
    if (wasPending)
    {
        m_pathFeed.code = 12U;
        LogPathCoreFeedSameThread("INVALIDATED");
    }
}

NC_PATH_FEED_NOINLINE
void NCManager::ValidatePathCoreFeedSameThread()
{
    ValidateCncFeedSameThread();
    if (!m_pathFeed.armed) return;
    const NCState state = m_state.load(std::memory_order_acquire);
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() || !m_macroStack.empty() ||
        Homing.IsActive() || !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        InvalidatePathCoreFeedSameThread();
        return;
    }
    if (!m_pathFeed.pending) return;
    if (!IsPathCoreFeedConfigurationValid() ||
        !FeedTranslationCurrent(CoordSys, m_pathFeedMotion.receipt.translationGeneration) ||
        m_pathFeedMotion.receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathFeedMotion.receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_FEED_NOINLINE
void NCManager::BeginPathCoreFeedCaptureSameThread(const NCBlock& block,
    NCBlockDispatchId dispatchId) noexcept
{
    if (NCGCodeSemantics::Contains(block, 40) && m_cutterLine.valid)
    {
        // G40 changes the descriptor without moving the centre. A separate
        // exact-stop full-XY G01 must remove the remaining physical offset.
        m_cutterLine.leadOutRequired = true;
        m_cutterLine.valid = false;
    }
    if (NCGCodeSemantics::Contains(block, 0)) m_pathFeed.explicitFeed = false;
    if (!NCGCodeSemantics::Contains(block, 1)) return;
    // End the old G00-only data service explicitly, without faulting its store
    // or using its receipt/return cursor as a G01 execution permit.
    ClosePathCoreCommittedRunSameThread();
    if (m_pathFeed.pending) return; // Start rejects; never overwrite a live receipt.
    m_pathFeedMotion.receipt.Clear();
    m_cutterLine.staged = false;
    m_pathFeed.dispatch = dispatchId;
    m_pathFeed.commit = 0ULL;
    m_pathFeed.capturedFeed = CncFeedValueSnapshot{};
    m_pathFeed.sourcePC = -1;
    m_pathFeed.sourceLine = 0;
    m_pathFeed.bound = false;
    m_pathFeed.consumerAccepted = false;
    m_pathFeed.consumerStarted = false;
    m_pathFeed.completed = false;
    m_pathFeed.code = 0U;
}

NC_PATH_FEED_NOINLINE
void NCManager::RejectPathCoreFeedSameThread(std::uint32_t code, int alarmCode)
{
    m_cutterLine = CutterLineState{};
    if (alarmCode == AlarmManager::G_Code_Invalid_parameter)
    {
        if (code == 3U || code == 4U) alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
        else if (code == 5U) alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
        else if (code == 6U)
        {
            if (m_pathFeedMotion.receipt.travelLimitRejected)
                alarmCode = AlarmManager::PROGRAMMED_OVER_TRAVEL;
            else if (m_pathFeedMotion.receipt.code == MotionFeedLineCode::NOT_READY)
                alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
            else if (m_pathFeedMotion.receipt.code == MotionFeedLineCode::GEOMETRY_REJECTED)
                alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
            else alarmCode = AlarmManager::PATH_MOTION_NOT_ADMITTED;
        }
    }
    RtPrintf("[PCORE-ALARM] alarm=%d unit=FEED run=%llu dispatch=%llu code=%u pc=%d line=%d producer=%u geometry=%u accepted=%u\n",
        alarmCode, static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
        static_cast<unsigned int>(code), m_pathFeed.sourcePC, m_pathFeed.sourceLine,
        static_cast<unsigned int>(m_pathFeedMotion.receipt.code),
        static_cast<unsigned int>(m_pathFeedMotion.receipt.geometryCode),
        m_pathFeedMotion.receipt.commandAccepted ? 1U : 0U);
    if (alarmCode == AlarmManager::PATH_INVALIDATED_BY_GOTO)
        RtPrintf("[PCORE-CAUSE] alarm=2023 unit=FEED reason=GOTO_INVALIDATED action=EDIT_FLOW_RESET_RESTART pc=%d line=%d\n",
            m_pathFeed.sourcePC, m_pathFeed.sourceLine);
    m_pathFeed.code = code;
    if (m_pathFeedMotion.receipt.commandAccepted) ++m_pathFeed.failed;
    else ++m_pathFeed.rejected;
    m_pathFeed.pending = false;
    m_pathFeed.bound = false;
    m_pathFeed.completed = false;
    m_pathFeedMotion.receipt.valid = false;
    LogPathCoreFeedSameThread(m_pathFeedMotion.receipt.commandAccepted ? "FAILED" : "REJECTED");
    AlarmManager::GetInstance().Trigger(alarmCode, m_pathFeed.sourceLine);
    ChangeState(NCState::ALARM);
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreFeedSameThread(const NCBlock& block)
{
    if (!IsPathCoreFeedBlockShapeValid(block, true, CoordSys.isInchMode ? 20 : 21, CoordSys.isPolarCoordinateActive, CoordSys.activePlane) ||
        (CoordSys.activePlane != 17 && !IsPathCoreBasePlaneLinearBlockShapeValid(block,
            CoordSys.isInchMode ? 20 : 21, CoordSys.isPolarCoordinateActive, CoordSys.activePlane)))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // A bad active policy is a configuration error, not an out-of-range point.
    // HOME-before-limits and G23/Limit1 semantics are owned by CoordinateManager.
    for (int axisIndex = 0; axisIndex < 3; ++axisIndex)
    {
        const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(m_motion.GetAxisContext(axisIndex));
        if (invalidMask != 0U)
        {
            RtPrintf("[TRAVEL-CONFIG][REJECT] unit=FEED axis=%d invalidMask=%u beforeSubmit=1\n", axisIndex, invalidMask);
            RejectPathCoreFeedSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
            return nullptr;
        }
    }
    const bool cutter = CoordSys.toolRadiusMode != 40;
    // BASE-PLANE-39: every admitted cutter line and G40 lead-out proves
    // the accepted native XYZ basis, including the stationary normal. This
    // closes the G18/G19 G91 and G90/G16 lead-out gap without changing
    // ordinary G01, notation, queueing, or the physical plane-axis mapping.
    const bool nominalCutterLine = (cutter || m_cutterLine.leadOutRequired) &&
        IsNCTranslationCutterNotationAllowed(CoordSys.activePlane,
            CoordSys.isAbsoluteMode ? 90 : 91, CoordSys.isPolarCoordinateActive ? 16 : 15);
    NCArcPlaneAxes cutterPlane{};
    if (!TryGetNCArcPlaneAxes(CoordSys.activePlane, cutterPlane))
    { RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    if ((cutter || m_cutterLine.leadOutRequired) &&
        !IsCutterContourBlockShapeValid(block, CoordSys.isInchMode ? 20 : 21, CoordSys.activePlane, CoordSys.isPolarCoordinateActive, CoordSys.isAbsoluteMode ? 90 : 91))
    {
        RtPrintf("[CUTTER][REJECT] reason=FULL_PLANE_LINE_REQUIRED beforeSubmit=1\n");
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const bool queued = m_cncFeed.selected;
    const bool buffered = queued && m_cncFeed.active;
    if (queued && (CoordSys.activePlane != 17 || !IsCncFeedSelectedBlockSameThread(block) ||
        !IsCncFeedScopeSameThread() || m_cncFeed.count >= m_cncFeed.flights.size()))
    {
        RejectCncFeedSameThread("SUBMIT_SCOPE", m_pathFeed.sourceLine);
        return nullptr;
    }
    if (!m_pathFeed.armed || m_pathFeed.pending || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        !IsPathCoreFeedConfigurationValid() || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        (!buffered && (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
            m_motion.GetCommandReplaySize() != 0U)))
    {
        // Preserve the original admission failure and stop path. Only its
        // operator alarm differs when this permit was revoked by a taken GOTO.
        RejectPathCoreFeedSameThread(3U,
            (!m_pathFeed.armed && m_pathFeed.invalidatedByGoto) ?
                AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // Selected candidates retain their resolved source and F provenance.
    // Ordinary motion captures only after its original execution scope gates.
    if (queued) m_pathFeed.capturedFeed = m_cncFeed.candidateFeed;
    else if (!CaptureCncFeedValueSameThread(block, m_pathFeed.capturedFeed))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (!IsCncFeedValueCurrentSameThread(m_pathFeed.capturedFeed))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    const double effectiveFeed = m_pathFeed.capturedFeed.feedMMMin;
    m_pathFeedProgrammed.fill(false);
    m_pathFeedWCS.fill(0.0);
    m_pathFeedCandidate.fill(0.0);
    m_pathFeedAxes.clear();
    m_pathFeedTargets.clear();
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        const char letter = m_axisNames[i];
        if (!block.has(letter)) continue;
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!axis.isExist || axis.axisType != AxisType::LINEAR)
        {
            RejectPathCoreFeedSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        m_pathFeedProgrammed[i] = true;
        m_pathFeedWCS[i] = IsNCPolarAngleAxis(CoordSys.isPolarCoordinateActive, CoordSys.activePlane, static_cast<unsigned>(i)) ? block.val(letter) :
            NCTranslationLengthToMM(block.val(letter), CoordSys.isInchMode ? 20 : 21);
    }
    const bool requirePlanarBaselineMatch = cutter || nominalCutterLine || (CoordSys.activePlane == 17 && CoordSys.IsTranslationRunFrozen() &&
        (NCTranslationHasPlanarRotation(CoordSys.GetTranslationSnapshot()) || CoordSys.isPolarCoordinateActive) &&
        (m_pathFeedProgrammed[0] != m_pathFeedProgrammed[1]));
    const unsigned rawXYMask = (m_pathFeedProgrammed[0] ? 1U : 0U) |
        (m_pathFeedProgrammed[1] ? 2U : 0U);
    if ((CoordSys.isPolarCoordinateActive ||
            IsNCTranslationCutterSparseLineNotationAllowed(CoordSys.activePlane,
                CoordSys.isAbsoluteMode ? 90 : 91, CoordSys.isPolarCoordinateActive ? 16 : 15)) &&
        (cutter || m_cutterLine.leadOutRequired))
    {
        // BASE-PLANE-31: an omitted G91 author delta can still require a
        // physical offset move on that native axis, including G40 X0/Y0.
        // Carry BOTH native endpoints through travel checks and Motion.
        // G91 supplies zero omitted deltas. BASE-PLANE-33 G90 instead
        // retains the absolute author coordinate decoded from the nominal
        // source. Neither may use physical-tail generic completion.
        // BASE-PLANE-18: never let the generic decoder infer a missing word
        // from the already compensated physical tail. The nominal source
        // preview below owns completion. Both native plane axes are selected
        // even if the literal NC block contains only radius OR angle.
        m_pathFeedProgrammed[cutterPlane.u] = true;
        m_pathFeedProgrammed[cutterPlane.v] = true;
    }
    else if (!CoordSys.CompleteFixedPlanarEndpoint(m_pathFeedWCS.data(), m_pathFeedProgrammed.data()))
    {
        RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (requirePlanarBaselineMatch && CoordSys.activePlane == 17)
        RtPrintf("[ROTATION][SPARSE_XY] g=1 rawXYMask=%u effectiveXYMask=3 beforeSubmit=1\n", rawXYMask);
    std::uint32_t programmedMask = 0U;
    for (std::size_t i = 0U; i < 3U; ++i)
        if (m_pathFeedProgrammed[i]) programmedMask |= (1U << static_cast<unsigned>(i));
    if (!CoordSys.isAbsoluteMode)
        RtPrintf("[INCREMENTAL][TARGET] g=1 rawMask=%u effectiveMask=%u beforeSubmit=1\n",
            rawXYMask | (block.has('Z') ? 4U : 0U), static_cast<unsigned>(programmedMask));
    const std::uint32_t physicalMask = queued ?
        FeedQueuedPhysicalMask(programmedMask, FeedPlanarXYMappingAvailable(m_motion)) : programmedMask;
    const std::uint32_t endpointAxisMask = cutter ? cutterPlane.mask : (physicalMask != programmedMask ? programmedMask : 0U);
    if (queued && physicalMask != m_cncFeed.mask)
    {
        RejectCncFeedSameThread("SUBMIT_MAPPING", m_pathFeed.sourceLine);
        return nullptr;
    }
    // Rotated sparse XY now carries both physical endpoints. Unrotated omission
    // keeps the original Motion-owned native start semantics.
    if (CoordSys.isPolarCoordinateActive && (cutter || m_cutterLine.leadOutRequired))
    {
        if (!PreviewCutterPolarEndpointSameThread(block, m_pathFeedCandidate))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
    }
    else if (!CoordSys.isAbsoluteMode && (cutter || m_cutterLine.leadOutRequired))
    {
        if (!PreviewCutterIncrementalEndpointSameThread(block, m_pathFeedCandidate))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
    }
    else if (nominalCutterLine)
    {
        if (!PreviewCutterAbsoluteLineEndpointSameThread(block, m_pathFeedCandidate))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
    }
    else CoordSys.Preview_WCS_to_MCS(m_pathFeedWCS.data(), m_pathFeedProgrammed.data(), m_pathFeedCandidate.data());
    if (!cutter && !m_cutterLine.leadOutRequired)
        FeedPreserveInchReadbackEndpoint(block, CoordSys, m_motion,
            buffered ? &m_cncFeed.tail : nullptr, m_pathFeedCandidate);
    if (cutter)
    {
        std::array<double, 2U> unusedCenter{};
        int unusedDirection = 1;
        if (!BuildCutterContourSameThread(block, m_pathFeed.sourcePC, m_pathFeed.run,
            m_pathFeed.cache, m_pathFeed.dispatch, m_pathFeedCandidate, unusedCenter, unusedDirection))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }

    // The new-plane contour envelope includes its stationary normal axis.
    // It is not a motion axis, but a homed out-of-range normal coordinate
    // must not disappear from the full physical tool-centre path check.
    if (((cutter || (m_cutterLine.leadOutRequired && (!CoordSys.isAbsoluteMode || CoordSys.isPolarCoordinateActive))) &&
            CoordSys.activePlane != 17) || nominalCutterLine)
    {
        const AxisContext& normal = m_motion.GetAxisContext(static_cast<int>(cutterPlane.normal));
        if (!CoordSys.IsTargetWithinSoftwareTravelLimit(normal, m_pathFeedCandidate[cutterPlane.normal]))
        {
            RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(normal,
                AlarmManager::PROGRAMMED_OVER_TRAVEL));
            return nullptr;
        }
    }

    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(m_pathFeedCandidate[i]))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        if ((physicalMask & (1U << static_cast<unsigned>(i))) == 0U) continue;
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (m_pathFeedProgrammed[i] &&
            (!CoordSys.IsTargetWithinSoftwareTravelLimit(axis, m_pathFeedCandidate[i]) ||
                ((cutter || CoordSys.activePlane != 17 || nominalCutterLine) &&
                    !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, CoordSys.commandedMCS[i]))))
        {
            RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(axis, AlarmManager::PROGRAMMED_OVER_TRAVEL));
            return nullptr;
        }
        m_pathFeedAxes.push_back(static_cast<int>(i));
        m_pathFeedTargets.push_back(m_pathFeedProgrammed[i] ? m_pathFeedCandidate[i] : 0.0);
    }
    const bool corner = block.has('Q');
    const double cornerToleranceMM = corner ?
        NCTranslationLengthToMM(block.val('Q'), CoordSys.isInchMode ? 20 : 21) *
            CoordSys.GetTranslationSnapshot().scalingFactor : 0.0;
    if (corner && (!std::isfinite(cornerToleranceMM) || cornerToleranceMM < 0.0001 || cornerToleranceMM > 1.0))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    std::array<double, 8U> cornerNext{};
    MotionArcTravelGuard cornerGuard{};
    double cornerNextFeed = 0.0;
    if (corner)
    {
        const NCProgramCacheLine* next = GetBaseProgramCache().TryGetLine(m_cncFeed.selectedPC + 1);
        // Read the immutable cache, never speculative resolver state or a live
        // servo position. No queue side effect until next-row permission is proven.
        if (!queued || !next || next->parsedBlock.isBlockSkip ||
            !FeedBuildCornerNextGeometry(next->parsedBlock, m_cncFeed.nextCandidate) ||
            !IsPathCoreFeedBlockShapeValid(m_cncFeed.nextCandidate, true, CoordSys.isInchMode ? 20 : 21, CoordSys.isPolarCoordinateActive) ||
            !m_cncFeed.nextCandidate.has('X') || !m_cncFeed.nextCandidate.has('Y') || m_cncFeed.nextCandidate.has('Z'))
        {
            RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter); return nullptr;
        }
        // EI: an omitted next-row F inherits this current source only after
        // its normal EH commit. Geometry planning uses the already validated
        // current value without reading or populating an absent F slot.
        cornerNextFeed = m_cncFeed.nextCandidate.has('F') ?
            NCTranslationLengthToMM(m_cncFeed.nextCandidate.val('F'), CoordSys.isInchMode ? 20 : 21) : effectiveFeed;
        std::array<double, 8U> nextWCS = m_pathFeedWCS;
        nextWCS[0] = NCTranslationLengthToMM(m_cncFeed.nextCandidate.val('X'), CoordSys.isInchMode ? 20 : 21);
        nextWCS[1] = CoordSys.isPolarCoordinateActive ? m_cncFeed.nextCandidate.val('Y') :
            NCTranslationLengthToMM(m_cncFeed.nextCandidate.val('Y'), CoordSys.isInchMode ? 20 : 21);
        // The declared vertex, before Q trimming, is the next polar baseline.
        // The immutable next row has both endpoint words; it is decoded once
        // without sampling a future variable or the moving servo position.
        std::array<bool, 8U> nextProgrammed = m_pathFeedProgrammed;
        if (CoordSys.isPolarCoordinateActive &&
            !TryCompleteNCTranslationPolarEndpoint(CoordSys.GetTranslationSnapshot(),
                m_pathFeedCandidate.data(), nextWCS.data(), nextProgrammed.data()))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter); return nullptr;
        }
        CoordSys.Preview_WCS_to_MCS(nextWCS.data(), m_pathFeedProgrammed.data(), cornerNext.data());
        cornerGuard.context = this;
        cornerGuard.check = [](const void* context, int axis, double value)->bool
        {
            const NCManager* nc = static_cast<const NCManager*>(context);
            return axis >= 0 && axis < 2 && nc->CoordSys.IsTargetWithinSoftwareTravelLimit(nc->m_motion.GetAxisContext(axis), value);
        };
        for (unsigned i = 0U; i < 2U; ++i)
            if (!std::isfinite(cornerNext[i]) || !cornerGuard.check(this, int(i), cornerNext[i]))
            {
                RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(m_motion.GetAxisContext(int(i)), AlarmManager::PROGRAMMED_OVER_TRAVEL)); return nullptr;
            }
    }
    const bool accepted = m_motion.TryG01MoveTransactionalCncTail(m_pathFeedAxes, m_pathFeedTargets,
        effectiveFeed, CoordSys.commandedMCS, m_pathFeedMotion, buffered ? &m_cncFeed.tail : nullptr, queued,
        corner ? &cornerNext : nullptr, cornerToleranceMM, corner ? &cornerGuard : nullptr, cornerNextFeed,
        endpointAxisMask, requirePlanarBaselineMatch, nominalCutterLine);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    const std::uint32_t expectedMask = physicalMask;
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (m_pathFeedProgrammed[i])
        {
            if (FeedDoubleBits(m_pathFeedCandidate[i]) != FeedDoubleBits(corner ? receipt.blendMetadata.vertex[i] : receipt.line.endMCS[i]))
            {
                RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
        }
        else if (FeedDoubleBits(receipt.line.startMCS[i]) != FeedDoubleBits(receipt.line.endMCS[i]) ||
            FeedDoubleBits(receipt.line.startPulse[i]) != FeedDoubleBits(receipt.line.endPulse[i]))
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    if (corner && (!receipt.blendGeometry.valid || receipt.blendGeometry.kind != NCPathCoreRetainedKind::LINE_ARC ||
        receipt.blendMetadata.toleranceMM != cornerToleranceMM ||
        receipt.blendMetadata.next[0] != cornerNext[0] || receipt.blendMetadata.next[1] != cornerNext[1] ||
        receipt.blendMetadata.deviationMM > cornerToleranceMM ||
        FeedDoubleBits(receipt.cornerNextFeedMMMin) != FeedDoubleBits(cornerNextFeed) ||
        FeedDoubleBits(receipt.cornerDispatchFeedMMMin) != FeedDoubleBits((std::min)(effectiveFeed, cornerNextFeed)) ||
        !std::isfinite(receipt.cornerDispatchVelocityPPS) || receipt.cornerDispatchVelocityPPS < 1.0))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return nullptr;
    }
    if (!receipt.valid || !receipt.commandAccepted || !receipt.tailCommitted || !receipt.captureBound ||
        !FeedTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        !receipt.line.valid || receipt.line.axisMask != expectedMask ||
        FeedDoubleBits(receipt.line.feedMMMin) != FeedDoubleBits(effectiveFeed) ||
        (receipt.validAxisMask & expectedMask) != expectedMask ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    if (cutter) LogCutterContourSameThread(m_pathFeed.run, m_pathFeed.dispatch, m_pathFeed.sourcePC);
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    if (queued) return nullptr; // DD ledger Commit binds the receipt before the next NC feedback drain.
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitPathCoreFeedCaptureSameThread(NCBlockDispatchId dispatchId,
    const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
    bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
    int sourcePC, int sourceLine)
{
    if (!m_pathFeed.pending || m_pathFeed.dispatch != dispatchId) return;
    m_pathFeed.sourcePC = sourcePC;
    m_pathFeed.sourceLine = sourceLine;
    const MotionFeedLineReceipt& r = m_pathFeedMotion.receipt;
    if (!IsCncFeedValueCurrentSameThread(m_pathFeed.capturedFeed) ||
        FeedDoubleBits(r.line.feedMMMin) != FeedDoubleBits(m_pathFeed.capturedFeed.feedMMMin) ||
        !r.valid || !committed || !commit.IsValid() || !ledgerFound ||
        capture.count != 1U || capture.overflow || ledger.dispatchId != dispatchId ||
        !ledger.programCommitted || ledger.ncDispatchFailed || ledger.motionCaptureOverflow ||
        ledger.motionSegmentCount != 1U || ledger.sourceLineNumber != sourceLine ||
        sourcePC < 0 || sourceLine <= 0 || commit.sourcePC != sourcePC ||
        commit.scope != NCProgramScope::MEMORY || commit.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
        commit.cacheGeneration != m_pathFeed.cache || !FeedSameSource(commit, ledger.programTarget) ||
        !FeedSameSource(commit, ledger.programCommit) || commit.sequence != ledger.programCommit.sequence ||
        !FeedTranslationCurrent(CoordSys, r.translationGeneration) ||
        capture.submissions[0U].translationGeneration != r.translationGeneration ||
        !capture.submissions[0U].producerAccepted ||
        capture.submissions[0U].immediateRejectReason != MotionRejectReason::NONE ||
        capture.submissions[0U].commandPathMode != (m_cncFeed.selected ?
            MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP) ||
        !ledger.motionSegments[0U].producerAccepted ||
        ledger.motionSegments[0U].immediateRejectReason != MotionRejectReason::NONE ||
        !FeedFullIdentity(capture.submissions[0U].identity, r.identity) ||
        !FeedFullIdentity(ledger.motionSegments[0U].identity, r.identity) ||
        r.identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) ||
        r.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !r.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (CoordSys.toolRadiusMode != 40)
    {
        if (!CommitCutterContourSameThread(m_pathFeed.run, m_pathFeed.cache, dispatchId,
            commit.sequence, r.translationGeneration, sourcePC, r.line.endMCS))
        {
            RejectPathCoreFeedSameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return;
        }
    }
    else if (m_cutterLine.leadOutRequired)
    {
        m_cutterLine = CutterLineState{};
        RtPrintf("[CUTTER][LEAD_OUT_BOUND] pc=%d commit=%llu\n", sourcePC,
            static_cast<unsigned long long>(commit.sequence));
    }
    m_pathFeed.commit = commit.sequence;
    m_pathFeed.bound = true;
    RtPrintf("[CNC-TRANSLATION-PATH] phase=BOUND kind=LINE translationGen=%llu wcs=%d dispatch=%llu epoch=%llu seg=%llu sourcePC=%d\n",
        static_cast<unsigned long long>(r.translationGeneration), CoordSys.GetTranslationSnapshot().wcsCode,
        static_cast<unsigned long long>(dispatchId), static_cast<unsigned long long>(r.identity.epoch),
        static_cast<unsigned long long>(r.identity.segmentId), sourcePC);
    if (m_cncFeed.selected)
    {
        CommitCncFeedSameThread();
        return;
    }
    CommitCncModalFeedSameThread(m_pathFeed.capturedFeed, dispatchId, commit.sequence, sourcePC, sourceLine);
    LogPathCoreFeedSameThread("BOUND");
}

NC_PATH_FEED_NOINLINE
void NCManager::ObservePathCoreFeedFeedbackSameThread(const MotionFeedbackEvent& event,
    bool ledgerAccepted)
{
    ObserveCncFeedFeedbackSameThread(event, ledgerAccepted);
    if (!m_pathFeed.pending || !m_pathFeed.bound ||
        !FeedFullIdentity(event.identity, m_pathFeedMotion.receipt.identity)) return;
    if (!FeedTranslationCurrent(CoordSys, m_pathFeedMotion.receipt.translationGeneration) ||
        !ledgerAccepted || event.sequence == 0ULL || event.sequence <= m_pathFeed.lastSequence ||
        event.owner != m_pathFeedMotion.receipt.ownerLease.owner ||
        event.ownerGeneration != m_pathFeedMotion.receipt.ownerLease.generation)
    {
        RejectPathCoreFeedSameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathFeed.lastSequence = event.sequence;
    switch (event.type)
    {
    case MotionFeedbackType::ACCEPTED:
        if (m_pathFeed.consumerAccepted) break;
        m_pathFeed.consumerAccepted = true;
        ++m_pathFeed.accepted;
        return;
    case MotionFeedbackType::STARTED:
        if (!m_pathFeed.consumerAccepted || m_pathFeed.consumerStarted) break;
        m_pathFeed.consumerStarted = true;
        ++m_pathFeed.started;
        return;
    case MotionFeedbackType::COMPLETED:
        if (!m_pathFeed.consumerAccepted || m_pathFeed.completed || event.rejectReason != MotionRejectReason::NONE || event.errorCode != 0U) break;
        m_pathFeed.completed = true;
        return;
    case MotionFeedbackType::PROGRESS:
    case MotionFeedbackType::HELD:
    case MotionFeedbackType::RESUMED:
        if (m_pathFeed.consumerAccepted) return;
        break;
    default:
        break;
    }
    RejectPathCoreFeedSameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_FEED_NOINLINE
bool NCManager::CompletePathCoreFeedSameThread()
{
    if (!m_pathFeed.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (m_state == NCState::ALARM) return false;
    if (!m_pathFeed.pending)
    {
        if (m_gapWindow.active && (m_gapWindow.budgetProven || m_gapWindow.normalProven) && m_pathFeed.completed &&
            m_pathHold.bound && m_pathFeed.dispatch == m_pathHold.dispatch)
        {
            if (!CompleteGapPathSourceSameThread(true)) return false;
        }
        return m_pathFeed.completed;
    }
    if (IsFeedHoldActive()) return false;
    ValidatePathCoreFeedSameThread();
    if (!m_pathFeed.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (!m_pathFeed.bound || !m_pathFeed.completed || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsGroupDone()) return false;
    if (m_gapWindow.active && m_gapWindow.normalSource && !m_gapWindow.normalProven) return false;
    if (CoordSys.toolRadiusMode == 40)
        RetainPathCoreFeedSameThread(); // Cutter replay is a later stage.
    m_pathFeed.pending = false;
    ++m_pathFeed.done;
    LogPathCoreFeedSameThread("COMPLETED");
    return CompleteGapPathSourceSameThread(true);
}

NC_PATH_FEED_NOINLINE
void NCManager::FinalizePathCoreFeedSameThread() noexcept
{
    m_cutterLine = CutterLineState{};
    ClearCncModalFeedSameThread();
    InvalidateCncFeedSameThread();
    if (m_pathFeed.submitted != 0U) LogPathCoreFeedSameThread("FINALIZED");
    m_pathFeed.armed = false;
    m_pathFeed.invalidatedByGoto = false;
    m_pathFeed.explicitFeed = false;
    m_pathFeedMotion.receipt.valid = false;
}

NC_PATH_FEED_NOINLINE
void NCManager::LogPathCoreFeedSameThread(const char* phase) const noexcept
{
    const PathFeedState& s = m_pathFeed;
    const MotionFeedLineReceipt& r = m_pathFeedMotion.receipt;
    RtPrintf("[PCORE-BX] run=%llu dispatch=%llu phase=%s code=%u pc=%d line=%d commit=%llu pending=%u bound=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
        static_cast<unsigned int>(s.code), s.sourcePC, s.sourceLine,
        static_cast<unsigned long long>(s.commit), s.pending ? 1U : 0U, s.bound ? 1U : 0U);
    RtPrintf("[PCORE-BX-ID] run=%llu dispatch=%llu epoch=%llu seg=%llu sourcePC=%u source=%u owner=%u gen=%u valid=%u producerCode=%u geometryCode=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(r.identity.epoch), static_cast<unsigned long long>(r.identity.segmentId),
        static_cast<unsigned int>(r.identity.sourceBlockId), static_cast<unsigned int>(r.identity.source),
        static_cast<unsigned int>(r.ownerLease.owner), static_cast<unsigned int>(r.ownerLease.generation),
        r.valid ? 1U : 0U, static_cast<unsigned int>(r.code), static_cast<unsigned int>(r.geometryCode));
    RtPrintf("[PCORE-BX-CNT] run=%llu submitted=%u accepted=%u started=%u done=%u rejected=%u failed=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned int>(s.submitted),
        static_cast<unsigned int>(s.accepted), static_cast<unsigned int>(s.started),
        static_cast<unsigned int>(s.done), static_cast<unsigned int>(s.rejected), static_cast<unsigned int>(s.failed));
}

NC_PATH_FEED_NOINLINE
void NCManager::LogPathCoreFeedGeometrySameThread() const noexcept
{
    const NCPathCoreFeedLineV2& g = m_pathFeedMotion.receipt.line;
    RtPrintf("[PCORE-BX-GEO] run=%llu dispatch=%llu mask=%u validMask=%u FBits=%llu lengthMMBits=%llu lengthPulseBits=%llu velocityPPSBits=%llu point=%u\n",
        static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
        static_cast<unsigned int>(g.axisMask), static_cast<unsigned int>(m_pathFeedMotion.receipt.validAxisMask),
        static_cast<unsigned long long>(FeedDoubleBits(g.feedMMMin)),
        static_cast<unsigned long long>(FeedDoubleBits(g.lengthMM)),
        static_cast<unsigned long long>(FeedDoubleBits(g.lengthPulse)),
        static_cast<unsigned long long>(FeedDoubleBits(g.velocityPPS)), g.point ? 1U : 0U);
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        const bool selected = (g.axisMask & (1U << i)) != 0U;
        RtPrintf("[PCORE-BX-AXIS] run=%llu dispatch=%llu axis=%u selected=%u programmed=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned int>(i), selected ? 1U : 0U, m_pathFeedProgrammed[i] ? 1U : 0U,
            static_cast<unsigned long long>(FeedDoubleBits(g.startMCS[i])),
            static_cast<unsigned long long>(FeedDoubleBits(g.endMCS[i])),
            static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedProgrammed[i] ? m_pathFeedCandidate[i] : g.startMCS[i])),
            static_cast<unsigned long long>(FeedDoubleBits(g.startPulse[i])),
            static_cast<unsigned long long>(FeedDoubleBits(g.endPulse[i])));
    }
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsCncPathQueuedBlockShapeValid(const NCBlock& block, int unitsMode, bool polar) noexcept
{
    // Pure shape only: future missing F never samples the modal value.
    return IsPathCoreFeedBlockShapeValid(block, true, unitsMode, polar) ||
        (IsPathCoreArcBlockShapeValid(block, true, unitsMode, polar) && block.has('P') && block.val('P') == 1.0);
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsCncFeedScopeSameThread() noexcept
{
    // BASE58 keeps auxiliary-axis configurations on the explicit XYZ
    // exact-stop lane. Consecutive G01 rows must not opt in automatically.
    for (int axis = 3; axis < 8; ++axis)
        if (m_motion.GetAxisContext(axis).isExist) return false;
    // Fixed rotation retains G90/G17 and the lifecycle gates below. Candidate
    // gates limit motion to XY G01, bounded full-XY Q and P1 planar arcs.
    // BASE-PLANE-2 expands exact-stop feed only, not this XY lookahead lane.
    return CoordSys.activePlane == 17 && CoordSys.toolRadiusMode == 40 && !m_cutterLine.leadOutRequired &&
        CoordSys.isAbsoluteMode && m_pathFeed.armed && !m_cncFeed.faulted && !m_pathFeed.pending &&
        m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY &&
        !m_isSingleBlockEnabled && !m_isG66Active && m_macroStack.empty() &&
        !Homing.IsActive() && !IsFeedHoldActive() && !m_pathArc.pending && !m_pathReplay.pending &&
        !m_pathHold.armed && !m_pathHold.bound && !m_gapDryRun.active &&
        !m_gapPath.active && !m_gapWindow.active && !AlarmManager::GetInstance().HasAlarm() &&
        m_pathFeed.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_pathFeed.cache == GetBaseProgramCache().GetGeneration() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) && IsPathCoreFeedConfigurationValid();
}

NC_PATH_FEED_NOINLINE
bool NCManager::PrepareCncFeedDispatchSameThread(const NCParsedBlock* parsed, int pc)
{
    m_cncFeed.selected = false;
    m_cncFeed.selectedPC = -1;
    m_cncFeed.candidateFeed = CncFeedValueSnapshot{};
    if (m_cncFeed.active && (IsFeedHoldActive() || m_motion.HasPendingSafetyOrRecoveryRequests()))
        return false;
    const bool scope = parsed != nullptr && pc >= 0 && pc < (std::numeric_limits<int>::max)() &&
        IsCncFeedScopeSameThread() && !parsed->isBlockSkip;
    std::uint32_t globalMask = 0U;
    const bool globalScope = scope && MacroSys.GetCurrentDepth() == 0 &&
        m_programMotionLease.owner == MotionOwner::AUTO &&
        !m_motion.HasPendingSafetyOrRecoveryRequests();
    const bool planarXYAvailable = scope && FeedPlanarXYMappingAvailable(m_motion);
    const bool fixedRotation = CoordSys.IsFixedPlanarRotationActive() || CoordSys.isPolarCoordinateActive;
    const bool globalHead = globalScope && FeedGlobalBlockSyntax(*parsed, globalMask) &&
        (!fixedRotation || FeedFixedRotationQueuedSyntaxAllowed(*parsed));
    globalMask = FeedQueuedPhysicalMask(globalMask, planarXYAvailable);
    // A full chain waits before sampling the current global values. Retrying
    // this PC must read the values current at the eventual dispatch.
    if (globalHead && m_cncFeed.active && pc == m_cncFeed.nextPC && globalMask == m_cncFeed.mask &&
        m_cncFeed.count == m_cncFeed.flights.size())
    {
        if (!m_cncFeed.capacityLogged) LogCncFeedSameThread("CAPACITY_WAIT");
        m_cncFeed.capacityLogged = true;
        return false;
    }
    NCExpressionResolveError resolveError = NCExpressionResolveError::NONE;
    bool eligible = scope &&
        (NCPreparedBlockQueueShadow::TryBuildLiteralBlock(*parsed, m_cncFeed.candidate) ||
            (globalHead && NCExpressionResolver::ResolveBlock(*parsed, MathParser,
                m_cncFeed.candidate, resolveError))) &&
        IsCncPathQueuedBlockShapeValid(m_cncFeed.candidate, CoordSys.isInchMode ? 20 : 21, CoordSys.isPolarCoordinateActive) &&
        (!fixedRotation || FeedFixedRotationQueuedBlockAllowed(m_cncFeed.candidate)) &&
        CaptureCncFeedValueSameThread(m_cncFeed.candidate, m_cncFeed.candidateFeed);
    std::uint32_t mask = 0U;
    if (eligible)
    {
        for (std::size_t i = 0U; i < 3U; ++i)
            if (m_cncFeed.candidate.has(m_axisNames[i])) mask |= (1U << static_cast<unsigned>(i));
        if (m_cncFeed.candidate.gCode == 2 || m_cncFeed.candidate.gCode == 3) mask = 3U;
        mask = FeedQueuedPhysicalMask(mask, planarXYAvailable);
    }
    if (m_cncFeed.active)
    {
        if (!eligible || pc != m_cncFeed.nextPC || mask != m_cncFeed.mask)
        {
            const bool drained = DrainCncFeedSameThread();
            const bool markedArc = eligible && (m_cncFeed.candidate.has('Q') ||
                (m_cncFeed.candidate.has('P') && (m_cncFeed.candidate.gCode == 2 || m_cncFeed.candidate.gCode == 3)));
            // A mandatory opt-in arc cannot fall through as an unselected
            // legacy callback. Revisit it on the next NC pass, after drain.
            return drained && !markedArc;
        }
        if (m_cncFeed.count == m_cncFeed.flights.size())
        {
            if (!m_cncFeed.capacityLogged) LogCncFeedSameThread("CAPACITY_WAIT");
            m_cncFeed.capacityLogged = true;
            return false;
        }
        m_cncFeed.capacityLogged = false;
    }
    else
    {
        if (!eligible) return true;
        // A marked arc may also form a one-packet chain; it still stops at
        // an unknown tail. Unmarked arcs never select this new path.
        const bool markedArc = m_cncFeed.candidate.has('Q') || (m_cncFeed.candidate.has('P') &&
            (m_cncFeed.candidate.gCode == 2 || m_cncFeed.candidate.gCode == 3));
        const NCProgramCacheLine* next = GetBaseProgramCache().TryGetLine(pc + 1);
        std::uint32_t nextMask = 0U;
        const bool nextLiteral = next != nullptr && !next->parsedBlock.isBlockSkip &&
            NCPreparedBlockQueueShadow::TryBuildLiteralBlock(next->parsedBlock, m_cncFeed.nextCandidate) &&
            IsCncPathQueuedBlockShapeValid(m_cncFeed.nextCandidate, CoordSys.isInchMode ? 20 : 21, CoordSys.isPolarCoordinateActive) &&
            (!fixedRotation || FeedFixedRotationQueuedBlockAllowed(m_cncFeed.nextCandidate));
        // DS/DU/DV/DW/DX/DY/DZ/EA do not predict future values. A bounded variable G01
        // or P1 arc may justify starting an unmarked line chain, but gains
        // Motion authority only after its own PC is resolved and committed.
        // Q keeps literal next-row XY and any present F, with deferred variable
        // Q. A current P1 arc can start a one-packet chain without a future value.
        const bool nextGlobal = globalScope && !markedArc && next != nullptr &&
            FeedGlobalBlockSyntax(next->parsedBlock, nextMask) &&
            (!fixedRotation || FeedFixedRotationQueuedSyntaxAllowed(next->parsedBlock));
        nextMask = FeedQueuedPhysicalMask(nextMask, planarXYAvailable);
        if (!nextLiteral && !nextGlobal)
        {
            if (!markedArc) return true;
            m_cncFeed.mask = mask;
            m_cncFeed.selected = true;
            m_cncFeed.selectedPC = pc;
            return true;
        }
        if (nextLiteral)
        {
            for (std::size_t i = 0U; i < 3U; ++i)
                if (m_cncFeed.nextCandidate.has(m_axisNames[i])) nextMask |= (1U << static_cast<unsigned>(i));
            if (m_cncFeed.nextCandidate.gCode == 2 || m_cncFeed.nextCandidate.gCode == 3) nextMask = 3U;
            nextMask = FeedQueuedPhysicalMask(nextMask, planarXYAvailable);
        }
        if (mask != nextMask && !markedArc) return true;
        m_cncFeed.mask = mask;
    }
    m_cncFeed.selected = true;
    m_cncFeed.selectedPC = pc;
    return true;
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsCncFeedSelectedBlockSameThread(const NCBlock& block) const noexcept
{
    if (!m_cncFeed.selected || !IsCncPathQueuedBlockShapeValid(block, CoordSys.isInchMode ? 20 : 21, CoordSys.isPolarCoordinateActive) ||
        ((CoordSys.IsFixedPlanarRotationActive() || CoordSys.isPolarCoordinateActive) && !FeedFixedRotationQueuedBlockAllowed(block)) ||
        !IsCncFeedValueCurrentSameThread(m_cncFeed.candidateFeed) ||
        m_cncFeed.candidateFeed.programmed != block.has('F') ||
        (block.has('F') && FeedDoubleBits(NCTranslationLengthToMM(block.val('F'), CoordSys.isInchMode ? 20 : 21)) != FeedDoubleBits(m_cncFeed.candidateFeed.feedMMMin)) ||
        block.gCode != m_cncFeed.candidate.gCode ||
        block.isBlockSkip || m_cncFeed.selectedPC < 0) return false;
    for (int i = 0; i < 26; ++i)
        if (block.hasParam[i] != m_cncFeed.candidate.hasParam[i] ||
            (block.hasParam[i] && FeedDoubleBits(block.param[i]) != FeedDoubleBits(m_cncFeed.candidate.param[i])))
            return false;
    return true;
}

NC_PATH_FEED_NOINLINE
bool NCManager::DrainCncFeedSameThread()
{
    if (!m_cncFeed.active) return true;
    if (m_cncFeed.count != 0U || m_pathFeed.pending || m_pathArc.pending || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        !m_motion.IsGroupNCDrained())
    {
        if (!m_cncFeed.drainLogged) LogCncFeedSameThread("DRAIN_WAIT");
        m_cncFeed.drainLogged = true;
        return false;
    }
    LogCncFeedSameThread("DRAINED");
    m_cncFeed.active = false;
    m_cncFeed.selected = false;
    m_cncFeed.drainLogged = false;
    m_cncFeed.capacityLogged = false;
    m_cncFeed.nextPC = -1;
    m_cncFeed.tail.Clear();
    return true;
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitCncFeedSameThread()
{
    if (!m_cncFeed.selected || !m_pathFeed.pending || !m_pathFeed.bound ||
        !IsCncFeedValueCurrentSameThread(m_pathFeed.capturedFeed) ||
        FeedDoubleBits(m_pathFeedMotion.receipt.line.feedMMMin) != FeedDoubleBits(m_pathFeed.capturedFeed.feedMMMin) ||
        m_pathFeed.sourcePC != m_cncFeed.selectedPC || m_cncFeed.count >= m_cncFeed.flights.size() ||
        m_pathFeedMotion.receipt.line.axisMask != m_cncFeed.mask ||
        !FeedTranslationCurrent(CoordSys, m_pathFeedMotion.receipt.translationGeneration) ||
        (m_cncFeed.active && (m_pathFeed.sourcePC != m_cncFeed.nextPC ||
            m_pathFeedMotion.receipt.identity.epoch != m_cncFeed.tail.identity.epoch ||
            m_pathFeedMotion.receipt.translationGeneration != m_cncFeed.tail.translationGeneration ||
            !m_pathFeedMotion.receipt.ownerLease.Matches(m_cncFeed.tail.ownerLease))))
    {
        RejectCncFeedSameThread("COMMIT_BINDING", m_pathFeed.sourceLine);
        return;
    }
    if (!m_cncFeed.active)
    {
        m_cncFeed.active = true;
        ++m_cncFeed.chain;
        m_cncFeed.run = m_pathFeed.run;
        m_cncFeed.cache = m_pathFeed.cache;
        m_cncFeed.head = 0U;
    }
    CncFeedFlight& row = m_cncFeed.flights[(m_cncFeed.head + m_cncFeed.count) % m_cncFeed.flights.size()];
    row = CncFeedFlight{};
    row.receipt = m_pathFeedMotion.receipt;
    row.dispatch = m_pathFeed.dispatch;
    row.commit = m_pathFeed.commit;
    row.pc = m_pathFeed.sourcePC;
    row.line = m_pathFeed.sourceLine;
    row.lastSequence = m_lastConsumedMotionFeedbackSequence;
    m_cncFeed.tail.Assign(row.receipt);
    m_cncFeed.nextPC = row.pc + 1;
    ++m_cncFeed.count;
    ++m_cncFeed.submitted;
    if (m_cncFeed.count > m_cncFeed.peak) m_cncFeed.peak = m_cncFeed.count;
    m_pathFeed.pending = false;
    CommitCncModalFeedSameThread(m_pathFeed.capturedFeed, row.dispatch, row.commit, row.pc, row.line);
    LogCncFeedSameThread("SUBMITTED", &row);
    LogPathCoreFeedGeometrySameThread();
    if (row.receipt.blendGeometry.valid)
    {
        const auto& b = row.receipt.blendMetadata;
        const auto& g = row.receipt.blendGeometry;
        RtPrintf("[CNC-DI-FEED] run=%llu dispatch=%llu epoch=%llu seg=%llu pc=%d owner=%u gen=%u sourceFBits=%llu nextFBits=%llu capFBits=%llu capPPSBits=%llu policy=MIN_INCIDENT_ARC_SEAM\n",
            static_cast<unsigned long long>(m_cncFeed.run), static_cast<unsigned long long>(row.dispatch),
            static_cast<unsigned long long>(row.Identity().epoch), static_cast<unsigned long long>(row.Identity().segmentId), row.pc,
            static_cast<unsigned int>(row.OwnerLease().owner), static_cast<unsigned int>(row.OwnerLease().generation),
            static_cast<unsigned long long>(FeedDoubleBits(row.receipt.line.feedMMMin)),
            static_cast<unsigned long long>(FeedDoubleBits(row.receipt.cornerNextFeedMMMin)),
            static_cast<unsigned long long>(FeedDoubleBits(row.receipt.cornerDispatchFeedMMMin)),
            static_cast<unsigned long long>(FeedDoubleBits(row.receipt.cornerDispatchVelocityPPS)));
        RtPrintf("[CNC-DH-GEO] run=%llu dispatch=%llu seg=%llu pc=%d QBits=%llu deviationBits=%llu trimBits=%llu vertexXBits=%llu vertexYBits=%llu entryXBits=%llu entryYBits=%llu exitXBits=%llu exitYBits=%llu radiusBits=%llu sweepBits=%llu prefixBits=%llu totalBits=%llu kind=LINE_ARC\n",
            static_cast<unsigned long long>(m_cncFeed.run), static_cast<unsigned long long>(row.dispatch),
            static_cast<unsigned long long>(row.Identity().segmentId), row.pc,
            static_cast<unsigned long long>(FeedDoubleBits(b.toleranceMM)), static_cast<unsigned long long>(FeedDoubleBits(b.deviationMM)),
            static_cast<unsigned long long>(FeedDoubleBits(b.trimMM)), static_cast<unsigned long long>(FeedDoubleBits(b.vertex[0])),
            static_cast<unsigned long long>(FeedDoubleBits(b.vertex[1])), static_cast<unsigned long long>(FeedDoubleBits(b.entry[0])),
            static_cast<unsigned long long>(FeedDoubleBits(b.entry[1])), static_cast<unsigned long long>(FeedDoubleBits(b.exit[0])),
            static_cast<unsigned long long>(FeedDoubleBits(b.exit[1])), static_cast<unsigned long long>(FeedDoubleBits(g.radiusMM)),
            static_cast<unsigned long long>(FeedDoubleBits(g.sweepRadians)), static_cast<unsigned long long>(FeedDoubleBits(row.receipt.line.lengthMM)),
            static_cast<unsigned long long>(FeedDoubleBits(g.lengthMM)));
    }
    m_cncFeed.selected = false;
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitCncArcSameThread()
{
    const MotionFeedArcReceipt& receipt = m_pathArcMotion.receipt;
    if (!m_cncFeed.selected || !m_pathArc.pending || !m_pathArc.bound ||
        !IsCncFeedValueCurrentSameThread(m_pathArc.capturedFeed) ||
        FeedDoubleBits(receipt.arc.feedMMMin) != FeedDoubleBits(m_pathArc.capturedFeed.feedMMMin) ||
        m_pathArc.sourcePC != m_cncFeed.selectedPC || m_cncFeed.count >= m_cncFeed.flights.size() ||
        !receipt.valid || receipt.arc.axisMask != m_cncFeed.mask ||
        !FeedTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        (m_cncFeed.active && (m_pathArc.sourcePC != m_cncFeed.nextPC ||
            receipt.identity.epoch != m_cncFeed.tail.identity.epoch ||
            receipt.translationGeneration != m_cncFeed.tail.translationGeneration ||
            !receipt.ownerLease.Matches(m_cncFeed.tail.ownerLease))))
    {
        RejectCncFeedSameThread("ARC_COMMIT_BINDING", m_pathArc.sourceLine);
        return;
    }
    if (!m_cncFeed.active)
    {
        m_cncFeed.active = true; ++m_cncFeed.chain;
        m_cncFeed.run = m_pathArc.run; m_cncFeed.cache = m_pathArc.cache; m_cncFeed.head = 0U;
    }
    CncFeedFlight& row = m_cncFeed.flights[(m_cncFeed.head + m_cncFeed.count) % m_cncFeed.flights.size()];
    row = CncFeedFlight{}; row.arc = true; row.arcReceipt = receipt;
    row.dispatch = m_pathArc.dispatch; row.commit = m_pathArc.commit;
    row.pc = m_pathArc.sourcePC; row.line = m_pathArc.sourceLine;
    row.lastSequence = m_lastConsumedMotionFeedbackSequence;
    m_cncFeed.tail.endMCS = receipt.arc.endMCS; m_cncFeed.tail.endPulse = receipt.arc.endPulse;
    m_cncFeed.tail.identity = receipt.identity; m_cncFeed.tail.ownerLease = receipt.ownerLease;
    m_cncFeed.tail.translationGeneration = receipt.translationGeneration;
    m_cncFeed.tail.axisMask = receipt.arc.axisMask;
    m_cncFeed.tail.validAxisMask = receipt.validAxisMask;
    m_cncFeed.tail.valid = receipt.valid && receipt.commandAccepted && receipt.captureBound &&
        receipt.tailCommitted && receipt.arc.valid;
    m_cncFeed.nextPC = row.pc + 1;
    ++m_cncFeed.count; ++m_cncFeed.submitted;
    if (m_cncFeed.count > m_cncFeed.peak) m_cncFeed.peak = m_cncFeed.count;
    m_pathArc.pending = false;
    CommitCncModalFeedSameThread(m_pathArc.capturedFeed, row.dispatch, row.commit, row.pc, row.line);
    LogCncFeedSameThread("SUBMITTED", &row);
    LogPathCoreArcGeometrySameThread();
    m_cncFeed.selected = false;
}

NC_PATH_FEED_NOINLINE
void NCManager::ObserveCncFeedFeedbackSameThread(const MotionFeedbackEvent& event, bool ledgerAccepted)
{
    if (!m_cncFeed.active) return;
    for (std::size_t i = 0U; i < m_cncFeed.count; ++i)
    {
        CncFeedFlight& row = m_cncFeed.flights[(m_cncFeed.head + i) % m_cncFeed.flights.size()];
        if (!FeedFullIdentity(event.identity, row.Identity())) continue;
        const std::uint64_t translationGeneration = row.arc ?
            row.arcReceipt.translationGeneration : row.receipt.translationGeneration;
        if (!FeedTranslationCurrent(CoordSys, translationGeneration) ||
            translationGeneration != m_cncFeed.tail.translationGeneration ||
            !ledgerAccepted || event.sequence == 0ULL || event.sequence <= row.lastSequence ||
            event.owner != row.OwnerLease().owner ||
            event.ownerGeneration != row.OwnerLease().generation)
        {
            RejectCncFeedSameThread("FEEDBACK_IDENTITY", row.line);
            return;
        }
        row.lastSequence = event.sequence;
        if (event.type == MotionFeedbackType::ACCEPTED && !row.accepted)
        {
            row.accepted = true;
            ++m_cncFeed.accepted;
            if (row.arc) ++m_pathArc.accepted; else ++m_pathFeed.accepted;
            LogCncFeedSameThread("ACCEPTED", &row);
            return;
        }
        if (event.type == MotionFeedbackType::STARTED && row.accepted && !row.started && i == 0U)
        {
            row.started = true;
            ++m_cncFeed.started;
            if (row.arc) ++m_pathArc.started; else ++m_pathFeed.started;
            LogCncFeedSameThread("STARTED", &row);
            return;
        }
        if (event.type == MotionFeedbackType::COMPLETED && row.accepted && i == 0U &&
            event.rejectReason == MotionRejectReason::NONE && event.errorCode == 0U)
        {
            // Exact terminal feedback replaces the old whole-group wait for this row only.
            // Later rows are immutable values and cannot overwrite this completed geometry.
            if (m_pathReplay.armed && m_pathReplayStore.Fault() == NCPathCoreRetainedFault::NONE)
            {
                if (((row.arc || !row.receipt.line.point) && !row.started) ||
                    !(row.arc ? BuildNCPathCoreRetainedArc(row.arcReceipt.arc, m_pathReplayGeometry) :
                        (row.receipt.blendGeometry.valid ?
                            (m_pathReplayGeometry = row.receipt.blendGeometry, IsNCPathCoreRetainedGeometryValid(m_pathReplayGeometry)) :
                            BuildNCPathCoreRetainedLine(row.receipt.line, m_pathReplayGeometry))))
                {
                    m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
                    LogPathCoreReplaySameThread("SAVE_INVALIDATED");
                }
                else
                    AppendPathCoreReplayGeometrySameThread(row.Identity(), row.ValidAxisMask(),
                        row.dispatch, row.commit, translationGeneration, row.pc, row.line);
            }
            ++m_cncFeed.done;
            if (row.arc) ++m_pathArc.done; else ++m_pathFeed.done;
            LogCncFeedSameThread("COMPLETED", &row);
            if (!row.arc && m_pathFeed.dispatch == row.dispatch)
            {
                m_pathFeed.completed = true;
                m_pathFeed.consumerAccepted = row.accepted;
                m_pathFeed.consumerStarted = row.started;
            }
            if (row.arc && m_pathArc.dispatch == row.dispatch)
            {
                m_pathArc.completed = true;
                m_pathArc.consumerAccepted = row.accepted;
                m_pathArc.consumerStarted = row.started;
            }
            row.Invalidate();
            m_cncFeed.head = static_cast<std::uint32_t>((m_cncFeed.head + 1U) % m_cncFeed.flights.size());
            --m_cncFeed.count;
            return;
        }
        if (row.accepted && i == 0U && (event.type == MotionFeedbackType::PROGRESS ||
            event.type == MotionFeedbackType::HELD || event.type == MotionFeedbackType::RESUMED)) return;
        RejectCncFeedSameThread("FEEDBACK_ORDER_OR_TERMINAL", row.line);
        return;
    }
}

NC_PATH_FEED_NOINLINE
void NCManager::ValidateCncFeedSameThread()
{
    if (!m_cncFeed.active) return;
    const NCState state = m_state.load(std::memory_order_acquire);
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() ||
        m_cncFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_cncFeed.cache != GetBaseProgramCache().GetGeneration() || !m_macroStack.empty() ||
        Homing.IsActive() || !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        InvalidateCncFeedSameThread();
        return;
    }
    if (CoordSys.activePlane != 17 || !IsPathCoreFeedConfigurationValid() ||
        !m_cncFeed.tail.valid || m_cncFeed.tail.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !FeedTranslationCurrent(CoordSys, m_cncFeed.tail.translationGeneration) ||
        !m_cncFeed.tail.ownerLease.Matches(m_programMotionLease) ||
        m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
        RejectCncFeedSameThread("LIVE_SCOPE_OR_TRANSPORT", m_pathFeed.sourceLine);
}

NC_PATH_FEED_NOINLINE
void NCManager::InvalidateCncFeedSameThread() noexcept
{
    ClearCncModalFeedSameThread();
    // NC receipts only. CL_FIX1 still owns when Motion geometry may be cleared.
    for (std::size_t i = 0U; i < m_cncFeed.count; ++i)
    {
        CncFeedFlight& row = m_cncFeed.flights[(m_cncFeed.head + i) % m_cncFeed.flights.size()];
        ++m_cncFeed.revoked;
        LogCncFeedSameThread("REVOKED", &row);
        row.Invalidate();
    }
    m_cncFeed.active = false;
    m_cncFeed.selected = false;
    m_cncFeed.head = m_cncFeed.count = 0U;
    m_cncFeed.tail.Clear();
    m_cncFeed.nextPC = m_cncFeed.selectedPC = -1;
    m_cncFeed.capacityLogged = m_cncFeed.drainLogged = false;
}

NC_PATH_FEED_NOINLINE
void NCManager::RejectCncFeedSameThread(const char* reason, int line)
{
    if (m_cncFeed.faulted) return;
    m_cncFeed.faulted = true;
    ++m_cncFeed.failures;
    LogCncFeedSameThread(reason);
    AlarmManager::GetInstance().Trigger(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY, line);
    ChangeState(NCState::ALARM);
}

NC_PATH_FEED_NOINLINE
void NCManager::LogCncFeedSameThread(const char* phase, const CncFeedFlight* row) noexcept
{
    const MotionExecutionIdentity& identity = row != nullptr ? row->Identity() : m_cncFeed.tail.identity;
    const MotionOwnerLease& lease = row != nullptr ? row->OwnerLease() : m_cncFeed.tail.ownerLease;
    RtPrintf("[CNC-DD] event=%s seq=%llu run=%llu chain=%llu cache=%llu epoch=%llu seg=%llu owner=%u gen=%u dispatch=%llu commit=%llu pc=%d line=%d mask=%u live=%u peak=%u submitted=%u accepted=%u started=%u done=%u revoked=%u fail=%u path=CONTINUOUS planner=DI kind=%s\n",
        phase, static_cast<unsigned long long>(++m_cncFeed.eventSequence),
        static_cast<unsigned long long>(m_cncFeed.run), static_cast<unsigned long long>(m_cncFeed.chain),
        static_cast<unsigned long long>(m_cncFeed.cache), static_cast<unsigned long long>(identity.epoch),
        static_cast<unsigned long long>(identity.segmentId), static_cast<unsigned int>(lease.owner),
        static_cast<unsigned int>(lease.generation), static_cast<unsigned long long>(row != nullptr ? row->dispatch : 0ULL),
        static_cast<unsigned long long>(row != nullptr ? row->commit : 0ULL), row != nullptr ? row->pc : m_cncFeed.nextPC,
        row != nullptr ? row->line : 0, static_cast<unsigned int>(m_cncFeed.mask),
        static_cast<unsigned int>(m_cncFeed.count), static_cast<unsigned int>(m_cncFeed.peak),
        static_cast<unsigned int>(m_cncFeed.submitted), static_cast<unsigned int>(m_cncFeed.accepted),
        static_cast<unsigned int>(m_cncFeed.started), static_cast<unsigned int>(m_cncFeed.done),
        static_cast<unsigned int>(m_cncFeed.revoked), static_cast<unsigned int>(m_cncFeed.failures), row == nullptr ? "TAIL" : row->arc ? "ARC" : row->receipt.blendGeometry.valid ? "BLEND" : "LINE");
}

#undef NC_PATH_FEED_NOINLINE
