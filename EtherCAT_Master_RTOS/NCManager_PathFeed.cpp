// BX / Path Core V2-01. G01 is a distinct feed producer, not a G00 override.
#include "NCManager.h"
#include "AlarmManager.h"
#include "GlobalConfig.h"
#include "NCPathCoreRadiusArc.h"
#include "NCTranslationArcPrecision.h"
#include "NCExpressionResolver.h"
#include "NCRotaryFeedScope.h"
#include "NCZCFeedScope.h"
#include "NCXYZCFeedScope.h"
#include "NCXYZCUVFeedScope.h"
#include "EDM149VoltageMotion.h"
#include "NCEccentricCFeedScope.h"
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
    if (!m_pathFeed.armed || !m_pathFeed.explicitFeed || block.hasG || block.gCount != 0) return false;
    if (block.has('X') || block.has('Y') || block.has('Z') || block.has('F')) return true;
    // BASE68 keeps G01 explicit on every native rotary row as well.
    const NCAxisIdentitySnapshot& axes = CoordSys.GetTranslationSnapshot().axisIdentity;
    for (unsigned axis = 3U; axis < 8U; ++axis)
        if (axes.address[axis] >= 'A' && axes.address[axis] <= 'Z' && block.has(axes.address[axis])) return true;
    return false;
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
    if (translation.axisIdentity.eccentricEnabled != 0U && translation.toolLengthMode != 49)
        return IsNCEccentricCFeedFrame(translation) && CoordSys.IsTranslationRunFrozen() &&
            CoordSys.IsTranslationRunCurrent() && IsPathCoreLiveNativeConfigCurrentSameThread() &&
            IsEccentricCFeedProfileCurrentSameThread();
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
    m_g76Orbit->active=false; m_g76Orbit->bound=false; m_g76Orbit->completed=false;
    m_g76Modal=G76ModalState{}; m_g76Confirmation=G76Confirmation{};
    m_edm171Air->history.Clear(); m_edm171Air->configured=m_edm171Air->selected=m_edm171Air->bound=false;
    m_edm171Air->e6Ready=false; m_edm171Air->e6Catalog=nullptr;
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
    // Keep the separate power latch until the old physical work retires.
    m_g76Orbit->active=false; m_g76Orbit->bound=false; m_g76Orbit->completed=false;
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
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    if (static_cast<unsigned>(receipt.eccentricFeed) + static_cast<unsigned>(receipt.rotaryFeed) + static_cast<unsigned>(receipt.zcFeed) +
            static_cast<unsigned>(receipt.xyzcFeed) + static_cast<unsigned>(receipt.xyzcuvFeed) > 1U)
    {
        RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (receipt.eccentricFeed && (!IsEccentricCFeedReceiptCurrentSameThread() ||
        !IsFixedTranslationTravelCurrentSameThread() || m_pathArc.pending || m_pathReplay.pending ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active))
    {
        RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if ((receipt.rotaryFeed || receipt.zcFeed || receipt.xyzcFeed || receipt.xyzcuvFeed) &&
        ((receipt.xyzcuvFeed ? (!IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) ||
            receipt.xyzcuv.absolute != (CoordSys.GetTranslationSnapshot().distanceMode == 90) ||
            receipt.xyzcuv.zOnlyCommissioning != IsEDM149ZFeedSameThread() ||
            receipt.xyzcuv.xyzOnlyCommissioning != IsEDM150XYZFeedSameThread() ||
            receipt.xyzcuv.sparseCommissioning != IsEDM154SparseFeedSameThread() ||
            !IsXYZCUVFeedProfileCurrentSameThread()) : receipt.xyzcFeed ? (!IsNCXYZCFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) ||
            receipt.xyzc.absolute != (CoordSys.GetTranslationSnapshot().distanceMode == 90) ||
            !IsXYZCFeedProfileCurrentSameThread()) : receipt.zcFeed ? (!IsNCZCFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) ||
            receipt.zc.absolute != (CoordSys.GetTranslationSnapshot().distanceMode == 90)) :
            !IsNCRotaryFeedNeutralFrame(CoordSys.GetTranslationSnapshot())) ||
        !IsFixedTranslationTravelCurrentSameThread() || m_cncFeed.selected || m_cncFeed.active ||
        m_cncFeed.count != 0U || m_pathArc.pending || m_pathReplay.pending ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active))
    {
        RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
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
    // An accepted curved packet may already be visible to RT. Revoke it
    // through the existing safety owner/epoch path on any late NC proof fault.
    if (m_pathFeedMotion.receipt.eccentricFeed && m_pathFeedMotion.receipt.commandAccepted)
        m_motion.RequestEmergencyStopAllAxes();
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsEccentricCFeedReceiptCurrentSameThread() const noexcept
{
    const MotionFeedLineReceipt& r = m_pathFeedMotion.receipt;
    if (!m_eccentricProducer || !r.eccentricFeed || r.rotaryFeed || r.zcFeed ||
        r.xyzcFeed || r.xyzcuvFeed || !r.valid || !r.commandAccepted ||
        !r.captureBound || !r.tailCommitted || !m_eccentricProducer->runtime.IsValid()) return false;
    const auto& w = *m_eccentricProducer;
    const auto& p = w.runtime.AuthoredPath();
    const auto& source = CoordSys.GetTranslationSnapshot();
    const AxisContext& cAxis = m_motion.GetAxisContext(3);
    NCEccentricCFeedTarget resolved{};
    NCEccentricCZFeedTarget resolvedZ{};
    std::array<NCEccentricCXYFeedTarget, 2U> resolvedXY{};
    const std::uint32_t linearMask = (m_pathFeedProgrammed[0] ? 1U : 0U) |
        (m_pathFeedProgrammed[1] ? 2U : 0U) | (m_pathFeedProgrammed[2] ? 4U : 0U);
    const bool xy = (linearMask & 3U) != 0U;
    const bool zc = (linearMask & 4U) != 0U;
    double angularFeed = 0.0;
    const std::uint32_t authoredMask = linearMask | 8U;
    // BASE79M: resolve G90 X/Y from the original physical anchor after
    // undoing its rotated H. G91 keeps literal nominal increments. Rebuild
    // the feed from authored XYZ length, never from the compensated chord.
    // Zero absolute targets are legal; zero resolved authored moves are not.
    for (unsigned a = 0U; a < 3U; ++a)
        if ((!m_pathFeedProgrammed[a] && FeedDoubleBits(m_pathFeedWCS[a]) != FeedDoubleBits(0.0)) ||
            (xy && m_pathFeedProgrammed[a] && (!std::isfinite(m_pathFeedWCS[a]) ||
                (source.distanceMode == 91 && m_pathFeedWCS[a] == 0.0))))
            return false;
    if (!IsNCEccentricCFeedFrame(source) || !IsEccentricCFeedProfileCurrentSameThread() ||
        !m_pathFeedProgrammed[3] ||
        !TryResolveNCEccentricCFeedTarget(source, m_pathFeedWCS[3],
            w.input.startPulse[3], w.input.pulsePerNative[3],
            cAxis.useShortestPath, cAxis.rotaryModulo, resolved) ||
        (zc && !TryResolveNCEccentricCZFeedTarget(source, m_pathFeedWCS[2],
            w.input.startPulse[2], w.input.pulsePerNative[2], resolvedZ))) return false;
    for (unsigned a = 0U; a < 2U; ++a)
        if (m_pathFeedProgrammed[a] && !TryResolveNCEccentricCXYFeedTarget(source, a,
            m_pathFeedWCS[a], w.input.startPulse[a], w.input.pulsePerNative[a],
            w.input.startPulse[3] / w.input.pulsePerNative[3], resolved.endMCS,
            resolvedXY[a])) return false;
    const double xDelta = m_pathFeedProgrammed[0] ? resolvedXY[0].deltaMM : 0.0;
    const double yDelta = m_pathFeedProgrammed[1] ? resolvedXY[1].deltaMM : 0.0;
    if (!TryResolveNCEccentricCLinearFeedRate(xDelta, yDelta, zc ? resolvedZ.deltaMM : 0.0,
            resolved.sweepDeg, m_pathFeed.eccentricProgrammedFeed, angularFeed) ||
        !SameNCTranslationSnapshot(source, p.source) || !SameNCTranslationSnapshot(source, w.command.sourceTranslation) ||
        !FeedFullIdentity(r.identity, w.command.execution) || !r.ownerLease.Matches(w.command.ownerLease) ||
        r.translationGeneration != source.generation || p.electrodeAxis != 3U || p.authoredMask != authoredMask ||
        p.generatedMask != (p.radiusMM > 0.0 ? 3U : 0U) || p.groupMask != (authoredMask | p.generatedMask) ||
        w.command.dir != (xy ? 3 : zc ? 2 : 0) ||
        (r.validAxisMask & p.groupMask) != p.groupMask || !p.valid ||
        !std::isfinite(p.sweepDeg) || p.sweepDeg == 0.0 || std::fabs(p.sweepDeg) > NCEccentricCFeedMaximumSweepDeg ||
        !std::isfinite(p.feedDegMin) || p.feedDegMin <= 0.0 || p.feedDegMin > NCEccentricCFeedMaximumFeedDegMin ||
        p.radiusMM > NCEccentricCFeedMaximumRadiusMM ||
        FeedDoubleBits(p.xDeltaMM) != FeedDoubleBits(xDelta) ||
        FeedDoubleBits(p.yDeltaMM) != FeedDoubleBits(yDelta) ||
        FeedDoubleBits(w.input.geometry.xDeltaMM) != FeedDoubleBits(xDelta) ||
        FeedDoubleBits(w.input.geometry.yDeltaMM) != FeedDoubleBits(yDelta) ||
        FeedDoubleBits(w.command.centerPos[0]) != FeedDoubleBits(xDelta) ||
        FeedDoubleBits(w.command.centerPos[1]) != FeedDoubleBits(yDelta) ||
        FeedDoubleBits(p.zDeltaMM) != FeedDoubleBits(zc ? resolvedZ.deltaMM : 0.0) ||
        FeedDoubleBits(w.input.geometry.zDeltaMM) != FeedDoubleBits(zc ? resolvedZ.deltaMM : 0.0) ||
        FeedDoubleBits(p.feedDegMin) != FeedDoubleBits(angularFeed) ||
        FeedDoubleBits(w.input.geometry.feedDegMin) != FeedDoubleBits(angularFeed) ||
        FeedDoubleBits(p.endMCS[2]) != FeedDoubleBits(zc ? resolvedZ.endMCS : p.startMCS[2]) ||
        FeedDoubleBits(w.runtime.EndPulse()[2]) !=
            FeedDoubleBits(zc ? resolvedZ.endPulse : w.input.startPulse[2]) ||
        FeedDoubleBits(p.sweepDeg) != FeedDoubleBits(resolved.sweepDeg) ||
        FeedDoubleBits(p.endMCS[3]) != FeedDoubleBits(resolved.endMCS) ||
        FeedDoubleBits(w.runtime.EndPulse()[3]) != FeedDoubleBits(resolved.endPulse) ||
        m_pathFeed.capturedFeed.valid || m_cncFeed.selected || m_cncFeed.active || m_cncFeed.count != 0U)
        return false;
    if (xy)
    {
        std::array<double, 3U> startOffset{}, endOffset{};
        if (!NCEccentricCDetail::Offset(source, w.input.startPulse[3] / w.input.pulsePerNative[3], startOffset) ||
            !NCEccentricCDetail::Offset(source, resolved.endMCS, endOffset)) return false;
        for (unsigned a = 0U; a < 2U; ++a)
        {
            if ((p.groupMask & (1U << a)) == 0U) continue;
            const double delta = (a == 0U ? xDelta : yDelta) + (endOffset[a] - startOffset[a]);
            const double endMCS = p.startMCS[a] + delta;
            const double endPulse = w.input.startPulse[a] + delta * w.input.pulsePerNative[a];
            if (!std::isfinite(delta) || !std::isfinite(endMCS) || !std::isfinite(endPulse) ||
                FeedDoubleBits(p.endMCS[a]) != FeedDoubleBits(endMCS) ||
                FeedDoubleBits(w.runtime.EndPulse()[a]) != FeedDoubleBits(endPulse)) return false;
            if (m_pathFeedProgrammed[a] &&
                (FeedDoubleBits(p.endMCS[a]) != FeedDoubleBits(resolvedXY[a].endMCS) ||
                    FeedDoubleBits(w.runtime.EndPulse()[a]) != FeedDoubleBits(resolvedXY[a].endPulse)))
                return false;
        }
    }
    for (unsigned a = 0U; a < 8U; ++a)
        if ((a >= 4U && m_pathFeedProgrammed[a]) ||
            !std::isfinite(w.physicalEndMCS[a]) ||
            FeedDoubleBits(CoordSys.commandedMCS[a]) != FeedDoubleBits(w.physicalEndMCS[a])) return false;
    return true;
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreEccentricCFeedSameThread(const NCBlock& block)
{
    const NCTranslationSnapshot& source = CoordSys.GetTranslationSnapshot();
    if (!IsNCEccentricCFeedBlockAllowed(source, block))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (int a = 0; a < 8; ++a)
        if (CoordSys.GetInvalidSoftwareTravelLimitMask(m_motion.GetAxisContext(a)) != 0U)
        {
            RejectPathCoreFeedSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
            return nullptr;
        }
    if (!m_eccentricProducer || !m_pathFeed.armed || m_pathFeed.pending || Close_System_Com_flag || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        IsFeedHoldActive() || !IsPathCoreFeedConfigurationValid() || !IsEccentricCFeedProfileCurrentSameThread() ||
        !IsFixedTranslationTravelCurrentSameThread() ||
        !FeedTranslationCurrent(CoordSys, source.generation) || !m_motion.MatchesNCTranslation(source) ||
        AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_cncFeed.selected || m_cncFeed.active || m_cncFeed.count != 0U ||
        m_pathArc.pending || m_pathReplay.pending || m_pathReplayStore.Pending() ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active ||
        m_cutterLine.valid || m_cutterLine.leadOutRequired)
    {
        RejectPathCoreFeedSameThread(3U, (!m_pathFeed.armed && m_pathFeed.invalidatedByGoto) ?
            AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    {
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    ClearCncModalFeedSameThread();
    ClearPathCoreReplayHistorySameThread();
    m_pathFeedWCS.fill(0.0);
    m_pathFeedProgrammed.fill(false);
    std::array<double, 3U> programmedXYZ{};
    std::uint32_t linearMask = 0U;
    for (unsigned a = 0U; a < 3U; ++a)
    {
        const char word = a == 0U ? 'X' : a == 1U ? 'Y' : 'Z';
        m_pathFeedProgrammed[a] = block.has(word);
        m_pathFeedWCS[a] = programmedXYZ[a] = block.has(word) ? block.val(word) : 0.0;
        if (block.has(word)) linearMask |= 1U << a;
    }
    m_pathFeedProgrammed[3] = true;
    m_pathFeedWCS[3] = block.val('C');
    m_pathFeed.eccentricProgrammedFeed = block.val('F');
    const bool accepted = (linearMask & 3U) != 0U ?
        m_motion.TryG01EccentricXYZCMoveTransactionalTail(block.val('C'),
            m_pathFeed.eccentricProgrammedFeed, CoordSys.commandedMCS, *m_eccentricProducer,
            m_pathFeedMotion.receipt, programmedXYZ, linearMask) :
        m_motion.TryG01EccentricCMoveTransactionalTail(block.val('C'),
            m_pathFeed.eccentricProgrammedFeed, CoordSys.commandedMCS, *m_eccentricProducer,
            m_pathFeedMotion.receipt, m_pathFeedWCS[2], m_pathFeedProgrammed[2]);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            static_cast<int>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY) :
            static_cast<int>(AlarmManager::G_Code_Invalid_parameter));
        return nullptr;
    }
    if (!IsEccentricCFeedReceiptCurrentSameThread() ||
        !FeedTranslationCurrent(CoordSys, m_pathFeedMotion.receipt.translationGeneration) ||
        m_pathFeedMotion.receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        m_pathFeedMotion.receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathFeedMotion.receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreRotaryFeedSameThread(const NCBlock& block)
{
    const NCTranslationSnapshot& source = CoordSys.GetTranslationSnapshot();
    unsigned selectedAxis = 0U;
    if (!TryGetNCRotaryFeedAxis(source.axisIdentity, block, selectedAxis) ||
        !IsNCRotaryFeedBlockAllowed(source, block))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(selectedAxis));
    const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(axis);
    if (invalidMask != 0U)
    {
        RtPrintf("[TRAVEL-CONFIG][REJECT] unit=ROTARY_FEED axis=%u invalidMask=%u beforeSubmit=1\n",
            selectedAxis, invalidMask);
        RejectPathCoreFeedSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
        return nullptr;
    }
    if (!m_pathFeed.armed || m_pathFeed.pending || Close_System_Com_flag || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        IsFeedHoldActive() || !IsPathCoreFeedConfigurationValid() ||
        !IsFixedTranslationTravelCurrentSameThread() || !FeedTranslationCurrent(CoordSys, source.generation) ||
        !m_motion.MatchesNCTranslation(source) || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_cncFeed.selected || m_cncFeed.active || m_cncFeed.count != 0U ||
        m_pathArc.pending || m_pathReplay.pending || m_pathReplayStore.Pending() ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active ||
        m_cutterLine.valid || m_cutterLine.leadOutRequired || !axis.isExist || axis.axisType != AxisType::ROTARY ||
        source.axisIdentity.address[selectedAxis] != m_axisNames[selectedAxis])
    {
        RejectPathCoreFeedSameThread(3U, (!m_pathFeed.armed && m_pathFeed.invalidatedByGoto) ?
            AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    {
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }

    // Native angular F is always explicit, in deg/min. It cannot seed or
    // inherit the independent XYZ mm/min modal feed or retained-path history.
    ClearCncModalFeedSameThread();
    ClearPathCoreReplayHistorySameThread();
    m_pathFeedProgrammed.fill(false);
    m_pathFeedWCS.fill(0.0);
    m_pathFeedCandidate.fill(0.0);
    m_pathFeedAxes.clear();
    m_pathFeedTargets.clear();
    const double programmedValue = block.val(source.axisIdentity.address[selectedAxis]);
    const bool absolute = source.distanceMode == 90;
    const double feedDegMin = block.val('F');
    m_pathFeedProgrammed[selectedAxis] = true;
    m_pathFeedWCS[selectedAxis] = programmedValue;
    CoordSys.Preview_WCS_to_MCS(m_pathFeedWCS.data(), m_pathFeedProgrammed.data(), m_pathFeedCandidate.data());
    for (unsigned i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(CoordSys.commandedMCS[i]) || !std::isfinite(m_pathFeedCandidate[i]) ||
            (i != selectedAxis && FeedDoubleBits(m_pathFeedCandidate[i]) != FeedDoubleBits(CoordSys.commandedMCS[i])))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    const double authoredStart = CoordSys.commandedMCS[selectedAxis];
    const double expectedTarget = absolute ?
        programmedValue + NCTranslationAxisOffsetMM(source, selectedAxis) : authoredStart + programmedValue;
    if (!std::isfinite(expectedTarget) || FeedDoubleBits(m_pathFeedCandidate[selectedAxis]) != FeedDoubleBits(expectedTarget))
    {
        RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // The producer also checks the physical unwrapped pulse envelope; this
    // authored MCS check alone is insufficient at a rotary modulo crossing.
    if (!CoordSys.IsTargetWithinSoftwareTravelLimit(axis, CoordSys.commandedMCS[selectedAxis]) ||
        !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, m_pathFeedCandidate[selectedAxis]))
    {
        RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(axis, AlarmManager::PROGRAMMED_OVER_TRAVEL));
        return nullptr;
    }
    const bool accepted = m_motion.TryG01RotaryMoveTransactionalTail(static_cast<int>(selectedAxis),
        programmedValue, m_pathFeedCandidate[selectedAxis], feedDegMin, CoordSys.commandedMCS, m_pathFeedMotion);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            static_cast<int>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY) :
            static_cast<int>(AlarmManager::G_Code_Invalid_parameter));
        return nullptr;
    }
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    const NCRotaryFeedLineValue& geometry = receipt.rotary;
    const std::uint32_t expectedMask = 1U << selectedAxis;
    double canonicalTarget = expectedTarget;
    if (absolute)
    {
        NCRotaryAbsoluteFeedTarget resolved{};
        if (!TryResolveNCRotaryAbsoluteFeedTarget(authoredStart, geometry.startPulse[selectedAxis],
                expectedTarget, axis.resolution_PPR / axis.finalLead,
                axis.useShortestPath, axis.rotaryModulo, resolved) ||
            FeedDoubleBits(resolved.endPulse) != FeedDoubleBits(geometry.endPulse[selectedAxis]) ||
            FeedDoubleBits(geometry.absoluteTargetWCS) != FeedDoubleBits(programmedValue) ||
            FeedDoubleBits(geometry.absoluteTargetMCS) != FeedDoubleBits(expectedTarget) ||
            FeedDoubleBits(geometry.rotaryModulo) != FeedDoubleBits(axis.rotaryModulo) ||
            geometry.rotaryShortestPath != axis.useShortestPath)
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
        canonicalTarget = resolved.endMCS;
    }
    if (!receipt.rotaryFeed || receipt.zcFeed || receipt.xyzcFeed || receipt.xyzcuvFeed || !receipt.valid || !receipt.commandAccepted || !receipt.tailCommitted ||
        !receipt.captureBound || !geometry.valid || geometry.axisMask != expectedMask ||
        !FeedTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        (receipt.validAxisMask & expectedMask) != expectedMask ||
        FeedDoubleBits(geometry.feedDegMin) != FeedDoubleBits(feedDegMin) ||
        FeedDoubleBits(geometry.startMCS[selectedAxis]) != FeedDoubleBits(authoredStart) ||
        FeedDoubleBits(geometry.sweepDeg) != FeedDoubleBits(canonicalTarget - authoredStart) ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    for (unsigned i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(geometry.startMCS[i]) || !std::isfinite(geometry.endMCS[i]) ||
            !std::isfinite(geometry.startPulse[i]) || !std::isfinite(geometry.endPulse[i]) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(i == selectedAxis ? canonicalTarget : m_pathFeedCandidate[i]) ||
            (i != selectedAxis && (FeedDoubleBits(geometry.startMCS[i]) != FeedDoubleBits(geometry.endMCS[i]) ||
                FeedDoubleBits(geometry.startPulse[i]) != FeedDoubleBits(geometry.endPulse[i]))))
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreZCFeedSameThread(const NCBlock& block)
{
    const NCTranslationSnapshot& source = CoordSys.GetTranslationSnapshot();
    if (!IsNCZCFeedBlockAllowed(source, block))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const AxisContext& zAxis = m_motion.GetAxisContext(2);
    const AxisContext& cAxis = m_motion.GetAxisContext(3);
    for (int i = 2; i <= 3; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(i);
        const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(axis);
        if (invalidMask != 0U)
        {
            RtPrintf("[TRAVEL-CONFIG][REJECT] unit=ZC_FEED axis=%d invalidMask=%u beforeSubmit=1\n", i, invalidMask);
            RejectPathCoreFeedSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
            return nullptr;
        }
    }
    if (!m_pathFeed.armed || m_pathFeed.pending || Close_System_Com_flag || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        IsFeedHoldActive() || !IsPathCoreFeedConfigurationValid() ||
        !IsFixedTranslationTravelCurrentSameThread() || !FeedTranslationCurrent(CoordSys, source.generation) ||
        !m_motion.MatchesNCTranslation(source) || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_cncFeed.selected || m_cncFeed.active || m_cncFeed.count != 0U ||
        m_pathArc.pending || m_pathReplay.pending || m_pathReplayStore.Pending() ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active ||
        m_cutterLine.valid || m_cutterLine.leadOutRequired ||
        !zAxis.isExist || zAxis.axisType != AxisType::LINEAR ||
        !cAxis.isExist || cAxis.axisType != AxisType::ROTARY ||
        m_axisNames[2] != 'Z' || m_axisNames[3] != 'C')
    {
        RejectPathCoreFeedSameThread(3U, (!m_pathFeed.armed && m_pathFeed.invalidatedByGoto) ?
            AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    {
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    // Z-distance F has its own explicit common-time contract. It cannot seed
    // modal XYZ feed, cutter nominal geometry, or the retained replay lane.
    ClearCncModalFeedSameThread();
    ClearPathCoreReplayHistorySameThread();
    m_pathFeedProgrammed.fill(false);
    m_pathFeedWCS.fill(0.0);
    m_pathFeedCandidate.fill(0.0);
    m_pathFeedAxes.clear();
    m_pathFeedTargets.clear();
    const double programmedZ = block.val('Z'), programmedC = block.val('C'), feedMMMin = block.val('F');
    const bool absolute = source.distanceMode == 90;
    m_pathFeedProgrammed[2] = m_pathFeedProgrammed[3] = true;
    m_pathFeedWCS[2] = programmedZ;
    m_pathFeedWCS[3] = programmedC;
    CoordSys.Preview_WCS_to_MCS(m_pathFeedWCS.data(), m_pathFeedProgrammed.data(), m_pathFeedCandidate.data());
    std::array<double, 8U> authoredStart{};
    for (unsigned i = 0U; i < 8U; ++i)
    {
        authoredStart[i] = CoordSys.commandedMCS[i];
        const double expected = i == 2U ? (absolute ? programmedZ + NCTranslationAxisOffsetMM(source, i) :
            authoredStart[i] + programmedZ) : (i == 3U ? (absolute ? programmedC + NCTranslationAxisOffsetMM(source, i) :
                authoredStart[i] + programmedC) : authoredStart[i]);
        if (!std::isfinite(authoredStart[i]) || !std::isfinite(expected) ||
            FeedDoubleBits(m_pathFeedCandidate[i]) != FeedDoubleBits(expected))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    // Prove the raw native start/end interval on BOTH axes. The producer
    // independently proves the unwrapped physical pulse interval as well.
    for (int i = 2; i <= 3; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(i);
        if (!CoordSys.IsTargetWithinSoftwareTravelLimit(axis, authoredStart[i]) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, m_pathFeedCandidate[i]))
        {
            RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(axis, AlarmManager::PROGRAMMED_OVER_TRAVEL));
            return nullptr;
        }
    }
    const bool accepted = m_motion.TryG01ZCMoveTransactionalTail(programmedZ, programmedC,
        m_pathFeedCandidate[2], m_pathFeedCandidate[3], feedMMMin, CoordSys.commandedMCS, m_pathFeedMotion);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            static_cast<int>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY) :
            static_cast<int>(AlarmManager::G_Code_Invalid_parameter));
        return nullptr;
    }
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    const NCZCFeedLineValue& geometry = receipt.zc;
    // Resolve the absolute endpoints independently from the receipt's immutable
    // starts. C uses the existing modulo/shortest policy exactly once; Z stays
    // the raw absolute MCS target. F derives from their effective native deltas.
    NCZCAbsoluteFeedTarget resolved{};
    if (absolute && !TryResolveNCZCAbsoluteFeedTarget(authoredStart[2], geometry.startPulse[2],
            authoredStart[3], geometry.startPulse[3], m_pathFeedCandidate[2], m_pathFeedCandidate[3],
            zAxis.resolution_PPR / zAxis.finalLead, cAxis.resolution_PPR / cAxis.finalLead,
            cAxis.useShortestPath, cAxis.rotaryModulo, resolved))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    const double deltaZ = absolute ? resolved.deltaZMM : programmedZ;
    const double deltaC = absolute ? resolved.deltaCDeg : programmedC;

    const double nominalSeconds = NCZCFeedDetail::ProductRatio(std::fabs(deltaZ), 60.0, feedMMMin);
    const double rotaryFeedDegMin = NCZCFeedDetail::ProductRatio(std::fabs(deltaC), 60.0, nominalSeconds);
    const double pulsePerMM = zAxis.resolution_PPR / zAxis.finalLead;
    const double pulsePerDegree = cAxis.resolution_PPR / cAxis.finalLead;
    const double deltaZPulse = geometry.endPulse[2] - geometry.startPulse[2];
    const double deltaCPulse = geometry.endPulse[3] - geometry.startPulse[3];
    const double lengthPulse = std::hypot(deltaZPulse, deltaCPulse);
    const double velocityPPS = NCZCFeedDetail::ProductRatio(lengthPulse, 1.0, nominalSeconds);
    const double linearVelocityPPS = std::fabs(velocityPPS * (deltaZPulse / lengthPulse));
    const double rotaryVelocityPPS = std::fabs(velocityPPS * (deltaCPulse / lengthPulse));
    const double accTime = NCZCFeedDetail::Maximum(NCZCFeedDetail::EffectiveTime(zAxis.G00_acc_time),
        NCZCFeedDetail::EffectiveTime(cAxis.G00_acc_time));
    const double decTime = NCZCFeedDetail::Maximum(NCZCFeedDetail::EffectiveTime(zAxis.G00_dec_time),
        NCZCFeedDetail::EffectiveTime(cAxis.G00_dec_time));
    if (!receipt.zcFeed || receipt.rotaryFeed || receipt.xyzcFeed || receipt.xyzcuvFeed || !receipt.valid || !receipt.commandAccepted ||
        !receipt.tailCommitted || !receipt.captureBound || !geometry.valid || geometry.axisMask != 12U ||
        geometry.absolute != absolute ||
        FeedDoubleBits(geometry.absoluteTargetZWCS) != FeedDoubleBits(absolute ? programmedZ : 0.0) ||
        FeedDoubleBits(geometry.absoluteTargetCWCS) != FeedDoubleBits(absolute ? programmedC : 0.0) ||
        FeedDoubleBits(geometry.absoluteTargetZMCS) != FeedDoubleBits(absolute ? m_pathFeedCandidate[2] : 0.0) ||
        FeedDoubleBits(geometry.absoluteTargetCMCS) != FeedDoubleBits(absolute ? m_pathFeedCandidate[3] : 0.0) ||
        FeedDoubleBits(geometry.rotaryModulo) != FeedDoubleBits(absolute ? cAxis.rotaryModulo : 0.0) ||
        geometry.rotaryShortestPath != (absolute && cAxis.useShortestPath) ||
        !FeedTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        (receipt.validAxisMask & 12U) != 12U ||
        FeedDoubleBits(geometry.feedMMMin) != FeedDoubleBits(feedMMMin) ||
        FeedDoubleBits(geometry.deltaZMM) != FeedDoubleBits(deltaZ) ||
        FeedDoubleBits(geometry.deltaCDeg) != FeedDoubleBits(deltaC) ||
        FeedDoubleBits(geometry.pulsePerMM) != FeedDoubleBits(pulsePerMM) ||
        FeedDoubleBits(geometry.pulsePerDegree) != FeedDoubleBits(pulsePerDegree) ||
        FeedDoubleBits(geometry.nominalSeconds) != FeedDoubleBits(nominalSeconds) ||
        FeedDoubleBits(geometry.rotaryFeedDegMin) != FeedDoubleBits(rotaryFeedDegMin) ||
        FeedDoubleBits(geometry.lengthPulse) != FeedDoubleBits(lengthPulse) ||
        FeedDoubleBits(geometry.velocityPPS) != FeedDoubleBits(velocityPPS) ||
        FeedDoubleBits(geometry.linearVelocityPPS) != FeedDoubleBits(linearVelocityPPS) ||
        FeedDoubleBits(geometry.rotaryVelocityPPS) != FeedDoubleBits(rotaryVelocityPPS) ||
        FeedDoubleBits(geometry.accTime) != FeedDoubleBits(accTime) ||
        FeedDoubleBits(geometry.decTime) != FeedDoubleBits(decTime) ||
        !std::isfinite(lengthPulse) || lengthPulse <= 0.0 || !std::isfinite(velocityPPS) || velocityPPS < 1.0 ||
        !std::isfinite(rotaryFeedDegMin) || rotaryFeedDegMin <= 0.0 || rotaryFeedDegMin > 100.0 ||
        !std::isfinite(geometry.linearVelocityPPS) || geometry.linearVelocityPPS <= 0.0 ||
        geometry.linearVelocityPPS > zAxis.maxVel_PPS ||
        !std::isfinite(geometry.rotaryVelocityPPS) || geometry.rotaryVelocityPPS <= 0.0 ||
        geometry.rotaryVelocityPPS > cAxis.maxVel_PPS ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    for (unsigned i = 0U; i < 8U; ++i)
    {
        const double expectedNative = absolute && i == 3U ? resolved.endCMCS : m_pathFeedCandidate[i];
        const double expectedPulse = i == 2U ? (absolute ? resolved.endZPulse :
            geometry.startPulse[i] + deltaZ * pulsePerMM) : (i == 3U ? (absolute ? resolved.endCPulse :
                geometry.startPulse[i] + deltaC * pulsePerDegree) : geometry.startPulse[i]);
        if (!std::isfinite(geometry.startMCS[i]) || !std::isfinite(geometry.endMCS[i]) ||
            !std::isfinite(geometry.startPulse[i]) || !std::isfinite(geometry.endPulse[i]) ||
            FeedDoubleBits(geometry.startMCS[i]) != FeedDoubleBits(authoredStart[i]) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(expectedNative) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(CoordSys.commandedMCS[i]) ||
            FeedDoubleBits(geometry.endPulse[i]) != FeedDoubleBits(expectedPulse))
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreXYZCFeedSameThread(const NCBlock& block)
{
    const NCTranslationSnapshot& source = CoordSys.GetTranslationSnapshot();
    if (!IsNCXYZCFeedBlockAllowed(source, block))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (int i = 0; i < 4; ++i)
    {
        const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(m_motion.GetAxisContext(i));
        if (invalidMask != 0U)
        {
            RtPrintf("[TRAVEL-CONFIG][REJECT] unit=XYZC_FEED axis=%d invalidMask=%u beforeSubmit=1\n", i, invalidMask);
            RejectPathCoreFeedSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
            return nullptr;
        }
    }
    if (!m_pathFeed.armed || m_pathFeed.pending || Close_System_Com_flag || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        IsFeedHoldActive() || !IsPathCoreFeedConfigurationValid() || !IsXYZCFeedProfileCurrentSameThread() ||
        !IsFixedTranslationTravelCurrentSameThread() || !FeedTranslationCurrent(CoordSys, source.generation) ||
        !m_motion.MatchesNCTranslation(source) || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_cncFeed.selected || m_cncFeed.active || m_cncFeed.count != 0U ||
        m_pathArc.pending || m_pathReplay.pending || m_pathReplayStore.Pending() ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active ||
        m_cutterLine.valid || m_cutterLine.leadOutRequired)
    {
        RejectPathCoreFeedSameThread(3U, (!m_pathFeed.armed && m_pathFeed.invalidatedByGoto) ?
            AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (unsigned i = 0U; i < 4U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!axis.isExist || axis.axisType != (i < 3U ? AxisType::LINEAR : AxisType::ROTARY) ||
            m_axisNames[i] != "XYZC"[i] || source.axisIdentity.address[i] != m_axisNames[i])
        {
            RejectPathCoreFeedSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    {
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    // Explicit XYZ-length feed has no XYZ modal, cutter, retained or EDM source.
    ClearCncModalFeedSameThread();
    ClearPathCoreReplayHistorySameThread();
    m_pathFeedProgrammed.fill(false);
    m_pathFeedWCS.fill(0.0);
    m_pathFeedCandidate.fill(0.0);
    m_pathFeedAxes.clear();
    m_pathFeedTargets.clear();
    const bool absolute = source.distanceMode == 90;
    std::array<double, 4U> programmedWords{}, targetMCS{};
    for (unsigned i = 0U; i < 4U; ++i)
    {
        programmedWords[i] = block.val("XYZC"[i]);
        m_pathFeedProgrammed[i] = true;
        m_pathFeedWCS[i] = programmedWords[i];
    }
    const double feedMMMin = block.val('F');
    CoordSys.Preview_WCS_to_MCS(m_pathFeedWCS.data(), m_pathFeedProgrammed.data(), m_pathFeedCandidate.data());
    std::array<double, 8U> authoredStart{};
    for (unsigned i = 0U; i < 8U; ++i)
    {
        authoredStart[i] = CoordSys.commandedMCS[i];
        const double expected = i < 4U ? (absolute ? programmedWords[i] + NCTranslationAxisOffsetMM(source, i) :
            authoredStart[i] + programmedWords[i]) : authoredStart[i];
        if (!std::isfinite(authoredStart[i]) || !std::isfinite(expected) ||
            FeedDoubleBits(m_pathFeedCandidate[i]) != FeedDoubleBits(expected))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        if (i < 4U) targetMCS[i] = expected;
    }
    // Raw native interval proof precedes publication. The producer also checks
    // each resolved canonical native and continuous physical-pulse interval.
    for (int i = 0; i < 4; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(i);
        if (!CoordSys.IsTargetWithinSoftwareTravelLimit(axis, authoredStart[i]) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, m_pathFeedCandidate[i]))
        {
            RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(axis, AlarmManager::PROGRAMMED_OVER_TRAVEL));
            return nullptr;
        }
    }
    const bool accepted = m_motion.TryG01XYZCMoveTransactionalTail(programmedWords, targetMCS,
        feedMMMin, CoordSys.commandedMCS, m_pathFeedMotion);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            static_cast<int>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY) :
            static_cast<int>(AlarmManager::G_Code_Invalid_parameter));
        return nullptr;
    }
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    const NCXYZCFeedLineValue& geometry = receipt.xyzc;
    // Re-resolve independently from the immutable continuous pulse anchor.
    // C applies its frozen modulo/shortest policy; U/V remain stationary.
    std::array<double, 4U> startNative{}, startPulse{}, pulsePerUnit{};
    double modulo = 0.0;
    bool shortest = false;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        startNative[i] = authoredStart[i];
        startPulse[i] = geometry.startPulse[i];
        pulsePerUnit[i] = axis.resolution_PPR / axis.finalLead;
        if (i >= 3U) { modulo = axis.rotaryModulo; shortest = axis.useShortestPath; }
    }
    NCXYZCAbsoluteFeedTarget resolved{};
    if (absolute && !TryResolveNCXYZCAbsoluteFeedTarget(startNative, startPulse, targetMCS,
            pulsePerUnit, shortest, modulo, resolved))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    const std::array<double, 4U>& effectiveDelta = absolute ? resolved.deltaNative : programmedWords;
    const double linearLengthMM = std::hypot(std::hypot(effectiveDelta[0], effectiveDelta[1]), effectiveDelta[2]);
    const double nominalSeconds = NCZCFeedDetail::ProductRatio(linearLengthMM, 60.0, feedMMMin);
    double lengthPulse = 0.0, accTime = 0.0, decTime = 0.0;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        lengthPulse = std::hypot(lengthPulse, geometry.endPulse[i] - geometry.startPulse[i]);
        accTime = NCZCFeedDetail::Maximum(accTime, NCZCFeedDetail::EffectiveTime(axis.G00_acc_time));
        decTime = NCZCFeedDetail::Maximum(decTime, NCZCFeedDetail::EffectiveTime(axis.G00_dec_time));
    }
    const double velocityPPS = NCZCFeedDetail::ProductRatio(lengthPulse, 1.0, nominalSeconds);
    if (!receipt.xyzcFeed || receipt.xyzcuvFeed || receipt.zcFeed || receipt.rotaryFeed || !receipt.valid || !receipt.commandAccepted ||
        !receipt.tailCommitted || !receipt.captureBound || !geometry.valid || geometry.axisMask != 15U ||
        geometry.absolute != absolute ||
        !FeedTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        (receipt.validAxisMask & 15U) != 15U || !IsXYZCFeedProfileCurrentSameThread() ||
        FeedDoubleBits(geometry.feedMMMin) != FeedDoubleBits(feedMMMin) ||
        FeedDoubleBits(geometry.linearLengthMM) != FeedDoubleBits(linearLengthMM) ||
        FeedDoubleBits(geometry.nominalSeconds) != FeedDoubleBits(nominalSeconds) ||
        FeedDoubleBits(geometry.lengthPulse) != FeedDoubleBits(lengthPulse) ||
        FeedDoubleBits(geometry.velocityPPS) != FeedDoubleBits(velocityPPS) ||
        FeedDoubleBits(geometry.accTime) != FeedDoubleBits(accTime) ||
        FeedDoubleBits(geometry.decTime) != FeedDoubleBits(decTime) ||
        !std::isfinite(linearLengthMM) || linearLengthMM <= 0.0 ||
        !std::isfinite(nominalSeconds) || nominalSeconds <= 0.0 ||
        !std::isfinite(lengthPulse) || lengthPulse <= 0.0 || !std::isfinite(velocityPPS) || velocityPPS < 1.0 ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    for (unsigned i = 0U; i < 8U; ++i)
    {
        const double ppu = i < 4U ? pulsePerUnit[i] : 0.0;
        const double expectedNative = absolute && i < 4U ? resolved.endMCS[i] : m_pathFeedCandidate[i];
        const double expectedPulse = i < 4U ? (absolute ? resolved.endPulse[i] :
            geometry.startPulse[i] + effectiveDelta[i] * ppu) : geometry.startPulse[i];
        if (!std::isfinite(geometry.startMCS[i]) || !std::isfinite(geometry.endMCS[i]) ||
            !std::isfinite(geometry.startPulse[i]) || !std::isfinite(geometry.endPulse[i]) ||
            FeedDoubleBits(geometry.startMCS[i]) != FeedDoubleBits(authoredStart[i]) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(expectedNative) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(CoordSys.commandedMCS[i]) ||
            FeedDoubleBits(geometry.endPulse[i]) != FeedDoubleBits(expectedPulse))
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
        if (i < 4U)
        {
            const double deltaPulse = geometry.endPulse[i] - geometry.startPulse[i];
            const double directVelocity = NCZCFeedDetail::ProductRatio(std::fabs(deltaPulse), 1.0, nominalSeconds);
            const double axisVelocity = std::fabs(velocityPPS * (deltaPulse / lengthPulse));
            const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
            if (FeedDoubleBits(geometry.absoluteTargetWCS[i]) != FeedDoubleBits(absolute ? programmedWords[i] : 0.0) ||
                FeedDoubleBits(geometry.absoluteTargetMCS[i]) != FeedDoubleBits(absolute ? targetMCS[i] : 0.0) ||
                !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, expectedNative))
            {
                RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
            if (i >= 3U)
            {
                const double rotaryFeedDegMin = NCZCFeedDetail::ProductRatio(std::fabs(effectiveDelta[i]), 60.0, nominalSeconds);
                if (FeedDoubleBits(geometry.rotaryModulo) != FeedDoubleBits(absolute ? modulo : 0.0) ||
                    geometry.rotaryShortestPath != (absolute && shortest) ||
                    FeedDoubleBits(geometry.rotaryFeedDegMin) != FeedDoubleBits(rotaryFeedDegMin) ||
                    !std::isfinite(rotaryFeedDegMin) || rotaryFeedDegMin <= 0.0 || rotaryFeedDegMin > 100.0)
                {
                    RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                    return nullptr;
                }
            }
            if (FeedDoubleBits(geometry.deltaNative[i]) != FeedDoubleBits(effectiveDelta[i]) ||
                FeedDoubleBits(geometry.pulsePerUnit[i]) != FeedDoubleBits(ppu) ||
                FeedDoubleBits(geometry.axisVelocityPPS[i]) != FeedDoubleBits(axisVelocity) ||
                !std::isfinite(deltaPulse) || std::fabs(deltaPulse) < 1.0e-5 ||
                !std::isfinite(directVelocity) || directVelocity <= 0.0 || directVelocity > axis.maxVel_PPS ||
                !std::isfinite(axisVelocity) || axisVelocity <= 0.0 || axisVelocity > axis.maxVel_PPS)
            {
                RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
        }
    }
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

bool NCManager::IsEDM149ZFeedSameThread() const noexcept
{
    return m_edm94.latched && m_edm94.ready && m_edm94.everArmed && !m_edm94.armPending &&
        !m_edm94.cancelled && !m_edm94.resetSeen && !m_edm94.resetCommitted && !m_edm94.releaseRequested &&
        !m_edm94.rtFaultReported && m_edm94.gapHistoryServo && m_edm94.gapHistoryMode && m_edm94.gapWorkTime &&
        m_edm94.gapModeFixture == EDM116::Fixture::ModalZ149 && EDM149::IsZTestArm(m_edm94.gapHistoryArm) &&
        m_edm130.bound && EDM130::IsMotionTest(m_edm130.recipe) &&
        m_edm130.recipe.diagnosticControlSource == EDM149::SourceForArm(m_edm94.gapHistoryArm);
}

bool NCManager::IsEDM149ZFeedBlockSameThread(const NCBlock& block) const noexcept
{
    const auto& source = CoordSys.GetTranslationSnapshot();
    if (!IsEDM149ZFeedSameThread() || source.distanceMode != 91 || !IsNCXYZCUVFeedNeutralFrame(source) ||
        !IsNCXYZFeedSingleGBlock(block,1) || !block.has('Z') || !block.has('F')) return false;
    for (char letter='A';letter<='Z';++letter)
        if (block.has(letter) && (std::strchr("NXYZCUVF",letter) == nullptr || !std::isfinite(block.val(letter)))) return false;
    std::array<double,6U> words{};
    for (unsigned axis=0U;axis<6U;++axis) if(block.has("XYZCUV"[axis])) words[axis]=block.val("XYZCUV"[axis]);
    return EDM149::ValidZIncrement(words,block.val('F'));
}

bool NCManager::IsEDM150XYZFeedSameThread() const noexcept
{
    return m_edm94.latched && m_edm94.ready && m_edm94.everArmed && !m_edm94.armPending &&
        !m_edm94.cancelled && !m_edm94.resetSeen && !m_edm94.resetCommitted && !m_edm94.releaseRequested &&
        !m_edm94.rtFaultReported && m_edm94.gapHistoryServo && m_edm94.gapHistoryMode && m_edm94.gapWorkTime &&
        m_edm94.gapModeFixture == EDM116::Fixture::ModalXYZ150 && EDM149::IsXYZTestArm(m_edm94.gapHistoryArm) &&
        m_edm130.bound && EDM130::IsMotionTest(m_edm130.recipe) &&
        m_edm130.recipe.diagnosticControlSource == EDM149::SourceForArm(m_edm94.gapHistoryArm);
}

bool NCManager::IsEDM150XYZFeedBlockSameThread(const NCBlock& block) const noexcept
{
    const auto& source = CoordSys.GetTranslationSnapshot();
    if (!IsEDM150XYZFeedSameThread() || source.distanceMode != 91 || !IsNCXYZCUVFeedNeutralFrame(source) ||
        !IsNCXYZFeedSingleGBlock(block,1) || !block.has('X') || !block.has('Y') || !block.has('Z') || !block.has('F')) return false;
    for (char letter='A';letter<='Z';++letter)
        if (block.has(letter) && (std::strchr("NXYZCUVF",letter) == nullptr || !std::isfinite(block.val(letter)))) return false;
    std::array<double,6U> words{};
    for (unsigned axis=0U;axis<6U;++axis) if(block.has("XYZCUV"[axis])) words[axis]=block.val("XYZCUV"[axis]);
    return EDM149::ValidXYZIncrement(words,block.val('F'));
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreXYZCUVFeedSameThread(const NCBlock& block)
{
    const NCTranslationSnapshot& source = CoordSys.GetTranslationSnapshot();
    const bool zOnlyCommissioning = IsEDM149ZFeedSameThread();
    const bool xyzOnlyCommissioning = IsEDM150XYZFeedSameThread();
    const bool sparseCommissioning = IsEDM154SparseFeedSameThread();
    std::array<double,6U> zWords{};
    if (zOnlyCommissioning || xyzOnlyCommissioning || sparseCommissioning)
        for (unsigned axis=0U;axis<6U;++axis) if(block.has("XYZCUV"[axis])) zWords[axis]=block.val("XYZCUV"[axis]);
    if (!(zOnlyCommissioning ? IsEDM149ZFeedBlockSameThread(block) : xyzOnlyCommissioning ? IsEDM150XYZFeedBlockSameThread(block) : sparseCommissioning ? IsEDM154SparseFeedBlockSameThread(block) : IsNCXYZCUVFeedBlockAllowed(source, block)))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (int i = 0; i < 6; ++i)
    {
        const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(m_motion.GetAxisContext(i));
        if (invalidMask != 0U)
        {
            RtPrintf("[TRAVEL-CONFIG][REJECT] unit=XYZCUV_FEED axis=%d invalidMask=%u beforeSubmit=1\n", i, invalidMask);
            RejectPathCoreFeedSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
            return nullptr;
        }
    }
    if (!m_pathFeed.armed || m_pathFeed.pending || Close_System_Com_flag || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        IsFeedHoldActive() || !IsPathCoreFeedConfigurationValid() || !IsXYZCUVFeedProfileCurrentSameThread() ||
        !IsFixedTranslationTravelCurrentSameThread() || !FeedTranslationCurrent(CoordSys, source.generation) ||
        !m_motion.MatchesNCTranslation(source) || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_cncFeed.selected || m_cncFeed.active || m_cncFeed.count != 0U ||
        m_pathArc.pending || m_pathReplay.pending || m_pathReplayStore.Pending() ||
        m_pathHold.armed || m_pathHold.bound || m_gapDryRun.active || m_gapPath.active || m_gapWindow.active ||
        m_cutterLine.valid || m_cutterLine.leadOutRequired)
    {
        RejectPathCoreFeedSameThread(3U, (!m_pathFeed.armed && m_pathFeed.invalidatedByGoto) ?
            AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (unsigned i = 0U; i < 6U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!axis.isExist || axis.axisType != (i < 3U ? AxisType::LINEAR : AxisType::ROTARY) ||
            m_axisNames[i] != "XYZCUV"[i] || source.axisIdentity.address[i] != m_axisNames[i])
        {
            RejectPathCoreFeedSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    {
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    // Explicit XYZ-length feed has no XYZ modal, cutter, retained or EDM source.
    ClearCncModalFeedSameThread();
    ClearPathCoreReplayHistorySameThread();
    m_pathFeedProgrammed.fill(false);
    m_pathFeedWCS.fill(0.0);
    m_pathFeedCandidate.fill(0.0);
    m_pathFeedAxes.clear();
    m_pathFeedTargets.clear();
    const bool absolute = source.distanceMode == 90;
    std::array<double, 6U> programmedWords{}, targetMCS{};
    for (unsigned i = 0U; i < 6U; ++i)
    {
        programmedWords[i] = (zOnlyCommissioning || xyzOnlyCommissioning || sparseCommissioning) ? zWords[i] : block.val("XYZCUV"[i]);
        m_pathFeedProgrammed[i] = true;
        m_pathFeedWCS[i] = programmedWords[i];
    }
    const double feedMMMin = block.val('F');
    CoordSys.Preview_WCS_to_MCS(m_pathFeedWCS.data(), m_pathFeedProgrammed.data(), m_pathFeedCandidate.data());
    std::array<double, 8U> authoredStart{};
    for (unsigned i = 0U; i < 8U; ++i)
    {
        authoredStart[i] = CoordSys.commandedMCS[i];
        const bool sparseStatic = sparseCommissioning && i < 6U && programmedWords[i] == 0.0;
        const double expected = sparseStatic ? authoredStart[i] : i < 6U ?
            (absolute ? programmedWords[i] + NCTranslationAxisOffsetMM(source, i) :
                authoredStart[i] + programmedWords[i]) : authoredStart[i];
        if (sparseStatic && m_pathFeedCandidate[i] == authoredStart[i]) m_pathFeedCandidate[i] = authoredStart[i];
        if (!std::isfinite(authoredStart[i]) || !std::isfinite(expected) ||
            FeedDoubleBits(m_pathFeedCandidate[i]) != FeedDoubleBits(expected))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        if (i < 6U) targetMCS[i] = expected;
    }
    // Raw native interval proof precedes publication. The producer also checks
    // each resolved canonical native and continuous physical-pulse interval.
    for (int i = 0; i < 6; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(i);
        if (!CoordSys.IsTargetWithinSoftwareTravelLimit(axis, authoredStart[i]) ||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, m_pathFeedCandidate[i]))
        {
            RejectPathCoreFeedSameThread(7U, CoordSys.GetSoftwareTravelLimitAlarmCode(axis, AlarmManager::PROGRAMMED_OVER_TRAVEL));
            return nullptr;
        }
    }
    if (!GuardEDM94FeedSameThread(block))
    {
        RejectEDM94SameThread("ENDPOINT_OR_SCOPE", m_gapDryRun.sourceLine);
        return nullptr;
    }
    const bool accepted = m_motion.TryG01XYZCUVMoveTransactionalTail(programmedWords, targetMCS,
        feedMMMin, CoordSys.commandedMCS, m_pathFeedMotion, sparseCommissioning);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            static_cast<int>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY) :
            static_cast<int>(AlarmManager::G_Code_Invalid_parameter));
        return nullptr;
    }
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    const NCXYZCUVFeedLineValue& geometry = receipt.xyzcuv;
    if (geometry.zOnlyCommissioning != zOnlyCommissioning || geometry.xyzOnlyCommissioning != xyzOnlyCommissioning ||
        geometry.sparseCommissioning != sparseCommissioning)
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    // Re-resolve independently from the immutable continuous pulse anchor.
    // The three positional rotaries each apply their own modulo/shortest policy.
    std::array<double, 6U> startNative{}, startPulse{}, pulsePerUnit{};
    std::array<double, 3U> modulo{};
    std::array<bool, 3U> shortest{};
    for (unsigned i = 0U; i < 6U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        startNative[i] = authoredStart[i];
        startPulse[i] = geometry.startPulse[i];
        pulsePerUnit[i] = axis.resolution_PPR / axis.finalLead;
        if (i >= 3U) { modulo[i - 3U] = axis.rotaryModulo; shortest[i - 3U] = axis.useShortestPath; }
    }
    NCXYZCUVAbsoluteFeedTarget resolved{};
    if (absolute && !TryResolveNCXYZCUVAbsoluteFeedTarget(startNative, startPulse, targetMCS,
            pulsePerUnit, shortest, modulo, resolved))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    const std::array<double, 6U>& effectiveDelta = absolute ? resolved.deltaNative : programmedWords;
    const double linearLengthMM = std::hypot(std::hypot(effectiveDelta[0], effectiveDelta[1]), effectiveDelta[2]);
    const double nominalSeconds = NCZCFeedDetail::ProductRatio(linearLengthMM, 60.0, feedMMMin);
    double lengthPulse = 0.0, accTime = 0.0, decTime = 0.0;
    for (unsigned i = 0U; i < 6U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        lengthPulse = std::hypot(lengthPulse, geometry.endPulse[i] - geometry.startPulse[i]);
        accTime = NCZCFeedDetail::Maximum(accTime, NCZCFeedDetail::EffectiveTime(axis.G00_acc_time));
        decTime = NCZCFeedDetail::Maximum(decTime, NCZCFeedDetail::EffectiveTime(axis.G00_dec_time));
    }
    const double velocityPPS = NCZCFeedDetail::ProductRatio(lengthPulse, 1.0, nominalSeconds);
    if (!receipt.xyzcuvFeed || receipt.xyzcFeed || receipt.zcFeed || receipt.rotaryFeed || !receipt.valid || !receipt.commandAccepted ||
        !receipt.tailCommitted || !receipt.captureBound || !geometry.valid || geometry.axisMask != 63U ||
        geometry.absolute != absolute ||
        !FeedTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        (receipt.validAxisMask & 63U) != 63U || !IsXYZCUVFeedProfileCurrentSameThread() ||
        FeedDoubleBits(geometry.feedMMMin) != FeedDoubleBits(feedMMMin) ||
        FeedDoubleBits(geometry.linearLengthMM) != FeedDoubleBits(linearLengthMM) ||
        FeedDoubleBits(geometry.nominalSeconds) != FeedDoubleBits(nominalSeconds) ||
        FeedDoubleBits(geometry.lengthPulse) != FeedDoubleBits(lengthPulse) ||
        FeedDoubleBits(geometry.velocityPPS) != FeedDoubleBits(velocityPPS) ||
        FeedDoubleBits(geometry.accTime) != FeedDoubleBits(accTime) ||
        FeedDoubleBits(geometry.decTime) != FeedDoubleBits(decTime) ||
        !std::isfinite(linearLengthMM) || linearLengthMM <= 0.0 ||
        !std::isfinite(nominalSeconds) || nominalSeconds <= 0.0 ||
        !std::isfinite(lengthPulse) || lengthPulse <= 0.0 || !std::isfinite(velocityPPS) || velocityPPS < 1.0 ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    for (unsigned i = 0U; i < 8U; ++i)
    {
        const double ppu = i < 6U ? pulsePerUnit[i] : 0.0;
        const bool sparseStatic = sparseCommissioning && i < 6U && effectiveDelta[i] == 0.0;
        const bool staticAxis = i < 6U && (EDM149::StaticAxis(zOnlyCommissioning,xyzOnlyCommissioning,i) || sparseStatic);
        const double expectedNative = sparseStatic ? authoredStart[i] :
            absolute && i < 6U ? resolved.endMCS[i] : m_pathFeedCandidate[i];
        const double expectedPulse = sparseStatic ? geometry.startPulse[i] : i < 6U ? (absolute ? resolved.endPulse[i] :
            geometry.startPulse[i] + effectiveDelta[i] * ppu) : geometry.startPulse[i];
        if (!std::isfinite(geometry.startMCS[i]) || !std::isfinite(geometry.endMCS[i]) ||
            !std::isfinite(geometry.startPulse[i]) || !std::isfinite(geometry.endPulse[i]) ||
            FeedDoubleBits(geometry.startMCS[i]) != FeedDoubleBits(authoredStart[i]) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(expectedNative) ||
            FeedDoubleBits(geometry.endMCS[i]) != FeedDoubleBits(CoordSys.commandedMCS[i]) ||
            FeedDoubleBits(geometry.endPulse[i]) != FeedDoubleBits(expectedPulse))
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
        if (i < 6U)
        {
            const double deltaPulse = geometry.endPulse[i] - geometry.startPulse[i];
            const double directVelocity = NCZCFeedDetail::ProductRatio(std::fabs(deltaPulse), 1.0, nominalSeconds);
            const double axisVelocity = std::fabs(velocityPPS * (deltaPulse / lengthPulse));
            const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
            if (FeedDoubleBits(geometry.absoluteTargetWCS[i]) != FeedDoubleBits(absolute ? programmedWords[i] : 0.0) ||
                FeedDoubleBits(geometry.absoluteTargetMCS[i]) != FeedDoubleBits(absolute ? targetMCS[i] : 0.0) ||
                !CoordSys.IsTargetWithinSoftwareTravelLimit(axis, expectedNative))
            {
                RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
            if (i >= 3U)
            {
                const double rotaryFeedDegMin = NCZCFeedDetail::ProductRatio(std::fabs(effectiveDelta[i]), 60.0, nominalSeconds);
                if (FeedDoubleBits(geometry.rotaryModulo[i - 3U]) != FeedDoubleBits(absolute ? modulo[i - 3U] : 0.0) ||
                    geometry.rotaryShortestPath[i - 3U] != (absolute && shortest[i - 3U]) ||
                    FeedDoubleBits(geometry.rotaryFeedDegMin[i - 3U]) != FeedDoubleBits(rotaryFeedDegMin) ||
                    !std::isfinite(rotaryFeedDegMin) || (staticAxis ? rotaryFeedDegMin != 0.0 : rotaryFeedDegMin <= 0.0) || rotaryFeedDegMin > 100.0)
                {
                    RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                    return nullptr;
                }
            }
            if (FeedDoubleBits(geometry.deltaNative[i]) != FeedDoubleBits(effectiveDelta[i]) ||
                FeedDoubleBits(geometry.pulsePerUnit[i]) != FeedDoubleBits(ppu) ||
                FeedDoubleBits(geometry.axisVelocityPPS[i]) != FeedDoubleBits(axisVelocity) ||
                !std::isfinite(deltaPulse) ||
                (staticAxis ? deltaPulse != 0.0 : std::fabs(deltaPulse) < 1.0e-5) ||
                !std::isfinite(directVelocity) || (staticAxis ? directVelocity != 0.0 : directVelocity <= 0.0) || directVelocity > axis.maxVel_PPS ||
                !std::isfinite(axisVelocity) || (staticAxis ? axisVelocity != 0.0 : axisVelocity <= 0.0) || axisVelocity > axis.maxVel_PPS)
            {
                RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
        }
    }
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreFeedSameThread(const NCBlock& block)
{
    // EDM149/150 share the six-axis producer with explicitly frozen static axes.
    if (IsEDM149ZFeedSameThread() || IsEDM150XYZFeedSameThread() || IsEDM154SparseFeedSameThread()) return StartPathCoreXYZCUVFeedSameThread(block);
    // Classify any addressed extra physical slot before XYZ admission, so a
    // mixed, multi-rotary or omitted-F candidate cannot fall into another lane.
    const NCAxisIdentitySnapshot& nativeAxes = CoordSys.GetTranslationSnapshot().axisIdentity;
    if (nativeAxes.eccentricEnabled != 0U && CoordSys.GetTranslationSnapshot().toolLengthMode != 49)
        return StartPathCoreEccentricCFeedSameThread(block);
    // Classify six-axis mixtures before XYZC and Z+C subsets. Standalone
    // U/V remain on the existing single-rotary lane; partial mixtures reject.
    if ((block.has('U') || block.has('V')) &&
        (block.has('X') || block.has('Y') || block.has('Z') || block.has('C') || (block.has('U') && block.has('V'))))
        return StartPathCoreXYZCUVFeedSameThread(block);
    if (block.has('C') && (block.has('X') || block.has('Y'))) return StartPathCoreXYZCFeedSameThread(block);
    if (block.has('Z') && block.has('C')) return StartPathCoreZCFeedSameThread(block);
    for (unsigned axis = 3U; axis < 8U; ++axis)
        if (nativeAxes.address[axis] >= 'A' && nativeAxes.address[axis] <= 'Z' && block.has(nativeAxes.address[axis]))
            return StartPathCoreRotaryFeedSameThread(block);
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
    bool sixRatesValid = true, sixAuthorshipCurrent = true;
    if (r.xyzcuvFeed)
    {
        const bool absolute = CoordSys.GetTranslationSnapshot().distanceMode == 90;
        if (r.xyzcuv.absolute != absolute || r.xyzcuv.zOnlyCommissioning != IsEDM149ZFeedSameThread() ||
            r.xyzcuv.xyzOnlyCommissioning != IsEDM150XYZFeedSameThread() ||
            r.xyzcuv.sparseCommissioning != IsEDM154SparseFeedSameThread()) sixAuthorshipCurrent = false;
        for (unsigned i = 0U; i < 6U; ++i)
        {
            if (FeedDoubleBits(r.xyzcuv.absoluteTargetWCS[i]) != FeedDoubleBits(absolute ? m_pathFeedWCS[i] : 0.0) ||
                FeedDoubleBits(r.xyzcuv.absoluteTargetMCS[i]) != FeedDoubleBits(absolute ? m_pathFeedCandidate[i] : 0.0))
                sixAuthorshipCurrent = false;
            if (i < 3U) continue;
            const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
            if (FeedDoubleBits(r.xyzcuv.rotaryModulo[i - 3U]) != FeedDoubleBits(absolute ? axis.rotaryModulo : 0.0) ||
                r.xyzcuv.rotaryShortestPath[i - 3U] != (absolute && axis.useShortestPath)) sixAuthorshipCurrent = false;
            if (!std::isfinite(r.xyzcuv.rotaryFeedDegMin[i - 3U]) ||
                ((EDM149::StaticAxis(r.xyzcuv.zOnlyCommissioning,r.xyzcuv.xyzOnlyCommissioning,i) ||
                    (r.xyzcuv.sparseCommissioning && r.xyzcuv.deltaNative[i] == 0.0)) ?
                r.xyzcuv.rotaryFeedDegMin[i - 3U] != 0.0 : r.xyzcuv.rotaryFeedDegMin[i - 3U] <= 0.0) ||
                r.xyzcuv.rotaryFeedDegMin[i - 3U] > 100.0) sixRatesValid = false;
        }
    }
    bool fourAuthorshipCurrent = true;
    if (r.xyzcFeed)
    {
        const bool absolute = CoordSys.GetTranslationSnapshot().distanceMode == 90;
        const AxisContext& rotary = m_motion.GetAxisContext(3);
        if (r.xyzc.absolute != absolute ||
            FeedDoubleBits(r.xyzc.rotaryModulo) != FeedDoubleBits(absolute ? rotary.rotaryModulo : 0.0) ||
            r.xyzc.rotaryShortestPath != (absolute && rotary.useShortestPath)) fourAuthorshipCurrent = false;
        for (unsigned i = 0U; i < 4U; ++i)
            if (FeedDoubleBits(r.xyzc.absoluteTargetWCS[i]) != FeedDoubleBits(absolute ? m_pathFeedWCS[i] : 0.0) ||
                FeedDoubleBits(r.xyzc.absoluteTargetMCS[i]) != FeedDoubleBits(absolute ? m_pathFeedCandidate[i] : 0.0))
                fourAuthorshipCurrent = false;
    }
    const bool feedValueCurrent = r.eccentricFeed ? IsEccentricCFeedReceiptCurrentSameThread() : r.xyzcuvFeed ?
        (r.xyzcuv.valid && !r.xyzcFeed && !r.zcFeed && !r.rotaryFeed && r.xyzcuv.axisMask == 63U &&
            std::isfinite(r.xyzcuv.feedMMMin) && r.xyzcuv.feedMMMin > 0.0 && r.xyzcuv.feedMMMin <= 100.0 && sixRatesValid && sixAuthorshipCurrent &&
            !m_pathFeed.capturedFeed.valid && !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
            IsNCXYZCUVFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) && IsXYZCUVFeedProfileCurrentSameThread()) : r.xyzcFeed ?
        (r.xyzc.valid && !r.zcFeed && !r.rotaryFeed && !r.xyzcuvFeed && r.xyzc.axisMask == 15U && fourAuthorshipCurrent &&
            std::isfinite(r.xyzc.feedMMMin) && r.xyzc.feedMMMin > 0.0 && r.xyzc.feedMMMin <= 100.0 &&
            std::isfinite(r.xyzc.rotaryFeedDegMin) && r.xyzc.rotaryFeedDegMin > 0.0 && r.xyzc.rotaryFeedDegMin <= 100.0 &&
            !m_pathFeed.capturedFeed.valid && !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
            IsNCXYZCFeedNeutralFrame(CoordSys.GetTranslationSnapshot()) && IsXYZCFeedProfileCurrentSameThread()) : r.zcFeed ?
        (r.zc.valid && !r.rotaryFeed && !r.xyzcFeed && !r.xyzcuvFeed && r.zc.axisMask == 12U &&
            r.zc.absolute == (CoordSys.GetTranslationSnapshot().distanceMode == 90) &&
            std::isfinite(r.zc.feedMMMin) && r.zc.feedMMMin > 0.0 && r.zc.feedMMMin <= 100.0 &&
            std::isfinite(r.zc.rotaryFeedDegMin) && r.zc.rotaryFeedDegMin > 0.0 && r.zc.rotaryFeedDegMin <= 100.0 &&
            !m_pathFeed.capturedFeed.valid && !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
            IsNCZCFeedNeutralFrame(CoordSys.GetTranslationSnapshot())) : r.rotaryFeed ?
        (r.rotary.valid && !r.zcFeed && !r.xyzcFeed && !r.xyzcuvFeed && r.rotary.axisMask != 0U && (r.rotary.axisMask & ~0xf8U) == 0U &&
            (r.rotary.axisMask & (r.rotary.axisMask - 1U)) == 0U &&
            std::isfinite(r.rotary.feedDegMin) && r.rotary.feedDegMin > 0.0 && r.rotary.feedDegMin <= 100.0 &&
            !m_pathFeed.capturedFeed.valid && !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
            IsNCRotaryFeedNeutralFrame(CoordSys.GetTranslationSnapshot())) :
        (IsCncFeedValueCurrentSameThread(m_pathFeed.capturedFeed) &&
            FeedDoubleBits(r.line.feedMMMin) == FeedDoubleBits(m_pathFeed.capturedFeed.feedMMMin));
    if (!feedValueCurrent || !r.valid || !committed || !commit.IsValid() || !ledgerFound ||
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
    if (!IsEDMDiagnosticQuietSameThread()) RtPrintf("[CNC-TRANSLATION-PATH] phase=BOUND kind=%s translationGen=%llu wcs=%d dispatch=%llu epoch=%llu seg=%llu sourcePC=%d\n",
        r.eccentricFeed ? "ECCENTRIC_C" : r.xyzcuvFeed ? "XYZCUV_LINE" : r.xyzcFeed ? "XYZC_LINE" : r.zcFeed ? "ZC_LINE" : r.rotaryFeed ? "ROTARY_LINE" : "LINE",
        static_cast<unsigned long long>(r.translationGeneration), CoordSys.GetTranslationSnapshot().wcsCode,
        static_cast<unsigned long long>(dispatchId), static_cast<unsigned long long>(r.identity.epoch),
        static_cast<unsigned long long>(r.identity.segmentId), sourcePC);
    if (m_cncFeed.selected)
    {
        CommitCncFeedSameThread();
        return;
    }
    if (!r.eccentricFeed && !r.rotaryFeed && !r.zcFeed && !r.xyzcFeed && !r.xyzcuvFeed)
        CommitCncModalFeedSameThread(m_pathFeed.capturedFeed, dispatchId, commit.sequence, sourcePC, sourceLine);
    LogPathCoreFeedSameThread("BOUND");
}

NC_PATH_FEED_NOINLINE
void NCManager::ObservePathCoreFeedFeedbackSameThread(const MotionFeedbackEvent& event,
    bool ledgerAccepted)
{
    ObserveG76OrbitFeedbackSameThread(event, ledgerAccepted);
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
        if (!m_pathFeed.consumerAccepted || m_pathFeed.completed || event.rejectReason != MotionRejectReason::NONE || event.errorCode != 0U ||
            ((m_pathFeedMotion.receipt.eccentricFeed || m_pathFeedMotion.receipt.xyzcuvFeed || m_pathFeedMotion.receipt.xyzcFeed || m_pathFeedMotion.receipt.zcFeed ||
                (m_pathFeedMotion.receipt.rotaryFeed && !m_pathFeedMotion.receipt.rotary.point)) && !m_pathFeed.consumerStarted)) break;
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
    if (m_state == NCState::ALARM)
    { m_edm94.boundary = EDM94NCState::FeedBoundary{}; return false; }
    if (!m_pathFeed.pending)
    {
        if (m_gapWindow.active && (m_gapWindow.budgetProven || m_gapWindow.normalProven) && m_pathFeed.completed &&
            m_pathHold.bound && m_pathFeed.dispatch == m_pathHold.dispatch)
        {
            if (!CompleteGapPathSourceSameThread(true)) return false;
        }
        return m_pathFeed.completed;
    }
    if (IsFeedHoldActive())
    { m_edm94.boundary = EDM94NCState::FeedBoundary{}; return false; }
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
    // FIX8: retain the completed EDM94 G1 and its correction authority until
    // the exact old Epoch is physically ready for the next ordinary command.
    if (m_edm94.latched && m_pathFeedMotion.receipt.xyzcuvFeed &&
        !WaitEDM94FeedBoundarySameThread()) return false;
    if (CoordSys.toolRadiusMode == 40 && !m_pathFeedMotion.receipt.eccentricFeed && !m_pathFeedMotion.receipt.rotaryFeed && !m_pathFeedMotion.receipt.zcFeed && !m_pathFeedMotion.receipt.xyzcFeed && !m_pathFeedMotion.receipt.xyzcuvFeed)
        RetainPathCoreFeedSameThread(); // Native rotary/mixed paths do not enter XYZ retained geometry.
    m_pathFeed.pending = false;
    ++m_pathFeed.done;
    LogPathCoreFeedSameThread("COMPLETED");
    if (m_pathFeedMotion.receipt.eccentricFeed || m_pathFeedMotion.receipt.rotaryFeed || m_pathFeedMotion.receipt.zcFeed || m_pathFeedMotion.receipt.xyzcFeed || m_pathFeedMotion.receipt.xyzcuvFeed) return true;
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
    if (IsEDMDiagnosticQuietSameThread() && phase != nullptr &&
        (std::strcmp(phase, "SUBMITTED") == 0 || std::strcmp(phase, "BOUND") == 0 ||
            std::strcmp(phase, "COMPLETED") == 0 || std::strcmp(phase, "FINALIZED") == 0)) return;
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
    if (IsEDMDiagnosticQuietSameThread()) return;
    if (m_pathFeedMotion.receipt.eccentricFeed && m_eccentricProducer)
    {
        const auto& w = *m_eccentricProducer;
        const auto& p = w.runtime.AuthoredPath();
        RtPrintf("[BASE79M][ECC-NC] dispatch=%llu role=%u authoredMask=%u generatedMask=%u groupMask=%u H=%d sign=%d mode=%d programmedCBits=%llu sweepBits=%llu feedBits=%llu zDeltaBits=%llu programmedZBits=%llu programmedFBits=%llu unit=%s\n",
            static_cast<unsigned long long>(m_pathFeed.dispatch), p.electrodeAxis + 1U,
            p.authoredMask, p.generatedMask, p.groupMask, p.source.toolHCode, p.source.toolLengthMode == 43 ? 1 : -1,
            p.source.distanceMode, static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedWCS[3])),
            static_cast<unsigned long long>(FeedDoubleBits(p.sweepDeg)),
            static_cast<unsigned long long>(FeedDoubleBits(p.feedDegMin)),
            static_cast<unsigned long long>(FeedDoubleBits(p.zDeltaMM)),
            static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedWCS[2])),
            static_cast<unsigned long long>(FeedDoubleBits(m_pathFeed.eccentricProgrammedFeed)),
            (m_pathFeedProgrammed[0] || m_pathFeedProgrammed[1]) ? "XYZ_MM_MIN" :
                m_pathFeedProgrammed[2] ? "Z_MM_MIN" : "DEG_MIN");
        if (m_pathFeedProgrammed[0] || m_pathFeedProgrammed[1])
            RtPrintf("[BASE79M][ECC-XYZ] dispatch=%llu linearMask=%u programmedXBits=%llu programmedYBits=%llu xDeltaBits=%llu yDeltaBits=%llu linearLengthBits=%llu\n",
                static_cast<unsigned long long>(m_pathFeed.dispatch), p.authoredMask & 7U,
                static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedWCS[0])),
                static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedWCS[1])),
                static_cast<unsigned long long>(FeedDoubleBits(p.xDeltaMM)),
                static_cast<unsigned long long>(FeedDoubleBits(p.yDeltaMM)),
                static_cast<unsigned long long>(FeedDoubleBits(std::hypot(std::hypot(p.xDeltaMM, p.yDeltaMM), p.zDeltaMM))));
        for (unsigned a = 0U; a < 8U; ++a)
            RtPrintf("[BASE79M][ECC-ENDPOINT] dispatch=%llu axis=%u startMCSBits=%llu endMCSBits=%llu startPulseBits=%llu endPulseBits=%llu\n",
                static_cast<unsigned long long>(m_pathFeed.dispatch), a,
                static_cast<unsigned long long>(FeedDoubleBits(p.startMCS[a])),
                static_cast<unsigned long long>(FeedDoubleBits(w.physicalEndMCS[a])),
                static_cast<unsigned long long>(FeedDoubleBits(w.runtime.StartPulse()[a])),
                static_cast<unsigned long long>(FeedDoubleBits(w.runtime.EndPulse()[a])));
        return;
    }
    if (m_pathFeedMotion.receipt.xyzcuvFeed)
    {
        const NCXYZCUVFeedLineValue& xyzcuv = m_pathFeedMotion.receipt.xyzcuv;
        // NC-thread proof rows remain bounded even at maximum uint64 widths.
        RtPrintf("[BASE74][XYZCUV-GEO] run=%llu dispatch=%llu mask=%u validMask=%u mode=%d FMMMinBits=%llu linearLengthMMBits=%llu nominalSecondsBits=%llu cFeedDegMinBits=%llu uFeedDegMinBits=%llu vFeedDegMinBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned int>(xyzcuv.axisMask), static_cast<unsigned int>(m_pathFeedMotion.receipt.validAxisMask),
            xyzcuv.absolute ? 90 : 91,
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.feedMMMin)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.linearLengthMM)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.nominalSeconds)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.rotaryFeedDegMin[0])),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.rotaryFeedDegMin[1])),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.rotaryFeedDegMin[2])));
        RtPrintf("[BASE74][XYZCUV-DYN] run=%llu dispatch=%llu lengthPulseBits=%llu velocityPPSBits=%llu accTimeBits=%llu decTimeBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.lengthPulse)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.velocityPPS)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.accTime)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.decTime)));
        for (unsigned i = 0U; i < 8U; ++i)
        {
            const bool selected = (xyzcuv.axisMask & (1U << i)) != 0U;
            RtPrintf("[BASE74][XYZCUV-AXIS] run=%llu dispatch=%llu axis=%u selected=%u programmed=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu deltaBits=%llu ppuBits=%llu velocityBits=%llu\n",
                static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
                i, selected ? 1U : 0U, m_pathFeedProgrammed[i] ? 1U : 0U,
                static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.startMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.endMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedCandidate[i])),
                static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.startPulse[i])),
                static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.endPulse[i])),
                static_cast<unsigned long long>(FeedDoubleBits(i < 6U ? xyzcuv.deltaNative[i] : 0.0)),
                static_cast<unsigned long long>(FeedDoubleBits(i < 6U ? xyzcuv.pulsePerUnit[i] : 0.0)),
                static_cast<unsigned long long>(FeedDoubleBits(i < 6U ? xyzcuv.axisVelocityPPS[i] : 0.0)));
        }
        if (xyzcuv.absolute)
            for (unsigned i = 0U; i < 6U; ++i)
                RtPrintf("[BASE74][XYZCUV-ABS] run=%llu dispatch=%llu axis=%u wcsBits=%llu mcsBits=%llu resolvedBits=%llu moduloBits=%llu shortest=%u\n",
                    static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch), i,
                    static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.absoluteTargetWCS[i])),
                    static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.absoluteTargetMCS[i])),
                    static_cast<unsigned long long>(FeedDoubleBits(xyzcuv.endMCS[i])),
                    static_cast<unsigned long long>(FeedDoubleBits(i >= 3U ? xyzcuv.rotaryModulo[i - 3U] : 0.0)),
                    i >= 3U && xyzcuv.rotaryShortestPath[i - 3U] ? 1U : 0U);
        return;
    }
    if (m_pathFeedMotion.receipt.xyzcFeed)
    {
        const NCXYZCFeedLineValue& xyzc = m_pathFeedMotion.receipt.xyzc;
        // NC-thread proof rows remain bounded even at maximum uint64 widths.
        RtPrintf("[BASE75][XYZC-GEO] run=%llu dispatch=%llu mask=%u validMask=%u mode=%d FMMMinBits=%llu linearLengthMMBits=%llu nominalSecondsBits=%llu rotaryFeedDegMinBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned int>(xyzc.axisMask), static_cast<unsigned int>(m_pathFeedMotion.receipt.validAxisMask),
            xyzc.absolute ? 90 : 91,
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.feedMMMin)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.linearLengthMM)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.nominalSeconds)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.rotaryFeedDegMin)));
        RtPrintf("[BASE75][XYZC-DYN] run=%llu dispatch=%llu lengthPulseBits=%llu velocityPPSBits=%llu accTimeBits=%llu decTimeBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.lengthPulse)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.velocityPPS)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.accTime)),
            static_cast<unsigned long long>(FeedDoubleBits(xyzc.decTime)));
        for (unsigned i = 0U; i < 8U; ++i)
        {
            const bool selected = (xyzc.axisMask & (1U << i)) != 0U;
            RtPrintf("[BASE75][XYZC-AXIS] run=%llu dispatch=%llu axis=%u selected=%u programmed=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu deltaBits=%llu ppuBits=%llu velocityBits=%llu\n",
                static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
                i, selected ? 1U : 0U, m_pathFeedProgrammed[i] ? 1U : 0U,
                static_cast<unsigned long long>(FeedDoubleBits(xyzc.startMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(xyzc.endMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedCandidate[i])),
                static_cast<unsigned long long>(FeedDoubleBits(xyzc.startPulse[i])),
                static_cast<unsigned long long>(FeedDoubleBits(xyzc.endPulse[i])),
                static_cast<unsigned long long>(FeedDoubleBits(i < 4U ? xyzc.deltaNative[i] : 0.0)),
                static_cast<unsigned long long>(FeedDoubleBits(i < 4U ? xyzc.pulsePerUnit[i] : 0.0)),
                static_cast<unsigned long long>(FeedDoubleBits(i < 4U ? xyzc.axisVelocityPPS[i] : 0.0)));
        }
        if (xyzc.absolute)
            for (unsigned i = 0U; i < 4U; ++i)
                RtPrintf("[BASE75][XYZC-ABS] run=%llu dispatch=%llu axis=%u wcsBits=%llu mcsBits=%llu resolvedBits=%llu moduloBits=%llu shortest=%u\n",
                    static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch), i,
                    static_cast<unsigned long long>(FeedDoubleBits(xyzc.absoluteTargetWCS[i])),
                    static_cast<unsigned long long>(FeedDoubleBits(xyzc.absoluteTargetMCS[i])),
                    static_cast<unsigned long long>(FeedDoubleBits(xyzc.endMCS[i])),
                    static_cast<unsigned long long>(FeedDoubleBits(i == 3U ? xyzc.rotaryModulo : 0.0)),
                    i == 3U && xyzc.rotaryShortestPath ? 1U : 0U);
        return;
    }
    if (m_pathFeedMotion.receipt.zcFeed)
    {
        const NCZCFeedLineValue& zc = m_pathFeedMotion.receipt.zc;
        // Bounded proof rows on the NC thread only. Each row retains run and
        // dispatch identity; no maximum-width uint64 formatting can exceed the
        // RT console's fixed line buffer or hide deceleration/newline fields.
        RtPrintf("[BASE71][ZC-TARGET] run=%llu dispatch=%llu mode=%d zWCSBits=%llu cWCSBits=%llu zMCSBits=%llu cMCSBits=%llu moduloBits=%llu shortest=%u\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            zc.absolute ? 90 : 91,
            static_cast<unsigned long long>(FeedDoubleBits(zc.absoluteTargetZWCS)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.absoluteTargetCWCS)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.absoluteTargetZMCS)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.absoluteTargetCMCS)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.rotaryModulo)), zc.rotaryShortestPath ? 1U : 0U);
        RtPrintf("[BASE71][ZC-GEO] run=%llu dispatch=%llu mask=%u validMask=%u FMMMinBits=%llu deltaZMMBits=%llu deltaCDegBits=%llu nominalSecondsBits=%llu rotaryFeedDegMinBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned int>(zc.axisMask), static_cast<unsigned int>(m_pathFeedMotion.receipt.validAxisMask),
            static_cast<unsigned long long>(FeedDoubleBits(zc.feedMMMin)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.deltaZMM)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.deltaCDeg)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.nominalSeconds)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.rotaryFeedDegMin)));
        RtPrintf("[BASE71][ZC-DYN] run=%llu dispatch=%llu pulsePerMMBits=%llu pulsePerDegreeBits=%llu lengthPulseBits=%llu velocityPPSBits=%llu linearVelocityPPSBits=%llu rotaryVelocityPPSBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned long long>(FeedDoubleBits(zc.pulsePerMM)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.pulsePerDegree)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.lengthPulse)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.velocityPPS)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.linearVelocityPPS)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.rotaryVelocityPPS)));
        RtPrintf("[BASE71][ZC-TIME] run=%llu dispatch=%llu accTimeBits=%llu decTimeBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned long long>(FeedDoubleBits(zc.accTime)),
            static_cast<unsigned long long>(FeedDoubleBits(zc.decTime)));
        for (unsigned i = 0U; i < 8U; ++i)
        {
            const bool selected = (zc.axisMask & (1U << i)) != 0U;
            RtPrintf("[BASE71][ZC-AXIS] run=%llu dispatch=%llu axis=%u selected=%u programmed=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu\n",
                static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
                i, selected ? 1U : 0U, m_pathFeedProgrammed[i] ? 1U : 0U,
                static_cast<unsigned long long>(FeedDoubleBits(zc.startMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(zc.endMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedCandidate[i])),
                static_cast<unsigned long long>(FeedDoubleBits(zc.startPulse[i])),
                static_cast<unsigned long long>(FeedDoubleBits(zc.endPulse[i])));
        }
        return;
    }
    if (m_pathFeedMotion.receipt.rotaryFeed)
    {
        const NCRotaryFeedLineValue& rotary = m_pathFeedMotion.receipt.rotary;
        // NC-thread-only proof row; no per-cycle RT printing or allocations.
        RtPrintf("[BASE69][ROTARY-TARGET] run=%llu dispatch=%llu mode=%d absoluteWCSBits=%llu absoluteMCSBits=%llu moduloBits=%llu shortest=%u\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            CoordSys.GetTranslationSnapshot().distanceMode,
            static_cast<unsigned long long>(FeedDoubleBits(rotary.absoluteTargetWCS)),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.absoluteTargetMCS)),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.rotaryModulo)), rotary.rotaryShortestPath ? 1U : 0U);
        RtPrintf("[BASE68][ROTARY-GEO] run=%llu dispatch=%llu mask=%u validMask=%u FDegMinBits=%llu sweepDegBits=%llu pulsePerDegreeBits=%llu lengthPulseBits=%llu velocityPPSBits=%llu point=%u\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned int>(rotary.axisMask), static_cast<unsigned int>(m_pathFeedMotion.receipt.validAxisMask),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.feedDegMin)),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.sweepDeg)),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.pulsePerDegree)),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.lengthPulse)),
            static_cast<unsigned long long>(FeedDoubleBits(rotary.velocityPPS)), rotary.point ? 1U : 0U);
        for (unsigned i = 0U; i < 8U; ++i)
        {
            const bool selected = (rotary.axisMask & (1U << i)) != 0U;
            RtPrintf("[BASE68][ROTARY-AXIS] run=%llu dispatch=%llu axis=%u selected=%u programmed=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu\n",
                static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
                i, selected ? 1U : 0U, m_pathFeedProgrammed[i] ? 1U : 0U,
                static_cast<unsigned long long>(FeedDoubleBits(rotary.startMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(rotary.endMCS[i])),
                static_cast<unsigned long long>(FeedDoubleBits(m_pathFeedCandidate[i])),
                static_cast<unsigned long long>(FeedDoubleBits(rotary.startPulse[i])),
                static_cast<unsigned long long>(FeedDoubleBits(rotary.endPulse[i])));
        }
        return;
    }
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
    if (IsEDM171AirGrantCurrentSameThread()) return false;
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

NC_PATH_FEED_NOINLINE
bool NCManager::IsG76OrbitBlockShapeValid(const NCBlock& block, bool allowMissingFeed, int unitsMode) noexcept
{
    if ((unitsMode != 20 && unitsMode != 21) || block.isEmpty || block.isGoto || block.isBlockSkip ||
        !block.hasG || block.gCode != 76 || block.gCount != 1 || block.gCodes[0] != 76 || block.mCount != 0 ||
        !block.has('P') || block.val('P') != 11.0 || !block.has('R') ||
        !(block.has('X') || block.has('Y') || block.has('Z')) || (!allowMissingFeed && !block.has('F'))) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
    {
        if (!block.has(word)) continue;
        if ((word != 'N' && word != 'X' && word != 'Y' && word != 'Z' && word != 'R' &&
             word != 'P' && word != 'F' && word != 'B') || !std::isfinite(block.val(word))) return false;
        if ((word == 'X' || word == 'Y' || word == 'Z') &&
            !std::isfinite(NCTranslationLengthToMM(block.val(word),unitsMode))) return false;
    }
    const double radius = NCTranslationLengthToMM(block.val('R'),unitsMode);
    const double feed = block.has('F') ? NCTranslationLengthToMM(block.val('F'),unitsMode) : 0.0;
    return std::isfinite(radius) && radius > 0.0 && radius <= NCG76::MaximumRadiusMm &&
        (!block.has('F') || (std::isfinite(feed) && feed > 0.0 && feed <= NCG76::MaximumFeedMmMin)) &&
        (!block.has('B') || block.val('B') == 0.0 || block.val('B') == 2.0 || block.val('B') == 3.0 || block.val('B') == 4.0);
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsG76OrbitFrameCurrentSameThread() const noexcept
{
    if (!CoordSys.IsTranslationRunBound() || !CoordSys.IsTranslationRunCurrent()) return false;
    const NCTranslationSnapshot& source = CoordSys.GetTranslationSnapshot();
    if (!NCG76::FrameSupported(source) || source.runToken == 0ULL ||
        source.runToken != m_pathCoreLiveBookkeeping.currentRunToken ||
        source.distanceMode != (CoordSys.isAbsoluteMode ? 90 : 91) ||
        source.unitsMode != (CoordSys.isInchMode ? 20 : 21) || CoordSys.activePlane != 17 ||
        CoordSys.toolRadiusMode != 40 || CoordSys.isPolarCoordinateActive || CoordSys.isCAxisOffsetRotationEnabled)
        return false;
    for (unsigned a = 0U; a < 3U; ++a)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(a));
        if (m_axisNames[a] != static_cast<char>('X'+a) || source.axisIdentity.address[a] != m_axisNames[a] ||
            !axis.isExist || axis.axisType != AxisType::LINEAR) return false;
    }
    return true;
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsG76OrbitBlockAllowedSameThread(const NCBlock& block) noexcept
{
    if (m_edm94.latched && m_edm94.gapHistoryArm==292U) return IsEDM173OrbitAllowedSameThread(block);
    bool valid = IsG76OrbitBlockShapeValid(block,true,CoordSys.isInchMode ? 20 : 21) &&
        IsG76OrbitFrameCurrentSameThread() && IsPathCoreLiveNativeConfigCurrentSameThread() &&
        m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY && !Close_System_Com_flag &&
        m_externalReadyInterlock && !m_isG66Active && m_macroStack.empty() && !Homing.IsActive() &&
        !m_isSingleBlockEnabled && !AlarmManager::GetInstance().HasAlarm() && !m_edm94.latched &&
        !m_g76Orbit->active && !m_pathFeed.pending && !m_pathArc.pending && !m_pathReplay.pending &&
        !m_pathHold.armed && !m_pathHold.bound && !m_gapDryRun.active && !m_gapPath.active &&
        !m_gapWindow.active && !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
        !m_cutterLine.leadOutRequired && !m_motion.HasPendingSafetyOrRecoveryRequests();
    const auto* authored = GetBaseProgramCache().TryGetLine(GetActiveDispatchPC());
    NCBlock literal{};
    valid = valid && authored && authored->parsedBlock.error == NCParseError::NONE &&
        authored->parsedBlock.duplicateAddressMask == 0U && !authored->parsedBlock.dependsOnMacroState &&
        authored->parsedBlock.controlType == NCParsedControlType::NONE &&
        NCPreparedBlockQueueShadow::TryBuildLiteralBlock(authored->parsedBlock,literal) &&
        IsG76OrbitBlockShapeValid(literal,true,CoordSys.isInchMode ? 20 : 21);
    if (valid)
        for (char word = 'A'; word <= 'Z'; ++word)
            if (literal.has(word) != block.has(word) ||
                (literal.has(word) && FeedDoubleBits(literal.val(word)) != FeedDoubleBits(block.val(word)))) valid = false;
    if (!valid) RejectG76OrbitSameThread("LITERAL_FRAME_OR_EXECUTION_SCOPE",AlarmManager::G_Code_Invalid_parameter);
    return valid;
}

NC_PATH_FEED_NOINLINE
bool NCManager::PrepareG76OrbitTranslationSameThread(const NCBlock& block)
{
    if (m_edm94.latched && m_edm94.gapHistoryArm==292U)
        return IsEDM173OrbitAllowedSameThread(block) && FreezeEDM173TranslationSameThread();
    if (block.gCode==76 ? !IsG76OrbitBlockAllowedSameThread(block) :
        (!m_edm171Air->selected || !IsEDM171AirGrantCurrentSameThread() || !IsG76OrbitFrameCurrentSameThread() ||
         !IsPathCoreLiveNativeConfigCurrentSameThread() || m_g76Orbit->active)) return false;
    if (CoordSys.IsTranslationRunFrozen())
        return IsFixedTranslationTravelCurrentSameThread() && m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot());
    const NCTranslationSnapshot candidate = CoordSys.GetTranslationSnapshot();
    if (!CoordSys.FreezeTranslationRun())
    { RejectG76OrbitSameThread("FRAME_FREEZE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
    for (unsigned a = 0U; a < 8U; ++a)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(a));
        FixedTranslationTravelPolicy& captured = m_fixedTranslationTravel[a];
        captured.minimum[0]=axis.travelLimit1Negative_unit; captured.minimum[1]=axis.travelLimit2Negative_unit; captured.minimum[2]=axis.travelLimit3Negative_unit;
        captured.maximum[0]=axis.travelLimit1Positive_unit; captured.maximum[1]=axis.travelLimit2Positive_unit; captured.maximum[2]=axis.travelLimit3Positive_unit;
        captured.enabled[0]=axis.travelLimit1Enable; captured.enabled[1]=axis.travelLimit2Enable; captured.enabled[2]=axis.travelLimit3Enable;
        captured.homed=axis.isHomed; captured.positionalRotary=axis.isExist && axis.axisType==AxisType::ROTARY;
        captured.shortestPath=axis.useShortestPath; captured.rotaryModulo=axis.rotaryModulo;
        captured.maximumVelocity=axis.maxVel_PPS; captured.accelerationTime=axis.G00_acc_time;
        captured.decelerationTime=axis.G00_dec_time; captured.stopDecelerationTime=axis.Stop_dec_time;
    }
    const NCTranslationSnapshot frozen=CoordSys.GetTranslationSnapshot();
    m_fixedTranslationTravelGeneration=frozen.generation;
    if (!SameNCTranslationSnapshot(candidate,frozen) || !m_motion.PublishNCTranslation(frozen))
    { RejectG76OrbitSameThread("FRAME_PUBLICATION",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
    if (frozen.rotationMode==68) m_motion.SetCoordinateTransform(false,0.0,0.0,0.0,0.0,0.0,0.0);
    RtPrintf("[EDM170] event=FRAME_FROZEN run=%llu generation=%llu distance=%d units=%d discharge=0\n",
        static_cast<unsigned long long>(frozen.runToken),static_cast<unsigned long long>(frozen.generation),frozen.distanceMode,frozen.unitsMode);
    return true;
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartG76OrbitSameThread(const NCBlock& block)
{
    if (m_edm94.latched && m_edm94.gapHistoryArm==292U) return StartEDM173OrbitSameThread(block);
    if (!IsG76OrbitBlockAllowedSameThread(block)) return nullptr;
    const auto& confirm=m_g76Confirmation;
    const int pc=GetActiveDispatchPC();
    const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if (!m_pathFeed.armed || m_pathFeed.run!=m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache!=GetBaseProgramCache().GetGeneration() || !source || !CoordSys.IsTranslationRunFrozen() ||
        !IsPathCoreFeedConfigurationValid() || !IsFixedTranslationTravelCurrentSameThread() ||
        !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()) ||
        !m_programMotionLease.IsValid() || m_programMotionLease.owner!=MotionOwner::AUTO ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) || !m_motion.IsGroupNCDrained() ||
        !m_motion.IsGroupDone() || m_motion.GetCommandIngressSize()!=0U || m_motion.GetCommandReplaySize()!=0U ||
        m_motion.GetMotionFeedbackOverflowCount()!=0ULL || m_motion.GetMotionFeedbackProducerNoticeOverflowCount()!=0ULL ||
        m_motionFeedbackSequenceGapCount!=0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger) ||
        !confirm.confirmed || confirm.waiting || confirm.run!=m_pathCoreLiveBookkeeping.currentRunToken ||
        confirm.cache!=GetBaseProgramCache().GetGeneration() || !confirm.dispatch ||
        m_currentExecutingBlockDispatchId<=confirm.dispatch || pc<confirm.firstPC ||
        !m_currentExecutingBlockDispatchId)
    { RejectG76OrbitSameThread("CONFIRMATION_OR_STOPPED_ADMISSION",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    ResetG76OrbitStateSameThread();
    auto& s=*m_g76Orbit;
    s.run=m_pathCoreLiveBookkeeping.currentRunToken; s.cache=GetBaseProgramCache().GetGeneration();
    s.dispatch=m_currentExecutingBlockDispatchId; s.pc=pc; s.line=source->sourceLineNumber;
    if (!CaptureCncFeedValueSameThread(block,s.feed) || s.feed.feedMMMin>NCG76::MaximumFeedMmMin)
    { RejectG76OrbitSameThread("FEED_PROVENANCE",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    unsigned mode=0U;
    if (block.has('B')) mode=static_cast<unsigned>(block.val('B'));
    else if (m_g76Modal.valid)
    {
        if (m_g76Modal.run!=s.run || m_g76Modal.cache!=s.cache || !m_g76Modal.commit ||
            !m_g76Modal.lease.Matches(m_programMotionLease))
        { RejectG76OrbitSameThread("INHERITED_B_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return nullptr; }
        mode=m_g76Modal.mode; s.inheritedB=true;
    }
    NCG76::PlanInput input{};
    input.flushMode=mode; input.feedMmMin=s.feed.feedMMMin;
    const NCTranslationSnapshot frozen=CoordSys.GetTranslationSnapshot();
    input.radiusMm=NCTranslationLengthToMM(block.val('R'),frozen.unitsMode);
    s.mode=mode; s.sourceG=76;
    if (mode==3U || mode==4U)
    {
        input.centerReturnFeedMmMin=mode==3U ? m_edm171Air->speeds.b3MmMin : m_edm171Air->speeds.b4MmMin;
        EDMRecipe::ParameterSnapshot height{};
        if (!ReadEDM171AirE6SameThread(height))
        { RejectG76OrbitSameThread("E6_HEIGHT_SNAPSHOT",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
        input.heightMm=static_cast<double>(height.actualValue)/static_cast<double>(EDMRecipe::Scale);
    }
    NCG76::Point8 start{};
    for (unsigned a=0U;a<8U;++a) start[a]=CoordSys.commandedMCS[a];
    double programStart[8]={};
    NCTranslationInversePoint(frozen,start.data(),programStart);
    for (unsigned a=0U;a<3U;++a)
    {
        if (!std::isfinite(programStart[a]))
        { RejectG76OrbitSameThread("ACCEPTED_NATIVE_BASELINE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return nullptr; }
        const char word=static_cast<char>('X'+a);
        input.start[a]=frozen.distanceMode==90 ? programStart[a] : 0.0;
        input.end[a]=block.has(word) ? NCTranslationLengthToMM(block.val(word),frozen.unitsMode) : input.start[a];
    }
    if (!NCG76::Build(input,s.program) || !(mode==2U ? NCG76::BuildNativeBase(s.program,frozen,start,s.native) : NCG76::BuildNative(s.program,frozen,start,s.native)))
    { RejectG76OrbitSameThread("PROGRAM_OR_TRANSFORMED_ENVELOPE",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    bool xyz[8]={true,true,true,false,false,false,false,false};
    for (unsigned k=0U;k<s.native.count;++k)
    {
        double words[8]={},preview[8]={};
        for (unsigned a=0U;a<3U;++a) words[a]=s.program.points[k].position[a];
        CoordSys.Preview_WCS_to_MCS(words,xyz,preview);
        for (unsigned a=0U;a<8U;++a)
        {
            if (FeedDoubleBits(preview[a])!=FeedDoubleBits(s.native.pointsMCS[k][a]))
            { RejectG76OrbitSameThread("FROZEN_PREVIEW_MISMATCH",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return nullptr; }
            const AxisContext& axis=m_motion.GetAxisContext(static_cast<int>(a));
            if (!axis.isExist) continue;
            const double low[3]={axis.travelLimit1Negative_unit,axis.travelLimit2Negative_unit,axis.travelLimit3Negative_unit};
            const double high[3]={axis.travelLimit1Positive_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Positive_unit};
            const bool enabled[3]={axis.travelLimit1Enable,axis.travelLimit2Enable,axis.travelLimit3Enable};
            for (unsigned limit=0U;limit<3U;++limit)
                if (enabled[limit] && (!std::isfinite(low[limit]) || !std::isfinite(high[limit]) || low[limit]>high[limit] ||
                    start[a]<low[limit] || start[a]>high[limit] || preview[a]<low[limit] || preview[a]>high[limit]))
                { RejectG76OrbitSameThread("SOFTWARE_TRAVEL",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
        }
    }
    if (mode==2U) return StartEDM171MixedMotionSameThread(block);
    // This latch precedes the one atomic Motion publication and survives any
    // subsequent NC invalidation until old physical work has actually retired.
    m_g76PowerInhibited=true;
    ForceEDMPLCOutputsOffSameThread("G76_AIR_RUN_ADMISSION");
    const bool accepted=m_motion.TryG76MoveTransactionalTail(s.native,CoordSys.commandedMCS,s.receipt);
    const auto& r=s.receipt;
    bool receipt=accepted && r.valid && r.commandAccepted && r.captureBound && r.tailCommitted &&
        r.count==s.native.count && r.count>0U && r.count<=NCG76::MaximumPoints && r.axisMask==7U &&
        (r.validAxisMask&7U)==7U && r.tail.axisMask==r.validAxisMask &&
        r.translationGeneration==frozen.generation && r.ownerLease.Matches(m_programMotionLease) &&
        r.tail.IsCommitted() && r.tail.captureBound && r.tail.ownerLease.Matches(r.ownerLease) &&
        FeedFullIdentity(r.tail.identity,r.identities[r.count-1U]);
    for (unsigned a=0U;a<8U && receipt;++a)
        receipt=FeedDoubleBits(r.startMCS[a])==FeedDoubleBits(start[a]) &&
            FeedDoubleBits(r.endMCS[a])==FeedDoubleBits(s.native.pointsMCS[s.native.count-1U][a]) &&
            FeedDoubleBits(CoordSys.commandedMCS[a])==FeedDoubleBits(r.endMCS[a]);
    for (unsigned k=0U;k<r.count && k<NCG76::MaximumPoints && receipt;++k)
    {
        const auto& id=r.identities[k];
        receipt=id.IsAssigned() && id.epoch==m_motion.GetCurrentExecutionEpoch() &&
            id.source==MotionCommandSource::NC_MEMORY && id.sourceBlockId==static_cast<MotionSourceBlockId>(pc);
        for (unsigned j=0U;j<k;++j) if (id.Matches(r.identities[j])) receipt=false;
    }
    if (!receipt)
    {
        RtPrintf("[EDM171] event=NATIVE_REJECT gate=%s axis=%d leg=%d accepted=%u\n",
            MotionG76AdmissionGateName(r.rejectionGate),r.rejectionAxis,r.rejectionSegment,r.commandAccepted?1U:0U);
        if (!r.commandAccepted && AlarmManager::GetInstance().HasAlarm()) { ChangeState(NCState::ALARM); return nullptr; }
        RejectG76OrbitSameThread("NATIVE_BATCH_RECEIPT",r.commandAccepted ? AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY : AlarmManager::PATH_MOTION_NOT_ADMITTED); return nullptr;
    }
    s.active=true;
    RtPrintf("[EDM170] event=AIR_RUN_BEGIN run=%llu cache=%llu dispatch=%llu pc=%d B=%u inheritedB=%u count=%u grow=%u R=%.6f F=%.6f H=%.6f centerReturnF=%.6f trigger=%s chordError=%.9f discharge=0\n",
        static_cast<unsigned long long>(s.run),static_cast<unsigned long long>(s.cache),static_cast<unsigned long long>(s.dispatch),s.pc,
        mode,s.inheritedB?1U:0U,s.native.count,NCG76::GrowSegments,input.radiusMm,input.feedMmMin,input.heightMm,
        input.centerReturnFeedMmMin>0.0 ? input.centerReturnFeedMmMin : input.feedMmMin,mode ? "DEMO_HALF_TURN" : "NONE",s.program.chordErrorBoundMm);
    return [](NCManager* n) { return n->CompleteG76OrbitSameThread(); };
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitG76OrbitCaptureSameThread(NCBlockDispatchId dispatch,const MotionProgramBlockCapture& capture,
    const NCProgramCommitSnapshot& commit,bool committed,bool ledgerFound,const NCBlockLifecycleSnapshot& ledger,int pc,int line)
{
    auto& s=*m_g76Orbit;
    if (!s.active || s.dispatch!=dispatch) return;
    const auto& r=s.receipt;
    bool valid=committed && commit.IsValid() && ledgerFound && s.pc==pc && s.line==line &&
        commit.scope==NCProgramScope::MEMORY && commit.frameId==NC_PROGRAM_FRAME_ID_INVALID &&
        commit.sourcePC==pc && commit.cacheGeneration==s.cache && ledger.dispatchId==dispatch &&
        ledger.programCommitted && !ledger.ncDispatchFailed && !ledger.motionCaptureOverflow &&
        ledger.sourceLineNumber==line && FeedSameSource(commit,ledger.programTarget) && FeedSameSource(commit,ledger.programCommit) &&
        commit.sequence==ledger.programCommit.sequence && capture.count==r.count && !capture.overflow &&
        r.count>0U && r.count<=128U && ledger.motionSegmentCount==r.count &&
        IsCncFeedValueCurrentSameThread(s.feed) && FeedTranslationCurrent(CoordSys,r.translationGeneration) &&
        m_motion.MatchesNCTranslation(s.mixed ? m_edm171Air->plan.input.translation : s.native.translation) && r.ownerLease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) && r.tail.IsCommitted() && r.tail.captureBound;
    valid=valid && r.count==(s.mixed ? m_edm171Air->plan.count : s.native.count);
    for (unsigned k=0U;k<r.count && k<128U && valid;++k)
    {
        const auto& sub=capture.submissions[k]; const auto& seg=ledger.motionSegments[k];
        valid=sub.producerAccepted && sub.immediateRejectReason==MotionRejectReason::NONE &&
            sub.commandPathMode==MotionCommandPathMode::EXACT_STOP && sub.translationGeneration==r.translationGeneration &&
            seg.producerAccepted && seg.immediateRejectReason==MotionRejectReason::NONE &&
            FeedFullIdentity(sub.identity,r.identities[k]) && FeedFullIdentity(seg.identity,r.identities[k]) &&
            r.identities[k].epoch==m_motion.GetCurrentExecutionEpoch() &&
            r.identities[k].sourceBlockId==static_cast<MotionSourceBlockId>(pc) && r.identities[k].source==MotionCommandSource::NC_MEMORY;
    }
    if (valid)
    {
        const auto& tail=capture.submissions[r.count-1U].queueTailReceipt;
        valid=tail.IsCommitted() && tail.captureBound && tail.transactionSequence==r.tail.transactionSequence &&
            FeedFullIdentity(tail.identity,r.tail.identity) && tail.ownerLease.Matches(r.ownerLease);
    }
    if (!valid)
    { RejectG76OrbitSameThread("AGGREGATE_COMMIT_BINDING",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
    s.commit=commit.sequence; s.bound=true;
    if (m_edm94.gapHistoryArm==292U && m_edm94.latched)
    {
        m_edm173->parentCommitted=true;
        RtPrintf("[EDM173] event=PARENT_BOUND session=%llu dispatch=%llu commit=%llu children=%u discharge=0\n",
            static_cast<unsigned long long>(m_edm94.session),static_cast<unsigned long long>(dispatch),
            static_cast<unsigned long long>(commit.sequence),r.count);
        return;
    }
    CommitCncModalFeedSameThread(s.feed,dispatch,commit.sequence,pc,line);
    m_g76Modal.lease=r.ownerLease; m_g76Modal.run=s.run; m_g76Modal.cache=s.cache;
    m_g76Modal.commit=commit.sequence; m_g76Modal.mode=s.mode; m_g76Modal.valid=true;
    RtPrintf("[EDM170] event=BOUND dispatch=%llu commit=%llu count=%u B=%u inheritedB=%u discharge=0\n",
        static_cast<unsigned long long>(dispatch),static_cast<unsigned long long>(commit.sequence),r.count,m_g76Modal.mode,s.inheritedB?1U:0U);
}

NC_PATH_FEED_NOINLINE
void NCManager::ObserveG76OrbitFeedbackSameThread(const MotionFeedbackEvent& event,bool ledgerAccepted)
{
    ObserveEDM171NominalFeedbackSameThread(event,ledgerAccepted);
    auto& s=*m_g76Orbit;
    if (!s.active || !s.bound) return;
    for (unsigned k=0U;k<s.receipt.count && k<128U;++k)
    {
        const auto& id=s.receipt.identities[k];
        if (!id.Matches(event.identity)) continue;
        bool valid=FeedFullIdentity(id,event.identity) && ledgerAccepted && event.sequence!=0ULL &&
            event.sequence>s.lastFeedback[k] && event.owner==s.receipt.ownerLease.owner &&
            event.ownerGeneration==s.receipt.ownerLease.generation && event.errorCode==0U &&
            event.rejectReason==MotionRejectReason::NONE;
        if (valid)
        {
            switch(event.type)
            {
            case MotionFeedbackType::ACCEPTED: valid=!s.accepted[k]; s.accepted[k]=true; break;
            case MotionFeedbackType::STARTED: valid=s.accepted[k] && !s.started[k] && (k==0U || s.done[k-1U]); s.started[k]=true; break;
            case MotionFeedbackType::COMPLETED: valid=s.accepted[k] && s.started[k] && !s.done[k] && (k==0U || s.done[k-1U]); s.done[k]=true; break;
            case MotionFeedbackType::PROGRESS:
            case MotionFeedbackType::HELD:
            case MotionFeedbackType::RESUMED: valid=s.accepted[k] && !s.done[k]; break;
            default: valid=false; break;
            }
        }
        if (!valid)
        { RejectG76OrbitSameThread("CHILD_FEEDBACK_IDENTITY_OR_LIFECYCLE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
        s.lastFeedback[k]=event.sequence;
        if (event.type==MotionFeedbackType::COMPLETED && m_edm94.gapHistoryArm!=292U)
        {
            auto& a=*m_edm171Air;
            if (s.mixed)
            {
                const auto& leg=a.plan.legs[k];
                if (leg.phase==NCG76Mixed::Phase::Nominal)
                    AppendEDM171HistorySameThread(leg.geometry,id,s.dispatch,s.commit,s.pc,s.line,leg.nominalOrdinal);
            }
            else if (k<s.program.count && (s.program.points[k].phase==NCG76::Phase::Grow || s.program.points[k].phase==NCG76::Phase::FinishCenter))
            {
                const auto& from=k ? s.native.pointsMCS[k-1U] : s.native.startMCS;
                const auto& fromPulse=k ? s.receipt.legEndPulse[k-1U] : s.receipt.startPulse;
                if (!BuildEDM171LineSameThread(from,s.native.pointsMCS[k],fromPulse,s.receipt.legEndPulse[k],s.native.feedMmMin[k],a.geometry))
                { RejectG76OrbitSameThread("G76_NOMINAL_CANONICAL_GEOMETRY",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
                AppendEDM171HistorySameThread(a.geometry,id,s.dispatch,s.commit,s.pc,s.line,s.program.points[k].phase==NCG76::Phase::FinishCenter ? 25U : s.program.points[k].growIndex);
            }
        }
        return;
    }
}

NC_PATH_FEED_NOINLINE
bool NCManager::CompleteG76OrbitSameThread()
{
    auto& s=*m_g76Orbit;
    if (!s.active) return s.completed;
    if (IsFeedHoldActive() || m_state==NCState::HOLD) return false;
    if (!s.bound || s.receipt.count==0U || s.receipt.count>128U || m_state!=NCState::RUN || m_mode!=NCOperationMode::MEMORY || Close_System_Com_flag ||
        AlarmManager::GetInstance().HasAlarm() || s.run!=m_pathCoreLiveBookkeeping.currentRunToken ||
        s.cache!=GetBaseProgramCache().GetGeneration() || !s.receipt.ownerLease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) || !IsG76OrbitFrameCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread() || !m_motion.MatchesNCTranslation(s.mixed ? m_edm171Air->plan.input.translation : s.native.translation) ||
        s.receipt.identities[0U].epoch!=m_motion.GetCurrentExecutionEpoch() ||
        m_motion.GetMotionFeedbackOverflowCount()!=0ULL || m_motion.GetMotionFeedbackProducerNoticeOverflowCount()!=0ULL ||
        m_motionFeedbackSequenceGapCount!=0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    { RejectG76OrbitSameThread("COMPLETION_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
    NCBlockLifecycleSnapshot& ledger=m_edm171Air->ledger;
    if (!m_blockLifecycleLedger.TryGetSnapshot(s.dispatch,ledger) || ledger.motionSegmentCount!=s.receipt.count ||
        !ledger.programCommitted || ledger.programCommit.sequence!=s.commit || ledger.motionFailedCount!=0U || ledger.terminalFailure)
    { RejectG76OrbitSameThread("COMPLETION_LEDGER",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
    bool done=true;
    for (unsigned k=0U;k<s.receipt.count;++k)
    {
        if (!FeedFullIdentity(ledger.motionSegments[k].identity,s.receipt.identities[k]))
        { RejectG76OrbitSameThread("COMPLETION_CHILD_IDENTITY",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
        done=done && s.done[k] && ledger.motionSegments[k].completed && ledger.motionSegments[k].terminal;
    }
    NCBlockMotionBoundarySnapshot boundary{};
    if (!m_blockLifecycleLedger.GetMotionBoundarySnapshot(s.dispatch,boundary))
    { RejectG76OrbitSameThread("COMPLETION_BOUNDARY",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
    if (!done || boundary.state!=NCBlockMotionBoundaryState::SUCCEEDED || boundary.motionCompletedCount!=s.receipt.count ||
        boundary.motionTerminalCount!=s.receipt.count || boundary.motionFailedCount!=0U ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupNCDrained() || !m_motion.IsGroupDone()) return false;
    for (unsigned a=0U;a<8U;++a)
        if (FeedDoubleBits(CoordSys.commandedMCS[a])!=FeedDoubleBits(s.receipt.endMCS[a]))
        { RejectG76OrbitSameThread("FINAL_ACCEPTED_TAIL",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return false; }
    if (m_edm94.gapHistoryArm==292U && !IsEDM173RuntimeCompleteSameThread()) return false;
    ServiceG76OrbitSameThread();
    s.active=false; s.completed=true;
    if (m_edm94.gapHistoryArm==292U)
    {
        RtPrintf("[EDM173] event=PARENT_COMPLETE dispatch=%llu commit=%llu completed=25 children=25 E5completed=1 evidence=ALL_CHILDREN_GROUP_DRAIN_AND_SHARED_E5 discharge=0\n",
            static_cast<unsigned long long>(s.dispatch),static_cast<unsigned long long>(s.commit));
        return true;
    }
    RtPrintf("[EDM170] event=COMPLETE dispatch=%llu commit=%llu completed=%u count=%u finalX=%.9f finalY=%.9f finalZ=%.9f evidence=ALL_CHILDREN_AND_GROUP_DRAIN discharge=0\n",
        static_cast<unsigned long long>(s.dispatch),static_cast<unsigned long long>(s.commit),ledger.motionCompletedCount,s.receipt.count,
        s.receipt.endMCS[0],s.receipt.endMCS[1],s.receipt.endMCS[2]);
    return true;
}

NC_PATH_FEED_NOINLINE
void NCManager::ServiceG76OrbitSameThread()
{
    if (m_g76PowerInhibited)
    {
        ForceEDMPLCOutputsOffSameThread("G76_AIR_RUN_LATCH");
        if (!m_g76Orbit->active && m_motion.GetMotionOwnerLease().owner==MotionOwner::NONE &&
            m_motion.IsGroupNCDrained() && m_motion.IsGroupDone() && !m_motion.HasPendingSafetyOrRecoveryRequests() &&
            !(m_edm94.latched && m_edm94.gapHistoryArm==292U) &&
            m_resetContinuationPhase==ResetContinuationPhase::IDLE) m_g76PowerInhibited=false;
    }
    auto& s=*m_g76Orbit;
    if (!s.active || !s.bound) return;
    NCBlockLifecycleSnapshot& ledger=m_edm171Air->ledger;
    if (!m_blockLifecycleLedger.TryGetSnapshot(s.dispatch,ledger) || ledger.motionSegmentCount!=s.receipt.count) return;
    while (s.loggedChildren<s.receipt.count)
    {
        const unsigned k=s.loggedChildren;
        const auto& seg=ledger.motionSegments[k];
        if (!seg.completed || !seg.terminal || !s.done[k] || !FeedFullIdentity(seg.identity,s.receipt.identities[k])) break;
        if (s.mixed)
        {
            const auto& leg=m_edm171Air->plan.legs[k];
            RtPrintf("[EDM171] event=CHILD_COMPLETED phase=%s pc=%d segment=%u count=%u sourceIndex=%u nominal=%u u0=%.9f u1=%.9f evidence=IDENTITY_MATCHED_FEEDBACK discharge=0\n",
                leg.phase==NCG76Mixed::Phase::Nominal?"NOMINAL":leg.phase==NCG76Mixed::Phase::Retreat?"RETREAT":"RETURN",s.pc,k+1U,s.receipt.count,
                leg.originalSourceIndex,leg.nominalOrdinal,leg.startU,leg.endU);
            ++s.loggedChildren; continue;
        }
        if (m_edm94.gapHistoryArm==292U)
        {
            RtPrintf("[EDM173] event=CHILD_COMPLETED dispatch=%llu child=%u children=25 evidence=IDENTITY_MATCHED_LEDGER discharge=0\n",
                static_cast<unsigned long long>(s.dispatch),k+1U);
            ++s.loggedChildren; continue;
        }
        const auto& point=s.program.points[k];
        const char* phase=point.phase==NCG76::Phase::Grow ? "GROW" : point.phase==NCG76::Phase::ToCenter ? "TO_CENTER" :
            point.phase==NCG76::Phase::Lift ? "LIFT" : point.phase==NCG76::Phase::ReturnCenter ? "RETURN_CENTER" :
            point.phase==NCG76::Phase::Resume ? "RESUME" : "FINISH_CENTER";
        RtPrintf("[EDM170] event=CHILD_COMPLETED phase=%s dispatch=%llu segment=%u count=%u grow=%u centerX=%.6f centerY=%.6f centerZ=%.6f targetX=%.6f targetY=%.6f targetZ=%.6f evidence=IDENTITY_MATCHED_FEEDBACK discharge=0\n",
            phase,static_cast<unsigned long long>(s.dispatch),k+1U,s.receipt.count,point.growIndex,
            point.center[0],point.center[1],point.center[2],point.position[0],point.position[1],point.position[2]);
        ++s.loggedChildren;
    }
}

NC_PATH_FEED_NOINLINE
void NCManager::RejectG76OrbitSameThread(const char* reason,int alarm)
{
    if (m_edm94.latched && m_edm94.gapHistoryArm==292U) CancelEDM94SameThread(reason);
    const bool accepted=m_g76Orbit->receipt.commandAccepted;
    const int line=m_g76Orbit->line;
    RtPrintf("[EDM170] event=REJECT reason=%s pc=%d accepted=%u discharge=0\n",reason,GetActiveDispatchPC(),accepted?1U:0U);
    m_g76Orbit->active=false; m_g76Orbit->bound=false; m_g76Orbit->completed=false;
    if (m_g76PowerInhibited) ForceEDMPLCOutputsOffSameThread("G76_AIR_RUN_REJECT");
    AlarmManager::GetInstance().Trigger(alarm,line);
    ChangeState(NCState::ALARM);
    if (accepted) m_motion.RequestEmergencyStopAllAxes();
}

NC_PATH_FEED_NOINLINE
void NCManager::CaptureG76OrbitConfirmationSameThread(int pc,NCBlockDispatchId dispatch) noexcept
{
    // A committed confirmation authorizes this one immutable main-program run.
    // Subsequent M00/HOLD stops do not consume or widen that run/cache grant.
    if (m_g76Confirmation.confirmed && !m_g76Confirmation.waiting &&
        m_g76Confirmation.run==m_pathCoreLiveBookkeeping.currentRunToken &&
        m_g76Confirmation.cache==GetBaseProgramCache().GetGeneration()) return;
    m_g76Confirmation=G76Confirmation{};
    const auto& transaction=m_gmBlockTransaction.snapshot;
    const auto* authored=GetBaseProgramCache().TryGetLine(pc);
    const auto* next=GetBaseProgramCache().TryGetLine(pc+1);
    NCBlock stop{},orbit{};
    if (!transaction.active || !transaction.fromMainProgram || transaction.sourcePC!=pc || transaction.dispatchId!=dispatch ||
        transaction.phase!=NCGMBlockTransactionPhase::READY_TO_FINALIZE || transaction.postAction!=NCGMBlockPostAction::PROGRAM_STOP_M00 ||
        m_state!=NCState::RUN || m_mode!=NCOperationMode::MEMORY || !m_macroStack.empty() || m_isSingleBlockEnabled ||
        m_edm94.latched || !dispatch || !m_pathCoreLiveBookkeeping.currentRunToken || AlarmManager::GetInstance().HasAlarm() ||
        !authored || !next || authored->parsedBlock.duplicateAddressMask || next->parsedBlock.duplicateAddressMask ||
        authored->parsedBlock.dependsOnMacroState || next->parsedBlock.dependsOnMacroState ||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(authored->parsedBlock,stop) ||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(next->parsedBlock,orbit) ||
        stop.gCount!=0 || stop.mCount!=1 || stop.mCode[0]!=0 || stop.isGoto || stop.isBlockSkip ||
        !IsEDM171AirProgramSameThread(pc+1)) return;
    for (char word='A';word<='Z';++word) if (stop.has(word) && word!='N') return;
    auto& c=m_g76Confirmation;
    c.run=m_pathCoreLiveBookkeeping.currentRunToken; c.cache=GetBaseProgramCache().GetGeneration();
    c.dispatch=dispatch; c.firstPC=pc+1; c.startRequests=m_startDiagnosticRequests; c.waiting=true;
    RtPrintf("[EDM170] event=OPERATOR_CONFIRMATION result=WAIT_M00_START firstPC=%d run=%llu cache=%llu discharge=0\n",
        c.firstPC,static_cast<unsigned long long>(c.run),static_cast<unsigned long long>(c.cache));
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitG76OrbitConfirmationResumeSameThread() noexcept
{
    auto& c=m_g76Confirmation;
    if (!c.waiting) return;
    if (m_state!=NCState::RUN || m_mode!=NCOperationMode::MEMORY || !m_macroStack.empty() || m_isSingleBlockEnabled ||
        c.run!=m_pathCoreLiveBookkeeping.currentRunToken || c.cache!=GetBaseProgramCache().GetGeneration() ||
        GetBasePC()!=c.firstPC || m_startDiagnosticRequests<=c.startRequests || m_waitCallback!=WaitForCycleStartCallback ||
        AlarmManager::GetInstance().HasAlarm() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    { c=G76Confirmation{}; return; }
    const auto* next=GetBaseProgramCache().TryGetLine(c.firstPC);
    NCBlock orbit{};
    if (!next || next->parsedBlock.duplicateAddressMask || next->parsedBlock.dependsOnMacroState ||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(next->parsedBlock,orbit) ||
        !IsEDM171AirProgramSameThread(c.firstPC))
    { c=G76Confirmation{}; return; }
    const auto speeds=GlobalConfig::GetInstance().GetG76CenterReturnSpeeds();
    if (!EDM171Config::ValidCenterReturnSpeeds(speeds)) {c=G76Confirmation{};return;}
    auto& a=*m_edm171Air;
    a.history.Clear(); a.speeds=speeds; a.run=c.run; a.cache=c.cache; a.lease=m_programMotionLease;
    a.configured=true; a.selected=a.bound=false;
    CaptureEDM171AirE6SameThread();
    c.waiting=false; c.confirmed=true; c.startRequests=m_startDiagnosticRequests;
    RtPrintf("[EDM170] event=OPERATOR_CONFIRMATION result=START_CONFIRMED firstPC=%d run=%llu cache=%llu discharge=0\n",
        c.firstPC,static_cast<unsigned long long>(c.run),static_cast<unsigned long long>(c.cache));
}

NC_PATH_FEED_NOINLINE
void NCManager::ResetG76OrbitStateSameThread() noexcept
{
    auto& s=*m_g76Orbit;
    s.run=s.cache=s.dispatch=s.commit=0ULL; s.pc=-1; s.line=0; s.loggedChildren=0U;
    s.active=s.bound=s.completed=s.inheritedB=s.mixed=false; s.mode=0U; s.sourceG=76;
    s.accepted.fill(false); s.started.fill(false); s.done.fill(false); s.lastFeedback.fill(0ULL);
    s.receipt.Clear();
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsEDM171AirGrantCurrentSameThread() const noexcept
{
    const auto& c=m_g76Confirmation; const auto& a=*m_edm171Air;
    return c.confirmed && !c.waiting && c.run!=0ULL && c.run==m_pathCoreLiveBookkeeping.currentRunToken &&
        c.cache!=0ULL && c.cache==GetBaseProgramCache().GetGeneration() && a.configured && a.run==c.run && a.cache==c.cache &&
        a.lease.Matches(m_programMotionLease) && m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) &&
        !m_edm94.latched && m_mode==NCOperationMode::MEMORY && !Close_System_Com_flag && m_externalReadyInterlock &&
        m_macroStack.empty() && !m_isG66Active && !m_isSingleBlockEnabled && !AlarmManager::GetInstance().HasAlarm();
}

// Capture only at the already validated fresh M00 START. This is an immutable
// AIR-run receipt, not a rebinding of the global recipe's motion epoch authority.
NC_PATH_FEED_NOINLINE
void NCManager::CaptureEDM171AirE6SameThread() noexcept
{
    auto& a=*m_edm171Air;
    a.e6Ready=false; a.e6Catalog=nullptr; a.e6Epoch=MOTION_EXECUTION_EPOCH_INVALID;
    a.e6=EDMRecipe::ParameterSnapshot{}; a.e6Recipe=EDMRecipe::Summary{};
    const auto* catalog=m_recipe.GetCatalog(); bool definition=false;
    if (catalog && catalog->schemaVersion==2U)
        for (const auto& field:catalog->fields)
            if (field.id==6U) definition=field.enabled && field.unitConfirmed &&
                std::strcmp(field.semanticKey,"JumpHeight")==0 && std::strcmp(field.unit,"mm")==0;
    EDMRecipe::ParameterSnapshot height{};
    if (definition && ReadEDMRecipeFieldSameThread(6U,height) && height.configured && height.fieldId==6U &&
        height.actualValue>0LL && height.actualValue<=2000000LL)
    {
        a.e6=height; a.e6Recipe=m_recipe.Current(); a.e6Catalog=catalog;
        a.e6Epoch=m_motion.GetCurrentExecutionEpoch(); a.e6Ready=true;
    }
    RtPrintf("[EDM171_FIX2] event=E6_SNAPSHOT result=%s run=%llu cache=%llu epoch=%llu owner=%u/%llu COND=%u E=%u generation=%llu E6microMm=%lld discharge=0\n",
        a.e6Ready ? "FROZEN" : "UNAVAILABLE",static_cast<unsigned long long>(a.run),
        static_cast<unsigned long long>(a.cache),static_cast<unsigned long long>(a.e6Epoch),
        static_cast<unsigned>(a.lease.owner),static_cast<unsigned long long>(a.lease.generation),
        static_cast<unsigned>(a.e6Recipe.tableId),static_cast<unsigned>(a.e6Recipe.eCode),
        static_cast<unsigned long long>(a.e6Recipe.generation),static_cast<long long>(a.e6.actualValue));
}

NC_PATH_FEED_NOINLINE
bool NCManager::ReadEDM171AirE6SameThread(EDMRecipe::ParameterSnapshot& output) const noexcept
{
    output=EDMRecipe::ParameterSnapshot{};
    const auto& a=*m_edm171Air;
    if (!IsEDM171AirGrantCurrentSameThread() || m_state!=NCState::RUN || m_recipeRecoveryDisplayOnly ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !a.e6Ready || !a.e6Catalog ||
        a.e6Epoch==MOTION_EXECUTION_EPOCH_INVALID || !a.e6.configured || a.e6.fieldId!=6U ||
        a.e6.actualValue<=0LL || a.e6.actualValue>2000000LL) return false;
    const auto* catalog=m_recipe.GetCatalog(); bool definition=false;
    if (catalog!=a.e6Catalog || !catalog || catalog->schemaVersion!=2U) return false;
    for (const auto& field:catalog->fields)
        if (field.id==6U) definition=field.enabled && field.unitConfirmed &&
            std::strcmp(field.semanticKey,"JumpHeight")==0 && std::strcmp(field.unit,"mm")==0;
    const auto current=m_recipe.Current(); const auto& frozen=a.e6Recipe;
    if (!definition || !current.catalogReady || !current.tableSelected || !current.rowSelected ||
        !frozen.catalogReady || !frozen.tableSelected || !frozen.rowSelected || !frozen.generation ||
        current.catalogRevision!=frozen.catalogRevision || current.tableId!=frozen.tableId ||
        current.tableRevision!=frozen.tableRevision || current.profileId!=frozen.profileId ||
        current.definitionRevision!=frozen.definitionRevision || current.eCode!=frozen.eCode ||
        current.fieldCount!=frozen.fieldCount || current.configuredCount!=frozen.configuredCount ||
        current.modifiedCount!=frozen.modifiedCount || current.generation!=frozen.generation) return false;
    // Read the logical value only to detect edits. The original authorized value
    // is returned; no global run/epoch/lease receipt is created or refreshed here.
    EDMRecipe::ParameterSnapshot live{}; const auto& value=a.e6;
    if (!m_recipe.ReadField(6U,live) || live.fieldId!=value.fieldId || live.configured!=value.configured ||
        live.rowKind!=value.rowKind || live.rowStageId!=value.rowStageId || live.rowBaseValue!=value.rowBaseValue ||
        live.stageId!=value.stageId || live.baseValue!=value.baseValue || live.actualValue!=value.actualValue ||
        live.overridePresent!=value.overridePresent || live.modified!=value.modified) return false;
    output=value;
    return true;
}

NC_PATH_FEED_NOINLINE
bool NCManager::NormalizeEDM171AirBlockSameThread(const NCBlock& input,NCBlock& output,
    int pc,int line,NCBlockDispatchId dispatch)
{
    output=input;
    auto& a=*m_edm171Air; a.selected=false; a.bound=false;
    // The existing live EDM/G180 modal-B family owns its unchanged decoder.
    if (m_edm94.latched) return true;
    const bool motion=NCGCodeSemantics::Contains(input,1) || NCGCodeSemantics::Contains(input,2) ||
        NCGCodeSemantics::Contains(input,3) || NCGCodeSemantics::Contains(input,76);
    if (!motion || (!input.has('B') && !IsEDM171AirGrantCurrentSameThread())) return true;
    const auto* authored=GetBaseProgramCache().TryGetLine(pc);
    bool valid=IsEDM171AirGrantCurrentSameThread() && m_state==NCState::RUN && !m_g76Orbit->active &&
        authored && authored->sourceLineNumber==line && !authored->parsedBlock.duplicateAddressMask &&
        !authored->parsedBlock.dependsOnMacroState && authored->parsedBlock.controlType==NCParsedControlType::NONE &&
        authored->parsedBlock.error==NCParseError::NONE &&
        NCPreparedBlockQueueShadow::TryBuildLiteralBlock(authored->parsedBlock,a.literal) &&
        input.gCount==1 && input.hasG && input.gCodes[0]==input.gCode && input.mCount==0 &&
        a.literal.gCount==input.gCount && a.literal.gCode==input.gCode && a.literal.gCodes[0]==input.gCodes[0] &&
        a.literal.hasG==input.hasG && a.literal.mCount==input.mCount && !input.isBlockSkip && !input.isGoto &&
        !input.has('Q') && (input.gCode==76 || !input.has('P'));
    for (char word='A';word<='Z' && valid;++word)
        valid=input.has(word)==a.literal.has(word) && (!input.has(word) ||
            (std::isfinite(input.val(word)) && FeedDoubleBits(input.val(word))==FeedDoubleBits(a.literal.val(word))));
    unsigned mode=0U;
    if (valid && input.has('B'))
    {
        const double b=input.val('B');
        valid=b==0.0 || b==2.0 || (input.gCode==76 && (b==3.0 || b==4.0));
        if (valid) mode=static_cast<unsigned>(b);
    }
    else if (valid && m_g76Modal.valid)
    {
        valid=m_g76Modal.run==a.run && m_g76Modal.cache==a.cache && m_g76Modal.commit!=0ULL &&
            m_g76Modal.lease.Matches(a.lease);
        mode=m_g76Modal.mode;
    }
    if (input.gCode!=76) { output.hasParam['B'-'A']=false; output.param['B'-'A']=0.0; }
    if (valid)
        valid=input.gCode==76 ? IsG76OrbitBlockShapeValid(output,true,CoordSys.isInchMode?20:21) :
            input.gCode==1 ? IsPathCoreFeedBlockShapeValid(output,true,CoordSys.isInchMode?20:21,false,17) :
            IsPathCoreArcBlockShapeValid(output,true,CoordSys.isInchMode?20:21,false,17);
    if (!valid)
    { RejectG76OrbitSameThread("AIR_LITERAL_OR_MODAL_SCOPE",AlarmManager::G_Code_Invalid_parameter); return false; }
    a.selected=true; a.mode=mode; a.inherited=!input.has('B'); a.dispatch=dispatch; a.pc=pc; a.lineNumber=line;
    a.normalized=output;
    if (input.gCode!=76)
    {
        ResetG76OrbitStateSameThread();
        auto& source=*m_g76Orbit; source.pc=pc; source.line=line; source.mode=mode; source.sourceG=input.gCode;
        if (!CaptureCncFeedValueSameThread(output,source.feed) || source.feed.feedMMMin>30.0 ||
            !PrepareG76OrbitTranslationSameThread(output) || !BuildEDM171NominalSameThread(output,false))
        { RejectG76OrbitSameThread("AIR_FEED_OR_NATIVE_SOURCE",AlarmManager::G_Code_Invalid_parameter); return false; }
        const auto& geometry=a.input.nominal.geometry[0U];
        double displacement=0.0;
        for (unsigned axis=0U;axis<3U;++axis) displacement=std::hypot(displacement,geometry.endMCS[axis]-geometry.startMCS[axis]);
        if (!std::isfinite(displacement) || displacement>2.0 ||
            (geometry.kind==NCPathCoreRetainedKind::ARC && geometry.radiusMM>1.0))
        { RejectG76OrbitSameThread("AIR_SMALL_NATIVE_ENVELOPE",AlarmManager::G_Code_Invalid_parameter); return false; }
    }
    m_g76PowerInhibited=true; ForceEDMPLCOutputsOffSameThread("EDM171_AIR_MOTION");
    return true;
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitEDM171AirBlockSameThread(NCBlockDispatchId dispatch,const MotionProgramBlockCapture& capture,
    const NCProgramCommitSnapshot& commit,bool committed,bool ledgerFound,const NCBlockLifecycleSnapshot& ledger,int pc,int line)
{
    auto& a=*m_edm171Air;
    if (!a.selected || a.dispatch!=dispatch) return;
    bool valid=IsEDM171AirGrantCurrentSameThread() && committed && commit.IsValid() && ledgerFound &&
        commit.scope==NCProgramScope::MEMORY && commit.cacheGeneration==a.cache && commit.frameId==0ULL &&
        commit.sourcePC==pc && pc==a.pc && line==a.lineNumber && ledger.sourceLineNumber==line &&
        ledger.dispatchId==dispatch && ledger.programCommitted && !ledger.ncDispatchFailed && !ledger.motionCaptureOverflow &&
        FeedSameSource(commit,ledger.programCommit) && commit.sequence==ledger.programCommit.sequence &&
        capture.count>0U && capture.count<=128U && !capture.overflow && ledger.motionSegmentCount==capture.count;
    if (a.normalized.gCode==76 || a.mode==2U)
        valid=valid && m_g76Orbit->active && m_g76Orbit->bound && m_g76Orbit->dispatch==dispatch && m_g76Orbit->commit==commit.sequence;
    else
    {
        const bool arc=a.normalized.gCode==2 || a.normalized.gCode==3;
        const MotionExecutionIdentity& identity=arc ? m_pathArcMotion.receipt.identity : m_pathFeedMotion.receipt.identity;
        valid=valid && capture.count==1U && (arc ? (m_pathArc.bound && m_pathArc.dispatch==dispatch && m_pathArc.commit==commit.sequence) :
            (m_pathFeed.bound && m_pathFeed.dispatch==dispatch && m_pathFeed.commit==commit.sequence)) &&
            capture.submissions[0U].producerAccepted && capture.submissions[0U].immediateRejectReason==MotionRejectReason::NONE &&
            FeedFullIdentity(identity,capture.submissions[0U].identity) && FeedFullIdentity(identity,ledger.motionSegments[0U].identity);
    }
    if (!valid)
    { RejectG76OrbitSameThread("AIR_MODAL_COMMIT_BINDING",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
    m_g76Modal.run=a.run; m_g76Modal.cache=a.cache; m_g76Modal.lease=a.lease;
    m_g76Modal.commit=commit.sequence; m_g76Modal.mode=a.mode; m_g76Modal.valid=true; a.bound=true;
    RtPrintf("[EDM171] event=AIR_MODE_COMMITTED g=%d B=%u inherited=%u pc=%d dispatch=%llu commit=%llu discharge=0\n",
        a.normalized.gCode,a.mode,a.inherited?1U:0U,pc,static_cast<unsigned long long>(dispatch),static_cast<unsigned long long>(commit.sequence));
}

NC_PATH_FEED_NOINLINE
bool NCManager::BuildEDM171LineSameThread(const std::array<double,8U>& from,const std::array<double,8U>& to,
    const std::array<double,8U>& fromPulse,const std::array<double,8U>& toPulse,double feed,NCPathCoreRetainedGeometry& geometry)
{
    auto& a=*m_edm171Air; auto& in=a.lineInput;
    in.startMCS=from; in.endMCS=to; in.startPulse=fromPulse; in.endPulse=toPulse; in.axisMask=0U; in.feedMMMin=feed;
    for (unsigned axis=0U;axis<3U;++axis) if (from[axis]!=to[axis] || fromPulse[axis]!=toPulse[axis]) in.axisMask|=1U<<axis;
    for (unsigned axis=0U;axis<8U;++axis) in.maxVelocityPPS[axis]=m_motion.GetAxisContext(static_cast<int>(axis)).maxVel_PPS;
    return BuildNCPathCoreFeedLine(in,a.line)==NCPathCoreFeedLineCode::BUILT_LINE && BuildNCPathCoreRetainedLine(a.line,geometry);
}

NC_PATH_FEED_NOINLINE
bool NCManager::BuildEDM171NominalSameThread(const NCBlock& block,bool orbit)
{
    auto& a=*m_edm171Air; auto& n=a.input.nominal; auto& s=*m_g76Orbit;
    auto& h=a.input.history;
    h.count=0U; h.validAxisMask=0U;
    for (unsigned axis=0U;axis<8U;++axis)
    {
        const AxisContext& context=m_motion.GetAxisContext(static_cast<int>(axis));
        h.pulsePerUnit[axis]=context.isExist ? context.resolution_PPR/context.finalLead : 0.0;
        if (context.isExist) h.validAxisMask|=1U<<axis;
    }
    if (a.history.Count()!=0U && !a.history.Snapshot(h)) return false;
    NCG76Mixed::Point8 start{},pulse{};
    for (unsigned axis=0U;axis<8U;++axis)
    {
        start[axis]=CoordSys.commandedMCS[axis];
        pulse[axis]=m_motion.GetAxisContext(static_cast<int>(axis)).isExist ?
            m_motion.GetAxisContext(static_cast<int>(axis)).logicalCmdPos.Load() : 0.0;
    }
    if (orbit) return NCG76Mixed::BuildG76Nominal(s.native,pulse,h.pulsePerUnit,s.pc,s.line,n);
    n.count=0U; n.triggerAfter=1U; n.pc=s.pc; n.line=s.line;
    n.kind=block.gCode==1 ? NCG76Mixed::Kind::LINE : NCG76Mixed::Kind::ARC;
    double words[8]={},targetRaw[8]={}; bool selected[8]={};
    for (unsigned axis=0U;axis<3U;++axis)
        if (block.has(static_cast<char>('X'+axis)))
        { selected[axis]=true; words[axis]=NCTranslationLengthToMM(block.val(static_cast<char>('X'+axis)),CoordSys.isInchMode?20:21); }
    const bool arc=block.gCode!=1;
    const bool fullCircle=arc && !selected[0] && !selected[1];
    if (!CoordSys.CompleteFixedPlanarEndpoint(words,selected)) return false;
    CoordSys.Preview_WCS_to_MCS(words,selected,targetRaw);
    NCG76Mixed::Point8 target{},endPulse{};
    for (unsigned axis=0U;axis<8U;++axis)
    {
        target[axis]=targetRaw[axis];
        endPulse[axis]=target[axis]==start[axis] ? pulse[axis] : target[axis]*h.pulsePerUnit[axis];
    }
    if (!arc)
    {
        if (!BuildEDM171LineSameThread(start,target,pulse,endPulse,s.feed.feedMMMin,n.geometry[0U])) return false;
    }
    else
    {
        auto& in=a.arcInput;
        in.startMCS=start; in.endMCS=target; in.startPulse=pulse; in.endPulse=endPulse;
        in.feedMMMin=s.feed.feedMMMin; in.plane=17U; in.fullCircle=fullCircle;
        const auto& frame=CoordSys.GetTranslationSnapshot();
        in.direction=(block.gCode==2 ? -1 : 1)*(((frame.mirrorMask&1U)!=0U)!=((frame.mirrorMask&2U)!=0U) ? -1 : 1);
        double i=block.has('I') ? NCTranslationLengthToMM(block.val('I'),frame.unitsMode) : 0.0;
        double j=block.has('J') ? NCTranslationLengthToMM(block.val('J'),frame.unitsMode) : 0.0;
        if (!NCTranslationRotateArcVector(frame,i,j,in.centerOffsetMM[0],in.centerOffsetMM[1])) return false;
        if (block.has('R') && !TryResolveNCPathRadiusArcCenter(start[0],start[1],target[0],target[1],
            NCTranslationLengthToMM(block.val('R'),frame.unitsMode)*(frame.scalingMode==51 ? frame.scalingFactor : 1.0),
            in.direction,in.centerOffsetMM[0],in.centerOffsetMM[1])) return false;
        in.pulsePerMM={{h.pulsePerUnit[0],h.pulsePerUnit[1]}};
        in.maxVelocityPPS={{m_motion.GetAxisContext(0).maxVel_PPS,m_motion.GetAxisContext(1).maxVel_PPS}};
        float roundoff=0.0F;
        if (!TryGetNCTranslationArcRoundoffMM(frame,roundoff,fullCircle) ||
            BuildNCPathCoreFeedArc(in,a.arc,roundoff)!=NCPathCoreFeedArcCode::BUILT_ARC ||
            !BuildNCPathCoreRetainedArc(a.arc,n.geometry[0U])) return false;
    }
    n.count=1U; n.feedMmMin[0U]=s.feed.feedMMMin;
    return true;
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartEDM171MixedMotionSameThread(const NCBlock& block)
{
    auto& a=*m_edm171Air; auto& s=*m_g76Orbit;
    const bool orbit=block.gCode==76;
    bool sourceCurrent=block.hasG==a.normalized.hasG && block.gCode==a.normalized.gCode && block.gCount==a.normalized.gCount &&
        block.gCodes[0]==a.normalized.gCodes[0] && block.mCount==a.normalized.mCount;
    for (char word='A';word<='Z' && sourceCurrent;++word)
        sourceCurrent=block.has(word)==a.normalized.has(word) && (!block.has(word) ||
            FeedDoubleBits(block.val(word))==FeedDoubleBits(a.normalized.val(word)));
    if (!sourceCurrent || !IsEDM171AirGrantCurrentSameThread() || !m_pathFeed.armed ||
        m_pathFeed.run!=m_pathCoreLiveBookkeeping.currentRunToken || m_pathFeed.cache!=GetBaseProgramCache().GetGeneration() ||
        !a.selected || a.mode!=2U ||
        a.dispatch!=m_currentExecutingBlockDispatchId || m_state!=NCState::RUN || !IsG76OrbitFrameCurrentSameThread() ||
        !IsPathCoreLiveNativeConfigCurrentSameThread() || !IsPathCoreFeedConfigurationValid() ||
        !IsFixedTranslationTravelCurrentSameThread() || !m_motion.MatchesNCTranslation(CoordSys.GetTranslationSnapshot()) ||
        !m_motion.IsGroupNCDrained() || !m_motion.IsGroupDone() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        m_motion.GetCommandIngressSize()!=0U || m_motion.GetCommandReplaySize()!=0U ||
        m_motion.GetMotionFeedbackOverflowCount()!=0ULL || m_motion.GetMotionFeedbackProducerNoticeOverflowCount()!=0ULL ||
        m_motionFeedbackSequenceGapCount!=0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
    { RejectG76OrbitSameThread("B2_SCOPE_OR_STOPPED_ADMISSION",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    if (!orbit)
    {
        ResetG76OrbitStateSameThread();
        s.run=a.run; s.cache=a.cache; s.dispatch=a.dispatch; s.pc=a.pc; s.line=a.lineNumber;
        if (!CaptureCncFeedValueSameThread(block,s.feed) || s.feed.feedMMMin>30.0)
        { RejectG76OrbitSameThread("B2_FEED_PROVENANCE",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    }
    s.sourceG=block.gCode; s.mode=2U; s.inheritedB=a.inherited; s.mixed=true;
    EDMRecipe::ParameterSnapshot distance{};
    if (!ReadEDM171AirE6SameThread(distance))
    { RejectG76OrbitSameThread("B2_E6_SNAPSHOT",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    if (!BuildEDM171NominalSameThread(block,orbit))
    { RejectG76OrbitSameThread("B2_NOMINAL_SOURCE",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    auto& input=a.input;
    if (!orbit)
    {
        const auto& geometry=input.nominal.geometry[0U]; double displacement=0.0;
        for (unsigned axis=0U;axis<3U;++axis) displacement=std::hypot(displacement,geometry.endMCS[axis]-geometry.startMCS[axis]);
        if (!std::isfinite(displacement) || displacement>2.0 || (geometry.kind==NCPathCoreRetainedKind::ARC && geometry.radiusMM>1.0))
        { RejectG76OrbitSameThread("B2_SMALL_NATIVE_ENVELOPE",AlarmManager::G_Code_Invalid_parameter); return nullptr; }
    }
    input.scope.run=a.run; input.scope.cache=a.cache; input.scope.lease=a.lease;
    input.scope.translationGeneration=CoordSys.GetTranslationSnapshot().generation;
    input.translation=CoordSys.GetTranslationSnapshot();
    input.retreatMm=static_cast<double>(distance.actualValue)/static_cast<double>(EDMRecipe::Scale);
    input.outFeedMmMin=input.returnFeedMmMin=s.feed.feedMMMin;
    if (!NCG76Mixed::Build(input,a.plan))
    {
        RtPrintf("[EDM171] event=B2_PLAN_REJECT status=%s E6=%.6f available=%.6f history=%u pc=%d beforePublish=1\n",
            NCG76Mixed::StatusName(a.plan.status),input.retreatMm,a.plan.availableMm,input.history.count,s.pc);
        RejectG76OrbitSameThread("B2_CANONICAL_HISTORY_PLAN",AlarmManager::G_Code_Invalid_parameter); return nullptr;
    }
    m_g76PowerInhibited=true; ForceEDMPLCOutputsOffSameThread("EDM171_B2_BATCH");
    const bool accepted=m_motion.TryG76MixedMoveTransactionalTail(a.plan,CoordSys.commandedMCS,s.receipt);
    const auto& r=s.receipt;
    unsigned movedMask=0U;
    for (unsigned k=0U;k<a.plan.count;++k) movedMask|=a.plan.legs[k].geometry.axisMask;
    bool valid=accepted && r.valid && r.commandAccepted && r.captureBound && r.tailCommitted &&
        r.count==a.plan.count && r.count>0U && r.count<=128U && movedMask!=0U && (movedMask&~7U)==0U &&
        r.axisMask==movedMask && (r.validAxisMask&movedMask)==movedMask &&
        r.translationGeneration==input.scope.translationGeneration && r.ownerLease.Matches(a.lease) &&
        r.tail.axisMask==r.validAxisMask && r.tail.ownerLease.Matches(r.ownerLease) &&
        r.tail.IsCommitted() && r.tail.captureBound && FeedFullIdentity(r.tail.identity,r.identities[r.count-1U]);
    for (unsigned k=0U;k<r.count && k<128U && valid;++k)
    {
        const auto& leg=a.plan.legs[k];
        valid=r.identities[k].IsAssigned() && r.identities[k].epoch==m_motion.GetCurrentExecutionEpoch() &&
            r.identities[k].source==MotionCommandSource::NC_MEMORY && r.identities[k].sourceBlockId==static_cast<MotionSourceBlockId>(s.pc);
        for (unsigned axis=0U;axis<8U && valid;++axis)
        {
            double end=0.0,endPulse=0.0;
            valid=EvaluateNCPathCoreRetainedAxisCanonical(leg.geometry,axis,leg.endU,end) &&
                EvaluateNCPathCoreRetainedPulseCanonical(leg.geometry,axis,leg.endU,endPulse) &&
                FeedDoubleBits(end)==FeedDoubleBits(r.legEndMCS[k][axis]) && FeedDoubleBits(endPulse)==FeedDoubleBits(r.legEndPulse[k][axis]);
        }
        for (unsigned j=0U;j<k && valid;++j) valid=!r.identities[k].Matches(r.identities[j]);
    }
    for (unsigned axis=0U;axis<8U && valid;++axis)
        valid=FeedDoubleBits(CoordSys.commandedMCS[axis])==FeedDoubleBits(r.endMCS[axis]) &&
            FeedDoubleBits(r.endMCS[axis])==FeedDoubleBits(input.nominal.geometry[input.nominal.count-1U].endMCS[axis]);
    if (!valid)
    {
        RtPrintf("[EDM171] event=B2_NATIVE_REJECT gate=%s axis=%d leg=%d accepted=%u\n",
            MotionG76AdmissionGateName(r.rejectionGate),r.rejectionAxis,r.rejectionSegment,r.commandAccepted?1U:0U);
        RejectG76OrbitSameThread("B2_NATIVE_BATCH_RECEIPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return nullptr;
    }
    s.active=true;
    RtPrintf("[EDM171] event=B2_BEGIN g=%d B=2 inherited=%u trigger=%s pc=%d count=%u reverse=%u E6=%.6f available=%.6f discharge=0\n",
        block.gCode,s.inheritedB?1U:0U,orbit?"GROW12":"SOURCE_END",s.pc,r.count,a.plan.reverseCount,input.retreatMm,a.plan.availableMm);
    return [](NCManager* nc){return nc->CompleteG76OrbitSameThread();};
}

NC_PATH_FEED_NOINLINE
void NCManager::AppendEDM171HistorySameThread(const NCPathCoreRetainedGeometry& geometry,
    const MotionExecutionIdentity& identity,std::uint64_t dispatch,std::uint64_t commit,int pc,int line,unsigned child)
{
    auto& a=*m_edm171Air;
    if (!IsEDM171AirGrantCurrentSameThread() || !IsPathCoreLiveNativeConfigCurrentSameThread() ||
        !IsFixedTranslationTravelCurrentSameThread() || !identity.IsAssigned() || identity.source!=MotionCommandSource::NC_MEMORY ||
        identity.sourceBlockId!=static_cast<MotionSourceBlockId>(pc) || !dispatch || !commit || pc<0 || line<=0 || !child)
    { RejectG76OrbitSameThread("HISTORY_SOURCE_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
    NCG76Mixed::HistoryRow row{};
    row.geometry=geometry; row.identity=identity; row.lease=a.lease; row.run=a.run; row.cache=a.cache;
    row.dispatch=dispatch; row.commit=commit; row.translationGeneration=CoordSys.GetTranslationSnapshot().generation;
    row.pc=pc; row.line=line; row.childOrdinal=child;
    NCG76Mixed::Point8 ppu{}; unsigned mask=0U;
    for (unsigned axis=0U;axis<8U;++axis)
    {
        const auto& context=m_motion.GetAxisContext(static_cast<int>(axis));
        ppu[axis]=context.isExist ? context.resolution_PPR/context.finalLead : 0.0;
        if (context.isExist) mask|=1U<<axis;
    }
    if (!a.history.Append(row,mask,ppu))
    {
        RtPrintf("[EDM171] event=HISTORY_REJECT status=%s pc=%d child=%u\n",NCG76Mixed::StatusName(a.history.Fault()),pc,child);
        RejectG76OrbitSameThread("CANONICAL_HISTORY_APPEND",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return;
    }
    RtPrintf("[EDM171] event=NOMINAL_RETAINED pc=%d child=%u kind=%s history=%u dispatch=%llu commit=%llu discharge=0\n",
        pc,child,geometry.kind==NCPathCoreRetainedKind::ARC?"ARC":"LINE",a.history.Count(),
        static_cast<unsigned long long>(dispatch),static_cast<unsigned long long>(commit));
}

NC_PATH_FEED_NOINLINE
void NCManager::ObserveEDM171NominalFeedbackSameThread(const MotionFeedbackEvent& event,bool ledgerAccepted)
{
    auto& a=*m_edm171Air;
    if (!a.selected || !a.bound || a.mode==2U || a.normalized.gCode==76 || !IsEDM171AirGrantCurrentSameThread()) return;
    const bool arc=a.normalized.gCode==2 || a.normalized.gCode==3;
    const auto& identity=arc ? m_pathArcMotion.receipt.identity : m_pathFeedMotion.receipt.identity;
    if (!identity.Matches(event.identity)) return;
    if (!FeedFullIdentity(identity,event.identity) || !ledgerAccepted || event.owner!=a.lease.owner ||
        event.ownerGeneration!=a.lease.generation || event.errorCode!=0U || event.rejectReason!=MotionRejectReason::NONE)
    { RejectG76OrbitSameThread("NOMINAL_FEEDBACK_SOURCE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
    if (event.type!=MotionFeedbackType::COMPLETED) return;
    auto& ledger=a.ledger;
    if (!m_blockLifecycleLedger.TryGetSnapshot(a.dispatch,ledger) || ledger.motionSegmentCount!=1U ||
        !ledger.motionSegments[0U].completed || !ledger.motionSegments[0U].started || !ledger.motionSegments[0U].runtimeAccepted ||
        !FeedFullIdentity(identity,ledger.motionSegments[0U].identity) || ledger.motionFailedCount || !ledger.programCommitted ||
        !(arc ? BuildNCPathCoreRetainedArc(m_pathArcMotion.receipt.arc,a.geometry) :
            BuildNCPathCoreRetainedLine(m_pathFeedMotion.receipt.line,a.geometry)))
    { RejectG76OrbitSameThread("NOMINAL_COMPLETION_RECEIPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY); return; }
    AppendEDM171HistorySameThread(a.geometry,identity,a.dispatch,ledger.programCommit.sequence,a.pc,a.lineNumber,1U);
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsEDM171AirProgramSameThread(int firstPC) const noexcept
{
    bool intent=false;
    for (unsigned offset=0U;offset<96U;++offset)
    {
        const auto* row=GetBaseProgramCache().TryGetLine(firstPC+static_cast<int>(offset));
        if (!row) return false;
        const auto& parsed=row->parsedBlock;
        if (parsed.error!=NCParseError::NONE || parsed.isBlockSkip || parsed.dependsOnMacroState ||
            parsed.duplicateAddressMask || parsed.controlType!=NCParsedControlType::NONE) return false;
        if (parsed.isEmpty) continue;
        NCBlock b{};
        if (!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed,b)) return false;
        if (b.gCount==0 && b.mCount==1 && b.mCode[0]==30)
        {
            for (char word='A';word<='Z';++word) if (b.has(word) && word!='N') return false;
            return offset!=0U && intent;
        }
        if (b.gCount!=1 || !b.hasG || b.mCount!=0 || b.isGoto || b.has('Q')) return false;
        const bool orbit=b.gCode==76;
        intent=intent || orbit || (b.has('B') && b.val('B')==2.0);
        if (b.has('B'))
        {
            const double mode=b.val('B');
            if (!std::isfinite(mode) || !(mode==0.0 || mode==2.0 || (orbit && (mode==3.0 || mode==4.0)))) return false;
            if (!orbit) {b.hasParam['B'-'A']=false;b.param['B'-'A']=0.0;}
        }
        if (orbit)
        { if (!IsG76OrbitBlockShapeValid(b,true,CoordSys.isInchMode?20:21)) return false; }
        else if (b.gCode==1)
        { if (!IsPathCoreFeedBlockShapeValid(b,true,CoordSys.isInchMode?20:21,false,17)) return false; }
        else if (b.gCode==2 || b.gCode==3)
        { if (b.has('P') || !IsPathCoreArcBlockShapeValid(b,true,CoordSys.isInchMode?20:21,false,17)) return false; }
        else return false;
    }
    return false;
}

#undef NC_PATH_FEED_NOINLINE
