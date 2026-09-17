// Exact-stop cutter contour preparation. Nominal source geometry is kept
// separate from the Motion-owned physical tool-centre endpoint.
#include "NCManager.h"
#include "NCPathCoreCutterContour.h"
#include "NCPathCoreRadiusArc.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <windows.h>
#include <rtapi.h>

namespace
{
    std::uint64_t CutterDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }

    bool CutterSamePrimitive(const NCPathCoreCutterPrimitive& a,
        const NCPathCoreCutterPrimitive& b) noexcept
    {
        if (a.kind != b.kind || a.clockwise != b.clockwise) return false;
        for (unsigned i = 0U; i < 2U; ++i)
            if (CutterDoubleBits(a.start[i]) != CutterDoubleBits(b.start[i]) ||
                CutterDoubleBits(a.end[i]) != CutterDoubleBits(b.end[i]) ||
                CutterDoubleBits(a.centre[i]) != CutterDoubleBits(b.centre[i])) return false;
        return true;
    }

    bool CutterSameLiteral(const NCBlock& literal, const NCBlock& current) noexcept
    {
        if (literal.gCode != current.gCode || literal.gCount != current.gCount ||
            literal.mCount != current.mCount) return false;
        for (unsigned i = 0U; i < 26U; ++i)
            if (literal.hasParam[i] != current.hasParam[i] ||
                (literal.hasParam[i] && CutterDoubleBits(literal.val(static_cast<char>('A' + i))) !=
                    CutterDoubleBits(current.val(static_cast<char>('A' + i))))) return false;
        return true;
    }
}

bool NCManager::IsCutterContourBlockShapeValid(const NCBlock& block, int unitsMode) noexcept
{
    if (block.isBlockSkip || !block.has('X') || !block.has('Y') || block.has('Z') ||
        block.has('Q') || block.has('P')) return false;
    return IsPathCoreFeedBlockShapeValid(block, true, unitsMode, false) ||
        IsPathCoreArcBlockShapeValid(block, true, unitsMode, false);
}

bool NCManager::BuildCutterContourSameThread(const NCBlock& block, int sourcePC,
    std::uint64_t run, std::uint64_t cache, std::uint64_t dispatch,
    std::array<double, 8U>& endpoint, std::array<double, 2U>& nativeCenterOffset,
    int& direction)
{
    m_cutterLine.staged = false;
    const auto reject = [sourcePC](const char* reason) -> bool
    {
        RtPrintf("[CUTTER][REJECT] reason=%s pc=%d beforeSubmit=1\n", reason, sourcePC);
        return false;
    };
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    if (m_cncFeed.selected || !CoordSys.IsTranslationRunFrozen() ||
        !CoordSys.IsTranslationRunCurrent() || !IsNCTranslationSnapshotValid(source) ||
        source.cutterMode == 40 || !CoordSys.isAbsoluteMode || CoordSys.isPolarCoordinateActive ||
        CoordSys.activePlane != 17 || CoordSys.isCAxisOffsetRotationEnabled ||
        m_cutterLine.leadOutRequired || sourcePC < 0 || dispatch == 0ULL ||
        run != source.runToken || cache != GetBaseProgramCache().GetGeneration() ||
        !IsCutterContourBlockShapeValid(block, source.unitsMode)) return reject("SCOPE");

    const NCProgramCacheLine* current = GetBaseProgramCache().TryGetLine(sourcePC);
    NCBlock literal{};
    if (!current || current->parsedBlock.isBlockSkip ||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(current->parsedBlock, literal) ||
        !IsCutterContourBlockShapeValid(literal, source.unitsMode) ||
        !CutterSameLiteral(literal, block)) return reject("LITERAL_CONTOUR_REQUIRED");

    const bool entry = !m_cutterLine.valid;
    if (entry && block.gCode != 1) return reject("LINE_ENTRY_REQUIRED");
    if (!entry && (m_cutterLine.run != run || m_cutterLine.cache != cache ||
        m_cutterLine.generation != source.generation || m_cutterLine.commit == 0ULL ||
        m_cutterLine.dispatch >= dispatch || m_cutterLine.terminal ||
        m_cutterLine.nextPC != sourcePC)) return reject("NOMINAL_SOURCE");
    if (!entry)
        for (unsigned i = 0U; i < 2U; ++i)
            if (CutterDoubleBits(CoordSys.commandedMCS[i]) !=
                CutterDoubleBits(m_cutterLine.physicalTail[i])) return reject("PHYSICAL_CONTINUITY");

    const bool mirrored = ((source.mirrorMask & 1U) != 0U) != ((source.mirrorMask & 2U) != 0U);
    const auto primitive = [this, &source, mirrored](const NCBlock& row, const double* start,
        NCPathCoreCutterPrimitive& result) -> bool
    {
        result = NCPathCoreCutterPrimitive{};
        result.kind = row.gCode == 1 ? NCPathCoreCutterPrimitiveKind::LINE : NCPathCoreCutterPrimitiveKind::ARC;
        std::array<double, 8U> wcs{}, mcs{};
        std::array<bool, 8U> programmed{};
        programmed[0] = programmed[1] = true;
        wcs[0] = NCTranslationLengthToMM(row.val('X'), source.unitsMode);
        wcs[1] = NCTranslationLengthToMM(row.val('Y'), source.unitsMode);
        CoordSys.Preview_WCS_to_MCS(wcs.data(), programmed.data(), mcs.data());
        for (unsigned i = 0U; i < 2U; ++i)
        {
            result.start[i] = start[i];
            result.end[i] = mcs[i];
            if (!std::isfinite(start[i]) || !std::isfinite(mcs[i])) return false;
        }
        if (row.gCode != 1)
        {
            result.clockwise = (row.gCode == 2) != mirrored;
            double x = 0.0, y = 0.0;
            if (row.has('R'))
            {
                // Both the current and lookahead arc use NOMINAL endpoints.
                // The physical tool-centre tail already includes cutter offset
                // and must never choose the R circle or its minor/major branch.
                const double signedRadiusMM = NCTranslationLengthToMM(row.val('R'), source.unitsMode) *
                    (source.scalingMode == 51 ? source.scalingFactor : 1.0);
                if (!TryResolveNCPathRadiusArcCenter(start[0], start[1], result.end[0], result.end[1],
                    signedRadiusMM, result.clockwise ? -1 : 1, x, y)) return false;
            }
            else
            {
                const double I = row.has('I') ? NCTranslationLengthToMM(row.val('I'), source.unitsMode) : 0.0;
                const double J = row.has('J') ? NCTranslationLengthToMM(row.val('J'), source.unitsMode) : 0.0;
                NCTranslationRotateXYVector(source, I, J, x, y);
            }
            result.centre[0] = start[0] + x;
            result.centre[1] = start[1] + y;
            if (!std::isfinite(result.centre[0]) || !std::isfinite(result.centre[1])) return false;
        }
        return true;
    };

    NCPathCoreCutterContourInput input{};
    input.entry = entry;
    input.signedRadius = (source.cutterMode == 41 ? 1.0 : -1.0) * source.cutterRadiusMM *
        (mirrored ? -1.0 : 1.0);
    const double* nominalStart = entry ? CoordSys.commandedMCS : m_cutterLine.nominal.data();
    if (!primitive(literal, nominalStart, input.current)) return reject("NONFINITE_NOMINAL");
    for (unsigned i = 0U; i < 2U; ++i)
    {
        if (CutterDoubleBits(endpoint[i]) != CutterDoubleBits(input.current.end[i]))
            return reject("NOMINAL_ENDPOINT");
        input.actualStart[i] = CoordSys.commandedMCS[i];
    }
    if (!entry && !CutterSamePrimitive(input.current, m_cutterLine.expectedPrimitive))
        return reject("NOMINAL_CONTINUITY");

    // Bounded lookahead reads literal immutable rows only. It never resolves a
    // future variable, macro, conditional, block skip or side-effecting word.
    NCBlock following{};
    int nextPC = -1;
    for (int look = 1; look <= 32; ++look)
    {
        if (sourcePC > (std::numeric_limits<int>::max)() - look) break;
        const int pc = sourcePC + look;
        const NCProgramCacheLine* row = GetBaseProgramCache().TryGetLine(pc);
        if (!row || row->parsedBlock.isBlockSkip) break;
        if (row->parsedBlock.isEmpty) continue;
        if (!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(row->parsedBlock, following)) break;
        nextPC = pc;
        break;
    }
    if (nextPC < 0) return reject("NEXT_LITERAL_REQUIRED");
    bool terminal = following.hasG && following.gCount == 1 && following.gCode == 40 &&
        following.gCodes[0] == 40 && following.mCount == 0;
    if (terminal)
        for (unsigned i = 0U; i < 26U; ++i)
            if (following.hasParam[i] && i != 'N' - 'A') terminal = false;
    input.hasNext = !terminal;
    if (!terminal && (!IsCutterContourBlockShapeValid(following, source.unitsMode) ||
        !primitive(following, input.current.end, input.next))) return reject("NEXT_XY_OR_G40_REQUIRED");

    NCPathCoreCutterContourOutput output{};
    BuildNCPathCoreCutterContour(input, output);
    if (!output.valid)
    {
        RtPrintf("[CUTTER][GEOMETRY_REJECT] code=%u g=%d entry=%u beforeSubmit=1\n",
            static_cast<unsigned>(output.reason), block.gCode, entry ? 1U : 0U);
        return reject("OFFSET_GEOMETRY");
    }

    // Only stage NC-owned values here. Physical/program tails commit after the
    // original producer, capture, source, owner and lifecycle gates succeed.
    m_cutterLine.stagedNominal = endpoint;
    m_cutterLine.stagedPrimitive = input.current;
    m_cutterLine.stagedNextPrimitive = input.next;
    m_cutterLine.stagedGeometry = output;
    m_cutterLine.stagedNext[0] = input.next.end[0];
    m_cutterLine.stagedNext[1] = input.next.end[1];
    m_cutterLine.stagedNextPC = nextPC;
    m_cutterLine.stagedTerminal = terminal;
    m_cutterLine.stagedRun = run;
    m_cutterLine.stagedCache = cache;
    m_cutterLine.stagedGeneration = source.generation;
    m_cutterLine.stagedDispatch = dispatch;
    m_cutterLine.stagedPC = sourcePC;
    for (unsigned i = 0U; i < 2U; ++i)
    {
        endpoint[i] = output.endpoint[i];
        if (block.gCode != 1) nativeCenterOffset[i] = output.centre[i] - input.actualStart[i];
    }
    if (block.gCode != 1) direction = input.current.clockwise ? -1 : 1;
    m_cutterLine.staged = true;
    return true;
}

bool NCManager::CommitCutterContourSameThread(std::uint64_t run, std::uint64_t cache,
    std::uint64_t dispatch, std::uint64_t commit, std::uint64_t generation,
    int sourcePC, const std::array<double, 8U>& physicalEnd) noexcept
{
    if (!m_cutterLine.staged || m_cncFeed.selected || commit == 0ULL ||
        m_cutterLine.stagedRun != run || m_cutterLine.stagedCache != cache ||
        m_cutterLine.stagedDispatch != dispatch || m_cutterLine.stagedGeneration != generation ||
        m_cutterLine.stagedPC != sourcePC || !m_cutterLine.stagedGeometry.valid) return false;
    for (unsigned i = 0U; i < 2U; ++i)
        if (CutterDoubleBits(physicalEnd[i]) != CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[i]))
            return false;
    m_cutterLine.nominal = m_cutterLine.stagedNominal;
    m_cutterLine.expectedNext = m_cutterLine.stagedNext;
    m_cutterLine.expectedPrimitive = m_cutterLine.stagedNextPrimitive;
    m_cutterLine.nextPC = m_cutterLine.stagedNextPC;
    m_cutterLine.terminal = m_cutterLine.stagedTerminal;
    m_cutterLine.run = run;
    m_cutterLine.cache = cache;
    m_cutterLine.generation = generation;
    m_cutterLine.commit = commit;
    m_cutterLine.dispatch = dispatch;
    m_cutterLine.physicalTail[0] = physicalEnd[0];
    m_cutterLine.physicalTail[1] = physicalEnd[1];
    m_cutterLine.valid = true;
    m_cutterLine.staged = false;
    return true;
}

void NCManager::LogCutterContourSameThread(std::uint64_t run, std::uint64_t dispatch,
    int sourcePC) const noexcept
{
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    RtPrintf("[CUTTER][SUBMITTED] run=%llu dispatch=%llu mode=%d D=%d radiusMMBits=%llu entry=%u terminal=%u pc=%d nominalXMMBits=%llu nominalYMMBits=%llu centreXMMBits=%llu centreYMMBits=%llu\n",
        static_cast<unsigned long long>(run), static_cast<unsigned long long>(dispatch),
        source.cutterMode, source.cutterD, static_cast<unsigned long long>(CutterDoubleBits(source.cutterRadiusMM)),
        m_cutterLine.valid ? 0U : 1U, m_cutterLine.stagedTerminal ? 1U : 0U, sourcePC,
        static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedNominal[0])),
        static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedNominal[1])),
        static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[0])),
        static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[1])));
    if (m_cutterLine.stagedPrimitive.kind == NCPathCoreCutterPrimitiveKind::ARC)
        RtPrintf("[CUTTER][ARC] run=%llu dispatch=%llu direction=%d circleXMMBits=%llu circleYMMBits=%llu pathRadiusMMBits=%llu sweepBits=%llu\n",
            static_cast<unsigned long long>(run), static_cast<unsigned long long>(dispatch),
            m_cutterLine.stagedPrimitive.clockwise ? -1 : 1,
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.centre[0])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.centre[1])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.radius)),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.sweepRadians)));
}
