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
        if (a.kind != b.kind || a.clockwise != b.clockwise || a.fullCircle != b.fullCircle) return false;
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

bool NCManager::IsCutterContourBlockShapeValid(const NCBlock& block, int unitsMode, int planeCode, bool polar) noexcept
{
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(planeCode, plane) || block.isBlockSkip ||
        block.has(static_cast<char>('X' + plane.normal)) || block.has('Q') || block.has('P')) return false;
    // BASE-PLANE-19: G90 polar lines and PARTIAL arcs may omit radius OR
    // angle. No omission grants a circle; full revolutions still require
    // the unchanged exact repeated, complete authored-pair proof below.
    const bool sparsePolarContour = polar && planeCode != 17 &&
        (block.gCode == 1 || block.gCode == 2 || block.gCode == 3);
    if (sparsePolarContour ? (!block.has(plane.uAddress) && !block.has(plane.vAddress)) :
        (!block.has(plane.uAddress) || !block.has(plane.vAddress))) return false;
    // Complete polar G01/IJK/R syntax retains its numerical path. An IJK circle
    // is opt-in only after exact repeated authored radius/angle and tangent
    // seams are proved below. R, aliases and rounded coincidence cannot
    // manufacture a circle. Sparse syntax is only a point-decoding permit.
    if (polar)
        return planeCode != 17 &&
            (IsPathCoreFeedBlockShapeValid(block, true, unitsMode, true, planeCode) ||
                IsPathCoreArcBlockShapeValid(block, true, unitsMode, true, planeCode, true, true));
    // BASE-PLANE-11: canonical literal contours; an IJK full revolution
    // needs G90 equal endpoints or G91 zero deltas, plus tangent-line seams.
    // The immutable
    // lookahead proves that classification before the preceding line moves.
    // Entry and G40 lead-out remain straight; G17 keeps its prior scope.
    return IsPathCoreFeedBlockShapeValid(block, true, unitsMode, false, planeCode) ||
        IsPathCoreArcBlockShapeValid(block, true, unitsMode, false, planeCode);
}

// This is an NC-only nominal source. No RT encoder, inverse-transform round
// trip or mutable future variable is allowed to replace the accepted tail.
bool NCManager::PreviewCutterIncrementalEndpointSameThread(const NCBlock& block,
    std::array<double, 8U>& endpoint) const noexcept
{
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    NCArcPlaneAxes plane{};
    if (CoordSys.isAbsoluteMode || source.distanceMode != 91 || source.rotationPlane == 17 ||
        !TryGetNCArcPlaneAxes(source.rotationPlane, plane) ||
        !IsNCTranslationSnapshotValid(source) || !CoordSys.IsTranslationRunFrozen() ||
        !CoordSys.IsTranslationRunCurrent() || CoordSys.activePlane != source.rotationPlane ||
        CoordSys.isPolarCoordinateActive || CoordSys.isCAxisOffsetRotationEnabled ||
        !IsCutterContourBlockShapeValid(block, source.unitsMode, source.rotationPlane, source.polarMode == 16)) return false;
    const bool leadOut = m_cutterLine.leadOutRequired;
    const bool continuing = m_cutterLine.valid || leadOut;
    if (leadOut ? (source.cutterMode != 40 || block.gCode != 1 || m_cutterLine.valid || !m_cutterLine.terminal) :
        (source.cutterMode == 40)) return false;
    if (continuing)
    {
        if (m_cutterLine.run != source.runToken || m_cutterLine.cache != GetBaseProgramCache().GetGeneration() ||
            m_cutterLine.commit == 0ULL || m_cutterLine.dispatch == 0ULL ||
            m_cutterLine.plane != source.rotationPlane || m_cutterLine.distanceMode != 91 ||
            m_cutterLine.polarMode != 15) return false;
        // G40 is the sole descriptor change permitted before a pending lead-out.
        // Its generation advances exactly once and it does not change the tail.
        if (leadOut ? (m_cutterLine.generation == (std::numeric_limits<std::uint64_t>::max)() ||
                source.generation != m_cutterLine.generation + 1ULL) :
            (source.generation != m_cutterLine.generation)) return false;
        for (unsigned axis = 0U; axis < 3U; ++axis)
            if (CutterDoubleBits(CoordSys.commandedMCS[axis]) !=
                CutterDoubleBits(m_cutterLine.physicalTail[axis])) return false;
    }
    else if (block.gCode != 1) return false;
    std::array<double, 8U> delta{}, candidate{};
    std::array<bool, 8U> selected{};
    selected[plane.u] = selected[plane.v] = true;
    delta[plane.u] = NCTranslationLengthToMM(block.val(plane.uAddress), source.unitsMode);
    delta[plane.v] = NCTranslationLengthToMM(block.val(plane.vAddress), source.unitsMode);
    const double* nominal = continuing ? m_cutterLine.nominal.data() : CoordSys.commandedMCS;
    if (!TryNCTranslationIncrementalTarget(source, nominal, delta.data(), selected.data(), candidate.data())) return false;
    // Untouched native axes are copied bit-exactly from the accepted physical tail.
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (axis != plane.u && axis != plane.v && CutterDoubleBits(candidate[axis]) !=
            CutterDoubleBits(CoordSys.commandedMCS[axis])) return false;
    endpoint = candidate;
    return true;
}

bool NCManager::PreviewCutterPolarEndpointSameThread(const NCBlock& block,
    std::array<double, 8U>& endpoint) const noexcept
{
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    NCArcPlaneAxes plane{};
    if (!CoordSys.isAbsoluteMode || !CoordSys.isPolarCoordinateActive ||
        source.distanceMode != 90 || source.polarMode != 16 || source.rotationPlane == 17 ||
        !TryGetNCArcPlaneAxes(source.rotationPlane, plane) ||
        !IsNCTranslationSnapshotValid(source) || !CoordSys.IsTranslationRunFrozen() ||
        !CoordSys.IsTranslationRunCurrent() || CoordSys.activePlane != source.rotationPlane ||
        CoordSys.isCAxisOffsetRotationEnabled ||
        !IsCutterContourBlockShapeValid(block, source.unitsMode, source.rotationPlane, true)) return false;
    const bool leadOut = m_cutterLine.leadOutRequired;
    const bool continuing = m_cutterLine.valid || leadOut;
    if (leadOut ? (source.cutterMode != 40 || block.gCode != 1 || m_cutterLine.valid || !m_cutterLine.terminal) :
        (source.cutterMode == 40)) return false;
    if (!continuing && block.gCode != 1) return false;
    if (continuing)
    {
        if (m_cutterLine.run != source.runToken || m_cutterLine.cache != GetBaseProgramCache().GetGeneration() ||
            m_cutterLine.commit == 0ULL || m_cutterLine.dispatch == 0ULL ||
            m_cutterLine.plane != source.rotationPlane || m_cutterLine.distanceMode != 90 ||
            m_cutterLine.polarMode != 16) return false;
        if (leadOut ? (m_cutterLine.generation == (std::numeric_limits<std::uint64_t>::max)() ||
                source.generation != m_cutterLine.generation + 1ULL) :
            (source.generation != m_cutterLine.generation)) return false;
        for (unsigned axis = 0U; axis < 3U; ++axis)
            if (CutterDoubleBits(CoordSys.commandedMCS[axis]) !=
                CutterDoubleBits(m_cutterLine.physicalTail[axis])) return false;
    }
    const std::uint32_t authoredMask = (block.has(plane.uAddress) ? (1U << plane.u) : 0U) |
        (block.has(plane.vAddress) ? (1U << plane.v) : 0U);
    const double radiusMM = block.has(plane.uAddress) ?
        NCTranslationLengthToMM(block.val(plane.uAddress), source.unitsMode) : 0.0;
    const double angleDeg = block.has(plane.vAddress) ? block.val(plane.vAddress) : 0.0;
    if (authoredMask != plane.mask)
        return TryNCTranslationCutterSparsePolarEndpoint(source,
            continuing ? m_cutterLine.nominal.data() : CoordSys.commandedMCS,
            CoordSys.commandedMCS, radiusMM, angleDeg, authoredMask, endpoint.data());
    return TryNCTranslationCutterPolarLineEndpoint(source, CoordSys.commandedMCS,
        radiusMM, angleDeg, endpoint.data());
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
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(source.rotationPlane, plane) || m_cncFeed.selected || !CoordSys.IsTranslationRunFrozen() ||
        !CoordSys.IsTranslationRunCurrent() || !IsNCTranslationSnapshotValid(source) ||
        source.cutterMode == 40 || !IsNCTranslationCutterNotationAllowed(source.rotationPlane, source.distanceMode, source.polarMode) ||
        CoordSys.isAbsoluteMode != (source.distanceMode == 90) ||
        CoordSys.isPolarCoordinateActive != (source.polarMode == 16) ||
        CoordSys.activePlane != source.rotationPlane || CoordSys.isCAxisOffsetRotationEnabled ||
        m_cutterLine.leadOutRequired || sourcePC < 0 || dispatch == 0ULL ||
        run != source.runToken || cache != GetBaseProgramCache().GetGeneration() ||
        !IsCutterContourBlockShapeValid(block, source.unitsMode, source.rotationPlane, source.polarMode == 16)) return reject("SCOPE");

    const NCProgramCacheLine* current = GetBaseProgramCache().TryGetLine(sourcePC);
    NCBlock literal{};
    if (!current || current->parsedBlock.isBlockSkip ||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(current->parsedBlock, literal) ||
        !IsCutterContourBlockShapeValid(literal, source.unitsMode, source.rotationPlane, source.polarMode == 16) ||
        !CutterSameLiteral(literal, block)) return reject("LITERAL_CONTOUR_REQUIRED");

    const bool entry = !m_cutterLine.valid;
    if (entry && block.gCode != 1) return reject("LINE_ENTRY_REQUIRED");
    if (!entry && (m_cutterLine.run != run || m_cutterLine.cache != cache ||
        m_cutterLine.generation != source.generation || m_cutterLine.commit == 0ULL ||
        m_cutterLine.dispatch >= dispatch || m_cutterLine.terminal ||
        m_cutterLine.nextPC != sourcePC || m_cutterLine.plane != source.rotationPlane ||
        m_cutterLine.distanceMode != source.distanceMode || m_cutterLine.polarMode != source.polarMode)) return reject("NOMINAL_SOURCE");
    if (!entry)
        for (unsigned i = 0U; i < (source.rotationPlane == 17 ? 2U : 3U); ++i)
            if (CutterDoubleBits(CoordSys.commandedMCS[i]) !=
                CutterDoubleBits(m_cutterLine.physicalTail[i])) return reject("PHYSICAL_CONTINUITY");
    // No authored normal-axis word and no tilted frame is allowed. Prove the
    // unselected normal as well, rather than just discarding a transformed Z/Y/X.
    if (source.rotationPlane != 17 &&
        CutterDoubleBits(endpoint[plane.normal]) != CutterDoubleBits(CoordSys.commandedMCS[plane.normal]))
        return reject("NORMAL_CONTINUITY");

    const bool mirrored = ((source.mirrorMask & (1U << plane.u)) != 0U) != ((source.mirrorMask & (1U << plane.v)) != 0U);
    const auto primitive = [this, &source, &plane, mirrored](const NCBlock& row, const double* start,
        bool fullCircle, NCPathCoreCutterPrimitive& result) -> bool
    {
        result = NCPathCoreCutterPrimitive{};
        result.kind = row.gCode == 1 ? NCPathCoreCutterPrimitiveKind::LINE : NCPathCoreCutterPrimitiveKind::ARC;
        if (fullCircle && (source.rotationPlane == 17 || row.gCode == 1 || row.has('R')))
            return false;
        result.fullCircle = fullCircle;
        std::array<double, 8U> wcs{}, mcs{};
        std::array<bool, 8U> programmed{};
        programmed[plane.u] = programmed[plane.v] = true;
        wcs[plane.u] = NCTranslationLengthToMM(row.val(plane.uAddress), source.unitsMode);
        wcs[plane.v] = source.polarMode == 16 ? row.val(plane.vAddress) :
            NCTranslationLengthToMM(row.val(plane.vAddress), source.unitsMode);
        if (source.polarMode == 16)
        {
            const std::uint32_t authoredMask = (row.has(plane.uAddress) ? (1U << plane.u) : 0U) |
                (row.has(plane.vAddress) ? (1U << plane.v) : 0U);
            if (authoredMask != plane.mask)
            {
                if (fullCircle) return false; // Missing words can never authorize a revolution.
                // Lookahead's start is the CURRENT nominal endpoint. It is
                // not the Motion physical tail nor the current line's start.
                std::array<double, 8U> nominal{};
                for (unsigned axis = 0U; axis < 8U; ++axis) nominal[axis] = CoordSys.commandedMCS[axis];
                nominal[plane.u] = start[0]; nominal[plane.v] = start[1];
                if (!TryNCTranslationCutterSparsePolarEndpoint(source, nominal.data(),
                    CoordSys.commandedMCS, row.has(plane.uAddress) ? wcs[plane.u] : 0.0,
                    row.has(plane.vAddress) ? row.val(plane.vAddress) : 0.0,
                    authoredMask, mcs.data())) return false;
            }
            else if ((fullCircle && (source.distanceMode != 90 || wcs[plane.u] <= 0.0)) ||
                !TryNCTranslationCutterPolarLineEndpoint(source, CoordSys.commandedMCS,
                    wcs[plane.u], row.val(plane.vAddress), mcs.data())) return false;
        }
        else if (source.distanceMode == 91)
        {
            // "start" is the nominal end of the preceding authored primitive.
            // Lookahead therefore accumulates current delta then NEXT delta,
            // never either delta from the physical offset tool-centre tail.
            std::array<double, 8U> nominal{};
            for (unsigned axis = 0U; axis < 8U; ++axis) nominal[axis] = CoordSys.commandedMCS[axis];
            nominal[plane.u] = start[0]; nominal[plane.v] = start[1];
            if (!TryNCTranslationIncrementalTarget(source, nominal.data(), wcs.data(),
                programmed.data(), mcs.data())) return false;
        }
        else CoordSys.Preview_WCS_to_MCS(wcs.data(), programmed.data(), mcs.data());
        for (unsigned i = 0U; i < 2U; ++i)
        {
            const unsigned axis = i == 0U ? plane.u : plane.v;
            result.start[i] = start[i];
            result.end[i] = mcs[axis];
            if (!std::isfinite(start[i]) || !std::isfinite(mcs[axis])) return false;
        }
        if (row.gCode != 1)
        {
            // Only the preceding immutable literal lookahead may grant a
            // polar circle. Equal decoded coordinates alone (360-degree alias,
            // zero radius or rounding collapse) still fail. A proved circle
            // must retain both nominal endpoint bits; no tolerance/pinning.
            if (source.polarMode == 16)
            {
                if (fullCircle)
                {
                    if (CutterDoubleBits(result.end[0]) != CutterDoubleBits(start[0]) ||
                        CutterDoubleBits(result.end[1]) != CutterDoubleBits(start[1])) return false;
                }
                else if (result.end[0] == start[0] && result.end[1] == start[1]) return false;
            }
            result.clockwise = (row.gCode == 2) != mirrored;
            double x = 0.0, y = 0.0;
            if (row.has('R'))
            {
                // Both the current and lookahead arc use NOMINAL endpoints.
                // The physical tool-centre tail already includes cutter offset
                // and must never choose the R circle or its minor/major branch.
                // Polar words were decoded above using the same frozen frame.
                // R is a signed LENGTH (not the polar endpoint radius): units
                // and positive uniform scale apply once, rotations do not
                // change it, and reflected direction already includes parity.
                const double signedRadiusMM = NCTranslationLengthToMM(row.val('R'), source.unitsMode) *
                    (source.scalingMode == 51 ? source.scalingFactor : 1.0);
                if (!TryResolveNCPathRadiusArcCenter(start[0], start[1], result.end[0], result.end[1],
                    signedRadiusMM, result.clockwise ? -1 : 1, x, y)) return false;
            }
            else
            {
                // Authored IJK belongs to the nominal source, not the already
                // offset tool-centre tail. G18 uses K/I in canonical Z/X;
                // G19 uses J/K in Y/Z. Scale/mirror/rotations apply once to
                // this vector; pivots and fixed offsets never enter it.
                const double u = row.has(plane.uCenter) ?
                    NCTranslationLengthToMM(row.val(plane.uCenter), source.unitsMode) : 0.0;
                const double v = row.has(plane.vCenter) ?
                    NCTranslationLengthToMM(row.val(plane.vCenter), source.unitsMode) : 0.0;
                if (!NCTranslationRotateArcVector(source, u, v, x, y)) return false;
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
    const double* nominalXYZ = entry ? CoordSys.commandedMCS : m_cutterLine.nominal.data();
    const double nominalStart[2] = {nominalXYZ[plane.u], nominalXYZ[plane.v]};
    // The previous immutable lookahead compared AUTHORED endpoint words,
    // not transformed coordinates. A current circle inherits that proved
    // marker only with the same run/cache/plane/source and primitive below.
    const bool currentFull = !entry && source.rotationPlane != 17 &&
        m_cutterLine.expectedPrimitive.fullCircle;
    if (!primitive(literal, nominalStart, currentFull, input.current)) return reject("NONFINITE_NOMINAL");
    for (unsigned i = 0U; i < 2U; ++i)
    {
        const unsigned axis = i == 0U ? plane.u : plane.v;
        if (CutterDoubleBits(endpoint[axis]) != CutterDoubleBits(input.current.end[i]))
            return reject("NOMINAL_ENDPOINT");
        input.actualStart[i] = CoordSys.commandedMCS[axis];
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
    if (!terminal)
    {
        if (!IsCutterContourBlockShapeValid(following, source.unitsMode, source.rotationPlane, source.polarMode == 16))
            return reject("NEXT_PLANAR_CONTOUR_OR_G40_REQUIRED");
        // No tolerance, inverse transform or angular normalization grants a
        // revolution. G90 polar requires exactly repeated authored radius AND
        // angle, with a positive radius. Thus 0/360 (or +/-0) alias spellings,
        // polar-origin words, or tiny changes lost in trig/affine rounding do
        // not turn a partial request into a full circle. Cartesian G90/G91
        // retain their established same-words/explicit-zero-deltas rule.
        const bool repeatedWords =
            literal.has(plane.uAddress) && literal.has(plane.vAddress) &&
            following.has(plane.uAddress) && following.has(plane.vAddress) &&
            CutterDoubleBits(following.val(plane.uAddress)) == CutterDoubleBits(literal.val(plane.uAddress)) &&
            CutterDoubleBits(following.val(plane.vAddress)) == CutterDoubleBits(literal.val(plane.vAddress));
        const bool nextFull = source.rotationPlane != 17 && following.gCode != 1 && !following.has('R') &&
            (source.polarMode == 16 ?
                (source.distanceMode == 90 && literal.val(plane.uAddress) > 0.0 && repeatedWords) :
                (source.distanceMode == 91 ?
                    (following.val(plane.uAddress) == 0.0 && following.val(plane.vAddress) == 0.0) : repeatedWords));
        if (!primitive(following, input.current.end, nextFull, input.next))
            return reject("NEXT_PLANAR_CONTOUR_OR_G40_REQUIRED");
    }

    NCPathCoreCutterContourOutput output{};
    if (source.rotationPlane == 17) BuildNCPathCoreCutterContour(input, output);
    else BuildNCPathCoreCutterSeamCircle(input, output);
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
    m_cutterLine.stagedPlane = source.rotationPlane;
    m_cutterLine.stagedDistanceMode = source.distanceMode;
    m_cutterLine.stagedPolarMode = source.polarMode;
    for (unsigned i = 0U; i < 2U; ++i)
    {
        endpoint[i == 0U ? plane.u : plane.v] = output.endpoint[i];
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
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(m_cutterLine.stagedPlane, plane) ||
        CoordSys.activePlane != m_cutterLine.stagedPlane ||
        (CoordSys.isAbsoluteMode ? 90 : 91) != m_cutterLine.stagedDistanceMode ||
        (CoordSys.isPolarCoordinateActive ? 16 : 15) != m_cutterLine.stagedPolarMode) return false;
    if (m_cutterLine.stagedDistanceMode == 91 || m_cutterLine.stagedPolarMode == 16)
    {
        const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
        if (!IsNCTranslationSnapshotValid(source) || !CoordSys.IsTranslationRunCurrent() ||
            source.runToken != run || source.generation != generation ||
            source.distanceMode != m_cutterLine.stagedDistanceMode || source.polarMode != m_cutterLine.stagedPolarMode ||
            source.rotationPlane != m_cutterLine.stagedPlane ||
            !IsNCTranslationCutterNotationAllowed(source.rotationPlane, source.distanceMode, source.polarMode)) return false;
    }
    for (unsigned i = 0U; i < 2U; ++i)
        if (CutterDoubleBits(physicalEnd[i == 0U ? plane.u : plane.v]) !=
            CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[i])) return false;
    if (m_cutterLine.stagedPlane != 17 &&
        CutterDoubleBits(physicalEnd[plane.normal]) !=
            CutterDoubleBits(m_cutterLine.stagedNominal[plane.normal])) return false;
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
    m_cutterLine.physicalTail = physicalEnd;
    m_cutterLine.plane = m_cutterLine.stagedPlane;
    m_cutterLine.distanceMode = m_cutterLine.stagedDistanceMode;
    m_cutterLine.polarMode = m_cutterLine.stagedPolarMode;
    m_cutterLine.valid = true;
    m_cutterLine.staged = false;
    return true;
}

void NCManager::LogCutterContourSameThread(std::uint64_t run, std::uint64_t dispatch,
    int sourcePC) const noexcept
{
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(source.rotationPlane, plane)) return;
    // Preserve the established G17 event format. New planes expose their
    // canonical coordinates explicitly instead of labelling Z/X as X/Y.
    if (source.rotationPlane == 17)
        RtPrintf("[CUTTER][SUBMITTED] run=%llu dispatch=%llu mode=%d D=%d radiusMMBits=%llu entry=%u terminal=%u pc=%d nominalXMMBits=%llu nominalYMMBits=%llu centreXMMBits=%llu centreYMMBits=%llu\n",
            static_cast<unsigned long long>(run), static_cast<unsigned long long>(dispatch),
            source.cutterMode, source.cutterD, static_cast<unsigned long long>(CutterDoubleBits(source.cutterRadiusMM)),
            m_cutterLine.valid ? 0U : 1U, m_cutterLine.stagedTerminal ? 1U : 0U, sourcePC,
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedNominal[0])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedNominal[1])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[0])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[1])));
    else
        RtPrintf("[CUTTER][SUBMITTED] run=%llu dispatch=%llu mode=%d D=%d radiusMMBits=%llu entry=%u terminal=%u pc=%d plane=%d nominalUMMBits=%llu nominalVMMBits=%llu centreUMMBits=%llu centreVMMBits=%llu\n",
            static_cast<unsigned long long>(run), static_cast<unsigned long long>(dispatch),
            source.cutterMode, source.cutterD, static_cast<unsigned long long>(CutterDoubleBits(source.cutterRadiusMM)),
            m_cutterLine.valid ? 0U : 1U, m_cutterLine.stagedTerminal ? 1U : 0U, sourcePC, source.rotationPlane,
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedNominal[plane.u])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedNominal[plane.v])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[0])),
            static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.endpoint[1])));
    if (m_cutterLine.stagedPrimitive.kind == NCPathCoreCutterPrimitiveKind::ARC)
    {
        if (source.rotationPlane == 17)
            RtPrintf("[CUTTER][ARC] run=%llu dispatch=%llu direction=%d circleXMMBits=%llu circleYMMBits=%llu pathRadiusMMBits=%llu sweepBits=%llu\n",
                static_cast<unsigned long long>(run), static_cast<unsigned long long>(dispatch),
                m_cutterLine.stagedPrimitive.clockwise ? -1 : 1,
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.centre[0])),
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.centre[1])),
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.radius)),
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.sweepRadians)));
        else
            RtPrintf("[CUTTER][ARC] run=%llu dispatch=%llu plane=%d direction=%d circleUMMBits=%llu circleVMMBits=%llu pathRadiusMMBits=%llu sweepBits=%llu\n",
                static_cast<unsigned long long>(run), static_cast<unsigned long long>(dispatch), source.rotationPlane,
                m_cutterLine.stagedPrimitive.clockwise ? -1 : 1,
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.centre[0])),
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.centre[1])),
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.radius)),
                static_cast<unsigned long long>(CutterDoubleBits(m_cutterLine.stagedGeometry.sweepRadians)));
    }
}
