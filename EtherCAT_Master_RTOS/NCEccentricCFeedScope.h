#pragma once

#include "NCXYZFeedScope.h"
#include "NCEccentricCPath.h"
#include "NCRotaryFeedTarget.h"
#include "GCodeParser.h"

// BASE79G: the first NC producer is deliberately bounded independently of the
// immutable geometry kernel. These predicates grant no owner, epoch, queue,
// receipt or Motion authority. Admission still proves the live frozen source,
// complete execution drain and the whole authored/stopping travel envelope.
// The old value-kernel admission constant stays false: building geometry
// alone grants no motion. Only this independently guarded NC entry is enabled.
constexpr bool NCEccentricCRestrictedNCAdmission = true;
constexpr double NCEccentricCFeedMaximumSweepDeg = 10.0;
constexpr double NCEccentricCFeedMaximumZDeltaMM = 1.0;
constexpr double NCEccentricCFeedMaximumLinearLengthMM = 1.0;
constexpr double NCEccentricCFeedMaximumRadiusMM = 2.0;
constexpr double NCEccentricCFeedMaximumFeedDegMin = 100.0;
// Existing absolute-rotary resolution contract uses a 1e-5 pulse arithmetic
// budget. A G90 displacement inside that budget cannot distinguish a new
// endpoint from the represented residual of the same previously executed
// target. Refuse it before motion; this is not an arrival window or a no-op.
constexpr double NCEccentricCFeedAbsolutePulseResolution = 1.0e-5;

inline bool IsNCEccentricCFeedDecimalLiteral(const std::string& expression) noexcept
{
    if (expression.empty()) return false;
    std::size_t index = expression[0] == '+' || expression[0] == '-' ? 1U : 0U;
    bool digit = false, point = false;
    for (; index < expression.size(); ++index)
    {
        const char value = expression[index];
        if (value >= '0' && value <= '9') digit = true;
        else if (value == '.' && !point) point = true;
        else return false;
    }
    return digit;
}

// The resolved NCBlock cannot prove authorship: expressions/macros have already
// become doubles. Check its cached parsed source before any setting/capture.
// Range/finiteness and actual axis ownership remain the resolved block's job.
inline bool IsNCEccentricCFeedParsedBlockAllowed(const NCParsedBlock& block) noexcept
{
    if (block.isEmpty || block.isBlockSkip || block.error != NCParseError::NONE ||
        block.dependsOnMacroState || block.controlType != NCParsedControlType::NONE ||
        block.gCount != 1 || block.mCount != 0 ||
        (block.gExpressions[0] != "1" && block.gExpressions[0] != "01") ||
        !block.has('C') || !block.has('F')) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'X' && word != 'Y' && word != 'Z' && word != 'C' && word != 'F') ||
            !IsNCEccentricCFeedDecimalLiteral(block.expression(word)))) return false;
    return true;
}

inline bool IsNCEccentricCFeedFrame(const NCTranslationSnapshot& source) noexcept
{
    return NCEccentricCRestrictedNCAdmission && NCEccentricCDetail::Scope(source) &&
        IsNCXYZFeedAxisScopeValid(source.axisIdentity) &&
        (source.distanceMode == 90 || source.distanceMode == 91) &&
        source.axisIdentity.electrodeAxisPlusOne == 4U &&
        source.axisIdentity.exists[3] == 1U &&
        source.axisIdentity.axisType[3] == 1U &&
        source.axisIdentity.nativeUnit[3] == 2U &&
        source.axisIdentity.address[3] == 'C' &&
        std::hypot(source.toolOffsetMM[0], source.toolOffsetMM[1]) <=
            NCEccentricCFeedMaximumRadiusMM;
}

// BASE79I: resolve only the commanded physical C endpoint. G90 uses the
// accepted native rotary wrap/tie rule; eccentric phase remains the unwrapped
// physical angle. This value grants no travel, owner, queue or Motion authority.
struct NCEccentricCFeedTarget
{
    double requestedMCS = 0.0; // Authored G90 target including frozen offsets.
    double endMCS = 0.0;
    double endPulse = 0.0; // Represented endpoint emitted by the existing runtime.
    double sweepDeg = 0.0;
    bool valid = false;
};

inline bool TryResolveNCEccentricCFeedTarget(const NCTranslationSnapshot& source,
    double programmedC, double startPulse, double pulsePerDegree,
    bool shortestPath, double rotaryModulo, NCEccentricCFeedTarget& output) noexcept
{
    using namespace NCEccentricCDetail;
    output = NCEccentricCFeedTarget{};
    if (!IsNCEccentricCFeedFrame(source) || !std::isfinite(programmedC) ||
        !std::isfinite(startPulse) || !Positive(pulsePerDegree)) return false;
    const double startMCS = startPulse / pulsePerDegree;
    if (!std::isfinite(startMCS)) return false;
    NCEccentricCFeedTarget resolved{};
    double canonicalPulse = 0.0;
    if (source.distanceMode == 90)
    {
        resolved.requestedMCS = programmedC + NCTranslationAxisOffsetMM(source, 3U);
        NCRotaryAbsoluteFeedTarget absolute{};
        if (!TryResolveNCRotaryAbsoluteFeedTarget(startMCS, startPulse,
            resolved.requestedMCS, pulsePerDegree, shortestPath, rotaryModulo,
            absolute) || !absolute.valid) return false;
        resolved.sweepDeg = absolute.sweepDeg;
        canonicalPulse = absolute.endPulse;
    }
    else
    {
        // Preserve the original G91 sweep bits; coordinate offsets never alter
        // an incremental displacement, and no shortest-path wrap applies.
        resolved.sweepDeg = programmedC;
        resolved.requestedMCS = startMCS + programmedC;
    }
    if (!std::isfinite(resolved.requestedMCS) || !std::isfinite(resolved.sweepDeg) ||
        resolved.sweepDeg == 0.0 ||
        std::fabs(resolved.sweepDeg) > NCEccentricCFeedMaximumSweepDeg) return false;
    resolved.endMCS = startMCS + resolved.sweepDeg;
    resolved.endPulse = startPulse + resolved.sweepDeg * pulsePerDegree;
    const double nativeSweep = resolved.endMCS - startMCS;
    const double pulseSweep = resolved.endPulse - startPulse;
    if (!std::isfinite(resolved.endMCS) || !std::isfinite(resolved.endPulse) ||
        !std::isfinite(nativeSweep) || !std::isfinite(pulseSweep) ||
        nativeSweep == 0.0 || pulseSweep == 0.0 ||
        std::signbit(nativeSweep) != std::signbit(resolved.sweepDeg) ||
        std::signbit(pulseSweep) != std::signbit(resolved.sweepDeg)) return false;
    const double nativeError = std::fabs(nativeSweep - resolved.sweepDeg);
    if (nativeError > NativeRoundoffBudget ||
        nativeError > std::fabs(resolved.sweepDeg) * RelativeSweepRoundoff) return false;
    if (source.distanceMode == 90)
    {
        const double canonicalSweep = canonicalPulse - startPulse;
        const double endpointError = std::fabs(resolved.endPulse - canonicalPulse);
        const double scale = Max(std::fabs(startPulse), std::fabs(canonicalPulse));
        const double roundoff = Min(1.0e-5, scale * (32.0 * Epsilon));
        // Never clamp or erase a residual move. Reject loss of direction or
        // excessive endpoint error before any identity/enqueue/tail mutation.
        if (!std::isfinite(canonicalSweep) || std::fabs(canonicalSweep) <= NCEccentricCFeedAbsolutePulseResolution ||
            std::signbit(canonicalSweep) != std::signbit(pulseSweep) ||
            !std::isfinite(endpointError) || !std::isfinite(roundoff) ||
            endpointError > roundoff ||
            endpointError / pulsePerDegree > NativeRoundoffBudget ||
            endpointError > std::fabs(canonicalSweep) * RelativeSweepRoundoff) return false;
    }
    resolved.valid = true;
    output = resolved;
    return true;
}

// BASE79K: resolve the authored Z endpoint independently from C target/wrap
// selection. A signed H-Z offset translates an absolute target once; it never
// becomes an additional incremental displacement or a rotating XY component.
struct NCEccentricCZFeedTarget
{
    double requestedMCS = 0.0;
    double endMCS = 0.0;
    double endPulse = 0.0;
    double deltaMM = 0.0;
    bool valid = false;
};

inline bool TryResolveNCEccentricCZFeedTarget(const NCTranslationSnapshot& source,
    double programmedZ, double startPulse, double pulsePerMM,
    NCEccentricCZFeedTarget& output) noexcept
{
    using namespace NCEccentricCDetail;
    output = NCEccentricCZFeedTarget{};
    if (!IsNCEccentricCFeedFrame(source) || !std::isfinite(programmedZ) ||
        !std::isfinite(startPulse) || !Positive(pulsePerMM)) return false;
    const double startMCS = startPulse / pulsePerMM;
    if (!std::isfinite(startMCS)) return false;
    NCEccentricCZFeedTarget resolved{};
    double canonicalPulse = 0.0;
    if (source.distanceMode == 90)
    {
        resolved.requestedMCS = programmedZ + NCTranslationAxisOffsetMM(source, 2U);
        canonicalPulse = resolved.requestedMCS * pulsePerMM;
        if (!IsNCRotaryAbsolutePulseResolutionValid(startPulse) ||
            !IsNCRotaryAbsolutePulseResolutionValid(canonicalPulse) ||
            (resolved.requestedMCS != 0.0 && canonicalPulse == 0.0)) return false;
        resolved.deltaMM = resolved.requestedMCS - startMCS;
    }
    else
    {
        resolved.deltaMM = programmedZ;
        resolved.requestedMCS = startMCS + programmedZ;
    }
    if (!std::isfinite(resolved.requestedMCS) || !std::isfinite(resolved.deltaMM) ||
        resolved.deltaMM == 0.0 ||
        std::fabs(resolved.deltaMM) > NCEccentricCFeedMaximumZDeltaMM) return false;
    resolved.endMCS = startMCS + resolved.deltaMM;
    resolved.endPulse = startPulse + resolved.deltaMM * pulsePerMM;
    const double nativeDelta = resolved.endMCS - startMCS;
    const double pulseDelta = resolved.endPulse - startPulse;
    if (!std::isfinite(resolved.endMCS) || !std::isfinite(resolved.endPulse) ||
        !std::isfinite(nativeDelta) || !std::isfinite(pulseDelta) ||
        nativeDelta == 0.0 || pulseDelta == 0.0 ||
        std::signbit(nativeDelta) != std::signbit(resolved.deltaMM) ||
        std::signbit(pulseDelta) != std::signbit(resolved.deltaMM)) return false;
    const double nativeError = std::fabs(nativeDelta - resolved.deltaMM);
    if (nativeError > NativeRoundoffBudget ||
        nativeError > std::fabs(resolved.deltaMM) * RelativeSweepRoundoff) return false;
    if (source.distanceMode == 90)
    {
        const double canonicalDelta = canonicalPulse - startPulse;
        const double endpointError = std::fabs(resolved.endPulse - canonicalPulse);
        const double scale = Max(std::fabs(startPulse), std::fabs(canonicalPulse));
        const double roundoff = Min(1.0e-5, scale * (32.0 * Epsilon));
        if (!std::isfinite(canonicalDelta) ||
            std::fabs(canonicalDelta) <= NCEccentricCFeedAbsolutePulseResolution ||
            std::signbit(canonicalDelta) != std::signbit(pulseDelta) ||
            !std::isfinite(endpointError) || !std::isfinite(roundoff) ||
            endpointError > roundoff || endpointError / pulsePerMM > NativeRoundoffBudget ||
            endpointError > std::fabs(canonicalDelta) * RelativeSweepRoundoff) return false;
    }
    resolved.valid = true;
    output = resolved;
    return true;
}

// BASE79M: XY absolute words address the nominal tool path. Remove the
// signed H rotated at the physical starting C angle, then replace the authored
// nominal coordinate in the frozen EXT/WCS frame. The physical endpoint adds
// H at the resolved unwrapped ending C angle. Omitted axes never call this
// helper and retain zero authored delta even when their generated H moves.
struct NCEccentricCXYFeedTarget
{
    double requestedNominalMCS = 0.0;
    double nominalStartMCS = 0.0;
    double requestedMCS = 0.0; // Canonical physical endpoint including rotated H.
    double endMCS = 0.0;       // Existing geometry arithmetic, kept bit for bit.
    double endPulse = 0.0;     // Existing runtime arithmetic, kept bit for bit.
    double deltaMM = 0.0;
    bool valid = false;
};

inline bool TryResolveNCEccentricCXYFeedTarget(const NCTranslationSnapshot& source,
    unsigned axis, double programmed, double startPulse, double pulsePerMM,
    double startCDeg, double endCDeg, NCEccentricCXYFeedTarget& output) noexcept
{
    using namespace NCEccentricCDetail;
    output = NCEccentricCXYFeedTarget{};
    if (axis > 1U || !IsNCEccentricCFeedFrame(source) ||
        !std::isfinite(programmed) || !std::isfinite(startPulse) ||
        !Positive(pulsePerMM)) return false;
    std::array<double, 3U> startOffset{}, endOffset{};
    if (!Offset(source, startCDeg, startOffset) ||
        !Offset(source, endCDeg, endOffset)) return false;
    const double startMCS = startPulse / pulsePerMM;
    NCEccentricCXYFeedTarget resolved{};
    resolved.nominalStartMCS = startMCS - startOffset[axis];
    if (!std::isfinite(startMCS) || !std::isfinite(resolved.nominalStartMCS)) return false;
    if (source.distanceMode == 90)
    {
        // Scope excludes WORK rotation/translation. Do not add then subtract
        // static H: that loses small EXT/WCS values through cancellation.
        resolved.requestedNominalMCS = programmed +
            (source.extOffsetMM[axis] + source.wcsOffsetMM[axis]);
        resolved.deltaMM = resolved.requestedNominalMCS - resolved.nominalStartMCS;
    }
    else
    {
        resolved.deltaMM = programmed;
        resolved.requestedNominalMCS = resolved.nominalStartMCS + programmed;
    }
    if (!std::isfinite(resolved.requestedNominalMCS) ||
        !std::isfinite(resolved.deltaMM) || resolved.deltaMM == 0.0 ||
        std::fabs(resolved.deltaMM) > NCEccentricCFeedMaximumLinearLengthMM) return false;
    const double combinedDelta = resolved.deltaMM + (endOffset[axis] - startOffset[axis]);
    resolved.endMCS = startMCS + combinedDelta;
    resolved.endPulse = startPulse + combinedDelta * pulsePerMM;
    resolved.requestedMCS = resolved.requestedNominalMCS + endOffset[axis];
    if (!std::isfinite(combinedDelta) || !std::isfinite(resolved.endMCS) ||
        !std::isfinite(resolved.endPulse) || !std::isfinite(resolved.requestedMCS)) return false;
    if (source.distanceMode == 90)
    {
        const double nominalStartPulse = resolved.nominalStartMCS * pulsePerMM;
        const double canonicalNominalPulse = resolved.requestedNominalMCS * pulsePerMM;
        const double canonicalPulse = resolved.requestedMCS * pulsePerMM;
        if (!IsNCRotaryAbsolutePulseResolutionValid(startPulse) ||
            !IsNCRotaryAbsolutePulseResolutionValid(nominalStartPulse) ||
            !IsNCRotaryAbsolutePulseResolutionValid(canonicalNominalPulse) ||
            !IsNCRotaryAbsolutePulseResolutionValid(canonicalPulse) ||
            (resolved.nominalStartMCS != 0.0 && nominalStartPulse == 0.0) ||
            (resolved.requestedNominalMCS != 0.0 && canonicalNominalPulse == 0.0) ||
            (resolved.requestedMCS != 0.0 && canonicalPulse == 0.0)) return false;
        const double canonicalDelta = canonicalNominalPulse - nominalStartPulse;
        const double representedNominalPulse = nominalStartPulse + resolved.deltaMM * pulsePerMM;
        const double representedDelta = representedNominalPulse - nominalStartPulse;
        const double nominalError = std::fabs(representedNominalPulse - canonicalNominalPulse);
        const double nominalScale = Max(std::fabs(nominalStartPulse), std::fabs(canonicalNominalPulse));
        const double nominalRoundoff = Min(NCEccentricCFeedAbsolutePulseResolution,
            nominalScale * (32.0 * Epsilon));
        const double endpointError = std::fabs(resolved.endPulse - canonicalPulse);
        const double endpointScale = Max(std::fabs(startPulse), Max(std::fabs(canonicalPulse),
            Max(std::fabs(nominalStartPulse), std::fabs(canonicalNominalPulse))));
        const double endpointRoundoff = Min(NCEccentricCFeedAbsolutePulseResolution,
            endpointScale * (64.0 * Epsilon));
        // This is an arithmetic resolution bound, never an arrival window.
        // It rejects repeat-target residuals instead of silently turning an
        // authored word into an omitted axis. Physical cancellation is valid:
        // the nonzero proof concerns the nominal delta, not net physical XY.
        if (!std::isfinite(canonicalDelta) ||
            std::fabs(canonicalDelta) <= NCEccentricCFeedAbsolutePulseResolution ||
            !std::isfinite(representedDelta) || representedDelta == 0.0 ||
            std::signbit(canonicalDelta) != std::signbit(resolved.deltaMM) ||
            std::signbit(representedDelta) != std::signbit(resolved.deltaMM) ||
            !std::isfinite(nominalError) || !std::isfinite(nominalRoundoff) ||
            nominalError > nominalRoundoff || nominalError / pulsePerMM > NativeRoundoffBudget ||
            nominalError > std::fabs(canonicalDelta) * RelativeSweepRoundoff ||
            !std::isfinite(endpointError) || !std::isfinite(endpointRoundoff) ||
            endpointError > endpointRoundoff || endpointError / pulsePerMM > NativeRoundoffBudget ||
            endpointError > std::fabs(canonicalDelta) * RelativeSweepRoundoff) return false;
    }
    resolved.valid = true;
    output = resolved;
    return true;
}

// BASE79K: existing C-only F is degrees/minute; authored Z+C F
// is Z millimetres/minute. Only this conversion grants the scalar C rate. An
// explicit G91 Z0 word is rejected by NC shape checks. G90 Z0 instead resolves
// its physical delta; an actual zero delta also rejects instead of becoming C-only.
inline bool TryResolveNCEccentricCFeedRate(double zDeltaMM, double sweepDeg,
    double programmedFeed, double& feedDegMin) noexcept
{
    feedDegMin = 0.0;
    if (!std::isfinite(zDeltaMM) || !std::isfinite(programmedFeed) ||
        programmedFeed <= 0.0) return false;
    double resolved = programmedFeed;
    if (zDeltaMM != 0.0)
    {
        if (std::fabs(zDeltaMM) > NCEccentricCFeedMaximumZDeltaMM ||
            !std::isfinite(sweepDeg) || sweepDeg == 0.0 ||
            std::fabs(sweepDeg) > NCEccentricCFeedMaximumSweepDeg) return false;
        resolved = programmedFeed * std::fabs(sweepDeg / zDeltaMM);
    }
    if (!std::isfinite(resolved) || resolved <= 0.0 ||
        resolved > NCEccentricCFeedMaximumFeedDegMin) return false;
    feedDegMin = resolved;
    return true;
}

// BASE79L: only authored linear motion sets the mixed feed length. The
// rotating H contribution is generated geometry, not extra programmed feed.
// Preserve the established C-only / Z+C arithmetic when X and Y are absent.
inline bool TryResolveNCEccentricCLinearFeedRate(double xDeltaMM, double yDeltaMM,
    double zDeltaMM, double sweepDeg, double programmedFeed, double& feedDegMin) noexcept
{
    if (xDeltaMM == 0.0 && yDeltaMM == 0.0)
        return TryResolveNCEccentricCFeedRate(zDeltaMM, sweepDeg, programmedFeed, feedDegMin);
    feedDegMin = 0.0;
    if (!std::isfinite(xDeltaMM) || !std::isfinite(yDeltaMM) ||
        !std::isfinite(zDeltaMM) || !std::isfinite(sweepDeg) || sweepDeg == 0.0 ||
        std::fabs(sweepDeg) > NCEccentricCFeedMaximumSweepDeg ||
        !std::isfinite(programmedFeed) || programmedFeed <= 0.0) return false;
    const double lengthMM = std::hypot(std::hypot(xDeltaMM, yDeltaMM), zDeltaMM);
    if (!std::isfinite(lengthMM) || lengthMM <= 0.0 ||
        lengthMM > NCEccentricCFeedMaximumLinearLengthMM) return false;
    const double resolved = programmedFeed * (std::fabs(sweepDeg) / lengthMM);
    if (!std::isfinite(resolved) || resolved <= 0.0 ||
        resolved > NCEccentricCFeedMaximumFeedDegMin) return false;
    feedDegMin = resolved;
    return true;
}

inline bool IsNCEccentricCFeedBlockAllowed(const NCTranslationSnapshot& source,
    const NCBlock& block) noexcept
{
    if (!IsNCEccentricCFeedFrame(source) ||
        !IsNCXYZFeedSingleGBlock(block, 1) || !block.has('C') ||
        !block.has('F') || (source.distanceMode == 91 &&
            (block.val('C') == 0.0 ||
                std::fabs(block.val('C')) > NCEccentricCFeedMaximumSweepDeg)))
        return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'X' && word != 'Y' && word != 'Z' && word != 'C' && word != 'F') ||
            !std::isfinite(block.val(word)))) return false;
    if (block.has('X') || block.has('Y'))
    {
        if (source.distanceMode == 90)
            // Raw absolute XYZ/C values carry no path length or feed ratio.
            // Resolve every authored delta against the stopped physical anchor.
            return block.val('F') > 0.0;
        for (char word : { 'X', 'Y', 'Z' })
            if (block.has(word) && block.val(word) == 0.0) return false;
        double angularFeed = 0.0;
        return TryResolveNCEccentricCLinearFeedRate(
            block.has('X') ? block.val('X') : 0.0,
            block.has('Y') ? block.val('Y') : 0.0,
            block.has('Z') ? block.val('Z') : 0.0,
            block.val('C'), block.val('F'), angularFeed);
    }
    const double zDeltaMM = block.has('Z') ? block.val('Z') : 0.0;
    if (block.has('Z') && source.distanceMode == 90)
        // Raw absolute coordinates do not describe the physical displacement
        // or pitch. Both caps and the derived C rate are proved at the anchor.
        return std::isfinite(block.val('F')) && block.val('F') > 0.0;
    if (block.has('Z') && zDeltaMM == 0.0)
        return false;
    double feedDegMin = 0.0;
    return TryResolveNCEccentricCFeedRate(zDeltaMM, block.val('C'),
        block.val('F'), feedDegMin);
}

inline bool IsNCEccentricCRotationSelectionBlockAllowed(const NCBlock& block) noexcept
{
    if (!IsNCXYZFeedSingleGBlock(block, 162) &&
        !IsNCXYZFeedSingleGBlock(block, 163)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && (word != 'N' || !std::isfinite(block.val(word))))
            return false;
    return true;
}

// A frozen active eccentric H source cannot enter a legacy geometry or frame
// lane. Cancel with a drained standalone G163 before other frame selections.
// Dwell and program stops retain their existing downstream validation/lifecycle.
inline bool IsNCEccentricCFrozenNonMotionBlockAllowed(const NCBlock& block) noexcept
{
    if (IsNCXYZFeedSingleGBlock(block, 4))
    {
        for (char word = 'A'; word <= 'Z'; ++word)
            if (block.has(word) && ((word != 'N' && word != 'X' && word != 'P') ||
                !std::isfinite(block.val(word)))) return false;
        return true;
    }
    if (block.isGoto || block.isBlockSkip || block.hasG || block.gCount != 0)
        return false;
    if (!block.isEmpty && block.mCount == 1 &&
        (block.mCode[0] == 0 || block.mCode[0] == 30))
    {
        for (char word = 'A'; word <= 'Z'; ++word)
            if (block.has(word) && (word != 'N' || !std::isfinite(block.val(word))))
                return false;
        return true;
    }
    if (!block.isEmpty || block.mCount != 0) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word)) return false;
    return true;
}
