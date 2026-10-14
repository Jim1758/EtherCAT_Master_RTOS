#pragma once

#include "EDMCoordinatedRTFixture.h"
#include "EDMPhysicalRecipeSequence.h"
#include <cerrno>
#include <cstdlib>

// EDM85: bounded immutable NC-authored DIAGNOSTIC geometry. The first M30
// remains the real execution barrier. Everything after it is data only and
// must never be sent to the ordinary NC dispatcher. Coordinates below belong
// to a private G21/G90/G17 frame, not CoordSys or actual machine axes.
namespace EDM85NC
{
constexpr unsigned MaximumPhysicalLines = 96U;
enum class Error : std::uint8_t
{
    None, Scope, ExecutionBarrier, MissingData, InvalidBlock, Declaration,
    Origin, Number, Feed, UnsupportedMotion, Geometry, Capacity,
    MissingTerminal, TrailingData, LineBudget, Capture
};
inline const char* ErrorName(Error value) noexcept
{
    switch (value)
    {
    case Error::None: return "NONE";
    case Error::Scope: return "SCOPE";
    case Error::ExecutionBarrier: return "EXECUTION_M30";
    case Error::MissingData: return "MISSING_DATA";
    case Error::InvalidBlock: return "INVALID_BLOCK";
    case Error::Declaration: return "DECLARATION_G21_G90_G17";
    case Error::Origin: return "VIRTUAL_ORIGIN";
    case Error::Number: return "FINITE_DECIMAL_LITERAL";
    case Error::Feed: return "CONSISTENT_FEED";
    case Error::UnsupportedMotion: return "UNSUPPORTED_MOTION";
    case Error::Geometry: return "CANONICAL_GEOMETRY";
    case Error::Capacity: return "SEGMENT_CAPACITY";
    case Error::MissingTerminal: return "DATA_M30";
    case Error::TrailingData: return "TRAILING_DATA";
    case Error::LineBudget: return "PHYSICAL_LINE_BUDGET";
    case Error::Capture: return "IMMUTABLE_CAPTURE";
    }
    return "UNKNOWN";
}
struct Result
{
    bool valid = false;
    Error error = Error::None;
    unsigned errorOffset = 0U, terminalOffset = 0U, trailingLines = 0U;
    unsigned segmentCount = 0U, lineCount = 0U, arcCount = 0U, movedAxisMask = 0U;
    double feedMmMin = 0.0;
};
// All substantial construction storage is owned persistently by the NC
// object. There are no aggregate workspace temporaries or borrowed pointers.
struct Workspace
{
    Workspace() noexcept {}
    EDM84RT::FixtureWorkspace geometry{};
    EDM84::RotaryPoint rotary[EDM84::SynchronizedPath::Capacity + 1U]{};
    NCPathCoreFeedLineInput lineInput{};
    NCPathCoreFeedLineV2 line{};
    NCPathCoreFeedArcInput arcInput{};
    NCPathCoreFeedArcV2 arc{};
    NCPathCoreRetainedGeometry row{};
};
namespace Detail
{
inline bool Decimal(const std::string& text, double& value) noexcept
{
    value = 0.0;
    if (text.empty() || text.size() > 64U) return false;
    std::size_t i = 0U;
    if (text[i] == '+' || text[i] == '-') ++i;
    bool digit = false, point = false;
    for (; i < text.size(); ++i)
    {
        const char c = text[i];
        if (c >= '0' && c <= '9') digit = true;
        else if (c == '.' && !point) point = true;
        else return false;
    }
    if (!digit) return false;
    // Same binary64 decimal conversion as the CNC literal parser. Lexical
    // filtering excludes exponents, macros, NaN, whitespace and locale commas.
    // A non-dot process locale fails closed rather than accepting a prefix.
    const int savedError = errno;
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    const int parseError = errno;
    errno = savedError;
    if (parseError == ERANGE || end != text.c_str() + text.size() || !EDM81::Finite(parsed)) return false;
    value = parsed;
    return true;
}
inline bool SingleG(const NCParsedBlock& block, unsigned code) noexcept
{
    return !block.isEmpty && block.gCount == 1 && block.mCount == 0 &&
        EDM77::Detail::EqualLiteral(block.gExpressions[0], code);
}
inline bool Declaration(const NCParsedBlock& block) noexcept
{
    if (block.isEmpty || block.gCount != 3 || block.mCount != 0 ||
        !EDM77::Detail::OnlyAddresses(block, "N")) return false;
    unsigned mask = 0U;
    for (int i = 0; i < 3; ++i)
    {
        const unsigned bit = EDM77::Detail::EqualLiteral(block.gExpressions[i], 21U) ? 1U :
            EDM77::Detail::EqualLiteral(block.gExpressions[i], 90U) ? 2U :
            EDM77::Detail::EqualLiteral(block.gExpressions[i], 17U) ? 4U : 0U;
        if (!bit || (mask & bit)) return false;
        mask |= bit;
    }
    return mask == 7U;
}
inline bool PoseWords(const NCParsedBlock& block, std::array<double, 8U>& pose,
    bool allRequired) noexcept
{
    const char names[] = "XYZCUV";
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if (block.has(names[axis]))
        {
            if (!Decimal(block.expression(names[axis]), pose[axis])) return false;
        }
        else if (allRequired) return false;
    }
    return true;
}
inline bool Append(Workspace& work, const std::array<double, 8U>& a,
    const std::array<double, 8U>& b, const NCParsedBlock& block, double feed,
    bool isLine, int direction, const std::array<double, 8U>& scale) noexcept
{
    if (isLine)
    {
        auto& input = work.lineInput;
        input = NCPathCoreFeedLineInput{};
        input.axisMask = 7U; input.feedMMMin = feed;
        input.maxVelocityPPS.fill(100000.0);
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            input.startMCS[axis] = a[axis]; input.endMCS[axis] = b[axis];
            input.startPulse[axis] = a[axis] * scale[axis];
            input.endPulse[axis] = b[axis] * scale[axis];
        }
        if (BuildNCPathCoreFeedLine(input, work.line) != NCPathCoreFeedLineCode::BUILT_LINE ||
            !BuildNCPathCoreRetainedLine(work.line, work.row)) return false;
    }
    else
    {
        auto& input = work.arcInput;
        input = NCPathCoreFeedArcInput{};
        input.feedMMMin = feed; input.direction = direction; input.plane = 17U;
        input.fullCircle = !block.has('X') && !block.has('Y');
        input.pulsePerMM = {{scale[0], scale[1]}};
        input.maxVelocityPPS = {{100000.0, 100000.0}};
        if ((!block.has('I') && !block.has('J')) ||
            (block.has('I') && !Decimal(block.expression('I'), input.centerOffsetMM[0])) ||
            (block.has('J') && !Decimal(block.expression('J'), input.centerOffsetMM[1]))) return false;
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            input.startMCS[axis] = a[axis]; input.endMCS[axis] = b[axis];
            input.startPulse[axis] = a[axis] * scale[axis];
            input.endPulse[axis] = b[axis] * scale[axis];
        }
        if (BuildNCPathCoreFeedArc(input, work.arc) != NCPathCoreFeedArcCode::BUILT_ARC ||
            !BuildNCPathCoreRetainedArc(work.arc, work.row)) return false;
    }
    // Native base rows deliberately hold C/U/V at zero. Their authored values
    // are stored solely in the synchronized sidecar, never in XYZ arc math.
    return work.geometry.source.Append(work.row, 0x3FU, scale);
}
}

// get(1) is the exact physical line immediately following G180 and must be
// the real M30. A getter returns nullptr only at immutable-cache EOF, and must
// not allocate, mutate, resolve expressions, or throw. We inspect at most 96
// physical suffix rows and one EOF probe. No source pointer survives return.
template<class Getter>
Result Capture(EDM84RT::Request& request, Workspace& workspace, Getter get) noexcept
{
    EDM84RT::FixtureDetail::ResetInPlace(request.path);
    EDM84RT::FixtureDetail::ResetInPlace(workspace);
    Result result{};
    const auto fail = [&](Error error, unsigned offset) noexcept -> Result
    { result.error = error; result.errorOffset = offset; return result; };
    if (!EDM81::ValidScope(request.scope)) return fail(Error::Scope, 0U);
    const NCParsedBlock* barrier = get(1U);
    if (!barrier || !EDM77::Detail::Clean(*barrier) || !EDM77::Detail::Terminal(*barrier))
        return fail(Error::ExecutionBarrier, 1U);
    unsigned stage = 0U; // declaration, origin, path, terminal-following empties
    std::array<double, 8U> pose{};
    std::array<double, 8U> scale{}; scale.fill(1000.0);
    for (unsigned offset = 2U; offset <= MaximumPhysicalLines + 1U; ++offset)
    {
        const NCParsedBlock* block = get(offset);
        if (!block)
        {
            if (stage != 3U) return fail(stage < 2U ? Error::MissingData : Error::MissingTerminal, offset);
            if (!workspace.geometry.base.Capture(workspace.geometry.source, request.scope, scale) ||
                !request.path.Capture(workspace.geometry.base, workspace.rotary, result.segmentCount + 1U, request.scope))
            { EDM84RT::FixtureDetail::ResetInPlace(request.path); return fail(Error::Capture, offset); }
            result.valid = true;
            return result;
        }
        if (offset > MaximumPhysicalLines) return fail(Error::LineBudget, offset);
        if (!EDM77::Detail::Clean(*block)) return fail(Error::InvalidBlock, offset);
        if (EDM77::Detail::Empty(*block))
        { if (stage == 3U) ++result.trailingLines; continue; }
        if (stage == 3U) return fail(Error::TrailingData, offset);
        if (stage == 0U)
        {
            if (!Detail::Declaration(*block)) return fail(Error::Declaration, offset);
            stage = 1U; continue;
        }
        if (stage == 1U)
        {
            if (!Detail::SingleG(*block, 0U) || !EDM77::Detail::OnlyAddresses(*block, "NXYZCUV") ||
                !Detail::PoseWords(*block, pose, true)) return fail(Error::Origin, offset);
            workspace.rotary[0] = {{pose[3], pose[4], pose[5]}};
            stage = 2U; continue;
        }
        if (EDM77::Detail::Terminal(*block))
        {
            if (!result.segmentCount) return fail(Error::MissingData, offset);
            result.terminalOffset = offset; stage = 3U; continue;
        }
        const bool isLine = Detail::SingleG(*block, 1U);
        const int direction = Detail::SingleG(*block, 2U) ? -1 : Detail::SingleG(*block, 3U) ? 1 : 0;
        if ((!isLine && !direction) || !EDM77::Detail::OnlyAddresses(*block, isLine ? "NXYZCUVF" : "NXYZCUVFIJ"))
            return fail(Error::UnsupportedMotion, offset);
        if (result.segmentCount >= EDM84::SynchronizedPath::Capacity) return fail(Error::Capacity, offset);
        double feed = result.feedMmMin;
        if (block->has('F'))
        {
            if (!Detail::Decimal(block->expression('F'), feed) || feed <= 0.0 || feed > 100.0 ||
                (result.segmentCount && feed != result.feedMmMin)) return fail(Error::Feed, offset);
        }
        else if (!result.segmentCount) return fail(Error::Feed, offset);
        auto end = pose;
        if (!Detail::PoseWords(*block, end, false)) return fail(Error::Number, offset);
        if (!Detail::Append(workspace, pose, end, *block, feed, isLine, direction, scale))
            return fail(Error::Geometry, offset);
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (pose[axis] != end[axis]) result.movedAxisMask |= 1U << axis;
        if (!isLine) result.movedAxisMask |= 3U; // Any nondegenerate XY circle moves both axes.
        ++result.segmentCount;
        if (isLine) ++result.lineCount; else ++result.arcCount;
        result.feedMmMin = feed;
        workspace.rotary[result.segmentCount] = {{end[3], end[4], end[5]}};
        pose = end;
    }
    return fail(Error::LineBudget, MaximumPhysicalLines + 1U);
}
static_assert(std::is_nothrow_default_constructible<Workspace>::value &&
    std::is_trivially_destructible<Workspace>::value, "EDM85 capture workspace must reset in place");
}
