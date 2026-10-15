#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// EDM109: fixed-value G01 geometry and completed-only numerical history.
// XYZ determines distance in mm; C/U/V stay unwrapped native degrees and use
// the same parameter. None of these values grants execution/output authority.
namespace EDM109
{
constexpr unsigned AxisCount = 6U;
constexpr unsigned Capacity = 16U;
using Point = std::array<double, AxisCount>;

inline bool SameDouble(double a, double b) noexcept
{ return std::memcmp(&a, &b, sizeof(double)) == 0; }
inline bool SamePulse(const Point& a, const Point& b) noexcept
{
    for (unsigned i = 0U; i < AxisCount; ++i)
        if (!SameDouble(a[i], b[i])) return false;
    return true;
}

struct Line
{
    Point startPulse{}, endPulse{}, pulsePerNative{}, ratio{};
    double pulseLength = 0.0, lengthMm = 0.0;
    bool valid = false;
};

inline bool BuildLine(const Point& startPulse, const Point& endPulse,
    const Point& pulsePerNative, Line& output) noexcept
{
    Line candidate{};
    candidate.startPulse = startPulse; candidate.endPulse = endPulse;
    candidate.pulsePerNative = pulsePerNative;
    for (unsigned i = 0U; i < AxisCount; ++i)
    {
        if (!std::isfinite(startPulse[i]) || !std::isfinite(endPulse[i]) ||
            !std::isfinite(pulsePerNative[i]) || pulsePerNative[i] <= 0.0) return false;
        const double delta = endPulse[i] - startPulse[i];
        if (!std::isfinite(delta)) return false;
        candidate.ratio[i] = delta;
        // Preserve ordinary six-axis LINEAR loader's operation order.
        candidate.pulseLength = std::hypot(candidate.pulseLength, delta);
        if (i < 3U)
        {
            const double nativeDelta = delta / pulsePerNative[i];
            if (!std::isfinite(nativeDelta)) return false;
            candidate.lengthMm = std::hypot(candidate.lengthMm, nativeDelta);
        }
    }
    if (!std::isfinite(candidate.pulseLength) || candidate.pulseLength <= 0.0 ||
        !std::isfinite(candidate.lengthMm) || candidate.lengthMm <= 0.0) return false;
    for (unsigned i = 0U; i < AxisCount; ++i)
    {
        candidate.ratio[i] /= candidate.pulseLength;
        if (!std::isfinite(candidate.ratio[i])) return false;
    }
    candidate.valid = true; output = candidate; return true;
}

inline bool ValidLine(const Line& line) noexcept
{
    Line rebuilt{};
    return line.valid && BuildLine(line.startPulse, line.endPulse, line.pulsePerNative, rebuilt) &&
        SameDouble(line.pulseLength, rebuilt.pulseLength) && SameDouble(line.lengthMm, rebuilt.lengthMm) &&
        SamePulse(line.ratio, rebuilt.ratio);
}

// Raw physical LINEAR mapping: retain the original multiplication/addition
// order, including endpoints. The caller owns scalar bounds and formal-stop
// authority. Finite q outside [0,pulseLength] is not a B2 permission.
inline bool EvaluateScalar(const Line& line, double q, double v,
    Point& position, Point& velocity) noexcept
{
    if (!line.valid || !std::isfinite(q) || !std::isfinite(v)) return false;
    Point candidatePosition{}, candidateVelocity{};
    for (unsigned i = 0U; i < AxisCount; ++i)
    {
        const double mappedTerm = q * line.ratio[i];
        candidatePosition[i] = line.startPulse[i] + mappedTerm;
        candidateVelocity[i] = v * line.ratio[i];
        if (!std::isfinite(candidatePosition[i]) || !std::isfinite(candidateVelocity[i])) return false;
    }
    position = candidatePosition; velocity = candidateVelocity; return true;
}

// Numerical history evaluator. Endpoint copies are exact and are deliberately
// separate from ordinary LINEAR mapping, whose rounding must stay unchanged.
inline bool EvaluateMm(const Line& line, double s, Point& position) noexcept
{
    if (!ValidLine(line) || !std::isfinite(s) || s < 0.0 || s > line.lengthMm) return false;
    if (s == 0.0) { position = line.startPulse; return true; }
    if (s == line.lengthMm) { position = line.endPulse; return true; }
    const double u = s / line.lengthMm;
    Point candidate{};
    for (unsigned i = 0U; i < AxisCount; ++i)
    {
        candidate[i] = line.startPulse[i] + (line.endPulse[i] - line.startPulse[i]) * u;
        if (!std::isfinite(candidate[i])) return false;
    }
    position = candidate; return true;
}

class Mapping
{
public:
    bool BindLine(const Point& startPulse, const Point& endPulse,
        const Point& pulsePerNative, double expectedPulseLength,
        const Point& expectedRatio) noexcept
    {
        Clear(); Line candidate{};
        if (!BuildLine(startPulse, endPulse, pulsePerNative, candidate) ||
            !SameDouble(candidate.pulseLength, expectedPulseLength) ||
            !SamePulse(candidate.ratio, expectedRatio)) return false;
        line_ = candidate; return true;
    }
    bool MapPulse(double q, double v, Point& position, Point& velocity) const noexcept
    { return EvaluateScalar(line_, q, v, position, velocity); }
    const Line& Geometry() const noexcept { return line_; }
    void Clear() noexcept { line_ = Line{}; }
private:
    Line line_{};
};

struct Scope
{
    std::uint64_t session = 0ULL, run = 0ULL, cache = 0ULL, ownerGeneration = 0ULL;
    std::uint64_t configTag = 0ULL, mappingTag = 0ULL, translationGeneration = 0ULL;
};
inline bool ValidScope(const Scope& s) noexcept
{
    return s.session && s.run && s.cache && s.ownerGeneration &&
        s.configTag && s.mappingTag && s.translationGeneration;
}
inline bool SameScope(const Scope& a, const Scope& b) noexcept
{
    return a.session == b.session && a.run == b.run && a.cache == b.cache &&
        a.ownerGeneration == b.ownerGeneration && a.configTag == b.configTag &&
        a.mappingTag == b.mappingTag && a.translationGeneration == b.translationGeneration;
}
struct Source
{
    Scope scope{};
    std::uint64_t dispatch = 0ULL, epoch = 0ULL, segment = 0ULL;
    int sourcePC = -1;
    unsigned step = 0U;
};
// Filled only after the caller validates normal completion and the fresh,
// stopped physical boundary for this exact source. It is evidence, not a token.
struct Completion
{
    std::uint64_t tick = 0ULL, commit = 0ULL;
    bool boundaryValidated = false;
};
struct CompletedRow
{
    Line line{};
    Source source{};
    Completion completion{};
    double stationBegin = 0.0, stationEnd = 0.0;
};
enum class Reason : std::uint8_t
{ None, Scope, Geometry, Completion, Capacity, Sequence, Seam, Numeric, Lifecycle };

class CompletedLedger
{
public:
    bool Begin(const Scope& scope) noexcept
    {
        Invalidate();
        if (!ValidScope(scope) || scope.session <= retiredSession_) return Fail(Reason::Scope);
        retiredSession_ = scope.session; scope_ = scope;
        reason_ = Reason::None; valid_ = true; return true;
    }
    bool Append(const Line& line, const Source& source, const Completion& completion) noexcept
    {
        if (!valid_) return Fail(Reason::Lifecycle);
        if (!SameScope(source.scope, scope_) || !ValidScope(source.scope)) return Fail(Reason::Scope);
        if (count_ >= Capacity) return Fail(Reason::Capacity);
        if (!ValidLine(line)) return Fail(Reason::Geometry);
        if (!source.dispatch || !source.epoch ||
            source.epoch > (std::numeric_limits<std::uint32_t>::max)() ||
            !source.segment || source.sourcePC < 0 ||
            source.step != count_) return Fail(Reason::Sequence);
        if (!completion.boundaryValidated || !completion.tick || !completion.commit)
            return Fail(Reason::Completion);
        if (count_)
        {
            const auto& previous = rows_[count_ - 1U];
            if (source.dispatch <= previous.source.dispatch || source.segment <= previous.source.segment ||
                source.epoch <= previous.source.epoch || source.sourcePC <= previous.source.sourcePC ||
                completion.tick <= previous.completion.tick ||
                completion.commit <= previous.completion.commit) return Fail(Reason::Sequence);
            if (!SamePulse(previous.line.endPulse, line.startPulse) ||
                !SamePulse(previous.line.pulsePerNative, line.pulsePerNative)) return Fail(Reason::Seam);
        }
        const double next = lengthMm_ + line.lengthMm;
        if (!std::isfinite(next) || next <= lengthMm_) return Fail(Reason::Numeric);
        CompletedRow candidate{};
        candidate.line = line; candidate.source = source; candidate.completion = completion;
        candidate.stationBegin = lengthMm_; candidate.stationEnd = next;
        rows_[count_] = candidate; ++count_; lengthMm_ = next; return true;
    }
    void Invalidate(Reason reason = Reason::Lifecycle) noexcept
    { valid_ = false; count_ = 0U; lengthMm_ = 0.0; scope_ = Scope{}; reason_ = reason; }
    bool Valid() const noexcept { return valid_; }
    unsigned Count() const noexcept { return valid_ ? count_ : 0U; }
    double LengthMm() const noexcept { return valid_ ? lengthMm_ : 0.0; }
    const Scope& FrozenScope() const noexcept { return scope_; }
    Reason Failure() const noexcept { return reason_; }
    const CompletedRow* Row(unsigned ordinal) const noexcept
    { return valid_ && ordinal < count_ ? &rows_[ordinal] : nullptr; }
    constexpr bool PhysicalPermit() const noexcept { return false; }
private:
    bool Fail(Reason reason) noexcept { Invalidate(reason); return false; }
    std::array<CompletedRow, Capacity> rows_{};
    Scope scope_{};
    std::uint64_t retiredSession_ = 0ULL;
    double lengthMm_ = 0.0;
    unsigned count_ = 0U;
    Reason reason_ = Reason::None;
    bool valid_ = false;
};

struct Span
{
    unsigned ordinal = 0U; // zero-based ledger row
    double startMm = 0.0, endMm = 0.0; // local distances in that row
    bool stopRequired = true;
};
struct B2Plan
{
    std::array<Span, Capacity * 2U> spans{};
    unsigned count = 0U, anchorOrdinal = 0U, targetOrdinal = 0U;
    double anchorMm = 0.0, targetMm = 0.0; // global station, never live axis q
    Point anchorPulse{}, returnedPulse{};
    bool valid = false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
};

// Completed-only diagnostic plan. Traverse the last row backwards, retreat to
// the preceding row's midpoint, then retrace both spans to the exact anchor.
// Every span ends at a required stop. No NC PC, motion queue or live cursor is
// changed, and no plan authorizes replay of an old execution identity.
inline bool BuildB2Plan(const CompletedLedger& ledger, B2Plan& output) noexcept
{
    // Caller keeps this workspace outside the small NC/RT stack. Do not create
    // another whole plan as a local candidate or aggregate reset temporary.
    output.valid = false; output.count = 0U;
    output.anchorOrdinal = output.targetOrdinal = 0U;
    output.anchorMm = output.targetMm = 0.0;
    output.anchorPulse = Point{}; output.returnedPulse = Point{};
    for (auto& span : output.spans) span = Span{};
    if (!ledger.Valid() || ledger.Count() < 2U) return false;
    const unsigned last = ledger.Count() - 1U, previous = last - 1U;
    const auto* a = ledger.Row(last); const auto* b = ledger.Row(previous);
    if (!a || !b || !ValidLine(a->line) || !ValidLine(b->line) ||
        !SamePulse(b->line.endPulse, a->line.startPulse)) return false;
    const double midpoint = b->line.lengthMm * 0.5;
    if (!std::isfinite(midpoint) || midpoint <= 0.0 || midpoint >= b->line.lengthMm) return false;
    output.anchorOrdinal = last; output.targetOrdinal = previous;
    output.anchorMm = a->stationEnd; output.targetMm = b->stationBegin + midpoint;
    if (!std::isfinite(output.targetMm) || output.targetMm <= b->stationBegin ||
        output.targetMm >= b->stationEnd || output.targetMm >= output.anchorMm) return false;
    output.spans[0U] = Span{last, a->line.lengthMm, 0.0, true};
    output.spans[1U] = Span{previous, b->line.lengthMm, midpoint, true};
    output.spans[2U] = Span{previous, midpoint, b->line.lengthMm, true};
    output.spans[3U] = Span{last, 0.0, a->line.lengthMm, true};
    if (!EvaluateMm(a->line, a->line.lengthMm, output.anchorPulse) ||
        !EvaluateMm(a->line, output.spans[3U].endMm, output.returnedPulse) ||
        !SamePulse(output.anchorPulse, output.returnedPulse)) return false;
    output.count = 4U; output.valid = true; return true;
}

static_assert(std::is_trivially_copyable<Line>::value && sizeof(Line) <= 216U,
    "EDM109 fixed six-axis line budget");
static_assert(std::is_trivially_copyable<CompletedLedger>::value && sizeof(CompletedLedger) <= 5760U,
    "EDM109 fixed completed history budget");
static_assert(std::is_trivially_copyable<B2Plan>::value && sizeof(B2Plan) <= 1160U,
    "EDM109 fixed numerical plan budget");
}
