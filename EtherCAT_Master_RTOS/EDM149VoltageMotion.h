#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// Voltage-only commissioning adapter. It has no Motion/IO handle and cannot
// advance a cursor, claim settlement, authorize a row, or publish motor output.
namespace EDM149
{
constexpr unsigned ZHistoryFixture = 2U, XYZHistoryFixture = 3U, SixHistoryFixture = 4U, ArcHistoryFixture = 5U, OrderHistoryFixture = 6U, SparseHistoryFixture = 7U, SparseArcHistoryFixture = 8U, ProgramHistoryFixture = 9U, PauseHistoryFixture = 10U, VisibleFlushHistoryFixture = 11U, XYZVisibleFlushHistoryFixture = 12U, SixVisibleFlushHistoryFixture = 13U, G76GapHistoryFixture = 14U;
inline bool IsZTestArm(unsigned arm) noexcept
{ return arm == 230U || arm == 232U || arm == 234U; }
inline bool IsXYZTestArm(unsigned arm) noexcept
{ return arm == 236U || arm == 238U || arm == 240U; }
inline bool IsSixTestArm(unsigned arm) noexcept
{ return arm == 242U || arm == 244U || arm == 246U; }
inline bool IsArcTestArm(unsigned arm) noexcept
{ return arm == 248U || arm == 250U || arm == 252U; }
inline bool IsOrderTestArm(unsigned arm) noexcept
{ return arm == 254U || arm == 256U || arm == 258U; }
inline bool IsSparseTestArm(unsigned arm) noexcept
{ return arm == 260U || arm == 262U || arm == 264U; }
inline bool IsSparseArcTestArm(unsigned arm) noexcept
{ return arm == 266U || arm == 268U || arm == 270U; }
inline bool IsSparseArcTestFinish(unsigned finish) noexcept
{ return finish == 267U || finish == 269U || finish == 271U; }
inline bool IsG76GapTestArm(unsigned arm) noexcept
{ return arm == 292U || arm == 294U || arm == 296U || arm == 298U || arm == 300U; }
inline bool IsManualVoltageTestArm(unsigned arm) noexcept
{ return arm == 290U; }
inline bool IsProgramTestArm(unsigned arm) noexcept
{ return arm == 272U || arm == 274U || arm == 276U || IsManualVoltageTestArm(arm); }
inline bool IsPauseTestArm(unsigned arm) noexcept
{ return arm == 278U || arm == 280U || arm == 282U; }
inline bool IsVisibleFlushTestArm(unsigned arm) noexcept
{ return arm == 284U; }
inline bool IsXYZVisibleFlushTestArm(unsigned arm) noexcept
{ return arm == 286U; }
inline bool IsSixVisibleFlushTestArm(unsigned arm) noexcept
{ return arm == 288U; }
// EDM166/167 reuse the existing E5 pause transaction for the 2 mm fixtures.
inline bool IsFlushHoldTestArm(unsigned arm) noexcept
{ return IsProgramTestArm(arm) || IsXYZVisibleFlushTestArm(arm) || IsSixVisibleFlushTestArm(arm); }
inline bool IsBoundedProgramTestArm(unsigned arm) noexcept
{ return IsProgramTestArm(arm) || IsPauseTestArm(arm) || IsVisibleFlushTestArm(arm) || IsXYZVisibleFlushTestArm(arm) || IsSixVisibleFlushTestArm(arm); }
inline bool IsProgramHistoryFixture(unsigned fixture) noexcept
{ return fixture == ProgramHistoryFixture || fixture == PauseHistoryFixture || fixture == VisibleFlushHistoryFixture || fixture == XYZVisibleFlushHistoryFixture || fixture == SixVisibleFlushHistoryFixture || fixture == G76GapHistoryFixture; }
inline unsigned SelectedRetreatCount(unsigned rows) noexcept
{ return rows <= 16U ? (rows + 1U) / 4U : 0U; }
inline bool IsSparseLineHistoryFixture(unsigned fixture) noexcept
{ return fixture == SparseHistoryFixture || fixture == SparseArcHistoryFixture || IsProgramHistoryFixture(fixture); }
inline bool IsTestArm(unsigned arm) noexcept
{ return IsG76GapTestArm(arm) || IsZTestArm(arm) || IsXYZTestArm(arm) || IsSixTestArm(arm) || IsArcTestArm(arm) || IsOrderTestArm(arm) || IsSparseTestArm(arm) || IsSparseArcTestArm(arm) || IsBoundedProgramTestArm(arm); }
inline unsigned MovementMaskForArm(unsigned arm) noexcept
{ return (IsG76GapTestArm(arm) || IsXYZTestArm(arm) || IsXYZVisibleFlushTestArm(arm)) ? 7U : IsZTestArm(arm) ? 4U : 63U; }
inline bool StaticAxis(bool zOnly, bool xyzOnly, unsigned axis) noexcept
{ return (zOnly && axis != 2U) || (xyzOnly && axis >= 3U); }
inline unsigned SourceForArm(unsigned arm) noexcept
{ return IsG76GapTestArm(arm) ? 3U : IsManualVoltageTestArm(arm) ? 5U : (arm == 230U || arm == 236U || arm == 242U || arm == 248U || arm == 254U || arm == 260U || arm == 266U || arm == 272U || arm == 278U) ? 2U : (arm == 232U || arm == 238U || arm == 244U || arm == 250U || arm == 256U || arm == 262U || arm == 268U || arm == 274U || arm == 280U || arm == 284U || arm == 286U || arm == 288U) ? 3U : (arm == 234U || arm == 240U || arm == 246U || arm == 252U || arm == 258U || arm == 264U || arm == 270U || arm == 276U || arm == 282U) ? 4U : 0U; }
inline bool IsTestSource(unsigned source) noexcept
{ return source >= 2U && source <= 5U; }
inline bool IsManualSource(unsigned source) noexcept
{ return source == 5U; }
inline bool IsSoftwareSource(unsigned source) noexcept
{ return source == 2U || source == 3U || source == 5U; }
inline bool ValidHistoryBinding(unsigned fixture, unsigned source) noexcept
{ return source == 5U ? fixture == ProgramHistoryFixture : (fixture == VisibleFlushHistoryFixture || fixture == XYZVisibleFlushHistoryFixture || fixture == SixVisibleFlushHistoryFixture || fixture == G76GapHistoryFixture) ? source == 3U : (fixture == ZHistoryFixture || fixture == XYZHistoryFixture || fixture == SixHistoryFixture || fixture == ArcHistoryFixture || fixture == OrderHistoryFixture || IsSparseLineHistoryFixture(fixture)) ? IsTestSource(source) : fixture < 2U && source <= 1U; }
inline const char* SourceName(unsigned source) noexcept
{ return source == 2U ? "SOFTWARE_STEPS" : source == 3U ? "SOFTWARE_WAVE" :
    source == 4U ? "PHYSICAL_AD_MOTION_ONLY" : source == 5U ? "SOFTWARE_MANUAL" : "LEGACY"; }
inline bool ValidZIncrement(const std::array<double, 6U>& delta, double feed) noexcept
{
    if (feed != 0.12 || !std::isfinite(delta[2]) ||
        std::fabs(delta[2]) < 0.001 || std::fabs(delta[2]) > 0.002) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        if (axis != 2U && delta[axis] != 0.0) return false;
    return true;
}

// EDM150 keeps the three rotary references fixed in the same six-axis packet.
inline bool ValidXYZIncrement(const std::array<double, 6U>& delta, double feed) noexcept
{
    if (feed != 0.12) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        if (!std::isfinite(delta[axis]) || (axis < 3U ?
            (std::fabs(delta[axis]) < 0.001 || std::fabs(delta[axis]) > 0.002) : delta[axis] != 0.0)) return false;
    return true;
}

// EDM154 is an authored G91 line with at least one linear axis moving.
// Omitted/zero axes have no displacement on this row. Historical traversal
// and flush motion still use their own retained six-axis geometry.
inline bool ValidSparseIncrement(const std::array<double, 6U>& delta, double feed) noexcept
{
    if (feed != 0.12) return false;
    bool linear = false;
    for (unsigned axis = 0U; axis < 6U; ++axis) {
        if (!std::isfinite(delta[axis])) return false;
        if (delta[axis] == 0.0) continue;
        const double magnitude = std::fabs(delta[axis]);
        if (magnitude < (axis < 3U ? 0.001 : 0.020) ||
            magnitude > (axis < 3U ? 0.002 : 0.040)) return false;
        if (axis < 3U) linear = true;
    }
    return linear;
}

class Source
{
public:
    bool Begin(std::uint64_t session, unsigned source, double referenceV) noexcept
    {
        *this = Source{};
        if (!session || !IsSoftwareSource(source) || !std::isfinite(referenceV) ||
            referenceV < 15.0 || referenceV > 200.0) return Fail();
        session_ = session; source_ = source; referenceV_ = manualVoltageV_ = referenceV; enabled_ = true;
        return true;
    }
    // RT-owned setpoint only. The scoped receiver owns request freshness.
    bool SetManualVoltage(std::uint64_t session,double voltageV) noexcept
    {
        if(!enabled_||faulted_||source_!=5U||session!=session_||
            !std::isfinite(voltageV)||voltageV<0.0||voltageV>300.0)return false;
        manualVoltageV_=voltageV;return true;
    }
    bool Enabled() const noexcept { return enabled_; }
    bool BeginCompound(std::uint64_t session, unsigned source, double referenceV, unsigned count) noexcept
    {
        if (source != 3U || (count != 25U && count != 26U)) return Fail();
        if (!Begin(session,source,referenceV)) return false;
        compoundCount_ = count; return true;
    }
    bool Faulted() const noexcept { return faulted_; }
    bool Matches(std::uint64_t session, unsigned source, double referenceV) const noexcept
    { return enabled_ && !faulted_ && session_ == session && source_ == source && referenceV_ == referenceV; }
    bool BeginRow(unsigned row) noexcept
    {
        if (!enabled_ || faulted_ || row >= (compoundCount_ ? compoundCount_ : 16U) || (!rowValid_ && row != 0U) ||
            (rowValid_ && (row != row_ + 1U || (source_ != 5U && Selected(row_) && phase_ != 3U)))) return Fail();
        row_ = row; rowValid_ = true; phase_ = 0U; retreatTicks_ = 0U;
        anchorStationMm_ = 0.0; return true;
    }
    template<class Traversal, class Coordination>
    double Voltage(const Traversal& f, const Coordination& coordination,
        std::uint64_t tick, std::uint64_t nowUs, bool freezeProgress = false) noexcept
    {
        if (!enabled_ || faulted_ || !tick || !nowUs ||
            (tick_ && (tick != tick_ + 1ULL || nowUs <= nowUs_))) return Invalid();
        tick_ = tick; nowUs_ = nowUs;
        if (!rowValid_) return source_==5U?manualVoltageV_:High(tick);
        if (!f.active || f.faulted || coordination.faulted ||
            f.authorizedSource.scope.session != session_ || f.cursorSource.scope.session != session_ ||
            f.authorizedSource.ordinal != row_ || f.cursorSource.ordinal > row_ ||
            f.historyDepth != row_ - f.cursorSource.ordinal || !f.tick || f.tick > tick ||
            !f.nowUs || f.nowUs > nowUs || !std::isfinite(f.localMm) ||
            !std::isfinite(f.lengthMm) || f.lengthMm <= 0.0 || f.localMm < 0.0 ||
            f.localMm > f.lengthMm || !std::isfinite(f.stationMm) || f.stationMm < 0.0) return Invalid();
        if(source_==5U)return manualVoltageV_;
        using Phase = decltype(coordination.phase);
        // A scoped machining HOLD preserves the stimulus phase while source
        // freshness, noise and actual voltage sample accounting keep advancing.
        if (!freezeProgress && Selected(row_) && coordination.phase == Phase::Gap)
        {
            if (phase_ == 0U && f.historyDepth == 0U && f.localMm >= 0.1 * f.lengthMm)
            {
                if (!f.historyCount) return Invalid();
                anchorStationMm_ = f.stationMm; phase_ = 1U; ++retreats_;
            }
            if (phase_ == 1U)
            {
                ++retreatTicks_;
                if (f.historyDepth == 1U && f.localMm <= 0.8 * f.lengthMm)
                { phase_ = 2U; ++returns_; }
            }
            if (phase_ == 2U && f.historyDepth == 0U && f.stationMm >= anchorStationMm_)
                phase_ = 3U;
        }
        // Flush owns the demand while lifting/returning. Keep its independent
        // voltage input healthy; the provider never treats a lift as GAP work.
        if (coordination.phase != Phase::Gap || phase_ != 1U) return High(tick);
        // A 40 ms zero-voltage short enters the existing short detector. It is
        // followed by low voltage until the real committed cursor retreats.
        if (retreatTicks_ <= 160U) { ++shortSamples_; return 0.0; }
        const double low = referenceV_ > 20.0 ? referenceV_ - 20.0 : 0.0;
        return low + (source_ == 3U && low > 2.0 ? Noise(tick) * 0.5 : 0.0);
    }
    unsigned Retreats() const noexcept { return retreats_; }
    unsigned Returns() const noexcept { return returns_; }
    std::uint64_t ShortSamples() const noexcept { return shortSamples_; }
private:
    static bool Selected(unsigned row) noexcept
    { return row == 2U || row == 6U || row == 10U || row == 14U; }
    static double Noise(std::uint64_t tick) noexcept
    { return static_cast<double>(static_cast<int>(tick % 33ULL) - 16) * 0.125; }
    double High(std::uint64_t tick) const noexcept
    { return referenceV_ + 40.0 + (source_ == 3U ? Noise(tick) : 0.0); }
    bool Fail() noexcept { faulted_ = true; return false; }
    double Invalid() noexcept
    { (void)Fail(); return (std::numeric_limits<double>::quiet_NaN)(); }
    std::uint64_t session_ = 0ULL, tick_ = 0ULL, nowUs_ = 0ULL, shortSamples_ = 0ULL;
    double referenceV_ = 0.0, anchorStationMm_ = 0.0, manualVoltageV_ = 0.0;
    unsigned source_ = 0U, row_ = 0U, phase_ = 0U, retreatTicks_ = 0U, retreats_ = 0U, returns_ = 0U;
    unsigned compoundCount_ = 0U;
    bool enabled_ = false, faulted_ = false, rowValid_ = false;
};
}
