#pragma once
#include "EDMSixAxisGapTraversal.h"

// Explicit, unloaded RT test source only. This producer supplies voltage;
// qualification, signed servo demand, traversal and output commit are separate.
namespace EDM126
{
class TestVoltage
{
public:
    bool Begin(unsigned fixture) noexcept
    {
        *this = TestVoltage{};
        if (fixture > 1U) return Fail();
        begun_ = true; return true;
    }
    bool BeginRow(unsigned ordinal) noexcept
    {
        if (!begun_ || faulted_ || ordinal >= 16U ||
            (!rowValid_ && ordinal != 0U) ||
            (rowValid_ && (ordinal != ordinal_ + 1U || phase_ == 1U))) return Fail();
        ordinal_ = ordinal; rowValid_ = true; phase_ = 0U; return true;
    }
    double Voltage(const Feedback& f) noexcept
    {
        if (!begun_ || faulted_ || f.faulted) return Invalid();
        // Startup qualification precedes the first authenticated motion row.
        if (!rowValid_) return 50.0;
        if (!f.active || f.authorizedSource.ordinal != ordinal_ ||
            f.cursorSource.ordinal > f.authorizedSource.ordinal ||
            f.historyDepth != f.authorizedSource.ordinal - f.cursorSource.ordinal ||
            !std::isfinite(f.localMm) || !Positive(f.lengthMm) ||
            f.localMm < 0.0 || f.localMm > f.lengthMm) return Invalid();
        const bool selected = ordinal_ == 2U || ordinal_ == 4U || ordinal_ == 8U;
        if (!selected) return 50.0;
        if (phase_ == 0U && f.historyDepth == 0U && f.localMm >= f.lengthMm * 0.1) {
            if (f.historyCount < 2U) return Invalid();
            phase_ = 1U; ++retreats_;
        }
        if (phase_ == 1U && f.historyDepth >= 2U && f.localMm <= f.lengthMm * 0.8) {
            phase_ = 2U; ++returns_;
        }
        return phase_ == 1U ? 20.0 : 50.0;
    }
    bool Faulted() const noexcept { return faulted_; }
    unsigned Retreats() const noexcept { return retreats_; }
    unsigned Returns() const noexcept { return returns_; }
private:
    bool Fail() noexcept { faulted_ = true; return false; }
    double Invalid() noexcept
    { faulted_ = true; return (std::numeric_limits<double>::quiet_NaN)(); }
    unsigned ordinal_ = 0U, phase_ = 0U, retreats_ = 0U, returns_ = 0U;
    bool begun_ = false, rowValid_ = false, faulted_ = false;
};
}
