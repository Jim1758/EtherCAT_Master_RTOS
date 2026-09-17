#pragma once

#include <cmath>
#include <limits>

// Native planar geometry only. The caller has already transformed endpoints,
// scaled R into millimetres and adjusted direction for a reflected plane.
// R > 0 selects the minor arc; R < 0 selects the major arc. A coincident
// endpoint cannot distinguish the centre of an R-format full circle.
// Failure leaves both caller-owned offsets unchanged.
inline bool TryResolveNCPathRadiusArcCenter(
    const double sx, const double sy, const double ex, const double ey,
    const double signedRadiusMM, const int direction,
    double& offsetX, double& offsetY) noexcept
{
    constexpr double RoundoffFactor = 64.0 * std::numeric_limits<double>::epsilon();
    constexpr double MaximumBudgetMM = 1.0e-7;
    constexpr double Pi = 3.141592653589793238462643383279502884;
    constexpr double TwoPi = 2.0 * Pi;
    if ((direction != -1 && direction != 1) || &offsetX == &offsetY ||
        !std::isfinite(sx) || !std::isfinite(sy) ||
        !std::isfinite(ex) || !std::isfinite(ey) ||
        !std::isfinite(signedRadiusMM) || signedRadiusMM == 0.0)
        return false;

    const double radius = std::fabs(signedRadiusMM);
    double magnitude = radius;
    const double coordinates[4] = { sx, sy, ex, ey };
    for (const double coordinate : coordinates)
        if (std::fabs(coordinate) > magnitude) magnitude = std::fabs(coordinate);
    double budget = RoundoffFactor * magnitude;
    if (!std::isfinite(budget) || budget <= 0.0 || budget > MaximumBudgetMM ||
        radius <= budget)
        return false;

    const double dx = ex - sx, dy = ey - sy;
    if (!std::isfinite(dx) || !std::isfinite(dy)) return false;
    const double chord = std::hypot(dx, dy);
    if (!std::isfinite(chord) || chord <= budget) return false;
    const double half = 0.5 * chord;
    if (half <= 0.0) return false;

    // A diameter excess may only consume the same native coordinate roundoff
    // budget as ordinary arc validation. Never clamp a real undersized R.
    double height = 0.0;
    if (half > radius)
    {
        const double excess = 2.0 * (half - radius);
        if (!std::isfinite(excess) || excess > budget) return false;
    }
    else
    {
        // Factored square roots avoid squaring radius, underflow in r*r and
        // cancellation from 1 - (half/r)^2 near a semicircle.
        height = std::sqrt(radius - half) * std::sqrt(radius + half);
        if (!std::isfinite(height)) return false;
    }

    const double side = static_cast<double>(direction) *
        (signedRadiusMM > 0.0 ? 1.0 : -1.0);
    const double candidateX = 0.5 * dx - side * height * (dy / chord);
    const double candidateY = 0.5 * dy + side * height * (dx / chord);
    const double cx = sx + candidateX, cy = sy + candidateY;
    if (!std::isfinite(candidateX) || !std::isfinite(candidateY) ||
        !std::isfinite(cx) || !std::isfinite(cy)) return false;
    if (std::fabs(cx) > magnitude) magnitude = std::fabs(cx);
    if (std::fabs(cy) > magnitude) magnitude = std::fabs(cy);
    budget = RoundoffFactor * magnitude;
    if (!std::isfinite(budget) || budget > MaximumBudgetMM) return false;

    // Check the centre actually representable at this native origin, not just
    // the ideal local construction. The downstream pulse-space guard remains
    // authoritative for the complete NC / Motion conversion.
    const double startRadius = std::hypot(sx - cx, sy - cy);
    const double endRadius = std::hypot(ex - cx, ey - cy);
    if (!std::isfinite(startRadius) || !std::isfinite(endRadius) ||
        startRadius <= budget || endRadius <= budget ||
        std::fabs(startRadius - radius) > budget ||
        std::fabs(endRadius - radius) > budget ||
        std::fabs(startRadius - endRadius) > budget)
        return false;
    const double startAngle = std::atan2(sy - cy, sx - cx);
    const double endAngle = std::atan2(ey - cy, ex - cx);
    double sweep = endAngle - startAngle;
    if (direction == 1 && sweep <= 0.0) sweep += TwoPi;
    if (direction == -1 && sweep >= 0.0) sweep -= TwoPi;
    const double travel = std::fabs(sweep);
    if (!std::isfinite(travel) || travel < 1.0e-12 || travel >= TwoPi)
        return false;
    // Semicircles can straddle pi by rounding only. A branch reversal with
    // meaningful arc-length error must never change minor into major or back.
    const double wrongSide = signedRadiusMM > 0.0 ? travel - Pi : Pi - travel;
    if (wrongSide > 0.0 && wrongSide * radius > budget) return false;

    offsetX = candidateX;
    offsetY = candidateY;
    return true;
}
