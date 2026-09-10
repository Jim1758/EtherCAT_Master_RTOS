#pragma once

#include "NCPathCoreFeedLine.h"
#include "NCPathCoreFeedArc.h"
#include <array>
#include <cstdint>
#include <type_traits>

// BZ: immutable canonical geometry. No execution authority or provenance is
// stored here. All calls are single-threaded, bounded and allocation-free.
enum class NCPathCoreRetainedKind : std::uint8_t { NONE = 0U, LINE = 1U, ARC = 2U };

struct NCPathCoreRetainedGeometry
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    std::array<double, 2U> centerMCS{}, centerPulse{};
    std::array<double, 2U> boundsMinMCS{}, boundsMaxMCS{};
    double radiusMM = 0.0, radiusPulse = 0.0;
    double startAngle = 0.0, sweepRadians = 0.0;
    double lengthMM = 0.0, lengthPulse = 0.0;
    std::uint32_t axisMask = 0U;
    int direction = 0;
    NCPathCoreRetainedKind kind = NCPathCoreRetainedKind::NONE;
    bool fullCircle = false, point = false, valid = false;
    void Clear() noexcept;
};

bool BuildNCPathCoreRetainedLine(const NCPathCoreFeedLineV2& source,
    NCPathCoreRetainedGeometry& output) noexcept;
bool BuildNCPathCoreRetainedArc(const NCPathCoreFeedArcV2& source,
    NCPathCoreRetainedGeometry& output) noexcept;
bool IsNCPathCoreRetainedGeometryValid(const NCPathCoreRetainedGeometry& geometry) noexcept;

// u is the ORIGINAL canonical parameter, regardless of traversal direction.
// Exact endpoint copies; no bitwise equivalence of distinct runtime ticks is
// claimed. Callers use u = reverse ? 1-progress : progress for traversal.
bool EvaluateNCPathCoreRetainedAxisCanonical(const NCPathCoreRetainedGeometry& geometry,
    std::uint32_t axis, double u, double& outputMCS) noexcept;
bool EvaluateNCPathCoreRetainedPulseCanonical(const NCPathCoreRetainedGeometry& geometry,
    std::uint32_t axis, double u, double& outputPulse) noexcept;

// Both coordinate-space errors must fit 64 eps times that axis's canonical
// geometry magnitude; their combined budget must not exceed 1e-7 mm. No
// geometry is changed to bridge a seam. The caller supplies current positive
// pulse/mm for every valid native axis (held axes use their native units).
bool AreNCPathCoreRetainedEndpointsConnected(const NCPathCoreRetainedGeometry& previous,
    const NCPathCoreRetainedGeometry& next, std::uint32_t validAxisMask,
    const std::array<double, 8U>& pulsePerMM) noexcept;
bool DoesNCPathCoreRetainedStartMatch(const NCPathCoreRetainedGeometry& geometry,
    bool reverse, const std::array<double, 8U>& actualMCS,
    const std::array<double, 8U>& actualPulse, std::uint32_t validAxisMask,
    const std::array<double, 8U>& pulsePerMM) noexcept;

// CA: start matching at an original source parameter; geometry is never trimmed.
bool DoesNCPathCoreRetainedStartMatchAt(const NCPathCoreRetainedGeometry& geometry,
    double u, const std::array<double, 8U>& actualMCS,
    const std::array<double, 8U>& actualPulse, std::uint32_t validAxisMask,
    const std::array<double, 8U>& pulsePerMM) noexcept;

enum class NCPathCoreRetainedFault : std::uint8_t
{
    NONE = 0U, INVALID_GEOMETRY = 1U, DISCONTINUITY = 2U,
    CAPACITY = 3U, INTERRUPTED = 4U, LIFECYCLE = 5U
};
enum class NCPathCoreRetainedCursorState : std::uint8_t
{
    IDLE = 0U, RETREATING = 1U, AT_START = 2U, ADVANCING = 3U, RETURNED = 4U
};

class NCPathCoreRetainedPath
{
public:
    static constexpr std::uint32_t Capacity = 16U;
    // Reset invalidates all previously borrowed rows. Clear/fail never creates
    // a full-size aggregate temporary or silently retains a truncated suffix.
    void Clear(NCPathCoreRetainedFault reason = NCPathCoreRetainedFault::NONE) noexcept;
    void Fail(NCPathCoreRetainedFault reason) noexcept;
    bool Append(const NCPathCoreRetainedGeometry& geometry, std::uint32_t validAxisMask,
        const std::array<double, 8U>& pulsePerMM) noexcept;
    const NCPathCoreRetainedGeometry* Get(std::uint32_t index) const noexcept;
    bool BeginRetreat(std::uint32_t requested) noexcept;
    bool SelectStep(bool forward, const NCPathCoreRetainedGeometry*& geometry,
        std::uint32_t& ordinal) noexcept;
    // CA distance sessions are separate from the legacy full-row session.
    // One selection stays within one row. Early reversal is allowed; the
    // committed ordinal/u changes only after MarkSubmitted + CompleteStep.
    bool BeginDistanceRetreat(std::uint32_t requested) noexcept;
    bool SelectDistanceStep(bool forward, double maxDistanceMM,
        const NCPathCoreRetainedGeometry*& geometry, std::uint32_t& ordinal,
        double& startU, double& endU) noexcept;
    bool DistanceMode() const noexcept { return m_distanceMode; }
    std::uint32_t CurrentOrdinal() const noexcept { return m_currentOrdinal; }
    double CurrentU() const noexcept { return m_currentU; }
    double SelectedStartU() const noexcept { return m_selectedStartU; }
    double SelectedEndU() const noexcept { return m_selectedEndU; }
    bool MarkSubmitted() noexcept;
    bool CompleteStep() noexcept;
    std::uint32_t Count() const noexcept { return m_count; }
    std::uint32_t Position() const noexcept { return m_position; }
    std::uint32_t LowerBound() const noexcept { return m_lower; }
    std::uint32_t Requested() const noexcept { return m_requested; }
    std::uint32_t ValidAxisMask() const noexcept { return m_validMask; }
    bool Pending() const noexcept { return m_pending; }
    NCPathCoreRetainedFault Fault() const noexcept { return m_fault; }
    NCPathCoreRetainedCursorState State() const noexcept { return m_state; }
private:
    std::array<NCPathCoreRetainedGeometry, Capacity> m_rows{};
    std::uint32_t m_count = 0U, m_position = 0U, m_lower = 0U;
    std::uint32_t m_requested = 0U, m_validMask = 0U;
    NCPathCoreRetainedFault m_fault = NCPathCoreRetainedFault::NONE;
    NCPathCoreRetainedCursorState m_state = NCPathCoreRetainedCursorState::IDLE;
    bool m_selected = false, m_pending = false, m_forward = false;
    bool m_distanceMode = false;
    std::uint32_t m_currentOrdinal = 0U, m_selectedOrdinal = 0U;
    double m_currentU = 0.0, m_selectedStartU = 0.0, m_selectedEndU = 0.0;
};

static_assert(sizeof(NCPathCoreRetainedGeometry) == 384U,
    "BZ canonical geometry fixed storage budget changed.");
static_assert(sizeof(NCPathCoreRetainedPath) == 6208U,
    "CA retained path fixed storage budget changed.");
static_assert(std::is_standard_layout<NCPathCoreRetainedGeometry>::value&&
    std::is_trivially_copyable<NCPathCoreRetainedGeometry>::value,
    "BZ canonical geometry must remain a plain value.");
