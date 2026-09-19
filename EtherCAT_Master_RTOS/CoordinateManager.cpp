#include "CoordinateManager.h"
#include "NCManager.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <limits>
#include "GlobalConfig.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

namespace
{
bool TryGetFixedWorkPlaneAngle(const std::vector<double>& row, int plane,
    double& angle) noexcept
{
    angle = 0.0;
    if (row.size() != 8U || !IsNCArcPlaneCode(plane)) return false;
    const unsigned selected = plane == 18 ? CoordinateManager::WO_ANGLE_XZ_PITCH :
        (plane == 19 ? CoordinateManager::WO_ANGLE_YZ_ROLL :
            CoordinateManager::WO_ANGLE_XY_YAW);
    for (unsigned field = CoordinateManager::WO_ANGLE_XY_YAW;
        field <= CoordinateManager::WO_ANGLE_YZ_ROLL; ++field)
    {
        if (!std::isfinite(row[field]) || std::fabs(row[field]) > 360.0) return false;
        if (field != selected && row[field] != 0.0) return false;
    }
    if (row[CoordinateManager::WO_RESERVED_6] != 0.0 ||
        row[CoordinateManager::WO_RESERVED_7] != 0.0) return false;
    angle = row[selected];
    return true;
}
}

CoordinateManager::CoordinateManager()
{
    // 初始化表格大小 (8軸)
    m_WCSTable.resize(60, std::vector<double>(8, 0.0));
    m_ToolOffset.resize(100, std::vector<double>(8, 0.0));
    m_ToolRadius.resize(100, std::vector<double>(8, 0.0));
    m_WorkOffset.resize(100, std::vector<double>(8, 0.0));
    m_RefPoints.resize(100, std::vector<double>(8, 0.0));

    // 2. 開機自動載入所有檔案

    LoadAllParameters();

    // 🌟 開機預設為 G49 關閉刀長補正
    toolLengthMode = 49;
    currentHCode = 0;

}

bool CoordinateManager::ConfigureElectrodeRotationAxis(int oneBasedAxis,
    const std::vector<AxisContext>& axes, NCManager* nc)
{
    // This setting is immutable after startup, including unchanged requests.
    // RESET is a modal reset, never authorization to replace the axis role.
    if (m_electrodeRotationConfigured || m_translationRunToken != 0ULL ||
        m_translationFrozen || m_translationResetBypass ||
        m_translationGenerationCounter != 0ULL ||
        (nc != nullptr && nc->GetState() != NCState::IDLE))
        return RejectCoordinateMutation("ELECTRODE_ROLE", "STARTUP_ONLY", nc, true);
    if (oneBasedAxis != 0 && (oneBasedAxis < 4 || oneBasedAxis > 8))
        return RejectCoordinateMutation("ELECTRODE_ROLE", "AXIS_NUMBER", nc, true);
    const int index = oneBasedAxis == 0 ? -1 : oneBasedAxis - 1;
    int type = -1;
    if (index >= 0)
    {
        if (static_cast<std::size_t>(index) >= axes.size() ||
            !axes[index].isExist || axes[index].axisIndex != index ||
            (axes[index].axisType != AxisType::ROTARY &&
             axes[index].axisType != AxisType::ROTARY_CONTINUOUS))
            return RejectCoordinateMutation("ELECTRODE_ROLE", "EXISTING_ROTARY_AXIS_REQUIRED", nc, true);
        type = static_cast<int>(axes[index].axisType);
    }
    if (!GuardCoordinateMutation("ELECTRODE_ROLE", nc)) return false;
    m_electrodeRotationAxes = &axes;
    m_translationAxisSource = nc;
    m_electrodeRotationAxisIndex = index;
    m_electrodeRotationAxisType = type;
    m_electrodeRotationConfigured = true;
    return true;
}

int CoordinateManager::GetElectrodeRotationAxisIndex() const noexcept
{
    const int index = m_electrodeRotationAxisIndex;
    if (!m_electrodeRotationConfigured || index < 3 || index >= 8 ||
        m_electrodeRotationAxes == nullptr ||
        static_cast<std::size_t>(index) >= m_electrodeRotationAxes->size()) return -1;
    const AxisContext& axis = (*m_electrodeRotationAxes)[index];
    if (!axis.isExist || axis.axisIndex != index ||
        static_cast<int>(axis.axisType) != m_electrodeRotationAxisType ||
        (axis.axisType != AxisType::ROTARY &&
         axis.axisType != AxisType::ROTARY_CONTINUOUS)) return -1;
    return index;
}

// Native rotary coordinates are degrees. No unit conversion, WCS subtraction,
// angle wrapping, or configured letter lookup belongs at this read boundary.
double CoordinateManager::GetElectrodeRotationAngleMCS(const double* nativeMCS) const noexcept
{
    const int index = GetElectrodeRotationAxisIndex();
    if (nativeMCS == nullptr || index < 0 || !std::isfinite(nativeMCS[index]))
        return (std::numeric_limits<double>::quiet_NaN)();
    return nativeMCS[index];
}

bool CoordinateManager::IsElectrodeOffsetRotationRequested() const noexcept
{
    if (!isCAxisOffsetRotationEnabled ||
        GlobalConfig::GetInstance().systemMode != SystemMode::EDM_SINKER_MODE ||
        (toolLengthMode != 43 && toolLengthMode != 44)) return false;
    // A malformed selected H is unsupported too; never let a partial legacy
    // transform commit NaN values before discovering the invalid table row.
    if (!IsToolOffsetRowValid(currentHCode, false)) return true;
    const std::vector<double>& row = m_ToolOffset[currentHCode - 1];
    return row[0] != 0.0 || row[1] != 0.0;
}

bool CoordinateManager::TryBuildAxisIdentity(const NCManager* nc,
    NCAxisIdentitySnapshot& identity) const noexcept
{
    if (nc == nullptr || !m_electrodeRotationConfigured ||
        m_electrodeRotationAxes == nullptr ||
        (m_electrodeRotationAxisIndex >= 0 && GetElectrodeRotationAxisIndex() < 0)) return false;
    char addresses[8]{};
    for (int axis = 0; axis < 8; ++axis)
    {
        const char name = nc->GetAxisName(axis);
        addresses[axis] = name == '?' || name == ' ' ? '\0' : name;
    }
    return TryCaptureNCAxisIdentitySnapshot(*m_electrodeRotationAxes, addresses,
        m_electrodeRotationAxisIndex, static_cast<int>(GlobalConfig::GetInstance().systemMode),
        isCAxisOffsetRotationEnabled, identity);
}

bool CoordinateManager::IsTranslationAxisIdentityCurrent() const noexcept
{
    if (m_translationRunToken == 0ULL) return false;
    NCAxisIdentitySnapshot live{};
    if (!TryBuildAxisIdentity(m_translationAxisSource, live)) return false;
    NCAxisIdentitySnapshot expected = m_runAxisIdentity;
    // G162/G163 is a guarded modal selection, not a new physical axis layout.
    // Once frozen, the full coordinate snapshot also compares this switch.
    expected.eccentricEnabled = live.eccentricEnabled;
    return SameNCAxisIdentitySnapshot(live, expected);
}

bool CoordinateManager::BeginTranslationRun(std::uint64_t runToken, NCManager* nc) noexcept
{
    if (runToken == 0ULL || m_translationRunToken != 0ULL ||
        m_translationFrozen || m_translationGenerationCounter ==
            (std::numeric_limits<std::uint64_t>::max)() ||
        (m_translationAxisSource != nullptr && m_translationAxisSource != nc)) return false;
    NCAxisIdentitySnapshot identity{};
    if (!TryBuildAxisIdentity(nc, identity)) return false;
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationAxisSource = nc;
    m_runAxisIdentity = identity;
    m_translationRunToken = runToken;
    m_translationGeneration = ++m_translationGenerationCounter;
    m_translationResetBypass = false;
    return true;
}

bool CoordinateManager::IsToolOffsetRowValid(int hCode, bool xyzOnly) const noexcept
{
    if (hCode < 1 || hCode > 100 ||
        static_cast<std::size_t>(hCode) > m_ToolOffset.size()) return false;
    const std::vector<double>& row = m_ToolOffset[hCode - 1];
    if (row.size() != 8U) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (!std::isfinite(row[axis]) ||
            (xyzOnly && axis >= 3U && row[axis] != 0.0)) return false;
    return true;
}

bool CoordinateManager::IsToolLengthSelectionSupported(int normalizedMode,
    int normalizedH) const noexcept
{
    if (normalizedMode == 49) return normalizedH == 0;
    // The bounded XYZ path requires G163 before any active H selection,
    // even for a zero XY row. Reject here before the selector can be committed.
    return (normalizedMode == 43 || normalizedMode == 44) &&
        (m_translationRunToken == 0ULL || !isCAxisOffsetRotationEnabled) &&
        IsToolOffsetRowValid(normalizedH, true);
}

bool CoordinateManager::IsWorkOffsetRowValid(int wCode, bool fixedPlanar) const noexcept
{
    if (wCode < 1 || wCode > 100 ||
        static_cast<std::size_t>(wCode) > m_WorkOffset.size()) return false;
    const std::vector<double>& row = m_WorkOffset[wCode - 1];
    if (row.size() != 8U) return false;
    for (unsigned field = 0U; field < 8U; ++field)
        if (!std::isfinite(row[field])) return false;
    if (fixedPlanar)
    {
        for (unsigned field = WO_ANGLE_XY_YAW; field <= WO_ANGLE_YZ_ROLL; ++field)
            if (std::fabs(row[field]) > 360.0) return false;
        if (row[WO_RESERVED_6] != 0.0 || row[WO_RESERVED_7] != 0.0) return false;
    }
    return true;
}

bool CoordinateManager::IsWorkpieceSelectionSupported(int normalizedMode,
    int normalizedW) const noexcept
{
    if (normalizedMode == 169) return normalizedW == 0;
    if (normalizedMode != 168 || !IsWorkOffsetRowValid(normalizedW, true)) return false;
    // BASE-PLANE-7 binds one WORK rotation component to the active plane's
    // positive normal: G17 yaw, G18 pitch, G19 roll. Cross-plane/multi-angle
    // rows remain explicit rejects; the unbound legacy 3D matrix is unchanged.
    if (!IsTranslationRunBound()) return true;
    double angle = 0.0;
    return TryGetFixedWorkPlaneAngle(m_WorkOffset[normalizedW - 1], activePlane, angle);
}

bool CoordinateManager::IsFixedPlanarRotationActive() const noexcept
{
    if (isG68Active) return true;
    if (!isWorkpieceRotationActive || !IsWorkOffsetRowValid(currentWCode, true)) return false;
    double angle = 0.0;
    return TryGetFixedWorkPlaneAngle(m_WorkOffset[currentWCode - 1], activePlane, angle) &&
        angle != 0.0;
}

bool CoordinateManager::IsScaleMirrorActive() const noexcept
{
    if (isScalingActive) return true;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (isMirrorActive[axis]) return true;
    return false;
}

bool CoordinateManager::IsBaseArcPlaneSelectionSupported(int plane) const noexcept
{
    if (!IsNCArcPlaneCode(plane)) return false;
    if (plane == 17) return true; // Preserve every accepted G17 combination.
    if ((toolRadiusMode == 40 ? currentDCode != 0 :
            (!IsToolRadiusSelectionSupported(toolRadiusMode, currentDCode) ||
                !IsNCTranslationCutterNotationAllowed(plane, isAbsoluteMode ? 90 : 91,
                    isPolarCoordinateActive ? 16 : 15) || plane != activePlane)) ||
        !IsToolLengthSelectionSupported(toolLengthMode, currentHCode) ||
        (isPolarCoordinateActive && !isAbsoluteMode) || isCAxisOffsetRotationEnabled) return false;
    // Fixed native XYZ H/WORK vectors are added after scale/mirror/G68.
    // A WORK rotation is bound to the plane that gives its matching normal;
    // a plane change must not reinterpret yaw/pitch/roll.
    if (isWorkpieceRotationActive)
    {
        double angle = 0.0;
        if (!IsWorkOffsetRowValid(currentWCode, true) ||
            !TryGetFixedWorkPlaneAngle(m_WorkOffset[currentWCode - 1], plane, angle)) return false;
    }
    else if (currentWCode != 0) return false;
    // BASE-PLANE-4 keeps scale/mirror in fixed XYZ slots, independent of
    // circle (u,v). Inactive fields cannot authorize an auxiliary-axis map.
    if (isScalingActive && (!std::isfinite(scaleFactor) || scaleFactor <= 0.0)) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (axis >= 3U && (isMirrorActive[axis] ||
            (isScalingActive && scalingCenterWCS[axis] != 0.0))) return false;
        if (axis < 3U && ((isScalingActive && !std::isfinite(scalingCenterWCS[axis])) ||
            (isMirrorActive[axis] && !std::isfinite(mirrorCenterWCS[axis])))) return false;
    }
    return true;
}

bool CoordinateManager::IsTranslationModeSupported() const noexcept
{
    // Out-of-band axis metadata edits invalidate a selected role and its run.
    if (m_electrodeRotationAxisIndex >= 0 && GetElectrodeRotationAxisIndex() < 0) return false;
    if (!IsBaseArcPlaneSelectionSupported(activePlane) ||
        !IsToolRadiusSelectionSupported(toolRadiusMode, currentDCode) ||
        (toolRadiusMode != 40 && (!IsNCTranslationCutterNotationAllowed(activePlane, isAbsoluteMode ? 90 : 91,
                isPolarCoordinateActive ? 16 : 15) ||
            isCAxisOffsetRotationEnabled)) ||
        currentWCSIndex < 0 ||
        currentWCSIndex >= 60 ||
        static_cast<std::size_t>(currentWCSIndex) >= m_WCSTable.size()) return false;
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(activePlane, plane)) return false;
    if (isG68Active && (isCAxisOffsetRotationEnabled || !std::isfinite(g68Angle) ||
        std::fabs(g68Angle) > 360.0 || !std::isfinite(g68CenterWCS[plane.u]) ||
        !std::isfinite(g68CenterWCS[plane.v]) || g68CenterWCS[plane.normal] != 0.0 ||
        std::signbit(g68CenterWCS[plane.normal]))) return false;
    if (isPolarCoordinateActive && (!isAbsoluteMode || isCAxisOffsetRotationEnabled)) return false;
    if (IsScaleMirrorActive() && isCAxisOffsetRotationEnabled) return false;
    if (isScalingActive && (!std::isfinite(scaleFactor) || scaleFactor <= 0.0)) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (axis >= 3U && (isMirrorActive[axis] ||
            (isScalingActive && scalingCenterWCS[axis] != 0.0))) return false;
        if (axis < 3U && ((isScalingActive && !std::isfinite(scalingCenterWCS[axis])) ||
            (isMirrorActive[axis] && !std::isfinite(mirrorCenterWCS[axis])))) return false;
    }
    if (toolLengthMode == 49)
    {
        if (currentHCode != 0) return false;
    }
    else if ((toolLengthMode != 43 && toolLengthMode != 44) ||
        isCAxisOffsetRotationEnabled || !IsToolOffsetRowValid(currentHCode, true))
        return false;
    if (isWorkpieceRotationActive)
    {
        if (isCAxisOffsetRotationEnabled || !IsWorkOffsetRowValid(currentWCode, true))
            return false;
        double angle = 0.0;
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(activePlane, plane) ||
            !TryGetFixedWorkPlaneAngle(m_WorkOffset[currentWCode - 1], activePlane, angle)) return false;
        if (angle != 0.0 &&
            (!m_workRotationCenterFixed ||
             !std::isfinite(rotationCenterMCS[plane.u]) ||
             !std::isfinite(rotationCenterMCS[plane.v]) ||
             rotationCenterMCS[plane.normal] != 0.0 ||
             std::signbit(rotationCenterMCS[plane.normal])))
            return false;
    }
    else if (currentWCode != 0) return false;
    // G162 remains dormant only without active H/WORK. Active fixed offsets
    // require G163 so their source is independent of any C-axis position.
    return m_WCSTable[currentWCSIndex].size() == 8U;
}

NCTranslationSnapshot CoordinateManager::BuildCurrentCoordinateSnapshot() const noexcept
{
    NCTranslationSnapshot result{};
    int wcsCode = 0;
    if (!TryEncodeNCWorkCoordinateCode(currentWCSIndex, wcsCode) ||
        static_cast<std::size_t>(currentWCSIndex) >= m_WCSTable.size() ||
        m_WCSTable[currentWCSIndex].size() != 8U ||
        !TryBuildAxisIdentity(m_translationAxisSource, result.axisIdentity)) return result;
    if (m_translationRunToken != 0ULL)
    {
        NCAxisIdentitySnapshot expected = m_runAxisIdentity;
        expected.eccentricEnabled = result.axisIdentity.eccentricEnabled;
        if (!SameNCAxisIdentitySnapshot(expected, result.axisIdentity)) return NCTranslationSnapshot{};
    }
    result.runToken = m_translationRunToken;
    result.generation = m_translationGeneration;
    result.revision = m_translationRevision;
    result.wcsCode = wcsCode;
    result.distanceMode = isAbsoluteMode ? 90 : 91;
    result.unitsMode = isInchMode ? 20 : 21;
    result.polarMode = isPolarCoordinateActive ? 16 : 15;
    result.storedStrokeMode = m_programmableTravelLimitEnabled ? 22 : 23;
    result.cutterMode = toolRadiusMode;
    result.cutterD = currentDCode;
    result.cutterRadiusMM = GetActiveToolRadius();
    result.toolLengthMode = toolLengthMode;
    result.toolHCode = currentHCode;
    result.workMode = isWorkpieceRotationActive ? 168 : 169;
    result.workWCode = currentWCode;
    result.rotationMode = isG68Active ? 68 : 69;
    result.rotationPlane = activePlane;
    result.scalingMode = isScalingActive ? 51 : 50;
    result.scalingFactor = isScalingActive ? scaleFactor : 1.0;
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        if (isScalingActive) result.scalingCenterMM[axis] = scalingCenterWCS[axis];
        if (isMirrorActive[axis])
        {
            result.mirrorMask |= 1U << axis;
            result.mirrorCenterMM[axis] = mirrorCenterWCS[axis];
        }
    }
    if (isG68Active)
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(activePlane, plane)) return NCTranslationSnapshot{};
        result.rotationCenterMM[0] = g68CenterWCS[plane.u];
        result.rotationCenterMM[1] = g68CenterWCS[plane.v];
        result.rotationAngleDeg = g68Angle;
    }
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        result.extOffsetMM[axis] = extOffset[axis];
        result.wcsOffsetMM[axis] = m_WCSTable[currentWCSIndex][axis];
        if (toolLengthMode != 49)
            result.toolOffsetMM[axis] = m_ToolOffset[currentHCode - 1][axis];
        if (isWorkpieceRotationActive)
            result.workOffset[axis] = m_WorkOffset[currentWCode - 1][axis];
    }
    double workPlaneAngle = 0.0;
    if (isWorkpieceRotationActive &&
        TryGetFixedWorkPlaneAngle(m_WorkOffset[currentWCode - 1], activePlane, workPlaneAngle) &&
        workPlaneAngle != 0.0)
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(activePlane, plane)) return NCTranslationSnapshot{};
        result.workRotationCenterMM[0] = rotationCenterMCS[plane.u];
        result.workRotationCenterMM[1] = rotationCenterMCS[plane.v];
    }
    return result;
}

NCTranslationSnapshot CoordinateManager::BuildLiveTranslationSnapshot() const noexcept
{
    if (m_translationRunToken == 0ULL || !IsTranslationModeSupported()) return NCTranslationSnapshot{};
    const NCTranslationSnapshot result = BuildCurrentCoordinateSnapshot();
    return IsNCTranslationSnapshotValid(result) ? result : NCTranslationSnapshot{};
}

NCTranslationSnapshot CoordinateManager::GetTranslationSnapshot() const noexcept
{
    return m_translationFrozen ? m_frozenTranslation : BuildLiveTranslationSnapshot();
}

bool CoordinateManager::FreezeTranslationRun() noexcept
{
    if (m_translationFrozen) return IsTranslationRunCurrent();
    const NCTranslationSnapshot candidate = BuildLiveTranslationSnapshot();
    if (!IsNCTranslationSnapshotValid(candidate)) return false;
    m_frozenTranslation = candidate;
    m_translationFrozen = true;
    return true;
}

bool CoordinateManager::PrepareArcPlaneTransition(int plane,
    NCTranslationSnapshot& candidate) const noexcept
{
    // Never reinterpret an active G68 center/angle in a different plane.
    if (isG68Active || isPolarCoordinateActive || toolRadiusMode != 40 || !IsBaseArcPlaneSelectionSupported(plane) || !m_translationFrozen ||
        m_translationResetBypass || plane == m_frozenTranslation.rotationPlane ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.rotationPlane = plane;
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitArcPlaneTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    NCTranslationSnapshot expected{};
    if (!PrepareArcPlaneTransition(candidate.rotationPlane, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Motion publication and exact drain were reserved first. Native MCS,
    // pulse tails, HOME state and every offset table remain untouched.
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    activePlane = candidate.rotationPlane;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PrepareDistanceModeTransition(int mode,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        (mode != 90 && mode != 91) || mode == m_frozenTranslation.distanceMode ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.distanceMode = mode;
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitDistanceModeTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    NCTranslationSnapshot expected{};
    if (!PrepareDistanceModeTransition(candidate.distanceMode, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Publication was accepted under the Motion lifecycle reservation first.
    // No pulse/MCS resampling: the accepted native endpoint remains exact.
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    isAbsoluteMode = candidate.distanceMode == 90;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PrepareStoredStrokeTransition(int mode,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        (mode != 22 && mode != 23) || mode == m_frozenTranslation.storedStrokeMode ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.storedStrokeMode = mode;
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitStoredStrokeTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    NCTranslationSnapshot expected{};
    if (!PrepareStoredStrokeTransition(candidate.storedStrokeMode, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Motion has already reserved/published the same source transaction.
    // The travel policy changes without resampling native geometry or tables.
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    m_programmableTravelLimitEnabled = candidate.storedStrokeMode == 22;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PrepareUnitModeTransition(int mode,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        (mode != 20 && mode != 21) || mode == m_frozenTranslation.unitsMode ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.unitsMode = mode;
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitUnitModeTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    NCTranslationSnapshot expected{};
    if (!PrepareUnitModeTransition(candidate.unitsMode, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Motion has already reserved/published the same source transaction.
    // The unit is input/display metadata: no geometry/table/feed rescaling.
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    isInchMode = candidate.unitsMode == 20;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PreparePolarTransition(int code,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if ((code != 15 && code != 16) || !m_translationFrozen ||
        m_translationResetBypass || !IsTranslationRunCurrent() ||
        m_translationGeneration != m_translationGenerationCounter ||
        (code == 16 && (!isAbsoluteMode || !IsBaseArcPlaneSelectionSupported(activePlane) ||
            isCAxisOffsetRotationEnabled))) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    if (next.polarMode != code)
    {
        if (m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
            m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
        next.polarMode = code;
        next.generation = m_translationGenerationCounter + 1ULL;
        next.revision = m_translationRevision + 1ULL;
    }
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitPolarTransition(const NCTranslationSnapshot& candidate) noexcept
{
    NCTranslationSnapshot expected{};
    if (!PreparePolarTransition(candidate.polarMode, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    if (SameNCTranslationSnapshot(candidate, m_frozenTranslation)) return true;
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    isPolarCoordinateActive = candidate.polarMode == 16;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::IsToolRadiusSelectionSupported(int mode, int dCode) const noexcept
{
    if (mode == 40) return dCode == 0;
    if ((mode != 41 && mode != 42) || dCode < 1 || dCode > 100 ||
        static_cast<std::size_t>(dCode) > m_ToolRadius.size()) return false;
    const std::vector<double>& row = m_ToolRadius[dCode - 1];
    return row.size() == 8U && std::isfinite(row[3]) && row[3] > 0.0;
}

bool CoordinateManager::PrepareToolRadiusSelectionTransition(int mode, int dCode,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (!m_translationFrozen || m_translationResetBypass || !IsTranslationRunCurrent() ||
        m_translationGeneration != m_translationGenerationCounter ||
        !IsToolRadiusSelectionSupported(mode, dCode) ||
        (mode != 40 && (!IsNCTranslationCutterNotationAllowed(activePlane, isAbsoluteMode ? 90 : 91,
            isPolarCoordinateActive ? 16 : 15) || !IsBaseArcPlaneSelectionSupported(activePlane) ||
            isCAxisOffsetRotationEnabled))) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.cutterMode = mode;
    next.cutterD = dCode;
    next.cutterRadiusMM = mode == 40 ? 0.0 : m_ToolRadius[dCode - 1][3];
    if (!SameNCTranslationSnapshot(next, m_frozenTranslation))
    {
        // An active contour must be led out through G40 before a new D/side.
        if ((m_frozenTranslation.cutterMode != 40 && mode != 40) ||
            m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
            m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
        next.generation = m_translationGenerationCounter + 1ULL;
        next.revision = m_translationRevision + 1ULL;
    }
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitToolRadiusSelectionTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    NCTranslationSnapshot expected{};
    if (!PrepareToolRadiusSelectionTransition(candidate.cutterMode, candidate.cutterD, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    if (SameNCTranslationSnapshot(candidate, m_frozenTranslation)) return true;
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    toolRadiusMode = candidate.cutterMode;
    currentDCode = candidate.cutterD;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::SetToolRadiusValue(int dCode, double radiusMM, NCManager* nc)
{
    if (dCode < 1 || dCode > 100 || static_cast<std::size_t>(dCode) > m_ToolRadius.size() ||
        m_ToolRadius[dCode - 1].size() != 8U || !std::isfinite(radiusMM) || radiusMM < 0.0)
        return RejectCoordinateMutation("TOOL_RADIUS_WRITE", "VALUE_OR_ROW", nc, true);
    if (toolRadiusMode != 40 || currentDCode != 0)
        return RejectCoordinateMutation("TOOL_RADIUS_WRITE", "REQUIRES_G40", nc, true);
    if (!GuardCoordinateMutation("TOOL_RADIUS_WRITE", nc)) return false;
    // The historic fourth table field is a scalar radius, never the C-axis value.
    // Program setup is intentionally RAM-only; disk settings remain operator owned.
    m_ToolRadius[dCode - 1][3] = radiusMM == 0.0 ? 0.0 : radiusMM;
    return true;
}

bool CoordinateManager::BuildScaleMirrorSelection(int code, const double* values,
    const bool* hasAxis, double factor, const NCTranslationSnapshot& source,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (!values || !hasAxis || (code != 50 && code != 51 && code != 150 && code != 151) ||
        !std::isfinite(factor) || factor <= 0.0 || (code != 51 && factor != 1.0)) return false;
    bool any = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        any = any || hasAxis[axis];
        if (hasAxis[axis] && (axis >= 3U || !std::isfinite(values[axis]))) return false;
    }
    if ((code == 50 && any) || (code == 151 && !any) ||
        (code == 51 && (!hasAxis[0] || !hasAxis[1] || !hasAxis[2])) ||
        ((code == 51 || code == 151) && isCAxisOffsetRotationEnabled)) return false;
    NCTranslationSnapshot next = source;
    if (code == 50 || code == 51)
    {
        next.scalingMode = code;
        next.scalingFactor = code == 51 ? factor : 1.0;
        for (unsigned axis = 0U; axis < 3U; ++axis)
            next.scalingCenterMM[axis] = code == 51 ? values[axis] : 0.0;
    }
    else
    {
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            if (code == 151 && hasAxis[axis])
            {
                next.mirrorMask |= 1U << axis;
                next.mirrorCenterMM[axis] = values[axis];
            }
            else if (code == 150 && (!any || hasAxis[axis]))
            {
                next.mirrorMask &= ~(1U << axis);
                next.mirrorCenterMM[axis] = 0.0;
            }
        }
    }
    candidate = next;
    return true;
}

bool CoordinateManager::PrepareScaleMirrorTransition(int code, const double* values,
    const bool* hasAxis, double factor, NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass || !IsTranslationRunCurrent() ||
        m_translationGeneration != m_translationGenerationCounter) return false;
    NCTranslationSnapshot next{};
    if (!BuildScaleMirrorSelection(code, values, hasAxis, factor, m_frozenTranslation, next)) return false;
    if (!SameNCTranslationSnapshot(next, m_frozenTranslation))
    {
        if (m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
            m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
        next.generation = m_translationGenerationCounter + 1ULL;
        next.revision = m_translationRevision + 1ULL;
    }
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitScaleMirrorTransition(const NCTranslationSnapshot& candidate) noexcept
{
    if (!m_translationFrozen || m_translationResetBypass || !IsTranslationRunCurrent() ||
        m_translationGeneration != m_translationGenerationCounter ||
        !IsNCTranslationSnapshotValid(candidate)) return false;
    if (SameNCTranslationSnapshot(candidate, m_frozenTranslation)) return true;
    NCTranslationSnapshot normalized = candidate;
    normalized.generation = m_frozenTranslation.generation;
    normalized.revision = m_frozenTranslation.revision;
    normalized.scalingMode = m_frozenTranslation.scalingMode;
    normalized.scalingFactor = m_frozenTranslation.scalingFactor;
    for (unsigned axis = 0U; axis < 3U; ++axis)
        normalized.scalingCenterMM[axis] = m_frozenTranslation.scalingCenterMM[axis];
    const bool scalingSelection = SameNCTranslationSnapshot(normalized, m_frozenTranslation);
    double values[8] = {};
    bool selected[8] = {};
    int code = candidate.scalingMode;
    double factor = candidate.scalingFactor;
    if (scalingSelection)
    {
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            selected[axis] = code == 51;
            values[axis] = candidate.scalingCenterMM[axis];
        }
    }
    else
    {
        factor = 1.0;
        const unsigned removed = m_frozenTranslation.mirrorMask & ~candidate.mirrorMask;
        code = removed != 0U ? 150 : 151;
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            selected[axis] = ((code == 150 ? removed : candidate.mirrorMask) & (1U << axis)) != 0U;
            values[axis] = code == 151 ? candidate.mirrorCenterMM[axis] : 0.0;
        }
    }
    NCTranslationSnapshot expected{};
    if (!PrepareScaleMirrorTransition(code, values, selected, factor, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    isScalingActive = candidate.scalingMode == 51;
    scaleFactor = candidate.scalingFactor;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        scalingCenterWCS[axis] = axis < 3U ? candidate.scalingCenterMM[axis] : 0.0;
        isMirrorActive[axis] = axis < 3U && (candidate.mirrorMask & (1U << axis)) != 0U;
        mirrorCenterWCS[axis] = axis < 3U ? candidate.mirrorCenterMM[axis] : 0.0;
    }
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PrepareWorkCoordinateTransition(int wcsCode,
    NCTranslationSnapshot& candidate) const noexcept
{
    int selectedRow = 0;
    if (toolRadiusMode != 40 || !TryDecodeNCWorkCoordinateCode(wcsCode, selectedRow)) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        wcsCode == m_frozenTranslation.wcsCode ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    const std::size_t rowIndex = static_cast<std::size_t>(selectedRow);
    if (rowIndex >= m_WCSTable.size() || m_WCSTable[rowIndex].size() != 8U) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.wcsCode = wcsCode;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        next.wcsOffsetMM[axis] = m_WCSTable[rowIndex][axis];
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitWorkCoordinateTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    int selectedRow = 0;
    NCTranslationSnapshot expected{};
    if (!TryDecodeNCWorkCoordinateCode(candidate.wcsCode, selectedRow) ||
        !PrepareWorkCoordinateTransition(candidate.wcsCode, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    m_workCenterConfirmation = WorkCenterConfirmation{};
    // The Motion publication is accepted first under its lifecycle reservation.
    // G68 keeps its WCS centre; G168 keeps its separately fixed MCS centre.
    // Selection changes neither the accepted native endpoint nor any table row.
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    currentWCSIndex = selectedRow;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PrepareToolLengthTransition(int normalizedMode,
    int normalizedH, NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        !IsToolLengthSelectionSupported(normalizedMode, normalizedH) ||
        (normalizedMode != 49 && isCAxisOffsetRotationEnabled) ||
        (normalizedMode == m_frozenTranslation.toolLengthMode &&
            normalizedH == m_frozenTranslation.toolHCode) ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.toolLengthMode = normalizedMode;
    next.toolHCode = normalizedH;
    // Cancellation has one canonical all-zero row, independent of old H bits.
    // An active selection copies the validated raw row; G43/G44 owns its sign.
    for (unsigned axis = 0U; axis < 8U; ++axis)
        next.toolOffsetMM[axis] = normalizedMode == 49 ? 0.0 :
            m_ToolOffset[normalizedH - 1][axis];
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitToolLengthTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    NCTranslationSnapshot expected{};
    if (!PrepareToolLengthTransition(candidate.toolLengthMode, candidate.toolHCode, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Motion accepted publication under the drained lifecycle reservation first.
    // Revalidate the old source and selected H row before the NC descriptor commit.
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    toolLengthMode = candidate.toolLengthMode;
    currentHCode = candidate.toolHCode;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PreparePlanarRotationTransition(int mode, double centerX,
    double centerY, double angle, NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        (mode != 68 && mode != 69) || !std::isfinite(centerX) ||
        !std::isfinite(centerY) || !std::isfinite(angle) || std::fabs(angle) > 360.0 ||
        (mode == 68 && (!isAbsoluteMode || isCAxisOffsetRotationEnabled)) ||
        (mode == 69 && (centerX != 0.0 || centerY != 0.0 || angle != 0.0 ||
            std::signbit(centerX) || std::signbit(centerY) || std::signbit(angle))) ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCTranslationSnapshot next = m_frozenTranslation;
    next.rotationMode = mode;
    next.rotationCenterMM[0] = mode == 68 ? centerX : 0.0;
    next.rotationCenterMM[1] = mode == 68 ? centerY : 0.0;
    next.rotationAngleDeg = mode == 68 ? angle : 0.0;
    // Identical bits are idempotent setup, not a descriptor transition.
    if (SameNCTranslationSnapshot(next, m_frozenTranslation)) return false;
    next.generation = m_translationGenerationCounter + 1ULL;
    next.revision = m_translationRevision + 1ULL;
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitPlanarRotationTransition(
    const NCTranslationSnapshot& candidate) noexcept
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    NCTranslationSnapshot expected{};
    if (!PreparePlanarRotationTransition(candidate.rotationMode,
        candidate.rotationCenterMM[0], candidate.rotationCenterMM[1],
        candidate.rotationAngleDeg, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Motion accepted the drained descriptor first. Revalidate all old sources
    // before selecting the new frame; never resample or transform native MCS.
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    isG68Active = candidate.rotationMode == 68;
    NCArcPlaneAxes plane{};
    (void)TryGetNCArcPlaneAxes(candidate.rotationPlane, plane); // validated candidate
    for (unsigned axis = 0U; axis < 3U; ++axis) g68CenterWCS[axis] = 0.0;
    g68CenterWCS[plane.u] = candidate.rotationCenterMM[0];
    g68CenterWCS[plane.v] = candidate.rotationCenterMM[1];
    g68Angle = candidate.rotationAngleDeg;
    m_frozenTranslation = candidate;
    return true;
}

bool CoordinateManager::PrepareWorkpieceTransition(int mode, int wCode,
    bool hasCenter, double centerU, double centerV,
    NCTranslationSnapshot& candidate) const noexcept
{
    if (toolRadiusMode != 40) return false;
    if (!m_translationFrozen || m_translationResetBypass ||
        !IsWorkpieceSelectionSupported(mode, wCode) ||
        !std::isfinite(centerU) || !std::isfinite(centerV) ||
        (!hasCenter && (centerU != 0.0 || centerV != 0.0 ||
            std::signbit(centerU) || std::signbit(centerV))) ||
        (mode == 169 && hasCenter) ||
        (mode == 168 && isCAxisOffsetRotationEnabled) ||
        !IsTranslationRunCurrent() || m_translationGeneration != m_translationGenerationCounter ||
        m_translationGenerationCounter == (std::numeric_limits<std::uint64_t>::max)() ||
        m_translationRevision == (std::numeric_limits<std::uint64_t>::max)()) return false;

    NCTranslationSnapshot next = m_frozenTranslation;
    next.workMode = mode;
    next.workWCode = wCode;
    for (unsigned field = 0U; field < 8U; ++field)
        next.workOffset[field] = mode == 168 ? m_WorkOffset[wCode - 1][field] : 0.0;
    next.workRotationCenterMM[0] = 0.0;
    next.workRotationCenterMM[1] = 0.0;
    double workAngle = 0.0;
    if (mode == 168 && !TryGetNCTranslationWorkPlaneAngle(next, workAngle)) return false;
    if (mode == 168 && workAngle != 0.0)
    {
        if (hasCenter)
        {
            if (!isAbsoluteMode) return false;
            NCArcPlaneAxes plane{};
            if (!TryGetNCArcPlaneAxes(m_frozenTranslation.rotationPlane, plane)) return false;
            // The canonical plane centre uses the entire OLD frozen
            // G68/H/WCS/WORK frame. Omitted normal/auxiliary axes cannot mix
            // into a plane-preserving rotation.
            double input[8] = {};
            input[plane.u] = centerU;
            input[plane.v] = centerV;
            double native[8] = {};
            NCTranslationForwardPoint(m_frozenTranslation, input, native);
            if (!std::isfinite(native[plane.u]) || !std::isfinite(native[plane.v])) return false;
            next.workRotationCenterMM[0] = native[plane.u];
            next.workRotationCenterMM[1] = native[plane.v];
        }
        else
        {
            if (m_frozenTranslation.workMode != 168 ||
                m_frozenTranslation.workWCode != wCode || !m_workRotationCenterFixed)
                return false;
            next.workRotationCenterMM[0] = m_frozenTranslation.workRotationCenterMM[0];
            next.workRotationCenterMM[1] = m_frozenTranslation.workRotationCenterMM[1];
        }
    }
    else if (hasCenter) return false;

    if (!SameNCTranslationSnapshot(next, m_frozenTranslation))
    {
        next.generation = m_translationGenerationCounter + 1ULL;
        next.revision = m_translationRevision + 1ULL;
    }
    if (!IsNCTranslationSnapshotValid(next)) return false;
    candidate = next;
    return true;
}

bool CoordinateManager::CommitWorkpieceTransition(int mode, int wCode,
    bool hasCenter, double centerU, double centerV,
    const NCTranslationSnapshot& candidate) noexcept
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    NCTranslationSnapshot expected{};
    if (!PrepareWorkpieceTransition(mode, wCode, hasCenter, centerU, centerV, expected) ||
        !SameNCTranslationSnapshot(expected, candidate)) return false;
    // Revalidate the original words against the old source and current target row.
    // Only the selected descriptor changes; native positions and tables do not.
    m_translationGenerationCounter = candidate.generation;
    m_translationGeneration = candidate.generation;
    m_translationRevision = candidate.revision;
    isWorkpieceRotationActive = candidate.workMode == 168;
    currentWCode = candidate.workWCode;
    double workAngle = 0.0;
    m_workRotationCenterFixed = isWorkpieceRotationActive &&
        TryGetNCTranslationWorkPlaneAngle(candidate, workAngle) && workAngle != 0.0;
    for (unsigned axis = 0U; axis < 3U; ++axis) rotationCenterMCS[axis] = 0.0;
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(candidate.rotationPlane, plane)) return false;
    rotationCenterMCS[plane.u] = candidate.workRotationCenterMM[0];
    rotationCenterMCS[plane.v] = candidate.workRotationCenterMM[1];
    m_frozenTranslation = candidate;
    if (hasCenter)
    {
        m_workCenterConfirmation.armed = true;
        m_workCenterConfirmation.runToken = candidate.runToken;
        m_workCenterConfirmation.generation = candidate.generation;
        m_workCenterConfirmation.revision = candidate.revision;
        m_workCenterConfirmation.wCode = wCode;
        m_workCenterConfirmation.inputXY[0] = centerU;
        m_workCenterConfirmation.inputXY[1] = centerV;
    }
    return true;
}

bool CoordinateManager::IsTranslationRunCurrent() const noexcept
{
    if (!m_translationFrozen) return IsNCTranslationSnapshotValid(BuildLiveTranslationSnapshot());
    const NCTranslationSnapshot live = BuildLiveTranslationSnapshot();
    return IsNCTranslationSnapshotValid(live) &&
        SameNCTranslationSnapshot(live, m_frozenTranslation);
}

bool CoordinateManager::IsTranslationRunFrozen() const noexcept
{
    return m_translationFrozen;
}

bool CoordinateManager::IsTranslationRunBound() const noexcept
{
    return m_translationRunToken != 0ULL;
}

void CoordinateManager::RetireTranslationRun() noexcept
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    m_translationFrozen = false;
    m_translationRunToken = 0ULL;
    m_translationGeneration = 0ULL;
    m_translationResetBypass = false;
    m_frozenTranslation = NCTranslationSnapshot{};
    m_runAxisIdentity = NCAxisIdentitySnapshot{};
}

void CoordinateManager::BeginTranslationReset() noexcept
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    // Only NC lifecycle cleanup may enter this scope. It never releases the
    // frozen source, which remains the display basis until full retirement.
    m_translationResetBypass = true;
}

void CoordinateManager::EndTranslationReset() noexcept
{
    m_translationResetBypass = false;
}

bool CoordinateManager::RejectCoordinateMutation(const char* operation,
    const char* reason, NCManager* nc, bool reportAlarm) const
{
    RtPrintf("[COORD][REJECT] run=%llu generation=%llu revision=%llu op=%s reason=%s\n",
        static_cast<unsigned long long>(m_translationRunToken),
        static_cast<unsigned long long>(m_translationGeneration),
        static_cast<unsigned long long>(m_translationRevision), operation, reason);
    if (reportAlarm && nc != nullptr)
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
        nc->ChangeState(NCState::HOLD);
    }
    return false;
}

bool CoordinateManager::GuardCoordinateMutation(const char* operation,
    NCManager* nc, bool reportAlarm)
{
    if (m_translationFrozen && !m_translationResetBypass)
        return RejectCoordinateMutation(operation, "RUN_FROZEN", nc, reportAlarm);
    if (m_translationRevision == (std::numeric_limits<std::uint64_t>::max)())
        return RejectCoordinateMutation(operation, "REVISION_EXHAUSTED", nc, reportAlarm);
    ++m_translationRevision;
    return true;
}

bool CoordinateManager::ApplyCoordinateTableValues(int offsetType, int row,
    const bool* hasField, const double* values, NCManager* nc, bool reportAlarm)
{
    if (hasField == nullptr || values == nullptr || offsetType < 0 || offsetType > 3 ||
        (offsetType == 1 && (row < 0 || static_cast<std::size_t>(row) >= m_WCSTable.size())) ||
        (offsetType == 2 && (row < 0 || static_cast<std::size_t>(row) >= m_ToolOffset.size())) ||
        (offsetType == 3 && (row < 0 || static_cast<std::size_t>(row) >= m_WorkOffset.size())))
        return RejectCoordinateMutation("TABLE_WRITE", "INDEX", nc, reportAlarm);
    bool anyField = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!hasField[axis]) continue;
        anyField = true;
        if (!std::isfinite(values[axis]))
            return RejectCoordinateMutation("TABLE_WRITE", "NONFINITE", nc, reportAlarm);
        if ((offsetType == 1 && m_WCSTable[row].size() <= axis) ||
            (offsetType == 2 && m_ToolOffset[row].size() <= axis) ||
            (offsetType == 3 && m_WorkOffset[row].size() <= axis))
            return RejectCoordinateMutation("TABLE_WRITE", "ROW_SHAPE", nc, reportAlarm);
    }
    if (!anyField) return true;
    if (!GuardCoordinateMutation("TABLE_WRITE", nc, reportAlarm)) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!hasField[axis]) continue;
        if (offsetType == 0) extOffset[axis] = values[axis];
        else if (offsetType == 1) m_WCSTable[row][axis] = values[axis];
        else if (offsetType == 2) m_ToolOffset[row][axis] = values[axis];
        else m_WorkOffset[row][axis] = values[axis];
    }
    return true;
}

bool CoordinateManager::TryDecodeToolTableWrite(const NCBlock& block,
    NCManager* nc, int& rowIndex, bool* fields, double* values) const noexcept
{
    const int codeCount = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
    if (nc == nullptr || &nc->CoordSys != this || fields == nullptr || values == nullptr ||
        block.gCount < 0 || !block.hasG || codeCount != 1 || block.gCode != 10 ||
        (block.has('G') && block.val('G') != 10.0) ||
        (block.gCount > 0 && block.gCodes[0] != 10) ||
        block.mCount != 0 || block.isGoto || block.isBlockSkip || !block.has('P') ||
        IsTranslationRunFrozen() ||
        (IsTranslationRunBound() && (m_translationAxisSource != nc ||
            !IsTranslationAxisIdentityCurrent()))) return false;
    const double requested = block.val('P');
    if (!std::isfinite(requested) || requested < 1.0 || requested > 100.0 ||
        std::floor(requested) != requested ||
        requested > static_cast<double>(m_ToolOffset.size())) return false;
    const int candidateRow = static_cast<int>(requested) - 1;
    if (m_ToolOffset[candidateRow].size() != 8U) return false;

    int owners[26];
    for (unsigned letter = 0U; letter < 26U; ++letter) owners[letter] = -1;
    bool rotary[8] = {};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        const AxisContext& native = nc->GetMotion().GetAxisContext(static_cast<int>(axis));
        if (!native.isExist) continue;
        const char letter = nc->m_axisNames[axis];
        // Control words never become tool-offset axes through configuration.
        // I/J/K may be native axes here; G160 owns those fixed WORK angles.
        if (native.axisIndex != static_cast<int>(axis) ||
            (native.axisType != AxisType::LINEAR && native.axisType != AxisType::ROTARY &&
                native.axisType != AxisType::ROTARY_CONTINUOUS) ||
            letter < 'A' || letter > 'Z' || letter == 'G' || letter == 'N' ||
            letter == 'P' || letter == 'L' || letter == 'M' || letter == 'T' || letter == 'H' ||
            owners[letter - 'A'] >= 0) return false;
        owners[letter - 'A'] = static_cast<int>(axis);
        rotary[axis] = native.axisType != AxisType::LINEAR;
    }
    bool candidateFields[8] = {};
    double candidateValues[8] = {};
    bool hasAxis = false;
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        if (!block.has(letter)) continue;
        const double value = block.val(letter);
        if (!std::isfinite(value)) return false;
        if (letter == 'G' || letter == 'N' || letter == 'P') continue;
        const int axis = owners[letter - 'A'];
        if (axis < 0) return false;
        const double nativeValue = ToInternalUnit(value, rotary[axis]);
        if (!std::isfinite(nativeValue)) return false;
        candidateFields[axis] = true;
        candidateValues[axis] = nativeValue;
        hasAxis = true;
    }
    if (!hasAxis) return false;
    // A partial write must leave a complete finite row. Selected bad fields
    // can be repaired; unselected fields retain their exact native values.
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (!std::isfinite(candidateFields[axis] ? candidateValues[axis] :
                m_ToolOffset[candidateRow][axis])) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        fields[axis] = candidateFields[axis];
        values[axis] = candidateValues[axis];
    }
    rowIndex = candidateRow;
    return true;
}

bool CoordinateManager::TryDecodeWorkTableWrite(const NCBlock& block,
    int& rowIndex, bool* fields, double* values) const noexcept
{
    if (!fields || !values || !block.has('P') || block.gCount != 1 ||
        block.gCodes[0] != 160 || block.mCount != 0) return false;
    const double requested = block.val('P');
    if (!std::isfinite(requested) || requested < 1.0 || requested > 100.0 ||
        std::floor(requested) != requested ||
        requested > static_cast<double>(m_WorkOffset.size())) return false;
    const int candidateRow = static_cast<int>(requested) - 1;
    if (m_WorkOffset[candidateRow].size() != 8U) return false;
    // WORK is XYZ millimetres / yaw,pitch,roll degrees / reserved,reserved.
    // In particular a configured C/I/J/K axis never aliases these fields.
    const char fieldNames[6] = { 'X', 'Y', 'Z', 'I', 'J', 'K' };
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        if (!block.has(letter)) continue;
        bool allowed = letter == 'G' || letter == 'N' || letter == 'P';
        for (unsigned field = 0U; field < 6U; ++field)
            if (letter == fieldNames[field]) allowed = true;
        if (!allowed || !std::isfinite(block.val(letter))) return false;
    }
    bool candidateFields[8] = {};
    double candidateValues[8] = {};
    for (unsigned field = 0U; field < 8U; ++field)
    {
        const bool selected = field < 6U && block.has(fieldNames[field]);
        // XYZ are programmed lengths; IJK are fixed WORK angles in degrees.
        // Existing unselected native table fields must not be converted again.
        const double value = selected ? ToInternalUnit(block.val(fieldNames[field]), field >= 3U) :
            m_WorkOffset[candidateRow][field];
        if (!std::isfinite(value)) return false;
        candidateFields[field] = selected;
        candidateValues[field] = selected ? value : 0.0;
    }
    for (unsigned field = 0U; field < 8U; ++field)
    {
        fields[field] = candidateFields[field];
        values[field] = candidateValues[field];
    }
    rowIndex = candidateRow;
    return true;
}

bool CoordinateManager::ApplyCoordinateOrigin(int axis, double desiredWCS,
    NCManager* nc, bool reportAlarm)
{
    if (IsFixedPlanarRotationActive() || IsScaleMirrorActive() || isPolarCoordinateActive)
        return RejectCoordinateMutation("ORIGIN_WRITE", "ROTATION_ACTIVE", nc, reportAlarm);
    if (axis < 0 || axis >= 8 || currentWCSIndex < 0 ||
        static_cast<std::size_t>(currentWCSIndex) >= m_WCSTable.size() ||
        !std::isfinite(desiredWCS) || !std::isfinite(actualMCS[axis]) ||
        !std::isfinite(extOffset[axis]))
        return RejectCoordinateMutation("ORIGIN_WRITE", "INPUT", nc, reportAlarm);
    bool fields[8] = {};
    double values[8] = {};
    fields[axis] = true;
    double toolOffset = 0.0;
    double workOffset = 0.0;
    const bool hasFixedOffset = toolLengthMode != 49 || currentHCode != 0 ||
        isWorkpieceRotationActive || currentWCode != 0;
    if (hasFixedOffset)
    {
        // Origin edits must invert the same fixed source as motion/display.
        // Reject unsupported transforms before changing any table value.
        if (axis >= 3 || !IsTranslationModeSupported())
            return RejectCoordinateMutation("ORIGIN_WRITE", "UNSUPPORTED_TRANSFORM", nc, reportAlarm);
        if (toolLengthMode != 49)
        {
            const double raw = m_ToolOffset[currentHCode - 1][axis];
            toolOffset = toolLengthMode == 43 ? raw : -raw;
        }
        if (isWorkpieceRotationActive)
            workOffset = m_WorkOffset[currentWCode - 1][axis];
    }
    values[axis] = hasFixedOffset ?
        actualMCS[axis] - extOffset[axis] - toolOffset - workOffset - desiredWCS :
        actualMCS[axis] - extOffset[axis] - desiredWCS;
    return ApplyCoordinateTableValues(1, currentWCSIndex, fields, values, nc, reportAlarm);
}

bool CoordinateManager::SetCAxisOffsetRotationEnabled(bool enabled, NCManager* nc)
{
    if (enabled && isPolarCoordinateActive && !m_translationResetBypass)
        return RejectCoordinateMutation("G162", "POLAR_ACTIVE", nc, true);
    if (enabled && !m_translationResetBypass &&
        GlobalConfig::GetInstance().systemMode == SystemMode::EDM_SINKER_MODE &&
        (toolLengthMode == 43 || toolLengthMode == 44) &&
        IsToolOffsetRowValid(currentHCode, false) &&
        (m_ToolOffset[currentHCode - 1][0] != 0.0 || m_ToolOffset[currentHCode - 1][1] != 0.0) &&
        GetElectrodeRotationAxisIndex() < 0)
        return RejectCoordinateMutation("G162", "ELECTRODE_ROLE_REQUIRED", nc, true);
    if (isCAxisOffsetRotationEnabled == enabled) return true;
    if (!GuardCoordinateMutation("G162_G163", nc)) return false;
    isCAxisOffsetRotationEnabled = enabled;
    return true;
}

bool CoordinateManager::SetWCS(int gCode, NCManager* nc, bool reportAlarm)
{
    int calculatedIndex = 0;
    if (nc == nullptr || !TryDecodeNCWorkCoordinateCode(gCode, calculatedIndex))
        return RejectCoordinateMutation("WCS_SELECT", "INDEX", nc, reportAlarm);
    // A bound run must never select an unreadable or nonfinite native row.
    // Unbound legacy table editing/selection retains its existing behaviour.
    if (m_translationRunToken != 0ULL)
    {
        if (static_cast<std::size_t>(calculatedIndex) >= m_WCSTable.size() ||
            m_WCSTable[calculatedIndex].size() != 8U)
            return RejectCoordinateMutation("WCS_SELECT", "ROW", nc, reportAlarm);
        for (unsigned axis = 0U; axis < 8U; ++axis)
            if (!std::isfinite(m_WCSTable[calculatedIndex][axis]))
                return RejectCoordinateMutation("WCS_SELECT", "ROW", nc, reportAlarm);
    }
    if (calculatedIndex != currentWCSIndex &&
        !GuardCoordinateMutation("WCS_SELECT", nc, reportAlarm)) return false;
    currentWCSIndex = calculatedIndex;
    nc->MacroSys.SetVar('$', 14, static_cast<double>(gCode));
    return true;
}

void CoordinateManager::SyncMachinePosition(const double* actualMCS) {
    // 將 EtherCAT 真實的機械座標同步到理論座標，確保 G91 接續移動的安全
    for (int i = 0; i < 8; i++) {
        commandedMCS[i] = actualMCS[i];
    }
}

void CoordinateManager::Transform_WCS_to_MCS(
    const double* targetWCS,
    const bool* hasAxis,
    double* outputMCS)
{
    Transform_WCS_to_MCS_Internal(
        targetWCS,
        hasAxis,
        outputMCS,
        true);
}

bool CoordinateManager::CompleteFixedPlanarEndpoint(
    double* targetWCS, bool* hasAxis) noexcept
{
    if (!targetWCS || !hasAxis) return false;
    if ((m_translationFrozen && m_frozenTranslation.polarMode == 16) ||
        (!m_translationFrozen && isPolarCoordinateActive))
    {
        if (m_translationFrozen && !IsTranslationRunCurrent()) return false;
        if (!m_translationFrozen && !IsTranslationModeSupported()) return false;
        NCTranslationSnapshot source = m_translationFrozen ?
            m_frozenTranslation : BuildCurrentCoordinateSnapshot();
        if (!IsNCAxisIdentitySnapshotValid(source.axisIdentity)) return false;
        if (!m_translationFrozen)
        {
            source.runToken = 1ULL;
            source.generation = 1ULL;
            source.revision = 1ULL;
        }
        return TryCompleteNCTranslationPolarEndpoint(source, commandedMCS, targetWCS, hasAxis);
    }
    if (!m_translationFrozen) return true;
    if (m_frozenTranslation.distanceMode == 91)
    {
        if (!IsTranslationRunCurrent()) return false;
        return TryCompleteNCTranslationIncrementalEndpoint(m_frozenTranslation,
            commandedMCS, targetWCS, hasAxis);
    }
    if (!NCTranslationHasSparseRotatedEndpoint(m_frozenTranslation, hasAxis)) return true;
    if (!IsTranslationRunCurrent()) return false;
    return TryCompleteNCTranslationPlanarEndpoint(m_frozenTranslation,
        commandedMCS, targetWCS, hasAxis);
}

void CoordinateManager::Preview_WCS_to_MCS(
    const double* targetWCS,
    const bool* hasAxis,
    double* outputMCS)
{
    Transform_WCS_to_MCS_Internal(
        targetWCS,
        hasAxis,
        outputMCS,
        false);
}

void CoordinateManager::Transform_WCS_to_MCS_Internal(
    const double* targetWCS,
    const bool* hasAxis,
    double* outputMCS,
    bool commitCommandedMCS)
{
    // Role binding establishes angle identity, not a compensated trajectory.
    // Until the continuous generated-XY path/envelope is implemented, reject
    // active eccentric forward requests atomically (including C-only and ZC).
    if (IsElectrodeOffsetRotationRequested())
    {
        for (unsigned axis = 0U; axis < 8U; ++axis)
            outputMCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
        return;
    }
    // Polar motion is owned by the audited NC preview/admission path only.
    // Legacy direct-transform callers must not treat raw radius/angle as XY.
    if (commitCommandedMCS && (isPolarCoordinateActive ||
        (m_translationFrozen && m_frozenTranslation.polarMode == 16)))
    {
        for (unsigned axis = 0U; axis < 8U; ++axis)
            outputMCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
        return;
    }
    if (m_translationFrozen || ((IsFixedPlanarRotationActive() || IsScaleMirrorActive() || isPolarCoordinateActive) && IsTranslationModeSupported()))
    {
        const NCTranslationSnapshot source = m_translationFrozen ?
            m_frozenTranslation : BuildCurrentCoordinateSnapshot();
        // A failed identity capture must never become an identity transform.
        // Validate before any output selection or commanded endpoint write.
        if (!IsNCAxisIdentitySnapshotValid(source.axisIdentity) ||
            (m_translationFrozen && !IsTranslationRunCurrent()))
        {
            for (unsigned axis = 0U; axis < 8U; ++axis)
                outputMCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
            return;
        }
        if (source.distanceMode == 91)
        {
            double candidate[8] = {};
            NCTranslationSnapshot vectorSource = source;
            if (!m_translationFrozen)
            {
                vectorSource.runToken = 1ULL;
                vectorSource.generation = 1ULL;
                vectorSource.revision = 1ULL;
            }
            if ((m_translationFrozen && !IsTranslationRunCurrent()) ||
                !TryNCTranslationIncrementalTarget(vectorSource, commandedMCS,
                    targetWCS, hasAxis, candidate))
            {
                for (unsigned axis = 0U; axis < 8U; ++axis)
                    outputMCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
                return;
            }
            for (unsigned axis = 0U; axis < 8U; ++axis)
                outputMCS[axis] = candidate[axis];
            if (commitCommandedMCS)
                for (unsigned axis = 0U; axis < 8U; ++axis)
                    if (hasAxis[axis]) commandedMCS[axis] = candidate[axis];
            return;
        }
        // Audited producers complete sparse planar endpoints before Preview.
        // Keep this guard for callers that have not proved their native baseline.
        if (NCTranslationHasSparseRotatedEndpoint(source, hasAxis))
        {
            for (unsigned axis = 0U; axis < 8U; ++axis)
                outputMCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
            return;
        }
        double selectedWCS[8] = {};
        for (unsigned axis = 0U; axis < 8U; ++axis)
            if (hasAxis[axis]) selectedWCS[axis] = targetWCS[axis];
        double transformed[8] = {};
        NCTranslationForwardPoint(source, selectedWCS, transformed);
        for (unsigned axis = 0U; axis < 8U; ++axis)
        {
            outputMCS[axis] = hasAxis[axis] ? transformed[axis] : commandedMCS[axis];
            if (commitCommandedMCS && hasAxis[axis]) commandedMCS[axis] = outputMCS[axis];
        }
        return;
    }

    // 🌟 先複製一份 targetWCS，方便我們做旋轉加工
    double finalTargetWCS[8];
    for (int i = 0; i < 8; i++) finalTargetWCS[i] = targetWCS[i];

    // Polar endpoints are decoded once by CompleteFixedPlanarEndpoint.
    // Never fall back to the old unchecked polar conversion in an unsupported frame.
    if (isPolarCoordinateActive)
    {
        for (unsigned axis = 0U; axis < 8U; ++axis)
            outputMCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
        return;
    }

    // ==========================================================
      // 🌟 數學過濾器 1：G51 縮放 (Scaling)
      // 距離 = (目標 - 中心) * 倍率。新的位置 = 中心 + 距離
      // ==========================================================
    if (isScalingActive && isAbsoluteMode)
    {
        for (int i = 0; i < 8; i++) {
            if (hasAxis[i]) {
                finalTargetWCS[i] = scalingCenterWCS[i] + (finalTargetWCS[i] - scalingCenterWCS[i]) * scaleFactor;
            }
        }
    }

    // ==========================================================
    // 🌟 數學過濾器 2：G51.1 鏡像 (Mirror)
    // 距離 = (目標 - 中心)。新的位置 = 中心 - 距離 (反向)
    // ==========================================================
    if (isAbsoluteMode)
    {
        for (int i = 0; i < 8; i++) {
            if (isMirrorActive[i] && hasAxis[i]) {
                if (isAbsoluteMode) {
                    // G90：絕對座標對稱翻轉
                    finalTargetWCS[i] = mirrorCenterWCS[i] - (finalTargetWCS[i] - mirrorCenterWCS[i]);
                }
                else {
                    // G91：增量距離直接加負號！(走 +10 變成走 -10)
                    finalTargetWCS[i] = -finalTargetWCS[i];
                }
            }
        }
    }


    // ==========================================================
    // 🌟 數學處理：G68 2D 平面座標旋轉
    // ==========================================================
    if (isG68Active && isAbsoluteMode)
    {
        double rad = g68Angle * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = std::sin(rad);
        double dx, dy;

        // 依照群組 2 的平面狀態，決定是哪兩軸要旋轉
        if (activePlane == 17) {
            // G17 (XY 平面)
            dx = targetWCS[0] - g68CenterWCS[0];
            dy = targetWCS[1] - g68CenterWCS[1];
            finalTargetWCS[0] = g68CenterWCS[0] + (dx * c - dy * s);
            finalTargetWCS[1] = g68CenterWCS[1] + (dx * s + dy * c);
        }
        else if (activePlane == 18) {
            // G18 (XZ 平面)
            dx = targetWCS[0] - g68CenterWCS[0];
            double dz = targetWCS[2] - g68CenterWCS[2];
            finalTargetWCS[0] = g68CenterWCS[0] + (dx * c - dz * s);
            finalTargetWCS[2] = g68CenterWCS[2] + (dx * s + dz * c);
        }
        else if (activePlane == 19) {
            // G19 (YZ 平面)
            dy = targetWCS[1] - g68CenterWCS[1];
            double dz = targetWCS[2] - g68CenterWCS[2];
            finalTargetWCS[1] = g68CenterWCS[1] + (dy * c - dz * s);
            finalTargetWCS[2] = g68CenterWCS[2] + (dy * s + dz * c);
        }
    }

    // Active eccentric forward paths were refused above. Fixed H offsets do
    // not depend on any rotary angle; never read an unrelated fourth axis.
    const double targetCAngleMCS = 0.0;

    // ==========================================================
    // 後續維持原本的偏移疊加運算 (注意要把 targetWCS 換成 finalTargetWCS)
    // ==========================================================
    for (int i = 0; i < 8; i++) {
        if (!hasAxis[i]) {
            outputMCS[i] = commandedMCS[i];
            continue;
        }

        // 🌟 這裡把預判好的 C 軸目標角度，傳給刀具補正引擎！
        double totalOffset = extOffset[i] + m_WCSTable[currentWCSIndex][i]
            + GetActiveToolOffset(i, targetCAngleMCS) // 👈 傳入
            + GetActiveWorkOffset(i);

        if (isAbsoluteMode) {
            outputMCS[i] = finalTargetWCS[i] + totalOffset; // 🌟 用旋轉後的 WCS 加偏移
        }
        else {
            outputMCS[i] = commandedMCS[i] + targetWCS[i]; // G91 增量通常不吃 G68 (視各廠牌規定)
        }
        if (commitCommandedMCS)
        {
            commandedMCS[i] = outputMCS[i];
        }
    }
}

// ==========================================
// 🌟 總管函式 (就是這裡遺失導致 LNK2019)
// ==========================================
void CoordinateManager::LoadAllParameters() {
    if (!GuardCoordinateMutation("LOAD_TABLES", nullptr, false)) return;
    // 🌟 1. 檔名改為 WCS_STATUS.ini
    std::ifstream inStatus(GlobalConfig::GetInstance().NCDataDir + "WCS_STATUS.ini");
    if (inStatus.is_open()) {
        std::string line, key;
        while (std::getline(inStatus, line)) {
            std::stringstream ss(line);
            if (std::getline(ss, key, '=') && key == "LastWCSIndex") {
                ss >> currentWCSIndex;
                if (currentWCSIndex < 0 || currentWCSIndex >= 60) currentWCSIndex = 0;
            }
        }
        inStatus.close();
    }

    // 讀取 EXT_OFFSET
    std::ifstream inExt(GlobalConfig::GetInstance().NCDataDir + "EXT_OFFSET.txt");
    if (inExt.is_open()) {
        for (int i = 0; i < 8; i++) inExt >> extOffset[i];
        inExt.close();
    }




    // 讀取三大表格
    LoadTableFromFile("WCS_TABLE.txt", m_WCSTable, 60);
    LoadTableFromFile("TOOL_OFFSET.txt", m_ToolOffset, 100);
    LoadTableFromFile("TOOL_RADIUS.txt", m_ToolRadius, 100);
    LoadTableFromFile("WORK_OFFSET.txt", m_WorkOffset, 100);
    LoadTableFromFile("REFPOINTS.txt", m_RefPoints, 100);

}

void CoordinateManager::SaveAllParameters() {
    SaveWCSStatus();
    SaveExtOffset();
    SaveWCSTable();
    SaveToolOffset();
    SaveWorkOffset();
    SaveToolRadius();
    SaveRefPoints();
}

// ==========================================
// 🌟 單獨儲存函式 (RTX64 相容版)
// ==========================================
void CoordinateManager::SaveWCSStatus() { // 🌟 3. 函式改名
    std::string path = GlobalConfig::GetInstance().NCDataDir + "WCS_STATUS.ini"; // 🌟 檔名改為 WCS_STATUS.ini
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return;

    out << "LastWCSIndex=" << currentWCSIndex << "\n";
    out.close();
}

void CoordinateManager::SaveExtOffset() {
    std::string path = GlobalConfig::GetInstance().NCDataDir + "EXT_OFFSET.txt";
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return;

    out << std::fixed << std::setprecision(4);
    for (int i = 0; i < 8; i++) out << extOffset[i] << (i == 7 ? "" : "\t");
    out << "\n";
    out.close();
}

void CoordinateManager::SaveWCSTable() { SaveTableToFile("WCS_TABLE.txt", m_WCSTable); }
void CoordinateManager::SaveToolOffset() { SaveTableToFile("TOOL_OFFSET.txt", m_ToolOffset); }
void CoordinateManager::SaveWorkOffset() { SaveTableToFile("WORK_OFFSET.txt", m_WorkOffset); }
void CoordinateManager::SaveToolRadius() { SaveTableToFile("TOOL_RADIUS.txt", m_ToolRadius); }
void CoordinateManager::SaveRefPoints() { SaveTableToFile("TOOL_REFPOINTS.txt", m_RefPoints); }

// ==========================================
// 🌟 共用底層：表格陣列讀寫引擎
// ==========================================
void CoordinateManager::SaveTableToFile(const std::string& filename, const std::vector<std::vector<double>>& table) {
    std::string path = GlobalConfig::GetInstance().NCDataDir + filename;
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
        // 若發生此問題，通常是因為 D 槽權限不足，或是 HMI 忘記建資料夾
        return;
    }

    out << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < table.size(); i++) {
        for (int j = 0; j < 8; j++) {
            out << table[i][j] << (j == 7 ? "" : "\t");
        }
        out << "\n";
    }
    out.close();
}

void CoordinateManager::LoadTableFromFile(const std::string& filename, std::vector<std::vector<double>>& table, int maxRows) {
    std::ifstream in(GlobalConfig::GetInstance().NCDataDir + filename);
    if (!in.is_open()) return; // 沒檔案就放空，維持 0.0

    std::string line;
    int row = 0;
    while (std::getline(in, line) && row < maxRows) {
        if (line.empty() || line[0] == ';') continue;
        std::stringstream ss(line);
        for (int i = 0; i < 8; i++) {
            ss >> table[row][i];
        }
        row++;
    }
    in.close();
}
int CoordinateManager::GetCurrentWCSGCode() const
{
    int code = 0;
    // Zero is an invalid/unavailable selection, never an implicit G54 fallback.
    return TryEncodeNCWorkCoordinateCode(currentWCSIndex, code) ? code : 0;
}

// 🌟 更新機械座標 (未來由 EtherCAT 更新)
void CoordinateManager::UpdateActualMCS(const double* newMCS) {
    for (int i = 0; i < 8; i++) {
        actualMCS[i] = newMCS[i];
    }
}



// 🌟 1. 核心公式：算回最簡單的 絕對座標 = 機械座標 - EXT - 表格偏移
void CoordinateManager::GetActualWCS(double* outWCS) const {
    if (m_translationFrozen || ((IsFixedPlanarRotationActive() || IsScaleMirrorActive() || isPolarCoordinateActive) && IsTranslationModeSupported()))
    {
        const NCTranslationSnapshot source = m_translationFrozen ?
            m_frozenTranslation : BuildCurrentCoordinateSnapshot();
        if (!IsNCAxisIdentitySnapshotValid(source.axisIdentity))
        {
            for (unsigned axis = 0U; axis < 8U; ++axis)
                outWCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
            return;
        }
        NCTranslationInversePoint(source, actualMCS, outWCS);
        return;
    }
    // 1. 複製一份真實的物理機械座標
    double tempMCS[8];
    for (int i = 0; i < 8; i++) tempMCS[i] = actualMCS[i];

    // ==========================================================
    // 🌟 2. 逆矩陣運算：把「歪掉的實體座標」轉回「方正的邏輯座標」
    // 在旋轉矩陣中，反矩陣 (Inverse) 剛好等於轉置矩陣 (Transpose)！
    // ==========================================================
    if (isWorkpieceRotationActive && IsWorkOffsetRowValid(currentWCode, false) &&
        (m_WorkOffset[currentWCode - 1][WO_ANGLE_XY_YAW] != 0.0 ||
         m_WorkOffset[currentWCode - 1][WO_ANGLE_XZ_PITCH] != 0.0 ||
         m_WorkOffset[currentWCode - 1][WO_ANGLE_YZ_ROLL] != 0.0))
    {
        int idx = currentWCode - 1;

        // 取得角度轉成弧度
        double a = m_WorkOffset[idx][WO_ANGLE_XY_YAW] * (3.14159265359 / 180.0);
        double b = m_WorkOffset[idx][WO_ANGLE_XZ_PITCH] * (3.14159265359 / 180.0);
        double c = m_WorkOffset[idx][WO_ANGLE_YZ_ROLL] * (3.14159265359 / 180.0);

        double ca = std::cos(a), sa = std::sin(a);
        double cb = std::cos(b), sb = std::sin(b);
        double cc = std::cos(c), sc = std::sin(c);

        // 建立原本的正向旋轉矩陣 R
        double R[3][3];
        R[0][0] = ca * cb;
        R[0][1] = ca * sb * sc - sa * cc;
        R[0][2] = ca * sb * cc + sa * sc;
        R[1][0] = sa * cb;
        R[1][1] = sa * sb * sc + ca * cc;
        R[1][2] = sa * sb * cc - ca * sc;
        R[2][0] = -sb;
        R[2][1] = cb * sc;
        R[2][2] = cb * cc;

        // 計算距離旋轉圓心的位移 (Delta)
        double dx = actualMCS[0] - rotationCenterMCS[0];
        double dy = actualMCS[1] - rotationCenterMCS[1];
        double dz = actualMCS[2] - rotationCenterMCS[2];

        // 矩陣乘法，但注意這裡是 [col][row] 轉置相乘 (等於逆旋轉)！
        tempMCS[0] = rotationCenterMCS[0] + (R[0][0] * dx + R[1][0] * dy + R[2][0] * dz);
        tempMCS[1] = rotationCenterMCS[1] + (R[0][1] * dx + R[1][1] * dy + R[2][1] * dz);
        tempMCS[2] = rotationCenterMCS[2] + (R[0][2] * dx + R[1][2] * dy + R[2][2] * dz);
    }

    // 1. 扣除線性偏移量，得到「包含 G68 旋轉的 WCS」
    double tempWCS[8];
    for (int i = 0; i < 8; i++) {
        double ext = extOffset[i];
        double wcsOffset = m_WCSTable[currentWCSIndex][i];

        // 🌟 傳入 EtherCAT 讀回來的真實 C 軸物理角度
        double toolOffset = GetActiveToolOffset(i, GetElectrodeRotationAngleMCS(actualMCS));

        double workOffset = GetActiveWorkOffset(i);

        tempWCS[i] = tempMCS[i] - (ext + wcsOffset + toolOffset + workOffset);
    }

    // ==========================================================
    // 🌟 2. 顯示反轉：把 G68 的 2D 旋轉逆轉回來！
    // ==========================================================
    for (int i = 0; i < 8; i++) outWCS[i] = tempWCS[i];

    if (isG68Active)
    {
        double rad = g68Angle * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = -std::sin(rad); // 🌟 逆矩陣，sin 變負的
        double dx, dy;

        if (activePlane == 17) {
            dx = tempWCS[0] - g68CenterWCS[0];
            dy = tempWCS[1] - g68CenterWCS[1];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dy * s);
            outWCS[1] = g68CenterWCS[1] + (dx * s + dy * c);
        }
        else if (activePlane == 18) {
            dx = tempWCS[0] - g68CenterWCS[0];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dx * s + dz * c);
        }
        else if (activePlane == 19) {
            dy = tempWCS[1] - g68CenterWCS[1];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[1] = g68CenterWCS[1] + (dy * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dy * s + dz * c);
        }
    }
}

// 🌟 1.5 核心公式：算回最簡單的 絕對座標 = 虛擬命令機械座標 - EXT - 表格偏移
void CoordinateManager::GetCommandedWCS(double* outWCS) const {
    if (m_translationFrozen || ((IsFixedPlanarRotationActive() || IsScaleMirrorActive() || isPolarCoordinateActive) && IsTranslationModeSupported()))
    {
        const NCTranslationSnapshot source = m_translationFrozen ?
            m_frozenTranslation : BuildCurrentCoordinateSnapshot();
        if (!IsNCAxisIdentitySnapshotValid(source.axisIdentity))
        {
            for (unsigned axis = 0U; axis < 8U; ++axis)
                outWCS[axis] = (std::numeric_limits<double>::quiet_NaN)();
            return;
        }
        NCTranslationInversePoint(source, commandedMCS, outWCS);
        return;
    }
    // 1. 複製一份大腦的理論命令機械座標 (Commanded MCS)
    double tempMCS[8];
    for (int i = 0; i < 8; i++) tempMCS[i] = commandedMCS[i]; // 🌟 唯一差別：吃 commandedMCS

    // ==========================================================
    // 🌟 2. 逆矩陣運算：把「歪掉的實體座標」轉回「方正的邏輯座標」
    // 在旋轉矩陣中，反矩陣 (Inverse) 剛好等於轉置矩陣 (Transpose)！
    // ==========================================================
    if (isWorkpieceRotationActive && IsWorkOffsetRowValid(currentWCode, false) &&
        (m_WorkOffset[currentWCode - 1][WO_ANGLE_XY_YAW] != 0.0 ||
         m_WorkOffset[currentWCode - 1][WO_ANGLE_XZ_PITCH] != 0.0 ||
         m_WorkOffset[currentWCode - 1][WO_ANGLE_YZ_ROLL] != 0.0))
    {
        int idx = currentWCode - 1;

        // 取得角度轉成弧度
        double a = m_WorkOffset[idx][WO_ANGLE_XY_YAW] * (3.14159265359 / 180.0);
        double b = m_WorkOffset[idx][WO_ANGLE_XZ_PITCH] * (3.14159265359 / 180.0);
        double c = m_WorkOffset[idx][WO_ANGLE_YZ_ROLL] * (3.14159265359 / 180.0);

        double ca = std::cos(a), sa = std::sin(a);
        double cb = std::cos(b), sb = std::sin(b);
        double cc = std::cos(c), sc = std::sin(c);

        // 建立原本的正向旋轉矩陣 R
        double R[3][3];
        R[0][0] = ca * cb;
        R[0][1] = ca * sb * sc - sa * cc;
        R[0][2] = ca * sb * cc + sa * sc;
        R[1][0] = sa * cb;
        R[1][1] = sa * sb * sc + ca * cc;
        R[1][2] = sa * sb * cc - ca * sc;
        R[2][0] = -sb;
        R[2][1] = cb * sc;
        R[2][2] = cb * cc;

        // 計算距離旋轉圓心的位移 (Delta)
        // ⚠️ 這裡的圓心還是用實體擷取的 rotationCenterMCS，這沒問題，因為 G168 是在靜止時啟動的
        double dx = tempMCS[0] - rotationCenterMCS[0];
        double dy = tempMCS[1] - rotationCenterMCS[1];
        double dz = tempMCS[2] - rotationCenterMCS[2];

        // 矩陣乘法，但注意這裡是 [col][row] 轉置相乘 (等於逆旋轉)！
        tempMCS[0] = rotationCenterMCS[0] + (R[0][0] * dx + R[1][0] * dy + R[2][0] * dz);
        tempMCS[1] = rotationCenterMCS[1] + (R[0][1] * dx + R[1][1] * dy + R[2][1] * dz);
        tempMCS[2] = rotationCenterMCS[2] + (R[0][2] * dx + R[1][2] * dy + R[2][2] * dz);
    }

    // 1. 扣除線性偏移量，得到「包含 G68 旋轉的 WCS」
    double tempWCS[8];
    for (int i = 0; i < 8; i++) {
        double ext = extOffset[i];
        double wcsOffset = m_WCSTable[currentWCSIndex][i];

        // 🌟 傳入大腦記錄的理論 C 軸角度
        double toolOffset = GetActiveToolOffset(i, GetElectrodeRotationAngleMCS(commandedMCS)); // 🌟 唯一差別：吃 commandedMCS

        double workOffset = GetActiveWorkOffset(i);

        tempWCS[i] = tempMCS[i] - (ext + wcsOffset + toolOffset + workOffset);
    }

    // ==========================================================
    // 🌟 2. 顯示反轉：把 G68 的 2D 旋轉逆轉回來！
    // ==========================================================
    for (int i = 0; i < 8; i++) outWCS[i] = tempWCS[i];

    if (isG68Active)
    {
        double rad = g68Angle * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = -std::sin(rad); // 🌟 逆矩陣，sin 變負的
        double dx, dy;

        if (activePlane == 17) {
            dx = tempWCS[0] - g68CenterWCS[0];
            dy = tempWCS[1] - g68CenterWCS[1];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dy * s);
            outWCS[1] = g68CenterWCS[1] + (dx * s + dy * c);
        }
        else if (activePlane == 18) {
            dx = tempWCS[0] - g68CenterWCS[0];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dx * s + dz * c);
        }
        else if (activePlane == 19) {
            dy = tempWCS[1] - g68CenterWCS[1];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[1] = g68CenterWCS[1] + (dy * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dy * s + dz * c);
        }
    }
}
// G92 preserves commanded machine position and changes only the selected
// work-coordinate row. Decode and prepare are pure, including every rejection.
bool CoordinateManager::TryDecodeG92Origin(const NCBlock& block, NCManager* nc,
    bool* fields, double* nativeTargets) const noexcept
{
    const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
    if (nc == nullptr || &nc->CoordSys != this || fields == nullptr || nativeTargets == nullptr ||
        block.gCount < 0 || !block.hasG || count != 1 || block.gCode != 92 ||
        (block.gCount > 0 && block.gCodes[0] != 92) ||
        (block.has('G') && block.val('G') != 92.0) ||
        block.mCount != 0 || block.isGoto || block.isBlockSkip) return false;
    int owners[26];
    for (unsigned letter = 0U; letter < 26U; ++letter) owners[letter] = -1;
    bool rotary[8] = {};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        const AxisContext& native = nc->GetMotion().GetAxisContext(static_cast<int>(axis));
        if (!native.isExist) continue;
        const char letter = nc->m_axisNames[axis];
        if (native.axisIndex != static_cast<int>(axis) ||
            (native.axisType != AxisType::LINEAR && native.axisType != AxisType::ROTARY &&
                native.axisType != AxisType::ROTARY_CONTINUOUS) ||
            letter < 'A' || letter > 'Z' || letter == 'G' || letter == 'N' ||
            letter == 'P' || letter == 'L' || letter == 'M' || letter == 'T' || letter == 'H' ||
            owners[letter - 'A'] >= 0) return false;
        owners[letter - 'A'] = static_cast<int>(axis);
        rotary[axis] = native.axisType != AxisType::LINEAR;
    }
    bool selected[8] = {};
    double targets[8] = {};
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        if (!block.has(letter)) continue;
        if (!std::isfinite(block.val(letter))) return false;
        if (letter == 'G' || letter == 'N') continue;
        const int axis = owners[letter - 'A'];
        if (axis < 0) return false;
        selected[axis] = true;
        targets[axis] = ToInternalUnit(block.val(letter), rotary[axis]);
        if (!std::isfinite(targets[axis])) return false;
    }
    double proposed[8] = {};
    bool changed = false;
    if (!TryPrepareG92Origin(selected, targets, nc, proposed, changed)) return false;
    std::memcpy(fields, selected, sizeof(selected));
    std::memcpy(nativeTargets, targets, sizeof(targets));
    return true;
}

bool CoordinateManager::TryPrepareG92Origin(const bool* fields, const double* nativeTargets,
    NCManager* nc, double* proposed, bool& changed) const noexcept
{
    if (nc == nullptr || &nc->CoordSys != this || fields == nullptr || nativeTargets == nullptr ||
        proposed == nullptr || m_translationFrozen || m_translationResetBypass ||
        (m_translationAxisSource != nullptr && m_translationAxisSource != nc) ||
        (IsTranslationRunBound() && !IsTranslationAxisIdentityCurrent()) ||
        !isAbsoluteMode || !IsTranslationModeSupported() || isG68Active || IsScaleMirrorActive() ||
        isPolarCoordinateActive || toolRadiusMode != 40 || currentDCode != 0 ||
        IsElectrodeOffsetRotationRequested()) return false;
    NCAxisIdentitySnapshot identity{};
    if (!TryBuildAxisIdentity(nc, identity)) return false;
    if (isWorkpieceRotationActive)
        for (unsigned field = WO_ANGLE_XY_YAW; field <= WO_ANGLE_YZ_ROLL; ++field)
            if (m_WorkOffset[currentWCode - 1][field] != 0.0) return false;

    // Check configured owners again for direct array callers, including a
    // no-op. Dormant slots never acquire a coordinate-table write implicitly.
    bool owners[26] = {};
    bool any = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        const AxisContext& native = nc->GetMotion().GetAxisContext(static_cast<int>(axis));
        if (native.isExist)
        {
            const char letter = nc->m_axisNames[axis];
            if (native.axisIndex != static_cast<int>(axis) ||
                (native.axisType != AxisType::LINEAR && native.axisType != AxisType::ROTARY &&
                    native.axisType != AxisType::ROTARY_CONTINUOUS) ||
                letter < 'A' || letter > 'Z' || letter == 'G' || letter == 'N' ||
                letter == 'P' || letter == 'L' || letter == 'M' || letter == 'T' || letter == 'H' ||
                owners[letter - 'A']) return false;
            owners[letter - 'A'] = true;
        }
        if (fields[axis] && (!native.isExist || !std::isfinite(nativeTargets[axis]))) return false;
        any = any || fields[axis];
        if (!std::isfinite(commandedMCS[axis]) || !std::isfinite(extOffset[axis]) ||
            !std::isfinite(m_WCSTable[currentWCSIndex][axis])) return false;
    }
    if (!any) return false;
    double current[8] = {};
    GetCommandedWCS(current);
    double candidate[8] = {};
    bool candidateChanged = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(current[axis])) return false;
        const double oldValue = m_WCSTable[currentWCSIndex][axis];
        // Equal commanded coordinates preserve the exact old field, even -0.
        candidate[axis] = fields[axis] && current[axis] != nativeTargets[axis] ?
            oldValue + (current[axis] - nativeTargets[axis]) : oldValue;
        if (!std::isfinite(candidate[axis])) return false;
        // Recompose the actual unrotated inverse path in its evaluation order.
        // Finite row fields alone do not prove a finite coordinate frame.
        const double offset = extOffset[axis] + candidate[axis] +
            GetActiveToolOffset(static_cast<int>(axis), GetElectrodeRotationAngleMCS(commandedMCS)) +
            GetActiveWorkOffset(static_cast<int>(axis));
        const double readback = commandedMCS[axis] - offset;
        if (!std::isfinite(offset) || !std::isfinite(readback)) return false;
        const bool fieldChanged = std::memcmp(&candidate[axis], &oldValue, sizeof(double)) != 0;
        // A nonzero requested shift must survive both storage and readback.
        // General rounded coordinates need not equal their decimal input bits.
        if (fields[axis] && current[axis] != nativeTargets[axis] &&
            (!fieldChanged || readback == current[axis])) return false;
        candidateChanged = candidateChanged || fieldChanged;
    }
    if (candidateChanged && m_translationRevision ==
        (std::numeric_limits<std::uint64_t>::max)()) return false;
    std::memcpy(proposed, candidate, sizeof(candidate));
    changed = candidateChanged;
    return true;
}

bool CoordinateManager::ApplyG92(const bool* axisProgrammed, const double* targetPos, NCManager* nc)
{
    double proposed[8] = {};
    bool changed = false;
    if (!TryPrepareG92Origin(axisProgrammed, targetPos, nc, proposed, changed))
        return RejectCoordinateMutation("G92", "INPUT_OR_FRAME", nc, true);
    if (changed)
    {
        if (!GuardCoordinateMutation("G92", nc)) return false;
        std::memcpy(m_WCSTable[currentWCSIndex].data(), proposed, sizeof(proposed));
        nc->UpdateSystemVariables();
    }
    RtPrintf("[ORIGIN][G92] wcs=%d changed=%d saveRequested=0\n", GetCurrentWCSGCode(), changed ? 1 : 0);
    return true;
}

void CoordinateManager::GetActualMCS(double* outMCS) const {
    for (int i = 0; i < 8; i++) {
        outMCS[i] = actualMCS[i];
    }
}

void  CoordinateManager::Set_G90G91(int value, NCManager* nc)//設定90絕對模式 91增量模式
{
    if (value == 90)
    {
        if (!isAbsoluteMode && !GuardCoordinateMutation("G90", nc)) return;
        isAbsoluteMode = true;
        nc->MacroSys.SetVar('$', 3, 90);

    }
    if (value == 91)
    {
        if (isPolarCoordinateActive && !m_translationResetBypass)
        { RejectCoordinateMutation("G91", "POLAR_ACTIVE", nc, true); return; }
        if (isAbsoluteMode && !GuardCoordinateMutation("G91", nc)) return;
        isAbsoluteMode = false;
        nc->MacroSys.SetVar('$', 3, 91);
    }
}

// ==========================================
// 🌟 新增：刀長補正邏輯
// ==========================================
void CoordinateManager::SetToolLengthCompensation(int gCode, int hCode, NCManager* nc)
{
    if ((gCode != 43 && gCode != 44 && gCode != 49) ||
        (gCode == 49 && hCode != 0))
    {
        RejectCoordinateMutation("TOOL_LENGTH", "MODE", nc, true);
        return;
    }
    // G49 and H0 share one canonical cancelled descriptor.
    if (gCode == 49 || hCode == 0)
    {
        CancelToolLengthCompensation(nc);
        return;
    }
    if (hCode < 1 || hCode > 100 ||
        static_cast<std::size_t>(hCode) > m_ToolOffset.size())
    {
        RejectCoordinateMutation("TOOL_LENGTH", "H_INDEX", nc, true);
        return;
    }
    // Preserve legacy multi-axis transforms outside the bounded NC run, but
    // never permit malformed/nonfinite table rows to become active.
    if (!IsToolOffsetRowValid(hCode, m_translationRunToken != 0ULL))
    {
        RejectCoordinateMutation("TOOL_LENGTH", "H_ROW", nc, true);
        return;
    }
    if (m_translationRunToken != 0ULL && isCAxisOffsetRotationEnabled)
    {
        RejectCoordinateMutation("TOOL_LENGTH", "G163_REQUIRED_FOR_BOUND_H", nc, true);
        return;
    }
    if (isCAxisOffsetRotationEnabled &&
        GlobalConfig::GetInstance().systemMode == SystemMode::EDM_SINKER_MODE &&
        (m_ToolOffset[hCode - 1][0] != 0.0 || m_ToolOffset[hCode - 1][1] != 0.0) &&
        GetElectrodeRotationAxisIndex() < 0)
    {
        RejectCoordinateMutation("TOOL_LENGTH", "ELECTRODE_ROLE_REQUIRED", nc, true);
        return;
    }
    if ((toolLengthMode != gCode || currentHCode != hCode) &&
        !GuardCoordinateMutation("TOOL_LENGTH", nc)) return;
    toolLengthMode = gCode;
    currentHCode = hCode;
    if (nc) nc->MacroSys.SetVar('$', 8, static_cast<double>(gCode));
}

void CoordinateManager::CancelToolLengthCompensation(NCManager* nc)
{
    if ((toolLengthMode != 49 || currentHCode != 0) &&
        !GuardCoordinateMutation("G49", nc)) return;
    toolLengthMode = 49; // 強制切換為 G49
    currentHCode = 0;


    // 更新系統巨集變數 (群組 8 的狀態改為 49)
    if (nc) nc->MacroSys.SetVar('$', 8, 49.0);
}

// 🌟 核心：算出「現在這個軸」要補償多少距離？
// 🌟 核心：算出「現在這個軸」要補償多少距離？(加入 EDM C 軸偏心旋轉)
double CoordinateManager::GetActiveToolOffset(int axisIndex, double cAngleMCS) const
{
    // 如果是 G49，或是 H0 (取消補正)，直接回傳 0.0
    if (toolLengthMode == 49 || currentHCode <= 0 || currentHCode > m_ToolOffset.size()) {
        return 0.0;
    }

    if (axisIndex < 0 || axisIndex >= 8 ||
        (toolLengthMode != 43 && toolLengthMode != 44) ||
        !IsToolOffsetRowValid(currentHCode, false))
        return (std::numeric_limits<double>::quiet_NaN)();
    int arrayIndex = currentHCode - 1;

    // 取出原始表格中的補正數值
    double rawOffset = m_ToolOffset[arrayIndex][axisIndex];

    // =========================================================
     // 🌟 【EDM 偏心補償】：只有在「功能啟動 (isCAxisOffsetRotationEnabled == true)」
     // 且計算 X (0) 或 Y (1) 時，才套用 C 軸旋轉矩陣
     // =========================================================
    if (IsElectrodeOffsetRotationRequested() && (axisIndex == 0 || axisIndex == 1))
    {
        if (GetElectrodeRotationAxisIndex() < 0 || !std::isfinite(cAngleMCS))
            return (std::numeric_limits<double>::quiet_NaN)();
        double offsetX = m_ToolOffset[arrayIndex][0];
        double offsetY = m_ToolOffset[arrayIndex][1];

        double rad = cAngleMCS * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = std::sin(rad);

        if (axisIndex == 0) {
            rawOffset = offsetX * c - offsetY * s; // 旋轉後的 X 偏心量
        }
        else if (axisIndex == 1) {
            rawOffset = offsetX * s + offsetY * c; // 旋轉後的 Y 偏心量
        }
    }

    // G43 為正向 (+)，G44 為負向 (-)
    return (toolLengthMode == 43) ? rawOffset : -rawOffset;
}
// ==========================================================
// 🌟 啟動工件旋轉 (G168)
// ==========================================================
void CoordinateManager::SetWorkpieceRotation(int wCode, const bool* hasAxis, const double* targetWCS, NCManager* nc)
{
    // Every handler attempt consumes the receipt, including mismatches/rejections.
    const WorkCenterConfirmation confirmation = m_workCenterConfirmation;
    m_workCenterConfirmation = WorkCenterConfirmation{};
    const bool fixedRun = IsTranslationRunBound();
    if (!hasAxis || !targetWCS || !IsWorkOffsetRowValid(wCode, fixedRun) ||
        (fixedRun && !IsWorkpieceSelectionSupported(168, wCode)))
    {
        RejectCoordinateMutation("G168", "W_SELECTION", nc, true);
        return;
    }
    bool hasAnyCenter = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        hasAnyCenter = hasAnyCenter || hasAxis[axis];
        if (hasAxis[axis] && (axis >= 3U || !std::isfinite(targetWCS[axis])))
        {
            RejectCoordinateMutation("G168", "CENTER_UNSUPPORTED", nc, true);
            return;
        }
    }
    if (fixedRun)
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(activePlane, plane))
        {
            RejectCoordinateMutation("G168", "PLANE", nc, true);
            return;
        }
        // The descriptor owns all fixed geometry. Never also enable the legacy
        // Motion matrix, and never sample Actual to choose a fixed centre.
        double workAngle = 0.0;
        if (!TryGetFixedWorkPlaneAngle(m_WorkOffset[wCode - 1], activePlane, workAngle))
        {
            RejectCoordinateMutation("G168", "PLANE_ANGLE", nc, true);
            return;
        }
        const bool hasPlaneRotation = workAngle != 0.0;
        if (confirmation.armed)
        {
            bool exactPlane = hasAxis[plane.u] && hasAxis[plane.v];
            for (unsigned axis = 0U; axis < 8U; ++axis)
                if (axis != plane.u && axis != plane.v) exactPlane = exactPlane && !hasAxis[axis];
            const double inputUV[2] = { targetWCS[plane.u], targetWCS[plane.v] };
            if (!m_translationFrozen || m_translationResetBypass || !exactPlane ||
                !hasPlaneRotation || !isAbsoluteMode || !isWorkpieceRotationActive ||
                currentWCode != wCode || !m_workRotationCenterFixed ||
                confirmation.runToken != m_translationRunToken ||
                confirmation.generation != m_translationGeneration ||
                m_translationGeneration != m_translationGenerationCounter ||
                confirmation.revision != m_translationRevision ||
                confirmation.wCode != wCode ||
                std::memcmp(confirmation.inputXY, inputUV, sizeof confirmation.inputXY) != 0 ||
                !IsTranslationRunCurrent())
            {
                RejectCoordinateMutation("G168", "CENTER_CONFIRMATION", nc, true);
                return;
            }
            // The frozen descriptor already owns the transform. Confirmation
            // changes neither Motion matrix state nor native geometry.
            return;
        }
        if (isWorkpieceRotationActive && currentWCode == wCode && !hasAnyCenter)
        {
            if (hasPlaneRotation && !m_workRotationCenterFixed)
            {
                RejectCoordinateMutation("G168", "FIXED_PLANE_CENTER_REQUIRED", nc, true);
                return;
            }
            if (!hasPlaneRotation) m_workRotationCenterFixed = false;
            if (m_translationFrozen)
            {
                if (!IsTranslationRunCurrent())
                    RejectCoordinateMutation("G168", "SOURCE_CHANGED", nc, true);
            }
            else if (nc)
                nc->GetMotion().SetCoordinateTransform(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            return;
        }
        if (m_translationFrozen && !m_translationResetBypass)
        {
            GuardCoordinateMutation("G168", nc);
            return;
        }
        bool exactPlaneCenter = hasAxis[plane.u] && hasAxis[plane.v];
        for (unsigned axis = 0U; axis < 8U; ++axis)
            if (axis != plane.u && axis != plane.v) exactPlaneCenter = exactPlaneCenter && !hasAxis[axis];
        if ((hasPlaneRotation && !exactPlaneCenter) ||
            (!hasPlaneRotation && hasAnyCenter))
        {
            RejectCoordinateMutation("G168", "FIXED_PLANE_CENTER_REQUIRED", nc, true);
            return;
        }
        double center[2] = {};
        if (hasPlaneRotation)
        {
            if (!isAbsoluteMode || !IsTranslationModeSupported() || isCAxisOffsetRotationEnabled)
            {
                RejectCoordinateMutation("G168", "PRIOR_SOURCE_UNSUPPORTED", nc, true);
                return;
            }
            // Explicit centre words belong to the complete prior source.
            // Both XY words are present; omitted native axes remain untouched.
            double convertedMCS[8] = {};
            Preview_WCS_to_MCS(targetWCS, hasAxis, convertedMCS);
            const unsigned slots[2] = { plane.u, plane.v };
            for (unsigned component = 0U; component < 2U; ++component)
            {
                center[component] = convertedMCS[slots[component]];
                if (!std::isfinite(center[component]))
                {
                    RejectCoordinateMutation("G168", "CENTER_NONFINITE", nc, true);
                    return;
                }
            }
        }
        if (!GuardCoordinateMutation("G168", nc)) return;
        currentWCode = wCode;
        isWorkpieceRotationActive = true;
        for (unsigned axis = 0U; axis < 3U; ++axis) rotationCenterMCS[axis] = 0.0;
        rotationCenterMCS[plane.u] = center[0];
        rotationCenterMCS[plane.v] = center[1];
        m_workRotationCenterFixed = hasPlaneRotation;
        if (nc) nc->GetMotion().SetCoordinateTransform(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        RtPrintf("[WORK][SELECT] mode=168 W=%d plane=%d fixedXYZ=1 fixedAngle=%d matrix=0\n",
            wCode, activePlane, hasPlaneRotation ? 1 : 0);
        return;
    }

    // Legacy rotation outside a bounded MEMORY run retains its matrix path.
    // Preview the centre against the prior source and validate all components
    // before publishing mode, index, centre or Motion matrix. Never move tail.
    double convertedMCS[8] = {};
    Preview_WCS_to_MCS(targetWCS, hasAxis, convertedMCS);
    double center[3] = {};
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        center[axis] = hasAxis[axis] ? convertedMCS[axis] : actualMCS[axis];
        if (!std::isfinite(center[axis]))
        {
            RejectCoordinateMutation("G168", "CENTER_NONFINITE", nc, true);
            return;
        }
    }
    if (!GuardCoordinateMutation("G168", nc)) return;
    currentWCode = wCode;
    isWorkpieceRotationActive = true;
    m_workRotationCenterFixed = false;
    for (unsigned axis = 0U; axis < 3U; ++axis) rotationCenterMCS[axis] = center[axis];
    const std::vector<double>& row = m_WorkOffset[wCode - 1];
    if (nc)
    {
        nc->GetMotion().SetCoordinateTransform(true, center[0], center[1], center[2],
            row[WO_ANGLE_XY_YAW], row[WO_ANGLE_XZ_PITCH], row[WO_ANGLE_YZ_ROLL]);
        RtPrintf("[G168] Workpiece Rotation ON (W%d). Center:(%.3f, %.3f, %.3f)\n",
            wCode, center[0], center[1], center[2]);
    }
}

// ==========================================================
// 🌟 關閉工件旋轉 (G169)
// ==========================================================
void CoordinateManager::CancelWorkpieceRotation(NCManager* nc)
{
    m_workCenterConfirmation = WorkCenterConfirmation{};
    if ((isWorkpieceRotationActive || currentWCode != 0) &&
        !GuardCoordinateMutation("G169", nc)) return;
    isWorkpieceRotationActive = false;
    currentWCode = 0;
    m_workRotationCenterFixed = false;
    for (unsigned axis = 0U; axis < 3U; ++axis) rotationCenterMCS[axis] = 0.0;

    if (nc) {
        // 傳入 false 關閉旋轉矩陣
        nc->GetMotion().SetCoordinateTransform(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        RtPrintf("[G169] Workpiece Rotation OFF.\n");
    }
}

// 🌟 核心：算出「現在這個軸」要因為 G168 額外平移多少距離？
double CoordinateManager::GetActiveWorkOffset(int axisIndex) const
{
    // 1. 如果 G168 沒開啟，或者是無效的 W 碼，就不平移
    if (!isWorkpieceRotationActive || !IsWorkOffsetRowValid(currentWCode, false)) {
        return 0.0;
    }

    // =========================================================
    // 🌟 【神級修復防護罩】：隔離角度與位移
    // 陣列 0, 1, 2 分別是 X, Y, Z 的線性平移 (允許回傳)
    // 陣列 3, 4, 5 是 旋轉角度，絕對不能當作平移量回傳！
    // =========================================================
    if (axisIndex < 0 || axisIndex > 2) {
        return 0.0; // 只要大於 Z 軸 (也就是角度欄位)，全部強制回傳 0.0！
    }

    // 2. 只有 X, Y, Z (0, 1, 2) 才會走到這一步，回傳正確的平移量
    int arrayIndex = currentWCode - 1;
    return m_WorkOffset[arrayIndex][axisIndex];
}

void CoordinateManager::SetActivePlane(int gCode, NCManager* nc)
{
    if (gCode == 17 || gCode == 18 || gCode == 19) {
        if (IsTranslationRunBound() && !m_translationResetBypass &&
            !IsBaseArcPlaneSelectionSupported(gCode))
        { RejectCoordinateMutation("PLANE", "BASE_FRAME_REQUIRED", nc, true); return; }
        if (isPolarCoordinateActive && gCode != activePlane && !m_translationResetBypass)
        { RejectCoordinateMutation("PLANE", "POLAR_ACTIVE", nc, true); return; }
        if (activePlane != gCode && isG68Active && !m_translationResetBypass)
        { RejectCoordinateMutation("PLANE", "CANCEL_G68_FIRST", nc, true); return; }
        if (activePlane != gCode && !GuardCoordinateMutation("PLANE", nc)) return;
        activePlane = gCode;

        // 更新系統巨集變數 (群組 2)
        if (nc) {
            nc->MacroSys.SetVar('$', 2, (double)gCode);
        }
    }
}

// ==========================================================
// 🌟 啟動 G68 座標旋轉
// ==========================================================
void CoordinateManager::SetG68Rotation(const double* centerPos, const bool* hasAxis, double angle, NCManager* nc)
{
    if (!centerPos || !hasAxis || !std::isfinite(angle) || std::fabs(angle) > 360.0)
    {
        RejectCoordinateMutation("G68", "ANGLE_OR_INPUT", nc, true);
        return;
    }
    const bool fixedRun = IsTranslationRunBound();
    NCArcPlaneAxes plane{};
    const bool planeValid = TryGetNCArcPlaneAxes(activePlane, plane);
    if (fixedRun && (!planeValid || !isAbsoluteMode ||
        !IsBaseArcPlaneSelectionSupported(activePlane) ||
        isCAxisOffsetRotationEnabled || !hasAxis[plane.u] || !hasAxis[plane.v] || hasAxis[plane.normal]))
    {
        RejectCoordinateMutation("G68", "FIXED_PLANE_SCOPE", nc, true);
        return;
    }
    if (!planeValid)
    {
        RejectCoordinateMutation("G68", "PLANE", nc, true);
        return;
    }
    // Capture omitted legacy centres against the complete prior source. Never
    // change angle/active before inverse conversion, or move commandedMCS.
    double priorWCS[8] = {};
    if (!fixedRun && (!hasAxis[0] || !hasAxis[1] || !hasAxis[2]))
        GetActualWCS(priorWCS);
    double proposedCenter[3] = {};
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        proposedCenter[axis] = fixedRun && axis == plane.normal ? 0.0 :
            (hasAxis[axis] ? centerPos[axis] : priorWCS[axis]);
        if (!std::isfinite(proposedCenter[axis]))
        {
            RejectCoordinateMutation("G68", "CENTER_NONFINITE", nc, true);
            return;
        }
    }
    const bool unchanged = isG68Active &&
        std::memcmp(&angle, &g68Angle, sizeof(angle)) == 0 &&
        std::memcmp(proposedCenter, g68CenterWCS, sizeof(proposedCenter)) == 0;
    if (unchanged)
    {
        if (m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
        {
            RejectCoordinateMutation("G68", "SOURCE_CHANGED", nc, true);
            return;
        }
        // A staged commit selected this frame before the normal modal setter.
        if (nc) nc->MacroSys.SetVar('$', 16, 68.0);
        return;
    }
    if (!GuardCoordinateMutation("G68", nc)) return;
    isG68Active = true;
    g68Angle = angle;
    for (unsigned axis = 0U; axis < 3U; ++axis) g68CenterWCS[axis] = proposedCenter[axis];
    if (nc) nc->MacroSys.SetVar('$', 16, 68.0);
    RtPrintf("[G68] 2D Rotation ON. Plane:%d, CenterUV:(%.3f, %.3f), Angle:%.3f\n",
        activePlane, g68CenterWCS[plane.u], g68CenterWCS[plane.v], g68Angle);
}

void CoordinateManager::CancelG68Rotation(NCManager* nc)
{
    if ((isG68Active || g68Angle != 0.0 || g68CenterWCS[0] != 0.0 ||
        g68CenterWCS[1] != 0.0 || g68CenterWCS[2] != 0.0) &&
        !GuardCoordinateMutation("G69", nc)) return;
    isG68Active = false;
    g68Angle = 0.0;
    for (unsigned axis = 0U; axis < 3U; ++axis) g68CenterWCS[axis] = 0.0;
    if (nc) nc->MacroSys.SetVar('$', 16, 69.0);
    RtPrintf("[G69] 2D Rotation OFF.\n");
}

// ==========================================================
// 🌟 G51 啟動縮放 / G50 關閉縮放
// ==========================================================
void CoordinateManager::SetScaling(const double* centerPos, const bool* hasAxis,
    double factor, NCManager* nc)
{
    if (!centerPos || !hasAxis || !std::isfinite(factor) || factor <= 0.0)
    { RejectCoordinateMutation("G51", "INPUT", nc, true); return; }
    double center[8] = {};
    bool selected[8] = {};
    if (!IsTranslationRunBound()) GetCommandedWCS(center);
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if ((axis >= 3U && hasAxis[axis]) ||
            (axis < 3U && IsTranslationRunBound() && !hasAxis[axis]))
        { RejectCoordinateMutation("G51", "XYZ_CENTER_REQUIRED", nc, true); return; }
        if (axis < 3U)
        {
            if (hasAxis[axis]) center[axis] = centerPos[axis];
            selected[axis] = true;
        }
        else center[axis] = 0.0;
    }
    NCTranslationSnapshot source{};
    source.scalingMode = isScalingActive ? 51 : 50;
    source.scalingFactor = scaleFactor;
    for (unsigned axis = 0U; axis < 3U; ++axis) source.scalingCenterMM[axis] = scalingCenterWCS[axis];
    NCTranslationSnapshot next{};
    if (!BuildScaleMirrorSelection(51, center, selected, factor, source, next))
    { RejectCoordinateMutation("G51", "CENTER_OR_FACTOR", nc, true); return; }
    const bool unchanged = SameNCTranslationSnapshot(source, next);
    if (unchanged && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("G51", "SOURCE_CHANGED", nc, true); return; }
    if (!unchanged && !GuardCoordinateMutation("G51", nc)) return;
    isScalingActive = true;
    scaleFactor = factor;
    for (unsigned axis = 0U; axis < 8U; ++axis) scalingCenterWCS[axis] = center[axis];
    if (nc) nc->MacroSys.SetVar('$', 11, 51.0);
}

void CoordinateManager::CancelScaling(NCManager* nc)
{
    bool changed = isScalingActive || scaleFactor != 1.0;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        changed = changed || scalingCenterWCS[axis] != 0.0 || std::signbit(scalingCenterWCS[axis]);
    if (changed && !GuardCoordinateMutation("G50", nc)) return;
    if (!changed && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("G50", "SOURCE_CHANGED", nc, true); return; }
    isScalingActive = false;
    scaleFactor = 1.0;
    for (unsigned axis = 0U; axis < 8U; ++axis) scalingCenterWCS[axis] = 0.0;
    if (nc) nc->MacroSys.SetVar('$', 11, 50.0);
}

void CoordinateManager::SetMirror(const double* mirrorPos, const bool* hasAxis, NCManager* nc)
{
    NCTranslationSnapshot source{};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (axis >= 3U && isMirrorActive[axis])
        { RejectCoordinateMutation("G151", "XYZ_ONLY", nc, true); return; }
        if (axis < 3U && isMirrorActive[axis])
        { source.mirrorMask |= 1U << axis; source.mirrorCenterMM[axis] = mirrorCenterWCS[axis]; }
    }
    NCTranslationSnapshot next{};
    if (!BuildScaleMirrorSelection(151, mirrorPos, hasAxis, 1.0, source, next))
    { RejectCoordinateMutation("G151", "XYZ_CENTER_REQUIRED", nc, true); return; }
    const bool unchanged = SameNCTranslationSnapshot(source, next);
    if (unchanged && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("G151", "SOURCE_CHANGED", nc, true); return; }
    if (!unchanged && !GuardCoordinateMutation("G151", nc)) return;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        isMirrorActive[axis] = axis < 3U && (next.mirrorMask & (1U << axis)) != 0U;
        mirrorCenterWCS[axis] = axis < 3U ? next.mirrorCenterMM[axis] : 0.0;
    }
}

void CoordinateManager::CancelMirror(const bool* hasAxis, NCManager* nc)
{
    if (!hasAxis) { RejectCoordinateMutation("G150", "INPUT", nc, true); return; }
    bool any = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        any = any || hasAxis[axis];
        if (axis >= 3U && hasAxis[axis])
        { RejectCoordinateMutation("G150", "XYZ_ONLY", nc, true); return; }
    }
    bool changed = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (!any || hasAxis[axis]) changed = changed || isMirrorActive[axis] ||
            mirrorCenterWCS[axis] != 0.0 || std::signbit(mirrorCenterWCS[axis]);
    if (changed && !GuardCoordinateMutation("G150", nc)) return;
    if (!changed && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("G150", "SOURCE_CHANGED", nc, true); return; }
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (!any || hasAxis[axis]) { isMirrorActive[axis] = false; mirrorCenterWCS[axis] = 0.0; }
}

// ==========================================================
// 🌟 G16 啟動極座標 / G15 關閉極座標
// ==========================================================
void CoordinateManager::SetPolarCoordinate(NCManager* nc)
{
    if (!IsTranslationRunBound() || !isAbsoluteMode || !IsNCArcPlaneCode(activePlane) ||
        isCAxisOffsetRotationEnabled || !IsTranslationModeSupported())
    { RejectCoordinateMutation("G16", "PLANE_G90_G163_SCOPE", nc, true); return; }
    if (isPolarCoordinateActive)
    {
        if (m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
        { RejectCoordinateMutation("G16", "SOURCE_CHANGED", nc, true); return; }
    }
    else if (!GuardCoordinateMutation("G16", nc)) return;
    isPolarCoordinateActive = true;
    if (nc) nc->MacroSys.SetVar('$', 17, 16.0);
    RtPrintf("[G16] Polar endpoint notation ON (plane=%d, G90).\n", activePlane);
}

void CoordinateManager::CancelPolarCoordinate(NCManager* nc)
{
    if (isPolarCoordinateActive && !GuardCoordinateMutation("G15", nc)) return;
    if (m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("G15", "SOURCE_CHANGED", nc, true); return; }
    isPolarCoordinateActive = false;
    if (nc) nc->MacroSys.SetVar('$', 17, 15.0);
    RtPrintf("[G15] Polar endpoint notation OFF.\n");
}


// 🌟 啟動 G41 / G42
void CoordinateManager::SetToolRadiusCompensation(int gCode, int dCode, NCManager* nc)
{
    if (!IsTranslationRunBound() || !IsToolRadiusSelectionSupported(gCode, dCode) ||
        (gCode != 41 && gCode != 42) || !IsNCTranslationCutterNotationAllowed(activePlane, isAbsoluteMode ? 90 : 91,
        isPolarCoordinateActive ? 16 : 15) || !IsBaseArcPlaneSelectionSupported(activePlane) ||
        isCAxisOffsetRotationEnabled)
    { RejectCoordinateMutation("TOOL_RADIUS", "PLANE_DISTANCE_NOTATION_G163_D_RADIUS", nc, true); return; }
    const bool changed = toolRadiusMode != gCode || currentDCode != dCode;
    if (changed && toolRadiusMode != 40)
    { RejectCoordinateMutation("TOOL_RADIUS", "REQUIRES_G40", nc, true); return; }
    if (changed && !GuardCoordinateMutation("TOOL_RADIUS", nc)) return;
    if (!changed && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("TOOL_RADIUS", "SOURCE_CHANGED", nc, true); return; }
    toolRadiusMode = gCode;
    currentDCode = dCode;
    if (nc) nc->MacroSys.SetVar('$', 7, static_cast<double>(gCode));
    // Keep RT diagnostics on the existing integer-only bit-pattern protocol.
    // The target RtPrintf did not consume %.9f as a floating-point argument.
    const double radiusMM = GetActiveToolRadius();
    std::uint64_t radiusMMBits = 0ULL;
    std::memcpy(&radiusMMBits, &radiusMM, sizeof(radiusMMBits));
    RtPrintf("[G%d] Tool Radius Comp ON. D-Code: %d radiusMMBits=%llu\n",
        gCode, dCode, static_cast<unsigned long long>(radiusMMBits));
}

void CoordinateManager::CancelToolRadiusCompensation(NCManager* nc)
{
    const bool changed = toolRadiusMode != 40 || currentDCode != 0;
    if (changed && !GuardCoordinateMutation("G40", nc)) return;
    if (!changed && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    { RejectCoordinateMutation("G40", "SOURCE_CHANGED", nc, true); return; }
    toolRadiusMode = 40;
    currentDCode = 0;
    if (nc) nc->MacroSys.SetVar('$', 7, 40.0);
    RtPrintf("[G40] Tool Radius Comp OFF.\n");
}

double CoordinateManager::GetActiveToolRadius() const
{
    if ((toolRadiusMode != 41 && toolRadiusMode != 42) || currentDCode < 1 ||
        currentDCode > 100 || static_cast<std::size_t>(currentDCode) > m_ToolRadius.size()) return 0.0;
    const std::vector<double>& row = m_ToolRadius[currentDCode - 1];
    if (row.size() != 8U || !std::isfinite(row[3]) || row[3] < 0.0) return 0.0;
    return row[3] == 0.0 ? 0.0 : row[3];
}

bool CoordinateManager::GetRefPoint(int pCode, double* outPos) const {
    int index = pCode - 1; // P1 對應 index 0
    if (index < 0 || index >= m_RefPoints.size()) return false;

    for (int i = 0; i < 8; i++) outPos[i] = m_RefPoints[index][i];
    return true;
}
// 🌟 實作設定刀號 API
void CoordinateManager::SetToolNumber(int tCode, NCManager* nc)
{
    currentTCode = tCode;

    // (選配) 如果需要將 T 碼狀態寫入巨集變數或觸發狀態廣播
    if (nc) {
        // 例如更新系統巨集變數
        nc->MacroSys.SetVar('$', 48, (double)tCode);
    }

    //DEBUG_PRINT("[Coordinate] Tool Number Updated: T%d\n", currentTCode);
}

// 🌟 實作設定獨立工件號 API
void CoordinateManager::SetWorkpieceNumber(int num, NCManager* nc)
{
    currentWorkpieceNum = num;

    // (選配) 如果你有需要，也可以在這裡直接即時更新 $49
    if (nc)
    {
        nc->MacroSys.SetVar('$', 49, (double)num);
    }

    // DEBUG_PRINT("[Coordinate] Independent Workpiece Number Updated: %d\n", currentWorkpieceNum);
}

// =========================================================
// 🌟 取得機台當下的剩餘移動量 (Distance To Go)
// =========================================================
void CoordinateManager::GetDistanceToGo(double* outDTG, NCManager* nc) const
{
    for (int i = 0; i < 8; i++) outDTG[i] = 0.0;
    if (!nc) return;

    double targetMCS_mm[8] = { 0.0 };

    // 這裡拿到的 targetMCS_mm 已經是正確的釐米 (mm) 單位了
    if (nc->GetMotion().GetExecutingTargetMCS(targetMCS_mm))
    {
        for (int i = 0; i < 8; i++) {
            // mm 減去 mm，結果完美！
            outDTG[i] = targetMCS_mm[i] - actualMCS[i];

            if (std::abs(outDTG[i]) < 0.001) {
                outDTG[i] = 0.0;
            }
        }
    }
}

// 🌟 設定 G20 / G21
void CoordinateManager::SetUnitMode(int gCode, NCManager* nc)
{
    if (gCode == 20) {
        if (!isInchMode && !GuardCoordinateMutation("G20", nc)) return;
        isInchMode = true;
        if (nc) nc->MacroSys.SetVar('$', 6, 20.0); // 更新群組 6 巨集變數
        // RtPrintf("[G20] Inch Mode Active.\n");
    }
    else if (gCode == 21) {
        if (isInchMode && !GuardCoordinateMutation("G21", nc)) return;
        isInchMode = false;
        if (nc) nc->MacroSys.SetVar('$', 6, 21.0);
        // RtPrintf("[G21] Metric Mode Active.\n");
    }
}

// 🌟 出口閘門：將底層純公制數值，轉換為「當下單位」準備顯示或存入系統變數
double CoordinateManager::ToDisplayUnit(double internalMmValue, bool isRotaryAxis) const
{
    // 鐵則：旋轉軸 (角度) 絕對不轉換！
    if (isRotaryAxis) {
        return internalMmValue;
    }

    // 若為英制模式，除以 25.4 送出去
    if (isInchMode) {
        return internalMmValue / 25.4;
    }

    return internalMmValue; // 公制模式，原封不動送出去
}

// 🌟 入口閘門：將外部 (G碼或 HMI 畫面) 輸入的數值，洗成「純公制」準備存入表格或給底層
double CoordinateManager::ToInternalUnit(double externalValue, bool isRotaryAxis) const
{
    // 鐵則：旋轉軸 (角度) 絕對不轉換！
    if (isRotaryAxis) {
        return externalValue;
    }

    // 若為英制模式，乘以 25.4 洗成公制再進入大腦
    if (isInchMode) {
        return externalValue * 25.4;
    }

    return externalValue; // 公制模式，原封不動進入
}

void CoordinateManager::TransformManualVector(
    const double* manualVector,
    double* machineVector) const
{
    // ======================================================
    // Pointer Guard
    // ======================================================

    if (manualVector == nullptr ||
        machineVector == nullptr)
    {
        return;
    }


    // ======================================================
    // Copy Input
    //
    // 保證允許：
    //
    // TransformManualVector(vector, vector);
    // ======================================================

    double input[8] =
    {
        manualVector[0],
        manualVector[1],
        manualVector[2],
        manualVector[3],
        manualVector[4],
        manualVector[5],
        manualVector[6],
        manualVector[7]
    };


    // ======================================================
    // Manual Frame OFF
    //
    // 完全 1:1 通過。
    //
    // 這樣 NCPLCManager 之後可以永遠呼叫這個 API，
    // 不需要自己判斷 Enabled。
    // ======================================================

    if (!m_manualFrameEnabled)
    {
        for (int i = 0;
            i < 8;
            ++i)
        {
            machineVector[i] =
                input[i];
        }

        return;
    }


    // ======================================================
    // Degree -> Radian
    // ======================================================

    constexpr double DEG_TO_RAD =
        0.01745329251994329576923690768489;


    const double yaw =
        m_manualFrameYawDeg *
        DEG_TO_RAD;

    const double pitch =
        m_manualFramePitchDeg *
        DEG_TO_RAD;

    const double roll =
        m_manualFrameRollDeg *
        DEG_TO_RAD;


    const double cy =
        std::cos(yaw);

    const double sy =
        std::sin(yaw);

    const double cp =
        std::cos(pitch);

    const double sp =
        std::sin(pitch);

    const double cr =
        std::cos(roll);

    const double sr =
        std::sin(roll);


    // ======================================================
    // Manual XYZ Vector
    // ======================================================

    const double x =
        input[0];

    const double y =
        input[1];

    const double z =
        input[2];


    // ======================================================
    // 1. Rx(Roll)
    // ======================================================

    const double x1 =
        x;

    const double y1 =
        cr * y -
        sr * z;

    const double z1 =
        sr * y +
        cr * z;


    // ======================================================
    // 2. Ry(Pitch)
    // ======================================================

    const double x2 =
        cp * x1 +
        sp * z1;

    const double y2 =
        y1;

    const double z2 =
        -sp * x1 +
        cp * z1;


    // ======================================================
    // 3. Rz(Yaw)
    // ======================================================

    const double x3 =
        cy * x2 -
        sy * y2;

    const double y3 =
        sy * x2 +
        cy * y2;

    const double z3 =
        z2;


    // ======================================================
    // Output XYZ
    // ======================================================

    machineVector[0] =
        x3;

    machineVector[1] =
        y3;

    machineVector[2] =
        z3;


    // ======================================================
    // A / B / C / U / V
    //
    // Manual Frame 不作用。
    // ======================================================

    machineVector[3] =
        input[3];

    machineVector[4] =
        input[4];

    machineVector[5] =
        input[5];

    machineVector[6] =
        input[6];

    machineVector[7] =
        input[7];
}


// ==========================================================
// Manual Frame
//
// 只供 Manual Motion 使用：
//
//   Normal JOG
//   Fine JOG
//   INCH JOG
//   MPG
//
// 不影響 NC Program / WCS / G68 / G168 / HOME。
// ==========================================================


// ==========================================================
// Enable / Disable
// ==========================================================

void CoordinateManager::SetManualFrameEnabled(
    bool enabled)
{
    m_manualFrameEnabled =
        enabled;
}


bool CoordinateManager::IsManualFrameEnabled() const
{
    return m_manualFrameEnabled;
}


// ==========================================================
// Set All Angles
//
// Angle Unit:
//     Degree
//
// 非有限數值不接受，避免 NaN / INF 進入 Motion。
// ==========================================================

void CoordinateManager::SetManualFrameAngles(
    double yawDeg,
    double pitchDeg,
    double rollDeg)
{
    if (!std::isfinite(yawDeg) ||
        !std::isfinite(pitchDeg) ||
        !std::isfinite(rollDeg))
    {
        return;
    }

    m_manualFrameYawDeg =
        yawDeg;

    m_manualFramePitchDeg =
        pitchDeg;

    m_manualFrameRollDeg =
        rollDeg;
}


// ==========================================================
// Individual Angle Set
// ==========================================================

void CoordinateManager::SetManualFrameYaw(
    double yawDeg)
{
    if (!std::isfinite(yawDeg))
    {
        return;
    }

    m_manualFrameYawDeg =
        yawDeg;
}


void CoordinateManager::SetManualFramePitch(
    double pitchDeg)
{
    if (!std::isfinite(pitchDeg))
    {
        return;
    }

    m_manualFramePitchDeg =
        pitchDeg;
}


void CoordinateManager::SetManualFrameRoll(
    double rollDeg)
{
    if (!std::isfinite(rollDeg))
    {
        return;
    }

    m_manualFrameRollDeg =
        rollDeg;
}


// ==========================================================
// Angle Get
// ==========================================================

double CoordinateManager::GetManualFrameYaw() const
{
    return m_manualFrameYawDeg;
}


double CoordinateManager::GetManualFramePitch() const
{
    return m_manualFramePitchDeg;
}


double CoordinateManager::GetManualFrameRoll() const
{
    return m_manualFrameRollDeg;
}
