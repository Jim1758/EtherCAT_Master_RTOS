#pragma once

#include "MotionCore.h"
#include "NCEccentricCProfile.h"
#include <cstring>
#include <new>
#include <type_traits>

// The accepted model kernels remain unchanged. Keep their preparation call
// chains separate from the two transport entries even in optimized RTSS builds.
#if defined(_MSC_VER)
#define BASE79B_TRANSPORT_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79B_TRANSPORT_NOINLINE __attribute__((noinline))
#else
#define BASE79B_TRANSPORT_NOINLINE
#endif

// BASE79B_FIX1: prepare-side scratch belongs to the caller, never the RT
// motion group. The startup / LOAD diagnostic owns it in its private heap
// workspace. Requiring it explicitly prevents hidden nested large stack
// objects or per-command allocations. One invocation at a time per scratch.
// Private storage prevents input/output references from aliasing its members.
class MotionEccentricCTransportWorkspace;
bool PrepareMotionEccentricCCommand(const MotionCommand&,
    NCEccentricCProfileValue&, MotionEccentricCTransportWorkspace&) noexcept;
bool BuildMotionEccentricCCommand(const NCEccentricCRuntimeInput&,
    const MotionExecutionIdentity&, const MotionOwnerLease&, MotionCommand&,
    MotionEccentricCTransportWorkspace&) noexcept;

class MotionEccentricCTransportWorkspace
{
public:
    MotionEccentricCTransportWorkspace() noexcept = default;
    MotionEccentricCTransportWorkspace(const MotionEccentricCTransportWorkspace&) = delete;
    MotionEccentricCTransportWorkspace& operator=(const MotionEccentricCTransportWorkspace&) = delete;
private:
    NCEccentricCRuntimeInput decodeInput_{};
    NCEccentricCRuntimeValue buildRuntime_{}, decodeRuntime_{};
    MotionCommand candidate_{};
    NCEccentricCProfileValue verifiedProfile_{};
    friend bool PrepareMotionEccentricCCommand(const MotionCommand&,
        NCEccentricCProfileValue&, MotionEccentricCTransportWorkspace&) noexcept;
    friend bool BuildMotionEccentricCCommand(const NCEccentricCRuntimeInput&,
        const MotionExecutionIdentity&, const MotionOwnerLease&, MotionCommand&,
        MotionEccentricCTransportWorkspace&) noexcept;
};



// BASE79B: self-contained structural transport, not an NC admission path.
// The actual MotionCore authorization gate rejects every marked packet.
// NCEccentricCDynamicMotionAdmission remains false; no executor is stored here.
// No public enqueue entrypoint is added. All 34 mem_* doubles are occupied:
// startPos[8]=pulse anchors; ratio[8]=PPU; radius=signed sweep;
// startAngle=F degrees/min; centerX=stop seconds; centerY=minX;
// totalDist=maxX; totalAngle=minY; origin={maxY,minRole,maxRole};
// dir=0 matrix rows {X,Y,role}, columns {native V,A,D ceilings}.
// BASE79J/K dir=2 is resolved Z+C: row0 holds componentwise-minimum XYZ
// ceilings, row1 role ceilings, row2 {signed Z delta, minZ, maxZ}. This explicit
// variant preserves the command ABI; both sides prepare identical limits.
// BASE79M dir=3 carries resolved G90/G91 nominal XY deltas in centerPos[0/1]. Row2
// still stores {Z delta, minZ, maxZ}, including held Z. Slots are the sorted
// union of authored linear axes, generated XY and the electrode role.
// Scalar targetVel/accTime/decTime and compact endpoint slots are independently
// reconstructed. One-sample runtime envelope, no additional FIR/history.
namespace MotionEccentricCTransportDetail
{
    // Reconstruct the exact type in place, preserving its nonzero defaults
    // without materializing a whole-value temporary on the bounded stack.
    template<class T> inline void ResetInPlace(T& value) noexcept
    {
        static_assert(std::is_trivially_destructible<T>::value &&
            std::is_nothrow_default_constructible<T>::value, "Bounded typed reset only.");
        value.~T();
        ::new (static_cast<void*>(&value)) T{};
    }
    inline bool Zero(double value) noexcept { return NCEccentricCDetail::Same(value, 0.0); }
    inline unsigned Role(const MotionCommand& c) noexcept
    { return static_cast<unsigned>(c.sourceTranslation.axisIdentity.electrodeAxisPlusOne) - 1U; }
    inline bool ZC(const MotionCommand& c) noexcept { return c.dir == 2; }
    inline bool AuthoredXY(const MotionCommand& c) noexcept { return c.dir == 3; }
    inline bool MixedLayout(const MotionCommand& c) noexcept { return c.dir == 2 || c.dir == 3; }
    inline std::uint32_t GroupMask(const MotionCommand& c, unsigned role) noexcept
    {
        const bool generated = c.sourceTranslation.toolOffsetMM[0] != 0.0 ||
            c.sourceTranslation.toolOffsetMM[1] != 0.0;
        std::uint32_t mask = (1U << role) | (generated ? 3U : 0U);
        if (AuthoredXY(c))
        {
            if (c.centerPos[0] != 0.0) mask |= 1U;
            if (c.centerPos[1] != 0.0) mask |= 2U;
        }
        if (MixedLayout(c) && c.mem_transformMatrix[2][0] != 0.0) mask |= 4U;
        return mask;
    }
    inline int AxisCount(std::uint32_t mask) noexcept
    {
        int count = 0;
        for (unsigned axis = 0U; axis < 8U; ++axis) if ((mask & (1U << axis)) != 0U) ++count;
        return count;
    }
    inline unsigned AxisForMask(std::uint32_t mask, int slot) noexcept
    {
        for (unsigned axis = 0U; axis < 8U; ++axis)
            if ((mask & (1U << axis)) != 0U && slot-- == 0) return axis;
        return 8U;
    }
    inline unsigned LimitRow(const MotionCommand& c, unsigned axis) noexcept
    { return MixedLayout(c) ? (axis < 3U ? 0U : 1U) : (axis < 2U ? axis : 2U); }
    // Callers validate axis membership independently. This accessor is shared by
    // decode, pre-start binding and every live consumer check.
    inline double Limit(const MotionCommand& c, unsigned axis, unsigned component) noexcept
    { return c.mem_transformMatrix[LimitRow(c, axis)][component]; }
    inline double Lower(const MotionCommand& c, unsigned axis) noexcept
    {
        return axis == 0U ? c.mem_centerY : axis == 1U ? c.mem_totalAngle :
            axis == 2U && MixedLayout(c) ? c.mem_transformMatrix[2][1] : c.mem_transformOrigin[1];
    }
    inline double Upper(const MotionCommand& c, unsigned axis) noexcept
    {
        return axis == 0U ? c.mem_totalDist : axis == 1U ? c.mem_transformOrigin[0] :
            axis == 2U && MixedLayout(c) ? c.mem_transformMatrix[2][2] : c.mem_transformOrigin[2];
    }
    inline unsigned AxisForSlot(bool generated, bool zc, unsigned role, int slot) noexcept
    {
        return generated && slot < 2 ? static_cast<unsigned>(slot) :
            zc && slot == (generated ? 2 : 0) ? 2U : role;
    }
}

// The producer prepares the same conservative XYZ ceilings that the compact
// Z+C packet carries. C-only inputs are left byte-for-byte unchanged.
inline bool CanonicalizeMotionEccentricCLinearLimits(NCEccentricCRuntimeInput& input) noexcept
{
    using namespace NCEccentricCDetail;
    if (!std::isfinite(input.geometry.xDeltaMM) || !std::isfinite(input.geometry.yDeltaMM) ||
        !std::isfinite(input.geometry.zDeltaMM)) return false;
    if (input.geometry.xDeltaMM == 0.0 && input.geometry.yDeltaMM == 0.0 &&
        input.geometry.zDeltaMM == 0.0) return true;
    auto& g = input.geometry;
    double velocity = g.maxVelocityNative[0];
    double acceleration = g.maxAccelerationNative[0];
    double deceleration = g.maxDecelerationNative[0];
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        if (!Positive(g.maxVelocityNative[axis]) || !Positive(g.maxAccelerationNative[axis]) ||
            !Positive(g.maxDecelerationNative[axis])) return false;
        velocity = Min(velocity, g.maxVelocityNative[axis]);
        acceleration = Min(acceleration, g.maxAccelerationNative[axis]);
        deceleration = Min(deceleration, g.maxDecelerationNative[axis]);
    }
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        g.maxVelocityNative[axis] = velocity;
        g.maxAccelerationNative[axis] = acceleration;
        g.maxDecelerationNative[axis] = deceleration;
    }
    return true;
}

inline bool IsMotionEccentricCFeedSourceAllowed(const MotionCommand& c) noexcept
{
    using namespace MotionEccentricCTransportDetail;
    const auto& s = c.sourceTranslation;
    unsigned role = 8U;
    if (!c.pathCoreEccentricCFeedExactStop || !c.execution.IsAssigned() ||
        c.execution.source != MotionCommandSource::NC_MEMORY || c.execution.sourceBlockId < 0 ||
        c.sourceLinePC != c.execution.sourceBlockId || !c.ownerLease.IsValid() ||
        c.ownerLease.owner != MotionOwner::AUTO || !NCEccentricCDetail::Scope(s) ||
        !NCEccentricCDetail::Role(s, role) || (s.distanceMode != 90 && s.distanceMode != 91) ||
        c.mode != InterpolationMode::LINEAR || c.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
        c.pathCorePlanarCircle || c.pathCoreFullCircle || c.pathCoreRetainedTraversal ||
        c.pathCoreRetainedReverse || c.replayTerminalAlreadyPublished || c.cncFeedLookahead ||
        c.cncCornerBlend || c.pathCoreFeedExactStop || c.pathCoreRotaryFeedExactStop ||
        c.pathCoreZCFeedExactStop || c.pathCoreXYZCFeedExactStop || c.pathCoreXYZCUVFeedExactStop ||
        c.mem_enableTransform || (c.dir != 0 && c.dir != 2 && c.dir != 3) ||
        !Zero(c.startRadius) || !Zero(c.endRadius) || !Zero(c.cncPrefixVelocityPPS) ||
        c.sourceWCS != s.wcsCode || c.sourceToolLengthMode != s.toolLengthMode ||
        c.sourceHCode != s.toolHCode || c.sourceToolRadiusMode != 40 || c.sourceDCode != 0 ||
        c.sourceIsAbsoluteMode != (s.distanceMode == 90) || c.sourceG68Active || !Zero(c.sourceG68Angle) ||
        c.sourceG168Active || c.sourceWCode != 0 || c.sourceG51Active ||
        !NCEccentricCDetail::Same(c.sourceScaleRatio, 1.0) || c.sourceMirrorMask != 0U ||
        c.sourceG16Active || !c.sourceG162Active || c.sourcePlaneMode != 17) return false;
    const bool xy = AuthoredXY(c);
    if (xy ? (!std::isfinite(c.centerPos[0]) ||
        !std::isfinite(c.centerPos[1]) || (c.centerPos[0] == 0.0 && c.centerPos[1] == 0.0)) :
        (!Zero(c.centerPos[0]) || !Zero(c.centerPos[1]))) return false;
    if (MixedLayout(c) && (!std::isfinite(c.mem_transformMatrix[2][0]) ||
        (ZC(c) && c.mem_transformMatrix[2][0] == 0.0))) return false;
    const std::uint32_t mask = GroupMask(c, role);
    const int count = AxisCount(mask);
    if (c.axisCount != count) return false;
    for (int j = 0; j < MAX_AXES; ++j)
    {
        if (j < count)
        {
            const int axis = static_cast<int>(AxisForMask(mask, j));
            if (c.axisIndices[j] != axis || !std::isfinite(c.targetPos[j])) return false;
        }
        else if (c.axisIndices[j] != 0 || !Zero(c.targetPos[j])) return false;
    }
    return true;
}

BASE79B_TRANSPORT_NOINLINE inline bool PrepareMotionEccentricCCommand(const MotionCommand& c,
    NCEccentricCProfileValue& output, MotionEccentricCTransportWorkspace& workspace) noexcept
{
    MotionEccentricCTransportDetail::ResetInPlace(output);
    if (!IsMotionEccentricCFeedSourceAllowed(c)) return false;
    const unsigned role = MotionEccentricCTransportDetail::Role(c);
    const bool mixed = MotionEccentricCTransportDetail::MixedLayout(c);
    auto& input = workspace.decodeInput_;
    MotionEccentricCTransportDetail::ResetInPlace(input);
    input.geometry.source = c.sourceTranslation;
    input.geometry.sweepDeg = c.mem_radius;
    input.geometry.xDeltaMM = MotionEccentricCTransportDetail::AuthoredXY(c) ? c.centerPos[0] : 0.0;
    input.geometry.yDeltaMM = MotionEccentricCTransportDetail::AuthoredXY(c) ? c.centerPos[1] : 0.0;
    input.geometry.zDeltaMM = mixed ? c.mem_transformMatrix[2][0] : 0.0;
    input.geometry.feedDegMin = c.mem_startAngle;
    input.geometry.accTime = c.accTime; input.geometry.decTime = c.decTime;
    input.stopSeconds = c.mem_centerX; input.firSamples = 1U;
    input.travelMask = (mixed ? 7U : 3U) | (1U << role);
    for (unsigned a = 0U; a < 8U; ++a)
    {
        input.startPulse[a] = c.mem_startPos[a]; input.pulsePerNative[a] = c.mem_ratio[a];
        if (!std::isfinite(input.startPulse[a]) || !NCEccentricCDetail::Positive(input.pulsePerNative[a]))
            return false;
        input.geometry.startMCS[a] = input.startPulse[a] / input.pulsePerNative[a];
        input.travelMinNative[a] = input.travelMaxNative[a] = input.geometry.startMCS[a];
    }
    for (unsigned j = 0U; j < (mixed ? 4U : 3U); ++j)
    {
        const unsigned a = j < (mixed ? 3U : 2U) ? j : role;
        input.travelMinNative[a] = MotionEccentricCTransportDetail::Lower(c, a);
        input.travelMaxNative[a] = MotionEccentricCTransportDetail::Upper(c, a);
        input.geometry.maxVelocityNative[a] = MotionEccentricCTransportDetail::Limit(c, a, 0U);
        input.geometry.maxAccelerationNative[a] = MotionEccentricCTransportDetail::Limit(c, a, 1U);
        input.geometry.maxDecelerationNative[a] = MotionEccentricCTransportDetail::Limit(c, a, 2U);
        // Even zero eccentric radius cannot hide nonfinite unused XY limits.
        if (!NCEccentricCDetail::Positive(input.geometry.maxVelocityNative[a]) ||
            !NCEccentricCDetail::Positive(input.geometry.maxAccelerationNative[a]) ||
            !NCEccentricCDetail::Positive(input.geometry.maxDecelerationNative[a])) return false;
    }
    auto& runtime = workspace.decodeRuntime_;
    if (PrepareNCEccentricCRuntime(input, runtime) != NCEccentricCRuntimeCode::PREPARED ||
        !NCEccentricCDetail::Same(c.targetVel, runtime.MaximumScalarVelocityPulsePerSec())) return false;
    for (int j = 0; j < c.axisCount; ++j)
        if (!NCEccentricCDetail::Same(c.targetPos[j], runtime.EndPulse()[c.axisIndices[j]])) return false;
    return PrepareNCEccentricCProfile(runtime, output) == NCEccentricCProfileCode::PREPARED;
}

// Pure packet construction for source/transport validation. Successful return
// neither queues a command nor grants live admission. Future producer must
// independently obtain a real execution identity/owner and current frozen frame.
BASE79B_TRANSPORT_NOINLINE inline bool BuildMotionEccentricCCommand(const NCEccentricCRuntimeInput& input,
    const MotionExecutionIdentity& execution, const MotionOwnerLease& owner,
    MotionCommand& output, MotionEccentricCTransportWorkspace& workspace) noexcept
{
    // Existing command identity/owner may be passed by reference while the
    // destination is rebuilt. Snapshot them before clearing the destination.
    const MotionExecutionIdentity sourceExecution = execution;
    const MotionOwnerLease sourceOwner = owner;
    MotionEccentricCTransportDetail::ResetInPlace(output);
    auto& original = workspace.buildRuntime_;
    const bool xy = input.geometry.xDeltaMM != 0.0 || input.geometry.yDeltaMM != 0.0;
    const bool mixed = xy || input.geometry.zDeltaMM != 0.0;
    if (!std::isfinite(input.geometry.xDeltaMM) || !std::isfinite(input.geometry.yDeltaMM) ||
        !std::isfinite(input.geometry.zDeltaMM)) return false;
    if (mixed)
    {
        if (input.geometry.source.distanceMode != 90 && input.geometry.source.distanceMode != 91)
            return false;
        for (unsigned axis = 1U; axis < 3U; ++axis)
            if (!NCEccentricCDetail::Same(input.geometry.maxVelocityNative[axis], input.geometry.maxVelocityNative[0]) ||
                !NCEccentricCDetail::Same(input.geometry.maxAccelerationNative[axis], input.geometry.maxAccelerationNative[0]) ||
                !NCEccentricCDetail::Same(input.geometry.maxDecelerationNative[axis], input.geometry.maxDecelerationNative[0]))
                return false;
    }
    if (input.firSamples != 1U || input.cycleSeconds != 0.00025 ||
        PrepareNCEccentricCRuntime(input, original) != NCEccentricCRuntimeCode::PREPARED) return false;
    const auto& path = original.AuthoredPath();
    const unsigned role = path.electrodeAxis;
    auto& c = workspace.candidate_;
    MotionEccentricCTransportDetail::ResetInPlace(c);
    c.execution = sourceExecution; c.ownerLease = sourceOwner; c.sourceLinePC = sourceExecution.sourceBlockId;
    c.pathCoreEccentricCFeedExactStop = true;
    c.commandPathMode = MotionCommandPathMode::EXACT_STOP;
    c.sourceTranslation = input.geometry.source;
    c.sourceWCS = c.sourceTranslation.wcsCode;
    c.sourceToolLengthMode = c.sourceTranslation.toolLengthMode;
    c.sourceHCode = c.sourceTranslation.toolHCode;
    c.sourceIsAbsoluteMode = c.sourceTranslation.distanceMode == 90;
    c.targetVel = original.MaximumScalarVelocityPulsePerSec();
    c.accTime = input.geometry.accTime; c.decTime = input.geometry.decTime;
    c.dir = xy ? 3 : mixed ? 2 : 0;
    if (xy) { c.centerPos[0] = input.geometry.xDeltaMM; c.centerPos[1] = input.geometry.yDeltaMM; }
    c.axisCount = MotionEccentricCTransportDetail::AxisCount(path.groupMask);
    for (int j = 0; j < c.axisCount; ++j)
    {
        const unsigned a = MotionEccentricCTransportDetail::AxisForMask(path.groupMask, j);
        c.axisIndices[j] = static_cast<int>(a); c.targetPos[j] = original.EndPulse()[a];
    }
    for (unsigned a = 0U; a < 8U; ++a)
    { c.mem_startPos[a] = input.startPulse[a]; c.mem_ratio[a] = input.pulsePerNative[a]; }
    c.mem_radius = input.geometry.sweepDeg; c.mem_startAngle = input.geometry.feedDegMin;
    c.mem_centerX = input.stopSeconds;
    c.mem_centerY = input.travelMinNative[0]; c.mem_totalDist = input.travelMaxNative[0];
    c.mem_totalAngle = input.travelMinNative[1]; c.mem_transformOrigin[0] = input.travelMaxNative[1];
    c.mem_transformOrigin[1] = input.travelMinNative[role];
    c.mem_transformOrigin[2] = input.travelMaxNative[role];
    for (unsigned j = 0U; j < (mixed ? 2U : 3U); ++j)
    {
        const unsigned a = mixed ? (j == 0U ? 0U : role) : (j < 2U ? j : role);
        c.mem_transformMatrix[j][0] = input.geometry.maxVelocityNative[a];
        c.mem_transformMatrix[j][1] = input.geometry.maxAccelerationNative[a];
        c.mem_transformMatrix[j][2] = input.geometry.maxDecelerationNative[a];
    }
    if (mixed)
    {
        c.mem_transformMatrix[2][0] = input.geometry.zDeltaMM;
        c.mem_transformMatrix[2][1] = input.travelMinNative[2];
        c.mem_transformMatrix[2][2] = input.travelMaxNative[2];
    }
    if (!PrepareMotionEccentricCCommand(c, workspace.verifiedProfile_, workspace)) return false;
    output = c; return true;
}

#undef BASE79B_TRANSPORT_NOINLINE

// Canonical primitive-by-primitive digest: never hash structure padding.
// This is transport diagnostics, not a substitute for source/owner validation.
inline std::uint64_t FoldMotionEccentricCTransportFingerprint(std::uint64_t hash,
    const MotionCommand& c) noexcept
{
    const auto fold = [&hash](const void* pointer, std::size_t size) noexcept
    {
        const auto* bytes = static_cast<const unsigned char*>(pointer);
        for (std::size_t i = 0U; i < size; ++i)
        { hash ^= static_cast<std::uint64_t>(bytes[i]); hash *= 1099511628211ULL; }
    };
#define BASE79_FOLD(field) fold(&(field), sizeof(field))
    BASE79_FOLD(c.execution.epoch); BASE79_FOLD(c.execution.segmentId);
    BASE79_FOLD(c.execution.sourceBlockId); BASE79_FOLD(c.execution.source);
    BASE79_FOLD(c.ownerLease.owner); BASE79_FOLD(c.ownerLease.generation);
    BASE79_FOLD(c.mode); BASE79_FOLD(c.axisCount); BASE79_FOLD(c.axisIndices); BASE79_FOLD(c.targetPos);
    BASE79_FOLD(c.centerPos); BASE79_FOLD(c.startRadius); BASE79_FOLD(c.endRadius); BASE79_FOLD(c.dir);
    BASE79_FOLD(c.pathCorePlanarCircle); BASE79_FOLD(c.pathCoreFullCircle);
    BASE79_FOLD(c.pathCoreRetainedTraversal); BASE79_FOLD(c.pathCoreRetainedReverse);
    BASE79_FOLD(c.targetVel); BASE79_FOLD(c.accTime); BASE79_FOLD(c.decTime);
    BASE79_FOLD(c.replayTerminalAlreadyPublished); BASE79_FOLD(c.cncFeedLookahead); BASE79_FOLD(c.cncCornerBlend);
    BASE79_FOLD(c.pathCoreFeedExactStop); BASE79_FOLD(c.pathCoreRotaryFeedExactStop);
    BASE79_FOLD(c.pathCoreZCFeedExactStop); BASE79_FOLD(c.pathCoreXYZCFeedExactStop);
    BASE79_FOLD(c.pathCoreXYZCUVFeedExactStop); BASE79_FOLD(c.pathCoreEccentricCFeedExactStop);
    BASE79_FOLD(c.mem_startPos); BASE79_FOLD(c.mem_ratio); BASE79_FOLD(c.mem_radius);
    BASE79_FOLD(c.mem_startAngle); BASE79_FOLD(c.mem_centerX); BASE79_FOLD(c.mem_centerY);
    BASE79_FOLD(c.mem_totalDist); BASE79_FOLD(c.mem_totalAngle); BASE79_FOLD(c.mem_enableTransform);
    BASE79_FOLD(c.mem_transformOrigin); BASE79_FOLD(c.mem_transformMatrix);
    BASE79_FOLD(c.sourceLinePC); BASE79_FOLD(c.sourceWCS); BASE79_FOLD(c.sourceToolLengthMode);
    BASE79_FOLD(c.sourceHCode); BASE79_FOLD(c.sourceToolRadiusMode); BASE79_FOLD(c.sourceDCode);
    BASE79_FOLD(c.sourceIsAbsoluteMode); BASE79_FOLD(c.sourceG68Active); BASE79_FOLD(c.sourceG68Angle);
    BASE79_FOLD(c.sourceG168Active); BASE79_FOLD(c.sourceWCode); BASE79_FOLD(c.sourceG51Active);
    BASE79_FOLD(c.sourceScaleRatio); BASE79_FOLD(c.sourceMirrorMask); BASE79_FOLD(c.sourceG16Active);
    BASE79_FOLD(c.sourceG162Active); BASE79_FOLD(c.commandPathMode); BASE79_FOLD(c.sourcePlaneMode);
    BASE79_FOLD(c.cncPrefixVelocityPPS);
    const auto& s = c.sourceTranslation;
    BASE79_FOLD(s.runToken); BASE79_FOLD(s.generation); BASE79_FOLD(s.revision);
    BASE79_FOLD(s.wcsCode); BASE79_FOLD(s.schema); BASE79_FOLD(s.extOffsetMM); BASE79_FOLD(s.wcsOffsetMM);
    BASE79_FOLD(s.toolLengthMode); BASE79_FOLD(s.toolHCode); BASE79_FOLD(s.toolOffsetMM);
    BASE79_FOLD(s.workMode); BASE79_FOLD(s.workWCode); BASE79_FOLD(s.workOffset);
    BASE79_FOLD(s.rotationMode); BASE79_FOLD(s.rotationPlane); BASE79_FOLD(s.rotationCenterMM);
    BASE79_FOLD(s.rotationAngleDeg); BASE79_FOLD(s.workRotationCenterMM); BASE79_FOLD(s.distanceMode);
    BASE79_FOLD(s.unitsMode); BASE79_FOLD(s.scalingMode); BASE79_FOLD(s.mirrorMask);
    BASE79_FOLD(s.scalingFactor); BASE79_FOLD(s.scalingCenterMM); BASE79_FOLD(s.mirrorCenterMM);
    BASE79_FOLD(s.polarMode); BASE79_FOLD(s.storedStrokeMode); BASE79_FOLD(s.cutterMode);
    BASE79_FOLD(s.cutterD); BASE79_FOLD(s.cutterRadiusMM);
    const auto& a = s.axisIdentity;
    BASE79_FOLD(a.exists); BASE79_FOLD(a.axisType); BASE79_FOLD(a.nativeUnit);
    BASE79_FOLD(a.physicalIndexPlusOne); BASE79_FOLD(a.address); BASE79_FOLD(a.electrodeAxisPlusOne);
    BASE79_FOLD(a.systemMode); BASE79_FOLD(a.eccentricEnabled); BASE79_FOLD(a.bound); BASE79_FOLD(a.reserved);
#undef BASE79_FOLD
    return hash;
}
