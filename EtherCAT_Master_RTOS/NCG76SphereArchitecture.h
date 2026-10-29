#pragma once

#include "NCG76SphereSource.h"
#include "NCG76SphereHold.h"

// EDM286 P50 preparation facade. Runtime uses one separately owned analytic
// parent, never these 24-chord preview chunks as executable contours.
// A valid source/geometry/hold receipt is NOT a Motion/PLC ownership receipt.
// The production lane separately binds source PC/cache/run/owner, proves
// stopped transitions and native
// envelopes, and use the existing final mechanical-compensation send stage.
// Never enlarge the legacy 25/26-child P01..P27 compound to carry P50 layers.
namespace NCG76SphereArchitecture
{
    constexpr unsigned Revision = 286U;
    constexpr unsigned Mode = 50U;
    constexpr bool MotionExecutionReleased = true; // Scoped B0 AIR parent only.
    constexpr bool DischargeExecutionReleased = false;
    constexpr bool AutomaticHoldMotionReleased = true; // Same AIR parent only.

    enum class Status : unsigned char
    {
        Ok, SourceRejected, GeometryRejected, NativeRejected, InvalidPreparation
    };

    struct Preparation
    {
        NCG76SphereCoordinates::NativePlan native{};
        unsigned authoredXYZMask = 0U, requestedB = 0U;
        bool explicitB = false, inheritedB = false, valid = false;
        NCG76SphereSource::ModalB inheritedBSource{};
    };

    // Caller-owned NC planning scratch, never allocated on a 250 us servo tick.
    struct Workspace
    {
        NCG76SphereSource::Receipt source{};
        NCG76Sphere::Descriptor descriptor{};
        NCG76Sphere::Chunk programChunk{};
    };

    struct NativeEndpoint
    {
        NCG76SphereCoordinates::Point8 position{}, center{};
        NCG76Sphere::StepKind kind = NCG76Sphere::StepKind::InitialDepth;
        unsigned layerIndex = 0U, segmentIndex = 0U;
    };

    struct PreviewChunk
    {
        NCG76SphereCoordinates::Point8 start{};
        std::array<NativeEndpoint, NCG76Sphere::MaximumPoints> points{};
        unsigned count = 0U, layerIndex = 0U;
        double chordErrorBoundMm = 0.0;
        bool initial = false;
    };

    // Full pipeline: exact literal source -> program-space descriptor -> one
    // frozen native mapping. This does not consume/commit the source block.
    // Workspace is separate from output and is exclusively owned by the caller.
    inline Status Prepare(const NCParsedBlock& parsed,
        const NCTranslationSnapshot& snapshot,
        const NCG76SphereCoordinates::Point8& acceptedStartMCS,
        const NCG76Sphere::Limits& limits,
        const NCG76SphereSource::ModalB& modal,
        Preparation& output, Workspace& workspace) noexcept
    {
        // Source may refer to output.native.frame; freeze it before output reset.
        const NCG76Sphere::Limits frozenLimits = limits;
        const auto sourceStatus = NCG76SphereSource::Build(parsed, snapshot,
            acceptedStartMCS, modal, workspace.source);
        output = Preparation{};
        workspace.descriptor = NCG76Sphere::Descriptor{};
        NCG76Sphere::Detail::ClearChunk(workspace.programChunk);
        if (sourceStatus != NCG76SphereSource::Status::Ok) return Status::SourceRejected;
        if (!NCG76Sphere::Build(workspace.source.input, frozenLimits, workspace.descriptor))
            return Status::GeometryRejected;
        if (NCG76SphereCoordinates::BuildNative(workspace.source.frame,
            workspace.descriptor, output.native) != NCG76SphereCoordinates::Status::Ok)
            return Status::NativeRejected;
        output.authoredXYZMask = workspace.source.words.targetMask;
        output.requestedB = workspace.source.words.requestedB;
        output.explicitB = workspace.source.words.explicitB;
        output.inheritedB = workspace.source.words.inheritedB;
        output.inheritedBSource = workspace.source.words.inheritedBSource;
        output.valid = true;
        return Status::Ok;
    }

    inline Status Prepare(const NCParsedBlock& parsed,
        const NCTranslationSnapshot& snapshot,
        const NCG76SphereCoordinates::Point8& acceptedStartMCS,
        const NCG76Sphere::Limits& limits,
        Preparation& output, Workspace& workspace) noexcept
    {
        return Prepare(parsed,snapshot,acceptedStartMCS,limits,
            NCG76SphereSource::ModalB{},output,workspace);
    }

    inline bool Valid(const Preparation& prepared) noexcept
    {
        return prepared.valid && prepared.authoredXYZMask != 0U &&
            (prepared.authoredXYZMask & ~7U) == 0U && prepared.requestedB <= 4U &&
            (prepared.explicitB != prepared.inheritedB) &&
            (!prepared.inheritedB || (NCG76SphereSource::ValidModalB(prepared.inheritedBSource) &&
                prepared.requestedB == prepared.inheritedBSource.value &&
                prepared.inheritedBSource.current.run == prepared.native.frame.translation.runToken)) &&
            prepared.native.descriptor.input.radiusLaw == NCG76Sphere::RadiusLaw::Linear &&
            NCG76SphereCoordinates::Valid(prepared.native);
    }

    // Bounded streaming preview: two initial segments OR one 27-segment layer.
    // The 24 chords approximate a circle. The reported native sagitta must be
    // replaced by an approved arc/subdivision tolerance before contour release.
    inline Status BuildPreviewChunk(const Preparation& prepared, bool initial,
        unsigned layerIndex, PreviewChunk& output, Workspace& workspace) noexcept
    {
        output = PreviewChunk{};
        NCG76Sphere::Detail::ClearChunk(workspace.programChunk);
        if (!Valid(prepared) || (initial && layerIndex != 0U)) return Status::InvalidPreparation;
        const auto& descriptor = prepared.native.descriptor;
        const bool built = initial ? NCG76Sphere::BuildInitial(descriptor, workspace.programChunk) :
            NCG76Sphere::BuildLayer(descriptor, layerIndex, workspace.programChunk);
        if (!built) return Status::GeometryRejected;
        const auto& chunk = workspace.programChunk;
        const auto& frame = prepared.native.frame;
        NCG76SphereCoordinates::Point8 startCenter{};
        if (NCG76SphereCoordinates::EvaluateNative(prepared.native,
            NCG76Sphere::Cursor{initial, layerIndex, 0U}, 0.0, output.start, startCenter) !=
            NCG76SphereCoordinates::Status::Ok) return Status::NativeRejected;
        for (unsigned i = 0U; i < chunk.count; ++i)
        {
            auto& point = output.points[i];
            // Share the evaluator's segment/orbit precision fences with HOLD.
            if (NCG76SphereCoordinates::EvaluateNative(prepared.native,
                NCG76Sphere::Cursor{initial, layerIndex, i}, 1.0, point.position, point.center) !=
                NCG76SphereCoordinates::Status::Ok)
            {
                output = PreviewChunk{};
                return Status::NativeRejected;
            }
            point.kind = chunk.points[i].kind;
            point.layerIndex = chunk.points[i].layerIndex;
            point.segmentIndex = chunk.points[i].segmentIndex;
        }
        output.count = chunk.count;
        output.layerIndex = chunk.layerIndex;
        output.initial = chunk.initial;
        output.chordErrorBoundMm = chunk.chordErrorBoundMm * frame.nativeScale;
        if (!std::isfinite(output.chordErrorBoundMm))
        {
            output = PreviewChunk{};
            return Status::NativeRejected;
        }
        return Status::Ok;
    }

    static_assert(!DischargeExecutionReleased,
        "EDM286_P50_does_not_authorize_discharge");
}
