#pragma once

#include "EDMPhysicalRecipeSequence.h"
#include "EDMPhysicalBidirectionalFlushContract.h"
#include "EDMRecipeControl.h"
#include <cstdint>

// EDM79: detached owner-thread expectations for an explicitly pinned Q2
// sequence. These values cannot grant motion authority or modify a controller.
// NC owns run/cache/epoch/lease, completion and cancellation independently.
namespace EDM79
{
    struct Checkpoint
    {
        unsigned sourceOffset = 0U, profile = 0U;
        EDMRecipeControl::Key key{};
        EDM57::Recipe flush{};
        EDM59::Recipe servo{};
    };

    struct Plan
    {
        bool valid = false;
        unsigned count = 0U, terminalOffset = 0U;
        Checkpoint checkpoints[EDM77::MaximumFixtures]{};

        // Stored checkpoints are historical after invalidation. Consumers must
        // require valid/count before indexing; no large temporary or heap work.
        void Invalidate() noexcept
        { valid = false; count = terminalOffset = 0U; }
    };
    static_assert(sizeof(Checkpoint) <= 640U, "EDM79 checkpoint remains a bounded value.");
    static_assert(sizeof(Plan) <= 5120U, "EDM79 plan belongs in NC fixed storage, not the RT stack.");

    enum class MatchError : std::uint8_t
    { None, Source, RecipeKey, FlushBinding, ServoBinding, FlushRecipe, ServoRecipe };

    inline const char* MatchErrorName(MatchError error) noexcept
    {
        switch (error)
        {
        case MatchError::None: return "NONE";
        case MatchError::Source: return "SOURCE";
        case MatchError::RecipeKey: return "RECIPE_KEY";
        case MatchError::FlushBinding: return "FLUSH_BINDING";
        case MatchError::ServoBinding: return "SERVO_BINDING";
        case MatchError::FlushRecipe: return "FLUSH_RECIPE";
        case MatchError::ServoRecipe: return "SERVO_RECIPE";
        }
        return "UNKNOWN";
    }
    inline const char* ErrorName(MatchError error) noexcept
    { return MatchErrorName(error); }

    inline MatchError MatchCheckpoint(const Checkpoint& expected,
        unsigned sourceOffset, unsigned profile, const EDMRecipe::Controller& live,
        const EDM20::ProcessProfile& process, std::uint64_t tuningGeneration) noexcept
    {
        if ((profile != 61U && profile != 62U) ||
            sourceOffset > EDM77::MaximumPhysicalLines ||
            expected.sourceOffset != sourceOffset || expected.profile != profile)
            return MatchError::Source;
        const auto key = EDMRecipeControl::CaptureKey(live.Current());
        if (!EDMRecipeControl::IsKeyValid(expected.key) ||
            !EDMRecipeControl::IsKeyValid(key) ||
            !EDMRecipeControl::SameKey(expected.key, key))
            return MatchError::RecipeKey;
        const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(live, process.machineProfileId);
        if (!flush.ready) return MatchError::FlushBinding;
        const auto servo = EDM74::BindPhysicalBidirectionalRecipe(live, process, tuningGeneration);
        if (!servo.ready) return MatchError::ServoBinding;
        if (!EDM75::ValidateRecipe(expected.flush) ||
            !EDM57::SameRecipe(expected.flush, flush.recipe) ||
            !EDM75::SameAdmission(flush.recipe, servo.recipe))
            return MatchError::FlushRecipe;
        if (!EDM74::ValidateRecipe(expected.servo) ||
            !EDM59::SameRecipe(expected.servo, servo.recipe))
            return MatchError::ServoRecipe;
        return MatchError::None;
    }
}
