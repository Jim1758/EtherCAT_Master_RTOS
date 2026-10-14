#pragma once

#include "EDMPhysicalRecipeSequence.h"
#include "EDMPhysicalBidirectionalFlushContract.h"
#include "EDMRecipeControl.h"
#include "EDMPhysicalRecipeSequencePlan.h"
#include <cstdint>

// EDM78: check every stopped recipe transition before the first physical Arm.
// This bounded owner-thread preview neither commits RAM edits nor grants motion
// authority. Existing per-leg capture and released-origin admission remain the
// authority. The caller owns reusable Controller storage outside the RT stack.
namespace EDM78
{
    enum class Error : std::uint8_t
    {
        None, Syntax, PreviewUnavailable, SourceNotReady, Selection,
        Operation, FlushBinding, ServoBinding, CombinedBinding, SourceChanged
    };
    inline const char* ErrorName(Error error) noexcept
    {
        switch (error)
        {
        case Error::None: return "NONE";
        case Error::Syntax: return "SYNTAX";
        case Error::PreviewUnavailable: return "PREVIEW_UNAVAILABLE";
        case Error::SourceNotReady: return "SOURCE_NOT_READY";
        case Error::Selection: return "SELECTION";
        case Error::Operation: return "OPERATION";
        case Error::FlushBinding: return "FLUSH_BINDING";
        case Error::ServoBinding: return "SERVO_BINDING";
        case Error::CombinedBinding: return "COMBINED_BINDING";
        case Error::SourceChanged: return "SOURCE_CHANGED";
        }
        return "UNKNOWN";
    }
    struct Result
    {
        bool valid = false;
        Error error = Error::None;
        EDM77::Error syntaxError = EDM77::Error::None;
        EDMRecipe::Error recipeError = EDMRecipe::Error::None;
        EDMRecipeControl::Reason controlError = EDMRecipeControl::Reason::None;
        EDM58::BindingError flushError = EDM58::BindingError::None;
        EDM59::BindingError servoError = EDM59::BindingError::None;
        std::uint16_t failedFieldId = 0U;
        unsigned errorOffset = 0U, fixtureOrdinal = 1U, profile = 0U;
        unsigned fixtureCount = 0U, operationCount = 0U, terminalOffset = 0U;
        std::uint64_t initialGeneration = 0ULL, finalGeneration = 0ULL;
    };
    namespace Detail
    {
        struct PlanLifetime
        {
            EDM79::Plan* output;
            const Result& result;
            PlanLifetime(EDM79::Plan* value, const Result& status) noexcept : output(value), result(status)
            { if (output) output->Invalidate(); }
            ~PlanLifetime() noexcept
            { if (output && !result.valid) output->Invalidate(); }
            PlanLifetime(const PlanLifetime&) = delete;
            PlanLifetime& operator=(const PlanLifetime&) = delete;
        };
        struct PreviewLifetime
        {
            EDMRecipe::Controller& controller;
            explicit PreviewLifetime(EDMRecipe::Controller& value) noexcept : controller(value) {}
            ~PreviewLifetime() noexcept { controller.DiscardPreview(); }
            PreviewLifetime(const PreviewLifetime&) = delete;
            PreviewLifetime& operator=(const PreviewLifetime&) = delete;
        };
        inline bool BindFixture(const EDMRecipe::Controller& preview,
            const NCParsedBlock& block, const EDM20::ProcessProfile& process,
            std::uint64_t tuningGeneration, Result& result,
            unsigned sourceOffset, EDM79::Plan* output) noexcept
        {
            std::uint32_t profile = 0U;
            if (!EDM77::IsSequenceFixture(block) ||
                !EDM77::Detail::UnsignedLiteral(block.expression('P'), 62U, profile))
            { result.error = Error::SourceChanged; return false; }
            result.profile = profile;
            const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(preview, process.machineProfileId);
            if (!flush.ready)
            {
                result.error = Error::FlushBinding;
                result.flushError = flush.error;
                result.failedFieldId = flush.failedFieldId;
                return false;
            }
            const auto servo = EDM74::BindPhysicalBidirectionalRecipe(preview, process, tuningGeneration);
            if (!servo.ready)
            {
                result.error = Error::ServoBinding;
                result.servoError = servo.error;
                result.failedFieldId = servo.failedFieldId;
                return false;
            }
            if (!EDM75::SameAdmission(flush.recipe, servo.recipe))
            { result.error = Error::CombinedBinding; return false; }
            if (output)
            {
                const auto key = EDMRecipeControl::CaptureKey(preview.Current());
                if (output->count >= EDM77::MaximumFixtures || !EDMRecipeControl::IsKeyValid(key))
                { result.error = Error::SourceChanged; return false; }
                auto& checkpoint = output->checkpoints[output->count];
                checkpoint.sourceOffset = sourceOffset;
                checkpoint.profile = static_cast<unsigned>(profile);
                checkpoint.key = key;
                checkpoint.flush = flush.recipe;
                checkpoint.servo = servo.recipe;
                ++output->count;
            }
            ++result.fixtureCount;
            return true;
        }
        inline bool ApplyOperation(EDMRecipe::Controller& preview,
            const NCParsedBlock& block, unsigned offset, Result& result) noexcept
        {
            bool mutation = false;
            if (!EDM77::Detail::Clean(block) || !EDM77::Detail::Operation(block, mutation))
            { result.error = Error::SourceChanged; return false; }
            std::uint32_t value = 0U;
            if (block.gCount == 0)
            {
                if (!EDM77::Detail::UnsignedLiteral(block.expression('E'),
                    static_cast<std::uint32_t>(EDMRecipe::RowsPerTable), value))
                { result.error = Error::SourceChanged; return false; }
                // NC's stopped selection prepare also checks row presence.
                // Valid installed catalogs have every row, but keep replay
                // equivalent if that invariant is ever violated.
                const auto* catalog = preview.GetCatalog();
                const EDMRecipe::Table* selected = nullptr;
                if (catalog)
                    for (const auto& table : catalog->tables)
                        if (table.id == preview.Current().tableId) { selected = &table; break; }
                if (!selected || !selected->rowPresent[value - 1U])
                {
                    result.error = Error::Selection;
                    result.recipeError = selected ? EDMRecipe::Error::EmptyRow : EDMRecipe::Error::InvalidTable;
                    return false;
                }
                if (!preview.SelectE(static_cast<std::uint16_t>(value)))
                { result.error = Error::Selection; result.recipeError = preview.Current().lastError; return false; }
            }
            else if (EDM77::Detail::EqualLiteral(block.gExpressions[0], 38U))
            {
                if (!EDM77::Detail::UnsignedLiteral(block.expression('P'),
                    EDMRecipe::MaximumConditionTableId, value))
                { result.error = Error::SourceChanged; return false; }
                if (!preview.SelectTableAndE1(value))
                { result.error = Error::Selection; result.recipeError = preview.Current().lastError; return false; }
            }
            else
            {
                EDMRecipeControl::Request request{};
                request.requestId = offset;
                if (!EDM77::Detail::UnsignedLiteral(block.expression('P'), 65535U, value))
                { result.error = Error::SourceChanged; return false; }
                request.fieldId = static_cast<std::uint16_t>(value);
                if (!EDM77::Detail::UnsignedLiteral(block.expression('Q'), 4U, value))
                { result.error = Error::SourceChanged; return false; }
                request.op = static_cast<EDMRecipeControl::Operation>(value);
                if (request.op == EDMRecipeControl::Operation::SelectStage)
                {
                    if (!EDM77::Detail::UnsignedLiteral(block.expression('R'), 65535U, value))
                    { result.error = Error::SourceChanged; return false; }
                    request.argument = static_cast<EDMRecipe::Value>(value);
                }
                else if (request.op == EDMRecipeControl::Operation::StepStage)
                    request.argument = block.expression('R') == "-1" ? -1LL : 1LL;
                else if (request.op == EDMRecipeControl::Operation::SetActual &&
                    !EDMRecipe::ParseValue(block.expression('R').c_str(), request.argument))
                { result.error = Error::SourceChanged; return false; }
                EDMRecipeControl::Result applied{};
                if (!EDMRecipeControl::Apply(preview,
                    EDMRecipeControl::CaptureKey(preview.Current()), request, applied))
                {
                    result.error = Error::Operation;
                    result.recipeError = applied.domainError;
                    result.controlError = applied.reason;
                    result.failedFieldId = request.fieldId;
                    return false;
                }
            }
            ++result.operationCount;
            return true;
        }
    }

    // get(offset) must return stable parsed source without allocation or side
    // effects, as in EDM77::ValidateSuffix. Calls are synchronous on the recipe
    // owner thread; the installed catalog and process profile cannot exchange
    // while replay runs. A valid result is advisory, never an Arm permission.
    // Optional EDM79 output is filled in caller-owned fixed storage. Only a
    // fully accepted suffix publishes valid=true; every failure clears its
    // count/terminalOffset. The caller decides whether Q2 pins that plan.
    template<class Getter>
    Result Preflight(const EDMRecipe::Controller& live, EDMRecipe::Controller& preview,
        const NCParsedBlock& current, Getter get, const EDM20::ProcessProfile& process,
        std::uint64_t tuningGeneration, EDM79::Plan* output = nullptr) noexcept
    {
        Result result{};
        Detail::PlanLifetime planLifetime(output, result);
        result.initialGeneration = live.Current().generation;
        result.finalGeneration = result.initialGeneration;
        if (&live == &preview)
        { result.error = Error::PreviewUnavailable; return result; }
        preview.DiscardPreview();
        const auto syntax = EDM77::ValidateSuffix(current, get);
        result.terminalOffset = syntax.terminalOffset;
        if (!syntax.valid)
        {
            result.error = Error::Syntax;
            result.syntaxError = syntax.error;
            result.errorOffset = syntax.errorOffset;
            return result;
        }
        if (!preview.ForkPreviewFrom(live))
        {
            result.error = live.Ready() ? Error::PreviewUnavailable : Error::SourceNotReady;
            result.recipeError = live.Current().lastError;
            return result;
        }
        Detail::PreviewLifetime lifetime(preview);
        if (!Detail::BindFixture(preview, current, process, tuningGeneration, result, 0U, output))
            return result;
        const unsigned sequenceMode = EDM77::SequenceMode(current);
        for (unsigned offset = 1U; offset <= syntax.terminalOffset; ++offset)
        {
            const auto* block = get(offset);
            result.errorOffset = offset;
            result.fixtureOrdinal = result.fixtureCount + 1U;
            if (!block || !EDM77::Detail::Clean(*block))
            { result.error = Error::SourceChanged; break; }
            if (EDM77::Detail::Empty(*block)) continue;
            if (EDM77::Detail::Terminal(*block))
            {
                if (offset != syntax.terminalOffset || result.fixtureCount != syntax.futureFixtures + 1U)
                { result.error = Error::SourceChanged; break; }
                result.valid = true;
                result.errorOffset = 0U;
                result.fixtureOrdinal = 0U;
                break;
            }
            if (EDM77::IsSequenceFixture(*block))
            {
                if (EDM77::SequenceMode(*block) != sequenceMode)
                { result.error = Error::SourceChanged; break; }
                if (!Detail::BindFixture(preview, *block, process, tuningGeneration, result, offset, output)) break;
            }
            else if (!Detail::ApplyOperation(preview, *block, offset, result)) break;
        }
        result.finalGeneration = preview.Current().generation;
        if (!result.valid && result.error == Error::None) result.error = Error::SourceChanged;
        if (output && result.valid)
        {
            output->terminalOffset = result.terminalOffset;
            output->valid = true;
        }
        // No borrowed catalog or potentially invalid RAM remains available to
        // accidental consumers after either acceptance or rejection.
        return result;
    }
}
