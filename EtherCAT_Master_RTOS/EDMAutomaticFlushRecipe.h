#pragma once

#include "EDMRecipe.h"
#include "EDMAutomaticFlushCycle.h"

#include <cstdint>

// EDM23 SHADOW_ONLY. This adapter reads only the selected COND row's actual
// values, including stage overrides. Numeric column IDs are never assumed.
// Owner-thread only; bounded work, no allocation, mutation, I/O or hardware.
namespace EDM23
{
    enum class RecipeBindingError : std::uint8_t
    {
        None, NotReady, UnsupportedSchema, ProfileMismatch, NoSelection,
        InvalidIdentity, MissingSemantic, DuplicateSemantic, FieldDisabled,
        UnitUnconfirmed, UnitMismatch, Unconfigured, ValueRange, ValuePrecision,
        ZeroWorkTime, EngineeringRange, InvalidDeepFlushMultiplier
    };

    inline const char* RecipeBindingErrorName(RecipeBindingError error) noexcept
    {
        switch (error)
        {
        case RecipeBindingError::None: return "NONE";
        case RecipeBindingError::NotReady: return "NOT_READY";
        case RecipeBindingError::UnsupportedSchema: return "UNSUPPORTED_SCHEMA";
        case RecipeBindingError::ProfileMismatch: return "PROFILE_MISMATCH";
        case RecipeBindingError::NoSelection: return "NO_SELECTION";
        case RecipeBindingError::InvalidIdentity: return "INVALID_IDENTITY";
        case RecipeBindingError::MissingSemantic: return "MISSING_SEMANTIC";
        case RecipeBindingError::DuplicateSemantic: return "DUPLICATE_SEMANTIC";
        case RecipeBindingError::FieldDisabled: return "FIELD_DISABLED";
        case RecipeBindingError::UnitUnconfirmed: return "UNIT_UNCONFIRMED";
        case RecipeBindingError::UnitMismatch: return "UNIT_MISMATCH";
        case RecipeBindingError::Unconfigured: return "UNCONFIGURED";
        case RecipeBindingError::ValueRange: return "VALUE_RANGE";
        case RecipeBindingError::ValuePrecision: return "VALUE_PRECISION";
        case RecipeBindingError::ZeroWorkTime: return "ZERO_WORK_TIME";
        case RecipeBindingError::EngineeringRange: return "ENGINEERING_RANGE";
        case RecipeBindingError::InvalidDeepFlushMultiplier: return "INVALID_DEEP_FLUSH_MULTIPLIER";
        }
        return "UNKNOWN";
    }

    struct RecipeBindingSnapshot
    {
        bool ready = false;
        RecipeBindingError error = RecipeBindingError::NotReady;
        std::uint16_t failedFieldId = 0U;
        std::uint32_t tableId = 0U;
        std::uint16_t eCode = 0U;
        std::uint64_t generation = 0ULL;
        std::uint64_t workTimeMs = 0ULL;
        double jumpHeightMm = 0.0;
        double speedOverridePercent = 0.0;
        std::uint32_t deepFlushCycleInterval = 0U;
        double deepFlushHeightMultiplier = 0.0;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    namespace RecipeBindingDetail
    {
        inline bool LabelEquals(const char* label, std::size_t capacity, const char* expected) noexcept
        {
            for (std::size_t i = 0U; i < capacity; ++i)
            {
                if (label[i] != expected[i]) return false;
                if (label[i] == '\0') return true;
            }
            return false;
        }

        inline bool Read(const EDMRecipe::Controller& controller, const EDMRecipe::Catalog& catalog,
            const char* semantic, const char* unit, EDMRecipe::Value& value,
            RecipeBindingSnapshot& output) noexcept
        {
            const EDMRecipe::FieldDefinition* match = nullptr;
            for (std::size_t i = 0U; i < catalog.fields.size(); ++i)
            {
                const EDMRecipe::FieldDefinition& field = catalog.fields[i];
                if (!LabelEquals(field.semanticKey, sizeof(field.semanticKey), semantic)) continue;
                if (match != nullptr)
                {
                    output.error = RecipeBindingError::DuplicateSemantic;
                    output.failedFieldId = field.id;
                    return false;
                }
                match = &field;
            }
            if (match == nullptr) { output.error = RecipeBindingError::MissingSemantic; return false; }
            output.failedFieldId = match->id;
            if (!match->enabled) { output.error = RecipeBindingError::FieldDisabled; return false; }
            if (!match->unitConfirmed) { output.error = RecipeBindingError::UnitUnconfirmed; return false; }
            if (!LabelEquals(match->unit, sizeof(match->unit), unit))
            { output.error = RecipeBindingError::UnitMismatch; return false; }
            EDMRecipe::ParameterSnapshot parameter{};
            if (!controller.ReadField(match->id, parameter) || !parameter.configured ||
                parameter.fieldId != match->id ||
                (parameter.rowKind != EDMRecipe::CellKind::Direct && parameter.rowKind != EDMRecipe::CellKind::Stage))
            { output.error = RecipeBindingError::Unconfigured; return false; }
            value = parameter.actualValue;
            if (value < match->minimum || value > match->maximum)
            { output.error = RecipeBindingError::ValueRange; return false; }
            if (!EDMRecipe::ValueMatchesPrecision(value, match->decimals))
            { output.error = RecipeBindingError::ValuePrecision; return false; }
            output.failedFieldId = 0U;
            return true;
        }
    }

    inline RecipeBindingSnapshot BindRecipe(const EDMRecipe::Controller& controller,
        std::uint32_t expectedProfileId) noexcept
    {
        RecipeBindingSnapshot output{};
        const EDMRecipe::Summary summary = controller.Current();
        output.tableId = summary.tableId;
        output.eCode = summary.eCode;
        output.generation = summary.generation;
        const EDMRecipe::Catalog* catalog = controller.GetCatalog();
        if (catalog == nullptr || !summary.catalogReady) return output;
        if (catalog->schemaVersion != 2U || catalog->mode != EDMRecipe::Mode::ShadowOnly)
        { output.error = RecipeBindingError::UnsupportedSchema; return output; }
        if (expectedProfileId == 0U || catalog->profileId != expectedProfileId || summary.profileId != expectedProfileId)
        { output.error = RecipeBindingError::ProfileMismatch; return output; }
        if (!summary.tableSelected || !summary.rowSelected)
        { output.error = RecipeBindingError::NoSelection; return output; }
        if (summary.generation == 0ULL || summary.tableId == 0U ||
            summary.tableId > EDMRecipe::MaximumConditionTableId || summary.eCode == 0U ||
            summary.eCode > EDMRecipe::RowsPerTable || catalog->catalogRevision == 0U ||
            catalog->definitionRevision == 0U || summary.catalogRevision != catalog->catalogRevision ||
            summary.definitionRevision != catalog->definitionRevision ||
            catalog->fields.empty() || catalog->fields.size() > EDMRecipe::MaximumFields)
        { output.error = RecipeBindingError::InvalidIdentity; return output; }

        EDMRecipe::Value work = 0LL, height = 0LL, speed = 0LL, interval = 0LL, multiplier = 0LL;
        if (!RecipeBindingDetail::Read(controller, *catalog, "DischargeWorkTime", "s", work, output) ||
            !RecipeBindingDetail::Read(controller, *catalog, "JumpHeight", "mm", height, output) ||
            !RecipeBindingDetail::Read(controller, *catalog, "JumpSpeedOverride", "%", speed, output) ||
            !RecipeBindingDetail::Read(controller, *catalog, "DeepFlushCycleInterval", "cycle", interval, output) ||
            !RecipeBindingDetail::Read(controller, *catalog, "DeepFlushHeightMultiplier", "x", multiplier, output))
            return output;
        if (work == 0LL) { output.error = RecipeBindingError::ZeroWorkTime; return output; }
        // Match the automatic shadow model's caps, independent of custom field ranges.
        // The interval is a count even if a custom catalog permits decimals.
        if (work < 0LL || work > 10LL * EDMRecipe::Scale || height < 0LL || height > 100LL * EDMRecipe::Scale ||
            speed < 0LL || speed > 100LL * EDMRecipe::Scale || interval < 0LL || interval > 20LL * EDMRecipe::Scale ||
            interval % EDMRecipe::Scale != 0LL || multiplier < 0LL || multiplier > 20LL * EDMRecipe::Scale)
        { output.error = RecipeBindingError::EngineeringRange; return output; }
        if (interval > 0LL && multiplier == 0LL)
        { output.error = RecipeBindingError::InvalidDeepFlushMultiplier; return output; }

        // Fixed-point seconds contain exact microseconds. Ceiling prevents a
        // positive sub-millisecond E5 from silently becoming zero or firing early.
        output.workTimeMs = static_cast<std::uint64_t>(work / 1000LL + (work % 1000LL != 0LL ? 1LL : 0LL));
        output.jumpHeightMm = static_cast<double>(height) / static_cast<double>(EDMRecipe::Scale);
        output.speedOverridePercent = static_cast<double>(speed) / static_cast<double>(EDMRecipe::Scale);
        output.deepFlushCycleInterval = static_cast<std::uint32_t>(interval / EDMRecipe::Scale);
        output.deepFlushHeightMultiplier = static_cast<double>(multiplier) / static_cast<double>(EDMRecipe::Scale);
        output.error = RecipeBindingError::None;
        output.ready = true;
        return output;
    }

    // Turn the selected COND row's frozen actual values into the same model
    // config used by the automatic shadow cycle. The caller must explicitly
    // provide the virtual frame, B0/B1 policy and confirmed height permission;
    // this adapter never infers a physical machining direction or E11 units.
    // Failure preserves output. A successful result has passed model admission,
    // including both ordinary and deep-flush plans, without starting hardware.
    inline bool MakeAutomaticFlushCycleConfig(const RecipeBindingSnapshot& binding,
        const EDM20::FlushProfile& profile, std::uint64_t configRevision,
        const EDM20::FlushRequest& explicitVirtualRequest,
        AutomaticFlushCycleConfig& output) noexcept
    {
        if (!binding.ready || binding.error != RecipeBindingError::None ||
            binding.generation == 0ULL || binding.tableId == 0U ||
            binding.tableId > EDMRecipe::MaximumConditionTableId ||
            binding.eCode == 0U || binding.eCode > EDMRecipe::RowsPerTable) return false;
        AutomaticFlushCycleConfig candidate{};
        candidate.workTimeMs = binding.workTimeMs;
        candidate.baseJumpHeightMm = binding.jumpHeightMm;
        candidate.jumpSpeedOverridePercent = binding.speedOverridePercent;
        candidate.deepFlushCycleInterval = binding.deepFlushCycleInterval;
        candidate.deepFlushHeightMultiplier = binding.deepFlushHeightMultiplier;
        candidate.flushProfile = profile;
        candidate.flushRequest = explicitVirtualRequest;
        candidate.flushRequest.jumpHeightMm = binding.jumpHeightMm;
        candidate.flushRequest.speedOverridePercent = binding.speedOverridePercent;
        candidate.configRevision = configRevision;
        candidate.recipeGeneration = binding.generation;
        AutomaticFlushCycle admission{};
        if (!admission.Start(candidate, 0ULL)) return false;
        output = candidate;
        return true;
    }
}
