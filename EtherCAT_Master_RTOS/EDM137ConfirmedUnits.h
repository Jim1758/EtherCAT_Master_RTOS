#pragma once

#include "EDMFlushRecipeBinding.h"

// EDM137-only bridge for the four formerly unspecified fields. Existing
// installed catalogs may still describe them as raw. Only a new explicit
// actual entry since the current selection is interpreted as the user-defined
// percent/mm value. Stored values, stage presets and persisted overrides are
// never silently reinterpreted, rewritten or converted from an assumed unit.
// All other consumers retain their original confirmed-unit requirements.
namespace EDM137Units
{
inline bool LegacyExplicitDefinition(const EDMRecipe::FieldDefinition& field,
    const char* semantic, const char* unit) noexcept
{
    if (!field.enabled || field.unitConfirmed || field.decimals != 6U ||
        !EDM58::BindingDetail::LabelEquals(field.unit, sizeof(field.unit), "raw") ||
        !EDM58::BindingDetail::LabelEquals(field.semanticKey, sizeof(field.semanticKey), semantic)) return false;
    const char* expectedSemantic = nullptr; const char* expectedUnit = nullptr;
    EDMRecipe::Value minimum = 0LL, maximum = 0LL;
    switch (field.id)
    {
    case 11U: expectedSemantic = "FinalApproachSpeed"; expectedUnit = "%";
        minimum = 10000000LL; maximum = 100000000LL; break;
    case 13U: expectedSemantic = "DecelerationDistance"; expectedUnit = "mm";
        minimum = 0LL; maximum = 100000000LL; break;
    case 19U: expectedSemantic = "LargeAreaMachiningSpeed"; expectedUnit = "%";
        minimum = 1000000LL; maximum = 100000000LL; break;
    case 20U: expectedSemantic = "LargeAreaFlushDistance"; expectedUnit = "mm";
        minimum = 0LL; maximum = 900000000LL; break;
    default: return false;
    }
    return field.minimum == minimum && field.maximum == maximum &&
        EDM58::BindingDetail::LabelEquals(field.semanticKey, sizeof(field.semanticKey), expectedSemantic) &&
        std::strcmp(unit, expectedUnit) == 0;
}

inline bool ReadExplicit(const EDMRecipe::Controller& controller, const EDMRecipe::Catalog& catalog,
    const char* semantic, const char* unit, EDMRecipe::Value& value,
    std::uint16_t& fieldId, EDM58::Binding& out) noexcept
{
    // Preserve every original error and normal typed-catalog rule first.
    if (EDM58::BindingDetail::Read(controller, catalog, semantic, unit, value, fieldId, out)) return true;
    if (out.error != EDM58::BindingError::UnitUnconfirmed) return false;
    const EDMRecipe::FieldDefinition* match = nullptr;
    for (const auto& field : catalog.fields)
    {
        if (!EDM58::BindingDetail::LabelEquals(field.semanticKey, sizeof(field.semanticKey), semantic)) continue;
        if (match) { out.error = EDM58::BindingError::DuplicateSemantic; out.failedFieldId = field.id; return false; }
        match = &field;
    }
    if (!match || !LegacyExplicitDefinition(*match, semantic, unit) ||
        !controller.HasExplicitActualSinceSelection(match->id)) return false;
    EDMRecipe::ParameterSnapshot snapshot{};
    if (!controller.ReadField(match->id, snapshot) || snapshot.fieldId != match->id || !snapshot.configured ||
        !snapshot.overridePresent || (snapshot.rowKind != EDMRecipe::CellKind::Direct &&
        snapshot.rowKind != EDMRecipe::CellKind::Stage))
    { out.error = EDM58::BindingError::Unconfigured; return false; }
    if (snapshot.actualValue < match->minimum || snapshot.actualValue > match->maximum)
    { out.error = EDM58::BindingError::ValueRange; return false; }
    if (!EDMRecipe::ValueMatchesPrecision(snapshot.actualValue, match->decimals))
    { out.error = EDM58::BindingError::ValuePrecision; return false; }
    value = snapshot.actualValue; fieldId = match->id;
    out.error = EDM58::BindingError::None; out.failedFieldId = 0U;
    return true;
}
}
