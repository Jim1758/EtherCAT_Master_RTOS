#pragma once

#include "EDMRecipe.h"

// EDM304: the machine owner confirmed percent for E11/E19 and mm for every
// distance field. Interpret only this machine profile's exact former seed
// definitions. This is a metadata correction: values, presets, overrides and
// persisted files are never converted or rewritten.
namespace EDMConditionUnits
{
inline bool NormalizeLegacyField(std::uint32_t schemaVersion, std::uint32_t profileId,
    EDMRecipe::FieldDefinition& field) noexcept
{
    if (schemaVersion != 2U || profileId != 1U || !field.enabled ||
        field.unitConfirmed || field.decimals != 6U ||
        std::strncmp(field.unit, "raw", sizeof(field.unit)) != 0) return false;
    const char* semantic = nullptr; const char* unit = nullptr;
    EDMRecipe::Value minimum = 0LL, maximum = 0LL;
    switch (field.id)
    {
    case 11U: semantic = "FinalApproachSpeed"; unit = "%";
        minimum = 10000000LL; maximum = 100000000LL; break;
    case 13U: semantic = "DecelerationDistance"; unit = "mm";
        maximum = 100000000LL; break;
    case 14U: semantic = "SlowRetractDistance"; unit = "mm";
        maximum = 100000000LL; break;
    case 19U: semantic = "LargeAreaMachiningSpeed"; unit = "%";
        minimum = 1000000LL; maximum = 100000000LL; break;
    case 20U: semantic = "LargeAreaFlushDistance"; unit = "mm";
        maximum = 900000000LL; break;
    default: return false;
    }
    if (field.minimum != minimum || field.maximum != maximum ||
        std::strncmp(field.semanticKey, semantic, sizeof(field.semanticKey)) != 0) return false;
    std::memset(field.unit, 0, sizeof(field.unit));
    std::strcpy(field.unit, unit);
    field.unitConfirmed = true;
    return true;
}
}
