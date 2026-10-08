#pragma once
#include "EDMRecipe.h"
#include <cstdint>
#include <limits>

// EDM16 shadow-only tests. Bind discovers fixtures once at startup; Step uses
// only fixed owner-thread state and the installed immutable catalog.
namespace EDMRecipeSimulationTest
{
    constexpr std::uint32_t CaseCount = 16U;
    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "TABLE_FIRST", "E_FIRST", "E_LAST", "TABLE_SWITCH", "SAME_TABLE_CLEARS",
            "E_RESELECT_RESETS", "STAGE_SELECT", "STAGE_UP_DOWN", "STAGE_BOUNDARY",
            "ACTUAL_NON_STAGE", "ACTUAL_KEEPS_ANCHOR", "STAGE_CLEARS_OVERRIDE",
            "RESET_OVERRIDE", "DIRECT_NEEDS_ANCHOR", "INVALID_SELECTION", "FINAL_BASELINE"
        };
        return index < CaseCount ? names[index] : "UNKNOWN";
    }
    struct CaseResult
    {
        EDMRecipe::Summary actual{};
        EDMRecipe::ParameterSnapshot field{};
        std::uint32_t checks = 0U;
        bool passed = false;
    };
    class Scenario
    {
    public:
        bool Bind(const EDMRecipe::CatalogStore& store) noexcept
        {
            if (!m_controller.Bind(store)) return false;
            const auto* catalog = m_controller.GetCatalog();
            if (!catalog || catalog->tables.size() < 2U) return false;
            const std::size_t count = catalog->fields.size();
            for (const auto& table : catalog->tables)
            {
                for (std::size_t row = 0U; row < EDMRecipe::RowsPerTable; ++row)
                {
                    std::uint16_t direct = 0U;
                    const EDMRecipe::FieldDefinition* staged = nullptr;
                    for (std::size_t f = 0U; f < count; ++f)
                    {
                        const auto& cell = table.cells[row * count + f];
                        if (cell.kind == EDMRecipe::CellKind::Direct && !direct) direct = catalog->fields[f].id;
                        if (cell.kind != EDMRecipe::CellKind::Unset && catalog->fields[f].stages.size() >= 2U && !staged)
                            staged = &catalog->fields[f];
                    }
                    if (direct && staged && FindNonStage(*staged, m_nonStage))
                    {
                        m_table = table.id; m_firstE = static_cast<std::uint16_t>(row + 1U);
                        m_stageField = staged->id; m_directField = direct;
                        m_stage0 = staged->stages[0]; m_stage1 = staged->stages[1];
                        break;
                    }
                }
                if (m_table) break;
            }
            if (!m_table) return false;
            // Prefer an actual configured direct value from this catalog as
            // the non-stage tuning example instead of inventing a setpoint.
            for (const auto& table : catalog->tables)
                if (table.id == m_table)
                    for (std::size_t f = 0U; f < count; ++f)
                        if (catalog->fields[f].id == m_stageField)
                            for (std::size_t row = 0U; row < EDMRecipe::RowsPerTable; ++row)
                            {
                                const auto& cell = table.cells[row * count + f];
                                if (cell.kind != EDMRecipe::CellKind::Direct) continue;
                                bool isStage = false;
                                for (const auto& stage : catalog->fields[f].stages) isStage = isStage || stage.value == cell.value;
                                if (!isStage) { m_nonStage = cell.value; break; }
                            }
            for (const auto& table : catalog->tables)
            {
                for (std::size_t row = 0U; row < EDMRecipe::RowsPerTable; ++row)
                {
                    bool configured = false;
                    for (std::size_t f = 0U; f < count; ++f)
                        configured = configured || table.cells[row * count + f].kind != EDMRecipe::CellKind::Unset;
                    if (table.id == m_table)
                    {
                        if (configured) m_lastE = static_cast<std::uint16_t>(row + 1U);
                        else if (!m_emptyE) m_emptyE = static_cast<std::uint16_t>(row + 1U);
                    }
                    else if (configured && !m_otherTable)
                    {
                        m_otherTable = table.id; m_otherE = static_cast<std::uint16_t>(row + 1U);
                    }
                }
            }
            m_ready = m_otherTable && m_lastE && m_emptyE;
            return m_ready;
        }
        bool Ready() const noexcept { return m_ready && m_controller.Ready(); }
        bool Restart() noexcept { Revoke(); m_nextCase = 0U; m_live = Ready(); return m_live; }
        void Revoke() noexcept
        {
            if (m_live || m_controller.Current().tableSelected) m_controller.Clear();
            m_live = false;
        }
        EDMRecipe::Summary Current() const noexcept { return m_controller.Current(); }
        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDMRecipe;
            CaseResult result{};
            result.passed = m_live && index == m_nextCase && index < CaseCount;
            const auto check = [&result](bool ok) { ++result.checks; result.passed = result.passed && ok; };
            ParameterSnapshot field{};
            const auto read = [this, &field]() { return m_controller.ReadField(m_stageField, field); };
            if (!result.passed) { Revoke(); result.actual = Current(); return result; }
            check(Base());
            if (!result.passed) { Revoke(); result.actual = Current(); return result; }
            switch (index)
            {
            case 0U:
                check(m_controller.SelectTable(m_table));
                check(Current().tableSelected && !Current().rowSelected && !m_controller.ReadField(m_stageField, field));
                break;
            case 1U:
                check(Current().eCode == m_firstE && Current().rowSelected && Current().modifiedCount == 0U);
                check(read() && field.actualValue == field.rowBaseValue && !field.modified);
                break;
            case 2U:
                check(m_controller.SelectE(m_lastE));
                check(Current().rowSelected && Current().eCode == m_lastE);
                break;
            case 3U:
                check(m_controller.SelectTable(m_otherTable));
                check(!Current().rowSelected && Current().tableId == m_otherTable);
                check(m_controller.SelectE(m_otherE) && Current().rowSelected);
                break;
            case 4U:
                check(m_controller.SetActual(m_stageField, m_nonStage));
                check(m_controller.SelectTable(m_table));
                check(!Current().rowSelected && Current().modifiedCount == 0U && !read());
                break;
            case 5U:
                check(m_controller.SetActual(m_stageField, m_nonStage));
                check(m_controller.SelectE(m_firstE));
                check(read() && !field.overridePresent && field.actualValue == field.rowBaseValue &&
                    field.stageId == field.rowStageId);
                break;
            case 6U:
                check(m_controller.SelectStage(m_stageField, m_stage1.id));
                check(read() && field.stageId == m_stage1.id && field.actualValue == m_stage1.value && !field.overridePresent);
                break;
            case 7U:
                check(m_controller.SelectStage(m_stageField, m_stage0.id));
                check(m_controller.StepStage(m_stageField, 1));
                check(read() && field.stageId == m_stage1.id && field.actualValue == m_stage1.value);
                check(m_controller.StepStage(m_stageField, -1));
                check(read() && field.stageId == m_stage0.id && field.actualValue == m_stage0.value);
                break;
            case 8U:
                check(m_controller.SelectStage(m_stageField, m_stage0.id));
                check(!m_controller.StepStage(m_stageField, -1) && Current().lastError == Error::StageBoundary);
                check(read() && field.stageId == m_stage0.id && field.actualValue == m_stage0.value && Current().rowSelected);
                break;
            case 9U:
                check(m_controller.SelectStage(m_stageField, m_stage0.id));
                check(m_controller.SetActual(m_stageField, m_nonStage));
                check(read() && field.actualValue == m_nonStage && field.overridePresent);
                break;
            case 10U:
                check(m_controller.SelectStage(m_stageField, m_stage1.id));
                check(m_controller.SetActual(m_stageField, m_nonStage));
                check(read() && field.actualValue == m_nonStage && field.stageId == m_stage1.id &&
                    field.baseValue == m_stage1.value && field.overridePresent);
                break;
            case 11U:
                check(m_controller.SelectStage(m_stageField, m_stage0.id));
                check(m_controller.SetActual(m_stageField, m_nonStage));
                check(m_controller.SelectStage(m_stageField, m_stage1.id));
                check(read() && field.stageId == m_stage1.id && field.actualValue == m_stage1.value && !field.overridePresent);
                break;
            case 12U:
                check(m_controller.SelectStage(m_stageField, m_stage1.id));
                check(m_controller.SetActual(m_stageField, m_nonStage));
                check(m_controller.ResetOverride(m_stageField));
                check(read() && field.actualValue == m_stage1.value && field.stageId == m_stage1.id && !field.overridePresent);
                break;
            case 13U:
                check(m_controller.ReadField(m_directField, field) && field.stageId == 0U);
                check(!m_controller.StepStage(m_directField, 1) && Current().lastError == Error::NoStageAnchor);
                check(Current().rowSelected && m_controller.ReadField(m_directField, field) &&
                    field.actualValue == field.rowBaseValue && field.stageId == 0U);
                break;
            case 14U:
                check(!m_controller.SelectTable(0U));
                check(!Current().tableSelected && !Current().rowSelected);
                check(Base());
                check(!m_controller.SelectE(0U));
                check(!Current().tableSelected && !Current().rowSelected);
                check(Base());
                check(!m_controller.SelectE(m_emptyE) && Current().lastError == Error::EmptyRow);
                check(!Current().tableSelected && !Current().rowSelected);
                break;
            case 15U:
                check(Current().tableId == m_table && Current().eCode == m_firstE && Current().modifiedCount == 0U);
                check(read() && field.actualValue == field.rowBaseValue && !field.overridePresent);
                break;
            default: check(false); break;
            }
            result.actual = Current();
            if (index != 13U) m_controller.ReadField(m_stageField, field);
            result.field = field;
            check(!result.actual.PhysicalDischargeEnabled());
            if (result.passed) ++m_nextCase; else Revoke();
            return result;
        }
    private:
        static bool FindNonStage(const EDMRecipe::FieldDefinition& field, EDMRecipe::Value& value) noexcept
        {
            EDMRecipe::Value quantum = 1;
            for (unsigned int d = field.decimals; d < 6U; ++d) quantum *= 10;
            EDMRecipe::Value candidate = field.minimum;
            for (std::size_t attempt = 0U; attempt <= field.stages.size(); ++attempt)
            {
                bool found = false;
                for (const auto& stage : field.stages) found = found || stage.value == candidate;
                if (!found) { value = candidate; return true; }
                if (candidate > (std::numeric_limits<EDMRecipe::Value>::max)() - quantum) break;
                const EDMRecipe::Value next = candidate + quantum;
                if (next > field.maximum) break;
                candidate = next;
            }
            return false;
        }
        bool Base() noexcept { return m_controller.SelectTable(m_table) && m_controller.SelectE(m_firstE); }
        EDMRecipe::Controller m_controller{};
        EDMRecipe::Stage m_stage0{}, m_stage1{};
        EDMRecipe::Value m_nonStage = 0;
        std::uint32_t m_table = 0U, m_otherTable = 0U, m_nextCase = 0U;
        std::uint16_t m_firstE = 0U, m_lastE = 0U, m_emptyE = 0U, m_otherE = 0U;
        std::uint16_t m_stageField = 0U, m_directField = 0U;
        bool m_ready = false, m_live = false;
    };
    static_assert(sizeof(Scenario) <= 16384U, "EDM16 test controller must stay within 16KiB");
}
