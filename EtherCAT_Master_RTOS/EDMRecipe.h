#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

// EDM16 SHADOW_ONLY recipes. Catalog construction/installation is BOOT ONLY.
// Controller methods are bounded, single-owner-thread, and perform no allocation,
// file access, hardware access, Motion or physical discharge control.
namespace EDMRecipe
{
    using Value = std::int64_t;
    constexpr Value Scale = 1000000LL;
    constexpr std::size_t MaximumTables = 8U;
    constexpr std::size_t RowsPerTable = 1000U;
    constexpr std::size_t MaximumFields = 256U;
    constexpr std::size_t MaximumStagesPerField = 1000U;
    constexpr std::uint32_t MaximumConditionTableId = 99999999U;

    enum class Mode : std::uint8_t { ShadowOnly };
    enum class CellKind : std::uint8_t { Unset, Direct, Stage };
    enum class Error : std::uint8_t
    {
        None, NotReady, AlreadyInstalled, InvalidCatalog, InvalidField, InvalidTable,
        InvalidRow, EmptyRow, InvalidStage, NoStageAnchor, StageBoundary,
        InvalidDirection, ValueRange, ValuePrecision, NoSelection, GenerationExhausted
    };
    inline const char* ErrorName(Error error) noexcept
    {
        switch (error)
        {
        case Error::None: return "NONE";
        case Error::NotReady: return "NOT_READY";
        case Error::AlreadyInstalled: return "ALREADY_INSTALLED";
        case Error::InvalidCatalog: return "INVALID_CATALOG";
        case Error::InvalidField: return "INVALID_FIELD";
        case Error::InvalidTable: return "INVALID_TABLE";
        case Error::InvalidRow: return "INVALID_ROW";
        case Error::EmptyRow: return "EMPTY_ROW";
        case Error::InvalidStage: return "INVALID_STAGE";
        case Error::NoStageAnchor: return "NO_STAGE_ANCHOR";
        case Error::StageBoundary: return "STAGE_BOUNDARY";
        case Error::InvalidDirection: return "INVALID_DIRECTION";
        case Error::ValueRange: return "VALUE_RANGE";
        case Error::ValuePrecision: return "VALUE_PRECISION";
        case Error::NoSelection: return "NO_SELECTION";
        case Error::GenerationExhausted: return "GENERATION_EXHAUSTED";
        }
        return "UNKNOWN";
    }

    inline bool ValueMatchesPrecision(Value value, std::uint8_t decimals) noexcept
    {
        if (decimals > 6U) return false;
        Value quantum = 1LL;
        for (unsigned int i = decimals; i < 6U; ++i) quantum *= 10LL;
        return value % quantum == 0LL;
    }

    // Exact [-]digits[.digits] with 1..6 fractional digits; no whitespace,
    // exponent, plus, locale conversion or rounding. Failure preserves output.
    inline bool ParseValue(const char* token, Value& output) noexcept
    {
        if (token == nullptr) return false;
        bool negative = false;
        if (*token == '-') { negative = true; ++token; }
        if (*token < '0' || *token > '9') return false;
        const std::uint64_t positiveLimit = static_cast<std::uint64_t>((std::numeric_limits<Value>::max)());
        const std::uint64_t limit = positiveLimit + (negative ? 1ULL : 0ULL);
        const std::uint64_t integerLimit = limit / static_cast<std::uint64_t>(Scale);
        std::uint64_t integer = 0ULL;
        while (*token >= '0' && *token <= '9')
        {
            const std::uint64_t digit = static_cast<std::uint64_t>(*token++ - '0');
            if (integer > integerLimit / 10ULL ||
                (integer == integerLimit / 10ULL && digit > integerLimit % 10ULL)) return false;
            integer = integer * 10ULL + digit;
        }
        std::uint64_t fraction = 0ULL;
        unsigned int places = 0U;
        if (*token == '.')
        {
            ++token;
            while (*token >= '0' && *token <= '9')
            {
                if (places == 6U) return false;
                fraction = fraction * 10ULL + static_cast<std::uint64_t>(*token++ - '0');
                ++places;
            }
            if (places == 0U) return false;
        }
        if (*token != '\0') return false;
        for (; places < 6U; ++places) fraction *= 10ULL;
        const std::uint64_t whole = integer * static_cast<std::uint64_t>(Scale);
        if (fraction > limit - whole) return false;
        const std::uint64_t magnitude = whole + fraction;
        if (negative && magnitude == positiveLimit + 1ULL) output = (std::numeric_limits<Value>::min)();
        else output = negative ? -static_cast<Value>(magnitude) : static_cast<Value>(magnitude);
        return true;
    }

    inline bool FormatValue(Value value, char* output, std::size_t capacity) noexcept
    {
        if (output == nullptr || capacity == 0U) return false;
        output[0] = '\0';
        const bool negative = value < 0;
        const std::uint64_t magnitude = negative
            ? static_cast<std::uint64_t>(-(value + 1LL)) + 1ULL : static_cast<std::uint64_t>(value);
        std::uint64_t integer = magnitude / static_cast<std::uint64_t>(Scale);
        const std::uint64_t fraction = magnitude % static_cast<std::uint64_t>(Scale);
        char reversed[24]{};
        std::size_t digits = 0U;
        do { reversed[digits++] = static_cast<char>('0' + integer % 10ULL); integer /= 10ULL; } while (integer != 0ULL);
        const std::size_t length = digits + 7U + (negative ? 1U : 0U);
        if (capacity <= length) return false;
        std::size_t cursor = 0U;
        if (negative) output[cursor++] = '-';
        while (digits != 0U) output[cursor++] = reversed[--digits];
        output[cursor++] = '.';
        std::uint64_t divisor = 100000ULL;
        for (unsigned int i = 0U; i < 6U; ++i)
        {
            output[cursor++] = static_cast<char>('0' + (fraction / divisor) % 10ULL);
            divisor /= 10ULL;
        }
        output[cursor] = '\0';
        return true;
    }

    struct Cell
    {
        // Value first keeps the maximum boot catalog at 16 bytes per cell.
        Value value = 0LL;
        std::uint16_t stageId = 0U;
        CellKind kind = CellKind::Unset;
        bool overridePresent = false;
        Cell() noexcept = default;
        Cell(CellKind cellKind, Value cellValue, std::uint16_t cellStageId) noexcept
            : value(cellValue), stageId(cellStageId), kind(cellKind) {}
    };
    struct Stage { std::uint16_t id = 0U; Value value = 0LL; };
    struct FieldDefinition
    {
        std::uint16_t id = 0U;
        char name[64]{}, unit[24]{};
        char semanticKey[64]{};
        bool enabled = true, unitConfirmed = false;
        std::uint8_t decimals = 0U;
        Value minimum = 0LL, maximum = 0LL;
        std::vector<Stage> stages;
    };
    struct Table
    {
        std::uint32_t id = 0U, revision = 0U;
        char name[64]{};
        std::array<bool, RowsPerTable> rowPresent{};
        std::vector<Cell> cells;
    };
    struct Catalog
    {
        std::uint32_t schemaVersion = 1U, catalogRevision = 0U;
        std::uint32_t profileId = 0U, definitionRevision = 0U;
        Mode mode = Mode::ShadowOnly;
        std::vector<FieldDefinition> fields;
        std::vector<Table> tables;
    };
    struct Validation
    {
        Error error = Error::None;
        std::size_t fieldIndex = 0U, tableIndex = 0U, rowIndex = 0U, stageIndex = 0U;
    };

    namespace Detail
    {
        inline bool Label(const char* text, std::size_t capacity) noexcept
        {
            if (capacity == 0U || text[0] == '\0') return false;
            for (std::size_t i = 0U; i < capacity; ++i)
            {
                const unsigned char c = static_cast<unsigned char>(text[i]);
                if (c == 0U) return true;
                if (c < 32U || c == 127U) return false;
            }
            return false;
        }
        inline bool Fail(Validation& result, Error error) noexcept { result.error = error; return false; }
        inline bool SemanticKey(const char* text, std::size_t capacity) noexcept
        {
            if (capacity == 0U || !((text[0] >= 'A' && text[0] <= 'Z') || (text[0] >= 'a' && text[0] <= 'z')))
                return false;
            for (std::size_t i = 1U; i < capacity; ++i)
            {
                const char c = text[i];
                if (c == '\0') return true;
                if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '.')) return false;
            }
            return false;
        }
        inline std::size_t StageIndex(const FieldDefinition& field, std::uint16_t id) noexcept
        {
            std::size_t first = 0U, last = field.stages.size();
            while (first < last)
            {
                const std::size_t middle = first + (last - first) / 2U;
                if (field.stages[middle].id < id) first = middle + 1U;
                else last = middle;
            }
            return first < field.stages.size() && field.stages[first].id == id ? first : field.stages.size();
        }
    }

    // BOOT ONLY: validation work is proportional to the entire catalog.
    inline bool ValidateCatalog(const Catalog& catalog, Validation& result) noexcept
    {
        result = Validation{};
        if ((catalog.schemaVersion != 1U && catalog.schemaVersion != 2U) || catalog.catalogRevision == 0U || catalog.mode != Mode::ShadowOnly ||
            catalog.fields.empty() || catalog.fields.size() > MaximumFields ||
            catalog.tables.empty() || catalog.tables.size() > MaximumTables)
            return Detail::Fail(result, Error::InvalidCatalog);
        if (catalog.schemaVersion == 2U && (catalog.profileId == 0U || catalog.definitionRevision == 0U))
            return Detail::Fail(result, Error::InvalidCatalog);
        for (std::size_t f = 0U; f < catalog.fields.size(); ++f)
        {
            result.fieldIndex = f;
            const FieldDefinition& field = catalog.fields[f];
            if (field.id == 0U || !Detail::Label(field.name, sizeof(field.name)) ||
                !Detail::Label(field.unit, sizeof(field.unit)) || field.decimals > 6U || field.minimum > field.maximum ||
                !ValueMatchesPrecision(field.minimum, field.decimals) || !ValueMatchesPrecision(field.maximum, field.decimals) ||
                field.stages.size() > MaximumStagesPerField)
                return Detail::Fail(result, Error::InvalidField);
            for (std::size_t prior = 0U; prior < f; ++prior)
                if (catalog.fields[prior].id == field.id) return Detail::Fail(result, Error::InvalidField);
            if (catalog.schemaVersion == 2U)
            {
                if (!Detail::SemanticKey(field.semanticKey, sizeof(field.semanticKey)) ||
                    (!field.unitConfirmed && std::strcmp(field.unit, "raw") != 0)) return Detail::Fail(result, Error::InvalidField);
                for (std::size_t prior = 0U; prior < f; ++prior)
                {
                    bool same = true;
                    for (std::size_t c = 0U; c < sizeof(field.semanticKey); ++c)
                    {
                        if (catalog.fields[prior].semanticKey[c] != field.semanticKey[c]) { same = false; break; }
                        if (field.semanticKey[c] == '\0') break;
                    }
                    if (same) return Detail::Fail(result, Error::InvalidField);
                }
            }
            for (std::size_t s = 0U; s < field.stages.size(); ++s)
            {
                result.stageIndex = s;
                const Stage& stage = field.stages[s];
                if (stage.id == 0U || (s != 0U && stage.id <= field.stages[s - 1U].id))
                    return Detail::Fail(result, Error::InvalidStage);
                if (stage.value < field.minimum || stage.value > field.maximum) return Detail::Fail(result, Error::ValueRange);
                if (!ValueMatchesPrecision(stage.value, field.decimals)) return Detail::Fail(result, Error::ValuePrecision);
            }
        }
        result.stageIndex = 0U;
        for (std::size_t t = 0U; t < catalog.tables.size(); ++t)
        {
            result.tableIndex = t;
            const Table& table = catalog.tables[t];
            if (table.id == 0U || table.revision == 0U || !Detail::Label(table.name, sizeof(table.name)) ||
                table.cells.size() != RowsPerTable * catalog.fields.size()) return Detail::Fail(result, Error::InvalidTable);
            if (catalog.schemaVersion == 2U && table.id > MaximumConditionTableId)
                return Detail::Fail(result, Error::InvalidTable);
            for (std::size_t prior = 0U; prior < t; ++prior)
                if (catalog.tables[prior].id == table.id) return Detail::Fail(result, Error::InvalidTable);
            for (std::size_t r = 0U; r < RowsPerTable; ++r)
            {
                result.rowIndex = r;
                if (!table.rowPresent[r]) return Detail::Fail(result, Error::InvalidRow);
                for (std::size_t f = 0U; f < catalog.fields.size(); ++f)
                {
                    result.fieldIndex = f;
                    const Cell& cell = table.cells[r * catalog.fields.size() + f];
                    const FieldDefinition& field = catalog.fields[f];
                    if (catalog.schemaVersion == 2U && !field.enabled && cell.kind != CellKind::Unset)
                        return Detail::Fail(result, Error::InvalidField);
                    if (cell.kind == CellKind::Unset)
                    {
                        if (cell.value != 0LL || cell.stageId != 0U || cell.overridePresent) return Detail::Fail(result, Error::InvalidRow);
                    }
                    else if (cell.kind == CellKind::Direct)
                    {
                        if (cell.stageId != 0U || cell.overridePresent) return Detail::Fail(result, Error::InvalidRow);
                        if (cell.value < field.minimum || cell.value > field.maximum) return Detail::Fail(result, Error::ValueRange);
                        if (!ValueMatchesPrecision(cell.value, field.decimals)) return Detail::Fail(result, Error::ValuePrecision);
                    }
                    else if (cell.kind == CellKind::Stage)
                    {
                        if (cell.stageId == 0U || Detail::StageIndex(field, cell.stageId) == field.stages.size() ||
                            (cell.overridePresent && catalog.schemaVersion != 2U) || (!cell.overridePresent && cell.value != 0LL))
                            return Detail::Fail(result, Error::InvalidStage);
                        if (cell.overridePresent && (cell.value < field.minimum || cell.value > field.maximum))
                            return Detail::Fail(result, Error::ValueRange);
                        if (cell.overridePresent && !ValueMatchesPrecision(cell.value, field.decimals))
                            return Detail::Fail(result, Error::ValuePrecision);
                    }
                    else return Detail::Fail(result, Error::InvalidRow);
                }
            }
        }
        result = Validation{};
        return true;
    }

    class CatalogStore
    {
    public:
        CatalogStore() noexcept = default;
        CatalogStore(const CatalogStore&) = delete;
        CatalogStore& operator=(const CatalogStore&) = delete;
        CatalogStore(CatalogStore&&) = delete;
        CatalogStore& operator=(CatalogStore&&) = delete;
        // BOOT ONLY. Sole ownership transfers; retain no mutable pointer aliases.
        bool Install(std::unique_ptr<Catalog> candidate) noexcept
        {
            if (m_attempted) { m_ready = false; m_error = Error::AlreadyInstalled; return false; }
            m_attempted = true;
            Validation diagnostic{};
            if (!candidate || !ValidateCatalog(*candidate, diagnostic))
            { m_error = candidate ? diagnostic.error : Error::InvalidCatalog; return false; }
            m_catalog = std::move(candidate);
            m_ready = true;
            m_error = Error::None;
            return true;
        }
        bool Ready() const noexcept { return m_ready; }
        const Catalog* Get() const noexcept { return m_ready ? m_catalog.get() : nullptr; }
        Error LastError() const noexcept { return m_error; }
        // OWNER THREAD ONLY after the background loader has successfully called
        // ValidateCatalog(candidate). Candidate ownership must be exclusive.
        // An empty retired slot is mandatory: no allocation, validation walk or
        // destruction is allowed here. The worker later destroys retired.
        bool ExchangeValidatedOffThreadCandidate(std::unique_ptr<Catalog>& candidate,
            std::unique_ptr<const Catalog>& retired) noexcept
        {
            if (!candidate || retired) return false;
            retired.swap(m_catalog);
            m_catalog.reset(candidate.release()); // m_catalog is empty after swap.
            m_attempted = m_ready = true; m_error = Error::None;
            return true;
        }
    private:
        std::unique_ptr<const Catalog> m_catalog;
        Error m_error = Error::NotReady;
        bool m_attempted = false, m_ready = false;
    };

    struct ParameterSnapshot
    {
        std::uint16_t fieldId = 0U;
        bool configured = false;
        CellKind rowKind = CellKind::Unset;
        std::uint16_t rowStageId = 0U;
        Value rowBaseValue = 0LL;
        std::uint16_t stageId = 0U;
        Value baseValue = 0LL, actualValue = 0LL;
        bool overridePresent = false, modified = false;
    };
    struct Summary
    {
        bool catalogReady = false, tableSelected = false, rowSelected = false;
        std::uint32_t catalogRevision = 0U, tableId = 0U, tableRevision = 0U;
        std::uint32_t profileId = 0U, definitionRevision = 0U;
        std::uint16_t eCode = 0U, fieldCount = 0U, configuredCount = 0U, modifiedCount = 0U;
        std::uint64_t generation = 0ULL;
        Error lastError = Error::NotReady;
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    struct ActiveSnapshot
    {
        Summary summary{};
        std::array<ParameterSnapshot, MaximumFields> fields{};
    };

    class Controller
    {
    public:
        Controller() noexcept = default;
        Controller(const Controller&) = delete;
        Controller& operator=(const Controller&) = delete;
        Controller(Controller&&) = delete;
        Controller& operator=(Controller&&) = delete;

        // BOOT ONLY; the stable store must outlive this controller.
        bool Bind(const CatalogStore& store) noexcept
        {
            if (m_boundAttempted)
            {
                m_boundGood = false;
                ClearActive(Error::AlreadyInstalled);
                return false;
            }
            m_boundAttempted = true;
            m_store = &store;
            m_boundGood = store.Ready();
            ClearActive(m_boundGood ? Error::None : Error::NotReady);
            return m_boundGood;
        }
        bool Ready() const noexcept { return m_boundGood && !m_exhausted && m_store != nullptr && m_store->Ready(); }
        const Catalog* GetCatalog() const noexcept { return Ready() ? m_store->Get() : nullptr; }
        Summary Current() const noexcept
        {
            if (Ready()) return m_active.summary;
            Summary result{};
            result.generation = m_active.summary.generation;
            result.lastError = m_exhausted ? Error::GenerationExhausted :
                (m_active.summary.lastError == Error::AlreadyInstalled ? Error::AlreadyInstalled : Error::NotReady);
            return result;
        }
        bool SelectTable(std::uint32_t tableId) noexcept
        {
            if (!CheckReady()) return false;
            const Catalog& catalog = *GetCatalog();
            std::size_t index = 0U;
            for (; index < catalog.tables.size() && catalog.tables[index].id != tableId; ++index) {}
            if (index == catalog.tables.size()) return SelectionFailure(Error::InvalidTable);
            if (!AdvanceGeneration()) return false;
            ClearActive(Error::None);
            m_tableIndex = index;
            m_active.summary.tableSelected = true;
            m_active.summary.tableId = catalog.tables[index].id;
            m_active.summary.tableRevision = catalog.tables[index].revision;
            return true;
        }
        bool SelectE(std::uint16_t eCode) noexcept
        {
            if (!CheckReady()) return false;
            const Catalog& catalog = *GetCatalog();
            if (!m_active.summary.tableSelected) return catalog.schemaVersion == 2U ?
                EditFailure(Error::NoSelection) : SelectionFailure(Error::NoSelection);
            if (eCode == 0U || eCode > RowsPerTable) return catalog.schemaVersion == 2U ?
                EditFailure(Error::InvalidRow) : SelectionFailure(Error::InvalidRow);
            const Table& table = catalog.tables[m_tableIndex];
            const std::size_t begin = (static_cast<std::size_t>(eCode) - 1U) * catalog.fields.size();
            std::uint16_t configured = 0U;
            for (std::size_t f = 0U; f < catalog.fields.size(); ++f)
                if (table.cells[begin + f].kind != CellKind::Unset) ++configured;
            if (configured == 0U) return catalog.schemaVersion == 2U ?
                EditFailure(Error::EmptyRow) : SelectionFailure(Error::EmptyRow);
            if (!AdvanceGeneration()) return false;
            ApplyRow(eCode, configured);
            return true;
        }
        // One generation and one committed state, including first E row.
        bool SelectTableAndE1(std::uint32_t tableId) noexcept
        {
            if (!CheckReady()) return false;
            const Catalog& catalog = *GetCatalog();
            std::size_t index = 0U;
            for (; index < catalog.tables.size() && catalog.tables[index].id != tableId; ++index) {}
            if (index == catalog.tables.size()) return EditFailure(Error::InvalidTable);
            std::uint16_t configured = 0U;
            for (std::size_t f = 0U; f < catalog.fields.size(); ++f)
                if (catalog.tables[index].cells[f].kind != CellKind::Unset) ++configured;
            if (!catalog.tables[index].rowPresent[0] || configured == 0U) return EditFailure(Error::EmptyRow);
            if (!AdvanceGeneration()) return false;
            ClearActive(Error::None);
            m_tableIndex = index;
            m_active.summary.tableSelected = true;
            m_active.summary.tableId = catalog.tables[index].id;
            m_active.summary.tableRevision = catalog.tables[index].revision;
            ApplyRow(1U, configured);
            return true;
        }
        // Called only after the same stable store exchanges a validated catalog.
        // Keep generation monotonic so old command keys cannot exhibit ABA.
        bool CanRefreshAfterCatalogExchange() const noexcept
        {
            return m_boundAttempted && m_store != nullptr && !m_exhausted &&
                m_active.summary.generation != (std::numeric_limits<std::uint64_t>::max)();
        }
        bool RefreshAfterCatalogExchange() noexcept
        {
            if (!AdvanceGeneration()) return false;
            m_boundGood = m_store != nullptr && m_store->Ready();
            ClearActive(m_boundGood ? Error::None : Error::NotReady);
            return m_boundGood;
        }
    private:
        void ApplyRow(std::uint16_t eCode, std::uint16_t configured) noexcept
        {
            const Catalog& catalog = *GetCatalog();
            const Table& table = catalog.tables[m_tableIndex];
            const std::size_t begin = (static_cast<std::size_t>(eCode) - 1U) * catalog.fields.size();
            ClearFields();
            m_active.summary.rowSelected = true;
            m_active.summary.eCode = eCode;
            m_active.summary.configuredCount = configured;
            m_active.summary.modifiedCount = 0U;
            m_active.summary.lastError = Error::None;
            for (std::size_t f = 0U; f < catalog.fields.size(); ++f)
            {
                const Cell& cell = table.cells[begin + f];
                ParameterSnapshot& value = m_active.fields[f];
                value.fieldId = catalog.fields[f].id;
                value.rowKind = cell.kind;
                if (cell.kind == CellKind::Unset) continue;
                value.configured = true;
                value.rowStageId = value.stageId = cell.stageId;
                value.rowBaseValue = cell.kind == CellKind::Direct ? cell.value :
                    catalog.fields[f].stages[Detail::StageIndex(catalog.fields[f], cell.stageId)].value;
                value.baseValue = value.actualValue = value.rowBaseValue;
                if (catalog.schemaVersion == 2U && cell.overridePresent)
                {
                    value.actualValue = cell.value; value.overridePresent = true;
                    value.modified = value.actualValue != value.baseValue;
                    if (value.modified) ++m_active.summary.modifiedCount;
                }
            }
        }
    public:
        bool SetActual(std::uint16_t fieldId, Value value) noexcept
        {
            std::size_t index = 0U;
            if (!EditableField(fieldId, index)) return false;
            const FieldDefinition& field = GetCatalog()->fields[index];
            if (value < field.minimum || value > field.maximum) return EditFailure(Error::ValueRange);
            if (!ValueMatchesPrecision(value, field.decimals)) return EditFailure(Error::ValuePrecision);
            if (!AdvanceGeneration()) return false;
            ParameterSnapshot& target = m_active.fields[index];
            const bool modified = value != target.baseValue;
            if (target.modified && !modified) --m_active.summary.modifiedCount;
            if (!target.modified && modified) ++m_active.summary.modifiedCount;
            target.actualValue = value;
            target.modified = modified;
            target.overridePresent = GetCatalog()->schemaVersion == 2U || modified;
            m_active.summary.lastError = Error::None;
            return true;
        }
        bool SetActualFromStage(std::uint16_t fieldId, std::uint16_t stageId, Value value) noexcept
        {
            std::size_t index = 0U;
            if (!EditableField(fieldId, index)) return false;
            const Catalog& catalog = *GetCatalog();
            const FieldDefinition& field = catalog.fields[index];
            if (catalog.schemaVersion != 2U) return EditFailure(Error::InvalidStage);
            const std::size_t stage = Detail::StageIndex(field, stageId);
            if (stage == field.stages.size()) return EditFailure(Error::InvalidStage);
            if (value < field.minimum || value > field.maximum) return EditFailure(Error::ValueRange);
            if (!ValueMatchesPrecision(value, field.decimals)) return EditFailure(Error::ValuePrecision);
            if (!AdvanceGeneration()) return false;
            ParameterSnapshot& target = m_active.fields[index];
            const Value base = field.stages[stage].value;
            const bool modified = value != base;
            if (target.modified && !modified) --m_active.summary.modifiedCount;
            if (!target.modified && modified) ++m_active.summary.modifiedCount;
            target.stageId = stageId; target.baseValue = base; target.actualValue = value;
            target.modified = modified; target.overridePresent = true;
            m_active.summary.lastError = Error::None;
            return true;
        }
        bool SelectStage(std::uint16_t fieldId, std::uint16_t stageId) noexcept
        {
            std::size_t index = 0U;
            if (!EditableField(fieldId, index)) return false;
            const FieldDefinition& field = GetCatalog()->fields[index];
            const std::size_t stage = Detail::StageIndex(field, stageId);
            if (stage == field.stages.size()) return EditFailure(Error::InvalidStage);
            return ApplyStage(index, stage);
        }
        bool StepStage(std::uint16_t fieldId, int direction) noexcept
        {
            std::size_t index = 0U;
            if (!EditableField(fieldId, index)) return false;
            if (direction != -1 && direction != 1) return EditFailure(Error::InvalidDirection);
            const ParameterSnapshot& target = m_active.fields[index];
            const FieldDefinition& field = GetCatalog()->fields[index];
            if (GetCatalog()->schemaVersion == 2U && (target.overridePresent || target.stageId == 0U))
            {
                std::size_t nearest = field.stages.size();
                for (std::size_t s = 0U; s < field.stages.size(); ++s)
                {
                    const Value candidate = field.stages[s].value;
                    const bool eligible = direction > 0 ? candidate > target.actualValue : candidate < target.actualValue;
                    if (!eligible) continue;
                    if (nearest == field.stages.size() || (direction > 0 ? candidate < field.stages[nearest].value :
                        candidate > field.stages[nearest].value)) nearest = s;
                }
                if (nearest == field.stages.size()) return EditFailure(Error::StageBoundary);
                return ApplyStage(index, nearest);
            }
            if (target.stageId == 0U) return EditFailure(Error::NoStageAnchor);
            const std::size_t stage = Detail::StageIndex(field, target.stageId);
            if (stage == field.stages.size()) return EditFailure(Error::InvalidStage);
            if ((direction == -1 && stage == 0U) || (direction == 1 && stage + 1U == field.stages.size()))
                return EditFailure(Error::StageBoundary);
            return ApplyStage(index, direction == -1 ? stage - 1U : stage + 1U);
        }
        bool ResetOverride(std::uint16_t fieldId) noexcept
        {
            std::size_t index = 0U;
            if (!EditableField(fieldId, index)) return false;
            if (!AdvanceGeneration()) return false;
            ParameterSnapshot& target = m_active.fields[index];
            if (target.modified) --m_active.summary.modifiedCount;
            target.actualValue = target.baseValue;
            target.overridePresent = target.modified = false;
            m_active.summary.lastError = Error::None;
            return true;
        }
        void Clear() noexcept
        {
            if (!AdvanceGeneration()) return;
            ClearActive(Ready() ? Error::None : Error::NotReady);
        }
        bool ReadField(std::uint16_t fieldId, ParameterSnapshot& result) const noexcept
        {
            result = ParameterSnapshot{};
            if (!Ready() || !m_active.summary.rowSelected) return false;
            const Catalog& catalog = *GetCatalog();
            for (std::size_t i = 0U; i < catalog.fields.size(); ++i)
                if (catalog.fields[i].id == fieldId && m_active.fields[i].configured)
                { result = m_active.fields[i]; return true; }
            return false;
        }
        void CopySnapshot(ActiveSnapshot& output) const noexcept
        {
            if (Ready()) { output = m_active; return; }
            output.summary = Current();
            for (auto& field : output.fields) field = ParameterSnapshot{};
        }

    private:
        void ClearFields() noexcept { for (auto& field : m_active.fields) field = ParameterSnapshot{}; }
        void ClearActive(Error error) noexcept
        {
            const std::uint64_t generation = m_active.summary.generation;
            m_active.summary = Summary{};
            m_active.summary.generation = generation;
            m_active.summary.lastError = error;
            m_active.summary.catalogReady = Ready();
            if (Ready())
            {
                m_active.summary.catalogRevision = GetCatalog()->catalogRevision;
                m_active.summary.profileId = GetCatalog()->profileId;
                m_active.summary.definitionRevision = GetCatalog()->definitionRevision;
                m_active.summary.fieldCount = static_cast<std::uint16_t>(GetCatalog()->fields.size());
            }
            m_tableIndex = MaximumTables;
            ClearFields();
        }
        bool AdvanceGeneration() noexcept
        {
            if (m_exhausted || m_active.summary.generation == (std::numeric_limits<std::uint64_t>::max)())
            {
                m_exhausted = true;
                ClearActive(Error::GenerationExhausted);
                return false;
            }
            ++m_active.summary.generation;
            return true;
        }
        bool CheckReady() noexcept
        {
            if (m_active.summary.generation == (std::numeric_limits<std::uint64_t>::max)())
                m_exhausted = true;
            if (Ready()) return true;
            ClearActive(m_exhausted ? Error::GenerationExhausted : Error::NotReady);
            return false;
        }
        bool SelectionFailure(Error error) noexcept
        {
            if (!AdvanceGeneration()) return false;
            ClearActive(error);
            return false;
        }
        bool EditFailure(Error error) noexcept { m_active.summary.lastError = error; return false; }
        bool EditableField(std::uint16_t fieldId, std::size_t& result) noexcept
        {
            if (!CheckReady()) return false;
            if (!m_active.summary.rowSelected) return EditFailure(Error::NoSelection);
            const Catalog& catalog = *GetCatalog();
            for (std::size_t i = 0U; i < catalog.fields.size(); ++i)
                if (catalog.fields[i].id == fieldId && m_active.fields[i].configured) { result = i; return true; }
            return EditFailure(Error::InvalidField);
        }
        bool ApplyStage(std::size_t fieldIndex, std::size_t stageIndex) noexcept
        {
            if (!AdvanceGeneration()) return false;
            const Stage& stage = GetCatalog()->fields[fieldIndex].stages[stageIndex];
            ParameterSnapshot& target = m_active.fields[fieldIndex];
            if (target.modified) --m_active.summary.modifiedCount;
            target.stageId = stage.id;
            target.baseValue = target.actualValue = stage.value;
            target.overridePresent = target.modified = false;
            m_active.summary.lastError = Error::None;
            return true;
        }
        ActiveSnapshot m_active{};
        const CatalogStore* m_store = nullptr;
        std::size_t m_tableIndex = MaximumTables;
        bool m_boundAttempted = false, m_boundGood = false, m_exhausted = false;
    };

    static_assert(sizeof(Cell) <= 16U, "EDM16 catalog cell budget");
    static_assert(sizeof(Controller) <= 16384U, "EDM16 fixed runtime controller exceeds 16KiB");
    static_assert(sizeof(Summary) <= 64U, "EDM16 summary must stay small");
}
