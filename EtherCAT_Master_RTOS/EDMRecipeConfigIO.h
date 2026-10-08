#pragma once

#include "EDMRecipe.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <new>
#include <string>
#include <vector>
#if defined(_WIN32) && !defined(UNDER_RTSS)
#include <Windows.h>
#elif !defined(UNDER_RTSS)
#include <dirent.h>
#include <cerrno>
#endif

// EDM16: strict, bounded startup-only loader. No file access belongs on an NC/RT
// cyclic thread. A whole candidate is validated before ownership is returned.
namespace EDMRecipeConfigIO
{
    constexpr std::size_t MaximumCatalogBytes = 4096U;
    constexpr std::size_t MaximumFieldsBytes = 128U * 1024U;
    constexpr std::size_t MaximumStagesBytes = 16U * 1024U * 1024U;
    constexpr std::size_t MaximumTablesBytes = 16U * 1024U;
    constexpr std::size_t MaximumTableBytes = 16U * 1024U * 1024U;
    constexpr std::size_t MaximumTableLineBytes = 16384U;

    enum class Error : std::uint8_t
    {
        None, Directory, Open, Read, Size, Lines, Characters, Format, Schema,
        Revision, Mode, Number, UnknownKey, Duplicate, Missing, Count, Reference,
        Profile, Allocation
    };
    struct Diagnostic
    {
        Error error = Error::None;
        std::size_t line = 0U;
        char file[64]{}, key[64]{};
    };
    inline const char* ErrorName(Error error) noexcept
    {
        switch (error)
        {
        case Error::None: return "NONE";
        case Error::Directory: return "DIRECTORY";
        case Error::Open: return "OPEN";
        case Error::Read: return "READ";
        case Error::Size: return "SIZE";
        case Error::Lines: return "LINES";
        case Error::Characters: return "CHARACTERS";
        case Error::Format: return "FORMAT";
        case Error::Schema: return "SCHEMA";
        case Error::Revision: return "REVISION";
        case Error::Mode: return "MODE";
        case Error::Number: return "NUMBER";
        case Error::UnknownKey: return "UNKNOWN_KEY";
        case Error::Duplicate: return "DUPLICATE";
        case Error::Missing: return "MISSING";
        case Error::Count: return "COUNT";
        case Error::Reference: return "REFERENCE";
        case Error::Profile: return "PROFILE";
        case Error::Allocation: return "ALLOCATION";
        }
        return "UNKNOWN";
    }

    namespace Detail
    {
        inline void Copy(char* target, std::size_t capacity, const char* source) noexcept
        {
            std::size_t i = 0U;
            if (capacity == 0U) return;
            for (; source[i] != '\0' && i + 1U < capacity; ++i) target[i] = source[i];
            target[i] = '\0';
        }
        inline bool Fail(Diagnostic& d, Error error, const char* file,
            std::size_t line, const char* key = "") noexcept
        {
            d.error = error; d.line = line;
            Copy(d.file, sizeof(d.file), file); Copy(d.key, sizeof(d.key), key);
            return false;
        }
        inline std::string Join(const std::string& base, const std::string& leaf)
        {
            if (!base.empty() && (base.back() == '/' || base.back() == '\\')) return base + leaf;
#ifdef _WIN32
            return base + "\\" + leaf;
#else
            return base + "/" + leaf;
#endif
        }
        inline char* Trim(char* token) noexcept
        {
            while (*token == ' ' || *token == '\t') ++token;
            std::size_t size = std::strlen(token);
            while (size > 0U && (token[size - 1U] == ' ' || token[size - 1U] == '\t')) token[--size] = '\0';
            return token;
        }
        inline bool Unsigned(const char* text, std::uint64_t maximum, std::uint64_t& output) noexcept
        {
            if (*text == '\0') return false;
            std::uint64_t value = 0ULL;
            for (; *text != '\0'; ++text)
            {
                if (*text < '0' || *text > '9') return false;
                const std::uint64_t digit = static_cast<std::uint64_t>(*text - '0');
                if (digit > maximum || value > (maximum - digit) / 10ULL) return false;
                value = value * 10ULL + digit;
            }
            output = value;
            return true;
        }
        inline bool Label(const char* text, std::size_t maximum) noexcept
        {
            const std::size_t length = std::strlen(text);
            if (length == 0U || length > maximum || *text == '=' || *text == '+' || *text == '@' || *text == '-') return false;
            const unsigned char* p = reinterpret_cast<const unsigned char*>(text);
            std::size_t i = 0U;
            while (i < length)
            {
                const unsigned char c = p[i++];
                if (c < 0x80U)
                {
                    if (c < 32U || c == 127U || c == '"' || c == ',') return false;
                    continue;
                }
                std::uint32_t code = 0U, minimum = 0U;
                unsigned extra = 0U;
                if (c >= 0xC2U && c <= 0xDFU) { code = c & 0x1FU; minimum = 0x80U; extra = 1U; }
                else if (c >= 0xE0U && c <= 0xEFU) { code = c & 0x0FU; minimum = 0x800U; extra = 2U; }
                else if (c >= 0xF0U && c <= 0xF4U) { code = c & 0x07U; minimum = 0x10000U; extra = 3U; }
                else return false;
                if (length - i < extra) return false;
                for (unsigned j = 0U; j < extra; ++j)
                {
                    if ((p[i] & 0xC0U) != 0x80U) return false;
                    code = (code << 6U) | (p[i++] & 0x3FU);
                }
                if (code < minimum || code > 0x10FFFFU || (code >= 0xD800U && code <= 0xDFFFU)) return false;
            }
            return true;
        }

        class Reader
        {
        public:
            Reader(const std::string& path, const std::string& name, std::size_t maxBytes,
                std::size_t maxLine, std::size_t maxLines, Diagnostic& diagnostic)
                : m_name(name), m_maxBytes(maxBytes), m_maxLine(maxLine), m_maxLines(maxLines),
                  m_diagnostic(diagnostic), m_buffer(maxLine + 1U, '\0'), m_input(path.c_str(), std::ios::binary)
            {
                if (!m_input.is_open()) { Reject(Error::Open); return; }
                if (m_input.peek() == 0xEF)
                {
                    const int first = Byte(), second = Byte(), third = Byte();
                    if (!m_ok) return;
                    if (first != 0xEF || second != 0xBB || third != 0xBF) Reject(Error::Characters);
                }
            }
            bool Good() const noexcept { return m_ok; }
            std::size_t Line() const noexcept { return m_line; }
            const char* Name() const noexcept { return m_name.c_str(); }
            char* Text() noexcept { return m_buffer.data(); }
            bool Next()
            {
                if (!m_ok || m_done) return false;
                ++m_line;
                std::size_t used = 0U;
                for (;;)
                {
                    const int value = Byte();
                    if (!m_ok) return false;
                    if (value == std::char_traits<char>::eof())
                    {
                        m_done = true;
                        if (used == 0U) { --m_line; return false; }
                        break;
                    }
                    if (m_line > m_maxLines) return Reject(Error::Lines);
                    if (value == '\n' || value == '\r')
                    {
                        if (value == '\r' && m_input.peek() == '\n')
                        { Byte(); if (!m_ok) return false; }
                        break;
                    }
                    if (used >= m_maxLine) return Reject(Error::Size, "LINE_BYTES");
                    if (value == 0 || (value < 32 && value != '\t') || value == 127)
                        return Reject(Error::Characters);
                    m_buffer[used++] = static_cast<char>(value);
                }
                m_buffer[used] = '\0';
                return true;
            }
            bool Reject(Error error, const char* key = "") noexcept
            {
                m_ok = false;
                return Fail(m_diagnostic, error, Name(), m_line, key);
            }
        private:
            int Byte()
            {
                const int value = m_input.get();
                if (value == std::char_traits<char>::eof())
                {
                    if (m_input.bad() || (m_input.fail() && !m_input.eof())) Reject(Error::Read);
                    return value;
                }
                ++m_bytes;
                if (m_bytes > m_maxBytes) Reject(Error::Size, "FILE_BYTES");
                return value;
            }
            std::string m_name;
            std::size_t m_maxBytes = 0U, m_maxLine = 0U, m_maxLines = 0U;
            Diagnostic& m_diagnostic;
            std::vector<char> m_buffer;
            std::ifstream m_input;
            std::size_t m_bytes = 0U, m_line = 0U;
            bool m_ok = true, m_done = false;
        };

        struct Columns
        {
            std::array<char*, EDMRecipe::MaximumFields + 1U> values{};
            std::size_t count = 0U;
        };
        inline bool Split(Reader& reader, Columns& columns)
        {
            columns.count = 0U;
            char* start = reader.Text();
            if (*Trim(start) == '\0') return reader.Reject(Error::Format, "EMPTY_ROW");
            for (char* cursor = start;; ++cursor)
            {
                if (*cursor == '"') return reader.Reject(Error::Format, "QUOTED_CSV");
                if (*cursor == ',' || *cursor == '\0')
                {
                    const bool end = *cursor == '\0';
                    *cursor = '\0';
                    if (columns.count >= columns.values.size()) return reader.Reject(Error::Count, "COLUMNS");
                    columns.values[columns.count++] = Trim(start);
                    if (end) return true;
                    start = cursor + 1;
                }
            }
        }
        inline bool Row(Reader& reader, Columns& columns)
        {
            if (!reader.Next()) return reader.Good() ? reader.Reject(Error::Missing, "ROW") : false;
            return Split(reader, columns);
        }
        inline bool ExactHeader(Reader& reader, Columns& columns, const char* const* names, std::size_t count)
        {
            if (!Row(reader, columns)) return false;
            if (columns.count != count) return reader.Reject(Error::Count, "HEADER_COLUMNS");
            for (std::size_t i = 0U; i < count; ++i)
                if (std::strcmp(columns.values[i], names[i]) != 0) return reader.Reject(Error::Format, "HEADER");
            return true;
        }
        inline bool Number(Reader& reader, const char* token, std::uint64_t maximum,
            std::uint64_t& value, const char* key, bool positive = false)
        {
            if (!Unsigned(token, maximum, value) || (positive && value == 0ULL)) return reader.Reject(Error::Number, key);
            return true;
        }
        inline bool Metadata(Reader& reader, Columns& columns, std::uint32_t revision,
            const EDMRecipe::Table* table = nullptr)
        {
            const char* const names[] = { "SchemaVersion", "CatalogRevision", "TableId", "TableRevision" };
            const std::size_t count = table ? 4U : 2U;
            // Spreadsheet export may pad only these two metadata rows to the
            // sheet width. Keep all body-row widths exact, including unset cells.
            for (unsigned metadataRow = 0U; metadataRow < 2U; ++metadataRow)
            {
                if (!Row(reader, columns)) return false;
                if (columns.count < count) return reader.Reject(Error::Count, "METADATA_COLUMNS");
                // Split already bounds the width to MaximumFields + 1.
                for (std::size_t i = count; i < columns.count; ++i)
                    if (*columns.values[i] != '\0') return reader.Reject(Error::Format, "METADATA_TRAILING");
                if (metadataRow == 0U)
                    for (std::size_t i = 0U; i < count; ++i)
                        if (std::strcmp(columns.values[i], names[i]) != 0)
                            return reader.Reject(Error::Format, "HEADER");
            }
            std::uint64_t value = 0ULL;
            if (!Number(reader, columns.values[0], UINT32_MAX, value, names[0])) return false;
            if (value != 1ULL) return reader.Reject(Error::Schema, names[0]);
            if (!Number(reader, columns.values[1], UINT32_MAX, value, names[1])) return false;
            if (value != revision) return reader.Reject(Error::Revision, names[1]);
            if (table)
            {
                if (!Number(reader, columns.values[2], UINT32_MAX, value, names[2])) return false;
                if (value != table->id) return reader.Reject(Error::Reference, names[2]);
                if (!Number(reader, columns.values[3], UINT32_MAX, value, names[3])) return false;
                if (value != table->revision) return reader.Reject(Error::Revision, names[3]);
            }
            return true;
        }
        inline bool End(Reader& reader)
        {
            if (reader.Next()) return reader.Reject(Error::Count, "EXTRA_ROW");
            return reader.Good();
        }
        inline std::size_t FieldIndex(const EDMRecipe::Catalog& c, std::uint16_t id) noexcept
        {
            for (std::size_t i = 0U; i < c.fields.size(); ++i) if (c.fields[i].id == id) return i;
            return c.fields.size();
        }
        inline bool Value(Reader& reader, const char* text, EDMRecipe::Value& value, const char* key)
        {
            if (!EDMRecipe::ParseValue(text, value)) return reader.Reject(Error::Number, key);
            return true;
        }
        inline bool FieldValue(Reader& reader, const char* text, const EDMRecipe::FieldDefinition& field,
            EDMRecipe::Value& value, const char* key)
        {
            if (!Value(reader, text, value, key)) return false;
            if (value < field.minimum || value > field.maximum || !EDMRecipe::ValueMatchesPrecision(value, field.decimals))
                return reader.Reject(Error::Profile, key);
            return true;
        }
        struct Counts { std::size_t fields = 0U, tables = 0U, stages = 0U; };

        inline bool ReadCatalog(const std::string& directory, EDMRecipe::Catalog& catalog, Counts& counts, Diagnostic& d)
        {
            Reader reader(Join(directory, "Catalog.ini"), "Catalog.ini", MaximumCatalogBytes, 256U, 128U, d);
            if (!reader.Good()) return false;
            const char* const keys[] = { "SchemaVersion", "CatalogRevision", "Mode", "FieldCount", "TableCount", "StageCount", "MachineProfileId", "DefinitionRevision" };
            unsigned seen = 0U;
            while (reader.Next())
            {
                char* line = reader.Text();
                for (std::size_t i = 0U; line[i] != '\0'; ++i)
                {
                    if (line[i] == '#' || line[i] == ';' || (line[i] == '/' && line[i + 1U] == '/'))
                    { line[i] = '\0'; break; }
                    if (static_cast<unsigned char>(line[i]) >= 127U) return reader.Reject(Error::Characters);
                }
                char* key = Trim(line);
                if (*key == '\0') continue;
                char* equal = std::strchr(key, '=');
                if (equal == nullptr) return reader.Reject(Error::Format);
                *equal = '\0'; key = Trim(key);
                char* value = Trim(equal + 1);
                if (*key == '\0' || *value == '\0' || std::strchr(value, '=') != nullptr) return reader.Reject(Error::Format, key);
                unsigned index = 0U;
                for (; index < 8U && std::strcmp(key, keys[index]) != 0; ++index) {}
                if (index == 8U) return reader.Reject(Error::UnknownKey, key);
                if ((seen & (1U << index)) != 0U) return reader.Reject(Error::Duplicate, key);
                seen |= 1U << index;
                if (index == 2U)
                {
                    if (std::strcmp(value, "SHADOW_ONLY") != 0) return reader.Reject(Error::Mode, key);
                    catalog.mode = EDMRecipe::Mode::ShadowOnly;
                    continue;
                }
                std::uint64_t numeric = 0ULL;
                if (!Number(reader, value, UINT32_MAX, numeric, key)) return false;
                if (index == 0U)
                { if (numeric != 1ULL && numeric != 2ULL) return reader.Reject(Error::Schema, key); catalog.schemaVersion = static_cast<std::uint32_t>(numeric); }
                else if (index == 1U)
                { if (numeric == 0ULL) return reader.Reject(Error::Revision, key); catalog.catalogRevision = static_cast<std::uint32_t>(numeric); }
                else if (index == 3U)
                { if (numeric == 0ULL || numeric > EDMRecipe::MaximumFields) return reader.Reject(Error::Count, key); counts.fields = static_cast<std::size_t>(numeric); }
                else if (index == 4U)
                { if (numeric == 0ULL || numeric > EDMRecipe::MaximumTables) return reader.Reject(Error::Count, key); counts.tables = static_cast<std::size_t>(numeric); }
                else if (index == 5U)
                { if (numeric > EDMRecipe::MaximumFields * EDMRecipe::MaximumStagesPerField) return reader.Reject(Error::Count, key); counts.stages = static_cast<std::size_t>(numeric); }
                else if (index == 6U)
                { if (numeric == 0ULL) return reader.Reject(Error::Profile, key); catalog.profileId = static_cast<std::uint32_t>(numeric); }
                else
                { if (numeric == 0ULL) return reader.Reject(Error::Revision, key); catalog.definitionRevision = static_cast<std::uint32_t>(numeric); }
            }
            if (!reader.Good()) return false;
            const unsigned required = catalog.schemaVersion == 1U ? 0x3FU : 0xEFU;
            for (unsigned i = 0U; i < 8U; ++i)
            {
                if ((required & (1U << i)) != 0U && (seen & (1U << i)) == 0U) return reader.Reject(Error::Missing, keys[i]);
                if ((required & (1U << i)) == 0U && (seen & (1U << i)) != 0U) return reader.Reject(Error::UnknownKey, keys[i]);
            }
            if (counts.stages > counts.fields * EDMRecipe::MaximumStagesPerField) return reader.Reject(Error::Count, "StageCount");
            return true;
        }

        inline bool DefinitionMetadata(Reader& reader, Columns& columns, const EDMRecipe::Catalog& catalog)
        {
            if (catalog.schemaVersion == 1U) return Metadata(reader, columns, catalog.catalogRevision);
            const char* const names[] = { "SchemaVersion", "CatalogRevision", "MachineProfileId", "DefinitionRevision" };
            if (!ExactHeader(reader, columns, names, 4U) || !Row(reader, columns)) return false;
            if (columns.count != 4U) return reader.Reject(Error::Count, "METADATA_COLUMNS");
            const std::uint32_t expected[] = { 2U, catalog.catalogRevision, catalog.profileId, catalog.definitionRevision };
            for (std::size_t i = 0U; i < 4U; ++i) {
                std::uint64_t value = 0ULL;
                if (!Number(reader, columns.values[i], UINT32_MAX, value, names[i], true)) return false;
                if (value != expected[i]) return reader.Reject(Error::Revision, names[i]);
            }
            return true;
        }
        inline bool SemanticKey(const char* text) noexcept
        {
            const std::size_t size = std::strlen(text);
            if (size == 0U || size > 63U) return false;
            for (std::size_t i = 0U; i < size; ++i) {
                const char c = text[i]; const bool letter = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
                if (!letter && (i == 0U || !((c >= '0' && c <= '9') || c == '_' || c == '.'))) return false;
            }
            return true;
        }

        inline bool ReadFields(const std::string& directory, EDMRecipe::Catalog& catalog, const Counts& counts, Diagnostic& d)
        {
            Reader reader(Join(directory, "Fields.csv"), "Fields.csv", MaximumFieldsBytes, 1024U, 259U, d);
            Columns columns;
            const char* const names1[] = { "FieldId", "Name", "Unit", "Decimals", "Min", "Max" };
            const char* const names2[] = { "FieldId", "SemanticKey", "Name", "Unit", "Decimals", "Min", "Max", "Enabled", "UnitConfirmed" };
            const bool v2 = catalog.schemaVersion == 2U;
            if (!reader.Good() || !DefinitionMetadata(reader, columns, catalog) ||
                !ExactHeader(reader, columns, v2 ? names2 : names1, v2 ? 9U : 6U)) return false;
            catalog.fields.reserve(counts.fields);
            for (std::size_t row = 0U; row < counts.fields; ++row)
            {
                if (!Row(reader, columns)) return false;
                if (columns.count != (v2 ? 9U : 6U)) return reader.Reject(Error::Count, "COLUMNS");
                std::uint64_t value = 0ULL; EDMRecipe::FieldDefinition field;
                if (!Number(reader, columns.values[0], UINT16_MAX, value, "FieldId", true)) return false;
                field.id = static_cast<std::uint16_t>(value);
                if (FieldIndex(catalog, field.id) != catalog.fields.size()) return reader.Reject(Error::Duplicate, "FieldId");
                const std::size_t n = v2 ? 2U : 1U;
                if (!Label(columns.values[n], sizeof(field.name) - 1U) || !Label(columns.values[n + 1U], sizeof(field.unit) - 1U))
                    return reader.Reject(Error::Characters, "Name/Unit");
                Copy(field.name, sizeof(field.name), columns.values[n]); Copy(field.unit, sizeof(field.unit), columns.values[n + 1U]);
                if (!Number(reader, columns.values[n + 2U], 6ULL, value, "Decimals")) return false;
                field.decimals = static_cast<std::uint8_t>(value);
                if (!Value(reader, columns.values[n + 3U], field.minimum, "Min") || !Value(reader, columns.values[n + 4U], field.maximum, "Max")) return false;
                if (field.minimum > field.maximum || !EDMRecipe::ValueMatchesPrecision(field.minimum, field.decimals) ||
                    !EDMRecipe::ValueMatchesPrecision(field.maximum, field.decimals)) return reader.Reject(Error::Profile, "Min/Max");
                if (v2) {
                    if (!SemanticKey(columns.values[1])) return reader.Reject(Error::Characters, "SemanticKey");
                    for (const auto& prior : catalog.fields) if (std::strcmp(prior.semanticKey, columns.values[1]) == 0) return reader.Reject(Error::Duplicate, "SemanticKey");
                    Copy(field.semanticKey, sizeof(field.semanticKey), columns.values[1]);
                    if (!Number(reader, columns.values[7], 1ULL, value, "Enabled")) return false;
                    field.enabled = value == 1ULL;
                    if (!Number(reader, columns.values[8], 1ULL, value, "UnitConfirmed")) return false;
                    field.unitConfirmed = value == 1ULL;
                    if (!field.unitConfirmed && std::strcmp(field.unit, "raw") != 0) return reader.Reject(Error::Profile, "Unit");
                }
                catalog.fields.push_back(std::move(field));
            }
            return End(reader);
        }

        inline bool ReadStages(const std::string& directory, EDMRecipe::Catalog& catalog, const Counts& counts, Diagnostic& d)
        {
            Reader reader(Join(directory, "Stages.csv"), "Stages.csv", MaximumStagesBytes, 128U, 256003U, d);
            Columns columns;
            const char* const names[] = { "FieldId", "StageId", "Value" };
            if (!reader.Good() || !DefinitionMetadata(reader, columns, catalog) || !ExactHeader(reader, columns, names, 3U)) return false;
            for (std::size_t row = 0U; row < counts.stages; ++row)
            {
                if (!Row(reader, columns)) return false;
                if (columns.count != 3U) return reader.Reject(Error::Count, "COLUMNS");
                std::uint64_t value = 0ULL;
                if (!Number(reader, columns.values[0], UINT16_MAX, value, "FieldId", true)) return false;
                const std::size_t index = FieldIndex(catalog, static_cast<std::uint16_t>(value));
                if (index == catalog.fields.size()) return reader.Reject(Error::Reference, "FieldId");
                EDMRecipe::FieldDefinition& field = catalog.fields[index];
                if (field.stages.size() >= EDMRecipe::MaximumStagesPerField) return reader.Reject(Error::Count, "StagesPerField");
                EDMRecipe::Stage stage;
                if (!Number(reader, columns.values[1], UINT16_MAX, value, "StageId", true)) return false;
                stage.id = static_cast<std::uint16_t>(value);
                for (const auto& existing : field.stages) if (existing.id == stage.id) return reader.Reject(Error::Duplicate, "StageId");
                if (!FieldValue(reader, columns.values[2], field, stage.value, "Value")) return false;
                field.stages.push_back(stage);
            }
            if (!End(reader)) return false;
            for (auto& field : catalog.fields)
                std::sort(field.stages.begin(), field.stages.end(), [](const EDMRecipe::Stage& a, const EDMRecipe::Stage& b) { return a.id < b.id; });
            return true;
        }

        inline bool ReadTables(const std::string& directory, EDMRecipe::Catalog& catalog, const Counts& counts, Diagnostic& d)
        {
            Reader reader(Join(directory, "Tables.csv"), "Tables.csv", MaximumTablesBytes, 256U, 11U, d);
            Columns columns;
            const char* const names[] = { "TableId", "TableRevision", "Name" };
            if (!reader.Good() || !Metadata(reader, columns, catalog.catalogRevision) || !ExactHeader(reader, columns, names, 3U)) return false;
            catalog.tables.reserve(counts.tables);
            for (std::size_t row = 0U; row < counts.tables; ++row)
            {
                if (!Row(reader, columns)) return false;
                if (columns.count != 3U) return reader.Reject(Error::Count, "COLUMNS");
                EDMRecipe::Table table;
                std::uint64_t value = 0ULL;
                if (!Number(reader, columns.values[0], UINT32_MAX, value, "TableId", true)) return false;
                table.id = static_cast<std::uint32_t>(value);
                for (const auto& existing : catalog.tables) if (existing.id == table.id) return reader.Reject(Error::Duplicate, "TableId");
                if (!Number(reader, columns.values[1], UINT32_MAX, value, "TableRevision", true)) return false;
                table.revision = static_cast<std::uint32_t>(value);
                if (!Label(columns.values[2], sizeof(table.name) - 1U)) return reader.Reject(Error::Characters, "Name");
                Copy(table.name, sizeof(table.name), columns.values[2]);
                table.cells.resize(EDMRecipe::RowsPerTable * counts.fields);
                catalog.tables.push_back(std::move(table));
            }
            return End(reader);
        }

        inline bool ReadTable(const std::string& directory, const EDMRecipe::Catalog& catalog,
            EDMRecipe::Table& table, Diagnostic& d)
        {
            const std::string leaf = "Table_" + std::to_string(table.id) + ".csv";
            const std::string name = "Tables/" + leaf;
            Reader reader(Join(Join(directory, "Tables"), leaf), name, MaximumTableBytes, MaximumTableLineBytes, 1003U, d);
            Columns columns;
            if (!reader.Good() || !Metadata(reader, columns, catalog.catalogRevision, &table) || !Row(reader, columns)) return false;
            const std::size_t fields = catalog.fields.size();
            if (columns.count != fields + 1U || std::strcmp(columns.values[0], "ECode") != 0) return reader.Reject(Error::Format, "HEADER");
            std::array<std::size_t, EDMRecipe::MaximumFields> mapping{};
            std::array<bool, EDMRecipe::MaximumFields> seen{};
            for (std::size_t column = 0U; column < fields; ++column)
            {
                const char* token = columns.values[column + 1U];
                std::uint64_t id = 0ULL;
                if (*token != 'F' || !Number(reader, token + 1, UINT16_MAX, id, "FieldColumn", true)) return reader.Reject(Error::Format, "FieldColumn");
                const std::size_t index = FieldIndex(catalog, static_cast<std::uint16_t>(id));
                if (index == fields) return reader.Reject(Error::Reference, "FieldColumn");
                if (seen[index]) return reader.Reject(Error::Duplicate, "FieldColumn");
                seen[index] = true; mapping[column] = index;
            }
            for (std::size_t row = 0U; row < EDMRecipe::RowsPerTable; ++row)
            {
                if (!Row(reader, columns)) return false;
                if (columns.count != fields + 1U) return reader.Reject(Error::Count, "COLUMNS");
                std::uint64_t eCode = 0ULL;
                if (!Number(reader, columns.values[0], EDMRecipe::RowsPerTable, eCode, "ECode", true)) return false;
                const std::size_t rowIndex = static_cast<std::size_t>(eCode - 1ULL);
                if (table.rowPresent[rowIndex]) return reader.Reject(Error::Duplicate, "ECode");
                table.rowPresent[rowIndex] = true;
                for (std::size_t column = 0U; column < fields; ++column)
                {
                    const std::size_t index = mapping[column];
                    const EDMRecipe::FieldDefinition& field = catalog.fields[index];
                    const char* token = columns.values[column + 1U];
                    EDMRecipe::Cell& cell = table.cells[rowIndex * fields + index];
                    if (*token == '\0') continue;
                    if (token[0] == 'V' && token[1] == ':')
                    {
                        cell.kind = EDMRecipe::CellKind::Direct;
                        if (!FieldValue(reader, token + 2, field, cell.value, "CellValue")) return false;
                    }
                    else if (token[0] == 'S' && token[1] == ':')
                    {
                        std::uint64_t stage = 0ULL;
                        if (!Number(reader, token + 2, UINT16_MAX, stage, "CellStage", true)) return false;
                        bool found = false;
                        for (const auto& candidate : field.stages) if (candidate.id == stage) { found = true; break; }
                        if (!found) return reader.Reject(Error::Reference, "CellStage");
                        cell.kind = EDMRecipe::CellKind::Stage; cell.stageId = static_cast<std::uint16_t>(stage);
                    }
                    else return reader.Reject(Error::Format, "CellMode");
                }
            }
            return End(reader);
        }
        inline bool ReadCondition(const std::string& directory, const std::string& leaf, const EDMRecipe::Catalog& catalog,
            std::uint32_t expectedId, EDMRecipe::Table& table, bool& matches, Diagnostic& d)
        {
            const std::string name = "COND/" + leaf;
            Reader reader(Join(Join(directory, "COND"), leaf), name, MaximumTableBytes, MaximumTableLineBytes, 1003U, d);
            Columns columns; matches = false;
            const char* const names[] = { "SchemaVersion", "CatalogRevision", "MachineProfileId", "DefinitionRevision", "TableId", "TableRevision", "Name" };
            if (!reader.Good() || !ExactHeader(reader, columns, names, 7U) || !Row(reader, columns)) return false;
            if (columns.count != 7U) return reader.Reject(Error::Count, "METADATA_COLUMNS");
            std::uint64_t values[6]{};
            for (std::size_t i = 0U; i < 6U; ++i)
                if (!Number(reader, columns.values[i], i == 4U ? 99999999ULL : UINT32_MAX, values[i], names[i], true)) return false;
            if (values[0] != 2ULL) return reader.Reject(Error::Schema, names[0]);
            if (values[4] != expectedId) return reader.Reject(Error::Reference, "FILENAME_ID");
            if (values[2] != catalog.profileId) return true;
            if (values[1] != catalog.catalogRevision || values[3] != catalog.definitionRevision) return reader.Reject(Error::Revision, "DEFINITION");
            table.id = static_cast<std::uint32_t>(values[4]); table.revision = static_cast<std::uint32_t>(values[5]);
            if (!Label(columns.values[6], sizeof(table.name) - 1U)) return reader.Reject(Error::Characters, "Name");
            Copy(table.name, sizeof(table.name), columns.values[6]);
            const std::size_t fields = catalog.fields.size(); table.cells.resize(EDMRecipe::RowsPerTable * fields);
            if (!Row(reader, columns)) return false;
            if (columns.count != fields + 1U || std::strcmp(columns.values[0], "ECode") != 0) return reader.Reject(Error::Format, "HEADER");
            std::array<std::size_t, EDMRecipe::MaximumFields> mapping{};
            std::array<bool, EDMRecipe::MaximumFields> seen{};
            for (std::size_t column = 0U; column < fields; ++column) {
                const char* token = columns.values[column + 1U]; std::uint64_t id = 0ULL;
                if (*token != 'F' || !Number(reader, token + 1, UINT16_MAX, id, "FieldColumn", true)) return reader.Reject(Error::Format, "FieldColumn");
                const std::size_t index = FieldIndex(catalog, static_cast<std::uint16_t>(id));
                if (index == fields || seen[index]) return reader.Reject(Error::Reference, "FieldColumn");
                seen[index] = true; mapping[column] = index;
            }
            for (std::size_t row = 0U; row < EDMRecipe::RowsPerTable; ++row) {
                if (!Row(reader, columns)) return false;
                if (columns.count != fields + 1U) return reader.Reject(Error::Count, "COLUMNS");
                std::uint64_t e = 0ULL; if (!Number(reader, columns.values[0], EDMRecipe::RowsPerTable, e, "ECode", true)) return false;
                const std::size_t r = static_cast<std::size_t>(e - 1ULL);
                if (table.rowPresent[r]) return reader.Reject(Error::Duplicate, "ECode");
                table.rowPresent[r] = true;
                for (std::size_t column = 0U; column < fields; ++column) {
                    const std::size_t index = mapping[column]; const auto& field = catalog.fields[index];
                    char* token = columns.values[column + 1U]; auto& cell = table.cells[r * fields + index];
                    if (*token == '\0') continue;
                    if (!field.enabled) return reader.Reject(Error::Profile, "DISABLED_FIELD");
                    if (token[0] == 'V' && token[1] == ':') {
                        cell.kind = EDMRecipe::CellKind::Direct;
                        if (!FieldValue(reader, token + 2, field, cell.value, "CellValue")) return false;
                    } else if (token[0] == 'S' && token[1] == ':') {
                        char* custom = std::strchr(token + 2, ':');
                        if (custom != nullptr) {
                            if (std::strncmp(custom, ":V:", 3U) != 0) return reader.Reject(Error::Format, "CellOverride");
                            *custom = '\0'; cell.overridePresent = true;
                            if (!FieldValue(reader, custom + 3, field, cell.value, "CellOverride")) return false;
                        }
                        std::uint64_t id = 0ULL; if (!Number(reader, token + 2, UINT16_MAX, id, "CellStage", true)) return false;
                        bool found = false; for (const auto& stage : field.stages) if (stage.id == id) { found = true; break; }
                        if (!found) return reader.Reject(Error::Reference, "CellStage");
                        cell.kind = EDMRecipe::CellKind::Stage; cell.stageId = static_cast<std::uint16_t>(id);
                    } else return reader.Reject(Error::Format, "CellMode");
                }
            }
            matches = true; return End(reader);
        }
        inline bool ConditionFilename(const std::string& name, std::uint32_t& id)
        {
            if (name.size() != 16U || name.compare(0U, 4U, "COND") != 0 || name.compare(12U, 4U, ".csv") != 0) return false;
            std::uint64_t number = 0ULL;
            if (!Unsigned(name.substr(4U, 8U).c_str(), 99999999ULL, number) || number == 0ULL) return false;
            id = static_cast<std::uint32_t>(number); return true;
        }
        inline bool CsvSuffix(const std::string& name) noexcept
        {
            if (name.size() < 4U) return false;
            const std::size_t n = name.size();
            return name[n - 4U] == '.' && (name[n - 3U] == 'c' || name[n - 3U] == 'C') &&
                (name[n - 2U] == 's' || name[n - 2U] == 'S') && (name[n - 1U] == 'v' || name[n - 1U] == 'V');
        }
        // RTX64 does not support FindFirstFile/FindNextFile/FindClose. The
        // Windows SDK publishes this bounded, atomic index before core reload.
        // RTSS reads ordinary files only; no directory enumeration enters RTSS.
        inline bool ReadConditionIndex(const std::string& directory, const EDMRecipe::Catalog& catalog,
            std::vector<std::string>& names, Diagnostic& d)
        {
            Reader reader(Join(directory, "ConditionIndex.csv"), "ConditionIndex.csv",
                16384U, 128U, 1027U, d);
            Columns columns;
            if (!reader.Good() || !DefinitionMetadata(reader, columns, catalog)) return false;
            const char* const header[] = { "TableId" };
            if (!ExactHeader(reader, columns, header, 1U)) return false;
            std::uint64_t previous = 0ULL;
            while (reader.Next())
            {
                if (!Split(reader, columns)) return false;
                if (columns.count != 1U) return reader.Reject(Error::Count, "TABLE_ID_COLUMNS");
                std::uint64_t id = 0ULL;
                if (!Number(reader, columns.values[0], 99999999ULL, id, "TableId", true)) return false;
                if (id <= previous) return reader.Reject(id == previous ? Error::Duplicate : Error::Format, "TABLE_ID_ORDER");
                if (names.size() >= 1024U) return reader.Reject(Error::Count, "INDEX_TABLES");
                std::string name("COND00000000.csv");
                std::uint64_t digits = id;
                for (std::size_t position = 12U; position > 4U; )
                { --position; name[position] = static_cast<char>('0' + digits % 10ULL); digits /= 10ULL; }
                names.push_back(name);
                previous = id;
            }
            if (!reader.Good()) return false;
            return names.empty() ? reader.Reject(Error::Missing, "TABLE_IDS") : true;
        }
        inline bool ReadConditions(const std::string& directory, EDMRecipe::Catalog& catalog, Diagnostic& d)
        {
            std::vector<std::string> names;
#ifdef UNDER_RTSS
            if (!ReadConditionIndex(directory, catalog, names, d)) return false;
#else
            const std::string folder = Join(directory, "COND");
            bool okay = true;
#ifdef _WIN32
            WIN32_FIND_DATAA data{}; HANDLE handle = FindFirstFileA(Join(folder, "*").c_str(), &data);
            if (handle == INVALID_HANDLE_VALUE) return Fail(d, Error::Directory, "COND", 0U);
            do {
                const std::string name(data.cFileName);
                if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0U && CsvSuffix(name)) names.push_back(name);
                if (names.size() > 1024U) { okay = false; break; }
            } while (FindNextFileA(handle, &data));
            const DWORD last = GetLastError(); FindClose(handle);
            if (okay && last != ERROR_NO_MORE_FILES) okay = false;
#else
            DIR* handle = opendir(folder.c_str()); if (handle == nullptr) return Fail(d, Error::Directory, "COND", 0U);
            errno = 0; struct dirent* entry = nullptr;
            while ((entry = readdir(handle)) != nullptr) {
                const std::string name(entry->d_name); if (CsvSuffix(name)) names.push_back(name);
                if (names.size() > 1024U) { okay = false; break; }
            }
            if (errno != 0) okay = false;
            closedir(handle);
#endif
            if (!okay) return Fail(d, Error::Directory, "COND", 0U, "ENUMERATION");
#endif
            std::sort(names.begin(), names.end());
            for (const auto& name : names) {
                std::uint32_t id = 0U; if (!ConditionFilename(name, id)) return Fail(d, Error::Format, name.c_str(), 0U, "FILENAME");
                EDMRecipe::Table table; bool matches = false;
                if (!ReadCondition(directory, name, catalog, id, table, matches, d)) return false;
                if (!matches) continue;
                if (catalog.tables.size() >= EDMRecipe::MaximumTables) return Fail(d, Error::Count, "COND", 0U, "RESIDENT_TABLES");
                catalog.tables.push_back(std::move(table));
            }
            return true;
        }
    }

    inline bool LoadDirectory(const char* directory, std::unique_ptr<EDMRecipe::Catalog>& result,
        Diagnostic& diagnostic) noexcept
    {
        diagnostic = Diagnostic{};
        if (directory == nullptr || *directory == '\0') return Detail::Fail(diagnostic, Error::Directory, "", 0U);
        try
        {
            std::unique_ptr<EDMRecipe::Catalog> candidate(new EDMRecipe::Catalog());
            Detail::Counts counts;
            const std::string base(directory);
            if (!Detail::ReadCatalog(base, *candidate, counts, diagnostic) ||
                !Detail::ReadFields(base, *candidate, counts, diagnostic) ||
                !Detail::ReadStages(base, *candidate, counts, diagnostic)) return false;
            if (candidate->schemaVersion == 1U) {
                if (!Detail::ReadTables(base, *candidate, counts, diagnostic)) return false;
                for (auto& table : candidate->tables)
                    if (!Detail::ReadTable(base, *candidate, table, diagnostic)) return false;
            } else if (!Detail::ReadConditions(base, *candidate, diagnostic)) return false;
            EDMRecipe::Validation validation;
            if (!EDMRecipe::ValidateCatalog(*candidate, validation))
                return Detail::Fail(diagnostic, Error::Profile, "CATALOG", 0U, EDMRecipe::ErrorName(validation.error));
            result = std::move(candidate);
            diagnostic = Diagnostic{};
            return true;
        }
        catch (const std::bad_alloc&) { return Detail::Fail(diagnostic, Error::Allocation, "CATALOG", 0U); }
        catch (...) { return Detail::Fail(diagnostic, Error::Read, "CATALOG", 0U); }
    }
}
