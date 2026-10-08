#pragma once
#include "EDMRecipeConfigIO.h"
#include "EDMConditionDefaults.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <locale>
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#endif

// Startup / priority-20 worker only. No directory enumeration, rename, or
// FlushFileBuffers: those APIs are not documented in RTX64's supported matrix.
namespace EDMConditionStartupIO
{
    using Diagnostic = EDMRecipeConfigIO::Diagnostic;
    using Error = EDMRecipeConfigIO::Error;
    struct Identity {
        std::uint32_t schema = 0U, profile = 0U, definition = 0U, catalog = 0U;
    };
    inline Identity GetIdentity(const EDMRecipe::Catalog& c) noexcept
    { Identity i; i.schema = c.schemaVersion; i.profile = c.profileId; i.definition = c.definitionRevision; i.catalog = c.catalogRevision; return i; }
    inline bool SameIdentity(const Identity& a, const Identity& b) noexcept
    { return a.schema == b.schema && a.profile == b.profile && a.definition == b.definition && a.catalog == b.catalog; }
    inline bool CompatibleProfile(const Identity& a, const Identity& b) noexcept
    { return a.schema == b.schema && a.profile == b.profile; }
    struct StartupResult {
        std::uint32_t tableId = 0U;
        bool generated = false, recovered = false, emergencyDisplay = false, allowPersistence = true;
        bool saveFailed = false;
        Diagnostic diagnostic{};
    };
    namespace Detail {
        using EDMRecipeConfigIO::Detail::Join;
        using EDMRecipeConfigIO::Detail::Fail;
        enum class Presence { Missing, Present, Inaccessible };
        inline Presence Probe(const std::string& path) noexcept {
#ifdef _WIN32
            HANDLE h = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
            if (h != INVALID_HANDLE_VALUE) { CloseHandle(h); return Presence::Present; }
            const DWORD error = GetLastError();
            return error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND ? Presence::Missing : Presence::Inaccessible;
#else
            struct stat info{};
            if (::stat(path.c_str(), &info) == 0) return S_ISREG(info.st_mode) ? Presence::Present : Presence::Inaccessible;
            return errno == ENOENT ? Presence::Missing : Presence::Inaccessible;
#endif
        }
        inline bool EnsureDirectory(const std::string& path) noexcept {
#ifdef _WIN32
            return CreateDirectoryA(path.c_str(), nullptr) != 0 || GetLastError() == ERROR_ALREADY_EXISTS;
#else
            if (::mkdir(path.c_str(), 0770) == 0) return true;
            struct stat info{}; return errno == EEXIST && ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
#endif
        }
        inline bool ReadBytes(const std::string& path, std::string& bytes, std::size_t maximum) {
            std::ifstream input(path.c_str(), std::ios::binary);
            if (!input) return false;
            bytes.clear(); char buffer[4096];
            while (input) {
                input.read(buffer, sizeof(buffer)); const std::streamsize n = input.gcount();
                if (n < 0 || bytes.size() + static_cast<std::size_t>(n) > maximum) return false;
                bytes.append(buffer, static_cast<std::size_t>(n));
            }
            return input.eof() && !input.bad();
        }
        inline bool WriteBytes(const std::string& path, const std::string& bytes, bool createOnly) {
#ifdef _WIN32
            HANDLE h = CreateFileA(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr,
                createOnly ? CREATE_NEW : CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr);
            if (h == INVALID_HANDLE_VALUE) return false;
            bool okay = true; std::size_t position = 0U;
            while (position < bytes.size()) {
                DWORD done = 0U; const DWORD count = static_cast<DWORD>((std::min)(bytes.size() - position, std::size_t(65536U)));
                if (!WriteFile(h, bytes.data() + position, count, &done, nullptr) || done == 0U) { okay = false; break; }
                position += done;
            }
            if (!CloseHandle(h)) okay = false;
#else
            int h = ::open(path.c_str(), O_WRONLY | O_CREAT | (createOnly ? O_EXCL : O_TRUNC), 0660);
            if (h < 0) return false;
            bool okay = true; std::size_t position = 0U;
            while (position < bytes.size()) {
                const ssize_t done = ::write(h, bytes.data() + position, bytes.size() - position);
                if (done <= 0) { okay = false; break; } position += static_cast<std::size_t>(done);
            }
            if (::fsync(h) != 0) okay = false;
            if (::close(h) != 0) okay = false;
#endif
            std::string check;
            return okay && ReadBytes(path, check, bytes.size()) && check == bytes;
        }
        inline std::uint32_t Checksum(const std::string& bytes) noexcept {
            std::uint32_t result = 2166136261U;
            for (unsigned char c : bytes) { result ^= c; result *= 16777619U; }
            return result;
        }
        struct Record { std::uint64_t sequence = 0ULL; std::uint32_t tableId = 0U; Identity identity{}; };
        inline std::string RecordPrefix(const Record& r) {
            std::ostringstream s; s.imbue(std::locale::classic());
            s << "EDMCOND1 " << r.sequence << ' ' << r.tableId << ' ' << r.identity.schema << ' '
              << r.identity.profile << ' ' << r.identity.definition << ' ' << r.identity.catalog;
            return s.str();
        }
        inline std::string EncodeRecord(const Record& r) {
            const std::string prefix = RecordPrefix(r);
            return prefix + ' ' + std::to_string(Checksum(prefix)) + "\n";
        }
        inline bool ReadRecord(const std::string& path, Record& r) {
            std::string data; if (!ReadBytes(path, data, 256U)) return false;
            std::istringstream input(data); input.imbue(std::locale::classic());
            std::string magic; std::uint64_t table = 0ULL, schema = 0ULL, profile = 0ULL, definition = 0ULL, catalog = 0ULL, check = 0ULL;
            if (!(input >> magic >> r.sequence >> table >> schema >> profile >> definition >> catalog >> check)) return false;
            std::string extra; if (input >> extra) return false;
            if (magic != "EDMCOND1" || r.sequence == 0ULL || table == 0ULL || table > 99999999ULL ||
                schema == 0ULL || schema > 2ULL || profile > UINT32_MAX || definition > UINT32_MAX || catalog == 0ULL || catalog > UINT32_MAX) return false;
            r.tableId = static_cast<std::uint32_t>(table); r.identity.schema = static_cast<std::uint32_t>(schema);
            r.identity.profile = static_cast<std::uint32_t>(profile); r.identity.definition = static_cast<std::uint32_t>(definition);
            r.identity.catalog = static_cast<std::uint32_t>(catalog);
            return data == EncodeRecord(r) && check == Checksum(RecordPrefix(r));
        }
        inline std::string Slot(const std::string& directory, unsigned slot) {
            return Join(directory, slot == 0U ? "LastCondition0.dat" : "LastCondition1.dat");
        }
        inline bool LastRecord(const std::string& directory, Record& record, unsigned& slot, bool& damaged) {
            Record a, b; const bool va = ReadRecord(Slot(directory, 0U), a), vb = ReadRecord(Slot(directory, 1U), b);
            damaged = (!va && Probe(Slot(directory, 0U)) != Presence::Missing) || (!vb && Probe(Slot(directory, 1U)) != Presence::Missing);
            if (!va && !vb) return false;
            if (va && vb && a.sequence == b.sequence && EncodeRecord(a) != EncodeRecord(b)) { damaged = true; return false; }
            slot = vb && (!va || b.sequence > a.sequence) ? 1U : 0U; record = slot == 0U ? a : b; return true;
        }
        inline std::string TableLeaf(std::uint32_t id) {
            char name[32]{}; std::snprintf(name, sizeof(name), "COND%08u.csv", static_cast<unsigned>(id)); return name;
        }
        inline std::string TableBytes(const EDMRecipe::Catalog& c, std::uint32_t id) {
            std::ostringstream s; s.imbue(std::locale::classic());
            s << "SchemaVersion,CatalogRevision,MachineProfileId,DefinitionRevision,TableId,TableRevision,Name\r\n2,"
              << c.catalogRevision << ',' << c.profileId << ',' << c.definitionRevision << ',' << id << ",1," << TableLeaf(id).substr(0U, 12U) << "\r\nECode";
            std::string defaults;
            for (const auto& f : c.fields) {
                s << ",F" << f.id; defaults += ',';
                if (!f.enabled) continue;
                if (!f.stages.empty()) defaults += "S:" + std::to_string(f.stages.front().id);
                else {
                    const EDMRecipe::Value value = f.minimum > 0LL ? f.minimum : (f.maximum < 0LL ? f.maximum : 0LL);
                    char text[48]{}; EDMRecipe::FormatValue(value, text, sizeof(text)); defaults += std::string("V:") + text;
                }
            }
            s << "\r\n";
            for (std::size_t row = 1U; row <= EDMRecipe::RowsPerTable; ++row) s << row << defaults << "\r\n";
            return s.str();
        }
        inline std::string IndexBytes(const EDMRecipe::Catalog& c, const std::vector<std::string>& names) {
            std::ostringstream s; s.imbue(std::locale::classic());
            s << "SchemaVersion,CatalogRevision,MachineProfileId,DefinitionRevision\r\n2," << c.catalogRevision << ','
              << c.profileId << ',' << c.definitionRevision << "\r\nTableId\r\n";
            for (const auto& name : names) { std::uint32_t id = 0U; if (EDMRecipeConfigIO::Detail::ConditionFilename(name, id)) s << id << "\r\n"; }
            return s.str();
        }
        inline bool ReadIndex(const std::string& directory, const char* leaf, const EDMRecipe::Catalog& catalog,
            std::vector<std::string>& names, Diagnostic& d) {
            EDMRecipeConfigIO::Detail::Reader reader(Join(directory, leaf), leaf, 16384U, 128U, 1027U, d);
            EDMRecipeConfigIO::Detail::Columns columns;
            if (!reader.Good() || !EDMRecipeConfigIO::Detail::DefinitionMetadata(reader, columns, catalog)) return false;
            const char* const header[] = { "TableId" };
            if (!EDMRecipeConfigIO::Detail::ExactHeader(reader, columns, header, 1U)) return false;
            std::uint64_t previous = 0ULL;
            while (reader.Next()) {
                if (!EDMRecipeConfigIO::Detail::Split(reader, columns)) return false;
                if (columns.count != 1U) return reader.Reject(Error::Count, "TABLE_ID_COLUMNS");
                std::uint64_t id = 0ULL;
                if (!EDMRecipeConfigIO::Detail::Number(reader, columns.values[0], 99999999ULL, id, "TableId", true)) return false;
                if (id <= previous || names.size() >= 1024U) return reader.Reject(Error::Count, "TABLE_ID_ORDER");
                names.push_back(TableLeaf(static_cast<std::uint32_t>(id))); previous = id;
            }
            return reader.Good() && !names.empty();
        }
        inline bool LoadDefinitions(const std::string& directory, EDMRecipe::Catalog& c, Diagnostic& d) {
            EDMRecipeConfigIO::Detail::Counts counts;
            return EDMRecipeConfigIO::Detail::ReadCatalog(directory, c, counts, d) &&
                EDMRecipeConfigIO::Detail::ReadFields(directory, c, counts, d) &&
                EDMRecipeConfigIO::Detail::ReadStages(directory, c, counts, d);
        }
        inline bool BootstrapDefinitions(const std::string& directory, StartupResult& state) {
            const char* const names[] = { "Fields.csv", "Stages.csv", "Catalog.ini" };
            const char* const values[] = { EDMConditionDefaults::Fields, EDMConditionDefaults::Stages, EDMConditionDefaults::Catalog };
            bool anyMissing = false;
            for (unsigned i = 0U; i < 3U; ++i) {
                const Presence presence = Probe(Join(directory, names[i]));
                if (presence == Presence::Missing) anyMissing = true;
                else {
                    std::string existing;
                    // An interrupted first bootstrap may have created an exact seed subset.
                    if (!ReadBytes(Join(directory, names[i]), existing, 128U * 1024U) || existing != values[i]) return false;
                }
            }
            if (!anyMissing || !EnsureDirectory(directory)) return false;
            for (unsigned i = 0U; i < 3U; ++i)
                if (Probe(Join(directory, names[i])) == Presence::Missing && !WriteBytes(Join(directory, names[i]), values[i], true)) return false;
            state.generated = true; state.recovered = true; return true;
        }
    }

    // Bounded, checksummed, two-slot journal. Only table identity is retained;
    // E number and transient custom values are deliberately never serialized.
    inline bool SaveLastSelection(const char* directory, const Identity& identity, std::uint32_t tableId) noexcept {
        try {
            if (!directory || !*directory || tableId == 0U || tableId > 99999999U ||
                (identity.schema != 1U && identity.schema != 2U) || identity.catalog == 0U ||
                (identity.schema == 2U && (identity.profile == 0U || identity.definition == 0U))) return false;
            Detail::Record last; unsigned previous = 1U; bool damaged = false;
            const bool have = Detail::LastRecord(directory, last, previous, damaged);
            if (have && !damaged && last.tableId == tableId && SameIdentity(last.identity, identity)) return true;
            if (have && last.sequence == (std::numeric_limits<std::uint64_t>::max)()) return false;
            Detail::Record next; next.sequence = have ? last.sequence + 1ULL : 1ULL; next.tableId = tableId; next.identity = identity;
            return Detail::WriteBytes(Detail::Slot(directory, have ? (previous ^ 1U) : 0U), Detail::EncodeRecord(next), false);
        } catch (...) { return false; }
    }

    inline bool LoadForStartup(const char* directory, std::unique_ptr<EDMRecipe::Catalog>& result, StartupResult& state) noexcept {
        state = StartupResult{};
        try {
            if (!directory || !*directory) return Detail::Fail(state.diagnostic, Error::Directory, "EDM", 0U);
            const std::string base(directory);
            std::unique_ptr<EDMRecipe::Catalog> candidate(new EDMRecipe::Catalog());
            if (!Detail::LoadDefinitions(base, *candidate, state.diagnostic)) {
                if (!Detail::BootstrapDefinitions(base, state)) return false;
                candidate.reset(new EDMRecipe::Catalog());
                if (!Detail::LoadDefinitions(base, *candidate, state.diagnostic)) return false;
            }
            Detail::Record last; unsigned slot = 0U; bool damaged = false;
            const bool haveLast = Detail::LastRecord(base, last, slot, damaged);
            state.recovered = state.recovered || damaged;
            if (haveLast && !CompatibleProfile(last.identity, GetIdentity(*candidate))) { state.recovered = true; }
            std::uint32_t selected = haveLast && CompatibleProfile(last.identity, GetIdentity(*candidate)) ? last.tableId : 0U;
            if (candidate->schemaVersion == 1U) {
                if (!EDMRecipeConfigIO::LoadDirectory(directory, candidate, state.diagnostic) || candidate->tables.empty()) return false;
                if (selected == 0U) selected = candidate->tables.front().id;
                bool found = false; for (const auto& t : candidate->tables) if (t.id == selected) found = true;
                if (!found) return Detail::Fail(state.diagnostic, Error::Missing, "Tables.csv", 0U, "LAST_CONDITION");
            } else {
                std::vector<std::string> names;
                Diagnostic indexDiagnostic{};
                const bool goodIndex = EDMRecipeConfigIO::Detail::ReadConditionIndex(base, *candidate, names, indexDiagnostic);
                if (!goodIndex) {
                    names.clear(); state.recovered = true;
                    if (Detail::Probe(Detail::Join(base, "ConditionIndex.csv")) != Detail::Presence::Missing) {
                        // An invalid index is evidence, not a file to silently replace.
                        names.clear(); Diagnostic backupDiagnostic{};
                        if (!Detail::ReadIndex(base, "ConditionIndex.boot-backup.csv", *candidate, names, backupDiagnostic)) {
                            state.diagnostic = indexDiagnostic; return false;
                        }
                        state.emergencyDisplay = true; state.allowPersistence = false;
                        state.diagnostic = indexDiagnostic;
                    }
                }
                const std::vector<std::string> indexedNames = names;
                if (!goodIndex && names.empty() && selected == 0U &&
                    Detail::Probe(Detail::Join(Detail::Join(base, "COND"), Detail::TableLeaf(1U))) == Detail::Presence::Present)
                    names.push_back(Detail::TableLeaf(1U));
                if (selected == 0U) {
                    // One directory may contain several machine profiles. First boot
                    // chooses a matching table, never reinterprets another profile's ID.
                    for (const auto& name : names) {
                        if (Detail::Probe(Detail::Join(Detail::Join(base, "COND"), name)) == Detail::Presence::Missing) continue;
                        std::uint32_t id = 0U; EDMRecipeConfigIO::Detail::ConditionFilename(name, id);
                        EDMRecipe::Table probe; bool matches = false;
                        if (!EDMRecipeConfigIO::Detail::ReadCondition(base, name, *candidate, id, probe, matches, state.diagnostic)) return false;
                        if (matches) { selected = id; break; }
                    }
                    if (selected == 0U) {
                        for (std::uint32_t id = 1U; id <= 1025U; ++id)
                            if (Detail::Probe(Detail::Join(Detail::Join(base, "COND"), Detail::TableLeaf(id))) == Detail::Presence::Missing) { selected = id; break; }
                        if (selected == 0U) return Detail::Fail(state.diagnostic, Error::Count, "COND", 0U, "NO_FREE_DEFAULT_ID");
                    }
                }
                state.tableId = selected;
                const std::string selectedLeaf = Detail::TableLeaf(selected);
                const std::string selectedPath = Detail::Join(Detail::Join(base, "COND"), selectedLeaf);
                if (Detail::Probe(selectedPath) == Detail::Presence::Missing) {
                    if (state.emergencyDisplay) return Detail::Fail(state.diagnostic, Error::Missing, selectedLeaf.c_str(), 0U, "BACKUP_DISPLAY_ONLY");
                    if (!Detail::EnsureDirectory(Detail::Join(base, "COND")) ||
                        !Detail::WriteBytes(selectedPath, Detail::TableBytes(*candidate, selected), true))
                        return Detail::Fail(state.diagnostic, Error::Open, selectedLeaf.c_str(), 0U, "CREATE_MISSING_COND");
                    state.generated = true; state.recovered = true;
                }
                if (std::find(names.begin(), names.end(), selectedLeaf) == names.end()) { names.push_back(selectedLeaf); state.recovered = true; }
                std::sort(names.begin(), names.end());
                std::vector<std::string> loadedNames;
                for (const auto& name : names) {
                    std::uint32_t id = 0U; EDMRecipeConfigIO::Detail::ConditionFilename(name, id);
                    if (Detail::Probe(Detail::Join(Detail::Join(base, "COND"), name)) == Detail::Presence::Missing) { state.recovered = true; continue; }
                    EDMRecipe::Table table; bool matches = false;
                    if (!EDMRecipeConfigIO::Detail::ReadCondition(base, name, *candidate, id, table, matches, state.diagnostic)) return false;
                    if (!matches) { if (id == selected) return Detail::Fail(state.diagnostic, Error::Profile, name.c_str(), 0U, "LAST_CONDITION"); loadedNames.push_back(name); continue; }
                    if (candidate->tables.size() == EDMRecipe::MaximumTables) return Detail::Fail(state.diagnostic, Error::Count, "COND", 0U, "RESIDENT_TABLES");
                    loadedNames.push_back(name); candidate->tables.push_back(std::move(table));
                }
                bool selectedPresent = false; for (const auto& t : candidate->tables) if (t.id == selected) selectedPresent = true;
                if (!selectedPresent) return Detail::Fail(state.diagnostic, Error::Missing, selectedLeaf.c_str(), 0U);
                for (const auto& t : candidate->tables) if (t.id == selected) {
                    bool configured = false;
                    for (std::size_t f = 0U; f < candidate->fields.size(); ++f) if (t.cells[f].kind != EDMRecipe::CellKind::Unset) configured = true;
                    if (!t.rowPresent[0] || !configured) return Detail::Fail(state.diagnostic, Error::Missing, selectedLeaf.c_str(), 0U, "E1_EMPTY");
                }
                EDMRecipe::Validation validation;
                if (!EDMRecipe::ValidateCatalog(*candidate, validation)) return Detail::Fail(state.diagnostic, Error::Profile, "CATALOG", 0U, EDMRecipe::ErrorName(validation.error));
                if ((!goodIndex || indexedNames != loadedNames) && state.allowPersistence) {
                    const std::string indexPath = Detail::Join(base, "ConditionIndex.csv");
                    // Startup is not cyclic. Preserve the previous valid index before replacing it.
                    if (goodIndex) {
                        std::string prior;
                        if (!Detail::ReadBytes(indexPath, prior, 16384U) ||
                            !Detail::WriteBytes(Detail::Join(base, "ConditionIndex.boot-backup.csv"), prior, false)) state.saveFailed = true;
                    }
                    if (!state.saveFailed && !Detail::WriteBytes(indexPath, Detail::IndexBytes(*candidate, loadedNames), !goodIndex)) state.saveFailed = true;
                }
            }
            state.tableId = selected;
            for (const auto& t : candidate->tables) if (t.id == selected) {
                bool configured = false;
                for (std::size_t f = 0U; f < candidate->fields.size(); ++f) if (t.cells[f].kind != EDMRecipe::CellKind::Unset) configured = true;
                if (!t.rowPresent[0] || !configured) return Detail::Fail(state.diagnostic, Error::Missing, "COND", 0U, "E1_EMPTY");
            }
            if (state.allowPersistence && !SaveLastSelection(directory, GetIdentity(*candidate), selected)) state.saveFailed = true;
            result = std::move(candidate); return true;
        } catch (...) { return Detail::Fail(state.diagnostic, Error::Read, "EDM", 0U, "STARTUP_EXCEPTION"); }
    }
}
