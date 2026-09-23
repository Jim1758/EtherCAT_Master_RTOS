#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// BASE52. Explicit, non-RT saves only. This is a checked backup-before-write
// protocol, NOT an atomic rename, cross-file transaction, or power-loss proof.
// BASE53: a missing target with any existing/unreadable .bak is NOT a
// first-save case. Preserve both names; do not manufacture a new target from
// RAM defaults and subsequently rotate those defaults over the old backup.
// No automatic recovery/load from .bak. The caller serializes writers.
// The checks are not an external-editor concurrency/namespace transaction.
namespace CoordinateFileSave
{
constexpr std::size_t MaxFileBytes = 1024U * 1024U;

enum class IOCode : std::uint8_t { OK, MISSING, OPEN, READ, SIZE, WRITE, CLOSE };
enum class WriteMode : std::uint8_t { BACKUP_REPLACE, TARGET_REPLACE, TARGET_NEW };
struct IOResult
{
    IOCode code = IOCode::OK;
    std::uint32_t error = 0U;
};
struct Result
{
    bool ok = false;
    const char* reason = "NONE";
    const char* detail = "OK";
    std::uint32_t error = 0U;
    bool hadPrevious = false;
    bool targetMayBePartial = false;
    bool backupMayBePartial = false;
    bool backupVerified = false;
    bool readbackVerified = false;
    const char* backupProbe = "NOT_NEEDED";
    bool firstSaveAllowed = false;
};
inline const char* CodeName(IOCode code) noexcept
{
    switch (code)
    {
    case IOCode::OK: return "OK";
    case IOCode::MISSING: return "MISSING";
    case IOCode::OPEN: return "OPEN";
    case IOCode::READ: return "READ";
    case IOCode::SIZE: return "SIZE";
    case IOCode::WRITE: return "WRITE";
    case IOCode::CLOSE: return "CLOSE";
    }
    return "UNKNOWN";
}

// Hold the existing source against writes/deletes through backup verification.
// This also prevents an existing backup alias/hard link from truncating source.
template <typename IO> class PreviousReadGuard
{
public:
    explicit PreviousReadGuard(IO& io) : io_(io) {}
    ~PreviousReadGuard() { if (active_) io_.ClosePrevious(); }
    IOResult Close() { active_ = false; return io_.ClosePrevious(); }
    PreviousReadGuard(const PreviousReadGuard&) = delete;
    PreviousReadGuard& operator=(const PreviousReadGuard&) = delete;
private:
    IO& io_;
    bool active_ = true;
};

// IO.ReadPrevious retains its read-only handle until ClosePrevious.
// IO.Read is bounded by MaxFileBytes and closes its handle before return.
// IO.Write reports short writes and close failures, and closes on all paths.
// validatePrevious returns nullptr only for a complete, valid prior file.
// A corrupt/short prior file must NEVER replace a previously verified backup.
// First-save is allowed only if both target AND backup read as MISSING.
// A backup that exists (even empty/invalid) or cannot be checked is preserved.
// This guard does not load backup values into RAM or repair either file.
template <typename IO, typename Validator>
Result Write(IO& io, const std::string& path, const std::string& payload,
    Validator validatePrevious)
{
    Result result;
    if (payload.empty() || payload.size() > MaxFileBytes)
    { result.reason = "TEXT_SIZE"; return result; }
    const std::string backupPath = path + ".bak";
    std::string previous;
    IOResult status = io.ReadPrevious(path, previous);
    PreviousReadGuard<IO> previousGuard(io);
    if (status.code != IOCode::OK && status.code != IOCode::MISSING)
    {
        result.reason = "PREVIOUS_READ";
        result.detail = CodeName(status.code); result.error = status.error;
        return result;
    }
    result.hadPrevious = status.code == IOCode::OK;
    if (!result.hadPrevious)
    {
        // Read-only, bounded probe using the existing adapter. Do not validate
        // the backup here: even invalid contents may be recovery evidence.
        // Permission/sharing/read/size/close errors do NOT mean "absent".
        std::string backup;
        status = io.Read(backupPath, backup);
        result.backupProbe = CodeName(status.code);
        if (status.code == IOCode::OK)
        {
            result.reason = "MISSING_WITH_BACKUP";
            result.detail = "BACKUP_PRESENT";
            return result;
        }
        if (status.code != IOCode::MISSING)
        {
            result.reason = "MISSING_BACKUP_CHECK";
            result.detail = CodeName(status.code);
            result.error = status.error;
            return result;
        }
        result.firstSaveAllowed = true;
    }
    if (result.hadPrevious)
    {
        const char* invalid = validatePrevious(previous);
        if (invalid != nullptr)
        { result.reason = "PREVIOUS_INVALID"; result.detail = invalid; return result; }

        // Only the backup is opened with truncation at this point. If any
        // backup operation fails, the target has not been opened for writing.
        result.backupMayBePartial = true;
        status = io.Write(backupPath, previous, WriteMode::BACKUP_REPLACE);
        if (status.code != IOCode::OK)
        {
            result.reason = "BACKUP_WRITE";
            result.detail = CodeName(status.code); result.error = status.error;
            return result;
        }
        std::string check;
        status = io.Read(backupPath, check);
        if (status.code != IOCode::OK || check != previous)
        {
            result.reason = "BACKUP_VERIFY";
            result.detail = status.code == IOCode::OK ? "MISMATCH" : CodeName(status.code);
            result.error = status.error;
            return result;
        }
        result.backupVerified = true;
        result.backupMayBePartial = false;
    }

    status = previousGuard.Close();
    if (status.code != IOCode::OK)
    {
        result.reason = "PREVIOUS_CLOSE";
        result.detail = CodeName(status.code); result.error = status.error;
        return result;
    }
    // No fallback from a failed backup to an unprotected target write.
    // Existing targets must still exist. First-save uses exclusive CREATE_NEW.
    result.targetMayBePartial = true; // conservative even for an open failure
    status = io.Write(path, payload, result.hadPrevious ?
        WriteMode::TARGET_REPLACE : WriteMode::TARGET_NEW);
    if (status.code != IOCode::OK)
    {
        result.reason = "TARGET_WRITE";
        result.detail = CodeName(status.code); result.error = status.error;
        return result;
    }
    std::string check;
    status = io.Read(path, check);
    if (status.code != IOCode::OK || check != payload)
    {
        result.reason = "TARGET_VERIFY";
        result.detail = status.code == IOCode::OK ? "MISMATCH" : CodeName(status.code);
        result.error = status.error;
        return result;
    }
    result.ok = true; result.reason = "OK";
    result.targetMayBePartial = false;
    result.readbackVerified = true;
    return result;
}
} // namespace CoordinateFileSave
