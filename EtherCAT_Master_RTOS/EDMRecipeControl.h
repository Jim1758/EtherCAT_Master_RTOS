#pragma once

#include "EDMRecipe.h"
#include <cstdint>
#include <limits>

// EDM17: exact optimistic control over the real owner-thread recipe Controller.
// NC owns run/cache/epoch/lease and stopped authority; this layer neither grants
// that authority nor performs transport, allocation, I/O, Motion or discharge.
namespace EDMRecipeControl
{
    enum class Operation : std::uint8_t
    { Read = 0U, SelectStage = 1U, StepStage = 2U, SetActual = 3U, ResetOverride = 4U };
    enum class Reason : std::uint8_t
    {
        None, InvalidRequest, NotReady, NoSelection, InvalidKey, StaleKey,
        FieldUnavailable, DomainRejected, Invariant,
        AuthorityUnavailable, AuthorityStale, AuthorityExhausted, ScopeStale, Revoked
    };

    inline const char* OperationName(Operation operation) noexcept
    {
        switch (operation)
        {
        case Operation::Read: return "READ";
        case Operation::SelectStage: return "SELECT_STAGE";
        case Operation::StepStage: return "STEP_STAGE";
        case Operation::SetActual: return "SET_ACTUAL";
        case Operation::ResetOverride: return "RESET_OVERRIDE";
        }
        return "UNKNOWN";
    }
    inline const char* ReasonName(Reason reason) noexcept
    {
        switch (reason)
        {
        case Reason::None: return "NONE";
        case Reason::InvalidRequest: return "INVALID_REQUEST";
        case Reason::NotReady: return "NOT_READY";
        case Reason::NoSelection: return "NO_SELECTION";
        case Reason::InvalidKey: return "INVALID_KEY";
        case Reason::StaleKey: return "STALE_KEY";
        case Reason::FieldUnavailable: return "FIELD_UNAVAILABLE";
        case Reason::DomainRejected: return "DOMAIN_REJECTED";
        case Reason::Invariant: return "INVARIANT";
        case Reason::AuthorityUnavailable: return "AUTHORITY_UNAVAILABLE";
        case Reason::AuthorityStale: return "AUTHORITY_STALE";
        case Reason::AuthorityExhausted: return "AUTHORITY_EXHAUSTED";
        case Reason::ScopeStale: return "SCOPE_STALE";
        case Reason::Revoked: return "REVOKED";
        }
        return "UNKNOWN";
    }

    struct Key
    {
        std::uint32_t catalogRevision = 0U, tableId = 0U, tableRevision = 0U;
        std::uint32_t profileId = 0U, definitionRevision = 0U;
        std::uint16_t eCode = 0U;
        std::uint64_t generation = 0ULL;
    };
    inline bool IsKeyValid(const Key& key) noexcept
    {
        return key.catalogRevision != 0U && key.tableId != 0U && key.tableRevision != 0U &&
            key.eCode != 0U && key.eCode <= EDMRecipe::RowsPerTable && key.generation != 0ULL;
    }
    inline bool SameKey(const Key& a, const Key& b) noexcept
    {
        return a.catalogRevision == b.catalogRevision && a.tableId == b.tableId &&
            a.tableRevision == b.tableRevision && a.eCode == b.eCode && a.generation == b.generation &&
            a.profileId == b.profileId && a.definitionRevision == b.definitionRevision;
    }
    inline Key CaptureKey(const EDMRecipe::Summary& summary) noexcept
    {
        Key key{};
        if (!summary.catalogReady || !summary.tableSelected || !summary.rowSelected) return key;
        key.catalogRevision = summary.catalogRevision;
        key.tableId = summary.tableId;
        key.tableRevision = summary.tableRevision;
        key.eCode = summary.eCode;
        key.profileId = summary.profileId; key.definitionRevision = summary.definitionRevision;
        key.generation = summary.generation;
        return IsKeyValid(key) ? key : Key{};
    }

    // EDM19 detached optimistic HMI command contract. NC supplies authority;
    // these values grant no permission and contain no live pointers.
    enum class HmiOperation : std::uint8_t
    { SelectTable = 1U, SelectE = 2U, SelectStage = 3U, StepStage = 4U, SetActual = 5U, ResetOverride = 6U, ReloadCatalog = 7U };
    enum class HmiStatus : std::uint8_t
    { Success, Busy, Stale, NotAllowed, InvalidCommand, DomainRejected };
    struct HmiContext
    {
        std::uint64_t generation = 0ULL, controlGeneration = 0ULL;
        std::uint32_t schemaVersion = 0U, profileId = 0U, definitionRevision = 0U;
        std::uint32_t catalogRevision = 0U, tableRevision = 0U, tableId = 0U;
        std::uint16_t eCode = 0U;
        bool canEdit = false, canReload = false;
    };
    struct HmiCommand
    {
        HmiOperation operation = HmiOperation::SelectTable;
        std::uint32_t tableId = 0U;
        std::uint16_t eCode = 0U, fieldId = 0U, stageId = 0U;
        std::int32_t step = 0;
        EDMRecipe::Value actualValue = 0LL;
    };
    struct HmiResult
    {
        HmiStatus status = HmiStatus::InvalidCommand;
        EDMRecipe::Error domainError = EDMRecipe::Error::None;
        HmiContext after{};
    };
    inline bool SameHmiKey(const HmiContext& a, const HmiContext& b) noexcept
    {
        return a.generation == b.generation && a.controlGeneration == b.controlGeneration &&
            a.schemaVersion == b.schemaVersion && a.profileId == b.profileId &&
            a.definitionRevision == b.definitionRevision && a.catalogRevision == b.catalogRevision &&
            a.tableRevision == b.tableRevision && a.tableId == b.tableId && a.eCode == b.eCode;
    }
    inline bool IsHmiCommandCanonical(const HmiCommand& command) noexcept
    {
        switch (command.operation)
        {
        case HmiOperation::SelectTable:
            return command.tableId != 0U && command.tableId <= EDMRecipe::MaximumConditionTableId &&
                command.eCode == 0U && command.fieldId == 0U && command.stageId == 0U && command.step == 0 && command.actualValue == 0LL;
        case HmiOperation::SelectE:
            return command.tableId == 0U && command.eCode >= 1U && command.eCode <= EDMRecipe::RowsPerTable &&
                command.fieldId == 0U && command.stageId == 0U && command.step == 0 && command.actualValue == 0LL;
        case HmiOperation::SelectStage:
            return command.tableId == 0U && command.eCode == 0U && command.fieldId != 0U &&
                command.stageId != 0U && command.step == 0 && command.actualValue == 0LL;
        case HmiOperation::StepStage:
            return command.tableId == 0U && command.eCode == 0U && command.fieldId != 0U &&
                command.stageId == 0U && (command.step == -1 || command.step == 1) && command.actualValue == 0LL;
        case HmiOperation::SetActual:
            // stageId==0 retains the current anchor; nonzero atomically selects
            // the requested anchor and explicit custom actual value together.
            return command.tableId == 0U && command.eCode == 0U && command.fieldId != 0U && command.step == 0;
        case HmiOperation::ResetOverride:
            return command.tableId == 0U && command.eCode == 0U && command.fieldId != 0U &&
                command.stageId == 0U && command.step == 0 && command.actualValue == 0LL;
        case HmiOperation::ReloadCatalog:
            return command.tableId == 0U && command.eCode == 0U && command.fieldId == 0U &&
                command.stageId == 0U && command.step == 0 && command.actualValue == 0LL;
        }
        return false;
    }

    struct Request
    {
        // Correlation only. Reusing this number cannot bypass expected-key checks.
        std::uint64_t requestId = 0ULL;
        Operation op = Operation::Read;
        std::uint16_t fieldId = 0U;
        // Stage/direction use unscaled integers; SetActual uses six-place Value.
        // Read and ResetOverride require exactly zero (no hidden extra payload).
        EDMRecipe::Value argument = 0LL;
    };
    inline bool IsRequestCanonical(const Request& request) noexcept
    {
        if (request.requestId == 0ULL || request.fieldId == 0U) return false;
        switch (request.op)
        {
        case Operation::Read:
        case Operation::ResetOverride: return request.argument == 0LL;
        case Operation::SelectStage: return request.argument >= 1LL && request.argument <= 65535LL;
        case Operation::StepStage: return request.argument == -1LL || request.argument == 1LL;
        case Operation::SetActual: return true;
        }
        return false;
    }

    struct Result
    {
        bool success = false, applied = false;
        Reason reason = Reason::InvalidRequest;
        EDMRecipe::Error domainError = EDMRecipe::Error::None;
        std::uint64_t requestId = 0ULL;
        Operation op = Operation::Read;
        std::uint16_t fieldId = 0U;
        EDMRecipe::Summary before{}, after{};
        bool beforeFieldValid = false, afterFieldValid = false;
        EDMRecipe::ParameterSnapshot beforeField{}, afterField{};
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    inline void ResetResult(const Request& request, Result& result) noexcept
    {
        result = Result{};
        result.requestId = request.requestId;
        result.op = request.op;
        result.fieldId = request.fieldId;
    }

    namespace Detail
    {
        inline void CaptureAfter(const EDMRecipe::Controller& controller, Result& result) noexcept
        {
            result.after = controller.Current();
            result.afterFieldValid = controller.ReadField(result.fieldId, result.afterField);
        }
        inline bool Reject(Result& result, Reason reason) noexcept
        {
            result.reason = reason;
            return false;
        }
    }

    inline bool Apply(EDMRecipe::Controller& controller, const Key& expected,
        const Request& request, Result& result) noexcept
    {
        ResetResult(request, result);
        result.before = controller.Current();
        result.beforeFieldValid = controller.ReadField(request.fieldId, result.beforeField);
        result.after = result.before;
        result.afterFieldValid = result.beforeFieldValid;
        result.afterField = result.beforeField;
        if (!IsRequestCanonical(request)) return Detail::Reject(result, Reason::InvalidRequest);
        if (!controller.Ready() || !result.before.catalogReady) return Detail::Reject(result, Reason::NotReady);
        if (!result.before.tableSelected || !result.before.rowSelected) return Detail::Reject(result, Reason::NoSelection);
        if (!IsKeyValid(expected)) return Detail::Reject(result, Reason::InvalidKey);
        const Key beforeKey = CaptureKey(result.before);
        if (!SameKey(expected, beforeKey)) return Detail::Reject(result, Reason::StaleKey);
        if (!result.beforeFieldValid) return Detail::Reject(result, Reason::FieldUnavailable);

        if (request.op == Operation::Read)
        {
            // Read is observational: it does not clear an earlier domain error,
            // consume a generation or change a field's current stage/override.
            result.success = true;
            result.reason = Reason::None;
            return true;
        }

        bool changed = false;
        switch (request.op)
        {
        case Operation::SelectStage:
            changed = controller.SelectStage(request.fieldId, static_cast<std::uint16_t>(request.argument));
            break;
        case Operation::StepStage:
            changed = controller.StepStage(request.fieldId, static_cast<int>(request.argument));
            break;
        case Operation::SetActual:
            changed = controller.SetActual(request.fieldId, request.argument);
            break;
        case Operation::ResetOverride:
            changed = controller.ResetOverride(request.fieldId);
            break;
        default: return Detail::Reject(result, Reason::InvalidRequest);
        }
        Detail::CaptureAfter(controller, result);
        if (!changed)
        {
            // Includes the existing permanent exhaustion closure; report the
            // actual after state, never the request's assumed previous values.
            result.domainError = result.after.lastError;
            return Detail::Reject(result, Reason::DomainRejected);
        }
        Key committed = beforeKey;
        const bool canAdvance = committed.generation != (std::numeric_limits<std::uint64_t>::max)();
        if (canAdvance) ++committed.generation;
        if (!canAdvance || !result.afterFieldValid || !SameKey(committed, CaptureKey(result.after)))
        {
            controller.Clear();
            Detail::CaptureAfter(controller, result);
            result.domainError = result.after.lastError;
            return Detail::Reject(result, Reason::Invariant);
        }
        result.success = result.applied = true;
        result.reason = Reason::None;
        return true;
    }

    static_assert(sizeof(Result) <= 512U, "EDM17 control result must remain a small fixed snapshot");
}
