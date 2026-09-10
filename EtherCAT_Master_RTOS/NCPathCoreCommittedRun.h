#pragma once

#include "MotionCommandedEndpointReceipt.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2BQ: one successful MEMORY NCStart owns at most 32 producer
// commanded receipts. This is a data service, never a Motion execution permit.
// The NC caller proves the matching committed dispatch and Ledger binding before
// Admit. No D/AY head is synthesized. Epoch changes and M00 do not erase rows.
// Native-MCS command interpolation is neither actual axis motion nor RT timing.

enum class NCPathCoreCommittedRunState : std::uint8_t
{
    IDLE = 0U, TRACKING = 1U, PREPARED = 2U, PUBLISHED = 3U,
    CLOSED = 4U, FAULTED = 5U
};

enum class NCPathCoreCommittedRunFault : std::uint8_t
{
    NONE = 0U, SCOPE = 1U, RECEIPT = 2U, IDENTITY = 3U,
    SEQUENCE = 4U, TRANSPORT = 5U, LEDGER = 6U, TRANSITION = 7U,
    CAPACITY = 8U, UNSUPPORTED = 9U, FINALIZE = 10U
};

enum NCPathCoreCommittedBoundaryFlag : std::uint8_t
{
    NC_PATH_CORE_COMMITTED_FIRST = 1U,
    NC_PATH_CORE_COMMITTED_EPOCH_CHANGED = 2U,
    NC_PATH_CORE_COMMITTED_AFTER_HOLD = 4U,
    NC_PATH_CORE_COMMITTED_DISCONTINUITY = 8U
};

struct NCPathCoreCommittedScopeV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t runToken = 0ULL;
    std::uint64_t cacheGeneration = 0ULL;
    std::uint32_t programScope = 0U;
    std::uint32_t operationMode = 0U;
    MotionOwnerLease ownerLease{};
    void Clear() noexcept;
    bool IsValid() const noexcept;
    bool Matches(const NCPathCoreCommittedScopeV1& other) const noexcept;
};

struct NCPathCoreCommittedRecordV1
{
    MotionCommandedEndpointReceiptV1 receipt{};
    std::uint64_t dispatchId = 0ULL;
    std::uint64_t commitSequence = 0ULL;
    MotionFeedbackSequence lastSequence = 0ULL;
    std::uint32_t errorCode = 0U;
    std::uint8_t boundaryFlags = 0U;
    MotionFeedbackType terminalType = MotionFeedbackType::NONE;
    MotionFeedbackType lastType = MotionFeedbackType::NONE;
    MotionRejectReason rejectReason = MotionRejectReason::NONE;
    bool consumerAccepted = false;
    bool started = false;
    std::uint8_t reserved[6U]{};
    void Clear() noexcept;
};

struct NCPathCoreCommittedSampleV1
{
    std::array<double, 8U> positionMCS{};
    std::array<double, 8U> derivativeMCS{};
    void Clear() noexcept;
};

struct NCPathCoreCommittedRunStatusV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t runToken = 0ULL;
    MotionFeedbackSequence lastSequence = 0ULL;
    std::uint32_t count = 0U;
    std::uint32_t accepted = 0U;
    std::uint32_t started = 0U;
    std::uint32_t completed = 0U;
    std::uint32_t failed = 0U;
    std::uint32_t pending = 0U;
    std::uint32_t epochChanges = 0U;
    std::uint32_t discontinuities = 0U;
    std::uint32_t holds = 0U;
    std::uint32_t resumes = 0U;
    NCPathCoreCommittedRunState state = NCPathCoreCommittedRunState::IDLE;
    NCPathCoreCommittedRunFault fault = NCPathCoreCommittedRunFault::NONE;
    bool held = false;
    std::uint8_t reserved[5U]{};
};

class NCPathCoreCommittedRun final
{
public:
    NCPathCoreCommittedRun() noexcept = default;
    NCPathCoreCommittedRun(const NCPathCoreCommittedRun&) = delete;
    NCPathCoreCommittedRun& operator=(const NCPathCoreCommittedRun&) = delete;
    NCPathCoreCommittedRun(NCPathCoreCommittedRun&&) = delete;
    NCPathCoreCommittedRun& operator=(NCPathCoreCommittedRun&&) = delete;
    ~NCPathCoreCommittedRun() = default;

    // Validate before changing state. Prior run must have been closed/published;
    // the same nonzero owner and a strictly newer nonwrapping run are required.
    bool Arm(const NCPathCoreCommittedScopeV1& scope,
        MotionFeedbackSequence lastConsumedSequence) noexcept;
    // Epoch is deliberately absent from this base scope; each receipt carries
    // its own producer epoch and full source identity.
    bool CheckScope(const NCPathCoreCommittedScopeV1& scope) noexcept;
    bool Admit(const MotionCommandedEndpointReceiptV1& receipt,
        std::uint64_t dispatchId, std::uint64_t commitSequence) noexcept;
    // Observe the existing NC consumer's ALL-event sequence after Ledger apply.
    // progress is ignored. ACCEPTED -> COMPLETED without STARTED is valid.
    void Observe(const MotionFeedbackEvent& event, bool ledgerAccepted) noexcept;
    void CheckTransport(bool healthy) noexcept;
    void Hold() noexcept;
    void Resume() noexcept;
    void Fail(NCPathCoreCommittedRunFault reason) noexcept;
    // Close preserves diagnostic counts and immediately revokes reads. Late
    // feedback cannot turn a pending-at-boundary row into a completed row.
    void Close() noexcept;
    // Prepare only after every retained row completed successfully. Publication
    // follows the caller's existing successful finalization gate; no authority
    // is granted by this class. Both stages freeze admission and feedback.
    bool Prepare() noexcept;
    bool Publish() noexcept;
    // BR: copy the last retained row only while the current open run is
    // tracking, unheld, healthy, and every retained row has consumer-confirmed
    // successful completion. This data read grants no Motion authority and
    // does not publish or freeze the run. Caller must revalidate its live scope
    // and keep output disjoint from this owner; failure clears the output.
    bool ReadLastCompleted(NCPathCoreCommittedRecordV1& output) const noexcept;
    // BT: bounded zero-based read under the same live all-completed guard.
    // It neither publishes nor grants Motion authority; failure clears output.
    bool ReadCompleted(std::uint32_t index,
        NCPathCoreCommittedRecordV1& output) const noexcept;
    // Caller output must be disjoint from this owner. Zero-based bounded read;
    // Evaluate returns native-MCS chord position and derivative with respect to
    // u, not velocity or Motion progress. All failure outputs are cleared.
    bool Read(std::uint32_t index, NCPathCoreCommittedRecordV1& output) const noexcept;
    bool Evaluate(std::uint32_t index, double u,
        NCPathCoreCommittedSampleV1& output) const noexcept;
    void Describe(NCPathCoreCommittedRunStatusV1& output) const noexcept;

private:
    bool IsObserving() const noexcept;
    NCPathCoreCommittedRecordV1 m_rows[32U]{};
    NCPathCoreCommittedScopeV1 m_scope{};
    MotionFeedbackSequence m_lastSequence = 0ULL;
    std::uint32_t m_count = 0U;
    std::uint32_t m_epochChanges = 0U;
    std::uint32_t m_discontinuities = 0U;
    std::uint32_t m_holds = 0U;
    std::uint32_t m_resumes = 0U;
    NCPathCoreCommittedRunState m_state = NCPathCoreCommittedRunState::IDLE;
    NCPathCoreCommittedRunFault m_fault = NCPathCoreCommittedRunFault::NONE;
    bool m_held = false;
    bool m_afterHold = false;
    bool m_runOpen = false;
    bool m_observeOpen = false;
};

static_assert(sizeof(NCPathCoreCommittedScopeV1) == 40U &&
    alignof(NCPathCoreCommittedScopeV1) == 8U &&
    sizeof(NCPathCoreCommittedRecordV1) == 256U &&
    alignof(NCPathCoreCommittedRecordV1) == 8U &&
    sizeof(NCPathCoreCommittedSampleV1) == 128U &&
    alignof(NCPathCoreCommittedSampleV1) == 8U &&
    sizeof(NCPathCoreCommittedRunStatusV1) == 72U &&
    alignof(NCPathCoreCommittedRunStatusV1) == 8U &&
    sizeof(NCPathCoreCommittedRun) == 8272U &&
    alignof(NCPathCoreCommittedRun) == 8U,
    "BQ fixed native layouts changed; these are not wire or SHM payloads.");
static_assert(offsetof(NCPathCoreCommittedRecordV1, receipt) == 0U &&
    offsetof(NCPathCoreCommittedRecordV1, dispatchId) == 216U &&
    offsetof(NCPathCoreCommittedRecordV1, commitSequence) == 224U &&
    offsetof(NCPathCoreCommittedRecordV1, lastSequence) == 232U &&
    offsetof(NCPathCoreCommittedRecordV1, errorCode) == 240U &&
    offsetof(NCPathCoreCommittedRecordV1, boundaryFlags) == 244U &&
    offsetof(NCPathCoreCommittedRecordV1, terminalType) == 245U &&
    offsetof(NCPathCoreCommittedRecordV1, lastType) == 246U &&
    offsetof(NCPathCoreCommittedRecordV1, rejectReason) == 247U &&
    offsetof(NCPathCoreCommittedRecordV1, consumerAccepted) == 248U &&
    offsetof(NCPathCoreCommittedRecordV1, started) == 249U &&
    offsetof(NCPathCoreCommittedRecordV1, reserved) == 250U &&
    offsetof(NCPathCoreCommittedRunStatusV1, state) == 64U &&
    offsetof(NCPathCoreCommittedRunStatusV1, fault) == 65U &&
    offsetof(NCPathCoreCommittedRunStatusV1, held) == 66U,
    "BQ native field offsets changed.");
static_assert(std::is_standard_layout<NCPathCoreCommittedScopeV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommittedScopeV1>::value&&
    std::is_standard_layout<NCPathCoreCommittedRecordV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommittedRecordV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommittedSampleV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommittedRunStatusV1>::value&&
    std::is_standard_layout<NCPathCoreCommittedRun>::value&&
    std::is_trivially_destructible<NCPathCoreCommittedRun>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommittedRun>::value &&
    !std::is_copy_constructible<NCPathCoreCommittedRun>::value &&
    !std::is_copy_assignable<NCPathCoreCommittedRun>::value &&
    !std::is_move_constructible<NCPathCoreCommittedRun>::value &&
    !std::is_move_assignable<NCPathCoreCommittedRun>::value,
    "BQ values and sole heap-owner traits changed.");
