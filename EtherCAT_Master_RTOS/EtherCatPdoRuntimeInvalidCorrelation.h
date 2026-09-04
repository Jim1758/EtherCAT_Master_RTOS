#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// NC-0.2K.7.2.1 - PDO Runtime Invalid Source Correlation Diagnostic
//
// This contract is diagnostic only.  The Priority-64 EtherCAT PDO owner feeds
// the tracker with values it has already computed for the LRW Process Data
// validity decision. DC WKC is retained as a correlated diagnostic, but a
// DC-only miss is not a Process Data invalid cycle after DC-RX.3A. No value in
// this record is consumed by Motion, RESET, Registry, read-ahead, EtherCAT
// output, or any admission gate.
// =============================================================================

enum class EtherCatPdoRuntimeInvalidEventKind : std::uint8_t
{
    NONE = 0,
    INVALID_CYCLE = 1,
    RECOVERED_AFTER_INVALID = 2,
    VALID_TICK_GAP = 3,
    VALIDITY_CONTRACT_MISMATCH = 4,
    VALIDITY_CONTRACT_RESTORED = 5
};

enum EtherCatPdoRuntimeInvalidReasonMask : std::uint32_t
{
    ECAT_PDO_INVALID_REASON_NONE = 0U,
    // A negative return from ecx_LRW()/ecx_LRW_FRMW().  This is deliberately
    // not named RX_TIMEOUT: the lower exchange function can also return -1
    // for send/frame/argument guards before an RX deadline is reached.
    ECAT_PDO_INVALID_REASON_LRW_CALL_NEGATIVE = 1U << 0U,
    ECAT_PDO_INVALID_REASON_LRW_WKC_MISMATCH = 1U << 1U,
    ECAT_PDO_INVALID_REASON_DC_WKC_INVALID = 1U << 2U,
    ECAT_PDO_INVALID_REASON_VALIDITY_CONTRACT_MISMATCH = 1U << 3U
};

struct EtherCatPdoRuntimeInvalidCorrelationSnapshot
{
    std::uint64_t eventSequence = 0ULL;
    std::uint64_t observationsAtLastEvent = 0ULL;
    std::uint64_t lastEventRuntimeCycleTick = 0ULL;

    std::uint64_t invalidCycleCount = 0ULL;
    std::uint64_t invalidEpisodeCount = 0ULL;
    std::uint64_t recoveryAfterInvalidCount = 0ULL;
    std::uint64_t validTickGapCount = 0ULL;
    std::uint64_t expectedNCSettleGapCount = 0ULL;

    std::uint64_t lrwCallNegativeCount = 0ULL;
    std::uint64_t lrwWkcMismatchCount = 0ULL;
    std::uint64_t dcWkcInvalidCount = 0ULL;
    std::uint64_t combinedLrwDcInvalidCount = 0ULL;
    std::uint64_t validityContractMismatchCount = 0ULL;

    std::uint64_t lastInvalidRuntimeCycleTick = 0ULL;
    std::uint64_t lastRecoveryRuntimeCycleTick = 0ULL;
    std::uint64_t lastGapFromRuntimeCycleTick = 0ULL;
    std::uint64_t lastGapToRuntimeCycleTick = 0ULL;

    // Existing Runtime "combined" path measurement.  It includes exchange
    // plus subsequent QPC/DC estimator work and is not a raw LRW/FRMW RTT.
    std::uint64_t lastEventCombinedPathNs = 0ULL;
    std::uint64_t lastInvalidCombinedPathNs = 0ULL;
    std::uint64_t lastRecoveryCombinedPathNs = 0ULL;
    std::uint64_t runtimePdoNegativeTotalAtLastEvent = 0ULL;
    std::uint64_t runtimePdoWkcErrorTotalAtLastEvent = 0ULL;

    std::int32_t lastInvalidActualLrwWkc = 0;
    std::int32_t lastInvalidExpectedLrwWkc = 0;
    std::int32_t lastInvalidDcWkc = 0;

    std::int32_t lastRecoveryActualLrwWkc = 0;
    std::int32_t lastRecoveryExpectedLrwWkc = 0;
    std::int32_t lastRecoveryDcWkc = 0;

    std::int32_t lastEventActualLrwWkc = 0;
    std::int32_t lastEventExpectedLrwWkc = 0;
    std::int32_t lastEventDcWkc = 0;

    std::uint32_t lastInvalidReasonMask =
        ECAT_PDO_INVALID_REASON_NONE;
    std::uint32_t lastEventReasonMask =
        ECAT_PDO_INVALID_REASON_NONE;
    std::uint32_t currentConsecutiveInvalidCycles = 0U;
    std::uint32_t maximumConsecutiveInvalidCycles = 0U;
    std::uint32_t lastRecoveredEpisodeLength = 0U;
    std::uint32_t lastEventSubTick = 0U;

    EtherCatPdoRuntimeInvalidEventKind lastEventKind =
        EtherCatPdoRuntimeInvalidEventKind::NONE;
    bool lastEventCycleValid = false;
    bool dcReferenceRequired = false;
};

static_assert(
    std::is_trivially_copyable<
    EtherCatPdoRuntimeInvalidCorrelationSnapshot>::value,
    "PDO invalid correlation snapshot must remain trivially copyable.");

class EtherCatPdoRuntimeInvalidCorrelationTracker
{
public:
    bool Observe(
        std::uint64_t runtimeCycleTick,
        bool processDataValid,
        std::int32_t actualLrwWkc,
        std::int32_t expectedLrwWkc,
        std::int32_t dcWkc,
        bool dcReferenceRequired,
        std::uint64_t combinedPathNs = 0ULL,
        std::uint64_t runtimePdoNegativeTotal = 0ULL,
        std::uint64_t runtimePdoWkcErrorTotal = 0ULL,
        std::uint32_t subTick = 0U) noexcept
    {
        IncrementSaturating(m_observationCount);

        const bool hadPreviousObservation = m_observed;
        const bool previousCycleValid = m_previousCycleValid;
        const std::uint64_t previousRuntimeCycleTick =
            m_previousRuntimeCycleTick;

        const bool lrwCallNegative = actualLrwWkc < 0;
        const bool lrwWkcMismatch =
            !lrwCallNegative && actualLrwWkc != expectedLrwWkc;
        const bool dcWkcInvalid =
            dcReferenceRequired && dcWkc <= 0;
        const bool lrwInvalid = lrwCallNegative || lrwWkcMismatch;
        const bool derivedProcessDataValid = !lrwInvalid;
        const bool validityContractMismatch =
            derivedProcessDataValid != processDataValid;
        const bool contractMismatchWasOpen =
            m_consecutiveContractMismatchCycles != 0U;
        const std::uint32_t previousContractMismatchReasonMask =
            m_previousContractMismatchReasonMask;

        std::uint32_t invalidReasonMask =
            ECAT_PDO_INVALID_REASON_NONE;
        if (lrwCallNegative)
        {
            invalidReasonMask |=
                ECAT_PDO_INVALID_REASON_LRW_CALL_NEGATIVE;
        }
        if (lrwWkcMismatch)
        {
            invalidReasonMask |=
                ECAT_PDO_INVALID_REASON_LRW_WKC_MISMATCH;
        }
        if (dcWkcInvalid)
        {
            invalidReasonMask |= ECAT_PDO_INVALID_REASON_DC_WKC_INVALID;
        }
        if (validityContractMismatch)
        {
            invalidReasonMask |=
                ECAT_PDO_INVALID_REASON_VALIDITY_CONTRACT_MISMATCH;
        }

        // DC WKC is counted independently from Process Data invalidity.
        // This preserves the physical/DC evidence while avoiding a false PDO
        // invalid episode when LRW itself is coherent.
        if (dcWkcInvalid)
        {
            IncrementSaturating(m_snapshot.dcWkcInvalidCount);
        }

        bool publishEvent = false;
        EtherCatPdoRuntimeInvalidEventKind eventKind =
            EtherCatPdoRuntimeInvalidEventKind::NONE;

        if (!processDataValid)
        {
            const bool invalidEpisodeStart =
                m_consecutiveInvalidCycles == 0U;
            const std::uint32_t previousInvalidReasonMask =
                m_snapshot.lastInvalidReasonMask;

            if (invalidEpisodeStart)
            {
                IncrementSaturating(m_snapshot.invalidEpisodeCount);
            }

            if (m_consecutiveInvalidCycles <
                (std::numeric_limits<std::uint32_t>::max)())
            {
                ++m_consecutiveInvalidCycles;
            }

            IncrementSaturating(m_snapshot.invalidCycleCount);
            if (lrwCallNegative)
            {
                IncrementSaturating(m_snapshot.lrwCallNegativeCount);
            }
            if (lrwWkcMismatch)
            {
                IncrementSaturating(m_snapshot.lrwWkcMismatchCount);
            }
            if (lrwInvalid && dcWkcInvalid)
            {
                IncrementSaturating(
                    m_snapshot.combinedLrwDcInvalidCount);
            }

            m_snapshot.lastInvalidRuntimeCycleTick = runtimeCycleTick;
            m_snapshot.lastInvalidActualLrwWkc = actualLrwWkc;
            m_snapshot.lastInvalidExpectedLrwWkc = expectedLrwWkc;
            m_snapshot.lastInvalidDcWkc = dcWkc;
            m_snapshot.lastInvalidCombinedPathNs = combinedPathNs;
            m_snapshot.lastInvalidReasonMask = invalidReasonMask;

            if (m_consecutiveInvalidCycles >
                m_snapshot.maximumConsecutiveInvalidCycles)
            {
                m_snapshot.maximumConsecutiveInvalidCycles =
                    m_consecutiveInvalidCycles;
            }

            eventKind = EtherCatPdoRuntimeInvalidEventKind::INVALID_CYCLE;
            // A sustained bus fault must not turn the 4 kHz owner into a
            // 4 kHz 240-byte atomic publisher.  Publish the episode edge,
            // source changes, and logarithmic 1/2/4/8... checkpoints.  The
            // recovery edge publishes the exact final counters and length.
            publishEvent =
                invalidEpisodeStart ||
                invalidReasonMask != previousInvalidReasonMask ||
                IsPowerOfTwo(m_consecutiveInvalidCycles);
        }
        else if (m_consecutiveInvalidCycles != 0U)
        {
            IncrementSaturating(m_snapshot.recoveryAfterInvalidCount);
            m_snapshot.lastRecoveredEpisodeLength =
                m_consecutiveInvalidCycles;
            m_snapshot.lastRecoveryRuntimeCycleTick = runtimeCycleTick;
            m_snapshot.lastRecoveryActualLrwWkc = actualLrwWkc;
            m_snapshot.lastRecoveryExpectedLrwWkc = expectedLrwWkc;
            m_snapshot.lastRecoveryDcWkc = dcWkc;
            m_snapshot.lastRecoveryCombinedPathNs = combinedPathNs;
            m_snapshot.lastGapFromRuntimeCycleTick =
                previousRuntimeCycleTick;
            m_snapshot.lastGapToRuntimeCycleTick = runtimeCycleTick;
            m_consecutiveInvalidCycles = 0U;

            eventKind =
                EtherCatPdoRuntimeInvalidEventKind::RECOVERED_AFTER_INVALID;
            publishEvent = true;
        }
        else if (
            hadPreviousObservation &&
            previousCycleValid &&
            runtimeCycleTick != previousRuntimeCycleTick + 1ULL)
        {
            IncrementSaturating(m_snapshot.validTickGapCount);
            m_snapshot.lastGapFromRuntimeCycleTick =
                previousRuntimeCycleTick;
            m_snapshot.lastGapToRuntimeCycleTick = runtimeCycleTick;

            eventKind = EtherCatPdoRuntimeInvalidEventKind::VALID_TICK_GAP;
            publishEvent = true;
        }

        if (validityContractMismatch)
        {
            IncrementSaturating(
                m_snapshot.validityContractMismatchCount);
            if (m_consecutiveContractMismatchCycles <
                (std::numeric_limits<std::uint32_t>::max)())
            {
                ++m_consecutiveContractMismatchCycles;
            }
            if (eventKind == EtherCatPdoRuntimeInvalidEventKind::NONE)
            {
                eventKind = EtherCatPdoRuntimeInvalidEventKind::
                    VALIDITY_CONTRACT_MISMATCH;
            }
            publishEvent =
                publishEvent ||
                !contractMismatchWasOpen ||
                invalidReasonMask !=
                previousContractMismatchReasonMask ||
                IsPowerOfTwo(m_consecutiveContractMismatchCycles);
            m_previousContractMismatchReasonMask = invalidReasonMask;
        }
        else if (contractMismatchWasOpen)
        {
            m_consecutiveContractMismatchCycles = 0U;
            m_previousContractMismatchReasonMask =
                ECAT_PDO_INVALID_REASON_NONE;
            if (eventKind == EtherCatPdoRuntimeInvalidEventKind::NONE)
            {
                eventKind = EtherCatPdoRuntimeInvalidEventKind::
                    VALIDITY_CONTRACT_RESTORED;
            }
            // Publish the exact cumulative mismatch count even when the
            // contract recovers between logarithmic checkpoints.
            publishEvent = true;
        }

        m_observed = true;
        m_previousRuntimeCycleTick = runtimeCycleTick;
        m_previousCycleValid = processDataValid;

        if (!publishEvent)
        {
            return false;
        }

        IncrementSaturating(m_snapshot.eventSequence);
        m_snapshot.observationsAtLastEvent = m_observationCount;
        m_snapshot.lastEventRuntimeCycleTick = runtimeCycleTick;
        m_snapshot.expectedNCSettleGapCount =
            AddSaturating(
                m_snapshot.recoveryAfterInvalidCount,
                m_snapshot.validTickGapCount);
        m_snapshot.currentConsecutiveInvalidCycles =
            m_consecutiveInvalidCycles;
        m_snapshot.lastEventKind = eventKind;
        m_snapshot.lastEventCycleValid = processDataValid;
        m_snapshot.dcReferenceRequired = dcReferenceRequired;
        m_snapshot.lastEventActualLrwWkc = actualLrwWkc;
        m_snapshot.lastEventExpectedLrwWkc = expectedLrwWkc;
        m_snapshot.lastEventDcWkc = dcWkc;
        m_snapshot.lastEventReasonMask = invalidReasonMask;
        m_snapshot.lastEventCombinedPathNs = combinedPathNs;
        m_snapshot.runtimePdoNegativeTotalAtLastEvent =
            runtimePdoNegativeTotal;
        m_snapshot.runtimePdoWkcErrorTotalAtLastEvent =
            runtimePdoWkcErrorTotal;
        m_snapshot.lastEventSubTick = subTick;
        return true;
    }

    const EtherCatPdoRuntimeInvalidCorrelationSnapshot& Snapshot()
        const noexcept
    {
        return m_snapshot;
    }

private:
    static bool IsPowerOfTwo(std::uint32_t value) noexcept
    {
        return value != 0U && (value & (value - 1U)) == 0U;
    }

    static void IncrementSaturating(std::uint64_t& value) noexcept
    {
        if (value != (std::numeric_limits<std::uint64_t>::max)())
        {
            ++value;
        }
    }

    static std::uint64_t AddSaturating(
        std::uint64_t left,
        std::uint64_t right) noexcept
    {
        const std::uint64_t maximum =
            (std::numeric_limits<std::uint64_t>::max)();
        return (maximum - left < right) ? maximum : left + right;
    }

    EtherCatPdoRuntimeInvalidCorrelationSnapshot m_snapshot{};
    std::uint64_t m_observationCount = 0ULL;
    std::uint64_t m_previousRuntimeCycleTick = 0ULL;
    std::uint32_t m_consecutiveInvalidCycles = 0U;
    std::uint32_t m_consecutiveContractMismatchCycles = 0U;
    std::uint32_t m_previousContractMismatchReasonMask =
        ECAT_PDO_INVALID_REASON_NONE;
    bool m_observed = false;
    bool m_previousCycleValid = false;
};

// =============================================================================
// NC-0.2K.7.2.2 - PDO Safety Stop Cause Latch + Alarm Bridge
//
// The Runtime owner calls this tracker only after the existing consecutive
// invalid-cycle threshold has already applied EmergencyStopAllAxes().  It
// never decides whether Motion must stop.  Its sole policy is to request one
// dedicated Alarm publication for that already-applied safety containment,
// preserve the exact K.7.2.1 source values, and request publication again if
// RESET clears Alarm while the same PDO fault is still present.
// =============================================================================

struct EtherCatPdoSafetyStopCauseSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t alarmRequestCount = 0ULL;
    std::uint64_t alarmedInvalidEpisodeCount = 0ULL;
    std::uint64_t alarmRepublishCount = 0ULL;

    std::uint64_t runtimeCycleTick = 0ULL;
    std::uint64_t invalidEpisodeCount = 0ULL;
    std::uint64_t combinedPathNs = 0ULL;
    std::uint64_t runtimePdoNegativeTotal = 0ULL;
    std::uint64_t runtimePdoWkcErrorTotal = 0ULL;

    std::int32_t actualLrwWkc = 0;
    std::int32_t expectedLrwWkc = 0;
    std::int32_t dcWkc = 0;
    std::int32_t alarmCode = 0;

    std::uint32_t reasonMask = ECAT_PDO_INVALID_REASON_NONE;
    std::uint32_t consecutiveInvalidCycles = 0U;
    std::uint32_t subTick = 0U;

    bool dcReferenceRequired = false;
    bool safetyContainmentApplied = false;
    bool alarmRequestRepublished = false;
    bool reserved = false;
};

static_assert(
    std::is_trivially_copyable<
    EtherCatPdoSafetyStopCauseSnapshot>::value,
    "PDO safety stop cause snapshot must remain trivially copyable.");

class EtherCatPdoSafetyStopAlarmBridgeTracker
{
public:
    bool ObserveSafetyContainment(
        bool alarmActive,
        std::int32_t alarmCode,
        const EtherCatPdoRuntimeInvalidCorrelationSnapshot& correlation,
        std::uint32_t consecutiveInvalidCycles,
        bool dcReferenceRequired,
        std::uint64_t runtimePdoNegativeTotal,
        std::uint64_t runtimePdoWkcErrorTotal,
        std::uint32_t subTick) noexcept
    {
        // AlarmManager Clear/RESET is global.  If Alarm is no longer active,
        // release only this publication latch.  A sustained PDO fault will
        // therefore re-publish immediately instead of leaving bare ESTOP.
        if (m_alarmRequestOutstanding && !alarmActive)
        {
            m_alarmRequestOutstanding = false;
        }

        if (m_alarmRequestOutstanding)
        {
            return false;
        }

        const bool republishSameEpisode =
            m_hasAlarmedInvalidEpisode &&
            correlation.invalidEpisodeCount ==
            m_lastAlarmedInvalidEpisode;

        IncrementSaturating(m_snapshot.publicationSequence);
        IncrementSaturating(m_snapshot.alarmRequestCount);
        if (republishSameEpisode)
        {
            IncrementSaturating(m_snapshot.alarmRepublishCount);
        }
        else
        {
            IncrementSaturating(
                m_snapshot.alarmedInvalidEpisodeCount);
        }

        m_snapshot.runtimeCycleTick =
            correlation.lastInvalidRuntimeCycleTick;
        m_snapshot.invalidEpisodeCount =
            correlation.invalidEpisodeCount;
        m_snapshot.combinedPathNs =
            correlation.lastInvalidCombinedPathNs;
        m_snapshot.runtimePdoNegativeTotal =
            runtimePdoNegativeTotal;
        m_snapshot.runtimePdoWkcErrorTotal =
            runtimePdoWkcErrorTotal;
        m_snapshot.actualLrwWkc =
            correlation.lastInvalidActualLrwWkc;
        m_snapshot.expectedLrwWkc =
            correlation.lastInvalidExpectedLrwWkc;
        m_snapshot.dcWkc = correlation.lastInvalidDcWkc;
        m_snapshot.alarmCode = alarmCode;
        m_snapshot.reasonMask = correlation.lastInvalidReasonMask;
        m_snapshot.consecutiveInvalidCycles =
            consecutiveInvalidCycles;
        m_snapshot.subTick = subTick;
        m_snapshot.dcReferenceRequired = dcReferenceRequired;
        m_snapshot.safetyContainmentApplied = true;
        m_snapshot.alarmRequestRepublished = republishSameEpisode;

        m_alarmRequestOutstanding = true;
        m_hasAlarmedInvalidEpisode = true;
        m_lastAlarmedInvalidEpisode =
            correlation.invalidEpisodeCount;
        return true;
    }

    const EtherCatPdoSafetyStopCauseSnapshot& Snapshot() const noexcept
    {
        return m_snapshot;
    }

private:
    static void IncrementSaturating(std::uint64_t& value) noexcept
    {
        if (value != (std::numeric_limits<std::uint64_t>::max)())
        {
            ++value;
        }
    }

    EtherCatPdoSafetyStopCauseSnapshot m_snapshot{};
    std::uint64_t m_lastAlarmedInvalidEpisode = 0ULL;
    bool m_alarmRequestOutstanding = false;
    bool m_hasAlarmedInvalidEpisode = false;
};

// Bounded, read-only Priority-50 seam implemented by
// EtherCatMaster_DC_Runtime.cpp.  False means the rare event publisher was
// overlapping the read; the caller retries on its next supervisory sample.
bool TryReadEtherCatPdoRuntimeInvalidCorrelation(
    EtherCatPdoRuntimeInvalidCorrelationSnapshot& snapshot) noexcept;

// Bounded, read-only Priority-50 seam for the K.7.2.2 cause latch.
bool TryReadEtherCatPdoSafetyStopCause(
    EtherCatPdoSafetyStopCauseSnapshot& snapshot) noexcept;
