#include "AlarmManager.h"
// #include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 或 RtPrintf 請取消註解

/*

// 發現 G 碼寫錯，直接呼叫 (此時 axisIndex 自動 = -1)
AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);

// 或是帶上行號，但依然不需要軸號
AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR, 15);


// 假設現在迴圈跑到第 2 軸 (Z 軸)，發生了追隨誤差
int axisId = 2; // 0=X, 1=Y, 2=Z...
AlarmManager::GetInstance().Trigger(AlarmManager::AXIS_LAG_ERROR, 0, axisId);

*/



// Do not replace this with a function-local static.  MSVC's thread-safe local
// static guard is not safe to enter from the RTX64 PDO callback, even after a
// non-RT prewarm call.  Module static construction finishes before that timer
// exists, and GetInstance() below remains guard-free on every RT invocation.
AlarmManager AlarmManager::s_instance;

AlarmManager::AlarmManager() {
    m_alarmCount.store(0, std::memory_order_relaxed);
    m_hasAlarm.store(false, std::memory_order_relaxed);
    m_updateCount.store(0U, std::memory_order_relaxed);
    m_motionSafetyIntentState.store(0ULL, std::memory_order_relaxed);
    m_deferredAlarmPublication.store(0ULL, std::memory_order_relaxed);
    m_deferredAlarmOverflowPayload.store(0ULL, std::memory_order_relaxed);
    m_alarmWriterReserved.store(false, std::memory_order_relaxed);
    for (int i = 0; i < MAX_ALARMS; ++i) {
        m_alarms[i].code.store(0, std::memory_order_relaxed);
        m_alarms[i].lineNo.store(0, std::memory_order_relaxed);
        m_alarms[i].axisIndex.store(-1, std::memory_order_relaxed);
    }
}

AlarmManager& AlarmManager::GetInstance() {
    return s_instance;
}

void AlarmManager::Trigger(int code, int lineNo, int axisIndex) {
    // One RMW declares the intent before any Alarm image mutation.  The low
    // 31-bit active count is intentionally enormous relative to the fixed
    // Runtime thread set; overflow would require 2^31 simultaneous callers.
    const std::uint64_t previousIntentState =
        m_motionSafetyIntentState.fetch_add(
            MOTION_ALARM_INTENT_BEGIN_DELTA,
            std::memory_order_acq_rel);

    if ((previousIntentState & MOTION_ADMISSION_RESERVED) != 0ULL)
    {
        // The already-admitted operation linearizes first.  Never wait and
        // never mutate its Alarm image; publish one bounded deferred record.
        DeferAlarm(code, lineNo, axisIndex);
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        TryPromoteDeferredAlarms();
        return;
    }

    bool writerExpected = false;
    if (!m_alarmWriterReserved.compare_exchange_strong(
        writerExpected,
        true,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        DeferAlarm(code, lineNo, axisIndex);
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        TryPromoteDeferredAlarms();
        return;
    }

    PublishAlarmUnderWriter(code, lineNo, axisIndex, 1U);
    m_alarmWriterReserved.store(false, std::memory_order_release);
    m_motionSafetyIntentState.fetch_sub(
        1ULL,
        std::memory_order_acq_rel);
    TryPromoteDeferredAlarms();
}

void AlarmManager::Clear() {
    // Promotion is bounded (two attempts).  A still-pending publication is a
    // logical Alarm and must never be erased by a best-effort legacy Clear.
    TryPromoteDeferredAlarms();
    if (HasDeferredAlarmPublication() ||
        !m_hasAlarm.load(std::memory_order_acquire))
    {
        return;
    }

    const std::uint64_t previousIntentState =
        m_motionSafetyIntentState.fetch_add(
            MOTION_ALARM_INTENT_BEGIN_DELTA,
            std::memory_order_acq_rel);
    if ((previousIntentState & MOTION_ADMISSION_RESERVED) != 0ULL)
    {
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        TryPromoteDeferredAlarms();
        return;
    }

    bool writerExpected = false;
    if (!m_alarmWriterReserved.compare_exchange_strong(
        writerExpected,
        true,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        TryPromoteDeferredAlarms();
        return;
    }

    // A Trigger which lost the writer race is newer than this Clear.  Leave
    // the old image latched and let the deferred publisher materialize it.
    if (HasDeferredAlarmPublication())
    {
        m_alarmWriterReserved.store(false, std::memory_order_release);
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        TryPromoteDeferredAlarms();
        return;
    }

    m_hasAlarm.store(false, std::memory_order_release);
    m_alarmCount.store(0, std::memory_order_release);
    for (int i = 0; i < MAX_ALARMS; ++i) {
        m_alarms[i].code.store(0, std::memory_order_relaxed);
        m_alarms[i].lineNo.store(0, std::memory_order_relaxed);
        m_alarms[i].axisIndex.store(-1, std::memory_order_relaxed);
    }
    m_updateCount.fetch_add(1U, std::memory_order_acq_rel);
    m_alarmWriterReserved.store(false, std::memory_order_release);
    m_motionSafetyIntentState.fetch_sub(
        1ULL,
        std::memory_order_acq_rel);
    TryPromoteDeferredAlarms();
}

bool AlarmManager::HasAlarm() const {
    return
        m_hasAlarm.load(std::memory_order_acquire) ||
        HasDeferredAlarmPublication();
}

int AlarmManager::GetAlarmCount() const {
    return m_alarmCount.load(std::memory_order_acquire);
}

int AlarmManager::GetAlarmId(int index) const {
    if (index >= 0 &&
        index < m_alarmCount.load(std::memory_order_acquire)) {
        return m_alarms[index].code.load(std::memory_order_acquire);
    }
    return 0;
}

int AlarmManager::GetAlarmAxisIndex(int index) const {
    if (index >= 0 &&
        index < m_alarmCount.load(std::memory_order_acquire)) {
        return m_alarms[index].axisIndex.load(std::memory_order_acquire);
    }
    return -1;
}

uint32_t AlarmManager::GetUpdateCount() const {
    return m_updateCount.load(std::memory_order_acquire);
}


std::uint64_t AlarmManager::GetMotionSafetyIntentState() const noexcept
{
    return m_motionSafetyIntentState.load(std::memory_order_acquire);
}


std::uint64_t AlarmManager::MotionAdmissionBaseState(
    std::uint64_t state) noexcept
{
    return state & ~MOTION_ADMISSION_RESERVED;
}


AlarmManager::MotionAdmissionResult
AlarmManager::TryBeginMotionAdmission(
    std::uint32_t expectedUpdateCount,
    MotionAdmissionReservation& reservation,
    bool requireNoAlarm) noexcept
{
    return TryBeginMotionAdmissionImpl(
        expectedUpdateCount,
        nullptr,
        reservation,
        requireNoAlarm);
}


AlarmManager::MotionAdmissionResult
AlarmManager::TryBeginMotionAdmission(
    std::uint32_t expectedUpdateCount,
    std::uint64_t expectedIntentState,
    MotionAdmissionReservation& reservation,
    bool requireNoAlarm) noexcept
{
    return TryBeginMotionAdmissionImpl(
        expectedUpdateCount,
        &expectedIntentState,
        reservation,
        requireNoAlarm);
}


bool AlarmManager::BeginMotionAdmission(
    std::uint32_t expectedUpdateCount,
    MotionAdmissionReservation& reservation,
    bool requireNoAlarm) noexcept
{
    return
        TryBeginMotionAdmission(
            expectedUpdateCount,
            reservation,
            requireNoAlarm) == MotionAdmissionResult::ACQUIRED;
}


bool AlarmManager::IsMotionAdmissionCurrent(
    const MotionAdmissionReservation& reservation) const noexcept
{
    return
        reservation.acquired &&
        m_motionSafetyIntentState.load(std::memory_order_acquire) ==
        reservation.reservedState &&
        m_updateCount.load(std::memory_order_acquire) ==
        reservation.expectedUpdateCount &&
        !HasDeferredAlarmPublication() &&
        !m_alarmWriterReserved.load(std::memory_order_acquire) &&
        (!reservation.requireNoAlarm ||
            !m_hasAlarm.load(std::memory_order_acquire));
}


bool AlarmManager::EndMotionAdmission(
    MotionAdmissionReservation& reservation) noexcept
{
    if (!reservation.acquired)
    {
        return false;
    }

    const bool current = IsMotionAdmissionCurrent(reservation);
    std::uint64_t expectedReservedState = reservation.reservedState;
    const bool released =
        m_motionSafetyIntentState.compare_exchange_strong(
            expectedReservedState,
            reservation.baseState,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
    if (!released)
    {
        m_motionSafetyIntentState.fetch_and(
            ~MOTION_ADMISSION_RESERVED,
            std::memory_order_acq_rel);
    }

    reservation.acquired = false;
    TryPromoteDeferredAlarms();
    return current && released;
}


bool AlarmManager::ClearUnderMotionAdmission(
    MotionAdmissionReservation& reservation) noexcept
{
    if (!reservation.acquired ||
        m_motionSafetyIntentState.load(std::memory_order_acquire) !=
        reservation.reservedState ||
        m_updateCount.load(std::memory_order_acquire) !=
        reservation.expectedUpdateCount ||
        HasDeferredAlarmPublication() ||
        m_alarmWriterReserved.load(std::memory_order_acquire))
    {
        return false;
    }

    if (!m_hasAlarm.load(std::memory_order_acquire))
    {
        return true;
    }

    std::uint64_t expectedReservedState = reservation.reservedState;
    const std::uint64_t advancedReservedState =
        reservation.reservedState +
        MOTION_ALARM_INTENT_SEQUENCE_DELTA;
    if (!m_motionSafetyIntentState.compare_exchange_strong(
        expectedReservedState,
        advancedReservedState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    reservation.baseState += MOTION_ALARM_INTENT_SEQUENCE_DELTA;
    reservation.reservedState = advancedReservedState;
    m_hasAlarm.store(false, std::memory_order_release);
    m_alarmCount.store(0, std::memory_order_release);
    for (int i = 0; i < MAX_ALARMS; ++i)
    {
        m_alarms[i].code.store(0, std::memory_order_relaxed);
        m_alarms[i].lineNo.store(0, std::memory_order_relaxed);
        m_alarms[i].axisIndex.store(-1, std::memory_order_relaxed);
    }
    m_updateCount.fetch_add(1U, std::memory_order_acq_rel);
    ++reservation.expectedUpdateCount;
    return IsMotionAdmissionCurrent(reservation);
}


bool AlarmManager::TriggerUnderMotionAdmission(
    MotionAdmissionReservation& reservation,
    int code,
    int lineNo,
    int axisIndex) noexcept
{
    // This is the sole authorised way to materialize an already-classified
    // pre-boundary Alarm (for example delayed pre-Reset mapping Alarm 3021).
    // A no-Alarm admission cannot intentionally publish an Alarm.
    if (!reservation.acquired ||
        reservation.requireNoAlarm ||
        !IsMotionAdmissionCurrent(reservation))
    {
        return false;
    }

    bool writerExpected = false;
    if (!m_alarmWriterReserved.compare_exchange_strong(
        writerExpected,
        true,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    std::uint64_t expectedReservedState = reservation.reservedState;
    const std::uint64_t advancedReservedState =
        reservation.reservedState +
        MOTION_ALARM_INTENT_SEQUENCE_DELTA;
    if (!m_motionSafetyIntentState.compare_exchange_strong(
        expectedReservedState,
        advancedReservedState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_alarmWriterReserved.store(false, std::memory_order_release);
        return false;
    }

    reservation.baseState += MOTION_ALARM_INTENT_SEQUENCE_DELTA;
    reservation.reservedState = advancedReservedState;
    PublishAlarmUnderWriter(code, lineNo, axisIndex, 1U);
    ++reservation.expectedUpdateCount;
    m_alarmWriterReserved.store(false, std::memory_order_release);
    return IsMotionAdmissionCurrent(reservation);
}


void AlarmManager::DeferAlarm(
    int code,
    int lineNo,
    int axisIndex) noexcept
{
    const std::uint64_t payload =
        (static_cast<std::uint64_t>(
            static_cast<std::uint16_t>(code)) &
            DEFERRED_ALARM_CODE_MASK) |
        ((static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(lineNo)) <<
            DEFERRED_ALARM_LINE_SHIFT) &
            DEFERRED_ALARM_LINE_MASK) |
        ((static_cast<std::uint64_t>(
            static_cast<std::uint8_t>(axisIndex)) <<
            DEFERRED_ALARM_AXIS_SHIFT) &
            DEFERRED_ALARM_AXIS_MASK);

    std::uint64_t observed =
        m_deferredAlarmPublication.load(std::memory_order_acquire);
    for (unsigned attempt = 0U;
        attempt < DEFERRED_ALARM_CAS_ATTEMPTS;
        ++attempt)
    {
        const std::uint32_t count =
            static_cast<std::uint32_t>(
                (observed & DEFERRED_ALARM_COUNT_MASK) >>
                DEFERRED_ALARM_COUNT_SHIFT);
        std::uint64_t desired = observed;
        if (count == 0U)
        {
            // Preserve the first payload until one writer promotes the whole
            // coalesced incident.  A prior overflow-only marker is retained.
            desired =
                (observed & DEFERRED_ALARM_OVERFLOW) |
                payload |
                (1ULL << DEFERRED_ALARM_COUNT_SHIFT);
        }
        else if (count < DEFERRED_ALARM_COUNT_MAX)
        {
            desired =
                (observed & ~DEFERRED_ALARM_COUNT_MASK) |
                (static_cast<std::uint64_t>(count + 1U) <<
                    DEFERRED_ALARM_COUNT_SHIFT);
        }
        else
        {
            desired = observed | DEFERRED_ALARM_OVERFLOW;
        }

        if (m_deferredAlarmPublication.compare_exchange_weak(
            observed,
            desired,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            return;
        }
    }

    // A fixed retry budget keeps every producer bounded.  The overflow bit
    // is itself a logical pending Alarm even if promotion exchanged the old
    // first payload during all attempts.
    m_deferredAlarmOverflowPayload.store(
        payload,
        std::memory_order_release);
    m_deferredAlarmPublication.fetch_or(
        DEFERRED_ALARM_OVERFLOW,
        std::memory_order_acq_rel);
}


void AlarmManager::TryPromoteDeferredAlarms() noexcept
{
    // Two non-blocking handoff attempts close the common writer-release race;
    // a remaining publication stays visible through HasAlarm and is retried
    // by the next Trigger/admission without weakening safety.
    TryPromoteDeferredAlarmsOnce();
    TryPromoteDeferredAlarmsOnce();
}


void AlarmManager::TryPromoteDeferredAlarmsOnce() noexcept
{
    if (!HasDeferredAlarmPublication())
    {
        return;
    }

    const std::uint64_t previousIntentState =
        m_motionSafetyIntentState.fetch_add(
            MOTION_ALARM_INTENT_BEGIN_DELTA,
            std::memory_order_acq_rel);
    if ((previousIntentState & MOTION_ADMISSION_RESERVED) != 0ULL)
    {
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        return;
    }

    bool writerExpected = false;
    if (!m_alarmWriterReserved.compare_exchange_strong(
        writerExpected,
        true,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        m_motionSafetyIntentState.fetch_sub(
            1ULL,
            std::memory_order_acq_rel);
        return;
    }

    const std::uint64_t publication =
        m_deferredAlarmPublication.exchange(
            0ULL,
            std::memory_order_acq_rel);
    if (publication != 0ULL)
    {
        std::uint32_t deferredCount =
            static_cast<std::uint32_t>(
                (publication & DEFERRED_ALARM_COUNT_MASK) >>
                DEFERRED_ALARM_COUNT_SHIFT);
        const bool overflow =
            (publication & DEFERRED_ALARM_OVERFLOW) != 0ULL;
        int code = static_cast<int>(
            publication & DEFERRED_ALARM_CODE_MASK);
        const int lineNo = static_cast<int>(
            static_cast<std::uint32_t>(
                (publication & DEFERRED_ALARM_LINE_MASK) >>
                DEFERRED_ALARM_LINE_SHIFT));
        const int axisIndex = static_cast<int>(
            static_cast<std::int8_t>(
                (publication & DEFERRED_ALARM_AXIS_MASK) >>
                DEFERRED_ALARM_AXIS_SHIFT));

        if (deferredCount == 0U)
        {
            // Overflow-only is possible only after repeated CAS contention.
            // Keep the safety incident visible even when its diagnostic
            // payload could not win the bounded first-payload publication.
            const std::uint64_t overflowPayload =
                m_deferredAlarmOverflowPayload.load(
                    std::memory_order_acquire);
            deferredCount = 1U;
            code = static_cast<int>(
                overflowPayload & DEFERRED_ALARM_CODE_MASK);
            const int overflowLineNo = static_cast<int>(
                static_cast<std::uint32_t>(
                    (overflowPayload & DEFERRED_ALARM_LINE_MASK) >>
                    DEFERRED_ALARM_LINE_SHIFT));
            const int overflowAxisIndex = static_cast<int>(
                static_cast<std::int8_t>(
                    (overflowPayload & DEFERRED_ALARM_AXIS_MASK) >>
                    DEFERRED_ALARM_AXIS_SHIFT));
            PublishAlarmUnderWriter(
                code == AlarmManager::NONE
                ? AlarmManager::SHM_LINK_ERROR
                : code,
                overflowLineNo,
                overflowAxisIndex,
                deferredCount);
            m_alarmWriterReserved.store(false, std::memory_order_release);
            m_motionSafetyIntentState.fetch_sub(
                1ULL,
                std::memory_order_acq_rel);
            return;
        }
        else if (overflow)
        {
            ++deferredCount;
        }

        PublishAlarmUnderWriter(
            code,
            lineNo,
            axisIndex,
            deferredCount);
    }

    m_alarmWriterReserved.store(false, std::memory_order_release);
    m_motionSafetyIntentState.fetch_sub(
        1ULL,
        std::memory_order_acq_rel);
}


bool AlarmManager::HasDeferredAlarmPublication() const noexcept
{
    return
        m_deferredAlarmPublication.load(std::memory_order_acquire) !=
        0ULL;
}


void AlarmManager::PublishAlarmUnderWriter(
    int code,
    int lineNo,
    int axisIndex,
    std::uint32_t updateDelta) noexcept
{
    const int count = m_alarmCount.load(std::memory_order_acquire);
    if (count >= 0 && count < MAX_ALARMS)
    {
        m_alarms[count].code.store(code, std::memory_order_relaxed);
        m_alarms[count].lineNo.store(lineNo, std::memory_order_relaxed);
        m_alarms[count].axisIndex.store(
            axisIndex,
            std::memory_order_relaxed);
        m_alarmCount.store(count + 1, std::memory_order_release);
    }

    m_hasAlarm.store(true, std::memory_order_release);
    m_updateCount.fetch_add(
        updateDelta == 0U ? 1U : updateDelta,
        std::memory_order_acq_rel);
}


AlarmManager::MotionAdmissionResult
AlarmManager::TryBeginMotionAdmissionImpl(
    std::uint32_t expectedUpdateCount,
    const std::uint64_t* expectedIntentState,
    MotionAdmissionReservation& reservation,
    bool requireNoAlarm) noexcept
{
    reservation = MotionAdmissionReservation{};

    if (HasDeferredAlarmPublication())
    {
        TryPromoteDeferredAlarms();
    }

    const std::uint32_t updateCount =
        m_updateCount.load(std::memory_order_acquire);
    const bool alarmPresent = HasAlarm();
    std::uint64_t state =
        m_motionSafetyIntentState.load(std::memory_order_acquire);

    if (updateCount != expectedUpdateCount ||
        (requireNoAlarm && alarmPresent))
    {
        return MotionAdmissionResult::SUPERSEDED;
    }

    if (expectedIntentState != nullptr &&
        state != *expectedIntentState)
    {
        // A holder on the exact expected quiet state is temporary.  Every
        // sequence/active change denotes a newer Alarm/Clear publication.
        if (MotionAdmissionBaseState(state) ==
            *expectedIntentState &&
            (state & MOTION_ADMISSION_RESERVED) != 0ULL)
        {
            return MotionAdmissionResult::BUSY;
        }
        return MotionAdmissionResult::SUPERSEDED;
    }

    if (HasDeferredAlarmPublication())
    {
        return requireNoAlarm
            ? MotionAdmissionResult::SUPERSEDED
            : MotionAdmissionResult::BUSY;
    }
    if ((state &
        (MOTION_ADMISSION_ACTIVE_MASK |
            MOTION_ADMISSION_RESERVED)) != 0ULL ||
        m_alarmWriterReserved.load(std::memory_order_acquire))
    {
        return MotionAdmissionResult::BUSY;
    }

    const std::uint64_t baseState = state;
    const std::uint64_t reservedState =
        baseState | MOTION_ADMISSION_RESERVED;
    if (!m_motionSafetyIntentState.compare_exchange_strong(
        state,
        reservedState,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        const bool superseded =
            m_updateCount.load(std::memory_order_acquire) !=
            expectedUpdateCount ||
            (requireNoAlarm && HasAlarm()) ||
            (expectedIntentState != nullptr &&
                MotionAdmissionBaseState(state) !=
                *expectedIntentState);
        return superseded
            ? MotionAdmissionResult::SUPERSEDED
            : MotionAdmissionResult::BUSY;
    }

    const bool identityCurrent =
        m_updateCount.load(std::memory_order_acquire) ==
        expectedUpdateCount &&
        (!requireNoAlarm || !HasAlarm()) &&
        !HasDeferredAlarmPublication() &&
        !m_alarmWriterReserved.load(std::memory_order_acquire) &&
        m_motionSafetyIntentState.load(std::memory_order_acquire) ==
        reservedState;
    if (!identityCurrent)
    {
        std::uint64_t expectedReservedState = reservedState;
        if (!m_motionSafetyIntentState.compare_exchange_strong(
            expectedReservedState,
            baseState,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
            m_motionSafetyIntentState.fetch_and(
                ~MOTION_ADMISSION_RESERVED,
                std::memory_order_acq_rel);
        }
        TryPromoteDeferredAlarms();

        const bool superseded =
            m_updateCount.load(std::memory_order_acquire) !=
            expectedUpdateCount ||
            (requireNoAlarm && HasAlarm()) ||
            HasDeferredAlarmPublication() ||
            (expectedIntentState != nullptr &&
                MotionAdmissionBaseState(
                    m_motionSafetyIntentState.load(
                        std::memory_order_acquire)) !=
                *expectedIntentState);
        return superseded
            ? MotionAdmissionResult::SUPERSEDED
            : MotionAdmissionResult::BUSY;
    }

    reservation.baseState = baseState;
    reservation.reservedState = reservedState;
    reservation.expectedUpdateCount = expectedUpdateCount;
    reservation.requireNoAlarm = requireNoAlarm;
    reservation.acquired = true;
    return MotionAdmissionResult::ACQUIRED;
}
