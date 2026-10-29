#pragma once

// EDM282: NC-thread-owned, allocation-free M/T auxiliary handshake.
// PLC owns C10. Hold C10 ON until S51/S52 is OFF, then release C10.
// A fresh request is never asserted while C10 is already ON.
class NCAuxiliaryHandshake
{
public:
    enum class Kind { None, Tool, Misc };
    enum class Phase { Idle, WaitLow, Active, Complete };

    bool Begin(int tCode, int mCode) noexcept
    {
        if (IsPending() || tCode < -1 || mCode < -1) return false;
        m_count = 0;
        m_index = 0;
        if (tCode >= 0) m_items[m_count++] = Item{Kind::Tool, tCode};
        if (mCode >= 0) m_items[m_count++] = Item{Kind::Misc, mCode};
        m_phase = m_count ? Phase::WaitLow : Phase::Complete;
        return true;
    }

    void Service(bool fin, bool allowDispatch) noexcept
    {
        if (m_phase == Phase::Active)
        {
            if (fin)
            {
                ++m_index;
                m_phase = m_index < m_count ? Phase::WaitLow : Phase::Complete;
            }
            // Never acknowledge one item and assert another in the same scan.
            return;
        }
        if (m_phase != Phase::WaitLow || fin || !allowDispatch) return;
        const Item& item = m_items[m_index];
        if (item.kind == Kind::Tool) m_lastT = item.code;
        else m_lastM = item.code;
        m_phase = Phase::Active;
    }

    // Abandon a transaction without erasing the last published codes.
    // Cancellation is not completion; a surviving callback remains false.
    void Cancel() noexcept
    {
        m_phase = Phase::Idle;
        m_count = 0;
        m_index = 0;
    }

    // RESET/ALARM explicitly clear the user-facing registers.
    void Reset() noexcept
    {
        Cancel();
        m_lastM = -1;
        m_lastT = -1;
    }

    bool IsPending() const noexcept
    {
        return m_phase == Phase::WaitLow || m_phase == Phase::Active;
    }
    bool IsComplete() const noexcept { return m_phase == Phase::Complete; }
    bool IsActive() const noexcept { return m_phase == Phase::Active; }
    bool IsMActive() const noexcept { return ActiveKind() == Kind::Misc; }
    bool IsTActive() const noexcept { return ActiveKind() == Kind::Tool; }
    Kind ActiveKind() const noexcept
    {
        return IsActive() ? m_items[m_index].kind : Kind::None;
    }
    int ActiveCode() const noexcept
    {
        return IsActive() ? m_items[m_index].code : -1;
    }
    int LastM() const noexcept { return m_lastM; }
    int LastT() const noexcept { return m_lastT; }
    Phase CurrentPhase() const noexcept { return m_phase; }

private:
    struct Item
    {
        Kind kind = Kind::None;
        int code = -1;
    };
    Item m_items[2]{};
    unsigned m_count = 0;
    unsigned m_index = 0;
    int m_lastM = -1;
    int m_lastT = -1;
    Phase m_phase = Phase::Idle;
};
