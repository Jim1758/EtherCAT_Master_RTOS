#include "GMCodeHandlers.h"
#include "NCManager.h"
#include "AlarmManager.h"
#include <cmath>
#include <limits>
#include <windows.h>
#include <rtapi.h>

namespace GCodeHandlers
{
    namespace
    {
        using Timer = NCManager::G04TimerState;

        bool RejectG04(NCManager* nc, const char* reason)
        {
            if (nc == nullptr) return false;
            Reset_G04(nc);
            RtPrintf("[G04][BASE50] REJECT reason=%s alarm=2009\n", reason);
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->GetMotion().RequestEmergencyStopAllAxes();
            nc->ChangeState(NCState::ALARM);
            return false;
        }

        bool ReadCounter(std::uint64_t& ticks)
        {
            LARGE_INTEGER sample{};
            if (!RtQueryPerformanceCounter(&sample) || sample.QuadPart < 0) return false;
            ticks = static_cast<std::uint64_t>(sample.QuadPart);
            return true;
        }

        bool DecodeG04(const NCBlock& block, NCManager* nc, Timer& decoded)
        {
            // Preserve X precedence and the no-argument/zero no-op contract.
            // X is seconds and P is milliseconds, independent of G20/G21.
            const bool seconds = block.has('X');
            const double value = seconds ? block.val('X') :
                (block.has('P') ? block.val('P') : 0.0);
            if (!std::isfinite(value) || value < 0.0)
                return RejectG04(nc, "DURATION");
            if (value == 0.0) return true;

            LARGE_INTEGER frequency{};
            if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0)
                return RejectG04(nc, "FREQUENCY");
            const double infinity = (std::numeric_limits<double>::infinity)();
            const double secondsUpper = seconds ? value :
                std::nextafter(value / 1000.0, infinity);
            const double frequencyUpper = std::nextafter(
                static_cast<double>(frequency.QuadPart), infinity);
            const double ticks = secondsUpper * frequencyUpper;
            // Round outward before converting. The strict 2^63 bound avoids
            // an out-of-range float-to-integer cast on MSVC as well as HOST.
            const double rounded = std::ceil(std::nextafter(ticks,
                (std::numeric_limits<double>::infinity)()));
            if (!std::isfinite(rounded) || rounded >= 9223372036854775808.0)
                return RejectG04(nc, "TICK_RANGE");
            decoded.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
            decoded.targetTicks = static_cast<std::uint64_t>(rounded);
            if (decoded.targetTicks == 0ULL) decoded.targetTicks = 1ULL;
            return true;
        }

        void PublishRemaining(NCManager* nc)
        {
            const Timer& timer = nc->GetG04TimerSameThread();
            const std::uint64_t remaining = timer.targetTicks - timer.elapsedTicks;
            nc->SetG04TimeMs(timer.frequency == 0ULL ? 0.0 :
                (static_cast<double>(remaining) / static_cast<double>(timer.frequency)) * 1000.0);
        }

        bool SampleActive(NCManager* nc)
        {
            Timer& timer = nc->GetG04TimerSameThread();
            std::uint64_t now = 0ULL;
            if (!ReadCounter(now) || now < timer.lastTick)
                return RejectG04(nc, "COUNTER");
            const std::uint64_t delta = now - timer.lastTick;
            const std::uint64_t remaining = timer.targetTicks - timer.elapsedTicks;
            // Saturating accumulation cannot overflow even after a long
            // scheduling gap. No time is inferred from callback count.
            timer.elapsedTicks += delta < remaining ? delta : remaining;
            timer.lastTick = now;
            PublishRemaining(nc);
            return true;
        }

        bool CheckG04Done(NCManager* nc)
        {
            if (nc == nullptr || nc->GetState() != NCState::RUN ||
                AlarmManager::GetInstance().HasAlarm()) return false;
            Timer& timer = nc->GetG04TimerSameThread();
            if (timer.completed) return true;
            if (!timer.active) return false; // Cancelled waits cannot retire a block.
            if (timer.paused && !Resume_G04(nc)) return false;
            if (!SampleActive(nc)) return false;
            if (timer.elapsedTicks < timer.targetTicks) return false;
            timer.active = false;
            timer.completed = true;
            RtPrintf("[G04][BASE50] DONE targetTicks=%llu freq=%llu counter=%llu\n",
                static_cast<unsigned long long>(timer.targetTicks),
                static_cast<unsigned long long>(timer.frequency),
                static_cast<unsigned long long>(timer.lastTick));
            return true;
        }
    }

    void Reset_G04(NCManager* nc)
    {
        if (nc == nullptr) return;
        Timer& timer = nc->GetG04TimerSameThread();
        if (timer.active)
            RtPrintf("[G04][BASE50] CANCEL remainingTicks=%llu freq=%llu\n",
                static_cast<unsigned long long>(timer.targetTicks - timer.elapsedTicks),
                static_cast<unsigned long long>(timer.frequency));
        timer = Timer{};
        nc->SetG04TimeMs(0.0);
    }

    bool Pause_G04(NCManager* nc)
    {
        if (nc == nullptr) return false;
        Timer& timer = nc->GetG04TimerSameThread();
        if (!timer.active || timer.paused) return true;
        if (!SampleActive(nc)) return false;
        timer.paused = true;
        RtPrintf("[G04][BASE50] PAUSE remainingTicks=%llu freq=%llu counter=%llu\n",
            static_cast<unsigned long long>(timer.targetTicks - timer.elapsedTicks),
            static_cast<unsigned long long>(timer.frequency),
            static_cast<unsigned long long>(timer.lastTick));
        return true;
    }

    bool Resume_G04(NCManager* nc)
    {
        if (nc == nullptr) return false;
        Timer& timer = nc->GetG04TimerSameThread();
        if (!timer.active || !timer.paused) return true;
        if (nc->GetState() != NCState::RUN || AlarmManager::GetInstance().HasAlarm()) return false;
        std::uint64_t now = 0ULL;
        if (!ReadCounter(now) || now < timer.lastTick)
            return RejectG04(nc, "RESUME_COUNTER");
        timer.lastTick = now; // Explicitly exclude the complete HOLD interval.
        timer.paused = false;
        RtPrintf("[G04][BASE50] RESUME remainingTicks=%llu freq=%llu counter=%llu\n",
            static_cast<unsigned long long>(timer.targetTicks - timer.elapsedTicks),
            static_cast<unsigned long long>(timer.frequency),
            static_cast<unsigned long long>(timer.lastTick));
        return true;
    }

    bool ValidateG04Block(const NCBlock& block, NCManager* nc)
    {
        if (nc == nullptr) return false;
        Timer decoded{};
        return DecodeG04(block, nc, decoded);
    }

    WaitConditionFunc Handle_G04(const NCBlock& block, NCManager* nc)
    {
        if (nc == nullptr) return nullptr;
        Timer decoded{};
        if (!DecodeG04(block, nc, decoded)) return CheckG04Done;
        if (decoded.targetTicks == 0ULL)
        {
            Reset_G04(nc);
            return nullptr;
        }
        if (!ReadCounter(decoded.lastTick))
        {
            RejectG04(nc, "START_COUNTER");
            return CheckG04Done;
        }
        decoded.active = true;
        nc->GetG04TimerSameThread() = decoded;
        PublishRemaining(nc);
        RtPrintf("[G04][BASE50] START targetTicks=%llu freq=%llu counter=%llu holdPauses=1\n",
            static_cast<unsigned long long>(decoded.targetTicks),
            static_cast<unsigned long long>(decoded.frequency),
            static_cast<unsigned long long>(decoded.lastTick));
        return CheckG04Done;
    }
}
