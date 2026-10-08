#pragma once
#include "EDMProcessTuningContract.h"
#include "EDMProcessProfile.h"
#include <cstring>

namespace EDMProcessTuning
{
    inline bool Zero(const void* location, std::size_t size) noexcept
    {
        const auto* bytes = static_cast<const unsigned char*>(location);
        for (std::size_t i = 0; i < size; ++i) if (bytes[i] != 0U) return false;
        return true;
    }
    inline bool Canonical(const EDMProcessTuningRequest& request) noexcept
    {
        if (request.SessionId == 0ULL || request.RequestId == 0ULL || request.ExpectedGeneration == 0ULL ||
            request.RuntimeEpoch == 0ULL || request.SelectionMask == 0U ||
            (request.SelectionMask & ~7U) != 0U || !Zero(request.Reserved, sizeof(request.Reserved))) return false;
        const auto& values = request.Values;
        if ((request.SelectionMask & 1U) == 0U) {
            if (!Zero(&values.Process, sizeof(values.Process))) return false;
        } else if (values.Process.Reserved32 != 0U || (values.Process.Flags & ~7U) != 0U ||
            !Zero(values.Process.Reserved, sizeof(values.Process.Reserved))) return false;
        for (unsigned bank = 0U; bank < 2U; ++bank) {
            const auto* gains = bank == 0U ? values.Discharge : values.Flush;
            if ((request.SelectionMask & (2U << bank)) == 0U) {
                if (!Zero(gains, 8U * sizeof(*gains))) return false;
            } else for (unsigned axis = 0U; axis < 8U; ++axis)
                if (gains[axis].InheritCnc > 1U || gains[axis].Reserved != 0U) return false;
        }
        return true;
    }
    inline void Encode(const EDM20::ProcessProfile& profile, EDMProcessTuningValues& out) noexcept
    {
        out = EDMProcessTuningValues{};
        auto& p = out.Process;
        const auto& s = profile.servo;
        const auto& f = profile.flush;
        p.SchemaVersion = profile.schemaVersion; p.Revision = profile.revision;
        p.MachineProfileId = profile.machineProfileId; p.Mode = static_cast<std::uint32_t>(profile.mode);
        p.Flags = (s.limitsConfirmed ? 1U : 0U) | (s.shortIdleEnabled ? 2U : 0U) | (s.shortMachiningEnabled ? 4U : 0U);
        p.ShortEnterMs = s.shortEnterMs; p.ShortExitMs = s.shortExitMs;
        for (unsigned i = 0U; i < 3U; ++i) { p.PositiveGain[i] = s.positiveGain[i]; p.NegativeGain[i] = s.negativeGain[i]; }
        for (unsigned i = 0U; i < 2U; ++i) { p.PositiveBreakV[i] = s.positiveBreakV[i]; p.NegativeBreakV[i] = s.negativeBreakV[i]; }
        p.CuttingScaleMmPerVoltMin = s.cuttingScaleMmPerVoltMin;
        p.MaxFeedMmPerMin = s.maxFeedMmPerMin;
        p.MaxRetreatMmPerMin = s.maxRetreatMmPerMin;
        p.DeadbandV = s.deadbandV;
        p.ShortIdleV = s.shortIdleV;
        p.ShortMachiningV = s.shortMachiningV;
        p.ShortHysteresisV = s.shortHysteresisV;
        p.ShortRetreatMmPerMin = s.shortRetreatMmPerMin;
        p.FinalApproachMmPerMin = f.finalApproachMmPerMin;
        p.CenterRetractMmPerMin = f.centerRetractMmPerMin;
        p.MainRetractMmPerMin = f.mainRetractMmPerMin;
        p.CenterReturnMmPerMin = f.centerReturnMmPerMin;
        p.MainReturnMmPerMin = f.mainReturnMmPerMin;
        p.PathRetractMmPerMin = f.pathRetractMmPerMin;
        p.InitialSlowMmPerMin = f.initialSlowMmPerMin;
        p.ManualReturnMmPerMin = f.manualReturnMmPerMin;
        p.FinalApproachDistanceMm = f.finalApproachDistanceMm;
        p.PathDistanceMm = f.pathDistanceMm;
        p.InitialSlowDistanceMm = f.initialSlowDistanceMm;
        for (unsigned axis = 0U; axis < 8U; ++axis) {
            const EDM20::AxisGainProfile source[2] = { profile.axisGains.discharge[axis], profile.axisGains.flush[axis] };
            EDMProcessTuningAxisGain* target[2] = { &out.Discharge[axis], &out.Flush[axis] };
            for (unsigned bank = 0U; bank < 2U; ++bank) {
                target[bank]->InheritCnc = source[bank].inheritCnc ? 1U : 0U;
                target[bank]->Kp = source[bank].custom.Kp; target[bank]->Ki = source[bank].custom.Ki;
                target[bank]->Kd = source[bank].custom.Kd; target[bank]->Kvff = source[bank].custom.Kvff;
            }
        }
    }
    // Identity stays immutable during live tuning; the NC owner increments
    // revision after all selected blocks have validated as one candidate.
    inline bool DecodeSelected(const EDMProcessTuningRequest& request,
        const EDM20::ProcessProfile& current, EDM20::ProcessProfile& out) noexcept
    {
        if (!Canonical(request)) return false;
        EDM20::ProcessProfile candidate = current;
        const auto& p = request.Values.Process;
        if ((request.SelectionMask & 1U) != 0U) {
            if (p.SchemaVersion != current.schemaVersion || p.Revision != current.revision ||
                p.MachineProfileId != current.machineProfileId || p.Mode != 0U) return false;
            auto& s = candidate.servo;
            auto& f = candidate.flush;
            s.limitsConfirmed = (p.Flags & 1U) != 0U;
            s.shortIdleEnabled = (p.Flags & 2U) != 0U; s.shortMachiningEnabled = (p.Flags & 4U) != 0U;
            s.shortEnterMs = p.ShortEnterMs; s.shortExitMs = p.ShortExitMs;
            for (unsigned i = 0U; i < 3U; ++i) { s.positiveGain[i] = p.PositiveGain[i]; s.negativeGain[i] = p.NegativeGain[i]; }
            for (unsigned i = 0U; i < 2U; ++i) { s.positiveBreakV[i] = p.PositiveBreakV[i]; s.negativeBreakV[i] = p.NegativeBreakV[i]; }
            s.cuttingScaleMmPerVoltMin = p.CuttingScaleMmPerVoltMin;
            s.maxFeedMmPerMin = p.MaxFeedMmPerMin;
            s.maxRetreatMmPerMin = p.MaxRetreatMmPerMin;
            s.deadbandV = p.DeadbandV;
            s.shortIdleV = p.ShortIdleV;
            s.shortMachiningV = p.ShortMachiningV;
            s.shortHysteresisV = p.ShortHysteresisV;
            s.shortRetreatMmPerMin = p.ShortRetreatMmPerMin;
            f.finalApproachMmPerMin = p.FinalApproachMmPerMin;
            f.centerRetractMmPerMin = p.CenterRetractMmPerMin;
            f.mainRetractMmPerMin = p.MainRetractMmPerMin;
            f.centerReturnMmPerMin = p.CenterReturnMmPerMin;
            f.mainReturnMmPerMin = p.MainReturnMmPerMin;
            f.pathRetractMmPerMin = p.PathRetractMmPerMin;
            f.initialSlowMmPerMin = p.InitialSlowMmPerMin;
            f.manualReturnMmPerMin = p.ManualReturnMmPerMin;
            f.finalApproachDistanceMm = p.FinalApproachDistanceMm;
            f.pathDistanceMm = p.PathDistanceMm;
            f.initialSlowDistanceMm = p.InitialSlowDistanceMm;
        }
        for (unsigned bank = 0U; bank < 2U; ++bank) {
            if ((request.SelectionMask & (2U << bank)) == 0U) continue;
            for (unsigned axis = 0U; axis < 8U; ++axis) {
                const auto& source = bank == 0U ? request.Values.Discharge[axis] : request.Values.Flush[axis];
                auto& target = bank == 0U ? candidate.axisGains.discharge[axis] : candidate.axisGains.flush[axis];
                target.inheritCnc = source.InheritCnc != 0U;
                target.custom.Kp = source.Kp; target.custom.Ki = source.Ki;
                target.custom.Kd = source.Kd; target.custom.Kvff = source.Kvff;
            }
        }
        if (!EDM20::ValidateProcessProfile(candidate)) return false;
        out = candidate;
        return true;
    }

    // Bounded, owner-only duplicate cache. When a session is evicted, its old
    // successful command remains stale because every commit advances Generation.
    class RequestHistory
    {
        struct Entry { bool used = false; EDMProcessTuningRequest request{}; EDMProcessTuningAck ack{}; };
        Entry entries_[32]{};
        unsigned nextEviction_ = 0U;
    public:
        enum class Result { New, Duplicate, Reused };
        Result Inspect(const EDMProcessTuningRequest& request, EDMProcessTuningAck& previous) const noexcept
        {
            for (const auto& entry : entries_) {
                if (!entry.used) continue;
                if (entry.request.SessionId != request.SessionId) continue;
                if (request.RequestId > entry.request.RequestId) return Result::New;
                if (request.RequestId == entry.request.RequestId &&
                    std::memcmp(&request, &entry.request, sizeof(request)) == 0) {
                    previous = entry.ack; return Result::Duplicate;
                }
                return Result::Reused;
            }
            return Result::New;
        }
        void Remember(const EDMProcessTuningRequest& request, const EDMProcessTuningAck& ack) noexcept
        {
            Entry* target = nullptr;
            for (auto& entry : entries_) {
                if (entry.used && entry.request.SessionId == request.SessionId) { target = &entry; break; }
                if (!entry.used && target == nullptr) target = &entry;
            }
            if (target == nullptr) { target = &entries_[nextEviction_]; nextEviction_ = (nextEviction_ + 1U) % 32U; }
            target->used = true; target->request = request; target->ack = ack;
        }
        void Reset() noexcept { for (auto& entry : entries_) entry.used = false; nextEviction_ = 0U; }
    };
}
