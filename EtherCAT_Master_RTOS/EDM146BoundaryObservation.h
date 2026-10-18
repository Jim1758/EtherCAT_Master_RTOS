#pragma once
#include <cstdint>

// NC-thread observation only. A retry never grants motion authority: the
// caller must rerun its complete live boundary proof with fresh snapshots.
namespace EDM146
{
    enum class Gate : std::uint8_t
    {
        None, BoundaryState, PathProof, HistoryProof, TimingRead, HistoryRead,
        PhysicalRead, GapRead, BoundaryRead, PhysicalProof, SourcePayload,
        SourceSkew, GapProof, GapPayload, GapSkew, Selection, Translation,
        BoundaryProof, AxisProof, RowBinding, Geometry, ModeOrigin, Policy
    };
    struct Failure
    {
        Gate gate = Gate::None;
        std::uint64_t tickA = 0ULL, tickB = 0ULL, nowA = 0ULL, nowB = 0ULL;
    };
    inline bool Fail(Failure* out, Gate gate, std::uint64_t tickA = 0ULL,
        std::uint64_t tickB = 0ULL, std::uint64_t nowA = 0ULL, std::uint64_t nowB = 0ULL) noexcept
    {
        if (out) { out->gate = gate; out->tickA = tickA; out->tickB = tickB;
            out->nowA = nowA; out->nowB = nowB; }
        return false;
    }
    inline bool Retryable(Gate gate) noexcept
    {
        return gate == Gate::TimingRead || gate == Gate::HistoryRead ||
            gate == Gate::PhysicalRead || gate == Gate::GapRead ||
            gate == Gate::BoundaryRead || gate == Gate::SourceSkew || gate == Gate::GapSkew;
    }
    inline const char* Name(Gate gate) noexcept
    {
        switch (gate)
        {
        case Gate::None: return "NONE";
        case Gate::BoundaryState: return "BOUNDARY_STATE";
        case Gate::PathProof: return "PATH_PROOF";
        case Gate::HistoryProof: return "HISTORY_PROOF";
        case Gate::TimingRead: return "TIMING_READ";
        case Gate::HistoryRead: return "HISTORY_READ";
        case Gate::PhysicalRead: return "PHYSICAL_READ";
        case Gate::GapRead: return "GAP_READ";
        case Gate::BoundaryRead: return "BOUNDARY_READ";
        case Gate::PhysicalProof: return "PHYSICAL_PROOF";
        case Gate::SourcePayload: return "SOURCE_PAYLOAD";
        case Gate::SourceSkew: return "SOURCE_SKEW";
        case Gate::GapProof: return "GAP_PROOF";
        case Gate::GapPayload: return "GAP_PAYLOAD";
        case Gate::GapSkew: return "GAP_SKEW";
        case Gate::Selection: return "SELECTION";
        case Gate::Translation: return "TRANSLATION";
        case Gate::BoundaryProof: return "BOUNDARY_PROOF";
        case Gate::AxisProof: return "AXIS_PROOF";
        case Gate::RowBinding: return "ROW_BINDING";
        case Gate::Geometry: return "GEOMETRY";
        case Gate::ModeOrigin: return "MODE_ORIGIN";
        case Gate::Policy: return "POLICY";
        }
        return "UNKNOWN";
    }
    template<class Check, class Report>
    bool FreshBoundary(Check check, Report report) noexcept
    {
        Failure first{}, last{};
        for (unsigned attempt = 1U; attempt <= 3U; ++attempt)
        {
            Failure observed{};
            if (check(observed))
            {
                if (attempt > 1U) report(true, attempt, first, last);
                return true;
            }
            if (attempt == 1U) first = observed;
            last = observed;
            if (!Retryable(observed.gate) || attempt == 3U)
            { report(false, attempt, first, last); return false; }
        }
        return false;
    }
}
