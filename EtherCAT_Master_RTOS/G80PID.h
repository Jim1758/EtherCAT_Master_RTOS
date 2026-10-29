#pragma once
#include <cmath>

namespace G80
{
// G80-only D history. Ordinary G00/G01/G76 retain their existing PI law.
struct DerivativeState
{
    double previousError=0.0,filteredDerivative=0.0;
    bool initialized=false;
};
// Input/output units are the existing pulse-domain servo law. On failure the
// history and output remain unchanged; the RT caller must fault and force zero.
inline bool AddFilteredDerivative(double kd,double filterSeconds,double error,
    double periodSeconds,double piFeedforward,DerivativeState& state,double& output) noexcept
{
    if(!std::isfinite(kd) || kd<0.0 || !std::isfinite(filterSeconds) || filterSeconds<=0.0 ||
        !std::isfinite(error) || !std::isfinite(periodSeconds) || periodSeconds<=0.0 ||
        !std::isfinite(piFeedforward) || !std::isfinite(state.previousError) ||
        !std::isfinite(state.filteredDerivative)) return false;
    DerivativeState next=state;
    double result=piFeedforward;
    if(!state.initialized || kd==0.0) {
        // Exact Kd=0 compatibility, including signed zero, and no first-tick
        // derivative kick at a session's current physical following error.
        next.filteredDerivative=0.0;
    }
    else {
        const double difference=error-state.previousError;
        const double raw=difference/periodSeconds;
        const double denominator=filterSeconds+periodSeconds;
        const double alpha=periodSeconds/denominator;
        const double change=raw-state.filteredDerivative;
        if(!std::isfinite(difference) || !std::isfinite(raw) || !std::isfinite(denominator) ||
            !std::isfinite(alpha) || alpha<=0.0 || alpha>1.0 || !std::isfinite(change)) return false;
        next.filteredDerivative=state.filteredDerivative+alpha*change;
        const double derivative=kd*next.filteredDerivative;
        result=piFeedforward+derivative;
        if(!std::isfinite(next.filteredDerivative) || !std::isfinite(derivative) || !std::isfinite(result)) return false;
    }
    next.previousError=error;next.initialized=true;
    state=next;output=result;
    return true;
}
}
