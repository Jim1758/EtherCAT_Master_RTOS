#pragma once

#include <cmath>

// EDM93: the scalar PI law shared by the existing physical servo loop and
// the isolated six-axis CSV model. This helper owns no history or output.
// Validation, lag checks, caps, hardware direction and serialization belong
// to the caller. In particular the legacy caller retains its old behavior.
namespace EDM93PI
{
struct Gains
{
    double kp=0.0,ki=0.0,kd=0.0,kvff=0.0,maxIntegral=0.0;
};
struct Result
{
    double integralAcc=0.0,pTerm=0.0,iTerm=0.0,velocityPps=0.0;
};
inline bool ValidGains(const Gains& gains) noexcept
{
    return std::isfinite(gains.kp) && gains.kp>=0.0 &&
        std::isfinite(gains.ki) && gains.ki>=0.0 &&
        std::isfinite(gains.kd) && gains.kd>=0.0 &&
        std::isfinite(gains.kvff) && gains.kvff>=0.0 &&
        std::isfinite(gains.maxIntegral) && gains.maxIntegral>=0.0;
}
inline Result Evaluate(const Gains& gains,double integralBefore,double error,
    double feedforwardPps,double periodSeconds) noexcept
{
    Result result{};
    result.pTerm=error*gains.kp;
    result.integralAcc=integralBefore;
    result.integralAcc+=error*periodSeconds;
    if(result.integralAcc>gains.maxIntegral) result.integralAcc=gains.maxIntegral;
    if(result.integralAcc<-gains.maxIntegral) result.integralAcc=-gains.maxIntegral;
    result.iTerm=result.integralAcc*gains.ki;
    // The production controller currently disables the derivative term.
    // Retain Kd in the binding while preserving that exact control law.
    const double dTerm=0.0;
    result.velocityPps=(feedforwardPps*gains.kvff)+(result.pTerm+result.iTerm+dTerm);
    return result;
}
}
