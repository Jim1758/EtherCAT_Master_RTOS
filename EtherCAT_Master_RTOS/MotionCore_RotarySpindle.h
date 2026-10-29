#pragma once
#include "NCSpindleDispatch.h"
#include "MotionExecutionContract.h"
#include <array>
#include <cstdint>

// EDM289: coherent RT-owned status. Receipt readiness is based on physical
// feedback plus the exact serialized image accepted at the NIC boundary.
// It is not a drive acknowledgement or a claim that hardware has been tested.
struct MotionRotarySpindleFeedback
{
    NCSpindleDispatch::Authority authority{};
    std::array<NCSpindleDispatch::Receipt,NCRotarySpindle::Count> receipt{};
    std::array<double,8> handbackMcsUnit{};
    std::array<double,8> handbackPulse{};
    std::array<double,NCRotarySpindle::Count> actualRpm{},commandRpm{};
    std::uint64_t tick=0ULL;
    std::uint32_t axisMask=0U,faultCode=0U;
    bool initialized=false,active=false,allStopped=false,handbackReady=false,fault=false;
};

namespace MotionRotarySpindle
{
constexpr bool RuntimeConnected=true;
enum class Fault : std::uint32_t
{None,Source,Authority,Configuration,Positioning,Admission,Output,Feedback,Mailbox,Handback,Timeout};
}
