#pragma once
#include "NCSpindleDispatch.h"
#include "MotionExecutionContract.h"
#include <array>
#include <cstdint>

// EDM289 FIX3: observation only. These fields never authorize an output or
// substitute for a Receipt. Counts are session totals and saturate at UINT32_MAX.
struct MotionRotarySpindleProgress
{
    std::uint64_t observationTick=0ULL,requestedSequence=0ULL,appliedSequence=0ULL,qualifiedSequence=0ULL;
    std::uint64_t deadlineTick=0ULL,finalSendTick=0ULL,finalSendRequestSequence=0ULL;
    std::uint64_t maximumAbsRawDelta=0ULL;
    std::int32_t rawDelta=0;
    std::uint32_t zeroSamples=0U,maximumZeroSamples=0U,currentReasonMask=0U,lastResetReasonMask=0U;
    std::uint32_t zeroResetCount=0U,invalidSourceSamples=0U,missingSendSamples=0U,overStopSamples=0U,requestMismatchSamples=0U;
    double actualRpm=0.0,priorCommandRpm=0.0,stoppedToleranceRpm=0.0,maximumAbsActualRpm=0.0;
    // actualRpm above remains the raw 250 us derivative for diagnosis.
    double filteredRpm=0.0,stopMotionRpm=0.0;
    std::uint32_t speedSamples=0U;
    bool speedReady=false;
    bool sourceValid=false,rawDeltaValid=false,exactPriorSend=false,zeroSent=false,packetApplied=false;
};
struct MotionRotarySpindleFirstFault
{
    std::array<MotionRotarySpindleProgress,NCRotarySpindle::Count> progress{};
    std::uint64_t tick=0ULL,beginTick=0ULL;
    std::uint32_t code=0U;
    bool valid=false,initialized=false;
};

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
    // EDM291: one RT-owned display observation, in output-shaft units.
    std::array<double,NCRotarySpindle::Count> requestedRpm{},actualMcsDegrees{};
    std::array<NCRotarySpindle::Phase,NCRotarySpindle::Count> phase{};
    std::uint32_t positionValidMask=0U,homedMask=0U;
    std::array<MotionRotarySpindleProgress,NCRotarySpindle::Count> progress{};
    MotionRotarySpindleFirstFault firstFault{};
    std::uint64_t tick=0ULL;
    std::uint32_t axisMask=0U,faultCode=0U;
    // EDM290: bits 0..2 identify logical spindles, not axis slots. An enabled
    // spindle stays ON until fresh physical input and 200 exact final-zero
    // observations prove it stopped. This indication is never motion authority.
    std::uint32_t rotationActiveMask=0U;
    bool initialized=false,active=false,allStopped=false,handbackReady=false,fault=false;
};

namespace MotionRotarySpindle
{
constexpr bool RuntimeConnected=true;
enum class Fault : std::uint32_t
{None,Source,Authority,Configuration,Positioning,Admission,Output,Feedback,Mailbox,Handback,Timeout};
enum ProgressReason : std::uint32_t
{
    InvalidSource=1U,NoExactSend=2U,NonAdjacentSend=4U,SendScopeChanged=8U,
    NonzeroPriorCommand=16U,AboveStopTolerance=32U,NoAppliedPacket=64U,
    SendRequestMismatch=128U,ConfigurationChanged=256U,RawDeltaUnavailable=512U,
    SpeedWindowIncomplete=1024U,InstantaneousOverspeed=2048U
};
}
static_assert(sizeof(MotionRotarySpindleFeedback)<=4096U,"Spindle diagnostics must fit the existing bounded feedback channel.");
