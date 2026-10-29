#pragma once
#include "G80Cycle.h"
#include "MotionExecutionContract.h"
#include "EDMGapPhysicalSource.h"
#include "MotionServoHandoffDiagnostic.h"
#include <cmath>

// One RT writer publishes this value. NC may consume only a matching session.
struct MotionG80Feedback
{
    G80::Feedback cycle{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease owner{};
    G80::Point6 endpointMCS{};
    // Native diagnostic sample paired with cycle.tick (XYZ mm, CUV degrees;
    // velocityNative is commanded native units/s, never inferred actual speed).
    G80::Point6 commandMCS{}, velocityNative{}, actualMCS{};
    std::uint64_t session=0;
    double voltageV=0.0;
    std::uint32_t runtimeFault=0;
    // Source is evidence, not a physical-test claim: Unknown=0, Physical=1, SIM=2.
    std::uint32_t source=0,simScenario=0;
    std::uint64_t inputSequence=0;
    double simProjectedDistance=0.0,simPlaneDistance=0.0;
    bool simContact=false;
    bool accepted=false,started=false,terminal=false,completed=false,endpointCommitted=false;
};

// Pure predicates also used by the live RT integration. None grants ownership;
// the caller must retain the exact execution/owner and active group checks.
namespace MotionG80RuntimeProof
{
inline bool SameInputScope(std::uint64_t sampleSession,const MotionExecutionIdentity& sample,
    std::uint64_t requestSession,const MotionExecutionIdentity& command) noexcept
{
    return sampleSession!=0ULL && sampleSession==requestSession && sample.IsAssigned() && command.IsAssigned() &&
        sample.epoch==command.epoch && sample.segmentId==command.segmentId &&
        sample.sourceBlockId==command.sourceBlockId && sample.source==command.source;
}
inline double DecodeMotorPulse(std::int32_t raw,std::uint32_t previousRaw,
    double previousUnwrapped,bool driveReverse,bool axisReverse,double homeOffset) noexcept
{
    const auto delta=static_cast<std::int32_t>(static_cast<std::uint32_t>(raw)-previousRaw);
    double pulse=previousUnwrapped+static_cast<double>(delta);
    if(driveReverse) pulse=-pulse;
    if(axisReverse) pulse=-pulse;
    return pulse-homeOffset;
}
inline bool FreshPhysicalInput(const EDM63::Receipt& receipt,std::uint64_t nowUs,
    std::uint64_t frequency,std::uint64_t lastSequence,bool clockValid) noexcept
{
    const auto& sample=receipt.sample;
    return receipt.Usable() && receipt.conversion.inputStatus==EDMGapInput::InputStatus::Valid &&
        receipt.conversion.boardVoltageValid && receipt.conversion.gapInRange &&
        sample.sequence!=0ULL && sample.sequence>lastSequence && clockValid && frequency!=0ULL &&
        sample.qpcFrequency==frequency && sample.capturedAtUs<=nowUs &&
        nowUs-sample.capturedAtUs<=EDM63::MaxReceiptAgeUs;
}
inline bool ExactZeroSend(const MotionServoHandoffSample& sample,MotionExecutionEpoch epoch,
    const MotionOwnerLease& owner,std::uint32_t axisMask,std::uint64_t tick) noexcept
{
    if(!axisMask || (axisMask&~63U)!=0U || !tick || !sample.attempted || !sample.apiAccepted ||
        !sample.finalChecked || sample.scrubbed || sample.epoch!=epoch ||
        sample.owner!=static_cast<std::uint32_t>(owner.owner) || sample.ownerGeneration!=owner.generation ||
        sample.sourceTick!=tick || (sample.slotMask&axisMask)!=axisMask ||
        (sample.observedMask&axisMask)!=axisMask || (sample.sourceInputMask&axisMask)!=axisMask) return false;
    for(unsigned i=0U;i<6U;++i) if(axisMask&(1U<<i))
        if(sample.proposedVelocity[i]!=0 || sample.copiedVelocity[i]!=0 ||
            (sample.sourceStatusWord[i]&0x006FU)!=0x0027U || sample.sourceMode[i]!=9) return false;
    return true;
}
}
