#pragma once
#include "MotionExecutionContract.h"
#include "MotionQueueTailTransaction.h"
#include <array>
#include <cstdint>

enum class MotionG76AdmissionGate : std::uint8_t
{
    NONE=0U, PLAN_INPUT, OWNER_SCOPE, SOURCE_FRAME, RUNTIME_READY, CAPTURE_SCOPE,
    START_MCS, AXIS_MAPPING, AXIS_SCALE, NATIVE_BASELINE, TRAVEL_CONFIG,
    TRAVEL_START, AXIS_DYNAMICS, TRAVEL_ENDPOINT, POINT_CONVERSION,
    POINT_DYNAMICS, LINE_STAGE, QUEUE_CAPACITY, EPOCH_COMMIT, QUEUE_PUBLISH,
    CAPTURE_BIND, POST_ACCEPT_TUPLE
};
inline const char* MotionG76AdmissionGateName(MotionG76AdmissionGate gate) noexcept
{
    switch (gate) {
#define EDM171_G76_GATE(name) case MotionG76AdmissionGate::name: return #name;
    EDM171_G76_GATE(NONE) EDM171_G76_GATE(PLAN_INPUT) EDM171_G76_GATE(OWNER_SCOPE)
    EDM171_G76_GATE(SOURCE_FRAME) EDM171_G76_GATE(RUNTIME_READY) EDM171_G76_GATE(CAPTURE_SCOPE)
    EDM171_G76_GATE(START_MCS) EDM171_G76_GATE(AXIS_MAPPING) EDM171_G76_GATE(AXIS_SCALE)
    EDM171_G76_GATE(NATIVE_BASELINE) EDM171_G76_GATE(TRAVEL_CONFIG) EDM171_G76_GATE(TRAVEL_START)
    EDM171_G76_GATE(AXIS_DYNAMICS) EDM171_G76_GATE(TRAVEL_ENDPOINT) EDM171_G76_GATE(POINT_CONVERSION)
    EDM171_G76_GATE(POINT_DYNAMICS) EDM171_G76_GATE(LINE_STAGE) EDM171_G76_GATE(QUEUE_CAPACITY)
    EDM171_G76_GATE(EPOCH_COMMIT) EDM171_G76_GATE(QUEUE_PUBLISH) EDM171_G76_GATE(CAPTURE_BIND)
    EDM171_G76_GATE(POST_ACCEPT_TUPLE)
#undef EDM171_G76_GATE
    }
    return "UNKNOWN";
}

// Producer evidence for one bounded native G76 block. It is not an RT
// controller and grants no authority independently of the normal NC ledger.
struct MotionG76OrbitReceipt
{
    std::array<MotionExecutionIdentity,128U> identities{};
    MotionOwnerLease ownerLease{};
    std::uint64_t translationGeneration=0ULL;
    unsigned count=0U,axisMask=0U,validAxisMask=0U;
    std::array<double,8U> startMCS{},endMCS{},startPulse{},endPulse{},pulsePerUnit{};
    std::array<std::array<double,8U>,128U> legEndMCS{},legEndPulse{};
    MotionG76AdmissionGate rejectionGate=MotionG76AdmissionGate::NONE;
    int rejectionAxis=-1,rejectionSegment=-1;
    MotionRejectReason rejectionReason=MotionRejectReason::NONE;
    MotionQueueTailCommitReceipt tail{};
    bool valid=false,commandAccepted=false,captureBound=false,tailCommitted=false;
    void Clear() noexcept
    {
        for (auto& identity:identities) identity=MotionExecutionIdentity{};
        ownerLease=MotionOwnerLease{}; translationGeneration=0ULL;
        count=axisMask=validAxisMask=0U;
        startMCS.fill(0.0); endMCS.fill(0.0); startPulse.fill(0.0); endPulse.fill(0.0); pulsePerUnit.fill(0.0);
        for (auto& point:legEndMCS) point.fill(0.0);
        for (auto& point:legEndPulse) point.fill(0.0);
        rejectionGate=MotionG76AdmissionGate::NONE; rejectionAxis=rejectionSegment=-1;
        rejectionReason=MotionRejectReason::NONE; tail=MotionQueueTailCommitReceipt{};
        valid=commandAccepted=captureBound=tailCommitted=false;
    }
};
