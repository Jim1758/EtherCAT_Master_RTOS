#pragma once
#include <cstdint>
#include <cstddef>
class EtherCatMaster;
namespace DriverAlarm381 {
constexpr std::uint32_t Magic=0x31414c44u, Version=1u, AxisCapacity=8u;
constexpr const char* SharedMemoryName="OSCARMAX_DRIVER_ALARM_DIAG";
enum QueryState : std::uint32_t { NotSupported=0,NeverRead=1,Pending=2,Current=3,NoAlarm=4,Stale=5,Unavailable=6,ReadFailed=7,Deferred=8 };
enum ReadFailure : std::uint32_t { None=0,Wkc=1,Size=2,Abort=3,Timeout=4,ZeroDuringFault=5,CodeRange=6,IdentityMismatch=7,PdoUnavailable=8,ServiceUnavailable=9,MotionDeferred=10 };
enum RowFlags : std::uint32_t { Configured=1,PdoValid=2,StatuswordValid=4,Fault=8,Warning=16,NativeCurrent=32,HasLastNonzero=64,Supported=128 };
enum HeaderFlags : std::uint32_t { Ready=1,SourceCoherent=2,HeaderPdoValid=4,RuntimeReady=8,CommandQuiet=16 };
struct alignas(8) Header {
 std::uint32_t magic,version,bytes,sequence;
 std::uint64_t runtimeId,heartbeat,nowMs,sourcePdoTick;
 std::uint32_t axisCount,flags;
 std::int32_t lrwActual,lrwExpected,dcWkc;
 std::uint32_t reasonMask,consecutiveInvalid,latchedReasonMask;
 std::uint64_t communicationEpisode,lastInvalidTick;
 std::int32_t lastInvalidLrwActual,lastInvalidLrwExpected,lastInvalidDc;
 std::uint32_t queryCount,queryFailures,reserved[3];
};
struct alignas(8) AxisRow {
 std::uint32_t axisIndex,slavePosition,configuredAddress,vendorId,productCode,revision,statusword,flags;
 std::uint32_t nativeCode,lastKnownNativeCode,queryState;
 std::int32_t lastWkc;
 std::uint32_t abortCode,attemptCount,profileId,readFailureReason;
 std::uint64_t faultEpisode,lastGoodAtMs,lastFaultAtMs,lastKnownAtMs,lastKnownEpisode;
 std::uint32_t queryIndex,querySubIndex,querySize,reserved0;
 std::uint64_t identity,reserved[4];
};
struct alignas(8) Image { Header header; AxisRow rows[AxisCapacity]; };
static_assert(sizeof(Header)==128,"Driver alarm header ABI");
static_assert(sizeof(AxisRow)==160,"Driver alarm axis ABI");
static_assert(sizeof(Image)==1408,"Driver alarm image ABI");
static_assert(offsetof(AxisRow,lastKnownEpisode)==96,"Driver alarm episode ABI");
// The PDO owner calls CaptureRt only from its existing 100 Hz diagnostic capture.
// No EtherCAT transaction, allocation, clock query, lock or text output occurs there.
void CaptureRt(EtherCatMaster*,int,int,bool) noexcept;
void Initialize(EtherCatMaster*) noexcept;
void Shutdown() noexcept;
void Tick(EtherCatMaster*) noexcept;
bool OwnsCommand() noexcept;
// Called only at the engineering bridge's no-request/no-active/no-drain branch.
void TrySubmitBackground(EtherCatMaster*) noexcept;
}
