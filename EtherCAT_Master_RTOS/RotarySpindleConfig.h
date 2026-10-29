#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <type_traits>

namespace NCRotarySpindle {struct Config;}

// EDM287: independent file configuration. File I/O and parsing belong only to
// startup / a stopped configuration worker, never a 250 us servo callback.
// A valid file is data, not permission to spin. Runtime connection is separate.
namespace RotarySpindleFile
{
constexpr unsigned SpindleCount=3U,AxisCapacity=8U;
constexpr double MinimumRampSeconds=0.00025,MaximumRampSeconds=3600.0;
constexpr double MaximumConfiguredRpm=1000000.0; // format bound, NOT a machine rating

struct Spindle
{
    bool enabled=false;
    unsigned axisNumber=4U; // human-facing 1..8; never an index into AxisContext
    double maximumRpm=500.0;
    double accelTimeSeconds=2.0,decelTimeSeconds=2.0,stopTimeSeconds=1.0;
    double stoppedToleranceRpm=0.1,readyToleranceRpm=5.0;
};
struct Settings
{
    std::array<Spindle,SpindleCount> spindle{};
    Settings() noexcept {spindle[0].axisNumber=4U;spindle[1].axisNumber=5U;spindle[2].axisNumber=6U;}
};
enum class ErrorCode : std::uint8_t
{
    None,FileTooLarge,LineTooLong,InvalidByte,InvalidSyntax,UnknownKey,
    DuplicateKey,MissingKey,InvalidValue,InvalidSchema,AxisRange,DuplicateAxis,
    RpmRange,RampRange,ToleranceRange,AxisUnavailable,AxisNotRotary,
    AxisIdentity,AxisScale,MachineVelocityLimit,OpenFailed,ReadFailed,
    CreateDirectoryFailed,CreateFailed,WriteFailed,AllocationFailure
};
struct Error
{
    ErrorCode code=ErrorCode::None;
    unsigned line=0U,spindle=0U; // 1..3; 0 means global or no spindle
    char key[48]{};
};
enum class LoadStatus : std::uint8_t {Loaded,CreatedDisabled,Invalid,IoError};

// Axis facts must come from the installed, immutable configuration snapshot.
// AxisSlot equals the selected array index. For native degrees, scale is
// PPR/finalLead and unitsPerOutputRevolution is 360 (gearing already included).
struct AxisFacts
{
    unsigned axisSlot=0U;
    bool exists=false,rotary=false;
    double pulsesPerNative=0.0,nativeUnitsPerOutputRevolution=360.0;
    double maximumOutputPps=0.0; // proven drive/machine cap; never inferred from S
};
using RuntimeConfig=std::array<NCRotarySpindle::Config,SpindleCount>;

// Derived capability only; these are not additional parameter-file fields.
// maximumOutputPps is the intersection of the installed and signed PDO caps.
struct Capability
{
    double configuredRpm=0.0,axisRpm=0.0,pdoRpm=0.0,effectiveRpm=0.0;
    double pulsesPerOutputRevolution=0.0,maximumOutputPps=0.0;
    bool valid=false;
};

const char* DefaultPath() noexcept;
const char* DefaultText() noexcept;
const char* ErrorName(ErrorCode code) noexcept;
bool Validate(const Settings& settings,Error& error) noexcept;
// UTF-8 BOM, ASCII keys, #/;/ // comments, and all 25 unique keys required.
// On any failure output becomes disabled defaults; no partial settings survive.
bool ParseText(const std::string& text,Settings& output,Error& error) noexcept;
// Missing file only: create all parent directories and a disabled template,
// using exclusive create. Existing malformed/inaccessible files are preserved.
LoadStatus LoadOrCreateDefault(const std::string& path,Settings& output,Error& error) noexcept;
// Shared pure capability conversion for admission and NC diagnostics. It does
// not change an authored S command, ramp times, or tolerances. Failure clears
// output; error.spindle is zero because this helper has no logical-slot input.
bool ComputeCapability(const Spindle& settings,const AxisFacts& axis,
    Capability& output,Error& error) noexcept;
// Pure conversion. Effective maximum intersects configured/axis/PDO limits;
// callers still reject any requested S above that maximum. Slopes retain the
// configured MaxRPM/time definition. Disabled mappings need no installed axis.
bool BuildRuntime(const Settings& settings,const std::array<AxisFacts,AxisCapacity>& axes,
    RuntimeConfig& output,Error& error) noexcept;

static_assert(std::is_trivially_copyable<Settings>::value,"Spindle settings must remain RT-copyable data.");
static_assert(std::is_trivially_copyable<Error>::value,"Spindle diagnostics must not allocate.");
}
