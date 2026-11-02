#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

// EDM291 display-only byte contract in SHM_NC_Status.reserved[23].
// The owner publishes Sequence separately with its aligned seqlock writer.
// No packed scalar pointer casts or host-endian stores are used here.
namespace RotarySpindleTelemetry
{
constexpr std::size_t ImageSize=92U;
constexpr std::size_t SequenceOffset=3U;
constexpr std::size_t VersionOffset=7U;
constexpr std::size_t RecordsOffset=8U;
constexpr std::size_t RecordSize=28U;
constexpr unsigned SpindleCount=3U;
constexpr std::uint8_t Version=1U;
using Image=std::array<std::uint8_t,ImageSize>;

enum class State : std::uint8_t
{
    Disabled=0U,Idle=1U,Starting=2U,Forward=3U,Reverse=4U,Stopping=5U,
    Held=6U,Fault=7U,Positioning=8U,Homing=9U,Unavailable=10U
};
enum Flag : std::uint8_t
{
    Enabled=1U,RpmValid=2U,PositionValid=4U,Homed=8U,
    ClaimActive=16U,PLCActive=32U,Fault=64U
};

struct Record
{
    std::uint8_t axisNumber=0U;
    State state=State::Disabled;
    std::uint8_t flags=0U;
    double targetRpm=0.0,commandRpm=0.0,actualRpm=0.0;
    double angleDegrees=0.0,continuousMcsDegrees=0.0;
};

inline void Initialize(Image& image) noexcept
{
    image.fill(0U);
    image[0]='R';image[1]='S';image[2]='P';image[VersionOffset]=Version;
}

namespace Detail
{
inline void Write32(Image& image,std::size_t offset,std::uint32_t value) noexcept
{
    for(unsigned byte=0U;byte<4U;++byte)
        image[offset+byte]=static_cast<std::uint8_t>(value>>(byte*8U));
}
inline void Write64(Image& image,std::size_t offset,std::uint64_t value) noexcept
{
    for(unsigned byte=0U;byte<8U;++byte)
        image[offset+byte]=static_cast<std::uint8_t>(value>>(byte*8U));
}
inline bool MilliRpm(double value,std::int32_t& output) noexcept
{
    output=0;
    if(!std::isfinite(value))return false;
    const double rounded=std::round(value*1000.0);
    if(!std::isfinite(rounded)||
        rounded<static_cast<double>((std::numeric_limits<std::int32_t>::min)())||
        rounded>static_cast<double>((std::numeric_limits<std::int32_t>::max)()))return false;
    output=static_cast<std::int32_t>(rounded);return true;
}
inline bool Position(double angle,double continuous,
    std::int32_t& angleFixed,std::int64_t& continuousFixed) noexcept
{
    angleFixed=0;continuousFixed=0;
    if(!std::isfinite(angle)||!std::isfinite(continuous))return false;
    double wrapped=std::fmod(angle,360.0);
    if(wrapped<0.0)wrapped+=360.0;
    double angleRounded=std::round(wrapped*10000.0);
    if(angleRounded>=3600000.0)angleRounded=0.0;
    const double rounded=std::round(continuous*10000.0);
    // INT64_MAX rounds upward when represented as double. Use the exclusive
    // positive 2^63 bound to reject that value before any integer conversion.
    constexpr double low=-9223372036854775808.0,high=9223372036854775808.0;
    if(!std::isfinite(rounded)||rounded<low||rounded>=high)return false;
    angleFixed=static_cast<std::int32_t>(angleRounded);
    continuousFixed=static_cast<std::int64_t>(rounded);return true;
}
}

inline void Encode(Image& image,unsigned spindle,const Record& record) noexcept
{
    if(spindle>=SpindleCount)return;
    const std::size_t offset=RecordsOffset+RecordSize*spindle;
    auto flags=static_cast<std::uint8_t>(record.flags&127U);
    std::int32_t target=0,command=0,actual=0,angle=0;
    std::int64_t continuous=0;
    const bool rpm=(flags&RpmValid)!=0U&&Detail::MilliRpm(record.targetRpm,target)&&
        Detail::MilliRpm(record.commandRpm,command)&&Detail::MilliRpm(record.actualRpm,actual);
    if(!rpm){flags=static_cast<std::uint8_t>(flags&~RpmValid);target=command=actual=0;}
    const bool position=(flags&PositionValid)!=0U&&
        Detail::Position(record.angleDegrees,record.continuousMcsDegrees,angle,continuous);
    if(!position){
        flags=static_cast<std::uint8_t>(flags&~(PositionValid|Homed));angle=0;continuous=0;
    }
    image[offset]=record.axisNumber;image[offset+1U]=static_cast<std::uint8_t>(record.state);
    image[offset+2U]=flags;image[offset+3U]=0U;
    Detail::Write32(image,offset+4U,static_cast<std::uint32_t>(target));
    Detail::Write32(image,offset+8U,static_cast<std::uint32_t>(command));
    Detail::Write32(image,offset+12U,static_cast<std::uint32_t>(actual));
    Detail::Write32(image,offset+16U,static_cast<std::uint32_t>(angle));
    Detail::Write64(image,offset+20U,static_cast<std::uint64_t>(continuous));
}

static_assert(sizeof(Image)==ImageSize,"Spindle display image ABI");
static_assert(SequenceOffset+sizeof(std::uint32_t)==VersionOffset,"Spindle sequence ABI");
static_assert(RecordsOffset+RecordSize*SpindleCount==ImageSize,"Spindle record ABI");
}
