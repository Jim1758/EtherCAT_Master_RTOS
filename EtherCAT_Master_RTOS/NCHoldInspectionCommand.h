#pragma once
#include "NCHoldTelemetry.h"
#include <array>
#include <cstdint>
#include <cstring>

// EDM331 uses ONLY the previously unused 64 bytes in NC_Command.reserved.
// The packed legacy mailbox is not 32-bit aligned. The one-byte opcode is
// published LAST by the existing NC writer and cleared LAST by the NC owner;
// payload reads/writes use byte copies, never unaligned atomics or typed casts.
namespace NCHoldInspectionCommand {
constexpr unsigned Size=64U;
constexpr std::uint8_t Version=1U;
constexpr std::uint32_t Magic=0x474A484EU; // NHJG, little endian.
enum class Opcode:std::uint8_t { None=0U,InspectionJog=1U };
using Bytes=std::array<std::uint8_t,Size>;
struct Request {
    std::int32_t axis=-1,direction=0;
    std::uint64_t publisherGeneration=0ULL,parentSession=0ULL,holdSequence=0ULL,clientSequence=0ULL;
};
template<class T> inline void Put(Bytes& b,unsigned offset,T value) noexcept {
    std::memcpy(b.data()+offset,&value,sizeof(value));
}
template<class T> inline T Get(const Bytes& b,unsigned offset) noexcept {
    T value{};std::memcpy(&value,b.data()+offset,sizeof(value));return value;
}
inline bool Valid(const Request& r) noexcept {
    return r.publisherGeneration&&r.parentSession&&r.holdSequence&&r.clientSequence&&
        r.axis>=-1&&r.axis<=2&&r.direction>=-1&&r.direction<=1&&
        (r.axis>=0||r.direction==0);
}
inline bool Encode(const Request& r,Bytes& b) noexcept {
    b=Bytes{};if(!Valid(r))return false;
    b[0]=static_cast<std::uint8_t>(Opcode::InspectionJog);b[1]=Version;b[2]=Size;
    Put(b,4U,r.axis);Put(b,8U,r.direction);Put(b,16U,r.publisherGeneration);
    Put(b,24U,r.parentSession);Put(b,32U,r.holdSequence);Put(b,40U,r.clientSequence);Put(b,48U,Magic);
    return true;
}
inline bool Decode(const Bytes& b,Request& r) noexcept {
    r=Request{};
    if(b[0]!=static_cast<std::uint8_t>(Opcode::InspectionJog)||b[1]!=Version||b[2]!=Size||b[3]||
       Get<std::uint32_t>(b,12U)!=0U||Get<std::uint32_t>(b,48U)!=Magic)return false;
    for(unsigned i=52U;i<Size;++i)if(b[i])return false;
    r.axis=Get<std::int32_t>(b,4U);r.direction=Get<std::int32_t>(b,8U);
    r.publisherGeneration=Get<std::uint64_t>(b,16U);r.parentSession=Get<std::uint64_t>(b,24U);
    r.holdSequence=Get<std::uint64_t>(b,32U);r.clientSequence=Get<std::uint64_t>(b,40U);
    return Valid(r);
}
inline bool ScopeMatches(const Request& r,std::uint64_t generation,const NCHoldTelemetry::Image& current) noexcept {
    return Valid(r)&&generation&&r.publisherGeneration==generation&&r.parentSession==current.parentSession&&
        r.holdSequence==current.holdSequence;
}
// Single NC-owner state. Call only after matching the current native scope.
// Consume the sequence before attempting the command, including scoped STOP
// and requests discarded by a higher-priority control action. An older BEGIN
// or renewal can then never become a fresh lease after STOP in the same HOLD.
struct ReplayFence {
    std::uint64_t publisherGeneration=0ULL,parentSession=0ULL,holdSequence=0ULL,lastClientSequence=0ULL;
    bool AdmitFresh(const Request& r) noexcept {
        if(!Valid(r))return false;
        if(r.publisherGeneration!=publisherGeneration||r.parentSession!=parentSession||r.holdSequence!=holdSequence){
            publisherGeneration=r.publisherGeneration;parentSession=r.parentSession;holdSequence=r.holdSequence;
            lastClientSequence=0ULL;
        }
        if(r.clientSequence<=lastClientSequence)return false;
        lastClientSequence=r.clientSequence;return true;
    }
};
}
