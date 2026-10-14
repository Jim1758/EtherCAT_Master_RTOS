#pragma once

#include "EDMToolHeadKinematics.h"
#include <cstdint>
#include <cstring>

// EDM87 fixed-value, boot-loaded reference TOOL HEAD definitions. A profile
// declares geometry; its presence is not machine calibration or motion permit.
// No file access, allocation, or mutable global catalog belongs in this header.
namespace EDM87
{
constexpr unsigned MaximumProfiles=8U;
constexpr std::uint64_t MaximumRevision=UINT64_C(4294967295);
struct HeadProfile
{
    unsigned id=0U;
    std::uint64_t revision=0U;
    EDM86::HeadChain chain{};
    EDM86::Vector3 toolTipMm{};
    EDM86::Vector3 directionUnit{{0.0,0.0,1.0}};
};
struct Catalog
{
    std::uint64_t revision=0U;
    unsigned count=0U;
    std::array<HeadProfile,MaximumProfiles> profiles{};
};
struct Selection
{
    std::uint64_t catalogRevision=0U;
    HeadProfile profile{};
    std::uint64_t fingerprint=0U;
};
namespace ProfileDetail
{
inline bool EqualFields(const HeadProfile& a,const HeadProfile& b) noexcept
{
    return a.id==b.id && a.revision==b.revision &&
        EDM86::ChainsEqual(a.chain,b.chain) && a.toolTipMm==b.toolTipMm &&
        a.directionUnit==b.directionUnit;
}
inline bool EmptyProfile(const HeadProfile& profile) noexcept
{ return EqualFields(profile,HeadProfile{}); }
inline void HashWord(std::uint64_t value,std::uint64_t& hash) noexcept
{
    // Explicit low-byte-first order, independent of host endianness/padding.
    for(unsigned byte=0U;byte<8U;++byte)
    { hash^=value&UINT64_C(255); hash*=UINT64_C(1099511628211); value>>=8U; }
}
inline void HashDouble(double value,std::uint64_t& hash) noexcept
{
    static_assert(sizeof(double)==sizeof(std::uint64_t) &&
        std::numeric_limits<double>::is_iec559,"EDM87 fingerprints require IEEE binary64");
    if(value==0.0) value=0.0; // +0 and -0 are the same semantic value.
    std::uint64_t bits=0U;
    std::memcpy(&bits,&value,sizeof(bits));
    HashWord(bits,hash);
}
}
inline bool ValidProfile(const HeadProfile& profile) noexcept
{
    if(profile.id==0U || profile.id>MaximumProfiles || profile.revision==0U ||
        profile.revision>MaximumRevision || !EDM86::ValidChain(profile.chain) ||
        !EDM86::Detail::Unit(profile.directionUnit)) return false;
    EDM86::Vector3 center{}; double radius=0.0;
    if(!EDM86::OffsetEnvelope(profile.chain,profile.toolTipMm,center,radius)) return false;
    for(unsigned axis=0U;axis<3U;++axis)
        if(!EDM81::Finite(center[axis]-radius) || !EDM81::Finite(center[axis]+radius)) return false;
    return true;
}
inline bool ProfilesEqual(const HeadProfile& a,const HeadProfile& b) noexcept
{ return ValidProfile(a) && ValidProfile(b) && ProfileDetail::EqualFields(a,b); }
inline bool ValidCatalog(const Catalog& catalog) noexcept
{
    if(catalog.revision==0U || catalog.revision>MaximumRevision ||
        catalog.count==0U || catalog.count>MaximumProfiles) return false;
    unsigned ids=0U;
    for(unsigned index=0U;index<MaximumProfiles;++index)
    {
        const auto& profile=catalog.profiles[index];
        if(index>=catalog.count)
        { if(!ProfileDetail::EmptyProfile(profile)) return false; continue; }
        if(!ValidProfile(profile)) return false;
        const unsigned bit=1U<<(profile.id-1U);
        if(ids&bit) return false;
        ids|=bit;
    }
    return true;
}
inline const HeadProfile* FindProfile(const Catalog& catalog,unsigned id) noexcept
{
    if(id==0U || id>MaximumProfiles || !ValidCatalog(catalog)) return nullptr;
    for(unsigned index=0U;index<catalog.count;++index)
        if(catalog.profiles[index].id==id) return &catalog.profiles[index];
    return nullptr;
}
inline std::uint64_t Fingerprint(const HeadProfile& profile) noexcept
{
    if(!ValidProfile(profile)) return 0U;
    std::uint64_t hash=UINT64_C(14695981039346656037);
    ProfileDetail::HashWord(UINT64_C(0x45444D3837500001),hash); // EDM87 profile schema 1.
    ProfileDetail::HashWord(profile.id,hash);
    ProfileDetail::HashWord(profile.revision,hash);
    for(const auto& joint:profile.chain.joints)
    {
        ProfileDetail::HashWord(joint.channel,hash);
        for(const auto value:joint.axis) ProfileDetail::HashDouble(value,hash);
        for(const auto value:joint.parentToPivotMm) ProfileDetail::HashDouble(value,hash);
        ProfileDetail::HashWord(joint.sign==1?1U:2U,hash);
        ProfileDetail::HashDouble(joint.zeroAngleDeg,hash);
    }
    for(const auto value:profile.toolTipMm) ProfileDetail::HashDouble(value,hash);
    for(const auto value:profile.directionUnit) ProfileDetail::HashDouble(value,hash);
    // Fingerprint is trace integrity only, not a collision-proof security hash.
    return hash==0U?UINT64_C(1):hash;
}
inline bool IsEmptySelection(const Selection& selection) noexcept
{
    return selection.catalogRevision==0U && selection.fingerprint==0U &&
        ProfileDetail::EmptyProfile(selection.profile);
}
inline bool ValidSelection(const Selection& selection) noexcept
{
    return selection.catalogRevision!=0U && selection.catalogRevision<=MaximumRevision &&
        ValidProfile(selection.profile) && selection.fingerprint!=0U &&
        selection.fingerprint==Fingerprint(selection.profile);
}
static_assert(std::is_trivially_copyable<HeadProfile>::value &&
    std::is_trivially_copyable<Selection>::value &&
    std::is_trivially_copyable<Catalog>::value,"EDM87 profiles are fixed-value copies");
static_assert(sizeof(Catalog)<=4096U,"EDM87 catalog exceeds fixed storage budget");
}
