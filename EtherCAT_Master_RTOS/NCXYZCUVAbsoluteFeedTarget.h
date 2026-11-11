#pragma once

#include "NCRotaryFeedTarget.h"
#include "NCAxisIdentitySnapshot.h"
#include <array>
#include <cmath>

// EDM384: G90 native targets for XYZ and optional mixed-type C/U/V.
// Linear/continuous axes use raw native absolutes; positional rotary axes alone
// use the established shortest/tie policy. Fixed axes retain their accepted
// pulse origin. Source offsets, travel and ownership remain caller duties.
// Inputs/output are disjoint; failure clears the entire output value.
struct NCXYZCUVAbsoluteFeedTarget
{
    std::array<double, 6U> endMCS{}, endPulse{}, deltaNative{};
    bool valid = false;
};

inline bool TryResolveNCXYZCUVAbsoluteFeedTarget(
    const NCAxisIdentitySnapshot& identity,
    const std::array<double,6U>& startMCS,
    const std::array<double,6U>& startPulse,
    const std::array<double,6U>& requestedMCS,
    const std::array<double,6U>& pulsePerUnit,
    const std::array<bool,3U>& shortestPath,
    const std::array<double,3U>& rotaryModulo,
    NCXYZCUVAbsoluteFeedTarget& output,std::uint32_t authoredAxisMask=0xffffffffU) noexcept
{
    output=NCXYZCUVAbsoluteFeedTarget{};
    if(!IsNCAxisIdentitySnapshotValid(identity)||identity.exists[6]||identity.exists[7])return false;
    std::uint32_t presentMask=0U;
    for(unsigned axis=0U;axis<6U;++axis)if(identity.exists[axis])presentMask|=1U<<axis;
    const std::uint32_t authored=authoredAxisMask==0xffffffffU?presentMask:authoredAxisMask;
    if((authored&~presentMask)!=0U)return false;
    NCXYZCUVAbsoluteFeedTarget candidate{};
    for(unsigned axis=0U;axis<6U;++axis){
        if(axis<3U&&(identity.exists[axis]!=1U||identity.axisType[axis]!=0U))return false;
        if(identity.exists[axis]&&identity.address[axis]!="XYZCUV"[axis])return false;
        if(!std::isfinite(startMCS[axis])||!std::isfinite(startPulse[axis])||!std::isfinite(requestedMCS[axis]))return false;
        if(!identity.exists[axis]){
            if(std::memcmp(&requestedMCS[axis],&startMCS[axis],sizeof(double))!=0)return false;
            candidate.endMCS[axis]=startMCS[axis];candidate.endPulse[axis]=startPulse[axis];continue;
        }
        if(!std::isfinite(pulsePerUnit[axis])||pulsePerUnit[axis]<=0.0)return false;
        if(!(authored&(1U<<axis))){
            if(std::memcmp(&requestedMCS[axis],&startMCS[axis],sizeof(double))!=0)return false;
            candidate.endMCS[axis]=startMCS[axis];candidate.endPulse[axis]=startPulse[axis];continue;
        }
        if(identity.axisType[axis]==1U){
            NCRotaryAbsoluteFeedTarget resolved{};
            if(axis<3U||!TryResolveNCRotaryAbsoluteFeedTarget(startMCS[axis],startPulse[axis],requestedMCS[axis],
                pulsePerUnit[axis],shortestPath[axis-3U],rotaryModulo[axis-3U],resolved))return false;
            candidate.endMCS[axis]=resolved.endMCS;candidate.endPulse[axis]=resolved.endPulse;
            candidate.deltaNative[axis]=resolved.sweepDeg;continue;
        }
        if(std::memcmp(&requestedMCS[axis],&startMCS[axis],sizeof(double))==0){
            // An unchanged raw coordinate must not turn a tolerated frozen
            // native/pulse baseline rounding difference into a correction.
            candidate.endMCS[axis]=startMCS[axis];candidate.endPulse[axis]=startPulse[axis];continue;
        }
        // Linear and continuous rotary coordinates are raw native absolutes.
        // A continuous axis never acquires positional rotary wrap/tie policy.
        const double targetPulse=requestedMCS[axis]*pulsePerUnit[axis];
        if(!IsNCRotaryAbsolutePulseResolutionValid(startPulse[axis])||
           !IsNCRotaryAbsolutePulseResolutionValid(targetPulse)||(requestedMCS[axis]!=0.0&&targetPulse==0.0))return false;
        const double nativeDelta=requestedMCS[axis]-startMCS[axis];
        const double pulseDelta=targetPulse-startPulse[axis];
        if(!std::isfinite(nativeDelta)||!std::isfinite(pulseDelta)||
           ((nativeDelta==0.0)!=(pulseDelta==0.0))||
           (nativeDelta!=0.0&&(nativeDelta<0.0)!=(pulseDelta<0.0)))return false;
        candidate.endMCS[axis]=pulseDelta==0.0?startMCS[axis]:requestedMCS[axis];
        candidate.endPulse[axis]=pulseDelta==0.0?startPulse[axis]:targetPulse;
        candidate.deltaNative[axis]=candidate.endMCS[axis]-startMCS[axis];
    }
    candidate.valid=true;output=candidate;return true;
}

// Source compatibility for the original full-six positional-rotary contract.
inline bool TryResolveNCXYZCUVAbsoluteFeedTarget(
    const std::array<double,6U>& startMCS,
    const std::array<double,6U>& startPulse,
    const std::array<double,6U>& requestedMCS,
    const std::array<double,6U>& pulsePerUnit,
    const std::array<bool,3U>& shortestPath,
    const std::array<double,3U>& rotaryModulo,
    NCXYZCUVAbsoluteFeedTarget& output) noexcept
{
    NCAxisIdentitySnapshot identity{};identity.bound=1U;
    for(unsigned axis=0U;axis<6U;++axis){identity.exists[axis]=1U;identity.axisType[axis]=axis<3U?0U:1U;
        identity.nativeUnit[axis]=axis<3U?1U:2U;identity.physicalIndexPlusOne[axis]=static_cast<std::uint8_t>(axis+1U);
        identity.address[axis]="XYZCUV"[axis];}
    return TryResolveNCXYZCUVAbsoluteFeedTarget(identity,startMCS,startPulse,requestedMCS,pulsePerUnit,shortestPath,rotaryModulo,output);
}
