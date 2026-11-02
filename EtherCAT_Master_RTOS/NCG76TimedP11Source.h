#pragma once

#include "NCG76SphereSource.h"
#include "NCMachiningTimePolicy.h"

// EDM345: source/provenance for explicitly timed G76 P11/P12/P13/P14/P15. No omitted
// timer word, numeric default, or shape-only result grants this common parent.
// R remains program radius and K remains the legacy contour angle in degrees.
// Native radius/travel/frame limits belong to the separate geometry builder.
namespace NCG76TimedP11Source
{
constexpr unsigned ModeP=11U;
inline bool SupportedMode(double mode) noexcept {return mode==11.0||mode==12.0||mode==13.0||mode==14.0||mode==15.0;}
enum class Status : unsigned char
{
    Ok=0U,InvalidSource,NonLiteral,UnsupportedBlock,InvalidScalar,
    InvalidProcessOption,UnresolvedModalB
};
struct Words
{
    NCBlock authored{};
    NCG76Sphere::Point3 authoredXYZ{};
    unsigned targetMask=0U;
    double radiusProgramMm=0.0,contourAngleDeg=0.0,feedMmMin=0.0;
    unsigned requestedB=0U;
    bool explicitB=false,inheritedB=false;
    NCG76SphereSource::ModalB inheritedBSource{};
    bool valid=false;
};
// Dispatch must still prove the complete authored source and current frame.
// In particular, absent D AND H does not acquire this implementation.
inline bool IsOptIn(const NCBlock& block) noexcept
{
    return block.hasG&&block.gCode==76&&block.has('P')&&SupportedMode(block.val('P'))&&
        (block.has(NCMachiningTimePolicy::TotalAddress)||block.has(NCMachiningTimePolicy::AdditionalAddress));
}
inline Status Decode(const NCParsedBlock& parsed,const NCBlock& resolved,
    const NCG76SphereSource::ModalB& inherited,Words& output) noexcept
{
    // Freeze aliases before clearing caller storage, including its authored
    // block or modal proof. Failures leave an empty, invalid source receipt.
    const NCBlock frozen=resolved;
    const NCG76SphereSource::ModalB modal=inherited;
    output=Words{};
    if(!IsOptIn(frozen))return Status::UnsupportedBlock;
    NCMachiningTime::Config timing{};
    if(!NCMachiningTimePolicy::Decode(parsed,frozen,timing))return Status::InvalidSource;
    NCBlock literal{};
    if(!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed,literal))return Status::NonLiteral;
    if(literal.hasG!=frozen.hasG||literal.gCode!=frozen.gCode||literal.gCount!=frozen.gCount||
       literal.mCount!=frozen.mCount||literal.isEmpty!=frozen.isEmpty||literal.isBlockSkip!=frozen.isBlockSkip||
       literal.isGoto!=frozen.isGoto||literal.gotoTarget!=frozen.gotoTarget)return Status::InvalidSource;
    for(char address='A';address<='Z';++address)if(literal.has(address)!=frozen.has(address)||
        literal.val(address)!=frozen.val(address)||std::signbit(literal.val(address))!=std::signbit(frozen.val(address)))
        return Status::InvalidSource;
    for(int i=0;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(literal.gCodes[i]!=frozen.gCodes[i])return Status::InvalidSource;
    for(int i=0;i<NC_MAX_M_CODES_PER_BLOCK;++i)if(literal.mCode[i]!=frozen.mCode[i])return Status::InvalidSource;
    if(!NCMachiningTimePolicy::ExactIntegerLiteral(parsed.gExpressions[0],76U)||
       !NCMachiningTimePolicy::ExactIntegerLiteral(parsed.expression('P'),static_cast<unsigned>(frozen.val('P'))))return Status::NonLiteral;
    if(!frozen.has('R')||!frozen.has('F')||
       !(frozen.has('X')||frozen.has('Y')||frozen.has('Z')))return Status::UnsupportedBlock;
    if(!(frozen.val('R')>0.0)||!(frozen.val('F')>0.0)||frozen.val('F')>30.0||
       (frozen.has('K')&&(frozen.val('K')< -180.0||frozen.val('K')>180.0)))return Status::InvalidScalar;
    const bool explicitB=frozen.has('B');
    if(explicitB&&((frozen.val('B')!=0.0&&frozen.val('B')!=2.0)||
       !NCMachiningTimePolicy::ExactIntegerLiteral(parsed.expression('B'),static_cast<unsigned>(frozen.val('B')))))
        return Status::InvalidProcessOption;
    if(!explicitB&&!NCG76SphereSource::ValidModalB(modal))return Status::UnresolvedModalB;
    if(!explicitB&&modal.value!=0U&&modal.value!=2U)return Status::InvalidProcessOption;
    Words candidate{};
    candidate.authored=frozen;
    for(unsigned axis=0U;axis<3U;++axis){
        const char address=static_cast<char>('X'+axis);
        if(frozen.has(address)){candidate.targetMask|=1U<<axis;candidate.authoredXYZ[axis]=frozen.val(address);}
    }
    candidate.radiusProgramMm=frozen.val('R');
    candidate.contourAngleDeg=frozen.has('K')?frozen.val('K'):0.0;
    candidate.feedMmMin=frozen.val('F');
    candidate.explicitB=explicitB;candidate.inheritedB=!explicitB;
    candidate.requestedB=explicitB?static_cast<unsigned>(frozen.val('B')):modal.value;
    if(candidate.inheritedB)candidate.inheritedBSource=modal;
    candidate.valid=true;output=candidate;return Status::Ok;
}
inline Status Decode(const NCParsedBlock& parsed,const NCG76SphereSource::ModalB& inherited,
    Words& output) noexcept
{
    NCBlock literal{};
    if(!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed,literal)){
        output=Words{};return Status::NonLiteral;
    }
    return Decode(parsed,literal,inherited,output);
}
inline Status Decode(const NCParsedBlock& parsed,Words& output) noexcept
{return Decode(parsed,NCG76SphereSource::ModalB{},output);}
}
