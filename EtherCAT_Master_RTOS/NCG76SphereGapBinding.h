#pragma once

#include "NCG76SphereGapProcess.h"
#include "EDM130WorkTimeBinding.h"
#include "EDMSpecialGapGainBinding.h"
#include <array>
#include <cmath>
#include <cstring>

// NC-owner recipe capture for SIMULATED P50 processing. Catalog pointers never
// cross to RT. EDM298 also proposes the bounded current-leg SIM feed mode;
// the motion owner independently admits its frozen scope and physical limits.
// No legacy G76 profile, 25-child recipe or discharge permit is fabricated.
namespace NCG76SphereGapBinding
{
enum class Reason : unsigned char
{ None, ProcessUnavailable, ProcessInvalid, RecipeUnavailable, FieldRejected,
  EngineeringRange, CaptureChanged, InvalidConfig };
inline const char* ReasonName(Reason reason) noexcept
{
    switch(reason){
    case Reason::None:return "NONE";case Reason::ProcessUnavailable:return "PROCESS_UNAVAILABLE";
    case Reason::ProcessInvalid:return "PROCESS_INVALID";case Reason::RecipeUnavailable:return "RECIPE_UNAVAILABLE";
    case Reason::FieldRejected:return "FIELD_REJECTED";case Reason::EngineeringRange:return "ENGINEERING_RANGE";
    case Reason::CaptureChanged:return "CAPTURE_CHANGED";case Reason::InvalidConfig:return "INVALID_CONFIG";
    default:return "UNKNOWN";}
}
enum class ApproachReason : unsigned char
{ Disabled,Enabled,ZeroDistance,MissingField,Unconfirmed,Unconfigured,InvalidField };
inline const char* ApproachReasonName(ApproachReason reason) noexcept
{
    switch(reason){
    case ApproachReason::Disabled:return "DISABLED";case ApproachReason::Enabled:return "ENABLED";
    case ApproachReason::ZeroDistance:return "ZERO_DISTANCE";case ApproachReason::MissingField:return "MISSING_FIELD";
    case ApproachReason::Unconfirmed:return "UNCONFIRMED";case ApproachReason::Unconfigured:return "UNCONFIGURED";
    case ApproachReason::InvalidField:return "INVALID_FIELD";default:return "UNKNOWN";}
}
enum class DeepFlushReason : unsigned char
{ Disabled,Enabled,ZeroInterval,MissingField,Unconfirmed,Unconfigured,InvalidField };
inline const char* DeepFlushReasonName(DeepFlushReason reason) noexcept
{
    switch(reason){
    case DeepFlushReason::Disabled:return "DISABLED";case DeepFlushReason::Enabled:return "ENABLED";
    case DeepFlushReason::ZeroInterval:return "ZERO_INTERVAL";case DeepFlushReason::MissingField:return "MISSING_FIELD";
    case DeepFlushReason::Unconfirmed:return "UNCONFIRMED";case DeepFlushReason::Unconfigured:return "UNCONFIGURED";
    case DeepFlushReason::InvalidField:return "INVALID_FIELD";default:return "UNKNOWN";}
}
enum class E11UnitSource : unsigned char
{ Unavailable,Catalog,ExplicitActual,OwnerConfirmedLegacy };
inline const char* E11UnitSourceName(E11UnitSource source) noexcept
{
    switch(source){
    case E11UnitSource::Unavailable:return "UNAVAILABLE";case E11UnitSource::Catalog:return "CATALOG";
    case E11UnitSource::ExplicitActual:return "EXPLICIT_ACTUAL";
    case E11UnitSource::OwnerConfirmedLegacy:return "OWNER_CONFIRMED_LEGACY";default:return "UNKNOWN";}
}
// NC-only identity data, including unconfirmed values without assigning them
// physical units. No catalog vector or pointer is copied into the RT config.
struct ApproachFieldSnapshot
{
    EDMRecipe::Value minimum=0LL,maximum=0LL,actual=0LL;
    std::uint16_t id=0U;
    std::uint8_t decimals=0U;
    char unit[24]{};
    bool enabled=false,confirmed=false,configured=false,explicitActual=false;
};
struct Binding
{
    NCG76SphereGapProcess::Config config{};
    // NC-only provenance: keep the file curve distinct from the effective
    // E-selected curve; even dormant file edits revoke an existing parent.
    EDMSpecialGapGainBinding::Binding specialGain{};
    EDMGapServo::Profile sourceServo{};
    EDMRecipe::Summary selection{};
    const EDMRecipe::Catalog* catalog=nullptr;
    std::array<EDMRecipe::Value,5U> values{};
    std::array<std::uint16_t,5U> fields{};
    std::array<EDMRecipe::Value,2U> approachValues{};
    std::array<std::uint16_t,2U> approachFields{};
    std::array<ApproachFieldSnapshot,2U> approachSnapshots{};
    ApproachReason approachReason=ApproachReason::Disabled;
    EDM58::BindingError approachError=EDM58::BindingError::None;
    std::uint16_t approachFailedFieldId=0U;
    // EDM305: initial outbound E19/E20 remain observable even when disabled,
    // so a recipe or definition edit revokes the same frozen parent.
    std::array<EDMRecipe::Value,2U> initialSlowValues{};
    std::array<std::uint16_t,2U> initialSlowFields{};
    std::array<ApproachFieldSnapshot,2U> initialSlowSnapshots{};
    ApproachReason initialSlowReason=ApproachReason::Disabled;
    EDM58::BindingError initialSlowError=EDM58::BindingError::None;
    std::uint16_t initialSlowFailedFieldId=0U;
    std::array<EDMRecipe::Value,2U> deepFlushValues{};
    std::array<std::uint16_t,2U> deepFlushFields{};
    std::array<ApproachFieldSnapshot,2U> deepFlushSnapshots{};
    DeepFlushReason deepFlushReason=DeepFlushReason::Disabled;
    EDM58::BindingError deepFlushError=EDM58::BindingError::None;
    std::uint16_t deepFlushFailedFieldId=0U;
    // EDM303: P50 alone accepts the machine owner's confirmed E11 percent
    // semantics for the exact legacy definition. This does not edit catalogs
    // or change any other G76 consumer. E13 raw zero only disables approach.
    double resolvedE11Percent=0.0;
    E11UnitSource e11UnitSource=E11UnitSource::Unavailable;
    bool distanceZeroSentinel=false;
    std::uint32_t processRevision=0U,machineProfileId=0U;
    EDM58::BindingError error=EDM58::BindingError::NotReady;
    std::uint16_t failedFieldId=0U;
    Reason reason=Reason::ProcessUnavailable;
    bool ready=false;
    // NC-only capture provenance. A later mutation of the RT config mode
    // cannot change which engineering envelope this binding was captured for.
    bool parameterFeeds=false;
    // EDM339: source-scoped P50 role selection, separate from Line/Arc feeds.
    bool orbitFeeds=false;
    EDMRecipe::Value orbitValue=0LL;
    std::uint16_t orbitField=0U;
};
inline bool SameDouble(double a,double b) noexcept
{return std::memcmp(&a,&b,sizeof(double))==0;}
inline bool SameFlushProfile(const EDM20::FlushProfile& a,const EDM20::FlushProfile& b) noexcept
{
    return SameDouble(a.finalApproachMmPerMin,b.finalApproachMmPerMin)&&
        SameDouble(a.centerRetractMmPerMin,b.centerRetractMmPerMin)&&SameDouble(a.mainRetractMmPerMin,b.mainRetractMmPerMin)&&
        SameDouble(a.centerReturnMmPerMin,b.centerReturnMmPerMin)&&SameDouble(a.mainReturnMmPerMin,b.mainReturnMmPerMin)&&
        SameDouble(a.pathRetractMmPerMin,b.pathRetractMmPerMin)&&SameDouble(a.initialSlowMmPerMin,b.initialSlowMmPerMin)&&
        SameDouble(a.manualReturnMmPerMin,b.manualReturnMmPerMin)&&SameDouble(a.finalApproachDistanceMm,b.finalApproachDistanceMm)&&
        SameDouble(a.pathDistanceMm,b.pathDistanceMm)&&SameDouble(a.initialSlowDistanceMm,b.initialSlowDistanceMm);
}
inline bool SameApproachSnapshots(const Binding& a,const Binding& b) noexcept
{
    for(unsigned i=0U;i<2U;++i){const auto& x=a.approachSnapshots[i];const auto& y=b.approachSnapshots[i];
        if(x.minimum!=y.minimum||x.maximum!=y.maximum||x.actual!=y.actual||x.id!=y.id||x.decimals!=y.decimals||
            x.enabled!=y.enabled||x.confirmed!=y.confirmed||x.configured!=y.configured||x.explicitActual!=y.explicitActual||
            std::memcmp(x.unit,y.unit,sizeof(x.unit))!=0)return false;
    }
    return true;
}
inline bool CaptureApproach(const EDMRecipe::Controller& controller,const EDMRecipe::Catalog& catalog,Binding& out) noexcept
{
    const char* semantics[]={"FinalApproachSpeed","DecelerationDistance"};const char* units[]={"%","mm"};
    const EDMRecipe::Value low[]={10000000LL,0LL},high[]={100000000LL,100000000LL};
    bool ready=true;
    for(unsigned i=0U;i<2U;++i){
        auto& snapshot=out.approachSnapshots[i];
        const EDMRecipe::FieldDefinition* match=nullptr;
        for(const auto& field:catalog.fields){
            if(!EDM58::BindingDetail::LabelEquals(field.semanticKey,sizeof(field.semanticKey),semantics[i]))continue;
            if(match){out.approachReason=ApproachReason::InvalidField;out.approachError=EDM58::BindingError::DuplicateSemantic;
                out.approachFailedFieldId=field.id;return false;}match=&field;
        }
        if(match){
            snapshot.id=match->id;snapshot.minimum=match->minimum;snapshot.maximum=match->maximum;
            snapshot.decimals=match->decimals;snapshot.enabled=match->enabled;snapshot.confirmed=match->unitConfirmed;
            std::memcpy(snapshot.unit,match->unit,sizeof(snapshot.unit));
            snapshot.explicitActual=controller.HasExplicitActualSinceSelection(match->id);
            EDMRecipe::ParameterSnapshot value{};
            if(controller.ReadField(match->id,value)&&value.fieldId==match->id){
                snapshot.configured=value.configured;snapshot.actual=value.actualValue;
                out.approachValues[i]=value.actualValue;
            }
            out.approachFields[i]=match->id;
        }
        EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
        bool read=EDM137Units::ReadExplicit(controller,catalog,semantics[i],units[i],value,field,semantic);
        E11UnitSource e11Source=read?(match&&match->unitConfirmed?E11UnitSource::Catalog:E11UnitSource::ExplicitActual):E11UnitSource::Unavailable;
        if(!read&&semantic.error==EDM58::BindingError::UnitUnconfirmed&&match&&
            EDM137Units::LegacyExplicitDefinition(*match,semantics[i],units[i])){
            EDMRecipe::ParameterSnapshot actual{};
            const bool configured=controller.ReadField(match->id,actual)&&actual.fieldId==match->id&&actual.configured&&
                (actual.rowKind==EDMRecipe::CellKind::Direct||actual.rowKind==EDMRecipe::CellKind::Stage);
            // The owner's E11 confirmation applies to actual persisted values
            // too. The separate E13 exception grants no nonzero-mm meaning.
            if(configured&&(i==0U||actual.actualValue==0LL)){
                value=actual.actualValue;field=match->id;read=true;
                if(i==0U)e11Source=E11UnitSource::OwnerConfirmedLegacy;
                else out.distanceZeroSentinel=true;
            }
        }
        if(!read){
            ApproachReason reason=ApproachReason::InvalidField;
            switch(semantic.error){
            case EDM58::BindingError::MissingSemantic:case EDM58::BindingError::FieldDisabled:
                reason=ApproachReason::MissingField;break;
            case EDM58::BindingError::UnitUnconfirmed:reason=ApproachReason::Unconfirmed;break;
            case EDM58::BindingError::Unconfigured:reason=ApproachReason::Unconfigured;break;
            default:break;
            }
            if(ready||reason==ApproachReason::InvalidField){out.approachReason=reason;out.approachError=semantic.error;
                out.approachFailedFieldId=semantic.failedFieldId;}
            if(reason==ApproachReason::InvalidField)return false;
            ready=false;continue;
        }
        if(value<low[i]||value>high[i]||!match||!EDMRecipe::ValueMatchesPrecision(value,match->decimals)){
            out.approachReason=ApproachReason::InvalidField;out.approachError=EDM58::BindingError::EngineeringRange;
            out.approachFailedFieldId=field;return false;
        }
        out.approachValues[i]=value;out.approachFields[i]=field;
        if(i==0U){out.resolvedE11Percent=static_cast<double>(value)/EDMRecipe::Scale;out.e11UnitSource=e11Source;}
    }
    if(!ready)return true;
    out.approachReason=out.approachValues[1]==0LL?ApproachReason::ZeroDistance:ApproachReason::Enabled;
    if(out.config.flushMotionEnabled&&out.approachValues[1]>0LL){
        out.config.finalApproachPercent=static_cast<double>(out.approachValues[0])/EDMRecipe::Scale;
        out.config.finalApproachDistanceMm=static_cast<double>(out.approachValues[1])/EDMRecipe::Scale;
        out.config.finalApproachEnabled=true;
    }else if(!out.config.flushMotionEnabled)out.approachReason=ApproachReason::Disabled;
    return true;
}
inline bool SameInitialSlowSnapshots(const Binding& a,const Binding& b) noexcept
{
    for(unsigned i=0U;i<2U;++i){const auto& x=a.initialSlowSnapshots[i];const auto& y=b.initialSlowSnapshots[i];
        if(x.minimum!=y.minimum||x.maximum!=y.maximum||x.actual!=y.actual||x.id!=y.id||x.decimals!=y.decimals||
            x.enabled!=y.enabled||x.confirmed!=y.confirmed||x.configured!=y.configured||x.explicitActual!=y.explicitActual||
            std::memcmp(x.unit,y.unit,sizeof(x.unit))!=0)return false;
    }
    return true;
}
inline bool CaptureInitialSlow(const EDMRecipe::Controller& controller,const EDMRecipe::Catalog& catalog,Binding& out) noexcept
{
    const char* semantics[]={"LargeAreaMachiningSpeed","LargeAreaFlushDistance"};const char* units[]={"%","mm"};
    const EDMRecipe::Value low[]={1000000LL,0LL},high[]={100000000LL,900000000LL};
    bool ready=true;
    for(unsigned i=0U;i<2U;++i){
        auto& snapshot=out.initialSlowSnapshots[i];
        const EDMRecipe::FieldDefinition* match=nullptr;
        for(const auto& field:catalog.fields){
            if(!EDM58::BindingDetail::LabelEquals(field.semanticKey,sizeof(field.semanticKey),semantics[i]))continue;
            if(match){out.initialSlowReason=ApproachReason::InvalidField;out.initialSlowError=EDM58::BindingError::DuplicateSemantic;
                out.initialSlowFailedFieldId=field.id;return false;}match=&field;
        }
        if(match){
            snapshot.id=match->id;snapshot.minimum=match->minimum;snapshot.maximum=match->maximum;
            snapshot.decimals=match->decimals;snapshot.enabled=match->enabled;snapshot.confirmed=match->unitConfirmed;
            std::memcpy(snapshot.unit,match->unit,sizeof(snapshot.unit));
            snapshot.explicitActual=controller.HasExplicitActualSinceSelection(match->id);
            EDMRecipe::ParameterSnapshot actual{};
            if(controller.ReadField(match->id,actual)&&actual.fieldId==match->id){
                snapshot.configured=actual.configured;snapshot.actual=actual.actualValue;
                out.initialSlowValues[i]=actual.actualValue;
            }
            out.initialSlowFields[i]=match->id;
        }
        EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
        const bool read=EDM137Units::ReadExplicit(controller,catalog,semantics[i],units[i],value,field,semantic);
        if(!read){
            ApproachReason reason=ApproachReason::InvalidField;
            switch(semantic.error){
            case EDM58::BindingError::MissingSemantic:case EDM58::BindingError::FieldDisabled:
                reason=ApproachReason::MissingField;break;
            case EDM58::BindingError::UnitUnconfirmed:reason=ApproachReason::Unconfirmed;break;
            case EDM58::BindingError::Unconfigured:reason=ApproachReason::Unconfigured;break;
            default:break;
            }
            if(ready||reason==ApproachReason::InvalidField){out.initialSlowReason=reason;out.initialSlowError=semantic.error;
                out.initialSlowFailedFieldId=semantic.failedFieldId;}
            if(reason==ApproachReason::InvalidField)return false;
            ready=false;continue;
        }
        if(value<low[i]||value>high[i]||!match||!EDMRecipe::ValueMatchesPrecision(value,match->decimals)){
            out.initialSlowReason=ApproachReason::InvalidField;out.initialSlowError=EDM58::BindingError::EngineeringRange;
            out.initialSlowFailedFieldId=field;return false;
        }
        out.initialSlowValues[i]=value;out.initialSlowFields[i]=field;
    }
    if(!ready)return true;
    out.initialSlowReason=out.initialSlowValues[1]==0LL?ApproachReason::ZeroDistance:ApproachReason::Enabled;
    if(out.config.flushMotionEnabled&&out.initialSlowValues[1]>0LL){
        out.config.initialSlowPercent=static_cast<double>(out.initialSlowValues[0])/EDMRecipe::Scale;
        out.config.initialSlowDistanceMm=static_cast<double>(out.initialSlowValues[1])/EDMRecipe::Scale;
        out.config.initialSlowEnabled=true;
    }else if(!out.config.flushMotionEnabled)out.initialSlowReason=ApproachReason::Disabled;
    return true;
}
inline bool SameDeepFlushSnapshots(const Binding& a,const Binding& b) noexcept
{
    for(unsigned i=0U;i<2U;++i){const auto& x=a.deepFlushSnapshots[i];const auto& y=b.deepFlushSnapshots[i];
        if(x.minimum!=y.minimum||x.maximum!=y.maximum||x.actual!=y.actual||x.id!=y.id||x.decimals!=y.decimals||
            x.enabled!=y.enabled||x.confirmed!=y.confirmed||x.configured!=y.configured||x.explicitActual!=y.explicitActual||
            std::memcmp(x.unit,y.unit,sizeof(x.unit))!=0)return false;
    }
    return true;
}
inline bool CaptureDeepFlush(const EDMRecipe::Controller& controller,const EDMRecipe::Catalog& catalog,Binding& out) noexcept
{
    const char* semantics[]={"DeepFlushCycleInterval","DeepFlushHeightMultiplier"};const char* units[]={"cycle","x"};
    bool ready=true;
    for(unsigned i=0U;i<2U;++i){
        auto& snapshot=out.deepFlushSnapshots[i];const EDMRecipe::FieldDefinition* match=nullptr;
        for(const auto& field:catalog.fields){
            if(!EDM58::BindingDetail::LabelEquals(field.semanticKey,sizeof(field.semanticKey),semantics[i]))continue;
            if(match){out.deepFlushReason=DeepFlushReason::InvalidField;out.deepFlushError=EDM58::BindingError::DuplicateSemantic;
                out.deepFlushFailedFieldId=field.id;return false;}match=&field;
        }
        if(match){
            snapshot.id=match->id;snapshot.minimum=match->minimum;snapshot.maximum=match->maximum;
            snapshot.decimals=match->decimals;snapshot.enabled=match->enabled;snapshot.confirmed=match->unitConfirmed;
            std::memcpy(snapshot.unit,match->unit,sizeof(snapshot.unit));snapshot.explicitActual=controller.HasExplicitActualSinceSelection(match->id);
            EDMRecipe::ParameterSnapshot actual{};
            if(controller.ReadField(match->id,actual)&&actual.fieldId==match->id){
                snapshot.configured=actual.configured;snapshot.actual=actual.actualValue;out.deepFlushValues[i]=actual.actualValue;
            }
            out.deepFlushFields[i]=match->id;
        }
        EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
        const bool read=EDM58::BindingDetail::Read(controller,catalog,semantics[i],units[i],value,field,semantic);
        if(!read){
            DeepFlushReason reason=DeepFlushReason::InvalidField;
            switch(semantic.error){
            case EDM58::BindingError::MissingSemantic:case EDM58::BindingError::FieldDisabled:
                reason=DeepFlushReason::MissingField;break;
            case EDM58::BindingError::UnitUnconfirmed:reason=DeepFlushReason::Unconfirmed;break;
            case EDM58::BindingError::Unconfigured:reason=DeepFlushReason::Unconfigured;break;
            default:break;
            }
            if(ready||reason==DeepFlushReason::InvalidField){out.deepFlushReason=reason;out.deepFlushError=semantic.error;
                out.deepFlushFailedFieldId=semantic.failedFieldId;}
            if(reason==DeepFlushReason::InvalidField)return false;
            ready=false;continue;
        }
        if(value<0LL||value>20000000LL||(!i&&value%EDMRecipe::Scale!=0LL)||!match||
            !EDMRecipe::ValueMatchesPrecision(value,match->decimals)){
            out.deepFlushReason=DeepFlushReason::InvalidField;out.deepFlushError=EDM58::BindingError::EngineeringRange;
            out.deepFlushFailedFieldId=field;return false;
        }
        out.deepFlushValues[i]=value;out.deepFlushFields[i]=field;
    }
    if(!ready)return true;
    if(out.deepFlushValues[0]>0LL&&out.deepFlushValues[1]==0LL){
        out.deepFlushReason=DeepFlushReason::InvalidField;out.deepFlushError=EDM58::BindingError::EngineeringRange;
        out.deepFlushFailedFieldId=out.deepFlushFields[1];return false;
    }
    if(!out.config.flushMotionEnabled){out.deepFlushReason=DeepFlushReason::Disabled;return true;}
    if(out.deepFlushValues[0]==0LL){out.deepFlushReason=DeepFlushReason::ZeroInterval;return true;}
    const auto heightProduct=static_cast<std::uint64_t>(out.values[1])*static_cast<std::uint64_t>(out.deepFlushValues[1]);
    const double multiplier=static_cast<double>(out.deepFlushValues[1])/EDMRecipe::Scale;
    const double height=static_cast<double>(heightProduct)/1000000000000.0;double resolved=0.0;
    const auto maximumProduct=out.config.parameterFeeds?10000000000000ULL:2000000000000ULL;
    if(heightProduct==0ULL||heightProduct>maximumProduct||
        !NCG76SphereDeepFlush::ResolveHeight(out.config.jumpHeightMm,multiplier,resolved,out.config.parameterFeeds?10.0:2.0)||height!=resolved){
        out.deepFlushReason=DeepFlushReason::InvalidField;out.deepFlushError=EDM58::BindingError::EngineeringRange;
        out.deepFlushFailedFieldId=out.deepFlushFields[1];return false;
    }
    out.config.deepFlushCycleInterval=static_cast<std::uint32_t>(out.deepFlushValues[0]/EDMRecipe::Scale);
    out.config.deepFlushHeightMultiplier=multiplier;out.config.deepFlushHeightMm=height;out.config.deepFlushEnabled=true;
    out.deepFlushReason=DeepFlushReason::Enabled;return true;
}
// Profile padding is deliberately excluded from identity comparisons.
inline bool SameServoProfile(const EDMGapServo::Profile& a,const EDMGapServo::Profile& b) noexcept
{
    if(a.revision!=b.revision||a.limitsConfirmed!=b.limitsConfirmed||
       a.shortIdleEnabled!=b.shortIdleEnabled||a.shortMachiningEnabled!=b.shortMachiningEnabled||
       a.shortEnterMs!=b.shortEnterMs||a.shortExitMs!=b.shortExitMs)return false;
    for(unsigned i=0U;i<3U;++i)
        if(!SameDouble(a.positiveGain[i],b.positiveGain[i])||!SameDouble(a.negativeGain[i],b.negativeGain[i]))return false;
    for(unsigned i=0U;i<2U;++i)
        if(!SameDouble(a.positiveBreakV[i],b.positiveBreakV[i])||!SameDouble(a.negativeBreakV[i],b.negativeBreakV[i]))return false;
    return SameDouble(a.cuttingScaleMmPerVoltMin,b.cuttingScaleMmPerVoltMin)&&
        SameDouble(a.maxFeedMmPerMin,b.maxFeedMmPerMin)&&SameDouble(a.maxRetreatMmPerMin,b.maxRetreatMmPerMin)&&
        SameDouble(a.deadbandV,b.deadbandV)&&SameDouble(a.shortIdleV,b.shortIdleV)&&
        SameDouble(a.shortMachiningV,b.shortMachiningV)&&SameDouble(a.shortHysteresisV,b.shortHysteresisV)&&
        SameDouble(a.shortRetreatMmPerMin,b.shortRetreatMmPerMin);
}

inline Binding Capture(const EDMRecipe::Controller& controller,const EDM20::ProcessProfile& process,
    std::uint64_t tuningGeneration,bool processReady,bool parameterFeeds=false,bool orbitFeeds=false) noexcept
{
    Binding out{};out.parameterFeeds=parameterFeeds;out.orbitFeeds=orbitFeeds;
    if(!processReady)return out;
    if(!tuningGeneration||!EDM20::ValidateProcessProfile(process)||(parameterFeeds&&orbitFeeds)){
        out.reason=Reason::ProcessInvalid;out.error=EDM58::BindingError::InvalidRecipe;return out;
    }
    const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const auto servo=process.servo;const auto flushProfile=process.flush;
    const double orbitScale=process.orbitScaleMmPerVoltMin;
    const auto revision=process.revision,profile=process.machineProfileId;
    const auto timed=EDM130::BindWorkTime(controller,profile);
    if(!timed.ready){out.reason=Reason::RecipeUnavailable;out.error=timed.error;
        out.failedFieldId=timed.failedFieldId;return out;}
    out.fields[0]=timed.recipe.fieldId;out.values[0]=static_cast<EDMRecipe::Value>(timed.recipe.workTimeUs);
    const char* semantics[]={"JumpHeight","JumpSpeedOverride","MachiningServoOverride","GapVoltageSetpoint"};
    const char* units[]={"mm","%","%","V"};
    // These are architecture metadata bounds, not a released flush envelope.
    // E9 is the confirmed reference voltage and admits the actual 15 V recipe.
    const EDMRecipe::Value low[]={0LL,50000000LL,1000000LL,15000000LL};
    const EDMRecipe::Value high[]={parameterFeeds?10000000LL:2000000LL,100000000LL,100000000LL,200000000LL};
    for(unsigned i=0U;i<4U;++i){
        EDM58::Binding semantic{};
        if(!catalog||!EDM58::BindingDetail::Read(controller,*catalog,semantics[i],units[i],
            out.values[i+1U],out.fields[i+1U],semantic)){
            out.reason=Reason::FieldRejected;out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;
        }
        if(out.values[i+1U]<low[i]||out.values[i+1U]>high[i]){
            out.reason=Reason::EngineeringRange;out.error=EDM58::BindingError::EngineeringRange;
            out.failedFieldId=out.fields[i+1U];return out;
        }
    }
    if(orbitFeeds){
        EDM58::Binding semantic{};
        if(!catalog||!EDM58::BindingDetail::Read(controller,*catalog,"OrbitSpeedOverride","%",
            out.orbitValue,out.orbitField,semantic)){
            out.reason=Reason::FieldRejected;out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;
        }
        // Existing catalogs legitimately store zero. Once this role is
        // enabled it must reject before motion rather than stall silently.
        if(out.orbitValue<=0LL||out.orbitValue>100000000LL){
            out.reason=Reason::EngineeringRange;out.error=EDM58::BindingError::EngineeringRange;
            out.failedFieldId=out.orbitField;return out;
        }
    }
    out.specialGain=EDMSpecialGapGainBinding::Capture(controller,profile);
    if(!out.specialGain.ready){out.reason=Reason::FieldRejected;out.error=EDM58::BindingError::InvalidRecipe;
        out.failedFieldId=out.specialGain.failedFieldId;return out;}
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current())||
       !EDM20::ValidateProcessProfile(process)||process.revision!=revision||process.machineProfileId!=profile||
       !SameServoProfile(servo,process.servo)||!SameFlushProfile(flushProfile,process.flush)||
       (orbitFeeds&&!SameDouble(orbitScale,process.orbitScaleMmPerVoltMin))){
        out.reason=Reason::CaptureChanged;out.error=EDM58::BindingError::CaptureChanged;return out;
    }
    out.sourceServo=servo;out.config.curve=servo;
    if(!EDMSpecialGapGainBinding::Apply(out.specialGain,out.config.curve)){
        out.reason=Reason::FieldRejected;out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.config.workTimeUs=timed.recipe.workTimeUs;
    out.config.recipeGeneration=before.generation;out.config.tuningGeneration=tuningGeneration;
    out.config.jumpHeightMm=static_cast<double>(out.values[1])/EDMRecipe::Scale;
    out.config.jumpSpeedPercent=static_cast<double>(out.values[2])/EDMRecipe::Scale;
    out.config.machiningOverridePercent=static_cast<double>(out.values[3])/EDMRecipe::Scale;
    out.config.referenceV=static_cast<double>(out.values[4])/EDMRecipe::Scale;
    out.config.enabled=true;out.config.flushMotionEnabled=out.config.jumpHeightMm>0.0;
    out.config.parameterFeeds=parameterFeeds;
    if(orbitFeeds){
        out.config.orbitScaleMmPerVoltMin=orbitScale;
        out.config.orbitOverridePercent=static_cast<double>(out.orbitValue)/EDMRecipe::Scale;
        out.config.orbitFeedsEnabled=true;
    }
    // EDM298: this P50 binding releases only the bounded software-simulation
    // feed lane. Actual GAP input and physical discharge remain unreleased.
    out.config.signedMotionEnabled=true;
    out.config.flushProfile=flushProfile;out.config.independentFeedsEnabled=out.config.flushMotionEnabled;
    if(!catalog||!CaptureApproach(controller,*catalog,out)){
        out.config=NCG76SphereGapProcess::Config{};
        out.reason=Reason::FieldRejected;out.error=out.approachError;out.failedFieldId=out.approachFailedFieldId;return out;
    }
    if(!CaptureInitialSlow(controller,*catalog,out)){
        out.config=NCG76SphereGapProcess::Config{};
        out.reason=Reason::FieldRejected;out.error=out.initialSlowError;out.failedFieldId=out.initialSlowFailedFieldId;return out;
    }
    if(!CaptureDeepFlush(controller,*catalog,out)){
        out.config=NCG76SphereGapProcess::Config{};
        out.reason=Reason::FieldRejected;out.error=out.deepFlushError;out.failedFieldId=out.deepFlushFailedFieldId;return out;
    }
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current())||
       !EDM20::ValidateProcessProfile(process)||process.revision!=revision||process.machineProfileId!=profile||
       !SameServoProfile(servo,process.servo)||!SameFlushProfile(flushProfile,process.flush)||
       !EDMSpecialGapGainBinding::IsCurrent(out.specialGain,controller,profile)||
       (orbitFeeds&&!SameDouble(orbitScale,process.orbitScaleMmPerVoltMin))){
        out.config=NCG76SphereGapProcess::Config{};
        out.reason=Reason::CaptureChanged;out.error=EDM58::BindingError::CaptureChanged;return out;
    }
    if(!NCG76SphereGapProcess::Valid(out.config)){
        out.config=NCG76SphereGapProcess::Config{};out.reason=Reason::InvalidConfig;
        out.error=EDM58::BindingError::InvalidRecipe;return out;
    }
    out.selection=before;out.catalog=catalog;out.processRevision=revision;out.machineProfileId=profile;
    out.error=EDM58::BindingError::None;out.reason=Reason::None;out.ready=true;return out;
}

// Single NC owner: current values and definition identity are re-read, so even
// unsupported in-place edits cannot revive a stale frozen simulated binding.
inline bool IsCurrent(const Binding& b,const EDMRecipe::Controller& controller,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,bool processReady) noexcept
{
    if(!b.ready||!b.config.enabled||!processReady||b.parameterFeeds!=b.config.parameterFeeds||
       b.orbitFeeds!=b.config.orbitFeedsEnabled||controller.GetCatalog()!=b.catalog||
       tuningGeneration!=b.config.tuningGeneration||process.revision!=b.processRevision||
       process.machineProfileId!=b.machineProfileId||!SameServoProfile(process.servo,b.sourceServo)||
       !SameFlushProfile(process.flush,b.config.flushProfile)||
       (b.orbitFeeds&&!SameDouble(process.orbitScaleMmPerVoltMin,b.config.orbitScaleMmPerVoltMin))||
       !EDM58::BindingDetail::SameSelection(b.selection,controller.Current()))return false;
    const auto current=Capture(controller,process,tuningGeneration,processReady,b.parameterFeeds,b.orbitFeeds);
    return current.ready&&current.parameterFeeds==b.parameterFeeds&&current.config.parameterFeeds==b.config.parameterFeeds&&
        EDMSpecialGapGainBinding::Same(current.specialGain,b.specialGain)&&SameServoProfile(current.sourceServo,b.sourceServo)&&
        current.orbitFeeds==b.orbitFeeds&&current.config.orbitFeedsEnabled==b.config.orbitFeedsEnabled&&
        current.orbitValue==b.orbitValue&&current.orbitField==b.orbitField&&
        SameDouble(current.config.orbitScaleMmPerVoltMin,b.config.orbitScaleMmPerVoltMin)&&
        SameDouble(current.config.orbitOverridePercent,b.config.orbitOverridePercent)&&
        current.catalog==b.catalog&&current.fields==b.fields&&current.values==b.values&&
        current.approachValues==b.approachValues&&current.approachFields==b.approachFields&&SameApproachSnapshots(current,b)&&
        current.approachReason==b.approachReason&&current.approachError==b.approachError&&
        current.approachFailedFieldId==b.approachFailedFieldId&&
        SameDouble(current.resolvedE11Percent,b.resolvedE11Percent)&&current.e11UnitSource==b.e11UnitSource&&
        current.distanceZeroSentinel==b.distanceZeroSentinel&&
        current.initialSlowValues==b.initialSlowValues&&current.initialSlowFields==b.initialSlowFields&&SameInitialSlowSnapshots(current,b)&&
        current.initialSlowReason==b.initialSlowReason&&current.initialSlowError==b.initialSlowError&&
        current.initialSlowFailedFieldId==b.initialSlowFailedFieldId&&
        current.deepFlushValues==b.deepFlushValues&&current.deepFlushFields==b.deepFlushFields&&SameDeepFlushSnapshots(current,b)&&
        current.deepFlushReason==b.deepFlushReason&&current.deepFlushError==b.deepFlushError&&
        current.deepFlushFailedFieldId==b.deepFlushFailedFieldId&&
        current.config.workTimeUs==b.config.workTimeUs&&
        current.config.recipeGeneration==b.config.recipeGeneration&&
        current.config.tuningGeneration==b.config.tuningGeneration&&
        current.config.flushMotionEnabled==b.config.flushMotionEnabled&&
        current.config.signedMotionEnabled==b.config.signedMotionEnabled&&
        current.config.independentFeedsEnabled==b.config.independentFeedsEnabled&&
        current.config.finalApproachEnabled==b.config.finalApproachEnabled&&
        SameDouble(current.config.finalApproachPercent,b.config.finalApproachPercent)&&
        SameDouble(current.config.finalApproachDistanceMm,b.config.finalApproachDistanceMm)&&
        current.config.initialSlowEnabled==b.config.initialSlowEnabled&&
        SameDouble(current.config.initialSlowPercent,b.config.initialSlowPercent)&&
        SameDouble(current.config.initialSlowDistanceMm,b.config.initialSlowDistanceMm)&&
        current.config.deepFlushEnabled==b.config.deepFlushEnabled&&
        current.config.deepFlushCycleInterval==b.config.deepFlushCycleInterval&&
        SameDouble(current.config.deepFlushHeightMultiplier,b.config.deepFlushHeightMultiplier)&&
        SameDouble(current.config.deepFlushHeightMm,b.config.deepFlushHeightMm)&&
        SameFlushProfile(current.config.flushProfile,b.config.flushProfile)&&
        SameDouble(current.config.jumpHeightMm,b.config.jumpHeightMm)&&
        SameDouble(current.config.jumpSpeedPercent,b.config.jumpSpeedPercent)&&
        SameDouble(current.config.machiningOverridePercent,b.config.machiningOverridePercent)&&
        SameDouble(current.config.referenceV,b.config.referenceV)&&SameServoProfile(current.config.curve,b.config.curve);
}
}
