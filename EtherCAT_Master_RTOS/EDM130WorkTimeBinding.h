#pragma once
#include "EDM130WorkTimer.h"
#include "EDMFlushRecipeBinding.h"
#include "EDMProcessProfile.h"
#include "EDM137ConfirmedUnits.h"

// Owner-thread adapters. EDM130 binds DischargeWorkTime only; the explicit
// EDM131 additionally freezes JumpSpeedOverride; the explicit EDM132 adapter
// captures JumpHeight in millimetres in the same immutable selection.
namespace EDM130
{
struct Binding
{
    Recipe recipe{};
    EDM58::BindingError error=EDM58::BindingError::NotReady;
    std::uint16_t failedFieldId=0U;
    bool ready=false;
};
inline Binding BindWorkTime(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId) noexcept
{
    static_assert(EDMRecipe::Scale==1000000LL,"Fixed-point seconds must be exact microseconds.");
    Binding out{}; const auto before=controller.Current(); const auto* catalog=controller.GetCatalog();
    if(!catalog||!before.catalogReady)return out;
    if(catalog->schemaVersion!=2U||catalog->mode!=EDMRecipe::Mode::ShadowOnly)
    {out.error=EDM58::BindingError::UnsupportedSchema;return out;}
    if(!expectedProfileId||catalog->profileId!=expectedProfileId||before.profileId!=expectedProfileId)
    {out.error=EDM58::BindingError::ProfileMismatch;return out;}
    if(!before.tableSelected||!before.rowSelected)
    {out.error=EDM58::BindingError::NoSelection;return out;}
    if(!before.generation||!before.tableId||before.tableId>EDMRecipe::MaximumConditionTableId||
        !before.tableRevision||!before.eCode||before.eCode>EDMRecipe::RowsPerTable||
        !catalog->catalogRevision||!catalog->definitionRevision||
        before.catalogRevision!=catalog->catalogRevision||before.definitionRevision!=catalog->definitionRevision||
        catalog->fields.empty()||catalog->fields.size()>EDMRecipe::MaximumFields||before.fieldCount!=catalog->fields.size())
    {out.error=EDM58::BindingError::InvalidIdentity;return out;}
    const EDMRecipe::Table* table=nullptr;
    for(const auto& item:catalog->tables)if(item.id==before.tableId)
    {if(table){out.error=EDM58::BindingError::InvalidIdentity;return out;}table=&item;}
    if(!table||table->revision!=before.tableRevision||!table->rowPresent[before.eCode-1U])
    {out.error=EDM58::BindingError::InvalidIdentity;return out;}
    EDMRecipe::Value value=0LL; std::uint16_t field=0U; EDM58::Binding semantic{};
    if(!EDM58::BindingDetail::Read(controller,*catalog,"DischargeWorkTime","s",value,field,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    if(value<50000LL||value>10000000LL)
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=field;return out;}
    Recipe r{};r.catalogRevision=before.catalogRevision;r.definitionRevision=before.definitionRevision;
    r.profileId=before.profileId;r.tableId=before.tableId;r.tableRevision=before.tableRevision;
    r.eCode=before.eCode;r.fieldId=field;r.generation=before.generation;r.workTimeUs=static_cast<std::uint64_t>(value);
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!ValidRecipe(r)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=r;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
inline Binding BindWorkTimeAndSpeed(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const Binding time=BindWorkTime(controller,expectedProfileId);
    if(!time.ready)return time;
    // One capture covers both fields, including the actual staged overrides.
    // The first adapter independently validates all catalog/table identities.
    if(!catalog||controller.GetCatalog()!=catalog||
        !EDM58::BindingDetail::SameSelection(before,controller.Current())||
        time.recipe.generation!=before.generation||time.recipe.tableId!=before.tableId||
        time.recipe.tableRevision!=before.tableRevision||time.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
    if(!EDM58::BindingDetail::Read(controller,*catalog,"JumpSpeedOverride","%",value,field,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    if(value<0LL||!ValidSpeedPercent(static_cast<std::uint64_t>(value)))
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=field;return out;}
    Recipe r=time.recipe;r.speedFieldId=field;r.jumpSpeedPercentScaled=static_cast<std::uint64_t>(value);
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!ValidRecipe(r)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=r;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
inline Binding BindWorkTimeHeightAndSpeed(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,unsigned commissioningProfile=0U) noexcept
{
    static_assert(EDMRecipe::Scale==1000000LL,"Fixed-point millimetres must preserve one nanometre.");
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const Binding timedSpeed=BindWorkTimeAndSpeed(controller,expectedProfileId);
    if(!timedSpeed.ready)return timedSpeed;
    // The outer capture covers all three reads, including actual staged values.
    if(!catalog||controller.GetCatalog()!=catalog||
        !EDM58::BindingDetail::SameSelection(before,controller.Current())||
        timedSpeed.recipe.generation!=before.generation||timedSpeed.recipe.tableId!=before.tableId||
        timedSpeed.recipe.tableRevision!=before.tableRevision||timedSpeed.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
    if(!EDM58::BindingDetail::Read(controller,*catalog,"JumpHeight","mm",value,field,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    if(value<0LL||!ValidJumpHeight(static_cast<std::uint64_t>(value),commissioningProfile))
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=field;return out;}
    Recipe r=timedSpeed.recipe;r.commissioningProfile=static_cast<std::uint16_t>(commissioningProfile);r.heightFieldId=field;r.jumpHeightScaled=static_cast<std::uint64_t>(value);
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!ValidRecipe(r)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=r;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
inline Binding BindWorkTimeHeightSpeedAndDeepFlush(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,unsigned commissioningProfile=0U) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const Binding base=BindWorkTimeHeightAndSpeed(controller,expectedProfileId,commissioningProfile);
    if(!base.ready)return base;
    if(!catalog||controller.GetCatalog()!=catalog||
        !EDM58::BindingDetail::SameSelection(before,controller.Current())||
        base.recipe.generation!=before.generation||base.recipe.tableId!=before.tableId||
        base.recipe.tableRevision!=before.tableRevision||base.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    EDMRecipe::Value interval=0LL,multiplier=0LL;std::uint16_t intervalField=0U,multiplierField=0U;
    EDM58::Binding semantic{};
    if(!EDM58::BindingDetail::Read(controller,*catalog,"DeepFlushCycleInterval","cycle",interval,intervalField,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    if(interval<0LL||interval>20000000LL||interval%EDMRecipe::Scale!=0LL)
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=intervalField;return out;}
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!EDM58::BindingDetail::Read(controller,*catalog,"DeepFlushHeightMultiplier","x",multiplier,multiplierField,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    if(multiplier<0LL||multiplier>20000000LL)
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=multiplierField;return out;}
    Recipe r=base.recipe;r.intervalFieldId=intervalField;r.multiplierFieldId=multiplierField;
    r.deepFlushCycleInterval=static_cast<std::uint32_t>(interval/EDMRecipe::Scale);
    r.deepFlushHeightMultiplierScaled=static_cast<std::uint64_t>(multiplier);
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!ValidDeepFlushValues(r.jumpHeightScaled,r.deepFlushCycleInterval,r.deepFlushHeightMultiplierScaled,commissioningProfile))
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=multiplierField;return out;}
    if(!ValidRecipe(r)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=r;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
// EDM134 explicitly opts in to E8 without changing earlier binding adapters.
inline Binding BindWorkTimeHeightSpeedDeepAndMachining(const EDMRecipe::Controller& controller,
    std::uint32_t expectedProfileId,unsigned commissioningProfile=0U) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const Binding base=BindWorkTimeHeightSpeedAndDeepFlush(controller,expectedProfileId,commissioningProfile);
    if(!base.ready)return base;
    if(!catalog||controller.GetCatalog()!=catalog||
        !EDM58::BindingDetail::SameSelection(before,controller.Current())||
        base.recipe.generation!=before.generation||base.recipe.tableId!=before.tableId||
        base.recipe.tableRevision!=before.tableRevision||base.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
    if(!EDM58::BindingDetail::Read(controller,*catalog,"MachiningServoOverride","%",value,field,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    // Check the signed native value before narrowing to the bounded RT record.
    if(value<1000000LL||value>100000000LL)
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=field;return out;}
    Recipe r=base.recipe;r.machiningFieldId=field;
    r.machiningSpeedPercentScaled=static_cast<std::uint32_t>(value);
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!ValidRecipe(r)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=r;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
// EDM135 explicitly opts in to E9 without changing earlier binding adapters.
inline Binding BindWorkTimeHeightSpeedDeepMachiningAndReference(const EDMRecipe::Controller& controller,
    std::uint32_t expectedProfileId,unsigned commissioningProfile=0U) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const Binding base=BindWorkTimeHeightSpeedDeepAndMachining(controller,expectedProfileId,commissioningProfile);
    if(!base.ready)return base;
    if(!catalog||controller.GetCatalog()!=catalog||
        !EDM58::BindingDetail::SameSelection(before,controller.Current())||
        base.recipe.generation!=before.generation||base.recipe.tableId!=before.tableId||
        base.recipe.tableRevision!=before.tableRevision||base.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    EDMRecipe::Value value=0LL;std::uint16_t field=0U;EDM58::Binding semantic{};
    if(!EDM58::BindingDetail::Read(controller,*catalog,"GapVoltageSetpoint","V",value,field,semantic))
    {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
    // Check the signed native value before narrowing to the bounded RT record.
    if(value<15000000LL||value>200000000LL)
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=field;return out;}
    Recipe r=base.recipe;r.referenceFieldId=field;
    r.referenceVoltageScaled=static_cast<std::uint32_t>(value);
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    if(!ValidRecipe(r)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=r;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
// EDM136: the owning NC scan captures one installed process shape alongside
// the seven E fields. Later tuning edits take effect at the next explicit arm.
inline Binding BindWorkTimeHeightSpeedDeepMachiningReferenceAndCurve(const EDMRecipe::Controller& controller,
    std::uint32_t expectedProfileId,const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile=0U) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    if(!tuningGeneration||!EDM20::ValidateProcessProfile(process))
    {out.error=EDM58::BindingError::InvalidRecipe;return out;}
    if(!expectedProfileId||process.machineProfileId!=expectedProfileId)
    {out.error=EDM58::BindingError::ProfileMismatch;return out;}
    Curve curve{};curve.processRevision=process.revision;curve.machineProfileId=process.machineProfileId;
    curve.tuningGeneration=tuningGeneration;curve.deadbandV=process.servo.deadbandV;
    for(unsigned i=0U;i<3U;++i){curve.positiveGain[i]=process.servo.positiveGain[i];curve.negativeGain[i]=process.servo.negativeGain[i];}
    for(unsigned i=0U;i<2U;++i){curve.positiveBreakV[i]=process.servo.positiveBreakV[i];curve.negativeBreakV[i]=process.servo.negativeBreakV[i];}
    const Binding base=BindWorkTimeHeightSpeedDeepMachiningAndReference(controller,expectedProfileId,commissioningProfile);
    if(!base.ready)return base;
    if(!catalog||controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current())||
        base.recipe.generation!=before.generation||base.recipe.tableId!=before.tableId||
        base.recipe.tableRevision!=before.tableRevision||base.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    // No mutable process pointer crosses into RT. Recheck the captured fields
    // before publishing, even though normal writes share this same NC owner.
    if(!EDM20::ValidateProcessProfile(process)||process.revision!=curve.processRevision||process.machineProfileId!=curve.machineProfileId||
        process.servo.revision!=curve.processRevision||
        CurveDoubleBits(process.servo.deadbandV)!=CurveDoubleBits(curve.deadbandV))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    for(unsigned i=0U;i<3U;++i)
        if(CurveDoubleBits(process.servo.positiveGain[i])!=CurveDoubleBits(curve.positiveGain[i])||
            CurveDoubleBits(process.servo.negativeGain[i])!=CurveDoubleBits(curve.negativeGain[i]))
        {out.error=EDM58::BindingError::CaptureChanged;return out;}
    for(unsigned i=0U;i<2U;++i)
        if(CurveDoubleBits(process.servo.positiveBreakV[i])!=CurveDoubleBits(curve.positiveBreakV[i])||
            CurveDoubleBits(process.servo.negativeBreakV[i])!=CurveDoubleBits(curve.negativeBreakV[i]))
        {out.error=EDM58::BindingError::CaptureChanged;return out;}
    Recipe recipe=base.recipe;recipe.curve=curve;
    if(!ValidRecipe(recipe)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=recipe;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
// EDM137 opts in to confirmed millimetres/percentages and polarity. Native
// process pump bases are copied at the same G180 admission as all E values.
// E15 belongs to the discharge circuit and is deliberately not interpreted.
inline Binding BindWorkTimeHeightSpeedDeepMachiningReferenceCurveAndFlushStages(
    const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile=0U) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const double finalBase=process.flush.finalApproachMmPerMin,initialBase=process.flush.initialSlowMmPerMin;
    const Binding base=BindWorkTimeHeightSpeedDeepMachiningReferenceAndCurve(
        controller,expectedProfileId,process,tuningGeneration,commissioningProfile);
    if(!base.ready)return base;
    if(!catalog||controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current())||
        base.recipe.generation!=before.generation||base.recipe.tableId!=before.tableId||
        base.recipe.tableRevision!=before.tableRevision||base.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    const char* semantics[]={"FinalApproachSpeed","DecelerationDistance","LargeAreaMachiningSpeed",
        "LargeAreaFlushDistance","ElectrodePolarity"};
    const char* units[]={"%","mm","%","mm","code"};
    const EDMRecipe::Value minimum[]={10000000LL,0LL,1000000LL,0LL,0LL};
    const EDMRecipe::Value maximum[]={100000000LL,100000000LL,100000000LL,900000000LL,1000000LL};
    EDMRecipe::Value values[5]{};std::uint16_t fields[5]{};EDM58::Binding semantic{};
    for(unsigned i=0U;i<5U;++i)
    {
        if(!EDM137Units::ReadExplicit(controller,*catalog,semantics[i],units[i],values[i],fields[i],semantic))
        {out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
        if(values[i]<minimum[i]||values[i]>maximum[i]||(i==4U&&values[i]%EDMRecipe::Scale!=0LL))
        {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=fields[i];return out;}
    }
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    Curve after{};after.processRevision=process.revision;after.machineProfileId=process.machineProfileId;
    after.tuningGeneration=tuningGeneration;after.deadbandV=process.servo.deadbandV;
    for(unsigned i=0U;i<3U;++i){after.positiveGain[i]=process.servo.positiveGain[i];after.negativeGain[i]=process.servo.negativeGain[i];}
    for(unsigned i=0U;i<2U;++i){after.positiveBreakV[i]=process.servo.positiveBreakV[i];after.negativeBreakV[i]=process.servo.negativeBreakV[i];}
    if(!EDM20::ValidateProcessProfile(process)||!SameCurve(after,base.recipe.curve)||
        CurveDoubleBits(process.flush.finalApproachMmPerMin)!=CurveDoubleBits(finalBase)||
        CurveDoubleBits(process.flush.initialSlowMmPerMin)!=CurveDoubleBits(initialBase))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    Recipe recipe=base.recipe;auto& s=recipe.flushStages;
    s.finalSpeedFieldId=fields[0];s.approachFieldId=fields[1];s.initialSpeedFieldId=fields[2];
    s.initialDistanceFieldId=fields[3];s.polarityFieldId=fields[4];
    s.finalSpeedPercentScaled=static_cast<std::uint32_t>(values[0]);
    s.approachDistanceScaled=static_cast<std::uint64_t>(values[1]);
    s.initialSpeedPercentScaled=static_cast<std::uint32_t>(values[2]);
    s.initialDistanceScaled=static_cast<std::uint64_t>(values[3]);
    s.polarity=static_cast<std::uint32_t>(values[4]/EDMRecipe::Scale);
    s.finalApproachMmPerMin=finalBase;s.initialSlowMmPerMin=initialBase;
    if(!ValidRecipe(recipe)){out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=recipe;out.ready=true;out.error=EDM58::BindingError::None;return out;
}

// EDM173 freezes the complete native G76 recipe atomically. The legacy
// incremental visible-flush adapters cannot validate this profile halfway
// through capture: source3 and the sealed 25-child plan are mandatory here.
// EDM267 FIX2: keep this NC-owner capture out of each recipe dispatch branch.
// Retain inline linkage while bounding compiler expansion; no RT callback uses it.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
inline Binding BindG76GapProfile175(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile,
    unsigned architectureSelector=0U) noexcept
{
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    const bool expansion=IsG76ExpansionSelector263(architectureSelector);
    const bool fixedRadius=IsG76FixedRadiusSelector266(architectureSelector);
    const bool fixedRadiusWork=IsG76FixedRadiusWorkSelector267(architectureSelector);
    const unsigned sharedCycleMode=G76SharedCycleMode284(architectureSelector);
    const bool sharedCycle=sharedCycleMode!=0U;
    const unsigned sharedMode=G76SharedShortMode274(architectureSelector);
    const bool sharedShort=sharedMode!=0U||(sharedCycle&&(commissioningProfile==VisibleFlushProfile173||
        commissioningProfile==G76B3Profile176||commissioningProfile==G76B4Profile177));
    const std::uint64_t sharedShortHeight=sharedCycle?G76SharedCycleShortHeightScaled284:G76SharedShortHeightScaled274;
    const unsigned sharedPathMode=G76SharedPathMode283(architectureSelector);
    const bool sharedPath=sharedPathMode!=0U;
    const bool oneMm=IsG76OneMmSelector260(architectureSelector)||(architectureSelector>=265U&&architectureSelector<=268U)||
        (architectureSelector>=273U&&architectureSelector<=276U)||(architectureSelector>=281U&&architectureSelector<=284U);
    const bool polygonB2=architectureSelector==264U||(architectureSelector>=269U&&architectureSelector<=272U)||
        (architectureSelector>=277U&&architectureSelector<=280U)||(architectureSelector>=285U&&architectureSelector<=288U);
    if((sharedPath&&commissioningProfile!=G76B0Profile192&&commissioningProfile!=G76B2Profile175)||
        (sharedShort&&commissioningProfile!=VisibleFlushProfile173&&commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)||
        (polygonB2&&commissioningProfile!=G76B2Profile175)||
        (oneMm&&commissioningProfile!=VisibleFlushProfile173&&commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)||
        (architectureSelector==250U&&commissioningProfile!=VisibleFlushProfile173)||
        (architectureSelector==251U&&commissioningProfile!=G76B2Profile175)||
        (architectureSelector==252U&&commissioningProfile!=G76B3Profile176)||
        (architectureSelector==253U&&commissioningProfile!=G76B4Profile177)||
        (architectureSelector==255U&&commissioningProfile!=VisibleFlushProfile173)||
        (architectureSelector==256U&&commissioningProfile!=G76B2Profile175)||
        (architectureSelector==257U&&commissioningProfile!=G76B3Profile176)||
        (architectureSelector==258U&&commissioningProfile!=G76B4Profile177)||
        (!sharedCycle&&!sharedPath&&!sharedShort&&!oneMm&&!polygonB2&&architectureSelector!=0U&&architectureSelector!=250U&&architectureSelector!=251U&&architectureSelector!=252U&&architectureSelector!=253U&&architectureSelector!=255U&&architectureSelector!=256U&&architectureSelector!=257U&&architectureSelector!=258U&&((architectureSelector!=233U&&architectureSelector!=239U&&architectureSelector!=240U&&architectureSelector!=241U&&architectureSelector!=242U&&architectureSelector!=243U&&architectureSelector!=244U&&architectureSelector!=245U&&architectureSelector!=246U&&architectureSelector!=247U&&architectureSelector!=248U)||
        (commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)||
        (architectureSelector==245U&&commissioningProfile!=G76B3Profile176)||
        (architectureSelector==246U&&commissioningProfile!=G76B4Profile177)||
        (architectureSelector==247U&&commissioningProfile!=G76B3Profile176)||
        (architectureSelector==248U&&commissioningProfile!=G76B4Profile177))))
    {out.error=EDM58::BindingError::InvalidRecipe;return out;}
    if(commissioningProfile!=VisibleFlushProfile173&&commissioningProfile!=G76B2Profile175&&commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177&&commissioningProfile!=G76B0Profile192)
    {out.error=EDM58::BindingError::InvalidRecipe;return out;}
    if(!tuningGeneration||!EDM20::ValidateProcessProfile(process))
    {out.error=EDM58::BindingError::InvalidRecipe;return out;}
    if(!expectedProfileId||process.machineProfileId!=expectedProfileId)
    {out.error=EDM58::BindingError::ProfileMismatch;return out;}
    const double finalBase=process.flush.finalApproachMmPerMin,initialBase=process.flush.initialSlowMmPerMin;
    Curve curve{};curve.processRevision=process.revision;curve.machineProfileId=process.machineProfileId;
    curve.tuningGeneration=tuningGeneration;curve.deadbandV=process.servo.deadbandV;
    for(unsigned i=0U;i<3U;++i){curve.positiveGain[i]=process.servo.positiveGain[i];curve.negativeGain[i]=process.servo.negativeGain[i];}
    for(unsigned i=0U;i<2U;++i){curve.positiveBreakV[i]=process.servo.positiveBreakV[i];curve.negativeBreakV[i]=process.servo.negativeBreakV[i];}
    const Binding base=BindWorkTimeAndSpeed(controller,expectedProfileId);
    if(!base.ready)return base;
    if(!catalog||controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current())||
        base.recipe.generation!=before.generation||base.recipe.tableId!=before.tableId||
        base.recipe.tableRevision!=before.tableRevision||base.recipe.eCode!=before.eCode)
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    const bool validSpeed=ValidG76FlushSpeedPercent(commissioningProfile,base.recipe.jumpSpeedPercentScaled)&&
        (!(sharedCycle||sharedPath||sharedShort||oneMm||polygonB2)||base.recipe.jumpSpeedPercentScaled==100000000ULL);
    if(base.recipe.workTimeUs<VisibleFlushMinimumWorkUs173||base.recipe.workTimeUs>VisibleFlushMaximumWorkUs173||!validSpeed||
        ((sharedCycle||sharedPath||sharedShort||oneMm||polygonB2||architectureSelector==250U||architectureSelector==251U||architectureSelector==252U||architectureSelector==253U||architectureSelector==255U||architectureSelector==256U||architectureSelector==257U||architectureSelector==258U)&&base.recipe.workTimeUs!=500000ULL))
    {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=
        !validSpeed?base.recipe.speedFieldId:base.recipe.fieldId;return out;}
    const char* semantics[]={"JumpHeight","DeepFlushCycleInterval","DeepFlushHeightMultiplier","MachiningServoOverride","GapVoltageSetpoint",
        "FinalApproachSpeed","DecelerationDistance","LargeAreaMachiningSpeed","LargeAreaFlushDistance","ElectrodePolarity"};
    const char* units[]={"mm","cycle","x","%","V","%","mm","%","mm","code"};
    const EDMRecipe::Value exact[]={sharedShort?static_cast<EDMRecipe::Value>(sharedShortHeight):oneMm?1000000LL:commissioningProfile==G76B2Profile175?1000LL:2000000LL,
        0LL,1000000LL,40000000LL,55000000LL,10000000LL,100LL,1000000LL,0LL,0LL};
    EDMRecipe::Value values[10]{};std::uint16_t fields[10]{};EDM58::Binding semantic{};
    for(unsigned i=0U;i<10U;++i)
    {
        const bool read=i<5U?EDM58::BindingDetail::Read(controller,*catalog,semantics[i],units[i],values[i],fields[i],semantic):
            EDM137Units::ReadExplicit(controller,*catalog,semantics[i],units[i],values[i],fields[i],semantic);
        if(!read){out.error=semantic.error;out.failedFieldId=semantic.failedFieldId;return out;}
        const bool centerHeight=i==0U&&(commissioningProfile==G76B3Profile176||commissioningProfile==G76B4Profile177);
        if(centerHeight?(values[i]<0LL||!ValidG76CenterHeightScaled(static_cast<std::uint64_t>(values[i]))||
            (sharedShort&&values[i]!=static_cast<EDMRecipe::Value>(sharedShortHeight))||
            (oneMm&&values[i]!=1000000LL)||
            ((architectureSelector==252U||architectureSelector==257U)&&values[i]!=static_cast<EDMRecipe::Value>(G76B3HeightScaled176))||
            ((architectureSelector==253U||architectureSelector==258U)&&values[i]!=static_cast<EDMRecipe::Value>(G76B4HeightScaled177))||
            (values[i]==static_cast<EDMRecipe::Value>(G76ArchitectureHeightScaled233)&&!sharedShort&&architectureSelector!=233U&&architectureSelector!=239U&&architectureSelector!=240U&&architectureSelector!=241U&&architectureSelector!=242U&&architectureSelector!=243U&&architectureSelector!=244U&&architectureSelector!=245U&&architectureSelector!=246U&&architectureSelector!=247U&&architectureSelector!=248U)||
            ((architectureSelector==239U||architectureSelector==240U||architectureSelector==241U||architectureSelector==242U||architectureSelector==243U||architectureSelector==244U||architectureSelector==245U||architectureSelector==246U||architectureSelector==247U||architectureSelector==248U)&&values[i]!=static_cast<EDMRecipe::Value>(G76ArchitectureHeightScaled233))):values[i]!=exact[i])
        {out.error=EDM58::BindingError::EngineeringRange;out.failedFieldId=fields[i];return out;}
    }
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    Curve after{};after.processRevision=process.revision;after.machineProfileId=process.machineProfileId;
    after.tuningGeneration=tuningGeneration;after.deadbandV=process.servo.deadbandV;
    for(unsigned i=0U;i<3U;++i){after.positiveGain[i]=process.servo.positiveGain[i];after.negativeGain[i]=process.servo.negativeGain[i];}
    for(unsigned i=0U;i<2U;++i){after.positiveBreakV[i]=process.servo.positiveBreakV[i];after.negativeBreakV[i]=process.servo.negativeBreakV[i];}
    if(!EDM20::ValidateProcessProfile(process)||!SameCurve(after,curve)||
        CurveDoubleBits(process.flush.finalApproachMmPerMin)!=CurveDoubleBits(finalBase)||
        CurveDoubleBits(process.flush.initialSlowMmPerMin)!=CurveDoubleBits(initialBase))
    {out.error=EDM58::BindingError::CaptureChanged;return out;}
    Recipe recipe=base.recipe;
    recipe.commissioningProfile=static_cast<std::uint16_t>(commissioningProfile);recipe.diagnosticControlSource=3U;recipe.planRows=expansion||fixedRadius||fixedRadiusWork||(sharedMode&&(sharedMode<10U||sharedMode>=20U))||
        (sharedPath&&(sharedPathMode<10U||sharedPathMode>=20U))||
        (sharedCycle&&(sharedCycleMode<10U||sharedCycleMode>=20U))?26U:25U;recipe.planMovementMask=7U;
    // Set the explicit selector before full validation, never patch a rejected
    // short-height capture into a nominally valid legacy binding afterwards.
    recipe.flushHoldTest=static_cast<std::uint16_t>(architectureSelector);
    recipe.heightFieldId=fields[0];recipe.jumpHeightScaled=static_cast<std::uint64_t>(values[0]);
    recipe.intervalFieldId=fields[1];recipe.multiplierFieldId=fields[2];recipe.deepFlushCycleInterval=0U;
    recipe.deepFlushHeightMultiplierScaled=static_cast<std::uint64_t>(values[2]);
    recipe.machiningFieldId=fields[3];recipe.machiningSpeedPercentScaled=static_cast<std::uint32_t>(values[3]);
    recipe.referenceFieldId=fields[4];recipe.referenceVoltageScaled=static_cast<std::uint32_t>(values[4]);recipe.curve=curve;
    auto& stages=recipe.flushStages;
    stages.finalSpeedFieldId=fields[5];stages.approachFieldId=fields[6];stages.initialSpeedFieldId=fields[7];
    stages.initialDistanceFieldId=fields[8];stages.polarityFieldId=fields[9];
    stages.finalSpeedPercentScaled=static_cast<std::uint32_t>(values[5]);stages.approachDistanceScaled=static_cast<std::uint64_t>(values[6]);
    stages.initialSpeedPercentScaled=static_cast<std::uint32_t>(values[7]);stages.initialDistanceScaled=static_cast<std::uint64_t>(values[8]);
    stages.polarity=static_cast<std::uint32_t>(values[9]);stages.finalApproachMmPerMin=finalBase;stages.initialSlowMmPerMin=initialBase;
    if(!ValidHistoryRecipeBinding(EDM149::G76GapHistoryFixture,recipe))
    {out.error=EDM58::BindingError::InvalidRecipe;return out;}
    out.recipe=recipe;out.ready=true;out.error=EDM58::BindingError::None;return out;
}
inline Binding BindG76Gap173(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,VisibleFlushProfile173);}
inline Binding BindG76SquareB1_250(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,VisibleFlushProfile173,250U);}
inline Binding BindG76SquareB2_251(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B2Profile175,251U);}
inline Binding BindG76SquareB3_252(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B3Profile176,252U);}
inline Binding BindG76SquareB4_253(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B4Profile177,253U);}
inline Binding BindG76B2Gap175(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B2Profile175);}
inline Binding BindG76DiamondB1_255(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,VisibleFlushProfile173,255U);}
inline Binding BindG76DiamondB2_256(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B2Profile175,256U);}
inline Binding BindG76DiamondB3_257(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B3Profile176,257U);}
inline Binding BindG76DiamondB4_258(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B4Profile177,258U);}
// The shape selector and B profile are validated before the complete capture.
// Legacy binders continue to require their original two-mm recipes.
inline Binding BindG76OneMm260(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned modeP,unsigned bMode) noexcept
{
    const unsigned selector=modeP==11U?260U:modeP==12U?261U:modeP==13U?262U:modeP==16U?263U:0U;
    const unsigned profile=bMode==1U?VisibleFlushProfile173:bMode==3U?G76B3Profile176:bMode==4U?G76B4Profile177:0U;
    if(!selector||!profile){Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,profile,selector);
}
inline Binding BindG76PolygonB2_261(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B2Profile175,264U);}
inline Binding BindG76Expansion263(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned modeP,unsigned bMode) noexcept
{
    const unsigned shape=modeP==21U?0U:modeP==22U?1U:modeP==23U?2U:modeP==26U?3U:4U;
    const unsigned profile=bMode==1U?VisibleFlushProfile173:bMode==2U?G76B2Profile175:
        bMode==3U?G76B3Profile176:bMode==4U?G76B4Profile177:0U;
    if(shape>=4U||!profile){Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,profile,
        (bMode==2U?269U:265U)+shape);
}
inline Binding BindG76FixedRadius266(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned modeP,unsigned bMode) noexcept
{
    const unsigned shape=modeP==1U?0U:modeP==2U?1U:modeP==3U?2U:modeP==6U?3U:4U;
    const unsigned profile=bMode==1U?VisibleFlushProfile173:bMode==2U?G76B2Profile175:
        bMode==3U?G76B3Profile176:bMode==4U?G76B4Profile177:0U;
    if(shape>=4U||!profile){Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,profile,
        (bMode==2U?277U:273U)+shape);
}
inline Binding BindG76FixedRadiusWork267(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned modeP,unsigned bMode) noexcept
{
    const unsigned shape=modeP==1U?0U:modeP==2U?1U:modeP==3U?2U:modeP==6U?3U:4U;
    const unsigned profile=bMode==1U?VisibleFlushProfile173:bMode==2U?G76B2Profile175:
        bMode==3U?G76B3Profile176:bMode==4U?G76B4Profile177:0U;
    if(shape>=4U||!profile){Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,profile,
        (bMode==2U?285U:281U)+shape);
}
inline Binding BindG76B3Gap176(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B3Profile176);}
inline Binding BindG76B4Gap177(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B4Profile177);}
inline Binding BindG76CenterArchitecture233(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,233U);
}
inline Binding BindG76CenterSequence239(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,239U);
}
inline Binding BindG76CenterSequence240(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,240U);
}
inline Binding BindG76CenterSequence241(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,241U);
}
inline Binding BindG76CenterSequence242(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,242U);
}
inline Binding BindG76CenterSequence243(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,243U);
}
inline Binding BindG76CenterSequence244(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176&&commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,244U);
}
inline Binding BindG76CenterSequence245(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,245U);
}
inline Binding BindG76CenterSequence246(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,246U);
}
inline Binding BindG76CenterSequence247(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B3Profile176)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,247U);
}
inline Binding BindG76CenterSequence248(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration,unsigned commissioningProfile) noexcept
{
    if(commissioningProfile!=G76B4Profile177)
    {Binding out{};out.error=EDM58::BindingError::InvalidRecipe;return out;}
    return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,commissioningProfile,248U);
}
inline Binding BindG76B0Gap192(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId,
    const EDM20::ProcessProfile& process,std::uint64_t tuningGeneration) noexcept
{return BindG76GapProfile175(controller,expectedProfileId,process,tuningGeneration,G76B0Profile192);}
}
