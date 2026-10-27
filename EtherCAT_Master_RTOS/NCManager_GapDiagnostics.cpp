// EDM267 FIX1: single-copy NC diagnostic observers, extracted without behavior changes.
#include "NCManager_GapInternal.h"

namespace NCGapDetail
{
namespace
{
    // EDM206_NC_HISTORY_BEGIN
    // NC owns only this cursor and two copied rows. Reading ownership keeps
    // the RT ring immutable until HISTORY_END; a new arm is never delayed.
    struct EDM206NCObserver
    {
        EDM206::Header header{};
        EDM206::Row row{}, previous{};
        std::uint64_t frequency=0ULL,lastCounter=0ULL,lastLiveSample=0ULL;
        unsigned index=0U,part=0U;
        bool active=false,haveRow=false,havePrevious=false,clockSeen=false;
    };
    EDM206NCObserver g_edm206NCObserver{};

    bool EDM206LiveCommandIdle(const MotionStopSettleSnapshot& sample) noexcept
    {
        // This is a logging gate, not standstill or physical stop proof.
        return sample.publicationGeneration && sample.sampleSequence &&
            !sample.groupActive && sample.existingAxisCount==6U &&
            sample.pdoTargetVelocitySampledAxisCount==6U &&
            sample.commandQueueDepth==0U && sample.commandIngressDepth==0U &&
            sample.commandReplayDepth==0U && sample.commandMovingAxisCount==0U &&
            sample.pdoTargetVelocityNonzeroAxisCount==0U;
    }
    bool EDM206LogPeriodReady(EDM206NCObserver& observer) noexcept
    {
        LARGE_INTEGER frequency{},counter{};
        if(!observer.frequency) {
            if(!RtQueryPerformanceFrequency(&frequency)||frequency.QuadPart<=0)return false;
            observer.frequency=static_cast<std::uint64_t>(frequency.QuadPart);
        }
        if(!RtQueryPerformanceCounter(&counter)||counter.QuadPart<0)return false;
        const auto now=static_cast<std::uint64_t>(counter.QuadPart);
        const auto period=observer.frequency/100ULL+(observer.frequency%100ULL?1ULL:0ULL);
        if(observer.clockSeen) {
            if(now<observer.lastCounter){observer.lastCounter=now;return false;}
            if(now-observer.lastCounter<period)return false;
        }
        observer.clockSeen=true;observer.lastCounter=now;return true;
    }
    bool EDM206ActualVelocity(const EDM206NCObserver& observer,unsigned axis,double& pps) noexcept
    {
        pps=0.0;
        if(!observer.havePrevious||axis>=EDM206::AxisCount)return false;
        const auto& row=observer.row;const auto& previous=observer.previous;
        const unsigned required=EDM206::Complete|EDM206::FrameCurrent|EDM206::SourceFresh|EDM206::ClockValid;
        if((row.flags&required)!=required||(previous.flags&required)!=required||
            !(row.flags&EDM206::SourceIntervalValid)||
            !(row.axis[axis].flags&EDM206::FrameAvailable)||
            !(previous.axis[axis].flags&EDM206::FrameAvailable)||
            previous.tick==(std::numeric_limits<std::uint64_t>::max)()||row.tick!=previous.tick+1ULL||
            !row.session||row.session!=previous.session||!row.epoch||row.epoch!=previous.epoch||
            !row.ownerGeneration||row.ownerGeneration!=previous.ownerGeneration||
            row.sourceUs<=previous.sourceUs||row.sourceDeltaUs!=row.sourceUs-previous.sourceUs||
            !std::isfinite(row.axis[axis].frameActualPulse)||!std::isfinite(previous.axis[axis].frameActualPulse))return false;
        const double delta=row.axis[axis].frameActualPulse-previous.axis[axis].frameActualPulse;
        if(!std::isfinite(delta))return false;
        const double value=delta/(static_cast<double>(row.sourceDeltaUs)*0.000001);
        if(!std::isfinite(value))return false;
        pps=value;return true;
    }
    const char* EDM207InputSourceName(EDM206::InputSource source) noexcept
    {
        switch(source) {
        case EDM206::InputSource::Legacy: return "LEGACY";
        case EDM206::InputSource::ControlledSemantic: return "CONTROLLED";
        case EDM206::InputSource::RetiredSemantic: return "RETIRED";
        case EDM206::InputSource::EmergencyFallback: return "EMERGENCY";
        default: return "UNKNOWN";
        }
    }
    bool EDM207InputCurrent(const EDM206::Row& row,unsigned axis) noexcept
    {
        if(axis>=EDM206::AxisCount||!(row.inputMask&(1U<<axis)))return false;
        const auto& input=row.axis[axis].input;
        const unsigned required=EDM206::ConsumedValid|EDM206::Decoded;
        return row.session&&input.session==row.session&&input.tick==row.tick&&
            input.sourceUs==row.sourceUs&&(input.flags&required)==required&&
            input.source!=EDM206::InputSource::Unknown&&
            static_cast<unsigned>(input.source)<=static_cast<unsigned>(EDM206::InputSource::EmergencyFallback);
    }
    void EndEDM206History(MotionCore& motion,const char* result) noexcept
    {
        auto& observer=g_edm206NCObserver;
        const auto session=observer.header.session;
        PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_END session=%llu result=%s rows=%u expected=%u firstTick=%llu lastTick=%llu driveAck=0 authority=NONE\n",
            static_cast<unsigned long long>(session),result,observer.index,observer.header.count,
            static_cast<unsigned long long>(observer.header.firstTick),static_cast<unsigned long long>(observer.header.lastTick));
        // No row reference or RT pointer survives this release.
        motion.EndReadEDM206History(session);
        observer.active=observer.haveRow=observer.havePrevious=false;
        observer.index=observer.part=0U;
    }
}

    void PollEDM206History(MotionCore& motion,NCState ncState) noexcept
    {
        // Run before the NC bracket's latched early return: RESET and a later
        // session cannot erase a pending immutable fault. Pause for new motion.
        if(ncState==NCState::RUN||ncState==NCState::HOLD)return;
        const auto live=motion.GetStopSettleSnapshot();
        if(!EDM206LiveCommandIdle(live))return;
        auto& observer=g_edm206NCObserver;
        if(!observer.active) {
            if(!motion.BeginReadEDM206History(observer.header))return;
            observer.active=true;observer.haveRow=observer.havePrevious=false;
            observer.index=observer.part=0U;
        }
        if(live.sampleSequence<=observer.lastLiveSample||!EDM206LogPeriodReady(observer))return;
        observer.lastLiveSample=live.sampleSequence;
        const auto& header=observer.header;
        if(!header.session||!header.count||header.count>EDM206::Capacity||header.capacity!=EDM206::Capacity||
            (header.mode!=3U&&header.mode!=4U)||!header.faultTick||header.firstTick>header.lastTick||
            header.lastTick!=header.faultTick||!header.zeroImageAccepted||header.containmentTick<header.faultTick) {
            EndEDM206History(motion,"HEADER_INVALID");return;
        }
        const auto u=[](std::uint64_t value) noexcept {return static_cast<unsigned long long>(value);};
        // At most two bounded lines per ten milliseconds, even when several
        // wait callbacks call ServiceEDM94SameThread in the same NC scan.
        for(unsigned budget=0U;budget<2U&&observer.active;++budget) {
            if(observer.part==0U) {
                PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_BEGIN session=%llu count=%u capacity=%u totalRows=%llu firstTick=%llu lastTick=%llu faultTick=%llu faultUs=%llu reason=%u axis=%u mode=%u partial=%u clock=SOURCE_OBSERVATION callbackTiming=EXISTING_TICK_LOG driveAck=0\n",
                    u(header.session),header.count,header.capacity,u(header.totalRows),u(header.firstTick),u(header.lastTick),
                    u(header.faultTick),u(header.faultUs),header.reason,header.axis,header.mode,header.faultRowPartial?1U:0U);
                ++observer.part;continue;
            }
            if(observer.part==1U) {
                PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_SCOPE session=%llu site=%u containmentTick=%llu containmentUs=%llu containment=ZERO_IMAGE_ACCEPTED liveCommandIdle=1 physicalStopProven=NOT_ASSERTED capture=XYZ_INPUT_TO_LOCAL inputValid=CONSUMED_DECODED csv=BEFORE_GROUP_GUARD local=PRE_GUARD_PI prior=NIC_ACCEPTED raw606CUnits=UNVERIFIED_NATIVE_RAW maxLinesPer10ms=2 authority=NONE\n",
                    u(header.session),header.site,u(header.containmentTick),u(header.containmentSourceUs));
                ++observer.part;continue;
            }
            if(observer.part<5U) {
                const unsigned axis=observer.part-2U;
                const auto& config=header.config[axis];
                const bool valid=(header.configMask&(1U<<axis))&&
                    config.sampleTick==header.faultTick&&config.sampleUs==header.faultUs;
                PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_CONFIG session=%llu axis=%c valid=%u sampleTick=%llu sampleUs=%llu slot=%d axisIndex=%d slaveIndex=%d fbMode=%d axisType=%d reverse=%u axisReverse=%u ppr=%.17g lead=%.17g offset=%.17g inputOffset=%d inputMap=%u outputOffset=%d outputMap=%u point=FAULT_SAMPLE\n",
                    u(header.session),"XYZ"[axis],valid?1U:0U,u(config.sampleTick),u(config.sampleUs),
                    static_cast<int>(config.slot),static_cast<int>(config.axisIndex),static_cast<int>(config.slaveIndex),
                    static_cast<int>(config.fbMode),static_cast<int>(config.axisType),config.isReverse?1U:0U,config.axisReverse?1U:0U,
                    config.resolutionPPR,config.finalLead,config.machineOffsetPulse,static_cast<int>(config.inputOffset),
                    (valid&&(config.flags&EDM206::InputOffsetValid)&&config.inputOffset>=0)?1U:0U,static_cast<int>(config.outputOffset),
                    (valid&&(config.flags&EDM206::OutputOffsetValid)&&config.outputOffset>=0)?1U:0U);
                ++observer.part;continue;
            }
            if(observer.index>=header.count){EndEDM206History(motion,"COMPLETE");return;}
            if(!observer.haveRow) {
                if(!motion.ReadEDM206HistoryRow(header.session,observer.index,observer.row)) {
                    EndEDM206History(motion,"ROW_UNAVAILABLE");return;
                }
                if(observer.row.session!=header.session||!observer.row.tick||
                    observer.row.tick<header.firstTick||observer.row.tick>header.lastTick||
                    (observer.havePrevious&&observer.row.tick<=observer.previous.tick)) {
                    EndEDM206History(motion,"ROW_IDENTITY_INVALID");return;
                }
                observer.haveRow=true;
            }
            const auto& row=observer.row;
            const unsigned part=observer.part-5U;
            if(part==0U) {
                const auto& accepted=row.priorAccepted;
                PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_ROW session=%llu row=%u tick=%llu sourceUs=%llu sourceDeltaUs=%llu epoch=%llu ownerGen=%u phase=%u child=%u mode=%u flags=%u controlMask=%u writtenMask=%u inputMask=%u priorValid=%u priorTick=%llu priorUs=%llu priorGen=%llu csvX=%d csvY=%d csvZ=%d zero=%u\n",
                    u(row.session),observer.index,u(row.tick),u(row.sourceUs),u(row.sourceDeltaUs),u(row.epoch),
                    row.ownerGeneration,row.phase,row.child,row.mode,row.flags,row.controlMask,row.writtenMask,row.inputMask,
                    accepted.valid?1U:0U,u(accepted.tick),u(accepted.sourceUs),u(accepted.frameGeneration),
                    static_cast<int>(accepted.csv[0]),static_cast<int>(accepted.csv[1]),static_cast<int>(accepted.csv[2]),accepted.allSixZero?1U:0U);
            } else {
                const unsigned axis=(part-1U)/2U;const auto& value=row.axis[axis];
                if((part&1U)!=0U) {
                    double actualPps=0.0;const bool velocityValid=EDM206ActualVelocity(observer,axis,actualPps);
                    PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_AXIS session=%llu row=%u axis=%c flags=%u cmd=%.17g act=%.17g err=%.17g ppu=%.17g strict=%.17g ref=%.17g actualPps=%.17g actualValid=%u writtenTick=%llu csv=%d local=%.17g\n",
                        u(row.session),observer.index,"XYZ"[axis],value.flags,value.frameCommandPulse,value.frameActualPulse,
                        value.followingErrorPulse,value.pulsePerUnit,value.strictWindowPulse,value.committedReferencePps,
                        actualPps,velocityValid?1U:0U,u(value.writtenTick),static_cast<int>(value.writtenCsv),value.control.logicalOutputVelocityPps);
                } else {
                    const auto& input=value.input;
                    PrintNCG76DiagnosticLine("[EDM207] event=HISTORY_INPUT session=%llu row=%u axis=%c valid=%u flags=%u source=%s inputTick=%llu inputUs=%llu raw6064=%d used6064=%d prevRaw=%u delta=%d unwrap=%.17g rawLogical=%.17g preFixture=%.17g raw606C=%d raw2510=%u\n",
                        u(row.session),observer.index,"XYZ"[axis],EDM207InputCurrent(row,axis)?1U:0U,static_cast<unsigned>(input.flags),
                        EDM207InputSourceName(input.source),u(input.tick),u(input.sourceUs),static_cast<int>(input.mappedRaw),
                        static_cast<int>(input.consumedRaw),static_cast<unsigned>(input.previousRaw),static_cast<int>(input.pulseDelta),
                        input.unwrappedPulse,input.rawLogicalPulse,input.preFixtureActualPulse,static_cast<int>(input.rawVelocity),static_cast<unsigned>(input.rawObject2510));
                }
            }
            ++observer.part;
            if(observer.part==12U) {
                observer.previous=observer.row;observer.havePrevious=true;observer.haveRow=false;
                ++observer.index;observer.part=5U;
            }
        }
    }
    // EDM206_NC_HISTORY_END

namespace
{

    // All G76 profiles. NC owns this observer; it neither writes RT state nor grants
    // readiness. A failed pre-submit floor read permits logging only.
    struct EDM192AdmissionObserver
    {
        EDM192Arm::Scope scope{};
        std::uint64_t floor=0ULL,lastPublication=0ULL;
        bool active=false,floorKnown=false;
    };
    EDM192AdmissionObserver g_edm192AdmissionObserver{};
}

    void PrimeEDM192AdmissionObserver(const EDM192Arm::Scope& scope) noexcept
    {
        auto& observer=g_edm192AdmissionObserver;
        observer=EDM192AdmissionObserver{};observer.scope=scope;observer.active=true;
        EDM192Arm::Snapshot previous{};std::uint64_t publication=0ULL;
        observer.floorKnown=EDM192Arm::Read(previous,publication);
        observer.floor=observer.floorKnown?publication:0ULL;
        observer.lastPublication=observer.floor;
    }
    bool PollEDM192AdmissionObserver(const EDM192Arm::Scope& scope) noexcept
    {
        auto& observer=g_edm192AdmissionObserver;
        const auto& request=observer.scope;
        if(!observer.active||request.session!=scope.session||request.armEpoch!=scope.armEpoch||
            request.liveEpoch!=scope.liveEpoch||request.owner!=scope.owner||
            request.ownerGeneration!=scope.ownerGeneration||request.fixture!=scope.fixture||request.profile!=scope.profile)
            return false;
        EDM192Arm::Snapshot receipt{};std::uint64_t publication=0ULL;
        if(!EDM192Arm::Read(receipt,publication)||!EDM192Arm::MatchesScope(receipt,scope)||
            !EDM192Arm::IsNewPublication(publication,observer.floor,observer.lastPublication))return false;
        observer.lastPublication=publication;
        const bool recipeMatch=EDM192Arm::MatchesRecipe(receipt,scope);
        // A recipe mismatch must remain visible, including a shape rejection.
        // Only a correlated terminal rejection can cancel this pending arm;
        // Consumed/Accepted are observations and cannot make NC ready.
        const bool reject=observer.floorKnown&&EDM192Arm::IsReject(receipt.stage);
        PrintNCG76DiagnosticLine("[EDM192_FIX1] event=ARM_ADMISSION stage=%s cause=%s session=%llu epoch=%llu ownerGen=%u fixture=%u profile=%u expected=%u/%u tick=%llu nowUs=%llu pub=%llu floorKnown=%u recipeMatch=%u axis=%u pid=%s reason=%u requestBits=%u action=%s authority=NONE\n",
            EDM192Arm::StageName(receipt.stage),EDM192Arm::CauseName(receipt.cause),
            static_cast<unsigned long long>(receipt.session),static_cast<unsigned long long>(receipt.armEpoch),
            receipt.ownerGeneration,receipt.fixture,receipt.profile,scope.fixture,scope.profile,
            static_cast<unsigned long long>(receipt.tick),static_cast<unsigned long long>(receipt.nowUs),
            static_cast<unsigned long long>(publication),observer.floorKnown?1U:0U,recipeMatch?1U:0U,
            receipt.axis,EDM192Arm::PidName(receipt.pid),receipt.engineReason,receipt.requestFailures,
            reject?"REJECT":"LOG_ONLY");
        if(receipt.stage==EDM192Arm::Stage::HeadroomRejected&&receipt.axis<EDM192Arm::NoAxis)
            PrintNCG76DiagnosticLine("[EDM192_FIX1] event=ARM_HEADROOM session=%llu axis=%u pid=%s ppu=%.12g windowPulse=%.12g maxVelPps=%.12g capPps=%.12g Kp=%.12g Ki=%.12g Kd=%.12g Kvff=%.12g maxI=%.12g referenceMmSec=%.12g units=PULSE_BASED output=NONE\n",
                static_cast<unsigned long long>(receipt.session),receipt.axis,EDM192Arm::PidName(receipt.pid),
                receipt.pulsePerUnit,receipt.followingWindowPulse,receipt.maxVelocityPps,receipt.serializedCapPps,
                receipt.kp,receipt.ki,receipt.kd,receipt.kvff,receipt.maxIntegral,EDM94::VisibleFlushReferenceMmSec);
        return reject;
    }

    // FIX7: NC prints the RT-owned immutable input comparison after a fault.
    // Raw means mapped host process-image bytes, never a drive alarm readback.
    void PrintEDM173InputFailure(const MotionCore& motion,std::uint64_t session,
        std::uint64_t guardTick) noexcept
    {
        EDM173InputDiag::Evidence evidence{};
        if (!motion.ReadEDM173InputDiagnostic(evidence) || !evidence.haveFirst ||
            evidence.session!=session || evidence.first.tick>guardTick)
        {
            RtPrintf("[EDM173_FIX7] event=SERVO_INPUT_FIRST session=%llu guardTick=%llu evidence=UNAVAILABLE authority=NONE\n",
                static_cast<unsigned long long>(session),static_cast<unsigned long long>(guardTick));
            return;
        }
        char record[481]{};
        const auto u=[](std::uint64_t value) noexcept {return static_cast<unsigned long long>(value);};
        const auto emit=[&record](int length) noexcept {
            if (length>0 && length<=480 && static_cast<std::size_t>(length)<sizeof(record)) RtPrintf("%s",record);
            else RtPrintf("[EDM173_FIX7] event=LOG_FORMAT_ERROR scope=SERVO_INPUT\n");
        };
        emit(std::snprintf(record,sizeof(record),
            "[EDM173_FIX7] event=SERVO_INPUT_FIRST session=%llu inputTick=%llu guardTick=%llu previousTick=%llu frames=%llu raw=HOST_PROCESS_IMAGE capture=AROUND_AXIS_UPDATE authority=NONE driveAck=0 flags=SERVO_ON1_FAULT2_LAG4_RESET8\n",
            u(session),u(evidence.first.tick),u(guardTick),u(evidence.havePrevious?evidence.previous.tick:0ULL),u(evidence.frames)));
        const EDM173InputDiag::Frame* frames[2]={&evidence.previous,&evidence.first};
        for (unsigned index=evidence.havePrevious?0U:1U;index<2U;++index)
        {
            const auto& frame=*frames[index];
            const char* phase=index?"FIRST":"PREVIOUS";
            emit(std::snprintf(record,sizeof(record),
                "[EDM173_FIX7] event=SERVO_INPUT_FRAME session=%llu frame=%s tick=%llu nowUs=%llu epoch=%llu complete=%u input=%02X raw=%02X semantic=%02X mismatch=%02X notReady=%02X internal=%02X trigger=%02X\n",
                u(session),phase,u(frame.tick),u(frame.nowUs),u(frame.epoch),frame.complete?1U:0U,
                frame.inputMask,frame.rawMask,frame.semanticMask,frame.rawMismatchMask,frame.notReadyMask,frame.internalFaultMask,frame.triggerMask));
            for (unsigned axis=0U;axis<6U;++axis)
            {
                if (!(frame.inputMask&(1U<<axis))) continue;
                const auto& a=frame.axis[axis];
                emit(std::snprintf(record,sizeof(record),
                    "[EDM173_FIX7] event=SERVO_INPUT_AXIS session=%llu frame=%s tick=%llu axis=%c usedSW=%04X rawSW=%04X usedMode=%d rawMode=%d usedPos=%d rawPos=%d rawValid=%u semantic=%u outputValid=%u beforeState=%d afterState=%d beforeFlags=%02X afterFlags=%02X cwBefore=%04X cwAfter=%04X\n",
                    u(session),phase,u(frame.tick),"XYZCUV"[axis],static_cast<unsigned>(a.consumedStatus),static_cast<unsigned>(a.rawStatus),
                    static_cast<int>(a.consumedMode),static_cast<int>(a.rawMode),static_cast<int>(a.consumedPosition),static_cast<int>(a.rawPosition),
                    a.rawValid?1U:0U,a.semantic?1U:0U,a.outputValid?1U:0U,static_cast<int>(a.stateBefore),static_cast<int>(a.stateAfter),
                    static_cast<unsigned>(a.flagsBefore),static_cast<unsigned>(a.flagsAfter),static_cast<unsigned>(a.controlBefore),static_cast<unsigned>(a.controlAfter)));
            }
        }
    }

    void PrintEDM145SourceFailure(std::uint64_t session) noexcept
    {
        static std::uint64_t lastSession=0ULL;
        if(!session || session==lastSession)return;
        EDM145::SourceFailure r{};
        if(!EDM145::ReadSourceFailure(r) || r.session!=session)return;
        lastSession=session;
        char record[481]{};
        const auto u=[](std::uint64_t value) noexcept {return static_cast<unsigned long long>(value);};
        const auto emit=[&record](int length) noexcept {
            if(length>0 && length<=480 && static_cast<std::size_t>(length)<sizeof(record))RtPrintf("%s",record);
            else RtPrintf("[EDM145] event=LOG_FORMAT_ERROR scope=SOURCE_FIRST\n");
        };
        emit(std::snprintf(record,sizeof(record),
            "[EDM145] event=SOURCE_FIRST part=DECISION session=%llu tick=%llu sourceCurrent=%u awaitEvaluated=%u awaitCurrent=%u mapped=%u awaitGate=%s snapshot=PRE_TRIP authority=NONE\n",
            u(r.session),u(r.tick),(r.decisionFlags&EDM145::SourceCurrent)?1U:0U,
            (r.decisionFlags&EDM145::AwaitEvaluated)?1U:0U,(r.decisionFlags&EDM145::AwaitCurrent)?1U:0U,
            (r.decisionFlags&EDM145::MappingCurrent)?1U:0U,EDM145::AwaitGateName(r.awaitFailure)));
        emit(std::snprintf(record,sizeof(record),
            "[EDM145] event=SOURCE_FIRST part=CLOCK session=%llu tick=%llu sendUs=%llu frameUs=%llu physicalUs=%llu sourceCaptureUs=%llu baselineCaptureUs=%llu baselineObserveUs=%llu admittedUs=%llu\n",
            u(r.session),u(r.tick),u(r.sendNowUs),u(r.frameNowUs),u(r.physicalNowUs),u(r.sourceCapturedUs),
            u(r.baselineCapturedUs),u(r.baselineObservedUs),u(r.admittedUs)));
        emit(std::snprintf(record,sizeof(record),
            "[EDM145] event=SOURCE_FIRST part=AGE session=%llu tick=%llu frameAgeUs=%llu sourceAgeUs=%llu baselineAgeUs=%llu baselineObserveAgeUs=%llu sourceMaxUs=%llu baselineMaxUs=%llu shadowMaxUs=%llu shadowTickMaxUs=%llu unknownAge=UINT64_MAX\n",
            u(r.session),u(r.tick),u(EDM145::AgeUs(r.sendNowUs,r.frameNowUs)),
            u(EDM145::AgeUs(r.sendNowUs,r.sourceCapturedUs)),u(EDM145::AgeUs(r.sendNowUs,r.baselineCapturedUs)),
            u(EDM145::AgeUs(r.sendNowUs,r.baselineObservedUs)),u(r.sourceMaxAgeUs),u(r.baselineMaxAgeUs),
            u(r.shadowMaxAgeUs),u(r.shadowMaxTickGapUs)));
        emit(std::snprintf(record,sizeof(record),
            "[EDM145] event=SOURCE_FIRST part=STATE session=%llu tick=%llu physicalReason=%s flags=%08X positionMismatch=%02X velocityMismatch=%02X expectedEpoch=%llu liveEpoch=%llu observedEpoch=%llu publication=%llu cycle=%llu fresh=%llu held=%llu\n",
            u(r.session),u(r.tick),EDM129::SourceReasonName(static_cast<EDM129::SourceReason>(r.physicalReason)),
            static_cast<unsigned>(r.stateFlags),static_cast<unsigned>(r.positionMismatchMask),
            static_cast<unsigned>(r.velocityMismatchMask),u(r.expectedEpoch),u(r.liveEpoch),u(r.observedEpoch),
            u(r.sourcePublication),u(r.sourceCycle),u(r.sourceFresh),u(r.sourceHeld)));
    }

}
