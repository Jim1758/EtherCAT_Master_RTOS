#pragma once
#include <cmath>
#include <cstdint>
#include "EDMZGapCurveFixture.h"
#include "EDMZGapReplanFixture.h"
#include "EDMZGapShortReplanFixture.h"
#include "EDMZGapFeedFixture.h"
#include "EDMZGapRapidFixture.h"
#include "EDMZGapEnduranceFixture.h"
#include "EDMExecutionCommand.h"

// EDM28/EDM29/EDM30/EDM31/EDM32/EDM34/EDM35/EDM36 local, unloaded single-Z policy. This computes no trajectory and
// performs no I/O. The RT adapter alone invokes the existing finite-position
// planner, stop primitive and scoped final PDO cap. No home or global EDM
// permit is granted. LOCAL_RT_APPLIED is not a drive/NIC acknowledgement.
namespace EDM28
{
    constexpr std::uint64_t CycleUs=250U, HeartbeatTicks=200U;
    constexpr std::uint32_t SettleCycles=200U;
    // P14 is a separate, fixed opt-in speed step. It is never a promotion of
    // P13 by F, feed override, installed COND, PID or AD parameters.
    enum class Profile:std::uint8_t { P13Unloaded=13U,P14UnloadedSpeed=14U,P15UnloadedRepeat=15U,P16UnloadedRetreat=16U,P17PersistentShort=17U,P18UnloadedSpeedRetreat=18U,P19UnloadedSpeedRetreat=19U,P20UnloadedSpeedRetreat=20U,P21GapShortRetreat=21U,P22GapPersistentShort=22U,P23GapCurveSegments=23U,P24GapCurveReplan=24U,P25GapShortReplan=25U,P26GapPersistentReplan=26U,P27GapFeedUpdate=27U,P28GapFeedPersistent=28U,P29GapRapid=29U,P30GapRapidPersistent=30U,P31GapEndurance=31U,P32GapEndurancePersistent=32U };
    constexpr bool IsGapEnduranceProfile(Profile profile)noexcept
    {return profile==Profile::P31GapEndurance||profile==Profile::P32GapEndurancePersistent;}
    constexpr bool IsGapRapidPersistentProfile(Profile profile)noexcept
    {return profile==Profile::P30GapRapidPersistent||profile==Profile::P32GapEndurancePersistent;}
    constexpr bool IsGapRapidProfile(Profile profile)noexcept
    {return profile==Profile::P29GapRapid||profile==Profile::P30GapRapidPersistent||IsGapEnduranceProfile(profile);}
    // Counts belong to the explicit profile; the negative persistent profiles
    // stop at the first low-voltage origin and never earn a normal run pass.
    constexpr std::uint32_t RapidCycleCount(Profile profile)noexcept
    {return IsGapEnduranceProfile(profile)?EDM44::CycleCount:EDM43::CycleCount;}
    constexpr std::uint64_t RapidWindowTimeoutMs(Profile profile)noexcept
    {return profile==Profile::P31GapEndurance?EDM44::WindowTimeoutMs:30000U;}
    constexpr bool IsGapFeedUpdateProfile(Profile profile)noexcept
    {return profile==Profile::P27GapFeedUpdate||profile==Profile::P28GapFeedPersistent;}
    constexpr bool IsGapShortReplanProfile(Profile profile)noexcept
    {return profile==Profile::P25GapShortReplan||profile==Profile::P26GapPersistentReplan;}
    constexpr bool IsGapCurveProfile(Profile profile)noexcept
    {return profile==Profile::P23GapCurveSegments||profile==Profile::P24GapCurveReplan||IsGapShortReplanProfile(profile)||IsGapFeedUpdateProfile(profile)||IsGapRapidProfile(profile);}
    constexpr bool IsSpeedRetreatProfile(Profile profile)noexcept
    {return profile==Profile::P18UnloadedSpeedRetreat||profile==Profile::P19UnloadedSpeedRetreat||profile==Profile::P20UnloadedSpeedRetreat||profile==Profile::P21GapShortRetreat;}
    constexpr double ProfileFeedMmMin(Profile profile)noexcept
    {if(IsGapRapidProfile(profile))return EDM43::FeedMmMin;
     if(profile==Profile::P20UnloadedSpeedRetreat||profile==Profile::P21GapShortRetreat||IsGapCurveProfile(profile))return 20.0;
     if(profile==Profile::P19UnloadedSpeedRetreat)return 15.0;
     if(profile==Profile::P18UnloadedSpeedRetreat)return 10.0;
     return profile==Profile::P13Unloaded?2.5:(profile==Profile::P14UnloadedSpeed||profile==Profile::P15UnloadedRepeat||profile==Profile::P16UnloadedRetreat||profile==Profile::P17PersistentShort||profile==Profile::P22GapPersistentShort)?5.0:0.0;}
    constexpr std::uint32_t ProfileMinimumCycles(Profile profile)noexcept
    {if(profile==Profile::P31GapEndurance)return EDM44::CycleCount;
     if(IsSpeedRetreatProfile(profile)||profile==Profile::P23GapCurveSegments||profile==Profile::P24GapCurveReplan||profile==Profile::P25GapShortReplan||profile==Profile::P27GapFeedUpdate||profile==Profile::P29GapRapid)return 6U;
     return (profile==Profile::P17PersistentShort||profile==Profile::P22GapPersistentShort)?0U:profile==Profile::P13Unloaded?2U:profile==Profile::P14UnloadedSpeed?4U:(profile==Profile::P15UnloadedRepeat||profile==Profile::P16UnloadedRetreat)?6U:0U;}
    constexpr double ProfilePdoCapMmS(Profile profile)noexcept
    {return IsGapRapidProfile(profile)?EDM43::PdoCapMmS:(profile==Profile::P20UnloadedSpeedRetreat||profile==Profile::P21GapShortRetreat||IsGapCurveProfile(profile))?.35:profile==Profile::P19UnloadedSpeedRetreat?.3:profile==Profile::P18UnloadedSpeedRetreat?.2:
        (profile==Profile::P13Unloaded||profile==Profile::P14UnloadedSpeed||profile==Profile::P15UnloadedRepeat||profile==Profile::P16UnloadedRetreat||profile==Profile::P17PersistentShort||profile==Profile::P22GapPersistentShort)?.1:0.0;}
    enum class RequestKind:std::uint8_t { Arm,Position,Heartbeat,Stop,Disarm,FeedUpdate=5U };
    enum class State:std::uint8_t { Idle,Armed,Moving,AtTarget,Stopping,Held,Disarmed,Fault };
    enum class Reason:std::uint8_t
    { None,InvalidConfig,InvalidScope,InvalidRequest,Sequence,IssueClock,StaleRequest,
      NotReady,Busy,FrameChanged,AuthorityLost,InvalidSource,RuntimeGap,Clock,
      Deadman,OuterBound,FollowingError,CompetingMotion,HardLimit,ApplyFailed,
      AwaitStop,StopTimeout,NativeRange,NativeSpeed,NotAtTarget,CancelledBeforeApply };
    enum class Ack:std::uint8_t { None,Accepted,Rejected,LocalRtApplied };
    enum class Action:std::uint8_t { None,Move,Stop,Fault,FeedUpdate=4U };
    struct Scope
    {
        std::uint64_t session=0U,runGeneration=0U,cacheGeneration=0U,dispatchGeneration=0U;
        std::uint64_t axisMapGeneration=0U,configGeneration=0U;
        std::uint32_t executionEpoch=0U,ownerGeneration=0U;
        std::uint8_t owner=0U;
    };
    inline bool SameScope(const Scope&a,const Scope&b) noexcept
    {return a.session==b.session&&a.runGeneration==b.runGeneration&&a.cacheGeneration==b.cacheGeneration&&
        a.dispatchGeneration==b.dispatchGeneration&&a.axisMapGeneration==b.axisMapGeneration&&
        a.configGeneration==b.configGeneration&&a.executionEpoch==b.executionEpoch&&
        a.ownerGeneration==b.ownerGeneration&&a.owner==b.owner;}
    inline bool ValidScope(const Scope&s) noexcept
    {return s.session&&s.runGeneration&&s.cacheGeneration&&s.dispatchGeneration&&
        s.axisMapGeneration&&s.configGeneration&&s.executionEpoch&&s.ownerGeneration&&s.owner==1U;}
    struct Config
    {
        std::uint32_t axisIndex=2U;
        double pulsePerMm=0.0,referenceOffsetPulse=0.0;
        int hardwareSign=1;
        double outerHalfMm=.1,targetHalfMm=.02,feedMmMin=2.5,pdoCapMmS=.1;
        double accelerationTimeSec=.2,decelerationTimeSec=.2;
        double followingLimitMm=.02,positionToleranceMm=.001,excursionToleranceMm=.001;
        std::uint64_t heartbeatTicks=HeartbeatTicks,maximumIssueAgeTicks=80U,stopTimeoutTicks=4000U;
        std::uint32_t settleCycles=SettleCycles;
        Profile profile=Profile::P13Unloaded; // Frozen with the first Arm, including HOLD re-arm.
    };
    struct Request
    {
        RequestKind kind=RequestKind::Heartbeat;
        Scope scope{};
        std::uint64_t sequence=0U,issueTick=0U,issueMonotonicUs=0U;
        Config config{}; // Arm only; later requests cannot alter frozen config.
        double relativeTargetMm=0.0; // Always relative to the FIRST arm origin.
        // P23..P32 Position, plus P27/P28 FeedUpdate: source evidence and signed curve rate. RT derives
        // its own native speed; config.feedMmMin is only the frozen ceiling.
        std::uint64_t curveSampleTick=0U,curveSampleUs=0U;
        double curveVoltage=0.0,signedCurveMmMin=0.0;
    };
    struct Observation
    {
        std::uint64_t tick=0U,monotonicUs=0U;
        std::uint64_t axisMapGeneration=0U,configGeneration=0U;
        std::uint32_t currentEpoch=0U,currentOwnerGeneration=0U;
        std::uint8_t currentOwner=0U;
        std::uint16_t statusWord=0U;
        std::int8_t modeValue=0;
        double pulsePerMm=0.0,referenceOffsetPulse=0.0,maximumVelocityPps=0.0,feedrateOverride=1.0;
        int hardwareSign=1;
        double actualPulse=0.0,commandPulse=0.0,planningPulse=0.0;
        double cmdVelocityPps=0.0,logicalVelocityPps=0.0,targetVelocityPps=0.0;
        double actualVelocityPps=0.0,pdoVelocityPps=0.0;
        bool pdoValid=false,contiguous=false,clockValid=false;
        bool axisExists=false,linear=false,axisIdle=false,servoReady=false,modeReady=false,fault=false;
        bool allOtherAxesIdle=false,groupQuiet=false,axisMailboxQuiet=false;
        bool noCompensation=false,motorEncoderSource=false;
        bool homeFact=false,softLimitsFact=false,hardPositive=false,hardNegative=false;
    };
    struct Decision
    {
        Action action=Action::None;
        bool accepted=false,requiresCommit=false,capHeld=false,stopLatched=false,forceZeroOutput=false;
        double nativeTargetPulse=0.0,nativeVelocityPps=0.0,accelerationTimeSec=.2,decelerationTimeSec=.2;
        // EDM51 binds the actuator call to this policy's existing request and
        // phase. These are local application identities, not drive receipts.
        std::uint64_t requestSession=0U,requestSequence=0U,applicationTick=0U,applicationUs=0U;
    };
    struct Feedback
    {
        Scope scope{};Config frozenConfig{};
        State state=State::Idle;Reason reason=Reason::None;Ack ack=Ack::None;
        RequestKind lastAppliedKind=RequestKind::Arm;
        std::uint64_t publicationSequence=0U,sampledTick=0U,monotonicUs=0U;
        std::uint64_t lastRequestSequence=0U,lastAppliedSequence=0U,lastAppliedTick=0U,proofSequence=0U;
        std::uint64_t stopAppliedSequence=0U,stopAppliedTick=0U;
        std::uint64_t lastAppliedMonotonicUs=0U,stopAppliedMonotonicUs=0U;
        std::uint64_t axisMapGeneration=0U,configGeneration=0U;
        std::uint32_t currentEpoch=0U,currentOwnerGeneration=0U,stableCycles=0U;
        std::uint8_t currentOwner=0U;
        double pulsePerMm=0.0,referenceOffsetPulse=0.0,outerOriginPulse=0.0,targetPulse=0.0,feedrateOverride=1.0;
        int hardwareSign=1;
        double actualMm=0.0,commandMm=0.0,cmdSpeedMmS=0.0,pdoSpeedMmS=0.0;
        double actualPulse=0.0,commandPulse=0.0,planningPulse=0.0,actualVelocityPps=0.0;
        std::uint16_t statusWord=0U;std::int8_t modeValue=0;
        bool sourceFresh=false,pdoValid=false,contiguous=false,clockValid=false;
        bool axisExists=false,linear=false,axisIdle=false,servoReady=false,modeReady=false,fault=false;
        bool homeFact=false,softLimitsFact=false,hardPositive=false,hardNegative=false;
        bool armReady=false,capHeld=false,stopLatched=false,forceZeroOutput=false,latchedFault=false,frameCurrent=false;
        bool originValid=false,stopProven=false,targetProven=false,queuedCancellation=false;
        // Last applied curve Position or P27/P28 FeedUpdate survives Stop/HOLD/Disarm.
        // Its sequence identifies that exact application; a later Stop is not a curve ack.
        std::uint64_t lastCurveSequence=0U,lastCurveSampleTick=0U,lastCurveSampleUs=0U;
        double lastCurveVoltage=0.0,lastSignedCurveMmMin=0.0;
        constexpr bool GlobalEDMPhysicalPermit()const noexcept{return false;}
        constexpr bool PhysicalDischargeEnabled()const noexcept{return false;}
    };
    class RTPolicy
    {
    public:
        // Whole-program teardown only, after real stopped Disarm. HOLD/re-arm
        // keeps the existing object and its immutable first-arm anchor.
        bool Reset() noexcept
        {
            if(Active())return false; // Cannot drop an armed/stopping PDO cap.
            const std::uint64_t retired=retiredSession_,tick=phaseTick_,us=phaseUs_;
            const bool seen=havePhase_;
            *this=RTPolicy{};retiredSession_=retired;phaseTick_=tick;phaseUs_=us;havePhase_=seen;
            haveCycle_=seen;lastCycleTick_=tick;lastClockUs_=us;return true;
        }
        const Feedback& Snapshot()const noexcept{return f_;}
        Decision OnRequest(const Request&r,const Observation&o) noexcept
        {
            f_.ack=Ack::Rejected;
            if(!Phase(o))return Active()?Trip(Reason::Clock,o):Reject(Reason::Clock);
            if(haveCycle_&&(o.tick>lastCycleTick_&&o.tick-lastCycleTick_>1U))
                return Active()?Trip(Reason::RuntimeGap,o):Reject(Reason::RuntimeGap);
            if(haveCycle_&&o.monotonicUs>lastClockUs_&&o.monotonicUs-lastClockUs_>50000U)
                return Active()?Trip(Reason::Clock,o):Reject(Reason::Clock);
            if(!Source(o)){if(!f_.latchedFault)f_.reason=Reason::InvalidSource;return Active()?Trip(Reason::InvalidSource,o):View();}
            if(Active())
            {
                f_.frameCurrent=Frame(o);
                if(!f_.frameCurrent||frameLost_)
                {frameLost_=true;f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
                 f_.state=State::Stopping;return Trip(Reason::FrameChanged,o);}
            }
            InvalidateReceipt(o);
            if(pendingValid_&&r.kind!=RequestKind::Stop)return Active()?Trip(Reason::Busy,o):Reject(Reason::Busy);
            if(!((r.kind==RequestKind::Position&&IsGapCurveProfile(f_.frozenConfig.profile))||
                (r.kind==RequestKind::FeedUpdate&&IsGapFeedUpdateProfile(f_.frozenConfig.profile)))&&!EmptyCurve(r))
                return Active()?Trip(Reason::InvalidRequest,o):Reject(Reason::InvalidRequest);
            if(r.kind==RequestKind::Arm)return Arm(r,o);
            if(!Active()||!SameScope(r.scope,f_.scope))
            {if(!f_.latchedFault)f_.reason=Reason::InvalidScope;return Active()&&r.scope.session==f_.scope.session?Trip(Reason::InvalidScope,o):View();}
            if(static_cast<unsigned>(r.kind)>static_cast<unsigned>(RequestKind::FeedUpdate)||
                (r.kind==RequestKind::FeedUpdate&&!IsGapFeedUpdateProfile(f_.frozenConfig.profile)))return Trip(Reason::InvalidRequest,o);
            if(!RequestFresh(r,o))return Trip(f_.reason,o);
            if(r.kind==RequestKind::Stop)return LatchStop(r,o);
            if(r.kind==RequestKind::Disarm)
            {
                if(frameLost_||!Frame(o)||!f_.stopProven||f_.state!=State::Held){if(!f_.latchedFault)f_.reason=Reason::AwaitStop;return View();}
                Seal(r,o);Decision d=View();d.accepted=d.requiresCommit=true;return BindApplication(d);
            }
            if(f_.stopLatched||f_.state==State::Fault){if(!f_.latchedFault)f_.reason=Reason::AwaitStop;return View();}
            if(o.tick>=deadlineTick_||o.monotonicUs>=deadlineUs_)return Trip(Reason::Deadman,o);
            if(!Authority(o))return Trip(Reason::AuthorityLost,o);
            if(!Frame(o)||frameLost_){frameLost_=true;return Trip(Reason::FrameChanged,o);}
            if(r.kind==RequestKind::Heartbeat)
            {deadlineTick_=r.issueTick+f_.frozenConfig.heartbeatTicks;deadlineUs_=r.issueMonotonicUs+50000U;
             f_.ack=Ack::Accepted;Decision d=View();d.accepted=true;return d;}
            if(r.kind==RequestKind::FeedUpdate)
            {
                double velocity=0.0;
                const Reason reason=FeedUpdateRequest(r,o,velocity);
                if(reason!=Reason::None)return Trip(reason,o);
                Seal(r,o);f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
                lastCurveAcceptedTick_=r.curveSampleTick;lastCurveAcceptedUs_=r.curveSampleUs;
                Decision d=View();d.action=Action::FeedUpdate;d.accepted=d.requiresCommit=true;
                d.nativeTargetPulse=f_.targetPulse;d.nativeVelocityPps=velocity;
                d.accelerationTimeSec=f_.frozenConfig.accelerationTimeSec;d.decelerationTimeSec=f_.frozenConfig.decelerationTimeSec;
                return BindApplication(d);
            }
            if(f_.state!=State::Armed&&(f_.state!=State::AtTarget||!f_.targetProven)){if(!f_.latchedFault)f_.reason=Reason::Busy;return View();}
            const Config&commandConfig=f_.frozenConfig;
            EDM50::TargetRequest command{};
            command.axis.backend=EDM50::Backend::LocalRTMotion; command.axis.unit=EDM50::AxisUnit::Millimeter;
            command.axis.axisIndex=static_cast<std::uint8_t>(commandConfig.axisIndex);
            command.axis.nativePerUnit=commandConfig.pulsePerMm;
            command.firstOriginNative=f_.outerOriginPulse; command.relativeTargetUnits=r.relativeTargetMm;
            command.targetHalfUnits=commandConfig.targetHalfMm; command.outerHalfUnits=commandConfig.outerHalfMm;
            const EDM50::TargetResult translated=EDM50::TranslateTarget(command);
            if(!translated.Ready())return Trip(translated.reason==EDM50::Reason::NativeRange?Reason::NativeRange:Reason::InvalidRequest,o);
            const double target=translated.nativeTarget;
            double velocity=EDM50::ConvertRate(commandConfig.feedMmMin,commandConfig.pulsePerMm);
            if(IsGapCurveProfile(f_.frozenConfig.profile))
            {
                const Reason curveReason=CurveRequest(r,o,target,velocity);
                if(curveReason!=Reason::None)return Trip(curveReason,o);
            }
            Seal(r,o);f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;
            if(IsGapCurveProfile(f_.frozenConfig.profile))
            {lastCurveAcceptedTick_=r.curveSampleTick;lastCurveAcceptedUs_=r.curveSampleUs;}
            Decision d=View();d.action=Action::Move;d.accepted=d.requiresCommit=true;
            d.nativeTargetPulse=target;d.nativeVelocityPps=velocity;
            d.accelerationTimeSec=f_.frozenConfig.accelerationTimeSec;d.decelerationTimeSec=f_.frozenConfig.decelerationTimeSec;
            return BindApplication(d);
        }
        bool HasPendingApplication()const noexcept{return pendingValid_;}
        Decision PrepareApplication(const Decision&d,const Observation&o) noexcept
        {
            if(!pendingValid_&&d.action==Action::None&&!d.requiresCommit)return View();
            if(pendingValid_&&pending_.kind==RequestKind::Disarm)InvalidateReceipt(o);
            const Reason reason=ApplicationReason(d,o);
            return reason==Reason::None?d:FailApplication(reason,o);
        }
        Decision CommitApplied(const Request&r,const Decision&d,bool success,const Observation&o) noexcept
        {
            // Never consume a newer pending Stop for a stale/mutated Move.
            if(!pendingValid_||!SameRequest(r,pending_))return FailApplication(Reason::InvalidRequest,o);
            if(r.kind==RequestKind::Disarm)InvalidateReceipt(o);
            const Reason applicationReason=ApplicationReason(d,o);
            if(applicationReason!=Reason::None)return FailApplication(applicationReason,o);
            if(!Phase(o))return FailApplication(Reason::Clock,o);
            if(r.kind==RequestKind::Disarm)
            {
                if(!Frame(o)||frameLost_)
                {frameLost_=true;f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
                 f_.state=State::Stopping;return FailApplication(Reason::FrameChanged,o);}
                InvalidateReceipt(o); // No credit; validate the current physical receipt.
                if(!Source(o)||!f_.stopProven||f_.state!=State::Held)
                {return FailApplication(!Source(o)?Reason::InvalidSource:Reason::NotAtTarget,o);}
            }
            if(!success)return FailApplication(Reason::ApplyFailed,o);
            if(r.kind!=RequestKind::Stop&&r.kind!=RequestKind::Disarm&&(!Source(o)||!Authority(o)||!Frame(o)||frameLost_))
            {if(!Frame(o)){frameLost_=true;return FailApplication(Reason::FrameChanged,o);}
             return FailApplication(Reason::AuthorityLost,o);}
            pendingValid_=pendingDecisionValid_=false;
            f_.lastAppliedSequence=r.sequence;f_.lastAppliedKind=r.kind;f_.lastAppliedTick=o.tick;f_.lastAppliedMonotonicUs=o.monotonicUs;
            f_.ack=Ack::LocalRtApplied;if(!f_.latchedFault)f_.reason=Reason::None;
            if(r.kind!=RequestKind::Disarm){f_.stableCycles=0U;f_.stopProven=f_.targetProven=false;haveCandidate_=false;}
            if(r.kind==RequestKind::Position)
            {f_.targetPulse=f_.outerOriginPulse+r.relativeTargetMm*f_.frozenConfig.pulsePerMm;f_.state=State::Moving;
             if(IsGapCurveProfile(f_.frozenConfig.profile))
             {f_.lastCurveSequence=r.sequence;f_.lastCurveSampleTick=r.curveSampleTick;f_.lastCurveSampleUs=r.curveSampleUs;
              f_.lastCurveVoltage=r.curveVoltage;f_.lastSignedCurveMmMin=r.signedCurveMmMin;}}
            else if(r.kind==RequestKind::FeedUpdate)
            {f_.state=State::Moving; // The finite target and first origin remain bit-exact.
             f_.lastCurveSequence=r.sequence;f_.lastCurveSampleTick=r.curveSampleTick;f_.lastCurveSampleUs=r.curveSampleUs;
             f_.lastCurveVoltage=r.curveVoltage;f_.lastSignedCurveMmMin=r.signedCurveMmMin;}
            else if(r.kind==RequestKind::Stop){f_.state=State::Stopping;stopSinceTick_=o.tick;stopSinceUs_=o.monotonicUs;
              f_.stopAppliedSequence=r.sequence;f_.stopAppliedTick=o.tick;f_.stopAppliedMonotonicUs=o.monotonicUs;}
            else if(r.kind==RequestKind::Disarm)
            {f_.state=State::Disarmed;f_.capHeld=f_.stopLatched=f_.forceZeroOutput=false;
             f_.originValid=false;retiredSession_=f_.scope.session;}
            else if(r.kind==RequestKind::Arm)f_.state=State::Armed;
            return View();
        }
        Decision PriorityStop(std::uint64_t session,const Observation&o) noexcept
        {
            if(!Active()||session!=f_.scope.session)return View();
            if(f_.stopLatched)return View();
            Request stop{};stop.kind=RequestKind::Stop;stop.scope=f_.scope;
            stop.sequence=f_.lastRequestSequence==UINT64_MAX?UINT64_MAX:f_.lastRequestSequence+1U;stop.issueTick=o.tick;
            stop.issueMonotonicUs=o.monotonicUs;return LatchStop(stop,o);
        }
        // Queue cancellation before an Arm setter ran is evidence of cancellation,
        // never evidence of physical stopping. Active cancellation uses Stop.
        Decision CancelBeforeApplication(const Request&r,const Observation&o) noexcept
        {
            if(Active())return PriorityStop(r.scope.session,o);
            if(r.kind!=RequestKind::Arm||!ValidScope(r.scope)||!r.sequence||r.scope.session<=retiredSession_)
                return Reject(Reason::InvalidRequest);
            (void)Phase(o);Publish(o,false);f_.scope=r.scope;f_.lastRequestSequence=r.sequence;
            f_.state=State::Disarmed;f_.ack=Ack::Rejected;if(!f_.latchedFault)f_.reason=Reason::CancelledBeforeApply;
            f_.queuedCancellation=true;f_.stopProven=f_.targetProven=f_.originValid=false;
            f_.capHeld=f_.stopLatched=f_.forceZeroOutput=false;f_.stableCycles=0U;
            pendingValid_=pendingDecisionValid_=false;retiredSession_=r.scope.session;return View();
        }
        // End-of-output-phase refresh: facts only, never another dwell cycle.
        Decision RefreshFacts(const Observation&o)noexcept
        {
            const bool valid=haveCycle_&&o.tick==lastCycleTick_&&Phase(o)&&Source(o);
            Publish(o,valid);InvalidateReceipt(o);
            if(Active()&&(!Frame(o)||frameLost_))
            {frameLost_=true;f_.frameCurrent=Frame(o);f_.stopProven=f_.targetProven=false;
             f_.stableCycles=0U;haveCandidate_=false;f_.state=State::Stopping;return Trip(Reason::FrameChanged,o);}
            if(!valid){f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
                if(Active())return Trip(Reason::InvalidSource,o);}
            return View();
        }
        const Request& PendingRequest()const noexcept{return pending_;}
        Decision OnCycle(const Observation&o) noexcept
        {
            const bool phaseGood=Phase(o);
            const bool adjacent=!haveCycle_||(lastCycleTick_!=UINT64_MAX&&o.tick==lastCycleTick_+1U);
            const bool wallGood=!haveCycle_||(o.monotonicUs>lastClockUs_&&o.monotonicUs-lastClockUs_<=50000U);
            const bool fresh=!haveCycle_||o.tick>lastCycleTick_;
            if(!haveCycle_||o.tick>lastCycleTick_)lastCycleTick_=o.tick;
            if(!haveCycle_||o.monotonicUs>lastClockUs_)lastClockUs_=o.monotonicUs;
            haveCycle_=true;
            Publish(o,fresh&&adjacent&&wallGood&&phaseGood&&Source(o));
            if(!fresh||!adjacent||!wallGood||!phaseGood||!Source(o))
            {
                f_.armReady=false;idleCycles_=0U;f_.stableCycles=0U;haveCandidate_=false;f_.stopProven=f_.targetProven=false;
                return Active()?Trip(!wallGood?Reason::Clock:(!adjacent?Reason::RuntimeGap:Reason::InvalidSource),o):View();
            }
            if(!Active())
            {IdleProof(o);if(f_.state==State::Disarmed&&!f_.queuedCancellation)Proof(o);return View();}
            f_.frameCurrent=Frame(o);
            if(!f_.frameCurrent||frameLost_)
            {
                // A changed map/coordinate frame cannot prove the original
                // physical Z stopped. Retain the cap and zero-output fault
                // until core teardown; restored numeric bits do not erase
                // this loss of source continuity within the session.
                frameLost_=true;f_.stopProven=f_.targetProven=false;
                f_.stableCycles=0U;haveCandidate_=false;f_.state=State::Stopping;
                return Trip(Reason::FrameChanged,o);
            }
            if(!o.allOtherAxesIdle||!o.groupQuiet||!o.axisMailboxQuiet)return Trip(Reason::CompetingMotion,o);
            if(o.hardPositive||o.hardNegative)return Trip(Reason::HardLimit,o);
            const Config&c=f_.frozenConfig;
            const double half=c.outerHalfMm*c.pulsePerMm;
            if(std::fabs(o.actualPulse-f_.outerOriginPulse)>half||std::fabs(o.commandPulse-f_.outerOriginPulse)>half||
                std::fabs(o.planningPulse-f_.outerOriginPulse)>half)return Trip(Reason::OuterBound,o);
            if(std::fabs(o.actualPulse-o.commandPulse)>c.followingLimitMm*c.pulsePerMm)return Trip(Reason::FollowingError,o);
            if(!f_.stopLatched&&(!Authority(o)||o.tick>=deadlineTick_||o.monotonicUs>=deadlineUs_))return Trip(!Authority(o)?Reason::AuthorityLost:Reason::Deadman,o);
            if(f_.stopLatched&&(o.tick-stopSinceTick_>c.stopTimeoutTicks||o.monotonicUs-stopSinceUs_>c.stopTimeoutTicks*CycleUs)&&!f_.stopProven)
            {
                // Timeout remains faulted and zero-fenced. A later physical
                // receipt may retire this exact stopped session by Disarm;
                // it never clears the fault or admits a faulted Arm.
                (void)Trip(Reason::StopTimeout,o);
                if(!pendingValid_&&(f_.state==State::Stopping||f_.state==State::Held)&&
                    f_.stopAppliedSequence&&o.tick>f_.stopAppliedTick)Proof(o);
                Decision d=View();d.accepted=false;return d;
            }
            if(!pendingValid_&&(((f_.state==State::Moving||f_.state==State::AtTarget)&&LastAppliedFiniteMotion()&&o.tick>f_.lastAppliedTick)||
                ((f_.state==State::Stopping||f_.state==State::Held)&&f_.stopAppliedSequence&&o.tick>f_.stopAppliedTick)))
                Proof(o);
            return View();
        }
    private:
        Feedback f_{};Request pending_{};Decision pendingDecision_{};
        bool pendingDecisionValid_=false;
        bool pendingValid_=false,haveCycle_=false,haveCandidate_=false,haveIdle_=false,havePhase_=false,frameLost_=false;
        std::uint64_t lastCycleTick_=0U,lastClockUs_=0U,deadlineTick_=0U,deadlineUs_=0U,stopSinceTick_=0U,nextProof_=0U,retiredSession_=0U;
        std::uint64_t stopSinceUs_=0U,candidateSinceUs_=0U,idleSinceUs_=0U;
        std::uint64_t phaseTick_=0U,phaseUs_=0U,pendingTick_=0U,pendingUs_=0U;
        std::uint64_t lastCurveAcceptedTick_=0U,lastCurveAcceptedUs_=0U;
        std::uint32_t idleCycles_=0U;
        double candidateMin_=0.0,candidateMax_=0.0,candidateCmdMin_=0.0,candidateCmdMax_=0.0,
            candidatePlanMin_=0.0,candidatePlanMax_=0.0,idleAnchor_=0.0;
        Observation idleIdentity_{},candidateIdentity_{};
        std::int8_t frozenMode_=0;
        static bool Finite(double v)noexcept{return std::isfinite(v);}
        static bool Native(double v)noexcept{return Finite(v)&&std::fabs(v)<=2147483647.0;}
        bool Active()const noexcept{return f_.capHeld;}
        bool LastAppliedFiniteMotion()const noexcept
        {return f_.lastAppliedKind==RequestKind::Position||
            (IsGapFeedUpdateProfile(f_.frozenConfig.profile)&&f_.lastAppliedKind==RequestKind::FeedUpdate);}
        Decision Reject(Reason r)noexcept{if(!f_.latchedFault)f_.reason=r;f_.ack=Ack::Rejected;return View();}
        bool Phase(const Observation&o)noexcept
        {
            if(havePhase_&&(o.tick<phaseTick_||o.monotonicUs<phaseUs_))return false;
            if(o.tick>phaseTick_)phaseTick_=o.tick;
            if(o.monotonicUs>phaseUs_)phaseUs_=o.monotonicUs;
            havePhase_=true;return true;
        }
        void Seal(const Request&r,const Observation&o)noexcept
        {pending_=r;pendingValid_=true;pendingDecisionValid_=false;pendingTick_=o.tick;pendingUs_=o.monotonicUs;}
        Decision BindApplication(Decision d)noexcept
        {
            if(d.requiresCommit&&pendingValid_)
            {d.requestSession=pending_.scope.session;d.requestSequence=pending_.sequence;
             d.applicationTick=pendingTick_;d.applicationUs=pendingUs_;
             pendingDecision_=d;pendingDecisionValid_=true;}
            return d;
        }
        static bool SameDecision(const Decision&a,const Decision&b)noexcept
        {
            return a.action==b.action&&a.accepted==b.accepted&&a.requiresCommit==b.requiresCommit&&
                a.capHeld==b.capHeld&&a.stopLatched==b.stopLatched&&a.forceZeroOutput==b.forceZeroOutput&&
                a.nativeTargetPulse==b.nativeTargetPulse&&a.nativeVelocityPps==b.nativeVelocityPps&&
                a.accelerationTimeSec==b.accelerationTimeSec&&a.decelerationTimeSec==b.decelerationTimeSec&&
                a.requestSession==b.requestSession&&a.requestSequence==b.requestSequence&&
                a.applicationTick==b.applicationTick&&a.applicationUs==b.applicationUs;
        }
        Reason ApplicationReason(const Decision&d,const Observation&o)const noexcept
        {
            if(!pendingValid_||!pendingDecisionValid_||!d.requiresCommit||!SameDecision(d,pendingDecision_)||
                d.requestSession!=pending_.scope.session||d.requestSequence!=pending_.sequence||
                d.applicationTick!=pendingTick_||d.applicationUs!=pendingUs_)return Reason::InvalidRequest;
            if(o.tick!=pendingTick_||o.monotonicUs!=pendingUs_||
                (havePhase_&&(o.tick<phaseTick_||o.monotonicUs<phaseUs_)))return Reason::Clock;
            // A fault Stop may have accepted=false. Source or authority loss
            // cannot veto its stop primitive, but it still needs an exact seal.
            if(pending_.kind==RequestKind::Stop)
                return d.action==Action::Stop&&f_.capHeld&&f_.stopLatched?Reason::None:Reason::InvalidRequest;
            if(!d.accepted)return Reason::InvalidRequest;
            const bool shape=(pending_.kind==RequestKind::Arm&&d.action==Action::None)||
                (pending_.kind==RequestKind::Disarm&&d.action==Action::None)||
                (pending_.kind==RequestKind::Position&&d.action==Action::Move)||
                (pending_.kind==RequestKind::FeedUpdate&&d.action==Action::FeedUpdate);
            if(!shape)return Reason::InvalidRequest;
            if(!Frame(o)||frameLost_)return Reason::FrameChanged;
            if(!Source(o))return Reason::InvalidSource;
            if(pending_.kind==RequestKind::Disarm)
                return f_.stopProven&&f_.state==State::Held?Reason::None:Reason::NotAtTarget;
            if(!Authority(o))return Reason::AuthorityLost;
            if(!f_.capHeld||f_.stopLatched||f_.latchedFault||f_.forceZeroOutput)return Reason::AwaitStop;
            return Reason::None;
        }
        Decision FailApplication(Reason reason,const Observation&o)noexcept
        {
            // Preserve a superseding Stop and its association. It is the only
            // pending request that a failed/stale actuator application keeps.
            const bool pendingStop=pendingValid_&&pending_.kind==RequestKind::Stop&&pendingDecisionValid_;
            if(!pendingStop)pendingValid_=pendingDecisionValid_=false;
            if(reason==Reason::FrameChanged)
            {frameLost_=true;f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;}
            Decision d=Trip(reason,o);
            if(!d.requiresCommit&&pendingStop)
            {d=pendingDecision_;d.accepted=false;d.forceZeroOutput=f_.forceZeroOutput;d=BindApplication(d);}
            if(!d.requiresCommit&&Active())
            {d=View();d.action=Action::Stop;d.accepted=false;}
            return d;
        }
        static bool SameRequest(const Request&a,const Request&b)noexcept
        {return a.kind==b.kind&&SameScope(a.scope,b.scope)&&a.sequence==b.sequence&&a.issueTick==b.issueTick&&
            a.issueMonotonicUs==b.issueMonotonicUs&&a.relativeTargetMm==b.relativeTargetMm&&
            a.curveSampleTick==b.curveSampleTick&&a.curveSampleUs==b.curveSampleUs&&
            a.curveVoltage==b.curveVoltage&&a.signedCurveMmMin==b.signedCurveMmMin&&
            (a.kind!=RequestKind::Arm||SameConfig(a.config,b.config));}
        static bool EmptyCurve(const Request&r)noexcept
        {return r.curveSampleTick==0U&&r.curveSampleUs==0U&&r.curveVoltage==0.0&&r.signedCurveMmMin==0.0;}
        Reason CurveRequest(const Request&r,const Observation&o,double target,double&velocity)const noexcept
        {
            const Config&c=f_.frozenConfig;
            const bool rapid=IsGapRapidProfile(c.profile);
            if(rapid&&o.feedrateOverride!=1.0)return Reason::InvalidRequest;
            const bool shortReplan=IsGapShortReplanProfile(c.profile)||IsGapFeedUpdateProfile(c.profile)||rapid;
            if(!r.curveSampleTick||!r.curveSampleUs||
                !(rapid?EDM43::IsScriptVoltage(r.curveVoltage):shortReplan?EDM41::IsScriptVoltage(r.curveVoltage):EDM39::IsScriptVoltage(r.curveVoltage))||
                !Finite(r.signedCurveMmMin))return Reason::InvalidRequest;
            if(r.curveSampleTick>r.issueTick||r.curveSampleUs>r.issueMonotonicUs)
                return Reason::IssueClock;
            if((r.curveSampleTick==r.issueTick)!=(r.curveSampleUs==r.issueMonotonicUs)||
                (r.issueTick==o.tick)!=(r.issueMonotonicUs==o.monotonicUs))
                return Reason::IssueClock;
            if(o.tick-r.curveSampleTick>c.maximumIssueAgeTicks||
                o.monotonicUs-r.curveSampleUs>c.maximumIssueAgeTicks*CycleUs)
                return Reason::StaleRequest;
            if(r.curveSampleTick<=lastCurveAcceptedTick_||r.curveSampleUs<=lastCurveAcceptedUs_)
                return Reason::Sequence;
            const EDMGapServo::Result curve=rapid?EDM43::Evaluate(r.curveVoltage):EDM39::Evaluate(r.curveVoltage);
            const double rate=curve.limitedSpeedMmPerMin;
            if(!curve.valid||!Finite(rate)||rate!=r.signedCurveMmMin||
                (rapid?(std::fabs(rate)!=15.0&&std::fabs(rate)!=30.0):(std::fabs(rate)!=10.0&&std::fabs(rate)!=20.0))||std::fabs(rate)>c.feedMmMin)
                return Reason::InvalidRequest;
            if(rapid&&r.relativeTargetMm!=0.0&&r.relativeTargetMm!=-EDM43::TargetHalfMm)
                return Reason::InvalidRequest;
            // A low-voltage short can only retreat to the immutable first-Arm
            // origin. Debounce history is owned by NC; this gate independently
            // forbids forward or arbitrary-target motion from a fresh 20V tuple.
            if(shortReplan&&r.curveVoltage==20.0&&(rate!=(rapid?-30.0:-20.0)||r.relativeTargetMm!=0.0))
                return Reason::InvalidRequest;
            // EDM50 converts only the already validated finite command. The
            // current RT observation and all profile/source authority stay here.
            EDM50::CurveMotionRequest command{};
            command.axis.backend=EDM50::Backend::LocalRTMotion; command.axis.unit=EDM50::AxisUnit::Millimeter;
            command.axis.axisIndex=static_cast<std::uint8_t>(c.axisIndex); command.axis.nativePerUnit=c.pulsePerMm;
            command.nativeTarget=target; command.actualNative=o.actualPulse;
            command.commandNative=o.commandPulse; command.planningNative=o.planningPulse;
            command.positionToleranceUnits=c.positionToleranceMm;
            // Positive curve advances along machine -Z, independently of the
            // relative endpoint sign and physical drive wiring sign.
            command.nativeDirection=rate>0.0?-1:1; command.positiveRateUnitsPerMinute=std::fabs(rate);
            command.maximumNativeRate=o.maximumVelocityPps; command.maximumOutputUnitsPerSecond=c.pdoCapMmS;
            command.accelerationTimeSec=c.accelerationTimeSec; command.decelerationTimeSec=c.decelerationTimeSec;
            const EDM50::CurveMotionResult translated=EDM50::EvaluateCurveMotion(command);
            if(!translated.Ready())return translated.reason==EDM50::Reason::NativeSpeed?Reason::NativeSpeed:Reason::InvalidRequest;
            velocity=translated.nativeVelocity;
            return Reason::None;
        }
        Reason FeedUpdateRequest(const Request&r,const Observation&o,double&velocity)const noexcept
        {
            const Config&c=f_.frozenConfig;
            if(!IsGapFeedUpdateProfile(c.profile)||f_.state!=State::Moving||o.axisIdle||
                !f_.originValid||f_.targetProven||!LastAppliedFiniteMotion()||
                f_.lastCurveSequence!=f_.lastAppliedSequence||o.tick<=f_.lastAppliedTick||
                o.monotonicUs<=f_.lastAppliedMonotonicUs)return Reason::InvalidRequest;
            if(!o.allOtherAxesIdle||!o.groupQuiet||!o.axisMailboxQuiet)return Reason::CompetingMotion;
            if(o.hardPositive||o.hardNegative)return Reason::HardLimit;
            const double half=c.outerHalfMm*c.pulsePerMm;
            if(std::fabs(o.actualPulse-f_.outerOriginPulse)>half||
                std::fabs(o.commandPulse-f_.outerOriginPulse)>half||
                std::fabs(o.planningPulse-f_.outerOriginPulse)>half)return Reason::OuterBound;
            if(std::fabs(o.actualPulse-o.commandPulse)>c.followingLimitMm*c.pulsePerMm)
                return Reason::FollowingError;
            if(o.feedrateOverride!=1.0||!Finite(r.relativeTargetMm)||
                std::fabs(r.relativeTargetMm)>c.targetHalfMm||
                f_.outerOriginPulse+r.relativeTargetMm*c.pulsePerMm!=f_.targetPulse||
                !EDM42::IsSameDirectionFeedPair(f_.lastCurveVoltage,r.curveVoltage))return Reason::InvalidRequest;
            const Reason curveReason=CurveRequest(r,o,f_.targetPulse,velocity);
            if(curveReason!=Reason::None)return curveReason;
            const double direction=r.signedCurveMmMin>0.0?-1.0:1.0;
            const double remaining=EDM42::RtFeedUpdateMinRemainingMm*c.pulsePerMm;
            if(f_.lastSignedCurveMmMin*r.signedCurveMmMin<=0.0||
                o.cmdVelocityPps*direction<=0.0||
                (f_.targetPulse-o.actualPulse)*direction<=remaining||
                (f_.targetPulse-o.commandPulse)*direction<=remaining||
                (f_.targetPulse-o.planningPulse)*direction<=remaining)return Reason::InvalidRequest;
            return Reason::None;
        }
        static bool SameProofFrame(const Observation&a,const Observation&b)noexcept
        {return a.axisMapGeneration==b.axisMapGeneration&&a.configGeneration==b.configGeneration&&
            a.pulsePerMm==b.pulsePerMm&&a.hardwareSign==b.hardwareSign&&a.referenceOffsetPulse==b.referenceOffsetPulse&&
            a.modeValue==b.modeValue;}
        static bool Source(const Observation&o)noexcept
        {return o.tick&&o.pdoValid&&o.contiguous&&o.clockValid&&o.axisExists&&o.linear&&o.servoReady&&o.modeReady&&!o.fault&&
            o.noCompensation&&o.motorEncoderSource&&(o.statusWord&0x006FU)==0x0027U&&o.modeValue==9&&Finite(o.pulsePerMm)&&o.pulsePerMm>0.0&&
            Finite(o.referenceOffsetPulse)&&Finite(o.maximumVelocityPps)&&o.maximumVelocityPps>0.0&&
            (o.hardwareSign==1||o.hardwareSign==-1)&&Native(o.actualPulse)&&Native(o.commandPulse)&&Native(o.planningPulse)&&
            Finite(o.cmdVelocityPps)&&Finite(o.logicalVelocityPps)&&Finite(o.targetVelocityPps)&&
            Finite(o.actualVelocityPps)&&Finite(o.pdoVelocityPps)&&
            Finite(o.feedrateOverride)&&o.feedrateOverride>=0.0&&o.feedrateOverride<=1.0;}
        bool Authority(const Observation&o)const noexcept
        {return o.currentOwner==f_.scope.owner&&o.currentOwnerGeneration==f_.scope.ownerGeneration&&o.currentEpoch==f_.scope.executionEpoch;}
        bool Frame(const Observation&o)const noexcept
        {return o.axisMapGeneration==f_.scope.axisMapGeneration&&o.configGeneration==f_.scope.configGeneration&&
            o.pulsePerMm==f_.frozenConfig.pulsePerMm&&o.hardwareSign==f_.frozenConfig.hardwareSign&&
            o.referenceOffsetPulse==f_.frozenConfig.referenceOffsetPulse&&o.modeValue==frozenMode_;}
        static bool ConfigValid(const Config&c,const Observation&o)noexcept
        {
            const double v=c.feedMmMin*c.pulsePerMm/60.0;
            const bool legacyP13=c.profile==Profile::P13Unloaded&&Finite(c.feedMmMin)&&c.feedMmMin>0.0&&c.feedMmMin<=2.5;
            const bool fixedLater=(c.profile==Profile::P14UnloadedSpeed||c.profile==Profile::P15UnloadedRepeat||c.profile==Profile::P16UnloadedRetreat||c.profile==Profile::P17PersistentShort||c.profile==Profile::P22GapPersistentShort)&&c.feedMmMin==ProfileFeedMmMin(c.profile)&&
                c.outerHalfMm==.1&&c.targetHalfMm==.02&&c.pdoCapMmS==.1&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles;
            // P18 alone admits the fixed F10/.2 step. Its full cap, including
            // possible PID correction, must fit the actual axis maximum.
            const double capPps=c.pdoCapMmS*c.pulsePerMm;
            const bool fixedP18=c.profile==Profile::P18UnloadedSpeedRetreat&&c.feedMmMin==10.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==.02&&c.pdoCapMmS==.2&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P19 is an explicit F15/.3 opt-in; no old profile can acquire
            // its higher cap through a mutable feed, recipe or re-arm.
            const bool fixedP19=c.profile==Profile::P19UnloadedSpeedRetreat&&c.feedMmMin==15.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==.02&&c.pdoCapMmS==.3&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P20 alone admits the fixed F20/.35 step; all other limits and
            // the first-Arm frozen profile remain mandatory through re-arm.
            const bool fixedP20=c.profile==Profile::P20UnloadedSpeedRetreat&&c.feedMmMin==20.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==.02&&c.pdoCapMmS==.35&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P21 changes only the NC SIM detector bridge; its RT motion limits
            // remain the accepted P20 envelope and are frozen independently.
            const bool fixedP21=c.profile==Profile::P21GapShortRetreat&&c.feedMmMin==20.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==.02&&c.pdoCapMmS==.35&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P23 freezes a ceiling while each finite SIM segment carries a
            // separately checked curve source and independently derived rate.
            const bool fixedP23=c.profile==Profile::P23GapCurveSegments&&c.feedMmMin==20.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==.02&&c.pdoCapMmS==.35&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P24 alone widens the finite target to leave a measurable moving
            // window for a SIM-triggered priority Stop. Re-arm still preserves
            // the first origin and all Position curve validation above.
            const bool fixedP24=c.profile==Profile::P24GapCurveReplan&&c.feedMmMin==20.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==EDM40::TargetHalfMm&&c.pdoCapMmS==.35&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P25/P26 opt in separately to short-circuit replan. No mutable
            // override can widen this fixed envelope or change a frozen mode.
            const bool fixedShortReplan=IsGapShortReplanProfile(c.profile)&&c.feedMmMin==20.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==EDM40::TargetHalfMm&&c.pdoCapMmS==.35&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P27/P28 preserve the P25/P26 envelope and independently opt in to
            // in-flight feed changes on the original finite endpoint only.
            const bool fixedFeedUpdate=IsGapFeedUpdateProfile(c.profile)&&c.feedMmMin==20.0&&
                c.outerHalfMm==.1&&c.targetHalfMm==EDM40::TargetHalfMm&&c.pdoCapMmS==.35&&
                c.accelerationTimeSec==.2&&c.decelerationTimeSec==.2&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                o.feedrateOverride==1.0&&Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            // P29..P32 alone admit the longer finite rapid path and slower
            // .3 s ramps. Every value remains frozen across same-origin re-arm.
            const bool fixedRapid=IsGapRapidProfile(c.profile)&&c.feedMmMin==EDM43::FeedMmMin&&
                c.outerHalfMm==EDM43::OuterHalfMm&&c.targetHalfMm==EDM43::TargetHalfMm&&c.pdoCapMmS==EDM43::PdoCapMmS&&
                c.accelerationTimeSec==EDM43::AccelerationTimeSec&&c.decelerationTimeSec==EDM43::DecelerationTimeSec&&c.followingLimitMm==.02&&
                c.positionToleranceMm==.001&&c.excursionToleranceMm==.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks==80U&&c.stopTimeoutTicks==4000U&&c.settleCycles==SettleCycles&&
                o.feedrateOverride==1.0&&Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps;
            return (legacyP13||fixedLater||fixedP18||fixedP19||fixedP20||fixedP21||fixedP23||fixedP24||fixedShortReplan||fixedFeedUpdate||fixedRapid)&&
                (c.profile!=Profile::P22GapPersistentShort||(Native(capPps)&&std::floor(capPps)>=1.0&&capPps<=o.maximumVelocityPps))&&c.axisIndex==2U&&c.pulsePerMm==o.pulsePerMm&&c.pulsePerMm>=1000.0&&c.pulsePerMm<=1000000000.0&&
                c.referenceOffsetPulse==o.referenceOffsetPulse&&(c.hardwareSign==1||c.hardwareSign==-1)&&c.hardwareSign==o.hardwareSign&&
                Finite(c.outerHalfMm)&&c.outerHalfMm>0.0&&(c.outerHalfMm<=.1||(fixedRapid&&c.outerHalfMm==EDM43::OuterHalfMm))&&
                Finite(c.targetHalfMm)&&c.targetHalfMm>0.0&&(c.targetHalfMm<=.02||((fixedP24||fixedShortReplan||fixedFeedUpdate)&&c.targetHalfMm==EDM40::TargetHalfMm)||(fixedRapid&&c.targetHalfMm==EDM43::TargetHalfMm))&&c.targetHalfMm<c.outerHalfMm&&
                Finite(c.feedMmMin)&&c.feedMmMin>0.0&&Finite(c.pdoCapMmS)&&
                c.pdoCapMmS>=c.feedMmMin/60.0&&(c.pdoCapMmS<=.1||(fixedP18&&c.pdoCapMmS==.2)||(fixedP19&&c.pdoCapMmS==.3)||((fixedP20||fixedP21||fixedP23||fixedP24||fixedShortReplan||fixedFeedUpdate)&&c.pdoCapMmS==.35)||(fixedRapid&&c.pdoCapMmS==EDM43::PdoCapMmS))&&
                Finite(c.accelerationTimeSec)&&c.accelerationTimeSec>=.001&&c.accelerationTimeSec<=1.0&&
                Finite(c.decelerationTimeSec)&&c.decelerationTimeSec>=.001&&c.decelerationTimeSec<=1.0&&
                Native(v)&&v>1.0&&Finite(o.maximumVelocityPps)&&v<=o.maximumVelocityPps&&
                v/c.accelerationTimeSec>10.0&&v/c.decelerationTimeSec>10.0&&
                Finite(c.followingLimitMm)&&c.followingLimitMm>0.0&&c.followingLimitMm<=.02&&
                Finite(c.positionToleranceMm)&&c.positionToleranceMm>=1.0/c.pulsePerMm&&c.positionToleranceMm<=.001&&
                Finite(c.excursionToleranceMm)&&c.excursionToleranceMm>=1.0/c.pulsePerMm&&c.excursionToleranceMm<=.001&&
                c.heartbeatTicks==HeartbeatTicks&&c.maximumIssueAgeTicks>0U&&c.maximumIssueAgeTicks<=80U&&
                c.settleCycles>=SettleCycles&&c.settleCycles<=1000U&&c.stopTimeoutTicks>=1000U&&c.stopTimeoutTicks<=4000U;
        }
        static bool SameConfig(const Config&a,const Config&b)noexcept
        {return a.axisIndex==b.axisIndex&&a.pulsePerMm==b.pulsePerMm&&a.referenceOffsetPulse==b.referenceOffsetPulse&&
          a.hardwareSign==b.hardwareSign&&a.outerHalfMm==b.outerHalfMm&&a.targetHalfMm==b.targetHalfMm&&
          a.feedMmMin==b.feedMmMin&&a.pdoCapMmS==b.pdoCapMmS&&a.accelerationTimeSec==b.accelerationTimeSec&&
          a.decelerationTimeSec==b.decelerationTimeSec&&a.followingLimitMm==b.followingLimitMm&&
          a.positionToleranceMm==b.positionToleranceMm&&a.excursionToleranceMm==b.excursionToleranceMm&&
          a.heartbeatTicks==b.heartbeatTicks&&a.maximumIssueAgeTicks==b.maximumIssueAgeTicks&&
          a.stopTimeoutTicks==b.stopTimeoutTicks&&a.settleCycles==b.settleCycles&&a.profile==b.profile;}
        bool RequestFresh(const Request&r,const Observation&o)noexcept
        {
            if(!r.sequence||r.sequence==UINT64_MAX||r.sequence<=f_.lastRequestSequence){if(!f_.latchedFault)f_.reason=Reason::Sequence;return false;}
            f_.lastRequestSequence=r.sequence;
            if(r.issueTick>o.tick||r.issueTick>UINT64_MAX-f_.frozenConfig.heartbeatTicks){if(!f_.latchedFault)f_.reason=Reason::IssueClock;return false;}
            if(o.tick-r.issueTick>f_.frozenConfig.maximumIssueAgeTicks){if(!f_.latchedFault)f_.reason=Reason::StaleRequest;return false;}
            if(r.issueMonotonicUs>o.monotonicUs||r.issueMonotonicUs>UINT64_MAX-50000U)
            {if(!f_.latchedFault)f_.reason=Reason::IssueClock;return false;}
            if(o.monotonicUs-r.issueMonotonicUs>f_.frozenConfig.maximumIssueAgeTicks*CycleUs)
            {if(!f_.latchedFault)f_.reason=Reason::StaleRequest;return false;}
            return true;
        }
        Decision Arm(const Request&r,const Observation&o)noexcept
        {
            if(!ValidScope(r.scope)||r.scope.owner!=o.currentOwner||r.scope.ownerGeneration!=o.currentOwnerGeneration||
                r.scope.executionEpoch!=o.currentEpoch||r.scope.axisMapGeneration!=o.axisMapGeneration||r.scope.configGeneration!=o.configGeneration)
            {if(!f_.latchedFault)f_.reason=Reason::InvalidScope;return View();}
            if(!ConfigValid(r.config,o)){if(!f_.latchedFault)f_.reason=Reason::InvalidConfig;return View();}
            const bool rearm=Active()&&f_.state==State::Held&&f_.stopProven;
            if(Active()&&!rearm){if(!f_.latchedFault)f_.reason=Reason::Busy;return View();}
            if(!o.axisIdle||!o.allOtherAxesIdle||!o.groupQuiet||!o.axisMailboxQuiet||o.hardPositive||o.hardNegative||
                std::fabs(o.cmdVelocityPps)>1.0||std::fabs(o.logicalVelocityPps)>1.0||std::fabs(o.targetVelocityPps)>1.0||
                (rearm?!Frame(o):(!f_.armReady||!SameProofFrame(o,idleIdentity_)||std::fabs(o.actualPulse-idleAnchor_)>.001*o.pulsePerMm||
                    std::fabs(o.actualPulse-o.commandPulse)>.001*o.pulsePerMm))){if(!f_.latchedFault)f_.reason=Reason::NotReady;return View();}
            if(rearm&&(frameLost_||f_.latchedFault||!SameConfig(r.config,f_.frozenConfig))){if(!f_.latchedFault)f_.reason=Reason::InvalidConfig;return View();}
            if(rearm&&(r.scope.session!=f_.scope.session||r.scope.runGeneration!=f_.scope.runGeneration||r.scope.cacheGeneration!=f_.scope.cacheGeneration||
                r.scope.dispatchGeneration!=f_.scope.dispatchGeneration)){if(!f_.latchedFault)f_.reason=Reason::InvalidScope;return View();}
            if(!rearm&&r.scope.session<=retiredSession_){if(!f_.latchedFault)f_.reason=Reason::InvalidScope;return View();}
            if(!r.sequence||r.sequence==UINT64_MAX||(rearm&&r.sequence<=f_.lastRequestSequence)){if(!f_.latchedFault)f_.reason=Reason::Sequence;return View();}
            if(r.issueTick>o.tick||r.issueMonotonicUs>o.monotonicUs||r.issueTick>UINT64_MAX-r.config.heartbeatTicks||
                r.issueMonotonicUs>UINT64_MAX-50000U){if(!f_.latchedFault)f_.reason=Reason::IssueClock;return View();}
            if(o.tick-r.issueTick>r.config.maximumIssueAgeTicks||
                o.monotonicUs-r.issueMonotonicUs>r.config.maximumIssueAgeTicks*CycleUs)
            {if(!f_.latchedFault)f_.reason=Reason::StaleRequest;return View();}
            const double origin=f_.originValid?f_.outerOriginPulse:o.actualPulse;
            if(!Native(origin-r.config.outerHalfMm*r.config.pulsePerMm)||
                !Native(origin+r.config.outerHalfMm*r.config.pulsePerMm))
            {if(!f_.latchedFault)f_.reason=Reason::NativeRange;return View();}
            if(rearm&&std::fabs(o.actualPulse-origin)>r.config.outerHalfMm*r.config.pulsePerMm)
            {if(!f_.latchedFault)f_.reason=Reason::OuterBound;return View();}
            if(!rearm)
            {lastCurveAcceptedTick_=lastCurveAcceptedUs_=0U;
             f_.lastCurveSequence=f_.lastCurveSampleTick=f_.lastCurveSampleUs=0U;
             f_.lastCurveVoltage=f_.lastSignedCurveMmMin=0.0;}
            f_.scope=r.scope;f_.frozenConfig=r.config;frozenMode_=o.modeValue;
            f_.lastRequestSequence=r.sequence;f_.outerOriginPulse=origin;f_.originValid=true;
            f_.capHeld=true;f_.stopLatched=f_.forceZeroOutput=false;f_.state=State::Armed;
            f_.armReady=false;f_.latchedFault=false;f_.queuedCancellation=false;
            f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
            deadlineTick_=r.issueTick+r.config.heartbeatTicks;deadlineUs_=r.issueMonotonicUs+50000U;Seal(r,o);
            Decision d=View();d.accepted=d.requiresCommit=true;return BindApplication(d);
        }
        Decision Trip(Reason reason,const Observation&o)noexcept
        {
            if(f_.latchedFault&&f_.stopLatched&&reason!=Reason::StopTimeout)
            {f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;}
            const bool firstFault=!f_.latchedFault;
            if(firstFault){f_.reason=reason;f_.latchedFault=true;f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
                if(f_.stopLatched)f_.state=State::Stopping;}
            if(!Active())return View();
            if(reason!=Reason::Deadman)f_.forceZeroOutput=true;
            Decision d=PriorityStop(f_.scope.session,o);
            d.forceZeroOutput=f_.forceZeroOutput;d.accepted=false;return BindApplication(d);
        }
        Decision LatchStop(const Request&r,const Observation&o)noexcept
        {
            f_.stopLatched=true;f_.stopProven=f_.targetProven=false;f_.stableCycles=0U;haveCandidate_=false;
            f_.stopAppliedSequence=f_.stopAppliedTick=f_.stopAppliedMonotonicUs=0U;
            Seal(r,o);f_.lastRequestSequence=r.sequence;f_.state=State::Stopping;stopSinceTick_=o.tick;stopSinceUs_=o.monotonicUs;
            Decision d=View();d.action=Action::Stop;d.accepted=d.requiresCommit=true;
            d.decelerationTimeSec=f_.frozenConfig.decelerationTimeSec;return BindApplication(d);
        }
        Decision View()const noexcept
        {Decision d{};d.capHeld=f_.capHeld;d.stopLatched=f_.stopLatched;d.forceZeroOutput=f_.forceZeroOutput;
            d.decelerationTimeSec=f_.frozenConfig.decelerationTimeSec;return d;}
        void Publish(const Observation&o,bool fresh)noexcept
        {
            ++f_.publicationSequence;f_.sampledTick=o.tick;f_.monotonicUs=o.monotonicUs;f_.sourceFresh=fresh;
            f_.pdoValid=o.pdoValid;f_.contiguous=o.contiguous;f_.clockValid=o.clockValid;
            f_.axisMapGeneration=o.axisMapGeneration;f_.configGeneration=o.configGeneration;
            f_.currentEpoch=o.currentEpoch;f_.currentOwner=o.currentOwner;f_.currentOwnerGeneration=o.currentOwnerGeneration;
            f_.pulsePerMm=o.pulsePerMm;f_.hardwareSign=o.hardwareSign;f_.referenceOffsetPulse=o.referenceOffsetPulse;f_.feedrateOverride=o.feedrateOverride;
            f_.actualPulse=o.actualPulse;f_.commandPulse=o.commandPulse;f_.planningPulse=o.planningPulse;
            f_.statusWord=o.statusWord;f_.modeValue=o.modeValue;f_.axisExists=o.axisExists;f_.linear=o.linear;f_.axisIdle=o.axisIdle;
            f_.servoReady=o.servoReady;f_.modeReady=o.modeReady;f_.fault=o.fault;f_.homeFact=o.homeFact;
            f_.softLimitsFact=o.softLimitsFact;f_.hardPositive=o.hardPositive;f_.hardNegative=o.hardNegative;
            f_.actualVelocityPps=o.actualVelocityPps;
            if(Finite(o.pulsePerMm)&&o.pulsePerMm>0.0)
            {f_.actualMm=(o.actualPulse-f_.outerOriginPulse)/o.pulsePerMm;
             f_.commandMm=(o.commandPulse-f_.outerOriginPulse)/o.pulsePerMm;
             f_.cmdSpeedMmS=o.cmdVelocityPps/o.pulsePerMm;f_.pdoSpeedMmS=o.pdoVelocityPps/o.pulsePerMm;}
        }
        void IdleProof(const Observation&o)noexcept
        {
            const bool identity=haveIdle_&&o.currentOwner==idleIdentity_.currentOwner&&o.currentOwnerGeneration==idleIdentity_.currentOwnerGeneration&&
                o.currentEpoch==idleIdentity_.currentEpoch&&o.axisMapGeneration==idleIdentity_.axisMapGeneration&&
                SameProofFrame(o,idleIdentity_);
            const bool quiet=o.axisIdle&&o.allOtherAxesIdle&&o.groupQuiet&&o.axisMailboxQuiet&&
                std::fabs(o.cmdVelocityPps)<=1.0&&std::fabs(o.logicalVelocityPps)<=1.0&&std::fabs(o.targetVelocityPps)<=1.0&&
                std::fabs(o.actualPulse-o.commandPulse)<=.001*o.pulsePerMm&&!o.hardPositive&&!o.hardNegative;
            if(!quiet||!identity||std::fabs(o.actualPulse-idleAnchor_)>.001*o.pulsePerMm)
            {idleCycles_=quiet?1U:0U;idleAnchor_=o.actualPulse;idleIdentity_=o;idleSinceUs_=o.monotonicUs;haveIdle_=true;}
            else if(idleCycles_<SettleCycles)++idleCycles_;
            f_.armReady=idleCycles_>=SettleCycles&&o.monotonicUs-idleSinceUs_>=SettleCycles*CycleUs;
        }
        // EDM43-FIX1: a previously proved Held stop can lose its receipt
        // during a long operator HOLD. Give that one loss a bounded fresh
        // proof window; the original Stop application stamps stay immutable.
        // An unproved initial stop, a fault, a changed frame or commanded
        // motion cannot restart this deadline. Repeated unsettled samples
        // cannot extend it: stopProven is cleared immediately by the caller.
        void BeginRapidStopProofRenewal(const Observation&o)noexcept
        {
            if(IsGapRapidProfile(f_.frozenConfig.profile)&&f_.state==State::Held&&
                f_.capHeld&&f_.originValid&&f_.stopLatched&&f_.stopProven&&
                !f_.latchedFault&&!f_.forceZeroOutput&&!frameLost_&&!pendingValid_&&
                f_.sourceFresh&&Source(o)&&Frame(o)&&Authority(o)&&o.axisIdle&&
                o.allOtherAxesIdle&&o.groupQuiet&&o.axisMailboxQuiet&&!o.hardPositive&&!o.hardNegative&&
                std::fabs(o.cmdVelocityPps)<=1.0&&std::fabs(o.logicalVelocityPps)<=1.0&&std::fabs(o.targetVelocityPps)<=1.0&&
                f_.lastAppliedKind==RequestKind::Stop&&f_.lastAppliedSequence==f_.stopAppliedSequence&&
                f_.lastAppliedTick==f_.stopAppliedTick&&f_.lastAppliedMonotonicUs==f_.stopAppliedMonotonicUs&&
                f_.stopAppliedSequence&&haveCycle_&&(o.tick==lastCycleTick_||
                    (lastCycleTick_!=UINT64_MAX&&o.tick==lastCycleTick_+1U&&o.monotonicUs>lastClockUs_&&o.monotonicUs-lastClockUs_<=50000U))&&
                o.tick>=stopSinceTick_&&o.monotonicUs>=lastClockUs_&&
                o.monotonicUs>=stopSinceUs_)
            {stopSinceTick_=o.tick;stopSinceUs_=o.monotonicUs;f_.state=State::Stopping;}
        }
        void InvalidateReceipt(const Observation&o)noexcept
        {
            if(!f_.targetProven&&!f_.stopProven)return;
            const Config&c=f_.frozenConfig;
            const double ppu=o.pulsePerMm<c.pulsePerMm?o.pulsePerMm:c.pulsePerMm;
            const double tol=c.excursionToleranceMm*ppu;
            bool valid=!frameLost_&&Source(o)&&haveCandidate_&&SameProofFrame(o,candidateIdentity_)&&o.axisIdle&&
                o.allOtherAxesIdle&&o.groupQuiet&&o.axisMailboxQuiet&&!o.hardPositive&&!o.hardNegative&&
                std::fabs(o.cmdVelocityPps)<=1.0&&std::fabs(o.logicalVelocityPps)<=1.0&&std::fabs(o.targetVelocityPps)<=1.0&&
                std::fabs(o.actualPulse-o.commandPulse)<=c.positionToleranceMm*ppu&&
                o.actualPulse>=candidateMax_-tol&&o.actualPulse<=candidateMin_+tol&&
                o.commandPulse>=candidateCmdMax_-tol&&o.commandPulse<=candidateCmdMin_+tol&&
                o.planningPulse>=candidatePlanMax_-tol&&o.planningPulse<=candidatePlanMin_+tol;
            if(f_.targetProven)valid=valid&&Frame(o)&&std::fabs(o.actualPulse-f_.targetPulse)<=c.positionToleranceMm*ppu&&
                std::fabs(o.commandPulse-f_.targetPulse)<=c.positionToleranceMm*ppu&&std::fabs(o.planningPulse-f_.targetPulse)<=c.positionToleranceMm*ppu;
            if(!valid){BeginRapidStopProofRenewal(o);f_.targetProven=f_.stopProven=false;f_.stableCycles=0U;haveCandidate_=false;
                if(f_.state==State::AtTarget)f_.state=State::Moving;}
        }
        void Proof(const Observation&o)noexcept
        {
            const Config&c=f_.frozenConfig;
            const bool targetPhase=f_.state==State::Moving||f_.state==State::AtTarget;
            // Stop association survives Disarm metadata. An accepted but not
            // applied Stop can never contribute to this window.
            if(frameLost_)return;
            if((targetPhase&&(pendingValid_||!LastAppliedFiniteMotion()||o.tick<=f_.lastAppliedTick))||
                (!targetPhase&&(!f_.stopAppliedSequence||o.tick<=f_.stopAppliedTick||pendingValid_)))return;
            const double ppu=o.pulsePerMm<c.pulsePerMm?o.pulsePerMm:c.pulsePerMm;
            const double tolerance=c.excursionToleranceMm*ppu;
            bool candidate=o.axisIdle&&std::fabs(o.cmdVelocityPps)<=1.0&&std::fabs(o.logicalVelocityPps)<=1.0&&std::fabs(o.targetVelocityPps)<=1.0&&std::fabs(o.targetVelocityPps)<=1.0&&
                std::fabs(o.actualPulse-o.commandPulse)<=c.positionToleranceMm*ppu;
            if(targetPhase)
                candidate=candidate&&Frame(o)&&std::fabs(o.actualPulse-f_.targetPulse)<=c.positionToleranceMm*ppu&&
                    std::fabs(o.commandPulse-f_.targetPulse)<=c.positionToleranceMm*ppu&&
                    std::fabs(o.planningPulse-f_.targetPulse)<=c.positionToleranceMm*ppu;
            if(!candidate)
            {BeginRapidStopProofRenewal(o);f_.stableCycles=0U;haveCandidate_=false;f_.stopProven=f_.targetProven=false;
             if(targetPhase)f_.state=State::Moving;
             return;}
            if(haveCandidate_&&!SameProofFrame(o,candidateIdentity_))
            {BeginRapidStopProofRenewal(o);haveCandidate_=false;f_.stableCycles=0U;f_.stopProven=f_.targetProven=false;}
            if(!haveCandidate_)
            {candidateMin_=candidateMax_=o.actualPulse;candidateCmdMin_=candidateCmdMax_=o.commandPulse;
             candidatePlanMin_=candidatePlanMax_=o.planningPulse;candidateIdentity_=o;candidateSinceUs_=o.monotonicUs;haveCandidate_=true;f_.stableCycles=0U;}
            if(o.actualPulse<candidateMin_)candidateMin_=o.actualPulse;
            if(o.actualPulse>candidateMax_)candidateMax_=o.actualPulse;
            if(o.commandPulse<candidateCmdMin_)candidateCmdMin_=o.commandPulse;
            if(o.commandPulse>candidateCmdMax_)candidateCmdMax_=o.commandPulse;
            if(o.planningPulse<candidatePlanMin_)candidatePlanMin_=o.planningPulse;
            if(o.planningPulse>candidatePlanMax_)candidatePlanMax_=o.planningPulse;
            if(candidateMax_-candidateMin_>tolerance||candidateCmdMax_-candidateCmdMin_>tolerance||candidatePlanMax_-candidatePlanMin_>tolerance)
            {BeginRapidStopProofRenewal(o);candidateMin_=candidateMax_=o.actualPulse;candidateCmdMin_=candidateCmdMax_=o.commandPulse;
             candidatePlanMin_=candidatePlanMax_=o.planningPulse;candidateSinceUs_=o.monotonicUs;f_.stableCycles=0U;f_.stopProven=f_.targetProven=false;}
            if(f_.stableCycles<c.settleCycles)++f_.stableCycles;
            if(f_.stableCycles>=c.settleCycles&&o.monotonicUs-candidateSinceUs_>=c.settleCycles*CycleUs&&
                o.monotonicUs-(targetPhase?f_.lastAppliedMonotonicUs:f_.stopAppliedMonotonicUs)>=c.settleCycles*CycleUs)
            {
                if(!f_.stopProven&&!f_.targetProven)f_.proofSequence=++nextProof_;
                if(targetPhase){f_.state=State::AtTarget;f_.targetProven=true;}
                else {if(f_.state!=State::Disarmed)f_.state=State::Held;f_.stopProven=true;}
            }
        }
    };
}
