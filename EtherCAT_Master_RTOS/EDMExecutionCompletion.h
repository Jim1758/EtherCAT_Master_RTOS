#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// EDM49: canonical terminal evidence for a local RT motion backend.
// This is a bounded, stateless evaluator, not a command path or a drive/NIC ACK.
// Each slot uses its own backend-native coordinate and paired rate units.
// Never combine linear and rotary magnitudes. The production adapter is Z only.
namespace EDM49
{
    constexpr std::size_t AxisSlots = 8U;
    enum class Backend : std::uint8_t { None, LocalShadow, RTShadow, LocalRTMotion };
    enum class Stage : std::uint8_t { StopSettled, Released, FreshReleased };
    enum class Status : std::uint8_t { Wait, Ready, Invalid };
    enum class AxisState : std::uint8_t { Other, Held, Disarmed };
    enum class AppliedKind : std::uint8_t { Other, Stop, Disarm };
    enum class Reason : std::uint8_t
    {
        None, Domain, Stage, Scope, AxisMask, MissingAxis, InvalidRule,
        Source, Fault, AwaitStop, Origin, Position, CommandRate, Target,
        StopStamp, DisarmStamp, Dwell, Proof, FreshFloor, AwaitFresh
    };
    struct Scope
    {
        std::uint64_t session=0U, run=0U, cache=0U, dispatch=0U;
        std::uint64_t axisMap=0U, config=0U, epoch=0U, ownerLease=0U;
    };
    inline bool ValidScope(const Scope& s) noexcept
    { return s.session && s.run && s.cache && s.dispatch && s.axisMap && s.config && s.epoch && s.ownerLease; }
    inline bool SameScope(const Scope& a,const Scope& b) noexcept
    {
        return a.session==b.session && a.run==b.run && a.cache==b.cache && a.dispatch==b.dispatch &&
            a.axisMap==b.axisMap && a.config==b.config && a.epoch==b.epoch && a.ownerLease==b.ownerLease;
    }
    struct Stamp { std::uint64_t sequence=0U, tick=0U, timeUs=0U; };
    inline bool ValidStamp(const Stamp& s) noexcept { return s.sequence && s.tick && s.timeUs; }
    inline bool SameStamp(const Stamp& a,const Stamp& b) noexcept
    { return a.sequence==b.sequence && a.tick==b.tick && a.timeUs==b.timeUs; }
    struct FreshFloor { std::uint64_t publication=0U, tick=0U, timeUs=0U; };
    struct AxisRule
    {
        double origin=0.0, positionTolerance=0.0, maximumCommandRate=0.0;
        std::uint64_t settleTicks=0U, settleUs=0U, proofFloor=0U, stopSourceTickFloor=0U;
        Stamp expectedStop{};
        std::uint64_t disarmSequence=0U, releasedProof=0U;
        bool requireTargetAtOrigin=false;
    };
    struct AxisEvidence
    {
        bool present=false;
        std::uint8_t axisIndex=255U;
        bool sourceCurrent=false;
        AxisState state=AxisState::Other;
        AppliedKind appliedKind=AppliedKind::Other;
        bool capHeld=false, stopLatched=false, latchedFault=false, forceZeroOutput=false;
        bool stopProven=false, frameCurrent=false, originValid=false, idle=false;
        std::uint64_t stableTicks=0U;
        double origin=0.0, target=0.0, actual=0.0, commanded=0.0, planned=0.0, commandRate=0.0;
        Stamp stop{}, applied{};
        std::uint64_t proof=0U;
    };
    struct Request
    {
        Backend backend=Backend::None;
        Stage stage=Stage::StopSettled;
        Scope scope{};
        std::uint32_t selectedAxisMask=0U;
        std::array<AxisRule,AxisSlots> axes{};
        FreshFloor freshFloor{};
    };
    struct Evidence
    {
        Backend backend=Backend::None;
        Scope scope{};
        std::uint64_t publication=0U, tick=0U, timeUs=0U;
        std::array<AxisEvidence,AxisSlots> axes{};
    };
    struct Result
    {
        Status status=Status::Invalid;
        Reason reason=Reason::None;
        std::uint32_t selectedMask=0U, provenMask=0U;
        std::uint8_t failingAxis=255U;
        bool Ready() const noexcept { return status==Status::Ready; }
        constexpr bool DriveAcknowledged() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    inline Result Evaluate(const Request& request,const Evidence& evidence) noexcept
    {
        Result result{}; result.selectedMask=request.selectedAxisMask;
        const auto invalid=[&result](Reason reason) noexcept -> Result
        { result.status=Status::Invalid; result.reason=reason; return result; };
        const auto wait=[&result](Reason reason) noexcept -> Result
        { result.status=Status::Wait; result.reason=reason; return result; };
        if(request.backend!=Backend::LocalRTMotion || evidence.backend!=Backend::LocalRTMotion)
            return invalid(Reason::Domain);
        if(request.stage!=Stage::StopSettled && request.stage!=Stage::Released && request.stage!=Stage::FreshReleased)
            return invalid(Reason::Stage);
        if(!ValidScope(request.scope) || !SameScope(request.scope,evidence.scope)) return invalid(Reason::Scope);
        if(!request.selectedAxisMask || (request.selectedAxisMask & ~0xFFU)) return invalid(Reason::AxisMask);
        if(!evidence.publication || !evidence.tick || !evidence.timeUs) return invalid(Reason::Source);
        for(std::size_t axis=0U;axis<AxisSlots;++axis)
        {
            const auto bit=1U<<axis;
            if(!(request.selectedAxisMask & bit)) continue;
            result.failingAxis=static_cast<std::uint8_t>(axis);
            const auto& rule=request.axes[axis]; const auto& value=evidence.axes[axis];
            if(!value.present || value.axisIndex!=axis) return invalid(Reason::MissingAxis);
            if(!std::isfinite(rule.origin) || !std::isfinite(rule.positionTolerance) || rule.positionTolerance<=0.0 ||
                !std::isfinite(rule.maximumCommandRate) || rule.maximumCommandRate<=0.0 ||
                !rule.settleTicks || !rule.settleUs) return invalid(Reason::InvalidRule);
            if(!value.sourceCurrent) return invalid(Reason::Source);
            if(value.latchedFault || value.forceZeroOutput) return invalid(Reason::Fault);
            if(request.stage==Stage::StopSettled)
            {
                // A decelerating backend is not an invalid endpoint. Preserve
                // the old Wait behavior until its actual Stop proof is settled.
                if(value.state!=AxisState::Held || !value.stopLatched || !value.stopProven ||
                    !value.idle || value.stableTicks<rule.settleTicks) return wait(Reason::AwaitStop);
                if(!value.capHeld) return invalid(Reason::Dwell);
                if(!value.originValid) return invalid(Reason::Origin);
            }
            else if(value.state!=AxisState::Disarmed || value.capHeld || !value.stopProven ||
                !value.idle || value.stableTicks<rule.settleTicks) return invalid(Reason::Dwell);
            if(!value.frameCurrent || !std::isfinite(value.origin) || value.origin!=rule.origin)
                return invalid(Reason::Origin);
            if(!std::isfinite(value.actual) || !std::isfinite(value.commanded) || !std::isfinite(value.planned) ||
                std::fabs(value.actual-rule.origin)>rule.positionTolerance ||
                std::fabs(value.commanded-rule.origin)>rule.positionTolerance ||
                std::fabs(value.planned-rule.origin)>rule.positionTolerance) return invalid(Reason::Position);
            if(!std::isfinite(value.commandRate) || std::fabs(value.commandRate)>rule.maximumCommandRate)
                return invalid(Reason::CommandRate);
            if(!ValidStamp(value.stop) || !ValidStamp(value.applied)) return invalid(Reason::StopStamp);
            if(evidence.tick<value.stop.tick || evidence.timeUs<value.stop.timeUs ||
                evidence.tick-value.stop.tick<rule.settleTicks || evidence.timeUs-value.stop.timeUs<rule.settleUs)
                return invalid(Reason::Dwell);
            if(!value.proof || value.proof<=rule.proofFloor) return invalid(Reason::Proof);
            if(request.stage==Stage::StopSettled)
            {
                if(value.appliedKind!=AppliedKind::Stop || !SameStamp(value.applied,value.stop) ||
                    value.stop.tick<=rule.stopSourceTickFloor) return invalid(Reason::StopStamp);
            }
            else
            {
                if(!ValidStamp(rule.expectedStop) || !SameStamp(value.stop,rule.expectedStop))
                    return invalid(Reason::StopStamp);
                if(!rule.disarmSequence || value.appliedKind!=AppliedKind::Disarm ||
                    value.applied.sequence!=rule.disarmSequence || value.applied.sequence<=value.stop.sequence ||
                    value.applied.tick<=value.stop.tick ||
                    value.applied.timeUs<=value.stop.timeUs || evidence.tick<value.applied.tick ||
                    evidence.timeUs<value.applied.timeUs) return invalid(Reason::DisarmStamp);
                if(!rule.releasedProof || value.proof!=rule.releasedProof) return invalid(Reason::Proof);
                // Disarm clears originValid in the physical policy. The frozen
                // first origin and the retained exact Stop tuple remain evidence.
                if(!std::isfinite(value.target) || (rule.requireTargetAtOrigin && value.target!=rule.origin))
                    return invalid(Reason::Target);
            }
            result.provenMask|=bit;
        }
        result.failingAxis=255U;
        if(request.stage==Stage::FreshReleased)
        {
            const auto& floor=request.freshFloor;
            if(!floor.publication || !floor.tick || !floor.timeUs) return invalid(Reason::FreshFloor);
            if(evidence.publication<floor.publication || evidence.tick<floor.tick || evidence.timeUs<floor.timeUs)
                return invalid(Reason::FreshFloor);
            if(evidence.publication==floor.publication || evidence.tick==floor.tick || evidence.timeUs==floor.timeUs)
                return wait(Reason::AwaitFresh);
            for(std::size_t axis=0U;axis<AxisSlots;++axis)
                if((request.selectedAxisMask & (1U<<axis)) &&
                    (evidence.tick<=evidence.axes[axis].applied.tick || evidence.timeUs<=evidence.axes[axis].applied.timeUs))
                    return wait(Reason::AwaitFresh);
        }
        result.status=Status::Ready; result.reason=Reason::None; return result;
    }
    inline const char* StageName(Stage value) noexcept
    {
        switch(value) { case Stage::StopSettled:return "STOP_SETTLED"; case Stage::Released:return "RELEASED";
            case Stage::FreshReleased:return "FRESH_RELEASED"; } return "UNKNOWN";
    }
    inline const char* StatusName(Status value) noexcept
    { switch(value) { case Status::Wait:return "WAIT"; case Status::Ready:return "READY"; case Status::Invalid:return "INVALID"; } return "UNKNOWN"; }
    inline const char* ReasonName(Reason value) noexcept
    {
        switch(value)
        {
#define EDM49_REASON(name,label) case Reason::name:return label
        EDM49_REASON(None,"NONE"); EDM49_REASON(Domain,"DOMAIN"); EDM49_REASON(Stage,"STAGE");
        EDM49_REASON(Scope,"SCOPE"); EDM49_REASON(AxisMask,"AXIS_MASK"); EDM49_REASON(MissingAxis,"MISSING_AXIS");
        EDM49_REASON(InvalidRule,"INVALID_RULE"); EDM49_REASON(Source,"SOURCE"); EDM49_REASON(Fault,"FAULT");
        EDM49_REASON(AwaitStop,"AWAIT_STOP"); EDM49_REASON(Origin,"ORIGIN"); EDM49_REASON(Position,"POSITION");
        EDM49_REASON(CommandRate,"COMMAND_RATE"); EDM49_REASON(Target,"TARGET"); EDM49_REASON(StopStamp,"STOP_STAMP");
        EDM49_REASON(DisarmStamp,"DISARM_STAMP"); EDM49_REASON(Dwell,"DWELL"); EDM49_REASON(Proof,"PROOF");
        EDM49_REASON(FreshFloor,"FRESH_FLOOR"); EDM49_REASON(AwaitFresh,"AWAIT_FRESH");
#undef EDM49_REASON
        } return "UNKNOWN";
    }
    static_assert(sizeof(Request)+sizeof(Evidence)<=4096U,"EDM49 completion evidence must remain bounded.");
}
