#pragma once
// EDM267 FIX1: private implementation helpers for independently compiled NC units.
// Only stateless definitions are shared; observer storage remains single-copy.
// CG / EDM01 / EDM02: NC-owned simulation tests. No Motion or IO writes.
#include "NCManager.h"
#include "NCG76DiagnosticPrint.h"
#include "GlobalConfig.h"
#include "EDM147PhysicalGapEvidence.h"
#include "EDM149VoltageMotion.h"
#include "NCExpressionResolver.h"
#include "NCXYZCUVFeedScope.h"
#include "EDMCNCRetainedRTFixture.h"
#include "EDMCoordinatedRTFixture.h"
#include "NCMainThreadStackPolicy.h"
#include "AlarmManager.h"
#include "EDMConditionService.h"
#include "NCGCodeSemantics.h"
#include "EDMPhysicalServoFlushContract.h"
#include "EDMPhysicalRetreatContract.h"
#include "EDMPhysicalBidirectionalContract.h"
#include "EDMPhysicalBidirectionalFlushContract.h"
#include "EDMPhysicalRecipeSequence.h"
#include "EDMPhysicalRecipePreflight.h"
#include "EDM137ConfirmedUnits.h"
#include "EDM145SourceDiagnostics.h"
#include "EDM192ArmAdmission.h"
#include "EDM206SyncHistory.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <windows.h>
#include <rtapi.h>
#include <RtssApi.h>

namespace NCGapDetail
{
    // Stateful observers are defined once in NCManager_GapDiagnostics.cpp.
    void PollEDM206History(MotionCore& motion,NCState ncState) noexcept;
    void PrimeEDM192AdmissionObserver(const EDM192Arm::Scope& scope) noexcept;
    bool PollEDM192AdmissionObserver(const EDM192Arm::Scope& scope) noexcept;
    void PrintEDM173InputFailure(const MotionCore& motion,std::uint64_t session,
        std::uint64_t guardTick) noexcept;
    void PrintEDM145SourceFailure(std::uint64_t session) noexcept;
}

namespace
{
    // Explicit opt-ins preserve the older timed physical-AD profiles.
    // EDM132 freezes E6 in millimetres; its I10 remains a fixed envelope only.
    bool E148AuthoredLineArm(unsigned arm) noexcept { return arm == 226U || arm == 228U; }
    bool E146PhysicalMotionArm(unsigned arm) noexcept { return arm == 222U || arm == 224U; }
    bool E141MotionArm(unsigned arm) noexcept { return arm == 218U || arm == 220U || E146PhysicalMotionArm(arm) || E148AuthoredLineArm(arm); }
    bool E139DiagnosticArm(unsigned arm) noexcept { return arm == 210U || arm == 212U || arm == 214U || arm == 216U || arm == 218U || arm == 220U || E148AuthoredLineArm(arm); }
    bool E140AssignmentArm(unsigned arm) noexcept { return arm == 214U || arm == 216U || E141MotionArm(arm); }
    bool E137FlushStageArm(unsigned arm) noexcept { return EDM173::IsArm(arm) || EDM149::IsTestArm(arm) || arm == 202U || arm == 204U || arm == 206U || arm == 208U || E139DiagnosticArm(arm) || E146PhysicalMotionArm(arm); }
    bool E138CrossSegmentArm(unsigned arm) noexcept { return EDM149::IsTestArm(arm) || arm == 206U || arm == 208U || E139DiagnosticArm(arm) || E146PhysicalMotionArm(arm); }
    bool E136CurveArm(unsigned arm) noexcept { return arm == 198U || arm == 200U || E137FlushStageArm(arm); }
    bool E135ReferenceArm(unsigned arm) noexcept { return arm == 194U || arm == 196U || E136CurveArm(arm); }
    bool E134MachiningArm(unsigned arm) noexcept { return arm == 190U || arm == 192U || E135ReferenceArm(arm); }
    bool E133DeepArm(unsigned arm) noexcept { return arm == 186U || arm == 188U || E134MachiningArm(arm); }
    bool E132HeightArm(unsigned arm) noexcept { return arm == 182U || arm == 184U || E133DeepArm(arm); }
    // The ARM J50 cap is authored; the selected E7 actual value scales that cap.
    bool E131SpeedArm(unsigned arm) noexcept { return arm == 178U || arm == 180U || E132HeightArm(arm); }
    bool E130TimedArm(unsigned arm) noexcept
    { return arm == 174U || arm == 176U || E131SpeedArm(arm); }
    bool E130CanonicalArm(unsigned arm) noexcept { return arm == 174U || arm == 178U || arm == 182U || arm == 186U || arm == 190U || arm == 194U || arm == 198U || arm == 202U || arm == 206U || arm == 210U || arm == 214U || arm == 218U || arm == 222U || arm == 226U; }
    unsigned E149HistoryFixture(unsigned arm) noexcept
    { return EDM149::IsSixVisibleFlushTestArm(arm) ? EDM149::SixVisibleFlushHistoryFixture : EDM149::IsXYZVisibleFlushTestArm(arm) ? EDM149::XYZVisibleFlushHistoryFixture : EDM149::IsVisibleFlushTestArm(arm) ? EDM149::VisibleFlushHistoryFixture : EDM149::IsPauseTestArm(arm) ? EDM149::PauseHistoryFixture : EDM149::IsProgramTestArm(arm) ? EDM149::ProgramHistoryFixture : EDM149::IsSparseArcTestArm(arm) ? EDM149::SparseArcHistoryFixture : EDM149::IsSparseTestArm(arm) ? EDM149::SparseHistoryFixture : EDM149::IsOrderTestArm(arm) ? EDM149::OrderHistoryFixture : EDM149::IsArcTestArm(arm) ? EDM149::ArcHistoryFixture : EDM149::IsSixTestArm(arm) ? EDM149::SixHistoryFixture : EDM149::IsXYZTestArm(arm) ? EDM149::XYZHistoryFixture : EDM149::IsZTestArm(arm) ? EDM149::ZHistoryFixture : (arm == 158U || arm == 166U || arm == 170U || E130CanonicalArm(arm)) ? 0U : 1U; }
    EDM116::Fixture E149ModalFixture(unsigned fixture) noexcept
    { return fixture == EDM149::SixVisibleFlushHistoryFixture ? EDM116::Fixture::ModalSixVisibleFlush167 : fixture == EDM149::XYZVisibleFlushHistoryFixture ? EDM116::Fixture::ModalXYZVisibleFlush159 : fixture == EDM149::VisibleFlushHistoryFixture ? EDM116::Fixture::ModalVisibleFlush158 : fixture == EDM149::PauseHistoryFixture ? EDM116::Fixture::ModalPause157 : fixture == EDM149::ProgramHistoryFixture ? EDM116::Fixture::ModalProgram156 : fixture == EDM149::SparseArcHistoryFixture ? EDM116::Fixture::ModalSparseArc155 : fixture == EDM149::SparseHistoryFixture ? EDM116::Fixture::ModalSparse154 : fixture == EDM149::OrderHistoryFixture ? EDM116::Fixture::ModalOrder153 : fixture == EDM149::ArcHistoryFixture ? EDM116::Fixture::ModalArc152 : fixture == EDM149::SixHistoryFixture ? EDM116::Fixture::ModalSix151 : fixture == EDM149::XYZHistoryFixture ? EDM116::Fixture::ModalXYZ150 : fixture == EDM149::ZHistoryFixture ? EDM116::Fixture::ModalZ149 : fixture ? EDM116::Fixture::ModalComplement127 : EDM116::Fixture::Modal127; }
    unsigned E149ControlSource(unsigned arm) noexcept
    { return EDM149::IsTestArm(arm) ? EDM149::SourceForArm(arm) : E139DiagnosticArm(arm) ? 1U : 0U; }
    bool E149PhysicalExpected(unsigned arm) noexcept
    { return !EDM149::IsSoftwareSource(EDM149::SourceForArm(arm)); }
    // A per-event observation of the real B2 cursor excursion. General GAP
    // traversal counters also include voltage retreats, so they cannot prove
    // that an E5 flush itself crossed an NC seam.
    void EmitEDM161B2History(std::uint64_t session, const EDM128::MotionFeedback& timed,
        const EDM130::Recipe& expected, const char* event, bool identityMatch,
        unsigned anchorG, unsigned targetG) noexcept
    {
        const auto& e = timed.coordinator;
        if (!identityMatch || !timed.enabled || !timed.workTimeFlush || timed.session != session ||
            e.policy.mode != EDM116::Mode::B2 || !e.eventIndex) return;
        const bool recipeMatches = EDM130::SameRecipe(timed.timerRecipe, expected);
        const bool sourcesMatch = EDM126::ValidSource(e.anchor.cursorSource) && EDM126::ValidSource(e.targetSource) &&
            e.anchor.cursorSource.scope.session == session && e.targetSource.scope.session == session &&
            EDM109::SameScope(e.anchor.cursorSource.scope, e.targetSource.scope);
        const bool planned = e.anchorCaptured && e.targetCaptured && sourcesMatch &&
            e.targetSource.ordinal < e.anchor.cursorSource.ordinal;
        const bool proof = recipeMatches && EDM130::ValidRecipe(expected) &&
            EDM116::SamePolicy(timed.rowPolicy, e.policy) &&
            EDM128::ValidHeightProof(e, EDM130::FrozenJumpHeightMm(expected), expected.deepFlushCycleInterval,
                EDM130::FrozenDeepMultiplier(expected), EDM130::HasBoundDeep(expected));
        const bool verified = planned && proof && e.topCaptured && e.exactReturn && e.rowEventComplete &&
            e.work.completedEvents >= e.eventIndex;
        char record[481]{};
        const int length = std::snprintf(record, sizeof(record),
            "[EDM161] event=B2_HISTORY result=%.24s session=%llu eventIndex=%llu phase=%s anchorRow=%llu anchorPC=%d anchorG=%u targetRow=%llu targetPC=%d targetG=%u planned=%u top=%u exactReturn=%u complete=%u recipeMatches=%u proofValid=%u verifiedCross=%u authority=NONE\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(session),
            static_cast<unsigned long long>(e.eventIndex), EDM128::PhaseName(e.phase),
            static_cast<unsigned long long>(e.anchor.cursorSource.ordinal), e.anchor.cursorSource.sourcePC, anchorG,
            static_cast<unsigned long long>(e.targetSource.ordinal), e.targetSource.sourcePC, targetG,
            planned ? 1U : 0U, e.topCaptured ? 1U : 0U, e.exactReturn ? 1U : 0U, e.rowEventComplete ? 1U : 0U,
            recipeMatches ? 1U : 0U, proof ? 1U : 0U, verified ? 1U : 0U);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM161] event=LOG_FORMAT_ERROR part=B2_HISTORY\n");
    }

    void EmitEDM137FlushStage(std::uint64_t session, const EDM128::MotionFeedback& timed,
        const char* event, bool recipeMatches) noexcept
    {
        char record[481]{};
        const auto& e = timed.coordinator;
        const int length = std::snprintf(record, sizeof(record),
            "[EDM137] event=FLUSH_STAGE result=%.24s session=%llu row=%u B=%u eventIndex=%llu phase=%s stage=%s distanceMm=%.9g targetMmMin=%.9g initialMm=%.9g finalMm=%.9g heightMm=%.9g proofValid=%u recipeMatches=%u powerControl=NC_PLC_S92\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(session), timed.completedRows,
            static_cast<unsigned>(timed.rowPolicy.mode), static_cast<unsigned long long>(e.eventIndex),
            EDM128::PhaseName(e.phase), EDM128::SpeedStageName(e.speedStage), e.stageDistanceMm,
            e.stageTargetMmSec * 60.0, e.initialStageDistanceMm, e.finalStageDistanceMm, e.effectiveJumpHeightMm,
            EDM128::ValidSpeedProof(e, timed.authorizedRows ? timed.timerRecipe.flushStages : EDM130::FlushStages{}) ? 1U : 0U,
            recipeMatches ? 1U : 0U);
        if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record)) RtPrintf("%s", record);
        else RtPrintf("[EDM137] event=LOG_FORMAT_ERROR part=FLUSH_STAGE\n");
        if (EDM130::IsVisibleFlush(timed.timerRecipe))
            RtPrintf("[EDM158] event=VISIBLE_FLUSH_STAGE result=%s session=%llu phase=%s stage=%s triggered=%llu completed=%llu exactReturn=%u rowEventComplete=%u anchorZBits=%llu topZBits=%llu distanceBits=%llu targetMmSecBits=%llu recipeMatches=%u cycles=EXACTLY_ONE then=REMAINING_NC hold=CANCEL_RESET_REQUIRED S92=INHIBITED S93=INHIBITED\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(session),
                EDM128::PhaseName(e.phase), EDM128::SpeedStageName(e.speedStage),
                static_cast<unsigned long long>(e.work.triggeredEvents), static_cast<unsigned long long>(e.work.completedEvents),
                e.exactReturn ? 1U : 0U, e.rowEventComplete ? 1U : 0U,
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.anchor.positionPulse[2])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.topPositionPulse[2])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.stageDistanceMm)),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.stageTargetMmSec)), recipeMatches ? 1U : 0U);
        if (EDM130::IsVisibleFlush159(timed.timerRecipe))
            RtPrintf("[EDM159] event=VISIBLE_FLUSH_STAGE result=%s session=%llu phase=%s stage=%s triggered=%llu completed=%llu exactReturn=%u rowEventComplete=%u anchorZBits=%llu topZBits=%llu distanceBits=%llu targetMmSecBits=%llu recipeMatches=%u cycles=EXACTLY_ONE then=REMAINING_NC hold=CANCEL_RESET_REQUIRED S92=INHIBITED S93=INHIBITED\n",
                event ? event : "UNKNOWN", static_cast<unsigned long long>(session),
                EDM128::PhaseName(e.phase), EDM128::SpeedStageName(e.speedStage),
                static_cast<unsigned long long>(e.work.triggeredEvents), static_cast<unsigned long long>(e.work.completedEvents),
                e.exactReturn ? 1U : 0U, e.rowEventComplete ? 1U : 0U,
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.anchor.positionPulse[2])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.topPositionPulse[2])),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.stageDistanceMm)),
                static_cast<unsigned long long>(EDM130::CurveDoubleBits(e.stageTargetMmSec)), recipeMatches ? 1U : 0U);
    }


    // NC-owner diagnostics only. Format locally, then use the same one-string
    // console route as the existing bounded EDM evidence records.
    void EmitEDM113Diagnostic(const char* event, const char* part, int code, int length) noexcept
    {
        char record[192]{};
        const int formatted = std::snprintf(record, sizeof(record),
            "[EDM113] event=%.24s logger=FIX1 part=%.32s rc=%d length=%d\n", event, part, code, length);
        if (formatted > 0 && static_cast<std::size_t>(formatted) < sizeof(record))
            RtPrintf("%s", record);
        else
            RtPrintf("%s", "[EDM113] event=LOG_FORMAT_ERROR logger=FIX1 part=LOGGER_DIAGNOSTIC\n");
    }
    void EmitEDM113Record(const char* record, int length, const char* part) noexcept
    {
        if (length <= 0 || length > 480)
        {
            EmitEDM113Diagnostic("LOG_FORMAT_ERROR", part, length, length);
            return;
        }
        const int written = RtPrintf("%s", record);
        if (written < 0)
            EmitEDM113Diagnostic("LOG_OUTPUT_ERROR", part, written, length);
    }

    // Each part is one bounded native console record. Shared keys identify one
    // copied bank; captured scope describes provenance, never current NC state.
    void LogEDM65PhysicalRecipeShadow(const EDM65::Snapshot& value) noexcept
    {
        char text[480]{};
        const auto publication = static_cast<unsigned long long>(value.publication);
        const auto recipePub = static_cast<unsigned long long>(value.recipePublication);
        const auto cycle = static_cast<unsigned long long>(value.cycle);
        const auto& request = value.request;
        const auto& recipe = request.recipe;
        const auto& key = recipe.binding;
        const bool usable = value.readerState == EDM64::ReaderState::Fresh &&
            value.state == EDM65::State::Valid && value.output.valid;
        int length = std::snprintf(text, sizeof(text),
            "[EDM65] event=RT_RECIPE_SHADOW part=STATE publication=%llu recipePub=%llu cycle=%llu state=%s reason=%s reader=%s kernelFault=%s usable=%u outputValid=%u mode=FROZEN_SIM_RECIPE_PHYSICAL_SHADOW appliedToMotion=0 physicalPermit=0 discharge=0\n",
            publication, recipePub, cycle, EDM65::StateName(value.state), EDM65::ReasonName(value.reason),
            EDM64::ReaderStateName(value.readerState), EDM64::FaultName(value.output.fault),
            usable ? 1U : 0U, value.output.valid ? 1U : 0U);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM65] event=RT_RECIPE_SHADOW part=STATE publication=%llu recipePub=%llu cycle=%llu reason=LOG_FORMAT appliedToMotion=0 physicalPermit=0 discharge=0\n", publication, recipePub, cycle);
        length = std::snprintf(text, sizeof(text),
            "[EDM65] event=RT_RECIPE_SHADOW part=SCOPE publication=%llu recipePub=%llu cycle=%llu session=%llu run=%llu cache=%llu dispatch=%llu epoch=%llu owner=%u ownerGen=%llu capturedUs=%llu qpcHz=%llu capturedNotActive=1 captureClock=HOST_NC_RECEIPT\n",
            publication, recipePub, cycle,
            static_cast<unsigned long long>(request.scope.session), static_cast<unsigned long long>(request.scope.run),
            static_cast<unsigned long long>(request.scope.cache), static_cast<unsigned long long>(request.scope.dispatch),
            static_cast<unsigned long long>(request.scope.epoch), static_cast<unsigned>(request.scope.owner),
            static_cast<unsigned long long>(request.scope.ownerGeneration),
            static_cast<unsigned long long>(request.capturedUs), static_cast<unsigned long long>(request.qpcFrequency));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM65] event=RT_RECIPE_SHADOW part=SCOPE publication=%llu recipePub=%llu cycle=%llu reason=LOG_FORMAT appliedToMotion=0 physicalPermit=0 discharge=0\n", publication, recipePub, cycle);
        const bool recipeFinite = std::isfinite(recipe.referenceV) && std::isfinite(recipe.overridePercent) &&
            std::isfinite(recipe.curve.maxFeedMmPerMin) && std::isfinite(recipe.curve.maxRetreatMmPerMin);
        length = recipeFinite ? std::snprintf(text, sizeof(text),
            "[EDM65] event=RT_RECIPE_SHADOW part=RECIPE publication=%llu recipePub=%llu cycle=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu processRev=%u tuningGen=%llu referenceV=%.9g overridePct=%.9g feedCap=%.9g retreatCap=%.9g curve=FROZEN_E8_E9_PN\n",
            publication, recipePub, cycle, key.tableId, static_cast<unsigned>(key.eCode), key.catalogRevision,
            key.tableRevision, key.profileId, key.definitionRevision, static_cast<unsigned long long>(key.generation),
            key.processRevision, static_cast<unsigned long long>(key.tuningGeneration), recipe.referenceV,
            recipe.overridePercent, recipe.curve.maxFeedMmPerMin, recipe.curve.maxRetreatMmPerMin) : -1;
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM65] event=RT_RECIPE_SHADOW part=RECIPE publication=%llu recipePub=%llu cycle=%llu reason=%s appliedToMotion=0 physicalPermit=0 discharge=0\n", publication, recipePub, cycle, recipeFinite ? "LOG_FORMAT" : "NONFINITE_VALUES");
        const bool valuesFinite = std::isfinite(value.output.rawVoltageV) && std::isfinite(value.output.filteredVoltageV) &&
            std::isfinite(value.output.targetMmPerMin) && std::isfinite(value.output.speedMmPerMin);
        length = valuesFinite ? std::snprintf(text, sizeof(text),
            "[EDM65] event=RT_RECIPE_SHADOW part=VALUES publication=%llu recipePub=%llu cycle=%llu seq=%llu sourceGen=%llu sourceCapturedUs=%llu observedUs=%llu readerAgeUs=%llu ageKnown=%u rawV=%.9g filteredV=%.9g targetMmMin=%.9g advisoryMmMin=%.9g short=%u feedInhibit=%u appliedToMotion=0 physicalPermit=0 discharge=0\n",
            publication, recipePub, cycle, static_cast<unsigned long long>(value.sourceSequence),
            static_cast<unsigned long long>(value.sourceGeneration), static_cast<unsigned long long>(value.sourceCapturedUs),
            static_cast<unsigned long long>(value.observedUs), static_cast<unsigned long long>(value.sourceAgeUs),
            value.sourceAgeKnown ? 1U : 0U, value.output.rawVoltageV, value.output.filteredVoltageV,
            value.output.targetMmPerMin, value.output.speedMmPerMin,
            value.output.shortActive ? 1U : 0U, value.output.feedInhibited ? 1U : 0U) : -1;
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM65] event=RT_RECIPE_SHADOW part=VALUES publication=%llu recipePub=%llu cycle=%llu reason=%s appliedToMotion=0 physicalPermit=0 discharge=0\n", publication, recipePub, cycle, valuesFinite ? "LOG_FORMAT" : "NONFINITE_VALUES");
    }

    // EDM66: these are historical PDO observations for the frozen recipe.
    // Reader freshness controls current usability, never erases earlier evidence.
    // Every part retains the same copied-bank keys; no RT-thread console work.
    void LogEDM66PhysicalCoverage(const EDM65::Snapshot& value) noexcept
    {
        char text[480]{};
        const auto publication = static_cast<unsigned long long>(value.publication);
        const auto recipePub = static_cast<unsigned long long>(value.recipePublication);
        const auto session = static_cast<unsigned long long>(value.request.scope.session);
        const auto& c = value.coverage;
        const bool usable = value.readerState == EDM64::ReaderState::Fresh &&
            value.state == EDM65::State::Valid && value.output.valid;
        int length = std::snprintf(text, sizeof(text),
            "[EDM66] event=PHYSICAL_COVERAGE part=WINDOW publication=%llu recipePub=%llu session=%llu hasSamples=%u saturated=%u reader=%s liveUsable=%u firstUs=%llu lastUs=%llu historyOnly=1 scope=FROZEN_RECIPE appliedToMotion=0 physicalPermit=0 discharge=0\n",
            publication, recipePub, session, c.hasSamples ? 1U : 0U, c.saturated ? 1U : 0U,
            EDM64::ReaderStateName(value.readerState), usable ? 1U : 0U,
            static_cast<unsigned long long>(c.firstObservedUs), static_cast<unsigned long long>(c.lastObservedUs));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM66] event=PHYSICAL_COVERAGE part=WINDOW publication=%llu recipePub=%llu session=%llu reason=LOG_FORMAT historyOnly=1\n", publication, recipePub, session);
        length = std::snprintf(text, sizeof(text),
            "[EDM66] event=PHYSICAL_COVERAGE part=SAMPLES publication=%llu recipePub=%llu session=%llu firstSeq=%llu lastSeq=%llu validTicks=%llu freshSamples=%llu repeatSamples=%llu historyOnly=1 source=PHYSICAL sampleUnit=HOST_PDO_RECEIPT\n",
            publication, recipePub, session, static_cast<unsigned long long>(c.firstSourceSequence),
            static_cast<unsigned long long>(c.lastSourceSequence), static_cast<unsigned long long>(c.validTicks),
            static_cast<unsigned long long>(c.freshSamples), static_cast<unsigned long long>(c.repeatSamples));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM66] event=PHYSICAL_COVERAGE part=SAMPLES publication=%llu recipePub=%llu session=%llu reason=LOG_FORMAT historyOnly=1\n", publication, recipePub, session);
        length = std::snprintf(text, sizeof(text),
            "[EDM66] event=PHYSICAL_COVERAGE part=RESPONSE publication=%llu recipePub=%llu session=%llu advanceTicks=%llu retreatTicks=%llu zeroTicks=%llu shortTicks=%llu feedInhibitTicks=%llu historyOnly=1 direction=TARGET_NOT_AXIS\n",
            publication, recipePub, session, static_cast<unsigned long long>(c.advanceTicks),
            static_cast<unsigned long long>(c.retreatTicks), static_cast<unsigned long long>(c.zeroTicks),
            static_cast<unsigned long long>(c.shortTicks), static_cast<unsigned long long>(c.feedInhibitTicks));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM66] event=PHYSICAL_COVERAGE part=RESPONSE publication=%llu recipePub=%llu session=%llu reason=LOG_FORMAT historyOnly=1\n", publication, recipePub, session);
        const bool finite = std::isfinite(c.minRawV) && std::isfinite(c.maxRawV);
        length = finite ? std::snprintf(text, sizeof(text),
            "[EDM66] event=PHYSICAL_COVERAGE part=SHORT_RANGE publication=%llu recipePub=%llu session=%llu shortEntries=%llu shortExits=%llu initialShort=%u lastShort=%u minRawV=%.9g maxRawV=%.9g historyOnly=1 initialShortIsEntry=0\n",
            publication, recipePub, session, static_cast<unsigned long long>(c.shortEntries),
            static_cast<unsigned long long>(c.shortExits), c.initialShort ? 1U : 0U, c.lastShort ? 1U : 0U,
            c.minRawV, c.maxRawV) : -1;
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM66] event=PHYSICAL_COVERAGE part=SHORT_RANGE publication=%llu recipePub=%llu session=%llu reason=%s historyOnly=1\n", publication, recipePub, session, finite ? "LOG_FORMAT" : "NONFINITE_VALUES");
    }

    constexpr std::uint32_t GapCaseCount = 16U;
    constexpr std::uint32_t GapAllCases = (1U << GapCaseCount) - 1U;
    constexpr std::uint64_t GapCaseMs = 750ULL;
    constexpr std::uint64_t GapMaxServiceGapMs = 250ULL;
    // Only HOLD resume may await a later owner observation. Initial P9 entry
    // retains its immediate Ready gate and the 10 s test starts at BEGIN.
    constexpr std::uint64_t EDMLiveResumeMaxWaitMs = 250ULL;
    bool EDMLiveResumeGapCurrent(const EDMGapInput::Snapshot& gap, std::uint64_t nowMs) noexcept
    {
        const bool physical = gap.configuredSource == EDMGap::Source::PHYSICAL;
        return (physical || gap.configuredSource == EDMGap::Source::SIMULATED) &&
            gap.profileRevision != 0U && gap.maxAgeMs != 0U && gap.maxAgeMs <= 1000U &&
            gap.configValid && gap.ownerClockValid && gap.liveVoltageValid && gap.ageKnown &&
            gap.gap.configured && gap.gap.quality == EDMGap::Quality::VALID &&
            gap.gap.source == gap.configuredSource && gap.sampleSequence != 0ULL &&
            gap.gap.sequence == gap.sampleSequence && gap.gap.sampledAtMs == gap.sampledAtMs &&
            gap.gap.voltageMv == gap.voltageMv && gap.observedAtMs <= nowMs &&
            gap.gap.observedAtMs <= nowMs && gap.sampledAtMs <= gap.observedAtMs &&
            gap.sampledAtMs <= gap.gap.observedAtMs && gap.ageMs == gap.observedAtMs - gap.sampledAtMs &&
            nowMs - gap.sampledAtMs <= gap.maxAgeMs && nowMs - gap.observedAtMs <= gap.maxAgeMs &&
            (!physical || (gap.adIndex != 0U && gap.calibrationRevision != 0U &&
                gap.calibrationConfirmed && gap.rawAvailable && gap.boardVoltageValid &&
                gap.inputStatus == EDMGapInput::InputStatus::Valid));
    }
    bool EDMLiveResumeSameSource(const EDMGapInput::Snapshot& current,
        const EDMGapInput::Snapshot& floor) noexcept
    {
        return current.configuredSource == floor.configuredSource &&
            current.gap.source == floor.configuredSource && current.profileRevision == floor.profileRevision &&
            current.maxAgeMs == floor.maxAgeMs &&
            (floor.configuredSource != EDMGap::Source::PHYSICAL ||
                (current.adIndex == floor.adIndex && current.deviceId == floor.deviceId &&
                 current.channelId == floor.channelId && current.pdoOffset == floor.pdoOffset &&
                 current.calibrationRevision == floor.calibrationRevision &&
                 current.calibrationConfirmed == floor.calibrationConfirmed));
    }
    constexpr std::uint64_t EDMProcessCaseMs = 500ULL;
    constexpr std::uint32_t EDMProcessAllCases =
        (1U << EDMProcessSimulationTest::CaseCount) - 1U;
    static_assert(EDMProcessSimulationTest::CaseCount > 0U &&
        EDMProcessSimulationTest::CaseCount < 32U, "EDM01 coverage mask must fit.");
    constexpr std::uint32_t EDMFeedRetreatAllCases =
        (1U << EDMFeedRetreatSimulationTest::CaseCount) - 1U;
    static_assert(EDMFeedRetreatSimulationTest::CaseCount > 0U &&
        EDMFeedRetreatSimulationTest::CaseCount < 32U, "EDM02 coverage mask must fit.");
    constexpr std::uint32_t EDMAcquisitionAllCases =
        (1U << EDMGapAcquisitionSimulationTest::CaseCount) - 1U;
    static_assert(EDMGapAcquisitionSimulationTest::CaseCount > 0U &&
        EDMGapAcquisitionSimulationTest::CaseCount < 32U, "EDM14 coverage mask must fit.");
    constexpr std::uint32_t EDMVoltageAllCases =
        (1U << EDMVoltageSimulationTest::CaseCount) - 1U;
    static_assert(EDMVoltageSimulationTest::CaseCount > 0U &&
        EDMVoltageSimulationTest::CaseCount < 32U, "EDM15 coverage mask must fit.");
    constexpr std::uint32_t EDMRecipeAllCases = (1U << EDMRecipeSimulationTest::CaseCount) - 1U;
    static_assert(EDMRecipeSimulationTest::CaseCount > 0U && EDMRecipeSimulationTest::CaseCount < 32U,
        "EDM16 coverage mask must fit.");
    constexpr std::uint32_t EDMShortFlushAllCases = (1U << EDMShortFlushSimulationTest::CaseCount) - 1U;
    static_assert(EDMShortFlushSimulationTest::CaseCount > 0U && EDMShortFlushSimulationTest::CaseCount <= 24U,
        "EDM22 coverage and bounded runtime budget changed.");
    constexpr std::uint32_t EDMAutomaticFlushAllCases = (1U << EDMAutomaticFlushSimulationTest::CaseCount) - 1U;
    static_assert(EDMAutomaticFlushSimulationTest::CaseCount > 0U && EDMAutomaticFlushSimulationTest::CaseCount <= 24U,
        "EDM23 coverage and bounded runtime budget changed.");
    constexpr std::uint32_t EDMCoordinatorAllCases = (1U << EDMProcessCoordinatorSimulationTest::CaseCount) - 1U;
    static_assert(EDMProcessCoordinatorSimulationTest::CaseCount > 0U && EDMProcessCoordinatorSimulationTest::CaseCount <= 28U,
        "EDM25 coverage and bounded runtime budget changed.");
    const char* GapDryRunTag(bool processTest, bool feedRetreatTest, bool adcShadowTest = false,
        bool voltageTest = false, bool recipeTest = false, bool shortFlushTest = false, bool automaticFlushTest = false, bool liveAutomaticFlushTest = false, bool processCoordinatorTest = false, bool liveProcessTest = false, bool motionBridgeTest = false) noexcept
    {
        if (motionBridgeTest) return "EDM27";
        if (liveProcessTest) return "EDM26";
        if (processCoordinatorTest) return "EDM25";
        if (liveAutomaticFlushTest) return "EDM24";
        if (automaticFlushTest) return "EDM23";
        if (shortFlushTest) return "EDM22";
        return recipeTest ? "EDM16" : (voltageTest ? "EDM15" : (adcShadowTest ? "EDM14" : (feedRetreatTest ? "EDM02" : (processTest ? "EDM01" : "GAP-CG"))));
    }
    const char* const GapCaseNames[GapCaseCount] = {
        "NO_SAMPLE", "NORMAL", "ZERO_LOW", "LOW_HYSTERESIS",
        "LOW_RECOVER", "HIGH", "HIGH_HYSTERESIS", "HIGH_RECOVER",
        "INVALID", "VALID_RECOVER", "FROZEN", "FRESH_RECOVER",
        "REPLAY", "SEQUENCE_RECOVER", "OUT_OF_RANGE", "FINAL_RECOVER"
    };
    EDMGap::Quality GapExpectedQuality(std::uint32_t phase) noexcept
    {
        if (phase == 0U) return EDMGap::Quality::NO_SAMPLE;
        if (phase == 8U) return EDMGap::Quality::INVALID;
        if (phase == 10U) return EDMGap::Quality::STALE;
        if (phase == 12U) return EDMGap::Quality::SEQUENCE_ERROR;
        if (phase == 14U) return EDMGap::Quality::OUT_OF_RANGE;
        return EDMGap::Quality::VALID;
    }
    EDMGap::Band GapExpectedBand(std::uint32_t phase) noexcept
    {
        if (GapExpectedQuality(phase) != EDMGap::Quality::VALID)
            return EDMGap::Band::UNKNOWN;
        if (phase == 2U || phase == 3U) return EDMGap::Band::LOW;
        if (phase == 5U || phase == 6U) return EDMGap::Band::HIGH;
        return EDMGap::Band::NORMAL;
    }
}

namespace
{
    // Native RtPrintf records are limited to 511 bytes. Each stage99/100/102/104
    // evidence part is formatted locally and bounded to 480 including newline.
    // session/tick plus result correlate parts from one copied Feedback.
    void LogEDM99SegmentRecords(const char* stage, const char* event,
        const EDM96::Feedback& gap, bool tuningCurrent) noexcept
    {
        char record[481]{};
        const auto session = static_cast<unsigned long long>(gap.session);
        const auto tick = static_cast<unsigned long long>(gap.tick);
        const char* result = event ? event : "UNKNOWN";
        const auto emit = [&](int length, const char* part) noexcept
        {
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
                RtPrintf("%s", record);
            else RtPrintf("[%.6s] event=LOG_FORMAT_ERROR part=%.16s session=%llu tick=%llu output=NONE\n",
                stage, part, session, tick);
        };
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=%.32s part=SEGMENT session=%llu tick=%llu epoch=%llu ownerGen=%u step=%u phase=%u direction=%s voltage=%.9g accepted=%llu completed=%llu active=%u segmentActive=%u segmentCompleted=%u faulted=%u reason=%.32s proofScope=COMMAND_SCALAR_ONLY\n",
            stage, result, session, tick, static_cast<unsigned long long>(gap.epoch),
            static_cast<unsigned>(gap.ownerGeneration), gap.expectedStep, gap.phase,
            gap.direction > 0 ? "FORWARD" : gap.direction < 0 ? "RETREAT" : "QUALIFYING", gap.voltageV,
            static_cast<unsigned long long>(gap.acceptedSegments), static_cast<unsigned long long>(gap.completedSegments),
            gap.active ? 1U : 0U, gap.segmentActive ? 1U : 0U, gap.segmentCompleted ? 1U : 0U,
            gap.faulted ? 1U : 0U, EDM96::ReasonName(gap.reason)), "SEGMENT");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=TRAJECTORY_EVIDENCE result=%.32s part=SCALAR session=%llu tick=%llu qPulse=%.12g vPulsePerSec=%.12g lengthPulse=%.12g reverseSegments=%llu reverseTicks=%llu returnForwardTicks=%llu encoderZeroProven=0 proofScope=COMMAND_SCALAR_ONLY\n",
            stage, result, session, tick, gap.q, gap.v, gap.length,
            static_cast<unsigned long long>(gap.reverseSegments), static_cast<unsigned long long>(gap.reverseTicks),
            static_cast<unsigned long long>(gap.returnForwardTicks)), "SCALAR");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=SOURCE_EVIDENCE result=%.32s part=SOURCE session=%llu tick=%llu sourceSequence=%llu sourceUs=%llu freshFrames=%llu heldFrames=%llu maxAgeUs=%llu output=GUARDED_CSV dischargeCommand=NONE\n",
            stage, result, session, tick, static_cast<unsigned long long>(gap.sourceSequence),
            static_cast<unsigned long long>(gap.sourceCapturedUs), static_cast<unsigned long long>(gap.freshFrames),
            static_cast<unsigned long long>(gap.heldFrames), static_cast<unsigned long long>(gap.maximumAgeUs)), "SOURCE");
        if (gap.sourceReason != EDM88::Reason::None)
            emit(std::snprintf(record, sizeof(record),
                "[%.6s] event=SOURCE_REJECT result=%.32s part=SOURCE session=%llu tick=%llu observerUs=%llu acceptedSourceUs=%llu rejectedMaxAgeUs=%llu sourceReason=%.24s sourceLimitUs=1500 acquisition=RT_SYNTHETIC output=NONE dischargeCommand=NONE\n",
                stage, result, session, tick, static_cast<unsigned long long>(gap.nowUs),
                static_cast<unsigned long long>(gap.sourceCapturedUs), static_cast<unsigned long long>(gap.maximumAgeUs),
                EDM88::ReasonName(gap.sourceReason)), "SOURCE_REJECT");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=GAIN_EVIDENCE result=%.32s part=SPEED session=%llu tick=%llu phase=%u voltage=%.9g referenceV=35 errorV=%.9g rawSpeedMmMin=%.12g limitedSpeedMmMin=%.12g targetRatio=%.12g proofScope=COMMAND_SCALAR_ONLY\n",
            stage, result, session, tick, gap.phase, gap.voltageV, gap.errorV,
            gap.rawSpeedMmMin, gap.limitedSpeedMmMin, gap.targetRatio), "SPEED");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=GAIN_EVIDENCE result=%.32s part=CONFIG session=%llu tick=%llu Q=%.9g R=%.9g S=%.9g configHash=%llu saturationFreshFrames=%llu freshFrames=%llu tunedMode=%u burstMode=%u continuousMode=%u enduranceMode=%u pairedMode=%u tuningCurrent=%u gainUnit=DIRECT_CURVE\n",
            stage, result, session, tick, gap.tuning.positiveGain, gap.tuning.negativeGain, gap.tuning.deadbandV,
            static_cast<unsigned long long>(gap.configHash), static_cast<unsigned long long>(gap.saturationFrames),
            static_cast<unsigned long long>(gap.freshFrames), gap.tunedMode ? 1U : 0U, gap.burstMode ? 1U : 0U,
            gap.continuousMode ? 1U : 0U, gap.enduranceMode ? 1U : 0U, gap.pairedMode ? 1U : 0U, tuningCurrent ? 1U : 0U), "CONFIG");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=BURST_EVIDENCE result=%.32s part=COUNTS session=%llu tick=%llu pulseLowFreshFrames=%llu suppressedPulseWindows=%llu qualifiedRetreatWindows=%llu reverseEpisodes=%llu zeroCrossings=%llu proofScope=COMMAND_SCALAR_ONLY\n",
            stage, result, session, tick, static_cast<unsigned long long>(gap.pulseLowFreshFrames),
            static_cast<unsigned long long>(gap.suppressedPulseWindows), static_cast<unsigned long long>(gap.qualifiedRetreatWindows),
            static_cast<unsigned long long>(gap.reverseEpisodes), static_cast<unsigned long long>(gap.zeroCrossings)), "COUNTS");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=BURST_EVIDENCE result=%.32s part=POLICY session=%llu tick=%llu shortPulseUs=5000 captureDwellUs=20000 suppressedRequiredPerSegment=4 qualifiedRequiredPerSegment=2 reverseEpisodesRequiredPerSegment=2 zeroCrossingsRequiredPerSegment=4 maxSteps=%u pairedPairs=%u midSegmentReverse=1 proofScope=COMMAND_SCALAR_ONLY dischargeCommand=NONE\n",
            stage, result, session, tick, gap.enduranceMode ? EDM102::MaximumSteps : gap.continuousMode ? EDM100::MaximumSteps : 4U,
            gap.pairedMode ? EDM104::MaximumSteps / 2U : 0U), "POLICY");
    }

    void LogEDM99GapSource(const char* stage, std::uint64_t value,
        const EDM97::Tuning& tuning, unsigned moves, bool continuous, bool endurance = false, bool paired = false) noexcept
    {
        char record[481]{};
        const auto session = static_cast<unsigned long long>(value);
        const auto emit = [&](int length, const char* part) noexcept
        {
            if (length > 0 && length <= 480 && static_cast<std::size_t>(length) < sizeof(record))
                RtPrintf("%s", record);
            else RtPrintf("[%.6s] event=LOG_FORMAT_ERROR part=%.16s session=%llu tick=0 output=NONE\n",
                stage, part, session);
        };
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=GAP_SOURCE part=PROFILE session=%llu tick=0 source=RT_SYNTHETIC producerUs=250 captureDwellUs=20000 segmentDeadlineUs=10000000 steps=%u maxSteps=%u continuousMode=%u pairedMode=%u pairedPairs=%u mode=CAPTURE_DWELL_BURST_SERVO midSegmentReverse=1 proofScope=COMMAND_SCALAR_ONLY encoderZeroProven=0 dischargeCommand=NONE\n",
            stage, session, moves, endurance ? EDM102::MaximumSteps : continuous ? EDM100::MaximumSteps : 4U,
            continuous ? 1U : 0U, paired ? 1U : 0U, paired ? EDM104::MaximumSteps / 2U : 0U), "PROFILE");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=GAP_SOURCE part=PLAN session=%llu tick=0 voltagePlan=50_200ms_4x20_5ms_50_45ms_50_200ms_20_700ms_50_300ms_20_700ms_50_STEADY shortPulseUs=5000 shortPulseWindowsPerSegment=4 sustainedRetreatWindowsPerSegment=2 commandReverseEpisodesPerSegment=2 commandZeroCrossingsPerSegment=4 proofScope=COMMAND_SCALAR_ONLY\n",
            stage, session), "PLAN");
        emit(std::snprintf(record, sizeof(record),
            "[%.6s] event=GAP_SOURCE part=CONFIG session=%llu tick=0 referenceV=35 scale=0.001 Q=%.9g R=%.9g S=%.9g configHash=%llu immutablePerArm=1 gainUnit=DIRECT_CURVE hold=CANCEL_RESET_REQUIRED dischargeCommand=NONE\n",
            stage, session, tuning.positiveGain, tuning.negativeGain, tuning.deadbandV,
            static_cast<unsigned long long>(EDM97::ConfigHash(tuning))), "CONFIG");
    }

    bool E94Number(const std::string& text, double& value) noexcept
    {
        if (text.empty() || text.size() > 32U) return false;
        unsigned digits = 0U, dots = 0U;
        for (std::size_t i = 0U; i < text.size(); ++i)
        {
            const char c = text[i];
            if (i == 0U && (c == '+' || c == '-')) continue;
            if (c == '.' && ++dots == 1U) continue;
            if (c < '0' || c > '9') return false;
            ++digits;
        }
        if (!digits) return false;
        char* end = nullptr;
        value = std::strtod(text.c_str(), &end);
        return end == text.c_str() + text.size() && std::isfinite(value);
    }
    bool E94Profile(const NCParsedBlock& b, unsigned p) noexcept
    {
        return EDM77::Detail::Clean(b) && !b.isEmpty && b.gCount == 1 && b.mCount == 0 &&
            EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) &&
            EDM77::Detail::OnlyAddresses(b, "NP") && b.has('P') &&
            EDM77::Detail::EqualLiteral(b.expression('P'), p);
    }
    bool E126Profile(const NCParsedBlock& b, unsigned arm, EDM97::Tuning& tuning) noexcept
    {
        if ((arm != 158U && arm != 160U) || !EDM77::Detail::Clean(b) || b.isEmpty ||
            b.gCount != 1 || b.mCount || !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), arm) || !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) ||
            parsed.positiveGain != 8.0 || parsed.negativeGain != 1.0 || parsed.deadbandV != 1.0 ||
            !EDM97::ValidTuning(parsed)) return false;
        tuning = parsed; return true;
    }
    bool E97ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 91U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM97::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E98ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 93U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM98::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E99ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 95U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM97::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E100ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 97U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM97::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E102ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 99U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM97::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E104ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 101U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM97::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E105ProfileOnlyNPQRS(const NCParsedBlock& b, EDM97::Tuning& tuning) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), 103U) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM105::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E106ProfileOnlyNPQRS(const NCParsedBlock& b, unsigned selector, EDM97::Tuning& tuning) noexcept
    {
        if ((selector != 105U && selector != 107U) || !EDM77::Detail::Clean(b) || b.isEmpty ||
            b.gCount != 1 || b.mCount != 0 || !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, "NPQRS") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), selector) ||
            !b.has('Q') || !b.has('R') || !b.has('S')) return false;
        EDM97::Tuning parsed{};
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM105::ValidTuning(parsed)) return false;
        tuning = parsed;
        return true;
    }
    bool E106FlushProfile(const NCParsedBlock& b, unsigned interruptArm, EDM97::Tuning& tuning) noexcept
    {
        return interruptArm ? E106ProfileOnlyNPQRS(b, interruptArm, tuning) : E105ProfileOnlyNPQRS(b, tuning);
    }
    // Explicit commissioning metadata only: RT must reject this request before
    // Engine::Arm. It is never a hold selection or a motion permission.
    bool E216RejectBeforeArm(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),216U);
    }
    bool E217CenterNominalHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),1U);
    }
    bool E218CenterFlushHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),2U);
    }
    bool E219CenterReturnHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),3U);
    }
    bool E220CenterAnchorHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),4U);
    }
    bool E221CenterStartHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),5U);
    }
    bool E222CenterTopDwellHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),6U);
    }
    bool E224CenterDwellHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),7U);
    }
    bool E225CenterReturnDwellHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),8U);
    }
    bool E226CenterFinalDwellHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),9U);
    }
    bool E227CenterInitialDwellHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),10U);
    }
    bool E228CenterBrakeHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),11U);
    }
    // EDM230 is a bounded two-event commissioning selector, not a HOLD checkpoint.
    bool E230CenterRepeat(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),230U);
    }
    // EDM231 keeps the two-event cap and selects only event 2 Outbound.
    bool E231CenterSecondFlushHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),231U);
    }
    // EDM232 keeps event 1 uninterrupted and pauses only event 2 Returning.
    bool E232CenterSecondReturnHold(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),232U);
    }
    // Two fresh parent sessions; this selector never requests an automatic HOLD.
    bool E233CenterParentSequence(const NCParsedBlock& b, unsigned selector) noexcept
    {
        return (selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&b.has('L')&&!b.has('H')&&
            EDM77::Detail::EqualLiteral(b.expression('L'),233U);
    }
    bool E234ArmSequence(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return (selector==274U||selector==EDM173::B2ArmP)&&EDM77::Detail::Clean(b)&&
            b.has('P')&&EDM77::Detail::EqualLiteral(b.expression('P'),selector)&&
            b.has('L')&&!b.has('H')&&(EDM77::Detail::EqualLiteral(b.expression('L'),234U)||
                EDM77::Detail::EqualLiteral(b.expression('L'),235U)||EDM77::Detail::EqualLiteral(b.expression('L'),236U));
    }
    bool E237ArmSequence(const NCParsedBlock& b,unsigned selector) noexcept
    {
        const bool history=(selector==274U||selector==EDM173::B2ArmP)&&
            (EDM77::Detail::EqualLiteral(b.expression('L'),237U)||EDM77::Detail::EqualLiteral(b.expression('L'),238U));
        const bool center=(selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),239U);
        const bool mixed=(selector==274U||selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),240U);
        const bool allKinds=(selector==274U||selector==EDM173::B2ArmP||selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            (EDM77::Detail::EqualLiteral(b.expression('L'),241U)||EDM77::Detail::EqualLiteral(b.expression('L'),242U)||EDM77::Detail::EqualLiteral(b.expression('L'),243U)||EDM77::Detail::EqualLiteral(b.expression('L'),244U));
        const bool repeatedOrdinary=(selector==274U||selector==EDM173::B2ArmP||selector==EDM173::B3ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),245U);
        const bool repeatedOrdinaryB4=(selector==274U||selector==EDM173::B2ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),246U);
        const bool repeatedB2=(selector==274U||selector==EDM173::B2ArmP||selector==EDM173::B3ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),247U);
        const bool repeatedB2B4=(selector==274U||selector==EDM173::B2ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),248U);
        return (history||center||mixed||allKinds||repeatedOrdinary||repeatedOrdinaryB4||repeatedB2||repeatedB2B4)&&EDM77::Detail::Clean(b)&&b.has('P')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),selector)&&b.has('L')&&!b.has('H');
    }
    // The ordinary arc kinds share the stopped-parent protocol, while their
    // program selector and geometry remain independently frozen in NC.
    unsigned E234ParentKind(unsigned arm,unsigned program,unsigned ordinaryCode) noexcept
    {
        if(program!=234U&&program!=235U)return 0U;
        if(arm==EDM173::B2ArmP)return 2U;
        if(arm!=274U)return 0U;
        return program==234U?(ordinaryCode==1U?1U:0U):
            ordinaryCode==2U?3U:ordinaryCode==3U?4U:0U;
    }
    bool E220CenterAnchorSpan(const EDM128::Feedback& e,double& span) noexcept
    {
        span=0.0;EDM204::Geometry geometry{};
        if(!EDM128::CenterAxisGeometry(e,geometry))return false;
        // B4's last leg is diagonal: its true native length is not radius
        // and is independent of E6's much longer axial withdrawal.
        for(unsigned i=0U;i<6U;++i) {
            if(i>=3U) {
                if(!EDM109::SameDouble(geometry.anchorPulse[i],geometry.centerPulse[i]))return false;
            } else span=std::hypot(span,(geometry.anchorPulse[i]-geometry.centerPulse[i])/geometry.pulsePerNative[i]);
        }
        return EDM126::Positive(span);
    }
    bool E221CenterStartProgress(const EDM128::Feedback& e,const EDM126::Point& position,
        double& span,double& traveled) noexcept
    {
        span=traveled=0.0;EDM204::Geometry geometry{};double tolerance=0.0;
        if(e.phase!=EDM128::Phase::ToCenter||!e.detached||
            !EDM128::CenterAxisGeometry(e,geometry)||
            !EDM204::CoordinateTolerance(geometry.anchorPulse,geometry.centerPulse,
                geometry.pulsePerNative,tolerance))return false;
        unsigned projection=0U;double largest=0.0;
        for(unsigned i=0U;i<6U;++i) {
            if(!std::isfinite(position[i]))return false;
            if(i>=3U) {
                if(!EDM109::SameDouble(position[i],geometry.anchorPulse[i])||
                    !EDM109::SameDouble(geometry.centerPulse[i],geometry.anchorPulse[i]))return false;
            } else {
                const double native=(geometry.centerPulse[i]-geometry.anchorPulse[i])/geometry.pulsePerNative[i];
                span=std::hypot(span,native);
                if(std::fabs(native)>largest){largest=std::fabs(native);projection=i;}
            }
        }
        // A sub-roundoff chord cannot provide a resolved 25% checkpoint.
        if(!EDM126::Positive(span)||span<=4.0*tolerance||!EDM126::Positive(largest))return false;
        const double from=geometry.anchorPulse[projection],to=geometry.centerPulse[projection];
        const double magnitude=(std::max)((std::max)(std::fabs(from),std::fabs(to)),std::fabs(position[projection]));
        const double qTolerance=2.0*EDM204::Roundoff(magnitude)/std::fabs(to-from);
        double q=(position[projection]-from)/(to-from);
        if(!std::isfinite(q)||!std::isfinite(qTolerance)||q < -qTolerance||q>1.0+qTolerance)return false;
        q=(std::max)(0.0,(std::min)(1.0,q));
        for(unsigned i=0U;i<3U;++i) {
            const double a=geometry.anchorPulse[i],b=geometry.centerPulse[i];
            const double want=q==0.0?a:q==1.0?b:a+(b-a)*q;
            const double scale=(std::max)((std::max)(std::fabs(a),std::fabs(b)),std::fabs(position[i]));
            const double budget=2.0*EDM204::Roundoff(scale);
            if(!std::isfinite(want)||!std::isfinite(budget)||budget/geometry.pulsePerNative[i]>EDM204::MaximumRoundoffMm||
                std::fabs(position[i]-want)>budget)return false;
        }
        // ToCenter has no StageSpeed sample: only the current mapped pulse
        // point proves progress. Retained main-path localMm is the E5 anchor.
        traveled=span*q;return std::isfinite(traveled)&&traveled>=0.0&&traveled<=span;
    }
    bool E218FlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return EDM130::IsG76CenterFlushHold218(f.scope.recipe)&&
            EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
            f.flushPhase==EDM128::Phase::Outbound&&f.flushEventIndex==1ULL;
    }
    bool E219CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E218FlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterReturnHold219(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::Returning&&f.flushEventIndex==1ULL);
    }
    bool E220CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E219CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterAnchorHold220(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::ToAnchor&&f.flushEventIndex==1ULL);
    }
    bool E221CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E220CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterStartHold221(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::ToCenter&&f.flushEventIndex==1ULL);
    }
    bool E222CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E221CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterTopDwellHold222(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::TopDwell&&f.flushEventIndex==1ULL);
    }
    bool E224CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E222CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterDwellHold224(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::CenterDwell&&f.flushEventIndex==1ULL);
    }
    const char* E224CenterDwellIntentTag(const EDM130::Recipe& recipe) noexcept
    {return EDM130::IsG76CenterDwellHold224(recipe)?"EDM224":"EDM223";}
    bool E225CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E224CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterReturnDwellHold225(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::CenterReturnDwell&&f.flushEventIndex==1ULL);
    }
    const char* E225CenterDwellIntentTag(const EDM130::Recipe& recipe) noexcept
    {return EDM130::IsG76CenterReturnDwellHold225(recipe)?"EDM225":E224CenterDwellIntentTag(recipe);}
    bool E226CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E225CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterFinalDwellHold226(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::ReturnDwell&&f.flushEventIndex==1ULL);
    }
    const char* E226CenterDwellIntentTag(const EDM130::Recipe& recipe) noexcept
    {return EDM130::IsG76CenterFinalDwellHold226(recipe)?"EDM226":E225CenterDwellIntentTag(recipe);}
    bool E227CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E226CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterInitialDwellHold227(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::AnchorDwell&&f.flushEventIndex==1ULL);
    }
    const char* E227CenterDwellIntentTag(const EDM130::Recipe& recipe) noexcept
    {return EDM130::IsG76CenterInitialDwellHold227(recipe)?"EDM227":E226CenterDwellIntentTag(recipe);}
    bool E228CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E227CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterBrakeHold228(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::Brake&&f.flushEventIndex==1ULL);
    }
    bool E231CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E228CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterSecondFlushHold231(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::Outbound&&f.flushEventIndex==2ULL);
    }
    bool E232CenterFlushCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return E231CenterFlushCheckpointReceipt(f)||
            (EDM130::IsG76CenterSecondReturnHold232(f.scope.recipe)&&
                EDM157::ValidG76CenterFlushReceipt(f)&&f.flushPause&&
                f.flushPhase==EDM128::Phase::Returning&&f.flushEventIndex==2ULL);
    }
    const char* E228CenterDwellIntentTag(const EDM130::Recipe& recipe) noexcept
    {return EDM130::IsG76CenterBrakeHold228(recipe)?"EDM228":E227CenterDwellIntentTag(recipe);}
    bool E217NominalCheckpointReceipt(const EDM157::Feedback& f) noexcept
    {
        return EDM130::IsG76CenterHold217(f.scope.recipe)&&EDM157::ValidNominalWorkReceipt(f)&&
            !f.flushPause&&f.flushPhase==EDM128::Phase::Gap&&!f.flushEventIndex&&
            !f.nominalWork.triggeredEvents&&!f.nominalWork.completedEvents&&
            f.nominalWork.elapsedUs>=f.scope.recipe.workTimeUs/4ULL&&
            f.nominalWork.elapsedUs<f.scope.recipe.workTimeUs;
    }
    bool E217SameNominalPose(const EDM157::Feedback& a,const EDM157::Feedback& b) noexcept
    {
        return EDM126::SameSource(a.cursorSource,b.cursorSource)&&a.historyDepth==b.historyDepth&&
            EDM109::SameDouble(a.localMm,b.localMm)&&EDM109::SameDouble(a.stationMm,b.stationMm)&&
            EDM109::SamePulse(a.positionPulse,b.positionPulse);
    }
    bool E223StartGuardCurrent(const EDM94::Feedback& guard,const EDM157::Feedback& paused,
        bool allowPublishingLag=false) noexcept
    {
        const bool current=guard.lastTick>=paused.tick&&guard.lastTick-paused.tick<=8ULL&&
            guard.lastNowUs>=paused.nowUs&&guard.lastNowUs-paused.nowUs<=2000ULL;
        // Pause proof is published just before the guard. A nearby older
        // guard may retain the operator intent, but can never submit it.
        const bool publishingLag=allowPublishingLag&&guard.lastTick<paused.tick&&
            paused.tick-guard.lastTick<=8ULL&&guard.lastNowUs<paused.nowUs&&
            paused.nowUs-guard.lastNowUs<=2000ULL;
        return guard.active&&guard.armAccepted&&guard.armedReady&&!guard.forceZero&&!guard.handoffZero&&
            !guard.finishRequested&&!guard.cancelled&&!guard.faulted&&
            guard.identity.session==paused.scope.authorizedSource.scope.session&&
            guard.identity.epoch==paused.scope.compoundArmEpoch&&
            guard.identity.ownerGeneration==paused.scope.ownerGeneration&&
            paused.tick&&paused.nowUs&&guard.lastTick&&guard.lastNowUs&&(current||publishingLag);
    }
    bool E250SquareB1Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),250U);
    }
    bool E251SquareB2Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B2ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::B2ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),251U);
    }
    bool E252SquareB3Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B3ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::B3ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),252U);
    }
    bool E253SquareB4Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B4ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::B4ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),253U);
    }
    bool E255DiamondB1Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),255U);
    }
    bool E256DiamondB2Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B2ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::B2ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),256U);
    }
    bool E257DiamondB3Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B3ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::B3ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),257U);
    }
    bool E258DiamondB4Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B4ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),EDM173::B4ArmP)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),258U);
    }
    // EDM260 keeps the 1 mm family literal and separate from all legacy tags.
    unsigned E260OneMmMode(const NCParsedBlock& b) noexcept
    {
        if(!b.has('L'))return 0U;
        for(unsigned marker=260U;marker<=263U;++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))
                return EDM130::G76OneMmMode260(marker);
        return 0U;
    }
    bool E260OneMmArm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return (selector==EDM173::ArmP||selector==EDM173::B3ArmP||selector==EDM173::B4ArmP)&&
            EDM77::Detail::Clean(b)&&!b.isEmpty&&b.gCount==1&&b.mCount==0&&
            EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),selector)&&E260OneMmMode(b)!=0U;
    }
    bool E261PolygonB2Arm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        return selector==EDM173::B2ArmP&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")&&b.has('P')&&b.has('L')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),selector)&&
            EDM77::Detail::EqualLiteral(b.expression('L'),264U);
    }
    bool E261PolygonOrbitShape(const NCBlock& b) noexcept
    {
        return b.has('Q')&&std::isfinite(b.val('Q'))&&b.val('Q')>=3.0&&b.val('Q')<=12.0&&
            std::floor(b.val('Q'))==b.val('Q');
    }
    // EDM263 binds each 26-child expansion to an exact authored family tag.
    unsigned E263ExpansionMode(const NCParsedBlock& b) noexcept
    {
        if(!b.has('L'))return 0U;
        for(unsigned marker=265U;marker<=272U;++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))
                return EDM130::G76ExpansionMode263(marker);
        return 0U;
    }
    bool E263ExpansionArm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        const bool b2=selector==EDM173::B2ArmP;
        if((!b2&&selector!=EDM173::ArmP&&selector!=EDM173::B3ArmP&&selector!=EDM173::B4ArmP)||
            !EDM77::Detail::Clean(b)||b.isEmpty||b.gCount!=1||b.mCount!=0||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)||
            !EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")||!b.has('P')||
            !EDM77::Detail::EqualLiteral(b.expression('P'),selector)||!E263ExpansionMode(b))return false;
        for(unsigned marker=b2?269U:265U;marker<=(b2?272U:268U);++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))return true;
        return false;
    }
    bool E263ExpansionOrbitShape(const NCBlock& b) noexcept
    {
        return b.has('P')&&(b.val('P')==21.0||b.val('P')==22.0||b.val('P')==23.0||b.val('P')==26.0)&&
            b.has('B')&&(b.val('B')==1.0||b.val('B')==2.0||b.val('B')==3.0||b.val('B')==4.0)&&
            (b.val('P')==26.0?E261PolygonOrbitShape(b):!b.has('Q'));
    }
    // EDM266 seals fixed-radius advancing plans to their own literal tags.
    unsigned E266FixedRadiusMode(const NCParsedBlock& b) noexcept
    {
        if(!b.has('L'))return 0U;
        for(unsigned marker=273U;marker<=280U;++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))
                return EDM130::G76FixedRadiusMode266(marker);
        return 0U;
    }
    bool E266FixedRadiusArm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        const bool b2=selector==EDM173::B2ArmP;
        if((!b2&&selector!=EDM173::ArmP&&selector!=EDM173::B3ArmP&&selector!=EDM173::B4ArmP)||
            !EDM77::Detail::Clean(b)||b.isEmpty||b.gCount!=1||b.mCount!=0||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)||
            !EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")||!b.has('P')||
            !EDM77::Detail::EqualLiteral(b.expression('P'),selector)||!E266FixedRadiusMode(b))return false;
        for(unsigned marker=b2?277U:273U;marker<=(b2?280U:276U);++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))return true;
        return false;
    }
    bool E266FixedRadiusOrbitShape(const NCBlock& b) noexcept
    {
        return b.has('P')&&(b.val('P')==1.0||b.val('P')==2.0||b.val('P')==3.0||b.val('P')==6.0)&&
            b.has('B')&&(b.val('B')==1.0||b.val('B')==2.0||b.val('B')==3.0||b.val('B')==4.0)&&
            (b.val('P')==6.0?E261PolygonOrbitShape(b):!b.has('Q'));
    }
    // WORK commissioning is a separate immutable selector family. The older
    // fixed-radius recipes retain their neutral WORK frame contract.
    unsigned E267FixedRadiusWorkMode(const NCParsedBlock& b) noexcept
    {
        if(!b.has('L'))return 0U;
        for(unsigned marker=281U;marker<=288U;++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))
                return EDM130::G76FixedRadiusWorkMode267(marker);
        return 0U;
    }
    bool E267FixedRadiusWorkArm(const NCParsedBlock& b,unsigned selector) noexcept
    {
        const bool b2=selector==EDM173::B2ArmP;
        if((!b2&&selector!=EDM173::ArmP&&selector!=EDM173::B3ArmP&&selector!=EDM173::B4ArmP)||
            !EDM77::Detail::Clean(b)||b.isEmpty||b.gCount!=1||b.mCount!=0||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0],180U)||
            !EDM77::Detail::OnlyAddresses(b,"NPQRSIJKL")||!b.has('P')||
            !EDM77::Detail::EqualLiteral(b.expression('P'),selector)||!E267FixedRadiusWorkMode(b))return false;
        for(unsigned marker=b2?285U:281U;marker<=(b2?288U:284U);++marker)
            if(EDM77::Detail::EqualLiteral(b.expression('L'),marker))return true;
        return false;
    }
    bool E267FixedRadiusWorkFrame(const NCTranslationSnapshot& frame) noexcept
    {
        return frame.workMode==168&&frame.workWCode>0&&NCG76::FrameSupported(frame,true,true);
    }
    bool E251SquareSourcePair(const NCParsedBlock& arm,const NCBlock& orbit) noexcept
    {
        const unsigned fixedRadiusWorkMode=E267FixedRadiusWorkMode(arm);
        if(fixedRadiusWorkMode)
        {
            unsigned selector=0U;
            for(unsigned candidate:{EDM173::ArmP,EDM173::B2ArmP,EDM173::B3ArmP,EDM173::B4ArmP})
                if(E267FixedRadiusWorkArm(arm,candidate))selector=candidate;
            return selector&&!orbit.isEmpty&&!orbit.isGoto&&!orbit.isBlockSkip&&orbit.hasG&&
                orbit.gCode==76&&orbit.gCount==1&&orbit.gCodes[0]==76&&orbit.mCount==0&&
                E266FixedRadiusOrbitShape(orbit)&&orbit.val('P')==static_cast<double>(fixedRadiusWorkMode)&&
                orbit.val('B')==static_cast<double>(EDM173::ModeForArm(selector));
        }
        const unsigned fixedRadiusMode=E266FixedRadiusMode(arm);
        if(fixedRadiusMode)
        {
            unsigned selector=0U;
            for(unsigned candidate:{EDM173::ArmP,EDM173::B2ArmP,EDM173::B3ArmP,EDM173::B4ArmP})
                if(E266FixedRadiusArm(arm,candidate))selector=candidate;
            return selector&&!orbit.isEmpty&&!orbit.isGoto&&!orbit.isBlockSkip&&orbit.hasG&&
                orbit.gCode==76&&orbit.gCount==1&&orbit.gCodes[0]==76&&orbit.mCount==0&&
                E266FixedRadiusOrbitShape(orbit)&&orbit.val('P')==static_cast<double>(fixedRadiusMode)&&
                orbit.val('B')==static_cast<double>(EDM173::ModeForArm(selector));
        }
        const unsigned expansionMode=E263ExpansionMode(arm);
        if(expansionMode)
        {
            unsigned selector=0U;
            for(unsigned candidate:{EDM173::ArmP,EDM173::B2ArmP,EDM173::B3ArmP,EDM173::B4ArmP})
                if(E263ExpansionArm(arm,candidate))selector=candidate;
            return selector&&!orbit.isEmpty&&!orbit.isGoto&&!orbit.isBlockSkip&&orbit.hasG&&
                orbit.gCode==76&&orbit.gCount==1&&orbit.gCodes[0]==76&&orbit.mCount==0&&
                E263ExpansionOrbitShape(orbit)&&orbit.val('P')==static_cast<double>(expansionMode)&&
                orbit.val('B')==static_cast<double>(EDM173::ModeForArm(selector));
        }
        if(arm.has('L')&&EDM77::Detail::EqualLiteral(arm.expression('L'),264U))
            return E261PolygonB2Arm(arm,EDM173::B2ArmP)&&!orbit.isEmpty&&!orbit.isGoto&&!orbit.isBlockSkip&&
                orbit.hasG&&orbit.gCode==76&&orbit.gCount==1&&orbit.gCodes[0]==76&&orbit.mCount==0&&
                orbit.has('P')&&orbit.val('P')==16.0&&orbit.has('B')&&orbit.val('B')==2.0&&E261PolygonOrbitShape(orbit);
        const unsigned oneMmMode=E260OneMmMode(arm);
        if(oneMmMode)
        {
            unsigned selector=0U;
            for(unsigned candidate:{EDM173::ArmP,EDM173::B3ArmP,EDM173::B4ArmP})
                if(E260OneMmArm(arm,candidate))selector=candidate;
            return selector&&!orbit.isEmpty&&!orbit.isGoto&&!orbit.isBlockSkip&&orbit.hasG&&
                orbit.gCode==76&&orbit.gCount==1&&orbit.gCodes[0]==76&&orbit.mCount==0&&
                orbit.has('P')&&orbit.val('P')==static_cast<double>(oneMmMode)&&orbit.has('B')&&
                orbit.val('B')==static_cast<double>(EDM173::ModeForArm(selector))&&
                (oneMmMode!=16U||E261PolygonOrbitShape(orbit));
        }
        const bool squareB1=E250SquareB1Arm(arm,EDM173::ArmP);
        const bool squareB2=E251SquareB2Arm(arm,EDM173::B2ArmP);
        const bool squareB3=E252SquareB3Arm(arm,EDM173::B3ArmP);
        const bool squareB4=E253SquareB4Arm(arm,EDM173::B4ArmP);
        const bool diamondB2=E256DiamondB2Arm(arm,EDM173::B2ArmP);
        const bool diamondB3=E257DiamondB3Arm(arm,EDM173::B3ArmP);
        const bool diamondB4=E258DiamondB4Arm(arm,EDM173::B4ArmP);
        const bool diamond=E255DiamondB1Arm(arm,EDM173::ArmP)||diamondB2||diamondB3||diamondB4;
        const bool square=squareB1||squareB2||squareB3||squareB4;
        if(arm.has('L')&&(EDM77::Detail::EqualLiteral(arm.expression('L'),250U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),251U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),252U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),253U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),255U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),256U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),257U)||
            EDM77::Detail::EqualLiteral(arm.expression('L'),258U))&&!square&&!diamond)return false;
        return (square||diamond)?(!orbit.isEmpty&&!orbit.isGoto&&!orbit.isBlockSkip&&orbit.hasG&&
            orbit.gCode==76&&orbit.gCount==1&&orbit.gCodes[0]==76&&orbit.mCount==0&&
            orbit.has('P')&&orbit.val('P')==(diamond?13.0:12.0)&&orbit.has('B')&&orbit.val('B')==((squareB4||diamondB4)?4.0:(squareB3||diamondB3)?3.0:(squareB2||diamondB2)?2.0:1.0)):
            (orbit.has('P')&&orbit.val('P')==11.0);
    }
    bool E251SquareConfirmationSource(const NCProgramCache& cache,int armPC) noexcept
    {
        const auto* arm=cache.TryGetLine(armPC);
        const auto* source=armPC<(std::numeric_limits<int>::max)()?cache.TryGetLine(armPC+1):nullptr;
        if(!arm)return false;
        const bool tagged=arm->parsedBlock.has('L')&&(EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),250U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),251U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),252U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),253U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),255U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),256U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),257U)||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),258U)||E260OneMmMode(arm->parsedBlock)!=0U||
            EDM77::Detail::EqualLiteral(arm->parsedBlock.expression('L'),264U)||E263ExpansionMode(arm->parsedBlock)!=0U||E266FixedRadiusMode(arm->parsedBlock)!=0U||E267FixedRadiusWorkMode(arm->parsedBlock)!=0U);
        const bool square=source&&source->parsedBlock.has('P')&&(EDM77::Detail::EqualLiteral(source->parsedBlock.expression('P'),12U)||
            EDM77::Detail::EqualLiteral(source->parsedBlock.expression('P'),13U));
        if(!tagged&&!square)return true;
        NCBlock orbit{};
        return source&&EDM77::Detail::Clean(source->parsedBlock)&&
            NCPreparedBlockQueueShadow::TryBuildLiteralBlock(source->parsedBlock,orbit)&&
            E251SquareSourcePair(arm->parsedBlock,orbit);
    }
    bool E108ProfileOnlyNPQRSIJK(const NCParsedBlock& b, unsigned selector, EDM97::Tuning& tuning,
        EDM107::FlushProfile& profile) noexcept
    {
        const bool rejectBeforeArm=E216RejectBeforeArm(b,selector);
        const bool centerNominalHold=E217CenterNominalHold(b,selector);
        const bool centerFlushHold=E218CenterFlushHold(b,selector)||E219CenterReturnHold(b,selector)||E220CenterAnchorHold(b,selector)||E221CenterStartHold(b,selector)||E222CenterTopDwellHold(b,selector)||E224CenterDwellHold(b,selector)||E225CenterReturnDwellHold(b,selector)||E226CenterFinalDwellHold(b,selector)||E227CenterInitialDwellHold(b,selector)||E228CenterBrakeHold(b,selector);
        const bool centerRepeat=E230CenterRepeat(b,selector);
        const bool centerSecondFlushHold=E231CenterSecondFlushHold(b,selector);
        const bool centerSecondReturnHold=E232CenterSecondReturnHold(b,selector);
        const bool parentSequence=E233CenterParentSequence(b,selector)||E234ArmSequence(b,selector)||E237ArmSequence(b,selector);
        const bool squareB1=E250SquareB1Arm(b,selector)||E251SquareB2Arm(b,selector)||E252SquareB3Arm(b,selector)||E253SquareB4Arm(b,selector)||E255DiamondB1Arm(b,selector)||E256DiamondB2Arm(b,selector)||E257DiamondB3Arm(b,selector)||E258DiamondB4Arm(b,selector)||E260OneMmArm(b,selector)||E261PolygonB2Arm(b,selector)||E263ExpansionArm(b,selector)||E266FixedRadiusArm(b,selector)||E267FixedRadiusWorkArm(b,selector);
        if (b.has('L')&&!rejectBeforeArm&&!centerNominalHold&&!centerFlushHold&&!centerRepeat&&!centerSecondFlushHold&&!centerSecondReturnHold&&!parentSequence&&!squareB1)
        {
            unsigned selection = 0U;
            for (unsigned value = 1U; value <= 3U; ++value)
                if (EDM77::Detail::EqualLiteral(b.expression('L'), value)) selection = value;
            // EDM200 keeps L3's crossed-history B2 checkpoint meaning. The
            // G76 commissioning arm has no H1 retreat; L3 never adds one.
            if (!EDM130::ValidFlushHoldSelection(selector, selection) ||
                (selector == EDM173::B2ArmP ? (selection != 3U || b.has('H')) : !b.has('H'))) return false;
        }
        if ((selector != 109U && selector != 111U && selector != 113U && selector != 115U && selector != 117U && selector != 119U && selector != 121U && selector != 123U && selector != 125U && selector != 127U && selector != 129U && selector != 132U && selector != 134U && selector != 136U && selector != 138U && selector != 140U && selector != 142U && selector != 144U && selector != 146U && selector != 148U && selector != 150U && selector != 152U && selector != 154U && selector != 156U && selector != 162U && selector != 164U && selector != 166U && selector != 168U && selector != 170U && selector != 172U && selector != 174U && selector != 176U && selector != 178U && selector != 180U && selector != 182U && selector != 184U && selector != 186U && selector != 188U && selector != 190U && selector != 192U && selector != 194U && selector != 196U && selector != 198U && selector != 200U && selector != 202U && selector != 204U && selector != 206U && selector != 208U && selector != 210U && selector != 212U && selector != 214U && selector != 216U && selector != 218U && selector != 220U && selector != 222U && selector != 224U && selector != 226U && selector != 228U && selector != 230U && selector != 232U && selector != 234U && selector != 236U && selector != 238U && selector != 240U && selector != 242U && selector != 244U && selector != 246U && selector != 248U && selector != 250U && selector != 252U && selector != 254U && selector != 256U && selector != 258U && selector != 260U && selector != 262U && selector != 264U && selector != 266U && selector != 268U && selector != 270U && selector != 272U && selector != 274U && selector != 276U && selector != 278U && selector != 280U && selector != 282U && selector != 284U && selector != 286U && selector != 288U && selector != 290U && selector != 292U && selector != 294U && selector != 296U && selector != 298U && selector != 300U) || !EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) ||
            !EDM77::Detail::OnlyAddresses(b, rejectBeforeArm||centerNominalHold||centerFlushHold||centerRepeat||centerSecondFlushHold||centerSecondReturnHold||parentSequence||squareB1||selector == EDM173::B2ArmP ? "NPQRSIJKL" :
                EDM149::IsFlushHoldTestArm(selector) ? "NPQRSIJKHL" :
                EDM149::IsManualVoltageTestArm(selector) ? "NPQRSIJKH" : "NPQRSIJK") || !b.has('P') ||
            !EDM77::Detail::EqualLiteral(b.expression('P'), selector) ||
            (b.has('H') && !EDM77::Detail::EqualLiteral(b.expression('H'), 1U)) ||
            !b.has('Q') || !b.has('R') || !b.has('S') || !b.has('I') || !b.has('J') || !b.has('K')) return false;
        EDM97::Tuning parsed{};
        EDM107::FlushProfile parsedProfile{};
        double stroke = 0.0, speed = 0.0, dwell = 0.0;
        if (!E94Number(b.expression('Q'), parsed.positiveGain) ||
            !E94Number(b.expression('R'), parsed.negativeGain) ||
            !E94Number(b.expression('S'), parsed.deadbandV) || !EDM105::ValidTuning(parsed) ||
            !E94Number(b.expression('I'), stroke) || !E94Number(b.expression('J'), speed) ||
            !E94Number(b.expression('K'), dwell) ||
            (E131SpeedArm(selector) && speed != 50.0) ||
            (E132HeightArm(selector) && (stroke != 10.0 || dwell != 100.0)) ||
            !EDM107::ParseFlushProfile(stroke, speed, dwell, parsedProfile)) return false;
        tuning = parsed; profile = parsedProfile;
        return true;
    }
    bool E107ProfileOnlyNPQRSIJK(const NCParsedBlock& b, EDM97::Tuning& tuning,
        EDM107::FlushProfile& profile) noexcept
    { return E108ProfileOnlyNPQRSIJK(b, 109U, tuning, profile); }
    bool E149M00Shape(const NCParsedBlock& block) noexcept
    {
        return EDM77::Detail::Clean(block) && !block.isEmpty && block.gCount == 0 && block.mCount == 1 &&
            EDM77::Detail::EqualLiteral(block.mExpressions[0], 0U) && EDM77::Detail::OnlyAddresses(block, "N");
    }
    bool E94Preamble(const NCParsedBlock& b) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::OnlyAddresses(b, "N")) return false;
        const unsigned codes[] = {21U, 17U, 91U, 15U, 40U, 49U, 69U, 169U, 50U, 163U};
        for (unsigned code : codes)
            if (EDM77::Detail::EqualLiteral(b.gExpressions[0], code)) return true;
        return false;
    }
    // EDM191 admits all sixty existing WCS rows through the shared decoder.
    // Keep literal syntax, complete row validation and stopped EDM173 scope.
    bool E190WorkCoordinateSelection(const NCParsedBlock& b) noexcept
    {
        if(b.gCount!=1) return false;
        std::uint32_t code=0U;int row=0;
        return EDM77::Detail::UnsignedLiteral(b.gExpressions[0],959U,code)&&
            TryDecodeNCWorkCoordinateCode(static_cast<int>(code),row);
    }
    // Setup syntax only. This grants no motion or arm authority; the caller
    // binds it to the unique preflighted EDM173 bracket before M00.
    bool E178CoordinatePreamble(const NCParsedBlock& b) noexcept
    {
        if(!EDM77::Detail::Clean(b)||b.isEmpty||b.gCount!=1||b.mCount!=0) return false;
        if(EDM77::Detail::EqualLiteral(b.gExpressions[0],90U))
            return EDM77::Detail::OnlyAddresses(b,"N");
        // Select an existing WCS row without axis words or table writes.
        // Keep these selectors out of the shared E94 profile preamble.
        if(E190WorkCoordinateSelection(b))
            return EDM77::Detail::OnlyAddresses(b,"N");
        // G51 uses the existing positive uniform XYZ scale and explicit
        // program-space center. This is stopped setup, before M00/arm only.
        if(EDM77::Detail::EqualLiteral(b.gExpressions[0],51U))
        {
            double x=0.0,y=0.0,z=0.0,factor=0.0;
            return EDM77::Detail::OnlyAddresses(b,"NXYZP")&&
                b.has('X')&&b.has('Y')&&b.has('Z')&&b.has('P')&&
                E94Number(b.expression('X'),x)&&E94Number(b.expression('Y'),y)&&
                E94Number(b.expression('Z'),z)&&E94Number(b.expression('P'),factor)&&factor>0.0;
        }
        // Existing XYZ mirror selectors are stopped setup only. Each chosen
        // axis carries its explicit program-coordinate center; cancellation
        // may omit all axes to clear the complete previous mirror selection.
        const bool mirrorSelect=EDM77::Detail::EqualLiteral(b.gExpressions[0],151U);
        if(mirrorSelect||EDM77::Detail::EqualLiteral(b.gExpressions[0],150U))
        {
            double x=0.0,y=0.0,z=0.0;
            return EDM77::Detail::OnlyAddresses(b,"NXYZ")&&
                (!mirrorSelect||b.has('X')||b.has('Y')||b.has('Z'))&&
                (!b.has('X')||E94Number(b.expression('X'),x))&&
                (!b.has('Y')||E94Number(b.expression('Y'),y))&&
                (!b.has('Z')||E94Number(b.expression('Z'),z));
        }
        // Reuse the ordinary H decoder, including H0 cancellation. This
        // selects an existing row; table validation and freezing remain with
        // CoordinateManager and the existing stopped transition lifecycle.
        const bool positiveH=EDM77::Detail::EqualLiteral(b.gExpressions[0],43U);
        if(positiveH||EDM77::Detail::EqualLiteral(b.gExpressions[0],44U))
        {
            double h=0.0;int mode=49,index=0;
            return EDM77::Detail::OnlyAddresses(b,"NH")&&b.has('H')&&
                E94Number(b.expression('H'),h)&&
                TryDecodeNCToolLengthSelection(positiveH?43:44,true,h,mode,index);
        }
        // Select an existing WORK row with an optional complete literal XY
        // center. The stopped coordinate handler decides whether its angle
        // requires that center and converts it through the old frozen frame.
        // This syntax check never captures the current native position.
        if(EDM77::Detail::EqualLiteral(b.gExpressions[0],168U))
        {
            double w=0.0,x=0.0,y=0.0;int mode=169,index=0;
            return EDM77::Detail::OnlyAddresses(b,"NWXY")&&b.has('W')&&
                b.has('X')==b.has('Y')&&
                (!b.has('X')||(E94Number(b.expression('X'),x)&&E94Number(b.expression('Y'),y)))&&
                E94Number(b.expression('W'),w)&&
                TryDecodeNCWorkSelection(168,true,w,mode,index);
        }
        if(!EDM77::Detail::EqualLiteral(b.gExpressions[0],68U)||
            !EDM77::Detail::OnlyAddresses(b,"NXYR")||!b.has('X')||!b.has('Y')||!b.has('R')) return false;
        double x=0.0,y=0.0,angle=0.0;
        return E94Number(b.expression('X'),x)&&E94Number(b.expression('Y'),y)&&
            E94Number(b.expression('R'),angle)&&std::fabs(angle)<=360.0;
    }
    bool E94Move(const NCParsedBlock& b, std::array<double, 6U>& delta, double& feed) noexcept
    {
        if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
            !EDM77::Detail::EqualLiteral(b.gExpressions[0], 1U) ||
            !EDM77::Detail::OnlyAddresses(b, "NXYZCUVF") || !b.has('F') ||
            !E94Number(b.expression('F'), feed) || feed <= 0.0 || feed > .12) return false;
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (!b.has("XYZCUV"[axis]) || !E94Number(b.expression("XYZCUV"[axis]), delta[axis]) || delta[axis] == 0.0)
                return false;
        const double length = std::hypot(std::hypot(delta[0], delta[1]), delta[2]);
        if (!std::isfinite(length) || length <= 0.0) return false;
        const double seconds = length * 60.0 / feed;
        if (!std::isfinite(seconds) || seconds <= 0.0 || seconds > 120.0) return false;
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (std::fabs(delta[axis]) / seconds > (axis < 3U ? .002 : .05)) return false;
        return true;
    }
    unsigned E116ArmP(EDM116::Fixture fixture) noexcept
    {
        return fixture == EDM116::Fixture::Modal127 ? 162U : fixture == EDM116::Fixture::ModalComplement127 ? 164U :
            fixture == EDM116::Fixture::ArcPair124 ? 150U :
            fixture == EDM116::Fixture::ArcPairComplement124 ? 152U :
            fixture == EDM116::Fixture::Complement117 ? 132U : 129U;
    }
    unsigned E116FinishP(EDM116::Fixture fixture) noexcept
    {
        return E116ArmP(fixture) + 1U;
    }
    bool E116Selector(const NCParsedBlock& b, EDM116::Mode& mode) noexcept
    {
        double value = 0.0;
        return EDM77::Detail::Clean(b) && !b.isEmpty && b.gCount == 1 && b.mCount == 0 &&
            EDM77::Detail::EqualLiteral(b.gExpressions[0], 180U) &&
            EDM77::Detail::OnlyAddresses(b, "NPB") && b.has('P') && b.has('B') &&
            EDM77::Detail::EqualLiteral(b.expression('P'), 131U) &&
            E94Number(b.expression('B'), value) && EDM116::ParseMode(value, mode);
    }
    bool E127Standalone(const NCParsedBlock& b, EDM116::Mode& mode) noexcept
    {
        double value = 0.0;
        return EDM77::Detail::Clean(b) && !b.isEmpty && b.gCount == 0 && b.mCount == 0 &&
            EDM77::Detail::OnlyAddresses(b, "NB") && b.has('B') &&
            E94Number(b.expression('B'), value) && EDM116::ParseMode(value, mode);
    }
    // EDM155 omitted center components are this literal's +0, never modal state.
    // Earlier fixtures still require both authored I and J words.
    bool E155ArcCenter(const NCParsedBlock& b, std::array<double, 2U>& center,
        EDM116::Fixture fixture) noexcept
    {
        center.fill(0.0);
        const bool sparse = fixture == EDM116::Fixture::ModalSparseArc155 || fixture == EDM116::Fixture::ModalProgram156 || fixture == EDM116::Fixture::ModalPause157 || fixture == EDM116::Fixture::ModalXYZVisibleFlush159 || fixture == EDM116::Fixture::ModalSixVisibleFlush167;
        if (sparse ? (!b.has('I') && !b.has('J')) : (!b.has('I') || !b.has('J'))) return false;
        return (!b.has('I') || E94Number(b.expression('I'), center[0])) &&
            (!b.has('J') || E94Number(b.expression('J'), center[1]));
    }
    bool E114Move(const NCParsedBlock& b, unsigned step, std::array<double, 6U>& delta, double& feed,
        EDM116::Fixture fixture = EDM116::Fixture::Canonical116, bool authoredLine = false) noexcept
    {
        if (!EDM116::ValidFixture(fixture) || step >= EDM114::RequiredRows) return false;
        if (fixture == EDM116::Fixture::ModalVisibleFlush158)
        {
            delta.fill(0.0);
            return step == 0U && EDM77::Detail::Clean(b) && !b.isEmpty && b.gCount == 1 && b.mCount == 0 &&
                EDM77::Detail::EqualLiteral(b.gExpressions[0], 1U) &&
                EDM77::Detail::OnlyAddresses(b, "NZF") && b.has('Z') && b.has('F') &&
                E94Number(b.expression('Z'), delta[2]) && E94Number(b.expression('F'), feed) &&
                delta[2] == -0.0015 && feed == 0.12;
        }
        int authoredCode = 0;
        if (EDM116::IsAuthoredOrderFixture(fixture))
        {
            double literalCode = 0.0;
            if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
                !E94Number(b.gExpressions[0], literalCode) ||
                (literalCode != 1.0 && literalCode != 2.0 && literalCode != 3.0)) return false;
            authoredCode = static_cast<int>(literalCode);
        }
        if (fixture == EDM116::Fixture::ModalZ149 ||
            (fixture == EDM116::Fixture::ModalXYZ150 && !EDM116::IsArcStep(step,fixture)))
        {
            delta.fill(0.0);
            if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
                !EDM77::Detail::EqualLiteral(b.gExpressions[0], 1U) ||
                !EDM77::Detail::OnlyAddresses(b, "NXYZCUVF") || !b.has('Z') || !b.has('F') ||
                (fixture == EDM116::Fixture::ModalXYZ150 && (!b.has('X') || !b.has('Y'))) ||
                !E94Number(b.expression('F'), feed)) return false;
            for (unsigned axis = 0U; axis < 6U; ++axis)
                if (b.has("XYZCUV"[axis]) && !E94Number(b.expression("XYZCUV"[axis]), delta[axis])) return false;
            return fixture == EDM116::Fixture::ModalXYZ150 ? EDM149::ValidXYZIncrement(delta,feed) : EDM149::ValidZIncrement(delta,feed);
        }
        if (!EDM116::IsArcStep(step, fixture, authoredCode))
        {
            if (fixture == EDM116::Fixture::ModalSparse154 || fixture == EDM116::Fixture::ModalSparseArc155 || fixture == EDM116::Fixture::ModalProgram156 || fixture == EDM116::Fixture::ModalPause157 || fixture == EDM116::Fixture::ModalXYZVisibleFlush159 || fixture == EDM116::Fixture::ModalSixVisibleFlush167)
            {
                delta.fill(0.0);
                if (!EDM77::Detail::Clean(b) || b.isEmpty || b.gCount != 1 || b.mCount != 0 ||
                    !EDM77::Detail::EqualLiteral(b.gExpressions[0], 1U) ||
                    !EDM77::Detail::OnlyAddresses(b, "NXYZCUVF") || !b.has('F') ||
                    !E94Number(b.expression('F'), feed)) return false;
                for (unsigned axis = 0U; axis < 6U; ++axis)
                    if (b.has("XYZCUV"[axis]) && !E94Number(b.expression("XYZCUV"[axis]), delta[axis])) return false;
                return EDM149::ValidSparseIncrement(delta, feed);
            }
            if (!E94Move(b, delta, feed) || feed != 0.12) return false;
            if (authoredLine || fixture == EDM116::Fixture::ModalSix151 || EDM116::IsBoundedArcFixture(fixture)) return EDM96::ValidIncrement(delta, feed);
            for (unsigned a = 0U; a < 6U; ++a)
                if (delta[a] != ((step & 1U) ? -1.0 : 1.0) * (a < 3U ? 0.001 : 0.020)) return false;
            return true;
        }
        delta.fill(0.0);
        std::array<double, 2U> center{};
        const bool sparse = fixture == EDM116::Fixture::ModalSparseArc155 || fixture == EDM116::Fixture::ModalProgram156 || fixture == EDM116::Fixture::ModalPause157 || fixture == EDM116::Fixture::ModalXYZVisibleFlush159 || fixture == EDM116::Fixture::ModalSixVisibleFlush167;
        return EDM77::Detail::Clean(b) && !b.isEmpty && b.gCount == 1 && b.mCount == 0 &&
            EDM77::Detail::EqualLiteral(b.gExpressions[0], static_cast<unsigned>(EDM116::ExpectedGCode(step, fixture, authoredCode))) &&
            EDM77::Detail::OnlyAddresses(b, "NXYIJF") && b.has('F') &&
            (sparse ? (b.has('X') || b.has('Y')) : (b.has('X') && b.has('Y'))) &&
            (!b.has('X') || E94Number(b.expression('X'), delta[0])) &&
            (!b.has('Y') || E94Number(b.expression('Y'), delta[1])) && E155ArcCenter(b, center, fixture) &&
            E94Number(b.expression('F'), feed) &&
            EDM116::ValidArcRecipe(EDM116::ExpectedGCode(step, fixture, authoredCode), step, delta, center, feed, fixture);
    }
    bool E152ArcAdmission(const NCParsedBlock& geometry, unsigned step,
        const std::array<double, 6U>& increment, double feed,
        const std::array<double, 6U>& origin, double& lengthMm,
        EDM116::Fixture fixture = EDM116::Fixture::ModalArc152) noexcept
    {
        lengthMm = 0.0;
        std::array<double, 6U> literalIncrement{}; double literalFeed = 0.0;
        std::array<double, 2U> center{}; EDM112::Curve curve{}; EDM112::Bounds bounds{};
        double literalCode = 0.0;
        if (!EDM116::IsBoundedArcFixture(fixture) || !E94Number(geometry.gExpressions[0], literalCode) || !E114Move(geometry, step, literalIncrement, literalFeed, fixture) ||
            literalIncrement != increment || literalFeed != feed ||
            !E155ArcCenter(geometry, center, fixture) ||
            !EDM114::BuildBoundedXYArc(EDM116::ExpectedGCode(step, fixture, static_cast<int>(literalCode)),
                increment, center, feed, curve) || !EDM112::WholeCurveBounds(curve, bounds) || !bounds.valid) return false;
        // The builder uses a synthetic pulse scale. Only its native bounds
        // can be translated into this arm-relative millimetre/degree frame.
        for (unsigned axis = 0U; axis < 6U; ++axis)
        {
            const double low = origin[axis] + bounds.minNative[axis];
            const double high = origin[axis] + bounds.maxNative[axis];
            const double limit = axis < 3U ? 0.025 : 0.5;
            if (!std::isfinite(origin[axis]) || !std::isfinite(low) || !std::isfinite(high) ||
                low < -limit || high > limit) return false;
        }
        if (!std::isfinite(curve.lengthMm) || curve.lengthMm <= 0.0) return false;
        lengthMm = curve.lengthMm; return true;
    }
    bool E115SameExcursion(const EDM115::Feedback& a, const EDM115::Feedback& b) noexcept
    {
        return a.tick == b.tick &&
            a.nowUs == b.nowUs &&
            a.startTick == b.startTick &&
            a.startUs == b.startUs &&
            a.started == b.started &&
            a.completed == b.completed &&
            a.backwardCrossings == b.backwardCrossings &&
            a.forwardCrossings == b.forwardCrossings &&
            a.anchorReturned == b.anchorReturned &&
            a.span == b.span &&
            a.settledCycles == b.settledCycles &&
            a.completedDwells == b.completedDwells &&
            a.direction == b.direction &&
            a.rowKind == b.rowKind &&
            a.phase == b.phase &&
            a.reason == b.reason &&
            a.geometryValid == b.geometryValid &&
            a.active == b.active &&
            a.moving == b.moving &&
            a.endpointReady == b.endpointReady &&
            a.stopRequested == b.stopRequested &&
            a.stopped == b.stopped &&
            a.faulted == b.faulted &&
            a.complete == b.complete &&
            EDM109::SameDouble(a.q, b.q) &&
            EDM109::SameDouble(a.v, b.v) &&
            EDM109::SameDouble(a.a, b.a) &&
            EDM109::SameDouble(a.progress, b.progress) &&
            EDM109::SameDouble(a.length, b.length) &&
            EDM109::SameDouble(a.maximumVelocity, b.maximumVelocity) &&
            EDM109::SameDouble(a.acceleration, b.acceleration) &&
            EDM109::SameDouble(a.deceleration, b.deceleration);
    }
    bool E115SameFeedback(const EDM115::MotionFeedback& a, const EDM115::MotionFeedback& b) noexcept
    {
        return a.session == b.session &&
            a.armEpoch == b.armEpoch &&
            a.ownerGeneration == b.ownerGeneration &&
            a.previousBoundaryTick == b.previousBoundaryTick &&
            a.previousCommit == b.previousCommit &&
            a.lastPhysicalTick == b.lastPhysicalTick &&
            a.rangeResets == b.rangeResets &&
            a.eventsStarted == b.eventsStarted &&
            a.eventsCompleted == b.eventsCompleted &&
            a.completedDwellsTotal == b.completedDwellsTotal &&
            a.anchorReturnsTotal == b.anchorReturnsTotal &&
            a.currentKind == b.currentKind &&
            a.previousKind == b.previousKind &&
            a.enabled == b.enabled &&
            a.tokenValidated == b.tokenValidated &&
            a.geometryCurrent == b.geometryCurrent &&
            EDM109::SameDouble(a.anchorQ, b.anchorQ) &&
            E115SameExcursion(a.excursion, b.excursion) &&
            EDM114::SameSource(a.trigger, b.trigger) &&
            EDM115::SameToken(a.previous, b.previous) &&
            EDM109::SamePulse(a.anchorPulse, b.anchorPulse) &&
            EDM109::SamePulse(a.returnedPulse, b.returnedPulse);
    }
    bool E116SameEvent(const EDM116::MotionFeedback& a, const EDM116::MotionFeedback& b) noexcept
    {
        return a.fixture == b.fixture && EDM116::SamePolicy(a.policy, b.policy) &&
            a.modeStarted == b.modeStarted && a.modeCompleted == b.modeCompleted &&
            a.deadlineUs == b.deadlineUs && EDM109::SameDouble(a.strokeNativeMm, b.strokeNativeMm) &&
            EDM109::SamePulse(a.retreatPulse, b.retreatPulse) &&
            a.topDwellTicks == b.topDwellTicks && a.topDwellUs == b.topDwellUs &&
            a.returnDwellTicks == b.returnDwellTicks && a.returnDwellUs == b.returnDwellUs &&
            a.topSettledCycles == b.topSettledCycles && a.returnSettledCycles == b.returnSettledCycles &&
            a.session == b.session &&
            a.armEpoch == b.armEpoch &&
            a.ownerGeneration == b.ownerGeneration &&
            a.previousBoundaryTick == b.previousBoundaryTick &&
            a.previousCommit == b.previousCommit &&
            a.lastPhysicalTick == b.lastPhysicalTick &&
            a.rangeResets == b.rangeResets &&
            a.eventsStarted == b.eventsStarted &&
            a.eventsCompleted == b.eventsCompleted &&
            a.completedDwellsTotal == b.completedDwellsTotal &&
            a.anchorReturnsTotal == b.anchorReturnsTotal &&
            a.currentKind == b.currentKind &&
            a.previousKind == b.previousKind &&
            a.enabled == b.enabled &&
            a.tokenValidated == b.tokenValidated &&
            a.geometryCurrent == b.geometryCurrent &&
            EDM109::SameDouble(a.anchorQ, b.anchorQ) &&
            E115SameExcursion(a.excursion, b.excursion) &&
            EDM114::SameSource(a.trigger, b.trigger) &&
            EDM115::SameToken(a.previous, b.previous) &&
            EDM109::SamePulse(a.anchorPulse, b.anchorPulse) &&
            EDM109::SamePulse(a.returnedPulse, b.returnedPulse);
    }
    bool E115AnchorMatches(const EDM114::Feedback& row, double q, const EDM115::Point& anchor) noexcept
    {
        if (!row.geometryValid || !std::isfinite(q) || !std::isfinite(row.pulseLength) ||
            row.pulseLength <= 0.0 || q < row.pulseLength * 0.5 || q >= row.pulseLength) return false;
        EDM115::Point point = row.startPulse;
        if (row.kind == EDM114::Kind::Line)
        {
            // This exact ratio/order is checked by BeginEDM96Segment before
            // binding the original normal mapping. It is never chordified from an arc.
            for (unsigned a = 0U; a < 6U; ++a)
                point[a] = row.startPulse[a] + q * ((row.endPulse[a] - row.startPulse[a]) / row.pulseLength);
        }
        else if (row.kind == EDM114::Kind::Arc)
        {
            if (!std::isfinite(row.radiusPulse) || row.radiusPulse <= 0.0 ||
                !std::isfinite(row.startAngle) || !std::isfinite(row.sweepRadians)) return false;
            const double angle = row.startAngle + (q / row.pulseLength) * row.sweepRadians;
            point[0] = row.centerPulse[0] + row.radiusPulse * std::cos(angle);
            point[1] = row.centerPulse[1] + row.radiusPulse * std::sin(angle);
        }
        else return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (!std::isfinite(point[a]) || !std::isfinite(anchor[a]) ||
                !EDM109::SameDouble(point[a], anchor[a])) return false;
        return true;
    }
    bool E114SameFeedback(const EDM114::Feedback& a, const EDM114::Feedback& b) noexcept
    {
        return EDM114::SameSource(a.source, b.source) &&
            a.session == b.session &&
            a.armEpoch == b.armEpoch &&
            a.ownerGeneration == b.ownerGeneration &&
            a.lastTick == b.lastTick &&
            a.lastMappingTick == b.lastMappingTick &&
            a.lastMappingUs == b.lastMappingUs &&
            a.samples == b.samples &&
            a.forwardSamples == b.forwardSamples &&
            a.reverseSamples == b.reverseSamples &&
            a.flushSamples == b.flushSamples &&
            a.rowSamples == b.rowSamples &&
            a.rowForward == b.rowForward &&
            a.rowReverse == b.rowReverse &&
            a.rowFlush == b.rowFlush &&
            a.begunRows == b.begunRows &&
            a.completedRows == b.completedRows &&
            a.lineRows == b.lineRows &&
            a.arcRows == b.arcRows &&
            a.cwRows == b.cwRows &&
            a.ccwRows == b.ccwRows &&
            a.endpointRows == b.endpointRows &&
            a.kind == b.kind &&
            a.reason == b.reason &&
            a.enabled == b.enabled &&
            a.geometryValid == b.geometryValid &&
            a.mappedOutput == b.mappedOutput &&
            a.failed == b.failed &&
            a.rowComplete == b.rowComplete &&
            a.completed == b.completed &&
            a.stopActive == b.stopActive &&
            a.stopped == b.stopped &&
            EDM109::SameDouble(a.radiusPulse, b.radiusPulse) &&
            EDM109::SameDouble(a.startAngle, b.startAngle) &&
            EDM109::SameDouble(a.sweepRadians, b.sweepRadians) &&
            EDM109::SameDouble(a.pulseLength, b.pulseLength) &&
            EDM109::SameDouble(a.centerPulse[0], b.centerPulse[0]) &&
            EDM109::SameDouble(a.centerPulse[1], b.centerPulse[1]) &&
            EDM109::SamePulse(a.startPulse, b.startPulse) &&
            EDM109::SamePulse(a.endPulse, b.endPulse) &&
            EDM109::SamePulse(a.pulsePerNative, b.pulsePerNative);
    }
    bool E114Counts(const EDM114::Feedback& f, unsigned completed,
        EDM116::Fixture fixture = EDM116::Fixture::Canonical116) noexcept
    {
        if (!EDM116::ValidFixture(fixture) || completed > EDM114::RequiredRows || !f.enabled || f.failed || f.reason != EDM114::Reason::None ||
            f.stopActive || f.stopped || f.begunRows != completed || f.completedRows != completed ||
            f.endpointRows != completed || f.completed != (completed == EDM114::RequiredRows) ||
            f.rowComplete != (completed != 0U) || f.geometryValid != (completed != 0U) ||
            f.mappedOutput != (completed != 0U)) return false;
        unsigned lines = 0U, arcs = 0U, cw = 0U, ccw = 0U;
        for (unsigned i = 0U; i < completed; ++i)
            if (EDM116::IsArcStep(i, fixture)) { ++arcs; if (EDM116::ExpectedGCode(i, fixture) == 2) ++cw; else ++ccw; }
            else ++lines;
        if (f.lineRows != lines || f.arcRows != arcs || f.cwRows != cw || f.ccwRows != ccw) return false;
        if (completed == 0U) return f.kind == EDM114::Kind::None;
        if (f.kind != (EDM116::IsArcStep(completed - 1U, fixture) ? EDM114::Kind::Arc : EDM114::Kind::Line) ||
            !f.lastMappingTick || f.lastTick != f.lastMappingTick || !f.lastMappingUs ||
            !f.rowSamples || !f.rowForward || !f.rowReverse ||
            f.rowForward > f.rowSamples || f.rowReverse > f.rowSamples - f.rowForward ||
            f.rowFlush > f.rowSamples || (!(completed & 1U) ? f.rowFlush != 0ULL : f.rowFlush == 0ULL) ||
            f.samples < f.rowSamples || f.forwardSamples < f.rowForward ||
            f.reverseSamples < f.rowReverse || f.flushSamples < f.rowFlush ||
            f.forwardSamples > f.samples || f.reverseSamples > f.samples - f.forwardSamples ||
            f.flushSamples > f.samples || !std::isfinite(f.pulseLength) || f.pulseLength <= 0.0) return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (!std::isfinite(f.startPulse[a]) || !std::isfinite(f.endPulse[a]) ||
                !std::isfinite(f.pulsePerNative[a]) || f.pulsePerNative[a] <= 0.0) return false;
        return true;
    }
    bool E126Counts(const EDM114::Feedback& f, unsigned completed,
        EDM116::Fixture fixture = EDM116::Fixture::Canonical116) noexcept
    {
        if ((fixture != EDM116::Fixture::ArcPair124 && fixture != EDM116::Fixture::ArcPairComplement124) || completed > EDM114::RequiredRows || !f.enabled || f.failed || f.reason != EDM114::Reason::None ||
            f.stopActive || f.stopped || f.begunRows != completed || f.completedRows != completed ||
            f.endpointRows != completed || f.completed != (completed == EDM114::RequiredRows) ||
            f.rowComplete != (completed != 0U) || f.geometryValid != (completed != 0U) ||
            f.mappedOutput != (completed != 0U)) return false;
        unsigned lines = 0U, arcs = 0U, cw = 0U, ccw = 0U;
        for (unsigned i = 0U; i < completed; ++i)
            if (EDM116::IsArcStep(i, fixture)) { ++arcs; if (EDM116::ExpectedGCode(i, fixture) == 2) ++cw; else ++ccw; }
            else ++lines;
        if (f.lineRows != lines || f.arcRows != arcs || f.cwRows != cw || f.ccwRows != ccw) return false;
        if (completed == 0U) return f.kind == EDM114::Kind::None;
        if (f.kind != (EDM116::IsArcStep(completed - 1U, fixture) ? EDM114::Kind::Arc : EDM114::Kind::Line) ||
            !f.lastMappingTick || f.lastTick != f.lastMappingTick || !f.lastMappingUs ||
            !f.rowSamples || !f.rowForward ||
            f.rowForward > f.rowSamples || f.rowReverse > f.rowSamples - f.rowForward ||
            f.rowFlush != 0ULL || f.flushSamples != 0ULL ||
            f.samples < f.rowSamples || f.forwardSamples < f.rowForward ||
            f.reverseSamples < f.rowReverse || f.flushSamples < f.rowFlush ||
            f.forwardSamples > f.samples || f.reverseSamples > f.samples - f.forwardSamples ||
            f.flushSamples > f.samples || !std::isfinite(f.pulseLength) || f.pulseLength <= 0.0) return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (!std::isfinite(f.startPulse[a]) || !std::isfinite(f.endPulse[a]) ||
                !std::isfinite(f.pulsePerNative[a]) || f.pulsePerNative[a] <= 0.0) return false;
        return true;
    }
    bool E128Counts(const EDM114::Feedback& f, unsigned completed,
        EDM116::Fixture fixture = EDM116::Fixture::Canonical116, bool workTime = false,
        const std::array<std::uint8_t, EDM114::RequiredRows>* authoredCodes = nullptr,
        unsigned expectedRows = EDM114::RequiredRows) noexcept
    {
        if (!expectedRows || expectedRows > EDM114::RequiredRows ||
            (fixture != EDM116::Fixture::ModalProgram156 && fixture != EDM116::Fixture::ModalPause157 && fixture != EDM116::Fixture::ModalVisibleFlush158 && fixture != EDM116::Fixture::ModalXYZVisibleFlush159 && fixture != EDM116::Fixture::ModalSixVisibleFlush167 && expectedRows != EDM114::RequiredRows) ||
            !EDM116::IsModalFixture(fixture) || completed > expectedRows || !f.enabled || f.failed || f.reason != EDM114::Reason::None ||
            f.stopActive || f.stopped || f.begunRows != completed || f.completedRows != completed ||
            f.endpointRows != completed || f.completed != (completed == expectedRows) ||
            f.rowComplete != (completed != 0U) || f.geometryValid != (completed != 0U) ||
            f.mappedOutput != (completed != 0U)) return false;
        if (EDM116::IsAuthoredOrderFixture(fixture) && !authoredCodes) return false;
        const auto codeAt = [&](unsigned row) noexcept -> int {
            return EDM116::ExpectedGCode(row, fixture, authoredCodes ? (*authoredCodes)[row] : 0);
        };
        unsigned lines = 0U, arcs = 0U, cw = 0U, ccw = 0U;
        for (unsigned i = 0U; i < completed; ++i)
        {
            const int code = codeAt(i);
            if (code == 2 || code == 3) { ++arcs; if (code == 2) ++cw; else ++ccw; }
            else if (code == 1) ++lines;
            else return false;
        }
        if (f.lineRows != lines || f.arcRows != arcs || f.cwRows != cw || f.ccwRows != ccw) return false;
        if (completed == 0U) return f.kind == EDM114::Kind::None;
        // EDM130 triggers by committed work time, so either row parity may have
        // zero or multiple flush events. IsEDM130CurrentSameThread checks their
        // timer, modal B, dwell and exact-return receipts independently.
        // Keep EDM128/129 fixed-row flush checks and all sample budgets intact.
        if (f.kind != ((codeAt(completed - 1U) == 2 || codeAt(completed - 1U) == 3) ? EDM114::Kind::Arc : EDM114::Kind::Line) ||
            !f.lastMappingTick || f.lastTick != f.lastMappingTick || !f.lastMappingUs ||
            !f.rowSamples || !f.rowForward ||
            f.rowForward > f.rowSamples || f.rowReverse > f.rowSamples - f.rowForward ||
            f.rowFlush > f.rowSamples || (!workTime && (!(completed & 1U) ? f.rowFlush != 0ULL : f.rowFlush == 0ULL)) ||
            f.samples < f.rowSamples || f.forwardSamples < f.rowForward ||
            f.reverseSamples < f.rowReverse || f.flushSamples < f.rowFlush ||
            f.forwardSamples > f.samples || f.reverseSamples > f.samples - f.forwardSamples ||
            f.flushSamples > f.samples || !std::isfinite(f.pulseLength) || f.pulseLength <= 0.0) return false;
        for (unsigned a = 0U; a < 6U; ++a)
            if (!std::isfinite(f.startPulse[a]) || !std::isfinite(f.endPulse[a]) ||
                !std::isfinite(f.pulsePerNative[a]) || f.pulsePerNative[a] <= 0.0) return false;
        return true;
    }
    template<class State>
    unsigned E156NominalMovementMask(const State& s) noexcept
    {
        if (!s.moves || s.moves > EDM114::RequiredRows) return 0U;
        unsigned mask = 0U;
        for (unsigned row = 0U; row < s.moves; ++row)
        {
            const auto& delta = s.increment[row];
            if (s.authoredGCode[row] == 1U)
            {
                if (!EDM149::ValidSparseIncrement(delta, s.feed[row])) return 0U;
                for (unsigned axis = 0U; axis < 6U; ++axis) if (delta[axis] != 0.0) mask |= 1U << axis;
            }
            else if (s.authoredGCode[row] == 2U || s.authoredGCode[row] == 3U)
            {
                if (s.feed[row] != 0.12 || (delta[0] == 0.0 && delta[1] == 0.0)) return 0U;
                for (unsigned axis = 0U; axis < 6U; ++axis)
                    if (!std::isfinite(delta[axis]) || (axis >= 2U && delta[axis] != 0.0)) return 0U;
                // Even an omitted/zero endpoint component moves inside a valid XY arc.
                mask |= 3U;
            }
            else return 0U;
        }
        return mask;
    }
    template<class State>
    bool E158VisiblePlan(const State& s) noexcept
    {
        if (!EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) ||
            s.gapModeFixture != EDM116::Fixture::ModalVisibleFlush158 || s.moves != 1U ||
            s.authoredGCode[0] != 1U || s.feed[0] != 0.12 || s.increment[0][2] != -0.0015) return false;
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (axis != 2U && s.increment[0][axis] != 0.0) return false;
        return true;
    }
    template<class State>
    bool E159VisiblePlan(const State& s) noexcept
    {
        if (!EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) ||
            s.gapModeFixture != EDM116::Fixture::ModalXYZVisibleFlush159 || s.moves != 3U) return false;
        const unsigned codes[3] = {1U, 2U, 3U};
        const double increments[3][6] = {{0.0015,-0.0015,-0.0015,0.0,0.0,0.0},
            {0.002,0.0,0.0,0.0,0.0,0.0},{-0.002,0.0,0.0,0.0,0.0,0.0}};
        for (unsigned row = 0U; row < 3U; ++row)
        {
            if (s.authoredGCode[row] != codes[row] || s.feed[row] != 0.12) return false;
            for (unsigned axis = 0U; axis < 6U; ++axis)
                if (s.increment[row][axis] != increments[row][axis]) return false;
        }
        return E156NominalMovementMask(s) == 7U;
    }
    template<class State>
    bool E167VisiblePlan(const State& s) noexcept
    {
        if (!EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) ||
            s.gapModeFixture != EDM116::Fixture::ModalSixVisibleFlush167 || s.moves != 3U) return false;
        const unsigned codes[3] = {1U, 2U, 3U};
        const double increments[3][6] = {{0.0015,-0.0015,-0.0015,0.020,-0.020,0.020},
            {0.002,0.0,0.0,0.0,0.0,0.0},{-0.002,0.0,0.0,0.0,0.0,0.0}};
        for (unsigned row = 0U; row < 3U; ++row)
        {
            if (s.authoredGCode[row] != codes[row] || s.feed[row] != 0.12) return false;
            for (unsigned axis = 0U; axis < 6U; ++axis)
                if (s.increment[row][axis] != increments[row][axis]) return false;
        }
        return E156NominalMovementMask(s) == 63U;
    }
    template<class State>
    bool E156BoundPlanCurrent(const State& s, const EDM130::Recipe& recipe) noexcept
    {
        if (EDM173::IsArm(s.gapHistoryArm)) return s.moves==1U && recipe.planRows==EDM173::CountForRecipe(recipe) && EDM130::ValidProgramPlan(recipe) &&
            EDM173::RecipeMatchesArm(s.gapHistoryArm,recipe) && EDM130::ValidRecipe(recipe);
        if (!EDM149::IsBoundedProgramTestArm(s.gapHistoryArm))
            return s.moves == EDM114::RequiredRows && !recipe.planRows && !recipe.planMovementMask &&
                !recipe.commissioningProfile && !recipe.holdRetreatMode && !recipe.flushHoldTest;
        const unsigned fixture = E149HistoryFixture(s.gapHistoryArm);
        return (EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm) ?
            (E167VisiblePlan(s) && EDM130::IsVisibleFlush167(recipe)) :
            EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm) ?
            (E159VisiblePlan(s) && EDM130::IsVisibleFlush159(recipe)) :
            EDM149::IsVisibleFlushTestArm(s.gapHistoryArm) ?
            (E158VisiblePlan(s) && recipe.flushStages.finalSpeedPercentScaled == 10000000U &&
                recipe.flushStages.approachDistanceScaled == 100ULL &&
                recipe.flushStages.initialSpeedPercentScaled == 1000000U && !recipe.flushStages.initialDistanceScaled) :
            !recipe.commissioningProfile) &&
            EDM149::IsProgramHistoryFixture(fixture) && s.gapModeFixture == E149ModalFixture(fixture) &&
            EDM130::HasBoundProgramPlan(recipe) && EDM130::ValidHistoryRecipeBinding(fixture, recipe) &&
            EDM130::ProgramRows(recipe) == s.moves &&
            EDM130::ProgramMovementMask(recipe) == E156NominalMovementMask(s);
    }
    template<class State>
    unsigned E156RequiredMovementMask(const State& s, const EDM130::Recipe& recipe) noexcept
    {
        if (EDM173::IsArm(s.gapHistoryArm)) return E156BoundPlanCurrent(s,recipe)?EDM130::ProgramMovementMask(recipe):64U;
        return EDM149::IsBoundedProgramTestArm(s.gapHistoryArm) ?
            (E156BoundPlanCurrent(s, recipe) ? EDM130::ProgramMovementMask(recipe) : 64U) :
            EDM149::MovementMaskForArm(s.gapHistoryArm);
    }
    template<class State>
    bool E156FinishTargetMatches(const State& s, const EDM130::Recipe& recipe,
        const EDM94::Feedback& guard, const EDM114::Feedback& last) noexcept
    {
        if (EDM173::IsArm(s.gapHistoryArm)) return E156BoundPlanCurrent(s,recipe) && guard.boundedProgram && guard.finishTargetBound &&
            guard.requiredMovementMask==EDM130::ProgramMovementMask(recipe) && last.session==s.session && last.armEpoch==s.armEpoch &&
            last.ownerGeneration==s.lease.generation && last.completed && !last.failed && last.rowComplete &&
            last.completedRows==EDM173::CountForRecipe(recipe) && last.source.step+1U==EDM173::CountForRecipe(recipe) && last.source.scope.session==s.session &&
            last.source.scope.run==s.run && last.source.scope.cache==s.cache && last.source.sourcePC==s.pc[0U] &&
            guard.finishTargetPulse==last.endPulse;
        if (!EDM149::IsBoundedProgramTestArm(s.gapHistoryArm)) return true;
        return E156BoundPlanCurrent(s, recipe) && guard.boundedProgram && guard.finishTargetBound &&
            guard.requiredMovementMask == EDM130::ProgramMovementMask(recipe) &&
            last.session == s.session && last.armEpoch == s.armEpoch &&
            last.ownerGeneration == s.lease.generation && last.completed && !last.failed && last.rowComplete &&
            last.completedRows == s.moves && last.source.step + 1U == s.moves &&
            last.source.scope.session == s.session && last.source.scope.run == s.run &&
            last.source.scope.cache == s.cache && last.source.sourcePC == s.pc[s.moves - 1U] &&
            guard.finishTargetPulse == last.endPulse;
    }
    bool E94Matches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!EDM77::Detail::Clean(parsed) || b.isEmpty != parsed.isEmpty || b.isGoto || b.isBlockSkip ||
            b.gCount != parsed.gCount || b.mCount != parsed.mCount) return false;
        for (int i = 0; i < b.gCount; ++i)
            if (!EDM77::Detail::EqualLiteral(parsed.gExpressions[i], static_cast<unsigned>(b.gCodes[i]))) return false;
        for (int i = 0; i < b.mCount; ++i)
            if (!EDM77::Detail::EqualLiteral(parsed.mExpressions[i], static_cast<unsigned>(b.mCode[i]))) return false;
        for (char a = 'A'; a <= 'Z'; ++a)
        {
            if (b.has(a) != parsed.has(a)) return false;
            if (!b.has(a)) continue;
            double value = 0.0;
            if (!E94Number(parsed.expression(a), value) || value != b.val(a)) return false;
        }
        return true;
    }
    // NC-only effective selection. Keep the authored B presence/value intact:
    // omitted B selects B4 for this G76 block, never a preceding modal B.
    // EDM192 admits explicit B0. Invalid is 255; an absent word stays B4.
    unsigned E181EffectiveOrbitMode(const NCBlock& b) noexcept
    {
        if (!b.has('B')) return EDM109::SameDouble(b.val('B'),0.0) ? 4U : 255U;
        const double value=b.val('B');
        if (EDM109::SameDouble(value,0.0)) return 0U;
        return value==1.0 || value==2.0 || value==3.0 || value==4.0 ?
            static_cast<unsigned>(value) : 255U;
    }
    // Explicit B0, K and the EDM181 omitted-B route require a complete authored literal
    // G76 row before arm; hidden expression/control fields grant nothing.
    // Other established literal profiles retain their original matcher.
    bool E180LiteralAngleSourceMatches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!E94Matches(parsed,b)) return false;
        const bool explicitB0=b.has('B')&&b.val('B')==0.0;
        if (explicitB0&&!EDM109::SameDouble(b.val('B'),0.0)) return false;
        if (!explicitB0&&!b.has('K')&&!(b.has('P')&&(b.val('P')==16.0||b.val('P')==21.0||b.val('P')==22.0||b.val('P')==23.0||b.val('P')==26.0)) && (b.gCode!=76 || b.has('B'))) return true;
        if (parsed.gCount!=1 || parsed.mCount!=0 || !b.hasG || b.gCode!=76 ||
            b.gCount!=1 || b.gCodes[0]!=76 || b.mCount!=0 || b.gotoTarget!=-1 ||
            parsed.assignment.prefix || !parsed.assignment.indexExpression.empty() ||
            !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
            !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty()) return false;
        for (std::size_t i=1U;i<parsed.gExpressions.size();++i) if (!parsed.gExpressions[i].empty()) return false;
        for (const auto& expression:parsed.mExpressions) if (!expression.empty()) return false;
        for (unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i) if (b.gCodes[i]!=0) return false;
        for (int code:b.mCode) if (code!=-1) return false;
        for (char word='A';word<='Z';++word)
            if (!parsed.has(word) && (!parsed.expression(word).empty() ||
                !EDM109::SameDouble(b.val(word),0.0))) return false;
        return true;
    }
    // Only the newly admitted H setup uses this complete source receipt.
    // Keep ordinary setup rows on their established admission path.
    bool E182ToolLengthSourceMatches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!E178CoordinatePreamble(parsed) || !E94Matches(parsed,b) ||
            !b.hasG || (b.gCode!=43 && b.gCode!=44) || b.gCount!=1 ||
            b.gCodes[0]!=b.gCode || b.mCount!=0 || b.gotoTarget!=-1 ||
            parsed.assignment.prefix || !parsed.assignment.indexExpression.empty() ||
            !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
            !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty()) return false;
        for (std::size_t i=1U;i<parsed.gExpressions.size();++i) if (!parsed.gExpressions[i].empty()) return false;
        for (const auto& expression:parsed.mExpressions) if (!expression.empty()) return false;
        for (unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i) if (b.gCodes[i]!=0) return false;
        for (int code:b.mCode) if (code!=-1) return false;
        for (char word='A';word<='Z';++word)
            if (!parsed.has(word) && (!parsed.expression(word).empty() ||
                !EDM109::SameDouble(b.val(word),0.0))) return false;
        return true;
    }
    // Complete source receipt for standalone G168 W, including its optional XY center.
    bool E183WorkSourceMatches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!E178CoordinatePreamble(parsed) || !E94Matches(parsed,b) ||
            !b.hasG || b.gCode!=168 || b.gCount!=1 ||
            b.gCodes[0]!=b.gCode || b.mCount!=0 || b.gotoTarget!=-1 ||
            parsed.assignment.prefix || !parsed.assignment.indexExpression.empty() ||
            !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
            !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty()) return false;
        for (std::size_t i=1U;i<parsed.gExpressions.size();++i) if (!parsed.gExpressions[i].empty()) return false;
        for (const auto& expression:parsed.mExpressions) if (!expression.empty()) return false;
        for (unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i) if (b.gCodes[i]!=0) return false;
        for (int code:b.mCode) if (code!=-1) return false;
        for (char word='A';word<='Z';++word)
            if (!parsed.has(word) && (!parsed.expression(word).empty() ||
                !EDM109::SameDouble(b.val(word),0.0))) return false;
        return true;
    }
    // Full authored/source receipt for the bounded standalone XYZ mirror setup.
    bool E185MirrorSourceMatches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!E178CoordinatePreamble(parsed) || !E94Matches(parsed,b) ||
            !b.hasG || (b.gCode!=150 && b.gCode!=151) || b.gCount!=1 ||
            b.gCodes[0]!=b.gCode || b.mCount!=0 || b.gotoTarget!=-1 ||
            parsed.assignment.prefix || !parsed.assignment.indexExpression.empty() ||
            !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
            !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty()) return false;
        for (std::size_t i=1U;i<parsed.gExpressions.size();++i) if (!parsed.gExpressions[i].empty()) return false;
        for (const auto& expression:parsed.mExpressions) if (!expression.empty()) return false;
        for (unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i) if (b.gCodes[i]!=0) return false;
        for (int code:b.mCode) if (code!=-1) return false;
        for (char word='A';word<='Z';++word)
            if (!parsed.has(word) && (!parsed.expression(word).empty() ||
                !EDM109::SameDouble(b.val(word),0.0))) return false;
        return true;
    }
    // Full authored/source receipt for the bounded standalone XYZ scaling setup.
    bool E186ScaleSourceMatches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!E178CoordinatePreamble(parsed) || !E94Matches(parsed,b) ||
            !b.hasG || b.gCode!=51 || b.gCount!=1 ||
            b.gCodes[0]!=b.gCode || b.mCount!=0 || b.gotoTarget!=-1 ||
            parsed.assignment.prefix || !parsed.assignment.indexExpression.empty() ||
            !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
            !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty()) return false;
        for (std::size_t i=1U;i<parsed.gExpressions.size();++i) if (!parsed.gExpressions[i].empty()) return false;
        for (const auto& expression:parsed.mExpressions) if (!expression.empty()) return false;
        for (unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i) if (b.gCodes[i]!=0) return false;
        for (int code:b.mCode) if (code!=-1) return false;
        for (char word='A';word<='Z';++word)
            if (!parsed.has(word) && (!parsed.expression(word).empty() ||
                !EDM109::SameDouble(b.val(word),0.0))) return false;
        return true;
    }
    // Bind the complete authored and resolved WCS selection before any
    // ordinary coordinate handler can change the selected frozen frame.
    bool E190WorkCoordinateSourceMatches(const NCParsedBlock& parsed, const NCBlock& b) noexcept
    {
        if (!E190WorkCoordinateSelection(parsed) || !E178CoordinatePreamble(parsed) || !E94Matches(parsed,b) ||
            !b.hasG || !IsNCWorkCoordinateCode(b.gCode) || b.gCount!=1 ||
            b.gCodes[0]!=b.gCode || b.mCount!=0 || b.gotoTarget!=-1 ||
            parsed.assignment.prefix || !parsed.assignment.indexExpression.empty() ||
            !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
            !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty()) return false;
        for (std::size_t i=1U;i<parsed.gExpressions.size();++i) if (!parsed.gExpressions[i].empty()) return false;
        for (const auto& expression:parsed.mExpressions) if (!expression.empty()) return false;
        for (unsigned i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i) if (b.gCodes[i]!=0) return false;
        for (int code:b.mCode) if (code!=-1) return false;
        for (char word='A';word<='Z';++word)
            if (!parsed.has(word) && (!parsed.expression(word).empty() ||
                !EDM109::SameDouble(b.val(word),0.0))) return false;
        return true;
    }
    // Read-only whole-program shape, including both literal parents and the
    // sole initial M00. Callers keep the cache generation as the source fence.
    template<class Cache,class BracketShape,class OrbitShape>
    bool E233ProgramShape(const Cache& cache,int& firstArmPC,std::array<unsigned,2U>& arms,
        BracketShape bracketShape,OrbitShape orbitShape) noexcept
    {
        firstArmPC=-1;arms.fill(0U);
        if(!cache.Size()||cache.Size()>128U)return false;
        int confirmation=-1;
        for(std::size_t pc=0U;pc<cache.Size();++pc)
        {
            const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line)return false;
            const auto& b=line->parsedBlock;
            if(!EDM77::Detail::Clean(b))return false;
            if(b.isEmpty)continue;
            if(E149M00Shape(b)){confirmation=static_cast<int>(pc);break;}
            bool mutation=false;
            bool recipe=EDM77::Detail::Operation(b,mutation);
            if(!recipe&&b.gCount==1&&EDM77::Detail::EqualLiteral(b.gExpressions[0U],39U)&&
                (EDM77::Detail::EqualLiteral(b.expression('P'),11U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),12U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),13U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),19U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),20U)))
            {
                NCParsedBlock syntax=b;syntax.paramExpressions['P'-'A']="5";
                recipe=EDM77::Detail::Operation(syntax,mutation);
            }
            if(!E94Preamble(b)&&!E178CoordinatePreamble(b)&&!recipe)return false;
        }
        if(confirmation<0||static_cast<std::size_t>(confirmation)+7U>=cache.Size())return false;
        const int first=confirmation+1;
        std::array<unsigned,2U> selected{};
        for(unsigned parent=0U;parent<2U;++parent)
        {
            const int pc=first+static_cast<int>(parent*3U);
            const auto* a=cache.TryGetLine(pc);const auto* o=cache.TryGetLine(pc+1);const auto* f=cache.TryGetLine(pc+2);
            NCBlock arm{},orbit{},finish{};EDM97::Tuning tuning{};EDM107::FlushProfile profile{};
            if(!a||!o||!f||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(a->parsedBlock,arm)||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(o->parsedBlock,orbit)||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(f->parsedBlock,finish)||
                !bracketShape(arm)||!orbitShape(orbit)||!bracketShape(finish)||!arm.has('P'))return false;
            const unsigned selector=static_cast<unsigned>(arm.val('P'));
            if(!E233CenterParentSequence(a->parsedBlock,selector)||
                !E108ProfileOnlyNPQRSIJK(a->parsedBlock,selector,tuning,profile)||
                !E94Matches(a->parsedBlock,arm)||!E180LiteralAngleSourceMatches(o->parsedBlock,orbit)||
                !E94Matches(f->parsedBlock,finish)||
                E181EffectiveOrbitMode(orbit)!=EDM173::ModeForArm(selector)||
                finish.val('P')!=static_cast<double>(EDM173::FinishForArm(selector)))return false;
            selected[parent]=selector;
        }
        if(selected[0U]==selected[1U])return false;
        const auto* last=cache.TryGetLine(first+6);NCBlock end{};
        if(!last||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(last->parsedBlock,end)||
            !E94Matches(last->parsedBlock,end)||end.isEmpty||end.isGoto||end.isBlockSkip||end.hasG||
            end.gCount||end.mCount!=1||end.mCode[0U]!=30||
            !EDM77::Detail::OnlyAddresses(last->parsedBlock,"N"))return false;
        for(std::size_t pc=static_cast<std::size_t>(first)+7U;pc<cache.Size();++pc)
        {const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line||!line->parsedBlock.isEmpty)return false;}
        firstArmPC=first;arms=selected;return true;
    }
    bool E234BracketLiteralShape(const NCBlock& b) noexcept
    {
        if(b.isEmpty||b.isGoto||b.isBlockSkip||!b.hasG||b.gCode!=180||b.gCount!=1||b.gCodes[0U]!=180||
            b.mCount||!b.has('P'))return false;
        const bool historyArm=b.val('P')==274.0||b.val('P')==294.0;
        const bool centerArm=b.val('P')==296.0||b.val('P')==298.0;
        const bool arm=historyArm||centerArm;
        if(!arm&&b.val('P')!=275.0&&b.val('P')!=295.0&&b.val('P')!=297.0&&b.val('P')!=299.0)return false;
        for(char word='A';word<='Z';++word)if(b.has(word)&&
            (!std::isfinite(b.val(word))||!std::strchr(arm?"NPQRSIJKL":"NP",word)))return false;
        return !arm||(b.has('L')&&(b.val('L')==241.0||b.val('L')==242.0||b.val('L')==243.0||b.val('L')==244.0||(b.val('L')==245.0&&b.val('P')!=298.0)||(b.val('L')==246.0&&b.val('P')!=296.0)||(b.val('L')==247.0&&b.val('P')!=298.0)||(b.val('L')==248.0&&b.val('P')!=296.0)||(b.val('L')==240.0&&(centerArm||b.val('P')==274.0))||(centerArm?b.val('L')==239.0:
            (b.val('L')==234.0||b.val('L')==235.0||b.val('L')==236.0||b.val('L')==237.0||b.val('L')==238.0)))&&b.has('Q')&&b.val('Q')==8.0&&
            b.has('R')&&b.val('R')==1.0&&b.has('S')&&b.val('S')==1.0&&b.has('I')&&b.val('I')==10.0&&
            b.has('J')&&b.val('J')==50.0&&b.has('K')&&b.val('K')==100.0);
    }
    // One genuine ordinary GAP row and one genuine 25-child G76 B2 parent.
    // L234 retains its exact G01 row; L235 separately seals one CW/CCW arc.
    template<class Cache>
    bool E234ProgramShape(const Cache& cache,int& firstArmPC,std::array<unsigned,2U>& arms,
        unsigned* program=nullptr,unsigned* ordinaryCode=nullptr) noexcept
    {
        firstArmPC=-1;arms.fill(0U);if(program)*program=0U;if(ordinaryCode)*ordinaryCode=0U;
        if(!cache.Size()||cache.Size()>128U)return false;
        int confirmation=-1;
        for(std::size_t pc=0U;pc<cache.Size();++pc)
        {
            const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line)return false;
            const auto& b=line->parsedBlock;if(!EDM77::Detail::Clean(b))return false;
            if(b.isEmpty)continue;
            if(E149M00Shape(b)){confirmation=static_cast<int>(pc);break;}
            const bool clearMirror=b.gCount==1&&b.mCount==0&&EDM77::Detail::OnlyAddresses(b,"N")&&
                EDM77::Detail::EqualLiteral(b.gExpressions[0U],150U);
            bool mutation=false;bool recipe=EDM77::Detail::Operation(b,mutation);
            if(!recipe&&b.gCount==1&&EDM77::Detail::EqualLiteral(b.gExpressions[0U],39U)&&
                (EDM77::Detail::EqualLiteral(b.expression('P'),11U)||EDM77::Detail::EqualLiteral(b.expression('P'),12U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),13U)||EDM77::Detail::EqualLiteral(b.expression('P'),19U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),20U)))
            {NCParsedBlock syntax=b;syntax.paramExpressions['P'-'A']="5";recipe=EDM77::Detail::Operation(syntax,mutation);}
            if(!E94Preamble(b)&&!clearMirror&&!recipe)return false;
        }
        if(confirmation<0||static_cast<std::size_t>(confirmation)+7U>=cache.Size())return false;
        const int first=confirmation+1;std::array<unsigned,2U> selected{};
        unsigned selectedProgram=0U,selectedOrdinaryCode=0U;
        for(unsigned parent=0U;parent<2U;++parent)
        {
            const int pc=first+static_cast<int>(parent*3U);
            const auto* a=cache.TryGetLine(pc);const auto* m=cache.TryGetLine(pc+1);const auto* f=cache.TryGetLine(pc+2);
            NCBlock arm{},motion{},finish{};EDM97::Tuning tuning{};EDM107::FlushProfile profile{};
            if(!a||!m||!f||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(a->parsedBlock,arm)||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(m->parsedBlock,motion)||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(f->parsedBlock,finish)||
                !E234BracketLiteralShape(arm)||!E234BracketLiteralShape(finish)||
                !E94Matches(a->parsedBlock,arm)||!E94Matches(m->parsedBlock,motion)||!E94Matches(f->parsedBlock,finish))return false;
            const unsigned selector=static_cast<unsigned>(arm.val('P'));
            if(!E234ArmSequence(a->parsedBlock,selector)||!E108ProfileOnlyNPQRSIJK(a->parsedBlock,selector,tuning,profile)||
                finish.val('P')!=static_cast<double>(selector+1U)||motion.isEmpty||motion.isGoto||motion.isBlockSkip||
                !motion.hasG||motion.gCount!=1||motion.mCount||!motion.has('F')||motion.val('F')!=0.12||
                !motion.has('B'))return false;
            const unsigned parentProgram=static_cast<unsigned>(arm.val('L'));
            if(parent?parentProgram!=selectedProgram:parentProgram!=234U&&parentProgram!=235U)return false;
            selectedProgram=parentProgram;
            if(selector==274U)
            {
                if(motion.val('B')!=1.0)return false;
                if(parentProgram==234U)
                {
                    if(motion.gCode!=1||motion.gCodes[0U]!=1||!motion.has('Z')||motion.val('Z')!=-0.0015)return false;
                    for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NZFB",word))return false;
                }
                else
                {
                    const bool cw=motion.gCode==2&&motion.gCodes[0U]==2;
                    const bool ccw=motion.gCode==3&&motion.gCodes[0U]==3;
                    const double x=cw?0.001:-0.001;
                    if((!cw&&!ccw)||!motion.has('X')||motion.val('X')!=x||!motion.has('Y')||motion.val('Y')!=0.001||
                        !motion.has('I')||motion.val('I')!=x||!motion.has('J')||motion.val('J')!=0.0)return false;
                    for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NXYIJFB",word))return false;
                    EDM114::Point increment{};increment[0U]=x;increment[1U]=0.001;EDM112::Curve curve{};
                    if(!EDM114::BuildBoundedXYArc(motion.gCode,increment,{{x,0.0}},motion.val('F'),curve))return false;
                }
                selectedOrdinaryCode=static_cast<unsigned>(motion.gCode);
            }
            else
            {
                if(motion.gCode!=76||motion.gCodes[0U]!=76||!motion.has('P')||motion.val('P')!=11.0||
                    !motion.has('X')||!motion.has('Y')||!motion.has('Z')||!motion.has('R')||motion.val('R')!=0.004||
                    !motion.has('K')||motion.val('K')!=45.0||motion.val('B')!=2.0||
                    (motion.val('X')!=0.001&&motion.val('X')!=-0.001)||motion.val('Y')!=motion.val('X')||motion.val('Z')!=motion.val('X'))return false;
                for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NPXYZRFBK",word))return false;
            }
            selected[parent]=selector;
        }
        if(selected[0U]==selected[1U])return false;
        const auto* last=cache.TryGetLine(first+6);NCBlock end{};
        if(!last||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(last->parsedBlock,end)||!E94Matches(last->parsedBlock,end)||
            end.isEmpty||end.isGoto||end.isBlockSkip||end.hasG||end.gCount||end.mCount!=1||end.mCode[0U]!=30||
            !EDM77::Detail::OnlyAddresses(last->parsedBlock,"N"))return false;
        for(std::size_t pc=static_cast<std::size_t>(first)+7U;pc<cache.Size();++pc)
        {const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line||!line->parsedBlock.isEmpty)return false;}
        firstArmPC=first;arms=selected;if(program)*program=selectedProgram;if(ordinaryCode)*ordinaryCode=selectedOrdinaryCode;return true;
    }
    unsigned E236ArmForKind(unsigned kind) noexcept
    {return kind==2U?294U:(kind==1U||kind==3U||kind==4U)?274U:0U;}
    unsigned E236GCodeForKind(unsigned kind) noexcept
    {return kind==1U?1U:kind==2U?76U:kind==3U?2U:kind==4U?3U:0U;}
    bool E236RecipeForKind(const EDM130::Recipe& recipe,unsigned kind) noexcept
    {
        return E236ArmForKind(kind)&&recipe.commissioningProfile==(kind==2U?EDM130::G76B2Profile175:0U)&&
            recipe.planRows==(kind==2U?25U:1U)&&recipe.planMovementMask==(kind==2U?7U:kind==1U?4U:3U)&&
            EDM130::ValidHistoryRecipeBinding(kind==2U?EDM173::HistoryFixture:EDM149::ProgramHistoryFixture,recipe)&&
            recipe.diagnosticControlSource==3U&&recipe.workTimeUs==500000ULL&&recipe.jumpHeightScaled==1000ULL&&
            recipe.jumpSpeedPercentScaled==100000000U&&!recipe.flushHoldTest&&!recipe.holdRetreatMode&&
            !recipe.deepFlushCycleInterval&&recipe.deepFlushHeightMultiplierScaled==1000000ULL;
    }
    bool E237NativeKind(unsigned kind) noexcept
    {return kind==2U||kind==6U||kind==7U;}
    unsigned E237ArmForKind(unsigned kind) noexcept
    {return kind==2U?294U:kind==5U?274U:kind==6U?296U:kind==7U?298U:0U;}
    unsigned E237RowsForKind(unsigned kind) noexcept
    {return E237NativeKind(kind)?1U:kind==5U?4U:0U;}
    bool E237RecipeForKind(const EDM130::Recipe& recipe,unsigned kind) noexcept
    {
        if(kind==6U||kind==7U)
            return EDM130::IsG76CenterSequence239(recipe)&&
                recipe.commissioningProfile==(kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177)&&
                recipe.planRows==25U&&recipe.planMovementMask==7U&&
                EDM130::ValidHistoryRecipeBinding(EDM173::HistoryFixture,recipe)&&
                recipe.diagnosticControlSource==3U&&recipe.workTimeUs==500000ULL&&recipe.jumpHeightScaled==200000ULL&&
                recipe.jumpSpeedPercentScaled==100000000U&&!recipe.holdRetreatMode&&
                !recipe.deepFlushCycleInterval&&recipe.deepFlushHeightMultiplierScaled==1000000ULL;
        return E237ArmForKind(kind)&&recipe.commissioningProfile==(kind==2U?EDM130::G76B2Profile175:0U)&&
            recipe.planRows==(kind==2U?25U:4U)&&recipe.planMovementMask==7U&&
            EDM130::ValidHistoryRecipeBinding(kind==2U?EDM173::HistoryFixture:EDM149::ProgramHistoryFixture,recipe)&&
            recipe.diagnosticControlSource==3U&&recipe.workTimeUs==500000ULL&&recipe.jumpHeightScaled==1000ULL&&
            recipe.jumpSpeedPercentScaled==100000000U&&!recipe.flushHoldTest&&!recipe.holdRetreatMode&&
            !recipe.deepFlushCycleInterval&&recipe.deepFlushHeightMultiplierScaled==1000000ULL;
    }
    unsigned E237RequiredBridgeMask(unsigned program,unsigned firstKind=0U) noexcept
    {return program==248U?(firstKind==5U?4U:firstKind==7U?1U:8U):program==247U?(firstKind==5U?4U:firstKind==6U?1U:8U):program==246U?(firstKind==5U?4U:firstKind==7U?1U:8U):program==245U?(firstKind==5U?4U:firstKind==6U?1U:8U):program==240U||program==241U?7U:program==242U?2U:(program==243U||program==244U)?5U:0U;}
    bool E237BridgeRequired(unsigned program,unsigned nextParent,unsigned firstKind=0U) noexcept
    {const unsigned mask=E237RequiredBridgeMask(program,firstKind);return mask<=7U&&nextParent>0U&&nextParent<4U&&(mask&(1U<<(nextParent-1U)))!=0U;}
    unsigned E237ArmStride(unsigned program,unsigned kind,unsigned nextParent=1U,unsigned firstKind=0U) noexcept
    {return E237RequiredBridgeMask(program,firstKind)>7U?0U:E237RowsForKind(kind)+2U+(E237BridgeRequired(program,nextParent,firstKind)?1U:0U);}
    bool E240RecipeBridgeShape(const NCParsedBlock& b,unsigned kind,unsigned program=240U) noexcept
    {
        return (program==240U||program==241U||program==242U||program==243U||program==244U||program==245U||program==246U||program==247U||program==248U)&&(program!=245U||kind!=7U)&&(program!=246U||kind!=6U)&&(program!=247U||kind!=7U)&&(program!=248U||kind!=6U)&&(kind==5U||kind==6U||kind==7U||((program==241U||program==242U||program==243U||program==244U||program==245U||program==246U||program==247U||program==248U)&&kind==2U))&&EDM77::Detail::Clean(b)&&!b.isEmpty&&
            b.gCount==1&&b.mCount==0&&EDM77::Detail::EqualLiteral(b.gExpressions[0U],39U)&&
            EDM77::Detail::OnlyAddresses(b,"NPQR")&&b.has('P')&&b.has('Q')&&b.has('R')&&
            EDM77::Detail::EqualLiteral(b.expression('P'),6U)&&EDM77::Detail::EqualLiteral(b.expression('Q'),3U)&&
            ((kind==2U||kind==5U)?b.expression('R')=="0.001":b.expression('R')=="0.2");
    }
    bool E237RecipeForProgramKind(const EDM130::Recipe& recipe,unsigned kind,unsigned program) noexcept
    {
        if(program!=240U&&program!=241U&&program!=242U&&program!=243U&&program!=244U&&program!=245U&&program!=246U&&program!=247U&&program!=248U)return E237RecipeForKind(recipe,kind);
        if(kind==5U||((program==241U||program==242U||program==243U||program==244U||program==245U||program==246U||program==247U||program==248U)&&kind==2U))return recipe.heightFieldId==6U&&E237RecipeForKind(recipe,kind);
        return (kind==6U||kind==7U)&&(program!=245U||kind==6U)&&(program!=246U||kind==7U)&&(program!=247U||kind==6U)&&(program!=248U||kind==7U)&&(program==240U?EDM130::IsG76CenterSequence240(recipe):program==241U?EDM130::IsG76CenterSequence241(recipe):program==242U?EDM130::IsG76CenterSequence242(recipe):program==243U?EDM130::IsG76CenterSequence243(recipe):program==244U?EDM130::IsG76CenterSequence244(recipe):program==245U?EDM130::IsG76CenterSequence245(recipe):program==246U?EDM130::IsG76CenterSequence246(recipe):program==247U?EDM130::IsG76CenterSequence247(recipe):EDM130::IsG76CenterSequence248(recipe))&&recipe.heightFieldId==6U&&
            recipe.commissioningProfile==(kind==6U?EDM130::G76B3Profile176:EDM130::G76B4Profile177)&&
            recipe.planRows==25U&&recipe.planMovementMask==7U&&
            EDM130::ValidHistoryRecipeBinding(EDM173::HistoryFixture,recipe)&&recipe.diagnosticControlSource==3U&&
            recipe.workTimeUs==500000ULL&&recipe.jumpHeightScaled==200000ULL&&
            recipe.jumpSpeedPercentScaled==100000000U&&!recipe.holdRetreatMode&&
            !recipe.deepFlushCycleInterval&&recipe.deepFlushHeightMultiplierScaled==1000000ULL;
    }
    bool E237SameParentRecipe(const EDM130::Recipe& first,const EDM130::Recipe& current,unsigned program) noexcept
    {
        auto expected=first;expected.commissioningProfile=current.commissioningProfile;
        expected.planRows=current.planRows;expected.planMovementMask=current.planMovementMask;
        if(program==240U||program==241U||program==242U||program==243U||program==244U||program==245U||program==246U||program==247U||program==248U)
        {expected.generation=current.generation;expected.jumpHeightScaled=current.jumpHeightScaled;expected.flushHoldTest=current.flushHoldTest;}
        return EDM130::SameRecipe(expected,current);
    }
    template<class State>
    bool E240BridgeHistory(const State& q) noexcept
    {
        const unsigned required=E237RequiredBridgeMask(q.program,q.kinds[0U]);
        if(required>7U)return false;
        if(!required)return q.bridgeMask==0U;
        if(q.parentCount!=4U||q.parentIndex>=4U||q.bridgeMask>7U)return false;
        const unsigned prefix=required&((1U<<q.parentIndex)-1U);
        const unsigned previous=q.parentIndex?required&((1U<<(q.parentIndex-1U))-1U):0U;
        if((q.bridgeMask!=prefix&&q.bridgeMask!=previous)||
            ((q.stageRecipeBound||q.stageStarted)&&q.bridgeMask!=prefix))return false;
        unsigned count=0U,total=0U;
        for(unsigned i=0U;i<3U;++i){count+=(q.bridgeMask>>i)&1U;total+=(required>>i)&1U;}
        if(!q.firstRecipeBound)return !q.parentIndex&&!q.bridgeMask;
        if(!E237RecipeForProgramKind(q.firstRecipe,q.kinds[0U],q.program)||!q.firstRecipe.generation||
            q.firstRecipe.generation>(std::numeric_limits<std::uint64_t>::max)()-total||
            q.recipeGeneration!=q.firstRecipe.generation+count)return false;
        unsigned ordinal=0U;int previousBridge=-1;
        for(unsigned i=0U;i<3U;++i)
        {
            const auto& r=q.bridges[i];
            if(!(q.bridgeMask&(1U<<i)))
            {if(r.committed||(r.prepared&&(!(required&(1U<<i))||!q.awaitNext||q.stageRecipeBound||i+1U!=q.parentIndex)))return false;continue;}
            auto key=r.beforeKey;key.generation=r.afterGeneration;
            if(!r.committed||r.prepared||r.pc!=q.armPC[i+1U]-1||r.line<=0||!r.dispatch||
                r.dispatch<=r.finishDispatch||!r.controlGeneration||!r.releaseTick||!r.previousSession||
                !r.finishDispatch||r.run!=q.run||r.cache!=q.cache||r.epoch==MOTION_EXECUTION_EPOCH_INVALID||
                !r.lease.Matches(q.lease)||r.beforeGeneration!=q.firstRecipe.generation+ordinal||
                r.afterGeneration!=r.beforeGeneration+1ULL||r.beforeKey.generation!=r.beforeGeneration||
                !EDMRecipeControl::IsKeyValid(r.beforeKey)||!EDMRecipeControl::SameKey(key,r.afterKey)||
                r.previewRecipe.generation!=r.afterGeneration||
                !E237RecipeForProgramKind(r.previewRecipe,q.kinds[i+1U],q.program)||
                !E237SameParentRecipe(q.firstRecipe,r.previewRecipe,q.program))return false;
            if(previousBridge>=0&&(r.dispatch<=q.bridges[previousBridge].dispatch||r.releaseTick<=q.bridges[previousBridge].releaseTick||
                r.previousSession<=q.bridges[previousBridge].previousSession||r.finishDispatch<=q.bridges[previousBridge].finishDispatch))return false;
            if(i+1U<q.parentIndex&&
                (r.releaseTick>q.previousReleaseFloorTick||r.releaseTick>=q.releaseTick||r.previousSession>=q.previousSession||
                    r.finishDispatch>=q.previousFinishDispatch||r.dispatch>=q.previousFinishDispatch))return false;
            previousBridge=static_cast<int>(i);++ordinal;
        }
        if(q.parentIndex&&E237BridgeRequired(q.program,q.parentIndex,q.kinds[0U])&&q.bridgeMask==prefix)
        {const auto& r=q.bridges[q.parentIndex-1U];if(r.releaseTick!=q.releaseTick||r.previousSession!=q.previousSession||
            r.finishDispatch!=q.previousFinishDispatch)return false;}
        return !q.stageRecipeBound||q.currentRecipe.generation==q.recipeGeneration;
    }
    // EDM237 seals two history parents, EDM238 four history parents, and
    // EDM239 four native B3/B4 parents. Each family has its own exact shape.
    // Ordinary rows share history/E5 only within their fresh parent session.
    template<class Cache>
    bool E237HistoryProgramShape(const Cache& cache,int& firstArmPC,std::array<unsigned,4U>& kinds,
        unsigned& program,unsigned& parentCount) noexcept
    {
        firstArmPC=-1;kinds.fill(0U);program=parentCount=0U;
        if(!cache.Size()||cache.Size()>128U)return false;
        int confirmation=-1;
        for(std::size_t pc=0U;pc<cache.Size();++pc)
        {
            const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line)return false;
            const auto& b=line->parsedBlock;if(!EDM77::Detail::Clean(b))return false;
            if(b.isEmpty)continue;
            if(E149M00Shape(b)){confirmation=static_cast<int>(pc);break;}
            const bool clearMirror=b.gCount==1&&b.mCount==0&&EDM77::Detail::OnlyAddresses(b,"N")&&
                EDM77::Detail::EqualLiteral(b.gExpressions[0U],150U);
            bool mutation=false;bool recipe=EDM77::Detail::Operation(b,mutation);
            if(!recipe&&b.gCount==1&&EDM77::Detail::EqualLiteral(b.gExpressions[0U],39U)&&
                (EDM77::Detail::EqualLiteral(b.expression('P'),11U)||EDM77::Detail::EqualLiteral(b.expression('P'),12U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),13U)||EDM77::Detail::EqualLiteral(b.expression('P'),19U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),20U)))
            {NCParsedBlock syntax=b;syntax.paramExpressions['P'-'A']="5";recipe=EDM77::Detail::Operation(syntax,mutation);}
            if(!E94Preamble(b)&&!clearMirror&&!recipe)return false;
        }
        if(confirmation<0||static_cast<std::size_t>(confirmation)+10U>=cache.Size())return false;
        const int first=confirmation+1;const auto* firstLine=cache.TryGetLine(first);
        if(!firstLine)return false;
        const auto& firstBlock=firstLine->parsedBlock;
        const unsigned selectedProgram=EDM77::Detail::EqualLiteral(firstBlock.expression('L'),237U)?237U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),238U)?238U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),239U)?239U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),240U)?240U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),241U)?241U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),242U)?242U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),243U)?243U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),244U)?244U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),245U)?245U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),246U)?246U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),247U)?247U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('L'),248U)?248U:0U;
        if(!selectedProgram)return false;
        const bool centerSequence=selectedProgram==239U;const bool mixedCenter=selectedProgram==240U||selectedProgram==241U||selectedProgram==242U||selectedProgram==243U||selectedProgram==244U||selectedProgram==245U||selectedProgram==246U||selectedProgram==247U||selectedProgram==248U;
        const unsigned count=selectedProgram==237U?2U:4U;
        const unsigned firstKind=selectedProgram==248U?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('P'),298U)?7U:0U):selectedProgram==247U?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('P'),296U)?6U:0U):selectedProgram==246U?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('P'),298U)?7U:0U):selectedProgram==245U?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('P'),296U)?6U:0U):(selectedProgram==243U||selectedProgram==244U)?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:
            EDM77::Detail::EqualLiteral(firstBlock.expression('P'),294U)?2U:0U):mixedCenter?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:7U):centerSequence?(EDM77::Detail::EqualLiteral(firstBlock.expression('P'),296U)?6U:7U):
            (EDM77::Detail::EqualLiteral(firstBlock.expression('P'),274U)?5U:2U);
        const unsigned otherKind=centerSequence?(firstKind==6U?7U:6U):(firstKind==5U?2U:5U);
        const double expectedOrbitSign=(selectedProgram==245U||selectedProgram==246U||selectedProgram==247U||selectedProgram==248U)?(firstKind==5U?0.001:-0.001):(firstKind==5U||firstKind==6U?0.001:-0.001);
        const std::array<unsigned,4U> forward{{5U,6U,5U,7U}},reverse{{7U,5U,6U,5U}};
        const std::array<unsigned,4U> allForward{{5U,6U,2U,7U}},allReverse{{7U,2U,6U,5U}};
        const std::array<unsigned,4U> adjacentForward{{5U,2U,6U,7U}},adjacentReverse{{7U,6U,2U,5U}};
        const std::array<unsigned,4U> twoBridgeForward{{5U,6U,7U,2U}},twoBridgeReverse{{2U,7U,6U,5U}};
        const std::array<unsigned,4U> repeatedForward{{5U,6U,6U,2U}},repeatedReverse{{2U,7U,7U,5U}};
        const std::array<unsigned,4U> ordinaryForward{{5U,2U,5U,6U}},ordinaryReverse{{6U,5U,2U,5U}};
        const std::array<unsigned,4U> ordinaryB4Forward{{5U,2U,5U,7U}},ordinaryB4Reverse{{7U,5U,2U,5U}};
        const std::array<unsigned,4U> repeatedB2Forward{{5U,2U,2U,6U}},repeatedB2Reverse{{6U,2U,2U,5U}};
        const std::array<unsigned,4U> repeatedB2B4Forward{{5U,2U,2U,7U}},repeatedB2B4Reverse{{7U,2U,2U,5U}};
        const auto& mixedKinds=selectedProgram==248U?(firstKind==5U?repeatedB2B4Forward:repeatedB2B4Reverse):selectedProgram==247U?(firstKind==5U?repeatedB2Forward:repeatedB2Reverse):selectedProgram==246U?(firstKind==5U?ordinaryB4Forward:ordinaryB4Reverse):selectedProgram==245U?(firstKind==5U?ordinaryForward:ordinaryReverse):selectedProgram==244U?(firstKind==5U?repeatedForward:repeatedReverse):selectedProgram==243U?(firstKind==5U?twoBridgeForward:twoBridgeReverse):selectedProgram==242U?(firstKind==5U?adjacentForward:adjacentReverse):selectedProgram==241U?(firstKind==5U?allForward:allReverse):(firstKind==5U?forward:reverse);
        if(mixedCenter)
        {
            unsigned heights=0U;
            for(int before=0;before<confirmation;++before)
            {
                const auto* line=cache.TryGetLine(before);const auto& b=line->parsedBlock;
                if(b.gCount==1&&EDM77::Detail::EqualLiteral(b.gExpressions[0U],39U)&&
                    EDM77::Detail::EqualLiteral(b.expression('P'),6U))
                {if(++heights!=1U||!E240RecipeBridgeShape(b,firstKind,selectedProgram))return false;}
            }
            if(heights!=1U)return false;
        }
        int pc=first;std::array<unsigned,4U> selected{};
        for(unsigned parent=0U;parent<count;++parent)
        {
            if(E237BridgeRequired(selectedProgram,parent,firstKind))
            {const auto* bridge=cache.TryGetLine(pc);if(!bridge||!E240RecipeBridgeShape(bridge->parsedBlock,mixedKinds[parent],selectedProgram))return false;++pc;}
            const auto* a=cache.TryGetLine(pc);NCBlock arm{};EDM97::Tuning tuning{};EDM107::FlushProfile profile{};
            if(!a||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(a->parsedBlock,arm)||
                !E234BracketLiteralShape(arm)||!E94Matches(a->parsedBlock,arm))return false;
            const unsigned selector=static_cast<unsigned>(arm.val('P'));
            if(!E237ArmSequence(a->parsedBlock,selector)||
                !EDM77::Detail::EqualLiteral(a->parsedBlock.expression('L'),selectedProgram)||
                !E108ProfileOnlyNPQRSIJK(a->parsedBlock,selector,tuning,profile))return false;
            const unsigned kind=selector==274U?5U:selector==294U?2U:selector==296U?6U:selector==298U?7U:0U;
            if(kind!=(mixedCenter?mixedKinds[parent]:(parent%2U?otherKind:firstKind)))return false;
            const unsigned rows=E237RowsForKind(kind);if(!rows)return false;
            for(unsigned row=0U;row<rows;++row)
            {
                const auto* line=cache.TryGetLine(pc+1+static_cast<int>(row));NCBlock motion{};
                if(!line||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(line->parsedBlock,motion)||
                    !E94Matches(line->parsedBlock,motion)||motion.isEmpty||motion.isGoto||motion.isBlockSkip||
                    !motion.hasG||motion.gCount!=1||motion.gCodes[0U]!=motion.gCode||motion.mCount||
                    !motion.has('F')||motion.val('F')!=0.12)return false;
                if(kind==5U)
                {
                    const int codes[4]={1,2,3,1};
                    if(motion.gCode!=codes[row]||motion.has('B')!=(row!=3U)||
                        (row!=3U&&motion.val('B')!=static_cast<double>(row)))return false;
                    if(row==0U||row==3U)
                    {
                        if(!motion.has('Z')||motion.val('Z')!=-0.0015)return false;
                        for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr(row==3U?"NZF":"NZFB",word))return false;
                    }
                    else
                    {
                        const double x=row==1U?0.001:-0.001;
                        if(!motion.has('X')||motion.val('X')!=x||!motion.has('Y')||motion.val('Y')!=0.001||
                            !motion.has('I')||motion.val('I')!=x||!motion.has('J')||motion.val('J')!=0.0)return false;
                        for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NXYIJFB",word))return false;
                        EDM114::Point increment{};increment[0U]=x;increment[1U]=0.001;EDM112::Curve curve{};
                        if(!EDM114::BuildBoundedXYArc(motion.gCode,increment,{{x,0.0}},motion.val('F'),curve))return false;
                    }
                }
                else
                {
                    if(motion.gCode!=76||!motion.has('P')||motion.val('P')!=11.0||
                        !motion.has('X')||!motion.has('Y')||!motion.has('Z')||!motion.has('R')||motion.val('R')!=0.004||
                        !motion.has('K')||motion.val('K')!=45.0||!motion.has('B')||
                        motion.val('B')!=static_cast<double>(kind==6U?3U:kind==7U?4U:2U)||
                        motion.val('X')!=expectedOrbitSign||motion.val('Y')!=motion.val('X')||motion.val('Z')!=motion.val('X'))return false;
                    for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NPXYZRFBK",word))return false;
                }
            }
            const auto* f=cache.TryGetLine(pc+1+static_cast<int>(rows));NCBlock finish{};
            if(!f||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(f->parsedBlock,finish)||
                !E234BracketLiteralShape(finish)||!E94Matches(f->parsedBlock,finish)||finish.val('P')!=selector+1.0)return false;
            selected[parent]=kind;pc+=static_cast<int>(rows)+2;
        }
        if(pc!=first+static_cast<int>(mixedCenter?((selectedProgram==247U||selectedProgram==248U)?16U:(selectedProgram==245U||selectedProgram==246U)?19U:(selectedProgram==243U||selectedProgram==244U)?17U:selectedProgram==242U?16U:selectedProgram==241U?18U:21U):centerSequence?count*3U:count/2U*9U))return false;
        const auto* last=cache.TryGetLine(pc);NCBlock end{};
        if(!last||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(last->parsedBlock,end)||!E94Matches(last->parsedBlock,end)||
            end.isEmpty||end.isGoto||end.isBlockSkip||end.hasG||end.gCount||end.mCount!=1||end.mCode[0U]!=30||
            !EDM77::Detail::OnlyAddresses(last->parsedBlock,"N"))return false;
        for(std::size_t after=static_cast<std::size_t>(pc)+1U;after<cache.Size();++after)
        {const auto* line=cache.TryGetLine(static_cast<int>(after));if(!line||!line->parsedBlock.isEmpty)return false;}
        firstArmPC=first;kinds=selected;program=selectedProgram;parentCount=count;return true;
    }
    // Existing preamble queries need only the first pair, but validate the
    // whole source through the same bounded parser before using either kind.
    template<class Cache>
    bool E237ProgramShape(const Cache& cache,int& firstArmPC,std::array<unsigned,2U>& kinds) noexcept
    {
        std::array<unsigned,4U> selected{};unsigned program=0U,count=0U;kinds.fill(0U);
        if(!E237HistoryProgramShape(cache,firstArmPC,selected,program,count))return false;
        kinds[0U]=selected[0U];kinds[1U]=selected[1U];return true;
    }
    // Four immutable stopped sessions. This seals only the authored forward
    // order and its exact reverse; it creates no shared-history motion lane.
    template<class Cache>
    bool E236ProgramShape(const Cache& cache,int& firstArmPC,std::array<unsigned,4U>& kinds) noexcept
    {
        firstArmPC=-1;kinds.fill(0U);
        if(!cache.Size()||cache.Size()>128U)return false;
        int confirmation=-1;
        for(std::size_t pc=0U;pc<cache.Size();++pc)
        {
            const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line)return false;
            const auto& b=line->parsedBlock;if(!EDM77::Detail::Clean(b))return false;
            if(b.isEmpty)continue;
            if(E149M00Shape(b)){confirmation=static_cast<int>(pc);break;}
            const bool clearMirror=b.gCount==1&&b.mCount==0&&EDM77::Detail::OnlyAddresses(b,"N")&&
                EDM77::Detail::EqualLiteral(b.gExpressions[0U],150U);
            bool mutation=false;bool recipe=EDM77::Detail::Operation(b,mutation);
            if(!recipe&&b.gCount==1&&EDM77::Detail::EqualLiteral(b.gExpressions[0U],39U)&&
                (EDM77::Detail::EqualLiteral(b.expression('P'),11U)||EDM77::Detail::EqualLiteral(b.expression('P'),12U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),13U)||EDM77::Detail::EqualLiteral(b.expression('P'),19U)||
                 EDM77::Detail::EqualLiteral(b.expression('P'),20U)))
            {NCParsedBlock syntax=b;syntax.paramExpressions['P'-'A']="5";recipe=EDM77::Detail::Operation(syntax,mutation);}
            if(!E94Preamble(b)&&!clearMirror&&!recipe)return false;
        }
        if(confirmation<0||static_cast<std::size_t>(confirmation)+13U>=cache.Size())return false;
        const int first=confirmation+1;std::array<unsigned,4U> selected{};double orbitSign=0.0;
        for(unsigned parent=0U;parent<4U;++parent)
        {
            const int pc=first+static_cast<int>(parent*3U);
            const auto* a=cache.TryGetLine(pc);const auto* m=cache.TryGetLine(pc+1);const auto* f=cache.TryGetLine(pc+2);
            NCBlock arm{},motion{},finish{};EDM97::Tuning tuning{};EDM107::FlushProfile profile{};
            if(!a||!m||!f||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(a->parsedBlock,arm)||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(m->parsedBlock,motion)||
                !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(f->parsedBlock,finish)||
                !E234BracketLiteralShape(arm)||!E234BracketLiteralShape(finish)||!arm.has('L')||arm.val('L')!=236.0||
                !E94Matches(a->parsedBlock,arm)||!E94Matches(m->parsedBlock,motion)||!E94Matches(f->parsedBlock,finish))return false;
            const unsigned selector=static_cast<unsigned>(arm.val('P'));
            if(!E234ArmSequence(a->parsedBlock,selector)||!E108ProfileOnlyNPQRSIJK(a->parsedBlock,selector,tuning,profile)||
                finish.val('P')!=static_cast<double>(selector+1U)||motion.isEmpty||motion.isGoto||motion.isBlockSkip||
                !motion.hasG||motion.gCount!=1||motion.gCodes[0U]!=motion.gCode||motion.mCount||
                !motion.has('F')||motion.val('F')!=0.12||!motion.has('B'))return false;
            unsigned kind=0U;
            if(selector==274U)
            {
                if(motion.val('B')!=1.0)return false;
                if(motion.gCode==1)
                {
                    if(!motion.has('Z')||motion.val('Z')!=-0.0015)return false;
                    for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NZFB",word))return false;
                    kind=1U;
                }
                else
                {
                    const bool cw=motion.gCode==2;const double x=cw?0.001:-0.001;
                    if((!cw&&motion.gCode!=3)||!motion.has('X')||motion.val('X')!=x||!motion.has('Y')||motion.val('Y')!=0.001||
                        !motion.has('I')||motion.val('I')!=x||!motion.has('J')||motion.val('J')!=0.0)return false;
                    for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NXYIJFB",word))return false;
                    EDM114::Point increment{};increment[0U]=x;increment[1U]=0.001;EDM112::Curve curve{};
                    if(!EDM114::BuildBoundedXYArc(motion.gCode,increment,{{x,0.0}},motion.val('F'),curve))return false;
                    kind=cw?3U:4U;
                }
            }
            else
            {
                if(selector!=294U||motion.gCode!=76||!motion.has('P')||motion.val('P')!=11.0||
                    !motion.has('X')||!motion.has('Y')||!motion.has('Z')||!motion.has('R')||motion.val('R')!=0.004||
                    !motion.has('K')||motion.val('K')!=45.0||motion.val('B')!=2.0||
                    (motion.val('X')!=0.001&&motion.val('X')!=-0.001)||motion.val('Y')!=motion.val('X')||motion.val('Z')!=motion.val('X'))return false;
                for(char word='A';word<='Z';++word)if(motion.has(word)&&!std::strchr("NPXYZRFBK",word))return false;
                kind=2U;orbitSign=motion.val('X');
            }
            selected[parent]=kind;
        }
        const std::array<unsigned,4U> forward{{1U,3U,4U,2U}},reverse{{2U,4U,3U,1U}};
        if((selected!=forward&&selected!=reverse)||orbitSign!=(selected==forward?0.001:-0.001))return false;
        const auto* last=cache.TryGetLine(first+12);NCBlock end{};
        if(!last||!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(last->parsedBlock,end)||!E94Matches(last->parsedBlock,end)||
            end.isEmpty||end.isGoto||end.isBlockSkip||end.hasG||end.gCount||end.mCount!=1||end.mCode[0U]!=30||
            !EDM77::Detail::OnlyAddresses(last->parsedBlock,"N"))return false;
        for(std::size_t pc=static_cast<std::size_t>(first)+13U;pc<cache.Size();++pc)
        {const auto* line=cache.TryGetLine(static_cast<int>(pc));if(!line||!line->parsedBlock.isEmpty)return false;}
        firstArmPC=first;kinds=selected;return true;
    }

    bool E132Near(double a, double b, double scale = 1.0) noexcept
    {
        return std::isfinite(a) && std::isfinite(b) && std::isfinite(scale) &&
            std::fabs(a - b) <= EDM126::Roundoff((std::max)(std::fabs(scale), (std::max)(std::fabs(a), std::fabs(b))));
    }

    // Independent NC receipt check. Reconstruct at most sixteen admitted rows
    // in relative coordinates, reusing one curve; this never executes a row,
    // creates retained history, or grants mapping/output authority.
    template<class State, class Cache>
    bool E132GeometryProof(const State& s, const Cache& cache, const EDM128::Feedback& e,
        unsigned completed, const std::array<EDM141::Row, EDM141::RowCapacity>* frozenRows = nullptr) noexcept
    {
        if (!completed || completed > EDM114::RequiredRows || !e.anchorCaptured || !e.targetCaptured || !e.topCaptured ||
            e.anchor.cursorSource.ordinal >= completed) return false;
        EDM112::Point scale{}, origin{}, start{}, anchorRelative{}, targetRelative{};
        for (unsigned axis = 0U; axis < 6U; ++axis)
        {
            scale[axis] = s.feedback.config.axis[axis].pulsePerUnit;
            origin[axis] = s.feedback.config.axis[axis].commandAnchor;
            if (!EDM126::Positive(scale[axis]) || !std::isfinite(origin[axis]) ||
                !EDM109::SameDouble(e.heightPulsePerNative[axis], scale[axis])) return false;
        }
        // Absolute-pulse subtraction in RT can change represented native
        // lengths by roundoff. Bound that reconstruction budget over sixteen
        // rows; it remains well below one fixed-point height unit (1e-6 mm).
        double nativeBudget = 0.0;
        for (unsigned axis = 0U; axis < 3U; ++axis)
            nativeBudget = std::hypot(nativeBudget, 2.0 * EDM126::Roundoff(origin[axis]) / scale[axis]);
        nativeBudget *= static_cast<double>(completed + 1U);
        if (!std::isfinite(nativeBudget) || nativeBudget > 1.0e-8) return false;
        const auto metricNear = [nativeBudget](double a, double b) noexcept
        { return std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= nativeBudget + EDM126::Roundoff((std::max)(std::fabs(a), std::fabs(b))); };
        const auto pulseNear = [nativeBudget](double a, double b, double originPulse, double pulsePerNative) noexcept
        { return std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= nativeBudget * pulsePerNative +
            EDM126::Roundoff((std::max)(std::fabs(originPulse), (std::max)(std::fabs(a), std::fabs(b)))); };
        EDM112::Curve curve{};
        double begin = 0.0;
        bool anchorFound = false, targetFound = e.policy.mode != EDM116::Mode::B2;
        for (unsigned row = 0U; row < completed; ++row)
        {
            const auto* line = cache.TryGetLine(s.pc[row]);
            if (!line) return false;
            NCParsedBlock literal{};
            if (frozenRows && (!EDM141::MakeLiteral(line->parsedBlock, (*frozenRows)[row], literal) ||
                (*frozenRows)[row].pc != s.pc[row])) return false;
            const auto& b = frozenRows ? literal : line->parsedBlock;
            if (!EDM77::Detail::Clean(b)) return false;
            double g = 0.0;
            if (b.gCount != 1 || !E94Number(b.gExpressions[0], g) || (g != 1.0 && g != 2.0 && g != 3.0) ||
                (EDM116::IsAuthoredOrderFixture(s.gapModeFixture) && g != s.authoredGCode[row])) return false;
            EDM112::Point end{};
            for (unsigned axis = 0U; axis < 6U; ++axis)
            {
                if (!std::isfinite(s.increment[row][axis])) return false;
                end[axis] = start[axis] + s.increment[row][axis] * scale[axis];
            }
            if (g == 1.0)
            {
                EDM109::Line straight{};
                if (!EDM109::BuildLine(start, end, scale, straight) || !EDM112::BuildLine(straight, curve)) return false;
            }
            else
            {
                std::array<double, 2U> center{};
                if (!E155ArcCenter(b, center, s.gapModeFixture) ||
                    !EDM112::BuildArc(start, end, scale, 17U, center, g == 2.0 ? -1 : 1, false, curve)) return false;
            }
            if (row == e.anchor.cursorSource.ordinal)
            {
                if (!std::isfinite(e.anchor.localMm) || e.anchor.localMm < 0.0 || (e.anchor.localMm > curve.lengthMm && !metricNear(e.anchor.localMm, curve.lengthMm)) ||
                    !metricNear(e.anchor.lengthMm, curve.lengthMm) || !metricNear(e.anchor.stationMm, begin + e.anchor.localMm)) return false;
                EDM112::Point derivative{};
                const double unit = e.anchor.localMm >= curve.lengthMm ? 1.0 : e.anchor.localMm / curve.lengthMm;
                if (!EDM112::EvaluateUnit(curve, unit, anchorRelative, &derivative)) return false;
                for (unsigned axis = 0U; axis < 6U; ++axis)
                    if (!pulseNear(e.anchor.positionPulse[axis], origin[axis] + anchorRelative[axis], origin[axis], scale[axis])) return false;
                if (e.policy.mode == EDM116::Mode::B0)
                {
                    double norm = 0.0;
                    for (unsigned axis = 0U; axis < 3U; ++axis) norm = std::hypot(norm, derivative[axis] / scale[axis]);
                    if (!EDM126::Positive(norm)) return false;
                    for (unsigned axis = 0U; axis < 6U; ++axis)
                    {
                        const double want = derivative[axis] / norm;
                        const double tangentBudget = 4.0 * (2.0 * EDM126::Roundoff(origin[axis]) *
                            static_cast<double>(completed + 1U) / curve.lengthMm +
                            std::fabs(want) * nativeBudget / curve.lengthMm) +
                            EDM126::Roundoff((std::max)(scale[axis], std::fabs(want)));
                        if (!std::isfinite(tangentBudget) || !std::isfinite(e.tangentPulsePerMm[axis]) ||
                            std::fabs(e.tangentPulsePerMm[axis] - want) > tangentBudget) return false;
                    }
                }
                anchorFound = true;
            }
            if (e.policy.mode == EDM116::Mode::B2 && row == e.targetSource.ordinal)
            {
                if (!std::isfinite(e.targetLocalMm) || e.targetLocalMm < 0.0 || (e.targetLocalMm > curve.lengthMm && !metricNear(e.targetLocalMm, curve.lengthMm)) ||
                    !metricNear(e.targetStationMm, begin + e.targetLocalMm) ||
                    (row + 1U < completed && e.targetLocalMm == curve.lengthMm)) return false;
                const double unit = e.targetLocalMm >= curve.lengthMm ? 1.0 : e.targetLocalMm / curve.lengthMm;
                if (!EDM112::EvaluateUnit(curve, unit, targetRelative)) return false;
                targetFound = true;
            }
            begin += curve.lengthMm;
            start = end;
        }
        if (!anchorFound || !targetFound) return false;
        if (e.policy.mode == EDM116::Mode::B2)
            for (unsigned axis = 0U; axis < 6U; ++axis)
                if (!pulseNear(e.topPositionPulse[axis], origin[axis] + targetRelative[axis], origin[axis], scale[axis]) ||
                    !pulseNear(e.topPositionPulse[axis] - e.anchor.positionPulse[axis],
                        targetRelative[axis] - anchorRelative[axis], origin[axis], scale[axis])) return false;
        return true;
    }

}

