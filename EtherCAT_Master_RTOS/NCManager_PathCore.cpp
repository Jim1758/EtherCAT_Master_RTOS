// NC-0.2L.2AS / Path Core Shadow Implementation Unit Isolation.
// These eight NCManager method definitions are moved verbatim from AR.
// The existing caller, member layout and observer order are unchanged.
// This is a translation-unit boundary, NOT a thread, queue or lifecycle
// boundary. Same NC thread, live owners and no reentry still required.
// No source snapshot, new runtime state or validation shortcut is added.
// Compile this file exactly once together with the matching NCManager.cpp.
#include "NCManager.h"
#include <cstdint>
#include <type_traits>
#include <cstring>
#include <limits>
#include <cmath>
#include <windows.h>
#include <rtapi.h>

// NC-0.2L.2AT / Split-Unit Link Pairing Guard.
// Link-only MSVC/COFF pairing of the two AS implementation units. Each unit
// contributes its own revisioned witness and requires the peer's witness.
// A partial AT/legacy-AS update must not silently link. detect_mismatch also
// rejects different TAGGED contract revisions; it does not hash source bytes.
// These non-exported empty symbols are never called by NC/CRT/PDO code. They
// are retained by /INCLUDE, not a callback, object or runtime registration.
// Keep both stamps in sync when changing this split-unit contract. Do not
// remove a witness or enable /FORCE:UNRESOLVED to bypass a missing-peer error.
// This is not a complete duplicate detector: extra untagged legacy objects,
// both-old files, or same-tag altered code still require source/SHA auditing.
#if defined(_MSC_VER)
#pragma detect_mismatch("NCPathCore.SplitPair", "AT1")
#if defined(_M_IX86)
#pragma comment(linker, "/include:_NCPathCoreSplit_AT1_ManagerWitness")
#else
#pragma comment(linker, "/include:NCPathCoreSplit_AT1_ManagerWitness")
#endif
extern "C" void __cdecl NCPathCoreSplit_AT1_PathCoreWitness() noexcept {}
#endif

#if defined(_MSC_VER)
#define NC_PATH_CORE_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_NOINLINE
#endif

NC_PATH_CORE_NOINLINE
void NCManager::ObservePathCoreAcceptedReadAheadInput(
    const NCOrdinaryG00InflightRegistrationProof& proof,
    const NCPreparedHeadCutoverContext& context) noexcept
{
    // L.2A receives an already-created K.7 proof by reference and extracts
    // only scalar identity.  This observer has no return value and cannot
    // participate in Runtime, Gate, PC, Alarm or Motion control decisions.
    m_pathCoreInputContractShadow.ObserveAcceptedReadAhead(
        static_cast<std::uint64_t>(proof.session),
        static_cast<std::uint64_t>(proof.entrySequence),
        proof.dispatchId,
        static_cast<std::uint64_t>(proof.commitSequence),
        static_cast<std::uint64_t>(proof.identity.epoch),
        static_cast<std::uint64_t>(proof.identity.segmentId),
        static_cast<std::int32_t>(proof.sourcePC),
        static_cast<std::int32_t>(proof.sourceLineNumber));

    // L.2AN isolates the existing D geometry fence and argument temporaries.
    ObservePathCoreAcceptedGeometrySameThread(proof, context);

    // BN: one live commanded-value admission, with no control return or logging.
    AdmitPathCoreLiveRetentionSameThread(proof, context);

    // L.2AO confines the borrowed D/F pairs to their immediate consumers.
    // Both groups always run, including neutral/invalid input observations.
    ObservePathCoreCommittedLinkAndSegmentSameThread();
    ObservePathCoreLinkedSegmentRunBoundarySameThread();

    // L.2J derives only the component-wise commanded net displacement
    // between the proven L.2I run head and current tail. It stores no
    // endpoint list, path length or retrace information.
    m_pathCoreLinkedCommittedSegmentRunDisplacementShadow.
        ObserveLatestBoundarySameThread(
            m_pathCoreLinkedCommittedSegmentRunBoundaryShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunBoundaryShadow.
            GetNewestObservationSameThread(0U));

    // L.2K consumes only the newest two L.2J observations and classifies
    // current run-head/tail endpoint closure with scalar axis masks. It
    // stores no geometry array and cannot influence any control decision.
    m_pathCoreLinkedCommittedSegmentRunClosureShadow.
        ObserveLatestDisplacementSameThread(
            m_pathCoreLinkedCommittedSegmentRunDisplacementShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunDisplacementShadow.
            GetNewestObservationSameThread(0U));

    // L.2L consumes only the adjacent newest two proven L.2K closure
    // observations. It records scalar endpoint return-state transitions and
    // cannot preserve a path, issue Motion, or influence control decisions.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionShadow.
        ObserveImmediateClosureTransitionSameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureShadow.
            GetNewestObservationSameThread(0U));

    // L.2M accumulates only scalar event-set coverage and saturating counts
    // from directly adjacent proven L.2L transitions. It stores no event
    // order, endpoint array, traversable path, or control-consumed state.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow.
        ObserveLatestTransitionSameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionShadow.
            GetNewestObservationSameThread());

    // L.2N qualifies only scalar endpoint-state and direct return/reopen
    // coverage from adjacent proven L.2M summaries. It stores no transition
    // order, coordinates, path history, or control-consumed state.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow.
        ObserveLatestSummarySameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow.
            GetNewestObservationSameThread(0U));
    // L.2O compares only the newest adjacent L.2N qualifications after
    // exact same-interval proof. Scalar changes only; no control consumer.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow.
        ObserveLatestQualificationSameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow.
            GetNewestObservationSameThread(0U));
    // L.2P checks the shared scalar projection of two adjacent L.2O
    // transitions, then describes pair-local qualification relations only.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow.
        ObserveImmediateQualificationTransitionPairSameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow.
            GetNewestObservationSameThread(0U));
    // L.2Q validates the retained overlap of adjacent L.2P pairs.
    // Only scalar continuity identities and counts are retained.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow.
        ObserveImmediateQualificationTransitionPairContinuitySameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow.
            GetNewestObservationSameThread(0U));
    // L.2R counts locally consecutive proven Q certificates only.
    // Any unproven boundary clears the scalar accumulation.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow.
        ObserveLatestQualificationTransitionPairContinuitySameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow.
            GetNewestObservationSameThread(0U));
    // L.2S captures the observed R head and latest scalar references.
    // A missing head is never reconstructed from publication counts.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow.
        ObserveLatestQualificationTransitionPairContinuityRunSameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow.
            GetNewestObservationSameThread(0U));
    // L.2T observes the two direct S certificate availability states.
    // Invalid or stale proof never becomes an availability-loss event.
    m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow.
        ObserveImmediateQualificationTransitionPairContinuityRunBoundaryAvailabilitySameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow.
            GetNewestObservationSameThread(0U));
    // L.2U proves only the shared identity/state of two adjacent T records.
    // Missing or invalid proof never becomes a loss/gain pair.
    m_pathCoreBoundaryAvailabilityTransitionPairShadow.
        ObserveImmediateBoundaryAvailabilityTransitionPairSameThread(
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow.
            GetNewestObservationSameThread(0U));
    // L.2V starts from two proven U certificates and extends only locally.
    // Rejected input clears the count; no interrupted run is bridged.
    m_pathCoreBoundaryAvailabilityTransitionPairRunShadow.
        ObserveImmediateBoundaryAvailabilityTransitionPairRunSameThread(
            m_pathCoreBoundaryAvailabilityTransitionPairShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreBoundaryAvailabilityTransitionPairShadow.
            GetNewestObservationSameThread(0U));
    // L.2W starts coverage only at an observed V head with both U sources.
    // A missed head or interruption never reconstructs earlier patterns.
    m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow.
        ObserveLatestBoundaryAvailabilityTransitionPairRunSameThread(
            m_pathCoreBoundaryAvailabilityTransitionPairRunShadow.
            GetNewestObservationSameThread(0U),
            m_pathCoreBoundaryAvailabilityTransitionPairShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreBoundaryAvailabilityTransitionPairShadow.
            GetNewestObservationSameThread(0U));
    // L.2X reports pattern additions only across a proven adjacent W pair.
    // A missing or invalid summary cannot prove loss or full-coverage exit.
    m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow.
        ObserveLatestBoundaryAvailabilityTransitionPairRunCoverageSameThread(
            m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow.
            GetNewestObservationSameThread(0U));
    // L.2Y describes only a proven local pair of X discovery transitions.
    // Shared summaries must bind; interruption never implies pattern loss.
    m_pathCoreRunCoverageTransitionPairShadow.
        ObserveLatestRunCoverageTransitionPairSameThread(
            m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow.
            GetNewestObservationSameThread(0U));
    // L.2Z starts at two proven Y records and extends only direct links.
    // Rejected observations clear the local count; no path length is implied.
    m_pathCoreRunCoverageTransitionPairRunShadow.
        ObserveLatestRunCoverageTransitionPairRunSameThread(
            m_pathCoreRunCoverageTransitionPairShadow.
            GetNewestObservationSameThread(1U),
            m_pathCoreRunCoverageTransitionPairShadow.
            GetNewestObservationSameThread(0U));
    // L.2AA retains only the observed first-Y coverage mask and its gain.
    // A missed Z head is unavailable; no later source reconstructs it.
    m_pathCoreRunCoverageBoundaryShadow.ObserveLatestRunCoverageBoundarySameThread(
        m_pathCoreRunCoverageTransitionPairRunShadow.GetNewestObservationSameThread(1U),
        m_pathCoreRunCoverageTransitionPairRunShadow.GetNewestObservationSameThread(0U));
    // L.2AB compares adjacent AA observations with exact diagnostic reasons.
    // This is boundary-certificate availability, without an execution claim.
    m_pathCoreRunCoverageBoundaryAvailabilityShadow.
        ObserveLatestRunCoverageBoundaryAvailabilitySameThread(
            m_pathCoreRunCoverageBoundaryShadow.GetNewestObservationSameThread(1U),
            m_pathCoreRunCoverageBoundaryShadow.GetNewestObservationSameThread(0U));
    // L.2AC validates the shared AA observation of two adjacent AB proofs.
    // This scalar pair does not retain an event list or execution history.
    m_pathCoreRunCoverageBoundaryAvailabilityPairShadow.
        ObserveLatestRunCoverageBoundaryAvailabilityPairSameThread(
            m_pathCoreRunCoverageBoundaryAvailabilityShadow.GetNewestObservationSameThread(1U),
            m_pathCoreRunCoverageBoundaryAvailabilityShadow.GetNewestObservationSameThread(0U));
    // L.2AD counts directly overlapping AC proofs in this observer only.
    // Availability loss/gain does not imply one common geometric run.
    m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow.
        ObserveImmediateRunCoverageBoundaryAvailabilityPairRunSameThread(
            m_pathCoreRunCoverageBoundaryAvailabilityPairShadow.GetNewestObservationSameThread(1U),
            m_pathCoreRunCoverageBoundaryAvailabilityPairShadow.GetNewestObservationSameThread(0U));
    // L.2AE checks current AD/AC/AB/AA semantic coherence and projects only
    // source AD's local certificate scope, never path/execution readiness.
    m_pathCoreCurrentProofScopeShadow.ObserveCurrentProofScopeSameThread(
        m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryAvailabilityPairShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryAvailabilityShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryShadow.GetNewestObservationSameThread());
    // L.2AF emits only a compact identity for the current AE-bound proof frontier.
    // It authorizes no path capture, queue operation, Motion action or B2 behavior.
    m_pathCoreProofFrontierShadow.ObserveProofFrontierSameThread(
        m_pathCoreCurrentProofScopeShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryAvailabilityPairShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryAvailabilityShadow.GetNewestObservationSameThread(),
        m_pathCoreRunCoverageBoundaryShadow.GetNewestObservationSameThread());
    // L.2AN isolates AI's temporary owner view from this long-lived frame.
    // Sampling, readback and compact publication remain in the same order.
    SamplePathCoreCurrentFrontierSameThread();
    // L.2AK consumes AJ once after AI; only a private diagnostic is retained.
    // The void probe result cannot participate in control/PC/Motion decisions.
    ProbePathCoreLastSampleReadbackSameThread();
    m_pathCoreInputHandoffCompactShadow.ObserveReadAheadInputEvent();
}

// NC-0.2L.2AO / Immediate-Pair Borrow-Scope Isolation.
// D is not published again while E/F borrow its two newest records. F is
// published before the next helper borrows its pair for G/H/I. Only const
// slot pointers are borrowed locally; no record, geometry, or history copy.
// These scopes are not locks, snapshots, epochs, or cross-thread protection.
// Keep E/F and G/H/I unconditional and ordered, even on rejected evidence.
// This preserves every predecessor proof fence and all later J->AF work.
NC_PATH_CORE_NOINLINE
void NCManager::ObservePathCoreCommittedLinkAndSegmentSameThread() noexcept
{
    // L.2E compares only the two newest L.2D records. Numeric endpoint
    // continuity and the opaque queue-tail seal remain orthogonal evidence;
    // neither this observer nor a mismatch can influence existing control.
    const NCPathCoreOrdinaryG00CommittedEndpointPairV1* const
        previousCommittedGeometry =
        m_pathCoreCommittedGeometryShadow.
        GetNewestObservationSameThread(1U);
    const NCPathCoreOrdinaryG00CommittedEndpointPairV1* const
        currentCommittedGeometry =
        m_pathCoreCommittedGeometryShadow.
        GetNewestObservationSameThread(0U);
    m_pathCoreCommittedGeometryLinkShadow.
        ObserveImmediatePairSameThread(
            previousCommittedGeometry,
            currentCommittedGeometry);

    // L.2F consumes only the newest proven L.2E relationship and forms a
    // bounded commanded endpoint-segment description. A neutral/invalid
    // result remains history-only and cannot affect the accepted G00 path.
    m_pathCoreLinkedCommittedSegmentShadow.
        ObserveLinkedCommittedPairSameThread(
            m_pathCoreCommittedGeometryLinkShadow.
            GetNewestObservationSameThread(),
            previousCommittedGeometry,
            currentCommittedGeometry);

    // NC-0.2L.2AV: first runtime consumer of AU, after current E/F publish.
    // Always replace the last diagnostic, including rejection; never gate
    // the following G->AF/AI/readback work or alter accepted motion/control.
    // No geometry is copied or retained here. The result ages immediately
    // when sources change and is not a current-proof/lifecycle token.
    m_pathCoreCommandedSegmentLastCheck =
        CheckPathCoreCurrentCommandedSegmentSameThread();
}

NC_PATH_CORE_NOINLINE
void NCManager::ObservePathCoreLinkedSegmentRunBoundarySameThread() noexcept
{
    // L.2G consumes only the newest two L.2F observations. It records one
    // immediate scalar continuity relation and cannot accumulate a path,
    // publish geometry, or influence any existing control decision.
    const NCPathCoreLinkedCommittedSegmentRecordV1* const
        previousLinkedCommittedSegment =
        m_pathCoreLinkedCommittedSegmentShadow.
        GetNewestObservationSameThread(1U);
    const NCPathCoreLinkedCommittedSegmentRecordV1* const
        currentLinkedCommittedSegment =
        m_pathCoreLinkedCommittedSegmentShadow.
        GetNewestObservationSameThread(0U);
    m_pathCoreLinkedCommittedSegmentPairShadow.
        ObserveImmediatePairSameThread(
            previousLinkedCommittedSegment,
            currentLinkedCommittedSegment);

    // L.2H reduces directly overlapping proven L.2G pairs to one scalar
    // run summary. It stores no endpoint arrays or per-segment history and
    // cannot become a traversable path or influence any control decision.
    m_pathCoreLinkedCommittedSegmentRunShadow.
        ObserveLatestPairSameThread(
            m_pathCoreLinkedCommittedSegmentPairShadow.
            GetNewestObservationSameThread());

    // L.2I consumes only the newest proven L.2H run, its source L.2G pair
    // and the same two L.2F segment observations. It retains only the run
    // head/tail commanded endpoints; no intermediate path is accumulated.
    m_pathCoreLinkedCommittedSegmentRunBoundaryShadow.
        ObserveLatestRunSameThread(
            m_pathCoreLinkedCommittedSegmentRunShadow.
            GetNewestObservationSameThread(),
            m_pathCoreLinkedCommittedSegmentPairShadow.
            GetNewestObservationSameThread(),
            previousLinkedCommittedSegment,
            currentLinkedCommittedSegment);

}

// L.2AN: the existing D observation, still immediately after B/C.
// Reference-only inputs; no proof/context/endpoint snapshot or new fence.
NC_PATH_CORE_NOINLINE
void NCManager::ObservePathCoreAcceptedGeometrySameThread(
    const NCOrdinaryG00InflightRegistrationProof& proof,
    const NCPreparedHeadCutoverContext& context) noexcept
{
    // L.2D observes the authoritative pre-resolve commanded MCS image and
    // the endpoint committed transactionally by the existing G00 producer.
    // It cannot alter the already-accepted Runtime, Gate, PC or Motion path.
    const NCPathCoreAcceptedInputRecord* const acceptedInput =
        m_pathCoreInputContractShadow.GetNewestSameThread();
    const bool committedGeometryFenceValid =
        acceptedInput != nullptr &&
        context.capturedBeforeResolve &&
        context.hasHead &&
        context.runtimeModalBeforeValid &&
        context.runtimeModalBefore.imageValid &&
        context.runtimeModalBefore.commandedMCSValid &&
        proof.registered &&
        proof.exact &&
        proof.active &&
        proof.readAheadCutover &&
        proof.bounded &&
        !proof.shadowOnly &&
        proof.runtimeInfluence &&
        !proof.motionWrite &&
        proof.accountingValid &&
        proof.session == context.queue.session &&
        proof.session == context.head.session &&
        proof.entrySequence == context.head.entrySequence &&
        proof.sourcePC == context.sourcePC &&
        proof.sourcePC == context.head.sourcePC &&
        proof.sourceLineNumber == context.sourceLineNumber &&
        proof.sourceLineNumber == context.head.sourceLineNumber &&
        proof.sourceExecutionEpoch ==
        context.runtimeSource.executionEpoch &&
        static_cast<std::uint64_t>(proof.identity.epoch) ==
        context.runtimeSource.executionEpoch &&
        acceptedInput->preparedSession ==
        static_cast<std::uint64_t>(proof.session) &&
        acceptedInput->preparedEntrySequence ==
        static_cast<std::uint64_t>(proof.entrySequence) &&
        acceptedInput->dispatchId == proof.dispatchId &&
        acceptedInput->commitSequence ==
        static_cast<std::uint64_t>(proof.commitSequence) &&
        acceptedInput->motionExecutionEpoch ==
        static_cast<std::uint64_t>(proof.identity.epoch) &&
        acceptedInput->motionSegmentId ==
        static_cast<std::uint64_t>(proof.identity.segmentId) &&
        acceptedInput->sourcePC == proof.sourcePC &&
        acceptedInput->sourceLineNumber == proof.sourceLineNumber;
    // These const scalar getters do not modify the accepted-input owner.
    // Materialize them before the wide D call to avoid live call operands.
    const NCPathCoreAcceptedInputPairRelation acceptedPairRelation =
        m_pathCoreInputContractShadow.GetPairRelationSameThread();
    const std::uint32_t acceptedRunLength =
        m_pathCoreInputContractShadow.GetLatestAcceptedHandoffRunLengthSameThread();
    m_pathCoreCommittedGeometryShadow.
        ObserveAcceptedOrdinaryG00CommittedEndpointPair(
            acceptedInput,
            acceptedPairRelation,
            acceptedRunLength,
            static_cast<std::uint64_t>(
                proof.queueTailTransactionSequence),
            proof.queueTailAxisMask,
            proof.queueTailBeforeFingerprint,
            proof.queueTailCommittedFingerprint,
            context.runtimeModalBefore.commandedMCS,
            CoordSys.commandedMCS,
            committedGeometryFenceValid);

}

// NC-0.2L.2AQ / Single-Source Frontier Owner Binding.
// Both AI sampling and AJ reading borrow THIS manager's same six owners here.
// Return a small reference view, never an owner/record snapshot or ring pointer.
// No cached view, publication check, validation shortcut or lifetime extension.
// The caller must remain on the NC thread, with live owners and no reentry;
// consume this view within the call and do not retain it across owner lifetime.
// AG/AH/AJ still perform their complete current-source validation afterward.
// C++14 may copy the view; that only copies its six references, not the owners.
NC_PATH_CORE_NOINLINE
NCPathCoreFrontierOwnersSameThread
NCManager::BorrowPathCoreFrontierOwnersSameThread() const noexcept
{
    // NC-0.2L.2AR / Borrowed Frontier View Field Contract Guard.
    // AG's 48-byte check alone does not specify how each field borrows an
    // owner. Bind every DECLARED view-field type to this manager's approved
    // member type: exactly a const lvalue reference, never a value, pointer,
    // mutable/rvalue/volatile reference or same-sized substitute.
    // Use unparenthesized member names in decltype: inspect declarations,
    // not the value category of an access through this const function.
    // Compile-time only; the existing six-owner initializer stays unchanged.
    // These checks do NOT prove instance identity, live owners, publication
    // freshness, same-thread execution, no allocation or lifetime extension.
    // Keep all AG/AH/AJ validation and the reviewed AP/FIX1 storage guards.
    static_assert(std::is_same<
        decltype(NCPathCoreFrontierOwnersSameThread::frontier),
        const decltype(m_pathCoreProofFrontierShadow)&>::value,
        "AR view field frontier must be its exact const owner reference.");
    static_assert(std::is_same<
        decltype(NCPathCoreFrontierOwnersSameThread::audit),
        const decltype(m_pathCoreCurrentProofScopeShadow)&>::value,
        "AR view field audit must be its exact const owner reference.");
    static_assert(std::is_same<
        decltype(NCPathCoreFrontierOwnersSameThread::run),
        const decltype(m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow)&>::value,
        "AR view field run must be its exact const owner reference.");
    static_assert(std::is_same<
        decltype(NCPathCoreFrontierOwnersSameThread::pair),
        const decltype(m_pathCoreRunCoverageBoundaryAvailabilityPairShadow)&>::value,
        "AR view field pair must be its exact const owner reference.");
    static_assert(std::is_same<
        decltype(NCPathCoreFrontierOwnersSameThread::transition),
        const decltype(m_pathCoreRunCoverageBoundaryAvailabilityShadow)&>::value,
        "AR view field transition must be its exact const owner reference.");
    static_assert(std::is_same<
        decltype(NCPathCoreFrontierOwnersSameThread::boundary),
        const decltype(m_pathCoreRunCoverageBoundaryShadow)&>::value,
        "AR view field boundary must be its exact const owner reference.");

    return NCPathCoreFrontierOwnersSameThread{
        m_pathCoreProofFrontierShadow,
        m_pathCoreCurrentProofScopeShadow,
        m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow,
        m_pathCoreRunCoverageBoundaryAvailabilityPairShadow,
        m_pathCoreRunCoverageBoundaryAvailabilityShadow,
        m_pathCoreRunCoverageBoundaryShadow
    };
}

// NC-0.2L.2AN / Accepted-Input Temporary-Frame Isolation.
// AI's six borrowed const owner references exist only during this call, not
// in the enclosing B->AF / readback frame. No source record is copied here.
// Keep this separate from the readback probe: merging them would retain the
// owner-view frame beneath its 112-byte output and nested validation again.
// No new state, validation shortcut, callback, or task is introduced. The
// entire chain still requires the same NC thread, live owners and no reentry.
NC_PATH_CORE_NOINLINE
void NCManager::SamplePathCoreCurrentFrontierSameThread() noexcept
{
    const NCPathCoreFrontierOwnersSameThread owners =
        BorrowPathCoreFrontierOwnersSameThread();
    m_pathCoreFrontierReadProbe.SampleCurrentFrontierSameThread(owners);
}

// NC-0.2L.2AJ: read-side diagnostic; no call site is added to any task/loop.
// Select corresponding private owners here, not in an external caller.
NC_PATH_CORE_NOINLINE
NCPathCoreSampleReadResult NCManager::ReadPathCoreLastFrontierSampleSameThread(
    NCPathCoreFrontierReadValueV1& output) const noexcept
{
    output.Clear();
    const NCPathCoreFrontierOwnersSameThread owners =
        BorrowPathCoreFrontierOwnersSameThread();
    const NCPathCoreSampleReadResult result = ValidateLastFrontierSampleSameThread(
        m_pathCoreFrontierReadProbe, owners);
    if (result.IsBound())
        output = m_pathCoreFrontierReadProbe.GetLastSampleSameThread().value;
    return result;
}

// NC-0.2L.2AM / Exact-Guarded Readback Consumer Consolidation.
// AJ performs current-source revalidation; AL then checks its complete receipt
// and exact copy-out against AI. After AL succeeds, its V1 contract entails:
// - either one of the six recognized rejection codes with EMPTY output; or
// - a bound available/unavailable result with output exactly equal to the
//   successful-shaped AI value, including ALL ticket and AF scalar fields.
// Repeating IsBound/IsEmpty/HasCapturedShape/availability checks here adds no
// rejection under the same-thread, no-mutation precondition. Classify only
// AFTER the unchanged guard. Unknown/future codes retain mismatch by default.
// No source validation is bypassed or cached, no extra certificate/history or
// member is added, and no result participates in control/HMI/PDO decisions.
// The existing 112-byte caller-owned output is disjoint from NCManager and
// discarded on return. These private fields describe the LAST CALL, not live
// readiness, a RESET/STOP epoch, or a Path Queue/Motion/B2 execution permit.
NC_PATH_CORE_NOINLINE
void NCManager::ProbePathCoreLastSampleReadbackSameThread() noexcept
{
    using Disposition = PathCoreSampleReadbackDisposition;
    using ReadCode = NCPathCoreSampleReadCode;
    m_pathCoreSampleReadbackDisposition = Disposition::READBACK_CONTRACT_MISMATCH;
    NCPathCoreFrontierReadValueV1 output{};
    m_pathCoreSampleReadbackResult = ReadPathCoreLastFrontierSampleSameThread(output);
    if (!IsLastSampleReadbackConsistentSameThread(
        m_pathCoreFrontierReadProbe.GetLastSampleSameThread(),
        m_pathCoreSampleReadbackResult, output)) return;

    switch (m_pathCoreSampleReadbackResult.code)
    {
    case ReadCode::BOUND_CURRENT_BOUNDARY_AVAILABLE:
        m_pathCoreSampleReadbackDisposition = Disposition::READ_BOUNDARY_AVAILABLE;
        break;
    case ReadCode::BOUND_CURRENT_BOUNDARY_UNAVAILABLE:
        m_pathCoreSampleReadbackDisposition = Disposition::READ_BOUNDARY_UNAVAILABLE;
        break;
    case ReadCode::NOT_SAMPLED:
    case ReadCode::SAMPLE_CAPTURE_REJECTED:
    case ReadCode::SAMPLE_READ_REJECTED:
    case ReadCode::SAMPLE_STATUS_MISMATCH:
    case ReadCode::INVALID_SAMPLE_RECORD:
    case ReadCode::CURRENT_VALUE_REJECTED:
        m_pathCoreSampleReadbackDisposition = Disposition::READ_REJECTED_EMPTY_OUTPUT;
        break;
    default:
        break; // NOT_CHECKED/unknown is never promoted to consistent rejection.
    }
}

// NC-0.2L.2AU / Current Commanded Segment Source Revalidation Contract.
// Internal same-NC-thread diagnostic; AV samples it immediately after E/F.
// This manager chooses its matching owners. Return only an instantaneous
// status, never an endpoint/record snapshot, slot pointer or execution permit.
NC_PATH_CORE_NOINLINE
NCPathCoreCommandedSegmentCheck
NCManager::CheckPathCoreCurrentCommandedSegmentSameThread() const noexcept
{
    return CheckCurrentCommandedSegmentSameThread(
        m_pathCoreLinkedCommittedSegmentShadow,
        m_pathCoreCommittedGeometryLinkShadow,
        m_pathCoreCommittedGeometryShadow);
}

// NC-0.2L.2AW: mathematical chord of newest D, including FIRST_INPUT.
// No source admission/Motion claim and no periodic call is introduced here.
NC_PATH_CORE_NOINLINE
NCPathCoreCommandedChordCode
NCManager::EvaluatePathCoreCurrentCommandedChordAxisSameThread(
    std::uint32_t axisIndex, double unitParameter,
    NCPathCoreCommandedChordAxisValueV1& output) const noexcept
{
    return EvaluateCurrentCommandedChordAxisSameThread(
        m_pathCoreCommittedGeometryShadow, axisIndex, unitParameter, output);
}

// NC-0.2L.2AX: single-axis inverse of the commanded D endpoint chord.
// No production caller, geometry snapshot or execution-progress inference.
NC_PATH_CORE_NOINLINE
NCPathCoreCommandedChordLocateCode
NCManager::LocatePathCoreCurrentCommandedChordAxisSameThread(
    const std::uint32_t axisIndex, const double queryCoordinateMCS,
    NCPathCoreCommandedChordLocationV1& output) const noexcept
{
    return LocateCurrentCommandedChordAxisSameThread(
        m_pathCoreCommittedGeometryShadow, axisIndex, queryCoordinateMCS, output);
}

// NC-0.2L.2AY: one explicit commanded chord, including FIRST/BOUNDARY D.
// Output storage is supplied by the caller; no production caller is added.
NC_PATH_CORE_NOINLINE
NCPathCoreCommandedChordSegmentCaptureCode
NCManager::CapturePathCoreCurrentCommandedChordSegmentSameThread(
    NCPathCoreCommandedChordSegmentV1& output) const noexcept
{
    return CaptureCurrentCommandedChordSegmentSameThread(
        m_pathCoreCommittedGeometryShadow, output);
}


// BN: this namespace is constant-initialized before global Master construction.
// Only startup/creation on one thread may allocate an owner tag.
namespace
{
    NCPathCoreLiveRetentionOwnerCounter g_pathCoreLiveOwnerCounter{};

    bool PathCoreLiveSegmentValuesEqual(
        const NCPathCoreCommandedChordSegmentV1& a,
        const NCPathCoreCommandedChordSegmentV1& b) noexcept
    {
        if (a.localIdentity.geometryPublicationSequence != b.localIdentity.geometryPublicationSequence ||
            a.localIdentity.acceptedInputChainGeneration != b.localIdentity.acceptedInputChainGeneration ||
            a.axisMask != b.axisMask || a.acceptedInputRunLength != b.acceptedInputRunLength ||
            a.schemaVersion != b.schemaVersion || a.sourcePairRelation != b.sourcePairRelation ||
            a.kind != b.kind || a.frame != b.frame || a.extent != b.extent ||
            a.reserved[0U] != b.reserved[0U] || a.reserved[1U] != b.reserved[1U])
            return false;
        for (std::size_t axis = 0U; axis < 8U; ++axis)
        {
            if (std::memcmp(&a.startMCS[axis], &b.startMCS[axis], sizeof(double)) != 0 ||
                std::memcmp(&a.endMCS[axis], &b.endMCS[axis], sizeof(double)) != 0)
                return false;
        }
        return true;
    }
}

std::uint64_t NCManager::AllocatePathCoreLiveOwnerTagStartup() noexcept
{
    return g_pathCoreLiveOwnerCounter.Allocate();
}

NC_PATH_CORE_NOINLINE
NCPathCoreLiveRetentionScopeV1 NCManager::BuildPathCoreLiveScopeSameThread() const noexcept
{
    NCPathCoreLiveRetentionScopeV1 scope{};
    scope.runToken = m_pathCoreLiveBookkeeping.currentRunToken;
    scope.cacheGeneration = GetBaseProgramCache().GetGeneration();
    scope.programScope = static_cast<std::uint32_t>(GetBaseProgramScope());
    scope.operationMode = static_cast<std::uint32_t>(m_mode);
    scope.ownerLease = m_programMotionLease;
    return scope;
}

NC_PATH_CORE_NOINLINE
void NCManager::CapturePathCoreLiveNativeConfigSameThread() noexcept
{
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        const AxisContext& native = m_motion.GetAxisContext(static_cast<int>(axis));
        m_pathCoreLiveNative.lead[axis] = native.finalLead;
        m_pathCoreLiveNative.resolution[axis] = native.resolution_PPR;
        m_pathCoreLiveNative.axisIndex[axis] = static_cast<std::int32_t>(native.axisIndex);
        m_pathCoreLiveNative.axisType[axis] = static_cast<std::uint8_t>(native.axisType);
        m_pathCoreLiveNative.axisName[axis] = m_axisNames[axis];
        m_pathCoreLiveNative.exists[axis] = native.isExist ? 1U : 0U;
    }
    m_pathCoreLiveBookkeeping.nativeBound = true;
}

NC_PATH_CORE_NOINLINE
bool NCManager::IsPathCoreLiveNativeConfigCurrentSameThread() noexcept
{
    if (!m_pathCoreLiveBookkeeping.nativeBound) return false;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        const AxisContext& native = m_motion.GetAxisContext(static_cast<int>(axis));
        const std::uint32_t type = static_cast<std::uint32_t>(native.axisType);
        if (type > static_cast<std::uint32_t>(AxisType::ROTARY_CONTINUOUS) ||
            static_cast<std::int32_t>(native.axisIndex) != m_pathCoreLiveNative.axisIndex[axis] ||
            type != static_cast<std::uint32_t>(m_pathCoreLiveNative.axisType[axis]) ||
            m_axisNames[axis] != m_pathCoreLiveNative.axisName[axis] ||
            (native.isExist ? 1U : 0U) != m_pathCoreLiveNative.exists[axis] ||
            std::memcmp(&native.finalLead, &m_pathCoreLiveNative.lead[axis], sizeof(double)) != 0 ||
            std::memcmp(&native.resolution_PPR, &m_pathCoreLiveNative.resolution[axis], sizeof(double)) != 0)
            return false;
    }
    return true;
}

NC_PATH_CORE_NOINLINE
void NCManager::QueuePathCoreLiveSummarySameThread() noexcept
{
    // No I/O here: this helper is also reached from the accepted-input Observe.
    // BO keeps old identity feedback after capacity/epoch closes BN admission.
    m_pathCoreExecutionLink.Seal(static_cast<std::uint8_t>(m_pathCoreLiveStatus.reason));
    if (m_pathCoreLiveStatus.runToken == 0ULL ||
        m_pathCoreLiveBookkeeping.runSummaryQueued ||
        m_pathCoreLiveBookkeeping.summaryPending) return;
    m_pathCoreLiveSummary.status = m_pathCoreLiveStatus;
    m_pathCoreLiveSummary.readbackCount = m_pathCoreLiveBookkeeping.readbackCount;
    m_pathCoreLiveSummary.holdCount = m_pathCoreLiveBookkeeping.holdCount;
    m_pathCoreLiveSummary.resumeCount = m_pathCoreLiveBookkeeping.resumeCount;
    m_pathCoreLiveSummary.lastReadbackMatched = m_pathCoreLiveBookkeeping.lastReadbackMatched;
    m_pathCoreLiveBookkeeping.runSummaryQueued = true;
    m_pathCoreLiveBookkeeping.summaryPending = true;
}

NC_PATH_CORE_NOINLINE
void NCManager::FlushPathCoreLiveSummarySameThread() noexcept
{
    // BR-BEGIN
    FlushPathCoreReturnSummarySameThread();
    // BR-END
        // Only ProcessTask entry and completed fresh Start call this low-rate seam.
    // BP-BEGIN
    FlushPathCoreCompletedSummarySameThread();
    // BP-END
    // BQ-BEGIN
    FlushPathCoreCommittedRunSummarySameThread();
    // BQ-END
    FlushPathCoreExecutionSummarySameThread();
    if (!m_pathCoreLiveBookkeeping.summaryPending) return;
    const NCPathCoreLiveRetentionStatusV1& s = m_pathCoreLiveSummary.status;
    RtPrintf("[PCORE-V1] run=%llu life=%llu state=%u reason=%u count=%u accepted=%u replay=%u readback=%llu holds=%u resumes=%u matched=%u store=%u\n",
        static_cast<unsigned long long>(s.runToken),
        static_cast<unsigned long long>(s.lifetime),
        static_cast<unsigned int>(s.state),
        static_cast<unsigned int>(s.reason),
        static_cast<unsigned int>(s.storedCount),
        static_cast<unsigned int>(s.admittedCount),
        static_cast<unsigned int>(s.replayCount),
        static_cast<unsigned long long>(m_pathCoreLiveSummary.readbackCount),
        static_cast<unsigned int>(m_pathCoreLiveSummary.holdCount),
        static_cast<unsigned int>(m_pathCoreLiveSummary.resumeCount),
        m_pathCoreLiveSummary.lastReadbackMatched ? 1U : 0U,
        static_cast<unsigned int>(s.lastStoreCode));
    m_pathCoreLiveBookkeeping.summaryPending = false;
}

NC_PATH_CORE_NOINLINE
void NCManager::FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason reason) noexcept
{
    // Constructor-safe: no NC state, Scope, Alarm or AxisContext reads.
// BP-BEGIN
    // Only this synchronous end attempt may keep an unpublished candidate.
    // Every other fence also revokes the last detached manager result.
    if (reason != PathCoreLiveFenceReason::PROGRAM_END)
        DiscardPathCoreCompletedSnapshotSameThread();
    // BP-END
    // BQ-BEGIN
        // Epoch-only BN admission loss does not end this distinct BQ run.
        // Explicit NC boundaries revoke BQ; successful End publishes below its Gate.
    if (reason != PathCoreLiveFenceReason::PROGRAM_END &&
        reason != PathCoreLiveFenceReason::EPOCH_CHANGED)
        ClosePathCoreCommittedRunSameThread();
    // BQ-END
    ClosePathCoreExecutionLinkSameThread(reason);
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (m_pathCoreLiveStatus.runToken == 0ULL) return;
    const std::uint32_t retainedBeforeFence = m_pathCoreLiveStatus.storedCount;
    m_pathCoreLiveRetention.Fence(reason);
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    // The end report's count is the retained count immediately before Fence.
    m_pathCoreLiveStatus.storedCount = retainedBeforeFence;
    QueuePathCoreLiveSummarySameThread();
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    m_pathCoreLiveCapture.Clear();
    m_pathCoreLiveReadback.Clear();
    m_pathCoreLiveHandle.Clear();
}

NC_PATH_CORE_NOINLINE
bool NCManager::ValidatePathCoreLiveRetentionSameThread() noexcept
{
    // BQ-FIX2-BEGIN
        // A proven BQ receipt can still await its producer epoch acknowledgement.
        // Keep all live queries unavailable while that existing pending guard is
        // set; do not let BN's generic pending fence discard the saved receipt.
    DrainPathCorePendingCommandedCaptureSameThread();
    if (m_pathCoreCommittedBookkeeping.pendingAdmission) return false;
    // BQ-FIX2-END
    using State = NCPathCoreLiveRetentionState;
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (m_pathCoreLiveStatus.state != State::ARMED &&
        m_pathCoreLiveStatus.state != State::OPEN &&
        m_pathCoreLiveStatus.state != State::HELD) return false;
    if (Close_System_Com_flag)
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::INTERRUPTION);
        return false;
    }
    const NCState state = m_state.load(std::memory_order_acquire);
    if (state != NCState::RUN && state != NCState::HOLD)
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::STATE_CHANGE);
        return false;
    }
    if (AlarmManager::GetInstance().HasAlarm() ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::SCOPE_CHANGED);
        return false;
    }
    if (Homing.IsActive())
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::HOME);
        return false;
    }
    if (!m_macroStack.empty())
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::MACRO);
        return false;
    }
    if (!IsPathCoreLiveNativeConfigCurrentSameThread())
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::AXIS_CONFIG);
        return false;
    }
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    if (!m_pathCoreLiveRetention.CheckScope(scope, m_motion.GetCurrentExecutionEpoch()))
    {
        m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
        QueuePathCoreLiveSummarySameThread();
        return false;
    }
    if (state == NCState::HOLD)
    {
        const bool alreadyHeld = m_pathCoreLiveStatus.state == State::HELD;
        m_pathCoreLiveRetention.Hold(scope, m_motion.GetCurrentExecutionEpoch());
        m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
        if (!alreadyHeld && m_pathCoreLiveStatus.state == State::HELD &&
            m_pathCoreLiveBookkeeping.holdCount < (std::numeric_limits<std::uint32_t>::max)())
            ++m_pathCoreLiveBookkeeping.holdCount;
        return false;
    }
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    // A provisional RUN or arbitrary state write cannot release a saved HOLD.
    return m_pathCoreLiveStatus.state == State::ARMED ||
        m_pathCoreLiveStatus.state == State::OPEN;
}

NC_PATH_CORE_NOINLINE
void NCManager::ArmPathCoreLiveRetentionSameThread() noexcept
{
    // Called only after the fresh Start admission's final successful End.
    FlushPathCoreLiveSummarySameThread();
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::EXPLICIT_FENCE);
    FlushPathCoreLiveSummarySameThread();
    if (m_pathCoreLiveBookkeeping.lastRunToken ==
        (std::numeric_limits<std::uint64_t>::max)())
    {
        m_pathCoreLiveRetention.DisableExhausted(PathCoreLiveFenceReason::RUN_EXHAUSTED);
        m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
        QueuePathCoreLiveSummarySameThread();
        return;
    }
    ++m_pathCoreLiveBookkeeping.lastRunToken;
    m_pathCoreLiveBookkeeping.currentRunToken = m_pathCoreLiveBookkeeping.lastRunToken;
    m_pathCoreLiveBookkeeping.readbackCount = 0ULL;
    m_pathCoreLiveBookkeeping.holdCount = 0U;
    m_pathCoreLiveBookkeeping.resumeCount = 0U;
    m_pathCoreLiveBookkeeping.lastReadbackMatched = false;
    m_pathCoreLiveBookkeeping.runSummaryQueued = false;
    CapturePathCoreLiveNativeConfigSameThread();
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    m_pathCoreLiveRetention.Arm(scope);
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (m_pathCoreLiveStatus.state != NCPathCoreLiveRetentionState::ARMED)
        QueuePathCoreLiveSummarySameThread();
    else
        (void)m_pathCoreExecutionLink.Arm(m_pathCoreLiveStatus.ownerTag,
            m_pathCoreLiveStatus.runToken, m_lastConsumedMotionFeedbackSequence);
    // BQ-BEGIN
    ArmPathCoreCommittedRunSameThread();
    // BQ-END
}

NC_PATH_CORE_NOINLINE
void NCManager::PausePathCoreLiveRetentionSameThread() noexcept
{
    // BQ-BEGIN
    if (ValidatePathCoreCommittedRunSameThread()) m_pathCoreCommittedRun.Hold();
    // BQ-END
    if (!ValidatePathCoreLiveRetentionSameThread()) return;
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    m_pathCoreLiveRetention.Hold(scope, m_motion.GetCurrentExecutionEpoch());
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (m_pathCoreLiveStatus.state == NCPathCoreLiveRetentionState::HELD &&
        m_pathCoreLiveBookkeeping.holdCount < (std::numeric_limits<std::uint32_t>::max)())
        ++m_pathCoreLiveBookkeeping.holdCount;
}

NC_PATH_CORE_NOINLINE
void NCManager::ResumePathCoreLiveRetentionSameThread() noexcept
{
    // BQ-BEGIN
    if (ValidatePathCoreCommittedRunSameThread() && m_state == NCState::RUN)
        m_pathCoreCommittedRun.Resume();
    // BQ-END
        // Only the two completed PROGRAM/single-block Resume transactions call here.
    (void)ValidatePathCoreLiveRetentionSameThread();
    if (m_pathCoreLiveStatus.state != NCPathCoreLiveRetentionState::HELD ||
        m_state != NCState::RUN) return;
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    m_pathCoreLiveRetention.Resume(scope, m_motion.GetCurrentExecutionEpoch(), true);
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (m_pathCoreLiveStatus.state != NCPathCoreLiveRetentionState::ARMED &&
        m_pathCoreLiveStatus.state != NCPathCoreLiveRetentionState::OPEN)
        QueuePathCoreLiveSummarySameThread();
    else if (m_pathCoreLiveBookkeeping.resumeCount < (std::numeric_limits<std::uint32_t>::max)())
        ++m_pathCoreLiveBookkeeping.resumeCount;
}

NC_PATH_CORE_NOINLINE
void NCManager::AdmitPathCoreLiveRetentionSameThread(
    const NCOrdinaryG00InflightRegistrationProof& proof,
    const NCPreparedHeadCutoverContext& context) noexcept
{
    if (!ValidatePathCoreLiveRetentionSameThread())
    {
        // A newly observed accepted input during HOLD cannot be silently skipped.
        if (m_pathCoreLiveStatus.state == NCPathCoreLiveRetentionState::HELD)
        {
            m_pathCoreLiveRetention.Reject(PathCoreLiveFenceReason::ADMISSION_DURING_HOLD);
            m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
            QueuePathCoreLiveSummarySameThread();
        }
        return;
    }
    const NCPathCoreOrdinaryG00CommittedEndpointPairV1* const d =
        m_pathCoreCommittedGeometryShadow.GetNewestObservationSameThread();
    const MotionExecutionEpoch epoch = m_motion.GetCurrentExecutionEpoch();
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    const bool exactSource = d != nullptr && d->IsAcceptedCommandCommitted() &&
        proof.registered && proof.exact && proof.active && proof.readAheadCutover &&
        proof.bounded && !proof.shadowOnly && proof.runtimeInfluence &&
        !proof.motionWrite && proof.accountingValid &&
        context.capturedBeforeResolve && context.hasHead &&
        proof.identity.IsAssigned() && proof.identity.epoch == epoch &&
        proof.identity.source == MotionCommandSource::NC_MEMORY &&
        proof.identity.sourceBlockId == static_cast<MotionSourceBlockId>(proof.sourcePC) &&
        proof.ownerLease.Matches(scope.ownerLease) &&
        proof.sourceExecutionEpoch == static_cast<std::uint64_t>(epoch) &&
        proof.sourceExecutionEpoch == context.runtimeSource.executionEpoch &&
        proof.scope == GetBaseProgramScope() && proof.cacheGeneration == scope.cacheGeneration &&
        proof.frameId == NC_PROGRAM_FRAME_ID_INVALID &&
        proof.owner == static_cast<std::uint8_t>(scope.ownerLease.owner) &&
        proof.ownerGeneration == static_cast<std::uint64_t>(scope.ownerLease.generation) &&
        context.runtimeSource.scope == proof.scope &&
        context.runtimeSource.cacheGeneration == proof.cacheGeneration &&
        context.runtimeSource.frameId == proof.frameId &&
        context.runtimeSource.owner == proof.owner &&
        context.runtimeSource.ownerGeneration == proof.ownerGeneration &&
        context.runtimeSource.programFlowGeneration == proof.programFlowGeneration &&
        proof.programFlowGeneration ==
        m_gmBlockTransactionCounters.m98Calls + m_gmBlockTransactionCounters.m99Returns &&
        proof.session == context.queue.session && proof.session == context.head.session &&
        proof.entrySequence == context.head.entrySequence &&
        proof.sourcePC == context.sourcePC && proof.sourcePC == context.head.sourcePC &&
        proof.sourceLineNumber == context.sourceLineNumber &&
        proof.sourceLineNumber == context.head.sourceLineNumber &&
        d->preparedSession == static_cast<std::uint64_t>(proof.session) &&
        d->preparedEntrySequence == static_cast<std::uint64_t>(proof.entrySequence) &&
        d->dispatchId == proof.dispatchId &&
        d->commitSequence == static_cast<std::uint64_t>(proof.commitSequence) &&
        d->motionExecutionEpoch == static_cast<std::uint64_t>(epoch) &&
        d->motionSegmentId == static_cast<std::uint64_t>(proof.identity.segmentId) &&
        d->sourcePC == proof.sourcePC && d->sourceLineNumber == proof.sourceLineNumber &&
        d->queueTailTransactionSequence == proof.queueTailTransactionSequence &&
        d->queueTailBeforeFingerprint == proof.queueTailBeforeFingerprint &&
        d->queueTailCommittedFingerprint == proof.queueTailCommittedFingerprint &&
        d->axisMask == proof.queueTailAxisMask;
    if (!exactSource)
    {
        m_pathCoreLiveRetention.Reject(PathCoreLiveFenceReason::CAPTURE_REJECTED);
        m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
        QueuePathCoreLiveSummarySameThread();
        return;
    }
    const NCPathCoreCommandedChordSegmentCaptureCode captured =
        CapturePathCoreCurrentCommandedChordSegmentSameThread(m_pathCoreLiveCapture);
    if ((captured != NCPathCoreCommandedChordSegmentCaptureCode::CAPTURED_LINE_CHORD &&
        captured != NCPathCoreCommandedChordSegmentCaptureCode::CAPTURED_POINT_CHORD) ||
        m_pathCoreLiveCapture.localIdentity.geometryPublicationSequence != d->publicationSequence ||
        m_pathCoreLiveCapture.localIdentity.acceptedInputChainGeneration !=
        d->acceptedInputChainGeneration)
    {
        m_pathCoreLiveRetention.Reject(PathCoreLiveFenceReason::CAPTURE_REJECTED);
        m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
        QueuePathCoreLiveSummarySameThread();
        m_pathCoreLiveCapture.Clear();
        return;
    }
    if (!ValidatePathCoreLiveRetentionSameThread() ||
        m_motion.GetCurrentExecutionEpoch() != epoch)
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::EPOCH_CHANGED);
        return;
    }
    const NCPathCoreCommandedChordStoreCode admitted = m_pathCoreLiveRetention.Admit(
        m_pathCoreLiveCapture, scope, epoch, m_pathCoreLiveHandle);
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (admitted != NCPathCoreCommandedChordStoreCode::APPENDED &&
        admitted != NCPathCoreCommandedChordStoreCode::ALREADY_RETAINED)
    {
        QueuePathCoreLiveSummarySameThread();
        m_pathCoreLiveCapture.Clear();
        return;
    }
    const NCPathCoreCommandedChordStoreCode read = m_pathCoreLiveRetention.Read(
        scope, m_motion.GetCurrentExecutionEpoch(), m_pathCoreLiveHandle,
        m_pathCoreLiveReadback);
    m_pathCoreLiveBookkeeping.lastReadbackMatched =
        read == NCPathCoreCommandedChordStoreCode::VALUE_READ &&
        PathCoreLiveSegmentValuesEqual(m_pathCoreLiveCapture, m_pathCoreLiveReadback);
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (!m_pathCoreLiveBookkeeping.lastReadbackMatched)
    {
        // Read may already have fenced a concurrently advanced RT epoch.
        // Preserve that CLOSED scope result; only an OPEN data mismatch faults.
        if (m_pathCoreLiveStatus.state == NCPathCoreLiveRetentionState::OPEN)
            m_pathCoreLiveRetention.Reject(PathCoreLiveFenceReason::READBACK_REJECTED, read);
    }
    else if (m_pathCoreLiveBookkeeping.readbackCount <
        (std::numeric_limits<std::uint64_t>::max)())
        ++m_pathCoreLiveBookkeeping.readbackCount;
    (void)ValidatePathCoreLiveRetentionSameThread();
    m_pathCoreLiveRetention.Describe(m_pathCoreLiveStatus);
    if (m_pathCoreLiveStatus.state != NCPathCoreLiveRetentionState::OPEN)
        QueuePathCoreLiveSummarySameThread();
    else if (m_pathCoreLiveBookkeeping.lastReadbackMatched)
        (void)m_pathCoreExecutionLink.Bind(m_pathCoreLiveHandle,
            proof.identity, proof.ownerLease);
    m_pathCoreLiveCapture.Clear();
    m_pathCoreLiveReadback.Clear();
    m_pathCoreLiveHandle.Clear();
}

NC_PATH_CORE_NOINLINE
void NCManager::GetPathCoreLiveRetentionStatusSameThread(
    NCPathCoreLiveRetentionStatusV1& output) noexcept
{
    (void)ValidatePathCoreLiveRetentionSameThread();
    m_pathCoreLiveRetention.Describe(output);
}

NC_PATH_CORE_NOINLINE
NCPathCoreCommandedChordStoreCode NCManager::GetPathCoreLiveRetainedHandleSameThread(
    std::uint32_t ordinal, NCPathCoreCommandedChordStoreHandleV1& output) noexcept
{
    output.Clear();
    if (!ValidatePathCoreLiveRetentionSameThread())
        return NCPathCoreCommandedChordStoreCode::NOT_OPEN;
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    const NCPathCoreCommandedChordStoreCode result = m_pathCoreLiveRetention.GetHandleAtOrdinal(
        scope, m_motion.GetCurrentExecutionEpoch(), ordinal, output);
    if (!ValidatePathCoreLiveRetentionSameThread())
    {
        output.Clear();
        return NCPathCoreCommandedChordStoreCode::NOT_OPEN;
    }
    return result;
}

NC_PATH_CORE_NOINLINE
NCPathCoreCommandedChordStoreCode NCManager::ReadPathCoreLiveRetainedSegmentSameThread(
    const NCPathCoreCommandedChordStoreHandleV1& handle,
    NCPathCoreCommandedChordSegmentV1& output) noexcept
{
    output.Clear();
    if (!ValidatePathCoreLiveRetentionSameThread())
        return NCPathCoreCommandedChordStoreCode::NOT_OPEN;
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    const NCPathCoreCommandedChordStoreCode result = m_pathCoreLiveRetention.Read(
        scope, m_motion.GetCurrentExecutionEpoch(), handle, output);
    if (!ValidatePathCoreLiveRetentionSameThread())
    {
        output.Clear();
        return NCPathCoreCommandedChordStoreCode::NOT_OPEN;
    }
    return result;
}



NC_PATH_CORE_NOINLINE
NCPathCoreCommandedChordSnapshotCode NCManager::CapturePathCoreLiveSnapshotSameThread(
    const NCPathCoreCommandedChordSubpathV1& subpath,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& output) noexcept
{
    workspace.Clear();
    output.Clear();
    if (!ValidatePathCoreLiveRetentionSameThread())
        return NCPathCoreCommandedChordSnapshotCode::STORE_CLOSED;
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    const NCPathCoreCommandedChordSnapshotCode result =
        m_pathCoreLiveRetention.CaptureSnapshot(scope,
            m_motion.GetCurrentExecutionEpoch(), subpath, workspace, output);
    if (!ValidatePathCoreLiveRetentionSameThread())
    {
        workspace.Clear();
        output.Clear();
        return NCPathCoreCommandedChordSnapshotCode::STORE_CLOSED;
    }
    return result;
}


// BO helpers remain in the paired PathCore translation unit.
NC_PATH_CORE_NOINLINE
void NCManager::ClosePathCoreExecutionLinkSameThread(PathCoreLiveFenceReason reason) noexcept
{
    // Constructor-safe and data-only. Do not sample Motion here: constructor
    // configuration loading can fence before all runtime dependencies exist.
    if (!m_pathCoreExecutionLink.Close(static_cast<std::uint8_t>(reason))) return;
    m_pathCoreExecutionLink.Describe(m_pathCoreExecutionSummary);
    m_pathCoreExecutionBookkeeping.summaryPending = true;
}

NC_PATH_CORE_NOINLINE
void NCManager::FlushPathCoreExecutionSummarySameThread() noexcept
{
    if (!m_pathCoreExecutionBookkeeping.summaryPending) return;
    const NCPathCoreExecutionLinkStatusV1& s = m_pathCoreExecutionSummary;
    RtPrintf("[PCORE-EXEC] run=%llu life=%llu state=%u fault=%u bound=%u accepted=%u started=%u completed=%u failed=%u pending=%u retain=%u end=%u\n",
        static_cast<unsigned long long>(s.runToken),
        static_cast<unsigned long long>(s.lifetime),
        static_cast<unsigned int>(s.state),
        static_cast<unsigned int>(s.fault),
        static_cast<unsigned int>(s.bound),
        static_cast<unsigned int>(s.accepted),
        static_cast<unsigned int>(s.started),
        static_cast<unsigned int>(s.completed),
        static_cast<unsigned int>(s.failed),
        static_cast<unsigned int>(s.pending),
        static_cast<unsigned int>(s.retentionReason),
        static_cast<unsigned int>(s.endReason));
    m_pathCoreExecutionBookkeeping.summaryPending = false;
}

NC_PATH_CORE_NOINLINE
bool NCManager::ReadPathCoreLiveExecutionSameThread(
    const NCPathCoreCommandedChordStoreHandleV1& handle,
    NCPathCoreExecutionRecordV1& output) noexcept
{
    output.Clear();
    if (!ValidatePathCoreLiveRetentionSameThread()) return false;
    const bool found = m_pathCoreExecutionLink.Read(handle, output);
    if (!ValidatePathCoreLiveRetentionSameThread())
    {
        output.Clear();
        return false;
    }
    return found;
}

// BP-BEGIN
// BP is a detached data result prepared before the original fresh End sample.
NC_PATH_CORE_NOINLINE
void NCManager::DiscardPathCoreCompletedSnapshotSameThread() noexcept
{
    m_pathCoreCompletedBookkeeping.prepared = false;
    m_pathCoreCompletedBookkeeping.readable = false;
    m_pathCoreCompletedSnapshot.Clear();
    m_pathCoreCompletedWorkspace.Clear();
    m_pathCoreCompletedInfo = NCPathCoreCompletedSnapshotInfoV1{};
    m_pathCoreCompletedRecord.Clear();
    m_pathCoreCompletedSample.Clear();
    // A queued summary is historical evidence and is flushed at the old seam.
}

NC_PATH_CORE_NOINLINE
void NCManager::PreparePathCoreCompletedSnapshotSameThread() noexcept
{
    DiscardPathCoreCompletedSnapshotSameThread();
    m_pathCoreCompletedSummary = PathCoreCompletedSummary{};
    m_pathCoreCompletedSummary.code = NCPathCoreCompletedSnapshotCode::NOT_LIVE;
    const bool live = ValidatePathCoreLiveRetentionSameThread();
    m_pathCoreCompletedSummary.run = m_pathCoreLiveStatus.runToken;
    m_pathCoreCompletedSummary.lifetime = m_pathCoreLiveStatus.lifetime;
    if (!live) return;
    const NCPathCoreLiveRetentionScopeV1 scope = BuildPathCoreLiveScopeSameThread();
    m_pathCoreCompletedSummary.code = m_pathCoreCompletedSnapshot.Capture(
        m_pathCoreLiveRetention, scope, m_motion.GetCurrentExecutionEpoch(),
        m_pathCoreExecutionLink, m_pathCoreCompletedWorkspace);
    if (m_pathCoreCompletedSummary.code != NCPathCoreCompletedSnapshotCode::CAPTURED) return;
    // Mark hidden ownership before final validation, which may itself Fence.
    m_pathCoreCompletedBookkeeping.prepared = true;
    if (!ValidatePathCoreLiveRetentionSameThread())
    {
        DiscardPathCoreCompletedSnapshotSameThread();
        m_pathCoreCompletedSummary.code = NCPathCoreCompletedSnapshotCode::NOT_LIVE;
    }
}

NC_PATH_CORE_NOINLINE
void NCManager::PublishPathCoreCompletedSnapshotSameThread() noexcept
{
    // Called only after original Release + Fence + MarkFinalized succeeded.
    // Exercise the actual detached output after live geometry/BO reads ended.
    if (m_pathCoreCompletedBookkeeping.prepared &&
        m_pathCoreCompletedSummary.code == NCPathCoreCompletedSnapshotCode::CAPTURED)
    {
        m_pathCoreCompletedSnapshot.Describe(m_pathCoreCompletedInfo);
        m_pathCoreCompletedSummary.count = m_pathCoreCompletedInfo.count;
        for (std::uint32_t index = 1U; index <= m_pathCoreCompletedInfo.count; ++index)
        {
            if (m_pathCoreCompletedSnapshot.ReadPiece(index,
                m_pathCoreCompletedWorkspace.segment, m_pathCoreCompletedRecord,
                m_pathCoreCompletedWorkspace.piece, m_pathCoreCompletedWorkspace.info) !=
                NCPathCoreCompletedSnapshotCode::PIECE_READ)
            {
                m_pathCoreCompletedSummary.code = NCPathCoreCompletedSnapshotCode::QUERY_REJECTED;
                break;
            }
            ++m_pathCoreCompletedSummary.reads;
            for (std::uint32_t query = 0U; query < 3U; ++query)
            {
                const double sourceU = static_cast<double>(query) * 0.5;
                if (m_pathCoreCompletedSnapshot.EvaluatePiece(index, sourceU,
                    m_pathCoreCompletedSample, m_pathCoreCompletedRecord) !=
                    NCPathCoreCompletedSnapshotCode::EVALUATED)
                {
                    m_pathCoreCompletedSummary.code = NCPathCoreCompletedSnapshotCode::QUERY_REJECTED;
                    break;
                }
                ++m_pathCoreCompletedSummary.evaluations;
            }
            if (m_pathCoreCompletedSummary.code != NCPathCoreCompletedSnapshotCode::CAPTURED) break;
        }
        if (m_pathCoreCompletedSummary.code == NCPathCoreCompletedSnapshotCode::CAPTURED)
        {
            m_pathCoreCompletedBookkeeping.readable = true;
            m_pathCoreCompletedBookkeeping.prepared = false;
            m_pathCoreCompletedSummary.published = true;
        }
        else
            DiscardPathCoreCompletedSnapshotSameThread();
    }
    m_pathCoreCompletedSummary.pending = m_pathCoreCompletedSummary.run != 0ULL;
}

NC_PATH_CORE_NOINLINE
void NCManager::FlushPathCoreCompletedSummarySameThread() noexcept
{
    if (!m_pathCoreCompletedSummary.pending) return;
    const PathCoreCompletedSummary& s = m_pathCoreCompletedSummary;
    RtPrintf("[PCORE-DONE] run=%llu life=%llu code=%u pieces=%u read=%u eval=%u published=%u\n",
        static_cast<unsigned long long>(s.run),
        static_cast<unsigned long long>(s.lifetime),
        static_cast<unsigned int>(s.code), static_cast<unsigned int>(s.count),
        static_cast<unsigned int>(s.reads), static_cast<unsigned int>(s.evaluations),
        s.published ? 1U : 0U);
    m_pathCoreCompletedSummary.pending = false;
}

NC_PATH_CORE_NOINLINE
bool NCManager::GetPathCoreCompletedSnapshotInfoSameThread(
    NCPathCoreCompletedSnapshotInfoV1& output) const noexcept
{
    output = NCPathCoreCompletedSnapshotInfoV1{};
    if (!m_pathCoreCompletedBookkeeping.readable) return false;
    m_pathCoreCompletedSnapshot.Describe(output);
    return output.count != 0U;
}

NC_PATH_CORE_NOINLINE
NCPathCoreCompletedSnapshotCode NCManager::ReadPathCoreCompletedPieceSameThread(
    std::uint32_t index, NCPathCoreCommandedChordSegmentV1& segment,
    NCPathCoreExecutionRecordV1& execution,
    NCPathCoreCommandedChordSubpathPieceV1& piece,
    NCPathCoreCommandedChordSubpathInfoV1& info) const noexcept
{
    segment.Clear(); execution.Clear(); piece.Clear(); info.Clear();
    if (!m_pathCoreCompletedBookkeeping.readable)
        return NCPathCoreCompletedSnapshotCode::NOT_CAPTURED;
    return m_pathCoreCompletedSnapshot.ReadPiece(index, segment, execution, piece, info);
}

NC_PATH_CORE_NOINLINE
NCPathCoreCompletedSnapshotCode NCManager::EvaluatePathCoreCompletedPieceSameThread(
    std::uint32_t index, double sourceU,
    NCPathCoreCommandedChordPositionSampleV1& sample,
    NCPathCoreExecutionRecordV1& execution) const noexcept
{
    sample.Clear(); execution.Clear();
    if (!m_pathCoreCompletedBookkeeping.readable)
        return NCPathCoreCompletedSnapshotCode::NOT_CAPTURED;
    return m_pathCoreCompletedSnapshot.EvaluatePiece(index, sourceU, sample, execution);
}
// BP-END

// BQ-BEGIN
namespace
{
    bool BQFullIdentity(const MotionExecutionIdentity& a,
        const MotionExecutionIdentity& b) noexcept
    {
        return a.epoch == b.epoch && a.segmentId == b.segmentId &&
            a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
    bool BQSameReceipt(const MotionQueueTailCommitReceipt& a,
        const MotionQueueTailCommitReceipt& b) noexcept
    {
        return BQFullIdentity(a.identity, b.identity) && a.ownerLease.Matches(b.ownerLease) &&
            a.transactionSequence == b.transactionSequence && a.axisMask == b.axisMask &&
            a.beforeFingerprint == b.beforeFingerprint && a.committedFingerprint == b.committedFingerprint &&
            a.attempted == b.attempted && a.commandAccepted == b.commandAccepted &&
            a.commandedMCSCommitted == b.commandedMCSCommitted &&
            a.lastQueuedPulseCommitted == b.lastQueuedPulseCommitted &&
            a.rapidOverrideCommitted == b.rapidOverrideCommitted &&
            a.preservedOnReject == b.preservedOnReject && a.endpointExact == b.endpointExact &&
            a.captureBound == b.captureBound && a.accountingValid == b.accountingValid;
    }
    bool BQSameCommitSource(const NCProgramCommitSnapshot& a,
        const NCProgramCommitSnapshot& b) noexcept
    {
        return a.scope == b.scope && a.cacheGeneration == b.cacheGeneration &&
            a.frameId == b.frameId && a.sourcePC == b.sourcePC;
    }
}

NC_PATH_CORE_NOINLINE
NCPathCoreCommittedScopeV1 NCManager::BuildPathCoreCommittedScopeSameThread() const noexcept
{
    NCPathCoreCommittedScopeV1 scope{};
    scope.ownerTag = m_pathCoreCommittedScope.ownerTag;
    scope.runToken = m_pathCoreLiveBookkeeping.currentRunToken;
    scope.cacheGeneration = GetBaseProgramCache().GetGeneration();
    scope.programScope = static_cast<std::uint32_t>(GetBaseProgramScope());
    scope.operationMode = static_cast<std::uint32_t>(m_mode);
    scope.ownerLease = m_programMotionLease;
    return scope;
}

NC_PATH_CORE_NOINLINE
bool NCManager::IsPathCoreCommittedNativeConfigCurrentSameThread() noexcept
{
    if (!IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        const double& modulo = m_motion.GetAxisContext(static_cast<int>(axis)).rotaryModulo;
        if (std::memcmp(&modulo, &m_pathCoreCommittedRotaryModulo[axis], sizeof(double)) != 0)
            return false;
    }
    return true;
}

NC_PATH_CORE_NOINLINE
bool NCManager::ValidatePathCoreCommittedBaseScopeSameThread() noexcept
{
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    if (m_pathCoreCommittedStatus.state != NCPathCoreCommittedRunState::TRACKING) return false;
    const NCState state = m_state.load(std::memory_order_acquire);
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) || Homing.IsActive() ||
        !m_macroStack.empty() || !IsPathCoreCommittedNativeConfigCurrentSameThread())
    {
        m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::SCOPE);
        ClosePathCoreCommittedRunSameThread();
        return false;
    }
    const NCPathCoreCommittedScopeV1 scope = BuildPathCoreCommittedScopeSameThread();
    if (!m_pathCoreCommittedRun.CheckScope(scope))
    {
        ClosePathCoreCommittedRunSameThread();
        return false;
    }
    return true;
}

NC_PATH_CORE_NOINLINE
bool NCManager::ValidatePathCoreCommittedRunSameThread() noexcept
{
    if (m_pathCoreCommittedBookkeeping.pendingAdmission)
    {
        DrainPathCorePendingCommandedCaptureSameThread();
        // A saved receipt is not admitted/readable until every existing pending
        // safety/epoch request has cleared. No sleep, spin or NC control wait.
        if (m_pathCoreCommittedBookkeeping.pendingAdmission) return false;
    }
    if (!ValidatePathCoreCommittedBaseScopeSameThread()) return false;
    if (m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::SCOPE);
        ClosePathCoreCommittedRunSameThread();
        return false;
    }
    return true;
}

NC_PATH_CORE_NOINLINE
void NCManager::DrainPathCorePendingCommandedCaptureSameThread() noexcept
{
    if (!m_pathCoreCommittedBookkeeping.pendingAdmission) return;
    if (!ValidatePathCoreCommittedBaseScopeSameThread() ||
        m_pathCoreCommandedReceipt.transaction.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathCoreCommandedReceipt.transaction.ownerLease.Matches(m_programMotionLease))
    {
        m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::SCOPE);
        ClosePathCoreCommittedRunSameThread();
        return;
    }
    if (m_motion.HasPendingSafetyOrRecoveryRequests()) return;
    // Capture/commit/Ledger provenance was checked synchronously in Commit.
    // The sole pending slot cannot be reused by a later dispatch below.
    (void)m_pathCoreCommittedRun.Admit(m_pathCoreCommandedReceipt,
        m_pathCoreCommittedBookkeeping.dispatchId,
        m_pathCoreCommittedBookkeeping.pendingCommitSequence);
    m_pathCoreCommittedBookkeeping.pendingAdmission = false;
    m_pathCoreCommittedBookkeeping.pendingCommitSequence = 0ULL;
    m_pathCoreCommittedBookkeeping.dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    m_pathCoreCommandedReceipt.Clear();
}

NC_PATH_CORE_NOINLINE
void NCManager::ArmPathCoreCommittedRunSameThread() noexcept
{
    m_pathCoreReturnCursor = PathCoreReturnCursor{};
    m_pathCoreCommandedReceipt.Clear();
    m_pathCoreCommittedBookkeeping = PathCoreCommittedBookkeeping{};
    m_pathCoreCommittedScope.ownerTag = m_pathCoreLiveStatus.ownerTag;
    m_pathCoreCommittedScope = BuildPathCoreCommittedScopeSameThread();
    for (std::size_t axis = 0U; axis < 8U; ++axis)
        m_pathCoreCommittedRotaryModulo[axis] =
        m_motion.GetAxisContext(static_cast<int>(axis)).rotaryModulo;
    if (!m_pathCoreCommittedRun.Arm(m_pathCoreCommittedScope,
        m_lastConsumedMotionFeedbackSequence)) return;
    (void)ValidatePathCoreCommittedRunSameThread();
}

NC_PATH_CORE_NOINLINE
void NCManager::ClosePathCoreCommittedRunSameThread() noexcept
{
    InvalidatePathCoreReturnCursorSameThread();
    m_pathCoreCommittedBookkeeping.pendingAdmission = false;
    m_pathCoreCommittedBookkeeping.pendingCommitSequence = 0ULL;
    m_pathCoreCommandedReceipt.Clear();
    m_pathCoreCommittedBookkeeping.eligible = false;
    m_pathCoreCommittedBookkeeping.requested = false;
    m_pathCoreCommittedBookkeeping.dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    const bool alreadyClosed = m_pathCoreCommittedBookkeeping.closed;
    m_pathCoreCommittedRun.Close();
    if (m_pathCoreCommittedStatus.runToken != 0ULL && !alreadyClosed)
    {
        m_pathCoreCommittedBookkeeping.closed = true;
        m_pathCoreCommittedBookkeeping.published = false;
        m_pathCoreCommittedRun.Describe(m_pathCoreCommittedSummary);
        m_pathCoreCommittedBookkeeping.summaryPending = true;
    }
}

NC_PATH_CORE_NOINLINE
void NCManager::BeginPathCoreCommandedCaptureSameThread(const NCBlock& block,
    NCBlockDispatchId dispatchId) noexcept
{
    ObservePathCoreReturnDispatchSameThread(block);
    // A later dispatch must never overwrite an unadmitted committed receipt.
    // This revokes only BQ data if RT requests are still pending; NC continues.
    DrainPathCorePendingCommandedCaptureSameThread();
    if (m_pathCoreCommittedBookkeeping.pendingAdmission)
    {
        m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::SCOPE);
        ClosePathCoreCommittedRunSameThread();
        return;
    }
    // Clear even when G00 exits before reaching Motion or the block is unsupported.
    m_pathCoreCommandedReceipt.Clear();
    m_pathCoreCommittedBookkeeping.dispatchId = dispatchId;
    m_pathCoreCommittedBookkeeping.requested = false;
    // BU: P1 is local to a well-formed explicit G172, never ordinary G00 P.
    m_pathCoreCommittedBookkeeping.eligible = (!block.has('P') ||
        (block.gCode == 172 && IsPathCoreReturnBlockShapeValid(block))) && block.mCount == 0 &&
        !block.isEmpty && dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
        ValidatePathCoreCommittedRunSameThread() && m_state == NCState::RUN;
}

NC_PATH_CORE_NOINLINE
MotionCommandedEndpointReceiptV1* NCManager::GetPathCoreCommandedReceiptWorkspaceSameThread() noexcept
{
    // Called only by the ordinary Handle_G00 after the same block's modal commit.
    if (!m_pathCoreCommittedBookkeeping.eligible || !CoordSys.isAbsoluteMode ||
        m_pathCoreCommittedBookkeeping.dispatchId != m_currentExecutingBlockDispatchId ||
        !ValidatePathCoreCommittedRunSameThread() || m_state != NCState::RUN) return nullptr;
    m_pathCoreCommittedBookkeeping.requested = true;
    return &m_pathCoreCommandedReceipt;
}

NC_PATH_CORE_NOINLINE
void NCManager::CommitPathCoreCommandedCaptureSameThread(NCBlockDispatchId dispatchId,
    const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
    bool commitSucceeded, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
    int sourcePC, int sourceLineNumber) noexcept
{
    const bool requested = m_pathCoreCommittedBookkeeping.requested;
    m_pathCoreCommittedBookkeeping.requested = false;
    m_pathCoreCommittedBookkeeping.eligible = false;
    if (!ValidatePathCoreCommittedBaseScopeSameThread())
    {
        m_pathCoreCommandedReceipt.Clear();
        return;
    }
    if (capture.count == 0U && !capture.overflow && !m_pathCoreCommandedReceipt.valid)
    {
        m_pathCoreCommandedReceipt.Clear();
        return; // Modal and M00/M30-only blocks do not manufacture geometry.
    }
    if (!requested)
        m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::UNSUPPORTED);
    else if (capture.count != 1U || capture.overflow ||
        dispatchId != m_pathCoreCommittedBookkeeping.dispatchId || !commitSucceeded ||
        !commit.IsValid() || !ledgerFound || ledger.dispatchId != dispatchId ||
        !ledger.programCommitted || ledger.ncDispatchFailed || ledger.motionCaptureOverflow ||
        ledger.motionSegmentCount != 1U || ledger.sourceLineNumber != sourceLineNumber ||
        sourcePC < 0 || sourceLineNumber <= 0 || commit.sourcePC != sourcePC ||
        commit.scope != NCProgramScope::MEMORY || commit.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
        commit.cacheGeneration != m_pathCoreCommittedScope.cacheGeneration ||
        !BQSameCommitSource(commit, ledger.programTarget) ||
        !BQSameCommitSource(commit, ledger.programCommit) ||
        commit.sequence != ledger.programCommit.sequence)
        m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::LEDGER);
    else
    {
        const MotionProgramBlockSubmission& submission = capture.submissions[0U];
        const NCBlockMotionSegmentSnapshot& segment = ledger.motionSegments[0U];
        const MotionQueueTailCommitReceipt& receipt = m_pathCoreCommandedReceipt.transaction;
        if (!submission.producerAccepted || submission.immediateRejectReason != MotionRejectReason::NONE ||
            submission.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
            !segment.producerAccepted || segment.immediateRejectReason != MotionRejectReason::NONE ||
            !BQFullIdentity(submission.identity, receipt.identity) ||
            !BQFullIdentity(segment.identity, receipt.identity) ||
            receipt.identity.source != MotionCommandSource::NC_MEMORY ||
            receipt.identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) ||
            receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
            !receipt.ownerLease.Matches(m_programMotionLease) ||
            !BQSameReceipt(receipt, submission.queueTailReceipt))
            m_pathCoreCommittedRun.Fail(NCPathCoreCommittedRunFault::RECEIPT);
        else
        {
            // ABORTING has legitimately published its own epoch, which the RT
            // consumer may not have acknowledged yet. Keep exactly this proven
            // receipt; do not confuse that transient state with an invalid run.
            m_pathCoreCommittedBookkeeping.pendingCommitSequence = commit.sequence;
            m_pathCoreCommittedBookkeeping.pendingAdmission = true;
            DrainPathCorePendingCommandedCaptureSameThread();
            return;
        }
    }
    m_pathCoreCommandedReceipt.Clear();
    m_pathCoreCommittedBookkeeping.dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
}

NC_PATH_CORE_NOINLINE
void NCManager::PreparePathCoreCommittedRunSameThread() noexcept
{
    if (ValidatePathCoreCommittedRunSameThread() && m_pathCoreCommittedRun.Prepare())
        InvalidatePathCoreReturnCursorSameThread();
}

NC_PATH_CORE_NOINLINE
void NCManager::PublishPathCoreCommittedRunSameThread() noexcept
{
    m_pathCoreCommittedBookkeeping.reads = 0U;
    m_pathCoreCommittedBookkeeping.evaluations = 0U;
    m_pathCoreCommittedRun.CheckTransport(
        m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
        m_motionFeedbackSequenceGapCount == 0ULL);
    bool good = m_pathCoreCommittedRun.Publish();
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    for (std::uint32_t index = 0U; good && index < m_pathCoreCommittedStatus.count; ++index)
    {
        good = m_pathCoreCommittedRun.Read(index, m_pathCoreCommittedRecord);
        if (!good) break;
        ++m_pathCoreCommittedBookkeeping.reads;
        for (std::uint32_t point = 0U; good && point < 3U; ++point)
        {
            good = m_pathCoreCommittedRun.Evaluate(index,
                static_cast<double>(point) * 0.5, m_pathCoreCommittedSample);
            if (good) ++m_pathCoreCommittedBookkeeping.evaluations;
        }
    }
    if (!good) ClosePathCoreCommittedRunSameThread();
    m_pathCoreCommittedBookkeeping.published = good;
    m_pathCoreCommittedBookkeeping.closed = true;
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedSummary);
    m_pathCoreCommittedBookkeeping.summaryPending = m_pathCoreCommittedSummary.runToken != 0ULL;
}

NC_PATH_CORE_NOINLINE
void NCManager::FlushPathCoreCommittedRunSummarySameThread() noexcept
{
    if (!m_pathCoreCommittedBookkeeping.summaryPending) return;
    const NCPathCoreCommittedRunStatusV1& s = m_pathCoreCommittedSummary;
    RtPrintf("[PCORE-BQ] run=%llu state=%u fault=%u count=%u acc=%u start=%u done=%u fail=%u pending=%u epochs=%u seams=%u holds=%u resumes=%u read=%u eval=%u published=%u\n",
        static_cast<unsigned long long>(s.runToken), static_cast<unsigned int>(s.state),
        static_cast<unsigned int>(s.fault), static_cast<unsigned int>(s.count),
        static_cast<unsigned int>(s.accepted), static_cast<unsigned int>(s.started),
        static_cast<unsigned int>(s.completed), static_cast<unsigned int>(s.failed),
        static_cast<unsigned int>(s.pending), static_cast<unsigned int>(s.epochChanges),
        static_cast<unsigned int>(s.discontinuities), static_cast<unsigned int>(s.holds),
        static_cast<unsigned int>(s.resumes), static_cast<unsigned int>(m_pathCoreCommittedBookkeeping.reads),
        static_cast<unsigned int>(m_pathCoreCommittedBookkeeping.evaluations),
        m_pathCoreCommittedBookkeeping.published ? 1U : 0U);
    m_pathCoreCommittedBookkeeping.summaryPending = false;
}

NC_PATH_CORE_NOINLINE
bool NCManager::ValidatePathCorePublishedCommittedRunSameThread() noexcept
{
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    if (m_pathCoreCommittedStatus.state != NCPathCoreCommittedRunState::PUBLISHED) return false;
    // End releases the lease. Historical queries check source/config, not a new permit.
    if (Close_System_Com_flag || m_mode != NCOperationMode::MEMORY ||
        GetBaseProgramScope() != NCProgramScope::MEMORY || !m_macroStack.empty() || Homing.IsActive() ||
        GetBaseProgramCache().GetGeneration() != m_pathCoreCommittedScope.cacheGeneration ||
        !IsPathCoreCommittedNativeConfigCurrentSameThread())
    {
        ClosePathCoreCommittedRunSameThread();
        return false;
    }
    return true;
}

NC_PATH_CORE_NOINLINE
bool NCManager::ReadPathCoreCommittedRunPieceSameThread(std::uint32_t index,
    NCPathCoreCommittedRecordV1& output) noexcept
{
    output.Clear();
    if (!ValidatePathCorePublishedCommittedRunSameThread()) return false;
    return m_pathCoreCommittedRun.Read(index, output);
}

NC_PATH_CORE_NOINLINE
bool NCManager::EvaluatePathCoreCommittedRunPieceSameThread(std::uint32_t index, double u,
    NCPathCoreCommittedSampleV1& output) noexcept
{
    output.Clear();
    if (!ValidatePathCorePublishedCommittedRunSameThread()) return false;
    return m_pathCoreCommittedRun.Evaluate(index, u, output);
}
// BQ-END

// BR-BEGIN
namespace
{
    // FIX1: RtPrintf did not render floating conversions on the target.
    // Transport the exact binary64 payload through its proven integer path.
    NC_PATH_CORE_NOINLINE
        std::uint64_t PathCoreReturnDoubleBits(double value) noexcept
    {
        static_assert(sizeof(double) == sizeof(std::uint64_t), "BR requires binary64 storage.");
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

NC_PATH_CORE_NOINLINE
bool NCManager::IsPathCoreReturnBlockShapeValid(const NCBlock& block) noexcept
{
    const bool stepReturn = block.gCode == 172;
    if (block.isEmpty || block.isGoto || !block.hasG ||
        (block.gCode != 171 && !stepReturn && block.gCode != 173) ||
        block.gCount != 1 || block.gCodes[0] != block.gCode || block.mCount != 0 ||
        !block.has('F') || !std::isfinite(block.val('F')) ||
        block.val('F') <= 0.0 || block.val('F') > 100.0) return false;
    for (int address = 0; address < 26; ++address)
        if (block.hasParam[address] && address != ('F' - 'A') && address != ('N' - 'A') &&
            !(stepReturn && (address == ('L' - 'A') || address == ('P' - 'A'))))
            return false;
    if (stepReturn && block.has('L') &&
        (!std::isfinite(block.val('L')) || block.val('L') < 1.0 ||
            block.val('L') > 32.0 || std::floor(block.val('L')) != block.val('L')))
        return false;
    // BU opt-in belongs to the initial suffix selection only.
    if (stepReturn && block.has('P') &&
        (!block.has('L') || !std::isfinite(block.val('P')) || block.val('P') != 1.0))
        return false;
    return true;
}

NC_PATH_CORE_NOINLINE
void NCManager::RejectPathCoreReturnSameThread(std::uint32_t code, int alarmCode)
{
    m_pathCoreReturnSummary.code = code;
    m_pathCoreReturnSummary.pending = true;
    if (m_pathCoreReturnCommand == 172U || m_pathCoreReturnCommand == 173U)
    {
        m_pathCoreReturnCursor.forwardAvailable = false;
        m_pathCoreReturnCursor.state = PathCoreReturnCursorState::INVALID;
        LogPathCoreReturnCursorSameThread("REJECTED");
    }
    // Only the explicitly requested G171/G172 fails. Existing autonomous data
    // capture/overflow never calls this path or changes NC control.
    if (!AlarmManager::GetInstance().HasAlarm())
        AlarmManager::GetInstance().Trigger(alarmCode);
    ChangeState(NCState::ALARM);
}

// BT-BEGIN
NC_PATH_CORE_NOINLINE
void NCManager::LogPathCoreReturnCursorSameThread(const char* phase) const noexcept
{
    const PathCoreReturnCursor& c = m_pathCoreReturnCursor;
    RtPrintf(m_pathCoreReturnCommand == 173U ?
        "[PCORE-BV] command=173 run=%llu dispatch=%llu phase=%s state=%u code=%u sourceCount=%u requested=%u remaining=%u srcOrd=%u nextOrd=%u firstOrd=%u expectedCount=%u newPC=%d newEpoch=%u newSeg=%llu newSource=%u\n" :
        "[PCORE-BT] command=172 run=%llu dispatch=%llu phase=%s state=%u code=%u sourceCount=%u requested=%u remaining=%u srcOrd=%u nextOrd=%u firstOrd=%u expectedCount=%u newPC=%d newEpoch=%u newSeg=%llu newSource=%u\n",
        static_cast<unsigned long long>(m_pathCoreReturnSummary.run),
        static_cast<unsigned long long>(m_pathCoreReturnSummary.dispatch), phase,
        static_cast<unsigned int>(c.state), static_cast<unsigned int>(m_pathCoreReturnSummary.code),
        static_cast<unsigned int>(c.sourceCount), static_cast<unsigned int>(c.requested),
        static_cast<unsigned int>(c.remaining), static_cast<unsigned int>(c.selectedOrdinal),
        static_cast<unsigned int>(c.nextOrdinal), static_cast<unsigned int>(c.lowerBound),
        static_cast<unsigned int>(c.expectedCount), static_cast<int>(c.expectedIdentity.sourceBlockId),
        static_cast<unsigned int>(c.expectedIdentity.epoch),
        static_cast<unsigned long long>(c.expectedIdentity.segmentId),
        static_cast<unsigned int>(c.expectedIdentity.source));
}

NC_PATH_CORE_NOINLINE
void NCManager::InvalidatePathCoreReturnCursorSameThread() noexcept
{
    m_pathCoreReturnCursor.forwardAvailable = false;
    m_pathCoreReturnCursor.completionReady = false;
    if (m_pathCoreReturnCursor.state == PathCoreReturnCursorState::ACTIVE ||
        m_pathCoreReturnCursor.state == PathCoreReturnCursorState::PENDING)
    {
        m_pathCoreReturnCursor.state = PathCoreReturnCursorState::INVALID;
        LogPathCoreReturnCursorSameThread("INVALIDATED");
    }
    else if (m_pathCoreReturnCursor.state == PathCoreReturnCursorState::NEW)
        m_pathCoreReturnCursor.state = PathCoreReturnCursorState::INVALID;
}

NC_PATH_CORE_NOINLINE
void NCManager::ObservePathCoreReturnDispatchSameThread(const NCBlock& block) noexcept
{
    if (m_pathCoreReturnCursor.state == PathCoreReturnCursorState::NEW ||
        m_pathCoreReturnCursor.state == PathCoreReturnCursorState::INVALID ||
        (m_pathCoreReturnCursor.state == PathCoreReturnCursorState::EXHAUSTED &&
            !m_pathCoreReturnCursor.forwardAvailable)) return;
    if (NCGCodeSemantics::Contains(block, 172) || NCGCodeSemantics::Contains(block, 173)) return;
    // Only a standalone M00/M30 may intervene once a suffix has been chosen.
    bool allowedPauseOrEnd = !block.isEmpty && !block.isGoto && !block.hasG &&
        block.gCount == 0 && block.mCount == 1 &&
        (block.mCode[0] == 0 || block.mCode[0] == 30);
    for (int address = 0; address < 26; ++address)
        if (block.hasParam[address] && address != ('N' - 'A')) allowedPauseOrEnd = false;
    if (!allowedPauseOrEnd) InvalidatePathCoreReturnCursorSameThread();
}

NC_PATH_CORE_NOINLINE
bool NCManager::PreparePathCoreReturnCursorSourceSameThread(const NCBlock& block)
{
    if (block.gCode == 173) return PreparePathCoreAdvanceCursorSourceSameThread();
    PathCoreReturnCursor& c = m_pathCoreReturnCursor;
    const bool first = c.state == PathCoreReturnCursorState::NEW;
    if (c.forward || (first && !block.has('L')) ||
        (!first && (block.has('L') || block.has('P') || c.state != PathCoreReturnCursorState::ACTIVE ||
            c.run != m_pathCoreLiveBookkeeping.currentRunToken || c.remaining == 0U ||
            c.nextOrdinal < c.lowerBound || c.nextOrdinal > c.sourceCount)))
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    // The last retained row seals freshness. During a walk this must be the
    // previous NEW return receipt, never the older source row being revisited.
    if (!m_pathCoreCommittedRun.ReadLastCompleted(m_pathCoreCommittedRecord))
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    if (!first && (m_pathCoreCommittedStatus.count != c.expectedCount ||
        !BQFullIdentity(m_pathCoreCommittedRecord.receipt.transaction.identity, c.expectedIdentity)))
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(CoordSys.commandedMCS[axis]) ||
            CoordSys.commandedMCS[axis] != m_pathCoreCommittedRecord.receipt.endMCS[axis])
        {
            RejectPathCoreReturnSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
    }
    if (first)
    {
        const std::uint32_t requested = static_cast<std::uint32_t>(block.val('L'));
        const std::uint32_t count = m_pathCoreCommittedStatus.count;
        if (requested > count)
        {
            RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
        if (count + requested > 32U)
        {
            RejectPathCoreReturnSameThread(10U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
        const bool mixedAxes = block.has('P');
        const std::uint32_t axisMask = m_pathCoreCommittedRecord.receipt.transaction.axisMask;
        const std::uint32_t validMask = m_pathCoreCommittedRecord.receipt.validAxisMask;
        if (axisMask == 0U || (axisMask & ~0xffU) != 0U ||
            (validMask & ~0xffU) != 0U || (axisMask & validMask) != axisMask)
        {
            RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
        // Reuse the existing disjoint record workspace one row at a time.
        // Check EVERY selected source and target before the first Motion call.
        for (std::uint32_t index = count - requested; index < count; ++index)
        {
            if (!m_pathCoreCommittedRun.ReadCompleted(index, m_pathCoreCommittedRecord))
            {
                RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
                return false;
            }
            const MotionCommandedEndpointReceiptV1& source = m_pathCoreCommittedRecord.receipt;
            const std::uint32_t rowMask = source.transaction.axisMask;
            if (rowMask == 0U || (rowMask & ~0xffU) != 0U ||
                (rowMask & validMask) != rowMask ||
                (!mixedAxes && rowMask != axisMask) || source.validAxisMask != validMask)
            {
                RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
                return false;
            }
            for (std::size_t axis = 0U; axis < 8U; ++axis)
            {
                const bool selected = (rowMask & (1U << axis)) != 0U;
                const AxisContext& context = m_motion.GetAxisContext(static_cast<int>(axis));
                if (!std::isfinite(source.startMCS[axis]) || !std::isfinite(source.endMCS[axis]) ||
                    (selected && (!context.isExist || context.axisType != AxisType::LINEAR)) ||
                    (!selected && (mixedAxes ?
                        std::memcmp(&source.startMCS[axis], &source.endMCS[axis], sizeof(double)) != 0 :
                        (std::memcmp(&source.startMCS[axis], &CoordSys.commandedMCS[axis], sizeof(double)) != 0 ||
                            std::memcmp(&source.endMCS[axis], &CoordSys.commandedMCS[axis], sizeof(double)) != 0))))
                {
                    RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
                    return false;
                }
                if (selected && !CoordSys.IsTargetWithinSoftwareTravelLimit(context, source.startMCS[axis]))
                {
                    RejectPathCoreReturnSameThread(7U, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    return false;
                }
            }
        }
        c.run = m_pathCoreLiveBookkeeping.currentRunToken;
        c.sourceCount = count;
        c.requested = requested;
        c.remaining = requested;
        c.nextOrdinal = count;
        c.lowerBound = count - requested + 1U;
        c.expectedCount = count;
        c.sourceAxisMask = static_cast<std::uint8_t>(axisMask);
        c.sourceValidMask = static_cast<std::uint8_t>(validMask);
        c.mixedAxes = mixedAxes;
        c.state = PathCoreReturnCursorState::ACTIVE;
    }
    if (!m_pathCoreCommittedRun.ReadCompleted(c.nextOrdinal - 1U, m_pathCoreCommittedRecord))
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if ((!c.mixedAxes && m_pathCoreCommittedRecord.receipt.transaction.axisMask != c.sourceAxisMask) ||
        m_pathCoreCommittedRecord.receipt.validAxisMask != c.sourceValidMask)
    {
        RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    // In P1 mode this names the current row, never a union of axes.
    c.sourceAxisMask = static_cast<std::uint8_t>(m_pathCoreCommittedRecord.receipt.transaction.axisMask);
    c.selectedOrdinal = c.nextOrdinal;
    return true;
}

NC_PATH_CORE_NOINLINE
bool NCManager::CompletePathCoreReturnCursorSameThread()
{
    PathCoreReturnCursor& c = m_pathCoreReturnCursor;
    // NC's completion gate may poll the same successful wait callback again.
    // Only this exact dispatch may replay readiness; never consume a second step.
    const bool completedThisDispatch = c.completionReady &&
        m_pathCoreReturnCommand == (c.forward ? 173U : 172U) &&
        c.completedDispatch == m_pathCoreReturnSummary.dispatch &&
        c.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        (c.state == PathCoreReturnCursorState::ACTIVE ||
            c.state == PathCoreReturnCursorState::EXHAUSTED);
    if (!completedThisDispatch && (c.state != PathCoreReturnCursorState::PENDING ||
        c.run != m_pathCoreLiveBookkeeping.currentRunToken))
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    // Repeated readiness still obeys the existing G00/G171 Hold/standstill gate.
    if (IsFeedHoldActive() || !m_motion.IsGroupDone()) return false;
    if (completedThisDispatch) return true;
    if (!ValidatePathCoreCommittedRunSameThread())
    {
        // The proven receipt can still await the existing RT epoch ack.
        if (m_pathCoreCommittedBookkeeping.pendingAdmission && m_state == NCState::RUN)
            return false;
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    m_pathCoreCommittedRun.CheckTransport(
        m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
        m_motionFeedbackSequenceGapCount == 0ULL);
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    if (m_pathCoreCommittedStatus.state != NCPathCoreCommittedRunState::TRACKING ||
        m_pathCoreCommittedStatus.fault != NCPathCoreCommittedRunFault::NONE ||
        m_pathCoreCommittedStatus.failed != 0U)
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if (m_pathCoreCommittedStatus.count != c.expectedCount || c.remaining == 0U)
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    // GroupDone alone does not consume the pending step. All retained rows
    // must have Ledger-confirmed ACCEPTED and successful COMPLETED feedback.
    if (!m_pathCoreCommittedRun.ReadLastCompleted(m_pathCoreCommittedRecord))
    {
        // A terminal row with an error/reject payload cannot become successful
        // on a later poll. Only genuinely pending feedback may keep waiting.
        if (m_pathCoreCommittedStatus.pending == 0U)
            RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if (!BQFullIdentity(m_pathCoreCommittedRecord.receipt.transaction.identity, c.expectedIdentity))
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        if (CoordSys.commandedMCS[axis] != m_pathCoreCommittedRecord.receipt.endMCS[axis])
        {
            RejectPathCoreReturnSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
    }
    --c.remaining;
    if (c.forward) ++c.nextOrdinal;
    else --c.nextOrdinal;
    if (!c.forward && c.remaining == 0U) c.forwardAvailable = true;
    c.state = c.remaining == 0U ? PathCoreReturnCursorState::EXHAUSTED :
        PathCoreReturnCursorState::ACTIVE;
    c.completedDispatch = m_pathCoreReturnSummary.dispatch;
    c.completionReady = true;
    LogPathCoreReturnCursorSameThread("COMPLETED");
    return true;
}
// BT-END

// BV-BEGIN
NC_PATH_CORE_NOINLINE
bool NCManager::PreparePathCoreAdvanceCursorSourceSameThread()
{
    PathCoreReturnCursor& c = m_pathCoreReturnCursor;
    const bool first = !c.forward;
    if (c.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        c.sourceCount == 0U || c.sourceCount > 32U || c.requested == 0U ||
        c.requested > c.sourceCount || c.lowerBound != c.sourceCount - c.requested + 1U ||
        (first && (c.state != PathCoreReturnCursorState::EXHAUSTED ||
            c.remaining != 0U || !c.forwardAvailable)) ||
        (!first && (c.state != PathCoreReturnCursorState::ACTIVE || c.remaining == 0U ||
            c.nextOrdinal < c.lowerBound || c.nextOrdinal > c.sourceCount)))
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if (!m_pathCoreCommittedRun.ReadLastCompleted(m_pathCoreCommittedRecord))
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    if (m_pathCoreCommittedStatus.count != c.expectedCount ||
        !BQFullIdentity(m_pathCoreCommittedRecord.receipt.transaction.identity, c.expectedIdentity))
    {
        RejectPathCoreReturnSameThread(11U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(CoordSys.commandedMCS[axis]) ||
            CoordSys.commandedMCS[axis] != m_pathCoreCommittedRecord.receipt.endMCS[axis])
        {
            RejectPathCoreReturnSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
    }
    if (first)
    {
        // Retreat admission is unchanged. Reserve the whole optional forward
        // traversal now, before issuing its first command.
        if (m_pathCoreCommittedStatus.count + c.requested > 32U)
        {
            RejectPathCoreReturnSameThread(10U, AlarmManager::G_Code_Invalid_parameter);
            return false;
        }
        for (std::uint32_t index = c.lowerBound - 1U; index < c.sourceCount; ++index)
        {
            if (!m_pathCoreCommittedRun.ReadCompleted(index, m_pathCoreCommittedRecord))
            {
                RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
                return false;
            }
            const MotionCommandedEndpointReceiptV1& source = m_pathCoreCommittedRecord.receipt;
            const std::uint32_t mask = source.transaction.axisMask;
            if (mask == 0U || (mask & ~0xffU) != 0U ||
                (source.validAxisMask & ~0xffU) != 0U ||
                (mask & source.validAxisMask) != mask ||
                source.validAxisMask != c.sourceValidMask ||
                (!c.mixedAxes && mask != c.sourceAxisMask))
            {
                RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
                return false;
            }
            for (std::size_t axis = 0U; axis < 8U; ++axis)
            {
                const bool selected = (mask & (1U << axis)) != 0U;
                const AxisContext& context = m_motion.GetAxisContext(static_cast<int>(axis));
                if (!std::isfinite(source.startMCS[axis]) || !std::isfinite(source.endMCS[axis]) ||
                    (selected && (!context.isExist || context.axisType != AxisType::LINEAR)) ||
                    (!selected && (c.mixedAxes ?
                        std::memcmp(&source.startMCS[axis], &source.endMCS[axis], sizeof(double)) != 0 :
                        (std::memcmp(&source.startMCS[axis], &CoordSys.commandedMCS[axis], sizeof(double)) != 0 ||
                            std::memcmp(&source.endMCS[axis], &CoordSys.commandedMCS[axis], sizeof(double)) != 0))))
                {
                    RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
                    return false;
                }
                if (selected && !CoordSys.IsTargetWithinSoftwareTravelLimit(context, source.endMCS[axis]))
                {
                    RejectPathCoreReturnSameThread(7U, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    return false;
                }
            }
        }
        c.forward = true;
        c.forwardAvailable = false;
        c.remaining = c.requested;
        c.nextOrdinal = c.lowerBound;
        c.state = PathCoreReturnCursorState::ACTIVE;
    }
    if (!m_pathCoreCommittedRun.ReadCompleted(c.nextOrdinal - 1U, m_pathCoreCommittedRecord))
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if ((!c.mixedAxes && m_pathCoreCommittedRecord.receipt.transaction.axisMask != c.sourceAxisMask) ||
        m_pathCoreCommittedRecord.receipt.validAxisMask != c.sourceValidMask)
    {
        RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    c.sourceAxisMask = static_cast<std::uint8_t>(m_pathCoreCommittedRecord.receipt.transaction.axisMask);
    c.selectedOrdinal = c.nextOrdinal;
    return true;
}
// BV-END

// BU-BEGIN
NC_PATH_CORE_NOINLINE
bool NCManager::CheckPathCoreMixedReturnOutputSameThread() noexcept
{
    // Borrow the same producer output. No receipt copy or new RT observer.
    const MotionCommandedEndpointReceiptV1& output = m_pathCoreCommandedReceipt;
    const PathCoreReturnSummary& s = m_pathCoreReturnSummary;
    std::uint32_t outsideMovedMask = 0U;
    bool finite = true;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        finite = finite && std::isfinite(output.startMCS[axis]) && std::isfinite(output.endMCS[axis]);
        if ((s.sourceAxisMask & (1U << axis)) == 0U &&
            std::memcmp(&output.startMCS[axis], &output.endMCS[axis], sizeof(double)) != 0)
            outsideMovedMask |= (1U << axis);
    }
    RtPrintf("[PCORE-BU] run=%llu dispatch=%llu mode=1 srcOrd=%u sourceMask=%u newMask=%u validMask=%u outsideMovedMask=%u savedOutsideDiffMask=%u outputValid=%u finite=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned int>(s.sourceOrdinal), static_cast<unsigned int>(s.sourceAxisMask),
        static_cast<unsigned int>(s.newAxisMask), static_cast<unsigned int>(s.newValidMask),
        static_cast<unsigned int>(outsideMovedMask), static_cast<unsigned int>(s.otherEndMask),
        output.valid ? 1U : 0U, finite ? 1U : 0U);
    return output.valid && finite && outsideMovedMask == 0U;
}
// BU-END

NC_PATH_CORE_NOINLINE
WaitConditionFunc NCManager::StartPathCoreReturnSameThread(const NCBlock& block)
{
    FlushPathCoreReturnSummarySameThread();
    m_pathCoreReturnCommand = static_cast<std::uint16_t>(block.gCode);
    const bool forwardReturn = block.gCode == 173;
    const bool stepReturn = block.gCode == 172 || forwardReturn;
    if (stepReturn)
    {
        m_pathCoreReturnCursor.completionReady = false;
        m_pathCoreReturnCursor.completedDispatch = 0ULL;
    }
    m_pathCoreReturnSummary = PathCoreReturnSummary{};
    m_pathCoreReturnSummary.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_pathCoreReturnSummary.dispatch = m_currentExecutingBlockDispatchId;
    m_pathCoreReturnSummary.rapidPercent = block.val('F');
    if (!IsPathCoreReturnBlockShapeValid(block))
    {
        RejectPathCoreReturnSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        !CoordSys.isAbsoluteMode || m_isG66Active ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathCoreCommittedBookkeeping.dispatchId != m_currentExecutingBlockDispatchId ||
        !m_pathCoreCommittedBookkeeping.eligible ||
        !ValidatePathCoreCommittedRunSameThread())
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    m_pathCoreCommittedRun.CheckTransport(
        m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
        m_motionFeedbackSequenceGapCount == 0ULL);
    if (stepReturn)
    {
        if (!PreparePathCoreReturnCursorSourceSameThread(block)) return nullptr;
    }
    else if (!m_pathCoreCommittedRun.ReadLastCompleted(m_pathCoreCommittedRecord))
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    m_pathCoreCommittedRun.Describe(m_pathCoreCommittedStatus);
    m_pathCoreReturnSummary.sourceOrdinal = static_cast<std::uint16_t>(stepReturn ?
        m_pathCoreReturnCursor.selectedOrdinal : m_pathCoreCommittedStatus.count);
    m_pathCoreReturnSummary.source = m_pathCoreCommittedRecord.receipt.transaction.identity;
    m_pathCoreReturnSummary.seamsBefore = m_pathCoreCommittedStatus.discontinuities;
    m_pathCoreReturnSummary.sourceFlags = m_pathCoreCommittedRecord.boundaryFlags;
    // Reserve room for the new return command's own captured receipt.
    if (m_pathCoreCommittedStatus.count >= 32U)
    {
        RejectPathCoreReturnSameThread(10U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const MotionCommandedEndpointReceiptV1& source = m_pathCoreCommittedRecord.receipt;
    const auto& targetMCS = forwardReturn ? source.endMCS : source.startMCS;
    const auto& sourceBaseline = forwardReturn ? source.startMCS : source.endMCS;
    m_pathCoreReturnSummary.sourceValidMask = static_cast<std::uint8_t>(source.validAxisMask);
    // BS: the captured programmed mask names native axis indices. Every
    // selected axis must still exist and be linear under the validated config.
    const std::uint32_t sourceMask = source.transaction.axisMask;
    m_pathCoreReturnSummary.sourceAxisMask = static_cast<std::uint8_t>(sourceMask);
    if (sourceMask == 0U || (sourceMask & ~0xffU) != 0U ||
        (sourceMask & source.validAxisMask) != sourceMask)
    {
        RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        const bool selected = (sourceMask & (1U << axis)) != 0U;
        const AxisContext& context = m_motion.GetAxisContext(static_cast<int>(axis));
        if (!std::isfinite(source.startMCS[axis]) || !std::isfinite(source.endMCS[axis]) ||
            (!selected && source.startMCS[axis] != source.endMCS[axis]) ||
            (selected && (!context.isExist || context.axisType != AxisType::LINEAR)))
        {
            RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        // Recheck unselected slots before every submission. BT retains the
        // frozen baseline contract. BU P1 requires the saved row itself to
        // leave these slots unchanged; Motion chooses the current baseline.
        if (stepReturn && !selected && (m_pathCoreReturnCursor.mixedAxes ?
            std::memcmp(&source.startMCS[axis], &source.endMCS[axis], sizeof(double)) != 0 :
            (std::memcmp(&source.startMCS[axis], &CoordSys.commandedMCS[axis], sizeof(double)) != 0 ||
                std::memcmp(&source.endMCS[axis], &CoordSys.commandedMCS[axis], sizeof(double)) != 0)))
        {
            RejectPathCoreReturnSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        // This is a source freshness guard, not an atomic producer start seal.
        // Motion still chooses its own baseline inside the existing transaction.
        if (!stepReturn && CoordSys.commandedMCS[axis] != source.endMCS[axis])
        {
            RejectPathCoreReturnSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    m_pathCoreReturnSummary.fromX = sourceBaseline[0U];
    m_pathCoreReturnSummary.targetX = targetMCS[0U];
    if (!m_motion.IsGroupDone() || m_motion.GetAxisCommandMailboxDepth() != 0U ||
        m_motion.GetAxisCommandResultDepth() != 0U || m_motion.GetCommandIngressSize() != 0U ||
        m_motion.GetCommandReplaySize() != 0U || m_motion.GetMotionFeedbackDepth() != 0U ||
        m_motion.GetMotionFeedbackProducerNoticeDepth() != 0U)
    {
        RejectPathCoreReturnSameThread(6U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // Check the complete selected target set before any producer submission.
    std::uint32_t axisCount = 0U;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        if ((sourceMask & (1U << axis)) == 0U) continue;
        const AxisContext& context = m_motion.GetAxisContext(static_cast<int>(axis));
        if (!CoordSys.IsTargetWithinSoftwareTravelLimit(context, targetMCS[axis]))
        {
            RejectPathCoreReturnSameThread(7U, AlarmManager::PROGRAMMED_OVER_TRAVEL);
            return nullptr;
        }
        ++axisCount;
    }
    m_pathCoreReturnSummary.axisCount = static_cast<std::uint8_t>(axisCount);
    MotionCommandedEndpointReceiptV1* output = GetPathCoreCommandedReceiptWorkspaceSameThread();
    if (output == nullptr)
    {
        RejectPathCoreReturnSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // Both capacities were allocated for eight slots at construction. Shrink
    // and regrow only within that capacity; never allocate in this request.
    m_pathCoreReturnAxes.resize(axisCount);
    m_pathCoreReturnTargets.resize(axisCount);
    std::uint32_t slot = 0U;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        if ((sourceMask & (1U << axis)) == 0U) continue;
        m_pathCoreReturnAxes[slot] = static_cast<int>(axis);
        m_pathCoreReturnTargets[slot] = targetMCS[axis];
        ++slot;
    }
    const bool accepted = m_motion.TryG00MoveTransactionalTail(
        m_pathCoreReturnAxes, m_pathCoreReturnTargets, BufferMode::ABORTING,
        MotionCommandPathMode::EXACT_STOP, block.val('F') / 100.0,
        CoordSys.commandedMCS, output);
    if (!accepted)
    {
        RejectPathCoreReturnSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    // The new producer assigns the new tuple. Never submit the saved identity.
    m_pathCoreReturnSummary.submitted = output->transaction.identity;
    m_pathCoreReturnSummary.code = output->valid ? 1U : 9U;
    m_pathCoreReturnSummary.pending = true;
    // FIX1 diagnostic copy only: never alter the accepted command, receipt,
    // commanded tail, BQ seam definition, qualification or completion gates.
    m_pathCoreReturnSummary.outputValid = output->valid;
    m_pathCoreReturnSummary.newValidMask = static_cast<std::uint8_t>(output->validAxisMask);
    m_pathCoreReturnSummary.newStartX = output->startMCS[0U];
    m_pathCoreReturnSummary.newEndX = output->endMCS[0U];
    m_pathCoreReturnSummary.newAxisMask =
        static_cast<std::uint8_t>(output->transaction.axisMask);
    m_pathCoreReturnSummary.targetExact = output->valid &&
        output->transaction.axisMask == sourceMask;
    if (output->valid)
    {
        for (std::size_t axis = 0U; axis < 8U; ++axis)
        {
            const std::uint8_t bit = static_cast<std::uint8_t>(1U << axis);
            const bool byteDifference = std::memcmp(
                &sourceBaseline[axis], &output->startMCS[axis], sizeof(double)) != 0;
            const bool selected = (sourceMask & bit) != 0U;
            const bool otherEndDifference = !selected && std::memcmp(
                &source.endMCS[axis], &output->endMCS[axis], sizeof(double)) != 0;
            if (selected)
            {
                const bool exact = std::memcmp(
                    &targetMCS[axis], &output->endMCS[axis], sizeof(double)) == 0;
                if (!exact)
                {
                    m_pathCoreReturnSummary.targetMismatchMask |= bit;
                    m_pathCoreReturnSummary.targetExact = false;
                }
                // Every programmed axis supplies target evidence, even when
                // no seam was observed. Borrow the two existing workspaces.
                RtPrintf(forwardReturn ?
                    "[PCORE-BV-AXIS] run=%llu dispatch=%llu axis=%u savedStartBits=%llu savedEndBits=%llu newStartBits=%llu newEndBits=%llu targetExact=%u\n" :
                    "[PCORE-BS-AXIS] run=%llu dispatch=%llu axis=%u savedStartBits=%llu savedEndBits=%llu newStartBits=%llu newEndBits=%llu targetExact=%u\n",
                    static_cast<unsigned long long>(m_pathCoreReturnSummary.run),
                    static_cast<unsigned long long>(m_pathCoreReturnSummary.dispatch),
                    static_cast<unsigned int>(axis),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(source.startMCS[axis])),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(source.endMCS[axis])),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(output->startMCS[axis])),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(output->endMCS[axis])),
                    exact ? 1U : 0U);
            }
            if (byteDifference) m_pathCoreReturnSummary.startByteMask |= bit;
            if (sourceBaseline[axis] != output->startMCS[axis])
                m_pathCoreReturnSummary.startNumericMask |= bit;
            if (otherEndDifference) m_pathCoreReturnSummary.otherEndMask |= bit;
            if (byteDifference || otherEndDifference)
            {
                // At most eight lines per explicit G171, in the NC task while
                // both existing workspaces are still valid. No RT observer.
                RtPrintf(forwardReturn ?
                    "[PCORE-BV-SEAM] run=%llu dispatch=%llu axis=%u sourceStartBits=%llu newStartBits=%llu newEndBits=%llu\n" :
                    "[PCORE-BR-AXIS] run=%llu dispatch=%llu axis=%u prevEndBits=%llu newStartBits=%llu newEndBits=%llu\n",
                    static_cast<unsigned long long>(m_pathCoreReturnSummary.run),
                    static_cast<unsigned long long>(m_pathCoreReturnSummary.dispatch),
                    static_cast<unsigned int>(axis),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(sourceBaseline[axis])),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(output->startMCS[axis])),
                    static_cast<unsigned long long>(PathCoreReturnDoubleBits(output->endMCS[axis])));
            }
        }
    }
    if (stepReturn)
    {
        // A submitted command is not a completed cursor step. If its export
        // cannot prove the selected targets, fail through the existing Alarm
        // boundary and never attempt another submission for this step.
        const bool outsideValid = m_pathCoreReturnCursor.mixedAxes ?
            CheckPathCoreMixedReturnOutputSameThread() : m_pathCoreReturnSummary.otherEndMask == 0U;
        if (!output->valid || !m_pathCoreReturnSummary.targetExact || !outsideValid ||
            output->validAxisMask != m_pathCoreReturnCursor.sourceValidMask)
        {
            RejectPathCoreReturnSameThread(9U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        m_pathCoreReturnCursor.expectedIdentity = output->transaction.identity;
        ++m_pathCoreReturnCursor.expectedCount;
        m_pathCoreReturnCursor.state = PathCoreReturnCursorState::PENDING;
        LogPathCoreReturnCursorSameThread("SUBMITTED");
        return [](NCManager* nc) {
            return nc->CompletePathCoreReturnCursorSameThread();
        };
    }
    // Match the existing ordinary G00 waiting contract. Capture/Ledger and
    // original completion/end gates own execution acknowledgement and PC.
    return [](NCManager* nc) {
        return !nc->IsFeedHoldActive() && nc->m_motion.IsGroupDone();
    };
}

NC_PATH_CORE_NOINLINE
void NCManager::FlushPathCoreReturnSummarySameThread() noexcept
{
    if (!m_pathCoreReturnSummary.pending) return;
    const PathCoreReturnSummary& s = m_pathCoreReturnSummary;
    RtPrintf("[PCORE-RETURN] command=%u run=%llu dispatch=%llu code=%u\n",
        static_cast<unsigned int>(m_pathCoreReturnCommand),
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned int>(s.code));
    RtPrintf(m_pathCoreReturnCommand == 173U ?
        "[PCORE-BV-MASK] run=%llu dispatch=%llu sourceMask=%u newMask=%u axes=%u targetMismatchMask=%u targetExact=%u otherEndMask=%u\n" :
        "[PCORE-BS] run=%llu dispatch=%llu sourceMask=%u newMask=%u axes=%u targetMismatchMask=%u targetExact=%u otherEndMask=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned int>(s.sourceAxisMask), static_cast<unsigned int>(s.newAxisMask),
        static_cast<unsigned int>(s.axisCount), static_cast<unsigned int>(s.targetMismatchMask),
        s.targetExact ? 1U : 0U, static_cast<unsigned int>(s.otherEndMask));
    // Retain the BR diagnostic names for compatibility. BR-GEO is the X-slot
    // view; BS-AXIS carries each selected axis, and targetExact covers the set.
    RtPrintf(m_pathCoreReturnCommand == 173U ?
        "[PCORE-BV-IDENTITY] run=%llu dispatch=%llu code=%u srcOrd=%u srcPC=%d srcEpoch=%u srcSeg=%llu newPC=%d newEpoch=%u newSeg=%llu\n" :
        "[PCORE-BR] run=%llu dispatch=%llu code=%u srcOrd=%u srcPC=%d srcEpoch=%u srcSeg=%llu newPC=%d newEpoch=%u newSeg=%llu\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned int>(s.code), static_cast<unsigned int>(s.sourceOrdinal),
        static_cast<int>(s.source.sourceBlockId), static_cast<unsigned int>(s.source.epoch),
        static_cast<unsigned long long>(s.source.segmentId),
        static_cast<int>(s.submitted.sourceBlockId), static_cast<unsigned int>(s.submitted.epoch),
        static_cast<unsigned long long>(s.submitted.segmentId));
    RtPrintf(m_pathCoreReturnCommand == 173U ?
        "[PCORE-BV-GEO] run=%llu dispatch=%llu fromXBits=%llu targetXBits=%llu newStartXBits=%llu newEndXBits=%llu FBits=%llu outputValid=%u targetExact=%u\n" :
        "[PCORE-BR-GEO] run=%llu dispatch=%llu fromXBits=%llu targetXBits=%llu newStartXBits=%llu newEndXBits=%llu FBits=%llu outputValid=%u targetExact=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(PathCoreReturnDoubleBits(s.fromX)),
        static_cast<unsigned long long>(PathCoreReturnDoubleBits(s.targetX)),
        static_cast<unsigned long long>(PathCoreReturnDoubleBits(s.newStartX)),
        static_cast<unsigned long long>(PathCoreReturnDoubleBits(s.newEndX)),
        static_cast<unsigned long long>(PathCoreReturnDoubleBits(s.rapidPercent)),
        s.outputValid ? 1U : 0U, s.targetExact ? 1U : 0U);
    RtPrintf(m_pathCoreReturnCommand == 173U ?
        "[PCORE-BV-JOIN] run=%llu dispatch=%llu seamsBefore=%u sourceFlags=%u startByteMask=%u startNumericMask=%u otherEndMask=%u srcValidMask=%u newValidMask=%u\n" :
        "[PCORE-BR-JOIN] run=%llu dispatch=%llu seamsBefore=%u sourceFlags=%u startByteMask=%u startNumericMask=%u otherEndMask=%u srcValidMask=%u newValidMask=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned int>(s.seamsBefore), static_cast<unsigned int>(s.sourceFlags),
        static_cast<unsigned int>(s.startByteMask), static_cast<unsigned int>(s.startNumericMask),
        static_cast<unsigned int>(s.otherEndMask), static_cast<unsigned int>(s.sourceValidMask),
        static_cast<unsigned int>(s.newValidMask));
    m_pathCoreReturnSummary.pending = false;
}
// BR-END

#undef NC_PATH_CORE_NOINLINE
