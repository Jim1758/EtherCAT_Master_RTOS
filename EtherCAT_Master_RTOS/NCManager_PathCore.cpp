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

#undef NC_PATH_CORE_NOINLINE
