#pragma once
#include "MechanicalCompensationLifecycle.h"
#include "MechanicalCompensationControl.h"
#include "MechanicalCompensationSendContract.h"
#include <atomic>
#include <vector>

struct AxisContext;
struct AxisCommand;

// PBC-3P: sealed physical X admits explicit bounded calibration profiles.
// P adds acknowledged directional pitch with asymmetric backlash; general scope stays LOCKED.
// Zero identity is established from immutable configuration AND current state;
// it is not an authorization to clear uncertain output or to claim unique HOME.
// Changing this constant alone does not establish those contracts.
class CompensationEngine
{
public:
    static constexpr bool MotionIntegrationReleased = false;
    static constexpr bool ZeroOnlyXIntegrationReleased = true;
    static constexpr bool PitchOnlyXIntegrationReleased = true;
    static constexpr bool BacklashOnlyXIntegrationReleased = true;
    static constexpr bool CombinedXIntegrationReleased = true;
    static constexpr bool AsymmetricCombinedXIntegrationReleased = true;
    static constexpr bool DirectionalPitchOnlyXIntegrationReleased = true;
    static constexpr bool DirectionalCombinedXIntegrationReleased = true;
    CompensationEngine() = default;

    bool InitAxisCompensation(int axisIndex, bool enBacklash, double b_Pos,
        double b_Neg, double b_Speed, bool enPitch, double startPos,
        double step, double p_Speed);
    bool SetAxisPolicy(int axisIndex, const pbc::Policy& policy);
    bool SetPitchTables(int axisIndex, const std::vector<double>& pos,
        const std::vector<double>& neg);
    bool SetPitchTablePos(int axisIndex, const std::vector<double>& errors);
    bool SetPitchTableNeg(int axisIndex, const std::vector<double>& errors);

    // Boot thread only. All axis candidates validate before any commit.
    // Success seals configuration and, for any admitted X profile, publishes a
    // finite numerical bootstrap frame before WCS/NC reads. No reference or HOME.
    bool FinalizeConfiguration(std::vector<AxisContext>& axes, pbc::Diagnostic& d);
    bool HasEnabledPitch() const noexcept;
    bool IsSealed() const noexcept { return m_sealed.load(std::memory_order_acquire); }
    unsigned EnabledAxisCount() const noexcept;
    unsigned CoordinateContractChecks() const noexcept { return m_coordinateContractChecks; }
    unsigned LifecycleContractChecks() const noexcept { return m_lifecycleContractChecks; }

    // True only for this exact sealed physical X with at least one enable flag,
    // zero enabled calibration, and unchanged AxisContext configuration fields.
    // Neither predicate authorizes output or clears the separate send quarantine.
    bool IsXZeroOnlyConfiguration(const AxisContext& axis) const noexcept;
    // Immutable boot classification for cross-thread admission guards. This is
    // NOT current coordinate validity, a motion authority, or a recovery permit.
    bool IsXZeroOnlyAdmitted() const noexcept { return IsSealed() && m_xZeroOnlyAdmitted; }
    // Canonical sealed OFF X only; mutated enabled-zero flags/frame cannot qualify.
    bool IsXConfiguredDisabledIdentity(const AxisContext& axis) const noexcept;
    // Before first reference: only the exact validated boot zero frame is accepted.
    // After reference: a valid enabled zero frame must match the committed model.
    // OutputUncertain is never an identity, even for a zero-only configuration.
    bool IsXZeroOnlyCoordinateIdentity(const AxisContext& axis) const noexcept;
    // Exact startup-alignment seam only, before any reference/proposal. Updates
    // only the numerical zero frame after Motion aligned its command scalars.
    // Caller retains raw-feedback/owner/quarantine authority; no HOME is minted.
    bool RefreshXZeroOnlyBootstrapFrame(AxisContext& axis, pbc::Diagnostic& d) noexcept;

    // PBC-3K, L, M, N, O and P are separate immutable profiles; none masquerades as a
    // zero-only calibration. These queries do not authorize NIC output.
    bool IsXPitchOnlyAdmitted() const noexcept { return IsSealed() && m_xPitchOnlyAdmitted; }
    bool IsXPitchOnlyConfiguration(const AxisContext& axis) const noexcept;
    // L is backlash-only with a complete dormant zero pitch pair. Magnitudes
    // name +/- branch offsets: reversal distance is Positive + Negative.
    bool IsXBacklashOnlyAdmitted() const noexcept { return IsSealed() && m_xBacklashOnlyAdmitted; }
    bool IsXBacklashOnlyConfiguration(const AxisContext& axis) const noexcept;
    // M requires both nonzero calibrations, exact paired pitch tables and a
    // per-row abs(pitch) + backlash bound. A zero sum is not zero calibration.
    bool IsXCombinedAdmitted() const noexcept { return IsSealed() && m_xCombinedAdmitted; }
    bool IsXCombinedConfiguration(const AxisContext& axis) const noexcept;
    // N retains M's exact same-PN pitch rule, but requires unequal positive
    // backlash magnitudes. Bound abs(pitch) + max(positive, negative) per row.
    bool IsXAsymmetricCombinedAdmitted() const noexcept { return IsSealed() && m_xAsymmetricCombinedAdmitted; }
    bool IsXAsymmetricCombinedConfiguration(const AxisContext& axis) const noexcept;
    // O requires distinct signed pitch tables, with backlash OFF and both
    // dormant backlash magnitudes zero. Both tables are bounded separately.
    bool IsXDirectionalPitchOnlyAdmitted() const noexcept { return IsSealed() && m_xDirectionalPitchOnlyAdmitted; }
    bool IsXDirectionalPitchOnlyConfiguration(const AxisContext& axis) const noexcept;
    // P requires distinct PN tables, unequal positive backlash magnitudes
    // and explicit directional-mixing policy acknowledgement. Each table
    // is bounded with the larger backlash magnitude, including transients.
    bool IsXDirectionalCombinedAdmitted() const noexcept { return IsSealed() && m_xDirectionalCombinedAdmitted; }
    bool IsXDirectionalCombinedConfiguration(const AxisContext& axis) const noexcept;
    // Shared lifecycle scope of the distinct K, L, M, N, O and P profiles, never J/OFF.
    bool IsXStagedProfileAdmitted() const noexcept
    { return IsSealed() && (m_xPitchOnlyAdmitted || m_xBacklashOnlyAdmitted || m_xCombinedAdmitted || m_xAsymmetricCombinedAdmitted || m_xDirectionalPitchOnlyAdmitted || m_xDirectionalCombinedAdmitted); }
    bool IsXStagedProfileConfiguration(const AxisContext& axis) const noexcept;
    // K/L/M/N/O/P-only, before any reference: exact no-output-yet numerical zero frame.
    bool IsXBootstrapCoordinateIdentity(const AxisContext& axis) const noexcept;
    bool RefreshXBootstrapFrame(AxisContext& axis, pbc::Diagnostic& d) noexcept;
    // Entry excludes a second K/L/M/N/O/P HOME even when its current correction is zero.
    // A pre-HOME Initial RESET may already have created a zero reference.
    bool IsXHomeEntryCoordinateIdentity(const AxisContext& axis) const noexcept;
    // Exact zero transform can continue through the first HOME's ending tick;
    // this is deliberately separate from permission to start another HOME.
    bool IsXHomeControlCoordinateIdentity(const AxisContext& axis) const noexcept;
    bool IsXKnownAppliedControlFrame(const AxisContext& axis) const noexcept;
    bool IsXPhysicalHomeEstablished() const noexcept
    { return m_xPhysicalHomeEstablished.load(std::memory_order_acquire); }


    // PBC-3D: RT-owned reference transactions for the configured physical X.
    // Preparation assigns a private monotone sequence in transaction.authority;
    // the caller must retain that value when rechecking its real reservation.
    enum class XRetainRecoveryKind : unsigned { None = 0U, Servo, Fault, Limit };
    struct XReferenceTransaction
    {
        pbc::ReferenceAuthority authority{};
        pbc::ReferencePlan plan{};
        XRetainRecoveryKind recoveryKind = XRetainRecoveryKind::None;
        bool prepared = false;
    };
    bool PrepareXResetReference(const AxisContext& axis,
        const pbc::ReferenceAuthority& authority, XReferenceTransaction& transaction,
        pbc::Diagnostic& d) noexcept;
    // A stopped, output-inhibited recovery retains only an exact known applied
    // offset. Motion owns fresh feedback, queue drain and coordinate reservation.
    // OFF keeps its historical numeric snap; uncertain active output is rejected.
    bool PrepareXRetainRecoveryReference(const AxisContext& axis, XRetainRecoveryKind kind,
        const pbc::ReferenceAuthority& authority, XReferenceTransaction& transaction,
        pbc::Diagnostic& d) noexcept;
    bool PrepareXHomeReference(const AxisContext& axis, double coordinateShiftPulse,
        const pbc::ReferenceAuthority& authority, XReferenceTransaction& transaction,
        pbc::Diagnostic& d) noexcept;
    bool FinishXReference(AxisContext& axis, const XReferenceTransaction& transaction,
        const pbc::ReferenceAuthority& current, pbc::ReferenceCommitResult result,
        pbc::Diagnostic& d) noexcept;
    std::uint64_t XReferenceGeneration() const noexcept { return m_models[0].ReferenceGeneration(); }
    pbc::LifecyclePhase XReferencePhase() const noexcept { return m_models[0].Phase(); }
    unsigned XReferenceChecks() const noexcept { return m_xReferenceChecks; }
    // Exact committed nonzero X frame; never a pending proposal, numeric snap,
    // guessed offset or recovery from uncertain output. Does not authorize IO.
    bool IsXKnownOffsetReferenceReady(const AxisContext& axis) const noexcept;
    // Successful Hold/ControlledStop commit with zero physical command speed.
    // A frozen unfinished ramp can prove STOP, never endpoint completion.
    bool IsXFrozenStopReady(const AxisContext& axis) const noexcept;
    // A scalar preview only: no HOME, reference generation or lifecycle changes.
    // The disabled path returns the original raw double without arithmetic.
    bool TryNominalSnapPulse(int axisIndex, const AxisContext& axis,
        double& nominalPulse) const noexcept;

    // PBC-3E: real OFF-only lifecycle proposals, consumed at the physical
    // software-send boundary. Neither prepare nor seal changes applied state.
    enum class XCyclePrepareResult : unsigned { Bypassed, Prepared, Rejected };
    struct XCycleTransaction
    {
        pbc::CycleAuthority authority{};
        pbc::CoordinateFrame frame{};
        std::uint64_t ticket = 0ULL;
        bool prepared = false;
    };
    XCyclePrepareResult PrepareXCycle(const AxisContext& axis,
        const AxisCommand& nominalCommand, const pbc::CycleAuthority& authority,
        double dt, XCycleTransaction& transaction, pbc::Diagnostic& d) noexcept;
    bool SealXCycle(const XCycleTransaction& transaction,
        const pbc::CycleIdentity& current, const pbc::CoordinateFrame& actualFrame,
        pbc::Diagnostic& d) noexcept;
    bool FinishXCycle(AxisContext& axis, const XCycleTransaction& transaction,
        const pbc::CycleIdentity& current, pbc::SendResult outcome,
        pbc::Diagnostic& d) noexcept;
    // Only this exact known-unadopted proposal may be discarded. A stale
    // consumer cannot clear a different pending cycle or a reference plan.
    bool CancelXCycle(const XCycleTransaction& transaction, pbc::Diagnostic& d) noexcept;
    bool HasPendingXCycle() const noexcept { return m_xCyclePending.prepared; }
    unsigned XControlChecks() const noexcept { return m_xControlChecks; }

    // Legacy seam preserves commands for OFF and any admitted X profile.
    // The real correction is composed exactly once at PrepareXCycle.
    bool ApplyCompensation(int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt);

private:
    struct Pending
    {
        pbc::Config config{};
        std::vector<double> positive;
        std::vector<double> negative;
        bool initialized = false;
        bool explicitPolicy = false;
    };
    std::array<Pending, pbc::AxisCount> m_pending{};
    std::array<pbc::AxisLifecycle, pbc::AxisCount> m_models{};
    std::array<bool, pbc::AxisCount> m_runtimeFaultReported{};
    std::atomic<bool> m_sealed{ false };
    unsigned m_coordinateContractChecks = 0U; // boot writer only
    unsigned m_lifecycleContractChecks = 0U; // PBC-3B boot-only local model tests
    unsigned m_xReferenceChecks = 0U; // boot-only, local nonzero HOME tests
    std::uint64_t m_xReferenceSequence = 0ULL; // RT writer, never reset/replayed
    const AxisContext* m_configuredXAxis = nullptr;
    double m_xConfiguredResolution = 0.0, m_xConfiguredLead = 0.0;
    bool m_xConfiguredReverse = false, m_xConfiguredAxisReverse = false;
    bool m_xConfiguredPitch = false, m_xConfiguredBacklash = false;
    bool m_xZeroOnlyAdmitted = false; // boot writer; immutable after seal
    bool m_xPitchOnlyAdmitted = false; // separate bounded symmetric-pitch profile
    bool m_xBacklashOnlyAdmitted = false; // separate symmetric-backlash-only profile
    bool m_xCombinedAdmitted = false; // separate bounded symmetric pitch + backlash profile
    bool m_xAsymmetricCombinedAdmitted = false; // separate bounded unequal-backlash mixed profile
    bool m_xDirectionalPitchOnlyAdmitted = false; // separate bounded distinct-PN pitch-only profile
    bool m_xDirectionalCombinedAdmitted = false; // acknowledged distinct-PN plus asymmetric backlash
    std::atomic<bool> m_xPhysicalHomeEstablished{ false }; // one-way K/L/M/N/O/P first HOME receipt
    pbc::CoordinateFrame m_xZeroBootstrapFrame{}; // numerical identity, NOT a HOME/reference
    XReferenceTransaction m_xReferencePending{};
    XCycleTransaction m_xCyclePending{};
    std::uint64_t m_xLastReferenceCommitTick = 0ULL;
    pbc::CycleIdentity m_xLastAppliedCycle{};
    pbc::CycleMode m_xLastAppliedCycleMode = pbc::CycleMode::Motion;
    unsigned m_xControlChecks = 0U;
    bool MatchesPendingXCycle(const XCycleTransaction& transaction) const noexcept;
    bool CanConfigure(int axisIndex) const noexcept;
    bool IsXConfiguredPhysicalScope(const AxisContext& axis) const noexcept;
    bool HasKnownXAppliedFrame(const AxisContext& axis) const noexcept;
    bool HasXExactZeroFrame(const AxisContext& axis) const noexcept;
    bool IsXReferenceScope(const AxisContext& axis) const noexcept;
    bool PrepareXReferenceAuthority(const AxisContext& axis,
        const pbc::ReferenceAuthority& authority, XReferenceTransaction& transaction,
        pbc::Diagnostic& d, bool allowKnownOffset = false) noexcept;
};
