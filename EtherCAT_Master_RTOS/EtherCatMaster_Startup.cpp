#include "EtherCatMaster.h"

#include <windows.h>
#include <rtapi.h>

#include "GlobalConfig.h"


// ============================================================================
// EtherCatMaster Startup
//
// 檔案責任：
//
// 1. EtherCAT Master Startup State Tracking
// 2. Runtime XML 載入
// 3. NIC Attach
// 4. Slave Scan
// 5. Process Image 建立
// 6. Slave Initialization
//
// 不負責：
//
// - 250 us PDO Runtime
// - PLC Runtime
// - OP Runtime Loop
// - Motion / NC
//
// 這些會在後續 Stage 4B 再分層整理。
// ============================================================================


// ============================================================================
// GetStartupStageName
// ============================================================================

const char* EtherCatMaster::GetStartupStageName(
    EtherCatStartupStage stage) const
{
    switch (stage)
    {
    case EtherCatStartupStage::Idle:
        return "IDLE";

    case EtherCatStartupStage::OpenNic:
        return "OPEN_NIC";

    case EtherCatStartupStage::LoadRuntimeConfig:
        return "LOAD_RUNTIME_CONFIG";

    case EtherCatStartupStage::AttachMaster:
        return "ATTACH_MASTER";

    case EtherCatStartupStage::ScanSlaves:
        return "SCAN_SLAVES";

    case EtherCatStartupStage::BuildProcessImage:
        return "BUILD_PROCESS_IMAGE";

    case EtherCatStartupStage::InitializeSlaves:
        return "INITIALIZE_SLAVES";

    case EtherCatStartupStage::Ready:
        return "READY";

    case EtherCatStartupStage::Fault:
        return "FAULT";

    default:
        return "UNKNOWN";
    }
}


// ============================================================================
// BeginStartupStage
// ============================================================================

void EtherCatMaster::BeginStartupStage(
    EtherCatStartupStage stage)
{
    InterlockedExchange(
        &m_startupStage,
        (LONG)stage);


    InterlockedExchange(
        &m_startupResult,
        (LONG)EtherCatStartupResult::Running);


    DEBUG_PRINT(
        "[MASTER-STARTUP] Stage:%s | Result:BEGIN\n",
        GetStartupStageName(
            stage));
}


// ============================================================================
// PassStartupStage
// ============================================================================

void EtherCatMaster::PassStartupStage(
    EtherCatStartupStage stage)
{
    InterlockedExchange(
        &m_startupStage,
        (LONG)stage);


    InterlockedExchange(
        &m_startupResult,
        (LONG)EtherCatStartupResult::Pass);


    DEBUG_PRINT(
        "[MASTER-STARTUP] Stage:%s | Result:PASS\n",
        GetStartupStageName(
            stage));
}


// ============================================================================
// FailStartupStage
// ============================================================================

void EtherCatMaster::FailStartupStage(
    EtherCatStartupStage failedStage,
    LONG errorCode,
    const char* reason)
{
    InterlockedExchange(
        &m_startupFailedStage,
        (LONG)failedStage);


    InterlockedExchange(
        &m_startupErrorCode,
        errorCode);


    InterlockedExchange(
        &m_startupStage,
        (LONG)EtherCatStartupStage::Fault);


    InterlockedExchange(
        &m_startupResult,
        (LONG)EtherCatStartupResult::Fail);


    DEBUG_PRINT(
        "[MASTER-STARTUP] Stage:%s | "
        "ErrorCode:%ld | Reason:%s | Result:FAIL\n",

        GetStartupStageName(
            failedStage),

        errorCode,

        reason != nullptr
        ? reason
        : "UNKNOWN");


    DEBUG_PRINT(
        "[MASTER-STARTUP-RESULT] "
        "Stage:FAULT | FailedStage:%s | "
        "ErrorCode:%ld | Result:FAIL\n",

        GetStartupStageName(
            failedStage),

        errorCode);
}


// ============================================================================
// MarkStartupReady
// ============================================================================

void EtherCatMaster::MarkStartupReady()
{
    InterlockedExchange(
        &m_startupFailedStage,
        (LONG)EtherCatStartupStage::Idle);


    InterlockedExchange(
        &m_startupErrorCode,
        EcatStartupOk);


    InterlockedExchange(
        &m_startupStage,
        (LONG)EtherCatStartupStage::Ready);


    InterlockedExchange(
        &m_startupResult,
        (LONG)EtherCatStartupResult::Pass);


    DEBUG_PRINT(
        "[MASTER-STARTUP] Stage:READY | Result:PASS\n");


    DEBUG_PRINT(
        "[MASTER-STARTUP-RESULT] "
        "Stage:READY | FailedStage:NONE | "
        "ErrorCode:0 | Result:PASS\n");
}


// ============================================================================
// GetStartupSnapshot
// ============================================================================

EtherCatStartupSnapshot
EtherCatMaster::GetStartupSnapshot() const
{
    EtherCatStartupSnapshot snapshot;


    snapshot.stage =
        (EtherCatStartupStage)
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_startupStage),
            0,
            0);


    snapshot.result =
        (EtherCatStartupResult)
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_startupResult),
            0,
            0);


    snapshot.failedStage =
        (EtherCatStartupStage)
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_startupFailedStage),
            0,
            0);


    snapshot.errorCode =
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_startupErrorCode),
            0,
            0);


    return
        snapshot;
}


// ============================================================================
// InitializeMaster
//
// 原本位於 EtherCAT_Master_RTOS.cpp 的 InitEtherCATMaster() 主體，
// 現在正式歸屬 EtherCatMaster。
// ============================================================================

bool EtherCatMaster::InitializeMaster(
    CNicDriver& nic,
    EtherCatEni& eni,
    const char* runtimeConfigPath)
{
    // =========================================================
    // Reset Startup Snapshot
    // =========================================================

    InterlockedExchange(
        &m_startupFailedStage,
        (LONG)EtherCatStartupStage::Idle);


    InterlockedExchange(
        &m_startupErrorCode,
        EcatStartupOk);


    InterlockedExchange(
        &m_startupStage,
        (LONG)EtherCatStartupStage::Idle);


    InterlockedExchange(
        &m_startupResult,
        (LONG)EtherCatStartupResult::Idle);


    // =========================================================
    // OPEN_NIC
    // =========================================================

    BeginStartupStage(
        EtherCatStartupStage::OpenNic);


    if (!nic.Open())
    {
        FailStartupStage(
            EtherCatStartupStage::OpenNic,
            EcatStartupErrorNicOpen,
            "NIC_OPEN_FAILED");

        return false;
    }


    PassStartupStage(
        EtherCatStartupStage::OpenNic);


    // =========================================================
    // LOAD_RUNTIME_CONFIG
    // =========================================================

    BeginStartupStage(
        EtherCatStartupStage::LoadRuntimeConfig);


    if (runtimeConfigPath == nullptr ||
        runtimeConfigPath[0] == '\0' ||
        !eni.LoadXml(
            runtimeConfigPath))
    {
        FailStartupStage(
            EtherCatStartupStage::LoadRuntimeConfig,
            EcatStartupErrorRuntimeConfig,
            "RUNTIME_CONFIG_LOAD_FAILED");

        return false;
    }


    PassStartupStage(
        EtherCatStartupStage::LoadRuntimeConfig);


    // =========================================================
    // ATTACH_MASTER
    // =========================================================

    BeginStartupStage(
        EtherCatStartupStage::AttachMaster);


    AttachNic(
        &nic);


    AttachEni(
        &eni);


    PassStartupStage(
        EtherCatStartupStage::AttachMaster);


    // =========================================================
    // SCAN_SLAVES
    // =========================================================

    BeginStartupStage(
        EtherCatStartupStage::ScanSlaves);


    const int slavesCount =
        ScanSlaves();


    if (slavesCount <= 0)
    {
        FailStartupStage(
            EtherCatStartupStage::ScanSlaves,
            EcatStartupErrorScanSlaves,
            "NO_ETHERCAT_SLAVES");

        return false;
    }


    DEBUG_PRINT(
        "SlavesCount>>%d\n",
        slavesCount);


    PassStartupStage(
        EtherCatStartupStage::ScanSlaves);


    // =========================================================
    // BUILD_PROCESS_IMAGE
    //
    // 內含 Stage 3：
    //
    // - Servo PDO Size Guard
    // - Process Image Guard
    // - Frame Estimate Guard
    // =========================================================

    BeginStartupStage(
        EtherCatStartupStage::BuildProcessImage);


    if (BuildIoMap() != 0)
    {
        FailStartupStage(
            EtherCatStartupStage::BuildProcessImage,
            EcatStartupErrorBuildProcessImage,
            "BUILD_PROCESS_IMAGE_FAILED");

        return false;
    }


    PassStartupStage(
        EtherCatStartupStage::BuildProcessImage);


    // =========================================================
    // Stage 7A - Process Image Binding Shadow Audit
    //
    // Runtime XML application binding
    //      vs
    // current BuildIoMap() result.
    //
    // BuildIoMap() remains authoritative in Stage 7A.
    // Any difference is logged, but Startup continues.
    // =========================================================

    const bool runtimeBindingAuditPass =
        AuditRuntimeProcessImageBindings();


    if (!runtimeBindingAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 7A Process Image Binding Audit found differences. "
            "Startup continues with existing BuildIoMap binding.\n");
    }


    // =========================================================
    // Stage 11A - Composite Multi-Binding Shadow Audit
    //
    // New <ApplicationBindings> is future-capable 0..N binding data.
    //
    // Current Stage7B ProcessImageBinding remains authoritative.
    // Any difference is diagnostic only in Stage11A.
    // =========================================================

    const bool compositeBindingAuditPass =
        AuditRuntimeCompositeApplicationBindings();


    if (!compositeBindingAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11A Composite Binding Audit found differences. "
            "Startup continues with Stage7B simple binding.\n");
    }


    // =========================================================
    // Stage 11C.1 - Persisted Composite Runtime Shadow Audit
    //
    // Verifies Project v1.3 relative -> absolute Process Image
    // translation. Shadow only. No active Runtime cutover.
    // =========================================================

    const bool compositeRuntimeShadowAuditPass =
        AuditRuntimeCompositeProjectShadowBindings();


    if (!compositeRuntimeShadowAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.1 Composite Runtime Shadow Audit found differences. "
            "Startup continues with Stage11A / Stage7B active binding.\n");
    }


    // =========================================================
    // Stage 11C.2 - Generic Application Descriptor Shadow
    // =========================================================

    const bool compositeAdapterShadowBuildPass =
        BuildRuntimeCompositeApplicationDescriptorsShadow();


    bool compositeAdapterShadowAuditPass =
        false;


    if (compositeAdapterShadowBuildPass)
    {
        compositeAdapterShadowAuditPass =
            AuditRuntimeCompositeApplicationDescriptorsShadow();
    }


    if (!compositeAdapterShadowBuildPass ||
        !compositeAdapterShadowAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.2 Composite Adapter Shadow found differences. "
            "Startup continues with legacy Stage7B adapters.\n");
    }


    // =========================================================
    // Stage 11C.3 - Generic Access API Shadow Audit
    //
    // Live Process Image READ only.
    // Output WRITE test uses a scratch copy and verifies live map
    // remains unchanged.
    // =========================================================

    bool compositeAccessShadowAuditPass =
        false;


    if (compositeAdapterShadowBuildPass &&
        compositeAdapterShadowAuditPass)
    {
        compositeAccessShadowAuditPass =
            AuditRuntimeCompositeAccessApiShadow();
    }


    if (!compositeAccessShadowAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.3 Composite Access API Shadow found differences. "
            "Startup continues with legacy Stage7B adapters.\n");
    }


    // =========================================================
    // Stage 11C.4 - Generic LIVE Read Route Cutover
    //
    // Cutover scope is intentionally limited to the generic read API.
    //
    // PLC remains on Stage7B.
    // Motion / Servo remains on the existing structured adapter.
    // Live output write remains disabled.
    // =========================================================

    bool compositeLiveReadRouteBuildPass =
        false;


    bool compositeLiveReadRouteAuditPass =
        false;


    if (compositeAccessShadowAuditPass)
    {
        compositeLiveReadRouteBuildPass =
            BuildRuntimeCompositeLiveReadRoute();


        if (compositeLiveReadRouteBuildPass)
        {
            compositeLiveReadRouteAuditPass =
                AuditRuntimeCompositeLiveReadRoute();
        }
    }


    if (!compositeLiveReadRouteBuildPass ||
        !compositeLiveReadRouteAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.4 Generic Live Read Route found differences. "
            "Legacy Stage7B PLC/Motion paths remain authoritative.\n");
    }


    // =========================================================
    // Stage 11C.5 - Generic Read Consumer Bridge SHADOW
    //
    // Consumer-facing semantic resolution:
    //
    //     Kind + AxisRef + occurrence
    //
    // Existing PLC / EDM / Motion consumers are not switched yet.
    // =========================================================

    bool compositeReadConsumerBridgeShadowPass =
        false;


    if (compositeLiveReadRouteBuildPass &&
        compositeLiveReadRouteAuditPass)
    {
        compositeReadConsumerBridgeShadowPass =
            AuditRuntimeCompositeReadConsumerBridgeShadow();
    }


    if (!compositeReadConsumerBridgeShadowPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.5 Generic Read Consumer Bridge Shadow found differences. "
            "Existing PLC/EDM/Motion consumers remain on legacy paths.\n");
    }


    // =========================================================
    // Stage 11C.6 - EDM Read Consumer API LIVE Cutover
    //
    // Activates the semantic consumer API itself.
    //
    // Existing System_EDM_SINKER_MODE.cpp read call sites are
    // intentionally not migrated yet.
    // =========================================================

    bool compositeReadConsumerLiveRouteBuildPass =
        false;


    bool compositeReadConsumerLiveRouteAuditPass =
        false;


    if (compositeReadConsumerBridgeShadowPass)
    {
        compositeReadConsumerLiveRouteBuildPass =
            BuildRuntimeCompositeReadConsumerLiveRoute();


        if (compositeReadConsumerLiveRouteBuildPass)
        {
            compositeReadConsumerLiveRouteAuditPass =
                AuditRuntimeCompositeReadConsumerLiveRoute();
        }
    }


    if (!compositeReadConsumerLiveRouteBuildPass ||
        !compositeReadConsumerLiveRouteAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.6 EDM Read Consumer API Live Route found differences. "
            "Existing EDM/PLC/Motion call sites remain on legacy paths.\n");
    }


    // =========================================================
    // Stage 11C.7 - PLC Generic Read Consumer Map SHADOW
    //
    // Source inspection confirmed that DI / AD application
    // consumption currently lives in PlcCore:
    //
    //     FetchInputs()
    //     SyncPhysicalToVirtual()
    //
    // System_EDM_SINKER_MODE.cpp itself does not directly read
    // m_IoList / m_AdList.
    //
    // This stage validates the existing IoMapConfig mapping
    // against the new semantic consumer API without changing the
    // 1ms live PLC path.
    // =========================================================

    bool plcGenericReadConsumerMapShadowPass =
        false;


    if (compositeReadConsumerLiveRouteBuildPass &&
        compositeReadConsumerLiveRouteAuditPass)
    {
        m_Plc.SetGenericReadMaster(
            this);


        plcGenericReadConsumerMapShadowPass =
            m_Plc.AuditGenericReadConsumerMapShadow();
    }


    if (!plcGenericReadConsumerMapShadowPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.7 PLC Generic Read Consumer Map Shadow found differences. "
            "PLC input/output paths remain fully legacy.\n");
    }


    // =========================================================
    // Stage 11C.8 - PLC Generic INPUT Live Cutover
    //
    // Prepare semantic occurrence tables once outside the 1ms
    // PLC loop, audit Generic vs Legacy values, then activate.
    //
    // Only DI -> PLC I and AD -> PLC DR are cut over.
    //
    // DO / Motion / Servo remain legacy.
    // =========================================================

    bool plcGenericInputPreparePass =
        false;


    bool plcGenericInputAuditPass =
        false;


    bool plcGenericInputActivatePass =
        false;


    if (plcGenericReadConsumerMapShadowPass)
    {
        plcGenericInputPreparePass =
            m_Plc.PrepareGenericInputLiveRoute();


        if (plcGenericInputPreparePass)
        {
            plcGenericInputAuditPass =
                m_Plc.AuditGenericInputLiveRoute();
        }


        if (plcGenericInputAuditPass)
        {
            plcGenericInputActivatePass =
                m_Plc.ActivateGenericInputLiveRoute();
        }
    }


    if (!plcGenericInputPreparePass ||
        !plcGenericInputAuditPass ||
        !plcGenericInputActivatePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.8 PLC Generic INPUT Live Cutover blocked. "
            "SyncPhysicalToVirtual keeps the legacy fallback available.\n");
    }


    // =========================================================
    // Stage 11C.11 - Generic PLC Output Shadow Ownership Gate
    //
    // Output remains 100% legacy.
    //
    // This gate only proves:
    // - semantic DigitalOutput ownership
    // - exclusive PLC O ownership
    // - legacy/generic output pointer envelope equivalence
    // - scratch-only bit mask correctness
    // - live output Process Image preservation
    //
    // There is NO Generic live output write in this stage.
    // =========================================================

    bool plcGenericOutputOwnershipPreparePass =
        false;


    bool plcGenericOutputOwnershipAuditPass =
        false;


    if (plcGenericReadConsumerMapShadowPass &&
        compositeAccessShadowAuditPass)
    {
        plcGenericOutputOwnershipPreparePass =
            m_Plc.PrepareGenericOutputOwnershipShadow();


        if (plcGenericOutputOwnershipPreparePass)
        {
            plcGenericOutputOwnershipAuditPass =
                m_Plc.AuditGenericOutputOwnershipShadow();
        }
    }


    if (!plcGenericOutputOwnershipPreparePass ||
        !plcGenericOutputOwnershipAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.11 Generic PLC Output Shadow Ownership Gate blocked. "
            "PLC output remains fully legacy; no Generic live write is enabled.\n");
    }


    // =========================================================
    // Stage 11C.12 - Generic PLC Output Command Bridge SHADOW
    //
    // Enables per-cycle comparison:
    //
    //     PLC O semantic command
    //         vs
    //     current legacy outBuffer command
    //
    // No live output route changes.
    // =========================================================

    bool plcGenericOutputCommandBridgeShadowPass =
        false;


    if (plcGenericOutputOwnershipPreparePass &&
        plcGenericOutputOwnershipAuditPass)
    {
        plcGenericOutputCommandBridgeShadowPass =
            m_Plc.PrepareGenericOutputCommandBridgeShadow();
    }


    if (!plcGenericOutputCommandBridgeShadowPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.12 Generic PLC Output Command Bridge Shadow blocked. "
            "PLC output remains fully legacy; no Generic live write is enabled.\n");
    }


    // =========================================================
    // Stage 11C.13 - Generic PLC Output Live-Write
    // Preflight SHADOW
    //
    // Future safe cutover does NOT add another pOutputLoc writer.
    //
    // PLC 1ms:
    //     semantic command -> existing shadow outBuffer
    //
    // PDO 250us:
    //     FlushOutputs -> pOutputLoc
    //
    // Physical writer count remains exactly one.
    // =========================================================

    bool plcGenericOutputLiveWritePreflightPreparePass =
        false;


    bool plcGenericOutputLiveWritePreflightAuditPass =
        false;


    if (plcGenericOutputCommandBridgeShadowPass)
    {
        plcGenericOutputLiveWritePreflightPreparePass =
            m_Plc.PrepareGenericOutputLiveWritePreflightShadow();


        if (plcGenericOutputLiveWritePreflightPreparePass)
        {
            plcGenericOutputLiveWritePreflightAuditPass =
                m_Plc.AuditGenericOutputLiveWritePreflightShadow();
        }
    }


    if (!plcGenericOutputLiveWritePreflightPreparePass ||
        !plcGenericOutputLiveWritePreflightAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.13 Generic PLC Output Live-Write Preflight Shadow blocked. "
            "PLC output remains fully legacy; Generic live write remains disabled.\n");
    }


    // =========================================================
    // Stage 11C.14 - Controlled Generic PLC DigitalOutput
    // Live Command Cutover PREPARE
    //
    // Startup only validates the route.
    //
    // Actual activation is delayed until the existing 1000ms
    // supervisor observes >=5000 clean Stage11C.13-qualified
    // command cycles on THIS boot.
    //
    // Physical writer remains FlushOutputs() only.
    // =========================================================

    bool plcGenericOutputControlledCutoverPreparePass =
        false;


    if (plcGenericOutputLiveWritePreflightPreparePass &&
        plcGenericOutputLiveWritePreflightAuditPass)
    {
        plcGenericOutputControlledCutoverPreparePass =
            m_Plc.PrepareGenericOutputControlledCutover();
    }


    if (!plcGenericOutputControlledCutoverPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11C.14 Generic PLC DigitalOutput controlled cutover prepare blocked. "
            "PLC output remains on legacy Set_O / FlushOutputs.\n");
    }


    // =========================================================
    // Stage 11D.1 - Structured ServoDrive Adapter SHADOW
    //
    // Prerequisite project evidence:
    // Stage11C.16 PLC Generic I/O Release Gate has passed on
    // real hardware.
    //
    // This startup step only expands each ServoDrive parent
    // Composite descriptor into its field-level ServoPDO_9_23
    // view and compares it with current ENI_ServoDrive.
    //
    // Motion and Servo commands remain completely legacy.
    // =========================================================

    bool structuredServoShadowBuildPass =
        false;


    bool structuredServoShadowAuditPass =
        false;


    if (compositeAdapterShadowBuildPass &&
        compositeAdapterShadowAuditPass &&
        compositeAccessShadowAuditPass)
    {
        structuredServoShadowBuildPass =
            BuildStructuredServoDriveFieldDescriptorsShadow();


        if (structuredServoShadowBuildPass)
        {
            structuredServoShadowAuditPass =
                AuditStructuredServoDriveFieldDescriptorsShadow();
        }
    }


    if (!structuredServoShadowBuildPass ||
        !structuredServoShadowAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.1 Structured ServoDrive Adapter Shadow found differences. "
            "Motion and Servo command paths remain legacy ENI_ServoDrive.\n");
    }


    // =========================================================
    // Stage 11D.2 - Structured ServoDrive Live Read SHADOW
    //
    // Only enables 1ms field comparison after Stage11D.1
    // exact-layout / pointer / value audit has passed.
    //
    // Motion and Servo commands remain legacy.
    // =========================================================

    bool structuredServoLiveReadShadowPreparePass =
        false;


    if (structuredServoShadowBuildPass &&
        structuredServoShadowAuditPass)
    {
        structuredServoLiveReadShadowPreparePass =
            PrepareStructuredServoDriveLiveReadShadow();
    }


    if (!structuredServoLiveReadShadowPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.2 Structured ServoDrive Live Read Shadow blocked. "
            "Motion and Servo command paths remain legacy ENI_ServoDrive.\n");
    }


    // =========================================================
    // Stage 11D.3 - Motion Servo Input Consumer Bridge SHADOW
    //
    // Bind the owning Master into MotionCore for a read-only
    // 250us shadow comparison.
    //
    // Prepare resolves the CURRENT Motion identity:
    //
    //     Motion slot + AxisContext.axisIndex
    //         -> Servo slave
    //         -> structured input fields
    //
    // Axis names are NC-managed and are not part of this identity.
    // =========================================================

    bool motionServoInputBridgeShadowPreparePass =
        false;


    if (structuredServoLiveReadShadowPreparePass)
    {
        m_Motion.BindStructuredServoReadShadowMaster(
            this);


        motionServoInputBridgeShadowPreparePass =
            PrepareMotionServoInputConsumerBridgeShadow();
    }


    if (!motionServoInputBridgeShadowPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.3 Motion Servo Input Consumer Bridge Shadow blocked. "
            "Motion remains legacy ENI_ServoDrive.\n");
    }


    // =========================================================
    // Stage 11D.4 Rev2 - AxisIndex Semantic Identity Gate
    //
    // EtherCAT / Motion identity is AxisContext.axisIndex.
    //
    // No axis NAME is read here.
    // AXIS_CFG.ini remains NC-owned.
    // Runtime AxisRef is not required.
    //
    // This is still a shadow / pre-cutover safety gate.
    // =========================================================

    bool motionServoAxisIndexIdentityPass =
        false;


    if (motionServoInputBridgeShadowPreparePass)
    {
        motionServoAxisIndexIdentityPass =
            AuditMotionServoAxisIndexIdentityGate();
    }


    if (!motionServoAxisIndexIdentityPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.4 Rev2 AxisIndex Semantic Identity Gate blocked. "
            "Check Motion Slot / AxisIndex / Servo mapping. "
            "Motion remains legacy ENI_ServoDrive.\n");
    }


    // =========================================================
    // Stage 11D.5 - AxisIndex Semantic Motion Input Preflight
    //
    // Build the fixed AxisIndex -> structured Servo input route.
    //
    // Actual MotionCore read consumers are NOT cut over.
    // =========================================================

    bool motionServoAxisIndexInputPreflightPass =
        false;


    if (motionServoAxisIndexIdentityPass)
    {
        motionServoAxisIndexInputPreflightPass =
            PrepareMotionServoAxisIndexInputPreflightShadow();
    }


    if (!motionServoAxisIndexInputPreflightPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.5 AxisIndex Semantic Motion Input Preflight blocked. "
            "Motion remains legacy ENI_ServoDrive.\n");
    }


    // =========================================================
    // Stage 11D.6 - Motion Servo Input Consumer Seam Preflight
    //
    // The current project already contains the D6 seam in
    // MotionCore and the D6 observer/counters in
    // EtherCatMaster_CompositeAccessShadow.cpp.
    //
    // This startup hook was missing in the uploaded baseline.
    // Without this call the D6 observer remains Prepared:NO and
    // can never accumulate AxisSamples.
    //
    // Actual Motion input source remains the centralized LEGACY
    // ENI_ServoDrive snapshot.  AxisIndex semantic input is still
    // comparison-only in Stage11D.6.
    // =========================================================

    bool motionServoInputConsumerSeamPreflightPass =
        false;


    if (motionServoAxisIndexInputPreflightPass)
    {
        motionServoInputConsumerSeamPreflightPass =
            PrepareMotionServoInputConsumerSeamPreflightShadow();
    }


    if (!motionServoInputConsumerSeamPreflightPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.6 Motion Servo Input Consumer Seam Preflight blocked. "
            "Motion input remains legacy snapshot.\n");
    }


    // =========================================================
    // Stage 11D.7 - Controlled AxisIndex Semantic Core Motion
    // Input Cutover
    //
    // Startup only PREPARES the controlled route.
    //
    // Runtime still starts on LEGACY snapshot and the 250us
    // Motion gate will enable semantic input only after this
    // boot's Stage11D.6 qualification is clean.
    // =========================================================

    bool controlledMotionServoInputCutoverPreparePass =
        false;


    if (motionServoInputConsumerSeamPreflightPass)
    {
        controlledMotionServoInputCutoverPreparePass =
            PrepareControlledMotionServoInputCutover();
    }


    if (!controlledMotionServoInputCutoverPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.7 Controlled Motion Semantic Input Cutover not prepared. "
            "Core Motion input remains legacy snapshot.\n");
    }


    // =========================================================
    // Stage 11D.8 - Semantic Motion Input Compatibility
    // Retirement
    //
    // Startup only prepares the publication/compatibility route.
    //
    // TouchProbe / Debug remain legacy during D7 warmup and
    // automatically move to the published active Motion snapshot
    // only after D7 is fully qualified.
    // =========================================================

    bool motionServoInputCompatibilityRetirementPreparePass =
        false;


    if (controlledMotionServoInputCutoverPreparePass)
    {
        motionServoInputCompatibilityRetirementPreparePass =
            PrepareMotionServoInputCompatibilityRetirement();
    }


    if (!motionServoInputCompatibilityRetirementPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.8 Motion Input Compatibility Retirement not prepared. "
            "Compatibility input remains legacy fallback.\n");
    }


    // =========================================================
    // Stage 11D.9 - Legacy Servo Input Normal-Path Retirement
    //
    // Startup only prepares retirement.
    //
    // Runtime starts with the existing qualification path and
    // retires legacy pInput normal reads only after D8 becomes
    // fully qualified in this boot.
    // =========================================================

    bool legacyMotionServoInputNormalPathRetirementPreparePass =
        false;


    if (motionServoInputCompatibilityRetirementPreparePass)
    {
        legacyMotionServoInputNormalPathRetirementPreparePass =
            PrepareLegacyMotionServoInputNormalPathRetirement();
    }


    if (!legacyMotionServoInputNormalPathRetirementPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.9 Legacy Motion Servo Input Normal-Path Retirement "
            "not prepared. Legacy qualification path remains active.\n");
    }


    // =========================================================
    // Stage 11D.10 - Servo Input Release / Diagnostic Retirement
    //
    // Startup only prepares the final release gate.
    //
    // Runtime release occurs after this boot proves:
    // - D9 semantic-only normal path qualified
    // - D2 structured live-read diagnostic qualified
    // =========================================================

    bool servoInputReleaseGatePreparePass =
        false;


    if (legacyMotionServoInputNormalPathRetirementPreparePass)
    {
        servoInputReleaseGatePreparePass =
            PrepareServoInputReleaseGate();
    }


    if (!servoInputReleaseGatePreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11D.10 Servo Input Release Gate not prepared. "
            "Historical Servo input diagnostics remain active.\n");
    }


    // =========================================================
    // Stage 11E.1 - Servo Output Command Ownership SHADOW
    //
    // Build the fixed AxisIndex -> structured Servo OUTPUT route.
    //
    // Runtime observation starts only after Stage11D.10 Servo
    // INPUT release has completed.
    //
    // Current Servo command source remains legacy pOutput.
    // =========================================================

    bool motionServoOutputCommandOwnershipShadowPreparePass =
        false;


    if (servoInputReleaseGatePreparePass)
    {
        motionServoOutputCommandOwnershipShadowPreparePass =
            PrepareMotionServoOutputCommandOwnershipShadow();
    }


    if (!motionServoOutputCommandOwnershipShadowPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.1 Servo Output Command Ownership Shadow blocked. "
            "Servo command remains legacy ENI_ServoDrive::pOutput.\n");
    }


    // =========================================================
    // Stage 11E.2 - Servo Output Semantic Scratch Command Bridge
    //
    // Prepare the structured OUTPUT writer against a LOCAL
    // ServoOutput scratch image only.
    //
    // Runtime E2 observation starts only after E1 itself reaches
    // its current-boot sustained qualification window.
    //
    // No live Process Image write.
    // =========================================================

    bool motionServoOutputSemanticScratchBridgePreparePass =
        false;


    if (motionServoOutputCommandOwnershipShadowPreparePass)
    {
        motionServoOutputSemanticScratchBridgePreparePass =
            PrepareMotionServoOutputSemanticScratchBridgeShadow();
    }


    if (!motionServoOutputSemanticScratchBridgePreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.2 Servo Output Semantic Scratch Bridge blocked. "
            "Servo command remains legacy ENI_ServoDrive::pOutput.\n");
    }


    // =========================================================
    // Stage 11E.3 - Servo Output Live-Write Target / Rollback
    // Preflight SHADOW
    //
    // Proves the exact future m_IoMap ServoOutput target and
    // rollback image using redirected LOCAL memory only.
    //
    // Runtime E3 accumulation starts only after E2 has reached
    // its current-boot sustained qualification.
    //
    // LiveWrite remains NO.
    // =========================================================

    bool motionServoOutputLiveWritePreflightPreparePass =
        false;


    if (motionServoOutputSemanticScratchBridgePreparePass)
    {
        motionServoOutputLiveWritePreflightPreparePass =
            PrepareMotionServoOutputLiveWritePreflightShadow();
    }


    if (!motionServoOutputLiveWritePreflightPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.3 Servo Output Live-Write Preflight blocked. "
            "Servo command remains legacy ENI_ServoDrive::pOutput.\n");
    }


    // =========================================================
    // Stage 11E.4 - Servo Output Command Seam SHADOW
    //
    // Motion/Homing normal command assignments are centralized
    // in MotionCore, but the active producer STILL writes the
    // original ENI ServoOutput / m_IoMap.
    //
    // E4 runtime starts only after E3 is qualified.
    //
    // No structured live output write.
    // =========================================================

    bool motionServoOutputCommandSeamPreparePass =
        false;


    if (motionServoOutputLiveWritePreflightPreparePass)
    {
        motionServoOutputCommandSeamPreparePass =
            PrepareMotionServoOutputCommandSeamShadow();
    }


    if (!motionServoOutputCommandSeamPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.4 Servo Output Command Seam blocked. "
            "Servo command remains legacy direct pOutput behavior.\n");
    }


    // =========================================================
    // Stage 11E.5 - Controlled Structured Servo Output Producer
    //
    // Startup prepares only.
    // Actual cutover waits for E4 runtime qualification.
    //
    // Any structured fault boot-latches rollback to legacy.
    // =========================================================

    bool motionServoOutputStructuredProducerPreparePass =
        false;


    if (motionServoOutputCommandSeamPreparePass)
    {
        motionServoOutputStructuredProducerPreparePass =
            PrepareMotionServoOutputStructuredProducerCutover();
    }


    if (!motionServoOutputStructuredProducerPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.5 Structured Servo Output Producer blocked. "
            "Servo command remains legacy command seam.\n");
    }


    // =========================================================
    // Stage 11E.6 - Servo Output Compatibility Retirement
    // =========================================================

    bool motionServoOutputCompatibilityRetirementPreparePass =
        false;


    if (motionServoOutputStructuredProducerPreparePass)
    {
        motionServoOutputCompatibilityRetirementPreparePass =
            PrepareMotionServoOutputCompatibilityRetirementShadow();
    }


    if (!motionServoOutputCompatibilityRetirementPreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.6 Servo Output Compatibility Retirement blocked. "
            "Historical output qualification remains active.\n");
    }


    // =========================================================
    // Stage 11E.7 - Servo Generic I/O Release Gate
    // Final release latch only. No behavior change.
    // =========================================================

    bool servoGenericIoReleasePreparePass =
        false;


    if (motionServoOutputCompatibilityRetirementPreparePass)
    {
        servoGenericIoReleasePreparePass =
            PrepareServoGenericIoReleaseGate();
    }


    if (!servoGenericIoReleasePreparePass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 11E.7 Servo Generic I/O Release Gate blocked.\n");
    }


    // =========================================================
    // INITIALIZE_SLAVES
    //
    // Stage 9B Runtime Cutover Gate is evaluated immediately
    // before any INIT / PRE-OP hardware configuration.
    //
    // Modern Runtime XML:
    //     ALL slaves have <ProcessImageBinding>
    //     -> Generic Runtime Contract is the REAL cutover gate.
    //
    // Legacy Runtime XML:
    //     ZERO slaves have <ProcessImageBinding>
    //     -> ProductCode Stage5C remains compatibility fallback.
    //
    // Partial schema:
    //     hard FAIL. Never mix gate models.
    // =========================================================

    BeginStartupStage(
        EtherCatStartupStage::InitializeSlaves);


    // Fail-safe default:
    // hardware Runtime cutover stays CLOSED until one approved gate opens it.
    m_runtimeEquivalenceVerified =
        false;


    if (m_pEni ==
        nullptr)
    {
        FailStartupStage(
            EtherCatStartupStage::InitializeSlaves,
            EcatStartupErrorInitializeSlaves,
            "RUNTIME_CONFIG_NOT_ATTACHED");

        return
            false;
    }


    const auto& runtimeContractSlaves =
        m_pEni->GetSlaves();


    int runtimeContractSchemaCount =
        0;


    for (const auto& slave :
        runtimeContractSlaves)
    {
        if (slave.runtimeProcessImageBinding.present)
        {
            runtimeContractSchemaCount++;
        }
    }


    // =========================================================
    // Stage 9B - Modern Runtime Contract Gate
    // =========================================================

    if (!runtimeContractSlaves.empty() &&
        runtimeContractSchemaCount ==
        (int)runtimeContractSlaves.size())
    {
        DEBUG_PRINT(
            "[RUNTIME-CUTOVER-GATE-ROUTE] "
            "ContractSchema:%d/%u | "
            "Source:RUNTIME_CONTRACT | "
            "Gate:GENERIC_STAGE9B | "
            "ProductCodeBaseline:NO | "
            "Result:SELECTED\n",

            runtimeContractSchemaCount,

            (unsigned int)
            runtimeContractSlaves.size());


        const bool runtimeContractPass =
            AuditRuntimeConfigurationContractGeneric();


        if (!runtimeContractPass)
        {
            DEBUG_PRINT(
                "[RUNTIME-CUTOVER-GATE-RESULT] "
                "Source:RUNTIME_CONTRACT | "
                "Gate:GENERIC_STAGE9B | "
                "Result:FAIL | CutoverGate:CLOSED\n");


            FailStartupStage(
                EtherCatStartupStage::InitializeSlaves,
                EcatStartupErrorInitializeSlaves,
                "RUNTIME_CONTRACT_GATE_FAILED");

            return
                false;
        }


        m_runtimeEquivalenceVerified =
            true;


        DEBUG_PRINT(
            "[RUNTIME-CUTOVER-GATE-RESULT] "
            "Source:RUNTIME_CONTRACT | "
            "Gate:GENERIC_STAGE9B | "
            "Result:PASS | CutoverGate:OPEN\n");
    }


    // =========================================================
    // Legacy Runtime XML fallback
    //
    // Old XML with no ProcessImageBinding keeps the exact
    // ProductCode-based Stage5C compatibility behavior.
    //
    // Stage5C itself owns m_runtimeEquivalenceVerified on this path.
    // A NOT-EQUIVALENT legacy audit keeps the gate CLOSED and the
    // old C++ hardware configuration remains the fallback.
    // =========================================================

    else if (runtimeContractSchemaCount ==
        0)
    {
        DEBUG_PRINT(
            "[RUNTIME-CUTOVER-GATE-ROUTE] "
            "ContractSchema:0/%u | "
            "Source:LEGACY_STAGE5C | "
            "Gate:PRODUCTCODE_FALLBACK | "
            "Result:SELECTED\n",

            (unsigned int)
            runtimeContractSlaves.size());


        const bool runtimeConfigEquivalent =
            AuditRuntimeConfigurationEquivalence();


        DEBUG_PRINT(
            "[RUNTIME-CUTOVER-GATE-RESULT] "
            "Source:LEGACY_STAGE5C | "
            "Gate:PRODUCTCODE_FALLBACK | "
            "Result:%s | CutoverGate:%s\n",

            runtimeConfigEquivalent
            ? "PASS"
            : "FALLBACK",

            runtimeConfigEquivalent
            ? "OPEN"
            : "CLOSED");
    }


    // =========================================================
    // Partial modern schema is never allowed.
    // =========================================================

    else
    {
        DEBUG_PRINT(
            "[RUNTIME-CUTOVER-GATE-ROUTE] "
            "ContractSchema:%d/%u | "
            "Partial Runtime contract schema is forbidden | "
            "Result:FAIL\n",

            runtimeContractSchemaCount,

            (unsigned int)
            runtimeContractSlaves.size());


        FailStartupStage(
            EtherCatStartupStage::InitializeSlaves,
            EcatStartupErrorInitializeSlaves,
            "PARTIAL_RUNTIME_CONTRACT_SCHEMA");

        return
            false;
    }


    // =========================================================
    // INIT / PRE-OP / SAFE-OP
    //
    // Stage 1B Topology / Identity Verify
    // INIT
    // DC Propagation
    // PRE-OP
    // Stage 8B Runtime SM / PDO Readback Safety Gate
    // SAFE-OP
    // =========================================================


    if (Initialize_Slaves() != 0)
    {
        FailStartupStage(
            EtherCatStartupStage::InitializeSlaves,
            EcatStartupErrorInitializeSlaves,
            "INITIALIZE_SLAVES_FAILED");

        return false;
    }


    PassStartupStage(
        EtherCatStartupStage::InitializeSlaves);


    // =========================================================
    // Stage 6A - Runtime Transport Shadow Audit
    //
    // Read/compare only. Existing Mailbox/FMMU writers remain
    // authoritative. A mismatch is diagnostic and does not block
    // the current verified Startup path.
    // =========================================================

    const bool runtimeTransportAuditPass =
        AuditRuntimeTransportConfiguration();


    if (!runtimeTransportAuditPass)
    {
        DEBUG_PRINT(
            "[MASTER-STARTUP] "
            "Stage 6A Transport Shadow Audit found differences. "
            "Startup continues with existing verified transport path.\n");
    }


    // =========================================================
    // READY
    //
    // READY = Startup Configuration 完成。
    //
    // 尚未表示：
    // - PDO Runtime Started
    // - PLC Runtime Started
    // - OP Entered
    //
    // 這些 Stage 4B 再納入。
    // =========================================================

    MarkStartupReady();


    return true;
}
