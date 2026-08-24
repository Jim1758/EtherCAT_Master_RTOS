#pragma once
#include "EtherCatTypes.h"
#include <vector>
#include <string>
#include <atomic>

class EtherCatMaster;


// 數位 IO 對映項目
struct DigitalMapItem {
    int listIdx;          // 在 m_pIo 陣列中的索引
    int userOrder;        // 🌟 新增：使用者自訂的邏輯順序 (例如 1, 2, 3...)
    int plcStartIndex;    // PLC 虛擬記憶體起點 (例如 I0 或 O0)
    int bitCount;         // 映射點數
    std::string comment;  // 註解說明
};

// 類比 IO 對映項目
struct AnalogMapItem {
    int listIdx;          // 在 m_pAd 陣列中的索引
    int userOrder;        // 🌟 新增：使用者自訂的順序
    int plcDrStartIndex;  // 對應到 PLC 的 DR 暫存器起點 (例如 DR100)
    int channelCount;     // 通道數量
    std::string comment;
};

class PlcCore {
public:
    PlcCore();

    int m_marqueeLed_debug = 0;
    // 🌟 1. 綁定數位與類比的硬體清單
    void SetIoLists(std::vector<ENI_GenericIO>* pIoList, std::vector<ENI_AnalogModule>* pAdList);

    // 🌟 2. 自動掃描硬體並建立 IO 對映表 (啟動時呼叫)
    void AutoMapIO();

    // 🌟 3. 核心橋接 API (在 1ms 中斷裡呼叫)
    void FetchInputs();           // [重要新增] 將硬體 Rx 拷貝到影子記憶體
    void SyncPhysicalToVirtual(); // 實體影子記憶體 -> PLC (I點 / DR暫存器)
    void SyncVirtualToPhysical(); // PLC (O點) -> 實體影子記憶體
    void FlushOutputs();          // 實體影子記憶體 -> 硬體 Tx

    void Update_Debug();

    // 底層操作 API
    bool Get_I(int listIdx, int bitIdx);
    void Set_O(int listIdx, int bitIdx, bool val);
    bool Get_O(int listIdx, int bitIdx);
    void Clear_Module_O(int listIdx);


    // =========================================================
    // Stage 11C.7 - PLC Generic Read Consumer Map SHADOW
    //
    // The actual live PLC path remains:
    //
    //     FetchInputs()
    //     SyncPhysicalToVirtual()
    //
    // This stage only proves that the CURRENT IoMapConfig-driven
    // DI / AD mapping resolves to the new semantic consumer API.
    // =========================================================
    void SetGenericReadMaster(
        EtherCatMaster* pMaster);

    bool AuditGenericReadConsumerMapShadow();


    // =========================================================
    // Stage 11C.8 - PLC Generic INPUT Live Cutover
    //
    // Prepare:
    //     resolves current IoMapConfig DI / AD mappings to
    //     semantic occurrences outside the 1ms PLC loop.
    //
    // Audit:
    //     compares prepared Generic reads with legacy values.
    //
    // Activate:
    //     switches SyncPhysicalToVirtual() INPUT source to the
    //     Generic Consumer API.
    //
    // Output path remains legacy.
    // =========================================================
    bool PrepareGenericInputLiveRoute();

    bool AuditGenericInputLiveRoute();

    bool ActivateGenericInputLiveRoute();

    bool IsGenericInputLiveRouteEnabled() const
    {
        return m_GenericInputLiveRouteEnabled;
    }


    // =========================================================
    // Stage 11C.9 - PLC Generic Input Compatibility Retirement
    // SHADOW diagnostics.
    //
    // Called from the existing 1000ms supervisory loop.
    // Never called from the 1ms PLC bridge path.
    // =========================================================
    void PrintGenericInputCompatibilityRetirementShadow();

    // Stage 11C.10 - guarded legacy input cache retirement.
    bool ActivateLegacyInputCacheRetirement();

    bool IsLegacyInputCacheRetired() const
    {
        return
            m_LegacyInputCacheRetired.load(
                std::memory_order_relaxed);
    }


    // =========================================================
    // Stage 11C.11 - Generic PLC Output Shadow Ownership Gate
    //
    // PREPARE:
    // - map current IoMapConfig DO modules to semantic
    //   DigitalOutput occurrences.
    //
    // AUDIT:
    // - exclusive descriptor ownership
    // - PLC O range ownership
    // - legacy pointer envelope equivalence
    // - scratch-only bit writes / masks
    // - live Process Image output must remain unchanged
    //
    // This stage NEVER changes the active PLC output route.
    // =========================================================

    bool PrepareGenericOutputOwnershipShadow();

    bool AuditGenericOutputOwnershipShadow();


    // =========================================================
    // Stage 11C.12 - Generic PLC Output Command Bridge SHADOW
    //
    // PREPARE:
    // - enables the per-cycle command comparison only after the
    //   Stage11C.11 ownership gate has passed.
    //
    // RUNTIME SHADOW:
    // - reads PLC O bits
    // - builds semantic DigitalOutput command values
    // - compares them with the current legacy outBuffer result
    //
    // NO Generic live output write.
    // =========================================================

    bool PrepareGenericOutputCommandBridgeShadow();

    void PrintGenericOutputCommandBridgeShadow() const;


    // =========================================================
    // Stage 11C.13 - Generic PLC Output Live-Write
    // Preflight SHADOW
    //
    // Future cutover architecture:
    //
    // PLC 1ms:
    //     semantic DigitalOutput command
    //         -> shadow outBuffer
    //
    // PDO 250us:
    //     FlushOutputs()
    //         -> pOutputLoc
    //
    // FlushOutputs remains the ONLY physical output writer.
    //
    // This stage performs NO Generic live output write.
    // =========================================================

    bool PrepareGenericOutputLiveWritePreflightShadow();

    bool AuditGenericOutputLiveWritePreflightShadow();

    void PrintGenericOutputLiveWritePreflightShadow() const;


    // =========================================================
    // Stage 11C.14 - Controlled Generic PLC DigitalOutput
    // Live Command Cutover
    //
    // The Generic route writes ONLY the existing software
    // outBuffer.  FlushOutputs() remains the sole pOutputLoc
    // physical writer.
    //
    // Activation is allowed only after the current boot has
    // qualified Stage11C.13 runtime evidence.
    //
    // Any Generic route fault latches this stage back to the
    // proven legacy Set_O() path for the remainder of the boot.
    // =========================================================

    bool PrepareGenericOutputControlledCutover();

    void ProcessGenericOutputControlledCutover();

    bool IsGenericOutputControlledCutoverEnabled() const
    {
        return
            m_GenericOutputControlledCutoverEnabled.load(
                std::memory_order_acquire);
    }


    // =========================================================
    // Stage 11C.15 - Generic PLC DigitalOutput Compatibility
    // Retirement SHADOW
    //
    // Observes the ACTIVE Stage11C.14 route after activation.
    //
    // PASS requires:
    // - >=5000 Generic live output cycles
    // - exact expected semantic map writes
    // - zero fallback
    // - zero route fault
    //
    // Legacy Set_O remains emergency fallback only.
    // No live behavior changes in this stage.
    // =========================================================

    void PrintGenericOutputCompatibilityRetirementShadow() const;


    // =========================================================
    // Stage 11C.16 - Generic PLC I/O Release Gate
    //
    // Final aggregate checkpoint for the PLC-facing Generic
    // input/output subsystem.
    //
    // Diagnostic only. No live behavior changes.
    // =========================================================
    void PrintGenericPlcIoReleaseGate() const;


private:
    std::vector<ENI_GenericIO>* m_pIo = nullptr;
    std::vector<ENI_AnalogModule>* m_pAd = nullptr;

    std::vector<DigitalMapItem> m_inputMaps;
    std::vector<DigitalMapItem> m_outputMaps;
    std::vector<AnalogMapItem>  m_analogInputMaps;


    // Stage 11C.7+:
    // non-owning EtherCatMaster bridge for Generic semantic APIs.
    //
    // Input route may be live.
    // Stage11C.11 output use remains SHADOW / scratch-only.
    EtherCatMaster* m_pGenericReadMaster =
        nullptr;


    // =========================================================
    // Stage 11C.8 prepared semantic route.
    //
    // Vectors are built once during startup.  The 1ms PLC loop
    // performs no allocation and does not derive physical scan
    // order dynamically.
    // =========================================================
    std::vector<int>
        m_GenericDiSemanticOccurrences;

    std::vector<int>
        m_GenericAdBaseOccurrences;


    // Stage 11C.11:
    // semantic DigitalOutput occurrence for each m_outputMaps item.
    std::vector<int>
        m_GenericDoSemanticOccurrences;

    bool m_GenericOutputOwnershipShadowPrepared =
        false;


    // =========================================================
    // Stage 11C.12 per-cycle command comparison state.
    //
    // Written from the 1ms PLC output bridge and observed from
    // the existing 1000ms diagnostic path.
    // =========================================================

    bool m_GenericOutputCommandBridgeShadowEnabled =
        false;

    int m_GenericOutputCommandBridgeMapCount =
        0;

    std::atomic<uint64_t> m_GenericOutputCommandShadowCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericOutputCommandShadowMapChecks
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericOutputCommandShadowMatches
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericOutputCommandShadowMismatches
    {
        0ULL
    };


    // =========================================================
    // Stage 11C.13 static preflight state.
    //
    // Built/audited during startup.
    // Read only by the 1000ms supervisory diagnostics.
    // =========================================================

    bool m_GenericOutputLiveWritePreflightPrepared =
        false;

    bool m_GenericOutputLiveWritePreflightAuditPassed =
        false;

    int m_GenericOutputLiveWritePreflightMapCount =
        0;

    int m_GenericOutputLiveWritePreflightRollbackChecks =
        0;

    int m_GenericOutputLiveWritePreflightRollbackMatches =
        0;

    int m_GenericOutputLiveWritePreflightLivePreserveChecks =
        0;

    int m_GenericOutputLiveWritePreflightLivePreserveMatches =
        0;


    // =========================================================
    // Stage 11C.14 controlled live command route.
    //
    // Physical Process Image ownership does NOT move:
    //
    //     outBuffer -> FlushOutputs() -> pOutputLoc
    //
    // Only the command producer changes from legacy Set_O()
    // to the prepared semantic DigitalOutput route.
    // =========================================================

    bool m_GenericOutputControlledCutoverPrepared =
        false;

    int m_GenericOutputControlledCutoverMapCount =
        0;

    std::atomic<bool> m_GenericOutputControlledCutoverEnabled
    {
        false
    };

    std::atomic<bool> m_GenericOutputControlledCutoverFaulted
    {
        false
    };

    std::atomic<uint64_t> m_GenericOutputControlledLiveCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericOutputControlledMapWrites
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericOutputControlledFallbacks
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericOutputControlledRouteFaults
    {
        0ULL
    };


    bool m_GenericInputLiveRoutePrepared =
        false;

    bool m_GenericInputLiveRouteEnabled =
        false;

    std::atomic<uint64_t> m_GenericInputLiveCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericInputFallbackCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_GenericInputReadFailures
    {
        0ULL
    };

    std::atomic<bool> m_LegacyInputCacheRetired
    {
        false
    };

    std::atomic<uint64_t> m_LegacyInputCacheSuppressedCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_DirectInputFallbackCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_DirectInputFallbackFailures
    {
        0ULL
    };


    int m_marqueeLed = 0;


};