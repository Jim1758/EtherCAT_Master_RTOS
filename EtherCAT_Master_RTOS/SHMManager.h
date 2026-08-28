#pragma once

#include "SHM_Types.h"
#include "EtherCatSmDiagContract.h"
#include "EtherCatFmmuDiagContract.h"
#include "EtherCatWatchdogDiagContract.h"
#include "EtherCatDcQualificationDiagContract.h"
#include "EtherCatDcV1aDiagContract.h"
#include "EtherCatRxCorrelationDiagContract.h"
#include <string>

// RTX64 API
#include <windows.h>
#include <rtapi.h>

// ============================================================================
// Stage 12E.7C - Shared Memory access declarations
// ============================================================================
//
// The main SHMManager class continues to own EDM_CNC_SHM.
//
// Engineering / diagnostic channels are separate fixed-contract mappings
// created in SHMManager.cpp and exposed through producer-side accessors.
//
// Existing channels:
//
//   OSCARMAX_ECAT_DIAG
//       -> SHM_ECAT_DiagData
//
//   OSCARMAX_ECAT_SERVICE
//       -> SHM_ECAT_ServiceData
//
//   OSCARMAX_ECAT_SM_DIAG
//       -> SHM_ECAT_SmDiagData
//
//   OSCARMAX_ECAT_FMMU_DIAG
//       -> SHM_ECAT_FmmuDiagData
//
// Stage 12E.7C:
//
//   OSCARMAX_ECAT_WATCHDOG_DIAG
//       -> SHM_ECAT_WatchdogDiagData
//
// Ownership:
//
//   Priority-64
//       EtherCAT owner
//       writes RT diagnostic shadows only.
//
//   Priority-50
//       reads RT shadows
//       writes Shared Memory.
//
//   Windows ENI Tool
//       read-only.
//
// Priority-64 must never write these mappings directly.
//
// Source fingerprint:
//
//   OSCARMAX_ECAT_WATCHDOG_DIAG_SHMMANAGER_H_12E7C_20260827
//
// ============================================================================

SHM_ECAT_DiagData*
GetEtherCatDiagSharedMemoryData();

SHM_ECAT_ServiceData*
GetEtherCatServiceSharedMemoryData();

SHM_ECAT_SmDiagData*
GetEtherCatSmDiagSharedMemoryData();

SHM_ECAT_FmmuDiagData*
GetEtherCatFmmuDiagSharedMemoryData();

SHM_ECAT_WatchdogDiagData*
GetEtherCatWatchdogDiagSharedMemoryData();


SHM_ECAT_DcQualDiagData*
GetEtherCatDcQualificationDiagSharedMemoryData();


SHM_ECAT_DcV1aDiagData*
GetEtherCatDcV1aDiagSharedMemoryData();


SHM_ECAT_RxCorrDiagData*
GetEtherCatRxCorrelationDiagSharedMemoryData();


class SHMManager
{
public:

    static SHMManager& GetInstance()
    {
        static SHMManager instance;
        return instance;
    }

    // RTX64 side does not prepend Global\.
    bool Initialize(
        const std::string& shmName =
        "EDM_CNC_SHM");

    // Release all Shared Memory resources owned by SHMManager.cpp.
    void Shutdown();

    // Main EDM_CNC_SHM data pointer.
    SHM_Data* GetData()
    {
        return m_pData;
    }

private:

    SHMManager() = default;

    ~SHMManager()
    {
        Shutdown();
    }

    SHMManager(
        const SHMManager&) = delete;

    SHMManager&
        operator=(
            const SHMManager&) = delete;

    SHM_Data* m_pData = nullptr;

    HANDLE m_hShm = NULL;
};
