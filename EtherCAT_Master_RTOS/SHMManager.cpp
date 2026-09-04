#include "SHMManager.h"

#include "EtherCatSmDiagContract.h"
#include "EtherCatFmmuDiagContract.h"
#include "EtherCatWatchdogDiagContract.h"
#include "EtherCatDcQualificationDiagContract.h"
#include "EtherCatDcV1aDiagContract.h"
#include "EtherCatRxCorrelationDiagContract.h"
#include "EtherCatRxForensicsDiagContract.h"

#include "GlobalConfig.h"

#include <cstring>


namespace
{
    // =====================================================================
    // Stage 12A.2 - OSCARMAX EtherCAT Online Diagnosis Shared Memory
    // =====================================================================

    constexpr const wchar_t*
        ECAT_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_DIAG";

    HANDLE
        g_hEtherCatDiagShm =
        NULL;

    SHM_ECAT_DiagData*
        g_pEtherCatDiagData =
        nullptr;


    // =====================================================================
    // Stage 12F.3B.2 - OSCARMAX EtherCAT Engineering Service SHM
    // =====================================================================

    constexpr const wchar_t*
        ECAT_SERVICE_SHM_NAME =
        L"OSCARMAX_ECAT_SERVICE";

    HANDLE
        g_hEtherCatServiceShm =
        NULL;

    SHM_ECAT_ServiceData*
        g_pEtherCatServiceData =
        nullptr;


    // =====================================================================
    // Stage 12E.4C - OSCARMAX EtherCAT SyncManager Live Diagnostic SHM
    // =====================================================================

    constexpr const wchar_t*
        ECAT_SM_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_SM_DIAG";

    HANDLE
        g_hEtherCatSmDiagShm =
        NULL;

    SHM_ECAT_SmDiagData*
        g_pEtherCatSmDiagData =
        nullptr;


    // =====================================================================
    // Stage 12E.5C - OSCARMAX EtherCAT FMMU Live Diagnostic SHM
    // =====================================================================

    constexpr const wchar_t*
        ECAT_FMMU_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_FMMU_DIAG";

    HANDLE
        g_hEtherCatFmmuDiagShm =
        NULL;

    SHM_ECAT_FmmuDiagData*
        g_pEtherCatFmmuDiagData =
        nullptr;


    // =====================================================================
    // Stage 12E.7C - OSCARMAX EtherCAT Watchdog Live Diagnostic SHM
    // =====================================================================
    //
    // Dedicated read-only Windows diagnostic channel for:
    //
    //     ESC 0x0400 Watchdog Divider
    //     ESC 0x0420 Process Data Watchdog Time
    //     ESC 0x0440 Process Data Watchdog Status
    //     ESC 0x0442 Process Data Watchdog Counter
    //
    // Producer:
    //     Priority-50 publisher
    //
    // Source:
    //     Priority-64 owner-safe RT diagnostic shadow
    //
    // Consumer:
    //     Windows ENI Tool read-only
    //
    // IMPORTANT:
    //
    // This Shared Memory mapping itself performs ZERO EtherCAT transactions.
    //
    // Source fingerprint:
    //
    //     OSCARMAX_ECAT_WATCHDOG_DIAG_SHM_12E7C_20260827
    //
    // =====================================================================

    constexpr const wchar_t*
        ECAT_WATCHDOG_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_WATCHDOG_DIAG";

    HANDLE
        g_hEtherCatWatchdogDiagShm =
        NULL;

    SHM_ECAT_WatchdogDiagData*
        g_pEtherCatWatchdogDiagData =
        nullptr;


    // =====================================================================
    // Stage 17A.3C.1 - OSCARMAX_ECAT_DC_QUAL_DIAG
    // =====================================================================

    constexpr const wchar_t*
        ECAT_DC_QUAL_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_DC_QUAL_DIAG";

    HANDLE
        g_hEtherCatDcQualDiagShm =
        NULL;

    SHM_ECAT_DcQualDiagData*
        g_pEtherCatDcQualDiagData =
        nullptr;


    // =====================================================================
    // Stage 17A.5.1 - OSCARMAX_ECAT_DC_V1A_DIAG
    // =====================================================================

    constexpr const wchar_t*
        ECAT_DC_V1A_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_DC_V1A_DIAG";

    HANDLE
        g_hEtherCatDcV1aDiagShm =
        NULL;

    SHM_ECAT_DcV1aDiagData*
        g_pEtherCatDcV1aDiagData =
        nullptr;


    // =====================================================================
    // Stage 17A.8.1 - OSCARMAX_ECAT_RX_CORR_DIAG
    // =====================================================================

    constexpr const wchar_t*
        ECAT_RX_CORR_DIAG_SHM_NAME =
        L"OSCARMAX_ECAT_RX_CORR_DIAG";

    HANDLE
        g_hEtherCatRxCorrDiagShm =
        NULL;

    SHM_ECAT_RxCorrDiagData*
        g_pEtherCatRxCorrDiagData =
        nullptr;


    // =====================================================================
    // DC-RX.4B - OSCARMAX_ECAT_RX_FORENSICS
    // =====================================================================

    constexpr const wchar_t*
        ECAT_RX_FORENSICS_SHM_NAME =
        L"OSCARMAX_ECAT_RX_FORENSICS";

    HANDLE
        g_hEtherCatRxForensicsShm =
        NULL;

    SHM_ECAT_RxForensicsData*
        g_pEtherCatRxForensicsData =
        nullptr;


    // =====================================================================
    // SyncManager Diagnostic Shared Memory
    // =====================================================================

    bool InitializeEtherCatSmDiagSharedMemory()
    {
        if (g_pEtherCatSmDiagData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-SM-DIAG-SHM] Initializing Shared Memory: "
            "OSCARMAX_ECAT_SM_DIAG\n");

        DEBUG_PRINT(
            "[ECAT-SM-DIAG-SHM] Contract Size:%zu B | "
            "Header:%zu B | Entry:%zu B x %u | "
            "Slaves:%u | SM/Slave:%u\n",
            sizeof(SHM_ECAT_SmDiagData),
            sizeof(SHM_ECAT_SmDiagHeader),
            sizeof(SHM_ECAT_SmDiagEntry),
            static_cast<unsigned int>(
                SHM_ECAT_SM_DIAG_ENTRY_COUNT),
            static_cast<unsigned int>(
                SHM_ECAT_SM_DIAG_MAX_SLAVES),
            static_cast<unsigned int>(
                SHM_ECAT_SM_DIAG_MAX_SYNC_MANAGERS));

        void* pLocation = nullptr;

        g_hEtherCatSmDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_SmDiagData),
                ECAT_SM_DIAG_SHM_NAME,
                &pLocation);

        if (
            g_hEtherCatSmDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-SM-DIAG-SHM] WARNING | "
                "Create failed. "
                "SyncManager Live Diagnosis unavailable; "
                "machine runtime will continue.\n");

            if (g_hEtherCatSmDiagShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatSmDiagShm);

                g_hEtherCatSmDiagShm =
                    NULL;
            }

            g_pEtherCatSmDiagData =
                nullptr;

            return false;
        }

        g_pEtherCatSmDiagData =
            static_cast<
            SHM_ECAT_SmDiagData*>(
                pLocation);

        std::memset(
            g_pEtherCatSmDiagData,
            0,
            sizeof(SHM_ECAT_SmDiagData));

        SHM_ECAT_SmDiagHeader& header =
            g_pEtherCatSmDiagData->Header;

        header.Magic =
            SHM_ECAT_SM_DIAG_MAGIC;

        header.VersionMajor =
            SHM_ECAT_SM_DIAG_VERSION_MAJOR;

        header.VersionMinor =
            SHM_ECAT_SM_DIAG_VERSION_MINOR;

        header.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_SmDiagData));

        header.Sequence =
            0u;

        header.Heartbeat =
            0u;

        header.PublishCount =
            0ULL;

        header.SlaveCount =
            0u;

        header.MaxSyncManagers =
            static_cast<uint16_t>(
                SHM_ECAT_SM_DIAG_MAX_SYNC_MANAGERS);

        for (
            uint32_t slave = 0u;
            slave <
            SHM_ECAT_SM_DIAG_MAX_SLAVES;
            ++slave)
        {
            for (
                uint32_t sm = 0u;
                sm <
                SHM_ECAT_SM_DIAG_MAX_SYNC_MANAGERS;
                ++sm)
            {
                const uint32_t flatIndex =
                    slave *
                    SHM_ECAT_SM_DIAG_MAX_SYNC_MANAGERS +
                    sm;

                SHM_ECAT_SmDiagEntry& entry =
                    g_pEtherCatSmDiagData
                    ->Entries[flatIndex];

                entry.SlavePosition =
                    static_cast<uint16_t>(
                        slave);

                entry.SmIndex =
                    static_cast<uint8_t>(
                        sm);

                entry.RegisterAddress =
                    static_cast<uint16_t>(
                        0x0800u +
                        static_cast<uint16_t>(
                            sm * 8u));
            }
        }

        DEBUG_PRINT(
            "[ECAT-SM-DIAG-SHM] READY | "
            "Name:OSCARMAX_ECAT_SM_DIAG | "
            "Magic:0x%08X | Version:%u.%u | "
            "Size:%u B | Slots:%u | "
            "ReadOnlyContract:YES | "
            "Publisher:NOT_YET_ACTIVE\n",
            static_cast<unsigned int>(
                header.Magic),
            static_cast<unsigned int>(
                header.VersionMajor),
            static_cast<unsigned int>(
                header.VersionMinor),
            static_cast<unsigned int>(
                header.StructSize),
            static_cast<unsigned int>(
                SHM_ECAT_SM_DIAG_ENTRY_COUNT));

        return true;
    }


    void ShutdownEtherCatSmDiagSharedMemory()
    {
        if (g_hEtherCatSmDiagShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatSmDiagShm);

            g_hEtherCatSmDiagShm =
                NULL;

            g_pEtherCatSmDiagData =
                nullptr;

            DEBUG_PRINT(
                "[ECAT-SM-DIAG-SHM] "
                "Shared Memory Closed.\n");
        }
    }


    // =====================================================================
    // FMMU Diagnostic Shared Memory
    // =====================================================================

    bool InitializeEtherCatFmmuDiagSharedMemory()
    {
        if (g_pEtherCatFmmuDiagData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-FMMU-DIAG-SHM] Initializing Shared Memory: "
            "OSCARMAX_ECAT_FMMU_DIAG\n");

        DEBUG_PRINT(
            "[ECAT-FMMU-DIAG-SHM] Contract Size:%zu B | "
            "Header:%zu B | Entry:%zu B x %u | "
            "Slaves:%u | FMMU/Slave:%u\n",
            sizeof(SHM_ECAT_FmmuDiagData),
            sizeof(SHM_ECAT_FmmuDiagHeader),
            sizeof(SHM_ECAT_FmmuDiagEntry),
            static_cast<unsigned int>(
                SHM_ECAT_FMMU_DIAG_ENTRY_COUNT),
            static_cast<unsigned int>(
                SHM_ECAT_FMMU_DIAG_MAX_SLAVES),
            static_cast<unsigned int>(
                SHM_ECAT_FMMU_DIAG_MAX_FMMUS));

        void* pLocation = nullptr;

        g_hEtherCatFmmuDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_FmmuDiagData),
                ECAT_FMMU_DIAG_SHM_NAME,
                &pLocation);

        if (
            g_hEtherCatFmmuDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-FMMU-DIAG-SHM] WARNING | "
                "Create failed. "
                "FMMU Live Diagnosis unavailable; "
                "machine runtime will continue.\n");

            if (g_hEtherCatFmmuDiagShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatFmmuDiagShm);

                g_hEtherCatFmmuDiagShm =
                    NULL;
            }

            g_pEtherCatFmmuDiagData =
                nullptr;

            return false;
        }

        g_pEtherCatFmmuDiagData =
            static_cast<
            SHM_ECAT_FmmuDiagData*>(
                pLocation);

        std::memset(
            g_pEtherCatFmmuDiagData,
            0,
            sizeof(SHM_ECAT_FmmuDiagData));

        SHM_ECAT_FmmuDiagHeader& header =
            g_pEtherCatFmmuDiagData->Header;

        header.Magic =
            SHM_ECAT_FMMU_DIAG_MAGIC;

        header.VersionMajor =
            SHM_ECAT_FMMU_DIAG_VERSION_MAJOR;

        header.VersionMinor =
            SHM_ECAT_FMMU_DIAG_VERSION_MINOR;

        header.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_FmmuDiagData));

        header.Sequence =
            0u;

        header.Heartbeat =
            0u;

        header.PublishCount =
            0ULL;

        header.SlaveCount =
            0u;

        header.MaxFmmus =
            static_cast<uint16_t>(
                SHM_ECAT_FMMU_DIAG_MAX_FMMUS);

        for (
            uint32_t slave = 0u;
            slave <
            SHM_ECAT_FMMU_DIAG_MAX_SLAVES;
            ++slave)
        {
            for (
                uint32_t fmmu = 0u;
                fmmu <
                SHM_ECAT_FMMU_DIAG_MAX_FMMUS;
                ++fmmu)
            {
                const uint32_t flatIndex =
                    slave *
                    SHM_ECAT_FMMU_DIAG_MAX_FMMUS +
                    fmmu;

                SHM_ECAT_FmmuDiagEntry& entry =
                    g_pEtherCatFmmuDiagData
                    ->Entries[flatIndex];

                entry.SlavePosition =
                    static_cast<uint16_t>(
                        slave);

                entry.FmmuIndex =
                    static_cast<uint8_t>(
                        fmmu);

                entry.RegisterAddress =
                    static_cast<uint16_t>(
                        0x0600u +
                        static_cast<uint16_t>(
                            fmmu * 16u));
            }
        }

        DEBUG_PRINT(
            "[ECAT-FMMU-DIAG-SHM] READY | "
            "Name:OSCARMAX_ECAT_FMMU_DIAG | "
            "Magic:0x%08X | Version:%u.%u | "
            "Size:%u B | Slots:%u | "
            "ReadOnlyContract:YES | "
            "Publisher:NOT_YET_ACTIVE\n",
            static_cast<unsigned int>(
                header.Magic),
            static_cast<unsigned int>(
                header.VersionMajor),
            static_cast<unsigned int>(
                header.VersionMinor),
            static_cast<unsigned int>(
                header.StructSize),
            static_cast<unsigned int>(
                SHM_ECAT_FMMU_DIAG_ENTRY_COUNT));

        return true;
    }


    void ShutdownEtherCatFmmuDiagSharedMemory()
    {
        if (g_hEtherCatFmmuDiagShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatFmmuDiagShm);

            g_hEtherCatFmmuDiagShm =
                NULL;

            g_pEtherCatFmmuDiagData =
                nullptr;

            DEBUG_PRINT(
                "[ECAT-FMMU-DIAG-SHM] "
                "Shared Memory Closed.\n");
        }
    }


    // =====================================================================
    // Stage 12E.7C - Watchdog Diagnostic Shared Memory
    // =====================================================================

    bool InitializeEtherCatWatchdogDiagSharedMemory()
    {
        if (g_pEtherCatWatchdogDiagData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-WATCHDOG-DIAG-SHM] "
            "Initializing Shared Memory: "
            "OSCARMAX_ECAT_WATCHDOG_DIAG\n");

        DEBUG_PRINT(
            "[ECAT-WATCHDOG-DIAG-SHM] "
            "Contract Size:%zu B | Header:%zu B | "
            "Entry:%zu B x %u | Slaves:%u\n",
            sizeof(SHM_ECAT_WatchdogDiagData),
            sizeof(SHM_ECAT_WatchdogDiagHeader),
            sizeof(SHM_ECAT_WatchdogDiagEntry),
            static_cast<unsigned int>(
                SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES),
            static_cast<unsigned int>(
                SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES));

        void* pLocation =
            nullptr;

        g_hEtherCatWatchdogDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(
                    SHM_ECAT_WatchdogDiagData),
                ECAT_WATCHDOG_DIAG_SHM_NAME,
                &pLocation);

        if (
            g_hEtherCatWatchdogDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-WATCHDOG-DIAG-SHM] WARNING | "
                "Create failed. "
                "Watchdog Live Diagnosis unavailable; "
                "machine runtime will continue.\n");

            if (
                g_hEtherCatWatchdogDiagShm !=
                NULL)
            {
                RtCloseHandle(
                    g_hEtherCatWatchdogDiagShm);

                g_hEtherCatWatchdogDiagShm =
                    NULL;
            }

            g_pEtherCatWatchdogDiagData =
                nullptr;

            return false;
        }

        g_pEtherCatWatchdogDiagData =
            static_cast<
            SHM_ECAT_WatchdogDiagData*>(
                pLocation);

        std::memset(
            g_pEtherCatWatchdogDiagData,
            0,
            sizeof(
                SHM_ECAT_WatchdogDiagData));

        SHM_ECAT_WatchdogDiagHeader& header =
            g_pEtherCatWatchdogDiagData->Header;

        header.Magic =
            SHM_ECAT_WATCHDOG_DIAG_MAGIC;

        header.VersionMajor =
            SHM_ECAT_WATCHDOG_DIAG_VERSION_MAJOR;

        header.VersionMinor =
            SHM_ECAT_WATCHDOG_DIAG_VERSION_MINOR;

        header.StructSize =
            static_cast<uint32_t>(
                sizeof(
                    SHM_ECAT_WatchdogDiagData));

        header.Sequence =
            0u;

        header.Heartbeat =
            0u;

        header.PublishCount =
            0ULL;

        header.SlaveCount =
            0u;

        header.MaxSlaves =
            static_cast<uint16_t>(
                SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES);

        for (
            uint32_t slave = 0u;
            slave <
            SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES;
            ++slave)
        {
            SHM_ECAT_WatchdogDiagEntry& entry =
                g_pEtherCatWatchdogDiagData
                ->Entries[slave];

            entry.SlavePosition =
                static_cast<uint16_t>(
                    slave);

            entry.Valid =
                0u;
        }

        DEBUG_PRINT(
            "[ECAT-WATCHDOG-DIAG-SHM] READY | "
            "Name:OSCARMAX_ECAT_WATCHDOG_DIAG | "
            "Magic:0x%08X | Version:%u.%u | "
            "Size:%u B | Slots:%u | "
            "ReadOnlyContract:YES | "
            "Publisher:NOT_YET_ACTIVE | "
            "EtherCATTrafficAdded:NO\n",
            static_cast<unsigned int>(
                header.Magic),
            static_cast<unsigned int>(
                header.VersionMajor),
            static_cast<unsigned int>(
                header.VersionMinor),
            static_cast<unsigned int>(
                header.StructSize),
            static_cast<unsigned int>(
                SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES));

        return true;
    }


    void ShutdownEtherCatWatchdogDiagSharedMemory()
    {
        if (
            g_hEtherCatWatchdogDiagShm !=
            NULL)
        {
            RtCloseHandle(
                g_hEtherCatWatchdogDiagShm);

            g_hEtherCatWatchdogDiagShm =
                NULL;

            g_pEtherCatWatchdogDiagData =
                nullptr;

            DEBUG_PRINT(
                "[ECAT-WATCHDOG-DIAG-SHM] "
                "Shared Memory Closed.\n");
        }
    }


    // =====================================================================
    // Main EtherCAT Online Diagnostic Shared Memory
    // =====================================================================

    bool InitializeEtherCatDiagSharedMemory()
    {
        if (g_pEtherCatDiagData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-DIAG-SHM] Initializing Shared Memory: "
            "OSCARMAX_ECAT_DIAG\n");

        DEBUG_PRINT(
            "[ECAT-DIAG-SHM] Contract Size:%zu B | "
            "Master:%zu B | Slave:%zu B x %u | "
            "ProcessImage:%u B\n",
            sizeof(SHM_ECAT_DiagData),
            sizeof(SHM_ECAT_DiagMasterStatus),
            sizeof(SHM_ECAT_DiagSlaveStatus),
            static_cast<unsigned int>(
                SHM_ECAT_DIAG_MAX_SLAVES),
            static_cast<unsigned int>(
                SHM_ECAT_DIAG_PROCESS_IMAGE_CAPACITY));

        void* pLocation =
            nullptr;

        g_hEtherCatDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_DiagData),
                ECAT_DIAG_SHM_NAME,
                &pLocation);

        if (
            g_hEtherCatDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-DIAG-SHM] WARNING | "
                "Create failed. "
                "EtherCAT Online Diagnosis unavailable; "
                "machine runtime will continue.\n");

            if (g_hEtherCatDiagShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatDiagShm);

                g_hEtherCatDiagShm =
                    NULL;
            }

            g_pEtherCatDiagData =
                nullptr;

            return false;
        }

        g_pEtherCatDiagData =
            static_cast<
            SHM_ECAT_DiagData*>(
                pLocation);

        std::memset(
            g_pEtherCatDiagData,
            0,
            sizeof(SHM_ECAT_DiagData));

        SHM_ECAT_DiagMasterStatus& master =
            g_pEtherCatDiagData->Master;

        master.Magic =
            SHM_ECAT_DIAG_MAGIC;

        master.VersionMajor =
            SHM_ECAT_DIAG_VERSION_MAJOR;

        master.VersionMinor =
            SHM_ECAT_DIAG_VERSION_MINOR;

        master.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_DiagData));

        master.ProcessImageCapacity =
            SHM_ECAT_DIAG_PROCESS_IMAGE_CAPACITY;

        master.DcReferenceSlave =
            0xFFFFu;

        master.DcReferenceConfigAddr =
            0u;

        master.Sequence =
            0u;

        for (
            uint32_t i = 0;
            i <
            SHM_ECAT_DIAG_MAX_SLAVES;
            ++i)
        {
            g_pEtherCatDiagData
                ->Slaves[i]
                .Position =
                static_cast<uint16_t>(
                    i);

            g_pEtherCatDiagData
                ->Slaves[i]
                .OutputOffset =
                -1;

            g_pEtherCatDiagData
                ->Slaves[i]
                .InputOffset =
                -1;
        }

        DEBUG_PRINT(
            "[ECAT-DIAG-SHM] READY | "
            "Name:OSCARMAX_ECAT_DIAG | "
            "Magic:0x%08X | Version:%u.%u | "
            "Size:%u B | "
            "ReadOnlyContract:YES | "
            "Publisher:NOT_YET_ACTIVE\n",
            static_cast<unsigned int>(
                master.Magic),
            static_cast<unsigned int>(
                master.VersionMajor),
            static_cast<unsigned int>(
                master.VersionMinor),
            static_cast<unsigned int>(
                master.StructSize));

        return true;
    }


    void ShutdownEtherCatDiagSharedMemory()
    {
        if (g_hEtherCatDiagShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatDiagShm);

            g_hEtherCatDiagShm =
                NULL;

            g_pEtherCatDiagData =
                nullptr;

            DEBUG_PRINT(
                "[ECAT-DIAG-SHM] "
                "Shared Memory Closed.\n");
        }
    }


    // =====================================================================
    // EtherCAT Engineering Service Shared Memory
    // =====================================================================

    bool InitializeEtherCatServiceSharedMemory()
    {
        if (g_pEtherCatServiceData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-SERVICE-SHM] "
            "Initializing Shared Memory: "
            "OSCARMAX_ECAT_SERVICE\n");

        DEBUG_PRINT(
            "[ECAT-SERVICE-SHM] "
            "Contract Size:%zu B | "
            "Version:%u.%u | "
            "Operation:SDO_READ\n",
            sizeof(SHM_ECAT_ServiceData),
            static_cast<unsigned int>(
                SHM_ECAT_SERVICE_VERSION_MAJOR),
            static_cast<unsigned int>(
                SHM_ECAT_SERVICE_VERSION_MINOR));

        void* pLocation =
            nullptr;

        g_hEtherCatServiceShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(
                    SHM_ECAT_ServiceData),
                ECAT_SERVICE_SHM_NAME,
                &pLocation);

        if (
            g_hEtherCatServiceShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-SERVICE-SHM] WARNING | "
                "Create failed. "
                "ENI Tool SDO service unavailable; "
                "machine runtime will continue.\n");

            if (
                g_hEtherCatServiceShm !=
                NULL)
            {
                RtCloseHandle(
                    g_hEtherCatServiceShm);

                g_hEtherCatServiceShm =
                    NULL;
            }

            g_pEtherCatServiceData =
                nullptr;

            return false;
        }

        g_pEtherCatServiceData =
            static_cast<
            SHM_ECAT_ServiceData*>(
                pLocation);

        std::memset(
            g_pEtherCatServiceData,
            0,
            sizeof(
                SHM_ECAT_ServiceData));

        g_pEtherCatServiceData->Magic =
            SHM_ECAT_SERVICE_MAGIC;

        g_pEtherCatServiceData
            ->VersionMajor =
            SHM_ECAT_SERVICE_VERSION_MAJOR;

        g_pEtherCatServiceData
            ->VersionMinor =
            SHM_ECAT_SERVICE_VERSION_MINOR;

        g_pEtherCatServiceData
            ->StructSize =
            static_cast<uint32_t>(
                sizeof(
                    SHM_ECAT_ServiceData));

        g_pEtherCatServiceData
            ->ServerState =
            SHM_ECAT_SERVICE_SERVER_OFFLINE;

        g_pEtherCatServiceData
            ->ResultCode =
            SHM_ECAT_SERVICE_RESULT_NONE;

        DEBUG_PRINT(
            "[ECAT-SERVICE-SHM] READY | "
            "Name:OSCARMAX_ECAT_SERVICE | "
            "Magic:0x%08X | Version:%u.%u | "
            "Size:%u B | "
            "Server:OFFLINE_WAIT_BRIDGE | "
            "SDOReadTransport:CONTRACT_ONLY\n",
            static_cast<unsigned int>(
                g_pEtherCatServiceData->Magic),
            static_cast<unsigned int>(
                g_pEtherCatServiceData
                ->VersionMajor),
            static_cast<unsigned int>(
                g_pEtherCatServiceData
                ->VersionMinor),
            static_cast<unsigned int>(
                g_pEtherCatServiceData
                ->StructSize));

        return true;
    }


    void ShutdownEtherCatServiceSharedMemory()
    {
        if (
            g_hEtherCatServiceShm !=
            NULL)
        {
            if (
                g_pEtherCatServiceData !=
                nullptr)
            {
                g_pEtherCatServiceData
                    ->ServerState =
                    SHM_ECAT_SERVICE_SERVER_OFFLINE;
            }

            RtCloseHandle(
                g_hEtherCatServiceShm);

            g_hEtherCatServiceShm =
                NULL;

            g_pEtherCatServiceData =
                nullptr;

            DEBUG_PRINT(
                "[ECAT-SERVICE-SHM] "
                "Shared Memory Closed.\n");
        }
    }
}


// =====================================================================
// Stage 17A.3C.1 - DC Qualification Diagnostic Shared Memory
// =====================================================================

namespace
{
    bool InitializeEtherCatDcQualificationDiagSharedMemory()
    {
        if (g_pEtherCatDcQualDiagData != nullptr)
        {
            return true;
        }


        DEBUG_PRINT(
            "[ECAT-DC-QUAL-SHM] Initializing "
            "OSCARMAX_ECAT_DC_QUAL_DIAG\n");


        void* pLocation =
            nullptr;


        g_hEtherCatDcQualDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_DcQualDiagData),
                ECAT_DC_QUAL_DIAG_SHM_NAME,
                &pLocation);


        if (g_hEtherCatDcQualDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-DC-QUAL-SHM] WARNING | "
                "Create failed.\n");


            if (g_hEtherCatDcQualDiagShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatDcQualDiagShm);

                g_hEtherCatDcQualDiagShm =
                    NULL;
            }


            g_pEtherCatDcQualDiagData =
                nullptr;

            return false;
        }


        g_pEtherCatDcQualDiagData =
            static_cast<SHM_ECAT_DcQualDiagData*>(
                pLocation);


        std::memset(
            g_pEtherCatDcQualDiagData,
            0,
            sizeof(SHM_ECAT_DcQualDiagData));


        g_pEtherCatDcQualDiagData->Header.Magic =
            SHM_ECAT_DC_QUAL_DIAG_MAGIC;

        g_pEtherCatDcQualDiagData->Header.VersionMajor =
            SHM_ECAT_DC_QUAL_DIAG_VERSION_MAJOR;

        g_pEtherCatDcQualDiagData->Header.VersionMinor =
            SHM_ECAT_DC_QUAL_DIAG_VERSION_MINOR;

        g_pEtherCatDcQualDiagData->Header.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_DcQualDiagData));

        g_pEtherCatDcQualDiagData->Header.Capacity =
            SHM_ECAT_DC_QUAL_DIAG_CAPACITY;

        g_pEtherCatDcQualDiagData->Header.Reserved32[0] =
            static_cast<uint32_t>(
                SHM_ECAT_DC_QUAL_DIAG_NEAR_MAD_PPB);

        g_pEtherCatDcQualDiagData->Header.Reserved32[1] =
            static_cast<uint32_t>(
                SHM_ECAT_DC_QUAL_DIAG_BAD_MAD_PPB);


        DEBUG_PRINT(
            "[ECAT-DC-QUAL-SHM] ACTIVE | "
            "Size:%zu B | Capacity:%u | "
            "Near:%d ppb | Bad:%d ppb\n",
            sizeof(SHM_ECAT_DcQualDiagData),
            static_cast<unsigned int>(
                SHM_ECAT_DC_QUAL_DIAG_CAPACITY),
            SHM_ECAT_DC_QUAL_DIAG_NEAR_MAD_PPB,
            SHM_ECAT_DC_QUAL_DIAG_BAD_MAD_PPB);


        return true;
    }


    void ShutdownEtherCatDcQualificationDiagSharedMemory()
    {
        g_pEtherCatDcQualDiagData =
            nullptr;


        if (g_hEtherCatDcQualDiagShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatDcQualDiagShm);

            g_hEtherCatDcQualDiagShm =
                NULL;


            DEBUG_PRINT(
                "[ECAT-DC-QUAL-SHM] Closed.\n");
        }
    }
}


// =====================================================================
// Stage 17A.5.1 - DC V1A Observer Diagnostic Shared Memory
// =====================================================================

namespace
{
    bool InitializeEtherCatDcV1aDiagSharedMemory()
    {
        if (g_pEtherCatDcV1aDiagData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-DC-V1A-SHM] Initializing "
            "OSCARMAX_ECAT_DC_V1A_DIAG\n");

        void* pLocation =
            nullptr;

        g_hEtherCatDcV1aDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_DcV1aDiagData),
                ECAT_DC_V1A_DIAG_SHM_NAME,
                &pLocation);

        if (g_hEtherCatDcV1aDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-DC-V1A-SHM] WARNING | Create failed.\n");

            if (g_hEtherCatDcV1aDiagShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatDcV1aDiagShm);

                g_hEtherCatDcV1aDiagShm =
                    NULL;
            }

            g_pEtherCatDcV1aDiagData =
                nullptr;

            return false;
        }

        g_pEtherCatDcV1aDiagData =
            static_cast<SHM_ECAT_DcV1aDiagData*>(
                pLocation);

        std::memset(
            g_pEtherCatDcV1aDiagData,
            0,
            sizeof(SHM_ECAT_DcV1aDiagData));

        g_pEtherCatDcV1aDiagData->Header.Magic =
            SHM_ECAT_DC_V1A_DIAG_MAGIC;

        g_pEtherCatDcV1aDiagData->Header.VersionMajor =
            SHM_ECAT_DC_V1A_DIAG_VERSION_MAJOR;

        g_pEtherCatDcV1aDiagData->Header.VersionMinor =
            SHM_ECAT_DC_V1A_DIAG_VERSION_MINOR;

        g_pEtherCatDcV1aDiagData->Header.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_DcV1aDiagData));

        g_pEtherCatDcV1aDiagData->Header.Capacity =
            SHM_ECAT_DC_V1A_DIAG_CAPACITY;

        DEBUG_PRINT(
            "[ECAT-DC-V1A-SHM] ACTIVE | "
            "Size:%zu B | Capacity:%u\n",
            sizeof(SHM_ECAT_DcV1aDiagData),
            static_cast<unsigned int>(
                SHM_ECAT_DC_V1A_DIAG_CAPACITY));

        return true;
    }


    void ShutdownEtherCatDcV1aDiagSharedMemory()
    {
        g_pEtherCatDcV1aDiagData =
            nullptr;

        if (g_hEtherCatDcV1aDiagShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatDcV1aDiagShm);

            g_hEtherCatDcV1aDiagShm =
                NULL;

            DEBUG_PRINT(
                "[ECAT-DC-V1A-SHM] Closed.\n");
        }
    }
}


// =====================================================================
// Stage 17A.8.1 - RX / V1A Correlation Diagnostic Shared Memory
// =====================================================================

namespace
{
    bool InitializeEtherCatRxCorrelationDiagSharedMemory()
    {
        if (g_pEtherCatRxCorrDiagData != nullptr)
        {
            return true;
        }


        DEBUG_PRINT(
            "[ECAT-RX-CORR-SHM] Initializing "
            "OSCARMAX_ECAT_RX_CORR_DIAG\n");


        void* pLocation =
            nullptr;


        g_hEtherCatRxCorrDiagShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_RxCorrDiagData),
                ECAT_RX_CORR_DIAG_SHM_NAME,
                &pLocation);


        if (g_hEtherCatRxCorrDiagShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-RX-CORR-SHM] WARNING | Create failed.\n");


            if (g_hEtherCatRxCorrDiagShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatRxCorrDiagShm);

                g_hEtherCatRxCorrDiagShm =
                    NULL;
            }


            g_pEtherCatRxCorrDiagData =
                nullptr;

            return false;
        }


        g_pEtherCatRxCorrDiagData =
            static_cast<SHM_ECAT_RxCorrDiagData*>(
                pLocation);


        std::memset(
            g_pEtherCatRxCorrDiagData,
            0,
            sizeof(SHM_ECAT_RxCorrDiagData));


        g_pEtherCatRxCorrDiagData->Header.Magic =
            SHM_ECAT_RX_CORR_DIAG_MAGIC;

        g_pEtherCatRxCorrDiagData->Header.VersionMajor =
            SHM_ECAT_RX_CORR_DIAG_VERSION_MAJOR;

        g_pEtherCatRxCorrDiagData->Header.VersionMinor =
            SHM_ECAT_RX_CORR_DIAG_VERSION_MINOR;

        g_pEtherCatRxCorrDiagData->Header.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_RxCorrDiagData));

        g_pEtherCatRxCorrDiagData->Header.Capacity =
            SHM_ECAT_RX_CORR_DIAG_CAPACITY;


        DEBUG_PRINT(
            "[ECAT-RX-CORR-SHM] ACTIVE | "
            "Size:%zu B | Capacity:%u | Cadence:~1s\n",
            sizeof(SHM_ECAT_RxCorrDiagData),
            static_cast<unsigned int>(
                SHM_ECAT_RX_CORR_DIAG_CAPACITY));


        return true;
    }


    void ShutdownEtherCatRxCorrelationDiagSharedMemory()
    {
        g_pEtherCatRxCorrDiagData =
            nullptr;


        if (g_hEtherCatRxCorrDiagShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatRxCorrDiagShm);

            g_hEtherCatRxCorrDiagShm =
                NULL;


            DEBUG_PRINT(
                "[ECAT-RX-CORR-SHM] Closed.\n");
        }
    }
}


// =====================================================================
// DC-RX.4B - RX Incident Forensics Shared Memory
// =====================================================================

namespace
{
    bool InitializeEtherCatRxForensicsSharedMemory()
    {
        if (g_pEtherCatRxForensicsData != nullptr)
        {
            return true;
        }

        DEBUG_PRINT(
            "[ECAT-RX-FORENSICS-SHM] Initializing "
            "OSCARMAX_ECAT_RX_FORENSICS\n");

        void* pLocation =
            nullptr;

        g_hEtherCatRxForensicsShm =
            RtCreateSharedMemory(
                PAGE_READWRITE,
                0,
                sizeof(SHM_ECAT_RxForensicsData),
                ECAT_RX_FORENSICS_SHM_NAME,
                &pLocation);

        if (g_hEtherCatRxForensicsShm == NULL ||
            pLocation == nullptr)
        {
            DEBUG_PRINT(
                "[ECAT-RX-FORENSICS-SHM] WARNING | "
                "Create failed. RX forensics UI/package unavailable; "
                "machine runtime will continue.\n");

            if (g_hEtherCatRxForensicsShm != NULL)
            {
                RtCloseHandle(
                    g_hEtherCatRxForensicsShm);

                g_hEtherCatRxForensicsShm =
                    NULL;
            }

            g_pEtherCatRxForensicsData =
                nullptr;

            return false;
        }

        g_pEtherCatRxForensicsData =
            static_cast<SHM_ECAT_RxForensicsData*>(
                pLocation);

        std::memset(
            g_pEtherCatRxForensicsData,
            0,
            sizeof(SHM_ECAT_RxForensicsData));

        g_pEtherCatRxForensicsData->Header.Magic =
            SHM_ECAT_RX_FORENSICS_MAGIC;

        g_pEtherCatRxForensicsData->Header.VersionMajor =
            SHM_ECAT_RX_FORENSICS_VERSION_MAJOR;

        g_pEtherCatRxForensicsData->Header.VersionMinor =
            SHM_ECAT_RX_FORENSICS_VERSION_MINOR;

        g_pEtherCatRxForensicsData->Header.StructSize =
            static_cast<uint32_t>(
                sizeof(SHM_ECAT_RxForensicsData));

        g_pEtherCatRxForensicsData->Header.IncidentCapacity =
            SHM_ECAT_RX_FORENSICS_INCIDENT_CAPACITY;

        g_pEtherCatRxForensicsData->Header.SlaveCapacity =
            SHM_ECAT_RX_FORENSICS_SLAVE_CAPACITY;

        DEBUG_PRINT(
            "[ECAT-RX-FORENSICS-SHM] ACTIVE | "
            "Size:%zu B | Incident:%u | Slave:%u | "
            "Publisher:P50 read-only bridge\n",
            sizeof(SHM_ECAT_RxForensicsData),
            static_cast<unsigned int>(
                SHM_ECAT_RX_FORENSICS_INCIDENT_CAPACITY),
            static_cast<unsigned int>(
                SHM_ECAT_RX_FORENSICS_SLAVE_CAPACITY));

        return true;
    }


    void ShutdownEtherCatRxForensicsSharedMemory()
    {
        g_pEtherCatRxForensicsData =
            nullptr;

        if (g_hEtherCatRxForensicsShm != NULL)
        {
            RtCloseHandle(
                g_hEtherCatRxForensicsShm);

            g_hEtherCatRxForensicsShm =
                NULL;

            DEBUG_PRINT(
                "[ECAT-RX-FORENSICS-SHM] Closed.\n");
        }
    }
}


// =====================================================================
// Producer-side Shared Memory accessors
// =====================================================================

SHM_ECAT_DiagData*
GetEtherCatDiagSharedMemoryData()
{
    return
        g_pEtherCatDiagData;
}


SHM_ECAT_ServiceData*
GetEtherCatServiceSharedMemoryData()
{
    return
        g_pEtherCatServiceData;
}


SHM_ECAT_SmDiagData*
GetEtherCatSmDiagSharedMemoryData()
{
    return
        g_pEtherCatSmDiagData;
}


SHM_ECAT_FmmuDiagData*
GetEtherCatFmmuDiagSharedMemoryData()
{
    return
        g_pEtherCatFmmuDiagData;
}


SHM_ECAT_WatchdogDiagData*
GetEtherCatWatchdogDiagSharedMemoryData()
{
    return
        g_pEtherCatWatchdogDiagData;
}


SHM_ECAT_DcQualDiagData*
GetEtherCatDcQualificationDiagSharedMemoryData()
{
    return
        g_pEtherCatDcQualDiagData;
}


SHM_ECAT_DcV1aDiagData*
GetEtherCatDcV1aDiagSharedMemoryData()
{
    return
        g_pEtherCatDcV1aDiagData;
}


SHM_ECAT_RxCorrDiagData*
GetEtherCatRxCorrelationDiagSharedMemoryData()
{
    return
        g_pEtherCatRxCorrDiagData;
}


SHM_ECAT_RxForensicsData*
GetEtherCatRxForensicsSharedMemoryData()
{
    return
        g_pEtherCatRxForensicsData;
}


// =====================================================================
// SHMManager
// =====================================================================

bool SHMManager::Initialize(
    const std::string& shmName)
{
    if (m_pData != nullptr)
    {
        InitializeEtherCatDiagSharedMemory();

        InitializeEtherCatSmDiagSharedMemory();

        InitializeEtherCatFmmuDiagSharedMemory();

        InitializeEtherCatWatchdogDiagSharedMemory();

        InitializeEtherCatDcQualificationDiagSharedMemory();

        InitializeEtherCatDcV1aDiagSharedMemory();

        InitializeEtherCatRxCorrelationDiagSharedMemory();

        InitializeEtherCatRxForensicsSharedMemory();

        InitializeEtherCatServiceSharedMemory();

        return true;
    }


    DEBUG_PRINT(
        "[SHM] Initializing Shared Memory: %s\n",
        shmName.c_str());

    DEBUG_PRINT(
        "[SHM] SHM_Data Size: %zu Bytes\n",
        sizeof(SHM_Data));


    std::wstring wName(
        shmName.begin(),
        shmName.end());


    void* pLocation =
        nullptr;


    m_hShm =
        RtCreateSharedMemory(
            PAGE_READWRITE,
            0,
            sizeof(SHM_Data),
            wName.c_str(),
            &pLocation);


    if (
        m_hShm == NULL ||
        pLocation == nullptr)
    {
        DEBUG_PRINT(
            "[SHM] Failed to create "
            "shared memory!\n");

        if (m_hShm != NULL)
        {
            RtCloseHandle(
                m_hShm);

            m_hShm =
                NULL;
        }

        m_pData =
            nullptr;

        return false;
    }


    m_pData =
        static_cast<
        SHM_Data*>(
            pLocation);


    std::memset(
        m_pData,
        0,
        sizeof(SHM_Data));


    DEBUG_PRINT(
        "[SHM] RTX64 Shared Memory "
        "Initialized Successfully!\n");


    InitializeEtherCatDiagSharedMemory();

    InitializeEtherCatSmDiagSharedMemory();

    InitializeEtherCatFmmuDiagSharedMemory();

    InitializeEtherCatWatchdogDiagSharedMemory();

    InitializeEtherCatDcQualificationDiagSharedMemory();

    InitializeEtherCatDcV1aDiagSharedMemory();

    InitializeEtherCatRxCorrelationDiagSharedMemory();

    InitializeEtherCatRxForensicsSharedMemory();

    InitializeEtherCatServiceSharedMemory();


    return true;
}


void SHMManager::Shutdown()
{
    ShutdownEtherCatServiceSharedMemory();

    ShutdownEtherCatRxForensicsSharedMemory();

    ShutdownEtherCatRxCorrelationDiagSharedMemory();

    ShutdownEtherCatDcV1aDiagSharedMemory();

    ShutdownEtherCatDcQualificationDiagSharedMemory();

    ShutdownEtherCatWatchdogDiagSharedMemory();

    ShutdownEtherCatFmmuDiagSharedMemory();

    ShutdownEtherCatSmDiagSharedMemory();

    ShutdownEtherCatDiagSharedMemory();


    if (m_hShm != NULL)
    {
        RtCloseHandle(
            m_hShm);

        m_hShm =
            NULL;

        m_pData =
            nullptr;

        DEBUG_PRINT(
            "[SHM] Shared Memory Closed.\n");
    }
}
