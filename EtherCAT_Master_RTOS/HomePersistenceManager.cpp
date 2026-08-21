#include "HomePersistenceManager.h"

#include "MotionCore.h"
#include "HomeTypes.h"
#include "NCManager.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>
#include <windows.h>
#include <rtapi.h>
#include <rtssapi.h>


namespace
{
    constexpr uint64_t FNV_OFFSET_BASIS =
        14695981039346656037ull;

    constexpr uint64_t FNV_PRIME =
        1099511628211ull;


    void HashBytes(
        uint64_t& hash,
        const void* data,
        size_t size)
    {
        const unsigned char* bytes =
            static_cast<const unsigned char*>(
                data);

        for (size_t i = 0;
            i < size;
            ++i)
        {
            hash ^=
                static_cast<uint64_t>(
                    bytes[i]);

            hash *=
                FNV_PRIME;
        }
    }


    template <typename T>
    void HashValue(
        uint64_t& hash,
        const T& value)
    {
        HashBytes(
            hash,
            &value,
            sizeof(T));
    }


    void HashBool(
        uint64_t& hash,
        bool value)
    {
        const uint8_t raw =
            value
            ? 1u
            : 0u;

        HashValue(
            hash,
            raw);
    }




    const char* HomeMethodName(
        int value)
    {
        switch (
            static_cast<HomeMethod>(
                value))
        {
        case HomeMethod::DOG_INDEX:
            return "DOG_INDEX";

        case HomeMethod::LIMIT_INDEX:
            return "LIMIT_INDEX";

        case HomeMethod::DOG_ONLY:
            return "DOG_ONLY";

        case HomeMethod::LIMIT_ONLY:
            return "LIMIT_ONLY";

        case HomeMethod::INDEX_ONLY:
            return "INDEX_ONLY";

        case HomeMethod::ABSOLUTE_REFERENCE:
            return "ABSOLUTE_REFERENCE";

        case HomeMethod::CURRENT_POSITION:
            return "CURRENT_POSITION";

        case HomeMethod::MECHANICAL_STOP:
            return "MECHANICAL_STOP";

        default:
            return "UNKNOWN";
        }
    }


    const char* ReferenceSourceName(
        int value)
    {
        switch (
            static_cast<HomeReferenceSource>(
                value))
        {
        case HomeReferenceSource::NONE:
            return "NONE";

        case HomeReferenceSource::MOTOR_ENCODER_INDEX:
            return "MOTOR_ENCODER_INDEX";

        case HomeReferenceSource::LINEAR_SCALE_INDEX_DRIVE:
            return "LINEAR_SCALE_INDEX_DRIVE";

        case HomeReferenceSource::EXTERNAL_IO_INDEX:
            return "EXTERNAL_IO_INDEX";

        case HomeReferenceSource::ABSOLUTE_MOTOR_ENCODER:
            return "ABSOLUTE_MOTOR_ENCODER";

        case HomeReferenceSource::ABSOLUTE_LINEAR_SCALE:
            return "ABSOLUTE_LINEAR_SCALE";

        default:
            return "UNKNOWN";
        }
    }


    const char* CaptureModeName(
        int value)
    {
        switch (
            static_cast<HomeReferenceCaptureMode>(
                value))
        {
        case HomeReferenceCaptureMode::DRIVE_HARDWARE_LATCH:
            return "DRIVE_HARDWARE_LATCH";

        case HomeReferenceCaptureMode::EXTERNAL_HARDWARE_LATCH:
            return "EXTERNAL_HARDWARE_LATCH";

        case HomeReferenceCaptureMode::SOFTWARE_SAMPLE:
            return "SOFTWARE_SAMPLE";

        default:
            return "UNKNOWN";
        }
    }


    bool UsesSwitch(
        HomeMethod method)
    {
        return
            method == HomeMethod::DOG_INDEX ||
            method == HomeMethod::LIMIT_INDEX ||
            method == HomeMethod::DOG_ONLY ||
            method == HomeMethod::LIMIT_ONLY;
    }


    bool UsesIndex(
        HomeMethod method)
    {
        return
            method == HomeMethod::DOG_INDEX ||
            method == HomeMethod::LIMIT_INDEX ||
            method == HomeMethod::INDEX_ONLY;
    }


    // ========================================================
    // RTSS-safe file copy
    //
    // 不使用不在 RTSS 支援清單中的 Win32 檔案管理 API。
    // 使用專案既有的 C++ stream file I/O 模式。
    // ========================================================

    // ========================================================
    // RTX64 Supported File I/O Helpers
    //
    // 使用 RTX64 官方支援：
    //
    // CreateFileA
    // ReadFile
    // WriteFile
    // SetFilePointer
    // CloseHandle
    //
    // 所有路徑皆使用完整絕對路徑。
    // ========================================================

    bool WriteBufferRtss(
        HANDLE handle,
        const char* data,
        size_t size)
    {
        if (handle ==
            INVALID_HANDLE_VALUE ||
            data == nullptr)
        {
            return false;
        }


        size_t offset =
            0;


        while (offset <
            size)
        {
            const size_t remaining =
                size -
                offset;

            const DWORD chunk =
                static_cast<DWORD>(
                    (std::min)(
                        remaining,
                        static_cast<size_t>(
                            64 * 1024)));


            DWORD written =
                0;


            const BOOL ok =
                WriteFile(
                    handle,
                    data +
                    offset,
                    chunk,
                    &written,
                    nullptr);


            if (!ok ||
                written !=
                chunk)
            {
                return false;
            }


            offset +=
                static_cast<size_t>(
                    written);
        }


        return true;
    }


    bool ReadAllTextRtss(
        const std::string& path,
        std::string& outText)
    {
        outText.clear();


        HANDLE handle =
            CreateFileA(
                path.c_str(),
                GENERIC_READ,
                FILE_SHARE_READ |
                FILE_SHARE_WRITE,
                nullptr,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);


        if (handle ==
            INVALID_HANDLE_VALUE)
        {
            return false;
        }


        char buffer[4096];


        while (true)
        {
            DWORD bytesRead =
                0;


            const BOOL ok =
                ReadFile(
                    handle,
                    buffer,
                    static_cast<DWORD>(
                        sizeof(buffer)),
                    &bytesRead,
                    nullptr);


            if (!ok)
            {
                CloseHandle(
                    handle);

                return false;
            }


            if (bytesRead ==
                0)
            {
                break;
            }


            outText.append(
                buffer,
                buffer +
                bytesRead);
        }


        CloseHandle(
            handle);


        return true;
    }


    bool CopyFileRtss(
        const std::string& sourcePath,
        const std::string& destinationPath)
    {
        HANDLE source =
            CreateFileA(
                sourcePath.c_str(),
                GENERIC_READ,
                FILE_SHARE_READ |
                FILE_SHARE_WRITE,
                nullptr,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);


        if (source ==
            INVALID_HANDLE_VALUE)
        {
            return false;
        }


        HANDLE destination =
            CreateFileA(
                destinationPath.c_str(),
                GENERIC_WRITE,
                FILE_SHARE_READ,
                nullptr,
                CREATE_ALWAYS,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);


        if (destination ==
            INVALID_HANDLE_VALUE)
        {
            CloseHandle(
                source);

            return false;
        }


        bool success =
            true;

        char buffer[4096];


        while (true)
        {
            DWORD bytesRead =
                0;


            if (!ReadFile(
                source,
                buffer,
                static_cast<DWORD>(
                    sizeof(buffer)),
                &bytesRead,
                nullptr))
            {
                success =
                    false;

                break;
            }


            if (bytesRead ==
                0)
            {
                break;
            }


            if (!WriteBufferRtss(
                destination,
                buffer,
                bytesRead))
            {
                success =
                    false;

                break;
            }
        }


        CloseHandle(
            destination);

        CloseHandle(
            source);


        return success;
    }

}


// ============================================================
// Singleton
// ============================================================

HomePersistenceManager&
HomePersistenceManager::GetInstance()
{
    static HomePersistenceManager instance;

    return instance;
}


// ============================================================
// Initialize / Load
// ============================================================

void HomePersistenceManager::Initialize(
    const std::string& ncDataDir,
    std::vector<AxisContext>& axes,
    const NCManager* ncManager)
{
    m_dataDir =
        NormalizeDirectory(
            ncDataDir);

    m_snapshotPath =
        m_dataDir +
        "HomeSnapshot.txt";

    m_snapshotBackupPath =
        m_dataDir +
        "HomeSnapshot.bak.txt";

    m_historyPath =
        m_dataDir +
        "HomeHistory.csv";


    m_lastSnapshots.fill(
        HomeSnapshotRecord{});

    m_pendingHistory.clear();

    m_snapshotDirty =
        false;

    m_nextSequenceNumber =
        1;


    // ========================================================
    // Dynamic Axis Mapping
    //
    // 唯一來源是 NCManager 已由 AXIS_CFG.ini 載入的 m_axisNames。
    // Persistence 不再維護另一套固定 XYZABCUV。
    // ========================================================

    if (ncManager != nullptr)
    {
        for (int axisIndex = 0;
            axisIndex < HOME_AXIS_COUNT;
            ++axisIndex)
        {
            const char axisName =
                ncManager->GetAxisName(
                    axisIndex);

            if (axisName != '?')
            {
                m_axisNames[
                    axisIndex] =
                    axisName;
            }
        }
    }


    LoadSnapshotFile();


    // ========================================================
    // 安全規則：
    //
    // 上次 Snapshot 只供診斷，不可恢復本次 Homed。
    //
    // Core 每次啟動都必須重新 G81。
    // ========================================================

    for (AxisContext& axis : axes)
    {
        axis.isHomed =
            false;

        axis.machineCoordinateOffsetPulse =
            0.0;
    }


    m_initialized =
        true;


    RtPrintf(
        "[HOME-PERSIST] Initialized. DataDir:%s Snapshot:%s History:%s\n",
        m_dataDir.c_str(),
        m_snapshotPath.c_str(),
        m_historyPath.c_str());


    RtPrintf(
        "[HOME-PERSIST] AxisMap: 0=%c 1=%c 2=%c 3=%c 4=%c 5=%c 6=%c 7=%c\n",
        m_axisNames[0],
        m_axisNames[1],
        m_axisNames[2],
        m_axisNames[3],
        m_axisNames[4],
        m_axisNames[5],
        m_axisNames[6],
        m_axisNames[7]);
}


bool HomePersistenceManager::LoadSnapshotFile()
{
    if (!FileExists(
        m_snapshotPath))
    {
        return false;
    }


    std::string fileText;

    if (!ReadAllTextRtss(
        m_snapshotPath,
        fileText))
    {
        RtPrintf(
            "[HOME-PERSIST] Load snapshot failed. Error:%lu Path:%s\n",
            static_cast<unsigned long>(
                GetLastError()),
            m_snapshotPath.c_str());

        return false;
    }


    std::istringstream in(
        fileText);


    std::map<std::string, std::string>
        values;


    std::string line;

    while (std::getline(
        in,
        line))
    {
        const size_t commentPos =
            line.find("//");

        if (commentPos !=
            std::string::npos)
        {
            line =
                line.substr(
                    0,
                    commentPos);
        }


        line =
            Trim(
                line);


        if (line.empty() ||
            line[0] == '#' ||
            line[0] == ';')
        {
            continue;
        }


        const size_t equals =
            line.find('=');

        if (equals ==
            std::string::npos)
        {
            continue;
        }


        const std::string key =
            Trim(
                line.substr(
                    0,
                    equals));

        const std::string value =
            Trim(
                line.substr(
                    equals + 1));


        if (!key.empty())
        {
            values[key] =
                value;
        }
    }


    uint64_t maxSequence =
        0;


    for (int axisIndex = 0;
        axisIndex < HOME_AXIS_COUNT;
        ++axisIndex)
    {
        const std::string prefix =
            "Axis" +
            std::to_string(
                axisIndex) +
            "_";


        const auto findValue =
            [&](const std::string& suffix,
                const std::string& defaultValue)
            -> std::string
        {
            const auto it =
                values.find(
                    prefix +
                    suffix);

            if (it ==
                values.end())
            {
                return defaultValue;
            }

            return
                it->second;
        };


        HomeSnapshotRecord record;

        record.hasSnapshot =
            ParseBool(
                findValue(
                    "HasSnapshot",
                    "0"),
                false);


        if (!record.hasSnapshot)
        {
            m_lastSnapshots[
                axisIndex] =
                record;

                continue;
        }


        record.version =
            ParseInt(
                findValue(
                    "Version",
                    "1"),
                1);

        record.sequenceNumber =
            ParseUInt64(
                findValue(
                    "Sequence",
                    "0"),
                0);

        record.completedAtUtc =
            findValue(
                "CompletedAtUtc",
                "");

        record.axisIndex =
            axisIndex;

        const std::string axisNameText =
            findValue(
                "AxisName",
                std::string(
                    1,
                    m_axisNames[
                        axisIndex]));

        record.axisName =
            axisNameText.empty()
            ? m_axisNames[
                axisIndex]
            : axisNameText[0];

                record.homeMethod =
                    ParseInt(
                        findValue(
                            "HomeMethod",
                            "0"),
                        0);

                record.referenceSource =
                    ParseInt(
                        findValue(
                            "ReferenceSource",
                            "0"),
                        0);

                record.captureMode =
                    ParseInt(
                        findValue(
                            "CaptureMode",
                            "0"),
                        0);

                record.homeDirection =
                    ParseInt(
                        findValue(
                            "HomeDirection",
                            "-1"),
                        -1);

                record.homeOrder =
                    ParseInt(
                        findValue(
                            "HomeOrder",
                            "0"),
                        0);

                record.capturedReferenceRaw =
                    ParseInt32(
                        findValue(
                            "CapturedReferenceRaw",
                            "0"),
                        0);

                record.capturedReferencePulse =
                    ParseDouble(
                        findValue(
                            "CapturedReferencePulse",
                            "0"),
                        0.0);

                record.switchDetectedPulse =
                    ParseDouble(
                        findValue(
                            "SwitchDetectedPulse",
                            "0"),
                        0.0);

                record.hasSwitchToReferenceDistance =
                    ParseBool(
                        findValue(
                            "HasSwitchToReferenceDistance",
                            "0"),
                        false);

                record.switchToReferencePulse =
                    ParseDouble(
                        findValue(
                            "SwitchToReferencePulse",
                            "0"),
                        0.0);

                record.switchToReferenceUnit =
                    ParseDouble(
                        findValue(
                            "SwitchToReferenceUnit",
                            "0"),
                        0.0);

                record.machineCoordinateOffsetPulse =
                    ParseDouble(
                        findValue(
                            "MachineCoordinateOffsetPulse",
                            "0"),
                        0.0);

                record.homeOffsetUnit =
                    ParseDouble(
                        findValue(
                            "HomeOffsetUnit",
                            "0"),
                        0.0);

                record.finalRawPulse =
                    ParseDouble(
                        findValue(
                            "FinalRawPulse",
                            "0"),
                        0.0);

                record.finalMachinePositionUnit =
                    ParseDouble(
                        findValue(
                            "FinalMachinePositionUnit",
                            "0"),
                        0.0);

                record.resolutionPPR =
                    ParseDouble(
                        findValue(
                            "ResolutionPPR",
                            "0"),
                        0.0);

                record.finalLead =
                    ParseDouble(
                        findValue(
                            "FinalLead",
                            "0"),
                        0.0);

                record.moveToZeroEnabled =
                    ParseBool(
                        findValue(
                            "MoveToZeroEnabled",
                            "0"),
                        false);

                record.moveToZeroCompleted =
                    ParseBool(
                        findValue(
                            "MoveToZeroCompleted",
                            "0"),
                        false);

                record.configFingerprint =
                    ParseUInt64(
                        findValue(
                            "ConfigFingerprint",
                            "0"),
                        0);

                record.hasPreviousSnapshot =
                    ParseBool(
                        findValue(
                            "HasPreviousSnapshot",
                            "0"),
                        false);

                record.configMatchedPrevious =
                    ParseBool(
                        findValue(
                            "ConfigMatchedPrevious",
                            "0"),
                        false);

                record.hasRepeatabilityComparison =
                    ParseBool(
                        findValue(
                            "HasRepeatabilityComparison",
                            "0"),
                        false);

                record.repeatabilityDeltaUnit =
                    ParseDouble(
                        findValue(
                            "RepeatabilityDeltaUnit",
                            "0"),
                        0.0);


                m_lastSnapshots[
                    axisIndex] =
                    record;


                    maxSequence =
                        (std::max)(
                            maxSequence,
                            record.sequenceNumber);
    }


    m_nextSequenceNumber =
        maxSequence +
        1;


    return true;
}


// ============================================================
// Queue Successful HOME
// ============================================================

void HomePersistenceManager::QueueSuccessfulHome(
    int axisIndex,
    const AxisContext& axis)
{
    if (axisIndex < 0 ||
        axisIndex >= HOME_AXIS_COUNT)
    {
        return;
    }


    HomeSnapshotRecord record =
        BuildSnapshot(
            axisIndex,
            axis);


    record.sequenceNumber =
        m_nextSequenceNumber++;


    const HomeSnapshotRecord&
        previous =
        m_lastSnapshots[
            axisIndex];


    if (previous.hasSnapshot)
    {
        record.hasPreviousSnapshot =
            true;

        record.configMatchedPrevious =
            previous.configFingerprint ==
            record.configFingerprint;


        if (record.configMatchedPrevious &&
            previous.hasSwitchToReferenceDistance &&
            record.hasSwitchToReferenceDistance)
        {
            record.hasRepeatabilityComparison =
                true;

            record.repeatabilityDeltaUnit =
                record.switchToReferenceUnit -
                previous.switchToReferenceUnit;
        }
    }


    m_lastSnapshots[
        axisIndex] =
        record;


        m_pendingHistory.push_back(
            record);


        m_snapshotDirty =
            true;


        const long long switchToReferenceMicroUnit =
            static_cast<long long>(
                record.switchToReferenceUnit *
                1000000.0);


        RtPrintf(
            "[HOME-PERSIST] Queue Axis:%d(%c) Seq:%llu 60BA:%d SwitchToRef_uUnit:%lld\n",
            axisIndex,
            record.axisName,
            static_cast<unsigned long long>(
                record.sequenceNumber),
            static_cast<int>(
                record.capturedReferenceRaw),
            switchToReferenceMicroUnit);
}


// ============================================================
// Build Snapshot
// ============================================================

HomeSnapshotRecord
HomePersistenceManager::BuildSnapshot(
    int axisIndex,
    const AxisContext& axis)
{
    HomeSnapshotRecord record;

    record.hasSnapshot =
        true;

    record.completedAtUtc =
        GetUtcTimestamp();

    record.axisIndex =
        axisIndex;

    record.axisName =
        (axisIndex >= 0 &&
            axisIndex < HOME_AXIS_COUNT)
        ? m_axisNames[
            axisIndex]
        : '?';

            record.homeMethod =
                static_cast<int>(
                    axis.home.method);

            record.referenceSource =
                static_cast<int>(
                    axis.home.referenceSource);

            record.captureMode =
                static_cast<int>(
                    axis.home.captureMode);

            record.homeDirection =
                axis.home.direction;

            record.homeOrder =
                axis.home.order;

            record.capturedReferenceRaw =
                axis.homeRuntime.capturedReferenceRaw;

            record.capturedReferencePulse =
                axis.homeRuntime.capturedReferencePulse;

            record.switchDetectedPulse =
                axis.homeRuntime.switchDetectedPulse;


            const bool usesSwitch =
                UsesSwitch(
                    axis.home.method);

            const bool usesIndex =
                UsesIndex(
                    axis.home.method);


            record.hasSwitchToReferenceDistance =
                usesSwitch &&
                usesIndex;


            if (record.hasSwitchToReferenceDistance)
            {
                record.switchToReferencePulse =
                    record.capturedReferencePulse -
                    record.switchDetectedPulse;
            }


            const double leadAbs =
                std::abs(
                    axis.finalLead);


            const double pulsePerUnit =
                (axis.resolution_PPR > 0.0 &&
                    leadAbs > 1.0e-12)
                ? axis.resolution_PPR /
                leadAbs
                : 0.0;


            if (record.hasSwitchToReferenceDistance &&
                pulsePerUnit > 0.0)
            {
                record.switchToReferenceUnit =
                    record.switchToReferencePulse /
                    pulsePerUnit;
            }


            record.machineCoordinateOffsetPulse =
                axis.machineCoordinateOffsetPulse;

            record.homeOffsetUnit =
                axis.home.homeOffset_unit;


            // currentActPos = Machine Pulse
            // Raw = Machine + MachineOffset
            record.finalRawPulse =
                axis.currentActPos +
                axis.machineCoordinateOffsetPulse;


            if (pulsePerUnit > 0.0)
            {
                record.finalMachinePositionUnit =
                    axis.currentActPos /
                    pulsePerUnit;
            }


            record.resolutionPPR =
                axis.resolution_PPR;

            record.finalLead =
                axis.finalLead;

            record.moveToZeroEnabled =
                axis.home.moveToZero;

            record.moveToZeroCompleted =
                !axis.home.moveToZero ||
                std::abs(
                    record.finalMachinePositionUnit) <=
                0.001;


            record.configFingerprint =
                ComputeHomeConfigFingerprint(
                    axisIndex,
                    axis);


            return record;
}


// ============================================================
// HOME Config Fingerprint
// ============================================================

uint64_t
HomePersistenceManager::ComputeHomeConfigFingerprint(
    int axisIndex,
    const AxisContext& axis) const
{
    uint64_t hash =
        FNV_OFFSET_BASIS;


    // Axis Letter 是機台組態的一部分。
    // 例如 Axis3=C 改成 Axis3=A 必須視為 Config Changed。
    const char axisName =
        (axisIndex >= 0 &&
            axisIndex < HOME_AXIS_COUNT)
        ? m_axisNames[
            axisIndex]
        : '?';

            HashValue(
                hash,
                axisName);


            const int method =
                static_cast<int>(
                    axis.home.method);

            const int source =
                static_cast<int>(
                    axis.home.referenceSource);

            const int capture =
                static_cast<int>(
                    axis.home.captureMode);

            const int backoffMode =
                static_cast<int>(
                    axis.home.backoffMode);

            const int driveProbeArmMode =
                static_cast<int>(
                    axis.home.driveProbeArmMode);


            HashValue(hash, method);
            HashValue(hash, source);
            HashValue(hash, capture);
            HashValue(hash, axis.home.direction);
            HashValue(hash, axis.home.order);

            HashBool(hash, axis.home.dogActiveHigh);
            HashBool(hash, axis.home.referenceActiveHigh);
            HashValue(hash, axis.home.externalReferenceCPoint);

            HashValue(hash, axis.home.searchSpeed_PPS);
            HashValue(hash, axis.home.searchAccTime);
            HashValue(hash, axis.home.searchDecTime);
            HashValue(hash, axis.home.searchMaxDistance_unit);
            HashValue(hash, axis.home.searchTimeoutSec);

            HashValue(hash, axis.home.switchStopDecTime);
            HashValue(hash, axis.home.switchStopMaxDistance_unit);

            HashValue(hash, backoffMode);
            HashValue(hash, axis.home.backoffDistance_unit);
            HashValue(hash, axis.home.backoffExtraDistance_unit);
            HashValue(hash, axis.home.backoffSpeed_PPS);
            HashValue(hash, axis.home.backoffAccTime);
            HashValue(hash, axis.home.backoffDecTime);
            HashValue(hash, axis.home.backoffMaxDistance_unit);
            HashValue(hash, axis.home.backoffTimeoutSec);

            HashBool(
                hash,
                axis.home.alarmIfDogNotReleasedBeforeIndex);

            HashBool(
                hash,
                axis.home.alarmIfHardLimitNotReleasedBeforeIndex);

            HashValue(hash, driveProbeArmMode);
            HashValue(hash, axis.home.driveProbeDisarmValue);
            HashValue(hash, axis.home.driveProbeArmValue);
            HashValue(hash, axis.home.driveProbeArmedMask);
            HashValue(hash, axis.home.driveProbeCapturedMask);
            HashValue(hash, axis.home.driveProbeCaptureToggleMask);
            HashValue(hash, axis.home.driveProbeSourceMask);
            HashValue(hash, axis.home.driveProbeExpectedSourceValue);
            HashValue(hash, axis.home.driveProbeClearTimeoutSec);
            HashValue(hash, axis.home.driveProbeArmTimeoutSec);

            HashBool(
                hash,
                axis.home.driveProbeRequireArmedStatus);

            HashBool(
                hash,
                axis.home.driveProbeDisarmAfterCapture);

            HashBool(
                hash,
                axis.home.driveProbeRequireNewCapture);

            HashBool(
                hash,
                axis.home.driveProbeAllowPositionChangeDetection);

            HashValue(hash, axis.home.indexSearchSpeed_PPS);
            HashValue(hash, axis.home.indexSearchAccTime);
            HashValue(hash, axis.home.indexStopDecTime);
            HashValue(hash, axis.home.indexMaxDistance_unit);
            HashValue(hash, axis.home.indexTimeoutSec);

            HashValue(hash, axis.home.homeOffset_unit);

            HashBool(hash, axis.home.moveToZero);
            HashValue(hash, axis.home.moveToZeroSpeed_PPS);
            HashValue(hash, axis.home.moveToZeroAccTime);
            HashValue(hash, axis.home.moveToZeroDecTime);

            HashValue(hash, axis.home.searchGain.Kp);
            HashValue(hash, axis.home.searchGain.Ki);
            HashValue(hash, axis.home.searchGain.Kd);
            HashValue(hash, axis.home.searchGain.Kvff);

            HashValue(hash, axis.home.indexGain.Kp);
            HashValue(hash, axis.home.indexGain.Ki);
            HashValue(hash, axis.home.indexGain.Kd);
            HashValue(hash, axis.home.indexGain.Kvff);

            HashValue(hash, axis.resolution_PPR);
            HashValue(hash, axis.finalLead);


            return hash;
}


// ============================================================
// Flush
// ============================================================

bool HomePersistenceManager::FlushPending()
{
    if (!m_initialized)
    {
        return false;
    }


    if (!m_snapshotDirty &&
        m_pendingHistory.empty())
    {
        return true;
    }


    if (m_snapshotDirty)
    {
        if (!WriteSnapshotFileAtomic())
        {
            RtPrintf(
                "[HOME-PERSIST] Flush failed at Snapshot.\n");

            return false;
        }
    }


    if (!m_pendingHistory.empty())
    {
        if (!AppendPendingHistory())
        {
            RtPrintf(
                "[HOME-PERSIST] Flush failed at History.\n");

            return false;
        }
    }


    m_snapshotDirty =
        false;

    m_pendingHistory.clear();


    return true;
}


// ============================================================
// Snapshot File
// ============================================================

bool HomePersistenceManager::WriteSnapshotFileAtomic()
{
    const std::string snapshotText =
        BuildSnapshotText();


    const bool hadPreviousSnapshot =
        FileExists(
            m_snapshotPath);


    if (hadPreviousSnapshot)
    {
        if (!CopyFileRtss(
            m_snapshotPath,
            m_snapshotBackupPath))
        {
            RtPrintf(
                "[HOME-PERSIST] Snapshot backup warning. Error:%lu Path:%s\n",
                static_cast<unsigned long>(
                    GetLastError()),
                m_snapshotBackupPath.c_str());
        }
    }


    HANDLE handle =
        CreateFileA(
            m_snapshotPath.c_str(),
            GENERIC_WRITE,
            FILE_SHARE_READ,
            nullptr,
            CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL |
            FILE_FLAG_WRITE_THROUGH,
            nullptr);


    if (handle ==
        INVALID_HANDLE_VALUE)
    {
        RtPrintf(
            "[HOME-PERSIST] Snapshot open failed. Error:%lu Path:%s\n",
            static_cast<unsigned long>(
                GetLastError()),
            m_snapshotPath.c_str());

        return false;
    }


    const bool success =
        WriteBufferRtss(
            handle,
            snapshotText.data(),
            snapshotText.size());


    const DWORD error =
        success
        ? ERROR_SUCCESS
        : GetLastError();


    CloseHandle(
        handle);


    if (!success)
    {
        RtPrintf(
            "[HOME-PERSIST] Snapshot write failed. Error:%lu Path:%s\n",
            static_cast<unsigned long>(
                error),
            m_snapshotPath.c_str());

        if (hadPreviousSnapshot &&
            FileExists(
                m_snapshotBackupPath))
        {
            CopyFileRtss(
                m_snapshotBackupPath,
                m_snapshotPath);
        }

        return false;
    }


    RtPrintf(
        "[HOME-PERSIST] Snapshot saved. Path:%s\n",
        m_snapshotPath.c_str());


    return true;
}


std::string
HomePersistenceManager::BuildSnapshotText() const
{
    std::ostringstream out;

    out <<
        "// ============================================================\n"
        "// G81 HOME Last Successful Snapshot\n"
        "//\n"
        "// 此檔只保存歷史／診斷資料。\n"
        "// Core 啟動時不會用此檔將 axis.isHomed 恢復成 true。\n"
        "// 每次 Core / 機台重新啟動仍必須重新執行 G81。\n"
        "// ============================================================\n\n";

    out <<
        "SnapshotFormatVersion=1\n";

    out <<
        "AxisCount=" <<
        HOME_AXIS_COUNT <<
        "\n\n";


    out <<
        std::setprecision(17);


    for (int axisIndex = 0;
        axisIndex < HOME_AXIS_COUNT;
        ++axisIndex)
    {
        const HomeSnapshotRecord&
            record =
            m_lastSnapshots[
                axisIndex];

        const std::string prefix =
            "Axis" +
            std::to_string(
                axisIndex) +
            "_";


        out <<
            "// ------------------------------------------------------------\n";

        out <<
            "// Axis " <<
            axisIndex <<
            " - " <<
            record.axisName <<
            "\n";

        out <<
            "// ------------------------------------------------------------\n";


        out <<
            prefix <<
            "HasSnapshot=" <<
            (record.hasSnapshot ? 1 : 0) <<
            "\n";


        if (!record.hasSnapshot)
        {
            out << "\n";
            continue;
        }


        out <<
            prefix <<
            "Version=" <<
            record.version <<
            "\n";

        out <<
            prefix <<
            "Sequence=" <<
            record.sequenceNumber <<
            "\n";

        out <<
            prefix <<
            "CompletedAtUtc=" <<
            record.completedAtUtc <<
            "\n";

        out <<
            prefix <<
            "AxisName=" <<
            record.axisName <<
            " // 來源：AXIS_CFG.ini\n";

        out <<
            prefix <<
            "HomeMethod=" <<
            record.homeMethod <<
            " // " <<
            HomeMethodName(
                record.homeMethod) <<
            "\n";

        out <<
            prefix <<
            "ReferenceSource=" <<
            record.referenceSource <<
            " // " <<
            ReferenceSourceName(
                record.referenceSource) <<
            "\n";

        out <<
            prefix <<
            "CaptureMode=" <<
            record.captureMode <<
            " // " <<
            CaptureModeName(
                record.captureMode) <<
            "\n";

        out <<
            prefix <<
            "HomeDirection=" <<
            record.homeDirection <<
            "\n";

        out <<
            prefix <<
            "HomeOrder=" <<
            record.homeOrder <<
            "\n";

        out <<
            prefix <<
            "CapturedReferenceRaw=" <<
            record.capturedReferenceRaw <<
            " // Motor Encoder 模式即 0x60BA\n";

        out <<
            prefix <<
            "CapturedReferencePulse=" <<
            record.capturedReferencePulse <<
            "\n";

        out <<
            prefix <<
            "SwitchDetectedPulse=" <<
            record.switchDetectedPulse <<
            "\n";

        out <<
            prefix <<
            "HasSwitchToReferenceDistance=" <<
            (record.hasSwitchToReferenceDistance ? 1 : 0) <<
            "\n";

        out <<
            prefix <<
            "SwitchToReferencePulse=" <<
            record.switchToReferencePulse <<
            "\n";

        out <<
            prefix <<
            "SwitchToReferenceUnit=" <<
            record.switchToReferenceUnit <<
            "\n";

        out <<
            prefix <<
            "MachineCoordinateOffsetPulse=" <<
            record.machineCoordinateOffsetPulse <<
            "\n";

        out <<
            prefix <<
            "HomeOffsetUnit=" <<
            record.homeOffsetUnit <<
            "\n";

        out <<
            prefix <<
            "FinalRawPulse=" <<
            record.finalRawPulse <<
            "\n";

        out <<
            prefix <<
            "FinalMachinePositionUnit=" <<
            record.finalMachinePositionUnit <<
            "\n";

        out <<
            prefix <<
            "ResolutionPPR=" <<
            record.resolutionPPR <<
            "\n";

        out <<
            prefix <<
            "FinalLead=" <<
            record.finalLead <<
            "\n";

        out <<
            prefix <<
            "MoveToZeroEnabled=" <<
            (record.moveToZeroEnabled ? 1 : 0) <<
            "\n";

        out <<
            prefix <<
            "MoveToZeroCompleted=" <<
            (record.moveToZeroCompleted ? 1 : 0) <<
            "\n";

        out <<
            prefix <<
            "ConfigFingerprint=" <<
            ToHex64(
                record.configFingerprint) <<
            "\n";

        out <<
            prefix <<
            "HasPreviousSnapshot=" <<
            (record.hasPreviousSnapshot ? 1 : 0) <<
            "\n";

        out <<
            prefix <<
            "ConfigMatchedPrevious=" <<
            (record.configMatchedPrevious ? 1 : 0) <<
            "\n";

        out <<
            prefix <<
            "HasRepeatabilityComparison=" <<
            (record.hasRepeatabilityComparison ? 1 : 0) <<
            "\n";

        out <<
            prefix <<
            "RepeatabilityDeltaUnit=" <<
            record.repeatabilityDeltaUnit <<
            "\n\n";
    }


    return
        out.str();
}


// ============================================================
// History CSV
// ============================================================

bool HomePersistenceManager::AppendPendingHistory()
{
    const bool needsHeader =
        !FileExists(
            m_historyPath);


    HANDLE handle =
        CreateFileA(
            m_historyPath.c_str(),
            GENERIC_WRITE,
            FILE_SHARE_READ,
            nullptr,
            OPEN_ALWAYS,
            FILE_ATTRIBUTE_NORMAL,
            nullptr);


    if (handle ==
        INVALID_HANDLE_VALUE)
    {
        RtPrintf(
            "[HOME-PERSIST] History open failed. Error:%lu Path:%s\n",
            static_cast<unsigned long>(
                GetLastError()),
            m_historyPath.c_str());

        return false;
    }


    SetLastError(
        NO_ERROR);


    const DWORD pointerResult =
        SetFilePointer(
            handle,
            0,
            nullptr,
            FILE_END);


    if (pointerResult ==
        INVALID_SET_FILE_POINTER &&
        GetLastError() !=
        NO_ERROR)
    {
        const DWORD error =
            GetLastError();

        CloseHandle(
            handle);

        RtPrintf(
            "[HOME-PERSIST] History seek failed. Error:%lu\n",
            static_cast<unsigned long>(
                error));

        return false;
    }


    bool success =
        true;


    if (needsHeader)
    {
        const std::string header =
            BuildHistoryHeader();

        success =
            WriteBufferRtss(
                handle,
                header.data(),
                header.size());
    }


    if (success)
    {
        for (const HomeSnapshotRecord&
            record :
            m_pendingHistory)
        {
            const std::string row =
                BuildHistoryRow(
                    record);

            if (!WriteBufferRtss(
                handle,
                row.data(),
                row.size()))
            {
                success =
                    false;

                break;
            }
        }
    }


    const DWORD error =
        success
        ? ERROR_SUCCESS
        : GetLastError();


    CloseHandle(
        handle);


    if (!success)
    {
        RtPrintf(
            "[HOME-PERSIST] History write failed. Error:%lu Path:%s\n",
            static_cast<unsigned long>(
                error),
            m_historyPath.c_str());

        return false;
    }


    RtPrintf(
        "[HOME-PERSIST] History saved. Records:%d Path:%s\n",
        static_cast<int>(
            m_pendingHistory.size()),
        m_historyPath.c_str());


    return true;
}


std::string
HomePersistenceManager::BuildHistoryHeader() const
{
    return
        "Sequence,CompletedAtUtc,AxisIndex,AxisName,"
        "HomeMethod,HomeMethodName,ReferenceSource,ReferenceSourceName,"
        "CaptureMode,CaptureModeName,HomeDirection,HomeOrder,"
        "CapturedReferenceRaw,CapturedReferencePulse,"
        "SwitchDetectedPulse,HasSwitchToReferenceDistance,"
        "SwitchToReferencePulse,SwitchToReferenceUnit,"
        "MachineCoordinateOffsetPulse,HomeOffsetUnit,"
        "FinalRawPulse,FinalMachinePositionUnit,"
        "ResolutionPPR,FinalLead,"
        "MoveToZeroEnabled,MoveToZeroCompleted,"
        "ConfigFingerprint,HasPreviousSnapshot,"
        "ConfigMatchedPrevious,HasRepeatabilityComparison,"
        "RepeatabilityDeltaUnit\n";
}


std::string
HomePersistenceManager::BuildHistoryRow(
    const HomeSnapshotRecord& record) const
{
    std::ostringstream out;

    out <<
        std::setprecision(17);

    out <<
        record.sequenceNumber <<
        "," <<
        record.completedAtUtc <<
        "," <<
        record.axisIndex <<
        "," <<
        record.axisName <<
        "," <<
        record.homeMethod <<
        "," <<
        HomeMethodName(
            record.homeMethod) <<
        "," <<
        record.referenceSource <<
        "," <<
        ReferenceSourceName(
            record.referenceSource) <<
        "," <<
        record.captureMode <<
        "," <<
        CaptureModeName(
            record.captureMode) <<
        "," <<
        record.homeDirection <<
        "," <<
        record.homeOrder <<
        "," <<
        record.capturedReferenceRaw <<
        "," <<
        record.capturedReferencePulse <<
        "," <<
        record.switchDetectedPulse <<
        "," <<
        (record.hasSwitchToReferenceDistance ? 1 : 0) <<
        "," <<
        record.switchToReferencePulse <<
        "," <<
        record.switchToReferenceUnit <<
        "," <<
        record.machineCoordinateOffsetPulse <<
        "," <<
        record.homeOffsetUnit <<
        "," <<
        record.finalRawPulse <<
        "," <<
        record.finalMachinePositionUnit <<
        "," <<
        record.resolutionPPR <<
        "," <<
        record.finalLead <<
        "," <<
        (record.moveToZeroEnabled ? 1 : 0) <<
        "," <<
        (record.moveToZeroCompleted ? 1 : 0) <<
        "," <<
        ToHex64(
            record.configFingerprint) <<
        "," <<
        (record.hasPreviousSnapshot ? 1 : 0) <<
        "," <<
        (record.configMatchedPrevious ? 1 : 0) <<
        "," <<
        (record.hasRepeatabilityComparison ? 1 : 0) <<
        "," <<
        record.repeatabilityDeltaUnit <<
        "\n";


    return
        out.str();
}


// ============================================================
// Queries
// ============================================================

bool HomePersistenceManager::HasLastSnapshot(
    int axisIndex) const
{
    return
        axisIndex >= 0 &&
        axisIndex < HOME_AXIS_COUNT&&
        m_lastSnapshots[
            axisIndex]
        .hasSnapshot;
}


bool HomePersistenceManager::TryGetLastSnapshot(
    int axisIndex,
    HomeSnapshotRecord& outSnapshot) const
{
    if (!HasLastSnapshot(
        axisIndex))
    {
        return false;
    }


    outSnapshot =
        m_lastSnapshots[
            axisIndex];


    return true;
}


const std::string&
HomePersistenceManager::GetSnapshotPath() const
{
    return
        m_snapshotPath;
}


const std::string&
HomePersistenceManager::GetHistoryPath() const
{
    return
        m_historyPath;
}


bool HomePersistenceManager::HasPendingWrite() const
{
    return
        m_snapshotDirty ||
        !m_pendingHistory.empty();
}


// ============================================================
// Helpers
// ============================================================

std::string
HomePersistenceManager::NormalizeDirectory(
    const std::string& directory)
{
    if (directory.empty())
    {
        return
            "D:\\EtherCAT_Master_Data\\Data\\";
    }


    std::string result =
        directory;


    const char last =
        result[
            result.size() -
                1];


    if (last != '\\' &&
        last != '/')
    {
        result +=
            "\\";
    }


    return
        result;
}


bool HomePersistenceManager::FileExists(
    const std::string& path)
{
    HANDLE handle =
        CreateFileA(
            path.c_str(),
            GENERIC_READ,
            FILE_SHARE_READ |
            FILE_SHARE_WRITE,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr);


    if (handle ==
        INVALID_HANDLE_VALUE)
    {
        return false;
    }


    CloseHandle(
        handle);


    return true;
}


bool HomePersistenceManager::WriteAllText(
    const std::string& path,
    const std::string& text)
{
    HANDLE handle =
        CreateFileA(
            path.c_str(),
            GENERIC_WRITE,
            FILE_SHARE_READ,
            nullptr,
            CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL |
            FILE_FLAG_WRITE_THROUGH,
            nullptr);


    if (handle ==
        INVALID_HANDLE_VALUE)
    {
        return false;
    }


    const bool success =
        WriteBufferRtss(
            handle,
            text.data(),
            text.size());


    CloseHandle(
        handle);


    return success;
}


std::string HomePersistenceManager::Trim(
    const std::string& value)
{
    const size_t begin =
        value.find_first_not_of(
            " \t\r\n");


    if (begin ==
        std::string::npos)
    {
        return
            std::string();
    }


    const size_t end =
        value.find_last_not_of(
            " \t\r\n");


    return
        value.substr(
            begin,
            end -
            begin +
            1);
}


std::string
HomePersistenceManager::GetUtcTimestamp()
{
    SYSTEMTIME st{};

    GetSystemTime(
        &st);


    char buffer[64]{};


    sprintf_s(
        buffer,
        sizeof(buffer),
        "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
        static_cast<unsigned int>(
            st.wYear),
        static_cast<unsigned int>(
            st.wMonth),
        static_cast<unsigned int>(
            st.wDay),
        static_cast<unsigned int>(
            st.wHour),
        static_cast<unsigned int>(
            st.wMinute),
        static_cast<unsigned int>(
            st.wSecond),
        static_cast<unsigned int>(
            st.wMilliseconds));


    return
        buffer;
}


std::string
HomePersistenceManager::ToHex64(
    uint64_t value)
{
    std::ostringstream out;

    out <<
        "0x" <<
        std::uppercase <<
        std::hex <<
        std::setw(16) <<
        std::setfill('0') <<
        value;


    return
        out.str();
}


bool HomePersistenceManager::ParseBool(
    const std::string& value,
    bool defaultValue)
{
    const std::string text =
        Trim(value);


    if (text == "1" ||
        text == "true" ||
        text == "TRUE")
    {
        return true;
    }


    if (text == "0" ||
        text == "false" ||
        text == "FALSE")
    {
        return false;
    }


    return
        defaultValue;
}


int HomePersistenceManager::ParseInt(
    const std::string& value,
    int defaultValue)
{
    try
    {
        return
            std::stoi(
                Trim(value),
                nullptr,
                0);
    }
    catch (...)
    {
        return
            defaultValue;
    }
}


int32_t HomePersistenceManager::ParseInt32(
    const std::string& value,
    int32_t defaultValue)
{
    try
    {
        return
            static_cast<int32_t>(
                std::stoll(
                    Trim(value),
                    nullptr,
                    0));
    }
    catch (...)
    {
        return
            defaultValue;
    }
}


uint64_t HomePersistenceManager::ParseUInt64(
    const std::string& value,
    uint64_t defaultValue)
{
    try
    {
        return
            static_cast<uint64_t>(
                std::stoull(
                    Trim(value),
                    nullptr,
                    0));
    }
    catch (...)
    {
        return
            defaultValue;
    }
}


double HomePersistenceManager::ParseDouble(
    const std::string& value,
    double defaultValue)
{
    try
    {
        return
            std::stod(
                Trim(value));
    }
    catch (...)
    {
        return
            defaultValue;
    }
}
