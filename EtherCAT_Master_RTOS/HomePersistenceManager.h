#pragma once

#ifndef EDM_HOME_PERSISTENCE_MANAGER_H
#define EDM_HOME_PERSISTENCE_MANAGER_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct AxisContext;
class NCManager;


// ============================================================
// HOME Snapshot
//
// 這是「上一次成功 HOME 的歷史資料」。
//
// 非常重要：
//     Snapshot 存在 != 本次開機已 Homed。
//
// 每次 Core 啟動仍然強制：
//     axis.isHomed = false
//
// 必須重新執行 G81 成功後，
// 本次執行期間的 axis.isHomed 才能重新變成 true。
// ============================================================

struct HomeSnapshotRecord
{
    static constexpr int CURRENT_VERSION = 1;

    int version = CURRENT_VERSION;

    bool hasSnapshot = false;

    uint64_t sequenceNumber = 0;

    std::string completedAtUtc;

    int axisIndex = -1;

    // AXIS_CFG.ini 在本次 HOME 完成時對應的軸代號。
    char axisName = '?';

    int homeMethod = 0;
    int referenceSource = 0;
    int captureMode = 0;
    int homeDirection = -1;
    int homeOrder = 0;

    // 台達 0x60BA 原始 Capture Position。
    int32_t capturedReferenceRaw = 0;

    // 已 unwrap / source conversion 後的 Pulse Domain。
    double capturedReferencePulse = 0.0;

    // DOG / Limit 第一階段觸發位置。
    double switchDetectedPulse = 0.0;

    // Reference - Switch 的相對距離。
    // INDEX_ONLY / CURRENT_POSITION 等沒有第一階段 Switch 的模式為 false。
    bool hasSwitchToReferenceDistance = false;

    double switchToReferencePulse = 0.0;
    double switchToReferenceUnit = 0.0;

    // Machine Coordinate 建立後的 Offset。
    double machineCoordinateOffsetPulse = 0.0;

    double homeOffsetUnit = 0.0;

    // HOME 全流程真正完成時的位置。
    double finalRawPulse = 0.0;
    double finalMachinePositionUnit = 0.0;

    double resolutionPPR = 0.0;
    double finalLead = 0.0;

    bool moveToZeroEnabled = false;
    bool moveToZeroCompleted = false;

    // HOME 設定 Fingerprint。
    // 用來判斷本次與上一筆是否仍是同一套 HOME 參數。
    uint64_t configFingerprint = 0;

    // 與上一筆 Snapshot 的比較結果。
    bool hasPreviousSnapshot = false;
    bool configMatchedPrevious = false;
    bool hasRepeatabilityComparison = false;

    // 本次 Switch->Reference - 上次 Switch->Reference。
    double repeatabilityDeltaUnit = 0.0;
};


// ============================================================
// HOME Persistence Manager
//
// 檔案位置：
//
//   <NCDataDir>\HomeSnapshot.txt
//   <NCDataDir>\HomeHistory.csv
//
// 設計原則：
//
// 1. HomingManager 完成時只 Queue 記憶體資料，不直接寫磁碟。
// 2. 由 1000ms supervisory task 呼叫 FlushPending()。
// 3. Snapshot 採 temp + replace。
// 4. History 只追加成功 HOME。
// 5. 啟動載入 Snapshot 只供診斷；絕不恢復 isHomed。
// ============================================================

class HomePersistenceManager
{
public:
    static HomePersistenceManager& GetInstance();

    void Initialize(
        const std::string& ncDataDir,
        std::vector<AxisContext>& axes,
        const NCManager* ncManager);

    // HOME 成功完成時呼叫。
    // 只建立記憶體 Snapshot / Pending History，不進行檔案 I/O。
    void QueueSuccessfulHome(
        int axisIndex,
        const AxisContext& axis);

    // 由低頻 supervisory task 呼叫。
    bool FlushPending();

    bool HasLastSnapshot(
        int axisIndex) const;

    bool TryGetLastSnapshot(
        int axisIndex,
        HomeSnapshotRecord& outSnapshot) const;

    const std::string& GetSnapshotPath() const;
    const std::string& GetHistoryPath() const;

    bool HasPendingWrite() const;

private:
    HomePersistenceManager() = default;

    HomePersistenceManager(
        const HomePersistenceManager&) = delete;

    HomePersistenceManager& operator=(
        const HomePersistenceManager&) = delete;


    bool LoadSnapshotFile();

    HomeSnapshotRecord BuildSnapshot(
        int axisIndex,
        const AxisContext& axis);

    uint64_t ComputeHomeConfigFingerprint(
        int axisIndex,
        const AxisContext& axis) const;

    bool WriteSnapshotFileAtomic();

    bool AppendPendingHistory();

    std::string BuildSnapshotText() const;

    std::string BuildHistoryHeader() const;

    std::string BuildHistoryRow(
        const HomeSnapshotRecord& record) const;

    static std::string NormalizeDirectory(
        const std::string& directory);

    static bool FileExists(
        const std::string& path);

    static bool WriteAllText(
        const std::string& path,
        const std::string& text);

    static std::string Trim(
        const std::string& value);

    static std::string GetUtcTimestamp();

    static std::string ToHex64(
        uint64_t value);

    static bool ParseBool(
        const std::string& value,
        bool defaultValue);

    static int ParseInt(
        const std::string& value,
        int defaultValue);

    static int32_t ParseInt32(
        const std::string& value,
        int32_t defaultValue);

    static uint64_t ParseUInt64(
        const std::string& value,
        uint64_t defaultValue);

    static double ParseDouble(
        const std::string& value,
        double defaultValue);


private:
    static constexpr int HOME_AXIS_COUNT = 8;

    bool m_initialized = false;

    std::string m_dataDir;
    std::string m_snapshotPath;
    std::string m_snapshotBackupPath;
    std::string m_historyPath;

    // 唯一來源：NCManager 已由 AXIS_CFG.ini 載入的動態軸代號。
    std::array<char, HOME_AXIS_COUNT>
        m_axisNames
    {
        'X', 'Y', 'Z', 'A',
        'B', 'C', 'U', 'V'
    };

    std::array<HomeSnapshotRecord, HOME_AXIS_COUNT>
        m_lastSnapshots{};

    std::vector<HomeSnapshotRecord>
        m_pendingHistory;

    bool m_snapshotDirty = false;

    uint64_t m_nextSequenceNumber = 1;
};


#endif // EDM_HOME_PERSISTENCE_MANAGER_H
