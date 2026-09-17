#pragma once
#include "NC_Types.h"
#include "NCTranslationSnapshot.h"
#include <vector>
#include <string>

class NCManager;
struct AxisContext;
class CoordinateManager {
public:
    CoordinateManager();

    // One NC-thread writer owns the run descriptor. The RT position update
    // writes actualMCS only; it never reads or mutates this descriptor.
    bool BeginTranslationRun(std::uint64_t runToken) noexcept;
    NCTranslationSnapshot GetTranslationSnapshot() const noexcept;
    bool FreezeTranslationRun() noexcept;
    // NC-only staged distance change; frame geometry and native tail stay fixed.
    bool PrepareDistanceModeTransition(int mode, NCTranslationSnapshot& candidate) const noexcept;
    bool CommitDistanceModeTransition(const NCTranslationSnapshot& candidate) noexcept;
    // NC-only drained unit selection; stored native mm/degrees never change.
    bool PrepareUnitModeTransition(int mode, NCTranslationSnapshot& candidate) const noexcept;
    bool CommitUnitModeTransition(const NCTranslationSnapshot& candidate) noexcept;
    // Fully drained XYZ scale/mirror selection; unchanged requests are idempotent.
    bool PrepareScaleMirrorTransition(int code, const double* values, const bool* hasAxis,
        double factor, NCTranslationSnapshot& candidate) const noexcept;
    bool CommitScaleMirrorTransition(const NCTranslationSnapshot& candidate) noexcept;
    bool IsScaleMirrorActive() const noexcept;
    // NC-only drained WCS selection; fixed transforms and native tail stay unchanged.
    bool PrepareWorkCoordinateTransition(int wcsCode, NCTranslationSnapshot& candidate) const noexcept;
    bool CommitWorkCoordinateTransition(const NCTranslationSnapshot& candidate) noexcept;
    // NC-only drained H selection; native geometry and fixed centres remain exact.
    bool PrepareToolLengthTransition(int normalizedMode, int normalizedH,
        NCTranslationSnapshot& candidate) const noexcept;
    bool CommitToolLengthTransition(const NCTranslationSnapshot& candidate) noexcept;
    // NC-only drained G68/G69 selection; MCS and all offset tables stay fixed.
    bool PreparePlanarRotationTransition(int mode, double centerX, double centerY,
        double angle, NCTranslationSnapshot& candidate) const noexcept;
    bool CommitPlanarRotationTransition(const NCTranslationSnapshot& candidate) noexcept;
    // Drained WORK selection. Explicit XY belongs to the complete prior frame.
    // A valid identical selection returns the unchanged descriptor.
    bool PrepareWorkpieceTransition(int mode, int wCode, bool hasCenter,
        double centerX, double centerY, NCTranslationSnapshot& candidate) const noexcept;
    bool CommitWorkpieceTransition(int mode, int wCode, bool hasCenter,
        double centerX, double centerY, const NCTranslationSnapshot& candidate) noexcept;
    bool IsTranslationRunCurrent() const noexcept;
    bool IsTranslationRunFrozen() const noexcept;
    bool IsTranslationRunBound() const noexcept;
    void RetireTranslationRun() noexcept;
    bool GuardCoordinateMutation(const char* operation, NCManager* nc,
        bool reportAlarm = true);
    void BeginTranslationReset() noexcept;
    void EndTranslationReset() noexcept;

    // Validate every selected field before changing any table value. Work
    // fields remain the existing schema; this does not enable new transforms.
    bool ApplyCoordinateTableValues(int offsetType, int row,
        const bool* hasField, const double* values, NCManager* nc,
        bool reportAlarm = true);
    // G160 uses fixed table fields XYZ / IJK, never configured axis letters.
    bool TryDecodeWorkTableWrite(const NCBlock& block, int& rowIndex,
        bool* fields, double* values) const noexcept;
    bool ApplyCoordinateOrigin(int axis, double desiredWCS, NCManager* nc,
        bool reportAlarm = false);
    bool SetCAxisOffsetRotationEnabled(bool enabled, NCManager* nc);

    // ==========================================
    // 狀態紀錄
    // ==========================================
    bool isAbsoluteMode = true;
    int currentWCSIndex = 0;
    double commandedMCS[8] = { 0.0 };

    // ==========================================
    // 🌟 定義 C 軸的索引位置 (通常第四軸為 index 3)
    // ==========================================
    const int C_AXIS_INDEX = 3;
    // ==========================================
    // 🌟 C 軸電極偏心旋轉補償開關 (預設開啟)
    // ==========================================
    bool isCAxisOffsetRotationEnabled = true; // 預设 true (開啟)
    // ==========================================
    // 🌟 平面選擇狀態 (群組 2)
    // ==========================================
    int activePlane = 17; // 預設 G17 (XY 平面)

    // 設定平面的 API
    void SetActivePlane(int gCode, NCManager* nc);

    // ==========================================
    // 🌟 即時座標資料區
    // ==========================================
    // 儲存 8 軸的真實機械位置 (未來由 EtherCAT 的 RxPDO/TxPDO 更新)
    double actualMCS[8] = { 0.0 };    // 🌟 新增：真實機械位置

    // 🌟 新增：刀長補正狀態 (G43/G44/G49, 群組 8)
    int toolLengthMode = 49; // 預設 G49 關閉
    int currentHCode = 0;    // 預設 H 碼為 0

    // ==========================================
    // 🌟 刀具與工件補正號碼狀態紀錄
    // ==========================================
    int currentTCode = 0;    // 🌟 新增：當前主軸上的刀具號碼 (T 碼，預設 0)

    // 🌟 新增：設定刀號 API
    void SetToolNumber(int tCode, NCManager* nc = nullptr);

    //目前工件號碼
    int currentWorkpieceNum = 0; // 🌟 新增：獨立的工件編號 (非 G168，生產管理用)

    void SetWorkpieceNumber(int num, NCManager* nc = nullptr); // 🌟 新增：設定獨立工件號 API




    // ==========================================
    // 🌟 定義 m_WorkOffset 陣列的 8 個欄位名稱
    // ==========================================
    enum WorkOffsetField {
        WO_OFFSET_X = 0,       // 平移 X
        WO_OFFSET_Y = 1,       // 平移 Y
        WO_OFFSET_Z = 2,       // 平移 Z
        WO_ANGLE_XY_YAW = 3,   // XY 平面旋轉角度 (繞 Z 軸旋轉 / Yaw)
        WO_ANGLE_XZ_PITCH = 4, // XZ 平面旋轉角度 (繞 Y 軸旋轉 / Pitch)
        WO_ANGLE_YZ_ROLL = 5,  // YZ 平面旋轉角度 (繞 X 軸旋轉 / Roll)
        WO_RESERVED_6 = 6,     // 保留備用
        WO_RESERVED_7 = 7      // 保留備用
    };

    // ==========================================
    // 🌟 G168 工件旋轉狀態紀錄
    // ==========================================
    bool isWorkpieceRotationActive = false;
    int currentWCode = 0;                          // 當前使用的 W 碼
    double rotationCenterMCS[3] = { 0.0, 0.0, 0.0 }; // 真實旋轉圓心 (機械座標)




    // ==========================================
    // 🌟 G68 座標旋轉狀態 (群組 16)
    // ==========================================
    bool isG68Active = false;
    double g68CenterWCS[3] = { 0.0, 0.0, 0.0 }; // 旋轉圓心 (WCS 邏輯座標)
    double g68Angle = 0.0;                      // 旋轉角度 (度)


    // 控制 API
    void SetG68Rotation(const double* centerPos, const bool* hasAxis, double angle, NCManager* nc);
    void CancelG68Rotation(NCManager* nc);

    // ==========================================
    // 🌟 G50 / G51 縮放比例 (群組 11)
    // ==========================================
    bool isScalingActive = false;
    double scalingCenterWCS[8] = { 0.0 }; // 縮放中心
    double scaleFactor = 1.0;             // 縮放倍率 (P)

    // ==========================================
    // 🌟 G150 / G151 鏡像 (通常歸類在群組 22 或獨立)
    // ==========================================
    bool isMirrorActive[8] = { false };     // 紀錄哪幾個軸開啟了鏡像
    double mirrorCenterWCS[8] = { 0.0 };    // 鏡像對稱中心

    bool IsMirrorActive(int axisIndex) const {
        if (axisIndex >= 0 && axisIndex < 8) return isMirrorActive[axisIndex];
        return false;
    }
    // ==========================================
    // 🌟 G15 / G16 極座標 (群組 17)
    // ==========================================
    bool isPolarCoordinateActive = false;

    void SetPolarCoordinate(NCManager* nc);
    void CancelPolarCoordinate(NCManager* nc);

    // ==========================================
    // 🌟 G40 / G41 / G42 刀徑補償 (群組 7)
    // ==========================================
    int toolRadiusMode = 40; // 預設 G40 (取消補償)
    int currentDCode = 0;    // 當前使用的 D 碼

    void SetToolRadiusCompensation(int gCode, int dCode, NCManager* nc);
    void CancelToolRadiusCompensation(NCManager* nc);
    double GetActiveToolRadius() const; // 取得當前刀具半徑

    // API
    void SetScaling(const double* centerPos, const bool* hasAxis, double factor, NCManager* nc);
    void CancelScaling(NCManager* nc);
    void SetMirror(const double* mirrorPos, const bool* hasAxis, NCManager* nc);
    void CancelMirror(const bool* hasAxis, NCManager* nc);

    // ==========================================
    // 🌟 新增：座標計算 API
    // ==========================================
    // 1. 把外部 (EtherCAT) 讀到的新座標更新進來
    void UpdateActualMCS(const double* newMCS);

    // 2. 獲取當前真實的工件座標 (WCS)
    void GetActualWCS(double* outWCS) const;



    // ==========================================
    // 參數表格 (支援 8 軸)
    // ==========================================
    double extOffset[8] = { 0.0 };
    std::vector<std::vector<double>> m_WCSTable;
    std::vector<std::vector<double>> m_ToolOffset;
    std::vector<std::vector<double>> m_ToolRadius;
    std::vector<std::vector<double>> m_WorkOffset;
    // 儲存 100 組參考點 (P1~P100)，每組 8 軸
    std::vector<std::vector<double>> m_RefPoints;


    // ==========================================
    // 核心操作函式
    // ==========================================
    bool SetWCS(int gCode, NCManager* nc, bool reportAlarm = true);
    void SyncMachinePosition(const double* actualMCS);
    void Transform_WCS_to_MCS(const double* targetWCS, const bool* hasAxis, double* outputMCS);

    // G90 sparse rotation uses the frozen inverse; G91 expands omitted XY
    // displacement to zero and validates against the accepted native tail.
    // This changes only caller-owned staging arrays, never commandedMCS.
    bool CompleteFixedPlanarEndpoint(double* targetWCS, bool* hasAxis) noexcept;

    // Stage NC-0.2K.6.2: calculate the complete G00 MCS candidate without
    // publishing commandedMCS.  The normal G00 producer commits that endpoint
    // together with lastQueuedPulse only after Motion ingress accepts the
    // matching command.
    void Preview_WCS_to_MCS(
        const double* targetWCS,
        const bool* hasAxis,
        double* outputMCS);


    // 🌟 新增：刀長補正控制 API
    bool IsToolLengthSelectionSupported(int normalizedMode, int normalizedH) const noexcept;
    void SetToolLengthCompensation(int gCode, int hCode, NCManager* nc);
    void CancelToolLengthCompensation(NCManager* nc); // 供 Reset 時呼叫
    double GetActiveToolOffset(int axisIndex, double cAngleMCS = 0.0) const;
    // 🌟 新增：取得目前作用中的工件平移偏移量 (G168 W 碼)
    bool IsWorkpieceSelectionSupported(int normalizedMode, int normalizedW) const noexcept;
    bool IsFixedPlanarRotationActive() const noexcept;
    double GetActiveWorkOffset(int axisIndex) const;
    // ==========================================
    // 🌟 新增：多檔案管理系統
    // ==========================================

    void LoadAllParameters();
    void SaveAllParameters();

    // 針對單一表格的讀寫 (如果你 HMI 只改了刀具，可以單獨呼叫 SaveToolOffset)
    void SaveWCSStatus();   // 🌟 改名：讓函式名稱更明確
    void SaveExtOffset();
    void SaveToolOffset();
    void SaveToolRadius();
    void SaveRefPoints();
    void SaveWorkOffset();

    void SaveWCSTable();
    int GetCurrentWCSGCode() const;


    // 🌟 新增：G92 相關 API
    void ApplyG92(const bool* axisProgrammed, const double* targetPos, NCManager* nc);

    void GetActualMCS(double* outMCS) const; // 取出當前的真實機械座標
     // 🌟 新增：獲取當前純粹的數學命令座標 (Commanded WCS) - 供 G92 等內部數學計算用
    void GetCommandedWCS(double* outWCS) const;

    void Set_G90G91(int value, NCManager* nc);//設定90絕對模式 91增量模式

    // ==========================================
    // 🌟 旋轉控制 API
    // ==========================================
    void SetWorkpieceRotation(int wCode, const bool* hasAxis, const double* targetWCS, NCManager* nc);
    void CancelWorkpieceRotation(NCManager* nc);


    bool GetRefPoint(int pCode, double* outPos) const;

    // 🌟 新增：取得機台當下的剩餘移動量 (Distance To Go)
    void GetDistanceToGo(double* outDTG, NCManager* nc) const;


    // ==========================================
// 🌟 公英制狀態紀錄 (G20 / G21, 群組 6)
// ==========================================
    bool isInchMode = false; // 預設為 false (G21 公制 mm)

    // 設定公英制 API
    void SetUnitMode(int gCode, NCManager* nc = nullptr);

    // ==========================================
    // 🌟 海關翻譯閘門 API (最核心的轉換工具)
    // ==========================================
    // 出去給別人看時呼叫 (底層 mm -> 畫面/巨集)
    double ToDisplayUnit(double internalMmValue, bool isRotaryAxis) const;

    // 從外面收進來時呼叫 (畫面/G碼輸入 -> 轉為底層 mm)
    double ToInternalUnit(double externalValue, bool isRotaryAxis) const;




    //G22 G23  軟體極限 及系統軟體極限-----------------------------------------------------

// ==========================================================
// Software Travel Limit / Stored Stroke Check
//
// 軟體行程保護歸 CoordinateManager 管理。
//
// 判斷基準：Machine Coordinate System (MCS)
//
// ----------------------------------------------------------
// Travel Limit 1
//
// Parameter Enable
// +
// G22 / G23
//
// G22 = Enable
// G23 = Disable
//
// ----------------------------------------------------------
// Travel Limit 2
//
// Parameter Enable Only
// 不受 G22 / G23 影響。
//
// ----------------------------------------------------------
// Travel Limit 3
//
// Parameter Enable Only
// 不受 G22 / G23 影響。
//
// ----------------------------------------------------------
// Physical +OT / -OT 不屬於這裡。
// Physical Limit 由 NCPLCManager / Motion Layer 處理。
// ==========================================================


// ----------------------------------------------------------
// G22 / G23
//
// Group 04 Stored Stroke Check Mode
//
// gCode:
//      22 = Enable
//      23 = Disable
//
// nc:
//      預留給後續 Alarm / System Variable / 狀態同步。
// ----------------------------------------------------------
    void SetStoredStrokeCheckMode(int gCode, NCManager* nc = nullptr);


    // ----------------------------------------------------------
    // 直接設定 Runtime State
    //
    // 主要提供：
    //      Power-On Parameter
    //      Reset 初始化
    //
    // 使用。
    //
    // false = G23 狀態
    // true  = G22 狀態
    // ----------------------------------------------------------
    void SetProgrammableTravelLimitEnabled(bool enabled, NCManager* nc = nullptr);


    // ----------------------------------------------------------
    // 取得目前 G22 / G23 狀態
    // ----------------------------------------------------------
    bool IsProgrammableTravelLimitEnabled() const;


    // ----------------------------------------------------------
    // 更新單軸 Software Travel Limit 狀態
    //
    // 根據：
    //      axis.currentActPos / Machine Position
    //      Travel Limit 1 Parameter
    //      Travel Limit 2 Parameter
    //      Travel Limit 3 Parameter
    //      G22 / G23 State
    //
    // 更新：
    //
    // travelLimit1PositiveActive
    // travelLimit1NegativeActive
    //
    // travelLimit2PositiveActive
    // travelLimit2NegativeActive
    //
    // travelLimit3PositiveActive
    // travelLimit3NegativeActive
    //
    // 注意：
    // 不處理 Physical +OT / -OT。
    // 不產生 Alarm。
    // 不直接停止 Motion。
    // ----------------------------------------------------------
    void UpdateSoftwareTravelLimitState(AxisContext& axis) const;


    // ----------------------------------------------------------
    // Automatic Target Pre-Check
    //
    // 給未來：
    //      G00
    //      G01
    //      MDI
    //      MEMORY
    //
    // 在運動真正開始以前先判斷 Target 是否合法。
    //
    // targetMCS:
    //
    //      Machine Coordinate
    //      Linear Axis = mm
    //      Rotary Axis = degree
    //
    // true  = Target 合法
    // false = Target 超出啟用中的 Software Travel Limit
    // ----------------------------------------------------------
    bool IsTargetWithinSoftwareTravelLimit(const AxisContext& axis, double targetMCS) const;


    // ----------------------------------------------------------
    // Manual Direction Permission
    //
    // 只判斷 Software Travel Limit。
    //
    // Physical +OT / -OT 之後再與這個結果做 OR。
    //
    // 例如：
    //
    // 已到達 Software +Limit
    //
    //      CanMoveSoftwarePositive() = false
    //      CanMoveSoftwareNegative() = true
    //
    // 所以 JOG 可以反方向離開。
    // ----------------------------------------------------------
    bool CanMoveSoftwarePositive(const AxisContext& axis) const;
    bool CanMoveSoftwareNegative(const AxisContext& axis) const;

    // ==========================================================
 // Manual Frame
 //
 // 只影響手動運動：
 //
 //   Normal JOG
 //   Fine JOG
 //   INCH JOG
 //   MPG
 //
 // 不影響：
 //
 //   NC Program
 //   G00 / G01 / G02 / G03
 //   HOME
 //   G68
 //   G168
 //   Mirror
 //
 // Rotation:
 //
 //   R = Rz(Yaw) * Ry(Pitch) * Rx(Roll)
 //
 // Angle unit:
 //   Degree
 //
 // Default:
 //   Disabled
 //   Yaw   = 0.0
 //   Pitch = 0.0
 //   Roll  = 0.0
 // ==========================================================

 // 開關
    void SetManualFrameEnabled(bool enabled);

    bool IsManualFrameEnabled() const;


    // ----------------------------------------------------------
    // 一次設定三個角度
    //
    // HMI 正式建議使用這個 API。
    // ----------------------------------------------------------

    void SetManualFrameAngles(double yawDeg, double pitchDeg, double rollDeg);


    // ----------------------------------------------------------
    // 各別設定
    // ----------------------------------------------------------

    void SetManualFrameYaw(double yawDeg);
    void SetManualFramePitch(double pitchDeg);
    void SetManualFrameRoll(double rollDeg);


    // ----------------------------------------------------------
    // 讀取
    // ----------------------------------------------------------

    double GetManualFrameYaw() const;

    double GetManualFramePitch() const;

    double GetManualFrameRoll() const;


    // ----------------------------------------------------------
    // Manual Vector -> Machine Vector
    //
    // 內部直接使用 CoordinateManager 保存的
    // Yaw / Pitch / Roll。
    //
    // NCPLCManager 不需要知道角度來源。
    // ----------------------------------------------------------

    void TransformManualVector(const double* manualVector, double* machineVector) const;


    // ==========================================================
// Software Travel Limit Modal State
// ==========================================================
//
// false = G23
//         Programmable Travel Limit 1 OFF
//
// true  = G22
//         Programmable Travel Limit 1 ON
//
// Power-On 預設先使用 G23。
// 後續再接 Parameter 決定開機是否直接進入 G22。
// ==========================================================

    bool m_programmableTravelLimitEnabled = false;
private:
    bool m_workRotationCenterFixed = false; // Only explicit fixed G168 XY selection grants this proof.
    // One-use confirmation for the ordinary G168 handler after staged publication.
    // It binds the original words, avoiding reinterpretation in the new frame.
    struct WorkCenterConfirmation
    {
        bool armed = false;
        std::uint64_t runToken = 0ULL;
        std::uint64_t generation = 0ULL;
        std::uint64_t revision = 0ULL;
        int wCode = 0;
        double inputXY[2] = {};
    };
    WorkCenterConfirmation m_workCenterConfirmation{};
    NCTranslationSnapshot BuildCurrentCoordinateSnapshot() const noexcept;
    NCTranslationSnapshot BuildLiveTranslationSnapshot() const noexcept;
    bool IsTranslationModeSupported() const noexcept;
    bool BuildScaleMirrorSelection(int code, const double* values, const bool* hasAxis,
        double factor, const NCTranslationSnapshot& source, NCTranslationSnapshot& candidate) const noexcept;
    bool IsToolOffsetRowValid(int hCode, bool xyzOnly) const noexcept;
    bool IsWorkOffsetRowValid(int wCode, bool fixedPlanar) const noexcept;
    bool RejectCoordinateMutation(const char* operation, const char* reason,
        NCManager* nc, bool reportAlarm) const;
    std::uint64_t m_translationGenerationCounter = 0ULL;
    std::uint64_t m_translationRunToken = 0ULL;
    std::uint64_t m_translationGeneration = 0ULL;
    std::uint64_t m_translationRevision = 1ULL;
    bool m_translationFrozen = false;
    bool m_translationResetBypass = false;
    NCTranslationSnapshot m_frozenTranslation{};

    void Transform_WCS_to_MCS_Internal(
        const double* targetWCS,
        const bool* hasAxis,
        double* outputMCS,
        bool commitCommandedMCS);

    // 底層輔助函式：負責讀寫 8 軸二維陣列，並確保原子寫入防護
    void SaveTableToFile(const std::string& filename, const std::vector<std::vector<double>>& table);
    void LoadTableFromFile(const std::string& filename, std::vector<std::vector<double>>& table, int maxRows);

    // ==========================================================
// Manual Frame State
// ==========================================================

    bool m_manualFrameEnabled = false;
    double m_manualFrameYawDeg = 0.0;
    double m_manualFramePitchDeg = 0.0;
    double m_manualFrameRollDeg = 0.0;


};
