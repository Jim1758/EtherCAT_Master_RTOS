#pragma once
#include <cstdint>
#include <vector>

// -------------------------------------------------------------------------
// CiA402 伺服驅動器資料結構 (對應 RxPDO 和 TxPDO)
// -------------------------------------------------------------------------

// [Output] 主站 -> 從站 (RxPDO)
// 對應 Index 0x1600 (Delta A3 預設映射)
#pragma pack(push, 1) // 強制 1-byte 對齊，避免記憶體空隙
struct ServoOutput {
    uint16_t ControlWord;        // 0x6040: 控制字 (啟動、停止、清除錯誤)
    int32_t  TargetPosition;     // 0x607A: 目標位置 (PUU)
    uint32_t TouchProbeFunc;     // 0x60B8: 探針功能 (選用)
    uint32_t DigitalOutputs;     // 0x60FE: 數位輸出 (選用)
};

// [Input] 從站 -> 主站 (TxPDO)
// 對應 Index 0x1A00 (Delta A3 預設映射)
struct ServoInput {
    uint16_t StatusWord;         // 0x6041: 狀態字 (Ready, Fault, Moving...)
    int32_t  ActualPosition;     // 0x6064: 實際位置
    int32_t  ActualVelocity;     // 0x606C: 實際速度 (選用)
    uint32_t DigitalInputs;      // 0x60FD: 數位輸入 (原點、極限...)
    int32_t  ActualTorque;       // 0x6077: 實際轉矩 (選用)
    uint16_t ErrorCode;          // 0x603F: 錯誤代碼 (非常重要!)
    uint32_t TouchProbeStatus;   // 0x60B9: 探針狀態
    int32_t  TouchProbePos1;     // 0x60BA: 探針位置
};
#pragma pack(pop)

// -------------------------------------------------------------------------
// 伺服模組物件 (管理單軸)
// -------------------------------------------------------------------------
struct ServoDrive {
    int slaveIndex;         // EtherCAT 從站索引
    uint32_t vendorId;      // 0x1DD (Delta)
    uint32_t productCode;   // A3E 的代碼

    // 指向 IO Map 的指標 (直接操作記憶體)
    ServoOutput* pOutput = nullptr;
    ServoInput* pInput = nullptr;

    // 狀態機控制用
    uint16_t lastControlWord = 0;

    // 建構子
    ServoDrive() : slaveIndex(0), vendorId(0), productCode(0) {}
};