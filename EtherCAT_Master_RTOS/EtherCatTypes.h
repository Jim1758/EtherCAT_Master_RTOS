#pragma once
#include <stdint.h>
#include <vector>
#include <string>

// ============================================================================
//  1. EtherCAT 基礎定義 (命令與標頭)
// ============================================================================
#define EC_CMD_APRD 0x01
#define EC_CMD_APWR 0x02
#define EC_CMD_FPRD 0x04
#define EC_CMD_FPWR 0x05
#define EC_CMD_BRD  0x07
#define EC_CMD_BWR  0x08
#define EC_CMD_LRD  0x0A
#define EC_CMD_LWR  0x0B
#define EC_CMD_LRW  0x0C

#define REG_STATION_ADDR 0x0010
#define REG_AL_CONTROL   0x0120
#define REG_AL_STATUS    0x0130

#pragma pack(push, 1)
typedef struct
{
    uint16_t Length;
    uint16_t Type;
} EtherCatHeader;

typedef struct
{
    uint8_t  Cmd;
    uint8_t  Index;
    uint16_t Address;
    uint16_t Register;
    uint16_t Length;
    uint16_t Irq;
} EtherCatDatagramHeader;
#pragma pack(pop)

// ============================================================================
//  2. CiA402 伺服驅動器資料結構 (對應 RxPDO 和 TxPDO)
//  [新增] 這是為了 Delta A3-E 定義的硬體映射結構
// ============================================================================
#pragma pack(push, 1)

// 對應 RxPDO (1601h): 總長度 9 Bytes
struct ServoOutput
{

    uint16_t ControlWord;      // [0-1] 控制字 (6040h) - 16 bit
    int32_t  TargetVelocity;   // [2-5] 目標速度 (60FFh) - 32 bit
    uint16_t TouchProbeFunc;   // [6-7] 探針功能 (60B8h) - 16 bit
    int8_t   ModesOfOperation; // [8]   操作模式 (6060h) - 8 bit (1 Byte)

};

// 對應 TxPDO (1A01h): 總長度 23 Bytes
struct ServoInput
{

    uint16_t StatusWord;              // [0-1]   狀態字 (6041h) - 16 bit
    int32_t  ActualPosition;          // [2-5]   實際位置 (6064h) - 32 bit
    int32_t  ActualVelocity;          // [6-9]   實際速度 (606Ch) - 32 bit
    int16_t  ActualTorque;            // [10-11] 實際扭力 (6077h) - 16 bit
    uint16_t TouchProbeStatus;        // [12-13] 探針狀態 (60B9h) - 16 bit
    int32_t  TouchProbePos1;          // [14-17] 探針抓取位置 (60BAh) - 32 bit
    int8_t   ModesOfOperationDisplay; // [18]    目前模式顯示 (6061h) - 8 bit (1 Byte)
    uint32_t Object_2510;             // [19-22] 廠商自定義參數 (2510h) - 32 bit


};
#pragma pack(pop)


// ============================================================================
//  3. 模組物件定義 (伺服、IO、類比)
// ============================================================================

// 伺服物件
struct ENI_ServoDrive
{
    int slaveIndex;
    uint32_t vendorId;
    uint32_t productCode;

    ServoOutput* pOutput = nullptr;
    ServoInput* pInput = nullptr;
};

// [保留] 數位模組 (Generic IO)
struct ENI_GenericIO
{
    int slaveIndex;
    uint32_t vendorId;
    uint32_t productCode;
    uint16_t inputAddr;
    std::vector<uint8_t> inBuffer;
    uint16_t outputAddr;
    std::vector<uint8_t> outBuffer;

    // PDO 資料指標
    void* pOutputLoc = nullptr; // 指向 Output (燈號)
    void* pInputLoc = nullptr;  // 指向 Input (按鈕)
};

// [保留] 類比模組 (Analog / Special)
struct ENI_AnalogModule
{
    int slaveIndex;
    uint32_t vendorId;
    uint32_t productCode;
    int channelCount;
    uint16_t configAddr;

    // 解析後的數值
    std::vector<int16_t> channelValues;

    // PDO 資料指標
    void* pInputLoc = nullptr;
};

// ============================================================================
//  4. ENI XML 解析結構
// ============================================================================

// [保留] 結構名稱必須是 EtherCatSlave
struct EtherCatSlave
{
    char name[128];
    char type[128];

    uint32_t vendorId;
    uint32_t productCode;

    uint16_t configAddrIn;
    uint16_t configAddrOut;
    uint32_t bitSize;

    // Input 資訊
    uint16_t config_addr_in;   // 輸入位址 (PhysAddr)
    uint32_t inputBitLength;   // 輸入長度 (BitSize)

    // Output 資訊
    uint16_t config_addr_out;  // 輸出位址 (PhysAddr)
    uint32_t outputBitLength;  // 輸出長度 (BitSize)

    struct InitCmd
    {
        uint16_t index;
        std::vector<uint8_t> data;
    };
    std::vector<InitCmd> initCmds;
};