#pragma once
#include "EtherCatTypes.h"
#include "EtherCatEni.h"
#include "NicDriver.h"
#include <vector>
#include "PlcCore.h"       // 引用 PLC 核心
#include "MotionCore.h"       // 引用 MotionCore 核心
#include "NCManager.h"

#pragma pack(push, 1)

// 標準 Mailbox 標頭 (6 Bytes)
struct MailboxHeader {
    uint16_t length;      // 資料長度 (Data Length)
    uint16_t address;     // 來源/目的位址 (Station Address)
    uint8_t  priority : 2;// 優先級 (0: Lowest)
    uint8_t  type : 4;    // 協議類型 (3 = CoE - CANopen over EtherCAT)
    uint8_t  cnt : 3;     // 計數器 (Counter, 1-7)
    uint8_t  reserved : 1;// 保留位
};

// CoE 標頭 (2 Bytes)
struct CoEHeader {
    uint16_t number : 9;  // PDO Number or Reserved
    uint16_t reserved : 3;// 保留
    uint16_t service : 4; // 服務類型 (2 = SDO Request, 3 = SDO Response, 8 = SDO Info)
};

// SDO 標頭 (4 Bytes) - 用於參數讀寫
struct SDOHeader {
    uint8_t  size_ind : 2; // 資料大小指示 (0: Set in size, 1: No size specified)
    uint8_t  exp_od : 1;   // 加速傳輸 (1 = Expedited Transfer, 資料在 Data 欄位中)
    uint8_t  size : 2;     // 資料大小 (僅在 Expedited 模式下有效, 表示未使用的 Bytes 數)
    uint8_t  command : 3;  // 指令 (1 = Download Request/Write, 2 = Upload Request/Read)
    uint16_t index;        // 物件字典索引 (Object Index, e.g., 0x6060)
    uint8_t  subIndex;     // 子索引 (Sub-Index)
    uint32_t data;         // 資料本體 (4 bytes, 用於 Expedited 傳輸)
};

struct EtherCatFrameHeader {
    uint16_t Length : 11;
    uint16_t Reserved : 1;
    uint16_t Type : 4;
};

struct EtherCatPduHeader {
    uint8_t  Command;
    uint8_t  Index;
    uint16_t Address;
    uint16_t Register;
    uint16_t Length : 11;
    uint16_t Reserved : 3;
    uint16_t Next : 1;
    uint16_t Irq : 1;
};


struct SlaveInfo {
    uint16_t configAddr;    // 我們分配給它的地址 (0x1001...)
    uint16_t APRDAPWR_Addr;    // APRDAPWR 使用的adp
    uint16_t mbxOutAddr;    // SM0 Addr
    uint16_t mbxOutLength;  // SM0 Len
    uint16_t mbxInAddr;     // SM1 Addr
    uint16_t mbxInLength;   // SM1 Len
    uint32_t Vendor_ID;
    uint32_t Product_Code;
   
};


// 定義指令類型
enum class EcatCmdType : int
{
    CMD_NONE = 0,
    CMD_SET_STATE,      // 切換狀態 (例如 OP, INIT)
    CMD_SDO_WRITE,      // SDO 寫入
    CMD_SDO_READ        // SDO 讀取
};

// 定義指令狀態
enum class EcatCmdStatus : int
{
    // --- 閒置狀態 ---
    // 代表目前指令槽位是空的，UI 執行緒可以安全地填寫新的指令資料。
    ECAT_STATUS_IDLE = 0,

    // --- 待處理/執行中 ---
    // UI 填寫完資料後將狀態改為 PENDING。
    // RT 執行緒在每個週期的空檔 (如 Tick 2) 會檢查此旗標，若看到 PENDING 則開始執行硬體通訊。
    ECAT_STATUS_PENDING = 1,

    // --- 執行完成 ---
    // RT 執行緒完成通訊並回填 WKC/結果後，會將狀態設為 DONE。
    // UI 執行緒看到 DONE 後會讀取結果，並將狀態重設回 IDLE。
    ECAT_STATUS_DONE = 2,

    // --- 錯誤狀態 ---
    // 代表指令在執行過程中發生異常 (例如：通訊超時、從站拒絕要求、或是 WKC 為 0)。
    ECAT_STATUS_ERROR = -1
};

struct AsyncCommandSlot 
{
    // --- 控制旗標 ---
    volatile int status = (int)EcatCmdStatus::ECAT_STATUS_IDLE; // [關鍵] volatile 防止編譯器優化順序
    int type;            // 指令類型 (對應 EcatCmdType)

    // --- SDO / 命令參數 ---
    uint16_t slaveAddr;  // 目標站號 (0:廣播, 1~N:指定站)
    uint16_t index;      // SDO Index (例如 0x6040, 0x6060)
    uint8_t subIndex;    // SDO SubIndex (例如 0x00)

    uint32_t dataValue;  // 數值本體 (最大支援 4 bytes)
    int dataSize;        // [新增] 資料長度 (1, 2, 或 4 bytes) <--- 建議加上這個

    // --- 輸出結果 ---
    int resultWKC;       // RT 回傳的 WKC (用來判斷成功/失敗)
};



// 全域或類別成員變數，用來存所有從站資訊
SlaveInfo m_slaveInfo[128];
int Motor_Start_Index = -1;
#pragma pack(pop)

// ============================================================================
// EtherCAT 主站類別 (EtherCatMaster)
// 負責底層封包收發、狀態機管理與從站配置
// ============================================================================

// 🌟 加上這兩行「前置宣告 (Forward Declaration)」
// 這樣所有 #include "EtherCatMaster.h" 的 .cpp 檔案，都會知道有這兩個函數存在！
void RTAPI GlobalTimerHandler_PDO(void* nContext);
void RTAPI GlobalTimerHandler_PLC(void* nContext);
class CoordinateManager;
class EtherCatMaster
{
public:
    EtherCatMaster();
    ~EtherCatMaster();

   
    // 🌟 2. 新增綁定 API
    void LinkCoordinateManager(CoordinateManager* pCoord);

    // 🌟 3. 新增指標變數 (預設設為 nullptr 防呆)
    CoordinateManager* pCoordMgr = nullptr;
  

    //EtherCAT  Function
    void AttachNic(CNicDriver* pNic);// 綁定網卡驅動程式
    void AttachEni(EtherCatEni* pEni); // 綁定 ENI 設定檔解析器

    int ecx_BRD(uint16_t ADP, uint16_t ADO, uint16_t length, int timeout);//廣播讀取
    int ecx_BWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout);//廣播寫入
    int ecx_APRD(uint16_t ADP, uint16_t ADO, uint16_t length, void* data, int timeout);//自動增量物理讀取
    int ecx_APWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout);//自動增量物理寫入
    int ecx_LRW(uint32_t LogAddr, uint16_t length, void* data, int timeout);//邏輯讀寫
    int ecx_FPWR(uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout);//指定位址物理寫入
    int ecx_FPRD(uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout);//指定位址物理讀取
    int ecx_SDOwrite(int slave_pos, uint16_t index, uint8_t subindex, int CA, int size, void* data, int timeout);//服務資料物件寫入 (SDO Write / 寫入物件字典)
    int ecx_SDOread(int slave_pos, uint16_t index, uint8_t subindex, int CA, int* size, void* data, int timeout);//服務資料物件讀取 (SDO Read / 讀取物件字典)
    bool SendAndReceiveRegister(uint8_t cmd, uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout);//暫存器底層收發核心

    bool PDO_SendCommandAndWait(EcatCmdType type, uint16_t slave, uint16_t index, uint8_t sub, uint32_t value, int len, int timeoutMs);//非同步指令發送與同步等待 (執行緒安全指令)


    int ScanSlaves();//掃描所有從站
    void InitSlaveMailboxInfo(int slave_idx);//手動設定從站Mailbox
    uint16_t ReadSII_Word16(int slave_idx, uint16_t word_addr);//讀取從站EEPROM 功能
    uint32_t ReadSII_Uint32(int slave_idx, uint16_t word_addr);
    // 流程: PRE-OP -> Config -> SAFE-OP -> OP
    int Initialize_Slaves();//初始化所有從站 INIT>>PRE-OP>>SAFE-OP>>OP
  

  
    void ConfigureSlaveGeneric_INIT(int slaveIdx);//從站配置INIT
    void ConfigureSlaveGeneric_PRE_OP(int slaveIdx);//從站配置PRE_OP
    void ConfigureSlaveGeneric_SAFE_OP(int slaveIdx);//從站配置SAFE_OP
    uint64_t GetCurrentMasterTimeNs();// 取得 RTX64 系統時間 (單位: 奈秒)
    void  Printf_Slaves_State();//印出從站狀態
    void  Printf_AL_Status_Code();//印出從站狀態0x134 code
 
    // IO  Function
    bool Get_I(int moduleIdx, int bitIdx);
    bool Get_O(int moduleIdx, int bitIdx);
    void Set_O(int moduleIdx, int bitIdx, bool val);

    std::vector<EtherCatSlave> m_slaves;
    std::vector<ENI_GenericIO> m_IoList;       // 只存 6002, 7062 (數位)
    std::vector<ENI_AnalogModule> m_AdList;    // 只存 8124 (類比)
    std::vector<ENI_ServoDrive> m_ServoList;

    // System Function
    int BuildIoMap();   // 自動掃描並建立清單
    void Config_Slave_FMMU(int slaveIdx);//設定 FMMU
    int WriteFmmuRegister(int slaveIdx, int fmmuIdx, uint32_t logAddr, uint16_t len, uint16_t physAddr, uint8_t type, int timeout);//寫入單一 FMMU 設定



     //範例區塊-----------------------------------------------------------------------------------
    void RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式
   
    // System par
    CNicDriver* m_pNic;         // 網卡指標
    EtherCatEni* m_pEni;        // ENI 設定指標
    uint8_t m_idx;              // 全域封包索引 (Packet Index)
    uint8_t m_mboxCnt;          // Mailbox 計數器
    uint8_t m_txBuffer[1514];
    uint8_t m_rxBuffer[1514];
    PlcCore    m_Plc;
   
    char m_IoMap[4096];
    int m_IoMapSize = 0;


    int EXPECTED_WKC_PDO = 0;
    int wkc_error_count_PDO = 0;
    int timeout_count_PDO = 0;
    unsigned long long tickCount_PDO = 0;
    int wkc_PDO = 0;

    unsigned long long tickCount_RunRealTimeCycle = 0;
    unsigned long long tickCount_PLC = 0;

    uint64_t DC_reference_time = 0;
    int wk_read = 0;
    int wk_write = 0;

    


    AsyncCommandSlot m_asyncCmd;


    //Motion----------------------------------------------
    MotionCore m_Motion;
    std::vector<AxisContext> m_Axes;

    int test_timer = 0;
     int test_dir = 1; // 1: 正向, -1: 反向

     int debug_EDM = 1;
   


      int timer_10ms = 0;
     int timer_100ms = 0;
     int timer_500ms = 0;
     int timer_1000ms = 0;
     int Debug_test_timer = 0;

     int timer_10ms_Count = 0;
     int timer_100ms_Count = 0;
     int timer_500ms_Count = 0;
     int timer_1000ms_Count = 0;
     int Debug_test_timer_Count = 0;




     //主系統區塊-----------------------------------------------------------------------------------
     int TotalSlave_WKC_Count = 0;//從站統計WKC 分數 判斷是否失聯
     void Get_TotalSlave_WKC_Count();//取得從站WKC 分數
     
     int RunRealTimeCycle_EDM_SINKER_MODE();//主要程式迴圈執行 EDM 雕磨模式



     NCManager* m_NC = nullptr;
};

