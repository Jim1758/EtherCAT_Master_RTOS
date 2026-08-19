/*
 * 檔案：EtherCAT_Function.cpp
 * 版本：EtherCAT DC Release Candidate RC1
 *
 * 主要責任：
 * 1. 建立 EtherCAT Ethernet Header 與 Datagram。
 * 2. 提供 BRD/BWR/APRD/APWR/LRW/FPWR/FPRD 基本通訊命令。
 * 3. 提供 CoE SDO、EEPROM、FMMU 與從站狀態操作。
 * 4. 測量並設定 Distributed Clocks propagation delay。
 * 5. 在 PDO 即時循環以 LRW + FRMW 同一 Frame 交換 process image 與 DC time。
 * 6. 發布 TX timing 與 RX Soft/Hard Deadline 診斷快照。
 *
 * 執行緒分工：
 * - 初始化/SDO/狀態命令：非 PDO 即時流程使用。
 * - ecx_LRW_FRMW()：Priority 64 PDO 路徑使用，不可加入 RtPrintf。
 * - PrintDcRuntimeDiagnostics1000ms()：Priority 50 讀取本檔案發布的快照。
 *
 * 正式候選版關鍵參數：
 * - PDO cycle：250 us（4 kHz）。
 * - RX Soft Deadline：205 us。
 * - RX Hard Deadline：210 us。
 * - RX coarse wait request：50 us，最多兩次。
 * - RTX64 HAL：25 us；NAL interrupt/TX complete priority：70/70。
 *
 * 安全原則：
 * - Hard Deadline 後才回到軟體的 Frame 即使內容正確也不採用。
 * - Timeout 後下一週期先 drain 最多 8 個殘留 RX Frame，避免資料錯週期。
 * - 所有 Priority 64 診斷只做計數與 seqlock publish，不做格式化輸出。
 */
#include "EtherCatMaster.h"
#include "ConfigReader.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <stdio.h>
#include <algorithm>
#include <cctype>
#include <string>
#define MAX_MBX_SIZE 1024



/*
 * 掃描 EtherCAT 從站並配置 station address。
 * 回傳廣播讀取取得的從站/WKC 數；0 表示總線沒有有效回應。
 */
int EtherCatMaster::ScanSlaves()
{
    int wkc = ecx_BRD(0x0000, REG_AL_STATUS, 2, 200);
    if (wkc <= 0)
    {
        return 0;
    }

    // 廣播重置 (讓大家回到 INIT 狀態，清空錯誤)
    uint16_t resetCmd = 0x0011; // Error Ack + Init State
    ecx_BWR(0x0000, REG_AL_CONTROL, 2, &resetCmd, 50);
    RtSleep(100); // 等一下讓硬體重置


    //分配物理地址
    for (int i = 1; i <= wkc; i++)
    {
        uint16_t adp = (uint16_t)(1 - i);
        uint16_t new_addr = 0x1000 + i;

        int ret = ecx_APWR(adp, 0x0010, 2, &new_addr, 200);
        if (ret > 0)
        {
            // 儲存到我們的結構中
            m_slaveInfo[i - 1].configAddr = new_addr;
            m_slaveInfo[i - 1].APRDAPWR_Addr = adp;
            InitSlaveMailboxInfo(i - 1);


        }
        else
        {
        }
    }



    return wkc;
}

/*
 * BRD：廣播讀取所有從站的同一暫存器。
 * ADP/ADO 指定 EtherCAT 位址，回傳 WKC；timeout 保留既有呼叫介面。
 */
int EtherCatMaster::ecx_BRD(uint16_t ADP, uint16_t ADO, uint16_t length, int timeout)
{
    if (!m_pNic) return 0;

    uint8_t* frame = m_txBuffer;

    // --- 1. Ethernet Header (14 bytes) ---
    uint8_t dec_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // 廣播 MAC
    uint8_t src_mac[6];

    // [關鍵] 從 NIC Driver 取得真實 MAC，避免被 Switch 阻擋
    m_pNic->GetMacAddress(src_mac);

    memcpy(&frame[0], dec_mac, 6); // 目的 MAC
    memcpy(&frame[6], src_mac, 6); // 來源 MAC
    frame[12] = 0x88; // EtherType: EtherCAT
    frame[13] = 0xA4;

    // --- 2. EtherCAT Header (2 bytes) ---
    // Length: 10 (Datagram Header) + Data Length + 2 (WKC)
    uint16_t current_len = 10 + length + 2;
    uint16_t ec_header = current_len | (0x1000); // Type 1

    frame[14] = ec_header & 0xFF;
    frame[15] = (ec_header >> 8) & 0xFF;

    // --- 3. Datagram Header (10 bytes) ---
    uint8_t currentIdx = m_idx++; // 取得並遞增 Index

    frame[16] = 0x07; // Command: BRD (廣播讀取)
    frame[17] = currentIdx;
    frame[18] = ADP & 0xFF;
    frame[19] = (ADP >> 8) & 0xFF;
    frame[20] = ADO & 0xFF; // 暫存器位址
    frame[21] = (ADO >> 8) & 0xFF;

    // 資料長度 (11 bits)
    uint16_t len_field = length & 0x7FF;
    frame[22] = len_field & 0xFF;
    frame[23] = (len_field >> 8) & 0xFF;

    frame[24] = 0x00; // IRQ
    frame[25] = 0x00;

    // --- 4. Data & WKC ---
    // 資料區清零 (等待從站填寫)
    if (length > 0) memset(&frame[26], 0, length);

    // WKC 初始為 0
    frame[26 + length] = 0x00;
    frame[26 + length + 1] = 0x00;

    // --- 5. 發送封包 ---
    int total_send_len = 14 + 2 + 10 + length + 2;
    m_pNic->SendPacket(frame, total_send_len);

    // --- 6. 接收回應 (Receive Loop) ---
    int max_retries = timeout * 100;
    LARGE_INTEGER wait; wait.QuadPart = 10;

    while (max_retries-- > 0)
    {
        int rxLen = m_pNic->ReceivePacket(m_rxBuffer);
        if (rxLen > 0)
        {
            // 檢查是否為 EtherCAT 封包 (0x88A4)
            if (m_rxBuffer[12] == 0x88 && m_rxBuffer[13] == 0xA4)
            {
                // 檢查 Index 是否吻合
                if (m_rxBuffer[17] == currentIdx)
                {
                    // 計算 WKC 位置並讀取
                    int wkc_offset = 26 + length;
                    if (rxLen >= wkc_offset + 2)
                    {
                        uint16_t wkc = m_rxBuffer[wkc_offset] | (m_rxBuffer[wkc_offset + 1] << 8);
                        return wkc; // 回傳 WKC (響應的從站數量)
                    }
                }
            }
        }
        RtSleepFt(&wait);
    }
    return 0; // 超時
}
/* BWR：將同一份資料廣播寫入所有從站，回傳 WKC。 */
int EtherCatMaster::ecx_BWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout)
{
    if (!m_pNic) return 0;

    uint8_t* frame = m_txBuffer;

    // --- 1. Ethernet Header ---
    uint8_t dec_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t src_mac[6];
    m_pNic->GetMacAddress(src_mac);

    memcpy(&frame[0], dec_mac, 6);
    memcpy(&frame[6], src_mac, 6);
    frame[12] = 0x88;
    frame[13] = 0xA4;

    // --- 2. EtherCAT Header ---
    uint16_t current_len = 10 + length + 2;
    uint16_t ec_header = current_len | (0x1000);

    frame[14] = ec_header & 0xFF;
    frame[15] = (ec_header >> 8) & 0xFF;

    // --- 3. Datagram Header ---
    uint8_t currentIdx = m_idx++;

    frame[16] = 0x08; // Command: BWR (廣播寫入)
    frame[17] = currentIdx;
    frame[18] = ADP & 0xFF;
    frame[19] = (ADP >> 8) & 0xFF;
    frame[20] = ADO & 0xFF;
    frame[21] = (ADO >> 8) & 0xFF;

    uint16_t len_field = length & 0x7FF;
    frame[22] = len_field & 0xFF;
    frame[23] = (len_field >> 8) & 0xFF;

    frame[24] = 0x00;
    frame[25] = 0x00;

    // --- 4. Data Copy ---
    // 將要寫入的資料複製到封包中
    if (length > 0 && data != nullptr) {
        memcpy(&frame[26], data, length);
    }

    // WKC
    frame[26 + length] = 0x00;
    frame[26 + length + 1] = 0x00;

    // --- 5. 發送 ---
    int total_send_len = 14 + 2 + 10 + length + 2;
    m_pNic->SendPacket(frame, total_send_len);

    // --- 6. 接收回應 ---
    int max_retries = timeout * 100;
    LARGE_INTEGER wait; wait.QuadPart = 10;

    while (max_retries-- > 0)
    {
        int rxLen = m_pNic->ReceivePacket(m_rxBuffer);
        if (rxLen > 0)
        {
            if (m_rxBuffer[12] == 0x88 && m_rxBuffer[13] == 0xA4)
            {
                if (m_rxBuffer[17] == currentIdx)
                {
                    int wkc_offset = 26 + length;
                    if (rxLen >= wkc_offset + 2)
                    {
                        uint16_t wkc = m_rxBuffer[wkc_offset] | (m_rxBuffer[wkc_offset + 1] << 8);
                        return wkc;
                    }
                }
            }
        }
        RtSleepFt(&wait);
    }
    return 0; // 超時
}
/* APRD：以 auto-increment physical address 讀取指定從站暫存器。 */
int EtherCatMaster::ecx_APRD(uint16_t ADP, uint16_t ADO, uint16_t length, void* data, int timeout)
{
    if (!m_pNic) return 0;

    uint8_t* frame = m_txBuffer;

    // --- Ethernet Header ---
    uint8_t dec_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t src_mac[6];
    m_pNic->GetMacAddress(src_mac); // 使用真實 MAC

    memcpy(&frame[0], dec_mac, 6);
    memcpy(&frame[6], src_mac, 6);
    frame[12] = 0x88;
    frame[13] = 0xA4;

    // --- EtherCAT Header ---
    uint16_t current_len = 10 + length + 2;
    uint16_t ec_header = current_len | (0x1000);
    frame[14] = ec_header & 0xFF;
    frame[15] = (ec_header >> 8) & 0xFF;

    // --- Datagram Header ---
    uint8_t currentIdx = m_idx++; // 取得並遞增 Index

    frame[16] = 0x01; // Command: APRD
    frame[17] = currentIdx;

    // [位址處理] ADP 直接填入
    // 例如傳入 (uint16_t)-1 會變成 0xFFFF (代表鏈路上的第 2 個裝置)
    frame[18] = ADP & 0xFF;
    frame[19] = (ADP >> 8) & 0xFF;

    frame[20] = ADO & 0xFF;
    frame[21] = (ADO >> 8) & 0xFF;

    uint16_t len_field = length & 0x7FF;
    frame[22] = len_field & 0xFF;
    frame[23] = (len_field >> 8) & 0xFF;

    frame[24] = 0x00;
    frame[25] = 0x00;

    // Data 區塊清零 (等待從站填寫)
    if (length > 0) memset(&frame[26], 0, length);

    // WKC
    frame[26 + length] = 0x00;
    frame[26 + length + 1] = 0x00;

    // 發送
    int total_send_len = 14 + 2 + 10 + length + 2;
    m_pNic->SendPacket(frame, total_send_len);

    // 接收迴圈 (Index Matching)
    int max_retries = timeout * 100;
    LARGE_INTEGER wait; wait.QuadPart = 10;

    while (max_retries-- > 0)
    {
        int rxLen = m_pNic->ReceivePacket(m_rxBuffer);
        if (rxLen > 0)
        {
            if (m_rxBuffer[12] == 0x88 && m_rxBuffer[13] == 0xA4)
            {
                // Index Matching (Offset 17)
                if (m_rxBuffer[17] == currentIdx)
                {
                    int wkc_offset = 26 + length;
                    if (rxLen >= wkc_offset + 2)
                    {
                        uint16_t wkc = m_rxBuffer[wkc_offset] | (m_rxBuffer[wkc_offset + 1] << 8);

                        // 如果 WKC > 0，代表讀取成功，把資料複製回 data 指標
                        if (wkc > 0 && data != nullptr) {
                            memcpy(data, &m_rxBuffer[26], length);
                        }
                        return wkc;
                    }
                }
            }
        }
        RtSleepFt(&wait);
    }
    return 0; // Timeout
}
/* APWR：以 auto-increment physical address 寫入指定從站暫存器。 */
int EtherCatMaster::ecx_APWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout)
{
    if (!m_pNic) return 0;

    uint8_t* frame = m_txBuffer;

    // --- Ethernet Header ---
    uint8_t dec_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t src_mac[6];
    m_pNic->GetMacAddress(src_mac);

    memcpy(&frame[0], dec_mac, 6);
    memcpy(&frame[6], src_mac, 6);
    frame[12] = 0x88;
    frame[13] = 0xA4;

    // --- EtherCAT Header ---
    uint16_t current_len = 10 + length + 2;
    uint16_t ec_header = current_len | (0x1000);
    frame[14] = ec_header & 0xFF;
    frame[15] = (ec_header >> 8) & 0xFF;

    // --- Datagram Header ---
    uint8_t currentIdx = m_idx++;

    frame[16] = 0x02; // Command: APWR
    frame[17] = currentIdx;

    frame[18] = ADP & 0xFF;
    frame[19] = (ADP >> 8) & 0xFF;
    frame[20] = ADO & 0xFF;
    frame[21] = (ADO >> 8) & 0xFF;

    uint16_t len_field = length & 0x7FF;
    frame[22] = len_field & 0xFF;
    frame[23] = (len_field >> 8) & 0xFF;

    frame[24] = 0x00;
    frame[25] = 0x00;

    // Data Copy
    if (length > 0 && data != nullptr) {
        memcpy(&frame[26], data, length);
    }

    // WKC
    frame[26 + length] = 0x00;
    frame[26 + length + 1] = 0x00;

    // 發送
    int total_send_len = 14 + 2 + 10 + length + 2;
    m_pNic->SendPacket(frame, total_send_len);

    // 接收迴圈
    int max_retries = timeout * 100;
    LARGE_INTEGER wait; wait.QuadPart = 10;

    while (max_retries-- > 0)
    {
        int rxLen = m_pNic->ReceivePacket(m_rxBuffer);
        if (rxLen > 0)
        {
            if (m_rxBuffer[12] == 0x88 && m_rxBuffer[13] == 0xA4)
            {
                if (m_rxBuffer[17] == currentIdx)
                {
                    int wkc_offset = 26 + length;
                    if (rxLen >= wkc_offset + 2)
                    {
                        uint16_t wkc = m_rxBuffer[wkc_offset] | (m_rxBuffer[wkc_offset + 1] << 8);
                        return wkc;
                    }
                }
            }
        }
        RtSleepFt(&wait);
    }
    return 0; // Timeout
}
/* LRW：以 logical address 同時寫出 Output PDO 並讀回 Input PDO。 */
int EtherCatMaster::ecx_LRW(uint32_t LogAddr, uint16_t length, void* data, int timeout)
{
    if (!m_pNic) return 0;

    uint8_t* frame = m_txBuffer;

    // --- 1. Ethernet Header (14 bytes) ---
    uint8_t dec_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t src_mac[6];
    m_pNic->GetMacAddress(src_mac);

    memcpy(&frame[0], dec_mac, 6);
    memcpy(&frame[6], src_mac, 6);
    frame[12] = 0x88;
    frame[13] = 0xA4;

    // --- 2. EtherCAT Header (2 bytes) ---
    uint16_t current_len = 10 + length + 2;
    uint16_t ec_header = current_len | (0x1000);

    frame[14] = ec_header & 0xFF;
    frame[15] = (ec_header >> 8) & 0xFF;

    // --- 3. Datagram Header (10 bytes) ---
    uint8_t currentIdx = m_idx++;

    // [修改 1] Command: 12 = LRW (Logical Read Write)
    frame[16] = 12;
    frame[17] = currentIdx;

    // [修改 2] Logical Address (32-bit)
    // 將 32位元位址拆成 4個 Byte (Little Endian)
    frame[18] = LogAddr & 0xFF;         // Byte 0
    frame[19] = (LogAddr >> 8) & 0xFF;  // Byte 1
    frame[20] = (LogAddr >> 16) & 0xFF; // Byte 2
    frame[21] = (LogAddr >> 24) & 0xFF; // Byte 3

    // Length
    uint16_t len_field = length & 0x7FF;
    frame[22] = len_field & 0xFF;
    frame[23] = (len_field >> 8) & 0xFF;

    frame[24] = 0x00;
    frame[25] = 0x00;

    // --- 4. Data & WKC ---

    // [修改 3-A] 發送 Payload (Output)
    // 不再是 memset 清零，而是把 m_IoMap 的內容複製進去！
    if (length > 0 && data != nullptr) {
        memcpy(&frame[26], data, length);
    }

    // WKC 初始為 0
    frame[26 + length] = 0x00;
    frame[26 + length + 1] = 0x00;

    // --- 5. 發送封包 ---
    int total_send_len = 14 + 2 + 10 + length + 2;
    m_pNic->SendPacket(frame, total_send_len);

    // --- 6. 接收回應 (Receive Loop) ---
    int max_retries = timeout * 100; // 視您的 timer 解析度而定
    LARGE_INTEGER wait; wait.QuadPart = 10; // 1us (RTX64 sleep ft)

    while (max_retries-- > 0)
    {
        int rxLen = m_pNic->ReceivePacket(m_rxBuffer);
        if (rxLen > 0)
        {
            // 檢查 EtherType
            if (m_rxBuffer[12] == 0x88 && m_rxBuffer[13] == 0xA4)
            {
                // 檢查 Index
                if (m_rxBuffer[17] == currentIdx)
                {
                    // 檢查 Command 是否為 12 (有些 slave 錯誤會回傳別的)
                    if (m_rxBuffer[16] == 12)
                    {
                        // [修改 3-B] 接收 Payload (Input)
                        // 把收到的資料複製回使用者的 m_IoMap
                        // 這樣您的 DI 和 AD 數值就會更新了
                        if (length > 0 && data != nullptr) {
                            memcpy(data, &m_rxBuffer[26], length);
                        }

                        // 讀取 WKC
                        int wkc_offset = 26 + length;
                        if (rxLen >= wkc_offset + 2)
                        {
                            uint16_t wkc = m_rxBuffer[wkc_offset] | (m_rxBuffer[wkc_offset + 1] << 8);
                            return wkc;
                        }
                    }
                }
            }
        }
        RtSleepFt(&wait);
    }
    return -1; // -1 代表超時/失敗 (區分 WKC=0 的情況)
}
/*
 * CoE SDO Write：透過 Mailbox 寫入指定從站 Object Dictionary。
 * 僅用於初始化或非即時命令，不應放入 4 kHz PDO 路徑。
 */
int EtherCatMaster::ecx_SDOwrite(int slave_pos, uint16_t index, uint8_t subindex, int CA, int size, void* data, int timeout)
{
    // ========================================================================
    // 1. 準備參數與地址
    // ========================================================================
    uint16_t mbxOutAddr = m_slaveInfo[slave_pos].mbxOutAddr;
    uint16_t mbxInAddr = m_slaveInfo[slave_pos].mbxInAddr;
    uint8_t mbx_req[MAX_MBX_SIZE] = { 0 }; // 請求封包
    uint8_t mbx_res[MAX_MBX_SIZE] = { 0 }; // 回應緩衝區

    // 防止 buffer overflow (根據信箱最大長度截斷)
    if (size > (MAX_MBX_SIZE - 32)) return 0; // 預留 Header 空間

    // ========================================================================
    // 2. 暴力破解 Loop (嘗試不同 Counter)
    // ========================================================================
    for (int try_cnt = 0; try_cnt < 8; try_cnt++) {

        // 更新 Counter (每次重試都換一個號碼)
        m_mboxCnt++;
        uint8_t current_cnt = m_mboxCnt & 0x7;

        // --- A. 建構請求封包 (Request) ---

        // 1. Mailbox Header
        // Byte 2-3: Station Address (固定 0)
        *(uint16_t*)(&mbx_req[2]) = 0x0000;
        // Byte 4: Channel(0) + Priority(0)
        mbx_req[4] = 0x00;
        // Byte 5: Type(3=CoE) + Counter
        mbx_req[5] = 0x03 | (current_cnt << 4);

        // 2. CoE Header
        // Byte 6-7: 0x2000 (CoE Service)
        *(uint16_t*)(&mbx_req[6]) = 0x2000;

        // 3. SDO Header & Data 處理
        // 這是與 Read 最大的不同點，要判斷數據大小

        uint16_t ethercat_data_len = 0; // 用來填寫 Byte 0-1 的長度

        // [模式 A] Complete Access (CA) - 讀寫整個物件
        if (CA) {
            mbx_req[8] = 0x31; // Command: Download Complete Access + Size Indicator
            mbx_req[9] = index & 0xFF;
            mbx_req[10] = (index >> 8) & 0xFF;
            mbx_req[11] = subindex;

            // CA 模式下，通常使用 Normal Transfer 格式 (長度放 Byte 12-15)
            *(uint32_t*)(&mbx_req[12]) = (uint32_t)size;

            // 數據放在 Byte 16 之後
            memcpy(&mbx_req[16], data, size);

            ethercat_data_len = 16 + size; // 10(Mbx) + 2(CoE) + 4(SDO header) + 4(Size) + Data
        }
        // [模式 B] Expedited Transfer (<= 4 Bytes) - 最常用的模式
        else if (size <= 4) {
            // Command 0x20: Download Request
            // Bit 1=1 (Expedited)
            // Bit 0=1 (Size Indicated)
            // Bit 2-3 (Size): 4-size (00=4byte, 01=3byte, 10=2byte, 11=1byte)
            mbx_req[8] = 0x23 | ((4 - size) << 2);

            mbx_req[9] = index & 0xFF;
            mbx_req[10] = (index >> 8) & 0xFF;
            mbx_req[11] = subindex;

            // 數據直接放在 Byte 12-15
            memcpy(&mbx_req[12], data, size);

            // 總長度固定 (因為即使寫 1 byte，封包還是要補滿到 4 byte 位置)
            ethercat_data_len = 16; // 10(Mbx) + 2(CoE) + 4(SDO header+Data)
        }
        // [模式 C] Normal Transfer (> 4 Bytes)
        else {
            mbx_req[8] = 0x21; // Download Normal + Size Indicator
            mbx_req[9] = index & 0xFF;
            mbx_req[10] = (index >> 8) & 0xFF;
            mbx_req[11] = subindex;

            // 總數據大小放在 Byte 12-15
            *(uint32_t*)(&mbx_req[12]) = (uint32_t)size;

            // 數據放在 Byte 16 之後
            memcpy(&mbx_req[16], data, size);

            ethercat_data_len = 16 + size;
        }

        // 4. 填寫 EtherCAT Header 長度 (Byte 0-1)
        // 注意：長度 = 整個 Mailbox Payload 長度 (不含這兩個 byte 也不含 Station Addr)
        // 這裡我們用計算出來的總長度 - 2 (因為 Length 欄位不包含自己) 
        // 但標準寫法通常是填入 Mailbox Data 的長度
        // Mailbox Data = (CoE Header + SDO Header + Data)
        *(uint16_t*)(&mbx_req[0]) = ethercat_data_len - 6; // 減去 MbxHeader(6 bytes) ? 
        // 更正：EtherCAT Length (Byte0-1) 是指隨後數據的長度。
        // Mailbox Frame = [Length(2)][Addr(2)][MbxHeader(2)][CoE(2)][SDO(4)][Data...]
        // ecx_APWR 函數會幫我們處理外層的 datagram header，我們這裡只需要填 Mailbox 內部的長度。
        // Wait, ecx_APWR 的 length 參數決定了 datagram 的長度。
        // Mailbox 協議裡的 Byte 0-1 是 "Length"，它指的是 Mailbox Service Data 的長度 (Counter Byte 之後的所有東西)
        // 通常是: CoE Header(2) + SDO Header(4) + Data
        *(uint16_t*)(&mbx_req[0]) = ethercat_data_len - 6;

        // --- B. 發送請求 (APWR) ---
        // 注意：傳入 APWR 的長度必須是我們計算出的完整封包長度
        int wkc = ecx_APWR(m_slaveInfo[slave_pos].APRDAPWR_Addr, mbxOutAddr, m_slaveInfo[slave_pos].mbxOutLength, mbx_req, timeout);

        if (wkc <= 0) {
            continue; // 寫入失敗，重試
        }

        // --- C. 接收回應 (APRD) ---
        int quick_retries = 20;
        LARGE_INTEGER sleepTime; sleepTime.QuadPart = 100 * 10; // 100us

        for (int i = 0; i < quick_retries; i++) {
            wkc = ecx_APRD(m_slaveInfo[slave_pos].APRDAPWR_Addr, mbxInAddr, m_slaveInfo[slave_pos].mbxInLength, mbx_res, 500);

            if (wkc > 0) {
                // 解析回應
                uint8_t  res_header_type = mbx_res[5] & 0x0F;
                uint8_t  res_cmd = mbx_res[8];
                uint16_t res_index = mbx_res[9] | (mbx_res[10] << 8);

                if (res_header_type == 0x03) { // CoE

                    // 檢查 Index 是否匹配
                    if (res_index == index) {

                        // 情況 A: 寫入成功 (Success)
                        // 下載成功的回應碼通常是 0x60
                        if (res_cmd == 0x60) {
                            return 1; // 成功！
                        }

                        // 情況 B: 發生錯誤 (Abort 0x80)
                        if (res_cmd == 0x80) {
                            uint32_t abortCode = *(uint32_t*)&mbx_res[12];
                            DEBUG_PRINT("[SDO Write Abort] Index:0x%X, Error: 0x%08X\n", index, abortCode);
                            return 0; // 失敗
                        }
                    }
                }
            }
            RtSleepFt(&sleepTime);
        }
    }

    return 0; // 全部嘗試失敗
}
/*
 * CoE SDO Read：透過 Mailbox 讀取指定從站 Object Dictionary。
 * size 同時是輸入緩衝區大小與輸出實際資料長度。
 */
int EtherCatMaster::ecx_SDOread(int slave_pos, uint16_t index, uint8_t subindex, int CA, int* size, void* data, int timeout)
{
    // ========================================================================
    // 1. 準備參數與地址
    // ========================================================================
    // 如果 slave_pos 是順序索引 (0=第1台, -1=第2台...)，直接轉成 uint16_t 使用
    //uint16_t adp = (uint16_t)slave_pos;

    // 決定 Mailbox 地址
    // 一般 CoE 驅動器: Out=0x1800, In=0x1880
    // 特殊 IO 模組 (如您之前用的): Out=0x1000, In=0x1080 (根據 Index 判斷)
    uint16_t mbxOutAddr = m_slaveInfo[slave_pos].mbxOutAddr;
    uint16_t mbxInAddr = m_slaveInfo[slave_pos].mbxInAddr;
    uint8_t mbx_req[MAX_MBX_SIZE] = { 0 }; // 請求封包
    uint8_t mbx_res[MAX_MBX_SIZE] = { 0 }; // 回應緩衝區

    // ========================================================================
    // 2. 暴力破解 Loop (嘗試不同 Counter，確保有一個能對上)
    // ========================================================================
    for (int try_cnt = 0; try_cnt < 8; try_cnt++) {

        // --- A. 建構請求封包 (Request) ---
        *(uint16_t*)(&mbx_req[0]) = 0x000A; // Length (10 bytes)
        *(uint16_t*)(&mbx_req[2]) = 0x0000; // Station Address

        // 更新 Counter
        m_mboxCnt++;
        uint8_t current_cnt = m_mboxCnt & 0x7;

        // [關鍵修正 !!!] -----------------------------------------------------
        // 之前填反了，導致 Type=0 無效。現在修正為：
        // Byte 4: Channel(0) + Priority(0) -> 必須是 0x00
        mbx_req[4] = 0x00;

        // Byte 5: Type(3=CoE) + Counter
        // 格式: (Counter << 4) | 0x03
        mbx_req[5] = 0x03 | (current_cnt << 4);
        // --------------------------------------------------------------------

        *(uint16_t*)(&mbx_req[6]) = 0x2000; // CoE Header (SDO Req)
        if (CA) {
            mbx_req[8] = 0x50; // Upload Request + Complete Access
            // 注意：CA 模式下，通常 subindex 建議為 0 (讀全部) 或 1 (不含 count)
            // 這裡保留您傳入的 subindex，但請留意規範

        }
        else {
            mbx_req[8] = 0x40; // Standard Upload Request

        }
        mbx_req[9] = index & 0xFF;
        mbx_req[10] = (index >> 8) & 0xFF;
        mbx_req[11] = subindex;





        // --- B. 發送請求 (APWR) ---
        // 使用 ecx_APWR 寫入 Mailbox Out
        int wkc = ecx_APWR(m_slaveInfo[slave_pos].APRDAPWR_Addr, mbxOutAddr, m_slaveInfo[slave_pos].mbxOutLength, mbx_req, timeout);

        if (wkc <= 0) {
            // 如果連寫都失敗 (WKC=0)，代表該位置沒有 Slave 或 Mailbox 沒開
            // 這裡可以選擇 continue 重試，或直接回傳失敗
            continue;
        }

        // --- C. 接收回應 (APRD) ---
        // Slave 需要一點時間處理，所以我們小迴圈輪詢
        int quick_retries = 20;
        LARGE_INTEGER sleepTime; sleepTime.QuadPart = 100 * 10; // 100us

        for (int i = 0; i < quick_retries; i++) {

            // 使用 ecx_APRD 讀取 Mailbox In
            // 注意：讀取時長度通常建議給足 (例如 16 或更大，看回傳數據量)
            wkc = ecx_APRD(m_slaveInfo[slave_pos].APRDAPWR_Addr, mbxInAddr, m_slaveInfo[slave_pos].mbxInLength, mbx_res, 500);

            if (wkc > 0) {
                // 解析回應
                uint8_t  res_header_type = mbx_res[5] & 0x0F; // 應該是 3 (CoE)
                uint8_t  res_cmd = mbx_res[8];               // SDO Command
                uint16_t res_index = mbx_res[9] | (mbx_res[10] << 8);

                // 檢查 1: 必須是 CoE 封包
                if (res_header_type == 0x03) {

                    // 檢查 2: Index 必須匹配 (確認不是舊的回應)
                    if (res_index == index) {

                        // 情況 A: 讀取成功 (0x4x)
                        if ((res_cmd & 0xE0) == 0x40) {
                            int valid_bytes = 4;
                            // 計算實際數據長度 (expedited transfer)
                            if (res_cmd & 0x02) {
                                int empty_bytes = (res_cmd >> 2) & 0x03;
                                valid_bytes = 4 - empty_bytes;
                            }

                            // 複製數據回傳給使用者
                            if (data != NULL && size != NULL) {
                                int copy_size = (*size < valid_bytes) ? *size : valid_bytes;
                                memcpy(data, &mbx_res[12], copy_size);
                                *size = copy_size;
                            }
                            return 1; // 成功！
                        }

                        // 情況 B: 發生錯誤 (Abort 0x80)
                        if (res_cmd == 0x80) {
                            uint32_t abortCode = *(uint32_t*)&mbx_res[12];
                            DEBUG_PRINT("[SDO Abort] Error Code: 0x%08X\n", abortCode);
                            return 0; // 失敗
                        }
                    }
                }
            }
            // 沒讀到或還沒準備好，稍等一下再試
            RtSleepFt(&sleepTime);
        }
    }

    return 0; // 試了所有 Counter 都沒回應，宣告失敗
}
/* FPWR：以已配置的 station address 寫入從站暫存器。 */
int EtherCatMaster::ecx_FPWR(uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout)
{
    // Command 5 = FPWR (寫入指定物理位址)
    return SendAndReceiveRegister(0x05, slaveAddr, regAddr, data, len, timeout);
}
/* FPRD：以已配置的 station address 讀取從站暫存器。 */
int EtherCatMaster::ecx_FPRD(uint16_t slaveAddr, uint16_t regAddr, void* buffer, int len, int timeout)
{
    // Command 4 = FPRD (讀取指定物理位址)
    return SendAndReceiveRegister(0x04, slaveAddr, regAddr, buffer, len, timeout);
}

/*
 * 建立單一 register datagram、送出、比對 EtherType/Index/Command 後取回資料與 WKC。
 * FPWR/FPRD 共用此函式，回傳值沿用 EtherCatMaster.h 的 int 宣告。
 */
int EtherCatMaster::SendAndReceiveRegister(
    uint8_t cmd,
    uint16_t slaveAddr,
    uint16_t regAddr,
    void* data,
    int len,
    int timeout)
{
    // =========================================================
    // EtherCAT Register Send / Receive Core
    //
    // 0x04 = FPRD
    // 0x05 = FPWR
    // 0x0E = FRMW
    //
    // 本版本重點：
    //
    // 1. 每個 Datagram 使用唯一 Index
    // 2. RX 必須比對 Cmd
    // 3. RX 必須比對 Index
    // 4. 防止上一個 Cycle 的 Late Response
    //    被下一次 Register Access 誤收
    // =========================================================


    // ---------------------------------------------------------
    // 基本防呆
    // ---------------------------------------------------------
    if (m_pNic == nullptr)
    {
        return 0;
    }

    if (len <= 0 ||
        len > 1480)
    {
        return 0;
    }


    // =========================================================
    // 1. Buffer
    // =========================================================
    uint8_t sendBuf[1518];
    uint8_t recvBuf[1518];

    memset(
        sendBuf,
        0,
        sizeof(sendBuf));

    memset(
        recvBuf,
        0,
        sizeof(recvBuf));


    int idx = 0;


    // =========================================================
    // 2. Ethernet Header
    // =========================================================

    // Destination MAC = Broadcast
    for (int i = 0; i < 6; i++)
    {
        sendBuf[idx++] = 0xFF;
    }


    // Source MAC
    //
    // 保留目前 NIC Driver 架構。
    // 你的原程式在這裡使用 0 Placeholder。
    for (int i = 0; i < 6; i++)
    {
        sendBuf[idx++] = 0x00;
    }


    // EtherType = EtherCAT 0x88A4
    sendBuf[idx++] = 0x88;
    sendBuf[idx++] = 0xA4;


    // =========================================================
    // 3. EtherCAT Header
    // =========================================================
    //
    // Datagram Header = 10 Bytes
    // Data            = len
    // WKC             = 2 Bytes
    // =========================================================

    uint16_t totalLen =
        (uint16_t)(
            10 +
            len +
            2);


    uint16_t ecHeader =
        (totalLen & 0x07FF) |
        0x1000;


    sendBuf[idx++] =
        (uint8_t)(
            ecHeader & 0xFF);


    sendBuf[idx++] =
        (uint8_t)(
            (ecHeader >> 8) & 0xFF);


    // =========================================================
    // 4. EtherCAT Datagram Header
    // =========================================================


    // ---------------------------------------------------------
    // Command
    // ---------------------------------------------------------
    sendBuf[idx++] =
        cmd;


    // ---------------------------------------------------------
    // Datagram Index
    //
    // 重要：
    //
    // 原本固定 = 0。
    //
    // 現在每一次 EtherCAT Register Access
    // 都取得新的 Index。
    //
    // uint8_t 自然會：
    //
    // 0 ... 255 -> 0
    //
    // Wrap 是正常的。
    // ---------------------------------------------------------
    uint8_t currentIdx =
        m_idx++;


    sendBuf[idx++] =
        currentIdx;


    // ---------------------------------------------------------
    // Configured Station Address
    // ---------------------------------------------------------
    sendBuf[idx++] =
        (uint8_t)(
            slaveAddr & 0xFF);


    sendBuf[idx++] =
        (uint8_t)(
            (slaveAddr >> 8) & 0xFF);


    // ---------------------------------------------------------
    // Register Address
    // ---------------------------------------------------------
    sendBuf[idx++] =
        (uint8_t)(
            regAddr & 0xFF);


    sendBuf[idx++] =
        (uint8_t)(
            (regAddr >> 8) & 0xFF);


    // ---------------------------------------------------------
    // Length
    // ---------------------------------------------------------
    uint16_t lenInfo =
        (uint16_t)len &
        0x07FF;


    sendBuf[idx++] =
        (uint8_t)(
            lenInfo & 0xFF);


    sendBuf[idx++] =
        (uint8_t)(
            (lenInfo >> 8) & 0xFF);


    // ---------------------------------------------------------
    // IRQ
    // ---------------------------------------------------------
    sendBuf[idx++] = 0x00;
    sendBuf[idx++] = 0x00;


    // =========================================================
    // 5. Datagram Data
    // =========================================================

    // FPWR：
    // Data = 要寫入的內容
    if (cmd == 0x05 &&
        data != nullptr)
    {
        memcpy(
            &sendBuf[idx],
            data,
            len);
    }


    // FPRD / FRMW：
    //
    // sendBuf 已經 memset = 0，
    // 所以 Data Area 保持 0 即可。
    idx += len;


    // =========================================================
    // 6. WKC Initial Value
    // =========================================================
    sendBuf[idx++] = 0x00;
    sendBuf[idx++] = 0x00;


    // =========================================================
    // 7. Send
    // =========================================================
    m_pNic->SendPacket(
        sendBuf,
        idx);


    // =========================================================
    // 8. RX Polling Configuration
    // =========================================================

    int maxRetries =
        timeout * 100;


    LARGE_INTEGER wait;

    // 原本設計：
    // 10 x 100ns = 1us
    //
    // 實際 timing 可能受到 RTX64 / NIC RX Path 影響，
    // 但這一步先以 Correctness 為主。
    wait.QuadPart =
        10;


    // =========================================================
    // 9. RX Loop
    // =========================================================
    while (maxRetries-- > 0)
    {
        int recvLen =
            m_pNic->ReceivePacket(
                recvBuf);


        if (recvLen > 0)
        {
            // =================================================
            // 最低基本長度
            // =================================================
            if (recvLen < 28)
            {
                RtSleepFt(
                    &wait);

                continue;
            }


            // =================================================
            // A. EtherType Matching
            // =================================================
            if (recvBuf[12] != 0x88 ||
                recvBuf[13] != 0xA4)
            {
                RtSleepFt(
                    &wait);

                continue;
            }


            // =================================================
            // B. Command Matching
            //
            // Offset 16 = EtherCAT Command
            // =================================================
            uint8_t recvCmd =
                recvBuf[16];


            if (recvCmd != cmd)
            {
                // 不是這一次 Register Access 的封包
                //
                // 直接忽略。
                RtSleepFt(
                    &wait);

                continue;
            }


            // =================================================
            // C. Datagram Index Matching
            //
            // Offset 17 = Datagram Index
            //
            // ★ 這是這次真正要修的核心
            // =================================================
            uint8_t recvIdx =
                recvBuf[17];


            if (recvIdx != currentIdx)
            {
                // -------------------------------------------------
                // 這代表可能是：
                //
                // - 前一個 Cycle 的 Late Response
                // - 其他 EtherCAT Datagram
                //
                // 絕對不能把它當成本次回應。
                // -------------------------------------------------

                RtSleepFt(
                    &wait);

                continue;
            }


            // =================================================
            // D. WKC Offset
            // =================================================
            int wkcOffset =
                14 +
                2 +
                10 +
                len;


            if (recvLen <
                (wkcOffset + 2))
            {
                RtSleepFt(
                    &wait);

                continue;
            }


            // =================================================
            // E. Read WKC
            // =================================================
            uint16_t wkc =
                (uint16_t)recvBuf[wkcOffset] |
                ((uint16_t)recvBuf[wkcOffset + 1]
                    << 8);


            if (wkc < 1)
            {
                RtSleepFt(
                    &wait);

                continue;
            }


            // =================================================
            // F. Copy Return Data
            //
            // FPRD / FRMW
            // =================================================
            if ((cmd == 0x04 ||
                cmd == 0x0E) &&
                data != nullptr)
            {
                const int dataOffset =
                    26;


                if (recvLen <
                    (dataOffset + len))
                {
                    return 0;
                }


                memcpy(
                    data,
                    &recvBuf[dataOffset],
                    len);
            }


            // =================================================
            // Success
            // =================================================
            return (int)wkc;
        }


        // =====================================================
        // 尚未收到 Response
        // =====================================================
        RtSleepFt(
            &wait);
    }


    // =========================================================
    // 10. Timeout
    // =========================================================

    // FRMW 每 250us 都執行，
    // 不要在 RT Thread 每次 Timeout 都洗 Log。
    if (cmd != 0x0E)
    {
        DEBUG_PRINT(
            "Register R/W Timeout! Cmd:0x%02X Addr:0x%X Reg:0x%X Idx:%u\n",
            cmd,
            slaveAddr,
            regAddr,
            (unsigned int)currentIdx);
    }


    return 0;
}


/* 從 Slave Information Interface EEPROM 讀取一個 16-bit word。 */
uint16_t EtherCatMaster::ReadSII_Word16(int slave_idx, uint16_t word_addr)
{
    uint16_t adp = m_slaveInfo[slave_idx].APRDAPWR_Addr;

    // =================================================================
    // 1. 準備 6 Bytes 的「組合包」
    // =================================================================
    // 結構: [Control(2 Bytes)] + [Address(4 Bytes)]
    uint8_t req_data[6];

    // Byte 0-1: 填入命令 0x0100 (Read Command)
    // Little Endian: 00 01
    *(uint16_t*)&req_data[0] = 0x0100;

    // Byte 2-5: 填入 EEPROM Word 地址
    // 例如要讀 0x001C -> 1C 00 00 00
    *(uint32_t*)&req_data[2] = word_addr;

    // =================================================================
    // 2. 發送「組合技」 (寫入 0x0502，長度 6)
    // =================================================================
    // 這裡我們只呼叫一次 APWR，就完成了原本兩次的工作！
    ecx_APWR(adp, 0x0502, 6, req_data, 200);

    // =================================================================
    // 3. 輪詢狀態 (這部分還是要乖乖等，不能省)
    // =================================================================
    uint16_t status = 0;
    int timeout = 200;
    bool success = false;

    do {
        ecx_APRD(adp, 0x0502, 2, &status, 200);

        // 檢查 Error
        if (status & 0x7800) {
            DEBUG_PRINT("EEPROM Error Slave[%d]: 0x%04X\n", slave_idx, status);
            return 0;
        }

        // 檢查 Busy (Bit 15)
        if ((status & 0x8000) == 0) {
            success = true;
            break;
        }
        RtSleep(1);
    } while (timeout-- > 0);

    if (!success) {
        DEBUG_PRINT("EEPROM Read Timeout! Slave[%d]\n", slave_idx);
        return 0;
    }

    // =================================================================
    // 4. 讀取數據 (這部分也沒變)
    // =================================================================
    uint32_t raw_data = 0;
    ecx_APRD(adp, 0x0508, 4, &raw_data, 200);

    return (uint16_t)(raw_data & 0xFFFF);
}

// 改回傳型別為 uint32_t
/* 連續讀取兩個 SII word 並組成 32-bit little-endian 數值。 */
uint32_t EtherCatMaster::ReadSII_Uint32(int slave_idx, uint16_t word_addr)
{
    uint16_t adp = m_slaveInfo[slave_idx].APRDAPWR_Addr;

    // =================================================================
    // 1. 準備 6 Bytes 的「組合包」
    // =================================================================
    uint8_t req_data[6];

    // Byte 0-1: Command 0x0100 (Read)
    *(uint16_t*)&req_data[0] = 0x0100;

    // Byte 2-5: EEPROM Word Address
    // 注意：這裡還是傳入 Word Address (例如 0x0008)，而不是 Byte Address
    *(uint32_t*)&req_data[2] = word_addr;

    // =================================================================
    // 2. 發送請求
    // =================================================================
    ecx_APWR(adp, 0x0502, 6, req_data, 200);

    // =================================================================
    // 3. 輪詢狀態 (等待 Busy Bit 清除)
    // =================================================================
    uint16_t status = 0;
    int timeout = 200;
    bool success = false;

    do
    {
        ecx_APRD(adp, 0x0502, 2, &status, 200);

        // 檢查 Error (Bit 14-11 通常保留或特定錯誤，檢查 0x7800 範圍)
        // 註：有些手冊定義 Error 為 Bit 11-13 (0x3800)，但 0x7800 也包含了高位檢查，通常沒問題
        if (status & 0x7800)
        {
            DEBUG_PRINT("EEPROM Error Slave[%d]: 0x%04X\n", slave_idx, status);
            return 0;
        }

        // 檢查 Busy (Bit 15) - 0表示完成
        if ((status & 0x8100) == 0x0100) { // 加強檢查：Busy=0 且 Command=1(Read)還在才算正常結束
            success = true;
            break;
        }
        // 簡單版檢查 (您原本的寫法)
        if ((status & 0x8000) == 0) {
            success = true;
            break;
        }

        RtSleep(1);
    } while (timeout-- > 0);

    if (!success) {
        DEBUG_PRINT("EEPROM Read Timeout! Slave[%d]\n", slave_idx);
        return 0;
    }

    // =================================================================
    // 4. 讀取數據 (關鍵修改處)
    // =================================================================
    uint32_t raw_data = 0;

    // 從 0x0508 讀取 4 Bytes (32 bits)
    // ESC 會把 [Word_Addr] 和 [Word_Addr + 1] 都放進來
    ecx_APRD(adp, 0x0508, 4, &raw_data, 200);

    // 直接回傳 32-bit 數據，不要轉型成 uint16_t
    return raw_data;
}

/* 寫入單一 FMMU entry，建立 Logical Address 與 Slave Process RAM 的映射。 */
int EtherCatMaster::WriteFmmuRegister(int slaveIdx, int fmmuIdx, uint32_t logAddr, uint16_t len, uint16_t physAddr, uint8_t type, int timeout)
{
    // 輔助函式：寫入單一 FMMU 設定
    // slaveIdx: 從站索引
    // fmmuIdx:  第幾號 FMMU (0, 1, 2...)
    // logAddr:  邏輯位址 (從 IoMap 算出來的)
    // len:      長度
    // physAddr: 物理位址 (0x1000, 0x1100...)
    // type:     1=Read(Input), 2=Write(Output)
    uint8_t fmmu[16];
    memset(fmmu, 0, 16);

    // 填寫 FMMU 內容
    fmmu[0] = logAddr & 0xFF;
    fmmu[1] = (logAddr >> 8) & 0xFF;
    fmmu[2] = (logAddr >> 16) & 0xFF;
    fmmu[3] = (logAddr >> 24) & 0xFF;

    fmmu[4] = len & 0xFF;
    fmmu[5] = (len >> 8) & 0xFF;

    fmmu[6] = 0x00; // Start Bit
    fmmu[7] = 0x07; // End Bit

    fmmu[8] = physAddr & 0xFF;
    fmmu[9] = (physAddr >> 8) & 0xFF;

    fmmu[10] = 0x00; // Start Bit
    fmmu[11] = type; // Type

    fmmu[12] = 0x01; // Enable FMMU

    // 計算 FMMU 暫存器位置: 0x0600 + (Index * 16)
    // FMMU0 = 0x0600, FMMU1 = 0x0610
    uint16_t regAddr = 0x0600 + (fmmuIdx * 16);

    // --- 以下為修改區塊：加入 WKC 檢查與重試機制 ---
    int wkc = 0;
    int retry = 0;
    const int MAX_RETRY = 5; // 最多重試 5 次

    // 使用 while 迴圈確保寫入成功
    while (retry < MAX_RETRY)
    {
        // 抓取 ecx_APWR 的回傳值 (即 WKC)
        // 參數中的 20 是 Timeout，通常單位是毫秒(ms)或微秒(us)，視您的底層 API 實作而定
        wkc = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, regAddr, 16, fmmu, timeout);

        // 對單一從站寫入時，若成功，WKC 會大於或等於 1
        if (wkc >= 1)
        {
            break; // 寫入成功，跳出迴圈
        }

        // 若 WKC == 0，代表從站沒收到或來不及處理，準備重試
        retry++;
        Sleep(1); // 給予網卡與從站 1 毫秒的緩衝時間再重試
    }

    // 防呆處理：如果重試 5 次都失敗，印出錯誤訊息方便除錯
    if (wkc == 0)
    {
        //DEBUG_PRINT("[Error] FMMU Write Failed! Slave: %d, FMMU: %d\n", slaveIdx, fmmuIdx);
    }

    return wkc;

}

/* 列印所有從站 AL state；僅供非即時診斷使用。 */
void EtherCatMaster::Printf_Slaves_State()
{
    int WK = 0;
    const auto& slaves = m_pEni->GetSlaves();
    int total_slaves = (int)slaves.size();
    DEBUG_PRINT("Printf_Slaves_State--------------------------------\n");
    for (int i = 0; i < total_slaves; i++)
    {
        uint16_t Slave_state = 0;
        WK = ecx_APRD(m_slaveInfo[i].APRDAPWR_Addr, 0x0130, 2, &Slave_state, 20);
        DEBUG_PRINT("State>>Slave>>0x%04X>>State>>0x%04X\n", m_slaveInfo[i].Product_Code, Slave_state);

    }
    DEBUG_PRINT("--------------------------------\n");
}

/* 列印 AL Status Code register 0x0134，協助定位狀態轉換錯誤。 */
void EtherCatMaster::Printf_AL_Status_Code()
{
    int WK = 0;
    const auto& slaves = m_pEni->GetSlaves();
    int total_slaves = (int)slaves.size();
    DEBUG_PRINT("Printf_AL_Status_Code--------------------------------\n");
    for (int i = 0; i < total_slaves; i++)
    {
        uint16_t al_status_code = 0;
        // 讀取 0x0134 暫存器 (AL Status Code)
        int wkc = ecx_APRD(m_slaveInfo[i].APRDAPWR_Addr, 0x0134, 2, &al_status_code, 20);

        DEBUG_PRINT("State Code 0x134>>Slave>>0x%04X>>State Code>>0x%04X\n", m_slaveInfo[i].Product_Code, al_status_code);

    }
    DEBUG_PRINT("--------------------------------\n");
}

/*
 * 將非 PDO 命令交給 PDO command channel，並等待完成或 timeout。
 * 用來避免一般執行緒直接與 4 kHz EtherCAT 交換流程競爭 NIC。
 */
bool EtherCatMaster::PDO_SendCommandAndWait(EcatCmdType type, uint16_t slave, uint16_t index, uint8_t sub, uint32_t value, int len, int timeoutMs)
{
    // 1. [Retry 機制] 如果 RT 正在忙上一件事，我們稍等一下，不要直接報錯
    int retry = 0;
    while (m_asyncCmd.status != (int)EcatCmdStatus::ECAT_STATUS_IDLE && m_asyncCmd.status != (int)EcatCmdStatus::ECAT_STATUS_DONE)
    {
        Sleep(1);
        retry++;
        if (retry > 100)
        { // 等了 100ms 還在忙，才真的放棄
            DEBUG_PRINT("[Timeout] RT Layer is busy.\n");
            return false;
        }
    }

    // 2. [填寫訂單]
    m_asyncCmd.type = (int)type;
    m_asyncCmd.slaveAddr = slave;
    m_asyncCmd.index = index;
    m_asyncCmd.subIndex = sub;
    m_asyncCmd.dataValue = value;

    // [修正點 2] 不要寫死 4，而是使用傳入的 len
    m_asyncCmd.dataSize = len;

    // 清除舊結果
    m_asyncCmd.resultWKC = 0;

    // 3. [發射] (Thread Barrier)
    //std::atomic_thread_fence(std::memory_order_release);
    m_asyncCmd.status = (int)EcatCmdStatus::ECAT_STATUS_PENDING;

    // 4. [同步等待] (這是商業軟體最關鍵的一步)
    // 我們不能射後不理，UI 會卡在這裡等結果 (Blocking Call)
    auto startTime = GetTickCount64();

    while (true)
    {
        // 檢查 RT 是否做完了 (DONE)
        if (m_asyncCmd.status == (int)EcatCmdStatus::ECAT_STATUS_DONE)
        {
            // 檢查結果：WKC > 0 才算成功
            if (m_asyncCmd.resultWKC > 0)
            {
                m_asyncCmd.status = (int)EcatCmdStatus::ECAT_STATUS_IDLE; // 重置，準備接下一單
                return true;
            }
            else
            {
                DEBUG_PRINT("[Error] Cmd Executed but WKC=0 (Slave Refused)\n");
                m_asyncCmd.status = (int)EcatCmdStatus::ECAT_STATUS_IDLE;
                return false;
            }
        }

        // 檢查超時
        if (GetTickCount64() - startTime > timeoutMs)
        {
            DEBUG_PRINT("[Timeout] Cmd Ignored by RT Layer!\n");
            m_asyncCmd.status = (int)EcatCmdStatus::ECAT_STATUS_IDLE; // 強制重置
            return false;
        }

        Sleep(1); // 讓出 CPU
    }

}




/* 計算目前拓撲預期 LRW/DC WKC，供每週期完整性檢查。 */
void EtherCatMaster::Get_TotalSlave_WKC_Count()
{
    //WKC 判斷-----------------------------------------------
    const auto& slaves = m_pEni->GetSlaves();
    int total_slaves = (int)slaves.size();
    for (int i = 0; i < total_slaves; i++)
    {
        int slaveWKC = 0;

        // 檢查是否有 Input (TxPDO)
        if (slaves[i].inputBitLength > 0)
        {
            slaveWKC += 1;
        }

        // 檢查是否有 Output (RxPDO)
        if (slaves[i].outputBitLength > 0)
        {
            slaveWKC += 2;
        }

        // 累加到總分
        EXPECTED_WKC_PDO += slaveWKC;

        // 
        DEBUG_PRINT("Slave %d: %s (In: %d bytes, Out: %d bytes) -> WKC contribution: %d\n", i, slaves[i].name, slaves[i].inputBitLength, slaves[i].outputBitLength, slaveWKC);
    }


    TotalSlave_WKC_Count = EXPECTED_WKC_PDO;
    DEBUG_PRINT("TotalSlave_WKC_Count>>%d\n", TotalSlave_WKC_Count);
}

/* 讀取各從站 DC receive-port timestamp，作為 propagation delay 計算輸入。 */
void EtherCatMaster::MeasureDCPortTimestamps()
{
    // =========================================================
    // EtherCAT DC Port Timestamp + Link Status Measurement
    //
    // Diagnostic Only
    //
    // 目的：
    //
    // 1. BWR 0x0900
    //    觸發所有 ESC 記錄 Port Receive Timestamp
    //
    // 2. 每顆 Slave 讀：
    //
    //    0x0900 ~ 0x090F
    //
    //    Port 0 = 0x0900 ~ 0x0903
    //    Port 1 = 0x0904 ~ 0x0907
    //    Port 2 = 0x0908 ~ 0x090B
    //    Port 3 = 0x090C ~ 0x090F
    //
    // 3. 每顆 Slave 再讀：
    //
    //    0x0110 ~ 0x0111
    //
    //    EtherCAT DL Status
    //
    // 4. 目前只做 Diagnostic
    //
    // 尚未：
    //
    // - 計算 Propagation Delay
    // - 寫入 0x0928
    // =========================================================


    // ---------------------------------------------------------
    // Safety
    // ---------------------------------------------------------
    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-TS] ERROR: ENI not initialized.\n");

        return;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int totalSlaves =
        (int)slaves.size();


    if (totalSlaves <= 0)
    {
        RtPrintf(
            "[DC-TS] ERROR: No EtherCAT slaves.\n");

        return;
    }


    RtPrintf(
        "\n"
        "============================================================\n"
        "[DC-TS] Starting DC Port Timestamp Measurement\n"
        "============================================================\n");


    // =========================================================
    // Step 1
    //
    // Broadcast Write 0x0900
    //
    // 觸發所有 ESC Latch Receive Timestamp
    // =========================================================

    uint32_t dcTimestampTrigger =
        0;


    int dcTimestampBwrWkc =
        ecx_BWR(
            0x0000,
            0x0900,
            4,
            &dcTimestampTrigger,
            20);


    RtPrintf(
        "[DC-TS] BWR 0x0900 WKC:%d\n",
        dcTimestampBwrWkc);


    if (dcTimestampBwrWkc <= 0)
    {
        RtPrintf(
            "[DC-TS] ERROR: Timestamp trigger failed.\n");


        RtPrintf(
            "============================================================\n"
            "[DC-TS] Timestamp Measurement FAILED\n"
            "============================================================\n\n");


        return;
    }


    // =========================================================
    // Step 2
    //
    // 每顆 Slave：
    //
    // A. Read DC Port Timestamp
    // B. Read DL Status
    // =========================================================

    for (int slaveIndex = 0;
        slaveIndex < totalSlaves;
        slaveIndex++)
    {
        // =====================================================
        // A. DC Port Receive Timestamp
        // =====================================================

        uint32_t dcPortTimestamp[4] =
        {
            0,
            0,
            0,
            0
        };


        int dcTimestampReadWkc =
            ecx_FPRD(
                m_slaveInfo[slaveIndex].configAddr,
                0x0900,
                dcPortTimestamp,
                16,
                20);


        if (dcTimestampReadWkc > 0)
        {
            // -------------------------------------------------
            // Decimal
            // -------------------------------------------------
            RtPrintf(
                "[DC-TS] Slave:%d "
                "Cfg:0x%04X "
                "WKC:%d | "
                "P0:%u "
                "P1:%u "
                "P2:%u "
                "P3:%u\n",

                slaveIndex,

                (unsigned int)
                m_slaveInfo[slaveIndex].configAddr,

                dcTimestampReadWkc,

                (unsigned int)
                dcPortTimestamp[0],

                (unsigned int)
                dcPortTimestamp[1],

                (unsigned int)
                dcPortTimestamp[2],

                (unsigned int)
                dcPortTimestamp[3]);


            // -------------------------------------------------
            // Hex
            // -------------------------------------------------
            RtPrintf(
                "[DC-TS-HEX] Slave:%d | "
                "P0:0x%08X "
                "P1:0x%08X "
                "P2:0x%08X "
                "P3:0x%08X\n",

                slaveIndex,

                (unsigned int)
                dcPortTimestamp[0],

                (unsigned int)
                dcPortTimestamp[1],

                (unsigned int)
                dcPortTimestamp[2],

                (unsigned int)
                dcPortTimestamp[3]);
        }
        else
        {
            RtPrintf(
                "[DC-TS] Slave:%d "
                "Cfg:0x%04X "
                "READ FAILED WKC:%d\n",

                slaveIndex,

                (unsigned int)
                m_slaveInfo[slaveIndex].configAddr,

                dcTimestampReadWkc);
        }


        // =====================================================
        // B. EtherCAT DL Status
        //
        // Register:
        //
        // 0x0110 ~ 0x0111
        //
        // Bit 4  = Physical Link Port 0
        // Bit 5  = Physical Link Port 1
        // Bit 6  = Physical Link Port 2
        // Bit 7  = Physical Link Port 3
        //
        // Bit 8  = Loop Port 0
        // Bit 9  = Communication Port 0
        //
        // Bit 10 = Loop Port 1
        // Bit 11 = Communication Port 1
        //
        // Bit 12 = Loop Port 2
        // Bit 13 = Communication Port 2
        //
        // Bit 14 = Loop Port 3
        // Bit 15 = Communication Port 3
        //
        // Loop:
        // 0 = Open
        // 1 = Closed
        //
        // Communication:
        // 0 = No stable communication
        // 1 = Communication established
        // =====================================================

        uint16_t dlStatus =
            0;


        int dlStatusWkc =
            ecx_FPRD(
                m_slaveInfo[slaveIndex].configAddr,
                0x0110,
                &dlStatus,
                2,
                20);


        if (dlStatusWkc > 0)
        {
            // -------------------------------------------------
            // Physical Link
            // -------------------------------------------------

            bool linkP0 =
                (dlStatus & (1U << 4)) != 0;

            bool linkP1 =
                (dlStatus & (1U << 5)) != 0;

            bool linkP2 =
                (dlStatus & (1U << 6)) != 0;

            bool linkP3 =
                (dlStatus & (1U << 7)) != 0;


            // -------------------------------------------------
            // Loop
            //
            // 0 = Open
            // 1 = Closed
            // -------------------------------------------------

            bool closedP0 =
                (dlStatus & (1U << 8)) != 0;

            bool closedP1 =
                (dlStatus & (1U << 10)) != 0;

            bool closedP2 =
                (dlStatus & (1U << 12)) != 0;

            bool closedP3 =
                (dlStatus & (1U << 14)) != 0;


            // -------------------------------------------------
            // Communication
            // -------------------------------------------------

            bool commP0 =
                (dlStatus & (1U << 9)) != 0;

            bool commP1 =
                (dlStatus & (1U << 11)) != 0;

            bool commP2 =
                (dlStatus & (1U << 13)) != 0;

            bool commP3 =
                (dlStatus & (1U << 15)) != 0;


            RtPrintf(
                "[DC-LINK] Slave:%d "
                "DL:0x%04X "
                "WKC:%d | "
                "P0[L:%d C:%d Loop:%s] "
                "P1[L:%d C:%d Loop:%s] "
                "P2[L:%d C:%d Loop:%s] "
                "P3[L:%d C:%d Loop:%s]\n",

                slaveIndex,

                (unsigned int)
                dlStatus,

                dlStatusWkc,

                linkP0 ? 1 : 0,
                commP0 ? 1 : 0,
                closedP0 ? "Closed" : "Open",

                linkP1 ? 1 : 0,
                commP1 ? 1 : 0,
                closedP1 ? "Closed" : "Open",

                linkP2 ? 1 : 0,
                commP2 ? 1 : 0,
                closedP2 ? "Closed" : "Open",

                linkP3 ? 1 : 0,
                commP3 ? 1 : 0,
                closedP3 ? "Closed" : "Open");
        }
        else
        {
            RtPrintf(
                "[DC-LINK] Slave:%d "
                "Cfg:0x%04X "
                "READ FAILED WKC:%d\n",

                slaveIndex,

                (unsigned int)
                m_slaveInfo[slaveIndex].configAddr,

                dlStatusWkc);
        }
    }


    RtPrintf(
        "============================================================\n"
        "[DC-TS] Timestamp Measurement Finished\n"
        "============================================================\n\n");
}

/*
 * 根據 DC port timestamps 與拓撲方向估算單一從站的 propagation delay。
 * 回傳 false 表示樣本無效或暫存器讀取失敗，不應套用該次結果。
 */
bool EtherCatMaster::MeasureDCPropagationDelay(
    uint32_t& delaySlave4,
    uint32_t& delaySlave5,
    uint32_t& delaySlave6)
{
    // =========================================================
    // EtherCAT DC Propagation Delay Measurement
    //
    // Current confirmed topology:
    //
    //                  +-- Slave1 -- Slave2 -- Slave3
    // Master -- Slave0 |
    //                  +-- Slave4 -- Slave5 -- Slave6
    //                      ^
    //                      |
    //                  Reference Clock
    //
    //
    // 功能：
    //
    // 1. 重複量測 10 次
    // 2. 根據 Port Timestamp 計算 propagation delay
    // 3. 保留所有有效 sample
    // 4. 使用 Median 選出穩定結果
    // 5. 將結果由 reference parameter 傳回
    //
    // 注意：
    //
    // 本函式只計算。
    // 不寫 0x0928。
    // =========================================================


    // ---------------------------------------------------------
    // Default output
    // ---------------------------------------------------------
    delaySlave4 = 0;
    delaySlave5 = 0;
    delaySlave6 = 0;


    // ---------------------------------------------------------
    // Safety
    // ---------------------------------------------------------
    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-DELAY] ERROR: ENI not initialized.\n");

        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int totalSlaves =
        (int)slaves.size();


    if (totalSlaves < 7)
    {
        RtPrintf(
            "[DC-DELAY] ERROR: Current topology requires Slave0~Slave6.\n");

        return false;
    }


    // =========================================================
    // Current topology constants
    //
    // 目前先明確綁定你的 EDM_SINKER_MODE。
    //
    // 後面再進一步 Generic 化。
    // =========================================================

    const int SLAVE_BRANCH =
        0;

    const int SLAVE_REF =
        4;

    const int SLAVE_SERVO_2 =
        5;

    const int SLAVE_SERVO_3 =
        6;


    // =========================================================
    // Step 1
    //
    // Validate topology using DL Status 0x0110
    // =========================================================

    uint16_t dlStatus0 = 0;
    uint16_t dlStatus4 = 0;
    uint16_t dlStatus5 = 0;
    uint16_t dlStatus6 = 0;


    int wkcDl0 =
        ecx_FPRD(
            m_slaveInfo[SLAVE_BRANCH].configAddr,
            0x0110,
            &dlStatus0,
            2,
            20);


    int wkcDl4 =
        ecx_FPRD(
            m_slaveInfo[SLAVE_REF].configAddr,
            0x0110,
            &dlStatus4,
            2,
            20);


    int wkcDl5 =
        ecx_FPRD(
            m_slaveInfo[SLAVE_SERVO_2].configAddr,
            0x0110,
            &dlStatus5,
            2,
            20);


    int wkcDl6 =
        ecx_FPRD(
            m_slaveInfo[SLAVE_SERVO_3].configAddr,
            0x0110,
            &dlStatus6,
            2,
            20);


    if (wkcDl0 <= 0 ||
        wkcDl4 <= 0 ||
        wkcDl5 <= 0 ||
        wkcDl6 <= 0)
    {
        RtPrintf(
            "[DC-DELAY] ERROR: Failed to read DL Status.\n");

        return false;
    }


    // =========================================================
    // Physical Link bits
    //
    // Bit4 = P0
    // Bit5 = P1
    // Bit6 = P2
    // Bit7 = P3
    // =========================================================

    bool s0p1 =
        (dlStatus0 & (1U << 5)) != 0;

    bool s0p2 =
        (dlStatus0 & (1U << 6)) != 0;


    bool s4p0 =
        (dlStatus4 & (1U << 4)) != 0;

    bool s4p1 =
        (dlStatus4 & (1U << 5)) != 0;


    bool s5p0 =
        (dlStatus5 & (1U << 4)) != 0;

    bool s5p1 =
        (dlStatus5 & (1U << 5)) != 0;


    bool s6p0 =
        (dlStatus6 & (1U << 4)) != 0;

    bool s6p1 =
        (dlStatus6 & (1U << 5)) != 0;


    // ---------------------------------------------------------
    // Expected Servo branch:
    //
    // Slave0 P2
    //    ↓
    // Slave4
    //    ↓
    // Slave5
    //    ↓
    // Slave6 End Point
    // ---------------------------------------------------------

    if (!s0p1 ||
        !s0p2 ||
        !s4p0 ||
        !s4p1 ||
        !s5p0 ||
        !s5p1 ||
        !s6p0 ||
        s6p1)
    {
        RtPrintf(
            "[DC-DELAY] ERROR: Servo branch topology mismatch.\n");

        RtPrintf(
            "[DC-DELAY] DL S0:0x%04X "
            "S4:0x%04X "
            "S5:0x%04X "
            "S6:0x%04X\n",
            (unsigned int)dlStatus0,
            (unsigned int)dlStatus4,
            (unsigned int)dlStatus5,
            (unsigned int)dlStatus6);

        return false;
    }


    RtPrintf(
        "\n"
        "============================================================\n"
        "[DC-DELAY] Starting Propagation Delay Measurement\n"
        "============================================================\n");


    // =========================================================
    // Measurement configuration
    // =========================================================

    const int MEASURE_COUNT =
        10;


    // 至少一半以上 Sample 有效才接受結果。
    const int MIN_VALID_SAMPLES =
        5;


    // =========================================================
    // Sample storage
    //
    // 使用固定陣列，避免動態 allocation。
    // =========================================================

    uint32_t sampleLink04[MEASURE_COUNT] =
    {
        0
    };

    uint32_t sampleLink45[MEASURE_COUNT] =
    {
        0
    };

    uint32_t sampleLink56[MEASURE_COUNT] =
    {
        0
    };

    uint32_t sampleDelay5[MEASURE_COUNT] =
    {
        0
    };

    uint32_t sampleDelay6[MEASURE_COUNT] =
    {
        0
    };


    int validSamples =
        0;


    // =========================================================
    // Statistics
    // =========================================================

    uint64_t sumLink04 = 0;
    uint64_t sumLink45 = 0;
    uint64_t sumLink56 = 0;

    uint64_t sumDelay5 = 0;
    uint64_t sumDelay6 = 0;


    uint32_t minLink04 =
        0xFFFFFFFFU;

    uint32_t maxLink04 =
        0;


    uint32_t minLink45 =
        0xFFFFFFFFU;

    uint32_t maxLink45 =
        0;


    uint32_t minLink56 =
        0xFFFFFFFFU;

    uint32_t maxLink56 =
        0;


    uint32_t minDelay5 =
        0xFFFFFFFFU;

    uint32_t maxDelay5 =
        0;


    uint32_t minDelay6 =
        0xFFFFFFFFU;

    uint32_t maxDelay6 =
        0;


    // =========================================================
    // Step 2
    //
    // Repeated measurements
    // =========================================================

    for (int sampleIndex = 0;
        sampleIndex < MEASURE_COUNT;
        sampleIndex++)
    {
        // -----------------------------------------------------
        // Trigger fresh DC Receive Timestamp latch
        // -----------------------------------------------------

        uint32_t trigger =
            0;


        int triggerWkc =
            ecx_BWR(
                0x0000,
                0x0900,
                4,
                &trigger,
                20);


        if (triggerWkc <= 0)
        {
            RtPrintf(
                "[DC-DELAY] Sample:%d "
                "Trigger FAILED WKC:%d\n",
                sampleIndex,
                triggerWkc);

            continue;
        }


        // =====================================================
        // Read required Port timestamps
        // =====================================================

        uint32_t ts0[4] =
        {
            0, 0, 0, 0
        };


        uint32_t ts4[4] =
        {
            0, 0, 0, 0
        };


        uint32_t ts5[4] =
        {
            0, 0, 0, 0
        };


        int wkc0 =
            ecx_FPRD(
                m_slaveInfo[SLAVE_BRANCH].configAddr,
                0x0900,
                ts0,
                16,
                20);


        int wkc4 =
            ecx_FPRD(
                m_slaveInfo[SLAVE_REF].configAddr,
                0x0900,
                ts4,
                16,
                20);


        int wkc5 =
            ecx_FPRD(
                m_slaveInfo[SLAVE_SERVO_2].configAddr,
                0x0900,
                ts5,
                16,
                20);


        if (wkc0 <= 0 ||
            wkc4 <= 0 ||
            wkc5 <= 0)
        {
            RtPrintf(
                "[DC-DELAY] Sample:%d "
                "READ FAILED "
                "S0:%d S4:%d S5:%d\n",
                sampleIndex,
                wkc0,
                wkc4,
                wkc5);

            continue;
        }


        // =====================================================
        // Step 3
        //
        // Round-trip intervals
        //
        // 只使用「同一顆 ESC」的 timestamp difference。
        //
        // uint32_t subtraction 也能自然處理
        // 32-bit timestamp wrap-around。
        // =====================================================


        // Slave0 P1 -> P2
        //
        // Servo branch total round-trip interval
        uint32_t branchRoundTrip =
            ts0[2] -
            ts0[1];


        // Slave4 P0 -> P1
        //
        // Slave4 downstream subtree
        uint32_t slave4RoundTrip =
            ts4[1] -
            ts4[0];


        // Slave5 P0 -> P1
        //
        // Slave5 downstream subtree
        uint32_t slave5RoundTrip =
            ts5[1] -
            ts5[0];


        // =====================================================
        // Sanity check
        // =====================================================

        if (branchRoundTrip <
            slave4RoundTrip)
        {
            RtPrintf(
                "[DC-DELAY] Sample:%d INVALID "
                "BranchRT:%u < S4RT:%u\n",
                sampleIndex,
                (unsigned int)branchRoundTrip,
                (unsigned int)slave4RoundTrip);

            continue;
        }


        if (slave4RoundTrip <
            slave5RoundTrip)
        {
            RtPrintf(
                "[DC-DELAY] Sample:%d INVALID "
                "S4RT:%u < S5RT:%u\n",
                sampleIndex,
                (unsigned int)slave4RoundTrip,
                (unsigned int)slave5RoundTrip);

            continue;
        }


        // =====================================================
        // Step 4
        //
        // One-way Link Delay
        // =====================================================

        uint32_t link04 =
            (branchRoundTrip -
                slave4RoundTrip)
            / 2U;


        uint32_t link45 =
            (slave4RoundTrip -
                slave5RoundTrip)
            / 2U;


        uint32_t link56 =
            slave5RoundTrip /
            2U;


        // =====================================================
        // Delay relative to Reference Slave4
        // =====================================================

        uint32_t currentDelay5 =
            link45;


        uint32_t currentDelay6 =
            link45 +
            link56;


        // =====================================================
        // Store sample
        // =====================================================

        sampleLink04[validSamples] =
            link04;

        sampleLink45[validSamples] =
            link45;

        sampleLink56[validSamples] =
            link56;

        sampleDelay5[validSamples] =
            currentDelay5;

        sampleDelay6[validSamples] =
            currentDelay6;


        // =====================================================
        // Statistics
        // =====================================================

        sumLink04 +=
            link04;

        sumLink45 +=
            link45;

        sumLink56 +=
            link56;

        sumDelay5 +=
            currentDelay5;

        sumDelay6 +=
            currentDelay6;


        if (link04 < minLink04)
            minLink04 = link04;

        if (link04 > maxLink04)
            maxLink04 = link04;


        if (link45 < minLink45)
            minLink45 = link45;

        if (link45 > maxLink45)
            maxLink45 = link45;


        if (link56 < minLink56)
            minLink56 = link56;

        if (link56 > maxLink56)
            maxLink56 = link56;


        if (currentDelay5 < minDelay5)
            minDelay5 = currentDelay5;

        if (currentDelay5 > maxDelay5)
            maxDelay5 = currentDelay5;


        if (currentDelay6 < minDelay6)
            minDelay6 = currentDelay6;

        if (currentDelay6 > maxDelay6)
            maxDelay6 = currentDelay6;


        validSamples++;


        // =====================================================
        // Log
        // =====================================================

        RtPrintf(
            "[DC-DELAY] Sample:%d | "
            "RT Branch:%u S4:%u S5:%u | "
            "Link 0-4:%u 4-5:%u 5-6:%u | "
            "RefDelay S4:0 S5:%u S6:%u ns\n",

            sampleIndex,

            (unsigned int)branchRoundTrip,
            (unsigned int)slave4RoundTrip,
            (unsigned int)slave5RoundTrip,

            (unsigned int)link04,
            (unsigned int)link45,
            (unsigned int)link56,

            (unsigned int)currentDelay5,
            (unsigned int)currentDelay6);
    }


    // =========================================================
    // Step 5
    //
    // Validate measurement quality
    // =========================================================

    if (validSamples <
        MIN_VALID_SAMPLES)
    {
        RtPrintf(
            "[DC-DELAY] ERROR: "
            "Only %d / %d valid samples.\n",
            validSamples,
            MEASURE_COUNT);


        RtPrintf(
            "============================================================\n"
            "[DC-DELAY] Measurement FAILED\n"
            "============================================================\n\n");


        return false;
    }


    // =========================================================
    // Step 6
    //
    // Average - Diagnostic only
    // =========================================================

    uint32_t avgLink04 =
        (uint32_t)(
            sumLink04 /
            (uint64_t)validSamples);


    uint32_t avgLink45 =
        (uint32_t)(
            sumLink45 /
            (uint64_t)validSamples);


    uint32_t avgLink56 =
        (uint32_t)(
            sumLink56 /
            (uint64_t)validSamples);


    uint32_t avgDelay5 =
        (uint32_t)(
            sumDelay5 /
            (uint64_t)validSamples);


    uint32_t avgDelay6 =
        (uint32_t)(
            sumDelay6 /
            (uint64_t)validSamples);


    // =========================================================
    // Step 7
    //
    // Sort samples for Median
    //
    // 只有最多 10 筆，
    // 用簡單排序即可，不需要 STL / allocation。
    // =========================================================

    for (int i = 0;
        i < validSamples - 1;
        i++)
    {
        for (int j = i + 1;
            j < validSamples;
            j++)
        {
            if (sampleDelay5[j] <
                sampleDelay5[i])
            {
                uint32_t temp =
                    sampleDelay5[i];

                sampleDelay5[i] =
                    sampleDelay5[j];

                sampleDelay5[j] =
                    temp;
            }


            if (sampleDelay6[j] <
                sampleDelay6[i])
            {
                uint32_t temp =
                    sampleDelay6[i];

                sampleDelay6[i] =
                    sampleDelay6[j];

                sampleDelay6[j] =
                    temp;
            }
        }
    }


    // =========================================================
    // Step 8
    //
    // Median
    // =========================================================

    uint32_t medianDelay5 =
        0;


    uint32_t medianDelay6 =
        0;


    if ((validSamples % 2) != 0)
    {
        // Odd number
        int middle =
            validSamples / 2;


        medianDelay5 =
            sampleDelay5[middle];


        medianDelay6 =
            sampleDelay6[middle];
    }
    else
    {
        // Even number
        int upper =
            validSamples / 2;

        int lower =
            upper - 1;


        medianDelay5 =
            (uint32_t)(
                (
                    (uint64_t)sampleDelay5[lower] +
                    (uint64_t)sampleDelay5[upper]
                    )
                / 2ULL);


        medianDelay6 =
            (uint32_t)(
                (
                    (uint64_t)sampleDelay6[lower] +
                    (uint64_t)sampleDelay6[upper]
                    )
                / 2ULL);
    }


    // =========================================================
    // Step 9
    //
    // Final selected values
    //
    // Reference Slave = 4
    // =========================================================

    delaySlave4 =
        0;


    delaySlave5 =
        medianDelay5;


    delaySlave6 =
        medianDelay6;


    // =========================================================
    // Statistics Print
    // =========================================================

    RtPrintf(
        "\n"
        "[DC-DELAY-STAT] Valid Samples:%d / %d\n",
        validSamples,
        MEASURE_COUNT);


    RtPrintf(
        "[DC-DELAY-STAT] Link S0->S4 "
        "Avg:%u Min:%u Max:%u ns\n",
        (unsigned int)avgLink04,
        (unsigned int)minLink04,
        (unsigned int)maxLink04);


    RtPrintf(
        "[DC-DELAY-STAT] Link S4->S5 "
        "Avg:%u Min:%u Max:%u ns\n",
        (unsigned int)avgLink45,
        (unsigned int)minLink45,
        (unsigned int)maxLink45);


    RtPrintf(
        "[DC-DELAY-STAT] Link S5->S6 "
        "Avg:%u Min:%u Max:%u ns\n",
        (unsigned int)avgLink56,
        (unsigned int)minLink56,
        (unsigned int)maxLink56);


    RtPrintf(
        "[DC-DELAY-STAT] Reference Slave4 | "
        "S4:0 ns | "
        "S5 Avg:%u Min:%u Max:%u | "
        "S6 Avg:%u Min:%u Max:%u ns\n",

        (unsigned int)avgDelay5,
        (unsigned int)minDelay5,
        (unsigned int)maxDelay5,

        (unsigned int)avgDelay6,
        (unsigned int)minDelay6,
        (unsigned int)maxDelay6);


    // =========================================================
    // ★ Actual result that will be written to 0x0928
    // =========================================================

    RtPrintf(
        "[DC-DELAY-RESULT] "
        "Median Selected | "
        "S4:%u ns "
        "S5:%u ns "
        "S6:%u ns\n",

        (unsigned int)delaySlave4,
        (unsigned int)delaySlave5,
        (unsigned int)delaySlave6);


    RtPrintf(
        "============================================================\n"
        "[DC-DELAY] Propagation Delay Measurement Finished\n"
        "============================================================\n\n");


    return true;
}

/* 將已驗證的 propagation delay 寫入從站 DC register。 */
bool EtherCatMaster::ConfigureDCPropagationDelay(
    uint32_t delaySlave4,
    uint32_t delaySlave5,
    uint32_t delaySlave6)
{
    // =========================================================
    // EtherCAT DC Propagation Delay Configuration
    //
    // Register:
    //
    // 0x0928 ~ 0x092B
    // System Time Delay
    //
    // Delay values are supplied by
    // MeasureDCPropagationDelay().
    //
    // 不再 Hard Code。
    // =========================================================


    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-CONFIG] ERROR: ENI not initialized.\n");

        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    if (slaves.size() < 7)
    {
        RtPrintf(
            "[DC-CONFIG] ERROR: Slave4~Slave6 unavailable.\n");

        return false;
    }


    RtPrintf(
        "\n"
        "============================================================\n"
        "[DC-CONFIG] Configure DC Propagation Delay\n"
        "============================================================\n");


    RtPrintf(
        "[DC-CONFIG] Requested | "
        "S4:%u ns "
        "S5:%u ns "
        "S6:%u ns\n",
        (unsigned int)delaySlave4,
        (unsigned int)delaySlave5,
        (unsigned int)delaySlave6);


    // =========================================================
    // Write 0x0928
    // =========================================================

    int wkc4 =
        ecx_FPWR(
            m_slaveInfo[4].configAddr,
            0x0928,
            &delaySlave4,
            4,
            20);


    int wkc5 =
        ecx_FPWR(
            m_slaveInfo[5].configAddr,
            0x0928,
            &delaySlave5,
            4,
            20);


    int wkc6 =
        ecx_FPWR(
            m_slaveInfo[6].configAddr,
            0x0928,
            &delaySlave6,
            4,
            20);


    RtPrintf(
        "[DC-CONFIG] Write 0x0928 | "
        "S4 WKC:%d | "
        "S5 WKC:%d | "
        "S6 WKC:%d\n",
        wkc4,
        wkc5,
        wkc6);


    // =========================================================
    // Write check
    // =========================================================

    if (wkc4 <= 0 ||
        wkc5 <= 0 ||
        wkc6 <= 0)
    {
        RtPrintf(
            "[DC-CONFIG] ERROR: "
            "0x0928 Write FAILED.\n");

        return false;
    }


    // =========================================================
    // ReadBack
    // =========================================================

    uint32_t readDelay4 =
        0;


    uint32_t readDelay5 =
        0;


    uint32_t readDelay6 =
        0;


    int readWkc4 =
        ecx_FPRD(
            m_slaveInfo[4].configAddr,
            0x0928,
            &readDelay4,
            4,
            20);


    int readWkc5 =
        ecx_FPRD(
            m_slaveInfo[5].configAddr,
            0x0928,
            &readDelay5,
            4,
            20);


    int readWkc6 =
        ecx_FPRD(
            m_slaveInfo[6].configAddr,
            0x0928,
            &readDelay6,
            4,
            20);


    RtPrintf(
        "[DC-CONFIG] ReadBack 0x0928 | "
        "S4:%u WKC:%d | "
        "S5:%u WKC:%d | "
        "S6:%u WKC:%d\n",

        (unsigned int)readDelay4,
        readWkc4,

        (unsigned int)readDelay5,
        readWkc5,

        (unsigned int)readDelay6,
        readWkc6);


    // =========================================================
    // Read check
    // =========================================================

    if (readWkc4 <= 0 ||
        readWkc5 <= 0 ||
        readWkc6 <= 0)
    {
        RtPrintf(
            "[DC-CONFIG] ERROR: "
            "0x0928 ReadBack FAILED.\n");

        return false;
    }


    // =========================================================
    // Value verification
    // =========================================================

    if (readDelay4 != delaySlave4 ||
        readDelay5 != delaySlave5 ||
        readDelay6 != delaySlave6)
    {
        RtPrintf(
            "[DC-CONFIG] ERROR: "
            "0x0928 Verification FAILED.\n");


        RtPrintf(
            "[DC-CONFIG] Expected "
            "S4:%u S5:%u S6:%u | "
            "Actual "
            "S4:%u S5:%u S6:%u\n",

            (unsigned int)delaySlave4,
            (unsigned int)delaySlave5,
            (unsigned int)delaySlave6,

            (unsigned int)readDelay4,
            (unsigned int)readDelay5,
            (unsigned int)readDelay6);


        return false;
    }


    RtPrintf(
        "[DC-CONFIG] "
        "Propagation Delay Configuration SUCCESS.\n");


    RtPrintf(
        "============================================================\n"
        "[DC-CONFIG] DC Propagation Delay Configuration Finished\n"
        "============================================================\n\n");


    return true;
}


namespace
{
    const int DC_AUTO_MEASURE_COUNT =
        10;

    const int DC_AUTO_MIN_VALID_SAMPLES =
        5;

    std::string ToUpperDcAutoMode(
        std::string value)
    {
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](unsigned char ch)
            {
                return (char)std::toupper(ch);
            });

        return value;
    }

    bool GetDcAutoPropagationReference(
        const int* servoSlaveIndices,
        int servoCount,
        int& referenceSlaveIndex,
        std::string& referenceMode)
    {
        referenceSlaveIndex =
            -1;

        referenceMode =
            "AUTO";

        if (servoSlaveIndices == nullptr ||
            servoCount <= 0)
        {
            return false;
        }

        const std::string configPath =
            GlobalConfig::GetInstance().BaseDataDir +
            "SystemConfig.txt";

        referenceMode =
            ToUpperDcAutoMode(
                ConfigUtil::ReadConfigString(
                    configPath,
                    "DC_Reference_Mode",
                    "AUTO"));

        if (referenceMode == "FIXED")
        {
            referenceSlaveIndex =
                (int)ConfigUtil::ReadParam(
                    configPath,
                    "DC_Reference_Slave_Index",
                    -1.0);
        }
        else
        {
            if (referenceMode != "AUTO")
            {
                RtPrintf(
                    "[DC-DELAY-AUTO] WARNING | "
                    "Unknown DC_Reference_Mode:%s | Using AUTO.\n",

                    referenceMode.c_str());

                referenceMode =
                    "AUTO";
            }

            referenceSlaveIndex =
                servoSlaveIndices[0];
        }

        // V1 的 propagation 公式以 Servo 鏈第一台為相對 delay 0。
        // FIXED 指到鏈中間時不可直接產生負 delay，因此安全拒絕、不寫 0x0928。
        return
            referenceSlaveIndex ==
            servoSlaveIndices[0];
    }

    uint32_t SelectDcAutoMedian(
        uint32_t* values,
        int count)
    {
        if (values == nullptr ||
            count <= 0)
        {
            return 0;
        }

        std::sort(
            values,
            values + count);

        if ((count % 2) != 0)
        {
            return values[count / 2];
        }

        const int upper =
            count / 2;

        const int lower =
            upper - 1;

        return
            (uint32_t)(
                (
                    (uint64_t)values[lower] +
                    (uint64_t)values[upper]
                    ) /
                2ULL);
    }

    bool RollbackDcAutoPropagationDelay(
        EtherCatMaster* pMaster,
        const DCAutoPropagationTable& delayTable,
        const uint32_t* previousDelay)
    {
        if (pMaster == nullptr ||
            previousDelay == nullptr)
        {
            return false;
        }

        bool rollbackOk =
            true;

        for (int i = 0;
            i < delayTable.count;
            i++)
        {
            const int slaveIndex =
                delayTable.entries[i].slaveIndex;

            uint32_t restoreValue =
                previousDelay[i];

            const int rollbackWkc =
                pMaster->ecx_FPWR(
                    m_slaveInfo[slaveIndex].configAddr,
                    0x0928,
                    &restoreValue,
                    4,
                    20);

            if (rollbackWkc <= 0)
            {
                rollbackOk =
                    false;
            }

            RtPrintf(
                "[DC-CONFIG-AUTO] ROLLBACK | "
                "SlaveIndex:%d | Delay:%u ns | WKC:%d\n",

                slaveIndex,
                (unsigned int)restoreValue,
                rollbackWkc);
        }

        return rollbackOk;
    }
}

/*
 * AUTO propagation-delay measurement V1。
 * 支援 1~8 台連續 Servo；非 Servo 可位於 Servo 鏈之前或之後。
 * 本函式只量測並建立 table，不寫入 0x0928。
 */
bool EtherCatMaster::MeasureDCPropagationDelayAuto(
    DCAutoPropagationTable& delayTable)
{
    delayTable =
        DCAutoPropagationTable{};

    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-DELAY-AUTO] FAILED | "
            "Reason:ENI not initialized | Write0928:NO\n");

        return false;
    }

    const auto& slaves =
        m_pEni->GetSlaves();

    const int totalSlaves =
        (int)slaves.size();

    const int servoCount =
        (int)m_ServoList.size();

    RtPrintf(
        "\n"
        "============================================================\n"
        "[DC-DELAY-AUTO] BEGIN | "
        "SlaveCount:%d | ServoCount:%d | Limit:%d | Write0928:NO\n"
        "============================================================\n",

        totalSlaves,
        servoCount,
        DC_AUTO_MAX_SERVO_COUNT);

    if (totalSlaves <= 0 ||
        servoCount <= 0 ||
        servoCount > DC_AUTO_MAX_SERVO_COUNT)
    {
        RtPrintf(
            "[DC-DELAY-AUTO] FAILED | "
            "Reason:Invalid slave or servo count | Write0928:NO\n");

        return false;
    }

    int servoSlaveIndices[DC_AUTO_MAX_SERVO_COUNT] = {};

    for (int i = 0;
        i < servoCount;
        i++)
    {
        servoSlaveIndices[i] =
            m_ServoList[(size_t)i].slaveIndex;
    }

    std::sort(
        servoSlaveIndices,
        servoSlaveIndices + servoCount);

    for (int i = 0;
        i < servoCount;
        i++)
    {
        if (servoSlaveIndices[i] < 0 ||
            servoSlaveIndices[i] >= totalSlaves)
        {
            RtPrintf(
                "[DC-DELAY-AUTO] FAILED | "
                "Reason:Servo index out of range | Order:%d | SlaveIndex:%d\n",

                i,
                servoSlaveIndices[i]);

            return false;
        }

        if (i > 0 &&
            servoSlaveIndices[i - 1] + 1 !=
            servoSlaveIndices[i])
        {
            RtPrintf(
                "[DC-DELAY-AUTO] FAILED | "
                "Reason:Servo chain is not consecutive | "
                "Previous:S%d | Current:S%d | Write0928:NO\n",

                servoSlaveIndices[i - 1],
                servoSlaveIndices[i]);

            return false;
        }
    }

    int referenceSlaveIndex =
        -1;

    std::string referenceMode;

    if (!GetDcAutoPropagationReference(
        servoSlaveIndices,
        servoCount,
        referenceSlaveIndex,
        referenceMode))
    {
        RtPrintf(
            "[DC-DELAY-AUTO] FAILED | "
            "Reason:Reference must be first Servo in V1 | "
            "Mode:%s | RequestedReference:S%d | FirstServo:S%d | "
            "Write0928:NO\n",

            referenceMode.c_str(),
            referenceSlaveIndex,
            servoSlaveIndices[0]);

        return false;
    }

    uint16_t dlStatus[DC_AUTO_MAX_SERVO_COUNT] = {};
    uint16_t physicalPortMask[DC_AUTO_MAX_SERVO_COUNT] = {};

    for (int i = 0;
        i < servoCount;
        i++)
    {
        const int slaveIndex =
            servoSlaveIndices[i];

        const uint16_t configAddr =
            m_slaveInfo[slaveIndex].configAddr;

        if (configAddr == 0)
        {
            RtPrintf(
                "[DC-DELAY-AUTO] FAILED | "
                "Reason:Config address is zero | SlaveIndex:%d\n",

                slaveIndex);

            return false;
        }

        const int dlWkc =
            ecx_FPRD(
                configAddr,
                0x0110,
                &dlStatus[i],
                2,
                20);

        uint64_t dcSystemTime =
            0;

        const int dcWkc =
            ecx_FPRD(
                configAddr,
                0x0910,
                &dcSystemTime,
                8,
                20);

        physicalPortMask[i] =
            (uint16_t)(
                (dlStatus[i] >> 4) &
                0x000FU);

        const bool p0Linked =
            (physicalPortMask[i] & 0x01U) != 0;

        const bool p1Linked =
            (physicalPortMask[i] & 0x02U) != 0;

        const bool branchOnP2OrP3 =
            (physicalPortMask[i] & 0x0CU) != 0;

        const bool isLastServo =
            i == servoCount - 1;

        const bool nodeValid =
            dlWkc > 0 &&
            dcWkc > 0 &&
            dcSystemTime > 0 &&
            p0Linked &&
            !branchOnP2OrP3 &&
            (isLastServo || p1Linked);

        RtPrintf(
            "[DC-DELAY-AUTO] NODE | "
            "Order:%d | SlaveIndex:%d | Role:%s | "
            "ConfigAddr:0x%04X | DL:0x%04X | PortMask:0x%X | "
            "DC:%llu WKC:%d | Valid:%s\n",

            i,
            slaveIndex,
            i == 0 ? "REFERENCE" : "FOLLOWER",
            (unsigned int)configAddr,
            (unsigned int)dlStatus[i],
            (unsigned int)physicalPortMask[i],
            (unsigned long long)dcSystemTime,
            dcWkc,
            nodeValid ? "YES" : "NO");

        if (!nodeValid)
        {
            RtPrintf(
                "[DC-DELAY-AUTO] FAILED | "
                "Reason:DC or linear Port validation failed | "
                "SlaveIndex:%d | Write0928:NO\n",

                slaveIndex);

            return false;
        }
    }

    uint32_t delaySamples
        [DC_AUTO_MAX_SERVO_COUNT]
        [DC_AUTO_MEASURE_COUNT] = {};

    uint64_t delaySum[DC_AUTO_MAX_SERVO_COUNT] = {};
    uint32_t delayMin[DC_AUTO_MAX_SERVO_COUNT] = {};
    uint32_t delayMax[DC_AUTO_MAX_SERVO_COUNT] = {};

    for (int i = 0;
        i < servoCount;
        i++)
    {
        delayMin[i] =
            0xFFFFFFFFU;
    }

    int validSamples =
        0;

    for (int sampleIndex = 0;
        sampleIndex < DC_AUTO_MEASURE_COUNT;
        sampleIndex++)
    {
        uint32_t trigger =
            0;

        const int triggerWkc =
            ecx_BWR(
                0x0000,
                0x0900,
                4,
                &trigger,
                20);

        if (triggerWkc <= 0)
        {
            RtPrintf(
                "[DC-DELAY-AUTO] Sample:%d REJECT | "
                "Reason:Timestamp trigger failed | WKC:%d\n",

                sampleIndex,
                triggerWkc);

            continue;
        }

        uint32_t timestamps
            [DC_AUTO_MAX_SERVO_COUNT]
            [4] = {};

        bool sampleValid =
            true;

        for (int i = 0;
            i < servoCount;
            i++)
        {
            const int slaveIndex =
                servoSlaveIndices[i];

            const int timestampWkc =
                ecx_FPRD(
                    m_slaveInfo[slaveIndex].configAddr,
                    0x0900,
                    timestamps[i],
                    16,
                    20);

            if (timestampWkc <= 0)
            {
                RtPrintf(
                    "[DC-DELAY-AUTO] Sample:%d REJECT | "
                    "Reason:Timestamp read failed | SlaveIndex:%d | WKC:%d\n",

                    sampleIndex,
                    slaveIndex,
                    timestampWkc);

                sampleValid =
                    false;

                break;
            }
        }

        if (!sampleValid)
        {
            continue;
        }

        uint32_t downstreamRoundTrip[DC_AUTO_MAX_SERVO_COUNT] = {};
        uint32_t currentDelay[DC_AUTO_MAX_SERVO_COUNT] = {};

        for (int i = 0;
            i < servoCount;
            i++)
        {
            const bool p1Linked =
                (physicalPortMask[i] & 0x02U) != 0;

            if (p1Linked)
            {
                downstreamRoundTrip[i] =
                    timestamps[i][1] -
                    timestamps[i][0];

                if (downstreamRoundTrip[i] == 0)
                {
                    sampleValid =
                        false;

                    break;
                }
            }
        }

        currentDelay[0] =
            0;

        for (int i = 0;
            sampleValid &&
            i < servoCount - 1;
            i++)
        {
            if (downstreamRoundTrip[i] <=
                downstreamRoundTrip[i + 1])
            {
                sampleValid =
                    false;

                break;
            }

            const uint32_t linkDelay =
                (
                    downstreamRoundTrip[i] -
                    downstreamRoundTrip[i + 1]
                    ) /
                2U;

            const uint64_t cumulativeDelay =
                (uint64_t)currentDelay[i] +
                (uint64_t)linkDelay;

            if (linkDelay == 0 ||
                cumulativeDelay > 0xFFFFFFFFULL)
            {
                sampleValid =
                    false;

                break;
            }

            currentDelay[i + 1] =
                (uint32_t)cumulativeDelay;
        }

        if (!sampleValid)
        {
            RtPrintf(
                "[DC-DELAY-AUTO] Sample:%d REJECT | "
                "Reason:Round-trip hierarchy invalid\n",

                sampleIndex);

            continue;
        }

        RtPrintf(
            "[DC-DELAY-AUTO] Sample:%d VALID | ",
            sampleIndex);

        for (int i = 0;
            i < servoCount;
            i++)
        {
            delaySamples[i][validSamples] =
                currentDelay[i];

            delaySum[i] +=
                currentDelay[i];

            if (currentDelay[i] < delayMin[i])
            {
                delayMin[i] =
                    currentDelay[i];
            }

            if (currentDelay[i] > delayMax[i])
            {
                delayMax[i] =
                    currentDelay[i];
            }

            RtPrintf(
                "%sS%d:%u",
                i == 0 ? "" : " ",
                servoSlaveIndices[i],
                (unsigned int)currentDelay[i]);
        }

        RtPrintf(" ns\n");

        validSamples++;
    }

    if (validSamples <
        DC_AUTO_MIN_VALID_SAMPLES)
    {
        RtPrintf(
            "[DC-DELAY-AUTO] FAILED | "
            "Reason:Insufficient valid samples | Valid:%d/%d | "
            "Required:%d | Write0928:NO\n",

            validSamples,
            DC_AUTO_MEASURE_COUNT,
            DC_AUTO_MIN_VALID_SAMPLES);

        return false;
    }

    delayTable.count =
        servoCount;

    delayTable.referenceSlaveIndex =
        referenceSlaveIndex;

    for (int i = 0;
        i < servoCount;
        i++)
    {
        uint32_t sortedDelay[DC_AUTO_MEASURE_COUNT] = {};

        for (int sample = 0;
            sample < validSamples;
            sample++)
        {
            sortedDelay[sample] =
                delaySamples[i][sample];
        }

        const uint32_t medianDelay =
            SelectDcAutoMedian(
                sortedDelay,
                validSamples);

        const uint32_t averageDelay =
            (uint32_t)(
                delaySum[i] /
                (uint64_t)validSamples);

        delayTable.entries[i].slaveIndex =
            servoSlaveIndices[i];

        delayTable.entries[i].delayNs =
            medianDelay;

        RtPrintf(
            "[DC-DELAY-AUTO-STAT] "
            "Order:%d | SlaveIndex:%d | "
            "Median:%u Avg:%u Min:%u Max:%u ns | Samples:%d\n",

            i,
            servoSlaveIndices[i],
            (unsigned int)medianDelay,
            (unsigned int)averageDelay,
            (unsigned int)delayMin[i],
            (unsigned int)delayMax[i],
            validSamples);
    }

    RtPrintf(
        "[DC-DELAY-AUTO-RESULT] "
        "Reference:S%d | ServoCount:%d | ",

        delayTable.referenceSlaveIndex,
        delayTable.count);

    for (int i = 0;
        i < delayTable.count;
        i++)
    {
        RtPrintf(
            "%sS%d:%u",
            i == 0 ? "" : " ",
            delayTable.entries[i].slaveIndex,
            (unsigned int)delayTable.entries[i].delayNs);
    }

    RtPrintf(
        " ns | ValidSamples:%d/%d | Result:PASS | Write0928:NO\n",
        validSamples,
        DC_AUTO_MEASURE_COUNT);

    RtPrintf(
        "============================================================\n"
        "[DC-DELAY-AUTO] END | Result:PASS | Write0928:NO\n"
        "============================================================\n\n");

    return true;
}

/* 將 AUTO table 寫入各 Servo 0x0928，逐站 ReadBack；失敗時回復原值。 */
bool EtherCatMaster::ConfigureDCPropagationDelayAuto(
    const DCAutoPropagationTable& delayTable)
{
    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-CONFIG-AUTO] FAILED | Reason:ENI not initialized\n");

        return false;
    }

    const auto& slaves =
        m_pEni->GetSlaves();

    const int totalSlaves =
        (int)slaves.size();

    if (delayTable.count <= 0 ||
        delayTable.count > DC_AUTO_MAX_SERVO_COUNT ||
        delayTable.count != (int)m_ServoList.size() ||
        delayTable.referenceSlaveIndex !=
        delayTable.entries[0].slaveIndex ||
        delayTable.entries[0].delayNs != 0)
    {
        RtPrintf(
            "[DC-CONFIG-AUTO] FAILED | Reason:Invalid delay table header\n");

        return false;
    }

    int currentServoIndices[DC_AUTO_MAX_SERVO_COUNT] = {};

    for (int i = 0;
        i < delayTable.count;
        i++)
    {
        currentServoIndices[i] =
            m_ServoList[(size_t)i].slaveIndex;
    }

    std::sort(
        currentServoIndices,
        currentServoIndices + delayTable.count);

    for (int i = 0;
        i < delayTable.count;
        i++)
    {
        const int slaveIndex =
            delayTable.entries[i].slaveIndex;

        if (slaveIndex < 0 ||
            slaveIndex >= totalSlaves ||
            m_slaveInfo[slaveIndex].configAddr == 0 ||
            currentServoIndices[i] != slaveIndex)
        {
            RtPrintf(
                "[DC-CONFIG-AUTO] FAILED | "
                "Reason:Servo order changed or invalid | Order:%d | SlaveIndex:%d\n",

                i,
                slaveIndex);

            return false;
        }

        if (i > 0 &&
            (
                delayTable.entries[i - 1].slaveIndex + 1 !=
                slaveIndex ||
                delayTable.entries[i].delayNs <
                delayTable.entries[i - 1].delayNs
                ))
        {
            RtPrintf(
                "[DC-CONFIG-AUTO] FAILED | "
                "Reason:Non-consecutive order or decreasing delay | Order:%d\n",

                i);

            return false;
        }
    }

    RtPrintf(
        "\n"
        "============================================================\n"
        "[DC-CONFIG-AUTO] BEGIN | Reference:S%d | ServoCount:%d\n"
        "============================================================\n",

        delayTable.referenceSlaveIndex,
        delayTable.count);

    uint32_t previousDelay[DC_AUTO_MAX_SERVO_COUNT] = {};

    for (int i = 0;
        i < delayTable.count;
        i++)
    {
        const int slaveIndex =
            delayTable.entries[i].slaveIndex;

        const int preflightWkc =
            ecx_FPRD(
                m_slaveInfo[slaveIndex].configAddr,
                0x0928,
                &previousDelay[i],
                4,
                20);

        RtPrintf(
            "[DC-CONFIG-AUTO] PREFLIGHT | "
            "Order:%d | SlaveIndex:%d | Previous:%u ns | WKC:%d\n",

            i,
            slaveIndex,
            (unsigned int)previousDelay[i],
            preflightWkc);

        if (preflightWkc <= 0)
        {
            RtPrintf(
                "[DC-CONFIG-AUTO] FAILED | "
                "Reason:Preflight read failed | No value was written\n");

            return false;
        }
    }

    for (int i = 0;
        i < delayTable.count;
        i++)
    {
        const int slaveIndex =
            delayTable.entries[i].slaveIndex;

        uint32_t requestedDelay =
            delayTable.entries[i].delayNs;

        const int writeWkc =
            ecx_FPWR(
                m_slaveInfo[slaveIndex].configAddr,
                0x0928,
                &requestedDelay,
                4,
                20);

        RtPrintf(
            "[DC-CONFIG-AUTO] WRITE | "
            "Order:%d | SlaveIndex:%d | Delay:%u ns | WKC:%d\n",

            i,
            slaveIndex,
            (unsigned int)requestedDelay,
            writeWkc);

        if (writeWkc <= 0)
        {
            const bool rollbackOk =
                RollbackDcAutoPropagationDelay(
                    this,
                    delayTable,
                    previousDelay);

            RtPrintf(
                "[DC-CONFIG-AUTO] FAILED | "
                "Reason:Write failed | Rollback:%s\n",

                rollbackOk ? "SUCCESS" : "FAILED");

            return false;
        }
    }

    bool verificationOk =
        true;

    for (int i = 0;
        i < delayTable.count;
        i++)
    {
        const int slaveIndex =
            delayTable.entries[i].slaveIndex;

        uint32_t readBackDelay =
            0;

        const int readBackWkc =
            ecx_FPRD(
                m_slaveInfo[slaveIndex].configAddr,
                0x0928,
                &readBackDelay,
                4,
                20);

        const bool valueMatch =
            readBackWkc > 0 &&
            readBackDelay ==
            delayTable.entries[i].delayNs;

        RtPrintf(
            "[DC-CONFIG-AUTO] READBACK | "
            "Order:%d | SlaveIndex:%d | Expected:%u | Actual:%u ns | "
            "WKC:%d | Match:%s\n",

            i,
            slaveIndex,
            (unsigned int)delayTable.entries[i].delayNs,
            (unsigned int)readBackDelay,
            readBackWkc,
            valueMatch ? "YES" : "NO");

        if (!valueMatch)
        {
            verificationOk =
                false;
        }
    }

    if (!verificationOk)
    {
        const bool rollbackOk =
            RollbackDcAutoPropagationDelay(
                this,
                delayTable,
                previousDelay);

        RtPrintf(
            "[DC-CONFIG-AUTO] FAILED | "
            "Reason:ReadBack verification failed | Rollback:%s\n",

            rollbackOk ? "SUCCESS" : "FAILED");

        return false;
    }

    RtPrintf(
        "[DC-CONFIG-AUTO-RESULT] "
        "Reference:S%d | ServoCount:%d | Result:PASS\n",

        delayTable.referenceSlaveIndex,
        delayTable.count);

    RtPrintf(
        "============================================================\n"
        "[DC-CONFIG-AUTO] END | Result:PASS\n"
        "============================================================\n\n");

    return true;
}


// =============================================================
// DC Diagnostic Snapshot V1 - Producer Side
//
// 用途：
// 1. UpdateDCMasterClockEstimator() 保留 estimator 計算，但完全不 RtPrintf。
// 2. UpdateDCPdoPhaseController() 改成 Monitor Only。
// 3. 不修改 PDO Timer。
// 4. 不修改 RTX64 HAL。
// 5. 不修改 Sync0。
// 6. 不修改 EtherCAT Slave DC。
// 7. Priority-64 只寫 Snapshot，不輸出文字。
// =============================================================


// =============================================================
// DC PLL Diagnostic Snapshot
//
// Definition 只放一份。
// 下一步會在 Main Thread .cpp 使用 extern 讀取。
// =============================================================

volatile LONG
g_dcPllDiagSequence =
0;

volatile LONGLONG
g_dcPllDiagEstimatorSequence =
0;

volatile LONGLONG
g_dcPllDiagEstimatorOffsetNs =
0;

volatile LONGLONG
g_dcPllDiagDriftPpb =
0;

volatile LONGLONG
g_dcPllDiagPdoPhaseNs =
0;

volatile LONGLONG
g_dcPllDiagTargetPhaseNs =
0;

volatile LONGLONG
g_dcPllDiagPhaseStepNs =
0;

volatile LONGLONG
g_dcPllDiagWrappedErrorNs =
0;

volatile LONGLONG
g_dcPllDiagUnwrappedErrorNs =
0;

volatile LONGLONG
g_dcPllDiagSync0MarginNs =
0;

volatile LONG
g_dcPllDiagTargetCaptured =
0;

volatile LONG
g_dcPllDiagStableWindows =
0;
volatile LONGLONG
g_dcPllDiagPCommandNs =
0;

volatile LONGLONG
g_dcPllDiagPeriodCorrectionPs =
0;

// =============================================================
// Master <-> EtherCAT DC Clock Estimator
//
// ★ Priority-64 safe version
//
// - NO RtPrintf
// - NO Timer modification
// - NO HAL modification
// - NO Sync0 modification
// - NO Slave DC modification
// =============================================================

/*
 * 更新 Master QPC 與 EtherCAT reference DC time 的 offset/drift estimator。
 * 此層提供時基估測，不直接執行 Motion/NC 命令。
 */
void EtherCatMaster::UpdateDCMasterClockEstimator(
    uint64_t masterBeforeNs,
    uint64_t masterAfterNs,
    uint64_t dcReferenceNs,
    int dcWkc)
{
    const uint32_t WINDOW_SAMPLES =
        4000U;

    const uint64_t MAX_RTT_NS =
        500000ULL;


    // =========================================================
    // Local Estimator State
    // =========================================================

    static uint32_t validSamples =
        0;

    static uint32_t rejectedSamples =
        0;

    static int64_t offsetSum =
        0;

    static uint64_t rttSum =
        0;

    static uint64_t windowStartMasterNs =
        0;

    static uint64_t windowEndMasterNs =
        0;


    // =========================================================
    // Previous Window
    // =========================================================

    static bool hasPreviousWindow =
        false;

    static int64_t previousAverageOffsetNs =
        0;

    static uint64_t previousWindowCenterNs =
        0;


    // =========================================================
    // Step 1 - Basic Validation
    // =========================================================

    if (dcWkc <= 0)
    {
        rejectedSamples++;

        return;
    }


    if (dcReferenceNs == 0)
    {
        rejectedSamples++;

        return;
    }


    if (masterAfterNs <
        masterBeforeNs)
    {
        rejectedSamples++;

        return;
    }


    // =========================================================
    // Step 2 - Combined LRW + FRMW Round Trip Time
    // =========================================================

    uint64_t rttNs =
        masterAfterNs -
        masterBeforeNs;


    if (rttNs >
        MAX_RTT_NS)
    {
        rejectedSamples++;

        return;
    }


    // =========================================================
    // Step 3 - Estimate CLOCK_2 time at DC capture
    //
    // Absolute offset contains deterministic frame/capture bias.
    // Slope is what we use for frequency drift.
    // =========================================================

    uint64_t masterMidpointNs =
        masterBeforeNs +
        (rttNs / 2ULL);


    // Offset definition:
    //
    //     CLOCK_2 - DC Reference
    //
    int64_t offsetNs =
        (int64_t)
        masterMidpointNs -
        (int64_t)
        dcReferenceNs;


    // =========================================================
    // Step 4 - Accumulate Window
    // =========================================================

    if (validSamples == 0)
    {
        windowStartMasterNs =
            masterMidpointNs;
    }


    windowEndMasterNs =
        masterMidpointNs;


    offsetSum +=
        offsetNs;


    rttSum +=
        rttNs;


    validSamples++;


    if (validSamples <
        WINDOW_SAMPLES)
    {
        return;
    }


    // =========================================================
    // Step 5 - Window Average
    // =========================================================

    int64_t averageOffsetNs =
        offsetSum /
        (int64_t)
        validSamples;


    uint64_t averageRttNs =
        rttSum /
        (uint64_t)
        validSamples;


    uint64_t windowCenterNs =
        windowStartMasterNs +
        (
            (
                windowEndMasterNs -
                windowStartMasterNs
                )
            / 2ULL
            );


    // =========================================================
    // Publish latest estimator reference point
    // =========================================================

    m_dcEstimatorOffsetNs =
        averageOffsetNs;


    m_dcEstimatorMasterTimeNs =
        windowCenterNs;


    // averageRttNs 目前只保留計算，避免 unused warning。
    // Main Thread 已經有 PDO-COMBINED snapshot 可觀察 RTT。
    (void)
        averageRttNs;


    // =========================================================
    // Step 6 - Frequency Drift
    //
    // Drift =
    //
    //     deltaOffset / elapsed
    //
    // ppb:
    //
    //     + = CLOCK_2 faster than DC
    //     - = CLOCK_2 slower than DC
    // =========================================================

    if (hasPreviousWindow &&
        windowCenterNs >
        previousWindowCenterNs)
    {
        int64_t deltaOffsetNs =
            averageOffsetNs -
            previousAverageOffsetNs;


        uint64_t elapsedNs =
            windowCenterNs -
            previousWindowCenterNs;


        int64_t driftPpb =
            (
                deltaOffsetNs *
                1000000000LL
                )
            /
            (int64_t)
            elapsedNs;


        m_dcEstimatorDriftPpb =
            driftPpb;


        m_dcEstimatorValid =
            true;


        m_dcEstimatorSequence++;
    }


    // =========================================================
    // Step 7 - Save Current Window
    // =========================================================

    previousAverageOffsetNs =
        averageOffsetNs;


    previousWindowCenterNs =
        windowCenterNs;


    hasPreviousWindow =
        true;


    // =========================================================
    // Step 8 - Reset Window
    // =========================================================

    validSamples =
        0;


    rejectedSamples =
        0;


    offsetSum =
        0;


    rttSum =
        0;


    windowStartMasterNs =
        0;


    windowEndMasterNs =
        0;
}


// =============================================================
// DC PDO Phase Monitor V1 - Snapshot Version
//
// ★ MONITOR ONLY
//
// - NO RtPrintf
// - NO RtSetTimer
// - NO RtSetTimerRelative
// - NO RtSetHalTimerPeriodCounts
// - NO HAL calibration / dither preparation
//
// 目的：
// 1. 使用已修正的 uint64 modulo phase algorithm。
// 2. 等兩個有效 estimator drift window 後自動 Capture target。
// 3. 觀察 Phase / Step / ErrWrap / ErrUnwrap。
// 4. 將結果寫入 snapshot，下一步由 Priority-50 Main Thread 印。
// =============================================================

/*
 * 更新 PDO send phase 相對於 Sync0 的監測與控制資料。
 * 計算結果透過 seqlock snapshot 交給 Priority 50 診斷輸出。
 */
void EtherCatMaster::UpdateDCPdoPhaseController(
    uint64_t pdoStartMasterNs)
{
    if (!m_dcEstimatorValid)
    {
        return;
    }


    // =========================================================
    // 每個新的 Estimator Window 只執行一次
    // =========================================================

    static uint64_t lastEstimatorSequence =
        0;


    if (m_dcEstimatorSequence ==
        lastEstimatorSequence)
    {
        return;
    }


    lastEstimatorSequence =
        m_dcEstimatorSequence;


    // =========================================================
    // Constants
    // =========================================================

    const int64_t DC_CYCLE_NS =
        250000LL;

    const int64_t SYNC0_PHASE_NS =
        125000LL;

    const int64_t DC_PLL_MONITOR_MAX_DRIFT_PPB =
        100000LL;


    // =========================================================
    // Estimate current CLOCK_2 - DC offset
    // =========================================================

    int64_t deltaMasterNs =
        0;


    if (pdoStartMasterNs >=
        m_dcEstimatorMasterTimeNs)
    {
        deltaMasterNs =
            (int64_t)
            (
                pdoStartMasterNs -
                m_dcEstimatorMasterTimeNs
                );
    }
    else
    {
        deltaMasterNs =
            -(int64_t)
            (
                m_dcEstimatorMasterTimeNs -
                pdoStartMasterNs
                );
    }


    int64_t offsetDriftNs =
        (
            deltaMasterNs *
            m_dcEstimatorDriftPpb
            )
        /
        1000000000LL;


    int64_t estimatedOffsetNowNs =
        m_dcEstimatorOffsetNs +
        offsetDriftNs;


    // =========================================================
    // DC Phase = Master Phase - Offset Phase
    //
    // ★ IMPORTANT
    //
    // pdoStartMasterNs 是 uint64_t absolute time。
    // 禁止先 cast 成 int64_t。
    //
    // 先在 uint64_t 世界做 modulo，避免 absolute-time overflow。
    // =========================================================

    uint64_t masterPhaseNs =
        pdoStartMasterNs %
        (uint64_t)
        DC_CYCLE_NS;


    int64_t offsetPhaseNs =
        estimatedOffsetNowNs %
        DC_CYCLE_NS;


    int64_t pdoPhaseNs =
        (int64_t)
        masterPhaseNs -
        offsetPhaseNs;


    pdoPhaseNs %=
        DC_CYCLE_NS;


    if (pdoPhaseNs < 0)
    {
        pdoPhaseNs +=
            DC_CYCLE_NS;
    }


    // =========================================================
    // Monitor State
    // =========================================================

    static bool targetCaptured =
        false;

    static uint32_t stableWindows =
        0;

    static int64_t targetPhaseNs =
        0;

    static int64_t previousPhaseNs =
        0;

    static int64_t unwrappedErrorNs =
        0;


    bool capturedThisWindow =
        false;


    // =========================================================
    // Capture target after two sane estimator windows
    // =========================================================

    if (!targetCaptured)
    {
        if (m_dcEstimatorDriftPpb != 0 &&
            m_dcEstimatorDriftPpb >=
            -DC_PLL_MONITOR_MAX_DRIFT_PPB &&
            m_dcEstimatorDriftPpb <=
            DC_PLL_MONITOR_MAX_DRIFT_PPB)
        {
            stableWindows++;


            if (stableWindows >=
                2U)
            {
                targetPhaseNs =
                    pdoPhaseNs;


                previousPhaseNs =
                    pdoPhaseNs;


                unwrappedErrorNs =
                    0;


                targetCaptured =
                    true;


                capturedThisWindow =
                    true;
            }
        }
    }


    // =========================================================
    // Phase Monitor
    // =========================================================

    int64_t phaseStepNs =
        0;

    int64_t wrappedPhaseErrorNs =
        0;


    if (targetCaptured)
    {
        if (!capturedThisWindow)
        {
            phaseStepNs =
                pdoPhaseNs -
                previousPhaseNs;


            if (phaseStepNs >
                (DC_CYCLE_NS / 2))
            {
                phaseStepNs -=
                    DC_CYCLE_NS;
            }
            else if (phaseStepNs <
                -(DC_CYCLE_NS / 2))
            {
                phaseStepNs +=
                    DC_CYCLE_NS;
            }


            unwrappedErrorNs +=
                phaseStepNs;


            previousPhaseNs =
                pdoPhaseNs;
        }


        wrappedPhaseErrorNs =
            pdoPhaseNs -
            targetPhaseNs;


        while (wrappedPhaseErrorNs >
            (DC_CYCLE_NS / 2))
        {
            wrappedPhaseErrorNs -=
                DC_CYCLE_NS;
        }


        while (wrappedPhaseErrorNs <
            -(DC_CYCLE_NS / 2))
        {
            wrappedPhaseErrorNs +=
                DC_CYCLE_NS;
        }
    }

    // =============================================================
// PDO PLL Actuator V1A
//
// DRY RUN ONLY
//
// 只計算 Command。
// 不修改任何 Timer / HAL。
// =============================================================

    const int64_t PDO_PLL_P_DIVISOR =
        8LL;

    const int64_t PDO_PLL_MAX_P_COMMAND_NS =
        5000LL;


    // -------------------------------------------------------------
    // P Command
    //
    // ErrWrap > 0
    //     PDO phase 比 Target 往正方向漂
    //
    // Command < 0
    //     預期 scheduler 往 EARLIER 修正
    // -------------------------------------------------------------

    int64_t pCommandNs =
        0;


    if (targetCaptured)
    {
        pCommandNs =
            -(
                wrappedPhaseErrorNs /
                PDO_PLL_P_DIVISOR
                );


        if (pCommandNs >
            PDO_PLL_MAX_P_COMMAND_NS)
        {
            pCommandNs =
                PDO_PLL_MAX_P_COMMAND_NS;
        }
        else if (pCommandNs <
            -PDO_PLL_MAX_P_COMMAND_NS)
        {
            pCommandNs =
                -PDO_PLL_MAX_P_COMMAND_NS;
        }
    }


    // -------------------------------------------------------------
    // Frequency Feed-Forward Diagnostic
    //
    // Drift:
    //     ppb
    //
    // Result:
    //     ps / PDO cycle
    //
    // 250000 ns × -8400 ppb
    // ≈ -2100 ps / cycle
    // -------------------------------------------------------------

    int64_t periodCorrectionPs =
        (
            DC_CYCLE_NS *
            m_dcEstimatorDriftPpb
            )
        /
        1000000LL;
    // =========================================================
    // Sync0 margin diagnostic
    // =========================================================

    int64_t sync0MarginNs =
        SYNC0_PHASE_NS -
        pdoPhaseNs;


    if (sync0MarginNs < 0)
    {
        sync0MarginNs +=
            DC_CYCLE_NS;
    }


    // =========================================================
    // Publish DC PLL Diagnostic Snapshot
    //
    // odd  = writer updating
    // even = complete
    // =========================================================

    InterlockedIncrement(
        &g_dcPllDiagSequence);


    g_dcPllDiagEstimatorSequence =
        (LONGLONG)
        m_dcEstimatorSequence;


    g_dcPllDiagEstimatorOffsetNs =
        (LONGLONG)
        estimatedOffsetNowNs;


    g_dcPllDiagDriftPpb =
        (LONGLONG)
        m_dcEstimatorDriftPpb;


    g_dcPllDiagPdoPhaseNs =
        (LONGLONG)
        pdoPhaseNs;


    g_dcPllDiagTargetPhaseNs =
        (LONGLONG)
        targetPhaseNs;


    g_dcPllDiagPhaseStepNs =
        (LONGLONG)
        phaseStepNs;


    g_dcPllDiagWrappedErrorNs =
        (LONGLONG)
        wrappedPhaseErrorNs;


    g_dcPllDiagUnwrappedErrorNs =
        (LONGLONG)
        unwrappedErrorNs;


    g_dcPllDiagSync0MarginNs =
        (LONGLONG)
        sync0MarginNs;


    // =============================================================
    // PDO PLL V1A Dry Run Command
    // =============================================================

    g_dcPllDiagPCommandNs =
        (LONGLONG)
        pCommandNs;


    g_dcPllDiagPeriodCorrectionPs =
        (LONGLONG)
        periodCorrectionPs;


    // =============================================================
    // Monitor State
    // =============================================================

    g_dcPllDiagTargetCaptured =
        targetCaptured
        ? 1L
        : 0L;


    g_dcPllDiagStableWindows =
        (LONG)
        stableWindows;


    MemoryBarrier();


    InterlockedIncrement(
        &g_dcPllDiagSequence);

    g_dcPllDiagStableWindows =
        (LONG)
        stableWindows;


    MemoryBarrier();


    InterlockedIncrement(
        &g_dcPllDiagSequence);


    // =========================================================
    // CONTROL OFF
    //
    // 這裡故意結束。
    //
    // 不再進入舊的：
    // - HAL calibration
    // - residual trim
    // - dither preparation
    // - HAL raw diagnostic
    //
    // 真正的 PDO-only PLL actuator 會在 Monitor 驗證後另做。
    // =========================================================

    return;
}










// =============================================================
// EtherCAT Send Point Diagnostic Snapshot
//
// Producer：ecx_LRW_FRMW() / Priority 64。
// Consumer：PrintDcRuntimeDiagnostics1000ms() / Priority 50。
// Publish：每 4000 個有效 QPC 樣本一次。
//
// Build：從開始建立 LRW+FRMW Frame 到呼叫 SendPacket() 前的時間。
// SendCall：SendPacket() API 本身的執行時間；不代表線上傳輸完成時間。
// Sequence 採 seqlock：publish 前後各遞增一次，讀取端只接受 even snapshot。
// =============================================================

volatile LONG
    g_ecatSendDiagSequence =
    0;

volatile LONGLONG
    g_ecatSendBuildAvgNs =
    0;

volatile LONGLONG
    g_ecatSendBuildMinNs =
    0;

volatile LONGLONG
    g_ecatSendBuildMaxNs =
    0;

volatile LONGLONG
    g_ecatSendCallAvgNs =
    0;

volatile LONGLONG
    g_ecatSendCallMinNs =
    0;

volatile LONGLONG
    g_ecatSendCallMaxNs =
    0;

volatile LONG
    g_ecatSendQpcValid =
    0;


// =============================================================
// EtherCAT RX Soft/Hard Deadline Diagnostic Snapshot
//
// Producer: ecx_LRW_FRMW() / Priority 64
// Consumer: System main reader / Priority 50
//
// SoftLateAccepted：205 us 後、210 us 前收到且驗證成功的 Frame。
// HardTimeout：本 4000-cycle 視窗內超過 210 us 的次數。
// TotalHardTimeout：程序啟動後累積，永不因 snapshot publish 歸零。
// PostReceiveLate：ReceivePacket() 返回時才發現已跨越 Hard Deadline。
// RecoveryAfterTimeout：Timeout 後下一次有效 Frame 恢復的事件數。
//
// 本區只宣告診斷快照，不修改 HAL、Timer、DC、Motion 或 NC 控制。
// Priority 64 路徑禁止 RtPrintf。
// =============================================================

volatile LONG g_ecatRxDiagSequence = 0;
volatile LONG g_ecatRxDiagCalls = 0;
volatile LONG g_ecatRxDiagFirstRxSuccess = 0;
volatile LONG g_ecatRxDiagEmptyRx = 0;
volatile LONG g_ecatRxDiagInvalidFrame = 0;
volatile LONG g_ecatRxDiagSoftLateAccepted = 0;
volatile LONGLONG g_ecatRxDiagTotalSoftLateAccepted = 0;
volatile LONGLONG g_ecatRxDiagSoftLateElapsedMaxNs = 0;
volatile LONG g_ecatRxDiagHardTimeout = 0;
volatile LONG g_ecatRxDiagPostReceiveLate = 0;
volatile LONG g_ecatRxDiagCurrentConsecutiveTimeout = 0;
volatile LONG g_ecatRxDiagMaxConsecutiveTimeout = 0;
volatile LONGLONG g_ecatRxDiagTotalHardTimeout = 0;
volatile LONG g_ecatRxDiagRecoveryAfterTimeout = 0;
volatile LONG g_ecatRxDiagQpcFail = 0;
volatile LONG g_ecatRxDiagSleepCount = 0;
volatile LONG g_ecatRxDiagElapsedValid = 0;
volatile LONGLONG g_ecatRxDiagElapsedAvgNs = 0;
volatile LONGLONG g_ecatRxDiagElapsedMaxNs = 0;
volatile LONG g_ecatRxDiagTimeoutPreReceive = 0;
volatile LONG g_ecatRxDiagTimeoutSleep0 = 0;
volatile LONG g_ecatRxDiagTimeoutSleep1 = 0;
volatile LONG g_ecatRxDiagTimeoutSleep2 = 0;
volatile LONG g_ecatRxDiagTimeoutAttemptAvg = 0;
volatile LONG g_ecatRxDiagTimeoutAttemptMax = 0;
volatile LONGLONG g_ecatRxDiagReceiveCallMaxNs = 0;
volatile LONGLONG g_ecatRxDiagTimeoutReceiveCallMaxNs = 0;
volatile LONG g_ecatRxDiagSoftDeadlineNs = 205000;
volatile LONG g_ecatRxDiagHardDeadlineNs = 210000;


/*
 * 4 kHz PDO 核心交換函式。
 *
 * 單一 Ethernet Frame 內包含：
 * - Datagram #1：LRW，交換 Output/Input process image。
 * - Datagram #2：FRMW，讀取 reference slave System Time 0x0910。
 *
 * 成功時：
 * - data 更新為 Slave 回傳的 Input PDO。
 * - dcReferenceTime 更新為 64-bit DC System Time。
 * - dcWkc 更新為 FRMW WKC。
 * - 回傳 LRW WKC。
 *
 * 失敗時：
 * - 回傳 -1。
 * - Hard Timeout 會設定 rxResyncPending，下一週期先清除殘留 RX Frame。
 *
 * 即時限制：
 * - 不可在此函式加入 RtPrintf、檔案 I/O 或非固定時間的長等待。
 * - 診斷採固定大小統計視窗與 seqlock snapshot。
 */
int EtherCatMaster::ecx_LRW_FRMW(
    uint32_t LogAddr,
    uint16_t length,
    void* data,
    uint16_t dcSlaveAddr,
    uint64_t* dcReferenceTime,
    int* dcWkc,
    int timeout)
{
    // =====================================================
    // 0. Parameter Check
    // =====================================================

    if (m_pNic == nullptr)
    {
        return -1;
    }


    if (data == nullptr)
    {
        return -1;
    }


    if (dcReferenceTime == nullptr)
    {
        return -1;
    }


    if (dcWkc == nullptr)
    {
        return -1;
    }


    // 預設 FRMW WKC = 0
    *dcWkc = 0;


    // =============================================================
    // EtherCAT Send Point Diagnostic
    //
    // QPC frequency 只初始化一次。
    // 本段只量測 frame build 與 SendPacket() 呼叫時間，沒有等待、
    // 沒有修改 timer，也不會改變實際送出時點。
    // =============================================================

    static bool sendQpcInitAttempted =
        false;

    static bool sendQpcValid =
        false;

    static uint64_t sendQpcFrequency =
        0;


    if (!sendQpcInitAttempted)
    {
        LARGE_INTEGER frequency = {};

        if (RtQueryPerformanceFrequency(
            &frequency) &&
            frequency.QuadPart > 0)
        {
            sendQpcFrequency =
                (uint64_t)
                frequency.QuadPart;

            sendQpcValid =
                true;
        }

        sendQpcInitAttempted =
            true;
    }


    LARGE_INTEGER qpcBuildStart = {};

    bool qpcBuildStartValid =
        false;


    if (sendQpcValid)
    {
        if (RtQueryPerformanceCounter(
            &qpcBuildStart))
        {
            qpcBuildStartValid =
                true;
        }
    }


    // =====================================================
    // Frame Layout
    //
    // Ethernet Header     14
    // EtherCAT Header      2
    //
    // Datagram #1 LRW:
    //     Header           10
    //     Data             length
    //     WKC               2
    //
    // Datagram #2 FRMW:
    //     Header           10
    //     Data               8
    //     WKC               2
    //
    // Total:
    //
    //     14 + 2
    //     + 10 + length + 2
    //     + 10 + 8 + 2
    //
    //   = 48 + length
    //
    // =====================================================

    const int DC_DATA_LENGTH =
        8;


    const int totalFrameLength =
        48 +
        (int)length;


    // Ethernet 最大 Frame Buffer
    if (totalFrameLength >
        1518)
    {
        return -1;
    }


    uint8_t* frame =
        m_txBuffer;


    // =====================================================
    // Clear TX Frame
    // =====================================================

    memset(
        frame,
        0,
        totalFrameLength);


    // =====================================================
    // 1. Ethernet Header
    // =====================================================

    uint8_t destMac[6] =
    {
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF
    };


    uint8_t srcMac[6];


    m_pNic->
        GetMacAddress(
            srcMac);


    memcpy(
        &frame[0],
        destMac,
        6);


    memcpy(
        &frame[6],
        srcMac,
        6);


    // EtherCAT EtherType = 0x88A4
    frame[12] =
        0x88;

    frame[13] =
        0xA4;


    // =====================================================
    // 2. EtherCAT Header
    //
    // EtherCAT Length：
    //
    // Datagram1:
    //      10 + length + 2
    //
    // Datagram2:
    //      10 + 8 + 2
    //
    // =====================================================

    uint16_t etherCatDataLength =
        (uint16_t)(
            (10 + length + 2) +
            (10 + DC_DATA_LENGTH + 2));


    // Type = 0x1
    //
    // bits 0..10 = Length
    // bit 12..15 = Type
    uint16_t etherCatHeader =
        (etherCatDataLength &
         0x07FF)
        |
        0x1000;


    frame[14] =
        (uint8_t)(
            etherCatHeader &
            0xFF);


    frame[15] =
        (uint8_t)(
            (etherCatHeader >>
             8)
            &
            0xFF);


    // =====================================================
    // Datagram Index
    //
    // 每個 Datagram 使用不同 Idx。
    // =====================================================

    uint8_t lrwIdx =
        m_idx++;


    uint8_t frmwIdx =
        m_idx++;


    // =====================================================
    // 3. Datagram #1
    //
    // LRW
    //
    // Offset:
    //
    // 16 = Cmd
    // 17 = Idx
    // 18 = Logical Address
    // 22 = Len
    // 24 = IRQ
    // 26 = Data
    // =====================================================

    int lrwOffset =
        16;


    // -----------------------------------------------------
    // Command
    //
    // 0x0C = LRW
    // -----------------------------------------------------

    frame[
        lrwOffset + 0] =
        0x0C;


    // -----------------------------------------------------
    // Index
    // -----------------------------------------------------

    frame[
        lrwOffset + 1] =
        lrwIdx;


    // -----------------------------------------------------
    // Logical Address
    // -----------------------------------------------------

    frame[
        lrwOffset + 2] =
        (uint8_t)(
            LogAddr &
            0xFF);


    frame[
        lrwOffset + 3] =
        (uint8_t)(
            (LogAddr >>
             8)
            &
            0xFF);


    frame[
        lrwOffset + 4] =
        (uint8_t)(
            (LogAddr >>
             16)
            &
            0xFF);


    frame[
        lrwOffset + 5] =
        (uint8_t)(
            (LogAddr >>
             24)
            &
            0xFF);


    // =====================================================
    // LRW Length Field
    //
    // Bit 0..10:
    //     Data Length
    //
    // M Bit:
    //     1 = 還有下一個 Datagram
    //
    // 因為後面還有 FRMW：
    //
    //     M = 1
    //
    // =====================================================

    uint16_t lrwLengthField =
        (length &
         0x07FF)
        |
        0x8000;


    frame[
        lrwOffset + 6] =
        (uint8_t)(
            lrwLengthField &
            0xFF);


    frame[
        lrwOffset + 7] =
        (uint8_t)(
            (lrwLengthField >>
             8)
            &
            0xFF);


    // -----------------------------------------------------
    // IRQ
    // -----------------------------------------------------

    frame[
        lrwOffset + 8] =
        0x00;


    frame[
        lrwOffset + 9] =
        0x00;


    // =====================================================
    // LRW Data
    //
    // Output Process Image
    // =====================================================

    int lrwDataOffset =
        lrwOffset +
        10;


    memcpy(
        &frame[
            lrwDataOffset],
        data,
        length);


    // =====================================================
    // LRW WKC
    //
    // TX 必須初始化 0。
    // =====================================================

    int lrwWkcOffset =
        lrwDataOffset +
        length;


    frame[
        lrwWkcOffset] =
        0x00;


    frame[
        lrwWkcOffset + 1] =
        0x00;


    // =====================================================
    // 4. Datagram #2
    //
    // FRMW
    //
    // Datagram #2 緊接在
    // Datagram #1 WKC 後面。
    // =====================================================

    int frmwOffset =
        lrwWkcOffset +
        2;


    // -----------------------------------------------------
    // Command
    //
    // 0x0E = FRMW
    // -----------------------------------------------------

    frame[
        frmwOffset + 0] =
        0x0E;


    // -----------------------------------------------------
    // Index
    // -----------------------------------------------------

    frame[
        frmwOffset + 1] =
        frmwIdx;


    // -----------------------------------------------------
    // Configured Station Address
    // -----------------------------------------------------

    frame[
        frmwOffset + 2] =
        (uint8_t)(
            dcSlaveAddr &
            0xFF);


    frame[
        frmwOffset + 3] =
        (uint8_t)(
            (dcSlaveAddr >>
             8)
            &
            0xFF);


    // -----------------------------------------------------
    // Register Address
    //
    // 0x0910 = System Time
    // -----------------------------------------------------

    const uint16_t dcRegister =
        0x0910;


    frame[
        frmwOffset + 4] =
        (uint8_t)(
            dcRegister &
            0xFF);


    frame[
        frmwOffset + 5] =
        (uint8_t)(
            (dcRegister >>
             8)
            &
            0xFF);


    // =====================================================
    // FRMW Length Field
    //
    // Length = 8
    //
    // M = 0
    //
    // 因為 FRMW 是最後一個 Datagram。
    // =====================================================

    uint16_t frmwLengthField =
        (uint16_t)
        DC_DATA_LENGTH;


    frame[
        frmwOffset + 6] =
        (uint8_t)(
            frmwLengthField &
            0xFF);


    frame[
        frmwOffset + 7] =
        (uint8_t)(
            (frmwLengthField >>
             8)
            &
            0xFF);


    // -----------------------------------------------------
    // IRQ
    // -----------------------------------------------------

    frame[
        frmwOffset + 8] =
        0x00;


    frame[
        frmwOffset + 9] =
        0x00;


    // =====================================================
    // FRMW Data
    //
    // TX 初始必須 0。
    //
    // Reference Slave 讀取 0x0910 後，
    // 會把 System Time 放進這 8 bytes。
    // =====================================================

    int frmwDataOffset =
        frmwOffset +
        10;


    memset(
        &frame[
            frmwDataOffset],
        0,
        DC_DATA_LENGTH);


    // =====================================================
    // FRMW WKC
    // =====================================================

    int frmwWkcOffset =
        frmwDataOffset +
        DC_DATA_LENGTH;


    frame[
        frmwWkcOffset] =
        0x00;


    frame[
        frmwWkcOffset + 1] =
        0x00;


    // =====================================================
    // Sanity Check
    //
    // 最後應該剛好：
    //
    // frmwWkcOffset + 2
    //
    // = totalFrameLength
    // =====================================================

    int calculatedFrameLength =
        frmwWkcOffset +
        2;


    if (calculatedFrameLength !=
        totalFrameLength)
    {
        return -1;
    }


    // =====================================================
    // 5. Send
    // =====================================================

    // 上一週期 Hard Timeout 後，RX Queue 可能殘留過期 Frame。
    // 下一次送出前最多 drain 8 筆，避免舊 Index 被誤判為本週期資料。
    static bool rxResyncPending =
        false;


    if (rxResyncPending)
    {
        const int RX_RESYNC_DRAIN_LIMIT =
            8;


        for (int drainIndex = 0;
             drainIndex < RX_RESYNC_DRAIN_LIMIT;
             drainIndex++)
        {
            int staleLength =
                (int)m_pNic->
                ReceivePacket(
                    m_rxBuffer);


            if (staleLength <= 0)
            {
                break;
            }
        }


        rxResyncPending =
            false;
    }

    // =============================================================
    // Exact software send point timing
    // =============================================================

    LARGE_INTEGER qpcBeforeSend = {};
    LARGE_INTEGER qpcAfterSend = {};

    bool qpcBeforeSendValid =
        false;

    bool qpcAfterSendValid =
        false;


    if (qpcBuildStartValid)
    {
        if (RtQueryPerformanceCounter(
            &qpcBeforeSend))
        {
            if (qpcBeforeSend.QuadPart >=
                qpcBuildStart.QuadPart)
            {
                qpcBeforeSendValid =
                    true;
            }
        }
    }


    // =============================================================
    // Actual NIC Send API
    // =============================================================

    m_pNic->
        SendPacket(
            frame,
            totalFrameLength);


    if (qpcBeforeSendValid)
    {
        if (RtQueryPerformanceCounter(
            &qpcAfterSend))
        {
            if (qpcAfterSend.QuadPart >=
                qpcBeforeSend.QuadPart)
            {
                qpcAfterSendValid =
                    true;
            }
        }
    }


    // =============================================================
    // Send Point Statistics
    // =============================================================

    static uint64_t buildSumNs =
        0;

    static uint64_t buildMinNs =
        0;

    static uint64_t buildMaxNs =
        0;

    static uint64_t sendCallSumNs =
        0;

    static uint64_t sendCallMinNs =
        0;

    static uint64_t sendCallMaxNs =
        0;

    static uint32_t sendDiagSamples =
        0;


    if (qpcBuildStartValid &&
        qpcBeforeSendValid &&
        qpcAfterSendValid &&
        sendQpcFrequency > 0)
    {
        uint64_t buildCounts =
            (uint64_t)
            (
                qpcBeforeSend.QuadPart -
                qpcBuildStart.QuadPart
            );


        uint64_t sendCallCounts =
            (uint64_t)
            (
                qpcAfterSend.QuadPart -
                qpcBeforeSend.QuadPart
            );


        uint64_t buildNs =
            (
                buildCounts *
                1000000000ULL
            )
            /
            sendQpcFrequency;


        uint64_t sendCallNs =
            (
                sendCallCounts *
                1000000000ULL
            )
            /
            sendQpcFrequency;


        if (sendDiagSamples == 0)
        {
            buildMinNs =
                buildNs;

            buildMaxNs =
                buildNs;

            sendCallMinNs =
                sendCallNs;

            sendCallMaxNs =
                sendCallNs;
        }


        if (buildNs <
            buildMinNs)
        {
            buildMinNs =
                buildNs;
        }


        if (buildNs >
            buildMaxNs)
        {
            buildMaxNs =
                buildNs;
        }


        if (sendCallNs <
            sendCallMinNs)
        {
            sendCallMinNs =
                sendCallNs;
        }


        if (sendCallNs >
            sendCallMaxNs)
        {
            sendCallMaxNs =
                sendCallNs;
        }


        buildSumNs +=
            buildNs;

        sendCallSumNs +=
            sendCallNs;

        sendDiagSamples++;


        if (sendDiagSamples >=
            4000U)
        {
            InterlockedIncrement(
                &g_ecatSendDiagSequence);


            g_ecatSendBuildAvgNs =
                (LONGLONG)
                (
                    buildSumNs /
                    sendDiagSamples
                );

            g_ecatSendBuildMinNs =
                (LONGLONG)
                buildMinNs;

            g_ecatSendBuildMaxNs =
                (LONGLONG)
                buildMaxNs;


            g_ecatSendCallAvgNs =
                (LONGLONG)
                (
                    sendCallSumNs /
                    sendDiagSamples
                );

            g_ecatSendCallMinNs =
                (LONGLONG)
                sendCallMinNs;

            g_ecatSendCallMaxNs =
                (LONGLONG)
                sendCallMaxNs;


            g_ecatSendQpcValid =
                1L;


            MemoryBarrier();


            InterlockedIncrement(
                &g_ecatSendDiagSequence);


            buildSumNs =
                0;

            buildMinNs =
                0;

            buildMaxNs =
                0;


            sendCallSumNs =
                0;

            sendCallMinNs =
                0;

            sendCallMaxNs =
                0;


            sendDiagSamples =
                0;
        }
    }


    // =============================================================
    // RX Deadline 診斷視窗
    //
    // 每 4000 次完成的 RX phase 發布一次，約等於 4 kHz 下的一秒。
    // *Window 欄位在 publish 後歸零；Total 欄位持續累積。
    // 這些計數只用於診斷，不會直接觸發伺服控制動作。
    // =============================================================

    static uint64_t rxDiagCallsWindow = 0;
    static uint64_t rxDiagFirstRxSuccessWindow = 0;
    static uint64_t rxDiagEmptyRxWindow = 0;
    static uint64_t rxDiagInvalidFrameWindow = 0;
    static uint64_t rxDiagSoftLateAcceptedWindow = 0;
    static uint64_t rxDiagTotalSoftLateAccepted = 0;
    static uint64_t rxDiagSoftLateElapsedMaxNsWindow = 0;
    static uint64_t rxDiagHardTimeoutWindow = 0;
    static uint64_t rxDiagPostReceiveLateWindow = 0;

    static uint64_t rxDiagCurrentConsecutiveTimeout = 0;
    static uint64_t rxDiagMaxConsecutiveTimeoutWindow = 0;
    static uint64_t rxDiagTotalHardTimeout = 0;
    static uint64_t rxDiagRecoveryAfterTimeoutWindow = 0;

    static uint64_t rxDiagQpcFailWindow = 0;
    static uint64_t rxDiagSleepCountWindow = 0;
    static uint64_t rxDiagElapsedValidWindow = 0;
    static uint64_t rxDiagElapsedSumNsWindow = 0;
    static uint64_t rxDiagElapsedMaxNsWindow = 0;
    static uint64_t rxDiagTimeoutPreReceiveWindow = 0;
    static uint64_t rxDiagTimeoutSleep0Window = 0;
    static uint64_t rxDiagTimeoutSleep1Window = 0;
    static uint64_t rxDiagTimeoutSleep2Window = 0;
    static uint64_t rxDiagTimeoutAttemptSumWindow = 0;
    static uint64_t rxDiagTimeoutAttemptMaxWindow = 0;
    static uint64_t rxDiagReceiveCallMaxNsWindow = 0;
    static uint64_t rxDiagTimeoutReceiveCallMaxNsWindow = 0;


    // =============================================================
    // 6. Receive - RX Soft/Hard Deadline
    //
    // 舊版本使用 timeout * 100 次重試，可能把一次封包遺失擴張成
    // 毫秒級阻塞，對 250 us PDO cycle 不可接受。
    //
    // RC1 行為：
    // - Soft Deadline = 205 us：晚於此時間但早於 Hard 的有效 Frame
    //   仍可接受，並累加 SoftLateAccepted。
    // - Hard Deadline = 210 us：到達或超過此時間立即失敗，Frame 不採用。
    // - 每次 RX phase 最多執行兩次 50 us coarse wait request。
    // - 第二次 wait 後使用 bounded polling，直到 Frame 或 Hard Deadline。
    // - QPC 無效時採固定 4 次 ReceivePacket() fallback，避免無界等待。
    //
    // RTX64 HAL 目前設定 25 us；RX_COARSE_SLEEP_NS 是要求等待 50 us，
    // 兩者用途不同，不應把此常數改成 HAL 值。
    // timeout 參數只為維持既有函式介面，不再控制 RX 等待長度。
    // =============================================================

    (void)timeout;


    // RX Deadline 調整規則：
    // - 必須保持 Soft < Hard < PDO cycle(250000 ns)，並保留 Handler 後段運算時間。
    // - Soft 調大：SoftLate 較少，但只是把警戒線往後移，不會改善真正 RX 延遲。
    // - Hard 調大：HardTimeout 可能下降，但會接受更舊的 frame，且壓縮 Motion/DC 時間。
    // - Hard 調小：更快判定失敗，但偶發 NAL／interrupt 延遲更容易被算 Timeout。
    // - 建議一次只移動 5000 ns，先看 RxElapsed Max、PostReceiveLate 與 PDO-EXEC Max。
    // - Soft 與 Hard 建議至少保留 5000 ns 間隔，讓 SoftLate 能成為提前警報。
    // - RX_COARSE_SLEEP_NS 不是 deadline；它是 coarse wait request，不要跟 HAL 值混用。
    //
    // 調整前後都必須重新執行長時間激磁三軸同動測試；正式基準先保持 205/210 us。
    const uint64_t RX_SOFT_DEADLINE_NS =
        205000ULL;

    const uint64_t RX_HARD_DEADLINE_NS =
        210000ULL;

    const uint64_t RX_COARSE_SLEEP_NS =
        50000ULL;       // 50 us


    LARGE_INTEGER wait;

    wait.QuadPart =
        500;            // 50 us in 100 ns units


    LARGE_INTEGER rxStartQpc =
        {};

    bool rxDeadlineValid =
        false;

    LONGLONG rxSoftDeadlineQpc =
        0;

    LONGLONG rxHardDeadlineQpc =
        0;

    uint64_t rxSleepCounts =
        0;


    if (sendQpcValid &&
        sendQpcFrequency > 0 &&
        RtQueryPerformanceCounter(
            &rxStartQpc))
    {
        uint64_t rxSoftDeadlineCounts =
            (
                RX_SOFT_DEADLINE_NS *
                sendQpcFrequency
            )
            /
            1000000000ULL;


        uint64_t rxHardDeadlineCounts =
            (
                RX_HARD_DEADLINE_NS *
                sendQpcFrequency
            )
            /
            1000000000ULL;


        rxSleepCounts =
            (
                RX_COARSE_SLEEP_NS *
                sendQpcFrequency
            )
            /
            1000000000ULL;


        if (rxSoftDeadlineCounts > 0 &&
            rxHardDeadlineCounts >
            rxSoftDeadlineCounts &&
            rxSleepCounts > 0)
        {
            rxSoftDeadlineQpc =
                rxStartQpc.QuadPart +
                (LONGLONG)
                rxSoftDeadlineCounts;

            rxHardDeadlineQpc =
                rxStartQpc.QuadPart +
                (LONGLONG)
                rxHardDeadlineCounts;

            rxDeadlineValid =
                true;
        }
    }


    // =============================================================
    // V1A diagnostic: one call entered the receive phase.
    // =============================================================

    rxDiagCallsWindow++;


    if (!rxDeadlineValid)
    {
        // Existing bounded fallback behavior remains unchanged.
        rxDiagQpcFailWindow++;
    }


    uint32_t rxAttemptNumber =
        0;

    uint32_t rxCoarseSleepCount =
        0U;


    // 集中記錄一次 Hard Timeout，避免不同 timeout 出口漏掉欄位。
    auto MarkRxHardTimeoutDiagnostic =
        [&](bool preReceiveDeadline,
            uint64_t timeoutReceiveCallNs)
    {
        rxResyncPending =
            true;

        rxDiagHardTimeoutWindow++;
        rxDiagTotalHardTimeout++;
        rxDiagCurrentConsecutiveTimeout++;


        if (preReceiveDeadline)
        {
            rxDiagTimeoutPreReceiveWindow++;
        }


        if (rxCoarseSleepCount == 0U)
        {
            rxDiagTimeoutSleep0Window++;
        }
        else if (rxCoarseSleepCount == 1U)
        {
            rxDiagTimeoutSleep1Window++;
        }
        else
        {
            rxDiagTimeoutSleep2Window++;
        }


        rxDiagTimeoutAttemptSumWindow +=
            rxAttemptNumber;


        if (rxAttemptNumber >
            rxDiagTimeoutAttemptMaxWindow)
        {
            rxDiagTimeoutAttemptMaxWindow =
                rxAttemptNumber;
        }


        if (timeoutReceiveCallNs >
            rxDiagTimeoutReceiveCallMaxNsWindow)
        {
            rxDiagTimeoutReceiveCallMaxNsWindow =
                timeoutReceiveCallNs;
        }


        if (rxDiagCurrentConsecutiveTimeout >
            rxDiagMaxConsecutiveTimeoutWindow)
        {
            rxDiagMaxConsecutiveTimeoutWindow =
                rxDiagCurrentConsecutiveTimeout;
        }
    };


    // 完成本週期統計；達到 4000 calls 時以 seqlock 發布一秒快照。
    auto FinalizeRxDiagnostic =
        [&]()
    {
        // Receive-phase elapsed time.
        if (rxDeadlineValid &&
            sendQpcFrequency > 0)
        {
            LARGE_INTEGER rxEndQpc =
                {};


            if (RtQueryPerformanceCounter(
                &rxEndQpc) &&
                rxEndQpc.QuadPart >=
                rxStartQpc.QuadPart)
            {
                uint64_t rxElapsedCounts =
                    (uint64_t)
                    (
                        rxEndQpc.QuadPart -
                        rxStartQpc.QuadPart
                    );


                uint64_t rxElapsedNs =
                    (
                        rxElapsedCounts *
                        1000000000ULL
                    )
                    /
                    sendQpcFrequency;


                rxDiagElapsedSumNsWindow +=
                    rxElapsedNs;

                rxDiagElapsedValidWindow++;


                if (rxElapsedNs >
                    rxDiagElapsedMaxNsWindow)
                {
                    rxDiagElapsedMaxNsWindow =
                        rxElapsedNs;
                }
            }
            else
            {
                rxDiagQpcFailWindow++;
            }
        }


        // Publish every 4000 completed RX calls.
        if (rxDiagCallsWindow >=
            4000ULL)
        {
            uint64_t rxElapsedAvgNs =
                rxDiagElapsedValidWindow > 0
                ?
                (
                    rxDiagElapsedSumNsWindow /
                    rxDiagElapsedValidWindow
                )
                :
                0ULL;

            uint64_t rxTimeoutAttemptAvg =
                rxDiagHardTimeoutWindow > 0
                ?
                (
                    rxDiagTimeoutAttemptSumWindow /
                    rxDiagHardTimeoutWindow
                )
                :
                0ULL;


            InterlockedIncrement(
                &g_ecatRxDiagSequence);


            g_ecatRxDiagCalls =
                (LONG)rxDiagCallsWindow;

            g_ecatRxDiagFirstRxSuccess =
                (LONG)rxDiagFirstRxSuccessWindow;

            g_ecatRxDiagEmptyRx =
                (LONG)rxDiagEmptyRxWindow;

            g_ecatRxDiagInvalidFrame =
                (LONG)rxDiagInvalidFrameWindow;

            g_ecatRxDiagSoftLateAccepted =
                (LONG)rxDiagSoftLateAcceptedWindow;

            g_ecatRxDiagTotalSoftLateAccepted =
                (LONGLONG)rxDiagTotalSoftLateAccepted;

            g_ecatRxDiagSoftLateElapsedMaxNs =
                (LONGLONG)rxDiagSoftLateElapsedMaxNsWindow;

            g_ecatRxDiagHardTimeout =
                (LONG)rxDiagHardTimeoutWindow;

            g_ecatRxDiagPostReceiveLate =
                (LONG)rxDiagPostReceiveLateWindow;

            g_ecatRxDiagCurrentConsecutiveTimeout =
                (LONG)rxDiagCurrentConsecutiveTimeout;

            g_ecatRxDiagMaxConsecutiveTimeout =
                (LONG)rxDiagMaxConsecutiveTimeoutWindow;

            g_ecatRxDiagTotalHardTimeout =
                (LONGLONG)rxDiagTotalHardTimeout;

            g_ecatRxDiagRecoveryAfterTimeout =
                (LONG)rxDiagRecoveryAfterTimeoutWindow;

            g_ecatRxDiagQpcFail =
                (LONG)rxDiagQpcFailWindow;

            g_ecatRxDiagSleepCount =
                (LONG)rxDiagSleepCountWindow;

            g_ecatRxDiagElapsedValid =
                (LONG)rxDiagElapsedValidWindow;

            g_ecatRxDiagElapsedAvgNs =
                (LONGLONG)rxElapsedAvgNs;

            g_ecatRxDiagElapsedMaxNs =
                (LONGLONG)rxDiagElapsedMaxNsWindow;

            g_ecatRxDiagTimeoutPreReceive =
                (LONG)rxDiagTimeoutPreReceiveWindow;

            g_ecatRxDiagTimeoutSleep0 =
                (LONG)rxDiagTimeoutSleep0Window;

            g_ecatRxDiagTimeoutSleep1 =
                (LONG)rxDiagTimeoutSleep1Window;

            g_ecatRxDiagTimeoutSleep2 =
                (LONG)rxDiagTimeoutSleep2Window;

            g_ecatRxDiagTimeoutAttemptAvg =
                (LONG)rxTimeoutAttemptAvg;

            g_ecatRxDiagTimeoutAttemptMax =
                (LONG)rxDiagTimeoutAttemptMaxWindow;

            g_ecatRxDiagReceiveCallMaxNs =
                (LONGLONG)rxDiagReceiveCallMaxNsWindow;

            g_ecatRxDiagTimeoutReceiveCallMaxNs =
                (LONGLONG)rxDiagTimeoutReceiveCallMaxNsWindow;

            g_ecatRxDiagSoftDeadlineNs =
                (LONG)RX_SOFT_DEADLINE_NS;

            g_ecatRxDiagHardDeadlineNs =
                (LONG)RX_HARD_DEADLINE_NS;


            MemoryBarrier();


            InterlockedIncrement(
                &g_ecatRxDiagSequence);


            rxDiagCallsWindow = 0;
            rxDiagFirstRxSuccessWindow = 0;
            rxDiagEmptyRxWindow = 0;
            rxDiagInvalidFrameWindow = 0;
            rxDiagSoftLateAcceptedWindow = 0;
            rxDiagSoftLateElapsedMaxNsWindow = 0;
            rxDiagHardTimeoutWindow = 0;
            rxDiagPostReceiveLateWindow = 0;
            rxDiagMaxConsecutiveTimeoutWindow = 0;
            rxDiagRecoveryAfterTimeoutWindow = 0;
            rxDiagQpcFailWindow = 0;
            rxDiagSleepCountWindow = 0;
            rxDiagElapsedValidWindow = 0;
            rxDiagElapsedSumNsWindow = 0;
            rxDiagElapsedMaxNsWindow = 0;
            rxDiagTimeoutPreReceiveWindow = 0;
            rxDiagTimeoutSleep0Window = 0;
            rxDiagTimeoutSleep1Window = 0;
            rxDiagTimeoutSleep2Window = 0;
            rxDiagTimeoutAttemptSumWindow = 0;
            rxDiagTimeoutAttemptMaxWindow = 0;
            rxDiagReceiveCallMaxNsWindow = 0;
            rxDiagTimeoutReceiveCallMaxNsWindow = 0;
        }
    };


    // 本機正常情況必須有有效 QPC。fallback 只防止 QPC 異常時回到
    // 舊版數千次重試的阻塞行為，不是正常 RX 等待策略。
    int fallbackReceiveAttempts =
        4;

    while (true)
    {
        LONGLONG rxAttemptStartQpc =
            0;

        bool rxAttemptStartQpcValid =
            false;

        bool rxAttemptSoftLate =
            false;

        uint64_t rxAttemptElapsedNs =
            0ULL;


        // 每次呼叫 ReceivePacket() 前先檢查 Hard Deadline，避免明知逾時
        // 仍進入可能耗時的 driver call。
        if (rxDeadlineValid)
        {
            LARGE_INTEGER rxNowQpc =
                {};


            if (!RtQueryPerformanceCounter(
                &rxNowQpc))
            {
                rxDiagQpcFailWindow++;
                MarkRxHardTimeoutDiagnostic(
                    false,
                    0ULL);

                FinalizeRxDiagnostic();

                return -1;
            }


            if (rxNowQpc.QuadPart >=
                rxHardDeadlineQpc)
            {
                MarkRxHardTimeoutDiagnostic(
                    true,
                    0ULL);

                FinalizeRxDiagnostic();

                return -1;
            }


            rxAttemptStartQpc =
                rxNowQpc.QuadPart;

            rxAttemptStartQpcValid =
                true;
        }
        else
        {
            if (fallbackReceiveAttempts-- <=
                0)
            {
                MarkRxHardTimeoutDiagnostic(
                    true,
                    0ULL);

                FinalizeRxDiagnostic();

                return -1;
            }
        }


        rxAttemptNumber++;


        int rxLen =
            m_pNic->
            ReceivePacket(
                m_rxBuffer);


        // =============================================================
        // ReceivePacket() 後的 Hard Deadline 強制檢查
        //
        // 只在呼叫前檢查仍不夠，因為 driver call 本身可能偶發延遲。
        // 因此 ReceivePacket() 返回後立刻讀 QPC：
        // - 未超時：繼續驗證 EtherType、Datagram Index 與 WKC。
        // - 已超時：即使 Frame 內容正確也丟棄，記錄 PostReceiveLate。
        //
        // 此檢查不修改 HAL 或 Timer。
        // =============================================================

        if (rxDeadlineValid)
        {
            LARGE_INTEGER rxAfterReceiveQpc =
                {};

            uint64_t rxReceiveCallNs =
                0ULL;


            if (!RtQueryPerformanceCounter(
                &rxAfterReceiveQpc))
            {
                rxDiagQpcFailWindow++;
                MarkRxHardTimeoutDiagnostic(
                    false,
                    0ULL);

                FinalizeRxDiagnostic();

                return -1;
            }


            if (rxAttemptStartQpcValid &&
                rxAfterReceiveQpc.QuadPart >=
                rxAttemptStartQpc)
            {
                uint64_t rxReceiveCallCounts =
                    (uint64_t)
                    (
                        rxAfterReceiveQpc.QuadPart -
                        rxAttemptStartQpc
                    );


                rxReceiveCallNs =
                    (
                        rxReceiveCallCounts *
                        1000000000ULL
                    )
                    /
                    sendQpcFrequency;


                if (rxReceiveCallNs >
                    rxDiagReceiveCallMaxNsWindow)
                {
                    rxDiagReceiveCallMaxNsWindow =
                        rxReceiveCallNs;
                }
            }


            if (rxAfterReceiveQpc.QuadPart >=
                rxStartQpc.QuadPart)
            {
                uint64_t rxAttemptElapsedCounts =
                    (uint64_t)
                    (
                        rxAfterReceiveQpc.QuadPart -
                        rxStartQpc.QuadPart
                    );


                rxAttemptElapsedNs =
                    (
                        rxAttemptElapsedCounts *
                        1000000000ULL
                    )
                    /
                    sendQpcFrequency;


                rxAttemptSoftLate =
                    rxAfterReceiveQpc.QuadPart >=
                    rxSoftDeadlineQpc;
            }


            if (rxAfterReceiveQpc.QuadPart >=
                rxHardDeadlineQpc)
            {
                // Count the actual ReceivePacket result as well.
                if (rxLen <= 0)
                {
                    rxDiagEmptyRxWindow++;
                }


                rxDiagPostReceiveLateWindow++;
                MarkRxHardTimeoutDiagnostic(
                    false,
                    rxReceiveCallNs);

                // IMPORTANT:
                // Even if rxLen contains a valid matching EtherCAT
                // frame, it is intentionally NOT accepted because it
                // arrived back to software after the hard deadline.
                FinalizeRxDiagnostic();

                return -1;
            }
        }


        if (rxLen <= 0)
        {
            rxDiagEmptyRxWindow++;

            if (rxCoarseSleepCount <
                2U)
            {
                if (rxDeadlineValid)
                {
                    LARGE_INTEGER rxBeforeSleepQpc =
                        {};


                    if (!RtQueryPerformanceCounter(
                        &rxBeforeSleepQpc))
                    {
                        rxDiagQpcFailWindow++;
                        MarkRxHardTimeoutDiagnostic(
                            false,
                            0ULL);

                        FinalizeRxDiagnostic();

                        return -1;
                    }


                    if (rxBeforeSleepQpc.QuadPart >=
                        rxHardDeadlineQpc)
                    {
                        MarkRxHardTimeoutDiagnostic(
                            true,
                            0ULL);

                        FinalizeRxDiagnostic();

                        return -1;
                    }


                    uint64_t remainingCounts =
                        (uint64_t)
                        (
                            rxHardDeadlineQpc -
                            rxBeforeSleepQpc.QuadPart
                        );


                    if (remainingCounts <=
                        rxSleepCounts)
                    {
                        continue;
                    }
                }


                rxCoarseSleepCount++;

                rxDiagSleepCountWindow++;


                RtSleepFt(
                    &wait);
            }

            continue;
        }


        // =================================================
        // EtherType Check
        // =================================================

        if (rxLen <
            totalFrameLength)
        {
            rxDiagInvalidFrameWindow++;

            continue;
        }


        if (m_rxBuffer[12] !=
                0x88 ||
            m_rxBuffer[13] !=
                0xA4)
        {
            rxDiagInvalidFrameWindow++;

            continue;
        }


        // =================================================
        // Datagram #1 Check
        //
        // LRW
        // =================================================

        if (m_rxBuffer[
                lrwOffset + 0] !=
            0x0C)
        {
            rxDiagInvalidFrameWindow++;

            continue;
        }


        if (m_rxBuffer[
                lrwOffset + 1] !=
            lrwIdx)
        {
            rxDiagInvalidFrameWindow++;

            continue;
        }


        // =================================================
        // Datagram #2 Check
        //
        // FRMW
        // =================================================

        if (m_rxBuffer[
                frmwOffset + 0] !=
            0x0E)
        {
            rxDiagInvalidFrameWindow++;

            continue;
        }


        if (m_rxBuffer[
                frmwOffset + 1] !=
            frmwIdx)
        {
            rxDiagInvalidFrameWindow++;

            continue;
        }


        // =================================================
        // 7. Copy LRW Input Process Image
        //
        // EtherCAT Slave 已經修改 LRW Data。
        // =================================================

        memcpy(
            data,
            &m_rxBuffer[
                lrwDataOffset],
            length);


        // =================================================
        // 8. LRW WKC
        // =================================================

        uint16_t lrwWkc =
            (uint16_t)
            m_rxBuffer[
                lrwWkcOffset]
            |
            (
                (uint16_t)
                m_rxBuffer[
                    lrwWkcOffset +
                    1]
                <<
                8
            );


        // =================================================
        // 9. DC Reference Time
        //
        // FRMW Data 8 Bytes
        // =================================================

        uint64_t receivedDcTime =
            0;


        memcpy(
            &receivedDcTime,
            &m_rxBuffer[
                frmwDataOffset],
            sizeof(
                uint64_t));


        *dcReferenceTime =
            receivedDcTime;


        // =================================================
        // 10. FRMW WKC
        // =================================================

        uint16_t receivedDcWkc =
            (uint16_t)
            m_rxBuffer[
                frmwWkcOffset]
            |
            (
                (uint16_t)
                m_rxBuffer[
                    frmwWkcOffset +
                    1]
                <<
                8
            );


        *dcWkc =
            (int)
            receivedDcWkc;


        if (rxAttemptSoftLate)
        {
            rxDiagSoftLateAcceptedWindow++;
            rxDiagTotalSoftLateAccepted++;

            if (rxAttemptElapsedNs >
                rxDiagSoftLateElapsedMaxNsWindow)
            {
                rxDiagSoftLateElapsedMaxNsWindow =
                    rxAttemptElapsedNs;
            }
        }


        // =============================================================
        // V1A RX diagnostic success.
        // =============================================================

        if (rxAttemptNumber ==
            1U)
        {
            rxDiagFirstRxSuccessWindow++;
        }


        // =============================================================
        // Consecutive Timeout Diagnostic
        //
        // A successful matching EtherCAT response after one or more
        // timed-out calls counts as one recovery and clears the streak.
        // =============================================================

        if (rxDiagCurrentConsecutiveTimeout >
            0)
        {
            rxDiagRecoveryAfterTimeoutWindow++;
            rxDiagCurrentConsecutiveTimeout = 0;
        }


        FinalizeRxDiagnostic();


        // =================================================
        // Success
        //
        // Return LRW Working Counter
        // =================================================

        return (int)
            lrwWkc;
    }


    // =====================================================
    // Timeout / no matching EtherCAT response before RX deadline
    // =====================================================

    return -1;
}
