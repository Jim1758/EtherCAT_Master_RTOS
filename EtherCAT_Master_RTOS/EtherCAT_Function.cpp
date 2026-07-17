// 檔案：EtherCatMaster_Run.cpp
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <stdio.h>
#define MAX_MBX_SIZE 1024

int EtherCatMaster::ScanSlaves() //掃描所有從站
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

int EtherCatMaster::ecx_BRD(uint16_t ADP, uint16_t ADO, uint16_t length, int timeout)//廣播讀取
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
int EtherCatMaster::ecx_BWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout)//廣播寫入
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
int EtherCatMaster::ecx_APRD(uint16_t ADP, uint16_t ADO, uint16_t length, void* data, int timeout)//自動增量物理讀取
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
int EtherCatMaster::ecx_APWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout)//自動增量物理寫入
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
int EtherCatMaster::ecx_LRW(uint32_t LogAddr, uint16_t length, void* data, int timeout)//邏輯讀寫
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
int EtherCatMaster::ecx_SDOwrite(int slave_pos, uint16_t index, uint8_t subindex, int CA, int size, void* data, int timeout)//服務資料物件寫入 (SDO Write / 寫入物件字典)
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
int EtherCatMaster::ecx_SDOread(int slave_pos, uint16_t index, uint8_t subindex, int CA, int* size, void* data, int timeout)//服務資料物件讀取 (SDO Read / 讀取物件字典)
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
int EtherCatMaster::ecx_FPWR(uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout)//指定位址物理寫入
{
    // Command 5 = FPWR (寫入指定物理位址)
    return SendAndReceiveRegister(0x05, slaveAddr, regAddr, data, len, timeout);
}
int EtherCatMaster::ecx_FPRD(uint16_t slaveAddr, uint16_t regAddr, void* buffer, int len, int timeout)//指定位址物理讀取
{
    // Command 4 = FPRD (讀取指定物理位址)
    return SendAndReceiveRegister(0x04, slaveAddr, regAddr, buffer, len, timeout);
}

bool EtherCatMaster::SendAndReceiveRegister(uint8_t cmd, uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout)//暫存器底層收發核心
{
    uint8_t sendBuf[1518];
    uint8_t recvBuf[1518];
    memset(sendBuf, 0, sizeof(sendBuf));

    // --- A. 組裝 Ethernet Header ---
    // Dest MAC: Broadcast (FF-FF-FF-FF-FF-FF) 或直接由 NIC Driver 處理
    // Src MAC: NIC Driver 會自己填
    // EtherType: 0x88A4 (EtherCAT)
    int idx = 0;

    // 為了簡單，假設 NIC Driver 會幫忙填 MAC Header (14 bytes)
    // 如果您的 NIC Driver 只送 Payload，請從 EtherCAT Header 開始填
    // 這裡示範包含 MAC Header 的寫法:
    for (int i = 0; i < 6; i++) sendBuf[idx++] = 0xFF; // Dest
    for (int i = 0; i < 6; i++) sendBuf[idx++] = 0x00; // Src (Placeholder)
    sendBuf[idx++] = 0x88; // EtherType High
    sendBuf[idx++] = 0xA4; // EtherType Low

    // --- B. 組裝 EtherCAT Header (2 Bytes) ---
    // Length: PDU Header(10) + Data(len) + WKC(2)
    uint16_t totalLen = 10 + len + 2;
    uint16_t ecHeader = (totalLen & 0x7FF) | (0x1000); // Type 1 = EtherCAT Command
    memcpy(&sendBuf[idx], &ecHeader, 2);
    idx += 2;

    // --- C. 組裝 PDU Header (10 Bytes) ---
    sendBuf[idx++] = cmd;       // Command (4=Read, 5=Write)
    sendBuf[idx++] = 0x00;      // Index (封包編號，這裡簡單設0)

    memcpy(&sendBuf[idx], &slaveAddr, 2); // Address (Physical)
    idx += 2;

    memcpy(&sendBuf[idx], &regAddr, 2);   // Register Offset
    idx += 2;

    uint16_t lenInfo = (uint16_t)len & 0x7FF; // Length (11 bits)
    memcpy(&sendBuf[idx], &lenInfo, 2);
    idx += 2;

    uint16_t irqInfo = 0x0000;  // Interrupt info
    memcpy(&sendBuf[idx], &irqInfo, 2);
    idx += 2;

    // --- D. 填入資料 (如果是 Write) ---
    if (cmd == 0x05 && data != nullptr) {
        memcpy(&sendBuf[idx], data, len);
    }
    // 如果是 Read，這裡留空 (0x00)
    idx += len;

    // --- E. WKC (Working Counter) 預留 2 Bytes ---
    sendBuf[idx++] = 0x00;
    sendBuf[idx++] = 0x00;

    // --- F. 發送封包 ---
    // 注意：呼叫您的網卡驅動
    // 這裡假設您的 MyNic 指標叫做 m_pNic
    if (!m_pNic) return false;
    m_pNic->SendPacket(sendBuf, idx);

    // --- G. 等待接收 (簡單的 Polling 機制) ---
    // 因為這是在 ConfigDC 初始化階段，可以用 Busy Wait
    // 實際 RTX 環境請注意不要卡死太久
    //int retry = 100; // 嘗試 100 次

    // 接收迴圈 (Index Matching)
    int max_retries = timeout * 100;
    LARGE_INTEGER wait; wait.QuadPart = 10;


    while (max_retries-- > 0)
    {
        int recvLen = m_pNic->ReceivePacket(recvBuf); // 假設回傳接收長度
        if (recvLen > 0)
        {
            // 簡單檢查 EtherType 是否為 0x88A4
            if (recvBuf[12] == 0x88 && recvBuf[13] == 0xA4)
            {
                // 檢查 WKC (在封包最後 2 bytes)
                // Offset 計算: MAC(14) + EC(2) + PDU(10) + Data(len)
                int wkcOffset = 14 + 2 + 10 + len;
                uint16_t wkc = *(uint16_t*)&recvBuf[wkcOffset];

                if (wkc >= 1) { // 至少有一個從站處理成功
                    // 如果是 Read 指令，把資料複製出來
                    if (cmd == 0x04 && data != nullptr) {
                        // Data Offset = 14 + 2 + 10 = 26
                        memcpy(data, &recvBuf[26], len);
                    }
                    return true; // 成功！
                }
            }
        }
        RtSleepFt(&wait);
    }

    DEBUG_PRINT("Register R/W Timeout! Addr: 0x%X Reg: 0x%X\n", slaveAddr, regAddr);
    return false;
}


uint16_t EtherCatMaster::ReadSII_Word16(int slave_idx, uint16_t word_addr)//讀取從站EEPROM 功能
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

int EtherCatMaster::WriteFmmuRegister(int slaveIdx, int fmmuIdx, uint32_t logAddr, uint16_t len, uint16_t physAddr, uint8_t type, int timeout)//輔助函式：寫入單一 FMMU 設定
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

void EtherCatMaster::Printf_Slaves_State()//印出從站狀態
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

void EtherCatMaster::Printf_AL_Status_Code()////印出從站狀態0x134 code
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

bool EtherCatMaster::PDO_SendCommandAndWait(EcatCmdType type, uint16_t slave, uint16_t index, uint8_t sub, uint32_t value, int len, int timeoutMs)//非同步指令發送與同步等待 (執行緒安全指令)
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




void EtherCatMaster::Get_TotalSlave_WKC_Count()//取得從站WKC 分數
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


