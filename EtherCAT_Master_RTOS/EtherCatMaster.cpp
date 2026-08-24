#include "EtherCatMaster.h"
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <cstring>
#include <stdio.h>
#include "GlobalConfig.h"
#include "PLCManager.h" // 🌟 1. 記得 include PLCManager 標頭檔
#define MAX_MBX_SIZE 1024
EtherCatMaster::EtherCatMaster() : m_pNic(nullptr), m_pEni(nullptr), m_idx(0), m_mboxCnt(0)
{
    // 清空傳送與接收緩衝區
    memset(m_txBuffer, 0, sizeof(m_txBuffer));
    memset(m_rxBuffer, 0, sizeof(m_rxBuffer));

    m_NC = new NCManager(m_Motion);

    // =========================================================
    // G81 HOME Manager Link
    //
    // 將 EtherCatMaster 內真正運行中的 PLCManager
    // 交給 HomingManager。
    //
    // 之後 HOME 才能讀取：
    //
    // C100~107 HOME DOG
    // C108~115 HOME INDEX
    // External Reference C Point
    //
    // 目前只建立連結，不會啟動任何 HOME Motion。
    // =========================================================

    m_NC->Homing.LinkPLCManager(
        &m_plcManager);

    // 🌟 將全域指標指向這顆唯一真正有在跑的 PLC 大腦！
    g_PLC = &this->m_plcManager;
}
EtherCatMaster::~EtherCatMaster()
{
    if (m_NC != nullptr)
    {
        delete m_NC;
        m_NC = nullptr;
    }
}
void EtherCatMaster::AttachNic(CNicDriver* pNic)// 綁定網卡驅動程式
{
    m_pNic = pNic;
}
void EtherCatMaster::AttachEni(EtherCatEni* pEni) // 綁定 ENI 設定檔解析器
{
    m_pEni = pEni;
}



bool EtherCatMaster::InitSlaveMailboxInfo(int slave_idx)//Stage 6B Runtime Mailbox Cutover
{
    // ========================================================================
    // Identity is still read from SII.
    //
    // Stage 1B topology verification continues to use these fields.
    // ========================================================================

    m_slaveInfo[slave_idx].Vendor_ID =
        ReadSII_Uint32(
            slave_idx,
            0x0008);


    m_slaveInfo[slave_idx].Product_Code =
        ReadSII_Uint32(
            slave_idx,
            0x000A);


    m_slaveInfo[slave_idx].Revision_No =
        ReadSII_Uint32(
            slave_idx,
            0x000C);


    // ========================================================================
    // Stage 6B - Runtime Mailbox Cutover
    //
    // If Runtime XML explicitly contains Mailbox data, it becomes the source
    // for the software Mailbox routing used by ecx_SDOread/ecx_SDOwrite.
    //
    // IMPORTANT:
    // - This does NOT write SyncManager hardware.
    // - SM hardware is already Runtime-driven by Stage 5D.
    // - Stage 6A already proved Runtime Mailbox == Software == ESC readback.
    // - If Runtime Mailbox is present but invalid, Startup must fail.
    //   We do NOT silently fall back after a partial Runtime definition.
    //
    // Legacy Vendor/ProductCode mapping remains only for:
    // - old Runtime XML with no <Mailbox>
    // - device families not yet represented by the Runtime schema
    // ========================================================================

    if (m_pEni != nullptr)
    {
        const auto& runtimeSlaves =
            m_pEni->GetSlaves();


        if (slave_idx >= 0 &&
            slave_idx <
            (int)runtimeSlaves.size())
        {
            const EtherCatSlave& runtimeSlave =
                runtimeSlaves[(size_t)slave_idx];


            if (runtimeSlave.runtimeMailbox.present)
            {
                const EtherCatRuntimeMailboxDirectionConfig& out =
                    runtimeSlave.runtimeMailbox.out;


                const EtherCatRuntimeMailboxDirectionConfig& in =
                    runtimeSlave.runtimeMailbox.in;


                const bool valid =
                    out.present &&
                    in.present &&
                    out.smIndex >= 0 &&
                    out.smIndex <= 15 &&
                    in.smIndex >= 0 &&
                    in.smIndex <= 15 &&
                    out.startAddress != 0 &&
                    in.startAddress != 0 &&
                    out.length > 0 &&
                    in.length > 0 &&
                    out.length <= MAX_MBX_SIZE &&
                    in.length <= MAX_MBX_SIZE;


                if (!valid)
                {
                    RtPrintf(
                        "[RUNTIME-MAILBOX-CUTOVER] "
                        "S%d | Runtime Mailbox present but invalid | "
                        "Out Present:%d SM:%d Addr:0x%04X Len:%u | "
                        "In Present:%d SM:%d Addr:0x%04X Len:%u | "
                        "Result:FAIL\n",

                        slave_idx,

                        out.present
                        ? 1
                        : 0,

                        out.smIndex,

                        (unsigned int)
                        out.startAddress,

                        (unsigned int)
                        out.length,

                        in.present
                        ? 1
                        : 0,

                        in.smIndex,

                        (unsigned int)
                        in.startAddress,

                        (unsigned int)
                        in.length);


                    return
                        false;
                }


                m_slaveInfo[slave_idx].mbxOutAddr =
                    out.startAddress;


                m_slaveInfo[slave_idx].mbxOutLength =
                    out.length;


                m_slaveInfo[slave_idx].mbxInAddr =
                    in.startAddress;


                m_slaveInfo[slave_idx].mbxInLength =
                    in.length;


                // Preserve current first-servo discovery semantics without
                // depending on a specific ProductCode.
                if (strcmp(
                    runtimeSlave.type,
                    "Servo") ==
                    0 &&
                    Motor_Start_Index ==
                    -1)
                {
                    Motor_Start_Index =
                        slave_idx;
                }


                RtPrintf(
                    "[RUNTIME-MAILBOX-CUTOVER] "
                    "S%d | Source:RUNTIME_XML | "
                    "Out SM%d 0x%04X/%u | "
                    "In SM%d 0x%04X/%u | "
                    "Result:PASS\n",

                    slave_idx,

                    out.smIndex,

                    (unsigned int)
                    m_slaveInfo[slave_idx].mbxOutAddr,

                    (unsigned int)
                    m_slaveInfo[slave_idx].mbxOutLength,

                    in.smIndex,

                    (unsigned int)
                    m_slaveInfo[slave_idx].mbxInAddr,

                    (unsigned int)
                    m_slaveInfo[slave_idx].mbxInLength);


                return
                    true;
            }
        }
    }


    // ========================================================================
    // Legacy fallback
    //
    // Runtime XML has no explicit Mailbox section for this slave.
    // Keep original behavior unchanged.
    // ========================================================================

    switch (m_slaveInfo[slave_idx].Vendor_ID)
    {
    case 0x0000066F://松下
        switch (m_slaveInfo[slave_idx].Product_Code)
        {
        case 0x60380000: //A6B

            if (Motor_Start_Index == -1)//紀錄第一站馬達 位置
            {
                Motor_Start_Index = slave_idx;
            }

            m_slaveInfo[slave_idx].mbxOutAddr = 0x1000;
            m_slaveInfo[slave_idx].mbxOutLength = 0x100;

            m_slaveInfo[slave_idx].mbxInAddr = 0x1200;
            m_slaveInfo[slave_idx].mbxInLength = 0x100;
            break;


        default:
            break;
        }
        break;


    case 0xAAAA://上銀
        switch (m_slaveInfo[slave_idx].Product_Code)
        {
        case 0x00000005: //E1

            if (Motor_Start_Index == -1)
            {
                Motor_Start_Index = slave_idx;
            }

            m_slaveInfo[slave_idx].mbxOutAddr = 0x1800;
            m_slaveInfo[slave_idx].mbxOutLength = 20;
            m_slaveInfo[slave_idx].mbxInAddr = 0x18F6;
            m_slaveInfo[slave_idx].mbxInLength = 20;
            break;


        default:
            break;
        }
        break;


    case 0x00278606://OSCARMAX
        switch (m_slaveInfo[slave_idx].Product_Code)
        {
        case 0x00000003:
            break;


        default:
            break;
        }

        break;


    case 0x000001dd:
        switch (m_slaveInfo[slave_idx].Product_Code)
        {
            // --------------------------------------------------------------------
            // These current Delta cases remain as LEGACY XML fallback only.
            //
            // With the Stage 6A Runtime XML used by the current machine,
            // A3E / 8124 return from the Runtime branch above and never execute
            // these constants.
            // --------------------------------------------------------------------

        case 0x00006010: //A3E legacy fallback

            if (Motor_Start_Index == -1)
            {
                Motor_Start_Index = slave_idx;
            }

            m_slaveInfo[slave_idx].mbxOutAddr = 0x1000;
            m_slaveInfo[slave_idx].mbxOutLength = 128;
            m_slaveInfo[slave_idx].mbxInAddr = 0x10C0;
            m_slaveInfo[slave_idx].mbxInLength = 128;
            break;


        case 0x00008124: //8124 legacy fallback
            m_slaveInfo[slave_idx].mbxOutAddr = 0x1000;
            m_slaveInfo[slave_idx].mbxOutLength = 128;
            m_slaveInfo[slave_idx].mbxInAddr = 0x1080;
            m_slaveInfo[slave_idx].mbxInLength = 128;
            break;


        default:
            break;
        }
        break;


    default:
        break;
    }


    RtPrintf(
        "[RUNTIME-MAILBOX-CUTOVER] "
        "S%d | RuntimeMailbox:NO | "
        "Action:LEGACY_FALLBACK | "
        "Out:0x%04X/%u | In:0x%04X/%u | Result:PASS\n",

        slave_idx,

        (unsigned int)
        m_slaveInfo[slave_idx].mbxOutAddr,

        (unsigned int)
        m_slaveInfo[slave_idx].mbxOutLength,

        (unsigned int)
        m_slaveInfo[slave_idx].mbxInAddr,

        (unsigned int)
        m_slaveInfo[slave_idx].mbxInLength);


    return
        true;
}


// ============================================================================
// Stage 1B - EtherCAT Topology / Identity Verification
//
// 目的：
//
// 在任何 Watchdog / SM / PDO / DC 設定寫入之前，先確認：
//
// 1. Runtime XML 期待的從站數量
// 2. EtherCAT Bus 實際回應的從站數量
// 3. 每個實體 Position 的 Vendor ID
// 4. 每個實體 Position 的 Product Code
// 5. Revision
// 6. Runtime XML ConfiguredAddress
//
// 這個函式只有 Read / Compare / Log。
// 不改 AL State、不寫 PDO、不寫 SM、不寫 DC。
//
// Stage 1B:
// Runtime XML Revision / ConfiguredAddress 已正式納入比對。
// 實體 Revision 由 SII Identity Word Address 0x000C 讀取。
// ============================================================================
bool EtherCatMaster::VerifyTopologyAgainstRuntimeConfig()
{
    // ------------------------------------------------------------------------
    // Basic safety
    // ------------------------------------------------------------------------

    if (m_pEni == nullptr)
    {
        RtPrintf(
            "\n"
            "============================================================\n"
            "[ECAT-TOPOLOGY-VERIFY] FAILED\n"
            "[ECAT-TOPOLOGY-VERIFY] Reason:Runtime Config is not attached.\n"
            "============================================================\n\n");

        return false;
    }


    if (m_pNic == nullptr)
    {
        RtPrintf(
            "\n"
            "============================================================\n"
            "[ECAT-TOPOLOGY-VERIFY] FAILED\n"
            "[ECAT-TOPOLOGY-VERIFY] Reason:NIC is not attached.\n"
            "============================================================\n\n");

        return false;
    }


    const auto& expectedSlaves =
        m_pEni->GetSlaves();


    const int expectedCount =
        (int)expectedSlaves.size();


    const int slaveInfoCapacity =
        (int)(
            sizeof(m_slaveInfo) /
            sizeof(m_slaveInfo[0]));


    RtPrintf(
        "\n"
        "============================================================\n"
        "[ECAT-TOPOLOGY-VERIFY] BEGIN | Stage:1B\n"
        "============================================================\n");


    RtPrintf(
        "[ECAT-TOPOLOGY-VERIFY] RuntimeExpected:%d | SlaveInfoCapacity:%d\n",
        expectedCount,
        slaveInfoCapacity);


    if (expectedCount <= 0)
    {
        RtPrintf(
            "[ECAT-TOPOLOGY-VERIFY] FAILED | "
            "Reason:Runtime Config contains no slaves.\n");

        RtPrintf(
            "============================================================\n"
            "[ECAT-TOPOLOGY-VERIFY] END | Result:FAIL\n"
            "============================================================\n\n");

        return false;
    }


    if (expectedCount > slaveInfoCapacity)
    {
        RtPrintf(
            "[ECAT-TOPOLOGY-VERIFY] FAILED | "
            "Reason:Expected slave count exceeds m_slaveInfo capacity | "
            "Expected:%d Capacity:%d\n",
            expectedCount,
            slaveInfoCapacity);

        RtPrintf(
            "============================================================\n"
            "[ECAT-TOPOLOGY-VERIFY] END | Result:FAIL\n"
            "============================================================\n\n");

        return false;
    }


    // ------------------------------------------------------------------------
    // Physical slave count
    //
    // BRD 0x0130 is read-only.
    // ------------------------------------------------------------------------

    const int detectedCount =
        ecx_BRD(
            0x0000,
            0x0130,
            2,
            20);


    RtPrintf(
        "[ECAT-TOPOLOGY-VERIFY] SlaveCount | "
        "Expected:%d | Detected:%d | Match:%s\n",
        expectedCount,
        detectedCount,
        detectedCount == expectedCount
        ? "YES"
        : "NO");


    if (detectedCount <= 0)
    {
        RtPrintf(
            "[ECAT-TOPOLOGY-VERIFY] FAILED | "
            "Reason:BRD 0x0130 returned no EtherCAT slaves.\n");

        RtPrintf(
            "============================================================\n"
            "[ECAT-TOPOLOGY-VERIFY] END | Result:FAIL\n"
            "============================================================\n\n");

        return false;
    }


    if (detectedCount > slaveInfoCapacity)
    {
        RtPrintf(
            "[ECAT-TOPOLOGY-VERIFY] FAILED | "
            "Reason:Detected slave count exceeds m_slaveInfo capacity | "
            "Detected:%d Capacity:%d\n",
            detectedCount,
            slaveInfoCapacity);

        RtPrintf(
            "============================================================\n"
            "[ECAT-TOPOLOGY-VERIFY] END | Result:FAIL\n"
            "============================================================\n\n");

        return false;
    }


    bool result =
        true;


    int errorCount =
        0;


    int skippedRevisionCount =
        0;


    int skippedConfiguredAddressCount =
        0;


    if (detectedCount !=
        expectedCount)
    {
        result =
            false;

        errorCount++;
    }


    const int compareCount =
        expectedCount < detectedCount
        ? expectedCount
        : detectedCount;


    // ------------------------------------------------------------------------
    // Per-position Identity Verification
    //
    // Physical:
    //   Vendor      = SII 0x0008
    //   Product     = SII 0x000A
    //   Revision    = SII 0x000C
    //   ConfigAddr  = ESC Station Address after ScanSlaves()
    //
    // Runtime:
    //   VendorId
    //   ProductCode
    //   Revision
    //   ConfiguredAddress
    //
    // Older Runtime XML:
    // If Revision / ConfiguredAddress are absent, only that field is SKIP.
    // ------------------------------------------------------------------------

    for (int i = 0;
        i < compareCount;
        i++)
    {
        const EtherCatSlave& expected =
            expectedSlaves[(size_t)i];


        const uint32_t actualVendor =
            m_slaveInfo[i].Vendor_ID;


        const uint32_t actualProduct =
            m_slaveInfo[i].Product_Code;


        const uint32_t actualRevision =
            m_slaveInfo[i].Revision_No;


        const uint16_t actualConfigAddr =
            m_slaveInfo[i].configAddr;


        const bool vendorMatch =
            actualVendor ==
            expected.vendorId;


        const bool productMatch =
            actualProduct ==
            expected.productCode;


        bool revisionMatch =
            true;


        if (expected.hasRevision)
        {
            revisionMatch =
                actualRevision ==
                expected.revision;
        }
        else
        {
            skippedRevisionCount++;
        }


        bool addressMatch =
            true;


        if (expected.hasConfiguredAddress)
        {
            addressMatch =
                actualConfigAddr ==
                expected.configuredAddress;
        }
        else
        {
            skippedConfiguredAddressCount++;
        }


        const bool identityMatch =
            vendorMatch &&
            productMatch &&
            revisionMatch;


        const bool slavePass =
            identityMatch &&
            addressMatch;


        RtPrintf(
            "[ECAT-TOPOLOGY-VERIFY] "
            "S%d | "
            "Cfg Exp:0x%04X Act:0x%04X %s | "
            "Vendor Exp:0x%08X Act:0x%08X %s | "
            "Product Exp:0x%08X Act:0x%08X %s | "
            "Revision Exp:0x%08X Act:0x%08X %s | "
            "Result:%s\n",

            i,

            (unsigned int)
            expected.configuredAddress,

            (unsigned int)
            actualConfigAddr,

            expected.hasConfiguredAddress
            ? (addressMatch
                ? "MATCH"
                : "MISMATCH")
            : "SKIP",

            (unsigned int)
            expected.vendorId,

            (unsigned int)
            actualVendor,

            vendorMatch
            ? "MATCH"
            : "MISMATCH",

            (unsigned int)
            expected.productCode,

            (unsigned int)
            actualProduct,

            productMatch
            ? "MATCH"
            : "MISMATCH",

            (unsigned int)
            expected.revision,

            (unsigned int)
            actualRevision,

            expected.hasRevision
            ? (revisionMatch
                ? "MATCH"
                : "MISMATCH")
            : "SKIP",

            slavePass
            ? "PASS"
            : "FAIL");


        if (!slavePass)
        {
            result =
                false;

            errorCount++;
        }


        if (!vendorMatch)
        {
            RtPrintf(
                "[ECAT-TOPOLOGY-ERROR] S%d Vendor mismatch | "
                "Expected:0x%08X Actual:0x%08X\n",

                i,

                (unsigned int)
                expected.vendorId,

                (unsigned int)
                actualVendor);
        }


        if (!productMatch)
        {
            RtPrintf(
                "[ECAT-TOPOLOGY-ERROR] S%d Product mismatch | "
                "Expected:0x%08X Actual:0x%08X\n",

                i,

                (unsigned int)
                expected.productCode,

                (unsigned int)
                actualProduct);
        }


        if (expected.hasRevision &&
            !revisionMatch)
        {
            RtPrintf(
                "[ECAT-TOPOLOGY-ERROR] S%d Revision mismatch | "
                "Expected:0x%08X Actual:0x%08X\n",

                i,

                (unsigned int)
                expected.revision,

                (unsigned int)
                actualRevision);
        }


        if (expected.hasConfiguredAddress &&
            !addressMatch)
        {
            RtPrintf(
                "[ECAT-TOPOLOGY-ERROR] S%d Configured Address mismatch | "
                "Expected:0x%04X Actual:0x%04X\n",

                i,

                (unsigned int)
                expected.configuredAddress,

                (unsigned int)
                actualConfigAddr);
        }
    }


    // ------------------------------------------------------------------------
    // Count mismatch detail
    // ------------------------------------------------------------------------

    if (expectedCount >
        detectedCount)
    {
        for (int i = detectedCount;
            i < expectedCount;
            i++)
        {
            RtPrintf(
                "[ECAT-TOPOLOGY-ERROR] "
                "Missing physical slave at S%d | "
                "Expected:0x%08X:0x%08X Rev:0x%08X | Name:%s\n",

                i,

                (unsigned int)
                expectedSlaves[(size_t)i].vendorId,

                (unsigned int)
                expectedSlaves[(size_t)i].productCode,

                (unsigned int)
                expectedSlaves[(size_t)i].revision,

                expectedSlaves[(size_t)i].name);
        }
    }
    else if (detectedCount >
        expectedCount)
    {
        RtPrintf(
            "[ECAT-TOPOLOGY-ERROR] "
            "Detected %d extra physical slave(s) after Runtime Config S%d.\n",

            detectedCount -
            expectedCount,

            expectedCount -
            1);
    }


    RtPrintf(
        "[ECAT-TOPOLOGY-VERIFY-RESULT] "
        "Expected:%d | Detected:%d | Compared:%d | "
        "SkipRevision:%d | SkipCfgAddr:%d | "
        "Errors:%d | Result:%s\n",

        expectedCount,
        detectedCount,
        compareCount,
        skippedRevisionCount,
        skippedConfiguredAddressCount,
        errorCount,

        result
        ? "PASS"
        : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[ECAT-TOPOLOGY-VERIFY] END | Result:%s\n"
        "============================================================\n\n",

        result
        ? "PASS"
        : "FAIL");


    return result;
}


// ============================================================================
// Stage 2 - PRE-OP SM / PDO Readback Verification
//
// 目的：
//
// 目前 Master 仍使用既有的 Vendor / Product hard-code 寫入 SM 與 A3E PDO。
// 在未改成 Runtime Config data-driven 之前，先建立一層可靠的 Readback Gate。
//
// 執行時機：
//
// PRE-OP
//   -> Config_Slave_FMMU()
//   -> ConfigureSlaveGeneric_PRE_OP()
//   -> Watchdog
//   -> VerifyPreOpSmAndPdoConfiguration()   <-- HERE
//   -> PASS 才允許 SAFE-OP
//
// 這個函式只做：
//
// FPRD 讀 ESC SyncManager Register
// SDO Upload 讀 CoE PDO Mapping / Assignment
// Compare
// Log
//
// 不會寫入 SM / PDO / DC。
// ============================================================================
bool EtherCatMaster::VerifyPreOpSmAndPdoConfiguration()
{
    if (m_pEni == nullptr)
    {
        RtPrintf(
            "\n"
            "============================================================\n"
            "[ECAT-CONFIG-VERIFY] FAILED\n"
            "[ECAT-CONFIG-VERIFY] Reason:Runtime Config is not attached.\n"
            "============================================================\n\n");

        return false;
    }


    const auto& runtimeSlaves =
        m_pEni->GetSlaves();


    int errorCount =
        0;


    int smCheckCount =
        0;


    int pdoCheckCount =
        0;


    int skipCount =
        0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[ECAT-CONFIG-VERIFY] BEGIN | Stage:2 | State:PRE-OP\n"
        "============================================================\n");


    // ========================================================================
    // SyncManager Readback Helper
    //
    // ESC SM Register Layout:
    //
    // +0..1 Physical Start Address
    // +2..3 Length
    // +4    Control
    // +5    Status       (dynamic, do not compare)
    // +6    Activate     (compare Bit0 only)
    // +7    PDI Control  (not used in this Stage)
    // ========================================================================

    auto verifySm =
        [this,
        &errorCount,
        &smCheckCount]
    (
        int slaveIdx,
        int smIndex,
        uint16_t expectedStart,
        uint16_t expectedLength,
        uint8_t expectedControl,
        bool expectedEnable
        ) -> bool
    {
        smCheckCount++;


        uint8_t raw[8] =
        {
            0
        };


        const uint16_t registerAddress =
            (uint16_t)(
                0x0800 +
                (smIndex * 8));


        const int wkc =
            ecx_FPRD(
                m_slaveInfo[slaveIdx].configAddr,
                registerAddress,
                raw,
                8,
                20);


        if (wkc <= 0)
        {
            errorCount++;

            RtPrintf(
                "[SM-VERIFY] S%d SM%d | "
                "Reg:0x%04X | ReadWKC:%d | Result:FAIL\n",

                slaveIdx,
                smIndex,
                (unsigned int)
                registerAddress,
                wkc);

            return false;
        }


        const uint16_t actualStart =
            (uint16_t)(
                ((uint16_t)raw[0]) |
                ((uint16_t)raw[1] << 8));


        const uint16_t actualLength =
            (uint16_t)(
                ((uint16_t)raw[2]) |
                ((uint16_t)raw[3] << 8));


        const uint8_t actualControl =
            raw[4];


        const bool actualEnable =
            (raw[6] & 0x01U) != 0;


        const bool startMatch =
            actualStart ==
            expectedStart;


        const bool lengthMatch =
            actualLength ==
            expectedLength;


        const bool controlMatch =
            actualControl ==
            expectedControl;


        const bool enableMatch =
            actualEnable ==
            expectedEnable;


        const bool pass =
            startMatch &&
            lengthMatch &&
            controlMatch &&
            enableMatch;


        RtPrintf(
            "[SM-VERIFY] S%d SM%d | "
            "Start Exp:0x%04X Act:0x%04X %s | "
            "Len Exp:%u Act:%u %s | "
            "Ctrl Exp:0x%02X Act:0x%02X %s | "
            "Enable Exp:%d Act:%d %s | "
            "WKC:%d | Result:%s\n",

            slaveIdx,
            smIndex,

            (unsigned int)
            expectedStart,

            (unsigned int)
            actualStart,

            startMatch
            ? "MATCH"
            : "MISMATCH",

            (unsigned int)
            expectedLength,

            (unsigned int)
            actualLength,

            lengthMatch
            ? "MATCH"
            : "MISMATCH",

            (unsigned int)
            expectedControl,

            (unsigned int)
            actualControl,

            controlMatch
            ? "MATCH"
            : "MISMATCH",

            expectedEnable
            ? 1
            : 0,

            actualEnable
            ? 1
            : 0,

            enableMatch
            ? "MATCH"
            : "MISMATCH",

            wkc,

            pass
            ? "PASS"
            : "FAIL");


        if (!pass)
        {
            errorCount++;
        }


        return pass;
    };


    // ========================================================================
    // SDO Readback Helpers
    // ========================================================================

    auto verifySdoU8 =
        [this,
        &errorCount,
        &pdoCheckCount]
    (
        int slaveIdx,
        uint16_t index,
        uint8_t subIndex,
        uint8_t expected
        ) -> bool
    {
        pdoCheckCount++;


        uint8_t actual =
            0;


        int size =
            1;


        const int result =
            ecx_SDOread(
                slaveIdx,
                index,
                subIndex,
                FALSE,
                &size,
                &actual,
                20);


        const bool readOk =
            result > 0 &&
            size == 1;


        const bool match =
            readOk &&
            actual == expected;


        RtPrintf(
            "[PDO-VERIFY] S%d %04X:%02X | "
            "Size:U8 | Exp:0x%02X | Act:0x%02X | "
            "Read:%s | Result:%s\n",

            slaveIdx,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            (unsigned int)
            expected,

            (unsigned int)
            actual,

            readOk
            ? "OK"
            : "FAIL",

            match
            ? "PASS"
            : "FAIL");


        if (!match)
        {
            errorCount++;
        }


        return match;
    };


    auto verifySdoU16 =
        [this,
        &errorCount,
        &pdoCheckCount]
    (
        int slaveIdx,
        uint16_t index,
        uint8_t subIndex,
        uint16_t expected
        ) -> bool
    {
        pdoCheckCount++;


        uint16_t actual =
            0;


        int size =
            2;


        const int result =
            ecx_SDOread(
                slaveIdx,
                index,
                subIndex,
                FALSE,
                &size,
                &actual,
                20);


        const bool readOk =
            result > 0 &&
            size == 2;


        const bool match =
            readOk &&
            actual == expected;


        RtPrintf(
            "[PDO-VERIFY] S%d %04X:%02X | "
            "Size:U16 | Exp:0x%04X | Act:0x%04X | "
            "Read:%s | Result:%s\n",

            slaveIdx,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            (unsigned int)
            expected,

            (unsigned int)
            actual,

            readOk
            ? "OK"
            : "FAIL",

            match
            ? "PASS"
            : "FAIL");


        if (!match)
        {
            errorCount++;
        }


        return match;
    };


    auto verifySdoU32 =
        [this,
        &errorCount,
        &pdoCheckCount]
    (
        int slaveIdx,
        uint16_t index,
        uint8_t subIndex,
        uint32_t expected
        ) -> bool
    {
        pdoCheckCount++;


        uint32_t actual =
            0;


        int size =
            4;


        const int result =
            ecx_SDOread(
                slaveIdx,
                index,
                subIndex,
                FALSE,
                &size,
                &actual,
                20);


        const bool readOk =
            result > 0 &&
            size == 4;


        const bool match =
            readOk &&
            actual == expected;


        RtPrintf(
            "[PDO-VERIFY] S%d %04X:%02X | "
            "Size:U32 | Exp:0x%08X | Act:0x%08X | "
            "Read:%s | Result:%s\n",

            slaveIdx,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            (unsigned int)
            expected,

            (unsigned int)
            actual,

            readOk
            ? "OK"
            : "FAIL",

            match
            ? "PASS"
            : "FAIL");


        if (!match)
        {
            errorCount++;
        }


        return match;
    };


    // ========================================================================
    // A3E PDO Mapping / Assignment Readback
    //
    // RxPDO 0x1601:
    //
    // 6040:00 16
    // 60FF:00 32
    // 60B8:00 16
    // 6060:00 8
    //
    // TxPDO 0x1A01:
    //
    // 6041:00 16
    // 6064:00 32
    // 606C:00 32
    // 6077:00 16
    // 60B9:00 16
    // 60BA:00 32
    // 6061:00 8
    // 2510:00 32
    // ========================================================================

    auto verifyA3EPdo =
        [&verifySdoU8,
        &verifySdoU16,
        &verifySdoU32]
    (
        int slaveIdx
        ) -> void
    {
        RtPrintf(
            "[PDO-VERIFY] S%d A3E | RxPDO Assignment / Mapping\n",
            slaveIdx);


        verifySdoU8(
            slaveIdx,
            0x1C12,
            0x00,
            1);


        verifySdoU16(
            slaveIdx,
            0x1C12,
            0x01,
            0x1601);


        verifySdoU8(
            slaveIdx,
            0x1601,
            0x00,
            4);


        verifySdoU32(
            slaveIdx,
            0x1601,
            0x01,
            0x60400010);


        verifySdoU32(
            slaveIdx,
            0x1601,
            0x02,
            0x60FF0020);


        verifySdoU32(
            slaveIdx,
            0x1601,
            0x03,
            0x60B80010);


        verifySdoU32(
            slaveIdx,
            0x1601,
            0x04,
            0x60600008);


        RtPrintf(
            "[PDO-VERIFY] S%d A3E | TxPDO Assignment / Mapping\n",
            slaveIdx);


        verifySdoU8(
            slaveIdx,
            0x1C13,
            0x00,
            1);


        verifySdoU16(
            slaveIdx,
            0x1C13,
            0x01,
            0x1A01);


        verifySdoU8(
            slaveIdx,
            0x1A01,
            0x00,
            8);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x01,
            0x60410010);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x02,
            0x60640020);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x03,
            0x606C0020);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x04,
            0x60770010);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x05,
            0x60B90010);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x06,
            0x60BA0020);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x07,
            0x60610008);


        verifySdoU32(
            slaveIdx,
            0x1A01,
            0x08,
            0x25100020);
    };


    // ========================================================================
    // Per-Slave Verification
    //
    // 目前只比對「現有 Master 確實有設定」的配置。
    //
    // 5500:
    //   Coupler，現有程式未配置 Process Data SM -> explicit N/A
    //
    // 6002:
    //   SM0 Input 0x1000 / 2 B
    //
    // 7062:
    //   SM0 Output 0x0F00 / 1 B
    //   SM1 Output 0x0F01 / 1 B
    //
    // 8124 / 8124D0:
    //   SM0 Mailbox Out
    //   SM1 Mailbox In
    //   SM2 Output Disabled
    //   SM3 Input 8 B
    //
    // A3E:
    //   SM0..SM3 + complete 1601/1A01/1C12/1C13 readback
    // ========================================================================

    for (int i = 0;
        i < (int)runtimeSlaves.size();
        i++)
    {
        const uint32_t vendor =
            m_slaveInfo[i].Vendor_ID;


        const uint32_t product =
            m_slaveInfo[i].Product_Code;


        RtPrintf(
            "[ECAT-CONFIG-VERIFY] S%d | "
            "Vendor:0x%08X Product:0x%08X\n",

            i,

            (unsigned int)
            vendor,

            (unsigned int)
            product);


        if (vendor ==
            0x000001DD)
        {
            switch (product)
            {
            case 0x00005500:
                // ------------------------------------------------------------
                // R1-EC5500 Coupler
                //
                // 現有 Master 沒有對它建立 Process Data SM。
                // 這不是未驗證錯誤，而是明確 N/A。
                // ------------------------------------------------------------

                skipCount++;

                RtPrintf(
                    "[SM-VERIFY] S%d R1-EC5500 | "
                    "No active Process Data SM in current runtime profile | "
                    "Result:N/A\n",

                    i);

                break;


            case 0x00006002:
                // ------------------------------------------------------------
                // R1-EC6002
                // SM0 Input
                // ------------------------------------------------------------

                verifySm(
                    i,
                    0,
                    0x1000,
                    2,
                    0x00,
                    true);

                break;


            case 0x00007062:
                // ------------------------------------------------------------
                // R1-EC7062
                // 16 outputs are split across two 1-byte SMs.
                // ------------------------------------------------------------

                verifySm(
                    i,
                    0,
                    0x0F00,
                    1,
                    0x44,
                    true);


                verifySm(
                    i,
                    1,
                    0x0F01,
                    1,
                    0x44,
                    true);

                break;


            case 0x00008124:
                // ------------------------------------------------------------
                // R1-EC8124 / 8124D0
                // ------------------------------------------------------------

                verifySm(
                    i,
                    0,
                    0x1000,
                    128,
                    0x26,
                    true);


                verifySm(
                    i,
                    1,
                    0x1080,
                    128,
                    0x22,
                    true);


                verifySm(
                    i,
                    2,
                    0x1100,
                    0,
                    0x24,
                    false);


                verifySm(
                    i,
                    3,
                    0x11C0,
                    8,
                    0x20,
                    true);

                break;


            case 0x00006010:
                // ------------------------------------------------------------
                // Delta ASDA-A3-E
                // ------------------------------------------------------------

                verifySm(
                    i,
                    0,
                    0x1000,
                    128,
                    0x36,
                    true);


                verifySm(
                    i,
                    1,
                    0x10C0,
                    128,
                    0x32,
                    true);


                verifySm(
                    i,
                    2,
                    0x1180,
                    9,
                    0x24,
                    true);


                verifySm(
                    i,
                    3,
                    0x1480,
                    23,
                    0x00,
                    true);


                verifyA3EPdo(
                    i);

                break;


            default:
                // ------------------------------------------------------------
                // Stage 2 safety policy:
                //
                // 新裝置若有 Process Data，但還沒有 Readback Profile，
                // 不允許默默跳過後進 SAFE-OP。
                // ------------------------------------------------------------

                if (runtimeSlaves[(size_t)i].inputBitLength > 0 ||
                    runtimeSlaves[(size_t)i].outputBitLength > 0)
                {
                    errorCount++;

                    RtPrintf(
                        "[ECAT-CONFIG-VERIFY-ERROR] S%d | "
                        "No Stage-2 verification profile for "
                        "Vendor:0x%08X Product:0x%08X | Result:FAIL\n",

                        i,

                        (unsigned int)
                        vendor,

                        (unsigned int)
                        product);
                }
                else
                {
                    skipCount++;

                    RtPrintf(
                        "[ECAT-CONFIG-VERIFY] S%d | "
                        "No Process Data | Result:N/A\n",

                        i);
                }

                break;
            }
        }
        else
        {
            if (runtimeSlaves[(size_t)i].inputBitLength > 0 ||
                runtimeSlaves[(size_t)i].outputBitLength > 0)
            {
                errorCount++;

                RtPrintf(
                    "[ECAT-CONFIG-VERIFY-ERROR] S%d | "
                    "Unsupported Stage-2 Vendor:0x%08X Product:0x%08X | "
                    "Result:FAIL\n",

                    i,

                    (unsigned int)
                    vendor,

                    (unsigned int)
                    product);
            }
            else
            {
                skipCount++;

                RtPrintf(
                    "[ECAT-CONFIG-VERIFY] S%d | "
                    "No Process Data | Result:N/A\n",

                    i);
            }
        }
    }


    const bool result =
        errorCount == 0;


    RtPrintf(
        "[ECAT-CONFIG-VERIFY-RESULT] "
        "SMChecks:%d | PDOChecks:%d | Skips:%d | "
        "Errors:%d | Result:%s\n",

        smCheckCount,
        pdoCheckCount,
        skipCount,
        errorCount,

        result
        ? "PASS"
        : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[ECAT-CONFIG-VERIFY] END | Result:%s\n"
        "============================================================\n\n",

        result
        ? "PASS"
        : "FAIL");


    return result;
}


int EtherCatMaster::Initialize_Slaves()//初始化所有從站 INIT>>PRE-OP>>SAFE-OP>>OP
{
    if (!m_pEni) return -1;
    const auto& slaves = m_pEni->GetSlaves();
    int total_slaves = (int)slaves.size();
    int WK = 0;

    // =========================================================
    // Stage 1 - Startup Safety Gate
    //
    // 在任何 EtherCAT 設定寫入前先確認：
    //
    // Runtime Config
    //      vs
    // Physical EtherCAT Topology
    //
    // 若站數 / 站序 / Vendor / Product / Station Address
    // 任一不一致，直接中止初始化。
    // =========================================================

    if (!VerifyTopologyAgainstRuntimeConfig())
    {
        RtPrintf(
            "[ECAT-STARTUP] ABORTED | "
            "Reason:Topology / Identity verification failed.\n");

        return -2;
    }

    LARGE_INTEGER wait; wait.QuadPart = 5 * 10000;// 等待 單位ms


    uint16_t state_INIT = 0x0001;//INIT (初始化)	這才是您要找的代碼。通訊重置，無 Mailbox。
    uint16_t state_INIT_ClearError = 0x0010;//清除錯誤 0x0010
    uint16_t state_PRE_OP = 0x0002;//PRE-OP (預操作)	Mailbox 啟用，可以設定 SDO/PDO。
    uint16_t state_BOOT = 0x0003;//BOOT (啟動/燒錄)	韌體更新模式 (通常用不到)。
    uint16_t state_SAFE_OP = 0x0004;//SAFE-OP (安全操作)	輸入更新，輸出鎖定。
    uint16_t state_OP = 0x0008;//OP(操作)	全速運轉，輸入輸出都更新。


    // ---------------------------------------------------------
    //  關閉看門狗
    // ---------------------------------------------------------

    for (int i = 0; i < total_slaves; i++)
    {
        // 關閉看門狗 0x0000
        uint16_t watchdog_zero = 0x0000;
        WK = ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0410, 2, &watchdog_zero, 20);
        WK = ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0420, 2, &watchdog_zero, 20);
    }
    RtSleepFt(&wait);//等待


    // ---------------------------------------------------------
    //  預先清除錯誤狀態
    // ---------------------------------------------------------
    for (int i = 0; i < total_slaves; i++)
    {

        WK = ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0120, 2, &state_INIT_ClearError, 20);
        WK = ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0120, 2, &state_INIT, 20);
    }
    RtSleepFt(&wait);//等待
    Printf_Slaves_State();//印出從站狀態



   // =========================================================
// DC Topology Diagnostic
// =========================================================
    MeasureDCPortTimestamps();


    // =========================================================
    // DC Propagation Delay AUTO
    //
    // 依 BuildIoMap() 的 Servo 實體順序建立最多 8 軸 delay table，
    // 使用 10 次有效樣本 Median，成功後才逐站寫入並 ReadBack 0x0928。
    // AUTO Gate 失敗時不寫入任何 propagation-delay 值。
    // =========================================================

    DCAutoPropagationTable dcDelayTable = {};

    bool dcDelayMeasured =
        MeasureDCPropagationDelayAuto(
            dcDelayTable);

    if (dcDelayMeasured)
    {
        RtPrintf(
            "[DC-INIT-AUTO] Propagation Delay measured | "
            "Reference:S%d | ServoCount:%d\n",

            dcDelayTable.referenceSlaveIndex,
            dcDelayTable.count);

        bool dcDelayConfigured =
            ConfigureDCPropagationDelayAuto(
                dcDelayTable);

        if (!dcDelayConfigured)
        {
            RtPrintf(
                "[DC-INIT-AUTO] WARNING: "
                "Propagation Delay configuration FAILED.\n");
        }
    }
    else
    {
        RtPrintf(
            "[DC-INIT-AUTO] WARNING: "
            "Propagation Delay measurement FAILED. "
            "0x0928 was not changed by AUTO.\n");
    }


    // =========================================================
    // Existing DC initialization continues
    // =========================================================

    uint64_t base_master_time =
        GetCurrentMasterTimeNs();

    uint32_t cycle_time_ns =
        250000;


    // ---------------------------------------------------------
    //  INIT 狀態
    // ---------------------------------------------------------
   // 🌟 關鍵修改：在進入迴圈前，先計算一個「全域統一的 Start Time」

    //uint32_t cycle_time_ns = 250000; // 250us

    // 設定 50ms 的緩衝讓所有從站初始化完畢，並強制對齊 250us 的整數倍
    uint64_t unified_start_time = base_master_time + 50000000;
    unified_start_time = ((unified_start_time / cycle_time_ns) + 1) * cycle_time_ns;

    for (int i = 0; i < total_slaves; i++)
    {


        WK = ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0120, 2, &state_INIT, 20);


        // =====================================================
        // Stage 5D - Complete Runtime INIT Cutover
        //
        // Equivalence Gate OPEN:
        // - Runtime SM
        // - Runtime DC / Sync0
        //
        // Gate CLOSED:
        // - existing C++ hard-code fallback
        // =====================================================

        if (ShouldUseRuntimeConfiguration(
            i))
        {
            if (!ConfigureRuntimeInitStage(
                i,
                unified_start_time))
            {
                RtPrintf(
                    "[ECAT-STARTUP] ABORTED | "
                    "Reason:Runtime INIT configuration failed | "
                    "SlaveIndex:%d\n",

                    i);


                return -4;
            }
        }
        else
        {
            // 🌟 將計算好的 unified_start_time 傳遞進去

            ConfigureSlaveGeneric_INIT(i, unified_start_time);////SM配置 從站配置_INIT
        }
        //Sleep(1000);
    }
    RtSleepFt(&wait);//等待
    Printf_Slaves_State();//印出從站狀態
    // ----------------------------------------------------------------
    // PRE-OP
    // ----------------------------------------------------------------
    for (int i = 0; i < total_slaves; i++)
    {
        ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0120, 2, &state_PRE_OP, 20);

    }
    RtSleepFt(&wait);//等待
    Printf_Slaves_State();//印出從站狀態


    // =========================================================
    // Stage 6C - Runtime FMMU Preflight
    //
    // The complete Runtime FMMU plan is validated against the
    // actual Process Image layout BEFORE the first Runtime FMMU
    // register write.
    // =========================================================

    if (!PreflightRuntimeFmmuConfiguration())
    {
        RtPrintf(
            "[ECAT-STARTUP] ABORTED | "
            "Reason:Runtime FMMU preflight failed.\n");


        return
            -7;
    }


    //PRE-OP狀態下的設定
    for (int i = 0; i < total_slaves; i++)
    {
        // =====================================================
        // Stage 6C - Runtime FMMU Cutover
        //
        // New Runtime XML:
        //     use <Fmmus> -> APWR -> immediate APRD readback.
        //
        // Old Runtime XML:
        //     preserve legacy Config_Slave_FMMU().
        // =====================================================

        if (ShouldUseRuntimeFmmuConfiguration(
            i))
        {
            if (!ConfigureRuntimeFmmus(
                i))
            {
                RtPrintf(
                    "[ECAT-STARTUP] ABORTED | "
                    "Reason:Runtime FMMU configuration failed | "
                    "SlaveIndex:%d\n",

                    i);


                return
                    -8;
            }
        }
        else
        {
            Config_Slave_FMMU(i); // Legacy XML fallback
        }


        // =====================================================
        // Stage 5D - Complete Runtime PRE-OP Cutover
        //
        // Runtime path:
        // - Configurable PDO Mapping
        // - DC 1C32 / 1C33
        // - Watchdog
        // - PRE_OP InitCommands
        //
        // FMMU stays on the existing verified path.
        // =====================================================

        if (ShouldUseRuntimeConfiguration(
            i))
        {
            if (!ConfigureRuntimePreOpStage(
                i))
            {
                RtPrintf(
                    "[ECAT-STARTUP] ABORTED | "
                    "Reason:Runtime PRE-OP configuration failed | "
                    "SlaveIndex:%d\n",

                    i);


                return -5;
            }
        }
        else
        {
            ConfigureSlaveGeneric_PRE_OP(i);////從站配置_PRE_OP  

            //看門狗設定
            uint16_t div = 2498;// 1. 設定頻率：每 100us 跳一次
            ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0400, 2, &div, 20);

            //開啟看門狗
            // 設定 Process Data Watchdog (0x0420) 為 50 個 ticks 50 *100us = 5ms
            // 如果除頻器單位是 100us，設定 1000 就等於 100ms
            // 對於 250us 的通訊週期來說，100ms 是非常安全的緩衝區
            uint16_t wd_pd_time = 20000; // 十進位的 50
            ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0420, 2, &wd_pd_time, 20);

            //如果 PDI (通訊晶片內部) 的看門狗也想設為 5ms
            //uint16_t wd_pdi_time = 0x0032;
            //ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0410, 2, &wd_pdi_time, 20);
        }


    }


    // =========================================================
    // Stage 8B - PRE-OP Verification Safety-Gate Cutover
    //
    // Verification routing is now schema-driven.
    //
    // ALL Runtime slaves have an explicit Runtime Profile with
    // PdoMappingMode:
    //     -> Runtime generic verifier is the REAL safety gate.
    //
    // ZERO Runtime slaves have that schema:
    //     -> old Runtime XML compatibility:
    //        legacy ProductCode Stage-2 verifier remains fallback.
    //
    // PARTIAL Runtime schema:
    //     -> hard FAIL.
    //
    // We never mix Runtime-driven and ProductCode-driven
    // verification in one Startup.
    // =========================================================

    if (m_pEni ==
        nullptr)
    {
        RtPrintf(
            "[PREOP-VERIFY-ROUTE] "
            "Runtime Config is not attached | Result:FAIL\n");


        return
            -3;
    }


    const auto& runtimeVerifySlaves =
        m_pEni->GetSlaves();


    int runtimeVerifySchemaCount =
        0;


    for (const auto& slave :
        runtimeVerifySlaves)
    {
        if (slave.runtimeProfile.present &&
            slave.runtimeProfile.hasPdoMappingMode)
        {
            runtimeVerifySchemaCount++;
        }
    }


    if (!runtimeVerifySlaves.empty() &&
        runtimeVerifySchemaCount ==
        (int)runtimeVerifySlaves.size())
    {
        RtPrintf(
            "[PREOP-VERIFY-ROUTE] "
            "RuntimeSchema:%d/%u | "
            "Source:RUNTIME_XML | "
            "SafetyGate:GENERIC_STAGE8B | "
            "Result:SELECTED\n",

            runtimeVerifySchemaCount,

            (unsigned int)
            runtimeVerifySlaves.size());


        if (!AuditRuntimePreOpConfigurationGeneric())
        {
            RtPrintf(
                "[ECAT-STARTUP] ABORTED | "
                "Reason:Runtime generic PRE-OP verification failed.\n");


            return
                -3;
        }
    }
    else if (runtimeVerifySchemaCount ==
        0)
    {
        RtPrintf(
            "[PREOP-VERIFY-ROUTE] "
            "RuntimeSchema:0/%u | "
            "Source:LEGACY_PRODUCT_PROFILE | "
            "SafetyGate:LEGACY_STAGE2 | "
            "Result:SELECTED\n",

            (unsigned int)
            runtimeVerifySlaves.size());


        if (!VerifyPreOpSmAndPdoConfiguration())
        {
            RtPrintf(
                "[ECAT-STARTUP] ABORTED | "
                "Reason:Legacy PRE-OP SM / PDO verification failed.\n");


            return
                -3;
        }
    }
    else
    {
        RtPrintf(
            "[PREOP-VERIFY-ROUTE] "
            "RuntimeSchema:%d/%u | "
            "Partial Runtime verification schema is forbidden | "
            "Result:FAIL\n",

            runtimeVerifySchemaCount,

            (unsigned int)
            runtimeVerifySlaves.size());


        return
            -3;
    }


    Printf_Slaves_State();//印出從站狀態
    // ----------------------------------------------------------------
    // SAFE_OP
    // ----------------------------------------------------------------
    for (int i = 0; i < total_slaves; i++)
    {
        WK = ecx_APWR(m_slaveInfo[i].APRDAPWR_Addr, 0x0120, 2, &state_SAFE_OP, 20);

    }
    RtSleepFt(&wait);//等待
    Printf_Slaves_State();//印出從站狀態
     //SAFE_OP狀態下的設定
    for (int i = 0; i < total_slaves; i++)
    {
        // =====================================================
        // Stage 5D - Complete Runtime SAFE-OP Cutover
        //
        // Apply=1 + Transition=SAFE_OP commands are executed
        // from Runtime XML. Current example:
        // R1-EC8124D0 0x2002:01..04 = UINT16(1).
        // =====================================================

        if (ShouldUseRuntimeConfiguration(
            i))
        {
            if (!ConfigureRuntimeSafeOpStage(
                i))
            {
                RtPrintf(
                    "[ECAT-STARTUP] ABORTED | "
                    "Reason:Runtime SAFE-OP configuration failed | "
                    "SlaveIndex:%d\n",

                    i);


                return -6;
            }
        }
        else
        {
            ConfigureSlaveGeneric_SAFE_OP(i);////從站配置_SAFE_OP   
        }
    }



    Printf_Slaves_State();//印出從站狀態



    return 0;
}
void EtherCatMaster::Config_Slave_FMMU(int slaveIdx)//設定 FMMU(告訴 Slave 它的資料在地圖的哪裡)
{
    int fmmuCount = 0; // 用來計算現在用到第幾號 FMMU (0, 1...)

    LARGE_INTEGER wait; wait.QuadPart = 5 * 10000;// 等待 單位ms

    int WK = 0;
    //Servo (馬達)--------------------------------------
    for (auto& axis : m_ServoList)
    {
        if (axis.slaveIndex == slaveIdx)
        {
            switch (m_slaveInfo[slaveIdx].Vendor_ID)
            {
            case 0x0000066F://松下
            case 0xAAAA://上銀
            case 0x000001dd: // 台達 Delta
                switch (m_slaveInfo[slaveIdx].Product_Code)
                {
                case 0x60380000: //A6B
                case 0x00000005: //E1
                case 0x00006010: //A3E           
                    // 1. 設定 Output (RxPDO) -> FMMU 0
                    if (axis.pOutput != nullptr)
                    {
                        uint32_t logicalAddr = (uint32_t)((uint8_t*)axis.pOutput - (uint8_t*)m_IoMap);
                        uint16_t len = sizeof(ServoOutput); // 假設您有定義結構大小，或動態計算

                        // 馬達標準 RxPDO 位址通常是 0x1100 (CoE)
                        // 如果是特殊的馬達，可在這裡 switch(vendorID)
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x1180, 0x02, 20);
                        RtSleepFt(&wait);//等待


                    }

                    // 2. 設定 Input (TxPDO) -> FMMU 1
                    if (axis.pInput != nullptr)
                    {
                        uint32_t logicalAddr = (uint32_t)((uint8_t*)axis.pInput - (uint8_t*)m_IoMap);
                        uint16_t len = sizeof(ServoInput);

                        // 馬達標準 TxPDO 位址通常是 0x11C0 或 0x1180
                        // Delta A2/A3 常用 0x1180 或 0x11C0，視 SM3 設定而定
                        // 這裡暫定 0x11C0 (因為您之前 AD 模組也是用這個)
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x1480, 0x01, 20);
                        RtSleepFt(&wait);//等待

                    }
                    break;
                }
                break;
            }

            return;
        }
    }

    //AD模組--------------------------------------
    for (auto& ad : m_AdList)
    {
        if (ad.slaveIndex == slaveIdx)
        {
            // AD 通常只有 Input
            if (ad.pInputLoc != nullptr)
            {
                uint32_t logicalAddr = (uint32_t)((uint8_t*)ad.pInputLoc - (uint8_t*)m_IoMap);
                uint16_t len;
                switch (m_slaveInfo[slaveIdx].Vendor_ID)
                {
                case 0x000001dd: // 台達 Delta
                    switch (m_slaveInfo[slaveIdx].Product_Code)
                    {
                    case 0x00008124: //           
                        len = (uint16_t)(ad.channelValues.size() * 2); // 4CH * 2Bytes = 8
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x11C0, 0x01, 20);
                        RtSleepFt(&wait);//等待
                        break;
                    }
                    break;
                }


            }
            return;
        }
    }

    //IO 模組--------------------------------------
    for (auto& mod : m_IoList)
    {
        if (mod.slaveIndex == slaveIdx)
        {


            switch (m_slaveInfo[slaveIdx].Vendor_ID)
            {
            case 0x00278606://OSCARMAX
                switch (m_slaveInfo[slaveIdx].Product_Code)
                {
                case 0x00000003: //
                    // 1. 設定 Output
                    if (mod.pOutputLoc != nullptr)
                    {
                        uint32_t logicalAddr = (uint32_t)((uint8_t*)mod.pOutputLoc - (uint8_t*)m_IoMap);
                        uint16_t len = (uint16_t)mod.outBuffer.size();

                        // IO 模組 Output 通常是 0x0F00
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x0Fa0, 0x02, 20);
                        RtSleepFt(&wait);//等待
                    }

                    // 2. 設定 Input
                    if (mod.pInputLoc != nullptr)
                    {
                        uint32_t logicalAddr = (uint32_t)((uint8_t*)mod.pInputLoc - (uint8_t*)m_IoMap);
                        uint16_t len = (uint16_t)mod.inBuffer.size();

                        // IO 模組 Input 通常是 0x1000
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x0f80, 0x01, 20);
                        RtSleepFt(&wait);//等待
                    }
                    break;
                }
                break;
            case 0x000001dd: // 台達 Delta
                switch (m_slaveInfo[slaveIdx].Product_Code)
                {

                case 0x00006002: //           
                case 0x00007062: //   
                     // 1. 設定 Output
                    if (mod.pOutputLoc != nullptr)
                    {
                        uint32_t logicalAddr = (uint32_t)((uint8_t*)mod.pOutputLoc - (uint8_t*)m_IoMap);
                        uint16_t len = (uint16_t)mod.outBuffer.size();

                        // IO 模組 Output 通常是 0x0F00
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x0F00, 0x02, 20);
                        RtSleepFt(&wait);//等待
                    }

                    // 2. 設定 Input
                    if (mod.pInputLoc != nullptr)
                    {
                        uint32_t logicalAddr = (uint32_t)((uint8_t*)mod.pInputLoc - (uint8_t*)m_IoMap);
                        uint16_t len = (uint16_t)mod.inBuffer.size();

                        // IO 模組 Input 通常是 0x1000
                        WK = WriteFmmuRegister(slaveIdx, fmmuCount++, logicalAddr, len, 0x1000, 0x01, 20);
                        RtSleepFt(&wait);//等待
                    }
                    break;
                }
                break;
            }

            return;
        }
    }
}


void EtherCatMaster::ConfigureSlaveGeneric_INIT(int slaveIdx, uint64_t unifiedStartTime)//從站配置_INIT
{

    // =========================================================
    // Stage 5D.1
    //
    // Fixed IO SMs may already have been configured from Runtime XML
    // by Initialize_Slaves().
    //
    // Do not write the legacy per-ProductCode SM constants a second time.
    // =========================================================

    if (ShouldUseRuntimeSyncManagerConfiguration(
        slaveIdx))
    {
        return;
    }


    //0x0800 - 0x0807	SyncManager 0 (SM0)	Mailbox Output (MbxOut) 主站 -> 從站 (寫信)
    //0x0808 - 0x080F	SyncManager 1 (SM1)	Mailbox Input (MbxIn)   從站 -> 主站 (收信)
    //0x0810 - 0x0817	SyncManager 2 (SM2)	Process Data Output (RxPDO)
    //0x0818 - 0x081F	SyncManager 3 (SM3)	Process Data Input (TxPDO)

    int WK = 0;

    LARGE_INTEGER wait; wait.QuadPart = 5 * 10000;// 等待 ms



    uint8_t SyncManager_0[8];
    uint8_t SyncManager_1[8];
    uint8_t SyncManager_2[8];
    uint8_t SyncManager_3[8];



    //DC Set--------------------------------------------------------
    uint64_t slave_time_ns = 0;
    uint64_t time_offset = 0;
    uint64_t master_time_ns = GetCurrentMasterTimeNs();
    uint32_t cycle_time = 250000; // 250000 ns = 250 us
    uint64_t start_time = master_time_ns + 50000000;
    uint8_t DC_Enable_Value = 0x03;
    uint8_t DC_Disable_Value = 0x00;
    uint8_t DC_Cyclic_Control = 0x00;

    uint32_t shift_time;
    uint64_t final_start_time;

    uint64_t DC_zero_time = 0;

    uint16_t dcSpeedCounterStart = 0x1000;
    switch (m_slaveInfo[slaveIdx].Vendor_ID)
    {
    case 0x0000066F://松下
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x60380000: //A6B

            break;


        default:
            break;
        }
        break;
    case 0xAAAA://上銀
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00000005: //E1

            break;


        default:
            break;
        }
        break;
    case 0x00278606://OSCARMAX
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00000003: //
            SyncManager_0[1] = 0x0f;// 設定實體起始位址 (Physical Start Address)
            SyncManager_0[0] = 0x80;
            SyncManager_0[2] = 0x03;//設定記憶體長度 (Length)
            SyncManager_0[3] = 0x00;
            SyncManager_0[4] = 0x20;// Control (模式)
            SyncManager_0[5] = 0x00;//狀態暫存器 (Status Register) 寫入時通常為 0，由硬體自動更新
            SyncManager_0[6] = 0x01;//啟用 SyncManager (Activate) Bit 0 = 1 代表啟用 (Enable)
            SyncManager_0[7] = 0x00;//PDI 控制(PDI Control) 對於 Mailbox 通常設為 0

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0800, 8, SyncManager_0, 20); //
            RtSleepFt(&wait);//等待



            SyncManager_1[1] = 0x0f;//設定實體起始位址 (Physical Start Address)
            SyncManager_1[0] = 0xa0;
            SyncManager_1[2] = 0x01;//設定記憶體長度 (Length)
            SyncManager_1[3] = 0x00;
            SyncManager_1[4] = 0x64;//Control (模式)
            SyncManager_1[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_1[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_1[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0808, 8, SyncManager_1, 20);
            RtSleepFt(&wait);//等待


            SyncManager_2[1] = 0x0f;//設定實體起始位址 (Physical Start Address)
            SyncManager_2[0] = 0xa1;
            SyncManager_2[2] = 0x01;//設定記憶體長度 (Length)
            SyncManager_2[3] = 0x00;
            SyncManager_2[4] = 0x64;//Control (模式)
            SyncManager_2[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_2[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_2[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0810, 8, SyncManager_2, 20); // Outputs (Disable)
            RtSleepFt(&wait);//等待
            break;
        }
        break;
    case 0x000001dd://台達
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00006010: //A3E  


            //DC 同預先關閉------------------------------------------------------------------
            WK = ecx_FPWR(m_slaveInfo[slaveIdx].configAddr, 0x0920, &DC_zero_time, 8, 20);
            WK = ecx_FPWR(m_slaveInfo[slaveIdx].configAddr, 0x0910, &DC_zero_time, 8, 20);




            //0x0800 - 0x0807	SyncManager 0 (SM0)	Mailbox Output (MbxOut) 主站 -> 從站 (寫信)

            SyncManager_0[1] = 0x10;// 設定實體起始位址 (Physical Start Address)
            SyncManager_0[0] = 0x00;
            SyncManager_0[2] = 0x80;//設定記憶體長度 (Length)
            SyncManager_0[3] = 0x00;
            SyncManager_0[4] = 0x36;// Control (模式)
            SyncManager_0[5] = 0x00;//狀態暫存器 (Status Register) 寫入時通常為 0，由硬體自動更新
            SyncManager_0[6] = 0x01;//啟用 SyncManager (Activate) Bit 0 = 1 代表啟用 (Enable)
            SyncManager_0[7] = 0x00;//PDI 控制(PDI Control) 對於 Mailbox 通常設為 0

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0800, 8, SyncManager_0, 20); //
            RtSleepFt(&wait);//等待


               //0x0808 - 0x080F	SyncManager 1 (SM1)	Mailbox Input (MbxIn)   從站 -> 主站 (收信)

            SyncManager_1[1] = 0x10;//設定實體起始位址 (Physical Start Address)
            SyncManager_1[0] = 0xC0;
            SyncManager_1[2] = 0x80;//設定記憶體長度 (Length)
            SyncManager_1[3] = 0x00;
            SyncManager_1[4] = 0x32;//Control (模式)
            SyncManager_1[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_1[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_1[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0808, 8, SyncManager_1, 20);
            RtSleepFt(&wait);//等待


              //0x0810 - 0x0817	SyncManager 2 (SM2)	Process Data Output (RxPDO)
          // <Sm MinSize="1" MaxSize="256" DefaultSize="14" StartAddress="#x1180" ControlByte="#x24" Enable="1">Outputs</Sm>
            SyncManager_2[1] = 0x11;//設定實體起始位址 (Physical Start Address)
            SyncManager_2[0] = 0x80;
            SyncManager_2[2] = 0x09;//設定記憶體長度 (Length)
            SyncManager_2[3] = 0x00;


            //0x24	3 - Buffer + WD On	寫入模式，啟用看門狗。	正式運行建議值。安全性最高，通訊中斷即跳機。
            //0x20	3 - Buffer + WD Off	寫入模式，關閉看門狗。	除錯測試建議值。排除因微小 Jitter 導致的 0x001B 錯誤。
            //0x64	3 - Buffer + WD On + Int	寫入模式，啟用看門狗與中斷。	用於特定需要 PDI 中斷觸發的硬體設定。
            SyncManager_2[4] = 0x24;//Control (模式)   0x24 的意義：啟用 3-Buffer 模式並開啟 SM Watchdog。

            SyncManager_2[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_2[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_2[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0810, 8, SyncManager_2, 20); // Outputs (Disable)
            RtSleepFt(&wait);//等待



            //0x0818 - 0x081F	SyncManager 3 (SM3)	Process Data Input (TxPDO)
            //<Sm MinSize = "1" MaxSize = "256" DefaultSize = "22" StartAddress = "#x1480" ControlByte = "#x00" Enable = "1">Inputs< / Sm>
            SyncManager_3[1] = 0x14;//設定實體起始位址 (Physical Start Address)
            SyncManager_3[0] = 0x80;
            SyncManager_3[2] = 0x17;//設定記憶體長度 (Length)
            SyncManager_3[3] = 0x00;
            SyncManager_3[4] = 0x00;//Control (模式)
            SyncManager_3[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_3[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_3[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0818, 8, SyncManager_3, 20); // Inputs
            RtSleepFt(&wait);//等待



        // =========================================================
// DC Sync0 Configuration
//
// Delta A3E
//
// 0x0910 : DC System Time
// 0x0920 : System Time Offset
// 0x0980 : Cyclic Unit Control
// 0x0981 : Sync Activation
// 0x0990 : Sync0 Start Time
// 0x09A0 : Sync0 Cycle Time
// =========================================================


// ---------------------------------------------------------
// 1. 先停止 Sync0
//
// 0x0981 是 1 Byte Register。
// ---------------------------------------------------------
            DC_Disable_Value = 0x00;

            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x0981,
                &DC_Disable_Value,
                1,
                20);


            // ---------------------------------------------------------
            // 2. DC Cyclic Unit 交由 EtherCAT 控制
            //
            // 0x0980 = 0
            // ---------------------------------------------------------
            DC_Cyclic_Control = 0x00;

            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x0980,
                &DC_Cyclic_Control,
                1,
                20);


            // ---------------------------------------------------------
            // 3. 讀取目前 Slave DC System Time
            // ---------------------------------------------------------
            slave_time_ns = 0;

            WK = ecx_FPRD(
                m_slaveInfo[slaveIdx].configAddr,
                0x0910,
                &slave_time_ns,
                8,
                20);


            // ---------------------------------------------------------
            // 4. 將 Slave DC Time 對到目前 Master Time
            //
            // 這一階段先保留原本架構。
            // Reference Clock / Propagation Delay
            // 下一階段再正式整理。
            // ---------------------------------------------------------
            master_time_ns =
                GetCurrentMasterTimeNs();

            time_offset =
                master_time_ns -
                slave_time_ns;

            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x0920,
                &time_offset,
                8,
                20);
            // ---------------------------------------------------------
// DC Time Control Loop Reset
//
// 修改 System Time Offset (0x0920) 後，
// 必須重新初始化 DC 的：
//
// 1. System Time Difference Filter
// 2. Speed Counter Filter
//
// 0x0930 = Speed Counter Start
//
// Beckhoff ESC Default = 0x1000
// Valid Range = 0x0080 ~ 0x3FFF
//
// 寫入 0x0930 本身就會觸發 Filter Reset。
// ---------------------------------------------------------
            dcSpeedCounterStart = 0x1000;

            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x0930,
                &dcSpeedCounterStart,
                2,
                20);

            // ---------------------------------------------------------
            // 5. 非常重要
            //
            // 寫完 0x0920 後重新讀一次 0x0910。
            //
            // 後面的 Sync0 Start Time 必須以
            // 「目前真正的 DC System Time」為基準，
            // 不能繼續使用 Offset 前的舊時間。
            // ---------------------------------------------------------
            slave_time_ns = 0;

            WK = ecx_FPRD(
                m_slaveInfo[slaveIdx].configAddr,
                0x0910,
                &slave_time_ns,
                8,
                20);


            // ---------------------------------------------------------
            // 6. Sync0 Cycle = 250 us
            // ---------------------------------------------------------
            cycle_time =
                250000;


            // ---------------------------------------------------------
            // 7. 計算第一個 Sync0 Start Time
            //
            // 使用 Slave 自己目前的 DC System Time。
            //
            // 預留 100 ms，避免 Start Time 在設定完成以前
            // 就已經變成 Past Time。
            //
            // Shift 先維持你原本的 1/2 Cycle：125 us。
            // Master PDO Phase 下一階段再處理。
            // ---------------------------------------------------------
            shift_time =
                cycle_time / 2;

            start_time =
                slave_time_ns +
                100000000ULL;

            start_time =
                ((start_time / cycle_time) + 1) *
                cycle_time;

            start_time +=
                shift_time;


            // ---------------------------------------------------------
            // 8. 寫入 Sync0 Start Time
            // ---------------------------------------------------------
            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x0990,
                &start_time,
                8,
                20);


            // ---------------------------------------------------------
            // 9. 寫入 Sync0 Cycle Time
            // ---------------------------------------------------------
            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x09A0,
                &cycle_time,
                4,
                20);


            // ---------------------------------------------------------
            // 10. 啟用 Cyclic Unit + Sync0
            //
            // Bit 0 = Cyclic Unit
            // Bit 1 = Sync0
            //
            // 0x03
            // ---------------------------------------------------------
            DC_Enable_Value =
                0x03;

            WK = ecx_FPWR(
                m_slaveInfo[slaveIdx].configAddr,
                0x0981,
                &DC_Enable_Value,
                1,
                20);

            break;
        case 0x00005500: // Delta R1-EC5500 (Coupler)

            break;
        case 0x00006002: // Delta R1-EC6002 (Digital Input)
        {
            //0x0800 - 0x0807	SyncManager 0 (SM0)	Mailbox Output (MbxOut) 主站 -> 從站 (寫信)

            SyncManager_0[1] = 0x10;// 設定實體起始位址 (Physical Start Address)
            SyncManager_0[0] = 0x00;
            SyncManager_0[2] = 0x02;//設定記憶體長度 (Length)
            SyncManager_0[3] = 0x00;
            SyncManager_0[4] = 0x00;// 控制暫存器 (Control Register) 設定運作模式 (Buffered mode / Mailbox mode)   
            SyncManager_0[5] = 0x00;//狀態暫存器 (Status Register) 寫入時通常為 0，由硬體自動更新
            SyncManager_0[6] = 0x01;//啟用 SyncManager (Activate) Bit 0 = 1 代表啟用 (Enable)
            SyncManager_0[7] = 0x00;//PDI 控制(PDI Control) 對於 Mailbox 通常設為 0

            uint8_t sm0_cfg[8] = { 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00 };
            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0800, 8, SyncManager_0, 20);
        }
        break;

        case 0x00007062: // Delta R1-EC7062 (Digital Output)
        {



            //0x0800 - 0x0807	SyncManager 0 (SM0)	Mailbox Output (MbxOut) 主站 -> 從站 (寫信)

            SyncManager_0[1] = 0x0F;// 設定實體起始位址 (Physical Start Address)
            SyncManager_0[0] = 0x00;
            SyncManager_0[2] = 0x01;//設定記憶體長度 (Length)
            SyncManager_0[3] = 0x00;
            SyncManager_0[4] = 0x44;// Control (模式)
            SyncManager_0[5] = 0x00;//狀態暫存器 (Status Register) 寫入時通常為 0，由硬體自動更新
            SyncManager_0[6] = 0x01;//啟用 SyncManager (Activate) Bit 0 = 1 代表啟用 (Enable)
            SyncManager_0[7] = 0x00;//PDI 控制(PDI Control) 對於 Mailbox 通常設為 0



            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0800, 8, SyncManager_0, 20);
            RtSleepFt(&wait);//等待

            //0x0808 - 0x080F	SyncManager 1 (SM1)	Mailbox Input (MbxIn)   從站 -> 主站 (收信)

            SyncManager_1[1] = 0x0F;//設定實體起始位址 (Physical Start Address)
            SyncManager_1[0] = 0x01;
            SyncManager_1[2] = 0x01;//設定記憶體長度 (Length)
            SyncManager_1[3] = 0x00;
            SyncManager_1[4] = 0x44;//Control (模式)
            SyncManager_1[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_1[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_1[7] = 0x00;//PDI 控制 (PDI Control)
            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0808, 8, SyncManager_1, 20);
            RtSleepFt(&wait);//等待

        }
        break;

        case 0x00008124: // Delta R1-EC8124
        {
            //0x0800 - 0x0807	SyncManager 0 (SM0)	Mailbox Output (MbxOut) 主站 -> 從站 (寫信)

            SyncManager_0[1] = 0x10;// 設定實體起始位址 (Physical Start Address)
            SyncManager_0[0] = 0x00;
            SyncManager_0[2] = 0x80;//設定記憶體長度 (Length)
            SyncManager_0[3] = 0x00;
            SyncManager_0[4] = 0x26;// Control (模式)
            SyncManager_0[5] = 0x00;//狀態暫存器 (Status Register) 寫入時通常為 0，由硬體自動更新
            SyncManager_0[6] = 0x01;//啟用 SyncManager (Activate) Bit 0 = 1 代表啟用 (Enable)
            SyncManager_0[7] = 0x00;//PDI 控制(PDI Control) 對於 Mailbox 通常設為 0

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0800, 8, SyncManager_0, 20); //
            RtSleepFt(&wait);//等待

             //0x0808 - 0x080F	SyncManager 1 (SM1)	Mailbox Input (MbxIn)   從站 -> 主站 (收信)

            SyncManager_1[1] = 0x10;//設定實體起始位址 (Physical Start Address)
            SyncManager_1[0] = 0x80;
            SyncManager_1[2] = 0x80;//設定記憶體長度 (Length)
            SyncManager_1[3] = 0x00;
            SyncManager_1[4] = 0x22;//Control (模式)
            SyncManager_1[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_1[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_1[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0808, 8, SyncManager_1, 20);
            RtSleepFt(&wait);//等待

            //0x0810 - 0x0817	SyncManager 2 (SM2)	Process Data Output (RxPDO)

            SyncManager_2[1] = 0x11;//設定實體起始位址 (Physical Start Address)
            SyncManager_2[0] = 0x00;
            SyncManager_2[2] = 0x00;//設定記憶體長度 (Length)
            SyncManager_2[3] = 0x00;
            SyncManager_2[4] = 0x24;//Control (模式)
            SyncManager_2[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_2[6] = 0x00;//啟用 SyncManager (Activate)
            SyncManager_2[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0810, 8, SyncManager_2, 20); // Outputs (Disable)
            RtSleepFt(&wait);//等待



            //0x0818 - 0x081F	SyncManager 3 (SM3)	Process Data Input (TxPDO)

            SyncManager_3[1] = 0x11;//設定實體起始位址 (Physical Start Address)
            SyncManager_3[0] = 0xC0;
            SyncManager_3[2] = 0x08;//設定記憶體長度 (Length)
            SyncManager_3[3] = 0x00;
            SyncManager_3[4] = 0x20;//Control (模式)
            SyncManager_3[5] = 0x00;//狀態暫存器 (Status Register)
            SyncManager_3[6] = 0x01;//啟用 SyncManager (Activate)
            SyncManager_3[7] = 0x00;//PDI 控制 (PDI Control)

            WK = ecx_APWR(m_slaveInfo[slaveIdx].APRDAPWR_Addr, 0x0818, 8, SyncManager_3, 20); // Inputs
            RtSleepFt(&wait);//等待


        }
        break;
        default:
            return;
        }
        break;
    default:
        return;
    }



}

// 取得 RTX64 系統時間 (單位: 奈秒)
uint64_t  EtherCatMaster::GetCurrentMasterTimeNs()
{
    LARGE_INTEGER current_time;

    // RTX64 API: RtGetClockTime
    // 參數 1: CLOCK_2 (代表 Monotonic Clock，單調遞增時間)
    // 參數 2: PLARGE_INTEGER (接收時間的變數位址)
    // 注意：RTX64 回傳的單位通常是 "100 奈秒" (100ns ticks)
    RtGetClockTime(CLOCK_2, &current_time);

    // 將 100ns 單位轉換為 1ns 單位 (乘上 100)
    // 1 tick = 100 ns
    // 10 ticks = 1000 ns = 1 us
    return (uint64_t)current_time.QuadPart * 100;
}
void EtherCatMaster::ConfigureSlaveGeneric_PRE_OP(int slaveIdx)//從站配置_PRE_OP
{

    LARGE_INTEGER wait; wait.QuadPart = 300 * 10000;// 等待 300ms
    uint8_t  u8val = 0;
    uint16_t u16val = 0;
    uint32_t u32val = 0;
    uint32_t DC_cycleTimeNs = 1000000; // 1ms (與您的 RTX Timer 一致)
    uint16_t DC_syncMode = 2;          // DC Sync0 Mode
    uint16_t DC_freeRunMode = 0; // 0 = Free Run


    int WK = 0;

    switch (m_slaveInfo[slaveIdx].Vendor_ID)
    {
    case 0x0000066F://松下
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x60380000: //A6B


            break;


        default:
            break;
        }
        break;
    case 0xAAAA://上銀
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00000005: //E1

            break;


        default:
            break;
        }
        break;
    case 0x000001dd://台達
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00006010: //A3E  

            //停用 PDO 分配
            u8val = 0;
            WK = ecx_SDOwrite(slaveIdx, 0x1C12, 0x00, FALSE, 1, &u8val, 20);
            WK = ecx_SDOwrite(slaveIdx, 0x1C13, 0x00, FALSE, 1, &u8val, 20);

            //停用 PDO 映射 (Mapping)
            u8val = 0;
            WK = ecx_SDOwrite(slaveIdx, 0x1601, 0x00, FALSE, 1, &u8val, 20);
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x00, FALSE, 1, &u8val, 20);

            //設定 RxPDO (0x1601)----------------------------------------------------------------------
            // Sub1: 6040h (Control Word) 16-bit
            u32val = 0x60400010;
            WK = ecx_SDOwrite(slaveIdx, 0x1601, 0x01, FALSE, 4, &u32val, 20);

            // Sub2: 60FFh (Target Velocity) 32-bit
            u32val = 0x60FF0020;
            WK = ecx_SDOwrite(slaveIdx, 0x1601, 0x02, FALSE, 4, &u32val, 20);

            // Sub3: 60B8h (Touch Probe Func) 16-bit
            u32val = 0x60B80010;
            WK = ecx_SDOwrite(slaveIdx, 0x1601, 0x03, FALSE, 4, &u32val, 20);

            // Sub4: 6060h (Modes of Operation) 8-bit [新增]
            // 這是 8-bit 整數，所以結尾是 08
            u32val = 0x60600008;
            WK = ecx_SDOwrite(slaveIdx, 0x1601, 0x04, FALSE, 4, &u32val, 20);




            // 設定 RxPDO 映射數目 = 4
            u8val = 4;
            WK = ecx_SDOwrite(slaveIdx, 0x1601, 0x00, FALSE, 1, &u8val, 20);

            //設定 TxPDO(0x1A01)----------------------------------------------------------------------

            // Sub1: 6041h (StatusWord) 16-bit
            u32val = 0x60410010;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x01, FALSE, 4, &u32val, 20);

            // Sub2: 6064h (Actual Position) 32-bit
            u32val = 0x60640020;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x02, FALSE, 4, &u32val, 20);

            // Sub3: 606Ch (Velocity Actual) 32-bit
            u32val = 0x606C0020;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x03, FALSE, 4, &u32val, 20);

            // Sub4: 6077h (Torque Actual) 16-bit
            u32val = 0x60770010;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x04, FALSE, 4, &u32val, 20);

            // Sub5: 60B9h (Touch Probe Status) 16-bit [修改] 
            // 圖片中是 0x60B90010，不是 ErrorCode
            u32val = 0x60B90010;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x05, FALSE, 4, &u32val, 20);

            // Sub6: 60BAh (Touch Probe Pos1) 32-bit
            u32val = 0x60BA0020;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x06, FALSE, 4, &u32val, 20);

            // Sub7: 6061h (Modes of Operation Display) 8-bit [修改]
            // 注意：這裡是 8-bit，所以結尾是 08
            u32val = 0x60610008;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x07, FALSE, 4, &u32val, 20);


            // Sub8: 2510h (Vendor Specific) 32-bit [新增]
            // 圖片中最後一個是 0x25100020
            u32val = 0x25100020;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x08, FALSE, 4, &u32val, 20);


            // 設定 TxPDO 映射數目
            u8val = 8;
            WK = ecx_SDOwrite(slaveIdx, 0x1A01, 0x00, FALSE, 1, &u8val, 20);

            //設定 PDO 分配----------------------------------------------------------------------
            // RxPDO Assign (1C12) -> 指向 0x1601
            u16val = 0x1601;
            WK = ecx_SDOwrite(slaveIdx, 0x1C12, 0x01, FALSE, 2, &u16val, 20);

            // TxPDO Assign (1C13) -> 指向 0x1A01
            u16val = 0x1A01;
            WK = ecx_SDOwrite(slaveIdx, 0x1C13, 0x01, FALSE, 2, &u16val, 20);

            //啟用 PDO 配置----------------------------------------------------------------------
            u8val = 1;
            WK = ecx_SDOwrite(slaveIdx, 0x1C12, 0x00, FALSE, 1, &u8val, 20);
            WK = ecx_SDOwrite(slaveIdx, 0x1C13, 0x00, FALSE, 1, &u8val, 20);


            //關閉DC同步---------------------------------------------------------------------------
            DC_freeRunMode = 0; // 0 = Free Run

            // 1. SDO 層面：將同步模式設為 0 (Free Run)
            ecx_SDOwrite(slaveIdx, 0x1C32, 0x01, FALSE, 2, &DC_freeRunMode, 20);
            ecx_SDOwrite(slaveIdx, 0x1C33, 0x01, FALSE, 2, &DC_freeRunMode, 20);


            //開啟DC 同步 設定 DC Sync 參數 1C32 & 1C33--------------------------------------------------------


            DC_cycleTimeNs = 250000; // 250us (與您的 RTX Timer 一致)
            DC_syncMode = 2;          // DC Sync0 Mode

            //設定 Output Sync (RxPDO) - 0x1C32
            // Sub 1: Sync Mode = 2 (DC Sync0)
            WK = ecx_SDOwrite(slaveIdx, 0x1C32, 0x01, FALSE, 2, &DC_syncMode, 20);

            // Sub A: Cycle Time = 1,000,000 ns
            WK = ecx_SDOwrite(slaveIdx, 0x1C32, 0x0A, FALSE, 4, &DC_cycleTimeNs, 20);


            // 設定 Input Sync (TxPDO) - 0x1C33 
            // Sub 1: Sync Mode = 2 (DC Sync0)
            WK = ecx_SDOwrite(slaveIdx, 0x1C33, 0x01, FALSE, 2, &DC_syncMode, 20);

            // Sub A: Cycle Time = 1,000,000 ns
            WK = ecx_SDOwrite(slaveIdx, 0x1C33, 0x0A, FALSE, 4, &DC_cycleTimeNs, 20);
            break;
        case 0x00005500: // Delta R1-EC5500 (Coupler)

            break;
        case 0x00006002: // Delta R1-EC6002 (Digital Input)
        {

        }
        break;

        case 0x00007062: // Delta R1-EC7062 (Digital Output)
        {

        }
        break;

        case 0x00008124: // Delta R1-EC8124
        {

        }
        break;
        default:
            return;
        }
        break;
    default:
        return;
    }
}
void EtherCatMaster::ConfigureSlaveGeneric_SAFE_OP(int slaveIdx)//從站配置_SAFE_OP
{

    LARGE_INTEGER wait; wait.QuadPart = 300 * 10000;// 等待 300ms
    uint8_t  u8val = 0;
    uint16_t u16val = 0;
    uint32_t u32val = 0;
    int WK = 0;
    switch (m_slaveInfo[slaveIdx].Vendor_ID)
    {
    case 0x0000066F://松下
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x60380000: //A6B


            break;


        default:
            break;
        }
        break;
    case 0xAAAA://上銀
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00000005: //E1

            break;


        default:
            break;
        }
        break;
    case 0x000001dd://台達
        switch (m_slaveInfo[slaveIdx].Product_Code)
        {
        case 0x00006010: //A3E  


            break;
        case 0x00005500: // Delta R1-EC5500 (Coupler)

            break;
        case 0x00006002: // Delta R1-EC6002 (Digital Input)
        {

        }
        break;

        case 0x00007062: // Delta R1-EC7062 (Digital Output)
        {

        }
        break;

        case 0x00008124: // Delta R1-EC8124
        {
            //8124 4個channel 
            uint16_t R1_EC8124_Channel_Enable[4];
            R1_EC8124_Channel_Enable[0] = 1;
            R1_EC8124_Channel_Enable[1] = 1;
            R1_EC8124_Channel_Enable[2] = 1;
            R1_EC8124_Channel_Enable[3] = 1;
            for (uint8_t ch = 1; ch <= 4; ch++)
            {
                WK = ecx_SDOwrite(slaveIdx, 0x2002, ch, 0, 2, &R1_EC8124_Channel_Enable[ch - 1], 20);
                RtSleepFt(&wait);//等待
            }
        }
        break;
        default:
            return;
        }
        break;
    default:
        return;
    }
}
int EtherCatMaster::BuildIoMap() //// 自動掃描並建立清單
{

    // =========================================================
    // Stage 3 - Process Image / Frame Preflight Safety Guard
    //
    // 重要：
    //
    // 所有容量檢查都在真正建立任何 m_IoMap Pointer 之前完成。
    //
    // 因此如果 Runtime XML 異常、裝置數量過多、Process Image
    // 超過固定 Buffer，會直接 Fail，不會先寫越界才發現。
    // =========================================================

    if (m_pEni == nullptr)
    {
        DEBUG_PRINT(
            "[PROCESS-IMAGE-GUARD] ERROR: ENI / Runtime Config is not attached.\n");

        return -1;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    uint64_t totalOutputBytes =
        0;


    uint64_t totalInputBytes =
        0;


    bool servoLayoutPass =
        true;


    for (int i = 0;
        i < (int)slaves.size();
        i++)
    {
        const auto& slave =
            slaves[(size_t)i];


        const uint64_t outBytes =
            ((uint64_t)slave.outputBitLength + 7ULL) /
            8ULL;


        const uint64_t inBytes =
            ((uint64_t)slave.inputBitLength + 7ULL) /
            8ULL;


        totalOutputBytes +=
            outBytes;


        totalInputBytes +=
            inBytes;


        // -----------------------------------------------------
        // Current Servo Runtime ABI Guard
        //
        // Type == "Servo" 目前會直接 cast 到：
        //
        // ServoOutput / ServoInput
        //
        // 因此 Runtime XML 的 PDO Byte Size 必須與 C++ struct 完全一致。
        // -----------------------------------------------------

        if (strcmp(slave.type, "Servo") == 0)
        {
            const bool outputMatch =
                outBytes ==
                sizeof(ServoOutput);


            const bool inputMatch =
                inBytes ==
                sizeof(ServoInput);


            const bool pass =
                outputMatch &&
                inputMatch;


            DEBUG_PRINT(
                "[SERVO-PDO-SIZE-GUARD] S%d | "
                "XML Out:%llu B Struct:%u B %s | "
                "XML In:%llu B Struct:%u B %s | "
                "Result:%s\n",

                i,

                (unsigned long long)
                outBytes,

                (unsigned int)
                sizeof(ServoOutput),

                outputMatch
                ? "MATCH"
                : "MISMATCH",

                (unsigned long long)
                inBytes,

                (unsigned int)
                sizeof(ServoInput),

                inputMatch
                ? "MATCH"
                : "MISMATCH",

                pass
                ? "PASS"
                : "FAIL");


            if (!pass)
            {
                servoLayoutPass =
                    false;
            }
        }
    }


    const uint64_t totalProcessImageBytes =
        totalOutputBytes +
        totalInputBytes;


    const uint64_t ioMapCapacity =
        sizeof(m_IoMap);


    // ---------------------------------------------------------
    // Current Combined LRW + FRMW Frame Estimation
    //
    // 目前 Configurator 與 Runtime 使用同一估算：
    //
    // Ethernet + EtherCAT + LRW + FRMW fixed overhead = 48 B
    //
    // Current NIC TX Buffer = 1514 B.
    //
    // 這是 Stage 3 的安全上限，不是用來取代實際 Frame Builder
    // 最後的 Length Guard；之後 Stage 3B 會在送出前再做一次 final guard。
    // ---------------------------------------------------------

    const uint64_t combinedFrameFixedOverheadBytes =
        48ULL;


    const uint64_t estimatedFrameBytes =
        totalProcessImageBytes +
        combinedFrameFixedOverheadBytes;


    const uint64_t txFrameLimit =
        sizeof(m_txBuffer);


    const uint64_t rxFrameLimit =
        sizeof(m_rxBuffer);


    const bool processImagePass =
        totalProcessImageBytes <=
        ioMapCapacity;


    const bool txFramePass =
        estimatedFrameBytes <=
        txFrameLimit;


    const bool rxFramePass =
        estimatedFrameBytes <=
        rxFrameLimit;


    const bool preflightPass =
        processImagePass &&
        txFramePass &&
        rxFramePass &&
        servoLayoutPass;


    const uint64_t frameMargin =
        estimatedFrameBytes <= txFrameLimit
        ? txFrameLimit - estimatedFrameBytes
        : 0ULL;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PROCESS-IMAGE-GUARD] BEGIN | Stage:3\n"
        "============================================================\n");


    DEBUG_PRINT(
        "[PROCESS-IMAGE-GUARD] "
        "Out:%llu B | In:%llu B | Total:%llu / %llu B | "
        "Result:%s\n",

        (unsigned long long)
        totalOutputBytes,

        (unsigned long long)
        totalInputBytes,

        (unsigned long long)
        totalProcessImageBytes,

        (unsigned long long)
        ioMapCapacity,

        processImagePass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "[FRAME-GUARD] "
        "Estimated:%llu B | FixedOverhead:%llu B | "
        "TXLimit:%llu B | RXLimit:%llu B | Margin:%llu B | "
        "Result:%s\n",

        (unsigned long long)
        estimatedFrameBytes,

        (unsigned long long)
        combinedFrameFixedOverheadBytes,

        (unsigned long long)
        txFrameLimit,

        (unsigned long long)
        rxFrameLimit,

        (unsigned long long)
        frameMargin,

        txFramePass &&
        rxFramePass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "[PROCESS-IMAGE-GUARD-RESULT] "
        "ProcessImage:%s | Frame:%s | ServoPDO:%s | Result:%s\n",

        processImagePass
        ? "PASS"
        : "FAIL",

        txFramePass &&
        rxFramePass
        ? "PASS"
        : "FAIL",

        servoLayoutPass
        ? "PASS"
        : "FAIL",

        preflightPass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[PROCESS-IMAGE-GUARD] END | Result:%s\n"
        "============================================================\n\n",

        preflightPass
        ? "PASS"
        : "FAIL");


    if (!preflightPass)
    {
        DEBUG_PRINT(
            "[Map] ABORTED: Process Image / Frame / Servo PDO preflight failed.\n");

        return -1;
    }


    // =========================================================
    // Stage 7B - Runtime Process Image Binding Cutover Gate
    //
    // 0 Runtime binding sections:
    //     legacy XML -> preserve existing ProductCode fallback.
    //
    // ALL Runtime slaves have binding sections:
    //     Runtime XML becomes authoritative for application
    //     classification and m_IoMap pointer binding.
    //
    // Partial schema:
    //     hard failure. Never mix Runtime and legacy classification.
    // =========================================================

    int runtimeBindingPresentCount =
        0;


    for (const auto& slave :
        slaves)
    {
        if (slave.runtimeProcessImageBinding.present)
        {
            runtimeBindingPresentCount++;
        }
    }


    if (runtimeBindingPresentCount ==
        (int)slaves.size())
    {
        DEBUG_PRINT(
            "[RUNTIME-BINDING-ROUTE] "
            "Schema:%d/%u | Source:RUNTIME_XML | Result:SELECTED\n",

            runtimeBindingPresentCount,

            (unsigned int)
            slaves.size());


        return
            BuildRuntimeProcessImageBindings();
    }


    if (runtimeBindingPresentCount !=
        0)
    {
        DEBUG_PRINT(
            "[RUNTIME-BINDING-ROUTE] "
            "Schema:%d/%u | Partial schema is forbidden | Result:FAIL\n",

            runtimeBindingPresentCount,

            (unsigned int)
            slaves.size());


        return
            -1;
    }


    DEBUG_PRINT(
        "[RUNTIME-BINDING-ROUTE] "
        "Schema:0/%u | Source:LEGACY_PRODUCT_FALLBACK | Result:SELECTED\n",

        (unsigned int)
        slaves.size());


    // =========================================================
    // Legacy BuildIoMap fallback
    //
    // Kept unchanged for old Runtime XML with no binding schema.
    // =========================================================

    // 1. 初始化
    m_IoList.clear();
    m_AdList.clear();
    m_ServoList.clear();

    // 清空 IO Map 記憶體
    memset(m_IoMap, 0, sizeof(m_IoMap));

    int currentOffset = 0;

    for (int i = 0; i < slaves.size(); i++)
    {
        const auto& slave = slaves[i];
        std::string Type_s = slaves[i].type;
        int outBytes = (slave.outputBitLength + 7) / 8;
        int inBytes = (slave.inputBitLength + 7) / 8;

        if (Type_s == "Servo")//伺服馬達--------------------------------------------
        {
            ENI_ServoDrive newAxis;
            newAxis.slaveIndex = i;
            newAxis.vendorId = slave.vendorId;
            newAxis.productCode = slave.productCode;

            // 設定 Output 指標 (若有)
            if (outBytes > 0)
            {
                newAxis.pOutput = (ServoOutput*)&m_IoMap[currentOffset];
                currentOffset += outBytes;
            }
            else
            {
                newAxis.pOutput = nullptr;
            }

            // 設定 Input 指標 (若有)
            if (inBytes > 0)
            {
                newAxis.pInput = (ServoInput*)&m_IoMap[currentOffset];
                currentOffset += inBytes;
            }
            else
            {
                newAxis.pInput = nullptr;
            }
            m_ServoList.push_back(newAxis);
            DEBUG_PRINT("[Map] Servo (Idx:%d) Out:%d bytes, In:%d bytes\n", i, outBytes, inBytes);
        }
        else if (Type_s == "IO")//IO 模組----------------------------------------------------------
        {
            ENI_GenericIO mod;
            ENI_AnalogModule ad;
            switch (slaves[i].vendorId)
            {
            case 0x000001dd: // Delta
            case 0x00278606://OSCARMAX
                switch (slaves[i].productCode)
                {
                    //OSCARMAX
                case 0x3:

                    // Delta
                case 0x00005500:
                case 0x00006002:
                case 0x00007062:
                    mod.slaveIndex = i;
                    mod.vendorId = slave.vendorId;
                    mod.productCode = slave.productCode;
                    mod.pOutputLoc = nullptr;
                    mod.pInputLoc = nullptr;

                    // 設定 Output
                    if (outBytes > 0)
                    {
                        mod.pOutputLoc = (void*)&m_IoMap[currentOffset];
                        mod.outBuffer.resize(outBytes, 0);
                        currentOffset += outBytes;
                    }

                    // 設定 Input
                    if (inBytes > 0)
                    {
                        mod.pInputLoc = (void*)&m_IoMap[currentOffset];
                        mod.inBuffer.resize(inBytes, 0);
                        currentOffset += inBytes;
                    }

                    m_IoList.push_back(mod);
                    DEBUG_PRINT("[Map] Gen IO (Idx:%d) Out:%d, In:%d\n", i, outBytes, inBytes);
                    break;

                case 0x00008124:
                    ad.slaveIndex = i;
                    ad.vendorId = slave.vendorId;
                    ad.productCode = slave.productCode;
                    ad.pInputLoc = nullptr;

                    // AD 只有 Input
                    if (inBytes > 0)
                    {
                        ad.pInputLoc = (void*)&m_IoMap[currentOffset];
                        ad.channelValues.resize(inBytes / 2, 0); // 假設每個通道 2 bytes
                        currentOffset += inBytes;
                    }

                    m_AdList.push_back(ad);
                    DEBUG_PRINT("[Map] Delta AD 8124 (Idx:%d)\n", i);
                    break;
                default:
                    DEBUG_PRINT("[Warning] Unknown Device (Idx:%d)\n", i);
                    return -1;
                }
                break;
            default:
                DEBUG_PRINT("[Warning] Unknown Device (Idx:%d)\n", i);
                return -1;

            }

        }
        else//未知設備-------------------------------------
        {
            DEBUG_PRINT("[Warning] Unknown Device (Idx:%d)\n", i);
            return -1;
        }
    }


    // =========================================================
    // Stage 3 Post-Build Consistency Check
    //
    // Preflight 計算的 Process Image 與實際 Mapping Offset
    // 必須完全一致。
    // =========================================================

    if ((uint64_t)currentOffset !=
        totalProcessImageBytes)
    {
        DEBUG_PRINT(
            "[PROCESS-IMAGE-GUARD] ERROR: "
            "Post-build mapping size mismatch | "
            "Preflight:%llu B Actual:%d B\n",

            (unsigned long long)
            totalProcessImageBytes,

            currentOffset);

        m_IoList.clear();
        m_AdList.clear();
        m_ServoList.clear();

        memset(
            m_IoMap,
            0,
            sizeof(m_IoMap));

        m_IoMapSize =
            0;

        return -1;
    }


    m_IoMapSize = currentOffset;
    // 🌟 1. 將 IO 和 AD 清單都傳給 PlcCore
    m_Plc.SetIoLists(&m_IoList, &m_AdList);

    // 🌟 2. 觸發一鍵自動排列 (Auto Mapping)
    m_Plc.AutoMapIO();

    DEBUG_PRINT(">>> Mapping Done. Total Map Size: %d Bytes\n", m_IoMapSize);
    return 0;
}
bool EtherCatMaster::Get_I(int ioListIdx, int bitIdx)
{
    if (ioListIdx < 0 || ioListIdx >= m_IoList.size()) return false;

    // 檢查 Buffer 是否有資料
    if (m_IoList[ioListIdx].inBuffer.empty()) return false;

    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;

    if (bytePos >= m_IoList[ioListIdx].inBuffer.size()) return false;

    // 從 inBuffer 讀取
    return (m_IoList[ioListIdx].inBuffer[bytePos] & (1 << bitPos)) != 0;
}
bool EtherCatMaster::Get_O(int moduleIdx, int bitIdx)
{
    // 1. 安全檢查
    if (moduleIdx < 0 || moduleIdx >= m_IoList.size()) return false;

    // 如果這張卡沒有輸出 Buffer，直接回傳 false
    if (m_IoList[moduleIdx].outBuffer.empty()) return false;

    // 2. 計算位置
    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;

    // 3. 檢查是否超出範圍
    if (bytePos >= m_IoList[moduleIdx].outBuffer.size()) return false;

    // 4. 讀取 Bit (檢查該位元是否為 1)
    return (m_IoList[moduleIdx].outBuffer[bytePos] & (1 << bitPos)) != 0;
}
void EtherCatMaster::Set_O(int ioListIdx, int bitIdx, bool val)
{
    if (ioListIdx < 0 || ioListIdx >= m_IoList.size()) return;

    // 檢查 Buffer 是否存在
    if (m_IoList[ioListIdx].outBuffer.empty()) return;

    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;

    if (bytePos >= m_IoList[ioListIdx].outBuffer.size()) return;

    // 寫入 outBuffer
    if (val)
        m_IoList[ioListIdx].outBuffer[bytePos] |= (1 << bitPos); // Set
    else
        m_IoList[ioListIdx].outBuffer[bytePos] &= ~(1 << bitPos); // Clear
}

void EtherCatMaster::LinkCoordinateManager(CoordinateManager* pCoord)
{
    pCoordMgr = pCoord;
}



