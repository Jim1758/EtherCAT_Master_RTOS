#include "EtherCatMaster_DC_Topology.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h"
#include "ConfigReader.h"
#include <windows.h>
#include <rtapi.h>
#include <rtssapi.h>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

// ============================================================================
// EtherCatMaster_DC_Topology.cpp
// DC Reference 選擇與 AUTO Topology Dry-Run V1
//
// SystemConfig.txt：
//
//   DC_Reference_Mode=AUTO
//   DC_Reference_Slave_Index=-1
//
// AUTO：
//   依 BuildIoMap() 建立的 m_ServoList，按照實體 slave index 由小到大測試；
//   第一台能成功讀取 0x0910 且 System Time 非 0 的伺服成為 DC Reference。
//
// FIXED：
//   使用 DC_Reference_Slave_Index 指定的零起算 slave index。指定站不存在、
//   無法讀取 0x0910 或 DC System Time 為 0 時，不會偷偷改選其他從站。
//
// AUTO Topology Dry-Run：
//   在 DC Reference 選定後，依 m_ServoList 的實體 slave index 建立最多 8 軸
//   的唯讀順序快照，檢查 0x0910、0x0928、DL Status 與線型 Port 形狀。
//   本階段不寫入 0x0928，既有 S4/S5/S6 propagation delay 邏輯保持不變。
// ============================================================================

namespace
{
    volatile LONG g_dcReferenceSlaveIndex = -1;

    struct DcAutoTopologyNode
    {
        int order;
        int slaveIndex;
        uint16_t configAddr;
        uint16_t dlStatus;
        uint16_t physicalPortMask;
        uint64_t dcSystemTime;
        uint32_t propagationDelayNs;
        int dlWkc;
        int dcWkc;
        int delayWkc;
    };

    std::string ToUpperCopy(
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

    bool ValidateDcReferenceCandidate(
        EtherCatMaster* pMaster,
        int slaveIndex,
        uint64_t& dcSystemTime,
        int& dcWkc)
    {
        dcSystemTime = 0;
        dcWkc = 0;

        if (pMaster == nullptr ||
            pMaster->m_pEni == nullptr)
        {
            return false;
        }

        const auto& slaves =
            pMaster->m_pEni->GetSlaves();

        if (slaveIndex < 0 ||
            slaveIndex >= (int)slaves.size())
        {
            return false;
        }

        uint16_t configAddr =
            m_slaveInfo[slaveIndex].configAddr;

        if (configAddr == 0)
        {
            return false;
        }

        // 最多嘗試三次，避免剛好碰到一次啟動期封包延遲。
        for (int attempt = 0;
            attempt < 3;
            attempt++)
        {
            uint64_t candidateDcTime = 0;

            int candidateWkc =
                pMaster->ecx_FPRD(
                    configAddr,
                    0x0910,
                    &candidateDcTime,
                    8,
                    20);

            if (candidateWkc > 0 &&
                candidateDcTime > 0)
            {
                dcSystemTime = candidateDcTime;
                dcWkc = candidateWkc;
                return true;
            }

            dcWkc = candidateWkc;
        }

        return false;
    }

    void PrintSelectedDcReference(
        int slaveIndex,
        const char* source,
        uint64_t dcSystemTime,
        int dcWkc)
    {
        RtPrintf(
            "[DC-REFERENCE] SELECTED | "
            "Mode:%s | "
            "SlaveIndex:%d | "
            "ConfigAddr:0x%04X | "
            "Vendor:0x%08lX | "
            "Product:0x%08lX | "
            "WKC:%d | "
            "DC:%llu\n",

            source,
            slaveIndex,
            (unsigned int)m_slaveInfo[slaveIndex].configAddr,
            (unsigned long)m_slaveInfo[slaveIndex].Vendor_ID,
            (unsigned long)m_slaveInfo[slaveIndex].Product_Code,
            dcWkc,
            (unsigned long long)dcSystemTime);
    }

    bool IsDcAutoNodeReadable(
        const DcAutoTopologyNode& node)
    {
        return
            node.dlWkc > 0 &&
            node.dcWkc > 0 &&
            node.delayWkc > 0 &&
            node.dcSystemTime > 0;
    }

    bool IsExpectedLinearServoPortShape(
        const std::vector<DcAutoTopologyNode>& nodes)
    {
        if (nodes.empty())
        {
            return false;
        }

        for (size_t i = 0;
            i < nodes.size();
            i++)
        {
            const uint16_t portMask =
                nodes[i].physicalPortMask;

            const bool p0Linked =
                (portMask & 0x01U) != 0;

            const bool p1Linked =
                (portMask & 0x02U) != 0;

            const bool branchedOnP2OrP3 =
                (portMask & 0x0CU) != 0;

            if (!p0Linked ||
                branchedOnP2OrP3)
            {
                return false;
            }

            const bool isLast =
                (i + 1U) == nodes.size();

            if (!isLast &&
                !p1Linked)
            {
                return false;
            }

            // 最後一台 Servo 的 P1 可以是 Open，也可以連到後續非 Servo 從站。
            // Dry-Run 只確認 Servo 彼此的鏈結，不把尾端設備誤判成失敗。
        }

        return true;
    }
}

int GetDcReferenceSlaveIndex()
{
    // Writer 只在正式 PDO timer 啟動前執行 InterlockedExchange；進入 Runtime 後
    // 索引不再改變。因此 4 kHz 路徑只做一次 volatile 32-bit read，不使用 locked
    // InterlockedCompareExchange，避免增加不必要的即時路徑成本。
    return (int)g_dcReferenceSlaveIndex;
}

bool DiagnoseDcAutoTopologyDryRun(
    EtherCatMaster* pMaster)
{
    if (pMaster == nullptr ||
        pMaster->m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-TOPOLOGY-AUTO] FAILED | "
            "Reason:Master or ENI unavailable | "
            "Write0928:NO\n");

        return false;
    }

    const auto& slaves =
        pMaster->m_pEni->GetSlaves();

    const int totalSlaves =
        (int)slaves.size();

    const int servoCount =
        (int)pMaster->m_ServoList.size();

    RtPrintf(
        "\n"
        "============================================================\n"
        "[DC-TOPOLOGY-AUTO] BEGIN | "
        "Mode:DRY_RUN | "
        "SlaveCount:%d | "
        "ServoCount:%d | "
        "ServoLimit:%d | "
        "Write0928:NO\n"
        "============================================================\n",

        totalSlaves,
        servoCount,
        DC_AUTO_MAX_SERVO_COUNT);

    if (totalSlaves <= 0 ||
        servoCount <= 0)
    {
        RtPrintf(
            "[DC-TOPOLOGY-AUTO] FAILED | "
            "Reason:No slave or no servo | "
            "Write0928:NO\n");

        return false;
    }

    if (servoCount >
        DC_AUTO_MAX_SERVO_COUNT)
    {
        RtPrintf(
            "[DC-TOPOLOGY-AUTO] FAILED | "
            "Reason:Servo count exceeds limit | "
            "ServoCount:%d | Limit:%d | "
            "Write0928:NO\n",

            servoCount,
            DC_AUTO_MAX_SERVO_COUNT);

        return false;
    }

    std::vector<int> servoSlaveIndices;

    servoSlaveIndices.reserve(
        (size_t)servoCount);

    for (size_t i = 0;
        i < pMaster->m_ServoList.size();
        i++)
    {
        servoSlaveIndices.push_back(
            pMaster->m_ServoList[i].slaveIndex);
    }

    std::sort(
        servoSlaveIndices.begin(),
        servoSlaveIndices.end());

    bool indexRangeValid =
        true;

    bool duplicateIndex =
        false;

    bool consecutiveServoIndices =
        true;

    for (size_t i = 0;
        i < servoSlaveIndices.size();
        i++)
    {
        const int slaveIndex =
            servoSlaveIndices[i];

        if (slaveIndex < 0 ||
            slaveIndex >= totalSlaves)
        {
            indexRangeValid =
                false;
        }

        if (i > 0)
        {
            if (servoSlaveIndices[i - 1] ==
                slaveIndex)
            {
                duplicateIndex =
                    true;
            }

            if (servoSlaveIndices[i - 1] + 1 !=
                slaveIndex)
            {
                consecutiveServoIndices =
                    false;
            }
        }
    }

    if (!indexRangeValid ||
        duplicateIndex)
    {
        RtPrintf(
            "[DC-TOPOLOGY-AUTO] FAILED | "
            "Reason:Invalid or duplicate servo slave index | "
            "IndexRangeValid:%s | Duplicate:%s | "
            "Write0928:NO\n",

            indexRangeValid ? "YES" : "NO",
            duplicateIndex ? "YES" : "NO");

        return false;
    }

    const int referenceSlaveIndex =
        GetDcReferenceSlaveIndex();

    std::vector<DcAutoTopologyNode> nodes;

    nodes.reserve(
        servoSlaveIndices.size());

    bool allNodesReadable =
        true;

    bool referenceInServoOrder =
        false;

    RtPrintf(
        "[DC-TOPOLOGY-AUTO] ORDER | ");

    for (size_t i = 0;
        i < servoSlaveIndices.size();
        i++)
    {
        RtPrintf(
            "%sS%d",
            i == 0 ? "" : " -> ",
            servoSlaveIndices[i]);
    }

    RtPrintf("\n");

    for (size_t i = 0;
        i < servoSlaveIndices.size();
        i++)
    {
        DcAutoTopologyNode node = {};

        node.order =
            (int)i;

        node.slaveIndex =
            servoSlaveIndices[i];

        node.configAddr =
            m_slaveInfo[node.slaveIndex].configAddr;

        node.dlWkc =
            pMaster->ecx_FPRD(
                node.configAddr,
                0x0110,
                &node.dlStatus,
                2,
                20);

        node.physicalPortMask =
            (uint16_t)(
                (node.dlStatus >> 4) &
                0x000FU);

        node.dcWkc =
            pMaster->ecx_FPRD(
                node.configAddr,
                0x0910,
                &node.dcSystemTime,
                8,
                20);

        node.delayWkc =
            pMaster->ecx_FPRD(
                node.configAddr,
                0x0928,
                &node.propagationDelayNs,
                4,
                20);

        const bool isReference =
            node.slaveIndex ==
            referenceSlaveIndex;

        if (isReference)
        {
            referenceInServoOrder =
                true;
        }

        const bool nodeReadable =
            IsDcAutoNodeReadable(
                node);

        if (!nodeReadable)
        {
            allNodesReadable =
                false;
        }

        RtPrintf(
            "[DC-TOPOLOGY-AUTO] NODE | "
            "Order:%d | "
            "SlaveIndex:%d | "
            "Role:%s | "
            "ConfigAddr:0x%04X | "
            "Vendor:0x%08lX | "
            "Product:0x%08lX | "
            "DL:0x%04X | "
            "PortMask:0x%X | "
            "DC:%llu WKC:%d | "
            "Delay:%u ns WKC:%d | "
            "Readable:%s\n",

            node.order,
            node.slaveIndex,
            isReference ? "REFERENCE" : "FOLLOWER",
            (unsigned int)node.configAddr,
            (unsigned long)m_slaveInfo[node.slaveIndex].Vendor_ID,
            (unsigned long)m_slaveInfo[node.slaveIndex].Product_Code,
            (unsigned int)node.dlStatus,
            (unsigned int)node.physicalPortMask,
            (unsigned long long)node.dcSystemTime,
            node.dcWkc,
            (unsigned int)node.propagationDelayNs,
            node.delayWkc,
            nodeReadable ? "YES" : "NO");

        nodes.push_back(
            node);
    }

    const bool referenceIsFirstServo =
        !servoSlaveIndices.empty() &&
        referenceSlaveIndex ==
        servoSlaveIndices.front();

    const bool linearPortShape =
        IsExpectedLinearServoPortShape(
            nodes);

    const int leadingNonServoCount =
        servoSlaveIndices.front();

    const int trailingNonServoCount =
        totalSlaves -
        1 -
        servoSlaveIndices.back();

    const bool dryRunPass =
        allNodesReadable &&
        referenceInServoOrder &&
        referenceIsFirstServo &&
        consecutiveServoIndices &&
        linearPortShape;

    RtPrintf(
        "[DC-TOPOLOGY-AUTO] SUMMARY | "
        "Reference:S%d | "
        "ReferenceInOrder:%s | "
        "ReferenceIsFirstServo:%s | "
        "AllServoReadable:%s | "
        "Consecutive:%s | "
        "LinearPortShape:%s | "
        "LeadingNonServo:%d | "
        "TrailingNonServo:%d | "
        "Write0928:NO | "
        "Result:%s\n",

        referenceSlaveIndex,
        referenceInServoOrder ? "YES" : "NO",
        referenceIsFirstServo ? "YES" : "NO",
        allNodesReadable ? "YES" : "NO",
        consecutiveServoIndices ? "YES" : "NO",
        linearPortShape ? "YES" : "NO",
        leadingNonServoCount,
        trailingNonServoCount,
        dryRunPass ? "PASS" : "CHECK");

    RtPrintf(
        "============================================================\n"
        "[DC-TOPOLOGY-AUTO] END | "
        "Mode:DRY_RUN | Write0928:NO\n"
        "============================================================\n\n");

    return dryRunPass;
}

bool ResolveDcReferenceSlave(
    EtherCatMaster* pMaster)
{
    InterlockedExchange(
        &g_dcReferenceSlaveIndex,
        -1);

    if (pMaster == nullptr ||
        pMaster->m_pEni == nullptr)
    {
        RtPrintf(
            "[DC-REFERENCE] FAILED | "
            "Master or ENI unavailable.\n");

        return false;
    }

    const auto& slaves =
        pMaster->m_pEni->GetSlaves();

    if (slaves.empty())
    {
        RtPrintf(
            "[DC-REFERENCE] FAILED | "
            "No EtherCAT slaves.\n");

        return false;
    }

    std::string configPath =
        GlobalConfig::GetInstance().BaseDataDir +
        "SystemConfig.txt";

    std::string mode =
        ToUpperCopy(
            ConfigUtil::ReadConfigString(
                configPath,
                "DC_Reference_Mode",
                "AUTO"));

    int fixedSlaveIndex =
        (int)ConfigUtil::ReadParam(
            configPath,
            "DC_Reference_Slave_Index",
            -1.0);

    if (mode == "FIXED")
    {
        uint64_t dcSystemTime = 0;
        int dcWkc = 0;

        if (!ValidateDcReferenceCandidate(
            pMaster,
            fixedSlaveIndex,
            dcSystemTime,
            dcWkc))
        {
            RtPrintf(
                "[DC-REFERENCE] FAILED | "
                "Mode:FIXED | "
                "SlaveIndex:%d | "
                "Reason:Invalid index or 0x0910 validation failed | "
                "WKC:%d\n",

                fixedSlaveIndex,
                dcWkc);

            return false;
        }

        InterlockedExchange(
            &g_dcReferenceSlaveIndex,
            (LONG)fixedSlaveIndex);

        PrintSelectedDcReference(
            fixedSlaveIndex,
            "FIXED",
            dcSystemTime,
            dcWkc);

        return true;
    }

    if (mode != "AUTO")
    {
        RtPrintf(
            "[DC-REFERENCE] WARNING | "
            "Unknown mode:%s | Using AUTO.\n",

            mode.c_str());
    }

    if (pMaster->m_ServoList.empty())
    {
        RtPrintf(
            "[DC-REFERENCE] FAILED | "
            "Mode:AUTO | Servo list is empty.\n");

        return false;
    }

    for (size_t servoIndex = 0;
        servoIndex < pMaster->m_ServoList.size();
        servoIndex++)
    {
        int slaveIndex =
            pMaster->m_ServoList[servoIndex].slaveIndex;

        uint64_t dcSystemTime = 0;
        int dcWkc = 0;

        if (!ValidateDcReferenceCandidate(
            pMaster,
            slaveIndex,
            dcSystemTime,
            dcWkc))
        {
            RtPrintf(
                "[DC-REFERENCE] AUTO REJECT | "
                "Servo:%u | "
                "SlaveIndex:%d | "
                "WKC:%d\n",

                (unsigned int)servoIndex,
                slaveIndex,
                dcWkc);

            continue;
        }

        InterlockedExchange(
            &g_dcReferenceSlaveIndex,
            (LONG)slaveIndex);

        PrintSelectedDcReference(
            slaveIndex,
            "AUTO",
            dcSystemTime,
            dcWkc);

        return true;
    }

    RtPrintf(
        "[DC-REFERENCE] FAILED | "
        "Mode:AUTO | No servo passed 0x0910 validation.\n");

    return false;
}
