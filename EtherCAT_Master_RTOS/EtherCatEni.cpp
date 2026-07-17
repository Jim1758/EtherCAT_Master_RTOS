#include "EtherCatEni.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <rtapi.h> // for RtPrintf
#include "GlobalConfig.h"
EtherCatEni::EtherCatEni() 
{
}

EtherCatEni::~EtherCatEni() 
{
}

// [關鍵] 這裡的回傳型別必須跟 Header 一模一樣
const std::vector<EtherCatSlave>& EtherCatEni::GetSlaves() const 
{
    return m_slaves;
}

// [關鍵] 這是成員函式，必須加上 EtherCatEni::
void EtherCatEni::Parse_XML_Value(const char* content, const char* tag, char* outBuffer) 
{
    char startTag[64];
    char endTag[64];
    sprintf(startTag, "<%s>", tag);
    sprintf(endTag, "</%s>", tag);

    char* pStart = strstr((char*)content, startTag);
    if (pStart) 
    {
        pStart += strlen(startTag);
        char* pEnd = strstr(pStart, endTag);
        if (pEnd) {
            int len = pEnd - pStart;
            if (len > 127) len = 127;
            strncpy(outBuffer, pStart, len);
            outBuffer[len] = 0;
        }
    }
}

int EtherCatEni::LoadXml(const char* filename) 
{
    m_slaves.clear();

    FILE* f = fopen(filename, "rb");
    if (!f) 
    {
        DEBUG_PRINT("[Error] Cannot open XML file at:\n%s\n", filename);
        return 0;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0) { fclose(f); return 0; }

    char* xml_buffer = (char*)malloc(fsize + 1);
    if (!xml_buffer) {
        fclose(f);
        return 0;
    }

    size_t readSize = fread(xml_buffer, 1, fsize, f);
    fclose(f);
    xml_buffer[readSize] = 0;

    DEBUG_PRINT(">>> Loading ENI Config (RTX64 Safe + 8124/7062 Fix)...\n");
    int count = 0;

    char* pCurrent = xml_buffer;

    while ((pCurrent = strstr(pCurrent, "<Slave>")) != NULL) 
    {
        char* pEnd = strstr(pCurrent, "</Slave>");
        if (!pEnd) break;

        // 計算長度與複製 (RTX64 安全性)
        int len = (int)(pEnd - pCurrent);
        if (len > 4095) len = 4095;

        char slaveContent[4096];
        memset(slaveContent, 0, sizeof(slaveContent));
        strncpy(slaveContent, pCurrent, len);
        slaveContent[len] = 0;

        EtherCatSlave newSlave;
        memset(&newSlave, 0, sizeof(EtherCatSlave));
        char buffer[128] = { 0 };

        // 1. 基本資料
        Parse_XML_Value(slaveContent, "Name", newSlave.name);
        Parse_XML_Value(slaveContent, "Type", newSlave.type);

        memset(buffer, 0, sizeof(buffer));
        Parse_XML_Value(slaveContent, "ProductCode", buffer);
        newSlave.productCode = (uint32_t)(strstr(buffer, "#x") ? strtoul(buffer + 2, NULL, 16) : atoi(buffer));

        memset(buffer, 0, sizeof(buffer));
        Parse_XML_Value(slaveContent, "VendorId", buffer);
        newSlave.vendorId = (uint32_t)(strstr(buffer, "#x") ? strtoul(buffer + 2, NULL, 16) : atoi(buffer));

        // 2. 讀取 Input (先讀取真實 XML 值)
        newSlave.configAddrIn = 0;
        newSlave.inputBitLength = 0;

        char* pInputStart = strstr(slaveContent, "<Input>");
        if (pInputStart) {
            char* pInputEnd = strstr(pInputStart, "</Input>");
            if (pInputEnd) {
                char inputBlock[512] = { 0 };
                int blockLen = (int)(pInputEnd - pInputStart);
                if (blockLen > 511) blockLen = 511;
                strncpy(inputBlock, pInputStart, blockLen);
                inputBlock[blockLen] = 0;

                memset(buffer, 0, sizeof(buffer));
                Parse_XML_Value(inputBlock, "PhysAddr", buffer);
                newSlave.configAddrIn = (uint16_t)atoi(buffer);

                memset(buffer, 0, sizeof(buffer));
                Parse_XML_Value(inputBlock, "BitSize", buffer);
                newSlave.inputBitLength = (uint32_t)atoi(buffer);
            }
        }

        // 3. 讀取 Output
        newSlave.configAddrOut = 0;
        newSlave.outputBitLength = 0;

        char* pOutputStart = strstr(slaveContent, "<Output>");
        if (pOutputStart) {
            char* pOutputEnd = strstr(pOutputStart, "</Output>");
            if (pOutputEnd) {
                char outputBlock[512] = { 0 };
                int blockLen = (int)(pOutputEnd - pOutputStart);
                if (blockLen > 511) blockLen = 511;
                strncpy(outputBlock, pOutputStart, blockLen);
                outputBlock[blockLen] = 0;

                memset(buffer, 0, sizeof(buffer));
                Parse_XML_Value(outputBlock, "PhysAddr", buffer);
                newSlave.configAddrOut = (uint16_t)atoi(buffer);

                memset(buffer, 0, sizeof(buffer));
                Parse_XML_Value(outputBlock, "BitSize", buffer);
                newSlave.outputBitLength = (uint32_t)atoi(buffer);
            }
        }

        // =================================================================
        // [關鍵邏輯修正] 針對台達特殊模組的處理
        // =================================================================
        switch (newSlave.vendorId)
        {
        case 0x1DD: // 台達 Delta
            switch (newSlave.productCode)
            {
            default:
                break;
            }
            break;

        default:
            break;
        }
        // =================================================================

        // 加入列表
        m_slaves.push_back(newSlave);

        // Log 顯示最終結果
        /*
        RtPrintf("[ENI] Slot %d: %s (Code:0x%X)\n", count, newSlave.name, newSlave.productCode);
        if (newSlave.inputBitLength > 0)
        {
            RtPrintf("In:  Addr 0x%X, Len %d bits\n", newSlave.configAddrIn, newSlave.inputBitLength);
        }
            
        if (newSlave.outputBitLength > 0)
        {
            RtPrintf("Out: Addr 0x%X, Len %d bits\n", newSlave.configAddrOut, newSlave.outputBitLength);
        }*/
        count++;
        pCurrent = pEnd + strlen("</Slave>");
    }

    free(xml_buffer);
    return count;
}