#pragma once
// [關鍵] 必須引入定義了 EtherCatSlave 的標頭檔
#include "EtherCatTypes.h" 
#include <vector>
#include <string>

class EtherCatEni
{
public:
    EtherCatEni();
    ~EtherCatEni();

    // 載入 XML (注意參數型別必須跟 CPP 一致)
    int LoadXml(const char* filename);

    // [關鍵] 這裡必須是 EtherCatSlave，不能是舊的 EniSlaveConfig
    const std::vector<EtherCatSlave>& GetSlaves() const;

private:
    // [關鍵] 宣告輔助函式
    void Parse_XML_Value(const char* content, const char* tag, char* outBuffer);

private:
    // [關鍵] 容器型別也要改
    std::vector<EtherCatSlave> m_slaves;
};