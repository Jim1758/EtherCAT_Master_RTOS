#pragma once
#ifndef TINYXML2_INCLUDED
#define TINYXML2_INCLUDED

#include <string>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <cstring>

// 針對 RTX64 環境優化：
// 1. 去除 Windows API 依賴
// 2. 增強解析結構以支援 Attribute (屬性) 和 Sibling (兄弟節點)

namespace tinyxml2
{
    enum XMLError {
        XML_SUCCESS = 0,
        XML_ERROR_FILE_NOT_FOUND,
        XML_ERROR_PARSING
    };

    // 輕量級 XML 元素類別
    class XMLElement {
    public:
        // [修改] 建構子擴充：需要傳入 Header(屬性區), Inner(內容區), Tail(後續區)
        XMLElement(const std::string& tag, const std::string& header, const std::string& inner, const std::string& tail);
        ~XMLElement();

        // 核心功能：尋找節點
        XMLElement* FirstChildElement(const char* name = nullptr);
        XMLElement* NextSiblingElement(const char* name = nullptr);

        // 取得內容與屬性
        const char* GetText();
        const char* Attribute(const char* name);

        // 方便除錯用的 Helper (可選)
        int IntText() { return std::atoi(GetText()); }

    private:
        // [核心結構變更]
        std::string _tagName;   // 標籤名 (e.g. "Slave")
        std::string _header;    // 開頭標籤字串 (e.g. <Slave PhysAddr="1001">) -> 用於解析 Attribute
        std::string _innerText; // 包在內部的內容 -> 用於 FirstChildElement
        std::string _tail;      // 結束標籤後的剩餘字串 -> 用於 NextSiblingElement

        // 記憶體管理與緩衝
        std::vector<XMLElement*> _childrenCache;
        std::string _tempText;
    };

    // 輕量級 XML 文件類別
    class XMLDocument {
    public:
        XMLDocument();
        ~XMLDocument();

        XMLError LoadFile(const char* filename);
        XMLElement* FirstChildElement(const char* name);

    private:
        std::string _fileContent;
        XMLElement* _rootElement;
    };
}

#endif // TINYXML2_INCLUDED