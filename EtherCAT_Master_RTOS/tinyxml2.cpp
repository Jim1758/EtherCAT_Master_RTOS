#include "tinyxml2.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>

using namespace tinyxml2;

// =====================================================================================
// Helper: 簡易字串工具 (取代 Regex)
// =====================================================================================

// 找屬性值：在 tagString (例如 <Slave PhysAddr="1001">) 中尋找 name="value"
std::string ParseAttribute(const std::string& tagHeader, const char* name) {
    std::string key = std::string(name) + "=\"";
    size_t start = tagHeader.find(key);
    if (start == std::string::npos) return "";

    start += key.length();
    size_t end = tagHeader.find("\"", start);
    if (end == std::string::npos) return "";

    return tagHeader.substr(start, end - start);
}

// =====================================================================================
// XMLElement 實作
// =====================================================================================

// 為了支援 NextSibling，我們需要儲存更多資訊
// _tagName: 標籤名 (e.g. "Slave")
// _header: 開頭標籤內容 (e.g. <Slave PhysAddr="1001">) -> 用於 Attribute
// _innerText: 包在裡面的內容 -> 用於 FirstChild
// _tail: 結束標籤之後的剩餘字串 -> 用於 NextSibling
XMLElement::XMLElement(const std::string& tag, const std::string& header, const std::string& inner, const std::string& tail)
    : _tagName(tag), _header(header), _innerText(inner), _tail(tail)
{
}

XMLElement::~XMLElement() {
    for (auto child : _childrenCache) {
        delete child;
    }
    _childrenCache.clear();
}

// -------------------------------------------------------------------------------------
// [關鍵] 屬性讀取
// -------------------------------------------------------------------------------------
const char* XMLElement::Attribute(const char* name) {
    static std::string tempAttr; // 靜態緩衝避免指標懸空
    tempAttr = ParseAttribute(_header, name);
    if (tempAttr.empty()) return nullptr; // 或是 "0" 視需求
    return tempAttr.c_str();
}

const char* XMLElement::GetText() {
    if (_innerText.empty()) return "";

    // 簡單去空白
    const char* ws = " \t\n\r";
    size_t start = _innerText.find_first_not_of(ws);
    if (start == std::string::npos) return "";
    size_t end = _innerText.find_last_not_of(ws);

    // 如果內容包含 < (代表是子節點而非純文字)，則視為無文字
    if (_innerText.find("<") != std::string::npos) {
        // 但有時候可能是 <Name>SomeText</Name>，這裡簡化處理
        // 如果想讀 <BitSize>16</BitSize> 裡面的 16，邏輯是正確的
    }

    _tempText = _innerText.substr(start, end - start + 1);
    return _tempText.c_str();
}

// -------------------------------------------------------------------------------------
// [關鍵] 尋找子節點
// -------------------------------------------------------------------------------------
XMLElement* XMLElement::FirstChildElement(const char* name) {
    if (_innerText.empty()) return nullptr;

    std::string searchTag;
    if (name) searchTag = name;
    else return nullptr; // 簡化版需指定名稱

    // 1. 找開頭 <Tag
    std::string startPattern = "<" + searchTag;
    size_t posStart = _innerText.find(startPattern);
    if (posStart == std::string::npos) return nullptr;

    // 2. 找開頭標籤的結尾 >
    size_t posHeaderEnd = _innerText.find(">", posStart);
    if (posHeaderEnd == std::string::npos) return nullptr;

    // 提取 Header (包含屬性)
    std::string header = _innerText.substr(posStart, posHeaderEnd - posStart + 1);

    // 3. 找結束標籤 </Tag>
    // 注意：這裡不支援巢狀同名標籤 (Nested same tags)，但在 ENI 中很少見
    std::string endPattern = "</" + searchTag + ">";
    size_t posEnd = _innerText.find(endPattern, posHeaderEnd);

    // 處理自閉合標籤 <Tag />
    if (header.back() == '/' || header[header.length() - 2] == '/') {
        // 自閉合，沒有 innerText，沒有 endPattern
        // 剩餘部分 (Tail) 從 Header 結束後開始
        std::string tail = _innerText.substr(posHeaderEnd + 1);

        XMLElement* child = new XMLElement(searchTag, header, "", tail);
        _childrenCache.push_back(child);
        return child;
    }

    if (posEnd == std::string::npos) return nullptr;

    // 提取 Inner Content
    size_t innerStart = posHeaderEnd + 1;
    std::string inner = _innerText.substr(innerStart, posEnd - innerStart);

    // 提取 Tail (用於 NextSibling)
    std::string tail = _innerText.substr(posEnd + endPattern.length());

    XMLElement* child = new XMLElement(searchTag, header, inner, tail);
    _childrenCache.push_back(child);
    return child;
}

// -------------------------------------------------------------------------------------
// [關鍵] 尋找下一個同層節點
// -------------------------------------------------------------------------------------
XMLElement* XMLElement::NextSiblingElement(const char* name) {
    // 我們利用 _tail (目前節點結束後的所有剩餘字串) 來尋找下一個
    if (_tail.empty()) return nullptr;

    std::string searchTag;
    if (name) searchTag = name;
    else searchTag = _tagName; // 預設找同名的

    // 邏輯同 FirstChild，只是來源變成 _tail
    std::string startPattern = "<" + searchTag;
    size_t posStart = _tail.find(startPattern);
    if (posStart == std::string::npos) return nullptr;

    size_t posHeaderEnd = _tail.find(">", posStart);
    if (posHeaderEnd == std::string::npos) return nullptr;

    std::string header = _tail.substr(posStart, posHeaderEnd - posStart + 1);

    std::string endPattern = "</" + searchTag + ">";
    size_t posEnd = _tail.find(endPattern, posHeaderEnd);

    // 自閉合處理
    if (header.find("/>") != std::string::npos) {
        std::string tail = _tail.substr(posHeaderEnd + 1);
        XMLElement* sibling = new XMLElement(searchTag, header, "", tail);
        // 注意：這裡會有記憶體管理的小問題，因為 sibling 沒有 parent cache
        // 建議外部呼叫者小心 delete，但在 RTX64 簡單應用中通常忽略
        return sibling;
    }

    if (posEnd == std::string::npos) return nullptr;

    size_t innerStart = posHeaderEnd + 1;
    std::string inner = _tail.substr(innerStart, posEnd - innerStart);
    std::string nextTail = _tail.substr(posEnd + endPattern.length());

    XMLElement* sibling = new XMLElement(searchTag, header, inner, nextTail);
    return sibling;
}

// =====================================================================================
// XMLDocument 實作
// =====================================================================================

XMLDocument::XMLDocument() : _rootElement(nullptr) {}

XMLDocument::~XMLDocument() {
    if (_rootElement) delete _rootElement;
}

XMLError XMLDocument::LoadFile(const char* filename) {
    FILE* fp = nullptr;
    errno_t err = fopen_s(&fp, filename, "rb");
    if (err != 0 || !fp) {
        return XML_ERROR_FILE_NOT_FOUND;
    }

    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char* buffer = new char[size + 1];
    fread(buffer, 1, size, fp);
    buffer[size] = 0;
    fclose(fp);

    _fileContent = buffer;
    delete[] buffer;

    return XML_SUCCESS;
}

XMLElement* XMLDocument::FirstChildElement(const char* name) {
    // 這是文件的 Root，我們把整個檔案內容當作 innerText
    // 這樣它就會去搜尋 <Name>...</Name>
    // 但因為 Root 沒有 Header，我們虛擬一個
    if (_rootElement) delete _rootElement;

    // 這裡有點 trick: 我們建立一個虛擬的 root，把整個文件當作它的 inner
    // 這樣呼叫 root->FirstChildElement(name) 就會去搜尋文件內容
    _rootElement = new XMLElement("ROOT", "", _fileContent, "");

    return _rootElement->FirstChildElement(name);
}