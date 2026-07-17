#pragma once
#pragma once  // 防止標頭檔被重複載入 (非常重要)

#include <string>

// =================================================================
// 建議加入 Namespace (命名空間)，避免未來函數名稱跟別人撞名
// =================================================================
namespace ConfigUtil
{
    // 宣告：讀取字串參數
    std::string ReadConfigString(const std::string& filePath, const std::string& targetKey, const std::string& defaultValue);

    // 宣告：讀取浮點數參數 (你之前寫的)
    double ReadParam(const std::string& filePath, const std::string& targetKey, double defaultValue);
}