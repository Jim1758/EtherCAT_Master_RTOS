#include "ConfigReader.h" // 必須 include 自己的標頭檔
#include <fstream>
#include <algorithm>

namespace ConfigUtil
{
    // 實作：讀取字串參數
    std::string ReadConfigString(const std::string& filePath, const std::string& targetKey, const std::string& defaultValue)
    {
        std::ifstream file(filePath);
        std::string line;

        if (file.is_open()) {
            while (std::getline(file, line)) {
                if (line.empty() || line.substr(0, 2) == "//") continue;

                size_t pos = line.find('=');
                if (pos != std::string::npos) {
                    std::string key = line.substr(0, pos);
                    if (key == targetKey) {
                        std::string value = line.substr(pos + 1);
                        value.erase(value.find_last_not_of(" \n\r\t") + 1);
                        return value;
                    }
                }
            }
            file.close();
        }
        return defaultValue;
    }

    // 實作：讀取浮點數參數
    double ReadParam(const std::string& filePath, const std::string& targetKey, double defaultValue)
    {
        std::ifstream file(filePath);
        std::string line;

        if (file.is_open()) {
            while (std::getline(file, line)) {
                if (line.empty() || line.substr(0, 2) == "//") continue;

                size_t pos = line.find('=');
                if (pos != std::string::npos) {
                    std::string key = line.substr(0, pos);
                    std::string valueStr = line.substr(pos + 1);

                    if (key == targetKey) {
                        return std::stod(valueStr);
                    }
                }
            }
            file.close();
        }
        return defaultValue;
    }
}