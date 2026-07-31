#include "GCodeParser.h"
#include "AlarmManager.h"
#include <cctype>
#include <cstdlib> // 🌟 新增：為了使用標準且安全的 strtod
GCodeParser::GCodeParser(MacroParser& macroParser) : m_macroParser(macroParser) {}

bool GCodeParser::IsMathKeyword(const std::string& str, size_t pos) {
    std::string keywords[] = { "SIN", "COS", "TAN", "ATAN", "SQRT", "ABS", "FIX", "ROUND", "AND", "OR", "NOT" };
    for (const auto& kw : keywords) {
        if (str.compare(pos, kw.length(), kw) == 0) return true;
    }
    return false;
}

NCBlock GCodeParser::ParseLine(const std::string& line) {
    NCBlock block;

    // 1. 優先處理純巨集指派 (例如 #1 = 100)
    // 如果執行成功，代表這行不是動作指令，直接回傳空區塊
    if (m_macroParser.ExecuteAssignment(line)) {
        return block;
    }

    // 2. 清除註解與空白
    std::string clean = MacroParser::CleanExpression(line);
    if (clean.empty()) return block;

    // 3. 處理單節跳躍符號 '/'
    size_t startIdx = 0;
    if (clean[0] == '/') {
        block.isBlockSkip = true;
        startIdx = 1;
        if (clean.length() <= 1) return block;
    }

    // ==========================================
    // 🌟 攔截 GOTO 與 IF 指令
    // ==========================================
    size_t gotoPos = clean.find("GOTO");
    if (gotoPos != std::string::npos) {
        bool shouldJump = true;
        size_t ifPos = clean.find("IF");

        // 處理 IF 條件判斷
        if (ifPos != std::string::npos && ifPos < gotoPos) {
            std::string condition = clean.substr(ifPos + 2, gotoPos - (ifPos + 2));
            if (m_macroParser.Evaluate(condition) == 0.0) {
                shouldJump = false;
            }
        }

        // 條件成立或純 GOTO
        if (shouldJump) {
            std::string targetStr = clean.substr(gotoPos + 4);
            block.isGoto = true;
            block.gotoTarget = (int)m_macroParser.Evaluate(targetStr);
            block.isEmpty = false;
            return block;
        }
        else {
            return block;
        }
    }



    // Lambda 函數：結算當前的 Word
 // Lambda 函數：結算當前的 Word (🌟 加上 -> bool)

    // ==========================================
    // 🌟 1. 宣告時，絕對用數字 0，不要用 '\0'
    // ==========================================
    char currentAddress = 0;
    std::string currentValueStr = "";
    int parenDepth = 0;


    // Lambda 函數：結算當前的 Word (加上 -> bool)
    auto ProcessWord = [&]() -> bool {
        // 清除字串尾部的隱形字元
        while (!currentValueStr.empty() && std::isspace(currentValueStr.back())) {
            currentValueStr.pop_back();
        }

        // =======================================================
        // 🌟 這裡所有的判斷，都改用數字 0
        // =======================================================
        // 狀況 A：正常的空狀態 (直接略過)
        if (currentAddress == 0 && currentValueStr.empty()) return true;

        // 狀況 B：有字母卻沒數字 (例如 gred)
        if (currentAddress != 0 && currentValueStr.empty()) {
            AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
            return false; // ❌ 報警並回傳失敗
        }

        // 狀況 C：只有數字卻沒字母
        if (currentAddress == 0 && !currentValueStr.empty()) {
            AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
            return false; // ❌ 報警並回傳失敗
        }

        block.isEmpty = false;
        double val = 0.0;

        bool isMacro = (currentValueStr.find('[') != std::string::npos ||
            currentValueStr.find('#') != std::string::npos ||
            currentValueStr.find('@') != std::string::npos ||
            currentValueStr.find('$') != std::string::npos ||
            std::isalpha(currentValueStr[0]));

        if (!isMacro) {
            char* endPtr = nullptr;
            val = std::strtod(currentValueStr.c_str(), &endPtr);

            // 轉換數字失敗 (格式怪異)
            if (endPtr == currentValueStr.c_str()) {
                AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
                return false;
            }
        }
        else {
            val = m_macroParser.Evaluate(currentValueStr);
        }

        // 處理 G 碼
        if (currentAddress == 'G') {
            if (block.gCount < 10) block.gCodes[block.gCount] = (int)val;
            if (block.gCount == 0) { block.hasG = true; block.gCode = (int)val; }
            block.gCount++;
        }
        // 處理 M 碼
        else if (currentAddress == 'M') {
            if (block.mCount < 3) { block.mCode[block.mCount] = (int)val; block.mCount++; }
        }
        // 處理 A~Z
        else if (currentAddress >= 'A' && currentAddress <= 'Z') {
            int index = currentAddress - 'A';
            block.hasParam[index] = true;
            block.param[index] = val;
        }

        // ==========================================
        // 🌟 2. 結算完畢後，安全歸零 (用數字 0)
        // ==========================================
        currentAddress = 0;
        currentValueStr = "";

        return true;
    };
    // ==========================================
     // 字串掃描迴圈
     // ==========================================
    for (size_t i = startIdx; i < clean.length(); i++) {
        char c = clean[i];

        if (c == '(' || c == '[') parenDepth++;
        if (c == ')' || c == ']') parenDepth--;

        if (parenDepth == 0 && std::isalpha(c)) {
            if (IsMathKeyword(clean, i)) {
                currentValueStr += c;
            }
            else {
                // 如果解析失敗，立刻中斷
                if (!ProcessWord()) {
                    return block;
                }

                // 轉大寫
                currentAddress = std::toupper(c);
            }
        }
        else {
            // ==========================================
            // 🌟 3. 確保這裡也是用數字 0
            // ==========================================
            if (currentAddress != 0) {
                currentValueStr += c;
            }
        }
    }

    // 結算最後一個 Word
    if (!ProcessWord()) {
        return block;
    }

    return block;
}