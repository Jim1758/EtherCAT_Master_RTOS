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

    // 4. 字碼掃描與解析 (Lexer)
    char currentAddress = '�';
    std::string currentValueStr = "";
    int parenDepth = 0;

    // Lambda 函數：結算當前的 Word
    auto ProcessWord = [&]() {
        // 清除字串尾部的隱形字元
        while (!currentValueStr.empty() && std::isspace(currentValueStr.back())) {
            currentValueStr.pop_back();
        }

        if (currentAddress == '�' || currentValueStr.empty()) return;

        block.isEmpty = false;
        double val = 0.0;

        // 智慧分流：是純數字，還是巨集算式？
        bool isMacro = (currentValueStr.find('[') != std::string::npos ||
            currentValueStr.find('#') != std::string::npos ||
            std::isalpha(currentValueStr[0]));

        if (!isMacro) {
            // 純數值：使用 RTOS 最安全的 strtod 進行解析與防呆
            char* endPtr = nullptr;
            val = std::strtod(currentValueStr.c_str(), &endPtr);

            if (endPtr == currentValueStr.c_str()) {
                AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
                return;
            }
        }
        else {
            // 巨集算式：交給巨集大腦去解
            val = m_macroParser.Evaluate(currentValueStr);
        }

        // 🌟 處理 G 碼 (重點修改區)
        if (currentAddress == 'G') {
            if (block.gCount < 10) {
                block.gCodes[block.gCount] = (int)val;
            }

            // 記錄這行的第一個 G 碼為主要 gCode
            if (block.gCount == 0) {
                block.hasG = true;
                block.gCode = (int)val;
            }
            block.gCount++; // 每次讀到 'G'，計數器就 +1
        }
        // 處理 M 碼
        else if (currentAddress == 'M') {
            if (block.mCount < 3) {
                block.mCode[block.mCount] = (int)val;
                block.mCount++;
            }
        }
        // 處理 A~Z
        else if (currentAddress >= 'A' && currentAddress <= 'Z') {
            int index = currentAddress - 'A';
            block.hasParam[index] = true;
            block.param[index] = val;
        }

        currentAddress = '�';
        currentValueStr = "";
    };

    // 逐字元掃描字串
    for (size_t i = startIdx; i < clean.length(); i++) {
        char c = clean[i];

        if (c == '(' || c == '[') parenDepth++;
        if (c == ')' || c == ']') parenDepth--;

        if (parenDepth == 0 && std::isalpha(c)) {
            if (IsMathKeyword(clean, i)) {
                currentValueStr += c;
            }
            else {
                ProcessWord();
                currentAddress = c;
            }
        }
        else {
            if (currentAddress != '�') {
                currentValueStr += c;
            }
        }
    }

    // 結算最後一個 Word
    ProcessWord();

    return block;
}