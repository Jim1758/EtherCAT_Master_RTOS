#include "GCodeParser.h"
#include <cctype>

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

    // 2. 清除註解與空白 (利用你寫好的 MacroParser 功能)
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
    // 🌟 新增：攔截 GOTO 與 IF 指令
    // ==========================================
    size_t gotoPos = clean.find("GOTO");
    if (gotoPos != std::string::npos) {
        bool shouldJump = true;
        size_t ifPos = clean.find("IF");

        // 處理 IF 條件判斷，例如：IF(#1<30)GOTO10
        if (ifPos != std::string::npos && ifPos < gotoPos) {
            // 擷取 IF 和 GOTO 中間的算式： "(#1<30)"
            std::string condition = clean.substr(ifPos + 2, gotoPos - (ifPos + 2));

            // 丟給算術大腦計算，如果結果為 0 代表 false，就不跳躍
            if (m_macroParser.Evaluate(condition) == 0.0) {
                shouldJump = false;
            }
        }

        // 如果條件成立 (或是純 GOTO 指令)
        if (shouldJump) {
            std::string targetStr = clean.substr(gotoPos + 4);
            block.isGoto = true;
            block.gotoTarget = (int)m_macroParser.Evaluate(targetStr);
            block.isEmpty = false;
            return block; // 這是跳躍指令，後面的 G 碼不看了，直接回傳
        }
        else {
            return block; // 條件不成立，忽略這行
        }
    }


    // 4. 字碼掃描與解析 (Lexer)
    char currentAddress = '\0';
    std::string currentValueStr = "";
    int parenDepth = 0; // 追蹤括號深度

    // Lambda 函數：結算當前的 Word
    auto ProcessWord = [&]() {
        if (currentAddress == '\0' || currentValueStr.empty()) return;

        block.isEmpty = false;
        double val = m_macroParser.Evaluate(currentValueStr);

        if (currentAddress == 'G') {
            // 限制：只有第一個 G 碼會被收錄
            if (!block.hasG) {
                block.hasG = true;
                block.gCode = (int)val;
            }
            else {
                // 你可以在這裡加入警告，例如：
                // DEBUG_PRINT("格式錯誤：同一行只能有一個 G 碼！\n");
            }
        }
        else if (currentAddress == 'M') {
            // 限制：最多收 3 個 M 碼
            if (block.mCount < 3) {
                block.mCode[block.mCount] = (int)val;
                block.mCount++;
            }
        }
        else if (currentAddress >= 'A' && currentAddress <= 'Z') {
            // A~Z (包含 X, Y, Z, F, E, B 等等) 全部自動存入陣列
            int index = currentAddress - 'A';
            block.hasParam[index] = true;
            block.param[index] = val;
        }

        currentAddress = '\0';
        currentValueStr = "";
    };

    // 逐字元掃描字串
    for (size_t i = startIdx; i < clean.length(); i++) {
        char c = clean[i];

        if (c == '(' || c == '[') parenDepth++;
        if (c == ')' || c == ']') parenDepth--;

        // 如果我們不在括號內，且遇到英文字母
        if (parenDepth == 0 && std::isalpha(c)) {
            // 檢查它是不是數學函數 (例如 SIN)，如果是，它屬於 Value 的一部分
            if (IsMathKeyword(clean, i)) {
                currentValueStr += c;
            }
            else {
                // 發現新的控制字母 (例如 X, Y, G)
                ProcessWord(); // 先結算上一個收集完的 Word
                currentAddress = c;
            }
        }
        else {
            // 數字、符號、括號內的字母，通通塞進 Value 裡面
            if (currentAddress != '\0') {
                currentValueStr += c;
            }
        }
    }

    // 結算最後一個 Word
    ProcessWord();

    return block;
}