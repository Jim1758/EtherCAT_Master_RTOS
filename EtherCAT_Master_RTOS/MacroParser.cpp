#include "MacroParser.h"
#include <cctype>
#include <algorithm>

#define PI 3.14159265358979323846

MacroParser::MacroParser(MacroEngine& engine) : m_engine(engine), m_p(nullptr) {}

// 去除註解與空白
std::string MacroParser::CleanExpression(const std::string& input) {
    std::string result = "";
    bool inComment = false;

    for (size_t i = 0; i < input.length(); ++i) {
        if (!inComment) {
            // 處理 /* 註解
            if (input[i] == '/' && i + 1 < input.length() && input[i + 1] == '*') {
                inComment = true;
                i++; // 跳過 *
                continue;
            }
            // 處理 ; 註解 (分號後面全略過)
            if (input[i] == ';') break;

            // 轉大寫並去除空白
            if (!std::isspace(input[i])) {
                result += std::toupper(input[i]);
            }
        }
        else {
            // 尋找 */ 結束註解
            if (input[i] == '*' && i + 1 < input.length() && input[i + 1] == '/') {
                inComment = false;
                i++;
            }
        }
    }
    return result;
}

// 執行指派語句，例如 "#1 = 100"
bool MacroParser::ExecuteAssignment(const std::string& expr) {
    std::string cleanExpr = CleanExpression(expr);
    size_t eqPos = cleanExpr.find('=');

    if (eqPos != std::string::npos) {
        // 等號左邊必須是變數，例如 #100
        std::string lhs = cleanExpr.substr(0, eqPos);
        std::string rhs = cleanExpr.substr(eqPos + 1);

        if (lhs.length() > 1 && (lhs[0] == '#' || lhs[0] == '@' || lhs[0] == '$')) {
            char prefix = lhs[0];
            // 把 "#1" 的 "1" 丟給 Evaluate 算出來 (支援 #[#1+1] 這種寫法)
            double index = Evaluate(lhs.substr(1));
            double value = Evaluate(rhs);           // 計算右邊的算式

            m_engine.SetVar(prefix, (int)index, value);
            return true;
        }
    }
    return false; // 不是指派語句
}

// 核心計算進入點
double MacroParser::Evaluate(const std::string& expression) {
    std::string cleanExpr = CleanExpression(expression);
    m_p = cleanExpr.c_str();
    return parseExpression();
}

void MacroParser::skipWhitespace() {
    while (std::isspace(*m_p)) m_p++;
}

bool MacroParser::match(const char* str) {
    const char* temp = m_p;
    while (*str) {
        if (*temp != *str) return false;
        temp++; str++;
    }
    m_p = temp; // 比對成功，指標前進
    return true;
}

// 1. 邏輯 OR (||)
double MacroParser::parseExpression() {
    double left = parseLogicalAnd();
    while (match("||") || match("OR")) left = (left != 0 || parseLogicalAnd() != 0) ? 1.0 : 0.0;
    return left;
}

// 2. 邏輯 AND (&&)
double MacroParser::parseLogicalAnd() {
    double left = parseEquality();
    while (match("&&") || match("AND")) left = (left != 0 && parseEquality() != 0) ? 1.0 : 0.0;
    return left;
}

// 3. 等於與不等於 (==, !=)
double MacroParser::parseEquality() {
    double left = parseRelational();
    while (true) {
        if (match("==")) left = (left == parseRelational()) ? 1.0 : 0.0;
        else if (match("!=")) left = (left != parseRelational()) ? 1.0 : 0.0;
        else break;
    }
    return left;
}

// 4. 大小關係 (>, <, >=, <=)
double MacroParser::parseRelational() {
    double left = parseAddSub();
    while (true) {
        if (match(">=")) left = (left >= parseAddSub()) ? 1.0 : 0.0;
        else if (match("<=")) left = (left <= parseAddSub()) ? 1.0 : 0.0;
        else if (match(">")) left = (left > parseAddSub()) ? 1.0 : 0.0;
        else if (match("<")) left = (left < parseAddSub()) ? 1.0 : 0.0;
        else break;
    }
    return left;
}

// 5. 加減 (+, -)
double MacroParser::parseAddSub() {
    double left = parseMulDiv();
    while (true) {
        if (match("+")) left += parseMulDiv();
        else if (match("-")) left -= parseMulDiv();
        else break;
    }
    return left;
}

// 6. 乘除 (*, /)
double MacroParser::parseMulDiv() {
    double left = parseUnary();
    while (true) {
        if (match("*")) left *= parseUnary();
        else if (match("/")) left /= parseUnary(); // CNC 遇 0 除錯通常由系統捕捉，這裡暫不防呆
        else break;
    }
    return left;
}

// 7. 正負號與 NOT (+, -, !)
double MacroParser::parseUnary() {
    if (match("+")) return parseUnary();
    if (match("-")) return -parseUnary();
    if (match("!") || match("NOT")) return (parseUnary() == 0) ? 1.0 : 0.0;
    return parsePrimary();
}

// 8. 數字、變數、括號、函數
double MacroParser::parsePrimary() {
    // 處理括號 ()
    if (match("(")) {
        double val = parseExpression();
        match(")"); // 關閉括號
        return val;
    }

    // 處理變數 (#, @, $)
    if (*m_p == '#' || *m_p == '@' || *m_p == '$') {
        char prefix = *m_p++;
        double index = parsePrimary(); // 遞迴解析：支援 #(#1 + 2)
        return m_engine.GetVar(prefix, (int)index);
    }

    // 處理數學函數 (CNC 預設使用角度 Degree)
    if (match("SIN(")) { double val = parseExpression(); match(")"); return std::sin(val * PI / 180.0); }
    if (match("COS(")) { double val = parseExpression(); match(")"); return std::cos(val * PI / 180.0); }
    if (match("TAN(")) { double val = parseExpression(); match(")"); return std::tan(val * PI / 180.0); }
    if (match("ATAN(")) { double val = parseExpression(); match(")"); return std::atan(val) * 180.0 / PI; }
    if (match("SQRT(")) { double val = parseExpression(); match(")"); return std::sqrt(val); }
    if (match("ABS(")) { double val = parseExpression(); match(")"); return std::abs(val); }
    if (match("FIX(")) { double val = parseExpression(); match(")"); return std::floor(val); } // 無條件捨去
    if (match("ROUND(")) { double val = parseExpression(); match(")"); return std::round(val); }

    // 處理純數字
    char* next;
    double val = std::strtod(m_p, &next);
    m_p = next;
    return val;
}