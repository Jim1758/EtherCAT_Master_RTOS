#pragma once
#include "NC_Types.h"
#include <array>
#include <cstdint>
#include <string>

class MacroParser;

// =============================================================================
// Stage NC-0.2B - Pure G-code parser contract
//
// GCodeParser 只做字串 -> 語法資料：
//   * 不讀 MacroEngine 變數
//   * 不寫 MacroEngine 變數
//   * 不判斷 IF 條件
//   * 不執行 GOTO
//   * 不觸發 AlarmManager
//
// 數值 / 巨集運算式會原樣保存，交由 NCExpressionResolver 在目前 PC
// 真正執行時求值；變數寫入與程式跳轉則由 NCManager Commit。
// =============================================================================

enum class NCParseError : std::uint8_t
{
    NONE = 0,
    MISSING_ADDRESS_VALUE,
    ORPHAN_VALUE,
    UNBALANCED_DELIMITER,
    INVALID_ASSIGNMENT,
    INVALID_GOTO
};

enum class NCParsedControlType : std::uint8_t
{
    NONE = 0,
    ASSIGNMENT,
    GOTO
};

struct NCParsedAssignment
{
    char prefix = 0;
    std::string indexExpression;
    std::string valueExpression;
};

struct NCParsedGoto
{
    bool conditional = false;
    std::string conditionExpression;
    std::string targetExpression;
};

struct NCParsedBlock
{
    bool isEmpty = true;
    bool isBlockSkip = false;

    // 任一 G/M/Address Expression 含 #/@/$ 時設為 true。
    // NCManager 會把這種 Block 當成 Program Commit Barrier，確保
    // Runtime/System Variable 不會在前段 Motion 尚未完成時被提早取樣。
    bool dependsOnMacroState = false;

    NCParseError error = NCParseError::NONE;

    int gCount = 0;
    std::array<std::string, NC_MAX_G_CODES_PER_BLOCK> gExpressions{};

    int mCount = 0;
    std::array<std::string, NC_MAX_M_CODES_PER_BLOCK> mExpressions{};

    std::array<bool, 26> hasParam{};
    std::array<std::string, 26> paramExpressions{};

    NCParsedControlType controlType = NCParsedControlType::NONE;
    NCParsedAssignment assignment{};
    NCParsedGoto gotoStatement{};

    bool has(char letter) const noexcept
    {
        return
            letter >= 'A' &&
            letter <= 'Z' &&
            hasParam[static_cast<std::size_t>(letter - 'A')];
    }

    const std::string& expression(char letter) const noexcept
    {
        static const std::string empty;
        if (letter < 'A' || letter > 'Z')
        {
            return empty;
        }

        return paramExpressions[static_cast<std::size_t>(letter - 'A')];
    }
};

class GCodeParser
{
public:
    GCodeParser() noexcept = default;

    // 相容舊建構方式；參數刻意不保存，Parser 仍然是 Pure Parser。
    explicit GCodeParser(MacroParser&) noexcept {}

    NCParsedBlock ParseLine(const std::string& line) const;

    // 載入程式時建立 N Label Table 使用。只接受可靜態確定的整數 N 值；
    // N[#1] 這種動態標籤不會被加入表格。
    static bool TryExtractLiteralSequenceNumber(
        const NCParsedBlock& block,
        int& sequenceNumber);

    static bool TryExtractLiteralSequenceNumber(
        const std::string& line,
        int& sequenceNumber);

private:
    static std::size_t GetExpressionKeywordLength(
        const std::string& text,
        std::size_t position) noexcept;
};
