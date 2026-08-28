#pragma once
#include "GCodeParser.h"
#include "MacroParser.h"
#include "NC_Types.h"
#include <cstdint>

enum class NCExpressionResolveError : std::uint8_t
{
    NONE = 0,
    INVALID_PARSED_BLOCK,
    TOO_MANY_G_CODES,
    TOO_MANY_M_CODES,
    EXPRESSION_ERROR,
    INVALID_CODE_VALUE,
    INVALID_VARIABLE_INDEX,
    INVALID_GOTO_TARGET
};

struct NCMacroAssignmentCommit
{
    char prefix = 0;
    int index = 0;
    double value = 0.0;
};

struct NCGotoDecision
{
    bool shouldJump = false;
    int targetSequence = -1;
};

namespace NCExpressionResolver
{
    // 只讀 Macro 變數，將 Raw Expression 解析成既有 NCBlock。
    bool ResolveBlock(
        const NCParsedBlock& parsed,
        MacroParser& evaluator,
        NCBlock& block,
        NCExpressionResolveError& error);

    // 只計算指派結果，不寫 MacroEngine；真正 SetVar 由 NCManager Commit。
    bool ResolveAssignment(
        const NCParsedBlock& parsed,
        MacroParser& evaluator,
        NCMacroAssignmentCommit& assignment,
        NCExpressionResolveError& error);

    // 只在條件成立時求值 GOTO Target。
    bool ResolveGoto(
        const NCParsedBlock& parsed,
        MacroParser& evaluator,
        NCGotoDecision& decision,
        NCExpressionResolveError& error);
}
