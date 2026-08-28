#include "NCExpressionResolver.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    bool TryEvaluate(
        MacroParser& evaluator,
        const std::string& expression,
        double& value,
        NCExpressionResolveError& error)
    {
        MacroEvalError macroError = MacroEvalError::NONE;
        if (!evaluator.TryEvaluate(expression, value, &macroError))
        {
            error =
                macroError == MacroEvalError::VARIABLE_INDEX_OUT_OF_RANGE
                ? NCExpressionResolveError::INVALID_VARIABLE_INDEX
                : NCExpressionResolveError::EXPRESSION_ERROR;
            return false;
        }
        return true;
    }

    bool TryToInteger(
        double value,
        int& integer) noexcept
    {
        if (!std::isfinite(value))
        {
            return false;
        }

        const double rounded = std::round(value);
        if (std::fabs(value - rounded) > 1.0e-9 ||
            rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
            rounded > static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }

        integer = static_cast<int>(rounded);
        return true;
    }

    bool IsValidVariableIndex(char prefix, int index) noexcept
    {
        return MacroVariableRules::IsValidIndex(prefix, index);
    }
}

namespace NCExpressionResolver
{
    bool ResolveBlock(
        const NCParsedBlock& parsed,
        MacroParser& evaluator,
        NCBlock& block,
        NCExpressionResolveError& error)
    {
        block = NCBlock{};
        error = NCExpressionResolveError::NONE;

        if (parsed.error != NCParseError::NONE ||
            parsed.controlType != NCParsedControlType::NONE)
        {
            error = NCExpressionResolveError::INVALID_PARSED_BLOCK;
            return false;
        }

        if (parsed.gCount > NC_MAX_G_CODES_PER_BLOCK)
        {
            error = NCExpressionResolveError::TOO_MANY_G_CODES;
            return false;
        }
        if (parsed.mCount > NC_MAX_M_CODES_PER_BLOCK)
        {
            error = NCExpressionResolveError::TOO_MANY_M_CODES;
            return false;
        }

        block.isEmpty = parsed.isEmpty;
        block.isBlockSkip = parsed.isBlockSkip;
        block.gCount = parsed.gCount;
        block.mCount = parsed.mCount;

        for (int i = 0; i < parsed.gCount; ++i)
        {
            double value = 0.0;
            if (!TryEvaluate(
                evaluator,
                parsed.gExpressions[static_cast<std::size_t>(i)],
                value,
                error))
            {
                return false;
            }

            int code = 0;
            if (!TryToInteger(value, code))
            {
                error = NCExpressionResolveError::INVALID_CODE_VALUE;
                return false;
            }

            block.gCodes[i] = code;
            if (i == 0)
            {
                block.hasG = true;
                block.gCode = block.gCodes[i];
            }
        }

        for (int i = 0; i < parsed.mCount; ++i)
        {
            double value = 0.0;
            if (!TryEvaluate(
                evaluator,
                parsed.mExpressions[static_cast<std::size_t>(i)],
                value,
                error))
            {
                return false;
            }

            int code = 0;
            if (!TryToInteger(value, code))
            {
                error = NCExpressionResolveError::INVALID_CODE_VALUE;
                return false;
            }
            block.mCode[i] = code;
        }

        for (int i = 0; i < 26; ++i)
        {
            if (!parsed.hasParam[static_cast<std::size_t>(i)])
            {
                continue;
            }

            double value = 0.0;
            if (!TryEvaluate(
                evaluator,
                parsed.paramExpressions[static_cast<std::size_t>(i)],
                value,
                error))
            {
                return false;
            }

            block.hasParam[i] = true;
            block.param[i] = value;
        }

        return true;
    }

    bool ResolveAssignment(
        const NCParsedBlock& parsed,
        MacroParser& evaluator,
        NCMacroAssignmentCommit& assignment,
        NCExpressionResolveError& error)
    {
        assignment = NCMacroAssignmentCommit{};
        error = NCExpressionResolveError::NONE;

        if (parsed.error != NCParseError::NONE ||
            parsed.controlType != NCParsedControlType::ASSIGNMENT)
        {
            error = NCExpressionResolveError::INVALID_PARSED_BLOCK;
            return false;
        }

        double indexValue = 0.0;
        double assignedValue = 0.0;
        if (!TryEvaluate(
            evaluator,
            parsed.assignment.indexExpression,
            indexValue,
            error) ||
            !TryEvaluate(
                evaluator,
                parsed.assignment.valueExpression,
                assignedValue,
                error))
        {
            return false;
        }

        int index = 0;
        if (!TryToInteger(indexValue, index) ||
            !IsValidVariableIndex(parsed.assignment.prefix, index))
        {
            error = NCExpressionResolveError::INVALID_VARIABLE_INDEX;
            return false;
        }

        assignment.prefix = parsed.assignment.prefix;
        assignment.index = index;
        assignment.value = assignedValue;
        return true;
    }

    bool ResolveGoto(
        const NCParsedBlock& parsed,
        MacroParser& evaluator,
        NCGotoDecision& decision,
        NCExpressionResolveError& error)
    {
        decision = NCGotoDecision{};
        error = NCExpressionResolveError::NONE;

        if (parsed.error != NCParseError::NONE ||
            parsed.controlType != NCParsedControlType::GOTO)
        {
            error = NCExpressionResolveError::INVALID_PARSED_BLOCK;
            return false;
        }

        if (parsed.gotoStatement.conditional)
        {
            double condition = 0.0;
            if (!TryEvaluate(
                evaluator,
                parsed.gotoStatement.conditionExpression,
                condition,
                error))
            {
                return false;
            }

            if (condition == 0.0)
            {
                decision.shouldJump = false;
                return true;
            }
        }

        double targetValue = 0.0;
        if (!TryEvaluate(
            evaluator,
            parsed.gotoStatement.targetExpression,
            targetValue,
            error))
        {
            return false;
        }

        int target = 0;
        if (!TryToInteger(targetValue, target) || target < 0)
        {
            error = NCExpressionResolveError::INVALID_GOTO_TARGET;
            return false;
        }

        decision.shouldJump = true;
        decision.targetSequence = target;
        return true;
    }
}
