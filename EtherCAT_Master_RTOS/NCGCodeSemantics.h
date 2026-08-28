#pragma once
#include "NC_Types.h"
#include <cstdint>

// =============================================================================
// Stage NC-0.2A - G-code semantic / modal-group contract
//
// Parser 只負責把一行拆成 Word；本層負責：
//   1. 判斷 G-code 是否為目前支援的語意。
//   2. 同一 Modal Group 衝突檢查（例如 G90 G91）。
//   3. 限制同一 Block 最多一個 Primary Action。
//   4. 建立固定容量、可預測的執行順序。
//
// 全部使用固定陣列，不配置動態記憶體。
// =============================================================================

enum class NCGCodeModalGroup : std::uint8_t
{
    NONE = 0,
    MOTION,
    PLANE,
    DISTANCE,
    UNITS,
    WORK_COORDINATE,
    STORED_STROKE,
    TOOL_LENGTH,
    CUTTER_RADIUS,
    COORD_ROTATION_2D,
    WORKPIECE_ROTATION_3D,
    SCALING,
    MIRROR,
    POLAR,
    C_AXIS_OFFSET_ROTATION,
    MODAL_MACRO,
    COUNT
};

enum class NCGCodeRole : std::uint8_t
{
    SETTING = 0,
    PRIMARY_ACTION
};

enum class NCGCodePlanError : std::uint8_t
{
    NONE = 0,
    TOO_MANY_G_CODES,
    UNSUPPORTED_G_CODE,
    MODAL_GROUP_CONFLICT,
    MULTIPLE_PRIMARY_ACTIONS
};

struct NCGCodeDescriptor
{
    int code = -1;
    NCGCodeModalGroup group = NCGCodeModalGroup::NONE;
    NCGCodeRole role = NCGCodeRole::SETTING;
    int dispatchOrder = 0;
    bool barrier = false;
    bool motionAction = false;
    bool suppressesImplicitMotion = false;
};

struct NCGCodeExecutionPlan
{
    int orderedCodes[NC_MAX_G_CODES_PER_BLOCK] = { 0 };
    int count = 0;
    bool hasPrimaryAction = false;
    int primaryActionCode = -1;
};

namespace NCGCodeSemantics
{
    bool TryGetDescriptor(
        int code,
        NCGCodeDescriptor& descriptor) noexcept;

    bool BuildExecutionPlan(
        const NCBlock& block,
        NCGCodeExecutionPlan& plan,
        NCGCodePlanError& error,
        int& firstConflictCode,
        int& secondConflictCode) noexcept;

    bool Contains(
        const NCBlock& block,
        int code) noexcept;

    bool IsBlockBarrier(
        const NCBlock& block) noexcept;

    bool IsMotionAction(
        int code) noexcept;

    int GetPrimaryActionCode(
        const NCBlock& block) noexcept;

    bool BlockSuppressesImplicitMotion(
        const NCBlock& block) noexcept;
}
