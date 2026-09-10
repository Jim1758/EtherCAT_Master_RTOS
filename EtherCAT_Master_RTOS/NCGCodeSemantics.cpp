#include "NCGCodeSemantics.h"
#include <algorithm>

namespace
{
    NCGCodeDescriptor MakeDescriptor(
        int code,
        NCGCodeModalGroup group,
        NCGCodeRole role,
        int dispatchOrder,
        bool barrier,
        bool motionAction,
        bool suppressesImplicitMotion) noexcept
    {
        NCGCodeDescriptor descriptor{};
        descriptor.code = code;
        descriptor.group = group;
        descriptor.role = role;
        descriptor.dispatchOrder = dispatchOrder;
        descriptor.barrier = barrier;
        descriptor.motionAction = motionAction;
        descriptor.suppressesImplicitMotion = suppressesImplicitMotion;
        return descriptor;
    }

    bool IsExtendedWorkCoordinateCode(int code) noexcept
    {
        if (code < 54 || code > 959)
        {
            return false;
        }

        const int suffix = code % 100;
        const int prefix = code / 100;

        return
            suffix >= 54 &&
            suffix <= 59 &&
            prefix >= 0 &&
            prefix <= 9;
    }

    int SafeStoredGCodeCount(const NCBlock& block) noexcept
    {
        if (block.gCount <= 0)
        {
            return block.hasG ? 1 : 0;
        }

        return
            block.gCount < NC_MAX_G_CODES_PER_BLOCK
            ? block.gCount
            : NC_MAX_G_CODES_PER_BLOCK;
    }

    int GetStoredGCode(const NCBlock& block, int index) noexcept
    {
        if (block.gCount <= 0)
        {
            return block.gCode;
        }

        return block.gCodes[index];
    }
}

namespace NCGCodeSemantics
{
    bool TryGetDescriptor(
        int code,
        NCGCodeDescriptor& descriptor) noexcept
    {
        // 執行順序：
        // Units -> Plane -> Distance -> WCS -> Compensation / Transform
        // -> Primary Action。
        // Primary Action 最多一個，避免同一組 X/Y/Z 被兩個動作同時解讀。

        if (IsExtendedWorkCoordinateCode(code))
        {
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::WORK_COORDINATE,
                NCGCodeRole::SETTING,
                40,
                true,
                false,
                false);
            return true;
        }

        switch (code)
        {
            // ---------------------------------------------------------
            // Primary motion / machine actions
            // ---------------------------------------------------------
        case 0:
        case 1: // BX: explicit G01 feed line, mm/min.
        case 2: // BY-ARC: explicit G17 XY arc, exact stop.
        case 3:
        case 7:
        case 28:
        case 30:
        case 32:
        case 53:
        case 81:
        case 161:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::MOTION,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                true,
                true,
                false);
            return true;

            // ---------------------------------------------------------
            // One-shot / parameter-owning actions
            // ---------------------------------------------------------
// BR-BEGIN
        case 171: // Explicit saved-start positioning, nonmodal; F is G00 percent.
        case 172: // BT: L selects a frozen suffix on the first one-shot step.
        case 173: // BV: one saved-end step in the frozen forward traversal.
        case 174: // BZ: retained G01/arc reverse, explicit F mm/min.
        case 175: // BZ: retained G01/arc forward after full retreat.
        case 176: // CA: saved canonical interval retreat, D mm / F mm/min.
        case 177: // CA: saved canonical interval advance, D mm / F mm/min.
        case 178: // CB: arm one original-source Feed Hold excursion.
        case 179: // CB: cancel an unused one-shot arm.
            descriptor = MakeDescriptor(code, NCGCodeModalGroup::NONE,
                NCGCodeRole::PRIMARY_ACTION, 200, true, true, true);
            return true;
            // BR-END
        case 4:   // Dwell
        case 10:  // Tool offset write
        case 12:  // Exact-stop / pre-read barrier
        case 65:  // One-shot macro
        case 92:  // Coordinate preset
        case 160: // Work offset write
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::NONE,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                code == 4 || code == 12 || code == 65 || code == 92,
                false,
                true);
            return true;

        case 66:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::MODAL_MACRO,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                true,
                false,
                true);
            return true;

        case 68:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::COORD_ROTATION_2D,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                false,
                false,
                true);
            return true;

        case 168:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::WORKPIECE_ROTATION_3D,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                false,
                false,
                true);
            return true;

        case 51:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::SCALING,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                false,
                false,
                true);
            return true;

        case 150:
        case 151:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::MIRROR,
                NCGCodeRole::PRIMARY_ACTION,
                200,
                false,
                false,
                true);
            return true;

            // ---------------------------------------------------------
            // Compatible modal settings
            // ---------------------------------------------------------
        case 20:
        case 21:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::UNITS,
                NCGCodeRole::SETTING,
                10,
                true,
                false,
                false);
            return true;

        case 17:
        case 18:
        case 19:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::PLANE,
                NCGCodeRole::SETTING,
                20,
                false,
                false,
                false);
            return true;

        case 90:
        case 91:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::DISTANCE,
                NCGCodeRole::SETTING,
                30,
                false,
                false,
                false);
            return true;

        case 22:
        case 23:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::STORED_STROKE,
                NCGCodeRole::SETTING,
                50,
                true,
                false,
                false);
            return true;

        case 43:
        case 44:
        case 49:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::TOOL_LENGTH,
                NCGCodeRole::SETTING,
                60,
                false,
                false,
                false);
            return true;

        case 40:
        case 41:
        case 42:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::CUTTER_RADIUS,
                NCGCodeRole::SETTING,
                70,
                false,
                false,
                false);
            return true;

        case 50:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::SCALING,
                NCGCodeRole::SETTING,
                80,
                false,
                false,
                true);
            return true;

        case 15:
        case 16:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::POLAR,
                NCGCodeRole::SETTING,
                90,
                false,
                false,
                false);
            return true;

        case 162:
        case 163:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::C_AXIS_OFFSET_ROTATION,
                NCGCodeRole::SETTING,
                100,
                false,
                false,
                false);
            return true;

        case 169:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::WORKPIECE_ROTATION_3D,
                NCGCodeRole::SETTING,
                110,
                false,
                false,
                true);
            return true;

        case 69:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::COORD_ROTATION_2D,
                NCGCodeRole::SETTING,
                120,
                false,
                false,
                true);
            return true;

        case 67:
            descriptor = MakeDescriptor(
                code,
                NCGCodeModalGroup::MODAL_MACRO,
                NCGCodeRole::SETTING,
                130,
                true,
                false,
                true);
            return true;

        default:
            descriptor = NCGCodeDescriptor{};
            return false;
        }
    }

    bool BuildExecutionPlan(
        const NCBlock& block,
        NCGCodeExecutionPlan& plan,
        NCGCodePlanError& error,
        int& firstConflictCode,
        int& secondConflictCode) noexcept
    {
        plan = NCGCodeExecutionPlan{};
        error = NCGCodePlanError::NONE;
        firstConflictCode = -1;
        secondConflictCode = -1;

        if (!block.hasG && block.gCount <= 0)
        {
            return true;
        }

        if (block.gCount > NC_MAX_G_CODES_PER_BLOCK)
        {
            error = NCGCodePlanError::TOO_MANY_G_CODES;
            return false;
        }

        int groupCode[static_cast<int>(NCGCodeModalGroup::COUNT)];
        std::fill_n(
            groupCode,
            static_cast<int>(NCGCodeModalGroup::COUNT),
            -1);

        const int count = SafeStoredGCodeCount(block);

        for (int i = 0; i < count; ++i)
        {
            const int code = GetStoredGCode(block, i);
            NCGCodeDescriptor descriptor{};

            if (!TryGetDescriptor(code, descriptor))
            {
                error = NCGCodePlanError::UNSUPPORTED_G_CODE;
                firstConflictCode = code;
                return false;
            }

            if (descriptor.group != NCGCodeModalGroup::NONE)
            {
                const int groupIndex =
                    static_cast<int>(descriptor.group);

                if (groupCode[groupIndex] >= 0)
                {
                    error = NCGCodePlanError::MODAL_GROUP_CONFLICT;
                    firstConflictCode = groupCode[groupIndex];
                    secondConflictCode = code;
                    return false;
                }

                groupCode[groupIndex] = code;
            }

            if (descriptor.role == NCGCodeRole::PRIMARY_ACTION)
            {
                if (plan.hasPrimaryAction)
                {
                    error = NCGCodePlanError::MULTIPLE_PRIMARY_ACTIONS;
                    firstConflictCode = plan.primaryActionCode;
                    secondConflictCode = code;
                    return false;
                }

                plan.hasPrimaryAction = true;
                plan.primaryActionCode = code;
            }

            plan.orderedCodes[plan.count++] = code;
        }

        // 固定容量 Stable Insertion Sort。
        // 相同 priority 保留原程式書寫順序。
        for (int i = 1; i < plan.count; ++i)
        {
            const int currentCode = plan.orderedCodes[i];
            NCGCodeDescriptor currentDescriptor{};
            TryGetDescriptor(currentCode, currentDescriptor);

            int j = i - 1;
            while (j >= 0)
            {
                NCGCodeDescriptor previousDescriptor{};
                TryGetDescriptor(
                    plan.orderedCodes[j],
                    previousDescriptor);

                if (previousDescriptor.dispatchOrder <=
                    currentDescriptor.dispatchOrder)
                {
                    break;
                }

                plan.orderedCodes[j + 1] =
                    plan.orderedCodes[j];
                --j;
            }

            plan.orderedCodes[j + 1] = currentCode;
        }

        return true;
    }

    bool Contains(
        const NCBlock& block,
        int code) noexcept
    {
        const int count = SafeStoredGCodeCount(block);
        for (int i = 0; i < count; ++i)
        {
            if (GetStoredGCode(block, i) == code)
            {
                return true;
            }
        }

        return false;
    }

    bool IsBlockBarrier(
        const NCBlock& block) noexcept
    {
        const int count = SafeStoredGCodeCount(block);

        for (int i = 0; i < count; ++i)
        {
            const int code = GetStoredGCode(block, i);
            NCGCodeDescriptor descriptor{};

            if (!TryGetDescriptor(code, descriptor) ||
                !descriptor.barrier)
            {
                continue;
            }

            // 既有 G00 P1 為連續 Buffer 模式；只取消 G00 自己的 Barrier。
            // 同一行若還有 WCS / Unit 等其他 Barrier，仍必須等待舊 Queue 清空。
            if (code == 0 &&
                block.has('P') &&
                block.val('P') == 1.0)
            {
                continue;
            }

            return true;
        }

        return false;
    }

    bool IsMotionAction(int code) noexcept
    {
        NCGCodeDescriptor descriptor{};
        return
            TryGetDescriptor(code, descriptor) &&
            descriptor.role == NCGCodeRole::PRIMARY_ACTION &&
            descriptor.motionAction;
    }

    int GetPrimaryActionCode(
        const NCBlock& block) noexcept
    {
        const int count = SafeStoredGCodeCount(block);
        for (int i = 0; i < count; ++i)
        {
            const int code = GetStoredGCode(block, i);
            NCGCodeDescriptor descriptor{};

            if (TryGetDescriptor(code, descriptor) &&
                descriptor.role == NCGCodeRole::PRIMARY_ACTION)
            {
                return code;
            }
        }

        return -1;
    }

    bool BlockSuppressesImplicitMotion(
        const NCBlock& block) noexcept
    {
        const int count = SafeStoredGCodeCount(block);
        for (int i = 0; i < count; ++i)
        {
            NCGCodeDescriptor descriptor{};
            if (TryGetDescriptor(
                GetStoredGCode(block, i),
                descriptor) &&
                descriptor.suppressesImplicitMotion)
            {
                return true;
            }
        }

        return false;
    }
}
