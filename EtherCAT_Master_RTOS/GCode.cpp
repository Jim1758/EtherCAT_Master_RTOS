#include "GMCodeHandlers.h"
#include "NCManager.h"      // 🌟 必須引入，才能使用 nc-> 的功能
#include "EtherCatMaster.h"
#include "GlobalConfig.h"   // 如果你有用到 DEBUG_PRINT 等功能
#include "AlarmManager.h"
#include "SHMManager.h"
#include <cmath>
#include <cstring>

namespace GCodeHandlers
{
    namespace
    {
        bool DecodeCutterSelector(const NCBlock& block, NCManager* nc,
            int selected, int& mode, int& dCode)
        {
            const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
            bool valid = nc != nullptr && count == 1 && block.mCount == 0 &&
                !block.isGoto && !block.isBlockSkip &&
                TryDecodeNCToolRadiusSelection(selected, block.has('D'), block.val('D'), mode, dCode);
            for (char word = 'A'; valid && word <= 'Z'; ++word)
                if (block.has(word) && ((word != 'N' && word != 'G' && word != 'D') ||
                    !std::isfinite(block.val(word)))) valid = false;
            return valid && nc->CoordSys.IsToolRadiusSelectionSupported(mode, dCode);
        }

        void RejectCutterParameter(NCManager* nc, const char* reason)
        {
            RtPrintf("[CUTTER][REJECT] reason=%s beforeCommit=1\n", reason);
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            if (nc != nullptr) nc->ChangeState(NCState::HOLD);
        }
    }

    WaitConditionFunc Handle_G12(const NCBlock& block, NCManager* nc)
    {
        // 程式能進到這裡，代表 isBarrier 已經成功攔截，
        // 底層倉庫 (cmdQueue) 已清空，馬達完全靜止。

        //DEBUG_PRINT("[NC] Executing G12: Buffer Flushed & Exact Stop.\n");

        // 既然只是為了中斷預讀，這裡不需要做任何事，
        // 直接回傳 nullptr，讓大腦繼續讀下一行。
        return nullptr;
    }
    WaitConditionFunc Handle_GCode(const NCBlock& block, NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        switch (block.gCode)
        {
        case 65: // 🌟 國際標準 G65：單次立即呼叫巨集 (立刻執行！)
        {
            int pVal = block.has('P') ? (int)block.val('P') : 0;
            int lVal = block.has('L') ? (int)block.val('L') : 1; // 讀取 L 次數，預設 1 次
            std::string macroFile = "O" + std::to_string(pVal) + ".nc";

            // 呼叫副程式
            if (nc->CallMacro(macroFile)) {
                // 賦予重複次數
                nc->m_macroStack.back().repeatCount = lVal;

                // 將 A~Z 參數對應到新一層的 #1~#26
                for (int i = 0; i < 26; i++) {
                    char c = 'A' + i;
                    if (c != 'P' && c != 'G' && c != 'L' && block.has(c)) {
                        nc->MacroSys.SetVar('#', i + 1, block.val(c));
                    }
                }
            }
            break;
        }

        case 66: // 🌟 國際標準 G66：開啟模態埋伏巨集 (不立刻執行！)
        {
            nc->m_isG66Active = true;
            nc->m_g66P = block.has('P') ? (int)block.val('P') : 0;
            nc->m_g66L = block.has('L') ? (int)block.val('L') : 1;
            nc->m_g66Block = block; // 把它存下來，稍後移動觸發時要把 A, B, C 傳進去
            break;
        }

        case 67: // 🌟 國際標準 G67：取消模態埋伏
        {
            nc->m_isG66Active = false;
            break;
        }

        case 54: case 55: case 56: case 57: case 58: case 59:
        case 154: case 155: case 156: case 157: case 158: case 159:
        case 254: case 255: case 256: case 257: case 258: case 259:
        case 354: case 355: case 356: case 357: case 358: case 359:
        case 454: case 455: case 456: case 457: case 458: case 459:
        case 554: case 555: case 556: case 557: case 558: case 559:
        case 654: case 655: case 656: case 657: case 658: case 659:
        case 754: case 755: case 756: case 757: case 758: case 759:
        case 854: case 855: case 856: case 857: case 858: case 859:
        case 954: case 955: case 956: case 957: case 958: case 959:
        {
            nc->GetCoordSys().SetWCS(block.gCode, nc);
        }
           
            break;

        case 17:  case 18:  case 19:
        {
            nc->CoordSys.SetActivePlane(block.gCode, nc);
        }
        break;

        case 20:  case 21:  
        {
            nc->CoordSys.SetUnitMode(block.gCode, nc);
        }
        break;

        case 22:
        case 23:
        {

            nc->CoordSys.SetStoredStrokeCheckMode(block.gCode,nc);
        }
        break;

        case 43:
        case 44:
        case 49:
        {
            int toolMode = 49;
            int hCode = 0;
            if (!TryDecodeNCToolLengthSelection(block.gCode, block.has('H'),
                    block.val('H'), toolMode, hCode) ||
                !nc->CoordSys.IsToolLengthSelectionSupported(toolMode, hCode))
            {
                RtPrintf("[TOOL][REJECT] g=%d reason=H_SELECTION beforeCommit=1\n", block.gCode);
                AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
                nc->ChangeState(NCState::HOLD);
                return nullptr;
            }
            nc->CoordSys.SetToolLengthCompensation(toolMode, hCode, nc);
        }
          
            break;
        case 90:
        case 91:
        {
            nc->CoordSys.Set_G90G91(block.gCode, nc);
        }
          
          
            break;

      

        case 92:
        {
            bool fields[8] = {};
            double targets[8] = {};
            if (!nc->CoordSys.TryDecodeG92Origin(block, nc, fields, targets))
            {
                RtPrintf("[ORIGIN][REJECT] g=92 reason=INPUT_OR_FRAME beforeCommit=1\n");
                AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
                nc->ChangeState(NCState::HOLD);
                return nullptr;
            }
            if (!nc->CoordSys.ApplyG92(fields, targets, nc)) return nullptr;
            break;
        }

        case 162:  
        {
            nc->CoordSys.SetCAxisOffsetRotationEnabled(true, nc);
        }
        break;
        case 163:
        {
            nc->CoordSys.SetCAxisOffsetRotationEnabled(false, nc);
        }
        break;
      

        default:
            break;
        }

        // 瞬間完成，回傳 nullptr 代表不用等
        return nullptr;
    }

    // ==========================================================
        // 🌟 G10 設定刀具補償值 (寫入 m_ToolOffset 表格)
        // 格式範例：G10 P1 X0.5 Y-0.2 Z10.0
        // ==========================================================
    WaitConditionFunc Handle_G10(const NCBlock& block, NCManager* nc)
    {
        if (block.has('L'))
        {
            const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
            bool valid = count == 1 && block.gCode == 10 && block.mCount == 0 &&
                !block.isGoto && !block.isBlockSkip && block.val('L') == 12.0 &&
                block.has('P') && block.has('R') && nc->CoordSys.toolRadiusMode == 40 &&
                !nc->CoordSys.IsTranslationRunFrozen();
            for (char word = 'A'; valid && word <= 'Z'; ++word)
                if (block.has(word) && ((word != 'N' && word != 'G' && word != 'L' &&
                    word != 'P' && word != 'R') || !std::isfinite(block.val(word)))) valid = false;
            const double row = block.val('P');
            const double radiusMM = nc->CoordSys.ToInternalUnit(block.val('R'), false);
            valid = valid && std::isfinite(row) && row >= 1.0 && row <= 100.0 &&
                row <= static_cast<double>(nc->CoordSys.m_ToolRadius.size()) && std::floor(row) == row &&
                std::isfinite(radiusMM) && radiusMM >= 0.0;
            if (!valid)
            {
                RejectCutterParameter(nc, "TABLE_SYNTAX");
                return [](NCManager*) { return true; };
            }
            const int dCode = static_cast<int>(row);
            if (nc->CoordSys.SetToolRadiusValue(dCode, radiusMM, nc))
                RtPrintf("[CUTTER][TABLE] D=%d storage=RAM_ONLY\n", dCode);
            return [](NCManager*) { return true; };
        }
        int arrayIndex = -1;
        bool writeFields[8] = {};
        double writeValues[8] = {};
        if (!nc->CoordSys.TryDecodeToolTableWrite(block, nc, arrayIndex,
            writeFields, writeValues))
        {
            RtPrintf("[TOOL][REJECT] g=10 reason=TABLE_INPUT beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        bool changed = false;
        for (unsigned axis = 0U; axis < 8U; ++axis)
            if (writeFields[axis] && std::memcmp(&writeValues[axis],
                &nc->CoordSys.m_ToolOffset[arrayIndex][axis], sizeof(double)) != 0) changed = true;
        if (!changed)
        {
            RtPrintf("[TOOL][TABLE] P=%d changed=0 saveRequested=0\n", arrayIndex + 1);
            return [](NCManager*) { return true; };
        }
        if (!nc->CoordSys.ApplyCoordinateTableValues(2, arrayIndex,
            writeFields, writeValues, nc)) return [](NCManager*) { return true; };
        // Preserve ordinary G10 persistence after the complete row commits.
        // This void API reports a request, not proof of successful disk I/O.
        nc->CoordSys.SaveToolOffset();
        RtPrintf("[TOOL][TABLE] P=%d changed=1 saveRequested=1\n", arrayIndex + 1);
        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // 🌟 G160 設定工件旋轉與平移補償值 (寫入 m_WorkOffset 表格)
    // 格式範例：G160 P1 X10. Y20. I45.0 J0.0 K0.0
    // (I=XY角度, J=XZ角度, K=YZ角度)
    // ==========================================================
    WaitConditionFunc Handle_G160(const NCBlock& block, NCManager* nc)
    {
        int arrayIndex = -1;
        bool writeFields[8] = {};
        double writeValues[8] = {};
        if (!nc->CoordSys.TryDecodeWorkTableWrite(block, arrayIndex, writeFields, writeValues))
        {
            RtPrintf("[WORK][REJECT] g=160 reason=TABLE_INPUT beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        if (!nc->CoordSys.ApplyCoordinateTableValues(3, arrayIndex,
            writeFields, writeValues, nc)) return [](NCManager*) { return true; };
        nc->CoordSys.SaveWorkOffset();
        RtPrintf("[G160] Work Offset P%d updated and saved.\n", arrayIndex + 1);
        return [](NCManager*) { return true; };
    }

    // ==========================================================
     // 🌟 G68 啟動 2D 座標旋轉
     // 格式範例：G17 G68 X0 Y0 R45. (R 為必填角度)
     // ==========================================================
    WaitConditionFunc Handle_G68(const NCBlock& block, NCManager* nc)
    {
        // 1. 防呆：G68 必須包含 R (角度)
        if (!block.has('R') || !std::isfinite(block.val('R'))) {
            RtPrintf("[ROTATION][REJECT] reason=ANGLE_REQUIRED beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }

        double angle = block.val('R');
        double centerPos[3] = { 0.0, 0.0, 0.0 };
        bool hasAxis[3] = { false, false, false };

        // 2. 抓取指定的旋轉圓心
        char axisNames[] = { 'X', 'Y', 'Z' };
        for (int i = 0; i < 3; i++) {
            if (block.has(axisNames[i])) {
                hasAxis[i] = true;
                centerPos[i] = nc->CoordSys.ToInternalUnit(block.val(axisNames[i]), false);
            }
        }

        // 3. 交給大腦執行設定
        nc->CoordSys.SetG68Rotation(centerPos, hasAxis, angle, nc);

        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // 🌟 G69 取消 2D 座標旋轉
    // ==========================================================
    WaitConditionFunc Handle_G69(const NCBlock& block, NCManager* nc)
    {
        nc->CoordSys.CancelG68Rotation(nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G168(const NCBlock& block, NCManager* nc)
    {
        int workMode = 169;
        int wCode = 0;
        bool valid = TryDecodeNCWorkSelection(168, block.has('W'),
            block.val('W'), workMode, wCode);
        if (nc->CoordSys.IsTranslationRunBound())
        {
            NCArcPlaneAxes plane{};
            valid = valid && TryGetNCArcPlaneAxes(nc->CoordSys.activePlane, plane);
            valid = valid && nc->CoordSys.IsWorkpieceSelectionSupported(workMode, wCode) &&
                block.gCount == 1 && block.mCount == 0;
            for (char letter = 'A'; letter <= 'Z'; ++letter)
                if (block.has(letter) && letter != 'G' && letter != 'N' && letter != 'W' &&
                    letter != plane.uAddress && letter != plane.vAddress) valid = false;
            valid = valid && block.has(plane.uAddress) == block.has(plane.vAddress) &&
                (!block.has(plane.uAddress) || (std::isfinite(block.val(plane.uAddress)) &&
                    std::isfinite(block.val(plane.vAddress))));
            if (valid)
            {
                const unsigned angleField = nc->CoordSys.activePlane == 18 ?
                    CoordinateManager::WO_ANGLE_XZ_PITCH : (nc->CoordSys.activePlane == 19 ?
                        CoordinateManager::WO_ANGLE_YZ_ROLL : CoordinateManager::WO_ANGLE_XY_YAW);
                const bool hasPlaneRotation =
                    nc->CoordSys.m_WorkOffset[wCode - 1][angleField] != 0.0;
                const bool sameWork = nc->CoordSys.isWorkpieceRotationActive && nc->CoordSys.currentWCode == wCode;
                valid = hasPlaneRotation ? (nc->CoordSys.IsTranslationRunCurrent() &&
                    (block.has(plane.uAddress) || sameWork)) : !block.has(plane.uAddress);
            }
        }
        if (!valid)
        {
            RtPrintf("[WORK][REJECT] g=168 reason=W_SELECTION_OR_SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        bool hasAxis[8] = {};
        double targetWCS[8] = {};
        const char axisNames[3] = { 'X', 'Y', 'Z' };
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            hasAxis[axis] = block.has(axisNames[axis]);
            if (hasAxis[axis]) targetWCS[axis] =
                nc->CoordSys.ToInternalUnit(block.val(axisNames[axis]), false);
        }
        nc->CoordSys.SetWorkpieceRotation(wCode, hasAxis, targetWCS, nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G169(const NCBlock& block, NCManager* nc)
    {
        int workMode = 169;
        int wCode = 0;
        if (!TryDecodeNCWorkSelection(169, block.has('W'), block.val('W'), workMode, wCode))
        {
            RtPrintf("[WORK][REJECT] g=169 reason=W_SELECTION beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.CancelWorkpieceRotation(nc);
        return [](NCManager*) { return true; };
    }


    // ==========================================================
    // 🌟 G51 縮放
    // 格式：G51 X0 Y0 Z0 P2.0 (P 為放大兩倍)
    // ==========================================================
    // Centers are authored XYZ lengths; P is dimensionless. NC runs require
    // explicit G51 XYZ P so publication and the normal handler share one pivot.
    bool DecodeScaleMirrorBlock(const NCBlock& block, NCManager* nc, int code,
        double* values, bool* hasAxis, double& factor)
    {
        if (!nc || !values || !hasAxis) return false;
        factor = code == 51 && block.has('P') ? block.val('P') : 1.0;
        if (!std::isfinite(factor) || factor <= 0.0 ||
            ((code == 51 || code == 151) &&
                (block.gCount != 1 || block.gCodes[0] != code || block.mCount != 0))) return false;
        const bool bound = nc->CoordSys.IsTranslationRunBound();
        if (code == 51 && bound && (!block.has('P') || !block.has('X') ||
            !block.has('Y') || !block.has('Z'))) return false;
        bool any = false;
        for (char letter = 'A'; letter <= 'Z'; ++letter)
        {
            if (!block.has(letter)) continue;
            const bool axis = letter == 'X' || letter == 'Y' || letter == 'Z';
            if ((letter != 'G' && letter != 'N' && !(axis && code != 50) &&
                !(letter == 'P' && code == 51)) || !std::isfinite(block.val(letter))) return false;
        }
        for (unsigned axis = 0U; axis < 8U; ++axis)
        {
            hasAxis[axis] = axis < 3U && block.has("XYZ"[axis]);
            values[axis] = hasAxis[axis] ?
                nc->CoordSys.ToInternalUnit(block.val("XYZ"[axis]), false) : 0.0;
            if (!std::isfinite(values[axis])) return false;
            any = any || hasAxis[axis];
        }
        return code != 151 || any;
    }

    WaitConditionFunc Handle_G51(const NCBlock& block, NCManager* nc)
    {
        double factor = 1.0, center[8] = {};
        bool selected[8] = {};
        if (!DecodeScaleMirrorBlock(block, nc, 51, center, selected, factor))
        {
            RtPrintf("[SCALE-MIRROR][REJECT] g=51 reason=XYZ_P_SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.SetScaling(center, selected, factor, nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G50(const NCBlock& block, NCManager* nc)
    {
        double factor = 1.0, values[8] = {};
        bool selected[8] = {};
        if (!DecodeScaleMirrorBlock(block, nc, 50, values, selected, factor))
        {
            RtPrintf("[SCALE-MIRROR][REJECT] g=50 reason=SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.CancelScaling(nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G151(const NCBlock& block, NCManager* nc)
    {
        double factor = 1.0, center[8] = {};
        bool selected[8] = {};
        if (!DecodeScaleMirrorBlock(block, nc, 151, center, selected, factor))
        {
            RtPrintf("[SCALE-MIRROR][REJECT] g=151 reason=XYZ_SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.SetMirror(center, selected, nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G150(const NCBlock& block, NCManager* nc)
    {
        double factor = 1.0, values[8] = {};
        bool selected[8] = {};
        if (!DecodeScaleMirrorBlock(block, nc, 150, values, selected, factor))
        {
            RtPrintf("[SCALE-MIRROR][REJECT] g=150 reason=XYZ_SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.CancelMirror(selected, nc);
        return [](NCManager*) { return true; };
    }

    bool ValidatePolarSelectionBlock(const NCBlock& block, int code)
    {
        if ((code != 15 && code != 16) || block.gCount != 1 ||
            block.gCodes[0] != code || block.mCount != 0) return false;
        for (char letter = 'A'; letter <= 'Z'; ++letter)
            if (block.has(letter) && ((letter != 'G' && letter != 'N') ||
                !std::isfinite(block.val(letter)))) return false;
        return true;
    }

    WaitConditionFunc Handle_G16(const NCBlock& block, NCManager* nc)
    {
        if (!ValidatePolarSelectionBlock(block, 16))
        {
            RtPrintf("[POLAR][REJECT] g=16 reason=STANDALONE_SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.SetPolarCoordinate(nc);
        return [](NCManager*) { return true; };
    }
    WaitConditionFunc Handle_G15(const NCBlock& block, NCManager* nc)
    {
        if (!ValidatePolarSelectionBlock(block, 15))
        {
            RtPrintf("[POLAR][REJECT] g=15 reason=STANDALONE_SYNTAX beforeCommit=1\n");
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::HOLD);
            return [](NCManager*) { return true; };
        }
        nc->CoordSys.CancelPolarCoordinate(nc);
        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // G41 / G42 select a physical radius row. Entry is the following G01 XY.
    // ==========================================================
    WaitConditionFunc Handle_G41(const NCBlock& block, NCManager* nc) {
        int mode = 40, dCode = 0;
        if (!DecodeCutterSelector(block, nc, 41, mode, dCode))
            RejectCutterParameter(nc, "D_SELECTION");
        else nc->CoordSys.SetToolRadiusCompensation(mode, dCode, nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G42(const NCBlock& block, NCManager* nc) {
        int mode = 40, dCode = 0;
        if (!DecodeCutterSelector(block, nc, 42, mode, dCode))
            RejectCutterParameter(nc, "D_SELECTION");
        else nc->CoordSys.SetToolRadiusCompensation(mode, dCode, nc);
        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // 🌟 G40 取消刀補
    // ==========================================================
    WaitConditionFunc Handle_G40(const NCBlock& block, NCManager* nc) {
        int mode = 40, dCode = 0;
        if (!TryDecodeNCToolRadiusSelection(40, block.has('D'), block.val('D'), mode, dCode))
            RejectCutterParameter(nc, "CANCEL_SELECTION");
        else nc->CoordSys.CancelToolRadiusCompensation(nc);
        return [](NCManager*) { return true; };
    }

} // end namespace