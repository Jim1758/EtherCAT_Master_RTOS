#include "GMCodeHandlers.h"
#include "NCManager.h"      // 🌟 必須引入，才能使用 nc-> 的功能
#include "EtherCatMaster.h"
#include "GlobalConfig.h"   // 如果你有用到 DEBUG_PRINT 等功能
#include "AlarmManager.h"
#include "SHMManager.h"

namespace GCodeHandlers
{

    WaitConditionFunc Handle_GCode(const NCBlock& block, NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        switch (block.gCode)
        {
        case 65: // 巨集呼叫處理區塊 (G65 P___ A___ B___)
        {   // 🌟 C++ 規定：在 case 內宣告變數，必須加上大括號限制作用域
            int pVal = block.has('P') ? (int)block.val('P') : 0;
            std::string macroFile = "O" + std::to_string(pVal) + ".nc";

            // 呼叫副程式
            if (nc->CallMacro(macroFile)) {
                // 如果呼叫成功，把 A~Z 參數對應到新一層的 #1~#26
                for (int i = 0; i < 26; i++) {
                    char c = 'A' + i;
                    // P 是檔名，G 是 G 碼本身，不當作變數傳入
                    if (c != 'P' && c != 'G' && block.has(c)) {
                        // 依照字母順序 A=#1, B=#2, C=#3... 寫入這層的區域變數
                        nc->MacroSys.SetVar('#', i + 1, block.val(c));
                    }
                }
            }
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

        case 43:
        case 44:
        case 49:
        {  int hCode = block.val('H');
        nc->CoordSys.SetToolLengthCompensation(block.gCode, hCode, nc); // 設定模式為 43，H碼為1
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
            // 1. 準備空陣列給 CoordinateManager
            bool axisProgrammed[8] = { false };
            double axisTarget[8] = { 0.0 };

            bool hasAnyAxis = false; // 用來檢查這行 G92 到底有沒有帶任何軸座標

            // 🌟 2. 直接走訪機台定義的 8 個軸
            for (int i = 0; i < 8; i++)
            {
                char axisLetter = nc->m_axisNames[i]; // 拿出設定檔定義的字母 (如 'X', 'Y', 'Z')

                // 防呆：如果這個軸未啟用 (空白字元) 則跳過
                if (axisLetter == ' ' || axisLetter == '\0' || axisLetter == 'N') continue;

                // 🌟 3. 利用你 NCBlock 寫好的神級 API，一句話完成判斷與取值！
                if (block.has(axisLetter))
                {
                    axisProgrammed[i] = true;
                    axisTarget[i] = block.val(axisLetter);
                    hasAnyAxis = true;
                }
            }

            // 4. 如果有讀到至少一個軸，就呼叫底層計算並覆寫表格
            if (hasAnyAxis)
            {
                nc->CoordSys.ApplyG92(axisProgrammed, axisTarget);

                // 💡 提示：如果需要，可以在這裡補上觸發 HMI 存檔的旗標
                 //pShm->Coord_Command.reqSave = 1;
                 //pShm->Coord_Command.saveType = 3; // 3 代表 WCS
            }
            else
            {
                // 報警：下達了 G92 卻沒有給任何座標
                printf("[Warning] G92 executed without any valid axis coordinate.\n");
                AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            }

            break;
        }

        case 162:  
        {
            nc->CoordSys.isCAxisOffsetRotationEnabled = true;
        }
        break;
        case 163:
        {
            nc->CoordSys.isCAxisOffsetRotationEnabled = false;
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
        // 防呆：必須指定 P (刀號)
        if (!block.has('P')) {
            RtPrintf(">>> [ALARM] G10 missing 'P' (Tool Index) parameter!\n");
            return [](NCManager*) { return true; };
        }

        int pCode = (int)block.val('P');
        int arrayIndex = pCode - 1; // P1 對應陣列 [0]

        // 防呆：檢查陣列範圍
        if (arrayIndex >= 0 && arrayIndex < nc->CoordSys.m_ToolOffset.size())
        {
            // 動態掃描機台啟用的 8 個軸
            for (int i = 0; i < 8; i++) {
                char axisLetter = nc->m_axisNames[i];
                if (axisLetter != ' ' && axisLetter != '\0' && axisLetter != 'N') {
                    // 如果 G 碼有下達這個軸，就覆寫表格內的數值
                    if (block.has(axisLetter)) {
                        nc->CoordSys.m_ToolOffset[arrayIndex][i] = block.val(axisLetter);
                    }
                }
            }

            // 🌟 貼心功能：設定完自動存檔，確保重開機數值還在
            nc->CoordSys.SaveToolOffset();
            RtPrintf("[G10] Tool Offset P%d updated and saved.\n", pCode);
        }
        else {
            RtPrintf(">>> [ALARM] G10 P%d is out of range!\n", pCode);
        }

        return [](NCManager*) { return true; }; // 瞬間設定完成
    }

    // ==========================================================
    // 🌟 G160 設定工件旋轉與平移補償值 (寫入 m_WorkOffset 表格)
    // 格式範例：G160 P1 X10. Y20. I45.0 J0.0 K0.0
    // (I=XY角度, J=XZ角度, K=YZ角度)
    // ==========================================================
    WaitConditionFunc Handle_G160(const NCBlock& block, NCManager* nc)
    {
        if (!block.has('P')) {
            RtPrintf(">>> [ALARM] G160 missing 'P' (Work Offset Index) parameter!\n");
            return [](NCManager*) { return true; };
        }

        int pCode = (int)block.val('P');
        int arrayIndex = pCode - 1; // P1 對應陣列 [0]

        if (arrayIndex >= 0 && arrayIndex < nc->CoordSys.m_WorkOffset.size())
        {
            // 1. 設定平移量 (X, Y, Z 等真實軸)
            for (int i = 0; i < 8; i++) {
                char axisLetter = nc->m_axisNames[i];
                if (axisLetter != ' ' && axisLetter != '\0' && axisLetter != 'N') {
                    if (block.has(axisLetter)) {
                        nc->CoordSys.m_WorkOffset[arrayIndex][i] = block.val(axisLetter);
                    }
                }
            }

            // 2. 設定旋轉角度 (I, J, K)
            // 對應 CoordinateManager::WorkOffsetField 的 3, 4, 5
            if (block.has('I')) {
                nc->CoordSys.m_WorkOffset[arrayIndex][3] = block.val('I');
            }
            if (block.has('J')) {
                nc->CoordSys.m_WorkOffset[arrayIndex][4] = block.val('J');
            }
            if (block.has('K')) {
                nc->CoordSys.m_WorkOffset[arrayIndex][5] = block.val('K');
            }

            // 🌟 設定完自動存檔
            nc->CoordSys.SaveWorkOffset();
            RtPrintf("[G160] Work Offset P%d updated and saved.\n", pCode);
        }
        else {
            RtPrintf(">>> [ALARM] G160 P%d is out of range!\n", pCode);
        }

        return [](NCManager*) { return true; };
    }

    // ==========================================================
     // 🌟 G68 啟動 2D 座標旋轉
     // 格式範例：G17 G68 X0 Y0 R45. (R 為必填角度)
     // ==========================================================
    WaitConditionFunc Handle_G68(const NCBlock& block, NCManager* nc)
    {
        // 1. 防呆：G68 必須包含 R (角度)
        if (!block.has('R')) {
            RtPrintf(">>> [ALARM] G68 missing 'R' (Angle) parameter!\n");
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
                centerPos[i] = block.val(axisNames[i]);
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
        // 防呆：必須有下達 W 參數
        if (!block.has('W')) {
            RtPrintf(">>> [ALARM] G168 missing 'W' parameter!\n");
            // 可在此觸發 AlarmManager
            return [](NCManager*) { return true; };
        }

        int wCode = (int)block.val('W');

        bool hasAxis[8] = { false };
        double targetWCS[8] = { 0.0 };

        // 檢查 X, Y, Z 是否有被賦值
        char axisNames[] = { 'X', 'Y', 'Z' };
        for (int i = 0; i < 3; i++) {
            if (block.has(axisNames[i])) {
                hasAxis[i] = true;
                targetWCS[i] = block.val(axisNames[i]);
            }
        }

        // 呼叫 CoordinateManager 執行旋轉設定
        nc->CoordSys.SetWorkpieceRotation(wCode, hasAxis, targetWCS, nc);

        return [](NCManager*) { return true; }; // 瞬間設定完成，繼續下一行
    }

    // ==========================================================
    // 🌟 G169 取消工件旋轉
    // 格式：G169
    // ==========================================================
    WaitConditionFunc Handle_G169(const NCBlock& block, NCManager* nc)
    {
        nc->CoordSys.CancelWorkpieceRotation(nc);
        return [](NCManager*) { return true; };
    }


    // ==========================================================
    // 🌟 G51 縮放
    // 格式：G51 X0 Y0 P2.0 (P 為放大兩倍)
    // ==========================================================
    WaitConditionFunc Handle_G51(const NCBlock& block, NCManager* nc) {
        double factor = block.has('P') ? block.val('P') : 1.0;

        bool hasAxis[8] = { false };
        double centerPos[8] = { 0.0 };
        for (int i = 0; i < 8; i++) {
            char axisLetter = nc->m_axisNames[i];
            if (axisLetter != ' ' && block.has(axisLetter)) {
                hasAxis[i] = true;
                centerPos[i] = block.val(axisLetter);
            }
        }
        nc->CoordSys.SetScaling(centerPos, hasAxis, factor, nc);
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G50(const NCBlock& block, NCManager* nc) {
        nc->CoordSys.CancelScaling(nc);
        return [](NCManager*) { return true; };
    }

    // ==========================================================
     // 🌟 G151 啟動鏡像 (自定義取代 G51.1)
     // 格式：G151 X100. (以 X=100 為對稱軸進行鏡像)
     // ==========================================================
    WaitConditionFunc Handle_G151(const NCBlock& block, NCManager* nc) {
        bool hasAxis[8] = { false };
        double mirrorPos[8] = { 0.0 };

        for (int i = 0; i < 8; i++) {
            char axisLetter = nc->m_axisNames[i];
            // 抓出操作員下達的對稱中心座標
            if (axisLetter != ' ' && block.has(axisLetter)) {
                hasAxis[i] = true;
                mirrorPos[i] = block.val(axisLetter);
            }
        }

        nc->CoordSys.SetMirror(mirrorPos, hasAxis, nc);
        RtPrintf("[G151] Mirror Image ON.\n");

        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // 🌟 G150 關閉鏡像 (自定義取代 G50.1)
    // 格式：G150 X (只取消 X 軸)，或單下 G150 (全部取消)
    // ==========================================================
    WaitConditionFunc Handle_G150(const NCBlock& block, NCManager* nc) {
        bool hasAxis[8] = { false };

        for (int i = 0; i < 8; i++) {
            char axisLetter = nc->m_axisNames[i];
            if (axisLetter != ' ' && block.has(axisLetter)) {
                hasAxis[i] = true;
            }
        }

        nc->CoordSys.CancelMirror(hasAxis, nc);
        RtPrintf("[G150] Mirror Image OFF.\n");

        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G16(const NCBlock& block, NCManager* nc) {
        nc->CoordSys.SetPolarCoordinate(nc);
        return [](NCManager*) { return true; };
    }
    WaitConditionFunc Handle_G15(const NCBlock& block, NCManager* nc) {
        nc->CoordSys.CancelPolarCoordinate(nc);
        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // 🌟 G41 左刀補 / G42 右刀補
    // 格式範例：G41 D1 X10. Y10.
    // ==========================================================
    WaitConditionFunc Handle_G41(const NCBlock& block, NCManager* nc) {
        if (block.has('D')) {
            nc->CoordSys.SetToolRadiusCompensation(41, (int)block.val('D'), nc);
        }
        else {
            // 如果沒寫 D，通常繼承上一次的 D 碼
            nc->CoordSys.SetToolRadiusCompensation(41, nc->CoordSys.currentDCode, nc);
        }
        return [](NCManager*) { return true; };
    }

    WaitConditionFunc Handle_G42(const NCBlock& block, NCManager* nc) {
        if (block.has('D')) {
            nc->CoordSys.SetToolRadiusCompensation(42, (int)block.val('D'), nc);
        }
        else {
            nc->CoordSys.SetToolRadiusCompensation(42, nc->CoordSys.currentDCode, nc);
        }
        return [](NCManager*) { return true; };
    }

    // ==========================================================
    // 🌟 G40 取消刀補
    // ==========================================================
    WaitConditionFunc Handle_G40(const NCBlock& block, NCManager* nc) {
        nc->CoordSys.CancelToolRadiusCompensation(nc);
        return [](NCManager*) { return true; };
    }

} // end namespace