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
            nc->GetCoordSys().SetWCS(block.gCode, nc);
            break;


        case 90:
            
            nc->GetCoordSys().isAbsoluteMode = true;
            nc->MacroSys.SetVar('$', 3, block.gCode);
            DEBUG_PRINT("[NC] -> Absolute Mode (G90) Active\n");
            break;

        case 91:
            nc->GetCoordSys().isAbsoluteMode = false;
            nc->MacroSys.SetVar('$', 3, block.gCode);
            DEBUG_PRINT("[NC] -> Incremental Mode (G91) Active\n");
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
          

        default:
            break;
        }

        // 瞬間完成，回傳 nullptr 代表不用等
        return nullptr;
    }

} // end namespace