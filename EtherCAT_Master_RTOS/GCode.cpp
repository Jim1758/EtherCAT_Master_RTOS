#include "GMCodeHandlers.h"
#include "NCManager.h"      // 🌟 必須引入，才能使用 nc-> 的功能
#include "EtherCatMaster.h"
#include "GlobalConfig.h"   // 如果你有用到 DEBUG_PRINT 等功能

namespace GCodeHandlers
{

    WaitConditionFunc Handle_GCode(const NCBlock& block, NCManager* nc)
    {
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

        case 90:
            nc->GetCoordSys().isAbsoluteMode = true;
            DEBUG_PRINT("[NC] -> Absolute Mode (G90) Active\n");
            break;

        case 91:
            nc->GetCoordSys().isAbsoluteMode = false;
            DEBUG_PRINT("[NC] -> Incremental Mode (G91) Active\n");
            break;

        default:
            break;
        }

        // 瞬間完成，回傳 nullptr 代表不用等
        return nullptr;
    }

} // end namespace