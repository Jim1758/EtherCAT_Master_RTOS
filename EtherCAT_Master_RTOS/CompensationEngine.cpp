#include "CompensationEngine.h"
#include "MotionCore.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#define NOMINMAX  // 必須放在最上面
CompensationEngine::CompensationEngine() {
    // 預設將 8 軸的表格初始化
    for (int i = 0; i < 8; ++i) {
        m_CompData[i].pitchErrors_Pos.clear();
        m_CompData[i].pitchErrors_Neg.clear();
    }
}

void CompensationEngine::InitAxisCompensation(int axisIndex, bool enBacklash, double b_Pos, double b_Neg, double b_Speed, bool enPitch, double startPos, double step_mm, double p_Speed) {
    if (axisIndex < 0 || axisIndex >= 8) return;

    m_CompData[axisIndex].enableBacklash = enBacklash;
    m_CompData[axisIndex].backlashAmount_Pos_mm = b_Pos;
    m_CompData[axisIndex].backlashAmount_Neg_mm = b_Neg;
    m_CompData[axisIndex].backlashSpeed_mm_s = b_Speed;

    m_CompData[axisIndex].enablePitch = enPitch;
    m_CompData[axisIndex].pitchStartPos_mm = startPos; // 🌟 記得寫入這裡！
    m_CompData[axisIndex].pitchStep_mm = step_mm;
    m_CompData[axisIndex].pitchSpeed_mm_s = p_Speed;
}

void CompensationEngine::SetPitchTables(int axisIndex, const std::vector<double>& posErrors, const std::vector<double>& negErrors) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    m_CompData[axisIndex].pitchErrors_Pos = posErrors;
    m_CompData[axisIndex].pitchErrors_Neg = negErrors;
}
// 🌟 最關鍵的核心運算 🌟
void CompensationEngine::ApplyCompensation(int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    if (!axis.isServoOn || !axis.isHomed) return; // 安全門

    AxisCompensation& comp = m_CompData[axisIndex];
    double pulsePerUnit = axis.resolution_PPR / axis.finalLead;
    double cmdPos_unit = cmd.instantCmdPos / pulsePerUnit;
    double compOffset_unit = 0.0;

    // 1. 方向判斷
    if (cmd.instantCmdVel > 5.0) comp.lastDir = 1;
    else if (cmd.instantCmdVel < -5.0) comp.lastDir = -1;

    // 2. 背隙補償 (平滑)
    if (comp.enableBacklash) {
        comp.targetBacklash_mm = (comp.lastDir >= 0) ? comp.backlashAmount_Pos_mm : -comp.backlashAmount_Neg_mm;
        if (std::abs(comp.currentBacklash_mm - comp.targetBacklash_mm) > 0.0001) {
            double delta = comp.backlashSpeed_mm_s * dt;
            if (comp.currentBacklash_mm < comp.targetBacklash_mm) comp.currentBacklash_mm = std::min<double>(comp.currentBacklash_mm + delta, comp.targetBacklash_mm);
            else comp.currentBacklash_mm = std::max<double>(comp.currentBacklash_mm - delta, comp.targetBacklash_mm);
        }
        compOffset_unit += comp.currentBacklash_mm;
    }

    // 3. 節距補償 (雙向表 + 平滑)
    if (comp.enablePitch && comp.pitchStep_mm > 0.001) {
        const std::vector<double>& activeTable = (comp.lastDir >= 0) ? comp.pitchErrors_Pos : comp.pitchErrors_Neg;

        if (activeTable.size() >= 2) {
            double lookupPos = cmdPos_unit;
            if (axis.axisType == AxisType::ROTARY || axis.axisType == AxisType::ROTARY_CONTINUOUS) {
                lookupPos = std::fmod(lookupPos, axis.rotaryModulo);
                if (lookupPos < 0.0) lookupPos += axis.rotaryModulo;
            }

            double relativePos = lookupPos - comp.pitchStartPos_mm;
            double indexFloat = relativePos / comp.pitchStep_mm;
            int idx = (int)std::floor(indexFloat);
            int maxIdx = (int)activeTable.size() - 1;

            if (idx < 0) comp.targetPitch_mm = activeTable[0];
            else if (idx >= maxIdx) comp.targetPitch_mm = activeTable[maxIdx];
            else {
                double ratio = indexFloat - (double)idx;
                comp.targetPitch_mm = activeTable[idx] * (1.0 - ratio) + activeTable[idx + 1] * ratio;
            }

            // 平滑漸變節距補償
            if (std::abs(comp.currentPitch_mm - comp.targetPitch_mm) > 0.0001) {
                double delta = comp.pitchSpeed_mm_s * dt;
                if (comp.currentPitch_mm < comp.targetPitch_mm) comp.currentPitch_mm = std::min<double>(comp.currentPitch_mm + delta, comp.targetPitch_mm);
                else comp.currentPitch_mm = std::max<double>(comp.currentPitch_mm - delta, comp.targetPitch_mm);
            }
            compOffset_unit += comp.currentPitch_mm;
        }
    }

    cmd.instantCmdPos += (compOffset_unit * pulsePerUnit);
    axis.currentCompOffset_unit = compOffset_unit;

    // 在 ApplyCompensation 最後面加上：
    /*
    if (axisIndex == 0) { // 只看第 0 軸
        static int counter = 0;
        if (counter++ % 1000 == 0) { // 每隔 1000 個週期印一次，避免洗版
            DEBUG_PRINT("[DEBUG] Axis0 Pos: %d, lastDir: %d, Offset: %d mm\n",
                cmdPos_unit, comp.lastDir, compOffset_unit);
        }
    }*/
}

void CompensationEngine::SetPitchTablePos(int axisIndex, const std::vector<double>& errors) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    m_CompData[axisIndex].pitchErrors_Pos = errors;
}

void CompensationEngine::SetPitchTableNeg(int axisIndex, const std::vector<double>& errors) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    m_CompData[axisIndex].pitchErrors_Neg = errors;
}