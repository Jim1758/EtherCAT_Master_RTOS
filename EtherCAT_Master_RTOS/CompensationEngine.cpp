#include "CompensationEngine.h"
#include "MotionCore.h"

CompensationEngine::CompensationEngine() {
    // 預設將 8 軸的表格初始化
    for (int i = 0; i < 8; ++i) {
        m_CompData[i].pitchErrors.clear();
    }
}

void CompensationEngine::InitAxisCompensation(int axisIndex, bool enBacklash, double backlash_mm, bool enPitch, double step_mm) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    m_CompData[axisIndex].enableBacklash = enBacklash;
    m_CompData[axisIndex].backlashAmount_mm = backlash_mm;
    m_CompData[axisIndex].enablePitch = enPitch;
    m_CompData[axisIndex].pitchStep_mm = step_mm;
}

void CompensationEngine::SetPitchTable(int axisIndex, const std::vector<double>& errors) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    m_CompData[axisIndex].pitchErrors = errors;
}

// 🌟 最關鍵的核心運算 🌟
void CompensationEngine::ApplyCompensation(int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt) {
    if (axisIndex < 0 || axisIndex >= 8) return;
    AxisCompensation& comp = m_CompData[axisIndex];

    // 🌟 修正 1: 使用自動計算出的 finalLead，而不是硬編碼的 10.0
    double pulsePerUnit = axis.resolution_PPR / axis.finalLead;

    // 將大腦算出來的 Pulse 座標轉換成單位 (mm 或 degree)
    double cmdPos_unit = cmd.instantCmdPos / pulsePerUnit;
    double compOffset_unit = 0.0;

    // ==========================================
    // 1. 背隙補償
    // ==========================================
    if (comp.enableBacklash) {
        if (cmd.instantCmdVel > 1.0) comp.lastDir = 1;
        else if (cmd.instantCmdVel < -1.0) comp.lastDir = -1;

        comp.targetBacklash_mm = (comp.lastDir == 1) ? comp.backlashAmount_mm : 0.0;

        if (comp.currentBacklash_mm < comp.targetBacklash_mm) {
            comp.currentBacklash_mm += comp.backlashSpeed_mm_s * dt;
            if (comp.currentBacklash_mm > comp.targetBacklash_mm) comp.currentBacklash_mm = comp.targetBacklash_mm;
        }
        else if (comp.currentBacklash_mm > comp.targetBacklash_mm) {
            comp.currentBacklash_mm -= comp.backlashSpeed_mm_s * dt;
            if (comp.currentBacklash_mm < comp.targetBacklash_mm) comp.currentBacklash_mm = comp.targetBacklash_mm;
        }
        compOffset_unit += comp.currentBacklash_mm;
    }

    // ==========================================
    // 2. 節距補償 (Pitch Error)
    // ==========================================
    if (comp.enablePitch && comp.pitchErrors.size() >= 2 && comp.pitchStep_mm > 0.001) {

        // 🌟 修正 2: 處理 Modulo 後的正確座標
        double lookupPos_unit = cmdPos_unit;
        if (axis.axisType == AxisType::ROTARY || axis.axisType == AxisType::ROTARY_CONTINUOUS) {
            lookupPos_unit = std::fmod(lookupPos_unit, axis.rotaryModulo);
            if (lookupPos_unit < 0.0) lookupPos_unit += axis.rotaryModulo;
        }

        // 🌟 修正 3: 這裡必須使用 lookupPos_unit，而不是原始的 cmdPos_mm
        double relativePos = lookupPos_unit - comp.pitchStartPos_mm;
        double indexFloat = relativePos / comp.pitchStep_mm;

        int idx = (int)std::floor(indexFloat);
        int maxIdx = (int)comp.pitchErrors.size() - 1;

        if (idx < 0) {
            compOffset_unit += comp.pitchErrors[0];
        }
        else if (idx >= maxIdx) {
            compOffset_unit += comp.pitchErrors[maxIdx];
        }
        else {
            double ratio = indexFloat - (double)idx;
            double error1 = comp.pitchErrors[idx];
            double error2 = comp.pitchErrors[idx + 1];
            compOffset_unit += error1 * (1.0 - ratio) + error2 * ratio;
        }
    }

    // ==========================================
    // 3. 疊加回 Pulse 給 PID
    // ==========================================
    cmd.instantCmdPos += (compOffset_unit * pulsePerUnit);
}