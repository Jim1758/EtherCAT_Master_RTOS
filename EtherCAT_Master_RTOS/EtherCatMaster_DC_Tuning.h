#pragma once

#include <cstdint>

// EtherCAT DC Runtime RC1.8 統一調整參數。
// Bootstrap 只在 AUTO 校正完成前使用；正式 Baseline 由每次啟動量測決定。
namespace EtherCatDcTuning
{
    static constexpr int64_t SchedulerBootstrapDriftPpb =
        -9500LL;

    // 每次開機 AUTO 捕獲的絕對安全邊界。
    // 冷機、溫機可以在這個範圍內建立各自的 Baseline。
    static constexpr int64_t DriftCalibrationMinimumPpb =
        -16000LL;

    static constexpr int64_t DriftCalibrationMaximumPpb =
        -5000LL;

    // 正式 Real FF 的絕對安全邊界。
    // 目前與啟動捕獲範圍相同，避免已鎖定的冷機 Baseline 被再次夾到邊界。
    static constexpr int64_t RealFfMinimumDriftPpb =
        -16000LL;

    static constexpr int64_t RealFfMaximumDriftPpb =
        -5000LL;

    static constexpr int64_t PdoCycleNs =
        250000LL;

    static constexpr uint32_t DriftCalibrationGoodWindows =
        5U;

    static constexpr int64_t DriftCalibrationMaximumMadPpb =
        800LL;

    static constexpr int64_t DriftCalibrationMaximumRawMedianDeviationPpb =
        2000LL;

    static constexpr int64_t SchedulerBootstrapPeriodFfPs =
        SchedulerBootstrapDriftPpb *
        PdoCycleNs /
        1000000LL;

    static_assert(
        DriftCalibrationMinimumPpb >= RealFfMinimumDriftPpb &&
        DriftCalibrationMaximumPpb <= RealFfMaximumDriftPpb,
        "Drift calibration range must stay inside Real FF range.");

    static_assert(
        SchedulerBootstrapDriftPpb >= RealFfMinimumDriftPpb &&
        SchedulerBootstrapDriftPpb <= RealFfMaximumDriftPpb,
        "Bootstrap drift must stay inside Real FF range.");
}
