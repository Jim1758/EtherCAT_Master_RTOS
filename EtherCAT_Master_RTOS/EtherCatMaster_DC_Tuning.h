#pragma once

#include <cstdint>

// EtherCAT DC Runtime RC1.7 参秸俱把计
// Bootstrap  AUTO タЧΘ玡ㄏノタΑ Baseline パ–Ω币笆秖代∕﹚
namespace EtherCatDcTuning
{
    static constexpr int64_t SchedulerBootstrapDriftPpb =
        -9500LL;

    static constexpr int64_t RealFfMinimumDriftPpb =
        -12000LL;

    static constexpr int64_t RealFfMaximumDriftPpb =
        -7800LL;

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
}
