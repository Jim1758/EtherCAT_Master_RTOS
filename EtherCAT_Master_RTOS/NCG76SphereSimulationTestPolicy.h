#pragma once

// EDM311 normal SIM build: use the ordinary 200/100 ms voltage wave.
// The EDM310 predecessor-retreat probe is kept as an explicit test-only
// opt-in for a future diagnostic build. Waveform selection grants no motion,
// history, stop or discharge authority.
// The RT loader must independently prove the completed predecessor/release,
// current frame, owner, recipe and travel envelope before selecting the probe.
namespace NCG76SphereSimulationTestPolicy
{
constexpr bool PreviousParentProbeEnabled = false;
}
