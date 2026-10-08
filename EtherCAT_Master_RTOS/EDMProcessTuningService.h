#pragma once
#include "EDMProcessTuningContract.h"
class NCManager;
// Startup only; shutdown runs on the NC owner as it exits its loop.
bool InitializeEDMProcessTuningService() noexcept;
void ShutdownEDMProcessTuningService() noexcept;
// Exactly one NC owner invocation per nominal 10ms scan; no file I/O/waits.
void ProcessEDMProcessTuningServiceSameThread(NCManager& nc) noexcept;
