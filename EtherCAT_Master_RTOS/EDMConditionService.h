#pragma once
#include "EDMConditionCommandContract.h"
class NCManager;
namespace EDMRecipe { struct Catalog; struct Summary; }
// Start/stop may allocate/wait and are startup/shutdown only. The same-thread
// 100ms service performs bounded copies/domain operations, never file I/O.
bool InitializeEDMConditionService(const char* recipeDirectory) noexcept;
void ShutdownEDMConditionService() noexcept;
void ProcessEDMConditionServiceSameThread(NCManager& nc) noexcept;
// Notify a committed G38 immediately; only bounded atomic stores on the NC owner.
void RememberEDMConditionSelectionSameThread(const EDMRecipe::Catalog& catalog, const EDMRecipe::Summary& summary) noexcept;
EDMConditionData* GetEDMConditionSharedMemoryData() noexcept;
