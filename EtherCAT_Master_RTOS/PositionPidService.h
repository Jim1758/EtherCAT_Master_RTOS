#pragma once
#include "PositionPidContract.h"
class NCManager;

bool InitializePositionPidService() noexcept;
void ShutdownPositionPidService() noexcept;
// NC owner: true means a request is in flight; defer ordinary NC processing
// until the RT owner has acknowledged it. Safety services still run first.
bool ProcessPositionPidServiceSameThread(NCManager& nc) noexcept;

// Only RT calls these. No mapping access, file I/O, allocation or logging.
// A single immutable request slot has release/acquire ownership transfer.
bool PositionPidRuntimeEnabled() noexcept;
bool PositionPidRuntimeTake(PositionPidRequest& request,std::uint64_t& queuedCycle) noexcept;
bool PositionPidNcIdleNow() noexcept;
bool PositionPidRuntimeDeadlineCurrent() noexcept;
void PositionPidRuntimePublish(const PositionPidContext& context,const PositionPidValues& values) noexcept;
void PositionPidRuntimeComplete(const PositionPidRequest& request,PositionPidStatus status,
    PositionPidReason reason,std::uint64_t generation,const PositionPidGains& applied) noexcept;
