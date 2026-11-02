#include "NCHoldTelemetry.h"
#include <windows.h>
#include <rtapi.h>
#include <cstring>
#include <limits>

namespace NCHoldTelemetry { namespace {
HANDLE mapping=nullptr;
Wire* target=nullptr;
std::uint64_t publisherGeneration=0ULL;
std::uint32_t heartbeat=0U;
std::uint32_t WritingSequence(std::uint32_t previous) noexcept {
    const auto next=(previous+1U)|1U;
    return next==(std::numeric_limits<std::uint32_t>::max)()?1U:next;
}
void CopyPublication(const Wire& image) noexcept {
    if(!target)return;
    const auto writing=WritingSequence(target->sequence);
    auto* sequence=reinterpret_cast<volatile LONG*>(&target->sequence);
    InterlockedExchange(sequence,static_cast<LONG>(writing));MemoryBarrier();
    auto* destination=reinterpret_cast<unsigned char*>(target);
    const auto* source=reinterpret_cast<const unsigned char*>(&image);
    std::memcpy(destination,source,SequenceOffset);
    std::memcpy(destination+SequenceOffset+sizeof(std::uint32_t),
        source+SequenceOffset+sizeof(std::uint32_t),WireSize-SequenceOffset-sizeof(std::uint32_t));
    MemoryBarrier();InterlockedExchange(sequence,static_cast<LONG>(writing+1U));
}
}
bool Initialize() noexcept {
    if(target)return true;
    void* location=nullptr;
    HANDLE handle=RtCreateSharedMemory(PAGE_READWRITE,0,WireSize,SharedMemoryName,&location);
    if(handle==nullptr||location==nullptr){if(handle!=nullptr)RtCloseHandle(handle);return false;}
    auto* page=static_cast<Wire*>(location);
    const std::uint64_t previous=page->magic==Magic&&page->size==WireSize?page->publisherGeneration:0ULL;
    if(previous==(std::numeric_limits<std::uint64_t>::max)()){RtCloseHandle(handle);return false;}
    mapping=handle;target=page;publisherGeneration=previous+1ULL;heartbeat=0U;
    Wire waiting{};waiting.magic=Magic;waiting.versionMajor=VersionMajor;waiting.versionMinor=VersionMinor;
    waiting.size=WireSize;waiting.publisherGeneration=publisherGeneration;
    CopyPublication(waiting);
    return true;
}
void Publish(const Image& image) noexcept {
    if(!target)return;
    Wire publication{};
    ++heartbeat;if(!heartbeat)++heartbeat;
    if(!Encode(image,publisherGeneration,heartbeat,publication)){
        // Malformed producer state is explicitly unavailable, never a valid N0.
        Image unavailable{};
        if(!Encode(unavailable,publisherGeneration,heartbeat,publication))return;
    }
    CopyPublication(publication);
}
std::uint64_t CurrentPublisherGeneration() noexcept { return target?publisherGeneration:0ULL; }
void Shutdown() noexcept {
    if(target){
        Wire unavailable{};unavailable.magic=Magic;unavailable.versionMajor=VersionMajor;
        unavailable.versionMinor=VersionMinor;unavailable.size=WireSize;
        unavailable.publisherGeneration=publisherGeneration;unavailable.heartbeat=heartbeat;
        CopyPublication(unavailable);
    }
    target=nullptr;publisherGeneration=0ULL;heartbeat=0U;
    if(mapping!=nullptr){RtCloseHandle(mapping);mapping=nullptr;}
}
}
