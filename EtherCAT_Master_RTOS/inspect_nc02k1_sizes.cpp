#include "NCPreparedBlockQueueShadow.h"

#include <iostream>

int main()
{
    std::cout << "Entry=" << sizeof(NCPreparedBlockEntrySnapshot) << '\n';
    std::cout << "Queue=" << sizeof(NCPreparedBlockQueueShadow) << '\n';
    std::cout << "Snapshot=" << sizeof(NCPreparedBlockQueueSnapshot) << '\n';
    std::cout << "Counters=" << sizeof(NCPreparedBlockQueueCounters) << '\n';
    return 0;
}
