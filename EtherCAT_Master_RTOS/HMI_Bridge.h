#pragma once

// 🌟 取消註解，告訴編譯器有這個類別，不用提早include全部標頭檔
class NCManager;

namespace HMI_Bridge {

    // 🌟 讓 ProcessTask 接收 NCManager 的指標
    void ProcessTask(NCManager* nc);

}