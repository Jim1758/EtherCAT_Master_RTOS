#pragma once

// 🌟 取消註解，告訴編譯器有這個類別，不用提早include全部標頭檔
class NCManager;

namespace HMI_Bridge {

    // 🌟 讓 ProcessTask 接收 NCManager 的指標
    void ProcessTask(NCManager* nc);
    // 🌟 100ms 中速迴圈 (負責字串與大陣列廣播，給 UI 更新用)
    void ProcessTask_100ms(NCManager* nc);
    void ProcessTask_500ms(NCManager* nc);  // 🌟 新增 500ms
    // 🌟 1000ms 慢速迴圈 (1秒一次，適合放系統健康檢查或背景緩慢任務)
    void ProcessTask_1000ms(NCManager* nc);

}