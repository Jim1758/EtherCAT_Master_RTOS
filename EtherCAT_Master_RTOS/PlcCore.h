#pragma once
#include "EtherCatTypes.h" // 引用上面的獨立型別檔
#include <vector>

class PlcCore {
public:
    PlcCore();

    // 1. 連結資料 (保留您的 Link 介面)
    void Link(std::vector<ENI_GenericIO>* pList);

    // 2. 這是您為了配合 EtherCatMaster.cpp 呼叫而加的，保留它！
    void SetIoList(std::vector<ENI_GenericIO>* pList) 
    {
        m_pIo = pList;
    }

    // 3. 核心邏輯
    void Update();

    // 4. API (控制介面)
    bool Get_I(int moduleIdx, int bitIdx);
    void Set_O(int moduleIdx, int bitIdx, bool val);
    bool Get_O(int moduleIdx, int bitIdx);

    // [新增] 為了讓 Update 跑馬燈更簡潔，我加了這個 helper
    void Clear_Module_O(int moduleIdx);

    
    void FlushOutputs();// 將邏輯狀態同步到硬體地圖

private:
    std::vector<ENI_GenericIO>* m_pIo = nullptr;

    // 內部變數
    int m_loopCount = 0;
    int m_marqueeLed = 0;
};