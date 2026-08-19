#pragma once

class EtherCatMaster;

// 啟動時依 SystemConfig.txt 的 AUTO／FIXED 規則選擇並驗證 DC Reference。
// 成功後結果會固定保存，正式 PDO Runtime 只讀取索引，不在即時路徑重新掃描。
bool ResolveDcReferenceSlave(
    EtherCatMaster* pMaster);

// 回傳已選擇的零起算 EtherCAT slave index；尚未選擇或驗證失敗時回傳 -1。
int GetDcReferenceSlaveIndex();

// 啟動階段唯讀檢查 AUTO Servo 順序、DC 能力、Port Link 與目前 0x0928。
// 本函式不修改任何 EtherCAT 暫存器，也不改變已選定的 DC Reference。
bool DiagnoseDcAutoTopologyDryRun(
    EtherCatMaster* pMaster);
