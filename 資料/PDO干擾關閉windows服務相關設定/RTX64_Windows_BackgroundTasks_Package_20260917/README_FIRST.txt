【2026-09-17 FIX3】
PowerShell 操作流程再簡化：
- 雙擊 CMD → UAC 按「是」→ 系統管理員 PowerShell 會自動執行 OneClick。
- PowerShell 開啟後不需要再按 Enter。
- OneClick 執行完成後也不需要按 Enter。
- 因為使用 -NoExit，PowerShell 會停留在提示字元，不會自動消失。
- 操作人員確認 FINAL CHECK / PASS 後，再手動按 X 關閉 PowerShell 視窗。

【2026-09-17 FIX2】
PowerShell 視窗保留行為已更新：
- CMD 啟動的「系統管理員 PowerShell」使用 -NoExit。
- OneClick 執行完成後 PowerShell 不會自動消失。
- 按 Enter 後只會回到 PowerShell 提示字元，視窗仍保持開啟。
- 最後由操作人員手動按 X 關閉 PowerShell 視窗。


【2026-09-17 FIX1】
Run_RTX64_Windows_BackgroundTasks_OneClick.cmd 已更新：
- 會等待系統管理員 PowerShell 執行完成。
- PowerShell 執行完成後不會自動關閉，需按 Enter。
- 回到 CMD 後，CMD 視窗也不會自動關閉，需按任意鍵。
- 適合現場操作人員確認 FINAL CHECK / PASS 結果。

RTX64 / EtherCAT Windows 背景排程設定包
版本日期：2026-09-17

用途
====
此包用於 RTX64 + EtherCAT 250 us 專用 IPC 的 Windows Scheduled Tasks 整理與驗收。

這份 Profile 是根據實機長時間測試中，曾與 PDO RX / Master-NAL 長尾、
Windows Idle/Maintenance、System Restore / VSS、MemoryDiagnostic、
DeviceDirectoryClient 等背景活動在時間上高度相關的事件整理而成。

重要：
- 本包是「專用控制器 A/B / 部署 Profile」，不是 Microsoft 官方通用調校清單。
- 停用項目代表降低背景活動干擾，不代表每一項都已單獨證明是唯一根因。
- EtherCAT Physical Error（ESC Physical/Invalid/Forwarded/LostLink Delta 增加）
  與 Windows/Master-NAL 長尾問題必須分開判讀。
- 本包不刪除 Task，不修改 Task ACL，不停用 Security-SPP service，
  不停用整個 VSS / System Restore 功能。

最快使用方式
============
1. 將 Tools 資料夾內的兩個檔案放在同一資料夾：
   - Run_RTX64_Windows_BackgroundTasks_OneClick.cmd
   - RTX64_Windows_BackgroundTasks_OneClick.ps1

2. 雙擊：
   Run_RTX64_Windows_BackgroundTasks_OneClick.cmd

3. UAC 跳出後按「是」。

4. 等待 FINAL CHECK。

5. 確認綠字：
   PASS: All existing target tasks are Disabled.

6. 桌面會產生：
   RTX64_BackgroundTasks_Check_YYYYMMDD_HHMMSS.txt

7. 畫面最後會停在：
   按 Enter 關閉此視窗...

詳細 SOP：
Docs\01_新電腦_操作步驟_SOP.txt

驗收表：
Docs\02_設定完成_檢查清單.txt

恢復 / 排錯：
Docs\04_還原與故障排除.txt
