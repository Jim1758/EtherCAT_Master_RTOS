PBC3B｜節距／背隙補償基底恢復與生命週期修正
2026-09-27（台灣）

【先讀結論】
節距補償與背隙補償尚未完整結案。本版先完成最新基底的缺件恢復、生命週期缺陷修正與 HOST 驗證；正式非零補償仍維持 LOCKED。
本次程式的 PBC 部分停在 PBC-2，未包含先前 PBC3A 交付的生命週期模型與 NIC 結果接點。本版以最新 RTOS(10) 為唯一工作基底，逐項合回 PBC3A，再加本次修正，保留後來的 C 軸動態補償內容。
本次不是把舊 MotionCore 整份覆蓋回去，也未將補償啟用常數切為 true。

【本次唯一基底】
EtherCAT_Master_RTOS(10).zip：2,018,623 bytes
SHA256 b87d8ee2fdcb8e31c614bdade5655f157c19fcf718caf5c9e2ac0e56dc35fe8d
EtherCAT_Master_Data(2).zip：1,987,581 bytes
SHA256 b2e181ef15f460158ca60d893b79ad81c6615f8639a2d6899e6e84aefb794a43
兩包全部 ZIP member CRC 通過。其他版本請勿混用此完整替換檔。

【包內檔案與套用】
共 10 個完整原碼檔＋2 支 NC＋本說明。只有以下原碼需要套用：
  CompensationEngine.cpp
  CompensationEngine.h
  MechanicalCompensationModel.h
  MechanicalCompensationLifecycle.h       （新增）
  MotionCore.cpp
  MotionCore.h
  MotionServoHandoffDiagnostic.h          （新增）
  EtherCAT_Function.cpp
  GlobalConfig.cpp
  HMI_Bridge.cpp

1. 先關閉現有 RTOS 程式，備份本機將被替換的原碼。
2. 10 個 CPP/H 一起放入原專案 CPP/H 目錄；新增 header 也要放入同目錄。
   此包使用既有 CPP 引用新增 header，不需要新增 CPP 編譯項目。
   MotionCore 與 EtherCAT_Function 的函式簽章一起改了，不能只更新其中一個。
3. 將 NC_Program 內兩支 NC 複製到既有 NC_Program 目錄。
4. Visual Studio 選 x64／RtssDebug，Rebuild 方案，再重新啟動 RTOS。
5. 本輪全部參數保留：六軸 EnablePitch=0、EnableBacklash=0；不要更換既有節距表或啟用開關。
   本包沒有 PID、AxisConfig、節距表、專案檔、平台替身或測試二進位。

【應看到的開機標記】
[PBC-1] CONFIG=PASS axes=6 enabled=0 MOTION_INTEGRATION=LOCKED
[PBC-2] COORDINATE_CONTRACT=PASS ... FEEDBACK=NOMINAL BEFORE_WCS=1 MOTION_INTEGRATION=LOCKED
[PBC-3B] LIFECYCLE_MODEL=PASS checks=27 TX_SEAM=NIC_API_RESULT NONZERO_INTEGRATION=LOCKED
[PBC3-TX] ENABLED build=PBC3B observationOnly=1 ... driveAck=UNKNOWN ...

若仍只出現 PBC-1/PBC-2，或顯示 PBC3A，先提供完整開機 LOG 核對實際執行版本。
PBC3-TX 僅觀察旋轉定位軸的 IDLE_HOLD，在約 0.5／2／5 秒及離開範圍時產生事件。
它不是 X/Y/Z 非零補償輸出追蹤器，不會啟用補償。apiOK 只代表 NIC SendPacket API 成功，不能當作驅動器接收、馬達到位或校正精度證明。

【這次怎麼測】
固定：六軸空載、MEMORY、SINGLE BLOCK OFF、面板倍率 100%。本輪不要求額外 HOME。

01：PBC3B_01_OFF_6AXIS_RETURN.nc
- 每次先讀取當下 G54 命令工作座標作起點，不回機械零點。
- XYZ 分別 +0.25 mm 再返回；C/U/V 分別 +1 deg 再返回，一次只動一軸。
- G00 F10 是快速倍率 10%，不是每分鐘 10 mm/deg。
- 跑到 M30，保留至少 5 秒 LOG；再 START 重跑一次，完成後同樣保留 5 秒。
- 兩次均應回到各次捕捉起點、完成 M30，無非預期軸移動或警報。

02：PBC3B_02_OFF_HOLD_RESET.nc
- N90：X 先 +1 mm；N100：半徑 1 mm、F40 全圓，理想等速圓周約 9.42 秒。
- 第一次：N100 開始 2～3 秒按 HOLD，等停穩後 START，應完成 N110 返回與 M30。
- 第二次：重新執行，N100 開始 2～3 秒按 RESET，尾段 N110／M30 應取消。
- RESET 不會自動回起點；下一次 START 會重新記錄起點。
- RESET 後再跑 01 一次，確認可重新啟動，完成後保留 5 秒 LOG。
- 完整路徑包絡相對捕捉起點：X[-1,+1] mm、Y[-1,+1] mm；Z/C/U/V 不下運動命令。

請提供完整開機、01 往返、02 HOLD/續行、02 RESET 及 RESET 後重跑的 LOG；自動切檔的各份一起提供。
這批測試驗收 OFF 行為與觀測接點恢復，不能用來宣告非零節距／背隙通過。

【本次修正了什麼】
A. 恢復 PBC3A 的預覽／封存／送出結果生命週期模型、boot 自檢、兩種 process-image 送出路徑的 NIC 結果接點、固定容量旋轉軸 IDLE_HOLD 觀測。
B. HOME／RESET／Servo 參考請求序號必須在同一模型生命週期持續遞增；完成或取消的請求不可換一個 tick 再套用一次。序號回捲也拒絕。
C. 已嘗試送出但 ticket／identity 不符，不能繼續信任舊座標，改標為 OutputUncertain。
D. attempted=false 但 apiAccepted=true 的矛盾回報，改標為 OutputUncertain。
E. 外部已套用 reference、回報 ticket 卻不符，同樣使座標失效。
F. 不確定處理只消耗本地 pending 的 tick／序號，不採用錯誤外部回報的巨大數值，避免阻擋後續合法 HOME 恢復。
上述 B～F 是模型契約修正；非零模型尚未接入正式軸輸出，不代表已修完實機所有停止／尋原點流程。

【本輪實際 HOST 驗證】
1. 真實模型／生命週期：GCC O2、-Wall -Wextra -Werror，197 checks／0 failures；啟動自檢 27 checks。
   同一測試對原 PBC3A 為 9 failures（7 個直接缺陷及 2 個恢復流程後果），已確認本次修正有效。
   含正負插值、旋轉週期、速度預算、HOLD、保留偏移 RESET、重播／回捲及不確定輸出；這不是實機幾何量測。
2. 實際 Data(2) 與完整 CompensationEngine／AlarmManager：48,385 checks 通過。
   六軸 OFF 共 12,000 次命令，位置／速度 double bits 完全不變；舊版與新版同樣通過。
   HOST 記憶體測試 X 原節距表 8.5 被 OffsetLimit 拒絕；X/C 合法零表啟用及 X 背隙啟用均被 IntegrationPending 拒絕；封存後改表、旗標偷改、非法軸索引均受保護。
   使用原 GlobalConfig 的 ReadPbcAxisParameters 函式擷取與原資料解析器；未宣稱執行整套 GlobalConfig 啟動／NIC。
3. 上述模型與 engine 測試也通過 ASan＋UBSan。LeakSanitizer 因環境限制停用，未宣稱漏記憶體掃描通過。
4. 實際擷取的新舊 Begin／Finalize／End send 方法，72 情境等價比較通過；原輸出、拒絕／清零、owner／epoch／alarm 釋放結果一致。周邊平台／外部權限來源有 HOST 替身，不是完整排程器或 NIC 實測。
5. 觀測器正常、失敗／未送出、清零、身分切換、來源間隔、滿佇列測試通過。
6. 原始與修改後 MotionCore／EtherCAT_Function／GlobalConfig／HMI_Bridge 四個完整 CPP，各通過 GCC C++14 -fsyntax-only（共 8/8、零編譯診斷）。只有 Windows／RTX／NAL／CRT 宣告使用替身。
   CompensationEngine 與真實 MotionCore header 可直接 HOST 編譯；沿用的 header 存在三處 signedness 警告，未修改無關程式。
   此非 VS／RTX64 SDK 編譯、連結、ABI 或實機驗證；仍須本機 x64／RtssDebug Rebuild。
7. 本版真實 GCodeParser 解析兩支 NC，32＋19＝51 指令區塊通過；可執行文字與先前 PBC3A NC 相同。
8. 三方合併無衝突，反向合併逐字重建最新基底；237 個既有 MotionCore 方法及 22 個 EccentricC 輔助檔保持原內容。PID、C 補償幾何／插補未修改。

【完整功能尚待完成，不能直接解鎖】
1. 正式 ApplyCompensation 仍只接受 OFF identity，尚未呼叫非零 PrepareCycle／SealCycle／FinishSend，也沒有正式發布非零補償 frame／offset。
2. 初始 reference、RESET、Servo OFF/ON、故障復歸、HOME 的 AxisContext 座標提交尚未與補償交易綁定。
3. 現有普通 RESET、IDLE_HOLD 及 PathCore 停止位置修正仍拒絕補償軸；不能忽略這些 guard。
4. 每軸候選補償→PID→最終 PDO→NIC 結果的同次權限／座標提交，以及失敗後恢復尚需完成。API 成功仍不是 drive ACK。
5. 補償速度／加速度餘量、真正 HOME 基準、名義與 raw 回授／到位一致性、旋轉週期與多軸範圍仍需正式接線及實機驗收。
6. 完成上述後，才提供單 X、小幅合成表的節距單獨→背隙單獨→合併→HOLD/RESET 測試包；其他五軸保持關閉，再擴軸。
   合成表只能證明軟體功能，真正節距與背隙校正仍需要機構量測。

【附件實際參數稽核】
二、附件的實際值
1. AxisCount=6；AXIS_CFG.ini 順序 X/Y/Z/C/U/V；第 7、8 軸不存在。
2. X/Y/Z：線性軸，Resolution=16,777,216 pulse/rev、MechanicalPitch=10 mm/rev、減速比兩側皆 1，故 1,677,721.6 pulse/mm。
3. C/U/V：旋轉軸，Resolution=16,777,216 pulse/rev、MechanicalPitch=360 deg/rev、RotaryModulo=360、ShortestPath=1，故約 46,603.37778 pulse/deg。
4. 六軸皆 FbMode=0（馬達編碼器）、ScaleRatio=1、isReverse=0、Axis_Reverse=0。
5. 六軸 EnablePitch=0、EnableBacklash=0；正負向背隙量皆 0。PitchStartPos=0、PitchStep=10、PitchSpeed_mm_s=3、backlashSpeed=3。
   名稱含 mm 的補償欄位實際採用軸原生單位：XYZ 為 mm、CUV 為 deg；速度是原生單位/秒，不是每週期，也不是 um/秒。
6. SpeedConfig 六軸 MAX_Speed=5000、G00_Speed=5000；載入器將原生單位/分換為 PPS。XYZ 上限 83.3333 mm/s，CUV 上限 83.3333 deg/s。
7. XYZ Acc/Dec_Time=1.0 s；CUV=0.5 s；六軸 SmoothTime=100 ms；G00 acc/dec=0.5 s。
8. XYZ inPositionWindow_mm=0.005；目前 C/U/V 檔案實際是 0.010 deg。這與先前要求 ±0.005 deg 不相同；本輪不藉此調整旋轉軸，也不能以本輪 OFF 通過宣稱已達 ±0.005 deg。
9. X EnableLagCheck=0；Y/Z/C/U/V=1；maxLag_mm 皆 2（CUV 實際為 deg）。本輪保留；後續非零補償前需處理 X 跟隨誤差保護條件，不能把目前 X 無跟隨警報當作補償合格。
10. 六軸 TravelLimit1/2/3Enable 皆 0。X 第一組上下限雖填 ±100，但該組未啟用；不可當成有效行程保護。
11. NCConfig：ProgrammableTravelLimitEnabled=1、ElectrodeRotationAxis=4（對應 C）；此項總開關不會把上述停用的各軸限位變成有效。
12. X PID：IDLE Kp=100/Ki=50/Kd=0/Kvff=0；G00 Kp=30/Ki=0/Kd=0/Kvff=1。本輪不調整。

三、舊節距表必須保持停用
兩個 PITCH_TABLE 檔皆 1000 資料列、每列 8 欄。數值為「要加到命令的修正量」，不會自動 um→mm。
以 X 欄、PitchStartPos=0、PitchStep=10 計：
資料列／X 座標／正向表／負向表
第 1 列／0 mm／8.5 mm／0 mm
第 2 列／10 mm／0.5 mm／0.2 mm
第 3 列／20 mm／0.6 mm／0.3 mm
第 4～1000 列／30～9990 mm／0／0
Y/Z/C/U/V/其餘欄皆 0。
尤其 8.5 是 8.5 mm，不能視為 8.5 um。不可直接將 EnablePitch 改成 1。這些資料在停用狀態可以解析，但「TABLE_PAIR=PARSED」不是校正驗收。
現有預設 CompMaxAbsOffset_unit=0.1、CompMaxAbsSlope=0.01，正向舊表超過限制，啟用應在啟動時拒絕。合法非零配置仍會被尚未釋出的 Motion 整合鎖拒絕；本輪不是繞過此鎖的操作說明。

四、HOME 與節距座標基準
HomeSnapshot 只是歷史診斷；目前 X/Y/Z 記錄來自 2026-09-15，C/U/V 無快照。每次 Core 啟動都將 isHomed=false、machineCoordinateOffsetPulse=0，不從快照恢復已尋原點。
本輪 OFF NC 不需要額外 G81。後續真正非零節距必須先完成相應軸本次啟動的有效參考建立，並確認表格座標對應 nominal machine coordinate；不能把畫面的 G54 工作座標或舊快照直接當作節距表零點。


【原碼與 NC SHA256】
CompensationEngine.cpp | 7584 bytes | 5074527c8f6c5614f27622199258021e0e0ea4f32467f8ecb06bde3f864359eb
CompensationEngine.h | 2793 bytes | 735d463b23e582257d9d3f04c55d4cd9d4ae1f6aed9bee55648225ada882873e
EtherCAT_Function.cpp | 216358 bytes | 64cc0ab11403e708a4bbe263fe35a683c8d1f29921b2a92bbe2deab28cfaae4e
GlobalConfig.cpp | 66729 bytes | 6e27d9a6e6f04c9ed6a43cd354ba535df92e3698e5ca87cf624e27946282e3eb
HMI_Bridge.cpp | 539035 bytes | 4b7e818adb40cd00c13757f02d8fa43919ce7af4d07144af2acec29bb6c44271
MechanicalCompensationLifecycle.h | 21054 bytes | 35cd365b0b88c92f0ff349c8b70ab09fa6cfbd020179a173dd917f263c0f20c9
MechanicalCompensationModel.h | 19679 bytes | f94354e6e061cac84e4d2af3db4cd3e717d04d527cfa768b4c794e3104acae43
MotionCore.cpp | 1041185 bytes | 251ccef904e777f472f226b36cdf02f58acece6b3004654a35e64b6522d77ae4
MotionCore.h | 213730 bytes | 861e1558310ea09eacc1166c51a41c9dbb8ee749bb80294e334e944518ddb273
MotionServoHandoffDiagnostic.h | 7530 bytes | 251833298bee03de1736661618e8ba6ab44c0850dd43aa96f58c3f6076bb0acc
NC_Program/PBC3B_01_OFF_6AXIS_RETURN.nc | 1167 bytes | d29323e27d5a61a29782ce1e0fea02b6e020e935e76c73a369146c1522b6d84f
NC_Program/PBC3B_02_OFF_HOLD_RESET.nc | 1073 bytes | d729f4ca4df3fa547fd4a6d0b5384b77544a3d5ab892b18ad53e2a314a5313a8
