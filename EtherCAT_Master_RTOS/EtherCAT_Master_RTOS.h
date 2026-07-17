//////////////////////////////////////////////////////////////////
//
// EtherCAT_Master_RTOS.h - header file
//
//////////////////////////////////////////////////////////////////

#pragma once

// 1. 基礎系統定義
#include <SDKDDKVer.h>

// 2. 網路庫定義 (必須在 windows.h 之前，否則 winsock1/2 會衝突)
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

// 3. Windows 基礎類型 (定義 PCHAR, ULONG, HANDLE 等)
#include <windows.h>
#include <tchar.h>
#include <stdio.h>

// 4. RTX64 核心 API (必須在 windows.h 之後)
#include <rtapi.h>

// 5. RTNAL / RTTCPIP 即時網路支持
#include <rtnapi.h>
#include <rtnalapi.h>

// 6. RTSS 專用 API
#ifdef UNDER_RTSS
#include <rtssapi.h>
#endif 



// --- 常數定義 ---
#define DEVICE_NOT_FOUND    -1
#define NUM_REGISTERS       PCI_TYPE0_ADDRESSES

// --- 函式原型宣告 ---

// 週期性計時器回呼函式 (1ms 迴圈核心)
void RTFCNDCL TimerHandler(void* nContext);

// 中斷處理相關 (ISR/IST)
INTERRUPT_DISPOSITION RTFCNDCL DeviceISR(PVOID pContext);
BOOLEAN RTAPI DeviceIST(PVOID pContext);

// 硬體初始化與搜尋 (PCI)
int DeviceSearch(int vendorID, int deviceID, PCI_SLOT_NUMBER* pSlotNumber, PPCI_COMMON_CONFIG PciData);
BOOLEAN DeviceInit(int busNumber, PCI_SLOT_NUMBER* pSlotNumber, PPCI_COMMON_CONFIG data);

// 中斷控制
void DisableInterruptsOnChip();
void EnableInterruptsOnChip();

// 資源回收與結束處理
void DeviceCleanup(HANDLE hHandle);
void MsgAndExit(TCHAR* msg);