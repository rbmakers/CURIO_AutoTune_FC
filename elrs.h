#ifndef __CURIO_ELRS_H__
#define __CURIO_ELRS_H__

#include <Arduino.h>

// ================================================================
//  CURIO ELRS / CRSF 接收機模組
//  負責：UART2 (Serial2) 設定、CRSF 封包解析、16 通道資料維護
//  對應 TX16S 已驗證之頻道順序 (CURIO_ELRS_Test.ino)
//  製作 : 火箭鳥創客倉庫
// ================================================================

// --- 接腳與通訊參數 ---
#define ELRS_SERIAL   Serial2
#define ELRS_BAUD     420000
#define PIN_ELRS_TX   4
#define PIN_ELRS_RX   5

// --- RC 頻道索引對應 (CRSF CH1~CH7) ---
#define CH_ROLL              0
#define CH_PITCH             1
#define CH_THROTTLE          2
#define CH_YAW               3
#define CH_ARM               4   // 解鎖開關
#define CH_AUTOTUNE          5   // AUX1：自動調校觸發開關
#define CH_EMERGENCY_DESCEND 6   // AUX2：緊急自動減速緩降開關

// ================================================================
//  對外 API
// ================================================================

// 初始化 ELRS UART，需於 setup() 中呼叫一次
void elrsInit();

// 解析目前已收到的 CRSF 位元組，需於 loop() 中每次呼叫
void elrsUpdate();

// 取得指定頻道 (0~15) 目前的 us 值；索引超界時回傳 1500 (安全中點)
unsigned long elrsGetChannel(uint8_t index);

// 距離上一個合法 CRSF 封包是否在 timeout_ms 之內 (供失聯保護判斷使用)
bool elrsLinkOk(unsigned long timeout_ms);

// CRC 校驗失敗的封包累計次數 (除錯用)
uint32_t elrsGetCrcErrorCount();

// 成功解碼的封包累計次數 (除錯用)
uint32_t elrsGetFrameCount();

#endif
