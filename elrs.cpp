/*
    CURIO ELRS / CRSF 接收機模組 — 實作檔
    從 CURIO_AutoTune_Dual_PID.ino 拆分而來
    製作 : 火箭鳥創客倉庫
*/

#include "elrs.h"

// ================================================================
//  CRSF 封包解析 — 內部狀態 (對外不可見，僅透過下方 API 存取)
// ================================================================
#define CRSF_SYNC_BYTE     0xC8
#define CRSF_TYPE_RC_CHAN  0x16
#define CRSF_MAX_FRAME_LEN 64

static uint8_t  crsfBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  crsfBufIdx  = 0;
static uint8_t  crsfExpLen  = 0;
static bool     crsfInFrame = false;

static unsigned long channels[16] = {
    1500, 1500, 1000, 1500,
    1000, 1000, 1000, 1000,
    1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
static uint32_t frameCount    = 0;
static uint32_t crcErrorCount = 0;
static uint32_t lastFrameMs   = 0;

static uint8_t crsfCrc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

static inline unsigned long crsfToUs(uint16_t raw)
{
    return (unsigned long)constrain((long)raw * 1000L / 1639L + 895L, 1000L, 2000L);
}

static void crsfDecodeChannels(const uint8_t *p)
{
    uint16_t ch[16];
    ch[0]  = ((uint16_t)(p[0])       | (uint16_t)(p[1])  << 8) & 0x07FF;
    ch[1]  = ((uint16_t)(p[1])  >> 3 | (uint16_t)(p[2])  << 5) & 0x07FF;
    ch[2]  = ((uint16_t)(p[2])  >> 6 | (uint16_t)(p[3])  << 2 | (uint16_t)(p[4]) << 10) & 0x07FF;
    ch[3]  = ((uint16_t)(p[4])  >> 1 | (uint16_t)(p[5])  << 7) & 0x07FF;
    ch[4]  = ((uint16_t)(p[5])  >> 4 | (uint16_t)(p[6])  << 4) & 0x07FF;
    ch[5]  = ((uint16_t)(p[6])  >> 7 | (uint16_t)(p[7])  << 1 | (uint16_t)(p[8]) << 9) & 0x07FF;
    ch[6]  = ((uint16_t)(p[8])  >> 2 | (uint16_t)(p[9])  << 6) & 0x07FF;
    ch[7]  = ((uint16_t)(p[9])  >> 5 | (uint16_t)(p[10]) << 3) & 0x07FF;
    ch[8]  = ((uint16_t)(p[11])      | (uint16_t)(p[12]) << 8) & 0x07FF;
    ch[9]  = ((uint16_t)(p[12]) >> 3 | (uint16_t)(p[13]) << 5) & 0x07FF;
    ch[10] = ((uint16_t)(p[13]) >> 6 | (uint16_t)(p[14]) << 2 | (uint16_t)(p[15]) << 10) & 0x07FF;
    ch[11] = ((uint16_t)(p[15]) >> 1 | (uint16_t)(p[16]) << 7) & 0x07FF;
    ch[12] = ((uint16_t)(p[16]) >> 4 | (uint16_t)(p[17]) << 4) & 0x07FF;
    ch[13] = ((uint16_t)(p[17]) >> 7 | (uint16_t)(p[18]) << 1 | (uint16_t)(p[19]) << 9) & 0x07FF;
    ch[14] = ((uint16_t)(p[19]) >> 2 | (uint16_t)(p[20]) << 6) & 0x07FF;
    ch[15] = ((uint16_t)(p[20]) >> 5 | (uint16_t)(p[21]) << 3) & 0x07FF;
    for (int i = 0; i < 16; i++) {
        channels[i] = crsfToUs(ch[i]);
    }
    frameCount++;
    lastFrameMs = millis();
}

// ================================================================
//  對外 API 實作
// ================================================================
void elrsInit()
{
    ELRS_SERIAL.setTX(PIN_ELRS_TX);
    ELRS_SERIAL.setRX(PIN_ELRS_RX);
    ELRS_SERIAL.begin(ELRS_BAUD);
}

void elrsUpdate()
{
    while (ELRS_SERIAL.available()) {
        uint8_t byte = ELRS_SERIAL.read();
        if (!crsfInFrame) {
            if (byte == CRSF_SYNC_BYTE) {
                crsfInFrame = true;
                crsfBufIdx  = 0;
                crsfBuf[crsfBufIdx++] = byte;
            }
        } else {
            crsfBuf[crsfBufIdx++] = byte;
            if (crsfBufIdx == 2) {
                crsfExpLen = byte + 2;
                if (crsfExpLen > CRSF_MAX_FRAME_LEN) {
                    crsfInFrame = false;
                }
            }
            if (crsfInFrame && crsfExpLen > 0 && crsfBufIdx >= crsfExpLen) {
                uint8_t rxCrc  = crsfBuf[crsfExpLen - 1];
                uint8_t calCrc = crsfCrc8(&crsfBuf[2], crsfExpLen - 3);
                if (rxCrc == calCrc) {
                    if (crsfBuf[2] == CRSF_TYPE_RC_CHAN) {
                        crsfDecodeChannels(&crsfBuf[3]);
                    }
                } else {
                    crcErrorCount++;
                }
                crsfInFrame = false;
            }
        }
    }
}

unsigned long elrsGetChannel(uint8_t index)
{
    if (index >= 16) return 1500;
    return channels[index];
}

bool elrsLinkOk(unsigned long timeout_ms)
{
    return (millis() - lastFrameMs) < timeout_ms;
}

uint32_t elrsGetCrcErrorCount()
{
    return crcErrorCount;
}

uint32_t elrsGetFrameCount()
{
    return frameCount;
}
