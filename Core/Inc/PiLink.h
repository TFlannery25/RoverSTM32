/*
 * PiLink.h
 *
 * Pi <-> STM32 serial protocol: CRC16, frame packing, and the
 *  dedicated UART-TX task/queue so IMU (and later wheel/status)
 *  frames never interleave on the wire.
 *
 *  Created on: Oct 11, 2026
 *      Author: Tim Flannery
 */

#ifndef INC_PILINK_H_
#define INC_PILINK_H_

 #include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

class PiLink {
public:
    // Call once from main(), in USER CODE BEGIN 2, alongside the existing
    // xTaskCreate(ImuTask, ...) call — before osKernelStart().
    static void init();

    // Call from ImuTask every loop iteration — no throttling. This is the
    // real 50 Hz telemetry stream, not the old once-per-second debug print.
    // yawUnwrapped is the filter's internal, unwrapped yaw; this wraps it
    // to [-pi, +pi] itself right before it goes on the wire, so ImuTask's
    // own yaw state is never touched.
    static void sendImuFrame(uint32_t timestampMs, float yawUnwrapped,
                              float gxRadS, float gyRadS, float gzRadS,
                              float axMs2, float ayMs2, float azMs2);

    // Call from HAL_UART_TxCpltCallback() in main.cpp's USER CODE BEGIN 4,
    // guarded by `if (huart->Instance == USART2)`.
    static void onTxComplete();

private:
    static constexpr uint8_t kSync          = 0xA5;
    static constexpr uint8_t kMsgImu        = 0x01;
    static constexpr size_t  kImuPayloadLen = 33;  // seq+ts+yaw+gyro(3)+accel(3)
    static constexpr size_t  kImuFrameLen   = 3 + kImuPayloadLen + 2;  // 38

    static constexpr size_t kTxQueueLen = 4;   // frames allowed in flight
    static constexpr size_t kMaxFrameLen = 48; // headroom over the 38-byte IMU frame

    struct TxItem {
        uint8_t data[kMaxFrameLen];
        size_t  len;
    };

    static uint16_t crc16CcittFalse(const uint8_t* data, size_t len);
    static float    wrapToPi(float angle);
    static void     uartTxTask(void* argument);

    static QueueHandle_t     txQueue_;
    static SemaphoreHandle_t txDoneSem_;
    static uint8_t            imuSeq_;
};

#endif /* INC_PILINK_H_ */
