/*
 * PiLink.cpp
 *
 *  Created on: Oct 11, 2026
 *      Author: Tim Flannery
 */


#include "PiLink.h"
#include "task.h"
#include <cstring>
#include <cmath>

extern UART_HandleTypeDef huart2;  // declared in main.cpp

QueueHandle_t     PiLink::txQueue_   = nullptr;
SemaphoreHandle_t PiLink::txDoneSem_ = nullptr;
uint8_t           PiLink::imuSeq_    = 0;

void PiLink::init() {
    txQueue_   = xQueueCreate(kTxQueueLen, sizeof(TxItem));
    txDoneSem_ = xSemaphoreCreateBinary();

    xTaskCreate(uartTxTask, "UartTxTask", 256, nullptr, tskIDLE_PRIORITY + 1, nullptr);
}

uint16_t PiLink::crc16CcittFalse(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

float PiLink::wrapToPi(float angle) {
    while (angle > static_cast<float>(M_PI))  angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void PiLink::sendImuFrame(uint32_t timestampMs, float yawUnwrapped,
                           float gxRadS, float gyRadS, float gzRadS,
                           float axMs2, float ayMs2, float azMs2) {
    if (txQueue_ == nullptr) return;  // init() not called yet — drop silently

    TxItem  item;
    uint8_t payload[kImuPayloadLen];
    size_t  p = 0;

    payload[p++] = imuSeq_++;  // wraps at 256 on its own

    std::memcpy(&payload[p], &timestampMs, 4); p += 4;

    float yaw = wrapToPi(yawUnwrapped);
    std::memcpy(&payload[p], &yaw, 4); p += 4;

    std::memcpy(&payload[p], &gxRadS, 4); p += 4;
    std::memcpy(&payload[p], &gyRadS, 4); p += 4;
    std::memcpy(&payload[p], &gzRadS, 4); p += 4;
    std::memcpy(&payload[p], &axMs2, 4); p += 4;
    std::memcpy(&payload[p], &ayMs2, 4); p += 4;
    std::memcpy(&payload[p], &azMs2, 4); p += 4;  // p == 33

    size_t f = 0;
    item.data[f++] = kSync;
    item.data[f++] = kMsgImu;
    item.data[f++] = static_cast<uint8_t>(kImuPayloadLen);
    std::memcpy(&item.data[f], payload, kImuPayloadLen); f += kImuPayloadLen;

    uint16_t crc = crc16CcittFalse(&item.data[1], 2 + kImuPayloadLen);  // type+len+payload
    item.data[f++] = crc & 0xFF;
    item.data[f++] = (crc >> 8) & 0xFF;
    item.len = f;  // 38

    // Non-blocking: if the queue is already full (TX falling behind), drop
    // this frame rather than stall ImuTask. The seq gap tells the Pi a
    // frame was lost.
    xQueueSend(txQueue_, &item, 0);
}

void PiLink::onTxComplete() {
    BaseType_t higherPriorityWoken = pdFALSE;
    xSemaphoreGiveFromISR(txDoneSem_, &higherPriorityWoken);
    portYIELD_FROM_ISR(higherPriorityWoken);
}

void PiLink::uartTxTask(void*) {
    TxItem item;
    for (;;) {
        if (xQueueReceive(txQueue_, &item, portMAX_DELAY) == pdTRUE) {
        	if (HAL_UART_Transmit_DMA(&huart2, item.data, item.len) == HAL_OK) {
        		xSemaphoreTake(txDoneSem_, portMAX_DELAY);  // waits for HAL_UART_TxCpltCallback
        	}
        	// if it wasn't HAL_OK, the transfer never started — drop this
        	// frame and move on instead of blocking on a completion that
        	// will never arrive
        }
    }
}

