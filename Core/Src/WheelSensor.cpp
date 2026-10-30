// WheelSensor.cpp
#include "main.h"
#include "cmsis_os.h"
#include "WheelSensor.h"

extern TIM_HandleTypeDef htim3;

namespace {
constexpr uint32_t TICK_HZ        = 100000;  // 10 us timer tick (TIM3 prescaler)
constexpr uint32_t SLOTS_PER_REV  = 20;      // <-- disc's slot count
constexpr uint32_t MIN_TICKS      = 300;     // 3 ms: shorter periods are noise, ignored
constexpr uint32_t STOP_TIMEOUT_MS = 600;    // no edge this long -> wheel is stopped

struct WheelSensor {
    volatile uint16_t lastCapture   = 0;
    volatile uint32_t periodTicks   = 0;   // time between the last two pulses
    volatile uint32_t lastEdgeMs    = 0;
    volatile uint32_t count         = 0;   // total pulses (unsigned, sign comes later)
    volatile bool     havePrev      = false;
};

WheelSensor lf;   // front-left
}

volatile uint32_t g_lfCount = 0;   // Live Expressions
volatile float    g_lfRpm   = 0.0f;

// Called by the HAL on every rising edge captured on TIM3.
// Only one HAL_TIM_IC_CaptureCallback may exist in the project.
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM3) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint16_t now   = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t nowMs = HAL_GetTick();

        if (lf.havePrev && (nowMs - lf.lastEdgeMs) < STOP_TIMEOUT_MS) {
            uint16_t period = (uint16_t)(now - lf.lastCapture);   // wrap-safe
            if (period < MIN_TICKS) return;                       // glitch: ignore this edge
            lf.periodTicks = period;
        } else {
            lf.periodTicks = 0;   // first edge after a stop: no valid period yet
        }

        lf.lastCapture = now;
        lf.lastEdgeMs  = nowMs;
        lf.havePrev    = true;
        lf.count++;
    }
}

void WheelSensor_Init()
{
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

// Wheel speed in rpm (always >= 0 for now).
float WheelSensor_GetRpmLF()
{
    taskENTER_CRITICAL();
    uint32_t period = lf.periodTicks;
    uint32_t lastMs = lf.lastEdgeMs;
    bool     have   = lf.havePrev;
    taskEXIT_CRITICAL();

    uint32_t sinceMs = HAL_GetTick() - lastMs;
    if (!have || period == 0 || sinceMs > STOP_TIMEOUT_MS) return 0.0f;

    // If it's been longer since the last pulse than the last period, the wheel
    // is slowing down: use the elapsed time so the speed decays instead of freezing.
    float periodSec = (float)period / TICK_HZ;
    float sinceSec  = sinceMs / 1000.0f;
    if (sinceSec > periodSec) periodSec = sinceSec;

    float pulsesPerSec = 1.0f / periodSec;
    return pulsesPerSec * 60.0f / SLOTS_PER_REV;
}

uint32_t WheelSensor_GetCountLF() { return lf.count; }
