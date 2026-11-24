// WheelSensor.cpp
#include "WheelSensor.h"
#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim3;

namespace {
constexpr uint32_t TICK_HZ         = 100000;  // 10 us tick
constexpr uint32_t SLOTS_PER_REV   = 20;
constexpr uint32_t MIN_TICKS       = 300;     // < 3 ms between edges = noise
constexpr uint32_t STOP_TIMEOUT_MS = 600;

struct SensorState {
    volatile uint16_t lastCapture = 0;
    volatile uint32_t periodTicks = 0;
    volatile uint32_t lastEdgeMs  = 0;
    volatile uint32_t count       = 0;
    volatile bool     havePrev    = false;
};

SensorState sensors[NUM_WHEELS];

// Wheel order matches TIM3 CH1..CH4 = LF, LR, RF, RR
const uint32_t kTimChannel[NUM_WHEELS] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };

int wheelFromActive(HAL_TIM_ActiveChannel ch)
{
    switch (ch) {
        case HAL_TIM_ACTIVE_CHANNEL_1: return LF;
        case HAL_TIM_ACTIVE_CHANNEL_2: return LR;
        case HAL_TIM_ACTIVE_CHANNEL_3: return RF;
        case HAL_TIM_ACTIVE_CHANNEL_4: return RR;
        default:                       return -1;
    }
}
} // namespace

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM3) return;

    int w = wheelFromActive(htim->Channel);
    if (w < 0) return;

    SensorState& s = sensors[w];
    uint16_t now   = (uint16_t)HAL_TIM_ReadCapturedValue(htim, kTimChannel[w]);
    uint32_t nowMs = HAL_GetTick();

    if (s.havePrev && (nowMs - s.lastEdgeMs) < STOP_TIMEOUT_MS) {
        uint16_t period = (uint16_t)(now - s.lastCapture);   // wrap-safe
        if (period < MIN_TICKS) return;                      // glitch
        s.periodTicks = period;
    } else {
        s.periodTicks = 0;   // first edge after a stop
    }

    s.lastCapture = now;
    s.lastEdgeMs  = nowMs;
    s.havePrev    = true;
    s.count++;
}

void WheelSensor_Init()
{
    for (int i = 0; i < NUM_WHEELS; ++i) {
        HAL_TIM_IC_Start_IT(&htim3, kTimChannel[i]);
    }
}

uint32_t WheelSensor_GetCount(Wheel w)
{
    return sensors[w].count;
}

float WheelSensor_GetRpm(Wheel w)
{
    taskENTER_CRITICAL();
    uint32_t period = sensors[w].periodTicks;
    uint32_t lastMs = sensors[w].lastEdgeMs;
    bool     have   = sensors[w].havePrev;
    taskEXIT_CRITICAL();

    uint32_t sinceMs = HAL_GetTick() - lastMs;
    if (!have || period == 0 || sinceMs > STOP_TIMEOUT_MS) return 0.0f;

    float periodSec = (float)period / TICK_HZ;
    float sinceSec  = sinceMs / 1000.0f;
    if (sinceSec > periodSec) periodSec = sinceSec;   // decays when slowing

    return (1.0f / periodSec) * 60.0f / SLOTS_PER_REV;
}
