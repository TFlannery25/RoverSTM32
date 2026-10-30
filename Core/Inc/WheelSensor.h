// WheelSensor.h
#pragma once

#include <cstdint>

// Starts TIM3 input-capture on channel 1 (front-left wheel sensor).
// Call once from main.cpp's USER CODE BEGIN 2, after MX_TIM3_Init().
void WheelSensor_Init();

// Front-left wheel speed in rpm. Returns 0.0f if stopped or no pulses yet.
float WheelSensor_GetRpmLF();

// Front-left total pulse count since boot (wraps at UINT32_MAX, not handled yet).
uint32_t WheelSensor_GetCountLF();

extern volatile uint32_t g_lfCount;
extern volatile float    g_lfRpm;
