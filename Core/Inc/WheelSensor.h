// WheelSensor.h
#pragma once
#include <cstdint>

enum Wheel { LF = 0, LR, RF, RR, NUM_WHEELS };

void     WheelSensor_Init();
float    WheelSensor_GetRpm(Wheel w);
uint32_t WheelSensor_GetCount(Wheel w);


