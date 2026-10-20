// MotorTest.cpp
#include "main.h"
#include "cmsis_os.h"
#include <cstdlib>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

volatile int32_t g_dutyPercent = 0;   // watch this in the debugger (Live Expressions)

static void setMotor(int32_t dutyCounts, uint32_t arr)
{
    if (dutyCounts > 0) {            // forward
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (dutyCounts < 0) {     // reverse
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        dutyCounts = -dutyCounts;
    } else {                         // coast
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    if ((uint32_t)dutyCounts > arr) dutyCounts = arr;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)dutyCounts);
}

extern "C" void MotorTestTask(void* /*arg*/)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    const int32_t  deadband = 150;          // ADC counts around center = stop

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);   // enable driver

    for (;;) {
        HAL_ADC_Start(&hadc1);
        int32_t raw = 2048;
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            raw = (int32_t)HAL_ADC_GetValue(&hadc1);            // 0..4095
        }

        int32_t centered = raw - 2048;                           // -2048..+2047
        int32_t duty = 0;
        if (std::abs(centered) > deadband) {
            int32_t mag = std::abs(centered) - deadband;         // 0..~1898
            duty = (int32_t)((int64_t)mag * arr / (2047 - deadband));
            if (centered < 0) duty = -duty;
        }

        g_dutyPercent = duty * 100 / (int32_t)arr;
        setMotor(duty, arr);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
