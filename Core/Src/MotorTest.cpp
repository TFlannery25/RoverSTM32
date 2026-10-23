// MotorTest.cpp
#include "main.h"
#include "cmsis_os.h"
#include <cstdlib>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

struct MotorChannel {
    uint32_t      timChannel;
    GPIO_TypeDef* in1Port; uint16_t in1Pin;
    GPIO_TypeDef* in2Port; uint16_t in2Pin;
    int8_t        invert;    // -1 if this wheel runs backward for "forward"
};

enum Wheel { LF = 0, LR, RF, RR, NUM_WHEELS };

static MotorChannel wheels[NUM_WHEELS] = {
    { TIM_CHANNEL_1, LF_IN1_GPIO_Port, LF_IN1_Pin, LF_IN2_GPIO_Port, LF_IN2_Pin,  1 },
    { TIM_CHANNEL_2, LR_IN1_GPIO_Port, LR_IN1_Pin, LR_IN2_GPIO_Port, LR_IN2_Pin,  1 },
    { TIM_CHANNEL_3, RF_IN1_GPIO_Port, RF_IN1_Pin, RF_IN2_GPIO_Port, RF_IN2_Pin,  1 },
    { TIM_CHANNEL_4, RR_IN1_GPIO_Port, RR_IN1_Pin, RR_IN2_GPIO_Port, RR_IN2_Pin,  1 },
};

volatile int32_t g_dutyPercent = 0;
volatile int32_t g_mode        = 0;
static const int32_t NUM_MODES = 6;

static void setMotor(const MotorChannel& m, int32_t duty, uint32_t arr)
{
    duty *= m.invert;
    if (duty > 0) {
        HAL_GPIO_WritePin(m.in1Port, m.in1Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m.in2Port, m.in2Pin, GPIO_PIN_RESET);
    } else if (duty < 0) {
        HAL_GPIO_WritePin(m.in1Port, m.in1Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m.in2Port, m.in2Pin, GPIO_PIN_SET);
        duty = -duty;
    } else {
        HAL_GPIO_WritePin(m.in1Port, m.in1Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m.in2Port, m.in2Pin, GPIO_PIN_RESET);
    }
    if ((uint32_t)duty > arr) duty = (int32_t)arr;
    __HAL_TIM_SET_COMPARE(&htim1, m.timChannel, (uint32_t)duty);
}

static int32_t readKnobDuty(uint32_t arr)
{
    const int32_t deadband = 150;
    HAL_ADC_Start(&hadc1);
    int32_t raw = 2048;
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        raw = (int32_t)HAL_ADC_GetValue(&hadc1);
    }
    int32_t centered = raw - 2048;
    if (std::abs(centered) <= deadband) return 0;
    int32_t mag  = std::abs(centered) - deadband;
    int32_t duty = (int32_t)((int64_t)mag * arr / (2047 - deadband));
    return centered < 0 ? -duty : duty;
}

extern "C" void MotorTestTask(void* /*arg*/)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    GPIO_PinState lastButton = GPIO_PIN_SET;   // B1 is active low

    for (;;) {
    	GPIO_PinState b = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN);
        if (lastButton == GPIO_PIN_SET && b == GPIO_PIN_RESET) {
            g_mode = (g_mode + 1) % NUM_MODES;
        }
        lastButton = b;

        int32_t duty = readKnobDuty(arr);
        int32_t cmd[NUM_WHEELS] = {0, 0, 0, 0};

        switch (g_mode) {
            case 0: cmd[LF] = cmd[LR] = cmd[RF] = cmd[RR] = duty; break;
            case 1: cmd[LF] = duty; break;
            case 2: cmd[LR] = duty; break;
            case 3: cmd[RF] = duty; break;
            case 4: cmd[RR] = duty; break;
            case 5: cmd[LF] = cmd[LR] = -duty; cmd[RF] = cmd[RR] = duty; break;
        }

        g_dutyPercent = duty * 100 / (int32_t)arr;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            setMotor(wheels[i], cmd[i], arr);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
