#include "main.h"
#include "state_machine_utils.h"

void set_led_intensity(uint8_t color, float intensity) {
    uint32_t ccr_val = (uint32_t)( ((100 - intensity)*ARR_VAL)/100 );
    switch (color) {
        case 1:
            htim3.Instance->CCR1 = ccr_val;
        case 2:
            htim3.Instance->CCR2 = ccr_val;
        case 3:
            htim3.Instance->CCR3 = ccr_val;
    }
}

void start_timers() {
    HAL_TIM_Base_Start(&htim3);

    set_led_intensity(RED, 0);
    set_led_intensity(GREEN, 0);
    set_led_intensity(BLUE, 0);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void init_can() {
    if (CANBus_init(&hcan) != HAL_OK) { Error_Handler(); }
    // subscribe CANBus to receive State Change Reqs and BMS + MC acks
    if (CANBus_subscribe(STATE_CHANGE_REQ) != HAL_OK) { Error_Handler(); }
    if (CANBus_subscribe(BMS_STATE_CHANGE_ACK_NACK) != HAL_OK) { Error_Handler(); };
    if (CANBus_subscribe(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK) != HAL_OK) { Error_Handler(); };
}

void setLEDColour(float R, float G, float B) {
    set_led_intensity(RED, R);
    set_led_intensity(GREEN, G);
    set_led_intensity(BLUE, B);
}

void setLEDBlink(float R, float G, float B) {
    // set LEDs on, off, then back on
    setLEDColour(R, G, B);
    HAL_Delay(200);
    setLEDColour(0.0, 0.0, 0.0);
    HAL_Delay(200);
    setLEDColour(R, G, B);
}