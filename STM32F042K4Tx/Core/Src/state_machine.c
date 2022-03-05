#include "main.h"
#include "state_machine.h"
#include "can.h"

// TODO - setup CAN library here
// TODO - setup LED access

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
    if (CANBus_init(&hcan1) != HAL_OK) { Error_Handler(); }
    if (CANBus_subscribe(STATE_CHANGE_REQ) != HAL_OK) { Error_Handler(); }
}

void setLEDColour(float R, float G, float B) {
    set_led_intensity(RED, R);
    set_led_intensity(GREEN, G);
    set_led_intensity(BLUE, B);
}

void *standbyState() {

    // set yellow to blinking 
    // set other leds to off 

    // poll can rx queue 

    // if msg is moving 
    if () {
        return movingState;
    } else if() { // error check

    } else {
        return standbyState;
    }
}

void *movingState() {

    // set green to solid
    // other leds to off

    // poll can rx queue 

    // if msg is braking
    if () {
        return brakingState;
    } else if() { // else check error

    } else {
        return movingState;
    }

}

void *brakingState() {

    // set green to blinking
    // set other leds to off

    // poll can rx queue 

    // if msg is braked 
    if() {
        return standbyState;
    } else if() {

    } else {
        return brakingState;
    }
}

void *batteryErrorState() {
    // leds blinking red if malfunctioning

    // how should we leave this state?

}

void *genericErrorState() {
    // solid red


    // how should we leave this state? manual reset???
}