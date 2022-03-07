#pragma once

// typedef enum {
//     standbyEvent,
//     movingEvent,
//     brakingEvent,
//     errorEvent,
//     batteryErrorEvent
// } PodEvent;

#include "can.h"

#define ARR_VAL    65535

typedef enum {
    RED = 3,
    GREEN = 1,
    BLUE = 2
} LED_COLOR;

void set_led_intensity(uint8_t color, float intensity); 
void start_timers(); 
void init_can();
void setLEDColour(float R, float G, float B);
void setLEDBlink(float R, float G, float B);