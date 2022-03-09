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

// global colour variables (used by tim14 interrupt)
extern float red;
extern float green;
extern float blue; 

// global blink variables
extern int blink;
extern int ledON;

void set_led_intensity(uint8_t colour, float intensity); 
void start_timers(void); 
void init_can(void);
void setLEDColour(float R, float G, float B);
void waitForAck(StateID stateEvent);

StateID stateMachine(StateID stateEvent);
int pilMain(); 