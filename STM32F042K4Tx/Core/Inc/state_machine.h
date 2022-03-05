#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

// typedef enum {
//     standbyEvent,
//     movingEvent,
//     brakingEvent,
//     errorEvent,
//     batteryErrorEvent
// } PodEvent;


// void (*nextState(void))(PodEvent*) standbyState( PodEvent *next ); 

// void (*nextState(void))(PodEvent*) movingState( PodEvent *next ); 

// void (*nextState(void))(PodEvent*) brakingState( PodEvent *next ); 

// void (*nextState(void))(PodEvent*) BatteryErrorState( PodEvent *next ); 

// void (*nextState(void))(PodEvent*) GenericErrorState( PodEvent *next ); 

// void (*currentState)() = NULL; 

#define ARR_VAL    65535

typedef enum {
    RED = 3,
    GREEN = 1,
    BLUE = 2
} LED_COLOR;

typedef void *(StateMachine)();

void set_led_intensity(uint8_t color, float intensity); 
void start_timers(); 
void init_can();

void setLEDColour(float R, float G, float B);

void *standbyState(void);
void *movingState(void); 
void *brakingState(void);
void *batteryErrorState(void); 
void *genericErrorState(void); 

#endif /* __STATE_MACHINE_H */