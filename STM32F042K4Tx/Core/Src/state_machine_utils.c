#include "main.h"
#include "state_machine_utils.h"

// global colour variables (used by tim14 interrupt)
float red;
float green;
float blue; 

// global blink variables
int blink;
int ledON;

/* 
 * Purpose: Set a specific RGB LED's intensity through PWM channel register
 * 
 * Input: 
 *  uint8_t colour (RED, GREEN, BLUE)
 *  float intensity (0.0 - 100.0)
 */
void set_led_intensity(uint8_t colour, float intensity) {
    uint32_t ccr_val = (uint32_t)( ((100 - intensity)*ARR_VAL)/100 );
    switch (colour) {
        case 1:
            htim3.Instance->CCR1 = ccr_val;
        case 2:
            htim3.Instance->CCR2 = ccr_val;
        case 3:
            htim3.Instance->CCR3 = ccr_val;
    }
}

/* 
 * Purpose: Initialize TIM3, TIM14, LED values, and start PWM 
 *
 * TIM3 -> PWM timer 
 * TIM14 -> Timer interrupt to flash the LED
 * 
 */
void start_timers() {
    // start tim3 
    HAL_TIM_Base_Start(&htim3);

    // start tim14 (interrupt timer)
    HAL_TIM_Base_Start_IT(&htim14);
    __HAL_TIM_SET_COUNTER(&htim14, 0);

    // initialize global colour variables to 0 
    red = 0.0;
    green = 0.0; 
    blue = 0.0;

    // initialize blink and ledON bool flags to false
    blink = 0;
    ledON = 0; 

    // init pwm duty == off
    set_led_intensity(RED, 0);
    set_led_intensity(GREEN, 0);
    set_led_intensity(BLUE, 0);

    // start PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

/* 
 * Purpose: initialize the device to use the CAN bus and subscribe to relevant messages 
 *
 */
void init_can() {
    if (CANBus_init(&hcan) != HAL_OK) { Error_Handler(); }
    // subscribe CANBus to receive State Change Reqs and BMS + MC acks
    if (CANBus_subscribe(STATE_CHANGE_REQ) != HAL_OK) { Error_Handler(); }
    if (CANBus_subscribe(BMS_STATE_CHANGE_ACK_NACK) != HAL_OK) { Error_Handler(); };
    if (CANBus_subscribe(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK) != HAL_OK) { Error_Handler(); };
}

/* 
 * Purpose: Set overall LED colour by setting individual RGB intensity
 *
 */
void setLEDColour(float R, float G, float B) {

    set_led_intensity(RED, R);
    set_led_intensity(GREEN, G);
    set_led_intensity(BLUE, B);
}

/* 
 * Purpose: Subroutine that waits for acknowledgements from the MC and BMS 
 * based on a given STATE_CHANGE_REQ message/event 
 * 
 * Input: StateID 
 * 
 */ 
void waitForAck(StateID stateEvent) {
  int BMS_ACK = 0;
  int MC_ACK = 0;

  // busy wait for acknowledgements from MC and BMS 
  while(!BMS_ACK && !MC_ACK) {
    // if a msg is received
    if(!Queue_empty(&RX_QUEUE)) {
      CANFrame ackFrame = CANBus_get_frame(); 

      // check that BMS_ACK has been received
      if( !BMS_ACK && ackFrame.id == 0xB) {
        int ackEvent = CANFrame_get_field(&ackFrame, 
                        BMS_STATE_CHANGE_ACK_ID);
        unsigned int ack = CANFrame_get_field(&ackFrame, 
                        BMS_STATE_CHANGE_ACK);
        if(ackEvent == stateEvent && ack == 0) {
          BMS_ACK = 1;
        }
      } 
      
      // check that MC_ACK has been received
      if( !MC_ACK && ackFrame.id == 0x15) {
        int ackEvent = CANFrame_get_field(&ackFrame, 
                          MOTOR_CONTROLLER_STATE_CHANGE_ACK_ID);
        unsigned int ack = CANFrame_get_field(&ackFrame, 
                          MOTOR_CONTROLLER_STATE_CHANGE_ACK);
        if(ackEvent == stateEvent && ack == 0) {
          MC_ACK = 1;
        } 
      }

      // what about receiving a NACK?
        // wait until receiving an ACK
    }
  }
}

/* 
 * Purpose: State machine sets the global colours,
 * actual colour setting is done via tim14 interrupt. 
 * The interrupt is called every 200 ms.
 * 
 * See HAL_TIM_PeriodElapsedCallback below
 * 
 * Input: StateID 
 * Return: StateID 
 */
StateID stateMachine(StateID stateEvent) {
    if(!Queue_empty(&RX_QUEUE)) {
      CANFrame rxFrame = CANBus_get_frame();

      if(rxFrame.id == 0) {
        stateEvent = CANFrame_get_field(&rxFrame, STATE_ID); 
      }

      if(stateEvent == ARMED) {
        // set LED to white if not acked yet by BMS 
        red = 50.0;
        green = 50.0;
        blue = 50.0;
        blink = 0;
      }

      // wait for MC and BMS to ack if not failure state
      if(stateEvent != SYSTEM_FAILURE) {
        waitForAck(stateEvent);
      }
    }

    // State machine
    switch (stateEvent)
    {
    case RESTING:
      // "sleeping", yellow
      red = 50.0;
      green = 50.0;
      blue = 0.0;
      blink = 0;
    break; 
    case LV_READY:
    case ARMED: 
    case MANUAL_OPERATION_WAITING: 
      // "idle", blinking yellow
      red = 50.0;
      green = 50.0;
      blue = 0.0;
      blink = 1;
      // set PWM to blinking 
    break;
    case BRAKING: 
    case EMERGENCY_BRAKE:
    case DECELERATING:
      // "stop" blinking green
      red = 0.0;
      green = 50.0;
      blue = 0.0;
      blink = 1; 
      // set PWM to blinking 
    break;
    case AUTO_PILOT:
    case ACCELERATING:
    case AT_SPEED:
      // "run", green
      red = 0.0;
      green = 50.0;
      blue = 0.0;
      blink = 0;
    break;
    case SYSTEM_FAILURE: 
      // "severe danger fault", flashing red
      red = 50.0;
      green = 0.0;
      blue = 0.0;
      blink = 1;
      // set PWM to blinking
    break;
    default:
      // "initialize/normal danger fault", solid red?
      red = 50.0;
      green = 0.0;
      blue = 0.0;
      blink = 0;
    break;
    }

    return stateEvent;
}

int pilMain() {
  start_timers();
  init_can(); 
  StateID stateEvent = RESTING;

  while(1) {
     stateEvent = stateMachine(stateEvent); 
  }

  return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* 
   * tim14 should call interrupt every 200 ms
   * freq = clock freq / (prescalar + 1) * (counter period + 1) * (repetition counter + 1)
   *      = 42 MHz/(199 + 1)(41999+1)(1)
   *      = 42 MHz/(200)(42000)
   *      = 5 Hz 
   * period = 1/5 s = 200 ms
   */ 
  if(blink) {
    if(ledON) {
      setLEDColour(0.0, 0.0, 0.0);
    } else {
      setLEDColour(red, green, blue);
    }

    ledON = ledON ? 0 : 1;

  } else { // non-flashing colour
    setLEDColour(red, green, blue);
  }
}