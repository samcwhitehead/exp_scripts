#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "Arduino.h"

const size_t WB_DT_NUM_AVERAGE = 10;

// WB and Frame constants
extern const uint32_t WB_AVG_TIMEOUT_US; 
extern const uint32_t WB_FLYING_TIMEOUT_US;
extern const uint32_t WB_FALLBACK_PERIOD_US;
extern const uint32_t WB_PER_FRAME;
extern const uint32_t WB_DFLT_TRIG_PERCENT;  
extern const uint32_t LED_PULSE_US;

// Pin assignment
extern const uint8_t WB_PIN;          
extern const uint8_t CAM_LEFT_TRIG_PIN;
extern const uint8_t CAM_RIGHT_TRIG_PIN;
extern const uint8_t LED_PIN;

// Command ids
const uint8_t CMD_SET_WB_PERCENT = 1;
const uint8_t CMD_GET_WB_PERCENT = 2;
const uint8_t CMD_CAM_ENABLED = 3;
const uint8_t CMD_CAM_DISABLED = 4;
const uint8_t CMD_GET_CAM_ENABLE = 5;
const uint8_t CMD_LED_ENABLED = 6;
const uint8_t CMD_LED_DISABLED = 7;
const uint8_t CMD_GET_LED_ENABLE = 8;

// Camera ids
extern const uint8_t CAMERA_LEFT;
extern const uint8_t CAMERA_RIGHT;


#endif
