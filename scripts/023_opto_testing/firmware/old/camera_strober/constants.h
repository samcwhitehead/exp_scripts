#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "Arduino.h"

const size_t WB_DT_NUM_AVERAGE = 10;

// WB and Frame constants
extern const uint32_t WB_AVG_TIMEOUT_US; 
extern const uint32_t WB_FLYING_TIMEOUT_US;
extern const uint32_t WB_FALLBACK_PERIOD_US;
extern const uint32_t WB_PER_FRAME;
extern const uint32_t WB_TRIG_PERCENT;  
extern const uint32_t LED_PULSE_US;

// Pin assignment
extern const uint8_t WB_PIN;          
extern const uint8_t CAM_LEFT_TRIG_PIN;
extern const uint8_t CAM_RIGHT_TRIG_PIN;
extern const uint8_t LED_PIN;


#endif
