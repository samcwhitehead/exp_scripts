#include "constants.h"

// WB and Frame constants 
const uint32_t WB_AVG_TIMEOUT_US = 10000; 
const uint32_t WB_FLYING_TIMEOUT_US = 30000;
const uint32_t WB_FALLBACK_PERIOD_US = 5555;
const uint32_t WB_PER_FRAME = 2;
const uint32_t WB_TRIG_PERCENT = 50; // Trigger delay in percent wing beat 
extern const uint32_t LED_PULSE_US = 1000;

// Pin assignment
const uint8_t WB_PIN = 2;          
const uint8_t CAM_LEFT_TRIG_PIN = 5;
const uint8_t CAM_RIGHT_TRIG_PIN = 7;
const uint8_t LED_PIN = 3;
