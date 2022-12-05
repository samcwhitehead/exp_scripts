#include "Arduino.h"
#include "constants.h"
#include "system_state.h"


SystemState::SystemState() { };

// Public Methods
// ----------------------------------------------------------------------------

void SystemState::initialize() {
  pinMode(CAM_LEFT_TRIG_PIN, OUTPUT);
  pinMode(CAM_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);  
  pinMode(WB_PIN, INPUT_PULLUP);
}


void SystemState::update_on_wb_isr() {
    wb_flag_ = true;
}


void SystemState::update_on_loop() {

    uint32_t t = micros();
    uint32_t wb_dt = t - last_wb_t_;

    // On a new wingbeat store time and set values so that we are ready to fire
    // pulses.
    if (is_new_wb()) { 
        last_wb_t_ = t;
        pulse_fired_ = false;
        if (wb_dt < WB_AVG_TIMEOUT_US) {
            // Only store wingbeat dt for averaging if it is in a reasonable range
            avg_buffer_.add(wb_dt);
        }
        if (!is_flying_) {
            digitalWrite(CAM_LEFT_TRIG_PIN, LOW);
            digitalWrite(CAM_RIGHT_TRIG_PIN, LOW);
            digitalWrite(LED_PIN, LOW); 
        }
        is_flying_ = true;
    }

    // If it has been too long since last wingbeat assume fly is not flying
    if (wb_dt > WB_FLYING_TIMEOUT_US) {
        is_flying_ = false;
        pulse_count_ = 0;
    }

    // If fly is not flying fallback to using a default period so cameras
    // still receive frames.
    if (!is_flying_) {
        if (wb_dt >= WB_FALLBACK_PERIOD_US) {
            last_wb_t_ = t;
            pulse_fired_ = false;
        }
    }


    // Set digital pins to trigger cameras and muscle imaging LED
    if (!pulse_fired_) {
        // Turn and LED and, if this is the first pulse, the cameras
        uint32_t wb_dt_avg = avg_buffer_.average();
        uint32_t dt_since_last_wb = t - last_wb_t_;
        if (dt_since_last_wb > (wb_dt_avg*WB_TRIG_PERCENT)/100) {
            digitalWrite(LED_PIN, HIGH); 
            if (pulse_count_ == 0) { 
                digitalWrite(CAM_LEFT_TRIG_PIN, HIGH);
                digitalWrite(CAM_RIGHT_TRIG_PIN, HIGH);
            }
            pulse_fired_ = true;
            pulse_count_ += 1;
            last_pulse_t_ = t;
        }
    }
    else {
        // Turn off LED and,  if this is the last pulse, the cameras
       uint32_t dt_since_last_pulse = t - last_pulse_t_;
        if (dt_since_last_pulse > LED_PULSE_US) {
            digitalWrite(LED_PIN, LOW); 
            if (pulse_count_ >= WB_PER_FRAME) {
                digitalWrite(CAM_LEFT_TRIG_PIN, LOW);
                digitalWrite(CAM_RIGHT_TRIG_PIN, LOW);
                pulse_count_ = 0;
            }
        }
    }
}


// Protected Methods
// ----------------------------------------------------------------------------

bool SystemState::is_new_wb()
{
    bool wb_flag_copy = false;
    noInterrupts();
    wb_flag_copy = wb_flag_;
    wb_flag_ = false;
    interrupts();
    return wb_flag_copy;
}


