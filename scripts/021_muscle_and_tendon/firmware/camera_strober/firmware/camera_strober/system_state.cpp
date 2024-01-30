#include "Arduino.h"
#include "Streaming.h"
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
        if (dt_since_last_wb > (wb_dt_avg*wb_trig_percent_)/100) {
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

void SystemState::process_messages()
{
    while (Serial.available() > 0) {
        receiver_.process(Serial.read());
        if (receiver_.messageReady()) {
            uint8_t cmd_num = receiver_.readInt(0);
            uint8_t cmd_val = receiver_.readInt(1);
            handle_message(cmd_num, cmd_val);
            receiver_.reset();
        }
    }
}

void SystemState::handle_message(uint8_t cmd_num, uint8_t cmd_val) {

    switch (cmd_num) {

        case CMD_SET_WB_PERCENT:
            wb_trig_percent_ = constrain(cmd_val,0,100);
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << wb_trig_percent_ << "}" << endl;
            break;

        case CMD_GET_WB_PERCENT:
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << wb_trig_percent_ << "}" << endl;
            break;

        case CMD_CAM_ENABLED:
            if (cmd_val == CAMERA_LEFT) {
                cam_left_enabled_ = true;
                pinMode(CAM_LEFT_TRIG_PIN, OUTPUT);
            } 
            if (cmd_val == CAMERA_RIGHT) {
                cam_right_enabled_ = true;
                pinMode(CAM_RIGHT_TRIG_PIN, OUTPUT);
            }
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << cmd_val << "}" << endl;
            break;

        case CMD_CAM_DISABLED:
            if (cmd_val == CAMERA_LEFT) {
                cam_left_enabled_ = false;
                pinMode(CAM_LEFT_TRIG_PIN, INPUT);
            } 
            if (cmd_val == CAMERA_RIGHT) {
                cam_right_enabled_ = false;
                pinMode(CAM_RIGHT_TRIG_PIN, INPUT);
            }
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << cmd_val << "}" << endl;
            break;

        case CMD_GET_CAM_ENABLE:
            if (cmd_val == CAMERA_LEFT) {
                Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << cam_left_enabled_ << "}" << endl;
            }
            if (cmd_val == CAMERA_RIGHT) {
                Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << cam_right_enabled_ << "}" << endl;
            }
            break;
            
        case CMD_LED_ENABLED:
            led_enabled_ = true;
            pinMode(LED_PIN, OUTPUT);
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << cmd_val << "}" << endl;
            break;
                
        case CMD_LED_DISABLED:
            led_enabled_ = false;
            pinMode(LED_PIN, INPUT);
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << cmd_val << "}" << endl;
            break;
            
        case CMD_GET_LED_ENABLE:
            Serial << "{\"cmd\":" << cmd_num << ",\"val\":" << led_enabled_ << "}" << endl;
            break;
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


