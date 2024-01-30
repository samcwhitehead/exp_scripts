#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H
#include "averaging_buffer.h"
#include "constants.h"
#include "SerialReceiver.h"

class SystemState {

    public:
        SystemState();
        void initialize();
        void update_on_loop();
        void update_on_wb_isr();
        void process_messages();

    protected:

        volatile bool wb_flag_ = false;
        bool is_flying_ = false;
        uint32_t last_wb_t_ = 0;
        uint32_t last_pulse_t_ = 0;
        uint32_t pulse_count_ = 0;
        uint32_t pulse_fired_ = false;
        uint32_t wb_trig_percent_ = WB_DFLT_TRIG_PERCENT;
        bool cam_left_enabled_ = true;
        bool cam_right_enabled_ = true;
        bool led_enabled_ = true;

        AveragingBuffer avg_buffer_;
        SerialReceiver receiver_;

        bool is_new_wb();
        void handle_message(uint8_t cmd_num, uint8_t cmd_val);
};

#endif
