#ifndef AVERAGING_BUFFER_H
#define AVERAGING_BUFFER_H
#include "constants.h"
#include "circular_buffer.h"

class AveragingBuffer {

    public:
        AveragingBuffer();
        uint32_t size();
        uint32_t average();
        void add(uint32_t val);
        void clear();

    protected: 
        CircularBuffer<uint32_t, WB_DT_NUM_AVERAGE> buffer_;

};

#endif
