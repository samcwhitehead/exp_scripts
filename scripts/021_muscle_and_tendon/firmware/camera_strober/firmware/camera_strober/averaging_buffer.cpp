#include "averaging_buffer.h"

AveragingBuffer::AveragingBuffer() {};


uint32_t AveragingBuffer::size() {
    return uint32_t(buffer_.size());
}


uint32_t AveragingBuffer::average() {
    uint32_t average = 0 ;
    for (uint32_t i=0; i<buffer_.size(); i++) {
        average += buffer_[i];
    }
    if (buffer_.size() > 0) {
        average /= buffer_.size();
    }
    return average;
}


void AveragingBuffer::add(uint32_t val) {
    if (buffer_.full()) {
        buffer_.pop_front();
    }
    buffer_.push_back(val);
}


void AveragingBuffer::clear() {
    buffer_.clear();
}
