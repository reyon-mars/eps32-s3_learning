#include "averager.hpp"
#include "esp_timer.h"
#include <algorithm>

BatchAverager::BatchAverager(size_t batch_size) noexcept
    : batch_size_(batch_size), count_(0), curr_avg_m_(0.0), start_us_(0) {}

void BatchAverager::addSample(double meters) noexcept {
    if (count_ == 0) start_us_ = esp_timer_get_time();
    ++count_;
    // incremental average
    curr_avg_m_ += (meters - curr_avg_m_) / static_cast<double>(count_);
}

bool BatchAverager::isComplete() const noexcept {
    return count_ >= batch_size_;
}

double BatchAverager::averageMeters() const noexcept {
    return curr_avg_m_;
}

uint16_t BatchAverager::averageMillimeters() const noexcept {
    double mm = curr_avg_m_ * 1000.0;
    if (mm < 0) return 0;
    if (mm > 65535.0) return 0xFFFF;
    return static_cast<uint16_t>(mm + 0.5);
}

int64_t BatchAverager::elapsedUs() const noexcept {
    if (count_ == 0) return 0;
    return esp_timer_get_time() - start_us_;
}

void BatchAverager::reset() noexcept {
    count_ = 0;
    curr_avg_m_ = 0.0;
    start_us_ = 0;
}

