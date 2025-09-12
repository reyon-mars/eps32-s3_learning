#pragma once
#include <cstdint>
#include <cstddef>

class BatchAverager {
public:
    explicit BatchAverager(size_t batch_size = 20) noexcept;
    // add sample in meters
    void addSample(double meters) noexcept;

    bool isComplete() const noexcept;
    // average in meters (valid if any samples)
    double averageMeters() const noexcept;
    // average in millimeters rounded to uint16 (saturates at 0xFFFF)
    uint16_t averageMillimeters() const noexcept;

    // elapsed collection time for current batch in microseconds
    int64_t elapsedUs() const noexcept;

    void reset() noexcept;

private:
    size_t batch_size_;
    size_t count_;
    double curr_avg_m_;
    int64_t start_us_;
};
