#pragma once
#include <cstdint>
#include <functional>
#include "driver/uart.h"

enum class SensorStatus : uint16_t {
    OK = 0,
    NO_DATA = 1,
    BAD_FRAME = 2,
    STALE = 3,
    OUT_OF_RANGE = 4,
    ERROR = 0xFFFF
};

struct VLD1Sample {
    int32_t distance_mm;      // signed mm
    uint16_t magnitude;       // amplitude/quality
    uint32_t frame_counter;   // optional per-packet counter if provided
    uint64_t timestamp_ms;    // monotonic ms from esp_timer
};

class VLD1Driver {
public:
    using SampleCb = std::function<void(const VLD1Sample&)>;
    using StatusCb = std::function<void(SensorStatus)>;

    VLD1Driver(uart_port_t port, int tx_gpio, int rx_gpio, int baud = 115200) noexcept;
    ~VLD1Driver();

    bool start() noexcept;  // installs UART, starts RX task
    void stop() noexcept;

    // set callbacks (called in RX task context)
    void set_sample_callback(SampleCb cb) noexcept;
    void set_status_callback(StatusCb cb) noexcept;

    // send text/binary command to the sensor (synchronous)
    bool send_command(const uint8_t* buf, size_t len, TickType_t wait_ms = pdMS_TO_TICKS(200)) noexcept;

    // diagnostic counters (atomic)
    uint32_t bad_frames() const noexcept;
    uint32_t crc_errors() const noexcept;
    uint32_t valid_frames() const noexcept;

private:
    struct Impl;
    Impl* impl_;
};
