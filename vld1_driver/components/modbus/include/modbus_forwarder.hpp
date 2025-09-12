#pragma once
#include <cstdint>
#include "hal_rs485.hpp"

class ModbusForwarder {
public:
    explicit ModbusForwarder(hal::Rs485& rs485) noexcept;
    void forwardPDAT(uint16_t distance_mm, uint16_t magnitude, uint16_t avg_mm) noexcept;
    static uint16_t crc16(const uint8_t* data, uint16_t length) noexcept;

private:
    hal::Rs485& rs485_;
};

