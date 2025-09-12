#include "modbus_forwarder.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "ModbusForwarder";

ModbusForwarder::ModbusForwarder(hal::Rs485& rs485) noexcept : rs485_(rs485) {}

void ModbusForwarder::forwardPDAT(uint16_t distance_mm, uint16_t magnitude, uint16_t avg_mm) noexcept {
    uint8_t frame[12];
    int idx = 0;
    frame[idx++] = 0x01;
    frame[idx++] = 0x04;
    frame[idx++] = 0x06;

    frame[idx++] = static_cast<uint8_t>(distance_mm >> 8);
    frame[idx++] = static_cast<uint8_t>(distance_mm & 0xFF);

    frame[idx++] = static_cast<uint8_t>(magnitude >> 8);
    frame[idx++] = static_cast<uint8_t>(magnitude & 0xFF);

    frame[idx++] = static_cast<uint8_t>(avg_mm >> 8);
    frame[idx++] = static_cast<uint8_t>(avg_mm & 0xFF);

    uint16_t crc = crc16(frame, idx);
    frame[idx++] = crc & 0xFF;
    frame[idx++] = (crc >> 8) & 0xFF;

    rs485_.send(frame, idx);
    ESP_LOGI(TAG, "Forwarded PDAT: dist=%u mm mag=%u avg=%u mm", distance_mm, magnitude, avg_mm);
}

uint16_t ModbusForwarder::crc16(const uint8_t* data, uint16_t length) noexcept {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

