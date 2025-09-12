#include "vld1.hpp"
#include "hal_uart.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "VLD1";

Vld1::Vld1(hal::Uart& uart) noexcept : uart_(uart) {}

void Vld1::sendPacket(const char header[4], const uint8_t* payload, uint32_t payload_len) noexcept {
    uint8_t buf[512];
    if (payload_len + 8 > sizeof(buf)) return;
    std::memcpy(buf, header, 4);
    buf[4] = payload_len & 0xFF;
    buf[5] = (payload_len >> 8) & 0xFF;
    buf[6] = (payload_len >> 16) & 0xFF;
    buf[7] = (payload_len >> 24) & 0xFF;
    if (payload_len && payload) std::memcpy(buf + 8, payload, payload_len);
    uart_.write(buf, 8 + payload_len);
}

int Vld1::parseMessage(uint8_t* buffer, int len, char* out_header, uint8_t* out_payload, uint32_t* out_len) noexcept {
    if (len < 8) return 0;
    std::memcpy(out_header, buffer, 4);
    out_header[4] = '\0';
    uint32_t payload_len = static_cast<uint32_t>(buffer[4]) |
                           (static_cast<uint32_t>(buffer[5]) << 8) |
                           (static_cast<uint32_t>(buffer[6]) << 16) |
                           (static_cast<uint32_t>(buffer[7]) << 24);
    *out_len = payload_len;
    if (len < static_cast<int>(8 + payload_len)) return 0;
    if (payload_len > 0 && out_payload) std::memcpy(out_payload, buffer + 8, payload_len);
    return static_cast<int>(8 + payload_len);
}

