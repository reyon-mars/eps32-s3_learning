#pragma once
#include <cstdint>
#include <cstddef>
#include "uart.hpp"
class vld1
{
public:
    vld1(uart &uart_no) noexcept;

    void send_packet(const char header[4], const uint8_t *payload, uint32_t payload_len) noexcept;

    int parse_message(uint8_t *buffer, int len, char *out_header, uint8_t *out_payload, uint32_t *out_len) noexcept;

private:
    uart &uart_;
};
