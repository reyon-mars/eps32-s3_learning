#pragma once
#include <cstdint>
#include <cstddef>

class Vld1 {
public:
    Vld1(class hal::Uart& uart) noexcept;
    // send a 4-char command + payload
    void sendPacket(const char header[4], const uint8_t* payload, uint32_t payload_len) noexcept;

    // parse a buffer, returns total parsed bytes (0 if incomplete)
    // out_header must be at least 5 bytes (null-terminated), out_payload buffer must be large enough
    int parseMessage(uint8_t* buffer, int len, char* out_header, uint8_t* out_payload, uint32_t* out_len) noexcept;

private:
    hal::Uart& uart_;
};

