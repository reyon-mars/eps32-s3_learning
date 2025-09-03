#pragma once

#include <cstdint>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "serial_channel.hpp"

class RFbeamVld1
{
public:
    using PayloadSize_t = uint32_t;
    using RadcArray_t = uint16_t;
    using RfftMagArray_t = uint16_t;
    using RfftThreshArray_t = uint16_t;
    using PdatDistance_t = float;
    using PdataMag_t = uint16_t;
    using Done_t = uint32_t;

    static constexpr PayloadSize_t radcArraySize{1024};
    static constexpr PayloadSize_t rfftMagArrySize{512};
    static constexpr PayloadSize_t rfftThreshArrySize{512};
    static constexpr uint32_t uartLowSpeed{115200};
    static constexpr uint32_t uartHighSpeed{2000000};
    static constexpr uint8_t doneMask = 0x20;
    static constexpr uint8_t pdatMask = 0x04;
    static constexpr uint8_t rfftMask = 0x02;
    static constexpr uint8_t radcMask = 0x01;

    enum class Vld1ReplyStatus_t : uint8_t
    {
        OK = 0,
        VLD1_UNKONW_COMMAND,
        VLD1_INVALID_PARAMETER,
        VLD1_INVALID_RPST,
        VLD1_UART_ERROR,
        VLD1_NO_CAL,
        VLD1_TIMEOUT,
        LVD1_APPLICATION_CORRUPT,
        BAD_OPCODE,
        BAD_PAYLOAD_LENGTH,
        READ_TIMEOUT,
        NO_REPLY
    };

    enum class Vld1Command_t : uint8_t
    {
        INIT,
        GNFD,
        GRPS,
        SRPS,
        RFSE,
        GBYE,
        RRAI,
        THOF,
        MIRA,
        MARA,
        RAVG,
        TGFI,
        PREC,
        TXPW,
        INTN,
        SRDF,
        JBTL
    };

    enum class Vld1Reply_t : uint8_t
    {
        RESP = 1,
        VERS,
        RADC,
        RFFT,
        PDAT,
        DONE,
        RPST
    };

    struct __attribute__((__packed__)) PdatData_t
    {
        PdatDistance_t distance{0.0f};
        PdataMag_t magnitude{0};
    };

    enum class Vld1Baud_t : uint8_t
    {
        BAUD_115200 = 0,
        BAUD_460800 = 1,
        BAUD_921600 = 2,
        BAUD_2000000 = 3
    };

    enum class Vld1DistanceRange_t : uint8_t
    {
        RANGE_20 = 0,
        RANGE_50 = 1
    };

    enum class TargetFilter_t : uint8_t
    {
        Strongest = 0,
        Nearest = 1,
        Farthest = 2
    };

    enum class PrecisionMode_t : uint8_t
    {
        low = 0,
        high = 1,
    };

    enum class ShortRangeFilter_t : uint8_t
    {
        disabled = 0,
        enabled = 1,
    };

    struct __attribute__((__packed__)) RadarDataStructure_t
    {
        char fwVersion[19];
        char uniqueId[12];
        uint8_t distRange;
        uint8_t threshOffset;
        uint16_t minRangeFilter;
        uint16_t maxRangeFilter;
        uint8_t distAvgCount;
        uint8_t targetFilter;
        uint8_t distPrecision;
        uint8_t txPower;
        uint8_t chirpIntCount;
        uint8_t shortDistFilter;
    };

    struct Vld1Responce_t
    {
        Vld1ReplyStatus_t statusCode{Vld1ReplyStatus_t::NO_REPLY};
        Vld1Command_t commandSent;
        Vld1Baud_t baud{Vld1Baud_t::BAUD_115200};
        std::string version;
        std::unique_ptr<std::array<RadcArray_t, radcArraySize>> radcData;
        uint8_t gnfdPayloadByte{0};
        std::unique_ptr<std::array<RfftMagArray_t, rfftMagArrySize>> rfftMagData;
        std::unique_ptr<std::array<RfftThreshArray_t, rfftThreshArrySize>> rfftThreshData;
        PdatData_t pdataData;
        Done_t frameCoun{0};
        std::unique_ptr<RadarDataStructure_t> rpstData;
    };

    using Vld1ResponcePtr = std::unique_ptr<const Vld1Responce_t>;
    using PowerControlFunction = std::function<void(bool)>;

    // Constructor: pass uart port and whether to use RS485 pins (for dev board use UART pins)
    RFbeamVld1(uart_port_t uartPort, SerialChannel::Mode mode, PowerControlFunction pwrCtrl);
    ~RFbeamVld1();

    RFbeamVld1(const RFbeamVld1 &other) = delete;
    RFbeamVld1 &operator=(const RFbeamVld1 &other) = delete;

    // High level API
    Vld1ResponcePtr begin();
    Vld1ResponcePtr getRadarReply(bool waitForReply = true);
    Vld1ResponcePtr getRadarFrameData(uint8_t dataRequestMask = pdatMask, bool waitForReply = true);
    Vld1ResponcePtr getRadarRadarParameters(bool waitForReply = true);
    Vld1ResponcePtr setRadarRadarParameters(const RadarDataStructure_t &radarDataStructure, bool waitForReply = true);
    Vld1ResponcePtr disconnect(bool waitForReply = true);
    Vld1ResponcePtr setBaud(Vld1Baud_t newBaud, bool waitForReply = true);
    Vld1ResponcePtr setDistanceRange(Vld1DistanceRange_t range = Vld1DistanceRange_t::RANGE_20, bool waitForReply = true);
    Vld1ResponcePtr setThresholdOffset(uint8_t offset, bool waitForReply = true);
    Vld1ResponcePtr setMinRangeFilter(uint16_t range, bool waitForReply = true);
    Vld1ResponcePtr setMaxRangeFilter(uint16_t range, bool waitForReply = true);
    Vld1ResponcePtr setAvgCount(uint8_t count, bool waitForReply = true);
    Vld1ResponcePtr setTargetFilter(TargetFilter_t filter, bool waitForReply = true);
    Vld1ResponcePtr setPrecisionMode(PrecisionMode_t precision, bool waitForReply = true);
    Vld1ResponcePtr setTxPower(uint8_t power, bool waitForReply = true);
    Vld1ResponcePtr setInegrationCount(uint8_t count, bool waitForReply = true);
    Vld1ResponcePtr setShortRangeFilter(ShortRangeFilter_t filter, bool waitForReply = true);

private:
    struct __attribute__((__packed__)) MessageHeader_t
    {
        char opCode[4];
        PayloadSize_t payloadLength;
    };

    using TaskFunction = std::function<void(void)>;
    struct CommandRequest_t
    {
        Vld1Command_t command;
        const uint8_t *payloadPtr;
    };

    struct CommandInfo_t
    {
        static constexpr uint8_t maxReplies{5};
        char opcode[5];
        PayloadSize_t payloadLength{0};
        std::array<Vld1Reply_t, maxReplies> replyCodes{};
        CommandInfo_t(const char (&op)[5], PayloadSize_t pl, const std::array<Vld1Reply_t, maxReplies> &codes);
        CommandInfo_t() = delete;
    };

    struct replyInfo_t
    {
        char messageName[5];
        PayloadSize_t payloadLength = 0;
        replyInfo_t(const char (&replyOpcode)[5], PayloadSize_t pl);
        replyInfo_t() = delete;
    };

    // static maps (defined in cpp)
    static const std::map<const Vld1Command_t, const CommandInfo_t> commandMap;
    static const std::map<const Vld1Reply_t, const replyInfo_t> replyMap;
    static const std::map<const Vld1Baud_t, const uint32_t> baudMap;

    // members
    SerialChannel channel;
    Vld1Baud_t currentBaud = static_cast<Vld1Baud_t>(255);
    QueueHandle_t pendingCommandQueue{nullptr};
    const size_t pendingCommandQueueSize{10};
    QueueHandle_t responceQueue{nullptr};
    const size_t responceQueueSize{10};
    PowerControlFunction powerControl{nullptr};
    TaskHandle_t taskHandle{nullptr};

    // internal helpers
    void processCommands();
    void sendCommand(Vld1Command_t cmd, const void *dataPtr);
    static void taskLauncher(void *param);

    // low-level I/O wrappers to SerialChannel
    esp_err_t sendData(const void *data, size_t len);
    esp_err_t receiveData(void *data, size_t len, TickType_t timeout = pdMS_TO_TICKS(3000));
    void flush(uint32_t timeout_ms = 100);
    void updateBaud(uint32_t baud);
};
