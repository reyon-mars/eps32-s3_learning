#include "vld1_application.hpp"

static constexpr char TAG[] = "Application";

void Application::start()
{
    xTaskCreate(uart_read_task, "uart_read_task", 4096, &ctx_, 10, nullptr);
}

void Application::uart_read_task(void *arg)
{
    AppContext *ctx = static_cast<AppContext *>(arg);

    uint8_t buffer[512];
    int buf_len = 0;

    while (true)
    {
        int read_bytes = ctx->vld_uart->read(buffer + buf_len, sizeof(buffer) - buf_len, pdMS_TO_TICKS(100));
        if (read_bytes > 0)
        {
            buf_len += read_bytes;
            int parsed_len = 0;
            do
            {
                char header[5] = {0};
                uint8_t payload[128] = {0};
                uint32_t payload_len = 0;

                parsed_len = ctx->vld->parse_message(buffer, buf_len, header, payload, &payload_len);
                if (parsed_len > 0)
                {
                    if (std::strcmp(header, "PDAT") == 0 && payload_len >= sizeof(vld1::pdat_resp_t))
                    {
                        auto *pdat_data = reinterpret_cast<vld1::pdat_resp_t *>(payload);
                        float distance_m = pdat_data->distance;
                        uint16_t distance_mm = static_cast<uint16_t>(distance_m * 1000.0f);

                        ctx->averager->add_sample(distance_m);
                        uint16_t avg_mm = ctx->averager->average_millimeters();

                        uint16_t regs[3] = {distance_mm, pdat_data->magnitude, avg_mm};
                        ctx->rs485_slave->write(regs, 3);

                        ESP_LOGI(TAG, "\tDistance (mm): %d \n\tMagnitude (db): %d ", distance_mm, pdat_data->magnitude);

                        if (ctx->averager->is_complete())
                        {
                            ESP_LOGI(TAG, "Batch complete: avg=%.3f m (%u mm)", ctx->averager->average_meters(), avg_mm);
                            ctx->averager->reset();
                        }
                    }
                    else if (std::strcmp(header, "RESP") == 0)
                    {
                        ESP_LOGI(TAG, "VLD1 RESP received");
                    }
                    else if (std::strcmp(header, "VERS") == 0)
                    {
                        char fw[65] = {0};
                        int copy_len = (payload_len < 64) ? static_cast<int>(payload_len) : 64;
                        std::memcpy(fw, payload, copy_len);
                        ESP_LOGI(TAG, "VLD1 Firmware: %s", fw);
                    }

                    std::memmove(buffer, buffer + parsed_len, buf_len - parsed_len);
                    buf_len -= parsed_len;
                }
            } while (parsed_len > 0 && buf_len > 0);
        }
    }
}
