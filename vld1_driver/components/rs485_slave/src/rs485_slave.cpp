#include "rs485_slave.hpp"
#include "esp_log.h"

static constexpr char TAG[] = "RS485";

rs485::rs485(uart &uart_no, gpio_num_t de_re_pin) noexcept
    : uart_(uart_no), de_re_pin_(de_re_pin),
      mbc_slave_handle_(nullptr),
      slave_address_(1),
      input_registers_(nullptr),
      input_reg_size_(0) {}

rs485::~rs485() noexcept
{
    if (mbc_slave_handle_)
    {
        mbc_slave_destroy();
        mbc_slave_handle_ = nullptr;
    }
    free_registers();
}

void rs485::free_registers() noexcept
{
    delete[] input_registers_;
    input_registers_ = nullptr;
}

esp_err_t rs485::configure_pins() noexcept
{
    gpio_config_t io_conf{};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << static_cast<uint32_t>(de_re_pin_));
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure DE/RE pin: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(de_re_pin_, 0);
    return ESP_OK;
}

esp_err_t rs485::init(uint8_t slave_addr, mb_param_type_t reg_type, size_t input_reg_count) noexcept
{
    slave_address_ = slave_addr;
    input_reg_size_ = input_reg_count;

    esp_err_t err = configure_pins();
    if (err != ESP_OK)
        return err;

    mb_communication_info_t comm_config{};
    comm_config.mode = MB_MODE_RTU;
    comm_config.slave_addr = slave_address_;
    comm_config.port = uart_.port();
    comm_config.baudrate = uart_.baud_rate();
    comm_config.parity = uart_.parity();

    err = mbc_slave_setup(&comm_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create Modbus slave: %s", esp_err_to_name(err));
        return err;
    }

    free_registers();
    input_registers_ = new uint16_t[input_reg_size_]();

    mb_register_area_descriptor_t reg_area{};
    reg_area.start_offset = 0x00;
    reg_area.type = reg_type;
    reg_area.address = static_cast<void *>(input_registers_);
    reg_area.size = sizeof(uint16_t) * input_reg_size_;

    err = mbc_slave_set_descriptor(reg_area);
    {
        ESP_LOGE(TAG, "Failed to set Modbus input register descriptor: %s", esp_err_to_name(err));
        return err;
    }

    err = mbc_slave_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Modbus slave: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "RS485 Modbus slave initialized (Addr=%u, InputRegs=%zu)", slave_address_, input_reg_size_);
    return ESP_OK;
}

esp_err_t rs485::write(const uint16_t *data, size_t count) noexcept
{
    if (!mbc_slave_handle_ || !input_registers_)
    {
        ESP_LOGE(TAG, "Slave not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (count > input_reg_size_)
    {
        ESP_LOGE(TAG, "Register write out of bounds: start=0x00, count=%zu", count);
        return ESP_ERR_INVALID_ARG;
    }

    if (data)
    {
        std::memcpy(input_registers_, data, count * sizeof(uint16_t));
        ESP_LOGI(TAG, "Wrote %zu input registers starting at address 0", count);
    }
    else
    {
        std::fill(input_registers_, input_registers_ + input_reg_size_, static_cast<uint16_t>(0));
        ESP_LOGW(TAG, "No data received. Filled %zu registers with error code 0", count);
    }
    
    return ESP_OK;
}
