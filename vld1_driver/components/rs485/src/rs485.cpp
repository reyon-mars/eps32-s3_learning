#include "rs485.hpp"
#include "esp_log.h"

static constexpr char TAG[] = "Rs485";

rs485::rs485(uart &uart_no, gpio_num_t de_re_pin) noexcept
    : uart_(uart_no), de_re_pin_(de_re_pin),
      mbc_slave_handle_(nullptr),
      slave_address_(1),
      input_registers_(nullptr),
      input_count_(0) {}

rs485::~rs485() noexcept
{
    if (mbc_slave_handle_)
    {
        mbc_slave_stop(mbc_slave_handle_);
        mbc_slave_delete(mbc_slave_handle_);
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
    gpio_set_level(de_re_pin_, 0); // default receive mode
    return ESP_OK;
}

esp_err_t rs485::init(uint8_t slave_addr, size_t input_reg_count) noexcept
{
    slave_address_ = slave_addr;
    input_count_ = input_reg_count;

    esp_err_t err = configure_pins();
    if (err != ESP_OK)
        return err;

    mb_communication_info_t comm_config{};
    comm_config.ser_opts.port = uart_.port();
    comm_config.ser_opts.baudrate = uart_.baud();
    comm_config.ser_opts.parity = MB_PARITY_NONE;
    comm_config.ser_opts.data_bits = UART_DATA_8_BITS;
    comm_config.ser_opts.stop_bits = UART_STOP_BITS_1;
    comm_config.ser_opts.mode = MB_MODE_RTU;
    comm_config.ser_opts.uid = slave_address_;

    // Create Modbus slave controller
    err = mbc_slave_create_serial(&comm_config, &mbc_slave_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create Modbus slave: %s", esp_err_to_name(err));
        return err;
    }

    // Allocate input registers
    free_registers();
    input_registers_ = new uint16_t[input_count_]();

    // Register input registers area
    mb_register_area_descriptor_t reg_area{};
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = 0x00;
    reg_area.address = input_registers_;
    reg_area.size = sizeof(uint16_t) * input_count_;
    reg_area.access = MB_ACCESS_RW;

    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mbc_slave_handle_, reg_area));

    // Start Modbus slave
    err = mbc_slave_start(mbc_slave_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Modbus slave: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "RS485 Modbus slave initialized (Addr=%u, InputRegs=%zu)", slave_address_, input_count_);
    return ESP_OK;
}

esp_err_t rs485::write(const uint16_t *data, size_t count, uint16_t start_addr) noexcept
{
    if (!mbc_slave_handle_ || !input_registers_)
    {
        ESP_LOGE(TAG, "Slave not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (start_addr + count > input_count_)
    {
        ESP_LOGE(TAG, "Register write out of bounds: start=%u, count=%zu", start_addr, count);
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < count; ++i)
    {
        esp_err_t err = mbc_slave_set_value(mbc_slave_handle_, MB_PARAM_INPUT, start_addr + i, &data[i]);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write input register %zu: %s", i + start_addr, esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "Wrote %zu input registers starting at address %u", count, start_addr);
    return ESP_OK;
}
