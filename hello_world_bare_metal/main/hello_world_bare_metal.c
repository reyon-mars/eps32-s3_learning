#include <stdint.h>
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_intr_alloc.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "soc/soc.h"

#define BAUD_RATE 9600

static const char hello[] = "Hello World!\r\n";
static volatile int tx_index = 0;

void IRAM_ATTR uart0_isr(void *arg)
{
    uint32_t st = UART0.int_st.val;
    if (st & UART_TXFIFO_EMPTY_INT_ST_M) {
        if (hello[tx_index] != '\0') {
            UART0.fifo.rw_byte = (uint32_t)hello[tx_index++];
        } else {
            tx_index = 0;
            UART0.int_ena.txfifo_empty_int_ena = 0;
        }
        UART0.int_clr.txfifo_empty_int_clr = 1;
    }
}

static void uart0_init(void)
{
    // Baud‐rate divisor
    uint32_t clk_div = APB_CLK_FREQ / BAUD_RATE;
    UART0.clkdiv.clkdiv = clk_div;

    // Frame: 8 bits, 1 stop bit, no parity
    UART0.conf0.bit_num       = 3;  // 0=5bits,1=6bits,2=7bits,3=8bits
    UART0.conf0.stop_bit_num  = 1;  // 1 stop bit
    UART0.conf0.parity_en     = 0;

    // Reset FIFOs
    UART0.conf0.rxfifo_rst = 1; UART0.conf0.rxfifo_rst = 0;
    UART0.conf0.txfifo_rst = 1; UART0.conf0.txfifo_rst = 0;

    // Enable TX‑FIFO empty interrupt
    UART0.int_ena.val = 0;
    UART0.int_ena.txfifo_empty_int_ena = 1;
    UART0.int_clr.val = 0xFFFFFFFF;

    // Attach ISR
    esp_intr_alloc(ETS_UART0_INTR_SOURCE, 0, uart0_isr, NULL, NULL);

    // Start first char
    UART0.fifo.rw_byte = (uint32_t)hello[tx_index++];
}

void app_main(void)
{
    uart0_init();
    while (1) {
        // wait for printing one full message
        while (UART0.int_ena.txfifo_empty_int_ena != 0) {
            // busy‑wait
        }
        esp_rom_delay_us(500000);
        UART0.int_ena.txfifo_empty_int_ena = 1;
    }
}
