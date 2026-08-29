/**
 * trikig_bb_i2c.c - bit-bang I2C (produkcja; TWIM zarzucony, patrz D-016)
 */
#include "trikig_bb_i2c.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

static void bb_delay(void) { nrf_delay_us(2); }   /* ~250kHz */

static void bb_low(uint32_t pin)  { nrf_gpio_pin_clear(pin); bb_delay(); }
static void bb_high(uint32_t pin) { nrf_gpio_pin_set(pin);  bb_delay(); }

void bb_i2c_init(void)
{
    nrf_gpio_cfg(BB_I2C_SDA_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(BB_I2C_SCL_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
    bb_high(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SCL_PIN);
}

void bb_i2c_bus_clear(void)
{
    bb_high(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SCL_PIN);
    for (int i = 0; i < 9; i++) {
        bb_low(BB_I2C_SCL_PIN);
        bb_high(BB_I2C_SCL_PIN);
    }
    bb_low(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SDA_PIN);   /* STOP */
}

void bb_start(void)
{
    bb_high(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SCL_PIN);
    bb_low(BB_I2C_SDA_PIN);
    bb_low(BB_I2C_SCL_PIN);
}

void bb_restart(void)
{
    bb_high(BB_I2C_SCL_PIN);
    bb_low(BB_I2C_SDA_PIN);
    bb_low(BB_I2C_SCL_PIN);
}

void bb_stop(void)
{
    bb_low(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SCL_PIN);
    bb_high(BB_I2C_SDA_PIN);
}

bool bb_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if (b & (1u << i)) bb_high(BB_I2C_SDA_PIN); else bb_low(BB_I2C_SDA_PIN);
        bb_high(BB_I2C_SCL_PIN);
        bb_low(BB_I2C_SCL_PIN);
    }
    bb_high(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SCL_PIN);
    bool ack = !nrf_gpio_pin_read(BB_I2C_SDA_PIN);
    bb_low(BB_I2C_SCL_PIN);
    return ack;
}

uint8_t bb_read_byte(bool ack)
{
    uint8_t v = 0;
    bb_high(BB_I2C_SDA_PIN);
    for (int i = 7; i >= 0; i--) {
        bb_high(BB_I2C_SCL_PIN);
        v = (uint8_t)((v << 1) | (nrf_gpio_pin_read(BB_I2C_SDA_PIN) ? 1u : 0u));
        bb_low(BB_I2C_SCL_PIN);
    }
    if (ack) bb_low(BB_I2C_SDA_PIN); else bb_high(BB_I2C_SDA_PIN);
    bb_high(BB_I2C_SCL_PIN);
    bb_low(BB_I2C_SCL_PIN);
    bb_high(BB_I2C_SDA_PIN);
    return v;
}
