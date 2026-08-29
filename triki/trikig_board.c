/**
 * trikig_board.c - LED/BTN/sense + SYSTEMOFF
 */
#include "trikig_board.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

void led_write(bool on)
{
#if LED_ACTIVE_LOW
    nrf_gpio_pin_write(PIN_LED, on ? 0 : 1);
#else
    nrf_gpio_pin_write(PIN_LED, on ? 1 : 0);
#endif
}

void led_blink(uint8_t n, uint16_t on_ms, uint16_t off_ms)
{
    for (uint8_t i = 0; i < n; i++) {
        led_write(true);  nrf_delay_ms(on_ms);
        led_write(false); nrf_delay_ms(off_ms);
    }
}

bool btn_pressed(void)
{
#if BTN_ACTIVE_LOW
    return nrf_gpio_pin_read(PIN_BTN) == 0;
#else
    return nrf_gpio_pin_read(PIN_BTN) != 0;
#endif
}

void enter_system_off(void)
{
    led_write(false);
#if BTN_ACTIVE_LOW
    nrf_gpio_cfg_sense_input(PIN_BTN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
#else
    nrf_gpio_cfg_sense_input(PIN_BTN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
#endif
    NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
    for (;;) { }
}
