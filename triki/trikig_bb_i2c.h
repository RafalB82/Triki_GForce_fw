/**
 * trikig_bb_i2c.h - bit-bang I2C master (nRF52810, open-drain S0D1 + pullup)
 *
 * D-016: TWIM0 nie dziala na tym sprzedzie mimo poprawnej konfiguracji;
 * bit-bang ~250kHz (bb_delay 2us) zweryfikowany na sprzecie end-to-end.
 */
#ifndef TRIKIG_BB_I2C_H
#define TRIKIG_BB_I2C_H

#include <stdint.h>
#include <stdbool.h>

#define BB_I2C_SDA_PIN   5u
#define BB_I2C_SCL_PIN   6u

void     bb_i2c_init(void);
void     bb_i2c_bus_clear(void);
void     bb_start(void);
void     bb_restart(void);
void     bb_stop(void);
bool     bb_write_byte(uint8_t b);
uint8_t  bb_read_byte(bool ack);

#endif /* TRIKIG_BB_I2C_H */
