/**
 * trikig_bb_i2c.h - bit-bang I2C master (nRF52810, open-drain S0D1 + pullup)
 *
 * D-016 — ZAMKNIETE (walidacja HW 0.3.4): twi=0 po ~500 odczytach TWIM na plycie => D-016
 * nieaktualne, TWIM0 DZIALA (ban po 3 faultach zostaje jako net). BB zostaje dla init/
 * bus-clear/fault-recovery.
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
