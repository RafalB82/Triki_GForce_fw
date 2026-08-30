/**
 * trikig_bb_i2c.h - bit-bang I2C master (nRF52810, open-drain S0D1 + pullup)
 *
 * D-016 (stan sprzed C8): TWIM0 nie dzialal na tym sprzecie mimo poprawnej konfiguracji;
 * bit-bang ~250kHz (bb_delay 2us) zweryfikowany end-to-end. Od 0.3.1 (C8) TWIM wraca jako
 * path danych z fallbackiem bb — sprzecznosc rozstrzyga licznik g_diag.twim_faults na
 * sprzecie: faults=0 => D-016 nieaktualne; faults rosnie systematycznie => system zyje z bb
 * (ban TWIM po 3 faultach z rzedu, patrz trikig_lsm6dsl.c). Do potwierdzenia przy flashu.
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
