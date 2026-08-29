/**
 * trikig_lsm6dsl.c - init + polling OUT (strategia v19, FIFO OFF)
 */
#include "trikig_lsm6dsl.h"
#include "trikig_bb_i2c.h"
#include "trikig_board.h"
#include "trikig_version.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"


static bool reg_write(uint8_t reg, uint8_t val)
{
    bb_start();
    bool ok = bb_write_byte(LSM_ADDR_W);
    ok = bb_write_byte(reg) && ok;
    ok = bb_write_byte(val) && ok;
    bb_stop();
    return ok;
}

static bool reg_read(uint8_t reg, uint8_t *dst, uint8_t len)
{
    bb_start();
    if (!bb_write_byte(LSM_ADDR_W)) { bb_stop(); return false; }
    if (!bb_write_byte(reg))        { bb_stop(); return false; }
    bb_restart();
    if (!bb_write_byte(LSM_ADDR_R)) { bb_stop(); return false; }
    for (uint8_t i = 0; i < len; i++)
        dst[i] = bb_read_byte(i < (uint8_t)(len - 1));
    bb_stop();
    return true;
}

bool lsm6dsl_init(void)
{
    uint8_t who = 0;

    /* WHO_AM_I z retry (odpornosc na stuck-bus po poprzednim fw / wolny start LDO) */
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        bb_i2c_bus_clear();
        if (reg_read(LSM_WHO_AM_I, &who, 1) && who == LSM_WHO_AM_I_VAL) break;
        who = 0;
        nrf_delay_ms(10);
    }
    if (who != LSM_WHO_AM_I_VAL) return false;

    /* kolejnosc wg datasheet: reset -> CTRL3 -> FIFO off -> CTRL1/2 (start ODR) */
    (void)reg_write(LSM_CTRL3_C, 0x01);              /* SW reset */
    do {
        if (!reg_read(LSM_CTRL3_C, &who, 1)) return false;
    } while (who & 0x01);
    (void)reg_write(LSM_CTRL3_C, LSM_CTRL3_C_CFG);   /* BDU + IF_INC */

    (void)reg_write(0x0A, 0x00);                     /* FIFO_CTRL5 = bypass (poll strategy) */

    (void)reg_write(LSM_CTRL1_XL, LSM_CTRL1_XL_V19);
    (void)reg_write(LSM_CTRL2_G,  LSM_CTRL2_G_V19);

    /* readback: co faktycznie siedzi na sprzecie (D-017) */
    {
        uint8_t rc1 = 0, rc2 = 0;
        (void)reg_read(LSM_CTRL1_XL, &rc1, 1);
        (void)reg_read(LSM_CTRL2_G,  &rc2, 1);
        rtt_diag_printf("FW=" TRIKIG_FW_TAG " c1=%02x c2=%02x", rc1, rc2);
    }
    return true;
}

bool lsm6dsl_read_motion(uint8_t *dst12)
{
    return reg_read(LSM_OUTX_L_G, dst12, 12);
}
