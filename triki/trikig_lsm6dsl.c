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

    /* kolejnosc wg datasheet: reset -> CTRL3 -> FIFO off -> CTRL1/2 (start ODR).
     * K6 (plan 027): bledy I2C configu EGZEKWOWANE (fail-fast, nie fail-silent). */
    bool ok = reg_write(LSM_CTRL3_C, 0x01);          /* SW reset */

    uint16_t wait_iters = 0;
    do {
        if (!reg_read(LSM_CTRL3_C, &who, 1)) return false;
        if (++wait_iters > LSM_RESET_MAX_ITERS) {    /* K7: timeout zamiast wiecznej petli */
            rtt_diag_printf("S2 CTRL3 reset TIMEOUT");
            return false;
        }
        nrf_delay_us(100);
    } while (who & 0x01);

    ok = reg_write(LSM_CTRL3_C, LSM_CTRL3_C_CFG) && ok;   /* BDU + IF_INC */
    ok = reg_write(0x0A, 0x00)                    && ok;   /* FIFO_CTRL5 bypass (poll) */
    ok = reg_write(LSM_CTRL1_XL, LSM_CTRL1_XL_V19) && ok;
    ok = reg_write(LSM_CTRL2_G,  LSM_CTRL2_G_V19)  && ok;

    /* readback + wyegzekwowanie (D-017): config musi siedziec, nie tylko byc zalogowany */
    uint8_t rc1 = 0, rc2 = 0, rc3 = 0;
    ok = reg_read(LSM_CTRL1_XL, &rc1, 1) && ok;
    ok = reg_read(LSM_CTRL2_G,  &rc2, 1) && ok;
    ok = reg_read(LSM_CTRL3_C,  &rc3, 1) && ok;   /* audyt 2026-08-30: BDU w readback (wczesniej CTRL3 bez kontroli) */
    rtt_diag_printf("FW=" TRIKIG_FW_TAG " c1=%02x c2=%02x c3=%02x", rc1, rc2, rc3);
    if (!ok || rc1 != LSM_CTRL1_XL_V19 || rc2 != LSM_CTRL2_G_V19 || rc3 != LSM_CTRL3_C_CFG) {
        rtt_diag_printf("S2 config MISMATCH ok=%d rc1=%02x rc2=%02x rc3=%02x", ok, rc1, rc2, rc3);
        return false;
    }
    return true;
}

bool lsm6dsl_read_motion(uint8_t *dst12)
{
    return reg_read(LSM_OUTX_L_G, dst12, 12);
}
