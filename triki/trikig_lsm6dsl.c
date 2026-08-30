/**
 * trikig_lsm6dsl.c - init + odczyt OUT (C7/C8: DRDY enable, TWIM 400kHz z fallbackiem bb)
 */
#include "trikig_lsm6dsl.h"
#include "trikig_bb_i2c.h"
#include "trikig_board.h"
#include "trikig_diag.h"
#include "trikig_version.h"
#include "nrf_delay.h"
#include "nrfx_twim.h"
#include "SEGGER_RTT.h"

/* C8: TWIM0 @400kHz — path danych (burst 12B ~40us vs bb ~300us, ~0 CPU / DMA).
 * Hybryda: init/bus-clear zostaja bit-bang (stuck-bus recovery), TWIM tylko odczyt;
 * fault TWIM => uninit (piny -> GPIO), odczyt bb, re-init lazy przy nastepnej probce.
 * Audyt A 2026-08-30 (D-016 vs C8): gdy TWIM realnie nie dziala na plycie (D-016),
 * re-init co probe = czysty koszt => ban po 3 faultach z rzedu (bb do resetu,
 * jednorazowy koszt, licznik twim_faults rozstrzyga na sprzecie). */
#define LSM_TWIM_BAN_FAULTS 3u
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);
static bool m_twim_ready = false;
static bool m_twim_banned = false;              /* true: bb do resetu (D-016 real) */
static uint8_t m_twim_consecutive = 0;

static bool lsm6dsl_twim_init(void)
{
    if (m_twim_banned) return false;
    if (m_twim_ready) return true;
    nrfx_twim_config_t cfg = NRFX_TWIM_DEFAULT_CONFIG;
    cfg.scl       = 6u;                       /* P0.06 SCL (SPEC 1) */
    cfg.sda       = 5u;                       /* P0.05 SDA */
    cfg.frequency = NRF_TWIM_FREQ_400K;
    if (nrfx_twim_init(&m_twi, &cfg, NULL, NULL) != NRFX_SUCCESS) return false;  /* NULL = blocking */
    nrfx_twim_enable(&m_twi);
    m_twim_ready = true;
    return true;
}


static bool reg_write(uint8_t reg, uint8_t val)
{
    bb_start();
    /* audyt#3 2026-08-30: abort przy pierwszym NACK (spojnie z reg_read) — zero ruchu
     * na magistrali po bledzie adresu/rejestru. */
    if (!bb_write_byte(LSM_ADDR_W)) { bb_stop(); return false; }
    if (!bb_write_byte(reg))        { bb_stop(); return false; }
    if (!bb_write_byte(val))        { bb_stop(); return false; }
    bb_stop();
    return true;
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

bool lsm6dsl_drdy_enable(bool on)
{
    /* INT1_CTRL (0x0D) bit0 = INT1_DRDY_XL (datasheet); zapis walidowany readbackiem (D-017) */
    if (!reg_write(LSM_INT1_CTRL, on ? 0x01u : 0x00u)) return false;
    uint8_t rb = 0;
    if (!reg_read(LSM_INT1_CTRL, &rb, 1)) return false;
    if (rb != (on ? 0x01u : 0x00u)) {
        rtt_diag_printf("S2 INT1_CTRL rb=%02x MISMATCH", rb);
        return false;
    }
    return true;
}

bool lsm6dsl_read_motion(uint8_t *dst12)
{
    if (lsm6dsl_twim_init()) {
        uint8_t reg = LSM_OUTX_L_G;
        /* STOP po TX; LSM6DSL pamieta adres rejestru (IF_INC), RX od nowego START */
        nrfx_err_t e = nrfx_twim_tx(&m_twi, LSM_ADDR, &reg, 1, false);
        if (e == NRFX_SUCCESS) e = nrfx_twim_rx(&m_twi, LSM_ADDR, dst12, 12);
        if (e == NRFX_SUCCESS) {
            m_twim_consecutive = 0;
            return true;
        }
        if (g_diag.twim_faults < 0xFFFFFFFFu) g_diag.twim_faults++;
        nrfx_twim_uninit(&m_twi);            /* piny 5/6 -> GPIO dla bb fallback */
        m_twim_ready = false;
        if (++m_twim_consecutive >= LSM_TWIM_BAN_FAULTS) {
            m_twim_banned = true;            /* D-016 real? bb do resetu (audyt A) */
            rtt_diag_printf("S2 TWIM banned after %u faults", g_diag.twim_faults);
        }
    }
    /* fallback: bit-bang (sprawdzona sciezka sprzed C8) */
    bb_i2c_bus_clear();
    return reg_read(LSM_OUTX_L_G, dst12, 12);
}
