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

/* v0.4.1: runtime reg access — TWIM-aware. Root-cause smoke v0.4.0 [P]: reg_write/
 * reg_read (bb) po starcie TWIM NIE dzialaja — nrfx_twim_enable przejmuje piny
 * 5/6, zapisy GPIO nie steruja magistrali => inact cfg=00 MISMATCH (log RTT
 * 2026-09-01). W 0.3.x to bylo latentne (po probe nikt nie dotykal rejestrow).
 * Wzorzec jak lsm6dsl_read_motion: TWIM jak zyje, fault => uninit (piny -> GPIO)
 * i fallback bb; ban po 3 faultach. Boot-init zostaje na czystym bb (przed TWIM,
 * dla stuck-bus recovery — C8). */
static bool reg_write_t(uint8_t reg, uint8_t val)
{
    if (lsm6dsl_twim_init()) {
        uint8_t buf[2] = { reg, val };
        nrfx_err_t e = nrfx_twim_tx(&m_twi, LSM_ADDR, buf, 2, false);
        if (e == NRFX_SUCCESS) {
            m_twim_consecutive = 0;
            return true;
        }
        if (g_diag.twim_faults < 0xFFFFFFFFu) g_diag.twim_faults++;
        nrfx_twim_uninit(&m_twi);            /* piny 5/6 -> GPIO dla bb fallback */
        m_twim_ready = false;
        if (++m_twim_consecutive >= LSM_TWIM_BAN_FAULTS) {
            m_twim_banned = true;
            rtt_diag_printf("S2 TWIM banned after %u faults", g_diag.twim_faults);
        }
    }
    return reg_write(reg, val);
}

static bool reg_read_t(uint8_t reg, uint8_t *dst, uint8_t len)
{
    if (lsm6dsl_twim_init()) {
        nrfx_err_t e = nrfx_twim_tx(&m_twi, LSM_ADDR, &reg, 1, false);
        if (e == NRFX_SUCCESS) e = nrfx_twim_rx(&m_twi, LSM_ADDR, dst, len);
        if (e == NRFX_SUCCESS) {
            m_twim_consecutive = 0;
            return true;
        }
        if (g_diag.twim_faults < 0xFFFFFFFFu) g_diag.twim_faults++;
        nrfx_twim_uninit(&m_twi);
        m_twim_ready = false;
        if (++m_twim_consecutive >= LSM_TWIM_BAN_FAULTS) {
            m_twim_banned = true;
            rtt_diag_printf("S2 TWIM banned after %u faults", g_diag.twim_faults);
        }
    }
    return reg_read(reg, dst, len);
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
    /* 0.3.5: DRDY ukladu wychodzi na INT2 (pin 9 ukladu wg datasheet ukladu) =>
     * INT2_CTRL (0x0E) bit0 = INT2_DRDY_XL; zapis walidowany readbackiem (D-017).
     * Wczesniejsze INT1_CTRL (0x0D) nie dawalo krawedzi — INT1 ukladu niepolaczony. */
    if (!reg_write(LSM_INT2_CTRL, on ? 0x01u : 0x00u)) return false;
    uint8_t rb = 0;
    if (!reg_read(LSM_INT2_CTRL, &rb, 1)) return false;
    if (rb != (on ? 0x01u : 0x00u)) {
        rtt_diag_printf("S2 INT2_CTRL rb=%02x MISMATCH", rb);
        return false;
    }
    return true;
}

bool lsm6dsl_drdy1_enable(bool on)
{
    /* INT1_CTRL (0x0D) bit0 = INT1_DRDY_XL (0.3.10: skan sprawdza oba piny INT ukladu) */
    if (!reg_write(LSM_INT1_CTRL, on ? 0x01u : 0x00u)) return false;
    uint8_t rb = 0;
    if (!reg_read(LSM_INT1_CTRL, &rb, 1)) return false;
    if (rb != (on ? 0x01u : 0x00u)) {
        rtt_diag_printf("S2 INT1_CTRL rb=%02x MISMATCH", rb);
        return false;
    }
    return true;
}

bool lsm6dsl_inactivity_enable(bool on)
{
    /* v0.4.0 IDLE-CONNECTED (plan pkt 1): HW activity/inactivity — INACT_EN=11
     * => po SLEEP_DUR bezruchu HW SAM przechodzi w low-power (acc 12.5Hz LP,
     * gyro power-down), activity => auto-restore. FW nie przelacza CTRL1/CTRL2
     * recznie (miedzy innymi dlatego, ze reczne 0x10/0x40 z pierwotnej propozycji
     * gubilo FS=16g => 8x zmiana czulosci vs kontrakt wire 2048 LSB/g).
     * LIR=1: poziom trzymamy do odczytu WAKE_UP_SRC (sync w lsm6dsl_inact_state).
     * v0.4.1: TWIM-aware (wywolywane po probe — bb na przejetych pinach = MISMATCH). */
    uint8_t tap_cfg = on ? LSM_TAP_CFG_INACT : 0x00u;
    uint8_t wk_ths  = on ? LSM_WK_THS : 0x00u;
    uint8_t wk_dur  = on ? (uint8_t)LSM_SLEEP_DUR_S : 0x00u;
    uint8_t md1     = on ? LSM_MD1_SLEEP_CHG : 0x00u;

    bool ok = reg_write_t(LSM_TAP_CFG, tap_cfg);
    ok = reg_write_t(LSM_WAKE_UP_THS, wk_ths) && ok;
    ok = reg_write_t(LSM_WAKE_UP_DUR, wk_dur) && ok;
    ok = reg_write_t(LSM_MD1_CFG, md1) && ok;

    uint8_t r_tap = 0, r_ths = 0, r_dur = 0, r_md1 = 0;
    ok = reg_read_t(LSM_TAP_CFG, &r_tap, 1) && ok;
    ok = reg_read_t(LSM_WAKE_UP_THS, &r_ths, 1) && ok;
    ok = reg_read_t(LSM_WAKE_UP_DUR, &r_dur, 1) && ok;
    ok = reg_read_t(LSM_MD1_CFG, &r_md1, 1) && ok;
    rtt_diag_printf("S2 inact cfg=%02x ths=%02x dur=%02x md1=%02x", r_tap, r_ths, r_dur, r_md1);
    if (!ok || r_tap != tap_cfg || r_ths != wk_ths || r_dur != wk_dur || r_md1 != md1) {
        rtt_diag_printf("S2 inact MISMATCH ok=%d tap=%02x ths=%02x dur=%02x md1=%02x",
                        ok, r_tap, r_ths, r_dur, r_md1);
        return false;
    }
    return true;
}

bool lsm6dsl_inact_state(bool *sleeping)
{
    /* Readback stanu z HW (D-017): SLEEP_STATE_IA=1 => uklad w inactivity (low-power).
     * Odczyt WAKE_UP_SRC kasuje LIR => kolejna zmiana stanu wygeneruje nowy poziom
     * na INT1 (GPIOTE TOGGLE/LOTOHI zlapie krawedz).
     * v0.4.1: TWIM-aware + wywolywane z petli glownej (NIE ze SWI — I2C w kontekscie
     * SoftDevice IRQ zakazane; sync przy CONNECTED przeniesiony do main loop). */
    uint8_t src = 0;
    if (!reg_read_t(LSM_WAKE_UP_SRC, &src, 1)) return false;
    *sleeping = ((src & 0x10u) != 0u);
    return true;
}

bool lsm6dsl_wake_force(void)
{
    /* v0.5.0 training mode: jawne wyjscie z inactivity. DS6207 6.5.2 opisuje
     * auto-restore przy activity evencie, NIE przy INACT_EN=00 — po wylaczeniu
     * funkcji HW moze pozostac w low-power (acc 12.5Hz LP + gyro power-down).
     * Wymuszamy exit przepisaniem kanonicznych CTRL1_XL/CTRL2_G (V19) z
     * readbackiem (D-017): zmiana ODR = gwarantowany exit z inactivity.
     * Bez tego gyro moglby zostac w power-down mimo zywego acc (klasa slowrot). */
    uint8_t c1 = 0, c2 = 0;
    bool ok = reg_write_t(LSM_CTRL1_XL, LSM_CTRL1_XL_V19);
    ok = reg_write_t(LSM_CTRL2_G, LSM_CTRL2_G_V19) && ok;
    ok = reg_read_t(LSM_CTRL1_XL, &c1, 1) && ok;
    ok = reg_read_t(LSM_CTRL2_G, &c2, 1) && ok;
    rtt_diag_printf("S6 train wake c1=%02x c2=%02x", c1, c2);
    return ok && c1 == LSM_CTRL1_XL_V19 && c2 == LSM_CTRL2_G_V19;
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
