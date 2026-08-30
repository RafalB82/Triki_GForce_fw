/**
 * trikig — nRF52810 + LSM6DSL, S112 7.2.0 (refaktor v19)
 *
 * Moduly: trikig_bb_i2c (I2C), trikig_lsm6dsl (IMU), trikig_board (LED/BTN/sleep/RTT),
 *          trikig_vbt (velocity side-band O-012 — wire 14B bez zmian).
 * Zachowane semantyki v19: poll 9ms -> ramka 14B -> ring -> NUS; skale CTRL1=0x44/CTRL2=0x4C.
 * v0.0.27 (audyt 2026-08-29): WDT 12s (kick w petli glownej, fault=>reset zamiast SOS-loop),
 * BLE send = jedna proba/drop (default:break — koniec nieskonczonego retry), static_assert
 * V_CLAMP vs int16_t, TODO btn_cnt, komentarz idle_secs.
 * v0.1.0 (plan 027 K1-K5): ring_slot_t struct, snapshot vel/flags przy poll (K2),
 * duplikat pomija VBT (K3), seq liczy KAZDA probke — luki informacyjne (K4), VBT
 * niezalezny od backpressure ringu (K5). Zmiana KONTRAKTU seq => bump MINOR (D-020).
 * v0.1.1 (audyt 2026-08-30): CTRL3_C 0x0C = BDU+IF_INC (0x44 ustawial H_LACTIVE zamiast
 * BDU) + readback c3; CP delay w APP_TIMER_TICKS (jednostki app_timer2); BTN debounc +
 * edge-detect (3 wcisniecia wg SPEC 6); clamp p w VBT isqrt (uint32 overflow); app_timer_init
 * przed BLE; g_info_req = licznik zadan 20 12; split GATTS/GATTC timeout; Makefile: target
 * release (TRIKIG_RTT_DIAG=0) + ostrzezenie SD-first przy flash.
 * v0.2.0 (F5): SAADC bateria CR2032 (trikig_batt, AIN2/P0.04, dzielnik przez diode);
 * probkowanie 1s w sleep tick; ramka statusowa 22 04 (RX 20 17) = bat_mV u16LE;
 * flags v2 bit3 = low-battery (< TRIKIG_BATT_LOW_MV).
 * v0.3.0 (plan VBT C1-C6): trikig_diag (liczniki dropow + okres probkowania + timingi
 * DWT acq/dsp/ble); VBT: gravity tracking ACC+GYRO (filtr komplementarny, propagacja
 * gyro + korekcja ACC gated |w|/innowacja), movement-axis (default X, vbt_set_axis),
 * dt = 1/ODR jako parametr; detektor rest 1. rzedu ||lin|| < 0.3; harness offline
 * tools/vbt_offline (5 scenariuszy PASS) — szczegoly bugow zlapanych w C6: trikig_vbt.c.
 * v0.3.1 (audyt VBT 2026-08-30, Faza 2): C7 DRDY (INT1_CTRL=DRDY_XL, GPIOTE auto-probe
 * polaryzacji z samowalidacja runtime, fallback polling + watchdog 30ms); probkowanie
 * przeniesione z SWI do main loop; dt = t[n]-t[n-1] z timestampow ISR (clamp 4-40ms,
 * dt_fault, gap>100ms => twardy ZUPT); memcmp = wylacznie diagnostyka w DRDY;
 * C8 TWIM 400kHz hybryda (bb init/bus-clear, fault -> bb fallback, licznik twim_faults);
 * C9 ring 16; profil DSP gravity/linear/velocity us (TRIKIG_VBT_PROFILE); isqrt 3->1.
 * v0.3.2 (audyt zewnetrzny 2026-08-30): A — ban TWIM po 3 faultach z rzedu (D-016 real?
 * nie bedzie re-init co probe; rozstrzyga twim_faults na sprzecie); F/G — SAADC faults
 * liczone w diag + guard sum==0 (OFFSET_MV != 0 nie wyglada jak odczyt); H — low-batt
 * snapshot przy probce (K2 spojnosc, nie OR przy wysylce); flags v2 bit4 = g-forced
 * (kalibracja wymuszona w ruchu => host oznacza serie jako niepewna); IMU_ODR_HZ zamiast
 * magic 104; CI: vbt_offline w GH Actions (.github/workflows/vbt-offline.yml).
 * v0.3.3 (walidacja HW baterii): pierwsza rzeczywista lektura — FW 6595mV @ skala 2/1 vs
 * real 3.3080V => AIN2/P0.04 = Vbat BEZ dzielnika 2x; SCALE 1/1; skala 2/1 byla zlego
 * punktu (P0.04 != P0.12; potwierdza tez WHO_AM_I=0x6A => SA0 wysoki). Dokladnosc ~0.3%.
 * v0.3.5: DRDY na INT2 (pin 9 ukladu wg datasheet ukladu — INT1 ukladu niepolaczony,
 * dlatego probe 0.3.3/0.3.4 nie widzial krawedzi); register INT2_CTRL (0x0E); probe
 * rozszerzona: pin P0.09/P0.10 x polaryzacja (drdy_mode 1-4).
 * v0.3.7 (log wire v2 21:42): nauka biasu gyro w quasi-bezruchu (||lin||<1.2 m/s^2,
 * |w-wbias|<15dps, tau 0.6s) — dryf propagacji z biasu podtrzymywal blad g na progu
 * gate'a innowacji => ZUPT nie gasil velocity przy ruchach (raczeta +250 mm/s/cykl,
 * vel stuck ~3000); propagacja i gate na gyro skorygowanym (w - wbias).
 * v0.3.6: LSM INT2/DRDY -> P0.10 potwierdzone plyta [P] — probe probuje P0.10 pierwsza.
 * v0.3.4 (log RTT 0.3.3): (1) DWT->CYCCNT NIE ISTNIEJE na nRF52810 — wszystkie timingi
 * diag=0 i watchdog DRDY martwy; timebase -> TIMER1 @1MHz 16-bit (wrap-safe uint16, okno
 * 65.5ms), detekcja gapow -> RTC1 (>60ms => twardy ZUPT). (2) rampa velocity do clampa
 * przy lezacym urzadzeniu: gyro bias ~3dps > stary gate |w|<2dps => korekcja trwale
 * zamknieta => gest rotuje z biasem (repro: harness scenario bias); gate 2->15 dps +
 * powolny leak korekcji 1/2048 zawsze. (3) bdrop=104/s => telemetria pierwszego bledu
 * ble send (8 = INVALID_STATE: klient bez subskrypcji CCCD).
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_nus.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_gpiote.h"

#include "trikig_board.h"
#include "trikig_diag.h"
#include "trikig_vbt.h"
#include "trikig_batt.h"
#include "trikig_version.h"
#include "trikig_bb_i2c.h"
#include "trikig_lsm6dsl.h"

#define APP_BLE_CONN_CFG_TAG    1

#define DEVICE_NAME             "Triki GForce"

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(15, UNIT_1_25_MS)
#define SLAVE_LATENCY           0
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)
#define APP_ADV_INTERVAL        MSEC_TO_UNITS(40, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT         0
#define FIRST_CP_DELAY          APP_TIMER_TICKS(5000)   /* 5s — audyt 2026-08-30: wczesniej surowe ticki v1 (5*32768) = zle jednostki pod app_timer2 */
#define NEXT_CP_DELAY           APP_TIMER_TICKS(5000)
#define MAX_CP_COUNT            3

/* ---- ramka wire: 14B = [0x22,0x00 | gyro(6) | acc(6)] i16LE, gyro FIRST (PWA layout, nie zmieniac) ---- */
#define FRAME_SIZE              14u
#define FRAME_V2_SIZE           19u         /* v2: 22 01 | seq16 | gyro6+acc6 | vel16 | flags8 */
#define RING_SLOTS              16u         /* C9: 4 -> 16 (~300B bss; margines BLE z ~29ms do ~145ms) */
#define POLL_INTERVAL_MS        9u          /* 9ms: teraz tylko BTN + watchdog DRDY (probki z DRDY) */
#define DRDY_TS_SLOTS           4u          /* FIFO timestampow ISR -> main */
#define DT_MIN_US               4000u       /* clamp dt (P3): < 250 Hz nie zdarzy sie przy 104 ODR */
#define DT_MAX_US               40000u      /* > 25 Hz = probka zgubiona */
#define DT_GAP_TICKS            1966u       /* gap > 60ms (1966x30.5us RTC1) => twardy ZUPT;
                                             * 60ms zamiast 100ms: timebase TIMER1 16-bit wrap 65.5ms */
#define IMU_ODR_HZ              104u        /* dzielnik logu diag ~1s; zmiana ODR => popraw tu (F2) */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static volatile bool m_stream_on = true;
static bool m_imu_ok = false;

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

/* ---- ring 4 x slot (SPSC: timer pisze, main petla czyta) ----
 * K1 (plan 027): struct zamiast rownoleglych tablic; snapshot vel/flags przy poll (K2).
 * K4: m_sample_seq liczy KAZDA probke IMU przy aktywnym polaczeniu — luki w seq sa
 *     oczekiwane i informacyjne (drop przy pelnym ringu), patrz SPEC 5.1. */
typedef struct {
    uint8_t  raw[12];      /* gyro6+acc6 i16LE, z LSM OUTX_L_G */
    uint16_t seq;
    int16_t  vel_mms;      /* snapshot VBT w chwili poll */
    uint8_t  flags;        /* snapshot VBT w chwili poll */
} ring_slot_t;

static ring_slot_t      ring[RING_SLOTS];
static volatile uint8_t ring_wr = 0, ring_rd = 0;
static volatile uint8_t  m_wire_mode = 1;    /* 1 = legacy 14B (default), 2 = v2 19B (komenda 20 11 01) */
static uint16_t          m_sample_seq = 0;
static uint8_t           s_prev_raw[12] = {0};   /* K3: detekcja duplikatu (memcmp) */
static volatile uint16_t s_dup_count = 0;        /* K3 diag: licznik duplikatow */
static bool     s_exp_valid = false;             /* C1: oczekiwany seq przy konsumpcji ringu */
static uint16_t s_exp_seq = 0;
static volatile uint8_t  g_info_req = 0;         /* licznik zadan FW info (20 12); set: SWI, consume: main (audyt 2026-08-30) */
static volatile uint8_t  g_batt_req = 0;         /* licznik zadan baterii (20 17); ten sam wzorzec co g_info_req */
static volatile uint16_t g_batt_mv  = 0;         /* ostatni pomiar [mV]; 0 = brak (producent: sleep tick, konsument: main) */
static volatile bool     g_batt_low = false;     /* flags v2 bit3 */
static nrf_drv_wdt_channel_id m_wdt_channel;

static void wdt_event_callback(void)
{
    /* timeout w kontekscie IRQ — feed niemozliwy; celowo puste, WDT wykona reset. */
}
static volatile bool     g_sleep_now = false;

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    uint32_t code = id;
    if (id == NRF_FAULT_ID_SDK_ERROR) {
        code = ((error_info_t *)info)->err_code;
        rtt_diag_printf("APP_ERROR code=%u line=%u", code, ((error_info_t *)info)->line_num);
    } else {
        rtt_diag_printf("FAULT id=%u pc=%08x", (unsigned)id, (unsigned)pc);
    }
    (void)code;   /* przy TRIKIG_RTT_DIAG=0 printf jest no-op (warning release build) */
    /* SOS-loop bez feedu: WDT (12s) wykona reset — fault konczy sie rebootem (audyt #1). */
    for (;;) {
        led_blink(3, 40, 40);
        nrf_delay_ms(1000);
    }
}

/* ---- btn: debounc + edge-detect (audyt 2026-08-30) ----
 * Wczesniej licznik rosly co iteracje petli glownej => pojedyncze wcisniecie
 * liczilo sie jako 3. Teraz probkowanie w kontekscie poll (9ms), 3 stabilne
 * probki = zmiana stanu, liczone tylko krawedzie wcisniecia (SPEC 6: 3 wcisniecia). */
#define BTN_DEBOUNCE_SAMPLES    3u              /* 3 x 9ms = 27ms filtr drgan */

static volatile uint8_t g_btn_presses = 0;      /* licznik wcisniec; producent: poll handler, konsument: main */
static uint8_t s_btn_stable = 0;                /* filtrowany stan: 1 = wcisniety */
static uint8_t s_btn_raw_cnt = 0;               /* licznik stabilnosci surowego stanu */

static void btn_sample(void)                    /* poll timer context (9ms) */
{
    uint8_t raw = btn_pressed() ? 1u : 0u;
    if (raw == s_btn_stable) {
        s_btn_raw_cnt = 0;
        return;
    }
    if (++s_btn_raw_cnt >= BTN_DEBOUNCE_SAMPLES) {
        s_btn_stable = raw;
        s_btn_raw_cnt = 0;
        if (raw && g_btn_presses < 255) g_btn_presses++;    /* krawedz: release -> press */
    }
}

/* ---- C7: akwizycja DRDY -> main loop (ISR tylko timestampuje; I2C/VBT/ring w main) ----
 * 0.3.5: DRDY wychodzi z INT2 ukladu (pin 9 ukladu wg datasheet ukladu; INT1 niepolaczony
 * — 0.3.3/0.3.4 probe nie widzial krawedzi). Register: INT2_CTRL (0x0E) bit0. nRF pin:
 * P0.09 lub P0.10 ("G-klasa INT", SPEC 1) — probe sprawdza pin x polaryzacje (max 400ms).
 * Watchdog runtime: brak probki > 30ms => fallback (licznik drdy_fallbacks). */
APP_TIMER_DEF(m_poll_timer);

static volatile uint32_t s_drdy_ts[DRDY_TS_SLOTS];   /* FIFO timestampow (ISR pisze, main czyta) */
static volatile uint8_t  s_drdy_wr = 0, s_drdy_rd = 0;
static volatile uint8_t  s_fallback_req = 0;         /* zadania watchdog/poll -> main (single-producer) */
static uint32_t s_last_sample_cyc = 0;               /* TIMER1 us @ ostatniej probki (watchdog DRDY) */
static uint32_t s_last_rtc = 0;                      /* RTC1 tick @ ostatniej probki (detekcja gapow > 65.5ms okna TIMER1) */
static bool     s_rtc_valid = false;
static uint32_t s_prev_ts_cyc = 0;                   /* C7: poprzedni timestamp (dt) */
static bool     s_prev_ts_valid = false;

static void drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin; (void)action;
    uint8_t wr = s_drdy_wr;
    uint8_t next = (uint8_t)((wr + 1) % DRDY_TS_SLOTS);
    if (next != s_drdy_rd) {
        s_drdy_ts[wr] = diag_cyc();                  /* [us] timestamp = czas DRDY (nie odczytu) */
        s_drdy_wr = next;
    }                                                /* pelne FIFO: drop (main opozniony; watchdog pokryje) */
}

/* boot-probe DRDY: kandydujace piny nRF (P0.09/P0.10 — "G-klasa INT", SPEC 1) x polaryzacja;
 * 0.3.5: DRDY wychodzi z INT2 ukladu (pin 9 ukladu) na JEDEN z nich — probe rozstrzyga.
 * Blokujace ~100ms na probe (max 400ms); zwraca true gdy DRDY zyje. */
static bool drdy_probe(void)
{
    static const uint32_t pins[2] = { PIN_IMU_INT2, PIN_IMU_INT1 };   /* P0.10 potwierdzony [P] */
    static const nrf_gpiote_polarity_t pols[2] = { NRF_GPIOTE_POLARITY_LOTOHI,
                                                   NRF_GPIOTE_POLARITY_HITOLO };
    if (!lsm6dsl_drdy_enable(true)) return false;   /* I2C padl — polling tez nie zyje */
    for (uint8_t pi = 0; pi < 2; pi++) {
        for (uint8_t pol = 0; pol < 2; pol++) {
            s_drdy_wr = s_drdy_rd = 0;
            nrf_drv_gpiote_in_config_t cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
            cfg.sense = pols[pol];
            if (nrf_drv_gpiote_in_init(pins[pi], &cfg, drdy_handler) != NRF_SUCCESS) {
                lsm6dsl_drdy_enable(false);
                return false;
            }
            nrf_drv_gpiote_in_event_enable(pins[pi], true);
            nrf_delay_ms(100);                       /* ~10 krawedzi @104Hz */
            uint8_t edges = (uint8_t)(s_drdy_wr - s_drdy_rd);
            if (edges >= 5) {
                g_diag.drdy_mode = (uint8_t)(pi * 2 + pol + 1);   /* 1/2 = P0.10 rising/falling
                                                                     (potwierdzony), 3/4 = P0.09 r/f */
                rtt_diag_printf("S2 DRDY ok pin=P0.%02u pol=%u edges=%u",
                                pins[pi], pol, edges);
                return true;
            }
            nrf_drv_gpiote_in_event_disable(pins[pi]);
            nrf_drv_gpiote_in_uninit(pins[pi]);
        }
    }
    lsm6dsl_drdy_enable(false);
    return false;
}

/* odczyt + VBT + ring: wspolne dla DRDY i fallback; ts = czas probki z ISR (DRDY)
 * albo teraz (fallback). ts_valid=false => dt nominalny (fallback nie zna czasu probki). */
static void process_sample(uint32_t ts_cyc, bool ts_valid)
{
    uint32_t t0 = diag_cyc();
    uint8_t raw[12];
    if (!lsm6dsl_read_motion(raw)) return;
    diag_max16(&g_diag.acq_us_max, diag_cyc_us(diag_cyc() - t0));   /* C1: czas I2C burst 12B */

    uint16_t seq = m_sample_seq++;               /* K4: kazda probka, nawet dropnieta */

    /* P0 (audyt VBT 2026-08-30): memcmp tylko DIAGNOSTYKA — probke identyfikuje DRDY.
     * W trybie polling (fallback) duplikat musi nadal nie wchodzic do calkowania
     * (timer dogania ODR); w trybie DRDY kazdy odczyt = nowa probka. */
    bool is_dup = (memcmp(raw, s_prev_raw, sizeof(raw)) == 0);
    memcpy(s_prev_raw, raw, sizeof(raw));
    if (is_dup && s_dup_count < 0xFFFF) s_dup_count++;
    g_diag.imu_dups += is_dup;
    if (!is_dup) g_diag.imu_samples++;

    /* C7: dt = t[n]-t[n-1] z timestampow DRDY; clamp min/max + dt_fault; gap > 100ms
     * => twardy ZUPT (reset integratora) i dt nominalny. */
    uint16_t dt_q16 = TRIKIG_VBT_DT_104HZ_Q16;

    /* detekcja gapow: RTC1 (app_timer, 30.5us/tick, wrap 512s) — niezalezna od 16-bit
     * okna TIMER1; gap => twardy ZUPT + dt nominalny (przegapiona probka nie wchodzi). */
    uint32_t now_rtc = NRF_RTC1->COUNTER;
    if (s_rtc_valid) {
        uint32_t rtc_diff = (now_rtc - s_last_rtc) & 0xFFFFFFu;
        if (rtc_diff > DT_GAP_TICKS) {
            g_diag.dt_faults++;
            vbt_reset_velocity();
            s_prev_ts_valid = false;             /* dt z timer-ow po gapie nie liczy sie */
        }
    }
    s_last_rtc = now_rtc;
    s_rtc_valid = true;

    if (ts_valid && g_diag.drdy_mode != 0) {
        if (s_prev_ts_valid) {
            uint32_t dt_us = (uint16_t)(ts_cyc - s_prev_ts_cyc);   /* wrap-safe (okno 65.5ms) */
            diag_period_us(dt_us);               /* min/avg/max okresu probkowania */
            if (dt_us < DT_MIN_US || dt_us > DT_MAX_US) {
                g_diag.dt_faults++;
                dt_us = (dt_us < DT_MIN_US) ? DT_MIN_US : DT_MAX_US;
            }
            dt_q16 = (uint16_t)(((uint64_t)dt_us << 16) / 1000000u);
        }
        s_prev_ts_cyc = ts_cyc;
        s_prev_ts_valid = true;
    }

    /* K5: VBT liczony NIEZALEZNIE od stanu ringu BLE (backpressure nie zatrzymuje calkowania) */
    if (!is_dup || g_diag.drdy_mode != 0) {
        t0 = diag_cyc();
        vbt_on_frame(raw, dt_q16);
        diag_max16(&g_diag.dsp_us_max, diag_cyc_us(diag_cyc() - t0));   /* C1: czas DSP/VBT */
    }

    uint8_t slot = ring_wr;
    uint8_t next = (uint8_t)((slot + 1) % RING_SLOTS);
    if (next == ring_rd) {
        g_diag.ring_drops++;                     /* C1: drop ramki BLE; seq juz poszedl (K4) */
        return;
    }

    memcpy(ring[slot].raw, raw, sizeof(raw));
    ring[slot].seq     = seq;
    ring[slot].vel_mms = (int16_t)vbt_velocity_mms();   /* K2: SNAPSHOT przy probce, nie przy wysylce */
    /* K2 (audyt H 2026-08-30): low-batt tez snapshot przy probce — jeden model danych,
     * nie OR przy wysylce; bateria wolnozmienna, ale spójnosc architektury wazniejsza. */
    ring[slot].flags   = (uint8_t)(vbt_flags() | (g_batt_low ? TRIKIG_BATT_FLAGS_LOW : 0));
    __DMB();
    ring_wr = next;
    s_last_sample_cyc = diag_cyc();
}

/* poll 9ms: BTN + zadawanie probek w trybie polling + watchdog DRDY */
static void poll_timeout_handler(void * p_context)
{
    (void)p_context;
    btn_sample();
    if (!m_imu_ok) return;
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) return;

    if (g_diag.drdy_mode == 0) {
        if (s_fallback_req < 255) s_fallback_req++;      /* tryb polling: probka co tick */
    } else if ((uint16_t)(diag_cyc() - s_last_sample_cyc) > 30000u) {   /* 30ms, wrap-safe */
        if (s_fallback_req < 255) s_fallback_req++;      /* watchdog: DRDY milczy > 30ms */
    }
}

/* ---- sleep 5 min bez polaczenia: app_timer 1s zamiast iteracji petli ---- */
APP_TIMER_DEF(m_sleep_timer);
/* idle_secs liczy sekundy BEZ polaczenia (zerowany co iteracje gdy polaczony); przy
 * disconnect celowo NIE zerowany — liczymy czas bez polaczenia, nie od rozlaczenia (audyt 027). */
static volatile uint32_t idle_secs = 0;
static volatile bool    g_go_sleep = false;

static void sleep_timeout_handler(void * p_context)
{
    (void)p_context;
    /* F5: pomiar baterii 1x/s niezaleznie od polaczenia (blokujace ~0.5ms w SWI —
     * dopuszczalne; poll I2C robi ~300us w tym samym kontekscie co 9ms). */
    uint16_t mv = batt_sample_mv();
    if (mv != 0) {
        g_batt_mv  = mv;
        g_batt_low = batt_is_low(mv);
    }
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        idle_secs = 0;
        return;
    }
    if (++idle_secs >= SLEEP_ITERS) {
        g_go_sleep = true;   /* SYSTEMOFF z petli glownej, nie z kontekstu SWI */
    }
}

/* ---- BLE events ---- */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            idle_secs = 0;
            s_exp_valid = false;                 /* C1: nowa sesja liczenia luk seq */
            nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            ble_gap_phys_t const phys = {BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO};
            (void)sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            break;
        }
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            (void)sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            break;
        case BLE_GATTC_EVT_TIMEOUT:
            (void)sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            (void)sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;
        default:
            break;
    }
}
NRF_SDH_BLE_OBSERVER(m_ble_obs, BLE_NUS_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

static void nus_evt_handler(ble_nus_evt_t *p_evt)
{
    if (p_evt->type != BLE_NUS_EVT_RX_DATA) return;
    const uint8_t *d = p_evt->params.rx_data.p_data;
    uint16_t len = p_evt->params.rx_data.length;
    if (len < 2 || d[0] != 0x20) return;

    switch (d[1]) {
        case 0x10:                       /* init stocka (kompat) = stream on */
            m_stream_on = true;
            break;
        case 0x11:                       /* wire mode: 01 = v2, 00 = v1 */
            if (len >= 3) {
                uint8_t nm = (d[2] == 0x01) ? 2u : 1u;
                if (nm != m_wire_mode) {
                    m_wire_mode = nm;
                    rtt_diag_printf("CMD wire v%u", m_wire_mode);
                }
            }
            break;
        case 0x12:                       /* FW info request -> ramka 22 03 */
            g_info_req++;                /* licznik: zadanie z SWI nie przepada przy consume w main */
            break;
        case 0x17:                       /* battery request -> ramka 22 04 */
            g_batt_req++;
            break;
        case 0x15:                       /* stream on/off */
            if (len >= 3) m_stream_on = (d[2] == 0x01);
            break;
        case 0x16:                       /* sleep now */
            g_sleep_now = true;
            break;
        default:
            break;
    }
}

static void ble_stack_init(void)
{
    APP_ERROR_CHECK(nrf_sdh_enable_request());
    uint32_t ram_start = 0;
    APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start));
    APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));
}

static void gap_params_init(void)
{
    ble_gap_conn_params_t cp = {
        .min_conn_interval = MIN_CONN_INTERVAL,
        .max_conn_interval = MAX_CONN_INTERVAL,
        .slave_latency     = SLAVE_LATENCY,
        .conn_sup_timeout  = CONN_SUP_TIMEOUT,
    };
    APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&cp));
    APP_ERROR_CHECK(sd_ble_gap_device_name_set(NULL, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME)));
}

static void services_init(void)
{
    ble_nus_init_t ni = { .data_handler = nus_evt_handler };
    APP_ERROR_CHECK(ble_nus_init(&m_nus, &ni));
}

static void advertising_init(void)
{
    ble_advertising_init_t ai = {0};
    ai.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    ai.advdata.include_appearance = false;
    ai.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ai.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids)/sizeof(m_adv_uuids[0]);
    ai.srdata.uuids_complete.p_uuids = m_adv_uuids;
    ai.config.ble_adv_fast_enabled = true;
    ai.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    ai.config.ble_adv_fast_timeout = APP_ADV_TIMEOUT;
    APP_ERROR_CHECK(ble_advertising_init(&m_advertising, &ai));
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void conn_params_init(void)
{
    ble_conn_params_init_t cpi = {0};
    cpi.first_conn_params_update_delay = FIRST_CP_DELAY;
    cpi.next_conn_params_update_delay = NEXT_CP_DELAY;
    cpi.max_conn_params_update_count = MAX_CP_COUNT;
    APP_ERROR_CHECK(ble_conn_params_init(&cpi));
}

/* ---- main ---- */
int main(void)
{
    nrf_gpio_cfg_output(PIN_LED);
    led_write(false);
    nrf_gpio_cfg_input(PIN_BTN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(PIN_IMU_INT1, NRF_GPIO_PIN_NOPULL);

    /* WDT uzbrojony jako pierwsza rzecz — chroni caly boot, nie tylko main loop (plan 027 K0).
     * Boot ma ~2.8s LED-blinkow; reload 12s daje zapas. Pierwszy feed w petli glownej przejmuje. */
    nrf_drv_wdt_config_t wdt_cfg = NRF_DRV_WDT_DEAFULT_CONFIG;
    wdt_cfg.reload_value = 12000;
    APP_ERROR_CHECK(nrf_drv_wdt_init(&wdt_cfg, wdt_event_callback));
    APP_ERROR_CHECK(nrf_drv_wdt_channel_alloc(&m_wdt_channel));
    nrf_drv_wdt_enable();

    rtt_diag_printf("S1 main enter " TRIKIG_FW_TAG);
    diag_init();                                 /* C1: TIMER1 @1MHz (DWT CYCCNT brak na 52810!) */
    led_blink(1, 40, 80);
    nrf_delay_ms(700);

    bb_i2c_init();
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        m_imu_ok = lsm6dsl_init();               /* WHO_AM_I z bus-clear + SW reset + config */
        rtt_diag_printf("S2 try=%u imu_ok=%d", attempt, (int)m_imu_ok);
        if (m_imu_ok) break;
        nrf_delay_ms(50);
    }
    if (!m_imu_ok) rtt_diag_printf("S2 IMU DEAD (HW?)");
    vbt_reset();                                 /* bias grawitacji zasilony pierwszymi ramkami (spoczynek) */
    /* C7: probe DRDY (polaryzacja INT1 niezweryfikowana plytowo — D-016; runtime-probe
     * zlicza krawedzie w 100ms, ~10 @104Hz; rising -> falling -> fallback polling) */
    if (m_imu_ok && nrf_drv_gpiote_init() == NRF_SUCCESS && drdy_probe()) {
        /* DRDY aktywny: probki procesowane w main loop z timestampow ISR */
    } else {
        rtt_diag_printf("S2 DRDY off -> polling");
    }
    batt_init();                                 /* F5: SAADC AIN2 + kalibracja offsetu (blokujaca ~ms) */
    uint16_t batt_mv_boot = batt_sample_mv();    /* pierwszy pomiar od razu (nie czekamy 1s ticka) */
    if (batt_mv_boot != 0) {
        g_batt_mv  = batt_mv_boot;
        g_batt_low = batt_is_low(batt_mv_boot);
    }
    rtt_diag_printf("S2 batt=%umV", batt_mv_boot);
    led_blink(2, 40, 80);
    nrf_delay_ms(700);

    /* app_timer PRZED stackiem BLE (audyt 2026-08-30): ble_advertising_init/ble_conn_params_init
     * tworza timery wewnatrz *_init; dotychczasowa kolejnosc dzialala wylacznie dzieki
     * APP_ADV_TIMEOUT=0 (advertising nie startowal timera przed app_timer_init). */
    APP_ERROR_CHECK(app_timer_init());

    ble_stack_init();
    gap_params_init();
    APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, NULL));
    nrf_ble_qwr_init_t qi = {0};
    APP_ERROR_CHECK(nrf_ble_qwr_init(&m_qwr, &qi));
    services_init();
    advertising_init();
    conn_params_init();
    rtt_diag_printf("S3 stack+services ok");
    led_blink(3, 40, 80);
    nrf_delay_ms(700);

    uint32_t adv_err = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    rtt_diag_printf("adv_start err=%u", (unsigned)adv_err);
    if (adv_err != NRF_SUCCESS) {
        for (;;) { led_blink(3, 40, 40); nrf_delay_ms(1000); }
    }
    rtt_diag_printf("S4 advertising ON: " DEVICE_NAME);
    led_blink(4, 40, 80);
    nrf_delay_ms(700);

    APP_ERROR_CHECK(app_timer_create(&m_poll_timer, APP_TIMER_MODE_REPEATED, poll_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(m_poll_timer, APP_TIMER_TICKS(POLL_INTERVAL_MS), NULL));
    APP_ERROR_CHECK(app_timer_create(&m_sleep_timer, APP_TIMER_MODE_REPEATED, sleep_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(m_sleep_timer, APP_TIMER_TICKS(SLEEP_TIMER_MS), NULL));
    rtt_diag_printf("S5 poll %ums ON, sleep %us, drdy=%u", POLL_INTERVAL_MS, SLEEP_TIMEOUT_S, g_diag.drdy_mode);

#if TRIKIG_RTT_DIAG
    uint16_t vbt_log_div = 0;    /* diag: dzielnik logu VBT ~1s; przy DIAG=0 nieuzywane */
#endif

    /* WDT uzbrojony na starcie main (K0); tu tylko feed (fault/SOS-loop => reset). */
    for (;;) {
        nrf_drv_wdt_channel_feed(m_wdt_channel);
        if (g_sleep_now) {
            g_go_sleep = true;
        }

        if (g_info_req != 0 && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
            g_info_req--;                /* consume PRZED wysylka: nowe zadania z SWI nie przepadna */
            uint8_t info[6] = {0x22, 0x03, TRIKIG_FW_MAJOR, TRIKIG_FW_MINOR, TRIKIG_FW_PATCH, m_wire_mode};
            uint16_t hvx = sizeof(info);
            (void)ble_nus_data_send(&m_nus, info, &hvx, m_conn_handle);
            rtt_diag_printf("CMD info -> mode v%u", m_wire_mode);
        }

        if (g_batt_req != 0 && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
            g_batt_req--;                /* ten sam wzorzec co g_info_req */
            uint16_t mv = g_batt_mv;
            if (mv == 0) mv = batt_sample_mv();   /* fallback: zapytanie przed pierwszym tickiem 1s */
            uint8_t batt[4] = {0x22, 0x04, (uint8_t)(mv & 0xFF), (uint8_t)(mv >> 8)};
            uint16_t bhvx = sizeof(batt);
            (void)ble_nus_data_send(&m_nus, batt, &bhvx, m_conn_handle);
            rtt_diag_printf("CMD batt -> %umV", mv);
        }

        if (g_go_sleep) {
            (void)app_timer_stop(m_poll_timer);
            (void)app_timer_stop(m_sleep_timer);
            enter_system_off();
        }

        /* Konsumpcja w sekcji krytycznej (audyt#2 2026-08-30): read-modify-write
         * g_btn_presses w main przegrywal wyscig z ++ z poll handlera (zgubione wcisniecie);
         * '-= 3' zachowuje nadmiarowe zliczenia zrobione podczas blinka. */
        bool btn_3 = false;
        CRITICAL_REGION_ENTER();
        if (g_btn_presses >= 3) {
            g_btn_presses -= 3;
            btn_3 = true;
        }
        CRITICAL_REGION_EXIT();
        if (btn_3) {                     /* SPEC 6: 3 wcisniecia = 2x mrug + reset licznika sleep */
            led_blink(2, 60, 60);
            idle_secs = 0;
        }

        /* C7: probki z FIFO timestampow DRDY (ISR -> main). Niepolaczony: drain-and-discard,
         * zeby po connect nie policzyc dt ze starych timestampow. */
        while (s_drdy_wr != s_drdy_rd) {
            uint32_t ts = s_drdy_ts[s_drdy_rd];
            s_drdy_rd = (uint8_t)((s_drdy_rd + 1) % DRDY_TS_SLOTS);
            if (m_imu_ok && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
                process_sample(ts, true);
            }
        }
        /* fallback (tryb polling albo watchdog DRDY): zadanie z poll-timera */
        if (s_fallback_req != 0) {
            s_fallback_req--;
            if (m_imu_ok && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
                process_sample(diag_cyc(), false);
            }
        }

        while (ring_rd != ring_wr) {
            uint8_t slot = ring_rd;

            /* C1: luki seq widziane przez konsumenta (drop ringu = gap zgodnie z kontraktem 5.1) */
            if (s_exp_valid && ring[slot].seq != s_exp_seq) {
                g_diag.seq_gaps += (uint16_t)(ring[slot].seq - s_exp_seq);
            }
            s_exp_seq = (uint16_t)(ring[slot].seq + 1);
            s_exp_valid = true;

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_stream_on) {
                uint8_t txbuf[FRAME_V2_SIZE];
                const uint8_t *p_tx;
                uint16_t txlen;
                if (m_wire_mode == 2) {
                    txbuf[0] = 0x22; txbuf[1] = 0x01;
                    txbuf[2] = (uint8_t)(ring[slot].seq & 0xFF);
                    txbuf[3] = (uint8_t)(ring[slot].seq >> 8);
                    memcpy(&txbuf[4], ring[slot].raw, 12);
                    txbuf[16] = (uint8_t)((uint16_t)ring[slot].vel_mms & 0xFF);
                    txbuf[17] = (uint8_t)((uint16_t)ring[slot].vel_mms >> 8);
                    /* bity 0-4 = snapshot VBT (bit4 = g-forced, audyt H: low-batt juz w snapshot) */
                    txbuf[18] = ring[slot].flags;
                    p_tx = txbuf;
                    txlen = FRAME_V2_SIZE;
                } else {
                    txbuf[0] = 0x22; txbuf[1] = 0x00;
                    memcpy(&txbuf[2], ring[slot].raw, 12);
                    p_tx = txbuf;
                    txlen = FRAME_SIZE;
                }
                /* v0.0.27 (audyt #2): JEDNA proba + drop przy kazdym bledzie (tez default).
                 * Zero retry => zero ryzyka zamrozenia petli glownej przez BLE. */
                if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
                    uint16_t hvx = txlen;
                    uint32_t t1 = diag_cyc();
                    /* SDK ble_nus_data_send przyjmuje uint8_t* (API bez const); funkcja nie modyfikuje bufora. */
                    uint32_t err = ble_nus_data_send(&m_nus, (uint8_t *)p_tx, &hvx, m_conn_handle);
                    diag_max16(&g_diag.ble_us_max, diag_cyc_us(diag_cyc() - t1));   /* C1 */
                    if (err != NRF_SUCCESS) {
                        g_diag.ble_drops++;                             /* C1: drop send-path */
                        static uint32_t s_ble_err_first = 0;            /* audyt: 100% bdrop w logu 0.3.3 */
                        if (s_ble_err_first == 0) {
                            s_ble_err_first = err;
                            rtt_diag_printf("BLE send err=0x%x (8=INVALID_STATE: brak CCCD)", err);
                        }
                    }
                }
            }
            ring_rd = (uint8_t)((ring_rd + 1) % RING_SLOTS);
#if TRIKIG_RTT_DIAG
            if (++vbt_log_div >= IMU_ODR_HZ) {   /* ~1s: velocity do walidacji vs PWA */
                vbt_log_div = 0;
                rtt_diag_printf("VBT v=%d mm/s mv=%u dup=%u", (int)vbt_velocity_mms(), (unsigned)vbt_moving(), (unsigned)s_dup_count);
                diag_print();                    /* C1: liczniki dropow + okresy + timingi */
            }
#endif
        }
        (void)sd_app_evt_wait();
    }
}
