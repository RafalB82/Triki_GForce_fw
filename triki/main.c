/**
 * trikig — nRF52810 + LSM6DSL, S112 7.2.0 (refaktor v19)
 *
 * Moduly: trikig_bb_i2c (I2C), trikig_lsm6dsl (IMU), trikig_board (LED/BTN/sleep/RTT),
 *          trikig_vbt (velocity side-band O-012 — wire 14B bez zmian).
 * Zachowane semantyki v19: poll 9ms -> ramka 14B -> ring -> NUS; skale CTRL1=0x44/CTRL2=0x4C.
 * v0.0.27 (audyt 2026-08-29): WDT 8s (kick w petli glownej, fault=>reset zamiast SOS-loop),
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
#include "nrf_drv_wdt.h"

#include "trikig_board.h"
#include "trikig_vbt.h"
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
#define RING_SLOTS              4u
#define POLL_INTERVAL_MS        9u          /* ~111Hz timer vs ODR 104 -> okazjonalne duplikaty (v19) */

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
static volatile uint8_t  g_info_req = 0;         /* licznik zadan FW info (20 12); set: SWI, consume: main (audyt 2026-08-30) */
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
    /* SOS-loop bez feedu: WDT (8s) wykona reset — fault konczy sie rebootem (audyt #1). */
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

/* ---- poll: IMU OUT -> ramka -> ring (app_timer context) ---- */
APP_TIMER_DEF(m_poll_timer);

static void poll_timeout_handler(void * p_context)
{
    (void)p_context;
    btn_sample();
    if (!m_imu_ok) return;
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) return;

    uint8_t raw[12];
    if (!lsm6dsl_read_motion(raw)) return;

    uint16_t seq = m_sample_seq++;               /* K4: kazda probka, nawet dropnieta */

    /* K3: duplikat (timer 9ms vs ODR 104Hz) nie wchodzi do calkowania VBT */
    bool is_dup = (memcmp(raw, s_prev_raw, sizeof(raw)) == 0);
    memcpy(s_prev_raw, raw, sizeof(raw));
    if (is_dup && s_dup_count < 0xFFFF) s_dup_count++;

    /* K5: VBT liczony NIEZALEZNIE od stanu ringu BLE (backpressure nie zatrzymuje calkowania) */
    if (!is_dup) {
        vbt_on_frame(raw);
    }

    uint8_t slot = ring_wr;
    uint8_t next = (uint8_t)((slot + 1) % RING_SLOTS);
    if (next == ring_rd) return;                 /* pelny ring -> drop ramki BLE; seq juz poszedl (K4) */

    memcpy(ring[slot].raw, raw, sizeof(raw));
    ring[slot].seq     = seq;
    ring[slot].vel_mms = (int16_t)vbt_velocity_mms();   /* K2: SNAPSHOT przy poll, nie przy wysylce */
    ring[slot].flags   = vbt_flags();                    /* K2: SNAPSHOT przy poll */
    __DMB();
    ring_wr = next;
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
    rtt_diag_printf("S5 poll %ums ON, sleep %us", POLL_INTERVAL_MS, SLEEP_TIMEOUT_S);

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

        if (g_go_sleep) {
            (void)app_timer_stop(m_poll_timer);
            (void)app_timer_stop(m_sleep_timer);
            enter_system_off();
        }

        if (g_btn_presses >= 3) {        /* SPEC 6: 3 wcisniecia = 2x mrug + reset licznika sleep */
            g_btn_presses = 0;
            led_blink(2, 60, 60);
            idle_secs = 0;
        }

        while (ring_rd != ring_wr) {
            uint8_t slot = ring_rd;

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
                    /* SDK ble_nus_data_send przyjmuje uint8_t* (API bez const); funkcja nie modyfikuje bufora. */
                    (void)ble_nus_data_send(&m_nus, (uint8_t *)p_tx, &hvx, m_conn_handle);
                }
            }
            ring_rd = (uint8_t)((ring_rd + 1) % RING_SLOTS);
#if TRIKIG_RTT_DIAG
            if (++vbt_log_div >= 104) {          /* ~1s: velocity do walidacji vs PWA */
                vbt_log_div = 0;
                rtt_diag_printf("VBT v=%d mm/s mv=%u dup=%u", (int)vbt_velocity_mms(), (unsigned)vbt_moving(), (unsigned)s_dup_count);
            }
#endif
        }
        (void)sd_app_evt_wait();
    }
}
