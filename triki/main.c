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
 * v0.5.0 (MINOR — nowa komenda RX, D-020; baseline diagnostyczny [P] 2026-09-03):
 * (1) RX 0x20 0x18 = training mode (01=ON/00=OFF, auto-OFF przy disconnect):
 *     ON wylacza HW activity/inactivity (lsm6dsl_inactivity_enable(false)) —
 *     klasa ruchu dip/pullup 140-246mg p95 siedzi pod WK_THS 250mg @FS16g (podloga
 *     LSB), wiec HW usypial w serii (CSV 21-36% pokrycia ramek vs duration, reszta
 *     serii bez ramek w ogole). ON + sleeping => lsm6dsl_wake_force() (jawne CTRL1/2
 *     V19 z readbackiem — INACT_EN=00 nie jest w DS6207 opisane jako wybudzenie),
 *     idle_connected_set(false) + vbt_reset() (czysta kalibracja). (2) SLEEP_DUR
 *     4s -> 6s (margines na starty serii bez komendy). WK_THS bez zmian (podloga).
 * v0.4.3 (smoke v0.4.2 [P], log RTT+nRF Connect 16:31): cpfail=0 przez 3 przejscia
 * (fix gated apply dziala), dtf=1 nierosnace (fix RTC1 dziala), conn params
 * 150ms/lat4 OD CONNECT (sync) i w rytmie S6. NOWY FINDING: klasa ruchu 30-250mg
 * (prog rest VBT 0.3 m/s^2 ~ 30mg << WK_THS 250mg) integrowala velocity @12.5Hz
 * z zamrozonym gyro => phantom az do clampa 15625 mm/s (obrot kapsla = g_est
 * nie sledzi). Fix: velocity w IDLE = 0 BY DESIGN (trikig_vbt: s_idle => s_vel=0);
 * realny trening >250mg wybudza HW => 104Hz => pelne VBT od zera.
 * v0.4.2 (smoke v0.4.1 [P], log RTT 2026-09-01): (1) dtf +1/probke w IDLE — RTC1
 * przy app_timer2 ma PRESCALER=1 => 16384Hz (61.035us/tick), nie 32768Hz; progi
 * IDLE przeliczone (min 655/max 1966/wdt 2621/gap 3277), dt q16.16 = ticks*4
 * (dokladne: 2^16/2^14). Odkrycie uboczne: DT_GAP_TICKS 1966 = de facto 120ms
 * (komentarz 60ms bledny od 0.3.4 — ACTIVE nieczuly, zostaje). (2) cpfail +1/
 * przejscie — readback WAKE_UP_SRC kasuje LIR, TOGGLE lapie deassert => drugi
 * event z tym samym stanem => ponowny change_conn_params = BUSY; idle_cp_apply
 * gated na zmiane stanu, connect-sync apply jawnie. (3) Obserwacje smoke: ruch
 * powoli (<250mg WK_THS) NIE wybudza z IDLE (v rampuje do ~1100 mm/s @12.5Hz,
 * ZUPT po powrocie do rest) — WK_THS do strojenia; wake burst: bdrop +~80 przy
 * 104Hz na conn 150ms przed aplikacja restore (przejsciowe, self-heal).
 * v0.4.1 (smoke v0.4.0 [P], log RTT 2026-09-01): inact cfg=00 MISMATCH — reg_write/
 * reg_read (bb) po starcie TWIM NIE steruja magistrala (nrfx_twim_enable przejmuje
 * piny 5/6) => zapisy activity/inactivity po probe szly w pustke. Fix: runtime reg
 * access TWIM-aware (reg_write_t/reg_read_t, wzorzec read_motion z fallbackiem bb),
 * sync readback przy CONNECTED przeniesiony ze SWI do main (g_conn_sync_req — I2C
 * nie w kontekscie BLE handlera). Boot-init bez zmian (bb przed TWIM, stuck-bus).
 * v0.4.0 (IDLE-CONNECTED, plan 2026-09-01): HW activity/inactivity LSM6DSL —
 * TAP_CFG INACT_EN=11 + LIR, WK_THS=250mg @FS16g, SLEEP_DUR=4s, MD1_CFG bit7
 * SLEEP_CHANGE -> INT1/P0.09 (GPIOTE TOGGLE). Po SLEEP_DUR bezruchu HW SAM schodzi
 * do low-power (acc 12.5Hz LP + gyro power-down), activity => auto-restore
 * (DS6207 6.5.2 — do potwierdzenia readbackiem na HW). FW: sync stanu HW readbackiem
 * WAKE_UP_SRC (event INT1 + connect), watchdog DRDY w IDLE 160ms na
 * RTC1 (okno TIMER1 65.5ms nie miesci okresu 80ms), dt w IDLE z RTC1
 * (ticks*2 => q16.16 dokladnie), gap 200ms, vbt_idle() wylacza propagacje gyro +
 * nauke biasu (OUT zamrozone BDU w power-down — klasa ryzyka slowrot), stream 19B
 * leci dalej @12.5Hz (luki seq = ODR, kontrakt 5.1), conn params 150ms/lat4 w
 * IDLE i restore 7.5-15ms po activity (best-effort: modul ble_conn_params moze
 * jednorazowo cofnac — licznik idle_cp_fail). IDLE tylko przy DRDY na P0.10.
 * v0.3.11 (pomiar plyty [P]: INT1->P0.09, INT2->P0.10): wczesniejszy "brak krawedzi"
 * byl FALSE-NEGATIVE — DRDY z BDU zalega HIGH dopoki nikt nie czyta OUT, a scan/probe
 * biegly przed startem pollingu (zero odczytow => zero rosnacych krawedzi). Fix: okna
 * probe/scan z drain-read co 10ms (krawedz co ~9.6ms); probe domyslnie ON.
 * v0.3.10: DRDY zamkniete dla tego HW — scan [P] (29 pinow, INT1+INT2 wlaczone, readback
 * OK) = zero krawedzi => zaden INT nie jest poprowadzony do nRF; probe domyslnie off
 * (TRIKIG_DRDY_PROBE), akwizycja = polling (zwalidowany). — WNIOSEK ODDWOLANY (fneg)
 * v0.3.9 (log wire v2 22:22): regresja 0.3.7 — nauka biasu w quasi-bezruchu pochlaniala
 * wolne rotacje (<=15dps) => wbias zatruty => propagacja nadrabiala w zla strone => rampa
 * do 15054; repro harness slowrot (FAIL przed, PASS po); nauka TYLKO przy pelnym rest I
 * |w-wbias| < 5 dps.
 * v0.3.8: probe DRDY miala bug licznika — FIFO timestampow 4 sloty wypelnialo sie w
 * 100ms zanim licznik doszedl do progu 5 krawedzi => probe NIGDY nie miala szansy
 * przejsc (od 0.3.1; nakladalo sie na zly register INT1/INT2). Teraz: dedykowany
 * licznik krawedzi w probe (s_probe_cnt), FIFO startuje czyste.
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

/* ---- v0.4.0 IDLE-CONNECTED: progi ODR-aware dla akwizycji 12.5Hz (okres 80ms) ----
 * Okno TIMER1 16-bit (65.5ms) NIE miesci okresu 80ms => dt i watchdog w IDLE na
 * RTC1. UWAGA [P] (log smoke 0.4.1): app_timer2 ustawia PRESCALER=1
 * (APP_TIMER_CONFIG_RTC_FREQUENCY=1) => RTC1 = 32768/2 = 16384 Hz = 61.035us/tick
 * (NIE 30.5us!). q16.16 sekundy z tickow = ticks*4 DOKLADNIE (2^16/2^14 = 4).
 * Progi przeliczone na 61.035us/tick. (Stary DT_GAP_TICKS 1966 to de facto 120ms,
 * nie 60ms jak w komentarzu — ACTIVE nieczuly na ta roznice, zostaje jak bylo.) */
#define IDLE_ODR_HZ             12u         /* ~12.5Hz: dzielnik logu VBT w IDLE */
#define IDLE_DT_125HZ_Q16       5243u       /* 1/12.5 s = 80ms q16.16 (nominal IDLE) */
#define IDLE_RTC_TICK_US        61u         /* RTC1 tick [us] przy PRESCALER=1 (16384Hz) */
#define DT_IDLE_MIN_TICKS       655u        /* 40ms (655 x 61.0356us) */
#define DT_IDLE_MAX_TICKS       1966u       /* 120ms — poza tym = probka zgubiona */
#define IDLE_WDT_TICKS          2621u       /* watchdog DRDY w IDLE: 160ms (2x okres) */
#define DT_IDLE_GAP_TICKS       3277u       /* gap > 200ms => twardy ZUPT (IDLE) */
#define IDLE_CP_INTERVAL_MS     150u        /* conn interval w IDLE; latency 4 => rzadkie eventy radiowe */
#define IDLE_CP_LATENCY         4u

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
static volatile uint16_t s_probe_cnt = 0;            /* licznik krawedzi probe (0.3.8: FIFO 4 sloty
                                                     * wypelnialo sie w 100ms zanim licznik doszedl
                                                     * do progu 5 — probe NIGDY nie mogla przejsc) */
static volatile bool     s_probing = false;
static volatile uint8_t  s_fallback_req = 0;         /* zadania watchdog/poll -> main (single-producer) */
static volatile bool     g_idle_connected = false;   /* v0.4.0: sesja w trybie IDLE-CONNECTED
                                                       * (definicja logiki: idle_connected_set nizej) */
static uint32_t s_last_sample_cyc = 0;               /* TIMER1 us @ ostatniej probki (watchdog DRDY) */
static uint32_t s_last_rtc = 0;                      /* RTC1 tick @ ostatniej probki (detekcja gapow > 65.5ms okna TIMER1) */
static bool     s_rtc_valid = false;
static uint32_t s_prev_ts_cyc = 0;                   /* C7: poprzedni timestamp (dt) */
static bool     s_prev_ts_valid = false;
static uint32_t s_prev_rtc_base = 0;                 /* v0.4.0: baza dt w IDLE (RTC1 ticks) */

static void drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin; (void)action;
    if (s_probing) {
        if (s_probe_cnt < 0xFFFF) s_probe_cnt++;     /* probe: licz krawedzie, FIFO nieistotne */
        return;
    }
    uint8_t wr = s_drdy_wr;
    uint8_t next = (uint8_t)((wr + 1) % DRDY_TS_SLOTS);
    if (next != s_drdy_rd) {
        s_drdy_ts[wr] = diag_cyc();                  /* [us] timestamp = czas DRDY (nie odczytu) */
        s_drdy_wr = next;
    }                                                /* pelne FIFO: drop (main opozniony; watchdog pokryje) */
}

/* ---- v0.4.0 IDLE-CONNECTED: INT1/P0.09 = SLEEP_CHANGE (activity/inactivity) ----
 * HW (INACT_EN=11): po SLEEP_DUR bezruchu acc -> 12.5Hz LP + gyro power-down,
 * activity => auto-restore. ISR tylko flaguje (ten sam wzorzec co drdy_handler);
 * konsumpcja + sync readbackiem WAKE_UP_SRC w petli glownej — NIE w ISR (I2C w
 * kontekscie GPIOTE zakazane, tak jak DRDY czyta dane dopiero w main).
 * g_idle_connected MIRRORUJE stan HW (przejscia lapane takze bez polaczenia —
 * GPIOTE zyje bez BLE), conn params aplikujemy osobno przy connect. */
static volatile bool g_activity_event = false;   /* INT1 edge -> main */
static volatile int8_t g_train_req = -1;         /* v0.5.0 training mode: RX 0x18 -> main
                                                  * (I2C nie w SWI): -1 brak, 0=OFF, 1=ON */
static volatile bool g_conn_sync_req = false;    /* v0.4.1: CONNECTED -> sync IDLE w main
                                                  * (I2C readback NIE w kontekscie SWI —
                                                  * ten sam wzorzec co g_info_req) */

static void activity_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin; (void)action;
    if (s_probing) return;                     /* probe DRDY na P0.09? nie miesza sie */
    g_activity_event = true;
}

/* v0.4.0: DRDY (INT2) zostaje WLACZONY takze w IDLE — stream 19B biegnie dalej
 * @12.5Hz (seq ciagly, tylko wolniejszy — K4 licznik nie robi luk z powodu ODR),
 * BLE zywe bez wymyslonych keep-alive (SoftDevice i tak utrzymuje lacze pustymi
 * conn eventami). Wylaczenie DRDY w IDLE odrzucone: BDU trzymaloby linie HIGH bez
 * odczytow (fneg 0.3.11) i szerzylo fallback-polling zamiast oszczednosci.
 * Conn params przez ble_conn_params_change_conn_params (SDK 17.1.1): modul
 * PRZEJMUJE nowe parametry jako preferowane i sam pilnuje egzekwowania z wlasnym
 * budzetem prob (MAX_CP_COUNT) — bez recznego ping-pongu. ACTIVE = ppcp
 * (7.5-15ms/lat0, sd_ble_gap_ppcp_set z conn_params_init), IDLE = 150ms/lat4.
 * Odrzucenie przez hosta = bezszkodne (lacze chodzi na tym co central dal). */
static void idle_cp_apply(void)
{
    /* dopasuj conn params do stanu IDLE — tylko przy zywym polaczeniu */
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) return;
    ble_gap_conn_params_t cp = {
        .min_conn_interval = g_idle_connected ? MSEC_TO_UNITS(IDLE_CP_INTERVAL_MS, UNIT_1_25_MS)
                                              : MIN_CONN_INTERVAL,
        .max_conn_interval = g_idle_connected ? MSEC_TO_UNITS(IDLE_CP_INTERVAL_MS, UNIT_1_25_MS)
                                              : MAX_CONN_INTERVAL,
        .slave_latency     = g_idle_connected ? IDLE_CP_LATENCY : SLAVE_LATENCY,
        .conn_sup_timeout  = CONN_SUP_TIMEOUT,
    };
    uint32_t err = ble_conn_params_change_conn_params(m_conn_handle, &cp);
    if (err != NRF_SUCCESS && g_diag.idle_cp_fail < 0xFFFF) g_diag.idle_cp_fail++;
}

static void idle_connected_set(bool on)
{
    if (on != g_idle_connected) {
        g_idle_connected = on;
        g_diag.idle_state = (uint8_t)(on ? 1u : 0u);
        if (g_diag.idle_trans < 0xFFFF) g_diag.idle_trans++;
        vbt_idle(on);                           /* zamrozony gyro: bez propagacji/nauki biasu */
        s_prev_ts_valid = false;               /* dt przez granice ODR nie ma sensu */
        s_fallback_req = 0;                     /* nie wnosic starych zadan watchdogu */
        if (!on) s_last_sample_cyc = diag_cyc();/* wyjscie: ostatnia probka IDLE mogla byc
                                                 * >30ms temu — odswiez, zeby watchdog ACTIVE
                                                 * nie strzelil na pierwszym oknie 104Hz */
        rtt_diag_printf(on ? "S6 inactive -> IDLE-CONNECTED" : "S6 activity -> ACTIVE");
        idle_cp_apply();                        /* negocjacja parametrow dla nowego stanu */
    }
    /* v0.4.2: idle_cp_apply TYLKO przy zmianie stanu — readback WAKE_UP_SRC kasuje
     * LIR => TOGGLE lapie deassert => drugi event z tym samym stanem; ponowny
     * change_conn_params w locie = NRF_ERROR_BUSY => cpfail roslo +1/przejscie
     * (log smoke 0.4.1). Reconnect w tym samym stanie: idle_cp_apply jawnie
     * w konsumencie g_conn_sync_req. */
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
            s_probe_cnt = 0;
            s_probing = true;
            nrf_drv_gpiote_in_config_t cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
            cfg.sense = pols[pol];
            if (nrf_drv_gpiote_in_init(pins[pi], &cfg, drdy_handler) != NRF_SUCCESS) {
                s_probing = false;
                lsm6dsl_drdy_enable(false);
                return false;
            }
            nrf_drv_gpiote_in_event_enable(pins[pi], true);
            /* 0.3.11: DRDY z BDU zalega HIGH dopoki nikt nie czyta OUT — w oknie probe
             * nikt nie czyta (polling jeszcze nie startuje) => zero rosnacych krawedzi
             * (false-negative wszystkich probe 0.3.1-0.3.10!). Drain-read co 10ms kasuje
             * DRDY => krawedz co ~9.6ms (104Hz). */
            for (uint8_t k = 0; k < 10; k++) {
                uint8_t drain[12];
                nrf_delay_ms(10);
                (void)lsm6dsl_read_motion(drain);
            }
            s_probing = false;
            uint16_t edges = s_probe_cnt;            /* 0.3.8: dedykowany licznik (FIFO 4 < prog 5) */
            if (edges >= 5) {
                g_diag.drdy_mode = (uint8_t)(pi * 2 + pol + 1);   /* 1/2 = P0.10 rising/falling
                                                                     (potwierdzony), 3/4 = P0.09 r/f */
                rtt_diag_printf("S2 DRDY ok pin=P0.%02u pol=%u edges=%u",
                                pins[pi], pol, edges);
                return true;                             /* handler juz nie pushuje (s_probing=false);
                                                            FIFO startuje czyste z petli glownej */
            }
            nrf_drv_gpiote_in_event_disable(pins[pi]);
            nrf_drv_gpiote_in_uninit(pins[pi]);
        }
    }
    lsm6dsl_drdy_enable(false);
    return false;
}

#if TRIKIG_PIN_SCAN
/* Debug: skan wszystkich wolnych GPIO pod katem krawedzi DRDY (INT2 ukladu wlaczony
 * na czas skanu). Wolne = wszystko procz: AIN2/4 + 12 (dzielnik), 5/6 (I2C), 21 (RESET),
 * 25 (BTN), 28 (LED). Pull-down na skanowanym pinie -> floaty ciche; DRDY push-pull
 * daje ~6 krawedzi / 60ms @104Hz. Wynik: RTT "S2 SCAN P0.xx edges=N". */
static void pin_scan(void)
{
    static const uint8_t skip[] = { 4u, 5u, 6u, 12u, 21u, 25u, 28u };
    /* 0.3.10: wlacz DRDY na OBU pinach INT ukladu (INT2_CTRL + INT1_CTRL) — nie wiemy
     * ktory fizycznie jest poprowadzony do nRF; 0.3.9 skanowal tylko z INT2_CTRL. */
    if (!lsm6dsl_drdy_enable(true)) {
        rtt_diag_printf("S2 SCAN: INT2 enable FAIL");
        return;
    }
    if (!lsm6dsl_drdy1_enable(true)) {
        rtt_diag_printf("S2 SCAN: INT1 enable FAIL");
        lsm6dsl_drdy_enable(false);
        return;
    }
    rtt_diag_printf("S2 PIN SCAN start (INT1+INT2 DRDY on)");
    for (uint8_t p = 0; p < 32; p++) {
        bool skip_pin = false;
        for (uint8_t i = 0; i < sizeof(skip); i++)
            if (skip[i] == p) { skip_pin = true; break; }
        if (skip_pin) continue;
        s_probe_cnt = 0;
        s_probing = true;
        nrf_drv_gpiote_in_config_t cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
        cfg.pull = NRF_GPIO_PIN_PULLDOWN;
        if (nrf_drv_gpiote_in_init(p, &cfg, drdy_handler) == NRF_SUCCESS) {
            nrf_drv_gpiote_in_event_enable(p, true);
            /* 0.3.11: drain-read — jak w probe (DRDY zalega HIGH bez odczytow) */
            for (uint8_t k = 0; k < 6; k++) {
                uint8_t drain[12];
                nrf_delay_ms(10);
                (void)lsm6dsl_read_motion(drain);
            }
            s_probing = false;
            nrf_drv_gpiote_in_event_disable(p);
            nrf_drv_gpiote_in_uninit(p);
            if (s_probe_cnt > 0)
                rtt_diag_printf("S2 SCAN P0.%02u edges=%u", p, s_probe_cnt);
        } else {
            s_probing = false;
        }
    }
    rtt_diag_printf("S2 PIN SCAN done");
    lsm6dsl_drdy_enable(false);
    lsm6dsl_drdy1_enable(false);
}
#endif

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

    /* C7: dt = t[n]-t[n-1] z timestampow DRDY; clamp min/max + dt_fault; gap > 60ms
     * => twardy ZUPT (reset integratora) i dt nominalny.
     * v0.4.0: dt ODR-aware — w IDLE (12.5Hz, okres 80ms) okno TIMER1 16-bit (65.5ms)
     * jest ZA KROTKIE na timestampy ISR => dt z RTC1 (ticks x 2 = q16.16 sekundy,
     * dokladne; nominal 80ms przy gapie), clamps 40-120ms, gap 200ms => twardy ZUPT.
     * Timestampy DRDY z ACTIVE (TIMER1) po wejsciu w IDLE nie sa uzywane. */
    uint16_t dt_q16 = g_idle_connected ? IDLE_DT_125HZ_Q16 : TRIKIG_VBT_DT_104HZ_Q16;

    /* detekcja gapow: RTC1 (app_timer, 30.5us/tick, wrap 512s) — niezalezna od 16-bit
     * okna TIMER1; gap => twardy ZUPT + dt nominalny (przegapiona probka nie wchodzi). */
    uint32_t now_rtc = NRF_RTC1->COUNTER;
    uint32_t gap_ticks = g_idle_connected ? DT_IDLE_GAP_TICKS : DT_GAP_TICKS;
    if (s_rtc_valid) {
        uint32_t rtc_diff = (now_rtc - s_last_rtc) & 0xFFFFFFu;
        if (rtc_diff > gap_ticks) {
            g_diag.dt_faults++;
            vbt_reset_velocity();
            s_prev_ts_valid = false;             /* dt z timer-ow po gapie nie liczy sie */
        }
    }
    s_last_rtc = now_rtc;
    s_rtc_valid = true;

    if (ts_valid && g_diag.drdy_mode != 0 && !g_idle_connected) {
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
    } else if (g_idle_connected) {
        /* IDLE: dt z RTC1 (ticks x 4 => q16.16, dokladne przy 16384Hz); clamp na
         * tickach. ts_cyc ignorowane (okno TIMER1 za krotkie dla 80ms). */
        if (s_prev_ts_valid) {
            uint32_t dt_ticks = (now_rtc - s_prev_rtc_base) & 0xFFFFFFu;
            if (dt_ticks < DT_IDLE_MIN_TICKS || dt_ticks > DT_IDLE_MAX_TICKS) {
                g_diag.dt_faults++;
                dt_ticks = (dt_ticks < DT_IDLE_MIN_TICKS) ? DT_IDLE_MIN_TICKS : DT_IDLE_MAX_TICKS;
            }
            dt_q16 = (uint16_t)(dt_ticks * 4u);  /* 16384 Hz: 2^16/2^14 = 4 (dokladne) */
        }
        s_prev_rtc_base = now_rtc;
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
    } else if (g_idle_connected) {
        /* v0.4.0: watchdog DRDY w IDLE na RTC1 — TIMER1 16-bit (65.5ms) NIE miesci
         * okresu 12.5Hz (80ms). Prog 160ms = 2x okres; zwolnienie zadan w main
         * (kolejnosc single-producer jak dotad). */
        if (((NRF_RTC1->COUNTER - s_last_rtc) & 0xFFFFFFu) > IDLE_WDT_TICKS) {
            if (s_fallback_req < 255) s_fallback_req++;
        }
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
            /* v0.4.1: kapsel mogl wejsc w inactivity w trakcie przerwy (HW niezalezny),
             * ale readback WAKE_UP_SRC = I2C — NIE w kontekscie SWI. Zadanie do main. */
            g_conn_sync_req = true;
            nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            /* v0.4.0: koniec sesji — flaga IDLE w dol (gyro/ODR zostaja w HW: gdy
             * kapsel lezy, oszczednosc jest wlasciwa takze bez polaczenia; sleep
             * 300s liczony jak dotad). Conn params bez restore — brak polaczenia. */
            idle_connected_set(false);
            /* v0.5.0: safety — rozlaczenie konczy serie; bez tego train ON zostalby
             * na zawsze (HW 104Hz bez IDLE = strata pradu). Zadanie do main (I2C). */
            g_train_req = 0;
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
        case 0x18:                       /* v0.5.0 training mode: 01=ON (IDLE off), 00=OFF */
            if (len >= 3) g_train_req = (d[2] == 0x01) ? 1 : 0;
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
#if TRIKIG_PIN_SCAN
    pin_scan();
#endif
#if TRIKIG_DRDY_PROBE
    /* 0.3.11: probe z powrotem domyslnie ON — INT1=P0.09, INT2=P0.10 potwierdzone
     * pomiarem plyty [P]; wczesniejszy "brak krawedzi" byl false-negative skanu/probe
     * (DRDY zalega HIGH bez odczytow — patrz komentarz w oknie probe). */
    if (m_imu_ok && nrf_drv_gpiote_init() == NRF_SUCCESS && drdy_probe()) {
        /* DRDY aktywny: probki procesowane w main loop z timestampow ISR */
    } else {
        rtt_diag_printf("S2 DRDY off -> polling");
    }
#endif
    /* v0.4.0 IDLE-CONNECTED: nasluch INT1/P0.09 (SLEEP_CHANGE) — TYLKO po probe
     * (drdy_probe inicjalizuje GPIOTE wlasnymi kanalami i sam probuje P0.09 —
     * wczesniejsza rejestracja = konflikt kanalu). IDLE tylko przy zywym DRDY
     * (drdy_mode != 0): fallback polling (zwykle awaryjny) zostaje bez IDLE.
     * Polaryzacja TOGGLE — SLEEP_CHANGE zmienia poziom przy KAZDEJ zmianie stanu
     * (inaktywnosc/activity), a wlasciwy stan i tak rozstrzyga readback
     * WAKE_UP_SRC w petli glownej (D-017: nie ufac polaryzacji krawedzi). */
    if (m_imu_ok && g_diag.drdy_mode != 0) {
        nrf_drv_gpiote_in_config_t act_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
        if (nrf_drv_gpiote_in_init(PIN_IMU_INT1, &act_cfg, activity_handler) == NRF_SUCCESS) {
            nrf_drv_gpiote_in_event_enable(PIN_IMU_INT1, true);
            if (lsm6dsl_inactivity_enable(true)) {
                rtt_diag_printf("S2 idle-detect ON (INT1/P0.09)");
            } else {
                nrf_drv_gpiote_in_event_disable(PIN_IMU_INT1);
                rtt_diag_printf("S2 idle-detect CFG FAIL (IDLE off)");
            }
        } else {
            rtt_diag_printf("S2 idle-detect GPIOTE FAIL (IDLE off)");
        }
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

        /* v0.4.1: sync stanu IDLE po CONNECTED (zadanie ze SWI, I2C w main).
         * Pierwszy na purpose: przy connect kapsel mogl juz lezec w inactivity —
         * host od pierwszej ramki dostaje wlasciwe ODR/conn params. Przy stanie
         * bez zmiany cp i tak trzeba odswiezyc (nowe polaczenie = nowe params). */
        if (g_conn_sync_req) {
            g_conn_sync_req = false;
            bool sleeping = false;
            if (lsm6dsl_inact_state(&sleeping)) {
                if (sleeping == g_idle_connected) idle_cp_apply();
                else                             idle_connected_set(sleeping);
            }
        }

        /* v0.5.0 training mode: RX 0x20 0x18 (01=ON, 00=OFF). ON = IDLE wylaczony
         * na czas serii — klasa ruchu dip/pullup 140-246mg p95 < WK_THS 250mg [P]
         * (trening 2026-09-03: 21-36% pokrycia ramek, HW spi w dominujacej czesci
         * serii; Wy Hevy: "usypianie za szybko wpływa na kalibrację po starcie
         * serii"). Auto-OFF przy disconnect (safety). */
        if (g_train_req >= 0) {
            bool on = (g_train_req != 0);
            g_train_req = -1;
            rtt_diag_printf("S6 train %s", on ? "ON (idle off)" : "OFF");
            g_diag.train_mode = on ? 1u : 0u;
            if (lsm6dsl_inactivity_enable(!on) && on) {
                bool sleeping = false;
                if (lsm6dsl_inact_state(&sleeping)) {
                    if (sleeping && lsm6dsl_wake_force()) {
                        idle_connected_set(false);   /* HW wyszedl z low-power */
                    } else if (!sleeping && g_idle_connected) {
                        idle_connected_set(false);   /* HW juz zywy, FW nie skoczyl */
                    }
                }
                vbt_reset();                         /* czysta kalibracja na start serii */
            }
        }

        /* v0.4.0: konsumpcja zmian stanu activity/inactivity (INT1 edge). Sync ze
         * stanem HW przez readback WAKE_UP_SRC (krawedz mówi ZE sie zmienilo, nie
         * DOKAD — D-017: nie ufac polaryzacji). Odczyt kasuje LIR => kolejna zmiana
         * wygeneruje kolejny poziom na INT1. */
        if (g_activity_event) {
            g_activity_event = false;
            bool sleeping = false;
            if (lsm6dsl_inact_state(&sleeping)) {
                idle_connected_set(sleeping);
            }
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
            /* ~1s wg ODR: 104Hz w ACTIVE (dzielnik 104), ~12.5Hz w IDLE (dzielnik 12) */
            uint16_t log_div = g_idle_connected ? IDLE_ODR_HZ : IMU_ODR_HZ;
            if (++vbt_log_div >= log_div) {
                vbt_log_div = 0;
                rtt_diag_printf("VBT v=%d mm/s mv=%u dup=%u idle=%u", (int)vbt_velocity_mms(), (unsigned)vbt_moving(), (unsigned)s_dup_count, (unsigned)g_diag.idle_state);
                diag_print();                    /* C1: liczniki dropow + okresy + timingi */
            }
#endif
        }
        (void)sd_app_evt_wait();
    }
}
