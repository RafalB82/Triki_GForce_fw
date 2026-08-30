# Triki_GForce_fw — plan wdrożenia optymalizacji MCU i VBT

**Branch bazowy:** `feature/nrf52-adc-sensor`  
**Cel:** poprawa poprawności fizycznej VBT, deterministyczności próbkowania oraz wykorzystania zasobów nRF52 bez niepotrzebnego przenoszenia logiki treningowej do firmware.

## 1. Architektura docelowa

```text
LSM6DSL
  │
  ├── RAW ──────────────────────────────► BLE / host
  │
  ▼
ACC + GYRO
  │
  ▼
gravity estimator
  │
  ▼
linear acceleration
  │
  ▼
movement axis
  │
  ▼
movement acceleration
  │
  ▼
velocity
  │
  ▼
phase / basic events
  │
  ▼
rep metrics
```

Firmware ma być lekkim, deterministycznym front-endem VBT. Klasyfikacja ćwiczeń i rozbudowana analityka treningowa pozostają po stronie hosta.

## 2. Priorytety

| ID | Zadanie | Priorytet | Cel |
|---|---|---:|---|
| VBT-01 | Gravity tracking ACC+GYRO | **P0** | poprawna kompensacja grawitacji |
| VBT-02 | Movement-axis velocity | **P0** | odejście od sztywnego `Vx` |
| VBT-03 | Poprawne `dt` / timing | **P0** | poprawna integracja |
| MCU-01 | DRDY zamiast polling | **P1** | synchronizacja z IMU |
| MCU-02 | TWIM zamiast bit-bang I²C | **P1** | redukcja CPU/jitteru |
| BLE-01 | BLE batching | **P1** | redukcja notifications |
| MCU-03 | Ring 4 → 16 | **P1** | większy margines bufora |
| VBT-04 | Phase FSM | **P1** | ECC/CON/rep |
| VBT-05 | MCV/MPV/ROM/duration | **P1** | metryki VBT |
| MCU-04 | FIFO IMU | **P2** | odporność przy wyższym ODR |
| MCU-05 | LTO benchmark | **P2** | dodatkowa optymalizacja |
| BUILD-01 | Cleanup Makefile | **P3** | uproszczenie builda |
| DSP-01 | Optymalizacja `isqrt32()` | **P3** | tylko po profilowaniu |

## 3. VBT-01 — Gravity tracking ACC+GYRO

### Problem

Obecny model:

```text
linear_acc = LPF(acc) - bias
```

traktuje `bias[]` jako stały wektor grawitacji. Po zmianie orientacji sensora kierunek grawitacji w układzie XYZ się zmienia, więc do integracji trafia pozorne przyspieszenie.

Obecne `|ACC| ≈ g` służy do REST i nie rozwiązuje tego problemu.

### Implementacja

Dodać lekki fixed-pointowy gravity estimator:

```text
gyro
  ↓
predykcja gravity
  ↓
ACC correction
  ↓
gravity estimate
```

Następnie:

```text
linear_acc = acc - gravity
```

Minimalny stan może obejmować:

```c
int32_t gravity_x;
int32_t gravity_y;
int32_t gravity_z;
```

Bez float, FPU, dynamicznej alokacji i pełnego AHRS, jeśli nie jest potrzebny.

### Akceptacja

Przy obrocie sensora bez translacji:

```text
velocity ≈ 0
```

bez narastającego błędu od zmiany orientacji.

## 4. VBT-02 — Movement axis

### Problem

Obecnie velocity jest liczone tylko dla osi X:

```text
velocity = Vx
```

To ogranicza użycie urządzenia.

### Implementacja

Wprowadzić wektor:

```c
movement_axis_x;
movement_axis_y;
movement_axis_z;
```

i:

```text
a_move = dot(linear_acc, movement_axis)
v_move += a_move * dt
```

Na pierwszym etapie oś powinna być stała dla danej sesji, ustalona podczas kalibracji lub przez host. Adaptację można dodać później.

### Akceptacja

Ten sam ruch przy różnych orientacjach sensora daje zbliżone:

```text
velocity
ROM
MCV
MPV
```

## 5. VBT-03 — Poprawne `dt`

### Problem

Występują równocześnie:

```text
timer ≈ 9 ms
IMU ODR = 104 Hz
integracja ≈ 9.6 ms
```

Duplicate detection maskuje część problemu, ale nie daje rzeczywistego czasu próbki.

### Implementacja

Preferowane:

```text
IMU DRDY
  ↓
timestamp
  ↓
sample
```

i:

```c
dt = timestamp[n] - timestamp[n-1];
```

Jeżeli timestampowanie zostanie odłożone:

```text
dt = 1 / ODR
```

ale tylko po zapewnieniu jednej rzeczywistej próbki na krok.

### Akceptacja

- velocity niezależne od jitteru timera,
- brak potrzeby `memcmp()` do synchronizacji,
- poprawne odstępy próbek.

## 6. MCU-01 — DRDY

Obecnie:

```text
APP_TIMER → poll → I²C read
```

Docelowo:

```text
LSM6DSL INT/DRDY → nowa próbka → I²C/TWIM
```

Korzyści:

- brak sztucznego polling rate,
- brak duplikatów wynikających z timera,
- mniejszy jitter,
- poprawny timing,
- łatwiejsze przejście do wyższego ODR.

Każdy `seq` powinien odpowiadać jednej rzeczywistej próbce IMU.

## 7. MCU-02 — TWIM

Obecny bit-bang I²C jest głównym kandydatem do redukcji CPU.

Docelowo:

```text
bit-bang I²C → TWIM
```

z burst read:

```text
OUTX_L_G → 12 B
```

### Pomiary przed/po

```text
CPU/sample
I²C transaction time
jitter
BLE drops
ring drops
```

Nie wdrażać na ślepo bez potwierdzenia możliwości sprzętowych użytego nRF52.

## 8. BLE-01 — batching

Przy wire v2 i MTU 247 można grupować próbki.

Przykład:

```text
10 × 19 B = 190 B
```

czyli około 10 próbek na notification.

Zamiast około:

```text
104 notifications/s
```

otrzymujemy około:

```text
10–11 notifications/s
```

Batching ma zmienić transport, nie semantykę RAW. Po stronie hosta musi być możliwe jednoznaczne odtworzenie kolejności po `seq`.

## 9. MCU-03 — Ring 4 → 16

Obecne:

```c
RING_SLOTS = 4
```

daje około 3 użyteczne sloty, czyli tylko około 29 ms zapasu przy 104 Hz.

Zmiana:

```c
RING_SLOTS = 16
```

kosztuje tylko kilkaset bajtów i daje znacznie większy margines.

Zasada pozostaje:

```text
sample
 ├──► VBT
 └──► RAW ring
          └── może zostać dropnięty
```

RAW drop nie może zatrzymać VBT.

Dodać jawny licznik:

```text
ring_drops
```

## 10. VBT-04 — Phase FSM

Dopiero po uzyskaniu poprawnego `linear_acc` i velocity:

```text
REST
  ↓
MOVING
  ↓
ECC
  ↓
TURN
  ↓
CON
  ↓
REP_END
```

Minimalne eventy:

```text
MOVEMENT_START
ECC_START
TURNING_POINT
CON_START
REP_END
REST
```

Event powinien zawierać co najmniej:

```text
seq
timestamp/sample index
velocity
flags
```

## 11. VBT-05 — Metryki treningowe

### Rep

```text
rep_index
rep_duration
ecc_duration
con_duration
ROM
```

### Velocity

```text
MCV
MPV
```

Definicje muszą być jednoznacznie zapisane w specyfikacji protokołu.

### Set

```text
rep_count
best_MCV
best_MPV
velocity_loss
```

Przykładowo:

```text
velocity_loss = 1 - current_MCV / best_MCV
```

Firmware dostarcza podstawowe, deterministyczne metryki. Interpretacja treningowa pozostaje po stronie hosta.

## 12. MCU-04 — FIFO

Nie wdrażać przed P0/P1.

Docelowo:

```text
LSM6DSL FIFO
  ↓
watermark
  ↓
burst read
  ↓
DSP
```

Szczególnie uzasadnione przy:

```text
ODR = 200 Hz
ODR = 400 Hz
```

FIFO zwiększa odporność na jitter i chwilowe obciążenie CPU/BLE.

## 13. MCU-05 — LTO

Wykonać benchmark:

```text
Build A: -Os
Build B: -Os -flto
```

Porównać:

```text
.text
.data
.bss
CPU/sample
binary size
```

Nie przechodzić automatycznie na `-O3`.

## 14. SAADC baterii

Pomiar baterii 1 Hz nie jest istotnym średnim obciążeniem dla VBT.

Pozostawić jako niezależny subsystem:

```text
IMU/VBT
   │
   └── SAADC 1 Hz
```

Obserwować przede wszystkim chwilowy jitter, nie średni CPU.

## 15. Instrumentacja

Przed optymalizacją i po każdej większej zmianie mierzyć:

```text
imu_samples
imu_duplicates
seq_gaps
ring_drops
ble_drops
vbt_events
```

oraz:

```text
sample_period_min
sample_period_max
sample_period_avg
```

Dla CPU:

```text
acq_time_us
dsp_time_us
ble_time_us
```

Jeżeli możliwe, mierzyć czas acquisition GPIO + oscyloskopem/logic analyzerem.

Nie optymalizować na podstawie intuicji.

## 16. Test plan

### A — REST

60 s bezruchu:

```text
velocity drift
false movement
false reps
seq
duplicates
```

Oczekiwane:

```text
velocity ≈ 0
no reps
no seq gaps
```

### B — Obrót bez translacji

Obracać sensor w miejscu.

Oczekiwane:

```text
brak istotnego narastania velocity
```

To jest kluczowy test gravity compensation.

### C — Kontrolowany ruch liniowy

Porównać:

```text
FW velocity
offline reference
```

Mierzyć:

```text
MAE
max error
drift
peak velocity
```

### D — Różne orientacje sensora

Ten sam ruch przy różnych orientacjach.

Porównać:

```text
MCV
MPV
ROM
rep duration
```

### E — BLE przeciążenie

Celowo wywołać opóźnienia BLE.

Sprawdzić:

```text
VBT events
ring_drops
seq gaps
```

VBT nie może zależeć od skuteczności wysyłania RAW.

### F — ODR 104/200/400 Hz

Dla każdego:

```text
sample rate
CPU
RAM
ring drops
BLE drops
seq gaps
```

## 17. Kryteria zakończenia

### P0 — poprawność

- [ ] gravity compensation działa przy zmianie orientacji,
- [ ] velocity nie dryfuje istotnie podczas samego obrotu,
- [ ] movement axis nie jest sztywno związana z X,
- [ ] `dt` odpowiada rzeczywistemu próbkowaniu.

### P1 — zasoby

- [ ] CPU/sample zmierzone,
- [ ] I²C time zmierzone,
- [ ] BLE notifications ograniczone przez batching,
- [ ] ring ma wystarczający zapas,
- [ ] BLE nie blokuje VBT.

### P1 — VBT

- [ ] ECC/CON wykrywane,
- [ ] turning point wykrywany,
- [ ] rep boundary stabilny,
- [ ] MCV,
- [ ] MPV,
- [ ] ROM,
- [ ] duration.

### P2 — skalowanie

- [ ] FIFO,
- [ ] ODR 200 Hz,
- [ ] ODR 400 Hz,
- [ ] LTO benchmark.

## 18. Kolejność commitów

```text
C1  instrumentacja timing/CPU/counters
C2  gravity estimator ACC+GYRO
C3  linear acceleration
C4  movement-axis projection
C5  poprawne dt
C6  testy offline VBT
C7  DRDY
C8  TWIM
C9  ring 16
C10 BLE batching
C11 phase FSM
C12 rep metrics
C13 set metrics
C14 FIFO
C15 LTO benchmark
C16 cleanup
```

Każdy commit:

```text
build
flash
smoke test
regression test
```

Zmiany algorytmiczne dodatkowo:

```text
offline comparison
```

## 19. Zasada nadrzędna

Priorytet:

```text
1. poprawność fizyczna próbek
2. deterministyczny timing
3. poprawność velocity
4. stabilność rep detection
5. CPU/RAM
6. BLE efficiency
7. micro-optimizations
```

**Najważniejsza decyzja:** nRF52 ma produkować wiarygodne dane fizyczne i podstawowe metryki VBT, a nie wykonywać całą inteligencję treningową. RAW pozostaje dostępny do dalszej analizy.
