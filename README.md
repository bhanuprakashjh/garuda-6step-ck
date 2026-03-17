# Garuda 6-Step CK — Sensorless BLDC ESC Firmware

Sensorless 6-step trapezoidal BLDC motor controller firmware for **dsPIC33CK64MP205 + ATA6847** gate driver (Microchip EV43F54A evaluation board).

Built from scratch as part of **Project Garuda** — an open-source drone ESC platform.

---

## Features

### Core Motor Control
- **Sensorless 6-step trapezoidal commutation** using ATA6847 built-in BEMF comparators
- **Full speed range**: 10k–102k eRPM (1,400–14,600 mechanical RPM on A2212)
- **Two motor profiles**: A2212 1400KV (7PP, drone) and Hurst DMB2424B10002 (5PP, bench)
- **Automatic startup**: alignment → open-loop ramp → closed-loop handoff
- **Linear timing advance**: 0–20° configurable, speed-dependent

### High-Resolution ZC Detection (Hybrid Architecture)
- **100kHz SCCP1 fast poll timer** — primary ZC detection in closed-loop
- **640ns SCCP4 hardware commutation timer** — 78x better resolution than Timer1
- **Adaptive deglitch filter**: FL:3–8, scales with speed, hysteretic on decel
- **Bounce-tolerant filtering**: soft decrement on noise instead of hard reset
- **Zero-timestamp candidate recording**: eliminates filter delay from scheduling
- **Verified: 0.02% false ZC rate** at 100k eRPM (3 false out of 15,551 accepted)

### Robust Operation
- **Zero desync** across full pot sweep (idle → max → idle), verified 30s runs
- **Forced step recovery**: 12.5% period increment per forced step prevents runaway
- **Automatic desync recovery**: up to 3 restart attempts before latching fault
- **Asymmetric duty slew**: fast up (20ms), slow down (17ms) for smooth throttle
- **Vbus monitoring**: OV/UV fault detection with debounce

### Hardware Protection
- **Trap handlers**: address error, stack overflow, math error, oscillator fail
- **ATA6847 fault monitoring**: SPI-based register readback
- **PWM kill on any fault**: immediate output disable from any ISR context
- **Dead-time enforced**: 50ns hardware dead-time on all phases

### Diagnostics & Tooling
- **Debug UART console** (115200 baud): 15+ interactive commands
- **Verbose telemetry mode**: 100ms status with 30+ fields
- **Python capture tool** (`tools/zc_capture.py`): CSV logging with summary statistics
- **CLI build support**: `Makefile-cli.mk` for headless builds

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| MCU | dsPIC33CK64MP205 | 100 MIPS, 64KB flash, 8KB RAM |
| Gate Driver | ATA6847 | 3-phase, BEMF comparators, SPI control |
| Board | EV43F54A | Microchip eval board with USB-UART |
| Motor (tested) | A2212 1400KV | 7 pole pairs, 12V, drone outrunner |
| Motor (tested) | Hurst DMB2424B10002 | 5 pole pairs, 24V, bench motor |
| Supply | 12–14.4V | 3S LiPo or bench supply |

### Pin Map

| Function | Pin | Peripheral |
|----------|-----|------------|
| PWM Phase A (H/L) | RB10/RB11 | PWM Generator 1 (Master) |
| PWM Phase B (H/L) | RB12/RB13 | PWM Generator 2 (Slave) |
| PWM Phase C (H/L) | RB14/RB15 | PWM Generator 3 (Slave) |
| BEMF Comparator A/B/C | RC6/RC7/RD10 | GPIO (from ATA6847) |
| SPI (SDI/SCK/SDO/nCS) | RC4/RC5/RC10/RC11 | SPI1 → ATA6847 |
| Debug UART (RX/TX) | RC8/RC9 | UART1 (115200 baud) |
| Pot (speed ref) | AN6 | ADC shared channel |
| Vbus monitor | AN9 | ADC shared channel |

---

## Architecture

```
 Pot ──> ADC ──> Duty Map ──> PWM (20kHz, center-aligned)
                                 |
                          ATA6847 Gate Driver
                                 |
                          3-Phase Inverter ──> BLDC Motor
                                 |
                          BEMF Comparators (RC6/RC7/RD10)
                                 |
              +------------------+------------------+
              |                                     |
    SCCP1 Fast Poll (100kHz)             ADC ISR Backup (20kHz)
    Adaptive deglitch FL:3-8             Fixed filter, OL primary
              |                                     |
              +------------------+------------------+
                                 |
                    SCCP4 HR Commutation Timer (640ns)
                                 |
                    6-Step Commutation Table
```

### ISR Priority Map

| Priority | ISR | Rate | Purpose |
|----------|-----|------|---------|
| 6 | SCCP4 Compare | On-demand | Commutation execution (640ns) |
| 5 | SCCP1 Period | 100kHz | ZC fast poll + deglitch |
| 4 | Timer1 | 20kHz | State machine, forced step timeout |
| 3 | ADC Complete | 20kHz | Sensor reads, backup ZC, Vbus faults |

### State Machine

```
IDLE ──> ARMED (200ms) ──> ALIGN (150ms) ──> OL_RAMP (~1s)
  ^                                              |
  |                                   zcSynced + rampDone
  |                                              v
  +──── RECOVERY (200ms) <──── FAULT <──── CLOSED_LOOP
         (auto-restart ×3)     (12 misses)    (pot control)
```

---

## Verified Performance

Tested on A2212 1400KV at 14.4V, 30-second capture:

| Metric | Value |
|--------|-------|
| Speed range | 10,272 – 102,124 eRPM |
| Mechanical RPM range | 1,467 – 14,589 |
| IC false ZC rate | **0.02%** (3 out of 15,551) |
| Missed ZCs | **0** |
| Desync events | **0** |
| TpHR jitter at max speed | **2.0%** (153–156 HR ticks) |
| ZC interval alternation | 0.6 mean diff (balanced) |
| Duty-to-speed linearity | R² = 0.999989 |
| Vbus sag at full load | 1.2% |
| Effective advance angle | ~3° electrical |
| Commutation quality | **9.0 / 10** |

---

## Getting Started

### Prerequisites

- **MPLAB X IDE** v6.20+ ([download](https://www.microchip.com/mplab/mplab-x-ide))
- **XC16 Compiler** v2.10 ([download](https://www.microchip.com/xc16))
- **DFP**: dsPIC33CK-MP_DFP (install via MPLAB X Pack Manager)
- **Python 3.8+** with `pyserial` (for capture tool)

### Build with MPLAB X IDE

1. Clone this repo
2. Open MPLAB X → File → Open Project
3. Navigate to the cloned directory (it's an `.X` project)
4. Select `default` configuration
5. Build (F11) or Build & Program (Ctrl+F5)

### Build from CLI

```bash
make -f Makefile-cli.mk clean
make -f Makefile-cli.mk
```

Output: `dist/default/production/garuda_6step_ck.X.production.hex`

### Motor Profile Selection

Edit `garuda_config.h`:
```c
#define MOTOR_PROFILE   1   // 0=Hurst, 1=A2212
```

### Run Telemetry Capture

```bash
pip install pyserial
python3 tools/zc_capture.py -p /dev/ttyACM0 -d 30 -o capture.csv
```

### UART Debug Console

Connect at 115200 baud, type `h` for help:

| Command | Description |
|---------|-------------|
| `v` | Toggle verbose telemetry (100ms) |
| `s` | Full state dump |
| `a` | ATA6847 register dump |
| `b` | BEMF comparator states |
| `d` | ADC readings (pot, Vbus) |
| `p` | PWM register dump |
| `x` | IC ZC statistics |
| `f` | Read/clear ATA6847 faults |

---

## Project Structure

```
garuda-6step-ck/
├── main.c                    Entry point, init sequence, main loop
├── garuda_config.h           Configuration, motor profiles, timing constants
├── garuda_types.h            Core data structures
├── garuda_service.h/c        State machine, all ISRs
├── Makefile-cli.mk           CLI build (no IDE required)
├── motor/
│   ├── bemf_zc.h/c           ZC detection, scheduling, fast poll, timeout
│   ├── commutation.h/c       6-step table, phase state application
│   └── startup.h/c           Alignment and open-loop ramp
├── hal/
│   ├── clock.h/c             PLL configuration (200MHz)
│   ├── port_config.h/c       GPIO, PPS, analog setup
│   ├── hal_pwm.h/c           PWM init, duty, commutation step
│   ├── hal_timer1.h/c        Timer1 (50µs tick)
│   ├── hal_ic.h/c            SCCP1 fast poll timer (100kHz)
│   ├── hal_com_timer.h/c     SCCP4 HR commutation timer (640ns)
│   ├── hal_adc.h/c           ADC (currents, pot, Vbus)
│   ├── hal_opa.h/c           Internal op-amps
│   ├── hal_spi.h/c           SPI for ATA6847
│   ├── hal_ata6847.h/c       Gate driver control
│   ├── hal_uart.h/c          Debug UART
│   ├── hal_diag.h/c          Diagnostic commands
│   ├── hal_clc.h/c           CLC D-FF blanking (optional)
│   ├── hal_trap.c            Trap/exception handlers
│   └── board_service.h/c     LED/button handling
├── tools/
│   └── zc_capture.py         Telemetry capture → CSV with analysis
├── docs/
│   └── firmware_architecture.md  Complete architecture document
└── nbproject/                MPLAB X project files
```

---

## Documentation

- **[Firmware Architecture](docs/firmware_architecture.md)** — Complete technical reference with flowcharts, API docs, timing analysis, and data structures

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| 100kHz poll vs edge-triggered IC | ATA6847 comparators are GPIO pins, not IC-compatible. Polling at 100kHz gives 10µs resolution with adaptive deglitch. |
| SCCP4 HR timer for commutation | Timer1 at 50µs has ±50% jitter at Tp:2 (100k eRPM). SCCP4 at 640ns gives 2% jitter. |
| Fire ASAP when HR target is past | At high speed, filter+compute latency exceeds ZC-to-comm delay. Immediate fire via HR timer prevents falling to Timer1 path. |
| FL:3 minimum deglitch | FL:2 allowed 50% false IC rate at 100k eRPM. FL:3 drops to 0.02% while fitting in post-blanking window. |
| Bounce-tolerant filter (decrement) | Hard reset on mismatch loses entire filter at Tp:2 where only ~8 polls are available. Soft decrement costs 2 polls. |
| IIR step period (75/25) | Smooths ZC interval noise. Combined with 2-step averaging and 1.5x rejection gate for robustness. |
| Duty slew asymmetric | Fast up for throttle response, slow down to prevent BEMF window collapse during decel. |

---

## Roadmap

- [ ] Prop-loaded testing (8×4.5 on A2212)
- [ ] DShot protocol input (SCCP4 IC on RD8)
- [ ] Current limiting (phase current ADC + software limit)
- [ ] Temperature monitoring
- [ ] EEPROM parameter storage
- [ ] GUI parameter tuning (React + WebSerial)
- [ ] Production board design (custom PCB)

---

## License

This project is open source. See individual files for applicable licenses.

---

## Acknowledgments

- **Microchip** — dsPIC33CK reference designs, AN1292, EV43F54A board
- **AM32 project** — IIR filter and deglitch inspiration
- **Project Garuda** — parent project for drone ESC development

---

*Part of [Project Garuda](https://github.com/bhanuprakashjh/ProjectGaruda) — Open-source drone ESC platform*
