# Garuda 6-Step CK — Sensorless BLDC ESC Firmware

Sensorless 6-step trapezoidal BLDC motor controller firmware for **dsPIC33CK64MP205 + ATA6847** gate driver (Microchip EV43F54A evaluation board).

Built from scratch as part of **Project Garuda** — an open-source drone ESC platform.

---

## Features

### Core Motor Control
- **Sensorless 6-step trapezoidal commutation** using ATA6847 built-in BEMF comparators
- **Full speed range**: 10k-102k eRPM (1,400-14,600 mechanical RPM on A2212)
- **Two motor profiles**: A2212 1400KV (7PP, drone) and Hurst DMB2424B10002 (5PP, bench)
- **Automatic startup**: alignment -> open-loop ramp -> closed-loop handoff
- **Linear timing advance**: 0-20deg configurable, speed-dependent

### High-Resolution ZC Detection (Hybrid Architecture)
- **100kHz SCCP1 fast poll timer** — primary ZC detection in closed-loop
- **640ns SCCP4 hardware commutation timer** — 78x better resolution than Timer1
- **Adaptive deglitch filter**: FL:3-8, scales with speed, hysteretic on decel
- **Bounce-tolerant filtering**: soft decrement on noise instead of hard reset
- **Verified: 0.02% false ZC rate** at 100k eRPM

### Current Sensing & Protection
- **2-shunt phase current measurement** (Ia, Ib via OA2/OA3, 3mOhm shunts, Gt=16)
- **Step-aware IBus reconstruction**: Ic = -(Ia+Ib) for Phase C commutation steps
- **ATA6847 hardware current limit**: cycle-by-cycle gate chopping (non-latching, drone-safe)
- **nIRQ fault monitoring**: VDS short circuit, overcurrent, over-temperature, supply UV
- **Vbus OV/UV monitoring** with debounce

### GSP Binary Protocol (GUI-Ready)
- **Protocol v2**: `[0x02][LEN][CMD][PAYLOAD][CRC16]`, 115200 baud
- **Board detection**: `boardId=0x0002` (EV43F54A) — GUI auto-selects CK panels
- **48-byte CK snapshot**: state, Ia/Ib/IBus, Vbus, eRPM, ZC diagnostics, ILIM status
- **10-100Hz telemetry streaming** with configurable rate
- **Remote motor control**: start/stop/fault-clear via serial commands
- **9 feature flags**: IC_ZC, ATA6847, ILIM_HW, CURRENT_SNS, GSP, and more

### Diagnostics & Tooling
- **Debug UART console** (115200 baud): 15+ interactive commands
- **Verbose telemetry mode**: 100ms status with 30+ fields including currents
- **Python tools**:
  - `gsp_ck_test.py` — GSP protocol test + live motor run with telemetry
  - `zc_capture.py` — CSV telemetry logging with summary statistics

---

## Verified Performance

Tested on A2212 1400KV at 14.4V with prop:

| Metric | Value |
|--------|-------|
| Speed range | 10,272 - 102,124 eRPM |
| IC false ZC rate | **0.02%** (3 out of 15,551) |
| Missed ZCs | **0** |
| Desync events | **0** |
| TpHR jitter at max speed | **2.0%** |
| Duty-to-speed linearity | R^2 = 0.999989 |
| GSP telemetry rate | 20Hz (confirmed) |
| Peak phase current (prop) | 8.3A instantaneous |
| ILIM chopping | Verified, motor keeps running |

---

## Getting Started

### Prerequisites

- **MPLAB X IDE** v6.20+ ([download](https://www.microchip.com/mplab/mplab-x-ide))
- **XC16 Compiler** v2.10 ([download](https://www.microchip.com/xc16))
- **DFP**: dsPIC33CK-MP_DFP (install via MPLAB X Pack Manager)
- **Python 3.8+** with `pyserial` (for test tools)

### Build with MPLAB X IDE

1. Clone this repo
2. Open MPLAB X -> File -> Open Project
3. Navigate to the cloned directory (it's an `.X` project)
4. Select `default` configuration
5. Clean and Build

### Build from CLI (recommended)

```bash
make -f Makefile-cli.mk clean
make -f Makefile-cli.mk
```

Output: `dist/default/production/garuda_6step_ck.X.production.hex`

### Test GSP Protocol

Set `FEATURE_GSP` to `1` in `garuda_config.h`, rebuild, flash, then:

```bash
pip install pyserial
python3 tools/gsp_ck_test.py -p /dev/ttyACM0
```

### Test with Motor Running

```bash
python3 tools/gsp_ck_test.py -p /dev/ttyACM0 --motor -d 30
```

### Debug UART Console (FEATURE_GSP=0)

Connect at 115200 baud, type `h` for help. Verbose telemetry with `v`.

```bash
python3 tools/zc_capture.py -p /dev/ttyACM0 -d 30 -o capture.csv
```

---

## Architecture

```
 Pot/DShot -> Throttle -> Duty Map -> PWM (20kHz, center-aligned)
                                         |
                                  ATA6847 Gate Driver
                                   |              |
                            3-Phase Inverter    ILIM Chopping
                                   |           (cycle-by-cycle)
                            BEMF Comparators
                                   |
              +--------------------+--------------------+
              |                                         |
    SCCP1 Fast Poll (100kHz)              ADC ISR Backup (20kHz)
    Adaptive deglitch FL:3-8              Fixed filter, OL primary
              |                                         |
              +--------------------+--------------------+
                                   |
                    SCCP4 HR Commutation Timer (640ns)
                                   |
                    6-Step Commutation + Current Sensing
                                   |
                         GSP Protocol (UART1)
                                   |
                    GUI / Python Test Tools
```

---

## Project Structure

```
garuda-6step-ck/
+-- main.c                    Entry point, init sequence, main loop
+-- garuda_config.h           Configuration, motor profiles, feature flags
+-- garuda_types.h            Core data structures
+-- garuda_service.h/c        State machine, all ISRs
+-- Makefile-cli.mk           CLI build (recommended)
+-- motor/
|   +-- bemf_zc.h/c           ZC detection, scheduling, fast poll, timeout
|   +-- commutation.h/c       6-step table, phase state application
|   +-- startup.h/c           Alignment and open-loop ramp
+-- hal/
|   +-- hal_pwm.h/c           PWM init, duty, commutation step
|   +-- hal_timer1.h/c        Timer1 (50us tick)
|   +-- hal_ic.h/c            SCCP1 fast poll timer (100kHz)
|   +-- hal_com_timer.h/c     SCCP4 HR commutation timer (640ns)
|   +-- hal_adc.h/c           ADC (currents, pot, Vbus)
|   +-- hal_ata6847.h/c       Gate driver control (ILIM, faults, CSA)
|   +-- hal_uart.h/c          UART (debug or GSP stub)
|   +-- hal_trap.c            Trap/exception handlers
|   +-- (and more...)
+-- gsp/
|   +-- gsp.h/c               Protocol engine (ring buffers, parser, CRC)
|   +-- gsp_commands.h/c      Command handlers (PING, INFO, SNAPSHOT, etc.)
|   +-- gsp_snapshot.h/c      48-byte CK telemetry snapshot
+-- tools/
|   +-- gsp_ck_test.py        GSP protocol test + motor run
|   +-- zc_capture.py         Debug UART telemetry capture
+-- docs/
|   +-- firmware_architecture.md   Complete technical reference
|   +-- ata6847_feature_roadmap.md ATA6847 feature enablement plan
|   +-- production_esc_plan.md     Full production ESC roadmap
+-- nbproject/                MPLAB X project files
```

---

## Documentation

- **[Firmware Architecture](docs/firmware_architecture.md)** — Complete technical reference with flowcharts, API docs, timing analysis
- **[ATA6847 Feature Roadmap](docs/ata6847_feature_roadmap.md)** — 10 prioritized gate driver features
- **[Production ESC Plan](docs/production_esc_plan.md)** — Full roadmap: feature matrix vs AM32/BLHeli/VESC, 4-phase plan, testing strategy

---

## Roadmap

### Phase 1: Flight-Ready (6 weeks)
- [ ] Software PD current limiting (AM32-style)
- [ ] DShot 300/600 input
- [ ] RPM-dependent duty ramp
- [ ] GUI CK dashboard + parameter editor
- [ ] GSP parameter system

### Phase 2: Robustness (6 weeks)
- [ ] Motor auto-detect (Rs/Ls/Ke)
- [ ] EEPROM/flash persistence
- [ ] DShot bidirectional telemetry
- [ ] Soft current/thermal derating
- [ ] Demag compensation

### Phase 3: Production (6 weeks)
- [ ] Bootloader (no-debugger firmware update)
- [ ] 4-way FC passthrough (Betaflight compatible)
- [ ] KISS telemetry
- [ ] Production test mode

### Phase 4: Advanced
- [ ] FOC sensorless (port from AK board)
- [ ] CAN bus / DroneCAN
- [ ] Custom PCB design

---

## License

This project is open source. See individual files for applicable licenses.

---

*Part of [Project Garuda](https://github.com/bhanuprakashjh/ProjectGaruda) — Open-source drone ESC platform*
