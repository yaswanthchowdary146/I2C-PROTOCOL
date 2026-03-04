# I2C-PROTOCOL
# I2C Protocol — Full RTL Design & Verilog Verification

> A complete, from-scratch RTL implementation of the I2C protocol in Verilog.  
> Covers **multi-master arbitration**, **clock stretching**, **wired-AND bus modeling**, and a full **self-checking testbench** — all simulated and verified in pure Verilog.

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
- [I2C Concepts Implemented](#i2c-concepts-implemented)
  - [Frame Format & MSB-First Transmission](#1-frame-format--msb-first-transmission)
  - [Wired-AND Bus Logic](#2-wired-and-bus-logic)
  - [Clock Stretching](#3-clock-stretching)
  - [Multi-Master Arbitration](#4-multi-master-arbitration)
- [Module Details](#module-details)
  - [i2c_master](#i2c_master)
  - [i2c_slave](#i2c_slave)
  - [i2c_top](#i2c_top)
  - [i2c_tb (Testbench)](#i2c_tb-testbench)
- [Test Scenarios](#test-scenarios)
- [How to Simulate](#how-to-simulate)
- [Key Design Decisions](#key-design-decisions)

---

## Overview

This project implements the **I2C (Inter-Integrated Circuit)** serial communication protocol at the RTL level using Verilog. The design includes:

- **2 Masters** — both capable of initiating transactions independently
- **4 Slaves** — each with a unique 7-bit address; two with configurable clock stretching
- **Full bus arbitration** — lossless multi-master conflict resolution
- **Clock stretching** — slave-driven SCL hold with master wait logic
- **Open-drain wired-AND bus** — accurate hardware modeling in simulation
- **Pure Verilog testbench** — 6 test scenarios, no SystemVerilog, no UVM

---

## Project Structure

```
i2c-verilog/
│
├── i2cmaster.v       # I2C Master FSM — START, address, data, ACK, arbitration, stretching
├── i2cslave.v        # I2C Slave FSM — address detection, clock stretching, ACK, data receive
├── i2ctop.v          # Top-level wrapper — connects 2 masters + 4 slaves on shared bus
├── i2ctb.v           # Testbench — 6 test scenarios, bus monitor, waveform dump
└── README.md
```

---

## Architecture

```
                        ┌─────────────────────────────────────────────┐
                        │                  i2c_top                    │
                        │                                             │
                        │   ┌──────────┐       ┌──────────┐          │
                        │   │ Master 0 │       │ Master 1 │          │
                        │   │BACKOFF=8 │       │BACKOFF=20│          │
                        │   └────┬─────┘       └────┬─────┘          │
                        │        │                   │                │
                        │  ──────┴───────────────────┴────── SDA     │
                        │  ──────┬───────────────────┬────── SCL     │
                        │        │  (Wired-AND Bus)   │              │
                        │  ┌─────┴┐ ┌──────┐ ┌──────┐┴─────┐        │
                        │  │  S0  │ │  S1  │ │  S2  │  S3  │        │
                        │  │ 0x10 │ │ 0x20 │ │ 0x30 │ 0x40 │        │
                        │  │NoStretch NoStretch Str=10 Str=6 │        │
                        │  └──────┘ └──────┘ └──────┴──────┘        │
                        └─────────────────────────────────────────────┘
```

**Bus equation (open-drain wired-AND):**
```verilog
assign sda = sda_m0 & sda_m1 & sda_s0 & sda_s1 & sda_s2 & sda_s3;
assign scl = scl_m0 & scl_m1 & scl_s0 & scl_s1 & scl_s2 & scl_s3;
```

---

## I2C Concepts Implemented

### 1. Frame Format & MSB-First Transmission

Every I2C transaction follows this exact sequence on the bus:

```
 ┌───────┬───────────────────────┬─────┬─────┬───────────────────────┬─────┬──────┐
 │ START │  A6 A5 A4 A3 A2 A1 A0│ R/W │ ACK │  D7 D6 D5 D4 D3 D2 D1 D0 │ ACK │ STOP │
 │       │  ←──── MSB first ────│     │     │  ←──── MSB first ────     │     │      │
 └───────┴───────────────────────┴─────┴─────┴───────────────────────┴─────┴──────┘
```

| Phase         | Description                                                        |
|---------------|--------------------------------------------------------------------|
| **START**     | SDA falls LOW while SCL is HIGH — signals bus is claimed           |
| **Address**   | 7-bit slave address, transmitted **MSB first** (A6 → A0)           |
| **R/W bit**   | 0 = Write, 1 = Read                                                |
| **ACK**       | Slave pulls SDA LOW to acknowledge; HIGH = NACK                    |
| **Data**      | 8-bit payload, also **MSB first** (D7 → D0)                        |
| **STOP**      | SDA rises HIGH while SCL is HIGH — bus is released                 |

**MSB-first in RTL:**  
The shift register is loaded as `shift <= {addr, 1'b0}` and the master drives `sda_out <= shift[7]`, shifting left each clock — bit 7 always goes first.

---

### 2. Wired-AND Bus Logic

I2C uses **open-drain** outputs. No device ever actively drives HIGH — instead, a pull-up resistor holds the line HIGH by default. Any device can pull LOW at any time.

```
 Device A output:  ──┐  (open-drain)
 Device B output:  ──┤  (open-drain)     ┌─── VCC
 Device C output:  ──┘  (open-drain)     │
                       └────────────────[R]─── Bus line
```

**Result:**
```
Bus = A AND B AND C AND ...
```

- If **any** device drives LOW → entire bus is LOW
- Bus goes HIGH **only** when **all** devices release

This single property enables both clock stretching and arbitration without any dedicated arbitration logic.

---

### 3. Clock Stretching

A slow slave can pause the master mid-transaction by holding SCL LOW after the master releases it.

```
 SCL (master releases): ─────────────────────╮    ╭──────────────────────
                                              │    │
 SCL (slave holds low): ──────────────────────╯────╯  (slave stretches here)
                                              ├────┤
                                           STRETCH period

 Master state:          ... SCL_RISE ──► WAIT_STRETCH ──► SCL_HIGH ...
                                         (master waits)
```

**How it works in RTL:**
1. Master raises `scl_out = 1` (releases SCL)
2. Master samples `scl_in` — if still LOW, slave is stretching
3. Master enters `S_WAIT_STRETCH` and polls until `scl_in == 1`
4. Transaction resumes from where it paused — no data loss

**Slave configuration:**

| Slave | Address | `STRETCH_CYC` |
|-------|---------|---------------|
| S0    | `0x10`  | 0 (no stretch)|
| S1    | `0x20`  | 0 (no stretch)|
| S2    | `0x30`  | 10 cycles     |
| S3    | `0x40`  | 6 cycles      |

---

### 4. Multi-Master Arbitration

When two masters start a transaction simultaneously, I2C resolves the conflict **losslessly** — the winning master never even knows a collision happened.

```
 Master 0 SDA:  1 1 0 0 0 1 0 ...   (address 0x10 = 0001 0000)
 Master 1 SDA:  1 1 0 0 1 0 0 ...   (address 0x20 = 0010 0000)
                        ↑
                  Bus = AND → 0   ← Master 1 drove 1, reads 0 → LOST

 Bus SDA:       1 1 0 0 0 ...       (Master 0's bits dominate)
```

**Arbitration FSM flow (losing master):**

```
 SEND_BIT ──► SCL_HIGH
                 │
                 ▼  sda_out==1 but sda_in==0
             ARB_WAIT_STOP ──► (watch for STOP on bus)
                 │
                 ▼  STOP detected
             ARB_BACKOFF ──► (count BACKOFF cycles)
                 │
                 ▼  backoff complete
             WAIT_BUS ──► retry transaction
```

**Asymmetric backoff** prevents re-collision:
- Master 0: `BACKOFF = 8` cycles
- Master 1: `BACKOFF = 20` cycles

---

## Module Details

### `i2c_master`

**Parameters:**

| Parameter   | Default | Description                              |
|-------------|---------|------------------------------------------|
| `MASTER_ID` | 0       | Display identifier for `$display` logs  |
| `BACKOFF`   | 10      | Cycles to wait after losing arbitration  |

**Ports:**

| Port       | Dir | Width | Description                        |
|------------|-----|-------|------------------------------------|
| `clk`      | in  | 1     | System clock                       |
| `rst`      | in  | 1     | Synchronous reset                  |
| `addr`     | in  | 7     | Target slave address               |
| `data_wr`  | in  | 8     | Data byte to write                 |
| `start_tx` | in  | 1     | Pulse high for 1 cycle to initiate |
| `done`     | out | 1     | 1-cycle pulse — transaction done   |
| `arb_lost` | out | 1     | 1-cycle pulse — arbitration lost   |
| `sda_in`   | in  | 1     | Shared SDA bus (read)              |
| `scl_in`   | in  | 1     | Shared SCL bus (read)              |
| `sda_out`  | out | 1     | Master's SDA driver                |
| `scl_out`  | out | 1     | Master's SCL driver                |

**Master FSM States:**

```
S_IDLE → S_WAIT_BUS → S_START_0 → S_START_1
       → S_SEND_BIT → S_SDA_SETUP → S_SCL_RISE → S_SCL_HIGH
       → S_ACK_RELEASE → S_ACK_SETUP → S_ACK_RISE → S_ACK_SAMPLE
       → S_STOP_0 → S_STOP_1 → S_STOP_2 → S_IDLE

Clock stretch path:  S_SCL_HIGH → S_WAIT_STRETCH → S_SCL_HIGH
Arbitration path:    S_SCL_HIGH → S_ARB_WAIT_STOP → S_ARB_BACKOFF → S_WAIT_BUS
```

---

### `i2c_slave`

**Parameters:**

| Parameter    | Default  | Description                            |
|--------------|----------|----------------------------------------|
| `MY_ADDR`    | `7'h10`  | 7-bit I2C address of this slave        |
| `SLAVE_ID`   | 0        | Display identifier                     |
| `STRETCH_CYC`| 0        | Clock stretch cycles (0 = disabled)    |

**Slave FSM States:**

```
IDLE → RECV_ADDR → ADDR_DONE → [STRETCHING] → ACK_ADDR_0 → ACK_ADDR_1 → ACK_ADDR_2
     → RECV_DATA → DATA_DONE → ACK_DATA_0 → ACK_DATA_1 → ACK_DATA_2 → DONE
```

The slave uses **edge detection** on `scl_in` and `sda_in` to detect START, STOP, rising edges, and falling edges each clock cycle.

---

### `i2c_top`

Top-level wrapper connecting all modules on the shared bus. Instantiates:
- 2 masters with different BACKOFF values
- 4 slaves with unique addresses and stretch configurations
- Wired-AND bus assignment

---

### `i2c_tb` (Testbench)

Clock: **100 MHz** (`#5` half-period)  
Bus model: Wired-AND `assign` statements — accurate open-drain simulation  
Done detection: Latched `done` flags to never miss 1-cycle pulses  
Waveform dump: `$dumpfile("i2c_wave.vcd")` — open with GTKWave  

---

## Test Scenarios

| Test | Masters       | Target Slave | Data   | Feature Tested                        |
|------|---------------|--------------|--------|---------------------------------------|
| 1    | Master 0 only | S0 (`0x10`)  | `0xAB` | Basic write, clean transaction        |
| 2    | Master 1 only | S1 (`0x20`)  | `0xCD` | Basic write, clean transaction        |
| 3    | Both (simultaneous) | S0 & S1 | `0xBB`, `0xCC` | **Multi-master arbitration** |
| 4    | Master 0 only | S2 (`0x30`)  | `0x55` | **Clock stretching** (10 cycles)      |
| 5    | Master 1 only | S3 (`0x40`)  | `0x77` | **Clock stretching** (6 cycles)       |
| 6    | Both (simultaneous) | S2 & S1 | `0xEE`, `0xFF` | **Arbitration + Stretching** |

**Expected outcomes:**
- Tests 1 & 2: Clean ACK, `done` asserted, no `arb_lost`
- Test 3: One master wins, other asserts `arb_lost`, waits for STOP, backs off, retries successfully
- Tests 4 & 5: Master stalls in `WAIT_STRETCH`, resumes after slave releases SCL, data correct
- Test 6: Arbitration and stretching interact correctly — both masters eventually complete

---

## How to Simulate

### Using Icarus Verilog (iverilog)

```bash
# Compile
iverilog -o i2c_sim i2ctb.v

# Run simulation
vvp i2c_sim

# View waveforms (requires GTKWave)
gtkwave i2c_wave.vcd
```

### Using any Verilog simulator

The testbench uses only standard Verilog-2001 constructs:
- `$display`, `$dumpfile`, `$dumpvars`, `$finish`
- `task`, `always`, `initial`
- No SystemVerilog, no UVM, no proprietary extensions

---

## Key Design Decisions

**One action per FSM state** — Each state does exactly one thing (drive a signal, wait for a condition, sample a value). This makes timing violations impossible and the waveform easy to trace.

**SDA changes only while SCL is LOW** — Except for START and STOP, SDA is only modified in states where `scl_out = 0`. This is enforced structurally by the FSM, not by logic conditions.

**Releasing SDA=1 while SCL=0 is always safe** — The only dangerous SDA transitions are while SCL=1. The FSM guarantees all SDA releases happen while SCL is low, preventing false STOP conditions.

**Asymmetric backoff** — Masters use different `BACKOFF` values so they never retry at the same clock edge, eliminating the possibility of infinite re-collision.

**STOP detection before retry** — The losing master does not retry immediately. It watches the bus for the winner's STOP condition before even starting its backoff counter. This ensures the bus is truly idle before any retry attempt.

---

