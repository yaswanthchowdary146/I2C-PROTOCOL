// ============================================================
// MODULE : i2c_top
// PURPOSE: Connects 2 masters and 4 slaves on a shared I2C bus
//
// OPEN-DRAIN BUS MODEL:
//   Each module has sda_out and scl_out.
//   0 = actively pulling the wire LOW
//   1 = releasing (pull-up resistor holds wire HIGH)
//
//   The shared bus wire = AND of all drivers:
//     If ANY driver drives 0 → bus = 0
//     Only if ALL drivers release (1) → bus = 1
//
// ┌──────────────────────────────────────────────────────┐
// │                    i2c_top                           │
// │                                                      │
// │  ┌──────────┐    ┌──────────┐                       │
// │  │ master_0 │    │ master_1 │   ← 2 masters         │
// │  └────┬─────┘    └────┬─────┘                       │
// │       │               │                              │
// │  ─────┴───────────────┴────── SDA bus (wired-AND)   │
// │  ─────┬───────────────┬────── SCL bus (wired-AND)   │
// │       │               │                              │
// │  ┌────┴─┐ ┌────┐ ┌────┴─┐ ┌────┐                   │
// │  │ S0   │ │ S1 │ │  S2  │ │ S3 │  ← 4 slaves       │
// │  │ 0x10 │ │0x20│ │ 0x30 │ │0x40│                    │
// │  └──────┘ └────┘ └──────┘ └────┘                    │
// └──────────────────────────────────────────────────────┘
// ============================================================

module i2c_top (
    input wire clk,
    input wire rst
);

// ── Shared bus wires ─────────────────────────────────────────
wire sda;   // the shared SDA line everyone reads
wire scl;   // the shared SCL line everyone reads

// ── Individual driver outputs from each module ───────────────
wire sda_m0, scl_m0;   // master 0's drivers
wire sda_m1, scl_m1;   // master 1's drivers
wire sda_s0, scl_s0;   // slave 0's drivers
wire sda_s1, scl_s1;   // slave 1's drivers
wire sda_s2, scl_s2;   // slave 2's drivers
wire sda_s3, scl_s3;   // slave 3's drivers

// ── Open-drain bus resolution (Wired-AND) ────────────────────
// Bus is 0 if anyone drives 0. Bus is 1 only when all release.
assign sda = (sda_m0 & sda_m1 & sda_s0 & sda_s1 & sda_s2 & sda_s3);
assign scl = (scl_m0 & scl_m1 & scl_s0 & scl_s1 & scl_s2 & scl_s3);

// ── Testbench control signals for each master ─────────────────
wire [6:0] addr_m0,  addr_m1;
wire [7:0] data_m0,  data_m1;
wire       start_m0, start_m1;
wire       done_m0,  done_m1;
wire       arb_m0,   arb_m1;
wire [7:0] rd_m0,    rd_m1;

// ── Master 0 ─────────────────────────────────────────────────
// BACKOFF = 8  → master 0 waits  8 cycles after losing arbitration
i2c_master #(.MASTER_ID(0), .BACKOFF(8)) master_0 (
    .clk      (clk),
    .rst      (rst),
    .addr     (addr_m0),
    .data_wr  (data_m0),
    .start_tx (start_m0),
    .data_rd  (rd_m0),
    .done     (done_m0),
    .arb_lost (arb_m0),
    .sda_in   (sda),        // reads the shared bus
    .scl_in   (scl),
    .sda_out  (sda_m0),     // drives its piece of the bus
    .scl_out  (scl_m0)
);

// ── Master 1 ─────────────────────────────────────────────────
// BACKOFF = 20 → master 1 waits 20 cycles (DIFFERENT from master 0!)
// This asymmetry prevents them from retrying at the same instant
i2c_master #(.MASTER_ID(1), .BACKOFF(20)) master_1 (
    .clk      (clk),
    .rst      (rst),
    .addr     (addr_m1),
    .data_wr  (data_m1),
    .start_tx (start_m1),
    .data_rd  (rd_m1),
    .done     (done_m1),
    .arb_lost (arb_m1),
    .sda_in   (sda),
    .scl_in   (scl),
    .sda_out  (sda_m1),
    .scl_out  (scl_m1)
);

// ── Slave 0 — address 0x10, NO clock stretching ──────────────
i2c_slave #(.MY_ADDR(7'h10), .SLAVE_ID(0), .STRETCH_CYC(0)) slave_0 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s0), .scl_out(scl_s0)
);

// ── Slave 1 — address 0x20, NO clock stretching ──────────────
i2c_slave #(.MY_ADDR(7'h20), .SLAVE_ID(1), .STRETCH_CYC(0)) slave_1 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s1), .scl_out(scl_s1)
);

// ── Slave 2 — address 0x30, CLOCK STRETCHING = 10 cycles ─────
// This slave is "slow" — it stretches the clock after address ACK
i2c_slave #(.MY_ADDR(7'h30), .SLAVE_ID(2), .STRETCH_CYC(10)) slave_2 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s2), .scl_out(scl_s2)
);

// ── Slave 3 — address 0x40, CLOCK STRETCHING = 6 cycles ─────
i2c_slave #(.MY_ADDR(7'h40), .SLAVE_ID(3), .STRETCH_CYC(6)) slave_3 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s3), .scl_out(scl_s3)
);

endmodule
