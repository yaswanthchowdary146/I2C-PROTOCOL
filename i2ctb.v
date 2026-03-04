

`timescale 1ns/1ps
`include "i2cmaster.v"
`include "i2cslave.v"
`include "i2ctop.v"

module i2c_tb;

// ── Clock and reset ──────────────────────────────────────────
reg clk, rst;
initial clk = 0;
always #5 clk = ~clk;   // 10ns period = 100MHz

// ── Signals to drive masters ─────────────────────────────────
reg [6:0] addr_m0,  addr_m1;
reg [7:0] data_m0,  data_m1;
reg       start_m0, start_m1;

// ── Signals from masters ─────────────────────────────────────
wire       done_m0,  done_m1;
wire       arb_m0,   arb_m1;
wire [7:0] rd_m0,    rd_m1;

// ── Latching done flags ───────────────────────────────────────
// done_m0/done_m1 are 1-cycle pulses. These regs latch them so
// wait() in the initial block never misses a pulse.
reg done_m0_latch, done_m1_latch;
reg clr_latch;   // pulse high for 1 cycle to clear both latches

// ── Bus visibility ────────────────────────────────────────────
wire sda, scl;

// ── Instantiate top module ───────────────────────────────────
// We need to connect our testbench signals into the top module.
// The top module internally connects masters and slaves.
// Here we use a slightly modified approach: instantiate everything
// directly in the testbench for cleaner signal driving.

// Master driver signals
wire sda_m0_drv, scl_m0_drv;
wire sda_m1_drv, scl_m1_drv;
wire sda_s0_drv, scl_s0_drv;
wire sda_s1_drv, scl_s1_drv;
wire sda_s2_drv, scl_s2_drv;
wire sda_s3_drv, scl_s3_drv;

// Open-drain bus
assign sda = sda_m0_drv & sda_m1_drv & sda_s0_drv & sda_s1_drv & sda_s2_drv & sda_s3_drv;
assign scl = scl_m0_drv & scl_m1_drv & scl_s0_drv & scl_s1_drv & scl_s2_drv & scl_s3_drv;

// ── Master 0 (BACKOFF=8 cycles) ───────────────────────────────
i2c_master #(.MASTER_ID(0), .BACKOFF(8)) master_0 (
    .clk(clk), .rst(rst),
    .addr(addr_m0), .data_wr(data_m0), .start_tx(start_m0),
    .data_rd(rd_m0), .done(done_m0), .arb_lost(arb_m0),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_m0_drv), .scl_out(scl_m0_drv)
);

// ── Master 1 (BACKOFF=20 cycles — different! prevents re-collision) ──
i2c_master #(.MASTER_ID(1), .BACKOFF(20)) master_1 (
    .clk(clk), .rst(rst),
    .addr(addr_m1), .data_wr(data_m1), .start_tx(start_m1),
    .data_rd(rd_m1), .done(done_m1), .arb_lost(arb_m1),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_m1_drv), .scl_out(scl_m1_drv)
);

// ── Slave 0 — 0x10 — no stretching ───────────────────────────
i2c_slave #(.MY_ADDR(7'h10), .SLAVE_ID(0), .STRETCH_CYC(0)) slave_0 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s0_drv), .scl_out(scl_s0_drv)
);

// ── Slave 1 — 0x20 — no stretching ───────────────────────────
i2c_slave #(.MY_ADDR(7'h20), .SLAVE_ID(1), .STRETCH_CYC(0)) slave_1 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s1_drv), .scl_out(scl_s1_drv)
);

// ── Slave 2 — 0x30 — CLOCK STRETCHING 10 cycles ─────────────
i2c_slave #(.MY_ADDR(7'h30), .SLAVE_ID(2), .STRETCH_CYC(10)) slave_2 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s2_drv), .scl_out(scl_s2_drv)
);

// ── Slave 3 — 0x40 — CLOCK STRETCHING 6 cycles ──────────────
i2c_slave #(.MY_ADDR(7'h40), .SLAVE_ID(3), .STRETCH_CYC(6)) slave_3 (
    .clk(clk), .rst(rst),
    .sda_in(sda), .scl_in(scl),
    .sda_out(sda_s3_drv), .scl_out(scl_s3_drv)
);

// ── Helper task: send one transaction from a master ──────────
task send_master0;
    input [6:0] a;
    input [7:0] d;
    begin
        addr_m0  = a;
        data_m0  = d;
        start_m0 = 1;
        @(posedge clk);
        start_m0 = 0;
        // Wait for done or arb_lost
        wait(done_m0 || arb_m0);
        @(posedge clk);
    end
endtask

task send_master1;
    input [6:0] a;
    input [7:0] d;
    begin
        addr_m1  = a;
        data_m1  = d;
        start_m1 = 1;
        @(posedge clk);
        start_m1 = 0;
        wait(done_m1 || arb_m1);
        @(posedge clk);
    end
endtask

task wait_cycles;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i+1)
            @(posedge clk);
    end
endtask

// ─────────────────────────────────────────────────────────────
// MAIN TEST SEQUENCE
// ─────────────────────────────────────────────────────────────
initial begin
    // Setup
    rst       = 1;
    start_m0  = 0;
    start_m1  = 0;
    addr_m0   = 0; data_m0 = 0;
    addr_m1   = 0; data_m1 = 0;
    clr_latch = 0;

    @(posedge clk);
    @(posedge clk);
    rst = 0;
    @(posedge clk);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  TEST 1: Master 0 writes 0xAB to Slave 0 (addr 0x10)");
    $display("  Expected: Clean transaction, no arbitration, no stretch");
    $display("========================================================");

    send_master0(7'h10, 8'hAB);
    wait_cycles(5);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  TEST 2: Master 1 writes 0xCD to Slave 1 (addr 0x20)");
    $display("  Expected: Clean transaction, no arbitration, no stretch");
    $display("========================================================");

    send_master1(7'h20, 8'hCD);
    wait_cycles(5);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  TEST 3: ARBITRATION TEST");
    $display("  Both masters start at EXACTLY the same clock edge.");
    $display("  Master 0 → slave 0x10 (address = 0001 0000)");
    $display("  Master 1 → slave 0x20 (address = 0010 0000)");
    $display("  Master 0 has lower address → Master 0 should WIN.");
    $display("  Master 1 should LOSE and retry after backoff.");
    $display("========================================================");

    // Load both masters simultaneously
    addr_m0 = 7'h10; data_m0 = 8'hBB;
    addr_m1 = 7'h20; data_m1 = 8'hCC;

    // Clear latches before triggering so we catch fresh done pulses
    clr_latch = 1; @(posedge clk); clr_latch = 0;

    // Trigger BOTH at the exact same clock edge
    start_m0 = 1;
    start_m1 = 1;
    @(posedge clk);
    start_m0 = 0;
    start_m1 = 0;

    // Wait for both to finish using latches — never misses a pulse
    wait(done_m0_latch);
    $display("[TB] Master 0 transaction DONE.");
    wait(done_m1_latch);
    $display("[TB] Master 1 transaction DONE (after retry).");
    wait_cycles(5);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  TEST 4: CLOCK STRETCHING TEST");
    $display("  Master 0 writes 0x55 to Slave 2 (addr 0x30)");
    $display("  Slave 2 is SLOW — it stretches clock for 10 cycles.");
    $display("  Master 0 must WAIT during stretching.");
    $display("========================================================");

    send_master0(7'h30, 8'h55);
    wait_cycles(5);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  TEST 5: CLOCK STRETCHING TEST");
    $display("  Master 1 writes 0x77 to Slave 3 (addr 0x40)");
    $display("  Slave 3 stretches clock for 6 cycles.");
    $display("========================================================");

    send_master1(7'h40, 8'h77);
    wait_cycles(5);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  TEST 6: ARBITRATION + CLOCK STRETCHING TOGETHER");
    $display("  Master 0 → slave 0x30 (has clock stretching)");
    $display("  Master 1 → slave 0x20 (no clock stretching)");
    $display("  Both start simultaneously.");
    $display("  Master 1 (0x20 > 0x10 in address) may lose arbitration");
    $display("  OR may win depending on bit pattern — watch the output!");
    $display("========================================================");

    addr_m0 = 7'h30; data_m0 = 8'hEE;
    addr_m1 = 7'h20; data_m1 = 8'hFF;

    // Clear latches before triggering so we catch fresh done pulses
    clr_latch = 1; @(posedge clk); clr_latch = 0;

    start_m0 = 1;
    start_m1 = 1;
    @(posedge clk);
    start_m0 = 0;
    start_m1 = 0;

    wait(done_m0_latch);
    $display("[TB] Master 0 finished Test 6.");
    wait(done_m1_latch);
    $display("[TB] Master 1 finished Test 6.");
    wait_cycles(10);

    // ============================================================
    $display("");
    $display("========================================================");
    $display("  ALL TESTS COMPLETE");
    $display("========================================================");
    $display("");

    $finish;
end

// ── Monitor: print bus activity ──────────────────────────────
// Watches SDA and SCL and prints whenever they change
// so you can follow the raw bus state too
reg sda_prev_mon, scl_prev_mon;
always @(posedge clk) begin
    sda_prev_mon <= sda;
    scl_prev_mon <= scl;

    // Detect START (SDA falls while SCL=1)
    if (!sda && sda_prev_mon && scl)
        $display("[BUS t=%0t] *** START condition on bus ***", $time);

    // Detect STOP (SDA rises while SCL=1)
    if (sda && !sda_prev_mon && scl)
        $display("[BUS t=%0t] *** STOP condition on bus ***", $time);
end

// ── Latch always block ───────────────────────────────────────
// Captures 1-cycle done pulses; cleared by rst or by clr_latch pulse
always @(posedge clk or posedge rst) begin
    if (rst || clr_latch) begin
        done_m0_latch <= 0;
        done_m1_latch <= 0;
    end else begin
        if (done_m0) done_m0_latch <= 1;
        if (done_m1) done_m1_latch <= 1;
    end
end

// ── Waveform dump ─────────────────────────────────────────────
initial begin
    $dumpfile("i2c_wave.vcd");
    $dumpvars(0, i2c_tb);
end

endmodule
