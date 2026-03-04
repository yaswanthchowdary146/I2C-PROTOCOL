

module i2c_master #(
    parameter MASTER_ID = 0,
    parameter BACKOFF   = 10
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [6:0] addr,
    input  wire [7:0] data_wr,
    input  wire       start_tx,
    output reg  [7:0] data_rd,
    output reg        done,
    output reg        arb_lost,
    input  wire       sda_in,
    input  wire       scl_in,
    output reg        sda_out,
    output reg        scl_out
);

// ── States ───────────────────────────────────────────────────
localparam [4:0]
    S_IDLE         = 0,
    S_WAIT_BUS     = 1,
    S_START_0      = 2,   // SDA falls while SCL=1 → START
    S_START_1      = 3,   // SCL falls, data phase begins
    S_SEND_BIT     = 4,   // SCL=0, drive SDA to next bit
    S_SDA_SETUP    = 5,   // SCL=0, SDA stable — 1 cycle before SCL rises
    S_SCL_RISE     = 6,   // release SCL, wait 1 cycle to settle
    S_SCL_HIGH     = 7,   // SCL=1: check stretch/arb, then SCL=0
    S_WAIT_STRETCH = 8,   // slave holding SCL low
    S_ACK_RELEASE  = 9,   // SCL=0, release SDA=1 while SCL is low (safe)
    S_ACK_SETUP    = 10,  // SCL=0, SDA=1 stable — 1 cycle setup
    S_ACK_RISE     = 11,  // release SCL=1
    S_ACK_SAMPLE   = 12,  // SCL=1: sample SDA, then SCL=0
    S_STOP_0       = 13,  // SCL=0, SDA=0
    S_STOP_1       = 14,  // SCL=1, SDA=0
    S_STOP_2       = 15,  // SCL=1, SDA=1 → STOP
    // ARB_WAIT has two sub-phases:
    //   phase 0: wait for STOP on bus (winner finishes)
    //   phase 1: count backoff cycles, then retry
    S_ARB_WAIT_STOP    = 16, // watching bus for STOP condition
    S_ARB_BACKOFF      = 17, // STOP seen, now counting backoff
    S_ACK_WAIT_STRETCH = 18; // slave stretching clock during ACK phase

reg [4:0] st;
reg [7:0] shift;
reg [2:0] bit_idx;
reg [7:0] arb_cnt;
reg       sending_addr;

// ── STOP detector (to observe winner finishing) ──────────────
// STOP = SDA rises while SCL has been high for ≥1 cycle
reg sda_prev, scl_prev;
always @(posedge clk) begin
    sda_prev <= sda_in;
    scl_prev <= scl_in;
end
wire stop_on_bus = sda_in & ~sda_prev & scl_in & scl_prev;

// ─────────────────────────────────────────────────────────────
always @(posedge clk or posedge rst) begin
    if (rst) begin
        st           <= S_IDLE;
        sda_out      <= 1;
        scl_out      <= 1;
        done         <= 0;
        arb_lost     <= 0;
        bit_idx      <= 7;
        arb_cnt      <= 0;
        sending_addr <= 1;
        data_rd      <= 0;
        shift        <= 0;
    end
    else begin
        done     <= 0;
        arb_lost <= 0;

        case (st)

        // ── IDLE ─────────────────────────────────────────────
        S_IDLE: begin
            sda_out <= 1;
            scl_out <= 1;
            if (start_tx) begin
                $display("[M%0d] ==== WRITE 0x%02h -> slave 0x%02h ====",
                          MASTER_ID, data_wr, addr);
                st <= S_WAIT_BUS;
            end
        end

        // ── WAIT_BUS ─────────────────────────────────────────
        // Do not proceed until BOTH lines are confirmed high.
        // Hold here as long as bus is busy.
        S_WAIT_BUS: begin
            sda_out <= 1;
            scl_out <= 1;
            if (sda_in == 1 && scl_in == 1) begin
                $display("[M%0d] Bus free. Generating START.", MASTER_ID);
                shift        <= {addr, 1'b0};   // {addr[6:0], W=0}
                bit_idx      <= 7;
                sending_addr <= 1;
                st           <= S_START_0;
            end
        end

        // ── START ────────────────────────────────────────────
        // S_START_0: SDA=0 while SCL=1 — this IS the START condition
        // S_START_1: SCL=0 — data phase begins
        S_START_0: begin
            sda_out <= 0;   // SDA falls while SCL=1 = START
            scl_out <= 1;
            st      <= S_START_1;
        end

        S_START_1: begin
            scl_out <= 0;   // SCL falls, now we can clock bits
            st      <= S_SEND_BIT;
        end

        // ── BIT CLOCK LOOP ───────────────────────────────────
        // SEND_BIT  → SDA_SETUP → SCL_RISE → SCL_HIGH → (repeat or ACK)
        //
        // SEND_BIT : change SDA (SCL is low — safe)
        // SDA_SETUP: hold SDA stable for 1 cycle before SCL rises
        //            (prevents false START/STOP if SDA changed to 1)
        // SCL_RISE : release SCL=1, wait 1 cycle before checking scl_in
        // SCL_HIGH : observe bus: stretch? arb? or next bit.

        S_SEND_BIT: begin
            sda_out <= shift[7];   // MSB first
            st      <= S_SDA_SETUP;
        end

        S_SDA_SETUP: begin
            // SCL stays 0. SDA is now stable.
            // Nothing to do except wait 1 cycle.
            st <= S_SCL_RISE;
        end

        S_SCL_RISE: begin
            scl_out <= 1;   // release SCL
            st      <= S_SCL_HIGH;
        end

        S_SCL_HIGH: begin
            if (scl_in == 0) begin
                // Slave is holding SCL low (clock stretching)
                $display("[M%0d] Clock stretch. Waiting...", MASTER_ID);
                st <= S_WAIT_STRETCH;
            end
            else if (sda_out == 1 && sda_in == 0) begin
                // We drove 1 but bus shows 0 — another master won
                $display("[M%0d] *** ARBITRATION LOST *** Waiting for STOP before retry.",
                          MASTER_ID);
                sda_out  <= 1;
                scl_out  <= 1;
                arb_cnt  <= 0;
                arb_lost <= 1;
                st       <= S_ARB_WAIT_STOP;
            end
            else begin
                // Bit clocked successfully
                scl_out <= 0;                    // pull SCL low
                shift   <= {shift[6:0], 1'b0};   // shift out sent bit

                if (bit_idx == 0) begin
                    $display("[M%0d] 8 bits sent. Getting ACK...", MASTER_ID);
                    st <= S_ACK_RELEASE;
                end
                else begin
                    bit_idx <= bit_idx - 1;
                    st      <= S_SEND_BIT;
                end
            end
        end

        S_WAIT_STRETCH: begin
            if (scl_in == 1) begin
                $display("[M%0d] Stretch over. Resuming.", MASTER_ID);
                st <= S_SCL_HIGH;
            end
        end

        // ── ACK RECEIVE ──────────────────────────────────────
        // Master releases SDA, clocks one pulse, reads sda_in.
        //
        // S_ACK_RELEASE: SCL=0, sda_out=1 (released while SCL low — safe)
        // S_ACK_SETUP  : SCL=0, SDA=1 stable
        // S_ACK_RISE   : SCL=1
        // S_ACK_SAMPLE : read sda_in, pull SCL=0

        S_ACK_RELEASE: begin
            sda_out <= 1;   // release SDA while SCL=0 (releasing 1 while SCL=0 is safe)
            scl_out <= 0;
            st      <= S_ACK_SETUP;
        end

        S_ACK_SETUP: begin
            st <= S_ACK_RISE;
        end

        S_ACK_RISE: begin
            scl_out <= 1;
            // Check if slave is holding SCL low (clock stretching during ACK phase)
            if (scl_in == 0) begin
                $display("[M%0d] Clock stretch during ACK. Waiting...", MASTER_ID);
                st <= S_ACK_WAIT_STRETCH;
            end
            else begin
                st <= S_ACK_SAMPLE;
            end
        end

        S_ACK_WAIT_STRETCH: begin
            if (scl_in == 1) begin
                $display("[M%0d] Stretch over (ACK phase). Resuming.", MASTER_ID);
                st <= S_ACK_SAMPLE;
            end
        end

        S_ACK_SAMPLE: begin
            scl_out <= 0;   // SCL low after reading

            if (sda_in == 0) begin
                // ACK
                if (sending_addr) begin
                    $display("[M%0d] ACK for address. Sending data 0x%02h.", MASTER_ID, data_wr);
                    sending_addr <= 0;
                    shift        <= data_wr;
                    bit_idx      <= 7;
                    st           <= S_SEND_BIT;
                end
                else begin
                    $display("[M%0d] ACK for data. DONE!", MASTER_ID);
                    $display("[M%0d] >>> Slave 0x%02h received 0x%02h <<<",
                              MASTER_ID, addr, data_wr);
                    done <= 1;
                    st   <= S_STOP_0;
                end
            end
            else begin
                $display("[M%0d] NACK. Aborting.", MASTER_ID);
                st <= S_STOP_0;
            end
        end

        // ── STOP ─────────────────────────────────────────────
        // S_STOP_0: SCL=0, SDA=0  (ensure SDA low before SCL rises)
        // S_STOP_1: SCL=1, SDA=0  (SCL high, SDA still low)
        // S_STOP_2: SCL=1, SDA=1  (SDA rises = STOP)

        S_STOP_0: begin
            scl_out <= 0;
            sda_out <= 0;
            st      <= S_STOP_1;
        end

        S_STOP_1: begin
            scl_out <= 1;
            sda_out <= 0;
            st      <= S_STOP_2;
        end

        S_STOP_2: begin
            sda_out <= 1;   // SDA rises while SCL=1 = STOP
            $display("[M%0d] STOP generated.", MASTER_ID);
            st <= S_IDLE;
        end

        // ── ARB_WAIT_STOP ────────────────────────────────────
        // Lost arbitration. Release bus completely.
        // Stay here watching for the winner's STOP condition.
        // Only after seeing STOP do we know the winner is done
        // and the bus is truly free.
        S_ARB_WAIT_STOP: begin
            sda_out <= 1;
            scl_out <= 1;
            if (stop_on_bus) begin
                $display("[M%0d] Winner's STOP seen. Starting backoff (%0d cycles).",
                          MASTER_ID, BACKOFF);
                arb_cnt <= 0;
                st      <= S_ARB_BACKOFF;
            end
        end

        // ── ARB_BACKOFF ──────────────────────────────────────
        // STOP was seen. Now wait BACKOFF cycles before retrying.
        // Different BACKOFF values per master (set in testbench)
        // prevents them both retrying at the same instant.
        S_ARB_BACKOFF: begin
            sda_out <= 1;
            scl_out <= 1;
            if (arb_cnt < BACKOFF)
                arb_cnt <= arb_cnt + 1;
            else begin
                $display("[M%0d] Backoff done. Retrying transaction.", MASTER_ID);
                arb_cnt      <= 0;
                sending_addr <= 1;
                shift        <= {addr, 1'b0};
                bit_idx      <= 7;
                st           <= S_WAIT_BUS;
            end
        end

        default: st <= S_IDLE;

        endcase
    end
end

endmodule
