
module i2c_slave #(
    parameter [6:0] MY_ADDR     = 7'h10,
    parameter       SLAVE_ID    = 0,
    parameter       STRETCH_CYC = 0
)(
    input  wire clk,
    input  wire rst,
    input  wire sda_in,
    input  wire scl_in,
    output reg  sda_out,
    output reg  scl_out
);

localparam [3:0]
    IDLE       = 0,
    RECV_ADDR  = 1,
    ADDR_DONE  = 2,
    STRETCHING = 3,
    ACK_ADDR_0 = 4,
    ACK_ADDR_1 = 5,
    ACK_ADDR_2 = 6,
    RECV_DATA  = 7,
    DATA_DONE  = 8,
    ACK_DATA_0 = 9,
    ACK_DATA_1 = 10,
    ACK_DATA_2 = 11,
    DONE       = 12;

reg [3:0] state;

// ── Edge detection ───────────────────────────────────────────
reg scl_prev, sda_prev;
wire scl_rise =  scl_in & ~scl_prev;
wire scl_fall = ~scl_in &  scl_prev;

// START: SDA falls while SCL=1
wire start_cond = ~sda_in & sda_prev & scl_in;

// STOP: SDA rises while SCL=1
// BUT only treat as real STOP when SCL has been high for at least
// 1 cycle (scl_prev=1 too). This filters out the SDA change
// that happens right as SCL rises from a data bit.
wire stop_cond = sda_in & ~sda_prev & scl_in & scl_prev;

always @(posedge clk) begin
    scl_prev <= scl_in;
    sda_prev <= sda_in;
end

reg [7:0] shift;
reg [7:0] rx_byte;
reg [2:0] bit_cnt;
reg [7:0] stretch_cnt;
reg [7:0] stored_data;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state       <= IDLE;
        sda_out     <= 1;
        scl_out     <= 1;
        shift       <= 0;
        rx_byte     <= 0;
        bit_cnt     <= 0;
        stretch_cnt <= 0;
        stored_data <= 0;
    end
    else begin

        // STOP overrides everything
        if (stop_cond && state != IDLE && state != RECV_ADDR) begin
            $display("[S%0d] STOP detected -> IDLE.", SLAVE_ID);
            state   <= IDLE;
            sda_out <= 1;
            scl_out <= 1;
        end

        else begin
        case (state)

        // ── IDLE ─────────────────────────────────────────────
        IDLE: begin
            sda_out <= 1;
            scl_out <= 1;
            shift   <= 0;
            bit_cnt <= 0;
            if (start_cond) begin
                $display("[S%0d] START detected.", SLAVE_ID);
                state <= RECV_ADDR;
            end
        end

        // ── RECV_ADDR ────────────────────────────────────────
        // Sample SDA on each scl_rise. Capture full byte when done.
        RECV_ADDR: begin
            if (scl_rise) begin
                shift   <= {shift[6:0], sda_in};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 7) begin
                    rx_byte <= {shift[6:0], sda_in};
                    state   <= ADDR_DONE;
                end
            end
        end

        // ── ADDR_DONE ────────────────────────────────────────
        // rx_byte is valid. SCL is high. Wait for scl_fall then
        // drive ACK (SDA=0) only while SCL is low — safe.
        ADDR_DONE: begin
            if (scl_fall) begin
                if (rx_byte[7:1] == MY_ADDR) begin
                    $display("[S%0d] MATCH addr=0x%02h R/W=%0b",
                              SLAVE_ID, rx_byte[7:1], rx_byte[0]);
                    if (STRETCH_CYC > 0) begin
                        $display("[S%0d] STRETCHING %0d cycles.", SLAVE_ID, STRETCH_CYC);
                        scl_out     <= 0;
                        stretch_cnt <= 0;
                        state       <= STRETCHING;
                    end
                    else begin
                        sda_out <= 0;       // drive ACK while SCL=0 (safe)
                        state   <= ACK_ADDR_0;
                    end
                end
                else begin
                    $display("[S%0d] no match (got 0x%02h).", SLAVE_ID, rx_byte[7:1]);
                    state <= IDLE;
                end
            end
        end

        // ── STRETCHING ───────────────────────────────────────
        STRETCHING: begin
            scl_out     <= 0;
            stretch_cnt <= stretch_cnt + 1;
            if (stretch_cnt == 1 || stretch_cnt == STRETCH_CYC/2)
                $display("[S%0d] Stretching %0d/%0d...", SLAVE_ID, stretch_cnt, STRETCH_CYC);
            if (stretch_cnt >= STRETCH_CYC) begin
                $display("[S%0d] Stretch done. Releasing SCL.", SLAVE_ID);
                scl_out <= 1;
                sda_out <= 0;
                state   <= ACK_ADDR_0;
            end
        end

        // ── ACK_ADDR_0 ───────────────────────────────────────
        // SDA=0 already. SCL is low. Wait for master to raise SCL.
        ACK_ADDR_0: begin
            sda_out <= 0;
            if (scl_rise) begin
                $display("[S%0d] Addr ACK: SCL high.", SLAVE_ID);
                state <= ACK_ADDR_1;
            end
        end

        // ── ACK_ADDR_1 ───────────────────────────────────────
        // SCL=1, SDA=0. Hold. Wait for SCL to fall.
        ACK_ADDR_1: begin
            sda_out <= 0;
            if (scl_fall) begin
                $display("[S%0d] Addr ACK: SCL low. Releasing SDA.", SLAVE_ID);
                state <= ACK_ADDR_2;
            end
        end

        // ── ACK_ADDR_2 ───────────────────────────────────────
        // SCL is low. NOW release SDA safely.
        // SCL=0 guarantees releasing SDA=1 is NOT seen as STOP.
        ACK_ADDR_2: begin
            sda_out <= 1;   // release SDA while SCL=0 — safe, not a STOP
            shift   <= 0;
            bit_cnt <= 0;
            $display("[S%0d] Addr ACK done. Ready for data.", SLAVE_ID);
            state   <= RECV_DATA;
        end

        // ── RECV_DATA ────────────────────────────────────────
        RECV_DATA: begin
            if (scl_rise) begin
                shift   <= {shift[6:0], sda_in};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 7) begin
                    rx_byte <= {shift[6:0], sda_in};
                    state   <= DATA_DONE;
                end
            end
        end

        // ── DATA_DONE ────────────────────────────────────────
        DATA_DONE: begin
            if (scl_fall) begin
                stored_data <= rx_byte;
                $display("[S%0d] Data 0x%02h stored.", SLAVE_ID, rx_byte);
                sda_out <= 0;
                state   <= ACK_DATA_0;
            end
        end

        ACK_DATA_0: begin
            sda_out <= 0;
            if (scl_rise) state <= ACK_DATA_1;
        end

        ACK_DATA_1: begin
            sda_out <= 0;
            if (scl_fall) begin
                $display("[S%0d] Data ACK done.", SLAVE_ID);
                state <= ACK_DATA_2;
            end
        end

        ACK_DATA_2: begin
            sda_out <= 1;   // release SDA while SCL=0 — safe
            state   <= DONE;
        end

        // ── DONE ─────────────────────────────────────────────
        DONE: begin
            sda_out <= 1;
            scl_out <= 1;
        end

        default: state <= IDLE;

        endcase
        end
    end
end

endmodule
