`timescale 1ns/1ps

// ─────────────────────────────────────────────────────────────────────────────
// input_frame_counter.v
// ─────────────────────────────────────────────────────────────────────────────
// Sits on the INPUT side of the AXI4-Stream path.
// Captures I_TimeStamp on the FIRST beat (first tvalid+tready) of each frame.
// Fires i_timestamp_valid as a single-cycle push strobe to i_timestamp_fifo.
//
// Connections:
//   global_timer.timer_out     → global_time
//   i_timestamp_valid          → i_timestamp_fifo.push
//   i_timestamp                → i_timestamp_fifo.timestamp_in
// ─────────────────────────────────────────────────────────────────────────────

module input_frame_counter (
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Stream — observing only, NOT modifying the data path
    input  wire        tvalid,
    input  wire        tready,
    input  wire        tlast,

    // From global_timer
    input  wire [63:0] global_time,

    // Outputs → i_timestamp_fifo
    output reg  [31:0] frame_count,       // total input frames seen
    output reg  [63:0] i_timestamp,       // I_TimeStamp to push into I_FIFO
    output reg         i_timestamp_valid  // single-cycle push strobe to I_FIFO
);

    reg in_frame; // HIGH when we are currently inside a frame

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_count       <= 32'd0;
            i_timestamp       <= 64'd0;
            i_timestamp_valid <= 1'b0;
            in_frame          <= 1'b0;
        end else begin

            // Default: de-assert every cycle (single-cycle pulse)
            i_timestamp_valid <= 1'b0;

            if (tvalid && tready) begin

                if (!in_frame) begin
                    // ── First beat of a new frame ──────────────────────────
                    // Capture global timer as I_TimeStamp
                    // Push strobe fires exactly ONCE per frame
                    i_timestamp       <= global_time;
                    i_timestamp_valid <= 1'b1;   // → I_FIFO.push
                    frame_count       <= frame_count + 1;
                    in_frame          <= 1'b1;
                end

                if (tlast) begin
                    // ── Last beat — back to idle ───────────────────────────
                    in_frame <= 1'b0;
                end

            end
        end
    end

endmodule
