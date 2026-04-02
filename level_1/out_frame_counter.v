`timescale 1ns/1ps

// output_frame_counter.v
// ─────────────────────────────────────────────────────────────────────────────
// Sits on the OUTPUT side of the AXI4-Stream path.
// Captures O_TimeStamp on the LAST beat (tlast) of each outgoing frame.
// Also counts total frames seen on the output side.
//
// tvalid & tready = valid data transfer happening this cycle
// tlast           = last beat of the current frame
//
// Timestamp is captured only when tvalid & tready & tlast are all high
// together — meaning the frame has fully passed through the black box.
// No in_frame tracking needed here since we only care about the last beat.
// ─────────────────────────────────────────────────────────────────────────────

module output_frame_counter (
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Stream — observing only, not modifying
    input  wire        tvalid,
    input  wire        tready,
    input  wire        tlast,

    // From global_timer
    input  wire [63:0] global_time,

    // Outputs
    output reg  [31:0] frame_count,      // total output frames seen
    output reg  [63:0] o_timestamp,      // captured O_TimeStamp for current frame
    output reg         o_timestamp_valid // single-cycle pulse when O_TimeStamp is captured
);

    reg in_frame; //tracks whether we are currently inside a frame 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_count       <= 32'd0;
            o_timestamp       <= 64'd0;
            o_timestamp_valid <= 1'b0;
            in_frame <= 1'b0;
        end else begin
           
	    // Default de-assert every cycle (single-cycle pulse behavior)
            o_timestamp_valid <= 1'b0;

            if (tvalid && tready) begin

                if (!in_frame) begin
                    // First beat of a new frame
                    // Capture the global timer value as I_TimeStamp
                    o_timestamp       <= global_time;
                    o_timestamp_valid <= 1'b1;   // pulse to notify latency_calc
                    frame_count       <= frame_count + 1;
                    in_frame          <= 1'b1;
                end

                // Last beat of this frame — go back to idle
                // Ready to detect the first beat of the next frame
                if (tlast)
                    in_frame <= 1'b0;
	    end
        end
    end

endmodule
