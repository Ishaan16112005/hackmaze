`timescale 1ns/1ps

module jitter_calc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        trigger,          // Connect to latency_ready from latency_calc
    input  wire [31:0] current_latency,
    output reg  [31:0] jitter_out,
    output reg         jitter_ready      // Pulse to trigger statistics update
);

    reg [31:0] prev_latency;
    reg        first_frame_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            jitter_out       <= 32'd0;
            prev_latency     <= 32'd0;
            jitter_ready     <= 1'b0;
            first_frame_done <= 1'b0;
        end else if (trigger) begin

            if (first_frame_done) begin
                // Frame 2 onwards: compute |current - prev| and notify stats block
                if (current_latency > prev_latency)
                    jitter_out <= current_latency - prev_latency;
                else
                    jitter_out <= prev_latency - current_latency;

                jitter_ready <= 1'b1; // Valid jitter — trigger stats update
            end else begin
                // Frame 1: no previous latency to compare against
                // Do NOT pulse jitter_ready — stats block must not be triggered
                // This prevents false min_jitter = 0 in statistics_block
                jitter_out       <= 32'd0;
                jitter_ready     <= 1'b0; // <-- key fix: was 1'b1 before
                first_frame_done <= 1'b1;
            end

            prev_latency <= current_latency; // Always store for next frame

        end else begin
            jitter_ready <= 1'b0; // De-assert when no trigger
        end
    end

endmodule
