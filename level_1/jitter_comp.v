`timescale 1ns/1ps

// jitter_comparator.v
// ─────────────────────────────────────────────────────────────────────────────
// Compares jitter against a configurable threshold.
// Fires a single-cycle alert pulse on violation.
// Forwards clean jitter + enable to the statistics block.
// Note: en_jit is only asserted from frame 2 onwards (guaranteed by
// jitter_calc) so no bogus zero-jitter values will ever reach here.
// ─────────────────────────────────────────────────────────────────────────────

module jitter_comparator (
    input  wire        clk,
    input  wire        rst_n,

    // From jitter_calc
    input  wire        en_jit,              // jitter_ready pulse (frame 2 onwards only)
    input  wire [31:0] jitter,

    // From register interface
    input  wire [31:0] threshold_jitter,

    // Violation output → top level
    output reg         alert_jit_violation, // single-cycle pulse

    // Pass-through → statistics block
    output reg         stat_en_jit,
    output reg  [31:0] stat_jitter
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alert_jit_violation <= 1'b0;
            stat_en_jit         <= 1'b0;
            stat_jitter         <= 32'd0;
        end else begin

            // Default de-assert (single-cycle pulse behavior)
            alert_jit_violation <= 1'b0;
            stat_en_jit         <= 1'b0;

            if (en_jit) begin
                stat_jitter <= jitter;
                stat_en_jit <= 1'b1;

                if (jitter > threshold_jitter)
                    alert_jit_violation <= 1'b1;
            end

        end
    end

endmodule
