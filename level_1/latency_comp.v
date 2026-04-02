`timescale 1ns/1ps

// latency_comparator.v
// ─────────────────────────────────────────────────────────────────────────────
// Compares latency against a configurable threshold.
// Fires a single-cycle alert pulse on violation.
// Forwards clean latency + enable to the statistics block.
// ─────────────────────────────────────────────────────────────────────────────

module latency_comparator (
    input  wire        clk,
    input  wire        rst_n,

    // From latency_calc
    input  wire        en_lat,              // latency_ready pulse
    input  wire [31:0] latency,

    // From register interface
    input  wire [31:0] threshold_latency,

    // Violation output → top level
    output reg         alert_lat_violation, // single-cycle pulse

    // Pass-through → statistics block
    output reg         stat_en_lat,
    output reg  [31:0] stat_latency
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alert_lat_violation <= 1'b0;
            stat_en_lat         <= 1'b0;
            stat_latency        <= 32'd0;
        end else begin

            // Default de-assert (single-cycle pulse behavior)
            alert_lat_violation <= 1'b0;
            stat_en_lat         <= 1'b0;

            if (en_lat) begin
                stat_latency <= latency;
                stat_en_lat  <= 1'b1;

                if (latency > threshold_latency)
                    alert_lat_violation <= 1'b1;
            end

        end
    end

endmodule
