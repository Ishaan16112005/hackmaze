`timescale 1ns/1ps

module latency_calc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        trigger,      // Connect to frame_complete signal
    input  wire [63:0] t_start,
    input  wire [63:0] t_end,
    output reg  [31:0] latency_out,  // Saturated to 32 bits safely
    output reg         latency_ready // Pulse to trigger jitter calculation
);

    // Internal 64-bit difference to avoid silent truncation
    wire [63:0] latency_full = t_end - t_start;

    // Saturation constant: max 32-bit value
    localparam [63:0] MAX_LAT = 64'h00000000FFFFFFFF;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latency_out   <= 32'd0;
            latency_ready <= 1'b0;
        end else if (trigger) begin
            // Saturate: if difference exceeds 32-bit range, clamp to max
            if (latency_full > MAX_LAT)
                latency_out <= 32'hFFFFFFFF;
            else
                latency_out <= latency_full[31:0];

            latency_ready <= 1'b1;
        end else begin
            latency_ready <= 1'b0;
        end
    end

endmodule
