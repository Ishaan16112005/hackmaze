`timescale 1ns/1ps

module statistics_block (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        stat_en_lat,
    input  wire [31:0] stat_latency,
    input  wire        alert_lat_violation,

    input  wire        stat_en_jit,
    input  wire [31:0] stat_jitter,
    input  wire        alert_jit_violation,

    output reg  [31:0] stat_total_frames,
    output reg  [31:0] stat_lat_violations,
    output reg  [31:0] stat_jit_violations,
    output reg  [31:0] stat_min_lat,
    output reg  [31:0] stat_max_lat,
    output reg  [31:0] stat_min_jit,
    output reg  [31:0] stat_max_jit,
    output reg  [63:0] stat_sum_lat,
    
    // ── Latency Histogram Buckets (p99) ──
    output reg  [31:0] stat_lat_bucket_0, // 0-10
    output reg  [31:0] stat_lat_bucket_1, // 11-20
    output reg  [31:0] stat_lat_bucket_2, // 21-50
    output reg  [31:0] stat_lat_bucket_3, // 51-100
    output reg  [31:0] stat_lat_bucket_4, // 101-250
    output reg  [31:0] stat_lat_bucket_5, // 251-500
    output reg  [31:0] stat_lat_bucket_6, // 501-1000
    output reg  [31:0] stat_lat_bucket_7, // > 1000

    // ── Jitter Histogram Buckets ──
    output reg  [31:0] stat_jit_bucket_0, // 0-5 cycles
    output reg  [31:0] stat_jit_bucket_1, // 6-10 cycles
    output reg  [31:0] stat_jit_bucket_2, // 11-20 cycles
    output reg  [31:0] stat_jit_bucket_3, // 21-50 cycles
    output reg  [31:0] stat_jit_bucket_4, // 51-100 cycles
    output reg  [31:0] stat_jit_bucket_5, // 101-200 cycles
    output reg  [31:0] stat_jit_bucket_6, // 201-500 cycles
    output reg  [31:0] stat_jit_bucket_7  // > 500 cycles
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_total_frames   <= 32'd0;
            stat_lat_violations <= 32'd0;
            stat_jit_violations <= 32'd0;
            stat_min_lat        <= 32'hFFFFFFFF;
            stat_max_lat        <= 32'd0;
            stat_min_jit        <= 32'hFFFFFFFF;
            stat_max_jit        <= 32'd0;
            stat_sum_lat        <= 64'd0;
            
            stat_lat_bucket_0 <= 32'd0; stat_lat_bucket_1 <= 32'd0; stat_lat_bucket_2 <= 32'd0; stat_lat_bucket_3 <= 32'd0;
            stat_lat_bucket_4 <= 32'd0; stat_lat_bucket_5 <= 32'd0; stat_lat_bucket_6 <= 32'd0; stat_lat_bucket_7 <= 32'd0;
            
            stat_jit_bucket_0 <= 32'd0; stat_jit_bucket_1 <= 32'd0; stat_jit_bucket_2 <= 32'd0; stat_jit_bucket_3 <= 32'd0;
            stat_jit_bucket_4 <= 32'd0; stat_jit_bucket_5 <= 32'd0; stat_jit_bucket_6 <= 32'd0; stat_jit_bucket_7 <= 32'd0;
        end else begin

            // ── Latency Stats & Histogram ──
            if (stat_en_lat) begin
                stat_total_frames <= stat_total_frames + 1;
                stat_sum_lat      <= stat_sum_lat + stat_latency;

                if (stat_latency < stat_min_lat) stat_min_lat <= stat_latency;
                if (stat_latency > stat_max_lat) stat_max_lat <= stat_latency;
                
                if      (stat_latency <= 10)   stat_lat_bucket_0 <= stat_lat_bucket_0 + 1;
                else if (stat_latency <= 20)   stat_lat_bucket_1 <= stat_lat_bucket_1 + 1;
                else if (stat_latency <= 50)   stat_lat_bucket_2 <= stat_lat_bucket_2 + 1;
                else if (stat_latency <= 100)  stat_lat_bucket_3 <= stat_lat_bucket_3 + 1;
                else if (stat_latency <= 250)  stat_lat_bucket_4 <= stat_lat_bucket_4 + 1;
                else if (stat_latency <= 500)  stat_lat_bucket_5 <= stat_lat_bucket_5 + 1;
                else if (stat_latency <= 1000) stat_lat_bucket_6 <= stat_lat_bucket_6 + 1;
                else                           stat_lat_bucket_7 <= stat_lat_bucket_7 + 1;
            end

            // ── Jitter Stats & Histogram ──
            if (stat_en_jit) begin
                if (stat_jitter < stat_min_jit) stat_min_jit <= stat_jitter;
                if (stat_jitter > stat_max_jit) stat_max_jit <= stat_jitter;
                
                if      (stat_jitter <= 5)   stat_jit_bucket_0 <= stat_jit_bucket_0 + 1;
                else if (stat_jitter <= 10)  stat_jit_bucket_1 <= stat_jit_bucket_1 + 1;
                else if (stat_jitter <= 20)  stat_jit_bucket_2 <= stat_jit_bucket_2 + 1;
                else if (stat_jitter <= 50)  stat_jit_bucket_3 <= stat_jit_bucket_3 + 1;
                else if (stat_jitter <= 100) stat_jit_bucket_4 <= stat_jit_bucket_4 + 1;
                else if (stat_jitter <= 200) stat_jit_bucket_5 <= stat_jit_bucket_5 + 1;
                else if (stat_jitter <= 500) stat_jit_bucket_6 <= stat_jit_bucket_6 + 1;
                else                         stat_jit_bucket_7 <= stat_jit_bucket_7 + 1;
            end

            if (alert_lat_violation) stat_lat_violations <= stat_lat_violations + 1;
            if (alert_jit_violation) stat_jit_violations <= stat_jit_violations + 1;
        end
    end
endmodule
