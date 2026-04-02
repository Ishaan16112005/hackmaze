`timescale 1ns/1ps

module register_interface (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  addr,
    input  wire        wr_en,
    input  wire [31:0] data_in,

    output reg  [31:0] threshold_latency,
    output reg  [31:0] threshold_jitter,
    output reg         soft_reset,

    input  wire [31:0] stat_total_frames, stat_lat_violations, stat_jit_violations,
    input  wire [31:0] stat_min_lat, stat_max_lat, stat_min_jit, stat_max_jit,
    input  wire [63:0] stat_sum_lat,
    
    input  wire [31:0] stat_lat_bucket_0, stat_lat_bucket_1, stat_lat_bucket_2, stat_lat_bucket_3,
    input  wire [31:0] stat_lat_bucket_4, stat_lat_bucket_5, stat_lat_bucket_6, stat_lat_bucket_7,

    input  wire [31:0] stat_jit_bucket_0, stat_jit_bucket_1, stat_jit_bucket_2, stat_jit_bucket_3,
    input  wire [31:0] stat_jit_bucket_4, stat_jit_bucket_5, stat_jit_bucket_6, stat_jit_bucket_7,

    // ── Violation FIFO Interface ──
    input  wire [95:0] viol_data_out,
    input  wire        viol_empty,
    output reg         viol_pop,

    // FIX 3: reg_viol_empty is 1-bit — matches the 1-bit viol_empty flag directly
    // Testbench checks: (reg_viol_empty == 0) means has data, (reg_viol_empty == 1) means empty
    // This is now unambiguous — no 32-bit packing needed
    output wire [31:0] reg_viol_ts_hi,
    output wire [31:0] reg_viol_ts_lo,
    output wire [31:0] reg_viol_dat,
    output wire        reg_viol_empty,  // FIX: was output wire [31:0]

    output wire [31:0] reg_total_frames, reg_lat_violations, reg_jit_violations,
    output wire [31:0] reg_min_lat, reg_max_lat, reg_min_jit, reg_max_jit,
    output wire [31:0] reg_sum_lat_lo, reg_sum_lat_hi, reg_thresh_lat, reg_thresh_jit,
    
    output wire [31:0] reg_lat_bucket_0, reg_lat_bucket_1, reg_lat_bucket_2, reg_lat_bucket_3,
    output wire [31:0] reg_lat_bucket_4, reg_lat_bucket_5, reg_lat_bucket_6, reg_lat_bucket_7,
    output wire [31:0] reg_jit_bucket_0, reg_jit_bucket_1, reg_jit_bucket_2, reg_jit_bucket_3,
    output wire [31:0] reg_jit_bucket_4, reg_jit_bucket_5, reg_jit_bucket_6, reg_jit_bucket_7
);

    localparam ADDR_THRESH_LAT = 8'h00;
    localparam ADDR_THRESH_JIT = 8'h04;
    localparam ADDR_SOFT_RESET = 8'h08;
    localparam ADDR_VIOL_POP   = 8'h5C;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            threshold_latency <= 32'h0000FFFF;
            threshold_jitter  <= 32'h0000FFFF;
            soft_reset        <= 1'b0;
            viol_pop          <= 1'b0;
        end else begin
            soft_reset <= 1'b0;
            viol_pop   <= 1'b0;
            if (wr_en) begin
                case (addr)
                    ADDR_THRESH_LAT : threshold_latency <= data_in;
                    ADDR_THRESH_JIT : threshold_jitter  <= data_in;
                    ADDR_SOFT_RESET : soft_reset        <= data_in[0];
                    ADDR_VIOL_POP   : viol_pop          <= data_in[0];
                    default         : ;
                endcase
            end
        end
    end

    // ── Violation FIFO output mapping ──
    // viol_data_out[95:64] = O_Timestamp high 32 bits
    // viol_data_out[63:32] = O_Timestamp low  32 bits
    // viol_data_out[31:0]  = Latency value
    assign reg_viol_ts_hi = viol_data_out[95:64];
    assign reg_viol_ts_lo = viol_data_out[63:32];
    assign reg_viol_dat   = viol_data_out[31:0];
    assign reg_viol_empty = viol_empty; // FIX: direct 1-bit passthrough, no 32-bit packing

    // ── Stats wire-through ──
    assign reg_total_frames   = stat_total_frames;
    assign reg_lat_violations = stat_lat_violations;
    assign reg_jit_violations = stat_jit_violations;
    assign reg_min_lat        = stat_min_lat;
    assign reg_max_lat        = stat_max_lat;
    assign reg_min_jit        = stat_min_jit;
    assign reg_max_jit        = stat_max_jit;
    assign reg_sum_lat_lo     = stat_sum_lat[31:0];
    assign reg_sum_lat_hi     = stat_sum_lat[63:32];
    assign reg_thresh_lat     = threshold_latency;
    assign reg_thresh_jit     = threshold_jitter;
    
    assign reg_lat_bucket_0 = stat_lat_bucket_0; assign reg_lat_bucket_1 = stat_lat_bucket_1;
    assign reg_lat_bucket_2 = stat_lat_bucket_2; assign reg_lat_bucket_3 = stat_lat_bucket_3;
    assign reg_lat_bucket_4 = stat_lat_bucket_4; assign reg_lat_bucket_5 = stat_lat_bucket_5;
    assign reg_lat_bucket_6 = stat_lat_bucket_6; assign reg_lat_bucket_7 = stat_lat_bucket_7;

    assign reg_jit_bucket_0 = stat_jit_bucket_0; assign reg_jit_bucket_1 = stat_jit_bucket_1;
    assign reg_jit_bucket_2 = stat_jit_bucket_2; assign reg_jit_bucket_3 = stat_jit_bucket_3;
    assign reg_jit_bucket_4 = stat_jit_bucket_4; assign reg_jit_bucket_5 = stat_jit_bucket_5;
    assign reg_jit_bucket_6 = stat_jit_bucket_6; assign reg_jit_bucket_7 = stat_jit_bucket_7;

endmodule
