`timescale 1ns/1ps

module top (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        tvalid_in, tlast_in,
    input  wire [31:0] tdata_in,
    input  wire        bb_s_tready,
    output wire        tready_in,
    output wire [31:0] bb_s_tdata_in,
    
    input  wire        tvalid_out, tlast_out,
    input  wire [31:0] tdata_out,
    input  wire        tready_out,
    output wire        bb_m_tready,
    output wire [31:0] axi_s_tdata_out,
    
    input  wire [7:0]  reg_addr,
    input  wire        reg_wr_en,
    input  wire [31:0] reg_data_in,

    output wire [31:0] reg_total_frames, reg_lat_violations, reg_jit_violations,
    output wire [31:0] reg_min_lat, reg_max_lat, reg_min_jit, reg_max_jit,
    output wire [31:0] reg_sum_lat_lo, reg_sum_lat_hi, reg_thresh_lat, reg_thresh_jit,
    
    output wire [31:0] reg_lat_bucket_0, reg_lat_bucket_1, reg_lat_bucket_2, reg_lat_bucket_3,
    output wire [31:0] reg_lat_bucket_4, reg_lat_bucket_5, reg_lat_bucket_6, reg_lat_bucket_7,
    output wire [31:0] reg_jit_bucket_0, reg_jit_bucket_1, reg_jit_bucket_2, reg_jit_bucket_3,
    output wire [31:0] reg_jit_bucket_4, reg_jit_bucket_5, reg_jit_bucket_6, reg_jit_bucket_7,

    // Flight Recorder outputs
    // reg_viol_empty is 1-bit — 0 = has data, 1 = empty
    output wire [31:0] reg_viol_ts_hi, reg_viol_ts_lo, reg_viol_dat,
    output wire        reg_viol_empty,  // FIX 3: 1-bit not 32-bit

    output wire        alert_lat_violation, alert_jit_violation
);

    // ── Pass-through ──
    assign tready_in       = bb_s_tready;
    assign bb_m_tready     = tready_out;
    assign bb_s_tdata_in   = tdata_in;
    assign axi_s_tdata_out = tdata_out;

    // ── Internal wires ──
    wire [63:0] global_time, i_ts, o_ts, i_ts_out, o_ts_out, arb_t_start, arb_t_end;
    wire        i_ts_valid, o_ts_valid, i_empty, o_empty, i_full, o_full, i_pop, o_pop;
    wire        arb_trigger, lat_ready, jit_ready, stat_en_lat, stat_en_jit, lat_viol, jit_viol;
    wire [31:0] inp_frame_count, out_frame_count, lat, jit, stat_lat, stat_jit;
    wire [31:0] threshold_latency, threshold_jitter;

    // FIX 1: soft_reset is 1-bit — was incorrectly declared as [31:0]
    wire        soft_reset;

    wire [31:0] sb_total_frames, sb_lat_violations, sb_jit_violations;
    wire [31:0] sb_min_lat, sb_max_lat, sb_min_jit, sb_max_jit;
    wire [63:0] sb_sum_lat;
    wire [31:0] sb_lat_bucket_0, sb_lat_bucket_1, sb_lat_bucket_2, sb_lat_bucket_3;
    wire [31:0] sb_lat_bucket_4, sb_lat_bucket_5, sb_lat_bucket_6, sb_lat_bucket_7;
    wire [31:0] sb_jit_bucket_0, sb_jit_bucket_1, sb_jit_bucket_2, sb_jit_bucket_3;
    wire [31:0] sb_jit_bucket_4, sb_jit_bucket_5, sb_jit_bucket_6, sb_jit_bucket_7;

    // Flight recorder wires
    wire [95:0] viol_data_out;
    wire        viol_empty, viol_full, viol_pop;

    // FIX 2: Register arb_t_end at the moment latency is computed
    // arb_t_end is valid when arb_trigger fires (arbiter popped both FIFOs)
    // lat_ready fires one cycle later (latency_calc registered output)
    // By the time lat_viol fires (one more cycle via latency_comparator),
    // arb_t_end may have moved on to the next frame on back-to-back traffic.
    // Solution: capture arb_t_end when arb_trigger fires — that is the
    // O_timestamp of the frame whose latency is currently being computed.
    reg [63:0] viol_timestamp;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            viol_timestamp <= 64'd0;
        else if (arb_trigger)
            viol_timestamp <= arb_t_end; // lock in the timestamp when arbiter fires
    end

    // ── Module Instantiations ──
    global_timer gt (
        .clk(clk), .rst_n(rst_n), .timer_out(global_time)
    );

    input_frame_counter ifc (
        .clk(clk), .rst_n(rst_n),
        .tvalid(tvalid_in), .tready(bb_s_tready), .tlast(tlast_in),
        .global_time(global_time),
        .frame_count(inp_frame_count),
        .i_timestamp(i_ts), .i_timestamp_valid(i_ts_valid)
    );

    i_timestamp_fifo #(.DEPTH(8)) i_fifo (
        .clk(clk), .rst_n(rst_n),
        .push(i_ts_valid), .timestamp_in(i_ts),
        .pop(i_pop), .timestamp_out(i_ts_out),
        .empty(i_empty), .full(i_full)
    );

    output_frame_counter ofc (
        .clk(clk), .rst_n(rst_n),
        .tvalid(tvalid_out), .tready(tready_out), .tlast(tlast_out),
        .global_time(global_time),
        .frame_count(out_frame_count),
        .o_timestamp(o_ts), .o_timestamp_valid(o_ts_valid)
    );

    o_timestamp_fifo #(.DEPTH(8)) o_fifo (
        .clk(clk), .rst_n(rst_n),
        .push(o_ts_valid), .timestamp_in(o_ts),
        .pop(o_pop), .timestamp_out(o_ts_out),
        .empty(o_empty), .full(o_full)
    );

    timestamp_arbiter arb (
        .clk(clk), .rst_n(rst_n),
        .i_fifo_empty(i_empty), .i_fifo_timestamp(i_ts_out), .i_fifo_pop(i_pop),
        .o_fifo_empty(o_empty), .o_fifo_timestamp(o_ts_out), .o_fifo_pop(o_pop),
        .t_start(arb_t_start), .t_end(arb_t_end), .trigger(arb_trigger)
    );

    latency_calc lc (
        .clk(clk), .rst_n(rst_n),
        .trigger(arb_trigger),
        .t_start(arb_t_start), .t_end(arb_t_end),
        .latency_out(lat), .latency_ready(lat_ready)
    );

    jitter_calc jc (
        .clk(clk), .rst_n(rst_n),
        .trigger(lat_ready),
        .current_latency(lat),
        .jitter_out(jit), .jitter_ready(jit_ready)
    );

    latency_comparator lcomp (
        .clk(clk), .rst_n(rst_n),
        .en_lat(lat_ready), .latency(lat),
        .threshold_latency(threshold_latency),
        .alert_lat_violation(lat_viol),
        .stat_en_lat(stat_en_lat), .stat_latency(stat_lat)
    );

    jitter_comparator jcomp (
        .clk(clk), .rst_n(rst_n),
        .en_jit(jit_ready), .jitter(jit),
        .threshold_jitter(threshold_jitter),
        .alert_jit_violation(jit_viol),
        .stat_en_jit(stat_en_jit), .stat_jitter(stat_jit)
    );

    // FIX 2: Use viol_timestamp (registered at arb_trigger time) instead of
    // raw arb_t_end which may be stale on back-to-back frames by the time
    // lat_viol fires (2 cycles after arb_trigger)
    violation_fifo #(.DEPTH(16)) v_fifo (
        .clk(clk), .rst_n(rst_n),
        .push(lat_viol),
        .data_in({viol_timestamp, stat_lat}), // FIX: was {arb_t_end, stat_lat}
        .pop(viol_pop),
        .data_out(viol_data_out),
        .empty(viol_empty),
        .full(viol_full)
    );

    statistics_block sb (
        .clk(clk), .rst_n(rst_n & ~soft_reset),
        .stat_en_lat(stat_en_lat), .stat_latency(stat_lat), .alert_lat_violation(lat_viol),
        .stat_en_jit(stat_en_jit), .stat_jitter(stat_jit), .alert_jit_violation(jit_viol),
        .stat_total_frames(sb_total_frames), .stat_lat_violations(sb_lat_violations), .stat_jit_violations(sb_jit_violations),
        .stat_min_lat(sb_min_lat), .stat_max_lat(sb_max_lat),
        .stat_min_jit(sb_min_jit), .stat_max_jit(sb_max_jit),
        .stat_sum_lat(sb_sum_lat),
        .stat_lat_bucket_0(sb_lat_bucket_0), .stat_lat_bucket_1(sb_lat_bucket_1),
        .stat_lat_bucket_2(sb_lat_bucket_2), .stat_lat_bucket_3(sb_lat_bucket_3),
        .stat_lat_bucket_4(sb_lat_bucket_4), .stat_lat_bucket_5(sb_lat_bucket_5),
        .stat_lat_bucket_6(sb_lat_bucket_6), .stat_lat_bucket_7(sb_lat_bucket_7),
        .stat_jit_bucket_0(sb_jit_bucket_0), .stat_jit_bucket_1(sb_jit_bucket_1),
        .stat_jit_bucket_2(sb_jit_bucket_2), .stat_jit_bucket_3(sb_jit_bucket_3),
        .stat_jit_bucket_4(sb_jit_bucket_4), .stat_jit_bucket_5(sb_jit_bucket_5),
        .stat_jit_bucket_6(sb_jit_bucket_6), .stat_jit_bucket_7(sb_jit_bucket_7)
    );

    register_interface ri (
        .clk(clk), .rst_n(rst_n),
        .addr(reg_addr), .wr_en(reg_wr_en), .data_in(reg_data_in),
        .threshold_latency(threshold_latency), .threshold_jitter(threshold_jitter),
        .soft_reset(soft_reset),
        .stat_total_frames(sb_total_frames), .stat_lat_violations(sb_lat_violations), .stat_jit_violations(sb_jit_violations),
        .stat_min_lat(sb_min_lat), .stat_max_lat(sb_max_lat),
        .stat_min_jit(sb_min_jit), .stat_max_jit(sb_max_jit),
        .stat_sum_lat(sb_sum_lat),
        .stat_lat_bucket_0(sb_lat_bucket_0), .stat_lat_bucket_1(sb_lat_bucket_1),
        .stat_lat_bucket_2(sb_lat_bucket_2), .stat_lat_bucket_3(sb_lat_bucket_3),
        .stat_lat_bucket_4(sb_lat_bucket_4), .stat_lat_bucket_5(sb_lat_bucket_5),
        .stat_lat_bucket_6(sb_lat_bucket_6), .stat_lat_bucket_7(sb_lat_bucket_7),
        .stat_jit_bucket_0(sb_jit_bucket_0), .stat_jit_bucket_1(sb_jit_bucket_1),
        .stat_jit_bucket_2(sb_jit_bucket_2), .stat_jit_bucket_3(sb_jit_bucket_3),
        .stat_jit_bucket_4(sb_jit_bucket_4), .stat_jit_bucket_5(sb_jit_bucket_5),
        .stat_jit_bucket_6(sb_jit_bucket_6), .stat_jit_bucket_7(sb_jit_bucket_7),
        .viol_data_out(viol_data_out), .viol_empty(viol_empty), .viol_pop(viol_pop),
        .reg_viol_ts_hi(reg_viol_ts_hi), .reg_viol_ts_lo(reg_viol_ts_lo),
        .reg_viol_dat(reg_viol_dat), .reg_viol_empty(reg_viol_empty),
        .reg_total_frames(reg_total_frames), .reg_lat_violations(reg_lat_violations), .reg_jit_violations(reg_jit_violations),
        .reg_min_lat(reg_min_lat), .reg_max_lat(reg_max_lat),
        .reg_min_jit(reg_min_jit), .reg_max_jit(reg_max_jit),
        .reg_sum_lat_lo(reg_sum_lat_lo), .reg_sum_lat_hi(reg_sum_lat_hi),
        .reg_thresh_lat(reg_thresh_lat), .reg_thresh_jit(reg_thresh_jit),
        .reg_lat_bucket_0(reg_lat_bucket_0), .reg_lat_bucket_1(reg_lat_bucket_1),
        .reg_lat_bucket_2(reg_lat_bucket_2), .reg_lat_bucket_3(reg_lat_bucket_3),
        .reg_lat_bucket_4(reg_lat_bucket_4), .reg_lat_bucket_5(reg_lat_bucket_5),
        .reg_lat_bucket_6(reg_lat_bucket_6), .reg_lat_bucket_7(reg_lat_bucket_7),
        .reg_jit_bucket_0(reg_jit_bucket_0), .reg_jit_bucket_1(reg_jit_bucket_1),
        .reg_jit_bucket_2(reg_jit_bucket_2), .reg_jit_bucket_3(reg_jit_bucket_3),
        .reg_jit_bucket_4(reg_jit_bucket_4), .reg_jit_bucket_5(reg_jit_bucket_5),
        .reg_jit_bucket_6(reg_jit_bucket_6), .reg_jit_bucket_7(reg_jit_bucket_7)
    );

    assign alert_lat_violation = lat_viol;
    assign alert_jit_violation = jit_viol;

endmodule
