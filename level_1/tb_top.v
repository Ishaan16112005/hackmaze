`timescale 1ns/1ps
// =============================================================================
// tb_top.v — AutoTimeMon-TSN Self-Checking Testbench (Fully Automated + Flight Recorder)
// =============================================================================

module tb_top;
    localparam CLK_PERIOD     = 10;
    localparam BB_DELAY_SHORT = 20;
    localparam BB_DELAY_LONG  = 300; // This will trigger a violation! (Threshold is 200)
    localparam THRESH_LAT     = 32'd200;
    localparam THRESH_JIT     = 32'd50;
    localparam BB_QDEPTH      = 8;
    localparam MAX_BEATS      = 8;
    localparam MAX_FRAMES_REF = 64;

    reg         clk;
    reg         rst_n;

    reg         tvalid_in, tlast_in;
    reg  [31:0] tdata_in;
    reg         bb_s_tready;
    wire        tready_in;
    wire [31:0] bb_s_tdata_in;

    reg         tvalid_out, tlast_out;
    reg  [31:0] tdata_out;
    reg         tready_out;
    wire        bb_m_tready;
    wire [31:0] axi_s_tdata_out;

    reg  [7:0]  reg_addr;
    reg         reg_wr_en;
    reg  [31:0] reg_data_in;

    wire [31:0] reg_total_frames, reg_lat_violations, reg_jit_violations;
    wire [31:0] reg_min_lat, reg_max_lat, reg_min_jit, reg_max_jit;
    wire [31:0] reg_sum_lat_lo, reg_sum_lat_hi, reg_thresh_lat, reg_thresh_jit;
    
    wire [31:0] reg_lat_bucket_0, reg_lat_bucket_1, reg_lat_bucket_2, reg_lat_bucket_3;
    wire [31:0] reg_lat_bucket_4, reg_lat_bucket_5, reg_lat_bucket_6, reg_lat_bucket_7;

    wire [31:0] reg_jit_bucket_0, reg_jit_bucket_1, reg_jit_bucket_2, reg_jit_bucket_3;
    wire [31:0] reg_jit_bucket_4, reg_jit_bucket_5, reg_jit_bucket_6, reg_jit_bucket_7;

    // ── FLIGHT RECORDER WIRES ──
    wire [31:0] reg_viol_ts_hi, reg_viol_ts_lo, reg_viol_dat;
    wire        reg_viol_empty;
    wire        alert_lat_violation, alert_jit_violation;

    top dut (
        .clk(clk), .rst_n(rst_n),
        .tvalid_in(tvalid_in), .tlast_in(tlast_in), .tdata_in(tdata_in), .bb_s_tready(bb_s_tready),
        .tready_in(tready_in), .bb_s_tdata_in(bb_s_tdata_in),
        .tvalid_out(tvalid_out), .tlast_out(tlast_out), .tdata_out(tdata_out), .tready_out(tready_out),
        .bb_m_tready(bb_m_tready), .axi_s_tdata_out(axi_s_tdata_out),
        .reg_addr(reg_addr), .reg_wr_en(reg_wr_en), .reg_data_in(reg_data_in),
        
        // Base Stats
        .reg_total_frames(reg_total_frames), .reg_lat_violations(reg_lat_violations), .reg_jit_violations(reg_jit_violations),
        .reg_min_lat(reg_min_lat), .reg_max_lat(reg_max_lat), .reg_min_jit(reg_min_jit), .reg_max_jit(reg_max_jit),
        .reg_sum_lat_lo(reg_sum_lat_lo), .reg_sum_lat_hi(reg_sum_lat_hi), .reg_thresh_lat(reg_thresh_lat), .reg_thresh_jit(reg_thresh_jit),
        
        // Latency Buckets
        .reg_lat_bucket_0(reg_lat_bucket_0), .reg_lat_bucket_1(reg_lat_bucket_1), .reg_lat_bucket_2(reg_lat_bucket_2), .reg_lat_bucket_3(reg_lat_bucket_3),
        .reg_lat_bucket_4(reg_lat_bucket_4), .reg_lat_bucket_5(reg_lat_bucket_5), .reg_lat_bucket_6(reg_lat_bucket_6), .reg_lat_bucket_7(reg_lat_bucket_7),
        
        // Jitter Buckets
        .reg_jit_bucket_0(reg_jit_bucket_0), .reg_jit_bucket_1(reg_jit_bucket_1), .reg_jit_bucket_2(reg_jit_bucket_2), .reg_jit_bucket_3(reg_jit_bucket_3),
        .reg_jit_bucket_4(reg_jit_bucket_4), .reg_jit_bucket_5(reg_jit_bucket_5), .reg_jit_bucket_6(reg_jit_bucket_6), .reg_jit_bucket_7(reg_jit_bucket_7),
        
        // Flight Recorder Ports
        .reg_viol_ts_hi(reg_viol_ts_hi), .reg_viol_ts_lo(reg_viol_ts_lo), .reg_viol_dat(reg_viol_dat), .reg_viol_empty(reg_viol_empty),
        .alert_lat_violation(alert_lat_violation), .alert_jit_violation(alert_jit_violation)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg [63:0] tb_cycle_count;
    initial tb_cycle_count = 64'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tb_cycle_count <= 64'd0;
        else tb_cycle_count <= tb_cycle_count + 1;
    end

    reg [31:0] bb_q_delay [0:BB_QDEPTH-1];
    reg [7:0]  bb_q_beats [0:BB_QDEPTH-1];
    reg [31:0] bb_q_data  [0:BB_QDEPTH-1][0:MAX_BEATS-1];
    reg [2:0]  bb_q_wr, bb_q_rd;
    reg [3:0]  bb_q_count;

    reg        req_send, bb_slave_ack, slave_frame_done;
    reg [31:0] req_beats, req_delay, req_base_data, req_bp_cycles;

    // Software Reference Model Memory
    reg [63:0] ref_i_ts [0:MAX_FRAMES_REF-1];
    reg [63:0] ref_o_ts [0:MAX_FRAMES_REF-1];
    reg [31:0] ref_lat  [0:MAX_FRAMES_REF-1];
    reg [31:0] ref_total_frames, ref_lat_violations, ref_jit_violations;
    reg [31:0] ref_min_lat, ref_max_lat, ref_min_jit, ref_max_jit;
    reg [63:0] ref_sum_lat;
    
    reg [31:0] ref_lat_bucket_0, ref_lat_bucket_1, ref_lat_bucket_2, ref_lat_bucket_3;
    reg [31:0] ref_lat_bucket_4, ref_lat_bucket_5, ref_lat_bucket_6, ref_lat_bucket_7;
    reg [31:0] ref_jit_bucket_0, ref_jit_bucket_1, ref_jit_bucket_2, ref_jit_bucket_3;
    reg [31:0] ref_jit_bucket_4, ref_jit_bucket_5, ref_jit_bucket_6, ref_jit_bucket_7;
    reg [31:0] ref_in_idx, ref_out_idx;

    // Verilog-2001 strict variables
    reg [63:0] active_o_ts; 
    reg [63:0] lat_full; 
    reg [31:0] lat32; 
    reg [31:0] jit32;
    integer tc8_i;
    integer fd;

    // ── PROC E: Input Reference Model ──
    reg ref_in_frame;
    initial begin ref_in_frame = 0; ref_in_idx = 0; end
    always @(posedge clk) begin
        if (!rst_n) begin ref_in_frame <= 0; ref_in_idx <= 0; end 
        else if (tvalid_in && bb_s_tready) begin
            if (!ref_in_frame) begin ref_i_ts[ref_in_idx] <= tb_cycle_count; ref_in_frame <= 1; end
            if (tlast_in) begin ref_in_frame <= 0; ref_in_idx <= ref_in_idx + 1; end
        end
    end

    // ── PROC E: Output Reference Model ──
    reg ref_out_frame;
    initial begin
        ref_out_frame = 0; ref_out_idx = 0; ref_total_frames = 0;
        ref_lat_violations = 0; ref_jit_violations = 0;
        ref_min_lat = 32'hFFFFFFFF; ref_max_lat = 0; ref_min_jit = 32'hFFFFFFFF; ref_max_jit = 0;
        ref_sum_lat = 0;
        ref_lat_bucket_0 = 0; ref_lat_bucket_1 = 0; ref_lat_bucket_2 = 0; ref_lat_bucket_3 = 0;
        ref_lat_bucket_4 = 0; ref_lat_bucket_5 = 0; ref_lat_bucket_6 = 0; ref_lat_bucket_7 = 0;
        ref_jit_bucket_0 = 0; ref_jit_bucket_1 = 0; ref_jit_bucket_2 = 0; ref_jit_bucket_3 = 0;
        ref_jit_bucket_4 = 0; ref_jit_bucket_5 = 0; ref_jit_bucket_6 = 0; ref_jit_bucket_7 = 0;
    end

    always @(posedge clk) begin
        if (!rst_n) ref_out_frame <= 0;
        else if (tvalid_out && tready_out) begin
            if (!ref_out_frame) begin active_o_ts = tb_cycle_count; ref_o_ts[ref_out_idx] <= tb_cycle_count; ref_out_frame <= 1; end 
            else begin active_o_ts = ref_o_ts[ref_out_idx]; end

            if (tlast_out) begin
                lat_full = active_o_ts - ref_i_ts[ref_out_idx];
                lat32    = lat_full[31:0];

                ref_lat[ref_out_idx] = lat32;
                ref_sum_lat = ref_sum_lat + lat32;
                ref_total_frames = ref_total_frames + 1;

                if (lat32 < ref_min_lat) ref_min_lat = lat32;
                if (lat32 > ref_max_lat) ref_max_lat = lat32;
                if (lat32 > THRESH_LAT)  ref_lat_violations = ref_lat_violations + 1;
                
                if      (lat32 <= 10)   ref_lat_bucket_0 = ref_lat_bucket_0 + 1;
                else if (lat32 <= 20)   ref_lat_bucket_1 = ref_lat_bucket_1 + 1;
                else if (lat32 <= 50)   ref_lat_bucket_2 = ref_lat_bucket_2 + 1;
                else if (lat32 <= 100)  ref_lat_bucket_3 = ref_lat_bucket_3 + 1;
                else if (lat32 <= 250)  ref_lat_bucket_4 = ref_lat_bucket_4 + 1;
                else if (lat32 <= 500)  ref_lat_bucket_5 = ref_lat_bucket_5 + 1;
                else if (lat32 <= 1000) ref_lat_bucket_6 = ref_lat_bucket_6 + 1;
                else                    ref_lat_bucket_7 = ref_lat_bucket_7 + 1;

                if (ref_out_idx >= 1) begin
                    if (lat32 > ref_lat[ref_out_idx - 1]) jit32 = lat32 - ref_lat[ref_out_idx - 1];
                    else jit32 = ref_lat[ref_out_idx - 1] - lat32;
                    
                    if (jit32 < ref_min_jit) ref_min_jit = jit32;
                    if (jit32 > ref_max_jit) ref_max_jit = jit32;
                    if (jit32 > THRESH_JIT)  ref_jit_violations = ref_jit_violations + 1;

                    if      (jit32 <= 5)   ref_jit_bucket_0 = ref_jit_bucket_0 + 1;
                    else if (jit32 <= 10)  ref_jit_bucket_1 = ref_jit_bucket_1 + 1;
                    else if (jit32 <= 20)  ref_jit_bucket_2 = ref_jit_bucket_2 + 1;
                    else if (jit32 <= 50)  ref_jit_bucket_3 = ref_jit_bucket_3 + 1;
                    else if (jit32 <= 100) ref_jit_bucket_4 = ref_jit_bucket_4 + 1;
                    else if (jit32 <= 200) ref_jit_bucket_5 = ref_jit_bucket_5 + 1;
                    else if (jit32 <= 500) ref_jit_bucket_6 = ref_jit_bucket_6 + 1;
                    else                   ref_jit_bucket_7 = ref_jit_bucket_7 + 1;
                end

                ref_out_idx = ref_out_idx + 1;
                ref_out_frame <= 0;
            end
        end
    end

    // ── PROC B: BB Slave ──
    integer bb_b;
    initial begin bb_s_tready = 0; bb_slave_ack = 0; bb_q_wr = 0; bb_q_rd = 0; bb_q_count = 0; end
    always @(posedge clk) begin
        if (!rst_n) begin bb_s_tready <= 0; bb_slave_ack <= 0; end
        else begin
            bb_slave_ack <= 0;
            if (req_send && !bb_slave_ack) begin
                bb_q_beats[bb_q_wr] = req_beats[7:0]; bb_q_delay[bb_q_wr] = req_delay; bb_s_tready <= 1;
                for (bb_b = 0; bb_b < req_beats; bb_b = bb_b + 1) begin
                    @(posedge clk); while (!(tvalid_in && bb_s_tready)) @(posedge clk);
                    bb_q_data[bb_q_wr][bb_b] = tdata_in;
                end
                bb_s_tready <= 0; bb_q_wr <= bb_q_wr + 1; bb_q_count <= bb_q_count + 1; bb_slave_ack <= 1;
            end
        end
    end

    // ── PROC C: BB Master ──
    integer bb_d, bb_o;
    initial begin tvalid_out = 0; tlast_out = 0; tdata_out = 0; end
    always @(posedge clk) begin
        if (!rst_n) begin tvalid_out <= 0; tlast_out <= 0; tdata_out <= 0; bb_q_rd <= 0; end
        else if (bb_q_count > 0) begin
            for (bb_d = 0; bb_d < bb_q_delay[bb_q_rd]; bb_d = bb_d + 1) @(posedge clk);
            for (bb_o = 0; bb_o < bb_q_beats[bb_q_rd]; bb_o = bb_o + 1) begin
                @(negedge clk); tvalid_out <= 1; tdata_out <= bb_q_data[bb_q_rd][bb_o];
                tlast_out <= (bb_o == bb_q_beats[bb_q_rd] - 1) ? 1 : 0;
                @(posedge clk); while (!tready_out) @(posedge clk);
            end
            @(negedge clk); tvalid_out <= 0; tlast_out <= 0; tdata_out <= 0;
            bb_q_rd <= bb_q_rd + 1; bb_q_count <= bb_q_count - 1;
        end
    end

    // ── PROC D: AXI Slave ──
    integer bp_i; reg bp_applied;
    initial begin tready_out = 1; slave_frame_done = 0; bp_applied = 0; end
    always @(posedge clk) begin
        if (!rst_n) begin tready_out <= 1; slave_frame_done <= 0; bp_applied <= 0; end 
        else begin
            slave_frame_done <= 0; tready_out <= 1; 
            if (tvalid_out && !bp_applied && req_bp_cycles > 0) begin
                bp_applied <= 1; tready_out <= 0;
                for (bp_i = 0; bp_i < req_bp_cycles; bp_i = bp_i + 1) @(posedge clk);
                tready_out <= 1;
            end
            if (tvalid_out && tready_out && tlast_out) begin
                slave_frame_done <= 1; bp_applied <= 0; 
            end
        end
    end

    // ── Tasks ──
    task wait_cycles; input integer n; integer i; begin for (i=0; i<n; i=i+1) @(posedge clk); end endtask
    task reg_write; input [7:0] addr; input [31:0] data; begin @(negedge clk); reg_addr = addr; reg_data_in = data; reg_wr_en = 1; @(posedge clk); #1; reg_wr_en = 0; end endtask
    
    integer pass_count, fail_count, tc_num, mf_b;
    initial begin pass_count=0; fail_count=0; tc_num=0; end

    task check_eq; input [200*8-1:0] label; input [63:0] got; input [63:0] expected; begin
        if (got === expected) begin $display("  [PASS] TC%0d | %-20s got=%-10d  expected=%-10d", tc_num, label, got, expected); pass_count = pass_count + 1; end 
        else begin $display("  [FAIL] TC%0d | %-20s got=%-10d  expected=%-10d  @t=%0t", tc_num, label, got, expected, $time); fail_count = fail_count + 1; end
    end endtask

    task master_send_frame; input [31:0] beats, base_data, bb_delay, bp_cycles; begin
        req_beats = beats; req_base_data = base_data; req_delay = bb_delay; req_bp_cycles = bp_cycles;
        @(negedge clk); req_send = 1;
        for (mf_b=0; mf_b<beats; mf_b=mf_b+1) begin @(negedge clk); tvalid_in = 1; tdata_in = base_data + mf_b; tlast_in = (mf_b == beats - 1) ? 1 : 0; @(posedge clk); while (!bb_s_tready) @(posedge clk); end
        @(negedge clk); tvalid_in = 0; tlast_in = 0; tdata_in = 0; req_send = 0;
        @(posedge clk); while (!bb_slave_ack) @(posedge clk);
        @(posedge clk); while (!slave_frame_done) @(posedge clk); wait_cycles(15);
    end endtask

    // ── Test Sequencer ──
    initial begin
        $dumpfile("tb_top.vcd"); $dumpvars(0, tb_top);
        rst_n=0; tvalid_in=0; tlast_in=0; tdata_in=0; req_send=0; req_beats=0; req_delay=0; req_base_data=0; req_bp_cycles=0; reg_addr=0; reg_wr_en=0; reg_data_in=0;
        wait_cycles(5); rst_n = 1; wait_cycles(3);
        
        reg_write(8'h00, THRESH_LAT); 
        reg_write(8'h04, THRESH_JIT); 
        wait_cycles(2);

        $display("\n================================================");
        $display("  AutoTimeMon-TSN Self-Checking Testbench");
        $display("================================================\n");

        tc_num = 1; master_send_frame(1, 32'hA001, BB_DELAY_SHORT, 0);
        tc_num = 2; master_send_frame(4, 32'hB000, BB_DELAY_SHORT, 0);
        tc_num = 3; master_send_frame(1, 32'hC001, BB_DELAY_SHORT, 0); master_send_frame(1, 32'hC002, 35, 0); master_send_frame(1, 32'hC003, 70, 0);
        
        tc_num = 4; // ── THIS WILL CAUSE A LATENCY VIOLATION ──
        master_send_frame(1, 32'hD001, BB_DELAY_LONG, 0);

        // ── VERIFY FLIGHT RECORDER (DEEP VIOLATION LOGGING) ──
        $display("\n--- CHECKING BLACK BOX FLIGHT RECORDER ---");
        @(posedge clk); #1;
        if (reg_viol_empty == 0) begin
            $display("  [PASS] TC4 | Violation caught! FIFO is not empty.");
            $display("             | Forensic Timestamp : %0d", {reg_viol_ts_hi, reg_viol_ts_lo});
            $display("             | Forensic Latency   : %0d cycles (Threshold was %0d)", reg_viol_dat, THRESH_LAT);
            
            // Pop the FIFO using the new register
            reg_write(8'h5C, 32'd1); 
            wait_cycles(2);
            if (reg_viol_empty == 1) $display("  [PASS] TC4 | FIFO popped successfully. Ready for next error.\n");
            else $display("  [FAIL] TC4 | FIFO did not clear after pop!\n");
        end else begin
            $display("  [FAIL] TC4 | Flight recorder failed to trap the violation!\n");
            fail_count = fail_count + 1;
        end

        tc_num = 5; master_send_frame(1, 32'hE001, BB_DELAY_SHORT, 0); master_send_frame(1, 32'hE002, BB_DELAY_SHORT+THRESH_JIT+20, 0);
        tc_num = 6; master_send_frame(2, 32'hF001, BB_DELAY_SHORT, 5); 

        tc_num = 8; $display("--- TC8: FIFO stress & Dual Histogram Population ---");
        for (tc8_i=0; tc8_i<8; tc8_i=tc8_i+1) begin
            master_send_frame(1, 32'h88000000+tc8_i, (tc8_i*25)+15, 0); 
        end
        wait_cycles(20);
        
        // ── VERIFICATION PRINTOUTS ──
        $display("\n--- P99 LATENCY HISTOGRAM VERIFICATION ---");
        check_eq("lat_bucket_0 (0-10)   ", reg_lat_bucket_0, ref_lat_bucket_0);
        check_eq("lat_bucket_1 (11-20)  ", reg_lat_bucket_1, ref_lat_bucket_1);
        check_eq("lat_bucket_2 (21-50)  ", reg_lat_bucket_2, ref_lat_bucket_2);
        check_eq("lat_bucket_3 (51-100) ", reg_lat_bucket_3, ref_lat_bucket_3);
        check_eq("lat_bucket_4 (101-250)", reg_lat_bucket_4, ref_lat_bucket_4);
        check_eq("lat_bucket_5 (251-500)", reg_lat_bucket_5, ref_lat_bucket_5);
        check_eq("lat_bucket_6 (501-1k) ", reg_lat_bucket_6, ref_lat_bucket_6);
        check_eq("lat_bucket_7 (>1000)  ", reg_lat_bucket_7, ref_lat_bucket_7);

        $display("\n--- JITTER HISTOGRAM VERIFICATION ---");
        check_eq("jit_bucket_0 (0-5)    ", reg_jit_bucket_0, ref_jit_bucket_0);
        check_eq("jit_bucket_1 (6-10)   ", reg_jit_bucket_1, ref_jit_bucket_1);
        check_eq("jit_bucket_2 (11-20)  ", reg_jit_bucket_2, ref_jit_bucket_2);
        check_eq("jit_bucket_3 (21-50)  ", reg_jit_bucket_3, ref_jit_bucket_3);
        check_eq("jit_bucket_4 (51-100) ", reg_jit_bucket_4, ref_jit_bucket_4);
        check_eq("jit_bucket_5 (101-200)", reg_jit_bucket_5, ref_jit_bucket_5);
        check_eq("jit_bucket_6 (201-500)", reg_jit_bucket_6, ref_jit_bucket_6);
        check_eq("jit_bucket_7 (>500)   ", reg_jit_bucket_7, ref_jit_bucket_7);

        // ── ASCII TABLE REPORTING ──
        $display("\n================================================");
        $display("          HARDWARE HISTOGRAM RESULTS            ");
        $display("================================================");
        $display("+-----------------------+-------------+");
        $display("| P99 Latency Range     | Frame Count |");
        $display("+-----------------------+-------------+");
        $display("| 0-10 cycles           | %11d |", reg_lat_bucket_0);
        $display("| 11-20 cycles          | %11d |", reg_lat_bucket_1);
        $display("| 21-50 cycles          | %11d |", reg_lat_bucket_2);
        $display("| 51-100 cycles         | %11d |", reg_lat_bucket_3);
        $display("| 101-250 cycles        | %11d |", reg_lat_bucket_4);
        $display("| 251-500 cycles        | %11d |", reg_lat_bucket_5);
        $display("| 501-1000 cycles       | %11d |", reg_lat_bucket_6);
        $display("| > 1000 cycles         | %11d |", reg_lat_bucket_7);
        $display("+-----------------------+-------------+");

        $display("\n+-----------------------+-------------+");
        $display("| Jitter Range          | Frame Count |");
        $display("+-----------------------+-------------+");
        $display("| 0-5 cycles            | %11d |", reg_jit_bucket_0);
        $display("| 6-10 cycles           | %11d |", reg_jit_bucket_1);
        $display("| 11-20 cycles          | %11d |", reg_jit_bucket_2);
        $display("| 21-50 cycles          | %11d |", reg_jit_bucket_3);
        $display("| 51-100 cycles         | %11d |", reg_jit_bucket_4);
        $display("| 101-200 cycles        | %11d |", reg_jit_bucket_5);
        $display("| 201-500 cycles        | %11d |", reg_jit_bucket_6);
        $display("| > 500 cycles          | %11d |", reg_jit_bucket_7);
        $display("+-----------------------+-------------+");

        // ── AUTOMATED DATA DUMP FOR PYTHON ──
        fd = $fopen("histogram_data.txt", "w");
        $fdisplay(fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                  reg_lat_bucket_0, reg_lat_bucket_1, reg_lat_bucket_2, reg_lat_bucket_3,
                  reg_lat_bucket_4, reg_lat_bucket_5, reg_lat_bucket_6, reg_lat_bucket_7);
        $fdisplay(fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                  reg_jit_bucket_0, reg_jit_bucket_1, reg_jit_bucket_2, reg_jit_bucket_3,
                  reg_jit_bucket_4, reg_jit_bucket_5, reg_jit_bucket_6, reg_jit_bucket_7);
        $fclose(fd);
        $display("\n  [INFO] Wrote histogram results to 'histogram_data.txt'");

        // ── FINAL REPORT ──
        $display("================================================");
        $display("  RESULTS: %0d PASSED  |  %0d FAILED", pass_count, fail_count);
        $display("================================================");
        if (fail_count == 0) $display("  *** ALL TESTS PASSED ***\n");
        else $display("  *** FAILURES DETECTED — review above ***\n");
        $finish;
    end

    initial begin #50_000_000; $display("[WATCHDOG] Timeout at %0t", $time); $finish; end
endmodule
