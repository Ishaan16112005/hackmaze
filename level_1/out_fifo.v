`timescale 1ns/1ps

// ─────────────────────────────────────────────────────────────────────────────
// o_timestamp_fifo.v (FWFT Version)
// ─────────────────────────────────────────────────────────────────────────────
// Stores O_TimeStamps (output-side timestamps) in order.
// Pushed by output_frame_counter on the LAST beat (tlast) of each outgoing frame.
// Popped by timestamp_arbiter when a matching I_TimeStamp is also ready.
//
// FWFT (First Word Fall Through):
//   timestamp_out is a combinational assign of mem[rd_ptr]
//   Data is valid BEFORE the pop strobe — arbiter reads it the same cycle
//   it checks !empty, no extra wait cycle needed.
//
// COUNT LOGIC:
//   Uses a priority-encoded case to handle all four combinations of
//   push/pop in a single assignment — avoids last-write-wins conflict
//   that occurs when two separate if-blocks both assign to count.
// ─────────────────────────────────────────────────────────────────────────────

module o_timestamp_fifo #(
    parameter DEPTH = 8
)(
    input  wire        clk,
    input  wire        rst_n,

    // Push side — from output_frame_counter
    input  wire        push,
    input  wire [63:0] timestamp_in,

    // Pop side — from timestamp_arbiter
    input  wire        pop,

    // Output — head of FIFO (FWFT: always valid when !empty)
    output wire [63:0] timestamp_out,
    output wire        empty,
    output wire        full
);

    reg [63:0]              mem    [0:DEPTH-1];
    reg [$clog2(DEPTH)-1:0] wr_ptr;
    reg [$clog2(DEPTH)-1:0] rd_ptr;
    reg [$clog2(DEPTH):0]   count;

    assign full          = (count == DEPTH);
    assign empty         = (count == 0);
    assign timestamp_out = mem[rd_ptr]; // FWFT: combinational read from head

    integer i;

    // Effective push/pop after guard checks
    wire do_push = push && !full;
    wire do_pop  = pop  && !empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= 64'd0;
        end else begin

            // ── PUSH — write data and advance write pointer ───────────────
            if (do_push) begin
                mem[wr_ptr] <= timestamp_in;
                wr_ptr      <= wr_ptr + 1;
            end

            // ── POP — advance read pointer ────────────────────────────────
            if (do_pop) begin
                rd_ptr <= rd_ptr + 1;
            end

            // ── COUNT — single assignment covering all four cases ─────────
            // Using do_push/do_pop wires avoids last-write-wins conflict
            // that happens when two separate if-blocks assign to count
            case ({do_push, do_pop})
                2'b10:   count <= count + 1; // push only
                2'b01:   count <= count - 1; // pop only
                2'b11:   count <= count;     // push + pop simultaneously: no change
                default: count <= count;     // no operation
            endcase

        end
    end

endmodule
