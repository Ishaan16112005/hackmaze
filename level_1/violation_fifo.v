`timescale 1ns/1ps

module violation_fifo #(
    parameter DEPTH = 16,
    parameter DWIDTH = 96 // 64-bit Timestamp + 32-bit Latency
)(
    input  wire              clk,
    input  wire              rst_n,
    
    input  wire              push,
    input  wire [DWIDTH-1:0] data_in,
    
    input  wire              pop,
    output wire [DWIDTH-1:0] data_out,
    output wire              empty,
    output wire              full
);

    reg [DWIDTH-1:0] mem [0:DEPTH-1];
    reg [4:0] wr_ptr;
    reg [4:0] rd_ptr;
    reg [5:0] count;

    assign empty = (count == 0);
    assign full  = (count == DEPTH);
    assign data_out = empty ? {DWIDTH{1'b0}} : mem[rd_ptr[3:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end else begin
            case ({push & ~full, pop & ~empty})
                2'b10: begin // Push only
                    mem[wr_ptr[3:0]] <= data_in;
                    wr_ptr <= wr_ptr + 1;
                    count  <= count + 1;
                end
                2'b01: begin // Pop only
                    rd_ptr <= rd_ptr + 1;
                    count  <= count - 1;
                end
                2'b11: begin // Push and Pop simultaneously
                    mem[wr_ptr[3:0]] <= data_in;
                    wr_ptr <= wr_ptr + 1;
                    rd_ptr <= rd_ptr + 1;
                end
                default: ; // Do nothing
            endcase
        end
    end
endmodule
