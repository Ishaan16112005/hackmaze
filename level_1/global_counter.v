`timescale 1ns/1ps
module global_timer (
    input  wire        clk,
    input  wire        rst_n,
    output reg [63:0]  timer_out
);
      
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            timer_out <= 64'd0;
        else 
            timer_out <= timer_out + 1;
    end
endmodule
