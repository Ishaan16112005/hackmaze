module timestamp_arbiter (
    input  wire        clk,
    input  wire        rst_n,

    // I_FIFO interface
    input  wire        i_fifo_empty,
    input  wire [63:0] i_fifo_timestamp,
    output wire        i_fifo_pop,        // Changed to wire

    // O_FIFO interface
    input  wire        o_fifo_empty,
    input  wire [63:0] o_fifo_timestamp,
    output wire        o_fifo_pop,        // Changed to wire

    // To latency_calc
    output reg  [63:0] t_start,
    output reg  [63:0] t_end,
    output reg         trigger  
);

    // Combinational pop generation: fire immediately when both are ready
    wire pair_ready = !i_fifo_empty && !o_fifo_empty;
    assign i_fifo_pop = pair_ready;
    assign o_fifo_pop = pair_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_start <= 64'd0;
            t_end   <= 64'd0;
            trigger <= 1'b0;
        end else begin
            // Default de-assert
            trigger <= 1'b0;

            if (pair_ready) begin
                t_start <= i_fifo_timestamp;
                t_end   <= o_fifo_timestamp;
                trigger <= 1'b1;
            end
        end
    end

endmodule
