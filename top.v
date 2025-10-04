// 文件名: top.v (根据官方IP文档核对后的最终版)
`timescale 1ns / 1ps

module top (
    input  wire        clk_27M,
    input  wire [7:0]  ad_data_in,
    input  wire        rst_n,
    output wire        ad_clk,
    output wire        uart_tx
);

//================================================================
// 参数定义 (与之前相同)
//================================================================
localparam FFT_POINTS = 1024;
localparam FFT_INPUT_WIDTH  = 8;//changed
localparam FFT_OUTPUT_WIDTH = 48;
localparam FFT_OUTPUT_BYTES = FFT_OUTPUT_WIDTH / 8;

localparam S_IDLE             = 3'd0;
localparam S_FEED_FFT         = 3'd1;
localparam S_STORE_FFT_RESULT = 3'd2;
localparam S_SEND_UART        = 3'd3;
localparam S_DONE_PROC        = 3'd4;


//================================================================
// 1. 时钟与复位生成 (与之前相同)
//================================================================
wire clk_24M_from_pll;
wire clk_5M_from_pll;
wire pll_locked;

my_pll u_my_pll (
    .clkin1(clk_27M),
    .clkout0(clk_24M_from_pll),
    .clkout1(clk_5M_from_pll),
    .lock(pll_locked)
);

assign ad_clk = clk_5M_from_pll;

wire global_reset_n = rst_n & pll_locked;

wire rst_24M_n;
wire rst_5M_n;

cdc_sync reset_sync_24M_inst (
    .i_dest_clk   (clk_24M_from_pll),
    .i_dest_rst_n (1'b1),
    .i_async_in   (global_reset_n),
    .o_sync_out   (rst_24M_n)
);

cdc_sync reset_sync_5M_inst (
    .i_dest_clk   (clk_5M_from_pll),
    .i_dest_rst_n (1'b1),
    .i_async_in   (global_reset_n),
    .o_sync_out   (rst_5M_n)
);

localparam PWAIT = 24_000_000;
reg [24:0] pwcnt;
reg        pwok;
always @(posedge clk_24M_from_pll or negedge rst_24M_n) begin
    if (!rst_24M_n) begin
        pwcnt <= 0;
        pwok  <= 0;
    end
    else if (!pwok) begin
        if (pwcnt == PWAIT - 1) begin
            pwok  <= 1;
        end else begin
            pwcnt <= pwcnt + 1'd1;
        end
    end
end

//================================================================
// 2. 数据缓冲器与信号定义 (与之前相同)
//================================================================
wire [7:0] fifo_wr_data;
wire       fifo_wr_en;
wire       fifo_wr_full;
wire [7:0] fifo_rd_data;
wire       fifo_rd_en;
wire       fifo_rd_empty; // FIFO的空标志信号

reg [FFT_OUTPUT_WIDTH-1:0] fft_result_buffer [0:FFT_POINTS-1];

reg [2:0] state;
reg        s_axis_data_tvalid;
reg signed [FFT_INPUT_WIDTH-1:0] s_axis_data_tdata;
reg        s_axis_data_tlast;
wire       s_axis_data_tready;
reg [$clog2(FFT_POINTS)-1:0] fft_input_cnt;
wire       m_axis_data_tvalid;
wire [FFT_OUTPUT_WIDTH-1:0] m_axis_data_tdata;
wire       m_axis_data_tlast;
reg [$clog2(FFT_POINTS)-1:0] fft_output_cnt;
reg        fft_storing_done;
reg [7:0]  uart_data_out;
reg        uart_start_send;
wire       uart_tx_busy;
reg [$clog2(FFT_POINTS)-1:0] uart_point_cnt;
reg [$clog2(FFT_OUTPUT_BYTES)-1:0] uart_byte_cnt;


//================================================================
// 4. 主控制逻辑 (与之前相同)
//================================================================
assign fifo_wr_data = ad_data_in;
assign fifo_wr_en   = !fifo_wr_full;

assign fifo_rd_en = (state == S_FEED_FFT) && s_axis_data_tready && !fifo_rd_empty;

always @(posedge clk_24M_from_pll or negedge rst_24M_n) begin
    if (!rst_24M_n) begin
        state <= S_IDLE;
        s_axis_data_tvalid <= 0;
        s_axis_data_tlast <= 0;
        s_axis_data_tdata <= 0;
        fft_input_cnt <= 0;
        fft_output_cnt <= 0;
        fft_storing_done <= 0;
        uart_start_send <= 0;
        uart_point_cnt <= 0;
        uart_byte_cnt <= 0;
    end
    else begin
        uart_start_send <= 0;

        case (state)
            S_IDLE:
                if (pwok) state <= S_FEED_FFT;
            
            S_FEED_FFT: begin
                s_axis_data_tvalid <= !fifo_rd_empty;
                s_axis_data_tdata <= fifo_rd_data -  8'd128;//changed

                if (fifo_rd_en) begin
                    if (fft_input_cnt == FFT_POINTS - 1) begin
                        fft_input_cnt <= 0;
                        s_axis_data_tlast <= 1'b1;
                        state <= S_STORE_FFT_RESULT;
                    end else begin
                        fft_input_cnt <= fft_input_cnt + 1;
                        s_axis_data_tlast <= 1'b0;
                    end
                end else begin
                    s_axis_data_tlast <= 1'b0;
                end
                
                if(state != S_FEED_FFT) begin
                    s_axis_data_tvalid <= 1'b0;
                end
            end

            S_STORE_FFT_RESULT: begin
                if (m_axis_data_tvalid) begin
                    fft_result_buffer[fft_output_cnt] <= m_axis_data_tdata;
                    if (m_axis_data_tlast) begin
                        fft_output_cnt <= 0;
                        fft_storing_done <= 1'b1;
                        state <= S_SEND_UART;
                    end else begin
                        fft_output_cnt <= fft_output_cnt + 1;
                    end
                end
            end

            S_SEND_UART: begin
                if (!uart_tx_busy) begin
                    uart_start_send <= 1'b1;
                    uart_data_out <= fft_result_buffer[uart_point_cnt][(FFT_OUTPUT_BYTES-1-uart_byte_cnt)*8 +: 8];
                    if (uart_byte_cnt == FFT_OUTPUT_BYTES - 1) begin
                        uart_byte_cnt <= 0;
                        if (uart_point_cnt == FFT_POINTS - 1) begin
                            uart_point_cnt <= 0;
                            state <= S_DONE_PROC;
                        end else begin
                            uart_point_cnt <= uart_point_cnt + 1;
                        end
                    end else begin
                        uart_byte_cnt <= uart_byte_cnt + 1;
                    end
                end
            end

            S_DONE_PROC:
                state <= S_DONE_PROC;

            default:
                state <= S_IDLE;
        endcase
    end
end

//================================================================
// 5. IP核及模块实例化
//================================================================
// *** 实例化异步FIFO ***
my_FIFO fifo_inst (
  .wr_clk(clk_5M_from_pll),
  .wr_rst(!rst_5M_n),             // 文档确认: IP核为高电平复位
  .wr_en(fifo_wr_en),
  .wr_data(fifo_wr_data),
  .wr_full(fifo_wr_full),
  .almost_full(),
  .rd_clk(clk_24M_from_pll),
  .rd_rst(!rst_24M_n),             // 文档确认: IP核为高电平复位
  .rd_en(fifo_rd_en),
  .rd_data(fifo_rd_data),
  .rd_empty(fifo_rd_empty),       // *** BUG FIX: 此处端口连接已修正 ***
  .almost_empty()
);


fft_test u_fft_ip (
    .i_aclk(clk_24M_from_pll),
    .i_aresetn(rst_24M_n),
    .i_axi4s_data_tvalid(s_axis_data_tvalid),
    .i_axi4s_data_tdata(s_axis_data_tdata),
    .i_axi4s_data_tlast(s_axis_data_tlast),
    .o_axi4s_data_tready(s_axis_data_tready),
    .i_axi4s_cfg_tvalid(1'b0),
    .i_axi4s_cfg_tdata(16'b0),
    .o_axi4s_data_tvalid(m_axis_data_tvalid),
    .o_axi4s_data_tdata(m_axis_data_tdata),
    .o_axi4s_data_tlast(m_axis_data_tlast)
);

uart_tx_only #(
    .BPS_NUM(208)
) my_uart_tx (
    .clk(clk_24M_from_pll),
    .temp_data(uart_data_out),
    .start_send(uart_start_send),
    .uart_tx(uart_tx),
    .tx_busy(uart_tx_busy)
);

endmodule