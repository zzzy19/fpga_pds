`timescale 1ns / 1ps

//================================================================
//--- 顶层模块 (修正版) ---
// 1. 修正了ADC数据从偏移二进制码到二进制补码的转换逻辑。
// 2. 保持了原有的“先存储后发送”架构，以最小化改动。
//================================================================
module top (
    input  wire        clk_27M,
    input  wire [7:0]  ad_data_in,
    input  wire        rst_n,
    output wire        ad_clk,
    output wire        uart_tx
);

//================================================================
// 1. 参数定义
//================================================================
// *** 请确保此参数与您在IP核生成工具中设置的 Transform Length 完全一致 ***
localparam FFT_POINTS = 1024; 

localparam FFT_AXI_INPUT_WIDTH = 16; 
localparam FFT_OUTPUT_WIDTH = 48;
localparam FFT_OUTPUT_BYTES = FFT_OUTPUT_WIDTH / 8;

localparam S_IDLE             = 4'd0;
localparam S_WAIT_BUFFERING   = 4'd1;
localparam S_FEED_FFT         = 4'd2;
localparam S_STORE_FFT_RESULT = 4'd3;
localparam S_SEND_UART        = 4'd4;
localparam S_DONE_PROC        = 4'd5;

//================================================================
// 2. 时钟与复位生成
//================================================================
wire clk_24M_from_pll, clk_5M_from_pll, pll_locked;
my_pll u_my_pll (.clkin1(clk_27M), .clkout0(clk_24M_from_pll), .clkout1(clk_5M_from_pll), .lock(pll_locked));

assign ad_clk = clk_5M_from_pll;
wire global_reset_n = rst_n & pll_locked;
wire rst_24M_n, rst_5M_n;

cdc_sync reset_sync_24M_inst (.i_dest_clk(clk_24M_from_pll), .i_dest_rst_n(1'b1), .i_async_in(global_reset_n), .o_sync_out(rst_24M_n));
cdc_sync reset_sync_5M_inst  (.i_dest_clk(clk_5M_from_pll),  .i_dest_rst_n(1'b1), .i_async_in(global_reset_n), .o_sync_out(rst_5M_n));

// 上电稳定延时
localparam PWAIT = 24_000_000;
reg [24:0] pwcnt;
reg        pwok;
always @(posedge clk_24M_from_pll or negedge rst_24M_n) begin
    if (!rst_24M_n) begin pwcnt <= 0; pwok  <= 0; end
    else if (!pwok) begin
        if (pwcnt == PWAIT - 1) pwok <= 1;
        else pwcnt <= pwcnt + 1'd1;
    end
end

//================================================================
// 3. 信号与缓冲器定义
//================================================================
wire       fifo_almost_full_raw, fifo_almost_full_synced;
wire [7:0] fifo_wr_data;
wire       fifo_wr_en, fifo_wr_full;
wire [7:0] fifo_rd_data;
wire       fifo_rd_en, fifo_rd_empty;

// 这个巨大的Buffer是您原有设计的核心，我们予以保留。
// 注意：当FFT_POINTS很大时，这会消耗大量FPGA片上RAM资源。
reg [FFT_OUTPUT_WIDTH-1:0] fft_result_buffer [0:FFT_POINTS-1];

reg [3:0] state;
reg        s_axis_data_tvalid;
reg signed [FFT_AXI_INPUT_WIDTH-1:0] s_axis_data_tdata; 
reg        s_axis_data_tlast;
wire       s_axis_data_tready;
reg [$clog2(FFT_POINTS)-1:0] fft_input_cnt;
wire       m_axis_data_tvalid;
wire [FFT_OUTPUT_WIDTH-1:0] m_axis_data_tdata;
wire       m_axis_data_tlast;
reg [$clog2(FFT_POINTS)-1:0] fft_output_cnt;
reg [7:0]  uart_data_out;
reg        uart_start_send;
wire       uart_tx_busy;
reg [$clog2(FFT_POINTS)-1:0] uart_point_cnt;
reg [$clog2(FFT_OUTPUT_BYTES)-1:0] uart_byte_cnt;


//================================================================
// 4. 主控制逻辑
//================================================================
assign fifo_wr_data = ad_data_in;
assign fifo_wr_en   = !fifo_wr_full;
assign fifo_rd_en   = (state == S_FEED_FFT) && s_axis_data_tready && !fifo_rd_empty;

cdc_sync almost_full_sync_inst (.i_dest_clk(clk_24M_from_pll), .i_dest_rst_n(rst_24M_n), .i_async_in(fifo_almost_full_raw), .o_sync_out(fifo_almost_full_synced));

always @(posedge clk_24M_from_pll or negedge rst_24M_n) begin
    if (!rst_24M_n) begin
        state <= S_IDLE; 
        s_axis_data_tvalid <= 0; 
        s_axis_data_tlast <= 0;
        s_axis_data_tdata <= 0; 
        fft_input_cnt <= 0; 
        fft_output_cnt <= 0;
        uart_start_send <= 0; 
        uart_point_cnt <= 0; 
        uart_byte_cnt <= 0;
    end
    else begin
        // 默认情况下，将控制信号置为非活动状态
        s_axis_data_tvalid <= 1'b0; 
        s_axis_data_tlast <= 1'b0; 
        uart_start_send <= 1'b0;

        case (state)
            S_IDLE:
                if (pwok) state <= S_WAIT_BUFFERING;

            S_WAIT_BUFFERING:
                // 等待FIFO快满，这是启动FFT处理的触发信号
                if (fifo_almost_full_synced) state <= S_FEED_FFT;

            S_FEED_FFT: begin
                // 只要FIFO不空，就准备好发送数据
                s_axis_data_tvalid <= !fifo_rd_empty;
                
                // ====================================================================
                // ====================   关键逻辑修正 (仅此一处)   ====================
                // ====================================================================
                // ADC输出通常是偏移二进制码 (0 ~ 255 代表负到正电压)。
                // 将其转换为FFT IP核需要的二进制补码 (-128 ~ 127)，需要将最高位(MSB)取反。
                // 您之前的代码 {fifo_rd_data[7], ~fifo_rd_data[6:0]} 是错误的，它会严重扭曲信号。
                s_axis_data_tdata  <= {8'd0, {~fifo_rd_data[7], fifo_rd_data[6:0]}};

                // 当IP核准备好(tready)并且我们有数据(FIFO不空)时，进行一次数据传输
                if (s_axis_data_tready && !fifo_rd_empty) begin
                    // 如果这是最后一个数据点
                    if (fft_input_cnt == FFT_POINTS - 1) begin
                        fft_input_cnt <= 0; 
                        s_axis_data_tlast <= 1'b1; // 发送tlast信号
                        state <= S_STORE_FFT_RESULT; // 进入存储结果状态
                    end else begin
                        fft_input_cnt <= fft_input_cnt + 1; // 增加输入点计数
                    end
                end
            end

            S_STORE_FFT_RESULT: begin
                // 当FFT IP核有有效输出时
                if (m_axis_data_tvalid) begin
                    // 将结果存入片上RAM Buffer
                    fft_result_buffer[fft_output_cnt] <= m_axis_data_tdata;
                    // 如果这是最后一个输出点
                    if (m_axis_data_tlast) begin
                        fft_output_cnt <= 0; 
                        state <= S_SEND_UART; // 进入UART发送状态
                    end else begin
                        fft_output_cnt <= fft_output_cnt + 1; // 增加输出点计数
                    end
                end
            end

            S_SEND_UART: begin
                // 当UART不忙时，可以发送下一个字节
                if (!uart_tx_busy) begin
                    uart_start_send <= 1'b1; // 启动UART发送
                    // 从Buffer中读取数据，并选择当前要发送的字节
                    uart_data_out <= fft_result_buffer[uart_point_cnt][(FFT_OUTPUT_BYTES-1-uart_byte_cnt)*8 +: 8];
                    
                    // 如果一个点的所有字节都发送完毕
                    if (uart_byte_cnt == FFT_OUTPUT_BYTES - 1) begin
                        uart_byte_cnt <= 0; // 字节计数器复位
                        // 如果所有的点都发送完毕
                        if (uart_point_cnt == FFT_POINTS - 1) begin
                            uart_point_cnt <= 0; 
                            state <= S_DONE_PROC; // 全部完成
                        end else begin
                            uart_point_cnt <= uart_point_cnt + 1; // 增加发送点计数
                        end
                    end else begin
                        uart_byte_cnt <= uart_byte_cnt + 1; // 增加字节计数
                    end
                end
            end

            S_DONE_PROC: begin 
                // 停留在完成状态，等待下一次复位
                state <= S_DONE_PROC; 
            end
            default:     begin state <= S_IDLE; end
        endcase
    end
end

//================================================================
// 5. IP核及模块实例化
//================================================================
my_FIFO fifo_inst (
  .wr_clk(clk_5M_from_pll), .wr_rst(!rst_5M_n), .wr_en(fifo_wr_en),
  .wr_data(fifo_wr_data), .wr_full(fifo_wr_full), .almost_full(fifo_almost_full_raw),
  .rd_clk(clk_24M_from_pll), .rd_rst(!rst_24M_n), .rd_en(fifo_rd_en),
  .rd_data(fifo_rd_data), .rd_empty(fifo_rd_empty), .almost_empty()
);

fft_test u_fft_ip (
    .i_aclk(clk_24M_from_pll), .i_aresetn(rst_24M_n), .i_axi4s_data_tvalid(s_axis_data_tvalid),
    .i_axi4s_data_tdata(s_axis_data_tdata), .i_axi4s_data_tlast(s_axis_data_tlast),
    .o_axi4s_data_tready(s_axis_data_tready), .i_axi4s_cfg_tvalid(1'b0),
    .i_axi4s_cfg_tdata(16'b0), .o_axi4s_data_tvalid(m_axis_data_tvalid),
    .o_axi4s_data_tdata(m_axis_data_tdata), .o_axi4s_data_tlast(m_axis_data_tlast)
);

uart_tx_only #(.BPS_NUM(208)) my_uart_tx (
    .clk(clk_24M_from_pll), .temp_data(uart_data_out), .start_send(uart_start_send),
    .uart_tx(uart_tx), .tx_busy(uart_tx_busy)
);

endmodule