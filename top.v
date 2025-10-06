// =============================================================================
// == 文件名: top.v (最终版 - 移除背景网格)
// == 功能:   FFT频谱分析仪
// == 版本:   4.0.0
// =============================================================================
`timescale 1ns / 1ps
module top(
input  wire        clk_27M,
input  wire        rst_n,
input  wire [7:0]  ad_data_in,
output wire        ad_clk,
output wire        iic_tx_scl,
inout  wire        iic_tx_sda,
output wire        led_int,
output wire        vout_hs,
output wire        vout_vs,
output wire        vout_de,
output wire        vout_clk,
output wire [23:0] vout_data
);
//================================================================
// A. 参数定义
//================================================================
localparam FFT_POINTS         = 1024;
localparam FFT_INPUT_WIDTH    = 16;
localparam FFT_OUTPUT_WIDTH   = 48;
localparam S_IDLE             = 4'd0;
localparam S_WAIT_FIFO        = 4'd1;
localparam S_FEED_FFT         = 4'd2;
localparam S_STORE_FFT_RESULT = 4'd3;
localparam S_CALC_POWER       = 4'd4;
localparam S_WAIT_FRAME       = 4'd5;
//================================================================
// B. 时钟与复位生成
//================================================================
wire clk_24M, clk_5M, pix_clk, hdmi_cfg_clk;
wire pll_hdmi_locked, pll_system_locked, pll_locked;
pll_hdmi u_pll_hdmi ( .clkin1(clk_27M), .clkout0(pix_clk), .clkout1(hdmi_cfg_clk), .lock(pll_hdmi_locked) );
pll_system u_pll_system ( .clkin1(clk_27M), .clkout0(clk_24M), .clkout1(clk_5M), .lock(pll_system_locked) );
assign pll_locked = pll_hdmi_locked & pll_system_locked;
wire global_reset_n = rst_n & pll_locked;
wire rst_24M_n, rst_5M_n;
cdc_sync reset_sync_24M_inst ( .i_dest_clk(clk_24M), .i_dest_rst_n(1'b1), .i_async_in(global_reset_n), .o_sync_out(rst_24M_n) );
cdc_sync reset_sync_5M_inst ( .i_dest_clk(clk_5M), .i_dest_rst_n(1'b1), .i_async_in(global_reset_n), .o_sync_out(rst_5M_n) );
localparam PWAIT = 24_000;
reg [15:0] pwcnt;
reg        pwok;
always @(posedge clk_24M) begin
if (!rst_24M_n) begin
pwcnt <= 0;
pwok <= 1'b0;
end
else if (!pwok) begin
if (pwcnt == PWAIT - 1) begin
pwok <= 1'b1;
end
else begin
pwcnt <= pwcnt + 1'b1;
end
end
end
assign ad_clk   = clk_5M;
assign vout_clk = pix_clk;
//================================================================
// C & D 部分: IP核实例化与信号连接
//================================================================
wire [7:0] fifo_rd_data;
wire       fifo_rd_empty, fifo_almost_full, fifo_wr_full, fifo_rd_en;
wire       s_axis_data_tready;
reg        s_axis_data_tvalid;
reg signed [FFT_INPUT_WIDTH-1:0] s_axis_data_tdata;
reg        s_axis_data_tlast;
reg [$clog2(FFT_POINTS)-1:0] fft_input_cnt;
wire       m_axis_data_tvalid;
wire [FFT_OUTPUT_WIDTH-1:0] m_axis_data_tdata;
wire       m_axis_data_tlast;
wire [23:0] m_axis_data_tuser;
wire [2:0]  m_alm;
wire        m_stat;
reg [3:0] state;
reg [FFT_OUTPUT_WIDTH-1:0] fft_result_buffer [0:FFT_POINTS-1];
wire signed [7:0] signed_adc_data;
assign signed_adc_data = fifo_rd_data - 8'sd128;
my_FIFO fifo_inst (
.wr_clk(clk_5M), .wr_rst(!rst_5M_n), .wr_en(!fifo_wr_full), .wr_data(ad_data_in), .wr_full(fifo_wr_full),
.rd_clk(clk_24M), .rd_rst(!rst_24M_n), .rd_en(fifo_rd_en), .rd_data(fifo_rd_data), .rd_empty(fifo_rd_empty), .almost_full(fifo_almost_full)
);
fft_test u_fft_ip (
.i_axi4s_data_tdata(s_axis_data_tdata), .i_axi4s_data_tvalid(s_axis_data_tvalid), .i_axi4s_data_tlast(s_axis_data_tlast),
.o_axi4s_data_tready(s_axis_data_tready), .i_axi4s_cfg_tdata(16'b0), .i_axi4s_cfg_tvalid(1'b0), .i_aclk(clk_24M), .i_aresetn(rst_24M_n),
.o_axi4s_data_tdata(m_axis_data_tdata), .o_axi4s_data_tvalid(m_axis_data_tvalid), .o_axi4s_data_tlast(m_axis_data_tlast),
.o_axi4s_data_tuser(m_axis_data_tuser), .o_alm(m_alm), .o_stat(m_stat)
);
wire [$clog2(FFT_POINTS)-1:0] calc_addr;
(* DONT_TOUCH = "TRUE" ) wire signed [23:0] fft_real_part = fft_result_buffer[calc_addr][23:0];
( DONT_TOUCH = "TRUE" *) wire signed [23:0] fft_imag_part = fft_result_buffer[calc_addr][47:24];
wire [48:0] power_val;
fft_power_calc u_fft_power_calc (
.clk(clk_24M), .rst_n(rst_24M_n), .real_in(fft_real_part), .imag_in(fft_imag_part), .power_out(power_val)
);
wire [7:0] db_scaled_height;
power_to_db_approx db_height_scaler_inst (
.power_in(power_val),
.height_out(db_scaled_height)
);
wire [9:0] ram_wr_addr, ram_rd_addr;
wire       ram_wr_en;
wire [7:0] ram_rd_data;
dual_port_ram_1024x8 display_ram (
.a_addr(ram_wr_addr),
.a_wr_data(db_scaled_height),
.a_wr_en(ram_wr_en),
.a_clk(clk_24M),
.a_rst(!rst_24M_n),
.a_rd_data(),
.b_addr(ram_rd_addr), .b_rd_data(ram_rd_data), .b_clk(pix_clk), .b_rst(!global_reset_n), .b_wr_data(8'b0), .b_wr_en(1'b0)
);
//================================================================
// E. 主控制状态机 (FSM) @ 24MHz
//================================================================
reg [$clog2(FFT_POINTS)-1:0] calc_addr_cnt;
reg [15:0] wait_cnt;
assign fifo_rd_en = (state == S_FEED_FFT) && !fifo_rd_empty;
assign calc_addr   = calc_addr_cnt;
assign ram_wr_addr = calc_addr_cnt;
assign ram_wr_en   = (state == S_CALC_POWER);
always @(posedge clk_24M) begin
if (!rst_24M_n) begin
state <= S_IDLE;
s_axis_data_tvalid <= 1'b0;
s_axis_data_tlast <= 1'b0;
s_axis_data_tdata <= 16'd0;
fft_input_cnt <= 0;
calc_addr_cnt <= 0;
wait_cnt <= 0;
end else begin
case (state)
S_IDLE:             if (pwok) state <= S_WAIT_FIFO;
S_WAIT_FIFO:        if (fifo_almost_full) state <= S_FEED_FFT;
S_FEED_FFT: begin
s_axis_data_tvalid <= !fifo_rd_empty;
s_axis_data_tdata  <= {8'b0, signed_adc_data};
s_axis_data_tlast  <= (fft_input_cnt == FFT_POINTS - 1);
if (s_axis_data_tvalid) begin
if (fft_input_cnt == FFT_POINTS - 1) begin
fft_input_cnt <= 0;
state <= S_STORE_FFT_RESULT;
end else begin
fft_input_cnt <= fft_input_cnt + 1;
end
end
end
S_STORE_FFT_RESULT: begin
s_axis_data_tvalid <= 1'b0;
s_axis_data_tlast <= 1'b0;
if (m_axis_data_tvalid) begin
fft_result_buffer[m_axis_data_tuser[9:0]] <= m_axis_data_tdata;
if (m_axis_data_tlast) begin
state <= S_CALC_POWER;
end
end
end
S_CALC_POWER: begin
if (calc_addr_cnt == FFT_POINTS - 1) begin
calc_addr_cnt <= 0;
state <= S_WAIT_FRAME;
end else begin
calc_addr_cnt <= calc_addr_cnt + 1;
end
end
S_WAIT_FRAME: begin
if (wait_cnt == 24000 - 1) begin
wait_cnt <= 0;
state <= S_WAIT_FIFO;
end else begin
wait_cnt <= wait_cnt + 1;
end
end
default: state <= S_IDLE;
endcase
end
end
//================================================================
// F. HDMI 显示流水线 (已移除背景网格)
//================================================================
wire hdmi_vs, hdmi_hs, hdmi_de;
hdmi_driver u_hdmi_driver (
.pix_clk(pix_clk), .cfg_clk(hdmi_cfg_clk), .rstn(global_reset_n), .iic_tx_scl(iic_tx_scl), .iic_tx_sda(iic_tx_sda),
.led_int(led_int), .vs_out(hdmi_vs), .hs_out(hdmi_hs), .de_out(hdmi_de),
.act_x(), .act_y()
);
// <<< MODIFICATION 1: 禁用 grid_display 模块 >>>
// 我们不再需要 grid_display 模块，可以直接将其注释掉或删除。
/*
wire [23:0] grid_data_out;
wire grid_vs, grid_hs, grid_de;
grid_display u_grid_display (
.rst_n(global_reset_n), .pclk(pix_clk), .i_hs(hdmi_hs), .i_vs(hdmi_vs), .i_de(hdmi_de),
.i_data(24'b0),
.o_hs(grid_hs), .o_vs(grid_vs), .o_de(grid_de), .o_data(grid_data_out)
);
*/
// <<< MODIFICATION 2: 修改 spectrum_display 的输入连接 >>>
// 将 u_spectrum_display 的输入直接连接到 u_hdmi_driver 的输出
spectrum_display u_spectrum_display (
.rst_n(global_reset_n),
.pclk(pix_clk),
.spectrum_color(24'h00FF00),
// 将时序信号直接从 hdmi_driver 接入
.i_hs(hdmi_hs),
.i_vs(hdmi_vs),
.i_de(hdmi_de),
// 将背景数据输入固定为黑色 (24'h000000)
.i_data(24'h000000),
.ram_rd_addr(ram_rd_addr),
.ram_rd_data(ram_rd_data),
// 将输出直接连接到顶层端口
.o_hs(vout_hs),
.o_vs(vout_vs),
.o_de(vout_de),
.o_data(vout_data)
);
endmodule
