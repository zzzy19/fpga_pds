// =============================================================================
// == 文件名: top.v (已集成占空比显示功能)
// == 功能:   FFT频谱分析仪，并显示方波占空比
// == 版本:   7.0.0
// =============================================================================
`timescale 1ns / 1ps

module top(
    // ... 端口列表不变 ...
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

localparam REGION_X_START     = 12'd448;
localparam REGION_Y_END       = 12'd800;
localparam H_ACTIVE_PIXELS    = 1024;
localparam V_ACTIVE_PIXELS    = 1080; // 新增，用于OSD居中
localparam FONT_COLOR         = 24'hFFFF00;

localparam FONT_WIDTH_PIXELS  = 8;
localparam FONT_HEIGHT_PIXELS = 16;

localparam NUM_CHARS_FREQ     = 7;
localparam TEXT_WIDTH         = NUM_CHARS_FREQ * FONT_WIDTH_PIXELS;
localparam TEXT_HALF_WIDTH    = TEXT_WIDTH / 2;

localparam NUM_CHARS_AMP      = 4; 
localparam AMP_TEXT_WIDTH     = NUM_CHARS_AMP * FONT_WIDTH_PIXELS;

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
    if (!rst_24M_n) begin pwcnt <= 0; pwok <= 1'b0; end
    else if (!pwok) begin
        if (pwcnt == PWAIT - 1) pwok <= 1'b1;
        else pwcnt <= pwcnt + 1'b1;
    end
end
assign ad_clk   = clk_5M;
assign vout_clk = pix_clk;

//================================================================
// C. 信号与连线
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

reg [7:0] time_domain_peak_deviation_reg;
wire [7:0] current_deviation;
assign current_deviation = (signed_adc_data[7] == 1'b1) ? -signed_adc_data : signed_adc_data;

// --- NEW: 用于占空比计算的信号 ---
wire [6:0] duty_cycle_calculated;
wire       duty_cycle_valid;

//================================================================
// D. 模块实例化
//================================================================
my_FIFO fifo_inst ( .wr_clk(clk_5M), .wr_rst(!rst_5M_n), .wr_en(!fifo_wr_full), .wr_data(ad_data_in), .wr_full(fifo_wr_full), .rd_clk(clk_24M), .rd_rst(!rst_24M_n), .rd_en(fifo_rd_en), .rd_data(fifo_rd_data), .rd_empty(fifo_rd_empty), .almost_full(fifo_almost_full) );
fft_test u_fft_ip ( .i_axi4s_data_tdata(s_axis_data_tdata), .i_axi4s_data_tvalid(s_axis_data_tvalid), .i_axi4s_data_tlast(s_axis_data_tlast), .o_axi4s_data_tready(s_axis_data_tready), .i_axi4s_cfg_tdata(16'b0), .i_axi4s_cfg_tvalid(1'b0), .i_aclk(clk_24M), .i_aresetn(rst_24M_n), .o_axi4s_data_tdata(m_axis_data_tdata), .o_axi4s_data_tvalid(m_axis_data_tvalid), .o_axi4s_data_tlast(m_axis_data_tlast), .o_axi4s_data_tuser(m_axis_data_tuser), .o_alm(m_alm), .o_stat(m_stat) );
wire [$clog2(FFT_POINTS)-1:0] calc_addr;
(* DONT_TOUCH = "TRUE" *) wire signed [23:0] fft_real_part = fft_result_buffer[calc_addr][23:0];
(* DONT_TOUCH = "TRUE" *) wire signed [23:0] fft_imag_part = fft_result_buffer[calc_addr][47:24];
wire [48:0] power_val;
fft_power_calc u_fft_power_calc ( .clk(clk_24M), .rst_n(rst_24M_n), .real_in(fft_real_part), .imag_in(fft_imag_part), .power_out(power_val) );
wire [7:0] db_scaled_height;
power_to_db_approx db_height_scaler_inst ( .power_in(power_val), .height_out(db_scaled_height) );
wire [9:0] ram_wr_addr, ram_rd_addr;
wire       ram_wr_en;
wire [7:0] ram_rd_data;
dual_port_ram_1024x8 display_ram ( .a_addr(ram_wr_addr), .a_wr_data(db_scaled_height), .a_wr_en(ram_wr_en), .a_clk(clk_24M), .a_rst(!rst_24M_n), .a_rd_data(), .b_addr(ram_rd_addr), .b_rd_data(ram_rd_data), .b_clk(pix_clk), .b_rst(!global_reset_n), .b_wr_data(8'b0), .b_wr_en(1'b0) );

// --- NEW: 例化占空比计算模块 (在5MHz时钟域工作) ---
duty_cycle_calc u_duty_calc (
    .clk(clk_5M),
    .rst_n(rst_5M_n),
    .data_in(ad_data_in),
    .duty_cycle(duty_cycle_calculated),
    .valid_out(duty_cycle_valid)
);

//================================================================
// E. 主控制状态机 (FSM) @ 24MHz
//================================================================
reg [$clog2(FFT_POINTS)-1:0] calc_addr_cnt;
reg [15:0] wait_cnt;
reg [7:0]  peak_height_reg;
reg [9:0]  peak_freq_addr_reg;
reg [11:0] text_x_start_pos;
reg [31:0] peak_freq_hz_reg;
reg        osd_update_strobe; 
reg [31:0] peak_amplitude_mv_reg;
reg [6:0]  duty_cycle_reg; // NEW: 用于锁存占空比结果

// --- NEW: CDC for duty cycle data ---
reg        duty_valid_sync1, duty_valid_sync2;
reg [6:0]  duty_cycle_from_5M;
wire       duty_valid_synced = duty_valid_sync2;

always @(posedge clk_24M or negedge rst_24M_n) begin
    if (!rst_24M_n) begin
        duty_valid_sync1 <= 1'b0;
        duty_valid_sync2 <= 1'b0;
    end else begin
        duty_valid_sync1 <= duty_cycle_valid; // 来自5M时钟域
        duty_valid_sync2 <= duty_valid_sync1;
    end
end


wire [9:0] ram_rd_addr_from_display;
assign ram_rd_addr = ram_rd_addr_from_display;
assign fifo_rd_en  = (state == S_FEED_FFT) && !fifo_rd_empty;
assign calc_addr   = calc_addr_cnt;
assign ram_wr_addr = calc_addr_cnt;
assign ram_wr_en   = (state == S_CALC_POWER);
wire [11:0] peak_x_center;
assign peak_x_center = (peak_freq_addr_reg << 1) + REGION_X_START;

always @(posedge clk_24M) begin
    if (!rst_24M_n) begin
        state <= S_IDLE; s_axis_data_tvalid <= 1'b0; s_axis_data_tlast <= 1'b0; s_axis_data_tdata <= 16'd0; 
        fft_input_cnt <= 0; calc_addr_cnt <= 0; wait_cnt <= 0;
        peak_height_reg <= 0; peak_freq_addr_reg <= 0; text_x_start_pos <= 0; peak_freq_hz_reg <= 0; osd_update_strobe <= 1'b0;
        time_domain_peak_deviation_reg <= 0;
        peak_amplitude_mv_reg <= 0;
        duty_cycle_reg <= 0; // NEW
        duty_cycle_from_5M <= 0; // NEW
    end else begin
        osd_update_strobe <= 1'b0; 

        // NEW: 在24MHz时钟域捕获来自5MHz时钟域的占空比数据
        if (duty_valid_synced) begin
            duty_cycle_from_5M <= duty_cycle_calculated;
        end

        case (state)
            S_IDLE:             if (pwok) state <= S_WAIT_FIFO;
            S_WAIT_FIFO:        if (fifo_almost_full) begin 
                                    state <= S_FEED_FFT; 
                                    peak_height_reg <= 0;
                                    time_domain_peak_deviation_reg <= 0;
                                end
            S_FEED_FFT: begin
                s_axis_data_tvalid <= !fifo_rd_empty; 
                s_axis_data_tdata  <= {8'b0, signed_adc_data}; 
                s_axis_data_tlast  <= (fft_input_cnt == FFT_POINTS - 1);

                if (fifo_rd_en) begin
                    if (current_deviation > time_domain_peak_deviation_reg) begin
                        time_domain_peak_deviation_reg <= current_deviation;
                    end
                end

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
                    if (m_axis_data_tlast) state <= S_CALC_POWER;
                end
            end
            S_CALC_POWER: begin
                if (calc_addr_cnt > 3) begin
                    if(db_scaled_height > peak_height_reg && (calc_addr_cnt - 3) > 3) begin
                        peak_height_reg <= db_scaled_height; 
                        peak_freq_addr_reg <= calc_addr_cnt - 3;
                    end
                end
                if (calc_addr_cnt == FFT_POINTS/2 - 1) begin
                    calc_addr_cnt <= 0; 
                    state <= S_WAIT_FRAME;
                    if (peak_x_center < REGION_X_START + TEXT_HALF_WIDTH) begin 
                        text_x_start_pos <= REGION_X_START;
                    end else if (peak_x_center > (REGION_X_START + H_ACTIVE_PIXELS - TEXT_HALF_WIDTH)) begin 
                        text_x_start_pos <= REGION_X_START + H_ACTIVE_PIXELS - TEXT_WIDTH;
                    end else begin 
                        text_x_start_pos <= peak_x_center - TEXT_HALF_WIDTH; 
                    end
                    peak_freq_hz_reg <= peak_freq_addr_reg * 4883;
                    peak_amplitude_mv_reg <= time_domain_peak_deviation_reg * 39;
                    duty_cycle_reg <= duty_cycle_from_5M; // NEW: 锁存最新的占空比值
                    osd_update_strobe <= 1'b1;
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
// F. HDMI 显示流水线 & OSD
//================================================================
wire hdmi_vs, hdmi_hs, hdmi_de;
wire [11:0] act_x, act_y;
hdmi_driver u_hdmi_driver ( .pix_clk(pix_clk), .cfg_clk(hdmi_cfg_clk), .rstn(global_reset_n), .iic_tx_scl(iic_tx_scl), .iic_tx_sda(iic_tx_sda), .led_int(led_int), .vs_out(hdmi_vs), .hs_out(hdmi_hs), .de_out(hdmi_de), .act_x(act_x), .act_y(act_y) );

wire [23:0] spectrum_data_out;
wire [11:0] pos_x_from_spec, pos_y_from_spec;
spectrum_display u_spectrum_display ( .rst_n(global_reset_n), .pclk(pix_clk), .spectrum_color(24'h00FF00), .i_hs(hdmi_hs), .i_vs(hdmi_vs), .i_de(hdmi_de), .i_data(24'h000000), .ram_rd_addr(ram_rd_addr_from_display), .ram_rd_data(ram_rd_data), .o_hs(vout_hs), .o_vs(vout_vs), .o_de(vout_de), .o_data(spectrum_data_out), .o_pos_x(pos_x_from_spec), .o_pos_y(pos_y_from_spec) );

// --- CDC & OSD 数据锁存 ---
reg        osd_update_strobe_sync1, osd_update_strobe_sync2;
reg [11:0] text_x_start_pos_final;
reg [31:0] peak_freq_hz_final;
reg [31:0] peak_amplitude_mv_final;
reg [6:0]  duty_cycle_final; // NEW

always @(posedge pix_clk or negedge global_reset_n) begin
    if (!global_reset_n) begin
        osd_update_strobe_sync1 <= 1'b0; 
        osd_update_strobe_sync2 <= 1'b0;
        text_x_start_pos_final  <= 0; 
        peak_freq_hz_final      <= 0; 
        peak_amplitude_mv_final <= 0;
        duty_cycle_final        <= 0; // NEW
    end
    else begin
        osd_update_strobe_sync1 <= osd_update_strobe; 
        osd_update_strobe_sync2 <= osd_update_strobe_sync1;
        if (osd_update_strobe_sync2) begin
            text_x_start_pos_final  <= text_x_start_pos; 
            peak_freq_hz_final      <= peak_freq_hz_reg; 
            peak_amplitude_mv_final <= peak_amplitude_mv_reg;
            duty_cycle_final        <= duty_cycle_reg; // NEW
        end
    end
end

wire osd_freq_pixel_on;
wire osd_amp_pixel_on;
wire osd_duty_pixel_on; // NEW
wire [11:0] amp_text_x_start;

assign amp_text_x_start = text_x_start_pos_final + ((TEXT_WIDTH - AMP_TEXT_WIDTH) / 2);

// OSD 1: 显示频率
osd_renderer #(
    .CHAR_WIDTH(FONT_WIDTH_PIXELS),
    .CHAR_HEIGHT(FONT_HEIGHT_PIXELS),
    .NUM_DIGITS(NUM_CHARS_FREQ)
) freq_osd_inst (
    .pclk(pix_clk), .x(pos_x_from_spec), .y(pos_y_from_spec), .bin_in(peak_freq_hz_final),
    .x_start(text_x_start_pos_final), .y_start(REGION_Y_END + 5), .display_en(peak_height_reg > 10),
    .pixel_on(osd_freq_pixel_on)
);

// OSD 2: 显示幅值 (时域电压幅值)
osd_renderer #(
    .CHAR_WIDTH(FONT_WIDTH_PIXELS),
    .CHAR_HEIGHT(FONT_HEIGHT_PIXELS),
    .NUM_DIGITS(NUM_CHARS_AMP)
) amp_osd_inst (
    .pclk(pix_clk), .x(pos_x_from_spec), .y(pos_y_from_spec), 
    .bin_in(peak_amplitude_mv_final),
    .x_start(amp_text_x_start),
    .y_start(REGION_Y_END + 5 + FONT_HEIGHT_PIXELS + 2), 
    .display_en(peak_height_reg > 10),
    .pixel_on(osd_amp_pixel_on)
);

// --- NEW: OSD 3: 显示占空比 ---
osd_renderer #(
    .CHAR_WIDTH(FONT_WIDTH_PIXELS),
    .CHAR_HEIGHT(FONT_HEIGHT_PIXELS),
    .NUM_DIGITS(3) // 占空比 0-100，最多3位数
) duty_osd_inst (
    .pclk(pix_clk), .x(pos_x_from_spec), .y(pos_y_from_spec), 
    .bin_in(duty_cycle_final),
    // 将文本显示在屏幕中央
    .x_start( (H_ACTIVE_PIXELS - (3 * FONT_WIDTH_PIXELS)) / 2 ), 
    .y_start( (V_ACTIVE_PIXELS - FONT_HEIGHT_PIXELS) / 2 ), 
    .display_en(1'b1), // 始终显示
    .pixel_on(osd_duty_pixel_on)
);

// MODIFIED: 修改最终的像素输出逻辑以包含占空比OSD
assign vout_data = (osd_freq_pixel_on || osd_amp_pixel_on || osd_duty_pixel_on) ? FONT_COLOR : spectrum_data_out;

endmodule