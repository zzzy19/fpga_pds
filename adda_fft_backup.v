// 文件名: project_top.v
// 描述: 整合了ADC采集、FFT处理和UART发送的最终顶层模块

`timescale 1ns / 1ps

module project_top (
    // 全局物理端口
    input  wire        clk_27M,       // 板载27MHz时钟
    input  wire        rst_n,         // 全局异步复位，低有效
    input  wire [7:0]  ad_data_in,    // 8位ADC数据输入

    // 外设控制端口
    output wire        ad_clk,        // 输出给ADC的采样时钟 (1MHz)
    output wire        uart_tx        // UART发送端口
);

    //================================================================
    // 1. 时钟和复位管理
    //================================================================

    // --- 1.1. 生成1MHz采样时钟 (ad_clk) ---
    reg        clk_1M;
    reg [4:0]  clk_cnt;
    always @(posedge clk_27M or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 0;
            clk_1M  <= 0;
        end else if (clk_cnt == 13) begin // 27MHz / 2 / 13.5 -> ~1MHz. (27分频，高低电平各13/14个周期)
            clk_cnt <= 0;
            clk_1M  <= ~clk_1M;
        end else begin
            clk_cnt <= clk_cnt + 1'd1;
        end
    end
    assign ad_clk = clk_1M;

    // --- 1.2. 上电延时，等待系统稳定 ---
    localparam PWAIT = 27_000_000; // 约1秒
    reg [24:0] pwcnt;
    reg        sys_ready; // 系统准备就绪信号

    always @(posedge clk_27M or negedge rst_n) begin
        if (!rst_n) begin
            pwcnt     <= 0;
            sys_ready <= 1'b0;
        end else if (!sys_ready) begin
            if (pwcnt == PWAIT - 1) begin
                sys_ready <= 1'b1;
            end else begin
                pwcnt <= pwcnt + 1'd1;
            end
        end
    end

    //================================================================
    // 2. 核心模块例化
    //================================================================
    
    // --- FFT分析模块的输入输出线网 ---
    // 输入流
    wire                        s_axis_data_tvalid;
    wire signed [15:0]          s_axis_data_tdata;
    wire                        s_axis_data_tlast;
    
    // 输出流
    wire                        m_axis_data_tvalid;
    wire signed [47:0]          m_axis_data_tdata;
    wire                        m_axis_data_tlast;
    
    // --- 例化信号分析仪 (FFT核心) ---
    signal_analyzer_top u_signal_analyzer (
        .clk(clk_27M),       // 使用系统高速时钟
        .rst_n(rst_n),
        .adc_data_in(1'b0), // 注意：这个端口不再直接使用，数据通过下面的流控制逻辑送入

        // --- 以下端口将在内部连接 ---
        .measured_frequency(), // 暂时悬空，我们将在下面逻辑中计算
        .measured_amplitude(), // 暂时悬空
        .pass_fail_flag()      // 暂时悬空
    );
    // 关键：为了复用 signal_analyzer_top，需要修改其内部，使其 AXI Stream 输入直接由顶层驱动
    // 或者，更简单的做法是直接将 signal_analyzer_top 内部的逻辑整合到这里。
    // 这里我们假设 signal_analyzer_top 被修改为接受AXI Stream输入，而不是adc_data_in
    // 并且我们在这里直接驱动其内部的FFT IP核。
    // 为了简化，我们直接在这里重新例化FFT IP。

    // --- 例化FFT IP核 (从 signal_analyzer_top 中拿出) ---
    fft_test u_fft_ip (
      .i_axi4s_data_tdata(s_axis_data_tdata),
      .i_axi4s_data_tvalid(s_axis_data_tvalid),
      .i_axi4s_data_tlast(s_axis_data_tlast),
      .o_axi4s_data_tready(), // tready可以悬空或连接，这里表示我们总能提供数据

      .i_axi4s_cfg_tdata(1'b0),
      .i_axi4s_cfg_tvalid(1'b0),

      .i_aclk(clk_27M),
      .i_aresetn(rst_n),

      .o_axi4s_data_tdata(m_axis_data_tdata),
      .o_axi4s_data_tvalid(m_axis_data_tvalid),
      .o_axi4s_data_tlast(m_axis_data_tlast),
      .o_axi4s_data_tuser(),
      
      .o_alm(),
      .o_stat()
    );


    //================================================================
    // 3. 数据输入流控制 (ADC -> FFT)
    //================================================================
    localparam FFT_POINTS = 1024;
    reg [10:0] sample_counter;
    reg        adc_capture_enable;

    // 使用 1MHz 时钟的边沿作为采样触发
    reg clk_1M_reg;
    wire clk_1M_posedge;
    always @(posedge clk_27M or negedge rst_n) begin
        if(!rst_n) clk_1M_reg <= 0;
        else       clk_1M_reg <= clk_1M;
    end
    assign clk_1M_posedge = (clk_1M == 1'b1) && (clk_1M_reg == 1'b0);

    // 控制数据流输入FFT模块
    assign s_axis_data_tvalid = adc_capture_enable;
    assign s_axis_data_tdata  = {8'b0, ad_data_in}; // 虚部为0，实部为ADC数据
    assign s_axis_data_tlast  = (sample_counter == FFT_POINTS - 1) && adc_capture_enable;

    always @(posedge clk_27M or negedge rst_n) begin
        if (!rst_n) begin
            sample_counter <= 0;
            adc_capture_enable <= 0;
        end else if (sys_ready) begin // 系统稳定后开始工作
            if (clk_1M_posedge) begin
                if (adc_capture_enable) begin
                    if (sample_counter == FFT_POINTS - 1) begin
                        sample_counter <= 0;
                        adc_capture_enable <= 0; // 采集完一帧后停止
                    end else begin
                        sample_counter <= sample_counter + 1;
                    end
                end else if (m_axis_data_tlast) begin 
                    // 在上一帧FFT计算完成后，且UART发送完成后（为简化，此处省略判断），再开始新一轮采集
                    // 此处简化为FFT计算一结束就开始下一轮
                     adc_capture_enable <= 1;
                     sample_counter <= 0;
                end else if (sample_counter == 0) begin
                    // 初始启动
                     adc_capture_enable <= 1;
                end
            end else if (adc_capture_enable && sample_counter == 0) begin
                // 在非采样时钟边沿，如果使能启动，则保持使能
                adc_capture_enable <= 1;
            end else begin
                adc_capture_enable <= 0;
            end
        end
    end


    //================================================================
    // 4. 数据输出处理 (FFT -> 峰值检测)
    //================================================================
    reg signed [23:0] fft_real;
    reg signed [23:0] fft_imag;
    
    // 幅度平方 (避免开方运算)
    reg [47:0] mag_sq; 
    reg [47:0] max_mag_sq;
    reg [9:0]  peak_index; // 峰值对应的频率点索引
    reg [9:0]  fft_out_counter;
    reg        analysis_done; // 分析完成标志

    always @(posedge clk_27M) begin
        fft_real <= m_axis_data_tdata[23:0];
        fft_imag <= m_axis_data_tdata[47:24];
        mag_sq   <= fft_real * fft_real + fft_imag * fft_imag;
    end

    always @(posedge clk_27M or negedge rst_n) begin
        if (!rst_n) begin
            max_mag_sq      <= 0;
            peak_index      <= 0;
            fft_out_counter <= 0;
            analysis_done   <= 0;
        end else begin
            analysis_done <= 0; // 默认拉低
            if (m_axis_data_tvalid) begin
                // 我们只关心前半部分频谱 (0 到 N/2)
                if (fft_out_counter < (FFT_POINTS / 2)) begin
                    if (mag_sq > max_mag_sq) begin
                        max_mag_sq <= mag_sq;
                        peak_index <= fft_out_counter;
                    end
                end
                
                if (m_axis_data_tlast) begin
                    fft_out_counter <= 0;
                    analysis_done <= 1'b1; // 一帧FFT数据处理完毕
                end else begin
                    fft_out_counter <= fft_out_counter + 1;
                end
            end
            
            // 当新一轮FFT开始时，清零上一轮的结果
            if (s_axis_data_tvalid && s_axis_data_tlast) begin
                max_mag_sq <= 0;
                peak_index <= 0;
            end
        end
    end

    //================================================================
    // 5. UART发送控制 (发送峰值频率和幅度)
    //================================================================
    reg [2:0]  send_state; // 发送状态机
    reg [7:0]  uart_data_out;
    reg        uart_start_send;
    wire       uart_tx_busy;

    always @(posedge clk_27M or negedge rst_n) begin
        if (!rst_n) begin
            send_state      <= 0;
            uart_data_out   <= 0;
            uart_start_send <= 0;
        end else begin
            uart_start_send <= 0; // 默认为单脉冲
            case (send_state)
                0: begin // 等待分析完成
                    if (analysis_done) begin
                        send_state <= 1;
                    end
                end
                1: begin // 发送帧头 (e.g., 0xA5)
                    uart_data_out   <= 8'hA5;
                    uart_start_send <= 1;
                    send_state      <= 2;
                end
                2: begin // 等待发送完成
                    if (!uart_tx_busy) begin
                        send_state <= 3;
                    end
                end
                3: begin // 发送频率索引高8位
                    uart_data_out   <= peak_index[9:2];
                    uart_start_send <= 1;
                    send_state      <= 4;
                end
                4: begin // 等待
                    if (!uart_tx_busy) begin
                        send_state <= 5;
                    end
                end
                5: begin // 发送频率索引低2位 (以及其他标志位)
                    uart_data_out   <= {peak_index[1:0], 6'b0};
                    uart_start_send <= 1;
                    send_state      <= 6;
                end
                6: begin // 等待
                    if (!uart_tx_busy) begin
                        // 此处可以继续发送幅度等信息
                        // 为简化，我们发送完毕，返回等待状态
                        send_state <= 0; 
                    end
                end
                default: send_state <= 0;
            endcase
        end
    end

    // --- 例化UART发送模块 ---
    uart_tx_only my_uart_tx (
        .clk        (clk_27M),
        .rst_n      (rst_n),
        .temp_data  (uart_data_out),
        .start_send (uart_start_send),
        .uart_tx    (uart_tx),
        .tx_busy    (uart_tx_busy)
    );

endmodule