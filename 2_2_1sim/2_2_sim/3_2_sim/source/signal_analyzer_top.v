// 文件名: signal_analyzer_top.v
// 描述: 项目的顶层模块，连接所有子模块。这是将来要综合的部分。

`timescale 1ns / 1ps

module signal_analyzer_top (
    // 全局信号
    input clk,
    input rst_n,

    // ADC输入
    input signed [7:0] adc_data_in, // 假设8位ADC

    // ---- 计算结果输出 ----
    // (这些是任务三、四的输出，现在先留空或定义为wire)
    output [15:0] measured_frequency, // 示例位宽
    output [15:0] measured_amplitude, // 示例位宽
    output pass_fail_flag
    // ... 其他输出
);

    // 在这里，你将会在任务二和任务三中实例化 fft_wrapper 和 parameter_calculator
    // 例如:
    // wire [...] fft_output_spectrum;
    //
    // fft_wrapper u_fft_wrapper (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .adc_data(adc_data_in),
    //     .spectrum_out(fft_output_spectrum)
    // );
    //
    // parameter_calculator u_param_calc (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .spectrum_in(fft_output_spectrum),
    //     .frequency_out(measured_frequency),
    //     .amplitude_out(measured_amplitude)
    // );
    
    // 临时赋值，以便仿真
    assign measured_frequency = 0;
    assign measured_amplitude = 0;
    assign pass_fail_flag = 0;

endmodule