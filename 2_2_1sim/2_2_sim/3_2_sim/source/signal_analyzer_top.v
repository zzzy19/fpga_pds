// �ļ���: signal_analyzer_top.v
// ����: ��Ŀ�Ķ���ģ�飬����������ģ�顣���ǽ���Ҫ�ۺϵĲ��֡�

`timescale 1ns / 1ps

module signal_analyzer_top (
    // ȫ���ź�
    input clk,
    input rst_n,

    // ADC����
    input signed [7:0] adc_data_in, // ����8λADC

    // ---- ��������� ----
    // (��Щ�����������ĵ���������������ջ���Ϊwire)
    output [15:0] measured_frequency, // ʾ��λ��
    output [15:0] measured_amplitude, // ʾ��λ��
    output pass_fail_flag
    // ... �������
);

    // ������㽫�������������������ʵ���� fft_wrapper �� parameter_calculator
    // ����:
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
    
    // ��ʱ��ֵ���Ա����
    assign measured_frequency = 0;
    assign measured_amplitude = 0;
    assign pass_fail_flag = 0;

endmodule