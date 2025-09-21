/**********************************************************************************
 * Module Name: signal_generator_tb (Final Corrected Version)
 * Description: 
 *   一个驱动 signal_analyzer_top 模块的Testbench。
 *   - 使用纯Verilog-2001兼容语法，移除了string类型。
 **********************************************************************************/

`timescale 1ns / 1ps

module signal_generator_tb;

    //================================================================
    // Parameters Definition
    //================================================================
    parameter WAVE_TYPE = 0; // 0: Sine, 1: Square, 2: Triangle
    parameter ADC_BITS      = 8;
    parameter CLK_PERIOD    = 10; // 10ns -> 100MHz
    parameter FFT_POINTS    = 1024;
    parameter SIGNAL_CYCLES = 10;
    parameter AMPLITUDE     = (2**(ADC_BITS-1)) - 1;
    parameter DC_OFFSET     = 0;
    localparam PI            = 3.1415926535;

    //================================================================
    // Signal and Variable Declaration
    //================================================================
    reg clk;
    reg rst_n;
    reg signed [ADC_BITS-1:0] adc_data_stimulus; 
    wire [15:0] freq_out;
    wire [15:0] amp_out;
    wire pf_flag_out;
    reg signed [ADC_BITS-1:0] wave_data_memory [0:FFT_POINTS-1];
    integer addr_counter = 0;
    
    //================================================================
    // Instantiate the Design Under Test (DUT)
    //================================================================
    signal_analyzer_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .adc_data_in(adc_data_stimulus),
        .measured_frequency(freq_out),
        .measured_amplitude(amp_out),
        .pass_fail_flag(pf_flag_out)
    );

    //================================================================
    // Clock and Reset Generation
    //================================================================
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    initial begin
        rst_n <= 0;
        generate_waveform();
        # (CLK_PERIOD * 5);
        rst_n <= 1;
    end

    //================================================================
    // Stimulus Generation
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_data_stimulus <= 0;
            addr_counter <= 0;
        end else begin
            adc_data_stimulus <= wave_data_memory[addr_counter];
            if (addr_counter == FFT_POINTS - 1) begin
                addr_counter <= 0;
            end else begin
                addr_counter <= addr_counter + 1;
            end
        end
    end

    //================================================================
    // Waveform Generation Task (纯Verilog-2001兼容版)
    //================================================================
    task generate_waveform;
        integer i;
        real temp_real;
        reg signed [ADC_BITS-1:0] temp_data;
        integer file_handle;
        integer half_period;
        integer pos_in_period;

        begin
            case (WAVE_TYPE)
                0: begin // Sine Wave
                    file_handle = $fopen("sine_wave.txt", "w");
                    for (i = 0; i < FFT_POINTS; i = i + 1) begin
                        temp_real = AMPLITUDE * $sin(2 * PI * SIGNAL_CYCLES * i / FFT_POINTS) + DC_OFFSET;
                        temp_data = temp_real;
                        wave_data_memory[i] = temp_data;
                        $fwrite(file_handle, "%d\n", temp_data);
                    end
                    $display("Waveform data generation complete for sine_wave.txt.");
                end
                1: begin // Square Wave
                    file_handle = $fopen("square_wave.txt", "w");
                    for (i = 0; i < FFT_POINTS; i = i + 1) begin
                        if ((i % (FFT_POINTS / SIGNAL_CYCLES)) < (FFT_POINTS / SIGNAL_CYCLES / 2))
                            temp_data = AMPLITUDE;
                        else
                            temp_data = -AMPLITUDE;
                        wave_data_memory[i] = temp_data;
                        $fwrite(file_handle, "%d\n", temp_data);
                    end
                    $display("Waveform data generation complete for square_wave.txt.");
                end
                2: begin // Triangle Wave
                    file_handle = $fopen("triangle_wave.txt", "w");
                    half_period = (FFT_POINTS / SIGNAL_CYCLES) / 2;
                    for (i = 0; i < FFT_POINTS; i = i + 1) begin
                        pos_in_period = i % (FFT_POINTS / SIGNAL_CYCLES);
                        if (pos_in_period < half_period)
                            temp_real = -AMPLITUDE + pos_in_period * (2.0 * AMPLITUDE / half_period);
                        else
                            temp_real = AMPLITUDE - (pos_in_period - half_period) * (2.0 * AMPLITUDE / half_period);
                        temp_data = temp_real;
                        wave_data_memory[i] = temp_data;
                        $fwrite(file_handle, "%d\n", temp_data);
                    end
                    $display("Waveform data generation complete for triangle_wave.txt.");
                end
                default: $display("Error: Invalid WAVE_TYPE selected!");
            endcase
            
            $fclose(file_handle);
        end
    endtask

endmodule