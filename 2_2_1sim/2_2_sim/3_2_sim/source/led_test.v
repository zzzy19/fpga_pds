`timescale    1ns/1ns
`define UD #1
module    led_test
#(
    parameter    CNT_MAX = 26'd13_500_000
)
(
    input            clk         ,
    input            rstn        ,

    output   [7:0]   led    

);


//==============================================================================  
//reg and wire  
  
reg    [25:0]    led_light_cnt = 26'd0;
reg    [7:0]     led_status = 8'b0000_0001;
      
//time counter  
always@(posedge clk)
begin
    if(!rstn)
        led_light_cnt <= `UD 26'd0;
    else    if(led_light_cnt == CNT_MAX-1)
        led_light_cnt <= `UD 26'd0;
    else
        led_light_cnt <= `UD led_light_cnt + 26'd1;
end


//led status change 
always@(posedge clk)
begin
    if(!rstn)
        led_status <= `UD 8'b0000_0001;
    else    if(led_light_cnt == CNT_MAX-1)
        led_status <= `UD {led_status[6:0],led_status[7]};
end

assign led = led_status;


endmodule