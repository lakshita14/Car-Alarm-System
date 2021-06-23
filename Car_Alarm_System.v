// Code your design here
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.05.2021 08:31:29
// Design Name: 
// Module Name: Car_Alarm_System
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Car_Alarm_System_S (
input Brake_depressed_switch , Hidden_switch , Ignition_switch , Driver_door , Passenger_door ,Clock, System_reset,
output Fuel_pump_power , Siren , Status_indicator
/*output expired,start_timer , one_hz,
output [1:0] interval,
output [3:0] value,
output [3:0] counter,
output [2:0] state*/
);
  wire [3:0] value; 
  wire [1:0] interval;
  wire expired,start_timer, one_hz;
 wire Brake_depressed_switch_d , Hidden_switch_d , Ignition_switch_d , Driver_door_d , Passenger_door_d , System_reset_d;
  
    Debouncer D1 (Clock, System_reset, System_reset_d);
    Debouncer D2 (Clock, Ignition_switch, Ignition_switch_d);
    Debouncer D3 (Clock, Brake_depressed_switch, Brake_depressed_switch_d);
    Debouncer D4 (Clock, Hidden_switch, Hidden_switch_d);
    Debouncer D5 (Clock, Driver_door, Driver_door_d);
    Debouncer D6 (Clock, Passenger_door, Passenger_door_d);
  
  fuel_pump_logic FPL(Clock,System_reset_d, Brake_depressed_switch_d , Hidden_switch_d , Ignition_switch_d , Fuel_pump_power);
  
  Anti_theft_FSM ATF( System_reset_d,Clock , Ignition_switch_d , Driver_door_d , Passenger_door_d, expired , one_hz , start_timer , Siren , Status_indicator , interval);
  
  time_parameters TP(interval,  Clock , value );
  
  timer T ( System_reset_d, Clock ,start_timer, value, expired, one_hz);
  
endmodule

module Debouncer (input clock, I, output reg O);

    reg [19:0] count = 20'b00000000000000000000;
    reg IReg;
    always @ (posedge clock)
        begin
            if (IReg != I)
                begin
                    IReg <= I;
                    count <= 20'b00000000000000000000;
                end
            else if (count == 1000000)
                O <= IReg;
            else
                count <= count + 1;
        end
endmodule

module fuel_pump_logic (
input Clock,System_reset,  Brake_depressed_switch , Hidden_switch , Ignition_switch,
  output reg Fuel_pump_power
);
  reg Fuel_pump_power_1;
  always @(System_reset or Ignition_switch or Brake_depressed_switch or Hidden_switch or Fuel_pump_power_1) 
    begin
      Fuel_pump_power = ~System_reset && Ignition_switch && Brake_depressed_switch && Hidden_switch || ~System_reset && Ignition_switch && Fuel_pump_power_1;
    end
    always @(posedge Clock)
     Fuel_pump_power_1 = Fuel_pump_power;
endmodule

module Anti_theft_FSM(
  input System_reset, clk ,ig_sw , driver_door , passenger_door, expired, one_hz,
  output reg start_timer , siren, output status_indicator, output reg [1:0] interval);
 
reg [2:0] state = 3'b000;
  
  // door open = 1; door close =0;
  parameter Armed = 3'b000,
            Triggered = 3'b001 ,
            Sound_Alarm = 3'b010 ,
            Disarmed = 3'b011 ,
            Ignition_off = 3'b 100 , 
            Door_open = 3'b101 ,
            Door_close = 3'b110,
            Alarm_on = 3'b111;

  reg [2:0] next_state;
  reg [1:0] led_state;

  always @(posedge clk)
    begin
    case(state)
      Armed : begin led_state <= 2'b01; siren <= 0; if(ig_sw == 0 && (driver_door == 1)) begin interval = 2'b01; start_timer <= 1;  end  else if(ig_sw == 0 && passenger_door == 1) begin  interval = 2'b10; start_timer <= 1;  end else start_timer <= 0;  end 
      
      Triggered : begin led_state <= 2'b10; siren <= 0;start_timer <= 0; end
      
      Sound_Alarm : begin led_state <= 2'b10; siren <= 1;if((driver_door == 0 && passenger_door == 0) || ig_sw == 1) begin  interval = 2'b11; start_timer <= 1; end   end
      
      Alarm_on : begin siren <= 1;start_timer <= 0; end
      
      Disarmed : begin led_state <= 2'b00 ; siren <= 0; end
      
      Ignition_off : begin led_state <= 2'b00; siren <= 0; start_timer <= 0; end
      
      Door_open : begin led_state <= 2'b00 ; siren <= 0; if(driver_door == 0 && passenger_door == 0) begin interval = 00;  start_timer <= 1;end end
      
      Door_close : begin start_timer <= 0;end     
    endcase 
    end
    
    always @(posedge clk)
    begin
    state = next_state;
    if(System_reset) 
       state = Armed;
      end
  
  always @(*)
    begin
      if(state == Armed)
        begin
          if(ig_sw == 1)
            next_state <= Disarmed;
          else if(driver_door == 1 || passenger_door == 1)
            next_state <= Triggered;
          else 
            next_state <= Armed;  
        end
      else if(state == Triggered)
        begin
          if(ig_sw == 1)
            next_state <= Disarmed;
          else if(expired == 1)
            next_state <= Sound_Alarm;
          else
             next_state <= Triggered;
        end
      else if(state == Sound_Alarm)
        begin
          if(driver_door == 0 && passenger_door == 0 || ig_sw == 1)
            next_state  <= Alarm_on;
          else
             next_state <= Sound_Alarm;
        end
      else if(state == Alarm_on)
        begin
          if(expired == 1)
            begin
              if(driver_door == 0 && passenger_door == 0)
                next_state <= Armed;
              else if(ig_sw ==1)
                next_state <= Disarmed;
              else
                next_state <= Alarm_on;
            end
          else
            next_state <= Alarm_on;
        end
     else if(state == Disarmed)
        begin
          if(ig_sw == 1)
            next_state <= Disarmed;
          else
            next_state <= Ignition_off;    
        end
      else if(state == Ignition_off)
        begin
          if(driver_door == 1)
            next_state <= Door_open;
            else
              next_state <= Ignition_off;
        end
      else if(state == Door_open)
        begin
          if(driver_door == 0 && passenger_door == 0)
            next_state <= Door_close;
          else
            next_state <= Door_open;
        end
      else
        begin
          if(expired)
            next_state <= Armed;
          else if(driver_door == 1 || passenger_door == 1)
            next_state <= Door_open;
          else
            next_state <= Door_close;
        end
        
    end
  led L(led_state ,clk, one_hz,status_indicator);
 endmodule
 
module led (input [1:0] status_indicator_led ,input clk ,one_hz, output  status_indicator);
reg status_indicator_2 = 0;

     assign status_indicator = status_indicator_2;
  always @(posedge one_hz)
    begin
      case(status_indicator_led)
        2'b 00 : status_indicator_2 <= 0;
        2'b 01 : status_indicator_2 <= ~status_indicator_2;
        2'b 10 : status_indicator_2 <= 1;
        2'b 11 : status_indicator_2 <= 0;
      endcase
    end
endmodule


module time_parameters (
  input [1:0]  interval, input clk,
  output reg [3:0] value
);
  always @(interval)    
    begin
      case(interval)
        2'b00 : value = 4'b0110; 
        2'b01 : value = 4'b0010;  // 1000
        2'b10 : value = 4'b1111;
        2'b11 : value = 4'b1010; 
        default : value = 4'b0000;
      endcase
    end
endmodule
  
 module timer (input System_reset , Clk , Start_timer , input [3:0] value , output reg expired , output enable);

reg reset , rst;
reg  expired_1 = 0 , expired_2 = 1 , expired_3 = 0;
reg count, count_1 = 1, count_2 = 0;
reg [3:0] counter, counter_1 , counter_2;
reg h, h_1 , h_2;

always @(Start_timer)
begin
 if(Start_timer) 
 rst = 1'b1; 
 else 
 rst = 1'b0;
 end
 
 always @(*)
reset = rst || System_reset;

//mod_8_counter FD(Clk ,reset ,Start_timer, enable);
freq_divider FD(Clk , reset ,Start_timer, enable);


always @(Start_timer)
h_1 = Start_timer ? 1'b0 : 1'b1;

always @(posedge Start_timer)
begin counter_1 = value;end

always @(posedge enable)
begin
  
     if(count && counter)
      begin counter_2 = counter - 1;h_2 = 1; end    
      else
      h_2 = 0; 
end

always @(*)
h = h_1 && h_2;

always @(posedge Clk)
begin
counter = Start_timer ? counter_1 : (h?  counter_2 : counter);
end
always @(posedge Clk)
begin
expired = (Start_timer && expired_1) || (~Start_timer  && expired && expired_3) || (enable  && count && !counter && expired_2);
count = (Start_timer && count_1) || (~Start_timer && !counter && count_2) || (~expired && ~Start_timer && count);
end
endmodule       
        
          
 module DFF (input clk , D , rst , output reg Q);
  always @ (posedge rst , posedge clk)
    begin
      if(rst == 1)
        Q <= 1'b1;
      else Q <= D;
    end
endmodule

module mod_8_counter (input clk , rst , start_timer, output final_out);
  wire [2:0] out;
  DFF A (clk , ~out[0] , rst , out[0]);
  DFF B(out[0] , ~out[1] , rst , out[1]);
  DFF C(out[1] , ~out[2] , rst , out[2]);
  assign final_out = out[2];
endmodule

module freq_divider (input clk , rst ,start_timer, output led);
  wire [8:0] Cout;
  
  mod_8_counter counter_inst0 (clk,rst,start_timer,Cout[0]);
  
genvar i;
generate
  for (i = 1; i < 9; i=i+1) 
begin : counter_gen_label
 mod_8_counter counter_inst (
 Cout[i-1],
 rst,start_timer,
 Cout[i]
 );
 end
endgenerate
  assign led = Cout[8];
endmodule
  
