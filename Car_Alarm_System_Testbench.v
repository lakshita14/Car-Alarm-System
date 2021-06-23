// Code your testbench here
// or browse Examples
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.05.2021 08:34:13
// Design Name: 
// Module Name: test_CAS
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

module tb_Car_Alarm_System;
  reg Brake_depressed_switch , Hidden_switch , Ignition_switch , Driver_door , Passenger_door , Clk , System_reset;
  wire Fuel_pump_power , Siren ;
  wire Status_indicator;
  
 /*wire [1:0] interval;
  wire expired,start_timer, one_hz ;
 wire [2:0] state;
  wire [3:0] counter, value ; */
  
  
  
  Car_Alarm_System_S CAS(Brake_depressed_switch , Hidden_switch , Ignition_switch , Driver_door , Passenger_door  ,Clk, System_reset,Fuel_pump_power , Siren ,Status_indicator
  //,expired , start_timer , one_hz  , interval, value , counter ,state
);
  
  initial
    begin
      Clk = 0;
      forever
       #4 Clk = ~Clk;
    end
  
  initial
    begin
      $monitor($time , "Fuel_pump_power = %d , Siren = %d , Status_indicator = %b" , Fuel_pump_power , Siren , Status_indicator);
      System_reset = 1;  Ignition_switch = 0 ; Driver_door = 0 ; Passenger_door = 0; Brake_depressed_switch = 0; Hidden_switch = 0;
      
      #8 System_reset = 0;
    
      
      #8 Ignition_switch = 1;
      #8 Brake_depressed_switch = 1; Hidden_switch = 1;
      #64 Brake_depressed_switch = 0; Hidden_switch = 0;
      
      #16 Ignition_switch = 0; #8 Driver_door = 1; #8 Driver_door = 0; #24 Passenger_door = 1; #16 Passenger_door = 0;
      #320 Driver_door = 1;
      #64 Driver_door = 0;
      #800 Ignition_switch = 1; #8 Hidden_switch = 1; Brake_depressed_switch = 1;
      #72 Ignition_switch = 0;
      
      
      
     
      
     
    end
endmodule
