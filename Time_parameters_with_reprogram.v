module time_parameters (
  input [1:0] time_parameter_selector , interval , input [3:0] time_value, input reprogram ,clk,
  output reg [3:0] value
);
  reg [3:0] Arming_delay = 4'b0110;
  reg [3:0] Countdown_driver_door = 4'b1000 ;
  reg [3:0] Countdown_passenger_door = 4'b1111;
  reg [3:0] Siren_ON_time= 4'b1010;
  
  wire [3:0] T_ARM_DELAY , T_DRIVER_DELAY , T_PASSENGER_DELAY , T_ALARM_ON ;
  
  assign T_ARM_DELAY = Arming_delay;
  assign T_DRIVER_DELAY = Countdown_driver_door;
  assign T_PASSENGER_DELAY = Countdown_passenger_door;
  assign T_ALARM_ON = Siren_ON_time; 
  always @(*)
    begin
      case(interval)
        2'b00 : value <= T_ARM_DELAY;
        2'b01 : value <= T_DRIVER_DELAY;
        2'b10 : value <= T_PASSENGER_DELAY;
        2'b11 : value <= T_ALARM_ON;
      endcase
    end
  always @(*)
    begin
      if(reprogram)
        begin
          case(time_parameter_selector)
            2'b00 : Arming_delay <= time_value;
            2'b01 : Countdown_driver_door <= time_value;
            2'b10 : Countdown_passenger_door <= time_value;
            2'b11 : Siren_ON_time <= time_value;
          endcase
        end
        
    end
endmodule