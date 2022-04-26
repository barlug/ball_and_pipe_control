function action = set_pwm(device, pwm_value)
%% Sets the PWM of the fan
% Inputs:
%  ~ device: serialport object controlling the real world system
%  ~ pwm_value: A value from 0 to 4095 to set the pulse width modulation of
%  the actuator
% Outputs:
%  ~ action: the control action to change the PWM
%
% Created by:  Kyle Naddeo 1/3/2022
% Zachary Heras 2/9/2022
% Jacob King 2/23/2022
% Long H. Chau 

%% Force PWM value to be valid
% Constrain the pwm value to a minimum of 0 and a maximim of 4095
if pwm_value > 4095
    pwm_value=4095;
elseif pwm_value < 0
    pwm_value = 0;
end


%% Send Command
% action = % string value of pwm_value
% use the serialport() command options to change the PWM value to action
    action = strcat('p', string(pwm_value));    % convert pwm_value to a string, concatenated with 'p'
    device.writeline(action);                   % Writes the ASCII data "action" to the device serial port

end
