% A MATLAB script to control Rowans Systems & Control Floating Ball 
% Apparatus designed by Mario Leone, Karl Dyer and Michelle Frolio. 
% The current control system is a PID controller.
%
% Created by Kyle Naddeo, Mon Jan 3 11:19:49 EST 
% Modified by Zachary Heras 2/9/2022
% Modified by Long H Chau 2/23/2022
% Modified by Jacob King 4/20/2022


%% Start fresh
close all; clc; clear device;

%% Connect to device
% device = open serial communication in the proper COM port
device = serialport('COM4', 19200);    % create an object that represents a serial client for communications with the seriel port

%% Parameters
target      = 0.5;   % Desired height of the ball [m]
sample_rate = 0.25;  % Amount of time between control actions [s]

set_pwm(device, 4000);
%while true
%% Give an initial burst to lift ball and keep in air
 % Initial burst to pick up ball
pause(0.75) % Wait 0.1 seconds
set_pwm(device, 2515);
pause(5)
set_pwm(device, 2750);
% set_pwm(add_proper_args); % Set to lesser value to level out somewhere in
% the pipe
%end

%% Initialize variables
action      = 2750; % Same value of last set_pwm   
error       = 0;
error_sum   = 0;

%% Feedback loop
while true
    %% Read current height
    [distance,pwm,target,deadpan] = read_data(device);
    y = ir2y(distance) % Convert from IR reading to distance from bottom [m]
    
    %% Calculate errors for PID controller
    error_prev = error;             % D
    error      = target - y;        % P
    error_sum  = error + error_sum; % I
    
    %% Control
    prev_action = action;
    %action = % Come up with a scheme no answer is right but do something
%     set_pwm(add_proper_args); % Implement action
        
    pause(sample_rate)              %Waits for next sample
end

