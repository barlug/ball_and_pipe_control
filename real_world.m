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
sample_rate = 0.001;  % Amount of time between control actions [s]

%% Give an initial burst to lift ball and keep in air
 % Initial burst to pick up ball
set_pwm(device, 4000);  %Initialize the device to full power to overcome static friction
pause(0.75)             % Wait 0.1 seconds

set_pwm(device, 1800);  % Set to lesser value to level out somewhere in the pipe

%% Initialize variables
action      = 1800; % Same value of last set_pwm   
error       = 0;
error_sum   = 0;

%% Feedback loop
while true
    %% Read current height
    [distance,pwm,target,deadpan] = read_data(device);
    y = ir2y(distance) % Convert from IR reading to distance from bottom [m]
    
    %% Calculate errors for PID controller
    Matrix_error = [];
    error      = target - y;        % P
%     error_sum  = error + error_sum; % I
%     error_prev = error;             % D
   
    
    %% Control
    prev_action = action;
    PWM = PID_controller(error,Matrix_error,sample_rate);
    set_pwm(PWM)

    pause(sample_rate)              % Waits for next sample
    
    %If error > 0 pwm must increase
    %If error < 0 pwm must decrease
    
    %PID controller function
    %Implement error
    %action = % Come up with a scheme no answer is right but do something
    
    
    
    %set_pwm(add_proper_args); % Implement action
     
end

function PWM = PID_controller(error,Matrix_error,sample_rate)
    Kp = 71.5551;
    Ki = 0.2055;
    Kd = 13.6132

    Matrix_error = [Matrix_error error];
    P = Kp * Matrix_error(end);
    I = Ki * sum(Matrix_error);
    D = Kd * ((Matrix_error(end)- Matrix_error(end-1))/sample_rate);
    PWM = P + I + D;
end