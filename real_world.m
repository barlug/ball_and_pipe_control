% A MATLAB script to control Rowans Systems & Control Floating Ball 
% Apparatus designed by Mario Leone, Karl Dyer and Michelle Frolio. 
% The current control system is a PID controller.
%
% Created by Kyle Naddeo, Mon Jan 3 11:19:49 EST 
% Modified by Zachary Heras 2/9/2022
% Modified by Long H Chau 4/26/2022
% Modified by Jacob King 4/20/2022
% Modified by Robert Kerwin 4/26/2022

%% Start fresh
clear variables;close all; clc; clear device;

%% Connect to device
% device = open serial communication in the proper COM port
device = serialport('COM6', 19200);    % create an object that represents a serial client for communications with the seriel port

%% Parameters
target      = 0.5;    % Desired height of the ball [m]
sample_rate = 0.25;  % Amount of time between control actions [s]

%% Give an initial burst to lift ball and keep in air
% Initial burst to pick up ball
set_pwm(device, 3000);  %Initialize the device to full power to overcome static friction
pause(0.75)             % Wait 0.75 seconds

set_pwm(device, 500);  % Set to lesser value to level out somewhere in the pipe

%% Initialize variables
action      = 500; % Same value of last set_pwm   
error       = 0;
error_sum   = 0;
Matrix_error = [-0.5];
%% Feedback loop
while true
    [distance,pwm,target1,deadpan] = read_data(device); % Read current height
    [y, pipePercentage] = ir2y(distance); % Convert from IR reading to distance from bottom [m]
    
    %% Calculate errors for PID controller
   
    error      = target - y;            % P
%     error_sum  = error + error_sum;   % I
%     error_prev = error;               % D
   
    
    %% PID Controller
    %PWM = PID_controller(error,sample_rate)   %Calls the PID_controller function to find the PWM value
    
    Kp = 44.1767;               % Proportional gain value found from the Genetic Algorithm
    Ki = 3.9588e-4;             % Integral gain value found from the Genetic Algorithm
    Kd = -9.6535e-4;            % Derivative gain value found from the Genetic Algorithm
   
    
    Matrix_error = [Matrix_error, error]                                % Adds the current error value to a matrix of all the previous errors
    P = Kp * Matrix_error(end)*4;                                         % Calculates the P by multiplying the Kp gain with the most recent error value
    I = Ki * sum(Matrix_error.*sample_rate);                            % Calculates the I by multiplying the Ki gain with the total sum of all error values
    D = Kd * ((Matrix_error(end)- Matrix_error(end-1))/sample_rate);    % Calculates the D by multiplying the Kd gain with the difference of the most recent and second to last gain values divided by the sample rate
    PWM = P + I + D;                                                    % Sets the PWM variable to the sum of the P,I,D values
    %PWM= P; 
    
    
    set_pwm(device,PWM)                                            %Sets the pwm value to the calculated variable of PWM

    pause(sample_rate)              % Waits for next sample

end

%% PID Controller function
function PWM = PID_controller(error,sample_rate)
    Kp = 44.1767;               % Proportional gain value found from the Genetic Algorithm
    Ki = 3.9588e-4;             % Integral gain value found from the Genetic Algorithm
    Kd = -9.6535e-4;            % Derivative gain value found from the Genetic Algorithm
    
    Matrix_error = [error] 
%     Matrix_error = [-0.5];
%     Matrix_error = [Matrix_error, error]                               % Adds the current error value to a matrix of all the previous errors
    P = Kp * Matrix_error(end);                                         % Calculates the P by multiplying the Kp gain with the most recent error value
    I = Ki * sum(Matrix_error.*sample_rate);                            % Calculates the I by multiplying the Ki gain with the total sum of all error values
    D = Kd * ((Matrix_error(end)- Matrix_error(end-1))/sample_rate);    % Calculates the D by multiplying the Kd gain with the difference of the most recent and second to last gain values divided by the sample rate
    PWM = P + I + D;                                                    % Sets the PWM variable to the sum of the P,I,D values
    %PWM= P;
end