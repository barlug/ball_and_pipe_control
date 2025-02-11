function [y, pipe_percentage] = ir2y(ir)
%% Converts IR reading from the top to the distance in meters from the bottom
% Inputs:
%  ~ ir: the IR reading from time of flight sensor
% Outputs:
%  ~ y: the distance in [m] from the bottom to the ball
%  ~ pipe_percentage: on a scale of 0 (bottom of pipe) to 1 (top of pipe)
%  where is the ball
%
% Created by:  Kyle Naddeo 1/3/2022
% Modified by: Jacob King 2/9/2022

%% Parameters
ir_bottom =   0956; % IR reading when ball is at bottom of pipe
ir_top    =   0060; % "                        " top of pipe
y_top     = 0.9144; % Ball at top of the pipe [m]

%% Bound the IR reading and send error message 
% (remeber the IR values are inverted ie small values == large height and large values == small height)

%% Set
pipe_percentage = 1 - (ir-ir_top)/(ir_bottom-ir_top);  % convert to percentage
y = y_top*pipe_percentage;          % convert to meters (distance from bottom)

