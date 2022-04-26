%***************************************************************
% A Genetic Algorithm Tuning Method for PID Controller
% Source: Data-Driven Science and Engineering: Machine Learning, Dynamical Systems, and Control
%           Steven L. Brunton and J. Nathan Kutz
%           Cambridge, 2018
%
% Description:
% Genetic algorithm tunes for optimized PID parameters, 
%   Ki,Kd,Kp.
% - For any optimaztion, first define the cost function 
%   (i.e. the objective function) which needs to be 
%   minimized
% 
% -Long H. Chau, Tues 4-19-2022 EST
% -Long H. Chau, Tues 4-26-2022 01:20 EST
% ********************************************************************%
clear variables;clf;close all;clc;

dt=0.001;   % sampling period (seconds per sample)

%% create open-loop system
s=tf('s');                                      % make transfer function model
rho_air=1.225;                                  % density of air (kg/m^3)
v_eq =2.4384;                                   % equilibrium v:= velocity of air at s.s. (m/s)
mball=0.0027;                                   % mass of ball (kg)
Vball=268.083e-6;                               % volume of ball (m^3)
g=9.80665;                                      % gravity constant (m/s^2)
c2=2*g/(v_eq)*((mball-rho_air*Vball)/mball);    % one portion of the 
c3=6.3787e-4;

G=c3*c2/(s*(s+c2));                             % Open-loop system

%% Genetic algorithm parameters
PopSize=100;          % set population size
MaxGenerations=1000;  % set generations

%% Configure options for the genetic algorithm function
% Sets the population size and max generation size for the genetic algorithm
options = optimoptions(@ga,'PopulationSize',PopSize,'MaxGenerations',MaxGenerations);

%% The genetic that sets the three gain values for the PID controller
%-The output x is the three gain values Kp, Ki, and Kd for the PID
%   controller.
%-The fval is the best value of J that was found in all of the
%   generations.
[x,fval]=ga(@(K)pidtest(G,dt,K),3,-eye(3),zeros(3,1),[],[],[],[],[],options);







