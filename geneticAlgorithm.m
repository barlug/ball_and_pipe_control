%***************************************************************
% A Genetic Algorithm Tuning Method for PID Controller
% Source: Data-Driven Science and Engineering: Machine Learning, Dynamical Systems, and Control
%           Steven L. Brunton and J. Nathan Kutz
%           Cambridge, 2018
% *************************************************************%
clear variables;clf;close all;clc;

dt=0.001;
% Genetic algorithm parameters
PopSize=25;
MaxGenerations=10;
options = optimoptions(@ga,'PopulationSize',PopSize,'MaxGenerations',MaxGenerations);

% create open-loop system
s=tf('s'); % make transfer function model
rho_air=1.225; % density of air (kg/m^3)
v_eq =2.4384; % equilibrium v:= velocity of air at s.s. (m/s)
mball=0.0027 % mass of ball (kg)
Vball=268.083e-6; % volume of ball (m^3)
g=9.80665; % gravity constant (m/s^2)
c2=2*g/(v_eq)*((mball-rho_air*Vball)/mball);
c3=6.3787e-4;

G=c3*c2/(s*(s+c2)); % Open-loop system

[x,fval]=ga(@(K)pidtest(G,dt,K),3,-eye(3),zeros(3,1))


%*******************************************************
% Genetic algorithm tunes for optimized PID parameters, 
%   Ki,Kd,Kp.
% - For any optimaztion, first define the cost function 
%   (i.e. the objective function) which needs to be 
%   minimized
% *****************************************************%




