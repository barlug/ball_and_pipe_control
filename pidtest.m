function J = pidtest(G,dt,parms)
s=tf('s');                                      % variable of the dynamic transfer function model
K=parms(1)+parms(2)/s+parms(3)*s/(1+.001*s);    % pid controller system, Kp + Ki + Kd(with high pass filter attached)

% set up the system
Loop=series(K,G);                               %Open loop in series
ClosedLoop=feedback(Loop,1);                    %Closed loop system
t=0:dt:300;                                     %Time in which each individual in the population is run for (300 seconds)
[y,t]=step(ClosedLoop,t);
CTRLtf=K/(1+K*G);                               % closed-loop transfer function

u=lsim(CTRLtf,1-y,t);
Q=1;
R=0.001;
J=dt*sum(Q*(1-y(:)).^2+R*u(:).^2)               % the LQR cost function made to have the fastest rise time while also minimizing steady state error

[y,t]=step(ClosedLoop,t);                       %Graphs the individuals as the populations and generations are made
plot(t,y,'LineWidth',2,'Color','r')
drawnow

end
