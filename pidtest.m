function J = pidtest(G,dt,parms)
s=tf('s'); % variable of the dynamic transfer function model
K=parms(1)+parms(2)/s+parms(3)*s/(1+.001*s); % pid controller system

% set up the system
Loop=series(K,G);
ClosedLoop=feedback(Loop,1);
t=0:dt:300;
[y,t]=step(ClosedLoop,t);
CTRLtf=K/(1+K*G); % closed-loop transfer function
u=lsim(CTRLtf,1-y,t);
Q=1;
R=0.001;
J=dt*sum(Q*(1-y(:)).^2+R*u(:).^2) % the cost function

[y,t]=step(ClosedLoop,t);
plot(t,y,'LineWidth',2,'Color','r')
drawnow

end
