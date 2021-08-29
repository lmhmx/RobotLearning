function [zdot, T1, T2, T3] = FullDyn( t,z,p )
% liming
th1 = z(1);
th2 = z(3);
th3 = z(5);
thdot1 = z(2);
thdot2 = z(4);
thdot3 = z(6);

%Current disturbance force on end effector
FxCurrent = p.Fx;
FyCurrent = p.Fy;

xCurrentTar = p.xtarget;
yCurrentTar = p.ytarget;

xdotCurrentTar = (p.xt(t+p.h)-p.xt(t))/p.h;
ydotCurrentTar = (p.yt(t+p.h)-p.yt(t))/p.h;

T = ImpedenceControl(p.Kd,p.Kp,p.l1,p.l2,p.l3,th1,th2,th3,thdot1,thdot2,thdot3,...
    xdotCurrentTar,xCurrentTar,ydotCurrentTar,yCurrentTar);


T1 = T(1) + GravityCompT1(0,0,p.I1,p.I2,p.I3,p.d1,p.d2,p.d3,p.g,p.l1,p.l2,p.l3,p.m1,p.m2,p.m3,th1,th2,th3,thdot1,thdot2,thdot3);
T2 = T(2) + GravityCompT2(0,0,p.I1,p.I2,p.I3,p.d1,p.d2,p.d3,p.g,p.l1,p.l2,p.l3,p.m1,p.m2,p.m3,th1,th2,th3,thdot1,thdot2,thdot3);
T3 = T(3) + GravityCompT3(0,0,p.I1,p.I2,p.I3,p.d1,p.d2,p.d3,p.g,p.l1,p.l2,p.l3,p.m1,p.m2,p.m3,th1,th2,th3,thdot1,thdot2,thdot3);

thdotdot1 = Thdotdot1(FxCurrent,FyCurrent,p.I1,p.I2,p.I3,T1,T2,T3,p.d1,p.d2,p.d3,p.g,p.l1,p.l2,p.l3,...
    p.m1,p.m2,p.m3,th1,th2,th3,thdot1,thdot2,thdot3);
thdotdot2 = Thdotdot2(FxCurrent,FyCurrent,p.I1,p.I2,p.I3,T1,T2,T3,p.d1,p.d2,p.d3,p.g,p.l1,p.l2,p.l3,...
    p.m1,p.m2,p.m3,th1,th2,th3,thdot1,thdot2,thdot3);
thdotdot3 = Thdotdot3(FxCurrent,FyCurrent,p.I1,p.I2,p.I3,T1,T2,T3,p.d1,p.d2,p.d3,p.g,p.l1,p.l2,p.l3,...
    p.m1,p.m2,p.m3,th1,th2,th3,thdot1,thdot2,thdot3);

zdot = [thdot1
    thdotdot1
    thdot2
    thdotdot2
    thdot3
    thdotdot3
    ];

end

