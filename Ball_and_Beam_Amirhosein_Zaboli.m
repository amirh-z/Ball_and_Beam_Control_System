%amirh_z
%HW6
clc
clear

zeros = [];
poles = [0 0];
P = zpk(zeros, poles, 0.21);

%rlocus
figure(1)

%rlocus(P)
sgrid(0.7, 1.9)
axis([-5 5 -2 2])

%bode
figure(2)
bode(P)

%nyquist
figure(3)
nyquist(P)

%rlocus-based controller
z_rlocus = -0.01;
p_rlocus = -5;
C_rlocus = zpk(z_rlocus, p_rlocus, 1);
figure(4)
G_rlocus = C_rlocus*P;
rlocus(G_rlocus)
sgrid(0.7, 1.9)
axis([-5 5 -2 2])
[k, poles] = rlocfind(G_rlocus)
CL_rlocus = feedback(k*G_rlocus, 1);
figure(5)
t = 0:0.01:6;
step(0.25*CL_rlocus, t)

%frequency-based controller
phi = 80*pi/180;
w = 1.9;
a = (1 - sin(phi))/(1+sin(phi));
T = 1/(w*sqrt(a));
k = 25; %gain of controller
z_f = -1/T;
p_f = -1/(a*T);
C_f = zpk(z_f, p_f, k);
G_f = C_f*P; %open-loop total gain including controller gain
figure(6)
bode(G_f)
figure(7)
CL_f = feedback(k*G_f, 1); %closed-loop gain
step(0.25*CL_f, t)

%PID controller
Kp = 20;
Kd = 45;
C_pid = pid(Kp, 0, Kd);
G_pid = C_pid * P;
CL_pid = feedback(G_pid, 1);
figure(8)
step(0.25*CL_pid)