clc;clear;
load('DesignProblem02_EOMs.mat');

M = symEOM.M;
C = symEOM.C;
N = symEOM.N;
tau = symEOM.tau;

syms q1 q2 v1 v2 tau1 real

q=[q1;q2];
qdot=[v1;v2];
T=[tau1;0];
qddot=M^(-1)*(-C*qdot-N+T);

%state matrix
s = [q;qdot];
sdot = [qdot;qddot];

%Output matrix
y  = q2;

%Asym=[0 0 1 0; 0 0 0 1;simplify(jacobian(data.qddot,data.q))];
%Bsym=[0;0;jacobian(data.qddot,tau1)];

%Linearising to find A,B,C,D
Asym = jacobian(sdot,s);
Bsym = jacobian(sdot,tau1);
Csym = jacobian(y,s);
Dsym = jacobian(y,tau1);

%Equilibrium Values
q1_e = 0;
q2_e = 0;
v1_e = 0;
v2_e = 0;

s_e = [q1_e;q2_e;v1_e;v2_e]; %Equilibrium state matrix
t_e = double(subs((M*[0;0]+C*qdot+N),[q1;q2;v1;v2],[q1_e;q2_e;v1_e;v2_e]));


%Subbing Equilibrium Values into  the  matrices

A = double(subs(Asym,[s;tau1],[s_e;t_e(1)]));
B = double(subs(Bsym,[s;tau1],[s_e;t_e(1)]));
%C = double(subs(Csym,[s;tau1],[s_e;t_e(1)]));
%D = double(subs(Dsym,[s;tau1],[s_e;t_e(1)]));
C = eye(4);
D = 0;

%Defining matrices to compute K using LQR and finding Kref

%data.Q=eye(size(data.A));
%data.K = lqr(data.A,data.B,data.Q,.05);

p  = [-1 -2 -3 -4];
K = acker(A, B, p);

Kref= -1/(C*inv(A-B*K)*B);
x_init = [0.1; 0.01; 0.01; 0.01].*randn(4,1);
%Checking Open Loop Controllability - Linear system controllability
W = rank(ctrb(A,B)); %==# Rows in B (4)

%Checking Open Loop Stability (is it all neg?) - zero input
stb = vpa(eig(A));
[t,x]=ode45(@(t,x) A*x,[0,10],x_init);
figure(1)
plot(t,x')
title('Zero Feedback Linear Model')
xlabel('time (s)')
ylabel('state variables')
legend({'q1','q2','v1','v2'},'Location','northwest')
xlim([0,4])


%Checking Closed loop Stability - linear system state feedback
stab = vpa(eig(A - B*K));


%Finding steady state error in reference tracking without unit disturbance
r = s_e;
d0 = 0;
y_ss = C*inv(A-B*K)*B*Kref*r + d0;
e_ss  = abs(y_ss)-r;
e_ssstr = mat2str(e_ss);

[t,x] = ode45(@(t,x)(odedynref(A,B,K,r,Kref,x,t)),[0,10],x_init);
figure(2)
plot(t,C*x')
title('State Feedback Linear Model')
xlabel('time (s)')
ylabel('state variables')
legend({'q1','q2','v1','v2'},'Location','north')
xlim([0,10])

%Finding steady state error in reference tracking with unit disturbance
E = A-B*K;
F = B*Kref;
G = C-D*K;
H = D*Kref;


d=1;

[t,x] = ode45(@(t,x)(odedynref_disturbance(A,B,K,r,Kref,x,t,d)),[0,10],x_init);
y = C*x';
yss = y(end);
ess = yss-r;
essstr = mat2str(ess);
figure(3)
plot(t,C*x')
title('State Feedback with Disturbance Rejection')
xlabel('time (s)')
ylabel('state variables')
legend({'q1','q2','v1','v2'},'Location','north')
xlim([0,10])

function [dx]=odedynref(A,B,K,ref,Kref,x,t)
    dx=(A-B*K)*x+B*Kref*ref;
end

function [dx]=odedynref_disturbance(A,B,K,r,Kref,x,t,d)
    dx=(A-B*K)*x+B*Kref*r+B*d;
end
