load('data.mat')
t=processdata.t;
q1=processdata.q1;
q2=processdata.q2
q3=processdata.v1;
q4=processdata.v2;


figure(2)
plot(t,q2,'linewidth',2)
axis([0 30 -1 1])
legend('q2')
xlabel('Time')
ylabel('q2')
figure(3)
error=abs(.03 - q2)
plot(t,error,'linewidth',2)
legend('q2')
axis([0 30 -1 1])
xlabel('Time')
ylabel('q2 Error')


