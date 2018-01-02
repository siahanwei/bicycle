%% Results from bike
load results
figure(1)
hold on
plot(results(:,1),results(:,2))
plot(results(:,1),results(:,4))
legend('desired angle','actual angle')

%% 2nd order model for steering
zita=0.7;
omega_n=11.2;
S1=1/omega_n^2;
S2=2*zita/omega_n;

sys=tf([1],[S1,S2,1]);
%step(45*sys)

%% Simulate step response of model
%create a step input of magnitude mag for 2s and step down. plot response to 6s
demanded_angle=[45,-45,5,-60,60,0];
t = 0:0.01:8;
u=zeros(size(t));
prev_index=1;
for i=1:length(demanded_angle)
    index=find(t==i);
    index2=find(t==i+1);
    u(index:index2-1)=demanded_angle(i)*ones(1,index2-index);
end    

 [y,t]=lsim(sys,u,t);

figure(1);
hold on
plot(t,u);
plot(results(:,1),results(:,4))
plot(t,y)
legend('delta demanded','delta','model')
ylabel('Steer angle/degrees')
xlabel('Time/seconds')