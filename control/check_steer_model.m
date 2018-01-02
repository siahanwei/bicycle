%load and plot actual results
load test_results
figure(1);
subplot(1,2,1)
plot(test_results(:,1),(pi/180).*test_results(:,3));
input=[-80,10,80,40,-20,50,-5,-40,0];

% 2nd order model for steering
tau=0.087;
ss_gain=10.2219;
a=1/tau;
b=ss_gain*a;

H=(pi/180)*tf([b],[1,a,0]);

% Simulate step response of model
%create a step input of magnitude mag for 2s and step down. plot response to 6s
t = 0.01:0.01:2;
q=repmat(input,length(t),1);
u=reshape(q,prod(size(q)),1);
u=[input(1);u];
t = 0:0.01:2*length(input);

[y,t]=lsim(H,u,t);

%get derivative 
speed = [0; diff(y) ./ diff(t)];

figure(1);
subplot(1,2,1);
hold on
plot(t,speed)
xlabel('time/s')
ylabel('speed rad/s')
legend('Actual motor','Model')

subplot(1,2,2)
plot(t,u)
ylim([-100,100])
xlabel('time/s')
ylabel('input v')
legend('Input')