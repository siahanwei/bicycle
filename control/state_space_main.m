%Get parameters of bike
[a,b,c,lambda,h,m,g,v]=get_bike_param('real');

%Bike state space equation
%coefficients of lean equation
coeff_1=m*h^2; 
coeff_2=m*a*h*v*sind(lambda)/b;
coeff_3=m*g*h;
coeff_4=m*(v^2*h-a*c*g)*sind(lambda)/b;  

%coefficients of steer model
zita=0.7;
omega_n=11.2;
S1=1/omega_n^2;
S2=2*zita/omega_n;

A=[0 1 0 0;-1/S1 -S2/S1 0 0;0 0 0 1; coeff_4/coeff_1 coeff_2/coeff_1 coeff_3/coeff_1 0];
B=[0;-1/S1;0;0];

%Check controllability
if (rank(ctrb(A,B)) ~= 4)
    disp('System not fully controllable')
end    

Q = diag([1,1,10,10]);
R = 1;
K = lqr(A,B,Q,R);
disp(K)

%Simulate system 
%state x=[steer_angle, steer_rate, lean_angle, lean_rate, n,e,phi]
tspan = 0:.1:5;
x0 = [0; 0; 25; 0;0;0;0];
x_ref=[0;0;0;0];

opt    = odeset('Events', @bike_fallen);  %stop the integration if bike falls to ground
[t,x] = ode45(@(t,x)bike(x,A,B,v,b,lambda,K*(x_ref-x(1:4))),tspan,x0,opt);
u=(repmat(x_ref',length(x),1)-x(:,1:4))*K';

%% Simulate Results

%display results
for k=1:length(t)
    drawbike(x(k,:));
end


%% Plot results
figure;
subplot(1,3,1)
hold on
plot(t,x(:,1))
plot(t,x(:,3))
%plot(t,u)
legend('Actual steer angle','Lean angle');%,'Demanded steer angle');
ylabel('Deg')
xlabel('time/s')
title('Angles')

subplot(1,3,2)
hold on
plot(t,x(:,2))
plot(t,x(:,4))
legend('steer rate','lean rate');
ylabel('deg/s')
xlabel('time/s')
title('Rates')

subplot(1,3,3)
plot(x(:,6),x(:,5))
title('bike path')
xlabel('e/m')
ylabel('n/m')


