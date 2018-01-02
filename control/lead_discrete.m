%lead compensator

%continuous form 1
s = tf('s');
a=5/3;
z=3;
p=5;
K1=(5/3)*(s+z)/(s+p);
bode(K1)
figure;
step(K1)

%%
%continuous form 2
tau1=0.1;
tau2=0.025;
s = tf('s');
K2=(1+s*tau1)/(1+s*tau2);
%bode(K2)
figure;
step(K2)

%%
%Discrete lead compensator form 1

z=5;
p=20;
a=p/z;

y_prev=0;
x_prev=0;
x=0;
i=1;
Ts=0.1;
results=[0,0,0];

while Ts*i<10
if Ts*i>2
    x=45;
end    
y=(a*(1+Ts*z/2)*x + a*(Ts*z/2-1)*x_prev -(Ts*p/2-1)*y_prev)/(1+Ts*p/2);
y_prev=y;
x_prev=x;
results(i,:)=[i-1,x,y];
i=i+1;
end

t=Ts*results(:,1);
figure;
hold on
plot(t,results(:,2))
plot(t,results(:,3))


%% Form 2

tau1=1;
tau2=0.05;

y_prev=0;
x_prev=0;
x=0;
i=1;
Ts=0.1;
results=[0,0,0];

while Ts*i<10
if Ts*i>2
    x=1;
end    
y=((Ts+2*tau1)*x + (Ts-2*tau1)*x_prev -(Ts-2*tau2)*y_prev)/(Ts+2*tau2);
y_prev=y;
x_prev=x;
results(i,:)=[i-1,x,y];
i=i+1;
end

t=Ts*results(:,1);
figure;
hold on
plot(t,results(:,2))
plot(t,results(:,3))
