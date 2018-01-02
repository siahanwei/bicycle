%% Open loop system
[a,b,c,lambda,h,m,g,v]=get_bike_param('lego');

%Lean TF
coeff_1=v*a*h*sind(lambda);
coeff_2=(v^2*h-a*c*g)*sind(lambda);
coeff_3=b*h^2;
coeff_4=-b*g*h;
G=tf([coeff_1,coeff_2],[coeff_3,0,coeff_4]);

%Steering controller TF
zita=0.7;
omega_n=11.2;
a=1/omega_n^2;
b=2*zita/omega_n;
H=-tf([1],[a,b,1]);

sys=G*H;
%% PID controller

%PID controller
Kp=6*-1.6;
Kd=-1.6;
Ki=0;
s = tf('s');
K = Kp + Kd*s +Ki/s;
tzero(K)
w=logspace(-3,3);

%% Simulate



