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
w=logspace(-3,3);
bode(sys,w)

%% Lead Compensator
%phase lead: make system stable

alpha=sqrt(5); 
wc=10;
lead=tf(alpha*[1 wc/alpha],[1 alpha*wc]);
bode(lead,w);

%% Lag compensator 
beta=10; 
wc=8;
lag=tf((1/beta)*[1 wc*beta],[1 wc/beta]);
bode(lag,w)

%%
bode(lead*lead*lag*sys*20,w)
Kc=20*lag;
Kb=lead*lead;
CL=feedback(G*Kc,Kb);
[m,p]=bode(CL,0);
Ka=1/m;
step(Ka*CL)

