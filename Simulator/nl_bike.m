function dx = nl_bike(x,a,b,c,lambda,h,m,g,v,S1,S2,u)
%state x=[steer_angle, steer_rate, lean_angle, lean_rate, n,e,phi]
%All angles in degrees

delta_f=atand(tand(x(1)*sind(lambda)));
delta_fdot=(secd(x(1))/secd(delta_f))^2*x(2)*sind(lambda);

C1=m*h^2;
C2=m*a*h*v*(secd(delta_f))^2*delta_fdot/b;
C3=m*g*(h*sind(x(3))-a*c*sind(lambda)*x(1)/(b*cosd(x(3))));
C4=m*v^2*h*tand(delta_f)*cosd(x(3))/b;

%Derivative of state dx
dx(1,1)=x(2);
dx(2,1)=(-1/S1)*x(1) - (S2/S1)*x(2) - (1/S1)*u;
dx(3,1)=x(4);
dx(4,1)=C2+C3+C4/C1;

%Limits of steering motor
%Maximum steer rate is 800 degrees/sec. Mantain this rate at the limits
if(x(2)>800)
    dx(1,1)=800;
    dx(2,1)=0;
end

%Maximum steer angle is 70 degrees. It will brake at the limits
if(x(1)>70)
    dx(1,1)=0;
    dx(2,1)=0;
end

%For the path
Sx=sind(x(7));
Cx=cosd(x(7));
dx(5,1)=v*Sx;
dx(6,1)=v*Cx;
dx(7,1)=v*tand(delta_f)/b;

end