% Establish connection with the NXT brick
warning('off','MATLAB:nargchk:deprecated') 
MyNXT = COM_OpenNXT();
COM_SetDefaultNXT(MyNXT);

% Set up parameters for driving the motor
mA = NXTMotor('A'); % associate mA with the motor connected to port A
mA.SmoothStart = 0; % do not use the smooth start option
mA.SpeedRegulation = 0; % do not use the speed regulation option
mA.ResetPosition();

%Desired angle
desired_angle=45;

%Set controller parameters
kp=0.38;
ki=0;
kd=0;

%Set sampling time, derivative filter period
Ts=0.1;
Tf=0.2;

%Initialize variables
int_e=0;
prev_dir_e=0;
prev_toc=0;
prev_e=0;
y_prev=0;
x_prev=0;
x=0;
results=zeros(1,9);

tic 
while toc<8 
    
    %step inpt at 1s
    if toc>1 && toc<2 
        x=desired_angle;
    elseif toc>2 && toc<3 
        x=-45;
    elseif toc>3 && toc<4 
        x=5;
    elseif toc>4 && toc<5
        x=-60;
    elseif toc>5 && toc<6
        x=60;
    elseif toc>6
        x=0;
    end    
    
    data = mA.ReadFromNXT();  % Read the current status of motor A
    
    %Lead compensator
    tau1=0.2;
    tau2=0.05;
    y=((Ts+2*tau1)*x + (Ts-2*tau1)*x_prev -(Ts-2*tau2)*y_prev)/(Ts+2*tau2);
    y_prev=y;
    x_prev=x;
    
       
    %discrete PID controller
    e=y-data.Position;  %error
    int_e=int_e+(e*Ts); %integral error by rectangle method
    dir_e=(e-prev_e)/Ts;  %error derivative by backward difference method
    dir_e_filtered=(Tf/(Ts+Tf))*prev_dir_e+(Ts/(Tf+Ts))*dir_e; %LPF 
    v=round(kp*e+ki*int_e+kd*dir_e);
    prev_e=e;
    prev_dir_e=dir_e_filtered;
    
    
    if v~=0
        v=v+sign(v)*10; %correct for dead band, make input v an integer 
    end
    
    if(abs(v)>100)
        disp('Input too large')
        v=sign(v)*100;
    end    
    
    
    mA.Power = v; % drive the motor at speed determined by v
    mA.SendToNXT();
    
             
    %Store results
    results=[results; toc, x, y, data.Position,v, e, int_e, dir_e, dir_e_filtered ];
       
    
    %Sampling time 0.1s
    while toc-prev_toc<0.1
        continue
    end
    
    prev_toc=toc;
end

mA.Stop('brake')
mA.Stop('off')
COM_CloseNXT(MyNXT);

%% plot results

figure(1)
hold on
plot(results(:,1),results(:,2))
plot(results(:,1),results(:,4))
legend('desired angle','actual angle')

figure(2)
hold on
plot(results(:,1),results(:,2))
plot(results(:,1),results(:,3))
plot(results(:,1),results(:,5))
legend('desired angle','after lead','input')

figure(3)
hold on
plot(results(:,1),results(:,6))
plot(results(:,1),results(:,7))
plot(results(:,1),results(:,8))
plot(results(:,1),results(:,9))
legend('error','integral','derivative','filtered derivative')

%% save results
save('results.mat','results')
