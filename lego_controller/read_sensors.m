% Establish connection with the NXT brick
warning('off','MATLAB:nargchk:deprecated') 
MyNXT = COM_OpenNXT();
COM_SetDefaultNXT(MyNXT);

% Set up parameters for steering motor
mA = NXTMotor('A'); % associate mA with the motor connected to port A
mA.SmoothStart = 0; % do not use the smooth start option
mA.SpeedRegulation = 0; % do not use the speed regulation option
mA.ResetPosition();
%Set steering controller parameters
kp=0.38;
tau1=0.2;
tau2=0.05;
Ts=0.02;
%Initialize steering controller variables
prev_toc=0;
prev_e=0;
y_prev=0;
x_prev=0;
x=0;


OpenAccelerator(SENSOR_4);
OpenGyro(SENSOR_1);
offset=CalibrateGyro(SENSOR_1, 'AUTO');
sensor_readings=[];
filter_angle=0;
gyro_angle=0;
   
tic
while toc<7
       
    
     %step inpt at 1s
    if toc>1 && toc<2 
        x=40;
    elseif toc>2 && toc<3 
        x=-40;
    elseif toc>3 && toc<4 
        x=5;
    elseif toc>4 && toc<5
        x=-10;
    elseif toc>5 && toc<6
        x=25;
    elseif toc>6
        x=0;
        
    end    
      
    accel = GetAccelerator(SENSOR_4);
    tilt_rate = GetGyro(SENSOR_1);
    data = mA.ReadFromNXT(); 
    steer_angle=data.Position;
    accel(1)=(accel(1)+28)/203.4;
    accel(2)=(accel(2)-7)/203.5;
    accel(3)=(accel(3)+7)/204.5;
    tilt_angle= atand(-(accel(2))./sqrt(accel(3)^2+accel(1)^2)); 
    gyro_angle=gyro_angle+tilt_rate*Ts;
    filter_angle=0.96*(filter_angle+tilt_rate*Ts)+0.04*tilt_angle;
    %Steering control 
    y=((Ts+2*tau1)*x + (Ts-2*tau1)*x_prev -(Ts-2*tau2)*y_prev)/(Ts+2*tau2);
    y_prev=y;
    x_prev=x;
    e=y-steer_angle;  %error
    v=round(kp*e);
    prev_e=e;
        
    if v~=0
        v=v+sign(v)*10; %correct for dead band, make input v an integer 
    end
    
    if(abs(v)>100)
        v=sign(v)*100;
    end    
    
    mA.Power = v; 
    mA.SendToNXT();
    
    
    sensor_readings=[sensor_readings;toc, steer_angle,tilt_angle,tilt_rate,filter_angle,gyro_angle];
     %Go round loop every 0.02s 
    
     
    while toc-prev_toc<0.02
        continue
    end
    prev_toc=toc;      
end

mA.Stop('brake')
mA.Stop('off')
CloseSensor(SENSOR_4);
CloseSensor(SENSOR_1);
COM_CloseNXT(MyNXT);

%%
figure
hold on
plot(sensor_readings(:,1),sensor_readings(:,2));
plot(sensor_readings(:,1),sensor_readings(:,3));
plot(sensor_readings(:,1),sensor_readings(:,6));
plot(sensor_readings(:,1),sensor_readings(:,5));

xlabel('seconds');
ylabel('deg');
title('Angle measurements');
legend('Input angle','Accel angle','Gyro angle','Filtered angle');

input_rate=[0;diff(sensor_readings(:,2))./Ts];
figure;
hold on
plot(sensor_readings(:,1),input_rate);
plot(sensor_readings(:,1),sensor_readings(:,4));
xlabel('seconds');
ylabel('deg/s');
title('Rate measurements');
legend('input rate','gyro');

%% Make sure initial start angle is at 0 
MyNXT = COM_OpenNXT();
COM_SetDefaultNXT(MyNXT);
OpenAccelerator(SENSOR_4);
tic
while toc<20
    accel = GetAccelerator(SENSOR_4);
    accel(1)=(accel(1)+28)/203.4;
    accel(2)=(accel(2)-7)/203.5;
    accel(3)=(accel(3)+7)/204.5;
    tilt_angle2= atand(-(accel(2))./sqrt(accel(3)^2+accel(1)^2))
    pause(0.2)
end    
CloseSensor(SENSOR_4);
COM_CloseNXT(MyNXT);