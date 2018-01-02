% Establish connection with the NXT brick
warning('off','MATLAB:nargchk:deprecated') 
MyNXT = COM_OpenNXT();
COM_SetDefaultNXT(MyNXT);

% Set up parameters for driving motor
mBC = NXTMotor('BC'); 
mBC.SmoothStart = 1; % do not use the smooth start option
mBC.SpeedRegulation = 0; % Use the speed regulation option
mBC.ResetPosition();
mBC.Power = 100;


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


%Initialize sensors
OpenAccelerator(SENSOR_4);
OpenGyro(SENSOR_1);
%offset=CalibrateGyro(SENSOR_1, 'AUTO');
load offset
CalibrateGyro(SENSOR_1, 'MANUAL',offset)

%start drive motor
%mBC.SendToNXT();
desired_lean=0;
results=[];

tic
while true
    %Read sensors
    accel = GetAccelerator(SENSOR_4);
    tilt_angle= atand((accel(2)-8)./(-abs(accel(3))))+2; 
    tilt_rate = GetGyro(SENSOR_1);
    data = mA.ReadFromNXT(); 
    steer_angle=data.Position;
    
    %Check fall
    if abs(tilt_angle)>30 || abs(steer_angle)>90
        disp('Bike crashed')
        break
    end    
        
    %Feedback controller
    x=desired_lean-(-4*tilt_angle-2*tilt_rate);
    
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
    
    %record results
    results=[results;toc,steer_angle,tilt_angle,tilt_rate,x,y,v];
    
    %Go round loop every 0.02s 
    disp(toc-prev_toc)
    while toc-prev_toc<0.02
        continue
    end
    prev_toc=toc;      
    
end

results=[results;toc,steer_angle,tilt_angle,tilt_rate,x,y,v];

mBC.Stop('brake')
mBC.Stop('off')
mA.Stop('brake')
mA.Stop('off')
COM_CloseNXT(MyNXT);

%% plot results

figure;
hold on
plot(results(:,1),results(:,2))
plot(results(:,1),results(:,3))
plot(results(:,1),results(:,4))
plot(results(:,1),results(:,5))
legend('steer','tilt','tilt rate','steer demanded')
