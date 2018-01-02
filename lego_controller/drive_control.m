% Establish connection with the NXT brick
warning('off','MATLAB:nargchk:deprecated') 
MyNXT = COM_OpenNXT();
COM_SetDefaultNXT(MyNXT);

% Set up parameters for driving the motor
mBC = NXTMotor('BC'); 
mBC.SmoothStart = 1; % do not use the smooth start option
mBC.SpeedRegulation = 0; % Use the speed regulation option
mBC.ResetPosition();
mBC.Power = 100;
mBC.SendToNXT();

tic
while toc<6
   continue 
end    

mBC.Stop('brake')
mBC.Stop('off')
COM_CloseNXT(MyNXT);