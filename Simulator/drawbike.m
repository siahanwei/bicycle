function drawbike(x)
lean=x(3);
steer=-x(1);
n=x(5);
e=x(6);
phi=x(7);


%Coordinates of tilt plot
length_mass=0.5;
width_mass=0.5;
length_bar=1.5;
mass_x=length_bar*sind(lean);
mass_y=length_bar*cosd(lean);

%Coordinates of steer plot
pivot_x=0;
pivot_y=1;
steer_length=.8;
a=(steer_length/2)*sind(steer);
b=(steer_length/2)*cosd(steer);
right=[pivot_x+b, pivot_y-a];
left=[pivot_x-b, pivot_y+a];
wheel_length=.8;
p1=(wheel_length*0.6)*sind(steer);
p2=(wheel_length*0.4)*sind(steer);
q1=(wheel_length*0.6)*cosd(steer);
q2=(wheel_length*0.4)*cosd(steer);
wheel_front=[pivot_x+p1, pivot_y+q1];
wheel_back=[pivot_x-p2, pivot_y-q2];

figure(1)
set(gcf,'units','normalized','outerposition',[0 0 1 1])

%tilt plot
subplot(1,2,1)
plot([-2,2],[0,0],'LineWidth',1)
hold on
plot([0,mass_x],[0,mass_y],'LineWidth',3)
rectangle('Position',[mass_x-length_mass/2,mass_y-width_mass/2,length_mass,width_mass],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[0 0 0])  %mass
xlim([-2,2])
ylim([-1,3])
set(gca,'YTickLabel',[],'XTickLabel',[])
grid on
title(sprintf('Tilt Angle = %f', lean))
hold off

%Steer plot
subplot(1,2,2)

plot([0,pivot_x],[0,pivot_y],'LineWidth',2)
hold on
plot([wheel_front(1),wheel_back(1)],[wheel_front(2),wheel_back(2)],'LineWidth',4)
plot([left(1),right(1)],[left(2),right(2)],'LineWidth',2)
set(gca,'YTickLabel',[],'XTickLabel',[])
xlim([-2,2])
ylim([0,3])
title(sprintf('Steer Angle = %f', steer))
hold off
grid on

drawnow
end