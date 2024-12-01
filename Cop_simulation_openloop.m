
clc;
clear;

global g;
 
g = 9.81; % m/s^2

%% Ellipse open loop control
clc
clear all

L1=2*0.1;
L2=2.5*0.1;
L3=1*0.1;
L4= 1.5*0.1;
L5=1*0.1;
a= 3*0.1;      % semi major axis
b= 0.5*0.1;    % semi minor axis
xc=1*0.1;      % Ellipse center
yc=1*0.1;
psi=15;    % Ellipse tilt angle
phi = linspace(0, 360, 360);

%% X desired
Xd = [ xc ; yc] + [cosd(psi) -sind(psi) ;sind(psi) cosd(psi)]*[a*cosd(phi) ; b*sind(phi)];

%% Simulation using ode45

theta0=[20;150;10;-30;-40];

[time theta_sol]=ode45(@openloopcontrol,[0,3*20], theta0);

theta_sol=theta_sol';


% Forward Kinematics end effector co-ordinates
X_actual = L1.*cosd(theta_sol(1,:)) + L2.*cosd(theta_sol(2,:)) + L3.*cosd(theta_sol(3,:)) + L4.*cosd(theta_sol(4,:)) + L5.*cosd(theta_sol(5,:));
Y_actual = L1.*sind(theta_sol(1,:)) + L2.*sind(theta_sol(2,:)) + L3.*sind(theta_sol(3,:)) + L4.*sind(theta_sol(4,:)) + L5.*sind(theta_sol(5,:));


%% Open-loop controller function
function [theta_d_dot]= openloopcontrol(t,theta)

L1=2*0.1;
L2=2.5*0.1;
L3=1*0.1;
L4= 1.5*0.1;
L5=1*0.1;
a= 3*0.1;      % semi major axis
b= 0.5*0.1;    % semi minor axis
xc=1*0.1;      % Ellipse center
yc=1*0.1;
psi=15;    % Ellipse tilt angle
w=360/20;


% Desired trajectory
X=[ xc ; yc] + [cosd(psi) -sind(psi) ;sind(psi) cosd(psi)]*[a*cosd(w*t); b*sind(w*t)];

% Desired X_dot
Xd_dot=[cosd(psi) -sind(psi) ;sind(psi) cosd(psi)]*[-a*w*sind(w*t); b*w*cosd(w*t)];

% Jacobian
J = [-L1*sind(theta(1))  -L2*sind(theta(2))  -L3*sind(theta(3)) -L4*sind(theta(4)) -L5*sind(theta(5)); L1*cosd(theta(1))  L2*cosd(theta(2))  L3*cosd(theta(3))  L4*cosd(theta(4))  L5*cosd(theta(5))];

% All angles in 0 to 360 deg
 theta(1:5)=wrapTo360(theta(1:5));

% Theta dot total
theta_d_dot= pinv(J)*Xd_dot;

end


SimStarted = false;

% Abolsute to relative
Abs_theta1=theta_sol(1,:);
Abs_theta2=theta_sol(2,:);
Abs_theta3=theta_sol(3,:);
Abs_theta4=theta_sol(4,:);
Abs_theta5=theta_sol(5,:);


theta_sol(2,:)=theta_sol(2,:)-Abs_theta1;
theta_sol(3,:)=theta_sol(3,:)-Abs_theta2;
theta_sol(4,:)=theta_sol(4,:)-Abs_theta3;
theta_sol(5,:)=theta_sol(5,:)-Abs_theta4;

theta_sol(1:5)=wrapTo360(theta_sol(1:5));


%% Connect to Coppeliasim
client = RemoteAPIClient();         % Create a client object
sim = client.getObject('sim');      % Reference the Sim Object

defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps); % Get default parameter for fps
sim.setInt32Param(sim.intparam_idle_fps, 0); % Control the fps

world = sim.getObject('./Floor');
origin= sim.getObject('./Origin');
End_effector = sim.getObject('./End_effector');    

joint1= sim.getObject('./joint1');
joint2= sim.getObject('./joint2');
joint3= sim.getObject('./joint3');
joint4= sim.getObject('./joint4');
joint5= sim.getObject('./joint5');



%try
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    % % Drive clock from MATLAB instead of Coppeliasim
    % client.setStepping(true); 
    sim.startSimulation(); % Start Simulation
    SimStarted = true;


%theta_sol is in degrees
for i = 1:length(time)
    sim.setJointTargetPosition(joint1,theta_sol(1,i)*pi/180);
    sim.setJointTargetPosition(joint2,theta_sol(2,i)*pi/180);
    sim.setJointTargetPosition(joint3,theta_sol(3,i)*pi/180);
    sim.setJointTargetPosition(joint4,theta_sol(4,i)*pi/180);
    sim.setJointTargetPosition(joint5,theta_sol(5,i)*pi/180);
    vPos(i,:)= cell2mat(sim.getObjectPosition(End_effector, origin));


    J1(i) = sim.getJointPosition(joint1, world)*180/pi; % Get position of joint 1 deg
    J2(i) = sim.getJointPosition(joint2, world)*180/pi; % Get position of joint 2 deg
    J3(i) = sim.getJointPosition(joint3, world)*180/pi; % Get position of joint 3 deg
    J4(i) = sim.getJointPosition(joint4, world)*180/pi; % Get position of joint 3 deg
    J5(i) = sim.getJointPosition(joint5, world)*180/pi; % Get position of joint 3 deg

    if i+1 < length(time)
        pause(0.01);
   
     end
end


if SimStarted
    sim.stopSimulation();    
end

x_position=[vPos(:,1)'];

y_position=[vPos(:,2)'];



%% Plot 
figure(1)
plot(Xd(1,:),Xd(2,:),'--','LineWidth',2,'DisplayName','Desired Trajectory')
hold on 
plot(x_position,y_position,'LineWidth',0.25,'DisplayName','Actual Trajectory')
xlabel("X [1/10 m]")
ylabel("Y [1/10 m]")
title('Open-loop control using CoppeliaSim')
subtitle('Scaled to 1/10th of a meter')
legend
grid on
grid minor
xlim([-0.6,1])
ylim([-0.6,1])
pbaspect([2 2 2])




%% plots
figure(2)
plot(time,J1,'r')
hold on
plot(time,J2,'g','LineWidth',1)
plot(time,J3,'b')
plot(time,J4,'k')
plot(time,J5,'c')
hold off
title('Open-loop control using CoppeliaSim')
subtitle("Joint angles vs time")
xlabel('time[s]')
ylabel('joint angle [deg]')
legend('theta 1','theta 2','theta 3','theta 4','theta 5')
grid on