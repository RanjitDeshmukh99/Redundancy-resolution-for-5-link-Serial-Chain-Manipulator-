%% Phase 3: Joint-space control and Task-Space Control 

%% i) Ellipse Closed-loop joint-space control

clc
clear all

L1=2;
L2=2.5;
L3=1;
L4= 1.5;
L5=1;
a= 3;      % semi major axis
b= 0.5;    % semi minor axis
xc=1;      % Ellipse center
yc=1;
psi=15;    % Ellipse tilt angle
phi = linspace(0, 360, 360);

%% X desired
Xd = [ xc ; yc] + [cosd(psi) -sind(psi) ;sind(psi) cosd(psi)]*[a*cosd(phi) ; b*sind(phi)];


%% Simulation using ode45

% Theta matrix for this problem is theta_desired (5 x 1) stacked over theta_actual (5 x 1)  

% Initial desired theta values obtained from the previous code 
% [69.8806; 284.1618 ;36.7248; 48.9231; 36.4706]

% theta0=[     ....  initial theta desire  ....        | ...initial theta actual...];
  theta0=[69.8806; 284.1618 ;36.7248; 48.9231; 36.4706;    20; 150; 10; -30; -40   ];

[time theta_sol]=ode45(@jointspacecontrol,[0,3*20], theta0);

theta_sol=theta_sol';

% Forward Kinematics end effector co-ordinates
X_actual = L1.*cosd(theta_sol(6,:)) + L2.*cosd(theta_sol(7,:)) + L3.*cosd(theta_sol(8,:)) + L4.*cosd(theta_sol(9,:)) + L5.*cosd(theta_sol(10,:));
Y_actual = L1.*sind(theta_sol(6,:)) + L2.*sind(theta_sol(7,:)) + L3.*sind(theta_sol(8,:)) + L4.*sind(theta_sol(9,:)) + L5.*sind(theta_sol(10,:));


%% Plot 
figure(1)
plot(Xd(1,:),Xd(2,:),'--','DisplayName','Desired Trajectory')
hold on 
plot(X_actual,Y_actual,'DisplayName','Actual Trajectory')
xlabel("X (m)")
ylabel("Y (m)")
title('Closed-loop joint-space control')
subtitle('Ellipse')
legend
grid on
grid minor
xlim([-4,4])
ylim([-4,4])
pbaspect([2 2 2])


%% Animation

for i=1:length(theta_sol)

    figure(2)

    for j=1:i
xplot(j)=X_actual(j);
yplot(j)=Y_actual(j);
    end

x1= L1.*cosd(theta_sol(6,i));
x2= x1 + L2.*cosd(theta_sol(7,i));
x3= x2 + L3.*cosd(theta_sol(8,i));
x4= x3 + L4.*cosd(theta_sol(9,i));
x5= x4 + L5.*cosd(theta_sol(10,i));

y1= L1.*sind(theta_sol(6,i));
y2= y1 + L2.*sind(theta_sol(7,i));
y3= y2 + L3.*sind(theta_sol(8,i));
y4= y3 + L4.*sind(theta_sol(9,i));
y5= y4 + L5.*sind(theta_sol(10,i));

plot(Xd(1,:),Xd(2,:),'--')
hold on 
plot(xplot,yplot)

plot([0],[0],'k-^','LineWidth',1.5)
plot([0 x1],[0 y1],'k-o','LineWidth',1)
plot([x1 x2],[y1 y2],'k-o','LineWidth',1)
plot([x2 x3],[y2 y3],'k-o','LineWidth',1)
plot([x3 x4],[y3 y4],'k-o','LineWidth',1)
plot([x4 x5],[y4 y5],'k-o','LineWidth',1)
plot([x5],[y5],'r-x','LineWidth',1.5)

xlabel("X (m)")
ylabel("Y (m)")
title(' Animation')
subtitle('Closed-loop joint-space control : Ellipse')
legend('Desired Trajectory','Actual Trajectory')
grid on
grid minor
xlim([-4.5,4])
ylim([-4.5,4])
pbaspect([4 4 4])


pause(0.01)
hold off

end



%% Open-loop controller function
function [theta_d_dot]= jointspacecontrol(t,theta)

L1=2;
L2=2.5;
L3=1;
L4= 1.5;
L5=1;
a= 3;      % semi major axis
b= 0.5;    % semi minor axis
xc=1;      % Ellipse center
yc=1;
psi=15;    % Ellipse tilt angle
w=360/20;

tau = 3;    % Error dynamics time constant
K1 = 1/tau; % Controller gain 1
K2 = 1/tau ; % Controller gain 2
K3 = 1/tau ; % Controller gain 3
K4 = 1/tau ; % Controller gain 4
K5 = 1/tau ; % Controller gain 5

K=diag([0 0 0 0 0 K1 K2 K3 K4 K5]);

% Desired trajectory
Xd=[ xc ; yc] + [cosd(psi) -sind(psi) ;sind(psi) cosd(psi)]*[a*cosd(w*t); b*sind(w*t)];

% Desired X_dot
Xd_dot=[cosd(psi) -sind(psi) ;sind(psi) cosd(psi)]*[-a*w*sind(w*t); b*w*cosd(w*t)];


% All angles in 0 to 360 deg
theta(1:10)=wrapTo360(theta(1:10));

% Jacobian

J_des=[-L1*sind(theta(1))  -L2*sind(theta(2))  -L3*sind(theta(3)) -L4*sind(theta(4)) -L5*sind(theta(5)); L1*cosd(theta(1))  L2*cosd(theta(2))  L3*cosd(theta(3))  L4*cosd(theta(4))  L5*cosd(theta(5))];
J_act=[-L1*sind(theta(6))  -L2*sind(theta(7))  -L3*sind(theta(8)) -L4*sind(theta(9)) -L5*sind(theta(10)); L1*cosd(theta(6))  L2*cosd(theta(7))  L3*cosd(theta(8))  L4*cosd(theta(9))  L5*cosd(theta(10))];


% Theta dot total
theta_d_dot= [pinv(J_des);pinv(J_act)]*Xd_dot + K*([theta(1)-theta(1) ; theta(2)-theta(2) ;theta(3)-theta(3); theta(4)-theta(4); theta(5)-theta(5) ; theta(1)-theta(6) ; theta(2)-theta(7) ;theta(3)-theta(8); theta(4)-theta(9); theta(5)-theta(10)]);


end


