%% Phase 2: Redundancy Resolution
%% (i) Using the traditional pseudo-inverse solution

%% Ellipse open loop control

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

theta0=[20;150;10;-30;-40];

[time theta_sol]=ode45(@openloopcontrol,[0,3*20], theta0);

theta_sol=theta_sol';

% Forward Kinematics end effector co-ordinates
X_actual = L1.*cosd(theta_sol(1,:)) + L2.*cosd(theta_sol(2,:)) + L3.*cosd(theta_sol(3,:)) + L4.*cosd(theta_sol(4,:)) + L5.*cosd(theta_sol(5,:));
Y_actual = L1.*sind(theta_sol(1,:)) + L2.*sind(theta_sol(2,:)) + L3.*sind(theta_sol(3,:)) + L4.*sind(theta_sol(4,:)) + L5.*sind(theta_sol(5,:));


%% Plot 
figure(1)
plot(Xd(1,:),Xd(2,:),'--','DisplayName','Desired Trajectory')
hold on 
plot(X_actual,Y_actual,'DisplayName','Actual Trajectory')
xlabel("X (m)")
ylabel("Y (m)")
title('Open-loop Control')
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

x1= L1.*cosd(theta_sol(1,i));
x2= x1 + L2.*cosd(theta_sol(2,i));
x3= x2 + L3.*cosd(theta_sol(3,i));
x4= x3 + L4.*cosd(theta_sol(4,i));
x5= x4 + L5.*cosd(theta_sol(5,i));

y1= L1.*sind(theta_sol(1,i));
y2= y1 + L2.*sind(theta_sol(2,i));
y3= y2 + L3.*sind(theta_sol(3,i));
y4= y3 + L4.*sind(theta_sol(4,i));
y5= y4 + L5.*sind(theta_sol(5,i));

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
subtitle('Open-loop Control : Ellipse')
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
function [theta_d_dot]= openloopcontrol(t,theta)

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