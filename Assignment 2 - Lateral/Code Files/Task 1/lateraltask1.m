clear all;
clc;

%4 wheel passenger vehicle
%% Data for the simulation.
% Data c0rresponds to Saab 9-3.

% Vehicle data
sampleTime                  = 1e-2;                             % Sample time for data output
vehicleData.m               = 1675;                             % Mass
vehicleData.J               = 2617;                             % Vehicle inertia about Z axis
vehicleData.L               = 2.675;                            % Wheelbase
vehicleData.c0              = 30.7;                             % Tyre stiffness parameter
vehicleData.c1              = -0.00235;                         % Tyre stiffness parameter
vehicleData.steeringRatio   = 15.9;                             % Steering ratio
vehicleData.h               = 0.543;                            % Height of c0G
vehicleData.hrcf            = 0.045;                            % Front roll center height
vehicleData.hrcr            = 0.101;                            % Rear roll center height
vehicleData.cw              = 7e4;                              % Total roll stiffness
vehicleData.w               = 1.51;                             % Track width
vehicleData.g               = 9.81;                             % Acceleration due to gravity
vehicleData.R               = 0.3;                              % Wheel radius

disp('Saab 9-3 parameters loaded');

% Parameters to be changed in different tasks:
vehicleData.lf              = 0.47*vehicleData.L;               % Distance of c0G from front axle
vehicleData.lr              = vehicleData.L-vehicleData.lf;     % Distance of c0G from rear axle 
simulationTime              = 10;                               % Simulation time
vx0                         = 100;                              % Initial speed
vehicleData.brakeDemand     = 0.0;                              % Brake force demand
SWA                         = 10;                               % Steering wheel angle                                        
vehicleData.pRollDist       = 0.65;                             % Front/Total roll stiffness distribution
vehicleData.brakeDist       = 0.5;                              % Front/Total brake distribution ratio
%% Task1
Lf_1= 0.37*vehicleData.L; %Load case 1
Lf_2= 0.63*vehicleData.L; %Load case 2
Lf_3= 0.47*vehicleData.L; %Load case 3

Lr_1= vehicleData.L-Lf_1;
Lr_2= vehicleData.L-Lf_2;
Lr_3= vehicleData.L-Lf_3;

F= vehicleData.m*vehicleData.g;

% By forming the moment equations we get the front and rear axle forces
F_fz1= ((F*Lr_1)/vehicleData.L)/2;
F_fz2= ((F*Lr_2)/vehicleData.L)/2;
F_fz3= ((F*Lr_3)/vehicleData.L)/2;
F_rz1= (F-(F*Lr_1)/vehicleData.L)/2;
F_rz2= (F-(F*Lr_2)/vehicleData.L)/2;
F_rz3= (F-(F*Lr_3)/vehicleData.L)/2;

%c0rnering stiffness of front axle (N/rad)
Cf_1= 2*((vehicleData.c0*F_fz1)+(vehicleData.c1*F_fz1^2));
Cf_2= 2*((vehicleData.c0*F_fz2)+(vehicleData.c1*F_fz2^2));
Cf_3= 2*((vehicleData.c0*F_fz3)+(vehicleData.c1*F_fz3^2));
%c0rnering stiffness of rear axle (N/rad)
Cr_1= 2*((vehicleData.c0*F_rz1)+(vehicleData.c1*F_rz1^2));
Cr_2= 2*((vehicleData.c0*F_rz2)+(vehicleData.c1*F_rz2^2));
Cr_3= 2*((vehicleData.c0*F_rz3)+(vehicleData.c1*F_rz3^2));

%% Task 1.2 understeer gradient
Ku_1= vehicleData.m*((Cr_1*Lr_1)-(Cf_1*Lf_1))/(Cf_1*Cr_1*vehicleData.L); %rad/(m/s^2)
Ku_2= vehicleData.m*((Cr_2*Lr_2)-(Cf_2*Lf_2))/(Cf_2*Cr_2*vehicleData.L); %rad/(m/s^2)
Ku_3= vehicleData.m*((Cr_3*Lr_3)-(Cf_3*Lf_3))/(Cf_3*Cr_3*vehicleData.L); %rad/(m/s^2)

%% Task 1.3 Critical/Characteristic speed
V_critical_1= sqrt(vehicleData.L/(-Ku_1));
V_critical_2= sqrt(vehicleData.L/(-Ku_2));
V_critical_3= sqrt(vehicleData.L/(-Ku_3));

V_characteristic_1= sqrt(vehicleData.L/(Ku_1));
V_characteristic_2= sqrt(vehicleData.L/(Ku_2));
V_characteristic_3= sqrt(vehicleData.L/(Ku_3));

%% Task 1.4 Steering wheel angle for one certain operating point
a_y= 4; % m/s^2
V_x= 100/3.6; % m/s

Rp4= V_x^2/a_y; %path radius(m)
%c0rresponding steering angles for the load cases
delta_f1= vehicleData.steeringRatio*((vehicleData.L/Rp4)+((Ku_1*V_x^2)/Rp4))*180/pi;
delta_f2= vehicleData.steeringRatio*((vehicleData.L/Rp4)+((Ku_2*V_x^2)/Rp4))*180/pi;
delta_f3= vehicleData.steeringRatio*((vehicleData.L/Rp4)+((Ku_3*V_x^2)/Rp4))*180/pi;

%% Task 1.5 Steering wheel angle for varying operating point
V_x_vector= 0:0.5:50;
Rp5=200; %Path radius(m)
%steering angles for the velocity vector with a given path radius

angle_1= vehicleData.steeringRatio*((vehicleData.L/Rp5)+((Ku_1.*V_x_vector.^2)/Rp5))*180/pi;
angle_2= vehicleData.steeringRatio*((vehicleData.L/Rp5)+((Ku_2.*V_x_vector.^2)/Rp5))*180/pi;
angle_3= vehicleData.steeringRatio*((vehicleData.L/Rp5)+((Ku_3.*V_x_vector.^2)/Rp5))*180/pi;


lol=figure;
plot(V_x_vector,angle_1,'r')
hold on
plot(V_x_vector,angle_2,'g')
plot(V_x_vector,angle_3,'b')
xlabel('velocity(m/s)')
ylabel('Steering angle(degrees)')
title('Steering angle vs velocity')
legend('understeer-Load case 1','oversteer-Load case 2','neutral steer-Load case 3','Location','SouthWest')

%% Task 2





