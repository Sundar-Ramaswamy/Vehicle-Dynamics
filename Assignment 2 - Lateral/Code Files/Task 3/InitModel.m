%% Data for the simulation.
% Data corresponds to Saab 9-3.

% Vehicle data
sampleTime                  = 1e-2;                             % Sample time for data output
vehicleData.m               = 1675;                             % Mass
vehicleData.J               = 2617;                             % Vehicle inertia about Z axis
vehicleData.L               = 2.675;                            % Wheelbase
vehicleData.c0              = 30.7;                             % Tyre stiffness parameter
vehicleData.c1              = -0.00235;                         % Tyre stiffness parameter
vehicleData.steeringRatio   = 15.9;                             % Steering ratio
vehicleData.h               = 0.543;                            % Height of CoG
vehicleData.hrcf            = 0.045;                            % Front roll center height
vehicleData.hrcr            = 0.101;                            % Rear roll center height
vehicleData.cw              = 7e4;                              % Total roll stiffness
vehicleData.w               = 1.51;                             % Track width
vehicleData.g               = 9.81;                             % Acceleration due to gravity
vehicleData.R               = 0.3;                              % Wheel radius

disp('Saab 9-3 parameters loaded');

% Parameters to be changed in different tasks:
vehicleData.lf              = 0.47*vehicleData.L;               % Distance of CoG from front axle
vehicleData.lr              = vehicleData.L-vehicleData.lf;     % Distance of CoG from rear axle 
simulationTime              = 65;                               % Simulation time
vx0                         = 100;                              % Initial speed
vehicleData.brakeDemand     = 0.0;                              % Brake force demand
SWA                         = 10;                               % Steering wheel angle                                        
vehicleData.pRollDist       = 0.6;                             % Front/Total roll stiffness distribution
vehicleData.brakeDist       = 0.5;                              % Front/Total brake distribution ratio
% end of file