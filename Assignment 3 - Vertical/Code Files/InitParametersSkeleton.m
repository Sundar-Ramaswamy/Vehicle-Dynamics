%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, initialize parameters
% 
% Add your own code where "%ADD YOUR CODE HERE" is stated
%
clear all;
close all;
clc;

set(0,'DefaultAxesFontSize',12)
set(0,'DefaultAxesLineWidth',2)
set(0,'defaultlinelinewidth',2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters [SI units]

%
totalMass = 1860;               %ADD YOUR CODE HERE;
totalUnsprungMass = 185;       %ADD YOUR CODE HERE;
totalSprungMass = 1860-185;       %ADD YOUR CODE HERE;
wheelBase = 2.675  ;            %ADD YOUR CODE HERE
distanceCogToFrontAxle = 0.4*wheelBase ; %ADD YOUR CODE HERE
frontWheelSuspStiff = 30800 ;    %ADD YOUR CODE HERE;
rearWheelSuspStiff = 29900 ;     %ADD YOUR CODE HERE;
frontWheelSuspDamp = 4500  ;    %ADD YOUR CODE HERE;
rearWheelSuspDamp = 3500  ;     %ADD YOUR CODE HERE;
tireStiff = 230000  ;             %ADD YOUR CODE HERE;
tireDamp = 0    ;           %ADD YOUR CODE HERE;

% Other parameters Task 1
%
frequencyVector = [0.1 : 0.01 : 50]';
angularFrequencyVector = 2 * pi * frequencyVector;
deltaAngularFrequency = 2 * pi * 0.01;

% notations for substitution
cs= frontWheelSuspStiff ;
cr=rearWheelSuspStiff ;
ct=tireStiff ;
ds=frontWheelSuspDamp;
dr=rearWheelSuspDamp ;
dt=tireDamp ;
% Uncomment following from Task 2 and 3

% Other parameters Task 2
%
roadDisplacementAmplitude = 1;
roadSeverity =   10*10^(-6)  ;       %Input data
vehicleVelocity =  80/3.6 ; %km/h to m/sec is vx/3.6     
roadWaviness =  2.5       ;   % Input data
frontWheelSuspDampVector = [1000 : 100 : 9000];
frontWheelSuspStiffVector = [0.5 0.75 1 1.25 1.5 2]*frontWheelSuspStiff;

% Other parameters Task 3
%
roadSeveritySmooth =  1e-6  ;  %ADD YOUR CODE HERE;
roadSeverityRough =  10e-6   ;  %ADD YOUR CODE HERE;
roadSeverityVeryRough = 100e-6 ;  %ADD YOUR CODE HERE;
roadWavinessSmooth =  3  ;  %ADD YOUR CODE HERE;
roadWavinessRough =  2.5 ;    %ADD YOUR CODE HERE;
roadWavinessVeryRough = 2 ; %ADD YOUR CODE HERE;

totalDrivingTime = 8*60*60;               % 8h driving period
totalDistance = 160E3;                    % total distance of route (there and back)
distanceSmoothRoad = 0.70*totalDistance;   % distribution of different road types
distanceRoughRoad = 0.175*totalDistance;
distanceVeryRoughRoad = 0.125*totalDistance;

vehicleVelocitySmooth = 110/3.6;    % max allowed vehicle velocity on any road type 100km/h
vehicleVelocityRough = 110/3.6;
vehicleVelocityVeryRough = 110/3.6;

