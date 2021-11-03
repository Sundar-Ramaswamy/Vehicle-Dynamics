%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 3

clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

run InitParametersSkeleton.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 3.1

numberOfTripsPerDay = totalDrivingTime/...
    (distanceSmoothRoad/vehicleVelocitySmooth+distanceRoughRoad/vehicleVelocityRough+distanceVeryRoughRoad/vehicleVelocityVeryRough);

% i)
% Calculate road spectrum
roadSpectrumSmooth = zeros(length(angularFrequencyVector),1);
roadSpectrumRough = zeros(length(angularFrequencyVector),1);
roadSpectrumVeryRough = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    roadSpectrumSmooth(j,:) = (vehicleVelocitySmooth^(roadWavinessSmooth-1))*roadSeveritySmooth*angularFrequencyVector(j)^-roadWavinessSmooth; %ADD YOUR CODE HERE
    roadSpectrumRough(j,:) =(vehicleVelocityRough^(roadWavinessRough-1))*roadSeverityRough*angularFrequencyVector(j)^-roadWavinessRough; %ADD YOUR CODE HERE
    roadSpectrumVeryRough(j,:) = (vehicleVelocityVeryRough^(roadWavinessVeryRough-1))*roadSeverityVeryRough*angularFrequencyVector(j)^-roadWavinessVeryRough; %ADD YOUR CODE HERE
end

% Calculate transfer functions for front wheel Zr to Ride
% You can use result from Task 1

sprungMassFront = (totalSprungMass*0.6)/2; % ,The load is considered w.r.t 2 axles,Also we hav condition where lf is 0.4*Wheelbase
unsprungMassFront = totalUnsprungMass/4 ;% for wheels
ms=sprungMassFront ; %notation considered
mu= unsprungMassFront ; %notation considered
% Identify A and B matrix
Af =  [0 0 1 0 ; ...
       0 0 0 1; ...
       -cs/ms cs/ms -ds/ms ds/ms ;...
       cs/mu (-ct-cs)/mu ds/mu (-dt-ds)/mu ];
Bf = [0 0 ;...
      0 0 ;...
      0 0 ;...
      ct/mu dt/mu];
  
% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 3) front wheel Zr to Tyre force
% similar to Task 1 define C and D matrices for both cases

C1f = [1 0 0 0] ;%
D1f = [0] ;%

% matrices for Zr to Zr to Tyre force, front wheel:
C3f = [0 1 0 0];%ADD YOUR CODE HERE
D3f = [0] ;%ADD YOUR CODE HERE

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    transferFunctionFrontZrToRide(j,:) = -angularFrequencyVector(j)^2*C1f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D1f ;%ADD YOUR CODE HERE 
end

% Calculate acceleration response spectrum for all roads

psdAccelerationSmooth = zeros(length(angularFrequencyVector),1);
psdAccelerationRough = zeros(length(angularFrequencyVector),1);
psdAccelerationVeryRough = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    psdAccelerationSmooth(j,:) = (abs(transferFunctionFrontZrToRide(j)))^2*roadSpectrumSmooth(j); %ADD YOUR CODE HERE
    psdAccelerationRough(j,:) = (abs(transferFunctionFrontZrToRide(j)))^2*roadSpectrumRough(j); %ADD YOUR CODE HERE
    psdAccelerationVeryRough(j,:) = (abs(transferFunctionFrontZrToRide(j)))^2*roadSpectrumVeryRough(j); %ADD YOUR CODE HERE
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ii)
% Calculate rms values weighted according to ISO2631 
[weightedRmsAccelerationSmooth] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationSmooth)
[weightedRmsAccelerationRough] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationRough)
[weightedRmsAccelerationVeryRough] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationVeryRough)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% iii)
% Calculate time averaged vibration exposure value for 8h period
timeOnSmoothRoad = (distanceSmoothRoad / vehicleVelocitySmooth)*numberOfTripsPerDay
timeOnRoughRoad = (distanceRoughRoad / vehicleVelocityRough)*numberOfTripsPerDay
timeOnVeryRoughRoad = (distanceVeryRoughRoad / vehicleVelocityVeryRough)*numberOfTripsPerDay

timeWeightedMsAcceleration = (weightedRmsAccelerationSmooth.^2*timeOnSmoothRoad + weightedRmsAccelerationRough.^2*timeOnRoughRoad + weightedRmsAccelerationVeryRough.^2*timeOnVeryRoughRoad )/(timeOnSmoothRoad + timeOnRoughRoad + timeOnVeryRoughRoad); %ADD YOUR CODE HERE

timeWeightedRmsAcceleration = sqrt(timeWeightedMsAcceleration);

disp(['timeWeightedRmsAcceleration =' num2str(timeWeightedRmsAcceleration),' m/s2 (1.15)',...
    ', numberOfTripsPerDay = ' num2str(numberOfTripsPerDay)]);
disp(['vehicleVelocitySmooth = ' num2str(vehicleVelocitySmooth*3.6),' km/h',...
    ', vehicleVelocityRough = ' num2str(vehicleVelocityRough*3.6),' km/h',...
    ', vehicleVelocityVeryRough = ' num2str(vehicleVelocityVeryRough*3.6),' km/h'])

