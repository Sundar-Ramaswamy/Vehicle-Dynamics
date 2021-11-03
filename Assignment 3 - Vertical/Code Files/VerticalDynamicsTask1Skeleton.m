%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 1
%
% Add your own code where "%ADD YOUR CODE HERE" is stated
%
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 run InitParametersSkeleton.m
% 
% InitParameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.3
%
% Consider one single front wheel, identify sprung mass and unsprung mass

sprungMassFront = (totalSprungMass*0.6)/2;  %  % The load is considered w.r.t 2 axles,Also we hav condition where lf is 0.4*Wheelbase
unsprungMassFront = totalUnsprungMass/4; % for wheels
ms=sprungMassFront ;%notation
mu= unsprungMassFront ;
% Identify indiviual A and B matrix+
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
% 2)front wheel Zr to Suspension travel and 
% 3) front wheel Zr to Tyre force

% matrices Zr to Ride,front wheel:
C1f = [1 0 0 0] ;%ADD YOUR CODE HERE
D1f = [0] ;%ADD YOUR CODE HERE

% matrices for Zr to Suspension travel, front wheel:
C2f = [-1 1 0 0] ; %ADD YOUR CODE HERE
D2f =[0] ; %ADD YOUR CODE HERE

% matrices for Zr to Zr to Tyre force, front wheel:
C3f = [0 1 0 0];%ADD YOUR CODE HERE
D3f = [0] ;%ADD YOUR CODE HERE

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

%consider bf1=4x1 matrix as we are ignoring tire damping.

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionFrontZrToRide(j,:) =-angularFrequencyVector(j)^2*C1f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D1f; %"For improved robustness,  replacing j by 1i"
    transferFunctionFrontZrToTravel(j,:) =(C2f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D2f);%ADD YOUR CODE HERE
    transferFunctionFrontZrToForce(j,:) = ct*(1-(C3f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D3f));%ADD YOUR CODE HERE
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Consider one single rear wheel, identify sprung mass and unsprung mass

sprungMassRear = (totalSprungMass*0.4)/2 %ADD YOUR CODE HERE
unsprungMassRear = totalUnsprungMass/4  %ADD YOUR CODE HERE
msr=sprungMassRear ;
mur= unsprungMassRear ;


% Identify indiviual A and B matrix
Ar = [0 0 1 0 ; ...
       0 0 0 1; ...
       -cr/msr cr/msr -dr/msr dr/msr ;...
       cr/mur (-ct-cr)/mur dr/mur (dt-dr)/mur ] ; %ADD YOUR CODE HERE
Br = [0 0 ;...
      0 0 ;...
      0 0 ;...
      ct/mur dt/mur] ; %ADD YOUR CODE HERE

% Calculate transfer functions for 
% 1) rear wheel Zr to Ride, 
% 2) rear wheel Zr to Suspension travel and 
% 3) rear wheel Zr to Tyre force

% matrices Zr to Ride, rear wheel:
C1r = [1 0 0 0] ; %ADD YOUR CODE HERE
D1r =[0] ; %ADD YOUR CODE HERE

% matrices for Zr to Suspension travel, rear wheel:
C2r = [-1 1 0 0] ;%ADD YOUR CODE HERE
D2r = [0] ;%ADD YOUR CODE HERE

% matrices for Zr to Zr to Tyre force, rear wheel:
C3r = [0 1 0 0]; %ADD YOUR CODE HERE
D3r = [0] ; %ADD YOUR CODE HERE

% Rear wheel
transferFunctionRearZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToForce = zeros(length(angularFrequencyVector),1);


for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionRearZrToRide(j,:) = -angularFrequencyVector(j)^2*C1r*inv(eye(4)*1i*angularFrequencyVector(j)-Ar)* Br(:,1) + D1r;%ADD YOUR CODE HERE
    transferFunctionRearZrToTravel(j,:) = (C2r*inv(eye(4)*1i*angularFrequencyVector(j)-Ar)* Br(:,1) + D2r);%ADD YOUR CODE HERE
    transferFunctionRearZrToForce(j,:) = ct*(1-(C3r*inv(eye(4)*1i*angularFrequencyVector(j)-Ar)* Br(:,1) + D3r)); %ADD YOUR CODE HERE
end

% Plot the transfer functions
figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToRide)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToRide)),'--r');
axis([0 50 -10 60]);grid
legend('Front','Rear','Location','northwest');
xlabel('Frequency-Hz');%ADD YOUR CODE HERE
ylabel('Ride comfort-m/s2');%ADD YOUR CODE HERE
title('Magnitude of transfer function Ride Comfort');


figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToTravel)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToTravel)),'--r');
axis([0 50 -50 10]);grid
legend('Front','Rear','Location','northwest');
xlabel('Frequency-Hz');%ADD YOUR CODE HERE
ylabel('Suspension Travel-m');%ADD YOUR CODE HERE
title('Magnitude of transfer function Suspension Travel');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToForce)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToForce)),'--r');
axis([0 50 40 115]);grid
legend('Front','Rear','Location','northwest');
xlabel('Frequency-Hz');%ADD YOUR CODE HERE

ylabel('Road grip');%ADD YOUR CODE HERE
title('Magnitude of transfer function Road Grip');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Task 1.4
%
% Identify natural frequencies

% Front wheel
resonanceFreqFrontBounce = (sqrt((1/(1/cs)+(1/ct))/ms))*0.159 ;%radians to hertz conversion
resonanceFreqFrontHop = (sqrt((cs+ct)/mu))*0.159 ; %radians to hertz conversion

% Rear wheel
resonanceFreqRearBounce = (((1/(1/cr+1/ct))/msr)^(1/2))*0.159;%radians to hertz conversion
resonanceFreqRearHop = (((cr+ct)/mur)^(1/2))*0.159;%radians to hertz conversion
