
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 2
% 
%
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

 run InitParametersSkeleton.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 2.1
%
% Front wheel

% Calculate road spectrum
roadSpectrum = zeros(length(angularFrequencyVector),1);

for i = 1 : length(angularFrequencyVector)
    roadSpectrum(i,:) = (vehicleVelocity^(roadWaviness-1))*roadSeverity*angularFrequencyVector(i)^(-roadWaviness);% equation 5.29 compendium
end
% Calculate transfer functions for front wheel Zr to Ride and Tyre force
% You can use the code from Task 1

% Consider one single front wheel
sprungMassFront = (totalSprungMass*0.6)/2; % The load is considered w.r.t 2 axles,Also we hav condition where lf is 0.4*Wheelbase
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
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionFrontZrToRide(j,:) =-angularFrequencyVector(j)^2*C1f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D1f; %%"For improved robustness,  replacing j by 1i"
    transferFunctionFrontZrToForce(j,:) = ct*(1-(C3f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D3f));%ADD YOUR CODE HERE
end

% Calculate acceleration and tyre force response spectrum
psdAcceleration = zeros(length(angularFrequencyVector),1);
psdForce = zeros(length(angularFrequencyVector),1);

for m = 1 : length(angularFrequencyVector)
    psdAcceleration(m,:) = (abs(transferFunctionFrontZrToRide(m)))^2*roadSpectrum(m);
    psdForce(m,:) = (abs(transferFunctionFrontZrToForce(m)))^2*roadSpectrum(m) ; 
end

% Calculate rms values of acceleration and tyre force
msAcceleration = 0;
msForce = 0;

for n = 1 : length(angularFrequencyVector)
    msAcceleration = msAcceleration + psdAcceleration(n)*deltaAngularFrequency ; %ADD YOUR CODE HERE
    msForce = msForce + psdForce(n)*deltaAngularFrequency; %ADD YOUR CODE HERE
end

rmsAcceleration = sqrt(msAcceleration);
rmsForce = sqrt(msForce);

figure(100);
semilogy(frequencyVector,psdAcceleration);grid
xlabel('Frequency-Hz');
ylabel('psdAcceleration [(m/s^2)^2/Hz]');
title('Sprung mass acceleration PSD');
legend(['rms value = ',num2str(rmsAcceleration)])

figure(21);
semilogy(frequencyVector,psdForce);grid
xlabel('Frequency-Hz');
ylabel('psdForce [(N)^2/Hz]');
title('Tyre force PSD');
legend(['rms value = ',num2str(rmsForce)])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 2.2
%
% Front wheel
rmsAcceleration = zeros(length(frontWheelSuspStiffVector),length(frontWheelSuspDampVector));
rmsForce = zeros(length(frontWheelSuspStiffVector),length(frontWheelSuspDampVector));

for ind1 = 1 : length(frontWheelSuspStiffVector)
    
    for ind2 = 1 : length(frontWheelSuspDampVector)
        
        % Update suspension stiffness and damping
        cs = frontWheelSuspStiffVector(ind1);
        ds = frontWheelSuspDampVector(ind2);
        % Update A and C matrices 
         
       Af =  [0 0 1 0 ; ...
       0 0 0 1; ...
       -cs/ms cs/ms -ds/ms ds/ms ;...
       cs/mu (-ct-cs)/mu ds/mu (-dt-ds)/mu ];% A matrice task 1
       
       C1f = [1 0 0 0] ;% C matrice task 1
       % matrices for Zr to Zr to Tyre force, front wheel used for transfer function:
        C3f = [0 1 0 0];%
        D3f = [0] ;%
               
        % Calculate transfer functions for front wheel Zr to Ride and Tyre force
        
            transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
            transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);
 
            for j = 1 : length(angularFrequencyVector)
                % Calculate H(w) not the absolut value |H(w)|
                transferFunctionFrontZrToRide(j,:) = -angularFrequencyVector(j)^2*C1f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D1f; %ADD YOUR CODE HERE
                transferFunctionFrontZrToForce(j,:) = ct*(1-(C3f*inv(eye(4)*1i*angularFrequencyVector(j)-Af)* Bf(:,1) + D3f)); %ADD YOUR CODE HERE
            end
               
        
        % Calculate acceleration and tyre force response spectrum
        psdAcceleration = zeros(length(angularFrequencyVector),1);
        psdForce = zeros(length(angularFrequencyVector),1);
        
        for m = 1 : length(angularFrequencyVector)
            psdAcceleration(m,:) = (abs(transferFunctionFrontZrToRide(m)))^2*roadSpectrum(m);%
            psdForce(m,:) = (abs(transferFunctionFrontZrToForce(m)))^2*roadSpectrum(m) ; %
        end
        
        % Calculate rms values of acceleration and tyre force
        msAcceleration = 0;
        msForce = 0;
        
        for n = 1 : length(angularFrequencyVector)
            msAcceleration = msAcceleration + psdAcceleration(n)*deltaAngularFrequency ; %
            msForce = msForce + psdForce(n)*deltaAngularFrequency ;%
        end
        
        rmsAcceleration(ind1,ind2) = sqrt(msAcceleration);
        rmsForce(ind1,ind2) = sqrt(msForce);
       
    end
end

% Plot rms values vs damping
figure(22);
plot(frontWheelSuspDampVector,rmsAcceleration);grid
legend(num2str(frontWheelSuspStiffVector'));
xlabel('FrontWheelSuspDamping[Nsec/m]'); %ADD YOUR CODE HERE
ylabel('rmsAcceleration[m/sec^2]'); %ADD YOUR CODE HERE
title('Sprung mass acceleration vs chassis damping for various spring stiffness');
axis([0 10000 0 3]);

figure(23);
plot(frontWheelSuspDampVector,rmsForce);grid
legend(num2str(frontWheelSuspStiffVector'));
xlabel('FrontWheelSuspDamping[Nsec/m]'); %ADD YOUR CODE HERE
ylabel('rmsForce[N]'); %ADD YOUR CODE HERE
title('Dynamic tyre force vs chassis damping for various spring stiffness');
axis([0 10000 0 1400]);

% Identify optimal damping values for each stiffness value
[optimalRmsAcceleration,iOptimalRmsAcceleration] = min(rmsAcceleration,[],2);
[optimalRmsForce,iOptimalRmsForce] = min(rmsForce,[],2);

% Plot optimal damping for Ride and Tyre force vs Stiffness
figure(24);
plot(frontWheelSuspStiffVector,frontWheelSuspDampVector(iOptimalRmsAcceleration)); %ADD YOUR CODE HERE
hold on
plot(frontWheelSuspStiffVector,frontWheelSuspDampVector(iOptimalRmsForce))
grid
xlabel('Optimal damping[Nsec/m]'); %ADD YOUR CODE HERE
ylabel('spring stiffness[N/m]'); %ADD YOUR CODE HERE
title('Optimal damping vs spring stiffness');
legend('rmsAcce','rmsForce');
axis([10000 65000 0 4000]);

