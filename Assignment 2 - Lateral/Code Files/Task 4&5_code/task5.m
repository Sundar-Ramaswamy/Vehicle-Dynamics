clc;
clf;
clear all;
% 
% load('Driver_1_20_80.mat')
% load('Driver_1_80_20.mat')
% load('Driver_2_20_80.mat')
% load('Driver_2_80_20.mat')
% load('Driver_3_20_80.mat')
 load('Driver_3_80_20.mat')


figure(1)
plot((position(:,1)),(position(:,2)),'--')
grid on
xlabel('Position along Xdirection [m]')
ylabel('Position along Ydirection [m]')




