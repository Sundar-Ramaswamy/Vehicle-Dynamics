% Postprocessing after the simulation. Generates plots of the relevant
% vehicle states and parameters. 

% x - [LongVel LatVel YawVel LongAcc LatAcc YawAcc]

% Some figure formatting
set(0,'DefaultAxesFontSize',10)
set(0,'DefaultAxesLineWidth',0.5)
set(0,'defaultlinelinewidth',0.5)

vx = x(:,1);
vy = x(:,2);
wz = x(:,3);
ax = y(:,1);
ay = y(:,2);
% wzDot = x(:,6);

%% Longitudinal velocity
% figure('Name','Longitudinal Velocity');
figure(1)
subplot(2,4,1)
plot(t,vx*3.6)
grid on;
xlabel('Time [s]')
ylabel('Longitudinal velocity [km/h]')

%% Lateral velocity
% figure('Name','Lateral velocity');
figure(1)
subplot(2,4,2)
plot(t,vy*3.6)
grid on;
xlabel('Time [s]')
ylabel('Lateral velocity [km/h]')

%% Yaw rate
% figure('Name','Yaw Rate');
figure(1)
subplot(2,4,3)
plot(t,wz*180/pi,t,delta.*vx/vehicleData.L*180/pi,'r--')
grid on;
ylabel('Yaw rate [deg/s]')
xlabel('Time [s]')
legend('Actual','Neutral Steer')

%% Vehicle travel animation
figure(2);
hTrajectory = plot(X(1),Y(1),'b--');
hTitle=title(sprintf('Time = %0.3g',t(1)));
axis equal;
xlim([min(X)-10 max(X)+10])
ylim([min(Y)-10 max(Y)+10])
axis manual;
grid on;
hold on;
xlabel('X [m]')
ylabel('Y [m]')

lf = vehicleData.lf*5;
lr = vehicleData.lr*5;
w  = vehicleData.w*5;

X1 =  lf*cos(psi) - w/2*sin(psi);
X2 =  lf*cos(psi) + w/2*sin(psi);
X3 = -lr*cos(psi) + w/2*sin(psi);
X4 = -lr*cos(psi) - w/2*sin(psi);

Y1 =  w/2*cos(psi) + lf*sin(psi);
Y2 = -w/2*cos(psi) + lf*sin(psi);
Y3 = -w/2*cos(psi) - lf*sin(psi);
Y4 =  w/2*cos(psi) - lf*sin(psi);

hCar = patch([X1(1) X2(1) X3(1) X4(1)], [Y1(1) Y2(1) Y3(1) Y4(1)],'red','facecolor','none','edgecolor','red');

xLimits = get(gca,'xlim');
yLimits = get(gca,'ylim');
axis manual;
xlim(xLimits)
ylim(yLimits)

for i = 1:1/(sampleTime*100):length(psi)
    set(hTrajectory,'XData',X(1:i),'YData',Y(1:i))
    set(hCar,'Xdata',X(i)+[X1(i) X2(i) X3(i) X4(i)], 'Ydata',Y(i)+[Y1(i) Y2(i) Y3(i) Y4(i)]);
    set(hTitle,'String',sprintf('Time= %0.3g',t(i)));
    drawnow;
end

%% Vehicle path and orientation
% figure('Name','Path');
figure(3)
plot(X,Y,'b--')
axis equal;
grid on;
hold on;
xlabel('X [m]')
ylabel('Y [m]')

for i = 1:1/(sampleTime):length(psi)
    patch(X(i)+[X1(i) X2(i) X3(i) X4(i)], Y(i)+[Y1(i) Y2(i) Y3(i) Y4(i)],'red','facecolor','none','edgecolor','red');
end

lf = vehicleData.lf;
lr = vehicleData.lr;
w = vehicleData.w;
m = vehicleData.m;
g = vehicleData.g;
L = vehicleData.L;

%% Vehicle lateral acceleration
% figure('Name','Lateral Acceleration');
figure(1)
subplot(2,4,4)
plot(t,ay)
grid on;
xlabel('Time [s]')
ylabel('Lateral acceleration [m/s^2]')

%% Vehicle longitudinal acceleration
% figure('Name','Longitudinal Acceleration');
figure(1)
subplot(2,4,5)
plot(t,ax)
grid on;
xlabel('Time [s]')
ylabel('Longitudinal acceleration [m/s^2]')

%% Lateral and longitudinal axle forces
% figure('Name','Lateral and longitudinal forces');
figure(1)
subplot(2,4,6)
plot(t,Fxw,t,Fyw)
grid on;
xlabel('Time [s]')
ylabel('Longitudinal and lateral axle forces [N]')
legend('Fx Front wheel-long','Fx Rear wheel-long','Fy Front wheel-lat','Fy Rear wheel-lat')

%% Axle front and rear slip angles
% figure('Name','Slip angles');
figure(1)
subplot(2,4,7)
plot(t,slipAngle*180/pi)
grid on;
xlabel('Time [s]')
ylabel('Axle slip angles [deg]')
legend('Front','Rear')

%% Normal load on the wheels
% figure('Name','Normal loads');
figure(1)
subplot(2,4,8)
plot(t,FzWheel)
grid on;
xlabel('Time [s]')
ylabel('Wheel normal loads [N]')
legend('Front left','Front right','Rear left','Rear right')

% end of file