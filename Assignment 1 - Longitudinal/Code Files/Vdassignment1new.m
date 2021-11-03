%%Vehicle Dynamics Assignment 1 Group 13
% Longitudinal 


%Group members
%1. Sundar Murugan Ramaswamy - murugan@student.chalmers.se
%2. Surya Prakash Avaragere Lokesh - prsurya@student.chalmers.se


clc;
clear;


%% Task 1a

%constants 
D = [1 .6 .1];
C = [1.45 1.35 1.5];
E = [-4 -.2 .8];
K = (3*pi)/180;
Sx = 0:.05:1;

for i=[1 2 3]
        B(i)=(100*(atan(K)))/(C(i)*D(i));
        i=i+1;
end

    for j=1:length(Sx)
        Tf1(j)=D(1)*sin(C(1)*atan(B(1)*Sx(j)-E(1)*(B(1)*Sx(j)-atan(B(1)*Sx(j)))));
        j=j+1;
    end
    
    for j=1:length(Sx)
        Tf2(j)=D(2)*sin(C(2)*atan(B(2)*Sx(j)-E(2)*(B(2)*Sx(j)-atan(B(2)*Sx(j)))));
        j=j+1;
    end
    
        for j=1:length(Sx)
        Tf3(j)=D(3)*sin(C(3)*atan(B(3)*Sx(j)-E(3)*(B(3)*Sx(j)-atan(B(3)*Sx(j)))));
        j=j+1;
        end   
        
hemp=figure;
hold on
plot(Sx,Tf1,'r', "LineWidth",1.5)
plot(Sx,Tf2,'b', "LineWidth",1.5)
plot(Sx,Tf3,'c', "LineWidth",1.5)
xlabel ('Longitudinal Slip (Sx)');
ylabel('Normalized Traction Force (Tf)');
legend('Dry Asphalt','Wet Asphalt','Ice')
title('Tf vs Sx (Theoretical)','fontsize',11)
hold off

%%Task 1b

bemp=figure;
Tf = [0 .25 .53 .77 .89 .95 .94 .92 .90 .86 .85 .83 .81 .80 .79 .78 .77 .76 .75 .74];
Sx1= 0:.05:.95;


plot(Sx1,Tf, "LineWidth",1.5)
xlabel ('Longitudinal Slip (Sx)');
ylabel('Normalized Traction Force (Tf)');
title('Tf vs Sx (Experimental)','fontsize',11)

%% Curve fitting
Da= .95;
Ca=1.5;
Ea=-4;
Ba=(100*(atan(K)))/(Ca*Da);
Tfa=Da*sin(Ca*atan(Ba*Sx-Ea*(Ba*Sx-atan(Ba*Sx))));
temp=figure;
hold on
plot(Sx,Tfa,'r', "LineWidth",1.5)
plot(Sx1,Tf,'b', "LineWidth",1.5)
xlabel ('Longitudinal Slip (Sx)');
ylabel('Normalized Traction Force (Tf)');
legend('Curve fit','Dry Asphalt')
title('Tf vs Sx (Curve fitting)','fontsize',11)
hold off

%when E=0
Eb=0;
Tfb=Da*sin(Ca*atan(Ba*Sx-Eb*(Ba*Sx-atan(Ba*Sx))));
zemp=figure;
plot(Sx,Tfb,'r', "LineWidth",1.5)
xlabel ('Longitudinal Slip (Sx)');
ylabel('Normalized Traction Force (Tf)');
title('Tf vs Sx (When E=0)','fontsize',11)


%% Task 2

% For flat road
%Known parameters
syms m g h L_f L_r J Tf Tr mu_f mu_r Fair R e 
%Unknown Parameters
syms Ffz Frz Ffx Frx omega_r omega_f a

soln=solve(...
    Ffx+Frx-(m*a)-Fair==0, ...
    Ffz*(L_f+L_r)+(m*a*h)+(Fair*h)-(m*g*L_r)==0, ...
    Frz*(L_f+L_r)-(m*a*h)-(Fair*h)-(m*g*L_f)==0, ...
    Tf-(J*omega_f)-(Ffx*R)-(Ffz*e)==0, ...
    Tr-(J*omega_r)-(Frx*R)-(Frz*e)==0, ...
    Ffx-(mu_f*Ffz)==0, ...
    Frx-(mu_r*Frz)==0, ...
    Ffz, Frz, Ffx, Frx, omega_f, omega_r, a);
Ffz= soln.Ffz;
Frz= soln.Frz;
Ffx= soln.Ffx;
Frx= soln.Frx;
omega_f= soln.omega_f;
omega_r= soln.omega_r;
a= soln.a;


%For road with gradient
%Known parameters
syms m g h L_f L_r J Tf Tr mu_f mu_r Fair R e  slope
%Unknown Parameters
syms Ffz Frz Ffx Frx omega_r omega_f a
soln2=solve(...
    Ffx+Frx-(m*a)-Fair-(m*g*sin(slope))==0, ...
    Ffz*(L_f+L_r)+(m*a*h)+(Fair*h)-(m*g*L_r*cos(slope))+(m*g*h*sin(slope))==0, ...
    Frz*(L_f+L_r)-(m*a*h)-(Fair*h)-(m*g*L_f*cos(slope))-(m*g*h*sin(slope))==0, ...
    Tf-(J*omega_f)-(Ffx*R)-(Ffz*e)==0, ...
    Tr-(J*omega_r)-(Frx*R)-(Frz*e)==0, ...
    Ffx-(mu_f*Ffz)==0, ...
    Frx-(mu_r*Frz)==0, ...
    Ffz, Frz, Ffx, Frx, omega_f, omega_r, a);
Ffz1= soln2.Ffz;
Frz1= soln2.Frz;
Ffx1= soln2.Ffx;
Frx1= soln2.Frx;
omega_f1= soln2.omega_f;
omega_r1= soln2.omega_r;
a1= soln2.a;

%% Task 2E

CONST= Sub_read_data;

m=1675;
L=2.675;
L_f=1.07;
L_r=1.605;
h=0.543;
A_f=2.17;
C_d=0.3;
R_wheel=0.316;
RRC=0.0164;
e=RRC*R_wheel;
g=9.81;
rho_air=1.3;
J_wheel=2;
slip=0:.05:1;
%----Define road condition-------------------------------------------------
% Function input
roadCondition =1; % dry=1, wet=2, ice=3 

mu= Sub_magic_tireformula(slip,roadCondition); %where 1 is the road condition
s    = 0;                % Initial travelled distance
v    = 0.1;              % Initial velocity
wf   = (1.05*v)/R_wheel;    % Initial rotational velocity of front wheel
wr   = (1.05*v)/R_wheel;    % Initial rotational velocity of rear wheel
gear = 1;
CONST.torqf = 1;
[Tdrivf,Tdrivr,gear,weng]=Sub_drivetrain(wf,wr,gear,CONST);
Tf=Tdrivf;
Tr=Tdrivr;

% mu_f=mu_r;
% mu_f=mu;
R_l=R_wheel;
J=J_wheel;
R=R_wheel;

%----Define slope of road--------------------------------------------------
% Function input
slope = 8*pi/180; % Slope of the road in 8 degrees in radians

%----Task 1: Define initial conditions-------------------------------------
% s    = 0;                % Initial travelled distance
% v    = 0.1;              % Initial velocity
% wf   = (1.05*v)/R_wheel;    % Initial rotational velocity of front wheel
% wr   = (1.05*v)/R_wheel;    % Initial rotational velocity of rear wheel
% gear = 1;
F_air=0.5*rho_air*A_f*v^2*C_d;
%----Define timestep and integration time----------------------------------
Tstart = 0;            % Starting t
Tend   = 20;           % Ending t
dt     = 0.0005;       % t step
t=(Tstart:dt:Tend);    % t vector

%----Preallocate arrays for speed------------------------------------------
a_vector=zeros(size(t));
v_vector=zeros(size(t));
s_vector=zeros(size(t));
slipf_vector=zeros(size(t));
slipr_vector=zeros(size(t));
wf_vector=zeros(size(t));
wr_vector=zeros(size(t));
Fzf_vector=zeros(size(t));
Fzr_vector=zeros(size(t));
wdotf_vector=zeros(size(t));
wdotr_vector=zeros(size(t));
Tdrivf_vector=zeros(size(t));
Tdrivr_vector=zeros(size(t));
gear_vector=zeros(size(t));
weng_vector=zeros(size(t));

%----Anti-slip control initialisation--------------------------------------
Fzfapprox = CONST.M*CONST.g*CONST.Lr/CONST.L;
Fzrapprox = CONST.M*CONST.g*CONST.Lf/CONST.L;
mue_step = 0.001;
[mu_vec,surface] = Sub_magic_tireformula(0:mue_step:1,roadCondition);
[mutilmax, idMuMax] = max(mu_vec);
slipopt = idMuMax*mue_step;

% Tdrivf_max0 = Fzfapprox*mutilmax*CONST.R;
% Tdrivr_max0 = Fzrapprox*mutilmax*CONST.R;
% Tdrivf_max = Tdrivf_max0;
% Tdrivr_max = Tdrivr_max0;
%----Simulation over t-----------------------------------------------------
for i=1:length(t)
    % Calcultate slip
   slipf=(wf*R_wheel-v)/abs(wf*R_wheel);
    slipr=(wr*R_wheel-v)/abs(wr*R_wheel);
    
    frontSlipAboveOptimal = slipf > slipopt;
    rearSlipAboveOptimal = slipr > slipopt;
%     frontTorqueAboveMax = Tdrivf > Tdrivf_max;
%     rearTorqueAboveMax = Tdrivr > Tdrivr_max;
%   
  
    % Update slip for anti-slip control
    if frontSlipAboveOptimal
        slipf = slipopt;
    end;
    if rearSlipAboveOptimal
        slipr = slipopt;
    end;
   
    mu_f=Sub_magic_tireformula(slipf,roadCondition);
    mu_r=Sub_magic_tireformula(slipr,roadCondition);

 

    a = -(F_air*L_f + F_air*L_r + F_air*h.*mu_f - F_air*h.*mu_r + L_f*g*m*sin(slope) + L_r*g*m*sin(slope) - L_f*g*m.*mu_r.*cos(slope) - L_r*g*m.*mu_f.*cos(slope) + g*h*m.*mu_f.*sin(slope) - g*h*m.*mu_r.*sin(slope))/(L_f*m + L_r*m + h*m.*mu_f - h*m.*mu_r);

    w_f_dot =(L_f*Tf + L_r*Tf + Tf*h.*mu_f - Tf*h.*mu_r - L_r*e*g*m*cos(slope) - L_r*R*g*m.*mu_f.*cos(slope) + e*g*h*m.*mu_r.*cos(slope) + R*g*h*m.*mu_f.*mu_r.*cos(slope))/(J*(L_f + L_r + h.*mu_f - h.*mu_r));

 
 
    w_r_dot =-(Tr*h.*mu_r - L_r*Tr - Tr*h.*mu_f - L_f*Tr + L_f*e*g*m*cos(slope) + L_f*R*g*m.*mu_r.*cos(slope) + e*g*h*m.*mu_f.*cos(slope) + R*g*h*m.*mu_f.*mu_r.*cos(slope))/(J*(L_f + L_r + h.*mu_f - h.*mu_r));

    F_fz =(L_r*g*m*cos(slope) - g*h*m.*mu_r.*cos(slope))/(L_f + L_r + h.*mu_f - h.*mu_r);
 
 
    F_rz =(L_f*g*m*cos(slope) + g*h*m.*mu_f.*cos(slope))/(L_f + L_r + h.*mu_f - h.*mu_r);
  

  
    % Updating the distance, velocity, and rotational velocity
    s=s+v*dt;
    v=v+a*dt;
    wf=wf+w_f_dot*dt;
    wr=wr+w_r_dot*dt;

    % Saving result in vector at position i
    a_vector(i)=a;
    v_vector(i)=v;
    s_vector(i)=s;
    slipf_vector(i)=slipf;
    slipr_vector(i)=slipr;
    wf_vector(i)=wf;
    wr_vector(i)=wr;
    Fzf_vector(i)=F_fz;
    Fzr_vector(i)=F_rz;
    wdotf_vector(i)=w_f_dot;
    wdotr_vector(i)=w_r_dot;
    Tdrivf_vector(i)=Tf;
    Tdrivr_vector(i)=Tr;
    gear_vector(i)=gear;

end %

% ----Plot the results------------------------------------------------------
Sub_plot(t,a_vector,v_vector,s_vector,Tdrivf_vector,Tdrivr_vector,...
    Fzf_vector,Fzr_vector,slipf_vector, slipr_vector,gear_vector, ...
    weng_vector);
