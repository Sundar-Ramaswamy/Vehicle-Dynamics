% Vehicle Dynamics 2017 HT
% Main file: Simulation_main.m for Assignment 1
%--------------------------------------------------------------------------
% 2012-09-18 by Ulrich Sander
% 2015-10-26 by Pär Pettersson
% 2017-10-30 by Toheed Ghandriz
%--------------------------------------------------------------------------
clear all;      % Clear all variables
close all;      % Close all windows
clc;            % Clear command window

%----Define vehicle parameters---------------------------------------------
CONST = Sub_read_data;

%-----Define torque distribution between front and rear axis [0 ... 1]
CONST.torqf = 1; % torqueRatio: 0 => RWD, 1 => FWD

%----Define road condition-------------------------------------------------
% Function input
roadCondition = 1; % dry=1, wet=2, ice=3 

%----Define slope of road--------------------------------------------------
% Function input
slope = 2*pi/180; % Slope of the road in 2 deg!

%----Task 1: Define initial conditions-------------------------------------
s    = 0;                % Initial travelled distance
v    = 0.1;              % Initial velocity
wf   = (1.05*v)/CONST.R;    % Initial rotational velocity of front wheel
wr   = (1.05*v)/CONST.R;    % Initial rotational velocity of rear wheel
gear = 1;

%----Define timestep and integration time----------------------------------
Tstart = 0;            % Starting t
Tend   = 18;           % Ending t
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
Tdrivf_max0 = Fzfapprox*mutilmax*CONST.R;
Tdrivr_max0 = Fzrapprox*mutilmax*CONST.R;
Tdrivf_max = Tdrivf_max0;
Tdrivr_max = Tdrivr_max0;
%----Simulation over t-----------------------------------------------------
for i=1:length(t)
    % Calcultate slip
    slipf=Sub_wheel_slip(v,wf,CONST);
    slipr=Sub_wheel_slip(v,wr,CONST);
    
    %----Simulation of vehicle dynamics based on full throttle-------------
    % Calculate Tdriv, gear and engine speed
    [Tdrivf,Tdrivr,gear,weng]=Sub_drivetrain(wf,wr,gear,CONST);
    
    % Solve vehicle dynamics equations with use of driver torque adjustment
    % from anti-slip function
    
    frontSlipAboveOptimal = slipf > slipopt;
    rearSlipAboveOptimal = slipr > slipopt;
    frontTorqueAboveMax = Tdrivf > Tdrivf_max;
    rearTorqueAboveMax = Tdrivr > Tdrivr_max;
  
  
    % Update slip for anti-slip control
    if frontSlipAboveOptimal
        slipf = slipopt;
    end;
    if rearSlipAboveOptimal
        slipr = slipopt;
    end;
    
    if frontTorqueAboveMax
        Tdrivf = Tdrivf_max;
    end
    if rearTorqueAboveMax
        Tdrivr = Tdrivr_max;
    end;
    
    [a,wdotf,wdotr,Fzf,Fzr]=Sub_vehicle_dynamics(...
            v,Tdrivf,Tdrivr,slipf,slipr,slope,roadCondition,CONST);
    
    % Updating the distance, velocity, and rotational velocity
    s=s+v*dt;
    v=v+a*dt;
    wf=wf+wdotf*dt;
    wr=wr+wdotr*dt;
    
    % Update drive torques and slip for anti-slip control
    Tdrivf_max = Fzf*mutilmax*CONST.R; 
    Tdrivr_max = Fzr*mutilmax*CONST.R;

    % Saving result in vector at position i
    a_vector(i)=a;
    v_vector(i)=v;
    s_vector(i)=s;
    slipf_vector(i)=slipf;
    slipr_vector(i)=slipr;
    wf_vector(i)=wf;
    wr_vector(i)=wr;
    Fzf_vector(i)=Fzf;
    Fzr_vector(i)=Fzr;
    wdotf_vector(i)=wdotf;
    wdotr_vector(i)=wdotr;
    Tdrivf_vector(i)=Tdrivf;
    Tdrivr_vector(i)=Tdrivr;
    gear_vector(i)=gear;
    weng_vector(i)=weng;

end %

% ----Plot the results------------------------------------------------------
Sub_plot(t,a_vector,v_vector,s_vector,Tdrivf_vector,Tdrivr_vector,...
    Fzf_vector,Fzr_vector,slipf_vector, slipr_vector,gear_vector, ...
    weng_vector);