function  [vDot,omegaDotf,omegaDotr,Fzf,Fzr]=...
    Sub_vehicle_dynamics(v,Tdrivf,Tdrivr,slipf,slipr,slope,road_cond,CONST)

% Read parameters
c_d=CONST.c_d;
A_f=CONST.A_f;
air=CONST.air;

M=CONST.M;
Ir=CONST.Ir; %[kg*m^2] Moment of inertia for two wheels and axle 
g=CONST.g;

h=CONST.h;
L=CONST.L;
L_f=CONST.Lf;
L_r=CONST.Lr;

R=CONST.R;
f_r=CONST.f_r; % rolling resistance coefficient

% Calculate air resistance 
Fair=0.5*c_d*A_f*air*v^2;

% Magic Tire Formula
muf=Sub_magic_tireformula(slipf,road_cond);
mur=Sub_magic_tireformula(slipr,road_cond);

% Write the explicit solutions here!

Fzf = ;    %[N] front normal force
Fzr = ;    %[N] rear normal force
omegaDotf = ; %[rad/s/s] rotational acceleration, front
omegaDotr = ; %[rad/s/s] rotational acceleration, rear
vDot = ;     %[m/s/s] acceleration

end