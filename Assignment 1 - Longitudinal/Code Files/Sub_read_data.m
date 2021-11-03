function CONST=Sub_read_data

%--------------------------------------------------------------------------
% Read vehicle data with use of specific vehicle data file
%--------------------------------------------------------------------------

%----Loading vehicle parameters--------------------------------------------
Saab_93_datasheet;    % Load data from the Saab 9-3 data file.

%----Chassis---------------------------------------------------------------
CONST.M   = m;            %[kg] Curb weight
CONST.mb  = m-m_unspr;    %[kg] Vehicle sprung mass
CONST.mf  = m_unspr/2;    %[kg] Unsprung mass front wheels
CONST.mr  = m_unspr/2;    %[kg] Unsprung mass rear wheels
CONST.L   = L;            %[m]  Wheel base 
CONST.Lf  = l_f;          %[m]  Distance along X-axis from CoG to front axle Default Data
CONST.Lr   = L-CONST.Lf;          %[m]  Distance along X-axis from CoG to rear axle
CONST.h   = h;            %[m]  Distance along Z-axis from CoG to front axle.
CONST.A_f = A_f;          %[m^2] Front area
CONST.c_d = c_d;          %[-]  Air drag coefficient 


%----Tire/Wheel------------------------------------------------------------
CONST.R   = radius_wheel; %[m]  Wheel radius
CONST.Ir  = J_wheel;      %[kg*m^2] Moment of inertia for two wheels and axle 
CONST.f_r = RRconst;         %[-]  Rolling resistance coefficient

%----Environment-----------------------------------------------------------
CONST.g   = 9.81;         %[m/s^2]  Gravitational constant
CONST.air = air;          %[kg/m^3] Air density 

%--------------------------------------------------------------------------
%-----------------------------Added for Task 3-----------------------------
%----Driveline-------------------------------------------------------------
CONST.Tmax_vec    = Tmax_vec;    %[Nm] Maximum engine torque vector, corresponding to w_vec
CONST.w_vec       = w_vec;       %[rad/s] Engine rotational speed, corresponding to Tmax_vec
CONST.ratio_vec   = ratio_vec;   %Gear ratios
CONST.ratio_final = ratio_final; %Ratio of final gear

%--------------------------------------------------------------------------
%-----------------------------Added for Task 4-----------------------------
CONST.I           = 2*J_wheel;   % [kg*m^2] Moment of inertia for four wheels and two axles 
CONST.ntrans      = 0.9;         % Efficiency coefficient for transmission
CONST.fuel        = 860;      % [kg/m^3] Fuel density

%----Loading vehicle parameters--------------------------------------------
%Engine_data;           % Load engine data from the data file.
%----Engine-------------------------------------------------------------
CONST.w_vec_fc    = w_vec;    % Engine speed vector corresponding to fuel consumption table [rad/s]
CONST.T_vec_fc    = Tmax_vec;    % Engine torque vector corresponding to fuel consumption table [Nm]

end
