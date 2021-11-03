%% Task 1 a
clc;
clear all;
clear table;
D = [1 0.6 0.1]; %D for dry asphalt, wet asphalt and ice
C = [1.45 1.35 1.5]; %C for dry asphalt, wet asphalt and ice
E = [-4 -0.2 0.8];%E for dry asphalt, wet asphalt and ice
K = (3*pi)/180;
B = 100 * (atan(K)./(C.*D));


sx =  0:0.05:0.95;
fxbyfz = [0.0,0.25,0.53,0.77,0.89,0.95,0.94,0.92,0.90,0.86,0.85,0.83,0.81,0.80,0.79,0.78,0.77,0.76,0.75,0.74];
figure(1)
plot (sx, fxbyfz)
hold off
%% Task 1 b



sx =  0:0.05:0.95
fxbyfz = [0.0,0.25,0.53,0.77,0.89,0.95,0.94,0.92,0.90,0.86,0.85,0.83,0.81,0.80,0.79,0.78,0.77,0.76,0.75,0.74]
%  for i = i : 20
%      K = (3*pi)/180;
%      B = 100 * (atand(K)./(C.*D));
    E = [0,-2,-4]
%      sx =  0:0.05:0.95
%      fxbyfz = [0.0,0.25,0.53,0.77,0.89,0.95,0.94,0.92,0.90,0.86,0.85,0.83,0.81,0.80,0.79,0.78,0.77,0.76,0.75,0.74]
%      fxbyfz(i) = D*sin(C*atan(B*sx(i)-E(i)*(B*sx(i)*100-atan(B*sx(i)*100))))
%  end
% for i = 2
% syms C D
% eq = [E == 0, fxbyfz(i)- D(1).*sin(C(1).*atan(B(1).*sx(i)-E.*(B(1).*sx(i)-atan(B(1).*sx(i)))))==0]
% sol = solve (eq, [C D])
% solution = [vpa(sol.C) vpa(sol.D)]
% end

mu_dry = Sub_magic_tireformula(sx,1);
mu_wet = Sub_magic_tireformula(sx,2);
mu_ice = Sub_magic_tireformula(sx,3);

figure (2)
hold on
plot (sx,mu_dry,'r');
plot (sx,mu_wet,'b');
plot (sx,mu_ice,'c');
plot(sx, fxbyfz,'--b');
legend('dry asphalt','wet asphalt','ice','given normalized force vs slip','location','best');
hold off

%Since dry asphalt curve is the closest to the given curve, we assume dry
%asphalt conditions

% for i = 
% fxbyfz() = D(1)*sin(C(1)*atan(B(1)*sx()-E(1)*(B*sx()-atan(B*sx))))
% 
% figure(3)
% hold on
% plot(sx,fxbyfz,'b:');
% plot()
% 
syms D_1 C_1 E_1 K_1 B_1 slip;
%assumption
d_1 = 0.8;
c_1 = 1.42;
e_1 = -3;
K_1  = 3*pi/180;
B_1  = atan(K_1)/(c_1*d_1);
slip = 0.25;
mu = D*sin(c_1*atan(B_1*slip*100-e_1*(B_1*slip*100-atan(B_1*slip*100))));

%% Task 2 b&c
syms F_fx F_rx F_fz F_rz w_f_dot w_r_dot L_f L_r v_dot h J T e m g theta F_air H_air R_l mu
%longitudinal:
equ1 = F_fx+F_rx-m*g*sin(theta)-m*v_dot-F_air==0;
%rotational around front contact with ground:
equ2 = -m*g*cos(theta)*L_f-m*g*sin(theta)*h+F_rz*(L_f+L_r)-m*v_dot*h-F_air*H_air==0;
%rotational around rear contact with ground:
equ3 = -m*g*sin(theta)*h+m*g*cos(theta)*L_r-F_fz*(L_f+L_r)-m*v_dot*h-F_air*H_air==0;
%front and rear axle, around respective wheel centre:
equ4 = J*w_f_dot-T+e*F_fz+R_l*F_fx==0;
equ5 = F_fz-mu*F_fx==0;
equ6 = J*w_r_dot-T+e*F_rz+R_l*F_rx==0;
equ7 = F_rz-mu*F_rx==0;


%% Task 2 d
[F_fx F_rx F_fz F_rz w_f_dot w_r_dot v_dot] = solve(equ1,equ2,equ3,equ4,equ5,equ6,equ7,F_fx,F_rx,F_fz,F_rz,w_f_dot,w_r_dot,v_dot)


% if h=H_air;
%open the file 'task_2_d_2.m'

%% Task 2 e
CONST = Sub_read_data;

m=1675;
g=9.81; 
radius_wheel=2.675;
L=radius_wheel;%[m]      Wheel base
L_f=0.4*L;         %[m]      Distance along X-axis from CoG to front axle, MODIFY WEIGHT HERE!!!!
L_r=L-L_f;          %[m]      Distance along X-axis from CoG to rear axle
h=0.543;  
mu=Sub_magic_tireformula(slip,1);
radius_wheel=0.316;
R_l=radius_wheel;
RRconst=0.0164;     %[-]      Rolling resistance constant
e=RRconst;
J_wheel=2;          %[kg*m^2] Moment of inertia for two wheels and axle
J=J_wheel;

%-----Define torque distribution between front and rear axis
torque_f=2000; %n*m
torque_r=2000; %n*m

%----Define road condition-------------------------------------------------
% Function input
roadCondition = 1; % dry=1, wet=2, ice=3 

%----Define slope of road--------------------------------------------------
% Function input
slope = 5*pi/180; % Slope of the road in 5 deg!

%----Task 1: Define initial conditions-------------------------------------
s    = 0;                % Initial travelled distance
v    = 1;              % Initial velocity
wf   = (1.1*v)/radius_wheel;    % Initial rotational velocity of front wheel
wr   = (1.1*v)/radius_wheel;    % Initial rotational velocity of rear wheel
gear = 1;

air=1.3;            %[kg/m^3] Air density
c_d=0.30;           %[-]      Air drag coefficient 
F_air=0.5*air*c_d*v^2;

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
Fzfapprox = m*g*L_r/L;
Fzrapprox = m*g*L_f/L;
mue_step = 0.001;
[mu_vec,surface] = Sub_magic_tireformula(0:mue_step:1,roadCondition);
[mutilmax, idMuMax] = max(mu_vec);
slipopt = idMuMax*mue_step;
%----Simulation over t-----------------------------------------------------
for i=1:length(t)
    % Calcultate slip
    slipf=(wf*radius_wheel-v)/abs(wf*radius_wheel);
    slipr=(wr*radius_wheel-v)/abs(wr*radius_wheel);
       
    % Solve vehicle dynamics equations with use of driver torque adjustment
    % from anti-slip function
    
    frontSlipAboveOptimal = slipf > slipopt;
    rearSlipAboveOptimal = slipr > slipopt;
 
    % Update slip for anti-slip control
    if frontSlipAboveOptimal
        slipf = slipopt;
    end;
    if rearSlipAboveOptimal
        slipr = slipopt;
    end;


v_dot =-(F_air*mu - g*m*cos(slope) + g*m*mu*sin(slope))/(m*mu);

w_f_dot =(L_f*torque_r*mu^2 + L_r*torque_r*mu^2 + R_l*g*h*m*cos(slope) - L_r*R_l*g*m*mu*cos(slope) + e*g*h*m*mu*cos(slope) - L_r*e*g*m*mu^2*cos(slope))/(J*mu^2*(L_f + L_r));
 
 
w_r_dot =-(R_l*g*h*m*cos(slope) - L_r*torque_f*mu^2 - L_f*torque_f*mu^2 + L_f*R_l*g*m*mu*cos(slope) + e*g*h*m*mu*cos(slope) + L_f*e*g*m*mu^2*cos(slope))/(J*mu^2*(L_f + L_r));

F_fz =-(g*h*m*cos(slope) - L_r*g*m*mu*cos(slope))/(mu*(L_f + L_r));
 
 
F_rz =(g*h*m*cos(slope) + L_f*g*m*mu*cos(slope))/(mu*(L_f + L_r));
    
    % Updating the distance, velocity, and rotational velocity
    s=s+v*dt;
    v=v+v_dot*dt;
    wf=wf+w_f_dot*dt;
    wr=wr+w_r_dot*dt;
    

    % Saving result in vector at position i
    a_vector(i)=v_dot;
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
    Tdrivf_vector(i)=torque_f;
    Tdrivr_vector(i)=torque_r;
    gear_vector(i)=gear;

end %

% ----Plot the results------------------------------------------------------
Sub_plot(t,a_vector,v_vector,s_vector,Tdrivf_vector,Tdrivr_vector,...
    Fzf_vector,Fzr_vector,slipf_vector, slipr_vector,gear_vector, ...
    weng_vector);






