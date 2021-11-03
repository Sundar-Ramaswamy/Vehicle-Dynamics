%% Task 2 b&c
syms F_fx F_rx F_fz F_rz w_f_dot w_r_dot L_f L_r v_dot h J T e m g theta F_air  R_l mu
%longitudinal:
equ1 = F_fx+F_rx-m*g*sin(theta)-m*v_dot-F_air==0;

%front and rear axle, around respective wheel centre:
equ4 = J*w_f_dot-T+e*F_fz+R_l*F_fx==0;
equ5 = F_fz-mu*F_fx==0;
equ6 = J*w_r_dot-T+e*F_rz+R_l*F_rx==0;
equ7 = F_rz-mu*F_rx==0;


%% Task 2 d


%if h=H_air;
equ8 = -m*g*cos(theta)*L_f-m*g*sin(theta)*h+F_rz*(L_f+L_r)-m*v_dot*h-F_air*h==0;
equ9 = -m*g*sin(theta)*h+m*g*cos(theta)*L_r-F_fz*(L_f+L_r)-m*v_dot*h-F_air*h==0;
[F_fx F_rx F_fz F_rz w_f_dot w_r_dot v_dot ] = solve(equ1,equ8,equ9,equ4,equ5,equ6,equ7,F_fx,F_rx,F_fz,F_rz,w_f_dot,w_r_dot,v_dot)