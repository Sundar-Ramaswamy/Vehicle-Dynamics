syms d_1 c_1 e_1 K_1 B_1 slip slip_1 slip_2;

d_1 = 0.95;
c_1 = 1.5;
e_1 = -4;
K_1  = 3*pi/180;
B_1  = atan(K_1)/(c_1*d_1);
slip = 0.25;
slip_1 =0.6
slip_2 = 0.95;
mu = d_1*sin(c_1*atan(B_1*slip*100-e_1*(B_1*slip*100-atan(B_1*slip*100))))
mu_1 = d_1*sin(c_1*atan(B_1*slip_1*100-e_1*(B_1*slip_1*100-atan(B_1*slip_1*100))))
mu_2 = d_1*sin(c_1*atan(B_1*slip_2*100-e_1*(B_1*slip_2*100-atan(B_1*slip_2*100))))