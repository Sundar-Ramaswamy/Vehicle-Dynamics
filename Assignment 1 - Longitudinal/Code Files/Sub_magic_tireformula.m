function [mu, surface_type] = Sub_magic_tireformula(slip,road_cond)
%----MAGIC TIRE formula----------------------------------------------------

%----Parameters for different road surfaces--------------------------------
if road_cond == 1
    %----Dry Asphalt
    C=1.45; D=1.00; E0=-4.00;
    surface_type = 'Dry';
elseif road_cond == 2
    %----wet Asphalt
    C=1.35; D=0.60; E0=-0.20;
    surface_type = 'Wet';
elseif road_cond == 3
    %----Ice
    C=1.50; D=0.10; E0=0.80;
    surface_type = 'Ice';
end

%----Common parameters-----------------------------------------------------
K  = 3*pi/180;
B  = atan(K)/(C*D);

%----Calculate the available friction--------------------------------------
mu = D*sin(C*atan(B*slip*100-E0*(B*slip*100-atan(B*slip*100))));

end

