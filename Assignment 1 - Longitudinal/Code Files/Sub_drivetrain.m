function [Tdrivf,Tdrivr,gear,weng]=Sub_drivetrain(wf,wr,gear_old,CONST)
% This function finds a gear that provides highest engine torque. 
% This function knowing the wheel speed, gear ratios and final gear finds
% all possible engine speeds and corresponding engine torques (based on the 
% given engine map). Then from
% engine torques using all gears calculates all wheel torques (one for
% each gear). Then selects the highest one and corresponding gear as output.
% i.e. Tdriv (for front or rear depending on the value of 
% CONST.torqf). Down-shifting is avoided. /Toheed

w_vec=CONST.w_vec;
Tmax_vec=CONST.Tmax_vec;
ratio_vec=CONST.ratio_vec;
ratio_final=CONST.ratio_final;

% Maximum engine speed
w_eng_max=CONST.w_vec(end);

% Engine speed at maximum torque
[~, b]=max(Tmax_vec);

% Minimum engine speed is set to speed at maximum torque
w_eng_min=w_vec(b); % Maxium torque is immediately reached when releasing clutch in 1st gear

% Determination of the driven axis
if CONST.torqf == 0
    w_ref = wr;
elseif CONST.torqf == 1
    w_ref = wf;
else
    w_ref = 0.5*(wf+wr);
end;

% Engine speed in all gears
w_eng_all_gears = (ratio_final*ratio_vec*w_ref);

% Set minimum engine speed in all gears
for i=1:5;
    if (w_eng_all_gears(i) < w_eng_min)
        w_eng_all_gears(i) = w_eng_min;
    end
end

% Calculate drive torque in all gears
Teng_all_gears=interp1(w_vec,Tmax_vec,w_eng_all_gears);  % Engine torque for all gears
Tdriv_all_gears=ratio_final*ratio_vec.*Teng_all_gears;    % Tdriv for all gears

% Limit drive torque to maximum engine speed
for i=1:length(Tdriv_all_gears)
    if (w_eng_all_gears(i) > w_eng_max)
        Tdriv_all_gears(i) = 0;     
    end
end


% Check to avoid down shifting
[~,gear]=max(Tdriv_all_gears);  % Max Tdriv and corresponding gear
gear = max(gear, gear_old);         % Select higher gear or use previous gear
Tdriv = Tdriv_all_gears(gear);      % Set drive torque for selected gear
Tdrivf = CONST.torqf*Tdriv;        % Set drive torque for front axis
Tdrivr = (1-CONST.torqf)*Tdriv;    % Set drive torque for rear axis
weng=w_eng_all_gears(gear);         % Set engine speed for selected gear

end