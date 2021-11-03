function slip=Sub_wheel_slip(v,w,CONST)

%----Calculate the slip----------------------------------------------------
% if v > w*CONST.R
%     slip=(w*CONST.R-v)/v;        % Calculate slip for braking   
% elseif v < w*CONST.R
%     slip=(w*CONST.R-v)/(w*CONST.R); % Calculate slip for traction
% else
%     slip = 0;
% end


%Alternatively, simply: 
slip=(w*CONST.R-v)/abs(w*CONST.R);
end