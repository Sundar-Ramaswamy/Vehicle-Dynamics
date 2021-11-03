%% Chassis 

function [accelerationGlobal,rotAcc,accelerationLocal] = VehDynModel(state,delta,Fx,Fy,vehicleData)
    %   © CASTER / MMF062
    %   Created: 2019-11-18
    % Inputs:
    %   state  - state vector
    %   delta  - steering wheel angle [rad]
    %   Fx     - Vector of forces from the wheels in x-direction
    %   Fy     - Vector of forces from the wheels in y-direction
    % Outputs: 
    %   accelerationGlobal  - global vehicle acceleration
    %                         [vxDot; vyDot; vzDot]
    %   rotAcc              - vector of vehicle rotational accelerations
    %                         [wx; wy; wz]
    %   accelerationLocal   - local vehicle acceleration 
    %                         [ax; ay; az]
    %
    % ----- Parameters -----
    %%========================DO NOT CHANGE THIS=============================%%
    m         = vehicleData.m;            % Mass of vehicle
    J         = vehicleData.J;
    lf        = vehicleData.lf;           % Distance of CoG from front axle
    lr        = vehicleData.lr;           % Distance of CoG from rear axle

    % State vector
    vx        = state(1);
    vy        = state(2);
    wz        = state(3);
  
    %%=======================================================================%%
    % Equations of motion

    vxDot     = (m*wz*vy+Fx(1)*cos(delta)-Fy(1)*sin(delta)+Fx(2))/m;       % Derivative of longtidunial velocity
    vyDot     = (-m*wz*vx+Fx(1)*sin(delta)+Fy(1)*cos(delta)+Fy(2))/m;       % Derivative of lateral velocity
    wzDot     = ((Fx(1)*sin(delta)+Fy(1)*cos(delta))*lf-Fy(2)*lr)/J;       % Derivative of yaw rotational velocity
    
    %%========================DO NOT CHANGE THIS=============================%%
    vzDot     = 0;
    wxDot     = 0;
    wyDot     = 0;
      

    % Accelerations
    accelerationGlobal = [vxDot;vyDot;vzDot];
    ax                 = vxDot-wz*vy; 
    ay                 = vyDot+wz*vx;
    az                 = 0;
    accelerationLocal  = [ax;ay;az];

    % Rotational accelerations
    rotAcc             = [wxDot;wyDot;wzDot];
    %%=======================================================================%%
end

% end of file


%% Tiremodel/latforce
function [Fy,Mz]  = LatForce(FzWhl,Fx,slipAngle,vehicleData)
%#codegen
    %   © CASTER / MMF062
    %   Created: 2019-11-18
    % Inputs:
    %   FzWhl      - Normal load on tires [N]
    %   Fx         - Longitudinal force vector [N]
    %   slipAngle  - Slip angle vector [rad]
    % Outputs:
    %   Fy         - A [2x1] vector with axle lateral forces [N]
    %   Mz         - Dummy value for StrWhlTrq [Nm]
    %
    %%========================DO NOT CHANGE THIS=============================%%
    c0          = vehicleData.c0;                       % Tyre stiffness parameter
    c1          = vehicleData.c1;                       % Tyre stiffness parameter
    mu          = 0.8;

    FzWhl       = max(FzWhl,[0;0;0;0]);                 % Limit minimum value
    FzFrontAxle = FzWhl(1)+FzWhl(2);                    % Front axle normal load
    FzRearAxle  = FzWhl(3)+FzWhl(4);                    % Rear axle normal load
    FzAxle      = [FzFrontAxle;FzRearAxle];
    %%=======================================================================%%
    
    % Axle stiffness calculation (keep in mind that the tyre data is for one
    % wheel and not an axle)
    maxLatForce = [0;0];
    
    % Fx could have been limited when it is calculated
    if (Fx(1)^2>=(mu*FzFrontAxle)^2)
        Cf             = 1e-3;                             % Solving the numerical problems
        maxLatForce(1) = 0;
    else
        %%=======================================================================%%
        Cf             =  (c0*FzWhl(1)+(c1*FzWhl(1)^2))+(c0*FzWhl(2)+(c1*FzWhl(2)^2));                             % Front cornering stiffness
        %%=======================================================================%%
        
        maxLatForce(1) = sqrt((mu*FzAxle(1))^2-Fx(1)^2);   % Friction circle
        
        %%=============================FOR TASK 4================================%%
        % Combined Slip
         Cf             = (sqrt((mu*FzAxle(1))^2-Fx(1)^2)/(mu*FzAxle(1)))*Cf;
        %%=======================================================================%%
    end
    
    if (Fx(2)^2>=(mu*FzRearAxle)^2)
        Cr             = 1e-3;                             % Solving the numerical problems
        maxLatForce(2) = 0;
    else
        %%=======================================================================%%
        Cr             =  (c0*FzWhl(3)+(c1*FzWhl(3)^2))+(c0*FzWhl(4)+(c1*FzWhl(4)^2));                             % Rear cornering stiffness
        %%=======================================================================%%
        
        maxLatForce(2) = sqrt((mu*FzAxle(2))^2-Fx(2)^2);   % Friction circle
        
        %%=============================FOR TASK 4================================%%
        % Combined Slip
        Cr             =  (sqrt((mu*FzAxle(2))^2-Fx(1)^2)/(mu*FzAxle(2)))*Cr;
        %%=======================================================================%%
    end
     Ffy= -Cf*slipAngle(1);
     Fry= -Cr*slipAngle(2);
    % Lateral force
    CAxle = [Cf;Cr];
    Fy    =  [Ffy;Fry];
    
    %%========================DO NOT CHANGE THIS=============================%%
    Fy    = min(maxLatForce,abs(Fy)).*sign(Fy);            % Saturation of Fy to sqrt(Fz^2-Fx^2)
    %%=======================================================================%%
    % Self-aligning moment
    A  = -0.001;
    C  = 0.4;
    Mz = A*Fy(1)*C;
    %%=======================================================================%%
end

%% Lateral slip equations

function slipAngle   = LateralSlip(state,delta,vehicleData) 
    %   © CASTER / MMF062
    %   Created: 2019-11-18
    % Inputs:
    %   state     - State vector
    %   delta     - steering wheel angle [rad]
    % Outputs:
    %   slipangle - A [2x1] vector containing the slipratio(0-1) for each axle [-]
    %
    %%========================DO NOT CHANGE THIS=============================%%
    % ----- Parameters -----
    % State vector
    vx = state(1);
    vy = state(2);
    wz = state(3);
    lf = vehicleData.lf;           % Distance of CoG from front axle
    lr = vehicleData.lr;           % Distance of CoG from rear axle
    % Preventing vx from being zero 
    vx = max(abs(vx),3);
   
    

    %%=======================================================================%%
    % Slip equations
    vfxw=(vy+lf*wz)*sin(delta)+(vx*cos(delta));
    vfyw=(vy+lf*wz)*cos(delta)-(vx*sin(delta));
    vrxw = vx; 
    vryw = vy - lr*wz;
    alphaf    = atan(vfyw/vfxw);   %  Front lateral tire slip 
    alphar    = atan(vryw/vrxw);   %  Rear lateral tire slip
%       alphaf    = atan(((vy+lf*wz)/vx))-(delta);   %  Front lateral tire slip 
%       alphar    = atan((vy-lr*wz)/vx); 
    slipAngle = [alphaf; alphar];  % Slip angle vector
    %%=======================================================================%%

end

% end of file

%% load transfer block

    function FzWhl  = RollModel(accelerationLocal,rollStiffDist,vehicleData)
    %   © CASTER / MMF062
    %   Created: 2019-11-18
    % Inputs: 
    %   accelerationLocal - vector of vehicle loacl acceleration [ax; ay; az;]
    %   rollStiffDist     - Roll stiffness distribution 0-1 on the front axle [-]
    % Outputs: 
    %   FzWhl             - vector of tyre normal forces [N]
    %
    % ----- Parameters -----
    
    %%========================DO NOT CHANGE THIS=============================%%
    h       = vehicleData.h;            % Center of gravity height
    g       = vehicleData.g;            % Acceleration due to gravity
    m       = vehicleData.m;            % Mass of vehicle
    cRoll   = vehicleData.cw;           % Total roll stiffness
    h1      = vehicleData.hrcf;         % Front roll center height
    h2      = vehicleData.hrcr;         % Rear roll center height
    L       = vehicleData.L;            % Wheelbase
    lf      = vehicleData.lf;           % Distance of CoG from front axle
    lr      = vehicleData.lr;           % Distance of CoG from rear axle
    dh      = h-(h1*lr+h2*lf)/L;        % Height of CoG from roll axis
    w       = vehicleData.w;            % Vehicle track width
    cfRoll  = cRoll*rollStiffDist;      % Front roll stiffness
    crRoll  = cRoll-cfRoll;             % Rear roll stiffness

    %%=======================================================================%%
    % Longitudinal and lateral acceleration
    ax      = accelerationLocal(1); 
    ay      = accelerationLocal(2);
    
    % Longitudinal load transfer (should be modified by students for tasks 3 and 4.)
    deltaFflzLong = -(m*ax*h)/(2*L);                      % Front left
    deltaFfrzLong = -(m*ax*h)/(2*L);                      % Front right
    deltaFrlzLong = (m*ax*h)/(2*L);                      % Rear left
    deltaFrrzLong = (m*ax*h)/(2*L);                      % Rear right  

    % Lateral load transfer (should be modified by students for tasks 3 and 4.)
    deltaFflzLat  = -ay*m*(((h1*lr)/(L*w))+((dh/w)*(cfRoll/cRoll)));    % Front left
    deltaFfrzLat  =ay*m*(((h1*lr)/(L*w))+((dh/w)*(cfRoll/cRoll)));    % Front right
    deltaFrlzLat  = -ay*m*(((h2*lf)/(L*w))+((dh/w)*(crRoll/cRoll)));    % Rear left
    deltaFrrzLat  = ay*m*(((h2*lf)/(L*w))+((dh/w)*(crRoll/cRoll)));    % Rear right
    %%=======================================================================%%

    % Collecting longitudinal and lateral load transfers
    deltaFzLong   = [deltaFflzLong; deltaFfrzLong; deltaFrlzLong; deltaFrrzLong];
    deltaFzLat    = [deltaFflzLat; deltaFfrzLat; deltaFrlzLat; deltaFrrzLat];

    % Total load transfer
    dFz           = deltaFzLong+deltaFzLat;
    FzStatic      = m*g/2/L*[lr; lr; lf; lf];
    FzWhl         = FzStatic+dFz;
    
    %%=======================FOR DRIVING SIMULATOR===========================%%
    % Switch off the load transfer if needed.
    % FzWhl         = FzStatic;
    
    %%========================DO NOT CHANGE THIS=============================%%
    % Wheel lift-off scenario. This is not really *handling* wheel lift-off,
    % but rather making sure the simulation does not crash in the event of
    % wheel lift-off. Solution may still be incorrect. Handling wheel lift-off
    % is not straight forward. So this simulation model is not good for cases
    % with excessive or frequent wheel lift-off.
    if any(FzWhl<=0)
        FzWhl(FzWhl<1e-3) = 1e-3;
    end
end

% end of file

