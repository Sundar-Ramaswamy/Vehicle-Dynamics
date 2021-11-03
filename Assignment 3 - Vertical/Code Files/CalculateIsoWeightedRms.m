function [weightedRmsAcceleration] = CalculateIsoWeightedRms(...
    frequencyVector,psdAcceleration)
%
% Calculate ISO2631 filtered rms value (vertical direction)
%
% To be used in MMF062 Vertical Dynamics Assignment, Task 3
% No modifications needed
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute weight as specified in ISO 2361-1:1997(E)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

omegaVector = 2.0*pi*frequencyVector;
p = 0.0+1i*omegaVector;

% Weighting Wk
% Highpass ...
  f1 = 0.4;
  om1 = 2*pi*f1;
  whp = p.*p./(p.*p+sqrt(2.0)*om1*p+om1*om1);
% Low pass ...
  f2 = 100.0;
  om2 = 2*pi*f2;
  wlp = 1.0./(1.0+sqrt(2.0)*p./om2+p.*p/(om2*om2));
% Acceleration-velocity transition ...
  f3 = 12.5;
  f4 = 12.5;
  q4 = 0.63;
  om3 = 2*pi*f3;
  om4 = 2*pi*f4;
  wavt = (1.0+p/om3)./(1.0+p./(q4*om4)+p.*p/(om4*om4));
% Upward step ...
  f5 = 2.37;
  q5 = 0.91;
  f6 = 3.35;
  q6 = 0.91;
  om5 = 2*pi*f5;
  om6 = 2*pi*f6;
  wus = (1.0+p/(q5*om5)+p.*p/(om5*om5))./(1.0+p/(q6*om6)+p.*p/(om6*om6))...
      *om5*om5/(om6*om6);
weight = whp .* wlp .* wavt .* wus;

weightedRmsAcceleration = 0;
for j = 1 : length(psdAcceleration)
    weightedRmsAcceleration = weightedRmsAcceleration + ...
        (abs(weight(j))^2*psdAcceleration(j)*(omegaVector(2)-omegaVector(1)));
end
weightedRmsAcceleration = sqrt(weightedRmsAcceleration);

% figure;
% semilogy(frequency,psdAcceleration,frequency,psdAcceleration.*abs(weight));
% grid;
% legend('no-iso','iso',-1)
% xlabel('Frequency [Hz]');
% ylabel('Acceleration [(m/s^2)^2/Hz]');
% title('Sprung mass acceleration PSD');

