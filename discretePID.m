% Function for a discrete PID controller
%
% For the integral term, 2 methods are being used for numerical
% integration (Trapezoidal rule and Composire Simpson's 1/3 rule). We have 
% an advanced integral term handling based on whether k is even or odd.
%
%
%
% INPUTS: error, integralSumPrev, Kp,Ki,Kd (PID Gains), 
%         Ts (Sampling Time), k (time instance)
%
% OUTPUTS: output (PID's control signal), integralSum (updated integral sum)
%
%
% Author: Spiros Papadopoulos
%

function [output, integralSum] = discretePID(error, integralSumPrev, Kp, Ki, Kd, Ts, k)

    % Initialize the integral sum if not provided (on the first iteration)
    if isempty(integralSumPrev)
        integralSum = 0;
    else
        integralSum = integralSumPrev;
    end
   
    % Set P,I,D terms to zero
    proportionalTerm = 0; % P
    integralTerm = 0;     % I
    derivativeTerm = 0;   % D

    
    if k > 1 
        proportionalTerm = Kp * error(k);
        if mod(k,2) == 0
            integralTerm = Ki * Ts * (integralSum + (error(k) + error(k-1)) / 2);     % Trapezoidal rule
        else
            % Update the integral sum for the next iteration
            integralSum = integralSum + (error(k) + 4 * error(k-1) + error(k-2)) / 3; % Composite Simpson's rule
            integralTerm = Ki * Ts * integralSum;
        end
        derivativeTerm = Kd * (error(k) - error(k-1)) / Ts;
    end    
    
    % Calculate the controller output
    output = proportionalTerm + integralTerm + derivativeTerm;
end



