function SenseData = SensorModel(X)
% ***********************************************************
% --------------- SENSOR MODEL --------------
% DESCRIPTION:
%     Scan threw six LM given as map, 
%     here each LM is distinct
%
% ARGUEMNT:
%   X : actual state, [X, Y, Theta]'
%   LM : land mark position
%
% RETURN:
%   Sensordata : each row [flag, distance, relative angle]
%   flag = 1 if corresponding LM detected
% --------------------------------------------
% ***********************************************************

LM_X = [-20 -20 -5  30   30 8];
LM_Y = [-25  0  25  -25  0  25];
LMc = length(LM_X);

SenseData = zeros(LMc,3);

%  Sensor Noise Covariance
Q = [
  0.02    0.00;
  0.00    0.02;
];

for id = 1:LMc  
    % A radius based detection, use physical distance!
    q = sqrt((LM_X(id) - X(1))^2 + (LM_Y(id) - X(2))^2);
	
	% if the landmark is in the range of sensing area
    if  q < 12.0
        ran = mvnrnd([q atan2(LM_Y(id) - X(2),  ...
		                      LM_X(id) - X(1)) - X(3)
					 ], Q);
        
        SenseData(id,1) = 1;         % is sensed
        SenseData(id,2) = ran(1);
        SenseData(id,3) = ran(2);
        
	end   
end
end