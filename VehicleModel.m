function new_pose = VehicleModel(v,w,previous_pose)
% **********************************************
% ----------- VEHICLE MODEL -----------
% DESCRIPTION:
%   - Mimic real world respond
% ARGUMENT:
%   v             : foward speed
%   w             : rotation speed
%   previous_pose : [X, Y, Theta]'
% 
% RETURN:
%   X : resulting pose
% --------------------------------------
% **********************************************
X     = 1;
Y     = 2;
THETA = 3;

% Enviroment uncertainty coeficient
c1 = 0.05;
c2 = 0.05;
c3 = 0.05;
c4 = 0.05;


% c1 = 0.01;
% c2 = 0.01;
% c3 = 0.01;
% c4 = 0.01;

% sampling interval (Unit: second)
del_t = 1;

% Covariance of input interference
r = [
  (c1*abs(v)+c2*abs(w))^2     0;
  0                           (c3*abs(v)+c4*abs(w))^2
];

% sample from the probability distribution
ran = mvnrnd([v w],r);

v_actual = ran(1);
w_actual = ran(2);

% Physical actual output
change_pose = [
    v_actual * del_t * cos(previous_pose(THETA) + w_actual * del_t);
    v_actual * del_t * sin(previous_pose(THETA) + w_actual * del_t);
    w_actual * del_t;
];

new_pose = previous_pose + change_pose;
end