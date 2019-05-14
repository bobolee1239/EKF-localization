% MIT License
%
% Copyright (c) 2019 PooPoo, Tsung-Han Brian Lee
% *************************************************
% -------------------------------------------------
% FILE       : EKFsimulation.m
% DESCRIPTION: An example of EKF localizing
%              based on work of "Ayush Dewan"
%
% Recontributed by PooPoo, Tsung-Han Brian Lee
% -------------------------------------------------
% *************************************************
clc;close all; clear all;
clear figure

DEBUG = false;

toUpdateWithSensor = true;         %  Switch of executing correction step.

WATCH_SCOPE            = 35;       %  Range of x and y axis, DEFAULT: 35.
CLEAR_PATH_STEPS       = 150000;      %  #steps to clear previous path.

MLH_THRESHOLD_LANDMARK = 0;        %  Threshold to corrected with sensor
                                   %  information, only used when unkown
                                   %  coorespondency case.

X     = 1;                         %  Global index of x axis.
Y     = 2;                         %  Global index of y axis.
THETA = 3;                         %  Global index of angle.

% *********************************
% **** Do not move code below *****
% *********************************

v     = 0;                        % linear velocity command
w     = 0;                        % angular velocity command
del_t = 1;                        % samping interval

%  Stochastic uncertainty gain
c1 = 0.05;
c2 = 0.05;
c3 = 0.05;
c4 = 0.05;

% c1 = 0.01;
% c2 = 0.01;
% c3 = 0.01;
% c4 = 0.01;

%  Covariance of Sensor Noise
Q = [
  0.02    0.00;
  0.00    0.02;
];

%  Land marks position
LM_X = [-20 -20 -5  30   30 8];
LM_Y = [-25  0  25  -25  0  25];
LM_c = length(LM_X);

axis ([-WATCH_SCOPE WATCH_SCOPE -WATCH_SCOPE WATCH_SCOPE]);
hold on;
for i = 1:length(LM_X)
    scatter(LM_X(i), LM_Y(i), [], 'd');
end
title('Upgraded EKF LOCALIZATION (green: corrected, red: predicted)')
xlabel('x axis')
ylabel('y axis')

% *********************************
% **** Do not move code above *****
% *********************************

%  initial position of vehicle
x_p   = 2.5;
y_p   = -20;
phi_p = 0;       % Point Y initially

%  init mean and covariance (100% confident)
pose           = [x_p y_p phi_p].';
poseCovariance = zeros(3);
predictedPose  = zeros(3, 1);

previous_pose  = [x_p y_p phi_p].';

count = 0;
while 1
    count = count + 1;
    if count > 3000
        break
    end

    %  clear draw every CLEAR_PATH_STEPS steps
    if mod(count, CLEAR_PATH_STEPS) == 0
        fprintf('[INFO] : CLEARING PATH ...\n');
        clf
        axis ([-WATCH_SCOPE WATCH_SCOPE -WATCH_SCOPE WATCH_SCOPE]);
        hold on;
        for i = 1:length(LM_X)
            scatter(LM_X(i), LM_Y(i), [], 'd');
		end
        title('Upgraded EKF LOCALIZATION (green: corrected, red: predicted)')
        xlabel('x axis')
        ylabel('y axis')
    end

    %  Velocity command to make the vehicle Move in Circle
    v = 1.0;
    w = 0.04;

    %  Physical actual output
    % *********************************
    % **** Do not move code below *****
    % *********************************

    pose_real     = VehicleModel(v,w,previous_pose);
    previous_pose = pose_real;

    scatter(pose_real(X), pose_real(Y), 'MarkerEdgeColor',[0 .5 .5], ...
            'MarkerFaceColor',[0 .7 .7]);

    %  Predict Robot Pose with physical model
    G = [
      1 0 -v*cos(pose(THETA))/w + v*cos(pose(THETA) + w*del_t)/w;
      0 1 -v*sin(pose(THETA))/w + v*sin(pose(THETA) + w*del_t)/w;
      0 0 1;
    ];

%     G = [
%       1 0 -v*del_t*sin(pose(THETA) + 0.5*w*del_t);
%       0 1  v*del_t*cos(pose(THETA) + 0.5*w*del_t);
%       0 0 1;
%     ];

    V = [
      (-sin(pose(THETA)) + sin(pose(THETA) + w*del_t))/w  v*(sin(pose(THETA)) - sin(pose(THETA) + w*del_t))/(w^2) + v*cos(pose(THETA) + w*del_t)*del_t/w;
      (cos(pose(THETA)) - cos(pose(THETA) + w*del_t))/w  -v*(cos(pose(THETA)) - cos(pose(THETA) + w*del_t))/(w^2) + v*sin(pose(THETA) + w*del_t)*del_t/w;
      0 del_t;
    ];
    
%     V = [
% 	    cos(pose(THETA) + 0.5*del_t*w) -0.5*sin(pose(THETA) + 0.5*del_t*w);
% 		sin(pose(THETA) + 0.5*del_t*w)  0.5*sin(pose(THETA) + 0.5*del_t*w);
% 		0                               1;
% 	];

    %  input interference Covariance
%     M = [
%     c1*v^2 + c2*w^2 0;
%     0               c3*v^2 + c4*w^2;
%   ];

    M = [
      (c1*abs(v)+c2*abs(w))^2 0;
      0                       (c3*abs(v)+c4*abs(w))^2;
    ];

    predictedPose = pose + [-v*sin(pose(THETA))/w + v*sin(pose(THETA) + w*del_t)/w;
                             v*cos(pose(THETA))/w - v*cos(pose(THETA) + w*del_t)/w;
                             w*del_t];
%     predictedPose = pose + [ v*del_t*cos(pose(THETA) + 0.5*w*del_t);
%                              v*del_t*sin(pose(THETA) + 0.5*w*del_t);
%                              w*del_t];

    poseCovariance = G*poseCovariance*G' + V*M*V';

    plotGaussian(predictedPose(1:2, 1), poseCovariance(1:2, 1:2), 'r');
    if DEBUG
        fprintf('[DEBUG] Real x    : %.2f\n', pose_real(X));
        fprintf('[DEBUG] Real y    : %.2f\n', pose_real(Y));
        fprintf('[DEBUG] Real theta: %.2f\n', pose_real(THETA));
        fprintf('----------\n');
        fprintf('[DEBUG] Predicted x    : %.2f\n', predictedPose(X));
        fprintf('[DEBUG] Predicted y    : %.2f\n', predictedPose(Y));
        fprintf('[DEBUG] Predicted theta: %.2f\n', predictedPose(THETA));
        fprintf('----------\n');
    end
    % *********************************
    % **** Do not move code above *****
    % *********************************

    SenseData = SensorModel(pose_real);
    if DEBUG
        SenseData
    end

    %  Update Robotic Pose with cooperation of sensor data
    if toUpdateWithSensor

    %  for each observation
    numWatchedLandmark = 0;
	
	z   = [];
    H   = [];
    S   = [];
    err = [];
	
	for ob = 1:1:size(SenseData, 1)
		if SenseData(ob, 1) == 0
			continue
		end
		numWatchedLandmark = numWatchedLandmark + 1;
		
		
		xDiff = LM_X(ob) - predictedPose(X);
		yDiff = LM_Y(ob) - predictedPose(Y);
		
		q = xDiff^2 + yDiff^2;
		
		z = [
		    z;
	        sqrt(q);
		    atan2(yDiff, xDiff) - predictedPose(THETA);
		];
		
		H = [
			H;
			-xDiff/sqrt(q)  -yDiff/sqrt(q)    0;
			yDiff/q        -xDiff/q         -1;
		];
		
	    err = [
			err;
			SenseData(ob, 2:3).' - z(end-1:end, 1);
		];
	    
	    %  Restrict err range in -pi ~ pi
		while err(end, 1) > pi | err(end, 1) < -pi
			if err(end, 1) > pi
				err(end, 1) = err(end, 1) - 2*pi;
			elseif err(end, 1) < -pi
				err(end, 1) = err(end, 1) + 2*pi;
			end
		end
	end
	if numWatchedLandmark > 0
		if numWatchedLandmark > 1
			fprintf('[INFO] LUCKY! Updating with %d observation!\n', numWatchedLandmark);
		end
		S = H * poseCovariance * H' + 0.02*eye(numWatchedLandmark*2);
		
		K = poseCovariance * H' * inv(S);
		
		updateVal      = K * err;
		predictedPose  = predictedPose + updateVal;
		poseCovariance = (eye(3) - K * H) * poseCovariance;
		
		if DEBUG
			fprintf('[DEBUG] Error distance    : %.2f\n', err(1));
			fprintf('[DEBUG] Error angle       : %.2f\n', err(2));
			fprintf('----------\n');
			fprintf('[DEBUG] Corrected x    : %.2f\n', predictedPose(X));
			fprintf('[DEBUG] Corrected y    : %.2f\n', predictedPose(Y));
			fprintf('[DEBUG] Corrected theta: %.2f\n', predictedPose(THETA));
			fprintf('----------\n');
		end
		
		plotGaussian(predictedPose(1:2, 1), poseCovariance(1:2, 1:2), 'g');
	end
    end

    pose = predictedPose;

    if DEBUG
        pause(0.05);
    else
        pause(0.01);
    end
end
