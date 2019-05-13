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
clc;clear
clear figure

toUpdateWithSensor = true;         %  Switch of executing correction step.

KNOWN_CORRESPONDENCY   = true;     %  (Known | Unkown) correspondency case.
WATCH_SCOPE            = 50;       %  Range of x and y axis, DEFAULT: 35.
CLEAR_PATH_STEPS       = 150;      %  #steps to clear previous path.

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
title('EKF LOCALIZATION (green: corrected, red: predicted)')
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
    if count > 1000
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
        title('EKF LOCALIZATION (green: corrected, red: predicted)')
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
      0 0 1
    ];

    V = [
      (-sin(pose(THETA)) + sin(pose(THETA) + w*del_t))/w  v*(sin(pose(THETA)) - sin(pose(THETA) + w*del_t))/(w^2) + v*cos(pose(THETA) + w*del_t)*del_t/w;
      (cos(pose(THETA)) - cos(pose(THETA) + w*del_t))/w  -v*(cos(pose(THETA)) - cos(pose(THETA) + w*del_t))/(w^2) + v*sin(pose(THETA) + w*del_t)*del_t/w;
      0 del_t
    ];

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

    poseCovariance = G*poseCovariance*G' + V*M*V';

    plotGaussian(predictedPose, poseCovariance, 'r');
    % *********************************
    % **** Do not move code above *****
    % *********************************

    SenseData = SensorModel(pose_real);

    %  Update Robotic Pose with cooperation of sensor data
    if toUpdateWithSensor

    %  for each observation
    numWatchedLandmark = 0;
    for ob = 1:1:size(SenseData, 1)
        if SenseData(ob, 1) == 0
            continue
        end
        numWatchedLandmark = numWatchedLandmark + 1;

        Q = [
          0.02    0.00;
          0.00    0.02;
        ];

        z = zeros(2, LM_c);
        H = zeros(2, 3, LM_c);
        S = zeros(2, 2, LM_c);

        %  probability that which landmark is the observation
        probabilities = zeros(LM_c, 1);

        %  for each landmark : to resolve which landmark we are observing
        for lm = 1:1:LM_c
            xDiff = LM_X(lm) - predictedPose(X);
            yDiff = LM_Y(lm) - predictedPose(Y);

            q = xDiff^2 + yDiff^2;

            z(:, lm) = [sqrt(q);
                        atan2(yDiff, xDiff) - predictedPose(THETA)];

            H(:, :, lm) = [
                -xDiff/sqrt(q)  -yDiff/sqrt(q)    0;
                 yDiff/q        -xDiff/q         -1;
            ];

            S(:, :, lm) = H(:, :, lm) * poseCovariance * H(:, :, lm)'...
                          + Q;

%           probabilities(lm, 1) = mvnpdf(SenseData(ob, 2:3).', ...
%                                         z(:, lm), round(S(:, :, lm), 7));
            probabilities(lm, 1) = det((2*pi)^2*S(:, :, lm))^(-0.5) * ...
                 exp(-.5 * (SenseData(ob, 2:3).' - z(:, lm))' * inv(S(:, :, lm)) *(SenseData(ob, 2:3).' - z(:, lm)));
        end

        [maxLikelihood, idx] = max(probabilities);

        if KNOWN_CORRESPONDENCY | maxLikelihood > MLH_THRESHOLD_LANDMARK
            if ~(ob == idx)
                if KNOWN_CORRESPONDENCY
                    idx = ob;
                else
                    fprintf('[WARN] : CORREDTED WITH WRONG LANDMARK -- true lm: %d, computed: %d\n', ob, idx);
                end
            end

            K = poseCovariance * H(:, :, idx)' * inv(S(:, :, idx));

            updateVal = K*(SenseData(ob, 2:3).' - z(:, idx));
            predictedPose  = predictedPose + updateVal;
            poseCovariance = (eye(3) - K*H(:, :, idx)) * poseCovariance;
        else
            fprintf('[INFO] : NOT CORRECTED FOR AMBIGUITY LANDMARK!\n');
            if ~(ob == idx)
                fprintf('[GOOD] : NOT CORREDTED WITH WRONG LANDMARK\n');
            else
                fprintf('[WARN] : NOT CORREDTED WITH RIGHT LANDMARK\n');
            end
        end
    end

    if numWatchedLandmark > 0
        plotGaussian(predictedPose, poseCovariance, 'g');
    end

    end

    pose = predictedPose;

    pause(0.01);
end
