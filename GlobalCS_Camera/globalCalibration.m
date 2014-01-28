clear all;
clc;
close all;

global Rhs;
global Lhs;

% IMU 2 chess = chess 2 IMU 
chess2imu = [ -1 0 0 ; 0 1 0; 0 0 -1];

% Loading data from IMU
xsense = load('xSense.mat');
counter = xsense.XsenseWork(:,1);
accData = xsense.XsenseWork(:,3:5);
magData = xsense.XsenseWork(:,13:15);

% Loading positions of the images
Calib_Results

% Names of the rodrigues angles of chessboard positions
omc = {omc_1, omc_2, omc_3, omc_4, omc_5, omc_6, omc_7, omc_8, omc_9, ...
    omc_10, omc_11, omc_12, omc_13, omc_14, omc_15, omc_16, omc_17, omc_18, ...
    omc_19, omc_20, omc_21, omc_22, omc_23, omc_24, omc_25, omc_26, omc_27, ...
    omc_28, omc_29, omc_30, omc_31, omc_32, omc_33, omc_34, omc_35, omc_36, ...
    omc_37, omc_38, omc_39, omc_40, omc_41, omc_42}; 

% For some of them, the chessboard was not properly detected
forbidden = [27, 28, 29, 34, 37];

Rhs = [];
Lhs = [];
for i = 1:42
    
    if ( sum(forbidden == i) == 0 ) % if we have a rotation estimate
        % chess board in global cs
        Rc =  rodrigues(omc{i});
  
        % The indices of the imu data to be avg
        zakres = 1:15 + (i-1)*15;

        % Let's take avg -> '-' because Android's Z is pointed up in the air
        Z = - mean(accData(zakres,:));
        Z = Z / norm(Z); % we want to know the direction
        % Similar -> avg of magnetometer treated as Android's Y
        Y = mean(magData(zakres,:));
        % To have a proper coordinate system, the magnetometer vector has
        % to defined in a plane perpendicular to the gravity vector ->
        % projection onto the plane
        Proj = [Z;Z;Z]; % making magnetic vector perpendicular to gravity
        Y = Y' - Proj*Y'; % removing part in gravity direction
        Y = Y' / norm(Y);
        % The X axis comes from cross product
        X = cross(Y,Z);
        X = X / norm(X);

        % This the rotation of GLOBAL coordinate system in the IMU cs
        Rimu = [X' Y' Z'];

        % Global in camera system -> chess in camera system * imu in chess
        % system * global in imu system
        globalInCamera = Rc * chess2imu * Rimu;
        
        % Solve system of equations: X * (CS of camera) = Global in camera 
        Rhs = [Rhs  globalInCamera];
        Lhs = [Lhs  [1 0 0; 0 1 0; 0 0 1]];
        
        % Lets minimize using Levenberg-Marquardt with initial estimate
        % equal to identity matrix
        % The optimization is done in rodrigues angles to avoid defining
        % additional constaints as it would have to using quaternions or
        % rotation matrices
        X = lsqnonlin(@funkcjaMin, rodrigues(eye(3)) );
        % resulting rotation matrix
        fprintf('Estimate after %d pairs\n', i);
        rodrigues(X)
    end;
end;

fprintf('\n\n\n\t\tFinal estimation:\n\n\n');
% Final reoptimization with parameters allowing to optimize it better
X = lsqnonlin(@funkcjaMin, rodrigues(eye(3)) , [], [], optimset('Algorithm', 'levenberg-marquardt', 'TolFun', 1e-40,'Tolx', 1e-40, 'MaxFunEvals', 2000) );
% resulting rotation matrix
rodrigues(X)

% Saving the estimate in rodrigues 
save('finalEstimateRodrigues.txt','X', '-ascii')
% Saving the estimate in quat
X = dcm2quat(rodrigues(X));
save('finalEstimateQuat.txt','X', '-ascii')
