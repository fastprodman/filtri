%%
% Load sensor data and true orientations
ld = load('CircularTrajectorySensorData.mat');

Fs = ld.Fs; % maximum MARG rate
trajOrient = ld.trajData.Orientation;
trajAngVel = ld.trajData.AngularVelocity;

accel = ld.accel;
gyro = ld.gyro;
mag = ld.mag;

% Initialize the AHRS filter
ahrs = ahrsfilter('SampleRate', Fs);

% Initialize variables for orientation estimates and errors
numSamples = size(accel, 1);
estimatedOrientations = zeros(numSamples, 4); % For storing estimated quaternions
orientationErrors = zeros(numSamples, 1); % For storing orientation errors in degrees
angularVelocityErrors = zeros(numSamples, 3); % For storing angular velocity errors

% Start timing the calculations
tic;

% Process sensor data through ahrsfilter and calculate errors
for i = 1:numSamples
    % Update AHRS filter with current measurements
    estimatedOrientation = ahrs(accel(i,:), gyro(i,:), mag(i,:));
    
    % Convert the estimated orientation quaternion to a 4-element vector
    estimatedOrientations(i,:) = compact(estimatedOrientation);
    
    % Calculate angular velocity error (true - estimated)
    angularVelocityErrors(i,:) = trajAngVel(i,:) - gyro(i,:);
    
    % Compute orientation error
    % Convert both estimated and true orientations to rotation matrices
    estimatedRotm = quat2rotm(estimatedOrientations(i,:));
    trueRotm = quat2rotm(trajOrient(i,:)); % trajOrient contains true orientations
    % Calculate orientation error as the angle between true and estimated rotation matrices
    rotmError = estimatedRotm' * trueRotm;
    orientationErrors(i) = rad2deg(acos((trace(rotmError) - 1) / 2));
end

% End timing the calculations
elapsedTime = toc;

% Calculate MAE and RMSE for Orientation and Angular Velocity
maeOrientation = mean(abs(orientationErrors));
rmseOrientation = sqrt(mean(orientationErrors.^2));

maeAngularVelocity = mean(sqrt(sum(angularVelocityErrors.^2, 2)));
rmseAngularVelocity = sqrt(mean(sum(angularVelocityErrors.^2, 2)));

% Display the results
disp('Orientation Error:');
disp(['Mean Absolute Error (MAE): ', num2str(maeOrientation), ' degrees']);
disp(['Root Mean Square Error (RMSE): ', num2str(rmseOrientation), ' degrees']);

disp('Angular Velocity Error:');
disp(['Mean Absolute Error (MAE) for Angular Velocity (rad/s): ', num2str(maeAngularVelocity)]);
disp(['Root Mean Square Error (RMSE) for Angular Velocity (rad/s): ', num2str(rmseAngularVelocity)]);

% Display elapsed time
disp(['Time required for calculations: ', num2str(elapsedTime), ' seconds']);
