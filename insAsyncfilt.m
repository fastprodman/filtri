%%
ld = load('CircularTrajectorySensorData.mat');

Fs = ld.Fs; % maximum MARG rate
gpsFs = ld.gpsFs; % maximum GPS rate
refloc = ld.refloc; 

trajOrient = ld.trajData.Orientation;
trajVel = ld.trajData.Velocity;
trajPos = ld.trajData.Position;
trajAcc = ld.trajData.Acceleration;
trajAngVel = ld.trajData.AngularVelocity;

accel = ld.accel;
gyro = ld.gyro;
mag = ld.mag;
lla = ld.lla;
gpsvel = ld.gpsvel;

fusionfilt = insfilterAsync('ReferenceLocation', refloc);

Nav = 100;
initstate = zeros(28,1);
initstate(1:4) = compact(meanrot(trajOrient(1:Nav))); 
initstate(5:7) = mean(trajAngVel(1:Nav,:), 1);
initstate(8:10) = mean(trajPos(1:Nav,:), 1);
initstate(11:13) = mean(trajVel(1:Nav,:), 1);
initstate(14:16) = mean(trajAcc(1:Nav,:), 1);
initstate(23:25) = ld.magField;

initstate(20:22) = deg2rad([3.125 3.125 3.125]); 
fusionfilt.State = initstate;

fusionfilt.QuaternionNoise = 1e-2; 
fusionfilt.AngularVelocityNoise = 100;
fusionfilt.AccelerationNoise = 100;
fusionfilt.MagnetometerBiasNoise = 1e-7;
fusionfilt.AccelerometerBiasNoise = 1e-7;
fusionfilt.GyroscopeBiasNoise = 1e-7;

Rmag = 0.4;
Rvel = 0.01;
Racc = 610;
Rgyro = 0.76e-5;
Rpos = 3.4; 

fusionfilt.StateCovariance = diag(1e-3*ones(28,1));

% Initialize arrays to store combined position errors and orientation errors
posErrors = [];
orientErrors = [];

% Start timing the calculations
tic;

for ii = 1:size(accel,1)
    fusionfilt.predict(1./Fs);
    
    % Fuse Accelerometer
    fusionfilt.fuseaccel(accel(ii,:), Racc);
    
    % Fuse Gyroscope
    fusionfilt.fusegyro(gyro(ii,:), Rgyro);
        
    % Fuse Magnetometer
    fusionfilt.fusemag(mag(ii,:), Rmag);
    
    % Fuse GPS
    if mod(ii, fix(Fs/gpsFs)) == 0
        fusionfilt.fusegps(lla(ii,:), Rpos, gpsvel(ii,:), Rvel);
    end

    % Compute the pose error
    [p, q] = pose(fusionfilt);
    posErr = norm(p - trajPos(ii,:));
    orientErr = rad2deg(dist(q, trajOrient(ii)));

    % Store combined position error and orientation error
    posErrors = [posErrors; posErr];
    orientErrors = [orientErrors; orientErr];
end

% End timing the calculations
elapsedTime = toc;

% Compute MAE and RMSE for combined position error and orientation error
maePos = mean(posErrors);
rmsePos = sqrt(mean(posErrors.^2));
maeOrient = mean(orientErrors);
rmseOrient = sqrt(mean(orientErrors.^2));

% Display results
fprintf('Combined Position MAE: %f meters, RMSE: %f meters\n', maePos, rmsePos);
fprintf('Orientation Error MAE: %f degrees, RMSE: %f degrees\n', maeOrient, rmseOrient);

% Display elapsed time
fprintf('Time required for calculations: %f seconds\n', elapsedTime);

