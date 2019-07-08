% John Kath
% Math 178 - Nonlinear Data Analytics
% Summer 2019
% Final Project Code - 7/7/19

%% Track 9-Axis IMU
% This example uses ahrsfilter to fuse 9-axis IMU data 
% from a sensor body that is shaken. Plot the quaternion distance between the 
% object and its final resting position to visualize performance and how quickly 
% the filter converges to the correct resting position. Then tune parameters of 
% the ahrsfilter so that the filter converges more quickly to the ground-truth 
% resting position.
% 
% Visualize the acceleration, 
% magnetic field, and angular velocity as recorded by the sensors.

accelReadings = acc_data;
gyroReadings = gyr_data;
magReadings  = mag_data;
sampleRate = 100;

numSamples = size(accelReadings,1);
time = (0:(numSamples-1))'/sampleRate;

figure(1)
subplot(3,1,1)
p = plot(time,accelReadings);
xlim([min(time) max(time)])
title("Accelerometer Reading - User " + userId + " Session " + sessionNum + " / " + activityLabel) %,'FontSize',12
ylabel('Acceleration (m/s^2)')
legend('x-axis', ...
       'y-axis', ...
       'z-axis')

subplot(3,1,2)
plot(time,magReadings)
xlim([min(time) max(time)])
title('Magnetometer Reading')
ylabel('Magnetic Field (\muT)')

subplot(3,1,3)
plot(time,gyroReadings)
xlim([min(time) max(time)])
title('Gyroscope Reading')
ylabel('Angular Velocity (rad/s)')
xlabel('Time (s)')

print('-dpng','-r300',parentfolder + "/images/" + userId + "_" + sessionNum + "_" + "acc_mag_gyr_data")
% print('-dpng','-r300',parentfolder + "\images\" + userId + "_" + sessionNum + "_" + "acc_mag_gyr_data")
% saveas(gcf,'test.png')

%% 
% Create an ahrsfilter and then fuse the IMU data to determine orientation. 
% The orientation is returned as a vector of quaternions; convert the quaternions 
% to Euler angles in degrees. Visualize the orientation of the sensor body over 
% time by plotting the Euler angles required, at each time step, to rotate the 
% global coordinate system to the sensor body coordinate system.

decim = 1; %2
fuse = ahrsfilter('SampleRate',sampleRate,'DecimationFactor',decim);
orientation = fuse(accelReadings,gyroReadings,magReadings);

orientationEulerAnglesAhrs = eulerd(orientation,'ZYX','frame');

figure(2)
hold on
plot(time,orientationEulerAnglesAhrs(:,1),'Color',p(3).Color) %'Color','#EDB120'
plot(time,orientationEulerAnglesAhrs(:,2),'Color',p(2).Color)
plot(time,orientationEulerAnglesAhrs(:,3),'Color',p(1).Color)
hold off
% plot(time,orientationEulerAngles(:,1),'y', ...
%      time,orientationEulerAngles(:,2),'r', ...
%      time,orientationEulerAngles(:,3),'b')
xlim([min(time) max(time)])
xlabel('Time (s)')
ylabel('Rotation (degrees)')
title("Orientation Over Time / ahrsfilter - User " + userId + " Session " + sessionNum + " / " + activityLabel)
legend('Rotation around z-axis', ...
       'Rotation around y-axis', ...
       'Rotation around x-axis')

print('-dpng','-r300',parentfolder + "/images/" + userId + "_" + sessionNum + "_" + "orientation_ahrsfilter")
% print('-dpng','-r300',parentfolder + "\images\" + userId + "_" + sessionNum + "_" + "acc_mag_gyr_data")
% saveas(gcf,'test2.png')

%% 
% Create an imufilter. Specify a decimation factor of two to reduce the computational 
% cost of the algorithm.
% Pass the accelerometer readings and gyroscope readings to the imufilter 
% object, fuse, to output an estimate of the sensor body orientation over time. 
% By default, the orientation is output as a vector of quaternions.

decim = 1; %2
fuse = imufilter('SampleRate',sampleRate,'DecimationFactor',decim);
orientation = fuse(accelReadings,gyroReadings);

orientationEulerAnglesImu = eulerd(orientation,'ZYX','frame');

%% 
% Orientation is defined by the angular displacement required to rotate a parent 
% coordinate system to a child coordinate system. Plot the orientation in Euler 
% angles in degrees over time.
% 
% imufilter fusion correctly estimates the change in orientation from an assumed 
% north-facing initial orientation. However, the device's _x_-axis was pointing 
% southward when recorded. To correctly estimate the orientation relative to the 
% true initial orientation or relative to NED, use |ahrsfilter|.

figure(3)
hold on
plot(time,orientationEulerAnglesImu(:,1),'Color',p(3).Color) %'Color','#EDB120'
plot(time,orientationEulerAnglesImu(:,2),'Color',p(2).Color)
plot(time,orientationEulerAnglesImu(:,3),'Color',p(1).Color)
hold off
xlim([min(time) max(time)])
xlabel('Time (s)')
ylabel('Rotation (degrees)')
title("Orientation Over Time / imufilter - User " + userId + " Session " + sessionNum + " / " + activityLabel)
legend('Rotation around z-axis', ...
       'Rotation around y-axis', ...
       'Rotation around x-axis')

print('-dpng','-r300',parentfolder + "/images/" + userId + "_" + sessionNum + "_" + "orientation_imufilter")

%% compare filters

figure(4)
hold on
plot(time,orientationEulerAnglesAhrs(:,1),'Color',p(3).Color) %'Color','#EDB120'
plot(time,orientationEulerAnglesAhrs(:,2),'Color',p(2).Color)
plot(time,orientationEulerAnglesAhrs(:,3),'Color',p(1).Color)
plot(time,orientationEulerAnglesImu(:,1),'-.','Color',p(3).Color) %'Color','#EDB120'
plot(time,orientationEulerAnglesImu(:,2),'-.','Color',p(2).Color)
plot(time,orientationEulerAnglesImu(:,3),'-.','Color',p(1).Color)
hold off
xlim([min(time) max(time)])
xlabel('Time (s)')
ylabel('Rotation (degrees)')
title("Orientation Over Time / Compare Filters - User " + userId + " Session " + sessionNum + " / " + activityLabel)
legend('Ahrs z-axis', ...
       'Ahrs y-axis', ...
       'Ahrs x-axis', ...
       ' Imu z-axis', ...
       ' Imu y-axis', ...
       ' Imu x-axis')

print('-dpng','-r300',parentfolder + "/images/" + userId + "_" + sessionNum + "_" + "orientation_compare_filters")

%% 
% In the IMU recording, the shaking stops after approximately six seconds. Determine 
% the resting orientation so that you can characterize how fast the |ahrsfilter| 
% converges.
% 
% To determine the resting orientation, calculate the averages of the magnetic 
% field and acceleration for the final four seconds and then use the |ecompass| 
% function to fuse the data.
% 
% Visualize the quaternion distance from the resting position over time.

% restingOrientation = ecompass(mean(accelReadings(6*SampleRate:end,:)), ...
%                               mean(magReadings(6*SampleRate:end,:)));
% 
% figure(3)
% plot(time,rad2deg(dist(restingOrientation,orientation)))
% hold on
% xlabel('Time (s)','FontSize',12)
% ylabel('Quaternion Distance (degrees)','FontSize',12)

%% 
% Modify the default |ahrsfilter| properties so that the filter converges to 
% gravity more quickly. Increase the |GyroscopeDriftNoise| to |1e-2| and decrease 
% the |LinearAccelerationNoise| to |1e-4|. This instructs the |ahrsfilter| algorithm 
% to weigh gyroscope data less and accelerometer data more. Because the |accelerometer| 
% data provides the stabilizing and consistent gravity vector, the resulting orientation 
% converges more quickly.
% 
% Reset the filter, fuse the data, and plot the results.

% fuse.LinearAccelerationNoise = 1e-4;
% fuse.GyroscopeDriftNoise     = 1e-2;
% reset(fuse)
% 
% orientation = fuse(accelReadings,gyroReadings,magReadings);
% 
% figure(3)
% plot(time,rad2deg(dist(restingOrientation,orientation)))
% legend('Default AHRS Filter','Tuned AHRS Filter','FontSize',12)
