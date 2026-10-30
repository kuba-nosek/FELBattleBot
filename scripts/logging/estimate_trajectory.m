%% estimate_trajectory.m -- Approximate world-frame trajectory
% This is an exploratory estimate, NOT accurate odometry.
% Assumes robot stays level; signed spin direction known; initial heading,
% position and velocity set below; and accelerometer biases are negligible.
% The translation field represents the SENSOR MIDPOINT, not necessarily CoM.
% Changing CoM/rotation center adds apparent translation acceleration.
clearvars; close all; clc;
csvFile = 'capture.csv';
sensorRadius_m = 0.015;
spinSign = +1;            % +1 counterclockwise, -1 clockwise
initialHeading_rad = 0;  % unknown initial world heading; arbitrary origin
initialVelocity_mps = [0,0];
initialPosition_m = [0,0];
% Only use a correction determined from a separately identified calibration
% interval. Do NOT blindly subtract the average of an entire moving run.
bodyBias_mps2 = [0,0];

T = readtable(csvFile);
required = {'t_us','centripetal_new_mps2','translation_x_mps2','translation_y_mps2'};
assert(all(ismember(required,T.Properties.VariableNames)), ...
    'CSV must include: %s', strjoin(required, ', '));
t = (double(T.t_us) - double(T.t_us(1))) * 1e-6;
assert(numel(t)>=2 && all(diff(t)>0), 'Need increasing timestamps.');
ar = double(T.centripetal_new_mps2);
axB = double(T.translation_x_mps2) - bodyBias_mps2(1);
ayB = double(T.translation_y_mps2) - bodyBias_mps2(2);

omega = spinSign * sqrt(max(ar,0) / sensorRadius_m); % rad/s
heading = initialHeading_rad + cumtrapz(t, omega);

% Rotate robot-frame horizontal acceleration to the stationary world frame.
axW = cos(heading).*axB - sin(heading).*ayB;
ayW = sin(heading).*axB + cos(heading).*ayB;

% Integrate twice; errors grow rapidly from bias, heading drift and noise.
vx = initialVelocity_mps(1) + cumtrapz(t,axW);
vy = initialVelocity_mps(2) + cumtrapz(t,ayW);
x = initialPosition_m(1) + cumtrapz(t,vx);
y = initialPosition_m(2) + cumtrapz(t,vy);

figure('Name','World-frame acceleration and velocity','Color','w');
subplot(2,1,1);
plot(t,axW,'DisplayName','World X'); hold on;
plot(t,ayW,'DisplayName','World Y');
grid on; ylabel('Acceleration [m/s^2]'); legend('Location','best');
subplot(2,1,2);
plot(t,vx,'DisplayName','World Vx'); hold on;
plot(t,vy,'DisplayName','World Vy');
grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('Location','best');

figure('Name','Estimated XY trajectory (approximate)','Color','w');
plot(x,y,'-','LineWidth',1.3); hold on;
scatter(x(1),y(1),45,'filled','DisplayName','Start');
scatter(x(end),y(end),45,'filled','DisplayName','End');
axis equal; grid on; xlabel('World X [m]'); ylabel('World Y [m]');
title('Double-integrated estimated midpoint trajectory (high drift)');
legend('Trajectory','Start','End','Location','best');

fprintf('Recording %.2f s; estimated final displacement: %.3f m\n', ...
        t(end), hypot(x(end)-x(1), y(end)-y(1)));
