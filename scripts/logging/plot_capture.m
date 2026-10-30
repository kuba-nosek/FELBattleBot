%% plot_capture.m -- BattleBot 5-second recording overview
% Put this file beside capture.csv, or edit csvFile below.
clearvars; close all; clc;
csvFile = 'capture.csv';
sensorRadius_m = 0.015;  % Effective radius: half of 30 mm spacing

T = readtable(csvFile);
required = {'t_us','centripetal_old_mps2','centripetal_new_mps2','translation_x_mps2','translation_y_mps2'};
assert(all(ismember(required, T.Properties.VariableNames)), ...
    'CSV must include: %s', strjoin(required, ', '));

t = (double(T.t_us) - double(T.t_us(1))) * 1e-6;  % seconds from start
ar = double(T.centripetal_new_mps2);
ar_old = double(T.centripetal_old_mps2);

tx = double(T.translation_x_mps2);
ty = double(T.translation_y_mps2);

%MATLAB CALCULATIONS
% Centripetal formula returns the magnitude of spin rate, not its sign.
rpm = (60/(2*pi)) * sqrt(max(ar,0) / sensorRadius_m);
sx = (T.accel1_x_mps2 + T.accel2_x_mps2)/2;
sy = (T.accel1_y_mps2 + T.accel2_y_mps2)/2;

radial = (sx + sy)/sqrt(2);
tangential = (sx - sy)/sqrt(2);

old = hypot(sx, sy);
difference = old - radial;

%
figure;
plot(T.t_us/1e6, tangential);
xlabel('Time [s]');
ylabel('Tangential acceleration [m/s²]');
grid on;

%

figure('Name','BattleBot: signals vs time','Color','w');
subplot(5,1,1);
plot(t,ar,'LineWidth',1); grid on;
ylabel('a_r [m/s^2]'); title('FROM ESP - Centripetal acceleration');
subplot(5,1,2);
plot(t,ar_old,'LineWidth',1); grid on;
ylabel('a_r [m/s^2]'); title('FROM ESP - Centripetal acceleration - old calculation');

subplot(5,1,3);
plot(t,tx,'DisplayName','FROM ESP - Translation X'); hold on;
plot(t,ty,'DisplayName','FROM ESP - Translation Y'); grid on;
ylabel('a_{body} [m/s^2]'); legend('Location','best');

subplot(5,1,4);
plot(t,hypot(tx,ty),'LineWidth',1); grid on;
ylabel('|a_{body}| [m/s^2]'); title('Translation acceleration magnitude');
subplot(5,1,5);
plot(t,rpm,'LineWidth',1); grid on;
xlabel('Time [s]'); ylabel('RPM'); title('Estimated absolute RPM');

% XY plot of acceleration *in the rotating robot frame* (NOT position).
figure('Name','Translation acceleration XY','Color','w');
scatter(tx,ty,18,t,'filled');
axis equal; grid on; colorbar;
xlabel('Body X acceleration [m/s^2]');
ylabel('Body Y acceleration [m/s^2]');
title('Translation acceleration vector tips (color = time [s])');

% Raw sensor axes, if selected in LogFields.h.
rawFields = {'accel1_x_mps2','accel1_y_mps2', ...
             'accel2_x_mps2','accel2_y_mps2'};
if all(ismember(rawFields,T.Properties.VariableNames))
    figure('Name','Raw accelerometer channels','Color','w');
    plot(t,double(T.accel1_x_mps2),'DisplayName','A1 X'); hold on;
    plot(t,double(T.accel1_y_mps2),'DisplayName','A1 Y');
    plot(t,double(T.accel2_x_mps2),'DisplayName','A2 X');
    plot(t,double(T.accel2_y_mps2),'DisplayName','A2 Y');
    grid on; xlabel('Time [s]'); ylabel('Specific force [m/s^2]');
    title('Calibrated sensor-local readings'); legend('Location','best');
end

fprintf('Loaded %d samples over %.3f seconds.\n', height(T), t(end));
