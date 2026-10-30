clear; clc; close all;

% =========================
% SETTINGS
% =========================
csvFile = 'capture.csv';
saveGif = true;
gifFile = 'translation_animation.gif';
gifDelay = 0.04;   % 25 fps approximately
step = 5;          % use 2, 4, ... to skip frames if needed

% =========================
% LOAD DATA
% =========================
T = readtable(csvFile);

t = T.t_us * 1e-6;
tx = T.translation_x_mps2;
ty = T.translation_y_mps2;

% Optional: axis limits
maxVal = max(sqrt(tx.^2 + ty.^2));
axisLimit = maxVal * 1.2;
if axisLimit == 0
    axisLimit = 1;
end

% =========================
% FIGURE
% =========================
fig = figure('Color', 'w');
axis equal;
grid on;
xlim([-axisLimit axisLimit]);
ylim([-axisLimit axisLimit]);
xlabel('Translation X [m/s^2]');
ylabel('Translation Y [m/s^2]');
title('Translation acceleration vector');
hold on;

% Arrow handle
h = quiver(0, 0, 0, 0, 0, 'LineWidth', 2, 'MaxHeadSize', 0.5);
trail = plot(0, 0, 'b-');

xHist = [];
yHist = [];

% =========================
% ANIMATION LOOP
% =========================
for k = 1:step:length(t)
    xHist(end+1) = tx(k); %#ok<SAGROW>
    yHist(end+1) = ty(k); %#ok<SAGROW>

    set(h, 'UData', tx(k), 'VData', ty(k));
    set(trail, 'XData', xHist, 'YData', yHist);

    title(sprintf('Translation acceleration vector, t = %.3f s', t(k)));

    drawnow;

    if saveGif
        frame = getframe(fig);
        img = frame2im(frame);
        [A, map] = rgb2ind(img, 256);

        if k == 1
            imwrite(A, map, gifFile, 'gif', ...
                'LoopCount', Inf, ...
                'DelayTime', gifDelay);
        else
            imwrite(A, map, gifFile, 'gif', ...
                'WriteMode', 'append', ...
                'DelayTime', gifDelay);
        end
    end
end

disp(['Saved GIF: ' gifFile]);