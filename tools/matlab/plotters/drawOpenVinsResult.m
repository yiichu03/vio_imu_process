function drawOpenVinsResult(resultdir)
% draw openvins result, one figure for one physical entity, e.g., p_BC.
motions = {'Lemniscate'; 'LineSegment'; 'Circle'};
runnos = [1, 2, 1];
addpath('/media/jhuai/Seagate2TB/jhuai/tools/export_fig');
est = readmatrix([resultdir, '/', motions{1}, '/state_estimate_0.txt'], 'NumHeaderLines', 1);

startstd = size(est, 2);
extrinsicp = 32:34;
extrinsicq = 28:31;
extrinsicpstd = (30:32) + startstd;
extrinsicqstd = (27:29) + startstd;
td = 18;
tdstd = 17 + startstd;

ba = 15:17;
bastd = (14:16) + startstd;
bg = 12:14;
bgstd = (11:13) + startstd;
q = 2:5;
qstd = (2:4) + startstd;
p = 6:8;
pstd = (5:7) + startstd;
v = 9:11;
vstd = (8:10) + startstd;

axisLabels = {'x', 'y', 'z'};
estLineStyles = {'r', 'b', 'k'};
stdLineStyles = {'r--', 'b--', 'k--'};

errordatalist = cell(3, 1);
for i = 1:length(runnos)
    errordatalist{i} = loadOpenVinsResult([resultdir, '/', motions{i}], runnos(i));
end

close all;
line_handles = zeros(6, 1);
dimen = 3;
legendLabels = cell(dimen * 2, 1);
for i = 1:dimen
    legendLabels{i} = motions{i};
    legendLabels{dimen + i} = ['3\sigma ', motions{i}];
end

lw = 1;
for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, q(j),  qstd(j), 180/pi, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
    if j == 1
    legend(line_handles(1:dimen*2), legendLabels);
    end
    xlabel('t (sec)');
    ylabel(['$\delta \theta_{WB, ', axisLabels{j}, '} (^\circ)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_theta_WB_', axisLabels{j}, '.pdf'], 'none');
end

for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, p(j),  pstd(j), 1, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
%     legend(line_handles(1:dimen*2), legendLabels);
%     xlabel('t (sec)');
    ylabel(['$\delta \mathbf{p}_{WB, ', axisLabels{j}, '} (m)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_p_WB_', axisLabels{j}, '.pdf'], 'none');
end

for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, v(j),  vstd(j), 1, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
%     legend(line_handles(1:dimen*2), legendLabels);
%     xlabel('t (sec)');
    ylabel(['$\delta \mathbf{v}_{WB, ', axisLabels{j}, '} (m/s)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_v_WB_', axisLabels{j}, '.pdf'], 'none');
end


for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, bg(j),  bgstd(j), 180 / pi, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
    if j == 1
    legend(line_handles(1:dimen*2), legendLabels);
    end
    xlabel('t (sec)');
    ylabel(['$\delta \mathbf{b}_{g, ', axisLabels{j}, '} (^\circ/s)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_bg_', axisLabels{j}, '.pdf'], 'none');
end

for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, ba(j),  bastd(j), 1, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
%     legend(line_handles(1:dimen*2), legendLabels);
%     xlabel('t (sec)');
    ylabel(['$\delta \mathbf{b}_{a, ', axisLabels{j}, '} (m/s^2)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_ba_', axisLabels{j}, '.pdf'], 'none');
end

for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, extrinsicq(j),  extrinsicqstd(j), 180 / pi, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
%     legend(line_handles(1:dimen*2), legendLabels);
%     xlabel('t (sec)');
    ylabel(['$\delta \mathbf{\theta}_{BC, ', axisLabels{j}, '} (^\circ)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_theta_BC_', axisLabels{j}, '.pdf'], 'none');
end

for j = 1:3
    figure;
    for i=1:length(runnos)
        handles = drawOneDimAndStd(errordatalist{i}, extrinsicp(j),  extrinsicpstd(j), 100, estLineStyles{i}, stdLineStyles{i}, lw);
        line_handles(i) = handles(1);
        line_handles(dimen + i) = handles(2);
    end
    grid on;
%     legend(line_handles(1:dimen*2), legendLabels);
%     xlabel('t (sec)');
    ylabel(['$\delta \mathbf{p}_{BC, ', axisLabels{j}, '} (cm)$'], 'Interpreter', 'latex');
    saveFigure([resultdir, '/', 'delta_p_BC_', axisLabels{j}, '.pdf'], 'none', 18);
end

figure;
for i=1:length(runnos)
    handles = drawOneDimAndStd(errordatalist{i}, td,  tdstd, 1000, estLineStyles{i}, stdLineStyles{i}, lw);
    line_handles(i) = handles(1);
    line_handles(dimen + i) = handles(2);
end
grid on;
% legend(line_handles(1:dimen*2), legendLabels);
% xlabel('t (sec)');
ylabel(['$\delta t_{d, ', axisLabels{j}, '} (ms)$'], 'Interpreter', 'latex');
saveFigure([resultdir, '/', 'delta_td.pdf'], 'none', 18);
end

function line_handles = drawOneDimAndStd(errordata, meanIndex, stdIndex, scale, estLineStyle, stdLineStyle, lineWidth)
line_handles = zeros(2, 1);
        line_handles(1) = plot(errordata(:, 1), ...
            errordata(:, meanIndex)*scale, estLineStyle, 'LineWidth', lineWidth); hold on;
        line_handles(2) = plot(errordata(:, 1), ...
            (3*errordata(:, stdIndex) + ...
            errordata(:, meanIndex))*scale, stdLineStyle, 'LineWidth', lineWidth);
        plot(errordata(:,1), ...
            (-3*errordata(:,stdIndex) + ...
            errordata(:, meanIndex))*scale, stdLineStyle, 'LineWidth', lineWidth);
end

function saveFigure(outputfig, backgroundColor, fontsize)
if nargin == 2
    fontsize = 16;
end
set(gcf, 'Color', backgroundColor);
ax = gca;
ax.FontSize = fontsize;
if exist(outputfig, 'file')==2
  delete(outputfig);
end
export_fig(outputfig);
end
