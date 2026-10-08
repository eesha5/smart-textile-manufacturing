function res = textile_line_sim(varargin)
%TEXTILE_LINE_SIM  Animated simulation of the Smart Textile line with live KPIs.
%
%   res = textile_line_sim                      300 s run, animated, 2x speed
%   res = textile_line_sim('Duration',600,'Speed',1,'Realtime',false)
%
%   Name / value options
%     Duration   simulated seconds                        (default 300)
%     Speed      machine speed factor, 1 = original       (default 2)
%     Animate    show the live plan view and KPI plots    (default true)
%     Realtime   pace the animation to the wall clock     (default true)
%     Video      file name, e.g. 'line.mp4' to record it  (default '' = no video)
%     PlcHost    IP of the OpenPLC Runtime, e.g. '127.0.0.1'. Needs the Industrial Communication
%                Toolbox. The cell then runs only while the PLC says run (coils 840 / 841 / 842).
%     PlcPort    Modbus port                              (default 502)
%
%   The state machine is in textile_core.m: feed, identify, belt + pick, cutting, finishing,
%   inspection, reject / good bin, packaging - the same states and times as the PLC, the HMI and
%   the CoppeliaSim cell. Returns a struct with the KPI time series.

o = struct('Duration', 300, 'Speed', 2, 'Animate', true, 'Realtime', true, 'Video', '', ...
           'PlcHost', '', 'PlcPort', 502);
for k = 1:2:numel(varargin)
    o.(varargin{k}) = varargin{k + 1};
end

dt = 0.02;
nSteps = round(o.Duration / dt);
every = 5;                                    % draw every 0.1 s of simulated time
nFrames = floor(nSteps / every);
T = zeros(1, nFrames); RATE = T; TAKT = T; DEF = T; GOOD = T; REJ = T; PACK = T; STATE = T;

% ---------------------------------------------------------------- optional PLC link
plc = [];
if ~isempty(o.PlcHost)
    try
        plc = modbus('tcpip', o.PlcHost, o.PlcPort);
        fprintf('Connected to the PLC at %s:%d\n', o.PlcHost, o.PlcPort);
    catch err
        warning('textile:plc', 'PLC not reachable (%s). The cell is held until it is.', err.message);
    end
end
runFlag = 1;
plcText = 'PLC: not used';
if ~isempty(o.PlcHost), runFlag = 0; plcText = 'PLC: connecting ...'; end

% ---------------------------------------------------------------- figure
if o.Animate
    fig = figure('Name', 'Smart Textile Manufacturing', 'Color', [0.97 0.98 0.99], ...
                 'NumberTitle', 'off', 'Position', [60 60 1280 720]);
    ax = axes('Position', [0.03 0.10 0.60 0.80]); hold(ax, 'on'); axis(ax, 'equal');
    axis(ax, [-2.85 1.3 -1.2 1.3]); set(ax, 'XTick', [], 'YTick', [], 'Box', 'on', 'Color', [0.82 0.85 0.87]);
    title(ax, 'Plan view (robot at the centre)', 'FontSize', 11);
    box_(-2.35, -0.17, 1.70, 0.34, [0.22 0.24 0.28], 'Belt');
    box_(-2.62, -0.15, 0.22, 0.30, [0.30 0.45 0.65], 'Feeder');
    st = {[-0.514 0.613], [0 0.8], [0.514 0.613]}; nm = {'Cutter', 'Press', 'Camera'};
    for i = 1:3, box_(st{i}(1) - 0.15, st{i}(2) - 0.13, 0.30, 0.26, [0.28 0.33 0.40], nm{i}); end
    box_(0.8 - 0.16, -0.14, 0.32, 0.28, [0.25 0.60 0.30], 'Good bin');
    box_(0.566 - 0.15, -0.566 - 0.12, 0.30, 0.24, [0.70 0.55 0.30], 'Packaging');
    box_(-0.566 - 0.16, -0.566 - 0.14, 0.32, 0.28, [0.70 0.22 0.20], 'Reject bin');
    hCut = rectangle('Position', [-0.514 - 0.11, 0.613 + 0.17, 0.22, 0.02], 'FaceColor', [0.85 0.85 0.9], 'EdgeColor', 'k');
    hPress = rectangle('Position', [0 - 0.10, 0.8 + 0.17, 0.20, 0.02], 'FaceColor', [0.85 0.4 0.3], 'EdgeColor', 'k');
    rectangle('Position', [-0.10 -0.10 0.20 0.20], 'Curvature', [1 1], 'FaceColor', [0.15 0.30 0.55], 'EdgeColor', 'k');
    hArm = line([0 0], [0 0.38], 'LineWidth', 7, 'Color', [0.95 0.55 0.10]);
    hSheet = patch(0, 0, [0.95 0.90 0.72], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 1.2);
    hSt1 = patch(0, 0, [0.22 0.10 0.04], 'EdgeColor', 'none');
    hSt2 = patch(0, 0, [0.22 0.10 0.04], 'EdgeColor', 'none');
    hGrip = plot(0, 0.38, 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.2 0.2 0.22], 'MarkerEdgeColor', 'k');
    hState = text(-2.8, 1.18, '', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.35 0.15]);
    hPlc = text(-2.8, 1.06, plcText, 'FontSize', 10, 'Color', [0.15 0.3 0.6]);

    axk = axes('Position', [0.66 0.55 0.31 0.35], 'Visible', 'off');
    hKpi = text(0, 1, '', 'Parent', axk, 'VerticalAlignment', 'top', 'FontSize', 12, 'FontName', 'FixedWidth', 'Interpreter', 'none');
    set(axk, 'XLim', [0 1], 'YLim', [0 1]);
    a1 = axes('Position', [0.70 0.38 0.27 0.10]); hR = plot(a1, 0, 0, 'Color', [0.1 0.4 0.8], 'LineWidth', 1.5); ylabel(a1, 'products/min'); xlim(a1, [0 o.Duration]); grid(a1, 'on');
    a2 = axes('Position', [0.70 0.22 0.27 0.10]); hT = plot(a2, 0, 0, 'Color', [0.85 0.45 0.1], 'LineWidth', 1.5); ylabel(a2, 'takt [s]'); xlim(a2, [0 o.Duration]); grid(a2, 'on');
    a3 = axes('Position', [0.70 0.06 0.27 0.10]); hD = plot(a3, 0, 0, 'Color', [0.75 0.2 0.2], 'LineWidth', 1.5); ylabel(a3, 'defect %'); xlabel(a3, 'time [s]'); xlim(a3, [0 o.Duration]); ylim(a3, [0 60]); grid(a3, 'on');
    tip = [0 0.38];
    vw = [];
    if ~isempty(o.Video)
        vw = VideoWriter(o.Video, 'MPEG-4'); vw.FrameRate = 10; open(vw);
    end
end

names = {'FEED', 'IDENTIFY', 'BELT / PICK', 'PROCESS 1 - CUTTING', 'PROCESS 2 - FINISHING', ...
         'INSPECTION (OpenCV)', 'REJECT', 'GOOD BIN', '-', 'PACKAGING'};

% ---------------------------------------------------------------- simulation loop
textile_core(1, dt, 1, 1);                    % reset
f = 0; tic;
for k = 1:nSteps
    simT = k * dt;
    if ~isempty(o.PlcHost) && mod(k, every) == 0
        try
            c = read(plc, 'coils', 841, 3);   % Modbus coils 840, 841, 842 (the toolbox counts from 1)
            runFlag = double(c(1) == 1 && c(2) == 0 && c(3) == 0);
            if c(3) == 1
                plcText = 'PLC: EMERGENCY STOP';
            elseif c(2) == 1
                plcText = 'PLC: FAULT';
            elseif c(1) == 1
                plcText = 'PLC: RUN';
            else
                plcText = 'PLC: STOPPED';
            end
        catch
            runFlag = 0; plcText = 'PLC: LINK LOST - cell held';
        end
    end
    out = textile_core(runFlag, dt, o.Speed, 0);

    if mod(k, every) == 0
        f = f + 1;
        T(f) = simT; STATE(f) = out(1); GOOD(f) = out(2); REJ(f) = out(3); PACK(f) = out(5);
        DEF(f) = out(7); RATE(f) = out(8); TAKT(f) = out(9);
        if o.Animate
            px = out(10); py = out(11); isB = out(16) ~= 0 && out(1) >= 1;
            % the arm follows the sheet while it works on it, otherwise it waits at the belt end
            if out(13) ~= 0 || any(out(1) == [3 4 5 6 7 9]), target = [px py]; else, target = [-0.85 0]; end
            tip = tip + 0.35 * (target - tip);
            set(hArm, 'XData', [0 tip(1)], 'YData', [0 tip(2)]); set(hGrip, 'XData', tip(1), 'YData', tip(2));
            hw = 0.11; hh = 0.08;
            set(hSheet, 'XData', px + [-hw hw hw -hw], 'YData', py + [-hh -hh hh hh]);
            if isB
                set(hSt1, 'XData', px + 0.05 + [-0.0225 0.0225 0.0225 -0.0225], 'YData', py + 0.03 + [-0.0175 -0.0175 0.0175 0.0175], 'Visible', 'on');
                set(hSt2, 'XData', px - 0.06 + [-0.02 0.02 0.02 -0.02], 'YData', py - 0.025 + [-0.015 -0.015 0.015 0.015], 'Visible', 'on');
            else
                set(hSt1, 'Visible', 'off'); set(hSt2, 'Visible', 'off');
            end
            set(hCut, 'Position', [-0.514 - 0.11, 0.613 + 0.17 - 0.12 * out(14), 0.22, 0.02]);
            set(hPress, 'Position', [0 - 0.10, 0.8 + 0.17 - 0.12 * out(15), 0.20, 0.02]);
            set(hState, 'String', sprintf('State %d: %s', out(1), names{out(1) + 1}));
            set(hPlc, 'String', plcText);
            set(hKpi, 'String', sprintf(['GOOD       %4d\nREJECT     %4d\nTOTAL      %4d\nPACKAGED   %4d\nBOXES      %4d\n' ...
                                         'DEFECT     %5.1f %%\nRATE       %5.2f /min\nTAKT       %5.1f s'], ...
                                        out(2), out(3), out(4), out(5), out(6), out(7), out(8), out(9)));
            set(hR, 'XData', T(1:f), 'YData', RATE(1:f)); set(hT, 'XData', T(1:f), 'YData', TAKT(1:f)); set(hD, 'XData', T(1:f), 'YData', DEF(1:f));
            drawnow;
            if ~isempty(vw), writeVideo(vw, getframe(fig)); end
            if o.Realtime, pause(max(0, simT - toc)); end
        end
    end
end
if o.Animate && ~isempty(vw), close(vw); end

res = struct('time', T(1:f), 'rate', RATE(1:f), 'takt', TAKT(1:f), 'defectPct', DEF(1:f), ...
             'good', GOOD(1:f), 'reject', REJ(1:f), 'packaged', PACK(1:f), 'state', STATE(1:f));
fprintf('\nDone: %d good, %d rejected, %d packaged | defect %.1f %% | rate %.2f /min | takt %.1f s\n', ...
        out(2), out(3), out(5), out(7), out(8), out(9));

    function box_(x, y, w, h, c, label)
        rectangle('Position', [x y w h], 'FaceColor', c, 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 1);
        text(x + w / 2, y - 0.07, label, 'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.1 0.1 0.15]);
    end
end
