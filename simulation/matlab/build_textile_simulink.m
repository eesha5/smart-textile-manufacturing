function build_textile_simulink(runAfter)
%BUILD_TEXTILE_SIMULINK  Creates and opens the Simulink model SmartTextile_Line.slx.
%
%   build_textile_simulink          builds the model (in the current folder) and opens it
%   build_textile_simulink(true)    builds it and also runs it for 300 s
%
%   The model contains the cell's state machine (textile_core.m, inside a MATLAB Function block),
%   a START / STOP switch that stands in for the PLC run signal, a speed constant, live KPI displays,
%   scopes for the state and the KPIs, and an XY graph that traces the path of the sheet
%   (feed -> belt -> cutter -> press -> camera -> bins -> packaging).
%   textile_core.m must be on the MATLAB path (keep both files in the same folder).
%
%   Click the START / STOP switch while the model runs to hold and release the cell.

if nargin < 1, runAfter = false; end
mdl = 'SmartTextile_Line';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.02', 'StopTime', '300');

% ------------------------------------------------------------------ inputs: START / STOP switch and speed
add_block('simulink/Sources/Constant', [mdl '/Run'],  'Value', '1', 'Position', [60 80 110 110]);
add_block('simulink/Sources/Constant', [mdl '/Hold'], 'Value', '0', 'Position', [60 140 110 170]);
add_block('simulink/Signal Routing/Manual Switch', [mdl '/START_STOP'], 'Position', [170 100 215 150]);
add_block('simulink/Sources/Constant', [mdl '/Speed'], 'Value', '2', 'Position', [60 230 110 260]);
add_line(mdl, 'Run/1',  'START_STOP/1', 'autorouting', 'on');
add_line(mdl, 'Hold/1', 'START_STOP/2', 'autorouting', 'on');
annotate(mdl, 'Double-click START_STOP while the model runs: up = RUN, down = HOLD (the PLC run / stop / E-stop)', [40 30 560 60]);
annotate(mdl, 'Speed: 1 = original speed, 2 = twice as fast', [40 275 360 300]);

% ------------------------------------------------------------------ the cell (state machine) in a MATLAB Function block
blk = [mdl '/Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', [300 80 470 520]);
code = sprintf(['function [state, good, reject, total, packaged, boxes, defectPct, rate, takt, posX, posY, posZ, carried, cutDepth, pressDepth, sheetIsB] = fcn(run, speed)\n' ...
    '%%#codegen\n' ...
    'o = textile_core(run, 0.02, speed, 0);\n' ...
    'state = o(1); good = o(2); reject = o(3); total = o(4); packaged = o(5); boxes = o(6); defectPct = o(7); rate = o(8); takt = o(9);\n' ...
    'posX = o(10); posY = o(11); posZ = o(12); carried = o(13); cutDepth = o(14); pressDepth = o(15); sheetIsB = o(16);\n' ...
    'end\n']);
setScript(mdl, blk, code);
add_line(mdl, 'START_STOP/1', 'Controller/1', 'autorouting', 'on');
add_line(mdl, 'Speed/1', 'Controller/2', 'autorouting', 'on');

% ------------------------------------------------------------------ KPI displays (output ports 2..9)
labels = {'GOOD', 'REJECT', 'TOTAL', 'PACKAGED', 'BOXES', 'DEFECT_pct', 'RATE_per_min', 'TAKT_s'};
for i = 1:8
    name = ['Disp_' labels{i}];
    y = 60 + (i - 1) * 55;
    add_block('simulink/Sinks/Display', [mdl '/' name], 'Position', [640 y 740 y + 40]);
    add_line(mdl, sprintf('Controller/%d', i + 1), [name '/1'], 'autorouting', 'on');
end
annotate(mdl, 'LIVE KPIs: GOOD, REJECT, TOTAL, PACKAGED, BOXES, DEFECT %, RATE (products per min), TAKT (s)', [600 20 1000 45]);

% ------------------------------------------------------------------ scopes
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_State'], 'Inputs', '2', 'Position', [560 540 565 600]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_State'], 'Position', [640 540 700 600]);
add_line(mdl, 'Controller/1',  'Mux_State/1', 'autorouting', 'on');
add_line(mdl, 'Controller/13', 'Mux_State/2', 'autorouting', 'on');
add_line(mdl, 'Mux_State/1', 'Scope_State/1', 'autorouting', 'on');
annotate(mdl, 'State (0 feed, 1 identify, 2 belt and pick, 3 cut, 4 finish, 5 inspect, 6 reject, 7 good, 9 pack) and carried', [520 610 900 640]);

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_KPI'], 'Inputs', '3', 'Position', [800 400 805 470]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_KPIs'], 'Position', [880 400 940 470]);
add_line(mdl, 'Controller/8', 'Mux_KPI/1', 'autorouting', 'on');   % rate
add_line(mdl, 'Controller/9', 'Mux_KPI/2', 'autorouting', 'on');   % takt
add_line(mdl, 'Controller/7', 'Mux_KPI/3', 'autorouting', 'on');   % defect %
add_line(mdl, 'Mux_KPI/1', 'Scope_KPIs/1', 'autorouting', 'on');
annotate(mdl, 'KPI trends: rate, takt, defect %', [800 480 1000 505]);
add_block('simulink/Sinks/To Workspace', [mdl '/To_Workspace'], 'VariableName', 'kpi', 'SaveFormat', 'Timeseries', 'Position', [1000 400 1080 430]);
add_line(mdl, 'Mux_KPI/1', 'To_Workspace/1', 'autorouting', 'on');

% ------------------------------------------------------------------ path of the sheet (XY graph)
try
    add_block('simulink/Sinks/XY Graph', [mdl '/Sheet_Path_XY'], 'Position', [880 540 960 620], ...
              'xmin', '-2.4', 'xmax', '1', 'ymin', '-1', 'ymax', '1', 'st', '-1');
    add_line(mdl, 'Controller/10', 'Sheet_Path_XY/1', 'autorouting', 'on');
    add_line(mdl, 'Controller/11', 'Sheet_Path_XY/2', 'autorouting', 'on');
    annotate(mdl, 'Path of the sheet in the plan view (x, y in metres; robot base at 0,0)', [860 630 1180 660]);
catch err
    disp(['XY Graph block not added: ' err.message]);
    add_block('simulink/Sinks/Terminator', [mdl '/T_posX'], 'Position', [880 560 900 580]);
    add_block('simulink/Sinks/Terminator', [mdl '/T_posY'], 'Position', [880 600 900 620]);
    add_line(mdl, 'Controller/10', 'T_posX/1', 'autorouting', 'on');
    add_line(mdl, 'Controller/11', 'T_posY/1', 'autorouting', 'on');
end

% ------------------------------------------------------------------ outputs not shown: terminate cleanly
extra = [12 14 15 16];                          % posZ, cutDepth, pressDepth, sheetIsB
for i = 1:numel(extra)
    nm = sprintf('Term_%d', extra(i));
    add_block('simulink/Sinks/Terminator', [mdl '/' nm], 'Position', [520 700 + i * 25 540 715 + i * 25]);
    add_line(mdl, sprintf('Controller/%d', extra(i)), [nm '/1'], 'autorouting', 'on');
end

annotate(mdl, 'Smart Textile Manufacturing: feed, cut, finish, inspect, sort, package', [40 -10 700 15]);

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Created %s.slx in %s\n', mdl, pwd);
if runAfter
    sim(mdl);
end
end

% ---------------------------------------------------------------------------------------------
function annotate(mdl, text, pos)
% notes on the diagram; never let a cosmetic problem stop the build
try
    text = strrep(text, '/', ' ');
    a = Simulink.Annotation([mdl '/' text]);
    a.Position = pos;
catch
end
end

function setScript(mdl, blk, code)
try
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
    chart.Script = code;
catch err
    warning('textile:script', '%s', ['Could not set the MATLAB Function code automatically (' err.message ...
        '). Open the Controller block and paste this:' char(10) char(10) code]);
end
end
