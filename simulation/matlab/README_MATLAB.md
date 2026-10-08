# Smart Textile line in MATLAB and Simulink 

Files (keep them in one folder):
| File | What it is |
|---|---|
| `textile_core.m` | The state machine of the cell (states 0 to 9, same times as the PLC, HMI and 3D cells) |
| `textile_line_sim.m` | Animated plan view with live KPIs (rate, takt, defect %), optional video, optional OpenPLC link |
| `build_textile_simulink.m` | Builds the Simulink model `SmartTextile_Line.slx` |
| `preview_plan_view.png` | What the animation looks like |

## 1. MATLAB animation (works with base MATLAB, no toolbox needed)
```matlab
cd 'C:\path\to\this\folder'
res = textile_line_sim;                                   % 300 s, 2x speed, animated
res = textile_line_sim('Duration',600,'Speed',1);         % original speed
res = textile_line_sim('Video','line.mp4');               % records the animation
res = textile_line_sim('Animate',false,'Duration',1800);  % no graphics, just the KPIs (fast)
```
The returned struct `res` holds the KPI time series (`res.rate`, `res.takt`, `res.defectPct`, ...) for your report plots.

Optional: `textile_line_sim('PlcHost','127.0.0.1')` makes the cell follow the real OpenPLC run / fault / E-stop bits
(coils 840 to 842). This needs the Industrial Communication Toolbox and is untested.

## 2. Simulink model
```matlab
build_textile_simulink          % creates and opens SmartTextile_Line.slx
```
Press Run. Double-click the `START_STOP` switch while it runs to hold and release the cell.
You get live displays (good, reject, total, packaged, boxes, defect %, rate, takt), a state scope,
a KPI scope, an XY graph of the sheet's path, and a `kpi` variable in the workspace.
Change the `Speed` constant (1 = original, 2 = twice as fast) before running.

If MATLAB says it could not set the code of the Controller block, open that block and paste the code it prints.

## What the model does
Feed, identify, belt travel and pick, cutting (3 s), finishing (A 5 s, B 8 s), inspection, reject or good bin, packaging.
Every 3rd sheet is the stained type B and is rejected. Edit the tuning values at the top of `textile_core.m`
(for example `P_DETECT` and `P_FALSE` to model a camera that sometimes misses or false-alarms).
