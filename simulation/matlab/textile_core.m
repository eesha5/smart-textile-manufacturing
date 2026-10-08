function out = textile_core(run, dt, speed, reset)
%TEXTILE_CORE  State machine of the Smart Textile cell (states 0-9), one call = one time step.
%
%   out = textile_core(run, dt, speed, reset)
%     run    1 = the PLC lets the cell run, 0 = hold (START / STOP / E-STOP from the OpenPLC program)
%     dt     simulation step in seconds
%     speed  1 = original speed, 2 = twice as fast (same idea as SPEED_FACTOR in the CoppeliaSim controller)
%     reset  non-zero clears the state and all counters
%
%   out is a 1x16 row:
%     [ state good reject total packaged boxes defectPct rate takt posX posY posZ carried cutDepth pressDepth sheetIsB ]
%       state  0 feed | 1 identify | 2 belt + pick | 3 cutting | 4 finishing | 5 inspection | 6 reject | 7 good bin | 9 packaging
%       rate   products per minute      takt  seconds between sorted products
%       posX/Y/Z  position of the sheet (metres, robot base = origin)     carried  1 = the robot holds the sheet
%       cutDepth / pressDepth  0..1 how far the cutter blade / press head is down      sheetIsB  1 = stained type B
%
%   Written with scalars and persistent variables only, so it also works inside a Simulink MATLAB Function block.

% ---- tuning (same values as the CoppeliaSim controller and the PLC) ----
DEFECT_EVERY_N = 3;      % every 3rd sheet is type B
CONV_T = 3.0;  CUT_T = 3.0;  FIN_A_T = 5.0;  FIN_B_T = 8.0;  SORT_T = 3.0;
ID_DWELL = 0.5;          % identification dwell (PLC timer T8)
INSP_MOVE_T = 1.3;  INSP_READ_T = 2.2;
PACK_LEVELS = 4;
P_DETECT = 1.0;          % chance the camera flags a stained sheet (1 = never misses)
P_FALSE  = 0.0;          % chance the camera flags a clean sheet by mistake

% ---- station positions (same layout as the 3D cells) ----
R = 0.80;
FEED  = [-2.15 0 0.21];  PICK = [-0.85 0 0.21];
CUT   = [R*cosd(130) R*sind(130) 0.21];
PRESS = [R*cosd(90)  R*sind(90)  0.21];
INSP  = [R*cosd(50)  R*sind(50)  0.21];
GOODP = [R 0 0.092];
REJP  = [R*cosd(-135) R*sind(-135) 0.092];
PACKP = [R*cosd(-45)  R*sind(-45)  0.052];

persistent st t cyc isB good rej tot pack simT lastSort takt pos carried cutD pressD inited
if isempty(inited) || reset ~= 0
    st = 0; t = 0; cyc = 0; isB = 0; good = 0; rej = 0; tot = 0; pack = 0; simT = 0;
    lastSort = 0; takt = 0; pos = FEED; carried = 0; cutD = 0; pressD = 0; inited = 1;
end

if run ~= 0
    simT = simT + dt;
    t = t + dt * speed;

    if st == 0                                   % FEED
        cyc = cyc + 1;
        isB = double(mod(cyc, DEFECT_EVERY_N) == 0);
        pos = FEED; carried = 0; t = 0; st = 1;
    end
    if st == 1                                   % IDENTIFICATION
        if t > ID_DWELL
            t = 0; st = 2;
        end
    end
    if st == 2                                   % BELT TRAVEL, THEN PICK
        if t < CONV_T
            pos = FEED + (PICK - FEED) * (t / CONV_T);
        else
            pos = PICK;
            if t > CONV_T + 1.2
                carried = 1; t = 0; st = 3;
            end
        end
    end
    if st == 3                                   % PROCESS 1: cutting
        if t < 0.8
            pos = arcmove(PICK, CUT, t / 0.8, 0.10);
        else
            pos = CUT; carried = 0;
            cutD = ease(seg(t, 1.4, 1.9)) - ease(seg(t, CUT_T - 0.7, CUT_T - 0.1));
        end
        if t > CUT_T
            cutD = 0; t = 0; st = 4;
        end
    end
    if st == 4                                   % PROCESS 2: finishing (A 5 s, B 8 s)
        fin = FIN_A_T;
        if isB ~= 0
            fin = FIN_B_T;
        end
        if t < 0.6
            pos = CUT;
        elseif t < 1.4
            carried = 1;
            pos = arcmove(CUT, PRESS, (t - 0.6) / 0.8, 0.10);
        else
            carried = 0; pos = PRESS;
            pressD = ease(seg(t, 2.0, 2.6)) - ease(seg(t, fin - 0.8, fin - 0.1));
        end
        if t > fin
            pressD = 0; t = 0; st = 5;
        end
    end
    if st == 5                                   % INSPECTION (camera + OpenCV in the 3D cell)
        if t < 0.5
            pos = PRESS;
        elseif t < INSP_MOVE_T
            carried = 1;
            pos = arcmove(PRESS, INSP, (t - 0.5) / (INSP_MOVE_T - 0.5), 0.10);
        else
            carried = 0; pos = INSP;
        end
        if t > INSP_READ_T
            if isB ~= 0
                flagged = rand() < P_DETECT;
            else
                flagged = rand() < P_FALSE;
            end
            tot = tot + 1;
            if lastSort > 0
                takt = simT - lastSort;
            end
            lastSort = simT;
            t = 0;
            if flagged
                rej = rej + 1; st = 6;
            else
                good = good + 1; st = 7;
            end
        end
    end
    if st == 6                                   % REJECT BIN
        if t < 0.6
            pos = INSP; carried = 0;
        else
            carried = 1;
            pos = arcmove(INSP, REJP, (t - 0.6) / (SORT_T - 0.6), 0.12);
        end
        if t > SORT_T
            carried = 0; pos = REJP; t = 0; st = 0;
        end
    end
    if st == 7                                   % GOOD BIN
        if t < 0.6
            pos = INSP; carried = 0;
        else
            carried = 1;
            pos = arcmove(INSP, GOODP, (t - 0.6) / (SORT_T - 0.6), 0.12);
        end
        if t > SORT_T
            carried = 0; pos = GOODP; t = 0; st = 9;
        end
    end
    if st == 9                                   % PACKAGING
        if t < 2.5
            pos = GOODP; carried = 0;
        elseif t < 6.0
            carried = 1;
            pos = arcmove(GOODP, PACKP + [0 0 0.03], (t - 2.5) / 3.5, LIFTP());
        elseif t < 6.5
            carried = 0;
            pos = PACKP + [0 0 0.03] * (1 - ease(seg(t, 6.0, 6.5)));
        else
            pos = PACKP; carried = 0;
        end
        if t > 7.3
            pack = pack + 1; t = 0; st = 0;
        end
    end
end

rate = 0;
if simT > 1
    rate = tot / (simT / 60);
end
defPct = 0;
if tot > 0
    defPct = 100 * rej / tot;
end
out = [st good rej tot pack floor(pack / PACK_LEVELS) defPct rate takt pos(1) pos(2) pos(3) carried cutD pressD isB];
end

function p = arcmove(a, b, u, lift)
u = min(max(u, 0), 1);
p = a + (b - a) * ease(u);
p(3) = p(3) + lift * sin(pi * ease(u));
end

function e = ease(u)
u = min(max(u, 0), 1);
e = u * u * (3 - 2 * u);
end

function u = seg(t, t0, t1)
u = (t - t0) / (t1 - t0);
end

function v = LIFTP()
v = 0.15;
end
