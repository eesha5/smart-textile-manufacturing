#!/usr/bin/env python3
"""
textile_hmi.py - live web HMI + virtual plant for the Smart Textile line running on OpenPLC.

What it is
  * It plays the whole plant for the PLC (conveyor, sensors, cutter, heat press, camera, robot, packaging machine),
    exactly like plant_sim.py, and talks to OpenPLC over Modbus TCP (port 502).
  * It serves an operator screen in your browser: animated facility schematic, KPIs, live PLC tags, fault
    injection, event log and the Start / Stop / Reset / E-STOP buttons (these press the HMI coils M100..M108).

Files:  keep this file in the SAME folder as plant_sim.py (it re-uses its plant model).
Install: pip install pymodbus
Run:    python textile_hmi.py                (then open http://127.0.0.1:3000)
        python textile_hmi.py --autostart    (also presses RESET + START once the PLC answers)
Options: --plc-host 127.0.0.1  --plc-port 502  --port 3000  --mix alt|A|B|rand

Do NOT run plant_sim.py at the same time: both would write the same input coils.
"""
import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import plant_sim as ps
except ImportError:
    sys.exit('plant_sim.py must be in the same folder as textile_hmi.py')

PERIOD = 0.02


class PlcLink(ps.ModbusIO):
    """ModbusIO that does not exit when the PLC is not there yet; it just reports 'disconnected'."""

    def __init__(self, host, port):
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError:
            sys.exit('pymodbus is not installed. Run:  pip install pymodbus')
        self.c = ModbusTcpClient(host, port=port)
        self._last = {}
        self.connected = False

    def try_connect(self):
        try:
            self.connected = bool(self.c.connect())
        except Exception:
            self.connected = False
        self._single_only = False
        self._last = {}
        return self.connected


class App:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.plant = ps.Plant(args.mix)
        self.io = PlcLink(args.plc_host, args.plc_port)
        self.cmds = deque()
        self.events = deque(maxlen=60)
        self.status = dict(run=False, fault=False, estop=False, good=0, reject=0, total=0, defect=0.0, takt=0.0,
                           rate=0.0, product=0, phase=0, code=0, packed=0)
        self.Y = {t: False for t in ps.Y_ORDER}
        self.X = {t: False for t in ps.X_ORDER}
        self.hmi = {k: False for k in ps.HMI}
        self.pulses = {}
        self.manual = False
        self.t0 = time.time()
        self.last = dict(run=None, code=0, estop=False, good=0, reject=0, packed=0, connected=None)
        self.autostart = args.autostart
        self.ev('HMI started. Waiting for the PLC at %s:%d' % (args.plc_host, args.plc_port), 'info')

    # ------------------------------------------------------------------ events
    def ev(self, text, level='info'):
        self.events.appendleft(dict(t=time.strftime('%H:%M:%S'), text=text, level=level))

    # ------------------------------------------------------------------ commands
    def command(self, name):
        self.cmds.append(name)

    def _do(self, name):
        p = self.plant
        if name in ('start', 'stop', 'reset', 'clear', 'estop'):
            self.pulses[ps.HMI[name]] = [time.time() + 0.3, False]
            self.hmi[name] = True
            if name == 'reset':
                p.clear_line()
            labels = dict(start='START pressed', stop='STOP pressed', reset='RESET pressed (line cleared)',
                          clear='Counters and KPIs cleared', estop='EMERGENCY STOP pressed (HMI button)')
            self.ev(labels[name], 'warn' if name in ('estop', 'reset') else 'info')
        elif name in ('auto', 'manual'):
            self.pulses[ps.HMI[name]] = [time.time() + 0.3, False]
            self.manual = name == 'manual'
            self.ev('Mode: ' + ('MANUAL (product type from HMI)' if self.manual else 'AUTO (type from sensors)'), 'info')
        elif name in ('prodA', 'prodB'):
            self.pulses[ps.HMI[name]] = [time.time() + 0.3, False]
            self.ev('Manual product type: ' + name[-1], 'info')
        elif name.startswith('fault:'):
            key = name.split(':', 1)[1]
            if key in p.flags:
                p.flags[key] = not p.flags[key]
                nice = dict(robotFault='Robot fault', camOff='Camera offline', jam='Conveyor jam',
                            noMaterial='No material at feeder', estop='E-stop chain opened (field)')[key]
                self.ev(nice + (' INJECTED' if p.flags[key] else ' CLEARED'), 'bad' if p.flags[key] else 'ok')
        elif name == 'faults_off':
            for k in p.flags:
                p.flags[k] = False
            self.ev('All injected faults cleared', 'ok')

    # ------------------------------------------------------------------ simulation loop
    def loop(self):
        n = 0
        t_prev = time.perf_counter()
        boot = None
        while True:
            now = time.perf_counter()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            try:
                if not self.io.connected:
                    if self.io.try_connect():
                        boot = time.time()
                        with self.lock:
                            self.ev('PLC connected (Modbus %s:%d)' % (self.args.plc_host, self.args.plc_port), 'ok')
                    else:
                        time.sleep(1.0)
                        t_prev = time.perf_counter()
                        continue
                with self.lock:
                    while self.cmds:
                        self._do(self.cmds.popleft())
                    # HMI coils: held for 0.3 s so the PLC sees the edge
                    for addr, pl in list(self.pulses.items()):
                        if not pl[1]:
                            self.io.set_coil(addr, True)
                            pl[1] = True
                        elif time.time() >= pl[0]:
                            self.io.set_coil(addr, False)
                            del self.pulses[addr]
                            for k, a in ps.HMI.items():
                                if a == addr:
                                    self.hmi[k] = False
                    self.Y = self.io.read_outputs()
                    self.X = self.plant.step(self.Y, dt)
                    self.io.write_inputs(self.X)
                    if n % 5 == 0:
                        self.status = self.io.read_status()
                        self._watch()
                    if self.autostart and boot and time.time() - boot > 1.5:
                        self.autostart = False
                        self.cmds.append('reset')
                        self.cmds.append('start')
                n += 1
            except Exception as exc:                                  # link lost: wait and reconnect
                with self.lock:
                    self.ev('PLC link lost (%s)' % str(exc)[:60], 'bad')
                    self.io.connected = False
                try:
                    self.io.c.close()
                except Exception:
                    pass
                t_prev = time.perf_counter()
                continue
            time.sleep(max(0.0, PERIOD - (time.perf_counter() - now)))

    def _watch(self):
        s, l = self.status, self.last
        if l['run'] is not None:
            if s['run'] != l['run']:
                self.ev('System ' + ('RUNNING' if s['run'] else 'STOPPED'), 'ok' if s['run'] else 'info')
            if s['code'] != l['code'] and s['code']:
                self.ev('FAULT %d: %s' % (s['code'], ps.FAULT_NAMES.get(s['code'], '?')), 'bad')
            if s['code'] != l['code'] and not s['code']:
                self.ev('Fault cleared', 'ok')
            if s['good'] > l['good']:
                self.ev('Good product sorted. Packaging started', 'ok')
            if s['reject'] > l['reject']:
                self.ev('Defective product REJECTED', 'warn')
            if s['packed'] > l['packed']:
                self.ev('Product packaged (%d)' % s['packed'], 'ok')
        l.update(run=s['run'], code=s['code'], good=s['good'], reject=s['reject'], packed=s['packed'])

    # ------------------------------------------------------------------ state for the browser
    def snapshot(self):
        with self.lock:
            p = self.plant
            R = p.robot
            s = self.status
            return dict(
                plc=self.io.connected, time=time.strftime('%H:%M:%S'), up=int(time.time() - self.t0),
                status=s, phaseName=ps.PHASE_NAMES.get(s['phase'], '?'), faultName=ps.FAULT_NAMES.get(s['code'], '?'),
                Y=self.Y, X=self.X, hmi=self.hmi, flags=dict(p.flags), manual=self.manual,
                products=[dict(id=q['id'], type=q['type'], pos=q['pos'], where=q['where'], cut=q['cut'],
                               finished=q['finished'], defective=q['defective']) for q in p.products],
                robot=dict(st=R['st'], t=R['t'], dest=R['dest'], fault=p.flags['robotFault']),
                pack=dict(has=p.pack_prod is not None, t=p.pack_t, type=(p.pack_prod or {}).get('type', '')),
                vision=dict(t=p.vision_t), pos=ps.POS, wip=p.wip(),
                events=list(self.events))


HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Smart Textile Manufacturing</title>
<style>
:root{
  --bg:#0a0f1a; --panel:#0f1626; --panel2:#131c30; --line:#223050; --text:#dbe4f5; --dim:#7d8db0;
  --green:#22c55e; --amber:#f59e0b; --red:#ef4444; --blue:#3b82f6; --cyan:#22d3ee; --violet:#a78bfa;
}
*{box-sizing:border-box}
html,body{margin:0;height:100%;background:var(--bg);color:var(--text);font:13px/1.35 "Segoe UI",system-ui,sans-serif}
body{display:flex;flex-direction:column;min-height:100vh}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;border-bottom:1px solid var(--line);flex-wrap:wrap}
header h1{font-size:17px;letter-spacing:.06em;margin:0 auto 0 0;font-weight:700}
.badge{display:flex;align-items:center;gap:7px;padding:5px 11px;border:1px solid var(--line);border-radius:6px;background:var(--panel);font-size:12px;letter-spacing:.04em;white-space:nowrap}
.dot{width:9px;height:9px;border-radius:50%;background:#475569;flex:none}
.dot.on{background:var(--green);box-shadow:0 0 8px var(--green)}
.dot.bad{background:var(--red);box-shadow:0 0 8px var(--red)}
.dot.warn{background:var(--amber);box-shadow:0 0 8px var(--amber)}
.clock{font-variant-numeric:tabular-nums;color:var(--dim)}
main{flex:1;display:grid;grid-template-columns:300px minmax(0,1fr) 320px;gap:10px;padding:10px;min-height:0}
.col{display:flex;flex-direction:column;gap:10px;min-height:0}
.card{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px 12px}
.card h2{margin:0 0 8px;font-size:11px;letter-spacing:.12em;color:var(--dim);font-weight:700;text-transform:uppercase}
.btns{display:grid;grid-template-columns:1fr 1fr;gap:7px}
button{font:inherit;font-weight:700;letter-spacing:.04em;color:#fff;border:0;border-radius:5px;padding:9px 8px;cursor:pointer;background:#334155;transition:filter .15s,transform .05s}
button:hover{filter:brightness(1.15)} button:active{transform:translateY(1px)}
button.start{background:#15803d} button.stop{background:#b45309} button.reset{background:#1d4ed8} button.clr{background:#475569}
button.estop{background:#b91c1c;grid-column:1/3;font-size:15px;padding:13px;box-shadow:0 0 0 2px #7f1d1d inset}
button.tog{background:#1e293b;border:1px solid var(--line);font-weight:600;text-align:left;font-size:12px}
button.tog.active{background:#7f1d1d;border-color:var(--red)}
button.seg{background:#1e293b;border:1px solid var(--line)} button.seg.active{background:#1d4ed8;border-color:#60a5fa}
.faults{display:grid;gap:6px}
.stat{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.tile{background:var(--panel2);border:1px solid var(--line);border-radius:5px;padding:8px 10px}
.tile .k{font-size:10px;letter-spacing:.1em;color:var(--dim);text-transform:uppercase}
.tile .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums}
.tile .v small{font-size:12px;color:var(--dim);font-weight:500}
.alarm{border-radius:6px;padding:10px 12px;font-weight:700;letter-spacing:.05em;border:1px solid var(--line);background:var(--panel2);display:flex;gap:10px;align-items:center}
.alarm.run{border-color:#166534;background:#0b2a17;color:#86efac}
.alarm.bad{border-color:#991b1b;background:#3b0d0d;color:#fecaca;animation:blink 1s infinite}
.alarm.warn{border-color:#92400e;background:#33200a;color:#fde68a}
@keyframes blink{50%{background:#5b1111}}
.tags{display:grid;grid-template-columns:1fr 1fr;gap:3px 12px;font-size:11.5px}
.tag{display:flex;align-items:center;gap:7px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.tag b{font-weight:600;color:#9fb0d0;min-width:26px}
.sub{font-size:10px;letter-spacing:.1em;color:var(--dim);margin:8px 0 4px;text-transform:uppercase}
.log{flex:1;min-height:120px;overflow:auto;font:11.5px/1.5 Consolas,monospace}
.log div{white-space:nowrap}
.log .ok{color:#86efac}.log .warn{color:#fcd34d}.log .bad{color:#fca5a5}.log .info{color:#93c5fd}
.log span{color:var(--dim)}
#stage{width:100%;height:auto;display:block;background:radial-gradient(ellipse at 50% 30%,#101a30,#0a0f1a 70%);border-radius:4px}
.seq{display:flex;align-items:center;justify-content:space-between;position:relative;padding:14px 20px 4px}
.seq:before{content:"";position:absolute;left:34px;right:34px;top:25px;height:2px;background:var(--line)}
.step{position:relative;text-align:center;font-size:10.5px;letter-spacing:.08em;color:var(--dim);flex:1}
.step i{display:block;margin:0 auto 6px;width:22px;height:22px;border-radius:50%;background:#1e293b;border:2px solid #334155}
.step.act{color:#e2e8f0}.step.act i{background:var(--blue);border-color:#93c5fd;box-shadow:0 0 12px var(--blue)}
.step.done i{background:var(--green);border-color:#86efac}
.hint{color:var(--dim);font-size:11px;margin-top:6px}
.offline{position:fixed;left:50%;top:70px;transform:translateX(-50%);background:#3b0d0d;border:1px solid var(--red);color:#fecaca;padding:10px 18px;border-radius:6px;z-index:5;display:none}
@media (max-width:1100px){main{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <h1>SMART TEXTILE MANUFACTURING</h1>
  <div class="badge"><span class="dot" id="b-sys"></span><span id="t-sys">SYSTEM</span></div>
  <div class="badge"><span class="dot" id="b-plc"></span><span id="t-plc">PLC</span></div>
  <div class="badge"><span class="dot" id="b-mode"></span><span id="t-mode">AUTO</span></div>
  <div class="badge"><span class="dot" id="b-es"></span><span id="t-es">E-STOP</span></div>
  <div class="clock" id="clock">--:--:--</div>
</header>
<div class="offline" id="offline">The HMI server is not answering. Is textile_hmi.py still running?</div>

<main>
  <div class="col">
    <div class="card">
      <h2>Operator controls (HMI coils M100 - M108)</h2>
      <div class="btns">
        <button class="start" data-cmd="start">START</button>
        <button class="stop" data-cmd="stop">STOP</button>
        <button class="reset" data-cmd="reset">RESET</button>
        <button class="clr" data-cmd="clear">CLEAR KPIs</button>
        <button class="estop" data-cmd="estop">EMERGENCY STOP</button>
      </div>
      <div class="sub">Mode</div>
      <div class="btns">
        <button class="seg" id="m-auto" data-cmd="auto">AUTO</button>
        <button class="seg" id="m-man" data-cmd="manual">MANUAL</button>
        <button class="seg" data-cmd="prodA">Next: type A</button>
        <button class="seg" data-cmd="prodB">Next: type B</button>
      </div>
      <div class="hint">Type A / B buttons only work in MANUAL mode.</div>
    </div>
    <div class="card">
      <h2>Fault injection (field side)</h2>
      <div class="faults">
        <button class="tog" data-cmd="fault:robotFault" id="f-robotFault">Robot fault (X16 opens)</button>
        <button class="tog" data-cmd="fault:camOff" id="f-camOff">Camera offline (no result)</button>
        <button class="tog" data-cmd="fault:jam" id="f-jam">Conveyor jam (belt stuck)</button>
        <button class="tog" data-cmd="fault:noMaterial" id="f-noMaterial">No material at feeder</button>
        <button class="tog" data-cmd="fault:estop" id="f-estop">E-stop chain open (X17 opens)</button>
        <button class="tog" data-cmd="faults_off">Clear all injected faults</button>
      </div>
      <div class="hint">After a fault: remove the cause, press RESET, then START.</div>
    </div>
    <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:170px">
      <h2>Event log</h2>
      <div class="log" id="log"></div>
    </div>
  </div>

  <div class="col">
    <div class="card" style="padding:8px">
      <h2 style="padding:2px 4px">Production line schematic (live)</h2>
      <svg id="stage" viewBox="0 0 1000 500" xmlns="http://www.w3.org/2000/svg"></svg>
      <div class="seq" id="seq"></div>
    </div>
    <div class="alarm" id="alarm"><span class="dot" id="al-dot"></span><span id="al-text">-</span></div>
  </div>

  <div class="col">
    <div class="card">
      <h2>Production KPIs (PLC registers D100 - D109)</h2>
      <div class="stat">
        <div class="tile"><div class="k">Good</div><div class="v" id="k-good">0</div></div>
        <div class="tile"><div class="k">Rejected</div><div class="v" id="k-rej">0</div></div>
        <div class="tile"><div class="k">Total</div><div class="v" id="k-tot">0</div></div>
        <div class="tile"><div class="k">Defect rate</div><div class="v"><span id="k-def">0</span><small> %</small></div></div>
        <div class="tile"><div class="k">Cycle (takt)</div><div class="v"><span id="k-takt">0</span><small> s</small></div></div>
        <div class="tile"><div class="k">Rate</div><div class="v"><span id="k-rate">0</span><small> /min</small></div></div>
        <div class="tile"><div class="k">Packaged</div><div class="v" id="k-pack">0</div></div>
        <div class="tile"><div class="k">On the line</div><div class="v" id="k-wip">0</div></div>
      </div>
      <div class="hint">Phase: <b id="k-phase">-</b> &nbsp; Product at station 1: <b id="k-prod">-</b></div>
    </div>
    <div class="card">
      <h2>PLC tags (OpenPLC Modbus)</h2>
      <div class="sub" style="margin-top:0">Inputs  PLC &larr; plant</div>
      <div class="tags" id="tags-in"></div>
      <div class="sub">Outputs  PLC &rarr; plant</div>
      <div class="tags" id="tags-out"></div>
    </div>
  </div>
</main>

<script>
var XN = {X3:'At S1 feed/ID',X4:'Type A',X5:'Type B',X6:'At S2 cutting',X7:'At S3 finishing',X10:'At S4 inspect',X11:'Insp. complete',X12:'Defect found',X13:'Robot ready',X14:'Place done',X15:'Packaged',X16:'Robot healthy',X17:'E-stop OK',X20:'At S5 robot'};
var YN = {Y0:'Feeder',Y1:'Conveyor',Y2:'Cutter',Y3:'Heat press',Y4:'Camera trigger',Y5:'Robot pick',Y6:'Dest: good',Y7:'Dest: reject',Y10:'Packaging',Y11:'Buzzer',Y12:'Run lamp',Y13:'Fault lamp'};
var S = null, disp = {}, T0 = performance.now();
var NS = 'http://www.w3.org/2000/svg';
var $ = function(id){return document.getElementById(id);};

/* ---- tag lists ---- */
function buildTags(){
  var a = '', b = '', k;
  for (k in XN) a += '<div class="tag"><span class="dot" id="x-'+k+'"></span><b>'+k+'</b>'+XN[k]+'</div>';
  for (k in YN) b += '<div class="tag"><span class="dot" id="y-'+k+'"></span><b>'+k+'</b>'+YN[k]+'</div>';
  $('tags-in').innerHTML = a; $('tags-out').innerHTML = b;
}
buildTags();

/* ---- static schematic ---- */
var BELT_Y = 270;
function px(pos){ return 70 + (pos + 4) * 9.1; }
var st = $('stage');
function el(tag, attrs, parent, text){
  var e = document.createElementNS(NS, tag), k;
  for (k in attrs) e.setAttribute(k, attrs[k]);
  if (text !== undefined) e.textContent = text;
  (parent || st).appendChild(e); return e;
}
var dyn = {};
function buildStage(){
  var P = {s1:12,s2:28,s3:44,s4:60,s5:76};
  el('rect',{x:40,y:BELT_Y-24,width:860,height:48,rx:6,fill:'#141d33',stroke:'#2a3a62'});
  dyn.belt = el('line',{x1:50,y1:BELT_Y+14,x2:890,y2:BELT_Y+14,stroke:'#3b82f6','stroke-width':3,'stroke-dasharray':'14 12',opacity:.55});
  el('line',{x1:50,y1:BELT_Y-14,x2:890,y2:BELT_Y-14,stroke:'#2a3a62','stroke-width':2});
  /* feeder */
  el('rect',{x:20,y:BELT_Y-100,width:100,height:62,rx:5,fill:'#10213f',stroke:'#3b82f6'});
  el('text',{x:70,y:BELT_Y-78,'text-anchor':'middle',fill:'#9db4e0','font-size':12,'font-weight':700},null,'FEEDER');
  dyn.feedLed = el('circle',{cx:70,cy:BELT_Y-56,r:6,fill:'#334155'});
  el('text',{x:70,y:BELT_Y-44,'text-anchor':'middle',fill:'#7d8db0','font-size':9},null,'raw material');
  /* stations */
  function box(x,w,title,color){
    el('rect',{x:x-w/2,y:BELT_Y-130,width:w,height:92,rx:5,fill:'#10213f',stroke:color,'stroke-width':1.5});
    el('text',{x:x,y:BELT_Y-112,'text-anchor':'middle',fill:'#c7d4f0','font-size':12,'font-weight':700},null,title);
  }
  box(px(P.s1),110,'IDENTIFY','#22d3ee');
  box(px(P.s2),110,'CUTTER','#f59e0b');
  box(px(P.s3),110,'HEAT PRESS','#fb923c');
  box(px(P.s4),110,'CAMERA','#a78bfa');
  el('text',{x:px(P.s1),y:BELT_Y-92,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'station 1');
  el('text',{x:px(P.s2),y:BELT_Y-92,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'station 2 / process 1');
  el('text',{x:px(P.s3),y:BELT_Y-92,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'station 3 / process 2');
  el('text',{x:px(P.s4),y:BELT_Y-92,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'station 4 / inspection');
  dyn.idTxt = el('text',{x:px(P.s1),y:BELT_Y-62,'text-anchor':'middle',fill:'#67e8f9','font-size':14,'font-weight':700},null,'-');
  dyn.blade = el('rect',{x:px(P.s2)-22,y:BELT_Y-70,width:44,height:5,fill:'#f59e0b'});
  dyn.press = el('rect',{x:px(P.s3)-26,y:BELT_Y-72,width:52,height:12,rx:2,fill:'#fb923c'});
  dyn.beam = el('polygon',{points:[px(P.s4)-4,BELT_Y-66,px(P.s4)+4,BELT_Y-66,px(P.s4)+26,BELT_Y-28,px(P.s4)-26,BELT_Y-28],fill:'#a78bfa',opacity:0});
  dyn.cam = el('rect',{x:px(P.s4)-14,y:BELT_Y-76,width:28,height:12,rx:3,fill:'#a78bfa'});
  dyn.verdict = el('text',{x:px(P.s4),y:BELT_Y-47,'text-anchor':'middle',fill:'#fff','font-size':12,'font-weight':700},null,'');
  /* robot cell */
  el('rect',{x:px(P.s5)-44,y:BELT_Y-190,width:88,height:34,rx:5,fill:'#10213f',stroke:'#22c55e'});
  el('text',{x:px(P.s5),y:BELT_Y-168,'text-anchor':'middle',fill:'#86efac','font-size':12,'font-weight':700},null,'ROBOT');
  el('text',{x:px(P.s5),y:BELT_Y+44,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'station 5 / sorting');
  dyn.arm = el('line',{x1:px(P.s5),y1:BELT_Y-156,x2:px(P.s5),y2:BELT_Y-70,stroke:'#22c55e','stroke-width':7,'stroke-linecap':'round'});
  el('circle',{cx:px(P.s5),cy:BELT_Y-156,r:9,fill:'#14532d',stroke:'#22c55e','stroke-width':2});
  dyn.grip = el('rect',{x:0,y:0,width:20,height:8,rx:2,fill:'#bbf7d0'});
  /* bins */
  el('rect',{x:850,y:60,width:130,height:80,rx:6,fill:'#2a1010',stroke:'#ef4444','stroke-width':1.5});
  el('text',{x:915,y:80,'text-anchor':'middle',fill:'#fca5a5','font-size':12,'font-weight':700},null,'REJECT BIN');
  dyn.rejN = el('text',{x:915,y:122,'text-anchor':'middle',fill:'#fecaca','font-size':28,'font-weight':700},null,'0');
  el('rect',{x:830,y:395,width:150,height:92,rx:6,fill:'#0b2a17',stroke:'#22c55e','stroke-width':1.5});
  el('text',{x:905,y:414,'text-anchor':'middle',fill:'#86efac','font-size':12,'font-weight':700},null,'PACKAGING');
  dyn.packBar = el('rect',{x:842,y:477,width:0,height:5,fill:'#22c55e'});
  dyn.packN = el('text',{x:960,y:456,'text-anchor':'end',fill:'#bbf7d0','font-size':20,'font-weight':700},null,'0');
  el('text',{x:960,y:470,'text-anchor':'end',fill:'#7d8db0','font-size':9},null,'packaged');
  dyn.packLed = el('circle',{cx:845,cy:414,r:5,fill:'#334155'});
  /* ground labels */
  el('text',{x:70,y:BELT_Y+44,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'feed');
  el('text',{x:px(P.s1),y:BELT_Y+44,'text-anchor':'middle',fill:'#7d8db0','font-size':10},null,'');
  dyn.prodLayer = el('g',{});
  dyn.carry = el('g',{});
  dyn.stopTxt = el('text',{x:470,y:BELT_Y+95,'text-anchor':'middle',fill:'#fca5a5','font-size':22,'font-weight':700},null,'');
}
buildStage();

var steps = [['FEED','Y0'],['IDENTIFY','X3'],['CUT','Y2'],['FINISH','Y3'],['INSPECT','Y4'],['SORT','Y5'],['PACK','Y10']];
$('seq').innerHTML = steps.map(function(s,i){return '<div class="step" id="st'+i+'"><i></i>'+s[0]+'</div>';}).join('');

/* ---- product sprites ---- */
function sprite(q){
  var g = document.createElementNS(NS,'g');
  var r = el('rect',{x:-15,y:-15,width:30,height:30,rx:4,fill:q.type==='A'?'#2563eb':'#16a34a',stroke:'#e2e8f0','stroke-width':1.5},g);
  el('text',{x:0,y:5,'text-anchor':'middle',fill:'#fff','font-size':14,'font-weight':700},g,q.type);
  var cut = el('line',{x1:-15,y1:-15,x2:15,y2:15,stroke:'#fde68a','stroke-width':2,opacity:0},g);
  var heat = el('rect',{x:-18,y:-18,width:36,height:36,rx:6,fill:'none',stroke:'#fb923c','stroke-width':2,opacity:0},g);
  var bad = el('g',{opacity:0},g);
  el('circle',{cx:7,cy:-6,r:4,fill:'#7f1d1d'},bad); el('circle',{cx:-6,cy:7,r:3,fill:'#7f1d1d'},bad);
  var flag = el('text',{x:0,y:-22,'text-anchor':'middle',fill:'#fca5a5','font-size':10,'font-weight':700,opacity:0},g,'DEFECT');
  g._p = {cut:cut,heat:heat,bad:bad,flag:flag};
  dyn.prodLayer.appendChild(g); return g;
}
var sprites = {};

/* ---- robot geometry ---- */
function ease(t){ t = Math.max(0,Math.min(1,t)); return t*t*(3-2*t); }
function lerp(a,b,t){ return a + (b-a)*t; }
var HOME = {x:px(76), y:BELT_Y-92}, PICK = {x:px(76), y:BELT_Y-8}, GOOD = {x:905, y:BELT_Y+150}, REJ = {x:915, y:BELT_Y-112};
function robotTip(R){
  var d = R.dest === 'reject' ? REJ : GOOD, t, a, b;
  if (R.st === 'ready') return HOME;
  if (R.st === 'pick'){ t = ease(R.t/1.2); return {x:lerp(HOME.x,PICK.x,t), y:lerp(HOME.y,PICK.y,t)}; }
  if (R.st === 'move'){ t = ease(R.t/1.6); return {x:lerp(PICK.x,d.x,t), y:lerp(PICK.y,d.y,t) - Math.sin(t*Math.PI)*40}; }
  if (R.st === 'place') return d;
  t = ease(R.t/1.2); return {x:lerp(d.x,HOME.x,t), y:lerp(d.y,HOME.y,t)};
}

/* ---- render ---- */
function render(){
  if (!S) return;
  var now = (performance.now() - T0)/1000, i, k;
  var s = S.status, Y = S.Y, X = S.X, f = S.flags;
  /* header */
  var sys = s.estop ? ['E-STOP','bad'] : s.fault ? ['FAULT','bad'] : s.run ? ['RUNNING','on'] : ['STOPPED','warn'];
  $('b-sys').className = 'dot ' + sys[1]; $('t-sys').textContent = 'SYSTEM ' + sys[0];
  $('b-plc').className = 'dot ' + (S.plc ? 'on' : 'bad'); $('t-plc').textContent = 'PLC: ' + (S.plc ? 'CONNECTED' : 'DISCONNECTED');
  $('b-mode').className = 'dot on'; $('t-mode').textContent = S.manual ? 'MODE: MANUAL' : 'MODE: AUTO';
  $('b-es').className = 'dot ' + (s.estop ? 'bad' : 'on'); $('t-es').textContent = 'E-STOP: ' + (s.estop ? 'ACTIVE' : 'NORMAL');
  $('clock').textContent = S.time;
  $('m-auto').classList.toggle('active', !S.manual); $('m-man').classList.toggle('active', S.manual);
  for (k in f){ var b = $('f-' + k); if (b) b.classList.toggle('active', !!f[k]); }
  /* alarm */
  var al = $('alarm');
  if (!S.plc){ al.className = 'alarm warn'; $('al-text').textContent = 'WAITING FOR THE PLC (start the program in OpenPLC Editor)'; $('al-dot').className = 'dot warn'; }
  else if (s.estop){ al.className = 'alarm bad'; $('al-text').textContent = 'EMERGENCY STOP ACTIVE: release the E-stop, press RESET, then START'; $('al-dot').className = 'dot bad'; }
  else if (s.fault){ al.className = 'alarm bad'; $('al-text').textContent = 'FAULT ' + s.code + ': ' + S.faultName.toUpperCase() + '  (remove the cause, RESET, START)'; $('al-dot').className = 'dot bad'; }
  else if (s.run){ al.className = 'alarm run'; $('al-text').textContent = 'PRODUCTION RUNNING  (' + S.phaseName + ')'; $('al-dot').className = 'dot on'; }
  else { al.className = 'alarm warn'; $('al-text').textContent = 'SYSTEM STOPPED: press START'; $('al-dot').className = 'dot warn'; }
  /* KPIs */
  $('k-good').textContent = s.good; $('k-rej').textContent = s.reject; $('k-tot').textContent = s.total;
  $('k-def').textContent = s.defect.toFixed(1); $('k-takt').textContent = s.takt.toFixed(1); $('k-rate').textContent = s.rate.toFixed(1);
  $('k-pack').textContent = s.packed; $('k-wip').textContent = S.wip;
  $('k-phase').textContent = S.phaseName; $('k-prod').textContent = s.product === 1 ? 'A' : s.product === 2 ? 'B' : '-';
  /* tags */
  for (k in XN){ $('x-'+k).className = 'dot' + (X[k] ? ' on' : ''); }
  for (k in YN){ $('y-'+k).className = 'dot' + (Y[k] ? (k === 'Y13' || k === 'Y11' ? ' bad' : ' on') : ''); }
  /* sequence bar */
  var act = [Y.Y0 && !f.noMaterial && S.products.some(function(q){return q.pos < 6;}) || Y.Y0, X.X3, Y.Y2 && X.X6, Y.Y3 && X.X7, Y.Y4 && X.X10, S.robot.st !== 'ready', S.pack.has];
  for (i = 0; i < steps.length; i++){ $('st'+i).className = 'step' + (act[i] && s.run ? ' act' : ''); }
  /* event log */
  var lg = '';
  S.events.forEach(function(e){ lg += '<div class="'+e.level+'"><span>'+e.t+'</span> '+e.text+'</div>'; });
  if ($('log')._h !== lg){ $('log').innerHTML = lg; $('log')._h = lg; }
  /* stage: belt, equipment */
  dyn.belt.setAttribute('stroke-dashoffset', (Y.Y1 && !f.jam) ? String(-(now*60) % 26) : '0');
  dyn.belt.setAttribute('stroke', (Y.Y1 && !f.jam) ? '#3b82f6' : (f.jam ? '#ef4444' : '#475569'));
  dyn.feedLed.setAttribute('fill', Y.Y0 ? '#22c55e' : '#334155');
  dyn.blade.setAttribute('y', Y.Y2 && X.X6 ? BELT_Y - 70 + 28 + Math.sin(now*14)*8 : BELT_Y - 70);
  dyn.press.setAttribute('y', Y.Y3 && X.X7 ? BELT_Y - 72 + 18 + Math.sin(now*3)*3 : BELT_Y - 72);
  dyn.press.setAttribute('fill', Y.Y3 && X.X7 ? '#f97316' : '#b45309');
  dyn.beam.setAttribute('opacity', Y.Y4 && X.X10 ? (0.25 + 0.2*Math.sin(now*8)) : 0);
  dyn.cam.setAttribute('fill', f.camOff ? '#475569' : '#a78bfa');
  dyn.verdict.textContent = X.X11 ? (X.X12 ? 'DEFECT' : 'GOOD') : (Y.Y4 && X.X10 ? 'scanning...' : '');
  dyn.verdict.setAttribute('fill', X.X11 ? (X.X12 ? '#fca5a5' : '#86efac') : '#c4b5fd');
  dyn.idTxt.textContent = X.X4 ? 'TYPE A' : X.X5 ? 'TYPE B' : (X.X3 ? '?' : '-');
  dyn.rejN.textContent = s.reject; dyn.packN.textContent = s.packed;
  dyn.packBar.setAttribute('width', S.pack.has ? Math.min(126, S.pack.t / 3.0 * 126) : 0);
  dyn.packLed.setAttribute('fill', Y.Y10 ? '#22c55e' : '#334155');
  dyn.stopTxt.textContent = f.jam ? 'CONVEYOR JAMMED' : (s.estop ? 'EMERGENCY STOP' : (!S.plc ? 'PLC OFFLINE' : ''));
  /* products */
  var seen = {}, tipR = robotTip(S.robot), carried = null;
  S.products.forEach(function(q){
    seen[q.id] = true;
    var g = sprites[q.id] || (sprites[q.id] = sprite(q));
    var x, y;
    if (q.where === 'robot'){ x = tipR.x; y = tipR.y + 16; carried = q; }
    else { var tx = px(q.pos); if (disp[q.id] === undefined) disp[q.id] = tx; disp[q.id] += (tx - disp[q.id]) * 0.4; x = disp[q.id]; y = BELT_Y; }
    g.setAttribute('transform', 'translate(' + x + ',' + y + ')');
    g._p.cut.setAttribute('opacity', q.cut ? 1 : 0);
    g._p.heat.setAttribute('opacity', q.finished ? 0.9 : 0);
    var seenByCam = q.defective && (q.pos >= S.pos.s4 - 1 || q.where === 'robot');
    g._p.bad.setAttribute('opacity', seenByCam ? 1 : 0);
    g._p.flag.setAttribute('opacity', seenByCam ? 1 : 0);
  });
  for (k in sprites){ if (!seen[k]){ sprites[k].remove(); delete sprites[k]; delete disp[k]; } }
  /* packaged product sitting in the packaging station */
  var pk = $('packed-sprite');
  if (S.pack.has){
    if (!pk){ pk = el('rect',{id:'packed-sprite',x:850,y:424,width:30,height:30,rx:4,stroke:'#e2e8f0','stroke-width':1.5}); }
    pk.setAttribute('fill', S.pack.type === 'A' ? '#2563eb' : '#16a34a');
  } else if (pk){ pk.remove(); }
  /* robot arm */
  var bx = px(76), by = BELT_Y - 156;
  dyn.arm.setAttribute('x2', tipR.x); dyn.arm.setAttribute('y2', tipR.y);
  dyn.arm.setAttribute('stroke', S.robot.fault ? '#ef4444' : '#22c55e');
  dyn.grip.setAttribute('x', tipR.x - 10); dyn.grip.setAttribute('y', tipR.y - 2);
}
(function frame(){ render(); requestAnimationFrame(frame); })();

/* ---- server link ---- */
var failures = 0;
function poll(){
  fetch('/api/state').then(function(r){return r.json();}).then(function(j){ S = j; failures = 0; $('offline').style.display = 'none'; })
    .catch(function(){ if (++failures > 5) $('offline').style.display = 'block'; })
    .then(function(){ setTimeout(poll, 70); });
}
poll();
document.addEventListener('click', function(e){
  var b = e.target.closest('button[data-cmd]'); if (!b) return;
  fetch('/api/cmd', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({cmd:b.getAttribute('data-cmd')})});
});
</script>
</body>
</html>
'''


class Handler(BaseHTTPRequestHandler):
    app = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith('/api/state'):
            self._send(200, json.dumps(self.app.snapshot()).encode(), 'application/json')
        elif self.path in ('/', '/index.html'):
            self._send(200, HTML.encode('utf-8'), 'text/html; charset=utf-8')
        else:
            self._send(404, b'not found', 'text/plain')

    def do_POST(self):
        if self.path.startswith('/api/cmd'):
            n = int(self.headers.get('Content-Length') or 0)
            try:
                cmd = json.loads(self.rfile.read(n) or b'{}').get('cmd', '')
            except Exception:
                cmd = ''
            if cmd:
                self.app.command(cmd)
            self._send(200, b'{"ok":true}', 'application/json')
        else:
            self._send(404, b'not found', 'text/plain')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plc-host', default='127.0.0.1')
    ap.add_argument('--plc-port', type=int, default=502)
    ap.add_argument('--port', type=int, default=3000)
    ap.add_argument('--mix', choices=['alt', 'A', 'B', 'rand'], default='alt')
    ap.add_argument('--autostart', action='store_true')
    args = ap.parse_args()

    app = App(args)
    Handler.app = app
    threading.Thread(target=app.loop, daemon=True).start()
    srv = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print('Smart Textile HMI:  http://127.0.0.1:%d   (PLC at %s:%d)   Ctrl+C to stop' % (args.port, args.plc_host, args.plc_port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
