#!/usr/bin/env python3
"""
plant_sim.py (v2) smart textile line: virtual plant for the OpenPLC program.

It plays the sensors, the vision system, the robot and the packaging machine, so the
pipelined PLC program (SmartTextile v2) runs complete cycles on its own. It talks to
the OpenPLC Modbus TCP server (port 502) with these addresses:

    outputs  Y0..Y13        coils 0..11                  (read)
    HMI      M100..M108     coils 800..808               (start, stop, reset, auto, manual, A, B, clear, E-stop)
    inputs   X0..X17,X20    coils 816..832               (written by this script)
    status   M0, M1, M2     coils 840, 841, 842          (run, fault, e-stop; read)
    KPIs     D100..D109     holding registers 1024..1033 (read)

Install:   pip install pymodbus
Run:       python plant_sim.py --autostart
Keys (Windows console):  s start | x stop | r reset | c clear counters | e emergency stop (HMI button)
                         1 robot fault | 2 camera offline | 3 conveyor jam | 4 no material | 5 E-stop chain open | q quit
"""
import argparse
import sys
import time

# ----------------------------------------------------------------------------- map
X_ORDER = ['X0', 'X1', 'X2', 'X3', 'X4', 'X5', 'X6', 'X7', 'X10', 'X11', 'X12', 'X13', 'X14', 'X15', 'X16', 'X17', 'X20']
X_BASE = 816                                   # X_ORDER[i] -> coil X_BASE + i
Y_ORDER = ['Y0', 'Y1', 'Y2', 'Y3', 'Y4', 'Y5', 'Y6', 'Y7', 'Y10', 'Y11', 'Y12', 'Y13']   # coil index = position
HMI = dict(start=800, stop=801, reset=802, auto=803, manual=804, prodA=805, prodB=806, clear=807, estop=808)
COIL_STATUS = 840                              # M0 run, M1 fault, M2 e-stop
HREG_BASE = 1024                               # D100..D109 -> 1024..1033
PHASE_NAMES = {0: 'IDLE', 10: 'INDEXING', 20: 'WORKING'}
FAULT_NAMES = {0: 'none', 1: 'inspection timeout', 2: 'robot fault/timeout',
               3: 'jam / no material / packaging', 4: 'unknown product type', 5: 'EMERGENCY STOP'}

POS = dict(feed=-4.0, s1=12.0, s2=28.0, s3=44.0, s4=60.0, s5=76.0, end=84.0)   # conveyor positions
SPEED = 18.0                                                                   # position units per second
PITCH = 16.0                                                                   # distance between stations = carrier spacing
WIN = 1.0                                                                      # sensor window (+/-): wide enough for slow, uneven updates


# ----------------------------------------------------------------------------- plant
class Plant:
    """Pipelined conveyor line, cutter, press, camera, robot and packaging machine."""

    def __init__(self, mix='alt'):
        self.mix = mix
        self.flags = dict(robotFault=False, camOff=False, jam=False, noMaterial=False, estop=False)
        self.packed = 0
        self.rejected = 0
        self.made = 0
        self.clear_line()

    def clear_line(self):
        """Operator clears the line (done when RESET is pressed)."""
        self.products = []
        self.pack_prod = None
        self.feed_t = 0.0
        self.fed = False
        self.vision_t = 0.0
        self.vision_for = None
        self.pack_t = 0.0
        self.robot = dict(st='ready', t=0.0, dest='good', prod=None)
        self.p14 = 0.0
        self.p15 = 0.0

    def wip(self):
        return len(self.products) + (1 if self.pack_prod else 0)

    def next_type(self):
        if self.mix == 'A':
            return 'A'
        if self.mix == 'B':
            return 'B'
        if self.mix == 'rand':
            import random
            return 'B' if random.random() < 0.3 else 'A'
        return 'A' if self.made % 2 == 0 else 'B'

    def step(self, Y, dt):
        """Advance the plant by dt seconds. Y = dict of PLC outputs. Returns dict of inputs X."""
        f = self.flags

        def belt():
            return [q for q in self.products if q['where'] == 'belt']

        def at(pos):
            for q in belt():
                if abs(q['pos'] - pos) <= WIN:
                    return q
            return None

        # feeder: one product per work phase at the belt start
        if Y['Y0'] and not f['noMaterial']:
            self.feed_t += dt
            # a new product is placed exactly one carrier pitch behind the product ahead of it, so the spacing
            # on the belt never drifts (otherwise the products cannot all sit on their sensors at once)
            ahead = min(belt(), key=lambda q: q['pos'], default=None)
            spawn = POS['feed']
            if ahead is not None:
                spawn = ahead['pos'] - PITCH * int((ahead['pos'] - POS['feed']) / PITCH + 0.5)
            if (not self.fed and self.feed_t >= 1.2
                    and not any(abs(q['pos'] - spawn) < 3 for q in belt())):
                t = self.next_type()
                self.made += 1
                self.products.append(dict(id=self.made, type=t, pos=spawn, where='belt', cut=False,
                                          finished=False, defective=(t == 'B'), cut_t=0.0, press_t=0.0))
                self.fed = True
        elif not Y['Y0']:
            self.feed_t = 0.0
            self.fed = False

        # conveyor moves every product on the belt together
        if Y['Y1'] and not f['jam']:
            for q in belt():
                q['pos'] = min(POS['end'], q['pos'] + SPEED * dt)

        # cutter and press (visual state only)
        pc = at(POS['s2'])
        if pc is not None and Y['Y2']:
            pc['cut_t'] += dt
            if pc['cut_t'] >= 2.8:
                pc['cut'] = True
        pf = at(POS['s3'])
        if pf is not None and Y['Y3']:
            pf['press_t'] += dt
            if pf['press_t'] >= 4.8:
                pf['finished'] = True

        X = {}
        p1 = at(POS['s1'])
        X['X3'] = p1 is not None
        X['X4'] = p1 is not None and p1['type'] == 'A'
        X['X5'] = p1 is not None and p1['type'] == 'B'
        X['X6'] = pc is not None
        X['X7'] = pf is not None
        p4 = at(POS['s4'])
        X['X10'] = p4 is not None
        p5 = at(POS['s5'])
        X['X20'] = p5 is not None

        # vision system (OpenCV in the real project): result after 1.5 s while triggered
        X['X11'] = False
        X['X12'] = False
        if p4 is not None and Y['Y4'] and not f['camOff']:
            if self.vision_for is not p4:
                self.vision_for = p4
                self.vision_t = 0.0
            self.vision_t += dt
            if self.vision_t >= 1.5:
                X['X11'] = True
                X['X12'] = p4['defective']
        else:
            self.vision_t = 0.0
            self.vision_for = None

        # robot
        R = self.robot
        X['X13'] = False
        if not f['robotFault']:
            st = R['st']
            if st == 'ready':
                X['X13'] = True
                if Y['Y5'] and p5 is not None:
                    R['st'], R['t'], R['prod'] = 'pick', 0.0, p5
                    R['dest'] = 'reject' if Y['Y7'] else 'good'
            elif st == 'pick':
                R['t'] += dt
                if R['t'] >= 1.2:
                    R['prod']['where'] = 'robot'
                    R['st'], R['t'] = 'move', 0.0
            elif st == 'move':
                R['t'] += dt
                if R['t'] >= 1.6:
                    R['st'], R['t'] = 'place', 0.0
            elif st == 'place':
                R['t'] += dt
                if R['t'] >= 0.8:
                    self.products = [q for q in self.products if q is not R['prod']]
                    if R['dest'] == 'good':
                        R['prod']['where'] = 'pack'
                        self.pack_prod = R['prod']
                        self.pack_t = 0.0
                    else:
                        self.rejected += 1
                    R['prod'] = None
                    self.p14 = 0.5
                    R['st'], R['t'] = 'return', 0.0
            elif st == 'return':
                R['t'] += dt
                if R['t'] >= 1.2:
                    R['st'], R['t'] = 'ready', 0.0
        X['X16'] = not f['robotFault']
        X['X17'] = not f['estop']

        # packaging machine
        if self.pack_prod is not None and Y['Y10']:
            self.pack_t += dt
            if self.pack_t >= 3.0:
                self.p15 = 0.5
                self.packed += 1
                self.pack_prod = None
                self.pack_t = 0.0
        else:
            self.pack_t = 0.0

        X['X14'] = self.p14 > 0
        if self.p14 > 0:
            self.p14 -= dt
        X['X15'] = self.p15 > 0
        if self.p15 > 0:
            self.p15 -= dt
        X['X0'] = X['X1'] = X['X2'] = False
        return {k: bool(v) for k, v in X.items()}


# ----------------------------------------------------------------------------- Modbus I/O
class ModbusIO:
    def __init__(self, host, port):
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError:
            sys.exit('pymodbus is not installed. Run:  pip install pymodbus')
        self.c = ModbusTcpClient(host, port=port)
        self._last = {}
        if not self.c.connect():
            sys.exit('Cannot connect to the Modbus server at %s:%d.\n'
                     'Is the runtime running with the program started, and the Modbus server enabled on port %d?'
                     % (host, port, port))

    @staticmethod
    def _check(rr, what):
        if rr is None or rr.isError():
            raise IOError('Modbus error while ' + what + ': ' + str(rr))
        return rr

    def read_outputs(self):
        rr = self._check(self.c.read_coils(0, count=16), 'reading outputs')
        return {tag: bool(rr.bits[i]) for i, tag in enumerate(Y_ORDER)}

    def write_inputs(self, X):
        values = [bool(X[t]) for t in X_ORDER]
        if not getattr(self, '_single_only', False):
            rr = self.c.write_coils(X_BASE, values)
            if rr is not None and not rr.isError():
                return
            self._single_only = True            # server rejects "write multiple coils": write one by one
        for i, v in enumerate(values):
            if self._last.get(i) != v:
                self.set_coil(X_BASE + i, v)
                self._last[i] = v

    def set_coil(self, addr, value):
        self._check(self.c.write_coil(addr, bool(value)), 'writing coil %d' % addr)

    def pulse(self, addr, seconds=0.25):
        self.set_coil(addr, True)
        time.sleep(seconds)
        self.set_coil(addr, False)

    def read_status(self):
        co = self._check(self.c.read_coils(COIL_STATUS, count=8), 'reading status')
        hr = self._check(self.c.read_holding_registers(HREG_BASE, count=10), 'reading registers')
        regs = [r - 65536 if r > 32767 else r for r in hr.registers]
        return dict(run=bool(co.bits[0]), fault=bool(co.bits[1]), estop=bool(co.bits[2]),
                    good=regs[0], reject=regs[1], total=regs[2], defect=regs[3] / 100.0,
                    takt=regs[4] / 10.0, rate=regs[5] / 10.0, product=regs[6], phase=regs[7],
                    code=regs[8], packed=regs[9])

    def reconnect(self):
        """Wait until the Modbus server answers again (the plant keeps its state meanwhile)."""
        try:
            self.c.close()
        except Exception:
            pass
        while not self.c.connect():
            time.sleep(1.0)
        self._single_only = False
        self._last = {}

    def close(self):
        self.c.close()


# ----------------------------------------------------------------------------- console
def status_line(st, Y, plant):
    state = 'E-STOP' if st['estop'] else ('FAULT' if st['fault'] else ('RUNNING' if st['run'] else 'STOPPED'))
    on = ' '.join(t for t in Y_ORDER if Y[t] and t not in ('Y11', 'Y12', 'Y13'))
    flags = ','.join(k for k, v in plant.flags.items() if v)
    return ('%-8s %-9s good %-3d reject %-3d packed %-3d defect %5.1f%%  takt %5.1fs  rate %5.1f/min  on line %d  '
            'fault: %-28s outputs: %-26s %s'
            % (state, PHASE_NAMES.get(st['phase'], '?'), st['good'], st['reject'], st['packed'], st['defect'],
               st['takt'], st['rate'], plant.wip(), FAULT_NAMES.get(st['code'], st['code']), on or '-',
               ('[injected: ' + flags + ']') if flags else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=502)
    ap.add_argument('--mix', choices=['alt', 'A', 'B', 'rand'], default='alt',
                    help='products delivered: alternate A/B (default), only A, only B, random')
    ap.add_argument('--autostart', action='store_true', help='press RESET then START after connecting')
    args = ap.parse_args()

    try:
        import msvcrt
    except ImportError:
        msvcrt = None

    io = ModbusIO(args.host, args.port)
    plant = Plant(args.mix)
    print('Connected to %s:%d. Playing the sensors, camera and robot.' % (args.host, args.port))
    print('Keys: s start | x stop | r reset | c clear counters | e E-stop button | 1 robot fault | 2 camera offline | '
          '3 jam | 4 no material | 5 E-stop chain open | q quit'
          + ('' if msvcrt else '   (keys need Windows; use --autostart)'))

    period = 0.02
    t_prev = time.perf_counter()
    t_status = 0.0
    boot = time.perf_counter()
    auto_pending = args.autostart
    try:
        while True:
            now = time.perf_counter()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            try:

                Y = io.read_outputs()
                X = plant.step(Y, dt)
                io.write_inputs(X)

                if auto_pending and now - boot > 1.0:
                    auto_pending = False
                    io.pulse(HMI['reset'])
                    plant.clear_line()
                    time.sleep(0.3)
                    io.pulse(HMI['start'])

                while msvcrt and msvcrt.kbhit():
                    k = msvcrt.getwch().lower()
                    if k == 's':
                        io.pulse(HMI['start'])
                    elif k == 'x':
                        io.pulse(HMI['stop'])
                    elif k == 'r':
                        io.pulse(HMI['reset'])
                        plant.clear_line()
                    elif k == 'c':
                        io.pulse(HMI['clear'])
                    elif k == 'e':
                        io.pulse(HMI['estop'])
                    elif k in '12345':
                        name = ['robotFault', 'camOff', 'jam', 'noMaterial', 'estop'][int(k) - 1]
                        plant.flags[name] = not plant.flags[name]
                    elif k == 'q':
                        return

                if now - t_status > 1.0:
                    t_status = now
                    print('\r' + status_line(io.read_status(), Y, plant)[:240].ljust(240), end='', flush=True)

            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print('\nModbus link lost (%s). Waiting for the PLC ...' % str(exc)[:70])
                io.reconnect()
                print('Reconnected.')
                t_prev = time.perf_counter()
                continue

            time.sleep(max(0.0, period - (time.perf_counter() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        io.close()


if __name__ == '__main__':
    main()
