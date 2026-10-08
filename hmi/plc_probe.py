"""plc_probe.py - checks what the OpenPLC program really sees and whether RESET / START work.
Stop the HMI first (Ctrl+C in its window), keep the OpenPLC Runtime running, then:  python plc_probe.py"""
import sys, time
try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    sys.exit('pip install pymodbus')

host, port = '127.0.0.1', 502
c = ModbusTcpClient(host, port=port, timeout=2)
if not c.connect():
    sys.exit('Cannot connect to %s:%d - is the runtime running and the Modbus server enabled?' % (host, port))

def coil(a):
    r = c.read_coils(a, count=1)
    return None if r is None or r.isError() else bool(r.bits[0])

def status():
    r = c.read_coils(840, count=3)
    return None if r is None or r.isError() else (bool(r.bits[0]), bool(r.bits[1]), bool(r.bits[2]))

def feed_inputs():                      # healthy field: robot ready X13=827, robot healthy X16=830, e-stop OK X17=831
    for a in (827, 830, 831):
        c.write_coil(a, True)

def hold(seconds):
    t = time.time()
    while time.time() - t < seconds:
        feed_inputs(); time.sleep(0.05)

def pulse(a, s=0.5):
    c.write_coil(a, True); hold(s); c.write_coil(a, False); hold(0.3)

print('1. status before (run, fault, estop):', status())
hold(1.0)
print('2. inputs written, read back  X13=%s  X16=%s  X17=%s   (all must be True)' % (coil(827), coil(830), coil(831)))
print('3. HMI coils now  start=%s stop=%s reset=%s estop=%s   (all should be False)' % (coil(800), coil(801), coil(802), coil(808)))
pulse(802)
print('4. after RESET pulse, status:', status())
pulse(800)
print('5. after START pulse, status:', status(), '  (expect run=True, fault=False, estop=False)')
y = c.read_coils(0, count=12)
print('6. outputs Y0..Y13 (run lamp is the 11th):', [int(b) for b in y.bits[:12]] if y and not y.isError() else y)
c.close()
