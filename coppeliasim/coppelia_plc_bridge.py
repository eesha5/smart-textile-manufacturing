#!/usr/bin/env python3
"""
coppelia_plc_bridge.py - links the OpenPLC program to the CoppeliaSim robot cell.

What it does
  PLC -> CoppeliaSim : the PLC's run state (M0), fault (M1) and emergency stop (M2) become the
                       CoppeliaSim signal 'PlcRun'.
                       PlcRun = 1  only while the PLC is running with no fault and no E-stop.
                       PlcRun = 0  otherwise, and the robot controller v2 freezes the whole cell.
                       So START, STOP and E-STOP on the HMI (or the Modbus client) really start and
                       stop the robot cell.
  CoppeliaSim -> you : prints the cell's own counters (good, defect, total, packaged, step).

Safety behaviour
  * If the connection to the PLC is lost, the bridge sets PlcRun = 0 (the cell holds) and keeps retrying.
  * When you quit with Ctrl+C the signal is removed, so the cell runs freely again.

Needs the Robot_Controller v2 (robot_controller_v2.lua), which reads 'PlcRun'.

Install:  pip install pymodbus coppeliasim-zmqremoteapi-client
Run:      python coppelia_plc_bridge.py
          (OpenPLC running with the program started and the Modbus server enabled; CoppeliaSim open)
Addresses used (Modbus coils): 840 = M0 run, 841 = M1 fault, 842 = M2 emergency stop.
"""
import argparse
import sys
import time

COIL_STATUS = 840          # M0 run, M1 fault, M2 e-stop


def connect_plc(host, port):
    try:
        from pymodbus.client import ModbusTcpClient
    except ImportError:
        sys.exit('pymodbus is not installed. Run:  pip install pymodbus')
    client = ModbusTcpClient(host, port=port)
    return client if client.connect() else None


def read_plc(client):
    rr = client.read_coils(COIL_STATUS, count=8)
    if rr is None or rr.isError():
        raise IOError(str(rr))
    return bool(rr.bits[0]), bool(rr.bits[1]), bool(rr.bits[2])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plc-host', default='127.0.0.1')
    ap.add_argument('--plc-port', type=int, default=502)
    ap.add_argument('--period', type=float, default=0.1, help='seconds between updates')
    args = ap.parse_args()

    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    except ImportError:
        sys.exit('The CoppeliaSim remote API client is missing. Run:  pip install coppeliasim-zmqremoteapi-client')

    print('Connecting to CoppeliaSim ...')
    sim = RemoteAPIClient().getObject('sim')
    print('Connected to CoppeliaSim.')

    plc = connect_plc(args.plc_host, args.plc_port)
    if plc is None:
        print('PLC not reachable yet at %s:%d. The cell is held (PlcRun = 0) until it is.' % (args.plc_host, args.plc_port))
    last_run = None
    last_print = 0.0
    try:
        while True:
            run, fault, estop, link = False, False, False, True
            try:
                if plc is None:
                    plc = connect_plc(args.plc_host, args.plc_port)
                    if plc is None:
                        raise IOError('no connection')
                run, fault, estop = read_plc(plc)
            except Exception:
                link = False
                try:
                    if plc is not None:
                        plc.close()
                except Exception:
                    pass
                plc = None

            permit = 1 if (link and run and not fault and not estop) else 0
            if permit != last_run:
                sim.setInt32Signal('PlcRun', permit)
                last_run = permit
                why = ('PLC running' if permit else
                       'E-STOP' if estop else 'PLC fault' if fault else
                       'PLC link lost' if not link else 'PLC stopped')
                print('\nPlcRun = %d  (%s)' % (permit, why))

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                try:
                    g = sim.getInt32Signal('GoodCount') or 0
                    d = sim.getInt32Signal('DefectCount') or 0
                    t = sim.getInt32Signal('TotalCount') or 0
                    p = sim.getInt32Signal('PackagedCount') or 0
                    s = sim.getInt32Signal('RobotState')
                    held = sim.getInt32Signal('RobotHeld')
                    print('\rPLC: %-8s | cell: good %d defect %d total %d packaged %d  state %s  %s'
                          % ('E-STOP' if estop else 'FAULT' if fault else 'RUNNING' if run else 'STOPPED' if link else 'NO LINK',
                             g, d, t, p, s, 'HELD' if held else 'running').ljust(110), end='', flush=True)
                except Exception:
                    pass
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sim.clearInt32Signal('PlcRun')
            print('\nPlcRun signal removed: the cell runs freely again.')
        except Exception:
            pass
        try:
            if plc is not None:
                plc.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
