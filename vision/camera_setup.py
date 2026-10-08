"""camera_setup.py - run ONCE with the simulation STOPPED.
Sets /Inspection_Camera view size from the real scene geometry. Moves no object."""
import math

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

CAMERA, TRIGGER = "/Inspection_Camera", "/Sensor_InspectionTrigger"
PRODUCTS = ("/Product_A", "/Product_B")
STATION_Z = 0.30        # where Robot_Controller State 5 places the product
MARGIN = 1.20           # 20 % free space around the product


def p(fn, h, pid):
    r = fn(h, pid)
    return r[-1] if isinstance(r, (list, tuple)) else r


sim = RemoteAPIClient().getObject("sim")
if sim.getSimulationState() != sim.simulation_stopped:
    raise SystemExit("Stop the simulation first (square Stop button), then run this again.")

cam = sim.getObject(CAMERA)
trig = sim.getObject(TRIGGER)


def bbox(h):
    try:
        return [float(v) for v in sim.getShapeBB(h)]
    except Exception:
        g = lambda pid: p(sim.getObjectFloatParam, h, pid)
        return [g(sim.objfloatparam_objbbox_max_x) - g(sim.objfloatparam_objbbox_min_x),
                g(sim.objfloatparam_objbbox_max_y) - g(sim.objfloatparam_objbbox_min_y),
                g(sim.objfloatparam_objbbox_max_z) - g(sim.objfloatparam_objbbox_min_z)]


half_diag, height = 0.0, 1.0
for path in PRODUCTS:
    s = sorted(bbox(sim.getObject(path)))
    print("%s size (sorted) : %.3f x %.3f x %.3f m" % (path, s[0], s[1], s[2]))
    half_diag = max(half_diag, 0.5 * math.hypot(s[1], s[2]))
    height = min(height, s[0])

M = sim.getObjectMatrix(cam, -1)
R = np.array([[M[0], M[1], M[2]], [M[4], M[5], M[6]], [M[8], M[9], M[10]]])
t = np.array([M[3], M[7], M[11]])
tp = sim.getObjectPosition(trig, -1)
pc = R.T @ (np.array([tp[0], tp[1], STATION_Z]) - t)        # station in camera frame
print("camera position         : (%.3f, %.3f, %.3f)" % tuple(t))
print("station (trigger) XY    : (%.3f, %.3f)" % (tp[0], tp[1]))
print("station offset from camera axis: %.3f m, %.3f m | depth %.3f m" % (pc[0], pc[1], pc[2]))
if pc[2] <= 0.02:
    raise SystemExit("The station is BEHIND the camera - the camera is aimed away. Send me this output.")

half_view = max(abs(pc[0]), abs(pc[1])) + half_diag
mode = p(sim.getObjectInt32Param, cam, sim.visionintparam_perspective_operation)

if int(mode) == 0:
    print("projection mode         : ORTHOGRAPHIC")
    sid = getattr(sim, "visionfloatparam_ortho_size", None)
    need = math.ceil(2.0 * half_view * MARGIN * 100) / 100.0
    if sid is None:
        raise SystemExit("API has no ortho-size constant. Type %.2f into the dialog field "
                         "'Persp. angle / ortho. size' and press Apply." % need)
    now = p(sim.getObjectFloatParam, cam, sid)
    print("ortho size now          : %.3f m" % now)
    print("ortho size required     : %.2f m" % need)
    sim.setObjectFloatParam(cam, sid, need)
    print("ortho size after        : %.3f m" % p(sim.getObjectFloatParam, cam, sid))
else:
    print("projection mode         : PERSPECTIVE")
    d_top = pc[2] - height / 2.0
    fov = math.degrees(2.0 * math.atan(MARGIN * half_view / d_top))
    print("perspective angle needed: %.1f deg" % fov)
    if fov > 120:
        raise SystemExit("Angle alone cannot fix this. Send me this output.")
    sim.setObjectFloatParam(cam, sim.visionfloatparam_perspective_angle, math.radians(math.ceil(fov)))
    print("perspective angle set to %d deg" % math.ceil(fov))

print("DONE. Now press Ctrl+S in CoppeliaSim to save the scene.")