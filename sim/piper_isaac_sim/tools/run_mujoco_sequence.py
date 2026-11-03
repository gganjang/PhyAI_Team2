"""Run sequence.json on Menagerie piper.xml in MuJoCo; write CSV of joint1..joint8 positions.
Targets: ctrl[0:6] = joint1..6, ctrl[6] = gripper (joint7). Starts from keyframe 'home'.
Usage: python run_mujoco_sequence.py piper.xml sequence.json out.csv"""
import json
import sys

import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path(sys.argv[1])
seq = json.load(open(sys.argv[2]))
if len(sys.argv) > 4 and sys.argv[4] == "nofriction":
    model.dof_frictionloss[:] = 0
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home"))
assert abs(model.opt.timestep - seq["dt"]) < 1e-12, model.opt.timestep
names = [f"joint{i}" for i in range(1, 9)]
qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
rows = []
t = 0.0
for seg in seq["segments"]:
    data.ctrl[:7] = seg["target"]
    for _ in range(round(seg["dur"] / seq["dt"])):
        mujoco.mj_step(model, data)
        t += seq["dt"]
        rows.append([t] + [data.qpos[a] for a in qadr])
    for b in ("link6", "link7", "link8"):
        print("LINKPOS", b, " ".join(f"{v:.4f}" for v in data.body(b).xpos))
np.savetxt(sys.argv[3], np.array(rows), delimiter=",", header="t," + ",".join(names), comments="")
print("MUJOCO", mujoco.__version__, "steps", len(rows), "timestep", model.opt.timestep)
