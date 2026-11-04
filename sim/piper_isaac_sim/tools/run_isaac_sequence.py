"""Run sequence.json on piper.usd in Isaac Sim (PhysX, 500 Hz); write CSV of joint1..joint8 positions
and print the articulation's joints, limits and drive gains.
Usage: run.sh <name> /workspace/repo/sim/piper_isaac_sim/tools/run_isaac_sequence.py piper.usd sequence.json out.csv [angular_gain_scale]"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import json
import math
import sys

import numpy as np
import omni.physics.tensors as tensors
import omni.timeline
import omni.usd
from omni.physx import get_physx_simulation_interface
from pxr import Gf, PhysxSchema, Usd, UsdPhysics

usd, seq_path, out = sys.argv[1:4]
gain_scale = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
seq = json.load(open(seq_path))
dt = seq["dt"]

omni.usd.get_context().open_stage(usd)
for _ in range(20):
    app.update()
stage = omni.usd.get_context().get_stage()
scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
px.CreateTimeStepsPerSecondAttr(round(1 / dt))
px.CreateEnableGPUDynamicsAttr(False)
px.CreateBroadphaseTypeAttr("MBP")
px.CreateSolverTypeAttr("TGS")

root_path = None
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        root_path = str(p.GetPath())
    if gain_scale != 1.0 and p.IsA(UsdPhysics.RevoluteJoint):
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        for a in (d.GetStiffnessAttr(), d.GetDampingAttr()):
            a.Set(a.Get() * gain_scale)
print("ROOT", root_path, flush=True)

omni.timeline.get_timeline_interface().play()
for _ in range(3):
    app.update()
omni.timeline.get_timeline_interface().pause()
app.update()
sim = tensors.create_simulation_view("numpy", omni.usd.get_context().get_stage_id())
art = sim.create_articulation_view(root_path)
meta = art.shared_metatype
dof = list(meta.dof_names)
print("DOFS", dof, flush=True)
lim = art.get_dof_limits()[0]
stiff = art.get_dof_stiffnesses()[0]
damp = art.get_dof_dampings()[0]
maxf = art.get_dof_max_forces()[0]
arm = art.get_dof_armatures()[0] if hasattr(art, "get_dof_armatures") else [float("nan")] * len(dof)
types = list(meta.dof_types)
for i, n in enumerate(dof):
    print(f"DOF {n} type={types[i]} lower={lim[i][0]:.4f} upper={lim[i][1]:.4f} stiffness={stiff[i]:.4f} "
          f"damping={damp[i]:.4f} maxForce={maxf[i]:.2f} armature={arm[i]:.4f}", flush=True)
names = [f"joint{i}" for i in range(1, 9)]
idx = [dof.index(n) for n in names]
masses = art.get_masses()[0]
for n, m in zip(meta.link_names, masses):
    print(f"MASS {n} {m:.6f}", flush=True)

idx_all = np.arange(1, dtype=np.int32)
q0 = np.zeros((1, len(dof)), dtype=np.float32)
home = seq["home"] + [-seq["home"][6]]
for n, v in zip(names, home):
    q0[0, dof.index(n)] = v
art.set_dof_positions(q0, idx_all)
art.set_dof_velocities(np.zeros_like(q0), idx_all)
art.set_dof_position_targets(q0.copy(), idx_all)

phys = get_physx_simulation_interface()
t = 0.0
rows = []
for seg in seq["segments"]:
    tgt = art.get_dof_position_targets().copy()
    for k in range(6):
        tgt[0, idx[k]] = seg["target"][k]
    tgt[0, idx[6]] = seg["target"][6]          # gripper ctrl -> joint7; joint8 follows via mimic
    art.set_dof_position_targets(tgt, idx_all)
    for _ in range(round(seg["dur"] / dt)):
        phys.simulate(dt, t)
        phys.fetch_results()
        t += dt
        q = art.get_dof_positions()[0]
        rows.append([t] + [float(q[i]) for i in idx])
    tf = art.get_link_transforms()[0]
    for b in ("link6", "link7", "link8"):
        print("LINKPOS", b, " ".join(f"{v:.4f}" for v in tf[list(meta.link_names).index(b)][:3]), flush=True)
np.savetxt(out, np.array(rows), delimiter=",", header="t," + ",".join(names), comments="")
print("ISAAC steps", len(rows), "gain_scale", gain_scale, flush=True)
sys.stdout.flush()
app.close()
