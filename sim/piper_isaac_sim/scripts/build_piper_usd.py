"""Wrap the converted Menagerie PiPER into /workspace/assets/piper/piper.usd with PhysX settings
that mirror piper.xml:
  - Physics variant = physx (drives from <position> actuators: kp->stiffness, kv->damping, forcerange->maxForce)
  - gravcomp="1" on every body  -> physxRigidBody:disableGravity = true
  - joint armature 0.005        -> physxJoint:armature
  - joint8 = -joint7 (equality) -> NewtonMimicAPI kept from the importer (read by PhysX at runtime)
  - USD angular drive gains are per degree; the importer copied kp/kv (per rad) verbatim, so
    revolute stiffness/damping are scaled by pi/180 to get kp/kv in N*m/rad as in MuJoCo
Run: sim/piper_isaac_sim/run.sh <name> /workspace/repo/sim/piper_isaac_sim/scripts/build_piper_usd.py
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import math
import sys

from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

SRC = "./piper/piper.usda"
OUT = "/workspace/assets/piper/piper.usd"
ARMATURE = 0.005

stage = Usd.Stage.CreateNew(OUT)
stage.SetMetadata("metersPerUnit", 1.0)
stage.SetMetadata("kilogramsPerUnit", 1.0)
stage.SetMetadata("upAxis", "Z")
root = stage.DefinePrim("/piper", "Xform")
root.GetReferences().AddReference(SRC)
root.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
stage.SetDefaultPrim(root)

for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        PhysxSchema.PhysxJointAPI.Apply(p).CreateArmatureAttr(ARMATURE)
    if p.IsA(UsdPhysics.RevoluteJoint):
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        for a in (d.GetStiffnessAttr(), d.GetDampingAttr()):
            a.Set(a.Get() * math.pi / 180)
        print("JOINT", p.GetPath(), flush=True)

stage.GetRootLayer().Save()
print("WROTE", OUT, flush=True)
sys.stdout.flush()
app.close()
