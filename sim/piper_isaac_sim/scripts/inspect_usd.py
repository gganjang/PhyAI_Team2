"""Dump joints/drives/masses of a USD robot. Usage: inspect_usd.py <usd>"""
import sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics, PhysxSchema

stage = Usd.Stage.Open(sys.argv[1])
print("STAGE", stage, len(list(stage.Traverse())), flush=True)
for p in stage.Traverse():
    t = p.GetTypeName()
    schemas = p.GetAppliedSchemas()
    if "Joint" in t or t == "PhysicsScene" or "PhysicsArticulationRootAPI" in schemas:
        attrs = {a.GetName(): a.Get() for a in p.GetAttributes() if a.HasAuthoredValue()
                 and not a.GetName().startswith(("xformOp", "physics:localRot", "physics:localPos"))}
        rels = {r.GetName(): [str(x) for x in r.GetTargets()] for r in p.GetRelationships() if r.HasAuthoredTargets()}
        print("PRIM", p.GetPath(), t, schemas, attrs, rels)
    if "PhysicsMassAPI" in schemas or "PhysicsRigidBodyAPI" in schemas:
        m = UsdPhysics.MassAPI(p)
        attrs = {a.GetName(): a.Get() for a in p.GetAttributes() if a.HasAuthoredValue() and ("physics" in a.GetName() or "physx" in a.GetName())}
        print("BODY", p.GetPath(), schemas, attrs)
sys.stdout.flush()
app.close()
