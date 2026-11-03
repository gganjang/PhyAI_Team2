"""Convert MuJoCo Menagerie agilex_piper (commit 4d038b3) to USD for Isaac Sim 6.2.

Run inside the isaac-sim:6.2.0 container:
  sim/piper_isaac_sim/run.sh <name> /workspace/repo/sim/piper_isaac_sim/scripts/convert_piper_mjcf.py
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import glob
import os

from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

SRC = "/workspace/assets/src/mujoco_menagerie/agilex_piper/piper.xml"
OUT = "/workspace/assets/piper"

config = MJCFImporterConfig(
    mjcf_path=SRC,
    usd_path=OUT,
    import_scene=False,
    fix_base=True,
    allow_self_collision=False,
)
out = MJCFImporter(config).import_mjcf()
print("IMPORT_RESULT", out)
for f in sorted(glob.glob(OUT + "/**/*.usd*", recursive=True)):
    print("FILE", f, os.path.getsize(f))
app.close()
