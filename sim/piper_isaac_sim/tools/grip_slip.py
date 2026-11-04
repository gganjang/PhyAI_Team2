"""Block_3 pose relative to the gripper (link7) over the lift, from record_scene.py CSVs.
Usage: grip_slip.py scene.json a.csv [b.csv ...]   (needs mujoco; uses the repo scene for FK)"""
import csv
import sys

import mujoco
import numpy as np

sys.path.insert(0, '/ws/src/PhyAI_Team2/sim/piper_mujoco_sim')
from piper_mujoco_sim.scene import build_scene  # noqa: E402

m = mujoco.MjModel.from_xml_string(build_scene('/opt/mujoco_menagerie/agilex_piper/piper.xml', sys.argv[1]))
d = mujoco.MjData(m)
adr = [m.joint(f'joint{i}').qposadr[0] for i in range(1, 9)]
for path in sys.argv[2:]:
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(path))]
    print('===', path)
    lifted = [r for r in rows if r['z'] > 0.012]
    for r in lifted[::4] + lifted[-1:]:
        d.qpos[adr] = [r[f'j{i}'] for i in range(1, 9)]
        mujoco.mj_kinematics(m, d)
        g = m.body('link7').id
        Rg = d.xmat[g].reshape(3, 3)
        Rb = np.zeros(9)
        mujoco.mju_quat2Mat(Rb, np.array([r['qw'], r['qx'], r['qy'], r['qz']]))
        Rrel = Rg.T @ Rb.reshape(3, 3)
        prel = Rg.T @ (np.array([r['x'], r['y'], r['z']]) - d.xpos[g])
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, Rrel.reshape(-1))
        angle = 2 * np.degrees(np.arccos(min(1, abs(q[0]))))
        print(f"t={r['t']:.1f} z={r['z']:.3f} rel_pos_mm={np.round(prel * 1000, 1)} rel_rot_deg={angle:.1f}")
