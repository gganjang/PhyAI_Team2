"""Optional view planning from a configured robot/camera installation model.

Only requested by configurations selecting this backend. Hardware installations
without MuJoCo continue to use fixed poses or their own ScanNode adapter.
"""
from pathlib import Path


def installation_model(config):
    options=config.get('view_planner',{})
    if options.get('backend')!='mujoco_geometry':
        raise NotImplementedError('Candidate inspection requires a configured robot view planner.')
    import mujoco
    from piper_mujoco_sim.scene import build_scene
    from piper_mujoco_sim.inspection import scan_geometry
    from .sim_config import geometry_matches
    robot=Path(options['robot_path'])
    scene=Path(options['scene_config'])
    model=mujoco.MjModel.from_xml_string(build_scene(robot,scene))
    if not geometry_matches(config['survey_geometry'],scan_geometry(model)):
        raise ValueError('Installation model changed since survey generation; regenerate the survey and relaunch.')
    return model


def create_view_planner(config, targets):
    model=installation_model(config)
    from piper_mujoco_sim.inspection import InspectionPlanner
    # Free objects are disabled; focus positions come from measured requests.
    return InspectionPlanner(model,list(targets.values()),None,
        gripper=config['scan'].get('gripper_position',.035),rgb_mask_clearance=True)


class RobotMaskModel:
    """Project only known robot geometry at capture-matched measured joints."""
    def __init__(self, config):
        import mujoco
        self.model=installation_model(config)
        self.data=mujoco.MjData(self.model)
        self.addresses=[self.model.joint(f'joint{i}').qposadr[0] for i in range(1,7)]

    def project(self, capture, history):
        import mujoco
        from piper_mujoco_sim.robot_mask import robot_mask
        if not history:
            raise ValueError('Waiting for capture-matched joint feedback for robot exclusion.')
        stamp,joints=min(history,key=lambda sample:abs(sample[0]-capture.stamp))
        if abs(stamp-capture.stamp)>.05:
            raise ValueError('Waiting for capture-matched joint feedback for robot exclusion.')
        self.data.qpos[self.addresses]=joints[:6]
        self.data.qpos[self.model.joint('joint7').qposadr[0]]=joints[6]
        self.data.qpos[self.model.joint('joint8').qposadr[0]]=-joints[6]
        mujoco.mj_kinematics(self.model,self.data)
        return robot_mask(self.model,self.data,capture)
