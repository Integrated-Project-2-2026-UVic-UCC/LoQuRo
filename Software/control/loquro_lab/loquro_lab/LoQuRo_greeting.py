"""Plays a routine of the real robot (ros2_ws/src/loquro_control/routines/<name>.yaml) in the simulation.

    uv run python loquro_lab/LoQuRo_greeting.py [greeting|squats] [--real]

--real: the same commands are sent at the same time to the real robot, so both move together. It needs
`ros2 launch loquro_bringup robot.launch.py` running and the robot ready (green led) at its initial pose.
Close the window (or Ctrl + C) to stop: the real robot goes back slowly to its initial pose.

Same file and same logic as routine_player in ROS: the robot starts at the firmware initial pose, goes slowly to the
first pose and then through the sequence, every joint at most at `speed` rad/s. The commands are the ones the real
robot receives (order lf rf lh rh x [haa, hfe, kfe]); firmware_to_sim_ctrl converts them to the simulation actuators.
"""
import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import yaml
from mjlab.entity.entity import Entity

from loquro_lab.quad_bot_constants import FIRMWARE_INIT_POSE, firmware_to_sim_ctrl, get_quad_bot_cfg

ROUTINES = Path(__file__).resolve().parents[3] / "ros2_ws" / "src" / "loquro_control" / "routines"
LEGS = ["lf", "rf", "lh", "rh"]
RATE = 50.0         # Hz, command rate of routine_player
SETTLE_TIME = 2.0   # s, wait at the first pose
RETURN_SPEED = 0.5  # rad/s, back to the initial pose when the routine is stopped (real robot)


def load_routine(name):
    data = yaml.safe_load((ROUTINES / f"{name}.yaml").read_text())
    leg_poses = data.get("leg_poses", {})
    poses = {}
    for pose_name, pose in data["poses"].items():
        if isinstance(pose, dict):
            pose = sum((list(leg_poses.get(pose[leg], pose[leg])) for leg in LEGS), [])
        poses[pose_name] = [float(p) for p in pose]
    speed = float(data.get("speed", 0.4))
    steps = [{"pose": poses[s["pose"]], "speed": float(s.get("speed", speed)), "hold": float(s.get("hold", 0.0))}
             for s in data["sequence"]]
    return steps, bool(data.get("loop", False)), speed


class RoutineController:
    """Same state machine as loquro_control/routine_player.py."""

    def __init__(self, name):
        self.steps, self.loop, self.speed = load_routine(name)
        self.dt = 1.0 / RATE
        self.current = list(FIRMWARE_INIT_POSE)
        self.index = 0
        self.wait = 0.0
        self.waiting = False
        self.finished = False

    def update(self):
        """Next /joint_commands message (firmware order)."""
        if self.finished:
            return self.current
        step = self.steps[self.index]
        if not self.waiting:
            max_delta = self.speed * self.dt
            self.current = [c + max(-max_delta, min(max_delta, t - c)) for c, t in zip(self.current, step["pose"])]
            if self.current == step["pose"]:
                self.wait = SETTLE_TIME if self.index == 0 else step["hold"]
                self.waiting = True
        else:
            self.wait -= self.dt
            if self.wait <= 0.0:
                self.waiting = False
                self.index += 1
                if self.index >= len(self.steps):
                    self.index = 0
                    self.finished = not self.loop
                self.speed = self.steps[self.index]["speed"]
        return self.current


def build_model():
    spec = Entity(get_quad_bot_cfg()).spec
    gnd = spec.worldbody.add_geom()
    gnd.name = "ground"
    gnd.type = mujoco.mjtGeom.mjGEOM_PLANE
    gnd.size = [5.0, 5.0, 0.1]
    gnd.rgba = [0.84, 0.87, 0.82, 1.0]
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("init_state").id)
    return model, data


def main():
    parser = argparse.ArgumentParser(description="Plays a routine in the simulation and, with --real, in the robot.")
    parser.add_argument("routine", nargs="?", default="greeting", help="name of a file in loquro_control/routines")
    parser.add_argument("--real", action="store_true", help="send the same commands to the real robot")
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447", help="zenoh bridge of robot.launch.py")
    args = parser.parse_args()

    model, data = build_model()
    controller = RoutineController(args.routine)
    steps_per_command = max(1, round(controller.dt / model.opt.timestep))
    command = list(FIRMWARE_INIT_POSE)

    robot = None
    if args.real:
        from loquro_lab.robot_link import RobotLink
        robot = RobotLink(args.endpoint)
        print("Waiting for the real robot (launch running, green led)...")

    def step(new_command):
        """One command period: the same command to the simulation and to the real robot."""
        data.ctrl[:] = firmware_to_sim_ctrl(new_command)
        if robot is not None:
            robot.send(new_command)
        for _ in range(steps_per_command):
            mujoco.mj_step(model, data)

    print(f'Routine "{args.routine}" from {ROUTINES}')
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            deadline = time.perf_counter()
            while viewer.is_running():
                # without the real robot ready both wait at the current pose, so they stay together
                if robot is None or robot.ready:
                    command = controller.update()
                step(command)
                viewer.sync()
                deadline += controller.dt
                time.sleep(max(0.0, deadline - time.perf_counter()))
    except KeyboardInterrupt:
        pass
    finally:
        if robot is not None:
            print("Real robot back to its initial pose...")
            max_delta = RETURN_SPEED * controller.dt
            while command != list(FIRMWARE_INIT_POSE):
                command = [c + max(-max_delta, min(max_delta, t - c)) for c, t in zip(command, FIRMWARE_INIT_POSE)]
                step(command)
                time.sleep(controller.dt)
            robot.close()


if __name__ == "__main__":
    main()
