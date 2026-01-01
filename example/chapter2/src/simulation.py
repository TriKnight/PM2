import time
import math
import mujoco
import mujoco.viewer
from motor import Motor

class MujocoSimulation:
    """
    Wraps MuJoCo as an engineering software component.
    """

    def __init__(self, model_path: str, motor: Motor) -> None:
        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._motor = motor
        self._time = 0.0

    def step(self) -> None:
        self._data.ctrl[0] = self._motor.command()
        mujoco.mj_step(self._model, self._data)
        self._time += self._model.opt.timestep

    def joint_position(self) -> float:
        return float(self._data.qpos[0])

    def run_sim(self, duration: float, controller=None, viewer: bool = True) -> None:
        """
        Run the simulation for a fixed duration, optionally with a viewer.

        :param duration: Simulation time in seconds
        :param controller: Optional callback function (time, motor) -> None
        :param viewer: Enable MuJoCo viewer if True
        """
        timestep = self._model.opt.timestep
        steps = int(duration / timestep)

        if viewer:
            import mujoco.viewer
            import time

            with mujoco.viewer.launch_passive(self._model, self._data) as v:
                for _ in range(steps):
                    if not v.is_running():
                        break

                    if controller:
                        controller(self._time, self._motor)

                    self.step()
                    v.sync()
                    time.sleep(timestep)
        else:
            for _ in range(steps):
                if controller:
                    controller(self._time, self._motor)
                self.step()

