from system_info import SystemInfo
from motor import Motor
from simulation import MujocoSimulation
from visualization import Visualization
import math

def main() -> None:
    system = SystemInfo("1-DOF Motor Simulation (Position Control)", "1.1")
    print(system.describe())

    motor = Motor()
    vis = Visualization()

    def controller(time_s: float, motor: Motor) -> None:  
        # Sinusoidal command around pi, amplitude 1.0
        command = 1.0
        motor.set_command(command)
        vis.log(time_s, command, sim.joint_position())

    sim = MujocoSimulation(
        model_path="chapter1/assets/motor_1dof.xml",
        motor=motor
    )

    # Run simulation for 20 seconds
    sim.run_sim(duration=5.0, controller=controller, viewer=True)

    print("Final joint position:", sim.joint_position())
    
    # Plot results
    vis.plot("position_control_plot.png")

if __name__ == "__main__":
    main()
