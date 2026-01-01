
import matplotlib.pyplot as plt
from typing import List

class Visualization:
    """
    Visualizes the simulation data (Command vs Output).
    """

    def __init__(self) -> None:
        self.times: List[float] = []
        self.commands: List[float] = []
        self.outputs: List[float] = []

    def log(self, time: float, command: float, output: float) -> None:
        """
        Log a data point.
        """
        self.times.append(time)
        self.commands.append(command)
        self.outputs.append(output)

    def plot(self, save_path: str = "simulation_plot.png") -> None:
        """
        Plot the logged data and save to a file.
        """
        plt.figure(figsize=(10, 6))
        plt.plot(self.times, self.commands, label='Command (Position)', linestyle='--')
        plt.plot(self.times, self.outputs, label='Output (Joint Position)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Joint position (rad)')
        plt.title('Motor Simulation (Position Control): Command vs Output')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")
        plt.close()
