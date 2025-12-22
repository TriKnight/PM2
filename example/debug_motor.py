
import mujoco
import time
import math

model_path = "chapter1/assets/motor_1dof.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Set command to 1.0
target_pos = 1.0
data.ctrl[0] = target_pos

print(f"Model KP: {model.actuator_gainprm[0, 0]}")
print(f"Joint Damping: {model.dof_damping[0]}")
print("Running simulation...")

for i in range(2000):
    mujoco.mj_step(model, data)
    if i % 500 == 0:
        print(f"Step {i}: Pos={data.qpos[0]:.4f}, Ctrl={data.ctrl[0]:.4f}, ActForce={data.qfrc_actuator[0]:.4f}")
        print(f"  Passive: {data.qfrc_passive[0]:.4f}")
        print(f"  Bias: {data.qfrc_bias[0]:.4f}")
        print(f"  QAcc: {data.qacc[0]:.4f}")
        print(f"  Constraint Force: {data.qfrc_constraint[0]:.4f}")
        print(f"  Friction Loss: {model.dof_frictionloss[0]:.4f}")

print(f"Final Pos: {data.qpos[0]}")
