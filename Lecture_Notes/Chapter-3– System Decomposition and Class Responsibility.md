---
marp: true
theme: default
paginate: true
math: mathjax
backgroundColor: #fff
---

# Chapter 3: System Decomposition and Class Responsibility

---

## Learning Objectives

By the end of this chapter, you will:
- Decompose a mechatronic system into functional components
- Identify candidate classes from system components
- Apply the Single Responsibility Principle (SRP) to class design
- Document class responsibilities before coding
- Prepare a Class Responsibility Document for your project

---

## Motivation

- Mechatronic software becomes unmaintainable when:
  - “God objects” do everything
  - Control logic, I/O, and logging are mixed
  - Responsibilities are not explicit
- Good decomposition:
  - Makes systems understandable and testable
  - Allows simulation and real hardware to share architecture
  - Supports long-term evolution of the system

---

## From Requirements to Structure

### Starting Point

You are given:
- Physical system (e.g., 1‑DOF servo motor in MuJoCo)
- High-level goals (track reference, handle errors, log data)
- Technical variables (from Chapter 2)

### Task

Turn this into:
- Components with clear responsibilities
- Classes with clear APIs
- A documented structure before implementation

---

## Functional Decomposition

### Idea

Break the system into *what it must do* before deciding *how* to implement it.

Typical functions:
- Read sensors
- Compute control actions
- Command actuators
- Manage modes/states
- Log and visualize data

---

## Example: 1‑DOF Motor System

### Functional Blocks

- **Sensor** – Provide joint position
- **Controller** – Compute motor command
- **Actuator** – Apply command
- **State Machine** – Manage modes (IDLE/RUNNING/ERROR)
- **Simulation** – Emulate physics
- **Logger/Visualization** – Record and display behavior

You should draw these as a block diagram **before** thinking of classes.

---

## From Functions to Components

### Component View

- Group related functions into components:
  - Sensor component
  - Controller component
  - Actuator component
  - FSM component
  - Simulation component
  - Logging/visualization component

---

## Each component:
- Has a clear purpose
- Interacts with a limited set of neighbors
- Encapsulates related technical variables

---

## Identifying Candidate Classes

### Noun Extraction

From the problem description:
- Motor, sensor, actuator
- Controller, state machine
- Simulation, visualization
- Logger, system info, configuration

These nouns are candidates for:
- Classes (e.g., `Motor`, `Controller`)
- Interfaces/abstract base classes (e.g., `SensorInterface`)

---

## Class Identification Heuristics

A concept becomes a class when:
- It owns state over time  
  (e.g., motor command, joint position, current mode)
- It has a clear interface  
  (e.g., `read()`, `step()`, `set_command()`)
- It is used in multiple places  
  (e.g., sensors accessed by several controllers)
- It corresponds to a real physical or conceptual entity

---

## Types of Classes

### Entity-Like Classes

- Represent physical or logical entities:
  - `Motor`, `Servo`, `JointSensor`
  - `MujocoSimulation`
- Own technical variables and enforce constraints

### Service/Logic Classes

- Represent processes:
  - `Controller`
  - `StateMachine`
  - `Logger`, `Visualizer`
- Coordinate entities and implement behavior

---

## Example: Component to Class Mapping

| Component     | Candidate Classes                    |
|--------------|--------------------------------------|
| Sensor       | `SensorInterface`, `SimulatedSensor` |
| Actuator     | `ActuatorInterface`, `Motor`         |
| Controller   | `PositionController`, `PidController`|
| State Machine| `StateMachine`, `SystemMode` enum    |
| Simulation   | `MujocoSimulation`                   |
| Logging/UI   | `Logger`, `Visualizer`               |

---

## Single Responsibility Principle (SRP)

> “A class should have one and only one reason to change.”

### Why SRP Matters

- Easier to understand
- Easier to test
- Easier to replace or extend
- Reduces coupling between unrelated concerns

---

## SRP in Mechatronic Systems

Good separation:

- `Motor`  
  → Owns and validates motor command

- `MujocoSimulation`  
  → Advances physics based on commands

- `Controller`  
  → Computes control commands from reference and feedback

- `StateMachine`  
  → Manages operating modes

- `Logger` / `Visualizer`  
  → Only observe and present data

---

## SRP Violations: “God Object” Anti-Pattern

Bad example:

- One class that:
  - Reads sensors
  - Computes control
  - Directly manipulates the physics engine
  - Logs data
  - Handles UI

---

Issues:
- Many reasons to change
- Hard to test in isolation
- Difficult to reuse or simulate

---

## SRP in Practice

When designing a class:

Ask:
- “What is this class **for**?”
- “What technical variables does it own?”
- “Which other classes should it **not** depend on?”

If you can’t summarize the responsibility in 1–2 sentences:
- The class is probably doing too much.

---

## Linking Variables to Classes

From Chapter 2, each technical variable should:

- Have a single **owning class**
- Be updated in one place
- Be read by well-defined collaborators

Examples:
- `motor_command` → owned by `Motor`  
- `joint_position` → owned by `MujocoSimulation`  
- `position_error` → owned by `Controller`  

---

## Example: Ownership Table

| Variable           | Owner Class        |
|--------------------|--------------------|
| `motor_command`    | `Motor`            |
| `joint_position`   | `MujocoSimulation` |
| `position_error`   | `Controller`       |
| `system_mode`      | `StateMachine`     |
| `log_entries`      | `Logger`           |

This mapping must be consistent across design and implementation.

---

## Dependency Direction

Good practice:

- Controllers depend on *interfaces*, not concrete implementations
  - `SensorInterface`, `ActuatorInterface`

- High-level policies do not know if they are using:
  - MuJoCo simulation
  - Real hardware driver

This supports:
- Simulation first
- Later deployment to real systems without redesign

---

## Implementation Examples

The following slides show how these concepts map to Python code from our `chapter2/src` examples.

---

### 1. Entity Class: `Motor`

The `Motor` class is an "Entity". It owns state (`_command`) and validates inputs. It does **not** decide what the command is (that's the Controller's job).

```python
class Motor:
    def __init__(self) -> None:
        self._command = 0.0

    # Responsibility: Guard state integrity
    def set_command(self, value: float) -> None:
        if not 0.0 <= value <= 6.29:
            raise ValueError(f"Motor command out of range: {value}")
        self._command = value

    def command(self) -> float:
        return self._command
```

---

### 2. Single Responsibility: `Visualization`

The `Visualization` class handles **only** data logging and plotting. It doesn't know about physics or control rules.

```python
class Visualization:
    def __init__(self) -> None:
        self.times = []
        self.commands = []
        self.outputs = []

    def log(self, time: float, command: float, output: float) -> None:
        self.times.append(time)
        self.commands.append(command)
        self.outputs.append(output)

    def plot(self, save_path: str) -> None:
        # ... plotting code using matplotlib ...
        plt.savefig(save_path)
```

---

### 3. Component Interaction: `MujocoSimulation`

The simulation component depends on the `Motor` entity. It applies the motor's command to the physics engine.

```python
class MujocoSimulation:
    def __init__(self, model_path: str, motor: Motor) -> None:
        # ...
        self._motor = motor

    def step(self) -> None:
        # Read from Motor entity
        self._data.ctrl[0] = self._motor.command()
        
        # Advance physics
        mujoco.mj_step(self._model, self._data)
```

---

### 4. Controller Logic (in `main.py`)

The control logic is separated from the hardware (Motor) and the environment (Simulation).

```python
    # Controller function: Pure logic
    def controller(time_s: float, motor: Motor) -> None:  
        command = 2.0 # Simple fixed command
        motor.set_command(command)
        # Log data to Visualizer
        vis.log(time_s, command, sim.joint_position())

    # Wiring it all together
    sim.run_sim(duration=5.0, controller=controller)
```


### Goal

Create a diagram that shows:
- All major components/classes
- Their responsibilities
- Their interactions

### Steps

1. Start from a functional block diagram
2. Turn each block into one or more classes
3. Write 3–5 bullets per class:
   - What it does
   - What it does **not** do
   - Which variables it owns

---

## Lab Instructions (Part 1)

1. **Choose System Scenario**  
   - Use the 1‑DOF motor control system from previous labs

2. **Draw Components**  
   - Sensor, controller, actuator, state machine, simulation, logger, visualization

3. **Label Interactions**  
   - Data types (e.g., `joint_position`, `motor_command`, `system_state`)

---

## Lab Instructions (Part 2)

4. **Assign Responsibilities**  
   For each component:
   - Purpose (1–2 sentences)
   - Main technical variables owned
   - Key operations (method names only)

5. **Check SRP**  
   - Does any component “do everything”?
   - Can you split it into smaller, focused parts?

You will refine this into the Class Responsibility Document.

---

## Deliverable: Class Responsibility Document

- Written document (Markdown, LaTeX, or PDF)
- Must be **approved before major coding**
- Forms the contract for:
  - Which classes exist
  - What each class is responsible for
  - How they interact

Next slides show a template you can reuse.

---

## Class Responsibility Document – Overview

Recommended sections:

1. System Overview  
2. Component Diagram  
3. Class Responsibility Table  
4. Ownership of Technical Variables  
5. SRP and Design Rationale  
6. Open Questions / Assumptions

---

## System Overview (Section 1)

Briefly describe:

- Physical system (e.g., 1‑DOF servo motor on MuJoCo)
- Control goal
  - Track reference position
  - Handle errors and safety limits
- Key modes
  - IDLE / RUNNING / ERROR

---

## Component Diagram (Section 2)

Include:

- Boxes for key components/classes:
  - Sensor, Controller, Motor, Simulation, StateMachine, Logger, Visualizer
- Arrows showing:
  - Data flow (what + direction)
  - Important dependencies

You may draw this by hand and scan it, or use any diagram tool.

---

## Class Responsibility Table (Section 3)

For each class, capture:

- Name
- Responsibility (1–3 sentences)
- Owned technical variables
- Public interface (methods)
- Collaborators (other classes it uses)

Use a table for consistency.

---

## Ownership of Technical Variables (Section 4)

Create a mapping:

- Each variable from the Technical Variable Table
- Exactly one owning class
- No hidden copies without justification

This ensures consistency between Chapter 2 and Chapter 3 artifacts.

---

## SRP and Design Rationale (Section 5)

Explain:

- Where you split responsibilities (e.g., Controller vs StateMachine)
- Any deliberate compromises:
  - Combined responsibilities kept for simplicity
  - Future refactoring ideas

This trains you to reason about design trade-offs explicitly.

---

## Open Questions and Assumptions (Section 6)

List:

- Unclear requirements
- Assumptions about sensors, actuators, simulation
- Interfaces that might change later

This section is important for discussions with instructors and teammates.

---

## Summary

Key ideas from this chapter:

1. Decompose the system by functions, not by code first
2. Identify candidate classes from components and nouns
3. Apply Single Responsibility Principle to each class
4. Map technical variables to unique class owners
5. Document class responsibilities before coding

The Class Responsibility Document is a **formal design artifact** that guides your implementation.

---

## Discussion & Questions

Topics for discussion:

- Borderline cases: where to put certain responsibilities
- How fine-grained classes should be at this level
- How to keep design simple but extensible

Use your draft documents as the basis for lab discussion.

---


