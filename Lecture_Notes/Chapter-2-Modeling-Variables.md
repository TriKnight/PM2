---
marp: true
theme: default
paginate: true
math: mathjax
backgroundColor: #fff
markdown:
  attributes: true
---
<style>
section::before {
  content: url('https://vgu.edu.vn/cms-vgu-theme-4/images/cms/vgu_logo.png'); /* Your logo URL */
  width: 20px; /* Adjust size */
  height: auto;
  position: absolute;
  right: 50px;
  top: 20px;
}
</style>

# Chapter 2: Modeling Technical Variables

---

## Learning Objectives

By the end of this chapter, you will:
- Translate physical systems into software variables
- Distinguish between inputs, outputs, internal states, and constraints
- Apply naming conventions, units, and responsibility principles
- Analyze technical variables for real systems
- Create comprehensive Technical Variable Tables
- Document constraints and validation rules

---

## What are Technical Variables?

**Definition:** Named representations of physical quantities in software

### Real-World Example: 1-DOF Servo Motor
```
Physical System          →  Software Variables
├─ Rotor Position             └─ joint_position (radians)
├─ Motor Command              └─ motor_command (range: 0-6.29)
├─ Applied Voltage            └─ pwm_duty_cycle (0-100%)
└─ Joint Constraints          └─ position_limits, velocity_limits
```

### Why Variables Matter
- Bridge physical reality and digital computation
- Enable clear communication between engineers
- Support testing, simulation, and safety validation
- Facilitate maintenance and debugging

---

## Three Categories of Variables

### 1. Inputs (Measured from Environment)
- Data coming **INTO** the software system
- From sensors, user commands, or external systems
- **Example:** `joint_position` (from encoder)

### 2. Outputs (Commands to Environment)  
- Data going **OUT FROM** the software system
- To actuators, displays, or external systems
- **Example:** `motor_command` (to servo driver)

### 3. Internal State (Processing)
- Data maintained within the software
- Intermediate calculations and memory
- **Example:** `velocity_estimate` (calculated from position changes)

---

## Variable Properties: The Four Pillars

### 1. **Naming** - Clear, Unambiguous
- Use intention-revealing names
- Avoid abbreviations unless universal
- Include units in documentation, not in name

**Good Examples:**
```
joint_position_rad        # Position in radians
motor_command_normalized  # Normalized 0-1 range
encoder_count_raw         # Raw encoder ticks
```

---
## Variable Properties: The Four Pillars (Cont.)

**Avoid:**
```
x, y, temp          # Unclear semantics
pos1, pos2          # Ambiguous units
val, data, stuff    # No meaning
```

---

## Variable Properties: Units

### 2. **Units** - Explicit and Consistent
- Specify all measurement units clearly
- Convert at boundaries between systems
- Document conversion factors

### SI Units (Recommended)
- **Position/Distance:** meters (m), radians (rad)
- **Velocity:** m/s, rad/s
- **Acceleration:** m/s², rad/s²
- **Force/Torque:** Newtons (N), Newton-meters (Nm)
- **Time:** seconds (s)

### Example Conversion
```python
# Position from encoder (ticks) → radians
TICKS_PER_REVOLUTION = 2048
position_rad = (encoder_ticks % TICKS_PER_REVOLUTION) * 2π / TICKS_PER_REVOLUTION
```

---

## Variable Properties: Constraints

### 3. **Constraints** - Valid Operating Range
- Define minimum and maximum values
- Physical limits (cannot move beyond mechanical stops)
- Software safety limits (conservative thresholds)
- Validation rules for each variable

---

### Servo Motor Example
```python
class Motor:
    def __init__(self):
        # Position: 0 to 2π radians (full rotation)
        self.min_position = 0.0      # radians
        self.max_position = 2 * π    # radians
        self.command = 0.0           # current command
    
    def set_command(self, value: float) -> None:
        # Constraint: command must be in valid range
        if not (0.0 <= value <= 6.29):  # 2π ≈ 6.29
            raise ValueError(f"Motor command out of range: {value}")
        self.command = value
```

---

## Variable Properties: Responsibility

### 4. **Responsibility** - Clear Ownership
- One variable per concept
- One place where variable is updated
- Clear dependencies between variables

### Separation of Concerns
```python
class MotorController:
    def __init__(self, motor: Motor, simulation):
        self.motor = motor                    # Output: command
        self.simulation = simulation          # Receives motor command
        self.joint_position = 0.0             # State: position feedback
        self.target_position = 0.0            # Input: user request
        self.position_error = 0.0             # State: difference
```

---

## A Practical Example: 1-DOF Motor System

### System Overview

<p align="center">
  <img src="./figs/controller_software_chap2.png" width="50%">
</p>
---

## Input Variables Analysis

### Motor Command (User Input)
| Property | Description |
|----------|-------------|
| **Name** | `motor_command` |
| **Type** | float |
| **Units** | radians (rad) |
| **Range** | 0.0 to 2π (0 to 6.29) |
| **Source** | Controller logic |
| **Update Rate** | 100 Hz (10 ms) |
| **Constraint** | Must be non-negative, ≤ 2π |

---

### Design Rationale
- **Why normalized?** Allows controller to specify absolute position (0 = home, π = opposite side)
- **Why bounded?** Prevents commanding beyond joint limits
- **Why radians?** Standard SI unit for angular quantities

---

## Output Variables Analysis

### Joint Position (Measurement Output)
| Property | Description |
|----------|-------------|
| **Name** | `joint_position` |
| **Type** | float |
| **Units** | radians (rad) |
| **Range** | 0.0 to 2π |
| **Source** | Physics simulation (encoder emulation) |
| **Measurement** | Sampled at 1 kHz simulation rate |
| **Accuracy** | ±0.01 rad (physics precision) |
| **Resolution** | 0.001 rad (3 decimal places) |

### Design Rationale
- **Why float?** Continuous measurement, not discrete steps
- **Why same units as command?** Direct comparison for error calculation
- **Why high precision?** Enables accurate control feedback

---

## Internal State Variables

### Position Error
```python
position_error = target_position - joint_position
```

**Purpose:** Error signal for control algorithms
- **Name:** `position_error`
- **Units:** radians
- **Range:** -2π to +2π (can overshoot in both directions)
- **Updated:** Every control cycle
- **Use:** Input to PID controller

---

### Velocity Estimate
```python
velocity = (position_new - position_old) / dt
```

**Purpose:** Estimate joint velocity for damping
- **Name:** `joint_velocity_estimate`
- **Units:** rad/s
- **Range:** -12 to +12 rad/s (physical limits)
- **Updated:** Every control cycle
- **Use:** Velocity feedback, damping terms

---

## Code Example: Motor Class

```python
class Motor:
    """Software representation of 1-DOF motor actuator.
    
    Position Control. Command range: [0, 6.29] radians.
    """
    
    MIN_COMMAND = 0.0      # radians
    MAX_COMMAND = 6.29     # ≈ 2π radians
    
    def __init__(self) -> None:
        self.command = 0.0  # Current command [rad]
    
    def set_command(self, value: float) -> None:
        """Set motor command with validation.
        
        Args:
            value: Commanded position in radians
            
        Raises:
            ValueError: If command is out of range
        """
        if not (self.MIN_COMMAND <= value <= self.MAX_COMMAND):
            raise ValueError(
                f"Motor command out of range: {value}. "
                f"Expected [{self.MIN_COMMAND}, {self.MAX_COMMAND}]"
            )
        self.command = value
    
    @property
    def command(self) -> float:
        """Get current motor command [rad]."""
        return self._command
```

---

## Code Example: Simulation Bridge

```python
class MujocoSimulation:
    """Wraps MuJoCo as an engineering software component."""
    
    def __init__(self, modelpath: str, motor: Motor) -> None:
        self.model = mujoco.MjModel.from_xml_path(modelpath)
        self.data = mujoco.MjData(self.model)
        self.motor = motor
        self.time = 0.0
    
    def step(self) -> None:
        """Advance simulation by one timestep.
        
        Key responsibilities:
        1. Read motor command from motor object
        2. Apply command to simulation actuator
        3. Step physics forward
        4. Update simulation time
        """
        self.data.ctrl[0] = self.motor.command
        mujoco.mj_step(self.model, self.data)
        self.time += self.model.opt.timestep
    
    @property
    def joint_position(self) -> float:
        """Get current joint position in radians."""
        return float(self.data.qpos[0])
```

---

## Creating a Technical Variable Table

### Template Structure

| Variable Name | Type | Units | Range | Source | Update Rate | Constraints | Responsibility |
|---|---|---|---|---|---|---|---|
| `motor_command` | float | rad | [0.0, 6.29] | Controller | 100 Hz | Must validate range | Motor.set_command() |
| `joint_position` | float | rad | [0.0, 6.29] | Simulation | 1 kHz | Measured output | Simulation.step() |
| `position_error` | float | rad | [-6.29, 6.29] | Controller | 100 Hz | None | Controller.compute() |

---

## Lab 1: System Analysis: Analyze 1-DOF Motor System

**Given Files:**
- `motor.py` - Motor software component
- `simulation.py` - Physics simulation wrapper
- `main.py` - Controller logic

**Requirements:**
1. Identify all input variables
2. Identify all output variables
3. Identify all internal state variables
4. Document units and ranges
5. List all constraints and validation rules

---

### Expected Outputs
```
INPUTS:
  - motor_command [rad] ∈ [0, 6.29]

OUTPUTS:
  - joint_position [rad] ∈ [0, 6.29]

INTERNAL STATES:
  - simulation_time [s]
  - position_error [rad]
  - ...
```

---

## Lab 2: Technical Variable Table Creation

### Step 1: Enumerate All Variables
```
From motor.py:
  - command (input to actuator)

From simulation.py:
  - time (internal state)
  - joint_position (output)
  - ctrl[0] (command transfer)

From controller:
  - target_position (input)
  - measured_position (output)
  - error (internal)
```
---

### Step 2: Document Each Variable
For each variable, specify:
- What does it represent?
- Where does it come from?
- What are valid ranges?
- How often is it updated?
- What validates it?

---

## Lab 3: Constraint Validation

### Write Validation Rules

```python
# Motor Command Validation
def validate_motor_command(value: float) -> bool:
    """Check if motor command is valid."""
    MIN_COMMAND = 0.0
    MAX_COMMAND = 6.29
    
    # Rule 1: Must be numeric
    if not isinstance(value, (int, float)):
        return False, "Command must be numeric"
    
    # Rule 2: Must be in range
    if not (MIN_COMMAND <= value <= MAX_COMMAND):
        return False, f"Command out of range: {value}"
    
    # Rule 3: Must be finite (no NaN, inf)
    if not math.isfinite(value):
        return False, "Command must be finite"
    
    return True, "Valid"
```

---

## Naming Conventions for Mechatronic Systems

### Recommended Patterns

**Sensor Readings (Inputs):**
```
joint_position          # Primary feedback
sensor_temperature      # From sensor
encoder_count          # Raw encoder ticks (not normalized)
```

**Commands (Outputs):**
```
motor_command          # To actuator
pwm_duty_cycle         # PWM signal
gripper_open_flag      # Boolean command
```
---

**Internal Calculations (States):**
```
position_estimate      # Filtered/estimated value
velocity_filtered      # Processed measurement
position_error         # Error signal (setpoint - actual)
```

**Physical Limits:**
```
joint_position_min     # Lower mechanical limit
joint_velocity_max     # Maximum safe velocity
temperature_warning    # Safety threshold
```

---

## Units and Conversions

### Standard SI Units in Robotics
```
Linear:  meter (m)           Time:   second (s)
         millimeter (mm)            millisecond (ms)
         
Angular: radian (rad)        Velocity: meter/second (m/s)
         degree (°)                   radian/second (rad/s)

Force:   Newton (N)          Torque: Newton-meter (Nm)
         kilonewton (kN)
```
---

### Conversion Functions
```python
def degrees_to_radians(deg: float) -> float:
    return deg * π / 180

def rpm_to_rad_per_sec(rpm: float) -> float:
    return rpm * 2π / 60

def encoder_ticks_to_radians(
    ticks: int,
    resolution: int = 2048  # ticks per revolution
) -> float:
    return (ticks / resolution) * 2π
```

---

## Common Pitfalls in Variable Modeling

### ❌ Bad: Mixing Units
```python
position = 45              # Is this degrees or radians?
velocity = 10              # rad/s or RPM?
temperature = 72           # Celsius or Fahrenheit?
```

### ✅ Good: Explicit Units
```python
position_rad = 0.785       # π/4 radians
velocity_rad_per_sec = 1.5
temperature_celsius = 72
```

### ❌ Bad: Unclear Constraints
```python
def set_torque(value):
    self.torque = value    # What's the valid range?
```

### ✅ Good: Documented Constraints
```python
def set_torque(self, value: float) -> None:
    """Set motor torque command.
    
    Args:
        value: Torque in Newton-meters, range [-10, 10]
        
    Raises:
        ValueError: If value outside valid range
    """
    MIN_TORQUE = -10.0  # Nm
    MAX_TORQUE = 10.0   # Nm
    if not (MIN_TORQUE <= value <= MAX_TORQUE):
        raise ValueError(f"Torque out of range: {value}")
    self._torque = value
```

---

## Responsibility Assignment

### Single Responsibility Principle for Variables

**Good Assignment:**
```python
class Motor:
    """Responsible for motor command."""
    def set_command(self, value: float): ...

class Simulation:
    """Responsible for joint position."""
    @property
    def joint_position(self) -> float: ...

class Controller:
    """Responsible for position error calculation."""
    def compute_error(self, target, actual): ...
```

**Bad Assignment (Shared Responsibility):**
```python
class Motor:
    def set_command(self, value: float):
        self.command = value
        self.position = self.compute_position()  # ❌ Two responsibilities!
```

---

## Technical Variable Table Template

### For Your Servo Motor System

| Variable | Type | Units | Min | Max | Source | Rate | Safety Check | Owner |
|----------|------|-------|-----|-----|--------|------|--------------|-------|
| `motor_command` | float | rad | 0.0 | 6.29 | Controller | 100 Hz | In-range check | Motor |
| `joint_position` | float | rad | 0.0 | 6.29 | Simulator | 1 kHz | None | Simulation |
| `position_error` | float | rad | -6.29 | 6.29 | Controller | 100 Hz | None | Controller |
| `joint_velocity` | float | rad/s | -12 | 12 | Controller | 100 Hz | None | Controller |
| `sim_time` | float | s | 0 | ∞ | Simulator | 1 kHz | Monotonic | Simulation |

---

## Deliverable: Technical Variable Table

### Approval Checklist

Before implementation, your table must include:

- [ ] **All input variables** listed with sources
- [ ] **All output variables** listed with destinations
- [ ] **All internal states** with roles and calculations
- [ ] **Units** explicitly specified (SI recommended)
- [ ] **Ranges** defined (min and max values)
- [ ] **Update rates** documented (Hz or timing)
- [ ] **Constraints** listed (validation rules)
- [ ] **Responsibility** assigned (who manages each)
- [ ] **Safety checks** specified (if applicable)
- [ ] **Review** by instructor or peer

### Signature Block
```
Variable Table Version: 1.0
System: 1-DOF Servo Motor Position Control
Date: [DATE]
Author: [NAME]
Reviewed By: [INSTRUCTOR]
Approval: ☐ Approved  ☐ Needs Revision
```

---

## Real-World Example: Servo Motor Complete Table

### Inputs
| Name | Type | Units | Range | Source | Comments |
|------|------|-------|-------|--------|----------|
| motor_command | float | rad | [0, 6.29] | Controller logic | Absolute position command |
| controller_active | bool | - | {T,F} | Safety supervisor | Enable/disable flag |

### Outputs
| Name | Type | Units | Range | Dest | Comments |
|------|------|-------|-------|------|----------|
| joint_position | float | rad | [0, 6.29] | Visualization, feedback | From encoder simulation |
| system_state | enum | - | {IDLE, MOVING, ERROR} | UI display | Current operating mode |

### Internal States
| Name | Type | Units | Range | Purpose | Update |
|------|------|-------|-------|---------|--------|
| position_error | float | rad | [-6.29, 6.29] | Error feedback to PID | Every cycle |
| velocity_estimate | float | rad/s | [-20, 20] | Damping & smoothing | Every cycle |
| command_buffer | float | rad | [0, 6.29] | Smooth transitions | As needed |

---

## Best Practices Summary

### Variable Modeling Checklist
1. **Name** - Use clear, descriptive names
2. **Type** - Specify data type explicitly
3. **Units** - Always include measurement units
4. **Range** - Define min/max values
5. **Source** - Identify where data comes from
6. **Update Rate** - Specify frequency/timing
7. **Constraints** - Document validation rules
8. **Responsibility** - Assign clear ownership
9. **Documentation** - Explain semantic meaning
10. **Review** - Approve before coding

### Implementation Order
1. List all variables
2. Classify (input/output/state)
3. Document each property
4. Create validation rules
5. **Get approval** ← Critical step!
6. Then implement code

---

## What's Next?

### Upcoming Topics
- Chapter 3: Control Algorithms 

### Key Principle
> "A well-defined variable table is the contract between the physical system and the software. Get it right, and everything else follows." - Timothy Repp, *Robotics for Programmers*

---

## Questions for Review

### Before Implementation, Verify:
1. What inputs does your system need?
2. What outputs does it produce?
3. What internal calculations occur?
4. What are valid ranges for each variable?
5. Who is responsible for updating each?
6. What validates constraints?
7. Are all units consistent?
8. Is naming clear and unambiguous?

---

### Assignment
Create a complete Technical Variable Table for:
- **System:** 1-DOF Motor Position Control
- **Format:** Table with 9 columns (see template)
- **Deliverable:** Markdown or PDF table
- **Approval:** Get instructor sign-off before coding

---

## References & Further Reading

### From Your Books:
- **"Robotics for Programmers"** - Chapter 2: Software Architecture
  - APIs and abstraction layers
  - Sensor/actuator interfaces
  
- **"Clean Code"** - Chapter 2: Meaningful Names
  - Intention-revealing names
  - Avoid disinformation

- **"Fluent Python"** - Chapter 8: Type Hints
  - Type annotations for clarity
  - Documentation through types

### Additional Resources:
- SI Units (BIPM): https://www.bipm.org/en/measurement-units/
- Robot Operating System (ROS) Style Guide
- IEEE Standard 1588 for time synchronization

---

## Summary

### Key Takeaways
1. **Variables bridge physics and software** - They are the interface
2. **Naming matters** - Clear names prevent bugs
3. **Units must be explicit** - Avoid confusion at boundaries
4. **Constraints are safety** - Validation prevents damage
5. **Responsibility prevents chaos** - One owner per variable
6. **Documentation enables others** - Future you will thank present you
7. **Approval before code** - Catch design issues early

---

### Remember
> *"Engineering is about precision. Sloppy variable definitions lead to sloppy systems. Take time upfront to define your variables correctly."* - Robert C. Martin, *Clean Code*

---

## Lab Assignment Checklist

### Deliverable 1: Variable Analysis
- [ ] Identified 8+ variables in the servo system
- [ ] Classified each as input/output/state
- [ ] Listed units for all variables
- [ ] Documented ranges

### Deliverable 2: Technical Variable Table
- [ ] Created table with all 9 required columns
- [ ] Included validation constraints
- [ ] Assigned responsibility
- [ ] Got instructor review

### Deliverable 3: Constraint Code
- [ ] Wrote validation functions
- [ ] Tested edge cases
- [ ] Documented assumptions

**Due:** Before starting Chapter 3 (Control Algorithms)