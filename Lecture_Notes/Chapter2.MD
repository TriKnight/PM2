---
marp: true
title: Chapter 2 – Modeling Technical Variables
author: Programming Method 2
theme: default
paginate: true
---

# Chapter 2  
## Modeling Technical Variables  
### From Physical Systems to Software

---

## Chapter Objectives

After completing this chapter, students will be able to:

- Translate a physical system into software variables
- Distinguish inputs, outputs, and internal states
- Assign correct units and data types
- Define responsibility for each variable
- Produce a Technical Variable Table before coding

---

## Why Modeling Variables Matters

In engineering software:

> **Wrong variables = wrong system behavior**

Good variable modeling:
- Prevents design errors
- Clarifies system responsibilities
- Enables testing and verification
- Reduces code complexity

---

## Physical System vs Software System

### Physical world
- Sensors measure reality
- Actuators affect reality
- Physics governs behavior

### Software world
- Variables represent reality
- Logic operates on variables
- Software enforces constraints

---

## Core Principle

> **Software never sees physics directly.  
> It only sees variables.**

Correct variables are the **bridge** between:
- Physical behavior
- Control logic

---

## Categories of Technical Variables

All variables fall into one of three categories:

1. Inputs
2. Outputs
3. Internal states

Every variable must belong to **exactly one** category.

---

## Input Variables

### Definition

Input variables represent information coming **into** the software.

Typically from:
- Sensors
- User commands
- External systems

---

## Input Variable Examples

Examples:
- Button pressed (boolean)
- Temperature (float, °C)
- Joint position (float, rad)
- Tank level (float, meters)

Key property:
> Inputs are **read-only** for control logic

---

## Output Variables

### Definition

Output variables represent commands sent **out** of the software.

Typically to:
- Actuators
- Displays
- External controllers

---

## Output Variable Examples

Examples:
- Motor torque command (float, Nm)
- Valve open percentage (0–100%)
- LED on/off (boolean)
- Target position (rad)

Key property:
> Outputs are **written by software**

---

## Internal State Variables

### Definition

Internal states represent the **software’s memory**.

They describe:
- Current mode
- History
- Progress in time

---

## Internal State Examples

Examples:
- Current FSM state
- Timer values
- Error flags
- Initialization status

Key property:
> Internal states are **not measured**  
> and **not actuated**

---

## Common Beginner Mistake

❌ Treating everything as an input  
❌ Mixing state and output  
❌ Using actuator command as state  

Correct rule:

> **State describes what the system *is*,  
> Output describes what the system *does*.**

---

## Variable Naming Rules

Good variable names must be:

- Descriptive
- Unambiguous
- Consistent
- Domain-specific

---

## Naming Examples

❌ Bad:
- `x`
- `value`
- `temp1`

✅ Good:
- `motor_position_rad`
- `tank_level_m`
- `door_is_open`

---

## Units Are Mandatory

Every technical variable must have:

- A physical unit
- A documented range

Examples:
- Position → radians
- Velocity → rad/s
- Level → meters
- Temperature → °C

---

## Why Units Matter

Without units:
- Bugs go undetected
- Control laws fail
- Systems behave unpredictably

Engineering rule:

> **If the unit is unclear, the variable is invalid.**

---

## Responsibility Assignment

Every variable must answer:

> **Who owns this variable?**

Ownership can be:
- Sensor
- Actuator
- Controller
- State machine

---

## Responsibility Example

| Variable | Owner |
|-------|------|
Button state | Sensor |
Motor command | Controller |
FSM state | State machine |
Timer | Controller |

---

## Example System: Automatic Door

Physical components:
- Button
- Motor
- Door
- Controller

We must model this **before coding**.

---

## Example: Input Variables (Door)

- `button_pressed : bool`
- `door_position_rad : float`
- `limit_switch_open : bool`

---

## Example: Output Variables (Door)

- `motor_torque_cmd : float`
- `motor_enable : bool`

---

## Example: Internal States (Door)

- `door_state : enum`
- `open_timer_s : float`
- `fault_flag : bool`

---

## Technical Variable Table

All variables must be written in a table **before coding**.

This table is a **mandatory design artifact**.

---

## Technical Variable Table – Template

| Name | Category | Type | Unit | Range | Owner |
|----|--------|------|------|-------|------|
| | | | | | |

---

## Example Technical Variable Table

| Name | Category | Type | Unit | Range | Owner |
|----|--------|------|------|-------|------|
| button_pressed | Input | bool | – | {0,1} | Sensor |
| motor_torque_cmd | Output | float | Nm | [-1,1] | Controller |
| door_state | State | enum | – | CLOSED…OPEN | FSM |

---

## Design Rule (Very Important)

> **No variable table → No code allowed**

This enforces:
- Thinking before coding
- Clean architecture
- Professional workflow

---

## Laboratory Session

### Objective

Model technical variables for a **real physical system**.

Examples:
- Automatic door
- Water tank
- Motor + arm
- Conveyor belt

---

## Laboratory Tasks

Students must:

1. Choose a system
2. Identify all inputs
3. Identify all outputs
4. Identify all internal states
5. Assign units and owners
6. Create a variable table

---

## What Is NOT Allowed in the Lab

❌ Writing control logic  
❌ Writing FSMs  
❌ Writing Python code  

This lab is **pure modeling**.

---

## Deliverable

Each student must submit:

- A complete Technical Variable Table
- Clear variable names
- Correct units and ranges
- Ownership assignment

---

## Approval Rule

> **The instructor must approve the variable table  
> before the student is allowed to code.**

This mirrors real engineering practice.

---

## Common Grading Criteria

- Correct categorization
- Correct units
- No missing variables
- Clear responsibility assignment
- No redundant variables

---

## Key Takeaways

- Software variables represent physical reality
- Inputs, outputs, and states must be separated
- Units and ranges are mandatory
- Variable modeling comes before coding

---

## Next Chapter

**Chapter 3 – System Decomposition and Class Responsibility**

Focus:
- Mapping variables to classes
- Responsibility-driven design
- Preparing for FSM implementation
