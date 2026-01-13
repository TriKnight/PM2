---
marp: true
title: Chapter 4 – Automata and Finite State Machines
author: Programming Method 2
theme: default
paginate: true
---

# Chapter 4  
## Automata and Finite State Machines (FSM)  
### Deterministic Control Logic

---

## Chapter Objectives

After completing this chapter, students will be able to:

- Explain automata and FSM concepts
- Identify states, events, transitions, and guards
- Design deterministic control behavior
- Represent control logic using state diagrams
- Review and validate FSM designs before implementation

---

## Why FSMs Are Critical in Mechatronic Systems

Mechatronic systems:

- Operate in distinct modes
- React to events
- Must behave predictably
- Often involve safety constraints

FSMs provide:
- Deterministic behavior
- Clear structure
- Verifiable logic

---

## Core Principle

> **Control behavior must be explicit, not implicit.**

FSMs make behavior:
- Visible
- Reviewable
- Testable

---

## What Is an Automaton?

An automaton is a mathematical model consisting of:

- A finite set of states
- A set of events
- Rules that define transitions between states

FSMs are **finite automata**.

---

## Finite State Machine (FSM)

An FSM consists of:

1. States  
2. Events  
3. Transitions  
4. Guards (conditions)  
5. Actions (optional)

---

## FSM State

A **state** represents:

- A mode of operation
- A stable system condition
- A snapshot of system intent

Examples:
- IDLE
- RUNNING
- OPEN
- CLOSED
- ERROR

---

## FSM Event

An **event** is something that happens:

- Sensor input changes
- Timer expires
- User command arrives
- Fault detected

Examples:
- `button_pressed`
- `timeout`
- `limit_reached`
- `error_detected`

---

## FSM Transition

A **transition** defines:

> How the system moves from one state to another

Formally: (current_state, event, guard) → next_state


---

## Guards (Conditions)

A **guard** is a Boolean condition that must be true
for a transition to occur.

Example:
- `button_pressed == true`
- `door_position > 0.9`
- `timer_expired == true`

Guards prevent invalid transitions.

---

## FSM Actions

Actions may occur:

- On entering a state
- On exiting a state
- During a transition

Examples:
- Start motor
- Stop motor
- Reset timer
- Raise error flag

---

## Deterministic Behavior

An FSM must be **deterministic**:

> For a given state and event,  
> **at most one transition is allowed.**

This ensures:
- Predictability
- Testability
- Safety

---

## Non-Determinism (Not Allowed)

❌ Multiple transitions for the same event  
❌ Overlapping guards  
❌ Implicit state changes  

Engineering rule:

> **Ambiguous behavior is unacceptable.**

---

## FSM vs Procedural Logic

### Procedural code
- Hidden states
- Implicit flow
- Hard to reason about

### FSM-based logic
- Explicit states
- Explicit transitions
- Easy to review

---

## FSM Example: Automatic Door (High-Level)
```
States:
- CLOSED
- OPENING
- OPEN
- CLOSING
- ERROR

Events:
- Button pressed
- Limit reached
- Timeout
- Obstruction detected

```

---


## Example Transition Description

From state: `CLOSED`  
Event: `button_pressed`  
Guard: `motor_ok == true`  
Action: `start_motor_open`  
Next state: `OPENING`

---

# Introduction to PlantUML  
## Text-Based Diagrams for Engineering Systems

---

## Learning Objectives

After this section, students will be able to:

- Explain what PlantUML is and why it is used
- Understand why text-based diagrams are preferred in engineering
- Write a basic PlantUML diagram
- Create FSM diagrams using PlantUML
- Integrate PlantUML into a version-controlled workflow

---

## What Is PlantUML?

**PlantUML** is a tool that allows you to create diagrams using **plain text**.

Instead of drawing with a mouse, you:
- Write structured text
- Generate diagrams automatically

Supported diagram types include:
- State machines (FSM)
- Sequence diagrams
- Class diagrams
- Activity diagrams

---

## Why Engineers Use PlantUML

PlantUML is popular in engineering because it is:

- Text-based (diff-friendly)
- Version-controllable (Git)
- Reproducible
- Reviewable
- Language-independent

Engineering rule:

> **If it cannot be reviewed in Git,  
> it is not a reliable design artifact.**

---

## PlantUML vs Drawing Tools

### Drawing tools (draw.io, PowerPoint)
- Manual editing
- Hard to diff
- Easy to drift from implementation
- Poor traceability

### PlantUML
- Text = single source of truth
- Changes are explicit
- Easy peer review
- Ideal for safety-critical systems

---

## Where PlantUML Fits in This Course

PlantUML is used to:

- Design FSMs before coding
- Review control logic with peers
- Get instructor approval
- Prevent logic bugs before implementation


---

## Basic PlantUML Structure

Every PlantUML diagram has the same structure:

```plantuml
@startuml
  diagram content
@enduml

---

## FSM Diagram with PlantUML (Design Artifact)

FSM diagrams are **mandatory before coding**.

PlantUML allows:
- Text-based diagrams
- Version control
- Peer review
- Reproducible visuals

---

## PlantUML – Automatic Door FSM

```plantuml
@startuml
skinparam monochrome true

[*] --> CLOSED

state CLOSED
state OPENING
state OPEN
state CLOSING
state ERROR

CLOSED --> OPENING : button_pressed\n[motor_ok == true]\n/start_motor_open
CLOSED --> ERROR   : button_pressed\n[motor_ok == false]

OPENING --> OPEN   : limit_reached\n/stop_motor
OPEN --> CLOSING   : timeout\n/start_motor_close
CLOSING --> CLOSED : limit_reached\n/stop_motor

OPENING --> ERROR  : obstruction_detected\n/stop_motor
CLOSING --> ERROR  : obstruction_detected\n/stop_motor

ERROR --> CLOSED   : reset\n/clear_error
@enduml
---

## Python Representation – States and Events

```python
from enum import Enum, auto

class DoorState(Enum):
    CLOSED = auto()
    OPENING = auto()
    OPEN = auto()
    CLOSING = auto()
    ERROR = auto()

class DoorEvent(Enum):
    BUTTON_PRESSED = auto()
    LIMIT_REACHED = auto()
    TIMEOUT = auto()
    OBSTRUCTION = auto()
```

---
# Python Representation – FSM Skeleton

```python
class DoorFSM:
    def __init__(self):
        self.state = DoorState.CLOSED

    def handle_event(self, event: DoorEvent) -> None:
        if self.state == DoorState.CLOSED:
            self._handle_closed(event)
        elif self.state == DoorState.OPENING:
            self._handle_opening(event)
        elif self.state == DoorState.OPEN:
            self._handle_open(event)
        elif self.state == DoorState.CLOSING:
            self._handle_closing(event)
        else:
            self.state = DoorState.ERROR

```

---

# Python Representation – One State Logic
```python
def _handle_closed(self, event: DoorEvent) -> None:
    if event == DoorEvent.BUTTON_PRESSED:
        self._start_motor_open()
        self.state = DoorState.OPENING
```
## Why This Code Is Deterministic

- One function per state

- One event handled at a time

- Explicit state transitions

- No hidden logic
- This directly mirrors the FSM diagram.

---
# What This Code Does NOT Do (Yet)

❌ Control the motor
❌ Read sensors
❌ Manage timing
❌ Handle concurrency

This code only represents behavioral structure.



---


## FSM Representation Methods

FSMs can be represented as:

- State diagrams (preferred)
- Transition tables
- Formal automata definitions

---

## State Diagrams

State diagrams show:

- States as nodes
- Transitions as arrows
- Events and guards on arrows

They are:
- Visual
- Easy to review
- Language-independent

---

## Why Diagrams Come Before Code

FSM diagrams:
- Reveal design errors early
- Enable peer review
- Prevent logic bugs
- Reduce coding mistakes

---

## Laboratory Session Overview

### Objective

Design FSM logic **without writing code**.

Focus:
- Thinking
- Modeling
- Reviewing

---

## Laboratory Tasks

Students must:

1. Choose a system (door, motor, tank, etc.)
2. Identify all states
3. Identify all events
4. Define transitions and guards
5. Draw a complete FSM diagram

---

## Peer Review Activity

Students must:

- Exchange FSM diagrams
- Review for:
  - Missing states
  - Invalid transitions
  - Non-determinism
- Provide written feedback

---

## What Is NOT Allowed in the Lab

❌ Writing Python code  
❌ Writing FSM libraries  
❌ Implementing transitions  

This lab is **pure design**.

---

## Deliverable

Each student must submit:

- A complete FSM diagram
- Clearly labeled states
- Events and guards on transitions
- Legible and unambiguous logic

---

## Mandatory Approval Rule

> **No FSM approval → No implementation allowed**

This mirrors professional safety-critical development.

---

## Common FSM Design Errors

- Missing error state
- No recovery path
- Hidden implicit states
- Transitions without guards

---

## Grading Criteria

- Completeness of states
- Correct use of events
- Deterministic transitions
- Diagram clarity
- Peer review participation

---

## Key Takeaways

- FSMs enforce deterministic control
- States and events must be explicit
- Diagrams prevent logic bugs
- Design always precedes implementation

---

## Next Chapter

**Chapter 5 – Implementing FSMs in Object-Oriented Software**

Focus:
- State pattern
- Class-based FSMs
- Mapping diagrams to code


