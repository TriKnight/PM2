---
marp: true
title: Chapter 1 – Introduction to Engineering Software for Mechatronic Systems
author: Programming Method 2
theme: default
paginate: true
---
# PROGRAMMING METHOD 2
# Chapter 1  
## Introduction to Engineering Software  
### for Mechatronic Systems

---

## Chapter Objectives

After completing this chapter, students will be able to:

- Understand the role of software in mechatronic systems
- Distinguish scripting vs control software
- Explain limits of procedural programming
- Explain why OOP is required for scalable systems
- Set up a professional Python environment
- Use Git for version-controlled development

---

## Mechatronic Systems Overview

A mechatronic system consists of:

- Mechanical components
- Electrical and electronic components
- Sensors and actuators
- **Software (control and coordination)**

---

## Role of Software

Software is responsible for:

- Interpreting sensor data
- Making control decisions
- Managing system states
- Enforcing safety constraints
- Coordinating subsystems

---

## Key Observation

> Two identical mechanical systems  
> can behave completely differently  
> **depending on their software**

Software is a **core system component**, not an accessory.

---

## Software as an Engineering Artifact

Engineering software must be:

- Deterministic
- Predictable
- Testable
- Maintainable
- Traceable
- Safe

---

## What Engineering Software Is NOT

Engineering software is different from:

- Scripts
- Prototypes
- One-off automation tools

Engineering software must be:

- Designed before implementation
- Verified before deployment
- Explainable to other engineers

---

## Scripting Software

### Typical Characteristics

- Linear execution
- Minimal structure
- Direct hardware access
- Written quickly for a single task

---

## Scripting Software – Limitations

Common properties:

- Top-down execution
- Global variables
- Hard-coded values
- Difficult to test
- Difficult to extend

---

## When Scripting Is Useful

Scripting is suitable for:

- Data processing
- Automation tasks
- Quick prototyping

Scripting is **not suitable** for:

- Safety-critical systems
- Long-lived control software
- Systems with multiple operating modes

---

## Control Software

### Core Characteristics

- Event-driven behavior
- Explicit system states
- Clear separation of responsibilities
- Hardware abstraction
- Testable without hardware

---

## Control Software Answers

Control software must answer:

- What state is the system in?
- What events can occur?
- What transitions are allowed?
- What actions must be executed?

---

## Control Software Requirements

Control software must be:

- Deterministic
- Structured
- Verifiable

---

## Procedural Programming Review

Procedural programming organizes software using:

- Functions
- Data passed between functions
- Execution flow via function calls

---

## Procedural Programming – Strengths

Advantages:

- Easy to understand for small programs
- Efficient for linear tasks

---

## Procedural Programming – Limitations

In mechatronic systems:

- Poor representation of system states
- Poor scalability with system complexity
- Tight coupling of logic and hardware
- Hard to test components independently

---

## Typical Procedural Problem

Example issue:

> Adding a new operating mode  
> requires modifying many functions

This increases error risk and maintenance cost.

---

## Object-Oriented Programming (OOP)

OOP organizes software using:

- Objects representing system components
- Classes defining behavior and responsibility
- Explicit interfaces between components

---

## Key OOP Concepts Used in This Course

- Classes as system representations
- Objects as runtime components
- Encapsulation to protect system integrity
- Polymorphism for multiple implementations
- Abstraction to decouple logic from hardware

---

## OOP in Control Systems

In control software:

- Controller → object
- Sensor → object
- Actuator → object
- State machine → object

---

## Why OOP Is Required

OOP enables:

- Clear system architecture
- Hardware-independent control logic
- Scalable system growth
- Reliable and isolated testing

---

## Laboratory Session Overview

In this lab, students will:

- Set up a professional Python environment
- Initialize a Git repository
- Create a structured Python project

---

## Development Environment Setup

Students must install:

- Python 3.10 or newer
- Virtual environment tool (venv)
- Code editor or IDE with debugger
- Git

---

## Environment Verification

Students must verify:

- Python version
- Virtual environment creation and activation
- Installation of basic development tools

---

## Git Repository Initialization

Required steps:

- Initialize Git repository
- Create initial commit
- Use meaningful commit messages
- Push to remote repository (if applicable)

---

## Version Control Rules

- All work must be committed
- No submission without Git history
- Each lab session ends with a commit

---

## Python Project Structure

Students must create a professional structure:
```
project/
├── src/
│ ├── main.py
│ └── init.py
├── tests/
│ └── init.py
├── README.md
├── requirements.txt

```

---

## Project Structure Rules

- No logic in global scope
- No single-file projects
- Tests must be separate
- Main entry point must be minimal

---

## Chapter 1 Deliverable

Students must submit:

- Verified Python environment
- Git repository with commit history
- Correct project structure
- Short README describing setup

---

## Failure Conditions

- Missing Git history
- Non-working environment
- Improper project structure

---

## Key Takeaways

- Software is a primary system component
- Control software ≠ scripting
- Procedural programming does not scale
- OOP enables safe and maintainable control systems
- Professional tooling is mandatory

---

## Next Chapter

**Chapter 2 – Modeling Technical Variables in Software**

Focus:
- Inputs, outputs, internal states
- Software representation of physical systems



