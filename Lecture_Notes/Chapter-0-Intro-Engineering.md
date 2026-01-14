---
marp: true
theme: default
paginate: true
math: mathjax
backgroundColor: #fff
---

# Chapter 0: Introduction to Engineering Software for Mechatronic Systems

---

## Learning Objectives

By the end of this chapter, you will:
- Understand the role of software in mechatronic systems
- Distinguish between scripting and control software
- Compare procedural vs object-oriented programming paradigms
- Set up a professional development environment
- Initialize and manage a Git repository
- Organize a Python project structure

---

## What is a Mechatronic System?

**Definition:** A system combining:
- **Mechanics** - Physical structures and components
- **Electronics** - Electrical hardware and controllers
- **Software** - Logic and decision making
- **Control** - Feedback loops and regulation

### Real-World Examples
- Industrial robotic arms
- Autonomous vehicles
- Medical devices
- Manufacturing equipment

---

## The Role of Software in Mechatronics

### Software Functions
1. **Sensing** - Process sensor data from the environment
2. **Decision Making** - Plan actions based on current state
3. **Control** - Execute commands to actuators
4. **Monitoring** - Verify system health and safety

### Why Software Matters
- Determines system behavior and capabilities
- Often determines product success/failure
- Must handle real-time constraints
- Needs to be maintainable and testable

---

## Scripting vs Control Software

### Scripting Software
**Purpose:** Automate repetitive tasks

- Simple, linear execution
- Direct user interaction
- High-level operations
- Example: Data processing scripts

```python
# Simple scripting example
for measurement in sensor_readings:
    print(measurement)
    save_to_file(measurement)
```

---

## Scripting vs Control Software (cont.)

### Control Software
**Purpose:** Manage real-time system behavior

- Reactive to environment changes
- Runs continuously (loops/events)
- Strict timing requirements
- Complex state management
- Safety critical

---
## Scripting vs Control Software (cont.)

### Control Software

```python
# Control software pattern
while system_running:
    sensor_data = read_sensors()
    decision = make_decision(sensor_data)
    execute_action(decision)
    check_safety_constraints()
```

---

## Procedural Programming

### Characteristics
- Functions are primary building blocks
- Code executed as sequence of procedures
- State shared via global variables
- Focus on **what** the code does

### Advantages
- Straightforward for simple programs
- Easy to understand execution flow
- Minimal overhead

---


## Procedural Programming
### Disadvantages
- Difficult to maintain as complexity grows
- State management becomes chaotic
- Code reuse is limited
- Testing is challenging

---

## Procedural Programming Example

```python
# Procedural approach
temperature = 0
sensor_error = False

def read_temperature():
    global temperature, sensor_error
    try:
        temperature = get_raw_value()
        sensor_error = False
    except:
        sensor_error = True
        temperature = -999

def check_temperature():
    if not sensor_error:
        if temperature > 100:
            trigger_alarm()

# Main loop
while True:
    read_temperature()
    check_temperature()
```

---

## Object-Oriented Programming (OOP)

### Characteristics
- Objects encapsulate data and behavior
- Code organized around entities/concepts
- Encapsulation hides internal details
- Focus on **objects** and their interactions

### Core Principles
- **Abstraction** - Simplify complex systems
- **Encapsulation** - Hide complexity
- **Inheritance** - Share common behavior
- **Polymorphism** - Flexible interfaces


---

## Encapsulation & Abstraction Example
Encapsulation bundles the data (reading) and methods (calibrate) together, while Abstraction hides the complex math inside a simple method call.

```python
class Sensor:
    def __init__(self, name):
        self.name = name
        self.__offset = 0.5  # Encapsulation: Private variable (hidden)

    def get_calibrated_value(self, raw_value):
        # Abstraction: User doesn't need to know the formula
        return raw_value - self.__offset

temp_sensor = Sensor("TMP36")
print(temp_sensor.get_calibrated_value(25.8))

```

---

## Inheritance Example

Inheritance allows a specific type of sensor to "inherit" the properties of a general sensor without rewriting code.

```python

class UltrasonicSensor(Sensor):
    def __init__(self, name, pin):
        super().__init__(name) # Inherit from parent
        self.pin = pin

    def ping(self):
        return "Sending sonic pulse..."

dist_sensor = UltrasonicSensor("HC-SR04", 12)
print(dist_sensor.name) # Accessed from parent
print(dist_sensor.ping()) # Own unique method
```

---

## Polymorphism

Polymorphism allows different objects to be treated the same way. In mechatronics, you might want to "read" all sensors in a loop, regardless of whether they are Temperature, Pressure, or Light sensors.

```python

class TemperatureSensor:
    def read(self):
        return "24°C"

class PressureSensor:
    def read(self):
        return "101.3 kPa"

# Polymorphism in action
sensors = [TemperatureSensor(), PressureSensor()]

for s in sensors:
    print(s.read()) # Same method name, different behaviors

```

---



## Object-Oriented Programming Example

```python
# Object-oriented approach
class TemperatureSensor:
    def __init__(self):
        self._value = 0
        self._error = False
    
    def read(self):
        try:
            self._value = self._get_raw_value()
            self._error = False
        except Exception as e:
            self._error = True
            self._value = -999
    
    def is_valid(self):
        return not self._error
    
    def get_value(self):
        return self._value if self.is_valid() else None

# Usage is cleaner and more intuitive
sensor = TemperatureSensor()
sensor.read()
if sensor.is_valid():
    print(sensor.get_value())
```

---

## Procedural vs OOP: Summary

| Aspect | Procedural | Object-Oriented |
|--------|-----------|-----------------|
| **Organization** | Functions | Classes/Objects |
| **Data** | Global variables | Instance variables |
| **Reusability** | Limited | High (inheritance) |
| **Maintainability** | Low (complex) | High (modular) |
| **Scalability** | Difficult | Better |
| **Testing** | Harder (dependencies) | Easier (isolation) |

---

## Lab 1: Development Environment Setup

### Requirements
1. Python 3.8+ interpreter
2. A code editor (VS Code, PyCharm)
3. Git version control
4. Project directory structure

### Installation Steps
1. Download Python from python.org
2. Install IDE of choice
3. Install Git: https://git-scm.com/
4. Verify installations with version commands

---

## Verification Commands

```bash
# Check Python installation
python --version
# Output: Python 3.10.0 (or newer)

# Check Git installation
git --version
# Output: git version 2.34.1 (or newer)

# Create test directory
mkdir mechatronics-lab
cd mechatronics-lab
```

---
# How git command works

<div style="text-align: center;">

![width:550px](./figs/chapter0/0202-git-commands.png)

</div>

---

## Lab 2: Git Repository Initialization

### Initialize Repository
```bash
# Navigate to project directory
cd your-project-folder

# Initialize git repository
git init

# Configure git (one time setup)
git config user.name "Your Name"
git config user.email "your.email@example.com"

# Verify configuration
git config --list
```

---

## Understanding Git Basics

### Key Concepts
- **Repository** - Local copy of project with history
- **Commit** - Snapshot of changes with message
- **Branch** - Independent line of development
- **Remote** - Shared repository (GitHub, GitLab)

### Common Workflow
```bash
git add <files>              # Stage changes
git commit -m "message"      # Save changes locally
git push origin main         # Upload to remote
git pull origin main         # Download updates
```

---

## Lab 3: Python Project Structure

### Recommended Organization
```
mechatronic-project/
├── .git/                 # Git metadata
├── .gitignore            # Files to ignore
├── README.md             # Project description
├── requirements.txt      # Dependencies
├── src/                  # Source code
│   ├── __init__.py
│   ├── main.py
│   └── sensors.py
├── tests/                # Test files
│   ├── __init__.py
│   └── test_sensors.py
└── docs/                 # Documentation
```

---

## Project Structure Details

### `.gitignore` - Essential Entries
```
# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/
.pytest_cache/
# IDE
.vscode/
.idea/
# Environment
venv/
env/
.env
# OS
.DS_Store
Thumbs.db
```

---

## Python Package Structure

### `__init__.py` - Makes package importable
```python
# src/__init__.py
"""
Mechatronic Systems Control Package
Version 1.0
"""

__version__ = "1.0.0"
__author__ = "Your Name"

# Expose main classes/functions
from .sensors import TemperatureSensor
from .controllers import MotorController

__all__ = ['TemperatureSensor', 'MotorController']
```

---

## Project Requirements File

### `requirements.txt`
```
# Core dependencies
numpy>=1.19.0
scipy>=1.5.0

# Testing
pytest>=6.0.0
pytest-cov>=2.10.0

# Code quality
pylint>=2.7.0
black>=21.0.0
```
### Installation
```bash
pip install -r requirements.txt
```

---

## Best Practices for Mechatronic Software

### Code Quality
1. **Clear Names** - Use intention-revealing names
2. **Small Functions** - Single responsibility principle
3. **Comments** - Explain why, not what
4. **Testing** - Write tests before code (TDD)
5. **Documentation** - Document interfaces and APIs

---
## Best Practices for Mechatronic Software (Cont.)
### Version Control
1. Commit frequently with clear messages
2. One feature per branch
3. Review code before merging
4. Tag releases with version numbers

---

## Clean Code Principles

### For Mechatronic Systems
- **Reliability** - Code must work under stress
- **Readability** - Future maintainers will thank you
- **Testability** - Facilitate automated testing
- **Safety** - Handle edge cases and failures
- **Performance** - Real-time constraints matter

### The Boy Scout Rule
> "Leave the code a little cleaner than you found it"

---

## Deliverable Checklist

### Lab Deliverables
- [ ] Development environment installed (Python, Git, IDE)
- [ ] Verified all tools with version commands
- [ ] Git repository initialized with proper configuration
- [ ] `.gitignore` file created
- [ ] Project directory structure created
- [ ] README.md written with project description
- [ ] `requirements.txt` with dependencies listed
- [ ] `src/__init__.py` and main module structure

---

## Deliverable Verification

### Test Your Setup
```bash
# Test Python
python -c "print('Python works!')"

# Test Git
git status
git log --oneline

# Test project import
cd ..
python -c "from mechatronic_project import TemperatureSensor"

# Create first commit
git add .
git commit -m "Initial project setup"
git log --oneline
```

---

## Summary

### Key Takeaways
1. Mechatronic systems require careful software architecture
2. Scripting and control software serve different purposes
3. OOP provides better structure for complex systems
4. Professional development practices matter
5. Version control and testing are essential
6. Clean code principles improve maintainability

### Remember
> *"Any fool can write code that a computer can understand. Good programmers write code that humans can understand."* - Martin Fowler

---

## Questions & Discussion

**Topics for Lab Session:**
- Installation troubleshooting
- Git workflow questions
- Python project organization tips
- Development environment preferences

**Resources:**
- Python docs: https://docs.python.org/3/
- Git tutorial: https://git-scm.com/docs
- GitHub: https://github.com
- Stack Overflow: https://stackoverflow.com

---

## Appendix: Useful Commands Reference

### Git Commands
```bash
git init                      # Initialize repo
git clone <url>              # Clone existing repo
git add <file>               # Stage changes
git commit -m "msg"          # Commit changes
git push origin main         # Push to remote
git pull origin main         # Pull from remote
git branch <name>            # Create branch
git checkout <branch>        # Switch branch
git status                   # Check repo status
git log --oneline            # View history
```
---

### Python Commands
```bash
python --version             # Check version
python <file>.py             # Run script
python -m pytest             # Run tests
python -m pip install <pkg>  # Install package
python -c "code"             # Run inline code
python -m venv venv          # Create virtual env
source venv/bin/activate     # Activate venv
```

---

## Appendix: Setup Troubleshooting

### Issue: Python not found
**Solution:** Add Python to PATH environment variable

### Issue: Git commits show wrong author
**Solution:** Set global git config
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Issue: Import errors
**Solution:** Ensure `__init__.py` exists in package directories

---
## Recommended Reading
- **Clean Code: A Handbook of Agile Software Craftsmanship** Robert C. Martin  

- **Software Modeling and Design: UML, Use Cases, Patterns, and Software Architectures** Hassan Gomaa  

- **Fluent Python: Clear, Concise, and Effective Programming** Luciano Ramalho  

- **Robotics, Vision and Control: Fundamental Algorithms in Python** Peter Corke

### Online
- Real Python: https://realpython.com
- Python official docs: https://docs.python.org
- GitHub Learning: https://github.com/skills
