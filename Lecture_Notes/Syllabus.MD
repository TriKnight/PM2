# Programming Method 2 – Object-Oriented Control of Mechatronic Systems (Python)

## Course Information

**Course Title:** Programming Method 2 – Object-Oriented Control of Mechatronic Systems  
**Programs:** Mechatronics Engineering, Robotics, Automation  
**Credits:** 4?? check 
**Language:** Python  
**Tool:** MuJoco, Git, Google Colab

---

## Prerequisites

Students are expected to have completed:

- Programming Method 1 (Python)
- Basic control theory concepts
- Basic knowledge of sensors and actuators
- Discrete mathematics with basic understanding of finite state machines (FSMs)

---

## Course Description

This course introduces object-oriented programming as an engineering discipline for controlling mechatronic systems. Students learn to model physical systems using software variables and states, implement automata-based control logic, and design clean object-oriented architectures for sensors and actuators.

The course emphasizes professional software development practices, including abstraction, polymorphism, testing, and tooling. Python is used as the implementation language, with strict architectural and verification requirements to ensure transferability to industrial systems and other languages such as C++ and ROS-based environments.

---

## Course Learning Outcomes (CLOs)

By the end of the course, students will be able to:

1. Model technical systems using software variables and internal states
2. Implement automata-based (finite state machine) control systems
3. Design object-oriented architectures for sensors and actuators
4. Apply abstraction and polymorphism to achieve hardware-independent control
5. Use Python containers and algorithms professionally for data processing
6. Verify software using unit, module, and system-level tests
7. Use professional development tools such as Git, debuggers, build and test systems

---

## Teaching and Learning Methods

- Lectures with engineering-focused explanations
- Weekly structured laboratory sessions
- System modeling and design before coding
- Incremental project-based learning
- Code walkthroughs and oral explanations
- No AI-assisted code generation allowed

---

## Software and Tools

- Python 3.10+
- Virtual environments (venv)
- Git for version control
- pytest for unit testing
- black for formatting
- logging module for runtime diagnostics
- Debugger (VSCode IDE)

---

## Course Schedule (14 Chapters)

### Chapter 1 – Introduction to Engineering Software for Mechatronic Systems
**Lecture Topics**
- Role of software in mechatronic systems
- Difference between scripting and control software
- Review of procedural vs object-oriented programming

**Laboratory**
- Development environment setup
- Git repository initialization
- Python project structure

**Deliverable**
- Verified development environment and Git repository

---

### Chapter 2 – Modeling Technical Variables
**Lecture Topics**
- Translating physical systems into software variables
- Inputs, outputs, internal states, and constraints
- Naming, units, and responsibility

**Laboratory**
- Variable analysis for a real system (e.g., automatic door, water tank)

**Deliverable**
- Technical Variable Table (approved before coding)

---

### Chapter 3 – System Decomposition and Class Responsibility
**Lecture Topics**
- Functional decomposition
- Identifying classes from system components
- Single responsibility principle

**Laboratory**
- Component and responsibility diagrams
- Initial class identification

**Deliverable**
- Class responsibility document

---

### Chapter 4 – Automata and Finite State Machines (FSM)
**Lecture Topics**
- FSM concepts: states, events, transitions, guards
- Deterministic behavior in control systems
- State diagrams

**Laboratory**
- FSM modeling on paper and diagrams
- Peer review of FSM logic

**Deliverable**
- FSM diagram (mandatory approval)

---

### Chapter 5 – Implementing State Machines in Python
**Lecture Topics**
- State representation using enums
- Transition logic and state integrity
- Introduction to the State design pattern

**Laboratory**
- FSM implementation in Python
- Debugging invalid transitions

**Deliverable**
- Working FSM module with tests

---

### Chapter 6 – Sensors as Software Objects
**Lecture Topics**
- Sensors as data providers
- Interface-based design using abstract base classes
- Simulation vs real sensors

**Laboratory**
- Sensor interface definition
- Simulated sensor implementations

**Deliverable**
- Sensor class hierarchy

---

### Chapter 7 – Actuators and Command Interfaces
**Lecture Topics**
- Actuators as command receivers
- Safety and limits in actuator software
- Separation of control logic and hardware access

**Laboratory**
- Actuator interface and implementations
- Polymorphic actuator usage

**Deliverable**
- Actuator abstraction and implementation

---

### Chapter 8 – System Integration and Object Interaction
**Lecture Topics**
- Controller–sensor–actuator interaction
- Dependency inversion
- Object ownership and lifetime

**Laboratory**
- Integration of FSM, sensors, and actuators
- Refactoring for clarity and testability

**Deliverable**
- Integrated control subsystem

---

### Chapter 9 – Data Processing with Python Containers and Algorithms
**Lecture Topics**
- Lists, dictionaries, sets
- Sorting, filtering, and aggregation
- Algorithmic thinking in control software

**Laboratory**
- Managing multiple sensors and actuators
- Data processing using Python standard library

**Deliverable**
- Data processing module

---

### Chapter 10 – Verification I: Unit Testing
**Lecture Topics**
- Purpose of testing in mechatronic software
- Unit testing principles
- Test isolation using fake objects

**Laboratory**
- Unit tests for FSMs and controllers
- Fault injection testing

**Deliverable**
- Unit test suite with coverage report

---

### Chapter 11 – Verification II: Module and System Testing
**Lecture Topics**
- Integration testing strategies
- Boundary and failure cases
- Simulation-based system verification

**Laboratory**
- System-level test scenarios
- Logging and assertion-based validation

**Deliverable**
- System test report

---

### Chapter 12 – Professional Development Tooling
**Lecture Topics**
- Git workflows and version control discipline
- Static analysis and type checking
- Debugging techniques

**Laboratory**
- Debugging a faulty control system
- Code quality analysis

**Deliverable**
- Tooling and debugging report

---

### Chapter 13 – Final Project Integration and Review
**Lecture Topics**
- Architecture review
- Common design mistakes in control software
- Preparation for final defense

**Laboratory**
- Final project integration
- Instructor-led design review

**Deliverable**
- Final project code freeze

---

### Chapter 14 – Final Evaluation
**Assessment**
- Project demonstration
- Oral defense and design explanation
- Code walkthrough and testing review

---

## Assessment Breakdown

- Final project (code, tests, report): 20%
- Final writing exam: 80%

---

## Academic Integrity Policy

- AI-assisted code generation is not permitted
- All submitted code must be explainable by the student
- Inability to explain submitted code may result in failure

---

## Expected Graduate Attributes

Upon successful completion, students will demonstrate:

- Engineering-level software modeling skills
- Object-oriented design competence
- Professional verification and tooling discipline
- Readiness for advanced robotics, embedded systems, and ROS-based development
