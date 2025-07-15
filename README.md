# Coffee Bot Environment

The project consists of a custom reinforcement learning environment for a coffee bot, integrating symbolic planning with PyBullet physics simulation. It includes:
- A Gym/PyBullet custom environment
- PDDL-based planning modules
- Plan execution in a PyBullet simulation

## Table of Contents

- [Coffee Bot Environment](#coffee-bot-environment)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Installation](#installation)
  - [Directory Structure](#directory-structure)
  - [Environment](#environment)
  - [PDDL Planning](#pddl-planning)
  - [Simulation Execution](#simulation-execution)
    <!-- - [Usage Examples](#usage-examples) -->
  - [Configuration](#configuration)
    <!-- - [Logging \& Visualization](#logging--visualization) -->
  - [Contributing](#contributing)
  - [Contact](#contact)

---

## Overview

This repository integrates symbolic planning with reinforcement learning by:

- Defining a custom Gym environment backed by PyBullet physics
- Specifying domain and problem definitions in PDDL
- Generating plans via an external planner
- Executing plans step-by-step in simulation

Use this template to organize modules for environment development, planner integration, and simulation-based execution.

## Installation

```bash
# Clone and enter
git clone https://github.com/goelshivam1210/coffee_bot.git
cd coffee_bot

# Create & activate virtual environment
conda env create
conda activate saycan
```


## Directory Structure

```text
coffee-bot/
├── envs/               # Custom Gym/PyBullet environments
|   ├── simulation/
|       ├── env.py
|       ├── configs.py
|       ├── gripper.py
|       ├── bowl        # Contains urdf for bowl
|       ├── robotiq-2f-85
|       └── ur5e
|   └── wrappers/
|       ├── cup.py
|       ├── button.py
|       ├── location.py
|       └── coffee_env.py
├── pddl/               # PDDL domain & problem files
|   ├── domain.pddl
|   ├── problem.pddl
|   └── ... other test pddl files ...
├── scripts/            # Helper scripts (plan, execute, render)
|   ├── pddl-parser/    # pddl-parser library (https://github.com/pucrs-automated-planning/pddl-parser)
|   └── planner/
|       └── planning_functions.py
├── utils/
|   ├── logger.py
|   └── transform_utils.py
├── environment.yml
├── main_w_planner.py   # Plan & execute
└── README.md           # This file
```

## Environment

- **Location:** `envs/`

## PDDL Planning

- **Domain files:** stored in `pddl/domain.pddl`
- **Problem files:** stored in `pddl/problem.pddl`
- **Planner interface:** `scripts/planner/planning_functions.py`. Uses the pddl-parser library (https://github.com/pucrs-automated-planning/pddl-parser).
- **TODO:** Add command line arguments for PDDL problem file.

## Simulation Execution

- **Execution script:** `main_w_planner.py`
- **Arguments:** None

Example:

```bash
python main_w_planner.py
```
Plans based on `pddl/domain.pddl` and the pddl file listed as the problem_file in the `main_w_planner.py` file. Executes generated plan in the simulation environment.

<!-- ## Usage Examples

1. TBD -->

## Configuration

- Modify environment parameters in `envs/sim/configs.py`
- Modify PDDL problem file in `main_w_planner.py`
- TODO: add global config file

<!-- ## Logging & Visualization

TBD -->

## Contributing

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/xyz`
3. Commit changes: `git commit -m 'Add xyz feature'`
4. Push branch: `git push origin feature/xyz`
5. Open a Pull Request

## Contact

Sam Sherman samuel.sherman@tufts.edu
Shivam Goel shivam.goel@tufts.edu

