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
  - [Usage Examples](#usage-examples)
  - [Configuration](#configuration)
  - [Logging \& Visualization](#logging--visualization)
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
git clone https://github.com/yourorg/your-domain-repo.git
cd your-domain-repo

# Create & activate virtual environment
python3 -m venv .venv
source .venv/bin/activate       # Linux/Mac
.venv\Scripts\activate.bat     # Windows

# Install core dependencies
pip install -r requirements.txt
```

*Optional tools:* Fast Downward, Metric-FF, or other PDDL planners installed separately.

## Directory Structure

```text
domain-repo/
├── envs/               # Custom Gym/PyBullet environments
|   ├── simulation
|       ├── env.py
|       ├── bowl        # Contains urdf for bowl
|       ├── robotiq-2f-85
|       └── ur5e
|   └── wrappers
|       └── coffee_env.py
├── pddl/               # PDDL domain & problem files
|   ├── domain.pddl
|   └── problem.pddl
├── planner/            # Planning interfaces & wrappers
├── sim/                # Simulation launch scripts & utilities
├── agents/             # Execution logic & agent controllers
├── configs/            # YAML/JSON experiment configs
├── logs/               # Simulation & training logs
├── models/             # Saved policies & artifacts
├── scripts/            # Helper scripts (plan, execute, render)
├── tests/              # Unit and integration tests
├── requirements.txt    # Python dependencies
└── README.md           # This file
```

## Environment

- **Location:** `envs/`
- **Registration:** Envs register via `gym.register(...)` in `envs/__init__.py`.
- **Usage:** Import and create with:
  ```python
  import gym
  env = gym.make('YourCustomEnv-v0')
  ```

## PDDL Planning

- **Domain files:** stored in `pddl/domain.pddl`
- **Problem files:** stored in `pddl/problem.pddl`
- **Planner interface:** `planner/run_planner.py`

Example:

```bash
python scripts/plan.py \
  --domain pddl/domain.pddl \
  --problem pddl/problem.pddl \
  --planner fast-downward \
  --output plan.txt
```

## Simulation Execution

- **Execution script:** `scripts/execute.py`
- **Arguments:** plan file, env name, rendering flags

Example:

```bash
python scripts/execute.py \
  --plan plan.txt \
  --env YourCustomEnv-v0 \
  --render True
```

## Usage Examples

1. **Full pipeline:** plan → simulate
   ```bash
   python scripts/plan.py --domain pddl/domain.pddl --problem pddl/problem.pddl --output plan.txt
   python scripts/execute.py --plan plan.txt --env YourCustomEnv-v0
   ```
2. **Interactive rendering:**
   ```bash
   python scripts/render.py --env YourCustomEnv-v0 --episodes 3
   ```

## Configuration

- Modify experiment settings in `configs/*.yaml`
- Typical fields:
  - Environment parameters
  - Planner options (e.g., search strategies)
  - Simulation settings (timestep, max steps)

## Logging & Visualization

- Logs saved under `logs/` by default
- Visualize metrics with TensorBoard:
  ```bash
  ```

tensorboard --logdir logs/

````

## Testing

Run unit & integration tests:
```bash
pytest tests/
````

## Contributing

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/xyz`
3. Commit changes: `git commit -m 'Add xyz feature'`
4. Push branch: `git push origin feature/xyz`
5. Open a Pull Request

## Contact

Sam Sherman samuel.sherman@tufts.edu
Shivam Goel shivam.goel@tufts.edu

