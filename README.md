# STAR-CCM+ DRL AFC Framework

This repository contains a preliminary framework for coupling STAR-CCM+ with a Python reinforcement-learning loop for active flow control of a circular cylinder with top and bottom jets.

The current state of the repository should be considered a development baseline. The prescribed and table-based jet actuation workflows are usable for testing. The real-time action communication between Python and STAR-CCM+ is partially implemented but not yet fully validated for closed-loop DRL training.

## Purpose

The framework is intended to support the following workflow:

1. Configure the cylinder AFC case from a JSON file.
2. Generate or update the STAR-CCM+ simulation using Java macros.
3. Define top and bottom jet actuation.
4. Exchange actions and observations between Python and STAR-CCM+ through files.
5. Train or test a DRL policy using a Gymnasium-compatible environment.

## Current status

Implemented:

- JSON-driven case configuration.
- STAR-CCM+ macro for setting up a baseline cylinder AFC simulation with inactive jets.
- STAR-CCM+ macro for prescribed sinusoidal or table-based jet actuation.
- STAR-CCM+ macro for a file-polling real-time actuation loop.
- Python dummy real-time sender for testing the file handshake.
- Python Gymnasium environment structure for coupling STAR-CCM+ with Stable-Baselines3.
- PPO training script skeleton.

Not finished:

- Full closed-loop action-observation communication.
- Robust export of observations and force coefficients from STAR-CCM+ at every time step.
- Validation of the reward signal used by the DRL agent.
- Complete automation from an empty STAR-CCM+ session. The current macros assume a prepared STAR-CCM+ template with expected names for regions, boundaries, CAD models, mesh operations, and controls.
- General path handling. Several scripts currently use absolute Windows paths and should be adapted to the local repository path.

## Recommended repository structure

```text
starccm-drl-afc/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── config/
│   └── case_config.json
│
├── macros/
│   ├── cylinder_afc_from_json.java
│   ├── add_periodic_jets_from_json.java
│   ├── add_realtime_jets_from_json.java
│   └── set_observation_form_list.java
│
├── python/
│   ├── starccm_env.py
│   ├── train_drl.py
│   └── dummy_rt_sender.py
│
├── data/
│   ├── jet_actions_table.txt
│   └── probe_positions.txt
│
├── runtime/
│   ├── jet_action.txt
│   ├── agent_ready.flag
│   ├── starccm_ready.flag
│   ├── observations.txt
│   └── forces.txt
│
├── simulations/
│   └── README.md
│
├── models/
└── logs/
```

The `runtime/`, `models/`, `logs/`, and generated STAR-CCM+ simulation files should normally be ignored by Git.

## File roles

### `config/case_config.json`

Main configuration file. It defines geometry, flow parameters, mesh settings, jet settings, probe files, time step, number of steps, and output paths.

Important sections:

- `geometry`: cylinder diameter, span, and domain size.
- `flow`: Reynolds number, density, and inlet velocity.
- `mesh`: base size, surface controls, prism layers, and wake refinement.
- `jets`: actuation mode and action settings.
- `jets.realtime`: file paths used for Python-STAR-CCM+ communication.
- `probes`: probe-position file and target region.
- `time`: physical time step and number of steps.
- `output`: output STAR-CCM+ simulation path.

The current configuration uses `jets.mode = "realtime"` and defines the action file, ready flag, observation flag, observation file, drag file, number of probes, and polling parameters.

### `macros/cylinder_afc_from_json.java`

Baseline STAR-CCM+ macro.

Main function:

1. Reads `case_config.json`.
2. Updates geometry-related design parameters.
3. Updates mesh settings.
4. Sets density and viscosity from the requested Reynolds number.
5. Sets inlet velocity.
6. Sets top and bottom jet velocities to zero.
7. Sets unsteady time step and maximum number of steps.
8. Executes the mesh operation.
9. Clears and initializes the solution.
10. Runs the baseline simulation.
11. Saves the resulting `.sim` file.

This macro creates the baseline AFC case with quiescent jets. It is intended to produce a clean starting simulation before applying prescribed, table-based, or real-time actuation.

Important limitation: this macro assumes that the STAR-CCM+ file already contains expected objects such as `3D-CAD Model 1`, `3D-CAD Model 2`, `Automated Mesh`, `Physics 1`, `fluid`, `inlet`, `jet_top`, `jet_bot`, and the relevant mesh controls.

### `macros/add_periodic_jets_from_json.java`

Standalone macro for non-DRL actuation.

Supported modes:

- `prescribed`: applies an antisymmetric sinusoidal jet signal.
- `table`: applies a piecewise-constant action schedule from a text file.

For both modes:

- The top jet receives the specified velocity.
- The bottom jet receives the negative value.
- STAR-CCM+ user field functions are created and assigned to the jet boundary velocity profiles.

This macro is useful for validating the flow solver and actuation setup before attempting real-time DRL coupling.

### `macros/add_realtime_jets_from_json.java`

Realtime actuation macro.

Supported modes:

- `prescribed`
- `table`
- `realtime`

In `realtime` mode, the macro controls the time loop itself. It does not use STAR-CCM+ step listeners. Instead, it uses a file-polling handshake:

```text
STAR-CCM+ touches starccm_ready.flag
Python reads observations and computes an action
Python writes jet_action.txt
Python touches agent_ready.flag
STAR-CCM+ reads jet_action.txt
STAR-CCM+ applies top/bottom jet velocities
STAR-CCM+ advances one time step
STAR-CCM+ touches starccm_ready.flag again
```

The current macro applies a scalar action as:

```text
jet_top =  action
jet_bot = -action
```

This is the intended entry point for the future closed-loop DRL workflow.

Important limitation: the action-file handshake is present, but the full observation and reward export from STAR-CCM+ still needs to be completed and validated.

### `macros/set_observation_form_list.java`

Macro for creating history-point probes from a plain-text list of coordinates.

It reads a probe-position file with entries of the form:

```text
x y z name
```

and creates STAR-CCM+ point probes and monitors in the target fluid region.

The uploaded file name is `set_observation_form_list.java`, but the Java class is named `add_history_points`. For consistency, the file should be renamed to:

```text
add_history_points.java
```

or the class name should be changed to match the file name.

### `python/starccm_env.py`

Gymnasium-compatible Python environment.

This file was originally named `set_env_STARCCM.py`, but the training script imports:

```python
from starccm_env import StarCCMEnv
```

Therefore, it should be renamed to:

```text
starccm_env.py
```

The environment:

1. Launches STAR-CCM+ with the real-time macro.
2. Waits for `starccm_ready.flag`.
3. Reads observations from the configured observation file.
4. Sends one scalar jet action through the action file.
5. Touches `agent_ready.flag`.
6. Waits for STAR-CCM+ to advance one time step.
7. Reads the new observation and reward.
8. Returns data using the Gymnasium API.

The observation space is currently a vector of length `n_probes`. The action space is a one-dimensional continuous box bounded by the jet amplitude in the JSON file.

### `python/train_drl.py`

PPO training script using Stable-Baselines3.

This file was originally named `train_DRL_starccm.py`. For consistency, it should be renamed to:

```text
train_drl.py
```

The script:

1. Builds the STAR-CCM+ command.
2. Creates the `StarCCMEnv`.
3. Wraps it with a Stable-Baselines3 monitor.
4. Creates or loads a PPO model.
5. Trains the policy.
6. Saves checkpoints and the final model.
7. Supports evaluation through `--eval`.

Current limitation: paths are hard-coded in the script and should be moved to command-line arguments or read directly from `case_config.json`.

### `python/dummy_rt_sender.py`

Dummy action sender for testing the real-time handshake without DRL.

It waits for STAR-CCM+ to touch the observation-ready flag, writes a random jet velocity to the action file, and touches the agent-ready flag.

This script is useful for debugging the real-time macro before introducing PPO.

### `data/jet_actions_table.txt`

Plain-text list of prescribed jet velocities for table-based actuation.

Each non-comment line is one action value for the top jet. The bottom jet is automatically negated by the macro.

This file is used when:

```json
"jets": {
  "mode": "table"
}
```

### `data/probe_positions.txt`

Plain-text list of probe coordinates.

Each valid line has the format:

```text
x y z probe_name
```

This file is used by the observation/probe macro to create STAR-CCM+ history points.

The uploaded file is currently named `observations_list.txt`, but its contents are probe coordinates. To avoid confusion, it should be renamed to:

```text
probe_positions.txt
```

Runtime observation values should instead be written to a different file, for example:

```text
runtime/observations.txt
```

## Recommended macro execution order

### 1. Prepare the STAR-CCM+ template

Open or create a STAR-CCM+ template containing the required geometry, boundaries, regions, physics continuum, mesh operation, and mesh controls.

Expected object names include:

```text
fluid
inlet
jet_top
jet_bot
Physics 1
Automated Mesh
3D-CAD Model 1
3D-CAD Model 2
surf-control-cylinder
surf-control-jets
Volumetric Control
```

The current macros depend on these names.

### 2. Run the baseline macro

Run:

```text
cylinder_afc_from_json.java
```

Purpose:

- apply parameters from `case_config.json`
- mesh the case
- initialize the solution
- run the inactive-jet baseline
- save the baseline `.sim` file

This should be the first macro in the workflow.

### 3. Add probes

Run:

```text
add_history_points.java
```

Purpose:

- read the probe coordinate list
- create point probes
- create report monitors

This step prepares observation locations.

### 4. Test prescribed or table actuation

Run:

```text
add_periodic_jets_from_json.java
```

Use this before real-time DRL.

Purpose:

- verify that the jet boundaries work
- test sinusoidal actuation
- test table-based actuation
- confirm top/bottom antiphase convention

### 5. Test real-time actuation with dummy sender

Set:

```json
"jets": {
  "mode": "realtime"
}
```

Start STAR-CCM+ with:

```text
add_realtime_jets_from_json.java
```

Then run the Python dummy sender:

```powershell
python python/dummy_rt_sender.py ^
  --action-file "runtime/jet_action.txt" ^
  --ready-flag  "runtime/agent_ready.flag" ^
  --obs-flag    "runtime/starccm_ready.flag" ^
  --dt 0.05 ^
  --vmin -0.10 ^
  --vmax 0.10
```

This checks whether STAR-CCM+ and Python can exchange action flags correctly.

### 6. Run PPO training

After the real-time communication is validated, run:

```powershell
python python/train_drl.py
```

To resume:

```powershell
python python/train_drl.py --resume models/ppo_afc_cylinder
```

To evaluate:

```powershell
python python/train_drl.py --eval models/ppo_afc_cylinder --episodes 5
```

This step should only be considered reliable once the observation file and reward file are correctly exported by STAR-CCM+ at each time step.

## Python environment on Windows

Create the virtual environment from the repository root:

```powershell
python -m venv .venv
```

Activate it:

```powershell
.\.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

Install dependencies:

```powershell
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

Select the `.venv` interpreter in VS Code:

```text
Ctrl+Shift+P
Python: Select Interpreter
Select .venv
```

## Requirements

A minimal `requirements.txt` is:

```text
numpy>=1.23
gymnasium>=0.29
stable-baselines3>=2.3.0
torch>=2.1
tensorboard>=2.15
matplotlib>=3.7
pandas>=2.0
tqdm>=4.65
```

STAR-CCM+ is not a Python dependency. It must be installed separately and accessible through the command line.

You can either add STAR-CCM+ to the system path or define:

```powershell
$env:STARCCM_EXE="C:\Path\To\starccm+.exe"
```

## Git setup from scratch

From the project root:

```powershell
git init
git add README.md requirements.txt .gitignore config macros python data
git commit -m "Initial STAR-CCM DRL AFC framework"
```

Create the remote repository on GitHub, then connect it:

```powershell
git remote add origin git@github.com:USER/starccm-drl-afc.git
git branch -M main
git push -u origin main
```

## Recommended `.gitignore`

```text
# Python
.venv/
__pycache__/
*.pyc

# Logs and models
logs/
models/
runs/
*.zip

# STAR-CCM+ generated files
*.sim
*.sim~
*.trn
*.trn~
*.log

# Runtime communication files
runtime/*.flag
runtime/*.txt

# Large outputs
output/
exports/
post/
*.mp4
*.avi
*.png

# OS/editor
.DS_Store
.vscode/
```

If some runtime text files are needed as templates, add placeholder files such as:

```text
runtime/.gitkeep
```

and keep generated files ignored.

## Known issues to fix before production use

1. Rename `set_env_STARCCM.py` to `starccm_env.py`.
2. Rename `train_DRL_starccm.py` to `train_drl.py`.
3. Rename `set_observation_form_list.java` or change its class name so Java file and class names match.
4. Rename `observations_list.txt` to `probe_positions.txt` if it contains probe coordinates.
5. Use a separate runtime observation file, for example `runtime/observations.txt`.
6. Make the action file name consistent across JSON, Java, and Python. Use either `jet_action.txt` or `jet_action_RT.txt`, not both.
7. Replace hard-coded absolute Windows paths with paths relative to the repository root.
8. Confirm that STAR-CCM+ writes force coefficients to `runtime/forces.txt`.
9. Confirm that STAR-CCM+ writes the observation vector to `runtime/observations.txt`.
10. Validate that one Python action corresponds to exactly one STAR-CCM+ time step.

## Development recommendation

Use the following validation sequence before training:

1. Run the baseline macro with jets off.
2. Run prescribed sinusoidal actuation.
3. Run table-based actuation.
4. Run the real-time macro with the dummy sender.
5. Verify action-file synchronization.
6. Add observation and force export.
7. Validate the Gymnasium environment manually with random actions.
8. Start PPO training.

Do not start long DRL training until the one-step action-observation handshake is verified.
