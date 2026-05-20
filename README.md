# STAR-CCM+ DRL Active Flow Control

Couples STAR-CCM+ (unsteady CFD) with a PPO agent (Stable-Baselines3) to perform active flow control on a circular cylinder with two opposing jet actuators. Communication between STAR-CCM+ and Python uses a lightweight file-based handshake — no STAR-CCM+ co-simulation license required.

## Repository layout

```
config/
  case_config.json              ← single source of truth for all parameters
macros/
  template_base_for_cylinderAFC.java   ← step 1: build, mesh, and save baseline sim
  add_history_points.java              ← step 2: create probe monitors in STAR-CCM+
  add_periodic_jets_from_json.java     ← step 3 (optional): prescribed/table validation
  add_realtime_jets_from_json.java     ← step 4: realtime DRL coupling macro
python/
  starccm_env.py               ← Gymnasium environment (handshake + obs/reward)
  train_drl.py                 ← PPO training & evaluation script
  dummy_rt_sender.py           ← test the handshake without DRL
data/
  observations_list.txt        ← probe positions  (x y z name, one per line)
  jet_actions_table.txt        ← action schedule for table mode
runtime/                       ← created automatically; gitignored
  jet_action.txt               ← Python → STAR-CCM+ (jet velocity)
  agent_ready.flag             ← Python → STAR-CCM+ (action is ready)
  starccm_ready.flag           ← STAR-CCM+ → Python (obs are ready)
  observations.txt             ← probe Vx/Vy values written by macro
  forces.txt                   ← force coefficients written by macro
simulations/                   ← .sim files; gitignored
models/                        ← saved PPO checkpoints; gitignored
logs/                          ← TensorBoard logs + starccm.log; gitignored
```

---

## Quick-start

### 1. Configure

Edit `config/case_config.json`.  At minimum, fill in the STAR-CCM+ executable path:

```json
"starccm": {
  "exe": "C:/Program Files/Siemens/XX.XX.XXX/STAR-CCM+XX.XX.XXX/star/bin/starccm+.exe",
  "licpath": "",
  "podkey":  ""
}
```

Leave `licpath` and `podkey` empty if you use a local license; they are omitted from the command when blank.

Find the executable path with:

```powershell
Get-ChildItem "C:\Program Files\Siemens" -Recurse -Filter "starccm+.exe" | Select-Object FullName
```

### 2. Python environment

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

---

## Macro execution order

### Step 1 — Build and save the baseline simulation

Open the STAR-CCM+ template file in the GUI, then run:

```
macros/template_base_for_cylinderAFC.java
```

This reads `case_config.json` (via the `STARCCM_JSON` env var or auto-detected path), applies geometry/mesh/physics parameters, runs the baseline flow with jets off, and saves the `.sim` file to `simulations/cylinder_afc_Re100.sim`.

Alternatively in batch mode:

```powershell
$env:STARCCM_JSON = (Resolve-Path "config/case_config.json").Path
& "C:\...\starccm+.exe" -batch -macro macros/template_base_for_cylinderAFC.java simulations/your_template.sim
```

### Step 2 — Create probe monitors

Still in the GUI (or in a second batch run on the saved `.sim`), run:

```
macros/add_history_points.java
```

This reads `data/observations_list.txt` (format: `x y z probe_name`) and creates one STAR-CCM+ MaxReport per probe per velocity component (`Rep_<name>_Vx`, `Rep_<name>_Vy`). Save the sim afterwards.

### Step 3 (optional) — Validate actuation without DRL

Set `jets.mode` in `case_config.json` to `"prescribed"` or `"table"`, then run:

```
macros/add_periodic_jets_from_json.java
```

This applies a sinusoidal or table-scheduled jet signal and runs the solver — useful for verifying that the jet boundaries and probe monitors work before introducing the Python handshake.

### Step 4 — Real-time DRL coupling

Set `jets.mode` to `"realtime"` in `case_config.json`, then choose one of the two run modes below.

---

## Run modes

### Manual mode (default) — recommended for development

You control when STAR-CCM+ starts. Useful when iterating on the macro or debugging.

**Terminal 1 — start STAR-CCM+ with the realtime macro:**

```powershell
$env:STARCCM_JSON = (Resolve-Path "config/case_config.json").Path
& "C:\...\starccm+.exe" -batch -macro macros/add_realtime_jets_from_json.java simulations/cylinder_afc_Re100.sim
```

Or run the macro from the STAR-CCM+ GUI (Tools → Macros → Run Macro).

**Terminal 2 — start training (default is manual mode):**

```powershell
python python/train_drl.py
```

Python waits for STAR-CCM+ to write `runtime/starccm_ready.flag` before sending the first action.

### Auto-launch mode — for HPC / batch training

Python spawns STAR-CCM+ automatically at the start of each episode and kills it at the end.

```powershell
python python/train_drl.py --auto-launch
```

STAR-CCM+ stdout goes to `logs/starccm.log`.  Monitor it in a separate terminal:

```powershell
Get-Content logs/starccm.log -Wait
```

### Resume training

```powershell
python python/train_drl.py --resume models/ppo_afc_cylinder
python python/train_drl.py --resume models/ppo_afc_cylinder --auto-launch
```

### Evaluate a saved model

```powershell
python python/train_drl.py --eval models/ppo_afc_cylinder --episodes 5
python python/train_drl.py --eval models/ppo_afc_cylinder --episodes 5 --auto-launch
```

---

## Testing the handshake without DRL

Before running PPO, verify the Python ↔ STAR-CCM+ communication with the dummy sender.

**Terminal 1** — start STAR-CCM+ with the realtime macro (manual mode, see above).

**Terminal 2** — run the dummy sender:

```powershell
python python/dummy_rt_sender.py
```

It waits for `starccm_ready.flag`, sends a random jet velocity, and touches `agent_ready.flag` in a loop.  Default range is `[-0.1, 0.1]` m/s.  Override with:

```powershell
python python/dummy_rt_sender.py --vmin -1.0 --vmax 1.0
```

---

## Handshake protocol

Each agent step covers `action_repeat` physical timesteps in STAR-CCM+. Only one flag exchange occurs per agent decision.

```
STAR-CCM+ macro                       Python (StarCCMEnv)
──────────────────                    ──────────────────
touch starccm_ready.flag         →    consume starccm_ready.flag
                                      read observations.txt
                                      compute action (PPO)
                                      write jet_action.txt
                                 ←    touch agent_ready.flag
consume agent_ready.flag
apply jet BCs (top = v, bot = -v)
advance action_repeat timesteps
  (PhysicalTimeStoppingCriterion
   += action_repeat × dt)
write observations.txt
touch starccm_ready.flag         →    (next cycle)
```

Key config parameters that control the loop:

| Parameter | Location | Meaning |
|---|---|---|
| `time.dt` | `case_config.json` | Physical timestep size [s] |
| `jets.realtime.n_steps` | `case_config.json` | Total CFD timesteps per episode |
| `jets.realtime.action_repeat` | `case_config.json` | CFD timesteps per agent decision |
| `jets.amplitude` | `case_config.json` | Max jet velocity [m/s] (action space bound) |

With the defaults (`n_steps=1000`, `action_repeat=3`, `dt=0.01`): each episode covers 10 s of flow time, the agent makes 333 decisions, each covering 0.03 s.

---

## `case_config.json` reference

```jsonc
{
  "starccm": {
    "exe":     "path/to/starccm+.exe",  // full path to executable
    "licpath": "1999@server",           // license server (omit if empty)
    "podkey":  "YOUR_KEY"               // pod key (omit if empty)
  },
  "geometry": { "diameter": 1.0, "span": 0.2, ... },
  "flow":     { "reynolds": 100, "density": 1.0, "inlet_velocity": 1.0 },
  "mesh":     { "base_reference": 0.5, ... },
  "jets": {
    "mode":      "realtime",            // "prescribed" | "table" | "realtime"
    "amplitude": 1.0,                   // action space bound [m/s]
    "realtime": {
      "action_file":      "runtime/jet_action.txt",
      "ready_flag":       "runtime/agent_ready.flag",
      "obs_flag":         "runtime/starccm_ready.flag",
      "obs_file":         "runtime/observations.txt",
      "drag_file":        "runtime/forces.txt",
      "n_probes":         22,           // observation vector length
      "n_steps":          1000,         // total CFD timesteps per episode
      "action_repeat":    3,            // CFD timesteps per agent step
      "reward_alpha":     0.1,          // Cl penalty weight
      "poll_interval_ms": 10,
      "poll_timeout_s":   120
    }
  },
  "probes":  { "points_file": "data/observations_list.txt", "region": "fluid" },
  "time":    { "dt": 0.01, "steps": 5000 },  // steps used by template macro only
  "output":  { "save_sim": "simulations/cylinder_afc_Re100.sim" }
}
```

> **Note:** `time.steps` is used only by the baseline template macro to set the initialization run length. The DRL episode length is controlled by `jets.realtime.n_steps`.

---

## Python environment

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# if PowerShell blocks activation:
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

pip install -r requirements.txt
```

`requirements.txt`:

```
numpy>=1.23
gymnasium>=0.29
stable-baselines3>=2.3.0
torch>=2.1
tensorboard>=2.15
```

Monitor training with TensorBoard:

```powershell
tensorboard --logdir logs
```
