#!/usr/bin/env python3
"""
dummy_rt_agent.py  –  v2 (lock-safe)

• Waits for STAR-CCM+ to *touch* starccm_ready.flag (we watch its timestamp).
• Writes a random jet velocity to jet_action_RT.txt.
• Touches agent_ready.flag so STAR-CCM+ can advance.
• Repeats every --dt seconds.

Run example (matches your JSON paths):
    python dummy_rt_agent.py \
      --action-file "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/RT/jet_action_RT.txt" \
      --ready-flag  "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/RT/agent_ready.flag" \
      --obs-flag    "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/RT/starccm_ready.flag" \
      --dt 0.05 --vmin -0.10 --vmax 0.10
"""

import argparse, json, pathlib, random, time

# ------------------------------------------------------------------ #
# 1. parse CLI, seeding defaults from case_config.JSON if present
# ------------------------------------------------------------------ #
DEF_CFG = pathlib.Path("case_config.JSON")
defaults = {}
if DEF_CFG.exists():
    try:
        cfg = json.loads(DEF_CFG.read_text(encoding="utf-8"))
        rt  = cfg["jets"]["realtime"]
        defaults = dict(
            action_file = rt["action_file"],
            ready_flag  = rt["ready_flag"],
            obs_flag    = rt["obs_flag"],
            dt          = cfg["jets"].get("delta_t_action", 0.05),
        )
    except Exception as e:
        print(f"[dummy_rt_agent] Warning: could not parse {DEF_CFG}: {e}")

P = argparse.ArgumentParser(description="Send random jet actions to STAR-CCM+")
P.add_argument("--action-file", default=defaults.get("action_file","jet_action_RT.txt"))
P.add_argument("--ready-flag",  default=defaults.get("ready_flag","agent_ready.flag"))
P.add_argument("--obs-flag",    default=defaults.get("obs_flag","starccm_ready.flag"))
P.add_argument("--dt",   type=float, default=defaults.get("dt",0.05),
               help="wall-clock seconds between actions")
P.add_argument("--vmin", type=float, default=-0.10, help="min random velocity [m/s]")
P.add_argument("--vmax", type=float, default= 0.10, help="max random velocity [m/s]")
args = P.parse_args()

ACT  = pathlib.Path(args.action_file)
READY= pathlib.Path(args.ready_flag)
OBS  = pathlib.Path(args.obs_flag)

print(f"[dummy_rt_agent] running  dt={args.dt}s  range=[{args.vmin},{args.vmax}] m/s")
print("Press Ctrl-C to stop.\n")

try:
    last_mtime = None
    while True:
        # 1) wait for STAR-CCM+ to advance (OBS mtime changes)
        while True:
            try:
                mtime = OBS.stat().st_mtime
                if mtime != last_mtime:
                    last_mtime = mtime
                    break
            except FileNotFoundError:
                pass
            time.sleep(0.05)

        # 2) generate & write new action
        v = random.uniform(args.vmin, args.vmax)
        ACT.parent.mkdir(parents=True, exist_ok=True)
        ACT.write_text(f"{v:.6f}\n", encoding="utf-8")

        # 3) signal STAR-CCM+
        READY.touch()
        print(f"[{time.strftime('%H:%M:%S')}] sent {v:+.4f} m/s")

        time.sleep(args.dt)

except KeyboardInterrupt:
    print("\n[dummy_rt_agent] stopped by user.")
finally:
    READY.unlink(missing_ok=True)