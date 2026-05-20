#!/usr/bin/env python3
"""
dummy_rt_sender.py

Simulates the Python-side of the STAR-CCM+ real-time handshake.
Run from the repo root while the realtime-jets macro is running in STAR-CCM+.

    python python/dummy_rt_sender.py --vmin -0.10 --vmax 0.10

Handshake (existence-based, no mtime):
  STAR-CCM+ creates  starccm_ready.flag  → Python detects, deletes it, sends action
  Python    creates  agent_ready.flag    → STAR-CCM+ detects, deletes it, advances solver
"""

import argparse, json, pathlib, random, time

# ------------------------------------------------------------------ #
# 1. parse CLI, seeding defaults from config/case_config.json
# ------------------------------------------------------------------ #
DEF_CFG = pathlib.Path("config/case_config.json")
defaults = {}
if DEF_CFG.exists():
    try:
        cfg = json.loads(DEF_CFG.read_text(encoding="utf-8"))
        rt  = cfg["jets"]["realtime"]
        defaults = dict(
            action_file = rt["action_file"],
            ready_flag  = rt["ready_flag"],
            obs_flag    = rt["obs_flag"],
        )
    except Exception as e:
        print(f"[dummy_rt_sender] Warning: could not parse {DEF_CFG}: {e}")

P = argparse.ArgumentParser(description="Send random jet actions to STAR-CCM+")
P.add_argument("--action-file", default=defaults.get("action_file", "runtime/jet_action.txt"))
P.add_argument("--ready-flag",  default=defaults.get("ready_flag",  "runtime/agent_ready.flag"))
P.add_argument("--obs-flag",    default=defaults.get("obs_flag",    "runtime/starccm_ready.flag"))
P.add_argument("--poll",  type=float, default=0.02,  help="poll interval in seconds (default 0.02)")
P.add_argument("--vmin",  type=float, default=-0.10, help="min random velocity [m/s]")
P.add_argument("--vmax",  type=float, default= 0.10, help="max random velocity [m/s]")
args = P.parse_args()

ACT   = pathlib.Path(args.action_file)
READY = pathlib.Path(args.ready_flag)
OBS   = pathlib.Path(args.obs_flag)

print(f"[dummy_rt_sender] poll={args.poll}s  range=[{args.vmin},{args.vmax}] m/s")
print("Press Ctrl-C to stop.\n")

def wait_and_consume(flag: pathlib.Path, poll: float) -> None:
    """Block until flag exists, then delete it (consume)."""
    while True:
        try:
            flag.unlink()   # raises FileNotFoundError if absent, PermissionError if locked
            return
        except FileNotFoundError:
            pass
        except PermissionError:
            pass  # STAR-CCM+ still has the file open; retry after poll
        time.sleep(poll)

try:
    while True:
        # 1) wait for STAR-CCM+ to signal (creates starccm_ready.flag)
        wait_and_consume(OBS, args.poll)

        # 2) generate & write new action
        v = random.uniform(args.vmin, args.vmax)
        ACT.parent.mkdir(parents=True, exist_ok=True)
        ACT.write_text(f"{v:.6f}\n", encoding="utf-8")

        # 3) signal STAR-CCM+ (create agent_ready.flag)
        READY.touch()
        print(f"[{time.strftime('%H:%M:%S')}] sent {v:+.4f} m/s")

except KeyboardInterrupt:
    print("\n[dummy_rt_sender] stopped by user.")
finally:
    READY.unlink(missing_ok=True)
