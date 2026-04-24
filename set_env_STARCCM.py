"""
starccm_env.py
==============
Gymnasium-compatible environment that drives a STAR-CCM+ simulation
via the flag-file handshake protocol used by add_realtime_jets_from_json.java.

Handshake per step
------------------
  Python                            STAR-CCM+ macro
  ------                            ---------------
  wait for starccm_ready.flag  <──  written after each timestep completes
  delete starccm_ready.flag
  read observations from obs files
  compute action
  write jet_action.txt
  touch agent_ready.flag       ──>  macro reads action, runs next timestep

Observation space
-----------------
  Concatenation of velocity probe values read from
  jets.realtime.obs_file (one float per line, written by STAR-CCM+
  via a FieldMeanMonitor / PointProbe export).
  Shape: (n_probes,)  — configured in case_config.json

Action space
------------
  Box([-A_max], [A_max], shape=(1,))
  Single scalar: top-jet velocity [m/s]. Bottom jet is negated by macro.

Reward
------
  Negative drag coefficient proxy:  r = -Cd   (read from drag_file)
  Or a weighted combination:        r = -Cd + alpha * |Cl|

Usage
-----
  env = StarCCMEnv(config_path="case_config.json")
  obs, info = env.reset()
  obs, reward, terminated, truncated, info = env.step(action)
"""

import json
import os
import subprocess
import time
import signal
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from pathlib import Path


class StarCCMEnv(gym.Env):
    """
    Gymnasium environment for AFC on a cylinder using STAR-CCM+.

    Parameters
    ----------
    config_path : str
        Path to case_config.json
    starccm_cmd : list[str]
        Full command to launch STAR-CCM+ with the realtime macro, e.g.:
        ["starccm+", "-batch", "add_realtime_jets_from_json.java",
         "-simFile", "cylinder.sim"]
        If None, STAR-CCM+ must be started manually before calling reset().
    verbose : bool
        Print handshake debug info.
    """

    metadata = {"render_modes": []}

    def __init__(self, config_path: str, starccm_cmd: list = None, verbose: bool = False):
        super().__init__()

        self.config_path  = Path(config_path)
        self.starccm_cmd  = starccm_cmd
        self.verbose      = verbose
        self._process     = None   # STAR-CCM+ subprocess handle

        # ── load config ────────────────────────────────────────────────── #
        with open(config_path) as f:
            self.cfg = json.load(f)

        rt = self.cfg["jets"]["realtime"]
        self.action_file  = Path(rt["action_file"])
        self.ready_flag   = Path(rt["ready_flag"])
        self.obs_flag     = Path(rt["obs_flag"])
        self.poll_s       = rt.get("poll_interval_ms", 50) / 1000.0
        self.timeout_s    = rt.get("poll_timeout_s", 60)

        self.obs_file     = Path(rt["obs_file"])       # written by STAR-CCM+ probes
        self.drag_file    = Path(rt.get("drag_file", rt["obs_file"]))  # Cd source
        self.n_probes     = int(rt["n_probes"])        # observation vector length
        self.reward_alpha = float(rt.get("reward_alpha", 0.0))  # lift penalty weight

        amp = self.cfg["jets"]["amplitude"]
        self.a_max = float(amp)

        n_steps = self.cfg["time"]["steps"]
        self.max_episode_steps = int(n_steps)
        self._step_count = 0

        # ── spaces ─────────────────────────────────────────────────────── #
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self.n_probes,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-self.a_max, high=self.a_max,
            shape=(1,), dtype=np.float32
        )

    # ------------------------------------------------------------------ #
    #  reset                                                               #
    # ------------------------------------------------------------------ #
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Kill previous STAR-CCM+ process if still running
        self._kill_starccm()

        # Clean up leftover flag files from previous episode
        for f in [self.action_file, self.ready_flag, self.obs_flag]:
            try:
                f.unlink()
            except FileNotFoundError:
                pass

        self._step_count = 0

        # Launch STAR-CCM+ subprocess
        if self.starccm_cmd is not None:
            env = os.environ.copy()
            env["STARCCM_JSON"] = str(self.config_path.resolve())
            self._log("Launching STAR-CCM+...")
            self._process = subprocess.Popen(
                self.starccm_cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                # Don't block Python — STAR-CCM+ runs in background
            )
            self._log(f"PID: {self._process.pid}")

        # Wait for STAR-CCM+ to finish initialization and write the first obs_flag
        self._log("Waiting for initial starccm_ready.flag...")
        self._wait_for_flag(self.obs_flag)

        obs = self._read_obs()
        self._log(f"Initial obs received: shape={obs.shape}")

        return obs, {}

    # ------------------------------------------------------------------ #
    #  step                                                                #
    # ------------------------------------------------------------------ #
    def step(self, action: np.ndarray):

        # ── 1. consume obs_flag (we already read obs) ─────────────────── #
        try:
            self.obs_flag.unlink()
        except FileNotFoundError:
            pass

        # ── 2. write action ───────────────────────────────────────────── #
        jet_velocity = float(np.clip(action[0], -self.a_max, self.a_max))
        self.action_file.write_text(f"{jet_velocity:.10f}\n")
        self.ready_flag.touch()
        self._log(f"Step {self._step_count+1}: sent action={jet_velocity:.4f}")

        # ── 3. wait for STAR-CCM+ to finish the timestep ─────────────── #
        self._wait_for_flag(self.obs_flag)

        # ── 4. read observations and reward ───────────────────────────── #
        obs    = self._read_obs()
        reward = self._read_reward()

        self._step_count += 1
        terminated = False
        truncated  = self._step_count >= self.max_episode_steps

        # Check if STAR-CCM+ crashed
        if self._process is not None and self._process.poll() is not None:
            ret = self._process.returncode
            self._log(f"WARNING: STAR-CCM+ process exited with code {ret}")
            terminated = True

        return obs, reward, terminated, truncated, {"jet_velocity": jet_velocity}

    # ------------------------------------------------------------------ #
    #  close                                                               #
    # ------------------------------------------------------------------ #
    def close(self):
        self._kill_starccm()

    # ------------------------------------------------------------------ #
    #  internal helpers                                                    #
    # ------------------------------------------------------------------ #
    def _wait_for_flag(self, flag: Path):
        """Block until flag file exists, checking every poll_s seconds."""
        deadline = time.time() + self.timeout_s
        while not flag.exists():
            if time.time() > deadline:
                raise TimeoutError(
                    f"Timeout ({self.timeout_s}s) waiting for {flag}. "
                    "Check STAR-CCM+ is still running."
                )
            # If STAR-CCM+ died early, raise immediately
            if self._process is not None and self._process.poll() is not None:
                raise RuntimeError(
                    f"STAR-CCM+ process terminated unexpectedly "
                    f"(exit code {self._process.returncode})."
                )
            time.sleep(self.poll_s)

    def _read_obs(self) -> np.ndarray:
        """
        Read observation vector from obs_file.
        Format: one float per line (probe velocity magnitudes, Cp, etc.)
        STAR-CCM+ writes this file after each timestep via a macro or
        monitor export. Falls back to zeros if file is missing/malformed.
        """
        try:
            lines = self.obs_file.read_text().strip().splitlines()
            values = []
            for line in lines:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                # Support "name  value" or just "value" formats
                parts = line.split()
                values.append(float(parts[-1]))
            arr = np.array(values, dtype=np.float32)
            if len(arr) != self.n_probes:
                self._log(f"WARNING: expected {self.n_probes} obs, got {len(arr)}. Padding.")
                arr = np.resize(arr, self.n_probes).astype(np.float32)
            return arr
        except Exception as e:
            self._log(f"WARNING: could not read obs ({e}). Returning zeros.")
            return np.zeros(self.n_probes, dtype=np.float32)

    def _read_reward(self) -> float:
        """
        Read Cd (and optionally Cl) from drag_file.
        Expected format (one value per line):
          Cd_value
          Cl_value   (optional second line)
        Reward = -Cd - alpha * |Cl|
        """
        try:
            lines = [l.strip() for l in self.drag_file.read_text().strip().splitlines()
                     if l.strip() and not l.strip().startswith("#")]
            cd = float(lines[0].split()[-1])
            cl = float(lines[1].split()[-1]) if len(lines) > 1 else 0.0
            reward = -cd - self.reward_alpha * abs(cl)
            self._log(f"  Cd={cd:.4f}  Cl={cl:.4f}  r={reward:.4f}")
            return float(reward)
        except Exception as e:
            self._log(f"WARNING: could not read reward ({e}). Returning 0.")
            return 0.0

    def _kill_starccm(self):
        if self._process is not None:
            try:
                self._process.terminate()
                self._process.wait(timeout=10)
            except Exception:
                try:
                    self._process.kill()
                except Exception:
                    pass
            self._process = None

    def _log(self, msg: str):
        if self.verbose:
            print(f"[StarCCMEnv] {msg}")