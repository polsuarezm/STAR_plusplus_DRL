"""
train_drl.py
============
Trains a PPO agent on the StarCCMEnv using Stable-Baselines3.

Usage
-----
  # Manual mode (default): start STAR-CCM+ with the realtime macro yourself first
  python python/train_drl.py
  python python/train_drl.py --resume models/ppo_afc_cylinder
  python python/train_drl.py --eval   models/ppo_afc_cylinder

  # Auto-launch mode: Python spawns starccm+ each episode (HPC / batch)
  python python/train_drl.py --auto-launch

Dependencies
------------
  pip install stable-baselines3 gymnasium numpy torch
"""
 
import argparse
import os
import sys
import json
from pathlib import Path

# Ensure python/ directory is on the path when running from repo root
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    BaseCallback,
)
from stable_baselines3.common.monitor import Monitor
from gymnasium.wrappers import RescaleAction

from starccm_env import StarCCMEnv
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Config                                                                      #
# ─────────────────────────────────────────────────────────────────────────── #
 
CONFIG_PATH   = "config/case_config.json"
SIM_FILE      = "simulations/cylinder_afc_Re100.sim"
MACRO_FILE    = "macros/add_realtime_jets_from_json.java"
MODELS_DIR    = "models"
LOGS_DIR      = "logs"
MODEL_NAME    = "ppo_afc_cylinder"

def _load_starccm_cfg() -> dict:
    """Read starccm block from case_config.json; env var STARCCM_EXE overrides exe."""
    try:
        with open(CONFIG_PATH) as f:
            cfg = json.load(f)
        sc = cfg.get("starccm", {})
    except Exception:
        sc = {}
    sc["exe"] = os.environ.get("STARCCM_EXE", sc.get("exe", "starccm+"))
    return sc

STARCCM_CFG = _load_starccm_cfg()


def _steps_per_episode() -> int:
    """Compute agent steps per episode from config (n_steps // action_repeat)."""
    try:
        with open(CONFIG_PATH) as f:
            cfg = json.load(f)
        rt = cfg["jets"]["realtime"]
        return int(rt["n_steps"]) // max(1, int(rt.get("action_repeat", 1)))
    except Exception:
        return 333  # fallback if config unreadable


STEPS_PER_EPISODE = _steps_per_episode()

# PPO hyperparameters — tuned for AFC on a cylinder at Re=100
PPO_KWARGS = dict(
    learning_rate    = 3e-4,
    n_steps          = 64,       # steps per rollout per env (keep short: 1 ep = n_steps_total)
    batch_size       = 32,
    n_epochs         = 10,
    gamma            = 0.99,
    gae_lambda       = 0.95,
    clip_range       = 0.2,
    ent_coef         = 0.01,     # exploration bonus
    vf_coef          = 0.5,
    max_grad_norm    = 0.5,
    policy_kwargs    = dict(net_arch=[64, 64], log_std_init=-1.0),
    verbose          = 1,
    tensorboard_log  = LOGS_DIR,
)
 
TOTAL_TIMESTEPS = 100   # increase for longer training runs
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  STAR-CCM+ launch command builder                                            #
# ─────────────────────────────────────────────────────────────────────────── #
 
def build_starccm_cmd(sim_file: str, macro_file: str) -> list:
    exe     = STARCCM_CFG.get("exe", "starccm+")
    licpath = STARCCM_CFG.get("licpath", "")
    podkey  = STARCCM_CFG.get("podkey",  "")

    cmd = [exe, "-batch", "-macro", str(Path(macro_file).resolve()),
           str(Path(sim_file).resolve())]
    if licpath:
        cmd += ["-licpath", licpath]
    if podkey:
        cmd += ["-podkey", podkey]
    return cmd
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Logging callback                                                            #
# ─────────────────────────────────────────────────────────────────────────── #
 
class EpisodeLogger(BaseCallback):
    """Prints mean reward and episode length every N episodes."""
 
    def __init__(self, log_every: int = 5, verbose: int = 0):
        super().__init__(verbose)
        self.log_every = log_every
        self._ep_count = 0
        self._ep_rewards = []
 
    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        for info in infos:
            if "episode" in info:
                self._ep_count += 1
                self._ep_rewards.append(info["episode"]["r"])
                if self._ep_count % self.log_every == 0:
                    mean_r = np.mean(self._ep_rewards[-self.log_every:])
                    print(f"[EpisodeLogger] ep={self._ep_count}  "
                          f"mean_reward={mean_r:.4f}")
        return True
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Environment factory                                                         #
# ─────────────────────────────────────────────────────────────────────────── #
 
def make_env(config_path: str, sim_file: str, macro_file: str,
             auto_launch: bool = False, verbose: bool = False) -> StarCCMEnv:
    """
    Create and wrap a single StarCCMEnv.
    auto_launch=False: STAR-CCM+ must be started manually before calling reset().
    auto_launch=True:  Python spawns starccm+ subprocess each episode (HPC mode).
    """
    cmd = build_starccm_cmd(sim_file, macro_file) if auto_launch else None
    env = StarCCMEnv(
        config_path  = config_path,
        starccm_cmd  = cmd,
        verbose      = verbose,
    )
    # Rescale to [-1, 1] so PPO's Gaussian (std≈1 at init) doesn't saturate at ±a_max
    env = RescaleAction(env, min_action=-1.0, max_action=1.0)
    env = Monitor(env, filename=str(Path(LOGS_DIR) / MODEL_NAME))
    return env
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Train                                                                       #
# ─────────────────────────────────────────────────────────────────────────── #
 
def train(resume_path: str = None, auto_launch: bool = False):
    Path(MODELS_DIR).mkdir(exist_ok=True)
    Path(LOGS_DIR).mkdir(exist_ok=True)

    env = make_env(CONFIG_PATH, SIM_FILE, MACRO_FILE, auto_launch=auto_launch, verbose=True)
 
    if resume_path and Path(resume_path + ".zip").exists():
        print(f"[train] Resuming from {resume_path}")
        model = PPO.load(resume_path, env=env, **{
            k: v for k, v in PPO_KWARGS.items()
            if k not in ("verbose", "tensorboard_log", "policy_kwargs")
        })
    else:
        print("[train] Starting fresh PPO model")
        model = PPO("MlpPolicy", env, **PPO_KWARGS)
 
    callbacks = [
        CheckpointCallback(
            save_freq   = STEPS_PER_EPISODE,   # save once per episode
            save_path   = MODELS_DIR,
            name_prefix = MODEL_NAME,
            verbose     = 1,
        ),
        EpisodeLogger(log_every=5),
    ]
 
    print(f"[train] Training for {TOTAL_TIMESTEPS} timesteps...")
    model.learn(
        total_timesteps = TOTAL_TIMESTEPS,
        callback        = callbacks,
        reset_num_timesteps = (resume_path is None),
        tb_log_name     = MODEL_NAME,
    )
 
    save_path = str(Path(MODELS_DIR) / MODEL_NAME)
    model.save(save_path)
    print(f"[train] Model saved to {save_path}.zip")
 
    env.close()
    return model
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Evaluate                                                                    #
# ─────────────────────────────────────────────────────────────────────────── #
 
def evaluate(model_path: str, n_episodes: int = 5, auto_launch: bool = False):
    env = make_env(CONFIG_PATH, SIM_FILE, MACRO_FILE, auto_launch=auto_launch, verbose=True)
    model = PPO.load(model_path, env=env)
 
    print(f"[eval] Evaluating {n_episodes} episodes...")
    all_rewards = []
 
    for ep in range(n_episodes):
        obs, _ = env.reset()
        ep_reward = 0.0
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            done = terminated or truncated
        all_rewards.append(ep_reward)
        print(f"  Episode {ep+1}: total_reward={ep_reward:.4f}")
 
    print(f"[eval] Mean reward over {n_episodes} episodes: {np.mean(all_rewards):.4f}")
    env.close()
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Main                                                                        #
# ─────────────────────────────────────────────────────────────────────────── #
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train/evaluate PPO on cylinder AFC.")
    parser.add_argument(
        "--mode", choices=["train", "exploit"], default="train",
        help="train: PPO training with exploration (default). "
             "exploit: load model and run deterministically (no learning).",
    )
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to model (without .zip) to resume training from.")
    parser.add_argument("--model", type=str, default=None,
                        help="Path to model (without .zip) to load for --mode exploit.")
    parser.add_argument("--episodes", type=int, default=5,
                        help="Number of episodes for --mode exploit.")
    parser.add_argument("--auto-launch", action="store_true",
                        help="Auto-launch STAR-CCM+ each episode (default: manual).")
    args = parser.parse_args()

    if args.mode == "exploit":
        model_path = args.model or (str(Path(MODELS_DIR) / MODEL_NAME))
        print(f"[main] Exploitation mode — loading {model_path}")
        evaluate(model_path, n_episodes=args.episodes, auto_launch=args.auto_launch)
    else:
        print(f"[main] Exploration/training mode  (steps_per_episode={STEPS_PER_EPISODE})")
        train(resume_path=args.resume, auto_launch=args.auto_launch)