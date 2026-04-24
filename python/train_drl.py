"""
train_drl.py
============
Trains a PPO agent on the StarCCMEnv using Stable-Baselines3.
 
Usage
-----
  python train_drl.py                         # train from scratch
  python train_drl.py --resume models/ppo_afc # resume from checkpoint
  python train_drl.py --eval  models/ppo_afc  # evaluate only
 
Dependencies
------------
  pip install stable-baselines3 gymnasium numpy torch
 
STAR-CCM+ must be accessible on PATH as "starccm+" (or set STARCCM_EXE).
The sim file and macros must already exist at the paths in case_config.json.
"""
 
import argparse
import os
import json
from pathlib import Path
 
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    BaseCallback,
)
from stable_baselines3.common.monitor import Monitor
 
from starccm_env import StarCCMEnv
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Config                                                                      #
# ─────────────────────────────────────────────────────────────────────────── #
 
CONFIG_PATH   = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/case_config.JSON"
SIM_FILE      = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/cylinder_afc_Re100.sim"
MACRO_FILE    = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/add_realtime_jets_from_json.java"
MODELS_DIR    = "models"
LOGS_DIR      = "logs"
MODEL_NAME    = "ppo_afc_cylinder"
 
# STAR-CCM+ executable — override with env var STARCCM_EXE if needed
STARCCM_EXE   = os.environ.get("STARCCM_EXE", "starccm+")
 
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
    policy_kwargs    = dict(net_arch=[64, 64]),
    verbose          = 1,
    tensorboard_log  = LOGS_DIR,
)
 
TOTAL_TIMESTEPS = 50_000   # increase for longer training runs
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  STAR-CCM+ launch command builder                                            #
# ─────────────────────────────────────────────────────────────────────────── #
 
def build_starccm_cmd(sim_file: str, macro_file: str) -> list:
    """
    Build the STAR-CCM+ batch command.
    Adjust flags for your licence server / installation as needed.
    """
    return [
        STARCCM_EXE,
        "-batch",          # no GUI
        "-macro", macro_file,
        sim_file,
        # Add licence flags here if needed, e.g.:
        # "-licpath", "1999@licence-server",
        # "-podkey",  "YOUR_POD_KEY",
    ]
 
 
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
             verbose: bool = False) -> StarCCMEnv:
    """
    Create and wrap a single StarCCMEnv.
    Note: SB3's VecEnv parallelism is NOT used here because each env
    needs its own STAR-CCM+ licence. Use n_envs=1.
    """
    cmd = build_starccm_cmd(sim_file, macro_file)
    env = StarCCMEnv(
        config_path  = config_path,
        starccm_cmd  = cmd,
        verbose      = verbose,
    )
    env = Monitor(env, filename=str(Path(LOGS_DIR) / MODEL_NAME))
    return env
 
 
# ─────────────────────────────────────────────────────────────────────────── #
#  Train                                                                       #
# ─────────────────────────────────────────────────────────────────────────── #
 
def train(resume_path: str = None):
    Path(MODELS_DIR).mkdir(exist_ok=True)
    Path(LOGS_DIR).mkdir(exist_ok=True)
 
    env = make_env(CONFIG_PATH, SIM_FILE, MACRO_FILE, verbose=True)
 
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
            save_freq   = 1000,
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
 
def evaluate(model_path: str, n_episodes: int = 5):
    env = make_env(CONFIG_PATH, SIM_FILE, MACRO_FILE, verbose=True)
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
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to existing model (without .zip) to resume training.")
    parser.add_argument("--eval",   type=str, default=None,
                        help="Path to model (without .zip) to evaluate only.")
    parser.add_argument("--episodes", type=int, default=5,
                        help="Number of evaluation episodes (used with --eval).")
    args = parser.parse_args()
 
    if args.eval:
        evaluate(args.eval, n_episodes=args.episodes)
    else:
        train(resume_path=args.resume)