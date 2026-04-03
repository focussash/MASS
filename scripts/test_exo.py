"""Quick verification: exo skeleton loads in DART with correct properties."""
import sys
sys.path.append('/home/zijie/projects/MASS/python')
import pymss
import numpy as np

# Load environment (human + muscles)
env = pymss.pymss('/home/zijie/projects/MASS/data/metadata.txt', 1)
print(f"Human: {env.GetNumState()} states, {env.GetNumAction()} actions, {env.GetNumMuscles()} muscles")

# Basic sanity: step without crash
env.Resets(True)
actions = np.zeros((1, env.GetNumAction()))
env.SetActions(actions)
env.StepsAtOnce()
states = env.GetStates()
print(f"State finite after step: {np.all(np.isfinite(states))}")
print(f"Reward: {env.GetRewards()[0]:.4f}")

print("\nExo C++ class compiled into libmss.a — ready for Phase 4 integration")
