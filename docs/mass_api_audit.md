# MASS C++ API Audit

> Audit of MASS codebase methods for the Exoskeleton RL project (Phase 1).
> Maps existing MASS methods to paper requirements (Luo et al., Nature 2024).

---

## 1. Skeleton & DOF Layout

| Body Node | Parent | Joint Type | DOFs | DOF Indices | Mass (kg) |
|-----------|--------|------------|------|-------------|-----------|
| Pelvis | None | Free | 6 | [0..5] | 15.0 |
| FemurR | Pelvis | Ball | 3 | [6..8] | 7.0 |
| TibiaR | FemurR | Revolute | 1 | [9] | 3.0 |
| TalusR | TibiaR | Ball | 3 | [10..12] | 0.6 |
| FootThumbR | TalusR | Revolute | 1 | [13] | 0.2 |
| FootPinkyR | TalusR | Revolute | 1 | [14] | 0.2 |
| FemurL | Pelvis | Ball | 3 | [15..17] | 7.0 |
| TibiaL | FemurL | Revolute | 1 | [18] | 3.0 |
| TalusL | TibiaL | Ball | 3 | [19..21] | 0.6 |
| FootThumbL | TalusL | Revolute | 1 | [22] | 0.2 |
| FootPinkyL | TalusL | Revolute | 1 | [23] | 0.2 |
| Spine | Pelvis | Ball | 3 | [24..26] | 5.0 |
| Torso | Spine | Ball | 3 | [27..29] | 10.0 |
| Neck | Torso | Ball | 3 | [30..32] | 2.0 |
| Head | Neck | Ball | 3 | [33..35] | 2.0 |
| ShoulderR | Torso | Ball | 3 | [36..38] | 1.0 |
| ArmR | ShoulderR | Ball | 3 | [39..41] | 1.0 |
| ForeArmR | ArmR | Revolute | 1 | [42] | 0.5 |
| HandR | ForeArmR | Ball | 3 | [43..45] | 0.2 |
| ShoulderL | Torso | Ball | 3 | [46..48] | 1.0 |
| ArmL | ShoulderL | Ball | 3 | [49..51] | 1.0 |
| ForeArmL | ArmL | Revolute | 1 | [52] | 0.5 |
| HandL | ForeArmL | Ball | 3 | [53..55] | 0.2 |

**Totals:** 23 bodies, 56 DOFs (6 root + 50 active), 61.4 kg total mass
**Scale factor for 72 kg:** 1.1726

### Hip Flexion/Extension DOF Indices

DART BallJoint uses exponential map (x, y, z). Joint frames are identity rotation.
Hip flex/ext is rotation around X axis (lateral axis).

| Joint | Full DOF Indices | Flex/Ext DOF | Active DOF Index (excl root) |
|-------|-----------------|--------------|------------------------------|
| FemurR | [6, 7, 8] | **6** | 0 |
| FemurL | [15, 16, 17] | **15** | 9 |

For ExoNN input: `skel->getPosition(6)`, `getVelocity(6)` for right hip;
`skel->getPosition(15)`, `getVelocity(15)` for left hip.

---

## 2. Environment API - Existing Methods

### Environment.h/.cpp

| Method | Signature | Status | Notes |
|--------|-----------|--------|-------|
| SetAction | `SetAction(VectorXd a)` | EXISTS | Stores `a*0.1`, gets BVH targets. Maps to SetHumanActions. |
| GetDesiredTorques | `() -> VectorXd(50)` | EXISTS | SPD controller. Returns tau excl root. Stores in mDesiredTorque. |
| GetMuscleTorques | `() -> VectorXd(1098)` | EXISTS | Compressed JtA per muscle. |
| SetActivationLevels | `(VectorXd)` | EXISTS | Sets 284 muscle activations. |
| Step | `()` | EXISTS | Single physics substep. Applies muscle forces, samples tuple at random index. |
| Reset | `(bool RSI)` | EXISTS | Resets world, optional random BVH time init. |
| IsEndOfEpisode | `() -> bool` | EXISTS | Checks: root_y < 1.3m, NaN, time > 10s. |
| GetState | `() -> VectorXd(136)` | EXISTS | 22x3 body pos + 23x3 vel + 1 phase. Scaled: pos*0.8, vel*0.2. |
| GetReward | `() -> double` | EXISTS | `r = r_ee*(w_q*r_q + w_v*r_v)`. DIFFERENT from paper. |
| GetMuscleTuples | `() -> vector<MuscleTuple>` | EXISTS | Returns collected tuples. |

### Character.h/.cpp

| Method | Status | Notes |
|--------|--------|-------|
| `LoadSkeleton(path)` | EXISTS | Builds DART skeleton from XML. |
| `LoadMuscles(path)` | EXISTS | Parses 284 muscles with waypoints. |
| `LoadBVH(path, cyclic)` | EXISTS | Loads BVH reference motion. |
| `SetPDParameters(kp, kv)` | EXISTS | Default: kp=300, kv=sqrt(600). |
| `GetSPDForces(p_desired)` | EXISTS | Stable PD using mass matrix. |
| `GetTargetPosAndVel(t, dt)` | EXISTS | BVH reference at time t. |

### Muscle.h/.cpp

| Method | Status | Notes |
|--------|--------|-------|
| `Update()` | EXISTS | Updates anchor positions and length. |
| `ApplyForceToBody()` | EXISTS | Applies force along waypoint directions. |
| `GetForce()` | EXISTS | `f0*g_al*activation + f0*g_pl`. |
| `GetJacobianTranspose()` | EXISTS | Full Jacobian (n_dof x 3*n_anchors). |
| `GetRelatedJtA()` | EXISTS | Compressed JtA (non-zero DOFs only). |
| `g_al(l_m)` | EXISTS | Active force-length: `exp(-(l_m-1)^2/0.5)`. |
| `g_pl(l_m)` | EXISTS | Passive force-length. |

### EnvManager.h/.cpp (pybind11 bindings)

| Python Method | Returns | Notes |
|--------------|---------|-------|
| `GetStates()` | Matrix(N,136) | All envs in parallel |
| `SetActions(Matrix)` | void | N x 50 |
| `Steps(num)` | void | OpenMP parallel |
| `StepsAtOnce()` | void | sim_hz/con_hz substeps |
| `Resets(RSI)` | void | All envs |
| `IsEndOfEpisodes()` | Vector(N) | |
| `GetRewards()` | Vector(N) | |
| `GetMuscleTorques()` | Matrix(N,1098) | OpenMP |
| `GetDesiredTorques()` | Matrix(N,50) | OpenMP |
| `SetActivationLevels(Matrix)` | void | N x 284 |
| `ComputeMuscleTuples()` | void | Collect+stack from all envs |
| `GetMuscleTuplesJtA()` | Matrix(T,1098) | |
| `GetMuscleTuplesTauDes()` | Matrix(T,50) | |
| `GetMuscleTuplesL()` | Matrix(T,14200) | 50*284 vectorized |
| `GetMuscleTuplesb()` | Matrix(T,50) | |

---

## 3. Dimension Summary

| Quantity | Value | Source |
|----------|-------|--------|
| State dim (human obs) | 136 | 22x3 pos + 23x3 vel + 1 phase |
| Action dim (human) | 50 | Active DOFs |
| Muscle activations | 284 | All muscles |
| Muscle torques (JtA) | 1098 | Sum of related DOFs per muscle |
| Desired torques | 50 | Active DOFs |
| L matrix (vectorized) | 14200 | 50 x 284 |
| b vector | 50 | Active DOFs |
| Exo obs (needed) | 16 | 4 values x 4 timesteps |
| Exo actions (needed) | 2 | L/R hip position targets |

---

## 4. Key Differences: MASS Default vs Paper

### PD Controller
- **MASS:** SPD (Stable PD, mass-matrix inversion), kp=300, kv=sqrt(600)=24.5
- **Paper:** PD (simple), kp=50, kv=14.14=sqrt(2*50)
- **Note:** SPD is more stable at larger timesteps. Consider keeping SPD structure with paper gains.

### Simulation Rates
- **MASS:** 600 Hz physics / 30 Hz control = 20 substeps
- **Paper:** 500 Hz physics / 100 Hz control = 5 substeps

### Reward Function
- **MASS:** `r = r_ee * (w_q*r_q + w_v*r_v)`
- **Paper:** `r_human = 0.75*r_p + 0.5*r_root + 0.2*r_cop` (different formulation)

### Action Scaling
- MASS multiplies actions by 0.1 internally in `SetAction()`.

### Muscle Tuple Sampling
- MASS samples ONE tuple per control step at a random substep index.
- Not every substep generates training data (data efficiency trick).

### Muscle NN Calls Per Control Step
- MASS calls muscle NN multiple times per control step (sim_per_control/2 = 10 calls with Steps(2)).
- Paper pseudocode calls it once per control step.
- MASS approach is more physically accurate (muscle activations update every 2 substeps).

---

## 5. Methods NEEDED for Phase 4

| Method | Type | Description |
|--------|------|-------------|
| `GetExoObservations()` | NEW | 16-dim hip angle/vel history buffer |
| `SetExoActions()` | NEW | Exo PD control + clamp +/-18 Nm |
| `GetHumanReward()` | NEW | Paper eqs. 7-9 |
| `GetExoReward()` | NEW | Paper eqs. 13-14 |
| Exoskeleton loading | NEW | Exo skeleton + weld + bushing |
| Hip history buffer | NEW | Ring buffer: 4 timesteps x 4 values |
| Exo action history | NEW | For smoothness reward |
| Domain randomization | NEW | On Reset() |
| `IsEndOfEpisode()` | MODIFY | Add pelvis tilt check |
| PD gains | MODIFY | kp 300->50, kv 24.5->14.14 |
| Sim rates | MODIFY | 600/30 -> 500/100 Hz |

### Mapping: Paper -> MASS Existing

| Paper Requirement | MASS Method | Action |
|-------------------|-------------|--------|
| `SetHumanActions(a)` | `SetAction(a)` | Reuse (stores pos offsets) |
| `GetDesiredTorquesHuman()` | `GetDesiredTorques()` | Reuse (SPD tau, 50-dim) |
| `GetMuscleTorques()` | `GetMuscleTorques()` | Reuse (JtA, 1098-dim) |
| `SetActivationLevels(a)` | `SetActivationLevels(a)` | Reuse directly |
| `Steps()` | `Step()` x N | Reuse, outer loop in Python |
| `Reset(j)` | `Reset(RSI)` | Reuse + add domain rand |
| `GetHumanObservations()` | `GetState()` | Reuse (136-dim) |
| `GetMuscleTuples()` | `ComputeMuscleTuples()+Get*()` | Reuse via EnvManager |

---

## 6. MASS Training Loop (main.py) Reference

```
for iteration:
    for step in rollout:
        states = env.GetStates()                 # 16 x 136
        actions = human_nn.get_action(states)    # 16 x 50
        env.SetActions(actions)                  # pos offsets * 0.1

        # Muscle loop: runs sim_per_control/2 times with Steps(2)
        for sub in range(sim_per_control // 2):
            mt = env.GetMuscleTorques()          # 16 x 1098
            dt = env.GetDesiredTorques()         # 16 x 50
            activations = muscle_nn(mt, dt)      # 16 x 284
            env.SetActivationLevels(activations)
            env.Steps(2)                         # 2 physics steps

        rewards = env.GetRewards()

    # PPO update human NN
    # Supervised update muscle NN via ComputeMuscleTuples()
```
