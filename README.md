# OG BRUCE: Parallel Mechanisms Overview

Branch overview (choose a Git branch to get the variant you need):
- mjx: GPU MuJoCo (MJX) model without explicit backlash deadband in the closed-chain constraints.
- mjx_backlash: Same as mjx, but with additional backlash modeled on the five-bar closures (e.g., larger solimp width).
- WR_mjx_backlash: Westwood Robotics BRUCE production version (defaults and assets aligned with hardware release).

This repository contains the MuJoCo XML model of BRUCE and related assets. BRUCE includes three parallel mechanisms that are explicitly modeled in the XML using MuJoCo equality constraints. This README gives a quick overview of how they are set up in `og_bruce.xml` and points to a more general, reusable guide.

- MuJoCo model: [`og_bruce.xml`](./og_bruce.xml)
- General guide (math + reusable XML snippets): [`docs/parallel_mechanisms.md`](./docs/parallel_mechanisms.md)
- Math notation and derivations: [`main.tex`](./main.tex)

## How the parallel mechanisms are set in `og_bruce.xml`

In MuJoCo, closed-chain kinematics are modeled with soft equality constraints. BRUCE uses two main types:
- equality/tendon: enforce scalar relations between linear combinations of joint coordinates.
- equality/connect: enforce coincidence of two points in 3D (bodies or sites).

Below is how each mechanism is realized in `og_bruce.xml` (see around lines 320–366 in the current file), followed by actual XML snippets.

1) Hip differential pulley (2-DoF parallel actuation)
- Tendons build linear combinations of the two actuator joints.
  - Right leg: `sum_r` (0.5/0.5) and `diff_r` (0.5/−0.5); outputs `roll_r`, `pitch_r`.
  - Left leg: `sum_l`, `diff_l`; outputs `roll_l`, `pitch_l`.
- equality/tendon ties outputs to those combinations with `polycoef="0 -1 0 0 0"`, i.e., output − 1·combination = 0.
- See `tendon` and `equality` blocks around:
  - Tendons: lines ~320–351
  - Equalities: lines ~354–360

2) Five-bar linkage (leg closed chain)
- An auxiliary kinematic branch mirrors one side of the linkage (e.g., `knee_pitch_*0 → ankle_pitch_*1 → knee_pitch_*1`).
- equality/connect closes the loop by coinciding the auxiliary endpoint with the main chain’s ankle body.
- Soft-constraint parameters (`solref`, `solimp`) add a small deadband to capture backlash/compliance.
- See `equality/connect` around lines ~361–366.

3) Four-bar linkage (1 DoF nonlinear transmission)
- Not explicitly added in this XML. Two options are provided in the general guide:
  - Polynomial tendon equality (fast, approximate)
  - Connect-based closure (exact geometry, captures singularities)

### XML snippets by mechanism

#### Differential drive (hip)

Tendons and equalities for both hips (differential mapping):

```xml
<tendon>
    <!-- Parallel mechanism for right leg -->
    <fixed name="roll_r">
        <joint joint="hip_roll_r" coef="1"/>
    </fixed>
    <fixed name="pitch_r">
        <joint joint="hip_pitch_r" coef="1"/>
    </fixed>
    <fixed name="sum_r">
        <joint joint="hip_actuatorR_r" coef="0.5"/>
        <joint joint="hip_actuatorR_l" coef="0.5"/>
    </fixed>
    <fixed name="diff_r">
        <joint joint="hip_actuatorR_r" coef=".5"/>
        <joint joint="hip_actuatorR_l" coef="-.5"/>
    </fixed>

    <!-- Parallel mechanism for left leg -->
    <fixed name="roll_l">
        <joint joint="hip_roll_l" coef="1"/>
    </fixed>
    <fixed name="pitch_l">
        <joint joint="hip_pitch_l" coef="1"/>
    </fixed>
    <fixed name="sum_l">
        <joint joint="hip_actuatorL_r" coef="0.5"/>
        <joint joint="hip_actuatorL_l" coef="0.5"/>
    </fixed>
    <fixed name="diff_l">
        <joint joint="hip_actuatorL_r" coef=".5"/>
        <joint joint="hip_actuatorL_l" coef="-.5"/>
    </fixed>
</tendon>

<equality>
    <!-- For right differential mechanism -->
    <tendon tendon1="roll_r" tendon2="sum_r" polycoef="0 -1 0 0 0"/>
    <tendon tendon1="pitch_r" tendon2="diff_r" polycoef="0 -1 0 0 0"/>
    <!-- For left differential mechanism -->
    <tendon tendon1="roll_l" tendon2="sum_l" polycoef="0 -1 0 0 0"/>
    <tendon tendon1="pitch_l" tendon2="diff_l" polycoef="0 -1 0 0 0"/>
</equality>
```

#### Five-bar linkage (leg closed chain)

Five-bar closure using a soft connect with backlash deadband:

```xml
<equality>
    <connect anchor="0 0 0" solref="0.005 1.05" solimp="0.2 0.95 0.002 0.9 6"
             body1="knee_pitch_r1" body2="ankle_pitch_r" name="equality_knee_r"/>
    <connect anchor="0 0 0" solref="0.005 1.05" solimp="0.2 0.95 0.002 0.9 6"
             body1="knee_pitch_l1" body2="ankle_pitch_l" name="equality_knee_l"/>
</equality>
```

For the full explanation, equations, and general XML patterns (including gear ratios and four-bar options), see the guide below.

## General formulation and reusable XML
- Read the full guide: [`docs/parallel_mechanisms.md`](./docs/parallel_mechanisms.md)
  - Same math notation as the paper’s [`main.tex`](./main.tex)
  - Ready-to-adapt XML snippets for:
    - Differential pulley with arbitrary gear ratios
    - Five-bar point-coincidence closures
    - Four-bar (polynomial or connect-based)
  - Backlash/compliance tuning via `solref`/`solimp`

## Tips
- Start simulations from a feasible configuration (see keyframes in `og_bruce.xml`) to avoid large initial constraint violations.
- Adjust `solimp` width for more/less backlash deadband; tune `solref` for convergence speed and damping.
- For differentials with non-unit gear ratios, set tendon `coef` according to the normalized mapping; the equality typically stays `polycoef="0 -1 0 0 0"`.

If you are building your own robot, the guide’s XML snippets can be copy-pasted and adapted by changing joint and body names, gear ratios, and link geometry.
