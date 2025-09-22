# Parallel Mechanisms in BRUCE: From General Formulation to MuJoCo XML

This document explains how BRUCE’s three parallel mechanisms are expressed mathematically (matching the notation in main.tex) and how those formulations are realized as MuJoCo XML equality constraints. It also shows how to model backlash/compliance via soft constraints and provides general XML snippets you can adapt to other robots.

- Mechanisms covered:
  - Hip differential pulley (2-DoF parallel actuation)
  - Five-bar linkage (closed chain via point coincidence)
  - Four-bar linkage (1-DoF nonlinear transmission)
- Backlash/compliance is modeled using MuJoCo’s soft equality constraints (solref/solimp).

Note: Files referenced in this repository: og_bruce.xml (XML implementation) and main.tex (general mathematical formulations and notation).

---

## MuJoCo soft equality constraints and backlash

MuJoCo treats all equality constraints as soft and enforces them with a virtual spring–damper system, modulated by a smoothly varying constraint impedance that depends on the violation. In the notation of main.tex:

$$
 a_{c1} + \mathcal{D}(r)\,(b_v\,v + k_v\,r) = (1 - \mathcal{D}(r))\,a_{c0}.
$$

- $r$: constraint violation
- $v$: relative velocity at the constraint
- $b_v, k_v$: damping and stiffness
- $\mathcal{D}(r)$: smooth sigmoid of $r$ that increases impedance outside a deadband $|r| < \epsilon_q$

How to tune in XML:
- `solref="tc zeta"` sets the spring–damper’s time constant and damping ratio.
- `solimp="imp_min imp_max width mid power"` shapes $\mathcal{D}(r)$:
  - width ≈ deadband size $\epsilon_q$ (low impedance region)
  - imp_min/imp_max bound the impedance range

Example used for five-bar closures:

```xml
<connect ... solref="0.005 1.05" solimp="0.2 0.95 0.002 0.9 6"/>
```

Here, width=0.002 creates a small deadband to model backlash/compliance before stiffening.

---

## 1) Hip differential pulley (2-DoF parallel actuation)

### General formulation (same notation as main.tex)
Let the two actuators be $q_L, q_R$ (positions/angles), and the outputs be $q_{\text{hip, roll}}$ and $q_{\text{hip, pitch}}$. Let $\rho_L, \rho_R$ be the gear ratios between the actuators and the differential idler. The velocity mapping is:

$$
\begin{bmatrix}
\dot q_{\text{hip, roll}} \\
\dot q_{\text{hip, pitch}}
\end{bmatrix}
=
\frac{1}{\rho_L + \rho_R}
\begin{bmatrix}
\rho_L & \rho_R \\
\rho_L & -\rho_R
\end{bmatrix}
\begin{bmatrix}
\dot q_L \\
\dot q_R
\end{bmatrix}.
$$

The same linear relation holds at the position level (up to a constant), so we enforce equality constraints between the output joints and linear combinations of actuator joints:

- Roll: $$q_{\text{hip, roll}} = \dfrac{\rho_L q_L + \rho_R q_R}{\rho_L + \rho_R}$$
- Pitch: $$q_{\text{hip, pitch}} = \dfrac{\rho_L q_L - \rho_R q_R}{\rho_L + \rho_R}$$

### XML realization pattern (tendons + equality)
Use tendons to build the linear combinations, and tie them with `equality/tendon` constraints. For arbitrary $\rho_L, \rho_R$:

```xml
<tendon>
  <!-- Output joints as 1× tendons -->
  <fixed name="hip_roll">
    <joint joint="hip_roll" coef="1"/>
  </fixed>
  <fixed name="hip_pitch">
    <joint joint="hip_pitch" coef="1"/>
  </fixed>

  <!-- Linear combinations of actuator joints -->
  <fixed name="sum_act">
    <joint joint="act_L" coef="{rho_L_over_sum}"/>
    <joint joint="act_R" coef="{rho_R_over_sum}"/>
  </fixed>
  <fixed name="diff_act">
    <joint joint="act_L" coef="{rho_L_over_sum}"/>
    <joint joint="act_R" coef="{-rho_R_over_sum}"/>
  </fixed>
</tendon>

<equality>
  <!-- out − 1·lincomb = 0  -->
  <tendon tendon1="hip_roll"  tendon2="sum_act"  polycoef="0 -1 0 0 0"/>
  <tendon tendon1="hip_pitch" tendon2="diff_act" polycoef="0 -1 0 0 0"/>
</equality>
```

Where:
- `rho_L_over_sum = \rho_L / (\rho_L + \rho_R)`
- `rho_R_over_sum = \rho_R / (\rho_L + \rho_R)`

Backlash deadband for the mapping (optional):

```xml
<equality>
  <tendon tendon1="hip_roll" tendon2="sum_act"  polycoef="0 -1 0 0 0"
          solref="0.005 1.0" solimp="0.2 0.95 0.002 0.9 6"/>
  <tendon tendon1="hip_pitch" tendon2="diff_act" polycoef="0 -1 0 0 0"
          solref="0.005 1.0" solimp="0.2 0.95 0.002 0.9 6"/>
</equality>
```

In `og_bruce.xml`, the special case $\rho_L = \rho_R$ is implemented with 0.5/±0.5 coefficients and identity equalities.

---

## 2) Five-bar linkage (closed chain via point coincidence)

### General formulation (same notation as main.tex)
Let the two planar chains end at $C$ and $D$ (chain joint angles $\theta_{5,i}$ and link lengths $l_{5,i}$):

$$
\begin{aligned}
\mathbf{p}_C &= \mathbf{R}(\theta_{5,1}) \begin{bmatrix} l_{5,1} \\ 0 \end{bmatrix}
               + \mathbf{R}(\theta_{5,2}) \begin{bmatrix} l_{5,2} \\ 0 \end{bmatrix},\\
\mathbf{p}_D &= \mathbf{R}(\theta_{5,4}) \begin{bmatrix} l_{5,4} \\ 0 \end{bmatrix}
               + \mathbf{R}(\theta_{5,3}) \begin{bmatrix} l_{5,3} \\ 0 \end{bmatrix},
\end{aligned}
$$

with $\mathbf{R}(\theta)\in\mathrm{SO}(2)$. Project to 3D via known fixed transforms to the base frame:

$$
\tilde{\mathbf{p}}_C = {}^{\text{b}}T_A \begin{bmatrix} \mathbf{p}_C \\ 0 \\ 1 \end{bmatrix}, \quad
\tilde{\mathbf{p}}_D = {}^{\text{b}}T_F \begin{bmatrix} \mathbf{p}_D \\ 0 \\ 1 \end{bmatrix}.
$$

Closed-chain constraint: $$\tilde{\mathbf{p}}_C \approx \tilde{\mathbf{p}}_D.$$

Actuated: $\theta_{5,1}, \theta_{5,4}$. Passive: $\theta_{5,2}, \theta_{5,3}$.

### XML realization pattern (auxiliary branch + connect)
Create a secondary kinematic branch to the alternative endpoint and close the loop with a connect equality. Use sites (recommended) to pick exact points to coincide:

```xml
<!-- endpoint sites on each chain -->
<site name="chainA_end" body="chainA_last" pos="xA yA zA"/>
<site name="chainB_end" body="chainB_last" pos="xB yB zB"/>

<equality>
  <connect site1="chainA_end" site2="chainB_end"
           solref="0.005 1.0" solimp="0.2 0.95 0.002 0.9 6"/>
</equality>
```

Initialization tip: Five-bars can have multiple IK branches and singularities; start from a feasible configuration (e.g., via a keyframe) to avoid large initial violations.

---

## 3) Four-bar linkage (1 actuator → 1 output with nonlinear transmission)

### General formulation (same notation as main.tex)
Velocity transmission ratio:

$$
\rho_4 = \frac{\dot{\theta}_{4,0}}{\dot{\theta}_{4,3}}
= \frac{L_{4,2}\,\sin(\theta_{4,1} - \theta_{4,2})}
       {L_{4,2}\,\sin(\theta_{4,1} - \theta_{4,2}) - L_{4,3}\,\sin(\theta_{4,1} - \theta_{4,3})}.
$$

Torque transmission is the reciprocal. For a parallelogram, $\rho$ is constant ($l_1/l_3$). Two practical XML approaches:

### Option A: Polynomial tendon equality (fast, approximate)
Enforce a position mapping $y = f(x)$ between the output $y=\theta_{4,3}$ and input $x=\theta_{4,0}$ using a polynomial around a nominal $(x_0,y_0)$:

$$
\mathbf{r}_f = y - y_0 - \mathbf{a}^\top \boldsymbol{\phi}(x - x_0) = 0,
\quad \boldsymbol{\phi}(x - x_0) = \begin{bmatrix} (x-x_0)^0 & \cdots & (x-x_0)^n \end{bmatrix}^\top.
$$

In XML:

```xml
<tendon>
  <fixed name="out4">
    <joint joint="theta4_out" coef="1"/>
  </fixed>
  <fixed name="in4">
    <joint joint="theta4_in" coef="1"/>
  </fixed>
</tendon>

<equality>
  <!-- y − (a0 + a1 x + a2 x^2 + ... + an x^n) = 0  -->
  <tendon tendon1="out4" tendon2="in4"
          polycoef="{-a0} {-a1} {-a2} ... {-an}"
          solref="0.005 1.0" solimp="0.2 0.95 0.0015 0.9 6"/>
</equality>
```

Notes:
- `polycoef` packs the polynomial applied to tendon2; `polycoef="0 -1 0 0 0"` reduces to identity (out − in = 0).
- Use a smaller width for tighter single-DoF mapping unless you want more free play.

### Option B: Closed-loop connect (exact geometry, captures singularities)
Mirror the five-bar approach: add a small virtual dyad to complete the four-bar and connect their endpoints with a site–site constraint:

```xml
<site name="crank_tip"  body="crank_body"  pos="x_c y_c z_c"/>
<site name="rocker_tip" body="rocker_body" pos="x_r y_r z_r"/>

<equality>
  <connect site1="crank_tip" site2="rocker_tip"
           solref="0.005 1.05" solimp="0.2 0.95 0.001 0.9 6"/>
</equality>
```

This captures singularities (e.g., collinearity) exactly and is preferred if the hardware can approach them.

---

## Backlash modeling cheatsheet

- Increase `solimp` width → larger deadband (more play)
- Decrease `solimp` width → crisper engagement
- Start from: `solref="0.005 1.0"`, `solimp="0.2 0.95 0.002 0.9 6"`
- Apply to:
  - `equality/connect` for five-/four-bars
  - `equality/tendon` for differential mappings

---

## Quick map from math (main.tex) to XML (og_bruce.xml)

- Differential (hip): tendons build sum/diff of actuator joints; `equality/tendon` ties outputs.
- Five-bar: auxiliary branch endpoints closed with `equality/connect` using a small deadband.
- Four-bar: not explicitly present in og_bruce.xml; use Option A (polynomial) or Option B (connect) above.

---

## Adapting to your robot (step-by-step)

1. Choose mechanism type:
   - Differential: derive $\rho_L, \rho_R$ from pulley/gear radii.
   - Five-bar: two serial chains closed by a point coincidence.
   - Four-bar: pick polynomial (fast) or connect-based (exact) modeling.
2. Encode the math in tendon coefficients or geometry:
   - Differential coefficients: $\big(\tfrac{\rho_L}{\rho_L+\rho_R}, \pm\tfrac{\rho_R}{\rho_L+\rho_R}\big)$.
   - Four-bar polynomial: use fitted $-a_i$ in `polycoef`.
3. Tune backlash with `solref/solimp`.
4. Initialize from a feasible configuration (keyframe) to avoid large initial violations.

---

## FAQ

- Why tendons for differentials? Tendons compute linear combinations of joint coordinates; `equality/tendon` then enforces equality to outputs.
- Why auxiliary bodies for five-bars/four-bars? MuJoCo’s kinematic tree is acyclic; closed chains are realized by adding a second branch and closing with an `equality/connect`.
- Equality choice?
  - `equality/tendon`: scalar coordinate relations $y=f(x)$.
  - `equality/connect`: enforce 3D point coincidence (preferred for multi-DoF closures).
- Non-unit gear ratios? Encode them in tendon `coef`; the equality often remains `polycoef="0 -1 0 0 0"`.
- Modeling bigger slop or stiction? Increase width, lower `imp_min`, and consider `joint frictionloss` on passive joints.
