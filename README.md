# 🦾 Claw Stand

### Hello all! 

This project is inspired by the **BattleBots claw stand robot**:

<div align="center">
  <img src="assets/BattleBots.gif" alt="BATTLEBOTS" />
</div>

---

### Overview

Each finger in the claw contains **3 motors** — **Motor A**, **Motor B**, and **Motor C**.  
Each motor rotates by an angle:  
- Motor A → $\theta_a$  
- Motor B → $\theta_b$  
- Motor C → $\theta_c$  

We need to perform **inverse kinematics** to find the angles needed to position the finger tip at a given **3D coordinate**:

$$(X_p, Y_p, Z_p)$$

<div align="center">
  <img src="assets/Ball_and_Stick_Diagram.png" alt="Diagram" width="400"/>
</div>

---

## Equation Setup

We’re going from a target point in space $(X_p, Y_p, Z_p)$ to joint angles $(\theta_a, \theta_b, \theta_c)$.

$$
\angle AB = \theta_a \quad \angle BC = \theta_b \quad \angle CT = \theta_c
$$

We also define:

$$
\theta_d = \theta_b - \theta_c
$$

---

### Deriving $Z_p$

$$
Z_p = B\sin\theta_a + C'\sin\theta_a + T'\sin\theta_a + A
$$

Substitute: $C' = C\sin\theta_b, \quad T' = T\sin\theta_d $

So:

$$
\boxed{Z_p = B\sin\theta_a + C\sin\theta_b\sin\theta_a + T\sin\theta_d\sin\theta_a + A} \quad \text{(Eq. 1)}
$$

---

### Deriving $Y_p$

$$
Y_p = B\cos\theta_a + C'\cos\theta_a + T'\cos\theta_a
$$

Substitute: $ C' = C\sin\theta_b, \quad T' = T\sin\theta_d $

So:

$$
\boxed{Y_p = B\cos\theta_a + C\sin\theta_b\cos\theta_a + T\sin\theta_d\cos\theta_a} \quad \text{(Eq. 2)}
$$

---

### Deriving $X_p$

$$
\boxed{X_p = C\cos\theta_b + T\cos\theta_d} \quad \text{(Eq. 3)}
$$

---

## Step 1 — Solving for $\theta_a$

Visually, this is true from Eq. (1) and Eq. (2):

$$
\tan\theta_a = \frac{Z_p - A}{Y_p}
$$

So:

$$
\theta_a = atan2(\frac{Z_p - A}{Y_p})
$$

Note - atan2 provides quadrant as atan loses information regarding negatives.
---

## Step 2 — Solving for $\theta_b$ and $\theta_d$

We now solve for $\theta_b$ and $\theta_d$, given:

$$
Y_p = B\cos\theta_a + C\sin\theta_b\cos\theta_a + T\sin\theta_d\cos\theta_a,
$$

$$
X_p = C\cos\theta_b + T\cos\theta_d.
$$

---

### Step 1 — Simplify the equations

Divide the first equation by $\cos\theta_a$ (assuming $\cos\theta_a \neq 0$) and define:

$$
S = \frac{Y_p}{\cos\theta_a} - B.
$$

Then the system becomes:

$$
C\sin\theta_b + T\sin\theta_d = S,  
$$

$$
C\cos\theta_b + T\cos\theta_d = X_p.
$$

---

### Step 2 — Interpret geometrically

These equations represent a **vector addition** in the plane:

$$
C e^{i\theta_b} + T e^{i\theta_d} = X_p + iS \equiv R e^{i\phi},
$$

where

$$
R = \sqrt{X_p^2 + S^2}, \quad \phi = atan2(\frac{S}{X_p}).
$$

---

### Step 3 — Eliminate $\theta_d$

Using the magnitude condition for the complex sum, we find:

$$
X_p\cos\theta_b + S\sin\theta_b = K,
$$

where

$$
K = \frac{R^2 + C^2 - T^2}{2C}.
$$

Let $ A = X_p $, $ B = S $, and $ R = \sqrt{A^2 + B^2} $.  
Then:

$$
A\cos\theta_b + B\sin\theta_b = K
$$

can be rewritten as:

$$
R\cos(\theta_b - \phi) = K.
$$

---

### Step 4 — Solve for $\theta_b$

$$
\boxed{\theta_b = \phi \pm \arccos\!\left(\frac{K}{R}\right)}
$$

Feasibility condition:

$$
\left|\frac{K}{R}\right| \le 1.
$$

If \(|K/R| > 1\), no real solution exists.

---

### Step 5 — Solve for $\theta_d$

For each valid $\theta_b$:

$$
\cos\theta_d &= \frac{X_p - C\cos\theta_b}{T},\$$4pt]
\sin\theta_d &= \frac{S - C\sin\theta_b}{T},\$$4pt]
\boxed{\theta_d = atan2(\frac{\sin\theta_d}{ \cos\theta_d)}}.
$$

---

### Step 6 — Python Implementation

```python
import numpy as np

def solve_thetas(Yp, Xp, B, C, T, theta_a):
    ca = np.cos(theta_a)
    if abs(ca) < 1e-12:
        raise ValueError("cos(theta_a) ~ 0; handle that special case separately.")

    S = Yp / ca - B
    R = np.hypot(Xp, S)
    phi = np.arctan2(S, Xp)
    K = (R**2 + C**2 - T**2) / (2 * C)

    if abs(K / R) > 1.0 + 1e-12:
        return []  # No real solutions

    val = np.clip(K / R, -1.0, 1.0)
    acos_val = np.arccos(val)

    thetab_solutions = [phi + acos_val, phi - acos_val]
    solutions = []

    for tb in thetab_solutions:
        cb, sb = np.cos(tb), np.sin(tb)
        cd = (Xp - C * cb) / T
        sd = (S - C * sb) / T
        cd, sd = np.clip(cd, -1.0, 1.0), np.clip(sd, -1.0, 1.0)
        td = np.arctan2(sd, cd)
        solutions.append((tb, td))

    return solutions
