# 🦾 Claw Stand (Work in Progress)

## Hello all! 

This project is inspired by the **BattleBots claw stand robot**:

<div align="center">
  <img src="assets/BattleBots.gif" alt="BATTLEBOTS" />
</div>

---

## Part 1: Doing Some Math

Each finger in the claw contains **3 motors** — **Motor A**, **Motor B**, and **Motor C**.  
Each motor rotates by an angle:  
- Motor A → $\theta_a$  
- Motor B → $\theta_b$  
- Motor C → $\theta_c$  

We need to perform **inverse kinematics** to find the angles needed to position the finger tip at a given **3D coordinate**:

$$(X_p, Y_p, Z_p)$$

<div align="center">
  <img src="assets/Ball_and_Stick_Diagram.png" alt="Diagram" width="750"/>
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

Substitute: $C' = C\sin\theta_b, \quad T' = T\sin\theta_d$

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

#### *Note: atan2 provides the quadrant and is used here, as atan loses information regarding negatives and associated quadrants.
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

Let $A = X_p$, $B = S$, and $R = \sqrt{A^2 + B^2}$.
  
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
\boxed{\theta_b = \phi \pm \arccos\left(\frac{K}{R}\right)}
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
\cos\theta_d = \frac{X_p - C\cos\theta_b}{T}
$$

$$
\sin\theta_d = \frac{S - C\sin\theta_b}{T}
$$

$$
\boxed{\theta_d = atan2(\frac{\sin\theta_d}{ \cos\theta_d})}.
$$


---

### Step 6 — Boundary Condition

The result of $\theta_b$ and $\theta_d$ can yield more than one solution. To guarantee that the joints bend outward:

$$
\theta_d > \theta_b
$$

Remembering that $\theta_d = \theta_b - \theta_c$ and substituting:

$$
\theta_c > 0
$$

guarantees a single solution.

---

### Step 7 — Python Implementation

```python
import numpy as np

def solve_thetas(Zp, Yp, Xp, A, B, C, T, Offset_R):
    """Returns the angles (in radians) of motors to obtain a point in space (Xp, Yp, Zp)"""
    Xp = Xp - Offset_R
    Yp = Yp

    # Helps with breakdown of atan2 at infinity as Yp -> 0
    if Yp < 0.0005 and Yp > 0:
        Yp = 0.0005        
    if Yp > -0.0005 and Yp <= 0:
        Yp = -0.0005
        

    theta_a = np.atan2((Zp-A),Yp)
    ca = np.cos(theta_a)
    
    S = Yp/ca - B
    R = np.hypot(Xp, S)           # sqrt(Xp^2 + S^2)
    phi = np.arctan2(S, Xp)
    K = (R*R + C*C - T*T) / (2.0*C)

    if abs(K/R) > 1.0 + 1e-12:
        return []  # no real solutions

    # clamp for numeric stability
    val = np.clip(K/R, -1.0, 1.0)
    acos_val = np.arccos(val)

    thetab_solutions = [phi + acos_val, phi - acos_val]
    solutions = []
    for tb in thetab_solutions:
        cb = np.cos(tb); sb = np.sin(tb)
        cd = (Xp - C*cb) / T
        sd = (S  - C*sb) / T
        # numeric clamp
        cd = np.clip(cd, -1.0, 1.0)
        sd = np.clip(sd, -1.0, 1.0)
        td = np.arctan2(sd, cd)

        # ensures only one solution
        tc = td - tb

        if tc > 0:
            solutions.append((theta_a, tb, td))
            
    return solutions

```

### Python Model Implementation 

[Model Source Code](https://github.com/drpykachu/Claw-Stand/blob/main/Edition%20V2.0%20-%20Stepper%20Motor/Software/Python/Windows%20-%20Claw%20Stick%20Model%20V2.2.py)

<div align="center">
  <img src="assets/Cropped Modeled Claw.gif" alt="Python Model" />
</div>

---

## Part 2: Picking Some Parts.

Here we will discuss the hardware necessary for porting over the mathmatical model to the a real world model. 

---
### The Motor:

There are several classes of motor that would work for this project, but servo motors are chosen as the range of movement (<180°), built-in rotary encoder (for position tracking and tuning with PID), and step accuracy (the smaller the better) are too good to pass up for this project. The mathemetical demonstration in the Python Model Implementation section above has the minimum and maximum angles for the motors as:

| Motor            | Min Angle | Max Angle | Delta Angle|
|------------------|------|------|------|
| Top (Motor C)    |  68.4    | 111.5     | 43.1   |
| Middle (Motor B) |  20.8    |  77.1    |  56.1   |
| Bottom (Motor A) |  87.8    |  109.7    |  21.9  |

I've used the [Waveshare 2.3kg Serial Bus Servo](https://www.waveshare.com/sc09-servo.htm) from Waveshare in the past with much success. Plus, these motors have a serial interface which allows them to be easily controlled by a Raspberry Pi, which I will also be using for the logic-processing aspect of this project.

### The Waveshare 2.3kg Serial Bus Servo Motor

These things pack quite a punch for their small size. The aspects we really care about for this project are:

| Specification                                     | Impact |
|---------------------------------------------------|--------|
| Dimensions: 23.2 × 12.0 × 25.5 mm                 | Positioning       |
| Position Sensor Resolution: 0.293° (300° / 1024)  | Positioning       |
| Gear Type: High-precision metal gear              | Positioning       |
| Max Locked-Rotor Torque: 2.3 kg·cm @ 6 V          | Torque Calculation       |
| Rated Torque: 0.7 kg·cm @ 6 V                     | Torque Calculation       |
| No-load Speed: 0.1 s / 60° (≈ 100 RPM) @ 6 V      | Torque Calculation       |
| Operating Voltage: 4.8 – 8.4 V                    | Power Supply Selection       |  
| No-load Current: 150 mA @ 6 V                     | Power Supply Selection       |
| Locked-Rotor Current (Stall): 1.0 A               | Power Supply Selection       |

### Positioning:

#### Dimensions:

Using the provided information of the motor sizes (23.2 × 12.0 × 25.5 mm), the Ball-And-Stick (BAS) model can be adjusted to reflect how these motors would behave. I used a CAD modeling software to create motor holders and joints to simulate how it would behave in real life. This is also a good sanity check to get $R_{offset}$, path height, and path radius to ensure the fingers don't crash into each other:

#### Position Sensor Resolution:

The BAS model has the oversight of showing us how the model would react with perfect decimal-point accuracy. In the real world, motors have an angle, the step angle or resolution angle, of how small they can exert movement in discrete steps. For instance, a large step can lead to unwanted behavior. Here is the model with a 2° step (notice the choppiness and inaccurate positioning on the the wanted path):

<div align="center">
  <img src="assets/Actual Claw Big Step.gif" alt="Actual Model 2°" />
</div>



<br>

Luckily, the Waveshare 2.3kg Serial Bus Servo Motor utilizes a gearbox to obtain a step size of 0.293°:

<div align="center">
  <img src="assets/Actual Claw.gif" alt="Actual Model" />
</div>


#### Gear Type:

Uh-oh..... "High-precision metal gear"..... "Gearbox".... that's not good. Gearboxes are good for increasing torque and increasing position resolution, but they are bad because they introduce *backlash*. Backlash is the clearance or lost motion in a mechanism caused by gaps between the parts. If there was no clearance between the gears, any anamoly would cause the gearbox to seize - causing lack or no motion and potential motor failure; so some backlash is desired. However, backlash in this system means that the rotor can move without the output shaft moving, causing a displacement between the modeled position and the actual position (*bad*).

Here is a picture depicting backlash:

<div align="center">
  <img src="assets/Backlash.png" alt="Backlash" />
</div>

See how one side of the teeth is engaged? We can use this to our advantage. Looking at bottom motor (Motor A) for example, we know that the gravity will pull the gearings to one side, until 90 degrees, where it will slop over to the other side. Assigning positive torque to a clockwise direction (and thus a negative torque is counter clockwise), we can deduce this diagram by finding the moment associated for each motor, where the moment is described as:

$$\tau = \sum_{i=1}^{n} l_i·m_i·g·cos(\theta); $$

$$\tau_C = [T(m_{plastic,T})]·g·cos(\theta)$$

$$\tau_B = [T(m_{plastic,T}) + (T+C)(m_{plastic}+m_{motor})]·g·cos(\theta) $$

$$\tau_B = [T(m_{plastic,T}) + (T+C)(m_{plastic}+m_{motor}) + (T+C+B)(m_{plastic}+m_{motor})]·g·cos(\theta) $$

<div align="center">
  <img src="assets/Torque_Raw.png" alt="Torque_Raw" />
</div>

To have the gears stay on one side, we will need to impose a torque associated with angle. Luckily, torsion springs are able to do so, with their torque equation is described as:

$$\tau_S = K_s\theta$$

We can increase the torque with the angle moved (assuming a 270° spring deflection) so that it preloads the gears when we get passed 90°, as such with a $K_S$ of 1.4 in·lbf/degree:

<div align="center">
  <img src="assets/Torque_Spring.png" alt="Torque_Spring" />
</div>

However, we must now check that the combined torque of the spring and the weight do not exceed the torque the motor can generate.

### Torque Calculation:

With the Waveshare SC09 servo rated at 0.7 kg·cm (≈ 68.6 mN·m) of torque at 6 V and a stall torque of 2.3 kg·cm (≈ 225.5 mN·m), achieving a torque of 1.3 mN·m is easily within the operating range. Since 1.3 mN·m is only about 0.013 kg·cm, it represents less than 2 % of the rated torque at 6 V. 

In practice, torque generated by the servo increases with higher applied voltage up to its specified limit, so operating at the full 6 V supply ensures we have a large torque margin above 1.3 mN·m even under load. Additionally, torque and speed are inversely related in hobby servos: as torque demand rises (closer to the stall torque), speed drops, so if your application requires maintaining torque above 1.3 mN·m under heavier loads, we should expect slower movement than the no-load speed of ~0.1 s/60° (≈100 RPM) at 6 V. So, lets assume that these motors can generate ~1.3 mN·m of torque and apply that to our diagram:

<div align="center">
  <img src="assets/Torque_Total.png" alt="Torque_Total" />
</div>

However, we forgot to include the load in our calculation. Right now, the diagram shows the only the torque associated with moving the finger itself, but we want to rotate and object and the plate as well. 

Bringing back this table:

| Motor            | Min Angle | Max Angle | Delta Angle|
|------------------|------|------|------|
| Top (Motor C)    |  68.4    | 111.5     | 43.1   |
| Middle (Motor B) |  20.8    |  77.1    |  56.1   |
| Bottom (Motor A) |  87.8    |  109.7    |  21.9  |

and adding the moment associated with a load to our torque calculations

$$\tau_{C,load} = [T(m_{load})]·g·cos(\theta)$$

$$\tau_{B,load} = [(C+T)(m_{load})]·g·cos(\theta)$$

$$\tau_{A,load} = [(B+C+T)(m_{load})]·g·cos(\theta)$$

we can predict the true behavior of the model (assuming a 0.5 lb load):


<div align="center">
  <img src="assets/Torque_Load.png" alt="Torque_Load" />
</div>

<br>


A half a pound is not a whole-lot of weight. But remember, these motors are small, and the entire claw is less than 7 inches in height, so it's pretty dang small. I'm surprised the math even works out that well in the first place. This model will serve as a proof of concept, with a bigger model with bigger (and more powerful) motors can be developed in the future.
