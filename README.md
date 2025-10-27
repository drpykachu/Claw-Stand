# Sonar Watch

### Hello all! 

### This project is inspired by the battle bots claw stand robot:

<div align="center">
  <img src="assets/BattleBots.gif" alt="BATTLEBOTS" />
</div>


### Each finger in the claw contains 3 motors - Motor A, Motor B, and Motor C. Each motor has an angle it can rotate - $\theta_a$, $\theta_b$, $\theta_c$. We need to perform inverse kinematics to find the angles need to obtain the positioning of the finger to a point in 3d-space - ($X_p$, $Y_p$, $Z_p$).   


<div align="center">
  <img src="assets/Ball_and_Stick_Diagram.png" alt="Diagram" width="400"/>
</div>


### Equation Setup - going from a point in space ($X_p$, $Y_p$, $Z_p$) to the angles $(θ_a, θ_b, θ_c)$

$\angle AB   =  \theta_a$
$\angle BC   =  \theta_b$
$\angle CT   =  \theta_c$
$\theta_d   =  \theta_b -\theta_c$

#### Deriving $Z_p$

$Z_p = B sin\theta_a   +   C' sin\theta_a   +   T'  sin \theta_a   +   A$
$\rightarrow   C'   =   C sin\theta_b$
$\rightarrow   T'   =   T sin\theta_d$

$Eq  (1):  Z_p = B sin\theta_a   +   C sin\theta_b sin\theta_a   +   T sin\theta_d  sin \theta_a  +   A$

#### Deriving $Y_p$

$Y_p = B cos\theta_a   +   C' cos\theta_a   +   T'  cos \theta_a$
$\rightarrow   C'   =   C sin\theta_b$
$\rightarrow   T'   =   T sin\theta_d$

$Eq  (2):  Y_p = B cos\theta_a   +   C sin\theta_b cos\theta_a   +   T sin\theta_d  cos\theta_a$
 
#### Deriving $X_p$

$Eq  (3):   X_p =   C cos\theta_b   +   T  cos \theta_d$

### Step 1 - Solving $\theta_a$

Visually, this is true, and is a combination of Eq (1) and Eq(2):  

$tan \theta_a   =   \frac{Z_p-A}{Y_p}$

$\theta_a   =   atan2(\frac{Z_p-A}{Y_p})$


to be continued...