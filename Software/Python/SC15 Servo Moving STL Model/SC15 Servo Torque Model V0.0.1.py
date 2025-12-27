import numpy as np
import matplotlib.pyplot as plt


# Digit Lengths
A = 33.55 # bottom digit + motor A height
B = 43.55 # lower-middle digit + motor B height
C = 43.55 # upper-middle middle 2 digit + motor B height
T = 50    # top digit height

titles = ['Motor C Torque', 'Motor B Torque', 'Motor A Torque']

# individual sections
weight_bearing = 13 / 1000 # kg
weight_motor =   21 / 1000 # kg

Holder_Motor_B_Weight = 17 / 1000 #kg
Holder_Motor_C_Weight = 15 / 1000 #kg
Holder_Motor_T_Weight = 8  / 1000 #kg

# weights to move (worst case, at the end of the arm)
Holder_Motor_C_Weight2Move = Holder_Motor_T_Weight 
Holder_Motor_B_Weight2Move = Holder_Motor_C_Weight2Move + Holder_Motor_C_Weight + weight_motor + weight_bearing
Holder_Motor_A_Weight2Move = Holder_Motor_B_Weight2Move + Holder_Motor_B_Weight + weight_motor + weight_bearing


lengths = np.array([B,C,T])/1000
weights = [Holder_Motor_C_Weight2Move,Holder_Motor_B_Weight2Move,Holder_Motor_A_Weight2Move]

# Overall
motor_torque = 58.8399/1000 #N·m, Motor Strength
K_array = np.array([0,0,0]) # in•lbf, Spring Strength

W_plate = 1  /  2.20462 # kg
theta_deflection = 270
inlbf2Nm = 1 / 8.85075 # in•lbf / N•m


K_array = K_array*inlbf2Nm / theta_deflection  # N•m/theta
G = 9.8 # m /s^2

fig, axs = plt.subplots(3,1,sharex = True,figsize=(6, 9))
fig.canvas.manager.window.move(500,0)

################### Motor C ###################

# All points - plotted
x = np.linspace(0,180,361)

for i in range(0,len(axs.reshape(-1))):
    L = lengths[i]
    K = K_array[i]
    M = weights[i] + W_plate/4 # four active fingers at a time
    weight_torque_array = []
    spring_torque_array = []
    motor_torque_array = []
    
    
    for k in range(0,len(x)):    
        theta = x[k]
        weight_torque = L*M*G*np.cos(np.deg2rad(theta)) # (m)*(m/s^2)*kg = m*N
        spring_torque = K*theta
        weight_torque_array.append(weight_torque)
        spring_torque_array.append(spring_torque)
        motor_torque_array.append(motor_torque)    

    axs[i].plot(x,motor_torque_array,label = 'Motor', c = 'k')
    axs[i].plot(x,-np.array(motor_torque_array), c = 'k')
    axs[i].plot(x,weight_torque_array,label = 'Weight',c='b',alpha = 0.25)
    axs[i].plot(x,spring_torque_array,label = 'Spring',c='r',alpha = 0.25)
    axs[i].plot(x,x*0,'--k')
    axs[i].plot(x,np.array(weight_torque_array) + np.array(spring_torque_array),label = 'Total',c='m')
    axs[i].set_title(titles[i])

axs[0].set_ylabel('Torque / N•m')
axs[1].set_ylabel('Torque / N•m')
axs[2].set_ylabel('Torque / N•m')

axs[2].set_xlabel('Theta / $\Theta$')
axs[2].set_xlim([0,180])


axs[0].legend()
plt.show()