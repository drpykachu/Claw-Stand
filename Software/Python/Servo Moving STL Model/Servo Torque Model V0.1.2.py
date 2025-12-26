import numpy as np
import matplotlib.pyplot as plt


# Digit Lengths
A = 33.55 # mm, bottom digit + motor A height
B = 43.55 # mm, lower-middle digit + motor B height
C = 43.55 # mm, upper-middle middle 2 digit + motor B height
T = 43.55    # mm, top digit height

titles = ['Motor C Torque', 'Motor B Torque', 'Motor A Torque']

# individual sections
A = A/1000 # m, bottom digit + motor A height
B = B/1000 # m, lower-middle digit + motor B height
C = C/1000 # m, upper-middle middle 2 digit + motor B height
T = T/1000 # m, top digit height


weight_bearing = 10 / 1000 # kg
weight_motor =   21 / 1000 # kg

buffer = 5
Holder_Motor_B_Weight = 17 / 1000 + buffer/1000 #kg
Holder_Motor_C_Weight = 15 / 1000 + buffer/1000  #kg
Holder_Motor_T_Weight = 8  / 1000 + buffer/1000  #kg

# weights to move
W_plate =  0  /  2.20462 # kg, only 4 fingers will hold at a time
W_plate = W_plate/4 # kg, only 4 fingers will hold at a time

Holder_Motor_C_Weight2Move = [0, 0, Holder_Motor_T_Weight+W_plate]
Holder_Motor_B_Weight2Move = [0, Holder_Motor_C_Weight + weight_motor + weight_bearing, Holder_Motor_T_Weight+W_plate]
Holder_Motor_A_Weight2Move = [Holder_Motor_B_Weight + weight_motor + weight_bearing, Holder_Motor_C_Weight + weight_motor + weight_bearing, Holder_Motor_T_Weight+W_plate]

# Lengths to move
Holder_Motor_C_Length2Move = [0,0,T]
Holder_Motor_B_Length2Move = [0,B,T]
Holder_Motor_A_Length2Move = [C,B,T]

lengths = np.array([Holder_Motor_C_Length2Move,Holder_Motor_B_Length2Move,Holder_Motor_A_Length2Move])
weights = np.array([Holder_Motor_C_Weight2Move,Holder_Motor_B_Weight2Move,Holder_Motor_A_Weight2Move])
moments = lengths*weights
moments = sum(moments.transpose()) # combines to find a single moment, order of C, B, A

# Overall
# The torque (stalled) is 2.3kg.cm@6V
# The torque (moving)  is 0.7kg.cm@6V
kgcm2mNm = 98.0665 # N•m / Kg•cm
motor_torque = 1.29 #Kg•cm

motor_torque = motor_torque * kgcm2mNm / 1000 #N·m, Motor Strength
K_constant = 1.4  # in•lbf, Spring Strength
K_array = np.ones(3)*K_constant

inlbf2Nm = 1 / 8.85075 # in•lbf / N•m
theta_deflection = 270
K_array = K_array*inlbf2Nm / theta_deflection  # N•m/theta
G = 9.8 # m /s^2

fig, axs = plt.subplots(3,1,sharex = True,figsize=(6, 9))
fig.canvas.manager.window.move(500,0)

################### Motor C ###################

# All points - plotted
x = np.linspace(0,180,361)

for i in range(0,len(axs.reshape(-1))):
    K = K_array[i]
    Mom = moments[i]
    weight_torque_array = []
    spring_torque_array = []
    motor_torque_array = []    
    for k in range(0,len(x)):    
        theta = x[k]
        weight_torque = Mom*G*np.cos(np.deg2rad(theta)) # (m)*(m/s^2)*kg = m*N
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