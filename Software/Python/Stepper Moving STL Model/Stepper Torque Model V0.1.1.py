import numpy as np
import matplotlib.pyplot as plt


# Digit Lengths
A = 57.38 # bottom digit + motor A height
B = 60 # lower-middle digit + motor B height
C = 60 # upper-middle middle 2 digit + motor B height
T = 75.66 # top digit height mm
titles = ['Motor A Torque', 'Motor B Torque', 'Motor C Torque']

# individual sections
weight_bearing = 13 / 1000 # kg
weight_motor =   40 / 1000 # kg


lengths = np.array([B,C,T])/1000
weights = [38/1000 + weight_bearing + weight_motor + 44/1000 + weight_bearing + weight_motor+ 27/1000,44/1000 + weight_bearing + weight_motor+ 27/1000,27/1000]

# Overall
################## Need to do some testing.

# This is stock (unipolar): 
motor_torque = 34.3/1000 #N·m

# This is stock (bipolar):
# https://www.youtube.com/watch?v=lLF9_rc9G3I&t=16s
motor_torque = 100/1000 #N·m


W_plate = 0  /  2.20462 # kg
theta_deflection = 270
inlbf2Nm = 1 / 8.85075 # in•lbf / N•m

K_array = np.array([1,0,0.465])
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
    M = weights[i]
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