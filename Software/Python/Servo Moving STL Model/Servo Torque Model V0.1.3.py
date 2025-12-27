import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator

# Digit Lengths
A = 33.55 # mm, bottom digit + motor A height
B = 43.55 # mm, lower-middle digit + motor B height
C = 43.55 # mm, upper-middle middle 2 digit + motor B height
T = 43.55    # mm, top digit height

#minmax angles, from the STL model


MINMAX_A = [66.7754,113.2585]
MINMAX_B = [33.7293,53.9247]
MINMAX_C = [102.9919,106.3376]

MINMAXES = [MINMAX_C,MINMAX_B,MINMAX_A]

titles = ['i.) Top Motor (C)', 'ii.) Middle Motor (B)', 'iii.) Bottom Motor (A)']

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
W_plate_lb =  2 #lb, only 4 fingers will hold at a time
W_plate =  W_plate_lb  /  2.20462 # kg, only 4 fingers will hold at a time
W_plate = W_plate/4 # kg, only 4 fingers will hold at a time


# Overall
# The torque (stalled) is 2.3kg.cm@6V
# The torque (moving)  is 0.7kg.cm@6V
motor_torque = 1.3 #Kg•cm

kgcm2mNm = 98.0665 # N•m / Kg•cm
motor_torque = motor_torque * kgcm2mNm / 1000 #N·m, Motor Strength
K_constant = 1.4  # in•lbf, Spring Strength
K_array = np.ones(3)*K_constant

inlbf2Nm = 1 / 8.85075 # in•lbf / N•m
theta_deflection = 270
K_array = K_array*inlbf2Nm / theta_deflection  # N•m/theta
G = 9.8 # m /                s^2

fig, axs = plt.subplots(3,1,sharex = True,sharey = True,figsize=(3, 6),constrained_layout=True)
fig.canvas.manager.window.move(500,0)
fig.canvas.manager.window.move(150,0)

 
# All points - plotted
x = np.linspace(0,180,361)

for i in range(0,len(axs.reshape(-1))):

    K = K_array[i]
    weight_torque_array = []
    spring_torque_array = []
    motor_torque_array = []    
    for k in range(0,len(x)):
        theta = x[k]
        if theta <  MINMAXES[i][0] or theta > MINMAXES[i][1]:
            Holder_Motor_C_Weight2Move = [0, 0, Holder_Motor_T_Weight]
            Holder_Motor_B_Weight2Move = [0, Holder_Motor_C_Weight + weight_motor + weight_bearing, Holder_Motor_T_Weight]
            Holder_Motor_A_Weight2Move = [Holder_Motor_B_Weight + weight_motor + weight_bearing, Holder_Motor_C_Weight + weight_motor + weight_bearing, Holder_Motor_T_Weight]
        else:
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
        Mom = moments[i]
        
        
        
        
        weight_torque = Mom*G*np.cos(np.deg2rad(theta)) # (m)*(m/s^2)*kg = m*N
        spring_torque = K*theta
        weight_torque_array.append(weight_torque)
        spring_torque_array.append(spring_torque)
        motor_torque_array.append(motor_torque)    

    motor_torque_array = np.array(motor_torque_array)*1000
    weight_torque_array = np.array(weight_torque_array)*1000
    spring_torque_array = np.array(spring_torque_array)*1000
    
    axs[i].plot(x,motor_torque_array,label = 'Motor', c = 'k')
    axs[i].plot(x,-motor_torque_array, c = 'k')
    axs[i].plot(x,weight_torque_array,label = 'Weight',c='b',alpha = 0.3,linestyle='-')
    axs[i].plot(x,spring_torque_array,label = 'Spring',c='r',alpha = 0.3,linestyle='-')
    axs[i].plot(x,x*0,'k',alpha = 0.5,linestyle='dotted')
    axs[i].plot(x,weight_torque_array + spring_torque_array,label = 'Total',c='m')


    rect = patches.Rectangle(
        (MINMAXES[i][0],0),
        MINMAXES[i][1]-MINMAXES[i][0],
        motor_torque*1000,
        linewidth=2,
        edgecolor='tab:orange',
        facecolor='tab:orange', # Blue fill color
        alpha=0.5 # 50% transparency
    )
    axs[i].add_patch(rect)


    axs[i].text(0.025, 1-0.075, titles[i],
            horizontalalignment='left',
            verticalalignment='center',
            transform=axs[i].transAxes,
            fontsize = 9)
    
    
    axs[i].tick_params(axis='both',
                   which='both',
                   direction='in',)

    axs[i].set_xticks(np.round(np.arange(-0,180,45),2))
    axs[i].xaxis.set_minor_locator(AutoMinorLocator(3))
    axs[i].set_yticks(np.round(np.arange(-100,101,100),2))        
    axs[i].yaxis.set_minor_locator(AutoMinorLocator(2))
    
axs[0].set_ylabel('Torque / mN•m')
axs[1].set_ylabel('Torque / mN•m')
axs[2].set_ylabel('Torque / mN•m')

axs[2].set_xlabel(r'Theta / $\theta$')
axs[2].set_xlim([0,180])
axs[2].set_ylim([-200,200])
axs[2].tick_params(axis='both',
               which='both',
               direction='in',)

axs[0].annotate(
    f'{W_plate_lb}lb Load',
    xy=((MINMAXES[0][0] + MINMAXES[0][1]) / 2+2, motor_torque * 1000 * 6 / 8 ),
    xytext=(0.2, 0.7),
    textcoords='axes fraction',
    arrowprops=dict(
        arrowstyle='-',
        color='black',      # arrow edge color
        lw=1.5,
        mutation_scale = 20
    ),
    fontsize=9
)





axs[0].legend(loc = 4,ncol = 1, columnspacing=0.5,handletextpad=0.2, fontsize = 8.5, handlelength = 1,facecolor = 'w',framealpha = 1,fancybox = False)

plt.show()
