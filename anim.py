import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

from scipy.integrate import odeint

def PidController(x):
    r = 27
    kp = 10
    ki = 1
    
    e = r - x
    
       
    
    return kp*e


def ElevatorPhysics(state, t):
    # State vector.
    x = state[0]
    x_dot = state[1]

    # Mass of elevator in kg.
    m = 100.0
    # Acceleration of gravity.
    g = -9.8

    x_dot_dot = g + PidController(x) - x_dot*0.4

    # Output state derivatives.
    return [x_dot, x_dot_dot]

# ODE Info.
state_initial = [30.0, 0.0]
t = np.arange(0.0, 110.0, 0.01)

state = odeint(ElevatorPhysics, state_initial, t)







###################
# SIMULATOR DISPLAY


def update_plot(num):
    # Time bar.
    time_bar.set_data(data[:,:int(num)]/100)

    # Elevator.
    el_l.set_data([3, 3],[state[num,0], state[num,0]+3])
    el_r.set_data([6, 6],[state[num,0], state[num,0]+3])
    el_t.set_data([3, 6],[state[num,0]+3, state[num,0]+3])
    el_b.set_data([3, 6],[state[num,0], state[num,0]])
    

    # Timer.
    time_text.set_text(str(num/100))

    # Strip Chart.
    l3.set_data(t[0:num], state[0:num,0])

    return time_bar, el_l, el_r, el_t, el_b, time_text, l3,


data = np.array([np.ones(1100)*980, np.arange(1100)])



# Total Figure
fig = plt.figure(figsize=(4,8))
gs = gridspec.GridSpec(17,1)

# Elevator plot settings.
ax = fig.add_subplot(gs[:13, :])
plt.xlim(0, 10)
plt.ylim(0, 31)
plt.xticks([])
plt.yticks(np.arange(0,31,3))
plt.title('Elevator')

# Time display.
time_text = ax.text(8.0, .5, '', fontsize=15)

# Floor Labels.
floors = ['G', '2', '10']
floor_height = [0.5, 3.5, 27.5]
floor_x = [0.25, 0.25, 0.25]
for i in range(len(floors)):
    ax.text(floor_x[i], floor_height[i], floors[i])
    ax.plot([0, 3], [floor_height[i]-0.5, floor_height[i]-0.5], 'k-')


# Plot info.
time_bar, = ax.plot([], [], 'r-')
el_l, el_r = ax.plot([], [], 'k-', [], [], 'k-')
el_t, el_b = ax.plot([], [], 'k-', [], [], 'k-')




# Strip chart settings.
ax = fig.add_subplot(gs[14:19,:])
l3, = ax.plot([], [], '-b')
plt.xlabel('Time (s)')
plt.xlim(0, 30)
plt.ylim(0, 31)



# Animation.
elevator_ani = animation.FuncAnimation(fig, update_plot, frames=int(30.0*100), interval=10, repeat = False, blit=True)
#line_ani.save('lines.mp4')

plt.show()
