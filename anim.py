import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

from scipy.integrate import odeint

def update_plot(num, data, state, line, line2, line3):
    line.set_data(data[:,:int(num/10)])
    line2.set_data(20,state[num,0])

    time_text.set_text(str(num/100))

    line3.set_data(t[0:num], state[0:num,0])

    return line, line2, time_text, line3



def PidController(x):
    if x < 20:
        return 30
    else:
        return 0


def ElevatorPhysics(state, t):
    # State vector.
    x = state[0]
    x_dot = state[1]

    # Mass of elevator in kg.
    m = 100.0
    # Acceleration of gravity.
    g = -9.8

    x_dot_dot = g + PidController(x)

    # Output state derivatives.
    return [x_dot, x_dot_dot]


state_initial = [50.0, 0.0]
t = np.arange(0.0, 110.0, 0.01)

state = odeint(ElevatorPhysics, state_initial, t)

data = np.array([np.ones(51)*39, np.arange(51)])




fig1 = plt.figure(figsize=(4,8))
gs = gridspec.GridSpec(17,1)

# Elevator plot.
ax = fig1.add_subplot(gs[:13, :])
time_text = ax.text(.5, .5, '', fontsize=15)
l, l2, = ax.plot([], [], 'r-', [], [], 'ks', markersize = 50, markerfacecolor='None', markeredgewidth=3)

plt.xlim(0, 40)
plt.ylim(0, 50)
#plt.xlabel('x')
plt.title('Elevator')


# Strip chart.
ax = fig1.add_subplot(gs[14:19,:])
l3, = ax.plot([], [], '-b')


plt.xlim(0, 10)
plt.ylim(-50, 50)




elevator_ani = animation.FuncAnimation(fig1, update_plot, int(10.02*100), fargs=(data, state, l, l2, l3),
    interval=10, repeat = False, blit=True)
#line_ani.save('lines.mp4')

plt.show()
