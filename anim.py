import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

from scipy.integrate import ode

class PidController:
    def __init__(self, reference):
        self.integral = 0
        self.prev_error = 0
        self.r = reference
        self.prev_time = 0
        self.finished = 0
        self.max_i = 10000

    def Run(self, x, t):
        kp = 15
        ki = 0.001
    
        e = self.r - x

        self.integral += e * (t - self.prev_time)

        if self.integral > self.max_i:
            self.integral = self.max_i
        if self.integral < -self.max_i:
            self.integral = -self.max_i

        output = kp*e + ki*self.integral
        return output





Pid = PidController(27)


# ODE Solver 
def ElevatorPhysics(t, state):
    if Pid.finished == 1:
        return [0,0]

    # State vector.
    x = state[0]
    x_dot = state[1]
    

    # Mass of elevator in kg.
    m = 100.0
    # Acceleration of gravity.
    g = -9.8
    
    x_dot_dot = g + Pid.Run(x,t) - x_dot*5

    print(t, x_dot, x_dot_dot)

    if abs(x_dot) < 0.01 and abs(Pid.r - x) < 0.03:
        Pid.finished = 1
        return [0, 0]
    # Output state derivatives.
    return [x_dot, x_dot_dot]


# ODE Info.
solver = ode(ElevatorPhysics)
solver.set_integrator('dopri5')

# Set initial values.
t0 = 0.0
t_end = 30.2
dt = 0.01


t = np.arange(t0, t_end, dt)


# Solution array and initial states.
sol = np.zeros((int(t_end/dt), 2))
state_initial = [30.0, 0.0]
solver.set_initial_value(state_initial, t0)
sol[0] = state_initial

# Repeatedly call the `integrate` method to advance the
# solution to time t[k], and save the solution in sol[k].
k = 1
while solver.successful() and solver.t < t_end-dt:
    solver.integrate(t[k])
    sol[k] = solver.y
    k += 1


state = sol


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
