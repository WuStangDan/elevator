import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
import time
from scipy.integrate import ode

# Simulator Options
FIG_SIZE = [8, 8] # [Width, Height]
PID_DEBUG = False

# Physics Options
GRAVITY = False
FRICTION = True

# Controller Options
CONTROLLER = True
START_LOC = 0.0
SET_POINT = 27.0




class PidController:
    def __init__(self, reference):
        self.integral = 0
        self.prev_error = 0
        self.prev_D_error = 0
        self.prev_D_time = 0
        self.prev_D_out = 0
        self.r = reference
        self.prev_time = 0
        self.finished = 0
        self.max_i = 1000
        self.count = 0

    def Run(self, x, t):
        kp = 0.5
        ki = 0.0
        kd = 0.01

        e = self.r - x

        dt = t - self.prev_time
        self.prev_time = t

        # Set Max I to Prevent Windup
        self.integral += e * (dt)
        if self.integral > self.max_i:
            self.integral = self.max_i
        if self.integral < -self.max_i:
            self.integral = -self.max_i

        P_out = kp * e
        I_out = ki * self.integral
        if t - self.prev_D_time > 0.1:
            D_out = kd * (e - self.prev_error)/(t - self.prev_D_time)
            self.prev_D_out = D_out
            self.prev_D_time = t
        else:
            D_out = self.prev_D_out

        output = P_out + I_out + D_out
        if output > 1.5:
            output = 1.5
        if output < -1.5:
            output = -1.5
        #print(round(P_out,2), round(I_out,2), round(D_out,2), output)
        print(self.count)
        self.count += 1
        return output



Pid = PidController(SET_POINT)


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
    x_dot_dot = 0
    if GRAVITY:
        x_dot_dot += g
    if CONTROLLER:
        x_dot_dot += Pid.Run(x,t)
    if FRICTION:
        x_dot_dot -= x_dot*0.5
    #x_dot_dot = g #+ Pid.Run(x,t) - x_dot*5

    #print(t, x_dot, x_dot_dot)

    #if abs(x_dot) < 0.01 and abs(Pid.r - x) < 0.03:
    #    Pid.finished = 1
    #    return [0, 0]
    # Output state derivatives.
    return [x_dot, x_dot_dot]




# ODE Info.
start = time.clock()
solver = ode(ElevatorPhysics)
solver.set_integrator('dopri5')

# Set initial values.
t0 = 0.0
t_end = 30.2
dt = 0.01


t = np.arange(t0, t_end, dt)


# Solution array and initial states.
sol = np.zeros((int(t_end/dt), 3))
state_initial = [START_LOC, 0.0]
solver.set_initial_value(state_initial, t0)
sol[0] = [state_initial[0], state_initial[1], 0.0]
prev_vel = state_initial[1]

# Repeatedly call the `integrate` method to advance the
# solution to time t[k], and save the solution in sol[k].
k = 1
while solver.successful() and solver.t < t_end-dt:
    solver.integrate(t[k])
    sol[k] = [solver.y[0], solver.y[1], (solver.y[1]-prev_vel)/dt]
    #print(sol[k])
    k += 1
    prev_vel = solver.y[1]



state = sol
print("ODE Compute Time: ", time.clock() - start, "seconds.")

###################
# SIMULATOR DISPLAY


def update_plot(num):
    # Time bar.
    time_bar.set_data([9.8, 9.8], [0, num/100])

    # Elevator.
    el_l.set_data([3, 3],[state[num,0], state[num,0]+3])
    el_r.set_data([6, 6],[state[num,0], state[num,0]+3])
    el_t.set_data([3, 6],[state[num,0]+3, state[num,0]+3])
    el_b.set_data([3, 6],[state[num,0], state[num,0]])

    #print(state[num])

    # Timer.
    time_text.set_text(str(round(num/100,1)))

    # Strip Chart.
    pos.set_data(t[0:num], state[0:num,0])
    vel.set_data(t[0:num], state[0:num,1])
    acc.set_data(t[0:num], state[0:num,2])

    # Status
    if abs(state[num,1]) < 0.01 and abs(SET_POINT-state[num,0]) < 0.03:
        pos_status.set_text('PASS')
    if abs(state[num,1]) > 18 and len(vel_status.get_text()) < 1:
        vel_status.set_text('FAIL')
    if abs(state[num,2]) > 5 and len(acc_status.get_text()) < 1:
        acc_status.set_text('FAIL')

    return time_bar, el_l, el_r, el_t, el_b, time_text, pos, vel, acc, acc_status, vel_status, pos_status


# Total Figure
fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
gs = gridspec.GridSpec(14,8)

# Elevator plot settings.
ax = fig.add_subplot(gs[:, :3])
plt.xlim(0, 10)
plt.ylim(0, 31)
plt.xticks([])
plt.yticks(np.arange(0,31,3))
plt.title('Elevator')

# Time display.
time_text = ax.text(7.5, 0.5, '', fontsize=15)

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
strip_width = 4

# Position
ax = fig.add_subplot(gs[0:4, strip_width:])
pos, = ax.plot([], [], '-b')
pos_status = ax.text(1.0, SET_POINT, '', fontsize=20, color='g')
plt.title('Position')
plt.xticks([0,30])
plt.xlim(0, 30)
if SET_POINT > START_LOC:
    plt.ylim(START_LOC - 10, SET_POINT+10)
else:
    plt.ylim(SET_POINT - 10, START_LOC+10)

# Velocity
ax = fig.add_subplot(gs[5:9, strip_width:])
vel, = ax.plot([], [], '-b')
vel_status = ax.text(1.0, -18.0, '', fontsize=20, color='r')
plt.title('Velocity')
plt.xticks([0,30])
plt.xlim(0, 30)
plt.ylim(-20, 20)

# Acceleration
ax = fig.add_subplot(gs[10:14, strip_width:])
acc, = ax.plot([], [], '-b')
acc_status = ax.text(1.0, -9.0, '', fontsize=20, color='r')
plt.title('Acceleration')
plt.xlabel('Time (s)')
plt.xlim(0, 30)
plt.ylim(-10, 10)

# Animation.
elevator_ani = animation.FuncAnimation(fig, update_plot, frames=range(0,int(30.0*100),10), interval=100, repeat = False, blit=True)
#line_ani.save('lines.mp4')

plt.show()
