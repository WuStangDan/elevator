import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
import time
from scipy.integrate import ode


def sim_run(options, PidController):
    start = time.clock()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    PID_DEBUG = options['PID_DEBUG']

    # Physics Options
    GRAVITY = options['GRAVITY']
    FRICTION = options['FRICTION']
    E_MASS = options['ELEVATOR_MASS']
    CW_MASS = options['COUNTERWEIGHT_MASS']
    P_MASS = options['PEOPLE_MASS']

    # Controller Options
    CONTROLLER = options['CONTROLLER']
    START_LOC = options['START_LOC']
    SET_POINT = options['SET_POINT']
    OUTPUT_GAIN = options['OUTPUT_GAIN']

    Pid = PidController(SET_POINT)


    # ODE Solver
    def elevator_physics(t, state):
        # State vector.
        x = state[0]
        x_dot = state[1]

        # Acceleration of gravity.
        g = -9.8
        x_dot_dot = 0

        if CONTROLLER:
            x_dot_dot += Pid.run(x,t) * OUTPUT_GAIN / (E_MASS + CW_MASS + P_MASS)
        if GRAVITY:
            x_dot_dot += g*(E_MASS + P_MASS - CW_MASS) / (E_MASS + P_MASS + CW_MASS)
        if FRICTION:
            x_dot_dot -= x_dot * 0.2

        #print(t, x_dot, x_dot_dot)
        # Output state derivatives.
        return [x_dot, x_dot_dot]




    # ODE Info.
    solver = ode(elevator_physics)
    solver.set_integrator('dopri5')

    # Set initial values.
    t0 = 0.0
    t_end = 30.2
    dt = 0.05
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


    ###################
    # SIMULATOR DISPLAY


    def update_plot(num):
        #print(state[num])

        # Time bar.
        time_bar.set_data([7.8, 7.8], [0, num/20])

        # Elevator.
        el_l.set_data([3, 3],[state[num,0], state[num,0]+3])
        el_r.set_data([6, 6],[state[num,0], state[num,0]+3])
        el_t.set_data([3, 6],[state[num,0]+3, state[num,0]+3])
        el_b.set_data([3, 6],[state[num,0], state[num,0]])

        # Timer.
        time_text.set_text(str(round(num/20+0.04,1)))

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

        # Debug time line.
        if PID_DEBUG:
            p_line.set_data([num/20, num/20], [-1000, 1000])
            i_line.set_data([num/20, num/20], [-1000, 1000])
            d_line.set_data([num/20, num/20], [-1000, 1000])
            return time_bar, el_l, el_r, el_t, el_b, time_text, \
                   pos, vel, acc, acc_status, vel_status, pos_status, \
                   p_line, i_line, d_line
        else:
            return time_bar, el_l, el_r, el_t, el_b, time_text, \
                   pos, vel, acc, acc_status, vel_status, pos_status



    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(14,8)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:, :3])
    plt.xlim(0, 8)
    plt.ylim(0, 31)
    plt.xticks([])
    plt.yticks(np.arange(0,31,3))
    plt.title('Elevator')

    # Time display.
    time_text = ax.text(6, 0.5, '', fontsize=15)

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
    if PID_DEBUG:
        strip_width = 7

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
    acc_status = ax.text(1.0, -4.0, '', fontsize=20, color='r')
    plt.title('Acceleration')
    plt.xlabel('Time (s)')
    plt.xlim(0, 30)
    plt.ylim(-5, 5)

    if PID_DEBUG:
        debug_width = 4
        data = Pid.output_data
        #print(len(data[:,0]))
        # P
        ax = fig.add_subplot(gs[0:4, debug_width:strip_width])
        p_plot, = ax.plot(data[:,0], data[:,1], '-k')
        p_line, = ax.plot([], [], '-r')
        plt.title('P Output Acceleration')
        plt.xticks([0,30])
        plt.xlim(0, 30)
        #plt.ylim(-10, 10)

        # I
        ax = fig.add_subplot(gs[5:9, debug_width:strip_width])
        i_plot, = ax.plot(data[:,0], data[:,2], '-k')
        i_line, = ax.plot([], [], '-r')
        plt.title('I Output Acceleration')
        plt.xticks([0,30])
        plt.xlim(0, 30)
        #plt.ylim(-10, 10)

        # D
        ax = fig.add_subplot(gs[10:14, debug_width:strip_width])
        d_plot, = ax.plot(data[:,0], data[:,3], '-k')
        d_line, = ax.plot([], [], '-r')
        plt.title('D Output Acceleration')
        plt.xlabel('Time (s)')
        plt.xlim(0, 30)

    print("Compute Time: ", round(time.clock() - start, 3), "seconds.")
    # Animation.
    elevator_ani = animation.FuncAnimation(fig, update_plot, frames=range(0,int(30.0*20)), interval=50, repeat = False, blit=True)
    #line_ani.save('lines.mp4')

    plt.show()
