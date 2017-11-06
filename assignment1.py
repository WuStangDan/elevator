import numpy as np
from sim.elevator import sim_run

# Simulator Options
options = {}
options['FIG_SIZE'] = [8, 8] # [Width, Height]
options['PID_DEBUG'] = False

# Physics Options
options['GRAVITY'] = False
options['FRICTION'] = False
options['ELEVATOR_MASS'] = 500
options['COUNTERWEIGHT_MASS'] = 0
options['PEOPLE_MASS'] = 0

# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 0.0
options['SET_POINT'] = 12.0
options['OUTPUT_GAIN'] = 500


class Controller:
    def __init__(self, reference):
        self.r = reference
        self.prev_time = 0
        self.output = 0

    def run(self, x, t):
        # Controller run time.
        if t - self.prev_time < 0.05:
            return self.output
        else:
            self.prev_time = t
            # INSERT CODE BELOW.

            self.output = 4
            # INSERT CODE ABOVE.
            return self.output

sim_run(options, Controller)
