import PIDControl
import numpy as np

# specifications, ep=0 assumed
te = 2.65
delta = 0.1
ep = 0

# Process
K=7
process_poles = np.array([0, -12])
process_zeros = np.array([])


# make controlled process object
task1 = PIDControl.PIDControl(K, process_zeros, process_poles, te, delta, ep)
print(task1.s)

# specify own s, otherwise s will be at limit of zone of specifications as calculated before
# s = -4
# task1.s = s

#argument and modulo criterion to figure out paramters of PID
# for PID you need to add a zero choosen by you, which ideally cancels a pole:
# task1.add_zero_to_controller(-3)
task1.add_zero_by_criterium()

task1.set_kp_by_criterium()

print(task1.process)
print(task1.pid)

print(task1.closed_loop)

task1.draw_root_locus()