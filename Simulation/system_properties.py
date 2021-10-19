import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import scipy.linalg as cp

sys.path.insert(1, '..')

from Simulation.Linear_Quadratic import ControllerLQ
from Experiment.reference_trajectory import Reference


# Functions
Jw = 0.04914830792783059
Bw = 0.3  # Max = 0.5
Kw = 0.0  # Max = 2.5
A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
B = np.array([[0], [1 / Jw]])
reference = Reference(duration=None)




# Simulation parameters
t = 67.5
h = 0.01
N = round(t/h) + 1
T = np.array(range(N)) * h
r = np.zeros((N, 2))

for i in range(N):
    r[i, :] = reference.generate_reference(T[i], 0, "robot", 0)

controller = ControllerLQ(A, B, 0, 0, True)

# Initial values
pos0 = 0
vel0 = 0
initial_state = np.array([pos0, vel0])
initial_error = np.array([pos0, vel0])
initial_input = 0

# Simulated Human Settings
# True cost values
Qh = np.array([[20, 0], [0, 0.5]])
Qr = np.array([[0, 0], [0, 0]])
C = Qh
vhg = None
controller_name = None
dynamics_name = None
dynamics = None
save = None
bias = np.array([0, 0])
ref = r


# Compute inputs for simulation
input_dict = {
    "simulation_steps": N,
    "step_size": h,
    "time_vector": T,
    "reference_signal": r,
    "human_weight": Qh,
    "robot_weight": Qr,
    "sharing_rule": C,
    "controller_type_name": controller_name,
    "dynamics_type_name": dynamics_name,
    "controller_type": controller,
    "dynamics_type": dynamics,
    "save": save,
    "gain_estimation_bias": bias,
    "initial_state": initial_state,
    "u_initial": initial_input,
    "e_initial": initial_error,
    "reference": ref,
    "time": T,
}

rmse = []
rmsu = []
L = []

for i in range(10):
    Qh = np.array([[2**i, 0], [0, 0.1]])
    input_dict["human_weight"] = Qh
    output = controller.simulate(inputs=input_dict)
    e = output["error_states"][:, 0]
    u = output["human_input"]
    L.append(output["human_gain"][0, 0])
    rmse.append(1/N * np.sqrt(np.inner(e, e)))
    rmsu.append(1/N * np.sqrt(np.inner(u, u)))

print("rmse = ", rmse, "rmsu = ", rmsu, "L = ", L)

plt.figure()
plt.plot(L, rmse)

plt.figure()
plt.plot(L, rmsu)

plt.show()
