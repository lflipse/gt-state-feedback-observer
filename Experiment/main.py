
import numpy as np
# from Experiment.Controllers.Normalized_Gradient import ControllerNG
from Experiment.Controllers.Normalized_Gradient_No_Robot import ControllerNG
from Experiment.experiment import Experiment

def generate_reference(T):

    x_d = 30*np.pi/180

    # Reference signal
    fs1 = 1 / 8
    fs2 = 1 / 4
    fs3 = 1 / 3
    fs4 = 1 / 10

    r = x_d * (np.sin(2 * np.pi * fs1 * T) + np.sin(2 * np.pi * fs2 * T) + np.sin(2 * np.pi * fs3 * T) + np.sin(2 * np.pi * fs4 * T))

    return r

# Simulation parameters
t = 40
h = 0.005
N = round(t/h) + 1
T = np.array(range(N)) * h
r = generate_reference(T)

# Dynamics
Jw = 0.22
Bw = -0.2
Kw = 0.0
A = np.array([[0, 1], [- Bw / Jw, -Kw / Jw]])
B = np.array([[0], [1 / Jw]])

# TODO: verify values
mu = 0.0
sigma = 0.0
alpha = 400
Gamma = alpha * np.array([[5, 0], [0, 1]])
kappa = 2

screen_width = 800
screen_height = 600
robot_controller = ControllerNG(A, B, mu, sigma, kappa, Gamma)
human_controller = ControllerNG(A, B, mu, sigma, kappa, Gamma)
experiment_handler = Experiment(Bw, Kw, screen_width, screen_height, robot_controller, human_controller)
experiment_handler.do_experiment(T, r, N, h)
