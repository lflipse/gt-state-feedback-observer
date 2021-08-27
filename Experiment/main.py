import numpy as np
import multiprocessing as mp
import time
import wres
import platform
import scipy.linalg as cp

from Experiment.reference_trajectory import Reference
from Experiment.live_plotter import LivePlotter
from Experiment.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from Experiment.experiment import Experiment
from Experiment.Controllers.Linear_Quadratic import ControllerLQ
from Experiment.Controllers.Differential_Game import ControllerDG
from Experiment.Controllers.Differential_Game_Gain_Descent import ControllerDG_GObs
from Experiment.Controllers.Differential_Game_Gain_Observer import ControllerDG_GObsKal
from Experiment.Controllers.Li2019 import ControllerDG_Li
from Experiment.plots import PlotStuff
from Simulation.Linear_Quadratic import ControllerLQ as LQsim
# from Simulation.Differential_Game import ControllerDG as DGsim
from Simulation.Differential_Game_Gain_Descent import ControllerNG as NGsim
from Simulation.Differential_Game_Gain_Observer import ControllerNG as NG_Obssim
from Experiment.human_gains import HumanEstimator

def input_controller():
    # Select controller type
    print("Choose a controller type. 0: Manual control, 1: LQ control, 2: Differential Game, 3: Differential Game w/ obs",
          "4: Differential Game w/ obs Kalman, 5: Differential Game w/ obs Li2019")
    exp_type = input("Please choose: ")
    if int(exp_type) == 0:
        controller = None
        controller_type = "Manual"
    elif int(exp_type) == 1:
        controller = ControllerLQ(A, B, Qr_start)
        controller_type = "LQ"
    elif int(exp_type) == 2:
        controller = ControllerDG(A, B, Qr_start, Qh)
        controller_type = "DG"
    elif int(exp_type) == 3:
        controller = ControllerDG_GObs(A, B, Gamma, Pi, kappa, Qr_start, Qh)
        controller_type = "Gain_observer"
    elif int(exp_type) == 4:
        controller = ControllerDG_GObsKal(A, B, Gamma, Pi, kappa, Qr_start, Qh)
        controller_type = "Gain_observer"
    elif int(exp_type) == 5:
        controller = ControllerDG_Li(A, B, Gamma, Pi, kappa, Qr_start, Qh)
        controller_type = "Gain_observer"
    else:
        print("You chose: ", int(exp_type))
        exit("That's no option, try again.")

    print("Validate using simulation data? No = 0, Yes = 1")
    sim = input("Please choose: ")
    return controller, controller_type, sim, exp_type

def run_simulation(experiment_data, exp_type, human_gain):
    # Use the correct settings for the simulation
    print("expected: ", N)
    N_exp = len(experiment_data["steering_angle"])
    print("got: ", N_exp)
    inputs = {
        # "simulation_steps": N_exp,
        # "step_size": t_step,
        "time": experiment_data["time"],
        "reference_signal": [],
        "human_weight": Qh,
        "robot_weight": Qr_start,
        "alpha": alpha,
        "Gamma": Gamma,
        "kappa": kappa,
        "e_initial": [experiment_data["angle_error"][0], experiment_data["rate_error"][0]],
        "u_initial": experiment_data["torque"][0],
        "reference": experiment_data["reference"],
        "initial_state": [experiment_data["steering_angle"][0], experiment_data["steering_rate"][0]],
        "sharing_rule": C,
        "controller_type_name": controller_type,
        "dynamics_type_name": "Steering_dynamics",
        "controller_type": controller_type,
        "dynamics_type": "Steering_dynamics",
        "save": 0,
        "gain_estimation_bias": 0.0,
        "virtual_human_gain": human_gain,
    }

    if int(exp_type) == 0:
        controller_sim = LQsim(A, B, mu=0.0, sigma=0.0, nonlin=True)
        inputs["robot_weight"] = np.array([[0, 0], [0, 0]])
    elif int(exp_type) == 1:
        controller_sim = LQsim(A, B, mu=0.0, sigma=0.0, nonlin=True)
    elif int(exp_type) == 2:
        controller_sim = DGsim(A, B, mu=0.0, sigma=0.0, nonlin=True)
    elif int(exp_type) == 3:
        controller_sim = NGsim(A, B, mu=0.0, sigma=0.0, nonlin=True)
    elif int(exp_type) == 4 or 5:
        controller_sim = NG_Obssim(A, B, mu=0.0, sigma=0.0, nonlin=True)

    simulation_data = controller_sim.simulate(inputs)
    return simulation_data

# This statement is necessary to allow for multiprocessing
if __name__ == "__main__":
    if platform.system() == 'Windows':
        import wres

    # Simulation parameters
    t_warmup = 5
    t_cooldown = 5
    t_exp = 90
    duration = t_warmup + t_cooldown + t_exp
    t_step = 0.015
    N = round(t_exp / t_step)
    N_warmup = round(t_warmup / t_step)
    N_cooldown = round(t_cooldown / t_step)
    N_tot = N + N_warmup + N_cooldown
    T = np.array(range(N)) * (t_step + 0.001)
    T_ex = np.array(range(N+N_warmup+N_cooldown)) * t_step
    fr_min = 1/(20 * np.pi)
    fr_max = 1/(1 * np.pi)
    increments = 20
    reference = Reference(duration)

    # Dynamics
    Jw = 0.05480475491037145
    Bw = 0.5  # Max = 0.5
    Kw = 0.0  # Max = 2.5
    A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
    B = np.array([[0], [1 / Jw]])

    # TODO: verify values
    alpha = 30
    Gamma = alpha * np.array([[2.5, 0], [0, 0.125]])
    # Pi = -0.1*np.array([[-1, 0.5], [-1.5, 2]])
    Pi = 4 * np.array([[2, 0], [0, 2]])
    kappa = 0.7
    Qr_end = np.array([[10.0, 0.0], [0.0, 0.1]])
    Qr_start = np.array([[10.0, 0.0], [0.0, 0.1]])

    C = Qr_start

    Qh = np.array([[10.0, 0.0], [0.0, 0.0]])
    virtual_gain = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qh, 1))
    vhg = np.zeros((6, 2))
    vhg[0, :] = np.array([virtual_gain[0, 0], virtual_gain[0, 1]])
    vhg[1, :] = np.array([0, 0])
    vhg[4, :] = np.array([0.5 * -virtual_gain[0, 0], 0.5 * -virtual_gain[0, 1]])
    vhg[3, :] = np.array([0, 0])
    vhg[2, :] = np.array([1.5 * virtual_gain[0, 0], 1.5 * virtual_gain[0, 1]])
    vhg[5, :] = np.array([0, 0])
    virtual_human_gain = vhg

    virt = input("Do you want to use a virtual human being? 0. No, 1. Yes. Your answer = ")
    if int(virt) == 0:
        virtual_human = False
    else:
        virtual_human = True


    print("Observer dynamics", A - Pi)

    screen_width = 1920
    screen_height = 1080
    # screen_width = 720
    # screen_height = 480

    # ask input
    controller, controller_type, sim, exp_type = input_controller()

    # Start the senso drive parallel process
    parent_conn, child_conn = mp.Pipe(True)
    senso_drive_process = SensoDriveModule(Bw, Kw, controller, controller_type, parent_conn, child_conn)
    senso_drive_process.start()
    print("process started!")

    # Start the data plotting parallel process
    send_conn, receive_conn = mp.Pipe(True)
    live_plotter_process = LivePlotter(receive_conn)
    live_plotter_process.start()
    print("Second process started!")

    # Time to do an experiment!
    full_screen = True
    preview = True
    do_exp = True

    if controller_type == "Manual" and int(sim) == 1:
        estimate_gains = input("Do you want to estimate the human gains? 0: No, 1: Yes.  Your anwser = ")
        if int(estimate_gains) == 1:
            # human_estimator =
            generate_data_set = input("Do you want to generate a new dataset? 0: No, 1: Yes Your answer = ")
            if int(generate_data_set) == 0:
                # experiment_data = load_data_set_manual()
                do_exp = False

    if do_exp == True:
        experiment_input = {
            "damping": Bw,
            "stiffness": Kw,
            "reference": reference,
            "controller_type": controller_type,
            "send_conn": send_conn,
            "parent_conn": parent_conn,
            "child_conn": child_conn,
            "senso_drive": senso_drive_process,
            "screen_width": screen_width,
            "screen_height": screen_height,
            "full_screen": full_screen,
            "preview": preview,
            "warm_up_time": t_warmup,
            "experiment_time": t_exp,
            "cooldown_time": t_cooldown,
            "virtual_human": virtual_human,
            "virtual_human_gain": virtual_human_gain,
            "init_robot_cost": Qr_start,
            "final_robot_cost": Qr_end,
        }
        experiment_handler = Experiment(experiment_input)
        if platform.system() == 'Windows':
            with wres.set_resolution(10000):
                experiment_data = experiment_handler.experiment()
        senso_drive_process.join(timeout=0)
        live_plotter_process.join(timeout=0)


    # Plot stuff
    plot_stuff = PlotStuff()

    if int(sim) == 1:
        # Estimate human gains
        if controller_type == "Manual":
            human_gain_estimator = HumanEstimator(Jw, Bw, Kw)
            estimated_human_gains = human_gain_estimator.estimate(experiment_data)
            simulation_data = run_simulation(experiment_data, exp_type, human_gain=estimated_human_gains)
        else:
            virtual_human_gains = np.array([experiment_data["virtual_human_gain_pos"],
                                            experiment_data["virtual_human_gain_vel"]])
            simulation_data = run_simulation(experiment_data, exp_type, human_gain=virtual_human_gains)
        # plot_stuff.plot_stuff_with_sim_data(experiment_data, simulation_data, controller_type, virtual_human)
        plot_stuff.plot_report(experiment_data, simulation_data)
    else:
        simulation_data = None
        plot_stuff.plot_report(experiment_data, simulation_data)
        # plot_stuff.plot_stuff(experiment_data, controller_type, virtual_human)

