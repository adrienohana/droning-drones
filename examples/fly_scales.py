"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python fly.py

Notes
-----



"""

import os
print(os.getcwd())
HOVER_THRUST = 38726.7
MIN_THRUST = 20000
MAX_THRUST = 65535

import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.PIDControl import PIDControl 
from gym_pybullet_drones.utils.Logger_plots import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool


if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default="cf2x",     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    #parser.add_argument('--num_drones',         default=3,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--num_drones',         default=1,          type=int,           help='Number of drones (default: 1)', metavar='')
    parser.add_argument('--physics',            default="pyb",      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=False,      type=str2bool,      help='Whether to use VisionAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=True,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=False,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=True,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=False,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=True,       type=str2bool,      help='Whether to aggregate physics steps (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=False,      type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=500,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=500,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=10,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    ARGS = parser.parse_args()


    #### Scale Definitions #############################
    major_up_notes = [38726.7, 48195.19271581488, 60055.49312577782]
    major_down_notes = [38725, 34731.37329743071, 27969.038385567823, 22560.592561926514]
    minor_up_notes = [38726.7, 48195.19271581488, 53791.33480208646]
    minor_down_notes = [38726.7, 31162.583174734853, 27970.253268488214, 22561.564635655854]

    uniform_up_notes = np.linspace(HOVER_THRUST, MAX_THRUST, 10)
    uniform_down_notes = np.linspace(HOVER_THRUST, MIN_THRUST, 10)

    #NOTE LENGTH !!
    note_length = 0.15

    major_up_scale = [(note, note_length) for note in major_up_notes]
    major_down_scale = [(note, note_length) for note in major_down_notes]
    uniform_up_scale = [(note, note_length) for note in uniform_up_notes]
    uniform_down_scale = [(note, note_length) for note in uniform_down_notes]

    #PICK SCALE AND STARTING HEIGHT
    scale = [(40000,2)]
    #scale = uniform_down_scale
    starting_height = 0.3




    #### Initialize the simulation #############################
    INIT_XYZS = np.array([[0, 0, starting_height] for i in range(ARGS.num_drones)])
    INIT_RPYS = np.array([[0, 0,  0] for i in range(ARGS.num_drones)])
    AGGR_PHY_STEPS = int(ARGS.simulation_freq_hz/ARGS.control_freq_hz) if ARGS.aggregate else 1





    #### Create the environment with or without video capture ##
    if ARGS.vision: 
        env = VisionAviary(drone_model=ARGS.drone,
                           num_drones=ARGS.num_drones,
                           initial_xyzs=INIT_XYZS,
                           initial_rpys=INIT_RPYS,
                           physics=ARGS.physics,
                           neighbourhood_radius=10,
                           freq=ARGS.simulation_freq_hz,
                           aggregate_phy_steps=AGGR_PHY_STEPS,
                           gui=ARGS.gui,
                           record=ARGS.record_video,
                           obstacles=ARGS.obstacles
                           )
    else: 
        env = CtrlAviary(drone_model=ARGS.drone,
                         num_drones=ARGS.num_drones,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=ARGS.physics,
                         neighbourhood_radius=10,
                         freq=ARGS.simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=ARGS.gui,
                         record=ARGS.record_video,
                         obstacles=ARGS.obstacles,
                         user_debug_gui=ARGS.user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
                    num_drones=ARGS.num_drones
                    )

    #### Initialize the controllers ############################
    ctrl = PIDControl(drone_model=ARGS.drone)



    #### Run the simulation ####################################

    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/ARGS.control_freq_hz))
    action = {str(i): np.array([0,0,0,0]) for i in range(ARGS.num_drones)}
    START = time.time()
    k=0
    l=0

    #start with position control
    POSITION_CONTROL = 0
    ATTITUDE_CONTROL = 1
    ctrl.controller_type = POSITION_CONTROL
    target_rpy_rates = np.array([0, 0, 0])
    note_start = time.time()
    melody_start = time.time()
    for i in range(0, int(ARGS.duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):
        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)
        #### Compute control at the desired frequency ##############
        if i%CTRL_EVERY_N_STEPS == 0:
            #HOVER 6 seconds BEFORE STARTING
            if ctrl.controller_type == POSITION_CONTROL:
                action[str(0)], _, _ = ctrl.computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                    state=obs[str(0)]["state"],
                                                                    target_pos = np.array([0,0,starting_height]),
                                                                    target_rpy_rates=target_rpy_rates
                                                                    )

                if time.time()-note_start >= 6:
                    ctrl.controller_type = ATTITUDE_CONTROL
                    note_start=time.time()
                
            else : 
                #if all notes have been played, hover
                if l >= len(scale):
                    break
                    ctrl.target_thrust = HOVER_THRUST
                
                else:
                    ctrl.target_thrust = scale[l][0]
                    if time.time()-note_start >=scale[l][1]:
                        l+=1
                        note_start=time.time()
                #### Compute control for the current way point ############   
                action[str(0)], _, _ = ctrl.computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                        state=obs[str(0)]["state"],
                                                                        target_pos = np.array([0,0,0]),
                                                                        target_rpy_rates=target_rpy_rates
                                                                        )
        k+=1

        # #### Log the simulation ####################################
        logger.log(drone=0,
                timestamp=i/env.SIM_FREQ,
                state= obs[str(0)]["state"]
                #control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                #control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
                )

        #### Printout ##############################################
        if i%env.SIM_FREQ == 0:
            env.render()
        #### Sync the simulation ###################################
        if ARGS.gui:
            sync(i, START, env.TIMESTEP)

    #### Close the environment #################################
    env.close()

    logger.save()
 #   logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    if ARGS.plot:
        logger.plot()
