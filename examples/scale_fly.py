"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python fly.py

Notes
-----
Modified by Adrien O'Hana 
This example plays a chosen scale from our library


"""
import note_lists

import time
import argparse
import numpy as np

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.PIDControl import PIDControl 
from gym_pybullet_drones.utils.Logger_plots import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool


########################  PARAMETERS ########################
note_length = 0.1

scale_up = note_lists.generate_scale(note_lists.chromatic_up_notes, note_length)
scale_down = note_lists.generate_scale(note_lists.chromatic_down_notes, note_length)
scale = scale_down[::-1] + scale_up[1:]
starting_height = 0.3
target_rpy_rates = np.array([0, 0, 0])

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default="cf2x",     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
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
    POSITION_CONTROL = 0
    ATTITUDE_CONTROL = 1
    ctrl.controller_type = POSITION_CONTROL


    #### Run the simulation ####################################

    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/ARGS.control_freq_hz))
    action = {str(i): np.array([0,0,0,0]) for i in range(ARGS.num_drones)}
    START = time.time()
    note_count=0
    note_start = time.time()
    for i in range(0, int(ARGS.duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):
        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)
        #### Compute control at the desired frequency ##############
        if i%CTRL_EVERY_N_STEPS == 0:
            #HOVER 3 seconds BEFORE STARTING
            if ctrl.controller_type == POSITION_CONTROL:
                action[str(0)], _, _ = ctrl.computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                    state=obs[str(0)]["state"],
                                                                    target_pos = np.array([0,0,starting_height]),
                                                                    target_rpy_rates=target_rpy_rates
                                                                    )

                if time.time()-note_start >= 3:
                    ctrl.controller_type = ATTITUDE_CONTROL
                    note_start=time.time()
                
            else : 
                #if all notes have been played, end simulation
                if note_count >= len(scale):
                    break
                    #ctrl.target_thrust = HOVER_THRUST

                #update target thrust with current note value
                else:
                    ctrl.target_thrust = scale[note_count][0]
                    #if note length is reached, play next note
                    if time.time()-note_start >=scale[note_count][1]:
                        note_count+=1
                        note_start=time.time()
                #### Compute control for the current way point ############   
                action[str(0)], _, _ = ctrl.computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                        state=obs[str(0)]["state"],
                                                                        target_pos = np.array([0,0,0]),
                                                                        target_rpy_rates=target_rpy_rates
                                                                        )


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
    #logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    logger.scale_plot(filename = 'chromatic_down_up_up_0-1.png')
