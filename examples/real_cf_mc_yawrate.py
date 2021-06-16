# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

The MotionCommander uses velocity setpoints.

Change the URI variable to your Crazyflie configuration.
"""
from examples.note_lists import generate_rate_scale
import logging
import time

import note_lists
from vel_to_acc import compute_velocity_commands
import pickle

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

#vz_rates = note_lists.generate_rate_scale(5, 0, 1, 0.3)
#scale = vz_rates[:-1] + vz_rates[::-1]
log_data = []

def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    log_data.append((timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable("controller.cmd_thrust", "float")
    lg_stab.add_variable("stateEstimate.vz", "float")
    lg_stab.add_variable("pwm.m1_pwm", "uint32_t")
    lg_stab.add_variable("pwm.m2_pwm", "uint32_t")
    lg_stab.add_variable("pwm.m3_pwm", "uint32_t")
    lg_stab.add_variable("pwm.m4_pwm", "uint32_t")

    y_rates = [90 for i in range(50)]
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_log_async(scf, lg_stab)
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)
            # There is also a set of functions that start a motion. The
            # Crazyflie will keep on going until it gets a new command.
            for y_rate in y_rates:
                print(y_rate)
                mc._set_vel_setpoint(0, 0, 0, y_rate)
                time.sleep(0.1)
            mc.stop()
            # We land when the MotionCommander goes out of scope
    """with open(f'logdata_{int(time.time())}.pkl', 'w+') as f:
        pickle.dump(log_data, f)
    """

    