# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
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
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import note_lists

import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

logging.basicConfig(level=logging.ERROR)

n_length = 1
scale_up = note_lists.generate_scale(note_lists.chromatic_up_notes, note_length = n_length)
scale_down = note_lists.generate_scale(note_lists.chromatic_down_notes, note_length = n_length)

nice_scale = note_lists.amazing_grace_scale

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        print("connected !")
        Thread(target=self._fly_scale).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _take_off(self):
        for i in range(50):
            pitch = 0
            roll = 0
            yawrate = 0
            self._cf.commander.send_setpoint(roll, pitch, yawrate, int(note_lists.HOVER_THRUST))
            time.sleep(0.1)

        for i in range(10):
            pitch = 0
            roll = 0
            yawrate = 0
            self._cf.commander.send_setpoint(roll, pitch, yawrate, int(note_lists.HOVER_THRUST))
            time.sleep(0.1)



    def _fly_scale(self):
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        self._take_off()

        scale  = scale_up[:-1] + scale_up[::-1] + scale_down[1:]
        print(scale)
        for note in nice_scale:
            print(note)
            thrust = note[0]
            thrust = int(thrust)
            note_length = note[1]
            start_time = time.time()
            while(time.time()-start_time < note_length):
                self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
                time.sleep(0.05)

        """                 
                thrust = int(note_lists.HOVER_THRUST) - 2000
                for i in range(50):
                    self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
                    time.sleep(0.1) """

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        
        self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = MotorRampExample(uri)
""" 
         for chord in chords:
            yawrate = chord[0]
            note_length = chord[1]
            thrust = note_lists.HOVER_THRUST
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(note_length) """
