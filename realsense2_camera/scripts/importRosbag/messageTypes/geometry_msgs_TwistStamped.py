# -*- coding: utf-8 -*-

"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of importRosbag.

The interpretMessages function receives a list of messages and returns
a dict with one field for each data field in the message, where the field
will contain an appropriate iterable to contain the interpretted contents of each message.

This function imports the ros message type defined at:
http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html
"""

#%%

from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosTimestamp, unpackRosFloat64Array

def importTopic(msgs, **kwargs):
    disable_bar = kwargs.get('disable_bar')
    sizeOfArray = 1024
    ts = np.zeros((sizeOfArray), dtype=np.float64)
    linV = np.zeros((sizeOfArray, 3), dtype=np.float64)
    angV = np.zeros((sizeOfArray, 3), dtype=np.float64)
    for idx, msg in enumerate(tqdm(msgs, position=0, leave=True, disable=disable_bar)):
        if sizeOfArray <= idx:
            ts = np.append(ts, np.zeros((sizeOfArray), dtype=np.float64))
            linV = np.concatenate((linV, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            angV = np.concatenate((angV, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            sizeOfArray *= 2
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        ts[idx], ptr = unpackRosTimestamp(data, 4)
        frame_id, ptr = unpackRosString(data, ptr)
        linV[idx, :], ptr = unpackRosFloat64Array(data, 3, ptr)
        angV[idx, :], _ = unpackRosFloat64Array(data, 3, ptr)
    # Crop arrays to number of events
    numEvents = idx + 1
    ts = ts[:numEvents]
    linV = linV[:numEvents]
    angV = angV[:numEvents]
    outDict = {
        'ts': ts,
        'linV': linV,
        'angV': angV}
    return outDict

