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

The importRosbag function receives a list of messages and returns
a dict with one field for each data field in the message, where the field
will contain an appropriate iterable to contain the interpretted contents of each message.

This function imports the ros message type defined at:

https://github.com/uzh-rpg/rpg_esim/blob/master/event_camera_simulator/esim_msgs/msg/OpticFlow.msg

"""

#%%

from tqdm import tqdm
import numpy as np

from .common import unpackRosFloat32Array, unpackRosUint32, \
                    unpackRosTimestamp, unpackRosString

def importTopic(msgs, **kwargs):
    disable_bar = kwargs.get('disable_bar')
    tsAll = []
    flowMaps = []
    for msg in tqdm(msgs, disable=disable_bar):
        
        data = msg['data']
        ptr = 0
        seq, ptr = unpackRosUint32(data, ptr) # Not used
        ts, ptr = unpackRosTimestamp(data, ptr)
        frame_id, ptr = unpackRosString(data, ptr) # Not used
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr) 

        if width > 0 and height > 0:
            arraySize, ptr = unpackRosUint32(data, ptr)
            #assert arraySize == width*height
            flowMapX, ptr = unpackRosFloat32Array(data, width*height, ptr)
            arraySize, ptr = unpackRosUint32(data, ptr)
            #assert arraySize == width*height
            flowMapY, ptr = unpackRosFloat32Array(data, width*height, ptr)
            flowMap = np.concatenate((flowMapX.reshape(height, width, 1), 
                                      flowMapY.reshape(height, width, 1)), 
                                     axis=2)
            flowMaps.append(flowMap)
            tsAll.append(ts)
    if not tsAll:
        return None
    outDict = {
        'ts': np.array(tsAll, dtype=np.float64),
        'flowMaps': flowMaps,
        }
    return outDict


