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

The importTopic function receives a list of messages and returns
a dict with one field for each data field in the message, where the field
will contain an appropriate iterable to contain the interpretted contents of each message.
In some cases, static info is repeated in each message; in which case a field may not contain an iterable. 

This function imports the ros message type defined at:    
http://docs.ros.org/api/tf/html/msg/tfMessage.html

Each message contains an array of transform_stamped messages

The result is a ts plus a 7-column np array of np.float64,
where the cols are x, y, z, q-w, q-x, q-y, q-z, (i.e. quaternion orientation)

NOTE: QUATERNION ORDER GETS MODIFIED from xyzw to wxyz

NOTE - this code is similar to geometry_msgs_TransformStamped
"""

#%%

from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosTimestamp, \
                    unpackRosFloat64Array, unpackRosUint32

def importTopic(msgs, **kwargs):
    #if 'Stamped' not in kwargs.get('messageType', 'Stamped'):
    #    return interpretMsgsAsPose6qAlt(msgs, **kwargs)
    disable_bar = kwargs.get('disable_bar')
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    poseAll = np.zeros((sizeOfArray, 7), dtype=np.float64)
    frameIdAll = []
    childFrameIdAll = []
    idx = 0
    for msg in tqdm(msgs, position=0, leave=True, disable=disable_bar):
        data = msg['data']
        numTfInMsg, ptr = unpackRosUint32(data, 0)
        for tfIdx in range(numTfInMsg): 
            while sizeOfArray <= idx + numTfInMsg:
                tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
                poseAll = np.concatenate((poseAll, np.zeros((sizeOfArray, 7), dtype=np.float64)))
                sizeOfArray *= 2
            seq, ptr = unpackRosUint32(data, ptr)
            tsAll[idx], ptr = unpackRosTimestamp(data, ptr)
            frame_id, ptr = unpackRosString(data, ptr)
            frameIdAll.append(frame_id)
            child_frame_id, ptr = unpackRosString(data, ptr)
            childFrameIdAll.append(child_frame_id)
            poseAll[idx, :], ptr = unpackRosFloat64Array(data, 7, ptr)
            idx += 1
    # Crop arrays to number of events
    numEvents = idx
    tsAll = tsAll[:numEvents]
    poseAll = poseAll[:numEvents]
    point = poseAll[:, 0:3]
    rotation = poseAll[:, [6, 3, 4, 5]] # Switch quaternion form from xyzw to wxyz
    outDict = {
        'ts': tsAll,
        'point': point,
        'rotation': rotation,
        'frameId': np.array(frameIdAll, dtype='object'),
        'childFrameId': np.array(childFrameIdAll, dtype='object')}
    return outDict
