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
http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html

For simplicity, we're currently directly unpacking the format that we are 
encountering in the data, which is x,y,z,_,rgb,_,_,_ 
each as 32-bit little-endian floats
"""

#%%


from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosUint8, unpackRosUint32, \
                    unpackRosTimestamp

def importTopic(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    the result is are np arrays of float64 for:
        rotQ (4 cols, quaternion)
        angV (3 cols)
        acc (3 cols)
        mag (3 cols)
        temp (1 cols) - but I'll probably ignore this to start with
    '''
    #tempAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    #for msg in tqdm(msgs, position=0, leave=True):
    disable_bar = kwargs.get('disable_bar')
    tsByMessage = []
    pointsByMessage = []
    for msg in tqdm(msgs, disable=disable_bar):
        
        data = msg['data']
        ptr = 0
        seq, ptr = unpackRosUint32(data, ptr)
        ts, ptr = unpackRosTimestamp(data, ptr)
        frame_id, ptr = unpackRosString(data, ptr)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr) 

        if width > 0 and height > 0:

            arraySize, ptr = unpackRosUint32(data, ptr)
            for element in range(arraySize):
                # Move through the field definitions - we'll ignore these
                # until we encounter a file that uses a different set
                name, ptr = unpackRosString(data, ptr)
                offset, ptr = unpackRosUint32(data, ptr)
                datatype, ptr = unpackRosUint8(data, ptr)
                count, ptr = unpackRosUint32(data, ptr)
        
            isBigendian, ptr = unpackRosUint8(data, ptr)
            pointStep, ptr = unpackRosUint32(data, ptr)
            rowStep, ptr = unpackRosUint32(data, ptr)

            numPoints = width * height
            points = np.empty((numPoints, 3), dtype=np.float32)
            arraySize, ptr = unpackRosUint32(data, ptr)
            # assert arraySize = width*height
            for x in range(width):
                for y in range(height):            
                    points[x*height + y, :] = np.frombuffer(data[ptr:ptr+12], dtype=np.float32)
                    ptr += pointStep
            pointsByMessage.append(points)
            tsByMessage.append(np.ones((numPoints), dtype=np.float64) * ts)
    if not pointsByMessage: # None of the messages contained any points
        return None
    points = np.concatenate(pointsByMessage)        
    ts = np.concatenate(tsByMessage)        
    
    # Crop arrays to number of events
    outDict = {
        'ts': ts,
        'point': points,
        }
    return outDict
