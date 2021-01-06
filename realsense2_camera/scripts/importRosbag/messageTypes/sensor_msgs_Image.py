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
http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
"""

#%%

from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosUint32, unpackRosUint8, unpackRosTimestamp

def importTopic(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    the result is a ts plus a 2d array of samples ()
    '''
    disable_bar = kwargs.get('disable_bar')

    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    framesAll = []
    for idx, msg in enumerate(tqdm(msgs, position=0, leave=True, disable=disable_bar)):
        if sizeOfArray <= idx:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            sizeOfArray *= 2            
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        if kwargs.get('useRosMsgTimestamps', False):
            tsAll[idx], _ = unpackRosTimestamp(msg['time'], 0)
        else:
            tsAll[idx], _ = unpackRosTimestamp(data, 4)
        frame_id, ptr = unpackRosString(data, 12)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr)
        fmtString, ptr = unpackRosString(data, ptr)
        isBigendian, ptr = unpackRosUint8(data, ptr)
        if isBigendian:
            print('data is bigendian, but it doesn''t matter')            
        step, ptr = unpackRosUint32(data, ptr) # not used
        arraySize, ptr = unpackRosUint32(data, ptr)
        # assert arraySize == height*width
        
        # The pain of writing this scetion will continue to increase until it
        # matches this reference implementation:
        # http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
        if fmtString in ['mono8', '8UC1']:
            frameData = np.frombuffer(data[ptr:ptr+height*width],np.uint8)
            depth = 1
        elif fmtString in ['mono16', '16UC1']:
            frameData = np.frombuffer(data[ptr:ptr+height*width*2],np.uint16)
            depth = 1
        elif fmtString in ['bgr8', 'rgb8']:
            frameData = np.frombuffer(data[ptr:ptr+height*width*3],np.uint8)
            depth = 3
        elif fmtString in ['bgra8', 'rgba8']:
            frameData = np.frombuffer(data[ptr:ptr+height*width*4],np.uint8)
            depth = 4
        elif fmtString == '16SC1':
            frameData = np.frombuffer(data[ptr:ptr+height*width*2],np.int16)
            depth = 1
        elif fmtString == '32FC1':
            frameData = np.frombuffer(data[ptr:ptr+height*width*4],np.float32)
            depth = 1
        else:
            print('image format not supported:' + fmtString)
            return None
        if depth > 1:
            frameData = frameData.reshape(height, width, depth)
        else:
            frameData = frameData.reshape(height, width)
            
        framesAll.append(frameData)
    numEvents = idx + 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    outDict = {
        'ts': tsAll,
        'frames': framesAll}
    return outDict
