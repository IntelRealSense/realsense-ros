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
https://github.com/uzh-rpg/rpg_dvs_ros/tree/master/dvs_msgs/msg
"""

#%%

from tqdm import tqdm
import numpy as np

# local imports

from .common import unpackRosString, unpackRosUint32

def importTopic(msgs, **kwargs):

    disable_bar = kwargs.get('disable_bar')
    tsByMessage = []
    xByMessage = []
    yByMessage = []
    polByMessage = []
    for msg in tqdm(msgs, position=0, leave=True, disable=disable_bar):
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        #timeS, timeNs = unpack('=LL', data[4:12])
        frame_id, ptr = unpackRosString(data, 12)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr) 
        numEventsInMsg, ptr = unpackRosUint32(data, ptr)
        # The format of the event is x=Uint16, y=Uint16, ts = Uint32, tns (nano seconds) = Uint32, pol=Bool  
        # Unpack in batch into uint8 and then compose
        dataAsArray = np.frombuffer(data[ptr:ptr+numEventsInMsg*13], dtype=np.uint8)
        dataAsArray = dataAsArray.reshape((-1, 13), order='C')
        # Assuming big-endian
        xByMessage.append((dataAsArray[:, 0] + dataAsArray[:, 1] * 2**8).astype(np.uint16))
        yByMessage.append((dataAsArray[:, 2] + dataAsArray[:, 3] * 2**8).astype(np.uint16))
        ts = ((dataAsArray[:, 4] + \
              dataAsArray[:, 5] * 2**8 + \
              dataAsArray[:, 6] * 2**16 + \
              dataAsArray[:, 7] * 2**24 ).astype(np.float64))
        tns = ((dataAsArray[:, 8] + \
               dataAsArray[:, 9] * 2**8 + \
               dataAsArray[:, 10] * 2**16 + \
               dataAsArray[:, 11] * 2**24).astype(np.float64))
        tsByMessage.append(ts + tns / 1000000000) # Combine timestamp parts, result is in seconds
        polByMessage.append(dataAsArray[:, 12].astype(np.bool))
    outDict = {
        'x': np.concatenate(xByMessage),
        'y': np.concatenate(yByMessage),
        'ts': np.concatenate(tsByMessage),
        'pol': np.concatenate(polByMessage),
        'dimX': width,
        'dimY': height}
    return outDict
