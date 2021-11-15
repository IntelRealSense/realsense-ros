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

"""

#%%

from struct import unpack
import numpy as np

def unpackHeader(headerLen, headerBytes):
    fields = {}
    ptr = 0
    while ptr < headerLen:
        fieldLen = unpack('=l', headerBytes[ptr:ptr+4])[0]
        ptr += 4
        #print(fieldLen)
        field = headerBytes[ptr:ptr+fieldLen]
        ptr += fieldLen
        #print(field)
        fieldSplit = field.find(b'\x3d')
        fieldName = field[:fieldSplit].decode("utf-8")
        fieldValue = field[fieldSplit+1:]
        fields[fieldName] = fieldValue
    return fields

def unpackRosUint32(data, ptr):
    return unpack('=L', data[ptr:ptr+4])[0], ptr+4

def unpackRosUint8(data, ptr):
    return unpack('=B', data[ptr:ptr+1])[0], ptr+1

def unpackRosString(data, ptr):
    stringLen = unpack('=L', data[ptr:ptr+4])[0]
    ptr += 4
    try:
        outStr = data[ptr:ptr+stringLen].decode('utf-8')
    except UnicodeDecodeError:
        outStr = 'UnicodeDecodeError'
    ptr += stringLen
    return outStr, ptr

def unpackRosFloat64Array(data, num, ptr):
    return np.frombuffer(data[ptr:ptr+num*8], dtype=np.float64), ptr+num*8 
    
def unpackRosFloat32Array(data, num, ptr):
    return np.frombuffer(data[ptr:ptr+num*4], dtype=np.float32), ptr+num*4 
    
def unpackRosFloat32(data, ptr):
    return unpack('=f', data[ptr:ptr+4])[0], ptr+4 
    
def unpackRosTimestamp(data, ptr):
    timeS, timeNs = unpack('=LL', data[ptr:ptr+8])
    timeFloat = np.float64(timeS)+np.float64(timeNs)*0.000000001 
    return timeFloat, ptr+8
