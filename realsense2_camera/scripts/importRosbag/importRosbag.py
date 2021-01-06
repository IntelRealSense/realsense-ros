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

Unpacks a rosbag into its topics and messages
Uses the topic types to interpret the messages from each topic, 
yielding dicts for each topic containing iterables for each field.
By default unpacks all topics, but you can use any of the following keyword 
params to limit which topics are intepretted:
    - 'listTopics' = True - no unpacking - just returns a list of the topics contained in 
       the file and their associated types
    - 'importTopics' = <list of strings> - only imports the listed topics
    - 'importTypes' = <list of strings> - only imports the listed types

Message types supported are strictly those listed in the initial imports section
of this file. There are a selection of standard message types and a couple
related to event-based sensors. 
The method of importation is honed to the particular needs of
the author, sometimes ignoring certain fields, grouping data in particular ways 
etc. However it should serve as a model for anyone who wishes to import rosbags.
Although it's possible to import messages programmatically given only the 
message definition files, we have chosen not to do this, because if we did it
we would anyway need to take the resulting data and pick out the bits we wanted. 

"""

#%%

from struct import unpack
from struct import error as structError
from tqdm import tqdm

# Local imports

from .messageTypes.common import unpackHeader

from .messageTypes.dvs_msgs_EventArray import importTopic as import_dvs_msgs_EventArray
from .messageTypes.esim_msgs_OpticFlow import importTopic as import_esim_msgs_OpticFlow
from .messageTypes.geometry_msgs_PoseStamped import importTopic as import_geometry_msgs_PoseStamped
from .messageTypes.geometry_msgs_Transform import importTopic as import_geometry_msgs_Transform
from .messageTypes.geometry_msgs_TransformStamped import importTopic as import_geometry_msgs_TransformStamped
from .messageTypes.geometry_msgs_TwistStamped import importTopic as import_geometry_msgs_TwistStamped
from .messageTypes.sensor_msgs_CameraInfo import importTopic as import_sensor_msgs_CameraInfo
from .messageTypes.sensor_msgs_Image import importTopic as import_sensor_msgs_Image
from .messageTypes.sensor_msgs_Imu import importTopic as import_sensor_msgs_Imu
from .messageTypes.sensor_msgs_PointCloud2 import importTopic as import_sensor_msgs_PointCloud2
from .messageTypes.tf_tfMessage import importTopic as import_tf_tfMessage

import logging
    
def importTopic(topic, **kwargs):
    msgs = topic['msgs']
    topicType = topic['type'].replace('/','_')
    if topicType == 'dvs_msgs_EventArray': topicDict = import_dvs_msgs_EventArray(msgs, **kwargs)
    elif topicType == 'esim_msgs_OpticFlow': topicDict = import_esim_msgs_OpticFlow(msgs, **kwargs)
    elif topicType == 'geometry_msgs_PoseStamped': topicDict = import_geometry_msgs_PoseStamped(msgs, **kwargs)
    elif topicType == 'geometry_msgs_Transform': topicDict = import_geometry_msgs_Transform(msgs, **kwargs)
    elif topicType == 'geometry_msgs_TransformStamped': topicDict = import_geometry_msgs_TransformStamped(msgs, **kwargs)
    elif topicType == 'geometry_msgs_TwistStamped': topicDict = import_geometry_msgs_TwistStamped(msgs, **kwargs)
    elif topicType == 'sensor_msgs_CameraInfo': topicDict = import_sensor_msgs_CameraInfo(msgs, **kwargs)
    elif topicType == 'sensor_msgs_Image': topicDict = import_sensor_msgs_Image(msgs, **kwargs)
    elif topicType == 'sensor_msgs_Imu': topicDict = import_sensor_msgs_Imu(msgs, **kwargs)
    elif topicType == 'sensor_msgs_PointCloud2': topicDict = import_sensor_msgs_PointCloud2(msgs, **kwargs)
    elif topicType == 'tf_tfMessage': topicDict = import_tf_tfMessage(msgs, **kwargs)
    else: 
        return None
    if topicDict:
        topicDict['rosbagType'] = topic['type']
    return topicDict

def readFile(filePathOrName):
    global disable_bar
    logging.info('Attempting to import ' + filePathOrName + ' as a rosbag 2.0 file.')
    with open(filePathOrName, 'rb') as file:
        # File format string
        fileFormatString = file.readline().decode("utf-8")
        logging.info('ROSBAG file format: ' + fileFormatString)
        if fileFormatString != '#ROSBAG V2.0\n':
            logging.error('This file format (%s) might not be supported' % fileFormatString)
        eof = False
        conns = []
        chunks = []
        while not eof:
            # Read a record header
            try:
                headerLen = unpack('=l', file.read(4))[0]
            except structError:
                if len(file.read(1)) == 0: # Distinguish EOF from other struct errors 
                   # a struct error could also occur if the data is downloaded by one os and read by another.
                   eof = True
                   continue
            # unpack the header into fields 
            headerBytes = file.read(headerLen)
            fields = unpackHeader(headerLen, headerBytes)
            # Read the record data
            dataLen = unpack('=l', file.read(4))[0]
            data = file.read(dataLen)
            # The op code tells us what to do with the record
            op = unpack('=b', fields['op'])[0]
            fields['op'] = op
            if op == 2:
                # It's a message
                # AFAIK these are not found unpacked in the file
                #fields['data'] = data 
                #msgs.append(fields)
                pass
            elif op == 3:
                # It's a bag header - use this to do progress bar for the read
                chunkCount = unpack('=l', fields['chunk_count'])[0]
                pbar = tqdm(total=chunkCount, position=0, leave=True, disable=disable_bar)
            elif op == 4:
                # It's an index - this is used to index the previous chunk
                conn = unpack('=l', fields['conn'])[0]
                count = unpack('=l', fields['count'])[0]
                for idx in range(count):
                    time, offset = unpack('=ql', data[idx*12:idx*12+12])
                    chunks[-1]['ids'].append((conn, time, offset))
            elif op == 5:
                # It's a chunk
                fields['data'] = data
                fields['ids'] = []
                chunks.append(fields)
                pbar.update(len(chunks))
            elif op == 6:
                # It's a chunk-info - seems to be redundant
                pass
            elif op == 7:
                # It's a conn
                # interpret data as a string containing the connection header
                connFields = unpackHeader(dataLen, data)
                connFields.update(fields) 
                connFields['conn'] = unpack('=l', connFields['conn'])[0]
                connFields['topic'] = connFields['topic'].decode("utf-8")
                connFields['type'] = connFields['type'].decode("utf-8").replace('/', '_')
                conns.append(connFields)
    return conns, chunks

#%% Break chunks into msgs

def breakChunksIntoMsgs(chunks):
    global disable_bar
    msgs = [] 
    logging.debug('Breaking chunks into msgs ...')           
    for chunk in tqdm(chunks, position=0, leave=True, disable=disable_bar):
        for idx in chunk['ids']:
            ptr = idx[2]
            headerLen = unpack('=l', chunk['data'][ptr:ptr+4])[0]
            ptr += 4
            # unpack the header into fields 
            headerBytes = chunk['data'][ptr:ptr+headerLen]
            ptr += headerLen
            fields = unpackHeader(headerLen, headerBytes)
            # Read the record data
            dataLen = unpack('=l', chunk['data'][ptr:ptr+4])[0]
            ptr += 4
            fields['data'] = chunk['data'][ptr:ptr+dataLen]
            fields['conn'] = unpack('=l', fields['conn'])[0]
            msgs.append(fields)
    return msgs

def rekeyConnsByTopic(connDict):
    topics = {}
    for conn in connDict:
        topics[connDict[conn]['topic']] = connDict[conn]
    return topics


def importRosbag(filePathOrName, **kwargs):
    global disable_bar
    disable_bar = kwargs.get('disable_bar')
    loglevel = kwargs.get('log')
    numeric_level = getattr(logging, loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    logging.getLogger().setLevel(numeric_level)

    logging.info('Importing file: ' + filePathOrName) 
    conns, chunks = readFile(filePathOrName)
    # Restructure conns as a dictionary keyed by conn number
    connDict = {}
    for conn in conns:
        connDict[conn['conn']] = conn
        conn['msgs'] = []
    if kwargs.get('listTopics', False):
        topics = rekeyConnsByTopic(connDict)
        logging.info('Topics in the file are (with types):')
        for topicKey, topic in topics.items():
            del topic['conn']
            del topic['md5sum']
            del topic['msgs']
            del topic['op']
            del topic['topic']
            topic['message_definition'] = topic['message_definition'].decode("utf-8")
            logging.info('    ' + topicKey + ' --- ' + topic['type'])
        return topics
    msgs = breakChunksIntoMsgs(chunks)
    for msg in msgs:     
        connDict[msg['conn']]['msgs'].append(msg)
    topics = rekeyConnsByTopic(connDict)

    importedTopics = {}
    importTopics = kwargs.get('importTopics')
    importTypes = kwargs.get('importTypes')
    if importTopics is not None:
        for topicToImport in importTopics:
            for topicInFile in list(topics.keys()):
                if topicInFile == topicToImport:
                    importedTopic = importTopic(topics[topicInFile], **kwargs)
                    if importedTopic is not None:
                        importedTopics[topicToImport] = importedTopic
                        del topics[topicInFile]            
    elif importTypes is not None:
        for typeToImport in importTypes:
            typeToImport = typeToImport.replace('/', '_')
            for topicInFile in list(topics.keys()):
                if topics[topicInFile]['type'].replace('/', '_') == typeToImport:
                    importedTopic = importTopic(topics[topicInFile], **kwargs)
                    if importedTopic is not None:
                        importedTopics[topicInFile] = importedTopic
                        del topics[topicInFile]    
    else: # import everything
        for topicInFile in list(topics.keys()):
            importedTopic = importTopic(topics[topicInFile], **kwargs)
            if importedTopic is not None:
                importedTopics[topicInFile] = importedTopic
                del topics[topicInFile]

    logging.info('')
    if importedTopics:
        logging.info('Topics imported are:')
        for topic in importedTopics.keys():
            logging.info(topic + ' --- ' + importedTopics[topic]['rosbagType'])
            #del importedTopics[topic]['rosbagType']
        logging.info('')

    if topics:
        logging.info('Topics not imported are:')
        for topic in topics.keys():
            logging.info(topic + ' --- ' + topics[topic]['type'])
        logging.info('')
    
    return importedTopics

