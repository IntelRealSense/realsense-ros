# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys, os, subprocess, re, platform, getopt, time
start_time = time.time()
running_on_ci = False
if 'WORKSPACE' in os.environ:
    #Path for ROS-CI on Jenkins
    ws_rosci = os.environ['WORKSPACE']
    sys.path.append( os.path.join( ws_rosci, 'lrs/unit-tests/py' ))
    running_on_ci = True
else:
    #For running this script locally
    #Extract the root where both realsense-ros and librealsense are cloned
    ws_local = '/'.join(os.path.abspath( __file__ ).split( os.path.sep )[0:-5])
    #expected to have 'librealsense' repo in parallel to 'realsense-ros'
    assert os.path.exists( os.path.join(ws_local, 'librealsense')), f" 'librealsense' doesn't exist at {ws_local} "
    sys.path.append( os.path.join( ws_local, 'librealsense/unit-tests/py' ))

dir_live_tests = os.path.dirname(__file__)
from rspy import log, libci
hub_reset = False
logdir = None
handle = None
test_ran = False

def usage():
    ourname = os.path.basename( sys.argv[0] )
    print( 'Syntax: ' + ourname + ' [options] ' )
    print( 'Options:' )
    print( '        -h, --help            Usage help' )
    print( '            --debug           Turn on debugging information (does not include LibRS debug logs; see --rslog)' )
        
    sys.exit( 2 )

def command(dev_name):
       cmd =  ['pytest-3']
       cmd += ['-s']
       cmd += ['-m', ''.join(dev_name)]
       cmd += ['-k', 'test_camera_imu_tests'] 
       cmd += [''.join(dir_live_tests)]
       cmd += ['--debug']
       return cmd

def run_test(cmd, dev_name, stdout=None, append =False):
        handle = None
        try:
            stdout = stdout + os.sep + str(dev_name) + '_' + "test_camera_imu_tests" + ".log"
            if stdout is None:
                sys.stdout.flush()
            elif stdout and stdout != subprocess.PIPE:
                if append:
                    handle = open( stdout, "a" )
                    handle.write(
                        "\n----------TEST-SEPARATOR----------\n\n" )
                    handle.flush()
                else:
                    handle = open( stdout, "w" )

            result = subprocess.run( cmd,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                    timeout=200,
                    check=True )
            if not result.returncode:
                 log.i("---Test Passed---")
        except Exception as e:
             log.e("---Test Failed---")
             log.w( "Error Exception:\n ",e )
                         
        finally:
            if handle:
                handle.close()      

try:
    opts, args = getopt.getopt( sys.argv[1:], 'h', longopts=['help', 'debug' ] )
except getopt.GetoptError as err:
    log.e( err )  # something like "option -a not recognized"
    usage()

for opt, arg in opts:
    if opt in ('-h', '--help'):
        usage()
      
#Find device, Run test    
try:
    #logs are stored @ ./realsense2_camera/test
    logdir = os.path.join( '/'.join(os.path.abspath( __file__ ).split( os.path.sep )[0:-2]), 'logs')
    os.makedirs( logdir, exist_ok=True )

    #Import '_device_by_sn' from devices.py module of librealsense repo
    from rspy import devices
    if not devices.hub:
         assert False, 'No hub available'
    else:
        devices.hub.connect()

    devices.query( hub_reset = hub_reset )
    global _device_by_sn         

    if not devices._device_by_sn:
         assert False, 'No Camera device detected!'
    else:
        #Loop in for all devices and run tests
        for device in devices._device_by_sn.values():
            log.i(f'Device found: {device.name} ')
            if device.name == 'D455':
                log.i('Running test on device:', device.name)
                cmd = command(str(device.name).lower())
                run_test(cmd, device.name, stdout=logdir, append =False)
                test_ran = True
            else:
                log.i('Skipping test on device:', device.name)

finally:
        devices.hub.disconnect()
        if running_on_ci:
            log.i("Log path- \"Build Artifacts\":/ros2/realsense_camera/test/logs ")
        else:
             log.i("log path:", logdir)
        assert test_ran, "No test cases ran"
        run_time = time.time() - start_time
        log.d( "server took", run_time, "seconds" )

sys.exit( 0 )
