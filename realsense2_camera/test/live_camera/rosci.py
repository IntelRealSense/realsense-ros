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
sys.path.append( os.path.join( os.environ['WORKSPACE'], 'lrs/unit-tests/py' ))
from rspy import log, file, repo, libci
start_time = time.time()
current_dir = os.path.dirname( os.path.abspath( __file__ ) )
print(f'{current_dir}')
root = os.path.dirname( os.path.dirname( os.path.dirname( os.path.dirname( os.path.dirname( os.path.abspath( __file__ ))))))
dir_live_tests = os.path.join( os.environ['WORKSPACE'], 'ros2/realsense2_camera/test/live_camera' )

hub_reset = False
logdir = None
handle = None
def usage():
    ourname = os.path.basename( sys.argv[0] )
    print( 'Syntax: ' + ourname + ' [options] [dir]' )
    print( '        dir: location of executable tests to run' )
    print( 'Options:' )
    print( '        --debug              Turn on debugging information (does not include LibRS debug logs; see --rslog)' )
    print( '        -v, --verbose        Errors will dump the log to stdout' )
    print( '        -q, --quiet          Suppress output; rely on exit status (0=no failures)' )
    print( '        -s, --stdout         Do not redirect stdout to logs' )
    print( '        -r, --regex          Run all tests whose name matches the following regular expression' )
    print( '        --list-tests         Print out all available tests. This option will not run any tests' )
    print( '        --repeat <#>         Repeat each test <#> times' )
    print( '        --config <>          Ignore test configurations; use the one provided' )
    print( '        --device <>          Run only on the specified devices; ignore any test that does not match (implies --live)' )
    print( '        --no-reset           Do not try to reset any devices, with or without a hub' )
    print( '        --hub-reset          If a hub is available, reset the hub itself' )
    print( '        --skip-disconnected  Skip live test if required device is disconnected (only applies w/o a hub)' )
    print( 'Examples:' )
    print( 'Running: python3.10 rosci.py -s' )
    print( '    Runs all tests, but direct their output to the console rather than log files' )
    print( 'Running: python3.10 rosci.py --list-tests' )
    print( "    Will find all tests and print" )

    sys.exit( 2 )

def command(dev_name):
       cmd =  ['pytest-3']
       cmd += ['-s']
       cmd += ['-m', ''.join(dev_name)]
       cmd += ['-k', 'test_camera_imu_tests'] 
       #cmd += ['-m', 'd415']
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

device_set = None
try:
    opts, args = getopt.getopt( sys.argv[1:], 'hvqr:st:',
                                longopts=['help', 'verbose', 'debug', 'quiet', 'regex=', 'stdout', 'tag=', 'list-tags',
                                          'list-tests', 'no-exceptions', 'context=', 'repeat=', 'config=', 'no-reset', 'hub-reset',
                                          'rslog', 'skip-disconnected', 'live', 'not-live', 'device='] )
except getopt.GetoptError as err:
    log.e( err )  # something like "option -a not recognized"
    usage()
for opt, arg in opts:
    if opt in ('-h', '--help'):
        usage()
    elif opt == '--device':
        device_set = arg.split()
        for dev in device_set:
             print(dev)        
#Get into action    
try:
    logdir = os.path.join( os.path.dirname( os.path.dirname( os.path.dirname( os.path.abspath( __file__ )))), 'log')
    os.makedirs( logdir, exist_ok=True )
    print(logdir)
    
   

    #Import '_device_by_sn' from devices.py module of librealsense repo
    from rspy import devices
    if not devices.hub:
         assert False, 'No hub available'
    else:
        devices.hub.connect()
    devices.query( hub_reset = hub_reset )
    #devices.map_unknown_ports()
    global _device_by_sn         

    if not devices._device_by_sn:
         assert False, 'No Camera device detected!'
    else:
         #Loop in for all devices and run tests
         for device in devices._device_by_sn.values():
              print(f' Device found: {device.name} ')
         for device in devices._device_by_sn.values():
            if device.name == 'D455':
                print('Running test on device:', device.name)
                cmd = command(str(device.name).lower())
                run_test(cmd, device.name, stdout=logdir, append =False)
            elif device.name != 'D455':
                 print('Skipping test on device:', device.name)

finally:
        devices.hub.disconnect()
        log.i("Log path- \"Build Artifacts\":/ros2/realsense_camera/log ")
        run_time = time.time() - start_time
        log.d( "server took", run_time, "seconds" )

sys.exit( 0 )
