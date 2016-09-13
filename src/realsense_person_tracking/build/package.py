import argparse
import os
import shutil
import sys

# *** Helper methods ***

def exit_on_error(message):
	print('\n *** ERROR ***\n');
	print(message);
	sys.exit(1)

def replace_in_file(filename, old_string, new_string):
	s = open(filename).read()
	if old_string in s:
		s = s.replace(old_string, new_string)
		f = open(filename, 'w')
		f.write(s)
		f.flush()
		f.close()
	else:
		exit_on_error('No occurances of "{}" found.'.format(old_string))

# *** End - Helper methods ***

# parse command line

parser = argparse.ArgumentParser(description='Create Linux package.')
parser.add_argument('-v', '-version', action="store", dest="version", help="set package version", default="99")
parser.add_argument('-dst', '-destination', action="store", dest="destination", help="set destination directory", default="\\\\3dlabdev06\\Shared\\Linux\\Package")
parser.add_argument('-sdk-r', '-sdk-revision', action="store", dest='sdkr', help="set from which revision the so was created")
parser.add_argument('-ros-r', '-ros-revision', action="store", dest='rosr', help="set from which revision the ROS package was created")

options = parser.parse_args()

# constant variables - some should go to command line...

version = options.version
source  = os.path.dirname(os.path.realpath(__file__)) + "\\..\\..\\.."
destination = options.destination + '\\v' + version

# print parameters summary

print("Creating package...")
print("   Version:    ", version);
print("   Source:     ", source);
print("   Destination:", destination);


### Build package

# Create root directory
os.makedirs(destination)

# Copy ROS contents
rosDir = destination + '\\ROS\\src'
os.makedirs(rosDir)

shutil.copytree(source + '\\libs\\persontracking'	, rosDir + '\\libs\\persontracking')
shutil.copytree(source + "\\Misc"					, rosDir + "\\Misc")

shutil.copytree(source + "\\Vision\\realsense_person_tracking"		, rosDir + "\\Vision\\realsense_person_tracking")
shutil.copytree(source + "\\Vision\\realsense_person_tracking_test"	, rosDir + "\\Vision\\realsense_person_tracking_test")
#copy latest realsense node - supports IR streams
shutil.copytree(source + "\\Vision\\realsense_ros"	, rosDir + "\\Vision\\realsense_ros")

#replace xml values
#	change TrackingScoreToDeleteID -200 -> -50
#replace_in_file(rosDir + '\\libs\\persontracking\\data\\person_tracking\\algo_config.xml', 'TrackingScoreToDeleteID value="-200"', 'TrackingScoreToDeleteID value="-50"')
#	enable ir streams in r200 node
replace_in_file(rosDir + '\\Vision\\realsense_ros\\launch\\realsense_r200_launch.launch', 'name="enableLR" default="0"', 'name="enableLR" default="1"')

shutil.make_archive(destination + "\\ROS", 'zip', destination + "\\ROS")

# Copy standalone contents
shutil.copytree(source + '\\libs\\persontracking', destination + "\\Standalone")

shutil.make_archive(destination + "\\Standalone", 'zip', destination + "\\Standalone")

### track source versions (TFS revisions)
sdkr = options.sdkr
rosr = options.rosr
if sdkr:
	open(destination + "\\sdk.changeset." + sdkr, 'w')
if rosr:
	open(destination + "\\ros.changeset." + rosr, 'w')
