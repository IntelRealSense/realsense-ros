#!/bin/bash

path=$(cd `dirname $0`; pwd)
source $path/roscd.bash

$path/roscd.bash $1
echo $absolute_path

