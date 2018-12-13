#!/usr/bin/env bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" # gets the current dir
PKG=$(cd ${DIR}/..; printf %s "$PWD") # gets this ROS pkg path

echo -e "package \e[1;4m"${PKG##*/}"\e[0m":
for test in $(find ${DIR} -name *.test -printf '%f\n'); do # finds the test file names
    FAILS=$(rostest ${PKG##*/} ${test} | grep "[a-zA-Z_][a-zA-Z_0-9]*\]\[FAILURE") # executes the tests and stores in FAILS
    SUITE="suite \e[1;4m"${test%.*}"\e[0m":
    if [ -z "${FAILS}" ] ; then
        echo -e "    "${SUITE}"\e[32m[SUCCESS]\e[0m" # suite was successful
    else
        echo -e "    "${SUITE}"\e[31m[FAILURE]\e[0m" # suite was unsuccessful
        for msg in ${FAILS}; do
            CASE=${msg%]*}; CASE=${CASE%]*}; CASE=${CASE##*/} # fetch case from GREP line
            echo "        "${CASE}
        done
    fi
done