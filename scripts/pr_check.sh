#!/bin/bash

# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

# This script makes sure all files with the following extensions - .h, .hpp, .cpp, .js, .py, .bat, .sh, .txt -
# include the Apache license reference and Intel copyright, as it shown in this file header.
# The script checks also that these files are using Unix line-endings and spaces as delmiters (instead of tabs).
# It is recommended to run this script when adding new files to the project.
# For more info, run ./pr_check.sh --help

set +e

sudo apt-get install dos2unix

ok=0
fixed=0


function check_folder {
    for filename in $(find $1 -type f \( -iname \*.cpp -o -iname \*.h -o -iname \*.hpp -o -iname \*.js -o -iname \*.bat -o -iname \*.sh -o -iname \*.txt -o -iname \*.py \)); do

        # Skip files of 3rd-party libraries which already have their own licenses and copyrights
        if [[ "$filename" == *"importRosbag"* ]]; then
            continue;
        fi

        if [[ $(grep -oP "Software License Agreement" $filename | wc -l) -ne 0 ]]; then
            echo "[WARNING] $filename contains 3rd-party license agreement"
        else
            if [[ ! $filename == *"usbhost"* ]]; then
                # Only check files that are not .gitignore-d
                if [[ $(git check-ignore $filename | wc -l) -eq 0 ]]; then
                    if [[ $(grep -oP "(?<=\(c\) )(.*)(?= Intel)" $filename | wc -l) -eq 0 ]]; then
                        echo "[ERROR] $filename is missing the copyright notice"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            if [[ $(date +%Y) == "2022" ]]; then
                                if [[ $filename == *".h"* || $filename == *".hpp"* || $filename == *".cpp"* || $filename == *".js"* ]]; then
                                    echo "Trying to auto-resolve...";
                                    ex -sc '1i|// Copyright(c) 2022 Intel Corporation. All Rights Reserved.' -cx $filename
                                    fixed=$((fixed+1))
                                fi
                                if [[ $filename == *".txt"* || $filename == *".py"* ]]; then
                                    echo "Trying to auto-resolve...";
                                    ex -sc '1i|# Copyright(c) 2022 Intel Corporation. All Rights Reserved.' -cx $filename
                                    fixed=$((fixed+1))
                                fi
                            else
                                echo Please update pr_check to auto-resolve missing copyright
                            fi
                        fi
                    fi

                    if [[ $(grep -oP "Apache 2.0" $filename | wc -l) -eq 0 ]]; then
                        echo "[ERROR] $filename is missing license notice"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            if [[ $filename == *".h"* || $filename == *".hpp"* || $filename == *".cpp"* || $filename == *".js"* ]]; then
                                echo "Trying to auto-resolve...";
                                ex -sc '1i|// License: Apache 2.0. See LICENSE file in root directory.' -cx $filename
                                fixed=$((fixed+1))
                            fi
                            if [[ $filename == *".txt"* || $filename == *".py"* ]]; then
                                echo "Trying to auto-resolve...";
                                ex -sc '1i|# License: Apache 2.0. See LICENSE file in root directory.' -cx $filename
                                fixed=$((fixed+1))
                            fi
                        fi
                    fi

                    if [[ $(grep -o -P '\t' $filename | wc -l) -ne 0 ]]; then
                        echo "[ERROR] $filename has tabs (this project is using spaces as delimiters)"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            echo "Trying to auto-resolve...";
                            sed -i.bak $'s/\t/    /g' $filename
                            fixed=$((fixed+1))
                        fi
                    fi

                    if [[ $(file ${filename} | grep -o -P 'CRLF' | wc -l) -ne 0 ]]; then
                        echo "[ERROR] $filename is using DOS line endings (this project is using Unix line-endings)"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            echo "Trying to auto-resolve...";
                            dos2unix $filename
                            fixed=$((fixed+1))
                        fi
                    fi
                fi
            fi
        fi
    done
}

if [[ $1 == *"help"* ]]; then
    echo Pull-Request Check tool
    echo "Usage: (run from repo scripts directory)"
    echo "    ./pr_check.sh [--help] [--fix]"
    echo "    --fix    Try to auto-fix defects"
    exit 0
fi


cd ..
check_folder . $1
cd scripts

if [[ ${fixed} -ne 0 ]]; then
    echo "Re-running pr_check..."
    ./pr_check.sh
else
    if [[ ${ok} -ne 0 ]]; then
        echo Pull-Request check failed, please address ${ok} the errors reported above
        exit 1
    fi
fi

exit 0
