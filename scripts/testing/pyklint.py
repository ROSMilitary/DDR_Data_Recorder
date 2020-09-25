#******************************************************************************
#
#"Distribution A: Approved for public release; distribution unlimited. OPSEC #4046"
#
#PROJECT: DDR
#
# PACKAGE         :
# ORIGINAL AUTHOR :
# MODIFIED DATE   :
# MODIFIED BY     :
# REVISION        :
#
# Copyright (c) 2020 DCS Corporation
#
# Unlimited Rights assigned to the U.S. Government
#
# This material may be reproduced by or for the U.S Government pursuant
# to the copyright license under the clause at DFARS 252.227-7013.  This
# notice must appear in all copies of this file and its derivatives.
#******************************************************************************
#
#Copyright (c) 2019-2020 U.S. Federal Government (in countries where recognized)
#Permission is hereby granted, free of charge, to any person obtaining a copy of
#this software and associated documentation files (the "Software"), to deal in
#the Software without restriction, including without limitation the rights to use,
#copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
#Software, and to permit persons to whom the Software is furnished to do so,
#subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
#DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#DEALINGS IN THE SOFTWARE.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.



import argparse
from pathlib import Path
import re
import sys
from pylint import epylint as lint

## Lints the python scripts in the directory.
# @param directory
def lint_directory(directory=None):
    scores = []
    for i in directory.iterdir():
        if i.is_dir():
            lint_directory(i)
        elif i.is_file() and i.as_posix().endswith('.py'):
            pylint_stdout, pylint_stderr = \
                    lint.py_run(i.as_posix(), return_std=True)
            if pylint_stderr.getvalue():
                print('There was an error parsing file: ', i.as_posix())
                return

            output = pylint_stdout.getvalue()
            if "Your code has been rated at 10.00/10" not in output:
                score = re.search(\
                    r'Your code has been rated at ([-0-9\.]+)/10', output)
                if score:
                    scores.append((i.as_posix(), score.group(1)))
                else:
                    print('***There was a problem linting file: ', i.as_posix())

    if len(scores) == 0:
        print(directory, ' has passed!')
        return

    max_length = 0

    for i in scores:
        temp_len = len(i[0])
        if temp_len > max_length:
            max_length = temp_len

    gap = max_length + 3

    for i in scores:
        file_path = i[0] + ':'
        print(file_path, ' '*(gap - len(file_path)), i[1])


## The main method
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", metavar="directory",\
            help="Directory to generate markdown")
    flags, _ = parser.parse_known_args(sys.argv[1:])


    directory = Path(flags.directory)
    if directory and directory.is_dir():
        lint_directory(directory)
    else:
        print('There was an error parsing the directory. Please try again.')


if __name__ == '__main__':
    main()
