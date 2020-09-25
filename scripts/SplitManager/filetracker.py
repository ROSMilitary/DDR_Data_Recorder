#!/usr/bin/python3

#******************************************************************************
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



import json
import pathlib

class FileTracker(dict):
    def __init__(self, file_path):
        self._file_path = file_path
        self._opened = False


    @property
    def file_path(self):
        return self._file_path


    @file_path.setter
    def file_path(self, value):
        raise RuntimeError('Setting filepath not allowed')


    @property
    def opened(self):
        return self._opened


    @opened.setter
    def opened(self, value):
        raise RuntimeError('Setting opened not allowed')


    def __enter__(self):
        self.open()
        return self


    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.close()


    def open(self):
        if self.opened:
            raise RuntimeError('Already opened')
        p = pathlib.Path(self.file_path)
        # Had to check if file exists and create it if it doesn't
        # w+ would do this but w+ overwrites the file.
        if not p.is_file():
            p.touch()
        self._file = open(self.file_path, 'r+')
        try:
            contents = json.load(self._file)
            self.update(contents)
        except json.decoder.JSONDecodeError:
            pass
        self._opened = True


    def close(self):
        if not self.opened:
            raise RuntimeError('Not opened')
        self._file.close()
        self._opened = False


    def update_persistant(self):
        if not self.opened:
            raise RuntimeError('Not opened')
        self._file.seek(0)
        json.dump(self, self._file)
        self._file.truncate()


    def __delitem__(self, key):
        super().__delitem__(key)
        if self.opened:
            self.update_persistant()


    def __setitem__(self, key, value):
        super().__setitem__(key, value)
        if self.opened:
            self.update_persistant()


    def update(self, other=None, **kwargs):
        super().update(other, **kwargs)
        if self.opened:
            self.update_persistant()


if __name__ == '__main__':
    with FileTracker("test.json") as ft:
        ft["key"] = 1
        val = ft['key']
        input()
        del ft['key']
