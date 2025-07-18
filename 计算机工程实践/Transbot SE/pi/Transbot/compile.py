#!/usr/bin/env python3
# coding=utf-8
"""
# file=input name cfile=output name
# python3 compile.py
"""

import py_compile
import os
py_compile.compile(file="transbot_main.py", cfile="transbot_main.pyc", optimize=-1)

os.system("chmod +x transbot_main.pyc")
