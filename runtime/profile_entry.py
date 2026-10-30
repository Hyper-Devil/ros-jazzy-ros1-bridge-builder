#!/usr/bin/python3
from pathlib import Path
import runpy
import sys
folder=Path(__file__).resolve().parent/'profiling'
sys.path.insert(0,str(folder))
runpy.run_path(str(folder/'ros_entry.py'),run_name='__main__')
