#!/usr/bin/python3
"""Run generated client or minimally wrapped risk exporter with wall-clock logging."""
import importlib.util
import os
from pathlib import Path
import runpy
import sys
from runtime_profile import timed

role=sys.argv[1]
path=Path(sys.argv[2])
sys.argv=[str(path)]+sys.argv[3:]
if role=='client':
    runpy.run_path(str(path),run_name='__main__')
else:
    spec=importlib.util.spec_from_file_location('profiled_risk',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    cls=module.RiskCostmap
    cls.callback=timed(cls.callback,'risk.grid_callback',stamp=lambda self,msg:msg.info.header.stamp.to_nsec())
    cls.publish=timed(cls.publish,'risk.cvar_and_publish')
    module.decode_grid_layer=timed(module.decode_grid_layer,'risk.decode_grid_layer')
    module.score_to_cost=timed(module.score_to_cost,'risk.five_level_encode')
    import rospy
    rospy.init_node('risk_costmap');cls();rospy.spin()
