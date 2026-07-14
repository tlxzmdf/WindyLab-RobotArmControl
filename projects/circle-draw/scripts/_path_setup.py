"""将 arm-platform demo 目录加入 import 路径，复用 pinocchio_ik。"""
import os
import sys

_ARM_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_DEMO_DIR = os.path.join(_ARM_ROOT, 'windylab_ws', 'src', 'arm-platform', 'demo')
if _DEMO_DIR not in sys.path:
    sys.path.insert(0, _DEMO_DIR)
