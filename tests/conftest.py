import sys
from pathlib import Path

# ROS-free modules of the repo-level scripts
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
