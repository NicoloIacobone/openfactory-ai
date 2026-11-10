import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# ROS-free modules: repo-level scripts and the pure core of openfactory_manipulation
sys.path.insert(0, str(REPO / 'scripts'))
sys.path.insert(0, str(REPO / 'src' / 'openfactory_manipulation'))
