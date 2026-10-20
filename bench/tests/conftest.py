import os
import sys

BENCH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)
