import sys
import os

def path():
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
