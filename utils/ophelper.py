import os
import sys
import subprocess
import numpy as np


def open_folder(path):
    if os.path.exists(path):
        dir_path = os.path.dirname(path)
        if sys.platform == 'win32':
            os.startfile(dir_path)
        else:
            opener = 'open' if sys.platform == 'darwin' else 'xdg-open'
            subprocess.call([opener, dir_path])


def combine_RT(R, t_x, t_y, t_z):
    """Combine rotation matrix and translation to homogeneous transformation matrix."""
    M = np.hstack([R, [[t_x], [t_y], [t_z]]])
    M = np.vstack((M, [0, 0, 0, 1]))
    return M
