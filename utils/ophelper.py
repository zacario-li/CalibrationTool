import os, sys, subprocess

def open_folder(path):
    if os.path.exists(path):
        dir_path = os.path.dirname(path)
        if sys.platform == 'win32':
            os.startfile(dir_path)
        else:
            opener = 'open' if sys.platform == 'darwin' else 'xdg-open'
            subprocess.call([opener, dir_path])