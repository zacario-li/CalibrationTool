import os, sys, subprocess
import time
from loguru import logger

def timer_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        logger.debug(f'{func.__name__} took {end_time - start_time} seconds')
        return result
    return wrapper

def open_folder(path):
    if os.path.exists(path):
        dir_path = os.path.dirname(path)
        if sys.platform == 'win32':
            os.startfile(dir_path)
        else:
            opener = 'open' if sys.platform == 'darwin' else 'xdg-open'
            subprocess.call([opener, dir_path])