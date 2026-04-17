"""
Database - Calibration Tool
"""

import sqlite3
import logging
from utils.ophelper import rot_2_quat, quat_2_rot
from functools import lru_cache

logger = logging.getLogger(__name__)

# Database helper class
class Database:
    def __len__(self):
        pass

    def create_table(self, table_name: str, table_schema: str) -> bool:
        """Create table if not exists.
        table_name: table name
        table_schema: table schema

        Returns: True if successful
        """
        pass

    def retrive_data(self, table_name: str, conditions: str = '') -> list:
        """Retrieve data from table."""
        pass

    def modify_data(self, table_name: str, conditions: str, values: dict = None) -> bool:
        """Modify data in table."""
        pass
