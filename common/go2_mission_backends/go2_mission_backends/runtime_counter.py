"""Durable, disjoint counter ranges for ROS publishers across process restarts."""
import os
from pathlib import Path
import sqlite3


class RuntimeCounter:
    # One process can emit over six years of values at 20 Hz. A retired process
    # never reserves another range, so it cannot overtake a newer instance.
    RANGE_SIZE = 1 << 32
    MAX_VALUE = (1 << 63) - 1  # SQLite integer range, also valid ROS uint64.

    def __init__(self, namespace, state_path=''):
        if not namespace:
            raise ValueError('counter namespace is required')
        state_home = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state'))
        path = Path(state_path) if state_path else state_home / 'rc2026/runtime_counters.sqlite3'
        path.parent.mkdir(parents=True, exist_ok=True)
        connection = sqlite3.connect(str(path), timeout=10)
        try:
            connection.execute('PRAGMA synchronous=FULL')
            connection.execute('BEGIN IMMEDIATE')
            connection.execute('CREATE TABLE IF NOT EXISTS counters '
                               '(name TEXT PRIMARY KEY, upper_bound INTEGER NOT NULL)')
            row = connection.execute('SELECT upper_bound FROM counters WHERE name=?',
                                     (namespace,)).fetchone()
            start = self.RANGE_SIZE if row is None else row[0]
            if (type(start) is not int or start < self.RANGE_SIZE
                    or start % self.RANGE_SIZE or start + self.RANGE_SIZE > self.MAX_VALUE):
                raise ValueError('counter state invalid or exhausted')
            self.limit = start + self.RANGE_SIZE
            connection.execute('INSERT OR REPLACE INTO counters VALUES (?, ?)',
                               (namespace, self.limit))
            connection.commit()  # Persist the reservation before emitting any value.
        finally:
            connection.close()
        self.value = start - 1

    def next(self):
        if self.value + 1 >= self.limit:
            raise RuntimeError('runtime counter range exhausted; restart node')
        self.value += 1
        return self.value
