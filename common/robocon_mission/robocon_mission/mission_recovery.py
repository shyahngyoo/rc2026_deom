"""Bounded mission-level retry policy; Nav2 owns its internal recovery."""


class RecoveryPolicy:
    def __init__(self, max_navigation_retries):
        if max_navigation_retries < 0:
            raise ValueError("max_navigation_retries must be nonnegative")
        self.default_retries = max_navigation_retries
        self.attempts = {}

    def retry_navigation(self, task, phase):
        key = (task.id, phase)
        count = self.attempts.get(key, 0)
        limit = (self.default_retries if task.max_retries is None
                 else task.max_retries)
        if count >= limit:
            return False
        self.attempts[key] = count + 1
        return True
