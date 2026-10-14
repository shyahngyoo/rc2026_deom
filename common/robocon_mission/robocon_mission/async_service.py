"""Single nonblocking service operation with stale callback isolation."""
import time
from typing import Callable, Optional


class AsyncServiceRunner:
    def __init__(self, client):
        self.client = client
        self.pending: Optional[dict] = None
        self.unknown_terminal = False

    def start(self, token: int, request, timeout: float, done: Callable,
              validate: Callable, mutating: bool = False) -> None:
        if self.pending is not None or (mutating and self.unknown_terminal):
            raise RuntimeError("service operation still active/unknown")
        self.pending = dict(token=token, request=request, done=done,
                            validate=validate, mutating=mutating,
                            deadline=time.monotonic() + timeout, future=None)

    def _finish(self, op, outcome, terminal, result=None):
        if self.pending is not op:
            return
        self.pending = None
        if not terminal:
            self.unknown_terminal = True
        op["done"](op["token"], outcome, terminal, result)

    def _response(self, op, future):
        if self.pending is not op:
            return
        try:
            result = future.result()
            valid = op["validate"](result)
        except Exception as error:
            self._finish(op, f"service_exception:{error}", not op["mutating"])
            return
        if time.monotonic() >= op["deadline"]:
            self._finish(op, "service_timeout", True, result)
            return
        self._finish(op, "succeeded" if valid else "service_rejected", True, result)

    def tick(self):
        op = self.pending
        if op is None:
            return
        if time.monotonic() >= op["deadline"]:
            # A sent mutation cannot be canceled by canceling the local Future.
            self._finish(op, "service_timeout",
                         not (op["mutating"] and op["future"] is not None))
        elif op["future"] is None and self.client.service_is_ready():
            try:
                op["future"] = self.client.call_async(op["request"])
                op["future"].add_done_callback(lambda f: self._response(op, f))
            except Exception as error:
                self._finish(op, f"service_send_exception:{error}", not op["mutating"])

    def cancel(self):
        op = self.pending
        if op is not None:
            self._finish(op, "canceled",
                         not (op["mutating"] and op["future"] is not None))

