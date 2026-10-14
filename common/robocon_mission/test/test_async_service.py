from robocon_mission.async_service import AsyncServiceRunner


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None
    def add_done_callback(self, callback):
        self.callbacks.append(callback)
    def result(self):
        return self.value
    def resolve(self, value):
        self.value = value
        for callback in self.callbacks:
            callback(self)


class Client:
    def __init__(self):
        self.future = Future()
    def service_is_ready(self):
        return True
    def call_async(self, request):
        return self.future


def test_mutating_timeout_is_unknown_and_late_ack_cannot_succeed(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr("robocon_mission.async_service.time.monotonic", lambda: clock[0])
    client, outcomes = Client(), []
    runner = AsyncServiceRunner(client)
    runner.start(1, object(), 1.0, lambda *args: outcomes.append(args), bool, True)
    runner.tick()
    clock[0] = 2
    runner.tick()
    client.future.resolve(True)
    assert len(outcomes) == 1 and outcomes[0][:3] == (1, "service_timeout", False)
    assert runner.unknown_terminal


def test_read_only_cancel_is_terminal_and_old_response_ignored():
    client, outcomes = Client(), []
    runner = AsyncServiceRunner(client)
    runner.start(1, object(), 1, lambda *args: outcomes.append(args), bool)
    runner.tick()
    old = client.future
    runner.cancel()
    client.future = Future()
    runner.start(2, object(), 1, lambda *args: outcomes.append(args), bool)
    runner.tick()
    old.resolve(True)
    assert len(outcomes) == 1
    client.future.resolve(True)
    assert [row[:3] for row in outcomes] == [(1, "canceled", True), (2, "succeeded", True)]

