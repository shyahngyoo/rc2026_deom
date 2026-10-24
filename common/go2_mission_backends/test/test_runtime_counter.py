from concurrent.futures import ThreadPoolExecutor
import sqlite3

import pytest

from go2_mission_backends.runtime_counter import RuntimeCounter


def test_restart_reserves_a_new_range_and_old_process_cannot_overtake(tmp_path):
    path = str(tmp_path / 'counters.sqlite3')
    old = RuntimeCounter('test', path)
    first = old.next()
    new = RuntimeCounter('test', path)
    restarted = new.next()
    assert first < old.next() < restarted
    old.value = old.limit - 2
    assert old.next() < restarted
    with pytest.raises(RuntimeError, match='exhausted'):
        old.next()
    assert RuntimeCounter('test', path).next() > restarted


def test_concurrent_startups_reserve_disjoint_ranges(tmp_path):
    path = str(tmp_path / 'counters.sqlite3')
    with ThreadPoolExecutor(max_workers=8) as pool:
        values = list(pool.map(lambda _: RuntimeCounter('test', path).next(), range(16)))
    assert len(set(values)) == 16
    assert all(b - a == RuntimeCounter.RANGE_SIZE for a, b in zip(sorted(values), sorted(values)[1:]))


def test_namespaces_reserve_independently(tmp_path):
    path = str(tmp_path / 'counters.sqlite3')
    assert RuntimeCounter('authority', path).next() == RuntimeCounter('telemetry:stm32', path).next()
    assert RuntimeCounter('authority', path).next() == 2 * RuntimeCounter.RANGE_SIZE


@pytest.mark.parametrize('value', [1, -1, 'invalid', (1 << 63) - (1 << 32)])
def test_invalid_or_exhausted_state_does_not_reset_counter(tmp_path, value):
    path = str(tmp_path / 'counters.sqlite3')
    RuntimeCounter('test', path)
    with sqlite3.connect(path) as db:
        db.execute('UPDATE counters SET upper_bound=? WHERE name=?', (value, 'test'))
    with pytest.raises(ValueError):
        RuntimeCounter('test', path)


def test_corrupt_database_fails_without_falling_back_to_zero(tmp_path):
    path = tmp_path / 'counters.sqlite3'
    path.write_bytes(b'not a sqlite database')
    with pytest.raises(sqlite3.DatabaseError):
        RuntimeCounter('test', str(path))


def test_default_location_uses_persistent_state_home(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    first = RuntimeCounter('test').next()
    assert RuntimeCounter('test').next() > first
    assert (tmp_path / 'rc2026/runtime_counters.sqlite3').is_file()
