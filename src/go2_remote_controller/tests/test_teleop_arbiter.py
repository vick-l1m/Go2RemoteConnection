from app.core.teleop_arbiter import TeleopArbiter


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_idle_page_zeros_are_dropped_while_another_drives():
    c = Clock()
    a = TeleopArbiter(hold_s=0.5, clock=c)
    assert a.accept("laptop", 0.75, 0.0, 0.0)
    c.t += 0.06
    assert not a.accept("phone", 0.0, 0.0, 0.0)       # the 2026-10-15 chatter
    c.t += 0.06
    assert a.accept("laptop", 0.75, 0.0, 0.0)
    assert a.dropped == 1


def test_driver_release_always_stops():
    c = Clock()
    a = TeleopArbiter(hold_s=0.5, clock=c)
    assert a.accept("laptop", 0.75, 0.0, 0.0)
    c.t += 0.1
    assert a.accept("laptop", 0.0, 0.0, 0.0)


def test_other_client_zero_passes_after_hold():
    c = Clock()
    a = TeleopArbiter(hold_s=0.5, clock=c)
    assert a.accept("laptop", 0.75, 0.0, 0.0)
    c.t += 0.6
    assert a.accept("phone", 0.0, 0.0, 0.0)


def test_nonzero_from_other_client_takes_over():
    c = Clock()
    a = TeleopArbiter(hold_s=0.5, clock=c)
    assert a.accept("laptop", 0.75, 0.0, 0.0)
    c.t += 0.1
    assert a.accept("phone", 0.0, 0.3, 0.0)
    assert a.driver == "phone" and a.contested == 1
    c.t += 0.1
    assert not a.accept("laptop", 0.0, 0.0, 0.0)


def test_anonymous_client_unchanged():
    a = TeleopArbiter()
    assert a.accept("", 0.0, 0.0, 0.0)
    assert a.accept("", 0.5, 0.0, 0.0)
