from src.controller import PIDController


def test_pid_creation():
    pid = PIDController()

    assert pid is not None


def test_pid_returns_numeric_output():

    pid = PIDController()

    output = pid.update(
        error=10.0,
        dt=0.01
    )

    assert isinstance(
        output,
        (int, float)
    )


def test_pid_zero_error():

    pid = PIDController()

    output = pid.update(
        error=0.0,
        dt=0.01
    )

    assert isinstance(
        output,
        (int, float)
    )
