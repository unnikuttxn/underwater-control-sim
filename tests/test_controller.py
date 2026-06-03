from src.controller import PIDController

def test_pid_output_numeric():
    pid=PIDController()
    out=pid.update(10,0.01)
    assert isinstance(out,(int,float))
