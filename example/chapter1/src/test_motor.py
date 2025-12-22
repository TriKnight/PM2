from motor import Motor
import pytest

def test_motor_command_range():
    motor = Motor()
    motor.set_command(0.5)
    assert motor.command() == 0.5

def test_motor_invalid_command():
    motor = Motor()
    with pytest.raises(ValueError):
        motor.set_command(2.0)
