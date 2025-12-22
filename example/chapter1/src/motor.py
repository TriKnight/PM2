class Motor:
    """
    Software representation of a 1-DOF motor actuator (Position Control).
    Command range: [0, 6.29]
    """

    def __init__(self) -> None:
        self._command = 0.0

    def set_command(self, value: float) -> None:
        if not 0.0 <= value <= 6.29:
            raise ValueError(f"Motor command out of range: {value}")
        self._command = value

    def command(self) -> float:
        return self._command
