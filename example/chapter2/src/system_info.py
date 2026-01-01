class SystemInfo:
    def __init__(self, name: str, version: str) -> None:
        self._name = name
        self._version = version

    def describe(self) -> str:
        return f"{self._name} (v{self._version})"
