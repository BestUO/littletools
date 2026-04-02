def hello():
    return "Hello from Python!"


def add(a, b):
    return a + b


class Robot:
    def __init__(self, name: str, speed: float):
        self.name = name
        self.speed = speed

    def move(self, distance: float) -> str:
        time = distance / self.speed
        return f"{self.name} moved {distance}m in {time:.2f}s"

    def get_name(self) -> str:
        return self.name

    def set_speed(self, speed: float):
        self.speed = speed
