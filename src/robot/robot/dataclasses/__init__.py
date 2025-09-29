import math
from dataclasses import dataclass

@dataclass
class Wheel:
    _id: int
    forward: bool
    pulses: float

    def __init__(self, _id: int, forward: bool):
        self._id = _id
        self.forward = forward
        self.pulses = 0

    def update_pulses(self, pulses: int):
        self.pulses = pulses

    def get_revolutions(self, diameter: int) -> float:
        return diameter * self.pulses * math.pi


@dataclass
class Wheels:
    front_left: Wheel
    front_right: Wheel
    rear_left: Wheel
    rear_right: Wheel

    def __init__(self, front_left: Wheel, front_right: Wheel, rear_left: Wheel, rear_right: Wheel):
        self.front_left = front_left
        self.front_right = front_right
        self.rear_left = rear_left
        self.rear_right = rear_right

    def __iter__(self):
        yield self.front_left
        yield self.front_right
        yield self.rear_left
        yield self.rear_right

    def get_forward_velocity(self, diameter: int) -> float:
        return (  self.front_left.get_revolutions(diameter)
                + self.front_right.get_revolutions(diameter)
                + self.rear_left.get_revolutions(diameter)
                + self.rear_right.get_revolutions(diameter)
            ) / 4

    def get_right_velocity(self, diameter: int) -> float:
        return (- self.front_left.get_revolutions(diameter)
                + self.front_right.get_revolutions(diameter)
                + self.rear_left.get_revolutions(diameter)
                - self.rear_right.get_revolutions(diameter)
            ) / 4

    def get_angular_velocity(self, wheel_base: int, track_width: int, diameter: int) -> float:
        factor = 4.0 * (wheel_base + track_width)
        return (- self.front_left.get_revolutions(diameter)
                + self.front_right.get_revolutions(diameter)
                - self.rear_left.get_revolutions(diameter)
                + self.rear_right.get_revolutions(diameter)
            ) / factor