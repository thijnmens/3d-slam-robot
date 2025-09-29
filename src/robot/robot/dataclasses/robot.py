from dataclasses import dataclass

from .wheel import Wheel


@dataclass
class Robot:
    front_left: Wheel
    front_right: Wheel
    rear_left: Wheel
    rear_right: Wheel
    wheel_base: int
    track_width: int
    wheel_diameter: int
    pulses_per_rev: int
    gear_ratio: float

    def __init__(self):
        # Robot details
        self.wheel_base: int = 200  # mm
        self.track_width: int = 210  # mm
        self.wheel_diameter: int = 60  # mm

        # Encoder details
        self.pulses_per_rev: int = 11
        self.gear_ratio: float = 36 / 1

        # Motor details
        self.front_left = Wheel(
            True,
            18,
            24,
            23,
            25,
            27
        )
        self.front_right = Wheel(
            False,
            12,
            1,
            22,
            16,
            20
        )
        self.rear_left = Wheel(
            False,
            19,
            21,
            26,
            0,
            11
        )
        self.rear_right = Wheel(
            True,
            13,
            6,
            5,
            9,
            10
        )

    def __iter__(self):
        yield self.front_left
        yield self.front_right
        yield self.rear_left
        yield self.rear_right

    def get_forward_velocity(self) -> float:
        return (  self.front_left.get_revolutions(self.wheel_diameter)
                + self.front_right.get_revolutions(self.wheel_diameter)
                + self.rear_left.get_revolutions(self.wheel_diameter)
                + self.rear_right.get_revolutions(self.wheel_diameter)
            ) / 4

    def get_right_velocity(self) -> float:
        return (- self.front_left.get_revolutions(self.wheel_diameter)
                + self.front_right.get_revolutions(self.wheel_diameter)
                + self.rear_left.get_revolutions(self.wheel_diameter)
                - self.rear_right.get_revolutions(self.wheel_diameter)
            ) / 4

    def get_angular_velocity(self) -> float:
        factor = 4.0 * (self.wheel_base + self.track_width)
        return (- self.front_left.get_revolutions(self.wheel_diameter)
                + self.front_right.get_revolutions(self.wheel_diameter)
                - self.rear_left.get_revolutions(self.wheel_diameter)
                + self.rear_right.get_revolutions(self.wheel_diameter)
            ) / factor
