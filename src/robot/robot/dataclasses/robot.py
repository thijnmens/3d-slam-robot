import math
from dataclasses import dataclass

from .wheel import Wheel


@dataclass
class Robot:
    front_left: Wheel
    front_right: Wheel
    rear_left: Wheel
    rear_right: Wheel
    robot_width: int
    robot_length: int
    wheel_diameter: int
    pulses_per_rev: int
    gear_ratio: float

    def __init__(self):
        # Robot details
        self.robot_width: int = 210  # mm
        self.robot_length: int = 200  # mm
        self.wheel_diameter: int = 60  # mm

        # Encoder details
        self.pulses_per_rev: int = 11
        self.gear_ratio: float = 36 / 1

        # Motor details
        self.front_left = Wheel(
            "front_left",
            True,
            18,
            24,
            23,
            25,
            27
        )
        self.front_right = Wheel(
            "front_right",
            False,
            12,
            1,
            22,
            16,
            20
        )
        self.rear_left = Wheel(
            "rear_left",
            False,
            19,
            21,
            26,
            0,
            11
        )
        self.rear_right = Wheel(
            "rear_right",
            True,
            13,
            6,
            5,
            9,
            10
        )

        # Parameters
        self.max_wheel_speed = 10.0
        self.max_pwm = 1
        self.min_pwm = -1

    def __iter__(self):
        yield self.front_left
        yield self.front_right
        yield self.rear_left
        yield self.rear_right

    def pulses_to_distance(self, pulses: int):
        revolutions = pulses / (self.pulses_per_rev * self.gear_ratio)
        return self.wheel_diameter * math.pi * revolutions

    def get_forward_velocity(self) -> float:

        return (  self.pulses_to_distance(self.front_left.pulses)
                + self.pulses_to_distance(self.front_right.pulses)
                + self.pulses_to_distance(self.rear_left.pulses)
                + self.pulses_to_distance(self.rear_right.pulses)
            ) / 4

    def get_right_velocity(self) -> float:
        return (- self.pulses_to_distance(self.front_left.pulses)
                + self.pulses_to_distance(self.front_right.pulses)
                + self.pulses_to_distance(self.rear_left.pulses)
                - self.pulses_to_distance(self.rear_right.pulses)
            ) / 4

    def get_angular_velocity(self) -> float:
        factor = 4.0 * (self.robot_width + self.robot_length)
        return (- self.pulses_to_distance(self.front_left.pulses)
                + self.pulses_to_distance(self.front_right.pulses)
                - self.pulses_to_distance(self.rear_left.pulses)
                + self.pulses_to_distance(self.rear_right.pulses)
            ) / factor
