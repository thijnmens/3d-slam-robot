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
        self.robot_width: int = 0.21  # 210 mm
        self.robot_length: int = 0.20  # 200 mm
        self.wheel_diameter: int = 0.06  # 60 mm

        # Encoder details
        self.gear_ratio: float = 36 / 1
        self.pulses_per_rev: int = 11 * self.gear_ratio

        # Motor details
        self.front_left = Wheel(
            "FL",
            False,
            18,
            24,
            23,
            25,
            27,
            (1.0, 0.05, 0.005)
        )
        self.front_right = Wheel(
            "FR",
            True,
            12,
            1,
            22,
            16,
            20,
            (0.8, 0.05, 0.005)
        )
        self.rear_left = Wheel(
            "RL",
            False,
            19,
            21,
            26,
            0,
            11,
            (0.8, 0.05, 0.005)
        )
        self.rear_right = Wheel(
            "RR",
            True,
            13,
            6,
            5,
            9,
            10,
            (0.8, 0.05, 0.005)
        )

        # Parameters
        self.max_wheel_speed = 0.5
        self.max_pwm = 1

        # PWM scaling
        self.duty_min = 0.35
        self.duty_gain = 0.75

    def __iter__(self):
        yield self.front_left
        yield self.front_right
        yield self.rear_left
        yield self.rear_right