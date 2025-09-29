import math
from dataclasses import dataclass

@dataclass
class Wheel:
    forward: bool
    pulses: float
    pwm: int
    in1: int
    in2: int
    encoder_a: int
    encoder_b: int

    def __init__(self, forward: bool, pwm: int, in1: int, in2: int, encoder_a: int, encoder_b: int):
        self.forward = forward
        self.pulses = 0
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b

    def update_pulses(self, pulses: int):
        self.pulses = pulses

    def get_revolutions(self, diameter: int) -> float:
        return diameter * self.pulses * math.pi