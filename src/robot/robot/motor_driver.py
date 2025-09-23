from gpiozero import Motor, PWMOutputDevice, RotaryEncoder
from time import sleep

#encoder = RotaryEncoder(a=9, b=11, max_steps=0)

class MotorDriver:
    def __init__(self, config):

        self.FL_motor = Motor(forward=config["FL"]["in1"], backward=config["FL"]["in2"], pwm=False)
        self.FL_enable = PWMOutputDevice(config["FL"]["ena"], frequency=1000, initial_value=0)

        self.FR_motor = Motor(forward=config["FR"]["in1"], backward=config["FR"]["in2"], pwm=False)
        self.FR_enable = PWMOutputDevice(config["FR"]["ena"], frequency=1000, initial_value=0)

        self.RL_motor = Motor(forward=config["RL"]["in1"], backward=config["RL"]["in2"], pwm=False)
        self.RL_enable = PWMOutputDevice(config["RL"]["ena"], frequency=1000, initial_value=0)

        self.RR_motor = Motor(forward=config["RR"]["in1"], backward=config["RR"]["in2"], pwm=False)
        self.RR_enable = PWMOutputDevice(config["RR"]["ena"], frequency=1000, initial_value=0)

    def drive_fl(self, direction="forward", speed=1.0):
        self.FL_enable.value = speed
        getattr(self.FL_motor, direction)()

    def drive_fr(self, direction="forward", speed=1.0):
        self.FR_enable.value = speed
        getattr(self.FR_motor, direction)()

    def drive_rl(self, direction="forward", speed=1.0):
        self.RL_enable.value = speed
        getattr(self.RL_motor, direction)()

    def drive_rr(self, direction="forward", speed=1.0):
        self.RR_enable.value = speed
        getattr(self.RR_motor, direction)()

    def stop_all(self):
        for m, e in [
            (self.FL_motor, self.FL_enable),
            (self.FR_motor, self.FR_enable),
            (self.RL_motor, self.RL_enable),
            (self.RR_motor, self.RR_enable),
        ]:
            m.stop()
            e.value = 0.0

config = {
    "FL": {"in1": 13, "in2": 6, "ena": 19},
    "FR": {"in1": 17, "in2": 27, "ena": 22},
    "RL": {"in1": 23, "in2": 24, "ena": 25},
    "RR": {"in1": 5,  "in2": 12, "ena": 16},
}

driver = MotorDriver(config)

try:
    driver.drive_fl("forward", 1.0)
    sleep(5)
finally:
    driver.stop_all()


