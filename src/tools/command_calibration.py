import numpy as np

class LongitudinalCalibration():
    """
    Calibrate the mapping from v,throttle,brake to acc/dec
    """

    def __init__(self):
        self.last_speed = None
        pass

    def run_step(self, target_speed, current_speed, debug=False, dt=0.05):
        """
        target is not useful
        """
        pass
        if self.last_speed is not None:
            acc = (current_speed - self.last_speed)/dt
            # save file
        control = 0.1
