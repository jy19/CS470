class PDController:
    def __init__(self):
        self.kd = 0.5
        self.kp = 0.5
        self.prev_error = 0

    def calc_error(self, error):
        """ calculate the error in angel between current location and target """
        change_in_error = self.kp * error + self.kd * self.prev_error
        self.prev_error = error
        # if error == self.prev_error:
        #     return change_in_error*0.05
        return change_in_error
