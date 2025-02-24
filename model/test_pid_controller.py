import PidControllerWrapper

class DummyEncoder(PidControllerWrapper.IEncoder):
    def __init__(self):
        super().__init__()
        self.count = 0

    def update_count(self, channel_A, channel_B):
        pass

    def update_delta(self):
        return 1

    def reset_count(self):
        self.count = 0

    def get_count(self):
        return self.count

    def get_last_count(self):
        return self.count

    def set_count(self, count):
        self.count = count

def main():
    encoder = DummyEncoder()
    pid = PidControllerWrapper.PidController(1.0, 0.1, 0.01, 0.5, encoder)
    pid.set_target_speed(100.0)
    pid.set_mode(PidControllerWrapper.ControlMode.CLOSED_LOOP)

    for _ in range(10):
        control_signal = pid.run_controller()
        print(f"Control Signal: {control_signal}")

if __name__ == "__main__":
    main()
