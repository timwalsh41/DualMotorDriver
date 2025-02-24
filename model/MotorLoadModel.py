import numpy as np

# A simple model for a DC motor with a load attached. The motor is specified by the maximum speed in rad/s, the stall
# current in A, the supply voltage in V, and the load inertia and damping. The model is updated by providing a voltage
# and a time step, and the model will calculate the new speed and acceleration based on the motor parameters and the
# load parameters. The model also calculates the current based on the voltage and the motor resistance. We neglect
# the motor inductance in this model.
# We assume in the model that the motor back EMF constant is defined by the ratio of the supply voltage and the maximum
# speed, and that the torque constant is equal to the back EMF constant.
class MotorLoadModel:
    def __init__(self,
                 max_speed_rad_per_s,
                 supply_V=12.0,
                 stall_current_A=5.0,   
                 load_inertia_kgm2=1e-6,
                 load_damping_Nm_per_rad_s=5e-5    # 1e-5 is a reasonable value that will limit the top speed but not too much
                 ):
        
        self.resistance_ohm = supply_V/stall_current_A
        self.back_emf_constant_V_per_rad_s = supply_V / max_speed_rad_per_s
        self.torque_constant_Nm_per_A = self.back_emf_constant_V_per_rad_s
        self.stall_current_A = stall_current_A
        self.load_inertia_kgm2 = load_inertia_kgm2
        self.load_damping_Nm_per_rad_s = load_damping_Nm_per_rad_s

        # Speed in rad/s and acceleration in rad/s^2
        self.speed_rad_per_s = 0.0
        self.acceleration_rad_per_s2 = 0.0
        self.current_A = 0.0

    def update(self, voltage_V, dt_s):
        # Calculate the back electromotive force (V)
        back_emf_V = self.back_emf_constant_V_per_rad_s * self.speed_rad_per_s
        
        # Update the motor current (A) using the voltage difference and motor parameters
        self.current_A = (voltage_V - back_emf_V)/self.resistance_ohm
        
        # Calculate the motor torque (Nm)
        torque_Nm = self.torque_constant_Nm_per_A * self.current_A
        
        # Calculate the load torque due to damping (Nm)
        load_torque_Nm = self.load_damping_Nm_per_rad_s * self.speed_rad_per_s
        
        # Calculate the net torque (Nm)
        net_torque_Nm = torque_Nm - load_torque_Nm
        
        # Calculate the angular acceleration (rad/s^2)
        self.acceleration_rad_per_s2 = net_torque_Nm / self.load_inertia_kgm2
        
        # Update the motor speed (rad/s)
        self.speed_rad_per_s += self.acceleration_rad_per_s2 * dt_s
        
        return self.speed_rad_per_s, self.acceleration_rad_per_s2

# A model for a gearmotor, which is a DC motor with integrated gearbox, and optionally an encoder. Typically
# these are specified by the gear ratio, the maximum output speed, the rated supply voltage, and the stall current.
# The encoder counts per revolution is also required if an encoder is present. This class extends the MotorLoadModel
# to manage transforming between speed at the motor itself and at the output shaft (e.g. through the gearbox).
class GearMotorLoadModel(MotorLoadModel):
    def __init__(self, gear_ratio: int, max_output_speed_rpm: float, encoder_cpr: int, supply_V: float = 12, stall_current_A: float = 5, **kwargs):
        if not isinstance(gear_ratio, int) or gear_ratio <= 0:
            raise TypeError("gear_ratio must be an int > 0")
        if not isinstance(max_output_speed_rpm, (int, float)) or max_output_speed_rpm <= 0:
            raise TypeError("max_output_speed_rpm must be a number > 0")
        if not isinstance(encoder_cpr, int) or encoder_cpr <= 0:
            raise TypeError("encoder_cpr must be an int > 0")
        if not isinstance(supply_V, (int, float)) or supply_V <= 0:
            raise TypeError("supply_V must be a number > 0")
        if not isinstance(stall_current_A, (int, float)) or stall_current_A <= 0:
            raise TypeError("stall_current_A must be a number > 0")

        self._gear_ratio = gear_ratio
        self._max_output_speed_rpm = max_output_speed_rpm
        self._encoder_cpr = encoder_cpr

        # the output shaft spins slower than the motor itself by the gear ratio
        self._max_motor_speed_rpm = max_output_speed_rpm * gear_ratio

        # we care about the speed at the motor in rad/s as the motor model equations are all in SI units,
        # so the calculated model speed is in rad/s
        self._max_motor_speed_rad_per_s = self._max_motor_speed_rpm * 2 * np.pi / 60

        # we care about the speed in counts per second as this is what we can measure
        self._max_motor_speed_cps = self._max_motor_speed_rad_per_s * encoder_cpr / (2 * np.pi)

        # we may also be interested in the output speed in rad/s
        self._max_output_speed_rad_per_s = self._max_output_speed_rpm * 2 * np.pi / 60

        super().__init__(self._max_motor_speed_rad_per_s, supply_V, stall_current_A, **kwargs)

    @property
    def max_motor_speed_rpm(self):
        return self._max_motor_speed_rpm

    @property
    def max_motor_speed_rad_per_s(self):
        return self._max_motor_speed_rad_per_s

    @property
    def max_motor_speed_cps(self):
        return self._max_motor_speed_cps

    @property
    def max_output_speed_rpm(self):
        return self._max_output_speed_rpm

    @property
    def max_output_speed_rad_per_s(self):
        return self._max_output_speed_rad_per_s

    @property
    def gear_ratio(self):
        return self._gear_ratio

    @property
    def encoder_cpr(self):
        return self._encoder_cpr

    @property
    def current_motor_speed_rad_per_s(self):
        return self.speed_rad_per_s

    @property
    def current_motor_speed_cps(self):
        return self.speed_rad_per_s * self.encoder_cpr / (2 * np.pi)

    @property
    def current_motor_speed_rpm(self):
        return self.speed_rad_per_s * 60 / (2 * np.pi)

    @property
    def current_output_speed_rad_per_s(self):
        return self.speed_rad_per_s / self.gear_ratio

    @property
    def current_output_speed_rpm(self):
        return self.current_output_speed_rad_per_s * 60 / (2 * np.pi)

    @property
    def current_motor_acceleration_rad_per_s2(self):
        return self.acceleration_rad_per_s2

    def motor_speed_cps_to_output_speed_rpm(self, motor_speed_cps):
        return motor_speed_cps * 60 / self.encoder_cpr / self.gear_ratio