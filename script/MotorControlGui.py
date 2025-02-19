import tkinter as tk
import time
import threading
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import DualMotorCommander

# range of speeds for selector
SPEED_RANGE_CPS = 1e4 # 1e4 cps ~ 200 rpm for 50:1 motor
UPDATE_TIME_S = 0.025
class MotorControlGUI:
    def __init__(self, controller: DualMotorCommander):
        self._master = tk.Tk()
        self._master.title("Motor Controller Interface")
        self.controller = controller

        self.pending_commands = []

        # Speed Control
        self.left_speed = tk.DoubleVar(value=0)
        self.right_speed = tk.DoubleVar(value=0)
        self.left_target_speed_data = []
        self.right_target_speed_data = []

        self.create_scale("Left Speed", self.left_speed, -SPEED_RANGE_CPS, SPEED_RANGE_CPS, row=0)
        self.create_scale("Right Speed", self.right_speed, -SPEED_RANGE_CPS, SPEED_RANGE_CPS, row=1)

        # Control Mode
        self.control_mode = tk.BooleanVar(value=True)
        tk.Checkbutton(self._master, text="Closed Loop", variable=self.control_mode, command=lambda: self.pending_commands.append('set_control_mode')).grid(row=2, column=0, columnspan=2)

        # Enable/Disable
        self.driver_enabled = tk.BooleanVar(value=False)
        tk.Checkbutton(self._master, text="Enable Driver", variable=self.driver_enabled, command=lambda: self.pending_commands.append('set_driver_state')).grid(row=3, column=0, columnspan=2)

        # Clear Encoders
        tk.Button(self._master, text="Clear Encoders", command=lambda: self.pending_commands.append('clear_encoders')).grid(row=4, column=0, columnspan=2)

        # Zero-out speeds
        def zero_speeds():
            self.left_speed.set(0)
            self.right_speed.set(0)
        tk.Button(self._master, text="Zero Speeds", command=zero_speeds).grid(row=4, column=1, columnspan=2)

        # Status Display
        self.status_label = tk.Label(self._master, text="Status: Ready")
        self.status_label.grid(row=5, column=0, columnspan=2)

        # Plotting
        self.fig, self.ax = plt.subplots(3, 1, figsize=(6, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self._master)
        self.canvas.get_tk_widget().grid(row=6, column=0, columnspan=2)
        self.left_encoder_data = []
        self.right_encoder_data = []
        self.left_speed_data = []
        self.right_speed_data = []
        self.time_data = []
        self.left_pwm_data = []
        self.right_pwm_data = []
        self.max_data_points = 100

        self.running = True

        # Exit Button
        tk.Button(self._master, text="Exit", command=self.exit_program).grid(row=7, column=0, columnspan=2)

        # PID Gains
        self.kp = tk.DoubleVar(value=0)
        self.ki = tk.DoubleVar(value=0)
        self.kd = tk.DoubleVar(value=0)

        self.create_entry("Kp", self.kp, row=8)
        self.create_entry("Ki", self.ki, row=9)
        self.create_entry("Kd", self.kd, row=10)
        tk.Button(self._master, text="Set PID Gains", command=lambda: self.pending_commands.append('set_pid_gains')).grid(row=11, column=0, columnspan=2)

        # PID Gains Display
        self.kp_display = tk.Label(self._master, text="Kp: 0.0")
        self.kp_display.grid(row=12, column=0, columnspan=2)
        self.ki_display = tk.Label(self._master, text="Ki: 0.0")
        self.ki_display.grid(row=13, column=0, columnspan=2)
        self.kd_display = tk.Label(self._master, text="Kd: 0.0")
        self.kd_display.grid(row=14, column=0, columnspan=2)

        # Handle window close event
        self._master.protocol("WM_DELETE_WINDOW", self.exit_program)

        # Start the periodic update in a separate thread
        self.update_thread = threading.Thread(target=self.periodic_update)
        self.update_thread.daemon = True
        self.update_thread.start()

        # Start the Tkinter main loop
        self._master.mainloop()

    def create_scale(self, label, variable, min_val, max_val, row):
        tk.Label(self._master, text=label).grid(row=row, column=0)
        scale = tk.Scale(self._master, from_=min_val, to=max_val, orient=tk.HORIZONTAL, variable=variable)
        scale.grid(row=row, column=1)

    def create_entry(self, label, variable, row, on_focus=None, on_focus_out=None):
        tk.Label(self._master, text=label).grid(row=row, column=0)
        entry = tk.Entry(self._master, textvariable=variable)
        entry.grid(row=row, column=1)
        if on_focus:
            entry.bind("<FocusIn>", on_focus)
        if on_focus_out:
            entry.bind("<FocusOut>", on_focus_out)

    def set_control_mode(self):
        if self.control_mode.get():
            success = self.controller.set_closed_loop_mode()
        else:
            success = self.controller.set_open_loop_mode()
        self.check_status(success, "Set Control Mode")

    def set_driver_state(self):
        if self.driver_enabled.get():
            success = self.controller.enable_driver()
        else:
            success = self.controller.disable_driver()
        self.check_status(success, "Set Driver State")

    def clear_encoders(self):
        success = self.controller.clear_encoder_counts()
        self.check_status(success, "Clear Encoders")

    def set_pid_gains(self):
        kp = self.kp.get()
        ki = self.ki.get()
        kd = self.kd.get()
        success = self.controller.set_pid_gains(kp, ki, kd)
        self.check_status(success, "Set PID Gains")

    def exit_program(self):
        self.running = False
        self._master.quit()
        self._master.destroy()

    def periodic_update(self):
        while self.running:
            for command in self.pending_commands:
                if command == 'clear_encoders':
                    self.clear_encoders()
                elif command == 'set_driver_state':
                    self.set_driver_state()
                elif command == 'set_control_mode':
                    self.set_control_mode()
                elif command == 'set_pid_gains':
                    self.set_pid_gains()

            self.pending_commands = []

            self.time_data.append(time.time())

            left_speed = self.left_speed.get()
            right_speed = self.right_speed.get()
            success = self.controller.set_target_speeds(left_speed, right_speed)
            if success:
                self.left_target_speed_data.append(left_speed)
                self.right_target_speed_data.append(right_speed)
            else:
                self.left_target_speed_data.append(np.nan)
                self.right_target_speed_data.append(np.nan)
            self.check_status(success, "Set Target Speeds")

            success, speeds = self.controller.get_motor_speeds()
            if success:
                self.left_speed_data.append(speeds[0])
                self.right_speed_data.append(speeds[1])
            else:
                self.left_speed_data.append(np.nan)
                self.right_speed_data.append(np.nan)
            self.check_status(success, "Get Motor Speeds")

            success, counts = self.controller.get_encoder_counts()
            if success:
                self.left_encoder_data.append(counts[0])
                self.right_encoder_data.append(counts[1])
            else:
                self.left_encoder_data.append(np.nan)
                self.right_encoder_data.append(np.nan)
            self.check_status(success, "Get Encoder Counts")

            success, is_faulted = self.controller.read_fault()
            if success:
                if is_faulted[0]:
                    self.status_label.config(text="Status: Faulted!", fg="red")
                else:
                    self.status_label.config(text="Status: Running", fg="black")
            self.check_status(success, "Read Fault")

            success, pwm_values = self.controller.get_pwm_values()
            if success:
                self.left_pwm_data.append(pwm_values[0])
                self.right_pwm_data.append(pwm_values[1])
            else:
                self.left_pwm_data.append(np.nan)
                self.right_pwm_data.append(np.nan)
            self.check_status(success, "Get PWM Values")
            
            success, pid_gains = self.controller.get_pid_gains()
            if success:
                self.kp_display.config(text=f"Kp: {pid_gains[0]:.4f}")
                self.ki_display.config(text=f"Ki: {pid_gains[1]:.4f}")
                self.kd_display.config(text=f"Kd: {pid_gains[2]:.4f}")
            self.check_status(success, "Get PID Gains")

            self.update_plot()
            time.sleep(UPDATE_TIME_S)

    def check_status(self, success, operation):
        if not success:
            self.status_label.config(text=f"Status: {operation} Failed!", fg="red")

    def update_plot(self):
        if len(self.time_data) > self.max_data_points:
            self.time_data.pop(0)
            self.left_encoder_data.pop(0)
            self.right_encoder_data.pop(0)
            self.left_speed_data.pop(0)
            self.right_speed_data.pop(0)
            self.left_target_speed_data.pop(0)
            self.right_target_speed_data.pop(0)
            self.left_pwm_data.pop(0)
            self.right_pwm_data.pop(0)

        self.ax[0].clear()
        self.ax[0].plot(self.time_data, self.left_encoder_data, label="Left Encoder")
        self.ax[0].plot(self.time_data, self.right_encoder_data, label="Right Encoder")
        self.ax[0].set_xlabel("Time")
        self.ax[0].set_ylabel("Encoder Count")
        self.ax[0].legend()
        self.ax[0].grid()

        clean_left_encoders = [val for val in self.left_encoder_data if not np.isnan(val)]
        clean_right_encoders = [val for val in self.right_encoder_data if not np.isnan(val)]
        all_encoders = clean_left_encoders + clean_right_encoders
        if len(all_encoders) == 0:
            min_encoder = 0
            max_encoder = 0
        else:
            min_encoder = min(all_encoders)
            max_encoder = max(all_encoders)
        self.ax[0].set_ylim(min_encoder - 10, max_encoder + 10)

        self.ax[1].clear()
        self.ax[1].plot(self.time_data, self.left_speed_data, label="Left Speed (Measured)", color='blue')
        self.ax[1].plot(self.time_data, self.right_speed_data, label="Right Speed (Measured)", color='green')
        self.ax[1].plot(self.time_data, self.left_target_speed_data, label="Left Speed (Target)", linestyle='--', color='blue')
        self.ax[1].plot(self.time_data, self.right_target_speed_data, label="Right Speed (Target)", linestyle='--', color='green')
        self.ax[1].set_xlabel("Time")
        self.ax[1].set_ylabel("Speed")
        self.ax[1].legend()
        self.ax[1].grid()

        clean_left_speed_data = [x for x in self.left_speed_data if not np.isnan(x)]
        clean_right_speed_data = [x for x in self.right_speed_data if not np.isnan(x)]
        clean_left_target_speed_data = [x for x in self.left_target_speed_data if not np.isnan(x)]
        clean_right_target_speed_data = [x for x in self.right_target_speed_data if not np.isnan(x)]
        all_speeds = (clean_left_speed_data + clean_right_speed_data +
                  clean_left_target_speed_data + clean_right_target_speed_data)
        if len(all_speeds) == 0:
            min_speed = 0
            max_speed = 0
        else:
            min_speed = min(all_speeds)
            max_speed = max(all_speeds)
        self.ax[1].set_ylim(min_speed - 10, max_speed + 10)

        self.ax[2].clear()
        self.ax[2].plot(self.time_data, self.left_pwm_data, label="Left PWM")
        self.ax[2].plot(self.time_data, self.right_pwm_data, label="Right PWM")
        self.ax[2].set_xlabel("Time")
        self.ax[2].set_ylabel("PWM Value")
        self.ax[2].legend()
        self.ax[2].grid()

        clean_left_pwm = [x for x in self.left_pwm_data if not np.isnan(x)]
        clean_right_pwm = [x for x in self.right_pwm_data if not np.isnan(x)]
        clean_pwm = clean_left_pwm + clean_right_pwm
        if len(clean_pwm) == 0:
            min_pwm = 0
            max_pwm = 0
        else:
            min_pwm = min(clean_pwm)
            max_pwm = max(clean_pwm)
        self.ax[2].set_ylim(min_pwm - 10, max_pwm + 10)

        self.canvas.draw()