from RealTimePlotter import RealTimePlotter
from DualMotorCommander import DualMotorCommander
import time

PLOT_UPDATE_TIME_S = 0.05
LABEL_UPDATE_TIME_S = 1.
PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

continue_flag = True

def exit_demo(event):
    global continue_flag
    continue_flag = False

def prepare_plot():
    rtp = RealTimePlotter.RealTimePlotter(num_rows=2, num_cols=1, fig_size=(12,8), display_len=200)
    rtp.set_time_step_s(PLOT_UPDATE_TIME_S)
    rtp.add_curve('Left encoder')
    rtp.add_curve('Right encoder')
    rtp.add_curve('Left velocity')
    rtp.add_curve('Right velocity')

    rtp.add_curve_to_plot('Left encoder', 0, 0)
    rtp.add_curve_to_plot('Right encoder', 0, 0)
    rtp.add_curve_to_plot('Left velocity', 1, 0)
    rtp.add_curve_to_plot('Right velocity', 1, 0)
    
    rtp.set_axis_ylabel(0,0,'Encoder counts')
    rtp.set_axis_ylabel(1,0,'Velocity (cps)')
    
    rtp.set_axis_ylimit(0,0,[-10000, 10000])
    rtp.set_axis_ylimit(1,0,[-10000, 10000])

    rtp.add_button("Exit", exit_demo)
    rtp.add_button("Set speed", lambda event: commander.set_target_speeds(1000, 1000))
    rtp.add_button("Zero speed", lambda event: commander.set_target_speeds(0, 0))
    rtp.add_label(width=7, caption='My label')
    
    return rtp


def update_plot(commander: DualMotorCommander, rtp: RealTimePlotter):
    d = {}
    
    result = commander.get_encoder_counts()
    if result[0]:
        left_count = result[1][0]
        right_count = result[1][1]
    else:
        left_count = 0
        right_count = 0

    result = commander.get_motor_speeds()
    if result[0]:
        left_speed = result[1][0]
        right_speed = result[1][1]
    else:
        left_speed = 0
        right_speed = 0

    d['Left encoder'] = left_count
    d['Right encoder'] = right_count
    d['Left velocity'] = left_speed
    d['Right velocity'] = right_speed

    rtp.update(d)


if __name__ == '__main__':
    commander = DualMotorCommander(PORT, BAUD_RATE)

    rtp = prepare_plot()
    rtp.start_animation()

    next_label_update_time_s = time.time() + LABEL_UPDATE_TIME_S

    while True:
        if not continue_flag:
            break

        if time.time() > next_label_update_time_s:
            result, fault = commander.read_fault()
            rtp.set_label_text(f'{time.time():.2f} Fault status: {fault[0]}')
            next_label_update_time_s = time.time() + LABEL_UPDATE_TIME_S

        update_plot(commander, rtp)
        time.sleep(PLOT_UPDATE_TIME_S)