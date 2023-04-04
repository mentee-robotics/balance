import yaml
import numpy as np

COEFF_FILE_LEFT = '/home/robot/code/ros2_workspace/menteebot/menteebot/state_manager/state_handlers/left_wrist_closed_kinematics_config.yaml'
COEFF_FILE_RIGHT = '/home/robot/code/ros2_workspace/menteebot/menteebot/state_manager/state_handlers/right_wrist_closed_kinematics_config.yaml'


def shift_pitch_yaw(pitch, yaw, coeff):
    yaw = yaw + coeff[2]['yaw_zero_encoder'][0]
    pitch = pitch + coeff[2]['pitch_zero_encoder'][0]
    return pitch, yaw


def shift_motors(mot1, mot2, new_zero_motors):
    mot1 = mot1 - new_zero_motors
    mot2 = mot2 + new_zero_motors
    return mot1, mot2


def get_motors_from_diff_avg(diff, avg):
    mot_pitch = 0.5 * (diff - 2 * avg)  # currently id 205
    mot_yaw = diff - mot_pitch  # currently id 204
    return mot_yaw, mot_pitch


def get_motors_from_pitch_yaw(pitch, yaw, side='left'):
    if side == 'left':
        coeff_file = COEFF_FILE_LEFT

    if side == 'right':
        coeff_file = COEFF_FILE_RIGHT

    # TODO: handel the not valid side case

    with open(coeff_file) as file:
        coeff = yaml.load(file, Loader=yaml.FullLoader)

    pitch, yaw = shift_pitch_yaw(pitch, yaw, coeff)
    yaw_coeff = np.array(coeff[0]['yaw'][0])
    pitch_coeff = np.array(coeff[1]['pitch'][0])
    yaw_diff_func = np.poly1d(yaw_coeff)
    pitch_avg_func = np.poly1d(pitch_coeff)

    diff = yaw_diff_func(yaw)  # = mot204+mot205
    avg = pitch_avg_func(pitch)  # = 0.5*(mot204-mot205)

    mot_yaw, mot_pitch = get_motors_from_diff_avg(diff, avg)  # values by mechanical zero

    return mot_yaw, mot_pitch
