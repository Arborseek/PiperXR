"""Mock of `xrobotoolkit_sdk` for offline validation of the teleop pipeline.

This does NOT talk to a PICO headset. It returns static/zero data so the
MujocoTeleopController can be constructed and the IK + MuJoCo step loop can
run headless, proving the PiPER model + URDF + joint mapping are wired up
correctly. Replace it with the real SDK built from XRoboToolkit-PC-Service-Pybind
for actual teleoperation.
"""

import numpy as np


def init():
    print("[mock xrobotoolkit_sdk] init() called — no real headset connected.")


def _zero_pose():
    # [x, y, z, qx, qy, qz, qw]
    return np.zeros(7, dtype=float)


def get_left_controller_pose():
    return _zero_pose()


def get_right_controller_pose():
    return _zero_pose()


def get_headset_pose():
    return _zero_pose()


def get_left_trigger():
    return 0.0


def get_right_trigger():
    return 0.0


def get_left_grip():
    return 0.0


def get_right_grip():
    return 0.0


def get_A_button():
    return False


def get_B_button():
    return False


def get_X_button():
    return False


def get_Y_button():
    return False


def get_left_menu_button():
    return False


def get_right_menu_button():
    return False


def get_left_axis_click():
    return False


def get_right_axis_click():
    return False


def get_time_stamp_ns():
    return 0


def num_motion_data_available():
    return 0


def get_motion_tracker_pose():
    return []


def get_motion_tracker_velocity():
    return []


def get_motion_tracker_acceleration():
    return []


def get_motion_tracker_serial_numbers():
    return []


def is_body_data_available():
    return False


def get_left_axis():
    return np.zeros(2)


def get_right_axis():
    return np.zeros(2)


def close():
    pass


def get_motion_tracker_data():
    return {}


def get_hand_tracking_state(hand):
    return None
