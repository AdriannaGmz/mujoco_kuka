import numpy as np


# simple PD joint controller
# control_law = gravitational_compensation + angular_error * kp - angular_velocity * kd
def joint_angle_control(data,ref_pose,kp,kd):
	return data.qfrc_bias + kp * (ref_pose - data.qpos) - kd * data.qvel
