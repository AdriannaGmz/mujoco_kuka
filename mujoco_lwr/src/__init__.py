import numpy as np


# simple PD joint controller
# control_law = gravitational_compensation + angular_error * kp - angular_velocity * kd
# TODO: possibly improve readability (make code longer :<)
def joint_angle_control(data,ref_pose,kd,kp):
	err = np.subtract(ref_pose,data.qpos)
	return np.add(data.qfrc_bias,np.subtract( np.multiply(err,kp),np.multiply(data.qvel,kd) ) )
