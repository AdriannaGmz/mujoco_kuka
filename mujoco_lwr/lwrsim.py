from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import random
import numpy as np
from src.controller import joint_angle_control


#initialisation / loading the model
model = load_model_from_path("./xmls/kuka_mjcf.xml")
sim = MjSim(model)
viewer = MjViewer(sim)
viewer.cam.distance = model.stat.extent * 1.2
sim_state = sim.get_state()
t = 0
ref_pose = np.array([0,0,0,0,0,0,0])

# joint specific control parameters, kp = stiffness, kd = dampning
kp = np.array([50, 150, 30, 100, 10, 50, 10])
kd = np.array([15, 25, 10, 20, 5, 15, 2])


# helper function that returns a random pose
def random_pose(low,high,joints):
	return np.array([ random.uniform(low,high) for i in range(joints)])

#main simulation loop, run until exiting the viewer
while True:

	# change to random joint configuration every 2 sec
	if t%2000 == 0:
		ref_pose = random_pose(-2,2,sim.data.nf)
		t = 0

	# step and render
	sim.step()
	viewer.render()

	# quick maffs (calculate joint controls)
	sim.data.ctrl[:] = joint_angle_control(sim.data,ref_pose,kp,kd)

	t+=1


	if os.getenv('TESTING') is not None:
		break



