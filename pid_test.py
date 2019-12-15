from mujoco_py import load_model_from_path, MjSim, MjViewer, cymj
import numpy as np
import matplotlib.pyplot as plt


def mj_viewer_setup(sim):
    viewer = MjViewer(sim)
    viewer.cam.azimuth = 0
    sim.forward()
    viewer.cam.distance = 1
    viewer._run_speed = 0.25
    return viewer


def mj_render(viewer):
    viewer.render()


visualize = False
# ---------------- Model to select ----------------
# model = load_model_from_path("position_control/env_position_control.xml")
# model = load_model_from_path("P_control/env_P_control.xml")
model = load_model_from_path("PID_control/env_PID_control.xml")
# ---------------- Program ----------------
sim = MjSim(model)
cymj.set_pid_control(sim.model, sim.data)
if visualize:
    viewer = mj_viewer_setup(sim)
dt = sim.model.opt.timestep

sim.reset()
sim.data.ctrl[:] = 0
points = []
reference = []
for t in range(800):  # 200 is maximal episode steps count
    if t > 50:
        sim.data.ctrl[9] = 0.3
    if t > 250:
        sim.data.ctrl[9] = 0.6
    if t > 500:
        sim.data.ctrl[9] = 0.3
    sim.step()
    points.append(sim.data.qpos[9])
    reference.append(sim.data.ctrl[9])
    if visualize:
        mj_render(viewer)

fig, ax = plt.subplots()
x = np.array(range(1, len(points) + 1)) * dt
ax.plot(x, points, label="output")
ax.plot(x, reference, label="reference")

ax.set(xlabel='time [s]', ylabel='joint position [rad]')
ax.grid()
ax.legend()

plt.show()

