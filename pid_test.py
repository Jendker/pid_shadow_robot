from mujoco_py import load_model_from_path, MjSim, MjViewer, cymj
import numpy as np


def mj_viewer_setup(sim):
    viewer = MjViewer(sim)
    viewer.cam.azimuth = 0
    sim.forward()
    viewer.cam.distance = 1
    viewer._run_speed = 0.25
    return viewer


def mj_render(viewer):
    viewer.render()


# model = load_model_from_path("DAPG_relocate_position_control.xml")
model = load_model_from_path("DAPG_relocate_P_control.xml")
# model = load_model_from_path("DAPG_relocate_PID_control.xml")
sim = MjSim(model)
cymj.set_pid_control(sim.model, sim.data)
viewer = mj_viewer_setup(sim)

sim.reset()
sim.data.ctrl[:] = 0
sim.data.ctrl[9] = 1.6
sim.data.ctrl[10] = 1.6
sim.data.ctrl[11] = 1.6
for t in range(200):  # 200 is maximal episode steps count
    sim.step()
    print("qpos distance:", sim.data.ctrl[9:12] - sim.data.qpos[9:12])
    mj_render(viewer)
