from mujoco_py import load_model_from_path, MjSim, MjViewer, cymj


def mj_viewer_setup(sim):
    viewer = MjViewer(sim)
    viewer.cam.azimuth = 0
    sim.forward()
    viewer.cam.distance = 1
    viewer._run_speed = 0.25
    return viewer


def mj_render(viewer):
    viewer.render()


# ---------------- Model to select ----------------
model = load_model_from_path("position_control/env_position_control.xml")
# model = load_model_from_path("P_control/env_P_control.xml")
# model = load_model_from_path("PID_control/env_PID_control.xml")

# ---------------- Program ----------------
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
