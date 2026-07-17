"""离屏渲染 PiPER 仿真场景为 PNG，便于无头查看。

支持单臂（scene.xml）与双臂（scene_dual.xml，含桌子与抓取物）。
"""

import os

import mujoco
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "assets", "piper")
OUT = os.path.join(ROOT, "docs")
os.makedirs(OUT, exist_ok=True)


def render_scene(xml, azimuth, elevation, distance, lookat, fname):
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array(lookat, dtype=float)
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    renderer.update_scene(data, camera=cam)
    img = renderer.render()
    path = os.path.join(OUT, fname)
    Image.fromarray(img).save(path)
    print("saved", path, "mean=", float(img.mean()))


if __name__ == "__main__":
    # 单臂
    render_scene(os.path.join(ASSETS, "scene.xml"), 120, -25, 1.2,
                 [0.2, 0, 0.35], "piper_home_front.png")
    # 双臂：斜前方（桌子 2m 沿 y × 1.2m 沿 x，相机框住整桌）
    render_scene(os.path.join(ASSETS, "scene_dual.xml"), 110, -22, 2.6,
                 [0.5, 0, 0.8], "piper_dual_front.png")
    # 双臂：俯视
    render_scene(os.path.join(ASSETS, "scene_dual.xml"), 110, -55, 3.2,
                 [0.5, 0, 0.7], "piper_dual_top.png")
    # 双臂：侧视
    render_scene(os.path.join(ASSETS, "scene_dual.xml"), 70, -18, 2.4,
                 [0.5, 0, 0.85], "piper_dual_side.png")
