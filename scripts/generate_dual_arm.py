"""从单臂 PiPER 生成双臂 MJCF (piper_dual.xml) 与双臂 URDF (piper_dual_description.urdf)。

双臂关节/链接名加 right_ / left_ 前缀，使 MujocoTeleopController 能按名字区分两臂。
两臂并排安装在桌面上（均朝 +x），分别位于 y = +0.18 / -0.18，z = 0.72。
"""

import os
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "assets", "piper")

RIGHT_OFFSET = "0 0.18 0.72"   # 右臂基座世界位置
LEFT_OFFSET = "0 -0.18 0.72"   # 左臂基座世界位置


def _prefix_names(elem, prefix, attrs=("name",), tag_is_body_or_joint=False):
    """递归给 body/joint 的 name 加前缀。"""
    tag = elem.tag
    if tag in ("body", "joint", "actuator", "geom", "light", "site", "camera"):
        # only body/joint need name prefixing for kinematics
        pass
    if tag in ("body", "joint") and elem.get("name"):
        elem.set("name", prefix + elem.get("name"))
    # equality joint1/joint2 refs, contact body refs, actuator joint refs handled separately
    for child in elem:
        _prefix_names(child, prefix)


def build_dual_mjcf():
    src = os.path.join(ASSETS, "piper.xml")
    tree = ET.parse(src)
    root = tree.getroot()

    asset_el = root.find("asset")
    default_el = root.find("default")
    worldbody = root.find("worldbody")
    contact_el = root.find("contact")
    equality_el = root.find("equality")
    actuator_el = root.find("actuator")
    keyframe_el = root.find("keyframe")

    # 找到 base_link body 子树（单臂的运动学根）
    base_body = None
    for b in worldbody.findall("body"):
        if b.get("name") == "base_link":
            base_body = b
            break
    assert base_body is not None

    new_root = ET.Element("mujoco", {"model": "piper_dual_description"})
    # 复制 compiler（含 meshdir）与 option（积分器等），保证网格与物理一致
    compiler_el = root.find("compiler")
    option_el = root.find("option")
    if compiler_el is not None:
        new_root.append(ET.fromstring(ET.tostring(compiler_el)))
    if option_el is not None:
        new_root.append(ET.fromstring(ET.tostring(option_el)))
    if default_el is not None:
        new_root.append(ET.fromstring(ET.tostring(default_el)))
    if asset_el is not None:
        new_root.append(ET.fromstring(ET.tostring(asset_el)))

    new_world = ET.SubElement(new_root, "worldbody")
    # 右臂
    right_base = ET.fromstring(ET.tostring(base_body))
    _prefix_names(right_base, "right_")
    right_base.set("pos", RIGHT_OFFSET)
    new_world.append(right_base)
    # 左臂
    left_base = ET.fromstring(ET.tostring(base_body))
    _prefix_names(left_base, "left_")
    left_base.set("pos", LEFT_OFFSET)
    new_world.append(left_base)

    # contact：仅添加两臂各自的 base_link/link1 自碰撞排除（原 exclude 引用未前缀名，不可复用）
    new_contact = ET.SubElement(new_root, "contact")
    for prefix in ("right_", "left_"):
        ex = ET.SubElement(new_contact, "exclude")
        ex.set("body1", prefix + "base_link")
        ex.set("body2", prefix + "link1")

    # equality
    new_eq = ET.SubElement(new_root, "equality")
    for prefix in ("right_", "left_"):
        j = ET.SubElement(new_eq, "joint")
        j.set("joint1", prefix + "joint8")
        j.set("joint2", prefix + "joint7")
        j.set("polycoef", "0 -1 0 0 0")

    # actuator
    new_act = ET.SubElement(new_root, "actuator")
    for prefix in ("right_", "left_"):
        for i in range(1, 7):
            a = ET.SubElement(new_act, "position")
            a.set("name", prefix + "joint" + str(i))
            a.set("joint", prefix + "joint" + str(i))
            a.set("class", "piper")
            a.set("kp", "80" if i <= 3 else ("40" if i == 4 else "10"))
            a.set("kv", "5" if i <= 4 else "1.5")
        g = ET.SubElement(new_act, "position")
        g.set("name", prefix + "gripper")
        g.set("joint", prefix + "joint7")
        g.set("class", "finger")
        g.set("kp", "40")
        g.set("kv", "5")

    # keyframe 不在此生成：含物体的完整 home keyframe 由 scene_dual.xml 提供，
    # 以避免与场景层物体初始位姿冲突。

    ET.indent(new_root, space="  ")
    out = os.path.join(ASSETS, "piper_dual.xml")
    ET.ElementTree(new_root).write(out, encoding="utf-8", xml_declaration=True)
    print("wrote", out)


def build_dual_urdf():
    src = os.path.join(ASSETS, "piper_description.urdf")
    tree = ET.parse(src)
    root = tree.getroot()

    links = {l.get("name"): l for l in root.findall("link")}
    joints = list(root.findall("joint"))

    # 单臂链中除 dummy_link 外的链接与除 base_to_dummy 外的关节
    arm_link_names = [n for n in links if n != "dummy_link"]
    arm_joints = [j for j in joints if j.get("name") != "base_to_dummy"]

    new_root = ET.Element("robot", {"name": "piper_dual"})

    # 公共根
    base = ET.SubElement(new_root, "link", {"name": "base_link"})

    def add_arm(prefix, offset):
        # 固定关节连接公共根到该臂基座
        jfix = ET.SubElement(new_root, "joint", {"name": f"base_to_{prefix}base", "type": "fixed"})
        ET.SubElement(jfix, "origin", {"xyz": offset, "rpy": "0 0 0"})
        ET.SubElement(jfix, "parent", {"link": "base_link"})
        ET.SubElement(jfix, "child", {"link": prefix + "base_link"})

        # 克隆该臂的所有链接（改名）
        for name in arm_link_names:
            lk = ET.fromstring(ET.tostring(links[name]))
            lk.set("name", prefix + name)
            new_root.append(lk)
        # 克隆该臂的所有关节（改名 + 父子链接加前缀）
        for j in arm_joints:
            nj = ET.fromstring(ET.tostring(j))
            nj.set("name", prefix + j.get("name"))
            p = nj.find("parent"); p.set("link", prefix + p.get("link"))
            c = nj.find("child"); c.set("link", prefix + c.get("link"))
            new_root.append(nj)

    add_arm("right_", RIGHT_OFFSET.replace(" ", " "))
    add_arm("left_", LEFT_OFFSET.replace(" ", " "))

    ET.indent(new_root, space="  ")
    out = os.path.join(ASSETS, "piper_dual_description.urdf")
    ET.ElementTree(new_root).write(out, encoding="utf-8", xml_declaration=True)
    print("wrote", out)


if __name__ == "__main__":
    build_dual_mjcf()
    build_dual_urdf()
