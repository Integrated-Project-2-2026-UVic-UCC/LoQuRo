"""Rebuilds the right legs of xmls/loquro.xml as the mirror image of the left legs.

    uv run python loquro_lab/mirror_right_legs.py

Run it after changing a left leg. fr is generated from fl and rr from rl (bodies, joints, geoms and meshes mirrored
with respect to the vertical plane along the trunk). The actuated joints of the right legs keep the convention of the
real robot: the same pose of a left leg at q is commanded on the right leg as MIRROR - q.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

XML = Path(__file__).parent / "xmls" / "loquro.xml"
MIRROR = {"shoulder_joint": 1.4, "knee_joint": 1.6, "femur_joint": 1.6}  # right = MIRROR - left [rad]
S_TRUNK = np.diag([1.0, -1.0, 1.0])  # mirror plane of the robot (trunk frame)
S_BODY = np.diag([1.0, 1.0, -1.0])   # reflection used inside every mirrored body (keeps the frames right-handed)


def vec(text):
    return np.array([float(v) for v in text.split()])


def fmt(values):
    return " ".join(f"{v:.9g}" for v in np.where(np.abs(values) < 1e-12, 0.0, values))


def rot(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    m = np.eye(3)
    m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
    return m


def orientation(el):
    """Rotation matrix of an element (quat or euler, MuJoCo defaults: radians, intrinsic xyz)."""
    if "quat" in el.attrib:
        w, x, y, z = vec(el.attrib["quat"]) / np.linalg.norm(vec(el.attrib["quat"]))
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    e = vec(el.attrib.get("euler", "0 0 0"))
    return rot(0, e[0]) @ rot(1, e[1]) @ rot(2, e[2])


def quat(m):
    w = np.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2
    k = int(np.argmax([w, x, y, z]))  # signs from the largest component
    if k == 0:
        x, y, z = np.copysign(x, m[2, 1] - m[1, 2]), np.copysign(y, m[0, 2] - m[2, 0]), np.copysign(z, m[1, 0] - m[0, 1])
    elif k == 1:
        w, y, z = np.copysign(w, m[2, 1] - m[1, 2]), np.copysign(y, m[1, 0] + m[0, 1]), np.copysign(z, m[0, 2] + m[2, 0])
    elif k == 2:
        w, x, z = np.copysign(w, m[0, 2] - m[2, 0]), np.copysign(x, m[1, 0] + m[0, 1]), np.copysign(z, m[2, 1] + m[1, 2])
    else:
        w, x, y = np.copysign(w, m[1, 0] - m[0, 1]), np.copysign(x, m[0, 2] + m[2, 0]), np.copysign(y, m[2, 1] + m[1, 2])
    return np.array([w, x, y, z])


def set_pose(el, parent_reflection):
    """Mirrors pos and orientation of a body or geom defined inside a frame reflected with `parent_reflection`."""
    if "pos" in el.attrib:
        el.set("pos", fmt(parent_reflection @ vec(el.attrib["pos"])))
    mirrored = parent_reflection @ orientation(el) @ S_BODY
    el.attrib.pop("euler", None)
    el.attrib.pop("quat", None)
    if not np.allclose(mirrored, np.eye(3)):
        el.set("quat", fmt(quat(mirrored)))


def mirror_body(body, parent_reflection, left, right, meshes):
    body.set("name", body.attrib["name"].replace(left, right, 1))
    set_pose(body, parent_reflection)
    for el in body:
        if el.tag == "body":
            mirror_body(el, S_BODY, left, right, meshes)
        elif el.tag == "joint":
            name = el.attrib["name"]
            el.set("name", name.replace(left, right, 1))
            el.set("pos", fmt(S_BODY @ vec(el.attrib.get("pos", "0 0 0"))))
            axis = vec(el.attrib["axis"])
            constant = MIRROR.get(name[len(left):])
            if constant is None:   # passive joint: same angle as the left leg
                el.set("axis", fmt(-(S_BODY @ axis)))
            else:                  # servo: right = constant - left
                el.set("axis", fmt(S_BODY @ axis))
                el.set("ref", f"{constant:g}")
        elif el.tag == "geom":
            if "name" in el.attrib:
                el.set("name", el.attrib["name"].replace(left, right, 1))
            if el.attrib.get("type") == "mesh":
                meshes.add(el.attrib["mesh"])
                el.set("mesh", el.attrib["mesh"] + "_mirror")
            if "pos" in el.attrib or "euler" in el.attrib or "quat" in el.attrib:
                set_pose(el, S_BODY)


def body_span(text, name):
    start = text.index(f'<body name="{name}"')
    depth = 0
    for m in re.finditer(r"<body\b|</body>", text[start:]):
        depth += 1 if m.group() != "</body>" else -1
        if depth == 0:
            return start, start + m.end()
    raise ValueError(name)


def main():
    text = XML.read_text()
    meshes = set()
    for left, right in (("fl_", "fr_"), ("rl_", "rr_")):
        a, b = body_span(text, left + "mount")
        body = ET.fromstring(text[a:b])
        mirror_body(body, S_TRUNK, left, right, meshes)
        ET.indent(body, space="  ", level=3)
        a, b = body_span(text, right + "mount")
        text = text[:a] + ET.tostring(body, encoding="unicode") + text[b:]
        # closed chain: same anchors as the left leg (the reflection does not change them, they are in the z = 0 plane)
        for eq in re.findall(rf'<connect body1="{left}[^>]*>', text):
            anchor = re.search(r'anchor="[^"]*"', eq).group()
            body1 = re.search(r'body1="([^"]*)"', eq).group(1).replace(left, right, 1)
            text = re.sub(rf'(<connect body1="{body1}"[^>]*?)anchor="[^"]*"', rf"\1{anchor}", text)

    # mirrored meshes: the left ones with the z axis flipped
    start = text.index("<!-- Mallas Derechas")
    end = text.index("</asset>")
    lines = ["<!-- Mallas Derechas: las izquierdas reflejadas (mirror_right_legs.py) -->"]
    for name in sorted(meshes):
        file = re.search(rf'<mesh name="{name}" file="([^"]*)"', text).group(1)
        lines.append(f'<mesh name="{name}_mirror" file="{file}" scale="0.001 0.001 -0.001"/>')
    text = text[:start] + "\n    ".join(lines) + "\n  " + text[end:]
    XML.write_text(text)
    print(f"{XML.name}: right legs rebuilt from the left ones, {len(meshes)} mirrored meshes")


if __name__ == "__main__":
    main()
