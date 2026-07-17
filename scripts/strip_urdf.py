"""Strip <visual> and <collision> elements from a URDF so placo/pinocchio can
load it for inverse kinematics without resolving `package://` mesh paths.

Usage:
    python scripts/strip_urdf.py <input.urdf> <output.urdf>
"""

import sys
import xml.etree.ElementTree as ET


def strip_geometry(input_path: str, output_path: str) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()

    removed = 0
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for elem in list(link.findall(tag)):
                link.remove(elem)
                removed += 1

    # Pretty-print with a declaration line.
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Stripped {removed} visual/collision elements -> {output_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python scripts/strip_urdf.py <input.urdf> <output.urdf>")
        sys.exit(1)
    strip_geometry(sys.argv[1], sys.argv[2])
