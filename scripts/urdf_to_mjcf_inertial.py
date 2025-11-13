#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
urdf_to_mjcf_inertial_fullinertia.py

Convert URDF <inertial> entries into MJCF <inertial> snippets using fullinertia.
 - pos = origin xyz (or provided CoM override)
 - mass = mass
 - fullinertia = the 6 independent components of the 3x3 inertia matrix about `pos` (MuJoCo order: M11 M22 M33 M12 M13 M23)

Usage:
    python urdf_to_mjcf_inertial_fullinertia.py input.urdf -o out.xml
    python urdf_to_mjcf_inertial_fullinertia.py input.urdf -o out.xml --com-override link1 0.01 0 0

Notes:
 - If URDF origin is not the CoM, use --com-override per link to provide actual CoM in link frame.
 - Script depends on numpy (pip install numpy).
"""

import argparse
import xml.etree.ElementTree as ET
import numpy as np
from typing import Optional, Dict, List


def parse_floats(text: Optional[str]) -> List[float]:
    if not text:
        return []
    return [float(x) for x in text.strip().split()]


def rpy_to_rot_matrix(rpy):
    """Construct rotation matrix from rpy (roll, pitch, yaw) in radians.
    URDF uses roll (x), pitch (y), yaw (z) intrinsic; building R = Rz * Ry * Rx.
    """
    roll, pitch, yaw = rpy
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx


def inertia_matrix_from_components(ixx, iyy, izz, ixy, ixz, iyz):
    return np.array([[ixx, ixy, ixz],
                     [ixy, iyy, iyz],
                     [ixz, iyz, izz]], dtype=float)


def parallel_axis_to_com(I_about_O: np.ndarray, mass: float, d_C_to_O: np.ndarray):
    """Given inertia about O, return inertia about Com (C).
    I_C = I_O - m( ||d||^2 I - d d^T )
    Here d_C_to_O is vector from C -> O.
    """
    d = np.asarray(d_C_to_O, dtype=float)
    return I_about_O - mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def ensure_symmetric(A: np.ndarray) -> np.ndarray:
    return 0.5 * (A + A.T)


def process_inertial_elem(inertial_elem: ET.Element, com_override: Optional[List[float]] = None):
    # parse origin
    origin_elem = inertial_elem.find('origin')
    if origin_elem is not None:
        xyz = parse_floats(origin_elem.get('xyz', '0 0 0'))
        rpy = parse_floats(origin_elem.get('rpy', '0 0 0'))
    else:
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]

    mass_elem = inertial_elem.find('mass')
    if mass_elem is None:
        raise ValueError("Missing <mass> in <inertial>")
    mass = float(mass_elem.get('value'))

    inertia_elem = inertial_elem.find('inertia')
    if inertia_elem is None:
        raise ValueError("Missing <inertia> in <inertial>")

    ixx = float(inertia_elem.get('ixx', '0'))
    ixy = float(inertia_elem.get('ixy', '0'))
    ixz = float(inertia_elem.get('ixz', '0'))
    iyy = float(inertia_elem.get('iyy', '0'))
    iyz = float(inertia_elem.get('iyz', '0'))
    izz = float(inertia_elem.get('izz', '0'))

    # inertia matrix expressed in the 'inertial frame' described by origin.rpy
    I_inertial_frame = inertia_matrix_from_components(ixx, iyy, izz, ixy, ixz, iyz)

    # rotation from inertial_frame -> link/body frame (origin.rpy gives orientation of inertial frame relative to link)
    R_inertial_to_link = rpy_to_rot_matrix(rpy)
    # express inertia in link frame about the same point (origin)
    I_link_frame = R_inertial_to_link @ I_inertial_frame @ R_inertial_to_link.T

    origin_pos = np.array(xyz, dtype=float)

    # If user provided CoM override, convert inertia to CoM
    if com_override is not None:
        com = np.array(com_override, dtype=float)
        # vector from C -> O
        d = origin_pos - com
        I_target = parallel_axis_to_com(I_link_frame, mass, d)
        pos_for_mjcf = f"{com[0]:.8g} {com[1]:.8g} {com[2]:.8g}"
    else:
        I_target = I_link_frame
        pos_for_mjcf = f"{origin_pos[0]:.8g} {origin_pos[1]:.8g} {origin_pos[2]:.8g}"

    # Symmetrize numeric noise
    I_target = ensure_symmetric(I_target)

    # Compose fullinertia in MuJoCo expected order: M11 M22 M33 M12 M13 M23
    M11 = I_target[0, 0]
    M22 = I_target[1, 1]
    M33 = I_target[2, 2]
    M12 = I_target[0, 1]
    M13 = I_target[0, 2]
    M23 = I_target[1, 2]

    fullinertia_list = [M11, M22, M33, M12, M13, M23]

    return {
        'pos': pos_for_mjcf,
        'mass': mass,
        'fullinertia': fullinertia_list,
        'I_inertial_frame': I_inertial_frame,
        'I_target': I_target
    }


def convert_urdf_string(urdf_text: str, com_overrides: Optional[Dict[str, List[float]]] = None):
    root = ET.fromstring(urdf_text)
    results = []
    for link in root.findall('.//link'):
        name = link.get('name', '<unnamed>')
        inertial = link.find('inertial')
        if inertial is None:
            continue
        override = None
        if com_overrides and name in com_overrides:
            override = com_overrides[name]
        info = process_inertial_elem(inertial, com_override=override)
        info['link_name'] = name
        results.append(info)
    return results


def format_mjcf_block_fullinertia(info):
    full = ' '.join(f"{f:.8g}" for f in info['fullinertia'])
    return f'<inertial pos="{info["pos"]}" mass="{info["mass"]:.8g}" fullinertia="{full}"/>'


def main():
    parser = argparse.ArgumentParser(description="Convert URDF <inertial> to MJCF <inertial fullinertia> snippets.")
    parser.add_argument('urdf', help='Path to URDF file (XML)')
    parser.add_argument('-o', '--output', help='Output file path for MJCF inertial blocks (optional)')
    parser.add_argument('--com-override', nargs=4, action='append',
                        metavar=('LINK', 'X', 'Y', 'Z'),
                        help='Provide CoM override for a link (link_name x y z). Can be used multiple times.')
    args = parser.parse_args()

    with open(args.urdf, 'r', encoding='utf-8') as f:
        urdf_text = f.read()

    com_overrides = {}
    if args.com_override:
        for entry in args.com_override:
            link = entry[0]
            coords = [float(entry[1]), float(entry[2]), float(entry[3])]
            com_overrides[link] = coords

    results = convert_urdf_string(urdf_text, com_overrides=com_overrides)

    out_lines = []
    for r in results:
        out_lines.append(f'<!-- link: {r["link_name"]} -->')
        out_lines.append(format_mjcf_block_fullinertia(r))
        out_lines.append('')

    output_text = '\n'.join(out_lines)
    print(output_text)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(output_text)
        print(f"\nWrote MJCF inertial blocks to: {args.output}")


if __name__ == '__main__':
    main()
