#!/usr/bin/env python3
"""Diagnose URDF joint chains for missing connections."""
import xml.etree.ElementTree as ET
from pathlib import Path

urdf_path = Path(r"D:\Keilprojects\HAL_LIB\Wheel-Legged_Robot\for_simulation\for_simulation\for_simulation.urdf")
tree = ET.parse(urdf_path)
root = tree.getroot()

# Parse all joints
joints = {}
for joint in root.findall('joint'):
    name = joint.get('name')
    jtype = joint.get('type')
    origin = joint.find('origin')
    parent = joint.find('parent').get('link')
    child = joint.find('child').get('link')

    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        if origin.get('xyz'):
            xyz = [float(v) for v in origin.get('xyz').split()]
        if origin.get('rpy'):
            rpy = [float(v) for v in origin.get('rpy').split()]

    axis_elem = joint.find('axis')
    axis = [1.0, 0.0, 0.0]
    if axis_elem is not None:
        axis = [float(v) for v in axis_elem.get('xyz').split()]

    joints[name] = {
        'type': jtype, 'parent': parent, 'child': child,
        'xyz': xyz, 'rpy': rpy, 'axis': axis,
    }

# Build maps
all_links = set()
for link in root.findall('link'):
    all_links.add(link.get('name'))

parents_of = {}  # child -> [(parent, joint_name, joint_type)]
children_of = {}  # parent -> [(child, joint_name, joint_type)]

for jname, jdata in joints.items():
    child = jdata['child']
    parent = jdata['parent']
    if child not in parents_of:
        parents_of[child] = []
    parents_of[child].append((parent, jname, jdata['type']))

    if parent not in children_of:
        children_of[parent] = []
    children_of[parent].append((child, jname, jdata['type']))

# Find floating links (no parent, or multiple parents)
print("=" * 60)
print("FLOATING LINKS (no parent in any joint):")
print("=" * 60)
for link in sorted(all_links):
    if link not in parents_of:
        print(f"  {link}")

print()
print("=" * 60)
print("LINKS WITH MULTIPLE PARENTS (kinematic loops):")
print("=" * 60)
for link in sorted(all_links):
    if link in parents_of and len(parents_of[link]) > 1:
        types = [t for _, _, t in parents_of[link]]
        print(f"  {link}:")
        for parent, jname, jtype in parents_of[link]:
            print(f"    <- {parent} [{jname}] ({jtype})")

print()
print("=" * 60)
print("KINEMATIC JOINTS (continuous/revolute):")
print("=" * 60)
for jname, jdata in sorted(joints.items()):
    if jdata['type'] in ('continuous', 'revolute'):
        print(f"  {jname}: {jdata['parent']} -> {jdata['child']} (axis={jdata['axis']})")

# Trace each leg chain
print()
print("=" * 60)
print("LEG CHAIN TRACING:")
print("=" * 60)

def trace_chain(link_name, depth=0, visited=None):
    if visited is None:
        visited = set()
    if link_name in visited:
        return
    visited.add(link_name)

    prefix = "  " * depth
    print(f"{prefix}{link_name}")

    if link_name in children_of:
        for child, jname, jtype in children_of[link_name]:
            if jtype in ('continuous', 'revolute'):
                print(f"{prefix}  --[{jname}]({jtype})-->")
                trace_chain(child, depth + 2, visited)
            elif jtype == 'fixed':
                print(f"{prefix}  --[{jname}](fixed)--> (following...)")
                trace_chain(child, depth + 1, visited)

print("\n--- From body-v6 ---")
trace_chain('body-v6')

# Also trace from floating links
print("\n--- From floating lowerleg_a1-v2 ---")
trace_chain('lowerleg_a1-v2')

print("\n--- From floating lowerleg_a2 ---")
trace_chain('lowerleg_a2')

print("\n--- From floating M0601C_411_left ---")
trace_chain('M0601C_411_left')

print("\n--- From floating M0601C_411_right ---")
trace_chain('M0601C_411_right')
