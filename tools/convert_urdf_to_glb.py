#!/usr/bin/env python3
"""
Convert URDF + STL meshes to individual GLB files + joint hierarchy JSON
for the wheel-legged robot tuning UI.

Output:
  - esp32_app/data/3d_articulated/*.glb (one per unique link mesh)
  - esp32_app/data/3d_articulated/robot_hierarchy.json (joint tree + defaults)
"""

import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


def euler_to_matrix(roll, pitch, yaw):
    """Convert Euler angles (rpy) to 4x4 homogeneous transform matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0, 0],
                   [0, cr, -sr, 0],
                   [0, sr, cr, 0],
                   [0, 0, 0, 1]], dtype=float)

    Ry = np.array([[cp, 0, sp, 0],
                   [0, 1, 0, 0],
                   [-sp, 0, cp, 0],
                   [0, 0, 0, 1]], dtype=float)

    Rz = np.array([[cy, -sy, 0, 0],
                   [sy, cy, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]], dtype=float)

    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=float)

    return Rz @ Ry @ Rx @ T


def rpy_to_matrix(rpy):
    """Convert [roll, pitch, yaw] to rotation matrix."""
    return euler_to_matrix(rpy[0], rpy[1], rpy[2])


def xyz_rpy_to_matrix(xyz, rpy):
    """Convert [x,y,z] and [roll,pitch,yaw] to 4x4 transform matrix."""
    T = rpy_to_matrix(rpy)
    T[0, 3] = xyz[0]
    T[1, 3] = xyz[1]
    T[2, 3] = xyz[2]
    return T


def normalize_path(path_str):
    """Convert a URDF mesh path to absolute path."""
    # Handle Windows paths
    path_str = path_str.replace('\\', '/')
    return path_str


def main():
    # Paths
    urdf_path = Path(r"D:\Keilprojects\HAL_LIB\Wheel-Legged_Robot\for_simulation\for_simulation\for_simulation.urdf")
    meshes_dir = Path(r"D:\Keilprojects\HAL_LIB\Wheel-Legged_Robot\for_simulation\for_simulation\meshes")
    output_dir = Path(r"D:\Keilprojects\HAL_LIB\Wheel-Legged_Robot\esp32_app\data\3d_articulated")

    output_dir.mkdir(parents=True, exist_ok=True)

    # Parse URDF
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # ----- 1. Parse all links and meshes -----
    links = {}
    for link in root.findall('link'):
        name = link.get('name')
        visual = link.find('visual')
        mesh_info = None
        if visual is not None:
            origin = visual.find('origin')
            geom = visual.find('geometry')
            mesh = geom.find('mesh') if geom is not None else None
            if mesh is not None:
                filename = mesh.get('filename')
                scale_str = mesh.get('scale', '1 1 1')
                scale = [float(v) for v in scale_str.split()]
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]
                if origin is not None:
                    if origin.get('xyz'):
                        xyz = [float(v) for v in origin.get('xyz').split()]
                    if origin.get('rpy'):
                        rpy = [float(v) for v in origin.get('rpy').split()]
                mesh_info = {
                    'filename': filename,
                    'scale': scale,
                    'xyz': xyz,
                    'rpy': rpy,
                }
        links[name] = mesh_info

    print(f"Parsed {len(links)} links with mesh references")

    # ----- 2. Parse all joints and build tree -----
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

        limit_elem = joint.find('limit')
        limit = None
        if limit_elem is not None:
            lower = float(limit_elem.get('lower', '-inf'))
            upper = float(limit_elem.get('upper', 'inf'))
            limit = [lower, upper]

        joints[name] = {
            'type': jtype,
            'parent': parent,
            'child': child,
            'xyz': xyz,
            'rpy': rpy,
            'axis': axis,
            'limit': limit,
        }

    print(f"Parsed {len(joints)} joints")

    # ----- 3. Build hierarchy tree (multi-level) -----
    # Find root links (links that are not children of any joint that is not fixed)
    # Find all non-root child links
    all_children = set()
    for jname, jdata in joints.items():
        if jdata['type'] not in ('continuous', 'revolute'):
            continue
        all_children.add(jdata['child'])

    # Root links are links that are NOT children of any non-fixed joint
    # The real roots are el05_1/2/3/4 (they connect to body-v6 via fixed joints)
    # Build a proper parent-child mapping

    # First, find which links are direct children of body-v6 through fixed joints
    # Then traverse from those roots through the kinematic tree

    # Build full parent->children mapping including fixed joints
    parent_to_children = {}
    for jname, jdata in joints.items():
        parent = jdata['parent']
        child = jdata['child']
        if parent not in parent_to_children:
            parent_to_children[parent] = []
        parent_to_children[parent].append({
            'joint_name': jname,
            'joint_data': jdata,
            'child': child,
        })

    # Find the body link (the real root of the URDF for mobile base)
    # body-v6 is the main body, everything attaches to it
    # Find kinematic chains from body-v6 to the tips

    def build_kinematic_chain(node_name, parent_transform=np.eye(4)):
        """Recursively build kinematic chain description."""
        if node_name not in parent_to_children:
            # Leaf node - no children
            return None

        children_info = parent_to_children[node_name]
        result_children = []

        for child_info in children_info:
            jdata = child_info['joint_data']
            child_name = child_info['child']

            # Only follow non-fixed joints as kinematic joints
            # Fixed joints are mesh grouping (screws, gaskets, bearings attached to a link)
            # We skip them in the kinematic tree - they'll merge visually

            if jdata['type'] in ('continuous', 'revolute'):
                # This is a real joint
                child_chain = build_kinematic_chain(child_name)
                result_children.append({
                    'joint_name': child_info['joint_name'],
                    'joint_type': jdata['type'],
                    'child_link': child_name,
                    'xyz': jdata['xyz'],
                    'rpy': jdata['rpy'],
                    'axis': jdata['axis'],
                    'limit': jdata.get('limit'),
                    'children': child_chain['children'] if child_chain else [],
                    'fixed_attachments': child_chain['fixed_attachments'] if child_chain else [],
                })
            else:
                # Fixed joint - attachment link
                # We don't recurse into fixed joints for the kinematic chain
                pass

        # Collect fixed-attachment links (non-kinematic children)
        fixed_attachments = []
        for child_info in parent_to_children.get(node_name, []):
            jdata = child_info['joint_data']
            child_name = child_info['child']
            if jdata['type'] == 'fixed':
                # Check if this child itself has kinematic children
                # (some fixed joints connect a structural part to a kinematic joint)
                child_has_kinematic = False
                for gc in parent_to_children.get(child_name, []):
                    if gc['joint_data']['type'] in ('continuous', 'revolute'):
                        child_has_kinematic = True
                        break

                if not child_has_kinematic:
                    # Structural attachment only - add to parent's visuals
                    fixed_attachments.append({
                        'link_name': child_name,
                        'xyz': jdata['xyz'],
                        'rpy': jdata['rpy'],
                    })

                # If the fixed child itself has fixed children, add those too
                # (depth-2 search for nested fixed joints)
                for gc in parent_to_children.get(child_name, []):
                    if gc['joint_data']['type'] == 'fixed':
                        fixed_attachments.append({
                            'link_name': gc['child'],
                            'xyz': compose_transform(
                                jdata['xyz'], jdata['rpy'],
                                gc['joint_data']['xyz'], gc['joint_data']['rpy']
                            ),
                            'rpy': [0, 0, 0],  # simplified
                        })

        return {
            'link_name': node_name,
            'mesh_file': None,  # filled in later
            'children': result_children,
            'fixed_attachments': fixed_attachments,
        }

    def compose_transform(xyz1, rpy1, xyz2, rpy2):
        """Compose two transform xyz,rpy pairs."""
        return [xyz1[i] + xyz2[i] for i in range(3)]

    # Build explicit kinematic chains from el05 motors -> thigh -> knee -> lowerleg -> wheel
    # The kinematic chains are:
    # body-v6 -(fixed)-> el05_1 -(revolute)-> thigh_1 -(fixed joints)-> gasket2/3 -(continuous)-> lowerleg_b1 -(fixed)-> bearings, wheels
    # etc.

    # Actually, the simplest approach: explicitly define the 4 leg kinematic chains
    # by tracing through the URDF structure

    # Chain 1: body-v6 -> el05_1 -> thigh_1 -> gasket2_12_24_1.2 -> lowerleg_b1 -> SKF-bearing/wheel mounted on lowerleg_b1
    # Chain 2: body-v6 -> el05_2 -> thigh_2 -> gasket5/8 -> lowerleg_a1-v2 -> wheel/m0601c
    # Chain 3: body-v6 -> el05_3 -> thigh_3 -> gasket8/11 -> lowerleg_a2 -> wheel/m0601c
    # Chain 4: body-v6 -> el05_4 -> thigh_4 -> gasket10/11 -> lowerleg_b2 -> wheel

    # Let me just build the tree manually by tracing URDF joints

    # Step 1: For each kinematic joint, find its parent chain up to body-v6
    kinematic_joints = {n: d for n, d in joints.items()
                        if d['type'] in ('continuous', 'revolute')}

    print(f"Kinematic joints: {list(kinematic_joints.keys())}")

    # Build a graph of parent->child for ALL joints
    parent_map = {}  # child_link -> (parent_link, joint_data)
    child_map = {}   # parent_link -> [(child_link, joint_data)]
    for jname, jdata in joints.items():
        parent_map[jdata['child']] = (jdata['parent'], jdata)
        if jdata['parent'] not in child_map:
            child_map[jdata['parent']] = []
        child_map[jdata['parent']].append((jdata['child'], jdata))

    # Build kinematic chains for each unique leg root (el05 motors)
    leg_roots = ['el05_1', 'el05_2', 'el05_3', 'el05_4']

    def build_leg_chain(root_link):
        """Build a kinematic chain starting from root_link, following only kinematic joints."""
        chain = {
            'link_name': root_link,
            'joint_name': None,
            'joint_type': None,
            'xyz': [0, 0, 0],
            'rpy': [0, 0, 0],
            'axis': [0, 0, 0],
            'limit': None,
            'children': [],
            'attachments': [],  # fixed-attachment sub-meshes on this link
        }

        # Find the kinematic joint where this link is the parent
        if root_link in child_map:
            for child_name, jdata in child_map[root_link]:
                if jdata['type'] in ('continuous', 'revolute'):
                    child_chain = build_child_chain(child_name, jdata)
                    chain['children'].append(child_chain)
                # Also collect fixed attachments on this link
                elif jdata['type'] == 'fixed':
                    chain['attachments'].append({
                        'link_name': child_name,
                        'xyz': jdata['xyz'],
                        'rpy': jdata['rpy'],
                    })

        return chain

    def build_child_chain(link_name, parent_joint):
        """Build chain for a link that is reached via parent_joint."""
        chain = {
            'link_name': link_name,
            'joint_name': parent_joint.get('name', 'unknown'),
            'joint_type': parent_joint['type'],
            'xyz': parent_joint['xyz'],
            'rpy': parent_joint['rpy'],
            'axis': parent_joint['axis'],
            'limit': parent_joint.get('limit'),
            'children': [],
            'attachments': [],
        }

        # Find further kinematic children of this link
        if link_name in child_map:
            for child_name, jdata in child_map[link_name]:
                if jdata['type'] in ('continuous', 'revolute'):
                    child_chain = build_child_chain(child_name, jdata)
                    chain['children'].append(child_chain)
                elif jdata['type'] == 'fixed':
                    # Check if this fixed-attached link has kinematic children
                    # If so, follow through (e.g., gasket -> knee joint)
                    has_kinematic_child = False
                    if child_name in child_map:
                        for gc_name, gc_jdata in child_map[child_name]:
                            if gc_jdata['type'] in ('continuous', 'revolute'):
                                has_kinematic_child = True
                                # This fixed-attached link acts as an intermediate node
                                # with a kinematic child - add the kinematic child
                                sub_chain = build_child_chain(gc_name, gc_jdata)
                                chain['children'].append(sub_chain)
                            elif gc_jdata['type'] == 'fixed':
                                # Nested fixed attachment
                                chain['attachments'].append({
                                    'link_name': gc_name,
                                    'xyz': gc_jdata['xyz'],
                                    'rpy': gc_jdata['rpy'],
                                })
                    if not has_kinematic_child:
                        chain['attachments'].append({
                            'link_name': child_name,
                            'xyz': jdata['xyz'],
                            'rpy': jdata['rpy'],
                        })

        return chain

    # Build the 4 leg kinematic chains
    leg_chains = []
    for root in leg_roots:
        chain = build_leg_chain(root)
        leg_chains.append(chain)

    # Print structure for debugging
    def print_chain(chain, indent=0):
        prefix = '  ' * indent
        print(f"{prefix}{chain['link_name']}", end='')
        if chain.get('joint_name'):
            print(f" [{chain['joint_name']}]({chain['joint_type']})", end='')
        print()
        for att in chain.get('attachments', []):
            print(f"{prefix}  +{att['link_name']}")
        for child in chain.get('children', []):
            print_chain(child, indent + 1)

    print("\nLeg chain structure:")
    for i, chain in enumerate(leg_chains):
        print(f"\n--- Leg {i+1} ---")
        print_chain(chain)

    # ----- 4. Convert STL to GLB for each unique link with mesh -----
    # Collect all unique link names that need meshes
    def collect_link_names(chain, result):
        if chain['link_name'] not in result:
            result.add(chain['link_name'])
        for child in chain.get('children', []):
            collect_link_names(child, result)
        for att in chain.get('attachments', []):
            result.add(att['link_name'])

    all_links_with_meshes = set()
    for chain in leg_chains:
        collect_link_names(chain, all_links_with_meshes)

    # Also add body-v6 and body-mounted parts
    all_links_with_meshes.add('body-v6')

    # Add all links from fixed joints that have meshes
    for link_name, mesh_info in links.items():
        if mesh_info is not None:
            all_links_with_meshes.add(link_name)

    print(f"\nConverting {len(all_links_with_meshes)} unique meshes to GLB...")

    converted_files = {}
    for link_name in sorted(all_links_with_meshes):
        mesh_info = links.get(link_name)
        if mesh_info is None:
            print(f"  SKIP {link_name}: no visual mesh")
            continue

        stl_path = normalize_path(mesh_info['filename'])
        # Try to find the STL file
        stl_basename = os.path.basename(stl_path)
        stl_full = meshes_dir / stl_basename

        if not stl_full.exists():
            print(f"  MISS {link_name}: {stl_full}")
            continue

        try:
            # Load mesh and apply scale
            mesh = trimesh.load(str(stl_full))
            scale = mesh_info['scale']
            if isinstance(mesh, trimesh.Scene):
                # Merge scene into single mesh
                meshes_list = []
                for geom_name, geom in mesh.geometry.items():
                    geom.apply_scale(scale)
                    meshes_list.append(geom)
                if meshes_list:
                    merged = trimesh.util.concatenate(meshes_list)
                    mesh = merged
            else:
                mesh.apply_scale(scale)

            # Apply visual origin transform
            T = xyz_rpy_to_matrix(mesh_info['xyz'], mesh_info['rpy'])
            mesh.apply_transform(T)

            # Export as GLB
            glb_filename = f"{link_name}.glb"
            glb_path = output_dir / glb_filename
            mesh.export(str(glb_path), file_type='glb')
            converted_files[link_name] = glb_filename
            print(f"  OK   {link_name} -> {glb_filename}")

        except Exception as e:
            print(f"  FAIL {link_name}: {e}")

    # ----- 5. Generate robot_hierarchy.json -----
    def chain_to_json(chain, converted_files, default_angle=0):
        """Convert chain dict to JSON-serializable format."""
        mesh_file = converted_files.get(chain['link_name'])

        result = {
            'name': chain['link_name'],
        }
        if mesh_file:
            result['mesh'] = mesh_file

        # Collect attachments that have meshes
        att_meshes = []
        for att in chain.get('attachments', []):
            att_mesh = converted_files.get(att['link_name'])
            if att_mesh:
                att_meshes.append({
                    'name': att['link_name'],
                    'mesh': att_mesh,
                    'xyz': att['xyz'],
                    'rpy': att['rpy'],
                })
        if att_meshes:
            result['attachments'] = att_meshes

        result['children'] = []
        for child in chain.get('children', []):
            child_json = chain_to_json(child, converted_files)
            # Add joint info to parent side
            child_json['joint'] = {
                'name': child['joint_name'],
                'type': child['joint_type'],
                'xyz': child['xyz'],
                'rpy': child['rpy'],
                'axis': child['axis'],
                'limit': child['limit'],
                'angle': default_angle,
            }
            result['children'].append(child_json)

        return result

    # Build the full robot JSON
    robot_json = {
        'name': 'Wheel-Legged Robot',
        'root': 'body-v6',
    }

    # Add body-v6 mesh and attachments
    robot_json['mesh'] = converted_files.get('body-v6')

    # Collect body-v6 fixed attachments (body-mounted components)
    body_attachments = []
    if 'body-v6' in child_map:
        for child_name, jdata in child_map['body-v6']:
            if jdata['type'] == 'fixed':
                att_mesh = converted_files.get(child_name)
                if att_mesh:
                    body_attachments.append({
                        'name': child_name,
                        'mesh': att_mesh,
                        'xyz': jdata['xyz'],
                        'rpy': jdata['rpy'],
                    })
    if body_attachments:
        robot_json['attachments'] = body_attachments

    # Leg chains
    robot_json['legs'] = []
    for i, chain in enumerate(leg_chains):
        leg_json = chain_to_json(chain, converted_files)
        robot_json['legs'].append(leg_json)

    # Save hierarchy JSON
    hierarchy_path = output_dir / 'robot_hierarchy.json'
    with open(hierarchy_path, 'w', encoding='utf-8') as f:
        json.dump(robot_json, f, indent=2, ensure_ascii=False)
    print(f"\nHierarchy saved to {hierarchy_path}")

    # ----- 6. Generate summary of joints for UI -----
    joints_summary = {}
    for chain in leg_chains:
        def collect_joints(chain, prefix=''):
            result = {}
            for child in chain.get('children', []):
                jname = child.get('joint', {}).get('name', 'unknown')
                jdata_h = child.get('joint', {})
                if jdata_h.get('type') in ('continuous', 'revolute'):
                    result[jname] = {
                        'type': jdata_h['type'],
                        'axis': jdata_h['axis'],
                        'limit': jdata_h.get('limit'),
                        'child_link': child['name'],
                        'default_angle': 0,
                    }
                # Recurse
                for sub_child in child.get('children', []):
                    result.update(collect_joints({'children': [sub_child]}, jname))
            return result
        joints_summary.update(collect_joints(chain))

    # Save joint index
    joints_path = output_dir / 'joint_index.json'
    with open(joints_path, 'w', encoding='utf-8') as f:
        json.dump({
            'joints': joints_summary,
            'motor_mapping': {
                'motor_0': 'el05_1_kuan_revolve_1',
                'motor_1': 'el05_2_kuan_revolve_2',
                'motor_2': 'el05_3_kuan_revolve_3',
                'motor_3': 'el05_4_kuan_revolve_4',
            }
        }, f, indent=2)
    print(f"Joint index saved to {joints_path}")

    # Print motor mapping
    print("\nMotor Joint Mapping:")
    for motor, joint_name in [
        ('M0 (EL05_1)', 'el05_1_kuan_revolve_1'),
        ('M1 (EL05_2)', 'el05_2_kuan_revolve_2'),
        ('M2 (EL05_3)', 'el05_3_kuan_revolve_3'),
        ('M3 (EL05_4)', 'el05_4_kuan_revolve_4'),
    ]:
        j = joints_summary.get(joint_name, {})
        print(f"  {motor} -> {joint_name} (axis={j.get('axis')}, limit={j.get('limit')})")

    print("\nDone! Converted GLB files:")
    for link_name, glb_file in sorted(converted_files.items()):
        print(f"  {link_name}: {glb_file}")


if __name__ == '__main__':
    main()
