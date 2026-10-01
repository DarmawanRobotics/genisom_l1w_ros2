#!/usr/bin/env python3
"""Plot L1W sensor frames over the STL meshes and report their distance to the mast."""
import argparse
import os
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

SENSOR_FRAMES = [
    'livox_frame', 'livox_imu_frame',
    'camera_bottom_screw_frame', 'camera_link', 'camera_color_optical_frame',
    'front_camera_optical_frame',
]
MAST_LINK = 'ssz_uper_link'


def rpy_to_R(r, p, y):
    """Rotation matrix from URDF roll-pitch-yaw."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    """Rotation matrix for angle `a` about `axis`."""
    k = np.asarray(axis, float)
    n = np.linalg.norm(k)
    if n < 1e-12:
        return np.eye(3)
    k /= n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def origin_T(el):
    """4x4 transform from a URDF <origin> element."""
    T = np.eye(4)
    if el is not None:
        T[:3, 3] = [float(v) for v in el.get('xyz', '0 0 0').split()]
        T[:3, :3] = rpy_to_R(*[float(v) for v in el.get('rpy', '0 0 0').split()])
    return T


def load_urdf(path, xacro_args):
    """Parse a URDF, expanding it with xacro first if needed."""
    if path.endswith('.xacro'):
        xml = subprocess.check_output(['xacro', path] + xacro_args, text=True)
        return ET.fromstring(xml)
    return ET.parse(path).getroot()


def forward_kinematics(root, q):
    """World transform of every link for joint positions `q`."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    T = {}

    def get(link):
        if link in T:
            return T[link]
        j = joints.get(link)
        if j is None:
            T[link] = np.eye(4)
            return T[link]
        M = origin_T(j.find('origin'))
        if j.get('type') in ('revolute', 'continuous') and j.get('name') in q:
            ax = j.find('axis')
            R = np.eye(4)
            R[:3, :3] = axis_angle([float(v) for v in ax.get('xyz').split()], q[j.get('name')])
            M = M @ R
        T[link] = get(j.find('parent').get('link')) @ M
        return T[link]

    for link in root.findall('link'):
        get(link.get('name'))
    return T


def resolve_mesh(uri, mesh_dir):
    """Resolve a package:// or file:// mesh URI to a local path."""
    if uri.startswith('package://'):
        pkg, rel = uri[len('package://'):].split('/', 1)
        if mesh_dir:
            return os.path.join(mesh_dir, os.path.basename(rel))
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory(pkg), rel)
        except Exception:
            return None
    if uri.startswith('file://'):
        return uri[len('file://'):]
    return uri


def load_stl(path):
    """Load binary or ASCII STL as an (N, 3, 3) triangle array."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + n * 50 == len(data):
            arr = np.frombuffer(data[84:], dtype=np.dtype([
                ('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n)
            return arr['v'].astype(float)
    verts = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors='ignore').splitlines()
             if ln.strip().startswith('vertex')]
    return np.array(verts, float).reshape(-1, 3, 3)


def point_tri_distance(p, tri):
    """Minimum distance from point `p` to a triangle soup (Ericson closest-point)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = np.einsum('ij,ij->i', ab, ap), np.einsum('ij,ij->i', ac, ap)
    bp = p - b
    d3, d4 = np.einsum('ij,ij->i', ab, bp), np.einsum('ij,ij->i', ac, bp)
    cp = p - c
    d5, d6 = np.einsum('ij,ij->i', ab, cp), np.einsum('ij,ij->i', ac, cp)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    with np.errstate(divide='ignore', invalid='ignore'):
        denom = va + vb + vc
        v, w = vb / denom, vc / denom
        q = a + ab * v[:, None] + ac * w[:, None]
        t_ab = np.clip(d1 / (d1 - d3), 0, 1)
        t_ac = np.clip(d2 / (d2 - d6), 0, 1)
        t_bc = np.clip((d4 - d3) / ((d4 - d3) + (d5 - d6)), 0, 1)
    m = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    q[m] = (a + ab * t_ab[:, None])[m]
    m = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    q[m] = (a + ac * t_ac[:, None])[m]
    m = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    q[m] = (b + (c - b) * t_bc[:, None])[m]
    q[(d1 <= 0) & (d2 <= 0)] = a[(d1 <= 0) & (d2 <= 0)]
    q[(d3 >= 0) & (d4 <= d3)] = b[(d3 >= 0) & (d4 <= d3)]
    q[(d6 >= 0) & (d5 <= d6)] = c[(d6 >= 0) & (d5 <= d6)]
    q = np.nan_to_num(q, nan=np.inf)
    return np.min(np.linalg.norm(q - p, axis=1))


def load_meshes(root, T, mesh_dir, max_faces):
    """World-space triangles per link, the full-res mast mesh and the list of skipped meshes."""
    tris, mast_tris, missing = [], None, []
    for link in root.findall('link'):
        name = link.get('name')
        for vis in link.findall('visual'):
            m = vis.find('geometry/mesh')
            if m is None:
                continue
            path = resolve_mesh(m.get('filename'), mesh_dir)
            if not path or not os.path.isfile(path) or not path.lower().endswith('.stl'):
                missing.append(m.get('filename'))
                continue
            scale = np.array([float(v) for v in m.get('scale', '1 1 1').split()])
            W = T[name] @ origin_T(vis.find('origin'))
            tw = (load_stl(path) * scale) @ W[:3, :3].T + W[:3, 3]
            if name.endswith(MAST_LINK):
                mast_tris = tw
            if len(tw) > max_faces:
                tw = tw[np.random.default_rng(0).choice(len(tw), max_faces, replace=False)]
            tris.append((name, tw))
    return tris, mast_tris, missing


def print_report(T, frames, mast_tris):
    """Print each sensor frame position and its distance to the mast surface."""
    print(f'\n{"frame":32s} {"x":>8s} {"y":>8s} {"z":>8s}   {"d_mast[mm]":>10s}  inside_mast_bbox')
    lo = hi = None
    if mast_tris is not None:
        verts = mast_tris.reshape(-1, 3)
        lo, hi = verts.min(0), verts.max(0)
    for f in frames:
        p = T[f][:3, 3]
        d, inside = '-', '-'
        if mast_tris is not None:
            d = f'{point_tri_distance(p, mast_tris) * 1000:10.1f}'
            inside = str(bool(np.all(p >= lo - 1e-3) and np.all(p <= hi + 1e-3)))
        print(f'{f:32s} {p[0]:8.4f} {p[1]:8.4f} {p[2]:8.4f}   {d:>10s}  {inside}')
    if lo is not None:
        print(f'\nmast bbox min={np.round(lo, 4)} max={np.round(hi, 4)}')


def frame_color(frame):
    """Plot color per sensor family."""
    if 'front_camera' in frame:
        return 'tab:green'
    return 'tab:red' if 'livox' in frame else 'tab:blue'


def plot_frames(T, frames, tris, title, out):
    """Save an iso + top/side/front plot of meshes and sensor frame axes."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    pts = (np.concatenate([t.reshape(-1, 3) for _, t in tris]) if tris
           else np.array([T[f][:3, 3] for f in frames]))
    span = max(np.ptp(pts, axis=0).max(), 0.3)
    mid = pts.mean(0)
    alen = span * 0.06

    fig = plt.figure(figsize=(16, 12))

    ax3 = fig.add_subplot(2, 2, 1, projection='3d')
    for name, t in tris:
        c = (0.55, 0.6, 0.7, 0.35) if name.endswith(MAST_LINK) else (0.7, 0.7, 0.7, 0.15)
        ax3.add_collection3d(Poly3DCollection(t, facecolor=c, edgecolor='none'))
    for f in frames:
        o = T[f][:3, 3]
        for i, c in enumerate('rgb'):
            ax3.plot(*zip(o, o + T[f][:3, i] * alen), color=c, lw=2)
        ax3.text(*o, f, fontsize=7, color=frame_color(f))
    ax3.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
    ax3.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
    ax3.set_zlim(mid[2] - span / 2, mid[2] + span / 2)
    ax3.set_box_aspect((1, 1, 1))
    ax3.set_xlabel('x')
    ax3.set_ylabel('y')
    ax3.set_zlabel('z')
    ax3.set_title('iso')

    views = [('top (X fwd, Y left)', 0, 1), ('side (X fwd, Z up)', 0, 2), ('front (Y left, Z up)', 1, 2)]
    for k, (view, i, j) in enumerate(views, start=2):
        ax = fig.add_subplot(2, 2, k)
        for name, t in tris:
            c = (0.45, 0.5, 0.65, 0.25) if name.endswith(MAST_LINK) else (0.6, 0.6, 0.6, 0.12)
            ax.add_collection(PolyCollection(t[:, :, [i, j]], facecolor=c, edgecolor='none'))
        for f in frames:
            o = T[f][:3, 3]
            for ai, c in enumerate('rgb'):
                e = o + T[f][:3, ai] * alen
                ax.plot([o[i], e[i]], [o[j], e[j]], color=c, lw=1.5)
            ax.plot(o[i], o[j], 'o', color=frame_color(f), ms=4)
            ax.annotate(f.replace('_frame', ''), (o[i], o[j]), fontsize=7, color=frame_color(f),
                        xytext=(4, 4), textcoords='offset points')
        ax.set_xlim(mid[i] - span / 2, mid[i] + span / 2)
        ax.set_ylim(mid[j] - span / 2, mid[j] + span / 2)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_xlabel('xyz'[i])
        ax.set_ylabel('xyz'[j])
        ax.set_title(view)

    fig.suptitle(f'L1W sensor frames vs mesh - {title}  (axes: x=red, y=green, z=blue)')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f'\nsaved -> {os.path.abspath(out)}')


def main():
    """Parse args, compute FK, print the frame report and save the plot."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', default=None, help='URDF or xacro, default: l1w_description/urdf/l1w.urdf.xacro')
    ap.add_argument('--mesh-dir', default=None, help='STL folder, overrides package:// lookup')
    ap.add_argument('--xacro-arg', action='append', default=[], help='e.g. use_camera_mesh:=false')
    ap.add_argument('--q', action='append', default=[], help='joint=rad, repeatable')
    ap.add_argument('--max-faces', type=int, default=8000, help='max faces per mesh in the plot')
    ap.add_argument('--out', default='l1w_frames.png')
    args = ap.parse_args()

    model = args.model
    if model is None:
        from ament_index_python.packages import get_package_share_directory
        model = os.path.join(get_package_share_directory('l1w_description'), 'urdf', 'l1w.urdf.xacro')

    q = {k: float(v) for k, v in (s.split('=') for s in args.q)}
    root = load_urdf(model, args.xacro_arg)
    T = forward_kinematics(root, q)

    tris, mast_tris, missing = load_meshes(root, T, args.mesh_dir, args.max_faces)
    if missing:
        print(f'[warn] skipped {len(missing)} missing or non-STL meshes:')
        for m in sorted(set(missing)):
            print('   ', m)

    frames = [f for f in T if any(f.endswith(s) for s in SENSOR_FRAMES)]
    print_report(T, frames, mast_tris)
    plot_frames(T, frames, tris, os.path.basename(model), args.out)


if __name__ == '__main__':
    sys.exit(main())
