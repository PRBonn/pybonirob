"""Provides helper methods for parsing Bonirob data"""
import os
import numpy as np
from collections import namedtuple


def load_velo_scans(velo_files):
    """Helper method to parse velodyne binary files into a list of scans."""
    scan_list = []
    for filename in velo_files:
        scan = np.fromfile(filename, dtype=np.float32)
        scan_list.append(scan.reshape((-1, 5)))
    return scan_list


def load_timestamps(ts_file):
    """ Helper method to load timestamps"""
    ts = []
    with open(ts_file, 'r') as f:
        for line in f.readlines():
            line = line.split()
            if line[0] != "#":
                ts.append(line)

    return ts


def load_velo_timestamps(velo_path):
    """Helper method to parse start and end of each velodyne scan."""
    ts_start_file = os.path.join(velo_path, 'timestamps_start.txt')
    ts_end_file = os.path.join(velo_path, 'timestamps_end.txt')

    ts_start = load_timestamps(ts_start_file)
    ts_end = load_timestamps(ts_end_file)

    return ts_start, ts_end

def save_velodyne_pointcloud_as_ply(velo_scan, ply_file):   
    f = open(ply_file, 'w')
    f.write('ply\nformat ascii 1.0\n')
    f.write('element vertex %s\n' % len(velo_scan))
    f.write('property float x\n')
    f.write('property float y\n')
    f.write('property float z\n')
    f.write('end_header\n')
    for p in velo_scan:
        new_point = str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2]) + '\n'
        f.write(new_point)
    f.close()



def generate_kinect_pointcloud(rgb, depth, calib):
    # Calibration parmeters
    CENTER_X = calib["projection"]["data"][2]
    CENTER_Y = calib["projection"]["data"][6]
    SCALING_FACTOR = 1000.0  # Verify this
    FOCAL_LENGTH = calib["projection"]["data"][0]

    KinectPoint = namedtuple('KinectPoint', 'x y z r g b')
    points = []
    for v in range(rgb.size[1]):
        for u in range(rgb.size[0]):
            color = rgb.getpixel((u, v))
            z = depth.getpixel((u, v)) / SCALING_FACTOR
            if z == 0:
                continue
            x = (u - CENTER_X) * z / FOCAL_LENGTH
            y = (v - CENTER_Y) * z / FOCAL_LENGTH
            new_point = KinectPoint(x, y, z, color[0], color[1], color[2])
            points.append(new_point)

    return points


def save_kinect_pointcloud_as_ply(points, ply_file):
    f = open(ply_file, 'w')
    f.write('ply\nformat ascii 1.0\n')
    f.write('element vertex %s\n' % len(points))
    f.write('property float x\n')
    f.write('property float y\n')
    f.write('property float z\n')
    f.write('property uchar red\n')
    f.write('property uchar green\n')
    f.write('property uchar blue\n')
    f.write('end_header\n')
    for p in points:
        new_point = str(p.x) + ' ' + str(p.y) + ' ' + str(p.z) + \
            ' ' + str(p.r) + ' ' + str(p.g) + ' ' + str(p.b) + '\n'
        f.write(new_point)
    f.close()

def generate_fx8_pointcloud(range, calib):
    print('To Do')

def save_fx8_pointcloud_as_ply(points, ply_file):
    print('To Do')
