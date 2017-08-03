import os
import pybonirob
import pytest


class TestUtils:
    def setup(self):
        this_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.join(this_dir, 'data')
        prefix = 'bonirob'
        seq = 'test'
        self.data = pybonirob.dataset(base_path, prefix, seq)
        self.data.set_load_into_memory()

    def test_kinect_pointcloud(self, tmpdir):
        self.data.load_camera()
        img_rgb = self.data.camera.kinect.rgb[0]
        img_depth = self.data.camera.kinect.depth[0]
        kinect_calib = self.data.camera.kinect.calib_ir
        kinect_points = pybonirob.utils.generate_kinect_pointcloud(
            img_rgb, img_depth, kinect_calib)
        ply_file = tmpdir.join('kinect.ply')
        ply_file_name = str(ply_file)
        pybonirob.utils.save_kinect_pointcloud_as_ply(
            kinect_points, ply_file_name)
        assert len(kinect_points) > 0
        assert os.path.getsize(ply_file_name) > 0

    def test_velodyne_pointcloud(self, tmpdir):
        self.data.load_laser()
        velo_scan = self.data.laser.velodyne.rear.scans[0]
        ply_file = tmpdir.join('velodyne.ply')
        ply_file_name = str(ply_file)
        print(ply_file_name)
        pybonirob.utils.save_velodyne_pointcloud_as_ply(
            velo_scan, ply_file_name)
        assert os.path.getsize(ply_file_name) > 0
