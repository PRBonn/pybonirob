import os
import pybonirob

class TestDataset:
    def setup(self):
        this_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.join(this_dir, 'data')
        prefix = 'bonirob'
        seq = 'test'
        self.data = pybonirob.dataset(base_path, prefix, seq)

    # Test if extrinisics are loaded correctly
    def test_extrinsics(self):
        self.data.load_extrinsics()
        assert len(self.data.extrinsics.base_to_jai_camera) == 6
        assert len(self.data.extrinsics.base_to_kinect) == 6
        assert len(self.data.extrinsics.base_to_fx8) == 6
        assert len(self.data.extrinsics.base_to_vlp_front) == 6
        assert len(self.data.extrinsics.base_to_vlp_rear) == 6
        assert len(self.data.extrinsics.base_to_ublox_gps) == 6
        assert len(self.data.extrinsics.base_to_leica_gps) == 6

    def test_camera(self):
        self.data.load_camera()
        # Jai camera
        assert len(self.data.camera.jai.rgb) > 0
        assert len(self.data.camera.jai.nir) > 0
        assert len(self.data.camera.jai.rgb) == len(self.data.camera.jai.nir)
        assert len(self.data.camera.jai.ts) > 0
        assert len(self.data.camera.jai.ts) == len(self.data.camera.jai.rgb)
        assert len(self.data.camera.jai.ts) == len(self.data.camera.jai.nir)
        assert len(self.data.camera.jai.calib_rgb) > 0 
        assert len(self.data.camera.jai.calib_nir) > 0

        # Kinect
        assert len(self.data.camera.kinect.rgb) > 0
        assert len(self.data.camera.kinect.ir) > 0
        assert len(self.data.camera.kinect.depth) > 0
        assert len(self.data.camera.kinect.rgb) == len(
            self.data.camera.kinect.ir)
        assert len(self.data.camera.kinect.ir) == len(
            self.data.camera.kinect.depth)
        assert len(self.data.camera.kinect.ts) > 0
        assert len(self.data.camera.kinect.ts) == len(
            self.data.camera.kinect.rgb)
        assert len(self.data.camera.kinect.ts) == len(
            self.data.camera.kinect.ir)
        assert len(self.data.camera.kinect.calib_color) > 0 
        assert len(self.data.camera.kinect.calib_ir) > 0
        assert len(self.data.camera.kinect.calib_depth) > 0
        assert len(self.data.camera.kinect.calib_pose) > 0

    def test_laser(self):
        self.data.load_laser()
        # Fx8
        assert len(self.data.laser.fx8.ir) > 0
        assert len(self.data.laser.fx8.range) > 0
        assert len(self.data.laser.fx8.ir) == len(self.data.laser.fx8.range)
        assert len(self.data.laser.fx8.ts) == len(self.data.laser.fx8.range)

        # Velodyne
        assert len(self.data.laser.velodyne.rear.scans) > 0
        assert len(self.data.laser.velodyne.rear.ts_start) == len(
            self.data.laser.velodyne.rear.ts_end)
        assert len(self.data.laser.velodyne.rear.ts_start) == len(
            self.data.laser.velodyne.rear.scans)
        assert len(self.data.laser.velodyne.front.scans) > 0
        assert len(self.data.laser.velodyne.front.ts_start) == len(
            self.data.laser.velodyne.front.ts_end)
        assert len(self.data.laser.velodyne.front.ts_start) == len(
            self.data.laser.velodyne.front.scans)
        assert len(self.data.laser.velodyne.calib) > 0

    def test_gps(self):
        self.data.load_gps()
        assert len(self.data.gps.leica) > 0
        assert len(self.data.gps.ublox) > 0

    def test_odometry(self):
        self.data.load_odometry()
        assert len(self.data.odom) > 0
