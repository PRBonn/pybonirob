""""Provides 'dataset' which parses and loads raw Bonirob data"""

import os
from collections import namedtuple
import glob
import pybonirob.utils as utils
import pybonirob.transformations as transformations
import yaml
import numpy as np
from PIL import Image

class dataset:
    """ Load and parse data into a usable format."""

    def __init__(self, base_path, prefix, seq):
        """Set the path."""
        self.bag = prefix + '_' + seq
        self.data_path = os.path.join(base_path, self.bag)
        self.calib_path = os.path.join(base_path, 'calibration')
        self.load_into_memory = False
        print(self.data_path)

    def set_load_into_memory(self):
        """ Loads all the images into memory (Not recommended)"""
        self.load_into_memory = True

    def load_extrinsics(self):
        """ Loads the sensor extrinsics with respect to the base."""
        print('Loading extrinsic calibration parameters ...')
        extrinsics_file_path = os.path.join(
            self.calib_path, 'extrinsics/extrinsics.yaml')
        extrinsics_all = yaml.load(open(extrinsics_file_path, 'r'))

        base_to_jai_camera = self._load_transformation(
            extrinsics_all['jai_camera'])
        base_to_kinect = self._load_transformation(
            extrinsics_all['kinect'])
        base_to_fx8 = self._load_transformation(
            extrinsics_all['fx8'])
        base_to_vlp_front = self._load_transformation(
            extrinsics_all['vlp_front'])
        base_to_vlp_rear = self._load_transformation(
            extrinsics_all['vlp_rear'])
        base_to_ublox_gps = self._load_transformation(
            extrinsics_all['ublox_gps'])
        base_to_leica_gps = self._load_transformation(
            extrinsics_all['leica_gps'])

        # Combined
        extrinsics_data = {}
        extrinsics_data['base_to_jai_camera'] = base_to_jai_camera
        extrinsics_data['base_to_kinect'] = base_to_kinect
        extrinsics_data['base_to_fx8'] = base_to_fx8
        extrinsics_data['base_to_vlp_front'] = base_to_vlp_front
        extrinsics_data['base_to_vlp_rear'] = base_to_vlp_rear
        extrinsics_data['base_to_ublox_gps'] = base_to_ublox_gps
        extrinsics_data['base_to_leica_gps'] = base_to_leica_gps

        self.extrinsics = namedtuple('Extrinsics', extrinsics_data.keys())(
            *extrinsics_data.values())

    def load_camera(self):
        """Loads all camera"""
        print('Loading camera data !!!')
        # Jai
        jai_data = {}
        camera_rgb_path = os.path.join(self.data_path, 'camera', 'jai', 'rgb')
        jai_data['rgb'] = self._load_images(camera_rgb_path)

        camera_nir_path = os.path.join(self.data_path, 'camera', 'jai', 'nir')
        jai_data['nir'] = self._load_images(camera_nir_path)

        jai_ts_file = os.path.join(
            self.data_path, 'camera', 'jai', 'timestamp', 'timestamps.txt')
        jai_data['ts'] = utils.load_timestamps(jai_ts_file)

        jai_nir_calib_file = os.path.join(
            self.calib_path, 'jai', 'jai_camera_nir.yaml')

        jai_data['calib_nir'] = yaml.load(open(jai_nir_calib_file, 'r'))

        jai_rgb_calib_file = os.path.join(
            self.calib_path, 'jai', 'jai_camera_rgb.yaml')

        jai_data['calib_rgb'] = yaml.load(open(jai_rgb_calib_file, 'r'))

        jai_all = namedtuple('jai_all', jai_data.keys())(*jai_data.values())

        # Kinect
        kinect_data = {}
        kinect_rgb_path = os.path.join(
            self.data_path, 'camera', 'kinect', 'color')
        kinect_data['rgb'] = self._load_images(kinect_rgb_path)

        kinect_ir_path = os.path.join(self.data_path, 'camera', 'kinect', 'ir')
        kinect_data['ir'] = self._load_images(kinect_ir_path)

        kinect_depth_path = os.path.join(
            self.data_path, 'camera', 'kinect', 'depth')
        kinect_data['depth'] = self._load_images(kinect_depth_path)

        kinect_ts_file = os.path.join(
            self.data_path, 'camera', 'kinect', 'timestamp', 'timestamps.txt')
        kinect_data['ts'] = utils.load_timestamps(kinect_ts_file)

        kinect_color_calib_file = os.path.join(
            self.calib_path, 'kinect', 'kinect_color.yaml')

        kinect_data['calib_color'] = yaml.load(
            open(kinect_color_calib_file, 'r'))

        kinect_ir_calib_file = os.path.join(
            self.calib_path, 'kinect', 'kinect_ir.yaml')

        kinect_data['calib_ir'] = yaml.load(
            open(kinect_ir_calib_file, 'r'))

        kinect_depth_calib_file = os.path.join(
            self.calib_path, 'kinect', 'kinect_depth.yaml')

        kinect_data['calib_depth'] = yaml.load(
            open(kinect_depth_calib_file, 'r'))

        kinect_pose_calib_file = os.path.join(
            self.calib_path, 'kinect', 'kinect_pose.yaml')

        kinect_data['calib_pose'] = yaml.load(
            open(kinect_pose_calib_file, 'r'))

        kinect_all = namedtuple('kinect_all', kinect_data.keys())(
            *kinect_data.values())

        # Combined
        camera_data = {}
        camera_data['jai'] = jai_all
        camera_data['kinect'] = kinect_all
        self.camera = namedtuple('CameraDevices', camera_data.keys())(
            *camera_data.values())

        print('done')

    def load_gps(self):
        print('Loading GPS data !!!')
        gps_data = {}
        # Ublox
        ublox_file = os.path.join(self.data_path, 'gps', 'ublox', 'gps.txt')
        gps_data['ublox'] = self._load_gps_file(ublox_file)

        # Leica
        leica_file = os.path.join(self.data_path, 'gps', 'leica', 'gps.txt')
        gps_data['leica'] = self._load_gps_file(leica_file)

        self.gps = namedtuple('GpsDevices', gps_data.keys())(
            *gps_data.values())

        print('done')

    def load_odometry(self):
        print('Loading odometry data!')
        OdomMsg = namedtuple(
            'OdomMsg', 'ts lvel_x lvel_y avel_z pos_x pos_y heading')
        odom_file = os.path.join(self.data_path, 'odometry', 'odom.txt')
        readings = []

        with open(odom_file, 'r') as f:
            for line in f.readlines():
                line = line.split()
                if line[0] != '#':
                    data = OdomMsg(*line)
                    readings.append(data)

        self.odom = readings
        print('done')

    def load_laser(self):
        print('Loading laser data!')

        # Velodyne
        velodyne_data = {}
        velo_rear_path = os.path.join(
            self.data_path, 'laser', 'velodyne', 'rear')
        velo_front_path = os.path.join(
            self.data_path, 'laser', 'velodyne', 'front')

        velodyne_data['rear'] = self._load_velo(velo_rear_path)
        velodyne_data['front'] = self._load_velo(velo_front_path)

        velodyne_calib_file = os.path.join(
            self.calib_path, 'velodyne', 'velodyne_calib.yaml')

        velodyne_data['calib'] = yaml.load(open(velodyne_calib_file, 'r'))

        velodyne_all = namedtuple('velodyne_all', velodyne_data.keys())(
            *velodyne_data.values())

        # Fx8
        fx8_data = {}
        fx8_ir_path = os.path.join(self.data_path, 'laser', 'fx8', 'ir')
        fx8_data['ir'] = self._load_images(fx8_ir_path)

        fx8_range_path = os.path.join(self.data_path, 'laser', 'fx8', 'range')
        fx8_data['range'] = self._load_images(fx8_range_path)

        fx8_ts_file = os.path.join(
            self.data_path, 'laser', 'fx8', 'timestamp', 'timestamps.txt')
        fx8_data['ts'] = utils.load_timestamps(fx8_ts_file)

        fx8_all = namedtuple('fx8_all', fx8_data.keys())(*fx8_data.values())

        # Combined
        laser_data = {}
        laser_data['velodyne'] = velodyne_all
        laser_data['fx8'] = fx8_all

        self.laser = namedtuple('LaserDevices', laser_data.keys())(
            *laser_data.values())

        print('done')

    # for internal use by the class

    def _load_transformation(self, tf):
        tf = tf.split()
        Transformation = namedtuple('Transformation', 'x y z roll pitch yaw')
        quat = np.array([float(tf[3]), float(tf[4]),
                         float(tf[5]), float(tf[6])])
        roll, pitch, yaw = transformations.euler_from_quaternion(quat, 'sxyz')
        transformation = Transformation(float(tf[0]), float(tf[1]), float(
            tf[2]), roll, pitch, yaw)
        return transformation

    def _load_gps_file(self, gps_file):
        GpsFix = namedtuple('GpsFix', 'ts lat long alt')
        gps_readings = []
        with open(gps_file, 'r') as f:
            for line in f.readlines():
                line = line.split()
                if line[0] != '#':
                    data = GpsFix(*line)
                    gps_readings.append(data)
        return gps_readings

    def _load_velo(self, velo_path):
        """Load velodyne [x,y,z,reflectance, ring number] scan data from binary
         files."""

        # Find all the Velodyne files
        velo_files_path = os.path.join(velo_path, '*.bin')
        velo_files = sorted(glob.glob(velo_files_path))
        print('Found ' + str(len(velo_files)) + ' Velodyne scans...')

        # Read the Velodyne scans. Each point is [x,y,z,reflectance, ring
        # number]
        scans_list = utils.load_velo_scans(velo_files)

        # Load corresponding timestamps
        ts_start, ts_end = utils.load_velo_timestamps(velo_path)

        velo_data = {}
        velo_data['scans'] = scans_list
        velo_data['ts_start'] = ts_start
        velo_data['ts_end'] = ts_end
        velodyne = namedtuple('velodyne', velo_data.keys())(
            *velo_data.values())

        return velodyne

    def _load_images(self, img_path):
        """Load RGB images from file."""
        print('Loading images from ' + img_path + '...')

        img_files_path = os.path.join(img_path, '*.png')
        img_files = sorted(glob.glob(img_files_path))

        print('Found ' + str(len(img_files)) + ' images ...')

        images = []
        for imfile in img_files:
            if(self.load_into_memory):
                im = Image.open(imfile)
                images.append(im)
            else:
                images.append(imfile)

        return images
