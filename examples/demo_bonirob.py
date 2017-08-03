import pybonirob
import cv2

# Set to the directory where the bonirob dataset is stored
base_path = '/home/alphabot/Data/pybonirob'

# Specify the dataset to load
prefix = 'bonirob'
seq = 'test'

# To turn on/off image visualization
visualize_data = True

# Load the data
data = pybonirob.dataset(base_path, prefix, seq)

# Load data from individual sensor modalities
data.load_extrinsics()
data.load_camera()
data.load_gps()
data.load_laser()
data.load_odometry()

# Display some of the data
# Extrinsics
print('Extrinsics:')
print("T: base to fx8")
print(data.extrinsics.base_to_fx8)
print('T: base to kinect')
print(data.extrinsics.base_to_kinect)
print('T: base to jai camera')
print(data.extrinsics.base_to_jai_camera)
print('T: base to velodyne_front')
print(data.extrinsics.base_to_vlp_front)
print('T: base to velodyne_rear')
print(data.extrinsics.base_to_vlp_rear)
print('T: base to ublox_gps')
print(data.extrinsics.base_to_ublox_gps)
print('T: base to leica_gps ')
print(data.extrinsics.base_to_leica_gps)

# # # Camera
print("Jai camera images")
print("Loaded", len(data.camera.jai.rgb), "Jai rgb images")
if visualize_data:
    print("Showing", len(data.camera.jai.rgb), "Jai rgb images")
    for img in data.camera.jai.rgb:
        if data.load_into_memory:
            cv2.imshow('jai_rgb', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('jai_rgb', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()

print("Loaded", len(data.camera.jai.nir), "Jai nir images")
if visualize_data:
    print("Showing", len(data.camera.jai.nir), "Jai nir images")
    for img in data.camera.jai.nir:
        if data.load_into_memory:
            cv2.imshow('jai_nir', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('jai_nir', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()

print("Kinect images")
print("Loaded", len(data.camera.kinect.rgb), "Kinect rgb images")
if visualize_data:
    print("Showing", len(data.camera.kinect.rgb), "Kinect rgb images")
    for img in data.camera.kinect.rgb:
        if data.load_into_memory:
            cv2.imshow('kinect_rgb', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('kinect_rgb', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()

print("Loaded", len(data.camera.kinect.ir), "Kinect ir images")
if visualize_data:
    print("Showing", len(data.camera.kinect.ir), "Kinect ir images")
    for img in data.camera.kinect.ir:
        if data.load_into_memory:
            cv2.imshow('kinect_ir', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('kinect_ir', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()


print("Loaded", len(data.camera.kinect.depth), "Kinect depth images")
if visualize_data:
    print("Showing", len(data.camera.kinect.depth), "Kinect depth images")
    for img in data.camera.kinect.depth:
        if data.load_into_memory:
            cv2.imshow('kinect_depth', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('kinect_depth', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()

# # GPS
print(" Ublox GPS:")
if visualize_data:
    for gps in data.gps.ublox:
        print('Timestamp', gps.ts, 'Lat', gps.lat,
            'Long', gps.long, 'Alt', gps.alt)

print("Leica GPS:")
if visualize_data:
    for gps in data.gps.leica:
        print('Timestamp', gps.ts, 'Lat', gps.lat,
            'Long', gps.long, 'Alt', gps.alt)

# Odometry
print("Odometry:")
if visualize_data:
    for odom in data.odom:
        print('Timestamp', odom.ts, 'lin_vel_x', odom.lvel_x, 'lin_vel_y', odom.lvel_y, 'ang_vel_z', odom.avel_z, 'pos_x',
            odom.pos_x, 'pos_y', odom.pos_y, 'heading', odom.heading)

# Laser
# Velodyne
print("Velodyne:")
print("Loaded", len(data.laser.velodyne.rear), "velodyne (rear) scans")
if visualize_data:
    print("Displaying velodyne data (rear):")
    for i in range(0, len(data.laser.velodyne.rear)):
        print("Timestamp start", data.laser.velodyne.rear.ts_start[i], "Timestamp stop", data.laser.velodyne.rear.ts_end[i],
            "Number of points",
            len(data.laser.velodyne.rear.scans[i]))

print("Loaded", len(data.laser.velodyne.front), "velodyne (front) scans")
if visualize_data:
    print("Displaying velodyne data (front):")
    for i in range(0, len(data.laser.velodyne.front)):
        print("Timestamp start", data.laser.velodyne.front.ts_start[i], "Timestamp stop",
            data.laser.velodyne.front.ts_end[i], "Number of points",
            len(data.laser.velodyne.front.scans[i]))

# # Fx8
print("Fx8:")
print("Loaded", len(data.laser.fx8.ir), "fx8 ir images")
if visualize_data:
    print("Showing", len(data.laser.fx8.ir), "fx8 ir images")
    for img in data.laser.fx8.ir:
        if data.load_into_memory:
            cv2.imshow('fx8_ir', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('fx8_ir', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()        

print("Loaded", len(data.laser.fx8.range), "fx8 range images")
if visualize_data:
    print("Showing", len(data.laser.fx8.range), "fx8 range images")
    for img in data.laser.fx8.range:
        if data.load_into_memory:
            cv2.imshow('fx8_range', img)
        else:
            im = cv2.imread(img)
            cv2.imshow('fx8_range', im)
            cv2.waitKey(0)
    cv2.destroyAllWindows()        
