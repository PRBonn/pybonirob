# pybonirob
This package provides a basic set of tools to work with the bonirob datasets. It parses through the dataset and
provides methods to access the data captured from different sensors. The dataset consists of measurements from a
camera, kinect, velodyne, range scanner, gps and odometry.

The complete dataset can be downloaded from:
```
[http://www.ipb.uni-bonn.de/data/sugarbeets2016/]
```

## Installation
### Prerequisites
* Requires python/python3: `sudo apt-get install python python3` 
* libpng, libfreetype : `sudo apt-get install libpng-dev libfreetype6-dev`

### Module Dependencies
* numpy
* pyyaml
* pillow
* pytest (for running tests)
* cv2 (for vizualization in the example)

These modules can be installed with
```
sudo pip install package-name
```

### Install
To install the package:
```
cd /path/to/pybonirob
python setup.py install 
```
## Usage

To load the dataset, we need to specify the location of the dataset to be loaded and then access 
different sensor measurements using the provided public methods.

Here is a basic example :
```python
import pybonirob

base_path = '/path/to/the/dataset/directory'
prefix = 'bonirob_2016-05-23-10-47-22'
seq = '2'

# Data from different sensor modalities can be loaded separately
data.load_extrinsics()
data.load_camera()
data.load_gps()
data.load_laser()
data.load_odometry()

```

In the examples folder, a detailed demo program is provided which shows how to access the data
from different sensors.

## Related publication
If you use the dataset for your research, please cite the related publication:
```
@article{chebrolu2017ijrr,
title = {Agricultural robot dataset for plant classification, localization and mapping on sugar beet fields},
author = {Nived Chebrolu and Philipp Lottes and Alexander Schaefer and Wera Winterhalter and Wolfram Burgard and Cyrill Stachniss},
journal = {The International Journal of Robotics Research},
year = {2017}
doi = {10.1177/0278364917720510},
}
```