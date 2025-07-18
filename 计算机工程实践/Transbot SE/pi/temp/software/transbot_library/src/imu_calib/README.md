# imu_calib

This repository contains a ROS package with tools for computing and applying calibration parameters to IMU measurements.

## Usage
The package contains two nodes. The first computes the accelerometer calibration parameters and saves them to a YAML file, and needs to be run only once. After you have run this node to generate the YAML calibration file, the second node uses that file to apply the calibration to an uncalibrated IMU topic to produce a calibrated IMU topic.

## Nodes

### do_calib
Computes the accelerometer calibration parameters. It should be run directly with a rosrun in a terminal rather than from a launch file, since it requires keyboard input. After receiving the first IMU message, the node will prompt you to hold the IMU in a certain orientation and then press Enter to record measurements. After all 6 orientations are complete, the node will compute the calibration parameters and write them to the specified YAML file.

The underlying algorithm is a least-squares calibration approach based on and similar to that described in STMicroeletronics Application Note [AN4508](http://www.st.com/content/ccc/resource/technical/document/application_note/a0/f0/a0/62/3b/69/47/66/DM00119044.pdf/files/DM00119044.pdf/jcr:content/translations/en.DM00119044.pdf). Due to the nature of the algorithm, obtaining a good calibration requires fairly accurate positioning of the IMU along each of its axes.

#### Topics

##### Subscribed Topics
- `raw_imu` (lino_msgs/Imu) <br>
  The raw, uncalibrated IMU measurements

#### Parameters
- `~calib_file` (string, default: "imu_calib.yaml") <br>
  The file to which the calibration parameters will be written
- `~measurements` (int, default: 500) <br>
  The number of measurements to collect for each orientation
- `~reference_acceleration` (double, default: 9.80665) <br>
  The expected acceleration due to gravity

### apply_calib
Applies the accelerometer calibration parameters computed by the do_calib node. Also optionally (enabled by default) computes the gyro biases at startup and subtracts them off.

#### Topics

##### Subscribed Topics
- `raw_imu` (lino_msgs/Imu) <br>
  The raw, uncalibrated IMU measurements

##### Published Topics
- `imu/data_raw` (sensor_msgs/Imu) <br>
  The corrected, calibrated IMU measurements

#### Parameters
- `~calib_file` (string, default: "imu_calib.yaml") <br>
  The file from which to read the calibration parameters
- `~calibrate_gyros` (bool, default: true) <br>
  Whether to compute gyro biases at startup and subsequently subtract them off
- `~gyro_calib_samples` (int, default: 100) <br>
  The number of measurements to use for computing the gyro biases
