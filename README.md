# VINS-Mono-QPEP
The QPEP (Quadratic Pose Estimation Problems) Enhanced VINS-Mono (Originated from https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) . The details of the QPEP can be found in https://github.com/zarathustr/LibQPEP.

## Usage
First go to https://github.com/zarathustr/LibQPEP for the repo of ```LibQPEP```. Then, follow the instructions to install the library:
```
cd LibQPEP
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make install
```

Then, clone the current repo to your catkin workspace and conduct catkin build:
```
cd catkin_ws/src
git clone https://github.com/zarathustr/VINS-Mono-QPEP
catkin build
```

Once finished, use the following command to run the demo from EuroC MAV Dataset:
```
roslaunch vins_estimator_QPEP euroc.launch
```
In another independent terminal, run
```
roslaunch vins_estimator vins_rviz.launch
```
In the terminal window of vins_estimator_QPEP, you will find:
```
QPEP PnP Converged
```
if the QPEP has been successfully executed for initialization of VINS-Mono.
