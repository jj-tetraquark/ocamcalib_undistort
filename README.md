# OCamCalib Undistort
ROS node for undistorting and rebroadcasting camera data using [Davide Scaramuzza's OCamCalib model](https://sites.google.com/site/scarabotix/ocamcalib-toolbox).

Currently the node only supports fisheye lenses however there is scope to support omnidirectional lenses in future.

## Build instructions

Tested on ROS Kinetic but should work fine on other versions.

git clone the repository in to your catkin workspace src directory like usual and build using `catkin_make`


## How to use

Once you have calibrated your camera using the MATLAB OCamCalib toolbox and exported the data you should have a file called `calib_results.txt`.
OCamCalib Undistort will use this file to undistort the images from your camera stream. By default it will look for this file in the `include` directory,
however it can be configured in your launchfile to be placed elsewhere.

Start the node using roslaunch like so

```
roslaunch ocamcalib_undistort undistort_node.launch
```

The launchfile can be configured to suit your needs

### Launchfile Params

An example launchfile is provided in the launch directory. A description of the different parameters is as follows:


* camera_type  - The type of camera. Currently only "fisheye" is supported, however hope to support "omni" in future
* base_in_topic - Topic to subscribe to for input camera images. Defaults to `/camera/image`
* base_out_topic - Topic to publish undistorted images on. Defaults to `ocamcalib_undistorted`
* calibration_file_path - Where to find the OCamCalib calibration results file. Defaults to "include/calib_results.txt"
* transport_hint - Transport hint for the input camera stream.

Once your image has been undistorted it will be scaled to be the same size as the input image. However this will result in black sections around the output image.
The following parameters define a bounding box to crop to the region of interest in the undistorted image.

* scale_factor - Scale factor, defaults to 4
* right_bound - Rightmost edge of bounding box
* left_bound - Leftmost edge of bounding box
* bottom_bound - Bottom edge of bounding box
* top_bound - Top edge of bounding box

## References
* Scaramuzza, D., Martinelli, A. and Siegwart, R.. "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", *Proceedings of IEEE International Conference of Vision Systems (ICVS'06)*, New York, January 5-7, 2006. [[PDF](http://rpg.ifi.uzh.ch/docs/ICVS06_scaramuzza.pdf)]
* Scaramuzza, D., Martinelli, A. and Siegwart, R.. "A Toolbox for Easy Calibrating Omnidirectional Cameras", *Proceedings to IEEE International Conference on Intelligent Robots and Systems (IROS 2006)*, Beijing China, October 7-15, 2006. [[PDF](http://rpg.ifi.uzh.ch/docs/IROS06_scaramuzza.pdf)]
