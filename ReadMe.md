## kitti_parser

This is a small helper library that allows for a developer to register callbacks for the different type of measurements in the KITTI dataset.
The program is laid out so that the main user interacts with a "Parser" class which handles all the events. In your main method you just need
to specify the directory of the KITTI dataset, the methods you want it to callback too, and then run it. This is not multithreaded so it will wait
for the method that it calls to finish before it moved on to the next measurement. There are two example main files, please run those to get a feel
for how the program interacts. One is just text events, the other displays the stereo images in an OpenCV window.


## TODO

* Read in config files
* Implement the gps/imu messages
* Add a bit of documentation


## Folder Structure

The folder structure that is expected is that of the kitti datset raw downloader script.
Point to the "day" folder where it has all the different sequences in it. It looks like the following:

* 2011_09_26_drive_0001_sync/
    * image_00/
    * image_01/
    * image_02/
    * image_03/
    * oxts/
    * velodyne_points/
* 2011_09_26_drive_0002_sync/
    * image_00/
    * image_01/
    * image_02/
    * image_03/
    * oxts/
    * velodyne_points/
* 2011_09_26_drive_0005_sync/
    * image_00/
    * image_01/
    * image_02/
    * image_03/
    * oxts/
    * velodyne_points/
* calib_cam_to_cam.txt
* calib_imu_to_velo.txt
* calib_velo_to_cam.txt


## Dependencies

* OpenCV 3.0 - http://opencv.org/
* Boost Library - `sudo apt-get install libboost-all-dev`



