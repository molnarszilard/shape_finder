Shape finder

Using Robotic Operating System and Point Cloud Library this repo is about a shape detection program.
It is capable of subscribing to point cloud topics and find spheres, cylinders and planes (separately large and small area). By providing an IMU, the program further differentiate floor, ceiling. In the demo video we have driven a Pioneer 3-AT robot over a ramp (to simulated the inclination), while the camera (Pico zense) created the depth images about the environment. These depth images were converted to point clouds for the input. At the output the program publishes another point cloud where each point is colored according to the recognised shape (purple - floor, magenta - ceiling, blud - cylinder, red - sphere, green - small plane, yellow - large plane or vertical plane like wall).

A little demonstration can be seen here:

[![DEMO](https://img.youtube.com/vi/rz4ELQGJsRw/0.jpg)](https://youtu.be/rz4ELQGJsRw)

To run the progeam, create a catkin workspace, copy the repository into the src, then run catkin_make, source devel/setup.bash then run:

roslaunch shape_finder find_shapes.launch

The rviz should start. Don't forget to start the camera or the rosbag replay before starting the program.
Make sure that an input point cloud is available, and its topic name is modified in the launch file (ocasionally you could modify the frame d in the cpp file to match the replay as well).
if you don't have an IMU signal, inside src/kalman.py set self.no_imu=True, otherwise self.no_imu=False (and set the topic name for it.)

The main file types are point clouds, RGB images and depth images captured by Pico Zense cameras and ADI Smart Cameras (Courtesy of Analog Devices).
