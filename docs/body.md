
    Plugin Choice: Use a package like pca9685_ros2_control. This maps your I2C commands into a C++ hardware interface that ROS 2 understands.

    The URDF Mapping: We will define Cara's skeleton in a .urdf file. For the 20 joints, you'll specify that the "Hardware System" is the PCA9685 driver.
