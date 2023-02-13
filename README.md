# ORB_SLAM3_Examples_ROS
ORB_SLAM Examples ROS.   

# Build ORB_SLAM3
```
$ git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
$ chmod +x build.sh
$ ./build.sh
```

# Build ORB_SLAM3 ROS
```
$ cd $HOME/ORB_SLAM3/Examples
$ git clone https://github.com/nopnop2002/ORB_SLAM3_Examples_ROS ROS
$ sudo chmod +x ./build_ros.sh
$ ./build_ros.sh
```

# Add package path
```
$ echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$HOME/ORB_SLAM3/Examples/ROS" >> ~/.bashrc
$ source ~/.bashrc
```



