# analysis the bag file from Didi

## RUN

run the following commands for testing at ROS indigo ubuntu 14.04.

```
mkdir src
cd src
git clone https://github.com/Didi-dataset-explorer
cd ..
# download all dependencies from rosdep
rosdep install --from-paths src --ignore-src -i -y
catkin build
roslaunch round_1 explore_dataset.launch bag_filename:=${BAG_FIEL_PATH.bag}

```

## ISSUES

the connections beween sensor is uncorrect, because the Didi dosenot publish 
this data right now. 

Any questions, please kindly let me known.
