source devel/setup.bash

ROS_IP=$(hostname -I | awk '{print $1}')

export ROS_IP
echo $ROS_IP

export ROS_MASTER_URI=http://${ROS_IP}:34303
echo ${ROS_MASTER_URI}
