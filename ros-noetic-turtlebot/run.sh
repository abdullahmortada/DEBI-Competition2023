xhost local:root 
XAUTH=/tmp/.docker.xauth 

docker run -it \
  --name=turt2 --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  -v /home/abdullah/dev/debi:/root/catkin_ws/src/debi\
  --net=host \
  --privileged ros-noetic-turtlebot bash
