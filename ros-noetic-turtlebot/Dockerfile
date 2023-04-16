FROM osrf/ros:noetic-desktop-full 

RUN apt update && \
  apt install -y git python3-pip && \
  apt install -y ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo && \
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && \
  git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git && \
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git && \
  git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git


RUN echo "source /ros_entrypoint.sh" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

