Follow the steps in this link:

  https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup
  to install ros noetic and turtlebot3 dependencies

  https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup
  to install openmanipulator dependencies 

Place the unzipped package in catkin_ws/src directory and catkin_make it.
Two methods to run the simulation:
  1)Premade .launch file:
  just run: 
  
      roslaunch turt_the_sisyphus turt_the_sisyphus.launch

  2) run the separate nodes:
  Run:

      roslaunch turt_gazebo_test debi_map.launch

  This launches the gazebo simulation. Unpause it, then:

      rosrun turt_the_sisyphus movement.py 

  Finally to run the publisher node run: 

      rosrun turt_the_sisyphus pub.py
