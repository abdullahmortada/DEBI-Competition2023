To run the simulation, place the package folder in your catkin_ws/src directory and catkin_make it.
Run:

    roslaunch turt_gazebo_test debi_map.launch

This launches the gazebo simulation. Unpause it, then:

    rosrun turt_the_sisyphus movement.py 

This starts the movement node. 
Finally to run the publisher node run: 

    rosrun turt_the_sisyphus pub.py
