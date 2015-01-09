NAOkinator
==========

ROS Module to run a NAO application which interacts with Akinator and performs an interactive speaking guess who game.

Needed resources and packages
------------------------------
Some resources are needed to use this packages:
  1. Main NAO ROS packages from [ros-naoqi](https://github.com/ros-naoqi "github ros-naoqi organization"). With the naoqi_bridge and nao_robot repositories should be enough.
  2. Selenium python library.
  3. nao_smach_utils package from the [NAO-UPC repository](https://github.com/gerardcanal/NAO-UPC "NAO-UPC"). This packages includes some utils to be used in SMACH state machines, and NAOkinator uses them intensively. The launch file in nao_utils may be handy as it launches every node that we need to run the NAOkinator on the NAO.
  4. All the behaviours from the folder NAOMIgestures should be installed previously on the NAO.

We have used and tested the package only in ROS Indigo. Our NAO is a 3.2 NAO, so we don't know how this performs in newer versions of the robot.

Running NAOkinator
------------------
To play the game we have assumed a setting in which the NAO is sitting on a table in front of the player (and in a quite silent room). The program begins making the NAO sit down, so place the robot in a safe position to prevent any possible fall or any other danger or damage to it.
To run the NAOkinator, one should run:
  1. A `roscore` to start ROS.
  2. The nao_basic launch from nao_utils package `roslaunch nao_utils nao_basic.launch`.
  3. The akinator_rossrv node which runs the service that connects with Akinator `rosrun naokinator_ros akinator_rossrv.py`.
  4. The game node `rosrun naokinator_ros start_naokinator.py`.

  
  
