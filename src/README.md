sudo apt-get update && apt-get upgrade

sudo apt-get install ros-kinetic-navigation

git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_interactions.git
git clone https://github.com/turtlebot/turtlebot_simulator.git


Dear student,

I tried doing the same procedure but it did not show any error for me. Maybe you can try the following to make it work.

After cloning the turtlebot package

Do catkin_make to make sure the package is build.
Then source the workspace in the terminal.
The run "rosdep update" to let the ros environment know that the package is present in the workspace.
Then run "rosdep -i install turtlebot_teleop" to install the dependencies (if any is missing)
One point to note here is if the catkin_make is successful in the first step them it most probably means that all the package dependencies are met.

run ` pip install rospkg `and then, ` catkin_make`