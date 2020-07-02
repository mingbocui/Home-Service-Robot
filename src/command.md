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

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc

cd ~
mkdir catkin_ws/
cd catkin_ws/
mkdir src/
cd src/
catkin_init_workspace
git clone https://github.com/tompoek/robond-proj-7
mv robond-proj-7/* ./
rm -rf robond-proj-7/
cd ..
catkin_make
source devel/setup.bash
chmod +x src/scripts/*

cd ~
rm -rf catkin_ws
mkdir catkin_ws/
cd catkin_ws/
mkdir src/
cd src/
catkin_init_workspace
git clone https://github.com/chk121/HomeServiceRobot.git
mv HomeServiceRobot/src/* ./
rm -rf HomeServiceRobot/
cd ..
catkin_make
source devel/setup.bash
chmod +x src/scripts/*

