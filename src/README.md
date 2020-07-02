# HomeServiceRobots
sudo apt-get update && apt-get upgrade

sudo apt-get install ros-kinetic-navigation

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


![image](./gif/home_service.gif)