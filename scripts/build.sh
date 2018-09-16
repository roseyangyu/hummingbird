source ~/.bashrc
cd  ~/hummingbird_ws
catkin build
cd ~/PX4-FlightX
make posix_sitl_tailsitter
read -p "Press any key to continue... " -n1 -s
