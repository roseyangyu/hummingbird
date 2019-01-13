rosshutdown

% Gazebo IP address
gazeboIp = '172.16.10.19';
localIp = '172.16.10.68';

% Connection
setenv('ROS_IP', localIp)
setenv('ROS_MASTER_URI',strcat('http://', gazeboIp, ':11311'))
rosinit(gazeboIp, 'NodeHost', localIp)