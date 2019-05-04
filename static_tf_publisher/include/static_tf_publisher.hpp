#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>


struct Transformation {
    std::string frame_id;
    std::string child_frame_id;
    geometry_msgs::Quaternion rotation;
    geometry_msgs::Vector3 translation;
};

std::vector<Transformation> parse_transformations_yaml(std::string file);

