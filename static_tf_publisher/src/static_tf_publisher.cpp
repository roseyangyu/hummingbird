#include <static_tf_publisher.hpp>
#include <Eigen/Dense>


void operator >> (const YAML::Node& node, geometry_msgs::Quaternion& q1) {
    Eigen::Matrix3f rotation; 
    rotation(0,0) = node[0].as<float>();
    rotation(0,1) = node[1].as<float>();
    rotation(0,2) = node[2].as<float>();

    rotation(1,0) = node[3].as<float>();
    rotation(1,1) = node[4].as<float>();
    rotation(1,2) = node[5].as<float>();

    rotation(2,0) = node[6].as<float>();
    rotation(2,1) = node[7].as<float>();
    rotation(2,2) = node[8].as<float>();

    Eigen::Quaternionf q2(rotation);
    q1.x = q2.x();
    q1.y = q2.y();
    q1.z = q2.z();
    q1.w = q2.w();
}

void operator >> (const YAML::Node& node, geometry_msgs::Vector3& v) {
    v.x = node[0].as<float>();
    v.y = node[1].as<float>();
    v.z = node[2].as<float>();
}

void operator >> (const YAML::Node& node, Transformation& transformation) {
    transformation.frame_id = node["frame_id"].as<std::string>();
    transformation.child_frame_id = node["child_frame_id"].as<std::string>();
    node["rotation"] >> transformation.rotation;
    node["translation"] >> transformation.translation;
}

std::vector<Transformation> parse_transformations_yaml(std::string file) {
    YAML::Node config = YAML::LoadFile(file);
    std::vector<Transformation> transformations;
    for (int i = 0; i < config["transformations"].size(); i++) {
        Transformation transform;
        config["transformations"][i] >> transform;
        transformations.push_back(transform);
    }
    return transformations;
}