
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "rclcpp/rclcpp.hpp"


class Node : public rclcpp::Node
{
public:
    Node(std::string const& name) : rclcpp::Node(name) {}
};

#endif //ROS_NODE_H
