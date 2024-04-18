
#ifndef __NODE_H__
#define __NODE_H__

#include "rclcpp/rclcpp.hpp"


class Node : public rclcpp::Node
{
public:
    Node(std::string const& name);

protected:

    rclcpp::Logger getLogger();
};

#endif //__NODE_H__
