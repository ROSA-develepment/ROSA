
#include "../../include/node/Node.h"

Node::Node(std::string const& name)
    : rclcpp::Node(name)
{

}

rclcpp::Logger Node::getLogger()
{
    return get_logger();
}
