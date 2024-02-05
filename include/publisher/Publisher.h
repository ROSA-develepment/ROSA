
#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"

template<typename MessageT>
class Publisher
{
public:
    Publisher(rclcpp::Node* parentNode)
            : _parent(parentNode)
    {

    }

    void publishOn(std::string const& topicName)
    {
        _publisher = _parent->create_publisher<MessageT>(topicName, 10);
    }

    void send(MessageT const& message)
    {
        _publisher->publish(message);
    }

private:
    typename rclcpp::Publisher<MessageT>::SharedPtr _publisher;
    rclcpp::Node* _parent;
};


#endif //ROS_PUBLISHER_H
