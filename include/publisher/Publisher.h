
#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"


template<typename MessageT>
class Publisher
{
public:
    Publisher(rclcpp::Node* parentNode);

    void publishOn(std::string const& topicName);

    void send(MessageT const& message);

private:
    typename rclcpp::Publisher<MessageT>::SharedPtr _publisher;
    rclcpp::Node* _parent;
};


template<typename MessageT>
Publisher<MessageT>::Publisher(rclcpp::Node* parentNode)
    : _parent(parentNode)
{

}

template<typename MessageT>
void Publisher<MessageT>::publishOn(std::string const& topicName)
{
    _publisher = _parent->create_publisher<MessageT>(topicName, 10);
}

template<typename MessageT>
void Publisher<MessageT>::send(MessageT const& message)
{
    _publisher->publish(message);
}

#endif //ROS_PUBLISHER_H
