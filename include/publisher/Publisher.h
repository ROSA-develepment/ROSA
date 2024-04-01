
#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__

#include "rclcpp/rclcpp.hpp"


template<typename MessageT>
class Publisher
{
public:
    Publisher(rclcpp::Node* parentNode, std::string const& topic);

    void send(MessageT const& message);

private:
    typename rclcpp::Publisher<MessageT>::SharedPtr _publisher;
    rclcpp::Node* _parent;
};


template<typename MessageT>
Publisher<MessageT>::Publisher(rclcpp::Node* parentNode, std::string const& topic)
    : _parent(parentNode)
{
    _publisher = _parent->create_publisher<MessageT>(topic, 10);
}

template<typename MessageT>
void Publisher<MessageT>::send(MessageT const& message)
{
    _publisher->publish(message);
}

#endif //__PUBLISHER_H__
