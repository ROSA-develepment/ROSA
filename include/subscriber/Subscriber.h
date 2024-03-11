
#ifndef ROS_SUBSCRIBER_H
#define ROS_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"


template<typename MessageT>
class Subscriber
{
public:
    Subscriber(rclcpp::Node* parentNode);

    void subscribe(std::string const& topicName, std::function<void(typename MessageT::SharedPtr const)> callback);

private:
    typename rclcpp::Subscription<MessageT>::SharedPtr _subscriber;
    rclcpp::Node* _parent;
};


template<typename MessageT>
Subscriber<MessageT>::Subscriber(rclcpp::Node* parentNode)
    : _parent(parentNode)
{

}

template<typename MessageT>
void Subscriber<MessageT>::subscribe(std::string const& topicName, std::function<void(typename MessageT::SharedPtr const)> callback)
{
    _subscriber = _parent->create_subscription<MessageT>(
        topicName,
        10,
        callback
    );
}

#endif //ROS_SUBSCRIBER_H
