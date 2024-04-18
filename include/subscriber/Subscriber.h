
#ifndef __SUBSCRIBER_H__
#define __SUBSCRIBER_H__

#include "rclcpp/rclcpp.hpp"


template<typename MessageT>
class Subscriber
{
public:
    Subscriber(rclcpp::Node* parent);

    void subscribe(std::string const& topicName, std::function<void(typename MessageT::SharedPtr const)> callback);

private:
    typename rclcpp::Subscription<MessageT>::SharedPtr _subscriber;
    rclcpp::Node* _parent;
};


template<typename MessageT>
Subscriber<MessageT>::Subscriber(rclcpp::Node* parent)
    : _parent(parent)
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

#endif //__SUBSCRIBER_H__
