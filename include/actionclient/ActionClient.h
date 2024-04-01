
#ifndef __ACTIONCLIENT_H__
#define __ACTIONCLIENT_H__

#include "rclcpp_action/rclcpp_action.hpp"
#include "../node/Node.h"


template<typename Service>
class ActionClient
{
public:
    ActionClient(Node* parent, std::string const& service);

protected:
    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Service>>>
        asyncSendGoal(typename Service::Goal goal,
            typename rclcpp_action::Client<Service>::SendGoalOptions options = nullptr);

    typename Service::Goal _goal;
    typename rclcpp_action::Client<Service>::SendGoalOptions _options;

private:
    rclcpp_action::Client<Service> _client;

    Node* _parent;
};


template<typename Service>
ActionClient<Service>::ActionClient(Node *parent, std::string const& service)
    : _parent(parent)
{
    _client = rclcpp_action::create_client<Service>(_parent, service);
}

template<typename Service>
std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Service>>>
ActionClient<Service>::asyncSendGoal(typename Service::Goal goal, typename rclcpp_action::Client<Service>::SendGoalOptions options)
{
    return _client.async_send_goal(goal, options);
}

#endif //__ACTIONCLIENT_H__
