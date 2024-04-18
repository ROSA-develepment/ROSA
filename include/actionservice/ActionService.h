
#ifndef __ACTIONSERVER_H__
#define __ACTIONSERVER_H__

#include "../node/Node.h"

#include "rclcpp_action/rclcpp_action.hpp"


template<typename Service>
class ActionServer
{
public:
    ActionServer(Node* parent, std::string const& service);

protected:
    virtual rclcpp_action::GoalResponse goalCallback(rclcpp_action::GoalUUID const& uuig,
            std::shared_ptr<typename Service::Goal const> const& goal) = 0;

    virtual rclcpp_action::CancelResponse cancelCallback(
            std::shared_ptr<typename rclcpp_action::ServerGoalHandle<Service>> const& goalHandle) = 0;

    virtual void handleAcceptedCallback(
            std::shared_ptr<typename rclcpp_action::ServerGoalHandle<Service>> const& goalHandle) = 0;

    rclcpp::Logger getLogger();

private:
    typename rclcpp_action::Server<Service>::SharedPtr _server;

    Node* _parent;
};



template<typename Service>
ActionServer<Service>::ActionServer(Node* parent, std::string const& service)
    : _parent(parent)
{
    _server = rclcpp_action::create_server<Service>(
        _parent,
        service,
        [this](auto const& uuig, auto const& goal) { return goalCallback(uuig, goal); },
        [this](auto const& goalHandle) { return cancelCallback(goalHandle); },
        [this](auto const& goalHandle) { return handleAcceptedCallback(goalHandle); }
    );
}

template<typename Service>
rclcpp::Logger ActionServer<Service>::getLogger()
{
    return _parent->get_logger();
}


#endif //__ACTIONSERVER_H__
