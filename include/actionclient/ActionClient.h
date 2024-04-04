
#ifndef __ACTIONCLIENT_H__
#define __ACTIONCLIENT_H__

#include "../node/Node.h"

#include <rclcpp_action/rclcpp_action.hpp>


template<typename Service>
class ActionClient
{
public:
    ActionClient(Node* parent, std::string const& service);

    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Service>>>
        asyncSendGoal();

protected:
    rclcpp::Logger getLogger();

    typename Service::Goal _goal;
    typename rclcpp_action::Client<Service>::SendGoalOptions _options;

private:
    typename rclcpp_action::Client<Service>::SharedPtr _client;

    Node* _parent;
};


template<typename Service>
ActionClient<Service>::ActionClient(Node *parent, std::string const& service)
    : _parent(parent)
    , _client(rclcpp_action::create_client<Service>(parent, service))
{

}

template<typename Service>
std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Service>>>
ActionClient<Service>::asyncSendGoal()
{
    _client->wait_for_action_server();

    return _client->async_send_goal(_goal, _options);
}

template<typename Service>
rclcpp::Logger ActionClient<Service>::getLogger()
{
    return _parent->get_logger();
}

#endif //__ACTIONCLIENT_H__
