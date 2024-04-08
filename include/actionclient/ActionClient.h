
#ifndef __ACTIONCLIENT_H__
#define __ACTIONCLIENT_H__

#include "../node/Node.h"

#include <rclcpp_action/rclcpp_action.hpp>


template<typename Action>
class ActionClient
{
public:
    ActionClient(Node* parent, std::string const& service);

    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>>>
        asyncSendGoal();

protected:

    virtual void goalResponseCallback(
        typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr const& response) = 0;

    virtual void goalResultCallback(
        typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult const& result) = 0;

    rclcpp::Logger getLogger();

    typename Action::Goal _goal;
    typename rclcpp_action::Client<Action>::SendGoalOptions _options;

private:
    typename rclcpp_action::Client<Action>::SharedPtr _client;

    Node* _parent;
};


template<typename Action>
ActionClient<Action>::ActionClient(Node *parent, std::string const& service)
    : _parent(parent)
    , _client(rclcpp_action::create_client<Action>(parent, service))
{
    _options.goal_response_callback = [this](auto const& response) { goalResponseCallback(response); };
    _options.result_callback = [this](auto const& result) { goalResultCallback(result); };
}

template<typename Action>
std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>>>
ActionClient<Action>::asyncSendGoal()
{
    _client->wait_for_action_server();

    return _client->async_send_goal(_goal, _options);
}

template<typename Action>
rclcpp::Logger ActionClient<Action>::getLogger()
{
    return _parent->get_logger();
}

#endif //__ACTIONCLIENT_H__
