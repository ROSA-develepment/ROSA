
#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include "rclcpp/rclcpp.hpp"

template<typename ServiceT>
class Client
{
public:
    Client(rclcpp::Node* parentNode, std::string const& serviceName)
            : _parent(parentNode)
    {
        _client = _parent->create_client<ServiceT>(serviceName);
        _request = std::make_shared<typename ServiceT::Request>();
    }

    virtual void startWorkerThread()
    {
        releaseThread();
        if (!_threadWorking)
        {
            _workerThread = std::thread([this]() {
                _threadWorking = true;
                sendRequest();
            });
        }
    }

protected:
    virtual void sendRequest() = 0;

    virtual void asyncSendRequest()
    {
        _response = _client->async_send_request(_request).get();
    }

    template <typename Duration>
    void waitForServer(Duration duration, std::string const& logMessage)
    {
        while (!_client->wait_for_service(duration))
        {
            RCLCPP_WARN(_parent->get_logger(), "%s", logMessage.c_str());
        }
    }

    void releaseThread()
    {
        if (_workerThread.joinable())
        {
            _workerThread.join();
            _threadWorking = false;
        }
    }

    rclcpp::Logger getLogger()
    {
        return _parent->get_logger();
    }

    std::shared_ptr<typename ServiceT::Request> _request;
    std::shared_ptr<typename ServiceT::Response> _response;

private:
    typename rclcpp::Client<ServiceT>::SharedPtr _client;

    rclcpp::Node* _parent;
    std::thread _workerThread;

    bool _threadWorking = false;
};

#endif //ROS_CLIENT_H
