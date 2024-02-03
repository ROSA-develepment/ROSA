
#ifndef ROS_SERVICE_H
#define ROS_SERVICE_H

#include <utility>

#include "rclcpp/rclcpp.hpp"


template<typename ServiceT>
class Service
{
public:
    Service(rclcpp::Node* parent, std::string serviceName)
        : _parent(parent)
    {
        createService(serviceName);
    }

protected:
    virtual void processRequest(std::shared_ptr<typename ServiceT::Request> request,
                        std::shared_ptr<typename ServiceT::Response> response) = 0;

    void createService(std::string const& serviceName)
    {
        auto processRequestPointer =
                [this](std::shared_ptr<typename ServiceT::Request> request,
                       std::shared_ptr<typename ServiceT::Response> response) {
                    this->processRequest(request, response);
                };
        _service = _parent->create_service<ServiceT>(serviceName, processRequestPointer);

        RCLCPP_INFO(_parent->get_logger(), "%s service started", serviceName.c_str());
    }

    rclcpp::Logger getLogger()
    {
        return _parent->get_logger();
    }

private:
    typename rclcpp::Service<ServiceT>::SharedPtr _service;
    rclcpp::Node* _parent;
};


#endif //ROS_SERVICE_H
